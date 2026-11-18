///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "InternalBridgeAnchors.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_post_infill.h"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionTreeVisitors.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/ParallelFor.hpp"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"
#include "libslic3r/Api/plugin/cpp/properties/BridgeProperties.hpp"
#include "libslic3r/Plugins/Surface/BridgePreparation.hpp"

/* Extend the two ends of requested internal bridge threads after infill has
   produced the real extrusion trees. A footprint is a rectangle around one
   straight segment with square ends; its hook rectangle is the same rectangle
   narrowed across the segment. The first hook reached on this layer wins. If
   none is reached, a lower-layer axis may be crossed and passed by one bridge
   nozzle diameter. The dedicated GAP_FILLS bucket is omitted on both layers.

   The pass reads all original trees before it edits any tree. It records one
   result per changed bridge leaf, then applies those results in a second pass.
   Ends already changed in a region-island traversal become footprints for later
   threads. The island contour, including holes, bounds every proposed point.
   Only a requested internal bridge with a straight, open leaf whose original
   segments all follow one prepared direction is eligible; arcs, returns, bends
   and inherited nonzero Z offsets also make the leaf ineligible. The infill root is regrouped so requested bridge
   children are printed last without changing their attributes or geometry.

       run_impl -> anchor_layer -> anchor_island
           |-- collect current-layer rectangles and bridge threads
           |-- collect lower-layer axes and the island outline
           `-- reach each eligible end and record rounded points
       run_impl -> apply_layer
           |-- apply every recorded point
           `-- regroup each affected infill root

   All views are borrowed only during the synchronous plugin call. Storage-owned
   groups are moved into their host roots; exceptions and cancellation propagate
   through PluginBase's C ABI boundary. */

namespace slic3r_api { namespace Infill { namespace InternalBridgeAnchorsPlugin {
namespace {

const char *k_dependencies[] = { "infill.recipe.bridge_preparation", nullptr };
const raw_used_config_key k_used_keys[] = {
    {"internal_bridge_expansion", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"infill_overlap", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"nozzle_diameter", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE}
};
// Limit the first-layer search to this many bridge-tool nozzle diameters.
constexpr double k_search = 25.;
// Enter ordinary infill, gap-fill and bridge footprints by one quarter width.
constexpr double k_entry = .25;
// A missing geometric distance is represented by positive infinity.
constexpr double k_none = std::numeric_limits<double>::infinity();

// A point or direction in the scaled XY coordinate system.
struct Vec { double x, y; };
// Add two scaled XY vectors component by component.
Vec operator+(Vec a, Vec b) { return {a.x + b.x, a.y + b.y}; }
// Subtract two scaled XY vectors component by component.
Vec operator-(Vec a, Vec b) { return {a.x - b.x, a.y - b.y}; }
// Scale a scaled XY vector by a finite scalar.
Vec operator*(Vec a, double k) { return {a.x * k, a.y * k}; }
// Return the scalar product of two scaled XY vectors.
double dot(Vec a, Vec b) { return a.x * b.x + a.y * b.y; }
// Return the signed two-dimensional cross product of two scaled XY vectors.
double cross(Vec a, Vec b) { return a.x * b.y - a.y * b.x; }
// Copy a C API point into the local double vector representation.
Vec vec(c_point p) { return {double(p.x), double(p.y)}; }
// Round a finite local vector back to the scaled integer grid.
c_point point(Vec v) { return {coord_t(std::llround(v.x)), coord_t(std::llround(v.y))}; }
// Test equality of two local scaled points without tolerance or allocation.
bool same_point(Vec a, Vec b) { return a.x == b.x && a.y == b.y; }

// A current-layer segment footprint and its narrower hook rectangle.
struct Footprint
{
    Vec a, b;
    double outer, hook;
    int64_t owner;
    struct Box { double min_x, min_y, max_x, max_y; } bounds;
};
// A lower-layer axis or island-outline edge and its axis-aligned bounds.
struct Edge
{
    Vec a, b;
    Footprint::Box bounds;
};
// A straight, open bridge leaf and the prepared unit direction of its path.
struct Thread
{
    uint32_t region, ordinal;
    Vec front, back, dir;
    double width, nozzle;
};
// Current footprints, lower axes, island outline and eligible bridge leaves.
struct Scene
{
    std::vector<Footprint> footprints;
    std::vector<Edge> lower, outline;
    std::vector<Thread> threads;
};
// Rounded endpoints of one changed bridge leaf in its infill traversal order.
struct Ends { uint32_t ordinal; c_point front, back; };
// Changed leaves belonging to one mutable region-island infill root.
struct RegionEnds { const layer_region_island_handle *region; std::vector<Ends> ends; };
using LayerEnds = std::vector<RegionEnds>;
// Prepared direction and bridge-tool nozzle for one extension-requested surface.
struct Bridge { double angle, nozzle; };
using Bridges = std::unordered_map<uint64_t, Bridge>;
using RowsKey = PluginPropertyKey<BridgePreparation::PreparedRows>;

// Return whether a role deposits material that can anchor or block a thread.
bool occupies(raw_extrusion_role role);
// Collect extension-requested internal bridges indexed by source surface id.
Bridges extended_bridges(const LayerRegionIsland &region, const RowsKey &rows_key);
// Return the first scaled ray parameter entering a square-ended segment rectangle, or positive infinity.
double rectangle_entry(Vec origin, Vec direction, Vec a, Vec b, double half_width);
// Return a signed scaled ray parameter at which a unit-direction ray crosses a finite segment, or positive infinity.
double crossing(Vec origin, Vec direction, Vec a, Vec b);
// Test one original straight segment against the prepared angle and return its signed projection.
bool follows_direction(const c_extrusion_segment &segment, double angle, double *projection);
// Accept only a non-loop, non-arc leaf whose original segments advance in one direction.
bool straight_leaf(const ExtrusionEntity &leaf, double angle, Vec &direction);
// Apply the current-layer, lower-axis and outline rules to one endpoint and return a positive scaled distance, or zero.
double reach(const Scene &scene, const ExPolygon &slice, int64_t owner, const Thread &thread,
    Vec origin, Vec direction);
// Compute changed endpoints for all eligible threads in one geometric island.
void anchor_island(const LayerIsland &island, const RowsKey &rows_key, LayerEnds &out,
    const plugin_run_context *run_ctx);
// Run the read-only anchor calculation for one object layer.
void anchor_layer(uint32_t idx, storage_handle *, const run_ctx_post_infill_generation *ctx,
    const RowsKey *rows_key, std::vector<LayerEnds> *ends, const plugin_run_context *run_ctx);
// Move requested bridge children into a final sortable priority group.
void print_bridges_last(storage_handle *storage, MutableExtrusionEntity root, const Bridges &bridges);
// Apply one layer's recorded endpoints and order its affected infill roots.
void apply_layer(uint32_t idx, storage_handle *storage, const run_ctx_post_infill_generation *ctx,
    const RowsKey *rows_key, const std::vector<LayerEnds> *ends, const plugin_run_context *run_ctx);

// Collect current-layer footprints and eligible bridge leaves from one bucket.
class FootprintCollector : public ExtrusionTreeConstVisitor<>
{
public:
    // Copy the region configuration and borrow scene storage and bridge metadata for one traversal.
    FootprintCollector(Scene &scene, const Config &config, double perimeter_width_mm, uint32_t region,
        const Bridges *bridges) : m_scene(scene), m_config(config), m_perimeter_width_mm(perimeter_width_mm),
        m_region(region), m_bridges(bridges) {}

private:
    // Append linearized occupied segments and record only eligible internal bridge leaves.
    void visit_leaf(ExtrusionEntity leaf) override
    {
        const uint32_t ordinal = m_ordinal++;
        const EPropertyAttributes *attr = current_property(EPropertyAttributes::key);
        if (attr == nullptr || leaf.point_count() < 2 || !occupies(attr->extrusion_role()))
            return;
        const double width = scale_d(attr->c_extrusion_property_attributes::width);
        if (!(width > 0.) || !std::isfinite(width))
            return;
        std::vector<c_point> points = leaf.linear_points();
        if (leaf.is_loop() && !points.empty() && points.front() != points.back())
            points.push_back(points.front());
        if (points.size() < 2)
            return;
        const raw_extrusion_role role = attr->extrusion_role();
        int64_t owner = -1;
        if (m_bridges != nullptr && role == RAW_EXTRUSION_ROLE_INTERNAL_BRIDGE_INFILL) {
            const EPropertyInfill *infill = current_property(EPropertyInfill::key);
            const Bridges::const_iterator bridge = infill == nullptr ? m_bridges->end() :
                m_bridges->find(infill->surface_id());
            const EPropertyZOffset *inherited_offset = current_property(EPropertyZOffset::key);
            if (bridge != m_bridges->end() && !leaf.has_z_offsets() &&
                (inherited_offset == nullptr || inherited_offset->z_offset == 0)) {
                Vec direction{};
                if (straight_leaf(leaf, bridge->second.angle, direction)) {
                    const Vec front = vec(points.front()), back = vec(points.back());
                    owner = int64_t(m_scene.threads.size());
                    m_scene.threads.push_back({m_region, ordinal, front, back, direction, width,
                        bridge->second.nozzle});
                }
            }
        }
        const bool perimeter = (role & RAW_EXTRUSION_ROLE_PERIMETER) != 0;
        for (size_t idx = 1; idx < points.size(); ++idx) {
            const Vec a = vec(points[idx - 1]), b = vec(points[idx]);
            if (same_point(a, b))
                continue;
            double entry = k_entry * width;
            if (perimeter && m_config.valid() && m_config.has("infill_overlap"))
                entry = scale_d(m_config.get("infill_overlap").get_effective_value(m_perimeter_width_mm));
            entry = std::max(0., std::min(entry, .5 * width));
            m_scene.footprints.push_back({a, b, .5 * width, .5 * width - entry, owner,
                {std::min(a.x, b.x) - .5 * width, std::min(a.y, b.y) - .5 * width,
                 std::max(a.x, b.x) + .5 * width, std::max(a.y, b.y) + .5 * width}});
        }
    }

    Scene &m_scene;
    Config m_config;
    double m_perimeter_width_mm;
    uint32_t m_region;
    const Bridges *m_bridges;
    uint32_t m_ordinal = 0;
};

// Collect occupied segment axes from a lower-layer extrusion bucket.
class AxisCollector : public ExtrusionTreeConstVisitor<>
{
public:
    // Borrow the destination vector for one lower-layer traversal.
    explicit AxisCollector(std::vector<Edge> &axes) : m_axes(axes) {}

private:
    // Append each linearized segment of an occupied lower extrusion.
    void visit_leaf(ExtrusionEntity leaf) override
    {
        const EPropertyAttributes *attr = current_property(EPropertyAttributes::key);
        if (attr == nullptr || leaf.point_count() < 2 || !occupies(attr->extrusion_role()))
            return;
        const std::vector<c_point> points = leaf.linear_points();
        for (size_t idx = 1; idx < points.size(); ++idx) {
            const Vec a = vec(points[idx - 1]), b = vec(points[idx]);
            if (!same_point(a, b))
                m_axes.push_back({a, b, {std::min(a.x, b.x), std::min(a.y, b.y),
                    std::max(a.x, b.x), std::max(a.y, b.y)}});
        }
    }

    std::vector<Edge> &m_axes;
};

// Apply endpoint replacements to leaves identified by their original ordinal.
class EndsVisitor : public ExtrusionTreeVisitor<>
{
public:
    // Borrow the ordered endpoint list until traversal completes.
    explicit EndsVisitor(const std::vector<Ends> &ends) : m_ends(ends) {}
    // Report whether every recorded ordinal was found and changed.
    bool complete() const { return m_next == m_ends.size(); }

private:
    // Replace both endpoints of the next matching leaf and reject malformed trees.
    void visit_leaf(MutableExtrusionEntity leaf) override
    {
        const uint32_t ordinal = m_ordinal++;
        if (m_next == m_ends.size() || m_ends[m_next].ordinal != ordinal)
            return;
        const Ends &ends = m_ends[m_next++];
        const uint32_t count = leaf.point_count();
        if (count < 2 || !leaf.set_point(0, ends.front) || !leaf.set_point(count - 1, ends.back))
            throw std::runtime_error("Cannot move the ends of an internal bridge thread");
    }

    const std::vector<Ends> &m_ends;
    size_t m_next = 0;
    uint32_t m_ordinal = 0;
};

// Return true for deposit roles except ironing and non-depositing movement roles.
bool occupies(raw_extrusion_role role)
{
    return (role & (RAW_EXTRUSION_ROLE_PERIMETER | RAW_EXTRUSION_ROLE_INFILL | RAW_EXTRUSION_ROLE_THIN)) != 0 &&
        (role & (RAW_EXTRUSION_ROLE_IRONING | RAW_EXTRUSION_ROLE_TRAVEL | RAW_EXTRUSION_ROLE_WIPE |
            RAW_EXTRUSION_ROLE_RETRACT | RAW_EXTRUSION_ROLE_UNRETRACT)) == 0;
}

// Read prepared directions and bridge-tool nozzles for extension-requested surfaces.
Bridges extended_bridges(const LayerRegionIsland &region, const RowsKey &rows_key)
{
    Bridges bridges;
    if (region.region_count() == 0)
        return bridges;
    for (const Surface surface : region.fill_surfaces_collection()) {
        const BridgeSurfaceInfo *info = surface.get(BridgeSurfaceInfo::key);
        if (info == nullptr || info->schema_version != 1 || info->core_rule != 2 || !info->extension_requested)
            continue;
        const BridgePreparation::PreparedRows *rows = rows_key.get(surface);
        const double angle = rows != nullptr ? rows->angle : surface.bridge_angle();
        const double nozzle = scale_d(region.region(0).infill_bridge_flow_mm(
            RAW_EXTRUSION_ROLE_SOLID_INFILL).nozzle_diameter);
        if (std::isfinite(angle) && angle >= 0. && std::isfinite(nozzle) && nozzle > 0.)
            bridges.emplace(surface.id(), Bridge{angle, nozzle});
    }
    return bridges;
}

// Clip a ray against the local rectangle [0,length] x [-half_width,half_width].
double rectangle_entry(Vec origin, Vec direction, Vec a, Vec b, double half_width)
{
    const Vec axis = b - a;
    const double length = std::hypot(axis.x, axis.y);
    if (!(length > 0.) || !(half_width >= 0.))
        return k_none;
    const Vec along = axis * (1. / length), across{-along.y, along.x};
    const Vec offset = origin - a;
    double low = 0., high = k_none;
    const auto clip = [&low, &high](double position, double speed, double minimum, double maximum) {
        if (std::abs(speed) <= 1e-14) {
            if (position < minimum || position > maximum)
                high = -1.;
            return;
        }
        const double first = (minimum - position) / speed;
        const double second = (maximum - position) / speed;
        low = std::max(low, std::min(first, second));
        high = std::min(high, std::max(first, second));
    };
    clip(dot(offset, along), dot(direction, along), 0., length);
    clip(dot(offset, across), dot(direction, across), -half_width, half_width);
    return low <= high ? low : k_none;
}

// Intersect a ray with a finite segment using a length-relative parallel test; direction must be unit-length,
// the returned signed parameter is in scaled XY units, and positive infinity means no finite intersection.
double crossing(Vec origin, Vec direction, Vec a, Vec b)
{
    const Vec side = b - a, offset = a - origin;
    const double side_length = std::hypot(side.x, side.y);
    const double denominator = cross(direction, side);
    if (!(side_length > 0.) || std::abs(denominator) <= 1e-12 * side_length)
        return k_none;
    const double along = cross(offset, direction) / denominator;
    return along < 0. || along > 1. ? k_none : cross(offset, side) / denominator;
}

// Compare one original straight segment with the prepared unit direction; projection receives its signed sense.
// A non-null segment has scaled endpoints, and the returned projection is dimensionless and nonzero on success.
bool follows_direction(const c_extrusion_segment &segment, double angle, double *projection)
{
    if (projection != nullptr)
        *projection = 0.;
    if (segment.radius != 0. || !std::isfinite(angle))
        return false;
    const double dx = double(segment.point_b.x) - double(segment.point_a.x);
    const double dy = double(segment.point_b.y) - double(segment.point_a.y);
    const double scaled_length = std::hypot(dx, dy);
    if (!(scaled_length > 0.))
        return false;
    const double length = unscaled(scaled_length);
    const double ux = dx / scaled_length, uy = dy / scaled_length;
    const double direction_roundoff = unscaled(std::abs(std::sin(angle)) + std::abs(std::cos(angle))) / length;
    if (std::abs(ux * std::sin(angle) - uy * std::cos(angle)) > 1e-6 + direction_roundoff)
        return false;
    if (projection != nullptr)
        *projection = ux * std::cos(angle) + uy * std::sin(angle);
    return true;
}

// Accept an open leaf only when every original segment is straight, parallel to the prepared axis and oriented alike;
// direction is returned with that common sense, while malformed leaves leave the output direction unused.
bool straight_leaf(const ExtrusionEntity &leaf, double angle, Vec &direction)
{
    if (!leaf.has_polyline() || leaf.point_count() < 2 || leaf.segment_count() == 0 || leaf.is_loop() ||
        leaf.local_front() == leaf.local_back())
        return false;
    direction = {std::cos(angle), std::sin(angle)};
    int sense = 0;
    for (uint32_t idx = 0; idx < leaf.segment_count(); ++idx) {
        double projection = 0.;
        if (!follows_direction(leaf.segment(idx), angle, &projection) || projection == 0.)
            return false;
        const int current_sense = projection > 0. ? 1 : -1;
        if (sense == 0)
            sense = current_sense;
        else if (sense != current_sense)
            return false;
    }
    if (sense < 0)
        direction = direction * -1.;
    return true;
}

// Apply the three anchoring rules to one endpoint while requiring a finite target and in-island result.
double reach(const Scene &scene, const ExPolygon &slice, int64_t owner, const Thread &thread,
    Vec origin, Vec direction)
{
    const double limit = k_search * thread.nozzle;
    const Footprint::Box ray = {std::min(origin.x - direction.x * thread.nozzle,
        origin.x + direction.x * (limit + thread.nozzle)),
        std::min(origin.y - direction.y * thread.nozzle,
        origin.y + direction.y * (limit + thread.nozzle)),
        std::max(origin.x - direction.x * thread.nozzle,
        origin.x + direction.x * (limit + thread.nozzle)),
        std::max(origin.y - direction.y * thread.nozzle,
        origin.y + direction.y * (limit + thread.nozzle))};
    double exit = k_none;
    for (const Edge &edge : scene.outline)
        if (edge.bounds.min_x <= ray.max_x && ray.min_x <= edge.bounds.max_x &&
            edge.bounds.min_y <= ray.max_y && ray.min_y <= edge.bounds.max_y) {
            const double t = crossing(origin, direction, edge.a, edge.b);
            if (t >= 0.)
                exit = std::min(exit, t);
        }
    double hook = k_none, blocked = k_none;
    for (const Footprint &footprint : scene.footprints)
        if (footprint.owner != owner && footprint.bounds.min_x <= ray.max_x && ray.min_x <= footprint.bounds.max_x &&
            footprint.bounds.min_y <= ray.max_y && ray.min_y <= footprint.bounds.max_y) {
            hook = std::min(hook, rectangle_entry(origin, direction, footprint.a, footprint.b, footprint.hook));
            blocked = std::min(blocked, rectangle_entry(origin, direction, footprint.a, footprint.b, footprint.outer));
        }
    // A finite current-layer hook wins, including a hook already touching the endpoint at t=0.
    if (std::isfinite(hook) && hook <= std::min(limit, exit)) {
        return slice.contains(point(origin + direction * hook)) ? hook : 0.;
    }
    double lower = k_none;
    for (const Edge &edge : scene.lower)
        if (edge.bounds.min_x <= ray.max_x && ray.min_x <= edge.bounds.max_x &&
            edge.bounds.min_y <= ray.max_y && ray.min_y <= edge.bounds.max_y) {
            const double t = crossing(origin, direction, edge.a, edge.b);
            if (t > -thread.nozzle && t <= limit)
                lower = std::min(lower, t);
        }
    if (!std::isfinite(lower))
        return 0.;
    const double target = lower + thread.nozzle;
    return target <= std::min(blocked, exit) && slice.contains(point(origin + direction * target)) ? target : 0.;
}

// Collect and extend eligible threads in traversal order, then quantize endpoints and store borrowed-root edits in out.
// The scene is local to this call; final extension footprints remain only to block later threads in the same island.
void anchor_island(const LayerIsland &island, const RowsKey &rows_key, LayerEnds &out,
    const plugin_run_context *run_ctx)
{
    std::vector<Bridges> bridges(island.region_island_count());
    bool any = false;
    for (uint32_t region_idx = 0; region_idx < island.region_island_count(); ++region_idx) {
        bridges[region_idx] = extended_bridges(island.region_island(region_idx), rows_key);
        any |= !bridges[region_idx].empty();
    }
    if (!any)
        return;
    Scene scene;
    for (uint32_t region_idx = 0; region_idx < island.region_island_count(); ++region_idx) {
        throw_if_cancelled(run_ctx);
        const LayerRegionIsland region = island.region_island(region_idx);
        Config config;
        double perimeter_width_mm = 0.;
        if (region.region_count() > 0) {
            config = region.region(0).print_region().config();
            perimeter_width_mm = unscaled(region.region(0).flow(RAW_EXTRUSION_ROLE_INTERNAL_PERIMETER).width);
        }
        for (uint32_t bucket = 0; bucket < layer_region_island_extrusion_count(region.handle()); ++bucket) {
            const raw_extrusion_role role = layer_region_island_extrusion_role_at(region.handle(), bucket);
            if (role == RAW_EXTRUSION_ROLE_GAP_FILL)
                continue;
            const extrusion_entity_handle *root = region.extrusion(role);
            if (root == nullptr)
                continue;
            FootprintCollector collector(scene, config, perimeter_width_mm, region_idx,
                role == RAW_EXTRUSION_ROLE_INTERNAL_INFILL && !bridges[region_idx].empty() ?
                    &bridges[region_idx] : nullptr);
            collector.traverse(ExtrusionEntity(root));
        }
    }
    if (scene.threads.empty())
        return;
    for (const LayerIsland lower : island.lower_islands()) {
        for (uint32_t region_idx = 0; region_idx < lower.region_island_count(); ++region_idx) {
            throw_if_cancelled(run_ctx);
            const LayerRegionIsland region = lower.region_island(region_idx);
            for (uint32_t bucket = 0; bucket < layer_region_island_extrusion_count(region.handle()); ++bucket) {
                const raw_extrusion_role role = layer_region_island_extrusion_role_at(region.handle(), bucket);
                if (role == RAW_EXTRUSION_ROLE_GAP_FILL)
                    continue;
                const extrusion_entity_handle *root = region.extrusion(role);
                if (root == nullptr)
                    continue;
                AxisCollector collector(scene.lower);
                collector.traverse(ExtrusionEntity(root));
            }
        }
    }
    const ExPolygon slice = island.slice();
    const auto add_outline = [&scene](const Polygon &polygon) {
        const std::vector<c_point> points = polygon.points();
        for (size_t idx = 0; idx < points.size(); ++idx) {
            const Vec a = vec(points[idx]), b = vec(points[(idx + 1) % points.size()]);
            if (!same_point(a, b))
                scene.outline.push_back({a, b, {std::min(a.x, b.x), std::min(a.y, b.y),
                    std::max(a.x, b.x), std::max(a.y, b.y)}});
        }
    };
    add_outline(slice.contour());
    for (const Polygon hole : slice.holes())
        add_outline(hole);

    // Process regions and leaves in their original order; accepted extensions become later footprints.
    for (size_t thread_idx = 0; thread_idx < scene.threads.size(); ++thread_idx) {
        if ((thread_idx & 63u) == 0)
            throw_if_cancelled(run_ctx);
        const Thread &thread = scene.threads[thread_idx];
        const Vec original[2] = {thread.front, thread.back};
        Vec candidates[2] = {original[0], original[1]};
        // Preserve the unrounded axial targets; final endpoints are quantized from an integer support, with a second
        // reprojection allowed when rejecting one end changes which original endpoint supplies that support.
        double positions[2] = {dot(original[0], thread.dir), dot(original[1], thread.dir)};
        bool changed[2] = {false, false};
        for (int side = 0; side < 2; ++side) {
            const Vec direction = side == 0 ? thread.dir * -1. : thread.dir;
            const double distance = reach(scene, slice, int64_t(thread_idx), thread, original[side], direction);
            if (!(distance > 0.))
                continue;
            positions[side] += side == 0 ? -distance : distance;
            const c_point candidate = point(original[side] + direction * distance);
            candidates[side] = vec(candidate);
            changed[side] = !same_point(candidates[side], original[side]) && slice.contains(candidate);
        }
        if (!changed[0] && !changed[1]) {
            continue;
        }
        // Reproject from a fixed integer support point twice at most: a first rejected end can change the support
        // to the other original end, so the second attempt preserves that remaining end's quantized position.
        for (int attempt = 0; attempt < 2; ++attempt) {
            const bool old_changed[2] = {changed[0], changed[1]};
            const Vec support = changed[0] ? (changed[1] ? original[0] : original[1]) : original[0];
            for (int side = 0; side < 2; ++side) {
                if (!changed[side]) {
                    candidates[side] = original[side];
                    continue;
                }
                const double support_position = dot(support, thread.dir);
                const c_point rounded = point(support + thread.dir * (positions[side] - support_position));
                candidates[side] = vec(rounded);
                if (!slice.contains(rounded))
                    changed[side] = false;
            }
            if (!changed[0] && !changed[1])
                break;
            if (old_changed[0] == changed[0] && old_changed[1] == changed[1])
                break;
        }
        if (!changed[0] && !changed[1])
            continue;
        for (int side = 0; side < 2; ++side)
            if (changed[side] && same_point(candidates[side], original[side]))
                changed[side] = false;
        if (!changed[0] && !changed[1])
            continue;
        for (int side = 0; side < 2; ++side)
            if (changed[side])
                scene.footprints.push_back({original[side], candidates[side], .5 * thread.width,
                    .25 * thread.width, int64_t(thread_idx),
                    {std::min(original[side].x, candidates[side].x) - .5 * thread.width,
                     std::min(original[side].y, candidates[side].y) - .5 * thread.width,
                     std::max(original[side].x, candidates[side].x) + .5 * thread.width,
                     std::max(original[side].y, candidates[side].y) + .5 * thread.width}});
        const c_point front = point(candidates[0]), back = point(candidates[1]);
        if (front == point(thread.front) && back == point(thread.back))
            continue;
        const layer_region_island_handle *handle = island.region_island(thread.region).handle();
        if (out.empty() || out.back().region != handle)
            out.push_back({handle, {}});
        out.back().ends.push_back({thread.ordinal, front, back});
    }
}

// Read one layer's islands without mutating their extrusion trees.
void anchor_layer(uint32_t idx, storage_handle *, const run_ctx_post_infill_generation *ctx,
    const RowsKey *rows_key, std::vector<LayerEnds> *ends, const plugin_run_context *run_ctx)
{
    const Layer layer = Object(ctx->object).layer(idx);
    for (uint32_t island_idx = 0; island_idx < layer.island_count(); ++island_idx) {
        throw_if_cancelled(run_ctx);
        anchor_island(layer.island(island_idx), *rows_key, (*ends)[idx], run_ctx);
    }
}

// Regroup direct infill children so bridge groups are sortable, reversible and rotatable while the root becomes fixed.
void print_bridges_last(storage_handle *storage, MutableExtrusionEntity root, const Bridges &bridges)
{
    std::vector<bool> bridge_child(root.child_count(), false);
    bool has_bridge = false, has_other = false;
    for (uint32_t idx = 0; idx < root.child_count(); ++idx) {
        const EPropertyInfill *infill = root.child(idx).get(EPropertyInfill::key);
        bridge_child[idx] = infill != nullptr && bridges.count(infill->surface_id()) != 0;
        (bridge_child[idx] ? has_bridge : has_other) = true;
    }
    if (!has_bridge)
        return;
    const uint32_t flags = root.flags();
    StoredExtrusionEntity other(storage), bridge(storage);
    const bool root_was_sortable = (flags & RAW_EXTRUSION_FLAG_SORTABLE) != 0;
    // A sortable root is rebuilt into ordinary-then-bridge groups; a fixed root keeps its other children in place.
    if (root_was_sortable) {
        for (uint32_t idx = root.child_count(); idx-- > 0;) {
            StoredExtrusionEntity &destination = bridge_child[idx] ? bridge : other;
            if (is_invalid_index(destination.move_child_from(0, root, idx)))
                throw std::runtime_error("Cannot regroup an infill bucket around its internal bridges");
        }
    } else {
        for (uint32_t idx = root.child_count(); idx-- > 0;)
            if (bridge_child[idx] && is_invalid_index(bridge.move_child_from(0, root, idx)))
                throw std::runtime_error("Cannot remove internal bridges from their fixed infill root");
    }
    // A fixed root keeps its ordinary children directly in place; do not add
    // an empty wrapper for them. A sortable root needs one group to preserve
    // the old children before the bridge group is appended.
    if (root_was_sortable && has_other) {
        const uint32_t group_flags = flags | RAW_EXTRUSION_FLAG_SORTABLE | RAW_EXTRUSION_FLAG_REVERSIBLE;
        if (!other.set_flags(group_flags))
            throw std::runtime_error("Cannot configure the ordinary infill priority group");
        if (!other.set_can_rotate(true) || is_invalid_index(root.append_child_move(other.mutable_view())))
            throw std::runtime_error("Cannot append the ordinary infill priority group");
    }
    const uint32_t group_flags = flags | RAW_EXTRUSION_FLAG_SORTABLE | RAW_EXTRUSION_FLAG_REVERSIBLE;
    if (!bridge.set_flags(group_flags))
        throw std::runtime_error("Cannot configure the bridge infill priority group");
    if (!bridge.set_can_rotate(true) || is_invalid_index(root.append_child_move(bridge.mutable_view())))
        throw std::runtime_error("Cannot append the bridge infill priority group");
    // Both groups are sortable/reversible and may rotate; the root is fixed,
    // non-reversible and non-rotatable so ordering cannot move the bridge group
    // back ahead of the ordinary group.
    root.disable_sort().disable_reverse();
    if (!root.set_can_rotate(false))
        throw std::runtime_error("Cannot fix the infill priority root");
}

// Apply saved endpoints only to an existing internal-infill root, reuse one mutable handle for edits and ordering,
// and throw when a recorded region or leaf is missing; regions without that bucket are never created implicitly.
void apply_layer(uint32_t idx, storage_handle *storage, const run_ctx_post_infill_generation *ctx,
    const RowsKey *rows_key, const std::vector<LayerEnds> *ends, const plugin_run_context *run_ctx)
{
    const Layer layer = Object(ctx->object).layer(idx);
    const LayerEnds &moved = (*ends)[idx];
    std::vector<bool> applied(moved.size(), false);
    for (uint32_t island_idx = 0; island_idx < layer.island_count(); ++island_idx) {
        const LayerIsland island = layer.island(island_idx);
        for (uint32_t region_idx = 0; region_idx < island.region_island_count(); ++region_idx) {
            throw_if_cancelled(run_ctx);
            const LayerRegionIsland region = island.region_island(region_idx);
            const Bridges bridges = extended_bridges(region, *rows_key);
            if (bridges.empty())
                continue;
            const LayerEnds::const_iterator found = std::find_if(moved.begin(), moved.end(),
                [&region](const RegionEnds &entry) { return entry.region == region.handle(); });
            const extrusion_entity_handle *existing = region.extrusion(RAW_EXTRUSION_ROLE_INTERNAL_INFILL);
            if (existing == nullptr) {
                if (found != moved.end())
                    throw std::runtime_error("Cannot obtain an internal infill root for recorded bridge ends");
                continue;
            }
            extrusion_entity_handle *handle = ctx->get_region_island_mutable_extrusion(
                region.handle(), RAW_EXTRUSION_ROLE_INTERNAL_INFILL);
            if (handle == nullptr)
                throw std::runtime_error("Cannot obtain an internal infill root for bridge ordering");
            if (found != moved.end()) {
                const size_t found_idx = size_t(std::distance(moved.begin(), found));
                applied[found_idx] = true;
                EndsVisitor visitor(found->ends);
                visitor.traverse(MutableExtrusionEntity(handle));
                if (!visitor.complete())
                    throw std::runtime_error("Not every recorded internal bridge end was applied");
            }
            print_bridges_last(storage, MutableExtrusionEntity(handle), bridges);
        }
    }
    if (std::find(applied.begin(), applied.end(), false) != applied.end())
        throw std::runtime_error("Not every recorded internal bridge region was applied");
}

} // namespace

// Return the process-local plugin instance attached to the requested orchestrator.
InternalBridgeAnchors &InternalBridgeAnchors::instance(orchestrator_handle *orch)
{
    static InternalBridgeAnchors instance(orch);
    return instance;
}

// Return the stable plugin identifier used by activation and dependencies.
const char *InternalBridgeAnchors::id_impl() const noexcept { return "infill.post_process.internal_bridge_anchors"; }
// Return the short plugin name shown in diagnostics.
const char *InternalBridgeAnchors::name_impl() const noexcept { return "Internal bridge anchors"; }
// Describe endpoint anchoring and final infill ordering without performing work.
const char *InternalBridgeAnchors::description_impl() const noexcept
{ return "Extend internal bridge threads to nearby extrusions and print them after the other infill."; }
// Select the post-infill generation step where all target trees exist.
slicing_step_t InternalBridgeAnchors::step_impl() const noexcept { return STEP_POST_INFILL; }
// Return the bridge recipe plugin required to publish bridge metadata and rows.
const char *const *InternalBridgeAnchors::dependencies_impl() const noexcept { return k_dependencies; }
// Run after gap fill, dense infill ordering and full-fill volume correction.
int32_t InternalBridgeAnchors::priority_impl() const noexcept { return 20; }

// Copy the keys associated with this pass into the caller's preallocated array.
int32_t InternalBridgeAnchors::used_config_keys(raw_used_config_key *keys) const noexcept
{
    if (keys != nullptr)
        std::copy(std::begin(k_used_keys), std::end(k_used_keys), keys);
    return int32_t(std::size(k_used_keys));
}

// Read all layers before applying endpoint/order edits; return without effect when the post-infill payload is absent,
// and propagate cancellation or structural failures from the host callbacks.
void InternalBridgeAnchors::run_impl(const plugin_run_context *run_ctx) const
{
    throw_if_cancelled(run_ctx);
    const run_ctx_post_infill_generation *ctx = plugin_ctx_as_post_infill_generation(run_ctx);
    if (ctx == nullptr || ctx->object == nullptr || ctx->get_region_island_mutable_extrusion == nullptr)
        return;
    const RowsKey rows_key = BridgePreparation::prepared_rows_key(m_orchestrator);
    const uint32_t count = Object(ctx->object).layer_count();
    std::vector<LayerEnds> ends(count);
    parallel_for_storage_with_progress(0, count, run_ctx, nullptr, anchor_layer, ctx, &rows_key, &ends, run_ctx);
    parallel_for_storage_with_progress(0, count, run_ctx, nullptr, apply_layer, ctx, &rows_key, &ends, run_ctx);
}

// Register the plugin and its activation group before plugin option finalization.
void register_internal_bridge_anchors_plugin(orchestrator_handle *orch)
{
    const char *members[] = {"surface.bridge_preparation", "infill.post_process.internal_bridge_anchors"};
    if (!orchestrator_register_activation_group(orch, "bridge_preparation", {members, 2}))
        throw std::runtime_error("Cannot add the internal bridge anchors to the bridge-preparation activation group.");
    orchestrator_register_plugin(orch, InternalBridgeAnchors::instance(orch).c_instance());
}

}}} // namespace slic3r_api::Infill::InternalBridgeAnchorsPlugin
