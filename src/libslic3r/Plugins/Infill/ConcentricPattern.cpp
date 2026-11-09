///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/ Copyright (c) Prusa Research 2016 - 2023 Vojtěch Bubník, Lukáš Hejl
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

// Keep classic onion offsets, Arachne walls and seam ties synchronized with FillConcentric.cpp and Point.cpp.
#include "ConcentricPattern.hpp"

#include <algorithm>
#include <iterator>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"
#include "libslic3r/Plugins/Infill/ArachneInfill.hpp"
#include "libslic3r/Plugins/Infill/InfillPaths.hpp"
#include "libslic3r/Plugins/Infill/PatternPluginHelpers.hpp"

namespace slic3r_api { namespace Infill {
namespace work = PluginCommon::work;
namespace {
struct Spacing { double extrusion; coord_t distance; };

// The rings of one onion level: one contour per ExPolygon, and every hole with
// the contour of its own ExPolygon.
struct LevelRings {
    std::vector<uint32_t> contours;
    std::vector<uint32_t> holes;
    std::vector<uint32_t> hole_owners;
};

// Nearest vertex of a loop to a reference point, and its squared distance.
struct NearestVertex {
    uint32_t index;
    lengthsqr_t distance;
};

// Adjusted extrusion spacing also controls the initial half-spacing inset.
Spacing concentric_spacing(const raw_infill_pattern_params &params);
// Arachne recipes get variable-width walls on full surfaces whose spacing may be adjusted.
bool uses_arachne(const raw_infill_pattern_params &params);
// Fill one component with Arachne walls, outer walls first, and append them to the group.
void fill_component_arachne(storage_handle *storage, const ExPolygon &component, const Spacing &spacing,
                            const raw_infill_pattern_params &params,
                            const Slic3r::Arachne::WallToolPathsSettings &settings, const plugin_run_context *run_ctx,
                            StoredExtrusionEntity &group);
// Build the onion of one component as a ring nesting tree, then print it outside-in.
void fill_component(storage_handle *storage, const ExPolygon &component, const Spacing &spacing,
                    const raw_infill_pattern_params &params, const plugin_run_context *run_ctx, work::Polylines &out);
// Store the rings of one level counter-clockwise, each with an empty child list.
LevelRings append_level(const ExPolygonCollection &level, StoredPolygonCollection &rings,
                        std::vector<std::vector<uint32_t>> &children);
// Attach the rings of a deeper level and the holes of the outer level to their enclosing ring.
void link_levels(const ExPolygonCollection &outer_level, const LevelRings &outer, const LevelRings &deeper,
                 const StoredPolygonCollection &rings, std::vector<std::vector<uint32_t>> &children);
// Walk the nesting tree depth first, printing each ring before the rings it encloses.
void walk_rings(storage_handle *storage, StoredPolygonCollection &rings,
                const std::vector<std::vector<uint32_t>> &children, const raw_infill_pattern_params &params,
                const plugin_run_context *run_ctx, work::Polylines &out);
// Reduced-bit integer distances retain the first candidate on equal distances, like nearest_point_index().
NearestVertex nearest_vertex(const Polygon &polygon, c_point reference, const plugin_run_context *run_ctx);
// Whether a box contains a point, borders included.
bool box_contains(const c_bounding_box &box, c_point point);
// Fixed group order preserves outside-in adhesion and keeps closed paths eligible for loop publication.
void publish(const run_ctx_infill_pattern &ctx, storage_handle *storage, const work::Polylines &lines,
             const plugin_run_context *run_ctx);
class ConcentricPattern;
// One stateless provider serves independent surface calls.
ConcentricPattern &instance(orchestrator_handle *orchestrator);

Spacing concentric_spacing(const raw_infill_pattern_params &params)
{
    Spacing spacing{params.spacing, PatternHelpers::line_spacing_for_density(params)};
    if (params.density > 0.9999f && (params.infill_flags & C_INFILL_FLAG_DONT_ADJUST) == 0) {
        spacing.distance = PatternHelpers::adjust_solid_spacing(
            params.object_bounding_box.max.x - params.object_bounding_box.min.x, spacing.distance);
        spacing.extrusion = unscaled(spacing.distance);
        // The native filler rescales this double, including truncation at integral boundaries.
        spacing.distance = scale_i(spacing.extrusion);
    }
    return spacing;
}

bool uses_arachne(const raw_infill_pattern_params &params)
{
    return (params.infill_flags & C_INFILL_FLAG_USE_ARACHNE) != 0 && params.density > 0.9999f &&
           (params.infill_flags & C_INFILL_FLAG_DONT_ADJUST) == 0;
}

/*
Like FillConcentric::_fill_surface_single() with Arachne: the component is
grown back to the surface, then Arachne lays as many walls as fit. The walls
come inset by inset, outer first; each closed wall starts at its vertex nearest
to the end of the previous one, before loop clipping.
*/
void fill_component_arachne(storage_handle *storage, const ExPolygon &component, const Spacing &spacing,
                            const raw_infill_pattern_params &params,
                            const Slic3r::Arachne::WallToolPathsSettings &settings, const plugin_run_context *run_ctx,
                            StoredExtrusionEntity &group)
{
    const c_bounding_box box = component.contour().bounding_box();
    const coord_t loops_count = std::max(box.max.x - box.min.x, box.max.y - box.min.y) / spacing.distance + 1;
    const StoredExPolygonCollection outline =
        clipper_offset(ClipperOperand(storage, component), double(float(spacing.distance) / 2.f)).to_expolygon_collection();
    const Slic3r::Polygons polygons = ArachneInfill::to_polygons(outline.readonly());
    throw_if_cancelled(run_ctx);
    Slic3r::Arachne::WallToolPaths walls(polygons, spacing.distance, params.flow.width, spacing.distance,
                                         params.flow.width, size_t(loops_count), 0, params.layer_height, settings);
    const std::vector<Slic3r::Arachne::VariableWidthLines> &insets = walls.getToolPaths();
    throw_if_cancelled(run_ctx);

    Slic3r::Point last_position(0, 0);
    for (const Slic3r::Arachne::VariableWidthLines &inset : insets)
        for (const Slic3r::Arachne::ExtrusionLine &wall : inset) {
            if (wall.empty())
                continue;
            Slic3r::ThickPolyline line = Slic3r::Arachne::to_thick_polyline(wall);
            if (wall.is_closed)
                line.start_at_index(int(ArachneInfill::nearest_point_index(line.points, last_position)));
            last_position = line.back();
            line.clip_end(params.loop_clipping);
            if (!line.is_valid())
                continue;
            std::optional<StoredExtrusionEntity> path = ArachneInfill::to_extrusion(storage, line, params);
            if (path.has_value())
                group.append_child_move(path->mutable_view());
        }
}

/*
Every onion level is offset from the previous one, so the nesting of its rings is
known without a global union: a contour lies inside the contour of the ExPolygon
it was offset from, and a hole lies inside the deeper hole that grew around it,
or else directly inside its own contour. Printing that tree depth first keeps
every ring after the ring around it, for adhesion, and finishes a branch before
moving to a sibling.
*/
void fill_component(storage_handle *storage, const ExPolygon &component, const Spacing &spacing,
                    const raw_infill_pattern_params &params, const plugin_run_context *run_ctx, work::Polylines &out)
{
    StoredPolygonCollection rings(storage);
    std::vector<std::vector<uint32_t>> children;
    StoredExPolygonCollection last(storage);
    last.push_back(component);
    LevelRings current = append_level(last.readonly(), rings, children);
    while (true) {
        throw_if_cancelled(run_ctx);
        const ClipperOperand subject(storage, last.readonly());
        const ClipperOperand cleaned = clipper_offset2(subject,
            -double(spacing.distance + scale_d(spacing.extrusion) / 2),
            double(scale_d(spacing.extrusion) / 2) + params.fill_resolution / 2);
        StoredExPolygonCollection next = clipper_offset(cleaned, double(-params.fill_resolution / 2)).to_expolygon_collection();
        if (next.empty())
            break;
        const LevelRings deeper = append_level(next.readonly(), rings, children);
        link_levels(last.readonly(), current, deeper, rings, children);
        last = std::move(next);
        current = deeper;
    }
    // The deepest holes have no deeper hole around them.
    for (size_t index = 0; index < current.holes.size(); ++index)
        children[current.hole_owners[index]].push_back(current.holes[index]);
    walk_rings(storage, rings, children, params, run_ctx, out);
}

LevelRings append_level(const ExPolygonCollection &level, StoredPolygonCollection &rings,
                        std::vector<std::vector<uint32_t>> &children)
{
    LevelRings result;
    for (const ExPolygon expolygon : level) {
        const uint32_t contour = uint32_t(rings.size());
        rings.push_back(expolygon.contour());
        result.contours.push_back(contour);
        for (const Polygon hole : expolygon.holes()) {
            result.holes.push_back(uint32_t(rings.size()));
            result.hole_owners.push_back(contour);
            rings.push_back(hole);
            // Every ring is printed counter-clockwise, holes included.
            multipoint_reverse(polygon_as_multipoint(polygons_at(rings.mutable_handle(), rings.size() - 1)));
        }
    }
    for (uint32_t index = uint32_t(children.size()); index < rings.size(); ++index)
        multipoint_ensure_valid(polygon_as_multipoint(polygons_at(rings.mutable_handle(), index)), SCALED_EPSILON);
    children.resize(rings.size());
    return result;
}

void link_levels(const ExPolygonCollection &outer_level, const LevelRings &outer, const LevelRings &deeper,
                 const StoredPolygonCollection &rings, std::vector<std::vector<uint32_t>> &children)
{
    // A deeper contour lies in the region of exactly one ExPolygon of the outer level.
    for (const uint32_t contour : deeper.contours) {
        const c_point point = rings[contour].front();
        uint32_t parent = outer.contours.front();
        for (uint32_t index = 0; index < outer.contours.size(); ++index)
            if (outer_level[index].contains(point)) {
                parent = outer.contours[index];
                break;
            }
        children[parent].push_back(contour);
    }
    // Holes only grow, so an outer hole lies in at most one deeper hole.
    for (size_t index = 0; index < outer.holes.size(); ++index) {
        const c_point point = rings[outer.holes[index]].front();
        uint32_t parent = outer.hole_owners[index];
        for (const uint32_t hole : deeper.holes) {
            const Polygon candidate = rings[hole];
            if (box_contains(candidate.bounding_box(), point) && candidate.contains(point)) {
                parent = hole;
                break;
            }
        }
        children[parent].push_back(outer.holes[index]);
    }
}

void walk_rings(storage_handle *storage, StoredPolygonCollection &rings,
                const std::vector<std::vector<uint32_t>> &children, const raw_infill_pattern_params &params,
                const plugin_run_context *run_ctx, work::Polylines &out)
{
    // Each stack entry holds the rings of one sibling set that are still to print.
    c_point last_position{0, 0};
    StoredPolylineCollection paths(storage);
    std::vector<std::vector<uint32_t>> stack{{0}};
    size_t printed = 0;
    while (!stack.empty()) {
        std::vector<uint32_t> &siblings = stack.back();
        if (siblings.empty()) {
            stack.pop_back();
            continue;
        }
        if ((printed++ & 1023u) == 0)
            throw_if_cancelled(run_ctx);
        // The nearest sibling to the last seam starts next; one sibling needs no search.
        size_t chosen = 0;
        if (siblings.size() > 1) {
            lengthsqr_t best = std::numeric_limits<lengthsqr_t>::max();
            for (size_t index = 0; index < siblings.size(); ++index) {
                const lengthsqr_t distance = nearest_vertex(rings[siblings[index]], last_position, run_ctx).distance;
                if (distance < best) {
                    best = distance;
                    chosen = index;
                }
            }
        }
        const uint32_t ring = siblings[chosen];
        siblings.erase(siblings.begin() + chosen);
        const Polygon loop = rings[ring];
        if (loop.size() >= 3) {
            StoredPolyline path(storage);
            polygon_split(loop.handle(), nearest_vertex(loop, last_position, run_ctx).index, path.mutable_handle());
            // The next seam follows the untrimmed loop's closing point, not the clipped endpoint.
            last_position = path.back();
            path.clip_end(params.loop_clipping);
            if (path.is_valid())
                paths.push_back_move(std::move(path));
        }
        stack.push_back(children[ring]);
    }
    work::Polylines converted = Paths::read_polylines(paths, run_ctx);
    for (work::Polyline &path : converted)
        out.push_back(std::move(path));
}

NearestVertex nearest_vertex(const Polygon &polygon, c_point reference, const plugin_run_context *run_ctx)
{
    NearestVertex nearest{0, std::numeric_limits<lengthsqr_t>::max()};
    const multipoint_const_view points = polygon.view();
    for (uint32_t index = 0; index < points.size; ++index) {
        if ((index & 1023u) == 0)
            throw_if_cancelled(run_ctx);
        lengthsqr_t candidate = coord_int_sqr(points.array[index].x - reference.x);
        if (candidate < nearest.distance) {
            candidate += coord_int_sqr(points.array[index].y - reference.y);
            if (candidate < nearest.distance) {
                nearest.index = index;
                nearest.distance = candidate;
            }
        }
    }
    return nearest;
}

bool box_contains(const c_bounding_box &box, c_point point)
{
    return box.min.x <= point.x && point.x <= box.max.x && box.min.y <= point.y && point.y <= box.max.y;
}

void publish(const run_ctx_infill_pattern &ctx, storage_handle *storage, const work::Polylines &lines,
             const plugin_run_context *run_ctx)
{
    StoredExtrusionEntity group(storage);
    for (size_t index = 0; index < lines.size(); ++index) {
        if ((index & 1023u) == 0)
            throw_if_cancelled(run_ctx);
        StoredExtrusionEntity path = PatternHelpers::make_path(storage, lines[index].points, *ctx.params);
        group.append_child_move(path.mutable_view());
    }
    group.set_flags(0);
    throw_if_cancelled(run_ctx);
    if (!group.empty())
        MutableExtrusionEntity(ctx.output).append_child_move(group.mutable_view());
}

class ConcentricPattern : public PluginBase
{
public:
    explicit ConcentricPattern(orchestrator_handle *orchestrator) : PluginBase(orchestrator) {}

    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override {
        if (keys != nullptr)
            std::copy(std::begin(ArachneInfill::k_settings_keys), std::end(ArachneInfill::k_settings_keys), keys);
        return int32_t(ArachneInfill::k_settings_key_count);
    }
private:
    const char *id_impl() const noexcept override { return "concentric"; }
    const char *name_impl() const noexcept override { return "Concentric"; }
    const char *description_impl() const noexcept override { return "Outside-in concentric infill loops."; }
    slicing_step_t step_impl() const noexcept override { return INFILL_PATTERN; }
    const char *const *dependencies_impl() const noexcept override {
        static const char *none[] = {nullptr};
        return none;
    }
    int32_t priority_impl() const noexcept override { return 9; }
    const char *progress_message_format_impl() const noexcept override { return "Generating concentric infill: %u / %u surfaces"; }
    void inilialize_impl(storage_handle *) const override {
        PatternHelpers::add_self_to_pattern_list(m_orchestrator, "concentric", "Concentric",
            {"fill_pattern", "top_fill_pattern", "bottom_fill_pattern", "solid_fill_pattern"});
    }
    void run_impl(const plugin_run_context *run_ctx) const override {
        throw_if_cancelled(run_ctx);
        const run_ctx_infill_pattern *ctx = plugin_ctx_as_infill_pattern(run_ctx);
        if (ctx == nullptr || ctx->surface == nullptr || ctx->params == nullptr || ctx->output == nullptr ||
            run_ctx->plugin_storage == nullptr)
            return;
        const Slic3r::Arachne::WallToolPathsSettings arachne_settings = uses_arachne(*ctx->params) ?
            ArachneInfill::read_settings(*ctx) : Slic3r::Arachne::WallToolPathsSettings{};
        fill_concentric_surface(*ctx, run_ctx->plugin_storage, run_ctx, arachne_settings);
        progress().increment();
    }
};

ConcentricPattern &instance(orchestrator_handle *orchestrator)
{
    static ConcentricPattern pattern(orchestrator);
    return pattern;
}
} // namespace

void fill_concentric_surface(const run_ctx_infill_pattern &ctx, storage_handle *storage, const plugin_run_context *run_ctx,
                             const Slic3r::Arachne::WallToolPathsSettings &arachne_settings)
{
    throw_if_cancelled(run_ctx);
    const raw_infill_pattern_params &params = *ctx.params;
    if (params.density <= 0.f || params.spacing <= 0.)
        return;
    // Sparse or fixed-spacing Arachne recipes keep the classic loops; gap fill is outside this provider.
    const Spacing spacing = concentric_spacing(params);
    const ExPolygon surface = Surface(ctx.surface).expolygon();
    // The component order of the offset is unspecified; each component restarts its own seam reference.
    StoredExPolygonCollection components =
        clipper_offset(ClipperOperand(storage, surface), scale_d(-0.5 * spacing.extrusion)).to_expolygon_collection();
    if (uses_arachne(params)) {
        StoredExtrusionEntity group(storage);
        for (const ExPolygon component : components) {
            throw_if_cancelled(run_ctx);
            fill_component_arachne(storage, component, spacing, params, arachne_settings, run_ctx, group);
        }
        group.set_flags(0);
        throw_if_cancelled(run_ctx);
        if (!group.empty())
            MutableExtrusionEntity(ctx.output).append_child_move(group.mutable_view());
        return;
    }
    components.ensure_valid();
    work::Polylines lines;
    for (const ExPolygon component : components) {
        throw_if_cancelled(run_ctx);
        fill_component(storage, component, spacing, params, run_ctx, lines);
        work::ensure_valid(lines, params.fill_resolution);
    }
    publish(ctx, storage, lines, run_ctx);
}

void register_concentric_pattern_plugin(orchestrator_handle *orchestrator)
{
    orchestrator_register_plugin(orchestrator, instance(orchestrator).c_instance());
}
}} // namespace slic3r_api::Infill
