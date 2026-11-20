///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

#include "ZonePlanner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "../../Common/ClipperHelpers.hpp"
#include "../../ElephantFootCompensation.hpp"
#include "../AvoidCrossing/LayerInputCache.hpp"
#include "libslic3r/Api/plugin/cpp/ConfigViews.hpp"

/*
ZonePlanner prepares one cached geometry entry per active-zone id, tool, sorted ignored
obstacle identities, spacing, and optional platter context (group and Z).
For each endpoint it groups the original pieces containing that point and extends
the group transitively through overlap or boundary contact. It tries keeping both
groups, omitting the source group, omitting the target group, then omitting both,
skipping repeats and absent groups. Each combination unions the remaining pieces
into Z3, then
subtracts Z3 from the active attractor union and from a fixed bed_shape contour. M_a
and M_p are the two membership domains; W_a is made by withdrawing each M_a component,
while W_p is the bed contour minus the nozzle offset Z4. Endpoint provenance selects a
matching attractor component. A nonempty W_p allows the planner to look for a
platter attachment, but the search may fail. An endpoint inside a kept group
instead tries straight exits to the nearest eligible W point of each type.
A blocked exit is discarded without trying a farther point.
Material checks shrink obstacles by SCALED_EPSILON because diff_pl has no reliable side for paths on a border.
Walls thinner than twice that inset can disappear from the check. Domains, detours,
the reference and the score still use the original obstacle geometry.
Hypotheses share their selected W points between stable estimates and construction.
Ordinary attractor attachments stay in M_a and ordinary platter attachments stay
in M_p. Endpoints in one M_a component drop their platter attachments when their
chord leaves that component, or leaves its W_a while both attach to it. The estimates treat exits and platter attachments as outside connectors;
bridges, external connectors and the platter-only reference use W_p rings, whose
obstacle-facing rings are Z4. Construction checks the route without exits against
the inset of Z3, then adds the checked exits and retains exact endpoints. Validation measures
the complete route outside the initial platter expanded by SCALED_EPSILON; this avoids
treating a path laid exactly on a border as outside when Clipper's diff_pl cannot assign
it a reliable side. It rejects more than SCALED_EPSILON, respects the detour
budget and, when the platter-only
reference is valid, stays within its outside-M_a length plus
SCALED_EPSILON. The first admissible route of each combination competes by length,
counting length inside the original endpoint groups three times. A tie keeps the
first route. If none passes, the straight path is returned and counted as one fallback.
Only the winner contributes counts. Paths using an attractor count as domain routes, attractor-to-attractor paths
between distinct components count as bridges. Outside-part travels also use translated normal
slices from the plan at the current group Z or below; the synthetic wipe tower is not
a blocker. The plan catalog, object slices and all other borrowed inputs remain
unchanged and outlive this worker-local cache.

Platter-only references and external connectors use the public
`clipper_detour_polyline_expolygons` operation in Shortest mode on W_p rings; its
ring-following and rounding rules are defined in `slic3r_clipper.h`. ZonePlanner
extends only external connector chords before the call so a crossing at an endpoint
is visible when the chord leaves a ring into its interior or arrives from its
interior. The M-to-W link uses the same operation without that extension. Temporary
extension points are removed and exact endpoints are restored. The material check
compares path length with its Clipper difference, and the total detour budget rejects
unsafe hypotheses; exhaustion produces the counted straight fallback. Cancellation
propagates.

    path: classify endpoints, group obstacles and score admitted routes
    |-- eligible: resolve regional and auxiliary activation
    |-- outside_part / is_off_object_layer: classify normal versus auxiliary provenance
    |-- platter_blockers: collect translated slices for outside-part and mixed travels
    |-- touches: group endpoint-containing obstacle pieces by overlap or contact
    |-- outside_length: measure the score against the union of the original endpoint groups
    `-- route: build and validate the first usable hypothesis for one combination
        |-- union / diff / offset: build Z3, Z4, M_a, W_a, M_p and W_p on a cache miss
        |-- build_boundary: index W_a for each component and, when needed, its combined boundary
        |-- endpoint_candidates: match source provenance and choose ordinary attachments
        |   |-- nearest_point: find a deterministic W attachment
        |   `-- detour_in_component: build a link inside M_a or M_p
        |-- exit_candidates: choose the nearest eligible W points and check straight exits
        |   `-- blocked: check each exit against the inset of the other kept obstacles
        |-- select_island_bridge: estimate weighted approaches between components
        |-- estimate_cost: weight inside ring estimates and clip the other connectors
        |-- append_component_path: route within one attractor component between selected points
        |   `-- route_on_boundary: follow crossed W_a rings, else keep the chord
        |-- detour: extend connector chords and call the public Clipper operation on W_p rings
        |-- blocked: check the rest of the route against the inset of Z3
        `-- outside_length: check bed containment and compare with the platter-only reference
*/
namespace slic3r_api { namespace LayerExtrusionEdit { namespace AdvancedTravel {
namespace work = PluginCommon::polygon_work;
namespace clipper = PluginCommon::clipper;
namespace travel = TravelConnection;

namespace {
// Candidates identify an attractor M_a component or a platter W_p component.
// ring_point is the exact point reached on W_a or W_p; link joins the endpoint to it.
enum class DomainType { Attractor, Platter };
struct Attachment {
    DomainType type = DomainType::Platter;
    size_t component = 0;
    work::Point ring_point;
    work::Points link;
    // A straight exit may cross the group containing the original endpoint.
    bool exit = false;
};
// Connector endpoints are shared by estimation and construction so both phases use the same points.
struct Hypothesis { Attachment source, target; work::Point exit, entry; double cost; size_t rank; };
// Append a component's route between already attached points (crossed rings, else a direct chord), retaining exact endpoints and borrowing its cached boundary.
// Epsilon-near points connect directly. The counter required by route_on_boundary is not published; cancellation propagates.
void append_component_path(work::Points &points, AvoidCrossing::Boundary &boundary, const work::Point &from,
    const work::Point &to, const plugin_run_context *context);
// Inside estimates stand for a route within a component; Outside retains geometric domain intersections.
enum class EstimateKind { Inside, Outside };
struct EstimateSegment { work::Point from, to; EstimateKind kind; };
// Classify regionless endpoints and endpoints on auxiliary layers as outside the part; normal object layers remain inside.
bool outside_part(const travel::TravelEndpoint &endpoint);
// Find the nearest point on any component ring, retaining ring/edge order on equal distances.
work::Point nearest_point(const work::ExPolygon &component, const work::Point &point);
// Return usable attractor and platter candidates for one endpoint; link checks use original M_a and M_p components.
std::vector<Attachment> endpoint_candidates(const work::ExPolygons &attractor_components,
    const work::ExPolygons &withdrawn_components, const std::vector<std::vector<size_t>> &component_sources,
    const work::ExPolygons &platter_components,
    const work::ExPolygons &platter_free_components,
    const std::vector<size_t> &platter_component_sources, const TravelZoneView &zones,
    const travel::TravelEndpoint &endpoint, const work::Point &point, bool include_attractor,
    const plugin_run_context *context);
// Return the nearest matching attractor and nearest platter exits, rejecting chords crossing other obstacles.
std::vector<Attachment> exit_candidates(const work::ExPolygons &withdrawn_components,
    const std::vector<std::vector<size_t>> &component_sources, const work::ExPolygons &platter_free_components,
    const work::ExPolygons &other_obstacles, const TravelZoneView &zones, const travel::TravelEndpoint &endpoint,
    const work::Point &point, bool include_attractor, const plugin_run_context *context);
// Check whether a source matches the endpoint's layer, instance and position.
bool matches_source(const TravelZoneView &zones, size_t index, const layer_handle *layer,
    const travel::TravelEndpoint &endpoint, const work::Point &point);
// Sum segment lengths after clipping to the complement of the domain; an empty domain leaves the whole path outside.
double outside_length(const work::Points &points, const work::ExPolygons &domain, const plugin_run_context *context);
// Score inside estimates as weighted inside length; clip other connectors against all domains before weighting their inside portions.
// Zero-length segments cost nothing. Cancellation propagates per segment.
double estimate_cost(const std::vector<EstimateSegment> &segments, const work::ExPolygons &domain, double weight, const plugin_run_context *context);
// Test filled areas without changing them; holes exclude their interior and borders follow the supplied flag.
bool contains(const work::ExPolygons &areas, const work::Point &point, bool border = true);
// Compare the typed domain identity carried by two endpoint candidates.
bool same_domain(const Attachment &source, const Attachment &target);
// Return whether two original pieces overlap or touch, including boundary-only contact between any rings.
bool touches(const work::ExPolygon &first, const work::ExPolygon &second, const plugin_run_context *context);
// Return whether two areas share a filled region, excluding point and edge-only contact.
bool overlaps_area(const work::ExPolygon &first, const work::ExPolygon &second, const plugin_run_context *context);
// Extend connector ends, call Clipper's shortest detour on original W_p rings, and restore exact endpoints.
// Border endpoints stay unextended; each inserted subject point differs from its predecessor.
work::Points detour(const work::Point &from, const work::Point &to, const work::ExPolygons &rings,
                    const work::ExPolygon &platter, const plugin_run_context *context);
// Call the Clipper detour operation on one original M component without extending its endpoints.
// The caller checks the resulting link against the original component.
work::Points detour_in_component(const work::Point &from, const work::Point &to, const work::ExPolygon &component,
                                 const plugin_run_context *context);
// Return whether the path loses more than the scaled tolerance inside the supplied material mask.
// An empty mask is never blocked; cancellation and Clipper errors propagate.
bool blocked(const work::Points &points, const work::ExPolygons &areas, const plugin_run_context *context);

} // namespace

// Copy the fixed bed contour and cache its SCALED_EPSILON-expanded copy for complete-route containment.
ZonePlanner::ZonePlanner(AvoidCrossing::CompatibilityPlanner &compatibility, TravelZoneView &zones,
                         const Config &print_config, const bool auxiliary_avoidance, const plugin_run_context *context,
                         const Print &print, AvoidCrossing::LayerInputCache &layer_inputs,
                         const PrintingPlan &plan, const uint32_t group_idx, const coord_t print_z) :
    m_compatibility(compatibility), m_zones(zones), m_config(print_config),
    m_auxiliary_avoidance(auxiliary_avoidance), m_context(context), m_print(print), m_layer_inputs(layer_inputs),
    m_plan(plan), m_group_idx(group_idx), m_print_z(print_z)
{
    // bed_shape is a millimetre-valued contour; copy it once in scaled coordinates
    // so every travel uses the same initial platter throughout this planner's life.
    const ConfigOption bed_shape = m_config.get("bed_shape");
    m_platter_initial.contour.points.reserve(bed_shape.size());
    for (uint32_t index = 0; index < bed_shape.size(); ++index) {
        const ConfigPoint point = bed_shape.get_point(index);
        m_platter_initial.contour.points.emplace_back(scale_i(point.x), scale_i(point.y));
    }
    if (!m_platter_initial.contour.points.empty())
        m_platter_expanded = clipper::offset_ex(m_context->plugin_storage, {m_platter_initial}, double(SCALED_EPSILON));
}

bool ZonePlanner::eligible(const travel::TravelEndpoint &source, const travel::TravelEndpoint &target) const
{
    return eligible(m_compatibility, m_config, m_auxiliary_avoidance, source, target);
}

bool ZonePlanner::eligible(const AvoidCrossing::CompatibilityPlanner &compatibility, const Config &print_config,
                           const bool auxiliary_avoidance, const travel::TravelEndpoint &source,
                           const travel::TravelEndpoint &target)
{
    const bool source_has_instance = compatibility.endpoint_has_object_instance(source);
    const bool target_has_instance = compatibility.endpoint_has_object_instance(target);
    if ((source_has_instance && !compatibility.endpoint_enables_avoidance(source)) ||
        (target_has_instance && !compatibility.endpoint_enables_avoidance(target))) return false;
    if (!source_has_instance && !target_has_instance && !auxiliary_avoidance) return false;
    if (target_has_instance && print_config.bool_or_default("avoid_crossing_not_first_layer", true)) {
        const Layer layer = target.region_island.region(0).layer();
        if (AvoidCrossing::is_first_layer(layer.object(), layer)) return false;
    }
    return true;
}

std::vector<c_point> ZonePlanner::path(const size_t t, const PendingTravel &pending)
{
    throw_if_cancelled(m_context);
    const std::vector<c_point> direct = travel::straight_path(pending.source, pending.target, pending.extruder_id);
    m_last = Decision{};
    if (!eligible(pending.source, pending.target)) return direct;
    const work::Point source(pending.source.position.x, pending.source.position.y);
    const work::Point target(pending.target.position.x, pending.target.position.y);
    if (source.coincides_with_epsilon(target)) {
        m_last.outcome = Decision::Outcome::NoMotion;
        return direct;
    }
    if (!m_platter_initial.contains(source, true) || !m_platter_initial.contains(target, true)) {
        m_last.outcome = Decision::Outcome::OutsideBed;
        ++m_fallbacks;
        return direct;
    }
    const bool source_outside = outside_part(pending.source), target_outside = outside_part(pending.target);
    m_last.outside_part = source_outside || target_outside;
    const bool use_platter = source_outside || target_outside;
    const double nozzle_diameter = m_config.vector_float_or_default("nozzle_diameter", pending.extruder_id, 0.4);
    coord_t spacing = scale_i(nozzle_diameter);
    double jump_weight = 0.;
    for (const travel::TravelEndpoint *endpoint : {&pending.target, &pending.source}) {
        if (!endpoint->region_island.valid() || endpoint->region_island.region_count() == 0) continue;
        const AvoidCrossing::LayerInput &layer = *m_layer_inputs.get(
            m_print, endpoint->region_island.region(0).layer(), m_context).target;
        spacing = layer.perimeter_spacing;
        for (const AvoidCrossing::RegionInput &region : layer.regions)
            if (region.avoid_travel_island) {
                jump_weight = region.avoid_travel_island_weight;
                break;
            }
        break;
    }
    const ActiveZones &active = m_zones.query(t, pending);
    std::vector<OriginalObstacle> original_parts;
    for (const size_t index : active.forbidden_indexes) {
        throw_if_cancelled(m_context);
        const work::ExPolygons &area = m_zones.zone_area(index);
        for (size_t component = 0; component < area.size(); ++component)
            original_parts.push_back({area[component], {false, index, component}});
    }
    std::optional<PlatterKey> platter_key;
    if (use_platter) {
        if (!m_platter_ready) {
            m_platter_blockers = AvoidCrossing::platter_blockers(m_context->plugin_storage, m_plan, m_group_idx, m_print_z);
            throw_if_cancelled(m_context);
            m_platter_ready = true;
        }
        for (size_t component = 0; component < m_platter_blockers.size(); ++component)
            original_parts.push_back({m_platter_blockers[component], {true, 0, component}});
        platter_key = PlatterKey(m_group_idx, m_print_z);
    }
    // Recover source identities directly by expanding each endpoint seed through touching pieces.
    const auto group = [&](const work::Point &point) {
        std::vector<bool> included(original_parts.size(), false);
        std::vector<size_t> pieces;
        for (size_t index = 0; index < original_parts.size(); ++index)
            if (original_parts[index].area.contains(point, true)) {
                included[index] = true;
                pieces.push_back(index);
            }
        for (size_t queue = 0; queue < pieces.size(); ++queue) {
            const size_t current = pieces[queue];
            for (size_t index = 0; index < original_parts.size(); ++index)
                if (!included[index] && touches(original_parts[current].area, original_parts[index].area, m_context)) {
                    included[index] = true;
                    pieces.push_back(index);
                }
        }
        std::sort(pieces.begin(), pieces.end());
        return pieces;
    };
    const std::vector<size_t> source_group = group(source), target_group = group(target);
    const bool distinct_target = !target_group.empty() && target_group != source_group;
    work::ExPolygons endpoint_obstacles;
    for (const size_t index : source_group) endpoint_obstacles.push_back(original_parts[index].area);
    if (distinct_target)
        for (const size_t index : target_group) endpoint_obstacles.push_back(original_parts[index].area);
    if (!endpoint_obstacles.empty())
        endpoint_obstacles = clipper::union_ex(m_context->plugin_storage, endpoint_obstacles);

    std::optional<Route> best;
    unsigned best_combination = 0;
    double best_score = 0.;
    m_last.source_in_obstacle = !source_group.empty();
    m_last.target_in_obstacle = !target_group.empty();
    // The low bit omits the source group, the high bit the distinct target group.
    for (unsigned combination = 0; combination < 4; ++combination) {
        if ((combination & 1) && source_group.empty()) continue;
        if ((combination & 2) && !distinct_target) continue;
        std::vector<bool> ignored(original_parts.size(), false);
        if (combination & 1)
            for (const size_t index : source_group) ignored[index] = true;
        if (combination & 2)
            for (const size_t index : target_group) ignored[index] = true;
        std::optional<Route> candidate = route(pending, active, original_parts, ignored,
            source_group, target_group, platter_key, spacing, nozzle_diameter, jump_weight);
        if (!candidate) continue;
        if (endpoint_obstacles.empty()) {
            best = std::move(candidate);
            best_combination = combination;
            break;
        }
        const double outside = outside_length(candidate->points, endpoint_obstacles, m_context);
        const double score = outside + 3 * (candidate->length - outside);
        if (!best || score < best_score) {
            best = std::move(candidate);
            best_combination = combination;
            best_score = score;
        }
    }
    if (!best) {
        m_last.outcome = Decision::Outcome::Exhausted;
        ++m_fallbacks;
        return direct;
    }
    if (best->domain_route) ++m_domain_routes;
    if (best->bridge) ++m_bridges;
    m_last.outcome = Decision::Outcome::Routed;
    // A shared source and target group is omitted by the low bit alone.
    m_last.source_obstacle_ignored = (best_combination & 1) != 0;
    m_last.target_obstacle_ignored = (best_combination & 2) != 0 || (!distinct_target && m_last.source_obstacle_ignored);
    m_last.source_island = best->source_island;
    m_last.target_island = best->target_island;
    m_last.source_exit = best->source_exit;
    m_last.target_exit = best->target_exit;
    m_last.source_link = best->source_link;
    m_last.target_link = best->target_link;
    m_last.bridge = best->bridge;
    std::vector<c_point> result;
    for (const work::Point &point : best->points) result.push_back({point.x(), point.y()});
    return result;
}

std::optional<ZonePlanner::Route> ZonePlanner::route(const PendingTravel &pending, const ActiveZones &active,
    const std::vector<OriginalObstacle> &original_parts, const std::vector<bool> &ignored,
    const std::vector<size_t> &source_group, const std::vector<size_t> &target_group,
    const std::optional<PlatterKey> &platter_key, const coord_t spacing, const double nozzle_diameter,
    const double jump_weight)
{
    const work::Point source(pending.source.position.x, pending.source.position.y);
    const work::Point target(pending.target.position.x, pending.target.position.y);
    throw_if_cancelled(m_context);
    std::vector<ObstacleIdentity> ignored_ids;
    for (size_t index = 0; index < original_parts.size(); ++index)
        if (ignored[index]) ignored_ids.push_back(original_parts[index].identity);
    std::sort(ignored_ids.begin(), ignored_ids.end());
    ignored_ids.erase(std::unique(ignored_ids.begin(), ignored_ids.end()), ignored_ids.end());
    throw_if_cancelled(m_context);
    const CacheKey key(active.id, pending.extruder_id, ignored_ids, spacing, platter_key);
    std::map<CacheKey, Obstacles>::iterator cached = m_obstacles.find(key);
    if (cached == m_obstacles.end()) {
        Obstacles obstacles;
        work::ExPolygons z3_parts;
        for (size_t index = 0; index < original_parts.size(); ++index)
            if (!ignored[index]) z3_parts.push_back(original_parts[index].area);
        if (!z3_parts.empty())
            obstacles.original = clipper::union_ex(m_context->plugin_storage, z3_parts);
        work::ExPolygons platter_expanded;
        if (!obstacles.original.empty()) {
            obstacles.blocking = clipper::offset_ex(m_context->plugin_storage, obstacles.original, -double(SCALED_EPSILON));
            const coord_t radius = std::max<coord_t>(1, scale_i(nozzle_diameter * 0.5));
            platter_expanded = clipper::offset_ex(m_context->plugin_storage, obstacles.original, double(radius));
        }
        obstacles.components = active.attractors;
        if (!obstacles.original.empty() && !obstacles.components.empty())
            obstacles.components = clipper::diff_ex(m_context->plugin_storage, obstacles.components, obstacles.original);
        obstacles.platter_components = {m_platter_initial};
        if (!obstacles.original.empty())
            obstacles.platter_components = clipper::diff_ex(m_context->plugin_storage,
                                                             obstacles.platter_components, obstacles.original);
        obstacles.platter_free_components = {m_platter_initial};
        if (!platter_expanded.empty())
            obstacles.platter_free_components = clipper::diff_ex(m_context->plugin_storage,
                                                                  obstacles.platter_free_components, platter_expanded);
        obstacles.component_sources.resize(obstacles.components.size());
        for (size_t component = 0; component < obstacles.components.size(); ++component) {
            throw_if_cancelled(m_context);
            for (const size_t source_index : active.attractor_indexes) {
                throw_if_cancelled(m_context);
                if (!clipper::intersection_ex(m_context->plugin_storage, m_zones.zone_area(source_index),
                                              {obstacles.components[component]}).empty())
                    obstacles.component_sources[component].push_back(source_index);
            }
        }
        // W_a uses min(0.01 mm, nozzle/10) and withdraws avoid_crossing_perimeters_inset (in mm, or a
        // percentage of the perimeter spacing, 150% by default) from each M_a component.
        const double minimum_width = std::min(0.01, nozzle_diameter / 10.);
        const c_float_or_percent inset = m_config.float_or_percent_or_default(
            "avoid_crossing_perimeters_inset", c_float_or_percent{150., 1});
        const coord_t withdrawal = inset.percent ? coord_t(0.01 * inset.value * double(spacing)) : scale_i(inset.value);
        for (const work::ExPolygon &component : obstacles.components) {
            throw_if_cancelled(m_context);
            const StoredExPolygonCollection stored_component = clipper::to_stored_expolygons(
                m_context->plugin_storage, {component});
            const slic3r_api::ExPolygon source_view = stored_component.readonly().at(0);
            const StoredExPolygonCollection withdrawn =
                XYSizeCompensationPlugin::detail::elephant_foot_compensation(
                    m_context->plugin_storage, source_view, minimum_width, withdrawal);
            const work::ExPolygons copied = clipper::from_expolygon_collection_view(withdrawn.readonly());
            obstacles.withdrawn_components.push_back(copied.front());
        }
        size_t ring = 0;
        for (const work::ExPolygon &component : obstacles.withdrawn_components) {
            throw_if_cancelled(m_context);
            obstacles.component_rings.push_back(ring);
            ring += 1 + component.holes.size();
            std::unique_ptr<AvoidCrossing::Boundary> boundary = std::make_unique<AvoidCrossing::Boundary>();
            AvoidCrossing::build_boundary(m_context->plugin_storage, {component}, 2 * spacing, *boundary);
            obstacles.component_boundaries.push_back(std::move(boundary));
        }
        if (obstacles.components.size() >= 2) {
            obstacles.domain = std::make_unique<AvoidCrossing::Boundary>();
            work::ExPolygons combined = obstacles.withdrawn_components;
            AvoidCrossing::build_boundary(m_context->plugin_storage, std::move(combined),
                                          2 * spacing, *obstacles.domain);
        }
        for (const work::ExPolygon &free_component : obstacles.platter_free_components) {
            size_t source_component = SIZE_MAX;
            for (size_t component = 0; component < obstacles.platter_components.size(); ++component)
                if (overlaps_area(free_component, obstacles.platter_components[component], m_context)) {
                    source_component = component;
                    break;
                }
            obstacles.platter_component_sources.push_back(source_component);
        }
        throw_if_cancelled(m_context);
        cached = m_obstacles.emplace(key, std::move(obstacles)).first;
    }
    const Obstacles &obstacles = cached->second;
    // Exit masks exclude their own group and any groups omitted from this combination.
    const auto other_obstacles = [&](const std::vector<size_t> &group) {
        work::ExPolygons parts;
        if (group.empty() || ignored[group.front()]) return parts;
        for (size_t index = 0; index < original_parts.size(); ++index)
            if (!ignored[index] && !std::binary_search(group.begin(), group.end(), index))
                parts.push_back(original_parts[index].area);
        return parts.empty() ? parts : clipper::offset_ex(m_context->plugin_storage,
            clipper::union_ex(m_context->plugin_storage, parts), -double(SCALED_EPSILON));
    };
    const work::ExPolygons source_mask = other_obstacles(source_group);
    const work::ExPolygons target_mask = target_group == source_group ? work::ExPolygons{} : other_obstacles(target_group);
    const auto candidates = [&](const travel::TravelEndpoint &endpoint, const work::Point &point,
                                const std::vector<size_t> &group, const work::ExPolygons &mask) {
        if (!group.empty() && !ignored[group.front()])
            return exit_candidates(obstacles.withdrawn_components, obstacles.component_sources,
                obstacles.platter_free_components, mask, m_zones, endpoint, point, !outside_part(endpoint), m_context);
        return endpoint_candidates(obstacles.components, obstacles.withdrawn_components, obstacles.component_sources,
            obstacles.platter_components, obstacles.platter_free_components, obstacles.platter_component_sources,
            m_zones, endpoint, point, !outside_part(endpoint), m_context);
    };
    const std::vector<Attachment> sources = candidates(pending.source, source, source_group, source_mask);
    const std::vector<Attachment> targets = candidates(pending.target, target, target_group,
        target_group == source_group ? source_mask : target_mask);
    std::vector<Attachment> filtered_sources = sources, filtered_targets = targets;
    const auto physical_component = [](const work::ExPolygons &areas, const work::Point &point) -> std::optional<size_t> {
        for (size_t component = 0; component < areas.size(); ++component)
            if (areas[component].contains(point, true)) return component;
        return std::nullopt;
    };
    const std::optional<size_t> source_membership = physical_component(obstacles.components, source);
    const std::optional<size_t> target_membership = physical_component(obstacles.components, target);
    const auto attached = [](const std::vector<Attachment> &candidates, const size_t component) {
        return std::any_of(candidates.begin(), candidates.end(), [component](const Attachment &candidate) {
            return candidate.type == DomainType::Attractor && candidate.component == component;
        });
    };
    // A platter chord inside one component would cross it or run along its outline, outside the withdrawn ring.
    if (source_membership.has_value() && target_membership.has_value() &&
        source_membership == target_membership &&
        (outside_length({source, target}, {obstacles.components[*source_membership]}, m_context) > SCALED_EPSILON ||
         (attached(sources, *source_membership) && attached(targets, *source_membership) &&
          outside_length({source, target}, {obstacles.withdrawn_components[*source_membership]}, m_context) >
              SCALED_EPSILON))) {
        const auto remove_platter = [](std::vector<Attachment> &candidates) {
            candidates.erase(std::remove_if(candidates.begin(), candidates.end(),
                [](const Attachment &candidate) { return candidate.type == DomainType::Platter; }), candidates.end());
        };
        remove_platter(filtered_sources);
        remove_platter(filtered_targets);
    }
    const auto append_route = [](work::Points &points, const work::Points &path, const bool reverse = false) {
        if (!reverse)
            points.insert(points.end(), path.begin(), path.end());
        else
            for (work::Points::const_reverse_iterator point = path.rbegin(); point != path.rend(); ++point)
                points.push_back(*point);
    };
    // Validate the core with ordinary attachments, then add the already checked exits.
    const auto validate = [&](const Attachment &from, const Attachment &to, const work::Points &middle) {
        work::Points core{from.exit ? from.ring_point : source};
        if (!from.exit) append_route(core, from.link);
        append_route(core, middle);
        if (!to.exit) append_route(core, to.link, true);
        core.push_back(to.exit ? to.ring_point : target);
        core.erase(std::unique(core.begin(), core.end()), core.end());
        if (core.size() > 1 && blocked(core, obstacles.blocking, m_context)) return work::Points{};
        work::Points points;
        if (from.exit) append_route(points, from.link);
        append_route(points, core);
        if (to.exit) append_route(points, to.link, true);
        points.erase(std::unique(points.begin(), points.end()), points.end());
        if (outside_length(points, m_platter_expanded, m_context) > SCALED_EPSILON) return work::Points{};
        return points;
    };
    std::vector<Attachment> reference_sources, reference_targets;
    for (const Attachment &candidate : sources)
        if (candidate.type == DomainType::Platter) reference_sources.push_back(candidate);
    for (const Attachment &candidate : targets)
        if (candidate.type == DomainType::Platter) reference_targets.push_back(candidate);
    bool reference_usable = false;
    double reference_outside = 0.;
    for (const Attachment &from : reference_sources) {
        for (const Attachment &to : reference_targets) {
            throw_if_cancelled(m_context);
            const work::Points candidate = validate(from, to, detour(from.ring_point, to.ring_point,
                obstacles.platter_free_components, m_platter_initial, m_context));
            if (candidate.empty()) continue;
            reference_outside = outside_length(candidate, obstacles.components, m_context);
            reference_usable = true;
            break;
        }
        if (reference_usable) break;
    }
    std::vector<Hypothesis> hypotheses;
    for (const Attachment &from : filtered_sources)
        for (const Attachment &to : filtered_targets) {
            throw_if_cancelled(m_context);
            const bool from_attractor = from.type == DomainType::Attractor;
            const bool to_attractor = to.type == DomainType::Attractor;
            if (!from_attractor && !to_attractor && from.component != to.component) continue;
            std::vector<EstimateSegment> estimate;
            const auto append_estimate = [&estimate](const work::Points &path, const EstimateKind kind,
                                                      const bool reverse = false) {
                if (!reverse) {
                    for (size_t i = 1; i < path.size(); ++i)
                        estimate.push_back({path[i - 1], path[i], kind});
                } else {
                    for (size_t i = path.size(); i > 1; --i)
                        estimate.push_back({path[i - 1], path[i - 2], kind});
                }
            };
            work::Point exit = from.ring_point, entry = to.ring_point;
            const EstimateKind from_kind = from_attractor && !from.exit ? EstimateKind::Inside : EstimateKind::Outside;
            const EstimateKind to_kind = to_attractor && !to.exit ? EstimateKind::Inside : EstimateKind::Outside;
            append_estimate(from.link, from_kind);
            size_t rank = from_attractor && to_attractor ? 0 : (from_attractor || to_attractor ? 1 : 2);
            if (from_attractor && to_attractor && same_domain(from, to)) {
                estimate.push_back({from.ring_point, to.ring_point, EstimateKind::Inside});
            } else if (from_attractor && to_attractor) {
                AvoidCrossing::RouterCounts estimate_counts;
                work::Point start = from.ring_point, end = to.ring_point;
                // Coincident attachments already meet; the bridge search needs a nonzero chord.
                if (start != end) {
                    const std::optional<AvoidCrossing::IslandBridge> bridge = AvoidCrossing::select_island_bridge(
                        *obstacles.domain, obstacles.component_rings[from.component], obstacles.component_rings[to.component],
                        start, end, jump_weight, estimate_counts,
                        [this]() { throw_if_cancelled(m_context); });
                    if (!bridge) continue;
                    exit = bridge->from;
                    entry = bridge->to;
                }
                estimate.push_back({from.ring_point, exit, EstimateKind::Inside});
                estimate.push_back({exit, entry, EstimateKind::Outside});
                estimate.push_back({entry, to.ring_point, EstimateKind::Inside});
            } else if (from_attractor) {
                exit = nearest_point(obstacles.withdrawn_components[from.component], to.ring_point);
                estimate.push_back({from.ring_point, exit, EstimateKind::Inside});
                estimate.push_back({exit, to.ring_point, EstimateKind::Outside});
            } else if (to_attractor) {
                entry = nearest_point(obstacles.withdrawn_components[to.component], from.ring_point);
                estimate.push_back({from.ring_point, entry, EstimateKind::Outside});
                estimate.push_back({entry, to.ring_point, EstimateKind::Inside});
            } else {
                estimate.push_back({from.ring_point, to.ring_point, EstimateKind::Outside});
            }
            append_estimate(to.link, to_kind, true);
            hypotheses.push_back({from, to, exit, entry,
                                  estimate_cost(estimate, obstacles.components, jump_weight, m_context), rank});
        }
    std::stable_sort(hypotheses.begin(), hypotheses.end(), [](const Hypothesis &a, const Hypothesis &b) {
        if (a.cost != b.cost) return a.cost < b.cost;
        return a.rank < b.rank;
    });
    const c_float_or_percent budget = m_config.float_or_percent_or_default(
        "avoid_crossing_perimeters_max_detour", c_float_or_percent{0., 0});
    const double direct_length = work::Line(source, target).length();
    for (const Hypothesis &hypothesis : hypotheses) {
        throw_if_cancelled(m_context);
        const Attachment &from = hypothesis.source, &to = hypothesis.target;
        const bool from_attractor = from.type == DomainType::Attractor;
        const bool to_attractor = to.type == DomainType::Attractor;
        const bool domain_route = from_attractor || to_attractor;
        work::Points middle;
        if (from_attractor && to_attractor && same_domain(from, to)) {
            append_component_path(middle, *obstacles.component_boundaries[from.component], from.ring_point,
                                  to.ring_point, m_context);
        } else {
            if (from_attractor)
                append_component_path(middle, *obstacles.component_boundaries[from.component], from.ring_point,
                                      hypothesis.exit, m_context);
            append_route(middle, detour(hypothesis.exit, hypothesis.entry,
                obstacles.platter_free_components, m_platter_initial, m_context));
            if (to_attractor)
                append_component_path(middle, *obstacles.component_boundaries[to.component], hypothesis.entry,
                                      to.ring_point, m_context);
        }
        work::Points points = validate(from, to, middle);
        if (points.empty()) continue;
        double length = 0.;
        for (size_t i = 1; i < points.size(); ++i) length += work::Line(points[i - 1], points[i]).length();
        if (budget.value > 0.) {
            const double limit = budget.percent ? direct_length * 0.01 * budget.value : scale_d(budget.value);
            if (length - direct_length > limit) continue;
        }
        if (reference_usable && outside_length(points, obstacles.components, m_context) >
            reference_outside + SCALED_EPSILON) continue;
        return Route{std::move(points), length, domain_route,
            from_attractor && to_attractor && from.component != to.component,
            from_attractor, to_attractor, from.exit, to.exit,
            !from.exit && !from.link.empty(), !to.exit && !to.link.empty()};
    }
    return std::nullopt;
}

namespace {
std::vector<Attachment> endpoint_candidates(const work::ExPolygons &attractor_components,
    const work::ExPolygons &withdrawn_components, const std::vector<std::vector<size_t>> &component_sources,
    const work::ExPolygons &platter_components,
    const work::ExPolygons &platter_free_components,
    const std::vector<size_t> &platter_component_sources, const TravelZoneView &zones,
    const travel::TravelEndpoint &endpoint, const work::Point &point, const bool include_attractor,
    const plugin_run_context *context)
{
    std::vector<Attachment> candidates;
    const layer_handle *layer = endpoint.region_island.valid() && endpoint.region_island.region_count() != 0 ?
        endpoint.region_island.region(0).layer().handle() : nullptr;
    if (include_attractor) {
        for (size_t component = 0; component < attractor_components.size(); ++component) {
            throw_if_cancelled(context);
            if (!attractor_components[component].contains(point, true)) continue;
            bool source_matches = false;
            for (const size_t index : component_sources[component])
                if (matches_source(zones, index, layer, endpoint, point)) {
                    source_matches = true;
                    break;
                }
            if (!source_matches) continue;
            Attachment selected{DomainType::Attractor, component, point, {}};
            if (!withdrawn_components[component].contains(point, true)) {
                selected.ring_point = nearest_point(withdrawn_components[component], point);
                selected.link = detour_in_component(point, selected.ring_point, attractor_components[component], context);
                if (outside_length(selected.link, {attractor_components[component]}, context) > SCALED_EPSILON)
                    continue;
            }
            candidates.push_back(std::move(selected));
            break;
        }
    }
    if (!platter_free_components.empty()) {
        bool in_free_platter = false;
        for (size_t component = 0; component < platter_free_components.size(); ++component) {
            throw_if_cancelled(context);
            if (platter_free_components[component].contains(point, true)) {
                candidates.push_back({DomainType::Platter, component, point, {}});
                in_free_platter = true;
            }
        }
        if (!in_free_platter) {
            size_t best_component = 0, best_m_component = 0;
            work::Point best_point;
            double best_distance = (std::numeric_limits<double>::max)();
            bool found_component = false;
            for (size_t component = 0; component < platter_free_components.size(); ++component) {
                throw_if_cancelled(context);
                const size_t m_component = platter_component_sources[component];
                if (m_component == SIZE_MAX || !platter_components[m_component].contains(point, true))
                    continue;
                const work::Point ring_point = nearest_point(platter_free_components[component], point);
                const double distance = point.distance_to(ring_point);
                if (distance < best_distance) {
                    found_component = true;
                    best_distance = distance;
                    best_component = component;
                    best_m_component = m_component;
                    best_point = ring_point;
                }
            }
            // Reject an endpoint with no W_p component mapped to its containing M_p component.
            if (!found_component) return candidates;
            const work::Points link = detour_in_component(point, best_point,
                platter_components[best_m_component], context);
            if (outside_length(link, {platter_components[best_m_component]}, context) <= SCALED_EPSILON)
                candidates.push_back({DomainType::Platter, best_component, best_point, link});
        }
    }
    return candidates;
}

std::vector<Attachment> exit_candidates(const work::ExPolygons &withdrawn_components,
    const std::vector<std::vector<size_t>> &component_sources, const work::ExPolygons &platter_free_components,
    const work::ExPolygons &other_obstacles, const TravelZoneView &zones, const travel::TravelEndpoint &endpoint,
    const work::Point &point, const bool include_attractor, const plugin_run_context *context)
{
    std::vector<Attachment> candidates;
    const layer_handle *layer = include_attractor ? endpoint.region_island.region(0).layer().handle() : nullptr;
    // Compare all eligible components before testing the exit; a blocked nearest point is not replaced.
    for (const DomainType type : {DomainType::Attractor, DomainType::Platter}) {
        if (type == DomainType::Attractor && !include_attractor) continue;
        const work::ExPolygons &components = type == DomainType::Attractor ?
            withdrawn_components : platter_free_components;
        std::optional<Attachment> best;
        double best_distance = (std::numeric_limits<double>::max)();
        for (size_t component = 0; component < components.size(); ++component) {
            throw_if_cancelled(context);
            if (type == DomainType::Attractor && std::none_of(component_sources[component].begin(),
                    component_sources[component].end(), [&](const size_t index) {
                        return matches_source(zones, index, layer, endpoint, point);
                    })) continue;
            const work::Point q = components[component].contains(point, true) ?
                point : nearest_point(components[component], point);
            const double distance = point.distance_to(q);
            if (distance < best_distance) {
                best = Attachment{type, component, q, {point, q}, true};
                best_distance = distance;
            }
        }
        if (best && (point == best->ring_point || !blocked(best->link, other_obstacles, context)))
            candidates.push_back(std::move(*best));
    }
    return candidates;
}

bool matches_source(const TravelZoneView &zones, const size_t index, const layer_handle *layer,
    const travel::TravelEndpoint &endpoint, const work::Point &point)
{
    const TravelZone &zone = zones.zone(index);
    return zone.source_layer == layer && zone.scope.printing_catalog_id == endpoint.printing_catalog_id &&
        contains(zones.zone_area(index), point, true);
}

bool outside_part(const travel::TravelEndpoint &endpoint)
{
    return !endpoint.region_island.valid() || endpoint.region_island.region_count() == 0 ||
        AvoidCrossing::is_off_object_layer(endpoint.region_island.region(0).layer());
}

void append_component_path(work::Points &points, AvoidCrossing::Boundary &boundary, const work::Point &from,
    const work::Point &to, const plugin_run_context *context)
{
    if (from.coincides_with_epsilon(to)) {
        points.push_back(from);
        points.push_back(to);
    } else {
        AvoidCrossing::RouterCounts counts;
        const work::Points interior = AvoidCrossing::route_on_boundary(boundary, from, to, 0, 0, false, 0.,
            context->plugin_storage, counts, [context]() { throw_if_cancelled(context); }).points;
        points.insert(points.end(), interior.begin(), interior.end());
    }
}

bool contains(const work::ExPolygons &areas, const work::Point &point, const bool border)
{
    for (const work::ExPolygon &area : areas)
        if (area.contains(point, border)) return true;
    return false;
}

bool same_domain(const Attachment &source, const Attachment &target)
{
    return source.type == target.type && source.component == target.component;
}

bool touches(const work::ExPolygon &first, const work::ExPolygon &second, const plugin_run_context *context)
{
    if (first.contour.points.empty() || second.contour.points.empty()) return false;
    throw_if_cancelled(context);
    // Disjoint outer boxes cannot contain a filled overlap or a ring contact.
    const work::BoundingBox first_box = first.bounding_box(), second_box = second.bounding_box();
    if (first_box.max.x() < second_box.min.x() || first_box.min.x() > second_box.max.x() ||
        first_box.max.y() < second_box.min.y() || first_box.min.y() > second_box.max.y()) return false;
    if (overlaps_area(first, second, context)) return true;
    const auto touches_ring = [](const work::Points &ring, const work::Polygon &other) {
        for (const work::Point &point : ring)
            if (work::contains(other, point, true) && !work::contains(other, point, false)) return true;
        return false;
    };
    const auto touches_any_ring = [&touches_ring](const work::Points &ring, const work::ExPolygon &area) {
        if (touches_ring(ring, area.contour)) return true;
        for (const work::Polygon &hole : area.holes)
            if (touches_ring(ring, hole)) return true;
        return false;
    };
    if (touches_any_ring(first.contour.points, second) || touches_any_ring(second.contour.points, first)) return true;
    for (const work::Polygon &hole : first.holes)
        if (touches_any_ring(hole.points, second)) return true;
    for (const work::Polygon &hole : second.holes)
        if (touches_any_ring(hole.points, first)) return true;
    return false;
}

bool overlaps_area(const work::ExPolygon &first, const work::ExPolygon &second, const plugin_run_context *context)
{
    if (first.contour.points.empty() || second.contour.points.empty()) return false;
    throw_if_cancelled(context);
    return !clipper::intersection_ex(context->plugin_storage, {first}, {second}).empty();
}

work::Point nearest_point(const work::ExPolygon &component, const work::Point &point)
{
    work::Point nearest = point;
    double best = std::numeric_limits<double>::max();
    for (size_t ring = 0; ring <= component.holes.size(); ++ring) {
        const work::Points &vertices = ring == 0 ? component.contour.points : component.holes[ring - 1].points;
        for (size_t edge = 0; edge < vertices.size(); ++edge) {
            work::Point candidate;
            const double distance = work::Line(vertices[edge], vertices[(edge + 1) % vertices.size()]).distance_to_squared(point, &candidate);
            if (distance < best) { best = distance; nearest = candidate; }
        }
    }
    return nearest;
}

double estimate_cost(const std::vector<EstimateSegment> &segments, const work::ExPolygons &domain, const double weight,
    const plugin_run_context *context)
{
    double cost = 0.;
    for (const EstimateSegment &segment : segments) {
        throw_if_cancelled(context);
        if (segment.from == segment.to) continue;
        const double length = segment.from.distance_to(segment.to);
        const double outside = segment.kind == EstimateKind::Inside ? 0. :
            outside_length({segment.from, segment.to}, domain, context);
        cost += outside + weight * (length - outside);
    }
    return cost;
}

double outside_length(const work::Points &points, const work::ExPolygons &domain, const plugin_run_context *context)
{
    work::Polyline path;
    path.points = points;
    if (domain.empty()) return path.length();
    const work::Polylines outside = clipper::diff_pl(context->plugin_storage, {path}, domain);
    double length = 0.;
    for (const work::Polyline &part : outside) length += part.length();
    return length;
}

work::Points detour(const work::Point &from, const work::Point &to, const work::ExPolygons &rings,
                    const work::ExPolygon &platter, const plugin_run_context *context)
{
    if (rings.empty() || from == to) return {from, to};
    throw_if_cancelled(context);
    const auto on_platter_border = [&platter](const work::Point &point) {
        for (size_t edge = 0; edge < platter.contour.points.size(); ++edge)
            if (work::Line(platter.contour.points[edge],
                           platter.contour.points[(edge + 1) % platter.contour.points.size()])
                    .distance_to_squared(point) <= double(SCALED_EPSILON * SCALED_EPSILON))
                return true;
        return false;
    };
    const bool from_on_border = on_platter_border(from), to_on_border = on_platter_border(to);
    const work::Vec2d delta = work::to_vec2d(to - from);
    const work::Vec2d step = double(SCALED_EPSILON) * delta / delta.norm();
    const work::Point before(
        coord_t(std::round(double(from.x()) - step.x())), coord_t(std::round(double(from.y()) - step.y())));
    const work::Point after(
        coord_t(std::round(double(to.x()) + step.x())), coord_t(std::round(double(to.y()) + step.y())));
    const bool add_before = !from_on_border && before != from;
    const bool add_after = !to_on_border && after != to;
    slic3r_api::StoredPolyline subject(context->plugin_storage);
    const auto append_subject = [&subject](const work::Point &point) {
        const c_point value{point.x(), point.y()};
        if (subject.empty() || subject.back() != value) subject.push_back(value);
    };
    if (add_before) append_subject(before);
    append_subject(from);
    append_subject(to);
    if (add_after) append_subject(after);
    slic3r_api::StoredExPolygonCollection stored_rings = clipper::to_stored_expolygons(context->plugin_storage, rings);
    slic3r_api::ClipperOperand operand(context->plugin_storage, stored_rings.readonly());
    const slic3r_api::StoredPolyline result = slic3r_api::clipper_detour_polyline_expolygons(
        context->plugin_storage, subject, operand, CLIPPER_DETOUR_SHORTEST);
    throw_if_cancelled(context);
    // Remove only the temporary extension points, then restore exact endpoints.
    work::Points points;
    for (uint32_t index = 0; index < result.size(); ++index) points.emplace_back(result.at(index));
    if (add_before && !points.empty() && points.front() == before) points.erase(points.begin());
    if (add_after && !points.empty() && points.back() == after) points.pop_back();
    if (points.empty() || points.front() != from) points.insert(points.begin(), from);
    else points.front() = from;
    if (points.back() != to) points.push_back(to);
    else points.back() = to;
    return points;
}

work::Points detour_in_component(const work::Point &from, const work::Point &to,
                                  const work::ExPolygon &component, const plugin_run_context *context)
{
    if (from == to) return {from, to};
    throw_if_cancelled(context);
    slic3r_api::StoredPolyline subject(context->plugin_storage);
    subject.push_back({from.x(), from.y()});
    subject.push_back({to.x(), to.y()});
    slic3r_api::StoredExPolygonCollection rings = clipper::to_stored_expolygons(
        context->plugin_storage, {component});
    slic3r_api::ClipperOperand operand(context->plugin_storage, rings.readonly());
    const slic3r_api::StoredPolyline result = slic3r_api::clipper_detour_polyline_expolygons(
        context->plugin_storage, subject, operand, CLIPPER_DETOUR_SHORTEST);
    throw_if_cancelled(context);
    work::Points points;
    for (uint32_t index = 0; index < result.size(); ++index)
        points.emplace_back(result.at(index));
    return points;
}

bool blocked(const work::Points &points, const work::ExPolygons &areas, const plugin_run_context *context)
{
    if (areas.empty()) return false;
    throw_if_cancelled(context);
    work::Polyline path;
    path.points = points;
    const work::Polylines outside = clipper::diff_pl(context->plugin_storage, {path}, areas);
    double outside_length = 0.;
    for (const work::Polyline &part : outside) outside_length += part.length();
    throw_if_cancelled(context);
    return path.length() - outside_length > SCALED_EPSILON;
}
}
}}}
