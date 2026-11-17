///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

#include "ZonePlanner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "../../Common/ClipperHelpers.hpp"
#include "../../ElephantFootCompensation.hpp"
#include "../AvoidCrossing/LayerInputCache.hpp"

/*
ZonePlanner prepares one cached geometry entry per active zone/tool/endpoint context.
It subtracts expanded forbidden material from active attractors to form M, withdraws
each M component into W rings, and adds translated printed platter components when an
endpoint is outside a normal object layer. Endpoint provenance selects an M component;
other endpoints use only the obstacle-only candidate. Hypotheses share their selected
attachments between stable cost estimates and construction.
The obstacle-only candidate follows expanded rings, including holes. An in-part
no-domain candidate must stay in its M component within SCALED_EPSILON. Route
construction keeps exact endpoints. Validation avoids original material, respects the
complete-path budget and, when the reference avoids original material, stays within
its outside length plus SCALED_EPSILON. That reference length is tripled for an endpoint strictly inside
expanded material; a blocked reference removes only the no-domain candidate.
Rejected hypotheses leave the next ranked choice available. If none passes, the straight
path is returned and counted as one fallback.
Accepted domain paths count as domain routes, inter-component paths count as bridges,
and paths longer than their chord by SCALED_EPSILON count as detours.
Endpoint membership removes individual unexpanded zones before union and nozzle
expansion. Outside-part travels also use translated normal slices from the plan at the
current group Z or below; the synthetic wipe tower is not a blocker. Components
containing either endpoint are ignored individually. The plan catalog, object slices
and all other borrowed inputs remain unchanged and outlive this worker-local cache.

Obstacle-only references and external connectors use the public
`clipper_detour_polyline_expolygons` operation in Shortest mode; its ring-following
and rounding rules are defined in `slic3r_clipper.h`. ZonePlanner extends only
external connector chords before the call so a crossing at an endpoint is visible
when the chord leaves a ring into its interior or arrives from its interior. The
M-to-W link uses the same operation without that extension. Only leading or
trailing result points near an endpoint are replaced by that exact endpoint. The
original-area check compares path length with its Clipper difference, and the total
detour budget rejects unsafe hypotheses; exhaustion produces the counted straight
fallback. Cancellation propagates.

    path: classify endpoints, cache one geometry entry and rank hypotheses once
    |-- LayerInputCache::get: resolve spacing and island-jump settings
    |-- build_boundary: index W for each component and, when needed, its combined boundary
    |-- append_component_path: route within one component between selected points
    |   `-- route_on_boundary: follow crossed rings, else keep the chord; no placement or island jumps
    |-- eligible: resolve regional and auxiliary activation
    |-- outside_part / is_off_object_layer: classify normal versus auxiliary provenance
    |-- platter_blockers: collect translated slices for outside-part and mixed travels
    |-- contains: test filled material, including its border
    |-- endpoint_candidates: match source provenance and choose the option without a domain
    |   |-- nearest_point: find a deterministic W attachment
    |   `-- detour_in_component: build a link from the endpoint to W inside M
    |-- select_island_bridge: estimate weighted approaches between components
    |-- estimate_cost: weight inside ring estimates and clip the other connectors
    |   `-- outside_length: measure outside connectors against the complete domain
    |-- detour: extend a connector chord and call the public Clipper ring-detour operation
    |-- detour_in_component: call the same operation without extending the M-to-W link
    |-- blocked: compare the complete path with its Clipper difference from original material
    `-- outside_length: clip each route against the cached domain for the reference bound
*/
namespace slic3r_api { namespace LayerExtrusionEdit { namespace AdvancedTravel {
namespace work = PluginCommon::polygon_work;
namespace clipper = PluginCommon::clipper;
namespace travel = TravelConnection;

namespace {
// SIZE_MAX denotes the option without a domain. For an M component, point is the
// exact endpoint, ring_point is on W, and link joins the two through M.
struct Attachment {
    size_t component = SIZE_MAX;
    work::Point point;
    work::Point ring_point;
    work::Points link;
    bool usable = true;
};
// Connector endpoints are shared by estimation and construction so both phases use the same points.
struct Hypothesis { Attachment source, target; work::Point exit, entry; double cost; };
// Append a component's route between already attached points (crossed rings, else a direct chord), retaining exact endpoints and borrowing its cached boundary.
// Epsilon-near points connect directly; no placement or nearby-ring search occurs. Update routing counts; cancellation propagates.
void append_component_path(work::Points &points, AvoidCrossing::Boundary &boundary, const work::Point &from,
    const work::Point &to, const plugin_run_context *context, AvoidCrossing::RouterCounts &counts);
// Inside estimates stand for a route within a component; Outside retains geometric domain intersections.
enum class EstimateKind { Inside, Outside };
struct EstimateSegment { work::Point from, to; EstimateKind kind; };
// Classify regionless endpoints and endpoints on auxiliary layers as outside the part; normal object layers remain inside.
bool outside_part(const travel::TravelEndpoint &endpoint);
// Find the nearest point on any component ring, retaining ring/edge order on equal distances.
work::Point nearest_point(const work::ExPolygon &component, const work::Point &point);
// Return the M component whose source layer, instance and raw slice contain the endpoint, plus the option without a domain.
// Keep the component candidate when its M-to-W link is unusable so admission of the option without a domain still sees physical M membership.
std::vector<Attachment> endpoint_candidates(const work::ExPolygons &components,
    const work::ExPolygons &withdrawn_components, const std::vector<std::vector<size_t>> &component_sources,
    const TravelZoneView &zones, const travel::TravelEndpoint &endpoint, const work::Point &point,
    const plugin_run_context *context);
// Sum segment lengths after clipping to the complement of the domain; an empty domain leaves the whole path outside.
double outside_length(const work::Points &points, const work::ExPolygons &domain, const plugin_run_context *context);
// Score inside estimates as weighted inside length; clip other connectors against all domains before weighting their inside portions.
// Zero-length segments cost nothing. Cancellation propagates per segment.
double estimate_cost(const std::vector<EstimateSegment> &segments, const work::ExPolygons &domain, double weight, const plugin_run_context *context);
// Test filled areas without changing them; holes exclude their interior and borders follow the supplied flag.
bool contains(const work::ExPolygons &areas, const work::Point &point, bool border = true);
// Extend a chord by one scaled epsilon at both ends, call Clipper's shortest ring detour, and restore exact endpoints.
// The extension makes endpoint crossings visible without turning tangencies or ring-following into crossings; only leading or trailing points of the result's interior are replaced by the exact endpoint.
// A reference endpoint inside or on expanded material can pair an exit with a later entry; original material still validates it. Cancellation and API errors propagate.
work::Points detour(const work::Point &from, const work::Point &to, const work::ExPolygons &expanded,
                    const plugin_run_context *context);
// Call the Clipper detour operation on one M component without extending its endpoints; return its owned points.
// The exact endpoint contact is intentionally ignored by the API, then checked by the caller as a complete link.
work::Points detour_in_component(const work::Point &from, const work::Point &to, const work::ExPolygon &component,
                                 const plugin_run_context *context);
// Return whether the path loses more than the scaled tolerance inside filled original material.
// An empty mask is never blocked; cancellation and Clipper errors propagate.
bool blocked(const work::Points &points, const work::ExPolygons &areas, const plugin_run_context *context);

} // namespace

ZonePlanner::ZonePlanner(AvoidCrossing::CompatibilityPlanner &compatibility, TravelZoneView &zones,
                         const Config &print_config, const bool auxiliary_avoidance, const plugin_run_context *context,
                         const Print &print, AvoidCrossing::LayerInputCache &layer_inputs,
                         const PrintingPlan &plan, const uint32_t group_idx, const coord_t print_z) :
    m_compatibility(compatibility), m_zones(zones), m_config(print_config),
    m_auxiliary_avoidance(auxiliary_avoidance), m_context(context), m_print(print), m_layer_inputs(layer_inputs),
    m_plan(plan), m_group_idx(group_idx), m_print_z(print_z)
{}

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

// Build or reuse one geometry entry for travel t, return the first accepted route or the straight fallback, and update route counters; cancellation and geometry failures propagate.
std::vector<c_point> ZonePlanner::path(const size_t t, const PendingTravel &pending)
{
    throw_if_cancelled(m_context);
    const std::vector<c_point> direct = travel::straight_path(pending.source, pending.target, pending.extruder_id);
    if (!eligible(pending.source, pending.target)) return direct;
    const work::Point source(pending.source.position.x, pending.source.position.y);
    const work::Point target(pending.target.position.x, pending.target.position.y);
    if (source.coincides_with_epsilon(target)) return direct;
    const bool source_outside = outside_part(pending.source), target_outside = outside_part(pending.target);
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
    std::vector<size_t> ignored;
    for (size_t index : active.forbidden_indexes) {
        throw_if_cancelled(m_context);
        const work::ExPolygons &area = m_zones.zone_area(index);
        if (contains(area, source) || contains(area, target)) ignored.push_back(index);
    }
    std::vector<size_t> ignored_platter;
    std::optional<PlatterKey> platter_key;
    if (use_platter) {
        if (!m_platter_ready) {
            m_platter_blockers = AvoidCrossing::platter_blockers(m_context->plugin_storage, m_plan, m_group_idx, m_print_z);
            throw_if_cancelled(m_context);
            m_platter_ready = true;
        }
        for (size_t component = 0; component < m_platter_blockers.size(); ++component) {
            throw_if_cancelled(m_context);
            if (m_platter_blockers[component].contains(source) || m_platter_blockers[component].contains(target))
                ignored_platter.push_back(component);
        }
        platter_key = PlatterKey(m_group_idx, m_print_z, ignored_platter);
    }
    throw_if_cancelled(m_context);
    const CacheKey key(active.id, pending.extruder_id, ignored, spacing, platter_key);
    std::map<CacheKey, Obstacles>::iterator cached = m_obstacles.find(key);
    if (cached == m_obstacles.end()) {
        Obstacles obstacles;
        for (size_t index : active.forbidden_indexes) {
            throw_if_cancelled(m_context);
            if (std::binary_search(ignored.begin(), ignored.end(), index)) continue;
            const work::ExPolygons &area = m_zones.zone_area(index);
            for (const work::ExPolygon &component : area) obstacles.original.push_back(component);
        }
        if (use_platter)
            for (size_t component = 0; component < m_platter_blockers.size(); ++component) {
                throw_if_cancelled(m_context);
                if (!std::binary_search(ignored_platter.begin(), ignored_platter.end(), component))
                    obstacles.original.push_back(m_platter_blockers[component]);
            }
        if (!obstacles.original.empty()) {
            obstacles.original = clipper::union_ex(m_context->plugin_storage, obstacles.original);
            const coord_t radius = std::max<coord_t>(1, scale_i(nozzle_diameter * 0.5));
            obstacles.expanded = clipper::offset_ex(m_context->plugin_storage, obstacles.original, double(radius));
        }
        work::ExPolygons domain = active.attractors;
        if (!obstacles.expanded.empty())
            domain = clipper::diff_ex(m_context->plugin_storage, domain, obstacles.expanded);
        if (!domain.empty()) {
            obstacles.components = domain;
            obstacles.component_sources.resize(domain.size());
            for (size_t component = 0; component < domain.size(); ++component) {
                throw_if_cancelled(m_context);
                for (const size_t source_index : active.attractor_indexes) {
                    throw_if_cancelled(m_context);
                    if (!clipper::intersection_ex(m_context->plugin_storage, m_zones.zone_area(source_index),
                                                  {domain[component]}).empty())
                        obstacles.component_sources[component].push_back(source_index);
                }
            }
            // W uses min(0.01 mm, nozzle/10) and withdraws 1.5 perimeter spacings from each M component.
            const double minimum_width = std::min(0.01, nozzle_diameter / 10.);
            const coord_t withdrawal = coord_t(1.5 * double(spacing));
            for (const work::ExPolygon &component : domain) {
                throw_if_cancelled(m_context);
                const StoredExPolygonCollection stored_component = clipper::to_stored_expolygons(
                    m_context->plugin_storage, {component});
                const slic3r_api::ExPolygon source = stored_component.readonly().at(0);
                const StoredExPolygonCollection withdrawn =
                    XYSizeCompensationPlugin::detail::elephant_foot_compensation(
                        m_context->plugin_storage, source, minimum_width, withdrawal);
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
        }
        throw_if_cancelled(m_context);
        cached = m_obstacles.emplace(key, std::move(obstacles)).first;
    }
    const Obstacles &obstacles = cached->second;
    const auto free_attachment = [](const work::Point &point) {
        return Attachment{SIZE_MAX, point, point, {}, true};
    };
    const std::vector<Attachment> sources = source_outside ?
        std::vector<Attachment>{free_attachment(source)} :
        endpoint_candidates(obstacles.components, obstacles.withdrawn_components, obstacles.component_sources,
                            m_zones, pending.source, source, m_context);
    const std::vector<Attachment> targets = target_outside ?
        std::vector<Attachment>{free_attachment(target)} :
        endpoint_candidates(obstacles.components, obstacles.withdrawn_components, obstacles.component_sources,
                            m_zones, pending.target, target, m_context);
    std::vector<Hypothesis> hypotheses;
    for (const Attachment &from : sources)
        for (const Attachment &to : targets) {
            throw_if_cancelled(m_context);
            if ((from.component != SIZE_MAX && !from.usable) || (to.component != SIZE_MAX && !to.usable)) continue;
            // The no-domain option for one M component is admitted only when its chord remains in that component.
            if (from.component == SIZE_MAX && to.component == SIZE_MAX &&
                !source_outside && !target_outside && sources.front().component != SIZE_MAX &&
                sources.front().component == targets.front().component &&
                outside_length({source, target}, {obstacles.components[sources.front().component]}, m_context) > SCALED_EPSILON)
                continue;
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
            work::Point exit = from.component != SIZE_MAX ? from.ring_point : source;
            work::Point entry = to.component != SIZE_MAX ? to.ring_point : target;
            if (from.component != SIZE_MAX && to.component != SIZE_MAX && from.component == to.component) {
                append_estimate(from.link, EstimateKind::Inside);
                estimate.push_back({from.ring_point, to.ring_point, EstimateKind::Inside});
                append_estimate(to.link, EstimateKind::Inside, true);
            } else if (from.component != SIZE_MAX && to.component != SIZE_MAX) {
                AvoidCrossing::RouterCounts estimate_counts;
                work::Point start = from.ring_point, end = to.ring_point;
                const std::optional<AvoidCrossing::IslandBridge> bridge = AvoidCrossing::select_island_bridge(
                    *obstacles.domain, obstacles.component_rings[from.component], obstacles.component_rings[to.component],
                    start, end, jump_weight, estimate_counts,
                    [this]() { throw_if_cancelled(m_context); });
                if (!bridge) continue;
                exit = bridge->from;
                entry = bridge->to;
                append_estimate(from.link, EstimateKind::Inside);
                estimate.push_back({from.ring_point, exit, EstimateKind::Inside});
                estimate.push_back({exit, entry, EstimateKind::Outside});
                estimate.push_back({entry, to.ring_point, EstimateKind::Inside});
                append_estimate(to.link, EstimateKind::Inside, true);
            } else if (from.component != SIZE_MAX) {
                exit = nearest_point(obstacles.withdrawn_components[from.component], target);
                append_estimate(from.link, EstimateKind::Inside);
                estimate.push_back({from.ring_point, exit, EstimateKind::Inside});
                estimate.push_back({exit, target, EstimateKind::Outside});
            } else if (to.component != SIZE_MAX) {
                entry = nearest_point(obstacles.withdrawn_components[to.component], source);
                estimate.push_back({source, entry, EstimateKind::Outside});
                estimate.push_back({entry, to.ring_point, EstimateKind::Inside});
                append_estimate(to.link, EstimateKind::Inside, true);
            } else {
                estimate.push_back({source, target, EstimateKind::Outside});
            }
            hypotheses.push_back({from, to, exit, entry,
                                  estimate_cost(estimate, obstacles.components, jump_weight, m_context)});
        }
    // Domain choices precede the option without a domain on equal estimates.
    std::stable_sort(hypotheses.begin(), hypotheses.end(),
                     [](const Hypothesis &a, const Hypothesis &b) { return a.cost < b.cost; });

    // Build the obstacle-only reference before trying hypotheses so every candidate uses one bound.
    const work::Points reference = detour(source, target, obstacles.expanded, m_context);
    const bool reference_usable = !blocked(reference, obstacles.original, m_context);
    double reference_outside = 0.;
    if (reference_usable) {
        reference_outside = outside_length(reference, obstacles.components, m_context);
        if (contains(obstacles.expanded, source, false) || contains(obstacles.expanded, target, false))
            reference_outside *= 3.;
    }
    const c_float_or_percent budget = m_config.float_or_percent_or_default(
        "avoid_crossing_perimeters_max_detour", c_float_or_percent{0., 0});
    const double direct_length = work::Line(source, target).length();
    for (const Hypothesis &hypothesis : hypotheses) {
        throw_if_cancelled(m_context);
        const Attachment &from = hypothesis.source, &to = hypothesis.target;
        const bool domain_route = from.component != SIZE_MAX || to.component != SIZE_MAX;
        work::Points points;
        const auto append_route = [&points](const work::Points &path, const bool reverse = false) {
            if (!reverse)
                points.insert(points.end(), path.begin(), path.end());
            else
                for (work::Points::const_reverse_iterator point = path.rbegin(); point != path.rend(); ++point)
                    points.push_back(*point);
        };
        if (!domain_route) {
            if (!reference_usable) continue;
            points = reference;
        } else if (from.component == to.component) {
            points.push_back(source);
            append_route(from.link);
            append_component_path(points, *obstacles.component_boundaries[from.component], from.ring_point, to.ring_point,
                                  m_context, m_router_counts);
            append_route(to.link, true);
            points.push_back(target);
        } else {
            points.push_back(source);
            if (from.component != SIZE_MAX) {
                append_route(from.link);
                append_component_path(points, *obstacles.component_boundaries[from.component], from.ring_point,
                                      hypothesis.exit, m_context, m_router_counts);
            }
            const work::Point connector_from = from.component != SIZE_MAX ? hypothesis.exit : source;
            const work::Point connector_to = to.component != SIZE_MAX ? hypothesis.entry : target;
            const work::Points connector = detour(connector_from, connector_to, obstacles.expanded, m_context);
            points.insert(points.end(), connector.begin(), connector.end());
            if (to.component != SIZE_MAX) {
                append_component_path(points, *obstacles.component_boundaries[to.component], hypothesis.entry,
                                      to.ring_point, m_context, m_router_counts);
                append_route(to.link, true);
            }
            points.push_back(target);
        }
        points.erase(std::unique(points.begin(), points.end()), points.end());
        if (blocked(points, obstacles.original, m_context)) continue;
        double length = 0.;
        for (size_t i = 1; i < points.size(); ++i) length += work::Line(points[i - 1], points[i]).length();
        if (budget.value > 0.) {
            const double limit = budget.percent ? direct_length * 0.01 * budget.value : scale_d(budget.value);
            if (length - direct_length > limit) continue;
        }
        // Clipping rounds intersection points: allow one SCALED_EPSILON over the complete path.
        if (reference_usable && outside_length(points, obstacles.components, m_context) > reference_outside + SCALED_EPSILON)
            continue;
        std::vector<c_point> result;
        for (const work::Point &point : points) result.push_back({point.x(), point.y()});
        if (domain_route) ++m_domain_routes;
        if (from.component != SIZE_MAX && to.component != SIZE_MAX && from.component != to.component) ++m_bridges;
        if (length > direct_length + SCALED_EPSILON) ++m_detours;
        return result;
    }
    ++m_fallbacks;
    return direct;
}

namespace {
std::vector<Attachment> endpoint_candidates(const work::ExPolygons &components,
    const work::ExPolygons &withdrawn_components, const std::vector<std::vector<size_t>> &component_sources,
    const TravelZoneView &zones, const travel::TravelEndpoint &endpoint, const work::Point &point,
    const plugin_run_context *context)
{
    std::vector<Attachment> candidates;
    const layer_handle *layer = endpoint.region_island.valid() && endpoint.region_island.region_count() != 0 ?
        endpoint.region_island.region(0).layer().handle() : nullptr;
    for (size_t component = 0; component < components.size(); ++component) {
        throw_if_cancelled(context);
        if (!components[component].contains(point, true)) continue;
        bool source_matches = false;
        for (const size_t index : component_sources[component]) {
            const TravelZone &zone = zones.zone(index);
            if (zone.source_layer != layer || zone.scope.printing_catalog_id != endpoint.printing_catalog_id)
                continue;
            if (contains(zones.zone_area(index), point, true)) {
                source_matches = true;
                break;
            }
        }
        if (!source_matches) continue;
        Attachment selected{component, point, point, {}, true};
        if (!withdrawn_components[component].contains(point, true)) {
            selected.ring_point = nearest_point(withdrawn_components[component], point);
            selected.link = detour_in_component(point, selected.ring_point, components[component], context);
            selected.usable = selected.link.size() >= 2 && selected.link.front() == point &&
                selected.link.back() == selected.ring_point &&
                outside_length(selected.link, {components[component]}, context) <= SCALED_EPSILON;
        }
        candidates.push_back(std::move(selected));
        break;
    }
    candidates.push_back({SIZE_MAX, point, point, {}, true});
    return candidates;
}

bool outside_part(const travel::TravelEndpoint &endpoint)
{
    return !endpoint.region_island.valid() || endpoint.region_island.region_count() == 0 ||
        AvoidCrossing::is_off_object_layer(endpoint.region_island.region(0).layer());
}

void append_component_path(work::Points &points, AvoidCrossing::Boundary &boundary, const work::Point &from,
    const work::Point &to, const plugin_run_context *context, AvoidCrossing::RouterCounts &counts)
{
    if (from.coincides_with_epsilon(to)) {
        points.push_back(from);
        points.push_back(to);
    } else {
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

work::Points detour(const work::Point &from, const work::Point &to, const work::ExPolygons &expanded,
                    const plugin_run_context *context)
{
    if (expanded.empty() || from == to) return {from, to};
    throw_if_cancelled(context);
    const work::Vec2d delta = work::to_vec2d(to - from);
    const work::Vec2d step = double(SCALED_EPSILON) * delta / delta.norm();
    work::Point before(coord_t(std::round(double(from.x()) - step.x())),
                       coord_t(std::round(double(from.y()) - step.y())));
    work::Point after(coord_t(std::round(double(to.x()) + step.x())),
                      coord_t(std::round(double(to.y()) + step.y())));
    slic3r_api::StoredPolyline subject(context->plugin_storage);
    subject.push_back({before.x(), before.y()});
    subject.push_back({from.x(), from.y()});
    subject.push_back({to.x(), to.y()});
    subject.push_back({after.x(), after.y()});
    slic3r_api::StoredExPolygonCollection rings = clipper::to_stored_expolygons(context->plugin_storage, expanded);
    slic3r_api::ClipperOperand operand(context->plugin_storage, rings.readonly());
    const slic3r_api::StoredPolyline result = slic3r_api::clipper_detour_polyline_expolygons(
        context->plugin_storage, subject, operand, CLIPPER_DETOUR_SHORTEST);
    throw_if_cancelled(context);
    // Remove points created only on the temporary endpoint extensions, then restore exact endpoints.
    const double endpoint_margin = double(SCALED_EPSILON + 2);
    size_t first = 1, last = result.size() - 1;
    while (first < last && work::to_vec2d(work::Point(result.at(first)) - from).norm() <= endpoint_margin) ++first;
    while (first < last && work::to_vec2d(work::Point(result.at(last - 1)) - to).norm() <= endpoint_margin) --last;
    work::Points points{from};
    for (size_t index = first; index < last; ++index) points.emplace_back(result.at(index));
    points.push_back(to);
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
