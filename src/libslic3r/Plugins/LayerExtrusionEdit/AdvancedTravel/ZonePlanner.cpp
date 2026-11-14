///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

#include "ZonePlanner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "../../Common/ClipperHelpers.hpp"
#include "../AvoidCrossing/LayerInputCache.hpp"

/*
ZonePlanner supplies ranked XY routes on active attractors minus expanded forbidden
material. A normal object-layer endpoint chooses its containing component, or the
nearest within 1.5 perimeter spacings, and also offers an option without a domain.
Regionless endpoints and endpoints on auxiliary layers (support, raft, skirt, brim
and tower) are outside the part and offer only the option without a domain. An attachment
is the endpoint itself when contained, otherwise the nearest ring point. Stable
ranking compares estimated outside length plus weighted inside length. The layer's
island-travel weight applies only when island avoidance is active; otherwise it is zero.
An inside estimate represents a route within one component, so its chord
length is weighted as wholly inside even when that chord spans a void. Other
estimate connectors use their actual clipped inside and outside portions.
Endpoint attachments within 1.5 spacings are also estimated as inside material;
bridges and mixed entry/exit legs retain clipped pricing.
Legs adjacent to an outside-part endpoint retain their geometric length without
the inside weight or an outside-domain penalty in the estimate.
Same-component paths use only that component: the chord between its attachments
follows the component's rings only where it crosses them.
Different-component paths join each component's route at bridge points selected
on the combined boundary, then contour expanded obstacles along the bridge. A mixed
choice routes within its component to the ring point nearest the free endpoint, then
contours obstacles to that endpoint. Selected connector points are retained with
the hypothesis so estimates and constructed routes use the same points.
The option without either domain follows the chord around expanded obstacles.
For an in-part travel whose endpoints have the same domain candidate, this option is admitted only if its
chord stays within the material proxy: components grown by 1.5 perimeter spacings,
cached once with the components. Outside length up to SCALED_EPSILON is allowed.
Each constructed hypothesis must retain exact endpoints, avoid original forbidden
material and respect the total detour budget. Its length outside the domain cannot
exceed that of the obstacle-only chord detour by more than SCALED_EPSILON, the
complete-path allowance for integer clipping. The reference detour is computed
at most once per active blocker set; an unusable reference skips only this outside-length bound.
Rejected hypotheses leave the next ranked choice available. After exhaustion with no
geometrically valid hypothesis, disable the remaining original component first crossed
along the chord, or, when the chord is clear, along the initial best-ranked estimate, and retry with rebuilt
geometry. Equal entry parameters prefer zone order, then component order, then platter
components. Each component can be disabled once per travel; the count crossed by the
initial chord or best estimate bounds deactivations. Budget-only and outside-length
rejections do not authorize removal. Disabled material is excluded from validation, and cache keys
retain disabled zone/component pairs or ignored platter identities. No crossed blocker
or exhaustion of the component bound counts
a straight fallback; accepted choices using a domain count once as domain routes,
accepted choices connecting distinct components count as bridges, and accepted
paths longer than the chord by SCALED_EPSILON count as detours.
Endpoint membership removes individual unexpanded zones before union and nozzle
expansion, so overlapping margins do not erase unrelated obstacles. Cached areas
also include platter blockers for travels touching outside-part endpoints: translated
normal slices of the plan's objects at the layer-group Z or below, from printing
groups up to this group. The synthetic wipe tower is not collected as a blocker.
Platter components containing either endpoint are ignored individually, leaving
an in-part endpoint's own component reachable on a mixed travel. The borrowed plan's
catalog and object slices stay unchanged during this planner's lifetime. Platter
slices are collected once; cache keys include their group, Z and ignored component
indexes only for travels using them. The remaining geometry key retains active
zones, tool, ignored zones and spacing. All cached areas
belong to this worker. The Config view of print settings is copied, its configuration borrowed;
the auxiliary avoidance flag is resolved from the default region configuration
and governs travels whose two endpoints have no valid object instance (skirt and brim).
The other inputs are borrowed too; all borrowed data must outlive the planner.

Segment intersections split the filled area into open intervals. Midpoint tests
exclude tangencies, collinear contacts and holes. For each blocked segment, the
first component's first entry and last exit delimit the shorter contour arc.
Scanning resumes strictly after the exit: the remaining line may meet another
component. Progress retains the unrounded exit parameter on the original line;
only output vertices are rounded. Inserted arcs are not classified as entries. The
component bound and original-area validation reject unsafe obstacle candidates;
the total detour budget can reject either kind of route. Exhaustion produces
the counted straight fallback. A connector endpoint in a hole whose line crosses
the surrounding material cannot connect through the outer contour. An endpoint
inside the expanded margin can likewise lack a contour entry or exit. These
failures reject that connector; other hypotheses can still provide a valid route.
Cancellation propagates.

    path: filter zones, cache domains and retry ranked hypotheses after bounded blocker removal
    |-- LayerInputCache::get: resolve spacing and island-jump settings
    |-- build_boundary: index the combined domain and each component separately
    |-- append_component_path: route within one component between selected points
    |   `-- route_on_boundary: follow crossed rings, else keep the chord; no placement or island jumps
    |-- eligible: resolve regional and auxiliary activation
    |-- outside_part / is_off_object_layer: classify normal versus auxiliary provenance
    |-- platter_blockers: collect translated slices for outside-part and mixed travels
    |-- contains: test filled material, including its border
    |-- endpoint_candidates: choose a component and the option without a domain
    |   `-- nearest_point: find a deterministic ring attachment
    |-- select_island_bridge: estimate weighted approaches between components
    |-- estimate_cost: weight inside ring estimates and clip the other connectors
    |   `-- outside_length: measure outside connectors against the complete domain
    |-- detour_around_forbidden: connect the chord around expanded components
    |   |-- crossing: find first entry and last exit, ignoring contacts
    |   `-- contour_arc: walk each contour direction and choose the shorter
    |-- blocked: validate candidate segments against filled material
    |   `-- crossing: shared with detour_around_forbidden to classify material intervals
    `-- outside_length: clip each route against the cached domain for the reference bound
*/
namespace slic3r_api { namespace LayerExtrusionEdit { namespace AdvancedTravel {
namespace work = PluginCommon::polygon_work;
namespace clipper = PluginCommon::clipper;
namespace travel = TravelConnection;

namespace {
struct Hit { double parameter; work::Point point; size_t edge; };
// SIZE_MAX denotes the option without a domain; its attachment is the endpoint itself.
struct Attachment { size_t component; work::Point point; };
// Connector endpoints are shared by estimation and construction; estimate holds the endpoint/attachment/bridge polyline.
// The initial best hypothesis's estimate also drives blocker selection and the fixed deactivation cap.
struct Hypothesis { Attachment source, target; work::Point exit, entry; double cost; work::Points estimate; };
// Append a component's route between already attached points (crossed rings, else a direct chord), retaining exact endpoints and borrowing its cached boundary.
// Epsilon-near points connect directly; no placement or nearby-ring search occurs. Update routing counts; cancellation propagates.
void append_component_path(work::Points &points, AvoidCrossing::Boundary &boundary, const work::Point &from,
    const work::Point &to, const plugin_run_context *context, AvoidCrossing::RouterCounts &counts);
// Inside estimates stand for a route within a component; Outside retains geometric domain intersections; OutsidePart counts length without a domain penalty or weight.
enum class EstimateKind { Inside, Outside, OutsidePart };
struct EstimateSegment { work::Point from, to; EstimateKind kind; };
// Classify regionless endpoints and endpoints on auxiliary layers as outside the part; normal object layers remain inside.
bool outside_part(const travel::TravelEndpoint &endpoint);
// Find the nearest point on any component ring, retaining ring/edge order on equal distances.
work::Point nearest_point(const work::ExPolygon &component, const work::Point &point);
// Return the containing component, else the nearest within 1.5 spacings, followed by the option without a domain.
// Interior points attach to themselves; outside points attach to the nearest ring. Cancellation propagates per component.
std::vector<Attachment> endpoint_candidates(const work::ExPolygons &components, const work::Point &point,
    coord_t spacing, const plugin_run_context *context);
// Sum segment lengths after clipping to the complement of the domain; an empty domain leaves the whole path outside.
double outside_length(const work::Points &points, const work::ExPolygons &domain, const plugin_run_context *context);
// Score inside estimates as weighted inside length; clip other connectors against all domains before weighting their inside portions.
// Outside-part legs count their full length without domain penalties; zero-length segments cost nothing. Cancellation propagates per segment.
double estimate_cost(const std::vector<EstimateSegment> &segments, const work::ExPolygons &domain, double weight, const plugin_run_context *context);
// Follow shorter outer-contour arcs along the chord through expanded components; return both endpoints, even on failure.
// Set changed after inserting an arc and failed for a missing arc or exhausted component bound; do not validate original material or count routes. Cancellation propagates.
work::Points detour_around_forbidden(const work::Point &from, const work::Point &to,
    const work::ExPolygons &expanded, const plugin_run_context *context, bool &changed, bool &failed);
// Test filled areas without changing them; holes exclude their interior and borders follow the supplied flag.
bool contains(const work::ExPolygons &areas, const work::Point &point, bool border = true);
// Classify an unrounded point against one ring: -1 outside, 0 on an edge, 1 inside, with sub-unit floating-point tolerance.
int ring_location(const work::Polygon &ring, const work::Vec2d &point);
// Find the first interval whose exit is strictly after minimum_parameter, returning its entry and the last material exit; contacts do not count.
// Stable ties follow insertion order (endpoint seeds, then rings/edges); coincident hits keep their retained point/parameter and acquire a known outer edge when missing.
bool crossing(const work::ExPolygon &area, const work::Point &from, const work::Point &to,
              Hit &entry, Hit &exit, const plugin_run_context *context, double minimum_parameter = 0.);
// Return whether any nonzero path segment enters filled material; cancellation propagates per component.
bool blocked(const work::Points &points, const work::ExPolygons &areas, const plugin_run_context *context);
// Build both outer-contour arcs including the rounded hits, choosing the shorter; hole hits cannot produce a contour route.
work::Points contour_arc(const work::Polygon &contour, const Hit &entry, const Hit &exit,
                         const plugin_run_context *context);

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

std::vector<Attachment> endpoint_candidates(const work::ExPolygons &components, const work::Point &point,
    const coord_t spacing, const plugin_run_context *context)
{
    Attachment selected{SIZE_MAX, point};
    double best = std::pow(1.5 * double(spacing), 2);
    for (size_t component = 0; component < components.size(); ++component) {
        throw_if_cancelled(context);
        if (components[component].contains(point)) { selected = {component, point}; break; }
        const work::Point nearest = nearest_point(components[component], point);
        const double distance = work::to_vec2d(nearest - point).squaredNorm();
        if (distance < best) { best = distance; selected = {component, nearest}; }
    }
    std::vector<Attachment> candidates;
    if (selected.component != SIZE_MAX) candidates.push_back(selected);
    candidates.push_back({SIZE_MAX, point});
    return candidates;
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

work::Points detour_around_forbidden(const work::Point &from, const work::Point &to,
    const work::ExPolygons &expanded, const plugin_run_context *context, bool &changed, bool &failed)
{
    work::Points points{from};
    changed = false;
    failed = false;
    double progress = 0.;
    for (size_t iteration = 0; iteration <= expanded.size(); ++iteration) {
        size_t first = SIZE_MAX;
        Hit entry{}, exit{};
        entry.parameter = std::numeric_limits<double>::max();
        for (size_t component = 0; component < expanded.size(); ++component) {
            throw_if_cancelled(context);
            Hit a, b;
            if (crossing(expanded[component], from, to, a, b, context, progress) &&
                a.parameter < entry.parameter) { first = component; entry = a; exit = b; }
        }
        if (first == SIZE_MAX) break;
        if (iteration == expanded.size()) { failed = true; break; }
        const work::Points arc = contour_arc(expanded[first].contour, entry, exit, context);
        if (arc.empty()) { failed = true; break; }
        points.insert(points.end(), arc.begin(), arc.end());
        progress = exit.parameter;
        changed = true;
    }
    points.push_back(to);
    return points;
}

double estimate_cost(const std::vector<EstimateSegment> &segments, const work::ExPolygons &domain, const double weight,
    const plugin_run_context *context)
{
    double cost = 0.;
    for (const EstimateSegment &segment : segments) {
        throw_if_cancelled(context);
        if (segment.from == segment.to) continue;
        const double length = segment.from.distance_to(segment.to);
        if (segment.kind == EstimateKind::OutsidePart) { cost += length; continue; }
        const double outside = segment.kind == EstimateKind::Inside ? 0. :
            outside_length({segment.from, segment.to}, domain, context);
        cost += outside + weight * (length - outside);
    }
    return cost;
}

bool contains(const work::ExPolygons &areas, const work::Point &point, const bool border)
{
    for (const work::ExPolygon &area : areas)
        if (area.contains(point, border)) return true;
    return false;
}

int ring_location(const work::Polygon &ring, const work::Vec2d &point)
{
    bool inside = false;
    for (size_t i = 0; i < ring.size(); ++i) {
        const work::Vec2d a = work::to_vec2d(ring.points[i]);
        const work::Vec2d b = work::to_vec2d(ring.points[(i + 1) % ring.size()]);
        const work::Vec2d edge = b - a, relative = point - a;
        const double cross = edge.x() * relative.y() - edge.y() * relative.x();
        const double tolerance = 1e-7 * edge.norm();
        if (std::abs(cross) <= tolerance && relative.dot(edge) >= -tolerance &&
            relative.dot(edge) <= edge.squaredNorm() + tolerance) return 0;
        if ((a.y() > point.y()) != (b.y() > point.y()) &&
            point.x() < a.x() + (point.y() - a.y()) * edge.x() / edge.y()) inside = !inside;
    }
    return inside ? 1 : -1;
}

bool crossing(const work::ExPolygon &area, const work::Point &from, const work::Point &to,
              Hit &entry, Hit &exit, const plugin_run_context *context, const double minimum_parameter)
{
    if (from == to) return false;
    const work::Vec2d delta = work::to_vec2d(to - from);
    const double squared = delta.squaredNorm();
    std::vector<Hit> hits{{0., from, SIZE_MAX}, {1., to, SIZE_MAX}};
    for (size_t ring = 0; ring <= area.holes.size(); ++ring) {
        throw_if_cancelled(context);
        const work::Points &points = ring == 0 ? area.contour.points : area.holes[ring - 1].points;
        for (size_t edge = 0; edge < points.size(); ++edge) {
            const work::Point &a = points[edge], &b = points[(edge + 1) % points.size()];
            const work::Vec2d edge_delta = work::to_vec2d(b - a), relative = work::to_vec2d(a - from);
            const double denominator = delta.x() * edge_delta.y() - delta.y() * edge_delta.x();
            if (std::abs(denominator) > 1e-12) {
                const double parameter = (relative.x() * edge_delta.y() - relative.y() * edge_delta.x()) / denominator;
                const double edge_parameter = (relative.x() * delta.y() - relative.y() * delta.x()) / denominator;
                if (parameter < 0. || parameter > 1. || edge_parameter < 0. || edge_parameter > 1.) continue;
                const work::Vec2d point = work::to_vec2d(from) + parameter * delta;
                hits.push_back({parameter, work::Point(coord_t(std::round(point.x())), coord_t(std::round(point.y()))),
                                ring == 0 ? edge : SIZE_MAX});
            } else if (std::abs(relative.x() * delta.y() - relative.y() * delta.x()) <= 1e-7 * delta.norm()) {
                // Split collinear overlap at its ends; the ring test excludes the edge itself.
                for (const work::Point &vertex : {a, b}) {
                    const double parameter = work::to_vec2d(vertex - from).dot(delta) / squared;
                    if (parameter >= 0. && parameter <= 1.) hits.push_back({parameter, vertex, ring == 0 ? edge : SIZE_MAX});
                }
            }
        }
    }
    std::stable_sort(hits.begin(), hits.end(), [](const Hit &a, const Hit &b) { return a.parameter < b.parameter; });
    // Endpoint seeds have no edge; retain the geometric hit's edge without moving the seed's point.
    size_t retained = 0;
    for (size_t i = 0; i < hits.size(); ++i) {
        const Hit hit = hits[i];
        if (retained > 0 && std::abs(hits[retained - 1].parameter - hit.parameter) < 1e-12) {
            if (hits[retained - 1].edge == SIZE_MAX) hits[retained - 1].edge = hit.edge;
        } else hits[retained++] = hit;
    }
    hits.resize(retained);
    bool found = false;
    for (size_t i = 1; i < hits.size(); ++i) {
        if (hits[i].parameter <= minimum_parameter) continue;
        const double middle = (hits[i - 1].parameter + hits[i].parameter) * 0.5;
        const work::Vec2d probe = work::to_vec2d(from) + middle * delta;
        bool material = ring_location(area.contour, probe) == 1;
        for (const work::Polygon &hole : area.holes)
            if (ring_location(hole, probe) != -1) material = false;
        if (material) {
            if (!found) entry = hits[i - 1];
            exit = hits[i];
            found = true;
        }
    }
    return found;
}

bool blocked(const work::Points &points, const work::ExPolygons &areas, const plugin_run_context *context)
{
    for (size_t i = 1; i < points.size(); ++i)
        for (const work::ExPolygon &area : areas) {
            throw_if_cancelled(context);
            Hit entry, exit;
            if (crossing(area, points[i - 1], points[i], entry, exit, context)) return true;
        }
    return false;
}

work::Points contour_arc(const work::Polygon &contour, const Hit &entry, const Hit &exit,
                         const plugin_run_context *context)
{
    if (entry.edge == SIZE_MAX || exit.edge == SIZE_MAX) return {};
    work::Points arcs[2];
    double lengths[2] = {0., 0.};
    for (size_t direction = 0; direction < 2; ++direction) {
        arcs[direction].push_back(entry.point);
        size_t edge = entry.edge;
        for (size_t count = 0; edge != exit.edge && count < contour.size(); ++count) {
            throw_if_cancelled(context);
            const size_t vertex = direction == 0 ? (edge + 1) % contour.size() : edge;
            arcs[direction].push_back(contour.points[vertex]);
            edge = direction == 0 ? (edge + 1) % contour.size() : (edge + contour.size() - 1) % contour.size();
        }
        arcs[direction].push_back(exit.point);
        for (size_t i = 1; i < arcs[direction].size(); ++i)
            lengths[direction] += work::Line(arcs[direction][i - 1], arcs[direction][i]).length();
    }
    return lengths[0] <= lengths[1] ? arcs[0] : arcs[1];
}
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
    coord_t spacing = scale_i(m_config.vector_float_or_default("nozzle_diameter", pending.extruder_id, 0.4));
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
    // Each retry removes one original component; crossings of the initial chord or best estimate bound deactivations.
    std::vector<std::pair<size_t, size_t>> disabled_zones;
    size_t deactivation_cap = 0;
    work::Points initial_best_estimate;
    for (size_t attempt = 0;; ++attempt) {
    throw_if_cancelled(m_context);
    if (use_platter) platter_key = PlatterKey(m_group_idx, m_print_z, ignored_platter);
    const CacheKey key(active.id, pending.extruder_id, ignored, spacing, platter_key, disabled_zones);
    std::map<CacheKey, Obstacles>::iterator cached = m_obstacles.find(key);
    if (cached == m_obstacles.end()) {
        Obstacles obstacles;
        for (size_t index : active.forbidden_indexes) {
            throw_if_cancelled(m_context);
            if (std::binary_search(ignored.begin(), ignored.end(), index)) continue;
            const work::ExPolygons &area = m_zones.zone_area(index);
            for (size_t component = 0; component < area.size(); ++component)
                if (!std::binary_search(disabled_zones.begin(), disabled_zones.end(), std::make_pair(index, component)))
                    obstacles.original.push_back(area[component]);
        }
        if (use_platter)
            for (size_t component = 0; component < m_platter_blockers.size(); ++component) {
                throw_if_cancelled(m_context);
                if (!std::binary_search(ignored_platter.begin(), ignored_platter.end(), component))
                    obstacles.original.push_back(m_platter_blockers[component]);
            }
        if (!obstacles.original.empty()) {
            obstacles.original = clipper::union_ex(m_context->plugin_storage, obstacles.original);
            const coord_t radius = std::max<coord_t>(1, scale_i(m_config.vector_float_or_default(
                "nozzle_diameter", pending.extruder_id, 0.4) * 0.5));
            obstacles.expanded = clipper::offset_ex(m_context->plugin_storage, obstacles.original, double(radius));
        }
        // Keep components intact for containment, nearest attachments and ring routing.
        work::ExPolygons domain = obstacles.expanded.empty() ? active.attractors :
            clipper::diff_ex(m_context->plugin_storage, active.attractors, obstacles.expanded);
        if (!domain.empty()) {
            obstacles.domain = std::make_unique<AvoidCrossing::Boundary>();
            AvoidCrossing::build_boundary(m_context->plugin_storage, work::ExPolygons(domain), 2 * spacing, *obstacles.domain);
            size_t ring = 0;
            for (const work::ExPolygon &component : domain) {
                throw_if_cancelled(m_context);
                obstacles.component_rings.push_back(ring);
                ring += 1 + component.holes.size();
                std::unique_ptr<AvoidCrossing::Boundary> boundary = std::make_unique<AvoidCrossing::Boundary>();
                AvoidCrossing::build_boundary(m_context->plugin_storage, {component}, 2 * spacing, *boundary);
                obstacles.component_boundaries.push_back(std::move(boundary));
            }
            obstacles.components = std::move(domain);
            obstacles.material_proxy = clipper::offset_ex(m_context->plugin_storage, obstacles.components, 1.5 * double(spacing));
        }
        throw_if_cancelled(m_context);
        cached = m_obstacles.emplace(key, std::move(obstacles)).first;
    }
    const Obstacles &obstacles = cached->second;
    const std::vector<Attachment> sources = source_outside ? std::vector<Attachment>{{SIZE_MAX, source}} :
        endpoint_candidates(obstacles.components, source, spacing, m_context);
    const std::vector<Attachment> targets = target_outside ? std::vector<Attachment>{{SIZE_MAX, target}} :
        endpoint_candidates(obstacles.components, target, spacing, m_context);
    std::vector<Hypothesis> hypotheses;
    for (const Attachment &from : sources)
        for (const Attachment &to : targets) {
            throw_if_cancelled(m_context);
            // Within one component, the option without a domain may cut only within the grown material proxy.
            if (from.component == SIZE_MAX && to.component == SIZE_MAX &&
                !source_outside && !target_outside &&
                sources.front().component != SIZE_MAX && sources.front().component == targets.front().component &&
                outside_length({source, target}, obstacles.material_proxy, m_context) > SCALED_EPSILON) continue;
            // Endpoint attachments within the selection tolerance cross perimeter material, so estimate them as inside legs.
            const EstimateKind source_kind = from.component != SIZE_MAX ? EstimateKind::Inside : EstimateKind::Outside;
            const EstimateKind target_kind = to.component != SIZE_MAX ? EstimateKind::Inside : EstimateKind::Outside;
            std::vector<EstimateSegment> estimate{{source, from.point, source_kind}};
            work::Point exit = from.point, entry = to.point;
            if (from.component != SIZE_MAX && to.component != SIZE_MAX && from.component != to.component) {
                work::Point start = from.point, end = to.point;
                AvoidCrossing::RouterCounts estimate_counts;
                const std::optional<AvoidCrossing::IslandBridge> bridge = AvoidCrossing::select_island_bridge(
                    *obstacles.domain, obstacles.component_rings[from.component], obstacles.component_rings[to.component],
                    start, end, jump_weight, estimate_counts, [this]() { throw_if_cancelled(m_context); });
                if (!bridge) continue;
                exit = bridge->from;
                entry = bridge->to;
                estimate.push_back({from.point, bridge->from, EstimateKind::Inside});
                estimate.push_back({bridge->from, bridge->to, EstimateKind::Outside});
                estimate.push_back({bridge->to, to.point, EstimateKind::Inside});
            } else if (from.component != SIZE_MAX && to.component == SIZE_MAX) {
                exit = nearest_point(obstacles.components[from.component], target);
                estimate.push_back({from.point, exit, EstimateKind::Inside});
                estimate.push_back({exit, target, target_outside ? EstimateKind::OutsidePart : EstimateKind::Outside});
            } else if (from.component == SIZE_MAX && to.component != SIZE_MAX) {
                entry = nearest_point(obstacles.components[to.component], source);
                estimate.push_back({source, entry, source_outside ? EstimateKind::OutsidePart : EstimateKind::Outside});
                estimate.push_back({entry, to.point, EstimateKind::Inside});
            } else estimate.push_back({from.point, to.point,
                from.component != SIZE_MAX ? EstimateKind::Inside :
                    use_platter ? EstimateKind::OutsidePart : EstimateKind::Outside});
            estimate.push_back({to.point, target, target_kind});
            work::Points estimate_points{source};
            for (const EstimateSegment &segment : estimate) estimate_points.push_back(segment.to);
            hypotheses.push_back({from, to, exit, entry, estimate_cost(estimate, obstacles.components, jump_weight, m_context), std::move(estimate_points)});
        }
    // Domain choices precede the option without a domain on equal estimates.
    std::stable_sort(hypotheses.begin(), hypotheses.end(), [](const Hypothesis &a, const Hypothesis &b) { return a.cost < b.cost; });
    if (attempt == 0) {
        // Freeze the finite retry budget before removals can change attachments or the ranking.
        // Retain this polyline even if removals change the best hypothesis: selection and its fixed cap must use the same crossings.
        initial_best_estimate = hypotheses.empty() ? work::Points{} : hypotheses.front().estimate;
        for (size_t index : active.forbidden_indexes) {
            if (std::binary_search(ignored.begin(), ignored.end(), index)) continue;
            for (const work::ExPolygon &component : m_zones.zone_area(index))
                if (blocked({source, target}, {component}, m_context) || blocked(initial_best_estimate, {component}, m_context)) ++deactivation_cap;
        }
        if (use_platter)
            for (size_t component = 0; component < m_platter_blockers.size(); ++component)
                if (!std::binary_search(ignored_platter.begin(), ignored_platter.end(), component) &&
                    (blocked({source, target}, {m_platter_blockers[component]}, m_context) ||
                     blocked(initial_best_estimate, {m_platter_blockers[component]}, m_context))) ++deactivation_cap;
    }

    // Build the obstacle-only reference once, even if several domain hypotheses need its outside-length bound.
    bool reference_ready = false, reference_failed = false;
    work::Points reference;
    double reference_outside = 0.;
    const auto prepare_reference = [&]() {
        if (reference_ready) return;
        bool changed = false;
        reference = detour_around_forbidden(source, target, obstacles.expanded, m_context, changed, reference_failed);
        reference_failed = reference_failed || blocked(reference, obstacles.original, m_context);
        if (!reference_failed) reference_outside = outside_length(reference, obstacles.components, m_context);
        reference_ready = true;
    };
    const c_float_or_percent budget = m_config.float_or_percent_or_default(
        "avoid_crossing_perimeters_max_detour", c_float_or_percent{0., 0});
    const double direct_length = work::Line(source, target).length();
    bool geometrically_valid = false;
    for (const Hypothesis &hypothesis : hypotheses) {
        throw_if_cancelled(m_context);
        const Attachment &from = hypothesis.source, &to = hypothesis.target;
        const bool domain_route = from.component != SIZE_MAX || to.component != SIZE_MAX;
        work::Points points;
        if (!domain_route) {
            prepare_reference();
            if (reference_failed) continue;
            points = reference;
        } else if (from.component == to.component) {
            points.push_back(source);
            append_component_path(points, *obstacles.component_boundaries[from.component], from.point, to.point, m_context, m_router_counts);
            points.push_back(target);
        } else {
            points.push_back(source);
            if (from.component != SIZE_MAX)
                append_component_path(points, *obstacles.component_boundaries[from.component], from.point, hypothesis.exit, m_context, m_router_counts);
            bool changed = false, failed = false;
            const work::Points connector = detour_around_forbidden(hypothesis.exit, hypothesis.entry,
                obstacles.expanded, m_context, changed, failed);
            if (failed) continue;
            points.insert(points.end(), connector.begin(), connector.end());
            if (to.component != SIZE_MAX)
                append_component_path(points, *obstacles.component_boundaries[to.component], hypothesis.entry, to.point, m_context, m_router_counts);
            points.push_back(target);
        }
        points.erase(std::unique(points.begin(), points.end()), points.end());
        if (points.size() < 2 || points.front() != source || points.back() != target ||
            blocked(points, obstacles.original, m_context)) continue;
        geometrically_valid = true;
        double length = 0.;
        for (size_t i = 1; i < points.size(); ++i) length += work::Line(points[i - 1], points[i]).length();
        if (budget.value > 0.) {
            const double limit = budget.percent ? direct_length * 0.01 * budget.value : scale_d(budget.value);
            if (length - direct_length > limit) continue;
        }
        prepare_reference();
        // Clipping rounds intersection points: allow one SCALED_EPSILON over the complete path.
        if (!reference_failed && outside_length(points, obstacles.components, m_context) > reference_outside + SCALED_EPSILON) continue;
        std::vector<c_point> result;
        for (const work::Point &point : points) result.push_back({point.x(), point.y()});
        if (domain_route) ++m_domain_routes;
        if (from.component != SIZE_MAX && to.component != SIZE_MAX && from.component != to.component) ++m_bridges;
        if (length > direct_length + SCALED_EPSILON) ++m_detours;
        return result;
    }
    if (geometrically_valid || attempt >= deactivation_cap) break;
    // Only geometric failure authorizes removal; prefer chord crossings, then the initial best estimate's attachments and bridge.
    double first_parameter = std::numeric_limits<double>::max();
    std::pair<size_t, size_t> first_zone(SIZE_MAX, SIZE_MAX);
    size_t first_platter = SIZE_MAX;
    for (size_t candidate = 0; candidate < 2; ++candidate) {
    const work::Points selection = candidate == 0 ? work::Points{source, target} :
        initial_best_estimate;
    for (size_t index : active.forbidden_indexes) {
        throw_if_cancelled(m_context);
        if (std::binary_search(ignored.begin(), ignored.end(), index)) continue;
        const work::ExPolygons &area = m_zones.zone_area(index);
        for (size_t component = 0; component < area.size(); ++component) {
            const std::pair<size_t, size_t> identity(index, component);
            for (size_t segment = 1; segment < selection.size(); ++segment) {
            Hit entry, exit;
            if (!std::binary_search(disabled_zones.begin(), disabled_zones.end(), identity) &&
                crossing(area[component], selection[segment - 1], selection[segment], entry, exit, m_context) &&
                double(segment - 1) + entry.parameter < first_parameter) {
                first_parameter = double(segment - 1) + entry.parameter;
                first_zone = identity;
            }
            }
        }
    }
    if (use_platter)
        for (size_t component = 0; component < m_platter_blockers.size(); ++component) {
            throw_if_cancelled(m_context);
            for (size_t segment = 1; segment < selection.size(); ++segment) {
            Hit entry, exit;
            if (!std::binary_search(ignored_platter.begin(), ignored_platter.end(), component) &&
                crossing(m_platter_blockers[component], selection[segment - 1], selection[segment], entry, exit, m_context) &&
                double(segment - 1) + entry.parameter < first_parameter) {
                first_parameter = double(segment - 1) + entry.parameter;
                first_platter = component;
            }
            }
        }
    if (first_zone.first != SIZE_MAX || first_platter != SIZE_MAX) break;
    }
    if (first_platter != SIZE_MAX)
        ignored_platter.insert(std::lower_bound(ignored_platter.begin(), ignored_platter.end(), first_platter), first_platter);
    else if (first_zone.first != SIZE_MAX)
        disabled_zones.insert(std::lower_bound(disabled_zones.begin(), disabled_zones.end(), first_zone), first_zone);
    else break;
    ++m_disabled_blockers;
    }
    ++m_fallbacks;
    return direct;
}
}}}
