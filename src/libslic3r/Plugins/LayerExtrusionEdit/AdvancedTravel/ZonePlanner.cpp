///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

#include "ZonePlanner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "../../Common/ClipperHelpers.hpp"
#include "../AvoidCrossing/LayerInputCache.hpp"

/*
ZonePlanner routes on the union of active attractors minus expanded forbidden
material, then routes around obstacles if the domain path crosses forbidden material.
If obstacle routing stays straight without failure and its chord passes validation
against all remaining original forbidden areas, that chord is accepted without
counting a fallback or a domain route. A chord inside only the nozzle margin may
still require a detour or fallback.
Endpoint membership removes individual unexpanded zones before union and nozzle
expansion, so overlapping margins do not erase unrelated obstacles. Cached areas
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
component bound and original-area validation reject unsafe obstacle routes with
a counted straight fallback; the total detour budget can reject either kind of
route with the same fallback. A source or target in a
hole whose route crosses the surrounding material cannot be connected through
the outer contour and also produces a counted straight fallback. A source or
target inside the expanded margin but outside the original material also causes
a counted straight fallback when the route needs a missing contour entry or exit.
Cancellation propagates.

    path: filter zones, cache the domain, route and validate all segments
    |-- LayerInputCache::get: resolve spacing and island-jump settings
    |-- build_boundary / route_on_boundary: index the domain and follow the rings a chord crosses
    |-- eligible: resolve regional and auxiliary activation
    |-- contains: test filled material, including its border
    |-- crossing: find first entry and last exit, ignoring contacts
    |-- contour_arc: walk each contour direction and choose the shorter
    `-- blocked: validate candidate segments against filled material
*/
namespace slic3r_api { namespace LayerExtrusionEdit { namespace AdvancedTravel {
namespace work = PluginCommon::polygon_work;
namespace clipper = PluginCommon::clipper;
namespace travel = TravelConnection;

namespace {
struct Hit { double parameter; work::Point point; size_t edge; };
// Test filled areas without changing them; holes exclude their interior and borders follow the supplied flag.
bool contains(const work::ExPolygons &areas, const work::Point &point, bool border = true);
// Classify an unrounded point against one ring: -1 outside, 0 on an edge, 1 inside, with sub-unit floating-point tolerance.
int ring_location(const work::Polygon &ring, const work::Vec2d &point);
// Find the first interval whose exit is strictly after minimum_parameter, returning its entry and the last material exit; contacts do not count.
bool crossing(const work::ExPolygon &area, const work::Point &from, const work::Point &to,
              Hit &entry, Hit &exit, const plugin_run_context *context, double minimum_parameter = 0.);
// Return whether any nonzero path segment enters filled material; cancellation propagates per component.
bool blocked(const work::Points &points, const work::ExPolygons &areas, const plugin_run_context *context);
// Build both outer-contour arcs including the rounded hits, choosing the shorter; hole hits cannot produce a contour route.
work::Points contour_arc(const work::Polygon &contour, const Hit &entry, const Hit &exit,
                         const plugin_run_context *context);

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
    std::sort(hits.begin(), hits.end(), [](const Hit &a, const Hit &b) { return a.parameter < b.parameter; });
    hits.erase(std::unique(hits.begin(), hits.end(), [](const Hit &a, const Hit &b) {
        return std::abs(a.parameter - b.parameter) < 1e-12;
    }), hits.end());
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
                         const Print &print, AvoidCrossing::LayerInputCache &layer_inputs) :
    m_compatibility(compatibility), m_zones(zones), m_config(print_config),
    m_auxiliary_avoidance(auxiliary_avoidance), m_context(context), m_print(print), m_layer_inputs(layer_inputs)
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
    coord_t spacing = scale_i(m_config.vector_float_or_default("nozzle_diameter", pending.extruder_id, 0.4));
    bool island_jump = false;
    double jump_weight = 0.;
    for (const travel::TravelEndpoint *endpoint : {&pending.target, &pending.source}) {
        if (!endpoint->region_island.valid() || endpoint->region_island.region_count() == 0) continue;
        const AvoidCrossing::LayerInput &layer = *m_layer_inputs.get(
            m_print, endpoint->region_island.region(0).layer(), m_context, nullptr).target;
        spacing = layer.perimeter_spacing;
        for (const AvoidCrossing::RegionInput &region : layer.regions)
            if (region.avoid_travel_island) {
                island_jump = true;
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
    const CacheKey key(active.id, pending.extruder_id, ignored, spacing);
    std::map<CacheKey, Obstacles>::iterator cached = m_obstacles.find(key);
    if (cached == m_obstacles.end()) {
        Obstacles obstacles;
        for (size_t index : active.forbidden_indexes) {
            throw_if_cancelled(m_context);
            if (std::binary_search(ignored.begin(), ignored.end(), index)) continue;
            const work::ExPolygons &area = m_zones.zone_area(index);
            obstacles.original.insert(obstacles.original.end(), area.begin(), area.end());
        }
        if (!obstacles.original.empty()) {
            obstacles.original = clipper::union_ex(m_context->plugin_storage, obstacles.original);
            const coord_t radius = std::max<coord_t>(1, scale_i(m_config.vector_float_or_default(
                "nozzle_diameter", pending.extruder_id, 0.4) * 0.5));
            obstacles.expanded = clipper::offset_ex(m_context->plugin_storage, obstacles.original, double(radius));
        }
        // The same components serve as containment masks, so interior endpoints need no projection.
        work::ExPolygons domain = obstacles.expanded.empty() ? active.attractors :
            clipper::diff_ex(m_context->plugin_storage, active.attractors, obstacles.expanded);
        if (!domain.empty()) {
            obstacles.domain = std::make_unique<AvoidCrossing::Boundary>();
            AvoidCrossing::build_boundary(m_context->plugin_storage, work::ExPolygons(domain), 2 * spacing, *obstacles.domain);
            for (const work::ExPolygon &component : domain)
                obstacles.domain->boundary_growth.emplace_back(component, component);
        }
        throw_if_cancelled(m_context);
        cached = m_obstacles.emplace(key, std::move(obstacles)).first;
    }
    const Obstacles &obstacles = cached->second;
    work::Points domain_points;
    // Even a direct domain path is valid unless it crosses remaining original forbidden material.
    if (obstacles.domain) {
        domain_points = AvoidCrossing::route_on_boundary(*obstacles.domain, source, target, spacing,
            2 * spacing, island_jump, jump_weight, m_context->plugin_storage, m_router_counts,
            [this]() { throw_if_cancelled(m_context); }).points;
        if (blocked(domain_points, obstacles.original, m_context)) domain_points.clear();
    }
    work::Points points{source};
    bool changed = false, failed = false;
    double progress = 0.;
    for (size_t iteration = 0; domain_points.empty() && iteration <= obstacles.expanded.size(); ++iteration) {
        size_t first = SIZE_MAX;
        Hit entry{}, exit{};
        entry.parameter = std::numeric_limits<double>::max();
        for (size_t component = 0; component < obstacles.expanded.size(); ++component) {
            throw_if_cancelled(m_context);
            Hit a, b;
            if (crossing(obstacles.expanded[component], source, target, a, b, m_context, progress) &&
                a.parameter < entry.parameter) { first = component; entry = a; exit = b; }
        }
        if (first == SIZE_MAX) break;
        if (iteration == obstacles.expanded.size()) { failed = true; break; }
        const work::Points arc = contour_arc(obstacles.expanded[first].contour, entry, exit, m_context);
        if (arc.empty()) { failed = true; break; }
        points.insert(points.end(), arc.begin(), arc.end());
        progress = exit.parameter;
        changed = true;
    }
    points.push_back(target);
    const bool domain_route = !domain_points.empty();
    if (domain_route) points = std::move(domain_points);
    points.erase(std::unique(points.begin(), points.end()), points.end());
    const bool crosses_forbidden = !failed && blocked(points, obstacles.original, m_context);
    // A rejected domain path does not make a validated, unchanged obstacle route a fallback.
    if (!domain_route && !changed && !failed && !crosses_forbidden) return direct;
    failed = failed || crosses_forbidden;
    double length = 0.;
    for (size_t i = 1; i < points.size(); ++i) length += work::Line(points[i - 1], points[i]).length();
    const c_float_or_percent budget = m_config.float_or_percent_or_default(
        "avoid_crossing_perimeters_max_detour", c_float_or_percent{0., 0});
    const double direct_length = work::Line(source, target).length();
    if (budget.value > 0.) {
        const double limit = budget.percent ? direct_length * 0.01 * budget.value : scale_d(budget.value);
        if (length - direct_length > limit) failed = true;
    }
    if (failed) { ++m_fallbacks; return direct; }
    std::vector<c_point> result;
    for (const work::Point &point : points) result.push_back({point.x(), point.y()});
    if (domain_route) ++m_domain_routes;
    if (length > direct_length + SCALED_EPSILON) ++m_detours;
    return result;
}
}}}
