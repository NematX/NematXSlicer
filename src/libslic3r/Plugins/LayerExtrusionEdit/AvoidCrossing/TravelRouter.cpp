///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ Copyright (c) Prusa Research 2020 - 2022 Lukas Hejl, Vojtech Bubnik
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#include "TravelRouter.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <unordered_set>
#include <boost/functional/hash.hpp>
#include "libslic3r/Plugins/Common/ClipperHelpers.hpp"

/*
TravelRouter supplies two-dimensional detours to plugin workers after callers
have decided that avoidance is active and routing is needed. Callers retain
responsibility for crossing tests, path insertion and Z motion. This heuristic
does not find a shortest path or guarantee that a returned path avoids every boundary.
A detour budget may replace any computed detour with the direct segment.
route_on_boundary exposes ring following on caller-built geometry in the caller's
coordinates, without layer selection, box clipping or a detour budget.

TravelBoundary.hpp provides internal rings in object-local coordinates and
external rings in print coordinates. Internal rings follow insets of filled
object areas, with regional tops handled during boundary construction. External
rings are built from object holes placed at their instance shifts, not from the
outlines of outer obstacles. An initial interior
inset decides whether object routing has usable geometry. Support uses the object
insets supplied as additional input: its travel may remain straight or enter an
object inset; it does not route support travels around objects.

Both endpoints and all returned points use object-local coordinates. External
routing adds context.origin while planning, then subtracts it from every returned
point. init_layer builds the initial interior inset. Routing rings are built on
demand when the selected mode needs them; island grids are built only for island jumps.

Routing chooses the requested rings lazily, clips the working endpoints to their
bounding box when needed, and restores the requested endpoints afterward.
Internal routing first tries to place endpoints in the retained insets. Fallback
placements may compare how much of a proposed shortcut lies outside retained top
areas. Grid crossings and nearby projected points select where to follow rings;
the nearby projections are protected from ordinary shortcut removal. Between
islands, bridge scores combine bridge length with weighted endpoint approaches.
Ring direction compares float scores derived from accumulated distances after
subtracting four endpoint-to-vertex distances, choosing backward on a tie.
Simplification coalesces epsilon-near neighbours, even protected ones, then skips
unprotected points only when the shortcut misses the rings. The final extra
length is checked against the absolute-mm or percentage detour budget.

    init_layer: borrow a target and reset geometry and diagnostics
    `-- TravelBoundary::init_layer: prepare the initial interior inset
    travel_to: choose local or print coordinates, rings and a working box; enforce the budget
    |-- TravelBoundary::build_internal or build_external: lazily prepare rings
    `-- route_on_boundary: place endpoints and follow the rings the segment crosses or nears
        |-- find_point_on_boundary: try the grid, then a containing island's inset
        |-- segment_intersections: collect crossings and accumulated distances
        |   `-- extend_for_closest_lines: protect nearby projected points
        |-- jump_between_island: score bridges using lazily built island grids
        |   |-- brute_force_jump: recover when grid queries leave edges unknown
        |   `-- segment_intersections: rebuild approaches around the selected bridge
        |-- get_shortest_direction: select the ring walk by float score
        `-- simplify_travel: coalesce neighbours and remove unobstructed waypoints

The caller keeps input and storage alive and unchanged during use. Each worker
owns a router for each layer/tool pair: the first internal boundary build selects
its tool-specific geometry. Callbacks are owned; their borrowed captures
must remain valid. External geometry is requested only when needed. Cancellation
and allocation failures propagate without returning a partial path.
*/
namespace slic3r_api { namespace LayerExtrusionEdit { namespace AvoidCrossing {
namespace {
using namespace PluginCommon::polygon_work;
using PluginCommon::polygon_work::Point;
using PluginCommon::polygon_work::Polygon;
using PluginCommon::polygon_work::Polyline;
using PluginCommon::polygon_work::Polylines;
using PluginCommon::polygon_work::ExPolygon;
using PluginCommon::polygon_work::BoundingBox;
using PluginCommon::polygon_work::Vec2d;
namespace EdgeGrid = PluginCommon::EdgeGrid;
namespace clipper = PluginCommon::clipper;
namespace line_alg = PluginCommon::polygon_work::line_alg;

struct TravelPoint { Point point; int border_idx; bool do_not_remove = false; };
struct Intersection { size_t border_idx; size_t line_idx; Point point; float distance; bool do_not_remove = false; };
struct ClosestLine { size_t border_idx; size_t line_idx; Point point; };
using SegmentSet = std::unordered_set<std::pair<size_t, size_t>, boost::hash<std::pair<size_t, size_t>>>;

// Evaluate a float norm before widening, preserving the contour-distance arithmetic.
static float float_norm(const Point &point);
// Return the float squared norm used to compare nearby boundary endpoints.
static float float_squared_norm(const Point &point);
// Return the selected vertex if different; otherwise start at index + 1 and scan in the selected direction.
// The ring must contain a vertex different from point.
template<bool forward> static Point find_first_different_vertex(const Polygon &polygon, size_t index, const Point &point);
// Return the inward bisector of two nonzero edges meeting at middle.
static Vec2d three_points_inward_normal(const Point &left, const Point &middle, const Point &right);
// Return a ring vertex moved inward with truncation of the normal's scaled components.
static Point get_polygon_vertex_offset(const Polygon &polygon, size_t index, int offset);
// Move middle along the inward bisector formed by the selected distinct vertices; truncate scaled components.
static Point get_middle_point_offset(const Polygon &polygon, size_t left, size_t right, const Point &middle, coord_t offset);
// Copy the selected travel points into an owned polyline.
static Polyline to_polyline(const std::vector<TravelPoint> &travel);
// Return whether point belongs to any of the inset islands retained during boundary construction.
static bool is_in_internal_boundary(const Boundary &boundary, const Point &point);
// Place point in an inset using the grid, then the containing original island's inset vertices and edges.
// max_distance limits the grid search, not the fallback result; return false if no placement is found.
static bool find_point_on_boundary(Point &point, const Boundary &boundary, coord_t max_distance, RouterCounts &counts);
// Collect nearby segments once and sort them by squared distance to center.
static std::vector<ClosestLine> get_closest_lines_in_radius(const EdgeGrid::Grid &grid, const Point &center, coord_t radius);
// Add protected close endpoints, preferring one shared ring over a chain of intersections.
static std::vector<Intersection> extend_for_closest_lines(const std::vector<Intersection> &intersections, const Boundary &boundary,
                                                         const Point &start, const Point &end, coord_t radius);
enum class Direction { Forward, Backward };
// Compare float direction scores after subtracting four endpoint-to-vertex distances; choose Backward on a tie.
static Direction get_shortest_direction(const Boundary &boundary, const Intersection &first, const Intersection &second, float length);
// Coalesce epsilon-near neighbours (including protected points), then skip unprotected points only across ring-free shortcuts.
static std::vector<TravelPoint> simplify_travel(const Boundary &boundary, const std::vector<TravelPoint> &travel);
// Record a projected point on its outgoing edge, using zero distance when it coincides with a vertex.
static void set_intersection(Intersection &intersection, const Polygon &polygon, size_t edge, const Point &point);
// Update first and last when grid queries leave an edge unknown, then score vertex-to-edge bridges using weighted approaches.
static void brute_force_jump(Intersection &first, Intersection &last, const Line &line,
                             const Polygon &a, const Polygon &b, double weight);
// Collect, parameterize and sort crossings along a segment, then add nearby protected endpoints.
static std::vector<Intersection> segment_intersections(const Boundary &boundary, const Point &start, const Point &end, coord_t radius);
// Replace the crossing chain by the weighted bridge between distinct islands, lazily building their grids.
static void jump_between_island(Boundary &boundary, Point start, Point end, std::vector<Intersection> &intersections,
                                coord_t radius, double weight, RouterCounts &counts, const std::function<void()> &cancel);

struct AllIntersectionsVisitor
{
    // Borrow a grid and output vector, and own the segment whose intersections are collected.
    AllIntersectionsVisitor(const EdgeGrid::Grid &grid, std::vector<Intersection> &out, const Line &line)
        : grid(grid), out(out), line(line) { seen.reserve(out.capacity()); }
    // Append each intersected grid segment once; traversal always continues.
    bool operator()(int64_t row, int64_t col) {
        const auto range = grid.cell_data_range(row, col);
        for (auto it = range.first; it != range.second; ++it) {
            Point point;
            if (line.intersection(grid.line(*it), &point) && seen.find(*it) == seen.end()) {
                out.push_back({it->first, it->second, point}); seen.insert(*it);
            }
        }
        return true;
    }
    const EdgeGrid::Grid &grid;
    std::vector<Intersection> &out;
    Line line;
    SegmentSet seen;
};

struct FirstIntersectionVisitor
{
    // Borrow the grid; the caller supplies endpoint pointers before visiting cells.
    explicit FirstIntersectionVisitor(const EdgeGrid::Grid &grid) : grid(grid) {}
    // Stop at the first intersected segment in grid visitation order.
    bool operator()(coord_t row, coord_t col) {
        assert(pt_current && pt_next);
        const auto range = grid.cell_data_range(row, col);
        intersect = false;
        for (auto it = range.first; it != range.second; ++it) {
            const Line segment = grid.line(*it);
            if ((segment.a == *pt_current) | (segment.a == *pt_next) |
                segments_intersect(segment.a, segment.b, *pt_current, *pt_next)) {
                intersect = true; intersection_contour_idx = it->first; intersection_line_idx = it->second;
                return false;
            }
        }
        return true;
    }
    const EdgeGrid::Grid &grid;
    const Point *pt_current = nullptr, *pt_next = nullptr;
    bool intersect = false;
    size_t intersection_contour_idx = size_t(-1), intersection_line_idx = size_t(-1);
};

struct FirstEpsilonIntersectionVisitor
{
    // Borrow the segment endpoints so a nearby endpoint can be extended across a contour.
    FirstEpsilonIntersectionVisitor(const EdgeGrid::Grid &grid, Point &a, Point &b) : grid(grid), a(a), b(b) {}
    // Find a segment within epsilon and extend the supplied endpoint across it using a truncated offset vector.
    bool operator()(coord_t row, coord_t col) {
        const auto range = grid.cell_data_range(row, col);
        intersect = false;
        for (auto it = range.first; it != range.second; ++it) {
            const Line segment = grid.line(*it);
            Point point;
            if (segment.distance_to_squared(a, &point) < SCALED_EPSILON * SCALED_EPSILON) {
                intersect = true; intersection_contour_idx = it->first; intersection_line_idx = it->second;
                Line line(point, b);
                for (int i = 2; i < SCALED_EPSILON && !segments_intersect(segment.a, segment.b, line.a, b); i += 2) {
                    line.a = point;
                    const Vec2d direction = to_vec2d(line.b - line.a).normalized();
                    const Point offset = to_point(direction * double(i));
                    line.a -= offset;
                    line.b += offset;
                }
                assert(a != line.a); a = line.a;
                assert(segments_intersect(segment.a, segment.b, a, b));
                return false;
            }
        }
        return true;
    }
    const EdgeGrid::Grid &grid;
    Point &a, &b;
    bool intersect = false;
    size_t intersection_contour_idx = size_t(-1), intersection_line_idx = size_t(-1);
};

struct MinDistanceVisitor
{
    // Borrow the grid and own the search center and squared radius.
    MinDistanceVisitor(const EdgeGrid::Grid &grid, const Point &center, double radius_squared)
        : grid(grid), center(center), radius_squared(radius_squared) {}
    // Append unique segments within the radius using the truncating line_alg projection.
    bool operator()(coord_t row, coord_t col) {
        const auto range = grid.cell_data_range(row, col);
        for (auto it = range.first; it != range.second; ++it) {
            Point point;
            if (seen.find(*it) == seen.end() && line_alg::distance_to_squared(grid.line(*it), center, &point) <= radius_squared) {
                lines.push_back({it->first, it->second, point}); seen.insert(*it);
            }
        }
        return true;
    }
    const EdgeGrid::Grid &grid;
    Point center;
    double radius_squared;
    std::vector<ClosestLine> lines;
    SegmentSet seen;
};

struct AnyIntersectionsVisitor
{
    // Borrow the grid and own the complete path tested in every visited cell.
    AnyIntersectionsVisitor(const EdgeGrid::Grid &grid, const Polyline &path) : grid(grid), path(path) {}
    // Stop when any path segment intersects a referenced contour segment.
    bool operator()(coord_t row, coord_t col) {
        if (has_intersection) return false;
        const auto range = grid.cell_data_range(row, col);
        for (auto it = range.first; it != range.second; ++it) {
            for (size_t i = 1; i < path.size(); ++i) {
                Point point;
                if (Line(path.points[i - 1], path.points[i]).intersection(grid.line(*it), &point)) {
                    has_intersection = true; return false;
                }
            }
        }
        return true;
    }
    const EdgeGrid::Grid &grid;
    Polyline path;
    bool has_intersection = false;
};

static float float_squared_norm(const Point &point) { const float x = float(point.x()), y = float(point.y()); return x * x + y * y; }
static float float_norm(const Point &point) { return std::sqrt(float_squared_norm(point)); }

template<bool forward> static Point find_first_different_vertex(const Polygon &polygon, size_t index, const Point &point)
{
    assert(index < polygon.size());
    if (point != polygon.points[index]) return polygon.points[index];
    int line = (int(index) + 1) % int(polygon.size());
    assert(line != int(index));
    if constexpr (forward) {
        for (; point == polygon.points[line] && line != int(index); line = line + 1 < int(polygon.size()) ? line + 1 : 0) {}
    } else {
        for (; point == polygon.points[line] && line != int(index); line = line - 1 >= 0 ? line - 1 : int(polygon.size()) - 1) {}
    }
    assert(point != polygon.points[line]);
    return polygon.points[line];
}

static Vec2d three_points_inward_normal(const Point &left, const Point &middle, const Point &right)
{
    assert(left != middle && middle != right);
    return (perp(to_vec2d(middle - left)).normalized() + perp(to_vec2d(right - middle)).normalized()).normalized();
}

static Point get_polygon_vertex_offset(const Polygon &polygon, size_t index, int offset)
{
    const Point &middle = polygon.points[index];
    const Point left = find_first_different_vertex<false>(polygon, index == 0 ? polygon.size() - 1 : index - 1, middle);
    const Point right = find_first_different_vertex<true>(polygon, index + 1 == polygon.size() ? 0 : index + 1, middle);
    return middle + to_point(three_points_inward_normal(left, middle, right) * double(offset));
}

static Point get_middle_point_offset(const Polygon &polygon, size_t left, size_t right, const Point &middle, coord_t offset)
{
    return middle + to_point(three_points_inward_normal(find_first_different_vertex<false>(polygon, left, middle), middle,
        find_first_different_vertex<true>(polygon, right, middle)) * double(offset));
}

static Polyline to_polyline(const std::vector<TravelPoint> &travel)
{
    Polyline out;
    out.points.reserve(travel.size());
    for (const TravelPoint &point : travel) out.points.push_back(point.point);
    return out;
}

static bool is_in_internal_boundary(const Boundary &boundary, const Point &point)
{
    for (const std::pair<ExPolygon, ExPolygon> &growth : boundary.boundary_growth)
        if (growth.second.contains(point)) return true;
    return false;
}

static bool find_point_on_boundary(Point &point, const Boundary &boundary, coord_t max_distance, RouterCounts &counts)
{
    if (!is_in_internal_boundary(boundary, point)) {
        const EdgeGrid::Grid::ClosestPointResult closest = boundary.grid.closest_point_signed_distance(point, max_distance);
        Point contour_point;
        if (closest.contour_idx == size_t(-1)) {
            ++counts.boundary_fallback;
            bool found = false;
            for (const std::pair<ExPolygon, ExPolygon> &growth : boundary.boundary_growth) {
                if (growth.first.contains(point)) {
                    double distance = std::numeric_limits<double>::max();
                    for (const Polygon &polygon : to_polygons(growth.second)) {
                        const Point candidate = *closest_point(polygon.points, point);
                        const double d = candidate.distance_to_square(point);
                        if (d < distance) { distance = d; contour_point = candidate; found = true; }
                    }
                    if (std::sqrt(distance) > max_distance) {
                        for (const Polygon &polygon : to_polygons(growth.second)) {
                            const Point candidate = point_projection(polygon.points, point).first;
                            const double d = candidate.distance_to_square(point);
                            if (d < distance) { distance = d; contour_point = candidate; found = true; }
                        }
                    }
                    break;
                }
            }
            if (found) {
                assert(boundary.grid.bbox().contains(contour_point));
                if (boundary.grid.bbox().contains(contour_point)) {
                    Line line(point, contour_point); line.extend_end(SCALED_EPSILON); point = line.b;
                    assert(boundary.grid.bbox().contains(point));
                    return true;
                }
            }
            return false;
        }
        const EdgeGrid::Contour &contour = boundary.grid.contours()[closest.contour_idx];
        contour_point = Point(contour.segment_start(closest.start_point_idx)).interpolate(closest.t, Point(contour.segment_end(closest.start_point_idx)));
        Line line(point, contour_point); line.extend_end(SCALED_EPSILON); point = line.b;
        assert(boundary.grid.bbox().contains(point));
    }
    return true;
}

static std::vector<ClosestLine> get_closest_lines_in_radius(const EdgeGrid::Grid &grid, const Point &center, coord_t radius)
{
    const Point r(radius, radius);
    MinDistanceVisitor visitor(grid, center, double(radius) * double(radius));
    grid.visit_cells_intersecting_box(BoundingBox(center - r, center + r), visitor);
    std::sort(visitor.lines.begin(), visitor.lines.end(), [&center](const ClosestLine &a, const ClosestLine &b) {
        return to_vec2d(center - a.point).squaredNorm() < to_vec2d(center - b.point).squaredNorm();
    });
    return visitor.lines;
}

static std::vector<Intersection> extend_for_closest_lines(const std::vector<Intersection> &intersections, const Boundary &boundary,
                                                         const Point &start, const Point &end, coord_t radius)
{
    const std::vector<ClosestLine> start_lines = get_closest_lines_in_radius(boundary.grid, start, radius);
    const std::vector<ClosestLine> end_lines = get_closest_lines_in_radius(boundary.grid, end, radius);
    // Store a protected intersection at the same accumulated float distance as a regular crossing.
    const auto convert = [&boundary](const ClosestLine &line) -> Intersection {
        const float distance = float_norm(line.point - boundary.boundaries[line.border_idx][line.line_idx]);
        return {line.border_idx, line.line_idx, line.point, boundary.boundaries_params[line.border_idx][line.line_idx] + distance, true};
    };
    // Prefer a ring near both endpoints so the route need not follow a chain across different rings.
    if (!start_lines.empty() && !end_lines.empty()) {
        for (const ClosestLine &last : end_lines) {
            for (const ClosestLine &first : start_lines) {
                if (first.border_idx == last.border_idx) return {convert(first), convert(last)};
            }
        }
    }
    // Prefer an endpoint closer than the existing crossing, but only within the search radius.
    const auto closer = [radius](const std::vector<ClosestLine> &lines, const Intersection &intersection, const Point &point) -> size_t {
        for (size_t i = 0; i < lines.size(); ++i) {
            const double old_distance = float_squared_norm(point - intersection.point);
            if (lines[i].border_idx == intersection.border_idx && old_distance <= double(radius) * double(radius) &&
                float_squared_norm(point - lines[i].point) < old_distance) return i;
        }
        return size_t(-1);
    };
    // Match intersections in their selected direction, then retain the first nearby segment on that ring.
    const auto same_boundary = [](const std::vector<ClosestLine> &lines, const std::vector<Intersection> &chain, bool reverse) -> size_t {
        for (size_t i = 0; i < chain.size(); ++i) {
            const Intersection &intersection = chain[reverse ? chain.size() - 1 - i : i];
            for (size_t j = 0; j < lines.size(); ++j)
                if (lines[j].border_idx == intersection.border_idx) return j;
        }
        return size_t(-1);
    };
    std::vector<Intersection> out = intersections;
    if (!out.empty() && !start_lines.empty()) {
        const size_t index = closer(start_lines, out.front(), start);
        if (index != size_t(-1)) out.front() = convert(start_lines[index]);
        else {
            const size_t same = same_boundary(start_lines, out, true);
            out.insert(out.begin(), convert(start_lines[same == size_t(-1) ? 0 : same]));
        }
    }
    if (!out.empty() && !end_lines.empty()) {
        const size_t index = closer(end_lines, out.back(), end);
        if (index != size_t(-1)) out.back() = convert(end_lines[index]);
        else {
            const size_t same = same_boundary(end_lines, out, false);
            out.push_back(convert(end_lines[same == size_t(-1) ? 0 : same]));
        }
    }
    return out;
}

static Direction get_shortest_direction(const Boundary &boundary, const Intersection &first, const Intersection &second, float length)
{
    assert(first.border_idx == second.border_idx);
    const Polygon &polygon = boundary.boundaries[first.border_idx];
    float a = first.distance, b = second.distance;
    assert(a >= 0.f && a <= length && b >= 0.f && b <= length);
    const bool reversed = a > b;
    if (reversed) std::swap(a, b);
    float forward = b - a, backward = a + length - b;
    if (reversed) std::swap(forward, backward);
    forward -= float_norm(first.point - polygon[first.line_idx]);
    backward -= float_norm(polygon[(first.line_idx + 1) % polygon.size()] - first.point);
    forward -= float_norm(polygon[(second.line_idx + 1) % polygon.size()] - second.point);
    backward -= float_norm(second.point - polygon[second.line_idx]);
    return forward < backward ? Direction::Forward : Direction::Backward;
}

static std::vector<TravelPoint> simplify_travel(const Boundary &boundary, const std::vector<TravelPoint> &travel)
{
    if (travel.size() < 3) return travel;
    FirstIntersectionVisitor visitor(boundary.grid);
    std::vector<TravelPoint> out;
    out.reserve(travel.size()); out.push_back(travel.front());
    Point current = travel.front().point;
    for (size_t i = 1; i < travel.size(); ++i) {
        TravelPoint next = travel[i];
        assert(current == out.back().point);
        // Coalesce near neighbours before shortcut removal, transferring protection when needed.
        if (i == 1 && current.coincides_with_epsilon(next.point)) {
            out.front().do_not_remove = next.do_not_remove; out.front().border_idx = next.border_idx; continue;
        }
        if (current.coincides_with_epsilon(next.point) && i + 1 < travel.size()) {
            if (next.do_not_remove && !out.back().do_not_remove) { out.back() = next; current = next.point; }
            continue;
        }
        visitor.pt_current = &current;
        if (!next.do_not_remove) {
            if (current.coincides_with_epsilon(next.point) && i + 1 < travel.size()) continue;
            for (size_t j = i + 1; j < travel.size(); ++j) {
                if (travel[j].point.coincides_with_epsilon(current)) { next = travel[j]; i = j; continue; }
                visitor.pt_next = &travel[j].point;
                boundary.grid.visit_cells_intersecting_line(current, *visitor.pt_next, visitor);
                if (!visitor.intersect) { next = travel[j]; i = j; }
                if (travel[j].do_not_remove) break;
            }
        }
        out.push_back(next); current = next.point;
    }
    if (out.size() > 2 && out.back().point.coincides_with_epsilon(out[out.size() - 2].point)) {
        out[out.size() - 2].point = out.back().point; out.pop_back();
    }
    if (out.size() > 2) {
        for (size_t i = 1; i < out.size(); ++i) assert(!out[i - 1].point.coincides_with_epsilon(out[i].point));
    }
    assert(out.front().point == travel.front().point && out.back().point == travel.back().point);
    return out;
}

static void set_intersection(Intersection &intersection, const Polygon &polygon, size_t edge, const Point &point)
{
    intersection.line_idx = edge;
    intersection.point = point;
    if (point == polygon.points[edge]) intersection.distance = 0;
    else if (point == polygon.points[(edge + 1) % polygon.size()]) {
        intersection.line_idx = (edge + 1) % polygon.size(); intersection.distance = 0;
    } else intersection.distance = float(polygon.points[edge].distance_to(point));
}

static void brute_force_jump(Intersection &first, Intersection &last, const Line &line,
                             const Polygon &a, const Polygon &b, double weight)
{
    double shortest = line.length() * (1 + weight), squared = shortest * shortest;
    for (int side = 0; side < 2; ++side) {
        Intersection &intersection = side == 0 ? first : last;
        const Polygon &polygon = side == 0 ? a : b;
        for (size_t i = 0; i < polygon.size(); ++i) {
            const Line edge(polygon.points[i], polygon.points[(i + 1) % polygon.size()]);
            if (edge.intersection(line, &intersection.point)) {
                set_intersection(intersection, polygon, i, intersection.point); break;
            }
        }
        if (intersection.line_idx == size_t(-1)) {
            double best = squared;
            for (size_t i = 0; i < polygon.size(); ++i) {
                Point point;
                const double distance = Line(polygon.points[i], polygon.points[(i + 1) % polygon.size()]).distance_to_squared(line.a, &point);
                if (distance < best) { best = distance; set_intersection(intersection, polygon, i, point); }
            }
        }
    }
    assert(first.line_idx != size_t(-1) && last.line_idx != size_t(-1));
    const Intersection saved_last = last;
    Point point;
    double distance = Line(b.points[last.line_idx], b.points[(last.line_idx + 1) % b.size()]).distance_to_squared(first.point, &point);
    if (distance < squared) {
        distance = std::sqrt(distance);
        distance += line.a.distance_to(first.point) * weight + point.distance_to(line.b) * weight;
        if (distance < shortest) { assert(distance > 0); shortest = distance; squared = shortest * shortest; set_intersection(last, b, last.line_idx, point); }
    }
    distance = Line(a.points[first.line_idx], a.points[(first.line_idx + 1) % a.size()]).distance_to_squared(saved_last.point, &point);
    if (distance < squared) {
        distance = std::sqrt(distance);
        distance += line.a.distance_to(point) * weight + saved_last.point.distance_to(line.b) * weight;
        if (distance < shortest) {
            last = saved_last; assert(distance > 0); shortest = distance; squared = shortest * shortest;
            set_intersection(first, a, first.line_idx, point);
        }
    }
    for (size_t i = 0; i < a.size(); ++i) {
        for (size_t j = 0; j < b.size(); ++j) {
            distance = Line(b.points[j], b.points[(j + 1) % b.size()]).distance_to_squared(a.points[i], &point);
            if (distance < squared) {
                distance = std::sqrt(distance);
                distance += line.a.distance_to(a.points[i]) * weight + point.distance_to(line.b) * weight;
                if (distance < shortest) {
                    assert(distance > 0); shortest = distance; squared = shortest * shortest;
                    set_intersection(first, a, i, a.points[i]); set_intersection(last, b, j, point);
                }
            }
            distance = Line(a.points[i], a.points[(i + 1) % a.size()]).distance_to_squared(b.points[j], &point);
            if (distance < squared) {
                distance = std::sqrt(distance);
                distance += line.a.distance_to(point) * weight + b.points[j].distance_to(line.b) * weight;
                if (distance < shortest) {
                    assert(distance > 0); shortest = distance; squared = shortest * shortest;
                    set_intersection(first, a, i, point); set_intersection(last, b, j, b.points[j]);
                }
            }
        }
    }
}

static std::vector<Intersection> segment_intersections(const Boundary &boundary, const Point &start, const Point &end, coord_t radius)
{
    std::vector<Intersection> out;
    out.reserve(boundary.boundaries.size());
    AllIntersectionsVisitor visitor(boundary.grid, out, Line(start, end));
    boundary.grid.visit_cells_intersecting_line(start, end, visitor);
    const Vec2d direction = to_vec2d(end - start);
    for (Intersection &intersection : out) {
        const float distance = float_norm(intersection.point - boundary.boundaries[intersection.border_idx][intersection.line_idx]);
        intersection.distance = boundary.boundaries_params[intersection.border_idx][intersection.line_idx] + distance;
    }
    std::sort(out.begin(), out.end(), [direction](const Intersection &a, const Intersection &b) {
        return to_vec2d(b.point - a.point).dot(direction) > 0.;
    });
    return extend_for_closest_lines(out, boundary, start, end, radius);
}

static void jump_between_island(Boundary &boundary, Point start, Point end, std::vector<Intersection> &intersections,
                                coord_t radius, double weight, RouterCounts &counts, const std::function<void()> &cancel)
{
    assert(intersections.size() > 1);
    if (boundary.islands[intersections.front().border_idx] == boundary.islands[intersections.back().border_idx]) return;
    ++counts.island_jumps;
    const double straight_distance = start.distance_to(end);
    size_t start_island = boundary.islands[intersections.front().border_idx];
    size_t end_island = boundary.islands[intersections.back().border_idx];
    // If one island's box contains the other, look for a hole whose box also contains it to use as the bridge ring.
    if (contains(boundary.bboxes[start_island], boundary.bboxes[end_island])) {
        for (size_t i = start_island + 1; i < boundary.islands.size() && boundary.islands[i] == start_island; ++i) {
            if (contains(boundary.bboxes[i], boundary.bboxes[end_island])) { start_island = i; break; }
        }
    }
    if (contains(boundary.bboxes[end_island], boundary.bboxes[start_island])) {
        for (size_t i = end_island + 1; i < boundary.islands.size() && boundary.islands[i] == end_island; ++i) {
            if (contains(boundary.bboxes[i], boundary.bboxes[start_island])) { end_island = i; break; }
        }
    }
    const Polygon &a = boundary.boundaries[start_island], &b = boundary.boundaries[end_island];
    Intersection first{start_island, size_t(-1), a.points.front(), 0, true};
    Intersection last{end_island, size_t(-1), b.points.front(), 0, true};
    double shortest = straight_distance * (1 + weight), squared = shortest * shortest;
    for (size_t island : {start_island, end_island}) {
        if (boundary.island_to_grid.find(int(island)) == boundary.island_to_grid.end()) {
            EdgeGrid::Grid &grid = boundary.island_to_grid[int(island)];
            grid.set_bbox(boundary.grid.bbox()); grid.create(boundary.boundaries[island], boundary.grid.resolution() * 4);
        }
    }
    EdgeGrid::Grid &start_grid = boundary.island_to_grid[int(start_island)];
    EdgeGrid::Grid &end_grid = boundary.island_to_grid[int(end_island)];
    size_t cell_distance = 1 + size_t(shortest / start_grid.resolution());
    Line start_end(start, end);
    // Seed bridge endpoints from direct-line crossings, trying epsilon extension when a crossing is missed.
    for (int side = 0; side < 2; ++side) {
        EdgeGrid::Grid &grid = side == 0 ? start_grid : end_grid;
        const Polygon &polygon = side == 0 ? a : b;
        Intersection &intersection = side == 0 ? first : last;
        FirstIntersectionVisitor visitor(grid);
        visitor.pt_current = &start; visitor.pt_next = &end;
        grid.visit_cells_intersecting_line(start, end, visitor);
        if (!visitor.intersect) {
            FirstEpsilonIntersectionVisitor epsilon(grid, side == 0 ? start_end.a : start_end.b,
                                                   side == 0 ? start_end.b : start_end.a);
            grid.visit_cells_intersecting_line(side == 0 ? start : end, side == 0 ? end : start, epsilon);
            visitor.intersect = epsilon.intersect;
            visitor.intersection_contour_idx = epsilon.intersection_contour_idx;
            visitor.intersection_line_idx = epsilon.intersection_line_idx;
            if (epsilon.intersect) {
                if (side == 0) { assert(start != start_end.a); start = start_end.a; }
                else { ++counts.arrival_epsilon; assert(end != start_end.b); end = start_end.b; }
            }
        }
        if (visitor.intersect) {
            assert(visitor.intersection_contour_idx == 0);
            intersection.line_idx = visitor.intersection_line_idx;
            assert(intersection.line_idx < polygon.size());
            if (start_end.intersection(Line(polygon.points[intersection.line_idx], polygon.points[(intersection.line_idx + 1) % polygon.size()]), &intersection.point))
                intersection.distance = float(polygon.points[intersection.line_idx].distance_to(intersection.point));
        }
    }
    if (first.line_idx >= a.size() || last.line_idx >= b.size()) {
        ++counts.brute_force;
        brute_force_jump(first, last, Line(start, end), a, b, weight);
    } else {
        Point from_start, from_end;
        double distance_start = Line(b.points[last.line_idx], b.points[(last.line_idx + 1) % b.size()]).distance_to_squared(first.point, &from_start);
        distance_start = start.distance_to(first.point) * weight + std::sqrt(distance_start) + from_start.distance_to(end) * weight;
        double distance_end = Line(a.points[first.line_idx], a.points[(first.line_idx + 1) % a.size()]).distance_to_squared(last.point, &from_end);
        distance_end = start.distance_to(from_end) * weight + std::sqrt(distance_end) + last.point.distance_to(end) * weight;
        if (distance_start <= distance_end) {
            assert(distance_start > 0); shortest = distance_start; squared = shortest * shortest;
            set_intersection(last, b, last.line_idx, from_start);
        } else {
            assert(distance_end > 0); shortest = distance_end; squared = shortest * shortest;
            set_intersection(first, a, first.line_idx, from_end);
        }
        // Prune cell pairs by the current score bound before comparing vertex-to-edge bridges.
        for (size_t sy = 0; sy < start_grid.rows(); ++sy) {
            cancel();
            const size_t min_y = sy > cell_distance ? sy - cell_distance : 0;
            const size_t max_y = std::min(1 + sy + cell_distance, end_grid.rows());
            for (size_t sx = 0; sx < start_grid.cols(); ++sx) {
                const size_t min_x = sx > cell_distance ? sx - cell_distance : 0;
                const size_t max_x = std::min(1 + sx + cell_distance, end_grid.cols());
                if (!start_grid.cell_has_data(coord_t(sy), coord_t(sx))) continue;
                const auto start_range = start_grid.cell_data_range(sy, sx);
                for (size_t ey = min_y; ey < max_y; ++ey) {
                    for (size_t ex = min_x; ex < max_x; ++ex) {
                        if (!end_grid.cell_has_data(coord_t(ey), coord_t(ex))) continue;
                        const auto end_range = end_grid.cell_data_range(ey, ex);
                        for (auto si = start_range.first; si != start_range.second; ++si) {
                            const Line start_line = start_grid.line(*si);
                            assert(si->first == 0);
                            for (auto ei = end_range.first; ei != end_range.second; ++ei) {
                                const Line end_line = end_grid.line(*ei);
                                assert(ei->first == 0);
                                Point point;
                                double distance = end_line.distance_to_squared(start_line.a, &point);
                                if (distance < squared) {
                                    distance = std::sqrt(distance);
                                    distance += start.distance_to(start_line.a) * weight + point.distance_to(end) * weight;
                                    if (distance < shortest) {
                                        assert(distance > 0); shortest = distance; squared = shortest * shortest;
                                        cell_distance = size_t(1 + shortest / start_grid.resolution());
                                        set_intersection(first, a, si->second, start_line.a);
                                        set_intersection(last, b, ei->second, point);
                                    }
                                }
                                distance = start_line.distance_to_squared(end_line.a, &point);
                                if (distance < squared) {
                                    distance = std::sqrt(distance);
                                    distance += start.distance_to(point) * weight + end_line.a.distance_to(end) * weight;
                                    if (distance < shortest) {
                                        assert(distance > 0); shortest = distance; squared = shortest * shortest;
                                        cell_distance = size_t(1 + shortest / start_grid.resolution());
                                        set_intersection(first, a, si->second, point);
                                        set_intersection(last, b, ei->second, end_line.a);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    assert(first.line_idx != size_t(-1) && last.line_idx != size_t(-1));
    // Recompute the approaches so ring following connects to the selected bridge endpoints.
    intersections = segment_intersections(boundary, start, first.point, radius);
    const std::vector<Intersection> tail = segment_intersections(boundary, last.point, end, radius);
    intersections.insert(intersections.end(), tail.begin(), tail.end());
}

} // namespace

Polyline route_on_boundary(Boundary &boundary, Point real_start, Point real_end, coord_t placement_spacing,
                           coord_t segment_radius, bool island_jump, double jump_weight, storage_handle *storage,
                           RouterCounts &counts, const std::function<void()> &cancel)
{
    if (boundary.boundaries.empty()) return Polyline{real_start, real_end};
    std::vector<TravelPoint> out;
    Point start = real_start, end = real_end;
    // Internal routing first places endpoints in the insets; external routing leaves their placement unchanged.
    if (placement_spacing > 0) {
        if (!boundary.grid.bbox().contains(start) || !boundary.grid.bbox().contains(end) ||
            !find_point_on_boundary(start, boundary, (placement_spacing * 3) / 2, counts) ||
            !find_point_on_boundary(end, boundary, (placement_spacing * 3) / 2, counts)) {
            // Try farther projections up to half the direct distance when normal inset placement fails.
            const double distance = start.distance_to(end);
            const EdgeGrid::Grid::ClosestPointResult first = boundary.grid.closest_point_signed_distance(start, distance / 2);
            const EdgeGrid::Grid::ClosestPointResult last = boundary.grid.closest_point_signed_distance(end, distance / 2);
            bool better = false;
            if (first.distance + last.distance < distance) {
                const EdgeGrid::Contour &a = boundary.grid.contours()[first.contour_idx];
                const EdgeGrid::Contour &b = boundary.grid.contours()[last.contour_idx];
                const Point new_start = Point(a.segment_start(first.start_point_idx)).interpolate(first.t, Point(a.segment_end(first.start_point_idx)));
                const Point new_end = Point(b.segment_start(last.start_point_idx)).interpolate(last.t, Point(b.segment_end(last.start_point_idx)));
                AnyIntersectionsVisitor visitor(boundary.to_avoid_grid, Polyline{start, new_start, new_end, end});
                if (boundary.to_avoid_grid.bbox().contains(start) && boundary.to_avoid_grid.bbox().contains(new_start))
                    boundary.to_avoid_grid.visit_cells_intersecting_line(start, new_start, visitor);
                if (boundary.to_avoid_grid.bbox().contains(new_start) && boundary.to_avoid_grid.bbox().contains(new_end))
                    boundary.to_avoid_grid.visit_cells_intersecting_line(new_start, new_end, visitor);
                if (boundary.to_avoid_grid.bbox().contains(new_end) && boundary.to_avoid_grid.bbox().contains(end))
                    boundary.to_avoid_grid.visit_cells_intersecting_line(new_end, end, visitor);
                if (visitor.has_intersection) {
                    ++counts.to_avoid;
                    // diff_pl retains parts outside to_avoid; accept the candidate only if its outside length is below half the direct path's.
                    Polylines paths = clipper::diff_pl(storage, {Polyline{start, end}}, boundary.to_avoid);
                    double old_distance = 0;
                    for (const Polyline &path : paths) old_distance += path.length();
                    if (old_distance > 0) {
                        paths = clipper::diff_pl(storage, {Polyline{start, new_start, new_end, end}}, boundary.to_avoid);
                        double new_distance = 0;
                        for (const Polyline &path : paths) new_distance += path.length();
                        if (new_distance < old_distance / 2) { better = true; start = new_start; end = new_end; }
                    }
                } else { better = true; start = new_start; end = new_end; }
            }
            // Scan consecutive ring edges so the unmatched endpoint projects onto the other endpoint's boundary.
            const auto search = [&boundary, distance](const EdgeGrid::Grid::ClosestPointResult &good, Point &bad) -> bool {
                double minimum = distance * distance;
                Point best = bad;
                const EdgeGrid::Contour &contour = boundary.grid.contours()[good.contour_idx];
                Point last_point(contour.back());
                for (size_t i = 0; i < contour.size(); ++i) {
                    Point nearest;
                    const double d = Line(last_point, Point(contour[i])).distance_to_squared(bad, &nearest);
                    if (d < minimum) { minimum = d; best = nearest; }
                    last_point = Point(contour[i]);
                }
                bad = best;
                return minimum < distance * distance;
            };
            if (!better && first.contour_idx == size_t(-1) && last.contour_idx != size_t(-1)) better = search(last, start);
            if (!better && last.contour_idx == size_t(-1) && first.contour_idx != size_t(-1)) better = search(first, end);
            if (!better) { cancel(); return Polyline{real_start, real_end}; }
        }
    }
    // Far endpoint placement may leave one working point beyond the grid's indexed cells.
    cancel();
    if (!boundary.grid.bbox().contains(start) || !boundary.grid.bbox().contains(end))
        return Polyline{real_start, real_end};
    std::vector<Intersection> intersections = segment_intersections(boundary, start, end, segment_radius);
    std::vector<TravelPoint> result{{start, -1}};
    if (intersections.size() > 1 && !boundary.islands.empty() && island_jump)
        jump_between_island(boundary, start, end, intersections, segment_radius, jump_weight, counts, cancel);
    for (auto first = intersections.begin(); first != intersections.end(); ++first) {
        cancel();
        const Intersection &a = *first;
        const Polygon &polygon = boundary.boundaries[a.border_idx];
        const auto stop = std::make_reverse_iterator(first) - 1;
        const auto reverse_last = std::find_if(intersections.rbegin(), stop, [&a](const Intersection &b) { return a.border_idx == b.border_idx; });
        result.push_back({get_middle_point_offset(polygon, a.line_idx, (a.line_idx + 1) % polygon.size(), a.point, coord_t(SCALED_EPSILON)), int(a.border_idx), a.do_not_remove});
        // Replace crossings on the same ring with a walk in the direction selected by float scores.
        if (reverse_last != stop) {
            const auto last = reverse_last.base() - 1;
            const Intersection &b = *last;
            const Direction direction = get_shortest_direction(boundary, a, b, boundary.boundaries_params[a.border_idx].back());
            if (direction == Direction::Forward) {
                for (int i = int(a.line_idx); i != int(b.line_idx); i = i + 1 < int(polygon.size()) ? i + 1 : 0)
                    result.push_back({get_polygon_vertex_offset(polygon, i + 1 == int(polygon.size()) ? 0 : i + 1, coord_t(SCALED_EPSILON)), int(a.border_idx)});
            } else {
                for (int i = int(a.line_idx); i != int(b.line_idx); i = i - 1 >= 0 ? i - 1 : int(polygon.size()) - 1)
                    result.push_back({get_polygon_vertex_offset(polygon, i, coord_t(SCALED_EPSILON)), int(a.border_idx)});
            }
            result.push_back({get_middle_point_offset(polygon, b.line_idx, (b.line_idx + 1) % polygon.size(), b.point, coord_t(SCALED_EPSILON)), int(b.border_idx), b.do_not_remove});
            first = last;
        }
    }
    result.push_back({end, -1});
    if (!intersections.empty()) result = simplify_travel(boundary, result);
    // Fuse two epsilon-near inward projections before reconnecting the real endpoints.
    if (start != real_start && end != real_end && result.size() == 2 && result.front().point.coincides_with_epsilon(result.back().point)) {
        ++counts.epsilon_merge;
        result.front().point = Line(result.front().point, result.back().point).midpoint(); result.pop_back();
    }
    if (start != real_start) out.push_back({real_start, -1});
    out.insert(out.end(), result.begin(), result.end());
    while (out.size() > 1 && out.front().point.coincides_with_epsilon(out[1].point)) out.erase(out.begin() + 1);
    if (end != real_end) out.push_back({real_end, -1});
    while (out.size() > 1 && out.back().point.coincides_with_epsilon(out[out.size() - 2].point)) out.erase(out.end() - 2);
    assert(out.size() > 1);
    for (size_t i = 1; i < out.size(); ++i) assert(!out[i - 1].point.coincides_with_epsilon(out[i].point));
    return to_polyline(out);
}

void TravelRouter::init_layer(storage_handle *storage, const Input &input, std::function<ExternalInput()> external_input,
                               std::function<void()> throw_if_canceled)
{
    m_storage = storage; m_input = &input; m_external_input = std::move(external_input);
    m_throw_if_canceled = std::move(throw_if_canceled); m_counts = RouterCounts{};
    m_throw_if_canceled(); m_boundaries.init_layer(storage, *input.target);
}

PluginCommon::work::Polyline TravelRouter::travel_to(const TravelContext &context, const PluginCommon::work::Point &point)
{
    using namespace PluginCommon::polygon_work;
    using PluginCommon::polygon_work::Point;
    using PluginCommon::polygon_work::Polyline;
    using PluginCommon::polygon_work::BoundingBox;
    assert(m_input);
    m_throw_if_canceled();
    const LayerInput &layer = *m_input->target;
    const Point origin = m_use_external_mp ? context.origin : Point(0, 0);
    const Point start = context.start + origin, end = point + origin;
    const Line travel(start, end);
    Polyline result;
    Vec2d startf = to_vec2d(start), endf = to_vec2d(end);
    Boundary *boundary = nullptr;
    coord_t spacing = layer.perimeter_spacing;
    // Select local internal rings or print-space external holes; without usable rings retain the direct travel.
    if (!m_use_external_mp && (layer.support || !m_boundaries.layer_slices_offset().empty())) {
        boundary = &m_boundaries.internal_boundary();
        if (boundary->boundaries.empty()) m_boundaries.build_internal(m_storage, *m_input, context.extruder_id);
    } else if (m_use_external_mp) {
        boundary = &m_boundaries.external_boundary(); spacing = 0;
        if (boundary->boundaries.empty()) m_boundaries.build_external(m_storage, layer, m_external_input());
    }
    // Route only when the segment reaches the boundary box; clamp outside working endpoints and restore the requested ones afterward.
    if (boundary && !boundary->boundaries.empty() && liang_barsky_line_clipping(startf, endf, boundary->bbox)) {
        Point nearest_start = start, nearest_end = end;
        const BoundingBox box(to_point(boundary->bbox.min), to_point(boundary->bbox.max));
        if (!boundary->bbox.contains(to_vec2d(start))) { ++m_counts.outside_bbox; nearest_start = nearest_point(box, start); }
        if (!boundary->bbox.contains(to_vec2d(end))) { ++m_counts.outside_bbox; nearest_end = nearest_point(box, end); }
        bool island_jump = false;
        double jump_weight = 0.;
        for (const RegionInput &region : layer.regions) {
            if (region.avoid_travel_island) {
                island_jump = true;
                jump_weight = region.avoid_travel_island_weight;
                break;
            }
        }
        result = route_on_boundary(*boundary, nearest_start, nearest_end, spacing, 2 * layer.perimeter_spacing,
                                   island_jump, jump_weight, m_storage, m_counts, m_throw_if_canceled);
        assert(result.size() > 1);
        for (size_t i = 1; i < result.size(); ++i) assert(!result.points[i - 1].coincides_with_epsilon(result.points[i]));
        result.points.front() = start; result.points.back() = end;
    }
    if (result.empty()) result = Polyline{start, end};
    assert(result.size() > 1);
    for (size_t i = 1; i < result.size(); ++i) assert(!result.points[i - 1].coincides_with_epsilon(result.points[i]));
    if (result.size() > 2) ++m_counts.deviated;
    if (context.max_detour > 0) {
        const double direct = travel.length();
        const double detour = result.length() - direct;
        const double maximum = context.max_detour_is_percent ? direct * 0.01 * context.max_detour : scale_d(context.max_detour);
        if (detour > maximum) { result = Polyline{start, end}; ++m_counts.detour_rejected; }
    }
    // Remove the planning origin so the returned path stays in the target's local coordinates.
    if (m_use_external_mp) {
        for (Point &p : result.points) p -= origin;
    }
    assert(result.size() > 1);
    for (size_t i = 1; i < result.size(); ++i) assert(!result.points[i - 1].coincides_with_epsilon(result.points[i]));
    return result;
}
}}}
