///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef plugins_common_polygon_work_buffers_hpp_
#define plugins_common_polygon_work_buffers_hpp_

/*
Mutable polygon buffers for plugin algorithms
============================================

Plugin algorithms use these owned point arrays when read-only API geometry must
be changed, for example while connecting infill paths. Polygon stores a closed
ring without a repeated closing vertex. ExPolygon groups an outer contour with
its holes. ContourView instead borrows an API or work point array for reading;
keep that array alive and do not reallocate it while the view is used.

Build work rings from points, change them locally, then use ClipperHelpers.hpp
when API geometry operations or stored results are needed. Coordinates use
scaled units. Point stores integer coordinates, Vec2f stores floats, and Vec2d,
Linef and BoundingBoxf store doubles. Length and area queries return double.
line_alg::distance_to_squared measures squared distance to a segment.
The core is the existing Slic3r implementation. These work types reproduce its
rounding and tolerance rules so ported algorithms match the legacy result.

    polygon_work::Polygon ring;
    ring.points = {{0, 0}, {100, 0}, {100, 100}, {0, 100}};
    ring.make_counter_clockwise();
    double area = ring.area();

Travel routing also uses the integer Line operations, ring containment,
nearest-point and box queries below. Line rounds projected points with
std::round; the integer line_alg overload truncates them toward zero. Both measure distance before
converting the projection to integers. Ring projection uses floor(x + 0.5).
core_area starts with the closing edge and accumulates signed areas in core
order; its ExPolygon overload adds signed hole areas. ExPolygon::area instead
subtracts the absolute hole areas and retains its own accumulation order.
Matching the core accumulation order also preserves the legacy area results.
*/

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_def.h"
#include "libslic3r/Api/plugin/c/slic3r_geometry.h"
#include "WorkBuffers.hpp"

namespace slic3r_api { namespace PluginCommon {
namespace polygon_work {

using std::size_t;
using slic3r_api::PluginCommon::work::Vec2d;
using slic3r_api::PluginCommon::work::Point;
using slic3r_api::PluginCommon::work::Points;
using slic3r_api::PluginCommon::work::BoundingBox;
using slic3r_api::PluginCommon::work::MultiPoint;
using slic3r_api::PluginCommon::work::Polyline;
using slic3r_api::PluginCommon::work::Polylines;
using slic3r_api::PluginCommon::work::perp;
using slic3r_api::PluginCommon::work::is_approx;
using slic3r_api::PluginCommon::work::to_vec2d;
using slic3r_api::PluginCommon::work::to_point;
using slic3r_api::PluginCommon::work::lerp;
using slic3r_api::PluginCommon::work::douglas_peucker_points;
using slic3r_api::PluginCommon::work::align_to_grid;
using slic3r_api::PluginCommon::work::ensure_valid;

// Integer segment operations preserve the core's distinct truncation and rounding rules.
struct Line
{
    Point a;
    Point b;
    // Construct a zero-length segment at the origin.
    Line() = default;
    // Store the two segment endpoints.
    Line(const Point &first, const Point &second) : a(first), b(second) {}
    // Return the endpoint distance in double precision.
    double length() const { return (to_vec2d(b) - to_vec2d(a)).norm(); }
    // Return the integer midpoint, truncating division toward zero.
    Point midpoint() const { return (a + b) / 2; }
    // Find an intersection of nonparallel segments, rounding integer coordinates.
    bool intersection(const Line &other, Point *result) const {
        const Vec2d v1 = to_vec2d(b - a);
        const Vec2d v2 = to_vec2d(other.b - other.a);
        const double denom = v1.x() * v2.y() - v1.y() * v2.x();
        if (std::abs(denom) < EPSILON)
            return false;
        const Vec2d v12 = to_vec2d(a - other.a);
        const double t1 = (v2.x() * v12.y() - v2.y() * v12.x()) / denom;
        const double t2 = (v1.x() * v12.y() - v1.y() * v12.x()) / denom;
        if (t1 < 0. || t1 > 1. || t2 < 0. || t2 > 1.)
            return false;
        const Vec2d point = to_vec2d(a) + t1 * v1;
        *result = Point(coord_t(std::round(point.x())), coord_t(std::round(point.y())));
        return true;
    }
    // Return squared distance before rounding the optional projection; a zero-length segment projects to a.
    double distance_to_squared(const Point &point, Point *nearest = nullptr) const {
        const Vec2d v = to_vec2d(b - a);
        const Vec2d va = to_vec2d(point - a);
        const double l2 = v.squaredNorm();
        if (l2 == 0.) {
            if (nearest) *nearest = a;
            return va.squaredNorm();
        }
        const double t = va.dot(v);
        if (t <= 0.) {
            if (nearest) *nearest = a;
            return va.squaredNorm();
        } else if (t >= l2) {
            if (nearest) *nearest = b;
            return to_vec2d(point - b).squaredNorm();
        }
        const Vec2d w = (t / l2) * v;
        if (nearest) {
            const Vec2d projected = to_vec2d(a) + w;
            *nearest = Point(coord_t(std::round(projected.x())), coord_t(std::round(projected.y())));
        }
        return (w - va).squaredNorm();
    }
    // Move the second endpoint beyond itself by distance; a zero-length segment is unchanged.
    void extend_end(double distance) {
        const double len = this->length();
        Point point = b;
        if (a.x() != b.x()) point.x() = coord_t(b.x() + (a.x() - b.x()) * -distance / len);
        if (a.y() != b.y()) point.y() = coord_t(b.y() + (a.y() - b.y()) * -distance / len);
        b = point;
    }
};

// Test two non-degenerate segments with integer cross products, including collinear overlap.
inline bool segments_intersect(const Point &a, const Point &b, const Point &c, const Point &d)
{
    assert(a != b && c != d);
    // Both endpoints must lie on opposite sides, or on the supporting line.
    const auto signs = [](const Point &p, const Point &q, const Point &r, const Point &s) {
        const Point v = q - p;
        const Point w = r - p;
        const Point z = s - p;
        const int64_t x = int64_t(v.x()) * int64_t(w.y()) - int64_t(v.y()) * int64_t(w.x());
        const int64_t y = int64_t(v.x()) * int64_t(z.y()) - int64_t(v.y()) * int64_t(z.x());
        return std::make_pair((x > 0) ? 1 : ((x < 0) ? -1 : 0), (y > 0) ? 1 : ((y < 0) ? -1 : 0));
    };
    const std::pair<int, int> s1 = signs(a, b, c, d);
    const std::pair<int, int> s2 = signs(c, d, a, b);
    const int t1 = s1.first * s1.second;
    const int t2 = s2.first * s2.second;
    if (t1 <= 0 && t2 <= 0) {
        if (t1 != 0 || t2 != 0) return true;
        assert((s1.first == 0 && s1.second == 0) == (s2.first == 0 && s2.second == 0));
        // Collinear supporting lines still need overlapping intervals to share a point.
        if (s1.first == 0 && s1.second == 0) {
            const Point v = b - a;
            const int axis = std::abs(v.x()) > std::abs(v.y()) ? 0 : 1;
            coord_t i = a(axis), j = b(axis), k = c(axis), l = d(axis);
            if (i > j) std::swap(i, j);
            if (k > l) std::swap(k, l);
            return (k >= i && k <= j) || (i >= k && i <= l);
        }
    }
    return false;
}

// Return whether a hit exists; write the rounded hit nearest line.a, including the closing edge.
// Leave *result unchanged on a miss; result must be non-null when a hit is found.
inline bool first_intersection(const Points &points, const Line &line, Point *result)
{
    if (points.size() < 2) return false;
    bool found = false;
    double dmin = 0.;
    Line edge(points.back(), points.front());
    size_t start = 0;
    if (edge.a == edge.b) { start = 1; edge.a = edge.b; }
    for (size_t i = start; i < points.size(); ++i) {
        edge.b = points[i];
        Point intersection;
        if (edge.intersection(line, &intersection)) {
            const double distance = to_vec2d(line.a - intersection).squaredNorm();
            if (!found || distance < dmin) { found = true; dmin = distance; *result = intersection; }
        }
        edge.a = edge.b;
    }
    return found;
}

// Borrow the nearest vertex, keeping the first vertex on equal distances; return null for an empty path.
inline const Point *closest_point(const Points &points, const Point &point)
{
    if (points.empty()) return nullptr;
    size_t best = 0;
    double distance = to_vec2d(point - points.front()).norm();
    for (size_t i = 1; i < points.size(); ++i) {
        const double candidate = to_vec2d(points[i] - point).norm();
        if (candidate < distance) { distance = candidate; best = i; }
    }
    return &points[best];
}

// Return the nearest ring point and its vertex index or edge-start index; ties keep traversal order.
// Feet use floor(x + 0.5); an empty ring returns {point, size_t(-1)}.
inline std::pair<Point, size_t> point_projection(const Points &points, const Point &point)
{
    size_t index = size_t(-1);
    Point projection = point;
    double minimum = std::numeric_limits<double>::max();
    for (size_t i = 0; i < points.size(); ++i) {
        const Point &p0 = points[i];
        const size_t next = i + 1 == points.size() ? 0 : i + 1;
        const Point &p1 = points[next];
        double distance = to_vec2d(point - p0).norm();
        if (distance < minimum) { minimum = distance; projection = p0; index = i; }
        distance = to_vec2d(point - p1).norm();
        if (distance < minimum) { minimum = distance; projection = p1; index = next; }
        const Vec2d v1 = to_vec2d(p1 - p0);
        const double divisor = v1.squaredNorm();
        if (divisor > 0.) {
            const Vec2d v2 = to_vec2d(point - p0);
            const double t = v1.dot(v2) / divisor;
            if (t > 0. && t < 1.) {
                const Point foot(coord_t(std::floor(double(p0.x()) + t * v1.x() + 0.5)),
                                 coord_t(std::floor(double(p0.y()) + t * v1.y() + 0.5)));
                distance = to_vec2d(point - foot).norm();
                if (distance < minimum) { minimum = distance; projection = foot; index = i; }
            }
        }
    }
    return {projection, index};
}

// Double precision bounding box, mirrors BoundingBoxf used by the connect helpers.
struct BoundingBoxf
{
    Vec2d min;
    Vec2d max;
    bool  defined = false;

    // Initialize an undefined box at the origin.
    BoundingBoxf() = default;
    // Store the two corners; define the box only when both coordinate spans are positive.
    BoundingBoxf(const Vec2d &pmin, const Vec2d &pmax) : min(pmin), max(pmax),
        defined(pmin.x() < pmax.x() && pmin.y() < pmax.y()) {}

    // Clear the defined flag and reset both corners to zero.
    void reset() {
        this->defined = false;
        this->min = Vec2d(0., 0.);
        this->max = Vec2d(0., 0.);
    }
    // Report whether both corners coincide.
    bool empty() const { return min == max; }

    // Extend these bounds to include the supplied point or defined box.
    void merge(const Vec2d &point) {
        if (!defined) {
            min = max = point;
            defined = true;
        } else {
            min.vx = std::min(min.vx, point.vx);
            min.vy = std::min(min.vy, point.vy);
            max.vx = std::max(max.vx, point.vx);
            max.vy = std::max(max.vy, point.vy);
        }
    }
    void merge(const BoundingBoxf &other) {
        if (other.defined) {
            this->merge(other.min);
            this->merge(other.max);
        }
    }

    // Move the lower bounds by -delta and upper bounds by delta; a negative delta shrinks the box.
    // Leave the defined flag unchanged.
    void offset(coordf_t delta) {
        const Vec2d v{ double(delta), double(delta) };
        min = min - v;
        max = max + v;
    }
    // Return a copy offset by delta; a negative delta shrinks its bounds without changing the defined flag.
    BoundingBoxf inflated(coordf_t delta) const {
        BoundingBoxf out(*this);
        out.offset(delta);
        return out;
    }

    // Return the midpoint of the two box corners.
    Vec2d center() const { return (min + max) / 2.; }
    // Return the coordinate span between box corners.
    Vec2d size() const { return max - min; }

    // Test inclusive coordinate containment in these bounds.
    bool contains(const Vec2d &point) const {
        return point.x() >= min.x() && point.x() <= max.x() && point.y() >= min.y() && point.y() <= max.y();
    }
    // Test whether the inclusive coordinate bounds overlap.
    bool overlap(const BoundingBoxf &other) const {
        return !(this->max.x() < other.min.x() || this->min.x() > other.max.x() ||
                 this->max.y() < other.min.y() || this->min.y() > other.max.y());
    }
};

// Double precision line segment and distance helper (core Linef/line_alg).
struct Linef
{
    Vec2d a;
    Vec2d b;
    // Initialize a double-coordinate segment from its endpoints, or their default values.
    Linef() = default;
    Linef(const Vec2d &pa, const Vec2d &pb) : a(pa), b(pb) {}
};

namespace line_alg {
// Return squared distance before truncating the required projection; a zero-length segment projects to line.a.
inline double distance_to_squared(const Line &line, const Point &point, Point *nearest)
{
    const Vec2d v = to_vec2d(line.b - line.a);
    const Vec2d va = to_vec2d(point - line.a);
    const double l2 = v.squaredNorm();
    if (l2 == 0.) {
        *nearest = line.a;
        return va.squaredNorm();
    }
    const double t = va.dot(v);
    if (t <= 0.) {
        *nearest = line.a;
        return va.squaredNorm();
    } else if (t >= l2) {
        *nearest = line.b;
        return to_vec2d(point - line.b).squaredNorm();
    }
    const Vec2d w = (t / l2) * v;
    *nearest = to_point(to_vec2d(line.a) + w);
    return (w - va).squaredNorm();
}

// Return squared distance to the segment, treating coincident endpoints as one point.
inline double distance_to_squared(const Linef &line, const Vec2d &point)
{
    const Vec2d  v  = line.b - line.a;
    const Vec2d  va = point - line.a;
    const double l2 = v.squaredNorm();
    if (l2 == 0.0)
        return va.squaredNorm();
    const double t = va.dot(v);
    if (t <= 0.0)
        return va.squaredNorm();
    else if (t >= l2)
        return (point - line.b).squaredNorm();
    const Vec2d w = (t / l2) * v;
    return (w - va).squaredNorm();
}
} // namespace line_alg

struct Vec2f
{
    float vx = 0.f;
    float vy = 0.f;
    // Initialize a float-coordinate vector from its components, or zero.
    Vec2f() = default;
    Vec2f(float x, float y) : vx(x), vy(y) {}
    // Return the Euclidean vector length in double precision.
    double norm() const { return std::sqrt(double(vx) * double(vx) + double(vy) * double(vy)); }
};

// Return the component-wise vector difference.
inline Vec2f operator-(const Vec2f &lhs, const Vec2f &rhs) { return Vec2f(lhs.vx - rhs.vx, lhs.vy - rhs.vy); }
// Return the component-wise vector sum.
inline Vec2f operator+(const Vec2f &lhs, const Vec2f &rhs) { return Vec2f(lhs.vx + rhs.vx, lhs.vy + rhs.vy); }
// Return the vector scaled by the supplied factor.
inline Vec2f operator*(float factor, const Vec2f &vec) { return Vec2f(factor * vec.vx, factor * vec.vy); }
inline Vec2f operator*(const Vec2f &vec, float factor) { return Vec2f(factor * vec.vx, factor * vec.vy); }

// Convert integer point components to float coordinates.
inline Vec2f to_vec2f(const Point &point) { return Vec2f(float(point.x()), float(point.y())); }
// Interpolate the vectors; t must be within epsilon of the interval [0, 1].
inline Vec2f lerp(const Vec2f &a, const Vec2f &b, float t) {
    assert((t >= -EPSILON) && (t <= 1.f + float(EPSILON)));
    return (1.f - t) * a + t * b;
}

// ---------------------------------------------------------------------------
// Point containers.
// ---------------------------------------------------------------------------

// A contour view borrows either an API point array or an internal work array.
// It returns c_point values so callers never reinterpret one point type as the
// other and no source contour is copied while it is only being read.
class ContourView
{
public:
    // Borrow the supplied point array, or initialize an empty view.
    ContourView() = default;
    explicit ContourView(const multipoint_const_view &view) : m_api_points(view.array), m_size(view.size) {}
    explicit ContourView(const Points &points) : m_work_points(points.data()), m_size(points.size()) {}

    // Return a view of the same array limited to no more than its current point count.
    ContourView with_size(size_t size) const {
        ContourView out(*this);
        assert(size <= m_size);
        out.m_size = size;
        return out;
    }

    // Return the number of borrowed points.
    size_t size() const { return m_size; }
    // Report whether the sequence has no points.
    bool empty() const { return m_size == 0; }

    // Return the point at an in-range index by value.
    c_point operator[](size_t idx) const {
        assert(idx < m_size);
        if (m_api_points != nullptr)
            return m_api_points[idx];
        const Point &point = m_work_points[idx];
        return c_point{ point.x(), point.y() };
    }
    // Return the first point by value; the sequence must be nonempty.
    c_point front() const {
        assert(!empty());
        return (*this)[0];
    }
    // Return the last point by value; the sequence must be nonempty.
    c_point back() const {
        assert(!empty());
        return (*this)[m_size - 1];
    }

private:
    const c_point *m_api_points = nullptr;
    const Point *m_work_points = nullptr;
    size_t m_size = 0;
};

using ContourViews = std::vector<ContourView>;

// Test exact equality of the two point coordinates.
inline bool same_point(c_point lhs, c_point rhs) { return lhs.x == rhs.x && lhs.y == rhs.y; }
inline bool same_point(const Point &lhs, c_point rhs) { return lhs.x() == rhs.x && lhs.y() == rhs.y; }

// Report whether the box is undefined or both corners coincide.
inline bool empty(const BoundingBox &box) { return !box.defined || box.min == box.max; }

// Return coordinate bounds containing the supplied points or rings.
inline BoundingBox get_extents(const Points &points) {
    BoundingBox out;
    out.merge(points);
    return out;
}
inline BoundingBox get_extents(const ContourView &contour) {
    BoundingBox out;
    for (size_t idx = 0; idx < contour.size(); ++idx)
        out.merge(contour[idx]);
    return out;
}

// ---------------------------------------------------------------------------
// Polygon / ExPolygon.
// ---------------------------------------------------------------------------

struct Polygon;
// Test the turn at the lowest leftmost vertex; require at least three vertices.
inline bool is_ccw(const Polygon &poly);
// Test polygon containment, returning border_result for a point on an edge.
inline bool contains(const Polygon &poly, const Point &point, bool border_result = true);

struct Polygon : MultiPoint
{
    using MultiPoint::MultiPoint;

    // Return whether a hit exists and write the rounded hit nearest line.a; leave *point unchanged on a miss.
    bool first_intersection(const Line &line, Point *point) const {
        return polygon_work::first_intersection(points, line, point);
    }
    // Borrow the closest vertex, or return null for an empty polygon.
    const Point *closest_point(const Point &point) const { return polygon_work::closest_point(points, point); }
    // Return the ring projection and vertex or edge-start index, keeping traversal order on ties.
    // An empty ring returns {point, size_t(-1)}; projected feet use floor(x + 0.5).
    std::pair<Point, size_t> point_projection(const Point &point) const {
        return polygon_work::point_projection(points, point);
    }
    // Leave inputs with fewer than three vertices unchanged; otherwise simplify with a repeated closing vertex.
    // Clear a result shorter than three entries, or remove its closing copy (three entries leave two vertices).
    void douglas_peucker(coord_t tolerance) {
        if (size() < 3) return;
        // Repeat the first vertex so open-path simplification includes the closing edge.
        points.push_back(points.front());
        MultiPoint::douglas_peucker(tolerance);
        assert(points.size() > 1);
        if (points.size() < 3) points.clear();
        else {
            assert(points.front().coincides_with(points.back()));
            points.pop_back();
            assert(!points.front().coincides_with_epsilon(points.back()));
            assert(points.size() > 1);
        }
    }

    // Return signed ring area, or zero for fewer than three vertices.
    double area() const {
        if (points.size() < 3)
            return 0.;
        double out = 0.;
        for (size_t i = 0; i < points.size(); ++i) {
            const Point &p1 = points[i];
            const Point &p2 = points[(i + 1) % points.size()];
            out += double(p1.x()) * double(p2.y()) - double(p2.x()) * double(p1.y());
        }
        return 0.5 * out;
    }
    // Reverse the ring only when its signed area is negative.
    void make_counter_clockwise() {
        if (this->area() < 0.)
            this->reverse();
    }
    // Reverse the ring only when its signed area is positive.
    void make_clockwise() {
        if (this->area() > 0.)
            this->reverse();
    }
    // Report whether the ring turns counterclockwise at its lowest leftmost vertex.
    bool is_counter_clockwise() const { return is_ccw(*this); }
    // Return the inverse of the counterclockwise orientation test.
    bool is_clockwise() const { return !this->is_counter_clockwise(); }
};

using Polygons = std::vector<Polygon>;

// Accumulate from the closing edge first, matching core Polygon::area and its Debug Clipper-area consistency check.
inline double core_area(const Polygon &polygon)
{
    double area = 0.;
    if (polygon.points.size() >= 3) {
        Vec2d previous = to_vec2d(polygon.points.back());
        for (const Point &point : polygon.points) {
            const Vec2d current = to_vec2d(point);
            area += previous.x() * current.y() - previous.y() * current.x();
            previous = current;
        }
    }
#ifndef NDEBUG
    // Retain the core's independent Clipper area check without adding a dependency on ClipperHelpers.
    double clipper_area = 0.;
    const int size = int(polygon.size());
    if (size >= 3) {
        for (int i = 0, j = size - 1; i < size; ++i) {
            clipper_area += (double(polygon[j].x()) + polygon[i].x()) *
                            (double(polygon[j].y()) - polygon[i].y());
            j = i;
        }
    }
    assert(std::fabs(-clipper_area * 0.5 - 0.5 * area) < SCALED_EPSILON * SCALED_EPSILON * 1.);
#endif
    return 0.5 * area;
}

// Test ring containment; use border_result on an edge and return false for fewer than three vertices.
inline bool contains(const Polygon &poly, const Point &point, bool border_result)
{
    int result = 0;
    if (poly.size() < 3) return false;
    Point current = poly.points.front();
    for (size_t i = 1; i <= poly.size(); ++i) {
        const Point next = i == poly.size() ? poly.points.front() : poly.points[i];
        if (next.y() == point.y() && (next.x() == point.x() ||
            (current.y() == point.y() && ((next.x() > point.x()) == (current.x() < point.x())))))
            return border_result;
        if ((current.y() < point.y()) != (next.y() < point.y())) {
            if (current.x() >= point.x() && next.x() > point.x()) result = 1 - result;
            else if (current.x() >= point.x() || next.x() > point.x()) {
                const double cross = double(current.x() - point.x()) * double(next.y() - point.y()) -
                                     double(next.x() - point.x()) * double(current.y() - point.y());
                if (cross == 0.) return border_result;
                if ((cross > 0.) == (next.y() > current.y())) result = 1 - result;
            }
        }
        current = next;
    }
    return result != 0;
}

struct ExPolygon
{
    Polygon  contour;
    Polygons holes;

    // Test the contour and exclude hole interiors; either ring boundary returns border_result.
    bool contains(const Point &point, bool border_result = true) const {
        if (!polygon_work::contains(contour, point, border_result)) return false;
        for (const Polygon &hole : holes)
            if (polygon_work::contains(hole, point, !border_result)) return false;
        return true;
    }

    // Return contour area minus the absolute area of each hole.
    double area() const {
        double out = contour.area();
        for (const Polygon &hole : holes)
            out -= std::abs(hole.area());
        return out;
    }
    // Return bounds of the outer contour only.
    BoundingBox bounding_box() const { return contour.bounding_box(); }
    // Report whether the outer contour contains at least three vertices.
    bool is_valid() const { return contour.points.size() > 2; }
    // Assert that the outer contour has at least three vertices.
    void assert_valid() const { assert(this->is_valid()); }

    // Return one outer contour plus the number of holes.
    size_t num_contours() const { return this->holes.size() + 1; }
    // Access the outer contour at zero or hole idx - 1; the index must be in range.
    Polygon &contour_or_hole(size_t idx) { return (idx == 0) ? this->contour : this->holes[idx - 1]; }
    const Polygon &contour_or_hole(size_t idx) const { return (idx == 0) ? this->contour : this->holes[idx - 1]; }
};

// Return the contour area plus signed hole areas, using the core accumulation order and double arithmetic.
inline double core_area(const ExPolygon &polygon)
{
    double area = core_area(polygon.contour);
    for (const Polygon &hole : polygon.holes)
        area -= -core_area(hole);
    return area;
}

using ExPolygons = std::vector<ExPolygon>;

// Copy the outer contour followed by its holes into a ring collection.
inline Polygons to_polygons(const ExPolygon &expolygon) {
    Polygons out;
    out.push_back(expolygon.contour);
    out.insert(out.end(), expolygon.holes.begin(), expolygon.holes.end());
    return out;
}

// Flatten contours followed by their holes, preserving collection and ring order.
inline Polygons to_polygons(const ExPolygons &expolygons) {
    Polygons out;
    for (const ExPolygon &expolygon : expolygons) {
        out.push_back(expolygon.contour);
        out.insert(out.end(), expolygon.holes.begin(), expolygon.holes.end());
    }
    return out;
}

// Test inclusive containment of both corners of another box.
inline bool contains(const BoundingBox &box, const BoundingBox &other) {
    return box.contains(other.min) && box.contains(other.max);
}

// Test whether a segment crosses any box edge, with the core's endpoint shortcut.
inline bool cross(const BoundingBox &box, const Line &line) {
    assert(box.defined || box.min.x() >= box.max.x() || box.min.y() >= box.max.y());
    if (box.contains(line.a) != box.contains(line.b)) return true;
    Point intersection;
    return Line(box.min, Point(box.min.x(), box.max.y())).intersection(line, &intersection) ||
           Line(box.min, Point(box.max.x(), box.min.y())).intersection(line, &intersection) ||
           Line(box.max, Point(box.min.x(), box.max.y())).intersection(line, &intersection) ||
           Line(box.max, Point(box.max.x(), box.min.y())).intersection(line, &intersection);
}

// Test whether a path has both inside and outside vertices or intersects a box edge.
inline bool cross(const BoundingBox &box, const Polyline &path) {
    size_t inside = 0, outside = 0;
    for (const Point &point : path.points) {
        if (box.contains(point)) ++inside;
        else ++outside;
    }
    if (inside > 0 && outside > 0) return true;
    for (size_t i = 1; i < path.size(); ++i)
        if (cross(box, Line(path[i - 1], path[i]))) return true;
    return false;
}

// Return the nearest point on the box boundary, retaining the first edge on ties.
inline Point nearest_point(const BoundingBox &box, const Point &point) {
    assert(box.defined || box.min.x() >= box.max.x() || box.min.y() >= box.max.y());
    Point best = box.min;
    double minimum = Line(box.min, Point(box.min.x(), box.max.y())).distance_to_squared(point, &best);
    const Line edges[] = {Line(Point(box.min.x(), box.max.y()), box.max),
                          Line(box.max, Point(box.max.x(), box.min.y())),
                          Line(Point(box.max.x(), box.min.y()), box.min)};
    for (const Line &edge : edges) {
        Point candidate;
        const double distance = edge.distance_to_squared(point, &candidate);
        if (distance < minimum) { minimum = distance; best = candidate; }
    }
    return best;
}

// ---------------------------------------------------------------------------
// Free helpers used by the ported code.
// ---------------------------------------------------------------------------

template<class T>
// Append copied or moved elements; the move overload then clears the source.
inline void append(std::vector<T> &dst, const std::vector<T> &src) {
    dst.insert(dst.end(), src.begin(), src.end());
}
template<class T>
inline void append(std::vector<T> &dst, std::vector<T> &&src) {
    dst.insert(dst.end(), std::make_move_iterator(src.begin()), std::make_move_iterator(src.end()));
    src.clear();
}

// Port of the Utils.hpp modulo index / value helpers.
template<typename INDEX_TYPE>
// Return the preceding index with wraparound; the count must be positive.
inline INDEX_TYPE prev_idx_modulo(INDEX_TYPE idx, const INDEX_TYPE count)
{
    if (idx == 0)
        idx = count;
    return --idx;
}

template<typename INDEX_TYPE>
// Return the following index with wraparound; the count must be positive.
inline INDEX_TYPE next_idx_modulo(INDEX_TYPE idx, const INDEX_TYPE count)
{
    if (++idx == count)
        idx = 0;
    return idx;
}

template<typename CONTAINER_TYPE>
// Return the preceding index with wraparound; the count must be positive.
inline typename CONTAINER_TYPE::size_type prev_idx_modulo(typename CONTAINER_TYPE::size_type idx, const CONTAINER_TYPE &container)
{
    return prev_idx_modulo(idx, container.size());
}

template<typename CONTAINER_TYPE>
// Return the following index with wraparound; the count must be positive.
inline typename CONTAINER_TYPE::size_type next_idx_modulo(typename CONTAINER_TYPE::size_type idx, const CONTAINER_TYPE &container)
{
    return next_idx_modulo(idx, container.size());
}

template<typename CONTAINER_TYPE>
// Borrow the preceding element with wraparound in a nonempty container.
inline const typename CONTAINER_TYPE::value_type &prev_value_modulo(typename CONTAINER_TYPE::size_type idx, const CONTAINER_TYPE &container)
{
    return container[prev_idx_modulo(idx, container.size())];
}

template<typename CONTAINER_TYPE>
inline typename CONTAINER_TYPE::value_type &prev_value_modulo(typename CONTAINER_TYPE::size_type idx, CONTAINER_TYPE &container)
{
    return container[prev_idx_modulo(idx, container.size())];
}

template<typename CONTAINER_TYPE>
// Borrow the following element with wraparound in a nonempty container.
inline const typename CONTAINER_TYPE::value_type &next_value_modulo(typename CONTAINER_TYPE::size_type idx, const CONTAINER_TYPE &container)
{
    return container[next_idx_modulo(idx, container.size())];
}

template<typename CONTAINER_TYPE>
inline typename CONTAINER_TYPE::value_type &next_value_modulo(typename CONTAINER_TYPE::size_type idx, CONTAINER_TYPE &container)
{
    return container[next_idx_modulo(idx, container.size())];
}

// Orientation of three points, exact for coord_t like the core helper.
enum Orientation
{
    ORIENTATION_CCW = 1,
    ORIENTATION_CW = -1,
    ORIENTATION_COLINEAR = 0
};

// Return the orientation of three points using integer products where the coordinate test permits.
inline Orientation orient(const Point &a, const Point &b, const Point &c) {
    if (a.x() <= 0xffffffff && b.x() <= 0xffffffff && c.x() <= 0xffffffff && a.y() <= 0xffffffff &&
        b.y() <= 0xffffffff && c.y() <= 0xffffffff) {
        const int64_t u = int64_t(b.x()) * int64_t(c.y()) - int64_t(b.y()) * int64_t(c.x());
        const int64_t v = int64_t(a.x()) * int64_t(c.y()) - int64_t(a.y()) * int64_t(c.x());
        const int64_t w = int64_t(a.x()) * int64_t(b.y()) - int64_t(a.y()) * int64_t(b.x());
        const int64_t d = u - v + w;
        return (d > 0) ? ORIENTATION_CCW : ((d == 0) ? ORIENTATION_COLINEAR : ORIENTATION_CW);
    }
    const double u = double(b.x()) * double(c.y()) - double(b.y()) * double(c.x());
    const double v = double(a.x()) * double(c.y()) - double(a.y()) * double(c.x());
    const double w = double(a.x()) * double(b.y()) - double(a.y()) * double(b.x());
    const double d = u - v + w;
    return (d > 0) ? ORIENTATION_CCW : ((d == 0) ? ORIENTATION_COLINEAR : ORIENTATION_CW);
}

// Return orientation of the polygon by checking the orientation of the left
// bottom corner, port of the exact core polygon-orientation helper.
inline bool is_ccw(const Polygon &poly) {
    assert(poly.points.size() >= 3);
    if (poly.points.size() < 3)
        return true;

    unsigned int imin = 0;
    for (unsigned int i = 1; i < poly.points.size(); ++i) {
        const Point &pmin = poly.points[imin];
        const Point &p = poly.points[i];
        if (p.x() < pmin.x() || (p.x() == pmin.x() && p.y() < pmin.y()))
            imin = i;
    }

    const size_t i_prev = ((imin == 0) ? poly.points.size() : imin) - 1;
    const size_t i_next = ((imin + 1 == poly.points.size()) ? 0 : imin + 1);
    const Orientation o = orient(poly.points[i_prev], poly.points[imin], poly.points[i_next]);
    return o == ORIENTATION_CCW;
}

// Assert that the polygon or every supplied path has a valid point count.
inline void assert_valid(const Polylines &polylines) {
    for (const Polyline &polyline : polylines)
        polyline.assert_valid();
}
inline void assert_valid(const Polygons &polygons) {
    for (const Polygon &polygon : polygons)
        polygon.assert_valid();
}


// Liang-Barsky line clipping, port of the Geometry.hpp template helpers for
// the double precision vectors and boxes used by the connect helpers.
inline bool liang_barsky_line_clipping_interval(const Vec2d &x0, const Vec2d &v, const BoundingBoxf &bbox,
                                                std::pair<double, double> &out_interval)
{
    double t0 = 0.0;
    double t1 = 1.0;
    const auto clip_side = [&t0, &t1](double p, double q) -> bool {
        if (p == 0) {
            return q >= 0;
        }
        const double r = q / p;
        if (p < 0) {
            if (r > t1)
                return false;
            if (r > t0)
                t0 = r;
        } else {
            if (r < t0)
                return false;
            if (r < t1)
                t1 = r;
        }
        return true;
    };

    if (clip_side(-v.x(), -bbox.min.x() + x0.x()) && clip_side(v.x(), bbox.max.x() - x0.x()) &&
        clip_side(-v.y(), -bbox.min.y() + x0.y()) && clip_side(v.y(), bbox.max.y() - x0.y())) {
        out_interval.first = t0;
        out_interval.second = t1;
        return true;
    }
    return false;
}

// Clip the segment to the box; return false on a miss, true with clipped endpoints otherwise.
inline bool liang_barsky_line_clipping(Vec2d &x0, Vec2d &x1, const BoundingBoxf &bbox) {
    const Vec2d v = x1 - x0;
    std::pair<double, double> interval;
    if (liang_barsky_line_clipping_interval(x0, v, bbox, interval)) {
        x1 = x0 + interval.second * v;
        x0 = x0 + interval.first * v;
        return true;
    }
    return false;
}

inline bool liang_barsky_line_clipping(const Vec2d &x0src, const Vec2d &x1src, const BoundingBoxf &bbox, Vec2d &x0clip,
                                       Vec2d &x1clip)
{
    x0clip = x0src;
    x1clip = x1src;
    return liang_barsky_line_clipping(x0clip, x1clip, bbox);
}

// Intersections of the line a*x + b*y + c = 0 with the circle x^2 + y^2 = r2,
// verbatim port of ray_circle_intersections_r2_lv2_c from Geometry/Circle.hpp.
// Returns 0, 1 (tangent) or 2 intersections.
template<typename T>
inline int ray_circle_intersections_r2_lv2_c(T r2, T a, T b, T lv2, T c, std::pair<Vec2d, Vec2d> &out)
{
    T x0 = -a * c;
    T y0 = -b * c;
    T d2 = r2 * lv2 - c * c;
    if (d2 < T(0))
        return 0;
    T d = std::sqrt(d2);
    out.first.vx = double((x0 + b * d) / lv2);
    out.first.vy = double((y0 - a * d) / lv2);
    out.second.vx = double((x0 - b * d) / lv2);
    out.second.vy = double((y0 + a * d) / lv2);
    return d == T(0) ? 1 : 2;
}

} // namespace polygon_work

}} // namespace slic3r_api::PluginCommon

#endif // plugins_common_polygon_work_buffers_hpp_
