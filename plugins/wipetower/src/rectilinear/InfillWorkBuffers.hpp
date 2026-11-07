///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

#ifndef nematx_wipetower_copy_work_buffers_hpp_
#define nematx_wipetower_copy_work_buffers_hpp_

// Private mutable paths retain core Point/MultiPoint rounding and simplification semantics outside the read-only API.
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <limits>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_def.h"
#include "libslic3r/Api/plugin/c/slic3r_geometry.h"

namespace slic3r_api { namespace NematXWipeTowerCopy { namespace work {
using std::size_t;

struct Vec2d
{
    double vx = 0.;
    double vy = 0.;
    Vec2d() = default;
    Vec2d(double x, double y) : vx(x), vy(y) {}
    double x() const { return vx; }
    double y() const { return vy; }
    double &operator()(size_t idx) { return idx == 0 ? vx : vy; }
    double operator()(size_t idx) const { return idx == 0 ? vx : vy; }
    double squaredNorm() const { return vx * vx + vy * vy; }
    double norm() const { return std::sqrt(this->squaredNorm()); }
    double dot(const Vec2d &other) const { return vx * other.vx + vy * other.vy; }
    Vec2d normalized() const {
        const double length = this->norm();
        return length == 0. ? Vec2d(0., 0.) : Vec2d(vx / length, vy / length);
    }
};

inline Vec2d operator-(const Vec2d &lhs, const Vec2d &rhs) { return Vec2d(lhs.vx - rhs.vx, lhs.vy - rhs.vy); }
inline Vec2d operator+(const Vec2d &lhs, const Vec2d &rhs) { return Vec2d(lhs.vx + rhs.vx, lhs.vy + rhs.vy); }
inline Vec2d operator*(double factor, const Vec2d &vec) { return Vec2d(factor * vec.vx, factor * vec.vy); }
inline Vec2d operator*(const Vec2d &vec, double factor) { return Vec2d(factor * vec.vx, factor * vec.vy); }
inline Vec2d operator/(const Vec2d &vec, double factor) { return Vec2d(vec.vx / factor, vec.vy / factor); }
inline bool operator==(const Vec2d &lhs, const Vec2d &rhs) { return lhs.vx == rhs.vx && lhs.vy == rhs.vy; }
inline bool operator!=(const Vec2d &lhs, const Vec2d &rhs) { return !(lhs == rhs); }
inline Vec2d perp(const Vec2d &v) { return Vec2d(-v.y(), v.x()); }

struct Point
{
    coord_t vx = 0;
    coord_t vy = 0;
    Point() = default;
    Point(coord_t x, coord_t y) : vx(x), vy(y) {}
    explicit Point(c_point point) : vx(point.x), vy(point.y) {}
    coord_t &x() { return vx; }
    coord_t x() const { return vx; }
    coord_t &y() { return vy; }
    coord_t y() const { return vy; }
    coord_t &operator()(size_t idx) { return idx == 0 ? vx : vy; }
    coord_t operator()(size_t idx) const { return idx == 0 ? vx : vy; }
    Point &operator+=(const Point &rhs) {
        vx += rhs.vx;
        vy += rhs.vy;
        return *this;
    }
    Point &operator-=(const Point &rhs) {
        vx -= rhs.vx;
        vy -= rhs.vy;
        return *this;
    }
    bool operator==(const Point &rhs) const { return vx == rhs.vx && vy == rhs.vy; }
    bool operator!=(const Point &rhs) const { return !(*this == rhs); }
    bool operator<(const Point &rhs) const { return vx < rhs.vx || (vx == rhs.vx && vy < rhs.vy); }
    double squaredNorm() const { return double(vx) * double(vx) + double(vy) * double(vy); }
    double norm() const { return std::sqrt(this->squaredNorm()); }
    double dot(const Point &other) const { return double(vx) * double(other.vx) + double(vy) * double(other.vy); }
    void rotate(double angle) { this->rotate(std::cos(angle), std::sin(angle)); }
    void rotate(double cos_a, double sin_a) {
        const double cur_x = double(vx);
        const double cur_y = double(vy);
        vx = coord_t(std::round(cos_a * cur_x - sin_a * cur_y));
        vy = coord_t(std::round(cos_a * cur_y + sin_a * cur_x));
    }
    void rotate(double angle, const Point &center) {
        const double cos_a = std::cos(angle);
        const double sin_a = std::sin(angle);
        const double cur_x = double(vx) - double(center.vx);
        const double cur_y = double(vy) - double(center.vy);
        vx = coord_t(std::round(double(center.vx) + cos_a * cur_x - sin_a * cur_y));
        vy = coord_t(std::round(double(center.vy) + cos_a * cur_y + sin_a * cur_x));
    }
    Point rotated(double angle) const {
        Point out(*this);
        out.rotate(angle);
        return out;
    }
    Point rotated(double cos_a, double sin_a) const {
        Point out(*this);
        out.rotate(cos_a, sin_a);
        return out;
    }
    Point rotated(double angle, const Point &center) const {
        Point out(*this);
        out.rotate(angle, center);
        return out;
    }
    double distance_to(const Point &point) const {
        const double dx = double(point.vx) - double(vx);
        const double dy = double(point.vy) - double(vy);
        return std::sqrt(dx * dx + dy * dy);
    }
    double distance_to_square(const Point &point) const {
        const double dx = double(point.vx) - double(vx);
        const double dy = double(point.vy) - double(vy);
        return dx * dx + dy * dy;
    }
    // Core interpolation truncates after double arithmetic, unlike rotations which round.
    Point interpolate(const double percent, const Point &point) const {
        return Point(coord_t(double(vx) * (1. - percent) + double(point.vx) * percent),
                     coord_t(double(vy) * (1. - percent) + double(point.vy) * percent));
    }
    bool coincides_with(const Point &point) const { return vx == point.vx && vy == point.vy; }
    bool coincides_with_epsilon(const Point &point) const {
        return std::abs(vx - point.vx) < SCALED_EPSILON / 2 && std::abs(vy - point.vy) < SCALED_EPSILON / 2;
    }
};

inline Point operator+(Point lhs, const Point &rhs) { lhs += rhs; return lhs; }
inline Point operator-(const Point &lhs, const Point &rhs) { return Point(lhs.vx - rhs.vx, lhs.vy - rhs.vy); }
inline Point operator/(const Point &lhs, coord_t divisor) { return Point(lhs.vx / divisor, lhs.vy / divisor); }
inline Point operator*(const Point &lhs, double factor) {
    const double min_value = double((std::numeric_limits<coord_t>::min)());
    const double max_value = double((std::numeric_limits<coord_t>::max)());
    return Point(coord_t(std::clamp(double(lhs.vx) * factor, min_value, max_value)),
                 coord_t(std::clamp(double(lhs.vy) * factor, min_value, max_value)));
}
inline Point operator*(double factor, const Point &rhs) { return rhs * factor; }
inline bool is_approx(const Point &p1, const Point &p2, coord_t epsilon = coord_t(SCALED_EPSILON)) {
    return std::abs(p1.vx - p2.vx) < epsilon && std::abs(p1.vy - p2.vy) < epsilon;
}
inline Vec2d to_vec2d(const Point &point) { return Vec2d(double(point.x()), double(point.y())); }
inline Vec2d to_vec2d(c_point point) { return Vec2d(double(point.x), double(point.y)); }
inline Point to_point(const Vec2d &vec) { return Point(coord_t(vec.x()), coord_t(vec.y())); }
inline Point to_point(c_point point) { return Point(point); }
inline Point lerp(const Point &a, const Point &b, double t) {
    assert((t >= -EPSILON) && (t <= 1. + EPSILON));
    return to_point((1. - t) * to_vec2d(a) + t * to_vec2d(b));
}
inline Vec2d lerp(const Vec2d &a, const Vec2d &b, double t) {
    assert((t >= -EPSILON) && (t <= 1. + EPSILON));
    return (1. - t) * a + t * b;
}

using Points = std::vector<Point>;

// Port of Slic3r::align_to_grid from PointUtils.hpp: the aligned value is
// never bigger than the original one, including for negative coordinates.
inline coord_t align_to_grid(coord_t coord, coord_t spacing) {
    const coord_t aligned = (coord < 0) ? ((coord - spacing + 1) / spacing) * spacing : (coord / spacing) * spacing;
    assert(aligned <= coord);
    return aligned;
}
inline Point align_to_grid(Point coord, Point spacing) {
    return Point(align_to_grid(coord.x(), spacing.x()), align_to_grid(coord.y(), spacing.y()));
}
inline coord_t align_to_grid(coord_t coord, coord_t spacing, coord_t base) {
    return base + align_to_grid(coord - base, spacing);
}
inline Point align_to_grid(Point coord, Point spacing, Point base) {
    return Point(align_to_grid(coord.x(), spacing.x(), base.x()), align_to_grid(coord.y(), spacing.y(), base.y()));
}

struct BoundingBox
{
    Point min;
    Point max;
    bool defined = false;
    BoundingBox() = default;
    BoundingBox(const Point &pmin, const Point &pmax) :
        min(pmin), max(pmax), defined(pmin.x() < pmax.x() && pmin.y() < pmax.y()) {}
    explicit BoundingBox(const Points &points) { this->merge(points); }
    bool empty() const { return min == max; }
    void merge(const Point &point) {
        if (!defined) {
            min = max = point;
            defined = true;
        } else {
            min.x() = std::min(min.x(), point.x());
            min.y() = std::min(min.y(), point.y());
            max.x() = std::max(max.x(), point.x());
            max.y() = std::max(max.y(), point.y());
        }
    }
    void merge(c_point point) { this->merge(Point(point)); }
    void merge(const Points &points) {
        for (const Point &point : points)
            this->merge(point);
    }
    void merge(const BoundingBox &other) {
        if (other.defined) {
            this->merge(other.min);
            this->merge(other.max);
        }
    }
    Point center() const { return Point((min.x() + max.x()) / 2, (min.y() + max.y()) / 2); }
    Point size() const { return max - min; }
    void offset(coordf_t delta) {
        // Core Point(delta, delta) truncates floating offsets.
        const coord_t dx = coord_t(delta);
        min.x() -= dx;
        min.y() -= dx;
        max.x() += dx;
        max.y() += dx;
    }
    BoundingBox inflated(coordf_t delta) const {
        BoundingBox out(*this);
        out.offset(delta);
        return out;
    }
    bool contains(const Point &point) const {
        return point.x() >= min.x() && point.x() <= max.x() && point.y() >= min.y() && point.y() <= max.y();
    }
};

struct MultiPoint
{
    Points points;
    MultiPoint() = default;
    MultiPoint(const Point &point1, const Point &point2) : points{ point1, point2 } {}
    MultiPoint(std::initializer_list<Point> init) : points(init) {}
    bool empty() const { return points.empty(); }
    size_t size() const { return points.size(); }
    Point &front() { return points.front(); }
    const Point &front() const { return points.front(); }
    Point &back() { return points.back(); }
    const Point &back() const { return points.back(); }
    Point &operator[](size_t idx) { return points[idx]; }
    const Point &operator[](size_t idx) const { return points[idx]; }
    Point &at(size_t idx) { return points.at(idx); }
    const Point &at(size_t idx) const { return points.at(idx); }
    void clear() { points.clear(); }
    void reserve(size_t count) { points.reserve(count); }
    void push_back(const Point &point) { points.push_back(point); }
    template<class... Args> void emplace_back(Args &&...args) { points.emplace_back(std::forward<Args>(args)...); }
    void erase(size_t idx) { points.erase(points.begin() + ptrdiff_t(idx)); }
    void erase(size_t begin, size_t end) { points.erase(points.begin() + ptrdiff_t(begin), points.begin() + ptrdiff_t(end)); }
    typename Points::iterator begin() { return points.begin(); }
    typename Points::iterator end() { return points.end(); }
    typename Points::const_iterator begin() const { return points.begin(); }
    typename Points::const_iterator end() const { return points.end(); }
    BoundingBox bounding_box() const { return BoundingBox(points); }
    bool is_valid() const { return points.size() > 1; }
    void assert_valid() const { assert(this->is_valid()); }
    void reverse() { std::reverse(points.begin(), points.end()); }
    void translate(const Point &offset) {
        for (Point &point : points)
            point += offset;
    }
    void translate(double x, double y) { this->translate(Point(coord_t(x), coord_t(y))); }
    void rotate(double angle) { this->rotate(std::cos(angle), std::sin(angle)); }
    void rotate(double cos_angle, double sin_angle) {
        for (Point &point : points) {
            const double cur_x = double(point.x());
            const double cur_y = double(point.y());
            point.x() = coord_t(std::round(cos_angle * cur_x - sin_angle * cur_y));
            point.y() = coord_t(std::round(cos_angle * cur_y + sin_angle * cur_x));
        }
    }
    void rotate(double angle, const Point &center) {
        for (Point &point : points)
            point.rotate(angle, center);
    }
    bool has_duplicate_points() const {
        for (size_t i = 1; i < points.size(); ++i)
            if (points[i - 1] == points[i])
                return true;
        return false;
    }
    bool remove_duplicate_points() {
        size_t j = 0;
        for (size_t i = 1; i < points.size(); ++i) {
            if (points[j] == points[i]) {
                // Retain the first point of an adjacent duplicate run.
            } else {
                ++j;
                if (j < i)
                    points[j] = points[i];
            }
        }
        if (++j < points.size()) {
            points.erase(points.begin() + ptrdiff_t(j), points.end());
            return true;
        }
        return false;
    }
    void douglas_peucker(coord_t tolerance = SCALED_EPSILON);
};

struct Polyline : MultiPoint
{
    using MultiPoint::MultiPoint;
    double length() const {
        double out = 0.;
        for (size_t i = 1; i < points.size(); ++i)
            out += points[i].distance_to(points[i - 1]);
        return out;
    }
};
using Polylines = std::vector<Polyline>;

// MultiPoint.hpp's anchor/floater walk preserves output order and its 100-unit tolerance floor.
inline Points douglas_peucker_points(const Points &src, coord_t tolerance)
{
    Points out;
    if (src.empty())
        return out;
    const double tolerance_d = double(tolerance);
    const double tolerance_sq_d = std::max(100. * 100., tolerance_d * tolerance_d);
    out.push_back(src.front());
    if (src.size() == 1)
        return out;
    if (src.size() == 2) {
        out.push_back(src.back());
        return out;
    }
    size_t anchor = 0;
    size_t floater = src.size() - 1;
    std::vector<size_t> dp_stack;
    dp_stack.reserve(src.size());
    dp_stack.push_back(floater);
    Point pt_start = src[anchor];
    Point pt_floater = src[floater];
    for (;;) {
        assert(anchor != floater);
        bool take_floater = false;
        size_t furthest = anchor;
        if (anchor + 1 == floater) {
            take_floater = true;
        } else {
            double max_dist_sq_d = 0.;
            const Vec2d vec_af_d = Vec2d(double(pt_floater.x() - pt_start.x()), double(pt_floater.y() - pt_start.y()));
            const double length_squared_d = vec_af_d.squaredNorm();
            if (length_squared_d == 0.) {
                for (size_t it = anchor + 1; it != floater; ++it) {
                    const Vec2d pttemp(double(src[it].x() - pt_start.x()), double(src[it].y() - pt_start.y()));
                    const double dist_sq_d = pttemp.squaredNorm();
                    if (dist_sq_d > max_dist_sq_d) {
                        max_dist_sq_d = dist_sq_d;
                        furthest = it;
                    }
                }
            } else {
                for (size_t it = anchor + 1; it != floater; ++it) {
                    const Vec2d vec_ap_d(double(src[it].x() - pt_start.x()), double(src[it].y() - pt_start.y()));
                    const double dot_d = vec_ap_d.dot(vec_af_d);
                    double dist_sq_d = 0.;
                    if (dot_d <= 0.) {
                        dist_sq_d = vec_ap_d.squaredNorm();
                    } else if (dot_d >= length_squared_d) {
                        const Vec2d vec_fp(double(src[it].x() - pt_floater.x()), double(src[it].y() - pt_floater.y()));
                        dist_sq_d = vec_fp.squaredNorm();
                    } else {
                        const Vec2d projected = (dot_d / length_squared_d) * vec_af_d;
                        dist_sq_d = (projected - vec_ap_d).squaredNorm();
                    }
                    if (dist_sq_d > max_dist_sq_d) {
                        max_dist_sq_d = dist_sq_d;
                        furthest = it;
                    }
                }
            }
            take_floater = max_dist_sq_d <= tolerance_sq_d;
        }
        if (take_floater) {
            pt_start = pt_floater;
            out.push_back(src[floater]);
            anchor = floater;
            assert(dp_stack.back() == floater);
            dp_stack.pop_back();
            if (dp_stack.empty())
                break;
            floater = dp_stack.back();
            pt_floater = src[floater];
        } else {
            floater = furthest;
            pt_floater = src[floater];
            dp_stack.push_back(floater);
        }
    }
    return out;
}

inline void MultiPoint::douglas_peucker(coord_t tolerance)
{
    points = douglas_peucker_points(points, tolerance);
}

// Port of the core ensure_valid(Polylines&, resolution): simplify each path and
// drop the ones collapsed to a single point.
inline void ensure_valid(Polylines &polylines, coord_t resolution) {
    for (size_t i = 0; i < polylines.size(); ++i) {
        assert(polylines[i].size() > 1);
        polylines[i].douglas_peucker(resolution);
        assert(polylines[i].size() > 1);
        if (polylines[i].size() == 2 && polylines[i].front().coincides_with_epsilon(polylines[i].back())) {
            polylines.erase(polylines.begin() + ptrdiff_t(i));
            --i;
        }
    }
}

}}} // namespace slic3r_api::NematXWipeTowerCopy::work
#endif
