///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_plugin_cpp_HoleGeometry_hpp_
#define slic3r_Api_plugin_cpp_HoleGeometry_hpp_

#include <cmath>
#include <cstddef>

#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"

/*
Hole geometry for size compensation
===================================

Small measurements used by the plugins that resize the holes of a slice:
- is_convex_hole(): only convex holes (round or polygonal bores) are resized;
  a hole with a concave corner is a cut-out, not a bore.
- hole_threshold_factor(): how much of the compensation a hole of a given area
  receives (hole_size_threshold).
- approximate_hole_diameter(): a rough diameter, used to read a compensation
  curve by diameter.

These are the rules of the legacy slicer (PrintObject::_shrink_contour_holes),
kept with the same tolerances so that compensated holes do not change.

Header-only and templated on the point container: it accepts a Polygon view,
a hole of an ExPolygon view, or a std::vector<c_point>. The container must
provide size() and operator[] returning c_point. Coordinates are scaled.

    if (is_convex_hole(hole))
        factor = hole_threshold_factor(std::abs(hole.area()), threshold_area);

Pass hole area and threshold in the same squared units. No function changes
the supplied points or retains a view after returning.
*/

namespace slic3r_api {

// True when every corner of the hole is convex, with a tolerance of 0.1 rad
// (about 5.7 degrees) so that a polygonal approximation of a circle with
// slightly irregular points still counts as convex. The hole is expected in
// the orientation of ExPolygon holes (clockwise). A hole with fewer than three
// points is not convex.
template<class Points>
bool is_convex_hole(const Points &hole)
{
    const size_t count = size_t(hole.size());
    if (count < 3)
        return false;
    // Angle at point `at`, between the directions to its previous and next points.
    const auto convex_at = [&hole](size_t previous, size_t at, size_t next) {
        const c_point p = hole[uint32_t(at)];
        const c_point a = hole[uint32_t(previous)];
        const c_point b = hole[uint32_t(next)];
        return abs_angle(angle_ccw(c_point{ a.x - p.x, a.y - p.y }, c_point{ b.x - p.x, b.y - p.y })) <= PI + 0.1;
    };
    for (size_t idx = 0; idx < count; ++idx)
        if (!convex_at((idx + count - 1) % count, idx, (idx + 1) % count))
            return false;
    return true;
}

// Fraction of the hole compensation applied to a hole of area `hole_area`,
// both areas in scaled units squared (see scale_d). With a threshold T:
// - T <= 0: every hole receives the full compensation (factor 1);
// - area <= T: factor 1;
// - area >= 4T: factor 0, a large hole is not changed;
// - in between, the factor decreases linearly, so that the walls of a sloped
//   hole (whose area changes from layer to layer) show no step.
inline double hole_threshold_factor(double hole_area, double threshold_area)
{
    if (threshold_area <= 0. || hole_area <= threshold_area)
        return 1.;
    if (hole_area >= 4. * threshold_area)
        return 0.;
    return (4. * threshold_area - hole_area) / (3. * threshold_area);
}

// Rough diameter of a hole, in scaled units: twice the mean distance of its
// points to its centroid. Exact for a regular polygon inscribed in a circle,
// only indicative for other shapes. Zero for fewer than three points.
template<class Points>
double approximate_hole_diameter(const Points &hole)
{
    const size_t count = size_t(hole.size());
    if (count < 3)
        return 0.;
    // Area centroid, as Polygon::centroid().
    double area2 = 0., cx = 0., cy = 0.;
    for (size_t idx = 0; idx < count; ++idx) {
        const c_point a = hole[uint32_t(idx)];
        const c_point b = hole[uint32_t((idx + 1) % count)];
        const double cross = double(a.x) * double(b.y) - double(b.x) * double(a.y);
        area2 += cross;
        cx += (double(a.x) + double(b.x)) * cross;
        cy += (double(a.y) + double(b.y)) * cross;
    }
    if (area2 == 0.)
        return 0.;
    cx /= 3. * area2;
    cy /= 3. * area2;
    double distance = 0.;
    for (size_t idx = 0; idx < count; ++idx) {
        const c_point p = hole[uint32_t(idx)];
        distance += std::hypot(double(p.x) - cx, double(p.y) - cy);
    }
    return 2. * distance / double(count);
}

} // namespace slic3r_api

#endif // slic3r_Api_plugin_cpp_HoleGeometry_hpp_
