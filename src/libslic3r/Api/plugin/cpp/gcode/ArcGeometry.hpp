///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_plugin_cpp_gcode_ArcGeometry_hpp_
#define slic3r_Api_plugin_cpp_gcode_ArcGeometry_hpp_

#include <cmath>
#include <stdexcept>

#include "libslic3r/Api/plugin/c/slic3r_geometry_types.h"

/*
Arc geometry for firmware encoders
==================================

The firmware session describes an arc by its start, its end, a signed radius
and a direction, which is enough for the radius form "G2/G3 ... R".
Some firmwares, such as Klipper, only accept the center form "G2/G3 ... I J",
where I and J are the offsets from the start point to the arc center.
arc_center() converts the first description into the center.

Signed radius convention (the same as PreparedMove::radius_mm):
- positive radius: the shorter of the two possible arcs;
- negative radius: the longer one.

Header-only, without Eigen or host headers, so built-in and external firmware
plugins share one implementation.

    const c_vec2d center = arc_center(start, end, move.radius_mm, is_ccw);
    // Emit I = center.x - start.x and J = center.y - start.y.
*/

namespace slic3r_api { namespace GCodeGeneration {

// Return the center of the arc from start to end with the given signed radius,
// turning counter-clockwise when is_ccw is true. Coordinates are in any one
// unit (usually mm); the result uses the same unit.
//
// Throws std::invalid_argument for a zero or non-finite radius, non-finite
// points, or identical start and end points: no single center exists then.
//
// When the radius is slightly smaller than half the chord (rounding on a half
// circle), the center is the chord's midpoint instead of failing.
inline c_vec2d arc_center(c_vec2d start, c_vec2d end, double radius, bool is_ccw)
{
    if (!std::isfinite(radius) || radius == 0.0)
        throw std::invalid_argument("An arc center needs a finite, non-zero radius.");
    if (!std::isfinite(start.x) || !std::isfinite(start.y) ||
        !std::isfinite(end.x) || !std::isfinite(end.y))
        throw std::invalid_argument("An arc center needs finite start and end points.");

    // Chord from start to end.
    const double vx = end.x - start.x;
    const double vy = end.y - start.y;
    const double chord_sqr = vx * vx + vy * vy;
    if (chord_sqr == 0.0)
        throw std::invalid_argument("An arc center needs distinct start and end points.");

    // The center lies on the perpendicular bisector of the chord, at distance
    // t * |chord| from its midpoint, where t = sqrt(r^2 / |chord|^2 - 1/4).
    const double t_sqr = radius * radius / chord_sqr - 0.25;
    const double t = t_sqr > 0.0 ? std::sqrt(t_sqr) : 0.0;
    const double mid_x = (start.x + end.x) * 0.5;
    const double mid_y = (start.y + end.y) * 0.5;
    // Chord rotated by +90 degrees and scaled by t.
    const double px = -vy * t;
    const double py = vx * t;

    // A short counter-clockwise arc (or a long clockwise one) turns around a
    // center on the left of the chord; the two other cases use the right side.
    if ((radius > 0.0) == is_ccw)
        return c_vec2d{ mid_x + px, mid_y + py };
    return c_vec2d{ mid_x - px, mid_y - py };
}

}} // namespace slic3r_api::GCodeGeneration

#endif // slic3r_Api_plugin_cpp_gcode_ArcGeometry_hpp_
