/* Shared polygon metrics and generators of the geometry tests of the support family (polygon simplify, tree geometry, tree volumes):
   areas in mm2, the area of the symmetric difference of two ring sets, and random star-shaped rings. */
#ifndef polygon_metrics_test_helpers_hpp_
#define polygon_metrics_test_helpers_hpp_

#include <cmath>
#include <random>

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"

namespace Slic3r::Test::PolygonMetrics {

// Area in mm2 from an area in scaled units squared.
inline double mm2(double scaled_area)
{
    return scaled_area * SCALING_FACTOR * SCALING_FACTOR;
}

// Area in mm2 of the symmetric difference of two ring sets (NonZero fill: counter-clockwise contours, clockwise holes). Polygon
// differences, without any filter: diff_ex() would drop the small areas (remove_small_areas) and could report 0 for a real difference.
inline double sym_diff_mm2(const Polygons &a, const Polygons &b)
{
    return mm2(area(diff(a, b)) + area(diff(b, a)));
}

// Star-shaped counter-clockwise ring around a center, with a random radius per vertex: radius_mm times a factor in [min_fraction, 1].
// The generator is consumed once per vertex.
inline Polygon random_star(std::mt19937 &rng, const Point &center, double radius_mm, size_t points, double min_fraction)
{
    std::uniform_real_distribution<double> jitter(min_fraction, 1.0);
    Polygon polygon;
    for (size_t i = 0; i < points; ++i) {
        const double angle = 2. * PI * double(i) / double(points);
        const double radius = radius_mm * jitter(rng);
        polygon.points.emplace_back(center.x() + scale_i(radius * std::cos(angle)), center.y() + scale_i(radius * std::sin(angle)));
    }
    return polygon;
}

} // namespace Slic3r::Test::PolygonMetrics

#endif // polygon_metrics_test_helpers_hpp_
