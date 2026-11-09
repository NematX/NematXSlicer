///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

#ifndef plugins_lightning_geometry_hpp_
#define plugins_lightning_geometry_hpp_

// Scaled-coordinate predicates and borrowed grid queries retain native Lightning boundary and rounding contracts.
#include "LightningPreparation.hpp"
#include "libslic3r/Plugins/Common/PluginEdgeGrid.hpp"
#include <boost/multiprecision/cpp_int.hpp>

namespace slic3r_api { namespace Infill { namespace Lightning {
namespace grid = PluginCommon::EdgeGrid;
constexpr coord_t locator_cell_size = scale_i(4.);

// Keep the native PointHash bit shifts; they determine grid lookup and dispersed cell priorities.
struct PointHash
{
    size_t operator()(const work::Point &point) const noexcept
    {
        return size_t(coord_t((89 * 31 + (int64_t(point.x()) >> 6)) * 31 + (point.y() >> 6)));
    }
};

// Native squared integer distances discard low coordinate bits to avoid overflow.
inline lengthsqr_t squared_int_norm(const work::Point &point)
{
    const lengthsqr_t x = std::abs(point.x()) >> SLIC3R_SQUARE_BIT_REDUCTION;
    const lengthsqr_t y = std::abs(point.y()) >> SLIC3R_SQUARE_BIT_REDUCTION;
    return x * x + y * y;
}

// Native Line projection rounds the nearest point but measures distance before that rounding.
inline double distance_to_segment(const work::Point &a, const work::Point &b,
                                   const work::Point &point, work::Point *nearest = nullptr)
{
    const work::Vec2d vector = work::to_vec2d(b - a);
    const work::Vec2d offset = work::to_vec2d(point - a);
    const double length_squared = vector.squaredNorm();
    const double projection = offset.dot(vector);
    if (length_squared == 0. || projection <= 0.) {
        if (nearest)
            *nearest = a;
        return offset.squaredNorm();
    }
    if (projection >= length_squared) {
        if (nearest)
            *nearest = b;
        return work::to_vec2d(point - b).squaredNorm();
    }
    const work::Vec2d scaled = (projection / length_squared) * vector;
    if (nearest) {
        const work::Vec2d result = work::to_vec2d(a) + scaled;
        *nearest = work::Point(coord_t(std::round(result.x())), coord_t(std::round(result.y())));
    }
    return (scaled - offset).squaredNorm();
}

// Exact orientations keep border points inside and avoid overflowing scaled-coordinate products.
inline int orientation(const work::Point &a, const work::Point &b, const work::Point &p)
{
    using boost::multiprecision::int128_t;
    const int128_t determinant = int128_t(b.x() - a.x()) * (p.y() - a.y()) -
                                 int128_t(b.y() - a.y()) * (p.x() - a.x());
    return (determinant > 0) - (determinant < 0);
}

// Even-odd containment includes boundaries independently of contour winding.
inline bool inside(const work::Polygons &polygons, const work::Point &point)
{
    bool interior = false;
    for (const work::Polygon &polygon : polygons) {
        if (polygon.empty())
            continue;
        work::Point previous = polygon.points.back();
        for (const work::Point &current : polygon.points) {
            const int side = orientation(previous, current, point);
            if (side == 0 && point.x() >= std::min(previous.x(), current.x()) && point.x() <= std::max(previous.x(), current.x()) &&
                point.y() >= std::min(previous.y(), current.y()) && point.y() <= std::max(previous.y(), current.y()))
                return true;
            if ((previous.y() > point.y()) != (current.y() > point.y()) &&
                (current.y() > previous.y() ? side > 0 : side < 0))
                interior = !interior;
            previous = current;
        }
    }
    return interior;
}

// Use the same finite-segment parameters as native line_alg::intersection.
inline bool intersection(const work::Point &a, const work::Point &b,
                          const work::Point &c, const work::Point &d, work::Vec2d &point)
{
    const work::Vec2d v1 = work::to_vec2d(b - a), v2 = work::to_vec2d(d - c);
    const double denominator = v1.x() * v2.y() - v1.y() * v2.x();
    if (std::abs(denominator) < EPSILON)
        return false;
    const work::Vec2d delta = work::to_vec2d(a - c);
    const double t = (v2.x() * delta.y() - v2.y() * delta.x()) / denominator;
    const double u = (v1.x() * delta.y() - v1.y() * delta.x()) / denominator;
    if (t < 0. || t > 1. || u < 0. || u > 1.)
        return false;
    point = work::to_vec2d(a) + t * v1;
    return true;
}

// Return the closest contour hit to b, not the first grid cell visited.
inline bool boundary_intersection(const work::Point &a, const work::Point &b, const grid::Grid &locator,
                                   work::Point &point, coord_t maximum_distance)
{
    struct Visitor
    {
        const grid::Grid &locator;
        const work::Point &a;
        const work::Point &b;
        double distance = std::numeric_limits<double>::max();
        work::Vec2d hit;
        bool operator()(coord_t y, coord_t x)
        {
            const auto range = locator.cell_data_range(y, x);
            for (std::vector<std::pair<size_t, size_t>>::const_iterator it = range.first; it != range.second; ++it) {
                const std::pair<c_point, c_point> segment = locator.segment(*it);
                work::Vec2d candidate;
                if (intersection(work::Point(segment.first), work::Point(segment.second), a, b, candidate)) {
                    const double squared = (candidate - work::to_vec2d(b)).squaredNorm();
                    if (squared < distance) {
                        distance = squared;
                        hit = candidate;
                    }
                }
            }
            return true;
        }
    } visitor{locator, a, b};
    locator.visit_cells_intersecting_line(a, b, visitor);
    if (visitor.distance >= double(maximum_distance) * maximum_distance)
        return false;
    point = work::Point(coord_t(std::round(visitor.hit.x())), coord_t(std::round(visitor.hit.y())));
    return true;
}

// Collision differs from intersection: non-collinear shared endpoints remain valid branch attachments.
inline bool boundary_collision(const work::Point &a, const work::Point &b, const grid::Grid &locator)
{
    struct Visitor
    {
        const grid::Grid &locator;
        const work::Point &a;
        const work::Point &b;
        bool collision = false;
        bool operator()(coord_t y, coord_t x)
        {
            const auto range = locator.cell_data_range(y, x);
            for (std::vector<std::pair<size_t, size_t>>::const_iterator it = range.first; it != range.second; ++it) {
                const std::pair<c_point, c_point> segment = locator.segment(*it);
                const work::Point c(segment.first), d(segment.second);
                const int first = orientation(a, b, c), second = orientation(a, b, d);
                const int other_first = orientation(c, d, a), other_second = orientation(c, d, b);
                if (first * second <= 0 && other_first * other_second <= 0) {
                    // Native collision testing allows a non-collinear shared endpoint.
                    bool intersects = first * second != 0 || other_first * other_second != 0;
                    if (first == 0 && second == 0) {
                        const bool x_axis = std::abs(b.x() - a.x()) > std::abs(b.y() - a.y());
                        const coord_t start = x_axis ? a.x() : a.y(), end = x_axis ? b.x() : b.y();
                        const coord_t low = x_axis ? c.x() : c.y(), high = x_axis ? d.x() : d.y();
                        intersects = std::max(std::min(start, end), std::min(low, high)) <=
                                     std::min(std::max(start, end), std::max(low, high));
                    }
                    if (intersects) {
                        collision = true;
                        return false;
                    }
                }
            }
            return true;
        }
    } visitor{locator, a, b};
    locator.visit_cells_intersecting_line(a, b, visitor);
    return visitor.collision;
}
}}}
#endif
