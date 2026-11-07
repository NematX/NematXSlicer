///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/ Copyright (c) 2021 Ultimaker B.V.
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

// Keep sampling and erasure synchronized with Fill/Lightning/DistanceField.cpp; distance/hash bands impose strict ordering.
#include "LightningDistanceField.hpp"

#include <tuple>
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Plugins/Infill/Rectilinear/RectilinearFills.hpp"

namespace slic3r_api { namespace Infill { namespace Lightning {

DistanceField::DistanceField(coord_t radius, const work::BoundingBox &bounds, const work::Polygons &overhang,
                              storage_handle *scratch, const plugin_run_context *run_ctx)
    : m_radius(radius), m_spacing(radius / 6), m_bounds(bounds)
{
    namespace clipper = PluginCommon::clipper;
    const work::BoundingBox sampling_bounds = clipper::get_extents(overhang);
    const StoredPolygonCollection stored = clipper::to_stored_polygons(scratch, overhang);
    const StoredExPolygonCollection polygons = clipper_union(ClipperOperand(scratch, stored.readonly())).to_expolygon_collection();
    for (const ExPolygon polygon : polygons) {
        throw_if_cancelled(run_ctx);
        const work::Points points = RectilinearPatternsPlugin::fills::sample_grid_pattern(scratch, polygon, m_spacing, sampling_bounds);
        for (const work::Point &point : points) {
            double distance = std::numeric_limits<double>::max();
            const auto measure = [&](const Polygon &contour) {
                work::Point previous(contour.back());
                const multipoint_const_view vertices = contour.view();
                for (uint32_t index = 0; index < vertices.size; ++index) {
                    const work::Point current(vertices.array[index]);
                    distance = std::min(distance, distance_to_segment(previous, current, point));
                    previous = current;
                }
            };
            measure(polygon.contour());
            for (const Polygon hole : polygon.holes())
                measure(hole);
            m_cells.push_back({point, coord_t(std::sqrt(distance))});
        }
    }
    std::stable_sort(m_cells.begin(), m_cells.end(), [radius](const Cell &a, const Cell &b) {
        return std::make_tuple(a.distance / radius, PointHash{}(a.location) % 191) <
               std::make_tuple(b.distance / radius, PointHash{}(b.location) % 191);
    });
    if (m_cells.empty())
        return;
    for (const Cell &cell : m_cells)
        m_grid_range.merge((cell.location - m_bounds.min) / m_spacing);
    const work::Point dimensions = m_grid_range.max - m_grid_range.min + work::Point(1, 1);
    m_grid_width = dimensions.x();
    m_grid.assign(size_t(dimensions.x() * dimensions.y()), std::numeric_limits<size_t>::max());
    for (size_t index = 0; index < m_cells.size(); ++index) {
        const work::Point address = (m_cells[index].location - m_bounds.min) / m_spacing - m_grid_range.min;
        const size_t flat = size_t(address.y() * m_grid_width + address.x());
        assert(m_grid[flat] == std::numeric_limits<size_t>::max());
        m_grid[flat] = index;
    }
}

bool DistanceField::next(work::Point &point, size_t &index, size_t begin) const
{
    for (size_t candidate = begin; candidate < m_cells.size(); ++candidate)
        if (!m_cells[candidate].erased) {
            point = m_cells[candidate].location;
            index = candidate;
            return true;
        }
    return false;
}

void DistanceField::update(const work::Point &ground, const work::Point &leaf)
{
    const work::Vec2d vector = work::to_vec2d(leaf - ground);
    const double length_squared = vector.squaredNorm();
    const double length = std::sqrt(length_squared);
    const work::Vec2d extent(-vector.y() * m_radius / length, vector.x() * m_radius / length);
    const work::Point integer_extent = work::to_point(extent);
    const work::Point diagonal(m_radius, m_radius);
    work::BoundingBox range(leaf - diagonal, leaf + diagonal);
    range.merge(ground - integer_extent);
    range.merge(ground + integer_extent);
    range.merge(leaf - integer_extent);
    range.merge(leaf + integer_extent);
    range.min.x() = std::max(range.min.x(), m_bounds.min.x());
    range.min.y() = std::max(range.min.y(), m_bounds.min.y());
    range.max.x() = std::min(range.max.x(), m_bounds.max.x());
    range.max.y() = std::min(range.max.y(), m_bounds.max.y());
    range.min = (range.min - m_bounds.min) / m_spacing;
    range.max = (range.max - m_bounds.min) / m_spacing;
    const lengthsqr_t radius_squared = coord_int_sqr(m_radius);
    for (coord_t y = range.min.y(); y <= range.max.y(); ++y)
        for (coord_t x = range.min.x(); x <= range.max.x(); ++x) {
            const work::Point address(x, y);
            if (!m_grid_range.contains(address))
                continue;
            const work::Point location(x * m_spacing + m_bounds.min.x(), y * m_spacing + m_bounds.min.y());
            const work::Point difference = location - leaf;
            if (squared_int_norm(difference) > radius_squared) {
                const work::Vec2d offset = work::to_vec2d(location - ground);
                const double projection = vector.dot(offset);
                if (projection >= 0. && projection <= length_squared) {
                    const double side = extent.dot(offset);
                    if (side < -1. || side > 1.)
                        continue;
                }
            }
            const work::Point relative = address - m_grid_range.min;
            const size_t index = m_grid[size_t(relative.y() * m_grid_width + relative.x())];
            if (index == std::numeric_limits<size_t>::max() || m_cells[index].erased)
                continue;
            const work::Point delta = m_cells[index].location - leaf;
            if (squared_int_norm(delta) <= radius_squared)
                m_cells[index].erased = true;
        }
}
}}}
