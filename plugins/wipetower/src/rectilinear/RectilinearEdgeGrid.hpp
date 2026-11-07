///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ Copyright (c) Prusa Research 2016 - 2023 Vojtech Bubnik @bubnikv, Lukas Hejl @hejllukas, Lukas Matena @lukasmatena
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef nematx_wipetower_copy_rectilinear_edge_grid_hpp_
#define nematx_wipetower_copy_rectilinear_edge_grid_hpp_

/*
Edge grid for the rectilinear pattern port
==========================================

Port of the EdgeGrid::Grid subset used by the infill connect helpers
(src/libslic3r/Fill/FillBase.cpp): contour storage, rasterization of the
contours into a uniform grid of cells, line traversal and the closest point
query on the contours.

The implementation is a verbatim port of src/libslic3r/EdgeGrid.hpp and
EdgeGrid.cpp, adapted to the private work buffers used by the ported
algorithms: Point accessors replace the Eigen operator(), the signed distance
field and the debugging or unused entry points are dropped.

Like the core EdgeGrid, a Grid references the point arrays of the contours it
was built from: the caller must keep them alive while the grid is used. API
contours are borrowed through multipoint_const_view and are not materialized.
*/

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"

#include "RectilinearWorkBuffers.hpp"

namespace slic3r_api { namespace NematXWipeTowerCopy { namespace RectilinearPatternsPlugin {
namespace EdgeGrid {

using namespace work;

class Contour
{
public:
    Contour() = default;
    Contour(const ContourView &view, bool open) : m_view(view), m_open(open) {}
    Contour(const Points &pts, bool open) : Contour(ContourView(pts), open) {}

    bool open() const { return m_open; }
    bool closed() const { return !m_open; }
    size_t size() const { return m_view.size(); }
    c_point operator[](size_t idx) const { return m_view[idx]; }
    c_point front() const { return m_view.front(); }
    c_point back() const { return m_view.back(); }

    // Start point of a segment idx.
    c_point segment_start(size_t idx) const {
        assert(idx < this->num_segments());
        return m_view[idx];
    }

    // End point of a segment idx.
    c_point segment_end(size_t idx) const {
        assert(idx < this->num_segments());
        return idx + 1 == m_view.size() ? m_view.front() : m_view[idx + 1];
    }

    // Start point of a segment preceding idx.
    c_point segment_prev(size_t idx) const {
        assert(idx < this->num_segments());
        assert(idx > 0 || !m_open);
        return idx == 0 ? m_view.back() : m_view[idx - 1];
    }

    // Index of a segment preceding idx.
    const size_t segment_idx_prev(size_t idx) const {
        assert(idx < this->num_segments());
        assert(idx > 0 || !m_open);
        return (idx == 0 ? this->size() : idx) - 1;
    }

    // Index of a segment following idx.
    const size_t segment_idx_next(size_t idx) const {
        assert(idx < this->num_segments());
        ++idx;
        return idx == m_view.size() ? 0 : idx;
    }

    size_t num_segments() const { return this->size() - (m_open ? 1 : 0); }

private:
    ContourView m_view;
    bool m_open { false };
};

class Grid
{
public:
    Grid() = default;
    Grid(const BoundingBox &bbox) : m_bbox(bbox) {}

    void set_bbox(const BoundingBox &bbox) { m_bbox = bbox; }

    // Fill in the grid with closed contours.
    void create(const std::vector<Points> &polygons, coord_t resolution) { this->create(polygons, resolution, false); }
    // Fill in the grid with open polylines or closed contours. A polyline is
    // considered closed when its first point equals its last point; the last
    // repeated point is removed when inserted into the grid.
    void create(const std::vector<Points> &polylines_or_polygons, coord_t resolution, bool open);
    void create(const ContourViews &contours, coord_t resolution, bool open = false);
    void create(const Polygon &polygon, coord_t resolution);
    void create(const Polygons &polygons, coord_t resolution);
    void create(const std::vector<const Polygon*> &polygons, coord_t resolution);
    void create(const slic3r_api::ExPolygon &expoly, coord_t resolution);

    const std::vector<Contour> &contours() const { return m_contours; }

    const BoundingBox &bbox() const { return m_bbox; }
    const coord_t resolution() const { return m_resolution; }
    const size_t rows() const { return m_rows; }
    const size_t cols() const { return m_cols; }

    // Closest point on the contours to pt, with the sign of the distance.
    struct ClosestPointResult
    {
        size_t contour_idx = size_t(-1);
        size_t start_point_idx = size_t(-1);
        // Signed distance to the closest point.
        double distance = std::numeric_limits<double>::max();
        // Parameter of the closest point on the edge starting at start_point_idx, <0, 1)
        double t = 0.;

        bool valid() const { return contour_idx != size_t(-1); }
    };

    ClosestPointResult closest_point_signed_distance(const Point &pt, coord_t search_radius,
                                                     size_t only_this_contour = size_t(-1)) const;

    std::pair<std::vector<std::pair<size_t, size_t>>::const_iterator,
              std::vector<std::pair<size_t, size_t>>::const_iterator>
    cell_data_range(int64_t row, int64_t col) const {
        assert(row >= 0 && size_t(row) < m_rows);
        assert(col >= 0 && size_t(col) < m_cols);
        const Cell &cell = m_cells[row * m_cols + col];
        return std::make_pair(m_cell_data.begin() + cell.begin, m_cell_data.begin() + cell.end);
    }

    std::pair<c_point, c_point> segment(const std::pair<size_t, size_t> &contour_and_segment_idx) const {
        const Contour &contour = m_contours[contour_and_segment_idx.first];
        const size_t iseg = contour_and_segment_idx.second;
        return std::make_pair(contour.segment_start(iseg), contour.segment_end(iseg));
    }

    template<typename VISITOR> void visit_cells_intersecting_line(Point p1, Point p2, VISITOR &visitor) const
    {
        // End points of the line segment.
        assert(m_bbox.contains(p1));
        assert(m_bbox.contains(p2));
        p1 -= m_bbox.min;
        p2 -= m_bbox.min;
        assert(std::abs(p1.x()) < coord_t(int32_t(0x7FFFFFFF)));
        assert(std::abs(p1.y()) < coord_t(int32_t(0x7FFFFFFF)));
        assert(std::abs(p2.x()) < coord_t(int32_t(0x7FFFFFFF)));
        assert(std::abs(p2.y()) < coord_t(int32_t(0x7FFFFFFF)));
        assert(p1.x() >= 0 && p1.x() < coord_t(m_cols) * m_resolution);
        assert(p1.y() >= 0 && p1.y() < coord_t(m_rows) * m_resolution);
        assert(p2.x() >= 0 && p2.x() < coord_t(m_cols) * m_resolution);
        assert(p2.y() >= 0 && p2.y() < coord_t(m_rows) * m_resolution);
        // Get the cells of the end points.
        int64_t ix = p1.x() / m_resolution;
        int64_t iy = p1.y() / m_resolution;
        int64_t ixb = p2.x() / m_resolution;
        int64_t iyb = p2.y() / m_resolution;
        assert(ix >= 0 && size_t(ix) < m_cols);
        assert(iy >= 0 && size_t(iy) < m_rows);
        assert(ixb >= 0 && size_t(ixb) < m_cols);
        assert(iyb >= 0 && size_t(iyb) < m_rows);
        // Account for the end points.
        if (!visitor(iy, ix) || (ix == ixb && iy == iyb))
            // Both ends fall into the same cell.
            return;
        // Raster the central part of the line.
        coord_t dx = std::abs(p2.x() - p1.x());
        coord_t dy = std::abs(p2.y() - p1.y());
        if (p1.x() < p2.x()) {
            int64_t ex = int64_t((ix + 1) * m_resolution - p1.x()) * int64_t(dy);
            if (p1.y() < p2.y()) {
                // x positive, y positive
                int64_t ey = int64_t((iy + 1) * m_resolution - p1.y()) * int64_t(dx);
                do {
                    assert(ix <= ixb && iy <= iyb);
                    if (ex < ey) {
                        ey -= ex;
                        ex = int64_t(dy) * m_resolution;
                        ix += 1;
                        assert(ix <= ixb);
                    } else if (ex == ey) {
                        ex = int64_t(dy) * m_resolution;
                        ey = int64_t(dx) * m_resolution;
                        ix += 1;
                        iy += 1;
                        assert(ix <= ixb);
                        assert(iy <= iyb);
                    } else {
                        assert(ex > ey);
                        ex -= ey;
                        ey = int64_t(dx) * m_resolution;
                        iy += 1;
                        assert(iy <= iyb);
                    }
                    if (!visitor(iy, ix))
                        return;
                } while (ix != ixb || iy != iyb);
            } else {
                // x positive, y non positive
                int64_t ey = int64_t(p1.y() - iy * m_resolution) * int64_t(dx);
                do {
                    assert(ix <= ixb && iy >= iyb);
                    if (ex <= ey) {
                        ey -= ex;
                        ex = int64_t(dy) * m_resolution;
                        ix += 1;
                        assert(ix <= ixb);
                    } else {
                        ex -= ey;
                        ey = int64_t(dx) * m_resolution;
                        iy -= 1;
                        assert(iy >= iyb);
                    }
                    if (!visitor(iy, ix))
                        return;
                } while (ix != ixb || iy != iyb);
            }
        } else {
            int64_t ex = int64_t(p1.x() - ix * m_resolution) * int64_t(dy);
            if (p1.y() < p2.y()) {
                // x non positive, y positive
                int64_t ey = int64_t((iy + 1) * m_resolution - p1.y()) * int64_t(dx);
                do {
                    assert(ix >= ixb && iy <= iyb);
                    if (ex < ey) {
                        ey -= ex;
                        ex = int64_t(dy) * m_resolution;
                        ix -= 1;
                        assert(ix >= ixb);
                    } else {
                        assert(ex >= ey);
                        ex -= ey;
                        ey = int64_t(dx) * m_resolution;
                        iy += 1;
                        assert(iy <= iyb);
                    }
                    if (!visitor(iy, ix))
                        return;
                } while (ix != ixb || iy != iyb);
            } else {
                // x non positive, y non positive
                int64_t ey = int64_t(p1.y() - iy * m_resolution) * int64_t(dx);
                do {
                    assert(ix >= ixb && iy >= iyb);
                    if (ex < ey) {
                        ey -= ex;
                        ex = int64_t(dy) * m_resolution;
                        ix -= 1;
                        assert(ix >= ixb);
                    } else if (ex == ey) {
                        // The lower edge of a grid cell belongs to the cell.
                        // Handle the case where the ray may cross the lower left corner of a cell in a general case,
                        // or a left or lower edge in a degenerate case (horizontal or vertical line).
                        if (dx > 0) {
                            ex = int64_t(dy) * m_resolution;
                            ix -= 1;
                            assert(ix >= ixb);
                        }
                        if (dy > 0) {
                            ey = int64_t(dx) * m_resolution;
                            iy -= 1;
                            assert(iy >= iyb);
                        }
                    } else {
                        assert(ex > ey);
                        ex -= ey;
                        ey = int64_t(dx) * m_resolution;
                        iy -= 1;
                        assert(iy >= iyb);
                    }
                    if (!visitor(iy, ix))
                        return;
                } while (ix != ixb || iy != iyb);
            }
        }
    }

private:
    struct Cell
    {
        Cell() : begin(0), end(0) {}
        size_t begin;
        size_t end;
    };

    void create_from_m_contours(coord_t resolution);

    // Bounding box around the contours.
    BoundingBox m_bbox;
    // Grid dimensions.
    coord_t m_resolution = 0;
    size_t m_rows = 0;
    size_t m_cols = 0;

    // Referencing the source contours. This format allows one to work with any
    // fixed point contour format (Polygon, ExPolygon, ...).
    std::vector<Contour> m_contours;

    // Referencing a contour and a line segment of m_contours.
    std::vector<std::pair<size_t, size_t>> m_cell_data;

    // Full grid of cells.
    std::vector<Cell> m_cells;
};

} // namespace EdgeGrid
}}} // namespace slic3r_api::NematXWipeTowerCopy::RectilinearPatternsPlugin

#endif // nematx_wipetower_copy_rectilinear_edge_grid_hpp_
