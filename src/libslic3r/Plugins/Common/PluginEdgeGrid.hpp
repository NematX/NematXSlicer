///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ Copyright (c) Prusa Research 2016 - 2023 Vojtech Bubnik @bubnikv, Lukas Hejl @hejllukas, Lukas Matena @lukasmatena
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef plugins_common_edge_grid_hpp_
#define plugins_common_edge_grid_hpp_

/*
Contour segment grid for plugin algorithms
==========================================

Infill connection helpers use Grid to find nearby contour segments without
scanning every edge. Create it from work rings or API geometry with a positive
cell resolution in scaled units, then query its cells or signed closest point.
Each cell stores contour and segment indices: these are candidates for a
geometric test, not proof that a query intersects a segment.

Grid borrows the source point arrays and indexes their coordinates. Keep them
alive and unchanged while querying: reallocation invalidates the borrowed
addresses, and coordinate edits make the cell index stale. Rebuild after edits.
Line traversal requires both endpoints inside bbox(). Signed closest-point
queries require closed contours; an unsuccessful search returns an invalid result.
Grid ports the subset of the existing Slic3r implementation (the core) in
src/libslic3r/EdgeGrid.hpp and EdgeGrid.cpp used by these algorithms, reproducing
its rasterization and closest-point results. It omits the core's optional
signed-distance-field grid and debug entry points; the signed closest-point
query remains available.

    polygon_work::Polygons rings;
    rings.emplace_back();
    rings.back().points = {{0, 0}, {100, 0}, {100, 100}, {0, 100}};
    EdgeGrid::Grid grid;
    grid.create(rings, 25);
    auto nearest = grid.closest_point_signed_distance({50, 50}, 100);
    // Keep rings alive and unchanged until queries finish.

Box queries subtract the grid origin and one unit from the upper coordinate
bound, then divide toward zero and clip to inclusive cell limits. Thus a box
just below the origin may include the first cell; this is not an exact geometric
filter. The returned box keeps its input defined flag. Visitors receive cells
in row order, visit none for inverted limits, and stop on false.
*/

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"

#include "PolygonWorkBuffers.hpp"

namespace slic3r_api { namespace PluginCommon {
namespace EdgeGrid {

using namespace polygon_work;

class Contour
{
public:
    // Borrow a contour view or point array with its closure flag, or initialize an empty closed view.
    Contour() = default;
    Contour(const ContourView &view, bool open) : m_view(view), m_open(open) {}
    Contour(const Points &pts, bool open) : Contour(ContourView(pts), open) {}

    // Report whether the contour omits a closing edge.
    bool open() const { return m_open; }
    // Report whether the contour includes a closing edge.
    bool closed() const { return !m_open; }
    // Return the number of borrowed points.
    size_t size() const { return m_view.size(); }
    // Return the point at an in-range index by value.
    c_point operator[](size_t idx) const { return m_view[idx]; }
    // Return the first point by value; the sequence must be nonempty.
    c_point front() const { return m_view.front(); }
    // Return the last point by value; the sequence must be nonempty.
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

    // Return the vertex count for a ring, or one less for an open contour.
    size_t num_segments() const { return this->size() - (m_open ? 1 : 0); }

private:
    ContourView m_view;
    bool m_open { false };
};

class Grid
{
public:
    // Initialize an empty grid, optionally with caller-supplied bounds.
    Grid() = default;
    Grid(const BoundingBox &bbox) : m_bbox(bbox) {}

    // Set the initial bounds to be extended during creation.
    void set_bbox(const BoundingBox &bbox) { m_bbox = bbox; }

    // Fill in the grid with closed contours.
    void create(const std::vector<Points> &polygons, coord_t resolution) { this->create(polygons, resolution, false); }
    // Fill in the grid with open polylines or closed contours. A polyline is
    // considered closed when its first point equals its last point; the last
    // repeated point is removed when inserted into the grid.
    void create(const std::vector<Points> &polylines_or_polygons, coord_t resolution, bool open);
    // Build the cell index from borrowed contours with positive resolution; keep source arrays stable and unchanged.
    void create(const ContourViews &contours, coord_t resolution, bool open = false);
    void create(const Polygon &polygon, coord_t resolution);
    void create(const Polygons &polygons, coord_t resolution);
    void create(const std::vector<const Polygon*> &polygons, coord_t resolution);
    void create(const slic3r_api::ExPolygon &expoly, coord_t resolution);

    // Rasterize the contour and holes of a work ExPolygon as closed rings; the grid borrows their point arrays.
    void create(const polygon_work::ExPolygon &expoly, coord_t resolution) {
        ContourViews contours;
        contours.reserve(1 + expoly.holes.size());
        contours.emplace_back(expoly.contour.points);
        for (const Polygon &hole : expoly.holes) contours.emplace_back(hole.points);
        this->create(contours, resolution, false);
    }

    // Convert coordinate bounds to clipped inclusive cell limits by division toward zero.
    // Near-origin negative bounds may include cell zero; retain the input defined flag.
    BoundingBox get_cells_intersecting_box(BoundingBox box) const {
        box.min -= m_bbox.min;
        box.max -= m_bbox.min + Point(1, 1);
        box.min = box.min / m_resolution;
        box.max = box.max / m_resolution;
        box.min.x() = std::max<coord_t>(box.min.x(), 0);
        box.min.y() = std::max<coord_t>(box.min.y(), 0);
        box.max.x() = std::min<coord_t>(box.max.x(), coord_t(m_cols) - 1);
        box.max.y() = std::min<coord_t>(box.max.y(), coord_t(m_rows) - 1);
        return box;
    }

    // Visit candidate cells in row order; inverted limits visit nothing, and false stops traversal.
    template<typename VISITOR> void visit_cells_intersecting_box(BoundingBox box, VISITOR &visitor) const {
        box = this->get_cells_intersecting_box(box);
        for (coord_t row = box.min.y(); row <= box.max.y(); ++row)
            for (coord_t col = box.min.x(); col <= box.max.x(); ++col)
                if (!visitor(row, col)) return;
    }

    // Report whether an in-range cell references at least one contour segment.
    bool cell_has_data(coord_t row, coord_t col) const {
        assert(row >= 0 && size_t(row) < m_rows);
        assert(col >= 0 && size_t(col) < m_cols);
        const Cell &cell = m_cells[row * m_cols + col];
        return cell.begin != cell.end;
    }

    // Copy the endpoints of a referenced contour segment into an integer line.
    Line line(const std::pair<size_t, size_t> &index) const {
        const Contour &contour = m_contours[index.first];
        return Line(Point(contour.segment_start(index.second)), Point(contour.segment_end(index.second)));
    }

    // Borrow the indexed contour views for the lifetime of this grid state.
    const std::vector<Contour> &contours() const { return m_contours; }

    // Borrow the coordinate bounds used by cell traversal.
    const BoundingBox &bbox() const { return m_bbox; }
    // Return the cell width and height in scaled units.
    const coord_t resolution() const { return m_resolution; }
    // Return the number of cell rows.
    const size_t rows() const { return m_rows; }
    // Return the number of cell columns.
    const size_t cols() const { return m_cols; }

    // Result of a signed closest-point search on closed contours.
    struct ClosestPointResult
    {
        size_t contour_idx = size_t(-1);
        size_t start_point_idx = size_t(-1);
        // Signed distance to the closest point.
        double distance = std::numeric_limits<double>::max();
        // Parameter of the closest point on the edge starting at start_point_idx, in [0, 1].
        double t = 0.;

        // Report whether a contour was found by the search.
        bool valid() const { return contour_idx != size_t(-1); }
    };

    // Search closed contours within the radius; return an invalid result when no eligible point is found.
    // only_this_contour limits the search to that contour index; size_t(-1) searches all contours.
    ClosestPointResult closest_point_signed_distance(const Point &pt, coord_t search_radius,
                                                     size_t only_this_contour = size_t(-1)) const;

    // Borrow the segment-index range for an in-range row and column.
    std::pair<std::vector<std::pair<size_t, size_t>>::const_iterator,
              std::vector<std::pair<size_t, size_t>>::const_iterator>
    cell_data_range(int64_t row, int64_t col) const {
        assert(row >= 0 && size_t(row) < m_rows);
        assert(col >= 0 && size_t(col) < m_cols);
        const Cell &cell = m_cells[row * m_cols + col];
        return std::make_pair(m_cell_data.begin() + cell.begin, m_cell_data.begin() + cell.end);
    }

    // Return endpoint values for an in-range contour and segment pair.
    std::pair<c_point, c_point> segment(const std::pair<size_t, size_t> &contour_and_segment_idx) const {
        const Contour &contour = m_contours[contour_and_segment_idx.first];
        const size_t iseg = contour_and_segment_idx.second;
        return std::make_pair(contour.segment_start(iseg), contour.segment_end(iseg));
    }

    // Visit crossed cells until false; both line endpoints must lie inside bbox().
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
        // Initialize an empty range of segment references.
        Cell() : begin(0), end(0) {}
        size_t begin;
        size_t end;
    };

    // Build contiguous per-cell segment ranges from the borrowed contours at a positive resolution.
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
}} // namespace slic3r_api::PluginCommon

#endif // plugins_common_edge_grid_hpp_
