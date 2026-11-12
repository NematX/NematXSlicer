///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ Copyright (c) Prusa Research 2016 - 2023 Vojtech Bubnik @bubnikv, Lukas Hejl @hejllukas, Lukas Matena @lukasmatena
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "PluginEdgeGrid.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

/*
Contour indexing for plugin connection queries
==============================================

This implementation supplies Grid creation and signed closest-point searches
for infill connection helpers. Creation first collects borrowed contour views,
discarding inputs with fewer than two points and removing a repeated endpoint
from open inputs that are actually closed. It measures the bounds and adds a
small margin so boundary segments fit inside the grid.

Rasterization counts segment references in each cell. Prefix sums turn those
counts into non-overlapping ranges in one flat array; a second rasterization
fills the ranges with contour/segment pairs. This keeps each cell's candidates
contiguous without copying source coordinates. Closest-point queries scan
nearby cells and compute distances and signs from the referenced closed rings.
The caller must keep source arrays alive, unchanged and at stable addresses
until queries finish. Editing points requires rebuilding the grid.

Grid
|-- create: collect work or API contour views and normalize closure
|   `-- create_from_m_contours: measure bounds, count hits, prefix-sum, fill pairs
`-- closest_point_signed_distance: scan nearby cells and evaluate ring edges
*/
namespace slic3r_api { namespace NematXWipeTowerCopy {
namespace EdgeGrid {

using namespace polygon_work;

void Grid::create(const std::vector<Points> &polygons, coord_t resolution, bool open_polylines)
{
    ContourViews contours;
    contours.reserve(polygons.size());
    for (const Points &points : polygons)
        contours.emplace_back(points);
    this->create(contours, resolution, open_polylines);
}

void Grid::create(const ContourViews &contours, coord_t resolution, bool open_polylines)
{
    // Collect the contours (port of EdgeGrid.cpp:69-90).
    m_contours.clear();
    m_contours.reserve(size_t(std::count_if(contours.begin(), contours.end(), [](const ContourView &contour) { return contour.size() > 1; })));
    for (const ContourView &source : contours)
        if (source.size() > 1) {
            ContourView contour = source;
            bool open = open_polylines;
            if (open_polylines) {
                if (same_point(contour.front(), contour.back())) {
                    open = false;
                    contour = contour.with_size(contour.size() - 1);
                }
            } else {
                assert(!same_point(contour.front(), contour.back()));
            }
            m_contours.emplace_back(contour, open);
        }

    create_from_m_contours(resolution);
}

void Grid::create(const slic3r_api::ExPolygon &expoly, coord_t resolution)
{
    ContourViews contours;
    contours.reserve(1 + expoly.hole_size());
    contours.emplace_back(expoly.contour().view());
    for (uint32_t hole_idx = 0; hole_idx < expoly.hole_size(); ++hole_idx)
        contours.emplace_back(expoly.hole(hole_idx).view());
    this->create(contours, resolution, false);
}

void Grid::create(const Polygon &polygon, coord_t resolution)
{
    // Verbatim port of EdgeGrid.cpp:35-43.
    ContourViews contours;
    if (! polygon.empty())
        contours.emplace_back(polygon.points);
    this->create(contours, resolution, false);
}

void Grid::create(const Polygons &polygons, coord_t resolution)
{
    // Verbatim port of EdgeGrid.cpp:45-55.
    ContourViews contours;
    contours.reserve(polygons.size());
    for (const Polygon &polygon : polygons)
        if (! polygon.empty())
            contours.emplace_back(polygon.points);
    this->create(contours, resolution, false);
}

void Grid::create(const std::vector<const Polygon*> &polygons, coord_t resolution)
{
    // Verbatim port of EdgeGrid.cpp:57-67.
    ContourViews contours;
    contours.reserve(polygons.size());
    for (const Polygon *polygon : polygons)
        if (! polygon->empty())
            contours.emplace_back(polygon->points);
    this->create(contours, resolution, false);
}

void Grid::create_from_m_contours(coord_t resolution)
{
    // Verbatim port of EdgeGrid.cpp:157-349.
    assert(resolution > 0);
    // 1) Measure the bounding box.
    for (const Contour &contour : m_contours) {
        assert(contour.num_segments() > 0);
        assert(!same_point(contour.front(), contour.back()));
        for (size_t idx = 0; idx < contour.size(); ++idx)
            m_bbox.merge(contour[idx]);
    }

    coord_t eps = 16;
    m_bbox.min.x() -= eps;
    m_bbox.min.y() -= eps;
    m_bbox.max.x() += eps;
    m_bbox.max.y() += eps;

    // 2) Initialize the edge grid.
    m_resolution = resolution;
    m_cols = (m_bbox.max.x() - m_bbox.min.x() + m_resolution - 1) / m_resolution;
    m_rows = (m_bbox.max.y() - m_bbox.min.y() + m_resolution - 1) / m_resolution;
    m_cells.assign(m_rows * m_cols, Cell());

    // 3) First round of contour rasterization, count the edges per grid cell.
    for (size_t i = 0; i < m_contours.size(); ++i) {
        const Contour &contour = m_contours[i];
        for (size_t j = 0; j < contour.num_segments(); ++j) {
            // End points of the line segment.
            Point p1(contour.segment_start(j));
            Point p2(contour.segment_end(j));
            p1.x() -= m_bbox.min.x();
            p1.y() -= m_bbox.min.y();
            p2.x() -= m_bbox.min.x();
            p2.y() -= m_bbox.min.y();
            // Get the cells of the end points.
            coord_t ix = p1.x() / m_resolution;
            coord_t iy = p1.y() / m_resolution;
            coord_t ixb = p2.x() / m_resolution;
            coord_t iyb = p2.y() / m_resolution;
            assert(ix >= 0 && size_t(ix) < m_cols);
            assert(iy >= 0 && size_t(iy) < m_rows);
            assert(ixb >= 0 && size_t(ixb) < m_cols);
            assert(iyb >= 0 && size_t(iyb) < m_rows);
            // Account for the end points.
            ++m_cells[iy * m_cols + ix].end;
            if (ix == ixb && iy == iyb)
                // Both ends fall into the same cell.
                continue;
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
                        } else if (ex == ey) {
                            ex = int64_t(dy) * m_resolution;
                            ey = int64_t(dx) * m_resolution;
                            ix += 1;
                            iy += 1;
                        } else {
                            assert(ex > ey);
                            ex -= ey;
                            ey = int64_t(dx) * m_resolution;
                            iy += 1;
                        }
                        ++m_cells[iy * m_cols + ix].end;
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
                        } else {
                            ex -= ey;
                            ey = int64_t(dx) * m_resolution;
                            iy -= 1;
                        }
                        ++m_cells[iy * m_cols + ix].end;
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
                        } else {
                            assert(ex >= ey);
                            ex -= ey;
                            ey = int64_t(dx) * m_resolution;
                            iy += 1;
                        }
                        ++m_cells[iy * m_cols + ix].end;
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
                        } else if (ex == ey) {
                            // The lower edge of a grid cell belongs to the cell.
                            // Handle the case where the ray may cross the lower left corner of a cell in a general case,
                            // or a left or lower edge in a degenerate case (horizontal or vertical line).
                            if (dx > 0) {
                                ex = int64_t(dy) * m_resolution;
                                ix -= 1;
                            }
                            if (dy > 0) {
                                ey = int64_t(dx) * m_resolution;
                                iy -= 1;
                            }
                        } else {
                            assert(ex > ey);
                            ex -= ey;
                            ey = int64_t(dx) * m_resolution;
                            iy -= 1;
                        }
                        ++m_cells[iy * m_cols + ix].end;
                    } while (ix != ixb || iy != iyb);
                }
            }
        }
    }

    // 4) Prefix sum the numbers of hits per cells to get an index into m_cell_data.
    size_t cnt = m_cells.front().end;
    for (size_t i = 1; i < m_cells.size(); ++i) {
        m_cells[i].begin = cnt;
        cnt += m_cells[i].end;
        m_cells[i].end = cnt;
    }

    // 5) Allocate the cell data.
    m_cell_data.assign(cnt, std::pair<size_t, size_t>(size_t(-1), size_t(-1)));

    // 6) Finally fill in m_cell_data by rasterizing the lines once again.
    for (size_t i = 0; i < m_cells.size(); ++i)
        m_cells[i].end = m_cells[i].begin;

    struct Visitor
    {
        Visitor(std::vector<std::pair<size_t, size_t>> &cell_data, std::vector<Cell> &cells, size_t cols) :
            cell_data(cell_data), cells(cells), cols(cols), i(0), j(0) {}

        inline bool operator()(coord_t iy, coord_t ix) {
            cell_data[cells[iy * cols + ix].end++] = std::pair<size_t, size_t>(i, j);
            // Continue traversing the grid along the edge.
            return true;
        }

        std::vector<std::pair<size_t, size_t>> &cell_data;
        std::vector<Cell> &cells;
        size_t cols;
        size_t i;
        size_t j;
    } visitor(m_cell_data, m_cells, m_cols);

    assert(visitor.i == 0);
    for (; visitor.i < m_contours.size(); ++visitor.i) {
        const Contour &contour = m_contours[visitor.i];
        for (visitor.j = 0; visitor.j < contour.num_segments(); ++visitor.j)
            this->visit_cells_intersecting_line(to_point(contour.segment_start(visitor.j)), to_point(contour.segment_end(visitor.j)),
                                                visitor);
    }
}

Grid::ClosestPointResult Grid::closest_point_signed_distance(const Point &pt, coord_t search_radius,
                                                             size_t only_this_contour) const
{
    // Verbatim port of EdgeGrid.cpp:1068-1199.
    BoundingBox bbox;
    bbox.min = bbox.max = Point(pt.x() - m_bbox.min.x(), pt.y() - m_bbox.min.y());
    bbox.defined = true;
    // Upper boundary, round to grid and test validity.
    bbox.max.x() += search_radius;
    bbox.max.y() += search_radius;
    ClosestPointResult result;
    if (bbox.max.x() < 0 || bbox.max.y() < 0)
        return result;
    bbox.max.x() /= m_resolution;
    bbox.max.y() /= m_resolution;
    if (size_t(bbox.max.x()) >= m_cols)
        bbox.max.x() = m_cols - 1;
    if (size_t(bbox.max.y()) >= m_rows)
        bbox.max.y() = m_rows - 1;
    // Lower boundary, round to grid and test validity.
    bbox.min.x() -= search_radius;
    bbox.min.y() -= search_radius;
    if (bbox.min.x() < 0)
        bbox.min.x() = 0;
    if (bbox.min.y() < 0)
        bbox.min.y() = 0;
    bbox.min.x() /= m_resolution;
    bbox.min.y() /= m_resolution;
    // Is the interval empty?
    if (bbox.min.x() > bbox.max.x() || bbox.min.y() > bbox.max.y())
        return result;
    // Traverse all cells in the bounding box.
    double d_min = double(search_radius);
    // Signum of the distance field at pt.
    int sign_min = 0;
    double l2_seg_min = 1.;
    for (coord_t r = bbox.min.y(); r <= bbox.max.y(); ++r) {
        for (coord_t c = bbox.min.x(); c <= bbox.max.x(); ++c) {
            const Cell &cell = m_cells[r * m_cols + c];
            for (size_t i = cell.begin; i < cell.end; ++i) {
                const size_t contour_idx = m_cell_data[i].first;
                if (only_this_contour != size_t(-1) && only_this_contour != contour_idx)
                    continue;
                const Contour &contour = m_contours[contour_idx];
                assert(contour.closed());
                const size_t ipt = m_cell_data[i].second;
                // End points of the line segment.
                const Point p1 = to_point(contour.segment_start(ipt));
                const Point p2 = to_point(contour.segment_end(ipt));
                const Point v_seg = p2 - p1;
                const Point v_pt = pt - p1;
                // dot(p2-p1, pt-p1)
                int64_t t_pt = int64_t(v_seg.x()) * int64_t(v_pt.x()) + int64_t(v_seg.y()) * int64_t(v_pt.y());
                // l2 of seg
                int64_t l2_seg = int64_t(v_seg.x()) * int64_t(v_seg.x()) + int64_t(v_seg.y()) * int64_t(v_seg.y());
                if (t_pt < 0) {
                    // Closest to p1.
                    double dabs = sqrt(int64_t(v_pt.x()) * int64_t(v_pt.x()) + int64_t(v_pt.y()) * int64_t(v_pt.y()));
                    if (dabs < d_min) {
                        // Previous point.
                        const Point p0 = to_point(contour.segment_prev(ipt));
                        const Point v_seg_prev = p1 - p0;
                        int64_t t2_pt = int64_t(v_seg_prev.x()) * int64_t(v_pt.x()) + int64_t(v_seg_prev.y()) * int64_t(v_pt.y());
                        if (t2_pt > 0) {
                            // Inside the wedge between the previous and the next segment.
                            d_min = dabs;
                            // Set the signum depending on whether the vertex is convex or reflex.
                            int64_t det = int64_t(v_seg_prev.x()) * int64_t(v_seg.y()) - int64_t(v_seg_prev.y()) * int64_t(v_seg.x());
                            assert(det != 0);
                            sign_min = (det > 0) ? 1 : -1;
                            result.contour_idx = contour_idx;
                            result.start_point_idx = ipt;
                            result.t = 0.;
#ifndef NDEBUG
                            Vec2d vfoot = to_vec2d(p1 - pt);
                            double dist_foot = vfoot.norm();
                            double dist_foot_err = dist_foot - d_min;
                            assert(std::abs(dist_foot_err) < 1e-7 * d_min);
#endif /* NDEBUG */
                        }
                    }
                } else if (t_pt > l2_seg) {
                    // Closest to p2. Then p2 is the starting point of another segment, which shall be discovered
                    // in the same cell.
                    continue;
                } else {
                    // Closest to the segment.
                    assert(t_pt >= 0 && t_pt <= l2_seg);
                    int64_t d_seg = int64_t(v_seg.y()) * int64_t(v_pt.x()) - int64_t(v_seg.x()) * int64_t(v_pt.y());
                    double d = double(d_seg) / sqrt(double(l2_seg));
                    double dabs = std::abs(d);
                    if (dabs < d_min) {
                        d_min = dabs;
                        sign_min = (d_seg < 0) ? -1 : ((d_seg == 0) ? 0 : 1);
                        l2_seg_min = l2_seg;
                        result.contour_idx = contour_idx;
                        result.start_point_idx = ipt;
                        result.t = t_pt;
#ifndef NDEBUG
                        Vec2d foot = to_vec2d(p1) * (1. - result.t / l2_seg_min) + to_vec2d(p2) * (result.t / l2_seg_min);
                        Vec2d vfoot = foot - to_vec2d(pt);
                        double dist_foot = vfoot.norm();
                        double dist_foot_err = dist_foot - d_min;
                        assert(std::abs(dist_foot_err) < 1e-7 || std::abs(dist_foot_err) < 1e-7 * d_min);
#endif /* NDEBUG */
                    }
                }
            }
        }
    }
    if (result.contour_idx != size_t(-1) && d_min <= double(search_radius)) {
        result.distance = d_min * sign_min;
        result.t /= l2_seg_min;
        assert(result.t >= 0. && result.t <= 1.);
#ifndef NDEBUG
        {
            const Contour &contour = m_contours[result.contour_idx];
            const Point p1 = to_point(contour.segment_start(result.start_point_idx));
            const Point p2 = to_point(contour.segment_end(result.start_point_idx));
            Vec2d vfoot;
            if (result.t == 0)
                vfoot = to_vec2d(p1) - to_vec2d(pt);
            else
                vfoot = to_vec2d(p1) * (1. - result.t) + to_vec2d(p2) * result.t - to_vec2d(pt);
            double dist_foot = vfoot.norm();
            double dist_foot_err = dist_foot - std::abs(result.distance);
            assert(std::abs(dist_foot_err) < 1e-7 || std::abs(dist_foot_err) < 1e-7 * std::abs(result.distance));
        }
#endif /* NDEBUG */
    } else
        result = ClosestPointResult();
    return result;
}

} // namespace EdgeGrid
}} // namespace slic3r_api::NematXWipeTowerCopy
