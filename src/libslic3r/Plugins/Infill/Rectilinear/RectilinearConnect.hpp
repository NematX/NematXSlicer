///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef plugins_infill_rectilinear_connect_hpp_
#define plugins_infill_rectilinear_connect_hpp_

/*
Infill connect helpers for the rectilinear pattern port
======================================================

Verbatim port of the connect stage of src/libslic3r/Fill/FillBase.cpp:

- namespace PrusaSimpleConnect (FillBase.cpp:967-1470),
- namespace FakePerimeterConnect (FillBase.cpp:1471-3381),
- the FakePerimeterConnect support-base connector (FillBase.cpp:3318-3772),
- the Fill::connect_infill() dispatchers (FillBase.cpp:3849-3863).

The algorithm uses API views at its boundary and private mutable work buffers
for the path operations that must insert, rotate and simplify points.
The Eigen point casts became to_vec2d() / to_point() calls.

The two connect strategies mirror the legacy behaviour:
- PrusaSimpleConnect links the infill lines along the boundary without
  extruding on it (used when anchor_length_max == 0),
- FakePerimeterConnect extrudes a fake perimeter of bounded length
  (params.anchor_length / params.anchor_length_max) while linking.
*/

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/c/steps/slic3r_step_infill.h"

#include "libslic3r/Plugins/Common/ClipperHelpers.hpp"
#include "libslic3r/Plugins/Common/PluginEdgeGrid.hpp"

namespace slic3r_api { namespace Infill { namespace RectilinearPatternsPlugin {
namespace work = PluginCommon::polygon_work;
namespace clipper = PluginCommon::clipper;
namespace EdgeGrid = PluginCommon::EdgeGrid;
namespace connect {

using namespace work;
using namespace clipper;
// Select mutable work types only in this namespace; RectilinearWithPerimeter.cpp uses API views with the same type names.
using work::ExPolygon;
using work::Polygon;
using work::Polyline;

// A boundary is represented as one outer contour followed by its holes. The
// views borrow the source geometry; mutable buffers are created only when the
// connection algorithm inserts infill junctions.
using BoundaryContours = ContourViews;

inline BoundaryContours boundary_contours(const ExPolygon &boundary)
{
    BoundaryContours out;
    out.reserve(boundary.holes.size() + 1);
    out.emplace_back(boundary.contour.points);
    for (const Polygon &hole : boundary.holes)
        out.emplace_back(hole.points);
    return out;
}

inline BoundaryContours boundary_contours(const Polygons &boundary)
{
    BoundaryContours out;
    out.reserve(boundary.size());
    for (const Polygon &polygon : boundary)
        out.emplace_back(polygon.points);
    return out;
}

// A conservative read-only check for the stick patterns handled by
// remove_sticks(). If none of the original cyclic triples is a stick, the
// compaction pass cannot remove a point and the source contour may be borrowed.
inline bool contour_has_stick(const ContourView &contour)
{
    if (contour.size() < 3)
        return false;
    for (size_t idx = 1; idx + 1 < contour.size(); ++idx)
        if (clipper::is_stick(to_point(contour[idx - 1]), to_point(contour[idx]), to_point(contour[idx + 1])))
            return true;
    return clipper::is_stick(to_point(contour[contour.size() - 2]), to_point(contour.back()), to_point(contour.front())) ||
           clipper::is_stick(to_point(contour.back()), to_point(contour.front()), to_point(contour[1]));
}

// Own only contours that need the cleanup. Clean API contours remain direct
// views; the outer contour and the holes are all cleaned, like
// remove_sticks(ExPolygon&) does.
class BoundaryViewStorage
{
public:
    explicit BoundaryViewStorage(const slic3r_api::ExPolygon &boundary)
    {
        m_owned_contours.reserve(size_t(boundary.hole_size()) + 1);
        m_contours.reserve(size_t(boundary.hole_size()) + 1);

        const ContourView outer(boundary.contour().view());
        if (contour_has_stick(outer))
            this->append_cleaned(outer, false);
        else
            m_contours.emplace_back(outer);

        for (uint32_t idx = 0; idx < boundary.hole_size(); ++idx) {
            const ContourView hole(boundary.hole(idx).view());
            if (hole.size() < 3)
                continue;
            if (contour_has_stick(hole))
                this->append_cleaned(hole, true);
            else
                m_contours.emplace_back(hole);
        }
    }

    const BoundaryContours &contours() const { return m_contours; }
    size_t materialized_contour_count() const { return m_owned_contours.size(); }

    size_t materialized_point_count() const
    {
        size_t out = 0;
        for (const Points &points : m_owned_contours)
            out += points.size();
        return out;
    }

private:
    static Polygon copy_contour(const ContourView &source)
    {
        Polygon out;
        out.points.reserve(source.size());
        for (size_t idx = 0; idx < source.size(); ++idx)
            out.points.emplace_back(source[idx]);
        return out;
    }

    // remove_sticks() reports whether it erased points, so it decides between
    // borrowing the source view and keeping the cleaned copy.
    void append_cleaned(const ContourView &source, bool hole)
    {
        Polygon cleaned = copy_contour(source);
        const bool changed = clipper::remove_sticks(cleaned);
        if (hole && cleaned.points.size() < 3)
            return;
        if (! changed) {
            m_contours.emplace_back(source);
            return;
        }
        m_owned_contours.emplace_back(std::move(cleaned.points));
        m_contours.emplace_back(m_owned_contours.back());
    }

    std::vector<Points> m_owned_contours;
    BoundaryContours    m_contours;
};

// Return the segment containing the point reached after clipping a polyline
// from either end. The segment index always names [idx_segment, idx_segment + 1).
struct SegmentPoint {
    size_t idx_segment = std::numeric_limits<size_t>::max();
    double t;
    Vec2d  point;

    bool valid() const { return idx_segment != std::numeric_limits<size_t>::max(); }
};

static inline SegmentPoint clip_start_segment_and_point(const Points& polyline, double distance)
{
    assert(polyline.size() >= 2);
    assert(distance > 0.);
    SegmentPoint out;
    if (polyline.size() >= 2) {
        Vec2d pt_prev = to_vec2d(polyline.front());
        for (size_t i = 1; i < polyline.size(); ++i) {
            Vec2d pt = to_vec2d(polyline[i]);
            Vec2d v = pt - pt_prev;
            double l = v.norm();
            if (l > distance) {
                out.idx_segment = i - 1;
                out.t = distance / l;
                out.point = pt_prev + out.t * v;
                break;
            }
            distance -= l;
            pt_prev = pt;
        }
    }
    return out;
}

static inline SegmentPoint clip_end_segment_and_point(const Points& polyline, double distance)
{
    assert(polyline.size() >= 2);
    assert(distance > 0.);
    SegmentPoint out;
    if (polyline.size() >= 2) {
        Vec2d pt_next = to_vec2d(polyline.back());
        for (int i = int(polyline.size()) - 2; i >= 0; --i) {
            Vec2d pt = to_vec2d(polyline[i]);
            Vec2d v = pt - pt_next;
            double l = v.norm();
            if (l > distance) {
                out.idx_segment = i;
                out.t = distance / l;
                out.point = pt_next + out.t * v;
                // Store the parameter referenced to the starting point of a segment.
                out.t = 1. - out.t;
                break;
            }
            distance -= l;
            pt_next = pt;
        }
    }
    return out;
}

// Remove a Euclidean distance from a work polyline using the core rounding rule.
static inline void clip_polyline_end(Polyline &polyline, double distance)
{
    while (distance > 0. && !polyline.points.empty()) {
        const Vec2d last_point = to_vec2d(polyline.points.back());
        polyline.points.pop_back();
        if (polyline.points.empty())
            break;
        const Vec2d vector = to_vec2d(polyline.points.back()) - last_point;
        const double length = vector.norm();
        if (length > distance) {
            const Vec2d point = last_point + vector * (distance / length);
            polyline.points.emplace_back(coord_t(std::round(point.x())), coord_t(std::round(point.y())));
            return;
        }
        distance -= length;
    }
}

// Remove a Euclidean distance from the start of a work polyline.
static inline void clip_polyline_start(Polyline &polyline, double distance)
{
    polyline.reverse();
    clip_polyline_end(polyline, distance);
    if (polyline.points.size() >= 2)
        polyline.reverse();
}

namespace PrusaSimpleConnect {


    struct ContourPointData {
        ContourPointData(float param) : param(param) {}
        // Eucleidean position of the contour point along the contour.
        float param = 0.f;
        // Was the segment starting with this contour point extruded?
        bool  segment_consumed = false;
        // Was this point extruded over?
        bool  point_consumed = false;
    };

    // Verify whether the contour from point idx_start to point idx_end could be taken (whether all segments along the contour were not yet extruded).
    static bool could_take(const std::vector<ContourPointData>& contour_data, size_t idx_start, size_t idx_end)
    {
        assert(idx_start != idx_end);
        for (size_t i = idx_start; i != idx_end; ) {
            if (contour_data[i].segment_consumed || contour_data[i].point_consumed)
                return false;
            if (++i == contour_data.size())
                i = 0;
        }
        return !contour_data[idx_end].point_consumed;
    }

    // Connect end of pl1 to the start of pl2 using the perimeter contour.
    // The idx_start and idx_end are ordered so that the connecting polyline points will be taken with increasing indices.
    static void take(Polyline& pl1, Polyline&& pl2, const Points& contour, std::vector<ContourPointData>& contour_data, size_t idx_start, size_t idx_end, bool reversed)
    {
        assert(idx_start != idx_end);

        {
            // Reserve memory at pl1 for the connecting contour and pl2.
            int new_points = int(idx_end) - int(idx_start) - 1;
            if (new_points < 0)
                new_points += int(contour.size());
            pl1.points.reserve(pl1.points.size() + size_t(new_points) + pl2.points.size());
        }

        contour_data[idx_start].point_consumed = true;
        contour_data[idx_start].segment_consumed = true;
        contour_data[idx_end].point_consumed = true;

        if (reversed) {
            size_t i = (idx_end == 0) ? contour_data.size() - 1 : idx_end - 1;
            while (i != idx_start) {
                contour_data[i].point_consumed = true;
                contour_data[i].segment_consumed = true;
                pl1.points.emplace_back(contour[i]);
                if (i == 0)
                    i = contour_data.size();
                --i;
            }
        } else {
            size_t i = idx_start;
            if (++i == contour_data.size())
                i = 0;
            while (i != idx_end) {
                contour_data[i].point_consumed = true;
                contour_data[i].segment_consumed = true;
                pl1.points.emplace_back(contour[i]);
                if (++i == contour_data.size())
                    i = 0;
            }
        }

        append(pl1.points, std::move(pl2.points));
    }

    // Optimized version with the precalculated v1 = p1b - p1a and l1_2 = v1.squaredNorm().
    // Assumption: l1_2 > EPSILON.
    static inline double segment_point_distance_squared(const Vec2d& p1a, const Vec2d& p1b, const Vec2d& v1, const double l1_2, const Vec2d& p2)
    {
        assert(l1_2 > EPSILON);
        Vec2d  v12 = p2 - p1a;
        double t = v12.dot(v1);
        return (t <= 0.) ? v12.squaredNorm() :
            (t >= l1_2) ? (p2 - p1b).squaredNorm() :
            ((t / l1_2) * v1 - v12).squaredNorm();
    }

    static inline double segment_point_distance_squared(const Vec2d& p1a, const Vec2d& p1b, const Vec2d& p2)
    {
        const Vec2d  v = p1b - p1a;
        const double l2 = v.squaredNorm();
        if (l2 < EPSILON)
            // p1a == p1b
            return (p2 - p1a).squaredNorm();
        return segment_point_distance_squared(p1a, p1b, v, v.squaredNorm(), p2);
    }

    // Distance to the closest point of line.
    static inline double min_distance_of_segments(const Vec2d& p1a, const Vec2d& p1b, const Vec2d& p2a, const Vec2d& p2b)
    {
        Vec2d   v1 = p1b - p1a;
        double  l1_2 = v1.squaredNorm();
        if (l1_2 < EPSILON)
            // p1a == p1b: Return distance of p1a from the (p2a, p2b) segment.
            return segment_point_distance_squared(p2a, p2b, p1a);

        Vec2d   v2 = p2b - p2a;
        double  l2_2 = v2.squaredNorm();
        if (l2_2 < EPSILON)
            // p2a == p2b: Return distance of p2a from the (p1a, p1b) segment.
            return segment_point_distance_squared(p1a, p1b, v1, l1_2, p2a);

        return std::min(
            std::min(segment_point_distance_squared(p1a, p1b, v1, l1_2, p2a), segment_point_distance_squared(p1a, p1b, v1, l1_2, p2b)),
            std::min(segment_point_distance_squared(p2a, p2b, v2, l2_2, p1a), segment_point_distance_squared(p2a, p2b, v2, l2_2, p1b)));
    }

    // Mark the segments of split boundary as consumed if they are very close to some of the infill line.
    inline void mark_boundary_segments_touching_infill(
        const std::vector<Points>& boundary,
        std::vector<std::vector<ContourPointData>>& boundary_data,
        const BoundingBox& boundary_bbox,
        const Polylines& infill,
        const double							     clip_distance,
        const double 								 distance_colliding)
    {
        EdgeGrid::Grid grid;
        grid.set_bbox(boundary_bbox.inflated(distance_colliding * 1.43));
        // Inflate the bounding box by a thick line width.
        grid.create(boundary, coord_t(clip_distance + scale_i(10.)));

        struct Visitor {
            Visitor(const EdgeGrid::Grid& grid, const std::vector<Points>& boundary, std::vector<std::vector<ContourPointData>>& boundary_data, const double dist2_max) :
                grid(grid), boundary(boundary), boundary_data(boundary_data), dist2_max(dist2_max) {}

            void init(const Vec2d& pt1, const Vec2d& pt2) {
                this->pt1 = &pt1;
                this->pt2 = &pt2;
            }

            bool operator()(coord_t iy, coord_t ix) {
                // Called with a row and colum of the grid cell, which is intersected by a line.
                auto cell_data_range = this->grid.cell_data_range(iy, ix);
                for (auto it_contour_and_segment = cell_data_range.first; it_contour_and_segment != cell_data_range.second; ++it_contour_and_segment) {
                    // End points of the line segment and their vector.
                    auto segment = this->grid.segment(*it_contour_and_segment);
                    const Vec2d seg_pt1 = to_vec2d(segment.first);
                    const Vec2d seg_pt2 = to_vec2d(segment.second);
                    if (min_distance_of_segments(seg_pt1, seg_pt2, *this->pt1, *this->pt2) < this->dist2_max) {
                        // Mark this boundary segment as touching the infill line.
                        ContourPointData& bdp = boundary_data[it_contour_and_segment->first][it_contour_and_segment->second];
                        bdp.segment_consumed = true;
                        // There is no need for checking seg_pt2 as it will be checked the next time.
                        bool point_touching = false;
                        if (segment_point_distance_squared(*this->pt1, *this->pt2, seg_pt1) < this->dist2_max) {
                            point_touching = true;
                            bdp.point_consumed = true;
                        }
                    }
                }
                // Continue traversing the grid along the edge.
                return true;
            }

            const EdgeGrid::Grid& grid;
            const std::vector<Points>& boundary;
            std::vector<std::vector<ContourPointData>>& boundary_data;
            // Maximum distance between the boundary and the infill line allowed to consider the boundary not touching the infill line.
            const double								 dist2_max;

            const Vec2d* pt1;
            const Vec2d* pt2;
        } visitor(grid, boundary, boundary_data, distance_colliding * distance_colliding);

        BoundingBoxf bboxf(to_vec2d(boundary_bbox.min), to_vec2d(boundary_bbox.max));
        bboxf.offset(coordf_t(-SCALED_EPSILON));

        for (const Polyline& polyline : infill) {
            // Clip the infill polyline by the Eucledian distance along the polyline.
            SegmentPoint start_point = clip_start_segment_and_point(polyline.points, clip_distance);
            SegmentPoint end_point = clip_end_segment_and_point(polyline.points, clip_distance);
            if (start_point.valid() && end_point.valid() &&
                (start_point.idx_segment < end_point.idx_segment || (start_point.idx_segment == end_point.idx_segment && start_point.t < end_point.t))) {
                // The clipped polyline is non-empty.
                for (size_t point_idx = start_point.idx_segment; point_idx <= end_point.idx_segment; ++point_idx) {
                    //FIXME extend the EdgeGrid to suport tracing a thick line.
#if 0
                    Point pt1, pt2;
                    Vec2d pt1d, pt2d;
                    if (point_idx == start_point.idx_segment) {
                        pt1d = start_point.point;
                        pt1 = to_point(pt1d);
                    } else {
                        pt1 = polyline.points[point_idx];
                        pt1d = to_vec2d(pt1);
                    }
                    if (point_idx == start_point.idx_segment) {
                        pt2d = end_point.point;
                        pt2 = to_point(pt1d);
                    } else {
                        pt2 = polyline.points[point_idx];
                        pt2d = to_vec2d(pt2);
                    }
                    visitor.init(pt1d, pt2d);
                    grid.visit_cells_intersecting_thick_line(pt1, pt2, distance_colliding, visitor);
#else
                    Vec2d pt1 = (point_idx == start_point.idx_segment) ? start_point.point : to_vec2d(polyline.points[point_idx]);
                    Vec2d pt2 = (point_idx == end_point.idx_segment) ? end_point.point : to_vec2d(polyline.points[point_idx + 1]);
                    visitor.init(pt1, pt2);
                    // Simulate tracing of a thick line. This only works reliably if distance_colliding <= grid cell size.
                    Vec2d v = (pt2 - pt1).normalized() * distance_colliding;
                    Vec2d vperp(-v.y(), v.x());
                    Vec2d a = pt1 - v - vperp;
                    Vec2d b = pt1 + v - vperp;
                    if (liang_barsky_line_clipping(a, b, bboxf))
                        grid.visit_cells_intersecting_line(to_point(a), to_point(b), visitor);
                    a = pt1 - v + vperp;
                    b = pt1 + v + vperp;
                    if (liang_barsky_line_clipping(a, b, bboxf))
                        grid.visit_cells_intersecting_line(to_point(a), to_point(b), visitor);
#endif
                }
            }
        }
    }

    inline void connect_infill(Polylines &&infill_ordered, const BoundaryContours &boundary_src, Polylines &polylines_out, const coord_t spacing, const raw_infill_pattern_params &params)
    {
        assert(!infill_ordered.empty());
        assert(!boundary_src.empty());
        assert(!boundary_src.front().empty());

        BoundingBox bbox = get_extents(boundary_src.front());
        bbox.offset(coordf_t(SCALED_EPSILON));

        // 1) Add the end points of infill_ordered to boundary_src.
        std::vector<Points>					   		boundary;
        std::vector<std::vector<ContourPointData>> 	boundary_data;
        boundary.assign(boundary_src.size(), Points());
        boundary_data.assign(boundary_src.size(), std::vector<ContourPointData>());
        // Mapping the infill_ordered end point to a (contour, point) of boundary.
        std::vector<std::pair<size_t, size_t>> map_infill_end_point_to_boundary;
        static constexpr auto                       boundary_idx_unconnected = std::numeric_limits<size_t>::max();
        map_infill_end_point_to_boundary.assign(infill_ordered.size() * 2, std::pair<size_t, size_t>(boundary_idx_unconnected, boundary_idx_unconnected));
        {
            // Project the infill_ordered end points onto boundary_src.
            std::vector<std::pair<EdgeGrid::Grid::ClosestPointResult, size_t>> intersection_points;
            {
                EdgeGrid::Grid grid;
                grid.set_bbox(bbox);
                grid.create(boundary_src, scale_i(10.));
                intersection_points.reserve(infill_ordered.size() * 2);
                for (const Polyline& pl : infill_ordered)
                    for (const Point* pt : { &pl.points.front(), &pl.points.back() }) {
                        EdgeGrid::Grid::ClosestPointResult cp = grid.closest_point_signed_distance(*pt, SCALED_EPSILON);
                        if (cp.valid()) {
                            // The infill end point shall lie on the contour.
                            //assert(cp.distance < 2.); //triggered with simple cube with gyroid. Is it dangerous?
                            intersection_points.emplace_back(cp, (&pl - infill_ordered.data()) * 2 + (pt == &pl.points.front() ? 0 : 1));
                        }
                    }
                std::sort(intersection_points.begin(), intersection_points.end(), [](const std::pair<EdgeGrid::Grid::ClosestPointResult, size_t>& cp1, const std::pair<EdgeGrid::Grid::ClosestPointResult, size_t>& cp2) {
                    return   cp1.first.contour_idx < cp2.first.contour_idx ||
                        (cp1.first.contour_idx == cp2.first.contour_idx &&
                        (cp1.first.start_point_idx < cp2.first.start_point_idx ||
                            (cp1.first.start_point_idx == cp2.first.start_point_idx && cp1.first.t < cp2.first.t)));
                });
            }
            auto it = intersection_points.begin();
            auto it_end = intersection_points.end();
            for (size_t idx_contour = 0; idx_contour < boundary_src.size(); ++idx_contour) {
                const ContourView &contour_src = boundary_src[idx_contour];
                Points& contour_dst = boundary[idx_contour];
                for (size_t idx_point = 0; idx_point < contour_src.size(); ++idx_point) {
                    contour_dst.emplace_back(contour_src[idx_point]);
                    for (; it != it_end && it->first.contour_idx == idx_contour && it->first.start_point_idx == idx_point; ++it) {
                        // Add these points to the destination contour.
                        const Vec2d pt1 = to_vec2d(contour_src[idx_point]);
                        const Vec2d pt2 = to_vec2d((idx_point + 1 == contour_src.size() ? contour_src.front() : contour_src[idx_point + 1]));
                        const Vec2d pt = lerp(pt1, pt2, it->first.t);
                        map_infill_end_point_to_boundary[it->second] = std::make_pair(idx_contour, contour_dst.size());
                        contour_dst.emplace_back(to_point(pt));
                    }
                }
                // Parametrize the curve.
                std::vector<ContourPointData>& contour_data = boundary_data[idx_contour];
                contour_data.reserve(contour_dst.size());
                contour_data.emplace_back(ContourPointData(0.f));
                for (size_t i = 1; i < contour_dst.size(); ++i)
                    contour_data.emplace_back(contour_data.back().param + (to_vec2f(contour_dst[i]) - to_vec2f(contour_dst[i - 1])).norm());
                contour_data.front().param = contour_data.back().param + (to_vec2f(contour_dst.back()) - to_vec2f(contour_dst.front())).norm();
            }

            assert(boundary.size() == boundary_src.size());
#if 0
            // Adaptive Cubic Infill produces infill lines, which not always end at the outer boundary.
            assert(std::all_of(map_infill_end_point_to_boundary.begin(), map_infill_end_point_to_boundary.end(),
                [&boundary](const std::pair<size_t, size_t>& contour_point) {
                return contour_point.first < boundary.size() && contour_point.second < boundary[contour_point.first].size();
            }));
            assert(boundary_data.size() == boundary_src.size());
#endif
        }

        // Mark the points and segments of split boundary as consumed if they are very close to some of the infill line.
        {
            // @supermerill used 2. * (spacing)
            const double clip_distance = 3. * (spacing);
            const double distance_colliding = 1.1 * (spacing);
            mark_boundary_segments_touching_infill(boundary, boundary_data, bbox, infill_ordered, clip_distance, distance_colliding);
        }

        // Connection from end of one infill line to the start of another infill line.
        //const float length_max = (spacing);
    //	const float length_max = ((2. / params.density) * spacing);
        const coord_t length_max = ((1000. / params.density) * spacing);
        std::vector<size_t> merged_with(infill_ordered.size());
        for (size_t i = 0; i < merged_with.size(); ++i)
            merged_with[i] = i;
        struct ConnectionCost {
            ConnectionCost(size_t idx_first, double cost, bool reversed) : idx_first(idx_first), cost(cost), reversed(reversed) {}
            size_t  idx_first;
            double  cost;
            bool 	reversed;
        };
        std::vector<ConnectionCost> connections_sorted;
        connections_sorted.reserve(infill_ordered.size() * 2 - 2);
        for (size_t idx_chain = 1; idx_chain < infill_ordered.size(); ++idx_chain) {
            const Polyline& pl1 = infill_ordered[idx_chain - 1];
            const Polyline& pl2 = infill_ordered[idx_chain];
            const std::pair<size_t, size_t>* cp1 = &map_infill_end_point_to_boundary[(idx_chain - 1) * 2 + 1];
            const std::pair<size_t, size_t>* cp2 = &map_infill_end_point_to_boundary[idx_chain * 2];
            if (cp1->first != boundary_idx_unconnected && cp1->first == cp2->first) {
                // End points on the same contour. Try to connect them.
                const std::vector<ContourPointData>& contour_data = boundary_data[cp1->first];
                float param_lo = (cp1->second == 0) ? 0.f : contour_data[cp1->second].param;
                float param_hi = (cp2->second == 0) ? 0.f : contour_data[cp2->second].param;
                float param_end = contour_data.front().param;
                bool  reversed = false;
                if (param_lo > param_hi) {
                    std::swap(param_lo, param_hi);
                    reversed = true;
                }
                assert(param_lo >= 0.f && param_lo <= param_end);
                assert(param_hi >= 0.f && param_hi <= param_end);
                coord_t len = coord_t(param_hi - param_lo);
                if (len < length_max)
                    connections_sorted.emplace_back(idx_chain - 1, len, reversed);
                len = coord_t(param_lo + param_end - param_hi);
                if (len < length_max)
                    connections_sorted.emplace_back(idx_chain - 1, len, !reversed);
            }
        }
        std::sort(connections_sorted.begin(), connections_sorted.end(), [](const ConnectionCost& l, const ConnectionCost& r) { return l.cost < r.cost; });

        //mark point as used depends of connection parameter
        if (params.connection == C_INFILL_CONNECTION_OUTER_SHELL) {
            for (auto it = boundary_data.begin() + 1; it != boundary_data.end(); ++it) {
                for (ContourPointData& pt : *it) {
                    pt.point_consumed = true;
                }
            }
        } else if (params.connection == C_INFILL_CONNECTION_HOLES) {
            for (ContourPointData& pt : boundary_data[0]) {
                pt.point_consumed = true;
            }
        }
        assert(boundary_data.size() == boundary_src.size());

        size_t idx_chain_last = 0;
        for (ConnectionCost& connection_cost : connections_sorted) {
            const std::pair<size_t, size_t>* cp1 = &map_infill_end_point_to_boundary[connection_cost.idx_first * 2 + 1];
            const std::pair<size_t, size_t>* cp1prev = cp1 - 1;
            const std::pair<size_t, size_t>* cp2 = &map_infill_end_point_to_boundary[(connection_cost.idx_first + 1) * 2];
            const std::pair<size_t, size_t>* cp2next = cp2 + 1;
            assert(cp1->first == cp2->first && cp1->first != boundary_idx_unconnected);
            std::vector<ContourPointData>& contour_data = boundary_data[cp1->first];
            if (connection_cost.reversed)
                std::swap(cp1, cp2);
            // Mark the the other end points of the segments to be taken as consumed temporarily, so they will not be crossed
            // by the new connection line.
            bool prev_marked = false;
            bool next_marked = false;
            if (cp1prev->first == cp1->first && !contour_data[cp1prev->second].point_consumed) {
                contour_data[cp1prev->second].point_consumed = true;
                prev_marked = true;
            }
            if (cp2next->first == cp1->first && !contour_data[cp2next->second].point_consumed) {
                contour_data[cp2next->second].point_consumed = true;
                next_marked = true;
            }
            if (could_take(contour_data, cp1->second, cp2->second)) {
                // Indices of the polygons to be connected.
                size_t idx_first = connection_cost.idx_first;
                size_t idx_second = idx_first + 1;
                for (size_t last = idx_first;;) {
                    size_t lower = merged_with[last];
                    if (lower == last) {
                        merged_with[idx_first] = lower;
                        idx_first = lower;
                        break;
                    }
                    last = lower;
                }
                // Connect the two polygons using the boundary contour.
                take(infill_ordered[idx_first], std::move(infill_ordered[idx_second]), boundary[cp1->first], contour_data, cp1->second, cp2->second, connection_cost.reversed);
                // Mark the second polygon as merged with the first one.
                merged_with[idx_second] = merged_with[idx_first];
            }
            if (prev_marked)
                contour_data[cp1prev->second].point_consumed = false;
            if (next_marked)
                contour_data[cp2next->second].point_consumed = false;
        }
        polylines_out.reserve(polylines_out.size() + std::count_if(infill_ordered.begin(), infill_ordered.end(), [](const Polyline& pl) { return !pl.empty(); }));
        for (Polyline& pl : infill_ordered)
            if (!pl.empty())
                polylines_out.emplace_back(std::move(pl));
    }
}

namespace FakePerimeterConnect {

// Forward declarations mirroring FillBase.hpp, so the dispatchers below can
// call the implementations in this namespace.
void connect_infill(storage_handle *storage, Polylines&& infill_ordered, const BoundaryContours &boundary, Polylines& polylines_out, const coord_t spacing, const raw_infill_pattern_params& params);
void connect_infill(storage_handle *storage, Polylines&& infill_ordered, const Polygons& boundary, const BoundingBox& bbox, Polylines& polylines_out, const coord_t spacing, const raw_infill_pattern_params& params);
void connect_infill(storage_handle *storage, Polylines&& infill_ordered, const BoundaryContours &boundary, const BoundingBox& bbox, Polylines& polylines_out, coord_t spacing, const raw_infill_pattern_params& params);

// A single T joint of an infill line to a closed contour or one of its holes.
struct ContourIntersectionPoint {
    // Contour and point on a contour where an infill line is connected to.
    size_t                      contour_idx;
    size_t                      point_idx;
    // Eucleidean parameter of point_idx along its contour.
    double                      param;
    // Other intersection points along the same contour. If there is only a single T-joint on a contour
    // with an intersection line, then the prev_on_contour and next_on_contour remain nulls.
    ContourIntersectionPoint* prev_on_contour{ nullptr };
    ContourIntersectionPoint* next_on_contour{ nullptr };
    // Length of the contour not yet allocated to some extrusion path going back (clockwise), or masked out by some overlapping infill line.
    double                      contour_not_taken_length_prev { std::numeric_limits<double>::max() };
    // Length of the contour not yet allocated to some extrusion path going forward (counter-clockwise), or masked out by some overlapping infill line.
    double                      contour_not_taken_length_next { std::numeric_limits<double>::max() };
    // End point is consumed if an infill line connected to this T-joint was already connected left or right along the contour,
    // or if the infill line was processed, but it was not possible to connect it left or right along the contour.
    bool                        consumed{ false };
    // Whether the contour was trimmed by an overlapping infill line, or whether part of this contour was connected to some infill line.
    bool                        prev_trimmed{ false };
    bool                        next_trimmed{ false };

    void                        consume_prev() { this->contour_not_taken_length_prev = 0.; this->prev_trimmed = true; this->consumed = true; }
    void                        consume_next() { this->contour_not_taken_length_next = 0.; this->next_trimmed = true; this->consumed = true; }

    void                        trim_prev(const double new_len) {
        if (new_len < this->contour_not_taken_length_prev) {
            this->contour_not_taken_length_prev = new_len;
            this->prev_trimmed = true;
        }
    }
    void                        trim_next(const double new_len) {
        if (new_len < this->contour_not_taken_length_next) {
            this->contour_not_taken_length_next = new_len;
            this->next_trimmed = true;
        }
    }

    // The end point of an infill line connected to this T-joint was not processed yet and a piece of the contour could be extruded going backwards.
    bool                        could_take_prev() const throw() { return !this->consumed && this->contour_not_taken_length_prev > SCALED_EPSILON; }
    // The end point of an infill line connected to this T-joint was not processed yet and a piece of the contour could be extruded going forward.
    bool                        could_take_next() const throw() { return !this->consumed && this->contour_not_taken_length_next > SCALED_EPSILON; }

    // Could extrude a complete segment from this to this->prev_on_contour.
    bool                        could_connect_prev() const throw()
        { return ! this->consumed && this->prev_on_contour != this && ! this->prev_on_contour->consumed && ! this->prev_trimmed && ! this->prev_on_contour->next_trimmed; }
    // Could extrude a complete segment from this to this->next_on_contour.
    bool                        could_connect_next() const throw()
        { return ! this->consumed && this->next_on_contour != this && ! this->next_on_contour->consumed && ! this->next_trimmed && ! this->next_on_contour->prev_trimmed; }
};

// Distance from param1 to param2 when going counter-clockwise.
static inline double closed_contour_distance_ccw(double param1, double param2, double contour_length)
{
    assert(param1 >= 0. && param1 <= contour_length);
    assert(param2 >= 0. && param2 <= contour_length);
    double d = param2 - param1;
    if (d < 0.)
        d += contour_length;
    return d;
}

// Distance from param1 to param2 when going clockwise.
static inline double closed_contour_distance_cw(double param1, double param2, double contour_length)
{
    return closed_contour_distance_ccw(param2, param1, contour_length);
}

// Length along the contour from cp1 to cp2 going counter-clockwise.
inline double path_length_along_contour_ccw(const ContourIntersectionPoint *cp1, const ContourIntersectionPoint *cp2, double contour_length)
{
    assert(cp1 != nullptr);
    assert(cp2 != nullptr);
    assert(cp1->contour_idx == cp2->contour_idx);
    assert(cp1 != cp2);
    return closed_contour_distance_ccw(cp1->param, cp2->param, contour_length);
}

// Lengths along the contour from cp1 to cp2 going CCW and going CW.
inline std::pair<double, double> path_lengths_along_contour(const ContourIntersectionPoint *cp1, const ContourIntersectionPoint *cp2, double contour_length)
{
    // Zero'th param is the length of the contour.
    double param_lo  = cp1->param;
    double param_hi  = cp2->param;
    assert(param_lo >= 0. && param_lo <= contour_length);
    assert(param_hi >= 0. && param_hi <= contour_length);
    bool  reversed = false;
    if (param_lo > param_hi) {
        std::swap(param_lo, param_hi);
        reversed = true;
    }
    auto out = std::make_pair(param_hi - param_lo, param_lo + contour_length - param_hi);
    if (reversed)
        std::swap(out.first, out.second);
    return out;
}

// Add contour points from interval (idx_start, idx_end> to polyline.
static inline void take_cw_full(Polyline &pl, const Points &contour, size_t idx_start, size_t idx_end)
{
    assert(!pl.empty() && pl.points.back() == contour[idx_start]);
    size_t i = (idx_start == 0) ? contour.size() - 1 : idx_start - 1;
    while (i != idx_end) {
        pl.points.emplace_back(contour[i]);
        if (i == 0)
            i = contour.size();
        -- i;
    }
    pl.points.emplace_back(contour[i]);
}

// Add contour points from interval (idx_start, idx_end> to polyline, limited by the Eucleidean length taken.
static inline double take_cw_limited(Polyline &pl, const Points &contour, const std::vector<double> &params, size_t idx_start, size_t idx_end, double length_to_take)
{
    // If appending to an infill line, then the start point of a perimeter line shall match the end point of an infill line.
    assert(pl.empty() || pl.points.back() == contour[idx_start]);
    assert(contour.size() + 1 == params.size());
    assert(length_to_take > SCALED_EPSILON);
    // Length of the contour.
    double length = params.back();
    // Parameter (length from contour.front()) for the first point.
    double p0     = params[idx_start];
    // Current (2nd) point of the contour.
    size_t i = (idx_start == 0) ? contour.size() - 1 : idx_start - 1;
    // Previous point of the contour.
    size_t iprev = idx_start;
    // Length of the contour curve taken for iprev.
    double lprev  = 0.;

    for (;;) {
        double l = closed_contour_distance_cw(p0, params[i], length);
        if (l >= length_to_take) {
            // Trim the last segment.
            double t = double(length_to_take - lprev) / (l - lprev);
            pl.points.emplace_back(lerp(contour[iprev], contour[i], t));
            return length_to_take;
        }
        // Continue with the other segments.
        pl.points.emplace_back(contour[i]);
        if (i == idx_end)
            return l;
        iprev = i;
        lprev = l;
        if (i == 0)
            i = contour.size();
        --i;
    }
    assert(false);
    return 0;
}

// Add contour points from interval (idx_start, idx_end> to polyline.
static inline void take_ccw_full(Polyline& pl, const Points& contour, size_t idx_start, size_t idx_end)
{
    assert(!pl.empty() && pl.points.back() == contour[idx_start]);
    size_t i = idx_start;
    if (++i == contour.size())
        i = 0;
    while (i != idx_end) {
        pl.points.emplace_back(contour[i]);
        if (++i == contour.size())
            i = 0;
    }
    pl.points.emplace_back(contour[i]);
}

// Add contour points from interval (idx_start, idx_end> to polyline, limited by the Eucleidean length taken.
// Returns length of the contour taken.
static inline double take_ccw_limited(Polyline &pl, const Points &contour, const std::vector<double> &params, size_t idx_start, size_t idx_end, double length_to_take)
{
    // If appending to an infill line, then the start point of a perimeter line shall match the end point of an infill line.
    assert(pl.empty() || pl.points.back() == contour[idx_start]);
    assert(contour.size() + 1 == params.size());
    assert(length_to_take > SCALED_EPSILON);
    // Length of the contour.
    double length = params.back();
    // Parameter (length from contour.front()) for the first point.
    double p0     = params[idx_start];
    // Current (2nd) point of the contour.
    size_t i = idx_start;
    if (++i == contour.size())
        i = 0;
    // Previous point of the contour.
    size_t iprev = idx_start;
    // Length of the contour curve taken at iprev.
    double lprev  = 0;
    for (;;) {
        double l = closed_contour_distance_ccw(p0, params[i], length);
        if (l >= length_to_take) {
            // Trim the last segment.
            double t = double(length_to_take - lprev) / (l - lprev);
            pl.points.emplace_back(lerp(contour[iprev], contour[i], t));
            return length_to_take;
        }
        // Continue with the other segments.
        pl.points.emplace_back(contour[i]);
        if (i == idx_end)
            return l;
        iprev = i;
        lprev = l;
        if (++i == contour.size())
            i = 0;
    }
    assert(false);
    return 0;
}

// Connect end of pl1 to the start of pl2 using the perimeter contour.
// If clockwise, then a clockwise segment from idx_start to idx_end is taken, otherwise a counter-clockwise segment is being taken.
static void take(Polyline& pl1, const Polyline& pl2, const Points& contour, size_t idx_start, size_t idx_end, bool clockwise)
{
    assert(idx_start != idx_end);
    assert(pl1.size() >= 2);
    assert(pl2.size() >= 2);

    {
        // Reserve memory at pl1 for the connecting contour and pl2.
        int new_points = int(idx_end) - int(idx_start) - 1;
        if (new_points < 0)
            new_points += int(contour.size());
        pl1.points.reserve(pl1.points.size() + size_t(new_points) + pl2.points.size());
    }

    if (clockwise)
        take_cw_full(pl1, contour, idx_start, idx_end);
    else
        take_ccw_full(pl1, contour, idx_start, idx_end);

    pl1.points.insert(pl1.points.end(), pl2.points.begin() + 1, pl2.points.end());
}

static void take(Polyline& pl1, const Polyline& pl2, const Points& contour, ContourIntersectionPoint* cp_start, ContourIntersectionPoint* cp_end, bool clockwise)
{
    assert(cp_start->prev_on_contour != nullptr);
    assert(cp_start->next_on_contour != nullptr);
    assert(cp_end  ->prev_on_contour != nullptr);
    assert(cp_end  ->next_on_contour != nullptr);
    assert(cp_start != cp_end);

    take(pl1, pl2, contour, cp_start->point_idx, cp_end->point_idx, clockwise);

    // Mark the contour segments in between cp_start and cp_end as consumed.
    if (clockwise)
        std::swap(cp_start, cp_end);
    if (cp_start->next_on_contour != cp_end)
        for (auto* cp = cp_start->next_on_contour; cp->next_on_contour != cp_end; cp = cp->next_on_contour) {
            cp->consume_prev();
            cp->consume_next();
        }
    cp_start->consume_next();
    cp_end->consume_prev();
}

static void take_limited(
    Polyline &pl1, const Points &contour, const std::vector<double> &params, 
    ContourIntersectionPoint *cp_start, ContourIntersectionPoint *cp_end, bool clockwise, double take_max_length, double line_half_width)
{
    // This is a valid case, where a single infill line connect to two different contours (outer contour + hole or two holes).
//    assert(cp_start != cp_end);
    assert(cp_start->prev_on_contour != nullptr);
    assert(cp_start->next_on_contour != nullptr);
    assert(cp_end  ->prev_on_contour != nullptr);
    assert(cp_end  ->next_on_contour != nullptr);
    assert(pl1.size() >= 2);
    assert(contour.size() + 1 == params.size());

    if (!(clockwise ? cp_start->could_take_prev() : cp_start->could_take_next()))
        return;

    assert(pl1.points.front() == contour[cp_start->point_idx] || pl1.points.back() == contour[cp_start->point_idx]);
    bool        add_at_start = pl1.points.front() == contour[cp_start->point_idx];
    Points      pl_tmp;
    if (add_at_start) {
        pl_tmp = std::move(pl1.points);
        pl1.points.clear();
    }

    {
        // Reserve memory at pl1 for the perimeter segment.
        // Pessimizing - take the complete segment.
        int new_points = int(cp_end->point_idx) - int(cp_start->point_idx) - 1;
        if (new_points < 0)
            new_points += int(contour.size());
        pl1.points.reserve(pl1.points.size() + pl_tmp.size() + size_t(new_points));
    }

    double length = params.back();
    double length_to_go = take_max_length;
    cp_start->consumed = true;
    if (cp_start == cp_end) {
        length_to_go = std::max(0., std::min(length_to_go, length - line_half_width));
        length_to_go = std::min(length_to_go, clockwise ? cp_start->contour_not_taken_length_prev : cp_start->contour_not_taken_length_next);
        cp_start->consume_prev();
        cp_start->consume_next();
        if (length_to_go > SCALED_EPSILON)
            clockwise ?
                take_cw_limited (pl1, contour, params, cp_start->point_idx, cp_start->point_idx, length_to_go) :
                take_ccw_limited(pl1, contour, params, cp_start->point_idx, cp_start->point_idx, length_to_go);
    } else if (clockwise) {
        // Going clockwise from cp_start to cp_end.
        assert(cp_start != cp_end);
        for (ContourIntersectionPoint* cp = cp_start; cp != cp_end; cp = cp->prev_on_contour) {
            // Length of the segment from cp to cp->prev_on_contour.
            double l = closed_contour_distance_cw(cp->param, cp->prev_on_contour->param, length);
            length_to_go = std::min(length_to_go, cp->contour_not_taken_length_prev);
            //if (cp->prev_on_contour->consumed)
                // Don't overlap with an already extruded infill line.
                length_to_go = std::max(0., std::min(length_to_go, l - line_half_width));
            cp->consume_prev();
            if (l >= length_to_go) {
                if (length_to_go > SCALED_EPSILON) {
                    cp->prev_on_contour->trim_next(l - length_to_go);
                    take_cw_limited(pl1, contour, params, cp->point_idx, cp->prev_on_contour->point_idx, length_to_go);
                }
                break;
            } else {
                cp->prev_on_contour->trim_next(0.);
                take_cw_full(pl1, contour, cp->point_idx, cp->prev_on_contour->point_idx);
                length_to_go -= l;
            }
        }
    } else {
        assert(cp_start != cp_end);
        for (ContourIntersectionPoint* cp = cp_start; cp != cp_end; cp = cp->next_on_contour) {
            double l = closed_contour_distance_ccw(cp->param, cp->next_on_contour->param, length);
            length_to_go = std::min(length_to_go, cp->contour_not_taken_length_next);
            //if (cp->next_on_contour->consumed)
                // Don't overlap with an already extruded infill line.
                length_to_go = std::max(0., std::min(length_to_go, l - line_half_width));
            cp->consume_next();
            if (l >= length_to_go) {
                if (length_to_go > SCALED_EPSILON) {
                    cp->next_on_contour->trim_prev(l - length_to_go);
                    take_ccw_limited(pl1, contour, params, cp->point_idx, cp->next_on_contour->point_idx, length_to_go);
                }
                break;
            } else {
                cp->next_on_contour->trim_prev(0.);
                take_ccw_full(pl1, contour, cp->point_idx, cp->next_on_contour->point_idx);
                length_to_go -= l;
            }
        }
    }

    if (add_at_start) {
        pl1.reverse();
        append(pl1.points, pl_tmp);
    }
}

// Calculate intersection of a line with a thick segment.
// Returns Eucledian parameters of the line / thick segment overlap.
static inline bool line_rounded_thick_segment_collision(
    const Vec2d& line_a, const Vec2d& line_b,
    const Vec2d& segment_a, const Vec2d& segment_b, const double offset,
    std::pair<double, double>& out_interval)
{
    const Vec2d  line_v0   = line_b - line_a;
    double       lv        = line_v0.squaredNorm();

    const Vec2d  segment_v = segment_b - segment_a;
    const double segment_l = segment_v.norm();
    const double offset2   = offset * offset;

    bool intersects = false;
    if (lv < SCALED_EPSILON * SCALED_EPSILON)
    {
        // Very short line vector. Just test whether the center point is inside the offset line.
        Vec2d lpt = 0.5 * (line_a + line_b);
        if (segment_l > SCALED_EPSILON) {
            intersects = line_alg::distance_to_squared(Linef{ segment_a, segment_b }, lpt) < offset2;
        } else
            intersects = (0.5 * (segment_a + segment_b) - lpt).squaredNorm() < offset2;
        if (intersects) {
            out_interval.first = 0.;
            out_interval.second = sqrt(lv);
        }
    }
    else
    {
        // Output interval.
        double tmin = std::numeric_limits<double>::max();
        double tmax = -tmin;
        auto extend_interval = [&tmin, &tmax](double atmin, double atmax) {
            tmin = std::min(tmin, atmin);
            tmax = std::max(tmax, atmax);
        };

        // Intersections with the inflated segment end points.
        auto ray_circle_intersection_interval_extend = [&extend_interval](const Vec2d &segment_pt, const double offset2, const Vec2d &line_pt, const Vec2d &line_vec) {
            std::pair<Vec2d, Vec2d> pts;
            Vec2d  p0 = line_pt - segment_pt;
            double lv2 = line_vec.squaredNorm();
            if (ray_circle_intersections_r2_lv2_c(offset2, line_vec.y(), - line_vec.x(), lv2, - line_vec.y() * p0.x() + line_vec.x() * p0.y(), pts)) {
                double tmin = (pts.first  - p0).dot(line_vec) / lv2;
                double tmax = (pts.second - p0).dot(line_vec) / lv2;
                if (tmin > tmax)
                    std::swap(tmin, tmax);
                tmin = std::max(tmin, 0.);
                tmax = std::min(tmax, 1.);
                if (tmin <= tmax)
                    extend_interval(tmin, tmax);
            }
        };

        // Intersections with the inflated segment.
        if (segment_l > SCALED_EPSILON) {
            ray_circle_intersection_interval_extend(segment_a, offset2, line_a, line_v0);
            ray_circle_intersection_interval_extend(segment_b, offset2, line_a, line_v0);
            // Clip the line segment transformed into a coordinate space of the segment,
            // where the segment spans (0, 0) to (segment_l, 0).
            const Vec2d dir_x = segment_v / segment_l;
            const Vec2d dir_y(- dir_x.y(), dir_x.x());
            const Vec2d line_p0(line_a - segment_a);
            std::pair<double, double> interval;
            if (liang_barsky_line_clipping_interval(
                    Vec2d(line_p0.dot(dir_x), line_p0.dot(dir_y)),
                    Vec2d(line_v0.dot(dir_x), line_v0.dot(dir_y)), 
                    BoundingBoxf(Vec2d(0., - offset), Vec2d(segment_l, offset)), 
                    interval))
                extend_interval(interval.first, interval.second);
        } else
            ray_circle_intersection_interval_extend(0.5 * (segment_a + segment_b), offset, line_a, line_v0);

        intersects = tmin <= tmax;
        if (intersects) {
            lv = sqrt(lv);
            out_interval.first  = tmin * lv;
            out_interval.second = tmax * lv;
        }
    }

    return intersects;
}

// Keep these assertion helpers in all builds: the asserts themselves stay
// active in a debug build.
static inline bool inside_interval(double low, double high, double p)
{
    return p >= low && p <= high;
}

static inline bool interval_inside_interval(double outer_low, double outer_high, double inner_low, double inner_high, double epsilon)
{
    outer_low -= epsilon;
    outer_high += epsilon;
    return inside_interval(outer_low, outer_high, inner_low) && inside_interval(outer_low, outer_high, inner_high);
}

static inline bool cyclic_interval_inside_interval(double outer_low, double outer_high, double inner_low, double inner_high, double length)
{
    if (outer_low > outer_high)
        outer_high += length;
    if (inner_low > inner_high)
        inner_high += length;
    else if (inner_high < outer_low) {
        inner_low += length;
        inner_high += length;
    }
    return interval_inside_interval(outer_low, outer_high, inner_low, inner_high, double(SCALED_EPSILON));
}

inline bool validate_boundary_intersections(const std::vector<std::vector<ContourIntersectionPoint*>> &boundary_intersections)
{
    for (const std::vector<ContourIntersectionPoint*>& contour : boundary_intersections) {
        for (ContourIntersectionPoint* ip : contour) {
            assert(ip->next_trimmed == ip->next_on_contour->prev_trimmed);
            assert(ip->prev_trimmed == ip->prev_on_contour->next_trimmed);
        }
    }
    return true;
}

// Mark the segments of split boundary as consumed if they are very close to some of the infill line.
inline void mark_boundary_segments_touching_infill(
    // Boundary contour, along which the perimeter extrusions will be drawn.
	const std::vector<Points>                              &boundary,
    // Parametrization of boundary with Euclidian length.
	const std::vector<std::vector<double>>                 &boundary_parameters,
    // Intersections (T-joints) of the infill lines with the boundary.
    std::vector<std::vector<ContourIntersectionPoint*>>    &boundary_intersections,
    // Bounding box around the boundary.
	const BoundingBox 		                               &boundary_bbox,
    // Infill lines, either completely inside the boundary, or touching the boundary.
	const Polylines 		                               &infill,
    // How much of the infill ends should be ignored when marking the boundary segments?
	const double			                                clip_distance,
    // Roughly width of the infill line.
	const double 				                            distance_colliding)
{
    assert(boundary.size() == boundary_parameters.size());
    for (size_t i = 0; i < boundary.size(); ++ i)
        assert(boundary[i].size() + 1 == boundary_parameters[i].size());
    assert(validate_boundary_intersections(boundary_intersections));

	EdgeGrid::Grid grid;
    // Make sure that the the grid is big enough for queries against the thick segment.
	grid.set_bbox(boundary_bbox.inflated(distance_colliding * 1.43));
	// Inflate the bounding box by a thick line width.
	grid.create(boundary, coord_t(std::max(clip_distance, distance_colliding)) + scale_i(10.));

    // Visitor for the EdgeGrid to trim boundary_intersections with existing infill lines.
	struct Visitor {
		Visitor(const EdgeGrid::Grid &grid,
                const std::vector<Points> &boundary, const std::vector<std::vector<double>> &boundary_parameters, std::vector<std::vector<ContourIntersectionPoint*>> &boundary_intersections,
                const double radius) :
			grid(grid), boundary(boundary), boundary_parameters(boundary_parameters), boundary_intersections(boundary_intersections), radius(radius), trim_l_threshold(0.5 * radius) {}

        // Init with a segment of an infill line.
		void init(const Vec2d &infill_pt1, const Vec2d &infill_pt2) {
			this->infill_pt1 = &infill_pt1;
			this->infill_pt2 = &infill_pt2;
            this->infill_bbox.reset();
            this->infill_bbox.merge(infill_pt1);
            this->infill_bbox.merge(infill_pt2);
            this->infill_bbox.offset(this->radius + SCALED_EPSILON);
        }

		bool operator()(coord_t iy, coord_t ix) {
			// Called with a row and colum of the grid cell, which is intersected by a line.
			auto cell_data_range = this->grid.cell_data_range(iy, ix);
			for (auto it_contour_and_segment = cell_data_range.first; it_contour_and_segment != cell_data_range.second; ++ it_contour_and_segment) {
				// End points of the line segment and their vector.
				auto segment = this->grid.segment(*it_contour_and_segment);
                std::vector<ContourIntersectionPoint*> &intersections = boundary_intersections[it_contour_and_segment->first];
                if (intersections.empty())
                    // There is no infil line touching this contour, thus effort will be saved to calculate overlap with other infill lines.
                    continue;
				const Vec2d seg_pt1 = to_vec2d(segment.first);
				const Vec2d seg_pt2 = to_vec2d(segment.second);
                std::pair<double, double> interval;
                BoundingBoxf bbox_seg;
                bbox_seg.merge(seg_pt1);
                bbox_seg.merge(seg_pt2);
                if (this->infill_bbox.overlap(bbox_seg) && line_rounded_thick_segment_collision(seg_pt1, seg_pt2, *this->infill_pt1, *this->infill_pt2, this->radius, interval)) {
                    // The boundary segment intersects with the infill segment thickened by radius.
                    // Interval is specified in Euclidian length from seg_pt1 to seg_pt2.
                    // 1) Find the Euclidian parameters of seg_pt1 and seg_pt2 on its boundary contour.
                    const std::vector<double> &contour_parameters = boundary_parameters[it_contour_and_segment->first];
                    const double contour_length = contour_parameters.back();
					const double param_seg_pt1  = contour_parameters[it_contour_and_segment->second];
                    const double param_seg_pt2  = contour_parameters[it_contour_and_segment->second + 1];
                    assert(interval.first >= 0.);
                    assert(interval.second >= 0.);
                    assert(interval.first <= interval.second);
                    const auto param_overlap1 = std::min(param_seg_pt2, param_seg_pt1 + interval.first);
                    const auto param_overlap2 = std::min(param_seg_pt2, param_seg_pt1 + interval.second);
                    // 2) Find the ContourIntersectionPoints before param_overlap1 and after param_overlap2.
                    // Find the span of ContourIntersectionPoints, that is trimmed by the interval (param_overlap1, param_overlap2).
                    ContourIntersectionPoint *ip_low, *ip_high;
                    if (intersections.size() == 1) {
                        // Only a single infill line touches this contour.
                        ip_low = ip_high = intersections.front();
                    } else {
                        assert(intersections.size() > 1);
                        auto it_low  = std::lower_bound(intersections.begin(), intersections.end(), param_overlap1,
                            [](const ContourIntersectionPoint *l, double param) { return l->param < param; });
                        auto it_high = std::lower_bound(intersections.begin(), intersections.end(), param_overlap2,
                            [](const ContourIntersectionPoint *l, double param) { return l->param < param; });
                        ip_low  = it_low  == intersections.end() ? intersections.front() : *it_low;
                        ip_high = it_high == intersections.end() ? intersections.front() : *it_high;
                        if (ip_low->param != param_overlap1)
                            ip_low = ip_low->prev_on_contour;
                        // Verify that the interval (param_overlap1, param_overlap2) is inside the interval (ip_low->param, ip_high->param).
                        assert(cyclic_interval_inside_interval(ip_low->param, ip_high->param, param_overlap1, param_overlap2, contour_length));
                    }
                    assert(validate_boundary_intersections(boundary_intersections));
                    // Mark all ContourIntersectionPoints between ip_low and ip_high as consumed.
                    if (ip_low->next_on_contour != ip_high)
                        for (ContourIntersectionPoint *ip = ip_low->next_on_contour; ip != ip_high; ip = ip->next_on_contour) {
                            ip->consume_prev();
                            ip->consume_next();
                        }
                    // Subtract the interval from the first and last segments.
                    double trim_l = closed_contour_distance_ccw(ip_low->param, param_overlap1, contour_length);
                    //if (trim_l > trim_l_threshold)
                    ip_low->trim_next(trim_l);
                    trim_l = closed_contour_distance_ccw(param_overlap2, ip_high->param, contour_length);
                    //if (trim_l > trim_l_threshold)
                    ip_high->trim_prev(trim_l);
                    assert(ip_low->next_trimmed == ip_high->prev_trimmed);
                    assert(validate_boundary_intersections(boundary_intersections));
                    //FIXME mark point as consumed?
                    //FIXME verify the sequence between prev and next?
				}
			}
			// Continue traversing the grid along the edge.
			return true;
		}

        const EdgeGrid::Grid                                &grid;
        const std::vector<Points>                           &boundary;
        const std::vector<std::vector<double>>              &boundary_parameters;
        std::vector<std::vector<ContourIntersectionPoint*>> &boundary_intersections;
        // Maximum distance between the boundary and the infill line allowed to consider the boundary not touching the infill line.
        const double                                         radius;
        // Region around the contour / infill line intersection point, where the intersections are ignored.
        const double                                         trim_l_threshold;

        const Vec2d* infill_pt1;
        const Vec2d* infill_pt2;
        BoundingBoxf                                         infill_bbox;

    } visitor(grid, boundary, boundary_parameters, boundary_intersections, distance_colliding);

    for (const Polyline& polyline : infill) {
        // Clip the infill polyline by the Eucledian distance along the polyline.
        SegmentPoint start_point = clip_start_segment_and_point(polyline.points, clip_distance);
        SegmentPoint end_point = clip_end_segment_and_point(polyline.points, clip_distance);
        if (start_point.valid() && end_point.valid() &&
            (start_point.idx_segment < end_point.idx_segment || (start_point.idx_segment == end_point.idx_segment && start_point.t < end_point.t))) {
            // The clipped polyline is non-empty.
            for (size_t point_idx = start_point.idx_segment; point_idx <= end_point.idx_segment; ++point_idx) {
                //FIXME extend the EdgeGrid to suport tracing a thick line.
#if 0
                Point pt1, pt2;
                Vec2d pt1d, pt2d;
                if (point_idx == start_point.idx_segment) {
                    pt1d = start_point.point;
                    pt1 = to_point(pt1d);
                } else {
                    pt1 = polyline.points[point_idx];
                    pt1d = to_vec2d(pt1);
                }
                if (point_idx == start_point.idx_segment) {
                    pt2d = end_point.point;
                    pt2 = to_point(pt1d);
                } else {
                    pt2 = polyline.points[point_idx];
                    pt2d = to_vec2d(pt2);
                }
                visitor.init(pt1d, pt2d);
                grid.visit_cells_intersecting_thick_line(pt1, pt2, distance_colliding, visitor);
#else
                Vec2d pt1 = (point_idx == start_point.idx_segment) ? start_point.point : to_vec2d(polyline.points[point_idx]);
                Vec2d pt2 = (point_idx == end_point.idx_segment) ? end_point.point : to_vec2d(polyline.points[point_idx + 1]);
                visitor.init(pt1, pt2);
                // Simulate tracing of a thick line. This only works reliably if distance_colliding <= grid cell size.
                Vec2d v = (pt2 - pt1).normalized() * distance_colliding;
                Vec2d vperp = perp(v);
                Vec2d a = pt1 - v - vperp;
                Vec2d b = pt2 + v - vperp;
                assert(grid.bbox().contains(to_point(a)));
                assert(grid.bbox().contains(to_point(b)));
                grid.visit_cells_intersecting_line(to_point(a), to_point(b), visitor);
                a = pt1 - v + vperp;
                b = pt2 + v + vperp;
                assert(grid.bbox().contains(to_point(a)));
                assert(grid.bbox().contains(to_point(b)));
                grid.visit_cells_intersecting_line(to_point(a), to_point(b), visitor);
#endif
            }
        }
    }

    assert(validate_boundary_intersections(boundary_intersections));
}

inline void connect_infill(storage_handle *storage, Polylines&& infill_ordered, const BoundaryContours &boundary_src, Polylines& polylines_out, const coord_t spacing, const raw_infill_pattern_params& params)
{
    assert(!boundary_src.empty());
    assert(!boundary_src.front().empty());
    BoundaryContours polygons_src;
    polygons_src.reserve(boundary_src.size());
    if (C_INFILL_CONNECTION_OUTER_SHELL == params.connection || C_INFILL_CONNECTION_CONNECTED == params.connection)
        polygons_src.push_back(boundary_src.front());
    if (C_INFILL_CONNECTION_HOLES == params.connection || C_INFILL_CONNECTION_CONNECTED == params.connection)
        for (size_t idx = 1; idx < boundary_src.size(); ++idx)
            polygons_src.push_back(boundary_src[idx]);

    connect_infill(storage, std::move(infill_ordered), polygons_src, get_extents(boundary_src.front()), polylines_out, spacing, params);
}

inline void connect_infill(storage_handle *storage, Polylines&& infill_ordered, const Polygons& boundary_src, const BoundingBox& bbox, Polylines& polylines_out, const coord_t spacing, const raw_infill_pattern_params& params)
{
    const BoundaryContours contours = boundary_contours(boundary_src);
    connect_infill(storage, std::move(infill_ordered), contours, bbox, polylines_out, spacing, params);
}

static constexpr auto boundary_idx_unconnected = std::numeric_limits<size_t>::max();

struct BoundaryInfillGraph
{
    std::vector<Points>                     boundary;
    std::vector<std::vector<double>>        boundary_params;
    std::vector<ContourIntersectionPoint>   map_infill_end_point_to_boundary;

    const Point&    point(const ContourIntersectionPoint &cp) const {
        assert(cp.contour_idx != size_t(-1));
        assert(cp.point_idx != size_t(-1));
        return this->boundary[cp.contour_idx][cp.point_idx];
    }

    const Point&    infill_end_point(size_t infill_end_point_idx) const {
        return this->point(this->map_infill_end_point_to_boundary[infill_end_point_idx]);
    }

    const Point     interpolate_contour_point(const ContourIntersectionPoint &cp, double param) {
        const Points                &contour        = this->boundary[cp.contour_idx];
        const std::vector<double>   &contour_params = this->boundary_params[cp.contour_idx];
        // Find the start of a contour segment with param.
        auto it = std::lower_bound(contour_params.begin(), contour_params.end(), param);
        if (*it != param) {
            assert(it != contour_params.begin());
            -- it;
        }
        size_t i = it - contour_params.begin();
        if (i == contour.size())
            i = 0;
        double t1 = contour_params[i];
        double t2 = next_value_modulo(i, contour_params);
        return lerp(contour[i], next_value_modulo(i, contour), (param - t1) / (t2 - t1));
    }

    enum Direction {
        Left,
        Right,
        Up,
        Down,
        Taken,
    };

    static Direction dir(const Point &p1, const Point &p2) {
        return p1.x() == p2.x() ? 
            (p1.y() < p2.y() ? Up : Down) :
            (p1.x() < p2.x() ? Right : Left);
    }

    const Direction dir_prev(const ContourIntersectionPoint &cp) const {
        assert(cp.prev_on_contour);
        return cp.could_take_prev() ? 
            dir(this->point(cp), this->point(*cp.prev_on_contour)) :
            Taken;
    }

    const Direction dir_next(const ContourIntersectionPoint &cp) const {
        assert(cp.next_on_contour);
        return cp.could_take_next() ? 
            dir(this->point(cp), this->point(*cp.next_on_contour)) :
            Taken;
    }

    bool            first(const ContourIntersectionPoint &cp) const {
        return ((&cp - this->map_infill_end_point_to_boundary.data()) & 1) == 0;
    }

    const ContourIntersectionPoint& other(const ContourIntersectionPoint &cp) const {
        return this->map_infill_end_point_to_boundary[((&cp - this->map_infill_end_point_to_boundary.data()) ^ 1)];
    }

    ContourIntersectionPoint& other(const ContourIntersectionPoint &cp) {
        return this->map_infill_end_point_to_boundary[((&cp - this->map_infill_end_point_to_boundary.data()) ^ 1)];
    }

    bool            prev_vertical(const ContourIntersectionPoint &cp) const {
        return this->point(cp).x() == this->point(*cp.prev_on_contour).x();
    }

    bool            next_vertical(const ContourIntersectionPoint &cp) const {
        return this->point(cp).x() == this->point(*cp.next_on_contour).x();
    }

};


// After mark_boundary_segments_touching_infill() marks boundary segments overlapping trimmed infill lines,
// there are possibly some very short boundary segments unmarked, but overlapping the untrimmed infill lines fully
// Mark those short boundary segments.
static inline void mark_boundary_segments_overlapping_infill(
    BoundaryInfillGraph                                    &graph,
    // Infill lines, either completely inside the boundary, or touching the boundary.
    const Polylines                                        &infill,
    // Spacing (width) of the infill lines.
    const coord_t                                            spacing)
{
    for (ContourIntersectionPoint &cp : graph.map_infill_end_point_to_boundary) {
        const Points                &contour         = graph.boundary[cp.contour_idx];
        const std::vector<double>   &contour_params  = graph.boundary_params[cp.contour_idx];
        const Polyline              &infill_polyline = infill[(&cp - graph.map_infill_end_point_to_boundary.data()) / 2];
        const double                 radius          = 0.5 * (spacing + SCALED_EPSILON);
        assert(infill_polyline.size() == 2);
        const Linef                  infill_line { to_vec2d(infill_polyline.points.front()), to_vec2d(infill_polyline.points.back()) };
        if (cp.could_take_next()) {
            bool inside = true;
            for (size_t i = cp.point_idx; i != cp.next_on_contour->point_idx; ) {
                size_t j = next_idx_modulo(i, contour);
                const Vec2d seg_pt2 = to_vec2d(contour[j]);
                if (line_alg::distance_to_squared(infill_line, seg_pt2) < radius * radius) {
                    // The segment is completely inside.
                } else {
                    std::pair<double, double> interval;
                    line_rounded_thick_segment_collision(to_vec2d(contour[i]), seg_pt2, infill_line.a, infill_line.b, radius, interval);
                    assert(interval.first == 0.);
                    double len_out = closed_contour_distance_ccw(contour_params[cp.point_idx], contour_params[i], contour_params.back()) + interval.second;
                    if (len_out < cp.contour_not_taken_length_next) {
                        // Leaving the infill line region before exiting cp.contour_not_taken_length_next, 
                        // thus at least some of the contour is outside and we will extrude this segment.
                        inside = false;
                        break;
                    }
                }
                if (closed_contour_distance_ccw(contour_params[cp.point_idx], contour_params[j], contour_params.back()) >= cp.contour_not_taken_length_next)
                    break;
                i = j;
            }
            if (inside) {
                if (! cp.next_trimmed)
                    // The arc from cp to cp.next_on_contour was not trimmed yet, however it is completely overlapping the infill line.
                    cp.next_on_contour->trim_prev(0);
                cp.trim_next(0);
            }
        } else
            cp.trim_next(0);
        if (cp.could_take_prev()) {
            bool inside = true;
            for (size_t i = cp.point_idx; i != cp.prev_on_contour->point_idx; ) {
                size_t j = prev_idx_modulo(i, contour);
                const Vec2d seg_pt2 = to_vec2d(contour[j]);
                // Distance of the second segment line from the infill line.
                if (line_alg::distance_to_squared(infill_line, seg_pt2) < radius * radius) {
                    // The segment is completely inside.
                } else {
                    std::pair<double, double> interval;
                    line_rounded_thick_segment_collision(to_vec2d(contour[i]), seg_pt2, infill_line.a, infill_line.b, radius, interval);
                    assert(interval.first == 0.);
                    double len_out = closed_contour_distance_cw(contour_params[cp.point_idx], contour_params[i], contour_params.back()) + interval.second;
                    if (len_out < cp.contour_not_taken_length_prev) {
                        // Leaving the infill line region before exiting cp.contour_not_taken_length_next, 
                        // thus at least some of the contour is outside and we will extrude this segment.
                        inside = false;
                        break;
                    }
                }
                if (closed_contour_distance_cw(contour_params[cp.point_idx], contour_params[j], contour_params.back()) >= cp.contour_not_taken_length_prev)
                    break;
                i = j;
            }
            if (inside) {
                if (! cp.prev_trimmed)
                    // The arc from cp to cp.prev_on_contour was not trimmed yet, however it is completely overlapping the infill line.
                    cp.prev_on_contour->trim_next(0);
                cp.trim_prev(0);
            }
        } else
            cp.trim_prev(0);
    }
}

inline BoundaryInfillGraph create_boundary_infill_graph(storage_handle *storage, const Polylines &infill_ordered, const BoundaryContours &boundary_src, const BoundingBox &bbox, const coord_t spacing)
{
    BoundaryInfillGraph out;
    out.boundary.assign(boundary_src.size(), Points());
    out.boundary_params.assign(boundary_src.size(), std::vector<double>());
    out.map_infill_end_point_to_boundary.assign(infill_ordered.size() * 2, ContourIntersectionPoint{ boundary_idx_unconnected, boundary_idx_unconnected });
    {
        // Project the infill_ordered end points onto boundary_src.
        std::vector<std::pair<EdgeGrid::Grid::ClosestPointResult, size_t>> intersection_points;
        {
            EdgeGrid::Grid grid;
            grid.set_bbox(bbox.inflated(SCALED_EPSILON));
            grid.create(boundary_src, scale_i(10.));
            intersection_points.reserve(infill_ordered.size() * 2);
            for (const Polyline &pl : infill_ordered)
                for (const Point *pt : { &pl.points.front(), &pl.points.back() }) {
                    EdgeGrid::Grid::ClosestPointResult cp = grid.closest_point_signed_distance(*pt, coord_t(SCALED_EPSILON));
                    if (cp.valid()) {
                        // The infill end point shall lie on the contour.
                        assert(cp.distance <= 3.);
                        intersection_points.emplace_back(cp, (&pl - infill_ordered.data()) * 2 + (pt == &pl.points.front() ? 0 : 1));
                    }
                }
            std::sort(intersection_points.begin(), intersection_points.end(), [](const std::pair<EdgeGrid::Grid::ClosestPointResult, size_t> &cp1, const std::pair<EdgeGrid::Grid::ClosestPointResult, size_t> &cp2) {
                return   cp1.first.contour_idx < cp2.first.contour_idx ||
                        (cp1.first.contour_idx == cp2.first.contour_idx &&
                            (cp1.first.start_point_idx < cp2.first.start_point_idx ||
                                (cp1.first.start_point_idx == cp2.first.start_point_idx && cp1.first.t < cp2.first.t)));
            });
        }
        auto it = intersection_points.begin();
        auto it_end = intersection_points.end();
        std::vector<std::vector<ContourIntersectionPoint*>> boundary_intersection_points(out.boundary.size(), std::vector<ContourIntersectionPoint*>());
        for (size_t idx_contour = 0; idx_contour < boundary_src.size(); ++ idx_contour) {
            // Copy contour_src to contour_dst while adding intersection points.
            // Map infill end points map_infill_end_point_to_boundary to the newly inserted boundary points of contour_dst.
            // chain the points of map_infill_end_point_to_boundary along their respective contours.
            const ContourView &contour_src = boundary_src[idx_contour];
            Points        &contour_dst = out.boundary[idx_contour];
            std::vector<ContourIntersectionPoint*> &contour_intersection_points = boundary_intersection_points[idx_contour];
            ContourIntersectionPoint *pfirst = nullptr;
            ContourIntersectionPoint *pprev  = nullptr;
            {
                // Reserve intersection points.
                size_t n_intersection_points = 0;
                for (auto itx = it; itx != it_end && itx->first.contour_idx == idx_contour; ++itx)
                    ++n_intersection_points;
                contour_intersection_points.reserve(n_intersection_points);
            }
            for (size_t idx_point = 0; idx_point < contour_src.size(); ++ idx_point) {
                const c_point ipt = contour_src[idx_point];
                if (contour_dst.empty() || !same_point(contour_dst.back(), ipt))
                    contour_dst.emplace_back(ipt);
                for (; it != it_end && it->first.contour_idx == idx_contour && it->first.start_point_idx == idx_point; ++ it) {
                    // Add these points to the destination contour.
                    const Polyline  &infill_line = infill_ordered[it->second / 2];
                    const Point     &pt          = (it->second & 1) ? infill_line.points.back() : infill_line.points.front();
                    size_t idx_tjoint_pt = 0;
                    if (idx_point + 1 < contour_src.size() || pt != contour_dst.front()) {
                        if (pt != contour_dst.back())
                            contour_dst.emplace_back(pt);
                        idx_tjoint_pt = contour_dst.size() - 1;
                    }
                    out.map_infill_end_point_to_boundary[it->second] = ContourIntersectionPoint{ /* it->second, */ idx_contour, idx_tjoint_pt };
                    ContourIntersectionPoint *pthis = &out.map_infill_end_point_to_boundary[it->second];
                    if (pprev) {
                        pprev->next_on_contour = pthis;
                        pthis->prev_on_contour = pprev;
                    } else
                        pfirst = pthis;
                    contour_intersection_points.emplace_back(pthis);
                    pprev = pthis;
                }
                if (pfirst) {
                    pprev->next_on_contour = pfirst;
                    pfirst->prev_on_contour = pprev;
                }
            }
            // Parametrize the new boundary with the intersection points inserted.
            std::vector<double> &contour_params = out.boundary_params[idx_contour];
            contour_params.assign(contour_dst.size() + 1, 0.);
            for (size_t i = 1; i < contour_dst.size(); ++i) {
                contour_params[i] = contour_params[i - 1] + (to_vec2d(contour_dst[i]) - to_vec2d(contour_dst[i - 1])).norm();
                assert(contour_params[i] > contour_params[i - 1]);
            }
            contour_params.back() = contour_params[contour_params.size() - 2] + (to_vec2d(contour_dst.back()) - to_vec2d(contour_dst.front())).norm();
            assert(contour_params.back() > contour_params[contour_params.size() - 2]);
            // Map parameters from contour_params to boundary_intersection_points.
            for (ContourIntersectionPoint *ip : contour_intersection_points)
                ip->param = contour_params[ip->point_idx];
            // and measure distance to the previous and next intersection point.
            const double contour_length = contour_params.back();
            for (ContourIntersectionPoint *ip : contour_intersection_points) {
                if (ip->next_on_contour == ip) {
                    assert(ip->prev_on_contour == ip);
                    ip->contour_not_taken_length_prev = ip->contour_not_taken_length_next = contour_length;
                } else {
                    assert(ip->prev_on_contour != ip);
                    ip->contour_not_taken_length_prev = closed_contour_distance_ccw(ip->prev_on_contour->param, ip->param, contour_length);
                    ip->contour_not_taken_length_next = closed_contour_distance_ccw(ip->param, ip->next_on_contour->param, contour_length);
                }
            }
        }

        assert(out.boundary.size() == boundary_src.size());
#if 0
        // Adaptive Cubic Infill produces infill lines, which not always end at the outer boundary.
        assert(std::all_of(out.map_infill_end_point_to_boundary.begin(), out.map_infill_end_point_to_boundary.end(),
            [&out.boundary](const ContourIntersectionPoint &contour_point) {
                return contour_point.contour_idx < out.boundary.size() && contour_point.point_idx < out.boundary[contour_point.contour_idx].size();
            }));
#endif

        // Mark the points and segments of split out.boundary as consumed if they are very close to some of the infill line.
        {
            // @supermerill used 2. * (spacing)
            const distf_t clip_distance = 1.7 * (spacing);
            // Allow a bit of overlap. This value must be slightly higher than the overlap of FillAdaptive, otherwise
            // the anchors of the adaptive infill will mask the other side of the perimeter line.
            // (see connect_lines_using_hooks() in FillAdaptive.cpp)
            const distf_t distance_colliding = 0.8 * (spacing);
            mark_boundary_segments_touching_infill(out.boundary, out.boundary_params, boundary_intersection_points, bbox, infill_ordered, clip_distance, distance_colliding);
        }
    }

    return out;
}

inline void connect_infill(storage_handle *storage, Polylines &&infill_ordered, const BoundaryContours &boundary_src, const BoundingBox &bbox, Polylines &polylines_out, const coord_t spacing, const raw_infill_pattern_params &params)
{
	assert(! infill_ordered.empty());
    assert(params.anchor_length     >= 0.);
    assert(params.anchor_length_max >= 0.01f);
    assert(params.anchor_length_max >= params.anchor_length);
    const coordf_t anchor_length     = scale_d(params.anchor_length);
    const coordf_t anchor_length_max = scale_d(params.anchor_length_max);

#if 0
    append(polylines_out, infill_ordered);
    return;
#endif

    BoundaryInfillGraph graph = create_boundary_infill_graph(storage, infill_ordered, boundary_src, bbox, spacing);

    std::vector<size_t> merged_with(infill_ordered.size());
    std::iota(merged_with.begin(), merged_with.end(), 0);

    //mark point as used depends of connection parameter
    //if (params.connection == C_INFILL_CONNECTION_OUTER_SHELL) {
    //    for (auto it = boundary_data.begin() + 1; it != boundary_data.end(); ++it) {
    //        for (ContourPointData& pt : *it) {
    //            pt.point_consumed = true;
    //        }
    //    }
    //} else if (params.connection == C_INFILL_CONNECTION_HOLES) {
    //    for (ContourPointData& pt : boundary_data[0]) {
    //        pt.point_consumed = true;
    //    }
    //}
    //assert(boundary_data.size() == boundary_src.holes.size() + 1);

    auto get_and_update_merged_with = [&merged_with](size_t polyline_idx) -> size_t {
        for (size_t last = polyline_idx;;) {
            size_t lower = merged_with[last];
            assert(lower <= last);
            if (lower == last) {
                merged_with[polyline_idx] = last;
                return last;
            }
            last = lower;
        }
        assert(false);
        return std::numeric_limits<size_t>::max();
    };

    const double line_half_width = 0.5 * (spacing);

#if 0
    // Connection from end of one infill line to the start of another infill line.
    //const double length_max = (spacing);
//  const auto length_max = double(((2. / params.density) * spacing));
    const auto length_max = double(((1000. / params.density) * spacing));
    struct ConnectionCost {
        ConnectionCost(size_t idx_first, double cost, bool reversed) : idx_first(idx_first), cost(cost), reversed(reversed) {}
        size_t  idx_first;
        double  cost;
        bool    reversed;
    };
    std::vector<ConnectionCost> connections_sorted;
    connections_sorted.reserve(infill_ordered.size() * 2 - 2);
    for (size_t idx_chain = 1; idx_chain < infill_ordered.size(); ++ idx_chain) {
        const ContourIntersectionPoint      *cp1            = &graph.map_infill_end_point_to_boundary[(idx_chain - 1) * 2 + 1];
        const ContourIntersectionPoint      *cp2            = &graph.map_infill_end_point_to_boundary[idx_chain * 2];
        if (cp1->contour_idx != boundary_idx_unconnected && cp1->contour_idx == cp2->contour_idx) {
            // End points on the same contour. Try to connect them.
            std::pair<double, double> len = path_lengths_along_contour(cp1, cp2, graph.boundary_params[cp1->contour_idx].back());
            if (len.first < length_max)
                connections_sorted.emplace_back(idx_chain - 1, len.first, false);
            if (len.second < length_max)
                connections_sorted.emplace_back(idx_chain - 1, len.second, true);
        }
    }
    std::sort(connections_sorted.begin(), connections_sorted.end(), [](const ConnectionCost& l, const ConnectionCost& r) { return l.cost < r.cost; });

    for (ConnectionCost &connection_cost : connections_sorted) {
		ContourIntersectionPoint *cp1    = &graph.map_infill_end_point_to_boundary[connection_cost.idx_first * 2 + 1];
		ContourIntersectionPoint *cp2    = &graph.map_infill_end_point_to_boundary[(connection_cost.idx_first + 1) * 2];
        assert(cp1 != cp2);
        assert(cp1->contour_idx == cp2->contour_idx && cp1->contour_idx != boundary_idx_unconnected);
        if (cp1->consumed || cp2->consumed)
            continue;
        const double              length = connection_cost.cost;
        bool                      could_connect;
        {
            // cp1, cp2 sorted CCW.
            ContourIntersectionPoint *cp_low  = connection_cost.reversed ? cp2 : cp1;
            ContourIntersectionPoint *cp_high = connection_cost.reversed ? cp1 : cp2;
            assert(std::abs(length - closed_contour_distance_ccw(cp_low->param, cp_high->param, graph.boundary_params[cp1->contour_idx].back())) < SCALED_EPSILON);
            could_connect = ! cp_low->next_trimmed && ! cp_high->prev_trimmed;
            if (could_connect && cp_low->next_on_contour != cp_high) {
                // Other end of cp1, may or may not be on the same contour as cp1.
                const ContourIntersectionPoint *cp1prev = cp1 - 1;
                // Other end of cp2, may or may not be on the same contour as cp2.
                const ContourIntersectionPoint *cp2next = cp2 + 1;
                for (auto *cp = cp_low->next_on_contour; cp != cp_high; cp = cp->next_on_contour)
                    if (cp->consumed || cp == cp1prev || cp == cp2next || cp->prev_trimmed || cp->next_trimmed) {
                        could_connect = false;
                        break;
                    }
            }
        }
        // Indices of the polylines to be connected by a perimeter segment.
        size_t idx_first  = connection_cost.idx_first;
        size_t idx_second = idx_first + 1;
        idx_first = get_and_update_merged_with(idx_first);
        assert(idx_first < idx_second);
        assert(idx_second == merged_with[idx_second]);
        if (could_connect && length < anchor_length_max) {
            // Take the complete contour.
            // Connect the two polygons using the boundary contour.
            take(infill_ordered[idx_first], infill_ordered[idx_second], graph.boundary[cp1->contour_idx], cp1, cp2, connection_cost.reversed);
            // Mark the second polygon as merged with the first one.
            merged_with[idx_second] = merged_with[idx_first];
            infill_ordered[idx_second].points.clear();
        } else {
            // Try to connect cp1 resp. cp2 with a piece of perimeter line.
            take_limited(infill_ordered[idx_first],  graph.boundary[cp1->contour_idx], graph.boundary_params[cp1->contour_idx], cp1, cp2, connection_cost.reversed, anchor_length, line_half_width);
            take_limited(infill_ordered[idx_second], graph.boundary[cp1->contour_idx], graph.boundary_params[cp1->contour_idx], cp2, cp1, ! connection_cost.reversed, anchor_length, line_half_width);
        }
	}
#endif

    struct Arc {
        ContourIntersectionPoint    *intersection;
        double                       arc_length;
    };
    std::vector<Arc> arches;
    arches.reserve(graph.map_infill_end_point_to_boundary.size());
    for (ContourIntersectionPoint &cp : graph.map_infill_end_point_to_boundary)
        if (cp.contour_idx != boundary_idx_unconnected && cp.next_on_contour != &cp && cp.could_connect_next())
            arches.push_back({ &cp, path_length_along_contour_ccw(&cp, cp.next_on_contour, graph.boundary_params[cp.contour_idx].back()) });
    std::sort(arches.begin(), arches.end(), [](const auto &l, const auto &r) { return l.arc_length < r.arc_length; });

    //FIXME improve the Traveling Salesman problem with 2-opt and 3-opt local optimization.
    for (Arc &arc : arches)
        if (! arc.intersection->consumed && ! arc.intersection->next_on_contour->consumed) {
            // Indices of the polylines to be connected by a perimeter segment.
            ContourIntersectionPoint *cp1            = arc.intersection;
            ContourIntersectionPoint *cp2            = arc.intersection->next_on_contour;
            size_t                    polyline_idx1  = get_and_update_merged_with(((cp1 - graph.map_infill_end_point_to_boundary.data()) / 2));
            size_t                    polyline_idx2  = get_and_update_merged_with(((cp2 - graph.map_infill_end_point_to_boundary.data()) / 2));
            const Points             &contour        = graph.boundary[cp1->contour_idx];
            const std::vector<double> &contour_params = graph.boundary_params[cp1->contour_idx];
            if (polyline_idx1 != polyline_idx2) {
                Polyline &polyline1 = infill_ordered[polyline_idx1];
                Polyline &polyline2 = infill_ordered[polyline_idx2];
                if (arc.arc_length < anchor_length_max) {
                    // Not closing a loop, connecting the lines.
                    assert(contour[cp1->point_idx] == polyline1.points.front() || contour[cp1->point_idx] == polyline1.points.back());
                    if (contour[cp1->point_idx] == polyline1.points.front())
                        polyline1.reverse();
                    assert(contour[cp2->point_idx] == polyline2.points.front() || contour[cp2->point_idx] == polyline2.points.back());
                    if (contour[cp2->point_idx] == polyline2.points.back())
                        polyline2.reverse();
                    take(polyline1, polyline2, contour, cp1, cp2, false);
                    // Mark the second polygon as merged with the first one.
                    if (polyline_idx2 < polyline_idx1) {
                        polyline2 = std::move(polyline1);
                        polyline1.points.clear();
                        merged_with[polyline_idx1] = merged_with[polyline_idx2];
                    } else {
                        polyline2.points.clear();
                        merged_with[polyline_idx2] = merged_with[polyline_idx1];
                    }
                } else if (anchor_length > SCALED_EPSILON) {
                    // Move along the perimeter, but don't take the whole arc.
                    take_limited(polyline1, contour, contour_params, cp1, cp2, false, anchor_length, line_half_width);
                    take_limited(polyline2, contour, contour_params, cp2, cp1, true,  anchor_length, line_half_width);
                }
            }
        }

    // Connect the remaining open infill lines to the perimeter lines if possible.
    for (ContourIntersectionPoint &contour_point : graph.map_infill_end_point_to_boundary)
        if (! contour_point.consumed && contour_point.contour_idx != boundary_idx_unconnected) {
            const Points              &contour        = graph.boundary[contour_point.contour_idx];
            const std::vector<double> &contour_params = graph.boundary_params[contour_point.contour_idx];

            double    lprev         = contour_point.could_connect_prev() ?
                path_length_along_contour_ccw(contour_point.prev_on_contour, &contour_point, contour_params.back()) :
                std::numeric_limits<double>::max();
            double    lnext         = contour_point.could_connect_next() ?
                path_length_along_contour_ccw(&contour_point, contour_point.next_on_contour, contour_params.back()) :
                std::numeric_limits<double>::max();
            size_t    polyline_idx  = get_and_update_merged_with(((&contour_point - graph.map_infill_end_point_to_boundary.data()) / 2));
            Polyline &polyline      = infill_ordered[polyline_idx];
            assert(! polyline.empty());
            assert(contour[contour_point.point_idx] == polyline.points.front() || contour[contour_point.point_idx] == polyline.points.back());
            bool connected = false;
            for (double l : { std::min(lprev, lnext), std::max(lprev, lnext) }) {
                if (l == std::numeric_limits<double>::max() || l > anchor_length_max)
                    break;
                // Take the complete contour.
                bool      reversed      = l == lprev;
                ContourIntersectionPoint *cp2 = reversed ? contour_point.prev_on_contour : contour_point.next_on_contour;
                // Identify which end of the polyline touches the boundary.
                size_t    polyline_idx2 = get_and_update_merged_with(((cp2 - graph.map_infill_end_point_to_boundary.data()) / 2));
                if (polyline_idx == polyline_idx2)
                    // Try the other side.
                    continue;
                // Not closing a loop.
                if (contour[contour_point.point_idx] == polyline.points.front())
                    polyline.reverse();
                Polyline &polyline2 = infill_ordered[polyline_idx2];
                assert(! polyline.empty());
                assert(contour[cp2->point_idx] == polyline2.points.front() || contour[cp2->point_idx] == polyline2.points.back());
                if (contour[cp2->point_idx] == polyline2.points.back())
                    polyline2.reverse();
                take(polyline, polyline2, contour, &contour_point, cp2, reversed);
                if (polyline_idx < polyline_idx2) {
                    // Mark the second polyline as merged with the first one.
                    merged_with[polyline_idx2] = polyline_idx;
                    polyline2.points.clear();
                } else {
                    // Mark the first polyline as merged with the second one.
                    merged_with[polyline_idx] = polyline_idx2;
                    polyline2 = std::move(polyline);
                    polyline.points.clear();
                }
                connected = true;
                break;
            }
            if (! connected && anchor_length > SCALED_EPSILON) {
                // Which to take? One could optimize for:
                // 1) Shortest path
                // 2) Hook length
                // ...
                // Let's take the longer now, as this improves the chance of another hook to be placed on the other side of this contour point.
                double l = std::max(contour_point.contour_not_taken_length_prev, contour_point.contour_not_taken_length_next);
                if (l > SCALED_EPSILON) {
                    if (contour_point.contour_not_taken_length_prev > contour_point.contour_not_taken_length_next)
                        take_limited(polyline, contour, contour_params, &contour_point, contour_point.prev_on_contour, true, anchor_length, line_half_width);
                    else
                        take_limited(polyline, contour, contour_params, &contour_point, contour_point.next_on_contour, false, anchor_length, line_half_width);
                }
            }
        }

    polylines_out.reserve(polylines_out.size() + std::count_if(infill_ordered.begin(), infill_ordered.end(), [](const Polyline &pl) { return ! pl.empty(); }));
	for (Polyline &pl : infill_ordered)
		if (! pl.empty())
			polylines_out.emplace_back(std::move(pl));
}

// Extend the infill lines along the perimeters, this is mainly useful for grid aligned support, where a perimeter line may be nearly
// aligned with the infill lines.
static inline void base_support_extend_infill_lines(Polylines &infill, BoundaryInfillGraph &graph, const coord_t line_spacing, const raw_infill_pattern_params &params)
{
/*
    // Backup the source lines.
    Lines lines;
    lines.reserve(linfill.size());
    std::transform(infill.begin(), infill.end(), std::back_inserter(lines), [](const Polyline &pl) { assert(pl.size() == 2); return Line(pl.points.begin(), pl.points.end()); });
*/
    // Maximum deviation perpendicular to the infill line to allow merging as a continuation of the same infill line.
    const auto      dist_max_x      = coord_t(line_spacing * 0.33);
    // Minimum length of the arc away from the infill end point to allow merging as a continuation of the same infill line.
    const auto      dist_min_y      = coord_t(line_spacing * 0.5);

    for (ContourIntersectionPoint &cp : graph.map_infill_end_point_to_boundary) {
        const Points                &contour         = graph.boundary[cp.contour_idx];
        const std::vector<double>   &contour_param   = graph.boundary_params[cp.contour_idx];
        const Point                 &pt              = contour[cp.point_idx];
        const bool                   first           = graph.first(cp);
        int                          extend_next_idx = -1;
        int                          extend_prev_idx = -1;
        coord_t                      dist_y_prev;
        coord_t                      dist_y_next;
        double                       arc_len_prev;
        double                       arc_len_next;

        if (! graph.next_vertical(cp)){
            size_t i = cp.point_idx;
            size_t j = next_idx_modulo(i, contour);
            while (j != cp.next_on_contour->point_idx) {
                //const Point &p1 = contour[i];
                const Point &p2 = contour[j];
                if (std::abs(p2.x() - pt.x()) > dist_max_x)
                    break;
                i = j;
                j = next_idx_modulo(j, contour);
            }
            if (i != cp.point_idx) {
                const Point &p2 = contour[i];
                coord_t      dist_y = p2.y() - pt.y();
                if (first)
                    dist_y = - dist_y;
                if (dist_y > dist_min_y) {
                    arc_len_next    = closed_contour_distance_ccw(contour_param[cp.point_idx], contour_param[i], contour_param.back());
                    if (arc_len_next < cp.contour_not_taken_length_next) {
                        extend_next_idx = i;
                        dist_y_next     = dist_y;
                    }
                }
            }
        }

        if (! graph.prev_vertical(cp)) {
            size_t i = cp.point_idx;
            size_t j = prev_idx_modulo(i, contour);
            while (j != cp.prev_on_contour->point_idx) {
                //const Point &p1 = contour[i];
                const Point &p2 = contour[j];
                if (std::abs(p2.x() - pt.x()) > dist_max_x)
                    break;
                i = j;
                j = prev_idx_modulo(j, contour);
            }
            if (i != cp.point_idx) {
                const Point &p2 = contour[i];
                coord_t      dist_y = p2.y() - pt.y();
                if (first)
                    dist_y = - dist_y;
                if (dist_y > dist_min_y) {
                    arc_len_prev = closed_contour_distance_ccw(contour_param[i], contour_param[cp.point_idx], contour_param.back());
                    if (arc_len_prev < cp.contour_not_taken_length_prev) {
                        extend_prev_idx = i;
                        dist_y_prev     = dist_y;
                    }
                }
            }
        }

        if (extend_prev_idx >= 0 && extend_next_idx >= 0)
            // Which side to move the point?
            dist_y_prev < dist_y_next ? extend_prev_idx : extend_next_idx = -1;

        assert(cp.prev_trimmed == cp.prev_on_contour->next_trimmed);
        assert(cp.next_trimmed == cp.next_on_contour->prev_trimmed);
        Polyline &infill_line = infill[(&cp - graph.map_infill_end_point_to_boundary.data()) / 2];
        if (extend_prev_idx >= 0) {
            if (first)
                infill_line.reverse();
            take_cw_full(infill_line, contour, cp.point_idx, extend_prev_idx);
            if (first)
                infill_line.reverse();
            cp.point_idx = extend_prev_idx;
            if (cp.prev_trimmed)
                cp.contour_not_taken_length_prev -= arc_len_prev;
            else
                cp.contour_not_taken_length_prev = cp.prev_on_contour->contour_not_taken_length_next =
                closed_contour_distance_ccw(contour_param[cp.prev_on_contour->point_idx], contour_param[cp.point_idx], contour_param.back());
            cp.trim_next(0);
            cp.next_on_contour->prev_trimmed = true;
        } else if (extend_next_idx >= 0) {
            if (first)
                infill_line.reverse();
            take_ccw_full(infill_line, contour, cp.point_idx, extend_next_idx);
            if (first)
                infill_line.reverse();
            cp.point_idx = extend_next_idx;
            cp.trim_prev(0);
            cp.prev_on_contour->next_trimmed = true;
            if (cp.next_trimmed)
                cp.contour_not_taken_length_next -= arc_len_next;
            else
                cp.contour_not_taken_length_next = cp.next_on_contour->contour_not_taken_length_prev =
                closed_contour_distance_ccw(contour_param[cp.point_idx], contour_param[cp.next_on_contour->point_idx], contour_param.back());
        }
    }
}

// Called by connect_base_support() as part of the sparse support infill generator.
// Emit contour loops tracing the contour from tbegin to tend inside a band of (left, right).
// The contour is supposed to enter the "forbidden" zone outside of the (left, right) band at tbegin and also at tend.
static inline void emit_loops_in_band(
    // Vertical band, which will trim the contour between tbegin and tend.
    coord_t                      left, 
    coord_t                      right,
    // Contour and its parametrization.
    const Points                &contour,
    const std::vector<double>   &contour_params,
    // Span of the parameters of an arch to trim with the vertical band.
    double                       tbegin,
    double                       tend, 
    // Minimum arch length to put into polylines_out. Shorter arches are not necessary to support a dense support infill.
    double                       min_length,
    Polylines                   &polylines_out)
{
    assert(left < right);
    assert(contour.size() + 1 == contour_params.size());
    assert(contour.size() >= 3);
#ifndef NDEBUG
    const double contour_length = contour_params.back();
    assert(tbegin >= 0 && tbegin < contour_length);
    assert(tend   >= 0 && tend   < contour_length);
    assert(min_length > 0);
#endif

    // Find iterators of the range of segments, where the first and last segment contains tbegin and tend.
    size_t ibegin, iend;
    {
        auto it_begin = std::lower_bound(contour_params.begin(), contour_params.end(), tbegin);
        auto it_end   = std::lower_bound(contour_params.begin(), contour_params.end(), tend);
        assert(it_begin != contour_params.end());
        assert(it_end   != contour_params.end());
        if (*it_begin != tbegin) {
            assert(it_begin != contour_params.begin());
            -- it_begin;
        }
        ibegin = it_begin - contour_params.begin();
        iend   = it_end   - contour_params.begin();
    }

    if (ibegin == contour.size())
        ibegin = 0;
    if (iend == contour.size())
        iend = 0;
    assert(ibegin != iend);

    // Trim the start and end segment to calculate start and end points.
    Point pbegin, pend;
    {
        double t1 = contour_params[ibegin];
        double t2 = next_value_modulo(ibegin, contour_params);
        pbegin = lerp(contour[ibegin], next_value_modulo(ibegin, contour), (tbegin - t1) / (t2 - t1));
        t1 = contour_params[iend];
        t2 = prev_value_modulo(iend, contour_params);
        pend = lerp(contour[iend], prev_value_modulo(iend, contour), (tend - t1) / (t2 - t1));
    }

    // Trace the contour from ibegin to iend.
    enum Side {
        Left,
        Right,
        Mid,
        Unknown
    };

    enum InOutBand {
        Entering, 
        Leaving,
    };

    class State {
    public:
        State(coord_t left, coord_t right, double min_length, Polylines &polylines_out) : 
            m_left(left), m_right(right), m_min_length(min_length), m_polylines_out(polylines_out) {}

        inline void add_inner_point(const Point* p)
        {
            m_polyline.points.emplace_back(*p);
        }

        inline void add_outer_point(const Point* p)
        {
            if (m_polyline_end > 0)
                m_polyline.points.emplace_back(*p);
        }

        inline void add_interpolated_point(const Point* p1, const Point* p2, Side side, InOutBand inout)
        {
            assert(side == Left || side == Right);

            coord_t x = side == Left ? m_left : m_right;
            coord_t y = p1->y() + coord_t(double(x - p1->x()) * double(p2->y() - p1->y()) / double(p2->x() - p1->x()));

            if (inout == Leaving) {
                assert(m_polyline_end == 0);
                m_polyline_end = m_polyline.size();
                m_polyline.points.emplace_back(x, y);
            } else {
                assert(inout == Entering);
                if (m_polyline_end > 0) {
                    if ((this->side1 == Left) == (y - m_polyline.points[m_polyline_end].y() < 0)) {
                        // Emit the vertical segment. Remove the point, where the source contour was split the last time at m_left / m_right.
                        m_polyline.points.erase(m_polyline.points.begin() + m_polyline_end);
                    } else {
                        // Don't emit the vertical segment, split the contour.
                        this->finalize();
                        m_polyline.points.emplace_back(x, y);
                    }
                    m_polyline_end = 0;
                } else
                    m_polyline.points.emplace_back(x, y);
            }
        };

        inline void finalize()
        {
            m_polyline.points.erase(m_polyline.points.begin() + m_polyline_end, m_polyline.points.end());
            if (! m_polyline.empty()) {
                if (! m_polylines_out.empty() && ((m_polylines_out.back().points.back() - m_polyline.points.front())).squaredNorm() < SCALED_EPSILON)
                    m_polylines_out.back().points.insert(m_polylines_out.back().points.end(), m_polyline.points.begin() + 1, m_polyline.points.end());
                else if (m_polyline.length() > m_min_length)
                    m_polylines_out.emplace_back(std::move(m_polyline));
                m_polyline.clear();
            }
        };

    private:
        coord_t      m_left;
        coord_t      m_right;
        double       m_min_length;
        Polylines   &m_polylines_out;

        Polyline     m_polyline;
        size_t       m_polyline_end { 0 };
        Polyline     m_overlapping;

    public:
        Side         side1 { Unknown };
        Side         side2 { Unknown };
    };

    State state { left, right, min_length, polylines_out };

    const Point *p1 = &pbegin;
    auto side = [left, right](const Point* p) {
        coord_t x = p->x();
        return x < left ? Left : x > right ? Right : Mid;
    };
    state.side1 = side(p1);
    if (state.side1 == Mid)
        state.add_inner_point(p1);

    for (size_t i = ibegin; i != iend; ) {
        size_t inext = i + 1;
        if (inext == contour.size())
            inext = 0;
        const Point *p2 = inext == iend ? &pend : &contour[inext];
        state.side2 = side(p2);
        if (state.side1 == Mid) {
            if (state.side2 == Mid) {
                // Inside the band.
                state.add_inner_point(p2);
            } else {
                // From intisde the band to the outside of the band.
                state.add_interpolated_point(p1, p2, state.side2, Leaving);
                state.add_outer_point(p2);
            }
        } else if (state.side2 == Mid) {
            // From outside the band into the band.
            state.add_interpolated_point(p1, p2, state.side1, Entering);
            state.add_inner_point(p2);
        } else if (state.side1 != state.side2) {
            // Both points outside the band.
            state.add_interpolated_point(p1, p2, state.side1, Entering);
            state.add_interpolated_point(p1, p2, state.side2, Leaving);
        } else {
            // Complete segment is outside.
            assert((state.side1 == Left && state.side2 == Left) || (state.side1 == Right && state.side2 == Right));
            state.add_outer_point(p2);
        }
        state.side1 = state.side2;
        p1 = p2;
        i  = inext;
    }
    state.finalize();
}

// To classify perimeter segments connecting infill lines, whether they are required for structural stability of the supports.
struct SupportArcCost
{
    // Connecting one end of an infill line to the other end of the same infill line.
    bool    self_loop { false };
    // Some of the arc touches some infill line.
    bool    open { false };
    // How needed is this arch for support structural stability.
    // Zero means don't take. The higher number, the more likely it is to take the arc.
    double  cost { 0 };
};

static double evaluate_support_arch_cost(const Polyline &pl)
{
    Point front = pl.points.front();
    Point back  = pl.points.back();

    coord_t ymin = front.y();
    coord_t ymax = back.y();
    if (ymin > ymax)
        std::swap(ymin, ymax);

    distf_t dmax = 0;
    // Maximum distance in Y axis out of the (ymin, ymax) band and from the (front, back) line.
    Linef line { to_vec2d(front), to_vec2d(back) };
    for (const Point &pt : pl.points) {
        distf_t dist = std::sqrt(line_alg::distance_to_squared(line, Vec2d(to_vec2d(pt))));
        dmax = std::max<distf_t>(std::max(dmax, dist), distf_t(std::max(pt.y() - ymax, ymin - pt.y())));
    }
    return dmax;
}

// Costs for prev / next arch of each infill line end point.
static inline std::vector<SupportArcCost> evaluate_support_arches(Polylines &infill, BoundaryInfillGraph &graph, const raw_infill_pattern_params &params)
{
    std::vector<SupportArcCost> arches(graph.map_infill_end_point_to_boundary.size() * 2);

    Polyline pl;
    for (ContourIntersectionPoint &cp : graph.map_infill_end_point_to_boundary) {
        // Not a losed loop, such loops should already be consumed.
        assert(cp.next_on_contour != &cp);
        const size_t                    infill_line_idx = &cp - graph.map_infill_end_point_to_boundary.data();
        const bool                      first           = (infill_line_idx & 1) == 0;
        const ContourIntersectionPoint *other_end       = first ? &cp + 1 : &cp - 1;

        SupportArcCost &out_prev = arches[infill_line_idx * 2];
        SupportArcCost &out_next = *(&out_prev + 1);
        out_prev.self_loop = cp.prev_on_contour == other_end;
        out_prev.open      = cp.prev_trimmed;
        out_next.self_loop = cp.next_on_contour == other_end;
        out_next.open      = cp.next_trimmed;

        if (cp.contour_not_taken_length_next > SCALED_EPSILON) {
            pl.clear();
            pl.points.emplace_back(graph.point(cp));
            if (cp.next_trimmed)
                take_ccw_limited(pl, graph.boundary[cp.contour_idx], graph.boundary_params[cp.contour_idx], cp.point_idx, cp.next_on_contour->point_idx, cp.contour_not_taken_length_next);
            else
                take_ccw_full(pl, graph.boundary[cp.contour_idx], cp.point_idx, cp.next_on_contour->point_idx);
            out_next.cost = evaluate_support_arch_cost(pl);
        }

        if (cp.contour_not_taken_length_prev > SCALED_EPSILON) {
            pl.clear();
            pl.points.emplace_back(graph.point(cp));
            if (cp.prev_trimmed)
                take_cw_limited(pl, graph.boundary[cp.contour_idx], graph.boundary_params[cp.contour_idx], cp.point_idx, cp.prev_on_contour->point_idx, cp.contour_not_taken_length_prev);
            else
                take_cw_full(pl, graph.boundary[cp.contour_idx], cp.point_idx, cp.prev_on_contour->point_idx);
            out_prev.cost = evaluate_support_arch_cost(pl);
        }
    }

    return arches;
}

// Connect sparse support infill lines with the boundary while preserving the
// legacy vertical, arch, zig-zag and cap selection order.
inline void connect_base_support(storage_handle *storage,
                                 Polylines &&infill_ordered,
                                 const BoundaryContours &boundary,
                                 const BoundingBox &bbox,
                                 Polylines &polylines_out,
                                 coord_t line_spacing,
                                 const raw_infill_pattern_params &params)
{
    assert(params.anchor_length >= 0.);
    assert(params.anchor_length_max >= 0.01f);
    assert(params.anchor_length_max >= params.anchor_length);

    const coord_t spacing = coord_t(line_spacing * params.density);
    BoundaryInfillGraph graph = create_boundary_infill_graph(storage, infill_ordered, boundary, bbox, spacing);

    const double line_half_width = 0.5 * spacing;
    const double min_arch_length = 1.3 * line_spacing;
    const double trim_length = line_half_width * 0.3;

    mark_boundary_segments_overlapping_infill(graph, infill_ordered, spacing);

    {
        std::vector<size_t> num_boundary_contour_infill_points(graph.boundary.size(), 0);
        for (ContourIntersectionPoint &cp : graph.map_infill_end_point_to_boundary)
            ++num_boundary_contour_infill_points[cp.contour_idx];
        for (size_t idx = 0; idx < num_boundary_contour_infill_points.size(); ++idx)
            if (num_boundary_contour_infill_points[idx] == 0 &&
                graph.boundary_params[idx].back() > trim_length + 0.5 * line_spacing) {
                Polyline polyline;
                polyline.points = graph.boundary[idx];
                polyline.points.emplace_back(polyline.points.front());
                clip_polyline_end(polyline, trim_length);
                if (polyline.size() > 1)
                    polylines_out.emplace_back(std::move(polyline));
            }
    }

    for (ContourIntersectionPoint &cp : graph.map_infill_end_point_to_boundary) {
        if (cp.next_on_contour && cp.next_trimmed && cp.next_on_contour->prev_trimmed) {
            const bool first = graph.first(cp);
            coord_t left = graph.point(cp).x();
            coord_t right = left;
            if (first) {
                left += line_half_width;
                right += line_spacing - line_half_width;
            } else {
                left -= line_spacing - line_half_width;
                right -= line_half_width;
            }
            double param_start = cp.param + cp.contour_not_taken_length_next;
            double param_end = cp.next_on_contour->param - cp.next_on_contour->contour_not_taken_length_prev;
            const double contour_length = graph.boundary_params[cp.contour_idx].back();
            if (param_start >= contour_length)
                param_start -= contour_length;
            if (param_end < 0)
                param_end += contour_length;
            assert(cyclic_interval_inside_interval(cp.param, cp.next_on_contour->param,
                                                   param_start, param_end, contour_length));
            emit_loops_in_band(left, right, graph.boundary[cp.contour_idx],
                               graph.boundary_params[cp.contour_idx], param_start, param_end,
                               0.5 * line_spacing, polylines_out);
        }
    }

    base_support_extend_infill_lines(infill_ordered, graph, line_spacing, params);

    std::vector<size_t> merged_with(infill_ordered.size());
    std::iota(merged_with.begin(), merged_with.end(), 0);
    const auto get_and_update_merged_with = [&graph, &merged_with](const ContourIntersectionPoint *cp) -> size_t {
        size_t polyline_idx = (cp - graph.map_infill_end_point_to_boundary.data()) / 2;
        for (size_t last = polyline_idx;;) {
            const size_t lower = merged_with[last];
            assert(lower <= last);
            if (lower == last) {
                merged_with[polyline_idx] = last;
                return last;
            }
            last = lower;
        }
        assert(false);
        return std::numeric_limits<size_t>::max();
    };

    const auto vertical = [](BoundaryInfillGraph::Direction direction) {
        return direction == BoundaryInfillGraph::Up || direction == BoundaryInfillGraph::Down;
    };
    const auto take_vertical_prev = [](const ContourIntersectionPoint &cp) {
        return cp.prev_trimmed == cp.next_trimmed ?
            cp.contour_not_taken_length_prev > cp.contour_not_taken_length_next :
            !cp.prev_trimmed && cp.next_trimmed;
    };

    const auto take_next = [&graph, &infill_ordered, &merged_with, get_and_update_merged_with,
                            line_half_width, trim_length](ContourIntersectionPoint &cp, bool take_first) {
        ContourIntersectionPoint *cp1 = &cp;
        ContourIntersectionPoint *cp2 = cp.next_on_contour;
        assert(cp1->next_trimmed == cp2->prev_trimmed);
        if (take_first ? cp1->consumed : cp2->consumed)
            return;

        const size_t polyline_idx1 = get_and_update_merged_with(cp1);
        const size_t polyline_idx2 = get_and_update_merged_with(cp2);
        Polyline &polyline1 = infill_ordered[polyline_idx1];
        Polyline &polyline2 = infill_ordered[polyline_idx2];
        const Points &contour = graph.boundary[cp1->contour_idx];
        const std::vector<double> &contour_params = graph.boundary_params[cp1->contour_idx];
        assert(cp1->consumed || contour[cp1->point_idx] == polyline1.points.front() ||
               contour[cp1->point_idx] == polyline1.points.back());
        assert(cp2->consumed || contour[cp2->point_idx] == polyline2.points.front() ||
               contour[cp2->point_idx] == polyline2.points.back());
        bool trimmed = take_first ? cp1->next_trimmed : cp2->prev_trimmed;
        if (!trimmed) {
            trimmed = cp1 == cp2 || polyline_idx1 == polyline_idx2 ||
                      (take_first ? cp2->consumed : cp1->consumed);
            if (!trimmed) {
                const bool cp1_first = ((cp1 - graph.map_infill_end_point_to_boundary.data()) & 1) == 0;
                const ContourIntersectionPoint *cp1_other = cp1_first ? cp1 + 1 : cp1 - 1;
                trimmed = cp2 == cp1_other;
            }
            if (trimmed) {
                const double length = cp1 == cp2 ? contour_params.back() :
                    path_length_along_contour_ccw(cp1, cp2, contour_params.back());
                if (take_first) {
                    cp1->trim_next(std::max(0., length - trim_length - SCALED_EPSILON));
                    cp2->trim_prev(0);
                } else {
                    cp1->trim_next(0);
                    cp2->trim_prev(std::max(0., length - trim_length - SCALED_EPSILON));
                }
            }
        }
        if (trimmed) {
            if (take_first)
                take_limited(polyline1, contour, contour_params, cp1, cp2, false, 1e10, line_half_width);
            else
                take_limited(polyline2, contour, contour_params, cp2, cp1, true, 1e10, line_half_width);
        } else if (!cp1->consumed && !cp2->consumed) {
            if (contour[cp1->point_idx] == polyline1.points.front())
                polyline1.reverse();
            if (contour[cp2->point_idx] == polyline2.points.back())
                polyline2.reverse();
            take(polyline1, polyline2, contour, cp1, cp2, false);
            if (polyline_idx2 < polyline_idx1) {
                polyline2 = std::move(polyline1);
                polyline1.points.clear();
                merged_with[polyline_idx1] = merged_with[polyline_idx2];
            } else {
                polyline2.points.clear();
                merged_with[polyline_idx2] = merged_with[polyline_idx1];
            }
        }
    };

    for (ContourIntersectionPoint &cp : graph.map_infill_end_point_to_boundary) {
        assert(cp.contour_idx != boundary_idx_unconnected);
        if (cp.consumed)
            continue;
        const ContourIntersectionPoint &cp_other = graph.other(cp);
        assert((cp.next_on_contour == &cp_other) == (cp_other.prev_on_contour == &cp));
        assert((cp.prev_on_contour == &cp_other) == (cp_other.next_on_contour == &cp));
        const BoundaryInfillGraph::Direction dir_prev = graph.dir_prev(cp);
        const BoundaryInfillGraph::Direction dir_next = graph.dir_next(cp);
        assert((cp.next_on_contour == &cp) == (cp.prev_on_contour == &cp));
        const bool can_take_prev = vertical(dir_prev) && !cp.prev_on_contour->consumed && cp.prev_on_contour != &cp_other;
        const bool can_take_next = vertical(dir_next) && !cp.next_on_contour->consumed && cp.next_on_contour != &cp_other;
        if (can_take_prev && (!can_take_next || take_vertical_prev(cp))) {
            if (!cp.prev_trimmed || cp.contour_not_taken_length_prev > min_arch_length)
                take_next(*cp.prev_on_contour, false);
        } else if (can_take_next) {
            if (!cp.next_trimmed || cp.contour_not_taken_length_next > min_arch_length)
                take_next(cp, true);
        }
    }

    const std::vector<SupportArcCost> arches = evaluate_support_arches(infill_ordered, graph, params);
    const double cost_low = line_spacing * 1.3;
    const double cost_high = line_spacing * 2.;
    const double cost_veryhigh = line_spacing * 3.;

    {
        std::vector<const SupportArcCost *> selected;
        selected.reserve(graph.map_infill_end_point_to_boundary.size());
        for (ContourIntersectionPoint &cp : graph.map_infill_end_point_to_boundary) {
            if (cp.consumed)
                continue;
            const SupportArcCost &cost_prev = arches[(&cp - graph.map_infill_end_point_to_boundary.data()) * 2];
            const SupportArcCost &cost_next = *(&cost_prev + 1);
            double cost_min = cost_prev.cost;
            double cost_max = cost_next.cost;
            if (cost_min > cost_max)
                std::swap(cost_min, cost_max);
            if (cost_max < cost_low || cost_min > cost_high)
                continue;
            const double cost_diff_relative = (cost_max - cost_min) / cost_max;
            if (cost_diff_relative < 0.25)
                continue;
            if (cost_prev.cost > cost_low)
                selected.emplace_back(&cost_prev);
            if (cost_next.cost > cost_low)
                selected.emplace_back(&cost_next);
        }
        std::sort(selected.begin(), selected.end(), [](const SupportArcCost *left, const SupportArcCost *right) {
            return left->cost > right->cost;
        });
        for (const SupportArcCost *arc : selected) {
            ContourIntersectionPoint &cp = graph.map_infill_end_point_to_boundary[(arc - arches.data()) / 2];
            if (!cp.consumed) {
                const bool previous = ((arc - arches.data()) & 1) == 0;
                if (previous)
                    take_next(*cp.prev_on_contour, false);
                else
                    take_next(cp, true);
            }
        }
    }

    for (ContourIntersectionPoint &cp : graph.map_infill_end_point_to_boundary) {
        assert(cp.contour_idx != boundary_idx_unconnected);
        if (cp.consumed)
            continue;
        const bool first = graph.first(cp);
        if (first) {
            if (get_and_update_merged_with(&cp) != get_and_update_merged_with(cp.next_on_contour))
                take_next(cp, true);
        } else if (get_and_update_merged_with(&cp) != get_and_update_merged_with(cp.prev_on_contour)) {
            take_next(*cp.prev_on_contour, false);
        }
    }

    for (ContourIntersectionPoint &cp : graph.map_infill_end_point_to_boundary) {
        const bool first = graph.first(cp);
        const ContourIntersectionPoint *other_end = first ? &cp + 1 : &cp - 1;
        const bool loop_next = cp.next_on_contour == other_end;
        const bool loop_prev = other_end->next_on_contour == &cp;
        const SupportArcCost &cost_prev = arches[(&cp - graph.map_infill_end_point_to_boundary.data()) * 2];
        const SupportArcCost &cost_next = *(&cost_prev + 1);
        assert(cost_prev.self_loop == loop_prev);
        assert(cost_next.self_loop == loop_next);
        if (loop_prev && cp.could_take_prev())
            take_next(*cp.prev_on_contour, false);
        if (loop_next && cp.could_take_next())
            take_next(cp, true);
    }

    {
        std::vector<const SupportArcCost *> candidates;
        for (ContourIntersectionPoint &cp : graph.map_infill_end_point_to_boundary) {
            if (cp.could_take_prev())
                candidates.emplace_back(&arches[(&cp - graph.map_infill_end_point_to_boundary.data()) * 2]);
            if (cp.could_take_next())
                candidates.emplace_back(&arches[(&cp - graph.map_infill_end_point_to_boundary.data()) * 2 + 1]);
        }
        std::sort(candidates.begin(), candidates.end(), [](const SupportArcCost *left, const SupportArcCost *right) {
            return left->cost > right->cost;
        });
        for (const SupportArcCost *candidate : candidates) {
            ContourIntersectionPoint &cp = graph.map_infill_end_point_to_boundary[(candidate - arches.data()) / 2];
            const bool previous = ((candidate - arches.data()) & 1) == 0;
            if (previous) {
                if (cp.could_take_prev() &&
                    (get_and_update_merged_with(&cp) != get_and_update_merged_with(cp.prev_on_contour) ||
                     candidate->cost > cost_high))
                    take_next(*cp.prev_on_contour, false);
            } else {
                if (cp.could_take_next() &&
                    (get_and_update_merged_with(&cp) != get_and_update_merged_with(cp.next_on_contour) ||
                     candidate->cost > cost_high))
                    take_next(cp, true);
            }
        }
    }

    const double cap_cost = 0.5 * line_spacing;
    for (ContourIntersectionPoint &cp : graph.map_infill_end_point_to_boundary) {
        const SupportArcCost &cost_prev = arches[(&cp - graph.map_infill_end_point_to_boundary.data()) * 2];
        const SupportArcCost &cost_next = *(&cost_prev + 1);
        if (cp.contour_not_taken_length_prev > SCALED_EPSILON &&
            (cost_prev.self_loop ? cost_prev.cost > cap_cost : cost_prev.cost > cost_veryhigh)) {
            assert(cp.consumed && (cp.prev_on_contour->consumed || cp.prev_trimmed));
            Polyline polyline { graph.point(cp) };
            if (!cp.prev_trimmed) {
                cp.trim_prev(cp.contour_not_taken_length_prev - line_half_width);
                cp.prev_on_contour->trim_next(0);
            }
            if (cp.contour_not_taken_length_prev > SCALED_EPSILON) {
                take_cw_limited(polyline, graph.boundary[cp.contour_idx], graph.boundary_params[cp.contour_idx],
                                cp.point_idx, cp.prev_on_contour->point_idx, cp.contour_not_taken_length_prev);
                cp.trim_prev(0);
                clip_polyline_start(polyline, line_half_width);
                polylines_out.emplace_back(std::move(polyline));
            }
        }
        if (cp.contour_not_taken_length_next > SCALED_EPSILON &&
            (cost_next.self_loop ? cost_next.cost > cap_cost : cost_next.cost > cost_veryhigh)) {
            assert(cp.consumed && (cp.next_on_contour->consumed || cp.next_trimmed));
            Polyline polyline { graph.point(cp) };
            if (!cp.next_trimmed) {
                cp.trim_next(cp.contour_not_taken_length_next - line_half_width);
                cp.next_on_contour->trim_prev(0);
            }
            if (cp.contour_not_taken_length_next > SCALED_EPSILON) {
                take_ccw_limited(polyline, graph.boundary[cp.contour_idx], graph.boundary_params[cp.contour_idx],
                                 cp.point_idx, cp.next_on_contour->point_idx, cp.contour_not_taken_length_next);
                cp.trim_next(0);
                clip_polyline_start(polyline, line_half_width);
                polylines_out.emplace_back(std::move(polyline));
            }
        }
    }

    polylines_out.reserve(polylines_out.size() + std::count_if(infill_ordered.begin(), infill_ordered.end(),
                                                                [](const Polyline &polyline) { return !polyline.empty(); }));
    for (Polyline &polyline : infill_ordered)
        if (!polyline.empty())
            polylines_out.emplace_back(std::move(polyline));
}
} // end namespace FakePerimeterConnect

// Value-geometry adapters retained for parity tests against the core
// Fill::connect_infill() implementations. Production calls use BoundaryContours
// directly and enter through the plugin API overloads below.
inline work::ExPolygon normalized_boundary(const slic3r_api::ExPolygon &boundary)
{
    work::ExPolygon value_boundary = clipper::from_expolygon_view(boundary);
    clipper::remove_sticks(value_boundary);
    return value_boundary;
}

inline void connect_infill(storage_handle *storage, Polylines &&infill_ordered, const BoundaryContours &boundary,
                           Polylines &polylines_out, const coord_t spacing, const raw_infill_pattern_params &params)
{
    if (params.anchor_length_max == 0) {
        PrusaSimpleConnect::connect_infill(std::move(infill_ordered), boundary, polylines_out, spacing, params);
    } else {
        FakePerimeterConnect::connect_infill(storage, std::move(infill_ordered), boundary, polylines_out, spacing, params);
    }
}

inline void connect_infill(storage_handle *storage, Polylines &&infill_ordered, const BoundaryContours &boundary,
                           const Polygons &polygons_src, Polylines &polylines_out, const coord_t spacing,
                           const raw_infill_pattern_params &params)
{
    if (params.anchor_length_max == 0) {
        PrusaSimpleConnect::connect_infill(std::move(infill_ordered), boundary, polylines_out, spacing, params);
    } else {
        FakePerimeterConnect::connect_infill(storage, std::move(infill_ordered), polygons_src,
                                             get_extents(boundary.front()), polylines_out, spacing, params);
    }
}

inline void connect_base_support(storage_handle *storage,
                                 Polylines &&infill_ordered,
                                 const BoundaryContours &boundary,
                                 const BoundingBox &bbox,
                                 Polylines &polylines_out,
                                 coord_t line_spacing,
                                 const raw_infill_pattern_params &params)
{
    FakePerimeterConnect::connect_base_support(storage, std::move(infill_ordered), boundary, bbox,
                                               polylines_out, line_spacing, params);
}

inline void connect_base_support(storage_handle *storage,
                                 Polylines &&infill_ordered,
                                 const Polygons &boundary,
                                 const BoundingBox &bbox,
                                 Polylines &polylines_out,
                                 coord_t line_spacing,
                                 const raw_infill_pattern_params &params)
{
    const BoundaryContours contours = boundary_contours(boundary);
    connect_base_support(storage, std::move(infill_ordered), contours, bbox, polylines_out, line_spacing, params);
}

// These value-geometry overloads are retained for core-versus-plugin parity
// tests; production callers pass BoundaryContours directly.
inline void connect_infill(storage_handle *storage, Polylines &&infill_ordered, const ExPolygon &boundary,
                           Polylines &polylines_out, const coord_t spacing, const raw_infill_pattern_params &params)
{
    const BoundaryContours contours = boundary_contours(boundary);
    connect_infill(storage, std::move(infill_ordered), contours, polylines_out, spacing, params);
}

inline void connect_infill(storage_handle *storage, Polylines &&infill_ordered, const ExPolygon &boundary,
                           const Polygons &polygons_src, Polylines &polylines_out, const coord_t spacing,
                           const raw_infill_pattern_params &params)
{
    const BoundaryContours contours = boundary_contours(boundary);
    connect_infill(storage, std::move(infill_ordered), contours, polygons_src, polylines_out, spacing, params);
}

inline void connect_infill(storage_handle *storage, Polylines &&infill_ordered, const slic3r_api::ExPolygon &boundary,
                           Polylines &polylines_out, const coord_t spacing, const raw_infill_pattern_params &params)
{
    const BoundaryViewStorage boundary_views(boundary);
    connect_infill(storage, std::move(infill_ordered), boundary_views.contours(), polylines_out, spacing, params);
}

inline void connect_infill(storage_handle *storage, Polylines &&infill_ordered, const slic3r_api::ExPolygon &boundary,
                           const Polygons &polygons_src, Polylines &polylines_out, const coord_t spacing,
                           const raw_infill_pattern_params &params)
{
    const BoundaryViewStorage boundary_views(boundary);
    connect_infill(storage, std::move(infill_ordered), boundary_views.contours(), polygons_src,
                   polylines_out, spacing, params);
}

} // namespace connect
}}} // namespace slic3r_api::Infill::RectilinearPatternsPlugin

#endif // plugins_infill_rectilinear_connect_hpp_
