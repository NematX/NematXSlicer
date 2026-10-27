///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ Copyright (c) Prusa Research 2016 - 2023 Vojtech Bubnik @bubnikv, Lukas Hejl @hejllukas, Lukas Matena @lukasmatena
///|/ Copyright (c) 2017 Eyal Soha
///|/ Copyright (c) Slic3r 2011 - 2015 Alessandro Ranellucci @alranel
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "RectilinearFills.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>


/*
Rectilinear fill algorithms for the plugin
===========================================

This file provides the shared raster and fill implementations using private
mutable work buffers. API `ExPolygon` views and raw infill recipes are adapted
at the pattern boundary; generated paths are published through the plugin API.
Sawtooth adds its Z-profile in RectilinearSawtooth.cpp, while
rectiwithperimeter adds its perimeter wrapper in RectilinearWithPerimeter.cpp.
*/

namespace slic3r_api { namespace GridVarSpeedCopy { namespace RectilinearPatternsPlugin {
namespace fills {

using namespace work;

// Having a segment of a closed polygon, calculate its Euclidian length.
// The segment indices seg1 and seg2 signify an end point of an edge in the forward direction of the loop,
// therefore the point p1 lies on poly.points[seg1-1], poly.points[seg1] etc.
static inline coordf_t segment_length(const Polygon &poly, size_t seg1, const Point &p1, size_t seg2, const Point &p2)
{
#ifdef SLIC3R_DEBUG
    // Verify that p1 lies on seg1. This is difficult to verify precisely,
    // but at least verify, that p1 lies in the bounding box of seg1.
    for (size_t i = 0; i < 2; ++ i) {
        size_t seg = (i == 0) ? seg1 : seg2;
        Point  px  = (i == 0) ? p1   : p2;
        Point  pa  = poly.points[((seg == 0) ? poly.points.size() : seg) - 1];
        Point  pb  = poly.points[seg];
        if (pa(0) > pb(0))
            std::swap(pa(0), pb(0));
        if (pa(1) > pb(1))
            std::swap(pa(1), pb(1));
        assert(px(0) >= pa(0) && px(0) <= pb(0));
        assert(px(1) >= pa(1) && px(1) <= pb(1));
    }
#endif /* SLIC3R_DEBUG */
    const Point *pPrev = &p1;
    const Point *pThis = NULL;
    coordf_t len = 0;
    if (seg1 <= seg2) {
        for (size_t i = seg1; i < seg2; ++ i, pPrev = pThis)
           len += to_vec2d((*pPrev - *(pThis = &poly.points[i]))).norm();
    } else {
        for (size_t i = seg1; i < poly.points.size(); ++ i, pPrev = pThis)
           len += to_vec2d((*pPrev - *(pThis = &poly.points[i]))).norm();
        for (size_t i = 0; i < seg2; ++ i, pPrev = pThis)
           len += to_vec2d((*pPrev - *(pThis = &poly.points[i]))).norm();
    }
    len += to_vec2d((*pPrev - p2)).norm();
    return len;
}

// Append a segment of a closed polygon to a polyline.
// The segment indices seg1 and seg2 signify an end point of an edge in the forward direction of the loop.
// Only insert intermediate points between seg1 and seg2.
static inline void polygon_segment_append(Points &out, const Polygon &polygon, size_t seg1, size_t seg2)
{
    if (seg1 == seg2) {
        // Nothing to append from this segment.
    } else if (seg1 < seg2) {
        // Do not append a point pointed to by seg2.
        out.insert(out.end(), polygon.points.begin() + seg1, polygon.points.begin() + seg2);
    } else {
        out.reserve(out.size() + seg2 + polygon.points.size() - seg1);
        out.insert(out.end(), polygon.points.begin() + seg1, polygon.points.end());
        // Do not append a point pointed to by seg2.
        out.insert(out.end(), polygon.points.begin(), polygon.points.begin() + seg2);
    }
}

// Append a segment of a closed polygon to a polyline.
// The segment indices seg1 and seg2 signify an end point of an edge in the forward direction of the loop,
// but this time the segment is traversed backward.
// Only insert intermediate points between seg1 and seg2.
static inline void polygon_segment_append_reversed(Points &out, const Polygon &polygon, size_t seg1, size_t seg2)
{
    if (seg1 >= seg2) {
        out.reserve(seg1 - seg2);
        for (size_t i = seg1; i > seg2; -- i)
            out.push_back(polygon.points[i - 1]);
    } else {
        // it could be, that seg1 == seg2. In that case, append the complete loop.
        out.reserve(out.size() + seg2 + polygon.points.size() - seg1);
        for (size_t i = seg1; i > 0; -- i)
            out.push_back(polygon.points[i - 1]);
        for (size_t i = polygon.points.size(); i > seg2; -- i)
            out.push_back(polygon.points[i - 1]);
    }
}

// Intersection point of a vertical line with a polygon segment.
struct SegmentIntersection
{
    // Index of a contour in ExPolygonWithOffset, with which this vertical line intersects.
    size_t      iContour{ 0 };
    // Index of a segment in iContour, with which this vertical line intersects.
    size_t      iSegment{ 0 };
    // y position of the intersection, rational number.
    int64_t     pos_p{ 0 };
    uint32_t    pos_q{ 1 };

    bool        is_hole{ false };

    coord_t     pos() const {
        // Division rounds both positive and negative down to zero.
        // Add half of q for an arithmetic rounding effect.
        int64_t p = pos_p;
        if (p < 0)
            p -= int64_t(pos_q>>1);
        else
            p += int64_t(pos_q>>1);
        return coord_t(p / int64_t(pos_q));
    }

    // Left vertical line / contour intersection point.
    // null if next_on_contour_vertical.
    int32_t	prev_on_contour{ 0 };
    // Right vertical line / contour intersection point.
    // If next_on_contour_vertical, then then next_on_contour contains next contour point on the same vertical line.
    int32_t	next_on_contour{ 0 };

    // Kind of intersection. With the original contour, or with the inner offestted contour?
    // A vertical segment will be at least intersected by OUTER_LOW, OUTER_HIGH,
    // but it could be intersected with OUTER_LOW, INNER_LOW, INNER_HIGH, OUTER_HIGH,
    // and there may be more than one pair of INNER_LOW, INNER_HIGH between OUTER_LOW, OUTER_HIGH.
    enum SegmentIntersectionType : char {
        UNKNOWN,
        OUTER_LOW,
        OUTER_HIGH,
        INNER_LOW,
        INNER_HIGH,
    };
    SegmentIntersectionType type{ UNKNOWN };

    enum class LinkType : uint8_t {
        // Horizontal link (left or right).
        Horizontal,
        // Vertical link, up.
        Up,
        // Vertical link, down.
        Down,
        // Phony intersection point has no link.
        Phony,
    };

    enum class LinkQuality : uint8_t {
        Invalid,
        Valid,
        // Valid link, but too long to be followed.
        TooLong,
    };

    // Kept grouped with other booleans for smaller memory footprint.
    LinkType 		prev_on_contour_type{ LinkType::Horizontal };
    LinkType 		next_on_contour_type{ LinkType::Horizontal };
    LinkQuality 	prev_on_contour_quality{ LinkQuality::Valid };
    LinkQuality 	next_on_contour_quality{ LinkQuality::Valid };

    // Was this segment along the y axis consumed?
    // Up means up along the vertical segment.
    bool consumed_vertical_up{ false };
    // Was a segment of the inner perimeter contour consumed?
    // Right means right from the vertical segment.
    bool consumed_perimeter_right{ false };

    // For the INNER_LOW type, this point may be connected to another INNER_LOW point following a perimeter contour.
    // For the INNER_HIGH type, this point may be connected to another INNER_HIGH point following a perimeter contour.
    // If INNER_LOW is connected to INNER_HIGH or vice versa,
    // one has to make sure the vertical infill line does not overlap with the connecting perimeter line.
    bool is_inner() const { return type == INNER_LOW  || type == INNER_HIGH; }
    bool is_outer() const { return type == OUTER_LOW  || type == OUTER_HIGH; }
    bool is_low  () const { return type == INNER_LOW  || type == OUTER_LOW; }
    bool is_high () const { return type == INNER_HIGH || type == OUTER_HIGH; }

    enum class Side {
        Left,
        Right
    };
    enum class Direction {
        Up,
        Down
    };

    bool 	has_left_horizontal()    		 	const { return this->prev_on_contour_type == LinkType::Horizontal; }
    bool 	has_right_horizontal()   		 	const { return this->next_on_contour_type == LinkType::Horizontal; }
    bool 	has_horizontal(Side side)		 	const { return side == Side::Left ? this->has_left_horizontal() : this->has_right_horizontal(); }

    bool 	has_left_vertical_up()   		 	const { return this->prev_on_contour_type == LinkType::Up; }
    bool 	has_left_vertical_down() 		 	const { return this->prev_on_contour_type == LinkType::Down; }
    bool 	has_left_vertical(Direction dir) 	const { return dir == Direction::Up ? this->has_left_vertical_up() : this->has_left_vertical_down(); }
    bool 	has_left_vertical()    	 		 	const { return this->has_left_vertical_up() || this->has_left_vertical_down(); }
    bool 	has_left_vertical_outside()			const { return this->is_low() ? this->has_left_vertical_down() : this->has_left_vertical_up(); }

    bool 	has_right_vertical_up()   			const { return this->next_on_contour_type == LinkType::Up; }
    bool 	has_right_vertical_down() 			const { return this->next_on_contour_type == LinkType::Down; }
    bool 	has_right_vertical(Direction dir) 	const { return dir == Direction::Up ? this->has_right_vertical_up() : this->has_right_vertical_down(); }
    bool 	has_right_vertical()    			const { return this->has_right_vertical_up() || this->has_right_vertical_down(); }
    bool 	has_right_vertical_outside()		const { return this->is_low() ? this->has_right_vertical_down() : this->has_right_vertical_up(); }

    bool 	has_vertical()						const { return this->has_left_vertical() || this->has_right_vertical(); }
    bool 	has_vertical(Side side)				const { return side == Side::Left ? this->has_left_vertical() : this->has_right_vertical(); }
    bool 	has_vertical_up()					const { return this->has_left_vertical_up() || this->has_right_vertical_up(); }
    bool 	has_vertical_down()					const { return this->has_left_vertical_down() || this->has_right_vertical_down(); }
    bool 	has_vertical(Direction dir)			const { return dir == Direction::Up ? this->has_vertical_up() : this->has_vertical_down(); }

    int 	left_horizontal()  					const { return this->has_left_horizontal() ? this->prev_on_contour : -1; }
    int 	right_horizontal()  				const { return this->has_right_horizontal() ? this->next_on_contour : -1; }
    int 	horizontal(Side side)  				const { return side == Side::Left ? this->left_horizontal() : this->right_horizontal(); }
    LinkQuality horizontal_quality(Side side)	const {
        assert(this->has_horizontal(side));
        return side == Side::Left ? this->prev_on_contour_quality : this->next_on_contour_quality;
    }

    int 	left_vertical_up()   		 		const { return this->has_left_vertical_up() ? this->prev_on_contour : -1; }
    int 	left_vertical_down()   		 		const { return this->has_left_vertical_down() ? this->prev_on_contour : -1; }
    int 	left_vertical(Direction dir) 		const { return (dir == Direction::Up ? this->has_left_vertical_up() : this->has_left_vertical_down()) ? this->prev_on_contour : -1; }
    int 	left_vertical()   			 		const { return this->has_left_vertical() ? this->prev_on_contour : -1; }
    int 	left_vertical_outside()				const { return this->is_low() ? this->left_vertical_down() : this->left_vertical_up(); }
    int 	right_vertical_up()   		 		const { return this->has_right_vertical_up() ? this->next_on_contour : -1; }
    int 	right_vertical_down()   	 		const { return this->has_right_vertical_down() ? this->next_on_contour : -1; }
    int 	right_vertical(Direction dir) 		const { return (dir == Direction::Up ? this->has_right_vertical_up() : this->has_right_vertical_down()) ? this->next_on_contour : -1; }
    int 	right_vertical()   			 		const { return this->has_right_vertical() ? this->next_on_contour : -1; }
    int 	right_vertical_outside()			const { return this->is_low() ? this->right_vertical_down() : this->right_vertical_up(); }

    int 	vertical_up(Side side)				const { return side == Side::Left ? this->left_vertical_up() : this->right_vertical_up(); }
    int 	vertical_down(Side side)			const { return side == Side::Left ? this->left_vertical_down() : this->right_vertical_down(); }
    int 	vertical_outside(Side side)			const { return side == Side::Left ? this->left_vertical_outside() : this->right_vertical_outside(); }
    // Returns -1 if there is no link up.
    int 	vertical_up()						const {
        return this->has_left_vertical_up() ? this->left_vertical_up() : this->right_vertical_up();
    }
    LinkQuality vertical_up_quality()			const {
        return this->has_left_vertical_up() ? this->prev_on_contour_quality : this->next_on_contour_quality;
    }
    // Returns -1 if there is no link down.
    int 	vertical_down()						const {
        //    	assert(! this->has_left_vertical_down() || ! this->has_right_vertical_down());
        return this->has_left_vertical_down() ? this->left_vertical_down() : this->right_vertical_down();
    }
    LinkQuality vertical_down_quality()			const {
        return this->has_left_vertical_down() ? this->prev_on_contour_quality : this->next_on_contour_quality;
    }
    int 	vertical_outside()					const { return this->is_low() ? this->vertical_down() : this->vertical_up(); }
    LinkQuality vertical_outside_quality()		const { return this->is_low() ? this->vertical_down_quality() : this->vertical_up_quality(); }

    // Compare two y intersection points given by rational numbers.
    // Note that the rational number is given as pos_p/pos_q, where pos_p is int64 and pos_q is uint32.
    // This function calculates pos_p * other.pos_q < other.pos_p * pos_q as a 48bit number.
    // We don't use 128bit intrinsic data types as these are usually not supported by 32bit compilers and
    // we don't need the full 128bit precision anyway.
    bool operator<(const SegmentIntersection &other) const
    {
        assert(pos_q > 0);
        assert(other.pos_q > 0);
        if (pos_p == 0 || other.pos_p == 0) {
            // Because the denominators are positive and one of the nominators is zero,
            // following simple statement holds.
            return pos_p < other.pos_p;
        } else {
            // None of the nominators is zero.
            int sign1 = (pos_p > 0) ? 1 : -1;
            int sign2 = (other.pos_p > 0) ? 1 : -1;
            int signs = sign1 * sign2;
            assert(signs == 1 || signs == -1);
            if (signs < 0) {
                // The nominators have different signs.
                return sign1 < 0;
            } else {
                // The nominators have the same sign.
                // Absolute values
                uint64_t p1, p2;
                if (sign1 > 0) {
                    p1 = uint64_t(pos_p);
                    p2 = uint64_t(other.pos_p);
                } else {
                    p1 = uint64_t(- pos_p);
                    p2 = uint64_t(- other.pos_p);
                };
                // Multiply low and high 32bit words of p1 by other_pos.q
                // 32bit x 32bit => 64bit
                // l_hi and l_lo overlap by 32 bits.
                uint64_t l_hi = (p1 >> 32) * uint64_t(other.pos_q);
                uint64_t l_lo = (p1 & 0xffffffffll) * uint64_t(other.pos_q);
                l_hi += (l_lo >> 32);
                uint64_t r_hi = (p2 >> 32) * uint64_t(pos_q);
                uint64_t r_lo = (p2 & 0xffffffffll) * uint64_t(pos_q);
                r_hi += (r_lo >> 32);
                // Compare the high 64 bits.
                if (l_hi == r_hi) {
                    // Compare the low 32 bits.
                    l_lo &= 0xffffffffll;
                    r_lo &= 0xffffffffll;
                    return (sign1 < 0) ? (l_lo > r_lo) : (l_lo < r_lo);
                }
                return (sign1 < 0) ? (l_hi > r_hi) : (l_hi < r_hi);
            }
        }
    }

    bool operator==(const SegmentIntersection &other) const
    {
        assert(pos_q > 0);
        assert(other.pos_q > 0);
        if (pos_p == 0 || other.pos_p == 0) {
            // Because the denominators are positive and one of the nominators is zero,
            // following simple statement holds.
            return pos_p == other.pos_p;
        }

        // None of the nominators is zero, none of the denominators is zero.
        bool positive = pos_p > 0;
        if (positive != (other.pos_p > 0))
            return false;
        // The nominators have the same sign.
        // Absolute values
        uint64_t p1 = positive ? uint64_t(pos_p) : uint64_t(- pos_p);
        uint64_t p2 = positive ? uint64_t(other.pos_p) : uint64_t(- other.pos_p);
        // Multiply low and high 32bit words of p1 by other_pos.q
        // 32bit x 32bit => 64bit
        // l_hi and l_lo overlap by 32 bits.
        uint64_t l_lo = (p1 & 0xffffffffll) * uint64_t(other.pos_q);
        uint64_t r_lo = (p2 & 0xffffffffll) * uint64_t(pos_q);
        if (l_lo != r_lo)
            return false;
        uint64_t l_hi = (p1 >> 32) * uint64_t(other.pos_q);
        uint64_t r_hi = (p2 >> 32) * uint64_t(pos_q);
        return l_hi + (l_lo >> 32) == r_hi + (r_lo >> 32);
    }
};
static_assert(sizeof(SegmentIntersection::pos_q) == 4, "SegmentIntersection::pos_q has to be 32bit long!");

// A vertical line with intersection points with polygons.
struct SegmentedIntersectionLine
{
    // Index of this vertical intersection line.
    size_t                              idx;
    // x position of this vertical intersection line.
    coord_t                             pos;
    // List of intersection points with polygons, sorted increasingly by the y axis.
    std::vector<SegmentIntersection>    intersections;
};


// A container maintaining an expolygon with its inner offsetted polygon.
// The purpose of the inner offsetted polygon is to provide segments to connect the infill lines.
struct ExPolygonWithOffset
{
public:
    ExPolygonWithOffset(
        storage_handle *storage,
        const slic3r_api::ExPolygon &expolygon,
        float   angle,
        coord_t aoffset1,
        // If the 2nd offset is zero, then it is ignored and only OUTER_LOW / OUTER_HIGH intersections are
        // populated into vertical intersection lines.
        coord_t aoffset2 = 0)
    {
        // Flatten the API view into mutable working polygons before rotating it.
        // The first polygon is the outer contour; the remaining polygons are holes.
        Polygon converted_contour;
        converted_contour.points = from_polygon_view(expolygon.contour());
        polygons_src.reserve(expolygon.hole_size() + 1);
        polygons_src.push_back(std::move(converted_contour));
        for (const slic3r_api::Polygon hole : expolygon.holes()) {
            Polygon converted_hole;
            converted_hole.points = from_polygon_view(hole);
            polygons_src.push_back(std::move(converted_hole));
        }
        bool source_unchanged = angle == 0.f;
        if (angle != 0.f) {
            for (Polygon &polygon : polygons_src)
                polygon.rotate(angle);
        }

        double miterLimit = DefaultMiterLimit;
        // for the infill pattern, don't cut the corners.
        // default miterLimt = 3
        //double miterLimit = 10.;
        // FIXME: Resolve properly the cases when it is constructed with aoffset1 = 0 and aoffset2 = 0,
        //        that is used in sample_grid_pattern() for Lightning infill.
        // assert(aoffset1 < 0);
        assert(aoffset2 <= 0);
        // assert(aoffset2 == 0 || aoffset2 < aoffset1);
//        bool sticks_removed =
        // Match ExPolygon.cpp:1174: the contour and the holes are cleaned
        // unconditionally. The "nothing changed" test feeding the fast path
        // below is taken from the point counts, which only ever shrink.
        const size_t contour_size_before = polygons_src.front().points.size();
        remove_sticks(polygons_src.front());
        source_unchanged = source_unchanged &&
                           polygons_src.front().points.size() == contour_size_before;
        size_t write = 1;
        for (size_t read = 1; read < polygons_src.size(); ++read) {
            const size_t hole_size_before = polygons_src[read].points.size();
            remove_sticks(polygons_src[read]);
            source_unchanged = source_unchanged &&
                               polygons_src[read].points.size() == hole_size_before;
            if (polygons_src[read].points.size() >= 3) {
                if (write != read)
                    std::swap(polygons_src[read].points, polygons_src[write].points);
                ++write;
            } else
                source_unchanged = false;
        }
        polygons_src.erase(polygons_src.begin() + ptrdiff_t(write), polygons_src.end());
        //        if (sticks_removed) BOOST_LOG_TRIVIAL(error) << "Sticks removed!";
        if (aoffset1 == 0)
            polygons_outer = polygons_src;
        else if (source_unchanged)
            // Clipper can consume the original API view directly when no
            // rotation or cleanup changed the source contour.
            polygons_outer = offset(storage, expolygon, float(aoffset1), CLIPPER_JOIN_MITER, miterLimit);
        else
            polygons_outer = offset(storage, polygons_src, float(aoffset1), CLIPPER_JOIN_MITER, miterLimit);
        if (aoffset2 < 0)
            polygons_inner = shrink(storage, polygons_outer, float(aoffset1 - aoffset2), CLIPPER_JOIN_MITER, miterLimit);
        // Filter out contours with zero area or small area, contours with 2 points only.
        const double min_area_threshold = 0.01 * aoffset2 * aoffset2;
        remove_small(polygons_outer, min_area_threshold);
        remove_small(polygons_inner, min_area_threshold);
        remove_sticks(polygons_outer);
        remove_sticks(polygons_inner);
        n_contours_outer = polygons_outer.size();
        n_contours_inner = polygons_inner.size();
        n_contours = n_contours_outer + n_contours_inner;
        polygons_ccw.assign(n_contours, false);
        for (size_t i = 0; i < n_contours; ++ i) {
            this->contour(i).remove_duplicate_points();
            assert(! this->contour(i).has_duplicate_points());
            polygons_ccw[i] = is_ccw(this->contour(i));
        }
    }

    ExPolygonWithOffset(const ExPolygonWithOffset& rhs, float angle) : ExPolygonWithOffset(rhs) {
        if (angle != 0.f) {
            for (Polygon &polygon : this->polygons_src)
                polygon.rotate(angle);
            for (Polygon& poly : this->polygons_outer)
                poly.rotate(angle);
            for (Polygon& poly : this->polygons_inner)
                poly.rotate(angle);
        }
    }

    // Any contour with offset1
    bool             is_contour_outer(size_t idx) const { return idx < n_contours_outer; }
    // Any contour with offset2
    bool             is_contour_inner(size_t idx) const { return idx >= n_contours_outer; }

    const Polygon&   contour(size_t idx) const
        { return is_contour_outer(idx) ? polygons_outer[idx] : polygons_inner[idx - n_contours_outer]; }

    Polygon&         contour(size_t idx)
        { return is_contour_outer(idx) ? polygons_outer[idx] : polygons_inner[idx - n_contours_outer]; }

    bool             is_contour_ccw(size_t idx) const { return polygons_ccw[idx]; }

    BoundingBox      bounding_box_src() const
        { return get_extents(polygons_src); }
    BoundingBox      bounding_box_outer() const
        { return get_extents(polygons_outer); }
    BoundingBox      bounding_box_inner() const
        { return get_extents(polygons_inner); }

    Polygons         polygons_src;
    Polygons         polygons_outer;
    Polygons         polygons_inner;

    size_t           n_contours_outer;
    size_t           n_contours_inner;
    size_t           n_contours;

protected:
    // For each polygon of polygons_inner, remember its orientation.
    std::vector<unsigned char> polygons_ccw;
};

static inline int distance_of_segmens(const Polygon &poly, size_t seg1, size_t seg2, bool forward)
{
    int d = int(seg2) - int(seg1);
    if (! forward)
        d = - d;
    if (d < 0)
        d += int(poly.points.size());
    return d;
}
// Find an intersection on a previous line, but return -1, if the connecting segment of a perimeter was already extruded.
static inline bool intersection_on_prev_next_vertical_line_valid(
    const std::vector<SegmentedIntersectionLine>& segs,
    size_t                                         iVerticalLine,
    size_t                                         iIntersection,
    SegmentIntersection::Side                      side)
{
    const SegmentedIntersectionLine& vline_this = segs[iVerticalLine];
    const SegmentIntersection& it_this = vline_this.intersections[iIntersection];
    if (it_this.has_vertical(side))
        // Not the first intersection along the contor. This intersection point
        // has been preceded by an intersection point along the vertical line.
        return false;
    int iIntersectionOther = it_this.horizontal(side);
    if (iIntersectionOther == -1)
        return false;
    assert(side == SegmentIntersection::Side::Right ? (iVerticalLine + 1 < segs.size()) : (iVerticalLine > 0));
    const SegmentedIntersectionLine& vline_other = segs[side == SegmentIntersection::Side::Right ? (iVerticalLine + 1) : (iVerticalLine - 1)];
    const SegmentIntersection& it_other = vline_other.intersections[iIntersectionOther];
    assert(it_other.is_inner());
    assert(iIntersectionOther > 0 && size_t(iIntersectionOther + 1) < vline_other.intersections.size());
    // Is iIntersectionOther at the boundary of a vertical segment?
    const SegmentIntersection& it_other2 = vline_other.intersections[it_other.is_low() ? iIntersectionOther - 1 : iIntersectionOther + 1];
    if (it_other2.is_inner())
        // Cannot follow a perimeter segment into the middle of another vertical segment.
        // Only perimeter segments connecting to the end of a vertical segment are followed.
        return false;
    assert(it_other.is_low() == it_other2.is_low());
    if (it_this.horizontal_quality(side) != SegmentIntersection::LinkQuality::Valid)
        return false;
    if (side == SegmentIntersection::Side::Right ? it_this.consumed_perimeter_right : it_other.consumed_perimeter_right)
        // This perimeter segment was already consumed.
        return false;
    if (it_other.is_low() ? it_other.consumed_vertical_up : vline_other.intersections[iIntersectionOther - 1].consumed_vertical_up)
        // This vertical segment was already consumed.
        return false;
    return true;
}

static inline bool intersection_on_prev_vertical_line_valid(
    const std::vector<SegmentedIntersectionLine>& segs,
    size_t                                         iVerticalLine,
    size_t                                         iIntersection)
{
    return intersection_on_prev_next_vertical_line_valid(segs, iVerticalLine, iIntersection, SegmentIntersection::Side::Left);
}

static inline bool intersection_on_next_vertical_line_valid(
    const std::vector<SegmentedIntersectionLine>& segs,
    size_t                                         iVerticalLine,
    size_t                                         iIntersection)
{
    return intersection_on_prev_next_vertical_line_valid(segs, iVerticalLine, iIntersection, SegmentIntersection::Side::Right);
}

// Measure an Euclidian length of a perimeter segment when going from iIntersection to iIntersection2.
static inline coordf_t measure_perimeter_horizontal_segment_length(
    const ExPolygonWithOffset& poly_with_offset,
    const std::vector<SegmentedIntersectionLine>& segs,
    size_t                                         iVerticalLine,
    size_t                                         iIntersection,
    size_t                                         iIntersection2)
{
    size_t                           iVerticalLineOther = iVerticalLine + 1;
    assert(iVerticalLineOther < segs.size());
    const SegmentedIntersectionLine& vline = segs[iVerticalLine];
    const SegmentIntersection& it = vline.intersections[iIntersection];
    const SegmentedIntersectionLine& vline2 = segs[iVerticalLineOther];
    const SegmentIntersection& it2 = vline2.intersections[iIntersection2];
    assert(it.iContour == it2.iContour);
    const Polygon& poly = poly_with_offset.contour(it.iContour);
    //    const bool                       ccw    = poly_with_offset.is_contour_ccw(vline.iContour);
    assert(it.type == it2.type);
    assert(it.iContour == it2.iContour);

    Point p1(vline.pos, it.pos());
    Point p2(vline2.pos, it2.pos());
    return it.is_low() ?
        segment_length(poly, it.iSegment, p1, it2.iSegment, p2) :
        segment_length(poly, it2.iSegment, p2, it.iSegment, p1);
}

// Append the points of a perimeter segment when going from iIntersection to iIntersection2.
// The first point (the point of iIntersection) will not be inserted,
// the last point will be inserted.
static inline void emit_perimeter_prev_next_segment(
    const ExPolygonWithOffset                     &poly_with_offset,
    const std::vector<SegmentedIntersectionLine>  &segs,
    size_t                                         iVerticalLine,
    size_t                                         iInnerContour,
    size_t                                         iIntersection,
    size_t                                         iIntersection2,
    Polyline                                      &out,
    bool                                           dir_is_next)
{
    size_t iVerticalLineOther = iVerticalLine;
    if (dir_is_next) {
        ++ iVerticalLineOther;
        assert(iVerticalLineOther < segs.size());
    } else {
        assert(iVerticalLineOther > 0);
        -- iVerticalLineOther;
    }

    const SegmentedIntersectionLine &il     = segs[iVerticalLine];
    const SegmentIntersection       &itsct  = il.intersections[iIntersection];
    const SegmentedIntersectionLine &il2    = segs[iVerticalLineOther];
    const SegmentIntersection       &itsct2 = il2.intersections[iIntersection2];
    const Polygon                   &poly   = poly_with_offset.contour(iInnerContour);
//    const bool                       ccw    = poly_with_offset.is_contour_ccw(iInnerContour);
    assert(itsct.type == itsct2.type);
    assert(itsct.iContour == itsct2.iContour);
    assert(itsct.is_inner());
    const bool                       forward = itsct.is_low() == dir_is_next;
    // Do not append the first point.
    // out.points.push_back(Point(il.pos, itsct.pos));
    if (forward)
        polygon_segment_append(out.points, poly, itsct.iSegment, itsct2.iSegment);
    else
        polygon_segment_append_reversed(out.points, poly, itsct.iSegment, itsct2.iSegment);
    // Append the last point.
    out.points.push_back(Point(il2.pos, itsct2.pos()));
}

static inline coordf_t measure_perimeter_segment_on_vertical_line_length(
    const ExPolygonWithOffset& poly_with_offset,
    const std::vector<SegmentedIntersectionLine>& segs,
    size_t                                         iVerticalLine,
    size_t                                         iIntersection,
    size_t                                         iIntersection2,
    bool                                           forward)
{
    const SegmentedIntersectionLine& il = segs[iVerticalLine];
    const SegmentIntersection& itsct = il.intersections[iIntersection];
    const SegmentIntersection& itsct2 = il.intersections[iIntersection2];
    const Polygon& poly = poly_with_offset.contour(itsct.iContour);
    assert(itsct.is_inner() == itsct2.is_inner());
    assert(itsct.type != itsct2.type);
    assert(itsct.iContour == itsct2.iContour);
    Point p1(il.pos, itsct.pos());
    Point p2(il.pos, itsct2.pos());
    return forward ?
        segment_length(poly, itsct.iSegment, p1, itsct2.iSegment, p2) :
        segment_length(poly, itsct2.iSegment, p2, itsct.iSegment, p1);
}

// Append the points of a perimeter segment when going from iIntersection to iIntersection2.
// The first point (the point of iIntersection) will not be inserted,
// the last point will be inserted.
static inline void emit_perimeter_segment_on_vertical_line(
    const ExPolygonWithOffset                     &poly_with_offset,
    const std::vector<SegmentedIntersectionLine>  &segs,
    size_t                                         iVerticalLine,
    size_t                                         iInnerContour,
    size_t                                         iIntersection,
    size_t                                         iIntersection2,
    Polyline                                      &out,
    bool                                           forward)
{
    const SegmentedIntersectionLine &il = segs[iVerticalLine];
    const SegmentIntersection       &itsct = il.intersections[iIntersection];
    const SegmentIntersection       &itsct2 = il.intersections[iIntersection2];
    const Polygon                   &poly = poly_with_offset.contour(iInnerContour);
    assert(itsct.is_inner());
    assert(itsct2.is_inner());
    assert(itsct.type != itsct2.type);
    assert(itsct.iContour == iInnerContour);
    assert(itsct.iContour == itsct2.iContour);
    // Do not append the first point.
    // out.points.push_back(Point(il.pos, itsct.pos));
    if (forward)
        polygon_segment_append(out.points, poly, itsct.iSegment, itsct2.iSegment);
    else
        polygon_segment_append_reversed(out.points, poly, itsct.iSegment, itsct2.iSegment);
    // Append the last point.
    out.points.push_back(Point(il.pos, itsct2.pos()));
}

//TBD: For precise infill, measure the area of a slab spanned by an infill line.
/*
static inline float measure_outer_contour_slab(
    const ExPolygonWithOffset                     &poly_with_offset,
    const std::vector<SegmentedIntersectionLine>  &segs,
    size_t                                         i_vline,
    size_t                                         iIntersection)
{
    const SegmentedIntersectionLine &il     = segs[i_vline];
    const SegmentIntersection       &itsct  = il.intersections[i_vline];
    const SegmentIntersection       &itsct2 = il.intersections[iIntersection2];
    const Polygon                   &poly   = poly_with_offset.contour((itsct.iContour);
    assert(itsct.is_outer());
    assert(itsct2.is_outer());
    assert(itsct.type != itsct2.type);
    assert(itsct.iContour == itsct2.iContour);
    if (! itsct.is_outer() || ! itsct2.is_outer() || itsct.type == itsct2.type || itsct.iContour != itsct2.iContour)
        // Error, return zero area.
        return 0.f;

    // Find possible connection points on the previous / next vertical line.
    int iPrev = intersection_on_prev_vertical_line(poly_with_offset, segs, i_vline, itsct.iContour, i_intersection);
    int iNext = intersection_on_next_vertical_line(poly_with_offset, segs, i_vline, itsct.iContour, i_intersection);
    // Find possible connection points on the same vertical line.
    int iAbove = iBelow = -1;
    // Does the perimeter intersect the current vertical line above intrsctn?
    for (size_t i = i_intersection + 1; i + 1 < seg.intersections.size(); ++ i)
        if (seg.intersections[i].iContour == itsct.iContour)
            { iAbove = i; break; }
    // Does the perimeter intersect the current vertical line below intrsctn?
    for (int i = int(i_intersection) - 1; i > 0; -- i)
        if (seg.intersections[i].iContour == itsct.iContour)
            { iBelow = i; break; }

    if (iSegAbove != -1 && seg.intersections[iAbove].type == SegmentIntersection::OUTER_HIGH) {
        // Invalidate iPrev resp. iNext, if the perimeter crosses the current vertical line earlier than iPrev resp. iNext.
        // The perimeter contour orientation.
        const Polygon &poly = poly_with_offset.contour(itsct.iContour);
        {
            int d_horiz = (iPrev  == -1) ? std::numeric_limits<int>::max() :
                distance_of_segmens(poly, segs[i_vline-1].intersections[iPrev].iSegment, itsct.iSegment, true);
            int d_down  = (iBelow == -1) ? std::numeric_limits<int>::max() :
                distance_of_segmens(poly, iSegBelow, itsct.iSegment, true);
            int d_up    = (iAbove == -1) ? std::numeric_limits<int>::max() :
                distance_of_segmens(poly, iSegAbove, itsct.iSegment, true);
            if (intrsection_type_prev == INTERSECTION_TYPE_OTHER_VLINE_OK && d_horiz > std::min(d_down, d_up))
                // The vertical crossing comes eralier than the prev crossing.
                // Disable the perimeter going back.
                intrsection_type_prev = INTERSECTION_TYPE_OTHER_VLINE_NOT_FIRST;
            if (d_up > std::min(d_horiz, d_down))
                // The horizontal crossing comes earlier than the vertical crossing.
                vert_seg_dir_valid_mask &= ~DIR_BACKWARD;
        }
        {
            int d_horiz = (iNext     == -1) ? std::numeric_limits<int>::max() :
                distance_of_segmens(poly, itsct.iSegment, segs[i_vline+1].intersections[iNext].iSegment, true);
            int d_down  = (iSegBelow == -1) ? std::numeric_limits<int>::max() :
                distance_of_segmens(poly, itsct.iSegment, iSegBelow, true);
            int d_up    = (iSegAbove == -1) ? std::numeric_limits<int>::max() :
                distance_of_segmens(poly, itsct.iSegment, iSegAbove, true);
            if (d_up > std::min(d_horiz, d_down))
                // The horizontal crossing comes earlier than the vertical crossing.
                vert_seg_dir_valid_mask &= ~DIR_FORWARD;
        }
    }
}
*/

void
FillRectilinear::init_spacing(coordf_t spacing, const raw_infill_pattern_params& params)
{
    (void)params;
    spacing_priv = spacing;
}

void configure_filler(FillRectilinear &filler, const raw_infill_pattern_params &params, storage_handle *storage)
{
    filler.layer_id = params.layer_id;
    filler.z = params.z;
    filler.angle = float(params.angle);
    filler.can_angle_cross = (params.infill_flags & C_INFILL_FLAG_CAN_ANGLE_CROSS) != 0;
    filler.link_max_length = params.link_max_length;
    filler.loop_clipping = params.loop_clipping;
    filler.overlap = params.overlap;
    filler.plugin_storage = storage;
    filler.bounding_box.min = Point(params.object_bounding_box.min.x, params.object_bounding_box.min.y);
    filler.bounding_box.max = Point(params.object_bounding_box.max.x, params.object_bounding_box.max.y);
    filler.bounding_box.defined = true;
    filler.init_spacing(params.spacing, params);
}


enum DirectionMask
{
    DIR_FORWARD  = 1,
    DIR_BACKWARD = 2
};

size_t compute_n_vlines(const BoundingBox& bounding_box, coord_t line_spacing)
{
    // n_vlines = ceil(bbox_width / line_spacing)
    return  1 + (bounding_box.max.x() - bounding_box.min.x() - 10) / line_spacing;
}
// Bridges and unit-density surfaces use the full-infill spacing path from the
// legacy FillParams implementation.
static inline bool is_full_infill(const raw_infill_pattern_params &params)
{
    return params.flow.is_bridge != 0 || (params.density > 0.9999f && params.density < 1.0001f);
}

coord_t compute_x0(const raw_infill_pattern_params& params, const BoundingBox& bounding_box, coord_t line_spacing)
{
    coord_t x0 = bounding_box.min.x();
    if (params.flow.is_bridge != 0 && params.bridge_offset >= 0) {
        x0 += params.bridge_offset;
    } else if (is_full_infill(params))
        x0 += (line_spacing + coord_t(SCALED_EPSILON)) / 2;
    return x0;
}

// Intersect a set of equally spaced vertical lines with expolygon.
std::vector<SegmentedIntersectionLine> vert_lines_for_polygon(const ExPolygonWithOffset &poly_with_offset, const BoundingBox &bounding_box, size_t n_vlines, coord_t x0, coord_t line_spacing)
{

    // For each contour
    // Allocate storage for the segments.
    std::vector<SegmentedIntersectionLine> segs(n_vlines, SegmentedIntersectionLine());
    for (size_t i = 0; i < n_vlines; ++ i) {
        segs[i].idx = i;
        segs[i].pos = x0 + i * line_spacing;
    }
    for (size_t iContour = 0; iContour < poly_with_offset.n_contours; ++ iContour) {
        const Points &contour = poly_with_offset.contour(iContour).points;
        bool is_hole = poly_with_offset.contour(iContour).is_clockwise();
        if (contour.size() < 2)
            continue;
        // For each segment
        for (size_t iSegment = 0; iSegment < contour.size(); ++ iSegment) {
            size_t iPrev = ((iSegment == 0) ? contour.size() : iSegment) - 1;
            const Point &p1 = contour[iPrev];
            const Point &p2 = contour[iSegment];
            // Which of the equally spaced vertical lines is intersected by this segment?
            coord_t l = p1(0);
            coord_t r = p2(0);
            if (l > r)
                std::swap(l, r);
            // il, ir are the left / right indices of vertical lines intersecting a segment
            int il = (l - x0) / line_spacing;
            while (il * line_spacing + x0 < l)
                ++ il;
            il = std::max(int(0), il);
            int ir = (r - x0 + line_spacing) / line_spacing;
            while (ir * line_spacing + x0 > r)
                -- ir;
            ir = std::min(int(segs.size()) - 1, ir);
            if (il > ir)
                // No vertical line intersects this segment.
                continue;
            assert(il >= 0 && size_t(il) < segs.size());
            assert(ir >= 0 && size_t(ir) < segs.size());
            for (int i = il; i <= ir; ++ i) {
                coord_t this_x = segs[i].pos;
                assert(this_x == i * line_spacing + x0);
                SegmentIntersection is;
                is.iContour = iContour;
                is.iSegment = iSegment;
                is.is_hole = is_hole;
                assert(l <= this_x);
                assert(r >= this_x);
                // Calculate the intersection position in y axis. x is known.
                if (p1.x() == this_x) {
                    if (p2.x() == this_x) {
                        // Ignore strictly vertical segments.
                        continue;
                    }
                    const Point &p0 = prev_value_modulo(iPrev, contour);
                    if (int64_t(p0.x() - p1.x()) * int64_t(p2.x() - p1.x()) > 0) {
                        // Ignore points of a contour touching the infill line from one side.
                        continue;
                    }
                    is.pos_p = p1.y();
                    is.pos_q = 1;
                } else if (p2.x() == this_x) {
                    const Point &p3 = next_value_modulo(iSegment, contour);
                    if (int64_t(p3.x() - p2.x()) * int64_t(p1.x() - p2.x()) > 0) {
                        // Ignore points of a contour touching the infill line from one side.
                        continue;
                    }
                    is.pos_p = p2.y();
                    is.pos_q = 1;
                } else {
                    // First calculate the intersection parameter 't' as a rational number with non negative denominator.
                    if (p2.x() > p1.x()) {
                        is.pos_p = this_x - p1.x();
                        is.pos_q = p2.x() - p1.x();
                    } else {
                        is.pos_p = p1.x() - this_x;
                        is.pos_q = p1.x() - p2.x();
                    }
                    assert(is.pos_q > 1);
                    assert(is.pos_p > 0 && is.pos_p < is.pos_q);
                    // Make an intersection point from the 't'.
                    is.pos_p *= int64_t(p2.y() - p1.y());
                    is.pos_p += p1.y() * int64_t(is.pos_q);
                }
                // +-1 to take rounding into account.
                assert(is.pos() + 1 >= std::min(p1.y(), p2.y()));
                assert(is.pos() <= std::max(p1.y(), p2.y()) + 1);
                segs[i].intersections.push_back(is);
            }
        }
    }

    return segs;
}
std::vector<SegmentedIntersectionLine> FillRectilinear::_vert_lines_for_polygon(const ExPolygonWithOffset& poly_with_offset, const BoundingBox& bounding_box, const raw_infill_pattern_params& params, coord_t line_spacing) const
{
    size_t n_vlines = compute_n_vlines(bounding_box, line_spacing);
    coord_t x0 = compute_x0(params, bounding_box, line_spacing);
    return vert_lines_for_polygon(poly_with_offset, bounding_box, n_vlines, x0, line_spacing);
}


static void slice_region_by_vertical_lines(const FillRectilinear* filler, std::vector<SegmentedIntersectionLine>& segs,
    const ExPolygonWithOffset& poly_with_offset)//, size_t n_vlines, coord_t x0, coord_t line_spacing)
{
    // Sort the intersections along their segments, specify the intersection types.
    for (size_t i_seg = 0; i_seg < segs.size(); ++i_seg) {
        SegmentedIntersectionLine& sil = segs[i_seg];
        // Sort the intersection points using exact rational arithmetic.
        std::sort(sil.intersections.begin(), sil.intersections.end());
        // Assign the intersection types, remove duplicate or overlapping intersection points.
        // When a loop vertex touches a vertical line, intersection point is generated for both segments.
        // If such two segments are oriented equally, then one of them is removed.
        // Otherwise the vertex is tangential to the vertical line and both segments are removed.
        // The same rule applies, if the loop is pinched into a single point and this point touches the vertical line:
        // The loop has a zero vertical size at the vertical line, therefore the intersection point is removed.
        size_t j = 0;
        for (size_t i = 0; i < sil.intersections.size(); ++i) {
            // What is the orientation of the segment at the intersection point?
            SegmentIntersection       &is       = sil.intersections[i];
            const size_t               iContour = is.iContour;
            const Points              &contour  = poly_with_offset.contour(iContour).points;
            const size_t               iSegment = is.iSegment;
            const size_t               iPrev    = prev_idx_modulo(iSegment, contour);
            const coord_t              dir      = contour[iSegment].x() - contour[iPrev].x();
            const bool                 low      = dir > 0;
            is.type = poly_with_offset.is_contour_outer(iContour) ?
                (low ? SegmentIntersection::OUTER_LOW : SegmentIntersection::OUTER_HIGH) :
                (low ? SegmentIntersection::INNER_LOW : SegmentIntersection::INNER_HIGH);
            bool take_next = true;
            if (j > 0) {
                SegmentIntersection &is2 = sil.intersections[j - 1];
                if (iContour == is2.iContour && is.pos_q == 1 && is2.pos_q == 1) {
                    // Two successive intersection points on a vertical line with the same contour, both points are end points of their respective contour segments.
                    if (is.pos_p == is2.pos_p) {
                        // Two successive segments meet exactly at the vertical line.
                        // Verify that the segments of sil.intersections[i] and sil.intersections[j-1] are adjoint.
                        assert(iSegment == prev_idx_modulo(is2.iSegment, contour) || is2.iSegment == iPrev);
                        assert(is.type == is2.type);
                        // Two successive segments of the same direction (both to the right or both to the left)
                        // meet exactly at the vertical line.
                        // Remove the second intersection point.
                        take_next = false;
                    } else if (is.type == is2.type) {
                        // Two non successive segments of the same direction (both to the right or both to the left)
                        // meet exactly at the vertical line. That means there is a Z shaped path, where the center segment
                        // of the Z shaped path is aligned with this vertical line.
                        // Remove one of the intersection points while maximizing the vertical segment length.
                        if (low) {
                            // Remove the second intersection point, keep the first intersection point.
                        } else {
                            // Remove the first intersection point, keep the second intersection point.
                            sil.intersections[j-1] = sil.intersections[i];
                        }
                        take_next = false;
                    }
                }
            }
            if (take_next) {
                // Vertical line intersects a contour segment at a general position (not at one of its end points).
                if (j < i)
                    sil.intersections[j] = sil.intersections[i];
                ++j;
            }
        }
        // Shrink the list of intersections, if any of the intersection was removed during the classification.
        if (j < sil.intersections.size())
            sil.intersections.erase(sil.intersections.begin() + j, sil.intersections.end());
    }
    // TODO: delete this when the bug will be fixed (you can slice  InfillIssue.3mf)
    for (size_t i_seg = 0; i_seg < segs.size(); ++i_seg) {
        SegmentedIntersectionLine& sil = segs[i_seg];
        if ((sil.intersections.size() & 1) == 1 && sil.intersections.size() > 1) {
            std::fprintf(stderr, "GridVarSpeed: invalid raster intersections\n");
            if (sil.intersections.back().iContour == sil.intersections[sil.intersections.size() - 2].iContour)
                sil.intersections.pop_back();
        }
        if (sil.intersections.size() == 1) {
            std::fprintf(stderr, "GridVarSpeed: invalid raster intersections\n");
            sil.intersections.clear();
        }
    }

    // Verify the segments. If something is wrong, give up.
#define ASSERT_THROW(CONDITION) do { assert(CONDITION); if (! (CONDITION)) throw std::runtime_error("Infill failed"); } while (0)
    for (size_t i_seg = 0; i_seg < segs.size(); ++ i_seg) {
        SegmentedIntersectionLine &sil = segs[i_seg];
        // The intersection points have to be even.
        ASSERT_THROW((sil.intersections.size() & 1) == 0);
        for (size_t i = 0; i < sil.intersections.size();) {
            // An intersection segment crossing the bigger contour may cross the inner offsetted contour even number of times.
            ASSERT_THROW(sil.intersections[i].type == SegmentIntersection::OUTER_LOW);
            size_t j = i + 1;
            ASSERT_THROW(j < sil.intersections.size());
            ASSERT_THROW(sil.intersections[j].type == SegmentIntersection::INNER_LOW || sil.intersections[j].type == SegmentIntersection::OUTER_HIGH);
            for (; j < sil.intersections.size() && sil.intersections[j].is_inner(); ++ j) ;
            ASSERT_THROW(j < sil.intersections.size());
            ASSERT_THROW((j & 1) == 1);
            ASSERT_THROW(sil.intersections[j].type == SegmentIntersection::OUTER_HIGH);
            ASSERT_THROW(i + 1 == j || sil.intersections[j - 1].type == SegmentIntersection::INNER_HIGH);
            i = j + 1;
        }
    }
#undef ASSERT_THROW

}

#ifndef NDEBUG
bool validate_segment_intersection_connectivity(const std::vector<SegmentedIntersectionLine> &segs)
{
    // Validate the connectivity.
    for (size_t i_vline = 0; i_vline + 1 < segs.size(); ++ i_vline) {
        const SegmentedIntersectionLine &il_left  = segs[i_vline];
        const SegmentedIntersectionLine &il_right = segs[i_vline + 1];
        for (const SegmentIntersection &it : il_left.intersections) {
            if (it.has_right_horizontal()) {
                const SegmentIntersection &it_right = il_right.intersections[it.right_horizontal()];
                // For a right link there is a symmetric left link.
                assert(it.iContour == it_right.iContour);
                assert(it.type == it_right.type);
                assert(it_right.has_left_horizontal());
                assert(it_right.left_horizontal() == int(&it - il_left.intersections.data()));
            }
        }
        for (const SegmentIntersection &it : il_right.intersections) {
            if (it.has_left_horizontal()) {
                const SegmentIntersection &it_left = il_left.intersections[it.left_horizontal()];
                // For a right link there is a symmetric left link.
                assert(it.iContour == it_left.iContour);
                assert(it.type == it_left.type);
                assert(it_left.has_right_horizontal());
                assert(it_left.right_horizontal() == int(&it - il_right.intersections.data()));
            }
        }
    }
    for (size_t i_vline = 0; i_vline < segs.size(); ++ i_vline) {
        const SegmentedIntersectionLine &il = segs[i_vline];
        for (const SegmentIntersection &it : il.intersections) {
            auto i_it = int(&it - il.intersections.data());
            if (it.has_left_vertical_up()) {
                assert(il.intersections[it.left_vertical_up()].left_vertical_down() == i_it);
                assert(il.intersections[it.left_vertical_up()].prev_on_contour_quality == it.prev_on_contour_quality);
            }
            if (it.has_left_vertical_down()) {
                assert(il.intersections[it.left_vertical_down()].left_vertical_up() == i_it);
                assert(il.intersections[it.left_vertical_down()].prev_on_contour_quality == it.prev_on_contour_quality);
            }
            if (it.has_right_vertical_up()) {
                assert(il.intersections[it.right_vertical_up()].right_vertical_down() == i_it);
                assert(il.intersections[it.right_vertical_up()].next_on_contour_quality == it.next_on_contour_quality);
            }
            if (it.has_right_vertical_down()) {
                assert(il.intersections[it.right_vertical_down()].right_vertical_up() == i_it);
                assert(il.intersections[it.right_vertical_down()].next_on_contour_quality == it.next_on_contour_quality);
            }
        }
    }
    return true;
}
#endif /* NDEBUG */

// Connect each contour / vertical line intersection point with another two contour / vertical line intersection points.
// (fill in SegmentIntersection::{prev_on_contour, prev_on_contour_vertical, next_on_contour, next_on_contour_vertical}.
// These contour points are either on the same vertical line, or on the vertical line left / right to the current one.
static void connect_segment_intersections_by_contours(
    const ExPolygonWithOffset& poly_with_offset, std::vector<SegmentedIntersectionLine>& segs,
    const raw_infill_pattern_params& params, const coord_t link_max_length)
{
    for (size_t i_vline = 0; i_vline < segs.size(); ++i_vline) {
        SegmentedIntersectionLine& il = segs[i_vline];
        const SegmentedIntersectionLine* il_prev = i_vline > 0 ? &segs[i_vline - 1] : nullptr;
        const SegmentedIntersectionLine* il_next = i_vline + 1 < segs.size() ? &segs[i_vline + 1] : nullptr;

        for (size_t i_intersection = 0; i_intersection < il.intersections.size(); ++i_intersection) {
            SegmentIntersection& itsct = il.intersections[i_intersection];
            const Polygon& poly = poly_with_offset.contour(itsct.iContour);
            const bool           forward = itsct.is_low(); // == poly_with_offset.is_contour_ccw(intrsctn->iContour);

            // 1) Find possible connection points on the previous / next vertical line.
            // Find an intersection point on il_prev, intersecting i_intersection
            // at the same orientation as i_intersection, and being closest to i_intersection
            // in the number of contour segments, when following the direction of the contour.
            //FIXME this has O(n) time complexity. Likely an O(log(n)) scheme is possible.
            int iprev = -1;
            int d_prev = std::numeric_limits<int>::max();
            if (il_prev) {
                for (int i = 0; i < il_prev->intersections.size(); ++i) {
                    const SegmentIntersection& itsct2 = il_prev->intersections[i];
                    if (itsct.iContour == itsct2.iContour && itsct.type == itsct2.type) {
                        // The intersection points lie on the same contour and have the same orientation.
                        // Find the intersection point with a shortest path in the direction of the contour.
                        int d = distance_of_segmens(poly, itsct2.iSegment, itsct.iSegment, forward);
                        if (d < d_prev) {
                            iprev = i;
                            d_prev = d;
                        }
                    }
                }
            }

            // The same for il_next.
            int inext = -1;
            int d_next = std::numeric_limits<int>::max();
            if (il_next) {
                for (size_t i = 0; i < il_next->intersections.size(); ++i) {
                    const SegmentIntersection& itsct2 = il_next->intersections[i];
                    if (itsct.iContour == itsct2.iContour && itsct.type == itsct2.type) {
                        // The intersection points lie on the same contour and have the same orientation.
                        // Find the intersection point with a shortest path in the direction of the contour.
                        int d = distance_of_segmens(poly, itsct.iSegment, itsct2.iSegment, forward);
                        if (d < d_next) {
                            inext = int(i);
                            d_next = d;
                        }
                    }
                }
            }

            // 2) Find possible connection points on the same vertical line.
            bool same_prev = false;
            bool same_next = false;
            // Does the perimeter intersect the current vertical line above intrsctn?
            for (size_t i = 0; i < il.intersections.size(); ++i)
                if (const SegmentIntersection& it2 = il.intersections[i];
                    i != i_intersection && it2.iContour == itsct.iContour && it2.type != itsct.type) {
                int d = distance_of_segmens(poly, it2.iSegment, itsct.iSegment, forward);
                if (d < d_prev) {
                    iprev = i;
                    d_prev = d;
                    same_prev = true;
                }
                d = distance_of_segmens(poly, itsct.iSegment, it2.iSegment, forward);
                if (d < d_next) {
                    inext = i;
                    d_next = d;
                    same_next = true;
                }
            }
            // note: here, an intersection can have its prev to vertical on the same line, but the other intersection can have its intersection on horizontal
            // it can be problematic...
            assert(iprev >= 0);
            assert(inext >= 0);

            itsct.prev_on_contour = iprev;
            itsct.prev_on_contour_type = same_prev ?
                (iprev < int(i_intersection) ? SegmentIntersection::LinkType::Down : SegmentIntersection::LinkType::Up) :
                SegmentIntersection::LinkType::Horizontal;
            itsct.next_on_contour = inext;
            itsct.next_on_contour_type = same_next ?
                (inext < int(i_intersection) ? SegmentIntersection::LinkType::Down : SegmentIntersection::LinkType::Up) :
                SegmentIntersection::LinkType::Horizontal;

            if (same_prev) {
                // Only follow a vertical perimeter segment if it skips just the outer intersections.
                SegmentIntersection* it = &itsct;
                SegmentIntersection* end = il.intersections.data() + iprev;
                assert(it != end);
                if (it > end)
                    std::swap(it, end);
                for (++it; it != end; ++it)
                    if (it->is_inner()) {
                        itsct.prev_on_contour_quality = SegmentIntersection::LinkQuality::Invalid;
                        break;
                    }
            }

            if (same_next) {
                // Only follow a vertical perimeter segment if it skips just the outer intersections.
                SegmentIntersection* it = &itsct;
                SegmentIntersection* end = il.intersections.data() + inext;
                assert(it != end);
                if (it > end)
                    std::swap(it, end);
                for (++it; it != end; ++it)
                    if (it->is_inner()) {
                        itsct.next_on_contour_quality = SegmentIntersection::LinkQuality::Invalid;
                        break;
                    }
            }

            // If both iprev and inext are on this vline, then there must not be any intersection with the previous or next contour and we will
            // not trace this contour when generating infill.
            if (same_prev && same_next) {
                assert(iprev != i_intersection);
                assert(inext != i_intersection);
                if ((iprev > i_intersection) == (inext > i_intersection)) {
                    // Both closest intersections of this contour are on the same vertical line and at the same side of this point.
                    // Ignore them when tracing the infill.
                    itsct.prev_on_contour_quality = SegmentIntersection::LinkQuality::Invalid;
                    itsct.next_on_contour_quality = SegmentIntersection::LinkQuality::Invalid;
                }
            }

            if (params.connection == C_INFILL_CONNECTION_NOT_CONNECTED
                || (params.connection == C_INFILL_CONNECTION_OUTER_SHELL && itsct.is_hole) // not: don't work as expected because the algo only start from top and go bottom.
                || (params.connection == C_INFILL_CONNECTION_HOLES && !itsct.is_hole)
                ) {
                if (itsct.prev_on_contour_quality == SegmentIntersection::LinkQuality::Valid)
                    itsct.prev_on_contour_quality = SegmentIntersection::LinkQuality::TooLong;
                if (itsct.next_on_contour_quality == SegmentIntersection::LinkQuality::Valid)
                    itsct.next_on_contour_quality = SegmentIntersection::LinkQuality::TooLong;
            } else if (link_max_length > 0 && false /*FIXME SuperSlicer::2141*/) {
                // Measure length of the links.
                if (itsct.prev_on_contour_quality == SegmentIntersection::LinkQuality::Valid &&
                    (same_prev ?
                        measure_perimeter_segment_on_vertical_line_length(poly_with_offset, segs, i_vline, iprev, i_intersection, forward) :
                        measure_perimeter_horizontal_segment_length(poly_with_offset, segs, i_vline - 1, iprev, i_intersection)) > link_max_length)
                    itsct.prev_on_contour_quality = SegmentIntersection::LinkQuality::TooLong;
                if (itsct.next_on_contour_quality == SegmentIntersection::LinkQuality::Valid &&
                    (same_next ?
                        measure_perimeter_segment_on_vertical_line_length(poly_with_offset, segs, i_vline, i_intersection, inext, forward) :
                        measure_perimeter_horizontal_segment_length(poly_with_offset, segs, i_vline, i_intersection, inext)) > link_max_length)
                    itsct.next_on_contour_quality = SegmentIntersection::LinkQuality::TooLong;
                }
            }

        // Removed un-mirrored vertical connection.
        for (size_t i_intersection = 0; i_intersection < il.intersections.size(); ++i_intersection) {
            SegmentIntersection& it = il.intersections[i_intersection];
            if (it.has_left_vertical()) {
                SegmentIntersection& it2 = il.intersections[it.left_vertical()];
                if (it2.left_vertical() != i_intersection) {
                    // as it can happen that a vertical connection isn't symetric, if it happens, break the erroneous link
                    it.prev_on_contour = -1;
                    it.prev_on_contour_type = SegmentIntersection::LinkType::Phony;
                }
            }
            if (it.has_right_vertical()) {
                SegmentIntersection& it2 = il.intersections[it.right_vertical()];
                if (it2.right_vertical() != i_intersection) {
                    // as it can happen that a vertical connection isn't symetric, if it happens, break the erroneous link
                    it.next_on_contour = -1;
                    it.next_on_contour_type = SegmentIntersection::LinkType::Phony;
                }
            }
        }

        // Make the LinkQuality::Invalid symmetric on vertical connections.
        for (size_t i_intersection = 0; i_intersection < il.intersections.size(); ++i_intersection) {
            SegmentIntersection& it = il.intersections[i_intersection];
            if (it.has_left_vertical() && it.prev_on_contour_quality == SegmentIntersection::LinkQuality::Invalid) {
                SegmentIntersection& it2 = il.intersections[it.left_vertical()];
                assert(it2.left_vertical() == i_intersection);
                it2.prev_on_contour_quality = SegmentIntersection::LinkQuality::Invalid;
            }
            if (it.has_right_vertical() && it.next_on_contour_quality == SegmentIntersection::LinkQuality::Invalid) {
                SegmentIntersection& it2 = il.intersections[it.right_vertical()];
                assert(it2.right_vertical() == i_intersection);
                it2.next_on_contour_quality = SegmentIntersection::LinkQuality::Invalid;
            }
        }
    }

    assert(validate_segment_intersection_connectivity(segs));
}

static void traverse_graph_generate_polylines(
    const ExPolygonWithOffset& poly_with_offset, const raw_infill_pattern_params& params, std::vector<SegmentedIntersectionLine>& segs, Polylines& polylines_out, coord_t spacing, bool inverted_dir = false)
{
    // For each outer only chords, measure their maximum distance to the bow of the outer contour.
    // Mark an outer only chord as consumed, if the distance is low.
    for (size_t i_vline = 0; i_vline < segs.size(); ++i_vline) {
        SegmentedIntersectionLine& vline = segs[i_vline];
        for (size_t i_intersection = 0; i_intersection + 1 < vline.intersections.size(); ++i_intersection) {
            if (vline.intersections[i_intersection].type == SegmentIntersection::OUTER_LOW &&
                vline.intersections[i_intersection + 1].type == SegmentIntersection::OUTER_HIGH) {
                bool consumed = false;
                //                if (is_full_infill(params)) {
                //                        measure_outer_contour_slab(poly_with_offset, segs, i_vline, i_ntersection);
                //                } else
                consumed = true;
                vline.intersections[i_intersection].consumed_vertical_up = consumed;
            }
        }
    }

    // Now construct a graph.
    // Find the first point.
    // Naively one would expect to achieve best results by chaining the paths by the shortest distance,
    // but that procedure does not create the longest continuous paths.
    // A simple "sweep left to right" procedure achieves better results.
    int    	  i_vline = 0;
    int    	  i_intersection = -1;
    // Follow the line, connect the lines into a graph.
    // Until no new line could be added to the output path:
    Point     pointLast;
    Polyline* polyline_current = nullptr;
    if (!polylines_out.empty())
        pointLast = polylines_out.back().points.back();
    for (;;) {
        if (i_intersection == -1) {
            if (!polylines_out.empty()) {
                // The path has been interrupted. Find a next starting point, closest to the previous extruder position.
                coordf_t dist2min = std::numeric_limits<coordf_t>().max();
                for (size_t i_vline2 = 0; i_vline2 < segs.size(); ++i_vline2) {
                    const SegmentedIntersectionLine& vline = segs[i_vline2];
                    if (!vline.intersections.empty()) {
                        assert(vline.intersections.size() > 1);
                        // Even number of intersections with the loops.
                        assert((vline.intersections.size() & 1) == 0);
                        assert(vline.intersections.front().type == SegmentIntersection::OUTER_LOW);
                        for (size_t i = 0; i < vline.intersections.size(); ++i) {
                            const SegmentIntersection& intrsctn = vline.intersections[i];
                            if (intrsctn.is_outer()) {
                                assert(intrsctn.is_low() || i > 0);
                                bool consumed = intrsctn.is_low() ?
                                    intrsctn.consumed_vertical_up :
                                    vline.intersections[i - 1].consumed_vertical_up;
                                if (!consumed) {
                                    const coordf_t dx = coordf_t(pointLast(0) - vline.pos);
                                    const coordf_t dy = coordf_t(pointLast(1) - intrsctn.pos());
                                    coordf_t dist2 = dx * dx + dy * dy;
                                    if (dist2 < dist2min) {
                                        dist2min = dist2;
                                        i_vline = int(i_vline2);
                                        i_intersection = int(i);
                                        //FIXME We are taking the first left point always. Verify, that the caller chains the paths
                                        // by a shortest distance, while reversing the paths if needed.
                                        //if (polylines_out.empty())
                                            // Initial state, take the first line, which is the first from the left.
                                        goto found;
                                    }
                                }
                            }
                        }
                    }
                }
                if (i_intersection == -1)
                    // We are finished.
                    break;
            found:
                // Start a new path.
                polylines_out.push_back(Polyline());
                polyline_current = &polylines_out.back();
                // Emit the first point of a path.
                pointLast = Point(segs[i_vline].pos, segs[i_vline].intersections[i_intersection].pos());
                polyline_current->points.push_back(pointLast);

            } else {
                //find the starting intersection
                i_vline = 0;
                i_intersection = 0;
                bool found = false;
                while (!found) {
                    //go to next column if we don't found a suitable one in the current column
                    while (i_intersection >= segs[i_vline].intersections.size()) {
                        i_vline++;
                        i_intersection = 0;
                        if (i_vline >= segs.size()) {
                            // nothing to merge
                            return;
                        }
                    }
                    assert(segs[i_vline].intersections[i_intersection].is_low() || i_intersection > 0);
                    //does the current on is suitable?
                    bool consumed = segs[i_vline].intersections[i_intersection].is_low() ?
                        segs[i_vline].intersections[i_intersection].consumed_vertical_up :
                        segs[i_vline].intersections[i_intersection - 1].consumed_vertical_up;
                    //move
                    if (consumed)
                        i_intersection++;
                    else
                        found = true;
                }
                //if inverted dir, stay on the current column but try to start at the opposide side
                if (inverted_dir) {
                    int i_intersection_inv = segs[i_vline].intersections.size() - 1;
                    found = false;
                    while (!found) {
                        assert(segs[i_vline].intersections[i_intersection_inv].is_low() || i_intersection_inv > 0);
                        bool consumed_inv = segs[i_vline].intersections[i_intersection_inv].is_low() ?
                            segs[i_vline].intersections[i_intersection_inv].consumed_vertical_up :
                            segs[i_vline].intersections[i_intersection_inv - 1].consumed_vertical_up;
                        if (consumed_inv)
                            i_intersection_inv--;
                        else
                            found = true;
                        if (i_intersection_inv <= i_intersection) {
                            //can't use another start point, return so we can use the other path.
                            return;
                        }
                    }
                    i_intersection = i_intersection_inv;
                }
                // Start a new path with no previous position
                polylines_out.push_back(Polyline());
                polyline_current = &polylines_out.back();
                // Emit the first point of a path.
                pointLast = Point(segs[i_vline].pos, segs[i_vline].intersections[i_intersection].pos());
                polyline_current->points.push_back(pointLast);
            }
        }

        assert(i_vline >= 0);
        assert(i_intersection >= 0);
        // From the initial point (i_vline, i_intersection), follow a path.
        SegmentedIntersectionLine& vline = segs[i_vline];
        SegmentIntersection* it = &vline.intersections[i_intersection];
        bool 					   going_up = it->is_low();
        bool 					   try_connect = false;
        if (going_up) {
            assert(!it->consumed_vertical_up);
            assert(size_t(i_intersection + 1) < vline.intersections.size());
            // Step back to the beginning of the vertical segment to mark it as consumed.
            if (it->is_inner()) {
                assert(i_intersection > 0);
                --it;
                --i_intersection;
            }
            // Consume the complete vertical segment up to the outer contour.
            do {
                it->consumed_vertical_up = true;
                ++it;
                ++i_intersection;
                assert(size_t(i_intersection) < vline.intersections.size());
            } while (it->type != SegmentIntersection::OUTER_HIGH);
            if ((it - 1)->is_inner()) {
                // Step back.
                --it;
                --i_intersection;
                assert(it->type == SegmentIntersection::INNER_HIGH);
                try_connect = true;
            }
        } else {
            // Going down.
            assert(it->is_high());
            assert(i_intersection > 0);
            assert(!(it - 1)->consumed_vertical_up);
            // Consume the complete vertical segment up to the outer contour.
            if (it->is_inner())
                it->consumed_vertical_up = true;
            do {
                assert(i_intersection > 0);
                --it;
                --i_intersection;
                it->consumed_vertical_up = true;
            } while (it->type != SegmentIntersection::OUTER_LOW);
            if ((it + 1)->is_inner()) {
                // Step back.
                ++it;
                ++i_intersection;
                assert(it->type == SegmentIntersection::INNER_LOW);
                try_connect = true;
            }
        }
        if (try_connect) {
            // Decide, whether to finish the segment, or whether to follow the perimeter.
            // 1) Find possible connection points on the previous / next vertical line.
            int  i_prev = it->left_horizontal();
            int  i_next = it->right_horizontal();
            bool intersection_prev_valid = intersection_on_prev_vertical_line_valid(segs, i_vline, i_intersection);
            bool intersection_next_valid = intersection_on_next_vertical_line_valid(segs, i_vline, i_intersection);
            // special path for bridges: do not make long connection, as it's over-extruded.
            if (params.flow.is_bridge != 0) {
                coordf_t max_length = coordf_t(spacing) * 2.1;
                intersection_prev_valid = (intersection_prev_valid
                    && measure_perimeter_horizontal_segment_length(poly_with_offset, segs, i_vline - 1, i_prev, i_intersection) < max_length);
                intersection_next_valid = (intersection_next_valid
                    && measure_perimeter_horizontal_segment_length(poly_with_offset, segs, i_vline, i_intersection, i_next) < max_length);
            }
            bool intersection_horizontal_valid = intersection_prev_valid || intersection_next_valid;
            // Mark both the left and right connecting segment as consumed, because one cannot go to this intersection point as it has been consumed.
            if (i_prev != -1)
                segs[i_vline - 1].intersections[i_prev].consumed_perimeter_right = true;
            if (i_next != -1)
                it->consumed_perimeter_right = true;

            // Try to connect to a previous or next vertical line, making a zig-zag pattern.
            if (intersection_horizontal_valid) {
                // A horizontal connection along the perimeter line exists.
                assert(it->is_inner());
                bool take_next = intersection_next_valid;
                if (intersection_prev_valid && intersection_next_valid) {
                    // Take the shorter segment. This greedy heuristics may not be the best.
                    coordf_t dist_prev = measure_perimeter_horizontal_segment_length(poly_with_offset, segs, i_vline - 1, i_prev, i_intersection);
                    coordf_t dist_next = measure_perimeter_horizontal_segment_length(poly_with_offset, segs, i_vline, i_intersection, i_next);
                    take_next = dist_next < dist_prev;
                }
                polyline_current->points.emplace_back(vline.pos, it->pos());
                emit_perimeter_prev_next_segment(poly_with_offset, segs, i_vline, it->iContour, i_intersection, take_next ? i_next : i_prev, *polyline_current, take_next);
                //FIXME consume the left / right connecting segments at the other end of this line? Currently it is not critical because a perimeter segment is not followed if the vertical segment at the other side has already been consumed.
                // Advance to the neighbor line.
                if (take_next) {
                    ++i_vline;
                    i_intersection = i_next;
                } else {
                    --i_vline;
                    i_intersection = i_prev;
                }
                continue;
            }

            // Try to connect to a previous or next point on the same vertical line.
            int i_vertical = it->vertical_outside();
            auto vertical_link_quality = (i_vertical == -1 || vline.intersections[i_vertical + (going_up ? 0 : -1)].consumed_vertical_up) ?
                SegmentIntersection::LinkQuality::Invalid : it->vertical_outside_quality();
#if 0
            if (vertical_link_quality == SegmentIntersection::LinkQuality::Valid ||
                // Follow the link if there is no horizontal link available.
                (!intersection_horizontal_valid && vertical_link_quality != SegmentIntersection::LinkQuality::Invalid)) {
#else
            if (vertical_link_quality != SegmentIntersection::LinkQuality::Invalid) {
#endif
                assert(it->iContour == vline.intersections[i_vertical].iContour);
                polyline_current->points.emplace_back(vline.pos, it->pos());
                if (vertical_link_quality == SegmentIntersection::LinkQuality::Valid)
                    // Consume the connecting contour and the next segment.
                    emit_perimeter_segment_on_vertical_line(poly_with_offset, segs, i_vline, it->iContour, i_intersection, i_vertical,
                        *polyline_current, going_up ? it->has_left_vertical_up() : it->has_right_vertical_down());
                else {
                    // Just skip the connecting contour and start a new path.
                    polylines_out.emplace_back();
                    polyline_current = &polylines_out.back();
                    polyline_current->points.emplace_back(vline.pos, vline.intersections[i_vertical].pos());
                }
                // Mark both the left and right connecting segment as consumed, because one cannot go to this intersection point as it has been consumed.
                // If there are any outer intersection points skipped (bypassed) by the contour,
                // mark them as processed.
                if (going_up)
                    for (int i = i_intersection; i < i_vertical; ++i)
                        vline.intersections[i].consumed_vertical_up = true;
                else
                    for (int i = i_vertical; i < i_intersection; ++i)
                        vline.intersections[i].consumed_vertical_up = true;
                // seg.intersections[going_up ? i_intersection : i_intersection - 1].consumed_vertical_up = true;
                it->consumed_perimeter_right = true;
                (going_up ? ++it : --it)->consumed_perimeter_right = true;
                i_intersection = i_vertical;
                continue;
            }

            // No way to continue the current polyline. Take the rest of the line up to the outer contour.
            // This will finish the polyline, starting another polyline at a new point.
            going_up ? ++it : --it;
        }

        // Finish the current vertical line,
        // reset the current vertical line to pick a new starting point in the next round.
        assert(it->is_outer());
        assert(it->is_high() == going_up);
        pointLast = Point(vline.pos, it->pos());
        polyline_current->points.emplace_back(pointLast);
        // Handle duplicate points and zero length segments.
        polyline_current->remove_duplicate_points();
        assert(!polyline_current->has_duplicate_points());
        // Handle nearly zero length edges.
        if (polyline_current->points.size() <= 1 ||
            (polyline_current->points.size() == 2 &&
                std::abs(polyline_current->points.front()(0) - polyline_current->points.back()(0)) < SCALED_EPSILON &&
                std::abs(polyline_current->points.front()(1) - polyline_current->points.back()(1)) < SCALED_EPSILON))
        {
            polylines_out.pop_back();
        }
        it = nullptr;
        i_intersection = -1;
        polyline_current = nullptr;
    }
}
bool FillRectilinear::fill_surface_by_lines(const slic3r_api::ExPolygon &expolygon, const raw_infill_pattern_params &params, float angleBase, float pattern_shift, Polylines &polylines_out) const
{
    assert(expolygon.is_valid());

    // At the end, only the new polylines will be rotated back.
    size_t n_polylines_out_initial = polylines_out.size();

    // Shrink the input polygon a bit first to not push the infill lines out of the perimeters.
//    const float INFILL_OVERLAP_OVER_SPACING = 0.3f;
    //const float INFILL_OVERLAP_OVER_SPACING = 0.45f; //merill: what is this value???
    //assert(INFILL_OVERLAP_OVER_SPACING > 0 && INFILL_OVERLAP_OVER_SPACING < 0.5f);

    // Rotate polygons so that we can work with vertical lines here
    std::pair<float, Point> rotate_vector = this->_infill_direction(expolygon, params);
    rotate_vector.first += angleBase;

    assert(params.density > 0.0001f);
    coord_t line_spacing = _line_spacing_for_density(params);

    // On the polygons of poly_with_offset, the infill lines will be connected.
    ExPolygonWithOffset poly_with_offset(
        this->plugin_storage,
        expolygon,
        - rotate_vector.first,
        (scale_i(0 /*this->overlap*/ - /*(0.5 - INFILL_OVERLAP_OVER_SPACING)*/ 0.05 * this->get_spacing())), // outer offset, have to be > to the inner one (less negative)
        // inner offset (don't put 0.5, as it will cut full-filled area when it's exactly at the right place)
        // Note: this will put the fill 2% too far into the perimeter.
        (scale_i(0 /*this->overlap*/ - 0.48f * this->get_spacing())));
    if (poly_with_offset.n_contours_inner == 0) {
        // Not a single infill line fits.
        //Prusa: maybe one shall trigger the gap fill here?
        //supermerill: not possible here, gapfill return a ThickPolyline, not a Polyline. Have to it after that, to fill un-exterded areas.
        return true;
    }

    BoundingBox bounding_box = poly_with_offset.bounding_box_src();

    // define flow spacing according to requested density
    if ((is_full_infill(params) && (params.infill_flags & C_INFILL_FLAG_DONT_ADJUST) == 0) || line_spacing == 0 ) {
        line_spacing = scale_i(this->get_spacing());
    } else if (!is_full_infill(params)) {
        // extend bounding box so that our pattern will be aligned with other layers
        // Transform the reference point to the rotated coordinate system.
        Point refpt = rotate_vector.second.rotated(- rotate_vector.first);
        // align_to_grid will not work correctly with positive pattern_shift.
        coord_t pattern_shift_scaled = scale_i(pattern_shift) % line_spacing;
        refpt.x() -= (pattern_shift_scaled >= 0) ? pattern_shift_scaled : (line_spacing + pattern_shift_scaled);
        bounding_box.merge(align_to_grid(
            bounding_box.min,
            Point(line_spacing, line_spacing),
            refpt));
    }

    // Intersect a set of equally spaced vertical lines with expolygon.
    std::vector<SegmentedIntersectionLine> segs = _vert_lines_for_polygon(poly_with_offset, bounding_box, params, line_spacing);

    if (segs.empty())
        return false;

    slice_region_by_vertical_lines(this, segs, poly_with_offset);

    //all the works is done HERE
    // Connect by horizontal / vertical links, classify the links based on link_max_length as too long.
    connect_segment_intersections_by_contours(poly_with_offset, segs, params, link_max_length);


    { // gridvarspeed accepts only the ordinary connected sweep.
        std::vector<SegmentedIntersectionLine> segs_save = segs;
        if (polylines_out.size() > 0) {
            traverse_graph_generate_polylines(poly_with_offset, params, segs, polylines_out, line_spacing);
        } else {
            traverse_graph_generate_polylines(poly_with_offset, params, segs, polylines_out, line_spacing);
            if (polylines_out.size() > 1) {
                //try with inverted dir if the connection is better
                Polylines next_try;
                traverse_graph_generate_polylines(poly_with_offset, params, segs_save, next_try, line_spacing, true);
                if (next_try.size() > 0 && next_try.size() < polylines_out.size())
                    polylines_out = next_try;
            }
        }
    }


    // paths must be rotated back
    for (Polylines::iterator it = polylines_out.begin() + n_polylines_out_initial; it != polylines_out.end();) {
        // No need to translate, the absolute position is irrelevant.
        // it->translate(- rotate_vector.second(0), - rotate_vector.second(1));
        // assert(! it->has_duplicate_points());
        it->remove_duplicate_points();
        it->rotate(rotate_vector.first);
        // simplify the paths to avoid very short edges
        it->douglas_peucker(std::max(SCALED_EPSILON * 10, params.fill_resolution / 10));
        if (it->length() <= params.fill_resolution) {
            it = polylines_out.erase(it);
        } else {
            it->assert_valid();
            ++it;
        }
    }

#ifdef SLIC3R_DEBUG
    // Verify, that there are no duplicate points in the sequence.
    for (Polyline &polyline : polylines_out)
        assert(! polyline.has_duplicate_points());
#endif /* SLIC3R_DEBUG */

    return true;
}

} // namespace fills
}}} // namespace slic3r_api::GridVarSpeedCopy::RectilinearPatternsPlugin
