///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* Geometry helpers of the tree support family, written on the plugin API types (c_point, polygon views, storage-owned collections).

   The tree routing and the organic smoothing were written on the native libslic3r types (Point, Polygons, ClipperUtils, AABB trees).
   This header holds the small pieces they need once they use the plugin API only. What the API already provides is not repeated here:
   - EPSILON, SCALED_EPSILON, scale_i/scale_d/unscaled: Api/plugin/c/slic3r_def.h;
   - c_point arithmetic (+ - * /, midpoint, norm, norm_square), c_bounding_box merge/size/center: GeometryViews.hpp
     (the operators live in slic3r_api and c_point is a C struct, so the using-declarations below make them visible here);
   - polygon, polyline, bounding box (MultiPoint::bounding_box), contains of one ring, ensure_valid: GeometryViews.hpp;
   - ClipperOperand and the Clipper operations, CLIPPER_JOIN_* (the join types jtMiter / jtRound), clipper_offset_polylines,
     ClipperOperand::bounding_box: ClipperViews.hpp; native-like *_ex results: Support/Common/ClipperResults.hpp;
   - polygons_simplify: Api/plugin/cpp/PolygonOps.hpp; LineDistancer: Api/plugin/cpp/LineDistancer.hpp.

   Groups, in the order of the declarations below:
   1. scalars: sqr, round_up_divide, sort_remove_duplicates;
   2. Eigen vector aliases (the same types as the native ones) and conversions between c_point and Eigen vectors;
   3. integer c_point products and the hash of a c_point;
   4. bounding boxes of polygon collections;
   5. polygon collections: area, length, closed polylines, line segments, flat polygons of expolygons;
   6. point_in_polygons: where a point lies relative to a set of rings (the border counts as inside);
   7. nearest_on_lines: the closest point of a set of line segments;
   8. spatial_order: the order the native AABB tree gives to its input;
   9. Clipper wrappers on storage-owned results;
   10. with_scratch_storage: a storage that lives for the duration of one computation, and the move of a collection to another storage;
   11. make_circle and move_inside_if_outside.

   Clipper results (rings, expolygons) are not ordered like the native ones, and their rings start at other points: compare them
   as regions, never with operator==.

   Storage: a function that returns a collection takes the storage_handle that will own it. The inputs are read-only views (a
   Stored* lvalue converts to its view), except the source of moved_to_storage(), and the result must not outlive the storage.

       StoredPolygonCollection free_area = diff_clipped(storage, increased_area, collision);
       StoredExPolygonCollection islands = expolygons_union(storage, free_area);

   The join type of the Clipper wrappers is CLIPPER_JOIN_*, and `miter_limit` is the miter limit for CLIPPER_JOIN_MITER and
   CLIPPER_JOIN_SQUARE and the arc tolerance (scaled units) for CLIPPER_JOIN_ROUND, as for ClipperUtils::offset(). */
#ifndef plugins_support_tree_geometry_hpp_
#define plugins_support_tree_geometry_hpp_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/LineDistancer.hpp"
#include "libslic3r/Plugins/Support/Common/ClipperResults.hpp"

namespace Slic3r::TreeFamily {

/* ============================== 1. scalars ============================== */

// The square of a number.
constexpr inline double sqr(double x) { return x * x; }
constexpr inline float sqr(float x) { return x * x; }

// dividend / divisor rounded up, for positive numbers.
template<typename INDEX_TYPE> inline INDEX_TYPE round_up_divide(const INDEX_TYPE dividend, const INDEX_TYPE divisor)
{
    return (dividend + divisor - 1) / divisor;
}

// Sort a vector and drop the repeated values.
template<typename T> inline void sort_remove_duplicates(std::vector<T> &vec)
{
    std::sort(vec.begin(), vec.end());
    vec.erase(std::unique(vec.begin(), vec.end()), vec.end());
}

/* ============================== 2. Eigen vectors and c_point ============================== */

// The same types as the native Slic3r::Vec2d, Vec3d and Vec3f.
using Vec2d = Eigen::Matrix<double, 2, 1, Eigen::DontAlign>;
using Vec3d = Eigen::Matrix<double, 3, 1, Eigen::DontAlign>;
using Vec3f = Eigen::Matrix<float, 3, 1, Eigen::DontAlign>;

// The arithmetic of c_point (GeometryViews.hpp): c_point has no namespace of its own, so the operators are brought in by hand.
// These declarations hide the operators of Slic3r (and of the global namespace) for the lookups that are not argument dependent. The
// operands of the native operators (Point, Polygon, ExPolygon, BoundingBox, Line, the config options, Eigen vectors) all have Slic3r or
// Eigen as an associated namespace, so argument dependent lookup still finds their operators: a native Point == and a c_point ==
// work in the same function (checked by the test "tree geometry operators coexist with the native ones"). The one expression that
// would not compile is one whose operands belong to no namespace of Slic3r or Eigen and whose operator is declared in Slic3r only.
using slic3r_api::operator+;
using slic3r_api::operator-;
using slic3r_api::operator*;
using slic3r_api::operator/;
using slic3r_api::operator==;
using slic3r_api::operator!=;

// The point as a vector of doubles, like Point::cast<double>().
inline Vec2d to_vec2d(c_point point) { return Vec2d(double(point.x), double(point.y)); }
// The vector with its coordinates cut towards zero, like Vec2d::cast<coord_t>().
inline c_point truncated_point(const Vec2d &vec) { return c_point{ coord_t(vec.x()), coord_t(vec.y()) }; }
// A point from a vector in millimeters, like Point::new_scale().
inline c_point scaled_point(const Vec2d &vec_mm) { return c_point{ scale_i(vec_mm.x()), scale_i(vec_mm.y()) }; }
// The point in millimeters, like unscale_p().
inline Vec2d unscale_p(c_point point) { return Vec2d(unscaled(point.x), unscaled(point.y)); }

/* ============================== 3. c_point products and hash ============================== */

// The dot product and the squared length in 64 bits integers, like Point::cast<int64_t>().dot() and squaredNorm(). The products and
// their sum must fit in int64 (coordinates below about 2^30 scaled units, 1000 mm): a signed overflow is undefined, it does not wrap.
inline int64_t dot(c_point lhs, c_point rhs) { return int64_t(lhs.x) * int64_t(rhs.x) + int64_t(lhs.y) * int64_t(rhs.y); }
inline int64_t squared_norm(c_point point) { return dot(point, point); }

// The hash of a c_point (same value as the native PointHash: points closer than 64 units can collide, on purpose) and the matching
// equality, for std::unordered_set<c_point, PointHash, PointEqual>.
struct PointHash
{
    size_t operator()(const c_point &pt) const noexcept { return coord_t((89 * 31 + (int64_t(pt.x) >> 6)) * 31 + (pt.y >> 6)); }
};
struct PointEqual
{
    bool operator()(const c_point &lhs, const c_point &rhs) const noexcept { return lhs.x == rhs.x && lhs.y == rhs.y; }
};

/* ============================== 4. bounding boxes ============================== */

// The box around every point of every ring. An empty collection gives an empty box (min > max, see bounding_box_empty()), which
// bounding_box_merge() fills with the first box it receives.
c_bounding_box polygons_bounding_box(const slic3r_api::PolygonCollection &polygons);
// The box grown by delta on every side, like BoundingBox::inflated(). An empty box stays empty.
c_bounding_box bounding_box_inflated(const c_bounding_box &box, coord_t delta);

/* ============================== 5. polygon collections ============================== */

// The sum of the signed areas of the rings, like area(Polygons): counter-clockwise rings count positive, clockwise ones negative.
double polygons_area(const slic3r_api::PolygonCollection &polygons);
// The sum of the perimeters of the rings, closing edge included, like total_length(Polygons).
double total_length(const slic3r_api::PolygonCollection &polygons);
// Each ring as a closed polyline (its first point repeated at the end), like to_polylines(Polygons). A ring without points gives
// no polyline.
slic3r_api::StoredPolylineCollection to_polylines(storage_handle *storage, const slic3r_api::PolygonCollection &polygons);

// A straight line between two scaled points.
struct LineSegment
{
    c_point a;
    c_point b;
};
// Every edge of every ring, closing edge included, ring after ring, like to_lines(Polygons). A ring under 2 points gives no line.
std::vector<LineSegment> to_lines(const slic3r_api::PolygonCollection &polygons);

// The contour then the holes of every expolygon, in a flat list, like to_polygons(ExPolygons).
slic3r_api::StoredPolygonCollection to_polygons(storage_handle *storage, const slic3r_api::ExPolygonCollection &expolygons);

/* ============================== 6. point in polygons ============================== */

// Whether a point is inside a set of rings: inside an odd number of rings (so a hole ring, inside its contour, puts the points
// in it outside), counting the rings with the crossing test of ClipperLib::PointInPolygon. A point on the border of any ring gives
// border_result at once, like the native contains(Polygons, Point, border_result); a ring under 3 points contains nothing.
bool point_in_polygons(const slic3r_api::PolygonCollection &polygons, c_point point, bool border_result = true);

/* ============================== 7. nearest point on lines ============================== */

// The closest point found by nearest_on_lines().
struct LineHit
{
    // Index in the vector of lines.
    size_t line_index = 0;
    // The closest point of that line to the query point (scaled units).
    Vec2d point = Vec2d::Zero();
    // The squared distance from the query point to the hit point (scaled units squared).
    double squared_distance = 0.;
};

// The line of `lines` closest to `from`, and the closest point on it, when its squared distance is strictly under max_squared_distance;
// nothing otherwise. Everything is in scaled units (`from`, the hit point, the distances): a caller that works in millimeters
// (TreeSmoothing) scales the query point and the limit, and unscales the hit point.
//
// Like AABBTreeLines::squared_distance_to_indexed_lines(), but on an exact tie the first line of the vector wins, where the native tree
// keeps the line its traversal meets first (it prefers the right child when the boxes are as far). A tie changes line_index when the
// nearest point is a vertex shared by two lines (same point, same distance), but it changes the POINT when several lines are exactly as
// close at different points: the centre of a square is as close to its four sides, and the hit point is the middle of the first side
// here, of another side natively (test "nearest_on_lines on an exact tie"). Its caller, push_out_of_outline() in TreeSmoothing.cpp (shared by
// smooth_organic_branches() and push_sphere_out_of_outline()), pushes the sphere away from that point, so the push direction may differ on
// such a tie. Measured on the dump, organic and family scenes (about 4000 real hits): no tie met, no hit differs.
//
// `index` must hold the same segments as `lines`, appended in the same order (LineDistancer::append_segment(), then build_index()
// before calling this function from several threads). It is only used to skip the lines that are too far: the distances are
// computed here in doubles, so a query point between two integer positions is not rounded.
std::optional<LineHit> nearest_on_lines(const slic3r_api::LineDistancer &index, const std::vector<LineSegment> &lines, const Vec2d &from,
                                        double max_squared_distance = std::numeric_limits<double>::infinity());

/* ============================== 8. spatial order ============================== */

/* Put the items in the order the native AABB tree gives to its input (AABBTreeIndirect::Tree::build_modify_input()): the items are
   split in two halves, along the longest side of their bounding box, by a quickselect on the centroids; each half is split again.
   Neighbouring items end up next to each other, so merging them by consecutive groups merges the closest ones first. The
   algorithm and the swaps are those of the native tree, so equal inputs give the same order; the tree itself is not built.

   BoxOf is called as box_of(item) and returns the c_bounding_box of the item (the native code gives it the box already grown by
   SCALED_EPSILON). The centroid used for the split is (min + max) / 2 of that box. */
template<typename Item, typename BoxOf> void spatial_order(std::vector<Item> &items, const BoxOf &box_of);

namespace detail {
// The coordinate of the centroid of a box on an axis (0: x, 1: y), with the integer division of Point / 2.
inline coord_t box_centroid(const c_bounding_box &box, size_t dimension)
{
    return dimension == 0 ? (box.min.x + box.max.x) / 2 : (box.min.y + box.max.y) / 2;
}

// Quickselect on items[left..right] (both included): after the call, the items left of the k-th are not above it on the axis
// and the items right of it are not below it. The same algorithm as AABBTreeIndirect::Tree::partition_input().
template<typename Item, typename BoxOf>
void spatial_partition(std::vector<Item> &items, const BoxOf &box_of, size_t dimension, size_t left, size_t right, const size_t k)
{
    while (left < right) {
        const size_t center = (left + right) / 2;
        coord_t pivot;
        {
            // Sort items[left], items[center], items[right] so that the median of the three ends up in items[center].
            coord_t left_value = box_centroid(box_of(items[left]), dimension);
            coord_t center_value = box_centroid(box_of(items[center]), dimension);
            coord_t right_value = box_centroid(box_of(items[right]), dimension);
            if (left_value > center_value) {
                std::swap(items[left], items[center]);
                std::swap(left_value, center_value);
            }
            if (left_value > right_value) {
                std::swap(items[left], items[right]);
                right_value = left_value;
            }
            if (center_value > right_value) {
                std::swap(items[center], items[right]);
                center_value = right_value;
            }
            pivot = center_value;
        }
        if (right <= left + 2)
            // The interval is sorted.
            break;
        size_t i = left;
        size_t j = right - 1;
        std::swap(items[center], items[j]);
        // Partition around the pivot; the scan on the left stops at worst at right - 1, where the pivot is.
        for (;;) {
            while (box_centroid(box_of(items[++i]), dimension) < pivot) {}
            while (box_centroid(box_of(items[--j]), dimension) > pivot && i < j) {}
            if (i >= j)
                break;
            std::swap(items[i], items[j]);
        }
        // Restore the pivot to the border of the two sides.
        std::swap(items[i], items[right - 1]);
        // Continue on the side of the k-th item.
        if (k < i)
            right = i - 1;
        else if (k == i)
            break;
        else
            left = i + 1;
    }
}

// Order items[left..right] (both included): split along the longest side of their box, then order each half.
template<typename Item, typename BoxOf> void spatial_order_range(std::vector<Item> &items, const BoxOf &box_of, size_t left, size_t right)
{
    if (left == right)
        return;
    c_bounding_box box = box_of(items[left]);
    for (size_t i = left + 1; i <= right; ++i) {
        const c_bounding_box other = box_of(items[i]);
        box.min.x = std::min(box.min.x, other.min.x);
        box.min.y = std::min(box.min.y, other.min.y);
        box.max.x = std::max(box.max.x, other.max.x);
        box.max.y = std::max(box.max.y, other.max.y);
    }
    // The first axis wins a tie.
    const size_t dimension = (box.max.y - box.min.y) > (box.max.x - box.min.x) ? 1 : 0;
    const size_t center = (left + right) / 2;
    spatial_partition(items, box_of, dimension, left, right, center);
    spatial_order_range(items, box_of, left, center);
    spatial_order_range(items, box_of, center + 1, right);
}
} // namespace detail

template<typename Item, typename BoxOf> void spatial_order(std::vector<Item> &items, const BoxOf &box_of)
{
    if (!items.empty())
        detail::spatial_order_range(items, box_of, 0, items.size() - 1);
}

/* ============================== 9. Clipper wrappers ============================== */

/* The inputs of the templates are polygon or expolygon collections (views or Stored* lvalues): whatever ClipperContext accepts.
   Polygon results are the flat rings of the operation; the *_ex results are filtered like the native *_ex functions
   (Support/Common/ClipperResults.hpp). */

// The union of the shapes, the union of two sets of shapes, the difference subject - clip, and the intersection, as rings.
template<typename Shape> slic3r_api::StoredPolygonCollection polygons_union(storage_handle *storage, const Shape &subject)
{
    const slic3r_api::ClipperContext clipper(storage);
    return slic3r_api::clipper_union(clipper(subject)).to_polygon_collection();
}
template<typename Shape, typename OtherShape>
slic3r_api::StoredPolygonCollection polygons_union(storage_handle *storage, const Shape &subject, const OtherShape &other)
{
    const slic3r_api::ClipperContext clipper(storage);
    return slic3r_api::clipper_union2(clipper(subject), clipper(other)).to_polygon_collection();
}
template<typename Shape, typename ClipShape>
slic3r_api::StoredPolygonCollection polygons_diff(storage_handle *storage, const Shape &subject, const ClipShape &clip)
{
    const slic3r_api::ClipperContext clipper(storage);
    return slic3r_api::clipper_diff(clipper(subject), clipper(clip)).to_polygon_collection();
}
template<typename Shape, typename ClipShape>
slic3r_api::StoredPolygonCollection polygons_intersection(storage_handle *storage, const Shape &subject, const ClipShape &clip)
{
    const slic3r_api::ClipperContext clipper(storage);
    return slic3r_api::clipper_intersection(clipper(subject), clipper(clip)).to_polygon_collection();
}
/* The rings grown by delta (shrunk when negative; delta must not be 0), as rings, like the native offset(Polygons). The native function
   offsets every ring alone and unites the results; the API offsets a flat ring list together or, for valid expolygons, ring by ring.
   The rule applied here, with valid API input only:
   - no clockwise ring and delta > 0: the rings go to the API as they are, and the API unites them. This is the native result for any
     counter-clockwise rings, overlapping or touching ones included;
   - otherwise the rings are united (non-zero fill); the union, whose rings are normalized (valid expolygons), is offset ring by ring,
     like the native offset_ex().
   Supported: (1) a list with no clockwise ring and delta > 0; (2) any list that already is a valid non-zero region (winding +1 in the
   material, 0 in the holes, no crossing rings, holes disjoint and strictly inside their contour, as the output of a union, a
   difference, an intersection, an offset or the slicing is), with any sign of delta: the union then changes nothing.
   Not supported: a clockwise ring outside every contour (any delta); overlapping or touching rings, crossing rings or overlapping holes
   when delta < 0 or when a clockwise ring is present (the union merges what the native function offsets ring by ring). The tests
   measure these differences without comparing them.
   A ring under 3 points is ignored, as ClipperOffset does. Approximately so for a ring that ClipperOffset collapses because all its
   edges are shorter than its shortest edge length (0.005 x |delta|): the flat route drops it, the union route keeps it. */
slic3r_api::StoredPolygonCollection polygons_offset(storage_handle *storage, const slic3r_api::PolygonCollection &polygons, double delta,
                                                    clipper_join_type_t join_type = CLIPPER_JOIN_MITER, double miter_limit = 3.0);
// The expolygons grown by delta, as rings, like offset(ExPolygons): the API offsets the contour and the holes of an expolygon alone.
// delta may be 0; the expolygons must not overlap when delta < 0, as for the native offset(ExPolygons).
slic3r_api::StoredPolygonCollection polygons_offset(storage_handle *storage, const slic3r_api::ExPolygonCollection &expolygons, double delta,
                                                    clipper_join_type_t join_type = CLIPPER_JOIN_MITER, double miter_limit = 3.0);
// The same operations with an expolygon result (contours and holes grouped), like union_ex(), diff_ex() and offset_ex().
template<typename Shape> slic3r_api::StoredExPolygonCollection expolygons_union(storage_handle *storage, const Shape &subject)
{
    const slic3r_api::ClipperContext clipper(storage);
    return slic3r_api::Support::Common::native_expolygons(slic3r_api::clipper_union(clipper(subject)));
}
template<typename Shape, typename ClipShape>
slic3r_api::StoredExPolygonCollection expolygons_diff(storage_handle *storage, const Shape &subject, const ClipShape &clip)
{
    const slic3r_api::ClipperContext clipper(storage);
    return slic3r_api::Support::Common::native_expolygons(slic3r_api::clipper_diff(clipper(subject), clipper(clip)));
}
// The expolygons grown by delta, like offset_ex(ExPolygons); delta may be 0. No small area is dropped, as in offset_ex() (unlike the
// *_ex results of union and diff, which are filtered like union_ex() and diff_ex()).
slic3r_api::StoredExPolygonCollection expolygons_offset(storage_handle *storage, const slic3r_api::ExPolygonCollection &expolygons, double delta,
                                                        clipper_join_type_t join_type = CLIPPER_JOIN_MITER, double miter_limit = 3.0);

// The rings of `polygons` that can touch the box, like ClipperUtils::clip_clipper_polygons_with_subject_bbox(): a flat ring list for
// the next boolean operation, not a final region.
slic3r_api::StoredPolygonCollection clip_polygons_with_subject_bbox(storage_handle *storage, const slic3r_api::PolygonCollection &polygons,
                                                                    const c_bounding_box &box);
// subject - clip, where only the rings of clip near the bounding box of subject are used (faster for a small subject and a large
// clip), like diff_clipped().
slic3r_api::StoredPolygonCollection diff_clipped(storage_handle *storage, const slic3r_api::PolygonCollection &subject,
                                                 const slic3r_api::PolygonCollection &clip);

/* ============================== 10. scratch storage ============================== */

// Runs work once, with a scratch storage that is destroyed when work returns, so that the temporary collections of a computation
// do not stay in the run storage. Everything allocated from that storage must be destroyed, or moved to another storage
// (StoredPolygonCollection::move_from), before work returns. An exception thrown by work propagates to the caller.
void with_scratch_storage(const std::function<void(storage_handle *)> &work);

// A new collection of storage that takes the rings of source, which belongs to another storage (typically a scratch one). Unlike
// the other functions here, it modifies its input: source is moved from and stays in its own storage, valid and empty. The result is
// owned by storage and lives as long as it; an allocation failure propagates. A copy is source.readonly().clone(storage).
slic3r_api::StoredPolygonCollection moved_to_storage(storage_handle *storage, slic3r_api::StoredPolygonCollection &source);

/* ============================== 11. circles and move inside ============================== */

// The points of a regular polygon centred on the origin, like the native make_circle(): `error` is the largest distance between a side
// and the true circle (scaled units); it is turned into a number of sides before any point is made. The points go counter-clockwise,
// starting at angle 0; their coordinates are truncated towards zero, so for a fractional radius the first point is (trunc(radius), 0),
// not exactly (radius, 0). Needs radius > 0 and 0 < error < 2 * radius.
std::vector<c_point> make_circle(double radius, double error);

// The point itself when it lies inside the rings (point_in_polygons(), border inside), else the closest point of the border of the rings
// (the foot of the point on an edge, or a vertex), like the native move_inside_if_outside() with a distance of 0 and no limit.
// The foot is cut to integers, so it approximates the border and may lie slightly outside the rings. Coordinates and their differences
// must stay below about 2^30 scaled units (1000 mm): the squared distances are int64. With no candidate (no ring of 2 points or more),
// `from` is returned unchanged. The first ring and the first edge win an exact distance tie, as natively; the rings of a Clipper
// output are not in the order of the native one, so on a tie between two rings at different points the result may differ from the
// native one (none occurs on the tree dump scenes; see the tests "move_inside_if_outside ...").
c_point move_inside_if_outside(const slic3r_api::PolygonCollection &polygons, c_point from);

} // namespace Slic3r::TreeFamily

#endif // plugins_support_tree_geometry_hpp_
