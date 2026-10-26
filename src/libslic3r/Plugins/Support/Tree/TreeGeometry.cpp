///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#include "TreeGeometry.hpp"

#include <cassert>
#include <cmath>

#include "libslic3r/Api/plugin/c/slic3r_utils.h"
#include "libslic3r/Api/plugin/cpp/DeferredCallbackException.hpp"

/* Implementation of the geometry helpers of the tree support family (see TreeGeometry.hpp).

   Most helpers read the rings of a polygon collection through multipoint_const_view, a pointer and a count over the points of a
   ring: no copy of the points is made. The two algorithms ported from the native code are point_in_polygons() (the crossing test of
   ClipperLib::PointInPolygon) and nearest_on_lines() (the closest point of a segment, as line_alg::distance_to_squared()).
   The Clipper wrappers build their operands in the storage they are given and return collections owned by that storage. */

namespace Slic3r::TreeFamily {

namespace {
// The points of a regular polygon of num_segments sides centred on the origin.
std::vector<c_point> make_circle_num_segments(double radius, size_t num_segments);
// Where a point lies relative to one ring: 0 outside, 1 inside, -1 on the border (the test of ClipperLib::PointInPolygon).
int ring_crossing(const multipoint_const_view &ring, c_point point);
// The squared distance from a point to the segment a-b; the closest point of the segment is written to `closest`.
double squared_distance_to_segment(const LineSegment &segment, const Vec2d &point, Vec2d &closest);
}

// Merges the points of every ring into one box, starting from the empty box.
c_bounding_box polygons_bounding_box(const slic3r_api::PolygonCollection &polygons)
{
    c_bounding_box box = slic3r_api::bounding_box_empty();
    for (uint32_t ring_idx = 0; ring_idx < polygons.size(); ++ring_idx) {
        const multipoint_const_view ring = polygons.at(ring_idx).view();
        for (uint32_t point_idx = 0; point_idx < ring.size; ++point_idx)
            slic3r_api::bounding_box_merge(box, ring.array[point_idx]);
    }
    return box;
}

// Moves the four sides out by delta; an empty box (min > max) is returned as it is, so it stays empty.
c_bounding_box bounding_box_inflated(const c_bounding_box &box, coord_t delta)
{
    if (!slic3r_api::bounding_box_valid(box))
        return box;
    return c_bounding_box{ c_point{ box.min.x - delta, box.min.y - delta }, c_point{ box.max.x + delta, box.max.y + delta } };
}

// Adds up the signed area of each ring, as given by the host.
double polygons_area(const slic3r_api::PolygonCollection &polygons)
{
    double total = 0.;
    for (uint32_t ring_idx = 0; ring_idx < polygons.size(); ++ring_idx)
        total += polygons.at(ring_idx).area();
    return total;
}

// Adds up the perimeter of each ring (a ring under 2 points has none), reading the points in place.
double total_length(const slic3r_api::PolygonCollection &polygons)
{
    double total = 0.;
    for (uint32_t ring_idx = 0; ring_idx < polygons.size(); ++ring_idx) {
        const multipoint_const_view ring = polygons.at(ring_idx).view();
        if (ring.size < 2)
            continue;
        // The closing edge first, then the edges in order, so the sum is the one of the native Polygon::length().
        double length = slic3r_api::norm(ring.array[ring.size - 1] - ring.array[0]);
        for (uint32_t point_idx = 1; point_idx < ring.size; ++point_idx)
            length += slic3r_api::norm(ring.array[point_idx] - ring.array[point_idx - 1]);
        total += length;
    }
    return total;
}

// One new polyline per non-empty ring, built in the storage and moved into the returned collection (no second copy).
slic3r_api::StoredPolylineCollection to_polylines(storage_handle *storage, const slic3r_api::PolygonCollection &polygons)
{
    slic3r_api::StoredPolylineCollection polylines(storage);
    for (uint32_t ring_idx = 0; ring_idx < polygons.size(); ++ring_idx) {
        const multipoint_const_view ring = polygons.at(ring_idx).view();
        if (ring.size == 0)
            continue;
        slic3r_api::StoredPolyline polyline(storage);
        for (uint32_t point_idx = 0; point_idx < ring.size; ++point_idx)
            polyline.push_back(ring.array[point_idx]);
        // A polyline has no closing edge: repeat the first point to close the ring.
        polyline.push_back(ring.array[0]);
        polylines.push_back_move(std::move(polyline));
    }
    return polylines;
}

// The edges of a ring in order, then its closing edge; the result is a plain vector, it needs no storage.
std::vector<LineSegment> to_lines(const slic3r_api::PolygonCollection &polygons)
{
    std::vector<LineSegment> lines;
    for (uint32_t ring_idx = 0; ring_idx < polygons.size(); ++ring_idx) {
        const multipoint_const_view ring = polygons.at(ring_idx).view();
        if (ring.size < 2)
            continue;
        for (uint32_t point_idx = 0; point_idx + 1 < ring.size; ++point_idx)
            lines.push_back(LineSegment{ ring.array[point_idx], ring.array[point_idx + 1] });
        lines.push_back(LineSegment{ ring.array[ring.size - 1], ring.array[0] });
    }
    return lines;
}

// The host fills a new collection of the storage with the contour then the holes of each expolygon.
slic3r_api::StoredPolygonCollection to_polygons(storage_handle *storage, const slic3r_api::ExPolygonCollection &expolygons)
{
    slic3r_api::StoredPolygonCollection polygons(storage);
    ::expolygons_to_polygons(expolygons.handle(), polygons.mutable_handle());
    return polygons;
}

// Counts the rings that contain the point; the first ring that has the point on its border decides the answer.
bool point_in_polygons(const slic3r_api::PolygonCollection &polygons, c_point point, bool border_result)
{
    int rings_containing_point = 0;
    for (uint32_t ring_idx = 0; ring_idx < polygons.size(); ++ring_idx) {
        const int crossing = ring_crossing(polygons.at(ring_idx).view(), point);
        if (crossing == -1)
            return border_result;
        rings_containing_point += crossing;
    }
    return (rings_containing_point % 2) == 1;
}

// The index gives the lines near the point, in the order of `lines`; the exact distance to each is computed in doubles and the first
// strictly closest one wins. Nothing is allocated in a storage: the result is a plain value.
std::optional<LineHit> nearest_on_lines(const slic3r_api::LineDistancer &index, const std::vector<LineSegment> &lines, const Vec2d &from,
                                        double max_squared_distance)
{
    // The index works on integer points: query it with the rounded point and a radius larger by more than the rounding (at most
    // 0.71 units in each direction), so it never drops a line that is within the limit of the exact point.
    const double radius = std::sqrt(max_squared_distance) + 1.5;
    const c_point rounded{ coord_t(std::llround(from.x())), coord_t(std::llround(from.y())) };
    std::optional<LineHit> best;
    double best_squared_distance = max_squared_distance;
    // The candidates come in the order of the lines, and a line only replaces the best one when it is strictly closer.
    for (const size_t line_index : index.all_lines_in_radius(rounded, radius)) {
        Vec2d closest;
        const double squared_distance = squared_distance_to_segment(lines[line_index], from, closest);
        if (squared_distance < best_squared_distance) {
            best_squared_distance = squared_distance;
            best = LineHit{ line_index, closest, squared_distance };
        }
    }
    return best;
}

/* The rule of the header: the rings go to the API as they are for a positive delta and no clockwise ring; otherwise their non-zero union
   (a Clipper result whose rings are normalized, so valid expolygons for the API) is offset ring by ring. The operands adapt the input
   without copying it; ClipperOffset and Clipper ignore the closed rings under 3 points themselves. */
slic3r_api::StoredPolygonCollection polygons_offset(storage_handle *storage, const slic3r_api::PolygonCollection &polygons, double delta,
                                                    clipper_join_type_t join_type, double miter_limit)
{
    assert(delta != 0.);
    const slic3r_api::ClipperContext clipper(storage);
    // The orientation test of ClipperLib::Orientation(), exact on integers; a ring under 3 points is no area. A negative delta takes
    // the union route anyway, so the rings are not scanned then.
    bool has_clockwise_ring = false;
    for (uint32_t ring_idx = 0; delta > 0. && ring_idx < polygons.size() && !has_clockwise_ring; ++ring_idx) {
        const slic3r_api::Polygon ring = polygons.at(ring_idx);
        has_clockwise_ring = ring.size() >= 3 && ring.is_clockwise();
    }
    if (delta > 0. && !has_clockwise_ring)
        return slic3r_api::clipper_offset(clipper(polygons), delta, join_type, miter_limit).to_polygon_collection();
    return slic3r_api::clipper_offset(slic3r_api::clipper_union(clipper(polygons)), delta, join_type, miter_limit).to_polygon_collection();
}

// The API offsets the contour and the holes of each expolygon alone; the result is the same collection as the polygons one.
slic3r_api::StoredPolygonCollection polygons_offset(storage_handle *storage, const slic3r_api::ExPolygonCollection &expolygons, double delta,
                                                    clipper_join_type_t join_type, double miter_limit)
{
    const slic3r_api::ClipperContext clipper(storage);
    return slic3r_api::clipper_offset(clipper(expolygons), delta, join_type, miter_limit).to_polygon_collection();
}

// The conversion to expolygons is the one of the native offset_ex(): no small area is removed (native_expolygons() would, as the
// native union_ex() and diff_ex() do, but offset_ex() does not).
slic3r_api::StoredExPolygonCollection expolygons_offset(storage_handle *storage, const slic3r_api::ExPolygonCollection &expolygons, double delta,
                                                        clipper_join_type_t join_type, double miter_limit)
{
    const slic3r_api::ClipperContext clipper(storage);
    return slic3r_api::clipper_offset(clipper(expolygons), delta, join_type, miter_limit).to_expolygon_collection();
}

// The operand adapts the polygons without copying them; the clipped rings are materialized in the storage.
slic3r_api::StoredPolygonCollection clip_polygons_with_subject_bbox(storage_handle *storage, const slic3r_api::PolygonCollection &polygons,
                                                                    const c_bounding_box &box)
{
    const slic3r_api::ClipperContext clipper(storage);
    return slic3r_api::clipper_clip_shapes_with_subject_bbox(clipper(polygons), box).to_polygon_collection();
}

// The clip is trimmed to the box of the subject inflated by SCALED_EPSILON, then subtracted; an empty subject gives an empty result.
slic3r_api::StoredPolygonCollection diff_clipped(storage_handle *storage, const slic3r_api::PolygonCollection &subject,
                                                 const slic3r_api::PolygonCollection &clip)
{
    if (subject.empty())
        return slic3r_api::StoredPolygonCollection(storage);
    // Only the rings of the clip near the subject can change the result: a clip trimmed to the subject box (plus the epsilon
    // of the native code) is as good and faster to subtract.
    const c_bounding_box box = bounding_box_inflated(polygons_bounding_box(subject), SCALED_EPSILON);
    const slic3r_api::ClipperContext clipper(storage);
    const slic3r_api::ClipperOperand trimmed_clip = slic3r_api::clipper_clip_shapes_with_subject_bbox(clipper(clip), box);
    return slic3r_api::clipper_diff(clipper(subject), trimmed_clip).to_polygon_collection();
}

// A job of one index: the host builds the scratch storage, runs the job on this thread (or a worker) and destroys the storage.
// slic3r_parallel_for_storage is an extern "C" host function, so no exception may leave the job: it is caught, kept, and rethrown here
// once the host call has returned. A cancellation raised in a parallel loop nested in `work` is deferred the same way until that loop joins.
void with_scratch_storage(const std::function<void(storage_handle *)> &work)
{
    struct Job {
        const std::function<void(storage_handle *)> &work;
        slic3r_api::DeferredCallbackException failure;
    } job{ work, {} };
    slic3r_parallel_for_storage(0, 1, &job,
        [](uint32_t, storage_handle *scratch_storage, void *user_data) {
            Job &job = *static_cast<Job *>(user_data);
            try {
                job.work(scratch_storage);
            } catch (...) {
                job.failure.capture_current();
            }
        });
    job.failure.rethrow_if_captured();
}
// Allocates the result in storage and moves the content of source into it, so that source stays valid and empty in its own storage.
// The result lives as long as storage. An allocation failure propagates to the caller.
slic3r_api::StoredPolygonCollection moved_to_storage(storage_handle *storage, slic3r_api::StoredPolygonCollection &source)
{
    slic3r_api::StoredPolygonCollection out(storage);
    out.move_from(source);
    return out;
}

// The sides are counted from the angle one side must span for the sagitta (radius - radius * cos(angle / 2)) to stay under `error`;
// that count is computed in doubles before the points are made. Needs radius > 0 and 0 < error < 2 * radius (acos of a value in
// [-1, 1)); the native function has the same domain.
std::vector<c_point> make_circle(double radius, double error)
{
    const double angle = 2. * acos(1. - error / radius);
    const size_t num_segments = size_t(ceil(2. * PI / angle));
    return make_circle_num_segments(radius, num_segments);
}

// For each ring in order, for each edge a-b in the order of the ring (the edge that closes the ring comes first), the closest point of the
// edge to `from` is looked for: the foot of `from` when its projection falls inside the edge, the vertex a when it falls before the
// start of the edge and the previous edge's projection fell beyond its end (a reflex corner seen from outside, the case "> ."), nothing
// otherwise. A candidate replaces the best one only when its distance is strictly smaller, so the first ring and the first edge win an
// exact tie. Same algorithm as the native move_inside() with a distance of 0 and no distance limit. When no ring gives a candidate
// (no ring of 2 points or more), `from` is returned unchanged.
// The products of the squared distances are int64: coordinates (and differences) must stay below about 2^30 scaled units (1000 mm).
// The foot is cut towards zero to integers, so it only approximates the border and may lie slightly outside the rings.
c_point move_inside_if_outside(const slic3r_api::PolygonCollection &polygons, c_point from)
{
    if (point_in_polygons(polygons, from))
        return from;
    c_point best_point = from;
    double best_distance2 = std::numeric_limits<double>::max();
    for (uint32_t ring_idx = 0; ring_idx < polygons.size(); ++ring_idx) {
        const multipoint_const_view ring = polygons.at(ring_idx).view();
        if (ring.size < 2)
            continue;
        c_point p1 = ring.array[ring.size - 1];
        // Whether `from` projects beyond the end of the closing edge (the one before the first edge), compared with squared lengths to avoid dividing.
        const c_point closing_edge = p1 - ring.array[ring.size - 2];
        bool projected_beyond_prev_edge = dot(closing_edge, from - ring.array[ring.size - 2]) >= squared_norm(closing_edge);
        for (uint32_t point_idx = 0; point_idx < ring.size; ++point_idx) {
            const c_point p2 = ring.array[point_idx];
            const c_point a = p1;
            const c_point ab = p2 - a;
            const int64_t ab_length2 = squared_norm(ab);
            if (ab_length2 <= 0) {
                // Two equal adjacent points: skip only one of them.
                p1 = p2;
                continue;
            }
            const int64_t dot_prod = dot(ab, from - a);
            if (dot_prod <= 0) {
                // `from` projects before the edge.
                if (projected_beyond_prev_edge) {
                    projected_beyond_prev_edge = false;
                    const int64_t distance2 = squared_norm(a - from);
                    if (distance2 < best_distance2) {
                        best_distance2 = distance2;
                        best_point = a;
                    }
                }
            } else if (dot_prod >= ab_length2) {
                // `from` projects beyond the edge.
                projected_beyond_prev_edge = true;
            } else {
                // `from` projects inside the edge: the foot point, cut towards zero like the native cast.
                projected_beyond_prev_edge = false;
                const c_point foot = a + truncated_point(to_vec2d(ab) * (double(dot_prod) / double(ab_length2)));
                const int64_t distance2 = squared_norm(from - foot);
                if (distance2 < best_distance2) {
                    best_distance2 = distance2;
                    best_point = foot;
                }
            }
            p1 = p2;
        }
    }
    return best_point;
}
namespace {
// Point i is at the angle 2 pi i / num_segments; the coordinates are cut towards zero like the native make_circle_num_segments(), so
// the first point is (trunc(radius), 0) and not exactly (radius, 0) for a fractional radius.
std::vector<c_point> make_circle_num_segments(double radius, size_t num_segments)
{
    std::vector<c_point> out;
    out.reserve(num_segments);
    const double angle_inc = 2. * PI / num_segments;
    for (size_t i = 0; i < num_segments; ++i) {
        const double angle = angle_inc * i;
        out.push_back(c_point{ coord_t(cos(angle) * radius), coord_t(sin(angle) * radius) });
    }
    return out;
}

int ring_crossing(const multipoint_const_view &ring, c_point point)
{
    // See "The Point in Polygon Problem for Arbitrary Polygons" by Hormann & Agathos. The result flips each time the ray going
    // right from the point crosses an edge; a point on an edge or a vertex is on the border.
    int result = 0;
    const size_t count = ring.size;
    if (count < 3)
        return 0;
    c_point current = ring.array[0];
    for (size_t i = 1; i <= count; ++i) {
        const c_point next = (i == count ? ring.array[0] : ring.array[i]);
        if (next.y == point.y && (next.x == point.x || (current.y == point.y && ((next.x > point.x) == (current.x < point.x)))))
            return -1;
        if ((current.y < point.y) != (next.y < point.y)) {
            if (current.x >= point.x) {
                if (next.x > point.x)
                    result = 1 - result;
                else {
                    const double cross = double(current.x - point.x) * double(next.y - point.y) - double(next.x - point.x) * double(current.y - point.y);
                    if (cross == 0.)
                        return -1;
                    if ((cross > 0) == (next.y > current.y))
                        result = 1 - result;
                }
            } else if (next.x > point.x) {
                const double cross = double(current.x - point.x) * double(next.y - point.y) - double(next.x - point.x) * double(current.y - point.y);
                if (cross == 0.)
                    return -1;
                if ((cross > 0) == (next.y > current.y))
                    result = 1 - result;
            }
        }
        current = next;
    }
    return result;
}

double squared_distance_to_segment(const LineSegment &segment, const Vec2d &point, Vec2d &closest)
{
    const Vec2d a = to_vec2d(segment.a);
    const Vec2d b = to_vec2d(segment.b);
    const Vec2d v = b - a;
    const Vec2d va = point - a;
    const double length_squared = v.squaredNorm();
    if (length_squared == 0.) {
        // a == b
        closest = a;
        return va.squaredNorm();
    }
    // The point projected on the line through a and b is at a + t / |v|^2 * v.
    const double t = va.dot(v);
    if (t <= 0.) {
        closest = a;
        return va.squaredNorm();
    }
    if (t >= length_squared) {
        closest = b;
        return (point - b).squaredNorm();
    }
    const Vec2d w = (t / length_squared) * v;
    closest = a + w;
    return (w - va).squaredNorm();
}
}

} // namespace Slic3r::TreeFamily
