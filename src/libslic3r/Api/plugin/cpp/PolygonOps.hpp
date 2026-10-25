///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_plugin_cpp_PolygonOps_hpp_
#define slic3r_Api_plugin_cpp_PolygonOps_hpp_

#include <cstddef>
#include <vector>

#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"

namespace slic3r_api {

/*
Polygon operations built on the plugin API
==========================================

This header holds C++ helpers that compose existing plugin API calls into one
polygon operation, for every plugin to share. They add no C function, so there
is no ABI change. polygons_simplify() has no Python mirror (C++ plugins only);
remove_collinear() has a Python twin in slic3r_datatree_views.py.

Mental model
------------
polygons_simplify() takes a read-only view of a collection and a storage, and returns a
StoredPolygonCollection owned by that storage. The input is never modified.
remove_collinear() is different: it edits the rings of its argument in place.
Intermediate handles are created in the same storage and are released before
the helper returns; only the returned collection outlives the call, and it
must not outlive the storage.

polygons_simplify()
-------------------
Replacement of the native Slic3r::polygons_simplify(polygons, tolerance, false)
for plugins. The native function cleans each ring alone; so does this one:
  1. every ring is simplified on its own with the Douglas-Peucker algorithm
     (polygons_ensure_valid on a one-ring collection, which is what the native
     function runs, with the same truncation of the tolerance to a coord_t); a
     ring left with fewer than 3 points disappears;
  2. that ring alone goes through a NonZero union (clipper_union), which is what
     ClipperLib::SimplifyPolygons executes in the native function: the self
     intersections created by the simplification (or already in the ring) are
     resolved into counter-clockwise rings. When the simplified ring was
     clockwise, the rings of its result are reversed back to clockwise, as the
     native function does, so the winding of each ring keeps its sign;
  3. one clipper_union over all the cleaned rings gives rings that do not overlap.
     The signs kept in step 2 matter here: a clockwise ring (a hole) inside a
     counter-clockwise one cancels its winding instead of adding to it.

The output is a flat ring list (contours counter-clockwise, holes clockwise), as
the native output is. Under the NonZero fill rule it describes the same region as
the native output.

Differences from the native function (accepted):
  - the order of the rings and the first point of each ring differ: compare the
    results as regions or canonical rings, not with operator==;
  - rings that overlap or touch after the cleaning are merged into one ring by
    step 3 (the native function keeps them as separate rings);
  - a clockwise ring that has no enclosing contour in the set is a filled area
    for the NonZero rule: the native function keeps it clockwise, the final union
    returns it counter-clockwise (same region);
  - areas can differ below the resolution of the simplification (a few scaled
    units of the Clipper integer grid);
  - there is no strictly_simple mode (the plugin callers do not use it).

remove_collinear()
------------------
Port of the native Polygon::remove_collinear(max_offset) (Polygon.cpp), which deletes the points that lie
within max_offset of the segment joining their neighbours. The result is identical to the native one,
point for point, because the native algorithm is order dependent and is reproduced as it is:
  1. while the ring has more than 2 points, the first point is deleted if it is closer than max_offset to
     the segment (last point, second point). The last point of the original ring is the predecessor, and
     the ring can shrink to 2 points here;
  2. one pass over the remaining points, in order: a point is deleted if it is closer than max_offset to the
     segment (last point kept, next ORIGINAL point). After a deletion, the last point kept is not examined
     again (no backtracking, so it is not a stack algorithm); the first and last points are never examined;
  3. while the ring has more than 2 points, the last point is deleted if it is closer than max_offset to
     the segment (point before it, first point left by step 1).
The distance is the squared distance to a segment computed with the arithmetic of the native
Line::distance_to_squared_abp, bit for bit, and compared with a strict < to double(max_offset)^2.
The bit-for-bit guarantee holds for targets that do not contract a*b+c into a fused multiply-add
(the default of x64 MSVC and GCC); aarch64 clang and GCC contract by default, where a result may differ.
A ring with fewer than 3 points is left alone, and a ring is rewritten only when a point is deleted.
The work is O(n): the kept points are compacted into one buffer, no point is shifted.
*/

namespace detail {

// Squared distance from point to the segment [a, b], with the exact floating point operations of the native
// Line::distance_to_squared_abp(a, b, point): differences in coord_t then converted to double, the projection
// parameter t, the three branches (a == b, before a, after b), then s = t / l2 applied to v before the subtraction.
inline double squared_distance_to_segment(c_point a, c_point b, c_point point)
{
    const double vx = double(coord_t(b.x - a.x)), vy = double(coord_t(b.y - a.y));
    const double vax = double(coord_t(point.x - a.x)), vay = double(coord_t(point.y - a.y));
    const double l2 = vx * vx + vy * vy;
    if (l2 == 0.0)
        return vax * vax + vay * vay;
    const double t = vax * vx + vay * vy;
    if (t <= 0.0)
        return vax * vax + vay * vay;
    if (t >= l2) {
        const double bx = double(coord_t(point.x - b.x)), by = double(coord_t(point.y - b.y));
        return bx * bx + by * by;
    }
    const double s = t / l2;
    const double wx = s * vx - vax, wy = s * vy - vay;
    return wx * wx + wy * wy;
}

// Collinear point removal of one ring (the three passes of the header comment). Returns the number of deleted
// points; the ring is rewritten (multipoint_clear then multipoint_insert_array) only when it is more than 0.
inline size_t remove_collinear_points(multipoint_handle *ring, coord_t max_offset)
{
    const multipoint_const_view view = multipoint_view_const(ring);
    const size_t count = view.size;
    if (count < 3)
        return 0;
    const c_point *points = view.array;
    const double threshold = double(max_offset) * max_offset;

    // Pass 1: skip the leading points that are collinear with the last point and their successor.
    size_t first = 0;
    while (count - first > 2 && squared_distance_to_segment(points[count - 1], points[first + 1], points[first]) < threshold)
        ++first;

    // Pass 2: keep a point unless it is within the threshold of the segment (last kept, next original point).
    std::vector<c_point> kept;
    kept.reserve(count - first);
    kept.push_back(points[first]);
    for (size_t idx = first + 1; idx + 1 < count; ++idx)
        if (!(squared_distance_to_segment(kept.back(), points[idx + 1], points[idx]) < threshold))
            kept.push_back(points[idx]);
    kept.push_back(points[count - 1]);

    // Pass 3: drop the trailing points that are collinear with their predecessor and the first kept point.
    while (kept.size() > 2 && squared_distance_to_segment(kept[kept.size() - 2], kept.front(), kept.back()) < threshold)
        kept.pop_back();

    const size_t removed = count - kept.size();
    if (removed > 0) {
        multipoint_clear(ring);
        multipoint_insert_array(ring, 0, kept.data(), uint32_t(kept.size()));
    }
    return removed;
}

} // namespace detail

// Simplify the rings of polygons with the tolerance (scaled units, Douglas-Peucker distance), each ring
// alone, then merge the result into non-overlapping rings. The result is a new collection owned by
// storage; polygons is not modified. An empty input gives an empty collection. See the header comment
// for the differences from the native Slic3r::polygons_simplify.
inline StoredPolygonCollection polygons_simplify(storage_handle *storage, const PolygonCollection &polygons, double tolerance)
{
    ClipperContext clipper(storage);
    // The cleaned rings of every input ring, with the orientation the native function gives them.
    StoredPolygonCollection cleaned_rings(storage);
    for (uint32_t ring_idx = 0; ring_idx < polygons.size(); ++ring_idx) {
        // A copy of the ring alone: ensure_valid edits its collection in place and may delete the ring.
        StoredPolygonCollection ring(storage);
        ring.push_back(polygons.at(ring_idx));
        ring.ensure_valid(coord_t(tolerance));
        if (ring.empty())
            continue;
        const bool clockwise = ring.at(0).is_clockwise();
        // The operand is an adapter over `ring`, which stays alive until the union result is materialized.
        StoredPolygonCollection cleaned = clipper_union(clipper(ring.readonly())).to_polygon_collection();
        if (!clockwise) {
            cleaned_rings.append_move_from(cleaned);
            continue;
        }
        // The union returns counter-clockwise rings: restore the clockwise orientation of a hole.
        while (!cleaned.empty()) {
            StoredPolygon reversed = cleaned.extract(cleaned.size() - 1);
            reversed.reverse();
            cleaned_rings.push_back_move(std::move(reversed));
        }
    }
    return clipper_union(clipper(cleaned_rings.readonly())).to_polygon_collection();
}

// Delete the points of the polygon that are within max_offset (scaled units) of the segment joining their
// neighbours, as the native Polygon::remove_collinear(max_offset) does. Returns the number of deleted points.
inline size_t remove_collinear(StoredPolygon &polygon, coord_t max_offset)
{
    return detail::remove_collinear_points(polygon.mutable_multipoint_handle(), max_offset);
}

// Same for every ring of the collection; returns the total number of deleted points. A ring is never removed:
// a ring that is not reduced keeps its points, a ring with fewer than 3 points is left unchanged, and a reduced
// ring keeps at least 2 points.
inline size_t remove_collinear(StoredPolygonCollection &polygons, coord_t max_offset)
{
    size_t removed = 0;
    for (uint32_t idx = 0; idx < polygons.size(); ++idx)
        removed += detail::remove_collinear_points(polygon_as_multipoint(polygons_at(polygons.mutable_handle(), idx)), max_offset);
    return removed;
}

} // namespace slic3r_api

#endif // slic3r_Api_plugin_cpp_PolygonOps_hpp_
