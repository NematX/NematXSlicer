///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef nematx_gridvarspeed_copy_common_clipper_helpers_hpp_
#define nematx_gridvarspeed_copy_common_clipper_helpers_hpp_

/*
Geometry operations for plugin work buffers
===========================================

Plugin algorithms use these helpers to offset or combine mutable polygon rings
through the public Clipper API. The core is the existing Slic3r implementation.
Conversion helpers copy API views to owned work
buffers, or copy work buffers into the caller's storage. The offset and shrink
overloads taking an API ExPolygon pass that view directly to ClipperOperand,
the shape-operation wrapper defined in Api/plugin/cpp/ClipperViews.hpp;
they do not first copy its points into work buffers. Work-ring overloads copy
their inputs into stored geometry before forming an operand.

Pass a live storage handle first to helpers that allocate temporary API objects.
Stored* types in Api/plugin/cpp/GeometryViews.hpp own geometry handles allocated
in that storage. Local Stored* objects release their temporary handles when they
leave scope; storage must outlive any returned Stored* object.
Returned work buffers own their points. Conversion, offset and boolean helpers
leave their inputs unchanged; remove_sticks and remove_small edit work inputs
in place.
API collections borrow their source geometry. Conversion retains each ExPolygon
view while traversing its holes and copies the points, so the work polygons
outlive the source collection and storage.
Distances use scaled coordinate units.

    // storage is the caller's storage; view is a live API ExPolygon.
    auto inset = clipper::shrink(storage, view, 100.);
    auto editable = clipper::from_expolygon_view(view);

The Polygons union, offset and shrink helpers use ClipperOperand operations.
The ExPolygons boolean and offset adapters instead call API operations that
return geometry collections in core ring, vertex and collection order.
simplify leaves its input unchanged and reduces copied closed rings,
reverses those whose orientation changed, and only then unions them to rebuild
topology if a reversal occurred. Variable inner offsets inset the filled area:
the contour shrinks and holes grow, and the result may split or disappear.
diff_pl requires a nonempty mask: with an empty mask the host copies subject
paths directly, whereas the core Clipper operation may reorder them.
*/

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"

#include "PolygonWorkBuffers.hpp"

namespace slic3r_api { namespace GridVarSpeedCopy {
namespace clipper {

using namespace polygon_work;

static constexpr const double DefaultMiterLimit = 3.;

// Copy ring points into a mutable API polygon without moving a source ring.
inline void copy_ring(polygon_handle *destination, const Polygon &source) {
    multipoint_handle *points = polygon_as_multipoint(destination);
    multipoint_clear(points);
    for (const Point &point : source.points) multipoint_push_back(points, c_point{point.x(), point.y()});
}

// Materialize work polygons in caller storage, retaining every ring vertex and its order.
inline StoredExPolygonCollection to_stored_expolygons(storage_handle *storage, const ExPolygons &source) {
    StoredExPolygonCollection stored(storage);
    expolygons_resize(stored.mutable_handle(), uint32_t(source.size()));
    for (size_t i = 0; i < source.size(); ++i) {
        expolygon_handle *destination = expolygons_at(stored.mutable_handle(), uint32_t(i));
        copy_ring(expolygon_contour(destination), source[i].contour);
        expolygon_holes_resize(destination, uint32_t(source[i].holes.size()));
        for (size_t h = 0; h < source[i].holes.size(); ++h)
            copy_ring(expolygon_hole_at(destination, uint32_t(h)), source[i].holes[h]);
    }
    return stored;
}

// Copy all API ring points into an owned work point array in source order.
inline Points from_polygon_view(const slic3r_api::Polygon &polygon)
{
    Points out;
    out.reserve(polygon.size());
    for (uint32_t idx = 0; idx < polygon.size(); ++idx) {
        const c_point point = polygon.at(idx);
        out.emplace_back(point.x, point.y);
    }
    return out;
}

// Copy the API contour and holes into an owned work ExPolygon.
inline ExPolygon from_expolygon_view(const slic3r_api::ExPolygon &expolygon)
{
    ExPolygon converted;
    converted.contour.points = from_polygon_view(expolygon.contour());
    for (const slic3r_api::Polygon hole : expolygon.holes()) {
        Polygon converted_hole;
        converted_hole.points = from_polygon_view(hole);
        converted.holes.push_back(std::move(converted_hole));
    }
    return converted;
}

// Copy all API polygons and their rings into an owned work collection.
inline ExPolygons from_expolygon_collection_view(const slic3r_api::ExPolygonCollection &collection)
{
    ExPolygons out;
    out.reserve(collection.size());
    for (uint32_t idx = 0; idx < collection.size(); ++idx)
        out.push_back(from_expolygon_view(collection.at(idx)));
    return out;
}

// Copy work rings into caller storage and return the object owning their temporary handles.
inline StoredPolygonCollection to_stored_polygons(storage_handle *storage, const Polygons &polygons)
{
    StoredPolygonCollection out(storage);
    for (const Polygon &polygon : polygons) {
        StoredPolygon stored(storage);
        for (const Point &point : polygon.points)
            stored.push_back(c_point{point.x(), point.y()});
        out.push_back_move(std::move(stored));
    }
    return out;
}

// Copy a live borrowed collection to owned contours and holes in stored order, retaining each element view during traversal.
inline Polygons to_polygons(const slic3r_api::ExPolygonCollection &collection)
{
    // Flatten the result directly from the API view instead of first creating
    // value ExPolygons and then copying their contours and holes again.
    Polygons out;
    size_t polygon_count = 0;
    for (uint32_t idx = 0; idx < collection.size(); ++idx) {
        ++polygon_count;
        const slic3r_api::ExPolygon expolygon = collection.at(idx);
        for (const slic3r_api::Polygon hole : expolygon.holes()) {
            (void)hole;
            ++polygon_count;
        }
    }
    out.reserve(polygon_count);

    for (uint32_t idx = 0; idx < collection.size(); ++idx) {
        const slic3r_api::ExPolygon expolygon = collection.at(idx);
        Polygon contour;
        contour.points = from_polygon_view(expolygon.contour());
        out.push_back(std::move(contour));
        for (const slic3r_api::Polygon hole : expolygon.holes()) {
            Polygon converted_hole;
            converted_hole.points = from_polygon_view(hole);
            out.push_back(std::move(converted_hole));
        }
    }
    return out;
}

// Union work rings through ClipperOperand and copy the resulting contours and holes into owned buffers.
inline ExPolygons union_ex(storage_handle *storage, const Polygons &polygons)
{
    if (polygons.empty())
        return ExPolygons{};
    StoredPolygonCollection stored = to_stored_polygons(storage, polygons);
    ClipperOperand subject(storage, stored.readonly());
    ClipperOperand result = clipper_union(storage, subject);
    StoredExPolygonCollection collection = result.to_expolygon_collection();
    return from_expolygon_collection_view(collection.readonly());
}

// Run the native union on work ExPolygons, preserving its ordered contour/hole result.
inline ExPolygons union_ex(storage_handle *storage, const ExPolygons &polygons) {
    const Polygons rings = polygon_work::to_polygons(polygons);
    StoredPolygonCollection stored = to_stored_polygons(storage, rings);
    StoredExPolygonCollection result = slic3r_api::clipper_union_ex(storage, stored.readonly(), CLIPPER_FILL_NONZERO);
    return from_expolygon_collection_view(result.readonly());
}

// Union two work collections after concatenating their rings into the subject.
inline ExPolygons union_ex(storage_handle *storage, const ExPolygons &first, const ExPolygons &second) {
    ExPolygons combined = first;
    combined.insert(combined.end(), second.begin(), second.end());
    return union_ex(storage, combined);
}

// Offset work ExPolygons using the native API; local Stored* objects free the temporary handles.
inline ExPolygons offset_ex(storage_handle *storage, const ExPolygons &polygons, double delta) {
    StoredExPolygonCollection stored = to_stored_expolygons(storage, polygons);
    StoredExPolygonCollection result = slic3r_api::clipper_offset_ex(storage, stored.readonly(), delta, CLIPPER_JOIN_MITER, 3.);
    return from_expolygon_collection_view(result.readonly());
}

// Offset one work ExPolygon using the same native collection operation.
inline ExPolygons offset_ex(storage_handle *storage, const ExPolygon &polygon, double delta) {
    return offset_ex(storage, ExPolygons{polygon}, delta);
}

// Subtract work collections without a safety offset, preserving native ring and island order.
inline ExPolygons diff_ex(storage_handle *storage, const ExPolygons &subject, const ExPolygons &clip) {
    StoredExPolygonCollection stored_subject = to_stored_expolygons(storage, subject);
    StoredExPolygonCollection stored_clip = to_stored_expolygons(storage, clip);
    StoredExPolygonCollection result = slic3r_api::clipper_diff_ex(storage, stored_subject.readonly(), stored_clip.readonly());
    return from_expolygon_collection_view(result.readonly());
}

// Subtract a nonempty collection of filled areas in one native batch, retaining native path and point order.
inline Polylines diff_pl(storage_handle *storage, const Polylines &subject, const ExPolygons &clip) {
    assert(!clip.empty());
    StoredPolylineCollection stored_subject(storage);
    for (const Polyline &path : subject) {
        StoredPolyline stored_path(storage);
        for (const Point &point : path.points)
            stored_path.push_back(c_point{point.x(), point.y()});
        stored_subject.push_back_move(std::move(stored_path));
    }
    StoredExPolygonCollection stored_clip = to_stored_expolygons(storage, clip);
    StoredPolylineCollection result = slic3r_api::clipper_diff_polylines_expolygons(
        storage, stored_subject.readonly(), stored_clip.readonly());
    Polylines out;
    for (uint32_t i = 0; i < result.size(); ++i) {
        Polyline path;
        const slic3r_api::Polyline view = result.readonly().at(i);
        for (uint32_t j = 0; j < view.size(); ++j)
            path.points.emplace_back(view.at(j));
        out.push_back(std::move(path));
    }
    return out;
}

// Subtract one work ExPolygon from another by delegating to the collection overload.
inline ExPolygons diff_ex(storage_handle *storage, const ExPolygon &subject, const ExPolygon &clip) {
    return diff_ex(storage, ExPolygons{subject}, ExPolygons{clip});
}

// Intersect work collections without a safety offset, preserving the native result order.
inline ExPolygons intersection_ex(storage_handle *storage, const ExPolygons &subject, const ExPolygons &clip) {
    StoredExPolygonCollection stored_subject = to_stored_expolygons(storage, subject);
    StoredExPolygonCollection stored_clip = to_stored_expolygons(storage, clip);
    StoredExPolygonCollection result = slic3r_api::clipper_intersection_ex(storage, stored_subject.readonly(), stored_clip.readonly());
    return from_expolygon_collection_view(result.readonly());
}

// Inset the filled region: shrink its CCW contour and grow its CW holes, with miter limit 2.0.
// Supply contour then hole delta vectors, with one finite value <= 0 per vertex in scaled units.
inline ExPolygons variable_offset_inner_ex(storage_handle *storage, const ExPolygon &polygon,
    const std::vector<std::vector<float>> &deltas) {
    assert(deltas.size() == polygon.holes.size() + 1);
    assert(deltas.front().size() == polygon.contour.size());
    for (size_t i = 0; i < polygon.holes.size(); ++i)
        assert(deltas[i + 1].size() == polygon.holes[i].size());
    StoredExPolygonCollection stored = to_stored_expolygons(storage, ExPolygons{polygon});
    std::vector<float> flat;
    for (const std::vector<float> &ring : deltas) flat.insert(flat.end(), ring.begin(), ring.end());
    StoredExPolygonCollection result = slic3r_api::clipper_variable_offset_inner_ex(
        storage, stored.readonly().at(0), flat.data(), uint32_t(flat.size()));
    return from_expolygon_collection_view(result.readonly());
}

// Expand CCW rings and shrink CW rings by a positive delta, retaining native vertices and order.
inline Polygons expand(storage_handle *storage, const Polygons &polygons, double delta) {
    StoredPolygonCollection stored = to_stored_polygons(storage, polygons);
    StoredPolygonCollection result = slic3r_api::clipper_expand(storage, stored.readonly(), delta);
    Polygons out;
    out.reserve(result.readonly().size());
    for (uint32_t i = 0; i < result.readonly().size(); ++i) {
        Polygon polygon;
        polygon.points = from_polygon_view(result.readonly().at(i));
        out.push_back(std::move(polygon));
    }
    return out;
}

// Compute the signed area with ClipperLib::Area's double arithmetic and accumulation order.
inline double orientation_area(const Polygon &polygon) {
    const int size = int(polygon.size());
    if (size < 3) return 0.;
    double area = 0.;
    for (int i = 0, j = size - 1; i < size; ++i) {
        area += (double(polygon[j].x()) + polygon[i].x()) * (double(polygon[j].y()) - polygon[i].y());
        j = i;
    }
    return -area * 0.5;
}

// Simplify a contour with non-negative area and clockwise holes; drop rings with fewer than three vertices.
// Reverse changed orientations, then rebuild topology by union only if a reversal occurred.
inline ExPolygons simplify(storage_handle *storage, const ExPolygon &polygon, coord_t tolerance) {
    assert(polygon.contour.size() < 3 || orientation_area(polygon.contour) >= 0.);
    for (const Polygon &hole : polygon.holes) assert(orientation_area(hole) < 0.);
    ExPolygons out{polygon};
    bool need_union = false;
    out.front().contour.douglas_peucker(tolerance);
    if (out.front().contour.size() < 3) out.clear();
    else {
        if (orientation_area(out.front().contour) < 0.) {
            out.front().contour.reverse();
            need_union = true;
        }
        for (size_t i = 0; i < out.front().holes.size(); ++i) {
            Polygon &hole = out.front().holes[i];
            assert(hole.size() < 3 || orientation_area(hole) < 0.);
            hole.douglas_peucker(tolerance);
            if (hole.size() < 3) {
                out.front().holes.erase(out.front().holes.begin() + ptrdiff_t(i));
                --i;
            } else if (orientation_area(hole) >= 0.) {
                hole.reverse();
                need_union = true;
            }
        }
    }
    // Reversed rings require rebuilding their filled-area topology before final validation.
    if (need_union) {
        out = union_ex(storage, out);
        StoredExPolygonCollection stored = to_stored_expolygons(storage, out);
        stored.ensure_valid(tolerance);
        out = from_expolygon_collection_view(stored.readonly());
    }
    return out;
}

// Test path containment by checking that the native difference leaves no open paths.
inline bool contains(storage_handle *storage, const ExPolygon &polygon, const Polyline &path) {
    StoredPolyline line(storage);
    for (const Point &point : path.points) line.push_back(c_point{point.x(), point.y()});
    StoredExPolygonCollection stored = to_stored_expolygons(storage, ExPolygons{polygon});
    StoredPolylineCollection result = slic3r_api::clipper_diff_polyline_expolygons(storage, line.readonly(), stored.readonly());
    return result.empty();
}

// Offset closed work rings by delta using the supplied join style and miter limit; return owned rings.
inline Polygons offset(storage_handle *storage, const Polygons &polygons, double delta,
                       clipper_join_type_t join_type = CLIPPER_JOIN_MITER, double miter_limit = 3.)
{
    StoredPolygonCollection stored = to_stored_polygons(storage, polygons);
    ClipperOperand subject(storage, stored.readonly());
    ClipperOperand result = clipper_offset(storage, subject, delta, join_type, miter_limit, CLIPPER_END_CLOSED_POLYGON);
    StoredExPolygonCollection collection = result.to_expolygon_collection();
    return to_polygons(collection.readonly());
}

// API views can be passed directly to Clipper. Only the result is materialized
// into value polygons; the source contour is not copied first.
inline Polygons offset(storage_handle *storage, const slic3r_api::ExPolygon &expolygon, double delta,
                       clipper_join_type_t join_type = CLIPPER_JOIN_MITER, double miter_limit = 3.)
{
    ClipperOperand subject(storage, expolygon);
    ClipperOperand result = clipper_offset(storage, subject, delta, join_type, miter_limit, CLIPPER_END_CLOSED_POLYGON);
    StoredExPolygonCollection collection = result.to_expolygon_collection();
    return to_polygons(collection.readonly());
}

// Offset work rings by negative delta using the supplied join style and miter limit; return owned rings.
inline Polygons shrink(storage_handle *storage, const Polygons &polygons, double delta,
                       clipper_join_type_t join_type = CLIPPER_JOIN_MITER, double miter_limit = 3.)
{
    return offset(storage, polygons, -delta, join_type, miter_limit);
}

// Offset by the negative of delta, returning owned result rings without changing the input.
inline Polygons shrink(storage_handle *storage, const slic3r_api::ExPolygon &expolygon, double delta,
                       clipper_join_type_t join_type = CLIPPER_JOIN_MITER, double miter_limit = 3.)
{
    return offset(storage, expolygon, -delta, join_type, miter_limit);
}

// Return coordinate bounds containing the supplied points or rings.
inline BoundingBox get_extents(const Polygon &polygon) { return BoundingBox(polygon.points); }
inline BoundingBox get_extents(const Polygons &polygons) {
    BoundingBox out;
    for (const Polygon &polygon : polygons)
        out.merge(polygon.points);
    return out;
}
inline BoundingBox get_extents(const ExPolygon &expolygon) {
    BoundingBox out = BoundingBox(expolygon.contour.points);
    for (const Polygon &hole : expolygon.holes)
        out.merge(hole.points);
    return out;
}
inline BoundingBox get_extents(const Polyline &polyline) { return BoundingBox(polyline.points); }

// Measure all outer contours; holes cannot enlarge the extent of valid ExPolygons.
inline BoundingBox get_extents(const ExPolygons &polygons) {
    BoundingBox out;
    for (const ExPolygon &polygon : polygons) out.merge(polygon.contour.points);
    return out;
}

// Test whether the middle point repeats a neighbour or nearly retraces a collinear segment.
static inline bool is_stick(const Point &p1, const Point &p2, const Point &p3)
{
    Point v1 = p2 - p1;
    Point v2 = p3 - p2;
    int64_t dir = int64_t(v1.x()) * int64_t(v2.x()) + int64_t(v1.y()) * int64_t(v2.y());
    if (dir > 0)
        // p3 does not turn back to p1. Do not remove p2.
        return false;
    double l2_1 = double(v1.x()) * double(v1.x()) + double(v1.y()) * double(v1.y());
    double l2_2 = double(v2.x()) * double(v2.x()) + double(v2.y()) * double(v2.y());
    if (dir == 0)
        // p1, p2, p3 may make a perpendicular corner, or there is a zero edge length.
        // Remove p2 if it is coincident with p1 or p3.
        return l2_1 == 0 || l2_2 == 0;
    // p3 turns back to p1 after p2. Are p1, p2, p3 collinear?
    // Calculate distance from p3 to a segment (p1, p2) or from p1 to a segment(p2, p3),
    // whichever segment is longer
    double cross = double(v1.x()) * double(v2.y()) - double(v2.x()) * double(v1.y());
    double dist2 = cross * cross / std::max(l2_1, l2_2);
    return dist2 < EPSILON * EPSILON;
}

// Remove repeated neighbours and nearly collinear reversals in place; return whether vertices were removed.
inline bool remove_sticks(Polygon &poly)
{
    bool modified = false;
    size_t j = 1;
    for (size_t i = 1; i + 1 < poly.points.size(); ++i) {
        if (! is_stick(poly.points[j - 1], poly.points[i], poly.points[i + 1])) {
            // Keep the point.
            if (j < i)
                poly.points[j] = poly.points[i];
            ++j;
        }
    }
    if (++j < poly.points.size()) {
        poly.points[j - 1] = poly.points.back();
        poly.points.erase(poly.points.begin() + ptrdiff_t(j), poly.points.end());
        modified = true;
    }
    while (poly.points.size() >= 3 && is_stick(poly.points[poly.points.size() - 2], poly.points.back(), poly.points.front())) {
        poly.points.pop_back();
        modified = true;
    }
    while (poly.points.size() >= 3 && is_stick(poly.points.back(), poly.points.front(), poly.points[1])) {
        poly.points.erase(poly.points.begin());
        modified = true;
    }
    return modified;
}

// Clean rings in place and drop rings with fewer than three vertices.
// Return whether per-ring cleanup removed vertices; dropping an already-short ring alone does not set the flag.
inline bool remove_sticks(Polygons &polys)
{
    bool modified = false;
    size_t j = 0;
    for (size_t i = 0; i < polys.size(); ++i) {
        modified |= remove_sticks(polys[i]);
        if (polys[i].points.size() >= 3) {
            if (j < i)
                std::swap(polys[i].points, polys[j].points);
            ++j;
        }
    }
    if (j < polys.size())
        polys.erase(polys.begin() + ptrdiff_t(j), polys.end());
    return modified;
}

// Clean the contour and holes in place and drop holes with fewer than three vertices.
// Return the OR of the contour and hole-cleanup flags.
inline bool remove_sticks(ExPolygon &expoly)
{
    // Clean the holes even when the contour changed, like the core helper:
    // both cleanups apply to independent point arrays, and the returned flag
    // has to report either.
    bool modified = remove_sticks(expoly.contour);
    modified |= remove_sticks(expoly.holes);
    return modified;
}

// Remove rings whose absolute area is below min_area and report whether any were removed.
inline bool remove_small(Polygons &polys, double min_area)
{
    bool modified = false;
    size_t j = 0;
    for (size_t i = 0; i < polys.size(); ++i) {
        if (std::abs(polys[i].area()) >= min_area) {
            if (j < i)
                std::swap(polys[i].points, polys[j].points);
            ++j;
        } else
            modified = true;
    }
    if (j < polys.size())
        polys.erase(polys.begin() + ptrdiff_t(j), polys.end());
    return modified;
}

} // namespace clipper
}} // namespace slic3r_api::GridVarSpeedCopy

#endif // nematx_gridvarspeed_copy_common_clipper_helpers_hpp_
