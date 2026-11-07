///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef nematx_wipetower_copy_rectilinear_clipper_hpp_
#define nematx_wipetower_copy_rectilinear_clipper_hpp_

/*
Clipper helpers for the rectilinear infill port
===============================================

The rectilinear algorithms call the core ClipperUtils helpers (offset, shrink,
union_ex, to_polygons, get_extents). This header keeps mutable result buffers
local to the algorithm while allowing API ExPolygon views to be passed
directly into Clipper.

Every helper that needs temporary Clipper storage takes the caller storage as
its first argument. The ported call sites pass the plugin storage handle.
*/

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"

#include "RectilinearWorkBuffers.hpp"

namespace slic3r_api { namespace NematXWipeTowerCopy { namespace RectilinearPatternsPlugin {
namespace clipper {

using namespace work;

static constexpr const double DefaultMiterLimit = 3.;

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

inline ExPolygons from_expolygon_collection_view(const slic3r_api::ExPolygonCollection &collection)
{
    ExPolygons out;
    out.reserve(collection.size());
    for (uint32_t idx = 0; idx < collection.size(); ++idx)
        out.push_back(from_expolygon_view(collection.at(idx)));
    return out;
}

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

// union_ex(polygons)
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

// offset(polygons, delta, join_type, miter_limit)
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

// shrink(polygons, delta, join_type, miter_limit)
inline Polygons shrink(storage_handle *storage, const Polygons &polygons, double delta,
                       clipper_join_type_t join_type = CLIPPER_JOIN_MITER, double miter_limit = 3.)
{
    return offset(storage, polygons, -delta, join_type, miter_limit);
}

inline Polygons shrink(storage_handle *storage, const slic3r_api::ExPolygon &expolygon, double delta,
                       clipper_join_type_t join_type = CLIPPER_JOIN_MITER, double miter_limit = 3.)
{
    return offset(storage, expolygon, -delta, join_type, miter_limit);
}

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

// Port of the Polygon.cpp stick / small area filters used by the ported code.
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
        // Remove p2 if it is coincident with p1 or p2.
        return l2_1 == 0 || l2_2 == 0;
    // p3 turns back to p1 after p2. Are p1, p2, p3 collinear?
    // Calculate distance from p3 to a segment (p1, p2) or from p1 to a segment(p2, p3),
    // whichever segment is longer
    double cross = double(v1.x()) * double(v2.y()) - double(v2.x()) * double(v1.y());
    double dist2 = cross * cross / std::max(l2_1, l2_2);
    return dist2 < EPSILON * EPSILON;
}

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

inline bool remove_sticks(ExPolygon &expoly)
{
    // Clean the holes even when the contour changed, like the core helper:
    // both cleanups apply to independent point arrays, and the returned flag
    // has to report either.
    bool modified = remove_sticks(expoly.contour);
    modified |= remove_sticks(expoly.holes);
    return modified;
}

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
}}} // namespace slic3r_api::NematXWipeTowerCopy::RectilinearPatternsPlugin

#endif // nematx_wipetower_copy_rectilinear_clipper_hpp_
