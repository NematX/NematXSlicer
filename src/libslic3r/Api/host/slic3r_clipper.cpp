///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <memory>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_clipper.h"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Polyline.hpp"

#include "ClipperShapes.hpp"
#include "Orchestrator.hpp"

/*
Provide polygon clipping and offsets to plugins without exposing native C++ types.
The shape route borrows geometry through ClipperShapes, computes intermediate
path lists or contour/hole trees, then materializes public geometry collections.
The native-result route calls ClipperUtils directly when callers need its exact
processing and order rather than the shape route's processing.
Closed-ring offsets clean spikes without changing unrelated rings. The configurable
variable inward offset validates per-vertex distances and coordinate margins
before calling the native geometry routine; inputs are never changed.

Native-result calls validate arguments, compute into local owning temporaries,
then transfer a successful collection to PluginStorage. They catch all exceptions
and return NULL without storing a result. The shape route does not provide that
exception guarantee. Storage owns returned handles until free, clear or destruction;
adapters borrow their source geometry for their entire use.

PluginStorage locks its ownership registry, not the geometry. Callers keep borrowed
inputs alive and unchanged during use and exclude concurrent edits or destruction.

Plugin calls
|-- clipper_shapes_from_*: retain a borrowed source adapter in storage
|-- clipper_execute / clipper_offset: compute a stored shape result
|   `-- offset_closed_rings: clean affected rings and unite the result
|-- clipper_shapes_to_*: materialize a storage-owned geometry collection
|-- clipper_union_ex / clipper_offset_ex / clipper_offset2_ex: compute native results
|   `-- store_expolygons: transfer the completed collection to storage
|-- clipper_variable_offset_inner: validate distances and coordinate margins
|   `-- variable_offset_inner_ex: compute with the requested miter limit
|-- clipper_diff_ex / clipper_intersection_ex: native cleanup and topology rebuild
|   `-- store_expolygons: transfer the completed collection to storage
|-- clipper_variable_offset_inner_ex: validate and regroup vertex distances
|   |-- variable_offset_inner_ex: compute inset areas using per-ring distances
|   `-- store_expolygons: transfer the completed collection to storage
`-- clipper_expand: expand individual rings, unite them and store Polygons
*/

// Borrow both collections and store the native difference unchanged; NULL handles
// or any exception return NULL without adding an object to storage.
expolygon_collection_handle *clipper_diff_ex(storage_handle *storage,
    const expolygon_collection_handle *subject, const expolygon_collection_handle *clip);
// Borrow both collections and store the native intersection unchanged; NULL
// handles or any exception return NULL without adding an object to storage.
expolygon_collection_handle *clipper_intersection_ex(storage_handle *storage,
    const expolygon_collection_handle *subject, const expolygon_collection_handle *clip);
// Validate borrowed rings and flattened non-positive deltas before rebuilding
// native ring vectors; store only a successful result, returning NULL on failure.
expolygon_collection_handle *clipper_variable_offset_inner_ex(storage_handle *storage,
    const expolygon_handle *expolygon, const float *deltas, uint32_t delta_count);
// Expand borrowed rings with native orientation handling and union; finite
// positive delta is required, and failures store nothing and return NULL.
polygon_collection_handle *clipper_expand(storage_handle *storage,
    const polygon_collection_handle *polygons, double delta);

namespace Slic3r {

// Borrowed subject collections remain read-only while core clipping owns its temporary paths.
static const Polylines *to_polylines(const polyline_collection_handle *handle);

// Offset normalized closed rings (contours counter-clockwise, holes clockwise):
// each ring whose offset emits a spike is cleaned alone, the others are offset
// raw, then one positive-fill union.
static ClipperLib::PolyTree offset_closed_rings(const ClipperLib::Paths &rings, double delta,
                                                ClipperLib::JoinType join_type, double miter_limit);

// Apply the join limit, the ClipperUtils shortest-edge rule and the cheaper
// positive-fill clean-up of negative offsets.
static void configure_offsetter(ClipperLib::ClipperOffset &offsetter, double delta,
                                ClipperLib::JoinType join_type, double miter_limit);

// Whether ClipperOffset emits at least one spike loop when offsetting the ring.
static bool has_offset_spike(const ClipperLib::Path &ring, double delta);

// Unite the rings of an offset result with a non-zero fill.
static ClipperLib::PolyTree united_polytree(ClipperLib::PolyTree tree);


/* ---- ABI handle casts ------------------------------------------------
The public headers expose opaque C handles. Internally they are pointers to the
native Slic3r objects or to the ApiClipper::ClipperShapes interface.
*/

static PluginStorage *to_storage(storage_handle *storage) {
    return reinterpret_cast<PluginStorage *>(storage);
}

static const Polygon *to_polygon(const polygon_handle *handle) {
    return reinterpret_cast<const Polygon *>(handle);
}

static const Polyline *to_polyline(const polyline_handle *handle) {
    return reinterpret_cast<const Polyline *>(handle);
}

static Polylines *to_polylines(polyline_collection_handle *handle) {
    return reinterpret_cast<Polylines *>(handle);
}

static const Polylines *to_polylines(const polyline_collection_handle *handle) {
    return reinterpret_cast<const Polylines *>(handle);
}

static const Polygons *to_polygons(const polygon_collection_handle *handle) {
    return reinterpret_cast<const Polygons *>(handle);
}

static Polygons *to_polygons(polygon_collection_handle *handle) {
    return reinterpret_cast<Polygons *>(handle);
}

static const ExPolygon *to_expolygon(const expolygon_handle *handle) {
    return reinterpret_cast<const ExPolygon *>(handle);
}

static ExPolygons *to_expolygons(expolygon_collection_handle *handle) {
    return reinterpret_cast<ExPolygons *>(handle);
}

static const ExPolygons *to_expolygons(const expolygon_collection_handle *handle) {
    return reinterpret_cast<const ExPolygons *>(handle);
}

static BoundingBox to_bounding_box(c_bounding_box bbox) {
    return BoundingBox(Point(bbox.min.x, bbox.min.y), Point(bbox.max.x, bbox.max.y));
}

static c_point to_c_point(const Point &point) {
    c_point out = {};
    out.x = point.x();
    out.y = point.y();
    return out;
}

static c_bounding_box to_c_bounding_box(const BoundingBox &bbox) {
    c_bounding_box out = {};
    out.min = to_c_point(bbox.min);
    out.max = to_c_point(bbox.max);
    return out;
}

static ApiClipper::ClipperShapes *to_shapes(clipper_shapes_handle *handle) {
    return reinterpret_cast<ApiClipper::ClipperShapes *>(handle);
}

static const ApiClipper::ClipperShapes *to_shapes(const clipper_shapes_handle *handle) {
    return reinterpret_cast<const ApiClipper::ClipperShapes *>(handle);
}

// Retain the shape through the locked storage registry; null shapes return null and insertion errors leave no tracked handle.
static ApiClipper::ClipperShapes *store_shape(PluginStorage &plugin_storage, std::unique_ptr<ApiClipper::ClipperShapes> shape) {
    return plugin_storage.store(std::move(shape));
}

/*
Store a newly created ClipperShapes implementation in plugin temporary storage.

The returned raw pointer is the ABI handle. The storage registry entry owns the
shape, so the pointer remains valid until storage_free() or storage_clear()
removes it.
*/
static clipper_shapes_handle *store_shape(storage_handle *storage, std::unique_ptr<ApiClipper::ClipperShapes> shape) {
    if (storage == nullptr || !shape)
        return nullptr;
    PluginStorage *plugin_storage = to_storage(storage);
    return reinterpret_cast<clipper_shapes_handle *>(store_shape(*plugin_storage, std::move(shape)));
}

/* Translate small ABI enums to ClipperLib enums without leaking Clipper headers
through the C ABI.
*/
static ClipperLib::ClipType to_clip_type(clipper_operation_t operation) {
    switch (operation) {
    case CLIPPER_OPERATION_DIFFERENCE:   return ClipperLib::ctDifference;
    case CLIPPER_OPERATION_INTERSECTION: return ClipperLib::ctIntersection;
    case CLIPPER_OPERATION_UNION:        return ClipperLib::ctUnion;
    case CLIPPER_OPERATION_XOR:          return ClipperLib::ctXor;
    default:                             return ClipperLib::ctDifference;
    }
}

// Strict conversions: return false for an unknown ABI value instead of defaulting.
static bool to_known_join_type(clipper_join_type_t join_type, ClipperLib::JoinType &out) {
    switch (join_type) {
    case CLIPPER_JOIN_SQUARE: out = ClipperLib::jtSquare; return true;
    case CLIPPER_JOIN_ROUND:  out = ClipperLib::jtRound;  return true;
    case CLIPPER_JOIN_MITER:  out = ClipperLib::jtMiter;  return true;
    default:                  return false;
    }
}

// Permissive variant for the older functions: an unknown value falls back to a miter join.
static ClipperLib::JoinType to_join_type(clipper_join_type_t join_type) {
    ClipperLib::JoinType native_join = ClipperLib::jtMiter;
    to_known_join_type(join_type, native_join);
    return native_join;
}

static bool to_fill_type(clipper_fill_rule_t fill_rule, ClipperLib::PolyFillType &out) {
    switch (fill_rule) {
    case CLIPPER_FILL_NONZERO:  out = ClipperLib::pftNonZero;  return true;
    case CLIPPER_FILL_EVENODD:  out = ClipperLib::pftEvenOdd;  return true;
    case CLIPPER_FILL_POSITIVE: out = ClipperLib::pftPositive; return true;
    default:                    return false;
    }
}

static ClipperLib::EndType to_end_type(clipper_end_type_t end_type) {
    switch (end_type) {
    case CLIPPER_END_CLOSED_POLYGON: return ClipperLib::etClosedPolygon;
    case CLIPPER_END_CLOSED_LINE:    return ClipperLib::etClosedLine;
    case CLIPPER_END_OPEN_BUTT:      return ClipperLib::etOpenButt;
    case CLIPPER_END_OPEN_SQUARE:    return ClipperLib::etOpenSquare;
    case CLIPPER_END_OPEN_ROUND:     return ClipperLib::etOpenRound;
    default:                         return ClipperLib::etClosedPolygon;
    }
}

static ClipperLib::Paths safety_offset_paths(const ApiClipper::ClipperShapes &clip_shapes) {
    ClipperLib::ClipperOffset offsetter;
    offsetter.MiterLimit = DefaultMiterLimit;
    offsetter.ShortestEdgeLength = ClipperSafetyOffset * Slic3r::ClipperOffsetShortestEdgeFactor;
    offsetter.AddPaths(clip_shapes.to_paths(), DefaultJoinType, ClipperLib::etClosedPolygon);

    ClipperLib::Paths out;
    offsetter.Execute(out, ClipperSafetyOffset);
    return out;
}

/* ---- Offset strategy -------------------------------------------------
At every vertex where the offset turns against the ring (a convex vertex of a
shrinking ring, a concave vertex of a growing one), ClipperOffset emits a small
self-intersecting loop, a "spike", and its final union removes the loops.

Offsetting all rings with one ClipperOffset resolves every spike in a single
union whose sweep line crosses the whole shape, so each spike costs as much as
the shape is wide.

Several rings are therefore offset one by one, like the native ClipperUtils
offsets. A ring with a spike is cleaned by its own small union. A ring without
spike is convex in the offset direction, so its raw offset is already a simple
polygon and skips that union. One positive-fill union then merges every ring:
contours count +1 and holes -1, so islands that meet merge, while a hole that
grows past its contour stays outside.

This needs normalized rings, which ExPolygon and PolyTree sources guarantee. A
raw polygon list may hold slivers of any orientation, such as projected facets:
a clockwise sliver offset outwards turns inside out, and in a grouped union its
raw loop still counts +1, so the whole set grows. Cleaned alone it would be read
as a vanishing hole, so raw polygon and path lists keep the grouped
ClipperOffset. A lone ring keeps it too: cleaning it alone then uniting it
again would run two unions for nothing.

Both paths can leave a zero-width bridge: an edge walked twice in opposite
directions, or a hole sharing an edge with its contour. Clipper joins output
rings that touch along an edge, so it happens when overlapping raw spike loops
or an exact collapse (a feature exactly twice the offset wide) leave a
zero-width remnant. A bridge always shows as two collinear edges overlapping
along a positive length: the edges are sorted along their support lines, and a
second non-zero union, which splits the rings there, only runs when such an
overlap exists (ApiClipper::has_overlapping_edges).
*/

static ClipperLib::PolyTree offset_closed_rings(const ClipperLib::Paths &rings, double delta,
                                                ClipperLib::JoinType join_type, double miter_limit)
{
    ClipperLib::ClipperOffset offsetter;
    configure_offsetter(offsetter, delta, join_type, miter_limit);
    ClipperLib::Paths offset_paths;
    ClipperLib::Paths ring_paths;
    for (const ClipperLib::Path &ring : rings) {
        // ClipperOffset turns a lone clockwise ring counter-clockwise, so a
        // hole is offset with the opposite sign and its result reversed to
        // keep the -1 winding of a hole.
        const bool contour = ClipperLib::Orientation(ring);
        const double ring_delta = contour ? delta : -delta;
        offsetter.Clear();
        offsetter.AddPath(ring, join_type, ClipperLib::etClosedPolygon);
        if (has_offset_spike(ring, delta))
            offsetter.Execute(ring_paths, ring_delta);
        else
            offsetter.ExecuteRaw(ring_paths, ring_delta);
        for (ClipperLib::Path &path : ring_paths) {
            if (!contour)
                std::reverse(path.begin(), path.end());
            offset_paths.push_back(std::move(path));
        }
    }

    ClipperLib::Clipper clipper;
    clipper.AddPaths(offset_paths, ClipperLib::ptSubject, true);
    ClipperLib::PolyTree result;
    clipper.Execute(ClipperLib::ctUnion, result, ClipperLib::pftPositive, ClipperLib::pftPositive);
    return result;
}

static void configure_offsetter(ClipperLib::ClipperOffset &offsetter, double delta,
                                ClipperLib::JoinType join_type, double miter_limit)
{
    if (join_type == ClipperLib::jtRound)
        offsetter.ArcTolerance = miter_limit;
    else
        offsetter.MiterLimit = miter_limit;
    // Keep the same shortest-edge heuristic as ClipperUtils offsets.
    offsetter.ShortestEdgeLength = std::abs(delta * ClipperOffsetShortestEdgeFactor);
    // Same area as the bounding-rectangle clean-up of negative offsets, cheaper.
    offsetter.ShrinkWithPositiveFill = true;
}

static bool has_offset_spike(const ClipperLib::Path &ring, double delta)
{
    // Same test as ClipperOffset::OffsetPoint, on the ring as given: a nearly
    // collinear vertex emits one point, any other vertex emits a spike when its
    // turn (sign of the cross product) opposes the offset sign.
    const size_t count = ring.size();
    for (size_t index = 0; index < count; ++index) {
        const ClipperLib::IntPoint &previous = ring[(index + count - 1) % count];
        const ClipperLib::IntPoint &point = ring[index];
        const ClipperLib::IntPoint &next = ring[(index + 1) % count];
        const double ax = double(point.x() - previous.x());
        const double ay = double(point.y() - previous.y());
        const double bx = double(next.x() - point.x());
        const double by = double(next.y() - point.y());
        const double lengths = std::hypot(ax, ay) * std::hypot(bx, by);
        if (lengths == 0.)
            continue;
        const double sin_a = (ax * by - ay * bx) / lengths;
        const double cos_a = (ax * bx + ay * by) / lengths;
        if (std::abs(sin_a * delta) < 1. && cos_a > 0.)
            continue;
        if (sin_a * delta < 0.)
            return true;
    }
    return false;
}

static ClipperLib::PolyTree united_polytree(ClipperLib::PolyTree tree)
{
    ClipperLib::Paths rings;
    ClipperLib::PolyTreeToPaths(std::move(tree), rings);
    ClipperLib::Clipper clipper;
    clipper.AddPaths(rings, ClipperLib::ptSubject, true);
    ClipperLib::PolyTree united;
    clipper.Execute(ClipperLib::ctUnion, united, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
    return united;
}
} // namespace Slic3r

extern "C" {

/* ---- Adapter creation ------------------------------------------------
These functions allocate only a small ClipperShapes adapter in storage. Source
geometry is intentionally not copied, keeping setup cheap for plugin code that
creates short-lived boolean operations.
*/

clipper_shapes_handle *clipper_shapes_create_empty(storage_handle *storage) {
    return Slic3r::store_shape(storage, Slic3r::ApiClipper::make_path_list_shapes(Slic3r::ClipperLib::Paths{}));
}

clipper_shapes_handle *clipper_shapes_from_polygon(storage_handle *storage, const polygon_handle *polygon) {
    return Slic3r::store_shape(storage, Slic3r::ApiClipper::make_polygon_shapes(Slic3r::to_polygon(polygon)));
}

clipper_shapes_handle *clipper_shapes_from_polyline(storage_handle *storage, const polyline_handle *polyline) {
    return Slic3r::store_shape(storage, Slic3r::ApiClipper::make_polyline_shapes(Slic3r::to_polyline(polyline)));
}

clipper_shapes_handle *clipper_shapes_from_polygons(storage_handle *storage, const polygon_collection_handle *polygons) {
    return Slic3r::store_shape(storage, Slic3r::ApiClipper::make_polygons_shapes(Slic3r::to_polygons(polygons)));
}

clipper_shapes_handle *clipper_shapes_from_expolygon(storage_handle *storage, const expolygon_handle *expolygon) {
    return Slic3r::store_shape(storage, Slic3r::ApiClipper::make_expolygon_shapes(Slic3r::to_expolygon(expolygon)));
}

clipper_shapes_handle *clipper_shapes_from_expolygons(storage_handle *storage, const expolygon_collection_handle *expolygons) {
    return Slic3r::store_shape(storage, Slic3r::ApiClipper::make_expolygons_shapes(Slic3r::to_expolygons(expolygons)));
}

int32_t clipper_shapes_empty(const clipper_shapes_handle *shapes) {
    const Slic3r::ApiClipper::ClipperShapes *source = Slic3r::to_shapes(shapes);
    return source == nullptr || source->empty() ? 1 : 0;
}

uint32_t clipper_shapes_path_count(const clipper_shapes_handle *shapes) {
    const Slic3r::ApiClipper::ClipperShapes *source = Slic3r::to_shapes(shapes);
    return source == nullptr ? 0 : source->path_count();
}

c_bounding_box clipper_shapes_bounding_box(const clipper_shapes_handle *shapes) {
    const Slic3r::ApiClipper::ClipperShapes *source = Slic3r::to_shapes(shapes);
    return source == nullptr ? c_bounding_box{} : Slic3r::to_c_bounding_box(source->bounding_box());
}

/* ---- Boolean operations ----------------------------------------------
Boolean operations create a PolyTree-backed ClipperShapes result. Returning a
ClipperShapes handle instead of immediately materializing Polygons/ExPolygons
lets callers chain operations without repeatedly converting between formats.
*/

static clipper_shapes_handle *clipper_execute_impl(storage_handle *storage,
                                                   clipper_operation_t operation,
                                                   const clipper_shapes_handle *subject,
                                                   const clipper_shapes_handle *clip,
                                                   bool apply_safety_offset,
                                                   Slic3r::ClipperLib::PolyFillType fill_type)
{
    const Slic3r::ApiClipper::ClipperShapes *subject_shapes = Slic3r::to_shapes(subject);
    const Slic3r::ApiClipper::ClipperShapes *clip_shapes = Slic3r::to_shapes(clip);
    if (storage == nullptr || subject_shapes == nullptr)
        return nullptr;

    Slic3r::ClipperLib::Clipper clipper;
    subject_shapes->add_to_clipper(clipper, Slic3r::ClipperLib::ptSubject, true);
    if (clip_shapes != nullptr) {
        const bool can_apply_safety_offset =
            operation == CLIPPER_OPERATION_DIFFERENCE || operation == CLIPPER_OPERATION_INTERSECTION;
        if (can_apply_safety_offset && apply_safety_offset) {
            const Slic3r::ClipperLib::Paths expanded_clip = Slic3r::safety_offset_paths(*clip_shapes);
            clipper.AddPaths(expanded_clip, Slic3r::ClipperLib::ptClip, true);
        } else {
            clip_shapes->add_to_clipper(clipper, Slic3r::ClipperLib::ptClip, true);
        }
    }

    // PolyTree keeps the contour/hole hierarchy needed by to_expolygons().
    Slic3r::ClipperLib::PolyTree result;
    clipper.Execute(Slic3r::to_clip_type(operation), result, fill_type, fill_type);
    return Slic3r::store_shape(storage, Slic3r::ApiClipper::make_polytree_shapes(std::move(result)));
}

clipper_shapes_handle *clipper_execute(storage_handle *storage,
                                       clipper_operation_t operation,
                                       const clipper_shapes_handle *subject,
                                       const clipper_shapes_handle *clip)
{
    return clipper_execute_impl(storage, operation, subject, clip, false, Slic3r::ClipperLib::pftNonZero);
}

clipper_shapes_handle *clipper_diff(storage_handle *storage,
                                    const clipper_shapes_handle *subject,
                                    const clipper_shapes_handle *clip)
{
    return clipper_execute(storage, CLIPPER_OPERATION_DIFFERENCE, subject, clip);
}

clipper_shapes_handle *clipper_intersection(storage_handle *storage,
                                           const clipper_shapes_handle *subject,
                                           const clipper_shapes_handle *clip)
{
    return clipper_execute(storage, CLIPPER_OPERATION_INTERSECTION, subject, clip);
}

int32_t clipper_split(storage_handle *storage,
                      const clipper_shapes_handle *subject,
                      const clipper_shapes_handle *clip,
                      clipper_shapes_handle **inside,
                      clipper_shapes_handle **outside)
{
    const Slic3r::ApiClipper::ClipperShapes *subject_shapes = Slic3r::to_shapes(subject);
    const Slic3r::ApiClipper::ClipperShapes *clip_shapes = Slic3r::to_shapes(clip);
    if (storage == nullptr || subject_shapes == nullptr || inside == nullptr || outside == nullptr)
        return 0;

    // Each operation runs on its own Clipper fed with the same inputs, so the two results are cut along
    // the same clip edges. add_to_clipper() keeps the kind of each adapter: a polyline operand stays
    // an open path, as in clipper_intersection() and clipper_diff().
    auto run = [&](Slic3r::ClipperLib::ClipType clip_type) {
        Slic3r::ClipperLib::Clipper clipper;
        subject_shapes->add_to_clipper(clipper, Slic3r::ClipperLib::ptSubject, true);
        if (clip_shapes != nullptr)
            clip_shapes->add_to_clipper(clipper, Slic3r::ClipperLib::ptClip, true);
        Slic3r::ClipperLib::PolyTree result;
        clipper.Execute(clip_type, result, Slic3r::ClipperLib::pftNonZero, Slic3r::ClipperLib::pftNonZero);
        return Slic3r::store_shape(storage, Slic3r::ApiClipper::make_polytree_shapes(std::move(result)));
    };
    *inside = run(Slic3r::ClipperLib::ctIntersection);
    *outside = run(Slic3r::ClipperLib::ctDifference);
    return 1;
}

clipper_shapes_handle *clipper_diff_with_safety_offset(storage_handle *storage,
                                                       const clipper_shapes_handle *subject,
                                                       const clipper_shapes_handle *clip)
{
    return clipper_execute_impl(storage, CLIPPER_OPERATION_DIFFERENCE, subject, clip, true, Slic3r::ClipperLib::pftNonZero);
}

clipper_shapes_handle *clipper_intersection_with_safety_offset(storage_handle *storage,
                                                              const clipper_shapes_handle *subject,
                                                              const clipper_shapes_handle *clip)
{
    return clipper_execute_impl(storage, CLIPPER_OPERATION_INTERSECTION, subject, clip, true, Slic3r::ClipperLib::pftNonZero);
}

// Union of the subject paths with the given fill rule; an unknown rule returns NULL.
clipper_shapes_handle *clipper_union(storage_handle *storage,
                                     const clipper_shapes_handle *subject,
                                     clipper_fill_rule_t fill_rule)
{
    // Validate the rule, then run the same path as every other boolean operation with that fill type
    // for the subject and the (absent) clip.
    Slic3r::ClipperLib::PolyFillType fill_type;
    if (!Slic3r::to_fill_type(fill_rule, fill_type))
        return nullptr;
    return clipper_execute_impl(storage, CLIPPER_OPERATION_UNION, subject, nullptr, false, fill_type);
}

// Stores the result in a new storage-owned collection through the C++ PluginStorage (not the extern "C"
// storage_new_expolygons(), which MSVC assumes never throws), so an allocation failure reaches the caller try/catch.
namespace Slic3r {
static expolygon_collection_handle *store_expolygons(storage_handle *storage, ExPolygons &&result) {
    return reinterpret_cast<expolygon_collection_handle *>(
        to_storage(storage)->store(std::make_unique<ExPolygons>(std::move(result))));
}
} // namespace Slic3r

// Native union_ex(Polygons, fill_type), stored as is in a new storage-owned collection.
expolygon_collection_handle *clipper_union_ex(storage_handle *storage,
                                              const polygon_collection_handle *polygons,
                                              clipper_fill_rule_t fill_rule)
{
    Slic3r::ClipperLib::PolyFillType fill_type;
    if (storage == nullptr || polygons == nullptr || !Slic3r::to_fill_type(fill_rule, fill_type))
        return nullptr;
    // The native result is computed first: an exception (bad_alloc, ClipperLib range error) must not cross the
    // C boundary nor leave an empty collection in the storage.
    try {
        Slic3r::ExPolygons result = Slic3r::union_ex(*Slic3r::to_polygons(polygons), fill_type);
        return Slic3r::store_expolygons(storage, std::move(result));
    } catch (...) {
        return nullptr;
    }
}

// Native offset_ex(ExPolygons, ...), stored as is. A zero delta is accepted: native does not assert on it here.
// Non-finite values are rejected because ClipperLib throws on them, and no exception may cross the C boundary.
expolygon_collection_handle *clipper_offset_ex(storage_handle *storage,
                                               const expolygon_collection_handle *src,
                                               double delta,
                                               clipper_join_type_t join_type,
                                               double miter_limit)
{
    Slic3r::ClipperLib::JoinType native_join;
    if (storage == nullptr || src == nullptr || !std::isfinite(delta) || !std::isfinite(miter_limit) ||
        !Slic3r::to_known_join_type(join_type, native_join))
        return nullptr;
    // Compute first, then store; an internal error returns NULL with nothing added to the storage.
    try {
        Slic3r::ExPolygons result = Slic3r::offset_ex(*Slic3r::to_expolygons(src), delta, native_join, miter_limit);
        return Slic3r::store_expolygons(storage, std::move(result));
    } catch (...) {
        return nullptr;
    }
}

// Native offset2_ex(ExPolygons, ...), stored as is. delta2 == 0 is rejected: native offset_paths asserts
// offset != 0 for the second pass. Non-finite values are rejected as in clipper_offset_ex().
expolygon_collection_handle *clipper_offset2_ex(storage_handle *storage,
                                                const expolygon_collection_handle *src,
                                                double delta1,
                                                double delta2,
                                                clipper_join_type_t join_type,
                                                double miter_limit)
{
    Slic3r::ClipperLib::JoinType native_join;
    if (storage == nullptr || src == nullptr || !std::isfinite(delta1) || !std::isfinite(delta2) ||
        !std::isfinite(miter_limit) || delta2 == 0. || !Slic3r::to_known_join_type(join_type, native_join))
        return nullptr;
    // Compute first, then store; an internal error returns NULL with nothing added to the storage.
    try {
        Slic3r::ExPolygons result =
            Slic3r::offset2_ex(*Slic3r::to_expolygons(src), delta1, delta2, native_join, miter_limit);
        return Slic3r::store_expolygons(storage, std::move(result));
    } catch (...) {
        return nullptr;
    }
}

expolygon_collection_handle *clipper_diff_ex(storage_handle *storage,
    const expolygon_collection_handle *subject, const expolygon_collection_handle *clip)
{
    if (storage == nullptr || subject == nullptr || clip == nullptr)
        return nullptr;
    try {
        Slic3r::ExPolygons result = Slic3r::diff_ex(*Slic3r::to_expolygons(subject), *Slic3r::to_expolygons(clip));
        return Slic3r::store_expolygons(storage, std::move(result));
    } catch (...) {
        return nullptr;
    }
}

expolygon_collection_handle *clipper_intersection_ex(storage_handle *storage,
    const expolygon_collection_handle *subject, const expolygon_collection_handle *clip)
{
    if (storage == nullptr || subject == nullptr || clip == nullptr)
        return nullptr;
    try {
        Slic3r::ExPolygons result = Slic3r::intersection_ex(*Slic3r::to_expolygons(subject), *Slic3r::to_expolygons(clip));
        return Slic3r::store_expolygons(storage, std::move(result));
    } catch (...) {
        return nullptr;
    }
}

expolygon_collection_handle *clipper_variable_offset_inner_ex(storage_handle *storage,
    const expolygon_handle *expolygon, const float *deltas, uint32_t delta_count)
{
    if (storage == nullptr || expolygon == nullptr)
        return nullptr;
    try {
        const Slic3r::ExPolygon &input = *Slic3r::to_expolygon(expolygon);
        // Reject orientations and delta layouts the native inset routine cannot use.
        size_t expected = input.contour.size();
        if (!(Slic3r::ClipperLib::Area(input.contour.points) > 0.))
            return nullptr;
        for (const Slic3r::Polygon &hole : input.holes) {
            if (!(Slic3r::ClipperLib::Area(hole.points) < 0.))
                return nullptr;
            expected += hole.size();
        }
        if (expected != delta_count || (expected > 0 && deltas == nullptr))
            return nullptr;
        for (size_t i = 0; i < expected; ++i)
            if (!std::isfinite(deltas[i]) || deltas[i] > 0.f)
                return nullptr;
        // The native routine needs one delta vector per ring, not the ABI flat array.
        std::vector<std::vector<float>> ring_deltas;
        ring_deltas.emplace_back(deltas, deltas + input.contour.size());
        size_t position = input.contour.size();
        for (const Slic3r::Polygon &hole : input.holes) {
            ring_deltas.emplace_back(deltas + position, deltas + position + hole.size());
            position += hole.size();
        }
        Slic3r::ExPolygons result = Slic3r::variable_offset_inner_ex(input, ring_deltas, 2.0);
        return Slic3r::store_expolygons(storage, std::move(result));
    } catch (...) {
        return nullptr;
    }
}

polygon_collection_handle *clipper_expand(storage_handle *storage,
    const polygon_collection_handle *polygons, double delta)
{
    if (storage == nullptr || polygons == nullptr || !std::isfinite(delta) || delta <= 0.)
        return nullptr;
    try {
        Slic3r::Polygons result = Slic3r::expand(*Slic3r::to_polygons(polygons), delta);
        return reinterpret_cast<polygon_collection_handle *>(
            Slic3r::to_storage(storage)->store(std::make_unique<Slic3r::Polygons>(std::move(result))));
    } catch (...) {
        return nullptr;
    }
}

clipper_shapes_handle *clipper_union_with_safety_offset(storage_handle *storage,
                                                        const clipper_shapes_handle *subject) {
    const Slic3r::ApiClipper::ClipperShapes *subject_shapes = Slic3r::to_shapes(subject);
    if (storage == nullptr || subject_shapes == nullptr)
        return nullptr;

    Slic3r::ClipperLib::Paths expanded = Slic3r::safety_offset_paths(*subject_shapes);

    Slic3r::ClipperLib::Clipper clipper;
    clipper.AddPaths(expanded, Slic3r::ClipperLib::ptSubject, true);

    Slic3r::ClipperLib::PolyTree result;
    clipper.Execute(Slic3r::ClipperLib::ctUnion,
                    result,
                    Slic3r::ClipperLib::pftNonZero,
                    Slic3r::ClipperLib::pftNonZero);

    return Slic3r::store_shape(storage, Slic3r::ApiClipper::make_polytree_shapes(std::move(result)));
}

clipper_shapes_handle *clipper_union2(storage_handle *storage,
                                      const clipper_shapes_handle *subject1,
                                      const clipper_shapes_handle *subject2)
{
    const Slic3r::ApiClipper::ClipperShapes *first = Slic3r::to_shapes(subject1);
    const Slic3r::ApiClipper::ClipperShapes *second = Slic3r::to_shapes(subject2);
    if (storage == nullptr || first == nullptr || second == nullptr)
        return nullptr;

    Slic3r::ClipperLib::Clipper clipper;
    first->add_to_clipper(clipper, Slic3r::ClipperLib::ptSubject, true);
    second->add_to_clipper(clipper, Slic3r::ClipperLib::ptSubject, true);

    Slic3r::ClipperLib::PolyTree result;
    clipper.Execute(Slic3r::ClipperLib::ctUnion, result, Slic3r::ClipperLib::pftNonZero, Slic3r::ClipperLib::pftNonZero);
    return Slic3r::store_shape(storage, Slic3r::ApiClipper::make_polytree_shapes(std::move(result)));
}

// Concatenation intentionally does not run a boolean operation. It creates a
// fresh PathListShapes accumulator containing the raw paths of both inputs.
clipper_shapes_handle *clipper_concat(storage_handle *storage,
                                      const clipper_shapes_handle *first,
                                      const clipper_shapes_handle *second)
{
    const Slic3r::ApiClipper::ClipperShapes *first_shapes = Slic3r::to_shapes(first);
    const Slic3r::ApiClipper::ClipperShapes *second_shapes = Slic3r::to_shapes(second);
    if (storage == nullptr || (first_shapes == nullptr && second_shapes == nullptr))
        return nullptr;

    Slic3r::ClipperLib::Paths paths = first_shapes != nullptr ? first_shapes->to_paths() : Slic3r::ClipperLib::Paths{};
    if (second_shapes != nullptr) {
        Slic3r::ClipperLib::Paths other = second_shapes->to_paths();
        paths.insert(paths.end(),
                     std::make_move_iterator(other.begin()),
                     std::make_move_iterator(other.end()));
    }
    return Slic3r::store_shape(storage, Slic3r::ApiClipper::make_path_list_shapes(std::move(paths)));
}

clipper_shapes_handle *clipper_concat_replace(storage_handle *storage,
                                              clipper_shapes_handle *first,
                                              const clipper_shapes_handle *second)
{
    if (storage == nullptr)
        return nullptr;
    // A null handle is an empty operand: keep first when there is nothing to
    // append, and copy second when there is nothing to append to.
    if (second == nullptr)
        return first;
    if (first == nullptr)
        return clipper_concat(storage, nullptr, second);

    Slic3r::PluginStorage *plugin_storage = Slic3r::to_storage(storage);
    Slic3r::ApiClipper::ClipperShapes *first_shapes = Slic3r::to_shapes(first);
    const Slic3r::ApiClipper::ClipperShapes *second_shapes = Slic3r::to_shapes(second);
    if (first_shapes == nullptr || second_shapes == nullptr)
        return nullptr;

    clipper_shapes_handle *result =
        reinterpret_cast<clipper_shapes_handle *>(first_shapes->concat(*second_shapes, *plugin_storage));
    if (result != nullptr && result != first)
        storage_free(storage, first);
    return result;
}

/* ---- Offset operations ------------------------------------------------
ClipperOffset works from raw Paths, so every ClipperShapes implementation first
materializes itself as Paths. Several closed rings from an ExPolygon or PolyTree
source are offset ring by ring (see "Offset strategy"); a lone ring, raw polygon
or path lists and open or closed-line paths use one grouped ClipperOffset. The result is again kept as a PolyTree-backed
ClipperShapes so it can be chained or converted later.
*/

clipper_shapes_handle *clipper_offset(storage_handle *storage,
                                      const clipper_shapes_handle *subject,
                                      double delta,
                                      clipper_join_type_t join_type,
                                      double miter_limit,
                                      clipper_end_type_t end_type)
{
    const Slic3r::ApiClipper::ClipperShapes *subject_shapes = Slic3r::to_shapes(subject);
    if (storage == nullptr || subject_shapes == nullptr)
        return nullptr;

    const Slic3r::ClipperLib::JoinType native_join = Slic3r::to_join_type(join_type);
    const Slic3r::ClipperLib::EndType native_end = Slic3r::to_end_type(end_type);
    const Slic3r::ClipperLib::Paths rings = subject_shapes->to_paths();
    // An empty path (for example an empty ExPolygon) is not a ring.
    const std::ptrdiff_t ring_count = std::count_if(rings.begin(), rings.end(),
        [](const Slic3r::ClipperLib::Path &ring) { return !ring.empty(); });
    Slic3r::ClipperLib::PolyTree result;
    if (native_end == Slic3r::ClipperLib::etClosedPolygon && subject_shapes->has_normalized_rings() &&
        ring_count > 1) {
        result = Slic3r::offset_closed_rings(rings, delta, native_join, miter_limit);
    } else {
        Slic3r::ClipperLib::ClipperOffset offsetter;
        Slic3r::configure_offsetter(offsetter, delta, native_join, miter_limit);
        offsetter.AddPaths(rings, native_join, native_end);
        offsetter.Execute(result, delta);
    }
    if (Slic3r::ApiClipper::has_overlapping_edges(result))
        result = Slic3r::united_polytree(std::move(result));
    return Slic3r::store_shape(storage, Slic3r::ApiClipper::make_polytree_shapes(std::move(result)));
}

clipper_shapes_handle *clipper_offset2(storage_handle *storage,
                                       const clipper_shapes_handle *subject,
                                       double delta1,
                                       double delta2,
                                       clipper_join_type_t join_type,
                                       double miter_limit,
                                       clipper_end_type_t end_type)
{
    // Store only the final result for the caller. The intermediate offset is
    // explicitly freed from the same storage before returning.
    clipper_shapes_handle *first = clipper_offset(storage, subject, delta1, join_type, miter_limit, end_type);
    clipper_shapes_handle *second = clipper_offset(storage, first, delta2, join_type, miter_limit, end_type);
    storage_free(storage, first);
    return second;
}

// Delegate to the native per-polyline offset and union so results match ClipperUtils point for point; the united polygons are copied into a path list.
clipper_shapes_handle *clipper_offset_polylines(storage_handle *storage,
                                                const polyline_collection_handle *polylines,
                                                double delta,
                                                clipper_join_type_t join_type,
                                                double miter_limit,
                                                clipper_end_type_t end_type)
{
    if (storage == nullptr || polylines == nullptr || delta <= 0.)
        return nullptr;
    const Slic3r::Polygons offset = Slic3r::offset(*Slic3r::to_polylines(polylines), delta,
                                                   Slic3r::to_join_type(join_type), miter_limit,
                                                   Slic3r::to_end_type(end_type));
    Slic3r::ClipperLib::Paths paths;
    paths.reserve(offset.size());
    for (const Slic3r::Polygon &polygon : offset)
        paths.emplace_back(polygon.points);
    return Slic3r::store_shape(storage, Slic3r::ApiClipper::make_path_list_shapes(std::move(paths)));
}

expolygon_collection_handle *clipper_variable_offset_inner(
    storage_handle *storage, const expolygon_handle *source,
    const float *deltas, uint32_t delta_count, double miter_limit)
{
    if (storage == nullptr || source == nullptr || (delta_count != 0 && deltas == nullptr) ||
        !std::isfinite(miter_limit) || miter_limit < 1.)
        return nullptr;
    const Slic3r::ExPolygon &input = *Slic3r::to_expolygon(source);
    size_t count = input.contour.size();
    for (const auto &hole : input.holes)
        count += hole.size();
    if (count != delta_count)
        return nullptr;
    if (input.contour.empty()) {
        return input.holes.empty() ? storage_new_expolygons(storage) : nullptr;
    }
    if (input.contour.size() < 3 || !input.contour.is_counter_clockwise())
        return nullptr;
    for (const auto &hole : input.holes)
        if (hole.size() < 3 || !hole.is_clockwise())
            return nullptr;

    double maximum_delta = 0.;
    for (uint32_t index = 0; index < delta_count; ++index) {
        if (!std::isfinite(deltas[index]) || deltas[index] > 0.f)
            return nullptr;
        maximum_delta = std::max(maximum_delta, -double(deltas[index]));
    }
    // Reject unsafe margins before any variable-offset coordinate arithmetic.
    // A double rounds hiRange up, so use the next lower representable value.
    const double limit = std::nextafter(double(Slic3r::ClipperLib::hiRange), 0.);
    const double margin = std::ceil(maximum_delta * std::max(2., miter_limit)) + 2. * Slic3r::SCALED_EPSILON;
    if (!std::isfinite(margin) || margin > limit)
        return nullptr;
    const auto bounds = Slic3r::get_extents(input);
    const double safe = limit - margin;
    if (double(bounds.min.x()) < -safe || double(bounds.min.y()) < -safe ||
        double(bounds.max.x()) > safe || double(bounds.max.y()) > safe)
        return nullptr;

    // Split the flat ABI array into one delta vector per ring: contour first, then holes.
    // delta_count was checked above against the total vertex count, so every slice stays in bounds.
    std::vector<std::vector<float>> ring_deltas;
    ring_deltas.reserve(input.holes.size() + 1);

    const float *const contour_ring_begin = deltas;
    const float *const contour_ring_end = contour_ring_begin + input.contour.size();
    ring_deltas.emplace_back(contour_ring_begin, contour_ring_end);

    const float *hole_ring_begin = contour_ring_end;
    for (const Slic3r::Polygon &hole_ring : input.holes) {
        const float *const hole_ring_end = hole_ring_begin + hole_ring.size();
        ring_deltas.emplace_back(hole_ring_begin, hole_ring_end);
        hole_ring_begin = hole_ring_end;
    }
    assert(hole_ring_begin == deltas + delta_count);
    Slic3r::ExPolygons output = Slic3r::variable_offset_inner_ex(input, ring_deltas, miter_limit);
    auto *result = storage_new_expolygons(storage);
    *Slic3r::to_expolygons(result) = std::move(output);
    return result;
}

/* ---- Conversion back to public geometry handles ----------------------
These functions materialize the abstract ClipperShapes result into normal ABI
collections. The output collections are also storage-owned and can be returned
from plugin code or freed with storage_free().
*/

polygon_collection_handle *clipper_shapes_to_polygons(storage_handle *storage, const clipper_shapes_handle *shapes) {
    const Slic3r::ApiClipper::ClipperShapes *source = Slic3r::to_shapes(shapes);
    if (storage == nullptr || source == nullptr)
        return nullptr;

    polygon_collection_handle *out_handle = storage_new_polygons(storage);
    Slic3r::Polygons *out = Slic3r::to_polygons(out_handle);
    out->clear();
    for (const Slic3r::Polygon &polygon : source->to_polygons())
        out->emplace_back(polygon);
    return out_handle;
}

expolygon_collection_handle *clipper_shapes_to_expolygons(storage_handle *storage, const clipper_shapes_handle *shapes) {
    const Slic3r::ApiClipper::ClipperShapes *source = Slic3r::to_shapes(shapes);
    if (storage == nullptr || source == nullptr)
        return nullptr;

    expolygon_collection_handle *out_handle = storage_new_expolygons(storage);
    *Slic3r::to_expolygons(out_handle) = source->to_expolygons();
    return out_handle;
}

void clipper_shapes_replace_polygons(polygon_collection_handle *dst, const clipper_shapes_handle *shapes) {
    const Slic3r::ApiClipper::ClipperShapes *source = Slic3r::to_shapes(shapes);
    if (dst == nullptr || source == nullptr)
        return;

    Slic3r::Polygons &out = *Slic3r::to_polygons(dst);
    // TODO: optimiser, en réutilisant les objets MultiPoint déjà alloués dans la collection de destination, plutôt
    // que de tout clear() et ré-emplace_back() à chaque fois. Mais cela demande de faire de nouvelles méthodes dans source
    // pour remplir directement un object déjà crée.
    out.clear();
    for (const Slic3r::Polygon &polygon : source->to_polygons())
        out.emplace_back(polygon);
}

void clipper_shapes_replace_expolygons(expolygon_collection_handle *dst, const clipper_shapes_handle *shapes) {
    const Slic3r::ApiClipper::ClipperShapes *source = Slic3r::to_shapes(shapes);
    if (dst == nullptr || source == nullptr)
        return;

    *Slic3r::to_expolygons(dst) = source->to_expolygons();
}

expolygon_collection_handle *clipper_clip_expolygons_with_subject_bbox(storage_handle *storage,
                                                                       const expolygon_collection_handle *src,
                                                                       c_bounding_box bbox)
{
    if (storage == nullptr || src == nullptr)
        return nullptr;

    expolygon_collection_handle *out_handle = storage_new_expolygons(storage);
    *Slic3r::to_expolygons(out_handle) =
        Slic3r::ClipperUtils::clip_clipper_expolygons_with_subject_bbox(*Slic3r::to_expolygons(src),
                                                                        Slic3r::to_bounding_box(bbox));
    return out_handle;
}

clipper_shapes_handle *clipper_clip_shapes_with_subject_bbox(storage_handle *storage,
                                                             const clipper_shapes_handle *src,
                                                             c_bounding_box bbox)
{
    const Slic3r::ApiClipper::ClipperShapes *source = Slic3r::to_shapes(src);
    if (storage == nullptr || source == nullptr)
        return nullptr;

    // This is a pre-filter for later Clipper operations. It deliberately keeps a
    // flat path list instead of rebuilding ExPolygon topology, because the next
    // boolean operation will rebuild the final contour/hole tree if needed.
    Slic3r::Polygons polygons;
    for (const Slic3r::ClipperLib::Path &path : source->to_paths()) {
        Slic3r::Polygon polygon;
        polygon.points.reserve(path.size());
        for (const Slic3r::ClipperLib::IntPoint &point : path)
            polygon.points.emplace_back(point.x(), point.y());
        polygons.emplace_back(std::move(polygon));
    }

    Slic3r::Polygons clipped =
        Slic3r::ClipperUtils::clip_clipper_polygons_with_subject_bbox(polygons,
                                                                      Slic3r::to_bounding_box(bbox));

    Slic3r::ClipperLib::Paths paths;
    paths.reserve(clipped.size());
    for (const Slic3r::Polygon &polygon : clipped) {
        Slic3r::ClipperLib::Path path;
        path.reserve(polygon.points.size());
        for (const Slic3r::Point &point : polygon.points)
            path.emplace_back(point.x(), point.y());
        paths.emplace_back(std::move(path));
    }

    return Slic3r::store_shape(storage, Slic3r::ApiClipper::make_path_list_shapes(std::move(paths)));
}

polyline_collection_handle *clipper_diff_polyline_expolygons(storage_handle *storage,
                                                             const polyline_handle *subject,
                                                             const expolygon_collection_handle *clip)
{
    if (storage == nullptr || subject == nullptr)
        return nullptr;

    polyline_collection_handle *out_handle = storage_new_polylines(storage);
    Slic3r::Polylines &out = *Slic3r::to_polylines(out_handle);
    if (clip == nullptr || Slic3r::to_expolygons(clip)->empty())
        out.push_back(*Slic3r::to_polyline(subject));
    else
        out = Slic3r::diff_pl(*Slic3r::to_polyline(subject), *Slic3r::to_expolygons(clip));
    return out_handle;
}

polyline_collection_handle *clipper_diff_polylines_expolygons(storage_handle *storage,
                                                             const polyline_collection_handle *subject,
                                                             const expolygon_collection_handle *clip)
{
    if (storage == nullptr || subject == nullptr)
        return nullptr;

    polyline_collection_handle *out_handle = storage_new_polylines(storage);
    Slic3r::Polylines &out = *Slic3r::to_polylines(out_handle);
    if (clip == nullptr || Slic3r::to_expolygons(clip)->empty())
        out = *Slic3r::to_polylines(subject);
    else if (!Slic3r::to_polylines(subject)->empty())
        out = Slic3r::diff_pl(*Slic3r::to_polylines(subject), *Slic3r::to_expolygons(clip));
    return out_handle;
}

polyline_collection_handle *clipper_intersection_polyline_expolygons(storage_handle *storage,
                                                                     const polyline_handle *subject,
                                                                     const expolygon_collection_handle *clip)
{
    if (storage == nullptr || subject == nullptr)
        return nullptr;

    polyline_collection_handle *out_handle = storage_new_polylines(storage);
    Slic3r::Polylines &out = *Slic3r::to_polylines(out_handle);
    if (clip != nullptr && !Slic3r::to_expolygons(clip)->empty())
        out = Slic3r::intersection_pl(*Slic3r::to_polyline(subject), *Slic3r::to_expolygons(clip));
    return out_handle;
}

polyline_collection_handle *clipper_intersection_polylines_expolygons(storage_handle *storage,
                                                                     const polyline_collection_handle *subject,
                                                                     const expolygon_collection_handle *clip)
{
    if (storage == nullptr || subject == nullptr)
        return nullptr;

    polyline_collection_handle *out_handle = storage_new_polylines(storage);
    Slic3r::Polylines &out = *Slic3r::to_polylines(out_handle);
    if (!Slic3r::to_polylines(subject)->empty() && clip != nullptr && !Slic3r::to_expolygons(clip)->empty())
        out = Slic3r::intersection_pl(*Slic3r::to_polylines(subject), *Slic3r::to_expolygons(clip));
    return out_handle;
}

int32_t clipper_split_polyline_expolygons(storage_handle *storage,
                                          const polyline_handle *subject,
                                          const expolygon_collection_handle *clip,
                                          polyline_collection_handle **inside,
                                          polyline_collection_handle **outside)
{
    if (storage == nullptr || subject == nullptr || inside == nullptr || outside == nullptr)
        return 0;

    polyline_collection_handle *inside_handle = storage_new_polylines(storage);
    polyline_collection_handle *outside_handle = storage_new_polylines(storage);
    if (clip == nullptr || Slic3r::to_expolygons(clip)->empty()) {
        Slic3r::to_polylines(outside_handle)->push_back(*Slic3r::to_polyline(subject));
    } else {
        std::pair<Slic3r::Polylines, Slic3r::Polylines> split =
            Slic3r::split_pl(*Slic3r::to_polyline(subject), *Slic3r::to_expolygons(clip));
        *Slic3r::to_polylines(inside_handle) = std::move(split.first);
        *Slic3r::to_polylines(outside_handle) = std::move(split.second);
    }
    *inside = inside_handle;
    *outside = outside_handle;
    return 1;
}

} // extern "C"
