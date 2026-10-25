///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_clipper_h_
#define slic3r_clipper_h_

#define SLIC3R_PLUGIN_API_CLIPPER_MAJOR 1u
#define SLIC3R_PLUGIN_API_CLIPPER_MINOR 3u

#include "slic3r_def.h"
#include "slic3r_geometry.h"
#include "slic3r_utils.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct clipper_shapes_handle clipper_shapes_handle;

/*
Opaque geometry adapter used by the Clipper ABI.

The goal of this handle is to avoid exposing a combinatorial set of functions
for Polygon / Polygons / ExPolygon / ExPolygons. A clipper_shapes_handle is a
single Clipper-compatible source of paths. It may either reference an existing
geometry handle, or own a Clipper result stored in the provided storage_handle.

Lifetime rules:
- every clipper_shapes_handle returned by this API is owned by storage;
- release it with storage_free(storage, handle) when the intermediate result is
  no longer needed;
- handles created from existing geometry do not copy that geometry, so the
  original polygon / expolygon collection must stay valid while the shapes
  handle is used.
*/

typedef enum clipper_operation_t {
    CLIPPER_OPERATION_DIFFERENCE = 0,
    CLIPPER_OPERATION_INTERSECTION = 1,
    CLIPPER_OPERATION_UNION = 2,
    CLIPPER_OPERATION_XOR = 3
} clipper_operation_t;

typedef enum clipper_join_type_t {
    CLIPPER_JOIN_SQUARE = 0,
    CLIPPER_JOIN_ROUND = 1,
    CLIPPER_JOIN_MITER = 2
} clipper_join_type_t;

typedef enum clipper_end_type_t {
    CLIPPER_END_CLOSED_POLYGON = 0,
    CLIPPER_END_CLOSED_LINE = 1,
    CLIPPER_END_OPEN_BUTT = 2,
    CLIPPER_END_OPEN_SQUARE = 3,
    CLIPPER_END_OPEN_ROUND = 4
} clipper_end_type_t;

/* ---- Shape adapters -------------------------------------------------
Create a Clipper input adapter from an existing geometry handle.

These functions allocate only the small adapter object in storage. They do not
copy the source geometry. This keeps conversion cheap, but also means mutating or
freeing the source geometry before using the clipper_shapes_handle is invalid.
*/
SLIC3R_HOST_API clipper_shapes_handle *clipper_shapes_create_empty(storage_handle *storage);
SLIC3R_HOST_API clipper_shapes_handle *clipper_shapes_from_polygon(storage_handle *storage, const polygon_handle *polygon);
SLIC3R_HOST_API clipper_shapes_handle *clipper_shapes_from_polyline(storage_handle *storage, const polyline_handle *polyline);
SLIC3R_HOST_API clipper_shapes_handle *clipper_shapes_from_polygons(storage_handle *storage, const polygon_collection_handle *polygons);
SLIC3R_HOST_API clipper_shapes_handle *clipper_shapes_from_expolygon(storage_handle *storage, const expolygon_handle *expolygon);
SLIC3R_HOST_API clipper_shapes_handle *clipper_shapes_from_expolygons(storage_handle *storage, const expolygon_collection_handle *expolygons);

/* Return non-zero when shapes is NULL or contains no usable paths/geometry. */
SLIC3R_HOST_API int32_t clipper_shapes_empty(const clipper_shapes_handle *shapes);

/* Count non-empty raw Clipper paths. Holes are paths too. */
SLIC3R_HOST_API uint32_t clipper_shapes_path_count(const clipper_shapes_handle *shapes);

/* Return the bounding box of all points represented by shapes, or a zero box if shapes is NULL or empty. */
SLIC3R_HOST_API c_bounding_box clipper_shapes_bounding_box(const clipper_shapes_handle *shapes);

/* ---- Boolean operations ---------------------------------------------
Run a Clipper boolean operation and return a new clipper_shapes_handle.

The returned handle owns the Clipper PolyTree result through storage. It can be
used as input to another Clipper operation, converted to Polygons/ExPolygons, or
released with storage_free().

clip may be NULL for operations that accept a single subject, such as union.
All boolean operations currently use pftNonZero for subject and clip fill types,
matching the common Slic3r ClipperUtils behaviour.
*/
SLIC3R_HOST_API clipper_shapes_handle *clipper_execute(storage_handle *storage,
                                                       clipper_operation_t operation,
                                                       const clipper_shapes_handle *subject,
                                                       const clipper_shapes_handle *clip);
SLIC3R_HOST_API clipper_shapes_handle *clipper_diff(storage_handle *storage,
                                                    const clipper_shapes_handle *subject,
                                                    const clipper_shapes_handle *clip);
SLIC3R_HOST_API clipper_shapes_handle *clipper_intersection(storage_handle *storage,
                                                           const clipper_shapes_handle *subject,
                                                           const clipper_shapes_handle *clip);

/*
subject intersection clip and subject - clip at once: *inside receives the part
of subject inside clip and *outside the part outside it, as two new
storage-owned shapes (release each with storage_free()). Both are computed from
the same inputs, so they share their boundary along clip. Each operand keeps
its kind: a polyline subject is clipped as an open path, as with
clipper_intersection() and clipper_diff(). Each side is
cleaned at SCALED_EPSILON like any Clipper result, so the two may differ from
an exact tiling by less than that distance. clip == NULL gives an empty *inside
and the union of subject in *outside.
Returns 1 on success. Returns 0 and leaves both outputs unchanged when storage,
subject, inside or outside is NULL.
*/
SLIC3R_HOST_API int32_t clipper_split(storage_handle *storage,
                                      const clipper_shapes_handle *subject,
                                      const clipper_shapes_handle *clip,
                                      clipper_shapes_handle **inside,
                                      clipper_shapes_handle **outside);
/*
Run a boolean operation after applying the Clipper safety offset to clip only.

This mirrors Slic3r's ApplySafetyOffset::Yes behaviour:
- difference becomes subject - offset(clip, ClipperSafetyOffset);
- intersection becomes subject & offset(clip, ClipperSafetyOffset).

The expanded clip is an internal temporary path list, not a storage-owned handle,
so callers only need to free the returned result.
*/
SLIC3R_HOST_API clipper_shapes_handle *clipper_diff_with_safety_offset(storage_handle *storage,
                                                                       const clipper_shapes_handle *subject,
                                                                       const clipper_shapes_handle *clip);
SLIC3R_HOST_API clipper_shapes_handle *clipper_intersection_with_safety_offset(storage_handle *storage,
                                                                              const clipper_shapes_handle *subject,
                                                                              const clipper_shapes_handle *clip);
SLIC3R_HOST_API clipper_shapes_handle *clipper_union(storage_handle *storage, const clipper_shapes_handle *subject);
SLIC3R_HOST_API clipper_shapes_handle *clipper_union_with_safety_offset(storage_handle *storage, const clipper_shapes_handle *subject);
SLIC3R_HOST_API clipper_shapes_handle *clipper_union2(storage_handle *storage,
                                                      const clipper_shapes_handle *subject1,
                                                      const clipper_shapes_handle *subject2);

/*
Concatenate the raw paths from two shapes without performing a geometric union.

This is the closest equivalent to "append then union later".
The returned handle is always a new storage-owned
ClipperShapes value backed by a flat path accumulator (PathListShapes)
containing the paths of both inputs.
*/
SLIC3R_HOST_API clipper_shapes_handle *clipper_concat(storage_handle *storage,
                                                      const clipper_shapes_handle *first,
                                                      const clipper_shapes_handle *second);

/*
Concatenate and replace one handle in a single call.

This helper asks first to perform the concatenation work itself. The default
behaviour is equivalent to clipper_concat(storage, first, second), but mutable
path-accumulator implementations may append directly into themselves and return
their own handle instead.

The caller shall always keep using the returned handle value:

    first = clipper_concat_replace(storage, first, second);

A null first or second is treated as an empty operand: a null second returns
first unchanged, and a null first returns a new handle with the paths of second.

Important limitation:
- this only updates the handle value returned to the caller;
- if other variables still store the old first pointer, they are not updated and
  become invalid if first was freed here.

In other words, use this only when first is treated as the unique current owner
of that handle value.
*/
SLIC3R_HOST_API clipper_shapes_handle *clipper_concat_replace(storage_handle *storage,
                                                              clipper_shapes_handle *first,
                                                              const clipper_shapes_handle *second);

/* ---- Offset operations -----------------------------------------------
Offset a Clipper shape and return a new clipper_shapes_handle.

delta follows ClipperOffset semantics: positive expands closed polygons and
negative shrinks them. For open paths, pass an open end type.

miter_limit is used as MiterLimit for square/miter joins and as ArcTolerance for
round joins, following the convention already used by ClipperUtils.

Rings meeting exactly along an edge after the offset are split as the native
per-ring offset_ex() splits them: the result has no zero-width bridge.
*/
SLIC3R_HOST_API clipper_shapes_handle *clipper_offset(storage_handle *storage,
                                                      const clipper_shapes_handle *subject,
                                                      double delta,
                                                      clipper_join_type_t join_type,
                                                      double miter_limit,
                                                      clipper_end_type_t end_type);
SLIC3R_HOST_API clipper_shapes_handle *clipper_offset2(storage_handle *storage,
                                                       const clipper_shapes_handle *subject,
                                                       double delta1,
                                                       double delta2,
                                                       clipper_join_type_t join_type,
                                                       double miter_limit,
                                                       clipper_end_type_t end_type);

/*
Offset a collection of open polylines into closed areas, exactly as the native
offset(Polylines) of ClipperUtils: each polyline is offset alone, then all the
results are united with a non-zero fill. The native defaults for lines are
CLIPPER_JOIN_SQUARE, miter_limit 0 (ignored for square joins) and
CLIPPER_END_OPEN_BUTT. delta is the scaled half width of the result and must be
positive.

Returns a new storage-owned shape holding the united polygons (release it with
storage_free()), ready for another Clipper operation or a conversion. Returns
NULL when storage or polylines is NULL or delta <= 0. Unlike
clipper_shapes_from_polyline(), the polylines are read during the call only.
*/
SLIC3R_HOST_API clipper_shapes_handle *clipper_offset_polylines(storage_handle *storage,
                                                                const polyline_collection_handle *polylines,
                                                                double delta,
                                                                clipper_join_type_t join_type,
                                                                double miter_limit,
                                                                clipper_end_type_t end_type);

/* ---- Conversion back to ABI collections ------------------------------
Materialize a Clipper shape into a new storage-owned collection.

clipper_shapes_to_polygons() returns a polygon_collection_handle where each
item is intended to be interpreted as a Polygon.

clipper_shapes_to_expolygons() returns a real expolygon_collection_handle.
Both returned handles are owned by storage and may be released with
storage_free().
*/
SLIC3R_HOST_API polygon_collection_handle *clipper_shapes_to_polygons(storage_handle *storage, const clipper_shapes_handle *shapes);
SLIC3R_HOST_API expolygon_collection_handle *clipper_shapes_to_expolygons(storage_handle *storage, const clipper_shapes_handle *shapes);

/*
Replace an existing collection with the materialized contents of shapes.

These functions are useful when the caller already owns a suitable output
collection and wants to avoid allocating another storage object. The destination
collection is cleared first, then filled with the converted result.

dst is not freed or replaced; only its contents are changed.
*/
SLIC3R_HOST_API void clipper_shapes_replace_polygons(polygon_collection_handle *dst, const clipper_shapes_handle *shapes);
SLIC3R_HOST_API void clipper_shapes_replace_expolygons(expolygon_collection_handle *dst, const clipper_shapes_handle *shapes);

/*
Clip an ExPolygon collection to a subject bounding box and return a new
storage-owned ExPolygon collection.

Use this as a cheap pre-pass before expensive boolean operations when a small
subject is compared against a large set of possible clip polygons. It keeps only
the parts of src that may overlap bbox, reducing both the number of paths and
their point count before diff/intersection/union work.

src is copied into the returned collection after clipping. The returned handle
is owned by storage and must be released with storage_free() when no longer
needed.
*/
SLIC3R_HOST_API expolygon_collection_handle *clipper_clip_expolygons_with_subject_bbox(storage_handle *storage,
                                                                                       const expolygon_collection_handle *src,
                                                                                       c_bounding_box bbox);

/*
Clip a generic Clipper shape to a subject bounding box and return a new
storage-owned shape handle.

Use this before an expensive boolean operation when the clip side is much larger
than the subject. Unlike clipper_clip_expolygons_with_subject_bbox(), this keeps
the result in the Clipper shape pipeline, so callers can continue with
clipper_intersection(), clipper_diff(), clipper_union(), or offsets without first
materializing an ExPolygon collection.

The returned shape is a flat path-list shape. It is intended as an optimization
for subsequent Clipper operations, not as a topology-preserving ExPolygon
materialization. Convert to ExPolygons only after the final boolean operation
when contour/hole hierarchy is required.
*/
SLIC3R_HOST_API clipper_shapes_handle *clipper_clip_shapes_with_subject_bbox(storage_handle *storage,
                                                                             const clipper_shapes_handle *src,
                                                                             c_bounding_box bbox);

/*
Clip an open polyline by polygonal areas and return the remaining open pieces.

This is the polyline equivalent of subject - clip. Two pieces cut less than
SCALED_EPSILON apart (by a clip thinner than that) are glued back into one.
It is intentionally exposed
as a direct helper instead of a generic ClipperOperand conversion because open
paths materialize as polylines, not as polygon/expolygon collections.

The returned polyline collection is owned by storage and must be released with
storage_free() when no longer needed.
*/
SLIC3R_HOST_API polyline_collection_handle *clipper_diff_polyline_expolygons(storage_handle *storage,
                                                                             const polyline_handle *subject,
                                                                             const expolygon_collection_handle *clip);

// Batch difference returns storage-owned, unordered paths; a null/empty mask copies subjects.
SLIC3R_HOST_API polyline_collection_handle *clipper_diff_polylines_expolygons(storage_handle *storage,
                                                                          const polyline_collection_handle *subject,
                                                                          const expolygon_collection_handle *clip);

/*
Clip an open polyline by polygonal areas and return the pieces inside clip.

This is the polyline equivalent of subject intersection clip. Two pieces cut
less than SCALED_EPSILON apart (by a gap thinner than that between clip areas)
are glued back into one. It is mostly used
by post-process plugins that need to split one extrusion into "inside this
region setting" and "outside this region setting" fragments while keeping the
original extrusion order.

clip == NULL or an empty collection returns an empty collection: there is no
area where the polyline is accepted. The returned polyline collection is owned
by storage and must be released with storage_free() when no longer needed.
*/
SLIC3R_HOST_API polyline_collection_handle *clipper_intersection_polyline_expolygons(
    storage_handle *storage,
    const polyline_handle *subject,
    const expolygon_collection_handle *clip);

// Batch intersection returns storage-owned, unordered paths; a null/empty mask gives an empty result.
SLIC3R_HOST_API polyline_collection_handle *clipper_intersection_polylines_expolygons(
    storage_handle *storage,
    const polyline_collection_handle *subject,
    const expolygon_collection_handle *clip);

/*
Clip an open polyline by polygonal areas and return both sides at once: the
pieces inside clip (as clipper_intersection_polyline_expolygons()) and the
pieces outside it (as clipper_diff_polyline_expolygons()).

Both sides are computed from the same prepared input, so they are cut at the
same points: every end of a piece that is not an end of subject is also an end
of a piece on the other side. When two cuts are less than SCALED_EPSILON apart,
the tiny piece between them is dropped and the two pieces around it are glued
back into one, so both sides still tile subject. A piece shorter than
SCALED_EPSILON at an end of subject is kept; the caller decides what to do with
it. The order and direction of the pieces are unspecified.

*inside and *outside receive new polyline collections owned by storage; release
each with storage_free() when no longer needed. clip == NULL or an empty
collection gives an empty *inside and a copy of subject in *outside.
Returns 1 on success. Returns 0 and leaves both outputs unchanged when storage,
subject, inside or outside is NULL.
*/
SLIC3R_HOST_API int32_t clipper_split_polyline_expolygons(storage_handle *storage,
                                                          const polyline_handle *subject,
                                                          const expolygon_collection_handle *clip,
                                                          polyline_collection_handle **inside,
                                                          polyline_collection_handle **outside);

#ifdef __cplusplus
}
#endif

#endif
