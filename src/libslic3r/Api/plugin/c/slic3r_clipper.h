///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_clipper_h_
#define slic3r_clipper_h_

#define SLIC3R_PLUGIN_API_CLIPPER_MAJOR 2u
#define SLIC3R_PLUGIN_API_CLIPPER_MINOR 3u

#include "slic3r_def.h"
#include "slic3r_geometry.h"
#include "slic3r_utils.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct clipper_shapes_handle clipper_shapes_handle;

/*
Polygon clipping and offsets for plugin authors.

Use geometry handles from slic3r_geometry.h and temporary storage from
slic3r_utils.h. Two routes are available. For a sequence of operations, wrap
geometry in clipper_shapes_handle adapters, apply booleans or offsets, then
materialize the result with clipper_shapes_to_polygons or
clipper_shapes_to_expolygons. For native core
results, call the native polygon functions below directly with geometry handles;
they preserve the native result order and processing, without an adapter step.

Keep borrowed geometry alive and unchanged throughout each call. Storage locks
protect its registry of owned objects, not geometry access. Do not mutate or
destroy an input or result while another thread uses it. Clear or destroy storage
only after all users of its handles have finished. Native-result functions catch
exceptions and return NULL; other entry points do not provide that guarantee.

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

/* Clipper polygon fill rule, the ClipperLib::PolyFillType of the native code. */
typedef enum clipper_fill_rule_t {
    CLIPPER_FILL_NONZERO = 0,
    CLIPPER_FILL_EVENODD = 1,
    CLIPPER_FILL_POSITIVE = 2
} clipper_fill_rule_t;

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
The boolean operations use pftNonZero for subject and clip fill types, matching the
common Slic3r ClipperUtils behaviour, except clipper_union(), which takes the fill
rule as an argument.
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
/*
Union of subject, with fill_rule applied to its paths (CLIPPER_FILL_NONZERO is the
usual Slic3r union). Returns NULL when storage or subject is NULL or fill_rule is
not one of the clipper_fill_rule_t values. The result is a new storage-owned shape
(release it with storage_free()).
*/
SLIC3R_HOST_API clipper_shapes_handle *clipper_union(storage_handle *storage,
                                                     const clipper_shapes_handle *subject,
                                                     clipper_fill_rule_t fill_rule);
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

/* ---- Native polygon results -----------------------------------------
The functions below call the native ClipperUtils code and return its result
as is: a new polygon or expolygon collection owned by storage (release it with
storage_free()), preserving the native order of polygons or expolygons and their rings, with no
extra cleaning. They return NULL when storage or an input handle is NULL, when an
enum value is unknown, or when a double argument is not finite, and also on an
internal error (for example ClipperLib rejecting coordinates outside its allowed
range, or an allocation failure), with nothing added to the storage. Prefer them over
the clipper_shapes_handle pipeline when the result must match the native code
exactly (same rings, same order, same start points).

delta, delta1 and delta2 are in scaled units. The native defaults are
CLIPPER_JOIN_MITER with miter_limit 3.0 (DefaultJoinType and DefaultMiterLimit of
ClipperUtils.hpp). For CLIPPER_JOIN_ROUND, miter_limit is the ArcTolerance, as in
the native code. Negative offsets assume normalised input, as in the native code:
no overlap or intersection between the input expolygons (ClipperUtils.hpp, shrink()
and offset2()).
*/

/*
Native union_ex(Polygons, fill_type): two passes with small-area removal, the
polygon tree converted with each contour before its islands. The conversion also
drops degenerate contours (clockwise artifacts, 3-point contours with coinciding
points; ClipperUtils.cpp PolyTreeToExPolygons), not only small areas.
Returns NULL for a NULL argument or an unknown rule, and also on an internal error
(for example ClipperLib rejecting coordinates outside its allowed range, or an
allocation failure); nothing is added to the storage in that case.
*/
SLIC3R_HOST_API expolygon_collection_handle *clipper_union_ex(storage_handle *storage,
                                                              const polygon_collection_handle *polygons,
                                                              clipper_fill_rule_t fill_rule);

/*
Native offset_ex(ExPolygons, delta, join_type, miter_limit). The Slic3r safety
offset, the native union_safety_offset_ex(ExPolygons), is
clipper_offset_ex(storage, src, 10.0 (ClipperSafetyOffset), CLIPPER_JOIN_MITER, 3.0).
delta == 0 is accepted (the native offset_ex does not assert on it for ExPolygons).
Returns NULL for invalid arguments and also on an internal error (see clipper_union_ex()).
*/
SLIC3R_HOST_API expolygon_collection_handle *clipper_offset_ex(storage_handle *storage,
                                                               const expolygon_collection_handle *src,
                                                               double delta,
                                                               clipper_join_type_t join_type,
                                                               double miter_limit);

/*
Native offset2_ex(ExPolygons, delta1, delta2, join_type, miter_limit): offset by
delta1, then by delta2. Closing is delta1 > 0 then delta2 = -delta1; opening is the
reverse. delta2 must not be 0 (the native code asserts it; NULL is returned): use
clipper_offset_ex() for a single offset. delta1 == 0 is accepted.
Also returns NULL on an internal error (see clipper_union_ex()).
*/
SLIC3R_HOST_API expolygon_collection_handle *clipper_offset2_ex(storage_handle *storage,
                                                                const expolygon_collection_handle *src,
                                                                double delta1,
                                                                double delta2,
                                                                clipper_join_type_t join_type,
                                                                double miter_limit);

/* Variable inward offset of one normalized ExPolygon.
deltas contains one scaled distance per vertex: first all contour vertices,
then the vertices of each hole in order. Distances must be finite and <= 0.
delta_count must match that complete vertex count. miter_limit must be finite
and >= 1. Returns NULL for invalid arguments or unsafe coordinate margins.
An empty result collection is a successful geometric collapse. Source and deltas
are borrowed for this call only; the returned collection is owned by storage. */
SLIC3R_HOST_API expolygon_collection_handle *clipper_variable_offset_inner(
    storage_handle *storage, const expolygon_handle *source,
    const float *deltas, uint32_t delta_count, double miter_limit);
/* Subtract clip from subject with native diff_ex, without safety offset.
The native operation removes tiny or narrow rings and then unions the remaining
paths to rebuild contour/hole topology. Even an empty clip can therefore remove
small subject parts. Empty collections are accepted; NULL storage, subject or
clip is not. Inputs are borrowed, unchanged, only during the call.
Returns storage-owned ExPolygons in native order; release with storage_free().
Invalid arguments or internal errors return NULL, store no result and let no
exception cross the C boundary. Inputs must not be edited concurrently. */
SLIC3R_HOST_API expolygon_collection_handle *clipper_diff_ex(storage_handle *storage,
    const expolygon_collection_handle *subject, const expolygon_collection_handle *clip);
/* Intersect subject and clip with native intersection_ex, without safety offset.
Removes tiny or narrow output rings before a union rebuilds contour/hole topology.
Empty collections are accepted and an empty clip gives an empty result. Native
order, borrowing, thread, ownership and NULL/error rules are as for clipper_diff_ex. */
SLIC3R_HOST_API expolygon_collection_handle *clipper_intersection_ex(storage_handle *storage,
    const expolygon_collection_handle *subject, const expolygon_collection_handle *clip);

/* Inset the filled area: the contour shrinks and holes grow; the result may
split or vanish. Uses native variable_offset_inner_ex with miter limit 2.0.
Deltas are scaled
distances, flattened contour first, then holes in order, one float per vertex.
expolygon and deltas are borrowed only for the duration of the call; the caller
keeps ownership and neither is retained after return.
Every delta must be finite and <= 0; count must equal the total vertex count.
The contour must have strictly positive area and every hole strictly negative
area. Input is neither reoriented nor cleaned. Returns storage-owned ExPolygons,
or NULL for NULL storage, expolygon or deltas, a count/area/delta violation,
or an internal error. Keep inputs unchanged during the call.
On failure the function returns NULL, stores no result and lets no exception
cross the C boundary. */
SLIC3R_HOST_API expolygon_collection_handle *clipper_variable_offset_inner_ex(storage_handle *storage,
    const expolygon_handle *expolygon, const float *deltas, uint32_t delta_count);

/* Native expand(Polygons, delta), with miter joins and limit 3.0. Each CCW ring
grows; each CW ring shrinks, then all results are united into Polygons (not
ExPolygons). Delta must be finite and > 0. Empty input gives empty output.
Returns a storage-owned collection or NULL for invalid arguments or internal
errors, without storing a result or letting an exception cross the C boundary. */
SLIC3R_HOST_API polygon_collection_handle *clipper_expand(storage_handle *storage,
    const polygon_collection_handle *polygons, double delta);

/* ---- Conversion back to ABI collections ------------------------------
Materialize a Clipper shape into a new storage-owned collection.

clipper_shapes_to_polygons() returns a polygon_collection_handle where each
item is intended to be interpreted as a Polygon.

clipper_shapes_to_expolygons() returns a real expolygon_collection_handle. The islands of the result of a Clipper operation
are listed from outside to inside (an island lying in a hole of another comes after it), as the host's layer islands require;
an ExPolygon collection converted without an operation keeps its source order.
Both returned handles are owned by storage and may be released with
storage_free().
*/
SLIC3R_HOST_API polygon_collection_handle *clipper_shapes_to_polygons(storage_handle *storage, const clipper_shapes_handle *shapes);
SLIC3R_HOST_API expolygon_collection_handle *clipper_shapes_to_expolygons(storage_handle *storage, const clipper_shapes_handle *shapes);

/*
Replace an existing collection with the materialized contents of shapes.

These functions are useful when the caller already owns a suitable output
collection and wants to avoid allocating another storage object. The destination
collection is cleared first, then filled with the converted result. The ExPolygons are in the same order as
clipper_shapes_to_expolygons() gives them.

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

/*
Detour an open polyline around closed rings.

Every path of rings is read as a closed ring, in the given order, without union or
offset: outer contours and holes are handled the same way. The polyline crosses a
ring where it goes from one side of that ring to the other. Touching a ring and
coming back on the same side is not a crossing, and neither is a contact at the
first or last point of the polyline. When the polyline runs along a ring edge and
leaves on the other side, the crossing is where it leaves the edge.

The crossings are visited in polyline order. When the ring of a crossing is crossed
again later, the stretch of polyline between them is replaced by an arc of that
ring: the ring is followed in the requested direction up to the first of those later
crossings it meets. The visit continues after that exit and ignores the crossings
of the replaced stretch. A crossing whose ring is not crossed again changes nothing.

direction chooses which way to follow the ring:
- CLIPPER_DETOUR_CW, CLIPPER_DETOUR_CCW: clockwise or counter-clockwise in XY;
- CLIPPER_DETOUR_POLYGON, CLIPPER_DETOUR_REVERSE: along the order of the ring's
  points, or against it;
- CLIPPER_DETOUR_SHORTEST, CLIPPER_DETOUR_LONGEST: the shorter or longer way to the
  next crossing of the same ring along the polyline (equal lengths follow the
  ring's point order). This chooses a direction only; it does not search for the
  shortest complete path.

The first and last points, and every polyline point outside a replaced stretch, are
kept exactly. Without any replacement the result is a point-for-point copy, also
for an empty or one-point polyline. Crossing points are rounded to integers and can
be one unit away from the exactly rounded intersection.

Expected input: the polyline has no two equal consecutive points and never passes
twice through the same point; for other polylines the result is unspecified. Rings
may touch each other. Repeated consecutive ring points are ignored and a ring with
fewer than three points is skipped. A ring touching itself is still followed, maybe
the long way round. Two crossings closer than the floating-point precision of
positions along the polyline can merge or vanish.

polyline and rings are only read, and must stay alive during this synchronous call,
which cannot be cancelled. The result is a new polyline owned by storage; release it
with storage_free(). An empty result for a non-empty polyline is reserved for a
geometric impossibility; no current input produces it. Returns NULL, without storing
anything, for a NULL argument, an unknown direction or an internal failure; no
exception crosses this C boundary.
*/
typedef enum clipper_detour_direction_t {
    CLIPPER_DETOUR_CW = 0,
    CLIPPER_DETOUR_CCW = 1,
    CLIPPER_DETOUR_POLYGON = 2,
    CLIPPER_DETOUR_REVERSE = 3,
    CLIPPER_DETOUR_SHORTEST = 4,
    CLIPPER_DETOUR_LONGEST = 5
} clipper_detour_direction_t;

SLIC3R_HOST_API polyline_handle *clipper_detour_polyline_expolygons(storage_handle *storage,
    const polyline_handle *polyline, const clipper_shapes_handle *rings,
    clipper_detour_direction_t direction);

#ifdef __cplusplus
}
#endif

#endif
