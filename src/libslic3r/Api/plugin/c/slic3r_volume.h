///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_volume_h_
#define slic3r_volume_h_

#define SLIC3R_PLUGIN_API_VOLUME_MAJOR 2u
#define SLIC3R_PLUGIN_API_VOLUME_MINOR 1u

#include <stdint.h>

#include "slic3r_data_tree.h"
#include "slic3r_geometry.h"
#include "slic3r_plugin_run_context.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================= HANDLES ========================= */

typedef struct volume_handle volume_handle;
typedef struct triangle_mesh_handle triangle_mesh_handle;
typedef struct slicing_layer_range_handle slicing_layer_range_handle;
typedef struct slicing_volume_region_handle slicing_volume_region_handle;

/* ========================= BASIC VALUE TYPES ========================= */

typedef struct c_triangle_indices {
    uint32_t a;
    uint32_t b;
    uint32_t c;
} c_triangle_indices;

/*
Low-level mesh slicing mode, matching MeshSlicingParams::SlicingMode.
Plugins should normally derive this from the Object's "slicing_mode" config:
Regular -> REGULAR, EvenOdd -> EVEN_ODD, CloseHoles -> POSITIVE.
*/
typedef enum raw_mesh_slicing_mode {
    RAW_MESH_SLICING_MODE_REGULAR                  = 0,
    RAW_MESH_SLICING_MODE_EVEN_ODD                 = 1,
    RAW_MESH_SLICING_MODE_POSITIVE                 = 2,
    RAW_MESH_SLICING_MODE_POSITIVE_LARGEST_CONTOUR = 3
} raw_mesh_slicing_mode;

/*
C ABI mirror of MeshSlicingParamsEx.

All distances are unscaled millimeters because TriangleMesh vertices and the
low-level slicer work in float/double model coordinates. The transform is also
unscaled and is applied before slicing.

triangle_mesh_slice_to_polygons() reads only mode, slicing_mode_normal_below_layer
(an index into its z_mm_by_layer array: layers below it use mode_below), mode_below
and transform. closing_radius, extra_offset, resolution and model_resolution drive the
plugin-side post-processing of the raw loops (slice_to_expolygons in the C++ and Python
helpers); the host still validates them (finite, closing_radius >= 0).
*/
typedef struct c_mesh_slicing_params {
    raw_mesh_slicing_mode mode;
    uint32_t slicing_mode_normal_below_layer;
    raw_mesh_slicing_mode mode_below;
    c_matrix4d transform;
    float closing_radius;
    float extra_offset;
    double resolution;
    double model_resolution;
} c_mesh_slicing_params;

/* ========================= VOLUME TYPE ========================= */

/*
Stable C mirror of the host volume type enum.
Keep the numeric values aligned with the host enum because plugins may persist
or compare them without including C++ headers.
*/
typedef enum raw_volume_type {
    RAW_VOLUME_TYPE_INVALID                     = -1,
    RAW_VOLUME_TYPE_MODEL_PART                  = 0,
    RAW_VOLUME_TYPE_NEGATIVE_VOLUME             = 1,
    RAW_VOLUME_TYPE_PARAMETER_MODIFIER          = 2,
    RAW_VOLUME_TYPE_SUPPORT_BLOCKER             = 3,
    RAW_VOLUME_TYPE_SUPPORT_ENFORCER            = 4,
    RAW_VOLUME_TYPE_SEAM_POSITION_CENTER        = 5,
    RAW_VOLUME_TYPE_SEAM_POSITION_CENTER_Z      = 6,
    /* Internal-perimeter attractor, active only through the volume's height.
       The old spelling is an alias; no existing numeric value changes. */
    RAW_VOLUME_TYPE_SEAM_POSITION_INTERNAL_CENTER_Z = 7,
    RAW_VOLUME_TYPE_SEAM_POSITION_INSIDE_CENTER = RAW_VOLUME_TYPE_SEAM_POSITION_INTERNAL_CENTER_Z,
    RAW_VOLUME_TYPE_SEAM_POSITION_INSIDE        = 8,
    RAW_VOLUME_TYPE_BRIM_PATCH                  = 9,
    RAW_VOLUME_TYPE_BRIM_NEGATIVE               = 10
} raw_volume_type;

/* ========================= FACET PAINTING ========================= */

/*
Painting stored on a model volume facet selection.

Every painting kind is identified by a stable UTF-8 key. Built-in paintings
use the three keys below; plugin FacetsAnnotation kinds use the key passed to
orchestrator_register_generic_facets_annotation().

The host decides internally whether a key maps to older hardcoded storage
(supports / MMU) or to generic plugin-style storage. Plugin code does not need
separate APIs for those cases.

FDM support painting, seam painting and plugin FacetsAnnotation kinds use
RAW_FACET_PAINTING_ENFORCER / RAW_FACET_PAINTING_BLOCKER as values.

MMU painting uses the value as a 1-based extruder selector:
1 means the first extruder, 2 means the second extruder, and so on. Passing
RAW_FACET_PAINTING_NONE returns no painted facets.
*/
#define RAW_FACET_PAINTING_FDM_SUPPORT      "builtin:fdm_support"
#define RAW_FACET_PAINTING_SEAM             "builtin:seam"
#define RAW_FACET_PAINTING_MMU_SEGMENTATION "builtin:mmu_segmentation"

typedef enum raw_facet_painting_value {
    RAW_FACET_PAINTING_NONE     = 0,
    RAW_FACET_PAINTING_ENFORCER = 1,
    RAW_FACET_PAINTING_BLOCKER  = 2
} raw_facet_painting_value;

/* ========================= PRINTOBJECT -> VOLUMES ========================= */

/* Returns the number of volumes borrowed from the Object's source model data. */
SLIC3R_HOST_API uint32_t object_volume_count(const object_handle *object);

/*
Returns a borrowed read-only Volume handle.
The handle stays valid only while the underlying Object stays valid.
*/
SLIC3R_HOST_API const volume_handle *object_volume_at(const object_handle *object, uint32_t idx);

/*
Read-only access to the Object's volume-to-region assignment.

These views expose the same table used by STEP_SLICING. They are useful for
helper geometry that already has a 2D subject and only wants to know which
model-part or modifier volumes provide alternate PrintRegion settings at a Z
height. Negative volumes are still visible in this table, but helpers that do
not build object material should ignore them unless they explicitly want
negative-volume behavior.

note: this part of the api may change in the future to support more flexible volume-to-region assignment
*/
SLIC3R_HOST_API uint32_t object_count_slicing_layer_range(const object_handle *object);
SLIC3R_HOST_API const slicing_layer_range_handle *object_get_slicing_layer_range(const object_handle *object,
                                                                                 uint32_t idx);
SLIC3R_HOST_API coord_t slicing_layer_range_get_z_min(const slicing_layer_range_handle *range);
SLIC3R_HOST_API coord_t slicing_layer_range_get_z_max(const slicing_layer_range_handle *range);
SLIC3R_HOST_API const config_handle *slicing_layer_range_get_config(const slicing_layer_range_handle *range);

SLIC3R_HOST_API uint32_t slicing_layer_range_count_volume_region(const slicing_layer_range_handle *range);
SLIC3R_HOST_API const slicing_volume_region_handle *slicing_layer_range_get_volume_region(
    const slicing_layer_range_handle *range,
    uint32_t idx);

SLIC3R_HOST_API const volume_handle *slicing_volume_region_get_volume(const slicing_volume_region_handle *volume_region);
SLIC3R_HOST_API int32_t slicing_volume_region_get_parent(const slicing_volume_region_handle *volume_region);
SLIC3R_HOST_API int32_t slicing_volume_region_get_layer_region_idx(const slicing_volume_region_handle *volume_region);
SLIC3R_HOST_API c_bounding_box3f slicing_volume_region_get_bbox(const slicing_volume_region_handle *volume_region);

/* ========================= VOLUME ========================= */

SLIC3R_HOST_API raw_volume_type volume_get_type(const volume_handle *volume);
SLIC3R_HOST_API uint64_t volume_get_id(const volume_handle *volume);
SLIC3R_HOST_API const config_handle *volume_get_config(const volume_handle *volume);

/* Returns -1 if the volume has no FFF extruder assignment. */
SLIC3R_HOST_API int32_t volume_get_extruder_id(const volume_handle *volume);

SLIC3R_HOST_API c_matrix4d volume_get_matrix(const volume_handle *volume);
SLIC3R_HOST_API c_matrix4d volume_get_matrix_no_offset(const volume_handle *volume);

/*
Returns non-zero if the volume has any painted facets for paint_key.

paint_key may be one of RAW_FACET_PAINTING_* or a plugin key registered with
orchestrator_register_generic_facets_annotation().
*/
SLIC3R_HOST_API int volume_has_painting(const volume_handle *volume, const char *paint_key);

/*
Project painted facets from all model-part volumes of an Object to its layers.

out_by_layer must point to layer_count mutable polygon_collection_handle
objects, usually created by the plugin in its storage. Every destination
collection is cleared first, then replaced by the projected polygons for the
matching layer.

For paint_key RAW_FACET_PAINTING_FDM_SUPPORT:
    painting_value is RAW_FACET_PAINTING_ENFORCER or RAW_FACET_PAINTING_BLOCKER.
    Downward facing painted facets are projected upward to the slicing planes,
    matching the native support painting behavior.

For paint_key RAW_FACET_PAINTING_SEAM or a plugin generic painting key:
    painting_value is RAW_FACET_PAINTING_ENFORCER or RAW_FACET_PAINTING_BLOCKER.
    Painted facets are projected through the touched layer slabs, matching the
    native seam painting behavior.

For paint_key RAW_FACET_PAINTING_MMU_SEGMENTATION:
    painting_value is a 1-based extruder index. The host projects both top and
    bottom painted facet slabs into the output. This is a raw geometric helper;
    it does not perform the full native MMU segmentation refinement.
*/
SLIC3R_HOST_API void object_project_painting_to_polygons(const object_handle *object,
                                                         const char *paint_key,
                                                         int32_t painting_value,
                                                         polygon_collection_handle **out_by_layer,
                                                         uint32_t layer_count);

SLIC3R_HOST_API const triangle_mesh_handle *volume_get_mesh(const volume_handle *volume);

/* ========================= TRIANGLE MESH ========================= */

/*
Create a triangle mesh owned by storage from plugin-built arrays; release it with
storage_free() or storage_clear(). Vertices are unscaled float model coordinates
and are copied without repair, so readback returns the same vertices and indices.
Each index must be below vertex_count, each triangle needs three distinct indices,
and each coordinate must be finite; null arrays are valid only with a zero count.
Distinct vertices at identical coordinates, or at coordinates that round to the same
slicer point, are accepted, but no triangle edge may join them: the slicer needs
edges of non-zero length. NULL reports a creation failure: invalid input or an
internal exception such as an allocation error.
*/
SLIC3R_HOST_API triangle_mesh_handle *storage_new_triangle_mesh(storage_handle *storage,
                                                                const c_vec3f *vertices,
                                                                uint32_t vertex_count,
                                                                const c_triangle_indices *triangles,
                                                                uint32_t triangle_count);

SLIC3R_HOST_API uint32_t triangle_mesh_vertex_count(const triangle_mesh_handle *mesh);
SLIC3R_HOST_API uint32_t triangle_mesh_triangle_count(const triangle_mesh_handle *mesh);
SLIC3R_HOST_API c_vec3f triangle_mesh_vertex_at(const triangle_mesh_handle *mesh, uint32_t idx);
SLIC3R_HOST_API c_triangle_indices triangle_mesh_triangle_at(const triangle_mesh_handle *mesh, uint32_t idx);

/* Project every transformed mesh triangle onto XY and store the raw polygons
    in storage. The returned collection belongs to storage and may be empty.
    Native and API calls share exact-degeneracy filtering on a working copy and
    ring normalization. The borrowed mesh and its face indices are not changed.
    NULL reports invalid arguments or an internal projection failure. */
SLIC3R_HOST_API polygon_collection_handle *triangle_mesh_project_to_polygons(
    storage_handle *storage, const triangle_mesh_handle *mesh, c_matrix4d transform);

/*
Result of triangle_mesh_slice_to_polygons(). The function returns an int32_t
holding one of these values.
*/
typedef enum raw_mesh_slice_status {
    RAW_MESH_SLICE_STATUS_INTERNAL_ERROR   = -2,
    RAW_MESH_SLICE_STATUS_INVALID_ARGUMENT = -1,
    RAW_MESH_SLICE_STATUS_CANCELLED        = 0,
    RAW_MESH_SLICE_STATUS_DONE             = 1
} raw_mesh_slice_status;

/*
Slice a TriangleMesh into raw polygon loops at object-local Z positions, with
the low-level slicer of the host (slice_mesh): no union, offset or simplification
is applied to the loops. Exact duplicate vertices in each ring are normalized
by the same native helper used for projections. Degenerate source faces are
filtered only in a working copy, preserving borrowed mesh data and paintings.
The triangles are flipped when the determinant of the
transform is negative, so a mirroring transform keeps outward-facing triangles.
RAW_MESH_SLICING_MODE_POSITIVE_LARGEST_CONTOUR keeps its native raw meaning
here (the largest raw loop of each layer by absolute area, made counter-clockwise; a
layer whose loops all have a zero area is empty).

params must be fully set: a zeroed c_mesh_slicing_params has a null transform
matrix, which crushes the mesh. Mode 0 is RAW_MESH_SLICING_MODE_REGULAR.
z_mm_by_layer holds layer_count unscaled float Z values, which must be finite
and non-decreasing. If a Z comes from a Layer view, convert Layer::slice_z() with
unscaled() first.

slices_by_layer holds layer_count mutable polygon_collection_handle pointers,
usually created in plugin storage by the plugin. Every non-NULL collection is
cleared and replaced by the polygons generated at the matching Z; a NULL entry is
ignored. Outputs are published only when the result is
RAW_MESH_SLICE_STATUS_DONE.

context may be NULL, and so may its is_cancelled callback: both mean "never
cancelled". The host calls is_cancelled() during the work, from several threads
at the same time: it must support concurrent calls and must never throw. The
real granularity of the checks is coarse. The host checks:
  - before starting;
  - at face 0 and at every 65,536th face while building the segments;
  - once between building and joining the segments;
  - at layer 0 and at every 65,536th layer while joining the segments into loops;
  - once before publishing the result.
A small mesh with few layers is therefore checked only a handful of times, and a
cancellation that arrives after the last check is still detected before the result
is published. Cancellation is also reported (also before starting, and also when
layer_count is 0) when the call runs inside a parallel group that was cancelled,
for example one cancelled by a failing sibling job, even when context is NULL.

Return value:
  RAW_MESH_SLICE_STATUS_DONE (1): finished, including layer_count == 0, for which
      z_mm_by_layer and slices_by_layer may be NULL (the cancellation checks still
      apply first).
  RAW_MESH_SLICE_STATUS_CANCELLED (0): stopped by the context or by the enclosing
      parallel group.
  RAW_MESH_SLICE_STATUS_INVALID_ARGUMENT (-1): mesh or params is NULL; z_mm_by_layer
      or slices_by_layer is NULL with layer_count > 0; a Z is not finite or is lower
      than the previous one; closing_radius is negative or not finite; extra_offset,
      resolution, model_resolution or a transform coefficient is not finite.
  RAW_MESH_SLICE_STATUS_INTERNAL_ERROR (-2): an exception such as an allocation
      error was caught at the boundary.
On 0, -1 and -2 every non-NULL output collection is cleared (not freed).
*/
SLIC3R_HOST_API int32_t triangle_mesh_slice_to_polygons(const triangle_mesh_handle *mesh,
                                                        const c_mesh_slicing_params *params,
                                                        const float *z_mm_by_layer,
                                                        polygon_collection_handle **slices_by_layer,
                                                        uint32_t layer_count,
                                                        const plugin_run_context *context);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* slic3r_volume_h_ */
