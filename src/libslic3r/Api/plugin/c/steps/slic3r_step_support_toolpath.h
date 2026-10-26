///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_step_support_toolpath_h_
#define slic3r_step_support_toolpath_h_

#define SLIC3R_PLUGIN_API_STEP_SUPPORT_TOOLPATH_MAJOR 1u
#define SLIC3R_PLUGIN_API_STEP_SUPPORT_TOOLPATH_MINOR 1u

#include "slic3r_step_common.h"
#include "slic3r_step_infill.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
Resolve a stable INFILL_PATTERN plugin id, for example "rectilinear" or
"supportbase", to the runtime id used by generate_pattern() during this step
run. Same semantics as run_ctx_generate_infill::resolve_pattern_id(): an
inactive id falls back to the first active pattern with a warning, and
INFILL_PATTERN_RUNTIME_ID_INVALID is returned only when no pattern is active.
Runtime ids are valid only for the current step run; never store them.

ctx is the run_ctx_support_toolpath received by the plugin.
*/
typedef infill_pattern_runtime_id (*support_toolpath_resolve_pattern_id_fn)(
    const struct run_ctx_support_toolpath *ctx,
    const char *pattern_plugin_id);

/*
Fill one surface with the INFILL_PATTERN plugin selected by pattern_id. Same
arguments and semantics as run_ctx_generate_infill::generate_pattern(), with
the support toolpath payload as ctx.

The pattern receives ctx->print and ctx->object, the given layer, island,
region_island, primary_region (may be NULL on support layers), surface,
no_overlap_areas (may be NULL) and params, and writes its paths into output.
output must be an empty mutable extrusion entity owned by the caller for the
call; the caller then publishes its content itself, for example by moving it
into a region-island extrusion root.

Returns 1 when the pattern ran, even if it produced nothing, and 0 when nothing
was generated because of an error: an unknown runtime id (warning message), or
a pattern that needs the STEP_INFILL initialization pass (lightning,
adaptivecubic, supportcubic; error message). No recipe modifier is applied.
An error reported by the pattern itself, or a cancellation, stops the step as
for any plugin error.
*/
typedef int32_t (*support_toolpath_generate_pattern_fn)(
    const struct run_ctx_support_toolpath *ctx,
    infill_pattern_runtime_id pattern_id,
    const layer_handle *layer,
    const layer_island_handle *island,
    const layer_region_island_handle *region_island,
    const layer_region_handle *primary_region,
    const surface_handle *surface,
    const expolygon_collection_handle *no_overlap_areas,
    const raw_infill_pattern_params *params,
    extrusion_entity_handle *output);

/*
Payload for STEP_SUPPORT_TOOLPATH, run once per object after STEP_SUPPORT and
before STEP_SKIRT_BRIM. With no active plugin the step does nothing.

Support families publish auxiliary layers marked with the support layer
property. Their region-island surfaces carry RAW_SURFACE_TYPE_MOD_SUPPORT plus
position and density bits: support | bottom | solid for lower contacts and
interfaces, support | internal | sparse for the base, and support | top | solid
for upper contacts and interfaces. A lower contact using bridge flow also
carries the bridge modifier. Algorithms distinguish contacts from intermediate
interfaces themselves when needed.

Toolpath algorithms declare one role through their exclusive group:
"support_toolpath_sparse", "support_toolpath_bottom_interface", or
"support_toolpath_top_interface". When several plugins share a role the host
generates a selector setting, and only the selected plugin of that role runs.
Selected plugins run sequentially by priority (recommended: sparse 0, bottom
interface 100, top interface 200). A later role can read earlier extrusions.

Each algorithm receives the whole object and walks its auxiliary layers,
islands and region islands to find support surfaces of its role. It may group
surfaces and process work in parallel. Each algorithm prints only the surfaces
of its own role: no role takes another role's area, so a role without interface
layers prints its own contact as base instead of handing it over.
It writes extrusions into the support-layer region islands through data-tree
mutators. Auxiliary layers remain editable through STEP_FINALIZE_SLICING.
Use the print and object handles only during the callback; never free them.

Infill patterns: an algorithm may fill a surface with any active INFILL_PATTERN
plugin through resolve_pattern_id() and generate_pattern(), passing this
payload back as ctx. The host creates every pattern context before the objects
run, so both callbacks may be called concurrently, from several objects and
from the algorithm's own worker threads. Unlike STEP_INFILL there is no recipe
modifier and no pattern initialization pass.

The caller builds raw_infill_pattern_params itself; the host fills nothing.
Start from a zeroed struct and set at least:
- pattern_id: the value returned by resolve_pattern_id();
- surface_type and extrusion_role (for example RAW_EXTRUSION_ROLE_SUPPORT or
  the interface role), which are copied onto every generated path;
- flow: width, spacing, height, nozzle_diameter, mm3_per_mm, spacing_ratio and
  is_bridge of the support flow at this layer height;
- spacing (mm, usually the flow spacing) and density in (0, 1]: a zero
  density is invalid and a zero spacing yields no usable line spacing;
- flow_mult = 1: zero gives paths with no width and no volume;
- angle (radians), layer_id and z: patterns alternate their direction from
  layer_id when bridge_angle is negative;
- bridge_angle = -1: zero is a valid bridge angle and forces every layer to 0;
- bridge_offset = -1: zero shifts bridge lines when flow.is_bridge is set;
- anchor_length and anchor_length_max (mm, anchor_length <= anchor_length_max),
  1000 for both in the legacy support: a zero maximum switches rectilinear to
  its simple connector, and supportbase requires a maximum of at least 0.01;
- fill_resolution (scaled), 0.0125 mm in the legacy support;
- object_bounding_box: the object's scaled bounding box, used to align sparse
  lines between layers; an empty box aligns them on the origin;
- connection (C_INFILL_CONNECTION_CONNECTED is 0) and infill_flags as needed;
  the legacy support sets C_INFILL_FLAG_DONT_ADJUST.
link_max_length, loop_clipping, overlap, max_sparse_infill_spacing, extruder
and priority may stay 0.
*/
typedef struct run_ctx_support_toolpath {
    const print_handle *print;
    const object_handle *object;

    /* Fields below were added in minor version 1. */

    /*
    Opaque host state behind the callbacks. Pass the original
    run_ctx_support_toolpath pointer back to the callbacks; never inspect or
    store host_context.
    */
    void *host_context;
    support_toolpath_resolve_pattern_id_fn resolve_pattern_id;
    support_toolpath_generate_pattern_fn generate_pattern;
} run_ctx_support_toolpath;

/* Support toolpath payload, or NULL for a null context, wrong step, or null data. */
static inline const run_ctx_support_toolpath *
plugin_ctx_as_support_toolpath(const plugin_run_context *ctx)
{
    if (!ctx || ctx->step != STEP_SUPPORT_TOOLPATH)
        return NULL;
    return (const run_ctx_support_toolpath *)ctx->data;
}

#ifdef __cplusplus
}
#endif

#endif // slic3r_step_support_toolpath_h_
