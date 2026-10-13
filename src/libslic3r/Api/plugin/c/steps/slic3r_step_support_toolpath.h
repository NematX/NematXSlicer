///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_step_support_toolpath_h_
#define slic3r_step_support_toolpath_h_

#define SLIC3R_PLUGIN_API_STEP_SUPPORT_TOOLPATH_MAJOR 1u
#define SLIC3R_PLUGIN_API_STEP_SUPPORT_TOOLPATH_MINOR 2u

#include "slic3r_step_common.h"
#include "slic3r_step_infill.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Declare the payload tag here so C callback prototypes do not introduce distinct prototype-scope types. */
struct run_ctx_support_toolpath;

/*
Resolve a stable INFILL_PATTERN plugin id, for example "rectilinear" or
"supportbase", to the runtime id used by generate_pattern() during this step
run. The id fallback works as in run_ctx_generate_infill::resolve_pattern_id():
an inactive id falls back to the first active pattern with a warning.
INFILL_PATTERN_RUNTIME_ID_INVALID is returned when:
- no pattern is active, or ctx is NULL or has no host state (ordinary
  refusals; the step goes on);
- the step is already stopping, because the plugin cancellation is requested
  or the print is cancelled: it is returned at once, without a message;
- an exception was contained: an error message is added and the plugin
  cancellation is requested, so the step stops when the algorithm returns. A
  cancellation of the print adds no message and no plugin cancellation: the
  host rethrows it when the algorithm returns.
No exception ever leaves the callback.
Runtime ids are valid only for the current step run; never store them.

ctx is the run_ctx_support_toolpath received by the plugin.
*/
typedef infill_pattern_runtime_id (*support_toolpath_resolve_pattern_id_fn)(
    const struct run_ctx_support_toolpath *ctx,
    const char *pattern_plugin_id);

/*
Fill one surface with the INFILL_PATTERN plugin selected by pattern_id. The
arguments are those of run_ctx_generate_infill::generate_pattern(), with the
support toolpath payload as ctx.

The pattern receives ctx->print and ctx->object, the given layer, island,
region_island, primary_region (may be NULL on support layers), surface,
no_overlap_areas (may be NULL) and params, and writes its paths into output.
output must be an empty mutable extrusion entity owned by the caller for the
call; the caller then publishes its content itself, for example by moving it
into a region-island extrusion root.

Returns 1 when the pattern ran, even if it produced nothing. Returns 0 in these
cases, which differ in what happens to output and to the step:
- ordinary refusal: ctx, its host state, params or output is NULL (nothing is
  touched), an
  unknown runtime id (warning message), or a pattern that needs the STEP_INFILL
  initialization pass (lightning, adaptivecubic, supportcubic; error message).
  The step goes on; output is untouched;
- already stopping: the plugin cancellation is requested or the print is
  cancelled. It returns at once with output left as an empty collection and no
  message;
- contained exception: the pattern reported an error or any other C++
  exception was raised. output is left as an empty collection, an error message
  is added and the plugin cancellation is requested, so the step stops when the
  algorithm returns. A cancellation of the print is not recorded: it gives the
  same 0 and the host rethrows it when the algorithm returns.
No recipe modifier is applied. No exception ever leaves the callback.
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
interfaces themselves when needed. A tree branch body carries
RAW_SURFACE_TYPE_MOD_BRANCH on top of support | internal | sparse.

Toolpath algorithms declare one role through their exclusive group:
"support_toolpath_sparse", "support_toolpath_branch",
"support_toolpath_bottom_interface", or "support_toolpath_top_interface". When
several plugins share a role the host generates a selector setting for each
object, and only the plugin selected for that object runs on it. Selected
plugins run sequentially by priority (recommended: sparse 0, branch 50, bottom
interface 100, top interface 200), each on the objects that selected it. A later
role can read earlier extrusions. setup() of an algorithm receives the number of
objects it runs on, so its progress counts only those runs.

The branch role prints the surfaces marked RAW_SURFACE_TYPE_MOD_BRANCH and no
other role does: sparse, bottom and top algorithms skip branch surfaces.

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
