///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_step_support_demand_h_
#define slic3r_step_support_demand_h_

#define SLIC3R_PLUGIN_API_STEP_SUPPORT_DEMAND_MAJOR 2u
#define SLIC3R_PLUGIN_API_STEP_SUPPORT_DEMAND_MINOR 0u

#include "slic3r_step_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
Payload for STEP_SUPPORT_DEMAND.

This step builds, for every layer island of the object, the areas that ask for
support. It runs right after STEP_POST_SLICING, before the perimeters, so every
later step can read the demand with layer_island_get_support_demand(). Support
generation consumes it; plugins of the steps between may refine it with
support_demand_get_island_mutable() (bridge removal in STEP_POST_PERIMETER).

The demand is stored in each LayerSliceIsland. It is empty when the step
starts; plugins run in priority order and each one reads and edits what the
previous ones left.

Normal plugin usage:

1. Cast the generic plugin_run_context with plugin_ctx_as_support_demand().
   If it returns NULL, the current run is not STEP_SUPPORT_DEMAND.

2. Walk the object geometry from ctx->object: its layers and layer islands are
   borrowed, read-only host handles, stable only for the current
   setup_run()/run() call.

3. Read an island's demand with layer_island_get_support_demand(), or get it
   in mutable form with support_demand_get_island_mutable() and edit the
   returned collection in place: add, replace, clip or clear its ExPolygons.
   Demand areas should stay inside the island slice.

The print/object data tree is read-only context for this step; the demand of
each island is the only mutable data.
*/

/*
Return the mutable support demand of one layer island.

run_ctx is the plugin_run_context of the calling plugin: during a pipeline
run, the host allows the edit while the running step is between
STEP_SUPPORT_DEMAND and STEP_SUPPORT inclusive, in execution order, so any
plugin of those steps may refine the demand (for example from the perimeters
or the bridges actually extruded). In another step it reports an error through
run_ctx and returns NULL: before the window the demand is not built yet, after
it support generation has consumed it. Outside a pipeline run (no running
step) the edit is allowed, like other data-tree edits. NULL arguments return
NULL.

Steps run objects in parallel: edit only islands of the object you process.
The collection is owned by the island: edit it in place and never release it
with storage_free(). It stays valid until the island is destroyed.
*/
SLIC3R_HOST_API expolygon_collection_handle *support_demand_get_island_mutable(const plugin_run_context *run_ctx,
                                                                               const layer_island_handle *island);

typedef struct run_ctx_support_demand {
    /*
    Borrowed print context. It gives access to global print config and objects.
    The handle is read-only for this step and valid only during the callback.
    */
    const print_handle *print;

    /*
    Borrowed object currently being processed. Plugins usually iterate its
    layers and layer islands to decide where support is needed.
    */
    const object_handle *object;
} run_ctx_support_demand;

/*
Return the STEP_SUPPORT_DEMAND payload from a generic plugin_run_context.

Returns NULL when ctx is NULL, when ctx belongs to another step, or when the
host did not provide this payload. Plugins should call this once at the
beginning of setup_run()/run() and use the returned pointer for the rest of the
callback.
*/
static inline const run_ctx_support_demand *
plugin_ctx_as_support_demand(const plugin_run_context *ctx)
{
    if (!ctx || ctx->step != STEP_SUPPORT_DEMAND)
        return NULL;
    return (const run_ctx_support_demand *)ctx->data;
}

#ifdef __cplusplus
}
#endif

#endif // slic3r_step_support_demand_h_
