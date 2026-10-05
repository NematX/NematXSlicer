///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_step_support_toolpath_h_
#define slic3r_step_support_toolpath_h_

#define SLIC3R_PLUGIN_API_STEP_SUPPORT_TOOLPATH_MAJOR 1u
#define SLIC3R_PLUGIN_API_STEP_SUPPORT_TOOLPATH_MINOR 0u

#include "slic3r_step_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
Payload for STEP_SUPPORT_TOOLPATH, run once per object after STEP_SUPPORT and
before STEP_SKIRT_BRIM. With no active plugin the step does nothing.

Support families publish auxiliary layers marked with the support layer
property. Their region-island surfaces carry RAW_SURFACE_TYPE_MOD_SUPPORT plus
position and density bits: bottom interface, sparse base, or top interface.

Toolpath algorithms declare one role through their exclusive group:
"support_toolpath_sparse", "support_toolpath_bottom_interface", or
"support_toolpath_top_interface". When several plugins share a role the host
generates a selector setting, and only the selected plugin of that role runs.
Selected plugins run sequentially by priority (recommended: sparse 0, bottom
interface 100, top interface 200). A later role can read earlier extrusions.

Each algorithm receives the whole object and walks its auxiliary layers,
islands and region islands to find support surfaces of its role. It may group
surfaces, process work in parallel, and call INFILL_PATTERN plugins internally.
It writes extrusions into the support-layer region islands through data-tree
mutators. Auxiliary layers remain editable through STEP_FINALIZE_SLICING.
Use the print and object handles only during the callback; never free them.
*/
typedef struct run_ctx_support_toolpath {
    const print_handle *print;
    const object_handle *object;
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
