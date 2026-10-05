///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_step_support_h_
#define slic3r_step_support_h_

#define SLIC3R_PLUGIN_API_STEP_SUPPORT_MAJOR 1u
#define SLIC3R_PLUGIN_API_STEP_SUPPORT_MINOR 1u

#include "slic3r_step_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
Payload for STEP_SUPPORT, called once per object after STEP_SUPPORT_DEMAND and
the infill steps, and before STEP_SUPPORT_TOOLPATH. All active plugins run in
priority order; there is no host selector for this step.

A support family handles only objects with support enabled and a
support_material_style that it implements. It leaves all other objects intact.
It reads island demand through layer_island_get_support_demand(), builds
auxiliary layers marked with LayerSupportProperty, and publishes fill surfaces:
- support | top | solid: upper contacts and interfaces;
- support | bottom | solid: lower contacts and interfaces, with bridge added
  when a lower contact uses bridge flow;
- support | internal | sparse: the support base.

The family writes no extrusions: STEP_SUPPORT_TOOLPATH generates them from
these surfaces. Use the print and object handles only during the callback;
never free them.
*/
typedef struct run_ctx_generate_support {
    const print_handle *print;
    const object_handle *object;
} run_ctx_generate_support;

static inline const run_ctx_generate_support *
plugin_ctx_as_generate_support(const plugin_run_context *ctx)
{
    if (!ctx || ctx->step != STEP_SUPPORT)
        return NULL;
    return (const run_ctx_generate_support *)ctx->data;
}

#ifdef __cplusplus
}
#endif

#endif // slic3r_step_support_h_
