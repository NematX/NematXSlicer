///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
/*
STEP_WIPETOWER: whole-plan tower manager
======================================

One selected exclusive manager receives the entire Print and mutable PrintingPlan
after ordering, before downstream extrusion editing. The Print is read-only;
plan mutations and ownership transfers use the plan editing API. The host lends
a snapshot of active WIPE_TOWER_WIPE services in priority order, with stable
zero-based indices for this manager run only. No service snapshot or payload
pointer may be retained beyond that run. Calls are synchronous and sequential;
the manager must not dispatch services concurrently or recursively.

For each call, the manager supplies a phase and its phase-specific fields as
documented in slic3r_step_wipe_tower_wipe.h. The host overwrites print/plan and
mutable_plan (non-NULL only for EDIT), supplies per-service error and diagnostic
contexts, calls setup once with data == NULL, and calls setup_run/run with the
borrowed phase payload. Geometry ownership stays with the supplied output roots.
The default manager adds forced contour visits before all service EDIT calls,
runs all COLLECT calls, then GENERATE in plan order and maintenance before insertion.
It removes unused manager-created visits only after insertion, so all request
anchors continue to address the final post-EDIT plan throughout generation.
It applies rotation followed by platter
translation to scaled local geometry as documented in the service header.

Dispatch returns 1 on success, or 0 for invalid arguments/indices, a prior service
failure, cancellation, plugin error, or exception. No exception crosses this C
callback. The host retains service failures for the enclosing step to report;
the manager must stop when dispatch returns 0. Managers themselves receive
data == NULL in setup and the whole-plan payload in setup_run/run.
*/
#ifndef slic3r_step_wipetower_h_
#define slic3r_step_wipetower_h_

#define SLIC3R_PLUGIN_API_STEP_WIPETOWER_MAJOR 2u
#define SLIC3R_PLUGIN_API_STEP_WIPETOWER_MINOR 0u

#include "slic3r_step_common.h"
#include "slic3r_step_wipe_tower_wipe.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Declare the payload tag here so C callback prototypes do not introduce distinct prototype-scope types. */
struct run_ctx_generate_wipe_tower;

/* Execute one snapshot service synchronously using borrowed ctx/payload; fill host-owned fields and return 1 on success, 0 on failure without transferring ownership. */
typedef int32_t (*wipe_tower_call_plugin_fn)(
    const struct run_ctx_generate_wipe_tower *ctx,
    uint32_t plugin_idx,
    run_ctx_wipe_tower_wipe *payload);

typedef struct run_ctx_generate_wipe_tower {
    /* Borrowed read-only Print and mutable host-owned whole plan for this run. */
    const print_handle *print;
    printing_plan_handle *plan;
    /* Opaque host dispatch state: pass this context unchanged, never dereference or retain it. */
    void *host_context;
    /* Number of snapshot services; valid indices are [0, wipe_plugin_count). */
    uint32_t wipe_plugin_count;
    /* Borrowed synchronous dispatcher; NULL is allowed when no services are available. */
    wipe_tower_call_plugin_fn call_wipe_plugin;
} run_ctx_generate_wipe_tower;

/* Return the borrowed whole-plan payload only for STEP_WIPETOWER. */
static inline const run_ctx_generate_wipe_tower *
plugin_ctx_as_generate_wipe_tower(const plugin_run_context *ctx)
{
    if (!ctx || ctx->step != STEP_WIPETOWER)
        return NULL;
    return (const run_ctx_generate_wipe_tower *)ctx->data;
}

#ifdef __cplusplus
}
#endif

#endif // slic3r_step_wipetower_h_
