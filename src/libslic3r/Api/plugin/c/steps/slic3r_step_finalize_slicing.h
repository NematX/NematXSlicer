///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_step_finalize_slicing_h_
#define slic3r_step_finalize_slicing_h_

#define SLIC3R_PLUGIN_API_STEP_FINALIZE_SLICING_MAJOR 2u
#define SLIC3R_PLUGIN_API_STEP_FINALIZE_SLICING_MINOR 0u

#include "slic3r_step_common.h"
#include "../slic3r_extrusion_entity.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
Allow plugins to adjust finished extrusion paths before ordering and G-code export.

This is the payload for STEP_FINALIZE_SLICING, the last step that may edit the
Print data tree. It runs after support toolpaths and
skirt/brim generation, before STEP_ORDERING builds the PrintingPlan. Later steps
read this tree and write their results into the PrintingPlan.

Use plugin_ctx_as_finalize_slicing() during the current plugin run. Read support
completion before interpreting an empty request or path set. Build private infill
roots in that run's plugin storage, then publish them with replace_infill_buckets().
The payload and all host handles are borrowed only for the callback; retain none.
Runs for different objects may be concurrent. A run may publish only to its own
object and must not change storage or destinations concurrently with publication.
*/
/* One exchange of printable content; both root identities remain unchanged.
   Existing views of either root's descendants must be discarded after success. */
typedef struct raw_final_infill_replacement {
    /* Borrowed existing model-layer infill, ironing or gap-fill root of this object. */
    const extrusion_entity_handle *destination;
    /* Unique donor owned by the current run's storage; owns the old content after success. */
    extrusion_entity_handle *prepared;
} raw_final_infill_replacement;

struct run_ctx_finalize_slicing;
/* Borrow the final request mask on an island of this object, without allocation
   or mutation. Null returns null. The mask follows the island's lifetime; inspect
   it only during this run, and only interpret emptiness when demand is computed.
   No exception crosses C. */
typedef const expolygon_collection_handle *(*final_requested_support_areas_fn)(const layer_island_handle *island);
/* Validate every destination and storage-owned donor before exchanging any content.
   Pass this payload unchanged and its run's storage. Destinations and donors must
   each be unique; donors cannot be live object buckets. The array is borrowed
   synchronously and not retained. Return 1 on success, leaving donors owning old
   content, or 0 with both sides unchanged on rejection or cancellation. A caught
   failure also requests plugin stop; no exception crosses C. No allocation,
   cancellation check or plugin call occurs between the nonthrowing swaps. */
typedef int32_t (*final_replace_infill_fn)(const struct run_ctx_finalize_slicing *ctx,
    storage_handle *storage, const raw_final_infill_replacement *replacements, uint32_t count);

typedef struct run_ctx_finalize_slicing {
    const print_handle *print;
    const object_handle *object;
    /* Opaque host state; pass this payload unchanged to replace_infill_buckets(). */
    void *host_context;
    /* 1 when final local requests were computed, including an empty result; else 0. */
    uint32_t support_demand_computed;
    /* 1 when matching support producers completed, or no material was expected;
       else 0. This does not assert that any paths exist. Read auxiliary support
       and raft paths at their actual Z through the data tree. */
    uint32_t support_material_generated;
    final_requested_support_areas_fn get_requested_support_areas;
    final_replace_infill_fn replace_infill_buckets;
} run_ctx_finalize_slicing;

/* Borrow this run's payload, or return null for a null context or another step;
   no ownership transfer, allocation or mutation occurs. */
static inline const run_ctx_finalize_slicing *
plugin_ctx_as_finalize_slicing(const plugin_run_context *ctx)
{
    if (!ctx || ctx->step != STEP_FINALIZE_SLICING)
        return NULL;
    return (const run_ctx_finalize_slicing *)ctx->data;
}

#ifdef __cplusplus
}
#endif

#endif // slic3r_step_finalize_slicing_h_
