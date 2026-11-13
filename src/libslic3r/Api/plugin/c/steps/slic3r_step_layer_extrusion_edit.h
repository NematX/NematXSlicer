///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_step_layer_extrusion_edit_h_
#define slic3r_step_layer_extrusion_edit_h_

#define SLIC3R_PLUGIN_API_STEP_LAYER_EXTRUSION_EDIT_MAJOR 1u
#define SLIC3R_PLUGIN_API_STEP_LAYER_EXTRUSION_EDIT_MINOR 2u

#include "slic3r_step_common.h"
#include "../slic3r_printing_plan.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Version 1.2 consumes the shared retraction crossing policy on printing roots
   before planning ordinary retractions (RetractionCrossingPolicy.hpp).
   Fan policies run after geometry/speed edits. Each plugin completes every
   layer before the next priority observes its published references. */
enum layer_extrusion_edit_fan_priority {
    LAYER_EXTRUSION_EDIT_PRIORITY_FAN_REGIONAL = 18,
    LAYER_EXTRUSION_EDIT_PRIORITY_FAN_STANDARD = 20,
    LAYER_EXTRUSION_EDIT_PRIORITY_FAN_COOLING = 22,
    LAYER_EXTRUSION_EDIT_PRIORITY_FAN_PROGRAM = 23,
    LAYER_EXTRUSION_EDIT_PRIORITY_FAN_FIRST_LAYERS = 24,
    LAYER_EXTRUSION_EDIT_PRIORITY_FAN_DYNAMIC = 26,
    LAYER_EXTRUSION_EDIT_PRIORITY_FAN_TEMPORAL = 40
};

/*
Payload for STEP_LAYER_EXTRUSION_EDIT.

The host creates one payload per PrintingLayerGroup after STEP_ORDERING. The
plan and group are read-only structural context: plugins must not append,
remove, or reorder their vectors while layer runs may execute in parallel.
During serialized setup(), plugins may resolve tool-group extruder IDs without
changing the hierarchy. Parallel runs may edit the cloned extrusion roots and
scope events owned by their layer_group; other layers must not be modified.
*/
typedef struct run_ctx_layer_extrusion_edition {
    const print_handle *print;
    printing_plan_handle *plan;
    printing_group_handle *group;
    printing_layer_group_handle *layer_group;
    uint32_t group_idx;
    uint32_t layer_group_idx;
} run_ctx_layer_extrusion_edition;

static inline const run_ctx_layer_extrusion_edition *
plugin_ctx_as_layer_extrusion_edition(const plugin_run_context *ctx)
{
    if (!ctx || ctx->step != STEP_LAYER_EXTRUSION_EDIT)
        return NULL;
    return (const run_ctx_layer_extrusion_edition *)ctx->data;
}

#ifdef __cplusplus
}
#endif

#endif // slic3r_step_layer_extrusion_edit_h_
