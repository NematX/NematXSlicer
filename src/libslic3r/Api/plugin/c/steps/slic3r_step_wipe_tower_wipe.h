///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
WIPE_TOWER_WIPE: non-exclusive geometry services driven by a tower manager
=======================================================================

The default manager adds forced contour-tool visits before calling every selected
service in priority order for EDIT, then every service in that order for COLLECT.
Forced contour visits are limited to owner layers through the last layer with a
tool change in the unmodified plan: a concrete visit's extruder differs from the
previous visit, including across layer/group boundaries. The initial selection
is not a change. No forced visit is added for an owner with no such layer.
Above that boundary, an absent configured contour tool is not added; maintenance
uses the first tower visit's tool, as when forced contour changes are disabled.
Only towers with at least one printed layer receive a synthetic catalog entry.
After reserving all bands, it calls the reporting service once per request for
GENERATE, in plan-anchor order; equal anchors retain service collection order.
Calls, including setup/setup_run/run, are sequential; a service need not support
concurrent calls within this manager run. setup receives data == NULL once;
setup_run and run receive this phase payload. No service is called when the
configured towers are ineligible, or there are no active services.

Tower choice uses the first PrintingExtrusion of an incoming part's tool visit,
regardless of its insertion index. An empty incoming visit uses the next extrusion
in tool-visit order inside the same layer group. For an outgoing-only request,
the extrusion at its anchor index is following; an end anchor searches later
tool visits in that layer. If no following extrusion exists, use the nearest
preceding extrusion in that same layer. No search crosses a layer-group boundary.
An instance tower takes precedence over the global tower; when neither exists,
the request is silently dropped. A source-less extrusion has no instance tower.

Phase payload availability (print and const plan are valid in every phase):
  EDIT:     mutable_plan is valid; report_context, report_request, request,
            band, and exit are NULL. Only this phase may adjust the plan.
  COLLECT:  mutable_plan is NULL; report_context and report_request are valid;
            request, band, and exit are NULL. Report value-only requests without
            changing the plan. Anchors refer to the plan after ALL EDIT calls.
  GENERATE: mutable_plan, report_context, and report_request are NULL; request,
            band, and exit are valid. Modify only band output roots and *exit;
            neither the plan nor the request/band descriptors may be changed.

Frames and units:
  Band min/max/entry and returned exit are XY millimeters in the tower's local
  frame. Local X lies in [0, W], and increasing Y points toward the open side.
  print_z, layer_height, and extrusion_width are millimeters; purge_volume is
  cubic millimeters. In contrast, geometry points written into output roots use
  Slic3r scaled integer coordinates IN THAT SAME LOCAL FRAME (scale mm before
  writing c_point values). Geometry property width/height remain millimeters,
  flow is mm3/mm, and explicit EPropertySpeed speeds are mm/s. Wipe services own
  their speeds; the default object-speed plugin does not assign wipe speeds.
  Source layer groups are assigned in print order to increasing tower Zs. A close
  group (its bottom overlaps the tower top) always shares the top layer, so its
  few bands never push the tower top above the source Zs; an equal or lower group
  shares it while its bands fit the tower's largest usage so far, otherwise a
  layer is stacked above using the source group's thinnest layer height. Band
  print_z can therefore differ from
  the anchor group's print_z. The manager sets a root EPropertyZOffset to their
  difference, in scaled coordinates, on both service output and maintenance.
  A request with a nonzero floating flag never makes the tower grow. The manager
  lays out the other requests first: a floating request whose source Z is at or
  below that tower top is laid out with them, exactly like a normal request.
  Above the top, its band is reserved as the first band of its own floating layer
  at the source Z; that layer receives no maintenance and changes neither the
  layer depth nor the cone.

Output handling:
  Roots start empty and are owned by the manager, not the service's storage.
  A service may move its temporary geometry into them, but may not free or retain
  the roots. A disabled request part has a NULL root. The manager drops untouched
  roots (no local points, children, or properties); property-only command output
  is retained. It rotates each retained root about the local origin, then
  translates it to the platter position, fixes the root as non-sortable and
  non-reversible, and moves it into a PrintingExtrusion with role WIPE_TOWER,
  the synthetic tower catalog id, and no source region island. All generation
  finishes before descending-anchor insertion; equal anchors retain collection
  order. The manager generates brim, perimeter, unused-area fill and optional cone
  after all GENERATE calls, placing maintenance before the bands of its visit.
  Only after all insertion does it remove its otherwise empty forced visits.

Lifetime and failures:
  All payload pointers and views are borrowed only for the current callback.
  report_context/report_request may only be used synchronously during COLLECT.
  The reporter copies *request before returning, so the submitted value may be
  stack-local or reused immediately. GENERATE's request is a manager-owned copy;
  band, roots, and exit remain manager-owned. exit starts at band.entry; services
  may return their last local XY point through it. That exit becomes the suggested
  entry of the next band on the same tower and target tool visit (incoming visit
  when present, otherwise outgoing). New visits use alternating band sides.
  Do not retain any of these pointers.
  The reporter returns 1 after copying a request and 0 for invalid arguments or
  allocation failure. On 0, a service must end collection with no further request
  and return normally without reporting a plugin error. The manager retains the
  reporter's allocation exception and rethrows it after COLLECT. A service still
  reports its own failures using its host context; the dispatch callback returns
  0 on service failure and the host preserves the original service
  diagnostic/exception for the step.
*/
#ifndef slic3r_step_wipe_tower_wipe_h_
#define slic3r_step_wipe_tower_wipe_h_

#define SLIC3R_PLUGIN_API_STEP_WIPE_TOWER_WIPE_MAJOR 2u
#define SLIC3R_PLUGIN_API_STEP_WIPE_TOWER_WIPE_MINOR 0u

#include "../slic3r_printing_plan.h"
#include "slic3r_step_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum c_wipe_tower_phase {
    C_WIPE_TOWER_EDIT = 0,
    C_WIPE_TOWER_COLLECT = 1,
    C_WIPE_TOWER_GENERATE = 2
} c_wipe_tower_phase;

/* Zero-based indices in the post-EDIT plan; insert before extrusion_idx, or at the end when it equals that tool group's extrusion count. */
typedef struct c_wipe_tower_anchor {
    uint32_t group_idx;
    uint32_t layer_group_idx;
    uint32_t tool_group_idx;
    uint32_t extrusion_idx;
} c_wipe_tower_anchor;

/* At least one part must be enabled; both parts, when present, share a group/layer and use their anchored tool groups' concrete zero-based extruders. A nonzero floating flag marks a band that may be printed above the tower top without support. */
typedef struct c_wipe_tower_request {
    /* Service-owned identifier copied unchanged into GENERATE; the manager does not interpret it. */
    uint64_t private_id;
    /* Ignored when has_incoming is zero; geometry is inserted into this tool visit. */
    c_wipe_tower_anchor incoming;
    /* Ignored when has_outgoing is zero; geometry is inserted into this tool visit. */
    c_wipe_tower_anchor outgoing;
    /* Zero disables a part; nonzero enables it. */
    int32_t has_incoming;
    int32_t has_outgoing;
    /* Finite nonnegative requested volume in mm3; one band is shared by both parts. */
    double purge_volume;
    /* Zero: a normal band. Nonzero: this band may be printed above the top of the
       tower without support, and it never makes the tower grow. */
    int32_t floating;
} c_wipe_tower_request;

/* Reserved local rectangle and borrowed empty roots; all doubles/vec2d values are millimeters, but root geometry uses scaled local coordinates. */
typedef struct c_wipe_tower_band {
    /* Synthetic plan catalog entry identifying the tower, not a source PrintObject. */
    uint32_t tower_catalog_id;
    /* Platter Z and tower-layer thickness, independent of local XY. */
    double print_z;
    double layer_height;
    /* Opposite corners [0,W] x [y0,y1] and suggested local starting point. */
    c_vec2d min;
    c_vec2d max;
    c_vec2d entry;
    /* Resolved line width for the incoming tool, or the outgoing tool for an outgoing-only request. */
    double extrusion_width;
    /* Non-NULL only for enabled parts; geometry may be edited/moved into these manager-owned roots. */
    extrusion_entity_handle *outgoing_root;
    extrusion_entity_handle *incoming_root;
    /* Distance in millimeters between the centres of two consecutive purge lines, used by the
       manager to reserve [y0,y1]: the rounded Flow spacing of extrusion_width times the tower's
       wipe_tower_extra_spacing ratio. */
    double line_spacing;
} c_wipe_tower_band;

/* Copy a non-NULL request synchronously through the borrowed COLLECT context; return 1 on success or 0 on invalid arguments/allocation failure.
   On 0, the service must end collection normally without further requests or calling report_error; the manager retains and rethrows allocation failures after COLLECT. */
typedef int32_t (*wipe_tower_report_request_fn)(void *context, const c_wipe_tower_request *request);

/* Borrowed service payload; field availability, ownership, and units are defined above. */
typedef struct run_ctx_wipe_tower_wipe {
    const print_handle *print;
    const printing_plan_handle *plan;
    printing_plan_handle *mutable_plan;
    c_wipe_tower_phase phase;
    void *report_context;
    wipe_tower_report_request_fn report_request;
    const c_wipe_tower_request *request;
    const c_wipe_tower_band *band;
    c_vec2d *exit;
} run_ctx_wipe_tower_wipe;

/* Return the borrowed wipe payload only for the wipe service step. */
static inline const run_ctx_wipe_tower_wipe *
plugin_ctx_as_wipe_tower_wipe(const plugin_run_context *ctx)
{
    if (!ctx || ctx->step != WIPE_TOWER_WIPE)
        return NULL;
    return (const run_ctx_wipe_tower_wipe *)ctx->data;
}

#ifdef __cplusplus
}
static_assert(sizeof(c_wipe_tower_phase) == sizeof(int32_t),
              "c_wipe_tower_phase must have the int32_t ABI representation");
#endif
#endif
