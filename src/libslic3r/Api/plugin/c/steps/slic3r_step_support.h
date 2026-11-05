///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_step_support_h_
#define slic3r_step_support_h_

#define SLIC3R_PLUGIN_API_STEP_SUPPORT_MAJOR 1u
#define SLIC3R_PLUGIN_API_STEP_SUPPORT_MINOR 3u

#include "slic3r_step_common.h"
#include "slic3r_step_infill.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
Declared before the callback types below: in C, a struct tag first named inside
a prototype has prototype scope and would not be the struct defined later.
*/
struct run_ctx_generate_support;

/*
Resolve a stable INFILL_PATTERN plugin id, for example "rectilinear" or
"supportbase", to the runtime id used by generate_pattern() during this step
run. Same semantics as run_ctx_generate_infill::resolve_pattern_id(): an
inactive id falls back to the first active pattern with a warning, and
INFILL_PATTERN_RUNTIME_ID_INVALID is returned only when no pattern is active.
Runtime ids are valid only for the current step run; never store them.

The first call of either callback in the step builds the pattern service, which
runs the setup() of every active pattern that this step may call.
INFILL_PATTERN_RUNTIME_ID_INVALID is also returned when that construction fails
(a pattern setup() reports an error) or the print is cancelled; see
support_generate_pattern_fn for what an error does to the step.

ctx is the run_ctx_generate_support received by the plugin.
*/
typedef infill_pattern_runtime_id (*support_resolve_pattern_id_fn)(
    const struct run_ctx_generate_support *ctx,
    const char *pattern_plugin_id);

/*
Fill one surface with the INFILL_PATTERN plugin selected by pattern_id. Same
arguments and semantics as the STEP_SUPPORT_TOOLPATH callback
(support_toolpath_generate_pattern_fn), with the STEP_SUPPORT payload as ctx.

The pattern receives ctx->print and ctx->object, the given layer, island,
region_island, primary_region, surface, no_overlap_areas and params, and
writes its paths into output. output must be an empty mutable extrusion entity
owned by the caller for the call. The host never publishes output at this step:
the caller only reads it (for example to sample points) and frees it itself.

At STEP_SUPPORT the support layers usually do not exist yet, so layer (an
object layer, or any layer of the object), island, region_island,
primary_region and no_overlap_areas may all be NULL. rectilinear, supportbase
and honeycomb do not read them. Other patterns are not guaranteed to accept
NULL: some read primary_region (arachne, smooth, ensuring), so pass real
handles when a pattern other than those three is used.

Returns 1 when the pattern ran, even if it produced nothing, and 0 when nothing
was generated because of an error: an unknown runtime id (warning message), or
a pattern that needs the STEP_INFILL initialization pass (lightning,
adaptivecubic, supportcubic; error message). A call returning 0 generates
nothing: the caller treats it as an empty fill. No recipe modifier is applied.

A failure inside the call never crosses this C boundary as an exception, so
callers written in C or in a language binding (Python through ctypes) are safe.
An error reported by the pattern itself, a failed service construction or a
cancellation makes the call return 0 and leaves output an empty collection:
children and a path written directly on it are removed, even when the pattern
had already written part of it. The error is recorded as a plugin error message
and the plugin cancellation is requested, as for any plugin error; the call
returns, and the host stops the step with the standard "was cancelled" error
when the calling family returns. A cancellation of the print stops the step
the same way. After a 0 return caused by an error, return from the family
soon (is_cancelled() is then true).
*/
typedef int32_t (*support_generate_pattern_fn)(
    const struct run_ctx_generate_support *ctx,
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

The dense areas of a support contact layer carry RAW_SURFACE_TYPE_MOD_CONTACT:
the area touching the object above or below and the margin of that contact, that
is the dense area of the contact layer around it. A bridging lower contact
carries it too. A dense top or bottom support surface without it is an
interface of its side. A contact merged into the base (the case below) is
published as sparse base and carries no contact bit. Families must mark every
contact area: no host check verifies the marking, and the toolpath algorithms
read the bit to select contact handling. On a side with interface layers,
contacts use the configured contact pattern. On a side without interface layers,
interfaces and contacts both use the base settings and bucket; the bit only
selects contact handling, such as contact patterns and loops. The bridge modifier
selects bridge flow independently. Toolpath providers use five roles: sparse,
bottom contact, bottom interface, top contact and top interface. A contact
provider may hand areas back by republishing the same surfaces with only contact
cleared, preserving geometry, order, properties and every other flag.
A package built against 1.2 does not mark contacts, so its contact areas are
classified as interfaces: they use interface settings when enabled on that
side, otherwise base settings and bucket without contact handling.

A contact is published as support | internal | sparse when no interface layer is configured on its
side and the support regions can merge (one extruder, no bridging contact, and the merged base
layer of that print Z, whose thickness is the smallest of that Z, has the contact's thickness or does
not exist): the user asked for no interface, so that contact is base.

Within one auxiliary layer these three kinds must stay disjoint: an area
published twice is extruded twice. An area can be both upper and lower when the
gap between the object above and the object below is thinner than the support
layers; such an area belongs to the top surface, as the native generator merged
the lower contact into the upper one.

The family writes no extrusions: STEP_SUPPORT_TOOLPATH generates them from
these surfaces. Use the print and object handles only during the callback;
never free them.

Infill patterns: a family may sample any active INFILL_PATTERN plugin through
resolve_pattern_id() and generate_pattern(), passing this payload back as ctx,
for example to place the tips of a tree. The first callback of the step builds
the pattern service, which runs the setup() of every active pattern that this
step may call (not lightning, adaptivecubic and supportcubic, which need the
STEP_INFILL initialization pass); a run that never calls them pays nothing. When
several threads make the first call together, one builds it and the others wait.
If the construction fails, the call that triggered it returns 0 (or
INFILL_PATTERN_RUNTIME_ID_INVALID) and the next caller tries to build it again.
Once built, both callbacks may be called concurrently, from several objects and
from the family's own worker threads. Unlike STEP_INFILL there is no recipe
modifier and no pattern initialization pass. The run count advertised to the
progress of the patterns is the object count of the print; their progress
display is clamped, so more calls than that are harmless.

The caller builds raw_infill_pattern_params itself; the host fills nothing. The
fields to set are described in steps/slic3r_step_support_toolpath.h, above
run_ctx_support_toolpath. A call returning 0 generated nothing.
*/
typedef struct run_ctx_generate_support {
    const print_handle *print;
    const object_handle *object;

    /* Fields below were added in minor version 2. */

    /*
    Opaque host state behind the callbacks. Pass the original
    run_ctx_generate_support pointer back to the callbacks; never inspect or
    store host_context.
    */
    void *host_context;
    support_resolve_pattern_id_fn resolve_pattern_id;
    support_generate_pattern_fn generate_pattern;
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
