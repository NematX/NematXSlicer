///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_step_infill_h_
#define slic3r_step_infill_h_

#define SLIC3R_PLUGIN_API_STEP_INFILL_MAJOR 2u
#define SLIC3R_PLUGIN_API_STEP_INFILL_MINOR 0u

#include "slic3r_step_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
STEP_INFILL plugin interface
============================

The host runs STEP_INFILL plugins in priority order. The generator exclusive
group selects one provider without excluding other passes on the step. The
generator resolves a recipe and applies surface modifiers during setup_run(),
reports grouped work to the host, and calls the selected INFILL_PATTERN plugin
during run(). The host owns pattern selection, plugin contexts and extrusion
publication. Payload handles borrow print data for the current callback; the
recipe modifier edits the generator-owned recipe before the pattern consumes it.
*/

/* Declare the payload tag here so C callback prototypes do not introduce distinct prototype-scope types. */
struct run_ctx_generate_infill;

/*
Runtime id for an active INFILL_PATTERN plugin.

The value is assigned by the STEP_INFILL host wrapper for the current process
run. It is intentionally not serialized: config files and presets keep using
the stable string plugin id. Recipe code uses this compact id only after asking
the host to resolve a string id for the current active pattern set.
*/
typedef uint32_t infill_pattern_runtime_id;

#define INFILL_PATTERN_RUNTIME_ID_INVALID ((infill_pattern_runtime_id)0)

/*
Boolean recipe options, grouped in one bitmask.

raw_infill_pattern_params::infill_flags stores a combination of these values.
The same bits are stored in the infill extrusion property
(c_extrusion_property_infill) so post-infill plugins can read the flags of the
surface that produced a subtree without re-reading the configuration.
*/
typedef enum c_infill_flag {
    /* The surface may split its area into normal infill plus a gap-fill pass. */
    C_INFILL_FLAG_ADD_GAP_FILL = 1u << 0,
    /* The pipeline can produce gap fill, for patterns that leave residuals. */
    C_INFILL_FLAG_GAP_FILL_ENABLED = 1u << 1,
    /* The fill flow must not adjust the spacing. */
    C_INFILL_FLAG_DONT_ADJUST = 1u << 2,
    /* The pattern must keep its paths monotonic. */
    C_INFILL_FLAG_MONOTONIC = 1u << 3,
    /* The surface requests an enforced full fill volume. */
    C_INFILL_FLAG_FILL_EXACTLY = 1u << 4,
    /* The pattern must fill the surface completely. */
    C_INFILL_FLAG_COMPLETE = 1u << 5,
    /* The pattern runs through the Arachne variable-width path. */
    C_INFILL_FLAG_USE_ARACHNE = 1u << 6,
    /* The pattern frame may cross the fill angle. */
    C_INFILL_FLAG_CAN_ANGLE_CROSS = 1u << 7,
    /* Full-density lines cover the surface as well as they can: their phase is chosen to give the
       longest extruded lines, a line near the border counting even where it overlaps the border. */
    C_INFILL_FLAG_BEST_COVERAGE = 1u << 8,
    /* With C_INFILL_FLAG_BEST_COVERAGE, only the part of the lines at least half a spacing from the
       border counts, so the chosen phase avoids lines that overlap the border. For surfaces with a
       clearance to keep, such as the XY gap of support interfaces. */
    C_INFILL_FLAG_BEST_COVERAGE_STRICT = 1u << 9
} c_infill_flag;

/*
Connection policy for generated infill paths. The numeric values match the
serialized legacy InfillConnection values used by the host recipe.
*/
typedef enum c_infill_connection {
    C_INFILL_CONNECTION_CONNECTED = 0,
    C_INFILL_CONNECTION_HOLES = 1,
    C_INFILL_CONNECTION_OUTER_SHELL = 2,
    C_INFILL_CONNECTION_NOT_CONNECTED = 3
} c_infill_connection;

/*
Parameters prepared by STEP_INFILL before it calls one INFILL_PATTERN plugin.

The pattern plugin should treat this struct as the complete per-surface recipe:
the host has already resolved the selected pattern id, role, flow, density,
angle, overlap and gap-fill flags from the print/object/region settings.

All distances stored as coord_t are scaled integer coordinates. All double or
float distances are unscaled millimeters unless the field comment says
otherwise. Angles are radians.
*/
typedef struct raw_infill_pattern_params {
    /*
    Runtime id of the active INFILL_PATTERN plugin selected for this surface.

    STEP_INFILL generators obtain it with resolve_pattern_id(). A recipe
    modifier may replace it with another resolved runtime id. Pattern plugins
    normally do not need to read this value; they are called because the host
    selected their plugin from this id.
    */
    infill_pattern_runtime_id pattern_id;

    raw_surface_type surface_type;
    raw_extrusion_role extrusion_role;
    c_flow flow;

    float density;
    float flow_mult;
    c_infill_connection connection;

    /*
    Bitmask of c_infill_flag values. It replaces the individual boolean fields
    the legacy FillParams exposed. C_INFILL_FLAG_ADD_GAP_FILL is the exact
    per-surface decision that old FillParams exposed;
    C_INFILL_FLAG_GAP_FILL_ENABLED is the broader pipeline state, useful to
    pattern implementations that want to leave narrow residuals for a later
    post-process instead of trying to cover every sliver themselves.
    */
    uint16_t infill_flags;

    uint32_t extruder;
    int32_t priority;

    double spacing;
    double angle;
    double bridge_angle;
    int32_t bridge_type;
    double layer_height;
    double z;
    uint32_t layer_id;
    double overlap;

    coord_t bridge_offset;
    coord_t fill_resolution;
    coord_t link_max_length;
    coord_t loop_clipping;
    float anchor_length;
    float anchor_length_max;
    float max_sparse_infill_spacing;

    /*
    Scaled bounding box of the whole object being filled. Legacy Fill
    implementations used it to align sparse patterns across layers and to
    rotate the pattern frame around the object center; plugins that do not need
    cross-layer alignment can ignore it.
    */
    c_bounding_box object_bounding_box;
} raw_infill_pattern_params;

/*
Resolve a stable INFILL_PATTERN plugin id to the runtime id used by this run.

pattern_plugin_id is the serialized string value stored in the project config,
for example "rectilinear" or "line". The callback returns a non-zero runtime id
for an active pattern plugin. If the requested plugin is not active, the host
may report a warning and return a fallback pattern so slicing can continue.

The ctx argument is the parent run_ctx_generate_infill. The callback needs it
because this is a C ABI: function pointers do not capture the host registry
state by themselves.

The callback never lets a C++ exception cross the ABI. It returns INVALID when
the step is already stopping or when a failure is contained; an inactive id may
still produce the documented warning and fallback. The runner observes the
requested stop after setup_run() returns.
*/
// Resolve a configured pattern id to the runtime id used by the current step run.
typedef infill_pattern_runtime_id (*infill_resolve_pattern_id_fn)(
    const struct run_ctx_generate_infill *ctx,
    const char *pattern_plugin_id);

/*
Return the stable plugin id behind a runtime INFILL_PATTERN id.

This is mostly for diagnostics and tests. The returned pointer is borrowed from
the host plugin registry and must not be stored beyond the current callback.
As with resolve_pattern_id(), ctx is the parent run_ctx_generate_infill used to
reach the host-side runtime registry. The result is never NULL and is "" for an
unknown runtime id or a step that is already stopping. The host lookup does not
allocate and never throws.
*/
// Return the borrowed stable plugin id for a runtime pattern id.
typedef const char *(*infill_pattern_plugin_id_fn)(
    const struct run_ctx_generate_infill *ctx,
    infill_pattern_runtime_id pattern_id);

/*
Generate one surface with the INFILL_PATTERN plugin selected by pattern_id.

STEP_INFILL plugins should call this instead of directly looking up pattern
plugins. The host owns plugin selection, stale-pattern fallback, per-pattern
setup_run(), progress plumbing and error reporting. output must be an empty
mutable extrusion entity owned by the caller for the duration of the call; the
selected pattern writes its generated paths into it. An ordinary refusal
returns 0 with output unchanged. A C++ failure returns 0 with output empty;
except for print cancellation, the host records the exception and requests
plugin cancellation. An already requested stop returns 0 with output empty and
no new message. CanceledException is left for the runner to rethrow as print
cancellation.
*/
// Generate paths for one surface into the caller-owned output entity.
typedef int32_t (*infill_generate_pattern_fn)(
    const struct run_ctx_generate_infill *ctx,
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
Run all active INFILL_SURFACE_RECIPE_MODIFIER plugins for one fill surface.

STEP_INFILL generators should call this after creating the default
raw_infill_pattern_params and before calling generate_pattern(). The host calls
the active recipe modifiers in plugin priority order. Each modifier receives the
same mutable params, so later modifiers see changes made by earlier modifiers.
The chain stops at its first contained failure and returns 0; edits made by
earlier modifiers remain in params. The step requests plugin cancellation and
does not consume the recipe. An already requested stop returns 0 without another
message, and print cancellation is rethrown by the runner.
*/
// Apply the active recipe modifiers to one mutable surface recipe.
typedef int32_t (*infill_modify_surface_recipe_fn)(
    const struct run_ctx_generate_infill *ctx,
    const layer_handle *layer,
    const layer_island_handle *island,
    const layer_region_island_handle *region_island,
    const layer_region_handle *primary_region,
    const surface_handle *surface,
    const expolygon_collection_handle *no_overlap_areas,
    raw_infill_pattern_params *params);

/*
Move generated extrusion into the LayerRegionIsland bucket matching role.

The host receives ownership by moving children out of extrusion. The caller may
reuse or destroy extrusion after the function returns, but must not expect it to
still contain the moved paths. The host stages children before publication; if
staging fails, the callback returns 0, previously published destination paths
remain unchanged, and the source may be partly consumed. The failure is
reported and requests plugin cancellation. Null inputs or an existing plugin
stop return 0 without publication. This callback has no print handle; the
generator and runner handle print cancellation.
*/
// Append the source extrusion's generated children to its role bucket.
typedef int32_t (*infill_append_region_island_extrusion_fn)(
    layer_region_island_handle *region_island,
    raw_extrusion_role role,
    extrusion_entity_handle *extrusion);

/*
One work group discovered by a STEP_INFILL generator for one INFILL_PATTERN
plugin: all surfaces of one LayerRegionIsland that share the same resolved
recipe.

The generator owns this data. It stays valid from the discovery pass until the
generator run for the same object completes; pattern plugins must copy anything
they need beyond their setup() call.
*/
typedef struct raw_infill_pattern_assignment {
    const layer_island_handle *layer_island;
    const layer_region_island_handle *region_island;
    const raw_infill_pattern_params *params;
    const surface_handle *const *surfaces;
    uint32_t surface_count;
} raw_infill_pattern_assignment;

/* One object and the work groups it contains for one pattern plugin. */
typedef struct raw_infill_pattern_object_init {
    const object_handle *object;
    const raw_infill_pattern_assignment *assignments;
    uint32_t assignment_count;
} raw_infill_pattern_object_init;

/*
Per-print initialization payload delivered to an INFILL_PATTERN plugin.

A generator reports its discovery result through
run_ctx_generate_infill::report_pattern_assignments() during its setup_run()
phase. After all object discoveries finish, the host services aggregate the
groups and call setup() once before the first normal pattern generation. The plugin may process the objects
in parallel; the data only describes work that the current print will generate.
*/
typedef struct run_ctx_infill_pattern_init {
    const print_handle *print;
    const raw_infill_pattern_object_init *objects;
    uint32_t object_count;
} run_ctx_infill_pattern_init;

static inline const run_ctx_infill_pattern_init *
plugin_ctx_as_infill_pattern_init(const plugin_run_context *ctx)
{
    if (!ctx || ctx->step != INFILL_PATTERN)
        return NULL;
    return (const run_ctx_infill_pattern_init *)ctx->data;
}

/*
Report the work groups one STEP_INFILL generator discovered for one pattern
plugin in one object.

Generators call this once per used pattern during their setup_run() phase. The
host copies the groups into its own storage, so the arrays only need to be valid
for the duration of the call. Unused patterns are never reported and therefore
do not receive an initialization call. Null or invalid inputs and an existing
stop return 0 without another message. A contained C++ failure returns 0,
records the failure and requests plugin cancellation; the runner sees the stop
after setup_run() and before pattern initialization.
*/
// Copy this object's discovered groups for one pattern into host-owned storage.
typedef int32_t (*infill_report_pattern_assignments_fn)(
    const struct run_ctx_generate_infill *ctx,
    infill_pattern_runtime_id pattern_id,
    const raw_infill_pattern_assignment *assignments,
    uint32_t assignment_count);

/*
Payload for STEP_INFILL.

Normal usage:
- setup_run() phase: walk the object, build one raw_infill_pattern_params per
  fill surface, apply modify_surface_recipe(), group surfaces that share one
  recipe and report each group with report_pattern_assignments(). The host then
  initializes every used INFILL_PATTERN plugin once for the print;
- run() phase: iterate the object layers, islands, region islands and fill
  surfaces;
- call generate_pattern() to delegate line creation to the selected
  INFILL_PATTERN plugin;
- call append_region_island_extrusion() to publish non-empty output.

A generator may keep the recipes it discovered between setup_run() and run()
instead of recomputing them; the reported params must stay valid until the run
for that object completes.

host_context is an opaque pointer reserved for the host callbacks. Plugins must
pass the original run_ctx_generate_infill pointer back to callbacks and must not
inspect or store host_context.
*/
typedef struct run_ctx_generate_infill {
    const print_handle *print;
    const object_handle *object;
    void *host_context;

    infill_resolve_pattern_id_fn resolve_pattern_id;
    infill_pattern_plugin_id_fn pattern_plugin_id;
    infill_generate_pattern_fn generate_pattern;
    infill_modify_surface_recipe_fn modify_surface_recipe;
    infill_append_region_island_extrusion_fn append_region_island_extrusion;
    /*
    Discovery reporting used during setup_run(). The generator walks the object,
    resolves every fill surface recipe, groups surfaces that share one recipe,
    and reports each group for its pattern plugin. The host aggregates the
    groups and initializes the used patterns before the first run.
    */
    infill_report_pattern_assignments_fn report_pattern_assignments;
} run_ctx_generate_infill;

static inline const run_ctx_generate_infill *
plugin_ctx_as_generate_infill(const plugin_run_context *ctx)
{
    if (!ctx || ctx->step != STEP_INFILL)
        return NULL;
    return (const run_ctx_generate_infill *)ctx->data;
}

/*
Payload for INFILL_PATTERN service plugins.

Normal usage:
- setup() receives a run_ctx_infill_pattern_init describing every object and
  work group this plugin will fill in the current print. Use it to precompute
  per-object state (grids, bounding boxes, synchronization) before any run;
- setup_run() is called once per surface with the run_ctx_infill_pattern below;
- run() reads surface and no_overlap_areas as the geometry to fill, reads params
  as the already-resolved process settings, writes generated extrusion into
  output, and returns with output empty when this surface produces no extrusion.

output is a mutable, storage-independent extrusion entity borrowed from the
host. It starts empty for each call and is valid only during the call. The host
moves its children under the LayerRegionIsland infill root if the plugin writes
anything into it.
*/
typedef struct run_ctx_infill_pattern {
    const print_handle *print;
    const object_handle *object;
    const layer_handle *layer;
    const layer_island_handle *island;
    const layer_region_island_handle *region_island;
    const layer_region_handle *primary_region;
    const surface_handle *surface;
    const expolygon_collection_handle *no_overlap_areas;
    const raw_infill_pattern_params *params;
    extrusion_entity_handle *output;
} run_ctx_infill_pattern;

static inline const run_ctx_infill_pattern *
plugin_ctx_as_infill_pattern(const plugin_run_context *ctx)
{
    if (!ctx || ctx->step != INFILL_PATTERN)
        return NULL;
    return (const run_ctx_infill_pattern *)ctx->data;
}

/*
Payload for INFILL_SURFACE_RECIPE_MODIFIER service plugins.

Normal usage:
- inspect surface, no_overlap_areas, layer/island context and plugin
  properties stored by earlier surface-generation plugins;
- edit params in place to change how this surface will be filled;
- optionally replace params->pattern_id with another id returned by
  resolve_pattern_id().

This payload is deliberately small and per-surface. Recipe modifiers should not
split surfaces or create extrusion. Geometry changes belong in surface
generation or post-infill plugins; this service only changes the recipe consumed
by the selected infill pattern.
*/
typedef struct run_ctx_infill_surface_recipe_modifier {
    const print_handle *print;
    const object_handle *object;
    const layer_handle *layer;
    const layer_island_handle *island;
    const layer_region_island_handle *region_island;
    const layer_region_handle *primary_region;
    const surface_handle *surface;
    const expolygon_collection_handle *no_overlap_areas;

    /*
    Parent STEP_INFILL context required by resolve_pattern_id() and
    pattern_plugin_id().

    The modifier payload is intentionally small and does not duplicate the host
    plugin registry state. Pass this pointer back to the resolver callbacks when
    replacing params->pattern_id:

        params->pattern_id =
            ctx->resolve_pattern_id(ctx->generate_infill_ctx, "line");
    */
    const struct run_ctx_generate_infill *generate_infill_ctx;
    infill_resolve_pattern_id_fn resolve_pattern_id;
    infill_pattern_plugin_id_fn pattern_plugin_id;
    raw_infill_pattern_params *params;
} run_ctx_infill_surface_recipe_modifier;

static inline const run_ctx_infill_surface_recipe_modifier *
plugin_ctx_as_infill_surface_recipe_modifier(const plugin_run_context *ctx)
{
    if (!ctx || ctx->step != INFILL_SURFACE_RECIPE_MODIFIER)
        return NULL;
    return (const run_ctx_infill_surface_recipe_modifier *)ctx->data;
}

#ifdef __cplusplus
}

static_assert(sizeof(c_infill_connection) == sizeof(int32_t),
              "c_infill_connection must have the raw int32_t ABI representation");
#endif

#endif // slic3r_step_infill_h_
