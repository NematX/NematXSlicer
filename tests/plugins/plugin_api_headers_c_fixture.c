///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
/* Compile slic3r_volume.h, slic3r_plugin_run_context.h and slic3r_clipper.h as C with their real
   dependencies (slic3r_utils.h, slic3r_data_tree.h, slic3r_slicing_step.h, ...): nothing is
   neutralised. Function pointers declared with the full signature and initialised without a cast
   make the compiler type-check the parameters and return type of the declarations; the pragmas below
   turn an incompatible pointer assignment into an error (MSVC only warns by default). Compile-time
   array checks verify the type and value of the slicing step constants.
   The step headers use their real dependencies. Exact C callback assignments
   and payload-field calls check that callback parameters use file-scope payload
   types, not distinct structs introduced inside function prototypes. The step
   calls are type-checked in uncalled functions and never reach a live host.
   Finalization also instantiates the fixed-size bridge payloads from slic3r_bridge.h.

   Calls written in the unexecuted compile-only probe body:

       test_finalize_callback_types_c() [never invoked by the test runtime]
           |-- ctx->replace_infill_buckets() [typed callback expression]
           `-- ctx->get_requested_support_areas() [typed callback expression]

   This checks declarations and C type compatibility, not runtime ownership or
   transactional geometry behavior. */
#if defined(_MSC_VER)
#pragma warning(1 : 4115)
#pragma warning(error : 4047 4113 4115 4133)
#elif defined(__GNUC__)
#pragma GCC diagnostic error "-Wincompatible-pointer-types"
#endif
#include "libslic3r/Api/plugin/c/slic3r_volume.h"
#include "libslic3r/Api/plugin/c/slic3r_plugin_run_context.h"
#include "libslic3r/Api/plugin/c/slic3r_clipper.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_infill.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_perimeter.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_support_toolpath.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_wipetower.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_finalize_slicing.h"
#include "libslic3r/Api/plugin/c/slic3r_bridge.h"
#ifdef __cplusplus
#error This fixture must be compiled as C.
#endif

/* Borrow but ignore the payload, storage and entries; return success without
   mutation. Its exact C signature checks the finalization callback type. */
static int32_t c_final_replace(const run_ctx_finalize_slicing *ctx, storage_handle *storage,
    const raw_final_infill_replacement *entries, uint32_t count)
{
    (void)ctx; (void)storage; (void)entries; (void)count;
    return 1;
}

/* Compile-only probe, never called: assign and invoke both finalization callbacks
   and instantiate bridge payloads so missing fields or incompatible types fail. */
void test_finalize_callback_types_c(run_ctx_finalize_slicing *ctx)
{
    final_replace_infill_fn replace = c_final_replace;
    c_bridge_surface_property surface = {0};
    c_bridge_volume_property volume = {0};
    ctx->replace_infill_buckets = replace;
    (void)ctx->replace_infill_buckets(ctx, NULL, NULL, 0);
    (void)ctx->get_requested_support_areas(NULL);
    (void)surface; (void)volume;
}

/* Borrow the infill payload and id string without reading host state; return a fixed id. */
static infill_pattern_runtime_id c_infill_resolve(const run_ctx_generate_infill *ctx, const char *id)
{
    (void)ctx; (void)id;
    return 1;
}

/* Borrow the infill payload and return a static string with no ownership transfer. */
static const char *c_infill_id(const run_ctx_generate_infill *ctx, infill_pattern_runtime_id id)
{
    (void)ctx; (void)id;
    return "rectilinear";
}

/* Borrow all generation arguments without touching output; return success. */
static int32_t c_infill_generate(const run_ctx_generate_infill *ctx, infill_pattern_runtime_id id,
    const layer_handle *layer, const layer_island_handle *island,
    const layer_region_island_handle *region_island, const layer_region_handle *region,
    const surface_handle *surface, const expolygon_collection_handle *areas,
    const raw_infill_pattern_params *params, extrusion_entity_handle *output)
{
    (void)ctx; (void)id; (void)layer; (void)island; (void)region_island;
    (void)region; (void)surface; (void)areas; (void)params; (void)output;
    return 1;
}

/* Borrow the recipe and geometry without mutation; return success. */
static int32_t c_infill_modify(const run_ctx_generate_infill *ctx,
    const layer_handle *layer, const layer_island_handle *island,
    const layer_region_island_handle *region_island, const layer_region_handle *region,
    const surface_handle *surface, const expolygon_collection_handle *areas,
    raw_infill_pattern_params *params)
{
    (void)ctx; (void)layer; (void)island; (void)region_island;
    (void)region; (void)surface; (void)areas; (void)params;
    return 1;
}

/* Borrow the assignment array without reading or retaining it; return success. */
static int32_t c_infill_report(const run_ctx_generate_infill *ctx, infill_pattern_runtime_id id,
    const raw_infill_pattern_assignment *assignments, uint32_t count)
{
    (void)ctx; (void)id; (void)assignments; (void)count;
    return 1;
}

/* Borrow the perimeter arguments without dispatching the node callback; return success. */
static int32_t c_perimeter_group(const run_ctx_generate_perimeter *ctx,
    const layer_region_handle *const *regions, uint32_t count, const expolygon_handle *area,
    void *state, perimeter_generate_node_fn generate)
{
    (void)ctx; (void)regions; (void)count; (void)area; (void)state; (void)generate;
    return 1;
}

/* Borrow the support payload and id string without reading host state; return a fixed id. */
static infill_pattern_runtime_id c_toolpath_resolve(const run_ctx_support_toolpath *ctx, const char *id)
{
    (void)ctx; (void)id;
    return 1;
}

/* Borrow all support generation arguments without touching output; return success. */
static int32_t c_toolpath_generate(const run_ctx_support_toolpath *ctx, infill_pattern_runtime_id id,
    const layer_handle *layer, const layer_island_handle *island,
    const layer_region_island_handle *region_island, const layer_region_handle *region,
    const surface_handle *surface, const expolygon_collection_handle *areas,
    const raw_infill_pattern_params *params, extrusion_entity_handle *output)
{
    (void)ctx; (void)id; (void)layer; (void)island; (void)region_island;
    (void)region; (void)surface; (void)areas; (void)params; (void)output;
    return 1;
}

/* Borrow the tower payloads without dispatching a service or mutating either; return success. */
static int32_t c_wipe_call(const run_ctx_generate_wipe_tower *ctx, uint32_t index,
    run_ctx_wipe_tower_wipe *payload)
{
    (void)ctx; (void)index; (void)payload;
    return 1;
}

/* Type-check all nine callback assignments and calls on real payload pointers.
   This externally visible function is never called by the test: no host service runs. */
void test_step_callback_types_c(run_ctx_generate_infill *infill, run_ctx_generate_perimeter *perimeter,
    run_ctx_support_toolpath *toolpath, run_ctx_generate_wipe_tower *tower)
{
    infill_resolve_pattern_id_fn resolve = c_infill_resolve;
    infill_pattern_plugin_id_fn id = c_infill_id;
    infill_generate_pattern_fn generate = c_infill_generate;
    infill_modify_surface_recipe_fn modify = c_infill_modify;
    infill_report_pattern_assignments_fn report = c_infill_report;
    perimeter_run_region_group_fn group = c_perimeter_group;
    support_toolpath_resolve_pattern_id_fn support_resolve = c_toolpath_resolve;
    support_toolpath_generate_pattern_fn support_generate = c_toolpath_generate;
    wipe_tower_call_plugin_fn wipe = c_wipe_call;
    infill->resolve_pattern_id = resolve;
    infill->pattern_plugin_id = id;
    infill->generate_pattern = generate;
    infill->modify_surface_recipe = modify;
    infill->report_pattern_assignments = report;
    perimeter->run_region_group = group;
    toolpath->resolve_pattern_id = support_resolve;
    toolpath->generate_pattern = support_generate;
    tower->call_wipe_plugin = wipe;
    (void)infill->resolve_pattern_id(infill, "rectilinear");
    (void)infill->pattern_plugin_id(infill, 1);
    (void)infill->generate_pattern(infill, 1, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    (void)infill->modify_surface_recipe(infill, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    (void)infill->report_pattern_assignments(infill, 1, NULL, 0);
    (void)perimeter->run_region_group(perimeter, NULL, 0, NULL, NULL, NULL);
    (void)toolpath->resolve_pattern_id(toolpath, "rectilinear");
    (void)toolpath->generate_pattern(toolpath, 1, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    (void)tower->call_wipe_plugin(tower, 0, NULL);
}

/* In C slicing_step_t is a plain 16-bit unsigned integer, like the C++ enum base type. */
typedef char slicing_step_must_be_16_bits[sizeof(slicing_step_t) == 2 ? 1 : -1];
typedef char slicing_step_must_be_unsigned[((slicing_step_t)-1 > 0) ? 1 : -1];
/* The custom range starts at 0x8000 and lies above the highest built-in step. */
typedef char slicing_step_custom_range[
    (SLICING_STEP_CUSTOM_BEGIN == (slicing_step_t)0x8000u && SLICING_STEP_CUSTOM_END == (slicing_step_t)0xfffeu) ? 1 : -1];
typedef char slicing_step_custom_above_builtin[SLICING_STEP_CUSTOM_BEGIN > WIPE_TOWER_WIPE ? 1 : -1];

/* Returns 1 when the header declarations and step callback checks are usable from C. */
int test_plugin_api_headers_c(void)
{
    const expolygon_collection_handle *(*perimeter_slices_fn)(const layer_island_handle *) =
        layer_island_get_perimeter_slices;
    clipper_shapes_handle *(*union_fn)(storage_handle *, const clipper_shapes_handle *, clipper_fill_rule_t) =
        clipper_union;
    expolygon_collection_handle *(*union_ex_fn)(storage_handle *, const polygon_collection_handle *,
                                                clipper_fill_rule_t) = clipper_union_ex;
    expolygon_collection_handle *(*offset_ex_fn)(storage_handle *, const expolygon_collection_handle *, double,
                                                 clipper_join_type_t, double) = clipper_offset_ex;
    expolygon_collection_handle *(*offset2_ex_fn)(storage_handle *, const expolygon_collection_handle *, double,
                                                  double, clipper_join_type_t, double) = clipper_offset2_ex;
    clipper_fill_rule_t fill_rules[3] = {CLIPPER_FILL_NONZERO, CLIPPER_FILL_EVENODD, CLIPPER_FILL_POSITIVE};
    expolygon_collection_handle *(*diff_ex_fn)(storage_handle *, const expolygon_collection_handle *,
        const expolygon_collection_handle *) = clipper_diff_ex;
    expolygon_collection_handle *(*intersection_ex_fn)(storage_handle *, const expolygon_collection_handle *,
        const expolygon_collection_handle *) = clipper_intersection_ex;
    expolygon_collection_handle *(*variable_offset_fn)(storage_handle *, const expolygon_handle *,
        const float *, uint32_t) = clipper_variable_offset_inner_ex;
    polygon_collection_handle *(*expand_fn)(storage_handle *, const polygon_collection_handle *, double) = clipper_expand;
    int32_t (*slice_fn)(const triangle_mesh_handle *, const c_mesh_slicing_params *, const float *,
                        polygon_collection_handle **, uint32_t, const plugin_run_context *) =
        triangle_mesh_slice_to_polygons;
    int32_t slice_statuses[4] = {RAW_MESH_SLICE_STATUS_DONE, RAW_MESH_SLICE_STATUS_CANCELLED,
                                 RAW_MESH_SLICE_STATUS_INVALID_ARGUMENT, RAW_MESH_SLICE_STATUS_INTERNAL_ERROR};
    c_surface surface;
    plugin_run_context context;
    slicing_step_t step = STEP_SLICING;

    surface.expolygon = 0;
    surface.type = (raw_surface_type)0;
    surface.id = 0;
    context.step = step;
    /* A call with no argument at all is rejected before anything is dereferenced. */
    if (slice_fn(NULL, NULL, NULL, NULL, 0u, NULL) != RAW_MESH_SLICE_STATUS_INVALID_ARGUMENT)
        return 0;
    return perimeter_slices_fn != 0 && perimeter_slices_fn(NULL) == NULL &&
           diff_ex_fn != 0 && intersection_ex_fn != 0 && variable_offset_fn != 0 && expand_fn != 0 &&
           union_fn != 0 && union_ex_fn != 0 && offset_ex_fn != 0 && offset2_ex_fn != 0 &&
           fill_rules[0] == 0 && fill_rules[1] == 1 && fill_rules[2] == 2 &&
           slice_statuses[0] == 1 && slice_statuses[1] == 0 && slice_statuses[2] == -1 &&
           slice_statuses[3] == -2 && context.step == STEP_SLICING && surface.id == 0;
}
