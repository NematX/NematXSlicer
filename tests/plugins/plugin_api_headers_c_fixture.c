///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
/* Compile slic3r_volume.h, slic3r_plugin_run_context.h and slic3r_clipper.h as C with their real
   dependencies (slic3r_utils.h, slic3r_data_tree.h, slic3r_slicing_step.h, ...): nothing is
   neutralised. Function pointers declared with the full signature and initialised without a cast
   make the compiler type-check the parameters and return type of the declarations; the pragmas below
   turn an incompatible pointer assignment into an error (MSVC only warns by default). Compile-time
   array checks verify the type and value of the slicing step constants. */
#if defined(_MSC_VER)
#pragma warning(error : 4047 4113 4133)
#elif defined(__GNUC__)
#pragma GCC diagnostic error "-Wincompatible-pointer-types"
#endif
#include "libslic3r/Api/plugin/c/slic3r_volume.h"
#include "libslic3r/Api/plugin/c/slic3r_plugin_run_context.h"
#include "libslic3r/Api/plugin/c/slic3r_clipper.h"
#ifdef __cplusplus
#error This fixture must be compiled as C.
#endif

/* In C slicing_step_t is a plain 16-bit unsigned integer, like the C++ enum base type. */
typedef char slicing_step_must_be_16_bits[sizeof(slicing_step_t) == 2 ? 1 : -1];
typedef char slicing_step_must_be_unsigned[((slicing_step_t)-1 > 0) ? 1 : -1];
/* The custom range starts at 0x8000 and lies above the highest built-in step. */
typedef char slicing_step_custom_range[
    (SLICING_STEP_CUSTOM_BEGIN == (slicing_step_t)0x8000u && SLICING_STEP_CUSTOM_END == (slicing_step_t)0xfffeu) ? 1 : -1];
typedef char slicing_step_custom_above_builtin[SLICING_STEP_CUSTOM_BEGIN > WIPE_TOWER_WIPE ? 1 : -1];

/* Returns 1 when the declarations of the three headers are usable from C. */
int test_plugin_api_headers_c(void)
{
    clipper_shapes_handle *(*union_fn)(storage_handle *, const clipper_shapes_handle *, clipper_fill_rule_t) =
        clipper_union;
    expolygon_collection_handle *(*union_ex_fn)(storage_handle *, const polygon_collection_handle *,
                                                clipper_fill_rule_t) = clipper_union_ex;
    expolygon_collection_handle *(*offset_ex_fn)(storage_handle *, const expolygon_collection_handle *, double,
                                                 clipper_join_type_t, double) = clipper_offset_ex;
    expolygon_collection_handle *(*offset2_ex_fn)(storage_handle *, const expolygon_collection_handle *, double,
                                                  double, clipper_join_type_t, double) = clipper_offset2_ex;
    clipper_fill_rule_t fill_rules[3] = {CLIPPER_FILL_NONZERO, CLIPPER_FILL_EVENODD, CLIPPER_FILL_POSITIVE};
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
    return union_fn != 0 && union_ex_fn != 0 && offset_ex_fn != 0 && offset2_ex_fn != 0 &&
           fill_rules[0] == 0 && fill_rules[1] == 1 && fill_rules[2] == 2 &&
           slice_statuses[0] == 1 && slice_statuses[1] == 0 && slice_statuses[2] == -1 &&
           slice_statuses[3] == -2 && context.step == STEP_SLICING && surface.id == 0;
}
