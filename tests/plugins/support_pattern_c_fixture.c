///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#if defined(_MSC_VER)
#pragma warning(error : 4133 4115)
#elif defined(__GNUC__)
#pragma GCC diagnostic error "-Wincompatible-pointer-types"
#endif
#include <stddef.h>
#include <stdint.h>
#include "libslic3r/Api/plugin/c/steps/slic3r_step_support.h"
#ifdef __cplusplus
#error This fixture must be compiled as C.
#endif

/* Compile the STEP_SUPPORT payload header as C: the callback types name
   struct run_ctx_generate_support, which must be the struct defined later in the
   header and not a struct local to the prototype. Without the forward
   declaration the assignments and calls below are incompatible pointer types,
   and the compilers are told to treat that diagnostic as an error.

   All dependencies, including the common step and infill headers, are real C
   headers here; no include guards or opaque replacement types hide them. */

/* Callbacks written against the typedefs, as a C plugin would. */
static infill_pattern_runtime_id c_resolve(const run_ctx_generate_support *ctx, const char *pattern_plugin_id)
{
    (void)ctx;
    return pattern_plugin_id != 0 ? (infill_pattern_runtime_id)5 : INFILL_PATTERN_RUNTIME_ID_INVALID;
}

static int32_t c_generate(const run_ctx_generate_support *ctx,
                          infill_pattern_runtime_id pattern_id,
                          const layer_handle *layer,
                          const layer_island_handle *island,
                          const layer_region_island_handle *region_island,
                          const layer_region_handle *primary_region,
                          const surface_handle *surface,
                          const expolygon_collection_handle *no_overlap_areas,
                          const raw_infill_pattern_params *params,
                          extrusion_entity_handle *output)
{
    (void)ctx; (void)layer; (void)island; (void)region_island; (void)primary_region;
    (void)surface; (void)no_overlap_areas; (void)params; (void)output;
    return (int32_t)pattern_id;
}

/* Assign the C callbacks to a payload, then call the callbacks of a payload given by the caller.
   Returns 1000 * generate result + resolved id. */
int test_support_pattern_callbacks_c(const run_ctx_generate_support *payload)
{
    run_ctx_generate_support local;
    support_resolve_pattern_id_fn resolve = c_resolve;
    support_generate_pattern_fn generate = c_generate;
    local.print = 0;
    local.object = 0;
    local.host_context = 0;
    local.resolve_pattern_id = resolve;
    local.generate_pattern = generate;
    if (local.resolve_pattern_id(&local, "x") != 5 || local.generate_pattern(&local, 3, 0, 0, 0, 0, 0, 0, 0, 0) != 3)
        return -1;

    infill_pattern_runtime_id id = payload->resolve_pattern_id(payload, "rectilinear");
    return 1000 * payload->generate_pattern(payload, id, 0, 0, 0, 0, 0, 0, 0, 0) + (int)id;
}
