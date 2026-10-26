///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "StepSupportToolpath.hpp"

#include <optional>

#include "libslic3r/Api/plugin/c/steps/slic3r_step_support_toolpath.h"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/PrintObject.hpp"
#include "InfillPatternService.hpp"
#include "StepRunner.hpp"

/* The object-step runner selects one algorithm per exclusive role group and
   orders the selected algorithms by priority. Each callback processes one
   object's support surfaces and adds extrusions through the data tree.
   The dispatcher does not store geometry or plugin output.

   Algorithms may fill surfaces with INFILL_PATTERN plugins through the payload
   callbacks. One InfillPatternService is built before the first algorithm runs
   and shared by every object run, which may be concurrent: the service is
   read-only after construction (see InfillPatternService.hpp). */

namespace Slic3r::Steps::StepSupportToolpath {
namespace {

// Number of auxiliary fill surfaces of the print, advertised to the pattern
// plugins as their maximum run count.
size_t count_auxiliary_surfaces(const Print &print);
// Payload callback: resolve a pattern plugin id through the step's service.
infill_pattern_runtime_id resolve_pattern_id_callback(const run_ctx_support_toolpath *ctx,
                                                      const char *pattern_plugin_id);
// Payload callback: fill one surface through the step's service.
int32_t generate_pattern_callback(const run_ctx_support_toolpath *ctx,
                                  infill_pattern_runtime_id pattern_id,
                                  const layer_handle *layer,
                                  const layer_island_handle *island,
                                  const layer_region_island_handle *region_island,
                                  const layer_region_handle *primary_region,
                                  const surface_handle *surface,
                                  const expolygon_collection_handle *no_overlap_areas,
                                  const raw_infill_pattern_params *params,
                                  extrusion_entity_handle *output);

size_t count_auxiliary_surfaces(const Print &print)
{
    size_t count = 0;
    for (const PrintObject &object : print.objects())
        for (const Layer &layer : object.auxiliary_layers())
            for (const LayerSliceIsland &island : layer.islands())
                for (const LayerRegionIsland &region_island : island.regions_islands())
                    count += region_island.fill_surfaces().size();
    return count;
}

infill_pattern_runtime_id resolve_pattern_id_callback(const run_ctx_support_toolpath *ctx,
                                                      const char *pattern_plugin_id)
{
    if (ctx == nullptr || ctx->host_context == nullptr)
        return INFILL_PATTERN_RUNTIME_ID_INVALID;
    return static_cast<const InfillPatternService *>(ctx->host_context)->resolve_pattern_id(pattern_plugin_id);
}

int32_t generate_pattern_callback(const run_ctx_support_toolpath *ctx,
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
    if (ctx == nullptr || ctx->host_context == nullptr || params == nullptr || output == nullptr)
        return 0;

    run_ctx_infill_pattern payload = {};
    payload.print = ctx->print;
    payload.object = ctx->object;
    payload.layer = layer;
    payload.island = island;
    payload.region_island = region_island;
    payload.primary_region = primary_region;
    payload.surface = surface;
    payload.no_overlap_areas = no_overlap_areas;
    payload.params = params;
    payload.output = output;
    return static_cast<InfillPatternService *>(ctx->host_context)->generate_pattern(pattern_id, payload);
}

} // namespace

// Nothing to clear: the extrusions live in the support auxiliary layers, which a Slicing replay destroys.
void clear(Print &) {}

// Nothing to prepare: the algorithms read the surfaces left by the support families.
void prepare(Print &) {}

// No common input check: input validation belongs to each algorithm.
bool validate_pre(const Print &, std::string &)
{
    return true;
}

// No common output check: output validation belongs to each algorithm.
bool validate_post(const Print &, std::string &)
{
    return true;
}

// Run the selected toolpath algorithms on every object.
void run_step(Orchestrator &orchestrator, Print &print)
{
    Detail::validate_or_report(validate_pre, print, "Support-toolpath pre-step validation");
    // Built before any object run so concurrent calls never create pattern
    // contexts, storages or error slots. Patterns needing the STEP_INFILL
    // initialization pass are refused because this step has none. The service
    // sets up the patterns, so it is skipped when no algorithm can call them.
    std::optional<InfillPatternService> patterns;
    if (!orchestrator.get_active_plugins_for_step(STEP_SUPPORT_TOOLPATH).empty())
        patterns.emplace(orchestrator, print, count_auxiliary_surfaces(print),
                         InfillPatternService::SetupPatterns::Refuse);
    // Without an active algorithm the runner never builds a payload; the null
    // check keeps the factory safe even if that selection rule changes.
    InfillPatternService *service = patterns.has_value() ? &*patterns : nullptr;
    Detail::run_object_step_plugins(orchestrator,
                                    print,
                                    STEP_SUPPORT_TOOLPATH,
                                    &print.full_print_config(),
                                    print.objects().size(),
                                    [&print, service](const size_t object_idx) {
        run_ctx_support_toolpath context = {};
        context.print = reinterpret_cast<const print_handle *>(&print);
        context.object = reinterpret_cast<const object_handle *>(&print.object(object_idx));
        context.host_context = service;
        context.resolve_pattern_id = &resolve_pattern_id_callback;
        context.generate_pattern = &generate_pattern_callback;
        return context;
    });
    Detail::validate_or_report(validate_post, print, "Support-toolpath post-step validation");
}

} // namespace Slic3r::Steps::StepSupportToolpath
