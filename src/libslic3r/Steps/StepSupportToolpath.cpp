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
#include "StepPipeline.hpp"
#include "StepRunner.hpp"

/* Each object selects one algorithm per exclusive role group through its own
   configuration. The dispatcher runs the selected algorithms by priority, each
   on the objects that selected it. Each callback processes one object's support
   surfaces and adds extrusions through the data tree. The dispatcher does not
   store geometry or plugin output.

   Algorithms may fill surfaces with INFILL_PATTERN plugins through the payload
   callbacks. One InfillPatternService is built before the first algorithm runs
   and shared by every object run, which may be concurrent: the service is
   read-only after construction (see InfillPatternService.hpp).

   Errors: the callbacks are called through the C ABI, also by Python
   algorithms through ctypes, which cannot catch a C++ exception, so they catch
   everything. An ordinary refusal (unknown runtime id, pattern needing the
   STEP_INFILL initialization pass) is a message from the service and the step
   goes on. A failure (a pattern reports an error, or any other exception)
   empties the output, returns the failure value and requests the plugin
   cancellation (a PluginStopped adds no message, any other exception adds an
   error message), so that the runner stops the step when the
   algorithm returns (contain_pattern_callback_failure). A cancellation of the
   print is not recorded: the runner rethrows it. Once the plugin cancellation
   is requested or the print is cancelled, the callbacks return the failure
   value at once, without a new message. */

namespace Slic3r::Steps::StepSupportToolpath {
namespace {

// What the payload's host_context points to: the pattern service (null when no algorithm is active, then
// every callback refuses) and the orchestrator that receives the contained failures.
struct HostContext
{
    InfillPatternService *service;
    Orchestrator &orchestrator;
};

// Number of auxiliary fill surfaces of the print, advertised to the pattern
// plugins as their maximum run count.
size_t count_auxiliary_surfaces(const Print &print);
// True when the plugin cancellation is requested or the print is cancelled: a pattern callback then
// fails at once instead of working for a step that is stopping.
bool step_is_stopping(const HostContext &host, const run_ctx_support_toolpath &ctx) noexcept;
// Payload callback: resolve a pattern plugin id through the step's service. Never throws: a failure is
// contained and gives INFILL_PATTERN_RUNTIME_ID_INVALID.
infill_pattern_runtime_id resolve_pattern_id_callback(const run_ctx_support_toolpath *ctx,
                                                      const char *pattern_plugin_id) noexcept;
// Payload callback: fill one surface through the step's service. Never throws: a failure empties output,
// is contained, and returns 0.
int32_t generate_pattern_callback(const run_ctx_support_toolpath *ctx,
                                  infill_pattern_runtime_id pattern_id,
                                  const layer_handle *layer,
                                  const layer_island_handle *island,
                                  const layer_region_island_handle *region_island,
                                  const layer_region_handle *primary_region,
                                  const surface_handle *surface,
                                  const expolygon_collection_handle *no_overlap_areas,
                                  const raw_infill_pattern_params *params,
                                  extrusion_entity_handle *output) noexcept;

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

// The stop flags are two atomic reads; the print is the one of the payload (a null print only checks the plugin flag).
bool step_is_stopping(const HostContext &host, const run_ctx_support_toolpath &ctx) noexcept
{
    const Print *print = reinterpret_cast<const Print *>(ctx.print);
    return host.orchestrator.is_plugin_cancelled() || (print != nullptr && print->canceled());
}

// A null context, host context or service is an ordinary refusal. Only C++ code runs inside the try (the
// service lookup and its warning message): a failure there is contained after the catch and gives INVALID.
infill_pattern_runtime_id resolve_pattern_id_callback(const run_ctx_support_toolpath *ctx,
                                                      const char *pattern_plugin_id) noexcept
{
    if (ctx == nullptr || ctx->host_context == nullptr)
        return INFILL_PATTERN_RUNTIME_ID_INVALID;
    const HostContext &host = *static_cast<const HostContext *>(ctx->host_context);
    if (host.service == nullptr || step_is_stopping(host, *ctx))
        return INFILL_PATTERN_RUNTIME_ID_INVALID;
    try {
        return host.service->resolve_pattern_id(pattern_plugin_id);
    } catch (...) {
        contain_pattern_callback_failure(host.orchestrator, STEP_SUPPORT_TOOLPATH, nullptr);
        return INFILL_PATTERN_RUNTIME_ID_INVALID;
    }
}

// Null arguments are refused with 0 and leave output untouched. A stopping step empties output and
// returns 0 without a message. Otherwise the payload of the pattern plugin is copied from the algorithm's
// arguments and the service runs it; the ordinary refusals (0 from the service) keep their service message
// and let the step go on, while an exception (pattern error, cancellation of the print or of the plugins)
// is contained: output is emptied and the plugin cancellation is requested. A PluginStopped adds no
// diagnostic (a known cause already has its own message; an Unknown cause has none, and the final step error
// says a stop was requested); any other exception is recorded as a host failure, unless the print was
// cancelled. Only C++ code runs inside the try.
int32_t generate_pattern_callback(const run_ctx_support_toolpath *ctx,
                                  infill_pattern_runtime_id pattern_id,
                                  const layer_handle *layer,
                                  const layer_island_handle *island,
                                  const layer_region_island_handle *region_island,
                                  const layer_region_handle *primary_region,
                                  const surface_handle *surface,
                                  const expolygon_collection_handle *no_overlap_areas,
                                  const raw_infill_pattern_params *params,
                                  extrusion_entity_handle *output) noexcept
{
    if (ctx == nullptr || ctx->host_context == nullptr || params == nullptr || output == nullptr)
        return 0;
    const HostContext &host = *static_cast<const HostContext *>(ctx->host_context);
    if (host.service == nullptr)
        return 0;
    if (step_is_stopping(host, *ctx)) {
        empty_output(output);
        return 0;
    }

    try {
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
        return host.service->generate_pattern(pattern_id, payload);
    } catch (...) {
        contain_pattern_callback_failure(host.orchestrator, STEP_SUPPORT_TOOLPATH, output);
        return 0;
    }
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

// Run each selected toolpath algorithm, in priority order, on the objects whose own selector chose it.
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
    HostContext host_context { patterns.has_value() ? &*patterns : nullptr, orchestrator };
    for (const ObjectStepPlugin &entry : selected_plugins_per_object(orchestrator, STEP_SUPPORT_TOOLPATH, print)) {
        Detail::run_object_step_plugin(orchestrator,
                                       print,
                                       STEP_SUPPORT_TOOLPATH,
                                       *entry.plugin,
                                       entry.object_indices,
                                       print.objects().size(),
                                       [&print, &host_context](const size_t object_idx) {
            run_ctx_support_toolpath context = {};
            context.print = reinterpret_cast<const print_handle *>(&print);
            context.object = reinterpret_cast<const object_handle *>(&print.object(object_idx));
            context.host_context = &host_context;
            context.resolve_pattern_id = &resolve_pattern_id_callback;
            context.generate_pattern = &generate_pattern_callback;
            return context;
        });
        print.pipeline_progress().complete_plugin(STEP_SUPPORT_TOOLPATH, entry.plugin->get_id());
    }
    Detail::validate_or_report(validate_post, print, "Support-toolpath post-step validation");
}

} // namespace Slic3r::Steps::StepSupportToolpath
