///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
/* Support-step pattern callback relay. Context is the support or support-toolpath payload; its host_context
   borrows a step-owned policy exposing available(), stop_requested(print), get() and orchestrator().
   The policy preserves lazy service construction for support generation and prebuilt access for toolpaths.

   resolve refuses null ctx or ctx->host_context; generate also refuses null params or output. These checks
   precede availability and stop checks, leaving output untouched. An unavailable service is an ordinary refusal,
   also leaving output untouched. Null pattern_plugin_id is forwarded so the service can choose a fallback.
   All other pointers (print, object, layer, island, region_island, primary_region, surface and no_overlap_areas)
   are forwarded, including null values; the policy reads print for its stop check before generation.
   A stopping policy returns INVALID from resolve and empties output before generate returns zero.

   Ordinary service refusals retain the service's diagnostics and result. Exceptions from service construction,
   lookup or generation are contained with contain_pattern_callback_failure: generation output is emptied,
   failure is recorded according to that helper's rules and no exception crosses the noexcept C boundary.
   Policy availability/stop checks are noexcept; all borrowed inputs must outlive the synchronous callback. */
#ifndef steps_supportpatterncallbacks_hpp_
#define steps_supportpatterncallbacks_hpp_

#include "InfillPatternService.hpp"

namespace Slic3r::Steps {

// Relays support payload callbacks while the borrowed policy selects lazy or prebuilt service access.
template<class Context, class Policy, slicing_step_t Step>
struct SupportPatternCallbacks
{
    // Returns INVALID for null ctx/host_context or unavailable/stopping policy; forwards pattern_plugin_id, including null, and contains service exceptions.
    static infill_pattern_runtime_id resolve(const Context *ctx, const char *pattern_plugin_id) noexcept
    {
        if (ctx == nullptr || ctx->host_context == nullptr)
            return INFILL_PATTERN_RUNTIME_ID_INVALID;
        Policy &host = *static_cast<Policy *>(ctx->host_context);
        if (!host.available() || host.stop_requested(ctx->print))
            return INFILL_PATTERN_RUNTIME_ID_INVALID;
        try {
            return host.get().resolve_pattern_id(pattern_plugin_id);
        } catch (...) {
            contain_pattern_callback_failure(host.orchestrator(), Step, nullptr);
            return INFILL_PATTERN_RUNTIME_ID_INVALID;
        }
    }

    // Refuses null ctx/host_context/params/output before availability or stop checks; forwards other pointers, preserves output on ordinary input refusal, and empties it on stop or contained exceptions.
    static int32_t generate(const Context *ctx, infill_pattern_runtime_id pattern_id,
                            const layer_handle *layer, const layer_island_handle *island,
                            const layer_region_island_handle *region_island, const layer_region_handle *primary_region,
                            const surface_handle *surface, const expolygon_collection_handle *no_overlap_areas,
                            const raw_infill_pattern_params *params, extrusion_entity_handle *output) noexcept
    {
        if (ctx == nullptr || ctx->host_context == nullptr || params == nullptr || output == nullptr)
            return 0;
        Policy &host = *static_cast<Policy *>(ctx->host_context);
        if (!host.available())
            return 0;
        if (host.stop_requested(ctx->print)) {
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
            return host.get().generate_pattern(pattern_id, payload);
        } catch (...) {
            contain_pattern_callback_failure(host.orchestrator(), Step, output);
            return 0;
        }
    }
};

} // namespace Slic3r::Steps
#endif // steps_supportpatterncallbacks_hpp_
