///|/ Copyright (c) SuperSlicer 2026 Durand R?mi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "SupportDemandStep.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/internal/LayerIslandAccess.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"

namespace Slic3r::ApiHost::Steps {

void clear_support_demand(Print &print)
{
    for (PrintObject &object : print.objects())
        for (Layer &layer : object.layers())
            for (LayerSliceIsland &island : layer.islands())
                ApiInternal::LayerIslandAccess::support_demand_mutable(island).clear();
}

} // namespace Slic3r::ApiHost::Steps

extern "C" {

expolygon_collection_handle *support_demand_get_island_mutable(const plugin_run_context *run_ctx,
                                                               const layer_island_handle *island)
{
    if (run_ctx == nullptr || island == nullptr)
        return nullptr;
    // The running pipeline step, not run_ctx->step, decides: service plugins
    // (infill patterns, perimeter modules) run inside a pipeline step under
    // their own step id. As for every data-tree edit, no running step (host
    // tools, tests calling a step directly) means no restriction.
    const plugin_host_context *host_context = run_ctx->host_context;
    const Slic3r::Print *print = host_context == nullptr ? nullptr : host_context->print;
    const slicing_step_t running = print == nullptr ? STEP_NONE : print->running_step();
    const bool allowed = running == STEP_NONE ||
        (!Slic3r::Steps::executes_before(running, STEP_SUPPORT_DEMAND) &&
         !Slic3r::Steps::executes_before(STEP_SUPPORT, running));
    if (!allowed) {
        if (run_ctx->report_error != nullptr)
            run_ctx->report_error(run_ctx->host_context,
                                  "support_demand_get_island_mutable: the support demand is editable only from "
                                  "STEP_SUPPORT_DEMAND through STEP_SUPPORT.");
        return nullptr;
    }
    // The traversal hands out const island handles; this call is the explicit
    // permission to edit the demand of one island.
    Slic3r::LayerSliceIsland &mutable_island =
        const_cast<Slic3r::LayerSliceIsland &>(*reinterpret_cast<const Slic3r::LayerSliceIsland *>(island));
    return reinterpret_cast<expolygon_collection_handle *>(
        &Slic3r::ApiInternal::LayerIslandAccess::support_demand_mutable(mutable_island));
}

} // extern "C"
