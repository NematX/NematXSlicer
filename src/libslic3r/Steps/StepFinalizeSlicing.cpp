///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
/* Run finalization plugins after support and adhesion paths exist, before ordering
   builds the PrintingPlan. Plugins can inspect completed support state and make
   private copies of existing model-layer infill, ironing or gap-fill roots.

   run_step() dispatches one plugin's object runs and joins them before dispatching
   the next plugin. Each payload borrows the print, its object and that plugin's
   storage. Stable context records remain alive until all object callbacks return.

       run_step()
           |-- selected_or_active_plugins_for_step()
           `-- Detail::run_object_step_plugin() [joined object dispatch]

   A plugin may call payload.replace_infill_buckets(), which points to the host
   callback below. This is not an unconditional direct call from run_step().

       replace_infill() [host publication callback]
           |-- stored_extrusions() [check donors belong to this run's storage]
           |-- throw_if_canceled() [after complete batch validation]
           `-- ExtrusionEntity::swap_content() [nonthrowing exchanges]

   Validate all destinations, unique donors and cancellation before any exchange.
   Rejection leaves both sides unchanged; caught failures request plugin stop and
   never cross C. No allocation, cancellation check or plugin call occurs between
   swaps. Both root identities stay unchanged and donors receive the old contents;
   discard earlier views of either root's descendants after success. Atomicity is
   limited to this batch, not a rollback of earlier plugins or later work.

   clear()/prepare() retain no state; validation accepts the incoming tree.
   Different objects may run concurrently, but a run may publish only its own
   object's roots and must exclude concurrent changes to its storage/destinations.
*/
#include "StepFinalizeSlicing.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <map>
#include <set>
#include <stdexcept>
#include <vector>
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_finalize_slicing.h"
#include "libslic3r/Api/plugin/cpp/properties/BridgeProperties.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "StepRunner.hpp"

namespace Slic3r::Steps::StepFinalizeSlicing {
namespace {
// One joined object's callback state; pointers to these records never escape its run.
struct FinalHostContext {
    Orchestrator &orchestrator;
    Print &print;
    PrintObject &object;
    PluginStorage &storage;
};
// Borrow the payload/array for one synchronous publication. Validate unique live
// destinations and storage-owned donors, then return 1 after nonthrowing swaps.
// Refusal/cancellation returns 0 unchanged; caught failures request stop, never cross C.
int32_t replace_infill(const run_ctx_finalize_slicing *ctx, storage_handle *storage,
    const raw_final_infill_replacement *entries, uint32_t count) noexcept;
int32_t replace_infill(const run_ctx_finalize_slicing *ctx, storage_handle *storage,
    const raw_final_infill_replacement *entries, uint32_t count) noexcept
{
    if (ctx == nullptr || ctx->host_context == nullptr || (count > 0 && entries == nullptr))
        return 0;
    FinalHostContext &host = *static_cast<FinalHostContext *>(ctx->host_context);
    try {
        if (reinterpret_cast<PluginStorage *>(storage) != &host.storage)
            return 0;
        std::set<const ExtrusionEntity *> allowed, destinations, donors;
        const std::vector<const ExtrusionEntity *> stored = host.storage.stored_extrusions();
        const std::set<const ExtrusionEntity *> owned(stored.begin(), stored.end());
        // Only existing object infill roots may receive content; support and
        // perimeter roots are inputs to the finalizer, not publication targets.
        for (Layer &layer : host.object.layers())
            for (LayerSliceIsland &island : layer.islands())
                for (LayerRegionIsland &region : island.regions_islands())
                    for (const auto &[role, root] : region.extrusion_buckets())
                        if (role.is_infill() || role == LayerRegionIsland::GAP_FILLS)
                            allowed.insert(&root);
        std::vector<std::pair<ExtrusionEntity *, ExtrusionEntity *>> swaps;
        swaps.reserve(count);
        for (uint32_t idx = 0; idx < count; ++idx) {
            const ExtrusionEntity *destination = reinterpret_cast<const ExtrusionEntity *>(entries[idx].destination);
            ExtrusionEntity *donor = reinterpret_cast<ExtrusionEntity *>(entries[idx].prepared);
            if (allowed.count(destination) == 0 || donor == nullptr || owned.count(donor) == 0 ||
                !destinations.insert(destination).second || !donors.insert(donor).second || allowed.count(donor) != 0)
                return 0;
            swaps.emplace_back(const_cast<ExtrusionEntity *>(destination), donor);
        }
        host.print.throw_if_canceled();
        if (host.orchestrator.is_plugin_cancelled())
            return 0;
        // Allocation, validation and cancellation are complete: this loop cannot
        // expose a partially exchanged batch through a failed operation.
        for (const std::pair<ExtrusionEntity *, ExtrusionEntity *> &entry : swaps)
            entry.first->swap_content(*entry.second);
        return 1;
    } catch (...) {
        Detail::contain_callback_failure(host.orchestrator, STEP_FINALIZE_SLICING);
        return 0;
    }
}
}
void clear(Print &) {}
void prepare(Print &) {}
bool validate_pre(const Print &, std::string *) { return true; }
bool validate_post(const Print &, std::string *) { return true; }
void run_step(Orchestrator &orchestrator, Print &print)
{
    for (Plugin *plugin : selected_or_active_plugins_for_step(orchestrator, STEP_FINALIZE_SLICING, &print.full_print_config())) {
        // Finish building stable callback records before any object receives
        // their addresses; the dispatcher joins all uses before destruction.
        std::vector<FinalHostContext> contexts;
        contexts.reserve(print.objects().size());
        for (PrintObject &object : print.objects())
            contexts.push_back({orchestrator, print, object, orchestrator.plugin_storage()[plugin]});
        Detail::run_object_step_plugin(orchestrator, print, STEP_FINALIZE_SLICING, *plugin,
            print.objects().size(), [&print, &contexts](size_t idx) {
                run_ctx_finalize_slicing payload = {};
                payload.print = reinterpret_cast<const print_handle *>(&print);
                payload.object = reinterpret_cast<const object_handle *>(&print.object(idx));
                payload.host_context = &contexts[idx];
                const slic3r_api::SupportPipelineInfo *state = print.object(idx).get_property<slic3r_api::SupportPipelineInfo>();
                payload.support_demand_computed = state == nullptr ? 0 : state->demand_computed;
                payload.support_material_generated = state == nullptr ? 0 : state->material_generated;
                payload.get_requested_support_areas = &layer_island_get_support_demand;
                payload.replace_infill_buckets = &replace_infill;
                return payload;
            });
        print.pipeline_progress().complete_plugin(STEP_FINALIZE_SLICING, plugin->get_id());
    }
}
}
