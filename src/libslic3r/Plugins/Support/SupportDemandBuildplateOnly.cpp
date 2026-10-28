///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.

#include "SupportDemandBuildplateOnly.hpp"

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_support_demand.h"
#include "libslic3r/Api/plugin/cpp/Views.hpp"
#include "libslic3r/Plugins/Support/Common/SupportSettings.hpp"
#include "libslic3r/Plugins/Support/Common/SupportDemand.hpp"

/* Buildplate-only support keeps demand only where a vertical column can reach
   the bed without crossing the object. For layer i, covered is the union
   of object slices from layers 0..i-1, each enlarged by 0.01 mm before union.
   The accumulated union is not enlarged again. Each island's demand is then
   replaced by demand minus covered through support_demand_get_island_mutable.
   This removes overhang, painted and modifier demand alike; unlike the legacy
   support generator, it does not keep enforced support above the object.
   The filter applies to grid and snug objects only: the tree and organic
   families route their branches to the bed themselves and keep their demand.
   With support_material_buildplate_only disabled, the filter does nothing.
   The plugin API supplies all geometry and settings. Temporary results use
   plugin storage and the object keeps the published demand. */

namespace slic3r_api::Support::SupportDemandBuildplateOnlyPlugin {

// Create the shared provider on its first registration and reuse it thereafter.
SupportDemandBuildplateOnly &SupportDemandBuildplateOnly::instance(orchestrator_handle *orch)
{
    static SupportDemandBuildplateOnly provider(orch);
    return provider;
}

// Declare the buildplate-only switch and the support style that selects the objects it applies to. Report two entries when no array is supplied.
int32_t SupportDemandBuildplateOnly::used_config_keys(raw_used_config_key *keys) const noexcept
{
    if (keys != nullptr) {
        keys[0] = {"support_material_buildplate_only", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE};
        keys[1] = {"support_material_style", RAW_CO_ENUM, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE};
    }
    return 2;
}

// Accumulates borrowed lower slices through Common and subtracts coverage from grid/snug island demand; the context supplies mutable demand, other styles/disabled filters/missing payloads are skipped, and cancellation/geometry/allocation failures propagate.
void SupportDemandBuildplateOnly::run_impl(const plugin_run_context *run_ctx) const
{
    throw_if_cancelled(run_ctx);
    const run_ctx_support_demand *ctx = plugin_ctx_as_support_demand(run_ctx);
    if (ctx == nullptr || ctx->object == nullptr)
        return;

    const Object object(ctx->object);
    const Config config = object.config();
    const Common::SupportStyle style = Common::support_style(config);
    const bool grid_or_snug = style == Common::SupportStyle::Grid || style == Common::SupportStyle::Snug;
    if (!grid_or_snug || !config.get("support_material_buildplate_only").get_bool()) {
        progress().finish_run();
        return;
    }

    const ClipperContext clipper(run_ctx->plugin_storage);
    ClipperOperand covered = clipper();
    for (uint32_t layer_idx = 0; layer_idx < object.layer_count(); ++layer_idx) {
        throw_if_cancelled(run_ctx);
        if (layer_idx > 0)
            covered = Common::accumulate_lower_slices(clipper, covered, object.layer(layer_idx - 1));

        const Layer layer = object.layer(layer_idx);
        for (uint32_t island_idx = 0; island_idx < layer.island_count(); ++island_idx) {
            throw_if_cancelled(run_ctx);
            const LayerIsland island = layer.island(island_idx);
            if (island.support_demand().empty() || covered.empty())
                continue;
            StoredExPolygonCollection remaining =
                clipper_diff(clipper(island.support_demand()), covered).to_expolygon_collection();
            expolygons_move(support_demand_get_island_mutable(run_ctx, island.handle()), remaining.mutable_handle());
        }
    }
    progress().finish_run();
}

// Add the shared provider to the orchestrator; plugin storage and activation remain managed by the host.
void register_support_demand_buildplate_only_plugin(orchestrator_handle *orch)
{
    orchestrator_register_plugin(orch, SupportDemandBuildplateOnly::instance(orch).c_instance());
}

} // namespace slic3r_api::Support::SupportDemandBuildplateOnlyPlugin
