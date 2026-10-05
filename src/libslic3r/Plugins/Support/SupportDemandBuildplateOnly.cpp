///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.

#include "SupportDemandBuildplateOnly.hpp"

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_support_demand.h"
#include "libslic3r/Api/plugin/cpp/Views.hpp"

/* Buildplate-only support keeps demand only where a vertical column can reach
   the bed without crossing the object. For layer i, covered is the union
   of object slices from layers 0..i-1, each enlarged by 0.01 mm before union.
   The accumulated union is not enlarged again. Each island's demand is then
   replaced by demand minus covered through support_demand_get_island_mutable.
   This removes overhang, painted and modifier demand alike; unlike the legacy
   support generator, it does not keep enforced support above the object.
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

// Declare the existing boolean key; its definition specifies neither container nor preset metadata. Report one entry when no array is supplied.
int32_t SupportDemandBuildplateOnly::used_config_keys(raw_used_config_key *keys) const noexcept
{
    if (keys != nullptr)
        keys[0] = {"support_material_buildplate_only", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE};
    return 1;
}

// Subtract expanded lower slices from every island's demand; disabled or missing object contexts are skipped and cancellation stops the filter.
void SupportDemandBuildplateOnly::run_impl(const plugin_run_context *run_ctx) const
{
    throw_if_cancelled(run_ctx);
    const run_ctx_support_demand *ctx = plugin_ctx_as_support_demand(run_ctx);
    if (ctx == nullptr || ctx->object == nullptr)
        return;

    const Object object(ctx->object);
    if (!object.config().get("support_material_buildplate_only").get_bool()) {
        progress().finish_run();
        return;
    }

    const ClipperContext clipper(run_ctx->plugin_storage);
    ClipperOperand covered = clipper();
    for (uint32_t layer_idx = 0; layer_idx < object.layer_count(); ++layer_idx) {
        throw_if_cancelled(run_ctx);
        if (layer_idx > 0)
            covered = clipper_union2(covered, clipper_offset(clipper(object.layer(layer_idx - 1).slices()), scale_d(0.01)));

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
