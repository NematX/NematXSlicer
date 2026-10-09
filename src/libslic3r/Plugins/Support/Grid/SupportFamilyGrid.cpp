///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#include "SupportFamilyGrid.hpp"
#include "GridSupportGenerator.hpp"

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_support.h"
#include "libslic3r/Plugins/Support/Common/SupportPublication.hpp"
#include "libslic3r/Plugins/Support/Common/SupportSettings.hpp"

/* Grid and snug support family run. It keeps the grid and snug styles, skips an object without demand,
   generates the support slabs with generate_support() and hands them to the shared publication
   (Common/SupportPublication.hpp), which creates the auxiliary layers. */

namespace slic3r_api::Support::Grid {
// Construct the provider once; the orchestrator retains its registration but does not own the static instance.
SupportFamilyGrid &SupportFamilyGrid::instance(orchestrator_handle *orchestrator)
{
    static SupportFamilyGrid provider(orchestrator);
    return provider;
}

// Declare the support settings shared with the toolpaths (heights, gaps, flows, extruders and the generator's own options); a null buffer returns the required count.
int32_t SupportFamilyGrid::used_config_keys(raw_used_config_key *keys) const noexcept
{
    return Common::append_used_config_keys(keys, 0, Common::support_geometry_keys());
}

// Skip unsupported styles or empty demand, warn once for a requested raft, and publish geometric support; host cancellation and calculation errors are reported by PluginBase.
void SupportFamilyGrid::run_impl(const plugin_run_context *context) const
{
    throw_if_cancelled(context);
    const run_ctx_generate_support *payload = plugin_ctx_as_generate_support(context);
    if (!payload || !payload->object || !payload->print) return;
    const Print print(payload->print);
    const Object object(payload->object);
    const Config config = object.config();
    const Common::SupportStyle style = Common::support_style(config);
    if (style != Common::SupportStyle::Grid && style != Common::SupportStyle::Snug) {
        progress().finish_run();
        return;
    }
    bool has_demand = false;
    for (uint32_t layer_idx = 0; layer_idx < object.layer_count() && !has_demand; ++layer_idx) {
        const Layer layer = object.layer(layer_idx);
        for (uint32_t island = 0; island < layer.island_count(); ++island)
            has_demand = has_demand || !layer.island(island).support_demand().empty();
    }
    if (!has_demand) {
        progress().finish_run();
        return;
    }
    if (config.get("raft_layers").get_int() > 0 && context->report_warning)
        context->report_warning(context->host_context, "Grid support is generated without a raft; raft layers are not implemented by this family.");
    const Common::SupportParams params(print, object);
    Common::GeneratedSupport support(context->plugin_storage);
    generate_support(context, print, object, params, support);
    Common::publish(context, object, support, m_orchestrator);
    progress().finish_run();
}

// Register the static provider with the host, which manages activation and shared scratch storage.
void register_support_family_grid_plugin(orchestrator_handle *orchestrator)
{
    orchestrator_register_plugin(orchestrator, SupportFamilyGrid::instance(orchestrator).c_instance());
}
}
