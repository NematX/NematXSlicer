///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#include "SupportFamilyTree.hpp"

#include <iterator>

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_support.h"
#include "libslic3r/Plugins/Support/Common/SupportPublication.hpp"
#include "libslic3r/Plugins/Support/Common/SupportSettings.hpp"
#include "libslic3r/Plugins/Support/Tree/TreeSupportGenerator.hpp"

/* Tree and organic support family run. It skips objects whose style is neither tree nor organic, or that have no demand,
   and generates the branch slabs with generate_tree_support(). Organic objects publish
   their slabs as branch surfaces; tree objects publish sparse base surfaces. The publication
   (Common/SupportPublication.hpp) creates the auxiliary layers. */

namespace slic3r_api::Support::Tree {
namespace {
// Tree branch settings read by the routing and the slab placement, beyond the geometry keys of SupportParams. "resolution" is the
// print slicing resolution that cleans the support blockers sliced by TreeVolumes.
const raw_used_config_key k_tree_keys[] = {
    {"resolution", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_tree_angle", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_tree_angle_slow", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_tree_branch_diameter", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_tree_branch_diameter_angle", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_tree_branch_distance", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_tree_max_overhang", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_tree_tip_diameter", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_tree_top_rate", RAW_CO_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
};
} // namespace

// Construct the provider once; the orchestrator retains its registration but does not own the static instance.
SupportFamilyTree &SupportFamilyTree::instance(orchestrator_handle *orchestrator)
{
    static SupportFamilyTree provider(orchestrator);
    return provider;
}

// Declare the geometry keys of SupportParams followed by the tree branch settings; a null buffer returns the required count.
int32_t SupportFamilyTree::used_config_keys(raw_used_config_key *keys) const noexcept
{
    const int32_t count = Common::append_used_config_keys(keys, 0, Common::support_geometry_keys());
    return Common::append_used_config_keys(keys, count, Common::UsedKeyList{k_tree_keys, std::size(k_tree_keys)});
}

// Skip other styles and empty demand, and publish the generated branch slabs; host cancellation and calculation errors are reported by PluginBase.
void SupportFamilyTree::run_impl(const plugin_run_context *context) const
{
    throw_if_cancelled(context);
    const run_ctx_generate_support *payload = plugin_ctx_as_generate_support(context);
    if (!payload || !payload->object || !payload->print) return;
    const Object object(payload->object);
    const Config config = object.config();
    const Common::SupportStyle style = Common::support_style(config);
    if (style != Common::SupportStyle::Tree && style != Common::SupportStyle::Organic) {
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
    Common::GeneratedSupport support(context->plugin_storage);
    Slic3r::TreeFamily::generate_tree_support(context, Print(payload->print), object, style == Common::SupportStyle::Organic, support);
    Common::publish(context, object, support, m_orchestrator);
    progress().finish_run();
}

// Register the static provider with the host, which manages activation and shared scratch storage.
void register_support_family_tree_plugin(orchestrator_handle *orchestrator)
{
    orchestrator_register_plugin(orchestrator, SupportFamilyTree::instance(orchestrator).c_instance());
}
} // namespace slic3r_api::Support::Tree
