///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#include "SupportToolpathBranch.hpp"
#include "BranchSheathPaths.hpp"

#include <iterator>

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Plugins/Support/Common/SupportSettings.hpp"

/* The host registers the branch provider as an ordinary built-in plugin. Its settings are the shared support geometry
   and path lists, plus the branch diameter that sets the double-wall threshold. Exceptions from calculation or
   publication are reported by PluginBase, rather than escaping the plugin C ABI. */

namespace slic3r_api::Support::Toolpath {
namespace {
// The key beyond the shared lists: the branch diameter whose circle area sets the double-wall threshold.
const raw_used_config_key k_branch_keys[] = {
    {"support_tree_branch_diameter_double_wall", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE}
};
} // namespace

// Declare the shared support geometry keys, then the path keys and the branch key; a null buffer returns the required count.
int32_t SupportToolpathBranch::used_config_keys(raw_used_config_key *keys) const noexcept
{
    const int32_t count = Common::append_used_config_keys(keys, 0, Common::support_geometry_keys());
    const int32_t path_count = Common::append_used_config_keys(keys, count, Common::support_path_keys());
    return Common::append_used_config_keys(keys, path_count, Common::UsedKeyList{k_branch_keys, std::size(k_branch_keys)});
}

// Run the branch role with callback-local calculation state and finish its progress; calculation and API errors are reported by PluginBase.
void SupportToolpathBranch::run_impl(const plugin_run_context *context) const
{
    throw_if_cancelled(context);
    generate_branch_paths(context);
    progress().finish_run();
}

// Register a static branch provider; the host retains its registration while the provider owns its process-lifetime instance.
void register_support_toolpath_branch_plugin(orchestrator_handle *orchestrator)
{
    static SupportToolpathBranch provider(orchestrator);
    orchestrator_register_plugin(orchestrator, provider.c_instance());
}
} // namespace slic3r_api::Support::Toolpath
