///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#include "SupportToolpathBranch.hpp"
#include "BranchSheathPaths.hpp"

#include <iterator>

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Plugins/Support/Common/SupportSettings.hpp"

/* The host registers the branch provider as an ordinary built-in plugin. Its settings are the shared support geometry
   and path lists, plus the branch diameter that sets the double-wall threshold. Exceptions from calculation or
   publication are reported by PluginBase, rather than escaping the plugin C ABI.
   Registration copies the member's object-scoped selector description into the host. */

namespace slic3r_api::Support::Toolpath {
// Register the process-lifetime branch instance and copy its object-scoped selector description into the borrowed host;
// provider ownership stays static, and rejected declarations throw invalid_argument.
void register_support_toolpath_branch_plugin(orchestrator_handle *orchestrator);

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

void register_support_toolpath_branch_plugin(orchestrator_handle *orchestrator)
{
    static SupportToolpathBranch provider(orchestrator);
    orchestrator_register_plugin(orchestrator, provider.c_instance());
    raw_exclusive_group_def definition = raw_exclusive_group_def_init();
    definition.exclusive_group = "support_toolpath_branch";
    definition.label = definition.exclusive_group;
    definition.tooltip = "Choose which active plugin owns this exclusive plugin group.";
    definition.container_type = RAW_CONTAINER_TYPE_OBJECT;
    definition.category = RAW_OPTION_CATEGORY_SUPPORT;
    definition.has_invalidates_step = 1;
    definition.invalidates_step = STEP_SUPPORT_TOOLPATH;
    definition.ui_page = "Support material";
    definition.ui_group = "Support toolpaths";
    provider.register_exclusive_group(STEP_SUPPORT_TOOLPATH, definition);
}
} // namespace slic3r_api::Support::Toolpath
