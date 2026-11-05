///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#include "SupportToolpathLegacy.hpp"
#include "LegacySupportPaths.hpp"

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Plugins/Support/Common/SupportSettings.hpp"
#include "libslic3r/Plugins/Support/Common/SupportToolpathRoles.hpp"

/* The host registers all three algorithms as ordinary built-in plugins.
   Their setting list is the shared support geometry list of the Common
   library plus the path list (patterns, angles and the region settings passed
   to Fill). Registration copies each member's selector description into the host.
   Exceptions from calculation or publication
   are reported by PluginBase, rather than escaping the plugin C ABI. */

namespace slic3r_api::Support::Toolpath {
// Read the fixed role and return fallback rank 0 for sparse or 100 otherwise; no state changes, ownership transfers or failures.
// int32_t SupportToolpathLegacy::exclusive_group_priority_impl() const noexcept;

int32_t SupportToolpathLegacy::exclusive_group_priority_impl() const noexcept
{
    return m_role == SurfaceRole::Sparse ? 0 : 100;
}

// Register the process-lifetime sparse instance and copy its object-scoped selector description into the borrowed host;
// provider ownership stays static, and rejected declarations throw invalid_argument.
void register_support_toolpath_sparse_plugin(orchestrator_handle *orchestrator);
// Register the process-lifetime bottom instance and copy its object-scoped selector description into the borrowed host;
// provider ownership stays static, and rejected declarations throw invalid_argument.
void register_support_toolpath_bottom_plugin(orchestrator_handle *orchestrator);
// Register the process-lifetime top instance and copy its object-scoped selector description into the borrowed host;
// provider ownership stays static, and rejected declarations throw invalid_argument.
void register_support_toolpath_top_plugin(orchestrator_handle *orchestrator);

// Declare the shared support geometry keys, then the pattern, angle and filler region keys; a null buffer returns the required count.
int32_t SupportToolpathLegacy::used_config_keys(raw_used_config_key *keys) const noexcept
{
    const int32_t count = Common::append_used_config_keys(keys, 0, Common::support_geometry_keys());
    return Common::append_used_config_keys(keys, count, Common::support_path_keys());
}

// Run this role with callback-local calculation state and finish its progress; calculation and API errors are reported by PluginBase.
void SupportToolpathLegacy::run_impl(const plugin_run_context *context) const
{
    throw_if_cancelled(context);
    generate_paths(context, m_role);
    progress().finish_run();
}

void register_support_toolpath_sparse_plugin(orchestrator_handle *orchestrator)
{
    static SupportToolpathLegacy provider(orchestrator, SurfaceRole::Sparse, Common::k_legacy_sparse_id, Common::k_sparse_role, 0);
    orchestrator_register_plugin(orchestrator, provider.c_instance());
    raw_exclusive_group_def definition = raw_exclusive_group_def_init();
    definition.exclusive_group = Common::k_sparse_role;
    definition.auto_value = 1;
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
void register_support_toolpath_bottom_plugin(orchestrator_handle *orchestrator)
{
    static SupportToolpathLegacy provider(orchestrator, SurfaceRole::Bottom, Common::k_legacy_bottom_id, Common::k_bottom_interface_role, 100);
    orchestrator_register_plugin(orchestrator, provider.c_instance());
    raw_exclusive_group_def definition = raw_exclusive_group_def_init();
    definition.exclusive_group = Common::k_bottom_interface_role;
    definition.auto_value = 1;
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
void register_support_toolpath_top_plugin(orchestrator_handle *orchestrator)
{
    static SupportToolpathLegacy provider(orchestrator, SurfaceRole::Top, Common::k_legacy_top_id, Common::k_top_interface_role, 200);
    orchestrator_register_plugin(orchestrator, provider.c_instance());
    raw_exclusive_group_def definition = raw_exclusive_group_def_init();
    definition.exclusive_group = Common::k_top_interface_role;
    definition.auto_value = 1;
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
}
