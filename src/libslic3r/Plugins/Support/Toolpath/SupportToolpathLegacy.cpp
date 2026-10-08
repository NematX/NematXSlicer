///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#include "SupportToolpathLegacy.hpp"
#include "LegacySupportPaths.hpp"

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Plugins/Support/Common/SupportSettings.hpp"

/* The host registers all three algorithms as ordinary built-in plugins.
   Their setting list is the shared support geometry list of the Common
   library plus the path list (patterns, angles and the region settings passed
   to Fill). Exceptions from calculation or publication
   are reported by PluginBase, rather than escaping the plugin C ABI. */

namespace slic3r_api::Support::Toolpath {
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

// Register a static sparse provider; the host retains its registration while the provider owns its process-lifetime instance.
void register_support_toolpath_sparse_plugin(orchestrator_handle *orchestrator)
{
    static SupportToolpathLegacy provider(orchestrator, SurfaceRole::Sparse, "support.toolpath.legacy_sparse", "support_toolpath_sparse", 0);
    orchestrator_register_plugin(orchestrator, provider.c_instance());
}
// Register a static bottom provider; the host retains its registration while the provider owns its process-lifetime instance.
void register_support_toolpath_bottom_plugin(orchestrator_handle *orchestrator)
{
    static SupportToolpathLegacy provider(orchestrator, SurfaceRole::Bottom, "support.toolpath.legacy_bottom", "support_toolpath_bottom_interface", 100);
    orchestrator_register_plugin(orchestrator, provider.c_instance());
}
// Register a static top provider; the host retains its registration while the provider owns its process-lifetime instance.
void register_support_toolpath_top_plugin(orchestrator_handle *orchestrator)
{
    static SupportToolpathLegacy provider(orchestrator, SurfaceRole::Top, "support.toolpath.legacy_top", "support_toolpath_top_interface", 200);
    orchestrator_register_plugin(orchestrator, provider.c_instance());
}
}
