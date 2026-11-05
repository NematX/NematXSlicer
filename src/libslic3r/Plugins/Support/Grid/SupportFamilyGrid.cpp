///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#include "SupportFamilyGrid.hpp"
#include "GridSupportGenerator.hpp"

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Plugins/Support/Common/SupportPublication.hpp"
#include "libslic3r/Plugins/Support/Common/SupportSettings.hpp"
#include "libslic3r/Plugins/Support/Common/SupportToolpathRoles.hpp"

/* Grid and snug support family run. It keeps the grid and snug styles, skips an object without demand,
   generates the support slabs with generate_support() and hands them to the shared publication
    (Common/SupportPublication.hpp), which creates the auxiliary layers.
    Initialization declares copied role preferences through Common::declare_toolpath_preferences before slicing. */

namespace slic3r_api::Support::Grid {
void SupportFamilyGrid::inilialize_impl(storage_handle *) const
{
    Common::declare_toolpath_preferences(m_orchestrator, {"grid", "snug"},
                                         Common::legacy_toolpath_preferences(Common::k_legacy_sparse_id));
}

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

// Skip unsupported styles or empty demand, and publish geometric support; host cancellation and calculation errors are reported by PluginBase.
void SupportFamilyGrid::run_impl(const plugin_run_context *context) const
{
    Common::run_support_family(context, m_orchestrator, progress(), Common::SupportStyle::Grid, Common::SupportStyle::Snug,
                               &generate_support);
}

// Register the static provider with the host, which manages activation and shared scratch storage.
void register_support_family_grid_plugin(orchestrator_handle *orchestrator)
{
    orchestrator_register_plugin(orchestrator, SupportFamilyGrid::instance(orchestrator).c_instance());
}
}
