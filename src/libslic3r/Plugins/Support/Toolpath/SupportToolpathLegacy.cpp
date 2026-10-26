///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#include "SupportToolpathLegacy.hpp"
#include "LegacySupportPaths.hpp"

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Plugins/Support/Grid/SupportFamilyGrid.hpp"

/* The host registers all three algorithms as ordinary built-in plugins.
   Their shared setting list includes the grid flow dependencies and the
   region settings passed to Fill. Exceptions from calculation or publication
   are reported by PluginBase, rather than escaping the plugin C ABI. */

namespace slic3r_api::Support::Toolpath {
// Copy flow, pattern and region keys into the caller's buffer; a null buffer returns the required size, and the provider owns the key names.
int32_t SupportToolpathLegacy::used_config_keys(raw_used_config_key *keys) const noexcept
{
    static const raw_used_config_key used[] = {
        {"support_material_pattern", RAW_CO_ENUM, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"support_material_with_sheath", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"support_material_top_interface_pattern", RAW_CO_ENUM, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"support_material_bottom_interface_pattern", RAW_CO_ENUM, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"support_material_interface_contact_loops", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"support_material_angle_height", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"support_material_interface_angle", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"support_material_interface_angle_increment", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"raft_first_layer_density", RAW_CO_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"gap_fill_min_area", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"gap_fill_min_width", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"gap_fill_max_width", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"gap_fill_min_length", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"gap_fill_extension", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"fill_smooth_width", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"fill_smooth_distribution", RAW_CO_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE}
    };
    const plugin_instance grid = Grid::SupportFamilyGrid::instance(m_orchestrator).c_instance();
    const int32_t grid_count = grid.vt->used_config_keys(grid.ctx, keys);
    if (keys) std::copy(std::begin(used), std::end(used), keys + grid_count);
    return grid_count + int32_t(std::size(used));
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
