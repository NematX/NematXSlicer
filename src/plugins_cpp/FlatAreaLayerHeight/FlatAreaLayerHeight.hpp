///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef plugins_flat_area_layer_height_hpp_
#define plugins_flat_area_layer_height_hpp_

#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"

namespace slic3r_api { namespace FlatAreaLayerHeightPlugin {

// STEP_LAYER_HEIGHT plugin that tries to place layer boundaries on large
// horizontal mesh surfaces while staying inside the configured min/max layer
// height range.
//
// The plugin does not directly create Layer objects. It only returns explicit
// [layer_top_z, layer_height] descriptors to the host through
// run_ctx_layer_height_generation. The next slicing step consumes those
// descriptors and builds the actual object layers.
class FlatAreaLayerHeight : public PluginBase
{
public:
    static FlatAreaLayerHeight &instance(orchestrator_handle *orch);
    static const char *print_ui_fragment() noexcept;

private:
    FlatAreaLayerHeight(orchestrator_handle *orch) : PluginBase(orch) {}

    const char *id_impl() const noexcept override;
    const char *name_impl() const noexcept override;
    const char *description_impl() const noexcept override;
    slicing_step_t step_impl() const noexcept override;
    const char *const *dependencies_impl() const noexcept override;
    // Return the common layer-height execution priority without changing state or transferring ownership; the call cannot fail.
    int32_t priority_impl() const noexcept override;
    // Return this algorithm's rank among layer-height alternatives.
    int32_t exclusive_group_priority_impl() const noexcept override { return 10; }
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override;
    int32_t defined_config_keys(const char **keys) const noexcept override;
    const char *progress_message_format_impl() const noexcept override;
    void inilialize_impl(storage_handle *storage) const override;
    void setup_run_impl(const plugin_run_context *run_ctx) const override;
    void run_impl(const plugin_run_context *run_ctx) const override;
};

void register_flat_area_layer_height_plugin(orchestrator_handle *orch);

}} // namespace slic3r_api::FlatAreaLayerHeightPlugin

#endif // plugins_flat_area_layer_height_hpp_
