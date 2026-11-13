///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef plugins_surface_initialtypedsurfacebuilder_hpp_
#define plugins_surface_initialtypedsurfacebuilder_hpp_

#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"

namespace slic3r_api { namespace SurfaceGeneration { namespace InitialTypedSurfaceBuilderPlugin {

// Baseline STEP_SURFACE_GENERATION plugin.
//
// Perimeter generation publishes island-level fill areas. This plugin is the
// first surface-generation pass: it creates the initial LayerRegionIsland
// surfaces and classifies each area as bottom, internal, or top by comparing it
// with the layer islands immediately below and above.
class InitialTypedSurfaceBuilder : public PluginBase
{
public:
    static InitialTypedSurfaceBuilder &instance(orchestrator_handle *orch);

private:
    InitialTypedSurfaceBuilder(orchestrator_handle *orch) : PluginBase(orch) {}

    const char *id_impl() const noexcept override;
    const char *name_impl() const noexcept override;
    const char *description_impl() const noexcept override;
    const char *exclusive_group_impl() const noexcept override;
    slicing_step_t step_impl() const noexcept override;
    const char *const *dependencies_impl() const noexcept override;
    int32_t priority_impl() const noexcept override;
    // Support settings decide whether bottoms above the first layer are bridges.
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override;
    // Prefer this algorithm among enabled alternatives for initial surface classification.
    int32_t exclusive_group_priority_impl() const noexcept override { return 0; }
    const char *progress_message_format_impl() const noexcept override;
    void setup_run_impl(const plugin_run_context *run_ctx) const override;
    void run_impl(const plugin_run_context *run_ctx) const override;
};

void register_initial_typed_surface_builder_plugin(orchestrator_handle *orch);

}}} // namespace slic3r_api::SurfaceGeneration::InitialTypedSurfaceBuilderPlugin

#endif // plugins_surface_initialtypedsurfacebuilder_hpp_
