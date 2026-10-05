///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.

/* This demand filter runs after overhang, painting and modifier demand.
   It removes areas above any lower object slice when buildplate-only support
   is enabled. Geometry is accessed only through the plugin API; the object
   keeps the resulting demand, while temporary polygons use plugin storage. */
#ifndef plugins_support_supportdemandbuildplateonly_hpp_
#define plugins_support_supportdemandbuildplateonly_hpp_

#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"

namespace slic3r_api::Support::SupportDemandBuildplateOnlyPlugin {

class SupportDemandBuildplateOnly : public PluginBase
{
public:
    // Return the shared built-in provider for this orchestrator.
    static SupportDemandBuildplateOnly &instance(orchestrator_handle *orch);

private:
    // Connect this provider to the orchestrator's plugin API.
    explicit SupportDemandBuildplateOnly(orchestrator_handle *orch) : PluginBase(orch) {}
    // Plugin id used by activation settings.
    const char *id_impl() const noexcept override { return "support.demand.buildplate_only"; }
    // Name displayed in the plugin list.
    const char *name_impl() const noexcept override { return "Buildplate-only support demand"; }
    // Explain the filter in the plugin list.
    const char *description_impl() const noexcept override { return "Remove support demand above lower object slices."; }
    // Filter demand before support families build surfaces.
    slicing_step_t step_impl() const noexcept override { return STEP_SUPPORT_DEMAND; }
    // No other plugin is required to run this filter.
    const char *const *dependencies_impl() const noexcept override { return nullptr; }
    // Run after overhangs (0), painting (5) and modifiers (10).
    int32_t priority_impl() const noexcept override { return 100; }
    // Declare the object setting used to enable the filter.
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override;
    // Remove demand above the union of earlier object slices.
    void run_impl(const plugin_run_context *run_ctx) const override;
};

// Register the built-in buildplate-only demand provider.
void register_support_demand_buildplate_only_plugin(orchestrator_handle *orch);

} // namespace slic3r_api::Support::SupportDemandBuildplateOnlyPlugin

#endif // plugins_support_supportdemandbuildplateonly_hpp_
