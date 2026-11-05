///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef plugins_support_supportdemandbridgeinfillremoval_hpp_
#define plugins_support_supportdemandbridgeinfillremoval_hpp_

#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"

namespace slic3r_api { namespace Support { namespace SupportDemandBridgeInfillRemovalPlugin {

// STEP_POST_INFILL refinement of the support demand (bridge infill removal).
//
// The demand is written by STEP_SUPPORT_DEMAND (overhangs, painting, modifier
// volumes). When dont_support_bridges is enabled, this plugin removes the demand
// under the printed bridge infill lines whose two ends lie on the layer below, the
// rule support.demand.bridge_removal applies to the perimeters and gap fills. A
// cantilever or a floating surface keeps its demand, because its lines are not
// anchored at both ends. The result depends on the printed direction of the lines:
// the plugin pipeline sets no bridge direction, so they are printed at the fill
// angle. The demand of the enforcers is not kept (known limit of this version).
// It must run after any STEP_POST_INFILL plugin that changes the geometry of the
// bridge infill.
class SupportDemandBridgeInfillRemoval : public PluginBase
{
public:
    // The process-wide instance, created on the first call with the orchestrator that owns the plugin; later calls return it unchanged.
    static SupportDemandBridgeInfillRemoval &instance(orchestrator_handle *orch);

private:
    SupportDemandBridgeInfillRemoval(orchestrator_handle *orch) : PluginBase(orch) {}

    const char *id_impl() const noexcept override;
    const char *name_impl() const noexcept override;
    const char *description_impl() const noexcept override;
    slicing_step_t step_impl() const noexcept override;
    const char *const *dependencies_impl() const noexcept override;
    int32_t priority_impl() const noexcept override;
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override;
    int32_t defined_config_keys(const char **keys) const noexcept override;
    const char *progress_message_format_impl() const noexcept override;
    // Reserves one progress unit per layer above the first one when bridge infill removal is enabled.
    void setup_run_impl(const plugin_run_context *run_ctx) const override;
    // Removes the demand under the bridge infill lines, one parallel job per layer; cancellation and job failures propagate.
    void run_impl(const plugin_run_context *run_ctx) const override;
};

// Registers the plugin instance with the orchestrator; called by the plugin loader at startup and by the tests.
void register_support_demand_bridge_infill_removal_plugin(orchestrator_handle *orch);

}}} // namespace slic3r_api::Support::SupportDemandBridgeInfillRemovalPlugin

#endif // plugins_support_supportdemandbridgeinfillremoval_hpp_
