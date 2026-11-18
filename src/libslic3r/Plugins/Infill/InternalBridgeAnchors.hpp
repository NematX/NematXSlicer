///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef plugins_infill_internalbridgeanchors_hpp_
#define plugins_infill_internalbridgeanchors_hpp_

#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"

namespace slic3r_api { namespace Infill { namespace InternalBridgeAnchorsPlugin {

// Register this built-in with register_internal_bridge_anchors_plugin(). The
// pass extends requested internal-bridge thread ends after infill generation
// and puts those bridge children after the other children of their bucket.
class InternalBridgeAnchors : public PluginBase
{
public:
    // Return the process-local plugin instance attached to the orchestrator.
    static InternalBridgeAnchors &instance(orchestrator_handle *orch);

private:
    // Construct the plugin view without taking ownership of the orchestrator.
    InternalBridgeAnchors(orchestrator_handle *orch) : PluginBase(orch) {}

    // Return the stable identifier used by activation and dependency lookup.
    const char *id_impl() const noexcept override;
    // Return the short name shown by plugin diagnostics.
    const char *name_impl() const noexcept override;
    // Describe the post-infill bridge anchoring operation.
    const char *description_impl() const noexcept override;
    // Select the post-infill generation step for this plugin.
    slicing_step_t step_impl() const noexcept override;
    // Return the bridge preparation dependency required before this pass.
    const char *const *dependencies_impl() const noexcept override;
    // Run after gap fill, dense ordering and full-fill volume processing.
    int32_t priority_impl() const noexcept override;
    // Report configuration keys associated with this pass's bridge anchoring.
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override;
    // Compute and apply bridge-end changes for one post-infill generation run.
    void run_impl(const plugin_run_context *run_ctx) const override;
};

// Register the plugin and add it to the bridge_preparation activation group.
void register_internal_bridge_anchors_plugin(orchestrator_handle *orch);

}}} // namespace slic3r_api::Infill::InternalBridgeAnchorsPlugin

#endif // plugins_infill_internalbridgeanchors_hpp_
