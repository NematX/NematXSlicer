///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_support_interface_gap_fill_hpp_
#define slic3r_support_interface_gap_fill_hpp_
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"

/* Complete narrow uncovered areas after dense support providers publish paths.
   Register and activate this ordinary provider before plugin initialization so
   its object option and UI rule exist before configuration caches are built.
   The process-lifetime instance retains no per-print state; object callbacks
   borrow host storage and build additions before changing destination trees. */
namespace slic3r_api::Support::Toolpath {
class SupportInterfaceGapFill final : public PluginBase {
public:
    // Return one process-lifetime provider; the first host supplies its orchestrator.
    static SupportInterfaceGapFill &instance(orchestrator_handle *);
    // Connect the provider to its host without retaining print data.
    explicit SupportInterfaceGapFill(orchestrator_handle *host) : PluginBase(host) {}
private:
    // Return the stable activation identifier.
    const char *id_impl() const noexcept override { return "support.toolpath.interface_gap_fill"; }
    // Return the plugin-list label.
    const char *name_impl() const noexcept override { return "Support interface gap fill"; }
    // Describe the additional paths.
    const char *description_impl() const noexcept override { return "Complete uncovered narrow areas of dense support interfaces."; }
    // Consume support surfaces and paths in the per-object toolpath step.
    slicing_step_t step_impl() const noexcept override { return STEP_SUPPORT_TOOLPATH; }
    // Run after the ordinary interface providers.
    int32_t priority_impl() const noexcept override { return 300; }
    // Consume published geometry without requiring a particular provider.
    const char *const *dependencies_impl() const noexcept override { return nullptr; }
    // Copy shared geometry/path keys and direct activation keys; a null buffer only counts them.
    int32_t used_config_keys(raw_used_config_key *) const noexcept override;
    // Write the sole owned option key, or just return its count for a null buffer.
    int32_t defined_config_keys(const char **) const noexcept override;
    // Create the disabled object option and UI rule; host registration failures throw.
    void inilialize_impl(storage_handle *) const override;
    // Run object-local processing and finish progress; exceptions reach PluginBase's C ABI boundary.
    void run_impl(const plugin_run_context *) const override;
};
// Register the process-lifetime provider with the host.
void register_support_interface_gap_fill_plugin(orchestrator_handle *);
}
#endif
