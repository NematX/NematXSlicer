///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* Register this built-in family to publish dense filled support at object-layer heights.
   The process-lifetime provider reads final demand, calls FilledSupportGenerator, then Common publication.
   Initialization declares automatic toolpath preferences; the provider retains no per-object geometry. */
#ifndef plugins_support_filled_family_hpp_
#define plugins_support_filled_family_hpp_
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"

namespace slic3r_api::Support::Filled {
class SupportFamilyFilled final : public PluginBase {
public:
    // Construct the process-lifetime provider on first use and return a borrowed reference attached to the host.
    static SupportFamilyFilled &instance(orchestrator_handle *orchestrator);
private:
    // Connect the provider to the borrowed host orchestrator.
    explicit SupportFamilyFilled(orchestrator_handle *orchestrator) : PluginBase(orchestrator) {}
    // Return the stable activation id.
    const char *id_impl() const noexcept override { return "support.family.filled"; }
    // Return the name shown in plugin lists.
    const char *name_impl() const noexcept override { return "Filled support family"; }
    // Describe the geometric output of this family.
    const char *description_impl() const noexcept override { return "Build dense interface-only support layers at the object layers."; }
    // Select per-object support generation.
    slicing_step_t step_impl() const noexcept override { return STEP_SUPPORT; }
    // Accept demand from any producer without a provider dependency.
    const char *const *dependencies_impl() const noexcept override { return nullptr; }
    // Run with the default support-family priority.
    int32_t priority_impl() const noexcept override { return 0; }
    // Copy role preferences to the host; initialize_safe returns an error id on rejection, then the host disables the family.
    void inilialize_impl(storage_handle *storage) const override;
    // Fill a caller-sized buffer with shared geometry keys and raft_layers, or return its size for null.
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override;
    // Filter filled objects with demand, then publish storage-owned slabs; PluginBase reports errors and cancellation.
    void run_impl(const plugin_run_context *context) const override;
};
// Register the borrowed static provider without transferring ownership to the host.
void register_support_family_filled_plugin(orchestrator_handle *orchestrator);
}
#endif
