///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* The branch provider turns the tree branch surfaces into sheaths and anchors. It owns the support_toolpath_branch
   group and runs at priority 50, between the sparse base (0) and the interfaces (100, 200). The sheath algorithm
   lives in BranchSheathPaths.hpp; the provider instance lives for the process. */
#ifndef plugins_support_toolpath_branch_hpp_
#define plugins_support_toolpath_branch_hpp_
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"

namespace slic3r_api::Support::Toolpath {
class SupportToolpathBranch final : public PluginBase {
public:
    // Connect the provider to the orchestrator; its identifiers and priority are fixed.
    explicit SupportToolpathBranch(orchestrator_handle *orchestrator) : PluginBase(orchestrator) {}

private:
    // Return the stable activation identifier.
    const char *id_impl() const noexcept override { return "support.toolpath.branch_sheath"; }
    // Return the provider's name in plugin lists.
    const char *name_impl() const noexcept override { return "support.toolpath.branch_sheath"; }
    // Describe the provider's output.
    const char *description_impl() const noexcept override { return "Generate tree branch sheaths and anchors from branch support surfaces."; }
    // Select the per-object support toolpath step.
    slicing_step_t step_impl() const noexcept override { return STEP_SUPPORT_TOOLPATH; }
    // Select exactly one algorithm for the branch surface role.
    const char *exclusive_group_impl() const noexcept override { return "support_toolpath_branch"; }
    // Allow support surfaces from any family without a provider dependency.
    const char *const *dependencies_impl() const noexcept override { return nullptr; }
    // Order the branch provider between the sparse base and the interfaces.
    int32_t priority_impl() const noexcept override { return 50; }
    // Declare the shared support geometry and path settings, then the branch diameter of the double wall.
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override;
    // Generate the branch sheaths in the support layers' region islands.
    void run_impl(const plugin_run_context *context) const override;
};
// Register the process-lifetime branch provider.
void register_support_toolpath_branch_plugin(orchestrator_handle *orchestrator);
} // namespace slic3r_api::Support::Toolpath
#endif // plugins_support_toolpath_branch_hpp_
