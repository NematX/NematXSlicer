///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* Five legacy providers print sparse, bottom contact, bottom interface, top contact and top interface
   support in that execution order, at priorities 0, 90, 100, 190 and 200 respectively.
   Their ids use the support.toolpath. prefix: legacy_sparse, legacy_bottom_contact, legacy_bottom, legacy_top_contact and legacy_top.

   Legacy sparse shares support_toolpath_sparse with branch_sheath: the object selector chooses one,
   using group priority as fallback when no family preference matches. Bottom and top have their own
   groups. The host uses execution priorities without imposing a role order; legacy contacts run before their interfaces
   so an interface can print areas returned by its contact provider. Provider instances live for the process; all calculation state
   lives inside the object callback and may run concurrently for other objects. */
#ifndef plugins_support_toolpath_legacy_hpp_
#define plugins_support_toolpath_legacy_hpp_
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"

namespace slic3r_api::Support::Toolpath {
enum class SurfaceRole { Sparse, BottomInterface, TopInterface, BottomContact, TopContact };
class SupportToolpathLegacy final : public PluginBase {
public:
    // Connect one support-surface role to its stable plugin metadata.
    SupportToolpathLegacy(orchestrator_handle *orchestrator, SurfaceRole role, const char *id, const char *group, int32_t priority)
        : PluginBase(orchestrator), m_role(role), m_id(id), m_group(group), m_priority(priority) {}
private:
    // Return the stable activation identifier.
    const char *id_impl() const noexcept override { return m_id; }
    // Return the provider's name in plugin lists.
    const char *name_impl() const noexcept override { return m_id; }
    // Describe the provider's output.
    const char *description_impl() const noexcept override { return "Generate native support paths from typed auxiliary surfaces."; }
    // Select the per-object support toolpath step.
    slicing_step_t step_impl() const noexcept override { return STEP_SUPPORT_TOOLPATH; }
    // Select exactly one algorithm for this surface role.
    const char *exclusive_group_impl() const noexcept override { return m_group; }
    // Allow support surfaces from any family without a provider dependency.
    const char *const *dependencies_impl() const noexcept override { return nullptr; }
    // Order sparse, bottom contact, bottom interface, top contact and top interface at 0/90/100/190/200.
    int32_t priority_impl() const noexcept override { return m_priority; }
    // Return group fallback priority 0 for sparse and 100 for the other four roles.
    int32_t exclusive_group_priority_impl() const noexcept override;
    // Declare direct settings and settings used by native flows and fillers.
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override;
    // Generate this role's paths in the support surface's region island.
    void run_impl(const plugin_run_context *context) const override;
    SurfaceRole m_role;
    const char *m_id;
    const char *m_group;
    int32_t m_priority;
};
// Register the process-lifetime sparse provider.
void register_support_toolpath_sparse_plugin(orchestrator_handle *orchestrator);
// Register the process-lifetime bottom-contact provider.
void register_support_toolpath_bottom_contact_plugin(orchestrator_handle *orchestrator);
// Register the process-lifetime bottom-interface provider.
void register_support_toolpath_bottom_plugin(orchestrator_handle *orchestrator);
// Register the process-lifetime top-contact provider.
void register_support_toolpath_top_contact_plugin(orchestrator_handle *orchestrator);
// Register the process-lifetime top-interface provider.
void register_support_toolpath_top_plugin(orchestrator_handle *orchestrator);
}
#endif
