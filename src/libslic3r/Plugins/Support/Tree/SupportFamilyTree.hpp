///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* This built-in support family handles the tree and organic styles. It reads the demand of the object islands, routes
   the branches and computes their slabs (TreeSupportGenerator.hpp), and publishes the slabs through the shared
   publication (Common/SupportPublication.hpp) as auxiliary slices, support properties and typed fill surfaces, all
   through the plugin API. Extrusion generation belongs to a later support-toolpath step. */
#ifndef plugins_support_tree_family_hpp_
#define plugins_support_tree_family_hpp_
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"

namespace slic3r_api::Support::Tree {
class SupportFamilyTree final : public PluginBase {
public:
    // Return the process-lifetime built-in provider.
    static SupportFamilyTree &instance(orchestrator_handle *orchestrator);
private:
    // Connect the provider to the host orchestrator.
    explicit SupportFamilyTree(orchestrator_handle *orchestrator) : PluginBase(orchestrator) {}
    // Return the stable activation id.
    const char *id_impl() const noexcept override { return "support.family.tree"; }
    // Return the name shown in plugin lists.
    const char *name_impl() const noexcept override { return "Tree and organic support family"; }
    // Describe the geometric output of this family.
    const char *description_impl() const noexcept override { return "Build tree and organic support branches with typed surfaces."; }
    // Select the per-object support generation step.
    slicing_step_t step_impl() const noexcept override { return STEP_SUPPORT; }
    // Allow any demand producer without requiring a particular provider.
    const char *const *dependencies_impl() const noexcept override { return nullptr; }
    // Run the family at the default support priority.
    int32_t priority_impl() const noexcept override { return 0; }
    // Declare the geometry keys and the tree branch settings.
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override;
    // Filter the object, generate the branch slabs and publish them as typed auxiliary layers.
    void run_impl(const plugin_run_context *context) const override;
};

// Register the built-in tree and organic support provider.
void register_support_family_tree_plugin(orchestrator_handle *orchestrator);
}

#endif
