///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#include "SupportToolpathRoles.hpp"
#include "libslic3r/Api/plugin/cpp/OrchestratorViews.hpp"

/* Support generators share preferences for the three automatic roles defined in SupportToolpathRoles.hpp.
   Sheath and legacy sparse share the sparse role. Neither helper reads or writes layer geometry.
   Call graph during generator initialization:
     legacy_toolpath_preferences -> build the three-role list
     declare_toolpath_preferences -> OrchestratorView::register_selector_auto_choice -> host registry */

namespace slic3r_api::Support::Common {
// Build an owned vector with borrowed static ids; sparse_provider must be a stable valid id.
// No host state changes; allocation failures propagate.
std::vector<ToolpathPreference> legacy_toolpath_preferences(const char *sparse_provider);
// Copy every style/role choice into the borrowed host before slicing; strings and providers remain borrowed.
// Rejection throws invalid_argument and stops initialization; allocation failures propagate without editing layers.
void declare_toolpath_preferences(orchestrator_handle *orchestrator, const std::vector<const char *> &styles,
                                  const std::vector<ToolpathPreference> &preferences);

std::vector<ToolpathPreference> legacy_toolpath_preferences(const char *sparse_provider)
{
    return {{k_sparse_role, sparse_provider},
            {k_bottom_interface_role, k_legacy_bottom_id},
            {k_top_interface_role, k_legacy_top_id}};
}

void declare_toolpath_preferences(orchestrator_handle *orchestrator, const std::vector<const char *> &styles,
                                  const std::vector<ToolpathPreference> &preferences)
{
    const OrchestratorView view(orchestrator);
    for (const char *style : styles)
        for (const ToolpathPreference &preference : preferences)
            view.register_selector_auto_choice(STEP_SUPPORT_TOOLPATH, preference.role, "support_material_style",
                                               style, preference.provider_id);
}
}
