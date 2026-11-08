///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#include "SupportToolpathRoles.hpp"
#include "libslic3r/Api/plugin/cpp/OrchestratorViews.hpp"

/* Support generators choose the sparse algorithm for each support style.
   Sheath and legacy sparse share that role. Dense roles use their group's
   ranking, so an alternative interface algorithm can be selected automatically
   without replacing a support family. Explicit object selections still win.
   Neither helper reads or writes layer geometry.
   Call graph during generator initialization:
     legacy_toolpath_preferences -> build the sparse preference
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
    return {{k_sparse_role, sparse_provider}};
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
