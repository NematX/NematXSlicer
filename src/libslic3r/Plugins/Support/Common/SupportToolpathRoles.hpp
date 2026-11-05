///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* A role groups alternative algorithms for sparse lines or either side's contact and interface lines.
   A family is a support generator such as grid or tree.
   These helpers declare preferences for all five automatic roles, including sheath as a sparse algorithm.
   During family initialization, for example:
     const std::vector<ToolpathPreference> choices = legacy_toolpath_preferences(k_legacy_sparse_id);
     declare_toolpath_preferences(orchestrator, {"grid", "snug"}, choices);
   The host copies choices for each serialized support style before planning slicing. */
#ifndef plugins_support_common_toolpath_roles_hpp_
#define plugins_support_common_toolpath_roles_hpp_

#include <vector>
#include "libslic3r/Api/plugin/c/slic3r_plugin_types.h"

namespace slic3r_api::Support::Common {
inline constexpr const char *k_sparse_role = "support_toolpath_sparse";
inline constexpr const char *k_bottom_contact_role = "support_toolpath_bottom_contact";
inline constexpr const char *k_top_contact_role = "support_toolpath_top_contact";
inline constexpr const char *k_bottom_interface_role = "support_toolpath_bottom_interface";
inline constexpr const char *k_top_interface_role = "support_toolpath_top_interface";
inline constexpr const char *k_legacy_sparse_id = "support.toolpath.legacy_sparse";
inline constexpr const char *k_legacy_bottom_contact_id = "support.toolpath.legacy_bottom_contact";
inline constexpr const char *k_legacy_top_contact_id = "support.toolpath.legacy_top_contact";
inline constexpr const char *k_legacy_bottom_id = "support.toolpath.legacy_bottom";
inline constexpr const char *k_legacy_top_id = "support.toolpath.legacy_top";
inline constexpr const char *k_branch_sheath_id = "support.toolpath.branch_sheath";

// One preferred algorithm for a role; both strings are borrowed static identifiers.
struct ToolpathPreference {
    const char *role;
    const char *provider_id;
};
// Return the five automatic roles' legacy algorithms, substituting the sparse algorithm.
std::vector<ToolpathPreference> legacy_toolpath_preferences(const char *sparse_provider);
// Copy each role preference for every serialized style into the host; rejection throws.
void declare_toolpath_preferences(orchestrator_handle *orchestrator, const std::vector<const char *> &styles,
                                  const std::vector<ToolpathPreference> &preferences);
}
#endif
