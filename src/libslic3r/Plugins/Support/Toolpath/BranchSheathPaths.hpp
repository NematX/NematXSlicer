///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* Tree branch sheaths: the organic base paths of the legacy support, written with the plugin API only. The branch
   surfaces of each support layer, marked RAW_SURFACE_TYPE_MOD_BRANCH, get the sheaths and anchors of the legacy
   tree_supports_generate_paths (the algorithm is described in BranchSheathPaths.cpp). Tests drive the algorithm
   through the support toolpath step with synthetic auxiliary layers, so only the entry point is public. */
#ifndef plugins_support_toolpath_branch_sheath_paths_hpp_
#define plugins_support_toolpath_branch_sheath_paths_hpp_

#include "libslic3r/Api/plugin/c/slic3r_plugin_run_context.h"

namespace slic3r_api::Support::Toolpath {
// Generate the branch sheaths of every support layer of the callback's object and publish them into the support bucket
// of each object island's base-tool region island.
void generate_branch_paths(const plugin_run_context *context);
} // namespace slic3r_api::Support::Toolpath

#endif // plugins_support_toolpath_branch_sheath_paths_hpp_
