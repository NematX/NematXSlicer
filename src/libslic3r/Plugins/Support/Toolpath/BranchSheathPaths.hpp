///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* Tree branch toolpaths: the organic base paths of the legacy support, written with the plugin API only. The branch
   surfaces of each support layer, marked RAW_SURFACE_TYPE_MOD_BRANCH, are fused into islands and classified. Sheath islands
   print the sheaths and anchors of the legacy tree_supports_generate_paths; thin elongated islands print one centreline (or
    one stationary deposit when too narrow for a line); blob islands print one stationary deposit at the contour centroid when the island contains it, else at the closest boundary point. Tests drive the algorithm
   through the support toolpath step with synthetic auxiliary layers, so only the entry point is public. */
#ifndef plugins_support_toolpath_branch_sheath_paths_hpp_
#define plugins_support_toolpath_branch_sheath_paths_hpp_

#include "libslic3r/Api/plugin/c/slic3r_plugin_run_context.h"

namespace slic3r_api::Support::Toolpath {
// Generate branch sheaths, centrelines or stationary deposits in each support layer's base-tool support bucket.
void generate_branch_paths(const plugin_run_context *context);
} // namespace slic3r_api::Support::Toolpath

#endif // plugins_support_toolpath_branch_sheath_paths_hpp_
