///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* Organic branch meshes: the paths of the routed trees as tubes, sliced at the middle of each slab.

   A path runs up one branch from the node where its trunk starts or splits, to the next split or to a tip. The tube of a
   path joins the circles of its nodes and closes with a half sphere at each end, as the legacy organic branches do.
   Slicing the tube at the middle of a slab gives the section of the branch in that slab. Tree branches are drawn as
   circles instead (TreeSupportGenerator.cpp).

       const std::vector<slic3r_api::StoredPolygonCollection> sections = slice_branch_meshes(out_storage, trees, slice_z, context); */
#ifndef plugins_support_tree_branch_mesh_hpp_
#define plugins_support_tree_branch_mesh_hpp_

#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_def.h"
#include "libslic3r/Api/plugin/c/slic3r_plugin_run_context.h"
#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"
#include "TreeRouting.hpp"

namespace Slic3r::TreeFamily {

// The sections of the branch meshes of the trees, one collection of out_storage per slice Z (scaled), in the order of slice_z. A Z
// that no branch crosses gets an empty collection. out_storage must outlive the returned collections.
// The trees are sliced in parallel, one job per tree, and the sections are joined in tree order, then in path order. That order of
// the polygons is the only one guaranteed: the order of the rings and their start points inside one section are the slicer's, and
// they vary from one call to the next when the slicer runs in parallel.
// The cancellation of the context is checked before each job and before each branch path; slicing one path is not interruptible
// (the slicing API does not expose the native slicer's cancellation). An invalid mesh (the host refuses it) throws std::invalid_argument.
std::vector<slic3r_api::StoredPolygonCollection> slice_branch_meshes(storage_handle *out_storage, const std::vector<RoutedTree> &trees,
                                                                     const std::vector<coord_t> &slice_z, const plugin_run_context *context);

} // namespace Slic3r::TreeFamily

#endif // plugins_support_tree_branch_mesh_hpp_
