///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* Organic branch meshes: the paths of the routed trees as tubes, sliced at the middle of each slab.

   A path runs up one branch from the node where its trunk starts or splits, to the next split or to a tip. The tube of a
   path joins the circles of its nodes and closes with a half sphere at each end, as the legacy organic branches do.
   Slicing the tube at the middle of a slab gives the section of the branch in that slab. Tree branches are drawn as
   circles instead (TreeSupportGenerator.cpp).

       const std::vector<Polygons> sections = slice_branch_meshes(trees, slice_z, throw_on_cancel); */
#ifndef plugins_support_tree_branch_mesh_hpp_
#define plugins_support_tree_branch_mesh_hpp_

#include <functional>
#include <vector>

#include "libslic3r/libslic3r.h"
#include "libslic3r/Polygon.hpp"
#include "TreeRouting.hpp"

namespace Slic3r::TreeFamily {

// The sections of the branch meshes of the trees, one entry per slice Z (scaled), in the order of slice_z. A Z that no branch
// crosses gets no polygons. throw_on_cancel is called while the meshes are sliced.
std::vector<Polygons> slice_branch_meshes(const std::vector<RoutedTree> &trees, const std::vector<coord_t> &slice_z,
                                          const std::function<void()> &throw_on_cancel);

} // namespace Slic3r::TreeFamily

#endif // plugins_support_tree_branch_mesh_hpp_
