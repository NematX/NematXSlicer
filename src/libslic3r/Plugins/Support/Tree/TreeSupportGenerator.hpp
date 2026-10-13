///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* Support slabs of the tree and organic styles of one object.

   generate_tree_support() routes the branches (TreeRouting.hpp), then builds the slabs that the family publishes:
   floors under the branches that stand on the model, the span between the extremes cut by the branch planner
   (TreeZPlanner.hpp), the object print Z values snapped into the slab bounds, the branches drawn in every slab (circles for
   tree, smoothed meshes for organic, TreeBranchMesh.hpp), the object outline removed, and the dense interfaces carved under the contacts and above the floors
   (Common/SupportInterfaces.hpp), the first base slab widened on the bed, and the reserved brim area of the object removed from the first layer slabs. Without interface layers the contacts keep only the branches drawn in them.
   generate_tree_slabs() builds the same slabs from trees routed beforehand. Organic trees are smoothed first
   (TreeSmoothing.hpp), and their branches become branch slabs, tree branches sparse base slabs. Nothing is published here:
   the family publishes the slabs. */
#ifndef plugins_support_tree_generator_hpp_
#define plugins_support_tree_generator_hpp_

#include <vector>

#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Plugins/Support/Common/SupportLayers.hpp"
#include "TreeRouting.hpp"

namespace Slic3r {

class Print;
class PrintObject;

namespace TreeFamily {

// Fill out with the slabs of the tree support of one object; organic selects branch slabs instead of base slabs. A null
// context never cancels.
void generate_tree_support(const plugin_run_context *context, const Print &print, const PrintObject &object, bool organic,
                           slic3r_api::Support::Common::GeneratedSupport &out);

// Fill out with the slabs of the given routed trees of one object, the contacts already in out.top. The same rules as
// generate_tree_support(), without the routing; organic trees are smoothed in place before the slabs are built. As in the grid,
// the first base slab is widened by raft_first_layer_expansion when it starts on the bed (Common::expand_first_layer_foot()), then the
// reserved brim area of the object is removed from the slabs of the first layer (Common::remove_first_layer_brim()), which can empty them.
void generate_tree_slabs(const plugin_run_context *context, const Print &print, const PrintObject &object, std::vector<RoutedTree> &trees,
                         bool organic, slic3r_api::Support::Common::GeneratedSupport &out);

// Snaps the bounds of the slabs onto the object print Z values within the layer merge tolerance of them, merges the support
// bounds that lie within the tolerance of each other, and merges the slabs that collapse; the rules are explained at the
// definition. object_zs is sorted and holds no duplicates.
void align_slab_bounds(const std::vector<coord_t> &object_zs, slic3r_api::Support::Common::GeneratedSupport &out);

} // namespace TreeFamily
} // namespace Slic3r

#endif // plugins_support_tree_generator_hpp_
