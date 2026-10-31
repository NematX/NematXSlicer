///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* Support slabs of the tree and organic styles of one object.

   generate_tree_support() routes the branches (TreeRouting.hpp), then builds the slabs that the family publishes:
   floors under the branches that stand on the model, the span between the extremes cut by the branch planner
   (TreeZPlanner.hpp), the object print Z values snapped into the slab bounds, the branches drawn in every slab (circles for
   tree, smoothed meshes for organic, TreeBranchMesh.hpp), the object outline removed, and the dense interfaces carved under the contacts and above the floors
   (Common/SupportInterfaces.hpp), the first base slab widened on the bed. The brim and the skirt are ignored. Without interface layers the contacts keep only the branches drawn in them.
   generate_tree_slabs() builds the same slabs from trees routed beforehand, and from the resting parts that route_trees() returns with them (the
   parts of the contacts over the object on their tip layer, which have no tip: each gets a floor on the object, and its stack shortens down to it,
   as in the grid). Organic trees are smoothed first
   (TreeSmoothing.hpp), and their branches become branch slabs, tree branches sparse base slabs. Nothing is published here:
   the family publishes the slabs. */
#ifndef plugins_support_tree_generator_hpp_
#define plugins_support_tree_generator_hpp_

#include <vector>

#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Plugins/Support/Common/SupportCloseZ.hpp"
#include "libslic3r/Plugins/Support/Common/SupportLayers.hpp"
#include "TreeRouting.hpp"

namespace slic3r_api {
class Object;
class Print;
}

namespace Slic3r {

namespace TreeFamily {

// Fill out with the slabs of the tree support of one object; organic selects branch slabs instead of base slabs. A null
// context never cancels.
// Routes and draws one object using its borrowed per-run parameters and one derived tree settings object.
void generate_tree_support(const plugin_run_context *context, const slic3r_api::Print &print, const slic3r_api::Object &object,
                           const slic3r_api::Support::Common::SupportParams &params, bool organic,
                           slic3r_api::Support::Common::GeneratedSupport &out);

// Fill out with the slabs of the given routed trees of one object, the contacts already in out.top. The same rules as
// generate_tree_support(), without the routing; organic trees are smoothed in place before the slabs are built. As in the grid,
// the first base slab is widened by raft_first_layer_expansion when it starts on the bed (Common::expand_first_layer_foot()).
// The brim and the skirt are ignored. The resting parts (RestingPart, from route_trees()) are the parts of the contacts that rest on the object: each gets
// a floor on the object under it, and the stack of the contact then shortens down to that floor, as in the grid.
// Builds slabs from routed nodes using the same borrowed parameters and tree settings that routed them.
void generate_tree_slabs(const plugin_run_context *context, const slic3r_api::Print &print, const slic3r_api::Object &object,
                         const slic3r_api::Support::Common::SupportParams &params, const TreeSettings &settings,
                         std::vector<RoutedTree> &trees, bool organic, slic3r_api::Support::Common::GeneratedSupport &out,
                         const std::vector<RestingPart> &resting);

// Snaps the bounds of the slabs onto the object print Z values within the tolerance of them, merges the support
// bounds that lie within the tolerance of each other, and merges the slabs that collapse; the rules are explained at the
// definition. object_zs is sorted and holds no duplicates. The tolerance is at least the 0.5 um that the tool ordering requires
// (Common::min_close_z_tolerance, the default); generate_tree_slabs() passes Common::close_z_tolerance() of the Z step.
void align_slab_bounds(const std::vector<coord_t> &object_zs, slic3r_api::Support::Common::GeneratedSupport &out,
                       coord_t tolerance = slic3r_api::Support::Common::min_close_z_tolerance);

} // namespace TreeFamily
} // namespace Slic3r

#endif // plugins_support_tree_generator_hpp_
