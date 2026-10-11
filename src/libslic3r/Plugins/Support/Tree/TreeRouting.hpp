///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* Routed tree branches of one sliced object, kept in memory for the slab drawing and for the tests.

   route_trees() reads the island support demand of the object, adds the top contacts it finds to the support storage
   through the shared helper (Common/SupportZPlacement.hpp), places the tips under those contacts and propagates the
   branches down the object layers, and keeps only the contact parts that a placed tip reaches. Nothing is drawn and nothing
   is published: the result only lists the nodes.

   A tree is one connected set of nodes: the tips that merged into the same trunk and the trunk down to the build
   plate or to the model. Its nodes are listed from the top layer down, and each node names the nodes of the layer
   above that it carries (carries), so the branches can be read from the tips down.

       Common::GeneratedSupport support(storage);
       const std::vector<RoutedTree> trees = route_trees(print, object, params, settings, support);
       for (const TreeNode &node : trees.front().nodes) ... */
#ifndef plugins_support_tree_routing_hpp_
#define plugins_support_tree_routing_hpp_

#include <cstddef>
#include <limits>
#include <vector>

#include "libslic3r/libslic3r.h"
#include "libslic3r/Point.hpp"
#include "libslic3r/Plugins/Support/Common/SupportLayers.hpp"
#include "libslic3r/Plugins/Support/Common/SupportParams.hpp"
#include "TreeSettings.hpp"

namespace Slic3r {

class Print;
class PrintObject;

namespace TreeFamily {

// One node of a branch, on one object layer.
struct TreeNode {
    // Object layer of the node, and the print Z of that layer in millimeters.
    size_t layer = 0;
    double print_z = 0;
    // Centre of the branch on its layer, and the radius it is drawn with (scaled).
    Point position;
    coord_t radius = 0;
    // The branch reaches the build plate from this node, or rests on the model.
    bool to_buildplate = false;
    bool to_model_gracious = false;
    // The propagation lost track of the branch here; a diagnostic flag of the legacy tree.
    bool lost = false;
    // The branch hangs in the air here; a root with this flag goes down to the bed, as the legacy tree does.
    bool verylost = false;
    // Indices into the same tree, of the nodes on the layer above that this node carries.
    std::vector<size_t> carries;
};

// A connected set of nodes, listed from the top layer down.
struct RoutedTree {
    std::vector<TreeNode> nodes;
};

// Marks a link to no node: the node below a trunk bottom, or a node that no other node carries.
inline constexpr size_t no_node = std::numeric_limits<size_t>::max();

// Route the tree branches of one object. The top contacts are added to support.top; the returned trees support them.
std::vector<RoutedTree> route_trees(const Print &print, const PrintObject &object, const slic3r_api::Support::Common::SupportParams &params,
                                    const TreeSettings &settings, slic3r_api::Support::Common::GeneratedSupport &support);

// The index of the node below each node of the tree, or no_node. Each node is carried by at most one node below it.
std::vector<size_t> nodes_below(const RoutedTree &tree);

} // namespace TreeFamily
} // namespace Slic3r

#endif // plugins_support_tree_routing_hpp_
