///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* Smoothing of the organic branches, before their Z planning and their drawing.

   The routed nodes of an organic tree are relaxed in place, as the legacy organic branches are (OrganicSupport.cpp): every
   free node moves toward the point of the straight line between the node below and the mean of the nodes above that lies at
   the Z of the node (the exact midpoint on equal layer heights; the mean of the nodes above when there is no node below), and it is pushed out of the object
   outline grown by the XY distance where its circle reaches into that outline. Tips and roots that stand on the model keep
   their positions. A final pass pushes the free nodes that the last pull left inside the outline out of it, once. The planner and the
   drawing then use the smoothed paths. */
#ifndef plugins_support_tree_smoothing_hpp_
#define plugins_support_tree_smoothing_hpp_

#include <vector>

#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "TreeGeometry.hpp"
#include "TreeRouting.hpp"

namespace slic3r_api {
class Object;
}

namespace Slic3r::TreeFamily {

// Relaxes the XY positions of the free nodes of the trees of one object in place and returns the number of iterations that ran.
// Tips and roots that rest on the model are locked; every node keeps its layer and its links. The layer of each node must be a
// layer of the object, keep_out must hold at least one outline per object layer, grown by the XY distance, and the links of a
// tree must index nodes of that same tree. The context is checked after every iteration of the relaxation and after the final pass
// (nullptr never cancels); a cancellation throws PluginCancelled out of this function before anything is written back, so the trees
// keep their input positions.
size_t smooth_organic_branches(const slic3r_api::Object &object, const std::vector<slic3r_api::StoredPolygonCollection> &keep_out,
                               std::vector<RoutedTree> &trees, const plugin_run_context *context);

// Pushes a sphere that reaches only its own layer and returns its new center; the inputs are not changed.
// Positions and radius in mm, print_z the Z of each layer in mm, keep_out the outline of each in scaled units, layer must index both.
Vec2d push_sphere_out_of_outline(size_t layer, double radius, const Vec2d &position, const std::vector<double> &print_z,
                                 const std::vector<slic3r_api::StoredPolygonCollection> &keep_out);

} // namespace Slic3r::TreeFamily

#endif // plugins_support_tree_smoothing_hpp_
