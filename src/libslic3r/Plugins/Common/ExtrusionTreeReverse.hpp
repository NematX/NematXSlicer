///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Plugins_Common_ExtrusionTreeReverse_hpp_
#define slic3r_Plugins_Common_ExtrusionTreeReverse_hpp_

#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"

/* Reverse a whole extrusion tree in place, for providers that impose a direction
   (loop direction) or choose one (ordering). Every local path is reversed, then
   the child order of every collection, so a loop split into fragments still runs
   as one continuous path. Existing handles survive; no permission is changed.

   An ordering provider uses reverse_tree_keeping_loops(): a closed descendant,
   reversible or not, keeps its direction. It ends where it starts, so turning it
   around would not help: only its place in the order changes and the tree stays
   continuous. The children of a loop (a fixed, continuous, closed node) are its
   pieces, so they still turn with it.

       if (want_ccw != is_ccw)
           ExtrusionTree::reverse_tree(loop, run_ctx);
       if (candidate.readonly().reversible())
           ExtrusionTree::reverse_tree_keeping_loops(candidate, run_ctx);
*/
namespace slic3r_api { namespace ExtrusionTree {

/* Reverse every local path and child order so the complete tree runs backward. */
void reverse_tree(MutableExtrusionEntity entity, const plugin_run_context *run_ctx = nullptr);

/* Like reverse_tree(), but a closed descendant keeps its direction, unless its parent is a loop. */
void reverse_tree_keeping_loops(MutableExtrusionEntity entity, const plugin_run_context *run_ctx = nullptr);

}} // namespace slic3r_api::ExtrusionTree

#endif // slic3r_Plugins_Common_ExtrusionTreeReverse_hpp_
