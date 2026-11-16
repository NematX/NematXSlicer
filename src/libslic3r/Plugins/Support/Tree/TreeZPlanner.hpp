///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* Heights of the branch slabs between two fixed Z extremes.

   A branch that leans sideways overhangs its own slab below: the horizontal outline overhang per unit of height is
   the rate of the branch, and a slab of height h keeps rate * h within support_max_overhang. The planner gives
   every object layer the largest rate of the branches entering it, and cuts the span between two extremes into
   slabs bottom first:

       the height of a slab starting at z is the largest grid height H within [min, max] such that the rate of the
       object layers the slab reaches, times H, stays within the maximum overhang; the search runs over the grid,
       since a slab that fits keeps fitting when it gets thinner.

   Slab boundaries advance from the lower extreme in these steps. When the rest below the upper extreme is thinner
   than the minimum height, the last boundaries are backed off, and their span is cut again: into the fewest even slabs
   that all satisfy the limits when such a cut exists, else by a search over the Z steps that can make the top slab the
   constrained one. A cut that does not fit backs off one more boundary. When no cut fits at all, the span becomes a
   single slab. Boundaries lie on the grid of max(z_step, 0.01 mm), measured from the lower extreme or from the
   upper one, so that a constrained slab can end at the upper extreme.

       const std::vector<coord_t> tops = plan_branch_slabs(from, to, layers, max_overhang, min, max, z_step); */
#ifndef plugins_support_tree_z_planner_hpp_
#define plugins_support_tree_z_planner_hpp_

#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_def.h"

namespace Slic3r::TreeFamily {

// One object layer of the planner: its Z span, and the largest horizontal overhang per unit of height of the branches that enter it.
struct OverhangLayer {
    coord_t bottom_z = 0, top_z = 0;
    double rate = 0;
};

// The Z grid of the planned heights and bounds: the machine Z step, and at least 0.01 mm.
coord_t planning_grid(coord_t z_step);

// Tops of the branch slabs that fill [from, to], bottom first; the last top is to. The layers are sorted by Z and do not overlap.
// The minimum and maximum heights and the Z step are scaled; a minimum or maximum off the step is rounded to it.
std::vector<coord_t> plan_branch_slabs(coord_t from, coord_t to, const std::vector<OverhangLayer> &layers, coord_t max_overhang,
                                       coord_t min_height, coord_t max_height, coord_t z_step);

} // namespace Slic3r::TreeFamily

#endif // plugins_support_tree_z_planner_hpp_
