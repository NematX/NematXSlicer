///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* The distance within which two print Z of the support are brought together, shared by the tree family (align_slab_bounds), the host pass
   at the end of STEP_SUPPORT (close_z_mapping) and the legacy toolpaths (the contact match). Header only, no plugin API dependency.

   The tool ordering asserts when two distinct print Z of the print are within 4 * SCALED_EPSILON (512 units, 0.5 um) of each other, so that
   distance is the minimum of the tolerance. With a Z step the print Z of the object layers are multiples of it (the layer heights, the first
   layer height, the raft and support heights are all rounded by check_z_step), so two distinct object Z differ by at least z_step: half of
   it as a tolerance reaches a second object Z only from the exact midpoint of two object Z at one step apart, where the tie rules decide (the
   tree takes the upper one, the host pass the lower one; the choice of the tree is then fixed for the host pass, which sees an object Z).
   The object Z are the fixed values of the passes and never move. */
#ifndef plugins_support_common_close_z_hpp_
#define plugins_support_common_close_z_hpp_

#include <algorithm>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_def.h"

namespace slic3r_api::Support::Common {

// The smallest tolerance: the distance that the tool ordering requires between two distinct print Z.
constexpr coord_t min_close_z_tolerance = 4 * SCALED_EPSILON;

// The largest tolerance for a Z step in mm (the z_step option, 0 for none): max(z_step / 2, 0.5 um). Any tolerance used by the passes is at most this
// value, so margins that must allow for the moves of both passes (the contact match of the toolpaths) are computed from it.
inline coord_t close_z_tolerance_limit(double z_step_mm)
{
    // Rounded like the layer Z (scale_to_layer_coord), so that 0.005 mm is exactly 5000 units.
    return z_step_mm > 0. ? std::max(min_close_z_tolerance, coord_t(scale_to_layer_coord(z_step_mm) / 2)) : min_close_z_tolerance;
}

// The tolerance of a pass whose fixed values (the print Z of the object layers; unsorted, repeats allowed) are given: the limit, reduced to half
// the smallest distance between two distinct fixed values when the layers are closer than the Z step suggests (a layer height that is not a
// multiple of it), but never below min_close_z_tolerance.
inline coord_t close_z_tolerance(double z_step_mm, std::vector<coord_t> fixed)
{
    coord_t tolerance = close_z_tolerance_limit(z_step_mm);
    std::sort(fixed.begin(), fixed.end());
    fixed.erase(std::unique(fixed.begin(), fixed.end()), fixed.end());
    for (size_t idx = 1; idx < fixed.size(); ++idx)
        tolerance = std::min(tolerance, coord_t((fixed[idx] - fixed[idx - 1]) / 2));
    return std::max(tolerance, min_close_z_tolerance);
}

} // namespace slic3r_api::Support::Common

#endif
