///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* Shared support-demand operations on borrowed object and island views.
   Enforcers are clipped to an island by the caller; add_to_demand() unions that result with existing demand.
   remove_from_demand() grows blockers by 1000 * SCALED_EPSILON before subtraction. Both convert and validate
   the result, then perform one cancellation check right before replacing island demand through the step context;
   callers keep their per-layer/island checks. Shared addition unites first, then converts and validates. The overhangs
   plugin keeps its own order: convert and validate unsupported areas before union, with no revalidation or cancellation
   check after union. Changing that order could change output.

   accumulate_lower_slices() adds one lower layer grown by 0.01 mm to existing coverage without growing the
   accumulated area again. Callers retain either an operand or polygon copies between layers.
   overhang_margin() converts threshold + 1 degrees to a horizontal allowance, falling back to half an external
   perimeter width. The explicit guard keeps angles below PI/2 only for callers that require that limit. */
#ifndef plugins_support_common_demand_hpp_
#define plugins_support_common_demand_hpp_

#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"

namespace slic3r_api::Support::Common {

// Counts object islands from first_layer onward for progress accounting.
uint32_t island_work_count(const Object &object, uint32_t first_layer = 0);
// Unions clipped enforcer areas with island demand and publishes after validation and cancellation checking.
void add_to_demand(const plugin_run_context *context, const LayerIsland &island, const ClipperContext &clipper,
                   ClipperOperand &&enforced);
// Subtracts enlarged blockers from existing island demand and publishes after validation and cancellation checking.
void remove_from_demand(const plugin_run_context *context, const LayerIsland &island, const ClipperContext &clipper,
                        ClipperOperand &&blockers);
// Unions existing coverage with one lower layer's slices grown by 0.01 mm.
ClipperOperand accumulate_lower_slices(const ClipperContext &clipper, const ClipperOperand &covered, const Layer &lower);
// Returns the scaled threshold allowance or half the scaled external width; unguarded callers must supply a threshold whose derived angle PI * (threshold + 1) / 180 yields a nonnegative allowance for scale_to_layer_coord().
coord_t overhang_margin(int32_t threshold, coord_t lower_height, coord_t external_width, bool guard_angle);

} // namespace slic3r_api::Support::Common
#endif // plugins_support_common_demand_hpp_
