///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#include "SupportDemand.hpp"

#include <cmath>
#include <utility>

#include "libslic3r/Api/plugin/c/steps/slic3r_step_support_demand.h"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"

/* Support-demand boolean edits and geometric allowances (see SupportDemand.hpp).
   Results live in the operand's storage until publication moves them into the island. Lower coverage is
   accumulated one layer at a time so callers keep their own polygon-copy and cancellation boundaries. */

namespace slic3r_api::Support::Common {
namespace {
// Converts and validates an operand, checks cancellation and moves the result into mutable island demand.
void set_demand_from_operand(const plugin_run_context *context, const LayerIsland &island, ClipperOperand &&operand);
}

// Sums island counts from the given layer index in a borrowed object without changing it; an index beyond its layers returns zero.
uint32_t island_work_count(const Object &object, uint32_t first_layer)
{
    uint32_t count = 0;
    for (uint32_t layer = first_layer; layer < object.layer_count(); ++layer)
        count += object.layer(layer).island_count();
    return count;
}

// Unions a nonempty island-clipped operand with borrowed existing demand and publishes owned geometry; valid mutable-demand context is required, and cancellation/geometry/allocation failures propagate.
void add_to_demand(const plugin_run_context *context, const LayerIsland &island, const ClipperContext &clipper,
                   ClipperOperand &&enforced)
{
    if (enforced.empty())
        return;
    const ExPolygonCollection existing = island.support_demand();
    if (!existing.empty())
        enforced = clipper_union2(clipper(existing), enforced);
    set_demand_from_operand(context, island, std::move(enforced));
}

// Grows borrowed blocker geometry by 1000 scaled epsilons and subtracts it from nonempty island demand, publishing owned geometry; valid mutable-demand context is required, and cancellation/geometry/allocation failures propagate.
void remove_from_demand(const plugin_run_context *context, const LayerIsland &island, const ClipperContext &clipper,
                        ClipperOperand &&blockers)
{
    const ExPolygonCollection existing = island.support_demand();
    if (existing.empty())
        return;
    ClipperOperand blocked = clipper_offset(blockers, 1000. * double(SCALED_EPSILON));
    ClipperOperand remaining = clipper_diff(clipper(existing), blocked);
    set_demand_from_operand(context, island, std::move(remaining));
}

namespace {
// Converts borrowed operand geometry into valid storage-owned polygons, then checks cancellation immediately before moving them into the island; context must permit demand mutation, and failures leave demand unpublished.
void set_demand_from_operand(const plugin_run_context *context, const LayerIsland &island, ClipperOperand &&operand)
{
    StoredExPolygonCollection polygons = operand.to_expolygon_collection();
    polygons.ensure_valid();
    throw_if_cancelled(context);
    expolygons_move(support_demand_get_island_mutable(context, island.handle()), polygons.mutable_handle());
}
}

// Unions borrowed coverage with the lower layer's slices enlarged once by 0.01 mm, returning an operand in clipper storage; no input is modified and geometry/allocation failures propagate.
ClipperOperand accumulate_lower_slices(const ClipperContext &clipper, const ClipperOperand &covered, const Layer &lower)
{
    return clipper_union2(covered, clipper_offset(clipper(lower.slices()), scale_d(0.01)));
}

// Computes a scaled horizontal allowance using the original height/tan rounding; positive thresholds use threshold+1 degrees unless the requested guard rejects it, otherwise half the scaled width is returned without host calls or mutation; unguarded negative allowances assert in scale_to_layer_coord().
coord_t overhang_margin(int32_t threshold, coord_t lower_height, coord_t external_width, bool guard_angle)
{
    if (threshold > 0) {
        const double angle = PI * double(threshold + 1) / 180.;
        if (angle > 0. && (!guard_angle || angle < PI / 2.))
            return scale_to_layer_coord(unscaled(lower_height) / std::tan(angle));
    }
    return external_width / 2;
}

} // namespace slic3r_api::Support::Common
