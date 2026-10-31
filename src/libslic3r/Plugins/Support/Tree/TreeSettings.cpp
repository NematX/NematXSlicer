///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/*
Tree supports by Thomas Rahm, losely based on Tree Supports by CuraEngine.
   Original source of Thomas Rahm's tree supports:
   https://github.com/ThomasRahm/CuraEngine
   Original CuraEngine copyright:
   Copyright (c) 2021 Ultimaker B.V.
   CuraEngine is released under the terms of the AGPLv3 or higher. */
#include "TreeSettings.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Plugins/Support/Common/SupportParams.hpp"
#include "libslic3r/Plugins/Support/Common/SupportZPlacement.hpp"

/* Derives the tree branch settings of one object from its configuration and from the shared support parameters.

   The constructor runs once per object. It reads the object configuration for branch angles, radii and distances,
   and the support parameters for Z gaps, minimum layer height, support flows, printed gap_xy and XY resolution;
   it does not read the print view.
   Then it computes the maximum move from each real layer height and the collision windows from the real layer Z.
   A window includes object layers closer than the gap, allowing EPSILON for Z rounding; its top extends virtually
   at the last layer height so the collision's XY ramp has its full physical extent near the top of the object.
   It also stores the real top of every object layer in scaled units (layer_print_z) and its unscaled millimeter value
   (layer_print_z_mm), shared by node placement and organic smoothing. reaches_min_height_to_model and
   lowest_layer_within read to measure the minimum height to the model and the root search depth in real Z. With top interface layers every
   branch is exempt from that minimum height (branches_carry_roof: the interface stack it carries plays the role of the legacy roof).
   The radius gain of a branch per layer step (tip_gain, cone_gain) comes from the real height of the upper layer of the
   step. advance() applies one step to a gain; path_gain() gives the gain of a branch that grew at every step since its tip
   in O(1), from prefix sums of the tip gains and from the layer where each possible tip ends.
   The foot near the build plate has its own per-layer data: bp_gain[L], the widening allowed on the step whose upper layer is
   L, and foot_target[L], the radius a branch resting on the bed should have on layer L: branch_radius plus the remainder of
   (bp_radius - branch_radius) after the bp_gain of layers 1..L (the foot cone anchored at layer 0), never below branch_radius.
   foot_cache_step, half the diameter of the largest support nozzle, is the spacing of the cache radii in the band where the branches
   widen into their foot (see TreeVolumes).
   Printed XY clearance is the shared gap_xy without a floor. Only safe_step_distance, used for collision-checked routing
   increments, retains the soluble 0.1 mm minimum so a zero printed clearance does not stop offset propagation.
   At zero printed clearance, safe offsets can cross walls thinner than that routing step (about 0.1 mm).
*/

namespace Slic3r::TreeFamily {

namespace {
// Fixed by the legacy tree support, which does not read these settings from the configuration.
constexpr double bp_diameter_mm = 7.5;
constexpr double min_height_to_model_mm = 1.0;
constexpr double max_diameter_increase_by_merges_mm = 1.0;

// Maximum move of a branch centre over a layer of the given height, for the branch angle; the cap keeps steep angles inside coord_t.
coord_t maximum_move(const double angle, const coord_t layer_height);

coord_t maximum_move(const double angle, const coord_t layer_height)
{
    constexpr double cap = double(std::numeric_limits<coord_t>::max() / 4);
    return coord_t(std::min(std::tan(angle) * double(layer_height), cap));
}
}

// Object options use Config views; shared printed XY clearance and XY resolution come from SupportParams. Flows come from the
// host (support_flow, region_flow_mm), which computes them like the legacy Flow functions. The views are borrowed; the constructor
// copies shared printed XY clearance and stores a separate routing step allowance, real-Z collision windows, per-layer moves
// and the real layer tops (layer_print_z, read by reaches_min_height_to_model and lowest_layer_within).
// Flow lookups return zero flows on failure; configuration/view and allocation failures propagate to the caller.
TreeSettings::TreeSettings(const slic3r_api::Object &object, const slic3r_api::Print &, const slic3r_api::Support::Common::SupportParams &params)
{
    const slic3r_api::Config config = object.config();
    const double layer_height_mm = slic3r_api::Support::Common::check_z_step_mm(config.get("layer_height").get_float(), params.z_step);
    // The routing feature size uses the widest external perimeter at the tree's nominal layer height.
    double external_perimeter_width = 0.;
    for (uint32_t region_id = 0; region_id < object.print_region_count(); ++ region_id)
        external_perimeter_width = std::max(external_perimeter_width,
            object.region_flow_mm(region_id, raw_extrusion_role(RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER), layer_height_mm, 2).width);

    resolution = params.resolution;
    min_feature_size = scale_i(external_perimeter_width * 0.25);
    const raw_extrusion_role support_role = raw_extrusion_role(RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL);
    support_line_width = object.support_flow(support_role, layer_height_mm).width;
    support_rests_on_model = ! config.get("support_material_buildplate_only").get_bool();
    // The same option as tip_sources(): the interface layers under which the tips start.
    branches_carry_roof = params.top_interfaces_enabled;
    xy_distance = params.gap_xy;
    // Only the internal offset step keeps the soluble minimum; it does not enlarge collision or printed keep-out areas.
    safe_step_distance = params.soluble ? std::max(xy_distance, scale_i(0.1)) : xy_distance;
    const coord_t gap_above = scale_to_layer_coord(params.gap_above);
    const coord_t gap_below = scale_to_layer_coord(params.gap_below);
    const coord_t eps = scale_to_layer_coord(EPSILON);
    // A gap within the Z rounding tolerance is zero, so true means both neighbouring layers are in the windows.
    z_distance_both_sides = gap_above > eps && gap_below > eps;

    branch_distance = scale_i(config.get("support_tree_branch_distance").get_float());
    // A percent option: the value itself (15 for 15%), which get_float() would divide by 100.
    top_rate = config.get("support_tree_top_rate").get_effective_value(100.);
    const coord_t branch_diameter = scale_i(config.get("support_tree_branch_diameter").get_float());
    branch_radius = branch_diameter / 2;
    min_radius = std::clamp(scale_i(config.get("support_tree_tip_diameter").get_float()), coord_t(0), branch_diameter) / 2;
    const double diameter_angle = std::clamp<double>(config.get("support_tree_branch_diameter_angle").get_float() * PI / 180., 0., 0.5 * PI - EPSILON);
    cone_slope = std::tan(diameter_angle);
    tip_total_gain = double(branch_radius - min_radius);

    bp_radius = scale_i(bp_diameter_mm) / 2;
    foot_cache_step = scale_i(0.5 * std::max(params.base_flow_mm.nozzle_diameter, params.interface_flow_mm.nozzle_diameter));
    min_height_to_model = scale_i(min_height_to_model_mm);
    max_to_model_radius_increase = scale_i(max_diameter_increase_by_merges_mm) / 2;

    // The overhang cap uses the support extrusion width for percentages and the minimum layer height of the support.
    const double angle_limit = std::clamp<double>(config.get("support_tree_angle").get_float() * PI / 180., 0., 0.5 * PI - EPSILON);
    // The plugin bounds the slow angle to [0, max(0, angle_limit - EPSILON)]: EPSILON keeps it below the branch angle,
    // and the upper bound falls to zero when the branch angle is below EPSILON, including zero.
    const double angle_slow_limit = std::clamp<double>(config.get("support_tree_angle_slow").get_float() * PI / 180., 0., std::max(0., angle_limit - EPSILON));
    const double support_width_mm = object.support_flow_mm(support_role, layer_height_mm).width;
    const c_float_or_percent max_overhang_setting = config.get("support_tree_max_overhang").get_float_or_percent();
    const double max_overhang_mm = max_overhang_setting.percent ? max_overhang_setting.value * support_width_mm / 100. : max_overhang_setting.value;
    const double overhang_angle = std::atan2(max_overhang_mm, params.support_min_height);
    angle = std::min(angle_limit, overhang_angle);
    const double angle_slow = std::min(angle_slow_limit, overhang_angle);

    max_overhang = scale_i(max_overhang_mm);
    z_step = scale_to_layer_coord(params.z_step);
    min_slab_height = scale_to_layer_coord(params.support_min_height);
    max_slab_height = slic3r_api::Support::Common::slab_step(config.get("support_material_layer_height").get_float_or_percent(), params.base_flow_mm.nozzle_diameter, params.max_support_height);
    dense_slab_height = slic3r_api::Support::Common::handed_over_dense_height(
        slic3r_api::Support::Common::slab_step(config.get("support_material_interface_layer_height").get_float_or_percent(), params.interface_flow_mm.nozzle_diameter, params.max_support_height),
        min_slab_height, z_step);

    // Monotone window ends visit each real layer once. Above the object, equal-height virtual layers keep the XY ramp complete.
    uint32_t top_end = 0, bottom_begin = 0;
    for (uint32_t layer_idx = 0; layer_idx < object.layer_count(); ++ layer_idx) {
        const coord_t height = object.layer(layer_idx).height();
        layer_print_z.push_back(object.layer(layer_idx).print_z());
        layer_print_z_mm.push_back(unscaled(layer_print_z.back()));
        max_move.push_back(maximum_move(angle, height));
        max_move_slow.push_back(maximum_move(angle_slow, height));
        const coord_t top_limit = object.layer(layer_idx).print_z() + gap_above - eps;
        top_end = std::max(top_end, layer_idx + 1);
        while (top_end < object.layer_count() && object.layer(top_end).bottom_z() < top_limit)
            ++top_end;
        uint32_t top_count = top_end - layer_idx - 1;
        const coord_t virtual_span = top_limit - object.layer(object.layer_count() - 1).print_z();
        if (virtual_span > 0) {
            const coord_t last_height = object.layer(object.layer_count() - 1).height();
            top_count += uint32_t((virtual_span + last_height - 1) / last_height);
        }
        z_distance_top_layers.push_back(top_count);
        const coord_t bottom_limit = object.layer(layer_idx).bottom_z() - gap_below + eps;
        while (bottom_begin < layer_idx && object.layer(bottom_begin).print_z() <= bottom_limit)
            ++bottom_begin;
        z_distance_bottom_layers.push_back(layer_idx - bottom_begin);
    }

    // The radius gains of the steps, indexed by the upper layer of the step. Lines always stack nicely: a layer deposits at most a
    // third of a line width of radius, and the tip is a cone of height branch_radius.
    const size_t layer_count = layer_print_z.size();
    tip_gain.reserve(layer_count);
    cone_gain.reserve(layer_count);
    bp_gain.reserve(layer_count);
    foot_target.reserve(layer_count);
    tip_gain_prefix.assign(1, 0.);
    // Foot: increase by half a line overlap per layer, but not faster than 40 degrees angle. The cone is anchored on the top of
    // layer 0 (the height of layer 0 does not enter): bp_radius there, and the radius decreases by bp_gain going up.
    double foot_left = std::max(0., double(bp_radius - branch_radius));
    const double foot_slope = std::tan(0.7);
    for (size_t layer_idx = 0; layer_idx < layer_count; ++ layer_idx) {
        const double height = double(layer_print_z[layer_idx] - (layer_idx > 0 ? layer_print_z[layer_idx - 1] : 0));
        tip_gain.push_back(branch_radius > 0 ? std::min(double(support_line_width) / 3., height * tip_total_gain / double(branch_radius)) : 0.);
        cone_gain.push_back(cone_slope * height);
        tip_gain_prefix.push_back(tip_gain_prefix.back() + tip_gain.back());
        bp_gain.push_back(layer_idx > 0 ? std::min(foot_slope * height, 0.5 * double(support_line_width)) : 0.);
        foot_left = std::max(0., foot_left - bp_gain.back());
        foot_target.push_back(double(branch_radius) + foot_left);
    }
    // The layer at which the tip of each tip layer T ends: the largest E < T such that the tip gains of the steps E + 1 .. T reach
    // tip_total_gain (within 1 unit). These sums decrease when E grows, so a binary search finds E.
    const double tip_end_threshold = tip_total_gain - 1.;
    tip_end_layer.assign(layer_count, -1);
    for (LayerIndex tip_layer = 0; tip_layer < LayerIndex(layer_count); ++ tip_layer) {
        if (tip_end_threshold <= 0.) {
            tip_end_layer[tip_layer] = tip_layer;
            continue;
        }
        LayerIndex low = 0, high = tip_layer - 1;
        while (low <= high) {
            const LayerIndex mid = (low + high) / 2;
            if (tip_gain_prefix[tip_layer + 1] - tip_gain_prefix[mid + 1] >= tip_end_threshold) {
                tip_end_layer[tip_layer] = mid;
                low = mid + 1;
            } else
                high = mid - 1;
        }
    }
}

// The tip is on layer layer_idx + distance_to_top; eps keeps an exact distance whose Z were rounded one unit short.
bool TreeSettings::reaches_min_height_to_model(LayerIndex layer_idx, size_t distance_to_top) const
{
    const size_t tip = size_t(layer_idx) + distance_to_top;
    assert(tip < layer_print_z.size());
    return layer_print_z[tip] - layer_print_z[layer_idx] + scale_to_layer_coord(EPSILON) >= min_height_to_model;
}

// The layer tops increase with the index, so the walk goes down until the distance exceeds depth.
LayerIndex TreeSettings::lowest_layer_within(LayerIndex layer_idx, coord_t depth) const
{
    const coord_t limit = layer_print_z[layer_idx] - depth - scale_to_layer_coord(EPSILON);
    LayerIndex lowest = layer_idx;
    while (lowest > 0 && layer_print_z[lowest - 1] >= limit)
        --lowest;
    return lowest;
}

// A tip within 1 unit of the end counts as ended, the same convention as the tip end layers of the constructor.
double TreeSettings::advance(const double gain, const LayerIndex upper_layer) const
{
    assert(upper_layer > 0 && size_t(upper_layer) < tip_gain.size());
    if (gain >= tip_total_gain - 1.)
        return std::max(gain, tip_total_gain) + cone_gain[upper_layer];
    const double grown = gain + tip_gain[upper_layer];
    return grown >= tip_total_gain - 1. ? tip_total_gain : grown;
}

// Above the tip end layer the gain is a sum of tip gains; from it down, the tip total plus the cone over the height between the
// end layer and layer_idx (the sum of the cone gains of the steps in between).
double TreeSettings::path_gain(const LayerIndex layer_idx, const size_t distance_to_top) const
{
    assert(layer_idx >= 0);
    if (distance_to_top == 0)
        return 0.;
    const size_t tip_layer = size_t(layer_idx) + distance_to_top;
    assert(tip_layer < layer_print_z.size());
    const LayerIndex end_layer = tip_end_layer[tip_layer];
    if (layer_idx > end_layer)
        return tip_gain_prefix[tip_layer + 1] - tip_gain_prefix[layer_idx + 1];
    return tip_total_gain + cone_slope * double(layer_print_z[end_layer] - layer_print_z[layer_idx]);
}

} // namespace Slic3r::TreeFamily
