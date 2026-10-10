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

#include "libslic3r/Flow.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/PrintRegion.hpp"
#include "libslic3r/Slicing.hpp"
#include "libslic3r/Plugins/Support/Common/SupportParams.hpp"
#include "libslic3r/Plugins/Support/Common/SupportZPlacement.hpp"

/* Derives the tree branch settings of one object from its configuration and from the shared support parameters.

   The constructor runs once per object. It reads the object and print configurations for the branch angles, radii and
   distances, and the support parameters for the Z gaps, the minimum layer height and the support extrusion width.
   Then it computes the maximum move of every object layer from the effective branch angle and the real layer height.
*/

namespace Slic3r::TreeFamily {

namespace {
// Fixed by the legacy tree support, which does not read these settings from the configuration.
constexpr double bp_diameter_mm = 7.5;
constexpr double min_height_to_model_mm = 1.0;
constexpr double max_diameter_increase_by_merges_mm = 1.0;

// A slab height setting as the grid reads it: 0 means the maximum support height, a percent is of the nozzle, and the maximum caps it.
double slab_height_mm(const ConfigOptionFloatOrPercent &setting, double nozzle, double max_support_height)
{
    if (setting.value == 0) return max_support_height;
    const double effective = setting.percent ? nozzle * setting.value / 100. : setting.value;
    return std::min(max_support_height, effective);
}

// Maximum move of a branch centre over a layer of the given height, for the branch angle; the cap keeps steep angles inside coord_t.
coord_t maximum_move(const double angle, const coord_t layer_height)
{
    constexpr double cap = double(std::numeric_limits<coord_t>::max() / 4);
    return coord_t(std::min(std::tan(angle) * double(layer_height), cap));
}
}

TreeSettings::TreeSettings(const PrintObject &object, const slic3r_api::Support::Common::SupportParams &params)
{
    const PrintConfig &print_config = object.print()->config();
    const PrintObjectConfig &config = object.config();
    const double layer_height_mm = check_z_step(config.layer_height.value, print_config.z_step);
    double external_perimeter_width = 0.;
    for (size_t region_id = 0; region_id < object.num_printing_regions(); ++ region_id) {
        const PrintRegion &region = object.printing_region(region_id);
        external_perimeter_width = std::max<double>(external_perimeter_width, region.flow(object, frExternalPerimeter, layer_height_mm, 2).width());
    }

    layer_height = scale_to_layer_coord(layer_height_mm);
    resolution = scale_to_layer_coord(print_config.resolution_internal.value);
    min_feature_size = scale_to_layer_coord(external_perimeter_width * 0.25);
    const Flow support_flow = support_material_flow(&object, float(layer_height_mm));
    support_line_width = support_flow.scaled_width();
    support_rests_on_model = ! config.support_material_buildplate_only;
    xy_distance = scale_i(config.support_material_xy_spacing.get_effective_value(external_perimeter_width));
    xy_min_distance = std::min(xy_distance, scale_i(0.5 * external_perimeter_width));
    if (params.soluble) {
        // A soluble support touches the object, so its XY distance is its only separation: the safe offset steps by it and needs it above zero.
        xy_min_distance = std::max(xy_min_distance, scale_i(0.1));
        xy_distance = std::max(xy_distance, xy_min_distance);
    }
    z_distance_top_layers = size_t(std::round(params.gap_above / layer_height_mm));
    z_distance_bottom_layers = size_t(std::round(params.gap_below / layer_height_mm));

    branch_distance = scale_i(config.support_tree_branch_distance.value);
    top_rate = config.support_tree_top_rate.value;
    const coord_t branch_diameter = scale_i(config.support_tree_branch_diameter.value);
    branch_radius = branch_diameter / 2;
    min_radius = std::clamp(scale_i(config.support_tree_tip_diameter.value), coord_t(0), branch_diameter) / 2;
    const double diameter_angle = std::clamp<double>(config.support_tree_branch_diameter_angle * M_PI / 180., 0., 0.5 * M_PI - EPSILON);
    branch_radius_increase_per_layer = std::tan(diameter_angle) * layer_height;
    // Ensure lines always stack nicely even if layer height is large.
    tip_layers = size_t(std::max<coord_t>(1, std::max((branch_radius - min_radius) / (support_line_width / 3), branch_radius / layer_height)));
    increase_radius_until_radius = branch_radius;
    increase_radius_until_layer = tip_layers;

    bp_radius = scale_i(bp_diameter_mm) / 2;
    // Increase by half a line overlap, but not faster than 40 degrees angle (0 degrees means zero increase in radius).
    bp_radius_increase_per_layer = std::min(std::tan(0.7) * layer_height, 0.5 * support_line_width);
    layer_start_bp_radius = LayerIndex((bp_radius - branch_radius) / bp_radius_increase_per_layer);
    min_dtt_to_model = size_t(round_up_divide(scale_i(min_height_to_model_mm), layer_height));
    max_to_model_radius_increase = scale_i(max_diameter_increase_by_merges_mm) / 2;

    // The overhang cap uses the support extrusion width for percentages and the minimum layer height of the support.
    const double angle_limit = std::clamp<double>(config.support_tree_angle * M_PI / 180., 0., 0.5 * M_PI - EPSILON);
    const double angle_slow_limit = std::clamp<double>(config.support_tree_angle_slow * M_PI / 180., 0., angle_limit - EPSILON);
    const double support_width_mm = support_flow.width();
    const ConfigOptionFloatOrPercent &max_overhang_setting = config.support_tree_max_overhang;
    const double max_overhang_mm = max_overhang_setting.percent ? max_overhang_setting.value * support_width_mm / 100. : max_overhang_setting.value;
    const double overhang_angle = std::atan2(max_overhang_mm, params.support_min_height);
    angle = std::min(angle_limit, overhang_angle);
    const double angle_slow = std::min(angle_slow_limit, overhang_angle);

    max_overhang = scale_i(max_overhang_mm);
    z_step = scale_to_layer_coord(params.z_step);
    min_slab_height = scale_to_layer_coord(params.support_min_height);
    max_slab_height = scale_to_layer_coord(slab_height_mm(config.support_material_layer_height, params.base_flow_mm.nozzle_diameter, params.max_support_height));
    dense_slab_height = slic3r_api::Support::Common::handed_over_dense_height(
        scale_to_layer_coord(slab_height_mm(config.support_material_interface_layer_height, params.interface_flow_mm.nozzle_diameter, params.max_support_height)),
        min_slab_height, z_step);

    for (size_t layer_idx = 0; layer_idx < object.layer_count(); ++ layer_idx) {
        const coord_t height = object.layer(layer_idx).scaled_height();
        max_move.push_back(maximum_move(angle, height));
        max_move_slow.push_back(maximum_move(angle_slow, height));
    }
}

coord_t TreeSettings::getRadius(size_t distance_to_top, const double elephant_foot_increases) const
{
    return (distance_to_top <= tip_layers ? min_radius + (branch_radius - min_radius) * distance_to_top / tip_layers : // tip
                   branch_radius + // base
                   (distance_to_top - tip_layers) * branch_radius_increase_per_layer)
           + // gradual increase
           elephant_foot_increases * (std::max(bp_radius_increase_per_layer - branch_radius_increase_per_layer, 0.0));
}

coord_t TreeSettings::recommendedMinRadius(LayerIndex layer_idx) const
{
    double num_layers_widened = layer_start_bp_radius - layer_idx;
    return num_layers_widened > 0 ? branch_radius + num_layers_widened * bp_radius_increase_per_layer : 0;
}

} // namespace Slic3r::TreeFamily
