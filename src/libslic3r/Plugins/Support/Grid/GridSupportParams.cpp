///|/ Copyright (c) Prusa Research 2023 Vojtech Bubnik.
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#include "GridSupportParams.hpp"

#include "libslic3r/Layer.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/PrintRegion.hpp"
#include "libslic3r/Slicing.hpp"

/* This file derives the geometric support settings from the same nozzle,
   height and flow settings as object slicing. Minimum thickness includes actual
   object layers; maximum thickness respects the selected support nozzles.
   Base-interface layers for soluble material on a non-soluble base are omitted. */

namespace Slic3r::SupportGrid {
namespace {
// Resolve the minimum printable nozzle height, including the default and machine Z step.
double min_layer_height_from_nozzle(const PrintConfig &print, uint16_t id);
// Resolve the maximum printable nozzle height, never below the minimum.
double max_layer_height_from_nozzle(const PrintConfig &print, uint16_t id);
}
// Resolve nozzle limits and contact distances for an already sliced object; this instance owns its flows and leaves the object's settings unchanged.
GridSupportParams::GridSupportParams(const PrintObject &object)
{
    const PrintConfig &print = object.print()->config();
    const PrintObjectConfig &config = object.config();
    const std::set<extruder_index_t> extruders = object.object_extruders();
    z_step = print.z_step;
    soluble = config.support_material_contact_distance_type.value == zdNone;
    double min_nozzle = print.nozzle_diameter.get_at(0);
    first_layer_height = config.first_layer_height.value;
    if (config.first_layer_height.percent) {
        first_layer_height = std::numeric_limits<double>::max();
        for (extruder_index_t id : extruders) {
            const double nozzle = print.nozzle_diameter.get_at(id);
            min_nozzle = std::min(min_nozzle, nozzle);
            first_layer_height = std::min(first_layer_height, config.first_layer_height.get_effective_value(nozzle));
        }
    }
    first_layer_height = check_z_step(first_layer_height, z_step);
    const size_t base_id = config.support_material_extruder.value - 1;
    const size_t interface_id = config.support_material_interface_extruder.value - 1;
    const double interface_nozzle = print.nozzle_diameter.get_at(interface_id);
    const double base_max = std::min(max_layer_height_from_nozzle(print, base_id),
        config.support_material_extruder.value > 0 ? print.nozzle_diameter.get_at(base_id) : min_nozzle);
    const double interface_max = std::min(max_layer_height_from_nozzle(print, interface_id),
        config.support_material_interface_extruder.value > 0 ? interface_nozzle : min_nozzle);
    double minimum = 0, maximum = std::numeric_limits<double>::max();
    if (config.support_material_extruder.value > 0) {
        minimum = min_layer_height_from_nozzle(print, base_id);
        maximum = base_max;
    }
    if (config.support_material_interface_extruder.value > 0) {
        minimum = std::max(minimum, min_layer_height_from_nozzle(print, interface_id));
        maximum = std::min(maximum, interface_max);
    }
    max_support_height = maximum == std::numeric_limits<double>::max() ? 0 : std::min(maximum, base_max);
    for (extruder_index_t id : extruders) {
        minimum = std::max(minimum, min_layer_height_from_nozzle(print, id));
        maximum = std::min(maximum, max_layer_height_from_nozzle(print, id));
    }
    if (maximum == std::numeric_limits<double>::max()) maximum = config.layer_height.value;
    if (minimum == 0) minimum = config.layer_height.value;
    minimum = check_z_step(minimum, z_step);
    maximum = check_z_step(maximum, z_step);
    if (max_support_height == 0) max_support_height = maximum;
    max_support_height = check_z_step(max_support_height, z_step);
    layer_height = std::clamp(check_z_step(config.layer_height.value, z_step), minimum, maximum);
    if (!soluble) {
        gap_above = check_z_step(config.support_material_contact_distance.get_effective_value(interface_nozzle), z_step);
        gap_below = check_z_step(config.support_material_bottom_contact_distance.get_effective_value(interface_nozzle), z_step);
        if (gap_below <= 0) gap_below = gap_above;
    }
    const int top = std::max(0, config.support_material_interface_layers.value);
    const int bottom = config.support_material_bottom_interface_layers.is_enabled() ?
        std::max(0, config.support_material_bottom_interface_layers.value) : top;
    top_interfaces = top > 0 ? size_t(top - 1) : 0;
    bottom_interfaces = bottom > 0 ? size_t(bottom - 1) : 0;
    first_flow = support_material_1st_layer_flow(&object, float(first_layer_height));
    base_flow = support_material_flow(&object, float(layer_height));
    interface_flow = support_material_interface_flow(&object, float(layer_height));
    if (top == 0) interface_flow = base_flow;
    resolution = scale_i(print.resolution_internal);
    min_height = std::numeric_limits<coord_t>::max();
    for (size_t id = 0; id < print.min_layer_height.size(); ++id) {
        const coord_t height = scale_to_layer_coord(print.min_layer_height.get_effective_value(print.nozzle_diameter.get_at(id), id));
        if (height > 0) min_height = std::min(min_height, height);
    }
    for (const Layer &layer : object.layers())
        if (layer.scaled_height() > 0) min_height = std::min(min_height, layer.scaled_height());
    double external_width = 0, bridge_ratio = 0;
    for (size_t id = 0; id < object.num_printing_regions(); ++id) {
        const PrintRegion &region = object.printing_region(id);
        external_width = std::max(external_width, double(region.flow(object, frExternalPerimeter, layer_height, 2).width()));
        bridge_ratio += region.config().bridge_flow_ratio.get_effective_value(1.);
    }
    gap_xy = scale_to_layer_coord(config.support_material_xy_spacing.get_effective_value(external_width));
    bridge_ratio /= object.num_printing_regions();
    bottom_flow = soluble ? interface_flow.with_flow_ratio(bridge_ratio) :
        Flow::bridging_flow(std::sqrt(bridge_ratio) * interface_flow.nozzle_diameter(), interface_flow.nozzle_diameter());
    bottom_expansion = boost::any_cast<FloatOrPercent>(config.support_material_bottom_interface_expansion.get_any());
    interface_density = std::min(1., interface_flow.spacing() / (config.support_material_interface_spacing.value + interface_flow.spacing()));
    if (top == 0)
        interface_density = std::min(1., base_flow.spacing() / (config.support_material_spacing.value + base_flow.spacing()));
}
namespace {
// Resolve percent heights against the nozzle; zero uses the slicer's minimum default and the result is rounded to the machine Z step.
double min_layer_height_from_nozzle(const PrintConfig &print, uint16_t id)
{
    const double height = print.min_layer_height.get_effective_value(print.nozzle_diameter.get_at(id), id);
    return check_z_step(height == 0 ? 0.07 : std::max(0.01, height), print.z_step);
}
// Resolve percent heights against the nozzle; a disabled or zero limit uses three quarters of its diameter and respects the configured minimum.
double max_layer_height_from_nozzle(const PrintConfig &print, uint16_t id)
{
    const double minimum = min_layer_height_from_nozzle(print, id);
    const double nozzle = print.nozzle_diameter.get_at(id);
    const double maximum = print.max_layer_height.get_effective_value(nozzle, id);
    return check_z_step(std::max(minimum, maximum == 0 || !print.max_layer_height.is_enabled() ? 0.75 * nozzle : maximum), print.z_step);
}
}
}
