///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#include "SupportParams.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

#include "libslic3r/Api/plugin/cpp/PrintHelpers.hpp"

/* Implementation of SupportParams (see SupportParams.hpp).

   The constructor follows the native geometry derivation (NativeGridParams in
   tests/plugins/test_support_common.cpp) line by line, so a reader can compare
   both: the same order, the same fallbacks and the same rounding. The differences
   are only in how values are read:
   - options through Config views, enums by their serialized value;
   - an extruder index of -1 (configured extruder 0, "current extruder") is
     passed as an out-of-range index, which the host resolves to the first
     vector item exactly like the native get_at();
   - flows through the host twice: object_get_support_flow / object_get_region_flow
     give the scaled integer coordinates (c_flow), and their _mm twins give the
     host's millimeter values; values the native code derives from a flow width
     or spacing in millimeters (densities, the XY gap) read the _mm values, so
     they are exact.
   The path fields reproduce the PathParams of the legacy toolpaths. */

namespace slic3r_api::Support::Common {
namespace {

// Native check_z_step() on millimeters: round val to the nearest multiple of z_step, at 1e-8 mm.
double check_z_step_mm(double val, double z_step);
// Nozzle diameter of a zero-based extruder; an index past the end reads the first nozzle.
double nozzle_diameter(const Config &print_config, uint32_t extruder_idx);
// Smallest layer height of an extruder: the configured minimum, 0.07 mm when unset, rounded to z_step.
double min_layer_height_from_nozzle(const Config &print_config, uint32_t extruder_idx, double z_step);
// Largest layer height of an extruder: the configured maximum, 3/4 of the nozzle when unset, never below the minimum.
double max_layer_height_from_nozzle(const Config &print_config, uint32_t extruder_idx, double z_step);
// Zero-based index of a one-based extruder setting; 0 ("current") becomes an out-of-range index.
uint32_t extruder_index(int32_t one_based);
// Convert degrees to radians in float, as Geometry::deg2rad<float>().
float deg2rad_float(float degrees);

double check_z_step_mm(double val, double z_step)
{
    if (z_step <= EPSILON)
        return val;
    const uint64_t valint = uint64_t(val * 100000000. + 0.5);
    const uint64_t stepint = uint64_t(z_step * 100000000. + 0.5);
    return (((valint + (stepint / 2)) / stepint) * stepint) / 100000000.;
}

double nozzle_diameter(const Config &print_config, uint32_t extruder_idx)
{
    return print_config.get("nozzle_diameter").get_float(extruder_idx);
}

double min_layer_height_from_nozzle(const Config &print_config, uint32_t extruder_idx, double z_step)
{
    const double height = print_config.get("min_layer_height")
                              .get_effective_value(nozzle_diameter(print_config, extruder_idx), extruder_idx);
    return check_z_step_mm(height == 0 ? 0.07 : std::max(0.01, height), z_step);
}

double max_layer_height_from_nozzle(const Config &print_config, uint32_t extruder_idx, double z_step)
{
    const double minimum = min_layer_height_from_nozzle(print_config, extruder_idx, z_step);
    const double nozzle = nozzle_diameter(print_config, extruder_idx);
    const ConfigOption max_layer_height = print_config.get("max_layer_height");
    const double maximum = max_layer_height.get_effective_value(nozzle, extruder_idx);
    // Index -1 asks whether the option as a whole is enabled, like the native is_enabled().
    const bool enabled = config_option_is_enabled(max_layer_height.handle(), -1) != 0;
    return check_z_step_mm(std::max(minimum, maximum == 0 || !enabled ? 0.75 * nozzle : maximum), z_step);
}

uint32_t extruder_index(int32_t one_based)
{
    return one_based > 0 ? uint32_t(one_based - 1) : std::numeric_limits<uint32_t>::max();
}

float deg2rad_float(float degrees)
{
    return float(PI) * degrees / float(180.0);
}

} // namespace

SupportParams::SupportParams(const Print &print, const Object &object)
{
    const Config print_config = print.config();
    const Config config = object.config();
    const std::set<extruder_index_t> extruders = object_extruders(print, object);
    z_step = print_config.get("z_step").get_float();
    style = support_style(config);
    distance_type = contact_distance_type(config);
    soluble = distance_type == ContactDistanceType::None;

    // First layer: a percent applies to every nozzle the object uses and the smallest result wins.
    double min_nozzle = nozzle_diameter(print_config, 0);
    const ConfigOption first_layer_option = config.get("first_layer_height");
    first_layer_height = first_layer_option.get_float();
    if (first_layer_option.is_percent()) {
        first_layer_height = std::numeric_limits<double>::max();
        for (extruder_index_t id : extruders) {
            const double nozzle = nozzle_diameter(print_config, uint32_t(id));
            min_nozzle = std::min(min_nozzle, nozzle);
            first_layer_height = std::min(first_layer_height, first_layer_option.get_effective_value(nozzle));
        }
    }
    first_layer_height = check_z_step_mm(first_layer_height, z_step);

    // Layer limits of the support tools and of the object tools.
    const int32_t base_extruder = config.get("support_material_extruder").get_int();
    const int32_t interface_extruder = config.get("support_material_interface_extruder").get_int();
    const uint32_t base_id = extruder_index(base_extruder);
    const uint32_t interface_id = extruder_index(interface_extruder);
    const double interface_nozzle = nozzle_diameter(print_config, interface_id);
    const double base_max = std::min(max_layer_height_from_nozzle(print_config, base_id, z_step),
                                     base_extruder > 0 ? nozzle_diameter(print_config, base_id) : min_nozzle);
    const double interface_max = std::min(max_layer_height_from_nozzle(print_config, interface_id, z_step),
                                          interface_extruder > 0 ? interface_nozzle : min_nozzle);
    double minimum = 0, maximum = std::numeric_limits<double>::max();
    if (base_extruder > 0) {
        minimum = min_layer_height_from_nozzle(print_config, base_id, z_step);
        maximum = base_max;
    }
    if (interface_extruder > 0) {
        minimum = std::max(minimum, min_layer_height_from_nozzle(print_config, interface_id, z_step));
        maximum = std::min(maximum, interface_max);
    }
    max_support_height = maximum == std::numeric_limits<double>::max() ? 0 : std::min(maximum, base_max);
    for (extruder_index_t id : extruders) {
        minimum = std::max(minimum, min_layer_height_from_nozzle(print_config, uint32_t(id), z_step));
        maximum = std::min(maximum, max_layer_height_from_nozzle(print_config, uint32_t(id), z_step));
    }
    const double object_layer_height = config.get("layer_height").get_float();
    if (maximum == std::numeric_limits<double>::max()) maximum = object_layer_height;
    if (minimum == 0) minimum = object_layer_height;
    minimum = check_z_step_mm(minimum, z_step);
    maximum = check_z_step_mm(maximum, z_step);
    if (max_support_height == 0) max_support_height = maximum;
    max_support_height = check_z_step_mm(max_support_height, z_step);
    layer_height = std::clamp(check_z_step_mm(object_layer_height, z_step), minimum, maximum);

    // Vertical gaps: soluble support touches the object.
    if (!soluble) {
        gap_above = check_z_step_mm(config.get("support_material_contact_distance").get_effective_value(interface_nozzle), z_step);
        gap_below = check_z_step_mm(config.get("support_material_bottom_contact_distance").get_effective_value(interface_nozzle), z_step);
        if (gap_below <= 0) gap_below = gap_above;
    }

    // Interface counts exclude the contact layer; a disabled bottom count follows the top one.
    const int32_t top = std::max(0, config.get("support_material_interface_layers").get_int());
    const ConfigOption bottom_option = config.get("support_material_bottom_interface_layers");
    const int32_t bottom = config_option_is_enabled(bottom_option.handle(), -1) != 0 ?
        std::max(0, bottom_option.get_int()) : top;
    top_interfaces = top > 0 ? size_t(top - 1) : 0;
    bottom_interfaces = bottom > 0 ? size_t(bottom - 1) : 0;

    // Flows at the nominal heights; without interface layers the interface uses the base flow.
    const raw_extrusion_role base_role = raw_extrusion_role(RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL);
    const raw_extrusion_role interface_role = top == 0 ? base_role : raw_extrusion_role(RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL_INTERFACE);
    first_flow = object.support_flow(base_role, first_layer_height, 0., OBJECT_SUPPORT_FLOW_FIRST_LAYER);
    base_flow = object.support_flow(base_role, layer_height);
    interface_flow = object.support_flow(interface_role, layer_height);
    bottom_flow = object.support_flow(interface_role, layer_height, 0., OBJECT_SUPPORT_FLOW_BOTTOM_CONTACT);
    first_flow_mm = object.support_flow_mm(base_role, first_layer_height, 0., OBJECT_SUPPORT_FLOW_FIRST_LAYER);
    base_flow_mm = object.support_flow_mm(base_role, layer_height);
    interface_flow_mm = object.support_flow_mm(interface_role, layer_height);
    bottom_flow_mm = object.support_flow_mm(interface_role, layer_height, 0., OBJECT_SUPPORT_FLOW_BOTTOM_CONTACT);
    resolution = scale_i(print_config.get("resolution_internal").get_float());

    // Thinnest printable layer among the configured minimums and the sliced object layers.
    min_height = std::numeric_limits<coord_t>::max();
    const ConfigOption min_layer_height = print_config.get("min_layer_height");
    for (uint32_t id = 0; id < min_layer_height.size(); ++id) {
        const coord_t height = scale_to_layer_coord(min_layer_height.get_effective_value(nozzle_diameter(print_config, id), id));
        if (height > 0) min_height = std::min(min_height, height);
    }
    for (uint32_t layer_idx = 0; layer_idx < object.layer_count(); ++layer_idx) {
        const coord_t height = object.layer(layer_idx).height();
        if (height > 0) min_height = std::min(min_height, height);
    }

    // XY gap: a percent applies to the widest external perimeter of the object regions.
    double external_width = 0;
    for (uint32_t region_idx = 0; region_idx < object.print_region_count(); ++region_idx)
        external_width = std::max(external_width, object.region_flow_mm(region_idx, raw_extrusion_role(RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER),
                                                                        layer_height, 2).width);
    gap_xy = scale_to_layer_coord(config.get("support_material_xy_spacing").get_effective_value(external_width));
    bottom_expansion = config.get("support_material_bottom_interface_expansion").get_float_or_percent();

    // Line densities from the configured gaps between lines.
    const double interface_spacing = interface_flow_mm.spacing;
    const double base_spacing = base_flow_mm.spacing;
    const double base_gap = config.get("support_material_spacing").get_float();
    interface_density = std::min(1., interface_spacing / (config.get("support_material_interface_spacing").get_float() + interface_spacing));
    if (top == 0)
        interface_density = std::min(1., base_spacing / (base_gap + base_spacing));

    // Path settings of the legacy toolpaths.
    base_pattern = Common::base_pattern(config);
    base_density = std::min(1., base_spacing / (base_gap + base_spacing));
    with_sheath = config.get("support_material_with_sheath").get_bool();
    interface_contact_loops = config.get("support_material_interface_contact_loops").get_bool();
    base_pattern_id = base_pattern_plugin_id(base_pattern, base_density, with_sheath);
    top_pattern_id = top_interface_pattern_plugin_id(config.get("support_material_top_interface_pattern").serialize(),
                                                     soluble, interface_density);
    bottom_pattern_id = bottom_interface_pattern_plugin_id(config.get("support_material_bottom_interface_pattern").serialize(),
                                                           top_pattern_id, soluble, interface_density);
    base_angle = deg2rad_float(float(config.get("support_material_angle").get_float()));
    interface_angle = deg2rad_float(float(config.get("support_material_interface_angle").get_float()));
    angle_height = config.get("support_material_angle_height").get_float();
    interface_angle_increment = config.get("support_material_interface_angle_increment").get_float();
    raft_first_layer_density = config.get("raft_first_layer_density").get_effective_value(1.);
}

} // namespace slic3r_api::Support::Common
