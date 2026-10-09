///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "PrintHelpers.hpp"
#include "PrintingPlanViews.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

/*
Print helper implementations
============================

These functions reproduce small derived calculations from the legacy Print,
PrintObject, and Extruder classes without making those host-only classes part
of the plugin API. They are compatibility helpers for plugin algorithms, not a
second source of persistent print state.

The implementation covers five related tasks:

    |-- normalize configured one-based extruder ids to zero-based ids
    |-- collect object, support, and skirt extruders
    |-- resolve first-layer heights from absolute or nozzle-relative settings
    |-- convert volumetric extrusion into E-axis distance per millimeter
    `-- derive the actual height of one finalized plan layer group

Configuration lengths and areas arrive in user-facing millimeters, while
`coord_t` and `c_flow` geometry values use scaled internal coordinates. The
helpers perform the conversion at their boundary and return scaled values when
the result is a coordinate. Extruder ids returned by this file are zero-based,
even though the corresponding user settings are one-based.

The fallback behavior is intentional: an invalid or unavailable configured
extruder resolves to tool zero, an object with no detected extruder falls back
to tool zero for skirt work, and a missing nozzle value uses 0.4 mm when a
first-layer percentage must be evaluated. `get_min_first_layer_height()` and
other functions that require actual objects throw `std::runtime_error` when
the input cannot provide a meaningful result. `check_z_step()` leaves its
input unchanged when the requested Z step is disabled or too small.
*/

namespace slic3r_api {
namespace {

extruder_index_t normalize_extruder_id(const Config &print_config, int32_t one_based_id)
{
    const ConfigOption nozzle_diameter = print_config.get("nozzle_diameter");
    const uint32_t num_extruders = nozzle_diameter.size();
    const int32_t idx = std::max<int32_t>(0, one_based_id - 1);
    const uint32_t normalized = (num_extruders == 0 || uint32_t(idx) >= num_extruders) ? 0 : uint32_t(idx);
    if (normalized > uint32_t(std::numeric_limits<extruder_index_t>::max()))
        throw std::invalid_argument("An object extruder index exceeds extruder_index_t.");
    return static_cast<extruder_index_t>(normalized);
}

void append_support_extruders(const Print &print,
                              const std::vector<Object> &objects,
                              std::set<extruder_index_t> &extruders)
{
    const Config print_config = print.config();
    bool support_uses_current_extruder = false;

    for (Object object : objects) {
        bool has_support_auxiliary = false;
        for (uint32_t layer_idx = 0; layer_idx < object.auxiliary_layer_count(); ++layer_idx)
            if (object.auxiliary_layer(layer_idx).properties().get(LayerSupportProperty::key) != nullptr) {
                has_support_auxiliary = true;
                break;
            }
        if (!has_support_auxiliary)
            continue;

        const Config object_config = object.config();
        const int32_t support_extruder = object_config.int_or_default("support_material_extruder", 0);
        const int32_t interface_extruder = object_config.int_or_default("support_material_interface_extruder", 0);

        if (support_extruder == 0)
            support_uses_current_extruder = true;
        else
            extruders.insert(normalize_extruder_id(print_config, support_extruder));

        if (object_config.int_or_default("support_material_interface_layers", 0) > 0) {
            if (interface_extruder == 0)
                support_uses_current_extruder = true;
            else
                extruders.insert(normalize_extruder_id(print_config, interface_extruder));
        }
    }

    if (support_uses_current_extruder)
        for (Object object : objects) {
            const std::set<extruder_index_t> object_tools = object_extruders(print, object);
            extruders.insert(object_tools.begin(), object_tools.end());
        }
}

} // namespace

// Read a borrowed finalized layer group and return its first source height, or its Z gap from previous_print_z when source-free; the group is not mutated and validation failures remain the caller's responsibility.
coord_t printing_layer_height(const ConstPrintingLayerGroup &layer, coord_t previous_print_z)
{
    if (layer.layer_count() > 0)
        return layer.layer(0).height();
    return layer.print_z() - previous_print_z;
}

coord_t check_z_step(coord_t val, coord_t z_step)
{
    if (z_step <= SCALED_EPSILON)
        return val;
    return ((((val * 2) + z_step) / (2 * z_step)) * z_step);
}

void collect_object_printing_extruders(const Print &print,
                                       const Object &object,
                                       const PrintRegion &region,
                                       std::set<extruder_index_t> &object_extruders)
{
    const ConfigOption nozzle_diameter = print.config().get("nozzle_diameter");
    const int num_extruders = static_cast<int>(nozzle_diameter.size());

    auto emplace_extruder = [num_extruders, &object_extruders](int extruder_id) {
        const int idx = std::max(0, extruder_id - 1);
        const int normalized = idx >= num_extruders ? 0 : idx;
        if (normalized > std::numeric_limits<extruder_index_t>::max())
            throw std::invalid_argument("An object extruder index exceeds extruder_index_t.");
        object_extruders.insert(static_cast<extruder_index_t>(normalized));
    };

    const Config object_config = object.config();
    const Config region_config = region.config();

    if (region_config.get("perimeters").get_int() > 0 ||
        object_config.get("brim_width").get_float() > 0.0 ||
        object_config.get("brim_width_interior").get_float() > 0.0)
        emplace_extruder(region_config.get("perimeter_extruder").get_int());

    if (region_config.get("fill_density").get_float() > 0.0)
        emplace_extruder(region_config.get("infill_extruder").get_int());

    if (region_config.get("top_solid_layers").get_int() > 0 ||
        region_config.get("bottom_solid_layers").get_int() > 0 ||
        (region_config.get("solid_infill_every_layers").get_int() > 0 &&
         region_config.get("fill_density").get_float() > 0.0))
        emplace_extruder(region_config.get("solid_infill_extruder").get_int());
}

std::set<extruder_index_t> object_extruders(const Print &print, const Object &object)
{
    std::set<extruder_index_t> extruders;
    for (uint32_t region_idx = 0; region_idx < object.print_region_count(); ++region_idx)
        collect_object_printing_extruders(print, object, object.print_region(region_idx), extruders);
    return extruders;
}

coord_t get_object_first_layer_height(const Print &print, const Object &object)
{
    const Config print_config = print.config();
    const Config object_config = object.config();
    const ConfigOption first_layer_height = object_config.get("first_layer_height");

    if (!first_layer_height.is_percent())
        return scale_to_layer_coord(first_layer_height.get_float());

    coord_t out = std::numeric_limits<coord_t>::max();
    const std::set<extruder_index_t> extruders = object_extruders(print, object);
    for (extruder_index_t extruder_id : extruders) {
        assert(extruder_id != EXTRUDER_ID_UNKNOWN &&
               uint32_t(extruder_id) < print_config.size("nozzle_diameter", 0));
        const double nozzle_diameter = print_config.vector_float_or_default("nozzle_diameter", extruder_id, 0.4);
        out = std::min(out, scale_to_layer_coord(first_layer_height.get_effective_value(nozzle_diameter)));
    }

    if (out == std::numeric_limits<coord_t>::max()) {
        const double nozzle_diameter = print_config.vector_float_or_default("nozzle_diameter", 0, 0.4);
        out = scale_to_layer_coord(first_layer_height.get_effective_value(nozzle_diameter));
    }
    return out;
}

coord_t get_min_first_layer_height(const Print &print)
{
    if (print.object_count() == 0)
        throw std::runtime_error("Cannot compute first-layer height without Print objects.");

    coord_t out = std::numeric_limits<coord_t>::max();
    for (uint32_t object_idx = 0; object_idx < print.object_count(); ++object_idx)
        out = std::min(out, get_object_first_layer_height(print, print.object(object_idx)));
    if (out == std::numeric_limits<coord_t>::max())
        throw std::runtime_error("Cannot compute first-layer height.");
    return out;
}

std::set<extruder_index_t> collect_print_extruders_for_skirt(const Print &print,
                                                             const std::vector<Object> &objects)
{
    std::set<extruder_index_t> extruders;
    for (Object object : objects) {
        const std::set<extruder_index_t> object_tools = object_extruders(print, object);
        extruders.insert(object_tools.begin(), object_tools.end());
    }
    append_support_extruders(print, objects, extruders);
    if (extruders.empty())
        extruders.insert(0);
    return extruders;
}

double e_per_mm(const Print &print, extruder_index_t extruder_id, double mm3_per_mm)
{
    const Config print_config = print.config();
    assert(extruder_id != EXTRUDER_ID_UNKNOWN &&
           uint32_t(extruder_id) < print_config.size("extrusion_multiplier", 0));
    const double extrusion_multiplier =
        print_config.vector_float_or_default("extrusion_multiplier", extruder_id, 1.0);
    if (print_config.bool_or_default("use_volumetric_e", false))
        return mm3_per_mm * extrusion_multiplier;

    const double filament_diameter =
        print_config.vector_float_or_default("filament_diameter", extruder_id, 1.75);
    const double filament_area = filament_diameter * filament_diameter * 0.25 * PI;
    if (filament_area <= 0.0)
        throw std::runtime_error("Cannot compute E per mm with a non-positive filament diameter.");
    return mm3_per_mm * extrusion_multiplier / filament_area;
}

} // namespace slic3r_api
