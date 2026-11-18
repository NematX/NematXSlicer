///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#include "SupportSettings.hpp"

#include <algorithm>
#include <iterator>

/* Serialized-value parsing, pattern id resolution and the shared used-key
   lists of the support plugins (see SupportSettings.hpp). The key lists are
   static arrays, so used_config_keys() can copy them without allocating. */

namespace slic3r_api::Support::Common {
namespace {

// Pattern id of an interface left on "auto": the legacy toolpaths print
// soluble interfaces concentric, nearly full ones rectilinear and the others
// with the support base pattern.
std::string auto_interface_pattern_plugin_id(bool soluble, double interface_density);

// Read one option's serialized value, or "" when the key is missing.
std::string serialized(const Config &config, const char *key);

const raw_used_config_key k_geometry_keys[] = {
    {"support_material_style", RAW_CO_ENUM, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_material_buildplate_only", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_material_enforce_layers", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_material_threshold", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_material_synchronize_layers", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_material_contact_distance_type", RAW_CO_ENUM, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_material_contact_distance", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_material_bottom_contact_distance", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_material_xy_spacing", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_material_contact_xy_spacing", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_material_contact_cone_growth", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_material_layer_height", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_material_interface_layer_height", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_material_interface_layers", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_material_bottom_interface_layers", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_material_interface_spacing", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_material_spacing", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_material_angle", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_material_closing_radius", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_material_bottom_interface_expansion", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_material_extruder", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_material_interface_extruder", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_material_extrusion_width", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"first_layer_height", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"layer_height", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"nozzle_diameter", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"min_layer_height", RAW_CO_VECTOR_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"max_layer_height", RAW_CO_VECTOR_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"z_step", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"resolution_internal", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"extrusion_width", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"extrusion_spacing", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"first_layer_extrusion_width", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"first_layer_extrusion_spacing", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"first_layer_infill_extrusion_width", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"first_layer_infill_extrusion_spacing", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"external_perimeter_extrusion_width", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"external_perimeter_extrusion_spacing", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"external_perimeter_extrusion_change_odd_layers", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"external_perimeter_overlap", RAW_CO_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"perimeter_extrusion_width", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"perimeter_extrusion_spacing", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"perimeter_extruder", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"infill_extruder", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"solid_infill_extruder", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"perimeters", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"fill_density", RAW_CO_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"top_solid_layers", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"bottom_solid_layers", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"solid_infill_every_layers", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"infill_extrusion_width", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"infill_extrusion_spacing", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"infill_extrusion_change_odd_layers", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"bridge_type", RAW_CO_ENUM, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"bridge_flow_ratio", RAW_CO_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"filament_max_overlap", RAW_CO_VECTOR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"raft_first_layer_expansion", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE}
};

const raw_used_config_key k_path_keys[] = {
    {"support_material_pattern", RAW_CO_ENUM, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_material_with_sheath", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_material_top_interface_pattern", RAW_CO_ENUM, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_material_bottom_interface_pattern", RAW_CO_ENUM, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_material_interface_contact_loops", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_material_angle_height", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_material_interface_angle", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_material_interface_angle_increment", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"raft_first_layer_density", RAW_CO_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"gap_fill_min_area", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"gap_fill_min_width", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"gap_fill_max_width", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"gap_fill_min_length", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"gap_fill_extension", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"fill_smooth_width", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"fill_smooth_distribution", RAW_CO_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE}
};

std::string auto_interface_pattern_plugin_id(bool soluble, double interface_density)
{
    return soluble ? "concentric" : interface_density > 0.95 ? "rectilinear" : "supportbase";
}

std::string serialized(const Config &config, const char *key)
{
    return config.has(key) ? config.get(key).serialize() : std::string();
}

} // namespace

// Compare with the serialized names of support_material_style.
SupportStyle parse_support_style(const std::string &serialized)
{
    if (serialized == "grid") return SupportStyle::Grid;
    if (serialized == "snug") return SupportStyle::Snug;
    if (serialized == "tree") return SupportStyle::Tree;
    if (serialized == "organic") return SupportStyle::Organic;
    if (serialized == "filled") return SupportStyle::Filled;
    if (serialized == "grid_merged") return SupportStyle::GridMerged;
    return SupportStyle::Unknown;
}

// Compare with the serialized names of support_material_contact_distance_type.
ContactDistanceType parse_contact_distance_type(const std::string &serialized)
{
    if (serialized == "filament") return ContactDistanceType::Filament;
    if (serialized == "plane") return ContactDistanceType::Plane;
    if (serialized == "none") return ContactDistanceType::None;
    return ContactDistanceType::Unknown;
}

// Compare with the serialized names of support_material_pattern.
BasePattern parse_base_pattern(const std::string &serialized)
{
    if (serialized == "rectilinear") return BasePattern::Rectilinear;
    if (serialized == "rectilinear-grid") return BasePattern::RectilinearGrid;
    if (serialized == "honeycomb") return BasePattern::Honeycomb;
    return BasePattern::Unknown;
}

// Parse the object's support_material_style.
SupportStyle support_style(const Config &object_config)
{
    return parse_support_style(serialized(object_config, "support_material_style"));
}

// Parse the object's support_material_contact_distance_type.
ContactDistanceType contact_distance_type(const Config &object_config)
{
    return parse_contact_distance_type(serialized(object_config, "support_material_contact_distance_type"));
}

// Parse the object's support_material_pattern.
BasePattern base_pattern(const Config &object_config)
{
    return parse_base_pattern(serialized(object_config, "support_material_pattern"));
}

// Honeycomb keeps its own pattern; the other bases are rectilinear when nearly full or sheathed.
std::string base_pattern_plugin_id(BasePattern pattern, double base_density, bool with_sheath)
{
    if (pattern == BasePattern::Honeycomb)
        return "honeycomb";
    return base_density > 0.95 || with_sheath ? "rectilinear" : "supportbase";
}

// Resolve "auto"; every other value is already a pattern plugin id.
std::string top_interface_pattern_plugin_id(const std::string &serialized, bool soluble, double interface_density)
{
    return serialized == "auto" ? auto_interface_pattern_plugin_id(soluble, interface_density) : serialized;
}

// Resolve "auto" from the resolved top pattern, except the top-only patterns.
std::string bottom_interface_pattern_plugin_id(const std::string &serialized, const std::string &top_plugin_id,
                                               bool soluble, double interface_density)
{
    if (serialized != "auto")
        return serialized;
    if (top_plugin_id != "hilbertcurve" && top_plugin_id != "smooth" && top_plugin_id != "sawtooth")
        return top_plugin_id;
    return auto_interface_pattern_plugin_id(soluble, interface_density);
}

// The geometry list, a static array.
UsedKeyList support_geometry_keys()
{
    return {k_geometry_keys, std::size(k_geometry_keys)};
}

// The path list, a static array.
UsedKeyList support_path_keys()
{
    return {k_path_keys, std::size(k_path_keys)};
}

// Copy into a caller-sized buffer, or only count when keys is null.
int32_t append_used_config_keys(raw_used_config_key *keys, int32_t offset, UsedKeyList list) noexcept
{
    if (keys != nullptr)
        std::copy(list.keys, list.keys + list.size, keys + offset);
    return offset + int32_t(list.size);
}

} // namespace slic3r_api::Support::Common
