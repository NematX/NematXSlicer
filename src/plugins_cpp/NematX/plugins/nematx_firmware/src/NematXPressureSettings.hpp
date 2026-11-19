///|/ Copyright (c) NematXSlicer 2026 Durand Remi @supermerill
///|/
///|/ NematXSlicer is released under the terms of the AGPLv3 or higher

// Pressure setting definitions published by the NematX firmware provider.
// Values are owned structures whose strings have static lifetime. For example,
// for (raw_config_option_def def : pressure_option_definitions()) publish(def);
// Publication and host errors remain the caller's responsibility.
#pragma once

#include <array>

#include "libslic3r/Api/plugin/c/slic3r_config_def.h"

namespace nematx_firmware {

inline constexpr const char *k_pressure_factor_key = "extruder_pressure_factor";
inline constexpr const char *k_split_acceleration_key = "split_extrusion_acceleration";
inline constexpr const char *k_compressibility_key = "filament_compressibility_factor";

// Return the three historical pressure definitions, including their host scopes.
inline std::array<raw_config_option_def, 3> pressure_option_definitions()
{
    std::array<raw_config_option_def, 3> definitions{
        raw_config_option_def_init(), raw_config_option_def_init(), raw_config_option_def_init() };
    for (raw_config_option_def &def : definitions) {
        def.container_type = RAW_CONTAINER_TYPE_PROJECT;
        def.printer_technology = RAW_PT_FFF;
        def.invalidates_step = STEP_GCODE;
        def.mode = RAW_CONFIG_OPTION_MODE_EXPERT;
        def.is_vector_extruder = 1;
        def.default_serialized_value = "0";
    }

    raw_config_option_def &pressure = definitions[0];
    pressure.opt_key = k_pressure_factor_key;
    pressure.type = RAW_CO_VECTOR_FLOAT;
    pressure.option_preset_type = RAW_PRESET_TYPE_FFF_TOOL_EXTRUDER;
    pressure.category = RAW_OPTION_CATEGORY_EXTRUDERS;
    pressure.label = "Extruder pressure factor";
    pressure.tooltip = "Pressure coefficient K in seconds for the slicer's model: the filament offset in mm "
                       "is K times the model speed in mm/s. Zero disables pressure correction.";
    pressure.sidetext = "s";
    pressure.has_min = 1;
    pressure.min_value = 0;
    pressure.precision = 6;

    raw_config_option_def &split = definitions[1];
    split.opt_key = k_split_acceleration_key;
    split.type = RAW_CO_VECTOR_BOOL;
    split.option_preset_type = RAW_PRESET_TYPE_FFF_TOOL_EXTRUDER;
    split.category = RAW_OPTION_CATEGORY_EXTRUDERS;
    split.label = "Split extrusion acceleration";
    split.tooltip = "Split extrusion moves into acceleration, steady speed and deceleration phases and "
                    "annotate them, independently of pressure correction, including when K is zero.";

    raw_config_option_def &compressibility = definitions[2];
    compressibility.opt_key = k_compressibility_key;
    compressibility.type = RAW_CO_VECTOR_FLOAT;
    compressibility.option_preset_type = RAW_PRESET_TYPE_FFF_FILAMENT;
    compressibility.category = RAW_OPTION_CATEGORY_FILAMENT;
    compressibility.label = "Filament compressibility factor (inactive)";
    compressibility.tooltip = "Inactive setting retained only for import and export of historical profiles. "
                              "It has no effect on extrusion or movement and is not a time delay.";
    compressibility.has_min = 1;
    compressibility.min_value = 0;
    compressibility.precision = 6;
    return definitions;
}

} // namespace nematx_firmware
