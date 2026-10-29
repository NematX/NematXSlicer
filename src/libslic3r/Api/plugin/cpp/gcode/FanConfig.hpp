///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#pragma once

#include "../../c/slic3r_config_def.h"

// Firmware alternatives define the same physical calibration setting in their
// exclusive group. Keeping its complete definition here lets an external
// firmware initialize alone without relying on another dialect's activation.
// Publish it before settings freeze, alongside any dialect-specific options:
//
//     const raw_config_option_def def = fan_minimum_option_definition();
//     orchestrator_create_option_def(host, &def);
namespace slic3r_api { namespace GCodeGeneration {

// Return borrowed constant metadata; callers publish it during initialization.
// The explicit host translation domain keeps built-in and external definitions
// compatible. Calibration affects export only, not logical layer requests.
inline raw_config_option_def fan_minimum_option_definition()
{
    raw_config_option_def def = raw_config_option_def_init();
    def.opt_key = "fan_printer_min_speed";
    def.type = RAW_CO_INT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINTER;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_GENERAL;
    def.label = "Minimum fan speed";
    def.full_label = "Minimum fan speed";
    def.tooltip = "This setting represents the minimum fan speed (like minimum PWM) your fan needs to work.";
    def.translation_domain = "Slic3r";
    def.sidetext = "%";
    def.has_min = 1;
    def.min_value = 0;
    def.has_max = 1;
    def.max_value = 100;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_ADVANCED | RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
    def.default_serialized_value = "0";
    return def;
}

}} // namespace slic3r_api::GCodeGeneration
