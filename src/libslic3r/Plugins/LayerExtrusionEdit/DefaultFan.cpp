///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
Default PrintingPlan fan assignment
===================================

This plugin moves the fan-only policy from CoolingBuffer into the ordered
PrintingPlan. It first computes one duration for each final layer group, then
builds the legacy role fan table independently for every extruder. Missing fan
fields are written on leaves and compacted towards collection roots.

No G-code is produced here. The firmware sees the resulting inherited
fan_speed_percent and decides whether a physical fan command is necessary.
*/

#include "DefaultFan.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <optional>
#include <set>
#include <stdexcept>
#include <vector>

#include "ExtrusionProcessParameterHelpers.hpp"
#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_layer_extrusion_edit.h"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanTimeEstimator.hpp"
#include "libslic3r/Plugins/PrintingPlan/PrintingLayerTimeProperty.h"

namespace slic3r_api { namespace LayerExtrusionEdit { namespace DefaultFanPlugin {
// Register the permanent borrowed plugin and copy its alternative-selection setting description into the application.
// Print data is unchanged; allocation failures or rejected descriptions throw.
void register_default_fan_plugin(orchestrator_handle *orch);

namespace {

using namespace ProcessParameterHelpers;

const char *const k_no_dependencies[] = { nullptr };

const raw_used_config_key k_used_config_keys[] = {
    {"bridge_fan_speed", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"default_fan_speed", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"disable_fan_first_layers", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"external_perimeter_fan_speed", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"fan_below_layer_time", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"fan_printer_min_speed", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"full_fan_speed_layer", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"gap_fill_fan_speed", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"infill_fan_speed", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"internal_bridge_fan_speed", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"max_fan_speed", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"overhangs_dynamic_fan_speed", RAW_CO_VECTOR_GRAPH, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"overhangs_fan_speed", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"perimeter_fan_speed", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"slowdown_below_layer_time", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"solid_infill_fan_speed", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_material_fan_speed", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_material_interface_fan_speed", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"top_fan_speed", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"travel_speed", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE}
};

const char *const k_defined_config_keys[] = {
    "bridge_fan_speed",
    "default_fan_speed",
    "disable_fan_first_layers",
    "external_perimeter_fan_speed",
    "fan_below_layer_time",
    "fan_printer_min_speed",
    "full_fan_speed_layer",
    "gap_fill_fan_speed",
    "infill_fan_speed",
    "internal_bridge_fan_speed",
    "max_fan_speed",
    "overhangs_fan_speed",
    "perimeter_fan_speed",
    "slowdown_below_layer_time",
    "solid_infill_fan_speed",
    "support_material_fan_speed",
    "support_material_interface_fan_speed",
    "top_fan_speed"
};

void initialize_default_fan_settings(orchestrator_handle *orchestrator)
    {
    /*
    Option definitions owned by this module. Each entry fixes the option type,
    override scope, default and GUI metadata; defined keys are published in the
    print presets, so profiles read and write them by name like any built-in
    setting. Called once from inilialize_impl().
    */
    raw_config_option_def def = raw_config_option_def_init();

    // ---- bridge_fan_speed | preset=RAW_PRESET_TYPE_FFF_FILAMENT ----
    def = raw_config_option_def_init();
    def.opt_key = "bridge_fan_speed";
    def.type = RAW_CO_VECTOR_INT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_FILAMENT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_COOLING;
    def.label = "Bridge Infill fan speed";
    def.tooltip = "This fan speed is enforced during bridges and overhangs. It won't slow down the fan if it's currently running at a higher speed.\nSet to 0 to stop the fan.\nIf disabled, default fan speed will be used.\nCan be disabled by disable_fan_first_layers and increased by low layer time.";
    def.sidetext = "%";
    def.is_vector_extruder = 1;
    def.can_be_disabled = 1;
    def.has_min = 1;
    def.min_value = 0;
    def.has_max = 1;
    def.max_value = 100;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_ADVANCED | RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_PRUSA;
    def.default_serialized_value = "100";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- disable_fan_first_layers | preset=RAW_PRESET_TYPE_FFF_FILAMENT ----
    def = raw_config_option_def_init();
    def.opt_key = "disable_fan_first_layers";
    def.type = RAW_CO_VECTOR_INT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_FILAMENT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_COOLING;
    def.label = "Disable fan for the first";
    def.tooltip = "You can set this to a positive value to disable fan at all during the first layers, so that it does not make adhesion worse.";
    def.sidetext = "layers";
    def.is_vector_extruder = 1;
    def.has_min = 1;
    def.min_value = 0;
    def.has_max = 1;
    def.max_value = 1000;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_PRUSA;
    def.default_serialized_value = "1";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- external_perimeter_fan_speed | preset=RAW_PRESET_TYPE_FFF_FILAMENT ----
    def = raw_config_option_def_init();
    def.opt_key = "external_perimeter_fan_speed";
    def.type = RAW_CO_VECTOR_INT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_FILAMENT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.label = "External perimeter fan speed";
    def.tooltip = "When set to a non-zero value this fan speed is used only for external perimeters (visible ones) and thin walls.\nSet to 0 to stop the fan.\nIf disabled, the default fan speed will be used.\nExternal perimeters can benefit from higher fan speed to improve surface finish, while internal perimeters, infill, etc. benefit from lower fan speed to improve layer adhesion.\nCan be disabled by disable_fan_first_layers, slowed down by full_fan_speed_layer and increased by low layer time.";
    def.sidetext = "%";
    def.is_vector_extruder = 1;
    def.can_be_disabled = 1;
    def.has_min = 1;
    def.min_value = 0;
    def.has_max = 1;
    def.max_value = 100;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_ADVANCED | RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
    def.default_serialized_value = "!100";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- fan_below_layer_time | preset=RAW_PRESET_TYPE_FFF_FILAMENT ----
    def = raw_config_option_def_init();
    def.opt_key = "fan_below_layer_time";
    def.type = RAW_CO_VECTOR_FLOAT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_FILAMENT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_COOLING;
    def.label = "Enable fan if layer print time is below";
    def.tooltip = "If layer print time is estimated below this number of seconds, fan will be enabled and its speed will be calculated by interpolating the default and maximum speeds.\nSet zero to disable.";
    def.sidetext = "approximate seconds";
    def.is_vector_extruder = 1;
    def.has_min = 1;
    def.min_value = 0;
    def.has_max = 1;
    def.max_value = 1000;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_PRUSA;
    def.default_serialized_value = "60";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- fan_printer_min_speed | preset=RAW_PRESET_TYPE_FFF_PRINTER ----
    def = raw_config_option_def_init();
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
    def.sidetext = "%";
    def.has_min = 1;
    def.min_value = 0;
    def.has_max = 1;
    def.max_value = 100;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_ADVANCED | RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
    def.default_serialized_value = "0";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- full_fan_speed_layer | preset=RAW_PRESET_TYPE_FFF_FILAMENT ----
    def = raw_config_option_def_init();
    def.opt_key = "full_fan_speed_layer";
    def.type = RAW_CO_VECTOR_INT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_FILAMENT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_FILAMENT;
    def.label = "Full fan speed at layer";
    def.tooltip = "Fan speed will be ramped up linearly from zero at layer \"disable_fan_first_layers\" to maximum at layer \"full_fan_speed_layer\". \"full_fan_speed_layer\" will be ignored if equal or lower than \"disable_fan_first_layers\", in which case the fan will be running at maximum allowed speed at layer \"disable_fan_first_layers\" + 1.\nset 0 to disable";
    def.is_vector_extruder = 1;
    def.has_min = 1;
    def.min_value = 0;
    def.has_max = 1;
    def.max_value = 1000;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_PRUSA;
    def.default_serialized_value = "4";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- gap_fill_fan_speed | preset=RAW_PRESET_TYPE_FFF_FILAMENT ----
    def = raw_config_option_def_init();
    def.opt_key = "gap_fill_fan_speed";
    def.type = RAW_CO_VECTOR_INT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_FILAMENT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_COOLING;
    def.label = "Gap fill fan speed";
    def.tooltip = "This fan speed is enforced during all gap fill Perimeter moves\nSet to 0 to stop the fan.\nIf disabled, default fan speed will be used.\nCan be disabled by disable_fan_first_layers, slowed down by full_fan_speed_layer and increased by low layer time.";
    def.sidetext = "%";
    def.is_vector_extruder = 1;
    def.can_be_disabled = 1;
    def.has_min = 1;
    def.min_value = 0;
    def.has_max = 1;
    def.max_value = 100;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
    def.default_serialized_value = "!100";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- infill_fan_speed | preset=RAW_PRESET_TYPE_FFF_FILAMENT ----
    def = raw_config_option_def_init();
    def.opt_key = "infill_fan_speed";
    def.type = RAW_CO_VECTOR_INT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_FILAMENT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_COOLING;
    def.label = "Internal Infill fan speed";
    def.tooltip = "This fan speed is enforced during all Internal Infill moves\nSet to 0 to stop the fan.\nIf disabled, default fan speed will be used.\nCan be disabled by disable_fan_first_layers, slowed down by full_fan_speed_layer and increased by low layer time.";
    def.sidetext = "%";
    def.is_vector_extruder = 1;
    def.can_be_disabled = 1;
    def.has_min = 1;
    def.min_value = 0;
    def.has_max = 1;
    def.max_value = 100;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
    def.default_serialized_value = "!100";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- internal_bridge_fan_speed | preset=RAW_PRESET_TYPE_FFF_FILAMENT ----
    def = raw_config_option_def_init();
    def.opt_key = "internal_bridge_fan_speed";
    def.type = RAW_CO_VECTOR_INT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_FILAMENT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_COOLING;
    def.label = "Internal Bridge Infill fan speed";
    def.tooltip = "This fan speed is enforced during all infill bridges. It won't slow down the fan if it's currently running at a higher speed.\nSet to 0 to stop the fan.\nIf disabled, Bridge fan speed will be used.\nCan be disabled by disable_fan_first_layers and increased by low layer time.";
    def.sidetext = "%";
    def.is_vector_extruder = 1;
    def.can_be_disabled = 1;
    def.has_min = 1;
    def.min_value = 0;
    def.has_max = 1;
    def.max_value = 100;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_ADVANCED | RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
    {
    static const char *const k_aliases[] = { "bridge_internal_fan_speed" };
    def.aliases = const_strings_t{ k_aliases, 1 };
    }
    def.default_serialized_value = "!100";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- max_fan_speed | preset=RAW_PRESET_TYPE_FFF_FILAMENT ----
    def = raw_config_option_def_init();
    def.opt_key = "max_fan_speed";
    def.type = RAW_CO_VECTOR_INT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_FILAMENT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_COOLING;
    def.label = "Max";
    def.full_label = "Max fan speed";
    def.tooltip = "This setting represents the maximum speed of your fan, used when the layer print time is Very short.";
    def.sidetext = "%";
    def.is_vector_extruder = 1;
    def.has_min = 1;
    def.min_value = 0;
    def.has_max = 1;
    def.max_value = 100;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_ADVANCED | RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_PRUSA;
    def.default_serialized_value = "100";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- overhangs_fan_speed | preset=RAW_PRESET_TYPE_FFF_FILAMENT ----
    def = raw_config_option_def_init();
    def.opt_key = "overhangs_fan_speed";
    def.type = RAW_CO_VECTOR_INT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_FILAMENT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_COOLING;
    def.label = "Overhangs Perimeter fan speed";
    def.tooltip = "This fan speed is enforced during all Overhang Perimeter moves\nIf disabled, the previous (perimeter) fan speed will be used.\nCan be overridden by disable_fan_first_layers and increased by low layer time.";
    def.sidetext = "%";
    def.is_vector_extruder = 1;
    def.can_be_disabled = 1;
    def.has_min = 1;
    def.min_value = 0;
    def.has_max = 1;
    def.max_value = 100;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_ADVANCED | RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
    def.default_serialized_value = "!100";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- perimeter_fan_speed | preset=RAW_PRESET_TYPE_FFF_FILAMENT ----
    def = raw_config_option_def_init();
    def.opt_key = "perimeter_fan_speed";
    def.type = RAW_CO_VECTOR_INT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_FILAMENT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_COOLING;
    def.label = "Internal Perimeter fan speed";
    def.tooltip = "This fan speed is enforced during all Perimeter moves\nSet to 0 to stop the fan.\nIf disabled, default fan speed will be used.\nCan be disabled by disable_fan_first_layers, slowed down by full_fan_speed_layer and increased by low layer time.";
    def.sidetext = "%";
    def.is_vector_extruder = 1;
    def.can_be_disabled = 1;
    def.has_min = 1;
    def.min_value = 0;
    def.has_max = 1;
    def.max_value = 100;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
    def.default_serialized_value = "!100";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- slowdown_below_layer_time | preset=RAW_PRESET_TYPE_FFF_FILAMENT ----
    def = raw_config_option_def_init();
    def.opt_key = "slowdown_below_layer_time";
    def.type = RAW_CO_VECTOR_FLOAT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_FILAMENT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_COOLING;
    def.label = "Slow down if layer print time is below";
    def.tooltip = "If layer print time is estimated below this number of seconds, print moves speed will be scaled down to extend duration to this value, if possible.\nSet zero to disable.";
    def.sidetext = "approximate seconds";
    def.is_vector_extruder = 1;
    def.has_min = 1;
    def.min_value = 0;
    def.has_max = 1;
    def.max_value = 1000;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_PRUSA;
    def.default_serialized_value = "5";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- solid_infill_fan_speed | preset=RAW_PRESET_TYPE_FFF_FILAMENT ----
    def = raw_config_option_def_init();
    def.opt_key = "solid_infill_fan_speed";
    def.type = RAW_CO_VECTOR_INT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_FILAMENT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_COOLING;
    def.label = "Solid Infill fan speed";
    def.tooltip = "This fan speed is enforced during all Solid Infill moves\nSet to 0 to stop the fan.\nIf disabled, default fan speed will be used.\nCan be disabled by disable_fan_first_layers, slowed down by full_fan_speed_layer and increased by low layer time.";
    def.sidetext = "%";
    def.is_vector_extruder = 1;
    def.can_be_disabled = 1;
    def.has_min = 1;
    def.min_value = 0;
    def.has_max = 1;
    def.max_value = 100;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
    def.default_serialized_value = "!100";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- support_material_fan_speed | preset=RAW_PRESET_TYPE_FFF_FILAMENT ----
    def = raw_config_option_def_init();
    def.opt_key = "support_material_fan_speed";
    def.type = RAW_CO_VECTOR_INT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_FILAMENT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_COOLING;
    def.label = "Support Material fan speed";
    def.tooltip = "This fan speed is enforced during all support moves\nSet to 0 to stop the fan.\nIf disabled, default fan speed will be used.\nCan be disabled by disable_fan_first_layers, slowed down by full_fan_speed_layer.";
    def.sidetext = "%";
    def.is_vector_extruder = 1;
    def.can_be_disabled = 1;
    def.has_min = 1;
    def.min_value = 0;
    def.has_max = 1;
    def.max_value = 100;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
    def.default_serialized_value = "!100";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- support_material_interface_fan_speed | preset=RAW_PRESET_TYPE_FFF_FILAMENT ----
    def = raw_config_option_def_init();
    def.opt_key = "support_material_interface_fan_speed";
    def.type = RAW_CO_VECTOR_INT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_FILAMENT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_COOLING;
    def.label = "Support interface fan speed";
    def.tooltip = "This fan speed is enforced during all support interfaces, to be able to weaken their bonding with a high fan speed.\nSet to 0 to stop the fan.\nIf disabled, Support Material fan speed will be used.\nCan only be overridden by disable_fan_first_layers.";
    def.sidetext = "%";
    def.is_vector_extruder = 1;
    def.can_be_disabled = 1;
    def.has_min = 1;
    def.min_value = 0;
    def.has_max = 1;
    def.max_value = 100;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_ADVANCED | RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
    def.default_serialized_value = "!100";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- top_fan_speed | preset=RAW_PRESET_TYPE_FFF_FILAMENT ----
    def = raw_config_option_def_init();
    def.opt_key = "top_fan_speed";
    def.type = RAW_CO_VECTOR_INT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_FILAMENT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_COOLING;
    def.label = "Top Solid fan speed";
    def.tooltip = "This fan speed is enforced during all top fills (including ironing).\nSet to 0 to stop the fan.\nIf disabled, Solid Infill fan speed will be used.\nCan be disabled by disable_fan_first_layers, slowed down by full_fan_speed_layer.";
    def.sidetext = "%";
    def.is_vector_extruder = 1;
    def.can_be_disabled = 1;
    def.has_min = 1;
    def.min_value = 0;
    def.has_max = 1;
    def.max_value = 100;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_ADVANCED | RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
    def.default_serialized_value = "!100";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- default_fan_speed | preset=RAW_PRESET_TYPE_FFF_FILAMENT ----
    def = raw_config_option_def_init();
    def.opt_key = "default_fan_speed";
    def.type = RAW_CO_VECTOR_INT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_FILAMENT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_COOLING;
    def.label = "Default fan speed";
    def.tooltip = "Default speed for the fan, to set the speed for features where there is no fan control. Useful for PLA and other low-temp filament.\nSet 0 to disable the fan by default. Useful for ABS and other high-temp filaments.\nIf disabled, no fan speed command will be emitted when possible (if a feature set a speed, it won't be reverted).";
    def.is_vector_extruder = 1;
    def.can_be_disabled = 1;
    def.has_min = 1;
    def.min_value = 0;
    def.has_max = 1;
    def.max_value = 100;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_SIMPLE | RAW_CONFIG_OPTION_MODE_ADVANCED | RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
    {
    static const char *const k_aliases[] = { "min_fan_speed" };
    def.aliases = const_strings_t{ k_aliases, 1 };
    }
    def.default_serialized_value = "!100";
    orchestrator_create_option_def(orchestrator, &def);
}


enum class FanRole : uint8_t
{
    Default,
    Perimeter,
    ExternalPerimeter,
    OverhangPerimeter,
    InternalInfill,
    InternalBridgeInfill,
    SolidInfill,
    TopSolidInfill,
    Ironing,
    BridgeInfill,
    ThinWall,
    GapFill,
    Skirt,
    SupportMaterial,
    SupportMaterialInterface,
    Count
};

struct GeneratedFanMarker
{
    uint8_t generated = 1;
};

struct LayerFanPolicy
{
    std::array<int, size_t(FanRole::Count)> speed = {};
    int minimum_speed = 0;
    int dynamic_min = 0;
    int dynamic_max = 100;
};

int role_index(FanRole role);
int legacy_fan_value(const Config &config, const char *key, extruder_index_t extruder_id);
int normalize_fan_value(int value);
FanRole fan_role(raw_extrusion_role role);
bool can_increase(FanRole role);
bool can_ramp(FanRole role);
LayerFanPolicy layer_fan_policy(const Config &config,
                                extruder_index_t extruder_id,
                                uint32_t layer_idx,
                                double duration_seconds);
float dynamic_overhang_fan(const Config &config,
                           extruder_index_t extruder_id,
                           raw_extrusion_role role,
                           const EffectiveTreeState &state,
                           const LayerFanPolicy &policy);
void remove_generated_fields(MutableExtrusionEntity entity,
                             const PluginPropertyKey<GeneratedFanMarker> &marker,
                             const plugin_run_context *run_ctx);
void mark_generated_fields(MutableExtrusionEntity entity,
                           const PluginPropertyKey<GeneratedFanMarker> &marker,
                           const std::set<extrusion_entity_handle *> &generated,
                           const plugin_run_context *run_ctx);
void assign_fan_tree(MutableExtrusionEntity entity,
                     const EffectiveTreeState &parent_state,
                     const Config &config,
                     extruder_index_t extruder_id,
                     const LayerFanPolicy &policy,
                     EPropertySpeedFieldEditor &editor,
                     const plugin_run_context *run_ctx);
void edit_extrusion(PrintingExtrusion extrusion,
                    const Config &config,
                    extruder_index_t extruder_id,
                    const LayerFanPolicy &policy,
                    const PluginPropertyKey<GeneratedFanMarker> &marker,
                    const plugin_run_context *run_ctx);

class DefaultFan : public PluginBase
{
public:
    static DefaultFan &instance(orchestrator_handle *orch);
    explicit DefaultFan(orchestrator_handle *orch);

private:
    const char *id_impl() const noexcept override;
    const char *name_impl() const noexcept override;
    const char *description_impl() const noexcept override;
    const char *exclusive_group_impl() const noexcept override;
    slicing_step_t step_impl() const noexcept override;
    const char *const *dependencies_impl() const noexcept override;
    int32_t priority_impl() const noexcept override;
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override;
    int32_t defined_config_keys(const char **keys) const noexcept override;
    void inilialize_impl(storage_handle *) const override;
    const char *progress_message_format_impl() const noexcept override;
    void setup_impl(const plugin_run_context *run_ctx, uint32_t run_count) const override;
    void setup_run_impl(const plugin_run_context *run_ctx) const override;
    void run_impl(const plugin_run_context *run_ctx) const override;

    PluginPropertyKey<PrintingLayerTimeProperty> m_layer_time_property;
    PluginPropertyKey<GeneratedFanMarker> m_generated_marker;
    mutable bool m_setup_valid = false;
    mutable std::vector<std::vector<double>> m_layer_durations;
};

int role_index(FanRole role)
{
    return int(uint8_t(role));
}

int legacy_fan_value(const Config &config, const char *key, extruder_index_t extruder_id)
{
    return normalize_fan_value(config.vector_int_or_default(key, extruder_id, -1));
}

int normalize_fan_value(int value)
{
    // Older profiles used 1 as their serialized spelling for a stopped fan.
    // Preserve that compatibility before any minimum-speed clamp is applied.
    return value == 1 ? 0 : value;
}

FanRole fan_role(raw_extrusion_role role)
{
    // Test the compound roles before their broader bit families. This is the
    // same specificity order used by the legacy G-code role conversion.
    if (role == RAW_EXTRUSION_ROLE_GAP_FILL)
        return FanRole::GapFill;
    if (RAW_EXTRUSION_ROLE_IS_SUPPORT(role))
        return RAW_EXTRUSION_ROLE_HAS(role, RAW_EXTRUSION_ROLE_EXTERNAL) ?
            FanRole::SupportMaterialInterface : FanRole::SupportMaterial;
    if (RAW_EXTRUSION_ROLE_IS_SKIRT(role))
        return FanRole::Skirt;
    if (RAW_EXTRUSION_ROLE_HAS(role, RAW_EXTRUSION_ROLE_IRONING))
        return FanRole::Ironing;
    if (RAW_EXTRUSION_ROLE_IS_INFILL(role) && RAW_EXTRUSION_ROLE_HAS(role, RAW_EXTRUSION_ROLE_BRIDGE))
        return RAW_EXTRUSION_ROLE_HAS(role, RAW_EXTRUSION_ROLE_EXTERNAL) ?
            FanRole::BridgeInfill : FanRole::InternalBridgeInfill;
    if (RAW_EXTRUSION_ROLE_IS_INFILL(role) && RAW_EXTRUSION_ROLE_HAS(role, RAW_EXTRUSION_ROLE_SOLID))
        return RAW_EXTRUSION_ROLE_HAS(role, RAW_EXTRUSION_ROLE_EXTERNAL) ?
            FanRole::TopSolidInfill : FanRole::SolidInfill;
    if (RAW_EXTRUSION_ROLE_IS_INFILL(role))
        return FanRole::InternalInfill;
    if (role == RAW_EXTRUSION_ROLE_THIN_WALL)
        return FanRole::ThinWall;
    if (RAW_EXTRUSION_ROLE_IS_PERIMETER(role) && RAW_EXTRUSION_ROLE_HAS(role, RAW_EXTRUSION_ROLE_BRIDGE))
        return FanRole::OverhangPerimeter;
    if (RAW_EXTRUSION_ROLE_IS_PERIMETER(role) && RAW_EXTRUSION_ROLE_HAS(role, RAW_EXTRUSION_ROLE_EXTERNAL))
        return FanRole::ExternalPerimeter;
    if (RAW_EXTRUSION_ROLE_IS_PERIMETER(role))
        return FanRole::Perimeter;
    return FanRole::Count;
}

bool can_increase(FanRole role)
{
    return role == FanRole::Default || role == FanRole::BridgeInfill ||
           role == FanRole::InternalBridgeInfill || role == FanRole::ExternalPerimeter ||
           role == FanRole::ThinWall || role == FanRole::Perimeter ||
           role == FanRole::SolidInfill || role == FanRole::InternalInfill ||
           role == FanRole::OverhangPerimeter || role == FanRole::GapFill;
}

bool can_ramp(FanRole role)
{
    return role == FanRole::Default || role == FanRole::TopSolidInfill ||
           role == FanRole::Ironing || role == FanRole::SupportMaterial ||
           role == FanRole::ExternalPerimeter || role == FanRole::ThinWall ||
           role == FanRole::Perimeter || role == FanRole::SolidInfill ||
           role == FanRole::InternalInfill || role == FanRole::GapFill;
}

LayerFanPolicy layer_fan_policy(const Config &config,
                                extruder_index_t extruder_id,
                                uint32_t layer_idx,
                                double duration_seconds)
{
    LayerFanPolicy policy;
    policy.speed.fill(-1);

    const int minimum = std::max(0, config.int_or_default("fan_printer_min_speed", 0));
    policy.minimum_speed = minimum;
    int default_speed = legacy_fan_value(config, "default_fan_speed", extruder_id);
    if (default_speed > 0)
        default_speed = std::max(default_speed, minimum);
    policy.speed[role_index(FanRole::Default)] = default_speed;
    policy.speed[role_index(FanRole::Skirt)] = default_speed;

    // Build the role table before applying time-based changes. A disabled role
    // keeps -1 so the late family fallback at the end of the pass can redirect
    // it to its reference role.
    policy.speed[role_index(FanRole::BridgeInfill)] = legacy_fan_value(config, "bridge_fan_speed", extruder_id);
    policy.speed[role_index(FanRole::InternalBridgeInfill)] = legacy_fan_value(config, "internal_bridge_fan_speed", extruder_id);
    policy.speed[role_index(FanRole::TopSolidInfill)] = legacy_fan_value(config, "top_fan_speed", extruder_id);
    policy.speed[role_index(FanRole::Ironing)] = policy.speed[role_index(FanRole::TopSolidInfill)];
    policy.speed[role_index(FanRole::SupportMaterialInterface)] = legacy_fan_value(config, "support_material_interface_fan_speed", extruder_id);
    policy.speed[role_index(FanRole::SupportMaterial)] = legacy_fan_value(config, "support_material_fan_speed", extruder_id);
    policy.speed[role_index(FanRole::ExternalPerimeter)] = legacy_fan_value(config, "external_perimeter_fan_speed", extruder_id);
    policy.speed[role_index(FanRole::ThinWall)] = policy.speed[role_index(FanRole::ExternalPerimeter)];
    policy.speed[role_index(FanRole::Perimeter)] = legacy_fan_value(config, "perimeter_fan_speed", extruder_id);
    policy.speed[role_index(FanRole::SolidInfill)] = legacy_fan_value(config, "solid_infill_fan_speed", extruder_id);
    policy.speed[role_index(FanRole::InternalInfill)] = legacy_fan_value(config, "infill_fan_speed", extruder_id);
    policy.speed[role_index(FanRole::OverhangPerimeter)] = legacy_fan_value(config, "overhangs_fan_speed", extruder_id);
    policy.speed[role_index(FanRole::GapFill)] = legacy_fan_value(config, "gap_fill_fan_speed", extruder_id);

    if (policy.speed[role_index(FanRole::TopSolidInfill)] < 0)
        policy.speed[role_index(FanRole::TopSolidInfill)] = policy.speed[role_index(FanRole::SolidInfill)];
    if (policy.speed[role_index(FanRole::SupportMaterialInterface)] < 0)
        policy.speed[role_index(FanRole::SupportMaterialInterface)] = policy.speed[role_index(FanRole::SupportMaterial)];
    if (policy.speed[role_index(FanRole::InternalBridgeInfill)] < 0)
        policy.speed[role_index(FanRole::InternalBridgeInfill)] = policy.speed[role_index(FanRole::BridgeInfill)];

    const ConfigOption dynamic_option = config.get("overhangs_dynamic_fan_speed");
    const bool dynamic_overhang = dynamic_option.valid() && size_t(extruder_id) < dynamic_option.size() &&
                                  dynamic_option.is_enabled(extruder_id);
    if (dynamic_overhang)
        policy.speed[role_index(FanRole::OverhangPerimeter)] = -1;

    if (default_speed >= 0) {
        for (size_t idx = 0; idx < policy.speed.size(); ++idx)
            if (policy.speed[idx] < 0 && idx != size_t(FanRole::OverhangPerimeter))
                policy.speed[idx] = default_speed;
    }

    const int disabled_layers = config.vector_int_or_default("disable_fan_first_layers", extruder_id, 0);
    if (int(layer_idx) < disabled_layers) {
        // The plan stores effective per-leaf state, so every controlled role
        // receives zero during the disabled prefix instead of relying on an
        // ambient default fan command.
        policy.speed.fill(0);
        policy.dynamic_min = 0;
        policy.dynamic_max = 0;
        return policy;
    }

    const int maximum = std::clamp(config.vector_int_or_default("max_fan_speed", extruder_id, 100), 0, 100);
    const double slowdown_time = config.vector_float_or_default("slowdown_below_layer_time", extruder_id, 0.0);
    const double fan_time = config.vector_float_or_default("fan_below_layer_time", extruder_id, 0.0);

    // Very short layers use maximum cooling. Between the two thresholds the
    // same rounded linear interpolation as CoolingBuffer is retained.
    if (duration_seconds < slowdown_time && fan_time > 0.0) {
        for (size_t idx = 0; idx < policy.speed.size(); ++idx)
            if (can_increase(FanRole(idx)))
                policy.speed[idx] = std::max(maximum, policy.speed[idx]);
        policy.dynamic_min = std::max(0, policy.speed[role_index(FanRole::Default)]);
    } else if (duration_seconds < fan_time && fan_time > slowdown_time) {
        const double ratio = (duration_seconds - slowdown_time) / (fan_time - slowdown_time);
        for (size_t idx = 0; idx < policy.speed.size(); ++idx) {
            if (can_increase(FanRole(idx)) && policy.speed[idx] >= 0 && policy.speed[idx] < maximum)
                policy.speed[idx] = std::clamp(
                    int(ratio * policy.speed[idx] + (1.0 - ratio) * maximum + 0.5), 0, 100);
        }
        policy.dynamic_min = std::max(0, policy.speed[role_index(FanRole::Default)]);
    }

    const int full_layer = config.vector_int_or_default("full_fan_speed_layer", extruder_id, 0);
    if (int(layer_idx) + 1 < full_layer && full_layer > disabled_layers) {
        const float factor = float(int(layer_idx) + 1 - disabled_layers) /
                             float(full_layer - disabled_layers);
        for (size_t idx = 0; idx < policy.speed.size(); ++idx) {
            if (can_ramp(FanRole(idx)) && policy.speed[idx] > 0)
                policy.speed[idx] = std::clamp(int(float(policy.speed[idx]) * factor + 0.01f), 0, 100);
        }
        policy.dynamic_max = std::max(0, policy.speed[role_index(FanRole::Default)]);
    }
    if (policy.dynamic_min > policy.dynamic_max)
        policy.dynamic_min = policy.dynamic_max;

    // Apply late family fallbacks after time rules, matching the legacy role
    // state machine, then enforce the physical minimum for active values.
    if (policy.speed[role_index(FanRole::BridgeInfill)] < 0)
        policy.speed[role_index(FanRole::BridgeInfill)] = policy.speed[role_index(FanRole::Default)];
    if (policy.speed[role_index(FanRole::InternalBridgeInfill)] < 0)
        policy.speed[role_index(FanRole::InternalBridgeInfill)] = policy.speed[role_index(FanRole::BridgeInfill)];
    if (policy.speed[role_index(FanRole::ExternalPerimeter)] < 0)
        policy.speed[role_index(FanRole::ExternalPerimeter)] = policy.speed[role_index(FanRole::Perimeter)];
    if (policy.speed[role_index(FanRole::TopSolidInfill)] < 0)
        policy.speed[role_index(FanRole::TopSolidInfill)] = policy.speed[role_index(FanRole::SolidInfill)];
    policy.speed[role_index(FanRole::Ironing)] = policy.speed[role_index(FanRole::TopSolidInfill)];
    policy.speed[role_index(FanRole::ThinWall)] = policy.speed[role_index(FanRole::ExternalPerimeter)];

    for (int &speed : policy.speed)
        if (speed > 0)
            speed = std::clamp(std::max(speed, minimum), 0, 100);
    return policy;
}

float dynamic_overhang_fan(const Config &config,
                           extruder_index_t extruder_id,
                           raw_extrusion_role role,
                           const EffectiveTreeState &state,
                           const LayerFanPolicy &policy)
{
    const FanRole fallback_role = RAW_EXTRUSION_ROLE_HAS(role, RAW_EXTRUSION_ROLE_EXTERNAL) ?
        FanRole::ExternalPerimeter : FanRole::Perimeter;
    int result = policy.speed[role_index(fallback_role)];

    const ConfigOption option = config.get("overhangs_dynamic_fan_speed");
    if (!state.has_overhang || !option.valid() || size_t(extruder_id) >= option.size() ||
        !option.is_enabled(extruder_id) || state.overhang.start_distance_from_prev_layer <= 0.f ||
        state.overhang.end_distance_from_prev_layer <= 0.f)
        return float(result);

    const std::optional<GraphData> graph = option.graph(extruder_id);
    if (!graph || !graph->validate())
        return float(result);

    // Dynamic cooling is an increase over the perimeter fallback. The layer
    // limits prevent it from bypassing first-layer/ramp timing decisions.
    const double start = graph->interpolate(100.0 - 100.0 * std::min(1.f, state.overhang.start_distance_from_prev_layer));
    const double end = graph->interpolate(100.0 - 100.0 * std::min(1.f, state.overhang.end_distance_from_prev_layer));
    const int dynamic = std::clamp(int(std::min(start, end) + 0.5),
                                   policy.dynamic_min, policy.dynamic_max);
    result = std::max(result, dynamic);
    if (result > 0)
        result = std::max(result, policy.minimum_speed);
    return float(std::clamp(result, 0, 100));
}

void remove_generated_fields(MutableExtrusionEntity entity,
                             const PluginPropertyKey<GeneratedFanMarker> &marker,
                             const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    // A marker means only the fan field on this exact node belongs to this
    // plugin. Other process fields and unmarked fan overrides are untouched.
    if (marker.has(entity)) {
        throw_if_cancelled(run_ctx);
        if (EPropertySpeed *process = entity.get_mutable(EPropertySpeed::key)) {
            process->fan_speed_percent = -1.f;
            const bool empty = process->speed_mm_per_s < 0.f && process->accel_mm_per_s2 < 0.f &&
                process->pressure_adv < 0.f && process->fan_speed_percent < 0.f && process->temperature_C < 0.f;
            if (empty) {
                throw_if_cancelled(run_ctx);
                entity.remove(EPropertySpeed::key);
            }
        }
        throw_if_cancelled(run_ctx);
        marker.remove(entity);
    }
    for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx) {
        throw_if_cancelled(run_ctx);
        remove_generated_fields(entity.child_mutable(child_idx), marker, run_ctx);
    }
}

void mark_generated_fields(MutableExtrusionEntity entity,
                           const PluginPropertyKey<GeneratedFanMarker> &marker,
                           const std::set<extrusion_entity_handle *> &generated,
                           const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    if (generated.count(entity.mutable_handle()) > 0) {
        throw_if_cancelled(run_ctx);
        marker.get_or_add(entity) = GeneratedFanMarker{};
    }
    for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx) {
        throw_if_cancelled(run_ctx);
        mark_generated_fields(entity.child_mutable(child_idx), marker, generated, run_ctx);
    }
}

void assign_fan_tree(MutableExtrusionEntity entity,
                     const EffectiveTreeState &parent_state,
                     const Config &config,
                     extruder_index_t extruder_id,
                     const LayerFanPolicy &policy,
                     EPropertySpeedFieldEditor &editor,
                     const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    const EffectiveTreeState state = effective_state(entity, parent_state);
    if (entity.child_count() > 0) {
        for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx) {
            throw_if_cancelled(run_ctx);
            assign_fan_tree(
                entity.child_mutable(child_idx), state, config, extruder_id, policy,
                editor, run_ctx);
        }
        return;
    }

    if (leaf_disposition(entity, state, EPropertySpeedField::FanSpeed) != LeafDisposition::Editable ||
        state.fan_speed >= 0.f)
        return;

    const raw_extrusion_role role = state.attributes.extrusion_role();
    const FanRole resolved_role = fan_role(role);
    if (resolved_role == FanRole::Count)
        return;
    float value = float(policy.speed[role_index(resolved_role)]);
    if (RAW_EXTRUSION_ROLE_IS_PERIMETER(role) && RAW_EXTRUSION_ROLE_HAS(role, RAW_EXTRUSION_ROLE_BRIDGE))
        value = dynamic_overhang_fan(config, extruder_id, role, state, policy);
    throw_if_cancelled(run_ctx);
    editor.set_value(entity, value);
}

void edit_extrusion(PrintingExtrusion extrusion,
                    const Config &config,
                    extruder_index_t extruder_id,
                    const LayerFanPolicy &policy,
                    const PluginPropertyKey<GeneratedFanMarker> &marker,
                    const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    MutableExtrusionEntity root = extrusion.mutable_root();
    remove_generated_fields(root, marker, run_ctx);

    // Recompute from the upstream tree after removing only our previous pass,
    // then compact equal leaf values without changing geometry or ordering.
    EPropertySpeedFieldEditor editor(EPropertySpeedField::FanSpeed);
    assign_fan_tree(
        root, EffectiveTreeState{}, config, extruder_id, policy, editor, run_ctx);
    throw_if_cancelled(run_ctx);
    editor.hoist(root);
    throw_if_cancelled(run_ctx);
    mark_generated_fields(root, marker, editor.modified_entities(), run_ctx);
}

DefaultFan &DefaultFan::instance(orchestrator_handle *orch)
{
    static DefaultFan plugin(orch);
    return plugin;
}

DefaultFan::DefaultFan(orchestrator_handle *orch) :
    PluginBase(orch),
    m_layer_time_property(printing_layer_time_property_key(orch)),
    m_generated_marker(PluginPropertyKey<GeneratedFanMarker>::register_dynamic(
        orch, "slic3r.layer_extrusion_edit.fan.default.generated"))
{
}

const char *DefaultFan::id_impl() const noexcept { return "layer_extrusion_edit.fan.default"; }
const char *DefaultFan::name_impl() const noexcept { return "Default fan speed"; }
const char *DefaultFan::description_impl() const noexcept
{
    return "Assign fan speed from extrusion role, layer duration, and filament cooling settings.";
}
const char *DefaultFan::exclusive_group_impl() const noexcept { return "layer_extrusion_edit.fan"; }
slicing_step_t DefaultFan::step_impl() const noexcept { return STEP_LAYER_EXTRUSION_EDIT; }
const char *const *DefaultFan::dependencies_impl() const noexcept { return k_no_dependencies; }
int32_t DefaultFan::priority_impl() const noexcept { return 20; }

int32_t DefaultFan::used_config_keys(raw_used_config_key *keys) const noexcept
{
    if (keys != nullptr)
        std::copy(std::begin(k_used_config_keys), std::end(k_used_config_keys), keys);
    return int32_t(std::size(k_used_config_keys));
}

int32_t DefaultFan::defined_config_keys(const char **keys) const noexcept
{
    if (keys != nullptr)
        for (size_t idx = 0; idx < std::size(k_defined_config_keys); ++idx)
            keys[idx] = k_defined_config_keys[idx];
    return int32_t(std::size(k_defined_config_keys));
}

void DefaultFan::inilialize_impl(storage_handle *) const
{
    initialize_default_fan_settings(m_orchestrator);
}

const char *DefaultFan::progress_message_format_impl() const noexcept
{
    return "Resolving extrusion fan speed: %u / %u trees";
}

void DefaultFan::setup_impl(const plugin_run_context *run_ctx, uint32_t) const
{
    throw_if_cancelled(run_ctx);
    m_setup_valid = false;
    m_layer_durations.clear();
    const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run_ctx);
    if (ctx == nullptr || ctx->print == nullptr || ctx->plan == nullptr)
        return;

    const Print print(ctx->print);
    PrintingPlan plan(ctx->plan);
    m_layer_durations.resize(plan.group_count());
    bool needs_estimate = false;
    for (uint32_t group_idx = 0; group_idx < plan.group_count(); ++group_idx) {
        throw_if_cancelled(run_ctx);
        m_layer_durations[group_idx].resize(plan.group(group_idx).layer_group_count(), 0.0);
        const PrintingGroup group = plan.group(group_idx);
        for (uint32_t layer_idx = 0; layer_idx < group.layer_group_count(); ++layer_idx) {
            throw_if_cancelled(run_ctx);
            const PrintingLayerTimeProperty *time =
                m_layer_time_property.get(group.layer_group(layer_idx).properties());
            if (time != nullptr && time->origin != RAW_PRINTING_LAYER_TIME_ORIGIN_ESTIMATED &&
                time->origin != RAW_PRINTING_LAYER_TIME_ORIGIN_FINAL)
                throw std::runtime_error("A PrintingLayerGroup duration has an unknown origin.");
            if (time != nullptr && time->is_final()) {
                if (!std::isfinite(time->duration_seconds) || time->duration_seconds < 0.0)
                    throw std::runtime_error("A final PrintingLayerGroup duration is invalid.");
                m_layer_durations[group_idx][layer_idx] = time->duration_seconds;
            } else {
                needs_estimate = true;
            }
        }
    }

    // Avoid touching geometry when an authoritative timing producer already
    // covered every layer. Otherwise one ordered pass supplies all missing or
    // replaceable estimates while preserving continuity between layers.
    if (needs_estimate) {
        throw_if_cancelled(run_ctx);
        const std::vector<PrintingLayerTimeEstimate> estimates =
            PrintingPlanTimeEstimator(print.config()).estimate(plan);
        for (const PrintingLayerTimeEstimate &estimate : estimates) {
            throw_if_cancelled(run_ctx);
            const PrintingLayerTimeProperty *time = m_layer_time_property.get(
                plan.group(estimate.group_idx).layer_group(estimate.layer_group_idx).properties());
            if (time == nullptr || !time->is_final())
                m_layer_durations.at(estimate.group_idx).at(estimate.layer_group_idx) =
                    estimate.duration_seconds;
        }
    }
    m_setup_valid = true;
}

void DefaultFan::setup_run_impl(const plugin_run_context *run_ctx) const
{
    const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run_ctx);
    if (!m_setup_valid || ctx == nullptr || ctx->layer_group == nullptr ||
        ctx->group_idx >= m_layer_durations.size() ||
        ctx->layer_group_idx >= m_layer_durations[ctx->group_idx].size())
        return;

    throw_if_cancelled(run_ctx);
    PrintingLayerGroup layer(ctx->layer_group);
    PluginProperties properties = layer.properties();
    PrintingLayerTimeProperty *time = m_layer_time_property.get_mutable(properties);
    if (time == nullptr || !time->is_final()) {
        throw_if_cancelled(run_ctx);
        PrintingLayerTimeProperty &published = m_layer_time_property.get_or_add(properties);
        published.duration_seconds = m_layer_durations[ctx->group_idx][ctx->layer_group_idx];
        published.origin = RAW_PRINTING_LAYER_TIME_ORIGIN_ESTIMATED;
    }
    progress().add_max(extrusion_tree_count(layer));
}

void DefaultFan::run_impl(const plugin_run_context *run_ctx) const
{
    if (!m_setup_valid)
        return;
    const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run_ctx);
    if (ctx == nullptr || ctx->print == nullptr || ctx->layer_group == nullptr)
        return;
    throw_if_cancelled(run_ctx);

    const Print print(ctx->print);
    const Config config = print.config();
    PrintingLayerGroup layer(ctx->layer_group);
    const PrintingLayerTimeProperty *time = m_layer_time_property.get(layer.properties());
    if (time == nullptr || !std::isfinite(time->duration_seconds) || time->duration_seconds < 0.0)
        throw std::runtime_error("Printing layer fan calculation needs a valid layer duration.");

    for (uint32_t tool_idx = 0; tool_idx < layer.tool_group_count(); ++tool_idx) {
        throw_if_cancelled(run_ctx);
        PrintingToolGroup tool = layer.tool_group(tool_idx);
        if (tool.extruder_id() == EXTRUDER_ID_UNKNOWN)
            throw std::runtime_error("Printing layer fan calculation needs a concrete extruder id.");
        const LayerFanPolicy policy = layer_fan_policy(
            config, tool.extruder_id(), ctx->layer_group_idx, time->duration_seconds);
        for (uint32_t extrusion_idx = 0; extrusion_idx < tool.extrusion_count(); ++extrusion_idx) {
            throw_if_cancelled(run_ctx);
            edit_extrusion(tool.extrusion(extrusion_idx), config, tool.extruder_id(), policy,
                           m_generated_marker, run_ctx);
            progress().increment();
        }
    }
}

} // namespace

void register_default_fan_plugin(orchestrator_handle *orch)
{
    orchestrator_register_plugin(orch, DefaultFan::instance(orch).c_instance());
    const plugin_instance member = DefaultFan::instance(orch).c_instance();
    raw_exclusive_group_def definition = raw_exclusive_group_def_init();
    definition.exclusive_group = member.vt->get_exclusive_group(member.ctx);
    definition.label = "Fan speed";
    definition.tooltip = "Selects how fan speed is assigned to ordered extrusion paths.";
    DefaultFan::instance(orch).register_exclusive_group(STEP_LAYER_EXTRUSION_EDIT, definition);
}

}}} // namespace slic3r_api::LayerExtrusionEdit::DefaultFanPlugin
