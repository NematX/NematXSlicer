///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
Default layer extrusion acceleration
====================================

This plugin resolves acceleration on extrusion trees cloned into PrintingPlan.
Unlike speed, acceleration needs no group-wide flow analysis. setup_impl()
still validates regional source attributes and configuration links before parallel
layer runs begin so an invalid later layer cannot leave earlier layers edited.

The plugin modifies only accel_mm_per_s2. Existing speed, pressure, fan, and
temperature fields remain owned by their original producers.

Synthetic catalog entries have no source object and an unset instance index.
Their region-less trees retain service-owned properties without regional defaults;
real-object trees still require a source region island.
*/

#include "DefaultAcceleration.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <iterator>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ExtrusionProcessParameterHelpers.hpp"
#include "RegionalProcessParameterHelpers.hpp"
#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_layer_extrusion_edit.h"
#include "libslic3r/Api/plugin/cpp/GuiRuleHelpers.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"

namespace slic3r_api { namespace LayerExtrusionEdit { namespace DefaultAccelerationPlugin {
namespace {

using namespace ProcessParameterHelpers;
using namespace RegionalProcessParameterHelpers;

const char *const k_no_dependencies[] = { nullptr };

const raw_used_config_key k_used_config_keys[] = {
    {"bridge_acceleration", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"brim_acceleration", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"default_acceleration", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"external_perimeter_acceleration", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"first_layer_acceleration", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"gap_fill_acceleration", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"infill_acceleration", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"internal_bridge_acceleration", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"ironing_acceleration", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"machine_limits_usage", RAW_CO_ENUM, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"machine_max_acceleration_extruding", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"machine_max_acceleration_travel", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"overhangs_acceleration", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"perimeter_acceleration", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"solid_infill_acceleration", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_material_acceleration", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_material_interface_acceleration", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"thin_walls_acceleration", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"top_solid_infill_acceleration", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"travel_acceleration", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE}
};

const char *const k_defined_config_keys[] = {
    "bridge_acceleration",
    "brim_acceleration",
    "default_acceleration",
    "external_perimeter_acceleration",
    "first_layer_acceleration",
    "gap_fill_acceleration",
    "infill_acceleration",
    "internal_bridge_acceleration",
    "ironing_acceleration",
    "machine_max_acceleration_extruding",
    "machine_max_acceleration_travel",
    "overhangs_acceleration",
    "solid_infill_acceleration",
    "thin_walls_acceleration",
    "top_solid_infill_acceleration",
    "travel_acceleration"
};

void initialize_default_acceleration_settings(orchestrator_handle *orchestrator)
{
    /*
    Option definitions owned by this module. Each entry fixes the option type,
    override scope, default and GUI metadata; defined keys are published in the
    print presets, so profiles read and write them by name like any built-in
    setting. Called once from inilialize_impl().
    */
    raw_config_option_def def = raw_config_option_def_init();

    // ---- bridge_acceleration | preset=RAW_PRESET_TYPE_FFF_PRINT ----
    def = raw_config_option_def_init();
    def.opt_key = "bridge_acceleration";
    def.type = RAW_CO_FLOAT_OR_PERCENT;
    def.container_type = RAW_CONTAINER_TYPE_REGION;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_SPEED;
    def.label = "Bridges";
    def.full_label = "Bridge acceleration";
    def.tooltip = "This is the acceleration your printer will use for bridges.\nCan be a % of the default acceleration\nSet zero to use default acceleration for bridges.";
    def.sidetext = "mm/s² or %";
    def.ratio_over = "default_acceleration";
    def.has_min = 1;
    def.min_value = 0;
    def.has_max_literal = 1;
    def.max_literal_value = -220;
    def.max_literal_is_percent = 0;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_ADVANCED | RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_PRUSA;
    def.default_serialized_value = "0";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- brim_acceleration | preset=RAW_PRESET_TYPE_FFF_PRINT ----
    def = raw_config_option_def_init();
    def.opt_key = "brim_acceleration";
    def.type = RAW_CO_FLOAT_OR_PERCENT;
    def.container_type = RAW_CONTAINER_TYPE_OBJECT;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_SPEED;
    def.label = "Brim & Skirt";
    def.full_label = "Brim & Skirt acceleration";
    def.tooltip = "This is the acceleration your printer will use for brim and skirt. \nCan be a % of the support acceleration\nSet zero to use support acceleration.";
    def.sidetext = "mm/s² or %";
    def.ratio_over = "support_material_acceleration";
    def.has_min = 1;
    def.min_value = 0;
    def.has_max_literal = 1;
    def.max_literal_value = -200;
    def.max_literal_is_percent = 0;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_ADVANCED | RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
    def.default_serialized_value = "0";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- default_acceleration | preset=RAW_PRESET_TYPE_FFF_PRINT ----
    def = raw_config_option_def_init();
    def.opt_key = "default_acceleration";
    def.type = RAW_CO_FLOAT_OR_PERCENT;
    def.container_type = RAW_CONTAINER_TYPE_REGION;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_SPEED;
    def.label = "Default";
    def.full_label = "Default acceleration";
    def.tooltip = "This is the acceleration your printer will be reset to after the role-specific acceleration values are used (perimeter/infill). \nAccelerations from the left column can also be expressed as a percentage of this value.\nThis can be expressed as a percentage (for example: 80%) over the machine Max Acceleration for X axis.\nSet zero to prevent resetting acceleration at all.";
    def.sidetext = "mm/s² or %";
    def.ratio_over = "machine_max_acceleration_x";
    def.has_min = 1;
    def.min_value = 0;
    def.has_max_literal = 1;
    def.max_literal_value = -200;
    def.max_literal_is_percent = 0;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_ADVANCED | RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_PRUSA;
    def.default_serialized_value = "0";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- external_perimeter_acceleration | preset=RAW_PRESET_TYPE_FFF_PRINT ----
    def = raw_config_option_def_init();
    def.opt_key = "external_perimeter_acceleration";
    def.type = RAW_CO_FLOAT_OR_PERCENT;
    def.container_type = RAW_CONTAINER_TYPE_REGION;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_SPEED;
    def.label = "External";
    def.full_label = "External Perimeter acceleration";
    def.tooltip = "This is the acceleration your printer will use for external perimeters. \nCan be a % of the internal perimeter acceleration\nSet zero to use internal perimeter acceleration for external perimeters.";
    def.sidetext = "mm/s² or %";
    def.ratio_over = "perimeter_acceleration";
    def.has_min = 1;
    def.min_value = 0;
    def.has_max_literal = 1;
    def.max_literal_value = -200;
    def.max_literal_is_percent = 0;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_PRUSA;
    def.default_serialized_value = "0";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- first_layer_acceleration | preset=RAW_PRESET_TYPE_FFF_PRINT ----
    def = raw_config_option_def_init();
    def.opt_key = "first_layer_acceleration";
    def.type = RAW_CO_FLOAT_OR_PERCENT;
    def.container_type = RAW_CONTAINER_TYPE_OBJECT;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_SPEED;
    def.label = "Max";
    def.full_label = "First layer acceleration";
    def.tooltip = "This is the maximum acceleration your printer will use for first layer.\nIf set to %, all accelerations will be reduced by that ratio.\nSet zero to disable acceleration control for first layer.";
    def.sidetext = "mm/s² or %";
    def.ratio_over = "depends";
    def.has_min = 1;
    def.min_value = 0;
    def.has_max_literal = 1;
    def.max_literal_value = -200;
    def.max_literal_is_percent = 0;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_PRUSA;
    def.default_serialized_value = "0";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- gap_fill_acceleration | preset=RAW_PRESET_TYPE_FFF_PRINT ----
    def = raw_config_option_def_init();
    def.opt_key = "gap_fill_acceleration";
    def.type = RAW_CO_FLOAT_OR_PERCENT;
    def.container_type = RAW_CONTAINER_TYPE_REGION;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_SPEED;
    def.label = "Gap fill";
    def.full_label = "Gap fill acceleration";
    def.tooltip = "This is the acceleration your printer will use for gap fills. \nThis can be expressed as a percentage over the perimeter acceleration.\nSet zero to use perimeter acceleration for gap fills.";
    def.sidetext = "mm/s² or %";
    def.ratio_over = "perimeter_acceleration";
    def.has_min = 1;
    def.min_value = 0;
    def.has_max_literal = 1;
    def.max_literal_value = -200;
    def.max_literal_is_percent = 0;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
    def.default_serialized_value = "0";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- infill_acceleration | preset=RAW_PRESET_TYPE_FFF_PRINT ----
    def = raw_config_option_def_init();
    def.opt_key = "infill_acceleration";
    def.type = RAW_CO_FLOAT_OR_PERCENT;
    def.container_type = RAW_CONTAINER_TYPE_REGION;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_SPEED;
    def.label = "Sparse";
    def.full_label = "Infill acceleration";
    def.tooltip = "This is the acceleration your printer will use for Sparse infill.\nCan be a % of the solid infill acceleration\nSet zero to use solid infill acceleration for infill.";
    def.sidetext = "mm/s² or %";
    def.ratio_over = "solid_infill_acceleration";
    def.has_min = 1;
    def.min_value = 0;
    def.has_max_literal = 1;
    def.max_literal_value = -200;
    def.max_literal_is_percent = 0;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_ADVANCED | RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_PRUSA;
    def.default_serialized_value = "0";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- internal_bridge_acceleration | preset=RAW_PRESET_TYPE_FFF_PRINT ----
    def = raw_config_option_def_init();
    def.opt_key = "internal_bridge_acceleration";
    def.type = RAW_CO_FLOAT_OR_PERCENT;
    def.container_type = RAW_CONTAINER_TYPE_REGION;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_SPEED;
    def.label = "Internal bridges ";
    def.full_label = "Internal bridges acceleration";
    def.tooltip = "This is the acceleration your printer will use for internal bridges. \nCan be a % of the default acceleration\nSet zero to use bridge acceleration for internal bridges.";
    def.sidetext = "mm/s² or %";
    def.ratio_over = "bridge_acceleration";
    def.has_min = 1;
    def.min_value = 0;
    def.has_max_literal = 1;
    def.max_literal_value = -200;
    def.max_literal_is_percent = 0;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
    {
        static const char *const k_aliases[] = { "bridge_internal_acceleration" };
        def.aliases = const_strings_t{ k_aliases, 1 };
    }
    def.default_serialized_value = "0";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- ironing_acceleration | preset=RAW_PRESET_TYPE_FFF_PRINT ----
    def = raw_config_option_def_init();
    def.opt_key = "ironing_acceleration";
    def.type = RAW_CO_FLOAT_OR_PERCENT;
    def.container_type = RAW_CONTAINER_TYPE_REGION;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_IRONING;
    def.label = "Ironing";
    def.full_label = "Ironing acceleration";
    def.tooltip = "This is the acceleration your printer will use for ironing. \nCan be a % of the top solid infill acceleration\nSet zero or 100% to use top solid infill acceleration for ironing.";
    def.sidetext = "mm/s² or %";
    def.ratio_over = "top_solid_infill_acceleration";
    def.has_min = 1;
    def.min_value = 0;
    def.has_max_literal = 1;
    def.max_literal_value = -200;
    def.max_literal_is_percent = 0;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
    def.default_serialized_value = "0";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- machine_max_acceleration_extruding | preset=RAW_PRESET_TYPE_FFF_PRINTER,RAW_PRESET_TYPE_FFF_PRINTER_MACHINE_LIMITS ----
    def = raw_config_option_def_init();
    def.opt_key = "machine_max_acceleration_extruding";
    def.type = RAW_CO_VECTOR_FLOAT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINTER_MACHINE_LIMITS;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_ANY;
    def.category = RAW_OPTION_CATEGORY_LIMITS;
    def.full_label = "Maximum acceleration when extruding";
    def.tooltip = "Maximum acceleration when extruding";
    def.sidetext = "mm/s²";
    def.has_min = 1;
    def.min_value = 0;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_ADVANCED | RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_PRUSA;
    def.default_serialized_value = "1500,1250";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- machine_max_acceleration_travel | preset=RAW_PRESET_TYPE_FFF_PRINTER,RAW_PRESET_TYPE_FFF_PRINTER_MACHINE_LIMITS ----
    def = raw_config_option_def_init();
    def.opt_key = "machine_max_acceleration_travel";
    def.type = RAW_CO_VECTOR_FLOAT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINTER_MACHINE_LIMITS;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_ANY;
    def.category = RAW_OPTION_CATEGORY_LIMITS;
    def.full_label = "Maximum acceleration for travel moves";
    def.tooltip = "Maximum acceleration for travel moves.";
    def.sidetext = "mm/s²";
    def.has_min = 1;
    def.min_value = 0;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_ADVANCED | RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_PRUSA;
    def.default_serialized_value = "1500,1250";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- overhangs_acceleration | preset=RAW_PRESET_TYPE_FFF_PRINT ----
    def = raw_config_option_def_init();
    def.opt_key = "overhangs_acceleration";
    def.type = RAW_CO_FLOAT_OR_PERCENT;
    def.container_type = RAW_CONTAINER_TYPE_REGION;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_SPEED;
    def.label = "Overhangs";
    def.full_label = "Overhang acceleration";
    def.tooltip = "This is the acceleration your printer will use for overhangs.\nCan be a % of the bridge acceleration\nSet zero to to use bridge acceleration for overhangs.";
    def.sidetext = "mm/s² or %";
    def.ratio_over = "bridge_acceleration";
    def.has_min = 1;
    def.min_value = 0;
    def.has_max_literal = 1;
    def.max_literal_value = -200;
    def.max_literal_is_percent = 0;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
    def.default_serialized_value = "0";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- solid_infill_acceleration | preset=RAW_PRESET_TYPE_FFF_PRINT ----
    def = raw_config_option_def_init();
    def.opt_key = "solid_infill_acceleration";
    def.type = RAW_CO_FLOAT_OR_PERCENT;
    def.container_type = RAW_CONTAINER_TYPE_REGION;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_SPEED;
    def.label = "Solid ";
    def.full_label = "Solid acceleration";
    def.tooltip = "This is the acceleration your printer will use for solid infill. \nCan be a % of the default acceleration\nSet zero or 100% to use default acceleration for solid infill.";
    def.sidetext = "mm/s² or %";
    def.ratio_over = "default_acceleration";
    def.has_min = 1;
    def.min_value = 0;
    def.has_max_literal = 1;
    def.max_literal_value = -200;
    def.max_literal_is_percent = 0;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_ADVANCED | RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_PRUSA;
    def.default_serialized_value = "0";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- thin_walls_acceleration | preset=RAW_PRESET_TYPE_FFF_PRINT ----
    def = raw_config_option_def_init();
    def.opt_key = "thin_walls_acceleration";
    def.type = RAW_CO_FLOAT_OR_PERCENT;
    def.container_type = RAW_CONTAINER_TYPE_REGION;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_SPEED;
    def.label = "Thin Walls";
    def.full_label = "Thin walls acceleration";
    def.tooltip = "This is the acceleration your printer will use for thin walls. \nCan be a % of the external perimeter acceleration\nSet zero to use external perimeter acceleration for thin walls.";
    def.sidetext = "mm/s² or %";
    def.ratio_over = "external_perimeter_acceleration";
    def.has_min = 1;
    def.min_value = 0;
    def.has_max_literal = 1;
    def.max_literal_value = -200;
    def.max_literal_is_percent = 0;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
    def.default_serialized_value = "0";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- top_solid_infill_acceleration | preset=RAW_PRESET_TYPE_FFF_PRINT ----
    def = raw_config_option_def_init();
    def.opt_key = "top_solid_infill_acceleration";
    def.type = RAW_CO_FLOAT_OR_PERCENT;
    def.container_type = RAW_CONTAINER_TYPE_REGION;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_SPEED;
    def.label = "Top solid ";
    def.full_label = "Top solid acceleration";
    def.tooltip = "This is the acceleration your printer will use for top solid infill. \nCan be a % of the solid infill acceleration\nSet zero or 100% to use solid infill acceleration for top solid infill.";
    def.sidetext = "mm/s² or %";
    def.ratio_over = "solid_infill_acceleration";
    def.has_min = 1;
    def.min_value = 0;
    def.has_max_literal = 1;
    def.max_literal_value = -200;
    def.max_literal_is_percent = 0;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_PRUSA;
    def.default_serialized_value = "0";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- travel_acceleration | preset=RAW_PRESET_TYPE_FFF_PRINT ----
    def = raw_config_option_def_init();
    def.opt_key = "travel_acceleration";
    def.type = RAW_CO_FLOAT_OR_PERCENT;
    def.container_type = RAW_CONTAINER_TYPE_REGION;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_SPEED;
    def.label = "Travel";
    def.full_label = "Travel acceleration";
    def.tooltip = "Acceleration for travel moves (jumps between distant extrusion points).\nCan be a % of the default acceleration\nSet zero to use default acceleration for travel moves.";
    def.sidetext = "mm/s² or %";
    def.ratio_over = "default_acceleration";
    def.has_min = 1;
    def.min_value = 0;
    def.has_max_literal = 1;
    def.max_literal_value = -200;
    def.max_literal_is_percent = 0;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_ADVANCED | RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
    def.default_serialized_value = "1500";
    orchestrator_create_option_def(orchestrator, &def);
}

void add_default_acceleration_gui_rules(orchestrator_handle *orchestrator)
{
    using slic3r_api::all_of;
    using slic3r_api::any_of;
    using slic3r_api::GuiCondition;
    using slic3r_api::GuiRule;
    using slic3r_api::int_equals;
    using slic3r_api::non_zero;
    using slic3r_api::true_;

    // Membership tests shared by the rules below. `has_top` holds when a top
    // solid surface exists: solid top layers, spiral vase perimeters or solid
    // infill on every layer. `has_solid` also accepts bottom solid layers, a
    // solid infill cadence and solid infill below the area threshold.
    // `ironing_pp` requires ironing on a top surface, and `ironing_any` also
    // accepts a smooth top, bottom or solid fill pattern. gui_rules.core keeps
    // equivalent copies of these conditions for the settings it hosts.
    const GuiCondition has_top = any_of({
        non_zero("top_solid_layers"),
        all_of({ non_zero("perimeters"), true_("spiral_vase") }),
        int_equals("solid_infill_every_layers", 1)
    });
    const GuiCondition has_solid = any_of({
        non_zero("top_solid_layers"),
        all_of({ non_zero("perimeters"), true_("spiral_vase") }),
        int_equals("solid_infill_every_layers", 1),
        non_zero("bottom_solid_layers"),
        all_of({ non_zero("fill_density"), non_zero("solid_infill_every_layers") }),
        all_of({ non_zero("fill_density"), non_zero("solid_infill_below_area") })
    });
    const GuiCondition ironing_pp = all_of({ has_top, true_("ironing") });
    const GuiCondition ironing_any = any_of({
        ironing_pp,
        fill_pattern_is_smooth("top_fill_pattern"),
        fill_pattern_is_smooth("bottom_fill_pattern"),
        fill_pattern_is_smooth("solid_fill_pattern")
    });

    // Each call registers one activation reason for the listed settings; a
    // setting stays editable while at least one of its reasons holds.
    const auto add_rule = [orchestrator](std::initializer_list<const char *> target_keys,
                                         const GuiCondition &condition) {
        GuiRule::activate(std::vector<std::string>(target_keys.begin(), target_keys.end()))
            .when(condition)
            .scope(RAW_PRESET_TYPE_FFF_PRINT)
            .add_to(orchestrator);
    };

    add_rule({ "bridge_acceleration", "internal_bridge_acceleration", "overhangs_acceleration",
               "gap_fill_acceleration", "travel_acceleration", "first_layer_acceleration" },
             non_zero("default_acceleration"));
    add_rule({ "external_perimeter_acceleration", "thin_walls_acceleration" },
             all_of({ non_zero("default_acceleration"), non_zero("perimeters") }));
    add_rule({ "infill_acceleration" },
             all_of({ non_zero("default_acceleration"), non_zero("fill_density") }));
    add_rule({ "solid_infill_acceleration" },
             all_of({ non_zero("default_acceleration"), has_solid }));
    add_rule({ "top_solid_infill_acceleration" },
             all_of({ non_zero("default_acceleration"), has_top }));
    add_rule({ "ironing_acceleration" },
             all_of({ non_zero("default_acceleration"), ironing_any }));
    add_rule({ "brim_acceleration" },
             all_of({ non_zero("default_acceleration"),
                      any_of({ non_zero("brim_width"), non_zero("brim_width_interior"),
                               non_zero("skirts") }) }));
}

const RegionSettings::OptionKeyGroup k_region_acceleration_keys = {
    "default_acceleration",
    "bridge_acceleration",
    "external_perimeter_acceleration",
    "gap_fill_acceleration",
    "infill_acceleration",
    "internal_bridge_acceleration",
    "ironing_acceleration",
    "overhangs_acceleration",
    "perimeter_acceleration",
    "solid_infill_acceleration",
    "thin_walls_acceleration",
    "top_solid_infill_acceleration"
};

struct AccelerationPartitionKey
{
    RegionalSourceKey source;
    extruder_index_t extruder_id = EXTRUDER_ID_UNKNOWN;
    raw_extrusion_role role = RAW_EXTRUSION_ROLE_NONE;

    bool operator<(const AccelerationPartitionKey &rhs) const
    {
        if (source < rhs.source)
            return true;
        if (rhs.source < source)
            return false;
        if (extruder_id != rhs.extruder_id)
            return extruder_id < rhs.extruder_id;
        return uint16_t(role) < uint16_t(rhs.role);
    }
};

using AccelerationPartitions = std::map<AccelerationPartitionKey, RegionalProcessPartition>;
using RegionalSettingsPartitions = std::map<RegionalSourceKey, RegionalSettingsPartition>;

// Resolve role acceleration, then apply first-layer and machine limits.
double role_acceleration(const ExtrusionSettingsContext &context,
                         raw_extrusion_role role);

// Resolve travel acceleration without applying printable-role layer rules.
float resolved_travel_acceleration(const ExtrusionSettingsContext &context);

// Collect roles that still need acceleration from one validated tree.
void collect_unresolved_roles(MutableExtrusionEntity entity,
                              const EffectiveTreeState &parent_state,
                              std::set<raw_extrusion_role> &roles,
                              const plugin_run_context *run_ctx);

// Validate all trees and prepare immutable partitions before parallel mutation.
// Leave region-less synthetic roots untouched while validating real-object sources.
void prepare_plan(storage_handle *storage,
                  const Print &print,
                  PrintingPlan &plan,
                  AccelerationPartitions &partitions,
                  const plugin_run_context *run_ctx);

// Assign missing acceleration recursively while preserving speed and overrides.
void assign_acceleration_tree(MutableExtrusionEntity entity,
                              const EffectiveTreeState &parent_state,
                              const ExtrusionSettingsContext &context,
                              const LayerRegionIsland &region_island,
                              size_t printing_catalog_idx,
                              const AccelerationPartitions &partitions,
                              EPropertySpeedFieldEditor &editor,
                              const plugin_run_context *run_ctx);

// Resolve and compact acceleration on one cloned extrusion root.
// Leave region-less synthetic roots untouched while validating real-object sources.
void edit_extrusion(const Print &print,
                    const PrintingPlan &plan,
                    PrintingToolGroup tool_group,
                    PrintingExtrusion extrusion,
                    const AccelerationPartitions &partitions,
                    const plugin_run_context *run_ctx);

class DefaultAcceleration : public PluginBase
{
public:
    static DefaultAcceleration &instance(orchestrator_handle *orch);
    explicit DefaultAcceleration(orchestrator_handle *orch);

private:
    const char *id_impl() const noexcept override;
    const char *name_impl() const noexcept override;
    const char *description_impl() const noexcept override;
    const char *exclusive_group_impl() const noexcept override;
    const char *exclusive_group_label_impl() const noexcept override;
    const char *exclusive_group_tooltip_impl() const noexcept override;
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

    mutable bool m_setup_valid = false;
    mutable AccelerationPartitions m_partitions;
};

double role_acceleration(const ExtrusionSettingsContext &context,
                         raw_extrusion_role role)
{
    // A missing default means acceleration remains unresolved. Every
    // role-specific percentage below is rooted in this default.
    const Config &region = context.region_config;
    double acceleration = region.computed_float_or_default(
        "default_acceleration", int32_t(context.extruder_id), 0.0);
    if (acceleration <= 0.0)
        return 0.0;

    const double perimeter = effective_value(region, "perimeter_acceleration", acceleration);
    const double external = effective_value(region, "external_perimeter_acceleration", perimeter);
    const double solid = effective_value(region, "solid_infill_acceleration", acceleration);
    const double bridge = effective_value(region, "bridge_acceleration", acceleration);
    // Support acceleration is also the base of the interface and brim/skirt
    // values; zero in any of them inherits its base.
    const double support = effective_value(context.object_config, "support_material_acceleration", acceleration);

    // Select the most specific role override while retaining the family
    // fallback for disabled or zero options.
    if (RAW_EXTRUSION_ROLE_IS_SKIRT(role))
        acceleration = effective_value(context.object_config, "brim_acceleration", support);
    else if (RAW_EXTRUSION_ROLE_IS_SUPPORT(role))
        acceleration = RAW_EXTRUSION_ROLE_IS_SUPPORT_INTERFACE(role) ?
            effective_value(context.object_config, "support_material_interface_acceleration", support) :
            support;
    else if (role == RAW_EXTRUSION_ROLE_GAP_FILL)
        acceleration = effective_value(region, "gap_fill_acceleration", perimeter);
    else if (role == RAW_EXTRUSION_ROLE_THIN_WALL)
        acceleration = effective_value(region, "thin_walls_acceleration", external);
    else if (RAW_EXTRUSION_ROLE_HAS(role, RAW_EXTRUSION_ROLE_IRONING))
        acceleration = effective_value(region, "ironing_acceleration",
            effective_value(region, "top_solid_infill_acceleration", solid));
    else if (RAW_EXTRUSION_ROLE_IS_INFILL(role)) {
        if (RAW_EXTRUSION_ROLE_IS_BRIDGE(role))
            acceleration = RAW_EXTRUSION_ROLE_IS_EXTERNAL(role) ? bridge :
                effective_value(region, "internal_bridge_acceleration", bridge);
        else if (RAW_EXTRUSION_ROLE_HAS(role, RAW_EXTRUSION_ROLE_SOLID))
            acceleration = RAW_EXTRUSION_ROLE_IS_EXTERNAL(role) ?
                effective_value(region, "top_solid_infill_acceleration", solid) : solid;
        else
            acceleration = effective_value(region, "infill_acceleration", solid);
    } else if (RAW_EXTRUSION_ROLE_IS_PERIMETER(role)) {
        acceleration = RAW_EXTRUSION_ROLE_IS_BRIDGE(role) ?
            effective_value(region, "overhangs_acceleration", bridge) :
            (RAW_EXTRUSION_ROLE_IS_EXTERNAL(role) ? external : perimeter);
    }

    // First-layer acceleration and the active machine limit are both ceilings;
    // neither rule may raise a slower role-specific value.
    if (context.first_layer) {
        const double first_layer = effective_value(
            context.object_config, "first_layer_acceleration", acceleration);
        if (first_layer > 0.0)
            acceleration = std::min(acceleration, first_layer);
    }

    const int32_t limits_usage = context.print_config.enum_or_default("machine_limits_usage", 3);
    if (limits_usage <= 2) {
        const double machine_limit = context.print_config.vector_float_or_default(
            "machine_max_acceleration_extruding", 0, 0.0);
        if (machine_limit > 0.0)
            acceleration = std::min(acceleration, machine_limit);
    }
    return acceleration;
}

float resolved_travel_acceleration(const ExtrusionSettingsContext &context)
{
    // computed_float_or_default() resolves a percentage against
    // default_acceleration. A zero travel value follows the documented legacy
    // fallback and uses that same default directly.
    double acceleration = context.region_config.computed_float_or_default(
        "travel_acceleration", int32_t(context.extruder_id), 0.0);
    if (acceleration <= 0.0)
        acceleration = context.region_config.computed_float_or_default(
            "default_acceleration", int32_t(context.extruder_id), 0.0);

    // Machine travel acceleration is an independent physical ceiling. It does
    // not reuse the extrusion ceiling merely because both live in one payload.
    const int32_t limits_usage = context.print_config.enum_or_default("machine_limits_usage", 3);
    if (limits_usage <= 2) {
        const double machine_limit = context.print_config.vector_float_or_default(
            "machine_max_acceleration_travel", 0, 0.0);
        if (machine_limit > 0.0)
            acceleration = std::min(acceleration, machine_limit);
    }

    // Zero disables both travel and fallback acceleration. Preserve that
    // deliberate absence so the firmware may keep its current acceleration;
    // a shared-register dialect separately decides whether a travel switch is
    // safe when no extrusion acceleration has been established.
    if (!std::isfinite(acceleration))
        throw std::runtime_error("Travel acceleration must be finite when specified.");
    if (acceleration <= 0.0)
        return 0.f;
    const float stored_acceleration = float(acceleration);
    if (!std::isfinite(stored_acceleration))
        throw std::runtime_error("Travel acceleration cannot be represented by the stored float.");
    return stored_acceleration;
}

void collect_unresolved_roles(MutableExtrusionEntity entity,
                              const EffectiveTreeState &parent_state,
                              std::set<raw_extrusion_role> &roles,
                              const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    const EffectiveTreeState state = effective_state(entity, parent_state);
    if (entity.child_count() > 0) {
        for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx) {
            throw_if_cancelled(run_ctx);
            collect_unresolved_roles(entity.child_mutable(child_idx), state, roles, run_ctx);
        }
        return;
    }

    if (leaf_disposition(entity, state, EPropertySpeedField::Acceleration) == LeafDisposition::Editable &&
        state.acceleration <= 0.f)
        roles.insert(state.attributes.extrusion_role());
}

// Borrow regional roots to validate and cache owned acceleration partitions; skip region-less synthetic output and throw on invalid sources or cancellation.
void prepare_plan(storage_handle *storage,
                  const Print &print,
                  PrintingPlan &plan,
                  AccelerationPartitions &partitions,
                  const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    partitions.clear();
    std::set<AccelerationPartitionKey> prepared_keys;
    RegionalSettingsPartitions regional_settings;

    // Visit every root before mutation. Besides validating source attributes,
    // collect every role that may need a regional partition in later runs.
    for (uint32_t group_idx = 0; group_idx < plan.group_count(); ++group_idx) {
        throw_if_cancelled(run_ctx);
        PrintingGroup group = plan.group(group_idx);
        for (uint32_t layer_idx = 0; layer_idx < group.layer_group_count(); ++layer_idx) {
            throw_if_cancelled(run_ctx);
            PrintingLayerGroup layer_group = group.layer_group(layer_idx);
            for (uint32_t tool_idx = 0; tool_idx < layer_group.tool_group_count(); ++tool_idx) {
                throw_if_cancelled(run_ctx);
                PrintingToolGroup tool_group = layer_group.tool_group(tool_idx);
                for (uint32_t extrusion_idx = 0; extrusion_idx < tool_group.extrusion_count(); ++extrusion_idx) {
                    throw_if_cancelled(run_ctx);
                    PrintingExtrusion extrusion = tool_group.extrusion(extrusion_idx);
                    if (is_regionless_synthetic(plan, extrusion))
                        continue;
                    const ExtrusionSettingsContext context = validated_settings_context(
                        print, tool_group, extrusion, false);
                    const LayerRegionIsland region_island = extrusion.region_island();
                    const size_t object_instance_idx = printing_plan_object_instance_idx(plan, extrusion);

                    std::set<raw_extrusion_role> roles;
                    collect_unresolved_roles(
                        extrusion.mutable_root(), EffectiveTreeState{}, roles, run_ctx);
                    for (const raw_extrusion_role role : roles) {
                        throw_if_cancelled(run_ctx);
                        const RegionalSourceKey source{
                            region_island.handle(), static_cast<uint16_t>(object_instance_idx)
                        };
                        const AccelerationPartitionKey key{
                            source, tool_group.extruder_id(), role
                        };
                        if (!prepared_keys.insert(key).second)
                            continue;

                        // The geometry and raw setting tuples depend only on
                        // the source region-island and object instance. Reuse
                        // that expensive partition for every role and tool.
                        RegionalSettingsPartitions::iterator settings_it = regional_settings.find(source);
                        if (settings_it == regional_settings.end()) {
                            throw_if_cancelled(run_ctx);
                            RegionalSettingsPartition settings = build_regional_settings_partition(
                                storage, region_island, k_region_acceleration_keys, plan.object_instance_shift(extrusion));
                            settings_it = regional_settings.emplace(source, std::move(settings)).first;
                        }

                        throw_if_cancelled(run_ctx);
                        RegionalProcessPartition partition = project_regional_process_values(
                            storage, settings_it->second,
                            [&context, role](const Config &region_config) {
                                ExtrusionSettingsContext regional_context = context;
                                regional_context.region_config = region_config;
                                return float(role_acceleration(regional_context, role));
                            });
                        throw_if_cancelled(run_ctx);
                        if (partition.requires_split())
                            partitions.emplace(key, std::move(partition));
                    }
                }
            }
        }
    }
}

void assign_acceleration_tree(MutableExtrusionEntity entity,
                              const EffectiveTreeState &parent_state,
                              const ExtrusionSettingsContext &context,
                              const LayerRegionIsland &region_island,
                              size_t printing_catalog_idx,
                              const AccelerationPartitions &partitions,
                              EPropertySpeedFieldEditor &editor,
                              const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    // Collections propagate inherited state; only editable leaves receive a
    // missing acceleration value.
    const EffectiveTreeState state = effective_state(entity, parent_state);
    if (entity.child_count() > 0) {
        for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx) {
            throw_if_cancelled(run_ctx);
            assign_acceleration_tree(entity.child_mutable(child_idx), state, context,
                                     region_island, printing_catalog_idx, partitions, editor,
                                     run_ctx);
        }
        return;
    }

    const LeafDisposition disposition =
        leaf_disposition(entity, state, EPropertySpeedField::Acceleration);
    if (disposition == LeafDisposition::Empty || state.acceleration > 0.f)
        return;

    const raw_extrusion_role role = state.attributes.extrusion_role();
    // Travels bypass regional partitioning and first-layer role policy. Their
    // single movement setting is resolved directly from the process config.
    if (RAW_EXTRUSION_ROLE_IS_TRAVEL(role)) {
        throw_if_cancelled(run_ctx);
        editor.set_value(entity, resolved_travel_acceleration(context));
        return;
    }
    if (disposition != LeafDisposition::Editable)
        return;

    const RegionalSourceKey source{ region_island.handle(), printing_catalog_idx };
    const AccelerationPartitionKey key{
        source, context.extruder_id, role
    };
    const AccelerationPartitions::const_iterator partition_it = partitions.find(key);
    if (partition_it == partitions.end()) {
        throw_if_cancelled(run_ctx);
        editor.set_value(entity, float(role_acceleration(context, role)));
        return;
    }

    // The common helper keeps clipping, fragment ordering and property edits
    // identical between acceleration and speed.
    throw_if_cancelled(run_ctx);
    apply_regional_process_partition(entity, partition_it->second, editor);
}

// Borrow a plan-owned root, skip region-less synthetic output, and assign/hoist unresolved regional acceleration; source or cancellation failures throw without ownership transfer.
void edit_extrusion(const Print &print,
                    const PrintingPlan &plan,
                    PrintingToolGroup tool_group,
                    PrintingExtrusion extrusion,
                    const AccelerationPartitions &partitions,
                    const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    if (is_regionless_synthetic(plan, extrusion))
        return;
    const ExtrusionSettingsContext context = settings_context(print, tool_group, extrusion);
    const LayerRegionIsland region_island = extrusion.region_island();
    const size_t object_instance_idx = printing_plan_object_instance_idx(plan, extrusion);
    MutableExtrusionEntity root = extrusion.mutable_root();
    EPropertySpeedFieldEditor editor(EPropertySpeedField::Acceleration);
    assign_acceleration_tree(root, EffectiveTreeState{}, context, region_island,
                             static_cast<uint16_t>(object_instance_idx), partitions, editor,
                             run_ctx);
    throw_if_cancelled(run_ctx);
    editor.hoist(root);
}

DefaultAcceleration &DefaultAcceleration::instance(orchestrator_handle *orch)
{
    static DefaultAcceleration singleton(orch);
    return singleton;
}

DefaultAcceleration::DefaultAcceleration(orchestrator_handle *orch) : PluginBase(orch) {}

const char *DefaultAcceleration::id_impl() const noexcept
{
    return "layer_extrusion_edit.acceleration.default";
}

const char *DefaultAcceleration::name_impl() const noexcept
{
    return "Default extrusion acceleration";
}

const char *DefaultAcceleration::description_impl() const noexcept
{
    return "Resolves missing extrusion acceleration and applies machine limits.";
}

const char *DefaultAcceleration::exclusive_group_impl() const noexcept
{
    return "layer_extrusion_edit.acceleration";
}

const char *DefaultAcceleration::exclusive_group_label_impl() const noexcept
{
    return "Extrusion acceleration";
}

const char *DefaultAcceleration::exclusive_group_tooltip_impl() const noexcept
{
    return "Selects how ordered layer extrusions receive their process acceleration.";
}

slicing_step_t DefaultAcceleration::step_impl() const noexcept
{
    return STEP_LAYER_EXTRUSION_EDIT;
}

const char *const *DefaultAcceleration::dependencies_impl() const noexcept
{
    return k_no_dependencies;
}

int32_t DefaultAcceleration::priority_impl() const noexcept
{
    return 10;
}

int32_t DefaultAcceleration::used_config_keys(raw_used_config_key *keys) const noexcept
{
    if (keys != nullptr)
        std::copy(std::begin(k_used_config_keys), std::end(k_used_config_keys), keys);
    return int32_t(std::size(k_used_config_keys));
}

int32_t DefaultAcceleration::defined_config_keys(const char **keys) const noexcept
{
    if (keys != nullptr)
        for (size_t idx = 0; idx < std::size(k_defined_config_keys); ++idx)
            keys[idx] = k_defined_config_keys[idx];
    return int32_t(std::size(k_defined_config_keys));
}

void DefaultAcceleration::inilialize_impl(storage_handle *) const
{
    initialize_default_acceleration_settings(m_orchestrator);
    add_default_acceleration_gui_rules(m_orchestrator);
}

const char *DefaultAcceleration::progress_message_format_impl() const noexcept
{
    return "Resolving extrusion acceleration: %u / %u trees";
}

void DefaultAcceleration::setup_impl(const plugin_run_context *run_ctx, uint32_t) const
{
    throw_if_cancelled(run_ctx);
    m_setup_valid = false;
    m_partitions.clear();
    const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run_ctx);
    assert(ctx != nullptr && ctx->print != nullptr && ctx->plan != nullptr &&
           run_ctx != nullptr && run_ctx->plugin_storage != nullptr);
    if (ctx == nullptr || ctx->print == nullptr || ctx->plan == nullptr ||
        run_ctx == nullptr || run_ctx->plugin_storage == nullptr)
        return;

    PrintingPlan plan(ctx->plan);
    prepare_plan(run_ctx->plugin_storage, Print(ctx->print), plan,
                 m_partitions, run_ctx);
    throw_if_cancelled(run_ctx);
    m_setup_valid = true;
}

void DefaultAcceleration::setup_run_impl(const plugin_run_context *run_ctx) const
{
    const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run_ctx);
    if (ctx == nullptr || ctx->layer_group == nullptr)
        return;
    throw_if_cancelled(run_ctx);
    progress().add_max(extrusion_tree_count(PrintingLayerGroup(ctx->layer_group)));
}

void DefaultAcceleration::run_impl(const plugin_run_context *run_ctx) const
{
    if (!m_setup_valid)
        return;
    const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run_ctx);
    assert(ctx != nullptr && ctx->print != nullptr && ctx->plan != nullptr &&
           ctx->layer_group != nullptr);
    if (ctx == nullptr || ctx->print == nullptr || ctx->plan == nullptr ||
        ctx->layer_group == nullptr)
        return;
    throw_if_cancelled(run_ctx);

    const Print print(ctx->print);
    PrintingPlan plan(ctx->plan);
    PrintingLayerGroup layer_group(ctx->layer_group);
    for (uint32_t tool_idx = 0; tool_idx < layer_group.tool_group_count(); ++tool_idx) {
        throw_if_cancelled(run_ctx);
        PrintingToolGroup tool_group = layer_group.tool_group(tool_idx);
        for (uint32_t extrusion_idx = 0; extrusion_idx < tool_group.extrusion_count(); ++extrusion_idx) {
            throw_if_cancelled(run_ctx);
            edit_extrusion(
                print, plan, tool_group, tool_group.extrusion(extrusion_idx),
                m_partitions, run_ctx);
            progress().increment();
        }
    }
}

} // namespace

void register_default_acceleration_plugin(orchestrator_handle *orch)
{
    orchestrator_register_plugin(orch, DefaultAcceleration::instance(orch).c_instance());
}

}}} // namespace slic3r_api::LayerExtrusionEdit::DefaultAccelerationPlugin
