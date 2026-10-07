///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
Default layer extrusion speed
=============================

This plugin resolves speed on extrusion trees cloned into PrintingPlan. Its
setup pass validates every printable regional tree and computes immutable autospeed
targets per PrintingGroup and extruder. Parallel layer runs then resolve only
missing speed values, apply first-layer and physical limits, and compact
uniform speed towards parent nodes.

Synthetic catalog entries have no source object and an unset instance index.
Their region-less trees retain service-owned extrusion speeds. Missing speeds
on Travel connectors inserted into those trees use the machine travel_speed;
real-object trees still require a source region island.

Acceleration is intentionally not handled here. A separate plugin may choose
and hoist acceleration without depending on this strategy.
*/

#include "DefaultSpeed.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <iterator>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "ExtrusionProcessParameterHelpers.hpp"
#include "RegionalProcessParameterHelpers.hpp"
#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_layer_extrusion_edit.h"
#include "libslic3r/Api/plugin/cpp/GuiRuleHelpers.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"

namespace slic3r_api { namespace LayerExtrusionEdit { namespace DefaultSpeedPlugin {
namespace {

using namespace ProcessParameterHelpers;
using namespace RegionalProcessParameterHelpers;

const char *const k_no_dependencies[] = { nullptr };

const raw_used_config_key k_used_config_keys[] = {
    {"autospeed_min_thin_flow", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"bridge_speed", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"brim_speed", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"external_perimeter_speed", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"filament_max_speed", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"filament_max_volumetric_speed", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"first_layer_flow_ratio", RAW_CO_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"first_layer_infill_speed", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"first_layer_min_speed", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"first_layer_speed", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"gap_fill_speed", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"infill_speed", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"internal_bridge_speed", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"ironing_speed", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"max_print_speed", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"max_volumetric_speed", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"milling_speed", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"overhangs", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"overhangs_speed", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"perimeter_speed", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"solid_infill_speed", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_material_interface_speed", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_material_speed", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"thin_walls_speed", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"top_solid_infill_speed", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"travel_speed", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE}
};

const char *const k_defined_config_keys[] = {
    "autospeed_min_thin_flow",
    "bridge_speed",
    "brim_speed",
    "external_perimeter_speed",
    "filament_max_speed",
    "filament_max_volumetric_speed",
    "first_layer_flow_ratio",
    "first_layer_infill_speed",
    "first_layer_min_speed",
    "first_layer_speed",
    "gap_fill_speed",
    "infill_speed",
    "internal_bridge_speed",
    "ironing_speed",
    "max_print_speed",
    "max_volumetric_speed",
    "milling_speed",
    "perimeter_speed",
    "solid_infill_speed",
    "thin_walls_speed",
    "top_solid_infill_speed"
};

void initialize_default_speed_settings(orchestrator_handle *orchestrator)
{
    /*
    Option definitions owned by this module. Each entry fixes the option type,
    override scope, default and GUI metadata; defined keys are published in the
    print presets, so profiles read and write them by name like any built-in
    setting. Called once from inilialize_impl().
    */
    raw_config_option_def def = raw_config_option_def_init();

    // ---- autospeed_min_thin_flow | preset=RAW_PRESET_TYPE_FFF_PRINT ----
    def = raw_config_option_def_init();
    def.opt_key = "autospeed_min_thin_flow";
    def.type = RAW_CO_FLOAT_OR_PERCENT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_SPEED;
    def.label = "Minimum flow for thin extrusions";
    def.tooltip = "Thin walls and gapfill have very thin extrusions (often at the ends). Using these thin extrusions force the autospeed to have a very low average speed, as the maximum speed is used for the thinnest extrusion. This setting allow to have a threshold, enforcing the max speed to this flow. Any lower flow will still use the max speed, creating a difference in mm3/s for these but keeping a high speed for all others.\nTo keep the automatic computation for the Autospeed, but excluding the thin gap fill & thin wall from it, set this setting to 0.\nCan be a % of the maximum flow (compute from print's autospeed max volumetric flow and filament's max volumetric flow.";
    def.sidetext = "mm3/s or %";
    def.can_be_disabled = 1;
    def.has_min = 1;
    def.min_value = 0;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_ADVANCED | RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
    def.default_serialized_value = "0";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- bridge_speed | preset=RAW_PRESET_TYPE_FFF_PRINT ----
    def = raw_config_option_def_init();
    def.opt_key = "bridge_speed";
    def.type = RAW_CO_FLOAT_OR_PERCENT;
    def.container_type = RAW_CONTAINER_TYPE_REGION;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_SPEED;
    def.label = "Bridges";
    def.full_label = "Bridge Infill speed";
    def.tooltip = "Speed for printing bridges.\nThis can be expressed as a percentage (for example: 60%) over the Default speed.\nSet zero to use the autospeed for this feature";
    def.sidetext = "mm/s or %";
    def.ratio_over = "default_speed";
    def.has_min = 1;
    def.min_value = 0;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_ADVANCED | RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_PRUSA;
    {
        static const char *const k_aliases[] = { "bridge_feed_rate" };
        def.aliases = const_strings_t{ k_aliases, 1 };
    }
    def.default_serialized_value = "60%";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- brim_speed | preset=RAW_PRESET_TYPE_FFF_PRINT ----
    def = raw_config_option_def_init();
    def.opt_key = "brim_speed";
    def.type = RAW_CO_FLOAT_OR_PERCENT;
    def.container_type = RAW_CONTAINER_TYPE_OBJECT;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_SPEED;
    def.label = "Brim & Skirt";
    def.full_label = "Brim & Skirt speed";
    def.tooltip = "This separate setting will affect the speed of brim and skirt. \nIf expressed as percentage (for example: 80%) it will be calculated over the Support speed setting.\nSet zero to use autospeed for this feature.";
    def.sidetext = "mm/s or %";
    def.ratio_over = "support_material_speed";
    def.has_min = 1;
    def.min_value = 0;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
    def.default_serialized_value = "50%";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- external_perimeter_speed | preset=RAW_PRESET_TYPE_FFF_PRINT ----
    def = raw_config_option_def_init();
    def.opt_key = "external_perimeter_speed";
    def.type = RAW_CO_FLOAT_OR_PERCENT;
    def.container_type = RAW_CONTAINER_TYPE_REGION;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_SPEED;
    def.label = "External";
    def.full_label = "External perimeters speed";
    def.tooltip = "This separate setting will affect the speed of external perimeters (the visible ones). \nIf expressed as percentage (for example: 80%) it will be calculated over the Internal Perimeters speed setting.\nSet zero to use autospeed for this feature.";
    def.sidetext = "mm/s or %";
    def.ratio_over = "perimeter_speed";
    def.has_min = 1;
    def.min_value = 0;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_PRUSA;
    def.default_serialized_value = "50%";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- filament_max_speed | preset=RAW_PRESET_TYPE_FFF_FILAMENT ----
    def = raw_config_option_def_init();
    def.opt_key = "filament_max_speed";
    def.type = RAW_CO_VECTOR_FLOAT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_FILAMENT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_LAYER_EXTRUSION_EDIT;
    def.category = RAW_OPTION_CATEGORY_FILAMENT;
    def.label = "Max speed";
    def.tooltip = "Maximum speed allowed for this filament. Limits the maximum speed of a print to the minimum of the print speed and the filament speed. Set zero for no limit.";
    def.sidetext = "mm/s";
    def.is_vector_extruder = 1;
    def.has_min = 1;
    def.min_value = 0;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_ADVANCED | RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
    def.default_serialized_value = "0";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- filament_max_volumetric_speed | preset=RAW_PRESET_TYPE_FFF_FILAMENT ----
    def = raw_config_option_def_init();
    def.opt_key = "filament_max_volumetric_speed";
    def.type = RAW_CO_VECTOR_FLOAT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_FILAMENT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_LAYER_EXTRUSION_EDIT;
    def.category = RAW_OPTION_CATEGORY_FILAMENT;
    def.label = "Max volumetric speed";
    def.tooltip = "Maximum volumetric speed allowed for this filament. Limits the maximum volumetric speed of a print to the minimum of print and filament volumetric speed. Set zero for no limit.";
    def.sidetext = "mm³/s";
    def.is_vector_extruder = 1;
    def.has_min = 1;
    def.min_value = 0;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_ADVANCED | RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_PRUSA;
    def.default_serialized_value = "0";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- first_layer_flow_ratio | preset=RAW_PRESET_TYPE_FFF_PRINT ----
    def = raw_config_option_def_init();
    def.opt_key = "first_layer_flow_ratio";
    def.type = RAW_CO_PERCENT;
    def.container_type = RAW_CONTAINER_TYPE_REGION;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_WIDTH;
    def.label = "First layer";
    def.full_label = "First layer flow ratio";
    def.tooltip = "You can increase this to over-extrude on the first layer if there is not enough plastic because your bed isn't levelled.\nNote: DON'T USE THIS if your only problem is bed leveling, LEVEL YOUR BED! Use this setting only as last resort after all calibrations failed.";
    def.sidetext = "%";
    def.has_min = 1;
    def.min_value = 0;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
    def.default_serialized_value = "100%";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- first_layer_infill_speed | preset=RAW_PRESET_TYPE_FFF_PRINT ----
    def = raw_config_option_def_init();
    def.opt_key = "first_layer_infill_speed";
    def.type = RAW_CO_FLOAT_OR_PERCENT;
    def.container_type = RAW_CONTAINER_TYPE_OBJECT;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_SPEED;
    def.label = "Max infill";
    def.full_label = "Infill max first layer speed";
    def.tooltip = "If expressed as absolute value in mm/s, this speed will be applied as a maximum for all infill print moves of the first layer.\nIf expressed as a percentage it will scale the current infill speed.\nSet it at 100% to remove any infill first layer speed modification.\nSet zero to disable (using first_layer_speed instead).";
    def.sidetext = "mm/s or %";
    def.ratio_over = "depends";
    def.has_min = 1;
    def.min_value = 0;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
    def.default_serialized_value = "0";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- first_layer_min_speed | preset=RAW_PRESET_TYPE_FFF_PRINT ----
    def = raw_config_option_def_init();
    def.opt_key = "first_layer_min_speed";
    def.type = RAW_CO_FLOAT;
    def.container_type = RAW_CONTAINER_TYPE_OBJECT;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_SPEED;
    def.label = "Min";
    def.full_label = "Min first layer speed";
    def.tooltip = "Minimum speed when printing the first layer.\nSet zero to disable.";
    def.sidetext = "mm/s";
    def.has_min = 1;
    def.min_value = 0;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_ADVANCED | RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
    def.default_serialized_value = "0";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- first_layer_speed | preset=RAW_PRESET_TYPE_FFF_PRINT ----
    def = raw_config_option_def_init();
    def.opt_key = "first_layer_speed";
    def.type = RAW_CO_FLOAT_OR_PERCENT;
    def.container_type = RAW_CONTAINER_TYPE_OBJECT;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_SPEED;
    def.label = "Max";
    def.full_label = "Default first layer speed";
    def.tooltip = "If expressed as absolute value in mm/s, this speed will be applied as a maximum to all the print moves (but infill) of the first layer.\nIf expressed as a percentage it will scale the current speed.\nSet it at 100% to remove any first layer speed modification (but for infill).";
    def.sidetext = "mm/s or %";
    def.ratio_over = "depends";
    def.has_min = 1;
    def.min_value = 0;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_ADVANCED | RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_PRUSA;
    def.default_serialized_value = "30";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- gap_fill_speed | preset=RAW_PRESET_TYPE_FFF_PRINT ----
    def = raw_config_option_def_init();
    def.opt_key = "gap_fill_speed";
    def.type = RAW_CO_FLOAT_OR_PERCENT;
    def.container_type = RAW_CONTAINER_TYPE_REGION;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_SLICING;
    def.category = RAW_OPTION_CATEGORY_SPEED;
    def.label = "Gap fill";
    def.full_label = "Gap fill speed";
    def.tooltip = "Speed for filling small gaps using short zigzag moves. Keep this reasonably low to avoid too much shaking and resonance issues.\nGap fill extrusions are ignored from the automatic volumetric speed computation, unless you set it to 0.\nThis can be expressed as a percentage (for example: 80%) over the Internal Perimeter speed.";
    def.sidetext = "mm/s or %";
    def.ratio_over = "perimeter_speed";
    def.has_min = 1;
    def.min_value = 0;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_PRUSA;
    def.default_serialized_value = "50%";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- infill_speed | preset=RAW_PRESET_TYPE_FFF_PRINT ----
    def = raw_config_option_def_init();
    def.opt_key = "infill_speed";
    def.type = RAW_CO_FLOAT_OR_PERCENT;
    def.container_type = RAW_CONTAINER_TYPE_REGION;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_SPEED;
    def.label = "Sparse";
    def.full_label = "Sparse infill speed";
    def.tooltip = "Speed for printing the internal fill.\nThis can be expressed as a percentage (for example: 80%) over the Solid Infill speed.\nSet zero to use autospeed for this feature.";
    def.sidetext = "mm/s or %";
    def.ratio_over = "solid_infill_speed";
    def.has_min = 1;
    def.min_value = 0;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_ADVANCED | RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_PRUSA;
    {
        static const char *const k_aliases[] = { "print_feed_rate", "infill_feed_rate" };
        def.aliases = const_strings_t{ k_aliases, 2 };
    }
    def.default_serialized_value = "300%";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- internal_bridge_speed | preset=RAW_PRESET_TYPE_FFF_PRINT ----
    def = raw_config_option_def_init();
    def.opt_key = "internal_bridge_speed";
    def.type = RAW_CO_FLOAT_OR_PERCENT;
    def.container_type = RAW_CONTAINER_TYPE_REGION;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_SPEED;
    def.label = "Internal Bridge Infill speed";
    def.tooltip = "Speed for printing the bridges that support the top layer.\nCan be a % of the bridge speed.";
    def.sidetext = "mm/s or %";
    def.ratio_over = "bridge_speed";
    def.has_min = 1;
    def.min_value = 0;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
    {
        static const char *const k_aliases[] = { "bridge_speed_internal" };
        def.aliases = const_strings_t{ k_aliases, 1 };
    }
    def.default_serialized_value = "150%";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- ironing_speed | preset=RAW_PRESET_TYPE_FFF_PRINT ----
    def = raw_config_option_def_init();
    def.opt_key = "ironing_speed";
    def.type = RAW_CO_FLOAT_OR_PERCENT;
    def.container_type = RAW_CONTAINER_TYPE_REGION;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_IRONING;
    def.label = "Ironing";
    def.full_label = "Ironing speed";
    def.tooltip = "Ironing speed. Used for the ironing pass of the ironing infill pattern, and the post-process infill.\nThis can be expressed as a percentage (for example: 80%) over the Top Solid Infill speed.\nIroning extrusions are ignored from the automatic volumetric speed computation.";
    def.sidetext = "mm/s";
    def.ratio_over = "top_solid_infill_speed";
    def.has_min = 1;
    def.min_value = 0.10000000000000001;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_PRUSA;
    def.default_serialized_value = "50%";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- max_print_speed | preset=RAW_PRESET_TYPE_FFF_PRINT ----
    def = raw_config_option_def_init();
    def.opt_key = "max_print_speed";
    def.type = RAW_CO_FLOAT_OR_PERCENT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_SPEED;
    def.label = "Max Autospeed";
    def.full_label = "Max print speed for Autospeed";
    def.tooltip = "When setting other speed settings to 0, Slic3r will autocalculate the optimal speed in order to keep constant extruder pressure. This experimental setting is used to set the highest print speed you want to allow.\nThis can be expressed as a percentage (for example: 100%) over the machine Max Feedrate for X axis.";
    def.sidetext = "mm/s or %";
    def.ratio_over = "machine_max_feedrate_x";
    def.has_min = 1;
    def.min_value = 1;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_PRUSA;
    def.default_serialized_value = "80";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- max_volumetric_speed | preset=RAW_PRESET_TYPE_FFF_PRINT ----
    def = raw_config_option_def_init();
    def.opt_key = "max_volumetric_speed";
    def.type = RAW_CO_FLOAT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_EXTRUDERS;
    def.label = "Maximum flow for Autospeed";
    def.full_label = "Maximum Volumetric print speed for Autospeed";
    def.tooltip = "This setting allows you to set the maximum flowrate for your print, and so cap the desired flow rate for the autospeed algorithm. The autospeed tries to keep a constant feedrate for the entire object, and so can lower the volumetric speed for some features.\nThe autospeed is only enable on speed fields that have a value of 0. If a speed field is a % of a 0 field, then it will be a % of the value it should have got from the autospeed.\nIf this field is set to 0, then there is no autospeed nor maximum flowrate. If a speed value i still set to 0, it will get the max speed allwoed by the printer.";
    def.sidetext = "mm³/s";
    def.has_min = 1;
    def.min_value = 0;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_PRUSA;
    def.default_serialized_value = "0";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- milling_speed | preset=RAW_PRESET_TYPE_FFF_PRINT ----
    def = raw_config_option_def_init();
    def.opt_key = "milling_speed";
    def.type = RAW_CO_FLOAT;
    def.container_type = RAW_CONTAINER_TYPE_REGION;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_MILLING;
    def.label = "Milling Speed";
    def.tooltip = "Speed for milling tool.";
    def.sidetext = "mm/s";
    def.has_min = 1;
    def.min_value = 0;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_ADVANCED | RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
    def.default_serialized_value = "30";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- perimeter_speed | preset=RAW_PRESET_TYPE_FFF_PRINT ----
    def = raw_config_option_def_init();
    def.opt_key = "perimeter_speed";
    def.type = RAW_CO_FLOAT_OR_PERCENT;
    def.container_type = RAW_CONTAINER_TYPE_REGION;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_SPEED;
    def.label = "Internal";
    def.full_label = "Internal perimeters speed";
    def.tooltip = "Speed for perimeters (contours, aka vertical shells).\nThis can be expressed as a percentage (for example: 80%) over the Default speed.\nSet zero to use autospeed for this feature.";
    def.sidetext = "mm/s or %";
    def.ratio_over = "default_speed";
    def.has_min = 1;
    def.min_value = 0;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_ADVANCED | RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_PRUSA;
    {
        static const char *const k_aliases[] = { "perimeter_feed_rate" };
        def.aliases = const_strings_t{ k_aliases, 1 };
    }
    def.default_serialized_value = "60%";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- solid_infill_speed | preset=RAW_PRESET_TYPE_FFF_PRINT ----
    def = raw_config_option_def_init();
    def.opt_key = "solid_infill_speed";
    def.type = RAW_CO_FLOAT_OR_PERCENT;
    def.container_type = RAW_CONTAINER_TYPE_REGION;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_SPEED;
    def.label = "Solid";
    def.full_label = "Solid infill speed";
    def.tooltip = "Speed for printing solid regions (top/bottom/internal horizontal shells). \nThis can be expressed as a percentage (for example: 80%) over the Default speed.\nSet zero to use autospeed for this feature.";
    def.sidetext = "mm/s or %";
    def.ratio_over = "default_speed";
    def.has_min = 1;
    def.min_value = 0;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_ADVANCED | RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_PRUSA;
    {
        static const char *const k_aliases[] = { "solid_infill_feed_rate" };
        def.aliases = const_strings_t{ k_aliases, 1 };
    }
    def.default_serialized_value = "30%";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- thin_walls_speed | preset=RAW_PRESET_TYPE_FFF_PRINT ----
    def = raw_config_option_def_init();
    def.opt_key = "thin_walls_speed";
    def.type = RAW_CO_FLOAT_OR_PERCENT;
    def.container_type = RAW_CONTAINER_TYPE_REGION;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_SPEED;
    def.label = "Thin walls";
    def.full_label = "Thin walls speed";
    def.tooltip = "Speed for thin walls (external extrusions that are alone because the object is too thin at these places).\nThis can be expressed as a percentage (for example: 80%) over the External Perimeter speed.\nSet zero to use autospeed for this feature.";
    def.sidetext = "mm/s or %";
    def.ratio_over = "external_perimeter_speed";
    def.has_min = 1;
    def.min_value = 0;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
    def.default_serialized_value = "100%";
    orchestrator_create_option_def(orchestrator, &def);

    // ---- top_solid_infill_speed | preset=RAW_PRESET_TYPE_FFF_PRINT ----
    def = raw_config_option_def_init();
    def.opt_key = "top_solid_infill_speed";
    def.type = RAW_CO_FLOAT_OR_PERCENT;
    def.container_type = RAW_CONTAINER_TYPE_REGION;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_SPEED;
    def.label = "Top solid";
    def.full_label = "Top solid speed";
    def.tooltip = "Speed for printing top solid layers (it only applies to the uppermost external layers and not to their internal solid layers). You may want to slow down this to get a nicer surface finish.\nThis can be expressed as a percentage (for example: 80%) over the Solid Infill speed.\nSet zero to use autospeed for this feature.";
    def.sidetext = "mm/s or %";
    def.ratio_over = "solid_infill_speed";
    def.has_min = 1;
    def.min_value = 0;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_PRUSA;
    def.default_serialized_value = "50%";
    orchestrator_create_option_def(orchestrator, &def);
}

void add_default_speed_gui_rules(orchestrator_handle *orchestrator)
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
    // `has_infill_or_solid` widens `has_solid` with any sparse infill, and
    // `brim_or_skirt` requires a brim or a skirt. `ironing_pp` requires ironing
    // on a top surface, and `ironing_any` also accepts a smooth top, bottom or
    // solid fill pattern. gui_rules.core keeps equivalent copies of these
    // conditions for the settings it hosts.
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
    const GuiCondition has_infill_or_solid = any_of({ has_solid, non_zero("fill_density") });
    const GuiCondition ironing_pp = all_of({ has_top, true_("ironing") });
    const GuiCondition ironing_any = any_of({
        ironing_pp,
        fill_pattern_is_smooth("top_fill_pattern"),
        fill_pattern_is_smooth("bottom_fill_pattern"),
        fill_pattern_is_smooth("solid_fill_pattern")
    });
    const GuiCondition brim_or_skirt = any_of({
        non_zero("brim_width"),
        non_zero("brim_width_interior"),
        non_zero("skirts")
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

    add_rule({ "perimeter_speed", "external_perimeter_speed" }, non_zero("perimeters"));
    add_rule({ "solid_infill_speed" }, has_solid);
    add_rule({ "infill_speed" }, has_infill_or_solid);
    add_rule({ "top_solid_infill_speed" }, has_top);
    add_rule({ "ironing_speed" }, ironing_any);
    add_rule({ "brim_speed" }, brim_or_skirt);
    add_rule({ "milling_speed" }, true_("milling_post_process"));
    add_rule({ "max_print_speed", "autospeed_min_thin_flow" }, non_zero("max_volumetric_speed"));
}

const RegionSettings::OptionKeyGroup k_region_speed_keys = {
    "bridge_speed",
    "external_perimeter_speed",
    "gap_fill_speed",
    "infill_speed",
    "internal_bridge_speed",
    "ironing_speed",
    "milling_speed",
    "overhangs",
    "overhangs_speed",
    "perimeter_speed",
    "solid_infill_speed",
    "thin_walls_speed",
    "top_solid_infill_speed"
};

using AutospeedTargets = std::vector<std::map<extruder_index_t, double>>;
using RegionalSettingsPartitions = std::map<RegionalSourceKey, RegionalSettingsPartition>;

struct AutospeedPartitionKey
{
    RegionalSourceKey source;
    extruder_index_t extruder_id = EXTRUDER_ID_UNKNOWN;
    raw_extrusion_role role = RAW_EXTRUSION_ROLE_NONE;
    bool full_overhang_speed = false;

    bool operator<(const AutospeedPartitionKey &rhs) const;
};

struct SpeedPartitionKey
{
    RegionalSourceKey source;
    uint32_t group_idx = 0;
    extruder_index_t extruder_id = EXTRUDER_ID_UNKNOWN;
    raw_extrusion_role role = RAW_EXTRUSION_ROLE_NONE;
    bool full_overhang_speed = false;
    double mm3_per_mm = 0.0;
    bool has_volumetric_flow = false;

    bool operator<(const SpeedPartitionKey &rhs) const;
};

using AutospeedPartitions = std::map<AutospeedPartitionKey, RegionalProcessPartition>;
using SpeedPartitions = std::map<SpeedPartitionKey, RegionalProcessPartition>;

// Resolve the role speed through the legacy percentage fallback hierarchy.
double role_speed(const ExtrusionSettingsContext &context,
                  raw_extrusion_role role,
                  bool full_overhang_speed,
                  double auto_base);

// Apply first-layer rules and physical print/filament caps after role choice.
double finalize_speed(const ExtrusionSettingsContext &context,
                      raw_extrusion_role role,
                      double mm3_per_mm,
                      bool has_volumetric_flow,
                      double speed);

// Resolve the exact float later written to one leaf or regional fragment.
float resolved_speed(const ExtrusionSettingsContext &context,
                     raw_extrusion_role role,
                     bool full_overhang_speed,
                     double mm3_per_mm,
                     bool has_volumetric_flow,
                     double autospeed_target);

// Resolve travel speed independently from printable-role and volumetric rules.
float resolved_travel_speed(const ExtrusionSettingsContext &context);

// Scan unresolved eligible leaves for the group's smallest flow cross-section.
void collect_autospeed_minimum(const MutableExtrusionEntity &entity,
                              const EffectiveTreeState &parent_state,
                              const ExtrusionSettingsContext &context,
                              storage_handle *storage,
                              const RegionalSourceKey &source,
                              const RegionalSettingsPartition &regional_settings,
                              AutospeedPartitions &regional_partitions,
                              bool exclude_thin_flows,
                              double &minimum,
                              const plugin_run_context *run_ctx);

// Prepare raw/final regional partitions and group/extruder autospeed targets.
// Borrow the plan, leave region-less synthetic roots untouched, and own cached speed outputs; invalid regional sources or cancellation throw.
void prepare_plan(storage_handle *storage,
                  const Print &print,
                  PrintingPlan &plan,
                  AutospeedTargets &targets,
                  SpeedPartitions &speed_partitions,
                  const plugin_run_context *run_ctx);

// Build final speed partitions required by unresolved leaves in one tree.
void collect_speed_partitions(MutableExtrusionEntity entity,
                              const EffectiveTreeState &parent_state,
                              const ExtrusionSettingsContext &context,
                              const RegionalSourceKey &source,
                              const RegionalSettingsPartition &regional_settings,
                              storage_handle *storage,
                              uint32_t group_idx,
                              double autospeed_target,
                              std::set<SpeedPartitionKey> &prepared_keys,
                              SpeedPartitions &speed_partitions,
                              const plugin_run_context *run_ctx);

// Assign missing speed recursively while preserving inherited overrides.
void assign_speed_tree(MutableExtrusionEntity entity,
                       const EffectiveTreeState &parent_state,
                       const ExtrusionSettingsContext &context,
                       uint32_t group_idx,
                       const RegionalSourceKey &source,
                       double autospeed_target,
                       const SpeedPartitions &speed_partitions,
                       EPropertySpeedFieldEditor &editor,
                       const plugin_run_context *run_ctx);

// Return a target for one group/extruder, or zero when autospeed is unavailable.
double autospeed_target(const AutospeedTargets &targets,
                        uint32_t group_idx,
                        extruder_index_t extruder_id);

// Resolve missing Travel connector speeds without editing synthetic extrusion speeds.
void assign_synthetic_travel_speed(MutableExtrusionEntity entity, const EffectiveTreeState &parent_state,
                                   const ExtrusionSettingsContext &context, EPropertySpeedFieldEditor &editor,
                                   const plugin_run_context *run_ctx);

// Resolve regional speeds or synthetic Travel connector speeds on one cloned root.
void edit_extrusion(const Print &print,
                    const PrintingPlan &plan,
                    PrintingToolGroup tool_group,
                    PrintingExtrusion extrusion,
                    uint32_t group_idx,
                    double target,
                    const SpeedPartitions &speed_partitions,
                    const plugin_run_context *run_ctx);

class DefaultSpeed : public PluginBase
{
public:
    static DefaultSpeed &instance(orchestrator_handle *orch);
    explicit DefaultSpeed(orchestrator_handle *orch);

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

    mutable AutospeedTargets m_autospeed_targets;
    mutable SpeedPartitions m_speed_partitions;
    mutable bool m_setup_valid = false;
};

bool AutospeedPartitionKey::operator<(const AutospeedPartitionKey &rhs) const
{
    if (source < rhs.source)
        return true;
    if (rhs.source < source)
        return false;
    if (extruder_id != rhs.extruder_id)
        return extruder_id < rhs.extruder_id;
    if (role != rhs.role)
        return uint16_t(role) < uint16_t(rhs.role);
    return full_overhang_speed < rhs.full_overhang_speed;
}

bool SpeedPartitionKey::operator<(const SpeedPartitionKey &rhs) const
{
    if (source < rhs.source)
        return true;
    if (rhs.source < source)
        return false;
    if (group_idx != rhs.group_idx)
        return group_idx < rhs.group_idx;
    if (extruder_id != rhs.extruder_id)
        return extruder_id < rhs.extruder_id;
    if (role != rhs.role)
        return uint16_t(role) < uint16_t(rhs.role);
    if (full_overhang_speed != rhs.full_overhang_speed)
        return full_overhang_speed < rhs.full_overhang_speed;
    if (mm3_per_mm != rhs.mm3_per_mm)
        return mm3_per_mm < rhs.mm3_per_mm;
    return has_volumetric_flow < rhs.has_volumetric_flow;
}

double role_speed(const ExtrusionSettingsContext &context,
                  raw_extrusion_role role,
                  bool full_overhang_speed,
                  double auto_base)
{
    // Build the shared fallback chain once. Role-specific percentages are
    // evaluated against the preceding value, ending at the autospeed base.
    const Config &region = context.region_config;
    const double perimeter = effective_value(region, "perimeter_speed", auto_base);
    const double external = effective_value(region, "external_perimeter_speed", perimeter);
    const double solid = effective_value(region, "solid_infill_speed", auto_base);
    const double bridge = effective_value(region, "bridge_speed", auto_base);
    // Support speed follows the autospeed base. It is also the base of the
    // interface and brim/skirt percentages.
    const double support = effective_value(context.object_config, "support_material_speed", auto_base);

    // Resolve roles outside the perimeter and infill families first. For
    // brim/skirt and support interface, a percentage refines the support
    // speed, while zero means autospeed for that feature itself, whatever the
    // support speed is.
    if (RAW_EXTRUSION_ROLE_IS_SKIRT(role)) {
        const double brim = context.object_config.effective_float_or_percent_or_default(
            "brim_speed", support, 0.0);
        return brim > 0.0 ? brim : auto_base;
    }
    if (RAW_EXTRUSION_ROLE_IS_SUPPORT(role)) {
        if (!RAW_EXTRUSION_ROLE_IS_SUPPORT_INTERFACE(role))
            return support;
        const double interface_speed = context.object_config.effective_float_or_percent_or_default(
            "support_material_interface_speed", support, 0.0);
        return interface_speed > 0.0 ? interface_speed : auto_base;
    }
    if (RAW_EXTRUSION_ROLE_HAS(role, RAW_EXTRUSION_ROLE_MILL))
        return region.float_or_default("milling_speed", 0.0);
    if (role == RAW_EXTRUSION_ROLE_GAP_FILL)
        return effective_value(region, "gap_fill_speed", perimeter);
    if (role == RAW_EXTRUSION_ROLE_THIN_WALL)
        return effective_value(region, "thin_walls_speed", external);
    if (RAW_EXTRUSION_ROLE_HAS(role, RAW_EXTRUSION_ROLE_IRONING))
        return effective_value(region, "ironing_speed",
                               effective_value(region, "top_solid_infill_speed", solid));

    // Infill refines solid and bridge bases according to its exact role.
    if (RAW_EXTRUSION_ROLE_IS_INFILL(role)) {
        if (RAW_EXTRUSION_ROLE_IS_BRIDGE(role))
            return RAW_EXTRUSION_ROLE_IS_EXTERNAL(role) ? bridge :
                effective_value(region, "internal_bridge_speed", bridge);
        if (RAW_EXTRUSION_ROLE_HAS(role, RAW_EXTRUSION_ROLE_SOLID))
            return RAW_EXTRUSION_ROLE_IS_EXTERNAL(role) ?
                effective_value(region, "top_solid_infill_speed", solid) : solid;
        return effective_value(region, "infill_speed", solid);
    }

    // Fully unsupported perimeters use overhang speed; other perimeters keep
    // their internal or external fallback.
    if (RAW_EXTRUSION_ROLE_IS_PERIMETER(role)) {
        const bool use_overhang = full_overhang_speed ||
            (RAW_EXTRUSION_ROLE_IS_BRIDGE(role) && region.bool_or_default("overhangs", false));
        if (use_overhang)
            return effective_value(region, "overhangs_speed", bridge);
        return RAW_EXTRUSION_ROLE_IS_EXTERNAL(role) ? external : perimeter;
    }
    return 0.0;
}

double finalize_speed(const ExtrusionSettingsContext &context,
                      raw_extrusion_role role,
                      double mm3_per_mm,
                      bool has_volumetric_flow,
                      double speed)
{
    // Apply first-layer limits after role selection. Infill may use a distinct
    // cap, while all remaining roles share first_layer_speed.
    if (context.first_layer) {
        const double first_layer_speed = effective_value(context.object_config, "first_layer_speed", speed);
        if (role == RAW_EXTRUSION_ROLE_INTERNAL_INFILL || role == RAW_EXTRUSION_ROLE_SOLID_INFILL) {
            const double infill_speed = effective_value(
                context.object_config, "first_layer_infill_speed", speed);
            const double limit = infill_speed > 0.0 ? infill_speed : first_layer_speed;
            if (limit > 0.0)
                speed = std::min(speed, limit);
        } else if (first_layer_speed > 0.0) {
            speed = std::min(speed, first_layer_speed);
        }

        // The minimum follows the cap. The flow ratio affects only the
        // volumetric ceilings converted below.
        speed = std::max(speed, context.object_config.float_or_default("first_layer_min_speed", 0.0));
        if (has_volumetric_flow)
            mm3_per_mm *= effective_value(context.object_config, "first_layer_flow_ratio", 1.0);
    }

    // Apply each independent linear or volumetric ceiling without ever raising
    // the speed selected by the role hierarchy.
    const double max_print_speed = context.print_config.computed_float_or_default(
        "max_print_speed", int32_t(context.extruder_id), 0.0);
    if (max_print_speed > 0.0)
        speed = std::min(speed, max_print_speed);

    const double max_volumetric = context.print_config.float_or_default("max_volumetric_speed", 0.0);
    if (has_volumetric_flow && max_volumetric > 0.0 && mm3_per_mm > 0.0)
        speed = std::min(speed, max_volumetric / mm3_per_mm);

    const double filament_max_volumetric = context.print_config.vector_float_or_default(
        "filament_max_volumetric_speed", context.extruder_id, 0.0);
    if (has_volumetric_flow && filament_max_volumetric > 0.0 && mm3_per_mm > 0.0)
        speed = std::min(speed, filament_max_volumetric / mm3_per_mm);

    const double filament_max_speed = context.print_config.vector_float_or_default(
        "filament_max_speed", context.extruder_id, 0.0);
    if (filament_max_speed > 0.0)
        speed = std::min(speed, filament_max_speed);
    return speed;
}

float resolved_speed(const ExtrusionSettingsContext &context,
                     raw_extrusion_role role,
                     bool full_overhang_speed,
                     double mm3_per_mm,
                     bool has_volumetric_flow,
                     double autospeed_target_value)
{
    double speed = role_speed(context, role, full_overhang_speed, 0.0);

    // Convert a shared volumetric target into the linear base appropriate for
    // this exact flow, then reevaluate role percentages against that base.
    if (speed <= 0.0 && autospeed_target_value > 0.0 && has_volumetric_flow) {
        const double max_print_speed = context.print_config.computed_float_or_default(
            "max_print_speed", int32_t(context.extruder_id), 0.0);
        const double auto_base = std::min(autospeed_target_value / mm3_per_mm, max_print_speed);
        speed = role_speed(context, role, full_overhang_speed, auto_base);
        if (speed <= 0.0)
            speed = auto_base;
    }

    // With no volumetric target, max_print_speed remains the last legacy
    // fallback before first-layer and physical ceilings are applied.
    if (speed <= 0.0)
        speed = context.print_config.computed_float_or_default(
            "max_print_speed", int32_t(context.extruder_id), 0.0);
    if (speed <= 0.0 || !std::isfinite(speed))
        throw std::runtime_error("Unable to resolve a positive finite extrusion speed.");

    if (!RAW_EXTRUSION_ROLE_HAS(role, RAW_EXTRUSION_ROLE_MILL))
        speed = finalize_speed(context, role, mm3_per_mm, has_volumetric_flow, speed);
    if (speed <= 0.0 || !std::isfinite(speed))
        throw std::runtime_error("Extrusion speed limits produced an invalid value.");
    const float stored_speed = float(speed);
    if (!std::isfinite(stored_speed))
        throw std::runtime_error("Extrusion speed cannot be represented by the stored float.");
    return stored_speed;
}

float resolved_travel_speed(const ExtrusionSettingsContext &context)
{
    // Travel speed is a machine movement setting. It must not inherit role
    // percentages or be reduced by material-flow and first-layer ceilings.
    const double speed = context.print_config.float_or_default("travel_speed", 0.0);
    if (speed <= 0.0 || !std::isfinite(speed))
        throw std::runtime_error("Unable to resolve a positive finite travel speed.");
    const float stored_speed = float(speed);
    if (!std::isfinite(stored_speed))
        throw std::runtime_error("Travel speed cannot be represented by the stored float.");
    return stored_speed;
}

void collect_autospeed_minimum(const MutableExtrusionEntity &entity,
                              const EffectiveTreeState &parent_state,
                              const ExtrusionSettingsContext &context,
                              storage_handle *storage,
                              const RegionalSourceKey &source,
                              const RegionalSettingsPartition &regional_settings,
                              AutospeedPartitions &regional_partitions,
                              bool exclude_thin_flows,
                              double &minimum,
                              const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    // Walk to leaves with inherited properties resolved because intermediate
    // collection nodes do not carry a printable flow cross-section.
    const EffectiveTreeState state = effective_state(entity, parent_state);
    if (entity.child_count() > 0) {
        for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx) {
            throw_if_cancelled(run_ctx);
            collect_autospeed_minimum(entity.child_mutable(child_idx), state, context,
                                      storage, source, regional_settings, regional_partitions,
                                      exclude_thin_flows, minimum, run_ctx);
        }
        return;
    }

    // Existing speed and roles with an explicit positive setting do not depend
    // on the shared target and therefore must not influence it.
    if (leaf_disposition(entity, state) != LeafDisposition::Editable || state.speed > 0.f)
        return;
    const raw_extrusion_role role = state.attributes.extrusion_role();
    if (state.attributes.c_extrusion_property_attributes::height == -2.f ||
        RAW_EXTRUSION_ROLE_HAS(role, RAW_EXTRUSION_ROLE_MILL) ||
        RAW_EXTRUSION_ROLE_HAS(role, RAW_EXTRUSION_ROLE_IRONING) ||
        (exclude_thin_flows && (role == RAW_EXTRUSION_ROLE_GAP_FILL || role == RAW_EXTRUSION_ROLE_THIN_WALL)))
        return;

    // Resolve auto-vs-explicit speed for every regional tuple, then split a
    // deep clone to inspect only the zones actually crossed by this leaf.
    const AutospeedPartitionKey partition_key{
        source, context.extruder_id, role, state.full_overhang_speed
    };
    throw_if_cancelled(run_ctx);
    AutospeedPartitions::iterator partition_it = regional_partitions.find(partition_key);
    if (partition_it == regional_partitions.end()) {
        throw_if_cancelled(run_ctx);
        RegionalProcessPartition partition = project_regional_process_values(
            storage, regional_settings,
            [&context, role, &state](const Config &region_config) {
                ExtrusionSettingsContext regional_context = context;
                regional_context.region_config = region_config;
                return float(role_speed(
                    regional_context, role, state.full_overhang_speed, 0.0));
            });
        partition_it = regional_partitions.emplace(partition_key, std::move(partition)).first;
    }
    if (!regional_partition_leaf_matches(
            storage, entity, partition_it->second,
            [](float regional_speed) { return regional_speed <= 0.f; }))
        return;

    // The thinnest eligible flow defines the linear speed required to reach a
    // common volumetric target.
    const double mm3_per_mm = state.attributes.c_extrusion_property_attributes::mm3_per_mm;
    throw_if_cancelled(run_ctx);
    if (mm3_per_mm > 0.0)
        minimum = std::min(minimum, mm3_per_mm);
}

// Borrow the plan, validate regional sources, and populate owned partitions/targets; synthetic region-less output stays unchanged and invalid sources or cancellation throw.
void prepare_plan(storage_handle *storage,
                  const Print &print,
                  PrintingPlan &plan,
                  AutospeedTargets &targets,
                  SpeedPartitions &speed_partitions,
                  const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    // Targets are isolated by printing group and extruder so complete-object
    // groups and different filaments cannot influence each other.
    targets.clear();
    targets.resize(plan.group_count());
    speed_partitions.clear();
    RegionalSettingsPartitions regional_settings;
    AutospeedPartitions autospeed_partitions;
    const Config print_config = print.config();
    const double maximum_volumetric = print_config.float_or_default("max_volumetric_speed", 0.0);
    const bool thin_floor_enabled = print_config.has("autospeed_min_thin_flow") &&
        print_config.get("autospeed_min_thin_flow").is_enabled();

    for (uint32_t group_idx = 0; group_idx < plan.group_count(); ++group_idx) {
        throw_if_cancelled(run_ctx);
        PrintingGroup group = plan.group(group_idx);
        std::map<extruder_index_t, double> minimum_by_extruder;

        // Validate the entire plan before any parallel mutation. During the
        // same pass, collect the minimum flow when autospeed is enabled.
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
                        print, tool_group, extrusion, true);
                    const LayerRegionIsland region_island = extrusion.region_island();
                    const size_t object_instance_idx = printing_plan_object_instance_idx(plan, extrusion);
                    if (object_instance_idx < size_t(-1) && object_instance_idx > std::numeric_limits<uint16_t>::max())
                        throw std::runtime_error("Speed source has no valid printing-plan instance.");
                    const RegionalSourceKey source{region_island.handle(), object_instance_idx};

                    // Build raw RegionSettings geometry once per source and
                    // translated object instance. Both autospeed analysis and
                    // final speed projection consume this immutable result.
                    RegionalSettingsPartitions::iterator settings_it = regional_settings.find(source);
                    if (settings_it == regional_settings.end()) {
                        throw_if_cancelled(run_ctx);
                        RegionalSettingsPartition settings = build_regional_settings_partition(
                            storage, region_island, k_region_speed_keys, plan.object_instance_shift(extrusion));
                        settings_it = regional_settings.emplace(source, std::move(settings)).first;
                    }
                    if (maximum_volumetric <= 0.0)
                        continue;

                    double &minimum = minimum_by_extruder[tool_group.extruder_id()];
                    if (minimum == 0.0)
                        minimum = (std::numeric_limits<double>::max)();
                    collect_autospeed_minimum(extrusion.mutable_root(), EffectiveTreeState{}, context,
                                              storage, source, settings_it->second,
                                              autospeed_partitions, thin_floor_enabled, minimum,
                                              run_ctx);
                }
            }
        }

        // Without a volumetric ceiling, assignment uses max_print_speed as its
        // final fallback and no autospeed target is materialized.
        if (maximum_volumetric <= 0.0)
            continue;

        for (std::map<extruder_index_t, double>::value_type &entry : minimum_by_extruder) {
            throw_if_cancelled(run_ctx);
            const extruder_index_t extruder_id = entry.first;
            const double max_print_speed = print_config.computed_float_or_default(
                "max_print_speed", int32_t(extruder_id), 0.0);
            if (max_print_speed <= 0.0)
                throw std::runtime_error("Autospeed requires a positive max_print_speed.");

            double minimum = entry.second;
            // An enabled thin-flow floor replaces geometric sampling with a
            // configured flow derived from the active volumetric ceiling.
            if (thin_floor_enabled) {
                const double filament_limit = print_config.vector_float_or_default(
                    "filament_max_volumetric_speed", extruder_id, 0.0);
                const double flow_limit = filament_limit > 0.0 ?
                    std::min(maximum_volumetric, filament_limit) : maximum_volumetric;
                const double configured_floor = print_config.effective_float_or_percent_or_default(
                    "autospeed_min_thin_flow", flow_limit, 0.0);
                if (configured_floor > 0.0)
                    minimum = configured_floor / max_print_speed;
            }

            if (minimum != (std::numeric_limits<double>::max)() && minimum > 0.0)
                targets[group_idx][extruder_id] =
                    std::min(minimum * max_print_speed, maximum_volumetric);
        }
    }

    // Autospeed targets are now stable. Project every unresolved leaf's raw
    // regional tuples through the complete final-speed calculation and retain
    // only partitions that can actually produce distinct stored floats.
    std::set<SpeedPartitionKey> prepared_keys;
    for (uint32_t group_idx = 0; group_idx < plan.group_count(); ++group_idx) {
        throw_if_cancelled(run_ctx);
        PrintingGroup group = plan.group(group_idx);
        for (uint32_t layer_idx = 0; layer_idx < group.layer_group_count(); ++layer_idx) {
            throw_if_cancelled(run_ctx);
            PrintingLayerGroup layer_group = group.layer_group(layer_idx);
            for (uint32_t tool_idx = 0; tool_idx < layer_group.tool_group_count(); ++tool_idx) {
                throw_if_cancelled(run_ctx);
                PrintingToolGroup tool_group = layer_group.tool_group(tool_idx);
                const double target = autospeed_target(targets, group_idx, tool_group.extruder_id());
                for (uint32_t extrusion_idx = 0; extrusion_idx < tool_group.extrusion_count(); ++extrusion_idx) {
                    throw_if_cancelled(run_ctx);
                    PrintingExtrusion extrusion = tool_group.extrusion(extrusion_idx);
                    if (is_regionless_synthetic(plan, extrusion))
                        continue;
                    const ExtrusionSettingsContext context = settings_context(print, tool_group, extrusion);
                    const size_t object_instance_idx = printing_plan_object_instance_idx(plan, extrusion);
                    if (object_instance_idx < size_t(-1) && object_instance_idx > std::numeric_limits<uint16_t>::max())
                        throw std::runtime_error("Speed source has no valid printing-plan instance.");
                    const RegionalSourceKey source{extrusion.region_island().handle(), object_instance_idx};
                    const RegionalSettingsPartitions::const_iterator settings_it =
                        regional_settings.find(source);
                    if (settings_it == regional_settings.end())
                        throw std::runtime_error("Validated speed source has no regional settings partition.");

                    collect_speed_partitions(
                        extrusion.mutable_root(), EffectiveTreeState{}, context, source,
                        settings_it->second, storage, group_idx, target, prepared_keys,
                        speed_partitions, run_ctx);
                }
            }
        }
    }
}

void collect_speed_partitions(MutableExtrusionEntity entity,
                              const EffectiveTreeState &parent_state,
                              const ExtrusionSettingsContext &context,
                              const RegionalSourceKey &source,
                              const RegionalSettingsPartition &regional_settings,
                              storage_handle *storage,
                              uint32_t group_idx,
                              double autospeed_target_value,
                              std::set<SpeedPartitionKey> &prepared_keys,
                              SpeedPartitions &speed_partitions,
                              const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    const EffectiveTreeState state = effective_state(entity, parent_state);
    if (entity.child_count() > 0) {
        for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx) {
            throw_if_cancelled(run_ctx);
            collect_speed_partitions(
                entity.child_mutable(child_idx), state, context, source, regional_settings,
                storage, group_idx, autospeed_target_value, prepared_keys, speed_partitions,
                run_ctx);
        }
        return;
    }

    if (leaf_disposition(entity, state) != LeafDisposition::Editable || state.speed > 0.f)
        return;

    const raw_extrusion_role role = state.attributes.extrusion_role();
    const double raw_mm3_per_mm =
        state.attributes.c_extrusion_property_attributes::mm3_per_mm;
    const bool has_volumetric_flow =
        state.attributes.c_extrusion_property_attributes::height != -2.f &&
        raw_mm3_per_mm > 0.0;
    const double mm3_per_mm = has_volumetric_flow ? raw_mm3_per_mm : 0.0;
    const SpeedPartitionKey key{
        source, group_idx, context.extruder_id, role, state.full_overhang_speed,
        mm3_per_mm, has_volumetric_flow
    };
    throw_if_cancelled(run_ctx);
    if (!prepared_keys.insert(key).second)
        return;

    // Resolve every regional tuple with the same flow and autospeed inputs as
    // the future leaf. Equal final floats are united by the common projector.
    RegionalProcessPartition partition = project_regional_process_values(
        storage, regional_settings,
        [&context, role, &state, mm3_per_mm, has_volumetric_flow,
         autospeed_target_value](const Config &region_config) {
            ExtrusionSettingsContext regional_context = context;
            regional_context.region_config = region_config;
            return resolved_speed(
                regional_context, role, state.full_overhang_speed, mm3_per_mm,
                has_volumetric_flow, autospeed_target_value);
        });
    throw_if_cancelled(run_ctx);
    if (partition.requires_split())
        speed_partitions.emplace(key, std::move(partition));
}

void assign_speed_tree(MutableExtrusionEntity entity,
                       const EffectiveTreeState &parent_state,
                       const ExtrusionSettingsContext &context,
                       uint32_t group_idx,
                       const RegionalSourceKey &source,
                       double autospeed_target_value,
                       const SpeedPartitions &speed_partitions,
                       EPropertySpeedFieldEditor &editor,
                       const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    // Collections only propagate inherited state. Values are assigned to
    // leaves first so hoisting can later prove uniformity.
    const EffectiveTreeState state = effective_state(entity, parent_state);
    if (entity.child_count() > 0) {
        for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx) {
            throw_if_cancelled(run_ctx);
            assign_speed_tree(entity.child_mutable(child_idx), state, context,
                              group_idx, source, autospeed_target_value,
                              speed_partitions, editor, run_ctx);
        }
        return;
    }

    const LeafDisposition disposition = leaf_disposition(entity, state);
    if (disposition == LeafDisposition::Empty || state.speed > 0.f)
        return;

    const raw_extrusion_role role = state.attributes.extrusion_role();
    // Travels are intentionally excluded from regional and autospeed setup,
    // but still need a concrete process speed before firmware serialization.
    if (RAW_EXTRUSION_ROLE_IS_TRAVEL(role)) {
        throw_if_cancelled(run_ctx);
        editor.set_value(entity, resolved_travel_speed(context));
        return;
    }
    if (disposition != LeafDisposition::Editable)
        return;

    const double raw_mm3_per_mm =
        state.attributes.c_extrusion_property_attributes::mm3_per_mm;
    const bool has_volumetric_flow =
        state.attributes.c_extrusion_property_attributes::height != -2.f &&
        raw_mm3_per_mm > 0.0;
    const double mm3_per_mm = has_volumetric_flow ? raw_mm3_per_mm : 0.0;
    const SpeedPartitionKey key{
        source, group_idx, context.extruder_id, role, state.full_overhang_speed,
        mm3_per_mm, has_volumetric_flow
    };
    const SpeedPartitions::const_iterator partition_it = speed_partitions.find(key);
    if (partition_it != speed_partitions.end()) {
        // The common splitter applies the already-final values to only the
        // regional fragments touched by this leaf.
        throw_if_cancelled(run_ctx);
        apply_regional_process_partition(entity, partition_it->second, editor);
        return;
    }

    throw_if_cancelled(run_ctx);
    editor.set_value(entity, resolved_speed(
        context, role, state.full_overhang_speed, mm3_per_mm,
        has_volumetric_flow, autospeed_target_value));
}

double autospeed_target(const AutospeedTargets &targets,
                        uint32_t group_idx,
                        extruder_index_t extruder_id)
{
    if (group_idx >= targets.size())
        return 0.0;
    const std::map<extruder_index_t, double>::const_iterator found = targets[group_idx].find(extruder_id);
    return found == targets[group_idx].end() ? 0.0 : found->second;
}

// Traverse a borrowed synthetic tree with inherited process fields, assigning
// machine travel speed only to geometric Travel leaves lacking an effective
// speed. Preserve service extrusion speeds and all other fields without hoisting
// or retaining views; invalid travel settings and cancellation propagate.
void assign_synthetic_travel_speed(MutableExtrusionEntity entity, const EffectiveTreeState &parent_state,
                                   const ExtrusionSettingsContext &context, EPropertySpeedFieldEditor &editor,
                                   const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    const EffectiveTreeState state = effective_state(entity, parent_state);
    for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx)
        assign_synthetic_travel_speed(entity.child_mutable(child_idx), state, context, editor, run_ctx);
    if (entity.segment_count() != 0 && state.speed <= 0.f && state.has_attributes &&
        RAW_EXTRUSION_ROLE_IS_TRAVEL(state.attributes.extrusion_role()))
        editor.set_value(entity, resolved_travel_speed(context));
}

// Borrow one plan-owned root and resolve synthetic Travel connectors from print
// settings while preserving service extrusion fields. Real-object roots use
// their regional context and hoist uniform resolved speed. Invalid sources,
// travel settings or cancellation throw without transferring root ownership.
void edit_extrusion(const Print &print,
                    const PrintingPlan &plan,
                    PrintingToolGroup tool_group,
                    PrintingExtrusion extrusion,
                    uint32_t group_idx,
                    double target,
                    const SpeedPartitions &speed_partitions,
                    const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    // Assign missing leaf speed, then remove redundant direct fields by moving
    // a uniform effective speed towards the root.
    if (is_regionless_synthetic(plan, extrusion)) {
        ExtrusionSettingsContext context;
        context.print_config = print.config();
        EPropertySpeedFieldEditor editor(EPropertySpeedField::Speed);
        assign_synthetic_travel_speed(extrusion.mutable_root(), EffectiveTreeState{}, context, editor, run_ctx);
        return;
    }
    const ExtrusionSettingsContext context = settings_context(print, tool_group, extrusion);
    const size_t object_instance_idx = printing_plan_object_instance_idx(plan, extrusion);
    if (object_instance_idx < size_t(-1) && object_instance_idx > std::numeric_limits<uint16_t>::max())
        throw std::runtime_error("Speed source has no valid printing-plan instance.");
    const RegionalSourceKey source{extrusion.region_island().handle(), static_cast<uint16_t>(object_instance_idx)};
    MutableExtrusionEntity root = extrusion.mutable_root();
    EPropertySpeedFieldEditor editor(EPropertySpeedField::Speed);
    assign_speed_tree(root, EffectiveTreeState{}, context, group_idx, source, target,
                      speed_partitions, editor, run_ctx);
    throw_if_cancelled(run_ctx);
    editor.hoist(root);
}

DefaultSpeed &DefaultSpeed::instance(orchestrator_handle *orch)
{
    static DefaultSpeed singleton(orch);
    return singleton;
}

DefaultSpeed::DefaultSpeed(orchestrator_handle *orch) : PluginBase(orch) {}

const char *DefaultSpeed::id_impl() const noexcept
{
    return "layer_extrusion_edit.speed.default";
}

const char *DefaultSpeed::name_impl() const noexcept
{
    return "Default extrusion speed";
}

const char *DefaultSpeed::description_impl() const noexcept
{
    return "Resolves missing extrusion speed, including volumetric autospeed.";
}

const char *DefaultSpeed::exclusive_group_impl() const noexcept
{
    return "layer_extrusion_edit.speed";
}

const char *DefaultSpeed::exclusive_group_label_impl() const noexcept
{
    return "Extrusion speed";
}

const char *DefaultSpeed::exclusive_group_tooltip_impl() const noexcept
{
    return "Selects how ordered layer extrusions receive their process speed.";
}

slicing_step_t DefaultSpeed::step_impl() const noexcept
{
    return STEP_LAYER_EXTRUSION_EDIT;
}

const char *const *DefaultSpeed::dependencies_impl() const noexcept
{
    return k_no_dependencies;
}

int32_t DefaultSpeed::priority_impl() const noexcept
{
    return 0;
}

int32_t DefaultSpeed::used_config_keys(raw_used_config_key *keys) const noexcept
{
    if (keys != nullptr)
        std::copy(std::begin(k_used_config_keys), std::end(k_used_config_keys), keys);
    return int32_t(std::size(k_used_config_keys));
}

int32_t DefaultSpeed::defined_config_keys(const char **keys) const noexcept
{
    if (keys != nullptr)
        for (size_t idx = 0; idx < std::size(k_defined_config_keys); ++idx)
            keys[idx] = k_defined_config_keys[idx];
    return int32_t(std::size(k_defined_config_keys));
}

void DefaultSpeed::inilialize_impl(storage_handle *) const
{
    initialize_default_speed_settings(m_orchestrator);
    add_default_speed_gui_rules(m_orchestrator);
}

const char *DefaultSpeed::progress_message_format_impl() const noexcept
{
    return "Resolving extrusion speed: %u / %u trees";
}

void DefaultSpeed::setup_impl(const plugin_run_context *run_ctx, uint32_t) const
{
    throw_if_cancelled(run_ctx);
    // Invalidate previous state first so a bad context cannot reuse targets
    // computed for an earlier printing plan.
    m_setup_valid = false;
    m_autospeed_targets.clear();
    m_speed_partitions.clear();

    const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run_ctx);
    assert(ctx != nullptr && ctx->print != nullptr && ctx->plan != nullptr &&
           run_ctx != nullptr && run_ctx->plugin_storage != nullptr);
    if (ctx == nullptr || ctx->print == nullptr || ctx->plan == nullptr ||
        run_ctx == nullptr || run_ctx->plugin_storage == nullptr)
        return;

    PrintingPlan plan(ctx->plan);
    prepare_plan(run_ctx->plugin_storage, Print(ctx->print), plan,
                 m_autospeed_targets, m_speed_partitions, run_ctx);
    throw_if_cancelled(run_ctx);
    m_setup_valid = true;
}

void DefaultSpeed::setup_run_impl(const plugin_run_context *run_ctx) const
{
    const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run_ctx);
    if (ctx == nullptr || ctx->layer_group == nullptr)
        return;
    throw_if_cancelled(run_ctx);
    progress().add_max(extrusion_tree_count(PrintingLayerGroup(ctx->layer_group)));
}

void DefaultSpeed::run_impl(const plugin_run_context *run_ctx) const
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

    // Each run owns one layer group and reads immutable targets prepared before
    // the host launched parallel work.
    const Print print(ctx->print);
    PrintingPlan plan(ctx->plan);
    PrintingLayerGroup layer_group(ctx->layer_group);
    for (uint32_t tool_idx = 0; tool_idx < layer_group.tool_group_count(); ++tool_idx) {
        throw_if_cancelled(run_ctx);
        PrintingToolGroup tool_group = layer_group.tool_group(tool_idx);
        const double target = autospeed_target(m_autospeed_targets, ctx->group_idx, tool_group.extruder_id());
        for (uint32_t extrusion_idx = 0; extrusion_idx < tool_group.extrusion_count(); ++extrusion_idx) {
            throw_if_cancelled(run_ctx);
            edit_extrusion(print, plan, tool_group, tool_group.extrusion(extrusion_idx),
                           ctx->group_idx, target, m_speed_partitions, run_ctx);
            progress().increment();
        }
    }
}

} // namespace

void register_default_speed_plugin(orchestrator_handle *orch)
{
    orchestrator_register_plugin(orch, DefaultSpeed::instance(orch).c_instance());
}

}}} // namespace slic3r_api::LayerExtrusionEdit::DefaultSpeedPlugin
