///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
Default PrintingPlan fan assignment
===================================

This standard provider normalizes inherited fan placement, publishes BASE role
references and fills only effective absences on deposition leaves. Cooling,
first-layer limits and geometric overhang increases are independent later plugins.

No G-code is produced here. The firmware sees the resulting inherited
EPropertyFan and decides whether a physical fan command is necessary.

    inilialize_impl(): publish the standard role options before settings freeze
    setup_run_impl(): count the current layer's trees
    run_impl(): read filament references and publish BASE per tool
        |-- normalize_requests(): move inherited pairs to eligible leaves
        `-- for_each_leaf(): fill only absences with Automatic requests

Property keys belong to the registering host. Callbacks borrow disjoint layer
trees and retain no views after returning; cancellation and malformed input
errors propagate to the plugin runner. Reprocessing starts from the saved tree,
not by trying to undo another policy's transformations.
*/

#include "DefaultFan.hpp"

#include <algorithm>
#include <cstdint>
#include <iterator>
#include <stdexcept>

#include "FanTreeHelpers.hpp"
#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_layer_extrusion_edit.h"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/FanPolicy.hpp"

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
    {"external_perimeter_fan_speed", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"gap_fill_fan_speed", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"infill_fan_speed", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"internal_bridge_fan_speed", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"overhangs_dynamic_fan_speed", RAW_CO_VECTOR_GRAPH, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"overhangs_fan_speed", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"perimeter_fan_speed", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"solid_infill_fan_speed", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_material_fan_speed", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_material_interface_fan_speed", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"top_fan_speed", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE}
};

const char *const k_defined_config_keys[] = {
    "bridge_fan_speed",
    "default_fan_speed",
    "external_perimeter_fan_speed",
    "gap_fill_fan_speed",
    "infill_fan_speed",
    "internal_bridge_fan_speed",
    "overhangs_fan_speed",
    "perimeter_fan_speed",
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
    def.invalidates_step = STEP_LAYER_EXTRUSION_EDIT;
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
    if (orchestrator_create_option_def(orchestrator, &def) != OPTION_DEF_ERROR_OK)
        throw std::runtime_error("Cannot register bridge_fan_speed.");

    // ---- external_perimeter_fan_speed | preset=RAW_PRESET_TYPE_FFF_FILAMENT ----
    def = raw_config_option_def_init();
    def.opt_key = "external_perimeter_fan_speed";
    def.type = RAW_CO_VECTOR_INT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_FILAMENT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_LAYER_EXTRUSION_EDIT;
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
    if (orchestrator_create_option_def(orchestrator, &def) != OPTION_DEF_ERROR_OK)
        throw std::runtime_error("Cannot register external_perimeter_fan_speed.");

    // ---- gap_fill_fan_speed | preset=RAW_PRESET_TYPE_FFF_FILAMENT ----
    def = raw_config_option_def_init();
    def.opt_key = "gap_fill_fan_speed";
    def.type = RAW_CO_VECTOR_INT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_FILAMENT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_LAYER_EXTRUSION_EDIT;
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
    if (orchestrator_create_option_def(orchestrator, &def) != OPTION_DEF_ERROR_OK)
        throw std::runtime_error("Cannot register gap_fill_fan_speed.");

    // ---- infill_fan_speed | preset=RAW_PRESET_TYPE_FFF_FILAMENT ----
    def = raw_config_option_def_init();
    def.opt_key = "infill_fan_speed";
    def.type = RAW_CO_VECTOR_INT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_FILAMENT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_LAYER_EXTRUSION_EDIT;
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
    if (orchestrator_create_option_def(orchestrator, &def) != OPTION_DEF_ERROR_OK)
        throw std::runtime_error("Cannot register infill_fan_speed.");

    // ---- internal_bridge_fan_speed | preset=RAW_PRESET_TYPE_FFF_FILAMENT ----
    def = raw_config_option_def_init();
    def.opt_key = "internal_bridge_fan_speed";
    def.type = RAW_CO_VECTOR_INT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_FILAMENT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_LAYER_EXTRUSION_EDIT;
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
    if (orchestrator_create_option_def(orchestrator, &def) != OPTION_DEF_ERROR_OK)
        throw std::runtime_error("Cannot register internal_bridge_fan_speed.");

    // ---- overhangs_fan_speed | preset=RAW_PRESET_TYPE_FFF_FILAMENT ----
    def = raw_config_option_def_init();
    def.opt_key = "overhangs_fan_speed";
    def.type = RAW_CO_VECTOR_INT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_FILAMENT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_LAYER_EXTRUSION_EDIT;
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
    if (orchestrator_create_option_def(orchestrator, &def) != OPTION_DEF_ERROR_OK)
        throw std::runtime_error("Cannot register overhangs_fan_speed.");

    // ---- perimeter_fan_speed | preset=RAW_PRESET_TYPE_FFF_FILAMENT ----
    def = raw_config_option_def_init();
    def.opt_key = "perimeter_fan_speed";
    def.type = RAW_CO_VECTOR_INT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_FILAMENT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_LAYER_EXTRUSION_EDIT;
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
    if (orchestrator_create_option_def(orchestrator, &def) != OPTION_DEF_ERROR_OK)
        throw std::runtime_error("Cannot register perimeter_fan_speed.");

    // ---- solid_infill_fan_speed | preset=RAW_PRESET_TYPE_FFF_FILAMENT ----
    def = raw_config_option_def_init();
    def.opt_key = "solid_infill_fan_speed";
    def.type = RAW_CO_VECTOR_INT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_FILAMENT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_LAYER_EXTRUSION_EDIT;
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
    if (orchestrator_create_option_def(orchestrator, &def) != OPTION_DEF_ERROR_OK)
        throw std::runtime_error("Cannot register solid_infill_fan_speed.");

    // ---- support_material_fan_speed | preset=RAW_PRESET_TYPE_FFF_FILAMENT ----
    def = raw_config_option_def_init();
    def.opt_key = "support_material_fan_speed";
    def.type = RAW_CO_VECTOR_INT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_FILAMENT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_LAYER_EXTRUSION_EDIT;
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
    if (orchestrator_create_option_def(orchestrator, &def) != OPTION_DEF_ERROR_OK)
        throw std::runtime_error("Cannot register support_material_fan_speed.");

    // ---- support_material_interface_fan_speed | preset=RAW_PRESET_TYPE_FFF_FILAMENT ----
    def = raw_config_option_def_init();
    def.opt_key = "support_material_interface_fan_speed";
    def.type = RAW_CO_VECTOR_INT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_FILAMENT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_LAYER_EXTRUSION_EDIT;
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
    if (orchestrator_create_option_def(orchestrator, &def) != OPTION_DEF_ERROR_OK)
        throw std::runtime_error("Cannot register support_material_interface_fan_speed.");

    // ---- top_fan_speed | preset=RAW_PRESET_TYPE_FFF_FILAMENT ----
    def = raw_config_option_def_init();
    def.opt_key = "top_fan_speed";
    def.type = RAW_CO_VECTOR_INT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_FILAMENT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_LAYER_EXTRUSION_EDIT;
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
    if (orchestrator_create_option_def(orchestrator, &def) != OPTION_DEF_ERROR_OK)
        throw std::runtime_error("Cannot register top_fan_speed.");

    // ---- default_fan_speed | preset=RAW_PRESET_TYPE_FFF_FILAMENT ----
    def = raw_config_option_def_init();
    def.opt_key = "default_fan_speed";
    def.type = RAW_CO_VECTOR_INT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_FILAMENT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_LAYER_EXTRUSION_EDIT;
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
    if (orchestrator_create_option_def(orchestrator, &def) != OPTION_DEF_ERROR_OK)
        throw std::runtime_error("Cannot register default_fan_speed.");
}


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
    void setup_run_impl(const plugin_run_context *run_ctx) const override;
    void run_impl(const plugin_run_context *run_ctx) const override;

    PluginPropertyKey<FanPolicy::Context> m_fan_context_property;
};

DefaultFan &DefaultFan::instance(orchestrator_handle *orch)
{
    static DefaultFan plugin(orch);
    return plugin;
}

DefaultFan::DefaultFan(orchestrator_handle *orch) :
    PluginBase(orch),
    m_fan_context_property(FanPolicy::context_property_key(orch))
{
}

const char *DefaultFan::id_impl() const noexcept { return "layer_extrusion_edit.fan.default"; }
const char *DefaultFan::name_impl() const noexcept { return "Default fan speed"; }
const char *DefaultFan::description_impl() const noexcept
{
    return "Publish role references and fill absent fan requests on deposition paths.";
}
const char *DefaultFan::exclusive_group_impl() const noexcept { return "layer_extrusion_edit.fan"; }
slicing_step_t DefaultFan::step_impl() const noexcept { return STEP_LAYER_EXTRUSION_EDIT; }
const char *const *DefaultFan::dependencies_impl() const noexcept { return k_no_dependencies; }
int32_t DefaultFan::priority_impl() const noexcept { return LAYER_EXTRUSION_EDIT_PRIORITY_FAN_STANDARD; }

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

void DefaultFan::setup_run_impl(const plugin_run_context *run_ctx) const
{
    const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run_ctx);
    if (ctx == nullptr || ctx->layer_group == nullptr)
        return;

    throw_if_cancelled(run_ctx);
    PrintingLayerGroup layer(ctx->layer_group);
    progress().add_max(extrusion_tree_count(layer));
}

void DefaultFan::run_impl(const plugin_run_context *run_ctx) const
{
    const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run_ctx);
    if (ctx == nullptr || ctx->print == nullptr || ctx->layer_group == nullptr)
        return;
    throw_if_cancelled(run_ctx);

    const Print print(ctx->print);
    const Config config = print.config();
    PrintingLayerGroup layer(ctx->layer_group);

    for (uint32_t tool_idx = 0; tool_idx < layer.tool_group_count(); ++tool_idx) {
        throw_if_cancelled(run_ctx);
        PrintingToolGroup tool = layer.tool_group(tool_idx);
        if (tool.extruder_id() == EXTRUDER_ID_UNKNOWN)
            throw std::runtime_error("Printing layer fan calculation needs a concrete extruder id.");
        const FanPolicy::Settings settings = FanPolicy::read_base_settings(config, tool.extruder_id());
        const FanPolicy::Context base = FanPolicy::make_base(settings);
        m_fan_context_property.get_or_add(tool.properties()) = base;
        for (uint32_t extrusion_idx = 0; extrusion_idx < tool.extrusion_count(); ++extrusion_idx) {
            throw_if_cancelled(run_ctx);
            MutableExtrusionEntity root = tool.extrusion(extrusion_idx).mutable_root();
            FanTreeHelpers::normalize_requests(root, run_ctx);
            FanTreeHelpers::for_each_leaf(root, [&](MutableExtrusionEntity leaf, const EffectiveTreeState &state) {
                if (state.fan.is_set()) return;
                const FanPolicy::Role role = FanPolicy::fan_role(state.attributes.extrusion_role());
                if (role != RAW_FAN_ROLE_COUNT && base.roles[role].percent >= 0)
                    leaf.get_or_add(EPropertyFan::key).set(float(base.roles[role].percent), FAN_REQUEST_MODE_AUTOMATIC);
            }, run_ctx);
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
