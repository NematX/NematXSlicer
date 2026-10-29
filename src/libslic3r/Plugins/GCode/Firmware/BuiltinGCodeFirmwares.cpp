///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "BuiltinGCodeFirmwares.hpp"

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <memory>

#include "BambuGCodeFirmware.hpp"
#include "KlipperGCodeFirmware.hpp"
#include "Marlin1GCodeFirmware.hpp"
#include "Marlin2GCodeFirmware.hpp"
#include "PrusaGCodeFirmware.hpp"
#include "RepRapGCodeFirmware.hpp"
#include "SprinterGCodeFirmware.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_gcode_firmware.h"
#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeScriptProcessorViews.hpp"
#include "libslic3r/Plugins/GCode/Reader/BambuGCodeReaderItems.hpp"

/*
Built-in G-code firmware provider implementation
================================================

This file registers the built-in firmware dialects as one exclusive
GCODE_FIRMWARE group. `BuiltinGCodeFirmwarePlugin` is a metadata and factory
adapter; the dialect-specific session classes perform the actual G-code
encoding.

The normal execution flow is:

All dialects share the common configuration keys. A dialect may add extra
keys, such as Klipper's `tool_name`. Only the owner selected by
`registers_ui` publishes the shared UI fragment, preventing duplicate
selectors. The plugin instances remain registered even when inactive so the
exclusive-group setting can list every available firmware.
Each definition may also create owned settings and UI fragments during plugin
initialization, independently of the shared selector owner. Declared read keys
let the host disable those controls when another firmware is selected.

Each run creates a fresh session, so machine state, tool histories, and script
processor state cannot leak between exports. The session is owned by the host
after `run_impl()` returns and is consumed by the selected STEP_GCODE output
writer. This adapter does not itself traverse the PrintingPlan or emit final
G-code.

Operation notes:
- register_builtin_gcode_firmware_plugins()
- define the available dialects and their priorities
- register one BuiltinGCodeFirmwarePlugin per dialect
- expose one shared firmware selector in the printer UI
- read the selected G-code script processor, if present
- call the definition's session factory
- transfer the new GCodeFirmwareSession to the host context

Representative direct calls (branches are not a timeline):

    register_builtin_gcode_firmware_plugins()
        |-- register_marlin2_gcode_firmware_plugin()
        |-- register_marlin1_gcode_firmware_plugin()
        |-- register_prusa_gcode_firmware_plugin()
        |-- register_reprap_gcode_firmware_plugin()
        |-- register_sprinter_gcode_firmware_plugin()
        `-- register_klipper_gcode_firmware_plugin()

    BuiltinGCodeFirmwarePlugin::run_impl()
        |-- plugin_ctx_as_gcode_firmware()
        |-- m_definition.factory()
        |-- default_session.set_reader_state_id()
        |-- default_session.set_gcode_reader_context()
        `-- make_gcode_firmware_instance()
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Firmware {
namespace {

typedef std::unique_ptr<GCodeFirmwareSession> (*FirmwareSessionFactory)(
    GCodeScriptProcessorView scripts,
    storage_handle *storage);

struct FirmwareSettingDefinition
{
    const char *key;
    raw_config_option_type type;
    raw_container_type container;
    raw_option_preset_type preset;
    const char *default_value;
    const char *label;
    const char *tooltip;
};

struct FirmwareDefinition
{
    const char *id;
    const char *name;
    const char *description;
    int32_t exclusive_group_priority;
    const char *reader_state_name;
    FirmwareSessionFactory factory;
    const raw_used_config_key *extra_used_config_keys;
    size_t extra_used_config_key_count;
    bool registers_ui;
    void (*create_settings)(orchestrator_handle *) = nullptr;
    const FirmwareSettingDefinition *settings = nullptr;
    size_t defined_key_count = 0;
};

// Create Bambu's printer capabilities and per-filament fan settings, copying
// definitions and UI text into the borrowed host. Option-definition errors throw;
// UI-fragment registration reports failures through its host status, not exceptions.
void create_bambu_settings(orchestrator_handle *orchestrator);

template<class SessionType>
std::unique_ptr<GCodeFirmwareSession> create_session(GCodeScriptProcessorView scripts,
                                                     storage_handle *storage)
{
    return std::unique_ptr<GCodeFirmwareSession>(new SessionType(scripts, storage));
}

const char *const k_no_dependencies[] = { nullptr };

const raw_used_config_key k_common_used_config_keys[] = {
    {"deretract_speed", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"extruder_fan_offset", RAW_CO_VECTOR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"extruder_offset", RAW_CO_VECTOR_POINT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"extruder_temperature_offset", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"extrusion_multiplier", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"filament_diameter", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"gcode_precision_e", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"gcode_precision_xyz", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"machine_limits_usage", RAW_CO_ENUM, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"machine_max_acceleration_e", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"machine_max_acceleration_extruding", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"machine_max_acceleration_retracting", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"machine_max_acceleration_travel", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"machine_max_acceleration_x", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"machine_max_acceleration_y", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"machine_max_acceleration_z", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"machine_max_feedrate_e", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"machine_max_feedrate_x", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"machine_max_feedrate_y", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"machine_max_feedrate_z", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"machine_max_jerk_e", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"machine_max_jerk_x", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"machine_max_jerk_y", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"machine_max_jerk_z", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"machine_min_extruding_rate", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"machine_min_travel_rate", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"nozzle_diameter", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"retract_speed", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"single_extruder_multi_material", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"travel_speed", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"use_firmware_retraction", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"use_relative_e_distances", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"use_volumetric_e", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"z_offset", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE}
};

const raw_used_config_key k_klipper_used_config_keys[] = {
    {"tool_name", RAW_CO_VECTOR_STRING, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE}
};

const raw_used_config_key k_bambu_used_config_keys[] = {
    {"retract_length", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"platter_fan", RAW_CO_BOOL, RAW_CONTAINER_TYPE_PROJECT, RAW_PRESET_TYPE_FFF_PRINTER},
    {"support_air_filtration", RAW_CO_BOOL, RAW_CONTAINER_TYPE_PROJECT, RAW_PRESET_TYPE_FFF_PRINTER},
    {"platter_fan_speed", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_PLATER, RAW_PRESET_TYPE_FFF_FILAMENT},
    {"activate_air_filtration", RAW_CO_VECTOR_BOOL, RAW_CONTAINER_TYPE_PLATER, RAW_PRESET_TYPE_FFF_FILAMENT},
    {"during_print_exhaust_fan_speed", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_PLATER, RAW_PRESET_TYPE_FFF_FILAMENT},
    {"complete_print_exhaust_fan_speed", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_PLATER, RAW_PRESET_TYPE_FFF_FILAMENT}
};

const FirmwareSettingDefinition k_bambu_settings[] = {
    {"platter_fan", RAW_CO_BOOL, RAW_CONTAINER_TYPE_PROJECT, RAW_PRESET_TYPE_FFF_PRINTER, "0", "Platter fan",
     "Enable the chamber-side platter cooling fan (M106 P2 S0-255). Its speed is set by the filament's Platter fan speed."},
    {"support_air_filtration", RAW_CO_BOOL, RAW_CONTAINER_TYPE_PROJECT, RAW_PRESET_TYPE_FFF_PRINTER, "0", "Air filtration / exhaust fan",
     "Enable the exhaust fan (M106 P3 S0-255). Filaments must also enable Activate air filtration."},
    {"platter_fan_speed", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_PLATER, RAW_PRESET_TYPE_FFF_FILAMENT, "0", "Platter fan speed",
     "Platter cooling fan speed (M106 P2 S0-255). Requires Platter fan in printer settings. The fan stays off for Disable fan for the first layers."},
    {"activate_air_filtration", RAW_CO_VECTOR_BOOL, RAW_CONTAINER_TYPE_PLATER, RAW_PRESET_TYPE_FFF_FILAMENT, "0", "Activate air filtration",
     "Enable exhaust fan control (M106 P3 S0-255) for this filament. Requires Air filtration / exhaust fan in printer settings."},
    {"during_print_exhaust_fan_speed", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_PLATER, RAW_PRESET_TYPE_FFF_FILAMENT, "60", "Exhaust fan speed during print",
     "Exhaust speed during printing (M106 P3 S0-255), using the maximum among used filaments with Activate air filtration. Requires Air filtration / exhaust fan in printer settings."},
    {"complete_print_exhaust_fan_speed", RAW_CO_VECTOR_INT, RAW_CONTAINER_TYPE_PLATER, RAW_PRESET_TYPE_FFF_FILAMENT, "80", "Exhaust fan speed after print",
     "Exhaust speed after end G-code (M106 P3 S0-255), using the maximum among used filaments with Activate air filtration. Requires Air filtration / exhaust fan in printer settings."}
};
static_assert(std::size(k_bambu_used_config_keys) == std::size(k_bambu_settings) + 1,
              "Bambu reads its fan settings and the core retraction length.");

// The host owns copies of these six definitions. Integer vectors are percentages
// in [0,100]; bool vectors follow filament selection. No borrowed data is retained.
void create_bambu_settings(orchestrator_handle *orchestrator)
{
    for (const FirmwareSettingDefinition &setting : k_bambu_settings) {
        raw_config_option_def def = raw_config_option_def_init();
        def.opt_key = setting.key;
        def.type = setting.type;
        def.container_type = setting.container;
        def.option_preset_type = setting.preset;
        def.printer_technology = RAW_PT_FFF;
        def.category = RAW_OPTION_CATEGORY_COOLING;
        def.label = setting.label;
        def.tooltip = setting.tooltip;
        def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
        def.is_vector_extruder = setting.preset == RAW_PRESET_TYPE_FFF_FILAMENT;
        def.default_serialized_value = setting.default_value;
        if (def.type == RAW_CO_VECTOR_INT) {
            def.sidetext = "%";
            def.has_min = 1;
            def.min_value = 0;
            def.has_max = 1;
            def.max_value = 100;
        }
        if (orchestrator_create_option_def(orchestrator, &def) != OPTION_DEF_ERROR_OK)
            throw std::runtime_error("Could not create Bambu fan setting.");
    }
    orchestrator_add_ui_fragment(orchestrator, "printer_fff.ui", "bambu_chamber_fans_printer",
        "page:General:printer\ngroup:Cooling fan\n"
        "\tsetting:insert$aftersetting$fan_percentage:platter_fan\n"
        "\tsetting:insert$aftersetting$platter_fan:support_air_filtration\n", 1);
    orchestrator_add_ui_fragment(orchestrator, "filament.ui", "bambu_chamber_fans_filament",
        "page:Cooling:time\ngroup:insert$aftergroup$Fan speed - default:Chamber fans\n"
        "\tsetting:id$0:platter_fan_speed\n\tsetting:id$0:activate_air_filtration\n"
        "\tsetting:id$0:during_print_exhaust_fan_speed\n"
        "\tsetting:id$0:complete_print_exhaust_fan_speed\n", 1);
}

const FirmwareDefinition k_marlin2_definition = {
    "gcode.firmware.marlin2",
    "Marlin 2",
    "G-code firmware session for Marlin 2.",
    0,
    nullptr,
    &create_session<Marlin2GCodeFirmwareSession>,
    nullptr,
    0,
    true
};

const FirmwareDefinition k_marlin1_definition = {
    "gcode.firmware.marlin1",
    "Marlin 1",
    "G-code firmware session for Marlin 1.",
    10,
    nullptr,
    &create_session<Marlin1GCodeFirmwareSession>,
    nullptr,
    0,
    false
};

const FirmwareDefinition k_prusa_definition = {
    "gcode.firmware.prusa",
    "Prusa firmware",
    "G-code firmware session for Prusa firmware.",
    20,
    nullptr,
    &create_session<PrusaGCodeFirmwareSession>,
    nullptr,
    0,
    false
};

const FirmwareDefinition k_reprap_definition = {
    "gcode.firmware.reprap",
    "RepRapFirmware",
    "G-code firmware session for RepRapFirmware.",
    30,
    Reader::REPRAP_READER_STATE_NAME,
    &create_session<RepRapGCodeFirmwareSession>,
    nullptr,
    0,
    false
};

const FirmwareDefinition k_sprinter_definition = {
    "gcode.firmware.sprinter",
    "Sprinter",
    "G-code firmware session for Sprinter.",
    40,
    nullptr,
    &create_session<SprinterGCodeFirmwareSession>,
    nullptr,
    0,
    false
};

const FirmwareDefinition k_klipper_definition = {
    "gcode.firmware.klipper",
    "Klipper",
    "G-code firmware session for Klipper.",
    50,
    nullptr,
    &create_session<KlipperGCodeFirmwareSession>,
    k_klipper_used_config_keys,
    std::size(k_klipper_used_config_keys),
    false
};

const FirmwareDefinition k_bambu_definition = {
    "gcode.firmware.bambu",
    "Bambu Lab",
    "G-code firmware session for Bambu Lab printers.",
    60,
    Reader::BAMBU_READER_STATE_NAME,
    &create_session<BambuGCodeFirmwareSession>,
    k_bambu_used_config_keys,
    std::size(k_bambu_used_config_keys),
    false,
    &create_bambu_settings,
    k_bambu_settings,
    std::size(k_bambu_settings)
};

class BuiltinGCodeFirmwarePlugin final : public PluginBase
{
public:
    BuiltinGCodeFirmwarePlugin(orchestrator_handle *orchestrator,
                               const FirmwareDefinition &definition) :
        PluginBase(orchestrator),
        m_definition(definition),
        m_reader_state_id(definition.reader_state_name == nullptr ?
            GCODE_READER_STATE_ID_PORTABLE :
            orchestrator_register_runtime_id(orchestrator,
                GCODE_READER_STATE_ID_TYPE, definition.reader_state_name))
    {
        if (m_reader_state_id == ORCHESTRATOR_RUNTIME_ID_INVALID)
            throw std::runtime_error("The firmware Reader state identity could not be registered.");
    }

private:
    const char *id_impl() const noexcept override { return m_definition.id; }
    const char *name_impl() const noexcept override { return m_definition.name; }
    const char *description_impl() const noexcept override { return m_definition.description; }
    const char *exclusive_group_impl() const noexcept override { return "gcode_firmware_plugin"; }
    slicing_step_t step_impl() const noexcept override { return GCODE_FIRMWARE; }
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }
    // Return the common firmware execution priority. Nothing is changed or transferred, and the call cannot fail.
    int32_t priority_impl() const noexcept override { return 0; }
    // Return this firmware's preference number. Nothing is changed or transferred, and the call cannot fail.
    int32_t exclusive_group_priority_impl() const noexcept override { return m_definition.exclusive_group_priority; }
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override;
    // Enumerate the settings owned by this definition.
    int32_t defined_config_keys(const char **keys) const noexcept override;
    // Create dialect settings before optionally publishing the shared selector.
    void inilialize_impl(storage_handle *storage) const override;
    void run_impl(const plugin_run_context *run_ctx) const override;

    const FirmwareDefinition &m_definition;
    uint32_t m_reader_state_id;
};

int32_t BuiltinGCodeFirmwarePlugin::used_config_keys(raw_used_config_key *keys) const noexcept
{
    const size_t common_count = std::size(k_common_used_config_keys);
    if (keys != nullptr) {
        std::copy(std::begin(k_common_used_config_keys), std::end(k_common_used_config_keys), keys);
        if (m_definition.extra_used_config_keys != nullptr)
            std::copy_n(m_definition.extra_used_config_keys,
                        m_definition.extra_used_config_key_count,
                        keys + common_count);
    }
    return int32_t(common_count + m_definition.extra_used_config_key_count);
}

// Copy static owned-key names into the caller's optional buffer and return their
// count. The host supplies enough slots; no ownership moves and no calls can fail.
int32_t BuiltinGCodeFirmwarePlugin::defined_config_keys(const char **keys) const noexcept
{
    if (keys != nullptr) {
        for (size_t idx = 0; idx < m_definition.defined_key_count; ++idx)
            keys[idx] = m_definition.settings[idx].key;
    }
    return int32_t(m_definition.defined_key_count);
}

// Initialize owned settings through the definition before the shared UI guard.
// Host registration copies strings; settings errors propagate, storage is unused.
void BuiltinGCodeFirmwarePlugin::inilialize_impl(storage_handle *) const
{
    if (m_definition.create_settings != nullptr)
        m_definition.create_settings(m_orchestrator);
    if (!m_definition.registers_ui)
        return;

    // One owner publishes the shared selector. Registering the same fragment
    // from every dialect would create duplicate controls in the printer page.
    orchestrator_add_ui_fragment(
        m_orchestrator,
        "printer_fff.ui",
        "gcode_firmware_plugin",
        printer_ui_fragment(),
        1);
}

void BuiltinGCodeFirmwarePlugin::run_impl(const plugin_run_context *run_ctx) const
{
    // Firmware sessions are published for later STEP_GCODE calls and cannot
    // retain this borrowed run context. Cancellation is therefore observed at
    // the provider boundary and by the writer around every session call.
    throw_if_cancelled(run_ctx);
    run_ctx_gcode_firmware *context = plugin_ctx_as_gcode_firmware(run_ctx);
    if (context == nullptr)
        return;

    // Session ownership moves to the host and ends after the selected output
    // writer completes. A fresh instance isolates every export's machine state.
    throw_if_cancelled(run_ctx);
    const GCodeScriptProcessorView scripts = context->script_processor != nullptr ?
        GCodeScriptProcessorView(context->script_processor) : GCodeScriptProcessorView();
    std::unique_ptr<GCodeFirmwareSession> session = m_definition.factory(scripts, run_ctx->plugin_storage);
    throw_if_cancelled(run_ctx);
    DefaultGCodeFirmwareSession &default_session =
        static_cast<DefaultGCodeFirmwareSession &>(*session);
    default_session.set_reader_state_id(m_reader_state_id);
    default_session.set_gcode_reader_context(m_orchestrator, context->print);
    context->instance = make_gcode_firmware_instance(std::move(session));
}

void register_firmware(orchestrator_handle *orchestrator,
                       BuiltinGCodeFirmwarePlugin &plugin)
{
    orchestrator_register_plugin(orchestrator, plugin.c_instance());
}

} // namespace

const char *printer_ui_fragment() noexcept
{
    return "page:General:printer\n"
           "group:Firmware\n"
           "\tsetting:insert$beforesetting$gcode_flavor:gcode_firmware_plugin\n";
}

void register_marlin1_gcode_firmware_plugin(orchestrator_handle *orchestrator)
{
    static BuiltinGCodeFirmwarePlugin plugin(orchestrator, k_marlin1_definition);
    register_firmware(orchestrator, plugin);
}

void register_marlin2_gcode_firmware_plugin(orchestrator_handle *orchestrator)
{
    static BuiltinGCodeFirmwarePlugin plugin(orchestrator, k_marlin2_definition);
    register_firmware(orchestrator, plugin);
}

void register_prusa_gcode_firmware_plugin(orchestrator_handle *orchestrator)
{
    static BuiltinGCodeFirmwarePlugin plugin(orchestrator, k_prusa_definition);
    register_firmware(orchestrator, plugin);
}

void register_reprap_gcode_firmware_plugin(orchestrator_handle *orchestrator)
{
    static BuiltinGCodeFirmwarePlugin plugin(orchestrator, k_reprap_definition);
    register_firmware(orchestrator, plugin);
}

void register_sprinter_gcode_firmware_plugin(orchestrator_handle *orchestrator)
{
    static BuiltinGCodeFirmwarePlugin plugin(orchestrator, k_sprinter_definition);
    register_firmware(orchestrator, plugin);
}

void register_klipper_gcode_firmware_plugin(orchestrator_handle *orchestrator)
{
    static BuiltinGCodeFirmwarePlugin plugin(orchestrator, k_klipper_definition);
    register_firmware(orchestrator, plugin);
}

// Register the Bambu Lab dialect as one more member of the exclusive firmware group.
void register_bambu_gcode_firmware_plugin(orchestrator_handle *orchestrator)
{
    static BuiltinGCodeFirmwarePlugin plugin(orchestrator, k_bambu_definition);
    register_firmware(orchestrator, plugin);
}

void register_builtin_gcode_firmware_plugins(orchestrator_handle *orchestrator)
{
    // Marlin 2 has the lowest priority and therefore becomes the first enum
    // value and default when no printer preset made an explicit selection.
    register_marlin2_gcode_firmware_plugin(orchestrator);
    register_marlin1_gcode_firmware_plugin(orchestrator);
    register_prusa_gcode_firmware_plugin(orchestrator);
    register_reprap_gcode_firmware_plugin(orchestrator);
    register_sprinter_gcode_firmware_plugin(orchestrator);
    register_klipper_gcode_firmware_plugin(orchestrator);
    register_bambu_gcode_firmware_plugin(orchestrator);
}

}}} // namespace slic3r_api::GCodeGeneration::Firmware
