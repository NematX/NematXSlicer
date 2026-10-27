///|/ Copyright (c) NematXSlicer 2026 Durand Remi @supermerill
///|/
///|/ NematXSlicer is released under the terms of the AGPLv3 or higher

// NematX firmware package: the G-code firmware gcode.firmware.nematx and its
// reader gcode.reader.nematx.
//
// The firmware is selected by the printer setting gcode_firmware_plugin. For
// each export, the host runs this provider, which creates a new
// NematXGCodeFirmwareSession; the PrintingPlan writer then asks the session for
// the G-code of every part of the plan. The session names its reader, used by
// the host for the preview and by the session for custom G-code scripts.
//
// This provider also defines the NematX printer settings: extruder_axis,
// disable_fan_from_unused_extruder, gcode_line_number and gcode_no_comment.

#include <algorithm>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>

#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/slic3r_plugin.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_gcode_firmware.h"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeFirmwareViews.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeScriptProcessorViews.hpp"

#include "NematXFirmwareSession.hpp"
#include "NematXGCodeReader.hpp"

namespace nematx_firmware {
namespace {

using namespace slic3r_api;
using namespace slic3r_api::GCodeGeneration;

const char *const k_no_dependencies[] = { nullptr };
const char *const k_defined_config_keys[] = {
    k_extruder_axis_key, k_disable_unused_fan_key, k_line_number_key, k_no_comment_key
};

// Settings read by the session: the ones of every built-in firmware, plus the
// NematX ones.
const raw_used_config_key k_used_config_keys[] = {
    {"deretract_speed", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"extruder_fan_offset", RAW_CO_VECTOR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"extruder_offset", RAW_CO_VECTOR_POINT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"extruder_temperature_offset", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"extrusion_multiplier", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"fan_percentage", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"filament_diameter", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"gcode_label_objects", RAW_CO_ENUM, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"gcode_precision_e", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"gcode_precision_xyz", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"machine_limits_usage", RAW_CO_ENUM, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"nozzle_diameter", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"retract_speed", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"single_extruder_multi_material", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"thumbnail_background", RAW_CO_ENUM, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"thumbnail_background_color", RAW_CO_STRING, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"thumbnail_color_mode", RAW_CO_ENUM, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"thumbnails", RAW_CO_VECTOR_POINT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"thumbnails_color", RAW_CO_STRING, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"thumbnails_end_file", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"thumbnails_format", RAW_CO_ENUM, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"thumbnails_tag_format", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"thumbnails_with_bed", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"thumbnails_with_support", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"travel_speed", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"use_firmware_retraction", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"use_relative_e_distances", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"use_volumetric_e", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"z_offset", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {k_extruder_axis_key, RAW_CO_VECTOR_STRING, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {k_disable_unused_fan_key, RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {k_line_number_key, RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {k_no_comment_key, RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE}
};

// Define one printer setting; a definition error makes the plugin fail to load.
void define_option(orchestrator_handle *orchestrator, raw_config_option_def def)
{
    def.container_type = RAW_CONTAINER_TYPE_PROJECT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT;
    const option_def_error_code result = orchestrator_create_option_def(orchestrator, &def);
    if (result != OPTION_DEF_ERROR_OK)
        throw std::runtime_error(std::string("Cannot define ") + def.opt_key + " (option definition error " +
                                 std::to_string(int(result)) + ").");
}

raw_config_option_def bool_option(const char *key, const char *label, const char *tooltip)
{
    raw_config_option_def def = raw_config_option_def_init();
    def.opt_key = key;
    def.type = RAW_CO_BOOL;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINTER;
    def.label = label;
    def.tooltip = tooltip;
    def.default_serialized_value = "0";
    return def;
}

class NematXFirmwarePlugin final : public PluginBase
{
public:
    explicit NematXFirmwarePlugin(orchestrator_handle *orchestrator) : PluginBase(orchestrator) {}

private:
    const char *id_impl() const noexcept override { return k_firmware_plugin_id; }
    const char *name_impl() const noexcept override { return "NematX"; }
    const char *description_impl() const noexcept override { return "G-code firmware session for NematX printers."; }
    // The host describes this group; its gcode_firmware_plugin setting lists all firmware providers.
    const char *exclusive_group_impl() const noexcept override { return "gcode_firmware_plugin"; }
    slicing_step_t step_impl() const noexcept override { return GCODE_FIRMWARE; }
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }
    // The host requires one execution priority in an exclusive group: the built-in firmwares use 0.
    int32_t priority_impl() const noexcept override { return 0; }
    // Prefer built-in Marlin 2 unless the profile explicitly selects NematX.
    int32_t exclusive_group_priority_impl() const noexcept override { return 100; }

    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override
    {
        if (keys != nullptr)
            std::copy(std::begin(k_used_config_keys), std::end(k_used_config_keys), keys);
        return int32_t(std::size(k_used_config_keys));
    }

    int32_t defined_config_keys(const char **keys) const noexcept override
    {
        if (keys != nullptr)
            std::copy(std::begin(k_defined_config_keys), std::end(k_defined_config_keys), keys);
        return int32_t(std::size(k_defined_config_keys));
    }

    void inilialize_impl(storage_handle *) const override
    {
        raw_config_option_def axis = raw_config_option_def_init();
        axis.opt_key = k_extruder_axis_key;
        axis.type = RAW_CO_VECTOR_STRING;
        axis.option_preset_type = RAW_PRESET_TYPE_FFF_TOOL_EXTRUDER;
        axis.category = RAW_OPTION_CATEGORY_EXTRUDERS;
        axis.is_vector_extruder = 1;
        axis.label = "Extrusion axis";
        axis.tooltip = "Axis letter of this extruder in NematX G-code (A, B...). "
                       "Empty: A for the first extruder, B for the second.";
        axis.default_serialized_value = "\"\"";
        define_option(m_orchestrator, axis);

        raw_config_option_def fan = bool_option(k_disable_unused_fan_key, "Disable fan from unused extruder",
            "Switch off the part cooling fan of the extruder left by a tool change.");
        fan.category = RAW_OPTION_CATEGORY_MMSETUP;
        define_option(m_orchestrator, fan);

        raw_config_option_def numbers = bool_option(k_line_number_key, "Add line numbers",
            "Add a line number (N10, N20...) at the start of each line of the G-code file.");
        numbers.category = RAW_OPTION_CATEGORY_OUTPUT;
        define_option(m_orchestrator, numbers);

        raw_config_option_def comments = bool_option(k_no_comment_key, "Remove all comments",
            "Remove comments, including the tags used by the G-code preview.");
        comments.category = RAW_OPTION_CATEGORY_OUTPUT;
        define_option(m_orchestrator, comments);
    }

    // Create one session per export and give its ownership to the host.
    void run_impl(const plugin_run_context *run_ctx) const override
    {
        throw_if_cancelled(run_ctx);
        run_ctx_gcode_firmware *context = plugin_ctx_as_gcode_firmware(run_ctx);
        if (context == nullptr)
            return;
        const GCodeScriptProcessorView scripts = context->script_processor != nullptr ?
            GCodeScriptProcessorView(context->script_processor) : GCodeScriptProcessorView();
        std::unique_ptr<NematXGCodeFirmwareSession> session(
            new NematXGCodeFirmwareSession(scripts, run_ctx->plugin_storage));
        // The NematX reader only uses the portable reader state.
        session->set_reader_state_id(GCODE_READER_STATE_ID_PORTABLE);
        session->set_thumbnail_services(m_orchestrator, context->thumbnail_renderer);
        session->set_gcode_reader_context(m_orchestrator, context->print);
        context->instance = make_gcode_firmware_instance(std::unique_ptr<GCodeFirmwareSession>(std::move(session)));
    }
};

} // namespace
} // namespace nematx_firmware

// Host entry point: register both plugins. Static instances stay alive for
// every callback; the host initializes them after registration.
extern "C" SLIC3R_PLUGIN_API void register_plugin(orchestrator_handle *orchestrator)
{
    static nematx_firmware::NematXFirmwarePlugin firmware(orchestrator);
    static nematx_firmware::NematXGCodeReader reader(orchestrator);
    orchestrator_register_plugin(orchestrator, firmware.c_instance());
    orchestrator_register_plugin(orchestrator, reader.c_instance());
}

// Keep this header last so its ABI registration includes every public API header above.
#include "libslic3r/Api/plugin/c/slic3r_plugin_register_version.h"
