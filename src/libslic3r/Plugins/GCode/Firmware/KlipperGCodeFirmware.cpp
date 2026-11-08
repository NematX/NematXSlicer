///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "KlipperGCodeFirmware.hpp"

#include <iomanip>
#include <sstream>

#include <stdexcept>
#include <utility>

#include "libslic3r/Api/plugin/cpp/gcode/ArcGeometry.hpp"

/*
Klipper firmware implementation
===============================

`KlipperGCodeFirmwareSession` specializes the common firmware session for
Klipper's command vocabulary. The inherited session still decides when the
machine state changes and when each PrintingPlan item is visited; this file
only encodes the dialect-specific result.

The main specialization flow is:

    setup_firmware()
    `-- build a stable name for every configured tool

    resolve_empty_script()
    `-- map color-change and pause events to `PAUSE`, then use the common
        fallback for other script types

    encode state changes
    |-- select a tool with `ACTIVATE_EXTRUDER`, or a SEMM filament with `T`
    |-- set active/SEMM temperature with `M104` or `M109`, or a named
        non-active heater with `SET_HEATER_TEMPERATURE` and `TEMPERATURE_WAIT`
    |-- set pressure advance with `SET_PRESSURE_ADVANCE`
    |-- encode shared acceleration with `M204`
    `-- report unsupported machine operations through the common fallback

    encode_move() / encode_arc()
    `-- emit Klipper-compatible linear and I/J-center arc commands, including
        only the axes and extrusion values that changed

Klipper uses one shared acceleration setting for the initial machine envelope,
while later movement-specific changes use `M204 S`. Tool names come from the
`tool_name` configuration and fall back to `extruder`, `extruder1`, and so on.
The selected tool's fan runtime is synchronized from the previous tool because
the emitted fan command is global. Outside SEMM, heater histories remain per
tool; SEMM shares all extruder runtime state through the base session.
This class does not own plan traversal or script placeholder expansion; those
responsibilities remain in the base session and the firmware output writer.
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Firmware {

std::string KlipperGCodeFirmwareSession::resolve_empty_script(
    gcode_script_type script_type,
    const Config *producer_config) const
{
    if (script_type == GCODE_SCRIPT_TYPE_COLOR_CHANGE_GCODE ||
        script_type == GCODE_SCRIPT_TYPE_PAUSE_PRINT_GCODE)
        return "PAUSE\n";
    return SingleAccelerationRegisterGCodeFirmwareSession::resolve_empty_script(
        script_type, producer_config);
}

std::string KlipperGCodeFirmwareSession::encode_machine_envelope(
    const MachineEnvelope &envelope) const
{
    // Klipper accepts P and T together and applies their minimum as its one
    // shared acceleration. Movement-specific changes later use M204 S.
    GCodeFormatter formatter(2, 2);
    formatter.emit_string("M204");
    formatter.emit_integer_parameter('P', envelope.max_print_acceleration);
    formatter.emit_integer_parameter('T', envelope.max_travel_acceleration);
    formatter.emit_comment(true, "sets the initial shared acceleration, mm/sec^2");
    return formatter.string();
}

void KlipperGCodeFirmwareSession::setup_firmware(const Config &config)
{
    SingleAccelerationRegisterGCodeFirmwareSession::setup_firmware(config);
    m_tool_names.clear();
    m_tool_names.reserve(extruders().size());
    // KlipperGCodeReader::klipper_tool_id reproduces this tool count and empty-name fallback; keep both implementations synchronized.
    for (size_t tool_idx = 0; tool_idx < extruders().size(); ++tool_idx) {
        std::string name = config.vector_string_or_default("tool_name", tool_idx, std::string());
        if (name.empty())
            name = tool_idx == 0 ? "extruder" : "extruder" + std::to_string(tool_idx);
        m_tool_names.push_back(std::move(name));
    }
}

// Borrow two tools and copy only fan runtime when a previous tool exists;
// destination settings remain owned and unchanged, with no output or failure.
void KlipperGCodeFirmwareSession::synchronize_selected_extruder_state(
    const DefaultExtruder *previous,
    DefaultExtruder &selected)
{
    // The current Klipper encoder emits one global M106 fan command. Heater
    // commands retain per-tool histories, and non-active targets use named
    // Klipper heater commands outside SEMM.
    if (previous != nullptr)
        selected.fan().synchronize_runtime_from(previous->fan());
}

const std::string &KlipperGCodeFirmwareSession::tool_name(extruder_index_t tool_id) const
{
    if (tool_id == EXTRUDER_ID_UNKNOWN || tool_id < EXTRUDER_ID_UNKNOWN ||
        static_cast<size_t>(tool_id) >= m_tool_names.size())
        throw std::out_of_range("Klipper has no configured name for the selected extruder.");
    return m_tool_names[static_cast<size_t>(tool_id)];
}

// Return a tool-selection command without changing state; SEMM selects a filament
// with T, otherwise a configured name is required and missing names throw.
std::string KlipperGCodeFirmwareSession::encode_tool_change(extruder_index_t tool_id) const
{
    if (single_extruder_multi_material())
        return DefaultGCodeFirmwareSession::encode_tool_change(tool_id);
    return "ACTIVATE_EXTRUDER EXTRUDER=" + tool_name(tool_id) + "\n";
}

// Format a Klipper heater command without changing state; active and SEMM tools use M104/M109, while a non-active named tool uses SET_HEATER_TEMPERATURE and positive waits use TEMPERATURE_WAIT with MAXIMUM for cooling or MINIMUM otherwise; no earlier target means MINIMUM and non-positive targets never emit a wait, whose omission remains acknowledged because waiting for a non-positive target is without effect.
std::string KlipperGCodeFirmwareSession::encode_tool_temperature(
    extruder_index_t tool_id,
    int16_t temperature,
    bool wait) const
{
    const bool emit_wait = wait && temperature > 0;
    if (single_extruder_multi_material() ||
        (current_extruder_id() && *current_extruder_id() == tool_id))
        return std::string(emit_wait ? "M109" : "M104") + " S" + std::to_string(temperature) + "\n";

    const std::string &name = tool_name(tool_id);
    std::string output = "SET_HEATER_TEMPERATURE HEATER=" + name +
        " TARGET=" + std::to_string(temperature) + "\n";
    if (emit_wait) {
        const std::optional<int16_t> encoded =
            extruders()[tool_id].heater().encoded_temperature();
        const bool cooling_wait = encoded && temperature < *encoded;
        output += "TEMPERATURE_WAIT SENSOR=" + name +
            (cooling_wait ? " MAXIMUM=" : " MINIMUM=") +
            std::to_string(temperature) + "\n";
    }
    return output;
}

std::string KlipperGCodeFirmwareSession::encode_chamber_temperature(int16_t, bool) const
{
    return encode_unsupported_operation("chamber temperature");
}

// Format pressure advance without mutating state; SEMM targets the active shared
// extruder, otherwise tool_name validates the ID and supplies the configured name.
std::string KlipperGCodeFirmwareSession::encode_pressure_advance(
    extruder_index_t tool_id,
    double pressure_advance) const
{
    return "SET_PRESSURE_ADVANCE ADVANCE=" + format_number(pressure_advance) +
           (single_extruder_multi_material() ? std::string() : " EXTRUDER=" + tool_name(tool_id)) + "\n";
}

std::string KlipperGCodeFirmwareSession::encode_acceleration(uint32_t acceleration, bool) const
{
    // [2026-08-25] Klipper accepts one shared M204 S value. Supplying only P
    // or T has no effect:
    // https://github.com/Klipper3d/klipper/blob/master/docs/G-Codes.md?plain=1
    return "M204 S" + std::to_string(acceleration) + "\n";
}

std::string KlipperGCodeFirmwareSession::encode_move(const PreparedMove &move,
                                                      bool include_speed) const
{
    const bool arc = move.destination && move.radius_mm != 0.f;
    if (!arc)
        return DefaultGCodeFirmwareSession::encode_move(move, include_speed);
    if (!gantry().position())
        throw std::invalid_argument("A Klipper arc requires a known start position.");
    if (move.arc_orientation == RAW_EXTRUSION_ARC_ORIENTATION_UNKNOWN)
        throw std::invalid_argument("A Klipper arc movement has no orientation.");

    // [2026-08-25] Klipper's gcode_arcs module accepts I/J center offsets and
    // rejects radius-form R arcs:
    // https://github.com/Klipper3d/klipper/blob/master/docs/G-Codes.md?plain=1
    const c_vec3d start_position = *gantry().position();
    const c_vec3d end_position = *move.destination;
    const c_vec2d start{ start_position.x, start_position.y };
    const c_vec2d center = arc_center(
        start,
        c_vec2d{ end_position.x, end_position.y },
        double(move.radius_mm),
        move.arc_orientation == RAW_EXTRUSION_ARC_ORIENTATION_CCW);

    GCodeFormatter formatter(gcode_formatter());
    formatter.emit_string(
        move.arc_orientation == RAW_EXTRUSION_ARC_ORIENTATION_CCW ? "G3" : "G2");
    if (start_position.x != end_position.x)
        formatter.emit_axis('X', end_position.x, formatter.m_gcode_precision_xyz);
    if (start_position.y != end_position.y)
        formatter.emit_axis('Y', end_position.y, formatter.m_gcode_precision_xyz);
    if (start_position.z != end_position.z)
        formatter.emit_axis('Z', end_position.z, formatter.m_gcode_precision_xyz);
    formatter.emit_axis('I', center.x - start.x, formatter.m_gcode_precision_xyz);
    formatter.emit_axis('J', center.y - start.y, formatter.m_gcode_precision_xyz);
    if (move.extrusion)
        formatter.emit_axis('E', *move.extrusion, formatter.m_gcode_precision_e);
    if (include_speed)
        formatter.emit_f(*gantry().requested_speed() * 60.0);
    return formatter.string();
}

std::string KlipperGCodeFirmwareSession::encode_extruder_current(extruder_index_t, double) const
{
    return encode_unsupported_operation("extruder current");
}

std::string KlipperGCodeFirmwareSession::encode_object_definitions(
    const std::vector<ObjectLabel> &objects) const
{
    std::string output;
    for (const ObjectLabel &object : objects) {
        std::ostringstream line;
        line << std::fixed << std::setprecision(3)
             << "EXCLUDE_OBJECT_DEFINE NAME='" << object.name << "'"
             << " CENTER=" << unscaled(object.projected_outline_center.x) << ','
             << unscaled(object.projected_outline_center.y) << " POLYGON=[";
        for (size_t idx = 0; idx < object.projected_outline.size(); ++idx) {
            if (idx != 0)
                line << ',';
            line << '[' << unscaled(object.projected_outline[idx].x) << ','
                 << unscaled(object.projected_outline[idx].y) << ']';
        }
        line << "]\n";
        output += line.str();
    }
    return output;
}

std::string KlipperGCodeFirmwareSession::encode_object_start(const ObjectLabel &object) const
{
    return "EXCLUDE_OBJECT_START NAME='" + object.name + "'\n";
}

std::string KlipperGCodeFirmwareSession::encode_object_end(const ObjectLabel &object) const
{
    return "EXCLUDE_OBJECT_END NAME='" + object.name + "'\n";
}

}}} // namespace slic3r_api::GCodeGeneration::Firmware
