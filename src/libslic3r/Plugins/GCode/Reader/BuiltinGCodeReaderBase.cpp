///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "BuiltinGCodeReaderBase.hpp"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <fast_float/fast_float.h>

#include "libslic3r/Api/plugin/c/slic3r_extrusions.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_gcode_reader.h"
#include "libslic3r/ExtrusionRole.hpp"

/*
Built-in G-code reader base
===========================

This file translates the common modal G-code vocabulary into normalized Reader
events. The most-derived provider enters first, but each dispatcher calls its
parent before trying additional commands. Common commands therefore reach this
class once and are routed through virtual handlers, allowing a dialect to
replace one interpretation without copying the dispatcher.

One run owns one local GCodeStateTracker, so the same provider can serve a
visible file or a firmware script without retaining input or parser state in
the plugin singleton.

Input may arrive in arbitrary chunks. The parser retains only an unfinished
last line during run(), assigns line numbers while complete lines are consumed,
then emits a normalized event for each command. Unknown commands invalidate
portable and private machine state instead of letting a later host-generated
command reuse a value which arbitrary dialect G-code may have changed.

    arbitrary chunks
          |
          v
    pending complete lines -- incomplete tail retained across chunks
          |
          v
    GCodeLineParser
          |
          +-- chained consume_*() -> virtual process_*() -> semantic event
          +-- marker or neutral  -> timeline/state event
          `-- no handler         -> INVALIDATE currently known portable state

Each implementation intentionally recognizes a conservative subset. A command
is only treated as harmless when the dialect guarantees that it cannot alter a
portable state item; otherwise an unsupported command invalidates known state
so the firmware writer cannot continue from guessed coordinates, modes or tools.
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Reader {
namespace {

const char *const k_no_dependencies[] = {nullptr};

// Return an unsigned command suffix only when every character after the command
// letter is decimal. Rejecting signs and suffixes prevents partial parses such
// as "G1foo" from being mistaken for a supported command.
std::optional<uint32_t> command_number(std::string_view command);
// Remove presentation whitespace around a historical comment payload.
std::string_view trim_presentation(std::string_view text);
// Parse one complete finite decimal payload without accepting a suffix.
bool parse_finite_comment_number(std::string_view text, double &value);
// Parse exactly two finite coordinates separated by one colon.
std::optional<std::pair<double, double>> parse_seam_coordinates(
    std::string_view text);
// Validate the optional historical color-change payload without accepting
// partial tool, RGB, or trailing-component parses.
bool valid_color_change_payload(std::string_view payload);
// Convert one legacy viewer role enum to the raw extrusion-role bitmask.
uint32_t raw_extrusion_role(Slic3r::GCodeExtrusionRole role);
// Resolve a TYPE name through the existing legacy role-name table.
std::optional<uint32_t> raw_extrusion_role_from_name(std::string_view name);
// Convert values expressed in the current input units to the millimetres used
// by the normalized Reader state.
double unit_scale(const GCodeStateTracker &state);
// Read a modal boolean, using the command family's standard default when the
// initial state did not establish the mode.
bool modal_bool(const GCodeStateTracker &state, uint32_t type, bool fallback);
// Return the selected logical tool, or conventional tool zero when selection
// is absent from an incomplete initial state.
uint32_t active_tool_value(const GCodeStateTracker &state);
// Resolve one parsed coordinate to its final absolute value. Relative values
// require a known previous coordinate; an empty result tells consume_line() to
// invalidate that axis rather than inventing an origin.
std::optional<double> final_axis_value(const GCodeStateTracker &state,
                                       uint32_t type,
                                       uint32_t target,
                                       std::optional<float> parsed,
                                       bool relative);
// Hold the portable movement delta separately from coordinates which were
// present in the command but could not be resolved from an unknown relative
// origin. Linear and arc handlers share this collection step.
struct ParsedMovementItems
{
    std::vector<raw_gcode_reader_event_item> values;
    std::vector<raw_gcode_reader_event_item> invalidated;
};
ParsedMovementItems parse_movement_items(
    const GCodeStateTracker &state,
    const GCodeLineParser::GCodeLine &line);

std::optional<uint32_t> command_number(std::string_view command)
{
    if (command.size() < 2)
        return std::nullopt;
    uint32_t number = 0;
    const std::from_chars_result result = std::from_chars(
        command.data() + 1, command.data() + command.size(), number);
    if (result.ec != std::errc() || result.ptr != command.data() + command.size())
        return std::nullopt;
    return number;
}

std::string_view trim_presentation(std::string_view text)
{
    size_t begin = 0;
    while (begin < text.size() && (text[begin] == ' ' || text[begin] == '\t'))
        ++begin;
    size_t end = text.size();
    while (end > begin && (text[end - 1] == ' ' || text[end - 1] == '\t'))
        --end;
    return text.substr(begin, end - begin);
}

bool parse_finite_comment_number(std::string_view text, double &value)
{
    const std::string_view trimmed = trim_presentation(text);
    if (trimmed.empty())
        return false;
    double parsed_value = 0.0;
    const fast_float::from_chars_result parsed = fast_float::from_chars(
        trimmed.data(), trimmed.data() + trimmed.size(), parsed_value);
    if (parsed.ec != std::errc() ||
        parsed.ptr != trimmed.data() + trimmed.size() ||
        !std::isfinite(parsed_value))
        return false;
    value = parsed_value;
    return true;
}

std::optional<std::pair<double, double>> parse_seam_coordinates(
    std::string_view text)
{
    const std::string_view trimmed = trim_presentation(text);
    const size_t separator = trimmed.find(':');
    if (separator == std::string_view::npos ||
        trimmed.find(':', separator + 1) != std::string_view::npos)
        return std::nullopt;

    double x = 0.0;
    double y = 0.0;
    if (!parse_finite_comment_number(trimmed.substr(0, separator), x) ||
        !parse_finite_comment_number(trimmed.substr(separator + 1), y))
        return std::nullopt;
    return std::pair<double, double>{x, y};
}

bool valid_color_change_payload(std::string_view payload)
{
    const size_t separator = payload.find(',');
    if (separator == std::string_view::npos || separator == 0 ||
        payload.find(',', separator + 1) != std::string_view::npos)
        return false;

    const std::string_view tool = payload.substr(0, separator);
    if (tool.size() < 2 || tool.front() != 'T')
        return false;
    for (size_t index = 1; index < tool.size(); ++index) {
        if (tool[index] < '0' || tool[index] > '9')
            return false;
    }

    uint32_t tool_number = 0;
    const std::from_chars_result parsed_tool = std::from_chars(
        tool.data() + 1, tool.data() + tool.size(), tool_number);
    if (parsed_tool.ec != std::errc() ||
        parsed_tool.ptr != tool.data() + tool.size() ||
        tool_number > UINT8_MAX)
        return false;

    const std::string_view rgb = payload.substr(separator + 1);
    if (rgb.size() != 7 || rgb.front() != '#')
        return false;
    for (size_t index = 1; index < rgb.size(); ++index) {
        const char value = rgb[index];
        const bool decimal = value >= '0' && value <= '9';
        const bool lower_hex = value >= 'a' && value <= 'f';
        const bool upper_hex = value >= 'A' && value <= 'F';
        if (!decimal && !lower_hex && !upper_hex)
            return false;
    }
    return true;
}

uint32_t raw_extrusion_role(Slic3r::GCodeExtrusionRole role)
{
    switch (role) {
    case Slic3r::GCodeExtrusionRole::None:
        return RAW_EXTRUSION_ROLE_NONE;
    case Slic3r::GCodeExtrusionRole::Perimeter:
        return RAW_EXTRUSION_ROLE_INTERNAL_PERIMETER;
    case Slic3r::GCodeExtrusionRole::ExternalPerimeter:
        return RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER;
    case Slic3r::GCodeExtrusionRole::OverhangPerimeter:
        return RAW_EXTRUSION_ROLE_OVERHANG_PERIMETER;
    case Slic3r::GCodeExtrusionRole::InternalInfill:
        return RAW_EXTRUSION_ROLE_INTERNAL_INFILL;
    case Slic3r::GCodeExtrusionRole::InternalBridgeInfill:
        return RAW_EXTRUSION_ROLE_INTERNAL_BRIDGE_INFILL;
    case Slic3r::GCodeExtrusionRole::SolidInfill:
        return RAW_EXTRUSION_ROLE_SOLID_INFILL;
    case Slic3r::GCodeExtrusionRole::TopSolidInfill:
        return RAW_EXTRUSION_ROLE_TOP_SOLID_INFILL;
    case Slic3r::GCodeExtrusionRole::Ironing:
        return RAW_EXTRUSION_ROLE_IRONING_INFILL;
    case Slic3r::GCodeExtrusionRole::BridgeInfill:
        return RAW_EXTRUSION_ROLE_BRIDGE_INFILL;
    case Slic3r::GCodeExtrusionRole::ThinWall:
        return RAW_EXTRUSION_ROLE_THIN_WALL;
    case Slic3r::GCodeExtrusionRole::GapFill:
        return RAW_EXTRUSION_ROLE_GAP_FILL;
    case Slic3r::GCodeExtrusionRole::Skirt:
        return RAW_EXTRUSION_ROLE_SKIRT;
    case Slic3r::GCodeExtrusionRole::SupportMaterial:
        return RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL;
    case Slic3r::GCodeExtrusionRole::SupportMaterialInterface:
        return RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL_INTERFACE;
    case Slic3r::GCodeExtrusionRole::WipeTower:
        return RAW_EXTRUSION_ROLE_WIPE_TOWER_DEFAULT;
    case Slic3r::GCodeExtrusionRole::Milling:
        return RAW_EXTRUSION_ROLE_MILLING;
    case Slic3r::GCodeExtrusionRole::Custom:
        return RAW_EXTRUSION_ROLE_NONE;
    case Slic3r::GCodeExtrusionRole::Travel:
        return RAW_EXTRUSION_ROLE_TRAVEL;
    case Slic3r::GCodeExtrusionRole::Count:
        break;
    }
    return RAW_EXTRUSION_ROLE_NONE;
}

std::optional<uint32_t> raw_extrusion_role_from_name(std::string_view name)
{
    const uint32_t role_count =
        static_cast<uint32_t>(Slic3r::GCodeExtrusionRole::Count);
    for (uint32_t index = 0; index < role_count; ++index) {
        const Slic3r::GCodeExtrusionRole role =
            static_cast<Slic3r::GCodeExtrusionRole>(index);
        if (Slic3r::gcode_extrusion_role_to_string(role) == name)
            return raw_extrusion_role(role);
    }
    // The legacy decoder also accepts this pre-2.3 spelling, so keep it
    // compatible while canonical writer output continues to use Skirt/Brim.
    if (name == "Skirt")
        return raw_extrusion_role(Slic3r::GCodeExtrusionRole::Skirt);
    return std::nullopt;
}

// Normalize textual coordinates to millimetres. Unknown units use the usual
// G-code default of millimetres, which keeps an incomplete initial snapshot
// useful without claiming it was explicitly configured.
double unit_scale(const GCodeStateTracker &state)
{
    const std::optional<double> millimetres = state.value(RAW_GCODE_READER_ITEM_UNITS_MM);
    return !millimetres || *millimetres != 0.0 ? 1.0 : 25.4;
}

// Read a modal boolean while preserving the command-specific default when the
// input state did not establish that mode.
bool modal_bool(const GCodeStateTracker &state, uint32_t type, bool fallback)
{
    const std::optional<double> value = state.value(type);
    return value ? *value != 0.0 : fallback;
}

// E coordinates and tool temperatures need a target. Unknown selection uses
// tool zero, matching the conventional single-tool G-code default.
uint32_t active_tool_value(const GCodeStateTracker &state)
{
    const std::optional<double> tool = state.value(RAW_GCODE_READER_ITEM_ACTIVE_TOOL);
    return tool ? static_cast<uint32_t>(*tool) : 0;
}

std::optional<double> final_axis_value(const GCodeStateTracker &state,
                                       uint32_t type,
                                       uint32_t target,
                                       std::optional<float> parsed,
                                       bool relative)
{
    // Absolute parameters directly replace a coordinate. Relative parameters
    // need a known origin; absence means the reader must later invalidate that
    // axis instead of inventing a destination.
    if (!parsed)
        return std::nullopt;
    const double value = double(*parsed) * unit_scale(state);
    if (!relative)
        return value;
    const std::optional<double> previous = state.value(type, target);
    return previous ? std::optional<double>(*previous + value) : std::nullopt;
}

ParsedMovementItems parse_movement_items(
    const GCodeStateTracker &state,
    const GCodeLineParser::GCodeLine &line)
{
    ParsedMovementItems result;
    const bool xyz_relative = !modal_bool(state, RAW_GCODE_READER_ITEM_XYZ_ABSOLUTE, true);
    const bool e_relative = !modal_bool(state, RAW_GCODE_READER_ITEM_E_ABSOLUTE, true);
    const std::pair<char, uint32_t> axes[] = {
        {'X', RAW_GCODE_READER_ITEM_X}, {'Y', RAW_GCODE_READER_ITEM_Y},
        {'Z', RAW_GCODE_READER_ITEM_Z}, {'E', RAW_GCODE_READER_ITEM_E}
    };
    for (const std::pair<char, uint32_t> &axis : axes) {
        const uint32_t target = axis.first == 'E' ? active_tool_value(state) : 0;
        const std::optional<double> value = final_axis_value(
            state, axis.second, target, line.parameter_value(axis.first),
            axis.first == 'E' ? e_relative : xyz_relative);
        if (value)
            result.values.push_back({axis.second, target, *value});
        else if (line.has_parameter(axis.first))
            result.invalidated.push_back({axis.second, target, 0.0});
    }
    if (const std::optional<float> speed = line.parameter_value('F'))
        result.values.push_back({RAW_GCODE_READER_ITEM_FEEDRATE, 0,
                                 double(*speed) * unit_scale(state)});
    return result;
}

} // namespace

uint32_t BuiltinGCodeReaderBase::active_tool(const GCodeStateTracker &state)
{
    return active_tool_value(state);
}

uint32_t BuiltinGCodeReaderBase::probe_marker(std::string_view text, std::string_view needle)
{
    if (needle.empty() || needle.size() > text.size())
        return 0;
    return std::search(text.begin(), text.end(), needle.begin(), needle.end(),
        [](char left, char right) {
            return std::tolower(static_cast<unsigned char>(left)) ==
                   std::tolower(static_cast<unsigned char>(right));
        }) != text.end() ? 100u : 0u;
}

std::optional<std::string_view> BuiltinGCodeReaderBase::named_parameter(
    std::string_view line, std::string_view name)
{
    size_t cursor = line.find_first_of(" \t");
    while (cursor != std::string_view::npos) {
        cursor = line.find_first_not_of(" \t", cursor);
        if (cursor == std::string_view::npos || line[cursor] == ';')
            break;
        const size_t end = line.find_first_of(" \t;", cursor);
        const std::string_view word = line.substr(
            cursor, end == std::string_view::npos ? line.size() - cursor : end - cursor);
        if (word.size() > name.size() && word.substr(0, name.size()) == name &&
            word[name.size()] == '=')
            return word.substr(name.size() + 1);
        cursor = end;
    }
    return std::nullopt;
}

BuiltinGCodeReaderBase::BuiltinGCodeReaderBase(orchestrator_handle *orchestrator) :
    GCodeReaderPluginBase(orchestrator)
{}

const char *const *BuiltinGCodeReaderBase::dependencies_impl() const noexcept
{
    return k_no_dependencies;
}

int32_t BuiltinGCodeReaderBase::priority_impl() const noexcept
{
    return 0;
}

bool BuiltinGCodeReaderBase::consume_text_command(
    const plugin_run_context &, const run_ctx_gcode_reader &, GCodeStateTracker &,
    const GCodeLineParser::GCodeLine &) const
{
    return false;
}

bool BuiltinGCodeReaderBase::is_neutral_command(char, uint32_t) const
{
    return false;
}

bool BuiltinGCodeReaderBase::has_unsupported_state_effect(char, uint32_t) const
{
    return false;
}

std::optional<raw_gcode_reader_marker_type>
BuiltinGCodeReaderBase::marker_type(char, uint32_t) const
{
    return std::nullopt;
}

void BuiltinGCodeReaderBase::validate_private_state(const GCodeStateTracker &) const
{}

void BuiltinGCodeReaderBase::read_impl(const plugin_run_context &run_context,
                                       const run_ctx_gcode_reader &context,
                                       GCodeStateTracker &state) const
{
    // A Reader may receive a very large host chunk. Check at the callback
    // boundary and again while that chunk is split into physical lines so a
    // stream without event-producing commands still stops promptly.
    throw_if_cancelled(&run_context);
    validate_private_state(state);
    // Chunks may split anywhere, including in CRLF and numeric parameters.
    // Retain only bytes after the last complete newline between pulls.
    std::string pending;
    std::optional<raw_gcode_reader_marker_type> pending_action;
    std::string_view chunk;
    while (next_chunk(run_context, context, chunk)) {
        pending.append(chunk.data(), chunk.size());
        size_t consumed = 0;
        size_t line_end = 0;
        uint64_t processed_lines = 0;
        while ((line_end = pending.find('\n', consumed)) != std::string::npos) {
            if ((processed_lines++ & 63u) == 0u)
                throw_if_cancelled(&run_context);
            const size_t line_begin = consumed;
            const size_t text_size = line_end > 0 && pending[line_end - 1] == '\r' ? line_end - 1 : line_end;
            const size_t line_size = text_size - line_begin;
            consume_line(run_context, context, state,
                         std::string_view(pending.data() + line_begin, line_size),
                         pending_action);
            consumed = line_end + 1;
            state.advance_line_number();
        }

        // Compact once per host chunk, not once per line. Repeatedly erasing
        // the front would move all remaining bytes for every physical line
        // and make a large input chunk quadratic to consume.
        if (consumed != 0) {
            pending.erase(0, consumed);
        }
    }
    if (!pending.empty()) {
        throw_if_cancelled(&run_context);
        consume_line(run_context, context, state, pending, pending_action);
    }
    validate_private_state(state);
}

void BuiltinGCodeReaderBase::publish_items(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state,
    raw_gcode_reader_event_kind kind,
    const std::vector<raw_gcode_reader_event_item> &items) const
{
    // LINE_NUMBER is mandatory in every event and belongs to this session's
    // cursor, not to a parser branch. Copy the supplied delta before adding it
    // so the caller's temporary vector remains untouched.
    std::vector<raw_gcode_reader_event_item> event_items;
    event_items.reserve(items.size() + 1);
    for (const raw_gcode_reader_event_item &item : items) {
        if (item.type == RAW_GCODE_READER_ITEM_LINE_NUMBER)
            throw std::logic_error("Built-in G-code reader must own line-number event items.");
        event_items.push_back(item);
    }
    event_items.push_back({RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, double(state.line_number())});

    raw_gcode_reader_event event = {};
    event.kind = kind;
    event.items = event_items.data();
    event.item_count = static_cast<uint32_t>(event_items.size());
    publish(run_context, context, state, event);
}

void BuiltinGCodeReaderBase::publish_state_update(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state) const
{
    publish_items(run_context, context, state,
                  RAW_GCODE_READER_EVENT_STATE_UPDATE, {});
}

void BuiltinGCodeReaderBase::publish_diagnostic(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state,
    raw_gcode_reader_diagnostic_type type) const
{
    publish_items(run_context, context, state,
                  RAW_GCODE_READER_EVENT_DIAGNOSTIC,
                  {{RAW_GCODE_READER_ITEM_DIAGNOSTIC_TYPE, 0, double(type)}});
}

void BuiltinGCodeReaderBase::publish_invalid_object_marker(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state) const
{
    publish_diagnostic(run_context, context, state,
                       RAW_GCODE_READER_DIAGNOSTIC_INVALID_COMMAND_PARAMETERS);
    const std::optional<double> object = state.value(RAW_GCODE_READER_ITEM_OBJECT);
    if (object)
        publish_items(run_context, context, state,
                      RAW_GCODE_READER_EVENT_INVALIDATE,
                      {{RAW_GCODE_READER_ITEM_OBJECT, 0, *object}});
}

void BuiltinGCodeReaderBase::publish_unknown(const plugin_run_context &run_context,
                                             const run_ctx_gcode_reader &context,
                                             GCodeStateTracker &state,
                                             raw_gcode_reader_diagnostic_type diagnostic) const
{
    publish_diagnostic(run_context, context, state, diagnostic);
    // One unsupported command may have changed any portable or private value. Send
    // an INVALIDATE item for every currently known value so downstream writers
    // re-establish state explicitly instead of continuing from stale data.
    // Use the exact current keys, including dynamically discovered tool and
    // fan targets. A future extension is therefore invalidated safely too.
    const raw_gcode_reader_state &snapshot = state.state();
    std::vector<raw_gcode_reader_event_item> items;
    for (uint32_t index = 0; index < snapshot.item_count; ++index) {
        if ((index & 63u) == 0u)
            throw_if_cancelled(&run_context);
        const raw_gcode_reader_event_item &item = snapshot.items[index];
        if (item.type != RAW_GCODE_READER_ITEM_LINE_NUMBER &&
            item.type != RAW_GCODE_READER_ITEM_READER_ID)
            items.push_back(item);
    }
    publish_items(run_context, context, state, RAW_GCODE_READER_EVENT_INVALIDATE, items);
}

void BuiltinGCodeReaderBase::process_G28(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state,
    const GCodeLineParser::GCodeLine &line) const
{
    // G28 selects axes by parameter presence; X0 is not a coordinate target.
    // The default Reader follows the common firmware convention that homing
    // establishes logical zero. A firmware-specific Reader may override this
    // virtual handler when its configured home position is different.
    const bool has_x = line.has_parameter('X');
    const bool has_y = line.has_parameter('Y');
    const bool has_z = line.has_parameter('Z');
    std::vector<raw_gcode_reader_event_item> items;
    if (!has_x && !has_y && !has_z) {
        items = {
            {RAW_GCODE_READER_ITEM_X, 0, 0.0},
            {RAW_GCODE_READER_ITEM_Y, 0, 0.0},
            {RAW_GCODE_READER_ITEM_Z, 0, 0.0}
        };
    } else {
        if (has_x)
            items.push_back({RAW_GCODE_READER_ITEM_X, 0, 0.0});
        if (has_y)
            items.push_back({RAW_GCODE_READER_ITEM_Y, 0, 0.0});
        if (has_z)
            items.push_back({RAW_GCODE_READER_ITEM_Z, 0, 0.0});
    }
    publish_items(run_context, context, state,
                  RAW_GCODE_READER_EVENT_SET_POSITION, items);
}

// Parse borrowed comment text and return true when a recognized comment publishes state, a marker,
// or a diagnostic. Invalid parameters publish diagnostics; callback failures propagate and text is not retained.
bool BuiltinGCodeReaderBase::consume_common_comment(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state,
    const GCodeLineParser::GCodeLine &line,
    std::optional<raw_gcode_reader_marker_type> &pending_action) const
{
    const std::string_view comment = trim_presentation(line.comment());
    if (comment.empty())
        return false;

    const auto has_prefix = [](std::string_view text, std::string_view prefix) {
        return text.size() >= prefix.size() &&
               text.compare(0, prefix.size(), prefix) == 0;
    };
    const auto publish_marker = [&](raw_gcode_reader_marker_type marker) {
        publish_items(run_context, context, state,
                      RAW_GCODE_READER_EVENT_MARKER,
                      {{RAW_GCODE_READER_ITEM_MARKER, 0, double(marker)}});
    };

    if (comment == "LAYER_CHANGE") {
        const std::optional<double> current =
            state.value(RAW_GCODE_READER_ITEM_LAYER);
        if (current && *current >= double(UINT32_MAX)) {
            publish_diagnostic(
                run_context, context, state,
                RAW_GCODE_READER_DIAGNOSTIC_INVALID_COMMAND_PARAMETERS);
            return true;
        }
        const uint32_t next_layer = current ?
            static_cast<uint32_t>(*current) + 1u : 0u;
        publish_items(run_context, context, state,
                      RAW_GCODE_READER_EVENT_STATE_UPDATE,
                      {{RAW_GCODE_READER_ITEM_LAYER, 0, double(next_layer)}});
        return true;
    }

    if (comment == "TYPE" || has_prefix(comment, "TYPE:")) {
        const std::string_view name = comment.size() > 5 ?
            trim_presentation(comment.substr(5)) : std::string_view();
        const std::optional<uint32_t> role = raw_extrusion_role_from_name(name);
        if (!role) {
            publish_diagnostic(
                run_context, context, state,
                RAW_GCODE_READER_DIAGNOSTIC_INVALID_COMMAND_PARAMETERS);
            return true;
        }
        publish_items(run_context, context, state,
                      RAW_GCODE_READER_EVENT_STATE_UPDATE,
                      {{RAW_GCODE_READER_ITEM_EXTRUSION_ROLE, 0,
                        double(*role)}});
        return true;
    }

    if (comment == "WIDTH" || has_prefix(comment, "WIDTH:")) {
        double width = 0.0;
        if (comment.size() <= 6 ||
            !parse_finite_comment_number(comment.substr(6), width) ||
            width <= 0.0) {
            publish_diagnostic(
                run_context, context, state,
                RAW_GCODE_READER_DIAGNOSTIC_INVALID_COMMAND_PARAMETERS);
            return true;
        }
        publish_items(run_context, context, state,
                      RAW_GCODE_READER_EVENT_STATE_UPDATE,
                      {{RAW_GCODE_READER_ITEM_WIDTH, 0, width}});
        return true;
    }

    if (comment == "HEIGHT" || has_prefix(comment, "HEIGHT:")) {
        double height = 0.0;
        if (comment.size() <= 7 ||
            !parse_finite_comment_number(comment.substr(7), height) ||
            height <= 0.0) {
            publish_diagnostic(
                run_context, context, state,
                RAW_GCODE_READER_DIAGNOSTIC_INVALID_COMMAND_PARAMETERS);
            return true;
        }
        publish_items(run_context, context, state,
                      RAW_GCODE_READER_EVENT_STATE_UPDATE,
                      {{RAW_GCODE_READER_ITEM_HEIGHT, 0, height}});
        return true;
    }

    // Publish the preceding LAYER_CHANGE's height value as an ordinary state delta;
    // consumers associate the event with that layer instead of inheriting it later.
    if (comment == "MAX_LAYER_HEIGHT" || has_prefix(comment, "MAX_LAYER_HEIGHT:")) {
        double max_layer_height = 0.0;
        if (comment.size() <= 17 ||
            !parse_finite_comment_number(comment.substr(17), max_layer_height) ||
            max_layer_height <= 0.0) {
            publish_diagnostic(
                run_context, context, state,
                RAW_GCODE_READER_DIAGNOSTIC_INVALID_COMMAND_PARAMETERS);
            return true;
        }
        publish_items(run_context, context, state,
                      RAW_GCODE_READER_EVENT_STATE_UPDATE,
                      {{RAW_GCODE_READER_ITEM_MAX_LAYER_HEIGHT, 0, max_layer_height}});
        return true;
    }

    if (comment == "Z" || has_prefix(comment, "Z:")) {
        double z = 0.0;
        if (comment.size() <= 2 ||
            !parse_finite_comment_number(comment.substr(2), z)) {
            publish_diagnostic(
                run_context, context, state,
                RAW_GCODE_READER_DIAGNOSTIC_INVALID_COMMAND_PARAMETERS);
            return true;
        }
        // Z is layer metadata. The following movement, rather than this
        // comment, remains authoritative for the machine position.
        publish_state_update(run_context, context, state);
        return true;
    }

    if (comment == "WIPE_START" || comment == "WIPE_END") {
        const std::optional<double> current = state.value(
            RAW_GCODE_READER_ITEM_EXTRUSION_ROLE);
        const uint32_t current_role = current ?
            static_cast<uint32_t>(*current) : 0u;
        const uint32_t role = comment == "WIPE_START" ?
            current_role | uint32_t(RAW_EXTRUSION_ROLE_WIPE) :
            current_role & ~uint32_t(RAW_EXTRUSION_ROLE_WIPE);
        publish_items(run_context, context, state,
                      RAW_GCODE_READER_EVENT_STATE_UPDATE,
                      {{RAW_GCODE_READER_ITEM_EXTRUSION_ROLE, 0,
                        double(role)}});
        return true;
    }

    if (comment == "SEAM" || has_prefix(comment, "SEAM:")) {
        const std::optional<std::pair<double, double>> coordinates =
            comment.size() > 5 ? parse_seam_coordinates(comment.substr(5)) :
                                 std::nullopt;
        if (!coordinates) {
            publish_diagnostic(
                run_context, context, state,
                RAW_GCODE_READER_DIAGNOSTIC_INVALID_COMMAND_PARAMETERS);
            return true;
        }
        publish_marker(RAW_GCODE_READER_MARKER_SEAM);
        return true;
    }

    if (comment == "COLOR_CHANGE") {
        publish_marker(RAW_GCODE_READER_MARKER_COLOR_CHANGE);
        pending_action = RAW_GCODE_READER_MARKER_COLOR_CHANGE;
        return true;
    }
    constexpr std::string_view color_change_prefix = "COLOR_CHANGE,";
    if (has_prefix(comment, color_change_prefix)) {
        if (!valid_color_change_payload(comment.substr(color_change_prefix.size()))) {
            publish_diagnostic(
                run_context, context, state,
                RAW_GCODE_READER_DIAGNOSTIC_INVALID_COMMAND_PARAMETERS);
            return true;
        }
        publish_marker(RAW_GCODE_READER_MARKER_COLOR_CHANGE);
        pending_action = RAW_GCODE_READER_MARKER_COLOR_CHANGE;
        return true;
    }
    if (comment == "PAUSE_PRINT") {
        publish_marker(RAW_GCODE_READER_MARKER_PAUSE_PRINT);
        pending_action = RAW_GCODE_READER_MARKER_PAUSE_PRINT;
        return true;
    }
    if (comment == "CUSTOM_GCODE") {
        publish_marker(RAW_GCODE_READER_MARKER_CUSTOM_GCODE);
        pending_action = RAW_GCODE_READER_MARKER_CUSTOM_GCODE;
        return true;
    }
    return false;
}

void BuiltinGCodeReaderBase::consume_line(const plugin_run_context &run_context,
                                         const run_ctx_gcode_reader &context,
                                         GCodeStateTracker &state,
                                         std::string_view text,
                                         std::optional<raw_gcode_reader_marker_type> &pending_action) const
{
    // Empty/comment-only lines do not publish an event, so the outer line
    // splitter cannot rely on publish() to observe cancellation for them.
    throw_if_cancelled(&run_context);
    // Convert one parsed command into the shared event vocabulary. Commands
    // outside the common subset and the derived dialect hooks invalidate
    // portable state so later output never relies on guessed machine values.
    GCodeLineParser parser;
    parser.parse_line(text,
                      [this, &run_context, &context, &state, &pending_action](
                          GCodeLineParser &, const GCodeLineParser::GCodeLine &line) {
        const std::string_view command = line.command();
        if (command.empty()) {
            if (consume_common_comment(
                    run_context, context, state, line, pending_action))
                return;
            constexpr std::string_view marker = "SLIC3R_OBJECT_INSTANCE_ID=";
            const std::string_view comment = line.comment();
            const size_t marker_pos = comment.find(marker);
            if (marker_pos != std::string_view::npos) {
                const std::string_view value = comment.substr(marker_pos + marker.size());
                int64_t marker_id = 0;
                const std::from_chars_result parsed = std::from_chars(
                    value.data(), value.data() + value.size(), marker_id);
                if (parsed.ec == std::errc() && parsed.ptr == value.data() + value.size() &&
                    marker_id >= -1 && marker_id <= int64_t(UINT32_MAX)) {
                    const uint32_t object_id = marker_id < 0 ? UINT32_MAX :
                        uint32_t(marker_id);
                    const std::optional<double> current = state.value(
                        RAW_GCODE_READER_ITEM_OBJECT);
                    if (!current || static_cast<uint32_t>(*current) != object_id)
                        publish_items(run_context, context, state,
                                      RAW_GCODE_READER_EVENT_STATE_UPDATE,
                                      {{RAW_GCODE_READER_ITEM_OBJECT, 0, double(object_id)}});
                }
                else
                    publish_invalid_object_marker(run_context, context, state);
                pending_action.reset();
            }
            return;
        }

        const char letter = command.front();
        const std::optional<uint32_t> number = command_number(command);
        std::optional<raw_gcode_reader_marker_type> command_action;
        if (command == "PAUSE")
            command_action = RAW_GCODE_READER_MARKER_PAUSE_PRINT;
        else if (number)
            command_action = marker_type(letter, *number);
        if (pending_action) {
            const bool duplicate = command_action == pending_action;
            pending_action.reset();
            if (duplicate)
                return;
        }
        if (consume_text_command(run_context, context, state, line))
            return;

        if (!number)
            return publish_unknown(run_context, context, state,
                                   RAW_GCODE_READER_DIAGNOSTIC_UNKNOWN_COMMAND);
        if (consume_numeric_command(run_context, context, state, line,
                                    letter, *number))
            return;

        if (has_unsupported_state_effect(letter, *number)) {
            publish_diagnostic(
                run_context, context, state,
                RAW_GCODE_READER_DIAGNOSTIC_UNSUPPORTED_STATE_EFFECT);
            return;
        } else if (is_neutral_command(letter, *number)) {
            // A neutral command is known not to change any modeled machine
            // state. Keep an event for its source line without warning.
        } else if (command_action) {
            publish_items(run_context, context, state,
                          RAW_GCODE_READER_EVENT_MARKER,
                          {{RAW_GCODE_READER_ITEM_MARKER, 0, double(*command_action)}});
            return;
        } else {
            publish_unknown(run_context, context, state,
                            RAW_GCODE_READER_DIAGNOSTIC_UNKNOWN_COMMAND);
            return;
        }
        publish_state_update(run_context, context, state);
    });
}

bool BuiltinGCodeReaderBase::consume_numeric_command(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state,
    const GCodeLineParser::GCodeLine &line,
    char letter,
    uint32_t number) const
{
    if (letter == 'G' && (number == 0 || number == 1))
        process_G0_G1(run_context, context, state, line);
    else if (letter == 'G' && (number == 2 || number == 3))
        process_G2_G3(run_context, context, state, line, number);
    else if (letter == 'G' && number == 28)
        process_G28(run_context, context, state, line);
    else if (letter == 'G' && (number == 20 || number == 21))
        process_G20_G21(run_context, context, state, number);
    else if (letter == 'G' && (number == 90 || number == 91))
        process_G90_G91(run_context, context, state, number);
    else if (letter == 'G' && number == 92)
        process_G92(run_context, context, state, line);
    else if (letter == 'M' && (number == 82 || number == 83))
        process_M82_M83(run_context, context, state, number);
    else if (letter == 'T')
        process_T(run_context, context, state, number);
    else if (letter == 'M' && (number == 104 || number == 109))
        process_M104_M109(run_context, context, state, line, number);
    else if (letter == 'M' && (number == 140 || number == 190))
        process_M140_M190(run_context, context, state, line, number);
    else if (letter == 'M' && (number == 141 || number == 191))
        process_M141_M191(run_context, context, state, line, number);
    else if (letter == 'M' && (number == 106 || number == 107))
        process_M106_M107(run_context, context, state, line, number);
    else if (letter == 'M' && number == 220)
        process_M220(run_context, context, state, line);
    else if (letter == 'M' && number == 221)
        process_M221(run_context, context, state, line);
    else if (letter == 'M' && number == 117)
        process_M117(run_context, context, state);
    else if (letter == 'M' && number == 486)
        process_M486(run_context, context, state, line);
    else if (letter == 'M' && (number == 17 || number == 18 || number == 84))
        // Motor enable/disable is not part of the portable state model. The
        // command is nevertheless recognized so it does not invalidate
        // unrelated coordinates or appear as an unknown-command warning.
        publish_state_update(run_context, context, state);
    else
        return false;
    return true;
}

void BuiltinGCodeReaderBase::process_M486(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state,
    const GCodeLineParser::GCodeLine &line) const
{
    int32_t firmware_id = 0;
    if (!line.has_parameter('S')) {
        publish_state_update(run_context, context, state);
        return;
    }
    if (!line.parameter_value('S', firmware_id) || firmware_id < -1) {
        publish_invalid_object_marker(run_context, context, state);
        return;
    }
    const uint32_t object_id = firmware_id < 0 ? UINT32_MAX : uint32_t(firmware_id);
    const std::optional<double> current = state.value(RAW_GCODE_READER_ITEM_OBJECT);
    if (current && static_cast<uint32_t>(*current) == object_id)
        return;
    publish_items(run_context, context, state, RAW_GCODE_READER_EVENT_STATE_UPDATE,
                  {{RAW_GCODE_READER_ITEM_OBJECT, 0, double(object_id)}});
}

void BuiltinGCodeReaderBase::process_G0_G1(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state,
    const GCodeLineParser::GCodeLine &line) const
{
    ParsedMovementItems movement = parse_movement_items(state, line);
    if (!movement.invalidated.empty())
        publish_items(run_context, context, state, RAW_GCODE_READER_EVENT_INVALIDATE, movement.invalidated);
    publish_items(run_context, context, state, RAW_GCODE_READER_EVENT_LINEAR_MOVE, movement.values);
}

void BuiltinGCodeReaderBase::process_G2_G3(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state,
    const GCodeLineParser::GCodeLine &line,
    uint32_t number) const
{
    ParsedMovementItems movement = parse_movement_items(state, line);
    const std::pair<char, uint32_t> centers[] = {
        {'I', RAW_GCODE_READER_ITEM_ARC_CENTER_X},
        {'J', RAW_GCODE_READER_ITEM_ARC_CENTER_Y},
        {'K', RAW_GCODE_READER_ITEM_ARC_CENTER_Z}
    };
    for (const std::pair<char, uint32_t> &center : centers)
        if (const std::optional<float> value = line.parameter_value(center.first))
            movement.values.push_back({center.second, 0,
                                       double(*value) * unit_scale(state)});
    movement.values.push_back({RAW_GCODE_READER_ITEM_ARC_CLOCKWISE, 0,
                               number == 2 ? 1.0 : 0.0});
    if (!movement.invalidated.empty())
        publish_items(run_context, context, state, RAW_GCODE_READER_EVENT_INVALIDATE, movement.invalidated);
    publish_items(run_context, context, state, RAW_GCODE_READER_EVENT_ARC_MOVE, movement.values);
}

void BuiltinGCodeReaderBase::process_G20_G21(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state,
    uint32_t number) const
{
    publish_items(run_context, context, state, RAW_GCODE_READER_EVENT_STATE_UPDATE, {{RAW_GCODE_READER_ITEM_UNITS_MM, 0,
                            number == 21 ? 1.0 : 0.0}});
}

void BuiltinGCodeReaderBase::process_G90_G91(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state,
    uint32_t number) const
{
    publish_items(run_context, context, state, RAW_GCODE_READER_EVENT_STATE_UPDATE, {{RAW_GCODE_READER_ITEM_XYZ_ABSOLUTE, 0,
                            number == 90 ? 1.0 : 0.0}});
}

void BuiltinGCodeReaderBase::process_G92(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state,
    const GCodeLineParser::GCodeLine &line) const
{
    std::vector<raw_gcode_reader_event_item> items;
    const std::pair<char, uint32_t> axes[] = {
        {'X', RAW_GCODE_READER_ITEM_X}, {'Y', RAW_GCODE_READER_ITEM_Y},
        {'Z', RAW_GCODE_READER_ITEM_Z}, {'E', RAW_GCODE_READER_ITEM_E}
    };
    for (const std::pair<char, uint32_t> &axis : axes)
        if (const std::optional<float> value = line.parameter_value(axis.first))
            items.push_back({axis.second, axis.first == 'E' ? active_tool(state) : 0,
                             double(*value) * unit_scale(state)});
    publish_items(run_context, context, state, RAW_GCODE_READER_EVENT_SET_POSITION, items);
}

void BuiltinGCodeReaderBase::process_M82_M83(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state,
    uint32_t number) const
{
    publish_items(run_context, context, state, RAW_GCODE_READER_EVENT_STATE_UPDATE, {{RAW_GCODE_READER_ITEM_E_ABSOLUTE, 0,
                            number == 82 ? 1.0 : 0.0}});
}

void BuiltinGCodeReaderBase::process_T(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state,
    uint32_t tool) const
{
    publish_items(run_context, context, state, RAW_GCODE_READER_EVENT_STATE_UPDATE, {{RAW_GCODE_READER_ITEM_ACTIVE_TOOL, 0, double(tool)}});
}

void BuiltinGCodeReaderBase::process_M104_M109(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state,
    const GCodeLineParser::GCodeLine &line,
    uint32_t number) const
{
    std::vector<raw_gcode_reader_event_item> items;
    const std::optional<float> temperature = line.parameter_value('S');
    if (!temperature) {
        publish_state_update(run_context, context, state);
        return;
    }
    int32_t requested_tool = int32_t(active_tool(state));
    line.parameter_value('T', requested_tool);
    if (requested_tool >= 0)
        items.push_back({RAW_GCODE_READER_ITEM_TOOL_TEMPERATURE,
                         uint32_t(requested_tool), double(*temperature)});
    publish_items(run_context, context, state,
                  number == 109 ? RAW_GCODE_READER_EVENT_WAIT_TEMPERATURE :
                                  RAW_GCODE_READER_EVENT_SET_TEMPERATURE, items);
}

void BuiltinGCodeReaderBase::process_M140_M190(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state,
    const GCodeLineParser::GCodeLine &line,
    uint32_t number) const
{
    std::vector<raw_gcode_reader_event_item> items;
    const std::optional<float> temperature = line.parameter_value('S');
    if (!temperature) {
        publish_state_update(run_context, context, state);
        return;
    }
    items.push_back({RAW_GCODE_READER_ITEM_BED_TEMPERATURE, 0,
                     double(*temperature)});
    publish_items(run_context, context, state,
                  number == 190 ? RAW_GCODE_READER_EVENT_WAIT_TEMPERATURE :
                                  RAW_GCODE_READER_EVENT_SET_TEMPERATURE, items);
}

void BuiltinGCodeReaderBase::process_M141_M191(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state,
    const GCodeLineParser::GCodeLine &line,
    uint32_t number) const
{
    std::vector<raw_gcode_reader_event_item> items;
    const std::optional<float> temperature = line.parameter_value('S');
    if (!temperature) {
        publish_state_update(run_context, context, state);
        return;
    }
    items.push_back({RAW_GCODE_READER_ITEM_CHAMBER_TEMPERATURE, 0,
                     double(*temperature)});
    publish_items(run_context, context, state,
                  number == 191 ? RAW_GCODE_READER_EVENT_WAIT_TEMPERATURE :
                                  RAW_GCODE_READER_EVENT_SET_TEMPERATURE, items);
}

void BuiltinGCodeReaderBase::process_M106_M107(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state,
    const GCodeLineParser::GCodeLine &line,
    uint32_t number) const
{
    std::vector<raw_gcode_reader_event_item> items;
    int32_t fan = 0;
    line.parameter_value('P', fan);
    if (fan >= 0) {
        const std::optional<float> raw_speed = number == 107 ?
            std::optional<float>(0.0f) : line.parameter_value('S');
        if (raw_speed)
            items.push_back({RAW_GCODE_READER_ITEM_FAN_SPEED, uint32_t(fan),
                             std::clamp(double(*raw_speed) * 100.0 / 255.0,
                                        0.0, 100.0)});
    }
    publish_items(run_context, context, state, RAW_GCODE_READER_EVENT_STATE_UPDATE, items);
}

void BuiltinGCodeReaderBase::process_M220(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state,
    const GCodeLineParser::GCodeLine &line) const
{
    std::vector<raw_gcode_reader_event_item> items;
    if (const std::optional<float> factor = line.parameter_value('S'))
        items.push_back({RAW_GCODE_READER_ITEM_SPEED_FACTOR, 0, double(*factor)});
    publish_items(run_context, context, state, RAW_GCODE_READER_EVENT_STATE_UPDATE, items);
}

void BuiltinGCodeReaderBase::process_M221(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state,
    const GCodeLineParser::GCodeLine &line) const
{
    std::vector<raw_gcode_reader_event_item> items;
    if (const std::optional<float> factor = line.parameter_value('S'))
        items.push_back({RAW_GCODE_READER_ITEM_EXTRUSION_FACTOR, 0, double(*factor)});
    publish_items(run_context, context, state, RAW_GCODE_READER_EVENT_STATE_UPDATE, items);
}

void BuiltinGCodeReaderBase::process_M117(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state) const
{
    // Display text is deliberately absent from the numeric Reader contract.
    // Recognizing M117 preserves modal state without fabricating a warning.
    publish_state_update(run_context, context, state);
}

}}} // namespace slic3r_api::GCodeGeneration::Reader
