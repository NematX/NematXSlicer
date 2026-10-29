///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "BambuGCodeReader.hpp"
#include "BambuGCodeReaderItems.hpp"

#include <algorithm>
#include <optional>
#include <stdexcept>
#include <string_view>
#include <vector>

/*
Bambu Lab Reader provider
=========================

This file lists the Bambu-only commands the Reader must accept without losing
its state, and decides when a file is recognized as a Bambu file. The class
inherits every Marlin 2 behavior and adds:

  - k_neutral_commands: M numbers which are plain integers. They are matched in
  - k_neutral_subcodes: dotted words such as `M622.1`. The common parser cannot
    turn them into a number, so they are matched as whole words in
  - `M620.1`, a dotted word handled separately in consume_text_command(): it
    is neutral only without an X, Y or Z parameter (see below).
  - process_M106_M107(): Bambu fans. P1 or no P is the part fan of the active
    tool, P2 the platter fan and P3 the exhaust fan; the last two have their own
    items (BambuGCodeReaderItems.hpp) because FAN_SPEED targets a tool.
  - probe_impl(): detection from the comment lines at the top of the file.

AMS (Automatic Material System) is the Bambu Lab multi-spool filament feeder;
its commands are written around each filament change.

Meaning of the integer commands (none of them changes a modeled value):
  M73          print progress / remaining time (M73 L is the layer number)
  M400         wait for the end of the moves (M400 P<ms> adds a pause)
  M620/M621    begin/end of an AMS filament change block (S<slot>A)
  M622/M623    begin/end of a block run only when a condition is true; the
               condition is set by `M1002 judge_flag <name>` (J1 / J0)
  M624/M625    begin/end of a skippable object block; M624 carries a Base64
               bitmask of the object, so the printer can cancel one object
  M628/M629    AMS related blocks; meaning not documented
  M960         chamber lamp
  M970/M974    vibration resonance measurement ("mech mode")
  M971         timelapse photo
  M972/M973    scanner and laser
  M975         vibration suppression
  M976/M977    first-layer scan
  M981         spaghetti detector on/off
  M982         motor noise reduction
  M991         layer-change notification (M991 S0 P<layer>)
  M1002        firmware directive: status text (gcode_claim_action),
               condition flag (judge_flag), speed level
  M1003        power-loss recovery on/off

Meaning of the dotted sub-codes:
  M620.1       AMS filament options (E = extrude feed rate, F, T = temperature)
  M620.10, M620.11  AMS long retraction at the cut
  M622.1       option of the conditional block
  M970.3       resonance measurement parameters
  M201.2       meaning not documented
  M73.2        remaining-time factor
  M982.2       motor noise reduction option
  G29.2        automatic bed leveling on/off

`M620.1` also exists with X, Y and/or Z: during an AMS filament change it moves
the toolhead along a path. A move the Reader does not model must not be read as
harmless, so that form is not neutral and invalidates the state like any
unknown command.

Not listed on purpose: G29 (probing; the end position is not known) and G29.1
(Z offset). They stay unknown, so they invalidate the state.

Detection: every Bambu file is also valid Marlin 2 text, so this Reader scores 0
by default. It scores 100 only when a comment line before the first executable
command contains a marker which only Bambu printers produce:
  - `BambuStudio`: the generator comment written by Bambu Studio;
  - `printer_model = Bambu Lab`: the printer line of the configuration block
    written by Orca Slicer and Bambu Studio.
`OrcaSlicer` alone is not a marker: Orca also writes it for Klipper or Marlin
printers, which belong to their own Reader. A marker after the first command
(or inside a command such as `M117 BambuStudio`) is ignored.

Operation notes:
- is_neutral_command(), which the common parser calls after the numeric
- handlers. Text after the command word is ignored, so
- M1002 gcode_claim_action : 1` and `M620 S0A` are accepted like `M624`.
- consume_text_command(), before the parser would report them as unknown.
- The comparison is exact: `M620.12` is not `M620.10` and stays unknown.

Representative direct calls (branches are not a timeline):

    BambuGCodeReader::consume_text_command()
        |-- Marlin2GCodeReader::consume_text_command()
        |-- line.command()
        |-- line.has_parameter()
        `-- publish_state_update()

    BambuGCodeReader::probe_impl()
        |-- prefix.substr()
        |-- line.find_first_not_of()
        `-- probe_marker()
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Reader {

namespace {

// M numbers with no effect on the modeled state. M17/M18 are not listed: the
// common parser already knows them.
constexpr uint32_t k_neutral_commands[] = {
    73, 400, 620, 621, 622, 623, 624, 625, 628, 629, 960, 970, 971, 972, 973,
    974, 975, 976, 977, 981, 982, 991, 1002, 1003
};

// Dotted sub-commands, matched as complete words. M620.1 is handled apart.
constexpr std::string_view k_neutral_subcodes[] = {
    "M620.10", "M620.11", "M622.1", "M970.3", "M201.2", "M73.2", "M982.2",
    "G29.2"
};

// Comment texts which only Bambu printers produce; see the file header.
constexpr std::string_view k_header_markers[] = {
    "BambuStudio", "printer_model = Bambu Lab"
};

} // namespace

// Bind the Reader to the orchestrator; the commands come from Marlin 2. The state
// identity is registered here so the fan items carry a private schema identity.
BambuGCodeReader::BambuGCodeReader(orchestrator_handle *orchestrator) :
    Marlin2GCodeReader(orchestrator),
    m_reader_state_id(orchestrator_register_runtime_id(
        orchestrator, GCODE_READER_STATE_ID_TYPE, BAMBU_READER_STATE_NAME))
{
    if (m_reader_state_id == ORCHESTRATOR_RUNTIME_ID_INVALID)
        throw std::runtime_error("The Bambu Reader state identity could not be registered.");
}

// Stable plugin identity used to select this Reader explicitly.
const char *BambuGCodeReader::id_impl() const noexcept
{
    return "gcode.reader.bambu";
}

// Name shown in plugin lists.
const char *BambuGCodeReader::name_impl() const noexcept
{
    return "Bambu Lab G-code reader";
}

// Short description shown in plugin lists.
const char *BambuGCodeReader::description_impl() const noexcept
{
    return "Reads Bambu Lab G-code into normalized machine-state events.";
}

// Scan the lines of the prefix up to the first executable command; blank lines
// and ';' comments are the only lines inspected. A prefix cut in the middle of
// a line only shortens the last line, which is harmless.
uint32_t BambuGCodeReader::probe_impl(std::string_view prefix) const
{
    size_t position = 0;
    while (position < prefix.size()) {
        size_t end = prefix.find('\n', position);
        if (end == std::string_view::npos)
            end = prefix.size();
        const std::string_view line = prefix.substr(position, end - position);
        position = end + 1;

        const size_t first = line.find_first_not_of(" \t\r");
        if (first == std::string_view::npos)
            continue;
        // The first command ends the header: later comments (and the text of
        // commands such as M117) are not trustworthy markers.
        if (line[first] != ';')
            return 0;
        for (const std::string_view marker : k_header_markers)
            if (probe_marker(line, marker) != 0)
                return 100;
    }
    return 0;
}

// Let the parents handle their text commands, then accept a Bambu dotted
// sub-code as a known command which publishes an unchanged state. M620.1 with
// X, Y or Z is a toolhead path: it is left unconsumed, so it is reported as
// unknown.
bool BambuGCodeReader::consume_text_command(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state,
    const GCodeLineParser::GCodeLine &line) const
{
    if (Marlin2GCodeReader::consume_text_command(
            run_context, context, state, line))
        return true;
    const std::string_view command = line.command();
    if (command == "M620.1") {
        if (line.has_parameter('X') || line.has_parameter('Y') ||
            line.has_parameter('Z'))
            return false;
        publish_state_update(run_context, context, state);
        return true;
    }
    for (const std::string_view subcode : k_neutral_subcodes) {
        if (command == subcode) {
            publish_state_update(run_context, context, state);
            return true;
        }
    }
    return false;
}

// Bambu reads P as a fan number. P1 or no P is the part fan of the active tool
// (FAN_SPEED of that tool), P2 the platter fan and P3 the exhaust fan (their own
// items, never FAN_SPEED, whose target is a tool). M107 is a speed of zero. The
// Marlin 2 parser keeps every other case: P0, P4..., or a part fan command while
// no tool is known. As in the Marlin 2 parser, a command without S sets nothing.
void BambuGCodeReader::process_M106_M107(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state,
    const GCodeLineParser::GCodeLine &line,
    uint32_t number) const
{
    int32_t fan = 0;
    const bool has_fan = line.parameter_value('P', fan);
    const std::optional<uint32_t> tool = state.active_tool();
    const bool platter = has_fan && fan == 2;
    const bool exhaust = has_fan && fan == 3;
    const bool part_fan = tool && (!has_fan || fan == 1);
    if (!platter && !exhaust && !part_fan) {
        Marlin2GCodeReader::process_M106_M107(run_context, context, state, line, number);
        return;
    }
    std::vector<raw_gcode_reader_event_item> items;
    const std::optional<float> raw_speed = number == 107 ?
        std::optional<float>(0.0f) : line.parameter_value('S');
    if (raw_speed) {
        const uint32_t type = platter ? uint32_t(BAMBU_ITEM_PLATTER_FAN_SPEED) :
            exhaust ? uint32_t(BAMBU_ITEM_EXHAUST_FAN_SPEED) : uint32_t(RAW_GCODE_READER_ITEM_FAN_SPEED);
        items.push_back({type, part_fan ? *tool : 0u,
                         std::clamp(double(*raw_speed) * 100.0 / 255.0, 0.0, 100.0)});
    }
    publish_items(run_context, context, state, RAW_GCODE_READER_EVENT_STATE_UPDATE, items);
}

// Accept the Marlin 2 neutral commands, then any listed Bambu M number; the
// parameters are never inspected, so malformed ones are accepted too.
bool BambuGCodeReader::is_neutral_command(char letter, uint32_t number) const
{
    if (Marlin2GCodeReader::is_neutral_command(letter, number))
        return true;
    if (letter != 'M')
        return false;
    for (const uint32_t neutral : k_neutral_commands)
        if (number == neutral)
            return true;
    return false;
}

}}} // namespace slic3r_api::GCodeGeneration::Reader
