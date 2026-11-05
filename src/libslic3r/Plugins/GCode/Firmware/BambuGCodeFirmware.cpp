///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "BambuGCodeFirmware.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <locale>
#include <map>
#include <sstream>
#include <string>

#include <boost/beast/core/detail/base64.hpp>

#include "libslic3r/Format/CustomVariables.hpp"
#include "libslic3r/Plugins/GCode/Reader/BambuGCodeReaderItems.hpp"

/*
This session extends the shared Marlin 2 traversal with Bambu command syntax and
the boundaries needed for object cancellation. A cancellable block contains
moves for one real object; cancelling that object skips those moves. Global fan
and tool-selection work therefore has to remain outside such blocks.

Retraction debt is filament pulled back but not yet restored. The entry/exit
rules below balance that debt for ordinary object blocks; the documented deeper
tool-change retraction is an exception, not an additional guarantee. The command
catalogue explains placement rules, while the shared base still owns traversal
and persistent machine-state accounting.

Bambu Lab firmware implementation
=================================

`BambuGCodeFirmwareSession` is a Marlin 2 session that adds the commands the
Bambu slicers write. The inherited session decides when things happen; this file
only provides the Bambu text and a few placement rules.

Bambu commands written, and where they appear in the file:

    start_gcode ...                       (everything of the start sequence)
    M106 P3 S<speed>                           optional exhaust fan, before M981
    M981 S1 P20000 ;open spaghetti detector    end of begin_print(), before the first layer
    ;LAYER_CHANGE ...
    (layer_gcode of the group's second layer)
    M1003 S<0|1>                               optional, first tool group of the group's second layer
    G1 E-<retract_length>                      optional scan, framed by a retraction pair when the tool is not retracted
    M976 S1 P1 ; scan model before printing 2nd layer
    M400 P100
    G1 E<retract_length>
    M106 P2 S<speed>                           optional platter fan, after tool selection,
                                               outside object blocks and before geometry
      G1 E-<complement>                        retraction up to retract_length, only when the
                                               tool is less retracted (or G10)
      ; start printing object, unique label id: <id>
      M624 <base64 mask>                       start of an object; in layer printing the
                                               travel to the object comes after this label
      G1 E<debt>                               explicit unretraction (or G11) after the
                                               approach, before the first deposited geometry,
                                               only when a debt remains
      ... extrusions, optionally `M900 K<pa> L1000 M10 ; Override pressure advance value`
      G1 E-<complement>                        retraction up to the entry depth before the end
                                               label, inside the block, only when needed
      ; stop printing object, unique label id: <id>
      M625
      G92 E0                                   absolute E only, right after M625
    M1003 S0                                  end of every group, only if power loss recovery is enabled
    M106 P2 S0                                stop the platter fan if still running
    M981 S0 P20000 ; close spaghetti detector  end of the last group, before end_gcode

Object labels: the printer skips the moves between `M624 <mask>` and `M625` when
the object whose bit is set in `<mask>` is cancelled. The mask is a 64-bit integer
with only bit `rank` set (`rank` = position of the object among the real, non
synthetic, objects of the plan sorted by id), written as 8 little-endian bytes in
standard Base64 (12 characters with `=`). The `<id>` of the comment is the plan
catalog id. Labels are written when `gcode_label_objects` is `firmware` or `both`
(rule of the base session). A plan with more than 64 real objects gets no label at
all, because a bit cannot be given to every object.

`G92 E0` after `M625`: a cancelled object's E moves are skipped by the printer, so
in absolute E the next extrusion would push all the skipped distance. Zeroing E
after each object makes the next E independent of what was skipped (relative E
needs nothing). The reset itself is done by the base session through
`resets_extrusion_axis_after_object_end()`, before the next move's E is prepared.

Balanced blocks aim to leave filament as they found it, because the printer skips
the lines of a cancelled object. The deeper tool-change retraction below is an exception. Between two objects the plugins already do it: the
retraction and wipe of the finished object are in its block (before `M625`), the
approach and the unretraction of the next one are in its block (after `M624`), so each
block is "unretract ... retract". Three cases are not covered by the plugins: the first
block, a block that follows a script or the wipe tower (the tower's retraction is
outside any block), and a block that ends at a layer change where the plan has no
retraction. For these ordinary cases, the base session enters and leaves a labelled block with
the same debt (object_block_retract_length() answers the tool's `retract_length`):
  - just before the start label, it retracts the complement up to `retract_length`
    (nothing if the tool is already that deep, so no pair is added between two balanced
    blocks); this retraction is outside the block and runs even if the object is skipped.
    The entry depth is the tool's debt after it, so the larger of its previous debt and
    `retract_length`. With native retraction (G10) already active nothing is written and
    no logical complement is recorded: the firmware keeps its own depth;
  - inside the block, after the approach and before the first deposited geometry, it
    repays any remaining retraction with an explicit unretraction, so the first
    extrusion is complete (without it the base would repay the debt on the flow of the
    first extrusion and the extrusion would be short);
  - just before the end label, inside the block, it retracts the complement up to the
    entry depth (the retraction comes before the `G92 E0`). A tool change closes the
    block first (select_extruder()), so the tool is still the block's tool.
A tool change is never written inside a block, whatever the tool's retraction depth
(even 0): closes_object_block_before_tool_change() makes the base session close the open
block before it selects another tool, so a cancelled object does not skip the change.
If geometry of the same tree follows that change, the base visitor reopens the
object through the normal transition before preparing its E: pending P2 outside
the block, entry retraction, M624, then explicit unretraction before deposition.

For ordinary blocks (excluding the tool-change case below), exit debt matches
entry debt, so cancelling such a block does not
change the filament state. Known limit: the plan's tool-change retraction
(`retract_length_toolchange`, deeper than `retract_length`) is written inside the block
that precedes the tool change, so that block ends deeper than its entry depth and is not
normalized; cancelling it leaves the tool at its entry depth. The per-region override
`print_retract_length` is not read. With native retraction (G10) already active, the
firmware's depth is the real one: a deeper retraction of the plan is the base session's
existing behaviour (the printer is already retracted, no second G10 is written) and the
exit retraction goes back to the entry depth only; this limit is accepted. The terminal
retraction of the plan is outside any
block (if the last object is cancelled it still runs, just before `end_gcode`, with no
effect on the print).

Known difference with Orca, sequential (by object) printing, traced with and without a
travel plugin: begin_group() of the base session writes the approach to the next object
between `M625` and `M624` when the first point of the object's tree differs from the
current position. Without a travel plugin (as in the plugin tests) the tree starts with
the first printed point, so the approach is written there and a cancelled object does
not skip it. With the default travel plugin the tree starts with the travel, whose first
point is the current position, so begin_group() writes nothing (write_lines() has no
move to make) and the whole approach is inside the block, as in layer printing.
An empty plan (no group) writes `M981 S1` but no `M981 S0`, because the closing
command is tied to the end of the last group.

Optional variables: `printer_custom_variables` (one `name = value` per line, not a
declared setting, so absent means nothing is written) may hold `scan_first_layer`
(1 or true, any case: write the scan) and `enable_power_loss_recovery` (`enable`:
`M1003 S1` and `M1003 S0` at the end of each group; `disable`: `M1003 S0` only; any
other value, `printer_configuration` included: nothing). They are written at the
start of the first tool group of the second layer of every group (index 1 of the
group's layer counter), after layer_gcode and before the first move, outside object
blocks, since a cancelled object would otherwise skip them. The scan is framed by the
base session's write_with_retraction() with the tool's `retract_length`: a tool that
is not retracted is retracted, scanned, then unretracted; one already retracted only
gets the complement, so the extrusion that follows never repays twice.

Optional fan settings are created by the Bambu firmware provider. `platter_fan`
enables P2 at the selected filament's `platter_fan_speed`; it stays zero while the
group's zero-based layer index is below `disable_fan_first_layers` (zero if that
optional fan-plugin setting is absent). Layer/tool changes defer the command until
the selected tool group or the next object opening/geometry, outside cancellable
blocks, and identical speeds are not repeated. Each sequential
group restarts the layer counter. `support_air_filtration` enables P3 if a used
filament has `activate_air_filtration`: only activated, used filaments enter the
maximum of `during_print_exhaust_fan_speed` and `complete_print_exhaust_fan_speed`.
Both fan channels convert percent to the integer part of 255 * percent / 100.
These commands do not update the inherited part-cooling fan state.

Custom scripts (the profile's start, layer or filament scripts) may write the same
fans. In a script, `M106`/`M107` read P as a fan number: P1 or no P is the part fan
of the selected tool, P2 and P3 are not the fans of the tools 2 and 3
(script_fan_tool(), used by the script interpreter of the base session). The Bambu
Reader publishes P2 and P3 as the items of BambuGCodeReaderItems.hpp, under the
Bambu Reader identity that the firmware provider registers for the session. Before
a script the session exports the platter speed written last (when known); after
it, synchronize_reader_private_state() adopts the speed the Reader reports, so the
next change is compared with the real fan. The speed becomes unknown (and is
written again at the next point outside any block) when the script changed it
inside an object block, as a cancelled object skips the block, or when the Reader
lost it (an unknown command discards the exported item). A pending speed is never
written from inside a block: it waits for the next object opening or tool group.
This includes the speed of a new filament after a TOOLCHANGE inside the tree of a
labelled object: the tool change closes the block, and the new speed is written
before the geometry that follows, outside the reopened block (a tool group boundary
or an object opening is the only other point where it can be written).
The end of the last group stops the platter fan unless it is known to be stopped,
and never when the platter fan is disabled, which is not tracked at all (nothing is
exported to the Reader nor adopted from it). The exhaust fan is not followed.

Operation notes:
- M106 P3 S<speed>                           optional completion exhaust speed

Representative direct calls (branches are not a timeline):

    BambuGCodeFirmwareSession::begin_print()
        |-- plan.object_instance()
        |-- Marlin2GCodeFirmwareSession::begin_print()
        `-- plan.group()

    BambuGCodeFirmwareSession::begin_layer()
        `-- Marlin2GCodeFirmwareSession::begin_layer()

    BambuGCodeFirmwareSession::begin_tool_group()
        |-- Marlin2GCodeFirmwareSession::begin_tool_group()
        |-- write_second_layer_commands()
        `-- write_platter_fan()
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Firmware {
namespace {

// Maximum number of objects a 64-bit cancellation mask can address.
constexpr size_t k_max_labelled_objects = 64;

// Encode `1 << rank` as 8 little-endian bytes in standard Base64 with padding.
// The caller guarantees rank < 64 (ranks come from a table limited to 64 objects).
std::string encode_rank_mask(uint32_t rank);

// Format a pressure advance value with 4 significant digits and no trailing zeros.
// Same as Orca (general notation): a value outside [1e-4, 1e4) would be written in
// exponential form; real pressure advance values are far inside that range.
std::string format_pressure_advance(double value);

std::string encode_rank_mask(const uint32_t rank)
{
    const uint64_t mask = uint64_t(1) << rank;
    unsigned char bytes[8];
    for (size_t byte_idx = 0; byte_idx < 8; ++byte_idx)
        bytes[byte_idx] = static_cast<unsigned char>((mask >> (8 * byte_idx)) & 0xFF);
    std::string encoded(boost::beast::detail::base64::encoded_size(sizeof(bytes)), '\0');
    encoded.resize(boost::beast::detail::base64::encode(&encoded[0], bytes, sizeof(bytes)));
    return encoded;
}

std::string format_pressure_advance(const double value)
{
    // Default float formatting with precision 4 is the %g behavior: 0.02,
    // 0.0225 and 0.1235 keep only their significant digits.
    std::ostringstream stream;
    stream.imbue(std::locale::classic());
    stream.precision(4);
    stream << value;
    return stream.str();
}

} // namespace

// The rank table is built before the base call so that the base can already ask
// for labels, even though it only writes them while traversing the plan. The
// detector command goes last: everything of the start sequence precedes it.
// The borrowed plan identifies used filaments; only their activated exhaust speeds
// contribute to the owned output and completion cache. Base failures propagate.
std::string BambuGCodeFirmwareSession::begin_print(const Print &print, const ConstPrintingPlan &plan)
{
    m_object_ranks.assign(plan.object_instance_count(), -1);
    int32_t real_object_count = 0;
    for (uint32_t id = 0; id < plan.object_instance_count(); ++id) {
        if (plan.object_instance(id).has_object())
            m_object_ranks[id] = real_object_count++;
    }
    if (size_t(real_object_count) > k_max_labelled_objects)
        m_object_ranks.clear();

    m_group_count = plan.group_count();
    m_ended_group_count = 0;
    std::string output = Marlin2GCodeFirmwareSession::begin_print(print, plan);
    int during_speed = -1;
    m_completion_exhaust_speed = -1;
    if (m_support_air_filtration) {
        for (uint32_t group_idx = 0; group_idx < plan.group_count(); ++group_idx) {
            const ConstPrintingGroup group = plan.group(group_idx);
            for (uint32_t layer_idx = 0; layer_idx < group.layer_group_count(); ++layer_idx) {
                const ConstPrintingLayerGroup layer = group.layer_group(layer_idx);
                for (uint32_t tool_idx = 0; tool_idx < layer.tool_group_count(); ++tool_idx) {
                    const ConstPrintingToolGroup tool = layer.tool_group(tool_idx);
                    const extruder_index_t filament = tool.extruder_id();
                    if (tool.extrusion_count() > 0 && filament >= 0 && size_t(filament) < m_activate_air_filtration.size() &&
                        m_activate_air_filtration[filament]) {
                        during_speed = std::max(during_speed, m_during_print_exhaust_fan_speeds[filament]);
                        m_completion_exhaust_speed = std::max(m_completion_exhaust_speed,
                            m_complete_print_exhaust_fan_speeds[filament]);
                    }
                }
            }
        }
    }
    if (during_speed >= 0)
        output += "M106 P3 S" + std::to_string(255 * during_speed / 100) + "\n";
    return output + "M981 S1 P20000 ;open spaghetti detector\n";
}

// Reset only the group-local layer index; borrowed group processing and failures
// stay with the base, whose owned approach text is returned unchanged.
std::string BambuGCodeFirmwareSession::begin_group(const ConstPrintingGroup &group)
{
    m_group_layer = -1;
    m_second_layer_pending = false;
    return Marlin2GCodeFirmwareSession::begin_group(group);
}

// Count this borrowed layer without using the previous layer's filament. The
// selected tool group resolves the fan; inherited text and failures are preserved.
// On the group's second layer it also arms m_second_layer_pending, which the layer's
// first tool group consumes in write_second_layer_commands().
std::string BambuGCodeFirmwareSession::begin_layer(const ConstPrintingLayerGroup &layer)
{
    ++m_group_layer;
    m_platter_fan_pending = true;
    m_second_layer_pending = m_group_layer == 1;
    return Marlin2GCodeFirmwareSession::begin_layer(layer);
}

// When power loss recovery is enabled it is switched off at the end of every group
// (after the base closed any object block). Only the end of the last group precedes end_gcode, so the detector
// is closed there after stopping the platter fan (when enabled and not known to be stopped) and updating its cache. Earlier
// groups emit no fan stop or detector close; owned base text and failures are preserved.
std::string BambuGCodeFirmwareSession::end_group()
{
    std::string output = Marlin2GCodeFirmwareSession::end_group();
    if (m_power_loss_recovery == 1)
        output += "M1003 S0\n";
    if (++m_ended_group_count == m_group_count) {
        if (m_platter_fan && m_last_platter_fan_speed != 0) {
            output += "M106 P2 S0\n";
            m_last_platter_fan_speed = 0;
        }
        output += "M981 S0 P20000 ; close spaghetti detector\n";
    }
    return output;
}

// Append the cached used-filament maximum after the base closes the print. A
// negative cache means no activated exhaust fan; base failures propagate.
std::string BambuGCodeFirmwareSession::end_print()
{
    std::string output = Marlin2GCodeFirmwareSession::end_print();
    if (m_completion_exhaust_speed >= 0)
        output += "M106 P3 S" + std::to_string(255 * m_completion_exhaust_speed / 100) + "\n";
    return output;
}

// Bambu declares nothing before the objects: the label is carried by M624 itself.
std::string BambuGCodeFirmwareSession::encode_object_definitions(const std::vector<ObjectLabel> &) const
{
    return {};
}

// Open the cancellable block of an object. Returns nothing for an object without
// rank (synthetic, or plan with more than 64 real objects).
std::string BambuGCodeFirmwareSession::encode_object_start(const ObjectLabel &object) const
{
    const int32_t rank = object_rank(object.id);
    if (rank < 0)
        return {};
    return "; start printing object, unique label id: " + std::to_string(object.id) +
        "\nM624 " + encode_rank_mask(uint32_t(rank)) + "\n";
}

// Copy retraction and fan settings from the borrowed Config into per-tool owned
// vectors, resetting pending/cache state. Missing optional settings use their
// documented defaults; the fan-plugin layer threshold falls back to zero.
// Base setup failures propagate; no Config reference survives this call.
void BambuGCodeFirmwareSession::setup_firmware(const Config &config)
{
    Marlin2GCodeFirmwareSession::setup_firmware(config);
    m_platter_fan = config.bool_or_default("platter_fan", false);
    m_support_air_filtration = config.bool_or_default("support_air_filtration", false);
    m_last_platter_fan_speed = -1;
    m_platter_fan_pending = false;
    // printer_custom_variables is deliberately not declared as a used key (it would
    // grey the field for other firmwares); absent variables write nothing.
    const std::map<std::string, std::string> variables =
        Slic3r::parse_custom_variable_lines(config.string_or_default("printer_custom_variables", ""));
    const auto scan = variables.find("scan_first_layer");
    std::string scan_value = scan != variables.end() ? scan->second : std::string();
    std::transform(scan_value.begin(), scan_value.end(), scan_value.begin(),
                   [](const unsigned char c) { return char(std::tolower(c)); });
    m_scan_first_layer = scan_value == "1" || scan_value == "true";
    m_power_loss_recovery = -1;
    m_second_layer_pending = false;
    const auto recovery = variables.find("enable_power_loss_recovery");
    if (recovery != variables.end() && recovery->second == "enable")
        m_power_loss_recovery = 1;
    else if (recovery != variables.end() && recovery->second == "disable")
        m_power_loss_recovery = 0;
    m_retract_lengths.clear();
    m_platter_fan_speeds.clear();
    m_disable_fan_first_layers.clear();
    m_activate_air_filtration.clear();
    m_during_print_exhaust_fan_speeds.clear();
    m_complete_print_exhaust_fan_speeds.clear();
    for (uint32_t tool_idx = 0; tool_idx < extruders().size(); ++tool_idx) {
        m_retract_lengths.push_back(config.vector_float_or_default("retract_length", tool_idx, 0.0));
        m_platter_fan_speeds.push_back(config.vector_int_or_default("platter_fan_speed", tool_idx, 0));
        m_disable_fan_first_layers.push_back(config.vector_int_or_default("disable_fan_first_layers", tool_idx, 0));
        m_activate_air_filtration.push_back(config.vector_bool_or_default("activate_air_filtration", tool_idx, false));
        m_during_print_exhaust_fan_speeds.push_back(config.vector_int_or_default("during_print_exhaust_fan_speed", tool_idx, 60));
        m_complete_print_exhaust_fan_speeds.push_back(config.vector_int_or_default("complete_print_exhaust_fan_speed", tool_idx, 80));
    }
}

// Borrow the tool pair for Marlin's shared part fan, then cache the selected id
// and defer P2. Tool ids index the configured vectors; no tool ownership moves.
void BambuGCodeFirmwareSession::synchronize_selected_extruder_state(
    const DefaultExtruder *previous, DefaultExtruder &selected)
{
    Marlin2GCodeFirmwareSession::synchronize_selected_extruder_state(previous, selected);
    m_selected_filament = selected.id();
    m_platter_fan_pending = true;
}

// Bambu reads P as a fan number: P2 and P3 are not tool fans, and P1 or no P is
// the part fan of the tool selected where the script runs. Without a selected tool
// P1 keeps the neutral reading (tool 1), and a line without P addresses no tool.
std::optional<int32_t> BambuGCodeFirmwareSession::script_fan_tool(
    std::optional<int32_t> fan_index, std::optional<extruder_index_t> selected_tool) const
{
    if (fan_index && (*fan_index == 2 || *fan_index == 3))
        return std::nullopt;
    if (selected_tool && (!fan_index || *fan_index == 1))
        return int32_t(*selected_tool);
    return Marlin2GCodeFirmwareSession::script_fan_tool(fan_index, selected_tool);
}

// The speed written last (0..255, as write_platter_fan() computes it) is handed to
// the Reader as a percentage when it is known. A disabled platter fan is not
// tracked: nothing is exported.
void BambuGCodeFirmwareSession::append_reader_private_state(
    std::vector<raw_gcode_reader_event_item> &items) const
{
    if (m_platter_fan && m_last_platter_fan_speed >= 0)
        items.push_back({Reader::BAMBU_ITEM_PLATTER_FAN_SPEED, 0,
                         m_last_platter_fan_speed * 100.0 / 255.0});
}

// Compare the Reader's final platter speed with the speed written last. The item
// only says which final value the Reader knows; whether a command of the script set
// it is not deduced from its presence or absence. Outcomes, each (except the first)
// marking the platter pending so the next opportunity compares the real fan with
// the wanted speed:
//  - the item equals the speed written last: no effect;
//  - the item differs: the cache holds the new speed, except inside an object
//    block: the script may be skipped with a cancelled object, so the real speed
//    is unknown and the wanted speed is written again outside any block;
//  - the item is absent although a speed was exported: no final value is known
//    (the Reader lost it, for instance after an unknown command), so the real
//    speed is unknown.
// An item absent when nothing was exported leaves the cache unknown as it was. The
// exhaust fan has no state to follow, and a disabled platter fan is not tracked.
void BambuGCodeFirmwareSession::synchronize_reader_private_state(const GCodeStateTracker &state)
{
    Marlin2GCodeFirmwareSession::synchronize_reader_private_state(state);
    if (!m_platter_fan)
        return;
    const std::optional<double> platter = state.value(Reader::BAMBU_ITEM_PLATTER_FAN_SPEED);
    if (!platter) {
        if (m_last_platter_fan_speed >= 0) {
            m_last_platter_fan_speed = -1;
            m_platter_fan_pending = true;
        }
        return;
    }
    const int speed = int(std::lround(*platter * 255.0 / 100.0));
    if (speed == m_last_platter_fan_speed)
        return;
    m_last_platter_fan_speed = object_block_open() ? -1 : speed;
    m_platter_fan_pending = true;
}

// The borrowed group may defer its actual TOOLCHANGE to tree events. Emit the
// currently selected speed after base selection; later selection marks it pending
// again. The first tool group of a group's second layer also writes the M1003/scan
// commands (write_second_layer_commands(), before the fan). Owned output closes a
// preceding object if needed; base failures propagate.
std::string BambuGCodeFirmwareSession::begin_tool_group(const ConstPrintingToolGroup &group)
{
    std::string output = Marlin2GCodeFirmwareSession::begin_tool_group(group);
    output += write_second_layer_commands();
    output += write_platter_fan();
    return output;
}

// A geometric leaf flushes changes from ordered tool events when no object opens
// in between. Empty events retain the pending change. Inside an open object block
// nothing is written: closing the block here would leave the rest of its object
// outside it, so the speed stays pending until the next opening or tool group,
// which are outside any block. Returned text is owned.
std::string BambuGCodeFirmwareSession::visit_extrusion_leaf(const ExtrusionEntity &entity)
{
    return entity.segment_count() > 0 && !object_block_open() ? write_platter_fan() : std::string();
}

// The base calls this after closing the previous block and before opening the
// next one, so the owned fan text cannot be skipped by cancelling either object.
std::string BambuGCodeFirmwareSession::write_before_object_start()
{
    return write_platter_fan();
}

// Runs once per group, on the first tool group of its second layer, so after the
// layer's own script and before its first move. The commands are written outside any
// object block (a cancelled object would skip them otherwise): an open block is closed
// first. `M1003` comes before the scan. The scan goes through write_with_retraction() with
// the selected tool's retract_length: it is framed by a retraction pair only when the
// tool is not already retracted (otherwise it gets at most the complement). Nothing is written
// when printer_custom_variables ask for neither. Base failures propagate.
std::string BambuGCodeFirmwareSession::write_second_layer_commands()
{
    if (!m_second_layer_pending)
        return {};
    m_second_layer_pending = false;
    if (m_power_loss_recovery < 0 && !m_scan_first_layer)
        return {};
    std::string output = transition_object(uint32_t(-1));
    if (m_power_loss_recovery >= 0)
        output += "M1003 S" + std::to_string(m_power_loss_recovery) + "\n";
    if (m_scan_first_layer) {
        const double retract_length = m_selected_filament < m_retract_lengths.size() ?
            std::max(0.0, m_retract_lengths[m_selected_filament]) : 0.0;
        output += write_with_retraction("M976 S1 P1 ; scan model before printing 2nd layer\nM400 P100\n",
                                        retract_length);
    }
    return output;
}

// Resolve the selected filament's percent to 0..255 for valid settings. Cache
// only changed speeds, closing any open object before the owned command text.
// Object transition failures propagate; no borrowed config or entity is retained.
std::string BambuGCodeFirmwareSession::write_platter_fan()
{
    if (!m_platter_fan || !m_platter_fan_pending)
        return {};
    const int pct = m_group_layer < m_disable_fan_first_layers[m_selected_filament] ?
        0 : m_platter_fan_speeds[m_selected_filament];
    const int speed = 255 * pct / 100;
    if (speed == m_last_platter_fan_speed) {
        m_platter_fan_pending = false;
        return {};
    }
    std::string output = transition_object(uint32_t(-1));
    output += "M106 P2 S" + std::to_string(speed) + "\n";
    m_platter_fan_pending = false;
    m_last_platter_fan_speed = speed;
    return output;
}

// Every labelled block opens with the tool's retract_length (see the file header).
double BambuGCodeFirmwareSession::object_block_retract_length(const ObjectLabel &,
                                                              const extruder_index_t tool_id) const
{
    return tool_id < m_retract_lengths.size() ? std::max(0.0, m_retract_lengths[tool_id]) : 0.0;
}

// Close the block opened by encode_object_start(); nothing for an unranked object.
std::string BambuGCodeFirmwareSession::encode_object_end(const ObjectLabel &object) const
{
    if (object_rank(object.id) < 0)
        return {};
    return "; stop printing object, unique label id: " + std::to_string(object.id) + "\nM625\n";
}

// A closed block gets the E zero exactly when it was labelled (the base session
// restricts the reset to absolute E).
bool BambuGCodeFirmwareSession::resets_extrusion_axis_after_object_end(const ObjectLabel &object) const
{
    return object_rank(object.id) >= 0;
}

// Bambu form of the pressure advance command. The tool index is ignored: the
// printer has one pressure advance setting for the active extruder.
std::string BambuGCodeFirmwareSession::encode_pressure_advance(extruder_index_t,
                                                               const double pressure_advance) const
{
    return "M900 K" + format_pressure_advance(pressure_advance) +
        " L1000 M10 ; Override pressure advance value\n";
}

// Rank of a catalog id in the bitmask, -1 for no label.
int32_t BambuGCodeFirmwareSession::object_rank(const uint32_t catalog_id) const
{
    return catalog_id < m_object_ranks.size() ? m_object_ranks[catalog_id] : -1;
}

}}} // namespace slic3r_api::GCodeGeneration::Firmware
