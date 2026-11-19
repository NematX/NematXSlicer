///|/ Released under the GNU Affero General Public License, version 3 or later.

#include "PressureClassification.hpp"

#include <array>
#include <charconv>
#include <cmath>
#include <string>
#include <string_view>
#include <utility>

#include "PressureNominal.hpp"
#include "PressureSource.hpp"
#include "../../nematx_firmware/src/NematXSyntax.hpp"
#include "libslic3r/Api/plugin/c/slic3r_extrusions.h"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeLineParser.hpp"

// Provides line decisions to a caller that owns the export's pressure cycle.
// A single ordered event traversal tracks only role, wipe scope, volume distrust
// and the last accepted print modes. NominalMotion supplies pre-motion proofs and
// source doubles; the SDK lexer and NematX word splitter recognize only controls.
// Barriers are accumulated per row before the next accepted cycle is recorded.
// Relative E requires its exact delta, not an external absolute origin. Only its
// adjacent E-only invalidation can be excused after the print is accepted.
// Source geometry already removed by the host is not reconstructed: such moves
// are E-only here. Nonzero script geometry remains a candidate before quantizing.
// All working state is local; borrowed inputs remain unchanged even on failure.
//
// Call tree:
//   classify_fragment: join motions and follow events once in physical row order.
//     read_controls: SDK lexical stops, wipe comments and numeric wait/M200 words.
//     accept_motion: select candidates, then require active print proofs.
//   ClassificationError: own row/tool/cause and build a local explanation.
//     cause_text: describe the missing proof or unsupported input.

namespace nematx_pressure {
// Accumulates barriers and accepted motion indices without modifying inputs.
// Invalid sizes, inconsistent joins and unproven active printing throw, so no
// partially classified fragment is observable by the caller.
ClassificationFragment classify_fragment(const SourceFragment &source,
    const NominalFragment &nominal, const std::vector<MoveHint> &hints,
    const ClassificationContext &context, ClassificationState prior);

namespace {
using Item = raw_gcode_reader_event_item;
using Parser = slic3r_api::GCodeGeneration::GCodeLineParser;
constexpr std::uint32_t X = RAW_GCODE_READER_ITEM_X, Z = RAW_GCODE_READER_ITEM_Z,
    E = RAW_GCODE_READER_ITEM_E, F = RAW_GCODE_READER_ITEM_FEEDRATE,
    XYZ = RAW_GCODE_READER_ITEM_XYZ_ABSOLUTE, EA = RAW_GCODE_READER_ITEM_E_ABSOLUTE,
    MM = RAW_GCODE_READER_ITEM_UNITS_MM, TOOL = RAW_GCODE_READER_ITEM_ACTIVE_TOOL,
    VOL = RAW_GCODE_READER_ITEM_VOLUMETRIC, ROLE = RAW_GCODE_READER_ITEM_EXTRUSION_ROLE,
    LN = RAW_GCODE_READER_ITEM_LINE_NUMBER;

// Follows the Reader's lexical order, changing only wipe/volume latches and the
// row's pause barrier. Command comments never act as wipe delimiters.
void read_controls(std::string_view raw, ClassificationState &state, bool &close);
// Selects physical nonprints before requiring tool/configuration and print proofs.
// Returns true only for eligible prints; failures identify this local row.
bool accept_motion(const NominalMotion &motion, bool arc, MoveHint hint,
    bool wipe, bool xyz_intent, bool stationary, bool e_present, std::size_t row,
    const ClassificationContext &context, const ClassificationState &state);
// Supplies fixed English descriptions for the owned local diagnostic.
const char *cause_text(ClassificationCause cause);

void read_controls(std::string_view raw, ClassificationState &state, bool &close)
{
    if (!raw.empty() && raw.back() == '\n')
        raw.remove_suffix(1);
    raw = nematx_firmware::strip_line_number(raw);
    Parser parser;
    Parser::GCodeLine line;
    parser.parse_line(raw, [&](Parser &, const Parser::GCodeLine &parsed) { line = parsed; });
    if (line.command().empty()) {
        std::string_view comment = line.comment();
        while (!comment.empty() && (comment.front() == ' ' || comment.front() == '\t'))
            comment.remove_prefix(1);
        while (!comment.empty() && (comment.back() == ' ' || comment.back() == '\t'))
            comment.remove_suffix(1);
        if (comment == "WIPE_START") state.wipe_scope = true;
        if (comment == "WIPE_END") state.wipe_scope = false;
    }
    // Syntax whitespace includes VT/FF, unlike the SDK's literal command view.
    const std::string_view text(line.raw());
    const std::vector<std::string_view> words = nematx_firmware::detail::split_words(
        nematx_firmware::detail::trim(text.substr(0, text.find(';'))));
    if (words.empty())
        return;
    const std::string_view command = words.front().substr(0, words.front().find('='));
    if (command.size() < 2 || (command.front() != 'M' && command.front() != 'G'))
        return;
    std::uint32_t number = 0;
    const std::from_chars_result parsed = std::from_chars(command.data() + 1,
        command.data() + command.size(), number);
    if (parsed.ec != std::errc() || parsed.ptr != command.data() + command.size())
        return;
    if (command.front() == 'M') {
        if (number == 200) state.volume_untrusted = true;
        if (number == 109 || number == 129 || number == 190) close = true;
    } else if (number == 4) {
        close = true;
    }
}

bool accept_motion(const NominalMotion &motion, bool arc, MoveHint hint,
    bool wipe, bool xyz_intent, bool stationary, bool e_present, std::size_t row,
    const ClassificationContext &context, const ClassificationState &state)
{
    if (!context.in_layer)
        return false;
    if (!arc) {
        if (wipe || hint == MoveHint::Travel || hint == MoveHint::Wipe ||
            (motion.e_delta && *motion.e_delta < 0) || !xyz_intent || stationary)
            return false;
        if (hint != MoveHint::Extrusion &&
            !(motion.e_delta && *motion.e_delta > 0) &&
            !(e_present && !motion.e_delta))
            return false;
    }
    if (!motion.active_tool || *motion.active_tool >= context.enabled_tools.size())
        throw ClassificationError(row, motion.active_tool, ClassificationCause::UnknownTool);
    if (!context.enabled_tools[*motion.active_tool])
        return false;
    if (arc)
        throw ClassificationError(row, motion.active_tool, ClassificationCause::UnsupportedArc);
    if (context.config_volumetric || state.volume_untrusted)
        throw ClassificationError(row, motion.active_tool, ClassificationCause::UnsupportedVolume);
    if (!motion.e_target)
        throw ClassificationError(row, motion.active_tool, ClassificationCause::UnknownExtrusion);
    if (motion.e_target != motion.active_tool)
        throw std::logic_error("Printing E target differs from proven active tool.");
    if (!motion.xyz_absolute || !motion.e_absolute || !motion.units_mm)
        throw ClassificationError(row, motion.active_tool, ClassificationCause::UnknownModes);
    if (!*motion.xyz_absolute || !*motion.units_mm)
        throw ClassificationError(row, motion.active_tool, ClassificationCause::UnsupportedModes);
    for (std::size_t axis = 0; axis < 3; ++axis)
        if (!motion.start[axis] || !motion.end[axis] ||
            !std::isfinite(*motion.start[axis]) || !std::isfinite(*motion.end[axis]))
            throw ClassificationError(row, motion.active_tool, ClassificationCause::UnknownCoordinates);
    if (!motion.e_delta || !std::isfinite(*motion.e_delta) || *motion.e_delta < 0 ||
        (*motion.e_absolute && (!motion.e_start || !motion.e_end ||
            !std::isfinite(*motion.e_start) || !std::isfinite(*motion.e_end))))
        throw ClassificationError(row, motion.active_tool, ClassificationCause::UnknownExtrusion);
    if (!motion.feedrate || !std::isfinite(*motion.feedrate) || *motion.feedrate <= 0)
        throw ClassificationError(row, motion.active_tool, ClassificationCause::UnknownFeedrate);
    return true;
}

const char *cause_text(ClassificationCause cause)
{
    switch (cause) {
    case ClassificationCause::UnknownTool: return "active tool is unknown or outside enabled_tools";
    case ClassificationCause::UnsupportedArc: return "active arc is not supported";
    case ClassificationCause::UnsupportedVolume: return "volumetric extrusion is not treated";
    case ClassificationCause::UnknownModes: return "required pre-motion modes are unknown";
    case ClassificationCause::UnsupportedModes: return "printing requires absolute XYZ and millimetres";
    case ClassificationCause::UnknownCoordinates: return "finite starting and ending XYZ are not proven";
    case ClassificationCause::UnknownExtrusion: return "required finite extrusion values are not proven";
    case ClassificationCause::UnknownFeedrate: return "positive finite feedrate is not proven";
    }
    return "unrecognized classification cause";
}
} // namespace

ClassificationError::ClassificationError(std::size_t line,
    std::optional<std::uint32_t> active_tool, ClassificationCause reason)
    : std::invalid_argument("Pressure classification at local row " + std::to_string(line) +
          (active_tool ? ", tool " + std::to_string(*active_tool) : ", unknown tool") +
          ": " + cause_text(reason)), line_index(line), tool(active_tool), cause(reason)
{
}

ClassificationFragment classify_fragment(const SourceFragment &source,
    const NominalFragment &nominal, const std::vector<MoveHint> &hints,
    const ClassificationContext &context, ClassificationState prior)
{
    if (!hints.empty() && hints.size() != source.lines.size())
        throw std::invalid_argument("Move hints must be empty or match physical source rows.");
    ClassificationFragment result;
    result.lines.resize(source.lines.size());
    std::optional<double> role = source.reader.initial.value(ROLE);
    if (source.reader.initial.value(VOL) == 1.0)
        prior.volume_untrusted = true;
    if (!context.in_layer)
        prior.cycle.reset();
    std::size_t cursor = 0;
    for (std::size_t row = 0; row < source.lines.size(); ++row) {
        const SourceLine &line = source.lines[row];
        LineDecision &decision = result.lines[row];
        read_controls(std::string_view(source.text).substr(line.offset, line.length),
                      prior, decision.close_before);
        const MoveHint hint = hints.empty() ? MoveHint::None : hints[row];
        std::size_t motions = 0;
        bool accepted = false, deferred_e = false, physical_noop = false;
        std::optional<double> net_xyz, net_e, net_mm, net_tool;
        for (std::size_t index = line.first_event; index < line.first_event + line.event_count; ++index) {
            const ReaderEvent &event = source.reader.events[index];
            const bool motion_event = event.kind == RAW_GCODE_READER_EVENT_LINEAR_MOVE ||
                                      event.kind == RAW_GCODE_READER_EVENT_ARC_MOVE;
            if (motion_event) {
                if (++motions > 1 && hint != MoveHint::None)
                    throw std::invalid_argument("A move hint cannot describe several Reader motions.");
                if (motions > 1 || cursor >= nominal.motions.size() ||
                    nominal.motions[cursor].event_index != index)
                    throw std::logic_error("Source row and nominal motion do not join.");
                const NominalMotion &motion = nominal.motions[cursor];
                bool xyz_intent = false, e_present = bool(motion.tokens[3]);
                std::array<bool, 3> intended{};
                for (std::size_t axis = 0; axis < 3; ++axis)
                    intended[axis] = bool(motion.tokens[axis]);
                // An invalid word may have no selected token, but still requested
                // motion. Only the immediately adjacent invalidation supplies it.
                const ReaderEvent *adjacent = index > line.first_event &&
                    source.reader.events[index - 1].kind == RAW_GCODE_READER_EVENT_INVALIDATE ?
                    &source.reader.events[index - 1] : nullptr;
                if (adjacent)
                    for (const Item &item : adjacent->items) {
                        if (item.type >= X && item.type <= Z) intended[item.type - X] = true;
                        e_present = e_present || item.type == E;
                    }
                for (const Item &item : event.items)
                    e_present = e_present || item.type == E;
                bool stationary = true;
                for (std::size_t axis = 0; axis < 3; ++axis) {
                    xyz_intent = xyz_intent || intended[axis];
                    if (intended[axis] && (!motion.start[axis] || !motion.end[axis] ||
                        *motion.start[axis] != *motion.end[axis])) stationary = false;
                }
                const bool wipe = prior.wipe_scope ||
                    (role && (std::uint32_t(*role) & RAW_EXTRUSION_ROLE_WIPE) != 0);
                physical_noop = event.kind == RAW_GCODE_READER_EVENT_LINEAR_MOVE &&
                    !e_present && (!xyz_intent || stationary);
                accepted = accept_motion(motion, event.kind == RAW_GCODE_READER_EVENT_ARC_MOVE,
                    hint, wipe, xyz_intent, stationary, e_present, row, context, prior);
                if (accepted) {
                    if (prior.cycle && (motion.active_tool != prior.cycle->tool ||
                        motion.xyz_absolute != prior.cycle->xyz_absolute ||
                        motion.e_absolute != prior.cycle->e_absolute ||
                        motion.units_mm != prior.cycle->units_mm))
                        decision.close_before = true;
                    decision.motion_index = cursor;
                    // This exception needs a real accepted relative print, not
                    // just an E word or a hint which might describe a travel.
                    deferred_e = deferred_e && !(motion.e_absolute == false &&
                        motion.tokens[3] && motion.e_delta && std::isfinite(*motion.e_delta) &&
                        *motion.e_delta >= 0 && motion.active_tool == motion.e_target);
                } else if ((xyz_intent && !stationary) || e_present) {
                    decision.close_before = true;
                }
                ++cursor;
                continue;
            }
            if (event.kind == RAW_GCODE_READER_EVENT_SET_POSITION ||
                event.kind == RAW_GCODE_READER_EVENT_SET_ORIGIN ||
                event.kind == RAW_GCODE_READER_EVENT_WAIT_TEMPERATURE)
                decision.close_before = true;
            bool only_target_e = false, other_invalid = false, invalid_barrier = false;
            for (const Item &item : event.items) {
                if (event.kind == RAW_GCODE_READER_EVENT_STATE_UPDATE) {
                    if (item.type == ROLE) role = item.value;
                    if (item.type == VOL && item.value == 1) prior.volume_untrusted = true;
                    if (item.type == XYZ) net_xyz = item.value;
                    if (item.type == EA) net_e = item.value;
                    if (item.type == MM) net_mm = item.value;
                    if (item.type == TOOL) net_tool = item.value;
                }
                if (event.kind == RAW_GCODE_READER_EVENT_INVALIDATE) {
                    if (item.type == ROLE) role.reset();
                    if ((item.type >= X && item.type <= F) || item.type == XYZ ||
                        item.type == EA || item.type == MM || item.type == TOOL || item.type == VOL)
                        invalid_barrier = true;
                    // Defer only an E-only event at the exact next linear motion.
                    if (item.type == E && cursor < nominal.motions.size() &&
                        nominal.motions[cursor].event_index == index + 1 &&
                        index + 1 < line.first_event + line.event_count &&
                        source.reader.events[index + 1].kind == RAW_GCODE_READER_EVENT_LINEAR_MOVE &&
                        nominal.motions[cursor].e_target == item.target)
                        only_target_e = true;
                    else if (item.type != LN)
                        other_invalid = true;
                }
                if (event.kind == RAW_GCODE_READER_EVENT_DIAGNOSTIC &&
                    item.type == RAW_GCODE_READER_ITEM_DIAGNOSTIC_TYPE &&
                    (item.value == RAW_GCODE_READER_DIAGNOSTIC_UNKNOWN_COMMAND ||
                     item.value == RAW_GCODE_READER_DIAGNOSTIC_UNSUPPORTED_STATE_EFFECT))
                    decision.close_before = true;
                if (event.kind == RAW_GCODE_READER_EVENT_MARKER &&
                    item.type == RAW_GCODE_READER_ITEM_MARKER &&
                    (item.value == RAW_GCODE_READER_MARKER_PAUSE_PRINT ||
                     item.value == RAW_GCODE_READER_MARKER_COLOR_CHANGE))
                    decision.close_before = true;
            }
            if (only_target_e && !other_invalid) {
                deferred_e = true;
            } else if (invalid_barrier) decision.close_before = true;
        }
        if (hint != MoveHint::None && motions != 1)
            throw std::invalid_argument("A nonempty move hint requires exactly one Reader motion on its row.");
        // A NOOP inherits the residual G91 of an earlier mixed move. Only slots
        // updated on this row matter, including updates after its motion event.
        if ((!motions || physical_noop) && prior.cycle &&
            ((net_xyz && (*net_xyz != double(prior.cycle->xyz_absolute))) ||
             (net_e && (*net_e != double(prior.cycle->e_absolute))) ||
             (net_mm && (*net_mm != double(prior.cycle->units_mm))) ||
             (net_tool && (*net_tool != double(prior.cycle->tool)))))
            decision.close_before = true;
        if (deferred_e) decision.close_before = true;
        if (decision.close_before || !context.in_layer) prior.cycle.reset();
        if (accepted) {
            const NominalMotion &motion = nominal.motions[*decision.motion_index];
            prior.cycle = CycleSnapshot{*motion.active_tool, *motion.xyz_absolute,
                                       *motion.e_absolute, *motion.units_mm};
        }
    }
    if (cursor != nominal.motions.size())
        throw std::logic_error("Nominal motions remain after the last source row.");
    result.final = std::move(prior);
    return result;
}
} // namespace nematx_pressure
