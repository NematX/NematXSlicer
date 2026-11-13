///|/ Released under the GNU Affero General Public License, version 3 or later.

#include "PressureRewrite.hpp"

#include <algorithm>
#include <charconv>
#include <cctype>
#include <cmath>
#include <optional>
#include <string_view>
#include <utility>

#include "PressureExtrusion.hpp"
#include "PressureGeometry.hpp"
#include "PressureNominal.hpp"
#include "PressureSource.hpp"
#include "PressureTokens.hpp"
#include "../../nematx_firmware/src/NematXSyntax.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeFormatter.hpp"

// Provides a transactional text boundary for prepared pressure output. Byte and
// word checks precede assembly, because a controller may interpret ignored or
// duplicate parameters differently from the Reader. The existing SDK lexer and
// NematX translator decide the lexical path; selected numeric ranges are checked
// against Nominal's ranges before editing. Geometry supplies quantized XYZ and
// Extrusion supplies quantized E, never recalculated here. Only local strings
// are changed, so exceptions cannot publish a prefix or retain borrowed views.
//
// Call tree:
//   rewrite_print: validate joins, lex words, assemble all groups locally.
//     validate_source: prove the shared Reader/Nominal joins and input state.
//     inspect: reject ambiguous words and reconnect selected numeric ranges.
//     number: use a fresh SDK formatter for one checked number.
//     annotation: encode analytical values with locale-free round-trip numbers.
//       metadata_number: reject nonfinite values and normalize negative zero.
//   rewrite_residue: validate the row, remove words, retain only useful effects.
//     validate_source: prove the shared Reader/Nominal joins and input state.
//     inspect: collect complete removable words after lexical checks.
//     replay_modes: use Syntax actions from the Reader state before the row.

namespace nematx_pressure {
namespace {
using Parser = slic3r_api::GCodeGeneration::GCodeLineParser;
using Formatter = slic3r_api::GCodeGeneration::GCodeFormatter;
using Action = nematx_firmware::ReaderAction;
constexpr std::array<char, 5> letters{{'X', 'Y', 'Z', 'E', 'F'}};

// Local facts from the unchanged source and its actual direct/translated path.
struct Shape
{
    // Command word recognized on that path; copied exactly to constructed rows.
    std::string command;
    // True for an extrusion word consumed as a tool axis, not a generic E word.
    bool axis_e = false;
    // Fragment byte offset immediately after the last numeric XYZ span.
    std::size_t insert_at = 0;
    // Complete XYZ/E words, including whitespace between letter and number.
    std::vector<SourceToken> removable;
};

// Proves the common source joins, modes, coordinates and extrusion values.
// Returns a borrowed row; invalid prepared values throw without changing inputs.
const SourceLine &validate_source(const SourceFragment &source, std::size_t row,
    const NominalMotion &motion, const std::vector<char> &axes);
// Applies only Syntax mode/tool actions to a value copy of the pre-row modes.
// Syntax returns nullopt for a direct SDK linear line, leaving modes unchanged.
nematx_firmware::ReaderModes replay_modes(std::string_view raw,
    const std::vector<char> &axes, nematx_firmware::ReaderModes modes);
// Validates complete source bytes and every consumed word using the actual
// direct/translated path. Throws a contextual error for user source ambiguity,
// or logic_error for a mismatched prepared span. Returns numeric insertion and
// complete removable word ranges; retains no borrowed view.
Shape inspect(std::string_view raw, std::size_t offset, std::size_t row,
    const NominalMotion &motion, const std::vector<char> &axes);
// Formats one finite number after checking the rounded scaled int64 domain.
// A fresh formatter keeps its fixed buffer independent of line/comment length.
std::string number(double value, int digits);
// Encodes one group's analytical annotation, rejecting invalid duration, K or
// nonfinite arithmetic. Precision is independent of the output axis grid.
std::string annotation(const GeometryGroup &group, const ExtrusionResult &e,
                       double k, std::uint32_t tool);
// Returns a shortest round-trip C-locale double; rejects nonfinite metadata.
std::string metadata_number(double value);
} // namespace

// Validates one joined eligible row and assembles only local replacement bytes.
// Invalid inputs throw before any result escapes; borrowed inputs stay unchanged.
std::string rewrite_print(const SourceFragment &source, std::size_t row,
    const NominalMotion &motion, const GeometryResult &geometry,
    const std::array<ExtrusionResult, 3> &extrusion, double pressure_factor_s,
    const std::vector<char> &resolved_axis_letters, const RewriteFormat &format);

// Removes checked complete words locally, then verifies that Syntax preserves
// the original final XYZ/E modes and tool from the actual pre-row Reader state.
// No input is mutated; a lost effect throws a contextual error before return.
std::string rewrite_residue(const SourceFragment &source, std::size_t row,
    const NominalMotion &motion, const std::vector<char> &resolved_axis_letters);

std::string rewrite_print(const SourceFragment &source, std::size_t row,
    const NominalMotion &motion, const GeometryResult &geometry,
    const std::array<ExtrusionResult, 3> &extrusion, double pressure_factor_s,
    const std::vector<char> &resolved_axis_letters, const RewriteFormat &format)
{
    if (row >= source.lines.size() || format.xyz_digits < 0 || format.xyz_digits > 9 ||
        format.e_digits < 0 || format.e_digits > 9 || !std::isfinite(pressure_factor_s) || pressure_factor_s < 0 ||
        geometry.status != GeometryStatus::Representable || geometry.count == 0 || geometry.count > 3)
        throw std::invalid_argument("Invalid pressure rewrite row, precision or spatial groups.");
    const SourceLine &line = validate_source(source, row, motion, resolved_axis_letters);
    std::string_view raw(source.text.data() + line.offset, line.length);
    const Shape shape = inspect(raw, line.offset, row, motion, resolved_axis_letters);
    bool restore_absolute = *motion.xyz_absolute;
    for (std::size_t i = line.first_event; i < line.first_event + line.event_count; ++i)
        if (source.reader.events[i].kind == RAW_GCODE_READER_EVENT_STATE_UPDATE)
            for (const raw_gcode_reader_event_item &item : source.reader.events[i].items)
                if (item.type == RAW_GCODE_READER_ITEM_XYZ_ABSOLUTE)
                    restore_absolute = item.value != 0;
    const bool has_lf = !raw.empty() && raw.back() == '\n';
    const bool crlf = has_lf && raw.size() > 1 && raw[raw.size() - 2] == '\r';
    const std::string separator = crlf ? "\r\n" : "\n";
    if (has_lf)
        raw.remove_suffix(crlf ? 2 : 1);
    std::string output;
    Point previous = geometry.groups[0].start;
    for (std::size_t group_index = 0; group_index < geometry.count; ++group_index) {
        const GeometryGroup &group = geometry.groups[group_index];
        const ExtrusionResult &e = extrusion[group_index];
        if (group.start != previous || group.start == group.end || !std::isfinite(group.distance) ||
            group.distance <= 0 || !std::isfinite(group.duration) || group.duration <= 0 ||
            !std::isfinite(e.position) || !std::isfinite(e.delta))
            throw std::invalid_argument("Invalid aligned spatial/extrusion group.");
        previous = group.end;
        const double e_value = *motion.e_absolute ? e.position : e.delta;
        const bool insert_e = !motion.tokens[3] && e.delta != 0;
        if (insert_e && shape.command != "G0" && shape.command != "G1")
            throw std::invalid_argument("Missing generated E requires exact G0 or G1.");
        std::string text;
        if (group_index == 0) {
            text.assign(raw);
            // Descending bounded edits preserve every unselected original byte.
            std::vector<std::pair<SourceToken, std::string>> edits;
            for (std::size_t axis = 0; axis < 4; ++axis)
                if (motion.tokens[axis])
                    edits.push_back({*motion.tokens[axis], number(axis < 3 ? group.end[axis] : e_value,
                        axis < 3 ? format.xyz_digits : format.e_digits)});
            if (insert_e) {
                std::string insertion = *motion.e_absolute ? " " : " G91 ";
                insertion += resolved_axis_letters[*motion.active_tool];
                insertion += number(e_value, format.e_digits);
                if (!*motion.e_absolute)
                    insertion += " G90";
                edits.push_back({SourceToken{shape.insert_at, 0, 0}, std::move(insertion)});
            }
            std::sort(edits.begin(), edits.end(), [](const auto &a, const auto &b) {
                return a.first.offset > b.first.offset;
            });
            for (const auto &edit : edits)
                text.replace(edit.first.offset - line.offset, edit.first.length, edit.second);
        } else {
            text = shape.command;
            const bool tool_axis = shape.axis_e || insert_e;
            if (!restore_absolute)
                text += " G90";
            for (std::size_t axis = 0; axis < 3; ++axis)
                if (motion.tokens[axis]) {
                    text += ' ';
                    text += letters[axis];
                    text += number(group.end[axis], format.xyz_digits);
                }
            if (motion.tokens[3] || insert_e) {
                if (tool_axis && !*motion.e_absolute)
                    text += " G91";
                text += ' ';
                text += motion.tokens[3] ? (shape.axis_e ? motion.tokens[3]->letter : 'E') :
                    resolved_axis_letters[*motion.active_tool];
                text += number(e_value, format.e_digits);
            }
            // Generic E does not use inline XYZ modes to choose its E mode.
            const bool current_absolute = !(tool_axis && !*motion.e_absolute &&
                                            (motion.tokens[3] || insert_e));
            if (current_absolute != restore_absolute)
                text += restore_absolute ? " G90" : " G91";
        }
        for (std::size_t axis = 0; axis < 3; ++axis)
            if (!motion.tokens[axis] &&
                (*motion.start[axis] != *motion.end[axis] || group.start[axis] != group.end[axis]))
                throw std::invalid_argument("Absent source XYZ axis changed in prepared geometry.");
        if (format.annotate)
            text += annotation(group, e, pressure_factor_s, *motion.active_tool);
        output += text;
        if (has_lf || group_index + 1 < geometry.count)
            output += separator;
    }
    return output;
}

std::string rewrite_residue(const SourceFragment &source, std::size_t row,
    const NominalMotion &motion, const std::vector<char> &resolved_axis_letters)
{
    const SourceLine &line = validate_source(source, row, motion, resolved_axis_letters);
    const std::string_view raw(source.text.data() + line.offset, line.length);
    Shape shape = inspect(raw, line.offset, row, motion, resolved_axis_letters);
    slic3r_api::GCodeStateTracker before = source.reader.initial;
    for (std::size_t i = 0; i < line.first_event; ++i) {
        const ReaderEvent &event = source.reader.events[i];
        before.apply({event.kind, event.items.data(), std::uint32_t(event.items.size())});
    }
    const nematx_firmware::ReaderModes initial{before.xyz_is_absolute(), before.e_is_absolute(), before.active_tool()};
    const nematx_firmware::ReaderModes original = replay_modes(raw, resolved_axis_letters, initial);
    std::string result(raw);
    std::sort(shape.removable.begin(), shape.removable.end(), [](const SourceToken &a, const SourceToken &b) {
        return a.offset > b.offset;
    });
    for (const SourceToken &word : shape.removable)
        result.erase(word.offset - line.offset, word.length);
    // A bare command and its number have no useful effect once all motion words
    // are gone. Preserve them only for F or an actual surviving modal change.
    const nematx_firmware::ReaderModes residue = replay_modes(result, resolved_axis_letters, initial);
    if (original.xyz_absolute != residue.xyz_absolute || original.e_absolute != residue.e_absolute ||
        original.active_tool != residue.active_tool)
        throw RewriteError(row, *motion.active_tool, RewriteCause::LostModalEffect);
    if (!motion.tokens[4] && residue.xyz_absolute == initial.xyz_absolute &&
        residue.e_absolute == initial.e_absolute && residue.active_tool == initial.active_tool) {
        const std::size_t comment = raw.find(';');
        result = comment == std::string_view::npos ? std::string() : std::string(raw.substr(comment));
    }
    return result;
}

RewriteError::RewriteError(std::size_t row, std::uint32_t active_tool, RewriteCause reason)
    : std::invalid_argument("Pressure rewrite local row " + std::to_string(row) +
        ", tool " + std::to_string(active_tool) + ": " +
        (reason == RewriteCause::InvalidBytes ? "NUL or nonterminal CR byte." :
         reason == RewriteCause::DuplicateParameter ? "duplicate effective XYZ/extrusion parameter." :
         reason == RewriteCause::ConflictingXyzModes ? "contradictory modes between XYZ coordinates." :
         reason == RewriteCause::LostModalEffect ? "removing XYZ/extrusion words loses a mode or tool effect." :
         "unsupported or unproven source word.")),
      line_index(row), tool(active_tool), cause(reason)
{}

namespace {
const SourceLine &validate_source(const SourceFragment &source, std::size_t row,
    const NominalMotion &motion, const std::vector<char> &axes)
{
    if (row >= source.lines.size())
        throw std::invalid_argument("Invalid pressure rewrite row.");
    const SourceLine &line = source.lines[row];
    if (line.offset > source.text.size() || line.length > source.text.size() - line.offset ||
        line.first_event > source.reader.events.size() ||
        line.event_count > source.reader.events.size() - line.first_event ||
        motion.event_index < line.first_event || motion.event_index - line.first_event >= line.event_count)
        throw std::invalid_argument("Pressure rewrite motion is not in the source row.");
    const ReaderEvent &event = source.reader.events[motion.event_index];
    bool event_row = false;
    for (const raw_gcode_reader_event_item &item : event.items)
        if (item.type == RAW_GCODE_READER_ITEM_LINE_NUMBER && item.value == double(row))
            event_row = true;
    if (event.kind != RAW_GCODE_READER_EVENT_LINEAR_MOVE || !event_row ||
        motion.xyz_absolute != true || motion.units_mm != true || !motion.e_absolute ||
        !motion.active_tool || motion.e_target != motion.active_tool || *motion.active_tool >= axes.size())
        throw std::invalid_argument("Pressure rewrite requires a proven linear mm print and active tool.");
    if (!motion.e_delta || !std::isfinite(*motion.e_delta) || *motion.e_delta < 0 ||
        (*motion.e_absolute && (!motion.e_start || !motion.e_end ||
            !std::isfinite(*motion.e_start) || !std::isfinite(*motion.e_end))) ||
        !motion.feedrate || !std::isfinite(*motion.feedrate) || *motion.feedrate <= 0)
        throw std::invalid_argument("Pressure rewrite lacks nominal extrusion proof.");
    for (std::size_t axis = 0; axis < 3; ++axis)
        if (!motion.start[axis] || !motion.end[axis] || !std::isfinite(*motion.start[axis]) ||
            !std::isfinite(*motion.end[axis]))
            throw std::invalid_argument("Pressure rewrite lacks nominal XYZ proof.");
    return line;
}

nematx_firmware::ReaderModes replay_modes(std::string_view raw,
    const std::vector<char> &axes, nematx_firmware::ReaderModes modes)
{
    if (!raw.empty() && raw.back() == '\n') {
        raw.remove_suffix(1);
        if (!raw.empty() && raw.back() == '\r')
            raw.remove_suffix(1);
    }
    const std::optional<std::vector<Action>> actions =
        nematx_firmware::translate_line(nematx_firmware::strip_line_number(raw), axes, modes);
    if (actions)
        for (const Action &action : *actions) {
            if (action.kind == Action::Kind::XyzMode)
                modes.xyz_absolute = action.absolute;
            else if (action.kind == Action::Kind::ExtrusionMode)
                modes.e_absolute = action.absolute;
            else if (action.kind == Action::Kind::SelectTool)
                modes.active_tool = action.tool;
        }
    return modes;
}

Shape inspect(std::string_view raw, std::size_t offset, std::size_t row,
    const NominalMotion &motion, const std::vector<char> &axes)
{
    const std::uint32_t tool = *motion.active_tool;
    for (std::size_t i = 0; i < raw.size(); ++i)
        if (raw[i] == 0 || (raw[i] == '\r' && !(i + 2 == raw.size() && raw[i + 1] == '\n')))
            throw RewriteError(row, tool, RewriteCause::InvalidBytes);
    if (!raw.empty() && raw.back() == '\n') {
        raw.remove_suffix(1);
        if (!raw.empty() && raw.back() == '\r')
            raw.remove_suffix(1);
    }
    const std::string_view stripped = nematx_firmware::strip_line_number(raw);
    const std::size_t base = offset + raw.size() - stripped.size();
    Parser parser;
    Parser::GCodeLine original;
    parser.parse_line(stripped, [&](Parser &, const Parser::GCodeLine &line) { original = line; });
    nematx_firmware::ReaderModes modes;
    modes.xyz_absolute = motion.xyz_absolute;
    modes.e_absolute = motion.e_absolute;
    modes.active_tool = tool;
    const std::optional<std::vector<Action>> actions =
        nematx_firmware::translate_line(original.raw(), axes, modes);
    if (actions)
        for (const Action &action : *actions)
            if (action.kind != Action::Kind::Standard && action.kind != Action::Kind::XyzMode &&
                action.kind != Action::Kind::ExtrusionMode && action.kind != Action::Kind::SelectTool)
                throw RewriteError(row, tool, RewriteCause::UnsupportedWord);
    const std::string_view code = stripped.substr(0, stripped.find(';'));
    const std::vector<std::string_view> words = nematx_firmware::detail::split_words(code);
    if (actions && words.empty())
        throw std::invalid_argument("Pressure rewrite translated source has no command.");
    Shape result{std::string(actions ? words.front() : original.command()), false, 0};
    if (result.command != "G0" && result.command != "G1" && result.command != "G00" && result.command != "G01")
        throw std::invalid_argument("Pressure rewrite requires a linear source command.");
    std::array<std::optional<SourceToken>, 5> selected;
    bool xyz_mode = true;
    std::optional<bool> first_xyz_mode;
    std::size_t cursor = std::size_t(original.command().data() - original.raw().data()) + original.command().size();
    std::size_t ordinal = 1;
    while (actions ? ordinal < words.size() : cursor < code.size()) {
        std::string_view word;
        if (actions) {
            word = words[ordinal++];
        } else {
            cursor = code.find_first_not_of(" \t", cursor);
            if (cursor == std::string_view::npos)
                break;
            const std::size_t end = code.find_first_of(" \t", cursor);
            word = code.substr(cursor, end == std::string_view::npos ? code.size() - cursor : end - cursor);
        }
        if (actions && (word == "G90" || word == "G91")) {
            xyz_mode = word == "G90";
            continue;
        }
        const char literal = word.front();
        const char letter = actions ? char(std::toupper(static_cast<unsigned char>(literal))) : literal;
        const std::optional<std::size_t> axis_tool = actions ? nematx_firmware::tool_of_axis(axes, letter) : std::nullopt;
        const char effective = axis_tool ? 'E' : letter;
        const std::string_view::size_type axis = std::string_view("XYZEF").find(effective);
        if (axis == std::string_view::npos || (axis_tool && *axis_tool != tool))
            throw RewriteError(row, tool, RewriteCause::UnsupportedWord);
        // Direct SDK parameters can own whitespace between letter and number.
        // Parsing a suffix keeps that grammar and consumes the standalone number.
        std::string candidate = "G1 ";
        candidate += effective;
        candidate += actions ? std::string(word.substr(1)) : std::string(code.substr(cursor + 1));
        Parser::GCodeLine numeric_line;
        parser.parse_line(candidate, [&](Parser &, const Parser::GCodeLine &line) { numeric_line = line; });
        std::optional<NumericToken> numeric;
        try { numeric = read_parameter(numeric_line, effective); }
        catch (const std::invalid_argument &) { throw RewriteError(row, tool, RewriteCause::UnsupportedWord); }
        if (!numeric || (actions && numeric->length + 1 != word.size()))
            throw RewriteError(row, tool, RewriteCause::UnsupportedWord);
        const std::size_t local_letter = std::size_t(word.data() - stripped.data());
        const SourceToken span{base + local_letter + numeric->offset - 3, numeric->length, literal};
        if (axis < 4)
            result.removable.push_back({base + local_letter,
                span.offset + span.length - base - local_letter, literal});
        if (axis < 4 && selected[axis])
            throw RewriteError(row, tool, RewriteCause::DuplicateParameter);
        if (!selected[axis])
            selected[axis] = span;
        if (axis < 3) {
            if (first_xyz_mode && *first_xyz_mode != xyz_mode)
                throw RewriteError(row, tool, RewriteCause::ConflictingXyzModes);
            first_xyz_mode = xyz_mode;
            result.insert_at = std::max(result.insert_at, span.offset + span.length);
        }
        if (axis == 3)
            result.axis_e = axis_tool.has_value();
        if (!actions)
            cursor = span.offset - base + span.length;
    }
    if (!first_xyz_mode || !*first_xyz_mode)
        throw std::invalid_argument("Pressure rewrite has no absolute spatial source word.");
    for (std::size_t axis = 0; axis < selected.size(); ++axis) {
        if (selected[axis].has_value() != motion.tokens[axis].has_value())
            throw std::logic_error("Pressure rewrite selected source token mismatch.");
        if (!selected[axis])
            continue;
        const SourceToken &a = *selected[axis], &b = *motion.tokens[axis];
        if (b.offset < offset || b.offset - offset > raw.size() || b.length == 0 ||
            b.length > raw.size() - (b.offset - offset) ||
            a.offset != b.offset || a.length != b.length || a.letter != b.letter)
            throw std::logic_error("Pressure rewrite numeric span differs from original bytes.");
    }
    return result;
}

std::string number(double value, int digits)
{
    const double rounded = std::round(value * Formatter::pow_10[std::size_t(digits)]);
    // 2^63 is exactly representable as double; INT64_MAX is not.
    if (!std::isfinite(value) || !std::isfinite(rounded) || rounded < -0x1p63 || rounded >= 0x1p63)
        throw std::invalid_argument("Pressure rewrite number exceeds the SDK integer range.");
    Formatter formatter{std::uint16_t(digits), std::uint16_t(digits)};
    const char *begin = formatter.emit_axis('X', value, std::size_t(digits));
    const std::string rendered = formatter.string();
    return std::string(begin, rendered.size() - 3);
}

std::string annotation(const GeometryGroup &group, const ExtrusionResult &e,
                       double k, std::uint32_t tool)
{
    const char *kind = nullptr;
    switch (group.kind) {
    case SegmentKind::Accel: kind = "accel"; break;
    case SegmentKind::Steady: kind = "steady"; break;
    case SegmentKind::Decel: kind = "decel"; break;
    case SegmentKind::Merged: kind = "merged"; break;
    }
    if (!kind || group.duration <= 0 || group.distance <= 0 || k < 0)
        throw std::invalid_argument("Invalid pressure annotation group.");
    const double ratio = (group.nominal_end - group.nominal_start) / group.distance;
    return " ;NX_PRESSURE v=1 tool=" + std::to_string(tool) + " phase=" + kind +
        " v0_mm_s=" + metadata_number(group.speed_start) + " v1_mm_s=" + metadata_number(group.speed_end) +
        " e0_mm_s=" + metadata_number(ratio * group.speed_start) + " e1_mm_s=" + metadata_number(ratio * group.speed_end) +
        " dt_s=" + metadata_number(group.duration) + " k_s=" + metadata_number(k) +
        " de_nom_mm=" + metadata_number(e.nominal_delta) + " de_pressure_mm=" + metadata_number(e.pressure_delta);
}

std::string metadata_number(double value)
{
    if (!std::isfinite(value))
        throw std::invalid_argument("Nonfinite pressure annotation value.");
    if (value == 0)
        value = 0;
    char buffer[64];
    const std::to_chars_result encoded = std::to_chars(buffer, buffer + sizeof(buffer), value);
    if (encoded.ec != std::errc())
        throw std::invalid_argument("Pressure annotation number could not be encoded.");
    return std::string(buffer, encoded.ptr);
}
} // namespace
} // namespace nematx_pressure
