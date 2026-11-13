///|/ Released under the GNU Affero General Public License, version 3 or later.

#include "PressureNominal.hpp"

#include <charconv>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include "PressureSource.hpp"
#include "PressureTokens.hpp"
#include "../../nematx_firmware/src/NematXSyntax.hpp"

// Supplies the nominal coordinate channel to clients of PressureSource.hpp.
// Entry and exit compare only known coordinate keys, because Reader values may
// have passed through float. A local SDK tracker replays every event to supply
// modes at the exact motion, without assuming final private state is replayable.
// Each line is first stripped and lexed like NematX, then translated once using
// NematXSyntax.hpp. The actual standard text supplies selected numeric doubles;
// translated word ordinals reconnect them to owned source byte ranges. Reader
// items alone decide which coordinates change or become unknown. All temporary
// text views die inside the synchronous call; results retain values and ranges.
//
// Call tree:
//   read_nominal_fragment: check entry, follow lines/events, check exit.
//     require_keys: compare coordinate presence, not numeric values.
//     decode_line: strip, lex, and select the actual standard geometry text.
//       command_number: decode the SDK's full numeric G command spelling.
//     follow_motion: capture proven flags and E delta, then update endpoints.
//       update_coordinates: use Reader items to select source numbers or zeros.
//       selected_tokens: gate SDK float acceptance, read double, map source bytes.
//     update_coordinates: also handle direct and bracket position resets.

namespace nematx_pressure {
// Replays validated Reader deltas locally and returns owned source-double
// endpoints. Entry/exit key mismatch throws invalid_argument; unprovable source
// mapping throws logic_error. Borrowed source and copied prior stay unchanged.
NominalFragment read_nominal_fragment(const SourceFragment &source, NominalState prior,
                                     const std::vector<char> &resolved_axis_letters);

namespace {
using Tracker = slic3r_api::GCodeStateTracker;
using Parser = slic3r_api::GCodeGeneration::GCodeLineParser;
using Line = Parser::GCodeLine;
using Action = nematx_firmware::ReaderAction;
using Item = raw_gcode_reader_event_item;
constexpr std::array<char, 5> letters{{'X', 'Y', 'Z', 'E', 'F'}};
constexpr std::array<std::uint32_t, 5> types{{RAW_GCODE_READER_ITEM_X,
    RAW_GCODE_READER_ITEM_Y, RAW_GCODE_READER_ITEM_Z, RAW_GCODE_READER_ITEM_E,
    RAW_GCODE_READER_ITEM_FEEDRATE}};

// Temporary owned parser lines and scalar source offset for one physical line.
struct DecodedLine
{
    Line original, effective;
    std::size_t offset = 0;
    bool translated = false;
    bool bracket = false;
    std::optional<std::uint32_t> direct_g;
};

// Rejects missing or extra XYZ/E/F keys in one complete Reader state scan.
void require_keys(const NominalState &nominal, const Tracker &reader);
// Normalizes only the Reader's lexical input and reuses its dialect translator.
// The returned parsed text owns its bytes; no source or action views escape.
DecodedLine decode_line(std::string_view raw, std::size_t offset, const Tracker &reader,
                        const std::vector<char> &axes);
// Captures one motion with pre-event modes and source doubles. Only local state
// changes; impossible E target provenance throws before a result can escape.
NominalMotion follow_motion(const SourceFragment &source, const SourceLine &line,
                            std::size_t index, const DecodedLine &decoded,
                            const Tracker &reader, NominalState &state);
// Applies only listed coordinate items. INVALIDATE erases; motion uses modes;
// G28 sets Reader-selected XYZ to zero, G92 reads doubles, bracket E is unscaled.
// Any other coordinate provenance throws logic_error instead of copying float.
void update_coordinates(NominalState &state, const ReaderEvent &event,
                        const DecodedLine &decoded, const Tracker &reader);
// Returns accepted doubles with source numeric ranges. Invalid SDK numbers stay
// absent, and translated ranges must match numeric bytes at the same word ordinal.
std::array<std::optional<NumericToken>, 5> selected_tokens(
    const DecodedLine &decoded, std::array<std::optional<SourceToken>, 5> &spans);
// Decodes only a literal uppercase G and a full uint32 suffix, including zeros.
std::optional<std::uint32_t> command_number(std::string_view command);

void require_keys(const NominalState &nominal, const Tracker &reader)
{
    std::array<bool, 3> xyz{};
    bool feed = false;
    std::size_t e_count = 0;
    const raw_gcode_reader_state &raw = reader.state();
    for (std::uint32_t i = 0; i < raw.item_count; ++i) {
        const Item &item = raw.items[i];
        for (std::size_t axis = 0; axis < 3; ++axis)
            if (item.type == types[axis])
                xyz[axis] = true;
        if (item.type == RAW_GCODE_READER_ITEM_FEEDRATE)
            feed = true;
        if (item.type == RAW_GCODE_READER_ITEM_E) {
            ++e_count;
            if (nominal.e.find(item.target) == nominal.e.end())
                throw std::invalid_argument("Nominal E keys differ from Reader state.");
        }
    }
    for (std::size_t axis = 0; axis < 3; ++axis)
        if (xyz[axis] != nominal.xyz[axis].has_value())
            throw std::invalid_argument("Nominal XYZ keys differ from Reader state.");
    if (feed != nominal.feedrate.has_value() || e_count != nominal.e.size())
        throw std::invalid_argument("Nominal E/F keys differ from Reader state.");
}

DecodedLine decode_line(std::string_view raw, std::size_t offset, const Tracker &reader,
                        const std::vector<char> &axes)
{
    DecodedLine decoded;
    if (!raw.empty() && raw.back() == '\n')
        raw.remove_suffix(1);
    const std::string_view stripped = nematx_firmware::strip_line_number(raw);
    decoded.offset = offset + raw.size() - stripped.size();
    Parser parser;
    // parse_line stops at CR/NUL before translate_line can inspect any suffix.
    parser.parse_line(stripped, [&](Parser &, const Line &line) { decoded.original = line; });
    decoded.effective = decoded.original;
    if (decoded.original.command().empty())
        return decoded;
    nematx_firmware::ReaderModes modes;
    modes.xyz_absolute = reader.xyz_is_absolute();
    modes.e_absolute = reader.e_is_absolute();
    if (reader.active_tool())
        modes.active_tool = *reader.active_tool();
    const std::optional<std::vector<Action>> actions =
        nematx_firmware::translate_line(decoded.original.raw(), axes, modes);
    if (!actions) {
        decoded.direct_g = command_number(decoded.original.command());
        return decoded;
    }
    decoded.translated = true;
    std::size_t geometry = 0;
    for (const Action &action : *actions) {
        if (action.kind == Action::Kind::SetPosition)
            decoded.bracket = true;
        if (action.kind != Action::Kind::Standard)
            continue;
        Line standard;
        parser.parse_line(action.text, [&](Parser &, const Line &line) { standard = line; });
        const std::optional<std::uint32_t> number = command_number(standard.command());
        if (number && *number <= 3) {
            if (++geometry > 1)
                throw std::logic_error("Multiple translated geometry commands on one line.");
            decoded.effective = std::move(standard);
        }
    }
    return decoded;
}

NominalMotion follow_motion(const SourceFragment &source, const SourceLine &line,
                            std::size_t index, const DecodedLine &decoded,
                            const Tracker &reader, NominalState &state)
{
    const ReaderEvent &event = source.reader.events[index];
    NominalMotion motion{};
    motion.event_index = index;
    motion.start = state.xyz;
    motion.xyz_absolute = reader.xyz_is_absolute();
    motion.e_absolute = reader.e_is_absolute();
    motion.units_mm = reader.units_are_millimetres();
    motion.active_tool = reader.active_tool();
    const std::array<std::optional<NumericToken>, 5> numbers = selected_tokens(decoded, motion.tokens);
    const bool present_e = decoded.effective.has_parameter('E');
    if (present_e) {
        // The base emits an invalidated E immediately before its motion when
        // relative origin or the word is invalid; no older invalidation qualifies.
        const ReaderEvent *preceding = index > line.first_event ? &source.reader.events[index - 1] : nullptr;
        if (preceding && preceding->kind == RAW_GCODE_READER_EVENT_INVALIDATE)
            for (const Item &item : preceding->items)
                if (item.type == RAW_GCODE_READER_ITEM_E)
                    motion.e_target = item.target;
        for (const Item &item : event.items)
            if (item.type == RAW_GCODE_READER_ITEM_E)
                motion.e_target = item.target;
        if (!motion.e_target)
            throw std::logic_error("Present E has no adjacent Reader E target.");
    } else {
        motion.e_target = motion.active_tool;
        motion.e_delta = 0.0;
    }
    if (motion.e_target) {
        const std::map<std::uint32_t, double>::const_iterator found = state.e.find(*motion.e_target);
        if (found != state.e.end())
            motion.e_start = found->second;
    }
    if (present_e && numbers[3]) {
        const double scaled = numbers[3]->value * (motion.units_mm.value_or(true) ? 1.0 : 25.4);
        if (motion.e_absolute.value_or(true)) {
            motion.e_end = scaled;
            if (motion.e_start)
                motion.e_delta = scaled - *motion.e_start;
        } else {
            motion.e_delta = scaled;
            if (motion.e_start)
                motion.e_end = *motion.e_start + scaled;
        }
    } else if (!present_e) {
        motion.e_end = motion.e_start;
    }
    update_coordinates(state, event, decoded, reader);
    motion.end = state.xyz;
    motion.feedrate = state.feedrate;
    return motion;
}

void update_coordinates(NominalState &state, const ReaderEvent &event,
                        const DecodedLine &decoded, const Tracker &reader)
{
    const bool motion = event.kind == RAW_GCODE_READER_EVENT_LINEAR_MOVE ||
                        event.kind == RAW_GCODE_READER_EVENT_ARC_MOVE;
    const bool reset = event.kind == RAW_GCODE_READER_EVENT_SET_POSITION;
    std::array<std::optional<SourceToken>, 5> spans;
    std::array<std::optional<NumericToken>, 5> numbers;
    if (motion || (reset && decoded.direct_g == 92))
        numbers = selected_tokens(decoded, spans);
    const double scale = reader.units_are_millimetres().value_or(true) ? 1.0 : 25.4;
    for (const Item &item : event.items) {
        std::size_t axis = 0;
        while (axis < types.size() && types[axis] != item.type)
            ++axis;
        if (axis == types.size())
            continue;
        if (event.kind == RAW_GCODE_READER_EVENT_INVALIDATE) {
            if (axis < 3)
                state.xyz[axis].reset();
            else if (axis == 3)
                state.e.erase(item.target);
            else
                state.feedrate.reset();
            continue;
        }
        double value = 0.0;
        if (reset && decoded.bracket && axis == 3) {
            value = item.value;
        } else if (reset && decoded.direct_g == 28 && axis < 3) {
            value = 0.0;
        } else if ((motion || (reset && decoded.direct_g == 92)) && numbers[axis]) {
            value = numbers[axis]->value * scale;
            if (motion && axis < 3 && !reader.xyz_is_absolute().value_or(true)) {
                if (!state.xyz[axis])
                    throw std::logic_error("Accepted relative XYZ has no nominal origin.");
                value += *state.xyz[axis];
            }
            if (motion && axis == 3 && !reader.e_is_absolute().value_or(true)) {
                const std::map<std::uint32_t, double>::const_iterator found = state.e.find(item.target);
                if (found == state.e.end())
                    throw std::logic_error("Accepted relative E has no nominal origin.");
                value += found->second;
            }
        } else {
            throw std::logic_error("Reader coordinate has no proven numeric source.");
        }
        if (axis < 3)
            state.xyz[axis] = value;
        else if (axis == 3)
            state.e[item.target] = value;
        else
            state.feedrate = value;
    }
}

std::array<std::optional<NumericToken>, 5> selected_tokens(
    const DecodedLine &decoded, std::array<std::optional<SourceToken>, 5> &spans)
{
    std::array<std::optional<NumericToken>, 5> numbers;
    std::vector<std::string_view> original_words, standard_words;
    if (decoded.translated) {
        const std::string_view original(decoded.original.raw());
        original_words = nematx_firmware::detail::split_words(
            nematx_firmware::detail::trim(original.substr(0, original.find(';'))));
        const std::string_view standard(decoded.effective.raw());
        standard_words = nematx_firmware::detail::split_words(
            nematx_firmware::detail::trim(standard.substr(0, standard.find(';'))));
    }
    for (std::size_t axis = 0; axis < letters.size(); ++axis) {
        float accepted = 0.0f;
        if (!decoded.effective.parameter_value(letters[axis], accepted))
            continue;
        numbers[axis] = read_parameter(decoded.effective, letters[axis]);
        if (!numbers[axis])
            throw std::logic_error("Accepted SDK parameter is absent.");
        const NumericToken &number = *numbers[axis];
        if (!decoded.translated) {
            const std::string_view position = decoded.effective.parameter_position(letters[axis]);
            spans[axis] = SourceToken{decoded.offset + number.offset, number.length, position.front()};
            continue;
        }
        // Translation copies all non-mode words in order, including other G
        // words. Only exact G90/G91 words disappear; selected duplicates remain.
        const std::string_view position = decoded.effective.parameter_position(letters[axis]);
        std::size_t ordinal = 1;
        while (ordinal < standard_words.size() && standard_words[ordinal].data() != position.data())
            ++ordinal;
        if (ordinal == standard_words.size())
            throw std::logic_error("Selected standard word has no ordinal.");
        std::size_t selected = 0;
        std::string_view source_word;
        for (std::size_t word = 1; word < original_words.size(); ++word) {
            if (original_words[word] == "G90" || original_words[word] == "G91")
                continue;
            if (++selected == ordinal) {
                source_word = original_words[word];
                break;
            }
        }
        const std::string_view numeric(decoded.effective.raw().data() + number.offset, number.length);
        if (source_word.size() != number.length + 1 || source_word.substr(1) != numeric)
            throw std::logic_error("Translated numeric bytes differ from original word.");
        const std::size_t local = std::size_t(source_word.data() - decoded.original.raw().data());
        spans[axis] = SourceToken{decoded.offset + local + 1, number.length, source_word.front()};
    }
    return numbers;
}

std::optional<std::uint32_t> command_number(std::string_view command)
{
    if (command.size() < 2 || command.front() != 'G')
        return std::nullopt;
    std::uint32_t number = 0;
    const std::from_chars_result result = std::from_chars(command.data() + 1,
        command.data() + command.size(), number);
    if (result.ec != std::errc() || result.ptr != command.data() + command.size())
        return std::nullopt;
    return number;
}
} // namespace

NominalFragment read_nominal_fragment(const SourceFragment &source, NominalState prior,
                                     const std::vector<char> &resolved_axis_letters)
{
    require_keys(prior, source.reader.initial);
    Tracker reader = source.reader.initial;
    NominalFragment result;
    for (const SourceLine &line : source.lines) {
        const DecodedLine decoded = decode_line(std::string_view(source.text).substr(line.offset, line.length),
                                               line.offset, reader, resolved_axis_letters);
        std::size_t motions = 0;
        for (std::size_t index = line.first_event; index < line.first_event + line.event_count; ++index) {
            const ReaderEvent &event = source.reader.events[index];
            if (event.kind == RAW_GCODE_READER_EVENT_LINEAR_MOVE || event.kind == RAW_GCODE_READER_EVENT_ARC_MOVE) {
                if (++motions > 1 || decoded.original.command().empty())
                    throw std::logic_error("A source line cannot join this Reader motion.");
                result.motions.push_back(follow_motion(source, line, index, decoded, reader, prior));
            } else {
                update_coordinates(prior, event, decoded, reader);
            }
            const raw_gcode_reader_event raw{event.kind, event.items.data(), std::uint32_t(event.items.size())};
            reader.apply(raw);
        }
    }
    require_keys(prior, source.reader.final);
    result.final = std::move(prior);
    return result;
}
} // namespace nematx_pressure
