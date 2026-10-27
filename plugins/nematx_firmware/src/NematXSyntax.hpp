///|/ Copyright (c) NematXSlicer 2026 Durand Remi @supermerill
///|/
///|/ NematXSlicer is released under the terms of the AGPLv3 or higher

#ifndef nematx_firmware_NematXSyntax_hpp_
#define nematx_firmware_NematXSyntax_hpp_

// Text rules of the NematX G-code dialect, written in plain C++ so they can be
// unit-tested without the slicer. The firmware session (writer) and the reader
// both use them.
//
// NematX differs from Marlin mainly in its syntax:
// - each extruder has its own axis letter (A for the first tool, B for the
//   second...) instead of a shared E axis and T tool changes;
// - heater and fan values use '=' (M104=215, M106=255) and the second tool has
//   its own codes (M124, M126, M129);
// - waiting is a separate bare command (M109, M129, M190);
// - an axis position is set with "A[SET_POSITION POS=0]" instead of G92;
// - with relative extrusion, G90/G91 appear inside each move:
//   "G1 G90 X10 Y5 G91 A0.3": X/Y are absolute, the A distance is relative.

#include <cctype>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace nematx_firmware {

// The NematX codes only address two tools (M104/M124, M106/M126, M109/M129).
constexpr size_t k_max_tools = 2;

// Default axis letter of a tool whose extruder_axis entry is empty: A, then B.
inline char default_axis_letter(size_t tool) { return char('A' + tool); }

// Letters that already have a meaning in a NematX move or command and so cannot
// name an extruder axis.
inline bool is_reserved_letter(char letter)
{
    switch (letter) {
    case 'F': case 'G': case 'I': case 'J': case 'K': case 'M': case 'N':
    case 'P': case 'R': case 'S': case 'T': case 'X': case 'Y': case 'Z':
        return true;
    default:
        return false;
    }
}

// Turn the extruder_axis entries into one letter per tool. An empty entry uses
// the default letter. Throws std::invalid_argument with a user-readable message
// when an entry is not one free letter, or when two tools share a letter.
inline std::vector<char> resolve_axis_letters(const std::vector<std::string> &configured, size_t tool_count)
{
    std::vector<char> letters;
    letters.reserve(tool_count);
    for (size_t tool = 0; tool < tool_count; ++tool) {
        std::string entry = tool < configured.size() ? configured[tool] : std::string();
        // Ignore surrounding spaces typed in a profile.
        while (!entry.empty() && std::isspace(static_cast<unsigned char>(entry.back())))
            entry.pop_back();
        while (!entry.empty() && std::isspace(static_cast<unsigned char>(entry.front())))
            entry.erase(entry.begin());
        char letter = default_axis_letter(tool);
        if (!entry.empty()) {
            const char upper = char(std::toupper(static_cast<unsigned char>(entry.front())));
            if (entry.size() != 1 || upper < 'A' || upper > 'Z' || is_reserved_letter(upper))
                throw std::invalid_argument(
                    "extruder_axis for extruder " + std::to_string(tool + 1) + " must be one free letter "
                    "(not F, G, I, J, K, M, N, P, R, S, T, X, Y or Z); found \"" + entry + "\".");
            letter = upper;
        }
        for (size_t previous = 0; previous < letters.size(); ++previous)
            if (letters[previous] == letter)
                throw std::invalid_argument(
                    std::string("extruder_axis: extruders ") + std::to_string(previous + 1) + " and " +
                    std::to_string(tool + 1) + " both use axis " + letter + ".");
        letters.push_back(letter);
    }
    return letters;
}

// Return the tool driven by an axis letter, or no value for another letter.
inline std::optional<size_t> tool_of_axis(const std::vector<char> &letters, char letter)
{
    for (size_t tool = 0; tool < letters.size(); ++tool)
        if (letters[tool] == letter)
            return tool;
    return std::nullopt;
}

// Codes of the heater, wait and fan commands of one tool.
inline const char *tool_temperature_code(size_t tool) { return tool == 0 ? "M104" : "M124"; }
inline const char *tool_wait_code(size_t tool) { return tool == 0 ? "M109" : "M129"; }
inline const char *tool_fan_code(size_t tool) { return tool == 0 ? "M106" : "M126"; }

// ---------------------------------------------------------------------------
// Output filter: line numbers (gcode_line_number) and comment removal
// (gcode_no_comment), applied to everything the firmware writes.
//
// As in the legacy slicer:
// - comment removal drops comment-only and blank lines and cuts the comment
//   at the end of a command line;
// - line numbering prefixes every non-blank line, comments included, with
//   "N<k> " where k grows by 10 (N10, N20...).
// Text may arrive in pieces that split a line, so an incomplete last line is
// kept until the next piece (or flush()).
// ---------------------------------------------------------------------------
class OutputFilter
{
public:
    OutputFilter(bool remove_comments = false, bool number_lines = false)
        : m_remove_comments(remove_comments), m_number_lines(number_lines) {}

    bool active() const { return m_remove_comments || m_number_lines; }

    // Filter one piece of output and return the complete lines it finishes.
    std::string process(std::string_view text)
    {
        if (!active())
            return std::string(text);
        std::string output;
        size_t begin = 0;
        while (begin < text.size()) {
            const size_t end = text.find('\n', begin);
            if (end == std::string_view::npos) {
                m_pending.append(text.substr(begin));
                break;
            }
            m_pending.append(text.substr(begin, end - begin));
            append_line(m_pending, output);
            m_pending.clear();
            begin = end + 1;
        }
        return output;
    }

    // Emit an incomplete last line, if any (end of the file).
    std::string flush()
    {
        std::string output;
        if (!m_pending.empty())
            append_line(m_pending, output);
        m_pending.clear();
        return output;
    }

private:
    void append_line(std::string line, std::string &output)
    {
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        if (m_remove_comments) {
            const size_t comment = line.find(';');
            if (comment != std::string::npos)
                line.erase(comment);
            while (!line.empty() && std::isspace(static_cast<unsigned char>(line.back())))
                line.pop_back();
        }
        if (is_blank(line))
            return;
        if (m_number_lines) {
            m_line_count += 1;
            output += "N" + std::to_string(m_line_count * 10) + " ";
        }
        output += line;
        output += '\n';
    }

    static bool is_blank(const std::string &line)
    {
        for (char c : line)
            if (!std::isspace(static_cast<unsigned char>(c)))
                return false;
        return true;
    }

    bool m_remove_comments;
    bool m_number_lines;
    uint64_t m_line_count = 0;
    std::string m_pending;
};

// Remove a leading line number ("N120 G1 X1" -> "G1 X1"). Other lines are
// returned unchanged. The reader applies this before interpreting a line.
inline std::string_view strip_line_number(std::string_view line)
{
    size_t pos = 0;
    while (pos < line.size() && (line[pos] == ' ' || line[pos] == '\t'))
        ++pos;
    if (pos >= line.size() || (line[pos] != 'N' && line[pos] != 'n'))
        return line;
    size_t digits = pos + 1;
    while (digits < line.size() && std::isdigit(static_cast<unsigned char>(line[digits])))
        ++digits;
    if (digits == pos + 1)
        return line;
    // The number must be a whole word: followed by a space, a comment or nothing.
    if (digits < line.size() && line[digits] != ' ' && line[digits] != '\t' && line[digits] != ';')
        return line;
    while (digits < line.size() && (line[digits] == ' ' || line[digits] == '\t'))
        ++digits;
    return line.substr(digits);
}

// ---------------------------------------------------------------------------
// Reader translation: turn one NematX line into simple actions that a standard
// (Marlin-like) reader already knows how to apply.
// ---------------------------------------------------------------------------
struct ReaderAction
{
    enum class Kind {
        XyzMode,     // G90 (absolute = true) or G91
        ExtrusionMode, // M82 (absolute = true) or M83
        SelectTool,  // tool change deduced from an axis letter
        Standard,    // a standard command in `text`, to dispatch normally
        SetPosition, // "A[SET_POSITION POS=v]": E position of `tool`
        WaitTool,    // bare M109 / M129: wait for the current target of `tool`
        WaitBed,     // bare M190: wait for the current bed target
        Neutral,     // understood, changes no tracked state (G4 X...)
        Unknown      // NematX-looking line that cannot be read
    };
    Kind kind = Kind::Unknown;
    bool absolute = true;
    size_t tool = 0;
    double value = 0.0;
    std::string text;
};

// What the reader knows before the line, needed to emit only real changes.
struct ReaderModes
{
    std::optional<bool> xyz_absolute;
    std::optional<bool> e_absolute;
    std::optional<size_t> active_tool;
    bool fan_percentage = false; // fan values are 0..100 instead of 0..255
};

namespace detail {

inline std::string_view trim(std::string_view text)
{
    while (!text.empty() && std::isspace(static_cast<unsigned char>(text.front())))
        text.remove_prefix(1);
    while (!text.empty() && std::isspace(static_cast<unsigned char>(text.back())))
        text.remove_suffix(1);
    return text;
}

inline std::vector<std::string_view> split_words(std::string_view text)
{
    std::vector<std::string_view> words;
    size_t pos = 0;
    while (pos < text.size()) {
        while (pos < text.size() && std::isspace(static_cast<unsigned char>(text[pos])))
            ++pos;
        const size_t begin = pos;
        while (pos < text.size() && !std::isspace(static_cast<unsigned char>(text[pos])))
            ++pos;
        if (pos > begin)
            words.push_back(text.substr(begin, pos - begin));
    }
    return words;
}

// Check a decimal number without converting it: the text is copied as is into
// the standard command, so the reader's own parser does the conversion.
inline bool is_number(std::string_view text)
{
    if (text.empty())
        return false;
    size_t pos = text.front() == '-' || text.front() == '+' ? 1 : 0;
    bool digits = false, dot = false;
    for (; pos < text.size(); ++pos) {
        const char c = text[pos];
        if (std::isdigit(static_cast<unsigned char>(c)))
            digits = true;
        else if (c == '.' && !dot)
            dot = true;
        else
            return false;
    }
    return digits;
}

// Convert a number already checked by is_number(). from_chars ignores the C
// locale, so a decimal comma setting cannot change the result.
inline double to_double(std::string_view text)
{
    if (!text.empty() && text.front() == '+')
        text.remove_prefix(1);
    double value = 0.0;
    std::from_chars(text.data(), text.data() + text.size(), value);
    return value;
}

inline ReaderAction make(ReaderAction::Kind kind)
{
    ReaderAction action;
    action.kind = kind;
    return action;
}

inline ReaderAction standard(std::string text)
{
    ReaderAction action = make(ReaderAction::Kind::Standard);
    action.text = std::move(text);
    return action;
}

} // namespace detail

// Translate one line (line number already removed, comment allowed). Return no
// value when the line is standard G-code that a Marlin-like reader handles
// directly; otherwise return the actions to apply in order.
inline std::optional<std::vector<ReaderAction>> translate_line(std::string_view line,
                                                               const std::vector<char> &axis_letters,
                                                               const ReaderModes &modes)
{
    using Kind = ReaderAction::Kind;
    const size_t comment = line.find(';');
    const std::string_view code = detail::trim(comment == std::string_view::npos ? line : line.substr(0, comment));
    if (code.empty())
        return std::nullopt;

    // "A[SET_POSITION POS=0.2]": set the position of one extruder axis.
    constexpr std::string_view set_position = "[SET_POSITION POS=";
    if (code.size() > set_position.size() + 2 && code.substr(1, set_position.size()) == set_position &&
        code.back() == ']') {
        const std::string_view number = code.substr(1 + set_position.size(),
                                                    code.size() - 2 - set_position.size());
        const std::optional<size_t> tool = tool_of_axis(axis_letters, char(std::toupper(
            static_cast<unsigned char>(code.front()))));
        if (!tool || !detail::is_number(number))
            return std::vector<ReaderAction>{ detail::make(Kind::Unknown) };
        ReaderAction action = detail::make(Kind::SetPosition);
        action.tool = *tool;
        action.value = detail::to_double(number);
        return std::vector<ReaderAction>{ action };
    }

    const std::vector<std::string_view> words = detail::split_words(code);
    const std::string_view command = words.front();

    // Commands with '=': "M104=215", "M106=127.5"...
    const size_t equal = command.find('=');
    if (equal != std::string_view::npos) {
        const std::string_view name = command.substr(0, equal);
        const std::string_view value = command.substr(equal + 1);
        if (words.size() != 1 || !detail::is_number(value))
            return std::vector<ReaderAction>{ detail::make(Kind::Unknown) };
        const std::string number(value);
        if (name == "M104" || name == "M124")
            return std::vector<ReaderAction>{ detail::standard(
                "M104 S" + number + (name == "M104" ? " T0" : " T1")) };
        if (name == "M140")
            return std::vector<ReaderAction>{ detail::standard("M140 S" + number) };
        if (name == "M106" || name == "M126") {
            // The standard reader expects a 0..255 value.
            const double raw = detail::to_double(number);
            const double pwm = modes.fan_percentage ? raw * 255.0 / 100.0 : raw;
            return std::vector<ReaderAction>{ detail::standard(
                std::string("M106 P") + (name == "M106" ? "0" : "1") + " S" + std::to_string(pwm)) };
        }
        return std::vector<ReaderAction>{ detail::make(Kind::Unknown) };
    }

    // Bare waits and the other NematX-only commands.
    if (words.size() == 1) {
        if (command == "M109" || command == "M129") {
            ReaderAction action = detail::make(Kind::WaitTool);
            action.tool = command == "M109" ? 0 : 1;
            return std::vector<ReaderAction>{ action };
        }
        if (command == "M190")
            return std::vector<ReaderAction>{ detail::make(Kind::WaitBed) };
        // NematX switches the bed heater off with M141.
        if (command == "M141")
            return std::vector<ReaderAction>{ detail::standard("M140 S0") };
        if (command == "M127")
            return std::vector<ReaderAction>{ detail::standard("M106 P1 S0") };
    }
    if (command == "G71")
        return std::vector<ReaderAction>{ detail::standard("G21") };
    if (command == "G70")
        return std::vector<ReaderAction>{ detail::standard("G20") };
    if (command == "G4")
        return std::vector<ReaderAction>{ detail::make(Kind::Neutral) };

    // Moves: translate the extruder axis letter to E and the inline G90/G91.
    if (command != "G0" && command != "G1" && command != "G2" && command != "G3")
        return std::nullopt;
    bool nematx_move = false;
    for (size_t idx = 1; idx < words.size(); ++idx) {
        const std::string_view word = words[idx];
        if (word == "G90" || word == "G91" ||
            (!word.empty() && tool_of_axis(axis_letters, char(std::toupper(static_cast<unsigned char>(word.front()))))))
            nematx_move = true;
    }
    if (!nematx_move)
        return std::nullopt;

    // The positioning mode in force before this line; NematX starts absolute.
    bool mode_absolute = modes.xyz_absolute.value_or(true);
    std::optional<bool> xyz_mode;       // mode in force for the first X/Y/Z word
    std::optional<bool> extrusion_mode; // mode in force for the extruder word
    std::optional<size_t> tool;
    std::string translated(command);
    for (size_t idx = 1; idx < words.size(); ++idx) {
        const std::string_view word = words[idx];
        if (word == "G90" || word == "G91") {
            mode_absolute = word == "G90";
            continue;
        }
        const char letter = char(std::toupper(static_cast<unsigned char>(word.front())));
        const std::string_view value = word.substr(1);
        if (!detail::is_number(value))
            return std::vector<ReaderAction>{ detail::make(Kind::Unknown) };
        if (const std::optional<size_t> axis_tool = tool_of_axis(axis_letters, letter)) {
            if (tool)
                return std::vector<ReaderAction>{ detail::make(Kind::Unknown) };
            tool = axis_tool;
            extrusion_mode = mode_absolute;
            translated += " E";
            translated += value;
            continue;
        }
        if ((letter == 'X' || letter == 'Y' || letter == 'Z') && !xyz_mode)
            xyz_mode = mode_absolute;
        translated += ' ';
        translated += letter;
        translated += value;
    }

    std::vector<ReaderAction> actions;
    if (tool && modes.active_tool != tool) {
        ReaderAction action = detail::make(Kind::SelectTool);
        action.tool = *tool;
        actions.push_back(action);
    }
    // XYZ words are read with the mode in force where they appear.
    bool current_xyz = modes.xyz_absolute.value_or(true);
    if (xyz_mode && (!modes.xyz_absolute || *xyz_mode != current_xyz)) {
        ReaderAction action = detail::make(Kind::XyzMode);
        action.absolute = *xyz_mode;
        actions.push_back(action);
        current_xyz = *xyz_mode;
    }
    if (extrusion_mode && modes.e_absolute != extrusion_mode) {
        ReaderAction action = detail::make(Kind::ExtrusionMode);
        action.absolute = *extrusion_mode;
        actions.push_back(action);
    }
    actions.push_back(detail::standard(translated));
    // The last G90/G91 of the line stays in force for the next lines.
    if (mode_absolute != current_xyz || (!modes.xyz_absolute && !xyz_mode)) {
        ReaderAction action = detail::make(Kind::XyzMode);
        action.absolute = mode_absolute;
        actions.push_back(action);
    }
    return actions;
}

} // namespace nematx_firmware

#endif // nematx_firmware_NematXSyntax_hpp_
