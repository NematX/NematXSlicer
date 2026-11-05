/*
G-code export text checks
========================

These test helpers validate complete width, statistics and configuration comment blocks
without emitting per-character or per-line Catch assertions. On failure, each
helper copies the first malformed line for the caller to include in one report.

Keep one check and its diagnostic together when the active Catch release lacks CHECK_MESSAGE.

Pass a complete block to the corresponding checker, retain its diagnostic when
the check returns false, and make one assertion on that result. This keeps a
malformed line visible without producing thousands of successful assertions.
The helpers copy failure text so the caller does not retain a view into a
temporary line buffer.
*/

#ifndef test_plugins_gcode_export_test_helpers_hpp_
#define test_plugins_gcode_export_test_helpers_hpp_

#include <algorithm>
#include <cstddef>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/Print.hpp"


#ifndef CHECK_MESSAGE
#define CHECK_MESSAGE(expression, message) \
    do { \
        const bool catch_check_message_result = (expression); \
        INFO(message); \
        CHECK(catch_check_message_result); \
    } while (false)
#endif

namespace Slic3r::Test {

// Export a reference through the exporter of `selection`, as a requester does: `destination` is set as the
// output_path of a copy of `config`. Return the local file that the exporter reported (empty when it reported none).
inline std::string export_print_to(Orchestrator &orchestrator,
                                   Print &print,
                                   const DynamicPrintConfig &config,
                                   const std::string &reference_path,
                                   const std::string &destination,
                                   const Orchestrator::GCodeExporterSelection &selection,
                                   ThumbnailsGeneratorCallback thumbnail_cb = nullptr,
                                   ExportInputsCallback inputs_cb = nullptr)
{
    DynamicPrintConfig request_config = config;
    request_config.set_key_value("output_path", new ConfigOptionString(destination));
    orchestrator.export_print(print, request_config, reference_path, selection, std::move(thumbnail_cb),
                              std::move(inputs_cb));
    return print.export_result().local_path;
}

} // namespace Slic3r::Test

namespace Slic3r::Test::Plugins::GCodeExportTest {

// Validate borrowed width comments and region separators, copying the first malformed line to owned output.
inline bool is_width_block(std::string_view block, std::string &first_bad_line)
{
    first_bad_line.clear();
    const std::string_view separator = " extrusion width = ";
    std::size_t line_begin = 0;
    while (line_begin < block.size()) {
        const std::size_t line_end = block.find('\n', line_begin);
        if (line_end == std::string_view::npos) {
            first_bad_line.assign(block.data() + line_begin, block.size() - line_begin);
            return false;
        }
        const std::string_view line = block.substr(line_begin, line_end - line_begin);
        if (!line.empty()) {
            const std::size_t label_begin = 2;
            const std::size_t label_end = line.find(separator);
            if (line.compare(0, 2, "; ") != 0 || label_end == std::string_view::npos || label_end <= label_begin) {
                first_bad_line.assign(line.data(), line.size());
                return false;
            }
            const std::string_view label = line.substr(label_begin, label_end - label_begin);
            if (!std::all_of(label.begin(), label.end(), [](char value) {
                    return (value >= 'a' && value <= 'z') || value == ' ';
                })) {
                first_bad_line.assign(line.data(), line.size());
                return false;
            }
            const std::string_view width = line.substr(label_end + separator.size());
            const std::size_t decimal = width.find('.');
            if (decimal == std::string_view::npos || decimal == 0 || decimal + 5 != width.size() ||
                !std::all_of(width.begin(), width.begin() + decimal, [](char value) {
                    return value >= '0' && value <= '9';
                }) ||
                width[decimal + 1] < '0' || width[decimal + 1] > '9' ||
                width[decimal + 2] < '0' || width[decimal + 2] > '9' ||
                width.substr(decimal + 3) != "mm") {
                first_bad_line.assign(line.data(), line.size());
                return false;
            }
        }
        line_begin = line_end + 1;
    }
    return true;
}

// Validate borrowed newline-terminated key/value comments, copying the first malformed line to owned output.
inline bool is_configuration_block(std::string_view block, std::string &first_bad_line)
{
    first_bad_line.clear();
    std::size_t line_begin = 0;
    while (line_begin < block.size()) {
        const std::size_t line_end = block.find('\n', line_begin);
        if (line_end == std::string_view::npos) {
            first_bad_line.assign(block.data() + line_begin, block.size() - line_begin);
            return false;
        }
        const std::string_view line = block.substr(line_begin, line_end - line_begin);
        const std::size_t separator = line.find(" = ");
        if (line.compare(0, 2, "; ") != 0 || separator == std::string_view::npos || separator <= 2 ||
            line.find('\r') != std::string_view::npos) {
            first_bad_line = line.empty() ? "<empty line>" : std::string(line);
            return false;
        }
        line_begin = line_end + 1;
    }
    return true;
}

// Tell whether text is one decimal number with exactly two fractional digits, such as "12.34".
inline bool is_two_decimal_number(std::string_view text)
{
    const std::size_t decimal = text.find('.');
    return decimal != std::string_view::npos && decimal > 0 && decimal + 3 == text.size() &&
           std::all_of(text.begin(), text.begin() + decimal, [](char value) { return value >= '0' && value <= '9'; }) &&
           text[decimal + 1] >= '0' && text[decimal + 1] <= '9' && text[decimal + 2] >= '0' && text[decimal + 2] <= '9';
}

// Tell whether text is a ", " separated list of two-decimal numbers.
inline bool is_two_decimal_list(std::string_view text)
{
    std::size_t begin = 0;
    while (true) {
        const std::size_t end = text.find(", ", begin);
        if (!is_two_decimal_number(text.substr(begin, end == std::string_view::npos ? std::string_view::npos : end - begin)))
            return false;
        if (end == std::string_view::npos)
            return true;
        begin = end + 2;
    }
}

// Validate a borrowed statistics block: "; label = value" lines in file order, the optional weight and cost lines
// allowed to be absent, each value in its expected number format. Copies the first malformed line to owned output.
inline bool is_statistics_block(std::string_view block, std::string &first_bad_line)
{
    first_bad_line.clear();
    struct Expected
    {
        std::string_view label;
        bool optional;
        int kind; // 0: list of two-decimal numbers, 1: two-decimal number, 2: integer, 3: flag
    };
    const std::vector<Expected> expected = {
        {"filament used [mm]", false, 0}, {"filament used [cm3]", false, 0}, {"filament used [g]", true, 0},
        {"filament cost", true, 0}, {"total filament used [g]", false, 1}, {"total filament cost", false, 1},
        {"total filament used for wipe tower [g]", false, 1}, {"total toolchanges", false, 2},
        {"objects layers count", false, 2}, {"total layers count", false, 2}, {"has_support", false, 3}};
    std::size_t next = 0;
    std::size_t line_begin = 0;
    while (line_begin < block.size()) {
        const std::size_t line_end = block.find('\n', line_begin);
        const std::string_view line = block.substr(
            line_begin, line_end == std::string_view::npos ? std::string_view::npos : line_end - line_begin);
        const std::size_t separator = line.find(" = ");
        bool matched = line_end != std::string_view::npos && line.compare(0, 2, "; ") == 0 &&
                       separator != std::string_view::npos && line.find('\r') == std::string_view::npos;
        if (matched) {
            const std::string_view label = line.substr(2, separator - 2);
            const std::string_view value = line.substr(separator + 3);
            while (next < expected.size() && expected[next].label != label && expected[next].optional)
                ++next;
            matched = next < expected.size() && expected[next].label == label;
            if (matched) {
                const int kind = expected[next].kind;
                matched = kind == 0 ? is_two_decimal_list(value) :
                          kind == 1 ? is_two_decimal_number(value) :
                          kind == 2 ? !value.empty() && std::all_of(value.begin(), value.end(), [](char c) { return c >= '0' && c <= '9'; }) :
                                      value == "0" || value == "1";
                ++next;
            }
        }
        if (!matched) {
            first_bad_line = line.empty() ? "<empty line>" : std::string(line);
            return false;
        }
        line_begin = line_end + 1;
    }
    // Every line before has_support is mandatory except the weight and cost lines.
    while (next < expected.size() && expected[next].optional)
        ++next;
    if (next != expected.size()) {
        first_bad_line = "<missing line: " + std::string(expected[next].label) + ">";
        return false;
    }
    return true;
}

} // namespace Slic3r::Test::Plugins::GCodeExportTest

#endif // test_plugins_gcode_export_test_helpers_hpp_
