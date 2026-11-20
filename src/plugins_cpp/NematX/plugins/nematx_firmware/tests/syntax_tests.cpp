///|/ Copyright (c) NematXSlicer 2026 Durand Remi @supermerill
///|/
///|/ NematXSlicer is released under the terms of the AGPLv3 or higher

// NematX syntax tests: plain C++, no host library. They cover the axis letters,
// the output filter (line numbers, comment removal), the reader translation and
// the object metadata lines and the pressure setting definitions published by
// the provider. main runs each group; require reports the first failed check.
//     main -> test_pressure_settings: inspect the provider's raw definitions
//     main -> test_*: check syntax, translation, filtering and metadata
#include "NematXPressureSettings.hpp"
#include "NematXObjectMetadata.hpp"
#include "NematXSyntax.hpp"

#include <array>
#include <cmath>
#include <iostream>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>

using namespace nematx_firmware;
using Kind = ReaderAction::Kind;

// Run all local checks and return failure with the first diagnostic.
int main();

namespace {

// Throw with the supplied diagnostic when a local check fails.
void require(bool condition, const std::string &message);

// Inspect the actual definitions without linking the host or parsing sources.
void test_pressure_settings();

// Exercise the exact metadata handoff for opaque non-LF text and a source CR.
void test_metadata_separator();

void test_metadata_separator()
{
    for (bool comments : {false, true}) {
        for (bool numbers : {false, true}) {
            for (const std::string &tail : {std::string("opaque"), std::string("opaque\r"), std::string("opaque\n")}) {
                OutputFilter filter(comments, numbers);
                std::string text = filter.process(tail);
                text += flush_before_metadata(filter, tail.back() != '\n');
                text += "; metadata\n";
                const std::string expected = (numbers ? "N10 opaque" : "opaque") +
                    std::string(!comments && !numbers && tail.back() == '\r' ? "\r\n" : "\n") + "; metadata\n";
                require(text == expected, "metadata starts on its own line without an invented CR");
            }
        }
    }
    OutputFilter inactive;
    require(inactive.process("opaque") + inactive.flush() == "opaque", "true EOF keeps the missing LF");
}

void test_pressure_settings()
{
    const std::array<raw_config_option_def, 3> definitions = pressure_option_definitions();
    const char *keys[] = { "extruder_pressure_factor", "split_extrusion_acceleration",
                           "filament_compressibility_factor" };
    for (size_t i = 0; i < definitions.size(); ++i) {
        const raw_config_option_def &def = definitions[i];
        require(std::string(def.opt_key) == keys[i], "pressure key and unique order");
        require(def.type == (i == 1 ? RAW_CO_VECTOR_BOOL : RAW_CO_VECTOR_FLOAT), "vector type");
        require(def.container_type == RAW_CONTAINER_TYPE_PROJECT, "project container");
        require(def.printer_technology == RAW_PT_FFF, "FFF technology");
        require(def.invalidates_step == STEP_GCODE, "G-code invalidation metadata");
        require(def.mode == RAW_CONFIG_OPTION_MODE_EXPERT, "expert mode");
        require(def.is_vector_extruder == 1, "one value per extruder");
        require(std::string(def.default_serialized_value) == "0", "disabled default");
        require(def.option_preset_type == (i == 2 ? RAW_PRESET_TYPE_FFF_FILAMENT :
                                                   RAW_PRESET_TYPE_FFF_TOOL_EXTRUDER), "preset scope");
        require(def.category == (i == 2 ? RAW_OPTION_CATEGORY_FILAMENT : RAW_OPTION_CATEGORY_EXTRUDERS),
                "GUI category");
        require(def.label != nullptr && def.label[0] != '\0', "visible label");
        require(def.tooltip != nullptr && def.tooltip[0] != '\0', "tooltip");
        if (i != 1) {
            require(def.has_min == 1, "float minimum enabled");
            require(def.min_value == 0, "nonnegative float");
            require(def.precision == 6, "float display precision");
        }
    }
    require(std::string(definitions[0].sidetext) == "s", "pressure unit seconds");
    require(std::string(definitions[0].tooltip).find("K times the model speed") != std::string::npos,
            "pressure tooltip identifies the model");
    require(definitions[1].sidetext == nullptr, "no boolean unit");
    require(std::string(definitions[1].tooltip).find("K is zero") != std::string::npos,
            "F09 independent of pressure");
    require(definitions[2].sidetext == nullptr, "no compressibility delay unit");
    require(std::string(definitions[2].tooltip).find("Inactive") != std::string::npos,
            "compatibility tooltip states inactivity");
}

void require(bool condition, const std::string &message)
{
    if (!condition)
        throw std::runtime_error(message);
}

template<class Function>
void require_throws(Function function, const std::string &message)
{
    try {
        function();
    } catch (const std::invalid_argument &) {
        return;
    }
    throw std::runtime_error(message);
}

const std::vector<char> k_axes{ 'A', 'B' };

std::vector<ReaderAction> translate(const std::string &line, ReaderModes modes = {})
{
    const std::optional<std::vector<ReaderAction>> actions = translate_line(line, k_axes, modes);
    require(actions.has_value(), "expected a NematX translation for: " + line);
    return *actions;
}

void test_axis_letters()
{
    require(resolve_axis_letters({}, 2) == std::vector<char>{ 'A', 'B' }, "empty entries use A and B");
    require(resolve_axis_letters({ " b ", "a" }, 2) == std::vector<char>{ 'B', 'A' }, "letters are trimmed and upper-cased");
    require(resolve_axis_letters({ "E" }, 1) == std::vector<char>{ 'E' }, "E is allowed");
    require_throws([] { resolve_axis_letters({ "X" }, 1); }, "X is reserved");
    require_throws([] { resolve_axis_letters({ "AB" }, 1); }, "two letters are rejected");
    require_throws([] { resolve_axis_letters({ "A", "A" }, 2); }, "a shared letter is rejected");
    require_throws([] { resolve_axis_letters({ "", "A" }, 2); }, "a letter equal to a default is rejected");
}

void test_output_filter()
{
    OutputFilter none;
    require(none.process("G1 X1 ; move\n;TYPE:x\n") == "G1 X1 ; move\n;TYPE:x\n", "inactive filter keeps text");

    OutputFilter comments(true, false);
    require(comments.process("G1 X1 ; move\n;TYPE:x\n\nM104=200\n") == "G1 X1\nM104=200\n",
            "comment removal keeps commands only");

    OutputFilter numbers(false, true);
    require(numbers.process("G90\n;comment\n\nG71\n") == "N10 G90\nN20 ;comment\nN30 G71\n",
            "line numbers on every non-blank line, by tens");

    // A line split between two pieces is numbered once, when complete.
    OutputFilter split(true, true);
    require(split.process("G1 X") == "", "incomplete line is kept");
    require(split.process("1 ; c\nG1 X2") == "N10 G1 X1\n", "completed line is emitted");
    require(split.flush() == "N20 G1 X2\n", "flush emits the last line");
}

void test_strip_line_number()
{
    require(strip_line_number("N120 G1 X1") == "G1 X1", "number removed");
    require(strip_line_number("N10 ;TYPE:Perimeter") == ";TYPE:Perimeter", "number before a comment removed");
    require(strip_line_number("N10") == "", "number alone removed");
    require(strip_line_number("G1 X1") == "G1 X1", "plain line kept");
    require(strip_line_number("NOTE") == "NOTE", "word starting with N kept");
}

void test_standard_lines_are_left_to_the_base()
{
    require(!translate_line("G1 X1 Y2 E0.5", k_axes, {}), "a Marlin move is standard");
    require(!translate_line("M104 S200", k_axes, {}), "standard heater command");
    require(!translate_line(";TYPE:Perimeter", k_axes, {}), "comment");
    require(!translate_line("M601", k_axes, {}), "pause is read by the base");
}

void test_heaters_and_fans()
{
    std::vector<ReaderAction> a = translate("M104=215");
    require(a.size() == 1 && a[0].kind == Kind::Standard && a[0].text == "M104 S215 T0", "M104=");
    a = translate("M124=190");
    require(a[0].text == "M104 S190 T1", "M124= targets the second tool");
    a = translate("M140=60");
    require(a[0].text == "M140 S60", "M140=");
    a = translate("M141 ; switch off bed");
    require(a[0].text == "M140 S0", "M141 switches the bed off");
    a = translate("M109");
    require(a[0].kind == Kind::WaitTool && a[0].tool == 0, "bare M109 waits for tool 0");
    a = translate("M129");
    require(a[0].kind == Kind::WaitTool && a[0].tool == 1, "bare M129 waits for tool 1");
    a = translate("M190");
    require(a[0].kind == Kind::WaitBed, "bare M190 waits for the bed");

    a = translate("M126=255");
    require(a[0].text.rfind("M106 P1 S255", 0) == 0, "M126= is the fan of tool 1 (0..255)");
    ReaderModes percent;
    percent.fan_percentage = true;
    a = translate("M106=50", percent);
    require(a[0].text.rfind("M106 P0 S127.5", 0) == 0, "fan_percentage values are scaled to 0..255");

    a = translate("M104=abc");
    require(a[0].kind == Kind::Unknown, "invalid value");
}

void test_set_position_and_units()
{
    std::vector<ReaderAction> a = translate("B[SET_POSITION POS=0.8]");
    require(a.size() == 1 && a[0].kind == Kind::SetPosition && a[0].tool == 1 && a[0].value == 0.8,
            "SET_POSITION of axis B");
    a = translate("C[SET_POSITION POS=0]");
    require(a[0].kind == Kind::Unknown, "unknown axis");
    a = translate("G71");
    require(a[0].text == "G21", "G71 selects millimetres");
    a = translate("G4 X1.5");
    require(a[0].kind == Kind::Neutral, "G4 X is a dwell");
}

void test_subprogram_call()
{
    // The NEX01 toolchange_gcode calls the extruder switch subprogram.
    std::vector<ReaderAction> a = translate("L C:\\NEX01_sys\\Standard\\switchToExtruder1AtSamePos-sub.nc");
    require(a.size() == 1 && a[0].kind == Kind::Neutral, "a subprogram call keeps the tracked state");
    a = translate("L C:\\NEX01_sys\\Standard\\switchToExtruder2AtSamePos-sub.nc ; switch");
    require(a.size() == 1 && a[0].kind == Kind::Neutral, "a subprogram call followed by a comment");
}

void test_profile_controller_commands()
{
    // The NematX start and end scripts set up the controller without moving the machine.
    for (const char *line : { "G133 = 1", "G133=1", "#BCS OFF", "# SLOPE[TYPE=HSC]", "Z[COMP ON PLANE]", "#BCS ON",
                              "M29 ; end part" }) {
        const std::vector<ReaderAction> a = translate(line);
        require(a.size() == 1 && a[0].kind == Kind::Neutral, std::string("controller command is neutral: ") + line);
    }
    // A bracket command on an extruder axis may move it: the standard reader reports it as unknown.
    require(!translate_line("A[COMP ON PLANE]", k_axes, {}).has_value(), "an extruder axis bracket command is not neutral");
    std::vector<ReaderAction> a = translate("G133 = 1 X5");
    require(a.size() == 1 && a[0].kind == Kind::Neutral, "G133 with any parameters is neutral");
}

void test_absolute_moves()
{
    ReaderModes modes;
    modes.xyz_absolute = true;
    modes.e_absolute = true;
    modes.active_tool = 0;
    std::vector<ReaderAction> a = translate("G1 X10 Y5 A0.42 F1200", modes);
    require(a.size() == 1 && a[0].text == "G1 X10 Y5 E0.42 F1200", "A becomes E for the active tool");

    a = translate("G1 X10 B0.42", modes);
    require(a.size() == 2 && a[0].kind == Kind::SelectTool && a[0].tool == 1 && a[1].text == "G1 X10 E0.42",
            "axis B selects the second tool");

    a = translate("G2 X10 Y0 I5 J0 A1", modes);
    require(a.back().text == "G2 X10 Y0 I5 J0 E1", "arcs keep I and J");
}

void test_relative_moves()
{
    // State after a previous "G1 G90 X.. G91 A..": relative XYZ, relative E.
    ReaderModes modes;
    modes.xyz_absolute = false;
    modes.e_absolute = false;
    modes.active_tool = 0;
    std::vector<ReaderAction> a = translate("G1 G90 X10 Y5 G91 A0.3 F1200", modes);
    require(a.size() == 3, "mode before, move, mode after");
    require(a[0].kind == Kind::XyzMode && a[0].absolute, "XY are absolute");
    require(a[1].text == "G1 X10 Y5 E0.3 F1200", "translated move");
    require(a[2].kind == Kind::XyzMode && !a[2].absolute, "G91 stays in force after the line");

    // A retraction: only the extruder axis, already relative.
    a = translate("G1 G91 A-0.2 F2400", modes);
    require(a.size() == 1 && a[0].text == "G1 E-0.2 F2400", "relative retraction");

    // From an absolute state, the first relative distance switches E to relative.
    modes.xyz_absolute = true;
    modes.e_absolute = true;
    a = translate("G1 G90 X1 G91 A0.1", modes);
    require(a.size() == 3 && a[0].kind == Kind::ExtrusionMode && !a[0].absolute, "E becomes relative");
    require(a[2].kind == Kind::XyzMode && !a[2].absolute, "XYZ relative after the line");

    // A travel keeps the absolute XY mode.
    modes.xyz_absolute = false;
    a = translate("G1 G90 X3 Y4 F9000", modes);
    require(a.size() == 2 && a[0].kind == Kind::XyzMode && a[0].absolute && a[1].text == "G1 X3 Y4 F9000",
            "travel");
}

// The 5 mm cube at the centre of the plater, as written by the legacy slicer
// (except the stray comma after "plater:{", which is removed).
ObjectMetadata cube_metadata()
{
    ObjectMetadata cube;
    cube.name = "cube_5mm_stl id:0 copy 0";
    cube.center = { 100, 100 };
    cube.box.merge(Point3{ 97.5, 97.5, 0 });
    cube.box.merge(Point3{ 102.5, 102.5, 5 });
    cube.outline = { { 102.5, 102.5 }, { 97.5, 102.5 }, { 97.5, 97.5 }, { 102.5, 97.5 } };
    return cube;
}

void test_object_metadata()
{
    require(format_objects_metadata({}).empty(), "no object, no metadata");
    require(format_objects_metadata({ cube_metadata() }) ==
                "\n"
                "; object:{\"name\":\"cube_5mm_stl id:0 copy 0\",\"id\":\"0\",\"object_id\":0,\"copy\":0,"
                "\"object_center\":[100.000,100.000,0.000],\"boundingbox_center\":[100.000,100.000,2.500],"
                "\"boundingbox_size\":[5.000,5.000,5.000],\"scale\":[1.000,1.000,1.000],"
                "\"outline\":[[102.500,102.500],[97.500,102.500],[97.500,97.500],[102.500,97.500]]}\n"
                "; plater:{\"object_center\":[100.000,100.000,0.000],\"boundingbox_center\":[100.000,100.000,2.500],"
                "\"boundingbox_size\":[5.000,5.000,5.000],"
                "\"outline\":[[97.500,97.500],[102.500,97.500],[102.500,102.500],[97.500,102.500]]}\n"
                "\n",
            "cube metadata, same text as the legacy slicer");

    // Use a comma without relying on an installed system locale.
    struct CommaDecimal : std::numpunct<char> {
        char do_decimal_point() const override { return ','; }
    };
    const std::locale previous = std::locale::global(std::locale(std::locale::classic(), new CommaDecimal));
    ObjectMetadata localized = cube_metadata();
    localized.center = { -1.2346, 2.3454 };
    const std::string localized_text = format_object_metadata(localized);
    std::locale::global(previous);
    require(localized_text.find("\"object_center\":[-1.235,2.345,0.000]") != std::string::npos,
            "metadata rounds to three decimals and uses a dot even in a comma locale");

    // A second cube: the plater covers both, its outline is their convex hull.
    ObjectMetadata second = cube_metadata();
    second.id = 1;
    for (Point2 &point : second.outline)
        point[0] += 20;
    second.box = Box3{};
    second.box.merge(Point3{ 117.5, 97.5, 0 });
    second.box.merge(Point3{ 122.5, 102.5, 10 });
    const std::string two = format_objects_metadata({ cube_metadata(), second });
    require(two.find("; plater:{\"object_center\":[110.000,100.000,0.000],\"boundingbox_center\":[110.000,100.000,5.000],"
                     "\"boundingbox_size\":[25.000,5.000,10.000],"
                     "\"outline\":[[97.500,97.500],[122.500,97.500],[122.500,102.500],[97.500,102.500]]}") !=
                std::string::npos,
            "plater of two objects");

    // A round object with many outline points: the lines are cut, and the
    // JSON is the concatenation of the lines without their "; " prefix.
    ObjectMetadata round = cube_metadata();
    round.outline.clear();
    for (int i = 0; i < 800; ++i)
        round.outline.push_back({ 100 + 50 * std::cos(i * 0.00785), 100 + 50 * std::sin(i * 0.00785) });
    for (const std::string &text : { format_object_metadata(round), format_objects_metadata({ round }) }) {
        std::istringstream lines(text);
        std::string line, json;
        size_t count = 0;
        while (std::getline(lines, line)) {
            if (line.empty())
                continue;
            ++count;
            // Each line ends at the first point after the limit: at most one point more.
            require(line.size() <= k_plater_line_limit + 30, "line longer than the controller accepts");
            require(line.rfind("; ", 0) == 0, "every line is a comment");
            if (line.rfind("; plater:", 0) == 0)
                json += "\n";
            json += line.substr(2);
        }
        require(count > 2, "a long outline is cut");
        require(json.find(",]") == std::string::npos && json.find("[,") == std::string::npos &&
                    json.find(",,") == std::string::npos,
                "no empty element where a line was cut");
    }
    // The object line is cut after 3000 characters, the plater line after 4000.
    const std::string object_text = format_object_metadata(round);
    require(object_text.find('\n') > k_object_line_limit && object_text.find('\n') < k_object_line_limit + 30,
            "object line cut just after its limit");
}

void test_convex_hull()
{
    const std::vector<Point2> hull = convex_hull({ { 0, 0 }, { 2, 0 }, { 1, 1 }, { 2, 2 }, { 0, 2 }, { 1, 0 }, { 0, 0 } });
    require(hull == std::vector<Point2>{ { 0, 0 }, { 2, 0 }, { 2, 2 }, { 0, 2 } },
            "hull drops inner, collinear and repeated points, counter-clockwise from the lowest x");
    const Point2 center = polygon_centroid({ { 0, 0 }, { 4, 0 }, { 4, 2 }, { 0, 2 } });
    require(center == Point2{ 2, 1 }, "centroid of a rectangle");
}

} // namespace

int main()
{
    try {
        test_pressure_settings();
        test_metadata_separator();
        test_axis_letters();
        test_output_filter();
        test_strip_line_number();
        test_standard_lines_are_left_to_the_base();
        test_heaters_and_fans();
        test_set_position_and_units();
        test_subprogram_call();
    test_profile_controller_commands();
        test_absolute_moves();
        test_relative_moves();
        test_object_metadata();
        test_convex_hull();
    } catch (const std::exception &error) {
        std::cerr << "FAILED: " << error.what() << "\n";
        return 1;
    }
    std::cout << "All NematX syntax tests passed.\n";
    return 0;
}
