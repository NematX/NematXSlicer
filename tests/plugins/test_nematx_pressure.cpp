#include <catch2/catch.hpp>

#include <cmath>
#include <cstdlib>
#include <limits>
#include <map>
#include <sstream>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/log/core.hpp>
#include <boost/log/sinks/sync_frontend.hpp>
#include <boost/log/sinks/text_ostream_backend.hpp>
#include <boost/make_shared.hpp>

#include "plugin_test_helpers.hpp"
#include "gcode_test_helpers.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/GCodeReaderService.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/Geometry/ArcWelder.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Steps/StepGenerateGcode.hpp"
#include "libslic3r/Utils.hpp"

// Exercises the external NematX firmware through the real provider and file
// writer and NematX Reader. The environment is mandatory so a missing package
// fails visibly. Analytical scripts isolate phase math; generated paths exercise
// movement hints, pending fan commands and nominal accounting. Every export
// owns a fresh provider session; failure checks invoke the real step directly.
//     prepare -> PrintingPlan scripts -> provider ABI -> writer -> Reader/Flow
//     annotations -> independent analytical checks of emitted phases
//     failure test -> preexisting final file -> step -> unchanged file and no tmp

using namespace Slic3r;
using namespace Slic3r::Test::GCode;
using namespace Slic3r::Printing;

namespace {
// Captures the existing application journal during one failure assertion and
// restores the logging level and clears cancellation on scope exit, including
// when an assertion or export throws.
struct FailureJournal
{
    typedef boost::log::sinks::synchronous_sink<boost::log::sinks::text_ostream_backend> Sink;
    boost::shared_ptr<std::ostringstream> messages = boost::make_shared<std::ostringstream>();
    boost::shared_ptr<Sink> sink = boost::make_shared<Sink>();
    unsigned previous_level = get_logging_level();
    // Attaches an owned synchronous sink and enables error messages for this scope.
    FailureJournal()
    {
        sink->locked_backend()->add_stream(messages);
        boost::log::core::get()->add_sink(sink);
        set_logging_level(previous_level < 2 ? 2 : previous_level);
        Orchestrator::instance().reset_plugin_cancel();
    }
    // Removes this sink, restores the prior logging level and clears cancellation.
    ~FailureJournal()
    {
        boost::log::core::get()->remove_sink(sink);
        set_logging_level(previous_level);
        Orchestrator::instance().reset_plugin_cancel();
    }
};
// Set both configuration snapshots: the step selects from full config, while SDK
// Print::config reads PrintConfig, including its plugin-defined dynamic options.
void set_setting(Print &print, const char *key, const std::string &value);
// Copies settings and creates one layer with known starting XYZ/E and F. No
// slicing is needed; the real writer still traverses every firmware boundary.
void prepare(Print &print, const std::string &k, const std::string &split,
             bool relative, const std::string &body);
// Reads actual inline annotations into owned fields and rejects missing fields.
std::vector<std::map<std::string, std::string>> annotations(const std::string &text);
// Appends owned raw text, or an explicitly requested placeholder script, to a scope.
void add_script(PrintingScopeEvents &events, const std::string &text, bool before = true, bool resolve = false);
// Checks the real Reader's final tool-axis coordinate and X endpoint.
void check_closed(Print &print, const std::string &text, double e, double x);
// Collects actual movement rows for one extrusion letter, preserving every byte.
std::string axis_moves(const std::string &text, char axis);

void set_setting(Print &print, const char *key, const std::string &value)
{
    const_cast<DynamicPrintConfig &>(print.full_print_config()).set_deserialize(key, value);
    const_cast<PrintConfig &>(print.config()).set_deserialize(key, value);
}

void prepare(Print &print, const std::string &k, const std::string &split,
             bool relative, const std::string &body)
{
    REQUIRE(std::getenv("SLIC3R_TEST_EXTERNAL_PLUGIN_REPOSITORY") != nullptr);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    configure_standard_firmware(print);
    PrintConfig &native = const_cast<PrintConfig &>(print.config());
    native.use_relative_e_distances.value = relative;
    native.machine_limits_usage.value = MachineLimitsUsage::Ignore;
    set_setting(print, "extruder_pressure_factor", k);
    set_setting(print, "split_extrusion_acceleration", split);
    set_setting(print, "use_relative_e_distances", relative ? "1" : "0");
    set_setting(print, "machine_limits_usage", "ignore");
    select_printing_plan_firmware(print, "nematx");
    PrintingPlan &plan = print.mutable_printing_plan();
    add_script(plan.events, std::string("G71\nG90\nG1 G90 X0 Y0 Z0.2 ") +
        (relative ? "G91" : "G90") + " A0 F6000\nA[SET_POSITION POS=0]\n");
    plan.groups.emplace_back();
    plan.groups.back().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.back().layers.back();
    layer.print_z = scale_i(0.2);
    add_script(layer.events, body);
}

std::vector<std::map<std::string, std::string>> annotations(const std::string &text)
{
    std::vector<std::map<std::string, std::string>> result;
    std::istringstream rows(text);
    std::string row;
    while (std::getline(rows, row)) {
        const size_t at = row.find(";NX_PRESSURE ");
        if (at == std::string::npos) continue;
        std::istringstream words(row.substr(at + 13));
        std::map<std::string, std::string> fields;
        std::string word;
        while (words >> word) {
            const size_t equal = word.find('=');
            REQUIRE(equal != std::string::npos);
            REQUIRE(fields.emplace(word.substr(0, equal), word.substr(equal + 1)).second);
        }
        for (const char *key : {"v", "tool", "phase", "v0_mm_s", "v1_mm_s", "e0_mm_s", "e1_mm_s",
                                "dt_s", "k_s", "de_nom_mm", "de_pressure_mm"})
            REQUIRE(fields.count(key) == 1);
        CHECK(fields.at("v") == "1");
        result.push_back(std::move(fields));
    }
    return result;
}

void add_script(PrintingScopeEvents &events, const std::string &text, bool before, bool resolve)
{
    ExtrusionNop script(ExtrusionPropertyCustomGcodeText(resolve ? ExtrusionPropertyCustomGcodeText::Code::SCRIPT :
                                                               ExtrusionPropertyCustomGcodeText::Code::GCODE, text));
    if (before) events.append_before(script);
    else events.append_after(script);
}

void check_closed(Print &print, const std::string &text, double e, double x)
{
    GCodeReaderService reader(Orchestrator::instance(), &print);
    const slic3r_api::GCodeStateTracker initial = reader.initial_state("gcode.reader.nematx");
    const slic3r_api::GCodeStateTracker final = reader.read("gcode.reader.nematx", initial.state(), text);
    REQUIRE(final.x());
    CHECK(*final.x() == Approx(x));
    REQUIRE(final.e(0));
    CHECK(*final.e(0) == Approx(e));
    CHECK(text.find("NX_INTERNAL_MOVE") == std::string::npos);
}

std::string axis_moves(const std::string &text, char axis)
{
    std::string output, line;
    std::istringstream lines(text);
    while (std::getline(lines, line))
        if (line.find("G1 ") == 0 && line.find(std::string(" ") + axis) != std::string::npos)
            output += line + '\n';
    return output;
}
} // namespace

// Require a real external selector value and the NematX startup/metadata bytes.
TEST_CASE("NematX pressure external selector selects the real provider",
          "[plugins][nematx-pressure][plugin-e2e][.]")
{
    REQUIRE(std::getenv("SLIC3R_TEST_EXTERNAL_PLUGIN_REPOSITORY") != nullptr);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    REQUIRE(Orchestrator::instance().get_plugin("nematx") != nullptr);
    REQUIRE(Orchestrator::instance().get_plugin("gcode.reader.nematx") != nullptr);
    Print print;
    configure_standard_firmware(print);
    print.mutable_printing_plan();
    DynamicPrintConfig &config = const_cast<DynamicPrintConfig &>(print.full_print_config());
    REQUIRE_NOTHROW(config.set_deserialize("gcode_flavor", "nematx"));
    CHECK(config.opt_serialize("gcode_flavor") == "nematx");
    const std::string output = export_with_firmware(print, "nematx");
    CHECK(output.find("G71\n") != std::string::npos);
    CHECK(output.find("; total layers count") == std::string::npos);
}

// Check independently calculated 50/100/50 mm phases and the discriminating 2 mm charge.
TEST_CASE("NematX pressure analytical phases close in absolute and relative extrusion",
          "[plugins][nematx-pressure][plugin-e2e][.]")
{
    for (bool relative : {false, true}) {
        for (bool pressure : {false, true}) {
            CAPTURE(relative, pressure);
            Print print;
            prepare(print, pressure ? "0.02" : "0", "1", relative,
                std::string(";M204 S100\nG1 ") + (relative ? "G90 X200 G91 A8" : "X200 A8") + " F6000 ; source\n");
            const std::string output = export_with_firmware(print, "nematx");
            const std::vector<std::map<std::string, std::string>> phases = annotations(output);
            REQUIRE(phases.size() == 3);
            CHECK(phases[0].at("phase") == "accel");
            CHECK(phases[1].at("phase") == "steady");
            CHECK(phases[2].at("phase") == "decel");
            CHECK(std::stod(phases[0].at("v1_mm_s")) == Approx(100));
            CHECK(std::stod(phases[0].at("dt_s")) == Approx(1));
            CHECK(std::stod(phases[1].at("dt_s")) == Approx(1));
            CHECK(std::stod(phases[2].at("dt_s")) == Approx(1));
            CHECK(std::stod(phases[0].at("de_nom_mm")) == Approx(2));
            CHECK(std::stod(phases[0].at("de_pressure_mm")) == Approx(pressure ? 2.0 : 0.0));
            CHECK(std::stod(phases[2].at("de_pressure_mm")) == Approx(pressure ? -2.0 : 0.0));
            CHECK(count_occurrences(output, "; source") == 1);
            GCodeReaderService reader(Orchestrator::instance(), &print);
            const GCodeProcessorResult actual = reader.read_for_viewer("gcode.reader.nematx", output);
            bool first_phase = false;
            for (const GCodeProcessorResult::MoveVertex &move : actual.moves) {
                if (move.position.x() == Approx(50)) {
                    first_phase = true;
                    CHECK(move.delta_extruder == Approx(pressure ? 4 : 2));
                }
            }
            CHECK(first_phase);
            check_closed(print, output, 8, 200);
        }
    }
}

// Compare full repeated neutral exports, including invalid but inert directives.
TEST_CASE("NematX neutral export and inactive compressibility preserve exact bytes",
          "[plugins][nematx-pressure][plugin-e2e][.]")
{
    Print print;
    const std::string source = "; PRESSURE_FACTOR=invalid\n;M204 Sinvalid\nG1 X200 A8 F6000 ; unchanged\n";
    prepare(print, "0", "0", false, source);
    const std::string off = export_with_firmware(print, "nematx");
    const std::string expected = "G71\nG90\nG71\nG90\n"
        "G1 G90 X0 Y0 Z0.2 G90 A0 F6000\nA[SET_POSITION POS=0]\n"
        ";LAYER_CHANGE\n;Z:0.2\n;MAX_LAYER_HEIGHT:0.2\n" + source;
    CHECK(off == expected);
    CHECK(off.find(source) != std::string::npos);
    CHECK(off.find("NX_PRESSURE") == std::string::npos);
    set_setting(print, "filament_compressibility_factor", "3.5");
    CHECK(export_with_firmware(print, "nematx") == off);
    CHECK(export_with_firmware(print, "nematx") == off);
    check_closed(print, off, 8, 200);
    set_setting(print, "use_volumetric_e", "1");
    const std::string volumetric_off = export_with_firmware(print, "nematx");
    CHECK(volumetric_off.find(source) != std::string::npos);
    CHECK(volumetric_off.find("NX_PRESSURE") == std::string::npos);
}

// Keep a preexisting output witness across deliberate real-step failures and
// require the intended cause in the application journal, not the generic Step error.
TEST_CASE("NematX pressure failures never replace a published file",
          "[plugins][nematx-pressure][plugin-e2e][.]")
{
    Print print;
    std::string body = "G1 X200 A8 F6000\n";
    std::string k = "0.02";
    bool volume = false;
    bool preamble = false;
    bool nonfinite_k = false;
    bool nonfinite_limit = false;
    std::string cause;
    std::size_t source_line = 0;
    SECTION("Reader arc") { body = "G2 X20 Y0 I10 J0 A1 F6000\n"; cause = "active arc is not supported"; source_line = 12; }
    SECTION("volumetric config") { volume = true; cause = "volumetric extrusion is not treated"; source_line = 12; }
    SECTION("M200 startup guard") { preamble = true; body = "M200 D0\n"; cause = "volumetric extrusion is not treated"; source_line = 13; }
    SECTION("negative K") { k = "-0.1"; cause = "extruder_pressure_factor must be finite and nonnegative, tool 0"; }
    SECTION("nonfinite K") { k = "0"; nonfinite_k = true; cause = "extruder_pressure_factor must be finite and nonnegative, tool 0"; }
    SECTION("nonfinite machine limit") { nonfinite_limit = true; cause = "requires finite machine_max_feedrate_x"; }
    SECTION("invalid override before layer one") { preamble = true; body = "; PRESSURE_FACTOR=invalid\n"; cause = "expected a complete finite nonnegative decimal field"; source_line = 9; }
    SECTION("unknown coordinates") { body = "M999\nG1 X200 A8 F6000\n"; cause = "required pre-motion modes are unknown"; source_line = 13; }
    SECTION("relative XYZ") { body = "G91\nG1 X200 A8 F6000\n"; cause = "printing requires absolute XYZ and millimetres"; source_line = 13; }
    SECTION("inches") { body = "G20\nG1 X200 A8 F6000\n"; cause = "printing requires absolute XYZ and millimetres"; source_line = 13; }
    prepare(print, k, "0", false, preamble ? "G1 X200 A8 F6000\n" : body);
    add_script(print.mutable_printing_plan().events, "; prior record one\n; prior record two\n");
    if (nonfinite_k) {
        ConfigOptionFloats *factor = const_cast<PrintConfig &>(print.config()).option<ConfigOptionFloats>("extruder_pressure_factor");
        REQUIRE(factor != nullptr);
        factor->set(std::vector<double>{std::numeric_limits<double>::quiet_NaN()});
        REQUIRE(std::isnan(factor->get_float(0)));
    }
    if (nonfinite_limit) {
        set_setting(print, "machine_limits_usage", "time_estimate_only");
        const_cast<PrintConfig &>(print.config()).machine_max_feedrate_x.set(
            std::vector<double>{std::numeric_limits<double>::infinity()});
        REQUIRE(std::isinf(print.config().machine_max_feedrate_x.get_float(0)));
    }
    if (preamble) add_script(print.mutable_printing_plan().events, body);
    if (volume) {
        const_cast<PrintConfig &>(print.config()).use_volumetric_e.value = true;
        set_setting(print, "use_volumetric_e", "1");
    }
    const boost::filesystem::path path = temporary_gcode_path();
    write_text_file(path, "previous output\n");
    try {
        FailureJournal journal;
        CHECK_THROWS(Steps::StepGenerateGcode::run_step(Orchestrator::instance(), print, path.string()));
        const std::string logged = journal.messages->str();
        INFO(logged);
        REQUIRE_FALSE(cause.empty());
        CHECK(logged.find(cause) != std::string::npos);
        if (source_line != 0) {
            CHECK(logged.find("NematX pressure source line " + std::to_string(source_line) + ": ") != std::string::npos);
            if (body != "; PRESSURE_FACTOR=invalid\n")
                CHECK(logged.find(", tool 0: " + cause) != std::string::npos);
        }
        CHECK(read_text_file(path) == "previous output\n");
        CHECK_FALSE(boost::filesystem::exists(path.string() + ".tmp"));
    } catch (...) { remove_output_pair(path); throw; }
    remove_output_pair(path);
}

// Literal private-looking comments, including a valid travel hint, must not
// suppress pressure classification in raw text or resolved placeholder scripts.
TEST_CASE("NematX pressure preserves literal movement hint lookalikes in scripts",
          "[plugins][nematx-pressure][plugin-e2e][.]")
{
    const std::string literals = ";NX_INTERNAL_MOVE=extrusion\n;NX_INTERNAL_MOVE=malformed\n;NX_INTERNAL_MOVE=travel\n";
    for (bool resolved : {false, true}) {
        CAPTURE(resolved);
        Print print;
        prepare(print, "0.02", "1", false, "");
        add_script(print.mutable_printing_plan().groups.front().layers.front().events,
            ";M204 S100\n" + literals + "G1 X200 A8 F6000\n", true, resolved);
        const std::string output = export_with_firmware(print, "nematx");
        CHECK(output.find(literals) != std::string::npos);
        CHECK(count_occurrences(output, "NX_INTERNAL_MOVE=") == 3);
        const std::vector<std::map<std::string, std::string>> phases = annotations(output);
        REQUIRE(phases.size() == 3);
        CHECK(std::stod(phases[0].at("de_pressure_mm")) == Approx(2));
        CHECK(std::stod(phases[2].at("de_pressure_mm")) == Approx(-2));
        set_setting(print, "gcode_no_comment", "1");
        const std::string uncommented = export_with_firmware(print, "nematx");
        CHECK(uncommented.find("NX_PRESSURE") == std::string::npos);
        CHECK(count_occurrences(uncommented, "G1 X") == 3);
        check_closed(print, uncommented, 8, 200);
    }
}

// A low X limit must alter phases even when it is not emitted to the controller.
TEST_CASE("NematX pressure uses time estimate limits but ignores disabled limits",
          "[plugins][nematx-pressure][plugin-e2e][.]")
{
    for (bool use_limits : {false, true}) {
        Print print;
        prepare(print, "0.02", "1", false, ";M204 S100\nG1 X200 A8 F6000\n");
        configure_machine_envelope(print);
        PrintConfig &native = const_cast<PrintConfig &>(print.config());
        native.machine_limits_usage.value = use_limits ? MachineLimitsUsage::TimeEstimateOnly : MachineLimitsUsage::Ignore;
        native.machine_max_feedrate_x.set(std::vector<double>{20.0});
        set_setting(print, "machine_limits_usage", use_limits ? "time_estimate_only" : "ignore");
        set_setting(print, "machine_max_feedrate_x", "20");
        const std::string output = export_with_firmware(print, "nematx");
        const std::vector<std::map<std::string, std::string>> phases = annotations(output);
        REQUIRE(phases.size() == 3);
        CHECK(std::stod(phases[0].at("v1_mm_s")) == Approx(use_limits ? 20 : 100));
        CHECK(std::stod(phases[0].at("dt_s")) == Approx(use_limits ? 0.2 : 1));
        check_closed(print, output, 8, 200);
    }
}

// Capture the existing logger while an invalid acceleration retains the previous value.
TEST_CASE("NematX pressure acceleration diagnostics reach the application journal",
          "[plugins][nematx-pressure][plugin-e2e][.]")
{
    Print print;
    prepare(print, "0.02", "1", false, ";M204 S100\n;M204 Sinvalid\nG1 X200 A8 F6000\n");
    typedef boost::log::sinks::synchronous_sink<boost::log::sinks::text_ostream_backend> LogSink;
    const boost::shared_ptr<std::ostringstream> messages = boost::make_shared<std::ostringstream>();
    const boost::shared_ptr<LogSink> sink = boost::make_shared<LogSink>();
    sink->locked_backend()->add_stream(messages);
    boost::log::core::get()->add_sink(sink);
    const unsigned previous_level = get_logging_level();
    set_logging_level(previous_level < 2 ? 2 : previous_level);
    std::string output;
    try { output = export_with_firmware(print, "nematx"); }
    catch (...) { set_logging_level(previous_level); boost::log::core::get()->remove_sink(sink); throw; }
    set_logging_level(previous_level);
    sink->flush();
    boost::log::core::get()->remove_sink(sink);
    CHECK(messages->str().find("invalid ;M204 S acceleration ignored, tool 0, source line") != std::string::npos);
    const std::vector<std::map<std::string, std::string>> phases = annotations(output);
    REQUIRE(phases.size() == 3);
    CHECK(std::stod(phases[0].at("v1_mm_s")) == Approx(100));
}

// Assert contiguous numbering after rewriting.
TEST_CASE("NematX pressure output filtering numbers only final lines",
          "[plugins][nematx-pressure][plugin-e2e][.]")
{
    Print print;
    prepare(print, "0.02", "1", true, ";M204 S100\nG1 G90 X200 G91 A8 F6000\n");
    set_setting(print, "gcode_line_number", "1");
    set_setting(print, "gcode_no_comment", "1");
    const std::string output = export_with_firmware(print, "nematx");
    CHECK(output.find("NX_PRESSURE") == std::string::npos);
    CHECK(output.find("NX_INTERNAL") == std::string::npos);
    std::istringstream lines(output);
    std::string line;
    size_t number = 0;
    size_t metadata = 0;
    while (std::getline(lines, line)) {
        if (line.empty()) continue;
        if (line[0] == ';') { ++metadata; continue; }
        CHECK(metadata == 0);
        number += 10;
        CHECK(line.substr(0, line.find(' ')) == "N" + std::to_string(number));
        CHECK(line.find(';') == std::string::npos);
    }
    // The session writes no controller metadata after the numbered block any more.
    CHECK(metadata == 0);
    CHECK(number > 0);
    check_closed(print, output, 8, 200);
}

// Compare real producer accounting and Reader endpoints with pressure on/off.
TEST_CASE("NematX generated paths retain nominal accounting and semantic commands",
          "[plugins][nematx-pressure][plugin-e2e][.]")
{
    std::string nominal_volume;
    std::vector<std::string> nominal_controls;
    double nominal_e = 0;
    size_t nominal_moves = 0;
    for (bool enabled : {false, true}) {
        Print print;
        prepare(print, enabled ? "0.02" : "0", "0", true, "");
        PrintingPlan &plan = print.mutable_printing_plan();
        PrintingLayerGroup &layer = plan.groups.front().layers.front();
        layer.tool_groups.emplace_back();
        PrintingToolGroup &tool = layer.tool_groups.back();
        tool.extruder_id = 0;
        append_path_extrusion(tool, make_firmware_path(
            ArcPolyline(Points{Point::new_scale(0, 0), Point::new_scale(200, 0)}), 100.f, 100.f));
        append_path_extrusion(tool, special_command_entity(ExtrusionPropertySpecialCommand::Code::RETRACT, -0.8));
        append_path_extrusion(tool, make_firmware_travel(
            ArcPolyline(Points{Point::new_scale(200, 0), Point::new_scale(205, 0)}), 100.f, 100.f));
        std::unique_ptr<ExtrusionPath> wipe = make_firmware_travel(
            ArcPolyline(Points{Point::new_scale(205, 0), Point::new_scale(210, 0)}), 40.f, 100.f);
        slic3r_api::MutableExtrusionEntity wipe_view(reinterpret_cast<extrusion_entity_handle *>(wipe.get()));
        wipe_view.get_mutable(slic3r_api::EPropertyAttributes::key)->extrusion_role(
            RAW_EXTRUSION_ROLE_TRAVEL | RAW_EXTRUSION_ROLE_WIPE);
        append_path_extrusion(tool, std::move(wipe));
        append_path_extrusion(tool, special_command_entity(ExtrusionPropertySpecialCommand::Code::RETRACT, 0.8));
        append_path_extrusion(tool, make_firmware_path(
            ArcPolyline(Points{Point::new_scale(210, 0), Point::new_scale(410, 0)}), 100.f, 100.f));
        for (PrintingExtrusion &extrusion : tool.extrusions)
            extrusion.root = extrusion.root->clone_canonical();
        add_script(plan.events, "; nominal_volume={extruded_volume[0]}\n; nominal_retracted={e_retracted[0]}\n", false, true);
        const std::string output = export_with_firmware(print, "nematx");
        INFO(output);
        CHECK(output.find("NX_PRESSURE") == std::string::npos);
        CHECK(output.find("NX_INTERNAL") == std::string::npos);
        const size_t volume_at = output.find("; nominal_volume=");
        REQUIRE(volume_at != std::string::npos);
        const size_t end_at = output.find('\n', output.find("; nominal_retracted=", volume_at));
        REQUIRE(end_at != std::string::npos);
        const std::string volume = output.substr(volume_at, end_at - volume_at);
        const double script_volume = std::stod(output.substr(volume_at + std::string("; nominal_volume=").size()));
        const std::string retracted_key = "; nominal_retracted=";
        CHECK(std::stod(output.substr(output.find(retracted_key) + retracted_key.size())) == Approx(0));
        CHECK(output.find("G1 G91 A-.8") != std::string::npos);
        CHECK(output.find("G1 G91 A.8") != std::string::npos);
        std::vector<std::string> controls;
        std::istringstream lines(output);
        std::string line;
        while (std::getline(lines, line)) {
            if (line.find("SET_POSITION") != std::string::npos ||
                (line.find("G1") == 0 && line.find('X') == std::string::npos &&
                 line.find('Y') == std::string::npos && line.find('Z') == std::string::npos))
                controls.push_back(line);
        }
        GCodeReaderService reader(Orchestrator::instance(), &print);
        const slic3r_api::GCodeStateTracker initial = reader.initial_state("gcode.reader.nematx");
        const slic3r_api::GCodeStateTracker final = reader.read("gcode.reader.nematx", initial.state(), output);
        REQUIRE(final.e(0));
        const GCodeProcessorResult viewer = reader.read_for_viewer("gcode.reader.nematx", output);
        double emitted_filament = 0;
        for (const GCodeProcessorResult::MoveVertex &move : viewer.moves)
            emitted_filament += move.delta_extruder;
        CHECK(emitted_filament * PI * 1.75 * 1.75 / 4 == Approx(script_volume).margin(0.001));
        CHECK(script_volume == Approx(32).margin(0.001));
        CHECK(output.find(";WIPE_START\n") != std::string::npos);
        CHECK(output.find(";WIPE_END\n") != std::string::npos);
        if (!enabled) {
            nominal_volume = volume;
            nominal_controls = controls;
            nominal_e = *final.e(0);
            nominal_moves = count_occurrences(output, "G1 G90 X");
        }
        else {
            CHECK(volume == nominal_volume);
            CHECK(controls == nominal_controls);
            CHECK(*final.e(0) == Approx(nominal_e).margin(0.00001));
            CHECK(count_occurrences(output, "G1 G90 X") > nominal_moves);
        }
        CHECK(nominal_e > 0);
    }
}

// Exercise real generated hints after queued fan commands, logical tool changes
// and shared physical extrusion state without adding test-only session access.
TEST_CASE("NematX generated two tool pressure export handles queued fans and SEMM",
          "[plugins][nematx-pressure][plugin-e2e][.]")
{
    for (bool semm : {false, true}) {
        Print print;
        prepare(print, "0.02,0", "1,0", true, "");
        configure_standard_firmware(print, 2);
        set_setting(print, "nozzle_diameter", "0.4,0.4");
        set_setting(print, "filament_diameter", "1.75,1.75");
        set_setting(print, "extrusion_multiplier", "1,1");
        set_setting(print, "extruder_axis", "\"A\";\"B\"");
        set_setting(print, "extruder_pressure_factor", "0.02,0");
        set_setting(print, "split_extrusion_acceleration", "1,0");
        set_setting(print, "disable_fan_from_unused_extruder", "1");
        set_setting(print, "filament_pressure_advance", "0.03,0.04");
        set_setting(print, "single_extruder_multi_material", semm ? "1" : "0");
        const_cast<PrintConfig &>(print.config()).single_extruder_multi_material.value = semm;
        append_multi_tool_process_sequence(print.mutable_printing_plan().groups.front().layers.front());
        for (PrintingToolGroup &tool : print.mutable_printing_plan().groups.front().layers.front().tool_groups)
            for (PrintingExtrusion &extrusion : tool.extrusions)
                extrusion.root = extrusion.root->clone_canonical();
        set_setting(print, "extruder_pressure_factor", "0,0");
        set_setting(print, "split_extrusion_acceleration", "0,0");
        const std::string neutral = export_with_firmware(print, "nematx");
        set_setting(print, "extruder_pressure_factor", "0.02,0");
        set_setting(print, "split_extrusion_acceleration", "1,0");
        const std::string output = export_with_firmware(print, "nematx");
        CHECK(axis_moves(output, 'B') == axis_moves(neutral, 'B'));
        const std::vector<std::map<std::string, std::string>> phases = annotations(output);
        REQUIRE(phases.size() >= 4);
        for (const std::map<std::string, std::string> &phase : phases)
            CHECK(phase.at("tool") == "0");
        CHECK(output.find(" B") != std::string::npos);
        CHECK(output.find("NX_INTERNAL") == std::string::npos);
        if (!semm) {
            CHECK(output.find("M106=0\n") != std::string::npos);
            CHECK(output.find("M126=0\n") != std::string::npos);
        }
        CHECK(output.find("M900") == std::string::npos);
        CHECK(output.find("M572") == std::string::npos);
        CHECK(output.find("SET_PRESSURE_ADVANCE") == std::string::npos);
    }

    // No new fan, heater or acceleration request may consume the queued command
    // before the second tool's encoder emits its private movement hint.
    Print pending;
    prepare(pending, "0.02,0", "1,0", false, "");
    configure_standard_firmware(pending, 2);
    set_setting(pending, "nozzle_diameter", "0.4,0.4");
    set_setting(pending, "filament_diameter", "1.75,1.75");
    set_setting(pending, "extrusion_multiplier", "1,1");
    set_setting(pending, "extruder_axis", "\"A\";\"B\"");
    set_setting(pending, "disable_fan_from_unused_extruder", "1");
    PrintingPlan &plan = pending.mutable_printing_plan();
    plan.events.clear();
    add_script(plan.events, "A[SET_POSITION POS=0]\nB[SET_POSITION POS=0]\n");
    PrintingLayerGroup &layer = plan.groups.front().layers.front();
    for (int tool = 0; tool < 2; ++tool) {
        layer.tool_groups.emplace_back();
        PrintingToolGroup &group = layer.tool_groups.back();
        group.extruder_id = tool;
        append_path_extrusion(group, make_firmware_path(
            ArcPolyline(Points{Point::new_scale(tool, 0), Point::new_scale(tool + 1, 0)}),
            15.f, 400.f, -1.f, tool == 0 ? 20.f : -1.f));
        group.extrusions.back().root = group.extrusions.back().root->clone_canonical();
    }
    const std::string queued = export_with_firmware(pending, "nematx");
    INFO(queued);
    CHECK(queued.find("M106=0\nG1 ") != std::string::npos);
    CHECK(queued.find("NX_INTERNAL") == std::string::npos);
    CHECK_FALSE(annotations(queued).empty());
}

// Verify the actual viewer accepts corrected negative XY extrusion and records
// its semantic classification, while the actual emitted E cycle still closes.
TEST_CASE("NematX pressure viewer reads actual negative XY deceleration",
          "[plugins][nematx-pressure][plugin-e2e][.]")
{
    Print print;
    prepare(print, "0.02", "1", true, ";M204 S100\nG1 G90 X200 G91 A2 F6000\n");
    const std::string output = export_with_firmware(print, "nematx");
    check_closed(print, output, 2, 200);
    GCodeReaderService reader(Orchestrator::instance(), &print);
    const GCodeProcessorResult viewer = reader.read_for_viewer("gcode.reader.nematx", output);
    REQUIRE(viewer.print_statistics.volumes_per_extruder.count(0) == 1);
    CHECK(viewer.print_statistics.volumes_per_extruder.at(0) == Approx(3.5 * PI * 1.75 * 1.75 / 4));
    bool negative_xy = false;
    for (const GCodeProcessorResult::MoveVertex &move : viewer.moves) {
        if (move.delta_extruder < 0 && move.position.x() == Approx(200)) {
            negative_xy = true;
            CHECK(move.type == EMoveType::Travel);
            CHECK(move.delta_extruder == Approx(-1.5));
        }
    }
    CHECK(negative_xy);
}

// Same-tool sections must preserve their printing junction; wipe and E-only
// commands close it and retain their text. Plan-end movement stays outside layers.
TEST_CASE("NematX pressure boundaries preserve same tool junctions and semantic scripts",
          "[plugins][nematx-pressure][plugin-e2e][.]")
{
    Print print;
    prepare(print, "0.02", "1", false, ";M204 S100\n");
    PrintingPlan &plan = print.mutable_printing_plan();
    PrintingLayerGroup &layer = plan.groups.front().layers.front();
    for (const std::string &body : {std::string("G1 X100 A4 F6000\n"), std::string("G1 X200 A8 F6000\n")}) {
        layer.tool_groups.emplace_back();
        layer.tool_groups.back().extruder_id = 0;
        add_script(layer.tool_groups.back().events, body);
    }
    const std::string semantic = ";WIPE_START\nG1 X201 A7.8 F1200\n;WIPE_END\nG1 A7 F1200\nA[SET_POSITION POS=0.25]\n";
    add_script(layer.events, semantic + "G1 X301 A4.25 F6000\n", false);
    const std::string outside = "G1 X401 A8.25 F6000 ; outside\n";
    add_script(plan.events, outside, false);
    const std::string output = export_with_firmware(print, "nematx");
    const std::vector<std::map<std::string, std::string>> phases = annotations(output);
    REQUIRE(phases.size() == 6);
    CHECK(std::stod(phases[1].at("v1_mm_s")) == Approx(100));
    CHECK(std::stod(phases[2].at("v0_mm_s")) == Approx(100));
    CHECK(std::stod(phases[3].at("v1_mm_s")) == Approx(0));
    CHECK(std::stod(phases[5].at("v1_mm_s")) == Approx(0));
    CHECK(output.find(semantic) != std::string::npos);
    CHECK(output.find(outside) != std::string::npos);
    check_closed(print, output, 8.25, 401);
}

// Distinguish existing tiny-arc G1 encoding from a real emitted Reader arc;
// pressure does not change the encoder's preexisting conversion threshold.
TEST_CASE("NematX pressure treats already linear tiny arcs but rejects real emitted arcs",
          "[plugins][nematx-pressure][plugin-e2e][.]")
{
    for (double chord : {0.006, 20.0}) {
        for (bool enabled : {false, true}) {
            CAPTURE(chord, enabled);
            Print print;
            prepare(print, enabled ? "0.02" : "0", "0", false, "");
            PrintingLayerGroup &layer = print.mutable_printing_plan().groups.front().layers.front();
            layer.tool_groups.emplace_back();
            PrintingToolGroup &tool = layer.tool_groups.back();
            tool.extruder_id = 0;
            Geometry::ArcWelder::Path arc;
            arc.emplace_back(Point::new_scale(0, 0));
            arc.emplace_back(Point::new_scale(chord, 0), float(scale_i(chord / 2)),
                             Geometry::ArcWelder::Orientation::CCW);
            append_path_extrusion(tool, make_firmware_path(ArcPolyline(arc), 1.f, 100.f));
            tool.extrusions.back().root = tool.extrusions.back().root->clone_canonical();
            if (enabled && chord > 0.01) {
                const boost::filesystem::path path = temporary_gcode_path();
                write_text_file(path, "previous output\n");
                try {
                    FailureJournal journal;
                    CHECK_THROWS(Steps::StepGenerateGcode::run_step(Orchestrator::instance(), print, path.string()));
                    CHECK(journal.messages->str().find("tool 0: active arc is not supported") != std::string::npos);
                    CHECK(read_text_file(path) == "previous output\n");
                    CHECK_FALSE(boost::filesystem::exists(path.string() + ".tmp"));
                } catch (...) { remove_output_pair(path); throw; }
                remove_output_pair(path);
            } else {
                const std::string output = export_with_firmware(print, "nematx");
                CHECK((output.find("G3 ") != std::string::npos) == (chord > 0.01));
                CHECK(output.find("NX_INTERNAL") == std::string::npos);
                GCodeReaderService reader(Orchestrator::instance(), &print);
                const slic3r_api::GCodeStateTracker initial = reader.initial_state("gcode.reader.nematx");
                const slic3r_api::GCodeStateTracker final = reader.read("gcode.reader.nematx", initial.state(), output);
                REQUIRE(final.x());
                CHECK(*final.x() == Approx(chord));
            }
        }
    }

    // A generated print whose nominal E rounds away still needs the producer's
    // Extrusion hint; source/Reader-only classification would call it a travel.
    Print rounded;
    prepare(rounded, "0", "1", false, "");
    set_setting(rounded, "gcode_precision_e", "0");
    PrintingLayerGroup &layer = rounded.mutable_printing_plan().groups.front().layers.front();
    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool = layer.tool_groups.back();
    tool.extruder_id = 0;
    append_path_extrusion(tool, make_firmware_path(
        ArcPolyline(Points{Point::new_scale(0, 0), Point::new_scale(10, 0)}), 100.f, 100.f));
    tool.extrusions.back().root = tool.extrusions.back().root->clone_canonical();
    const std::string output = export_with_firmware(rounded, "nematx");
    CHECK(annotations(output).size() == 2);
    check_closed(rounded, output, 0, 10);
}
