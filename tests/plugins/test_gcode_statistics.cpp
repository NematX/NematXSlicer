/*
Tests cover the plan facts and measured statistics of an export: STEP_GCODE facts, layer counts, the availability
flag, the support flag, the statistics block of an exported file, the command-line export and the file name variables.

Print statistics tests
======================

STEP_GCODE publishes the facts of the PrintingPlan (tool changes, initial and used
tools, layer counts) for every export. The measured totals and the availability
flag exist only after the generated G-code was reread into a Reader result. An
exported text file lists the statistics, in the order of the historical file,
between the reference body and the configuration, and only when they are
available.

Expected statistics lines are recomputed here from the Reader volumes and the
configuration with a stream formatter, never from the host formatting. The tool changes are
counted from the `T` commands of the generated body, which is independent of the
plan walk. Test files live in unique directories below the working directory and
are removed at scope exit.

Representative direct calls (branches are not a timeline):

    TEST_CASE
        |-- init_two_objects()
        |-- print.process()
        |-- orchestrator.export_gcode()
        |-- read_file()
        |-- orchestrator.export_print()
        `-- split_lines()
*/

#include <catch2/catch.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iterator>
#include <locale>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <tbb/global_control.h>
#include <boost/filesystem.hpp>
#include <boost/nowide/fstream.hpp>

#include "gcode_export_test_helpers.hpp"
#include "plugin_test_helpers.hpp"
#include "test_data.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/libslic3r.h"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Steps/StepReadGeneratedGcode.hpp"



namespace {

using namespace Slic3r;

class ScopedDirectory
{
public:
    // Create one unique directory below the current working directory for test-owned files.
    ScopedDirectory()
        : m_path(boost::filesystem::current_path() / boost::filesystem::unique_path("slic3r-statistics-%%%%-%%%%-%%%%"))
    {
        boost::filesystem::create_directories(m_path);
    }

    // Remove the directory without throwing during stack unwinding.
    ~ScopedDirectory()
    {
        boost::system::error_code ignored;
        boost::filesystem::remove_all(m_path, ignored);
    }

    // Return the borrowed path owned by this scope.
    const boost::filesystem::path &path() const { return m_path; }

private:
    boost::filesystem::path m_path;
};

// Read one whole file as bytes.
std::string read_file(const boost::filesystem::path &path)
{
    boost::nowide::ifstream input(path.string().c_str(), std::ios::in | std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

// Join the first count values with commas.
std::string join_csv(const std::vector<std::string> &values, size_t count)
{
    std::string joined;
    for (size_t index = 0; index < count; ++index)
        joined += (index == 0 ? "" : ",") + values[index];
    return joined;
}

// Build a configuration with the given number of tools, the simple exporter, the Marlin firmware and distinct filament
// properties per tool (diameter, density, price and type), with a time cost so that the total cost differs from the
// filament cost.
DynamicPrintConfig statistics_config(size_t extruders)
{
    const std::vector<std::string> diameters = {"1.75", "2.85", "1.75", "2.85"};
    const std::vector<std::string> densities = {"1.1", "1.2", "1.3", "1.4"};
    const std::vector<std::string> prices = {"20", "25", "30", "35"};
    const std::vector<std::string> types = {"PLA", "PETG", "ABS", "TPU"};
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_extruders(extruders);
    config.set_deserialize_strict({
        {"gcode_firmware_plugin", "gcode.firmware.marlin2"},
        {"gcode_exporter_plugin", "gcode.exporter.simple"},
        {"binary_gcode", "1"},
        {"wipe_tower", "0"},
        {"single_extruder_multi_material", "0"},
        {"layer_height", "0.3"},
        {"fill_density", "20%"},
        {"time_cost", "2"},
        {"filament_diameter", join_csv(diameters, extruders)},
        {"filament_density", join_csv(densities, extruders)},
        {"filament_cost", join_csv(prices, extruders)},
        {"filament_type", join_csv(types, extruders)}});
    return config;
}

// Enable the default wipe tower manager and a tower at a fixed place.
void enable_wipe_tower(DynamicPrintConfig &config)
{
    config.set_deserialize_strict({{"wipe_tower", "1"}, {"wipe_tower_x", "40"}, {"wipe_tower_y", "60"},
        {"wipe_tower_width", "20"}, {"wipe_tower_rotation_angle", "0"}, {"wipe_tower_brim_width", "0"},
        {"step_wipetower_plugin", "wipe_tower.manager.default"}, {"wiping_volumes_matrix", "0,140,140,0"},
        {"filament_minimal_purge_on_wipe_tower", "15,15"}});
}

// Initialize two cubes printed with the tools given as one-based extruder numbers.
void init_two_objects(Print &print, Model &model, DynamicPrintConfig &config, int first_extruder, int second_extruder)
{
    Test::init_print({Test::TestMesh::cube_20x20x20, Test::TestMesh::cube_20x20x20}, print, model, config);
    for (const char *key : {"perimeter_extruder", "infill_extruder", "solid_infill_extruder"}) {
        model.objects()[0].config.set_deserialize_strict(key, std::to_string(first_extruder));
        model.objects()[1].config.set_deserialize_strict(key, std::to_string(second_extruder));
    }
    print.apply(model, config);
}

// Return the number of changes between consecutive `T` commands of a G-code body; the first selection is not a change.
int count_tool_transitions(const std::string &gcode)
{
    int transitions = 0;
    int previous_tool = -1;
    size_t line_begin = 0;
    while (line_begin < gcode.size()) {
        size_t line_end = gcode.find('\n', line_begin);
        if (line_end == std::string::npos)
            line_end = gcode.size();
        const std::string line = gcode.substr(line_begin, line_end - line_begin);
        line_begin = line_end + 1;
        if (line.size() < 2 || line[0] != 'T' || line[1] < '0' || line[1] > '9')
            continue;
        const int tool = std::atoi(line.c_str() + 1);
        if (previous_tool >= 0 && tool != previous_tool)
            ++transitions;
        previous_tool = tool;
    }
    return transitions;
}

// Format one number with two decimals and a decimal point independent of the system language.
std::string two_decimals(double value)
{
    std::ostringstream output;
    output.imbue(std::locale::classic());
    output << std::fixed << std::setprecision(2) << value;
    return output.str();
}

// Join two-decimal values with ", ".
std::string join_values(const std::vector<double> &values)
{
    std::string joined;
    for (size_t index = 0; index < values.size(); ++index)
        joined += (index == 0 ? "" : ", ") + two_decimals(values[index]);
    return joined;
}

// Recompute the statistics lines of an export from the Reader volumes and the configuration, independently of the host
// formatting. A tool beyond the configured nozzles is ignored, except in the tower share, whose densities come from
// measured_config (the configuration of the Print) when given. The weight line exists when some tool weighs something, the cost line when some weighing tool has a price.
std::vector<std::string> expected_statistics_lines(const GCodeProcessorResult &result, const DynamicPrintConfig &config,
                                                   int toolchanges, int object_layers, int total_layers,
                                                   const DynamicPrintConfig *measured_config = nullptr)
{
    const size_t tools = config.option<ConfigOptionFloats>("nozzle_diameter")->get_values().size();
    const ConfigOptionFloats &diameters = *config.option<ConfigOptionFloats>("filament_diameter");
    const ConfigOptionFloats &densities = *config.option<ConfigOptionFloats>("filament_density");
    const ConfigOptionFloats &prices = *config.option<ConfigOptionFloats>("filament_cost");
    std::vector<double> millimeters(tools, 0.), cubic(tools, 0.), grams(tools, 0.), costs(tools, 0.);
    double total_grams = 0., total_cost = 0.;
    bool has_weight = false, has_cost = false;
    for (const auto &[tool, volume] : result.print_statistics.volumes_per_extruder) {
        if (size_t(tool) >= tools)
            continue;
        const double radius = diameters.get_at(tool) / 2.;
        millimeters[tool] = volume / (PI * radius * radius);
        cubic[tool] = volume / 1000.;
        grams[tool] = cubic[tool] * densities.get_at(tool);
        costs[tool] = grams[tool] * prices.get_at(tool) / 1000.;
        total_grams += grams[tool];
        total_cost += costs[tool];
        has_weight = has_weight || grams[tool] > 0.;
        has_cost = has_cost || (grams[tool] > 0. && costs[tool] > 0.);
    }
    // The tower share was measured with the configuration of the Print, whatever configuration the lines use.
    const ConfigOptionFloats &tower_densities =
        *(measured_config != nullptr ? measured_config : &config)->option<ConfigOptionFloats>("filament_density");
    double tower_grams = 0.;
    const std::map<GCodeExtrusionRole, std::map<extruder_index_t, double>>::const_iterator tower =
        result.print_statistics.volumes_per_role_per_extruder.find(GCodeExtrusionRole::WipeTower);
    if (tower != result.print_statistics.volumes_per_role_per_extruder.end())
        for (const auto &[tool, volume] : tower->second)
            tower_grams += volume / 1000. * tower_densities.get_at(tool);
    const bool has_support =
        result.print_statistics.used_filaments_per_role.count(GCodeExtrusionRole::SupportMaterial) != 0 ||
        result.print_statistics.used_filaments_per_role.count(GCodeExtrusionRole::SupportMaterialInterface) != 0;

    std::vector<std::string> lines;
    lines.push_back("; filament used [mm] = " + join_values(millimeters));
    lines.push_back("; filament used [cm3] = " + join_values(cubic));
    if (has_weight)
        lines.push_back("; filament used [g] = " + join_values(grams));
    if (has_cost)
        lines.push_back("; filament cost = " + join_values(costs));
    lines.push_back("; total filament used [g] = " + two_decimals(total_grams));
    lines.push_back("; total filament cost = " + two_decimals(total_cost));
    lines.push_back("; total filament used for wipe tower [g] = " + two_decimals(tower_grams));
    lines.push_back("; total toolchanges = " + std::to_string(toolchanges));
    lines.push_back("; objects layers count = " + std::to_string(object_layers));
    lines.push_back("; total layers count = " + std::to_string(total_layers));
    lines.push_back(std::string("; has_support = ") + (has_support ? "1" : "0"));
    return lines;
}

// Split borrowed text into lines without their newline.
std::vector<std::string> split_lines(const std::string &text)
{
    std::vector<std::string> lines;
    size_t line_begin = 0;
    while (line_begin < text.size()) {
        const size_t line_end = text.find('\n', line_begin);
        lines.push_back(text.substr(line_begin, line_end - line_begin));
        if (line_end == std::string::npos)
            break;
        line_begin = line_end + 1;
    }
    return lines;
}

// Compare the lines of a block with the expected ones in one report: the size, then each line (at most a dozen).
void expect_lines(const std::string &block, const std::vector<std::string> &expected)
{
    const std::vector<std::string> lines = split_lines(block);
    REQUIRE(lines.size() == expected.size());
    for (size_t index = 0; index < expected.size(); ++index)
        CHECK(lines[index] == expected[index]);
}

// Hold the files and Reader result of one export that rereads its reference.
struct ExportedFile
{
    GCodeProcessorResult result;
    std::string body;
    std::string text;
    std::string statistics_block;
};

// Return the text between one exact body and the configuration marker of an exported file.
std::string block_between_body_and_configuration(const std::string &text, const std::string &body)
{
    const size_t body_position = text.find(body);
    REQUIRE(body_position != std::string::npos);
    const std::string marker = "\n; " SLIC3R_APP_NAME "_config = begin\n";
    const size_t marker_position = text.find(marker, body_position + body.size());
    REQUIRE(marker_position != std::string::npos);
    return text.substr(body_position + body.size(), marker_position - body_position - body.size());
}

// Generate the reference of a processed Print (asking for a Reader result or not), export it with the Print's own
// configuration, and return the body, the exported text and the text between them.
ExportedFile export_print_file(Print &print, const boost::filesystem::path &directory, bool ask_for_result)
{
    ExportedFile exported;
    Orchestrator &orchestrator = Orchestrator::instance();
    const boost::filesystem::path reference = directory / "reference.gcode";
    orchestrator.export_gcode(print, reference.string(), ask_for_result ? &exported.result : nullptr);
    exported.body = read_file(reference);
    const DynamicPrintConfig config = print.full_print_config();
    const Orchestrator::GCodeExporterSelection selection = orchestrator.select_gcode_exporter(config);
    const boost::filesystem::path destination = directory / "exported.gcode";
    orchestrator.export_print(print, config, reference.string(), destination.string(), selection);
    exported.text = read_file(destination);
    exported.statistics_block = block_between_body_and_configuration(exported.text, exported.body);
    return exported;
}

} // namespace

// Export without a Reader result: the plan facts are there (tool changes counted from the plan, initial and used tools
// with their filament types, layer counts), the measured values are not, and the availability flag stays false. Cases:
// two tools with a tower, a single non-zero tool whose end G-code selects T0, and a configured tool that is never used.
TEST_CASE("STEP_GCODE publishes the plan facts without a Reader result",
          "[plugins][gcode][print-statistics][plan-facts]")
{
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const int scenario = GENERATE(0, 1, 2);
    CAPTURE(scenario);
    ScopedDirectory directory;
    Print print;
    Model model;
    DynamicPrintConfig config = statistics_config(scenario == 2 ? 3 : 2);
    if (scenario == 0) {
        enable_wipe_tower(config);
        init_two_objects(print, model, config, 1, 2);
    } else if (scenario == 1) {
        // The end G-code selects tool 0 after the print: scripts never count as planned transitions.
        config.set_deserialize_strict("end_gcode", "T0\n");
        init_two_objects(print, model, config, 2, 2);
    } else {
        Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
    }
    print.process();
    const boost::filesystem::path reference = directory.path() / "reference.gcode";
    Orchestrator::instance().export_gcode(print, reference.string(), nullptr);

    const PrintStatistics &statistics = print.print_statistics();
    CHECK_FALSE(statistics.statistics_available.load());
    CHECK(statistics.total_used_filament == 0.0);
    if (scenario == 0) {
        CHECK(statistics.total_toolchanges == count_tool_transitions(read_file(reference)));
        CHECK(statistics.total_toolchanges > 0);
        CHECK(statistics.initial_extruder_id == 0);
        CHECK(statistics.initial_filament_type == "PLA");
        CHECK(statistics.printing_extruders == std::vector<extruder_index_t>{0, 1});
        CHECK(statistics.printing_filament_types == "PETG,PLA");
    } else if (scenario == 1) {
        CHECK(statistics.total_toolchanges == 0);
        CHECK(statistics.initial_extruder_id == 1);
        CHECK(statistics.initial_filament_type == "PETG");
        CHECK(statistics.printing_extruders == std::vector<extruder_index_t>{1});
        CHECK(statistics.printing_filament_types == "PETG");
    } else {
        CHECK(statistics.total_toolchanges == 0);
        CHECK(statistics.initial_extruder_id == 0);
        CHECK(statistics.initial_filament_type == "PLA");
        CHECK(statistics.printing_extruders == std::vector<extruder_index_t>{0});
        CHECK(statistics.printing_filament_types == "PLA");
    }
    CHECK(statistics.object_layer_count == int(print.object(0).layer_count()));
    CHECK(statistics.total_layer_count >= statistics.object_layer_count);
}

// The objects layer count and the total layer count differ when a print-level skirt adds a leading auxiliary group
// (complete objects with one shared skirt), and agree otherwise. The literal values come from the cube height: 67 layers at 0.3 mm after a
// 0.2 mm first layer, and one auxiliary skirt layer.
TEST_CASE("The objects layer count and the total layer count are kept apart",
          "[plugins][gcode][print-statistics][layer-counts]")
{
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool complete_objects = GENERATE(false, true);
    CAPTURE(complete_objects);
    ScopedDirectory directory;
    DynamicPrintConfig config = statistics_config(1);
    config.set_deserialize_strict({{"complete_objects", complete_objects ? "1" : "0"}, {"skirts", "1"},
                                   {"skirt_height", "1"}, {"skirt_distance", "6"}, {"complete_objects_one_skirt", "1"}, {"brim_width", "0"}});
    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
    print.process();
    const ExportedFile exported = export_print_file(print, directory.path(), true);

    const PrintStatistics &statistics = print.print_statistics();
    REQUIRE(print.object(0).layer_count() == 67);
    CHECK(statistics.object_layer_count == 67);
    CHECK(statistics.total_layer_count == (complete_objects ? 68 : 67));
    // The exported lines carry each count under its own label.
    CHECK(exported.statistics_block.find("; objects layers count = 67\n") != std::string::npos);
    CHECK(exported.statistics_block.find(complete_objects ? "; total layers count = 68\n" : "; total layers count = 67\n") !=
          std::string::npos);
}

// The flag is false on a new Print and statistics, true after a successful reread (also when every total is zero),
// and false again after an invalidation of the G-code, a clear of the statistics and a clear of the Print.
TEST_CASE("The availability flag follows the life of the measured statistics",
          "[plugins][gcode][print-statistics][availability]")
{
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    ScopedDirectory directory;
    DynamicPrintConfig config = statistics_config(1);
    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
    CHECK_FALSE(print.print_statistics().statistics_available.load());
    print.process();
    CHECK_FALSE(print.print_statistics().statistics_available.load());

    // A reread whose totals are all zero is still a successful reread.
    PrintStatistics zero_totals;
    CHECK_FALSE(zero_totals.statistics_available.load());
    const GCodeProcessorResult empty_result = {};
    Steps::StepReadGeneratedGcode::fill_print_statistics(empty_result, print.config(), zero_totals);
    CHECK(zero_totals.statistics_available.load());
    CHECK(zero_totals.total_used_filament == 0.0);
    zero_totals.clear();
    CHECK_FALSE(zero_totals.statistics_available.load());

    GCodeProcessorResult result;
    const std::string reference = (directory.path() / "reference.gcode").string();
    Orchestrator::instance().export_gcode(print, reference, &result);
    CHECK(print.print_statistics().statistics_available.load());

    // An export that does not ask for a result replays STEP_GCODE and leaves the statistics unavailable.
    Orchestrator::instance().export_gcode(print, reference, nullptr);
    CHECK_FALSE(print.print_statistics().statistics_available.load());

    Orchestrator::instance().export_gcode(print, reference, &result);
    REQUIRE(print.print_statistics().statistics_available.load());
    // Invalidating STEP_GCODE alone, without the checkpoint cleanup of an apply, also withdraws the statistics.
    print.mark_step_for_execution(STEP_GCODE);
    CHECK_FALSE(print.print_statistics().statistics_available.load());
    Orchestrator::instance().export_gcode(print, reference, &result);
    REQUIRE(print.print_statistics().statistics_available.load());
    // A configuration change reaches STEP_GCODE through the usual invalidation.
    DynamicPrintConfig changed = print.full_print_config();
    changed.set_deserialize_strict("fill_density", "35%");
    print.apply(model, changed);
    CHECK_FALSE(print.print_statistics().statistics_available.load());

    print.process();
    Orchestrator::instance().export_gcode(print, reference, &result);
    REQUIRE(print.print_statistics().statistics_available.load());
    print.clear();
    CHECK_FALSE(print.print_statistics().statistics_available.load());
}

// Activate one registered provider for the scope of a test and restore its prior activation state afterwards.
class ScopedProviderActivation
{
public:
    // Activate the registered provider and remember whether it was active.
    ScopedProviderActivation(Orchestrator &orchestrator, const char *plugin_id)
        : m_orchestrator(orchestrator), m_plugin_id(plugin_id)
    {
        Plugin *plugin = m_orchestrator.get_plugin(m_plugin_id);
        REQUIRE(plugin != nullptr);
        m_was_active = m_orchestrator.is_plugin_active(plugin);
        REQUIRE(m_orchestrator.set_plugin_active(m_plugin_id, true));
    }

    // Restore the prior state without throwing during cleanup.
    ~ScopedProviderActivation()
    {
        if (!m_was_active)
            m_orchestrator.set_plugin_active(m_plugin_id, false);
    }

private:
    Orchestrator &m_orchestrator;
    std::string m_plugin_id;
    bool m_was_active = false;
};

// A reread that fails inside the Reader leaves the statistics unavailable, and the next export of that Print has no
// statistics line. The failing Reader is activated for this test only; it never wins automatic detection.
TEST_CASE("A failing reread leaves the statistics unavailable",
          "[plugins][gcode][exporter][print-statistics][availability][failing-read]")
{
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    ScopedDirectory directory;
    DynamicPrintConfig config = statistics_config(1);
    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
    print.process();
    GCodeProcessorResult result;
    Orchestrator::instance().export_gcode(print, (directory.path() / "reference.gcode").string(), &result);
    REQUIRE(print.print_statistics().statistics_available.load());

    GCodeProcessorResult failed_result;
    {
        const ScopedProviderActivation activation(Orchestrator::instance(), "gcode.reader.test.failing_read");
        const Plugin *failing_reader = Orchestrator::instance().get_plugin("gcode.reader.test.failing_read");
        REQUIRE(failing_reader != nullptr);
        REQUIRE(Orchestrator::instance().is_plugin_active(failing_reader));
        const uint64_t runs_before = failing_reader->get_run_count();
        CHECK_THROWS_WITH(Steps::StepReadGeneratedGcode::run(Orchestrator::instance(), print, "gcode.reader.test.failing_read",
                                                             failed_result),
                          Catch::Matchers::Contains("'gcode.reader.test.failing_read' failed while building"));
        // The host wraps the Reader's own message; the run count proves that the Reader really ran, so the failure
        // did not come from an availability check.
        CHECK(failing_reader->get_run_count() == runs_before + 1);
    }
    CHECK_FALSE(print.print_statistics().statistics_available.load());

    // The next export without a Reader result carries no statistics, and a good reread makes them available again.
    const ExportedFile without_statistics = export_print_file(print, directory.path(), false);
    CHECK(without_statistics.statistics_block.empty());
    Steps::StepReadGeneratedGcode::run(Orchestrator::instance(), print, "gcode.reader.marlin2", result);
    Steps::StepReadGeneratedGcode::fill_print_statistics(result, print.config(), print.print_statistics());
    CHECK(print.print_statistics().statistics_available.load());
}
// The support flag is assigned by every fill: true for a support or a support interface extrusion seen by the Reader,
// false again for a following result without support.
TEST_CASE("The support flag follows the roles seen by the Reader",
          "[plugins][gcode][print-statistics][has-support]")
{
    PrintConfig config;
    config.filament_diameter.set(std::vector<double>{1.75});
    config.filament_density.set(std::vector<double>{1.1});
    config.filament_cost.set(std::vector<double>{20.0});
    GCodeProcessorResult result;
    result.print_statistics.volumes_per_extruder = {{0, 100.0}};
    PrintStatistics statistics;

    Steps::StepReadGeneratedGcode::fill_print_statistics(result, config, statistics);
    CHECK_FALSE(statistics.has_support);

    result.print_statistics.used_filaments_per_role[GCodeExtrusionRole::SupportMaterial] = {1.0, 1.0};
    Steps::StepReadGeneratedGcode::fill_print_statistics(result, config, statistics);
    CHECK(statistics.has_support);

    result.print_statistics.used_filaments_per_role.clear();
    Steps::StepReadGeneratedGcode::fill_print_statistics(result, config, statistics);
    CHECK_FALSE(statistics.has_support);

    result.print_statistics.used_filaments_per_role[GCodeExtrusionRole::SupportMaterialInterface] = {1.0, 1.0};
    Steps::StepReadGeneratedGcode::fill_print_statistics(result, config, statistics);
    CHECK(statistics.has_support);

    result.print_statistics.used_filaments_per_role.clear();
    result.print_statistics.used_filaments_per_role[GCodeExtrusionRole::Perimeter] = {1.0, 1.0};
    Steps::StepReadGeneratedGcode::fill_print_statistics(result, config, statistics);
    CHECK_FALSE(statistics.has_support);
}

// Export a reread two-tool print with a tower: the lines, their order and formats, the filament-only total cost, the
// non-zero tower share, the tool changes and the layers follow the body, and the configuration follows the lines.
TEST_CASE("An exported file lists the statistics before the configuration",
          "[plugins][gcode][exporter][print-statistics][statistics-block]")
{
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    ScopedDirectory directory;
    DynamicPrintConfig config = statistics_config(2);
    enable_wipe_tower(config);
    Print print;
    Model model;
    init_two_objects(print, model, config, 1, 2);
    print.process();
    const ExportedFile exported = export_print_file(print, directory.path(), true);

    REQUIRE_FALSE(exported.statistics_block.empty());
    std::string first_bad_line;
    CHECK_MESSAGE(Test::Plugins::GCodeExportTest::is_statistics_block(exported.statistics_block, first_bad_line),
                  "Malformed statistics line: " << first_bad_line);
    const int toolchanges = count_tool_transitions(exported.body);
    CHECK(toolchanges > 0);
    expect_lines(exported.statistics_block,
                 expected_statistics_lines(exported.result, print.full_print_config(), toolchanges,
                                           int(print.object(0).layer_count()), 67));
    // The tower consumes material and the filament cost excludes the time cost of 2 per hour.
    CHECK(exported.statistics_block.find("; total filament used for wipe tower [g] = 0.00\n") == std::string::npos);
    CHECK(print.print_statistics().total_cost > 0.0);
    // The configuration follows the statistics immediately and ends the file.
    const std::string end_marker = "; " SLIC3R_APP_NAME "_config = end\n";
    CHECK(exported.text.compare(exported.text.size() - end_marker.size(), end_marker.size(), end_marker) == 0);
}

// The tool count of the lines is the number of nozzles of the exported configuration: a tool measured beyond it is in
// no per-tool line and in no weight or cost total; the tower share is the one measured for the whole Print.
TEST_CASE("Measured tools beyond the exported configuration are ignored",
          "[plugins][gcode][exporter][print-statistics][statistics-block]")
{
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    ScopedDirectory directory;
    DynamicPrintConfig config = statistics_config(2);
    enable_wipe_tower(config);
    Print print;
    Model model;
    init_two_objects(print, model, config, 1, 2);
    print.process();
    GCodeProcessorResult result;
    Orchestrator &orchestrator = Orchestrator::instance();
    const boost::filesystem::path reference = directory.path() / "reference.gcode";
    orchestrator.export_gcode(print, reference.string(), &result);
    REQUIRE(result.print_statistics.volumes_per_extruder.size() == 2);
    const std::string body = read_file(reference);

    DynamicPrintConfig one_tool = print.full_print_config();
    one_tool.set_num_extruders(1);
    const Orchestrator::GCodeExporterSelection selection = orchestrator.select_gcode_exporter(one_tool);
    const boost::filesystem::path destination = directory.path() / "exported.gcode";
    orchestrator.export_print(print, one_tool, reference.string(), destination.string(), selection);
    const std::string block = block_between_body_and_configuration(read_file(destination), body);

    const DynamicPrintConfig original_config = print.full_print_config();
    expect_lines(block, expected_statistics_lines(result, one_tool, count_tool_transitions(body),
                                                  int(print.object(0).layer_count()), 67, &original_config));
    const std::vector<std::string> lines = split_lines(block);
    REQUIRE(lines.size() >= 1);
    CHECK(lines[0].find(',') == std::string::npos);
}
// A single tool gives one value per line; a zero density removes the weight and cost lines, and a zero price removes
// the cost line only.
TEST_CASE("The weight and cost lines exist only when the material has a weight and a price",
          "[plugins][gcode][exporter][print-statistics][statistics-block]")
{
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const int scenario = GENERATE(0, 1, 2);
    CAPTURE(scenario);
    ScopedDirectory directory;
    DynamicPrintConfig config = statistics_config(1);
    if (scenario == 1)
        config.set_deserialize_strict("filament_cost", "0");
    if (scenario == 2)
        config.set_deserialize_strict("filament_density", "0");
    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
    print.process();
    const ExportedFile exported = export_print_file(print, directory.path(), true);

    expect_lines(exported.statistics_block,
                 expected_statistics_lines(exported.result, print.full_print_config(), 0,
                                           int(print.object(0).layer_count()), 67));
    CHECK((exported.statistics_block.find("; filament used [g] = ") != std::string::npos) == (scenario != 2));
    CHECK((exported.statistics_block.find("; filament cost = ") != std::string::npos) == (scenario == 0));
}

// Four tools are configured and only tools 0 and 2 print: every per-tool line has four values with 0.00 at the
// positions of the unused tools, and the used tools are those of the plan.
TEST_CASE("Unused configured tools appear as zero values in the statistics",
          "[plugins][gcode][exporter][print-statistics][statistics-block]")
{
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    ScopedDirectory directory;
    DynamicPrintConfig config = statistics_config(4);
    Print print;
    Model model;
    init_two_objects(print, model, config, 1, 3);
    print.process();
    const ExportedFile exported = export_print_file(print, directory.path(), true);

    const int toolchanges = count_tool_transitions(exported.body);
    expect_lines(exported.statistics_block,
                 expected_statistics_lines(exported.result, print.full_print_config(), toolchanges,
                                           int(print.object(0).layer_count()), 67));
    const std::vector<std::string> lines = split_lines(exported.statistics_block);
    REQUIRE(lines.size() >= 2);
    for (const std::string &line : {lines[0], lines[1]}) {
        const std::string values = line.substr(line.find(" = ") + 3);
        // Four values separated by ", ": the unused tools 1 and 3 are exactly 0.00.
        CHECK(std::count(values.begin(), values.end(), ',') == 3);
        CHECK(values.find(", 0.00, ") != std::string::npos);
        CHECK(values.compare(values.size() - 6, 6, ", 0.00") == 0);
        CHECK(values.compare(0, 4, "0.00") != 0);
    }
    const PrintStatistics &statistics = print.print_statistics();
    CHECK(statistics.printing_extruders == std::vector<extruder_index_t>{0, 2});
    CHECK(statistics.printing_filament_types == "ABS,PLA");
}

// An export without a Reader result has no statistics line at all, not even the plan facts.
TEST_CASE("An export without a Reader result has no statistics lines",
          "[plugins][gcode][exporter][print-statistics][statistics-block]")
{
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    ScopedDirectory directory;
    DynamicPrintConfig config = statistics_config(1);
    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
    print.process();
    const ExportedFile exported = export_print_file(print, directory.path(), false);

    CHECK(exported.statistics_block.empty());
    CHECK(exported.text.find("; total toolchanges") == std::string::npos);
    CHECK(exported.text.find("; has_support") == std::string::npos);
}

// The command-line export rereads its reference: its file carries the statistics block and its name uses a
// file name variable that comes from the plan facts.
TEST_CASE("The command-line export has the statistics lines and the plan-fact file name",
          "[plugins][gcode][exporter][print-statistics][cli]")
{
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    ScopedDirectory directory;
    DynamicPrintConfig config = statistics_config(2);
    enable_wipe_tower(config);
    config.set_deserialize_strict("output_filename_format", "cli_{printing_filament_types}.gcode");
    Print print;
    Model model;
    init_two_objects(print, model, config, 1, 2);
    print.process();
    const std::string exported_path = Orchestrator::instance().export_gcode_cli(print, directory.path().string());

    const std::string text = read_file(exported_path);
    const size_t block_begin = text.find("; filament used [mm] = ");
    const std::string begin_marker = "\n; " SLIC3R_APP_NAME "_config = begin\n";
    const size_t block_end = text.find(begin_marker);
    REQUIRE(block_begin != std::string::npos);
    REQUIRE(block_end != std::string::npos);
    REQUIRE(block_begin < block_end);
    const std::string block = text.substr(block_begin, block_end - block_begin);
    std::string first_bad_line;
    CHECK_MESSAGE(Test::Plugins::GCodeExportTest::is_statistics_block(block, first_bad_line),
                  "Malformed statistics line: " << first_bad_line);
    const int toolchanges = count_tool_transitions(text.substr(0, block_begin));
    CHECK(toolchanges > 0);
    CHECK(block.find("; total toolchanges = " + std::to_string(toolchanges) + "\n") != std::string::npos);
    CHECK(boost::filesystem::path(exported_path).filename().string() == "cli_PETG,PLA.gcode");
    // Both plan-fact variables reach a file name template through the statistics.
    CHECK(boost::filesystem::path(print.print_statistics().finalize_output_path("x_{printing_filament_types}_{total_toolchanges}.gcode")).filename().string() ==
          "x_PETG,PLA_" + std::to_string(toolchanges) + ".gcode");
}
