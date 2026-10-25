// The late variables of an output name ({printing_filament_types}, {total_toolchanges}...) are resolved from the print
// statistics after the export: filtering the illegal characters must keep them intact, whatever the length of the
// replaced text, and only then resolve from the statistics.
#include <catch2/catch.hpp>

#include <string>

#include <boost/filesystem.hpp>

#include "plugin_test_helpers.hpp"
#include "test_data.hpp"

#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"

using namespace Slic3r;

TEST_CASE("Output filenames preserve late variables during illegal-character filtering",
          "[plugins][output-filename]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    std::string filename_base = "name";
    std::string expected;
    std::string finalized;
    SECTION("Two late variables separated by filtered text") {
        config.set_deserialize_strict("output_filename_format", "cli?{printing_filament_types}_mid*_{total_toolchanges}.gcode");
        expected = "cli_{printing_filament_types}_mid__{total_toolchanges}.gcode";
        finalized = "cli_PLA_mid__7.gcode";
    }
    SECTION("Illegal characters between and after late variables") {
        config.set_deserialize_strict("output_filename_format", "{printing_filament_types}?{total_toolchanges}*.gcode");
        expected = "{printing_filament_types}_{total_toolchanges}_.gcode";
        finalized = "PLA_7_.gcode";
    }
    SECTION("A regex shortens text around two late variables") {
        config.set_deserialize_strict("gcode_filename_illegal_char", "[?*]+");
        config.set_deserialize_strict("output_filename_format", "cli??{printing_filament_types}?**{total_toolchanges}***.gcode");
        expected = "cli_{printing_filament_types}_{total_toolchanges}_.gcode";
        finalized = "cli_PLA_7_.gcode";
    }
    SECTION("A name without late variables keeps the same filtering") {
        config.set_deserialize_strict("gcode_filename_illegal_char", "[?*]+");
        config.set_deserialize_strict("output_filename_format", "cli??plain?**tail***.gcode");
        expected = "cli_plain_tail_.gcode";
        finalized = "cli_plain_tail_.gcode";
    }
    SECTION("A literal character filter preserves two late variables") {
        config.set_deserialize_strict("gcode_filename_illegal_char", "?*");
        config.set_deserialize_strict("output_filename_format", "cli?{printing_filament_types}_mid*_{total_toolchanges}.gcode");
        expected = "cli_{printing_filament_types}_mid__{total_toolchanges}.gcode";
        finalized = "cli_PLA_mid__7.gcode";
    }
    SECTION("Input filename expansion precedes illegal-character filtering") {
        filename_base = "na?me";
        config.set_deserialize_strict("output_filename_format", "{input_filename_base}_{printing_filament_types}.gcode");
        expected = "na_me_{printing_filament_types}.gcode";
        finalized = "na_me_PLA.gcode";
    }

    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
    const std::string filename = print.output_filename(filename_base);
    REQUIRE(filename == expected);

    PrintStatistics statistics;
    statistics.printing_filament_types = "PLA";
    statistics.total_toolchanges = 7;
    REQUIRE(boost::filesystem::path(statistics.finalize_output_path(filename)).filename().string() == finalized);
}
