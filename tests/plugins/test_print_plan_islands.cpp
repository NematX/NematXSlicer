///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#include <catch2/catch.hpp>
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <map>
#include <set>
#include <sstream>
#include <boost/filesystem.hpp>
#include "gcode_test_helpers.hpp"
#include "plugin_test_helpers.hpp"
#include "test_data.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/GCodeExportObjects.hpp"
#include "libslic3r/CustomGCode.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Plugins/Common/SourceLayerIndices.hpp"

/*
These production export tests measure the order of a shared base and two towers.
The mesh is made locally from boxes; its five base layers join the two towers,
whose twenty and nineteen layers have distinct source region islands. A region
island is the layer's material-specific printable part (LayerRegion.hpp).
The Print owns the source regions and plan throughout export and inspection.
Multiple extrusion entries for one island are collapsed within one plan layer, so a repeated
layer in another group still fails the coverage check. Emitted Z markers must
match every plan layer in order before island scheduling is checked.
Two-copy checks reconstruct each instance independently; height-limit checks also
require a shared final group in increasing Z and no upper layers in island stacks.
Plate-wide groups may contain several copies. Their source coverage and parent
order are checked independently of the stack order.
Height-event scenes also publish model rows before slicing and verify one event
across all copies. Relative-E deposition moves are checked against their emitted
layer blocks, and upper-copy tool assignments are checked against the final plan.
Source-tool scenes verify that substituted plan tools do not suppress later color
scripts, including ToolChange conversion on a single-tool printer.
Two-tool copy scenes compare the final plan and exported tool switches to ensure
the ordering consumer carries its last tool across group boundaries.

    TEST_CASE: select clearance shape and expected scheduling
    `-- check_scene: build boxes, process and export with the production host
        |-- source layers: associate region addresses with base or tower identity
        |-- plan groups and G-code Z markers: reconstruct ordered layer visits
        `-- coverage and height checks: require completeness and bounded lead
*/
namespace {
using namespace Slic3r;
constexpr const char *step_key = "complete_objects_step";
constexpr const char *limit_key = "complete_objects_step_max_z";
constexpr const char *islands_key = "complete_islands";
constexpr const char *whole_objects_key = "print_plan_builder";
enum class Expect { Alternate, Invariants, LayerByLayer };

// Slice and export one clearance variant, then check emitted layer order, exact
// source coverage and tower height lead. Borrowed region addresses live in Print;
// failures are reported by Catch and output files are removed before assertions.
// Shapes use the default, wide, flat, zero-radius, over-height, square or over-radius profile. Close places tower
// centers 6 mm apart rather than 40 mm. Every expectation checks all invariants;
// Alternate also checks stack run lengths, and LayerByLayer checks ascending global Z.
void check_scene(const std::string &name, int shape, double step, bool close, Expect expectation,
                 uint32_t copies = 1, double max_z = 0.);

// Require the alternating stack sizes for a nozzle that clears the distant towers.
TEST_CASE("Parallel islands alternate with intermediate nozzle clearance",
          "[plugins][printing-plan][parallel-islands]")
{
    check_scene("intermediate", 0, 1., false, Expect::Alternate);
    check_scene("zero-radius", 3, 1., false, Expect::Alternate);
    check_scene("profile-beyond-cap", 4, 1., false, Expect::Alternate);
    check_scene("square-profile", 5, 1., false, Expect::Alternate);
    check_scene("profile-beyond-radius", 6, 1., false, Expect::Alternate);
}

// Check the intermediate export and all invariants without accepting expected failures.
TEST_CASE("Parallel islands intermediate clearance preserves source layers",
          "[plugins][printing-plan][parallel-islands]")
{
    check_scene("intermediate-invariants", 0, 1., false, Expect::Invariants);
}

// Require layer-by-layer scheduling when shape, step or tower spacing prevents clearance.
TEST_CASE("Parallel islands remain grouped without nozzle clearance",
          "[plugins][printing-plan][parallel-islands]")
{
    SECTION("nozzle too wide") { check_scene("wide", 1, 1., false, Expect::LayerByLayer); }
    SECTION("nozzle too flat") { check_scene("flat", 2, 1., false, Expect::LayerByLayer); }
    SECTION("step above graph") { check_scene("high-step", 0, 3., false, Expect::LayerByLayer); }
    SECTION("nearby towers") { check_scene("close", 0, 1., true, Expect::LayerByLayer); }
}

// Export real copies through all consumers. Event tags belong to the target
// layer's before events, after its Z comment but before any deposited material.
TEST_CASE("Height events occur once across production copy bands",
          "[plugins][printing-plan][gcode][print-sequence][custom-per-print-z][height-event-export]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool islands = GENERATE(false, true);
    const bool tool_change = GENERATE(false, true);
    bool prefix = false, tail = false, grids = false;
    double barrier = 1.;
    SECTION("Ordinary stack bands") {}
    SECTION("Shared prefix") { prefix = true; grids = true; barrier = 0.3; }
    SECTION("Above the height limit") { tail = true; }
    SECTION("Different layer grids") { grids = true; }
    CAPTURE(islands, tool_change, prefix, tail, grids);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{whole_objects_key, "ordering.plan_builder.by_object"},
        {islands_key, islands ? "1" : "0"}, {step_key, "0.6"}, {limit_key, tail ? "0.7" : "0"},
        {"complete_objects_one_first_layer", prefix ? "1" : "0"},
        {"layer_height", "0.2"}, {"first_layer_height", "0.2"}, {"skirts", "0"},
        {"brim_width", "0"}, {"perimeters", "1"}, {"fill_density", "0%"},
        {"top_solid_layers", "0"}, {"bottom_solid_layers", "0"}, {"start_gcode", ""},
        {"end_gcode", ""}, {"binary_gcode", "0"}, {"use_relative_e_distances", "1"},
        {"pause_print_gcode", "M117 pause"}, {"color_change_gcode", "M117 color"}});
    if (tool_change) {
        config.set_deserialize_strict("nozzle_diameter", "0.4,0.4");
        config.set_deserialize_strict("min_layer_height", "0.05,0.05");
        config.set_deserialize_strict("max_layer_height", "0.3,0.3");
        config.set_deserialize_strict("filament_diameter", "1.75,1.75");
    }
    Model model;
    Print print;
    const TriangleMesh mesh = make_cube(2., 2., 1.6);
    Test::init_print(grids ? std::vector<TriangleMesh>{mesh, mesh} : std::vector<TriangleMesh>{mesh}, print, model, config);
    model.objects()[0].instances.front()->set_offset(Vec3d(30., 30., 0.));
    if (grids) {
        model.objects()[1].instances.front()->set_offset(Vec3d(100., 30., 0.));
        model.objects()[1].config.set_deserialize_strict("layer_height", "0.3");
        if (prefix)
            model.objects()[1].config.set_deserialize_strict("first_layer_height", "0.4");
    } else {
        model.objects()[0].add_instance(*model.objects()[0].instances.front())->set_offset(Vec3d(100., 30., 0.));
    }
    model.custom_gcode_per_print_z.mode = tool_change ? CustomGCode::MultiAsSingle : CustomGCode::SingleExtruder;
    if (tool_change)
        model.custom_gcode_per_print_z.gcodes.push_back({scale_i(barrier), CustomGCode::ToolChange, 2, "", ""});
    else {
        model.custom_gcode_per_print_z.gcodes.push_back({scale_i(barrier), CustomGCode::PausePrint, 1, "", "pause"});
        model.custom_gcode_per_print_z.gcodes.push_back({scale_i(barrier), CustomGCode::ColorChange, 1, "#FF0000", ""});
    }
    model.custom_gcode_per_print_z.gcodes.push_back({scale_i(3.), CustomGCode::Custom, 1, "", "M117 unreachable"});
    Orchestrator::instance().consume_plugin_messages();
    print.apply(model, config);
    print.set_status_silent();
    print.process();
    Test::GCode::select_printing_plan_firmware(print);
    const boost::filesystem::path output = Test::GCode::temporary_gcode_path();
    Orchestrator::instance().export_gcode(print, output.string(), nullptr);
    const std::string text = Test::GCode::read_text_file(output);
    Test::GCode::remove_output_pair(output);
    CHECK(text.find("M117 unreachable") == std::string::npos);
    size_t above_warnings = 0;
    for (const Orchestrator::PluginMessage &warning : Orchestrator::instance().consume_plugin_messages())
        if (warning.level == Orchestrator::PluginMessageLevel::Warning &&
            warning.message.find("Ignoring custom G-code marker above the last printing-plan layer") != std::string::npos)
            ++above_warnings;
    CHECK(above_warnings == 1);
    REQUIRE(print.printing_plan() != nullptr);
    std::vector<double> planned_z;
    std::set<uint32_t> upper_copies;
    for (const Printing::PrintingGroup &group : print.printing_plan()->groups)
        for (const Printing::PrintingLayerGroup &layer : group.layers) {
            planned_z.push_back(unscaled(layer.print_z));
            if (tool_change && layer.print_z >= scale_i(barrier))
                for (const Printing::PrintingToolGroup &tool : layer.tool_groups)
                    for (const Printing::PrintingExtrusion &extrusion : tool.extrusions) {
                        CHECK(tool.extruder_id == 1);
                        upper_copies.insert(extrusion.printing_catalog_id);
                    }
        }
    if (tool_change)
        CHECK(upper_copies.size() == 2);
    const size_t target = size_t(std::find_if(planned_z.begin(), planned_z.end(), [barrier](double z) {
        return z + 0.000001 >= barrier;
    }) - planned_z.begin());
    REQUIRE(target < planned_z.size());
    std::vector<double> emitted_z;
    size_t pause_count = 0, color_count = 0, deposits_below = 0, deposits_above = 0;
    int active_tool = -1;
    std::istringstream lines(text);
    std::string line;
    while (std::getline(lines, line)) {
        if (line.rfind(";Z:", 0) == 0)
            emitted_z.push_back(std::stod(line.substr(3)));
        if (line.rfind("T", 0) == 0 && line.size() > 1 && std::isdigit(static_cast<unsigned char>(line[1])))
            active_tool = std::stoi(line.substr(1));
        if (line.rfind(";PAUSE_PRINT", 0) == 0 || line.rfind(";COLOR_CHANGE", 0) == 0) {
            const bool pause = line.rfind(";PAUSE_PRINT", 0) == 0;
            if (pause) CHECK(color_count == 0);
            else CHECK(pause_count == 1);
            if (pause) ++pause_count; else ++color_count;
            REQUIRE_FALSE(emitted_z.empty());
            CHECK(emitted_z.size() - 1 == target);
            CHECK(emitted_z.back() == Approx(planned_z[target]));
            for (size_t idx = 0; idx + 1 < emitted_z.size(); ++idx)
                CHECK(emitted_z[idx] < barrier);
        }
        // This scene uses relative E, linear G1 paths with space-separated fields,
        // dot-decimal numbers and no extrusion in scripts. Positive E with XY
        // therefore identifies deposition, excluding retraction and unretraction.
        const size_t e = line.find(" E");
        if (line.rfind("G1 ", 0) == 0 && e != std::string::npos &&
            (line.find(" X") != std::string::npos || line.find(" Y") != std::string::npos) &&
            std::stod(line.substr(e + 2)) > 0.) {
            REQUIRE_FALSE(emitted_z.empty());
            if (emitted_z.back() + 0.000001 >= barrier) {
                ++deposits_above;
                if (tool_change) CHECK(active_tool == 1);
                else { CHECK(pause_count == 1); CHECK(color_count == 1); }
            } else {
                ++deposits_below;
                if (!tool_change) {
                    CHECK(pause_count == 0);
                    CHECK(color_count == 0);
                }
            }
        }
    }
    REQUIRE(emitted_z.size() == planned_z.size());
    for (size_t idx = 0; idx < planned_z.size(); ++idx)
        CHECK(emitted_z[idx] == Approx(planned_z[idx]));
    CHECK(deposits_below > 0);
    CHECK(deposits_above > 0);
    CHECK(pause_count == (tool_change ? 0 : 1));
    CHECK(color_count == (tool_change ? 0 : 1));
}

// Odd-height copies with two tools per layer end on the higher tool. Every later
// group can keep that tool first without adding an internal change. Verify the
// complete production chain and the serialized switches, not only the builder.
// The color variant uses single-layer copies: T1 is pinned first only in the
// first copy, whose real end T0 must start the second copy without another switch.
TEST_CASE("Production tool ordering continues across copy groups",
          "[plugins][ordering][tool-groups][tool-ordering][print-sequence][gcode][group-tool-continuity]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool by_object = GENERATE(false, true);
    const bool islands = GENERATE(false, true);
    const double step = GENERATE(0., 0.4);
    const bool color = GENERATE(false, true);
    CAPTURE(by_object, islands, step, color);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"print_plan_builder", by_object ? "ordering.plan_builder.by_object" : "ordering.plan_builder.by_layer"},
        {"complete_objects_step", std::to_string(step)}, {"complete_islands", islands ? "1" : "0"},
        {"complete_objects_one_first_layer", "0"}, {"layer_height", "0.2"}, {"first_layer_height", "0.2"},
        {"skirts", "0"}, {"brim_width", "0"}, {"wipe_tower", "0"}, {"perimeters", "1"},
        {"fill_density", "0%"}, {"top_solid_layers", "0"}, {"bottom_solid_layers", "0"},
        {"nozzle_diameter", "0.4,0.4"}, {"filament_diameter", "1.75,1.75"},
        {"min_layer_height", "0.05,0.05"}, {"max_layer_height", "0.3,0.3"},
        {"start_gcode", ""}, {"end_gcode", ""}, {"binary_gcode", "0"},
        {"color_change_gcode", "M117 continuity-color"}});
    Model model;
    Print print;
    const double height = color ? 0.2 : 0.6;
    Test::init_print({make_cube(2., 2., height)}, print, model, config);
    TriangleMesh second = make_cube(2., 2., height);
    second.translate(Vec3f(2.f, 0.f, 0.f));
    ModelVolume *volume = model.objects()[0].add_volume(second);
    volume->config.set_key_value("extruder", new ConfigOptionInt(2));
    model.objects()[0].instances.front()->set_offset(Vec3d(30., 30., 0.));
    model.objects()[0].add_instance(*model.objects()[0].instances.front())->set_offset(Vec3d(100., 30., 0.));
    if (color) {
        model.custom_gcode_per_print_z.mode = CustomGCode::MultiExtruder;
        model.custom_gcode_per_print_z.gcodes = {{scale_i(0.2), CustomGCode::ColorChange, 2, "#FF0000", ""}};
    }
    print.apply(model, config);
    print.set_status_silent();
    print.process();
    Test::GCode::select_printing_plan_firmware(print);
    const boost::filesystem::path output = Test::GCode::temporary_gcode_path();
    Orchestrator::instance().export_gcode(print, output.string(), nullptr);
    const std::string text = Test::GCode::read_text_file(output);
    Test::GCode::remove_output_pair(output);
    REQUIRE(print.printing_plan() != nullptr);
    const Printing::PrintingPlan &plan = *print.printing_plan();
    if (by_object) REQUIRE(plan.groups.size() >= 2);
    else REQUIRE(plan.groups.size() == 1);
    extruder_index_t previous = EXTRUDER_ID_UNKNOWN;
    size_t boundaries = 0, high_tool_boundaries = 0;
    std::vector<int> expected_switches;
    for (const Printing::PrintingGroup &group : plan.groups) {
        REQUIRE_FALSE(group.layers.empty());
        REQUIRE_FALSE(group.layers.front().tool_groups.empty());
        const extruder_index_t first = group.layers.front().tool_groups.front().extruder_id;
        if (previous != EXTRUDER_ID_UNKNOWN) {
            CHECK(first == previous);
            ++boundaries;
            if (previous == 1) ++high_tool_boundaries;
        }
        for (const Printing::PrintingLayerGroup &layer : group.layers) {
            REQUIRE(layer.tool_groups.size() == 2);
            CHECK(layer.tool_groups[0].extruder_id != layer.tool_groups[1].extruder_id);
            for (const Printing::PrintingToolGroup &tool : layer.tool_groups) {
                if (expected_switches.empty() || expected_switches.back() != tool.extruder_id)
                    expected_switches.push_back(tool.extruder_id);
                previous = tool.extruder_id;
            }
        }
    }
    if (by_object) {
        CHECK(boundaries > 0);
        if (step == 0. && !color) CHECK(high_tool_boundaries > 0);
        if (color) {
            CHECK(plan.groups.size() == 2);
            CHECK(expected_switches == (std::vector<int>{1, 0, 1}));
        }
    } else {
        CHECK(expected_switches == (color ? std::vector<int>{1, 0} : std::vector<int>{0, 1, 0, 1}));
    }
    std::vector<int> emitted_switches;
    std::istringstream lines(text);
    std::string line;
    size_t color_scripts = 0, color_tags = 0;
    while (std::getline(lines, line)) {
        if (line.size() > 1 && line[0] == 'T' && std::isdigit(static_cast<unsigned char>(line[1]))) {
            const int tool = std::stoi(line.substr(1));
            if (emitted_switches.empty() || emitted_switches.back() != tool)
                emitted_switches.push_back(tool);
        }
        if (line == "M117 continuity-color") ++color_scripts;
        if (line == ";COLOR_CHANGE,T1,#FF0000") {
            ++color_tags;
            REQUIRE_FALSE(emitted_switches.empty());
            CHECK(emitted_switches.back() == 1);
        }
    }
    CHECK(emitted_switches == expected_switches);
    CHECK(color_scripts == (color ? 1 : 0));
    CHECK(color_tags == (color ? 1 : 0));
}

// A tool substitution must not reclassify a single-source-tool model. Export both
// builders and single-tool conversion through the real consumers and firmware;
// distinct next_color arguments distinguish the two scripted rows independently.
TEST_CASE("Source tools preserve color scripts after a height tool change",
          "[plugins][gcode][custom-per-print-z][custom-gcode][source-tool-mode]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool by_object = GENERATE(false, true);
    const bool single_tool_printer = GENERATE(false, true);
    CAPTURE(by_object, single_tool_printer);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"print_plan_builder", by_object ? "ordering.plan_builder.by_object" : "ordering.plan_builder.by_layer"},
        {"complete_objects_step", "0.6"}, {"complete_islands", "1"},
        {"layer_height", "0.2"}, {"first_layer_height", "0.2"}, {"skirts", "0"}, {"brim_width", "0"},
        {"perimeters", "1"}, {"fill_density", "0%"}, {"top_solid_layers", "0"}, {"bottom_solid_layers", "0"},
        {"start_gcode", ""}, {"end_gcode", ""}, {"binary_gcode", "0"},
        {"color_change_gcode", "M117 color {color_change_extruder} {next_color}"}});
    if (!single_tool_printer) {
        config.set_deserialize_strict("nozzle_diameter", "0.4,0.4");
        config.set_deserialize_strict("filament_diameter", "1.75,1.75");
        config.set_deserialize_strict("min_layer_height", "0.05,0.05");
        config.set_deserialize_strict("max_layer_height", "0.3,0.3");
    }
    Model model;
    Print print;
    Test::init_print({make_cube(2., 2., 1.6)}, print, model, config);
    model.objects()[0].instances.front()->set_offset(Vec3d(30., 30., 0.));
    model.objects()[0].add_instance(*model.objects()[0].instances.front())->set_offset(Vec3d(100., 30., 0.));
    model.custom_gcode_per_print_z.mode = CustomGCode::MultiAsSingle;
    const int tool = single_tool_printer ? 1 : 2;
    model.custom_gcode_per_print_z.gcodes = {
        {scale_i(0.6), CustomGCode::ToolChange, tool, "#00FF00", ""},
        {scale_i(1.), CustomGCode::ColorChange, tool, "#FF0000", ""}};
    print.apply(model, config);
    print.set_status_silent();
    print.process();
    Test::GCode::select_printing_plan_firmware(print);
    const boost::filesystem::path output = Test::GCode::temporary_gcode_path();
    Orchestrator::instance().export_gcode(print, output.string(), nullptr);
    const std::string text = Test::GCode::read_text_file(output);
    Test::GCode::remove_output_pair(output);
    const std::string expected_color_script = "M117 color " + std::to_string(tool - 1) + " #FF0000";
    const std::string expected_conversion_script = "M117 color 0 #00FF00";
    const std::string expected_color_tag = ";COLOR_CHANGE,T" + std::to_string(tool - 1) + ",#FF0000";
    size_t color_scripts = 0, conversion_scripts = 0, color_tags = 0, conversion_tags = 0;
    double z = 0.;
    std::istringstream lines(text);
    std::string line;
    while (std::getline(lines, line)) {
        if (line.rfind(";Z:", 0) == 0)
            z = std::stod(line.substr(3));
        if (line.rfind("M117 color ", 0) == 0) {
            if (line.find("#FF0000") != std::string::npos) {
                ++color_scripts;
                CHECK(line == expected_color_script);
                CHECK(z == Approx(1.));
            } else if (line.find("#00FF00") != std::string::npos) {
                ++conversion_scripts;
                CHECK(line == expected_conversion_script);
                CHECK(z == Approx(0.6));
            } else {
                FAIL("Unexpected color script arguments");
            }
        }
        if (line.rfind(";COLOR_CHANGE,", 0) == 0) {
            if (line.find("#FF0000") != std::string::npos) {
                ++color_tags;
                CHECK(line == expected_color_tag);
            } else if (line.find("#00FF00") != std::string::npos) {
                ++conversion_tags;
                CHECK(line == ";COLOR_CHANGE,T0,#00FF00");
            }
        }
    }
    CHECK(color_scripts == 1);
    CHECK(color_tags == 1);
    CHECK(conversion_scripts == (single_tool_printer ? 1 : 0));
    CHECK(conversion_tags == (single_tool_printer ? 1 : 0));
    REQUIRE(print.printing_plan() != nullptr);
    std::set<uint32_t> upper_copies;
    for (const Printing::PrintingGroup &group : print.printing_plan()->groups)
        for (const Printing::PrintingLayerGroup &layer : group.layers)
            if (layer.print_z >= scale_i(0.6))
                for (const Printing::PrintingToolGroup &visit : layer.tool_groups)
                    for (const Printing::PrintingExtrusion &extrusion : visit.extrusions) {
                        CHECK(visit.extruder_id == tool - 1);
                        upper_copies.insert(extrusion.printing_catalog_id);
                    }
    CHECK(upper_copies.size() == 2);
}

// Export distant copies with two islands each and inspect island entries, including
// entries on a new source layer. Auxiliary brims use their travel-distance threshold.
TEST_CASE("Object entry travels honor an absolute minimum height",
          "[plugins][gcode][lift][travel][lift-min-export]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool by_object = GENERATE(false, true);
    const int scenario = GENERATE(0, 1, 2, 3, 4, 5, 6, 7);
    const double floor = scenario == 0 ? 0.0 : scenario == 1 ? 0.1 : 5.0;
    CAPTURE(by_object, floor, scenario);
    struct TravelActivation {
        Orchestrator &host;
        Plugin *plugin;
        bool active;
        // Enable production travel geometry and remember the previous activation.
        explicit TravelActivation(Orchestrator &value) : host(value),
            plugin(value.get_plugin("layer_extrusion_edit.travel.default")), active(value.is_plugin_active(plugin)) {
            host.set_plugin_active(plugin, true);
        }
        // Restore activation even if export or an assertion fails.
        ~TravelActivation() { host.set_plugin_active(plugin, active); }
    } travel(Orchestrator::instance());
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"print_plan_builder", by_object ? "ordering.plan_builder.by_object" : "ordering.plan_builder.by_layer"},
        {"complete_objects_step", "0.4"}, {"complete_islands", "0"}, {"complete_objects_one_first_layer", "0"},
        {"layer_height", "0.2"}, {"first_layer_height", "0.2"}, {"skirts", "0"}, {"brim_width", "0"},
        {"perimeters", "1"}, {"fill_density", "0%"}, {"top_solid_layers", "0"}, {"bottom_solid_layers", "0"},
        {"start_gcode", "G90\nG1 X0 Y0 Z0.1\nM117 lift-start"}, {"end_gcode", ""},
        {"use_relative_e_distances", "1"}, {"binary_gcode", "0"}, {"lift_min", std::to_string(floor)},
        {"retract_lift", "0.5"}, {"print_retract_lift", "0.5"}, {"retract_lift_above", "0"},
        {"retract_lift_before_travel", "0"}, {"retract_before_travel", "0"}, {"retract_length", "1"},
        {"travel_ramping_lift", "0"}, {"wipe", "0"}, {"retract_lift_top", "All surfaces"}});
    if (scenario == 3)
        config.set_deserialize_strict("start_gcode", "M117 lift-start");
    if (scenario == 4)
        config.set_deserialize_strict({{"retract_lift_above", "10"}, {"retract_lift_before_travel", "1000"},
            {"travel_ramping_lift", "1"}, {"travel_slope", "10"}});
    if (scenario == 5)
        config.set_deserialize_strict("retract_length", "0");
    if (scenario == 6)
        config.set_deserialize_strict({{"brim_width", "2"}, {"brim_per_object", "0"}});
    if (scenario == 7)
        config.set_deserialize_strict({{"lift_min", "50"}, {"max_print_height", "5"}});
    Model model;
    Print print;
    TriangleMesh mesh = make_cube(2., 2., 1.);
    TriangleMesh island = make_cube(2., 2., 1.);
    island.translate(Vec3f(10.f, 0.f, 0.f));
    mesh.merge(island);
    TriangleMesh tall = mesh;
    tall.scale(Vec3f(1.f, 1.f, 1.4f));
    Test::init_print({mesh, tall}, print, model, config);
    model.objects()[0].instances.front()->set_offset(Vec3d(30., 30., 0.));
    model.objects()[1].instances.front()->set_offset(Vec3d(100., 30., 0.));
    print.apply(model, config);
    print.set_status_silent();
    print.process();
    Test::GCode::select_printing_plan_firmware(print, "gcode.firmware.marlin2");
    const boost::filesystem::path output = Test::GCode::temporary_gcode_path();
    Orchestrator::instance().export_gcode(print, output.string(), nullptr);
    const std::string text = Test::GCode::read_text_file(output);
    boost::filesystem::remove(output);
    bool started = false, first_xy = true;
    double x = 0.0, y = 0.0, z = 0.1;
    size_t crossings = 0, deposits = 0, layer_entries = 0, island_travels = 0, brim_crossings = 0;
    bool brim = false, layer_entry = true;
    std::istringstream stream(text);
    std::string line;
    while (std::getline(stream, line)) {
        if (line == "M117 lift-start") { started = true; continue; }
        if (line == ";LAYER_CHANGE") layer_entry = true;
        if (line.rfind(";TYPE:", 0) == 0) brim = line.find("Skirt") != std::string::npos;
        if (!started || (line.rfind("G1 ", 0) != 0 && line.rfind("G0 ", 0) != 0)) continue;
        double next_x = x, next_y = y, next_z = z, e = 0.0;
        std::istringstream words(line.substr(3));
        std::string word;
        while (words >> word) {
            if (word[0] == ';') break;
            if (word.size() < 2) continue;
            if (word[0] == 'X') next_x = std::stod(word.substr(1));
            if (word[0] == 'Y') next_y = std::stod(word.substr(1));
            if (word[0] == 'Z') next_z = std::stod(word.substr(1));
            if (word[0] == 'E') e = std::stod(word.substr(1));
        }
        const bool xy = std::abs(next_x - x) > 0.001 || std::abs(next_y - y) > 0.001;
        INFO(line << " from " << x << "," << y << "," << z << " brim=" << brim);
        if (xy && first_xy) {
            CHECK(std::min(z, next_z) + 0.001 >= floor);
            if (floor == 0.0) CHECK(next_z < 1.0);
            first_xy = false;
        } else if (std::abs(next_x - x) > 20.0) {
            ++crossings;
            if (brim) ++brim_crossings;
            CHECK(e <= 0.0);
            CHECK(std::min(z, next_z) + 0.001 >= floor);
            if (floor == 0.0) CHECK(std::max(z, next_z) < 2.0);
        } else if (xy && e <= 0.0 && std::hypot(next_x - x, next_y - y) >= 4.0) {
            ++island_travels;
            CHECK(std::min(z, next_z) + 0.001 >= floor);
        } else if (xy && e <= 0.0) {
            if (layer_entry && !brim) {
                ++layer_entries;
                CHECK(std::min(z, next_z) + 0.001 >= floor);
            } else CHECK(std::max(z, next_z) < 2.0);
        }
        if (xy && e > 0.0) { ++deposits; layer_entry = false; CHECK(next_z < 2.0); }
        if (scenario == 7) CHECK(next_z <= 5.001);
        x = next_x; y = next_y; z = next_z;
    }
    CHECK_FALSE(first_xy);
    CHECK(crossings > 0);
    CHECK(deposits > 0);
    CHECK(layer_entries > 0);
    CHECK(island_travels > 0);
    if (scenario == 6) CHECK(brim_crossings > 0);
}

// Inspect the entire approach between deposits across every production group boundary,
// including firmware-only approaches, global auxiliary groups and tower source heights.
TEST_CASE("Stepped copy group entries clear all preceding plan heights",
          "[plugins][lift][travel][firmware][wipe-tower][group-clearance-export]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool by_object = GENERATE(false, true);
    const bool provider = GENERATE(false, true);
    const bool tower = GENERATE(false, true);
    const int scenario = GENERATE(0, 1, 2);
    const double floor = scenario == 0 ? 0.3 : 3.0;
    const double ceiling = scenario == 2 ? 1.6 : 200.0;
    CAPTURE(by_object, provider, tower, scenario);
    struct TravelActivation {
        Orchestrator &host;
        Plugin *plugin;
        bool previous;
        // Select whether production travel geometry is available; preserve global activation.
        TravelActivation(Orchestrator &value, bool enabled) : host(value),
            plugin(value.get_plugin("layer_extrusion_edit.travel.default")), previous(value.is_plugin_active(plugin)) {
            host.set_plugin_active(plugin, enabled);
        }
        // Restore the previous provider state even when export fails.
        ~TravelActivation() { host.set_plugin_active(plugin, previous); }
    } activation(Orchestrator::instance(), provider);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"print_plan_builder", by_object ? "ordering.plan_builder.by_object" : "ordering.plan_builder.by_layer"},
        {"complete_objects_step", "0.4"}, {"complete_objects_one_first_layer", "0"}, {"complete_islands", "0"},
        {"complete_objects_step_max_z", "0.8"},
        {"layer_height", "0.2"}, {"first_layer_height", "0.2"}, {"skirts", "1"}, {"skirt_height", "1"},
        {"skirt_per_object", "0"}, {"brim_width", "0"}, {"perimeters", "1"}, {"fill_density", "0%"},
        {"top_solid_layers", "0"}, {"bottom_solid_layers", "0"}, {"start_gcode", "G90\nG1 X0 Y0 Z0.1"}, {"end_gcode", ""},
        {"use_relative_e_distances", "1"}, {"binary_gcode", "0"}, {"lift_min", std::to_string(floor)},
        {"max_print_height", std::to_string(ceiling)}, {"retract_lift", "0.5"}, {"print_retract_lift", "0.5"},
        {"retract_length", "0"}, {"retract_lift_above", "100"}, {"retract_lift_below", "0.1"},
        {"retract_lift_before_travel", "1000"}, {"retract_lift_top", "Only on top"}, {"retract_lift_first_layer", "0"},
        {"wipe", "0"}, {"travel_ramping_lift", "1"}, {"travel_slope", "10"}});
    if (tower) {
        config.set_num_extruders(2);
        config.set_deserialize_strict({{"wipe_tower", "1"}, {"wipe_tower_x", "60"}, {"wipe_tower_y", "80"},
            {"wipe_tower_width", "20"}, {"wipe_tower_brim_width", "0"}, {"wiping_volumes_matrix", "0,140,140,0"},
            {"filament_minimal_purge_on_wipe_tower", "15,15"}, {"filament_diameter", "1.75,1.75"},
            {"extrusion_multiplier", "1,1"}, {"retract_lift", "0.5,0.5"}, {"retract_length", "0,0"}});
    }
    Model model;
    Print print;
    Test::init_print({make_cube(2., 2., 1.), make_cube(2., 2., 1.4), make_cube(2., 2., 1.2)}, print, model, config);
    for (size_t i = 0; i < model.objects().size(); ++i) {
        model.objects()[i].instances.front()->set_offset(Vec3d(30. + 50. * i, 30., 0.));
        if (tower) {
            TriangleMesh material = make_cube(2., 2., i == 1 ? 1.4 : i == 2 ? 1.2 : 1.);
            material.translate(Vec3f(0.f, 10.f, 0.f));
            model.objects()[i].add_volume(material)->config.set_key_value("extruder", new ConfigOptionInt(2));
        }
    }
    print.apply(model, config);
    print.set_status_silent();
    print.process();
    Test::GCode::select_printing_plan_firmware(print, "gcode.firmware.marlin2");
    const boost::filesystem::path output = Test::GCode::temporary_gcode_path();
    Orchestrator::instance().export_gcode(print, output.string(), nullptr);
    const std::string text = Test::GCode::read_text_file(output);
    boost::filesystem::remove(output);
    const Printing::PrintingPlan &plan = *print.printing_plan();
    if (by_object) REQUIRE(plan.groups.size() > 2);
    else REQUIRE(plan.groups.size() == 1);
    bool saw_tower = false, later_shared = false;
    for (size_t g = 0; g < plan.groups.size(); ++g) {
        size_t physical_copies = 0;
        for (const uint32_t id : plan.groups[g].printing_catalog_ids)
            if (plan.object_instances[id].object != nullptr) ++physical_copies;
        if (g > 0 && physical_copies > 1) later_shared = true;
        for (const Printing::PrintingLayerGroup &entry : plan.groups[g].layers)
            for (const Printing::PrintingToolGroup &tool : entry.tool_groups)
                for (const Printing::PrintingExtrusion &extrusion : tool.extrusions)
                    saw_tower |= extrusion.sregion_island_role == ExtrusionRole::WipeTower;
    }
    if (by_object) CHECK(later_shared);
    CHECK(saw_tower == tower);
    struct Move { double from_z, z; bool xy, deposit; };
    std::vector<Move> moves;
    std::vector<size_t> first, last;
    size_t layer = size_t(-1);
    double x = 0., y = 0., z = 0.;
    std::istringstream lines(text);
    std::string line;
    while (std::getline(lines, line)) {
        if (line == ";LAYER_CHANGE") {
            ++layer; first.push_back(size_t(-1)); last.push_back(size_t(-1));
        }
        if (line.rfind("G1 ", 0) != 0 && line.rfind("G0 ", 0) != 0) continue;
        double nx = x, ny = y, nz = z, e = 0.;
        std::istringstream words(line.substr(3));
        std::string word;
        while (words >> word) {
            if (word[0] == ';') break;
            if (word[0] == 'X') nx = std::stod(word.substr(1));
            if (word[0] == 'Y') ny = std::stod(word.substr(1));
            if (word[0] == 'Z') nz = std::stod(word.substr(1));
            if (word[0] == 'E') e = std::stod(word.substr(1));
        }
        const bool xy = std::hypot(nx - x, ny - y) > 0.001;
        if (xy && e > 0. && layer != size_t(-1)) {
            if (first[layer] == size_t(-1)) first[layer] = moves.size();
            last[layer] = moves.size();
        }
        moves.push_back({z, nz, xy, xy && e > 0.});
        x = nx; y = ny; z = nz;
    }
    size_t flat = 0, previous_last = size_t(-1), checked = 0;
    double maximum = 0.;
    for (size_t g = 0; g < plan.groups.size(); ++g) {
        bool entered = false;
        for (const Printing::PrintingLayerGroup &entry : plan.groups[g].layers) {
            REQUIRE(flat < first.size());
            if (first[flat] != size_t(-1)) {
                if (!entered && previous_last != size_t(-1)) {
                    const double clearance = std::min(ceiling, std::max(maximum + 0.5, floor));
                    size_t xy_moves = 0;
                    for (size_t i = previous_last + 1; i < first[flat]; ++i) {
                        INFO("group " << g << " move " << i << " clearance " << clearance);
                        CHECK(moves[i].z <= ceiling + 0.001);
                        if (xy_moves == 0 && !moves[i].xy)
                            CHECK(moves[i].z + 0.001 >= moves[i].from_z);
                        if (moves[i].xy) {
                            ++xy_moves;
                            CHECK_FALSE(moves[i].deposit);
                            CHECK(std::min(moves[i].from_z, moves[i].z) + 0.001 >= clearance);
                        }
                    }
                    CHECK(xy_moves == 1);
                    ++checked;
                }
                entered = true;
                previous_last = last[flat];
            }
            maximum = std::max(maximum, unscaled(entry.print_z));
            ++flat;
        }
    }
    CHECK(flat == first.size());
    CHECK(checked == (by_object ? plan.groups.size() - 1 : 0));
}

// Reconstruct copy-set transitions and finished-copy counts from the exported plan;
// scripts must use catalog IDs independently of label activation and object-list indices.
TEST_CASE("Copy-set scripts and instance placeholders follow production groups",
          "[plugins][gcode][firmware][script][group-script-export]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool by_object = GENERATE(false, true);
    const int scene = GENERATE(0, 1, 2);
    const bool labels = GENERATE(false, true);
    const bool before_move = GENERATE(false, true);
    const bool travel_provider = GENERATE(false, true);
    CAPTURE(by_object, scene, labels, before_move, travel_provider);
    struct TravelActivation {
        Orchestrator &host;
        Plugin *plugin;
        bool previous;
        // Select production travel generation and retain the caller's activation state.
        TravelActivation(Orchestrator &value, bool enabled) : host(value),
            plugin(value.get_plugin("layer_extrusion_edit.travel.default")), previous(value.is_plugin_active(plugin)) {
            host.set_plugin_active(plugin, enabled);
        }
        // Restore activation on normal completion and failed assertions alike.
        ~TravelActivation() { host.set_plugin_active(plugin, previous); }
    } activation(Orchestrator::instance(), travel_provider);
    const std::string context = " O{current_object_idx} C{current_instance_id} K{size(current_instances_id)}"
        " A{if size(current_instances_id)>0}{current_instances_id[0]}{else}-1{endif}"
        " B{if size(current_instances_id)>1}{current_instances_id[1]}{else}-1{endif}";
    const std::string transition = "M117 GO P{previous_object_id} N{next_object_id}"
        " U{size(previous_instances_id)} V{size(next_instances_id)}"
        " X{if size(previous_instances_id)>0}{previous_instances_id[0]}{else}-1{endif}"
        " Y{if size(previous_instances_id)>1}{previous_instances_id[1]}{else}-1{endif}"
        " Z{if size(next_instances_id)>0}{next_instances_id[0]}{else}-1{endif}"
        " W{if size(next_instances_id)>1}{next_instances_id[1]}{else}-1{endif}" + context;
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"print_plan_builder", by_object ? "ordering.plan_builder.by_object" : "ordering.plan_builder.by_layer"},
        {"complete_objects_step", "0"}, {"complete_objects_sort", "object"}, {"complete_objects_one_first_layer", "0"},
        {"complete_objects_one_skirt", "1"},
        {"complete_islands", scene == 1 ? "1" : "0"}, {"layer_height", "0.2"}, {"first_layer_height", "0.2"},
        {"skirts", scene == 2 ? "1" : "0"}, {"skirt_height", "1"}, {"skirt_per_object", "0"}, {"brim_width", "0"},
        {"perimeters", "1"}, {"fill_density", "0%"}, {"top_solid_layers", "0"}, {"bottom_solid_layers", "0"},
        {"start_gcode", "M117 OUTSTART" + context + " P{size(previous_instances_id)} N{size(next_instances_id)}"},
        {"end_gcode", "M117 OUTEND" + context + " P{size(previous_instances_id)} N{size(next_instances_id)}"},
        {"before_layer_gcode", "M117 CTX" + context}, {"between_objects_gcode", transition},
        {"between_objects_gcode_before_move", before_move ? "1" : "0"}, {"use_relative_e_distances", "1"},
        {"binary_gcode", "0"}, {"gcode_label_objects", labels ? "both" : "disabled"}, {"wipe_tower", "0"}, {"wipe", "0"}});
    if (scene == 2)
        config.set_key_value("extruder_clearance", new ConfigOptionGraphs({GraphData(0, 2,
            GraphData::GraphType::SQUARE, {{0, 100}, {100, 100}})}));
    Model model;
    Print print;
    TriangleMesh mesh = make_cube(2., 2., 1.2);
    if (scene == 1) {
        TriangleMesh island = mesh;
        island.translate(Vec3f(30.f, 0.f, 0.f));
        mesh.merge(island);
    }
    Test::init_print(scene == 2 ? std::vector<TriangleMesh>{mesh, make_cube(2., 2., 0.4)} :
        std::vector<TriangleMesh>{mesh}, print, model, config);
    model.objects()[0].instances.front()->set_offset(Vec3d(30., 30., 0.));
    if (scene == 0) model.objects()[0].add_instance(*model.objects()[0].instances.front())->set_offset(Vec3d(100., 30., 0.));
    if (scene == 2) model.objects()[1].instances.front()->set_offset(Vec3d(40., 30., 0.));
    if (scene != 1) {
        model.custom_gcode_per_print_z.mode = CustomGCode::SingleExtruder;
        model.custom_gcode_per_print_z.gcodes.push_back({scale_i(0.6), CustomGCode::Custom, 1, "", "M117 band"});
    }
    print.apply(model, config);
    print.set_status_silent();
    print.process();
    Test::GCode::select_printing_plan_firmware(print, "gcode.firmware.marlin2");
    const boost::filesystem::path output = Test::GCode::temporary_gcode_path();
    Orchestrator::instance().export_gcode(print, output.string(), nullptr);
    const std::string text = Test::GCode::read_text_file(output);
    boost::filesystem::remove(output);
    const Printing::PrintingPlan &plan = *print.printing_plan();
    std::vector<std::vector<int32_t>> groups;
    std::map<int32_t, size_t> last;
    for (size_t g = 0; g < plan.groups.size(); ++g) {
        std::set<int32_t> copies;
        for (uint32_t id : plan.groups[g].printing_catalog_ids)
            if (plan.object_instances[id].object) { copies.insert(int32_t(id)); last[int32_t(id)] = g; }
        groups.emplace_back(copies.begin(), copies.end());
    }
    // Produce the same compact marker format without using the placeholder implementation.
    const auto format_context = [](const std::vector<int32_t> &copies, int finished) {
        return " O" + std::to_string(finished) + " C" + std::to_string(copies.size() == 1 ? copies.front() : -1) +
            " K" + std::to_string(copies.size()) + " A" + std::to_string(copies.empty() ? -1 : copies[0]) +
            " B" + std::to_string(copies.size() < 2 ? -1 : copies[1]);
    };
    std::vector<int> finished(groups.size(), 0);
    for (size_t g = 0; g < groups.size(); ++g)
        for (const auto &[id, final_group] : last) if (final_group < g) ++finished[g];
    std::vector<std::string> expected_context, expected_go, actual_context, actual_go;
    std::vector<std::pair<size_t, size_t>> transition_groups;
    std::vector<std::vector<int32_t>> nonempty;
    size_t previous = size_t(-1);
    for (size_t g = 0; g < groups.size(); ++g) {
        for (size_t l = 0; l < plan.groups[g].layers.size(); ++l)
            expected_context.push_back("M117 CTX" + format_context(groups[g], finished[g]));
        if (groups[g].empty()) continue;
        nonempty.push_back(groups[g]);
        if (previous != size_t(-1) && groups[g] != groups[previous]) {
            transition_groups.emplace_back(previous, g);
            const std::vector<int32_t> &a = groups[previous], &b = groups[g];
            int p = -1, n = -1;
            for (uint32_t obj = 0; obj < print.objects().size(); ++obj) {
                if (a.size() == 1 && plan.object_instances[a[0]].object == &print.objects()[obj]) p = int(obj);
                if (b.size() == 1 && plan.object_instances[b[0]].object == &print.objects()[obj]) n = int(obj);
            }
            expected_go.push_back("M117 GO P" + std::to_string(p) + " N" + std::to_string(n) +
                " U" + std::to_string(a.size()) + " V" + std::to_string(b.size()) +
                " X" + std::to_string(a[0]) + " Y" + std::to_string(a.size() < 2 ? -1 : a[1]) +
                " Z" + std::to_string(b[0]) + " W" + std::to_string(b.size() < 2 ? -1 : b[1]) +
                format_context(groups[before_move ? previous : g], finished[before_move ? previous : g]));
        }
        previous = g;
    }
    std::istringstream lines(text);
    std::string line;
    size_t line_number = 0, layer_index = size_t(-1);
    std::vector<size_t> go_lines, travel_xy_lines, layer_groups;
    std::vector<size_t> first_deposit(groups.size(), size_t(-1)), last_deposit(groups.size(), size_t(-1));
    for (size_t g = 0; g < groups.size(); ++g)
        for (size_t l = 0; l < plan.groups[g].layers.size(); ++l) layer_groups.push_back(g);
    double x = 0., y = 0.;
    while (std::getline(lines, line)) {
        ++line_number;
        if (line == ";LAYER_CHANGE") ++layer_index;
        if (line.rfind("M117 CTX", 0) == 0) actual_context.push_back(line);
        if (line.rfind("M117 GO", 0) == 0) { actual_go.push_back(line); go_lines.push_back(line_number); }
        if (line.rfind("G1 ", 0) != 0 && line.rfind("G0 ", 0) != 0) continue;
        double nx = x, ny = y, e = 0.;
        std::istringstream words(line.substr(3));
        std::string word;
        while (words >> word) {
            if (word[0] == ';') break;
            if (word[0] == 'X') nx = std::stod(word.substr(1));
            if (word[0] == 'Y') ny = std::stod(word.substr(1));
            if (word[0] == 'E') e = std::stod(word.substr(1));
        }
        if (std::hypot(nx - x, ny - y) > 0.001) {
            if (e > 0. && layer_index < layer_groups.size()) {
                const size_t g = layer_groups[layer_index];
                if (first_deposit[g] == size_t(-1)) first_deposit[g] = line_number;
                last_deposit[g] = line_number;
            } else if (e <= 0.) travel_xy_lines.push_back(line_number);
        }
        x = nx; y = ny;
    }
    CHECK(actual_context == expected_context);
    CHECK(actual_go == expected_go);
    REQUIRE(go_lines.size() == transition_groups.size());
    for (size_t i = 0; i < transition_groups.size(); ++i) {
        const size_t source = transition_groups[i].first, target = transition_groups[i].second;
        REQUIRE(last_deposit[source] != size_t(-1));
        REQUIRE(first_deposit[target] != size_t(-1));
        std::vector<size_t> approach;
        for (size_t move : travel_xy_lines)
            if (move > last_deposit[source] && move < first_deposit[target]) approach.push_back(move);
        REQUIRE_FALSE(approach.empty());
        CHECK(go_lines[i] > last_deposit[source]);
        CHECK(go_lines[i] < first_deposit[target]);
        if (before_move) CHECK(go_lines[i] < approach.front());
        else CHECK(go_lines[i] > approach.back());
    }
    CHECK(text.find("M117 OUTSTART O0 C-1 K0 A-1 B-1 P0 N0") != std::string::npos);
    CHECK(text.find("M117 OUTEND" + format_context({}, finished.back()) + " P0 N0") != std::string::npos);
    if (!by_object) { CHECK(actual_go.empty()); for (int count : finished) CHECK(count == 0); }
    else if (scene == 0) {
        REQUIRE(nonempty.size() == 4);
        CHECK(nonempty[0] == nonempty[2]); CHECK(nonempty[1] == nonempty[3]); CHECK(actual_go.size() == 3);
    } else if (scene == 1) { CHECK(nonempty.size() > 1); CHECK(actual_go.empty()); }
    else { REQUIRE(nonempty.size() == 2); CHECK(groups[0].empty()); CHECK(nonempty[0].size() == 2); CHECK(nonempty[1].size() == 1); CHECK(actual_go.size() == 1); }
    const ApiHost::GCodeExportDescription metadata = ApiHost::copy_gcode_export_objects(reinterpret_cast<const print_handle *>(&print));
    for (const ApiHost::GCodeExportObject &object : metadata.objects) {
        CHECK(object.id < plan.object_instances.size());
        if (!object.real) continue;
        REQUIRE(object.object_index < print.objects().size());
        CHECK(plan.object_instances[object.id].object == &print.objects()[object.object_index]);
        CHECK(plan.object_instances[object.id].object_instance_idx == object.instance_index);
        CHECK(object.name == plan.object_instances[object.id].name);
        CHECK(last.count(int32_t(object.id)) == 1);
        if (labels) {
            CHECK(text.find("M486 S" + std::to_string(object.id) + "\n") != std::string::npos);
            CHECK(text.find("; printing object " + object.name) != std::string::npos);
        }
    }
}

// The absolute travel floor is limited by the printer ceiling without blocking slicing.
TEST_CASE("Validation warns when the minimum travel height exceeds the printer ceiling",
          "[plugins][validation][lift][lift-min-validation]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const double floor = GENERATE(0.0, 10.0, 12.0);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"lift_min", std::to_string(floor)}, {"max_print_height", "10"}});
    Model model;
    Print print;
    Test::init_print({make_cube(2., 2., 1.)}, print, model, config);
    std::vector<std::string> warnings;
    CHECK(print.validate(&warnings).first == PrintBase::PrintValidationError::pveNone);
    const std::string message = "_LIFT_MIN_CLAMPED";
    CHECK(std::count(warnings.begin(), warnings.end(), message) == (floor > 10.0 ? 1 : 0));
    CHECK(print.validate(nullptr).first == PrintBase::PrintValidationError::pveNone);
}

// Reconstruct source Z ranks independently, then compare first-layer fan/lift,
// layer scripts, the single bed switch and per-copy Bambu actions in real exports.
// Tower catalog entries must not become copies or change source ranks; P2 follows
// the same source rank as the part fan, and simultaneous second layers share a scan.
TEST_CASE("First-layer consumers follow each copy's source sequence",
          "[plugins][gcode][fans][temperature][lift][bambu][script][source-first-layer-export]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool by_object = GENERATE(false, true);
    const bool bambu = GENERATE(false, true);
    const bool raft = GENERATE(false, true);
    const bool tower = GENERATE(false, true);
    CAPTURE(by_object, bambu, raft, tower);
    struct TravelActivation {
        Orchestrator &host;
        Plugin *plugin;
        bool active;
        // Enable the real travel provider, which the shared test runtime disables.
        explicit TravelActivation(Orchestrator &value) : host(value),
            plugin(value.get_plugin("layer_extrusion_edit.travel.default")), active(value.is_plugin_active(plugin)) {
            host.set_plugin_active(plugin, true);
        }
        // Restore the exact activation state even after a fatal assertion.
        ~TravelActivation() { host.set_plugin_active(plugin, active); }
    } travel(Orchestrator::instance());
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"print_plan_builder", by_object ? "ordering.plan_builder.by_object" : "ordering.plan_builder.by_layer"},
        {"complete_objects_step", "0.4"}, {"complete_islands", "0"}, {"complete_objects_one_first_layer", "0"},
        {"layer_height", "0.2"}, {"first_layer_height", "0.2"}, {"skirts", "1"}, {"skirt_height", "1"},
        {"skirt_per_object", "0"}, {"brim_width", "0"}, {"perimeters", "1"}, {"fill_density", "0%"},
        {"top_solid_layers", "0"}, {"bottom_solid_layers", "0"}, {"start_gcode", ""}, {"end_gcode", ""},
        {"layer_gcode", "M117 source-layer"}, {"use_relative_e_distances", "1"}, {"binary_gcode", "0"},
        {"default_fan_speed", "100"}, {"perimeter_fan_speed", "100"}, {"external_perimeter_fan_speed", "100"},
        {"disable_fan_first_layers", "1"}, {"full_fan_speed_layer", "0"},
        {"retract_lift", "0.5"}, {"print_retract_lift", "0.5"}, {"retract_lift_first_layer", "1"}, {"retract_lift_above", "10"},
        {"retract_before_travel", "0"}, {"retract_lift_before_travel", "0"}, {"retract_length", "1"},
        {"travel_ramping_lift", "0"}, {"wipe", "0"}, {"retract_lift_top", "All surfaces"},
        {"print_first_layer_bed_temperature", "60"}, {"print_bed_temperature", "50"},
        {"printer_custom_variables", "scan_first_layer=1\nenable_power_loss_recovery=enable"},
        {"raft_layers", raft ? "2" : "0"}});
    config.set_deserialize_strict("platter_fan", "1");
    config.set_deserialize_strict("platter_fan_speed", "70");
    if (tower) {
        config.set_num_extruders(2);
        config.set_deserialize_strict({{"wipe_tower", "1"}, {"wipe_tower_x", "60"}, {"wipe_tower_y", "60"},
            {"wipe_tower_width", "20"}, {"wipe_tower_brim_width", "0"}, {"wiping_volumes_matrix", "0,140,140,0"},
            {"filament_minimal_purge_on_wipe_tower", "15,15"}, {"filament_diameter", "1.75,1.75"},
            {"extrusion_multiplier", "1,1"}, {"default_fan_speed", "100,100"},
            {"perimeter_fan_speed", "100,100"}, {"external_perimeter_fan_speed", "100,100"},
            {"disable_fan_first_layers", "1,1"}, {"full_fan_speed_layer", "0,0"}, {"platter_fan_speed", "70,70"}});
    }
    Model model;
    Print print;
    TriangleMesh short_mesh = make_cube(2., 2., 1.);
    TriangleMesh island = make_cube(2., 2., 1.);
    island.translate(Vec3f(10.f, 0.f, 0.f));
    short_mesh.merge(island);
    TriangleMesh tall_mesh = short_mesh;
    tall_mesh.scale(Vec3f(1.f, 1.f, 1.4f));
    Test::init_print({short_mesh, tall_mesh}, print, model, config);
    if (tower) {
        for (size_t obj = 0; obj < model.objects().size(); ++obj) {
            TriangleMesh second_material = make_cube(2., 2., obj == 0 ? 1. : 1.4);
            second_material.translate(Vec3f(0.f, 10.f, 0.f));
            model.objects()[obj].add_volume(second_material)->config.set_key_value("extruder", new ConfigOptionInt(2));
        }
    }
    model.objects()[0].instances.front()->set_offset(Vec3d(30., 30., 0.));
    model.objects()[1].instances.front()->set_offset(Vec3d(100., 30., 0.));
    model.objects()[1].config.set_deserialize_strict("first_layer_height", "0.4");
    print.apply(model, config);
    print.set_status_silent();
    print.process();
    Test::GCode::select_printing_plan_firmware(print, bambu ? "gcode.firmware.bambu" : "gcode.firmware.marlin2");
    const boost::filesystem::path output = Test::GCode::temporary_gcode_path();
    Orchestrator::instance().export_gcode(print, output.string(), nullptr);
    const std::string text = Test::GCode::read_text_file(output);
    Test::GCode::remove_output_pair(output);
    REQUIRE(print.printing_plan() != nullptr);
    const Printing::PrintingPlan &plan = *print.printing_plan();
    std::map<const PrintObject *, std::vector<coord_t>> sequences;
    const slic3r_api::SourceLayerIndices actual_indices = slic3r_api::source_layer_indices(
        slic3r_api::Print(reinterpret_cast<const print_handle *>(&print)),
        slic3r_api::ConstPrintingPlan(reinterpret_cast<const printing_plan_handle *>(&plan)));
    std::map<const LayerRegionIsland *, coord_t> source_z;
    std::set<coord_t> plate;
    for (const PrintObject &object : print.objects()) {
        std::set<coord_t> zs;
        for (const Layer *layer : [&object]() {
            std::vector<const Layer *> layers;
            for (const Layer &value : object.layers()) layers.push_back(&value);
            for (const Layer &value : object.auxiliary_layers()) layers.push_back(&value);
            return layers;
        }()) {
            zs.insert(layer->scaled_print_z());
            for (const LayerSliceIsland &slice : layer->islands())
                for (const LayerRegionIsland &region : slice.regions_islands()) source_z[&region] = layer->scaled_print_z();
        }
        sequences[&object] = std::vector<coord_t>(zs.begin(), zs.end());
    }
    REQUIRE(print.auxiliary_object() != nullptr);
    REQUIRE_FALSE(print.auxiliary_object()->auxiliary_layers().empty());
    for (const Printing::PrintingGroup &group : plan.groups)
        for (const Printing::PrintingLayerGroup &layer : group.layers) plate.insert(layer.print_z);
    std::vector<bool> first;
    std::vector<double> zs;
    std::vector<std::map<uint32_t, size_t>> copies;
    size_t expected_scripts = 0;
    size_t group_index = 0;
    bool saw_tower = false;
    for (const Printing::PrintingGroup &group : plan.groups) {
        size_t layer_index = 0;
        for (const Printing::PrintingLayerGroup &layer : group.layers) {
            std::map<uint32_t, size_t> ranks;
            for (const Printing::PrintingToolGroup &tool : layer.tool_groups)
                for (const Printing::PrintingExtrusion &extrusion : tool.extrusions)
                    if (extrusion.printing_catalog_id < plan.object_instances.size() &&
                        plan.object_instances[extrusion.printing_catalog_id].object != nullptr) {
                        const std::vector<coord_t> &sequence = sequences.at(plan.object_instances[extrusion.printing_catalog_id].object);
                        REQUIRE(source_z.count(extrusion.region_island) == 1);
                        const size_t rank = size_t(std::lower_bound(sequence.begin(), sequence.end(), source_z.at(extrusion.region_island)) - sequence.begin());
                        ranks.emplace(extrusion.printing_catalog_id, rank);
                    } else if (extrusion.printing_catalog_id < plan.object_instances.size()) {
                        saw_tower = true;
                    }
            bool is_first = layer.print_z == *plate.begin();
            if (!ranks.empty()) {
                is_first = false;
                for (const auto &[id, rank] : ranks) is_first |= rank == 0;
            }
            first.push_back(is_first);
            zs.push_back(unscaled(layer.print_z));
            copies.push_back(ranks);
            const slic3r_api::SourceLayerIndex &actual = actual_indices[group_index][layer_index++];
            CHECK(actual.copies.size() == ranks.size());
            size_t expected_rank = size_t(std::distance(plate.begin(), plate.lower_bound(layer.print_z)));
            if (!ranks.empty()) {
                expected_rank = ranks.begin()->second;
                for (const auto &[id, rank] : ranks) expected_rank = std::min(expected_rank, rank);
            }
            CHECK(actual.index == expected_rank);
            for (const slic3r_api::CopyLayerIndex &copy : actual.copies) {
                REQUIRE(ranks.count(copy.id) == 1);
                CHECK(copy.index == ranks.at(copy.id));
            }
            if (!is_first) ++expected_scripts;
        }
        ++group_index;
    }
    if (tower) CHECK(saw_tower);
    std::map<uint32_t, size_t> last;
    for (size_t l = 0; l < copies.size(); ++l)
        for (const auto &[id, rank] : copies[l]) last[id] = l;
    std::set<uint32_t> started, active;
    size_t expected_stops = 0, overlap_stops = 0;
    std::vector<size_t> expected_starts(copies.size(), 0), expected_ends(copies.size(), 0);
    for (size_t l = 0; l < copies.size(); ++l) {
        for (const auto &[id, rank] : copies[l]) if (rank == 1 && started.insert(id).second) {
            active.insert(id);
            expected_starts[l] = 1;
        }
        bool ending = false;
        for (const auto &[id, rank] : copies[l]) if (last[id] == l) { active.erase(id); ending = true; }
        if (ending && active.empty()) { ++expected_stops; ++expected_ends[l]; }
        if (ending && !active.empty()) ++overlap_stops;
    }
    CHECK(started.size() == 2);
    CHECK(overlap_stops > 0);
    size_t layer = size_t(-1), scripts = 0, bed_switches = 0, initial_bed = 0, starts = 0, stops = 0, scans = 0;
    int fan = -1, platter_fan = -1;
    std::vector<size_t> deposits(first.size(), 0);
    std::vector<size_t> lift_peaks(first.size(), 0);
    std::vector<size_t> actual_starts(first.size(), 0), actual_ends(first.size(), 0), actual_scans(first.size(), 0);
    std::istringstream stream(text);
    std::string line;
    while (std::getline(stream, line)) {
        if (line.rfind(";Z:", 0) == 0) { ++layer; REQUIRE(layer < zs.size()); CHECK(std::stod(line.substr(3)) == Approx(zs[layer])); }
        if (line == "M117 source-layer") { ++scripts; REQUIRE(layer < first.size()); CHECK_FALSE(first[layer]); }
        if (line == "M140 S50") ++bed_switches;
        if (line == "M190 S60") ++initial_bed;
        if (line == "M1003 S1") { REQUIRE(layer < first.size()); ++starts; ++actual_starts[layer]; }
        if (line == "M1003 S0") { REQUIRE(layer < first.size()); ++stops; ++actual_ends[layer]; }
        if (line.rfind("M976 S1 P1", 0) == 0) { REQUIRE(layer < first.size()); ++scans; ++actual_scans[layer]; }
        if (line.rfind("M106 P2 S", 0) == 0) platter_fan = std::stoi(line.substr(9));
        if (line.rfind("M107", 0) == 0 && line.find("P2") == std::string::npos && line.find("P3") == std::string::npos) fan = 0;
        if (line.rfind("M106", 0) == 0 && line.find("P2") == std::string::npos && line.find("P3") == std::string::npos) {
            const size_t s = line.find('S');
            if (s != std::string::npos) fan = std::stoi(line.substr(s + 1));
        }
        const size_t e = line.find(" E");
        const size_t move_z = line.find(" Z");
        if (layer < zs.size() && move_z != std::string::npos &&
            (line.rfind("G0 ", 0) == 0 || line.rfind("G1 ", 0) == 0) &&
            std::abs(std::stod(line.substr(move_z + 2)) - zs[layer] - 0.5) < 0.00001)
            ++lift_peaks[layer];
        if (line.rfind("G1 ", 0) == 0 && e != std::string::npos &&
            (line.find(" X") != std::string::npos || line.find(" Y") != std::string::npos) && std::stod(line.substr(e + 2)) > 0.) {
            REQUIRE(layer < first.size());
            ++deposits[layer];
            CHECK((fan == 0) == first[layer]);
            if (bambu) CHECK((platter_fan == 0) == first[layer]);
        }
    }
    CHECK(scripts == expected_scripts);
    CHECK(bed_switches == 1);
    CHECK(initial_bed == 1);
    for (size_t count : deposits) CHECK(count > 0);
    for (size_t l = 0; l < first.size(); ++l) {
        INFO(text);
        CAPTURE(l, zs[l], first[l]);
        if (first[l] && !copies[l].empty()) CHECK(lift_peaks[l] > 0);
        if (!first[l]) CHECK(lift_peaks[l] == 0);
    }
    if (bambu) {
        const size_t expected_start_count = std::count(expected_starts.begin(), expected_starts.end(), size_t(1));
        CHECK(starts == expected_start_count); CHECK(scans == expected_start_count); CHECK(stops == expected_stops);
        if (tower && !raft) { CHECK(starts == 2); CHECK(scans == 2); }
        CHECK(actual_starts == expected_starts);
        CHECK(actual_scans == expected_starts);
        CHECK(actual_ends == expected_ends);
    }
}

// Each physical copy must retain complete source coverage, including the by-layer tail.
TEST_CASE("Island stacks print every instance and retain layers above the height limit",
          "[plugins][printing-plan][parallel-islands]")
{
    for (const double limit : {0., 0.1, 0.8, 1.5, 10.})
        check_scene("copies-limit", 0, 1., false, Expect::Invariants, 2, limit);
}

void check_scene(const std::string &name, int shape, double step, bool close, Expect expectation,
                 uint32_t copies, double max_z)
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{whole_objects_key, "ordering.plan_builder.by_object"}, {step_key, std::to_string(step)},
        {limit_key, std::to_string(max_z)}, {islands_key, "1"}, {"layer_height", "0.2"}, {"first_layer_height", "0.2"},
        {"complete_objects_one_first_layer", "1"},
        {"seam_visibility", "0"}, {"skirts", "0"}, {"brim_width", "0"}, {"perimeters", "1"},
        {"fill_density", "0%"}, {"top_solid_layers", "0"}, {"bottom_solid_layers", "0"},
        {"start_gcode", ""}, {"end_gcode", ""}, {"binary_gcode", "0"}});
    if (shape == 1) {
        config.set_key_value("extruder_clearance_radius", new ConfigOptionFloat(60.));
        config.set_key_value("extruder_clearance", new ConfigOptionGraphs({GraphData(0, 3,
            GraphData::GraphType::LINEAR, {{0, 0}, {50, 0.1}, {51, 1.9}})}));
    }
    if (shape == 2) {
        config.set_key_value("extruder_clearance_radius", new ConfigOptionFloat(60.));
        config.set_key_value("extruder_clearance", new ConfigOptionGraphs({GraphData(0, 2,
            GraphData::GraphType::SQUARE, {{0, 0}, {2, 0.1}})}));
    }
    if (name == "high-step")
        config.set_key_value("extruder_clearance_radius", new ConfigOptionFloat(1000.));
    if (shape == 3)
        config.set_key_value("extruder_clearance_radius", new ConfigOptionFloat(0.));
    if (shape == 4)
        config.set_key_value("extruder_clearance", new ConfigOptionGraphs({GraphData(0, 4,
            GraphData::GraphType::LINEAR, {{0, 0}, {2, 1.9}, {3, 40}, {100, 50}})}));
    if (shape == 5)
        config.set_key_value("extruder_clearance", new ConfigOptionGraphs({GraphData(0, 2,
            GraphData::GraphType::SQUARE, {{0, 0}, {2, 1.9}})}));
    if (shape == 6)
        config.set_key_value("extruder_clearance", new ConfigOptionGraphs({GraphData(0, 3,
            GraphData::GraphType::LINEAR, {{0, 0}, {2, 0.5}, {120, 2}})}));
    TriangleMesh mesh = make_cube(20., 60., 1.);
    TriangleMesh first = make_cube(5., 5., 4.);
    TriangleMesh second = make_cube(5., 5., 3.8);
    first.translate(Vec3f(7.5f, 7.5f, 1.f));
    second.translate(Vec3f(7.5f, close ? 13.5f : 47.5f, 1.f));
    mesh.merge(first);
    mesh.merge(second);
    Model model;
    Print print;
    Test::init_print({mesh}, print, model, config);
    if (copies == 2) {
        model.objects().front().instances.front()->set_offset(Vec3d(30., 30., 0.));
        model.objects().front().add_instance(*model.objects().front().instances.front())->set_offset(Vec3d(110., 30., 0.));
        print.apply(model, config);
    }
    print.set_status_silent();
    print.process();
    Test::GCode::select_printing_plan_firmware(print);
    const boost::filesystem::path output = Test::GCode::temporary_gcode_path();
    Orchestrator::instance().export_gcode(print, output.string(), nullptr);
    const std::string text = Test::GCode::read_text_file(output);
    Test::GCode::remove_output_pair(output);
    REQUIRE(print.printing_plan() != nullptr);
    REQUIRE(print.objects().size() == 1);
    std::map<const LayerRegionIsland *, std::pair<int, int>> identities;
    std::set<std::pair<int, int>> expected;
    double split_y = 0.;
    for (const Layer &layer : print.objects().front().layers()) {
        const int z = int(std::lround(layer.unscaled_print_z() / 0.2));
        if (z == 6) {
            REQUIRE(layer.islands().size() == 2);
            split_y = 0.5 * (layer.island(0).get_bounding_box().center().y() +
                             layer.island(1).get_bounding_box().center().y());
        }
        for (const LayerSliceIsland &island : layer.islands()) {
            const int id = z <= 5 ? 0 : island.get_bounding_box().center().y() < split_y ? 1 : 2;
            expected.emplace(z, id);
            for (const LayerRegionIsland &region : island.regions_islands()) identities[&region] = {z, id};
        }
    }
    REQUIRE(expected.size() == 44);
    std::vector<double> markers;
    std::istringstream lines(text);
    std::string line;
    while (std::getline(lines, line))
        if (line.rfind(";Z:", 0) == 0) markers.push_back(std::stod(line.substr(3)));
    std::vector<std::vector<std::pair<int, int>>> visits_by_instance(copies);
    size_t marker = 0;
    for (const Printing::PrintingGroup &group : print.printing_plan()->groups)
        for (const Printing::PrintingLayerGroup &layer : group.layers) {
            REQUIRE(marker < markers.size());
            const double marker_z = markers.at(marker++);
            CHECK(marker_z == Approx(unscaled(layer.print_z)).margin(0.00001));
            std::set<std::pair<uint32_t, int>> seen;
            for (const Printing::PrintingToolGroup &tool : layer.tool_groups)
                for (const Printing::PrintingExtrusion &extrusion : tool.extrusions) {
                    if (extrusion.region_island == nullptr) continue;
                    REQUIRE(identities.count(extrusion.region_island) == 1);
                    const std::pair<int, int> source = identities.at(extrusion.region_island);
                    CHECK(source.first * 0.2 == Approx(unscaled(layer.print_z)).margin(0.00001));
                    REQUIRE(extrusion.printing_catalog_id < print.printing_plan()->object_instances.size());
                    const uint32_t instance = print.printing_plan()->object_instances[extrusion.printing_catalog_id].object_instance_idx;
                    REQUIRE(instance < copies);
                    if (seen.insert({instance, source.second}).second) visits_by_instance[instance].push_back(source);
                }
        }
    CHECK(marker == markers.size());
    // Every island stack names exactly the physical instances it actually prints.
    for (size_t idx = 1; idx < print.printing_plan()->groups.size(); ++idx) {
        const Printing::PrintingGroup &group = print.printing_plan()->groups[idx];
        if (max_z > 0. && max_z <= 5. && idx + 1 == print.printing_plan()->groups.size())
            break;
        std::set<uint32_t> printed;
        for (const Printing::PrintingLayerGroup &layer : group.layers)
            for (const Printing::PrintingToolGroup &tool : layer.tool_groups)
                for (const Printing::PrintingExtrusion &extrusion : tool.extrusions)
                    printed.insert(extrusion.printing_catalog_id);
        CHECK(printed == std::set<uint32_t>(group.printing_catalog_ids.begin(), group.printing_catalog_ids.end()));
    }
    if (max_z > 0. && max_z <= 5.) {
        const Printing::PrintingGroup &tail = print.printing_plan()->groups.back();
        REQUIRE(tail.printing_catalog_ids.size() == copies);
        coord_t previous = 0;
        for (const Printing::PrintingLayerGroup &layer : tail.layers) {
            CHECK(unscaled(layer.print_z) >= max_z - 0.00001);
            CHECK(layer.print_z > previous);
            previous = layer.print_z;
        }
        for (size_t idx = 1; idx + 1 < print.printing_plan()->groups.size(); ++idx)
            for (const Printing::PrintingLayerGroup &layer : print.printing_plan()->groups[idx].layers)
                CHECK(unscaled(layer.print_z) < max_z);
    }
    for (const std::vector<std::pair<int, int>> &visits : visits_by_instance) {
        std::set<std::pair<int, int>> actual;
        std::array<int, 3> highest{{0, 0, 0}};
        std::vector<int> runs;
        int previous_id = 0;
        int previous_z = 0;
        int first_tower = 0;
        for (const std::pair<int, int> &visit : visits) {
            const int z = visit.first, id = visit.second;
            CAPTURE(name, z, id);
            CHECK(actual.insert(visit).second);
            CHECK(z > highest[id]);
            if (id != 0) {
                if (first_tower == 0) first_tower = id;
                CHECK(highest[0] == 5);
                CHECK(highest[3 - id] * 0.2 <= z * 0.2 + step + 0.00001);
                if (runs.empty() || previous_id != id) runs.push_back(0);
                ++runs.back();
            }
            if (expectation == Expect::LayerByLayer) CHECK(z >= previous_z);
            highest[id] = z;
            previous_id = id;
            previous_z = z;
        }
        CHECK(actual == expected);
        if (expectation == Expect::Alternate) {
            INFO("Intermediate clearance must build alternating stacks, not a layer-by-layer plan");
            REQUIRE(runs.size() == 5);
            const std::vector<int> expected_runs = first_tower == 1 ?
                std::vector<int>{6, 12, 12, 7, 2} : std::vector<int>{6, 12, 12, 8, 1};
            CHECK(runs == expected_runs);
        }
    }
}
}
