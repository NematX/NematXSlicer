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
#include "libslic3r/CustomGCode.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"

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
