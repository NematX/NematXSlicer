///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#include <catch2/catch.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <set>
#include <sstream>
#include <boost/filesystem.hpp>
#include "gcode_test_helpers.hpp"
#include "plugin_test_helpers.hpp"
#include "test_data.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
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
Stack groups advance through the copies in order, so no copy leads by more than
one stack before the shared by-layer tail.

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
// Shape 0 uses the default graph, 1 is wide and 2 is flat. Close places tower
// centers 6 mm apart rather than 40 mm. Every expectation checks all invariants;
// Alternate also checks stack run lengths, and LayerByLayer checks ascending global Z.
void check_scene(const std::string &name, int shape, double step, bool close, Expect expectation,
                 uint32_t copies = 1, double max_z = 0.);

// Require the alternating stack sizes for a nozzle that clears the distant towers.
TEST_CASE("Parallel islands alternate with intermediate nozzle clearance",
          "[plugins][printing-plan][parallel-islands]")
{
    check_scene("intermediate", 0, 1., false, Expect::Alternate);
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
        {"seam_visibility", "0"}, {"skirts", "0"}, {"brim_width", "0"}, {"perimeters", "1"},
        {"fill_density", "0%"}, {"top_solid_layers", "0"}, {"bottom_solid_layers", "0"},
        {"start_gcode", ""}, {"end_gcode", ""}, {"binary_gcode", "0"}});
    if (shape == 1)
        config.set_key_value("extruder_clearance", new ConfigOptionGraphs({GraphData(0, 3,
            GraphData::GraphType::LINEAR, {{0, 0}, {50, 0.1}, {51, 1.9}})}));
    if (shape == 2)
        config.set_key_value("extruder_clearance", new ConfigOptionGraphs({GraphData(0, 2,
            GraphData::GraphType::LINEAR, {{0, 0}, {2, 0.1}})}));
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
    // Skip first-pass groups and the shared tail when checking stack-major progress.
    std::vector<size_t> completed_stacks(copies, 0);
    size_t next_instance = 0;
    for (size_t idx = copies; idx < print.printing_plan()->groups.size(); ++idx) {
        const Printing::PrintingGroup &group = print.printing_plan()->groups[idx];
        if (max_z > 0. && max_z <= 5. && idx + 1 == print.printing_plan()->groups.size())
            break;
        REQUIRE(group.printing_catalog_ids.size() == 1);
        const uint32_t instance = print.printing_plan()->object_instances[group.printing_catalog_ids.front()].object_instance_idx;
        REQUIRE(instance < copies);
        CHECK(instance == next_instance);
        next_instance = (next_instance + 1) % copies;
        ++completed_stacks[instance];
        const std::pair<std::vector<size_t>::const_iterator, std::vector<size_t>::const_iterator> bounds =
            std::minmax_element(completed_stacks.cbegin(), completed_stacks.cend());
        CHECK(*bounds.second - *bounds.first <= 1);
    }
    CHECK(next_instance == 0);
    if (max_z > 0. && max_z <= 5.) {
        const Printing::PrintingGroup &tail = print.printing_plan()->groups.back();
        REQUIRE(tail.printing_catalog_ids.size() == copies);
        coord_t previous = 0;
        for (const Printing::PrintingLayerGroup &layer : tail.layers) {
            CHECK(unscaled(layer.print_z) >= max_z - 0.00001);
            CHECK(layer.print_z > previous);
            previous = layer.print_z;
        }
        for (size_t idx = copies; idx + 1 < print.printing_plan()->groups.size(); ++idx)
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
