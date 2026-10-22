#include <catch2/catch.hpp>

#include <cstdint>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "gcode_test_helpers.hpp"
#include "layer_extrusion_edit_transition_test_helpers.hpp"
#include "support_grid_test_helpers.hpp"

#include "libslic3r/Api/plugin/cpp/AuxiliaryLayerHelpers.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Steps/StepExtrusionOrdering.hpp"

/*
Support and brim/skirt acceleration.
The default acceleration plugin roots support acceleration on
default_acceleration, the support interface on the support acceleration,
and brim/skirt on the support acceleration too; zero in any of them inherits
its base. Unit cases publish one support, one interface and one skirt leaf
on a small auxiliary layer, let ordering clone them into the PrintingPlan,
run only the acceleration plugin and read the effective acceleration of
each leaf. A final case slices a real object with grid support, exports it
and checks the print acceleration (Marlin M204 P) active on support moves.

Representative direct calls (branches are not a timeline):

    TEST_CASE
        |-- Test::Plugins::ensure_plugin_test_runtime_initialized()
        |-- Test::SupportGrid::config()
        |-- Test::init_print()
        |-- print.process()
        |-- export_gcode()
        `-- Test::GCode::read_text_file()
*/

using namespace Slic3r;
using namespace Slic3r::Printing;

namespace {
constexpr const char *DEFAULT_ACCELERATION_PLUGIN = "layer_extrusion_edit.acceleration.default";

// Effective accelerations of the three leaves published by support_accelerations().
struct SupportAccelerations
{
    float support = -1.f;
    float interface = -1.f;
    float brim = -1.f;
};

// Build one auxiliary layer on an unsliced cube object, publish one support,
// interface and skirt leaf, order them and run only the acceleration plugin.
// The layer sits on the bed, so the first-layer rules apply to it.
SupportAccelerations support_accelerations(std::initializer_list<ConfigBase::SetDeserializeItem> overrides);

// Walk one cloned tree with inherited acceleration and record it per leaf role.
void collect_accelerations(const ExtrusionEntity &entity, float acceleration, std::map<uint16_t, float> &out);

SupportAccelerations support_accelerations(std::initializer_list<ConfigBase::SetDeserializeItem> overrides)
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"default_acceleration", "1000"}, {"first_layer_acceleration", "100%"},
        {"machine_limits_usage", "ignore"}});
    config.set_deserialize_strict(overrides);
    Model model;
    Print print;
    PluginStorage storage;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);

    const ExPolygons subject{ExPolygon(Polygon({Point(scale_i(10.), scale_i(10.)), Point(scale_i(30.), scale_i(10.)),
        Point(scale_i(30.), scale_i(30.)), Point(scale_i(10.), scale_i(30.))}))};
    const slic3r_api::AuxiliaryLayerBuildResult result = slic3r_api::build_auxiliary_layer_regions_from_subject(
        reinterpret_cast<storage_handle *>(&storage),
        slic3r_api::Print(reinterpret_cast<const print_handle *>(&print)),
        slic3r_api::Object(reinterpret_cast<const object_handle *>(&print.object(0))),
        slic3r_api::ExPolygonCollection(reinterpret_cast<const expolygon_collection_handle *>(&subject)),
        scale_i(0.2), scale_i(0.2), scale_i(0.1));
    REQUIRE(result.created);
    REQUIRE(result.layer.island_count() == 1);
    layer_region_island_handle *region_island = layer_island_get_or_create_region_island(
        const_cast<layer_island_handle *>(result.layer.island(0).handle()), nullptr, 0, 0);
    REQUIRE(region_island != nullptr);
    ExtrusionEntityCollection &root = *reinterpret_cast<ExtrusionEntityCollection *>(
        layer_region_island_get_mutable_extrusion(region_island, RAW_EXTRUSION_ROLE_PERIMETER));
    const ExtrusionRole roles[] = {ExtrusionRole::SupportMaterial, ExtrusionRole::SupportMaterialInterface,
        ExtrusionRole::Skirt};
    for (size_t idx = 0; idx < 3; ++idx) {
        std::unique_ptr<ExtrusionPath> path = std::make_unique<ExtrusionPath>(
            ExtrusionAttributes(roles[idx], ExtrusionFlow(0.2, 0.4f, 0.2f)), nullptr, true);
        path->polyline().append(Point(scale_i(11.), scale_i(11. + double(idx))));
        path->polyline().append(Point(scale_i(29.), scale_i(11. + double(idx))));
        root.append(std::move(path));
    }

    Steps::StepExtrusionOrdering::clear(print);
    Steps::StepExtrusionOrdering::prepare(print);
    Steps::StepExtrusionOrdering::run_step(Orchestrator::instance(), print);
    REQUIRE(print.printing_plan() != nullptr);
    Test::TransitionPipeline::run_layer_plugins(print, {DEFAULT_ACCELERATION_PLUGIN});

    std::map<uint16_t, float> by_role;
    for (const PrintingGroup &group : print.printing_plan()->groups)
        for (const PrintingLayerGroup &layer : group.layers)
            for (const PrintingToolGroup &tool : layer.tool_groups)
                for (const PrintingExtrusion &extrusion : tool.extrusions)
                    collect_accelerations(*extrusion.root, -1.f, by_role);
    REQUIRE(by_role.size() == 3);
    return SupportAccelerations{by_role.at(uint16_t(ExtrusionRole::SupportMaterial)),
        by_role.at(uint16_t(ExtrusionRole::SupportMaterialInterface)), by_role.at(uint16_t(ExtrusionRole::Skirt))};
}

void collect_accelerations(const ExtrusionEntity &entity, float acceleration, std::map<uint16_t, float> &out)
{
    if (const ExtrusionPropertySpeed *process = entity.get_property<ExtrusionPropertySpeed>())
        if (process->accel_mm_per_s2 > 0.f)
            acceleration = process->accel_mm_per_s2;
    for (size_t child_idx = 0; child_idx < entity.child_count(); ++child_idx)
        collect_accelerations(entity.child(child_idx), acceleration, out);
    const ExtrusionAttributes *attributes = entity.get_property<ExtrusionAttributes>();
    if (entity.child_count() == 0 && entity.has_polyline() && attributes != nullptr)
        out[uint16_t(attributes->extrusion_role()())] = acceleration;
}
}

// Each section changes one setting of the support chain and checks all three roles.
TEST_CASE("Acceleration plugin resolves support interface and brim accelerations",
          "[plugins][layer-extrusion-edit][acceleration][support]")
{
    SECTION("a fixed support acceleration is the base of interface and brim percentages") {
        const SupportAccelerations accelerations = support_accelerations({{"support_material_acceleration", "600"},
            {"support_material_interface_acceleration", "50%"}, {"brim_acceleration", "50%"}});
        CHECK(accelerations.support == 600.f);
        CHECK(accelerations.interface == 300.f);
        CHECK(accelerations.brim == 300.f);
    }

    SECTION("a support percentage applies to the default acceleration") {
        const SupportAccelerations accelerations = support_accelerations({{"support_material_acceleration", "80%"},
            {"support_material_interface_acceleration", "0"}, {"brim_acceleration", "0"}});
        CHECK(accelerations.support == 800.f);
        CHECK(accelerations.interface == 800.f);
        CHECK(accelerations.brim == 800.f);
    }

    SECTION("zero interface and brim accelerations inherit the support acceleration") {
        const SupportAccelerations accelerations = support_accelerations({{"support_material_acceleration", "600"},
            {"support_material_interface_acceleration", "0"}, {"brim_acceleration", "0"}});
        CHECK(accelerations.support == 600.f);
        CHECK(accelerations.interface == 600.f);
        CHECK(accelerations.brim == 600.f);
    }

    SECTION("a zero support acceleration inherits the default acceleration") {
        const SupportAccelerations accelerations = support_accelerations({{"support_material_acceleration", "0"},
            {"support_material_interface_acceleration", "50%"}, {"brim_acceleration", "40%"}});
        CHECK(accelerations.support == 1000.f);
        CHECK(accelerations.interface == 500.f);
        CHECK(accelerations.brim == 400.f);
    }

    SECTION("the first layer acceleration caps support and brim") {
        const SupportAccelerations accelerations = support_accelerations({{"support_material_acceleration", "600"},
            {"support_material_interface_acceleration", "50%"}, {"brim_acceleration", "0"},
            {"first_layer_acceleration", "400"}});
        CHECK(accelerations.support == 400.f);
        CHECK(accelerations.interface == 300.f);
        CHECK(accelerations.brim == 400.f);
    }
}

// Slice and export a supported slab, then follow the Marlin print acceleration
// (M204 P) through the ;TYPE: sections: support and interface extrusions must
// run at their own accelerations.
TEST_CASE("A print with support exports support moves at the support accelerations",
          "[plugins][layer-extrusion-edit][acceleration][support][gcode]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = Test::SupportGrid::config("grid", false, 3, true);
    config.set_deserialize_strict({
        {"step_gcode_plugin", "gcode.printing_plan_file_writer"},
        {"gcode_firmware_plugin", "gcode.firmware.marlin2"},
        {"default_acceleration", "1000"}, {"support_material_acceleration", "600"},
        {"support_material_interface_acceleration", "50%"}, {"first_layer_acceleration", "0"},
        {"machine_limits_usage", "ignore"}, {"use_relative_e_distances", "1"}, {"wipe", "0,0"},
        {"filament_diameter", "1.75,1.75"}
    });
    Model model;
    Print print;
    Test::init_print({Test::SupportGrid::slab(true)}, print, model, config);
    const boost::filesystem::path path = Test::GCode::temporary_gcode_path();
    std::string code;
    try {
        print.process();
        Orchestrator::instance().export_gcode(print, path.string(), nullptr);
        code = Test::GCode::read_text_file(path);
    } catch (...) {
        Test::GCode::remove_output_pair(path);
        throw;
    }
    Test::GCode::remove_output_pair(path);

    // With relative E, a printing move is a G1 with X or Y and a positive E.
    std::set<int> base_accelerations;
    std::set<int> interface_accelerations;
    std::string type;
    int print_acceleration = -1;
    std::istringstream stream(code);
    std::string line;
    while (std::getline(stream, line)) {
        if (line.rfind(";TYPE:", 0) == 0) {
            type = line.substr(6);
            continue;
        }
        const std::string command = line.substr(0, line.find(';'));
        if (command.rfind("M204 ", 0) == 0) {
            const size_t pos = command.find(" P");
            if (pos != std::string::npos)
                print_acceleration = std::stoi(command.substr(pos + 2));
            continue;
        }
        if (command.rfind("G1 ", 0) != 0)
            continue;
        const size_t e_pos = command.find(" E");
        const bool moves_xy = command.find(" X") != std::string::npos || command.find(" Y") != std::string::npos;
        if (!moves_xy || e_pos == std::string::npos || std::stod(command.substr(e_pos + 2)) <= 0.)
            continue;
        if (type == "Support material")
            base_accelerations.insert(print_acceleration);
        else if (type == "Support material interface")
            interface_accelerations.insert(print_acceleration);
    }
    CHECK(base_accelerations == std::set<int>{600});
    CHECK(interface_accelerations == std::set<int>{300});
}
