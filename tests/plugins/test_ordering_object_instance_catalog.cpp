///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

#include <catch2/catch.hpp>
#include <set>
#include <string>
#include <vector>
#include <boost/filesystem.hpp>

#include "gcode_test_helpers.hpp"
#include "plugin_test_helpers.hpp"
#include "test_data.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"

/*
These production exports check the catalog names used as Klipper cancellation
identities. Equal object names and names equal after sanitization must identify
different physical copies; unrelated names must remain unchanged. A third object
uses a suffix-like name so generated identities cannot overwrite an existing one.
The Print owns source objects and the catalog for the complete export.

    TEST_CASE
        |-- Print::process() -> prepare small generated meshes
        |-- Orchestrator::export_gcode() -> build catalog and emit Klipper labels
        `-- inspect DEFINE/START/END names against the final owned catalog
*/

using namespace Slic3r;

// Export one copy of each object and verify distinct cancellation names, preserving
// all initially unique names even when they resemble generated collision suffixes.
TEST_CASE("Klipper catalog names distinguish homonymous physical copies",
          "[plugins][gcode][firmware][klipper][object-instance-catalog][catalog-name-collision]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const int scene = GENERATE(0, 1, 2);
    CAPTURE(scene);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"print_plan_builder", "ordering.plan_builder.by_object"},
        {"complete_objects_step", "0"}, {"complete_objects_sort", "object"}, {"complete_islands", "0"},
        {"complete_objects_one_first_layer", "0"}, {"gcode_label_objects", "firmware"}, {"gcode_flavor", "klipper"},
        {"layer_height", "0.2"}, {"first_layer_height", "0.2"}, {"skirts", "0"}, {"brim_width", "0"},
        {"perimeters", "1"}, {"fill_density", "0%"}, {"top_solid_layers", "0"}, {"bottom_solid_layers", "0"},
        {"start_gcode", ""}, {"end_gcode", ""}, {"binary_gcode", "0"}, {"wipe_tower", "0"}});
    Model model;
    Print print;
    Test::init_print({make_cube(2., 2., 0.4), make_cube(2., 2., 0.4), make_cube(2., 2., 0.4)}, print, model, config);
    const std::string base = scene == 1 ? "Part_A" : "Part";
    model.objects()[0].name = scene == 1 ? "Part A" : "Part";
    model.objects()[1].name = scene == 1 ? "Part-A" : scene == 2 ? "Other" : "Part";
    model.objects()[2].name = base + "_id_0";
    for (size_t i = 0; i < model.objects().size(); ++i) {
        REQUIRE(model.objects()[i].instances.size() == 1);
        model.objects()[i].instances.front()->set_offset(Vec3d(30. + 50. * i, 30., 0.));
    }
    print.apply(model, config);
    print.set_status_silent();
    print.process();
    Test::GCode::select_printing_plan_firmware(print, "gcode.firmware.klipper");
    const boost::filesystem::path output = Test::GCode::temporary_gcode_path();
    Orchestrator::instance().export_gcode(print, output.string(), nullptr);
    const std::string text = Test::GCode::read_text_file(output);
    boost::filesystem::remove(output);
    REQUIRE(print.printing_plan() != nullptr);
    const Printing::PrintingPlan &plan = *print.printing_plan();
    REQUIRE(plan.object_instances.size() == 3);
    std::set<std::string> names;
    for (size_t id = 0; id < plan.object_instances.size(); ++id) {
        const Printing::PrintingObjectInstance &copy = plan.object_instances[id];
        REQUIRE(copy.object != nullptr);
        CHECK(names.insert(copy.name).second);
        CHECK(Test::GCode::count_occurrences(text, "EXCLUDE_OBJECT_DEFINE NAME='" + copy.name + "'") == 1);
        CHECK(Test::GCode::count_occurrences(text, "EXCLUDE_OBJECT_START NAME='" + copy.name + "'\n") > 0);
        CHECK(Test::GCode::count_occurrences(text, "EXCLUDE_OBJECT_END NAME='" + copy.name + "'\n") > 0);
        if (copy.object == &print.objects()[2]) CHECK(copy.name == base + "_id_0");
        if (scene == 2 && copy.object == &print.objects()[0]) CHECK(copy.name == "Part");
        if (scene == 2 && copy.object == &print.objects()[1]) CHECK(copy.name == "Other");
    }
    CHECK(names.size() == 3);
}
