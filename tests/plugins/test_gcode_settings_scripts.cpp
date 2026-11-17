#include <catch2/catch.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

#include <boost/filesystem.hpp>
#include <tbb/global_control.h>

#include "plugin_test_helpers.hpp"
#include "gcode_test_helpers.hpp"
#include "layer_extrusion_edit_transition_test_helpers.hpp"

#include "libslic3r/Api/host/ApiHostUtils.hpp"
#include "libslic3r/Api/host/GCodeScriptProcessor.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_gcode.h"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/DefaultGCodeFirmwareSession.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeFirmwareViews.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeScriptProcessorViews.hpp"
#include "libslic3r/Api/plugin/cpp/PrintHelpers.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/MachineEnvelope.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Config/ConfigSnapshotSerialization.hpp"
#include "libslic3r/CustomGCode.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityVisitors.hpp"
#include "libslic3r/GCode.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/Geometry/ArcWelder.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Plugins/GCode/Firmware/BuiltinGCodeFirmwares.hpp"
#include "libslic3r/Plugins/GCode/Firmware/KlipperGCodeFirmware.hpp"
#include "libslic3r/Plugins/GCode/Firmware/Marlin1GCodeFirmware.hpp"
#include "libslic3r/Plugins/GCode/Firmware/Marlin2GCodeFirmware.hpp"
#include "libslic3r/Plugins/GCode/Firmware/PrusaGCodeFirmware.hpp"
#include "libslic3r/Plugins/GCode/Firmware/RepRapGCodeFirmware.hpp"
#include "libslic3r/Plugins/GCode/Firmware/SprinterGCodeFirmware.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Plugins/Ordering/ObjectInstanceCatalog.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"
#include "libslic3r/Steps/StepExtrusionEdition.hpp"
#include "libslic3r/Steps/StepLayerExtrusionEdition.hpp"
#include "libslic3r/Steps/StepGenerateGcode.hpp"
#include "libslic3r/Steps/StepExtrusionOrdering.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"
#include "test_data.hpp"

/*
Check where configured G-code scripts are inserted into a printing plan and
which values their placeholder context contains. Fixtures create the source
layers and event scopes, run the relevant production steps, then inspect the
published events or generated text. This separates script placement from the
firmware dialect used to encode ordinary extrusion moves.

Settings script tests verify how configured user scripts are inserted into the final PrintingPlan scopes.
Shared plan construction and output helpers live in gcode_test_helpers so this file keeps only component-specific behavior.
The source layer height belongs to an extrusion's linked regional island; path attributes remain separate printable-geometry values.

Representative direct calls (branches are not a timeline):

    TEST_CASE
        |-- Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized()
        |-- Test::init_print()
        |-- print.apply()
        |-- print.process()
        |-- orchestrator.export_gcode()
        `-- read_text_file()
*/

namespace {
using namespace Slic3r;
using namespace Slic3r::Printing;
using namespace Slic3r::Test::GCode;

// Apply the script-test settings to shared mixed-height cubes, configure firmware before processing, and propagate pipeline failures; Print and Model remain caller-owned.
void prepare_mixed_height_print(Print &print,
                                Model &model,
                                const int32_t first_tool = 1,
                                const int32_t second_tool = 1)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"first_layer_height", "0.2"},
        {"support_material", "0"},
        {"print_plan_builder", "ordering.plan_builder.by_layer"},
        {"complete_objects_step", "0"},
        {"seam_visibility", "0"}
    });
    initialize_mixed_height_cube_print(print, model, config, first_tool, second_tool);
    configure_standard_firmware(print, 2);
    print.process();
}

// Run STEP_ORDERING to build Print's source-linked layer, tool and extrusion
// groups; return a borrowed reference to the print-owned plan and propagate failures.
PrintingPlan &install_mixed_height_plan(Print &print)
{
    Steps::StepExtrusionOrdering::clear(print);
    Steps::StepExtrusionOrdering::prepare(print);
    Steps::StepExtrusionOrdering::run_step(Orchestrator::instance(), print);
    return print.mutable_printing_plan();
}

// Apply the selected layer and script editors to the Print-owned ordered plan;
// edits remain in that plan and plugin or cancellation failures propagate.
void prepare_mixed_height_gcode_plan(Print &print)
{
    Orchestrator &orchestrator = Orchestrator::instance();
    Steps::StepLayerExtrusionEdition::run_step(orchestrator, print);
    Steps::StepExtrusionEdition::clear(print);
    Steps::StepExtrusionEdition::run_step(orchestrator, print);
}

// Find the by-layer group at print_z with two distinct source-layer heights;
// return a borrowed pointer or null, without mutating the plan or taking ownership.
PrintingLayerGroup *find_mixed_height_layer(PrintingPlan &plan, const coord_t print_z)
{
    for (PrintingGroup &group : plan.groups)
        for (PrintingLayerGroup &layer : group.layers)
            if (layer.print_z == print_z && layer.layers.size() == 2 &&
                layer.layers[0] != nullptr && layer.layers[1] != nullptr &&
                layer.layers[0]->scaled_height() != layer.layers[1]->scaled_height())
                return &layer;
    return nullptr;
}

// Publish one color-change row with the supplied scaled Z and one-based tool
// number; Print owns the allocated options, and allocation or insertion failures
// propagate.
void publish_color_change_record(Print &print, const coord_t print_z, const int32_t extruder)
{
    DynamicConfig &record = print.records().get_or_add(CustomGCode::PrintRecordChannel);
    record.clear();
    record.set_key_value(CustomGCode::PrintRecordSizeKey, new ConfigOptionInt(1));
    record.set_key_value(CustomGCode::PrintRecordModeKey,
                         new ConfigOptionInt(int32_t(CustomGCode::MultiExtruder)));
    record.set_key_value(CustomGCode::PrintRecordPrintZKey,
                         new ConfigOptionFloats({unscaled(print_z)}));
    ConfigOptionInts *types = new ConfigOptionInts();
    types->set(std::vector<int32_t>{int32_t(CustomGCode::ColorChange)});
    record.set_key_value(CustomGCode::PrintRecordTypeKey, types);
    ConfigOptionInts *extruders = new ConfigOptionInts();
    extruders->set(std::vector<int32_t>{extruder});
    record.set_key_value(CustomGCode::PrintRecordExtruderKey, extruders);
    record.set_key_value(CustomGCode::PrintRecordColorKey,
                         new ConfigOptionStrings({"#123456"}));
    record.set_key_value(CustomGCode::PrintRecordExtraKey,
                         new ConfigOptionStrings({""}));
}

} // namespace

TEST_CASE("Configured G-code scripts become typed global plan events",
          "[plugins][gcode][script-type][extrusion-edit]") {
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.option<ConfigOptionString>("start_gcode")->value = "M117 configured start";
    config.option<ConfigOptionString>("end_gcode")->value = "M117 configured end";
    PrintingPlan &plan = print.mutable_printing_plan();

    Steps::StepExtrusionEdition::clear(print);
    Steps::StepExtrusionEdition::run_step(Orchestrator::instance(), print);

    REQUIRE(plan.events.before().child_count() == 1);
    REQUIRE(plan.events.after().child_count() == 1);
    const ExtrusionEntity &start = plan.events.before().child(0);
    const ExtrusionEntity &end = plan.events.after().child(0);
    const ExtrusionPropertyCustomGcode *start_property = start.get_property<ExtrusionPropertyCustomGcode>();
    const ExtrusionPropertyCustomGcode *end_property = end.get_property<ExtrusionPropertyCustomGcode>();
    REQUIRE(start_property != nullptr);
    REQUIRE(end_property != nullptr);
    CHECK(start_property->kind == C_EXTRUSION_CUSTOM_GCODE_SCRIPT);
    CHECK(start_property->script_type == GCODE_SCRIPT_TYPE_START_GCODE);
    CHECK(start.custom_gcode_string(*start_property) == "M117 configured start");
    CHECK(end_property->kind == C_EXTRUSION_CUSTOM_GCODE_SCRIPT);
    CHECK(end_property->script_type == GCODE_SCRIPT_TYPE_END_GCODE);
    CHECK(end.custom_gcode_string(*end_property) == "M117 configured end");

    config.option<ConfigOptionString>("start_gcode")->value.clear();
    config.option<ConfigOptionString>("end_gcode")->value.clear();
    Steps::StepExtrusionEdition::clear(print);
    Steps::StepExtrusionEdition::run_step(Orchestrator::instance(), print);
    CHECK_FALSE(plan.events.has_before());
    CHECK_FALSE(plan.events.has_after());
}
// Build a local PrintingPlan, place configured scripts in their ordered scopes, and verify stored values and cleanup; plugin failures surface through Catch assertions and the plan remains print-owned.
TEST_CASE("Configured layer and filament scripts become scoped plan events",
          "[plugins][gcode][script-type][extrusion-edit]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    configure_standard_firmware(print, 2);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.before_layer_gcode.value = "; before layer";
    config.layer_gcode.value = "; after layer move";
    config.option<ConfigOptionString>("toolchange_gcode")->value = "; custom toolchange";
    config.option<ConfigOptionStrings>("start_filament_gcode")->set(std::vector<std::string>{"; start filament 0", "; start filament 1"});
    config.option<ConfigOptionStrings>("end_filament_gcode")->set(std::vector<std::string>{"; end filament 0", "; end filament 1"});
    config.option<ConfigOptionString>("end_gcode")->value = "; end print";

    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    PrintingGroup &group = plan.groups.front();
    group.layers.emplace_back();
    group.layers.back().print_z = scale_i(0.2);
    group.layers.back().tool_groups.emplace_back();
    group.layers.back().tool_groups.back().extruder_id = 1;
    group.layers.emplace_back();
    group.layers.back().print_z = scale_i(0.4);
    group.layers.back().tool_groups.emplace_back();
    group.layers.back().tool_groups.back().extruder_id = 1;
    group.layers.back().tool_groups.emplace_back();
    group.layers.back().tool_groups.back().extruder_id = 0;

    Steps::StepExtrusionEdition::clear(print);
    Steps::StepExtrusionEdition::run_step(Orchestrator::instance(), print);

    REQUIRE(group.events.before().child_count() == 1);
    const ExtrusionPropertyCustomGcode *first_before_layer =
        group.events.before().child(0).get_property<ExtrusionPropertyCustomGcode>();
    REQUIRE(first_before_layer != nullptr);
    CHECK(first_before_layer->script_type == GCODE_SCRIPT_TYPE_BEFORE_LAYER_GCODE);
    CHECK_FALSE(group.layers[0].events.has_before());

    REQUIRE(group.layers[0].events.after().child_count() == 1);
    const ExtrusionPropertyCustomGcode *second_before_layer =
        group.layers[0].events.after().child(0).get_property<ExtrusionPropertyCustomGcode>();
    REQUIRE(second_before_layer != nullptr);
    CHECK(second_before_layer->script_type == GCODE_SCRIPT_TYPE_BEFORE_LAYER_GCODE);
    REQUIRE(group.layers[1].events.before().child_count() == 1);
    const ExtrusionPropertyCustomGcode *second_layer_script =
        group.layers[1].events.before().child(0).get_property<ExtrusionPropertyCustomGcode>();
    REQUIRE(second_layer_script != nullptr);
    CHECK(second_layer_script->script_type == GCODE_SCRIPT_TYPE_LAYER_GCODE);
    const ExtrusionEntity &second_layer_entity = group.layers[1].events.before().child(0);
    const slic3r_api::ExtrusionEntity second_layer_event(
        reinterpret_cast<const extrusion_entity_handle *>(&second_layer_entity));
    const slic3r_api::EPropertyCustomGcode *second_layer_payload =
        second_layer_event.get(slic3r_api::EPropertyCustomGcode::key);
    REQUIRE(second_layer_payload != nullptr);
    CHECK(second_layer_payload->processing_extruder_id == EXTRUDER_ID_UNKNOWN);
    CHECK(stored_script_int(second_layer_entity, *second_layer_script, "layer_num") == 1);
    CHECK(stored_script_float(second_layer_entity, *second_layer_script, "layer_z") == Approx(0.4));
    CHECK(stored_script_float(second_layer_entity, *second_layer_script, "previous_layer_z") == Approx(0.2));
    CHECK(stored_script_float(second_layer_entity, *second_layer_script, "max_layer_z") == Approx(0.4));

    PrintingToolGroup &initial_tool = group.layers[0].tool_groups[0];
    REQUIRE(initial_tool.events.before().child_count() == 1);
    const ExtrusionPropertyCustomGcode *initial_start =
        initial_tool.events.before().child(0).get_property<ExtrusionPropertyCustomGcode>();
    REQUIRE(initial_start != nullptr);
    CHECK(initial_start->script_type == GCODE_SCRIPT_TYPE_START_FILAMENT_GCODE);
    CHECK(initial_start->processing_extruder_id == 1);

    PrintingToolGroup &unchanged_tool = group.layers[1].tool_groups[0];
    CHECK_FALSE(unchanged_tool.events.has_before());
    REQUIRE(unchanged_tool.events.after().child_count() == 1);

    PrintingToolGroup &changed_tool = group.layers[1].tool_groups[1];
    REQUIRE(changed_tool.events.before().child_count() == 1);
    const ExtrusionPropertyCustomGcode *end_filament =
        unchanged_tool.events.after().child(0).get_property<ExtrusionPropertyCustomGcode>();
    const ExtrusionPropertyCustomGcode *start_filament =
        changed_tool.events.before().child(0).get_property<ExtrusionPropertyCustomGcode>();
    REQUIRE(end_filament != nullptr);
    REQUIRE(start_filament != nullptr);
    CHECK(end_filament->script_type == GCODE_SCRIPT_TYPE_END_FILAMENT_GCODE);
    CHECK(end_filament->processing_extruder_id == 1);
    CHECK(start_filament->script_type == GCODE_SCRIPT_TYPE_START_FILAMENT_GCODE);
    CHECK(start_filament->processing_extruder_id == 0);

    REQUIRE(plan.events.after().child_count() == 3);
    const ExtrusionPropertyCustomGcode *final_tool_zero =
        plan.events.after().child(0).get_property<ExtrusionPropertyCustomGcode>();
    const ExtrusionPropertyCustomGcode *final_tool_one =
        plan.events.after().child(1).get_property<ExtrusionPropertyCustomGcode>();
    const ExtrusionPropertyCustomGcode *end_print =
        plan.events.after().child(2).get_property<ExtrusionPropertyCustomGcode>();
    REQUIRE(final_tool_zero != nullptr);
    REQUIRE(final_tool_one != nullptr);
    REQUIRE(end_print != nullptr);
    CHECK(final_tool_zero->script_type == GCODE_SCRIPT_TYPE_END_FILAMENT_GCODE);
    CHECK(final_tool_zero->processing_extruder_id == 0);
    CHECK(final_tool_one->script_type == GCODE_SCRIPT_TYPE_END_FILAMENT_GCODE);
    CHECK(final_tool_one->processing_extruder_id == 1);
    CHECK(end_print->script_type == GCODE_SCRIPT_TYPE_END_GCODE);
    CHECK(end_print->processing_extruder_id == 0);
    const DynamicConfig end_arguments = stored_script_config(
        plan.events.after().child(2), *end_print);
    CHECK(end_arguments.option<ConfigOptionInt>("layer_num") == nullptr);
    CHECK(stored_script_float(
              plan.events.after().child(2), *end_print, "previous_layer_z") == Approx(0.2));

    config.before_layer_gcode.value.clear();
    config.layer_gcode.value.clear();
    config.option<ConfigOptionString>("toolchange_gcode")->value.clear();
    config.option<ConfigOptionStrings>("start_filament_gcode")->set(std::vector<std::string>{"", ""});
    config.option<ConfigOptionStrings>("end_filament_gcode")->set(std::vector<std::string>{"", ""});
    config.option<ConfigOptionString>("end_gcode")->value.clear();
    Steps::StepExtrusionEdition::clear(print);
    Steps::StepExtrusionEdition::run_step(Orchestrator::instance(), print);
    CHECK_FALSE(group.events.has_before());
    CHECK_FALSE(group.layers[0].events.has_after());
    CHECK_FALSE(group.layers[1].events.has_before());
    CHECK_FALSE(unchanged_tool.events.has_after());
    CHECK_FALSE(changed_tool.events.has_before());
    CHECK_FALSE(plan.events.has_after());
}
// Initialize the complete plan, then verify the finished-copy count on the producer-selected side of the move.
TEST_CASE("Between-object scripts select the requested side of the group move",
          "[plugins][gcode][script-type][extrusion-edit][sequential]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    DynamicPrintConfig source_config = DynamicPrintConfig::full_print_config();
    Print print;
    Model model;
    Test::init_print({make_cube(5., 5., 0.2), make_cube(4., 4., 0.2)}, print, model, source_config);
    REQUIRE(print.objects().size() == 2);
    print.objects()[0].model_object()->name = "First object";
    print.objects()[1].model_object()->name = "Second/object";

    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.option<ConfigOptionString>("start_gcode")->value.clear();
    config.option<ConfigOptionString>("end_gcode")->value.clear();
    config.before_layer_gcode.value.clear();
    config.layer_gcode.value.clear();
    config.option<ConfigOptionString>("toolchange_gcode")->value.clear();
    config.option<ConfigOptionStrings>("start_filament_gcode")->set(std::vector<std::string>{""});
    config.option<ConfigOptionStrings>("end_filament_gcode")->set(std::vector<std::string>{""});
    config.option<ConfigOptionString>("between_objects_gcode")->value =
        "; between [previous_object_id]:[previous_object_name] [next_object_id]:[next_object_name] I[current_object_idx]";

    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.back().printing_catalog_ids.push_back(
        plan.find_or_add_object_instance(print.objects()[0], 0));
    // Auxiliary groups do not participate in object-to-object script
    // selection, even when they sit between two real object groups.
    plan.groups.emplace_back();
    plan.groups.emplace_back();
    plan.groups.back().printing_catalog_ids.push_back(
        plan.find_or_add_object_instance(print.objects()[1], 0));
    PluginStorage catalog_storage;
    slic3r_api::Ordering::ObjectInstanceCatalogPlugin::populate_object_instance_catalog(
        reinterpret_cast<const print_handle *>(&print), reinterpret_cast<printing_plan_handle *>(&plan),
        reinterpret_cast<storage_handle *>(&catalog_storage));

    config.option<ConfigOptionBool>("between_objects_gcode_before_move")->value = false;
    Steps::StepExtrusionEdition::clear(print);
    Steps::StepExtrusionEdition::run_step(Orchestrator::instance(), print);
    CHECK_FALSE(plan.groups[0].events.has_after());
    CHECK_FALSE(plan.groups[1].events.has_before());
    CHECK_FALSE(plan.groups[1].events.has_after());
    REQUIRE(plan.groups[2].events.before().child_count() == 1);

    const ExtrusionEntity &event = plan.groups[2].events.before().child(0);
    const slic3r_api::ExtrusionEntity event_view(
        reinterpret_cast<const extrusion_entity_handle *>(&event));
    const slic3r_api::EPropertyCustomGcode *property =
        event_view.get(slic3r_api::EPropertyCustomGcode::key);
    REQUIRE(property != nullptr);
    REQUIRE(property->script_type == GCODE_SCRIPT_TYPE_BETWEEN_OBJECTS_GCODE);
    GCodeScriptProcessor host_processor(print, Orchestrator::instance());
    const slic3r_api::GCodeGeneration::GCodeScriptProcessorView scripts(host_processor.c_processor());
    PluginStorage script_storage;
    storage_handle *script_storage_handle =
        reinterpret_cast<storage_handle *>(&script_storage);
    ScriptStateFirmwareProbe firmware(scripts, script_storage_handle);
    const slic3r_api::Print print_view(reinterpret_cast<const print_handle *>(&print));
    const slic3r_api::PrintingPlan plan_view = printing_plan_view(print);
    firmware.begin_print(print_view, plan_view);
    CHECK(firmware.begin_group(plan_view.group(0)).empty());
    CHECK(firmware.begin_group(plan_view.group(1)).empty());
    CHECK(firmware.begin_group(plan_view.group(2)).empty());
    CHECK(firmware.write_event(plan_view.group(2).events().before()) ==
          "; between 0:First_object 1:Second_object I1\n");

    config.option<ConfigOptionBool>("between_objects_gcode_before_move")->value = true;
    Steps::StepExtrusionEdition::clear(print);
    Steps::StepExtrusionEdition::run_step(Orchestrator::instance(), print);
    REQUIRE(plan.groups[0].events.after().child_count() == 1);
    CHECK_FALSE(plan.groups[1].events.has_before());
    CHECK_FALSE(plan.groups[1].events.has_after());
    CHECK_FALSE(plan.groups[2].events.has_before());

    ScriptStateFirmwareProbe before_move_firmware(scripts, script_storage_handle);
    const slic3r_api::PrintingPlan refreshed_plan_view = printing_plan_view(print);
    before_move_firmware.begin_print(print_view, refreshed_plan_view);
    CHECK(before_move_firmware.begin_group(refreshed_plan_view.group(0)).empty());
    CHECK(before_move_firmware.write_event(refreshed_plan_view.group(0).events().after()) ==
          "; between 0:First_object 1:Second_object I0\n");
}
TEST_CASE("Single-extruder multimaterial finalizes only the active logical tool",
          "[plugins][gcode][script-type][extrusion-edit]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    configure_standard_firmware(print, 2);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.option<ConfigOptionString>("start_gcode")->value.clear();
    config.option<ConfigOptionString>("end_gcode")->value.clear();
    config.option<ConfigOptionString>("toolchange_gcode")->value.clear();
    config.option<ConfigOptionStrings>("start_filament_gcode")->set(std::vector<std::string>{"", ""});
    config.option<ConfigOptionStrings>("end_filament_gcode")->set(std::vector<std::string>{"; finalize 0", "; finalize 1"});
    config.single_extruder_multi_material.value = true;

    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    plan.groups.front().layers.front().print_z = scale_i(0.2);
    plan.groups.front().layers.front().tool_groups.emplace_back();
    plan.groups.front().layers.front().tool_groups.back().extruder_id = 0;
    plan.groups.front().layers.front().tool_groups.emplace_back();
    plan.groups.front().layers.front().tool_groups.back().extruder_id = 1;

    Steps::StepExtrusionEdition::clear(print);
    Steps::StepExtrusionEdition::run_step(Orchestrator::instance(), print);

    REQUIRE(plan.events.after().child_count() == 1);
    const ExtrusionEntity &event = plan.events.after().child(0);
    const ExtrusionPropertyCustomGcode *host_property =
        event.get_property<ExtrusionPropertyCustomGcode>();
    REQUIRE(host_property != nullptr);
    CHECK(event.custom_gcode_string(*host_property) == "; finalize 1");
    const slic3r_api::ExtrusionEntity event_view(
        reinterpret_cast<const extrusion_entity_handle *>(&event));
    const slic3r_api::EPropertyCustomGcode *property =
        event_view.get(slic3r_api::EPropertyCustomGcode::key);
    REQUIRE(property != nullptr);
    CHECK(property->processing_extruder_id == 1);
}
// Export a local plan after inserting configured scripts, then check script values and event order; export exceptions fail the case and all temporary output files are removed.
TEST_CASE("Configured scripts use ordered plan tools and final layer state",
          "[plugins][gcode][script-type][firmware][integration]") {
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    select_printing_plan_firmware(print);
    configure_standard_firmware(print, 2);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.filament_diameter.set(std::vector<double>{1.75, 2.85});
    DynamicPrintConfig &full_config = const_cast<DynamicPrintConfig &>(print.full_print_config());
    full_config.set_key_value("filament_diameter", new ConfigOptionFloats(std::vector<double>{1.75, 2.85}));
    config.print_first_layer_bed_temperature.value = 71;
    config.print_first_layer_bed_temperature.set_enabled(true);
    config.option<ConfigOptionString>("start_gcode")->value = "; SCRIPT_START T[current_extruder] D{filament_diameter[current_extruder]} "
                               "B[start_gcode_bed_temperature]";
    config.before_layer_gcode.value = "; BEFORE_LAYER L[layer_num] Z[layer_z] P[previous_layer_z]";
    config.layer_gcode.value = "; LAYER L[layer_num] Z[layer_z]";
    config.option<ConfigOptionString>("toolchange_gcode")->value = "; CUSTOM_TOOL P[previous_extruder] N[next_extruder]";
    config.option<ConfigOptionStrings>("start_filament_gcode")->set(std::vector<std::string>{
        "; START_FILAMENT F[filament_extruder_id] P[previous_extruder] N[next_extruder]",
        "; START_FILAMENT F[filament_extruder_id] P[previous_extruder] N[next_extruder]"
    });
    config.option<ConfigOptionStrings>("end_filament_gcode")->set(std::vector<std::string>{
        "; END_FILAMENT F[filament_extruder_id] P[previous_extruder] N[next_extruder]",
        "; END_FILAMENT F[filament_extruder_id] P[previous_extruder] N[next_extruder]"
    });
    config.option<ConfigOptionString>("end_gcode")->value = "; SCRIPT_END L[layer_num] Z[layer_z] M[max_layer_z] F[filament_extruder_id] "
                             "P[previous_extruder] N[next_extruder]";

    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    PrintingGroup &group = plan.groups.front();
    group.layers.emplace_back();
    group.layers.back().print_z = scale_i(0.2);
    group.layers.back().tool_groups.emplace_back();
    group.layers.back().tool_groups.back().extruder_id = 1;
    group.layers.emplace_back();
    group.layers.back().print_z = scale_i(0.6);
    group.layers.back().tool_groups.emplace_back();
    group.layers.back().tool_groups.back().extruder_id = 0;

    Steps::StepExtrusionEdition::clear(print);
    Slic3r::Test::TransitionPipeline::run_layer_plugins(
        print, {Slic3r::Test::TransitionPipeline::TOOLCHANGE_PLUGIN});
    Steps::StepExtrusionEdition::run_step(Orchestrator::instance(), print);

    // Only producer-frozen values stay attached to their events; end_gcode
    // reads live layer state after these Z edits.
    group.layers[0].print_z = scale_i(0.25);
    group.layers[1].print_z = scale_i(0.75);

    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    Orchestrator &orchestrator = Orchestrator::instance();
    orchestrator.reset_plugin_cancel();
    try {
        Steps::StepGenerateGcode::run_step(orchestrator, print, output_path.string());
        const std::string output = read_text_file(output_path);
        INFO(output);

        const std::string start = "; SCRIPT_START T1 D2.85 B71\n";
        const std::string first_before_layer = "; BEFORE_LAYER L0 Z0.2 P0\n";
        const std::string first_start_filament = "; START_FILAMENT F1 P-1 N1\n";
        const std::string second_before_layer = "; BEFORE_LAYER L1 Z0.6 P0.2\n";
        const std::string second_layer = "; LAYER L1 Z0.6\n";
        const std::string transition_end_filament = "; END_FILAMENT F1 P1 N0\n";
        const std::string custom_toolchange = "; CUSTOM_TOOL P1 N0\n";
        const std::string second_start_filament = "; START_FILAMENT F0 P1 N0\n";
        const std::string final_end_filament_zero = "; END_FILAMENT F0 P0 N-1\n";
        const std::string final_end_filament_one = "; END_FILAMENT F1 P0 N-1\n";
        const std::string end = "; SCRIPT_END L1 Z0.75 M0.6 F0 P0 N-1\n";
        const size_t start_position = output.find(start);
        const size_t first_before_layer_position = output.find(first_before_layer);
        const size_t first_tool_position = output.find("T1\n");
        const size_t first_start_filament_position = output.find(first_start_filament);
        const size_t second_before_layer_position = output.find(second_before_layer);
        const size_t second_layer_position = output.find(second_layer);
        const size_t transition_end_position = output.find(transition_end_filament);
        const size_t custom_toolchange_position = output.find(custom_toolchange);
        const size_t second_start_filament_position = output.find(second_start_filament);
        const size_t final_zero_position = output.find(final_end_filament_zero);
        const size_t final_one_position = output.find(final_end_filament_one);
        const size_t end_position = output.find(end);
        REQUIRE(start_position != std::string::npos);
        REQUIRE(first_before_layer_position != std::string::npos);
        REQUIRE(first_tool_position != std::string::npos);
        REQUIRE(first_start_filament_position != std::string::npos);
        REQUIRE(second_before_layer_position != std::string::npos);
        REQUIRE(second_layer_position != std::string::npos);
        REQUIRE(transition_end_position != std::string::npos);
        REQUIRE(custom_toolchange_position != std::string::npos);
        REQUIRE(second_start_filament_position != std::string::npos);
        REQUIRE(final_zero_position != std::string::npos);
        REQUIRE(final_one_position != std::string::npos);
        REQUIRE(end_position != std::string::npos);
        CHECK(start_position < first_before_layer_position);
        // The preamble selects the initial tool before layer events.
        CHECK(first_tool_position < first_before_layer_position);
        CHECK(first_tool_position < first_start_filament_position);
        CHECK(first_start_filament_position < transition_end_position);
        CHECK(transition_end_position < second_before_layer_position);
        CHECK(second_before_layer_position < second_layer_position);
        CHECK(second_layer_position < custom_toolchange_position);
        CHECK(custom_toolchange_position < second_start_filament_position);
        CHECK(second_start_filament_position < final_zero_position);
        CHECK(final_zero_position < final_one_position);
        CHECK(final_one_position < end_position);
        CHECK(count_command_lines(output, "T1") == 1);
        CHECK(count_command_lines(output, "T0") == 0);
        CHECK(output.back() == '\n');
    } catch (...) {
        orchestrator.reset_plugin_cancel();
        remove_output_pair(output_path);
        throw;
    }
    orchestrator.reset_plugin_cancel();
    remove_output_pair(output_path);
}
TEST_CASE("Layer scripts report consumed filament",
          "[plugins][gcode][script-type][firmware][integration]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    select_printing_plan_firmware(print);
    configure_standard_firmware(print);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.option<ConfigOptionString>("start_gcode")->value.clear();
    config.option<ConfigOptionString>("end_gcode")->value.clear();
    config.option<ConfigOptionStrings>("start_filament_gcode")->set(std::vector<std::string>{""});
    config.option<ConfigOptionStrings>("end_filament_gcode")->set(std::vector<std::string>{""});
    config.before_layer_gcode.value = "; USED L[layer_num] F{layer_used_filament[0]}";

    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    PrintingGroup &group = plan.groups.front();
    group.layers.emplace_back();
    group.layers.back().print_z = scale_i(0.2);
    group.layers.back().tool_groups.emplace_back();
    PrintingToolGroup &first_tool = group.layers.back().tool_groups.back();
    first_tool.extruder_id = 0;
    append_path_extrusion(
        first_tool,
        make_firmware_path(
            ArcPolyline(Points{Point(scale_i(0.0), scale_i(0.0)), Point(scale_i(1.0), scale_i(0.0))}),
            15.f, 400.f));
    append_path_extrusion(
        first_tool,
        make_firmware_travel(
            ArcPolyline(Points{Point(scale_i(1.0), scale_i(0.0)), Point(scale_i(5.0), scale_i(0.0))}),
            80.f, 800.f));
    append_path_extrusion(
        first_tool,
        make_firmware_path(
            ArcPolyline(Points{Point(scale_i(5.0), scale_i(0.0)), Point(scale_i(6.0), scale_i(0.0))}),
            15.f, 400.f));
    append_path_extrusion(
        first_tool,
        make_firmware_path(
            ArcPolyline(Points{Point(scale_i(10.0), scale_i(0.0)), Point(scale_i(11.0), scale_i(0.0))}),
            15.f, 400.f, -1.f, -1.f, -1.f, ExtrusionRole::InternalInfill));
    group.layers.emplace_back();
    group.layers.back().print_z = scale_i(0.4);
    group.layers.back().tool_groups.emplace_back();
    group.layers.back().tool_groups.back().extruder_id = 0;

    Steps::StepExtrusionEdition::clear(print);
    Steps::StepExtrusionEdition::run_step(Orchestrator::instance(), print);

    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    Orchestrator &orchestrator = Orchestrator::instance();
    orchestrator.reset_plugin_cancel();
    try {
        Steps::StepGenerateGcode::run_step(orchestrator, print, output_path.string());
        const std::string output = read_text_file(output_path);
        INFO(output);

        CHECK(output.find("; USED L0 F0\n") != std::string::npos);
        const std::string second_layer_prefix = "; USED L1 F";
        const size_t second_layer_position = output.find(second_layer_prefix);
        REQUIRE(second_layer_position != std::string::npos);
        const size_t second_layer_value_begin = second_layer_position + second_layer_prefix.size();
        const size_t second_layer_value_end = output.find('\n', second_layer_value_begin);
        REQUIRE(second_layer_value_end != std::string::npos);
        CHECK(std::stod(output.substr(
                  second_layer_value_begin,
                  second_layer_value_end - second_layer_value_begin)) > 0.0);
    } catch (...) {
        orchestrator.reset_plugin_cancel();
        remove_output_pair(output_path);
        throw;
    }
    orchestrator.reset_plugin_cancel();
    remove_output_pair(output_path);
}

// Verify accumulated filament volume and weight remain separated by logical tool in and out of SEMM.
TEST_CASE("Scripts read per-filament extruded volume and weight",
          "[plugins][gcode][script-type][firmware][integration][script-globals]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    const double filament_area = 1.75 * 1.75 * PI / 4.0;
    for (const bool semm : {false, true}) {
        Print print;
        select_printing_plan_firmware(print);
        configure_standard_firmware(print, 2);
        PrintConfig &config = const_cast<PrintConfig &>(print.config());
        config.single_extruder_multi_material.value = semm;
        config.filament_density.set({1.25, 2.0});
        config.option<ConfigOptionString>("start_gcode")->value.clear();
        config.option<ConfigOptionStrings>("start_filament_gcode")->set(
            std::vector<std::string>{"", ""});
        config.option<ConfigOptionStrings>("end_filament_gcode")->set(
            std::vector<std::string>{"", ""});
        config.before_layer_gcode.value =
            "; USED L[layer_num] {layer_used_filament[0]} {layer_used_filament[1]}";
        config.option<ConfigOptionString>("end_gcode")->value =
            "; EV {extruded_volume[0]} {extruded_volume[1]} {extruded_volume_total} "
            "W {extruded_weight[0]} {extruded_weight[1]} {extruded_weight_total} O[current_object_idx]";

        PrintingPlan &plan = print.mutable_printing_plan();
        plan.groups.emplace_back();
        PrintingGroup &group = plan.groups.front();
        group.layers.emplace_back();
        group.layers.back().print_z = scale_i(0.2);
        group.layers.back().tool_groups.emplace_back();
        group.layers.back().tool_groups.back().extruder_id = 0;
        append_path_extrusion(
            group.layers.back().tool_groups.back(),
            make_firmware_path(
                ArcPolyline(Points{Point(scale_i(0.0), scale_i(0.0)),
                                   Point(scale_i(1.0), scale_i(0.0))}),
                15.f, 400.f));
        group.layers.back().tool_groups.emplace_back();
        group.layers.back().tool_groups.back().extruder_id = 1;
        append_path_extrusion(
            group.layers.back().tool_groups.back(),
            make_firmware_path(
                ArcPolyline(Points{Point(scale_i(2.0), scale_i(0.0)),
                                   Point(scale_i(4.0), scale_i(0.0))}),
                15.f, 400.f));
        group.layers.emplace_back();
        group.layers.back().print_z = scale_i(0.4);
        group.layers.back().tool_groups.emplace_back();
        group.layers.back().tool_groups.back().extruder_id = 1;
        append_path_extrusion(
            group.layers.back().tool_groups.back(),
            make_firmware_path(
                ArcPolyline(Points{Point(scale_i(4.0), scale_i(0.0)),
                                   Point(scale_i(5.0), scale_i(0.0))}),
                15.f, 400.f));

        Steps::StepExtrusionEdition::clear(print);
        Steps::StepExtrusionEdition::run_step(Orchestrator::instance(), print);

        const boost::filesystem::path output_path = temporary_gcode_path();
        remove_output_pair(output_path);
        Orchestrator &orchestrator = Orchestrator::instance();
        orchestrator.reset_plugin_cancel();
        try {
            Steps::StepGenerateGcode::run_step(orchestrator, print, output_path.string());
            const std::string output = read_text_file(output_path);
            INFO(output);

            const size_t used_position = output.find("; USED L1 ");
            REQUIRE(used_position != std::string::npos);
            const size_t used_begin = used_position + std::string("; USED L1 ").size();
            const size_t used_separator = output.find(' ', used_begin);
            REQUIRE(used_separator != std::string::npos);
            const double used_zero = std::stod(output.substr(used_begin, used_separator - used_begin));
            const size_t used_end = output.find('\n', used_separator + 1);
            REQUIRE(used_end != std::string::npos);
            const double used_one = std::stod(output.substr(used_separator + 1,
                                                               used_end - used_separator - 1));
            CHECK(used_zero == Approx(0.08 / filament_area).margin(1e-6));
            CHECK(used_one == Approx(0.16 / filament_area).margin(1e-6));

            const size_t ev_position = output.find("; EV ");
            REQUIRE(ev_position != std::string::npos);
            const size_t ev_begin = ev_position + std::string("; EV ").size();
            const size_t ev_end = output.find('\n', ev_begin);
            const std::string ev_line = output.substr(
                ev_begin, ev_end == std::string::npos ? std::string::npos : ev_end - ev_begin);
            std::istringstream ev_stream(ev_line);
            std::vector<double> values(6);
            std::string weight_label;
            std::string object_label;
            ev_stream >> values[0] >> values[1] >> values[2] >> weight_label
                      >> values[3] >> values[4] >> values[5] >> object_label;
            REQUIRE_FALSE(ev_stream.fail());
            REQUIRE(weight_label == "W");
            CHECK(values[0] == Approx(0.08).margin(1e-6));
            CHECK(values[1] == Approx(0.24).margin(1e-6));
            CHECK(values[2] == Approx(0.32).margin(1e-6));
            CHECK(values[3] == Approx(0.0001).margin(1e-6));
            CHECK(values[4] == Approx(0.00048).margin(1e-6));
            CHECK(values[5] == Approx(0.00058).margin(1e-6));
            REQUIRE(object_label == "O0");
        } catch (...) {
            orchestrator.reset_plugin_cancel();
            remove_output_pair(output_path);
            throw;
        }
        orchestrator.reset_plugin_cancel();
        remove_output_pair(output_path);
    }
}

// Verify firmware layer state and z-hop are live values unless a producer Config freezes a layer field.
TEST_CASE("Scripts read the firmware layer and z-hop unless the producer froze them",
          "[plugins][gcode][firmware][custom-gcode][script-globals]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    configure_standard_firmware(print, 1);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.z_offset.value = 0.1;
    DynamicPrintConfig &full_config = const_cast<DynamicPrintConfig &>(print.full_print_config());
    full_config.set_key_value("layer_height", new ConfigOptionFloat(0.25));

    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    PrintingGroup &group = plan.groups.front();
    group.layers.emplace_back();
    group.layers.back().print_z = scale_i(0.2);
    group.layers.back().tool_groups.emplace_back();
    group.layers.back().tool_groups.back().extruder_id = 0;
    group.layers.emplace_back();
    group.layers.back().print_z = scale_i(0.5);
    group.layers.back().tool_groups.emplace_back();
    group.layers.back().tool_groups.back().extruder_id = 0;

    GCodeScriptProcessor host_processor(print, Orchestrator::instance());
    const slic3r_api::GCodeGeneration::GCodeScriptProcessorView scripts(
        host_processor.c_processor());
    PluginStorage storage;
    storage_handle *storage_handle_value = reinterpret_cast<storage_handle *>(&storage);
    ScriptStateFirmwareProbe firmware(scripts, storage_handle_value);
    const slic3r_api::Print print_view(reinterpret_cast<const print_handle *>(&print));
    const slic3r_api::PrintingPlan plan_view = printing_plan_view(print);
    firmware.setup(print_view.config());
    const std::string script = "L[layer_num] Z{layer_z} H{layer_current_height} J{zhop}";

    CHECK(firmware.process_script(GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM, script) ==
          "L-1 Z0 H0 J0\n");
    CHECK(firmware.begin_layer(plan_view.group(0).layer_group(0)).find(";LAYER_CHANGE\n") == 0);
    CHECK(firmware.process_script(GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM, "G1 X1 Y1 Z0.9") ==
          "G1 X1 Y1 Z0.9\n");
    CHECK(firmware.process_script(GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM, script) ==
          "L0 Z0.2 H0.2 J0.6\n");
    CHECK(firmware.process_script(GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM, "G1 Z0.3") ==
          "G1 Z0.3\n");
    CHECK(firmware.process_script(GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM, script) ==
          "L0 Z0.2 H0.2 J0\n");
    CHECK(firmware.process_script(GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM, "G1 Z0.25") ==
          "G1 Z0.25\n");
    CHECK(firmware.process_script(GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM, script) ==
          "L0 Z0.2 H0.2 J0\n");
    CHECK(firmware.begin_layer(plan_view.group(0).layer_group(1)).find(";LAYER_CHANGE\n") == 0);
    CHECK(firmware.process_script(GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM, script) ==
          "L1 Z0.5 H0.3 J0\n");

    slic3r_api::StoredConfig frozen_config(storage_handle_value);
    frozen_config.get_or_add("layer_num", SLIC3R_CONFIG_OPTION_INT).set_int(7);
    frozen_config.get_or_add("layer_current_height", SLIC3R_CONFIG_OPTION_FLOAT).set_float(0.15);
    CHECK(firmware.process_script(
              GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM, script, &frozen_config) ==
          "L7 Z0.5 H0.15 J0\n");

    CHECK(firmware.process_script(
              GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM, "{layer_height}") == "0.25\n");
}
// Put the 0.3 mm source first in the mixed layer list while its first ordered
// source extrusion remains 0.2 mm; check the pre-layer script before its marker.
// Print owns the plan through export, and slicing or export failures fail the case.
TEST_CASE("Before-layer scripts use the first source extrusion of the upcoming mixed group",
          "[plugins][gcode][settings-scripts][firmware]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_mixed_height_print(print, model, 2, 1);
    PrintingPlan &plan = install_mixed_height_plan(print);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.option<ConfigOptionString>("start_gcode")->value.clear();
    config.option<ConfigOptionString>("end_gcode")->value.clear();
    config.before_layer_gcode.value = "; UPCOMING H{layer_current_height}";
    config.layer_gcode.value.clear();
    config.option<ConfigOptionString>("toolchange_gcode")->value.clear();
    config.option<ConfigOptionStrings>("start_filament_gcode")->set(
        std::vector<std::string>{"", ""});
    config.option<ConfigOptionStrings>("end_filament_gcode")->set(
        std::vector<std::string>{"", ""});
    PrintingLayerGroup *mixed_layer = find_mixed_height_layer(plan, scale_i(0.8));
    REQUIRE(mixed_layer != nullptr);
    REQUIRE(mixed_layer->layers[0]->scaled_height() == scale_i(0.2));
    std::swap(mixed_layer->layers[0], mixed_layer->layers[1]);
    REQUIRE(mixed_layer->layers[0]->scaled_height() == scale_i(0.3));
    REQUIRE_FALSE(mixed_layer->tool_groups.empty());
    REQUIRE_FALSE(mixed_layer->tool_groups.front().extrusions.empty());
    REQUIRE(slic3r_api::first_source_layer_height(
                slic3r_api::ConstPrintingToolGroup(reinterpret_cast<const printing_tool_group_handle *>(
                    &mixed_layer->tool_groups.front()))) == scale_i(0.2));
    prepare_mixed_height_gcode_plan(print);

    const std::string output = export_with_firmware(print, "marlin2");
    const size_t layer_position = output.find(";LAYER_CHANGE\n;Z:0.8\n");
    REQUIRE(layer_position != std::string::npos);
    const std::string expected_script = "; UPCOMING H0.2\n";
    const size_t script_position = output.rfind("; UPCOMING H", layer_position);
    REQUIRE(script_position != std::string::npos);
    CHECK(output.compare(script_position, expected_script.size(), expected_script) == 0);
    CHECK(script_position < layer_position);
}

// Publish a one-based tool-2 color marker before ordering so that tool becomes
// the first visit; export must use its 0.3 mm source height, while Print and
// Model stay test-owned and setup or export failures remain visible.
TEST_CASE("Color-change producer uses its tool group's source height",
          "[plugins][gcode][settings-scripts][custom-per-print-z][firmware]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_mixed_height_print(print, model, 1, 2);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.option<ConfigOptionString>("start_gcode")->value.clear();
    config.option<ConfigOptionString>("end_gcode")->value.clear();
    config.option<ConfigOptionString>("color_change_gcode")->value =
        "; COLOR_TOOL H{layer_current_height}";
    config.option<ConfigOptionStrings>("start_filament_gcode")->set(
        std::vector<std::string>{"", ""});
    config.option<ConfigOptionStrings>("end_filament_gcode")->set(
        std::vector<std::string>{"", ""});
    Steps::StepExtrusionOrdering::clear(print);
    Steps::StepExtrusionOrdering::prepare(print);
    publish_color_change_record(print, scale_i(0.8), 2);
    Steps::StepExtrusionOrdering::run_step(Orchestrator::instance(), print);
    PrintingPlan &plan = print.mutable_printing_plan();
    PrintingLayerGroup *mixed_layer = find_mixed_height_layer(plan, scale_i(0.8));
    REQUIRE(mixed_layer != nullptr);
    prepare_mixed_height_gcode_plan(print);
    mixed_layer = find_mixed_height_layer(plan, scale_i(0.8));
    REQUIRE(mixed_layer != nullptr);
    REQUIRE(slic3r_api::first_source_layer_height(
                slic3r_api::ConstPrintingLayerGroup(reinterpret_cast<const printing_layer_group_handle *>(
                mixed_layer))) == scale_i(0.3));
    const PrintingToolGroup *thick_tool = nullptr;
    for (const PrintingToolGroup &tool_group : mixed_layer->tool_groups)
        if (tool_group.extruder_id == 1)
            thick_tool = &tool_group;
    REQUIRE(thick_tool != nullptr);
    REQUIRE(slic3r_api::first_source_layer_height(
                slic3r_api::ConstPrintingToolGroup(reinterpret_cast<const printing_tool_group_handle *>(
                    thick_tool))) == scale_i(0.3));

    const std::string output = export_with_firmware(print, "marlin2");
    CHECK(output.find("; COLOR_TOOL H0.3\n") != std::string::npos);
}

// Export two distinct tool visits in one mixed layer and check the outgoing
// filament script's live layer state, its frozen destination Z and its position
// before the next layer marker; writer and script failures fail the assertions.
TEST_CASE("End-filament scripts at layer transitions describe the outgoing layer",
          "[plugins][gcode][settings-scripts][firmware]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_mixed_height_print(print, model, 1, 2);
    PrintingPlan &plan = install_mixed_height_plan(print);
    REQUIRE(plan.groups.size() == 1);
    std::vector<PrintingLayerGroup> &layers = plan.groups.front().layers;
    size_t transition_layer = layers.size();
    for (size_t layer_idx = 0; layer_idx < layers.size(); ++layer_idx)
        if (layers[layer_idx].print_z == scale_i(0.8))
            transition_layer = layer_idx;
    REQUIRE(transition_layer < layers.size());
    size_t next_tool_layer = transition_layer + 1;
    while (next_tool_layer < layers.size()) {
        bool has_next_tool = false;
        for (const PrintingToolGroup &tool_group : layers[next_tool_layer].tool_groups)
            has_next_tool = has_next_tool || tool_group.extruder_id == 1;
        if (has_next_tool)
            break;
        ++next_tool_layer;
    }
    REQUIRE(next_tool_layer < layers.size());
    const coord_t next_print_z = layers[next_tool_layer].print_z;
    std::ostringstream next_z_text;
    next_z_text << unscaled(next_print_z);
    for (size_t tool_idx = 0; tool_idx < layers[transition_layer].tool_groups.size();) {
        if (layers[transition_layer].tool_groups[tool_idx].extruder_id != 0)
            layers[transition_layer].tool_groups.erase(
                layers[transition_layer].tool_groups.begin() + tool_idx);
        else
            ++tool_idx;
    }
    for (size_t layer_idx = transition_layer + 1; layer_idx < next_tool_layer; ++layer_idx)
        layers[layer_idx].tool_groups.clear();
    for (size_t tool_idx = 0; tool_idx < layers[next_tool_layer].tool_groups.size();) {
        if (layers[next_tool_layer].tool_groups[tool_idx].extruder_id != 1)
            layers[next_tool_layer].tool_groups.erase(
                layers[next_tool_layer].tool_groups.begin() + tool_idx);
        else
            ++tool_idx;
    }
    REQUIRE(layers[transition_layer].tool_groups.size() == 1);
    REQUIRE(layers[transition_layer].tool_groups.front().extruder_id == 0);
    REQUIRE(layers[next_tool_layer].tool_groups.size() == 1);
    REQUIRE(layers[next_tool_layer].tool_groups.front().extruder_id == 1);

    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.option<ConfigOptionString>("start_gcode")->value.clear();
    config.option<ConfigOptionString>("end_gcode")->value.clear();
    config.before_layer_gcode.value.clear();
    config.layer_gcode.value.clear();
    config.option<ConfigOptionString>("toolchange_gcode")->value.clear();
    config.option<ConfigOptionStrings>("start_filament_gcode")->set(
        std::vector<std::string>{"", ""});
    config.option<ConfigOptionStrings>("end_filament_gcode")->set(
        std::vector<std::string>{
            "; BETWEEN L[layer_num] Z{layer_z} H{layer_current_height} T{toolchange_z}",
            ""
        });
    prepare_mixed_height_gcode_plan(print);

    const std::string output = export_with_firmware(print, "marlin2");
    const std::string expected = "; BETWEEN L" + std::to_string(transition_layer) +
        " Z0.8 H0.2 T" + next_z_text.str() + "\n";
    const size_t script_position = output.find(expected);
    const size_t next_layer_position = output.find(
        ";LAYER_CHANGE\n;Z:" + next_z_text.str() + "\n");
    REQUIRE(script_position != std::string::npos);
    REQUIRE(next_layer_position != std::string::npos);
    CHECK(script_position < next_layer_position);
}

// Export the first start-filament script on the 0.3 mm tool group of a mixed
// layer; layer number and Z stay producer snapshots while the emitted height is
// read from firmware state, and export errors remain visible.
TEST_CASE("Start-filament scripts use the source height of their mixed tool group",
          "[plugins][gcode][settings-scripts][firmware]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_mixed_height_print(print, model, 1, 2);
    PrintingPlan &plan = install_mixed_height_plan(print);
    PrintingLayerGroup *mixed_layer = find_mixed_height_layer(plan, scale_i(0.8));
    REQUIRE(mixed_layer != nullptr);
    const size_t mixed_layer_idx = size_t(mixed_layer - plan.groups.front().layers.data());
    const PrintingToolGroup *thick_tool = nullptr;
    for (const PrintingToolGroup &tool_group : mixed_layer->tool_groups)
        if (tool_group.extruder_id == 1)
            thick_tool = &tool_group;
    REQUIRE(thick_tool != nullptr);
    REQUIRE(slic3r_api::first_source_layer_height(
                slic3r_api::ConstPrintingToolGroup(reinterpret_cast<const printing_tool_group_handle *>(
                    thick_tool))) == scale_i(0.3));

    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.option<ConfigOptionString>("start_gcode")->value.clear();
    config.option<ConfigOptionString>("end_gcode")->value.clear();
    config.before_layer_gcode.value.clear();
    config.layer_gcode.value.clear();
    config.option<ConfigOptionString>("toolchange_gcode")->value.clear();
    config.option<ConfigOptionStrings>("start_filament_gcode")->set(
        std::vector<std::string>{"", "; START L[layer_num] Z{layer_z} H{layer_current_height}"});
    config.option<ConfigOptionStrings>("end_filament_gcode")->set(
        std::vector<std::string>{"", ""});
    prepare_mixed_height_gcode_plan(print);

    const std::string output = export_with_firmware(print, "marlin2");
    CHECK(output.find("; START L" + std::to_string(mixed_layer_idx) +
                       " Z0.8 H0.3\n") != std::string::npos);
}

// Trim the local plan after a mixed layer whose final ordered extrusion is from
// the 0.3 mm object, then export end_gcode; it must observe that final extrusion
// rather than the group's first source; Print owns the altered plan, and setup
// or export failures remain visible as failed assertions.
TEST_CASE("End G-code uses the last source extrusion of a final mixed layer",
          "[plugins][gcode][settings-scripts][firmware]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_mixed_height_print(print, model);
    PrintingPlan &plan = install_mixed_height_plan(print);
    PrintingLayerGroup *mixed_layer = find_mixed_height_layer(plan, scale_i(0.8));
    REQUIRE(mixed_layer != nullptr);
    REQUIRE(mixed_layer->tool_groups.size() == 1);
    PrintingToolGroup &tool_group = mixed_layer->tool_groups.front();
    size_t thick_extrusion_idx = tool_group.extrusions.size();
    for (size_t extrusion_idx = 0; extrusion_idx < tool_group.extrusions.size(); ++extrusion_idx) {
        const std::optional<coord_t> height = slic3r_api::source_layer_height(
            slic3r_api::ConstPrintingExtrusion(reinterpret_cast<const printing_extrusion_handle *>(
                &tool_group.extrusions[extrusion_idx])));
        if (height && *height == scale_i(0.3)) {
            thick_extrusion_idx = extrusion_idx;
            break;
        }
    }
    REQUIRE(thick_extrusion_idx < tool_group.extrusions.size());
    std::swap(tool_group.extrusions[thick_extrusion_idx], tool_group.extrusions.back());
    REQUIRE(slic3r_api::first_source_layer_height(
                slic3r_api::ConstPrintingLayerGroup(reinterpret_cast<const printing_layer_group_handle *>(
                    mixed_layer))) == scale_i(0.2));
    REQUIRE(slic3r_api::source_layer_height(
                slic3r_api::ConstPrintingExtrusion(reinterpret_cast<const printing_extrusion_handle *>(
                    &tool_group.extrusions.back()))) == scale_i(0.3));
    const size_t mixed_layer_idx = size_t(mixed_layer - plan.groups.front().layers.data());
    plan.groups.front().layers.erase(
        plan.groups.front().layers.begin() + mixed_layer_idx + 1,
        plan.groups.front().layers.end());

    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.option<ConfigOptionString>("start_gcode")->value.clear();
    config.option<ConfigOptionString>("end_gcode")->value =
        "; LAST L[layer_num] Z{layer_z} H{layer_current_height}";
    config.before_layer_gcode.value.clear();
    config.layer_gcode.value.clear();
    config.option<ConfigOptionStrings>("start_filament_gcode")->set(
        std::vector<std::string>{""});
    config.option<ConfigOptionStrings>("end_filament_gcode")->set(
        std::vector<std::string>{""});
    prepare_mixed_height_gcode_plan(print);

    const std::string output = export_with_firmware(print, "marlin2");
    CHECK(output.find("; LAST L" + std::to_string(mixed_layer_idx) +
                       " Z0.8 H0.3\n") != std::string::npos);
}

// Export a Print-owned mixed-height plan with a final end-filament placeholder;
// verify its toolchange_z is the last plan Z, while Print and Model stay alive.
// Pipeline and export failures remain visible as failed assertions.
TEST_CASE("Final end-filament scripts use the last layer Z for toolchange_z",
          "[plugins][gcode][settings-scripts][firmware]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_mixed_height_print(print, model);
    PrintingPlan &plan = install_mixed_height_plan(print);
    REQUIRE(plan.groups.size() == 1);
    REQUIRE_FALSE(plan.groups.front().layers.empty());
    const coord_t last_layer_z = plan.groups.front().layers.back().print_z;
    REQUIRE(last_layer_z > 0);

    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.option<ConfigOptionString>("start_gcode")->value.clear();
    config.option<ConfigOptionString>("end_gcode")->value.clear();
    config.before_layer_gcode.value.clear();
    config.layer_gcode.value.clear();
    config.option<ConfigOptionString>("toolchange_gcode")->value.clear();
    config.option<ConfigOptionStrings>("start_filament_gcode")->set(
        std::vector<std::string>{"", ""});
    config.option<ConfigOptionStrings>("end_filament_gcode")->set(
        std::vector<std::string>{"; FINAL_FILAMENT Z{toolchange_z}", ""});
    prepare_mixed_height_gcode_plan(print);

    const std::string output = export_with_firmware(print, "marlin2");
    const std::string marker = "; FINAL_FILAMENT Z";
    const size_t script_position = output.rfind(marker);
    REQUIRE(script_position != std::string::npos);
    const size_t value_begin = script_position + marker.size();
    const size_t line_end = output.find('\n', value_begin);
    REQUIRE(line_end != std::string::npos);
    CHECK(std::stod(output.substr(value_begin, line_end - value_begin)) ==
          Approx(unscaled(last_layer_z)));
}

// Insert a script before a later source extrusion of the same tool and give its
// paths a different HEIGHT attribute; export must use the linked layer height
// for script expansion while geometry retains its own attributes, with failures
// propagated through the normal plan writer. Print owns the plan nodes through
// export, and the script and geometry remain in that plan tree.
TEST_CASE("Inline extrusion scripts use source height rather than path attributes",
          "[plugins][gcode][settings-scripts][firmware]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_mixed_height_print(print, model);
    PrintingPlan &plan = install_mixed_height_plan(print);

    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.option<ConfigOptionString>("start_gcode")->value.clear();
    config.option<ConfigOptionString>("end_gcode")->value.clear();
    config.before_layer_gcode.value.clear();
    config.layer_gcode.value.clear();
    config.option<ConfigOptionString>("toolchange_gcode")->value.clear();
    config.option<ConfigOptionStrings>("start_filament_gcode")->set(
        std::vector<std::string>{""});
    config.option<ConfigOptionStrings>("end_filament_gcode")->set(
        std::vector<std::string>{""});
    prepare_mixed_height_gcode_plan(print);

    PrintingLayerGroup *mixed_layer = find_mixed_height_layer(plan, scale_i(0.8));
    REQUIRE(mixed_layer != nullptr);
    REQUIRE(mixed_layer->tool_groups.size() == 1);
    PrintingToolGroup &tool_group = mixed_layer->tool_groups.front();
    size_t target_extrusion_idx = tool_group.extrusions.size();
    bool saw_thin_source = false;
    for (size_t extrusion_idx = 0; extrusion_idx < tool_group.extrusions.size(); ++extrusion_idx) {
        const std::optional<coord_t> height = slic3r_api::source_layer_height(
            slic3r_api::ConstPrintingExtrusion(reinterpret_cast<const printing_extrusion_handle *>(
                &tool_group.extrusions[extrusion_idx])));
        if (height && *height == scale_i(0.2))
            saw_thin_source = true;
        else if (height && *height == scale_i(0.3) && saw_thin_source) {
            target_extrusion_idx = extrusion_idx;
            break;
        }
    }
    REQUIRE(target_extrusion_idx < tool_group.extrusions.size());
    PrintingExtrusion &target_extrusion = tool_group.extrusions[target_extrusion_idx];
    REQUIRE(target_extrusion.root != nullptr);
    REQUIRE(slic3r_api::source_layer_height(
                slic3r_api::ConstPrintingExtrusion(reinterpret_cast<const printing_extrusion_handle *>(
                    &target_extrusion))) == scale_i(0.3));

    std::vector<ExtrusionEntity *> pending{target_extrusion.root.get()};
    bool saw_path_attributes = false;
    while (!pending.empty()) {
        ExtrusionEntity *entity = pending.back();
        pending.pop_back();
        ExtrusionAttributes *attributes = entity->get_property<ExtrusionAttributes>();
        if (attributes != nullptr) {
            attributes->height = 0.17f;
            if (entity->has_polyline())
                saw_path_attributes = true;
        }
        for (size_t child_idx = 0; child_idx < entity->child_count(); ++child_idx)
            pending.push_back(&entity->child(child_idx));
    }
    REQUIRE(saw_path_attributes);
    ExtrusionEntity *script = target_extrusion.root->emplace_ordered_leaf(
        OrderedLeafPosition::Before, ExistingPropertyPlacement::MoveWithExistingContent);
    REQUIRE(script != nullptr);
    script->add_property(ExtrusionPropertyCustomGcodeText(
        ExtrusionPropertyCustomGcodeText::Code::SCRIPT,
        "; INLINE_SOURCE H{layer_current_height}",
        GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM));

    const std::string output = export_with_firmware(print, "marlin2");
    const size_t script_position = output.find("; INLINE_SOURCE H0.3\n");
    REQUIRE(script_position != std::string::npos);
    const size_t attribute_position = output.find(";HEIGHT:0.17\n", script_position);
    REQUIRE(attribute_position != std::string::npos);
    CHECK(script_position < attribute_position);
}

// Export a tool change between visits in one mixed layer and verify the outgoing
// filament event remains between its tool's extrusion and the next tool select;
// Print owns the plan, while writer or export failures remain visible as failed
// assertions and generated output stays local to the test.
TEST_CASE("End-filament scripts inside a mixed layer retain their scope position",
          "[plugins][gcode][settings-scripts][firmware]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_mixed_height_print(print, model, 1, 2);
    PrintingPlan &plan = install_mixed_height_plan(print);
    PrintingLayerGroup *mixed_layer = find_mixed_height_layer(plan, scale_i(0.8));
    REQUIRE(mixed_layer != nullptr);
    const size_t mixed_layer_idx = size_t(mixed_layer - plan.groups.front().layers.data());

    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.option<ConfigOptionString>("start_gcode")->value.clear();
    config.option<ConfigOptionString>("end_gcode")->value.clear();
    config.before_layer_gcode.value.clear();
    config.layer_gcode.value.clear();
    config.option<ConfigOptionString>("toolchange_gcode")->value.clear();
    config.option<ConfigOptionStrings>("start_filament_gcode")->set(
        std::vector<std::string>{"", ""});
    config.option<ConfigOptionStrings>("end_filament_gcode")->set(
        std::vector<std::string>{
            "; INNER_END L[layer_num] Z{layer_z} H{layer_current_height}", ""
        });
    prepare_mixed_height_gcode_plan(print);

    const std::string output = export_with_firmware(print, "marlin2");
    const size_t end_position = output.find(
        "; INNER_END L" + std::to_string(mixed_layer_idx) + " Z0.8 H0.2\n");
    REQUIRE(end_position != std::string::npos);
    const size_t next_tool_position = output.find("T1\n", end_position);
    REQUIRE(next_tool_position != std::string::npos);
    CHECK(end_position < next_tool_position);
}

TEST_CASE("First layer bed temperature follows used tools unless overridden",
          "[plugins][gcode][script-type][temperature]") {
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_extruders(2);
    config.option<ConfigOptionInts>("first_layer_bed_temperature")->set(std::vector<int32_t>{55, 73});
    config.option<ConfigOptionInt>("print_first_layer_bed_temperature")->set_enabled(false);
    config.set_deserialize_strict({{"perimeter_extruder", 2}, {"infill_extruder", 2}, {"solid_infill_extruder", 2}});

    Print print;
    Model model;
    Test::init_print({make_cube(5., 5., 0.2)}, print, model, config);
    print.process();
    CHECK(print.first_layer_bed_temperature() == 73);

    PrintConfig &print_config = const_cast<PrintConfig &>(print.config());
    print_config.print_first_layer_bed_temperature.value = 64;
    print_config.print_first_layer_bed_temperature.set_enabled(true);
    CHECK(print.first_layer_bed_temperature() == 64);
}
TEST_CASE("End script uses stable defaults when a plan has no layers",
          "[plugins][gcode][script-type][firmware][integration]") {
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    select_printing_plan_firmware(print);
    configure_standard_firmware(print);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.option<ConfigOptionString>("start_gcode")->value.clear();
    config.option<ConfigOptionString>("end_gcode")->value = "; EMPTY_END L[layer_num] Z[layer_z] M[max_layer_z] F[filament_extruder_id] "
                             "P[previous_extruder] N[next_extruder]";
    print.mutable_printing_plan();

    Steps::StepExtrusionEdition::clear(print);
    Steps::StepExtrusionEdition::run_step(Orchestrator::instance(), print);

    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    Orchestrator &orchestrator = Orchestrator::instance();
    orchestrator.reset_plugin_cancel();
    try {
        Steps::StepGenerateGcode::run_step(orchestrator, print, output_path.string());
        const std::string output = read_text_file(output_path);
        INFO(output);
        CHECK(output == "G21\nG90\nM82\n; EMPTY_END L-1 Z0 M0 F0 P-1 N-1\n");
    } catch (...) {
        orchestrator.reset_plugin_cancel();
        remove_output_pair(output_path);
        throw;
    }
    orchestrator.reset_plugin_cancel();
    remove_output_pair(output_path);
}


TEST_CASE("Settings scripts plugin owns the configured script settings",
          "[plugins][gcode][settings-scripts][config]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();

    Plugin *plugin = orchestrator.get_plugin("gcode.settings_scripts");
    REQUIRE(plugin != nullptr);

    std::vector<std::string> defined_keys;
    for (const Plugin::DefinedConfigKey &key : plugin->get_defined_config_keys())
        defined_keys.push_back(key.key);
    CHECK(defined_keys.size() == 6);
    for (const char *expected : { "start_gcode", "end_gcode", "between_objects_gcode",
                                  "between_objects_gcode_before_move", "start_filament_gcode", "end_filament_gcode" })
        CHECK(std::find(defined_keys.begin(), defined_keys.end(), expected) != defined_keys.end());

    const ConfigOptionDef *def = PrintConfigDef::instance().get("start_gcode");
    REQUIRE(def != nullptr);
    CHECK(def->container_type == ConfigOptionContainerType::Plater);
    CHECK(def->option_preset_type == RAW_PRESET_TYPE_FFF_PRINTER);
    CHECK(def->invalidates_step == STEP_EXTRUSION_EDIT);
    CHECK(def->multiline);
    CHECK(def->full_width);

    def = PrintConfigDef::instance().get("end_gcode");
    REQUIRE(def != nullptr);
    CHECK(def->container_type == ConfigOptionContainerType::Plater);
    CHECK(def->option_preset_type == RAW_PRESET_TYPE_FFF_PRINTER);
    CHECK(def->invalidates_step == STEP_EXTRUSION_EDIT);

    def = PrintConfigDef::instance().get("between_objects_gcode");
    REQUIRE(def != nullptr);
    CHECK(def->option_preset_type == RAW_PRESET_TYPE_FFF_PRINTER);
    CHECK(def->invalidates_step == STEP_EXTRUSION_EDIT);

    def = PrintConfigDef::instance().get("between_objects_gcode_before_move");
    REQUIRE(def != nullptr);
    CHECK(def->type == coBool);
    CHECK(def->option_preset_type == RAW_PRESET_TYPE_FFF_PRINTER);
    CHECK(def->invalidates_step == STEP_EXTRUSION_EDIT);

    def = PrintConfigDef::instance().get("start_filament_gcode");
    REQUIRE(def != nullptr);
    CHECK(def->container_type == ConfigOptionContainerType::Plater);
    CHECK(def->option_preset_type == RAW_PRESET_TYPE_FFF_FILAMENT);
    CHECK(def->invalidates_step == STEP_EXTRUSION_EDIT);
    CHECK(def->multiline);
    CHECK(def->full_width);
    CHECK(def->is_vector_extruder);

    def = PrintConfigDef::instance().get("end_filament_gcode");
    REQUIRE(def != nullptr);
    CHECK(def->option_preset_type == RAW_PRESET_TYPE_FFF_FILAMENT);
    CHECK(def->invalidates_step == STEP_EXTRUSION_EDIT);
    CHECK(def->is_vector_extruder);

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    CHECK(config.option<ConfigOptionString>("start_gcode")->value ==
          "G28 ; home all axes\nG1 Z5 F5000 ; lift nozzle\n");
    CHECK(config.option<ConfigOptionString>("end_gcode")->value ==
          "M104 S0 ; turn off temperature\nG28 X0  ; home X axis\nM84     ; disable motors\n");
    CHECK(config.option<ConfigOptionString>("between_objects_gcode")->value.empty());
    CHECK_FALSE(config.option<ConfigOptionBool>("between_objects_gcode_before_move")->value);
    CHECK(config.option<ConfigOptionStrings>("start_filament_gcode")->get_at(0) ==
          "; Filament gcode\n");
    CHECK(config.option<ConfigOptionStrings>("end_filament_gcode")->get_at(0) ==
          "; Filament-specific end gcode \n;END gcode for filament\n");

    // The configured scripts stay always editable: the plugin owns no rule.
    for (const Orchestrator::PluginGuiRule &rule : orchestrator.gui_rules().rules())
        CHECK(rule.owner != "gcode.settings_scripts");
}

// Export two real objects in parallel and sequential modes, compare script Z with layer markers, and derive script height from this test setup's equal source heights. Both cubes share layer_height and the skirt uses first-layer height, so this equality is test-specific, not a printing-plan invariant; export or parsing failures are reported by assertions.
TEST_CASE("Exported scripts read the legacy export variables",
          "[plugins][gcode][script-type][integration][script-globals]")
{
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    for (const bool sequential : {false, true}) {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.set_num_extruders(2);
        config.set_deserialize_strict({
            {"wipe_tower", "0"},
            {"filament_diameter", "1.75,1.75"},
            {"extrusion_multiplier", "1,1"},
            {"filament_density", "1.25,2"},
            {"filament_type", "PLA;PETG"},
            {"perimeter_extruder", "1"},
            {"infill_extruder", "1"},
            {"solid_infill_extruder", "1"},
            {"top_solid_layers", "0"},
            {"bottom_solid_layers", "0"},
            {"print_plan_builder", sequential ? "ordering.plan_builder.by_object" : "ordering.plan_builder.by_layer"},
            {"step_perimeter_plugin", "perimeter.generator.classic"},
            {"step_surface_generation_plugin", "surface.initial_typed_surface_builder"},
            {"step_wipetower_plugin", "wipe_tower.manager.default"},
            {"layer_height", "0.3"},
            {"fill_density", "20%"}});
        config.option<ConfigOptionString>("start_gcode")->value =
            "; GS T[initial_tool] F[initial_filament_type] C[total_toolchanges] "
            "L[total_layer_count] A[all_layer_count] N[num_extruders] "
            "U{if is_extruder_used[0]}1{else}0{endif}"
            "{if is_extruder_used[1]}1{else}0{endif}"
            "{if is_extruder_used[2]}1{else}0{endif} "
            "V{extruded_volume_total} O[current_object_idx] K[layer_num] "
            "H{layer_current_height}\n"
            "; GB {bounding_box[0]} {bounding_box[1]} {bounding_box[2]} "
            "{bounding_box[3]} {bounding_box[4]} {bounding_box[5]}\n"
            "; GF {first_layer_print_min[0]} {first_layer_print_min[1]} "
            "{first_layer_print_max[0]} {first_layer_print_max[1]}\n"
            "; GP {print_bed_size[0]} {print_bed_size[1]}";
        config.option<ConfigOptionString>("feature_gcode")->value =
            "; GFEAT [layer_num] {layer_z} {layer_current_height} {max_layer_z}";
        config.option<ConfigOptionString>("end_gcode")->value =
            "; GE O[current_object_idx] V{extruded_volume_total} "
            "{extruded_volume[0]} {extruded_volume[1]}";
        config.option<ConfigOptionString>("between_objects_gcode")->value =
            "; GO [current_object_idx]";
        config.option<ConfigOptionString>("toolchange_gcode")->value.clear();
        config.option<ConfigOptionStrings>("start_filament_gcode")->set(
            std::vector<std::string>{"", ""});
        config.option<ConfigOptionStrings>("end_filament_gcode")->set(
            std::vector<std::string>{"", ""});

        Print print;
        Model model;
        Test::init_print({Test::TestMesh::cube_20x20x20, Test::TestMesh::cube_20x20x20},
                         print, model, config);
        model.objects()[0].instances.front()->set_offset(Vec3d(40., 40., 0.));
        model.objects()[1].instances.front()->set_offset(Vec3d(100., 40., 0.));
        for (const char *key : {"perimeter_extruder", "infill_extruder", "solid_infill_extruder"})
            model.objects()[1].config.set_deserialize_strict(key, "2");
        print.apply(model, config);
        REQUIRE_NOTHROW(print.process());

        extruder_index_t initial_tool = EXTRUDER_ID_UNKNOWN;
        extruder_index_t previous_tool = EXTRUDER_ID_UNKNOWN;
        int32_t expected_toolchanges = 0;
        int32_t expected_all_layer_count = 0;
        coord_t expected_max_print_z = 0;
        const PrintObject &first_object = print.objects().front();
        std::set<coord_t> object_layer_zs;
        for (const Layer &layer : first_object.layers())
            if (layer.has_extrusions())
                object_layer_zs.insert(layer.scaled_print_z());
        const int32_t expected_total_layer_count = sequential ?
            int32_t(object_layer_zs.size() * 2) : int32_t(object_layer_zs.size());
        const double expected_first_layer_height = first_object.layers().front().unscaled_height();

        const boost::filesystem::path output_path = temporary_gcode_path();
        remove_output_pair(output_path);
        Orchestrator &orchestrator = Orchestrator::instance();
        orchestrator.reset_plugin_cancel();
        try {
            REQUIRE_NOTHROW(orchestrator.export_gcode(print, output_path.string(), nullptr));
            const std::string output = read_text_file(output_path);
            INFO(output);

            const PrintingPlan *plan = print.printing_plan();
            REQUIRE(plan != nullptr);
            std::vector<double> expected_layer_heights;
            for (const PrintingGroup &group : plan->groups)
                for (const PrintingLayerGroup &layer : group.layers) {
                    REQUIRE_FALSE(layer.layers.empty());
                    REQUIRE(layer.layers.front() != nullptr);
                    // This test setup has equal source heights because both cubes
                    // share layer_height and the skirt uses first-layer height.
                    // Mixed groups are valid; see "Default firmware layer change describes mixed-height print Z groups",
                    // "Default firmware accepts different object layer heights in the by-layer plan", and
                    // "Default firmware exports different object layer heights in the by-layer plan".
                    const coord_t source_height = layer.layers.front()->scaled_height();
                    for (const Layer *source_layer : layer.layers) {
                        REQUIRE(source_layer != nullptr);
                        REQUIRE(source_layer->scaled_height() == source_height);
                    }
                    expected_layer_heights.push_back(
                        layer.layers.front()->unscaled_height());
                    ++expected_all_layer_count;
                    expected_max_print_z = std::max(expected_max_print_z, layer.print_z);
                    for (const PrintingToolGroup &tool : layer.tool_groups) {
                        if (tool.extruder_id == EXTRUDER_ID_UNKNOWN)
                            continue;
                        if (initial_tool == EXTRUDER_ID_UNKNOWN)
                            initial_tool = tool.extruder_id;
                        else if (previous_tool != EXTRUDER_ID_UNKNOWN && previous_tool != tool.extruder_id)
                            ++expected_toolchanges;
                        previous_tool = tool.extruder_id;
                    }
                }
            REQUIRE(initial_tool != EXTRUDER_ID_UNKNOWN);
            const double expected_max_layer_z = unscaled(expected_max_print_z);
            const std::string expected_filament_type = print.config().filament_type.get_at(
                static_cast<size_t>(initial_tool));

            std::string start_line;
            std::string bounding_box_line;
            std::string first_layer_line;
            std::string bed_line;
            std::string end_line;
            size_t layer_change_count = 0;
            size_t feature_count = 0;
            std::set<int32_t> feature_layers;
            double last_layer_z = 0.0;
            std::istringstream output_lines(output);
            std::string line;
            while (std::getline(output_lines, line)) {
                if (!line.empty() && line.back() == '\r')
                    line.pop_back();
                if (line == ";LAYER_CHANGE")
                    ++layer_change_count;
                else if (line.rfind(";Z:", 0) == 0)
                    last_layer_z = std::stod(line.substr(3));
                else if (line.rfind("; GS ", 0) == 0)
                    start_line = line;
                else if (line.rfind("; GB ", 0) == 0)
                    bounding_box_line = line;
                else if (line.rfind("; GF ", 0) == 0)
                    first_layer_line = line;
                else if (line.rfind("; GP ", 0) == 0)
                    bed_line = line;
                else if (line.rfind("; GE ", 0) == 0)
                    end_line = line;
                else if (line.rfind("; GFEAT ", 0) == 0) {
                    ++feature_count;
                    std::istringstream feature_values(line.substr(8));
                    int32_t feature_layer = -1;
                    double feature_z = 0.0;
                    double feature_height = 0.0;
                    double feature_max_z = 0.0;
                    const bool parsed = static_cast<bool>(feature_values >> feature_layer >>
                                                           feature_z >> feature_height >> feature_max_z);
                    REQUIRE(parsed);
                    REQUIRE(layer_change_count > 0);
                    REQUIRE(feature_layer >= 0);
                    REQUIRE(static_cast<size_t>(feature_layer) <
                            expected_layer_heights.size());
                    CHECK(feature_layer == int32_t(layer_change_count - 1));
                    CHECK(feature_z == Approx(last_layer_z).margin(1e-6));
                    CHECK(feature_height ==
                          Approx(expected_layer_heights[
                              static_cast<size_t>(feature_layer)]).margin(1e-6));
                    CHECK(feature_max_z == Approx(expected_max_layer_z).margin(1e-6));
                    feature_layers.insert(feature_layer);
                }
            }

            REQUIRE_FALSE(start_line.empty());
            REQUIRE_FALSE(bounding_box_line.empty());
            REQUIRE_FALSE(first_layer_line.empty());
            REQUIRE_FALSE(bed_line.empty());
            REQUIRE_FALSE(end_line.empty());
            REQUIRE(feature_count > 0);
            CHECK(feature_layers.size() == size_t(expected_all_layer_count));

            std::istringstream start_values(start_line.substr(5));
            std::string token;
            bool has_token = static_cast<bool>(start_values >> token);
            REQUIRE(has_token);
            CHECK(token == std::string("T") + std::to_string(initial_tool));
            has_token = static_cast<bool>(start_values >> token);
            REQUIRE(has_token);
            CHECK(token == std::string("F") + expected_filament_type);
            has_token = static_cast<bool>(start_values >> token);
            REQUIRE(has_token);
            CHECK(token == std::string("C") + std::to_string(expected_toolchanges));
            has_token = static_cast<bool>(start_values >> token);
            REQUIRE(has_token);
            CHECK(token == std::string("L") + std::to_string(expected_total_layer_count));
            has_token = static_cast<bool>(start_values >> token);
            REQUIRE(has_token);
            CHECK(token == std::string("A") + std::to_string(expected_all_layer_count));
            has_token = static_cast<bool>(start_values >> token);
            REQUIRE(has_token);
            CHECK(token == "N2");
            has_token = static_cast<bool>(start_values >> token);
            REQUIRE(has_token);
            CHECK(token == "U110");
            has_token = static_cast<bool>(start_values >> token);
            REQUIRE(has_token);
            CHECK(std::stod(token.substr(1)) == Approx(0.0).margin(1e-6));
            has_token = static_cast<bool>(start_values >> token);
            REQUIRE(has_token);
            CHECK(token == "O0");
            has_token = static_cast<bool>(start_values >> token);
            REQUIRE(has_token);
            CHECK(token == "K0");
            has_token = static_cast<bool>(start_values >> token);
            REQUIRE(has_token);
            CHECK(std::stod(token.substr(1)) == Approx(expected_first_layer_height).margin(1e-6));

            std::istringstream bounding_box_values(bounding_box_line.substr(5));
            std::vector<double> bounding_box;
            for (size_t idx = 0; idx < 6; ++idx) {
                double value = 0.0;
                const bool parsed = static_cast<bool>(bounding_box_values >> value);
                REQUIRE(parsed);
                bounding_box.push_back(value);
            }
            CHECK(bounding_box[2] == Approx(0.0).margin(1e-3));
            CHECK(bounding_box[5] == Approx(20.0).margin(1e-3));
            CHECK(bounding_box[3] - bounding_box[0] >= 20.0);
            CHECK(bounding_box[4] - bounding_box[1] >= 20.0);

            std::istringstream first_layer_values(first_layer_line.substr(5));
            std::vector<double> first_layer_box;
            for (size_t idx = 0; idx < 4; ++idx) {
                double value = 0.0;
                const bool parsed = static_cast<bool>(first_layer_values >> value);
                REQUIRE(parsed);
                first_layer_box.push_back(value);
            }
            CHECK(first_layer_box[0] <= bounding_box[0] + 1e-3);
            CHECK(first_layer_box[1] <= bounding_box[1] + 1e-3);
            CHECK(first_layer_box[2] >= bounding_box[3] - 1e-3);
            CHECK(first_layer_box[3] >= bounding_box[4] - 1e-3);

            std::istringstream bed_values(bed_line.substr(5));
            double bed_width = 0.0;
            double bed_depth = 0.0;
            const bool parsed_bed = static_cast<bool>(bed_values >> bed_width >> bed_depth);
            REQUIRE(parsed_bed);
            const BoundingBoxf bed_box(print.config().bed_shape.get_values());
            REQUIRE(bed_box.defined);
            CHECK(bed_width == Approx(bed_box.max.x() - bed_box.min.x()).margin(1e-6));
            CHECK(bed_depth == Approx(bed_box.max.y() - bed_box.min.y()).margin(1e-6));

            std::istringstream end_values(end_line.substr(5));
            has_token = static_cast<bool>(end_values >> token);
            REQUIRE(has_token);
            CHECK(token == (sequential ? "O1" : "O0"));
            has_token = static_cast<bool>(end_values >> token);
            REQUIRE(has_token);
            const double total_volume = std::stod(token.substr(1));
            double tool_zero_volume = 0.0;
            double tool_one_volume = 0.0;
            const bool parsed_volumes = static_cast<bool>(end_values >> tool_zero_volume >> tool_one_volume);
            REQUIRE(parsed_volumes);
            CHECK(tool_zero_volume > 0.0);
            CHECK(tool_one_volume > 0.0);
            CHECK(total_volume == Approx(tool_zero_volume + tool_one_volume).margin(1e-6));

            if (sequential)
                CHECK(output.find("; GO 1") != std::string::npos);
            else
                CHECK(output.find("; GO ") == std::string::npos);
        } catch (...) {
            orchestrator.reset_plugin_cancel();
            remove_output_pair(output_path);
            throw;
        }
        orchestrator.reset_plugin_cancel();
        remove_output_pair(output_path);
    }
}
