#include <catch2/catch.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

#include <boost/filesystem.hpp>

#include "plugin_test_helpers.hpp"
#include "gcode_test_helpers.hpp"

#include "libslic3r/Api/host/ApiHostUtils.hpp"
#include "libslic3r/Api/host/GCodeScriptProcessor.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/c/slic3r_gcode_reader.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_gcode.h"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/DefaultGCodeFirmwareSession.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeFirmwareViews.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeScriptProcessorViews.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/MachineEnvelope.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Config/ConfigSnapshotSerialization.hpp"
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
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"
#include "libslic3r/Steps/StepExtrusionEdition.hpp"
#include "libslic3r/Steps/StepGenerateGcode.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"
#include "test_data.hpp"

/*
G-code script processor tests cover the host PlaceholderParser bridge, stored Config snapshots and machine-state import.
Shared plan construction and output helpers live in gcode_test_helpers so this file keeps only component-specific behavior.
*/

namespace {
using namespace Slic3r;
using namespace Slic3r::Printing;
using namespace Slic3r::Test::GCode;

// Verifies that a derived firmware may augment the host-prepared context at
// execution time without owning or exposing the PlaceholderParser itself.
class ScriptContextCompletingFirmwareProbe final :
    public slic3r_api::GCodeGeneration::DefaultGCodeFirmwareSession
{
public:
    explicit ScriptContextCompletingFirmwareProbe(
        slic3r_api::GCodeGeneration::GCodeScriptProcessorView scripts,
        storage_handle *storage = nullptr) :
        DefaultGCodeFirmwareSession(scripts, storage)
    {}

    using DefaultGCodeFirmwareSession::process_script;

    uint32_t completion_count() const { return m_completion_count; }

protected:
    void complete_script_context(
        gcode_script_type script_type,
        extruder_index_t,
        slic3r_api::GCodeGeneration::GCodeScriptConfig &config) override
    {
        ++m_completion_count;
        if (script_type == GCODE_SCRIPT_TYPE_START_GCODE && config.has("max_layer_z"))
            config.set("max_layer_z", 12.5);
    }

private:
    uint32_t m_completion_count = 0;
};

} // namespace

TEST_CASE("Host G-code script processor prepares typed isolated contexts",
          "[plugins][gcode][firmware][custom-gcode]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    configure_standard_firmware(print, 2);
    DynamicPrintConfig &full_config =
        const_cast<DynamicPrintConfig &>(print.full_print_config());
    full_config.set_key_value("travel_speed", new ConfigOptionFloat(100.0));
    Orchestrator &orchestrator = Orchestrator::instance();
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    plan.groups.front().layers.front().tool_groups.emplace_back();
    plan.groups.front().layers.front().tool_groups.front().extruder_id = 1;
    GCodeScriptProcessor host_processor(print, orchestrator);
    const slic3r_api::GCodeGeneration::GCodeScriptProcessorView scripts(
        host_processor.c_processor());
    CHECK_THROWS(scripts.prepare(GCODE_SCRIPT_TYPE_INVALID));
    CHECK_THROWS(scripts.prepare(GCODE_SCRIPT_TYPE_CUSTOM_BEGIN - 1));

    PluginStorage argument_storage;
    storage_handle *argument_storage_handle =
        reinterpret_cast<storage_handle *>(&argument_storage);
    slic3r_api::StoredExtrusionEntity layer_event(
        argument_storage_handle);
    slic3r_api::StoredConfig layer_arguments(argument_storage_handle);
    layer_arguments.get_or_add("layer_num", SLIC3R_CONFIG_OPTION_INT).set_int(12);
    layer_arguments.get_or_add("layer_z", SLIC3R_CONFIG_OPTION_FLOAT).set_float(2.6);
    layer_arguments.get_or_add("previous_layer_z", SLIC3R_CONFIG_OPTION_FLOAT).set_float(2.4);
    layer_arguments.get_or_add("max_layer_z", SLIC3R_CONFIG_OPTION_FLOAT).set_float(20.0);
    layer_arguments.get_or_add("custom_bool", SLIC3R_CONFIG_OPTION_BOOL).set_bool(true);
    layer_arguments.get_or_add("custom_int", SLIC3R_CONFIG_OPTION_INT).set_int(-7);
    layer_arguments.get_or_add("custom_float", SLIC3R_CONFIG_OPTION_FLOAT).set_float(3.25);
    layer_arguments.get_or_add("custom_string", SLIC3R_CONFIG_OPTION_STRING).set_string("typed");
    slic3r_api::MutableConfigOption custom_ints =
        layer_arguments.get_or_add("custom_ints", SLIC3R_CONFIG_OPTION_INTS);
    custom_ints.resize(2);
    custom_ints.set_int(2, 0);
    custom_ints.set_int(4, 1);
    slic3r_api::MutableConfigOption custom_floats =
        layer_arguments.get_or_add("custom_floats", SLIC3R_CONFIG_OPTION_FLOATS);
    custom_floats.resize(2);
    custom_floats.set_float(1.5, 0);
    custom_floats.set_float(2.5, 1);
    const slic3r_api::EPropertyCustomGcode &layer_property = layer_event.script_gcode(
        "M117 layer", GCODE_SCRIPT_TYPE_BEFORE_LAYER_GCODE, layer_arguments);
    slic3r_api::StoredConfig stored_arguments(argument_storage_handle);
    stored_arguments.deserialize_all(layer_event.stored_string(layer_property.config_id));

    const slic3r_api::GCodeGeneration::GCodeScriptContext prepared_layer =
        scripts.prepare(GCODE_SCRIPT_TYPE_BEFORE_LAYER_GCODE, stored_arguments);
    CHECK(prepared_layer.config().get_int("layer_num") == 12);
    CHECK(prepared_layer.config().get_float("layer_z") == Approx(2.6));
    CHECK(prepared_layer.config().get_float("previous_layer_z") == Approx(2.4));
    CHECK(prepared_layer.config().get_float("max_layer_z") == Approx(20.0));
    CHECK(prepared_layer.config().get_bool("custom_bool"));
    CHECK(prepared_layer.config().get_int("custom_int") == -7);
    CHECK(prepared_layer.config().get_float("custom_float") == Approx(3.25));
    CHECK(prepared_layer.config().get_string("custom_string") == "typed");
    CHECK((prepared_layer.config().get_ints("custom_ints") == std::vector<int32_t>{2, 4}));
    CHECK((prepared_layer.config().get_floats("custom_floats") == std::vector<double>{1.5, 2.5}));
    CHECK(prepared_layer.process("L[layer_num] Z[layer_z] P[previous_layer_z] M[max_layer_z]", 0) ==
          "L12 Z2.6 P2.4 M20");

    // Feature scripts use the same producer Config path as every other script.
    // The modern names and their historical aliases therefore arrive as four
    // ordinary typed strings, with no feature-specific host context structure.
    slic3r_api::StoredConfig feature_arguments(argument_storage_handle);
    feature_arguments.get_or_add("previous_extrusion_role", SLIC3R_CONFIG_OPTION_STRING)
        .set_string("Travel");
    feature_arguments.get_or_add("next_extrusion_role", SLIC3R_CONFIG_OPTION_STRING)
        .set_string("Internal infill");
    feature_arguments.get_or_add("last_extrusion_role", SLIC3R_CONFIG_OPTION_STRING)
        .set_string("Travel");
    feature_arguments.get_or_add("extrusion_role", SLIC3R_CONFIG_OPTION_STRING)
        .set_string("Internal infill");
    const slic3r_api::GCodeGeneration::GCodeScriptContext prepared_feature =
        scripts.prepare(GCODE_SCRIPT_TYPE_FEATURE_GCODE, feature_arguments);
    CHECK(prepared_feature.process(
              "[previous_extrusion_role]>[next_extrusion_role] "
              "[last_extrusion_role]>[extrusion_role]", 0) ==
          "Travel>Internal infill Travel>Internal infill");

    // Producers may populate structural placeholders, but they cannot change
    // the type promised by the host's built-in script definition.
    slic3r_api::StoredExtrusionEntity wrong_type_event(
        argument_storage_handle);
    slic3r_api::StoredConfig wrong_type_arguments(argument_storage_handle);
    wrong_type_arguments.get_or_add("layer_num", SLIC3R_CONFIG_OPTION_FLOAT).set_float(12.0);
    const slic3r_api::EPropertyCustomGcode &wrong_type_property = wrong_type_event.script_gcode(
        "M117 wrong", GCODE_SCRIPT_TYPE_BEFORE_LAYER_GCODE, wrong_type_arguments);
    slic3r_api::StoredConfig stored_wrong_type(argument_storage_handle);
    stored_wrong_type.deserialize_all(wrong_type_event.stored_string(wrong_type_property.config_id));
    CHECK_THROWS(scripts.prepare(GCODE_SCRIPT_TYPE_BEFORE_LAYER_GCODE, stored_wrong_type));

    // Runtime machine values are owned by the firmware and remain protected
    // even when a producer transports an otherwise valid typed vector.
    slic3r_api::StoredExtrusionEntity reserved_event(
        argument_storage_handle);
    slic3r_api::StoredConfig reserved_arguments(argument_storage_handle);
    slic3r_api::MutableConfigOption position =
        reserved_arguments.get_or_add("position", SLIC3R_CONFIG_OPTION_FLOATS);
    position.resize(3);
    position.set_float(1.0, 0);
    position.set_float(2.0, 1);
    position.set_float(3.0, 2);
    const slic3r_api::EPropertyCustomGcode &reserved_property = reserved_event.script_gcode(
        "M117 reserved", GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM, reserved_arguments);
    slic3r_api::StoredConfig stored_reserved(argument_storage_handle);
    stored_reserved.deserialize_all(reserved_event.stored_string(reserved_property.config_id));
    CHECK_THROWS(scripts.prepare(GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM, stored_reserved));

    // Every name receives fresh machine state, while known names additionally
    // expose the exact option types declared by the legacy placeholder table.
    for (const auto &named_placeholders : custom_gcode_specific_placeholders()) {
        const slic3r_api::GCodeGeneration::GCodeScriptContext context = scripts.prepare(
            registered_script_type(orchestrator, named_placeholders.first));
        const slic3r_api::GCodeGeneration::GCodeScriptConfig config = context.config();
        for (const std::string &key : named_placeholders.second) {
            INFO(named_placeholders.first << ": " << key);
            const ConfigOptionDef *definition = custom_gcode_specific_config_def.get(key);
            REQUIRE(definition != nullptr);
            const config_option_handle *option = config_get(config.handle(), key.c_str());
            REQUIRE(option != nullptr);
            CHECK(config_option_type_get(option) ==
                  static_cast<config_option_type>(definition->type));
        }
    }

    const slic3r_api::GCodeGeneration::GCodeScriptContext toolchange = scripts.prepare(
        registered_script_type(orchestrator, "toolchange_gcode"));
    REQUIRE(toolchange.config().has("previous_extruder"));
    REQUIRE(toolchange.config().has("next_extruder"));
    toolchange.set("previous_extruder", int32_t(1));
    toolchange.set("next_extruder", int32_t(0));
    CHECK(toolchange.process(
              "M117 T{previous_extruder}>{next_extruder}", 0) ==
          "M117 T1>0");

    // prepare() discards per-call values rather than leaking the previous
    // toolchange context into the next script.
    const slic3r_api::GCodeGeneration::GCodeScriptContext fresh_toolchange = scripts.prepare(
        registered_script_type(orchestrator, "toolchange_gcode"));
    CHECK(fresh_toolchange.config().get_int("previous_extruder") == 0);
    CHECK_THROWS_AS(
        fresh_toolchange.set("previous_extruder", 1.5), std::invalid_argument);
    CHECK(fresh_toolchange.process("", 0).empty());

    const slic3r_api::GCodeGeneration::GCodeScriptContext color_change = scripts.prepare(
        registered_script_type(orchestrator, "color_change_gcode"));
    color_change.set("next_color", std::string("#12ab34"));
    CHECK(color_change.process("M117 {next_color}", 0) == "M117 #12ab34");

    // Names unknown to the specific-placeholder table remain useful: they see
    // the Print configuration and only the standard machine in/out options.
    const slic3r_api::GCodeGeneration::GCodeScriptContext generic = scripts.prepare(
        registered_script_type(orchestrator, "unknown_script"));
    CHECK(generic.process("M117 F{travel_speed}", 0) == "M117 F100");

    const slic3r_api::GCodeGeneration::GCodeScriptContext before_layer = scripts.prepare(
        registered_script_type(orchestrator, "before_layer_gcode"));
    const config_option_handle *used_filament =
        config_get(before_layer.config().handle(), "layer_used_filament");
    REQUIRE(used_filament != nullptr);
    CHECK(config_option_type_get(used_filament) == SLIC3R_CONFIG_OPTION_FLOATS);
    CHECK(config_option_size(used_filament) == 2);
    CHECK(before_layer.process("", 0).empty());

    // Parser globals deliberately survive between scripts handled by one
    // export, but a new host processor starts with an independent dictionary.
    CHECK(scripts.prepare(registered_script_type(orchestrator, "unknown_script"))
              .process("{global firmware_script_counter=7}", 0)
              .empty());
    CHECK(scripts.prepare(registered_script_type(orchestrator, "unknown_script"))
              .process("{firmware_script_counter}", 0) == "7");

    GCodeScriptProcessor independent_host(print, orchestrator);
    const slic3r_api::GCodeGeneration::GCodeScriptProcessorView independent_scripts(
        independent_host.c_processor());
    CHECK_THROWS(independent_scripts.prepare(registered_script_type(orchestrator, "unknown_script"))
                     .process("{firmware_script_counter}", 0));
}

// Verify host-published export constants and zero-initialized execution options before firmware runs.
TEST_CASE("Host G-code script processor publishes export constants and execution options",
          "[plugins][gcode][custom-gcode][script-globals]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    configure_standard_firmware(print, 2);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.bed_shape.set({{-10., -5.}, {190., -5.}, {190., 215.}, {-10., 215.}});
    config.option<ConfigOptionString>("thumbnails_color")->value = "#102030";
    config.option<ConfigOptionStrings>("filament_colour")->set(
        std::vector<std::string>{"#FF0000", "#00ff00"});
    config.option<ConfigOptionStrings>("extruder_colour")->set(
        std::vector<std::string>{"", "#0000FF"});
    config.option<ConfigOptionStrings>("filament_type")->set(
        std::vector<std::string>{"PLA", "PETG"});
    DynamicPrintConfig &full_config = const_cast<DynamicPrintConfig &>(print.full_print_config());
    full_config.set_key_value("layer_height", new ConfigOptionFloat(0.25));

    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups[0].layers.emplace_back();
    plan.groups[0].layers[0].print_z = scale_i(0.2);
    plan.groups[0].layers[0].tool_groups.emplace_back();
    plan.groups[0].layers[0].tool_groups[0].extruder_id = 1;
    plan.groups[0].layers[0].tool_groups.emplace_back();
    plan.groups[0].layers[0].tool_groups[1].extruder_id = 0;
    plan.groups[0].layers.emplace_back();
    plan.groups[0].layers[1].print_z = scale_i(0.4);
    plan.groups[0].layers[1].tool_groups.emplace_back();
    plan.groups[0].layers[1].tool_groups[0].extruder_id = EXTRUDER_ID_UNKNOWN;
    plan.groups[0].layers[1].tool_groups.emplace_back();
    plan.groups[0].layers[1].tool_groups[1].extruder_id = 0;
    plan.groups[0].layers[1].tool_groups.emplace_back();
    plan.groups[0].layers[1].tool_groups[2].extruder_id = 1;
    plan.groups.emplace_back();
    plan.groups[1].layers.emplace_back();
    plan.groups[1].layers[0].print_z = scale_i(0.6);

    GCodeScriptProcessor host_processor(print, Orchestrator::instance());
    const slic3r_api::GCodeGeneration::GCodeScriptProcessorView scripts(
        host_processor.c_processor());
    const slic3r_api::GCodeGeneration::GCodeScriptContext context = scripts.prepare(
        registered_script_type(Orchestrator::instance(), "unknown_script"));

    CHECK(context.process(
              "T[initial_tool] E[initial_extruder] F[initial_filament_type] "
              "C[total_toolchanges] A[all_layer_count] L[total_layer_count] N[num_extruders]",
              0) == "T1 E1 FPETG C2 A3 L0 N2");

    const slic3r_api::GCodeGeneration::GCodeScriptContext used_context = scripts.prepare(
        registered_script_type(Orchestrator::instance(), "unknown_script"));
    CHECK(used_context.process("{if is_extruder_used[0]}1{else}0{endif}", 0) == "1");
    const slic3r_api::GCodeGeneration::GCodeScriptContext used_one_context = scripts.prepare(
        registered_script_type(Orchestrator::instance(), "unknown_script"));
    CHECK(used_one_context.process("{if is_extruder_used[1]}1{else}0{endif}", 0) == "1");
    const slic3r_api::GCodeGeneration::GCodeScriptContext used_two_context = scripts.prepare(
        registered_script_type(Orchestrator::instance(), "unknown_script"));
    CHECK(used_two_context.process("{if is_extruder_used[2]}1{else}0{endif}", 0) == "0");
    const slic3r_api::GCodeGeneration::GCodeScriptContext used_254_context = scripts.prepare(
        registered_script_type(Orchestrator::instance(), "unknown_script"));
    CHECK(used_254_context.process("{if is_extruder_used[254]}1{else}0{endif}", 0) == "0");

    const slic3r_api::GCodeGeneration::GCodeScriptContext bed_context = scripts.prepare(
        registered_script_type(Orchestrator::instance(), "unknown_script"));
    CHECK(bed_context.process(
              "{print_bed_min[0]} {print_bed_min[1]} {print_bed_max[0]} "
              "{print_bed_max[1]} {print_bed_size[0]} {print_bed_size[1]}", 0) ==
          "-10 -5 190 215 200 220");
    const slic3r_api::GCodeGeneration::GCodeScriptContext colour_context = scripts.prepare(
        registered_script_type(Orchestrator::instance(), "unknown_script"));
    CHECK(colour_context.process(
              "{thumbnails_color_int} {filament_colour_int[0]} {filament_colour_int[1]} "
              "{extruder_colour_int[0]} {extruder_colour_int[1]}", 0) ==
          "1056816 16711680 65280 0 255");

    const slic3r_api::GCodeGeneration::GCodeScriptContext dimension_context = scripts.prepare(
        registered_script_type(Orchestrator::instance(), "unknown_script"));
    const std::string dimension_output = dimension_context.process(
        "{bounding_box[0]} {bounding_box[1]} {bounding_box[2]} {bounding_box[3]} "
        "{bounding_box[4]} {bounding_box[5]} {first_layer_print_min[0]} "
        "{first_layer_print_min[1]} {first_layer_print_max[0]} {first_layer_print_max[1]} "
        "{first_layer_print_size[0]} {first_layer_print_size[1]}", 0);
    std::istringstream dimension_stream(dimension_output);
    std::vector<double> dimension_values;
    double dimension_value = 0.0;
    while (dimension_stream >> dimension_value)
        dimension_values.push_back(dimension_value);
    REQUIRE(dimension_values.size() == 12);
    for (size_t idx = 0; idx < 6; ++idx)
        CHECK(dimension_values[idx] == Approx(0.0).margin(1e-6));

    const Polygon hull = print.first_layer_convex_hull_with_wipe_towers();
    std::vector<Vec2d> hull_points;
    for (const Point &point : hull)
        hull_points.emplace_back(unscaled(point.x()), unscaled(point.y()));
    if (hull_points.empty())
        hull_points.emplace_back(Vec2d::Zero());
    const BoundingBoxf hull_box(hull_points);
    const Vec2d expected_hull_min = hull_box.min;
    const Vec2d expected_hull_max = hull_box.max;
    CHECK(dimension_values[6] == Approx(expected_hull_min.x()).margin(1e-6));
    CHECK(dimension_values[7] == Approx(expected_hull_min.y()).margin(1e-6));
    CHECK(dimension_values[8] == Approx(expected_hull_max.x()).margin(1e-6));
    CHECK(dimension_values[9] == Approx(expected_hull_max.y()).margin(1e-6));
    CHECK(dimension_values[10] == Approx(expected_hull_max.x() - expected_hull_min.x()).margin(1e-6));
    CHECK(dimension_values[11] == Approx(expected_hull_max.y() - expected_hull_min.y()).margin(1e-6));

    const slic3r_api::GCodeGeneration::GCodeScriptContext execution_context = scripts.prepare(
        registered_script_type(Orchestrator::instance(), "unknown_script"));
    CHECK(execution_context.config().get_int("current_object_idx") == 0);
    CHECK(execution_context.config().get_int("layer_num") == -1);
    CHECK(execution_context.config().get_float("layer_z") == Approx(0.0).margin(1e-6));
    CHECK(execution_context.config().get_float("layer_current_height") == Approx(0.0).margin(1e-6));
    CHECK(execution_context.config().get_float("zhop") == Approx(0.0).margin(1e-6));
    CHECK(execution_context.config().get_float("extruded_volume_total") == Approx(0.0).margin(1e-6));
    CHECK(execution_context.config().get_float("extruded_weight_total") == Approx(0.0).margin(1e-6));
    CHECK_FALSE(execution_context.config().has("layer_height"));
    CHECK(execution_context.process("[layer_height]", 0) == "0.25");
    const std::vector<double> execution_volumes = execution_context.config().get_floats("extruded_volume");
    const std::vector<double> execution_weights = execution_context.config().get_floats("extruded_weight");
    REQUIRE(execution_volumes.size() == 2);
    REQUIRE(execution_weights.size() == 2);
    CHECK(execution_volumes[0] == Approx(0.0).margin(1e-6));
    CHECK(execution_volumes[1] == Approx(0.0).margin(1e-6));
    CHECK(execution_weights[0] == Approx(0.0).margin(1e-6));
    CHECK(execution_weights[1] == Approx(0.0).margin(1e-6));

    const slic3r_api::GCodeGeneration::GCodeScriptContext feature_context = scripts.prepare(
        GCODE_SCRIPT_TYPE_FEATURE_GCODE);
    CHECK(feature_context.config().get_float("max_layer_z") == Approx(0.6).margin(1e-6));
}
TEST_CASE("Firmware scripts import validated machine state atomically",
          "[plugins][gcode][firmware][custom-gcode]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    configure_standard_firmware(print, 2);
    Orchestrator &orchestrator = Orchestrator::instance();
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    plan.groups.front().layers.front().tool_groups.emplace_back();
    plan.groups.front().layers.front().tool_groups.front().extruder_id = 1;
    GCodeScriptProcessor host_processor(print, orchestrator);
    const slic3r_api::GCodeGeneration::GCodeScriptProcessorView scripts(
        host_processor.c_processor());
    ScriptStateFirmwareProbe firmware(scripts);
    const slic3r_api::Print print_view(
        reinterpret_cast<const print_handle *>(&print));
    firmware.setup(print_view.config());

    CHECK(firmware.process_script(
              GCODE_SCRIPT_TYPE_START_GCODE, "M104 S175", nullptr, uint16_t(1)) ==
          "M104 S175\n");
    REQUIRE(firmware.machine_extruder(1).heater().requested_temperature());
    CHECK(*firmware.machine_extruder(1).heater().requested_temperature() == 175);
    CHECK_FALSE(firmware.machine_extruder(0).heater().requested_temperature());

    // When the script itself leaves position untouched, the host derives the
    // new machine position from the successfully produced movement commands.
    CHECK(firmware.process_script(GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM, "G1 X12 Y34 Z5") == "G1 X12 Y34 Z5\n");
    REQUIRE(firmware.machine_gantry().position());
    CHECK(firmware.machine_gantry().position()->x == Approx(12.0));
    CHECK(firmware.machine_gantry().position()->y == Approx(34.0));
    CHECK(firmware.machine_gantry().position()->z == Approx(5.0));

    CHECK(
        firmware.process_script(GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM, "{position[0]=8}{position[1]=9}{position[2]=10}")
            .empty());
    REQUIRE(firmware.machine_gantry().position());
    CHECK(firmware.machine_gantry().position()->x == Approx(8.0));
    CHECK(firmware.machine_gantry().position()->y == Approx(9.0));
    CHECK(firmware.machine_gantry().position()->z == Approx(10.0));

    slic3r_api::GCodeGeneration::ExtrusionAxisState &axis =
        firmware.machine_extruder(0).extrusion_axis();
    CHECK(firmware.process_script(
              GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM,
              "M82\nG1 E5\nM83\nG1 E2\nM82\nG1 E8") ==
          "M82\nG1 E5\nM83\nG1 E2\nM82\nG1 E8\n");
    CHECK(axis.position() == Approx(8.0));
    CHECK(axis.used_filament() == Approx(8.0));
    CHECK_FALSE(axis.extrude(0.000004));
    REQUIRE(axis.extruded_dE_left() == Approx(0.000004));
    CHECK(firmware.process_script(GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM, "M117 unchanged").find("M117 unchanged") !=
          std::string::npos);
    CHECK(axis.extruded_dE_left() == Approx(0.000004));

    CHECK(firmware
              .process_script(GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM,
                              "{e_position[0]=4}{e_retracted[0]=1.5}{e_restart_extra[0]=0.2}")
              .empty());
    CHECK(axis.position() == Approx(4.0));

    CHECK(firmware.process_script(
              GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM,
              "G91\nG1 X1 Y2 Z3\nG90") ==
          "G91\nG1 X1 Y2 Z3\nG90\n");
    REQUIRE(firmware.machine_gantry().position());
    CHECK(firmware.machine_gantry().position()->x == Approx(9.0));
    CHECK(firmware.machine_gantry().position()->y == Approx(11.0));
    CHECK(firmware.machine_gantry().position()->z == Approx(13.0));

    CHECK(firmware.process_script(
              GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM,
              "G20\nG91\nG1 X1\nG90\nG21") ==
          "G20\nG91\nG1 X1\nG90\nG21\n");
    REQUIRE(firmware.machine_gantry().position());
    CHECK(firmware.machine_gantry().position()->x == Approx(34.4));

    CHECK(firmware.process_script(
              GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM,
              "T1\nM104 S205\nM140 S60\nM141 S35\nM106 S128") ==
          "T1\nM104 S205\nM140 S60\nM141 S35\nM106 S128\n");
    const slic3r_api::GCodeGeneration::DefaultExtruder &tool_one = firmware.machine_extruder(1);
    REQUIRE(tool_one.heater().requested_temperature());
    CHECK(*tool_one.heater().requested_temperature() == 205);
    REQUIRE(tool_one.fan().encoded_speed_percent());
    CHECK(*tool_one.fan().encoded_speed_percent() == Approx(128.0 * 100.0 / 255.0));
    REQUIRE(firmware.machine_printer().bed_heater().requested_temperature());
    CHECK(*firmware.machine_printer().bed_heater().requested_temperature() == 60);
    REQUIRE(firmware.machine_printer().chamber_heater().requested_temperature());
    CHECK(*firmware.machine_printer().chamber_heater().requested_temperature() == 35);

    CHECK(firmware.process_script(
              GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM,
              "M83\nG1 E2\nM82\nG92 E5\nG1 E6\nM107") ==
          "M83\nG1 E2\nM82\nG92 E5\nG1 E6\nM107\n");
    CHECK(firmware.machine_extruder(1).extrusion_axis().position() == Approx(6.0));
    CHECK(firmware.machine_extruder(1).extrusion_axis().used_filament() == Approx(3.0));
    REQUIRE(firmware.machine_extruder(1).fan().encoded_speed_percent());
    CHECK(*firmware.machine_extruder(1).fan().encoded_speed_percent() == Approx(0.0));

    CHECK(firmware.process_script(
              GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM,
              "{position[0]=50}G91\nG1 X1") ==
          "G91\nG1 X1\n");
    REQUIRE(firmware.machine_gantry().position());
    CHECK(firmware.machine_gantry().position()->x == Approx(50.0));
    CHECK(firmware.process_script(
              GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM,
              "M117 one newline\n\n") ==
          "M117 one newline\n");
    CHECK(axis.retracted() == Approx(1.5));
    CHECK(axis.restart_extra() == Approx(0.2));
    CHECK(axis.extruded_dE_left() == Approx(0.0));

    // Corrupting a prepared in/out vector simulates a hostile script or C
    // caller. Validation fails before any session state can be imported.
    const slic3r_api::GCodeGeneration::GCodeScriptContext invalid = scripts.prepare(
        GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM);
    DynamicConfig *prepared = dynamic_cast<DynamicConfig *>(
        ApiHost::to_config(invalid.config().handle()));
    REQUIRE(prepared != nullptr);
    prepared->set_key_value("position", new ConfigOptionFloats({1.0, 2.0}));
    CHECK_THROWS(invalid.process("", 0));
    REQUIRE(firmware.machine_gantry().position());
    CHECK(firmware.machine_gantry().position()->x == Approx(50.0));
    CHECK(axis.position() == Approx(4.0));

    ScriptStateFirmwareProbe processorless;
    CHECK_THROWS_AS(processorless.process_script(GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM, "M117 unavailable"),
                    std::invalid_argument);
    CHECK_THROWS_AS(
        firmware.process_script(GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM, "M117 invalid target", nullptr, uint16_t(2)),
        std::invalid_argument);
}
TEST_CASE("Derived firmware completes a script context immediately before parsing",
          "[plugins][gcode][firmware][custom-gcode]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    configure_standard_firmware(print);
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    plan.groups.front().layers.front().print_z = scale_i(0.4);
    plan.groups.front().layers.front().tool_groups.emplace_back();
    plan.groups.front().layers.front().tool_groups.front().extruder_id = 0;

    GCodeScriptProcessor host_processor(print, Orchestrator::instance());
    const slic3r_api::GCodeGeneration::GCodeScriptProcessorView scripts(
        host_processor.c_processor());
    ScriptContextCompletingFirmwareProbe firmware(scripts);
    const slic3r_api::Print print_view(
        reinterpret_cast<const print_handle *>(&print));

    firmware.setup(print_view.config());
    CHECK(firmware.process_script(
              GCODE_SCRIPT_TYPE_START_GCODE, "M117 Z[max_layer_z]") ==
          "M117 Z12.5\n");
    CHECK(firmware.completion_count() == 1);
}
// Script-owned selection keeps its stored tool context while layer entry emits Z and maximum height; selection-time output stays suppressed and E resets after the script.
TEST_CASE("Toolchange scripts use the structural Config stored at insertion",
          "[plugins][gcode][firmware][custom-gcode][printing-plan]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    configure_standard_firmware(print, 2);
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    PrintingGroup &group = plan.groups.front();
    group.layers.emplace_back();
    PrintingLayerGroup &layer = group.layers.front();
    layer.print_z = scale_i(0.2);
    layer.tool_groups.emplace_back();
    layer.tool_groups.back().extruder_id = 0;
    layer.tool_groups.emplace_back();
    layer.tool_groups.back().extruder_id = 1;

    layer.tool_groups[0].extruder_id = 1;
    layer.tool_groups[1].extruder_id = 0;
    // The producer freezes only structural facts. The firmware will still add
    // position, E and retraction from the exact execution point.
    PluginStorage script_storage;
    storage_handle *script_storage_handle =
        reinterpret_cast<storage_handle *>(&script_storage);
    ExtrusionNop toolchange_script;
    slic3r_api::MutableExtrusionEntity toolchange_view(
        reinterpret_cast<extrusion_entity_handle *>(&toolchange_script));
    slic3r_api::StoredConfig toolchange_arguments(script_storage_handle);
    toolchange_arguments.get_or_add("previous_extruder", SLIC3R_CONFIG_OPTION_INT).set_int(1);
    toolchange_arguments.get_or_add("next_extruder", SLIC3R_CONFIG_OPTION_INT).set_int(0);
    toolchange_arguments.get_or_add("toolchange_z", SLIC3R_CONFIG_OPTION_FLOAT).set_float(0.2);
    toolchange_view.script_gcode(
        "; FINAL_TOOL P[previous_extruder] N[next_extruder]",
        GCODE_SCRIPT_TYPE_TOOLCHANGE_GCODE,
        toolchange_arguments,
        uint16_t(0));
    layer.tool_groups[1].events.append_before(toolchange_script);

    GCodeScriptProcessor host_processor(print, Orchestrator::instance());
    ScriptStateFirmwareProbe firmware(
        slic3r_api::GCodeGeneration::GCodeScriptProcessorView(
            host_processor.c_processor()),
        script_storage_handle);
    const slic3r_api::Print print_view(
        reinterpret_cast<const print_handle *>(&print));
    slic3r_api::PrintingPlan plan_view = printing_plan_view(print);
    slic3r_api::PrintingGroup group_view = plan_view.group(0);
    slic3r_api::PrintingLayerGroup layer_view = group_view.layer_group(0);
    slic3r_api::PrintingToolGroup first_tool = layer_view.tool_group(0);
    slic3r_api::PrintingToolGroup second_tool = layer_view.tool_group(1);

    firmware.setup(print_view.config());
    REQUIRE(firmware.begin_group(group_view).empty());
    REQUIRE(firmware.begin_layer(layer_view) ==
            ";LAYER_CHANGE\n;Z:0.2\n;MAX_LAYER_HEIGHT:0.2\n");
    CHECK(firmware.begin_tool_group(first_tool) == "T1\nG92 E0\n");
    REQUIRE(firmware.end_tool_group().empty());
    // Suppressed selection leaves the coordinate alone: the script owns the
    // real tool command, so no host-generated G92 may be inserted here.
    firmware.machine_extruder(0).extrusion_axis().set_position(8.0);
    REQUIRE(firmware.begin_tool_group(second_tool).empty());
    CHECK(firmware.machine_extruder(0).extrusion_axis().position() == 8.0);
    CHECK(firmware.write_event(second_tool.events().before()) ==
          "; FINAL_TOOL P1 N0\nG92 E0\n");
    CHECK(firmware.machine_extruder(0).extrusion_axis().position() == 0.0);
}

// Script-owned T commands always establish a post-script absolute origin without repaying debt;
// both static and geometric E preparation then start from that origin.
TEST_CASE("Toolchange scripts reset E before static and geometric extrusion without consuming retraction",
          "[plugins][gcode][firmware][custom-gcode][toolchange-reset]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool relative_e = GENERATE(false, true);
    const bool script_reset = GENERATE(false, true);
    const bool semm = GENERATE(false, true);
    const int movement = GENERATE(0, 1, 2, 3);
    CAPTURE(relative_e, script_reset, semm, movement);
    Print print;
    configure_standard_firmware(print, 2);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.use_relative_e_distances.value = relative_e;
    config.single_extruder_multi_material.value = semm;
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.front().layers.front();
    layer.tool_groups.emplace_back();
    layer.tool_groups.back().extruder_id = 0;
    layer.tool_groups.emplace_back();
    layer.tool_groups.back().extruder_id = 1;
    ExtrusionNop script;
    slic3r_api::MutableExtrusionEntity script_view(reinterpret_cast<extrusion_entity_handle *>(&script));
    script_view.script_gcode(std::string("T1\n") + (script_reset ? "G92 E0\n" : "") + "; SCRIPT_DONE\n",
                            GCODE_SCRIPT_TYPE_TOOLCHANGE_GCODE);
    layer.tool_groups.back().events.append_before(script);
    GCodeScriptProcessor processor(print, Orchestrator::instance());
    ScriptStateFirmwareProbe firmware(slic3r_api::GCodeGeneration::GCodeScriptProcessorView(processor.c_processor()));
    firmware.set_gcode_reader_context(reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()),
                                     reinterpret_cast<const print_handle *>(&print));
    firmware.setup(slic3r_api::Print(reinterpret_cast<const print_handle *>(&print)).config());
    const slic3r_api::ConstPrintingToolGroup first(reinterpret_cast<const printing_tool_group_handle *>(&layer.tool_groups[0]));
    const slic3r_api::ConstPrintingToolGroup next(reinterpret_cast<const printing_tool_group_handle *>(&layer.tool_groups[1]));
    firmware.begin_tool_group(first);
    for (size_t idx = 0; idx < 2; ++idx) {
        firmware.machine_extruder(idx).extrusion_axis().set_position(8.0);
        firmware.machine_extruder(idx).extrusion_axis().set_retracted(2.0, 0.2);
        firmware.machine_extruder(idx).extrusion_axis().schedule_restart_extra(0.3, true);
    }
    REQUIRE(firmware.begin_tool_group(next).empty());
    slic3r_api::GCodeGeneration::ExtrusionAxisState &axis = firmware.machine_extruder(1).extrusion_axis();
    const double used_before = axis.used_filament();
    const std::string script_output = firmware.write_event(next.events().before());
    INFO(script_output);
    CHECK(script_output.find("T1\n") == 0);
    if (!relative_e) {
        const size_t script_end = script_output.find("; SCRIPT_DONE\n");
        const size_t reset = script_output.find("G92 E0\n", script_end);
        REQUIRE(script_end != std::string::npos);
        REQUIRE(reset != std::string::npos);
        CHECK(script_end < reset);
    }
    CHECK(axis.position() == 0.0);
    CHECK(axis.retracted() == 2.0);
    CHECK(axis.restart_extra() == Approx(0.2));
    CHECK(axis.used_filament() == used_before);
    CHECK(count_occurrences(script_output, "G92 E0\n") == size_t(script_reset) + size_t(!relative_e));

    std::unique_ptr<ExtrusionEntity> move;
    if (movement == 0) {
        move = special_command_entity(ExtrusionPropertySpecialCommand::Code::EXTRUSION, 1.25);
    } else if (movement == 2) {
        move.reset(new ExtrusionNop());
    } else {
        move = make_firmware_travel(ArcPolyline(Points{Point::new_scale(0.0, 0.0), Point::new_scale(1.0, 0.0)}),
                                    20.f, 500.f);
    }
    slic3r_api::MutableExtrusionEntity move_view(reinterpret_cast<extrusion_entity_handle *>(move.get()));
    move_view.get_or_add(slic3r_api::EPropertySpeed::key).speed(20.f);
    if (movement == 1) {
        move_view.get_or_add(slic3r_api::EPropertySpecialCommand::key).set(C_EXTRUSION_SPECIAL_COMMAND_EXTRUSION, 1.25);
    } else if (movement >= 2) {
        move_view.get_or_add(slic3r_api::EPropertyAttributes::key).extrusion_role(movement == 2 ?
            RAW_EXTRUSION_ROLE_UNRETRACT : RAW_EXTRUSION_ROLE_TRAVEL | RAW_EXTRUSION_ROLE_WIPE | RAW_EXTRUSION_ROLE_UNRETRACT);
        move_view.get_or_add(slic3r_api::EPropertyExtrusionAxis::key).unretract(0.2);
    }
    const std::string move_output = firmware.write_event(slic3r_api::ExtrusionEntity(
        reinterpret_cast<const extrusion_entity_handle *>(move.get())));
    INFO(move_output);
    const std::string expected_e = movement >= 2 ? " E2.5" : " E1.25";
    CHECK(move_output.find(expected_e) != std::string::npos);
    CHECK(move_output.find("G92") == std::string::npos);
    CHECK(axis.position() == Approx(relative_e ? 0.0 : (movement >= 2 ? 2.5 : 1.25)));
    // Static EXTRUSION is raw and keeps the retraction; only the explicit
    // UNRETRACT events restore it.
    CHECK(axis.retracted() == Approx(movement >= 2 ? 0.0 : 2.0));
    if (movement >= 2)
        CHECK_FALSE(axis.need_unretract());
    std::unique_ptr<ExtrusionEntity> second = special_command_entity(ExtrusionPropertySpecialCommand::Code::EXTRUSION, 0.5);
    second->add_property(ExtrusionPropertySpeed(20.f, 500.f));
    const std::string second_output = firmware.write_event(slic3r_api::ExtrusionEntity(
        reinterpret_cast<const extrusion_entity_handle *>(second.get())));
    CHECK(second_output.find(relative_e ? " E.5" : (movement >= 2 ? " E3" : " E1.75")) != std::string::npos);
    CHECK(second_output.find("G92") == std::string::npos);
}

// Custom G-code declares its retraction on the same line: a negative E move
// retracts only with "; retraction", a positive E move restores only with
// "; unretraction", and the comment ignores case and surrounding spaces.
TEST_CASE("Custom G-code declares retraction on the same line",
          "[plugins][gcode][firmware][custom-gcode][retraction]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool relative_e = GENERATE(false, true);
    Print print;
    configure_standard_firmware(print, 2);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.use_relative_e_distances.value = relative_e;
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    plan.groups.front().layers.front().tool_groups.emplace_back();
    plan.groups.front().layers.front().tool_groups.front().extruder_id = 0;
    GCodeScriptProcessor processor(print, Orchestrator::instance());
    ScriptStateFirmwareProbe firmware(slic3r_api::GCodeGeneration::GCodeScriptProcessorView(processor.c_processor()));
    firmware.setup(slic3r_api::Print(reinterpret_cast<const print_handle *>(&print)).config());
    slic3r_api::GCodeGeneration::ExtrusionAxisState &axis = firmware.machine_extruder(0).extrusion_axis();

    // Build one move with an explicit filament delta and an optional comment.
    double position = 0.0;
    const auto move = [&](const double delta, const std::string &comment = std::string()) {
        position += delta;
        const std::string text = "G1 E" + std::to_string(relative_e ? delta : position) + comment;
        return firmware.process_script(GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM, text);
    };
    // The first script establishes the addressing mode for the following moves.
    CHECK_FALSE(firmware.process_script(GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM,
        relative_e ? "M83" : "M82").empty());

    CHECK_FALSE(move(-10.0).empty());
    CHECK(axis.retracted() == 0.0);

    CHECK_FALSE(move(-10.0, " ; retraction").empty());
    CHECK(axis.retracted() == Approx(10.0));

    // A positive move without its own declaration does not repay.
    CHECK_FALSE(move(4.0).empty());
    CHECK(axis.retracted() == Approx(10.0));

    // Case and surrounding spaces are ignored.
    CHECK_FALSE(move(4.0, " ;Unretraction  ").empty());
    CHECK(axis.retracted() == Approx(6.0));

    CHECK_FALSE(move(-2.0, " ;  RETRACTION  ").empty());
    CHECK(axis.retracted() == Approx(8.0));

    // A comment on another line does not declare the move.
    position -= 1.0;
    CHECK_FALSE(firmware.process_script(GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM,
        "; retraction\nG1 E" + std::to_string(relative_e ? -1.0 : position)).empty());
    CHECK(axis.retracted() == Approx(8.0));

    // A declaration with the wrong sign leaves the state alone.
    CHECK_FALSE(move(1.0, " ; retraction").empty());
    CHECK(axis.retracted() == Approx(8.0));
    CHECK_FALSE(move(-1.0, " ; unretraction").empty());
    CHECK(axis.retracted() == Approx(8.0));
}

TEST_CASE("G-code script types have stable built-in and runtime identities", "[plugins][gcode][script-type]") {
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Orchestrator &orchestrator = Orchestrator::instance();
    orchestrator_handle *handle = reinterpret_cast<orchestrator_handle *>(&orchestrator);

    const std::array<std::pair<gcode_script_type, const char *>, 13> builtins = {{
        {GCODE_SCRIPT_TYPE_START_GCODE, "start_gcode"},
        {GCODE_SCRIPT_TYPE_END_GCODE, "end_gcode"},
        {GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM, "extrusion_custom_gcode_script"},
        {GCODE_SCRIPT_TYPE_START_FILAMENT_GCODE, "start_filament_gcode"},
        {GCODE_SCRIPT_TYPE_END_FILAMENT_GCODE, "end_filament_gcode"},
        {GCODE_SCRIPT_TYPE_BEFORE_LAYER_GCODE, "before_layer_gcode"},
        {GCODE_SCRIPT_TYPE_LAYER_GCODE, "layer_gcode"},
        {GCODE_SCRIPT_TYPE_TOOLCHANGE_GCODE, "toolchange_gcode"},
        {GCODE_SCRIPT_TYPE_BETWEEN_OBJECTS_GCODE, "between_objects_gcode"},
        {GCODE_SCRIPT_TYPE_FEATURE_GCODE, "feature_gcode"},
        {GCODE_SCRIPT_TYPE_COLOR_CHANGE_GCODE, "color_change_gcode"},
        {GCODE_SCRIPT_TYPE_PAUSE_PRINT_GCODE, "pause_print_gcode"},
        {GCODE_SCRIPT_TYPE_TEMPLATE_CUSTOM_GCODE, "template_custom_gcode"},
    }};
    for (const std::pair<gcode_script_type, const char *> &builtin : builtins) {
        CHECK(orchestrator_register_runtime_id(handle, GCODE_SCRIPT_ID_TYPE, builtin.second) == builtin.first);
        const char *resolved = orchestrator_runtime_id_name(handle, GCODE_SCRIPT_ID_TYPE, builtin.first);
        REQUIRE(resolved != nullptr);
        CHECK(std::string(resolved) == builtin.second);
    }

    const orchestrator_runtime_id first_id = orchestrator_register_runtime_id(
        handle, GCODE_SCRIPT_ID_TYPE, "test.script_type.first");
    const orchestrator_runtime_id repeated_id = orchestrator_register_runtime_id(
        handle, GCODE_SCRIPT_ID_TYPE, "test.script_type.first");
    const orchestrator_runtime_id second_id = orchestrator_register_runtime_id(
        handle, GCODE_SCRIPT_ID_TYPE, "test.script_type.second");
    const gcode_script_type first = static_cast<gcode_script_type>(first_id);
    REQUIRE(first >= GCODE_SCRIPT_TYPE_CUSTOM_BEGIN);
    CHECK(repeated_id == first_id);
    CHECK(second_id != first_id);
    const char *first_name = orchestrator_runtime_id_name(
        handle, GCODE_SCRIPT_ID_TYPE, first_id);
    REQUIRE(first_name != nullptr);
    CHECK(std::string(first_name) == "test.script_type.first");

    const orchestrator_runtime_id reader_id = orchestrator_register_runtime_id(
        handle, GCODE_READER_STATE_ID_TYPE, "test.script_type.first");
    REQUIRE(reader_id != ORCHESTRATOR_RUNTIME_ID_INVALID);
    const char *reader_name = orchestrator_runtime_id_name(
        handle, GCODE_READER_STATE_ID_TYPE, reader_id);
    REQUIRE(reader_name != nullptr);
    CHECK(std::string(reader_name) == "test.script_type.first");
    CHECK(std::string(first_name) == "test.script_type.first");

    CHECK(orchestrator_register_runtime_id(handle, GCODE_SCRIPT_ID_TYPE, "") ==
          ORCHESTRATOR_RUNTIME_ID_INVALID);
    CHECK(orchestrator_register_runtime_id(nullptr, GCODE_SCRIPT_ID_TYPE, "test.invalid") ==
          ORCHESTRATOR_RUNTIME_ID_INVALID);
    CHECK(orchestrator_runtime_id_name(handle, GCODE_SCRIPT_ID_TYPE,
                                       GCODE_SCRIPT_TYPE_INVALID) == nullptr);
    CHECK(orchestrator_runtime_id_name(handle, GCODE_SCRIPT_ID_TYPE,
                                       GCODE_SCRIPT_TYPE_CUSTOM_BEGIN - 1) == nullptr);
}
TEST_CASE("G-code script type survives entity copies moves and diagnostics",
          "[plugins][gcode][script-type][diagnostic]") {
    const gcode_script_type script_type = GCODE_SCRIPT_TYPE_CUSTOM_BEGIN + 42;
    ExtrusionNop source(
        ExtrusionPropertyCustomGcodeText(ExtrusionPropertyCustomGcodeText::Code::SCRIPT, "M117 typed", script_type));

    std::unique_ptr<ExtrusionEntity> copied(source.clone());
    const ExtrusionPropertyCustomGcode *copied_property = copied->get_property<ExtrusionPropertyCustomGcode>();
    REQUIRE(copied_property != nullptr);
    CHECK(copied_property->script_type == script_type);

    ExtrusionNop moved(std::move(source));
    const ExtrusionPropertyCustomGcode *moved_property = moved.get_property<ExtrusionPropertyCustomGcode>();
    REQUIRE(moved_property != nullptr);
    CHECK(moved_property->script_type == script_type);

    ExtrusionPrinter printer(/*mult=*/1.0, /*trunc=*/0, /*json=*/true);
    printer.traverse(moved);
    CHECK(printer.str().find("\"script_type\":2147483690") != std::string::npos);
}
TEST_CASE("G-code script processing tool survives stored entity copies moves and diagnostics",
          "[plugins][gcode][script-type][diagnostic]")
{
    PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);

    slic3r_api::StoredConfig arguments(storage);
    arguments.get_or_add("filament_extruder_id", SLIC3R_CONFIG_OPTION_INT).set_int(2);
    arguments.get_or_add("producer_note", SLIC3R_CONFIG_OPTION_STRING).set_string("owned with event");
    slic3r_api::StoredExtrusionEntity source(storage);
    const slic3r_api::EPropertyCustomGcode &source_property =
        source.script_gcode(
            "M117 stored", GCODE_SCRIPT_TYPE_END_FILAMENT_GCODE, arguments, uint16_t(2));
    CHECK(source_property.processing_extruder_id == 2);
    REQUIRE(source_property.config_id != EXTRUSION_DATA_ID_INVALID);
    slic3r_api::StoredConfig source_config(storage);
    source_config.deserialize_all(source.stored_string(source_property.config_id));
    CHECK(source_config.keys().size() == 2);

    slic3r_api::StoredExtrusionEntity copied(storage, source.readonly());
    const slic3r_api::EPropertyCustomGcode *copied_property =
        copied.get(slic3r_api::EPropertyCustomGcode::key);
    REQUIRE(copied_property != nullptr);
    CHECK(copied_property->processing_extruder_id == 2);
    REQUIRE(copied_property->config_id != EXTRUSION_DATA_ID_INVALID);
    slic3r_api::StoredConfig copied_config(storage);
    copied_config.deserialize_all(copied.stored_string(copied_property->config_id));
    CHECK(copied_config.get("producer_note").get_string() == "owned with event");
    copied.custom_gcode("M117 raw", C_EXTRUSION_CUSTOM_GCODE_GCODE);
    copied_property = copied.get(slic3r_api::EPropertyCustomGcode::key);
    REQUIRE(copied_property != nullptr);
    CHECK(copied_property->processing_extruder_id == EXTRUDER_ID_UNKNOWN);
    CHECK(copied_property->config_id == EXTRUSION_DATA_ID_INVALID);

    slic3r_api::StoredExtrusionEntity moved(std::move(source));
    const slic3r_api::EPropertyCustomGcode *moved_property =
        moved.get(slic3r_api::EPropertyCustomGcode::key);
    REQUIRE(moved_property != nullptr);
    CHECK(moved_property->processing_extruder_id == 2);
    REQUIRE(moved_property->config_id != EXTRUSION_DATA_ID_INVALID);
    slic3r_api::StoredConfig moved_config(storage);
    moved_config.deserialize_all(moved.stored_string(moved_property->config_id));
    CHECK(moved_config.get("filament_extruder_id").get_int() == 2);

    const ExtrusionEntity &host_entity =
        *reinterpret_cast<const ExtrusionEntity *>(moved.handle());
    ExtrusionPrinter printer(/*mult=*/1.0, /*trunc=*/0, /*json=*/true);
    printer.traverse(host_entity);
    CHECK(printer.str().find("\"processing_extruder_id\":2") != std::string::npos);
    CHECK(printer.str().find("\"config_key_count\":2") != std::string::npos);
}
TEST_CASE("G-code script Config snapshots replace and clear their owned data",
          "[plugins][gcode][firmware][custom-gcode][config]")
{
    PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    slic3r_api::StoredExtrusionEntity event(
        storage);
    slic3r_api::StoredConfig initial_config(storage);
    initial_config.get_or_add("layer_num", SLIC3R_CONFIG_OPTION_INT).set_int(7);
    const slic3r_api::EPropertyCustomGcode &property = event.script_gcode(
        "M117 [layer_num]", GCODE_SCRIPT_TYPE_LAYER_GCODE, initial_config);
    const extrusion_data_id initial_id = property.config_id;
    REQUIRE(initial_id != EXTRUSION_DATA_ID_INVALID);

    slic3r_api::StoredConfig replacement_config(storage);
    replacement_config.get_or_add("layer_num", SLIC3R_CONFIG_OPTION_INT).set_int(9);
    replacement_config.get_or_add("layer_z", SLIC3R_CONFIG_OPTION_FLOAT).set_float(0.4);
    event.script_gcode(
        "M117 [layer_num] [layer_z]", GCODE_SCRIPT_TYPE_LAYER_GCODE, replacement_config);

    // Publishing the replacement snapshot releases the buffer previously
    // owned by the same config_id field.
    const slic3r_api::EPropertyCustomGcode *replaced =
        event.get(slic3r_api::EPropertyCustomGcode::key);
    REQUIRE(replaced != nullptr);
    CHECK(replaced->config_id != EXTRUSION_DATA_ID_INVALID);
    CHECK(event.stored_data(initial_id) == nullptr);
    slic3r_api::StoredConfig decoded_replacement(storage);
    decoded_replacement.deserialize_all(event.stored_string(replaced->config_id));
    CHECK(decoded_replacement.get("layer_num").get_int() == 9);
    CHECK(decoded_replacement.get("layer_z").get_float() == Approx(0.4));

    slic3r_api::StoredConfig empty_config(storage);
    event.script_gcode("M117 empty", GCODE_SCRIPT_TYPE_LAYER_GCODE, empty_config);
    replaced = event.get(slic3r_api::EPropertyCustomGcode::key);
    REQUIRE(replaced != nullptr);
    REQUIRE(replaced->config_id != EXTRUSION_DATA_ID_INVALID);
    slic3r_api::StoredConfig decoded_empty(storage);
    decoded_empty.deserialize_all(event.stored_string(replaced->config_id));
    CHECK(decoded_empty.keys().empty());

    event.script_gcode("M117 plain", GCODE_SCRIPT_TYPE_LAYER_GCODE);
    replaced = event.get(slic3r_api::EPropertyCustomGcode::key);
    REQUIRE(replaced != nullptr);
    CHECK(replaced->config_id == EXTRUSION_DATA_ID_INVALID);
}
TEST_CASE("Firmware rejects invalid script Config snapshots and clears its scratch Config",
          "[plugins][gcode][firmware][custom-gcode][config]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    configure_standard_firmware(print);
    GCodeScriptProcessor host_processor(print, Orchestrator::instance());
    const slic3r_api::GCodeGeneration::GCodeScriptProcessorView scripts(
        host_processor.c_processor());
    PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    ScriptStateFirmwareProbe firmware(scripts, storage);
    const slic3r_api::Print print_view(
        reinterpret_cast<const print_handle *>(&print));
    firmware.setup(print_view.config());

    slic3r_api::StoredExtrusionEntity invalid_id_event(storage);
    slic3r_api::EPropertyCustomGcode &invalid_id_property = invalid_id_event.script_gcode(
        "M117 invalid", GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM, uint16_t(0));
    invalid_id_property.config_id = extrusion_data_id(0x7fffffffu);
    CHECK_THROWS_AS(firmware.write_event(invalid_id_event.readonly()), std::invalid_argument);

    slic3r_api::StoredExtrusionEntity corrupt_event(storage);
    slic3r_api::EPropertyCustomGcode &corrupt_property = corrupt_event.script_gcode(
        "M117 corrupt", GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM, uint16_t(0));
    REQUIRE(corrupt_event.store_property_string(
                slic3r_api::EPropertyCustomGcode::property_type,
                &corrupt_property.config_id,
                "SCFG9\n0\n") != EXTRUSION_DATA_ID_INVALID);
    CHECK_THROWS(firmware.write_event(corrupt_event.readonly()));

    slic3r_api::StoredConfig first_config(storage);
    first_config.get_or_add("first_only", SLIC3R_CONFIG_OPTION_INT).set_int(1);
    slic3r_api::StoredExtrusionEntity first_event(storage);
    first_event.script_gcode(
        "M117 [first_only]", GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM, first_config, uint16_t(0));
    CHECK(firmware.write_event(first_event.readonly()) == "M117 1\n");

    slic3r_api::StoredConfig second_config(storage);
    second_config.get_or_add("second_only", SLIC3R_CONFIG_OPTION_INT).set_int(2);
    slic3r_api::StoredExtrusionEntity second_event(storage);
    second_event.script_gcode(
        "M117 [first_only]", GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM, second_config, uint16_t(0));
    CHECK_THROWS(firmware.write_event(second_event.readonly()));
}
TEST_CASE("ExtrusionPrinter names every custom G-code kind",
          "[plugins][gcode][custom-gcode][diagnostic]")
{
    const auto printed_kind = [](ExtrusionPropertyCustomGcodeText::Code code) {
        ExtrusionNop entity(ExtrusionPropertyCustomGcodeText(code, "test"));
        ExtrusionPrinter printer(/*mult=*/1.0, /*trunc=*/0, /*json=*/true);
        printer.traverse(entity);
        return printer.str();
    };

    CHECK(printed_kind(ExtrusionPropertyCustomGcodeText::Code::GCODE).find(
              "\"kind\":\"gcode\"") != std::string::npos);
    CHECK(printed_kind(ExtrusionPropertyCustomGcodeText::Code::COMMENT).find(
              "\"kind\":\"comment\"") != std::string::npos);
    CHECK(printed_kind(ExtrusionPropertyCustomGcodeText::Code::SCRIPT).find(
              "\"kind\":\"script\"") != std::string::npos);
    CHECK(printed_kind(static_cast<ExtrusionPropertyCustomGcodeText::Code>(99)).find(
              "\"kind\":\"unknown\"") != std::string::npos);
}
TEST_CASE("Legacy custom scripts substitute and update machine state",
          "[plugins][gcode][legacy][custom-gcode]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "start_gcode", "" },
        { "end_gcode", "" },
        { "gcode_comments", "0" }
    });

    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
    print.process();

    GCodeGenerator generator;
    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    const std::string output_path_string = output_path.string();
    generator.do_export(&print, output_path_string.c_str());
    remove_output_pair(output_path);

    // This is the same parser entry point and diagnostic name used by the
    // SCRIPT branch of GCodeGenerator::apply_property().
    const std::string move = generator.placeholder_parser_process(
        "extrusion_custom_gcode_script", "G1 X123 Y45", EXTRUDER_ID_UNKNOWN);
    CHECK(move.find("G1 X123 Y45") != std::string::npos);
    CHECK(is_approx(generator.writer().get_position().x(), 123.));
    CHECK(is_approx(generator.writer().get_position().y(), 45.));

    const std::string state = generator.placeholder_parser_process(
        "extrusion_custom_gcode_script",
        "M117 X{current_position[0]} Y{current_position[1]}", EXTRUDER_ID_UNKNOWN);
    CHECK(state.find("M117 X123 Y45") != std::string::npos);
    CHECK(state.find("{current_position") == std::string::npos);
}
