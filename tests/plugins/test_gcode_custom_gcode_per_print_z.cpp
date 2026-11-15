#include <catch2/catch.hpp>

#include <algorithm>
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "gcode_test_helpers.hpp"
#include "perimeter_test_helpers.hpp"
#include "plugin_test_helpers.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/CustomGCode.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Plugins/GCode/CustomGCodePerPrintZRecord.hpp"
#include "libslic3r/Plugins/GCode/Firmware/KlipperGCodeFirmware.hpp"
#include "libslic3r/Plugins/GCode/Firmware/Marlin1GCodeFirmware.hpp"
#include "libslic3r/Plugins/GCode/Firmware/Marlin2GCodeFirmware.hpp"
#include "libslic3r/Plugins/GCode/Firmware/PrusaGCodeFirmware.hpp"
#include "libslic3r/Plugins/GCode/Firmware/RepRapGCodeFirmware.hpp"
#include "libslic3r/Plugins/GCode/Firmware/SprinterGCodeFirmware.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Plugins/Ordering/ObjectInstanceCatalog.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"
#include "libslic3r/Steps/StepExtrusionEdition.hpp"

/*
Custom height-marker plugin tests
=================================

These tests exercise the boundary between the host-owned PrintRecord and the
ordered PrintingPlan. The fixtures deliberately build the record independently
from Model so malformed producer data and plugin-side validation can be tested
without involving GUI persistence.
SEMM checks use the ordering plugin's real height-marker visits, including
empty visits for a color change on a tool that prints no object extrusion.
*/

namespace {

using namespace Slic3r;
using namespace Slic3r::Printing;
using namespace Slic3r::Test::GCode;
using namespace slic3r_api::GCodeGeneration::CustomGCodePerPrintZPlugin;

/* Publish a complete record table while preserving the row order supplied by the test. */
void publish_record(Print &print,
                    CustomGCode::Mode mode,
                    const std::vector<CustomGCode::Item> &items)
{
    DynamicConfig &record = print.records().get_or_add(CustomGCode::PrintRecordChannel);
    record.clear();

    std::vector<double> print_z;
    std::vector<int32_t> types;
    std::vector<int32_t> extruders;
    std::vector<std::string> colors;
    std::vector<std::string> extras;
    print_z.reserve(items.size());
    types.reserve(items.size());
    extruders.reserve(items.size());
    colors.reserve(items.size());
    extras.reserve(items.size());
    for (const CustomGCode::Item &item : items) {
        print_z.push_back(unscaled(item.print_z_));
        types.push_back(int32_t(item.type));
        extruders.push_back(item.extruder);
        colors.push_back(item.color);
        extras.push_back(item.extra);
    }

    record.set_key_value(CustomGCode::PrintRecordSizeKey,
                         new ConfigOptionInt(int32_t(items.size())));
    record.set_key_value(CustomGCode::PrintRecordModeKey,
                         new ConfigOptionInt(int32_t(mode)));
    record.set_key_value(CustomGCode::PrintRecordPrintZKey,
                         new ConfigOptionFloats(print_z));
    ConfigOptionInts *type_option = new ConfigOptionInts();
    type_option->set(types);
    record.set_key_value(CustomGCode::PrintRecordTypeKey, type_option);
    ConfigOptionInts *extruder_option = new ConfigOptionInts();
    extruder_option->set(extruders);
    record.set_key_value(CustomGCode::PrintRecordExtruderKey, extruder_option);
    record.set_key_value(CustomGCode::PrintRecordColorKey,
                         new ConfigOptionStrings(colors));
    record.set_key_value(CustomGCode::PrintRecordExtraKey,
                         new ConfigOptionStrings(extras));
}

/* Return the script child from the tag-plus-payload wrapper emitted by the plugin. */
const ExtrusionEntity &event_payload(const ExtrusionEntity &event)
{
    REQUIRE(event.child_count() == 2);
    return event.child(1);
}

/* Expose the protected fallback hook without changing the production API. */
template<class Session>
class EmptyScriptFallbackProbe : public Session
{
public:
    std::string fallback(gcode_script_type type) const
    {
        return Session::resolve_empty_script(type, nullptr);
    }
};

} // namespace

TEST_CASE("Custom height passes form one activation group",
          "[plugins][gcode][custom-per-print-z][activation]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const std::map<std::string, std::set<std::string>> &groups =
        Orchestrator::instance().activation_groups();
    const std::map<std::string, std::set<std::string>>::const_iterator group =
        groups.find("custom_gcode_per_print_z");
    REQUIRE(group != groups.end());
    CHECK(group->second == std::set<std::string>{
        "ordering.custom_gcode_tool_overrides",
        "ordering.custom_gcode_event_tools",
        "gcode.custom_gcode_per_print_z"
    });
}

TEST_CASE("Custom height record reader validates the complete table",
          "[plugins][gcode][custom-per-print-z][record]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    configure_standard_firmware(print, 2);
    publish_record(print, CustomGCode::MultiExtruder, {
        {scale_to_layer_coord(0.2), CustomGCode::ColorChange, 1, "#ff0000", "first"},
        {scale_to_layer_coord(0.4), CustomGCode::PausePrint, 2, "", "pause"}
    });

    const slic3r_api::Print print_view(
        reinterpret_cast<const print_handle *>(&print));
    const std::optional<CustomGCodeTable> table = read_custom_gcode_per_print_z(print_view);
    REQUIRE(table.has_value());
    REQUIRE(table->rows.size() == 2);
    CHECK(table->mode == CustomGCode::MultiExtruder);
    CHECK(table->rows[0].print_z_mm == Approx(0.2));
    CHECK(table->rows[0].color == "#ff0000");
    CHECK(table->rows[1].type == CustomGCode::PausePrint);
    CHECK(table->rows[1].extra == "pause");

    /* A malformed column must reject the entire record instead of exposing a
       valid prefix to one of the three coordinated plugins. */
    DynamicConfig &record = print.records().get_or_add(CustomGCode::PrintRecordChannel);
    record.set_key_value(CustomGCode::PrintRecordExtraKey,
                         new ConfigOptionStrings({"only one row"}));
    CHECK_THROWS_AS(read_custom_gcode_per_print_z(print_view), std::runtime_error);
}

TEST_CASE("Custom height row tools validate before zero-based conversion",
          "[plugins][gcode][custom-per-print-z][extruder-index]")
{
    CustomGCodeRow row;
    row.extruder = 1;
    CHECK(custom_gcode_row_extruder_index(row, 2) == extruder_index_t(0));
    row.extruder = 2;
    CHECK(custom_gcode_row_extruder_index(row, 2) == extruder_index_t(1));

    for (const int32_t invalid : { -1, 0, 3 }) {
        row.extruder = invalid;
        CHECK_THROWS_AS(custom_gcode_row_extruder_index(row, 2), std::out_of_range);
    }
    row.extruder = 32769;
    CHECK_THROWS_AS(custom_gcode_row_extruder_index(row, 32769), std::out_of_range);
}

TEST_CASE("Custom height rows map to the first matching layer in model order",
          "[plugins][gcode][custom-per-print-z][mapping]")
{
    PrintingPlan native_plan;
    native_plan.groups.emplace_back();
    native_plan.groups.front().layers.emplace_back();
    native_plan.groups.front().layers.back().print_z = scale_i(0.2);
    native_plan.groups.front().layers.emplace_back();
    native_plan.groups.front().layers.back().print_z = scale_i(0.4);

    CustomGCodeTable table;
    table.rows = {
        {0.1, CustomGCode::PausePrint, 1, "", "first", 0},
        {0.2, CustomGCode::Custom, 1, "", "second", 1},
        {0.21, CustomGCode::Template, 1, "", "third", 2},
        {0.5, CustomGCode::PausePrint, 1, "", "above", 3}
    };

    const slic3r_api::PrintingPlan plan(
        reinterpret_cast<printing_plan_handle *>(&native_plan));
    const std::vector<std::vector<const CustomGCodeRow *>> mapped =
        rows_by_layer(table, plan.group(0));
    REQUIRE(mapped.size() == 2);
    REQUIRE(mapped[0].size() == 2);
    REQUIRE(mapped[1].size() == 1);
    CHECK(mapped[0][0]->source_row == 0);
    CHECK(mapped[0][1]->source_row == 1);
    CHECK(mapped[1][0]->source_row == 2);
}

TEST_CASE("Custom height markers become ordered typed plan events",
          "[plugins][gcode][custom-per-print-z][extrusion-edit]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    configure_standard_firmware(print);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.option<ConfigOptionString>("color_change_gcode")->value.clear();
    config.option<ConfigOptionString>("pause_print_gcode")->value.clear();
    config.option<ConfigOptionString>("template_custom_gcode")->value.clear();
    config.option<ConfigOptionStrings>("start_filament_gcode")->set(std::vector<std::string>{"M117 filament start"});

    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    PrintingGroup &group = plan.groups.back();
    group.layers.emplace_back();
    PrintingLayerGroup &layer = group.layers.back();
    layer.print_z = scale_i(0.2);
    layer.tool_groups.emplace_back();
    layer.tool_groups.back().extruder_id = 0;

    publish_record(print, CustomGCode::MultiAsSingle, {
        {scale_to_layer_coord(0.2), CustomGCode::ColorChange, 1, "#112233", ""},
        {scale_to_layer_coord(0.2), CustomGCode::PausePrint, 1, "", "Inspect layer"},
        {scale_to_layer_coord(0.2), CustomGCode::ToolChange, 1, "#445566", ""},
        {scale_to_layer_coord(0.2), CustomGCode::Template, 1, "", ""},
        {scale_to_layer_coord(0.2), CustomGCode::Custom, 1, "", "G4 P25"}
    });

    Steps::StepExtrusionEdition::clear(print);
    Steps::StepExtrusionEdition::run_step(Orchestrator::instance(), print);

    const ExtrusionEntity &events = layer.tool_groups.front().events.before();
    REQUIRE(events.child_count() == 6);
    const ExtrusionPropertyCustomGcode *filament_start =
        events.child(0).get_property<ExtrusionPropertyCustomGcode>();
    REQUIRE(filament_start != nullptr);
    CHECK(filament_start->script_type == GCODE_SCRIPT_TYPE_START_FILAMENT_GCODE);
    const ExtrusionEntity &color = event_payload(events.child(1));
    const ExtrusionEntity &pause = event_payload(events.child(2));
    const ExtrusionEntity &converted_tool = event_payload(events.child(3));
    const ExtrusionEntity &templated = event_payload(events.child(4));
    const ExtrusionEntity &custom = event_payload(events.child(5));

    const ExtrusionPropertyCustomGcode *color_property =
        color.get_property<ExtrusionPropertyCustomGcode>();
    const ExtrusionPropertyCustomGcode *pause_property =
        pause.get_property<ExtrusionPropertyCustomGcode>();
    const ExtrusionPropertyCustomGcode *tool_property =
        converted_tool.get_property<ExtrusionPropertyCustomGcode>();
    const ExtrusionPropertyCustomGcode *template_property =
        templated.get_property<ExtrusionPropertyCustomGcode>();
    const ExtrusionPropertyCustomGcode *custom_property =
        custom.get_property<ExtrusionPropertyCustomGcode>();
    REQUIRE(color_property != nullptr);
    REQUIRE(pause_property != nullptr);
    REQUIRE(tool_property != nullptr);
    REQUIRE(template_property != nullptr);
    REQUIRE(custom_property != nullptr);

    CHECK(color_property->script_type == GCODE_SCRIPT_TYPE_COLOR_CHANGE_GCODE);
    CHECK(color_property->processing_extruder_id == 0);
    CHECK(stored_script_int(color, *color_property, "color_change_extruder") == 0);
    CHECK(stored_script_string(color, *color_property, "next_color") == "#112233");
    CHECK(stored_script_string(color, *color_property, "next_colour") == "#112233");
    CHECK(pause_property->script_type == GCODE_SCRIPT_TYPE_PAUSE_PRINT_GCODE);
    CHECK(stored_script_string(pause, *pause_property, "pause_message") == "Inspect layer");
    CHECK(tool_property->script_type == GCODE_SCRIPT_TYPE_COLOR_CHANGE_GCODE);
    CHECK(stored_script_string(converted_tool, *tool_property, "next_color") == "#445566");
    CHECK(template_property->script_type == GCODE_SCRIPT_TYPE_TEMPLATE_CUSTOM_GCODE);
    CHECK(custom_property->kind == C_EXTRUSION_CUSTOM_GCODE_GCODE);
    CHECK(custom.custom_gcode_string(*custom_property) == "G4 P25");
}

TEST_CASE("Built-in firmwares resolve empty height-marker scripts",
          "[plugins][gcode][custom-per-print-z][firmware]")
{
    using namespace slic3r_api::GCodeGeneration::Firmware;

    EmptyScriptFallbackProbe<Marlin1GCodeFirmwareSession> marlin1;
    EmptyScriptFallbackProbe<Marlin2GCodeFirmwareSession> marlin2;
    EmptyScriptFallbackProbe<PrusaGCodeFirmwareSession> prusa;
    EmptyScriptFallbackProbe<RepRapGCodeFirmwareSession> reprap;
    EmptyScriptFallbackProbe<KlipperGCodeFirmwareSession> klipper;
    EmptyScriptFallbackProbe<SprinterGCodeFirmwareSession> sprinter;

    CHECK(marlin1.fallback(GCODE_SCRIPT_TYPE_COLOR_CHANGE_GCODE) == "M600\n");
    CHECK(marlin1.fallback(GCODE_SCRIPT_TYPE_PAUSE_PRINT_GCODE) == "M0\n");
    CHECK(marlin2.fallback(GCODE_SCRIPT_TYPE_COLOR_CHANGE_GCODE) == "M600\n");
    CHECK(marlin2.fallback(GCODE_SCRIPT_TYPE_PAUSE_PRINT_GCODE) == "M0\n");
    CHECK(prusa.fallback(GCODE_SCRIPT_TYPE_COLOR_CHANGE_GCODE) == "M600\n");
    CHECK(prusa.fallback(GCODE_SCRIPT_TYPE_PAUSE_PRINT_GCODE) == "M601\n");
    CHECK(reprap.fallback(GCODE_SCRIPT_TYPE_COLOR_CHANGE_GCODE) == "M600\n");
    CHECK(reprap.fallback(GCODE_SCRIPT_TYPE_PAUSE_PRINT_GCODE) == "M226\n");
    CHECK(klipper.fallback(GCODE_SCRIPT_TYPE_COLOR_CHANGE_GCODE) == "PAUSE\n");
    CHECK(klipper.fallback(GCODE_SCRIPT_TYPE_PAUSE_PRINT_GCODE) == "PAUSE\n");
    CHECK(sprinter.fallback(GCODE_SCRIPT_TYPE_COLOR_CHANGE_GCODE).find("Unsupported") !=
          std::string::npos);
    CHECK(sprinter.fallback(GCODE_SCRIPT_TYPE_PAUSE_PRINT_GCODE).find("Unsupported") !=
          std::string::npos);
}

TEST_CASE("Custom height semantic tags precede exactly one script payload",
          "[plugins][gcode][custom-per-print-z][markers]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    configure_standard_firmware(print);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.option<ConfigOptionString>("color_change_gcode")->value = "M600\n";
    config.option<ConfigOptionString>("pause_print_gcode")->value = "M601\n";
    config.option<ConfigOptionStrings>("start_filament_gcode")->set(std::vector<std::string>{""});

    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    PrintingLayerGroup &layer = plan.groups.back().layers.emplace_back();
    layer.print_z = scale_i(0.2);
    layer.tool_groups.emplace_back();
    layer.tool_groups.back().extruder_id = 0;

    publish_record(print, CustomGCode::MultiAsSingle, {
        {scale_to_layer_coord(0.2), CustomGCode::ColorChange, 1, "#112233", ""},
        {scale_to_layer_coord(0.2), CustomGCode::PausePrint, 1, "", "Inspect"},
        {scale_to_layer_coord(0.2), CustomGCode::Custom, 1, "", "M117 custom action"}
    });

    Steps::StepExtrusionEdition::clear(print);
    Steps::StepExtrusionEdition::run_step(Orchestrator::instance(), print);

    const ExtrusionEntity &events = layer.tool_groups.front().events.before();
    REQUIRE(events.child_count() == 3);
    const std::string output = export_with_firmware(
        print, "marlin2");

    const std::string color = ";COLOR_CHANGE,T0,#112233\n";
    const std::string pause = ";PAUSE_PRINT\n";
    const std::string custom = ";CUSTOM_GCODE\n";
    REQUIRE(output.find(color + "M600\n") != std::string::npos);
    REQUIRE(output.find(pause + "M601\n") != std::string::npos);
    REQUIRE(output.find(custom + "M117 custom action\n") != std::string::npos);
    CHECK(count_occurrences(output, color) == 1);
    CHECK(count_occurrences(output, pause) == 1);
    CHECK(count_occurrences(output, custom) == 1);
    CHECK(count_occurrences(output, "M600\n") == 1);
    CHECK(count_occurrences(output, "M601\n") == 1);
    CHECK(count_occurrences(output, "M117 custom action\n") == 1);
}


TEST_CASE("Custom G-code per print Z plugin owns its script settings",
          "[plugins][gcode][custom-gcode-per-print-z][config]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();

    Plugin *plugin = orchestrator.get_plugin("gcode.custom_gcode_per_print_z");
    REQUIRE(plugin != nullptr);

    std::vector<std::string> defined_keys;
    for (const Plugin::DefinedConfigKey &key : plugin->get_defined_config_keys())
        defined_keys.push_back(key.key);
    CHECK(defined_keys.size() == 3);
    for (const char *expected : { "color_change_gcode", "pause_print_gcode", "template_custom_gcode" })
        CHECK(std::find(defined_keys.begin(), defined_keys.end(), expected) != defined_keys.end());

    for (const char *key : { "color_change_gcode", "pause_print_gcode", "template_custom_gcode" }) {
        const ConfigOptionDef *def = PrintConfigDef::instance().get(key);
        REQUIRE(def != nullptr);
        CHECK(def->type == coString);
        CHECK(def->container_type == ConfigOptionContainerType::Plater);
        CHECK(def->option_preset_type == RAW_PRESET_TYPE_FFF_PRINTER);
        CHECK(def->invalidates_step == STEP_EXTRUSION_EDIT);
        CHECK(def->multiline);
        CHECK(def->full_width);
        CHECK(PrintConfig::defaults().option<ConfigOptionString>(key)->value.empty());
    }

    // The scripts stay always editable: the plugin owns no activation rule.
    for (const Orchestrator::PluginGuiRule &rule : orchestrator.gui_rules().rules())
        CHECK(rule.owner != "gcode.custom_gcode_per_print_z");
}

// A height marker can add a second logical tool without adding object extrusion.
TEST_CASE("SEMM counts tools introduced only by a height color marker",
          "[plugins][gcode][custom-per-print-z][semm]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const int32_t marker_tool = GENERATE(1, 2);
    Print print;
    configure_standard_firmware(print, 2);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.single_extruder_multi_material.value = true;
    PrintingPlan &native_plan = print.mutable_printing_plan();
    native_plan.groups.emplace_back();
    for (const double z : {0.2, 0.4}) {
        PrintingLayerGroup &layer = native_plan.groups.back().layers.emplace_back();
        layer.print_z = scale_i(z);
        layer.tool_groups.emplace_back();
        layer.tool_groups.back().extruder_id = 0;
    }
    const slic3r_api::ConstPrintingPlan plan(
        reinterpret_cast<const printing_plan_handle *>(&native_plan));
    CHECK_FALSE(plan.uses_single_extruder_multi_material(true));
    publish_record(print, CustomGCode::MultiAsSingle, {
        {scale_to_layer_coord(0.4), CustomGCode::ColorChange, marker_tool, "#112233", ""}
    });
    Orchestrator &orchestrator = Orchestrator::instance();
    Plugin *plugin = orchestrator.get_plugin("ordering.custom_gcode_event_tools");
    REQUIRE(plugin != nullptr);
    run_ctx_extrusion_ordering payload = {};
    payload.print = reinterpret_cast<const print_handle *>(&print);
    payload.plan = reinterpret_cast<printing_plan_handle *>(&native_plan);
    plugin_host_context host_context =
        orchestrator.prepare_plugin_host_context(STEP_ORDERING, plugin, &print);
    plugin_run_context run_context =
        orchestrator.prepare_plugin_run_context(STEP_ORDERING, plugin, &host_context);
    run_context.data = &payload;
    plugin->setup(run_context, 1);
    plugin->setup_run(run_context);
    plugin->run(run_context);

    const PrintingLayerGroup &changed_layer = native_plan.groups.back().layers.back();
    REQUIRE(changed_layer.tool_groups.size() == (marker_tool == 2 ? 2 : 1));
    CHECK(changed_layer.tool_groups.front().extruder_id == marker_tool - 1);
    CHECK(changed_layer.tool_groups.front().extrusions.empty());
    CHECK(plan.uses_single_extruder_multi_material(true) == (marker_tool == 2));
    CHECK_FALSE(plan.uses_single_extruder_multi_material(false));
}

// Object tool selection at a height must reach the same plan-based SEMM decision.
TEST_CASE("SEMM counts a second tool selected only by a height tool change",
          "[plugins][gcode][custom-per-print-z][semm]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Slic3r::Test::PerimeterPluginTests::PreparedPerimeterPrint prepared;
    Slic3r::Test::PerimeterPluginTests::prepare_cube_print(
        prepared, Slic3r::Test::PerimeterPluginTests::perimeter_config({}));
    Print &print = prepared.print;
    configure_standard_firmware(print, 2);
    const_cast<PrintConfig &>(print.config()).single_extruder_multi_material.value = true;
    PrintingPlan &native_plan = print.mutable_printing_plan();
    const slic3r_api::Print print_view(reinterpret_cast<const print_handle *>(&print));
    const uint32_t catalog_id = printing_plan_view(print).find_or_add_object_instance(print_view.object(0), 0);
    PluginStorage storage;
    slic3r_api::Ordering::ObjectInstanceCatalogPlugin::populate_object_instance_catalog(
        reinterpret_cast<const print_handle *>(&print),
        reinterpret_cast<printing_plan_handle *>(&native_plan),
        reinterpret_cast<storage_handle *>(&storage));
    native_plan.groups.emplace_back();
    native_plan.groups.back().printing_catalog_ids.push_back(catalog_id);
    PrintObject &object = print.object(0);
    LayerRegionIsland source({&object.layer(0).region(0)}, 0);
    for (const double z : {0.2, 0.4}) {
        PrintingLayerGroup &layer = native_plan.groups.back().layers.emplace_back();
        layer.print_z = scale_i(z);
        layer.tool_groups.emplace_back();
        layer.tool_groups.back().extruder_id = 0;
        append_empty_extrusion(layer.tool_groups.back(), ExtrusionRole::Perimeter, catalog_id, &source);
    }
    publish_record(print, CustomGCode::MultiAsSingle, {
        {scale_to_layer_coord(0.4), CustomGCode::ToolChange, 2, "#112233", ""}
    });
    Orchestrator &orchestrator = Orchestrator::instance();
    Plugin *plugin = orchestrator.get_plugin("ordering.custom_gcode_tool_overrides");
    REQUIRE(plugin != nullptr);
    run_ctx_extrusion_ordering payload = {};
    payload.print = reinterpret_cast<const print_handle *>(&print);
    payload.plan = reinterpret_cast<printing_plan_handle *>(&native_plan);
    plugin_host_context host_context =
        orchestrator.prepare_plugin_host_context(STEP_ORDERING, plugin, &print);
    plugin_run_context run_context =
        orchestrator.prepare_plugin_run_context(STEP_ORDERING, plugin, &host_context);
    run_context.data = &payload;
    plugin->setup(run_context, 1);
    plugin->setup_run(run_context);
    plugin->run(run_context);
    const slic3r_api::ConstPrintingPlan plan(
        reinterpret_cast<const printing_plan_handle *>(print.printing_plan()));
    CHECK(plan.uses_single_extruder_multi_material(true));
    CHECK_FALSE(plan.uses_single_extruder_multi_material(false));
    bool earlier_object_tool = false;
    bool later_object_tool = false;
    for (const PrintingGroup &group : print.printing_plan()->groups)
        for (const PrintingLayerGroup &layer : group.layers)
            for (const PrintingToolGroup &tool : layer.tool_groups)
                if (!tool.extrusions.empty()) {
                    earlier_object_tool = earlier_object_tool || tool.extruder_id == 0;
                    later_object_tool = later_object_tool || tool.extruder_id == 1;
                }
    CHECK(earlier_object_tool);
    CHECK(later_object_tool);
}
