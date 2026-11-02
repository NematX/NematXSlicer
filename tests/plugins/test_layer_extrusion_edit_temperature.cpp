#include <catch2/catch.hpp>

#include <array>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "gcode_test_helpers.hpp"
#include "layer_extrusion_edit_transition_test_helpers.hpp"
#include "perimeter_test_helpers.hpp"
#include "plugin_test_helpers.hpp"
#include "test_data.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/internal/LayerAccess.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/PrintRegion.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"
#include "libslic3r/Steps/StepExtrusionEdition.hpp"
#include "libslic3r/Steps/StepLayerExtrusionEdition.hpp"
#include "libslic3r/Steps/StepLayerHeightGeneration.hpp"
#include "libslic3r/Steps/StepSlicing.hpp"

/*
Temperature plugin tests
========================

These fixtures build small source-linked PrintingPlans, run the plan-level and
layer-level temperature providers, and inspect the resulting firmware output.
The cases cover target precedence, sequential heater tracking, startup policy,
tool changes, second-layer updates, SEMM tower ownership, and object groups.
*/

namespace {

using namespace Slic3r;
using namespace Slic3r::Printing;
using namespace Slic3r::Test::GCode;
using namespace Slic3r::Test::PerimeterPluginTests;
using Slic3r::Test::TransitionPipeline::ScopedActivePlugins;

// Restore every active plugin after one activation change; construction checks the requested state and destruction restores the captured set even when a test throws.
class ScopedPluginActivation
{
public:
    // Capture the active plugin handles and set the requested plugin to the desired state.
    ScopedPluginActivation(const char *plugin_id, const bool active) :
        m_orchestrator(Orchestrator::instance())
    {
        for (Plugin *plugin : m_orchestrator.active_plugins())
            m_previous.push_back(plugin);
        if (m_orchestrator.is_plugin_active(plugin_id) != active)
            REQUIRE(m_orchestrator.set_plugin_active(plugin_id, active));
    }

    // Restore the exact captured active set and do not leak temporary activation changes.
    ~ScopedPluginActivation()
    {
        m_orchestrator.clear_active_plugins();
        for (Plugin *plugin : m_previous)
            m_orchestrator.set_plugin_active(plugin, true);
    }

private:
    Orchestrator &m_orchestrator;
    std::vector<Plugin *> m_previous;
};

// Add one empty plan layer with a stable print-Z value for the temperature pass.
PrintingLayerGroup &append_temperature_layer(Print &print, coord_t print_z)
{
    PrintingPlan &plan = print.mutable_printing_plan();
    if (plan.groups.empty())
        plan.groups.emplace_back();
    plan.groups.back().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.back().layers.back();
    layer.print_z = print_z;
    return layer;
}

// Start a new independent PrintingGroup before adding its first temperature layer.
PrintingLayerGroup &append_temperature_group_layer(Print &print, coord_t print_z)
{
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.back().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.back().layers.back();
    layer.print_z = print_z;
    return layer;
}

// Add one source-linked path to a tool visit, allowing regional settings to be recovered.
void append_temperature_root(PrintingToolGroup &tool_group,
                             const LayerRegionIsland &source,
                             double start_x,
                             double end_x)
{
    tool_group.region_islands.push_back(&source);
    std::unique_ptr<ExtrusionPath> path = make_firmware_path(
        ArcPolyline(Points{Point(scale_i(start_x), scale_i(0.0)),
                           Point(scale_i(end_x), scale_i(0.0))}),
        20.f, 400.f);
    PrintingExtrusion extrusion;
    extrusion.region_island = &source;
    extrusion.sregion_island_role = ExtrusionRole::Perimeter;
    extrusion.root = std::move(path);
    tool_group.extrusions.push_back(std::move(extrusion));
}

// Add a regionless wipe-tower root before a normal object root in one tool visit.
void append_temperature_tower_root(PrintingToolGroup &tool_group)
{
    PrintingExtrusion tower;
    tower.sregion_island_role = ExtrusionRole::WipeTower;
    tower.root = make_firmware_path(
        ArcPolyline(Points{Point(scale_i(2.0), scale_i(0.0)),
                           Point(scale_i(2.5), scale_i(0.0))}),
        20.f, 400.f);
    tool_group.extrusions.push_back(std::move(tower));
}

// Add a tool visit that owns one source-linked path, allowing regional settings to be recovered.
PrintingToolGroup &append_temperature_visit(PrintingLayerGroup &layer,
                                            extruder_index_t tool,
                                            const LayerRegionIsland &source,
                                            double start_x,
                                            double end_x)
{
    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool_group = layer.tool_groups.back();
    tool_group.extruder_id = tool;
    append_temperature_root(tool_group, source, start_x, end_x);
    return tool_group;
}

// Slice two owned cube objects at initial heights 0.2 and 0.3 with a shared 0.1 layer increment; populate source regions without creating plan roots, and let configuration or slicing failures propagate.
void prepare_temperature_source_layers(PreparedPerimeterPrint &prepared,
                                       const DynamicPrintConfig &config)
{
    Test::init_print({Test::TestMesh::cube_20x20x20, Test::TestMesh::cube_20x20x20},
                     prepared.print, prepared.model, config);
    prepared.model.objects()[0].config.set_deserialize_strict("first_layer_height", "0.2");
    prepared.model.objects()[1].config.set_deserialize_strict("first_layer_height", "0.3");
    prepared.model.objects()[0].config.set_deserialize_strict("layer_height", "0.1");
    prepared.model.objects()[1].config.set_deserialize_strict("layer_height", "0.1");
    prepared.print.apply(prepared.model, config);
    Orchestrator &orchestrator = Orchestrator::instance();
    Steps::StepLayerHeightGeneration::run_step(orchestrator, prepared.print);
    Steps::StepSlicing::run_step(orchestrator, prepared.print);
    for (PrintObject &object : prepared.print.objects())
        for (Layer &layer : object.layers())
            for (LayerSliceIsland &island : layer.islands())
                island.fill_regions(layer);
    REQUIRE(prepared.print.objects().size() == 2);
    REQUIRE(prepared.print.object(0).layer_count() > 1);
    REQUIRE(prepared.print.object(0).layer(0).unscaled_print_z() == Approx(0.2));
    REQUIRE(prepared.print.object(1).layer(0).unscaled_print_z() == Approx(0.3));
    REQUIRE(prepared.print.object(0).layer(1).unscaled_print_z() == Approx(0.3));
}

// Run both temperature-producing steps before exporting the already ordered plan.
void run_temperature_plugins(Print &print)
{
    Orchestrator &orchestrator = Orchestrator::instance();
    Steps::StepLayerExtrusionEdition::run_step(orchestrator, print);
    Steps::StepExtrusionEdition::run_step(orchestrator, print);
}

// Run only the temperature layer provider while preserving the orchestrator's active set; fake tower roots use this because the real wipe-tower producer did not create them.
void run_temperature_layer_plugin_only(Print &print)
{
    ScopedActivePlugins active({"layer_extrusion_edit.temperature.default"});
    Orchestrator &orchestrator = Orchestrator::instance();
    Steps::StepLayerExtrusionEdition::run_step(orchestrator, print);
}

// Run both temperature steps with only startup temperature disabled, restoring its original activation even when a step throws.
void run_temperature_plugins_without_startup(Print &print)
{
    ScopedPluginActivation startup("extrusion_edit.startup_temperature.default", false);
    run_temperature_plugins(print);
}

// Split exported G-code into logical lines without interpreting command substrings.
std::vector<std::string> gcode_lines(const std::string &output)
{
    std::vector<std::string> lines;
    std::istringstream stream(output);
    std::string line;
    while (std::getline(stream, line)) {
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        lines.push_back(line);
    }
    return lines;
}

// Return the first line exactly equal to the requested text at or after start.
size_t first_exact_line(const std::vector<std::string> &lines,
                        const std::string &text,
                        size_t start = 0)
{
    for (size_t idx = start; idx < lines.size(); ++idx)
        if (lines[idx] == text)
            return idx;
    return std::string::npos;
}

// Count lines exactly equal to the requested text.
size_t count_exact_lines(const std::vector<std::string> &lines, const std::string &text)
{
    size_t count = 0;
    for (const std::string &line : lines)
        if (line == text)
            ++count;
    return count;
}

// Identify a G1 move that carries E together with X or Y coordinates; E-only retractions and travel are not counted.
bool is_filament_move(const std::string &line)
{
    return line.compare(0, 3, "G1 ") == 0 && line.find(" E") != std::string::npos &&
        (line.find(" X") != std::string::npos || line.find(" Y") != std::string::npos);
}

// Identify a waited Marlin tool command addressed to one concrete tool.
bool is_wait_for_tool(const std::string &line, extruder_index_t tool)
{
    return line.compare(0, 4, "M109") == 0 &&
        line.find(" T" + std::to_string(tool)) != std::string::npos;
}

// Identify a line that is the command alone or followed by a space-separated suffix; Marlin appends " T<n>", so "M104 S210" matches "M104 S210 T0" but not "M104 S2100".
bool is_command_line(const std::string &line, const std::string &command)
{
    return line == command || line.compare(0, command.size() + 1, command + " ") == 0;
}

// Count the lines in [start, end) that are the requested command; an end past the last line stops at the last line.
size_t count_command_lines(const std::vector<std::string> &lines,
                           const std::string &command,
                           size_t start = 0,
                           size_t end = std::string::npos)
{
    size_t count = 0;
    for (size_t idx = start; idx < lines.size() && idx < end; ++idx)
        if (is_command_line(lines[idx], command))
            ++count;
    return count;
}

// Return the first line at or after start that is the requested command, or npos.
size_t first_command_line(const std::vector<std::string> &lines,
                          const std::string &command,
                          size_t start = 0)
{
    for (size_t idx = start; idx < lines.size(); ++idx)
        if (is_command_line(lines[idx], command))
            return idx;
    return std::string::npos;
}

// Return the first filament move at or after start, or npos.
size_t first_extrusion_line(const std::vector<std::string> &lines, size_t start)
{
    for (size_t idx = start; idx < lines.size(); ++idx)
        if (is_filament_move(lines[idx]))
            return idx;
    return std::string::npos;
}

// Add to the object one tower floor range [low, high) that carries its own print_temperature, with the layer_height and extruder every user-made range must hold.
void add_floor_height_range(ModelObject &object, double low, double high, int temperature)
{
    ModelConfig &floor_config = object.layer_config_ranges[{ low, high }];
    floor_config.set_key_value("layer_height", new ConfigOptionFloat(0.2));
    floor_config.set_key_value("extruder", new ConfigOptionInt(0));
    floor_config.set_key_value("print_temperature", new ConfigOptionInt(temperature));
}

// Check one floor change of the tower: the layer at previous_z has no M104, and the layer at floor_z requests command as its first M104 before its first extrusion; the command appears nowhere else.
void check_floor_change(const std::vector<std::string> &lines,
                        const std::string &previous_z,
                        const std::string &floor_z,
                        const std::string &command)
{
    const size_t previous_layer = first_exact_line(lines, previous_z);
    const size_t floor_layer = first_exact_line(lines, floor_z);
    REQUIRE(previous_layer != std::string::npos);
    REQUIRE(floor_layer != std::string::npos);
    REQUIRE(previous_layer < floor_layer);
    CHECK(count_command_lines(lines, "M104", previous_layer, floor_layer) == 0);
    const size_t change = first_command_line(lines, command, floor_layer);
    const size_t first_extrusion = first_extrusion_line(lines, floor_layer);
    REQUIRE(change != std::string::npos);
    REQUIRE(first_extrusion != std::string::npos);
    CHECK(change < first_extrusion);
    CHECK(first_command_line(lines, "M104", floor_layer) == change);
    CHECK(count_command_lines(lines, command) == 1);
}

// Append a second source region with a print-temperature override for root-order tests.
LayerRegionIsland append_temperature_override_region(Print &print, Layer &layer, int temperature)
{
    PrintRegionConfig config = layer.region(0).region().config();
    config.set_deserialize_strict("print_temperature", std::to_string(temperature));
    ApiInternal::LayerAccess::add_region(layer, print.register_print_region(config));
    return LayerRegionIsland({&layer.region(layer.region_count() - 1)}, 0);
}

// Count heater properties recursively without interpreting unrelated event properties.
size_t count_heater_properties(const ExtrusionEntity &entity)
{
    size_t count = entity.get_property<ExtrusionPropertyHeater>() != nullptr ? 1 : 0;
    for (size_t child_idx = 0; child_idx < entity.child_count(); ++child_idx)
        count += count_heater_properties(entity.child(child_idx));
    return count;
}

// Configure a two-tool profile with distinct startup, normal, bed, and chamber targets.
void configure_temperature_profile(Print &print)
{
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.first_layer_temperature.set(std::vector<int>{215, 220});
    config.temperature.set(std::vector<int>{200, 205});
    config.first_layer_bed_temperature.set(std::vector<int>{60, 65});
    config.bed_temperature.set(std::vector<int>{50, 55});
    config.chamber_temperature.set(std::vector<int>{70, 0});
    config.nozzle_diameter.set(std::vector<double>{0.4, 0.4});
    config.filament_diameter.set(std::vector<double>{1.75, 1.75});
    config.extrusion_multiplier.set(std::vector<double>{1.0, 1.0});
}

// Export a small source-linked two-layer plan after the plugin steps have materialized events.
std::string export_temperature_fixture(const DynamicPrintConfig &initial_config,
                                       bool ooze_prevention = false,
                                       bool matching_start_script = false)
{
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, initial_config);
    Print &print = prepared.print;
    configure_standard_firmware(print, 2);
    configure_temperature_profile(print);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.ooze_prevention.value = ooze_prevention;

    PrintObject &object = print.object(0);
    REQUIRE(object.layer_count() > 1);
    LayerRegionIsland first_source({&object.layer(0).region(0)}, 0);
    LayerRegionIsland second_source({&object.layer(1).region(0)}, 0);
    PrintingLayerGroup &first_layer = append_temperature_layer(
        print, object.layer(0).scaled_print_z());
    append_temperature_visit(first_layer, 0, first_source, 0.0, 1.0);
    append_temperature_visit(first_layer, 1, first_source, 1.0, 2.0);
    PrintingLayerGroup &second_layer = append_temperature_layer(
        print, object.layer(1).scaled_print_z());
    append_temperature_visit(second_layer, 1, second_source, 2.0, 3.0);
    append_temperature_visit(second_layer, 0, second_source, 3.0, 4.0);
    if (matching_start_script)
        config.option<ConfigOptionString>("start_gcode")->value =
            "M190 S65\nM191 S70\nM109 S215 T0";
    run_temperature_plugins(print);
    return export_with_firmware(print, "gcode.firmware.marlin2");
}

// Export one startup plan whose first visit uses tool 1 and whose later tool usage is explicit.
std::string export_startup_fixture(const std::string &firmware_id,
                                   bool semm,
                                   bool ooze_prevention,
                                   bool manual_start,
                                   bool autoemit)
{
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({}));
    Print &print = prepared.print;
    configure_standard_firmware(print, 2);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.single_extruder_multi_material.value = semm;
    config.ooze_prevention.value = ooze_prevention;
    config.start_gcode_manual.value = manual_start;
    config.autoemit_temperature_commands.value = autoemit;
    config.first_layer_temperature.set(std::vector<int>{205, 220});
    config.temperature.set(std::vector<int>{200, 210});
    config.first_layer_bed_temperature.set(std::vector<int>{60, 65});
    config.chamber_temperature.set(firmware_id == "gcode.firmware.marlin2" ?
        std::vector<int>{0, 70} : std::vector<int>{0, 0});
    config.option<ConfigOptionString>("start_gcode")->value = "; STARTUP_MARKER";
    config.option<ConfigOptionString>("end_gcode")->value.clear();
    if (ooze_prevention) {
        config.idle_temperature.set(std::vector<int>{150, -1});
        config.idle_temperature.set_enabled(true);
    }

    PrintObject &object = print.object(0);
    REQUIRE(object.layer_count() > 1);
    LayerRegionIsland first_source({&object.layer(0).region(0)}, 0);
    LayerRegionIsland second_source({&object.layer(1).region(0)}, 0);
    PrintingLayerGroup &first_layer = append_temperature_layer(
        print, object.layer(0).scaled_print_z());
    append_temperature_visit(first_layer, 1, first_source, 0.0, 1.0);
    PrintingLayerGroup &second_layer = append_temperature_layer(
        print, object.layer(1).scaled_print_z());
    append_temperature_visit(second_layer, 0, second_source, 1.0, 2.0);
    run_temperature_plugins(print);
    return export_with_firmware(print, firmware_id);
}

// Export a two-visit plan whose incoming tool is 0 so standby, tool selection and heating order are observable.
std::string export_tool_change_order_fixture(int idle_temperature = 150)
{
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({}));
    Print &print = prepared.print;
    configure_standard_firmware(print, 2);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.first_layer_temperature.set(std::vector<int>{215, 220});
    config.temperature.set(std::vector<int>{200, 205});
    config.first_layer_bed_temperature.set(std::vector<int>{0, 0});
    config.bed_temperature.set(std::vector<int>{0, 0});
    config.chamber_temperature.set(std::vector<int>{0, 0});
    // The exact movement lines below carry no F word: a short-layer slowdown
    // would change the speed and add one, and this test is about tool order.
    config.slowdown_below_layer_time.set(std::vector<double>{0., 0.});
    config.ooze_prevention.value = true;
    config.idle_temperature.set(std::vector<int>{-1, idle_temperature});
    config.idle_temperature.set_enabled(true);
    config.option<ConfigOptionString>("end_gcode")->value.clear();

    PrintObject &object = print.object(0);
    LayerRegionIsland source({&object.layer(0).region(0)}, 0);
    PrintingLayerGroup &layer = append_temperature_layer(
        print, object.layer(0).scaled_print_z());
    append_temperature_visit(layer, 1, source, 0.0, 1.0);
    append_temperature_visit(layer, 0, source, 1.0, 2.0);
    run_temperature_plugins(print);
    return export_with_firmware(print, "gcode.firmware.marlin2");
}

// Export two layers where the second layer has only the currently active tool and a bed override.
std::string export_second_layer_policy_fixture(bool semm, bool ooze_prevention)
{
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({}));
    Print &print = prepared.print;
    configure_standard_firmware(print, 2);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.single_extruder_multi_material.value = semm;
    config.ooze_prevention.value = ooze_prevention;
    config.first_layer_temperature.set(std::vector<int>{215, 220});
    config.temperature.set(std::vector<int>{200, 205});
    config.first_layer_bed_temperature.set(std::vector<int>{60, 65});
    config.bed_temperature.set(std::vector<int>{50, 55});
    config.print_bed_temperature.value = 45;
    config.print_bed_temperature.set_enabled(true);
    config.chamber_temperature.set(std::vector<int>{0, 0});
    config.option<ConfigOptionString>("end_gcode")->value.clear();

    PrintObject &object = print.object(0);
    LayerRegionIsland first_source({&object.layer(0).region(0)}, 0);
    LayerRegionIsland second_source({&object.layer(1).region(0)}, 0);
    PrintingLayerGroup &first_layer = append_temperature_layer(
        print, object.layer(0).scaled_print_z());
    append_temperature_visit(first_layer, 0, first_source, 0.0, 1.0);
    append_temperature_visit(first_layer, 1, first_source, 1.0, 2.0);
    PrintingLayerGroup &second_layer = append_temperature_layer(
        print, object.layer(1).scaled_print_z());
    append_temperature_visit(second_layer, 1, second_source, 2.0, 3.0);
    run_temperature_plugins(print);
    return export_with_firmware(print, "gcode.firmware.marlin2");
}

// Export two tool changes where the returning tool is four or five degrees above its known value.
std::string export_wait_margin_fixture(int normal_tool_zero)
{
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({}));
    Print &print = prepared.print;
    configure_standard_firmware(print, 2);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.first_layer_temperature.set(std::vector<int>{200, 220});
    config.temperature.set(std::vector<int>{normal_tool_zero, 220});
    config.first_layer_bed_temperature.set(std::vector<int>{0, 0});
    config.bed_temperature.set(std::vector<int>{0, 0});
    config.chamber_temperature.set(std::vector<int>{0, 0});
    config.ooze_prevention.value = true;
    config.standby_temperature_delta.value = 0;

    PrintObject &object = print.object(0);
    REQUIRE(object.layer_count() > 1);
    LayerRegionIsland first_source({&object.layer(0).region(0)}, 0);
    LayerRegionIsland second_source({&object.layer(1).region(0)}, 0);
    PrintingLayerGroup &first_layer = append_temperature_layer(
        print, object.layer(0).scaled_print_z());
    append_temperature_visit(first_layer, 0, first_source, 0.0, 1.0);
    append_temperature_visit(first_layer, 1, first_source, 1.0, 2.0);
    PrintingLayerGroup &second_layer = append_temperature_layer(
        print, object.layer(1).scaled_print_z());
    append_temperature_visit(second_layer, 1, second_source, 2.0, 3.0);
    append_temperature_visit(second_layer, 0, second_source, 3.0, 4.0);
    run_temperature_plugins(print);
    return export_with_firmware(print, "gcode.firmware.marlin2");
}

// Export a plan whose second tool first appears after the startup layer.
std::string export_late_tool_fixture(bool ooze_prevention)
{
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({}));
    Print &print = prepared.print;
    configure_standard_firmware(print, 2);
    configure_temperature_profile(print);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.ooze_prevention.value = ooze_prevention;
    PrintObject &object = print.object(0);
    REQUIRE(object.layer_count() > 1);
    LayerRegionIsland first_source({&object.layer(0).region(0)}, 0);
    LayerRegionIsland second_source({&object.layer(1).region(0)}, 0);
    PrintingLayerGroup &first_layer = append_temperature_layer(
        print, object.layer(0).scaled_print_z());
    append_temperature_visit(first_layer, 0, first_source, 0.0, 1.0);
    PrintingLayerGroup &second_layer = append_temperature_layer(
        print, object.layer(1).scaled_print_z());
    append_temperature_visit(second_layer, 1, second_source, 1.0, 2.0);
    run_temperature_plugins(print);
    return export_with_firmware(print, "gcode.firmware.marlin2");
}

} // namespace

TEST_CASE("Temperature plugins register and activate their two providers",
          "[plugins][temperature]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const Slic3r::Plugin *startup = Slic3r::Orchestrator::instance().get_plugin(
        "extrusion_edit.startup_temperature.default");
    const Slic3r::Plugin *layer = Slic3r::Orchestrator::instance().get_plugin(
        "layer_extrusion_edit.temperature.default");
    REQUIRE(startup != nullptr);
    REQUIRE(layer != nullptr);
    CHECK(Slic3r::Orchestrator::instance().is_plugin_active(startup));
    CHECK(Slic3r::Orchestrator::instance().is_plugin_active(layer));
}

TEST_CASE("Temperature startup and layer targets follow configured priorities",
          "[plugins][temperature]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const std::string output = export_temperature_fixture(perimeter_config({}));
    INFO(output);
    const std::vector<std::string> lines = gcode_lines(output);
    CHECK(count_exact_lines(lines, "M190 S65") == 1);
    CHECK(count_exact_lines(lines, "M191 S70") == 1);
    CHECK(count_exact_lines(lines, "M109 S215 T0") == 1);
    CHECK(count_exact_lines(lines, "M104 S220 T1") == 1);
    CHECK(count_exact_lines(lines, "M104 S200 T0") == 1);
    CHECK(count_exact_lines(lines, "M104 S205 T1") == 1);
}

TEST_CASE("Temperature regional overrides outrank filament targets",
          "[plugins][temperature]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const DynamicPrintConfig initial_config = perimeter_config({
        {"print_first_layer_temperature", "220"},
        {"print_temperature", "230"}
    });
    const std::string output = export_temperature_fixture(initial_config);
    INFO(output);
    const std::vector<std::string> lines = gcode_lines(output);
    CHECK(count_exact_lines(lines, "M104 S220 T0") == 1);
    CHECK(count_exact_lines(lines, "M109 S230 T0") == 1);
    CHECK(count_exact_lines(lines, "M104 S230 T1") == 1);
    CHECK(count_exact_lines(lines, "M109 S215 T0") == 1);
}

TEST_CASE("Temperature targets follow each regional extrusion in order",
          "[plugins][temperature]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({}));
    Print &print = prepared.print;
    configure_standard_firmware(print, 1);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.first_layer_temperature.set(std::vector<int>{215});
    config.temperature.set(std::vector<int>{200});
    config.first_layer_bed_temperature.set(std::vector<int>{0});
    config.bed_temperature.set(std::vector<int>{0});
    config.chamber_temperature.set(std::vector<int>{0});
    config.start_gcode_manual.value = true;
    config.option<ConfigOptionString>("end_gcode")->value.clear();

    PrintObject &object = print.object(0);
    LayerRegionIsland first_override = append_temperature_override_region(
        print, object.layer(0), 230);
    LayerRegionIsland second_override = append_temperature_override_region(
        print, object.layer(1), 230);
    LayerRegionIsland first_source({&object.layer(0).region(0)}, 0);
    LayerRegionIsland second_source({&object.layer(1).region(0)}, 0);
    PrintingLayerGroup &first_layer = append_temperature_layer(
        print, object.layer(0).scaled_print_z());
    append_temperature_visit(first_layer, 0, first_source, 0.0, 1.0);
    append_temperature_visit(first_layer, 0, first_override, 1.0, 2.0);
    append_temperature_visit(first_layer, 0, first_source, 2.0, 3.0);
    PrintingLayerGroup &second_layer = append_temperature_layer(
        print, object.layer(1).scaled_print_z());
    append_temperature_visit(second_layer, 0, second_source, 3.0, 4.0);
    append_temperature_visit(second_layer, 0, second_override, 4.0, 5.0);
    append_temperature_visit(second_layer, 0, second_source, 5.0, 6.0);
    run_temperature_plugins(print);
    const std::vector<std::string> lines = gcode_lines(
        export_with_firmware(print, "gcode.firmware.marlin2"));
    const size_t first_layer_marker = first_exact_line(lines, ";LAYER_CHANGE");
    REQUIRE(first_layer_marker != std::string::npos);
    const size_t second_layer_marker = first_exact_line(lines, ";LAYER_CHANGE", first_layer_marker + 1);
    REQUIRE(second_layer_marker != std::string::npos);
    const size_t first_215 = first_exact_line(lines, "M104 S215 T0", first_layer_marker);
    const size_t first_230 = first_exact_line(lines, "M104 S230 T0", first_215 + 1);
    const size_t first_215_again = first_exact_line(lines, "M104 S215 T0", first_230 + 1);
    const size_t second_200 = first_exact_line(lines, "M104 S200 T0", second_layer_marker);
    const size_t second_230 = first_exact_line(lines, "M104 S230 T0", second_200 + 1);
    const size_t second_200_again = first_exact_line(lines, "M104 S200 T0", second_230 + 1);
    REQUIRE(first_215 != std::string::npos);
    REQUIRE(first_230 != std::string::npos);
    REQUIRE(first_215_again != std::string::npos);
    REQUIRE(second_200 != std::string::npos);
    REQUIRE(second_230 != std::string::npos);
    REQUIRE(second_200_again != std::string::npos);
    CHECK(first_215 < first_230);
    CHECK(first_230 < first_215_again);
    CHECK(second_layer_marker < second_200);
    CHECK(second_200 < second_230);
    CHECK(second_230 < second_200_again);
}

TEST_CASE("Temperature first-layer regional setting overrides the filament first layer",
          "[plugins][temperature]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const std::string output = export_temperature_fixture(
        perimeter_config({{"print_first_layer_temperature", "220"}}));
    INFO(output);
    const std::vector<std::string> lines = gcode_lines(output);
    const size_t first_layer = first_exact_line(lines, ";LAYER_CHANGE");
    const size_t target = first_exact_line(lines, "M104 S220 T0");
    REQUIRE(first_layer != std::string::npos);
    REQUIRE(target != std::string::npos);
    CHECK(count_exact_lines(lines, "M104 S220 T0") == 1);
    CHECK(first_layer < target);
}

// Inspect exported targets from two source objects without relying on their position in the merged Z sequence; every configuration combination must retain the later object's initial temperature.
TEST_CASE("Temperature uses each object's source first layer at different initial heights",
          "[plugins][temperature][source-layer]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool change_tool = GENERATE(false, true);
    const bool regional_override = GENERATE(false, true);
    PreparedPerimeterPrint prepared;
    prepare_temperature_source_layers(prepared, regional_override ? perimeter_config({
        {"print_first_layer_temperature", "231"}, {"print_temperature", "241"}
    }) : perimeter_config({}));
    Print &print = prepared.print;
    configure_standard_firmware(print, 2);
    configure_temperature_profile(print);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.ooze_prevention.value = true;
    config.standby_temperature_delta.value = 0;
    config.option<ConfigOptionString>("end_gcode")->value.clear();
    LayerRegionIsland early({&print.object(0).layer(0).region(0)}, 0);
    LayerRegionIsland late({&print.object(1).layer(0).region(0)}, 0);
    LayerRegionIsland normal({&print.object(1).layer(1).region(0)}, 0);
    PrintingLayerGroup &first = append_temperature_layer(print, scale_i(0.2));
    append_temperature_visit(first, 0, early, 0., 1.);
    PrintingLayerGroup &second = append_temperature_layer(print, scale_i(0.3));
    const extruder_index_t incoming = change_tool ? 1 : 0;
    append_temperature_visit(second, incoming, late, 1., 2.);
    PrintingLayerGroup &third = append_temperature_layer(
        print, print.object(1).layer(1).scaled_print_z());
    append_temperature_visit(third, incoming, normal, 2., 3.);
    run_temperature_layer_plugin_only(print);
    const std::string output = export_with_firmware(print, "gcode.firmware.marlin2");
    INFO(output);
    const std::vector<std::string> lines = gcode_lines(output);
    const size_t first_marker = first_exact_line(lines, ";LAYER_CHANGE");
    REQUIRE(first_marker != std::string::npos);
    const size_t second_marker = first_exact_line(lines, ";LAYER_CHANGE", first_marker + 1);
    REQUIRE(second_marker != std::string::npos);
    const size_t third_marker = first_exact_line(lines, ";LAYER_CHANGE", second_marker + 1);
    REQUIRE(third_marker != std::string::npos);
    const int initial_target = regional_override ? 231 : (change_tool ? 220 : 215);
    const int normal_target = regional_override ? 241 : (change_tool ? 205 : 200);
    if (change_tool) {
        const size_t wait = first_exact_line(lines, "M109 S" + std::to_string(initial_target) + " T1", second_marker);
        REQUIRE(wait != std::string::npos);
        CHECK(wait < first_extrusion_line(lines, second_marker));
        CHECK(wait < third_marker);
    } else {
        // The same target persists across both objects' first layers, so no normal-temperature request belongs to the second group.
        CHECK(count_command_lines(lines, "M104 S" + std::to_string(normal_target), second_marker, third_marker) == 0);
        CHECK(first_exact_line(lines, "M104 S" + std::to_string(initial_target) + " T0", first_marker) < second_marker);
    }
    CHECK(first_exact_line(lines, "M104 S" + std::to_string(normal_target) + " T" + std::to_string(incoming), third_marker) != std::string::npos);
}

// Materialize mixed source visits in both orders and inspect the outgoing heater event; relative standby uses the last source's filament setting even with regional targets or a trailing synthetic root.
TEST_CASE("Temperature mixed visits derive standby from the last source extrusion",
          "[plugins][temperature][source-layer]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool last_is_first = GENERATE(false, true);
    const bool absolute_idle = GENERATE(false, true);
    PreparedPerimeterPrint prepared;
    prepare_temperature_source_layers(prepared, perimeter_config({
        {"print_first_layer_temperature", "231"}, {"print_temperature", "241"}
    }));
    Print &print = prepared.print;
    configure_standard_firmware(print, 2);
    configure_temperature_profile(print);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.ooze_prevention.value = true;
    config.standby_temperature_delta.value = -10;
    config.idle_temperature.set(std::vector<int>{absolute_idle ? 150 : -1, -1});
    config.idle_temperature.set_enabled(true);
    LayerRegionIsland early({&print.object(0).layer(0).region(0)}, 0);
    LayerRegionIsland normal({&print.object(0).layer(1).region(0)}, 0);
    LayerRegionIsland late({&print.object(1).layer(0).region(0)}, 0);
    append_temperature_visit(append_temperature_layer(print, scale_i(0.2)), 0, early, 0., 1.);
    PrintingLayerGroup &mixed = append_temperature_layer(print, scale_i(0.3));
    PrintingToolGroup &outgoing = append_temperature_visit(
        mixed, 0, last_is_first ? normal : late, 1., 2.);
    append_temperature_root(outgoing, last_is_first ? late : normal, 2., 3.);
    append_temperature_tower_root(outgoing);
    append_temperature_visit(mixed, 1, late, 3., 4.);
    run_temperature_layer_plugin_only(print);
    const PrintingToolGroup &finished = print.printing_plan()->groups[0].layers[1].tool_groups[0];
    const int expected = absolute_idle ? 150 : (last_is_first ? 205 : 190);
    // Export includes the outgoing after-events and verifies standby precedes selection of the next tool.
    const std::string output = export_with_firmware(print, "gcode.firmware.marlin2");
    INFO(output);
    const std::vector<std::string> lines = gcode_lines(output);
    const size_t standby = first_exact_line(lines, "M104 S" + std::to_string(expected) + " T0");
    REQUIRE(standby != std::string::npos);
    const size_t change = first_exact_line(lines, "T1", standby);
    REQUIRE(change != std::string::npos);
    CHECK(standby < change);
    REQUIRE(finished.extrusions.size() == 3);
    const ExtrusionPropertyHeater *last_source_target =
        finished.extrusions[1].root->get_property<ExtrusionPropertyHeater>();
    REQUIRE(last_source_target != nullptr);
    CHECK(last_source_target->temperature_C == (last_is_first ? 231 : 241));
    CHECK(finished.extrusions.back().root->get_property<ExtrusionPropertyHeater>() == nullptr);
}

// Exercise a visit with no regional source through firmware output; its incoming filament target and outgoing relative standby retain the group-layer fallback without annotating synthetic roots.
TEST_CASE("Temperature source-free visits retain group-layer filament fallback",
          "[plugins][temperature][source-layer]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({}));
    Print &print = prepared.print;
    configure_standard_firmware(print, 2);
    configure_temperature_profile(print);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.ooze_prevention.value = true;
    config.idle_temperature.set(std::vector<int>{-1, -1});
    config.idle_temperature.set_enabled(true);
    config.standby_temperature_delta.value = -10;
    config.option<ConfigOptionString>("end_gcode")->value.clear();
    LayerRegionIsland source({&print.object(0).layer(0).region(0)}, 0);
    append_temperature_visit(append_temperature_layer(print, scale_i(0.2)), 1, source, 0., 1.);
    PrintingLayerGroup &second = append_temperature_layer(print, scale_i(0.4));
    second.tool_groups.emplace_back();
    second.tool_groups.back().extruder_id = 0;
    append_temperature_tower_root(second.tool_groups.back());
    append_temperature_visit(second, 1, source, 3., 4.);
    run_temperature_layer_plugin_only(print);
    const std::string output = export_with_firmware(print, "gcode.firmware.marlin2");
    INFO(output);
    const std::vector<std::string> lines = gcode_lines(output);
    const size_t incoming = first_exact_line(lines, "M109 S200 T0");
    REQUIRE(incoming != std::string::npos);
    const size_t standby = first_exact_line(lines, "M104 S190 T0", incoming);
    REQUIRE(standby != std::string::npos);
    CHECK(incoming < standby);
    CHECK(first_exact_line(lines, "T1", standby) != std::string::npos);
    const PrintingToolGroup &synthetic = print.printing_plan()->groups[0].layers[1].tool_groups[0];
    REQUIRE(synthetic.extrusions.size() == 1);
    CHECK(synthetic.extrusions[0].root->get_property<ExtrusionPropertyHeater>() == nullptr);
}

TEST_CASE("Temperature startup does not duplicate matching start-script commands",
          "[plugins][temperature]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const std::string output = export_temperature_fixture(perimeter_config({}), false, true);
    INFO(output);
    const std::vector<std::string> lines = gcode_lines(output);
    CHECK(count_exact_lines(lines, "M190 S65") == 1);
    CHECK(count_exact_lines(lines, "M191 S70") == 1);
    CHECK(count_exact_lines(lines, "M109 S215 T0") == 1);
}

TEST_CASE("Temperature startup preheats tools first used after layer one",
          "[plugins][temperature]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const std::string output = export_late_tool_fixture(false);
    INFO(output);
    const std::vector<std::string> lines = gcode_lines(output);
    const size_t first_layer = first_exact_line(lines, ";LAYER_CHANGE");
    REQUIRE(first_layer != std::string::npos);
    const size_t preheat = first_exact_line(lines, "M104 S220 T1");
    const size_t first_tool_wait = first_exact_line(lines, "M109 S205 T1", first_layer);
    REQUIRE(preheat != std::string::npos);
    REQUIRE(first_tool_wait != std::string::npos);
    CHECK(preheat < first_layer);
    const std::string standby_output = export_late_tool_fixture(true);
    INFO(standby_output);
    const std::vector<std::string> standby_lines = gcode_lines(standby_output);
    const size_t standby_first_layer = first_exact_line(standby_lines, ";LAYER_CHANGE");
    REQUIRE(standby_first_layer != std::string::npos);
    const size_t standby_preheat = first_exact_line(standby_lines, "M104 S215 T1");
    const size_t standby_wait = first_exact_line(standby_lines, "M109 R205 T1", standby_first_layer);
    REQUIRE(standby_preheat != std::string::npos);
    REQUIRE(standby_wait != std::string::npos);
    CHECK(standby_preheat < standby_first_layer);
}

TEST_CASE("Temperature zero targets do not create heater requests",
          "[plugins][temperature]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool regional_overrides = GENERATE(false, true);
    PreparedPerimeterPrint prepared;
    const DynamicPrintConfig fixture_config = regional_overrides ? perimeter_config({
        {"print_first_layer_temperature", "0"},
        {"print_temperature", "0"}
    }) : perimeter_config({});
    prepare_cube_print(prepared, fixture_config);
    Print &print = prepared.print;
    configure_standard_firmware(print, 1);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.first_layer_temperature.set(std::vector<int>{0});
    config.temperature.set(std::vector<int>{0});
    config.first_layer_bed_temperature.set(std::vector<int>{0});
    config.bed_temperature.set(std::vector<int>{0});
    config.chamber_temperature.set(std::vector<int>{0});
    config.option<ConfigOptionString>("end_gcode")->value.clear();
    PrintObject &object = print.object(0);
    REQUIRE(object.layer_count() > 1);
    LayerRegionIsland source({&object.layer(0).region(0)}, 0);
    LayerRegionIsland second_source({&object.layer(1).region(0)}, 0);
    PrintingLayerGroup &first_layer = append_temperature_layer(
        print, object.layer(0).scaled_print_z());
    append_temperature_visit(first_layer, 0, source, 0.0, 1.0);
    PrintingLayerGroup &second_layer = append_temperature_layer(
        print, object.layer(1).scaled_print_z());
    append_temperature_visit(second_layer, 0, second_source, 1.0, 2.0);
    run_temperature_plugins(print);
    const std::string output = export_with_firmware(print, "gcode.firmware.marlin2");
    INFO(output);
    const std::vector<std::string> lines = gcode_lines(output);
    size_t heater_commands = 0;
    for (const std::string &line : lines)
        if (line.compare(0, 4, "M104") == 0 || line.compare(0, 4, "M109") == 0 ||
            line.compare(0, 4, "M140") == 0 || line.compare(0, 4, "M190") == 0 ||
            line.compare(0, 4, "M141") == 0 || line.compare(0, 4, "M191") == 0)
            ++heater_commands;
    CHECK(heater_commands == 0);
}

TEST_CASE("Temperature equal roots produce one request",
          "[plugins][temperature]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({}));
    Print &print = prepared.print;
    configure_standard_firmware(print, 1);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.first_layer_temperature.set(std::vector<int>{200});
    config.temperature.set(std::vector<int>{200});
    config.first_layer_bed_temperature.set(std::vector<int>{0});
    config.chamber_temperature.set(std::vector<int>{0});
    config.start_gcode_manual.value = true;
    PrintObject &object = print.object(0);
    LayerRegionIsland source({&object.layer(0).region(0)}, 0);
    PrintingLayerGroup &layer = append_temperature_layer(print, object.layer(0).scaled_print_z());
    append_temperature_visit(layer, 0, source, 0.0, 1.0);
    append_temperature_visit(layer, 0, source, 1.0, 2.0);
    run_temperature_plugins(print);
    const std::string output = export_with_firmware(print, "gcode.firmware.marlin2");
    INFO(output);
    CHECK(count_exact_lines(gcode_lines(output), "M104 S200 T0") == 1);
}

TEST_CASE("Temperature startup is disabled by manual start mode",
          "[plugins][temperature]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({}));
    Print &print = prepared.print;
    configure_standard_firmware(print, 1);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.first_layer_temperature.set(std::vector<int>{215});
    const bool manual_start = GENERATE(true, false);
    config.start_gcode_manual.value = manual_start;
    config.autoemit_temperature_commands.value = manual_start;
    PrintObject &object = print.object(0);
    LayerRegionIsland source({&object.layer(0).region(0)}, 0);
    PrintingLayerGroup &layer = append_temperature_layer(print, object.layer(0).scaled_print_z());
    append_temperature_visit(layer, 0, source, 0.0, 1.0);
    run_temperature_plugins(print);
    const std::string output = export_with_firmware(print, "gcode.firmware.marlin2");
    INFO(output);
    const std::vector<std::string> lines = gcode_lines(output);
    for (const std::string &line : lines) {
        CHECK(line.rfind("M109", 0) != 0);
        CHECK(line.rfind("M190", 0) != 0);
    }
}

TEST_CASE("Temperature layer plugin requests roots and waits without startup plugin",
          "[plugins][temperature]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({}));
    Print &print = prepared.print;
    configure_standard_firmware(print, 2);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.first_layer_temperature.set(std::vector<int>{215, 218});
    config.temperature.set(std::vector<int>{200, 200});
    config.first_layer_bed_temperature.set(std::vector<int>{0, 0});
    config.chamber_temperature.set(std::vector<int>{0, 0});
    config.option<ConfigOptionString>("end_gcode")->value.clear();
    PrintObject &object = print.object(0);
    LayerRegionIsland source({&object.layer(0).region(0)}, 0);
    PrintingLayerGroup &layer = append_temperature_layer(print, object.layer(0).scaled_print_z());
    append_temperature_visit(layer, 0, source, 0.0, 1.0);
    append_temperature_visit(layer, 1, source, 1.0, 2.0);
    run_temperature_plugins_without_startup(print);
    const std::vector<std::string> lines = gcode_lines(
        export_with_firmware(print, "gcode.firmware.marlin2"));
    const size_t initial_target = first_exact_line(lines, "M104 S215 T0");
    const size_t first_change = first_exact_line(lines, "T1");
    const size_t first_wait = first_exact_line(lines, "M109 S218 T1");
    size_t first_movement = std::string::npos;
    for (size_t idx = 0; idx < lines.size(); ++idx) {
        if (is_filament_move(lines[idx])) {
            first_movement = idx;
            break;
        }
    }
    size_t movement_after_change = std::string::npos;
    if (first_wait != std::string::npos)
        for (size_t idx = first_wait + 1; idx < lines.size(); ++idx) {
            if (is_filament_move(lines[idx])) {
                movement_after_change = idx;
                break;
            }
        }
    REQUIRE(initial_target != std::string::npos);
    REQUIRE(first_change != std::string::npos);
    REQUIRE(first_wait != std::string::npos);
    REQUIRE(first_movement != std::string::npos);
    REQUIRE(movement_after_change != std::string::npos);
    CHECK(count_exact_lines(lines, "M104 S215 T0") == 1);
    CHECK(count_exact_lines(lines, "M109 S218 T1") == 1);
    CHECK(initial_target < first_movement);
    CHECK(first_change < first_wait);
    CHECK(first_wait < movement_after_change);
}

TEST_CASE("Temperature tool-change waiting uses the five-degree margin",
          "[plugins][temperature]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const std::string four_degrees = export_wait_margin_fixture(204);
    const std::string five_degrees = export_wait_margin_fixture(205);
    const std::string lower = export_wait_margin_fixture(195);
    const std::string same = export_wait_margin_fixture(200);
    INFO(four_degrees);
    const std::vector<std::string> four_lines = gcode_lines(four_degrees);
    const std::vector<std::string> five_lines = gcode_lines(five_degrees);
    const std::vector<std::string> lower_lines = gcode_lines(lower);
    const std::vector<std::string> same_lines = gcode_lines(same);
    const size_t four_layer = first_exact_line(four_lines, ";LAYER_CHANGE");
    const size_t five_layer = first_exact_line(five_lines, ";LAYER_CHANGE");
    const size_t lower_layer = first_exact_line(lower_lines, ";LAYER_CHANGE");
    const size_t same_layer = first_exact_line(same_lines, ";LAYER_CHANGE");
    REQUIRE(four_layer != std::string::npos);
    REQUIRE(five_layer != std::string::npos);
    REQUIRE(lower_layer != std::string::npos);
    REQUIRE(same_layer != std::string::npos);
    CHECK(first_exact_line(four_lines, "M104 S204 T0", four_layer) != std::string::npos);
    CHECK(first_exact_line(four_lines, "M109 S204 T0", four_layer) == std::string::npos);
    INFO(five_degrees);
    CHECK(first_exact_line(five_lines, "M109 S205 T0", five_layer) != std::string::npos);
    INFO(lower);
    CHECK(first_exact_line(lower_lines, "M104 S195 T0", lower_layer) != std::string::npos);
    CHECK(first_exact_line(lower_lines, "M109 S195 T0", lower_layer) == std::string::npos);
    INFO(same);
    CHECK(first_exact_line(same_lines, "M104 S200 T0", same_layer) == std::string::npos);
    CHECK(first_exact_line(same_lines, "M109 S200 T0", same_layer) == std::string::npos);
}

TEST_CASE("Temperature startup follows start-script and heater order for both firmware families",
          "[plugins][temperature]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool semm = GENERATE(false, true);
    const std::string firmware = GENERATE(
        "gcode.firmware.marlin2", "gcode.firmware.klipper");
    const std::string output = export_startup_fixture(firmware, semm, false, false, true);
    INFO(output);
    const std::vector<std::string> lines = gcode_lines(output);
    const size_t marker = first_exact_line(lines, "; STARTUP_MARKER");
    const size_t first_layer = first_exact_line(lines, ";LAYER_CHANGE");
    const size_t first_move = first_exact_line(lines, "G1 X1 E.03326 F1200");
    REQUIRE(marker != std::string::npos);
    REQUIRE(first_layer != std::string::npos);
    REQUIRE(first_move != std::string::npos);
    if (firmware == "gcode.firmware.marlin2") {
        const size_t bed = first_exact_line(lines, "M190 S65", marker);
        const size_t chamber = first_exact_line(lines, "M191 S70", marker);
        const size_t initial = first_exact_line(lines, semm ? "M109 S220" : "M109 S220 T1", marker);
        REQUIRE(bed != std::string::npos);
        REQUIRE(chamber != std::string::npos);
        REQUIRE(initial != std::string::npos);
        CHECK(marker < bed);
        CHECK(bed < chamber);
        CHECK(chamber < initial);
        CHECK(initial < first_move);
        if (semm) {
            CHECK(count_exact_lines(lines, "M109 S220") == 1);
            CHECK(first_exact_line(lines, "M104 S220") == std::string::npos);
            for (size_t idx = 0; idx < first_layer; ++idx) {
                CHECK(lines[idx].rfind("M104 S205", 0) != 0);
                CHECK(lines[idx].rfind("M109 S205", 0) != 0);
                CHECK(lines[idx].rfind("M109 R205", 0) != 0);
            }
        } else {
            const size_t other = first_exact_line(lines, "M104 S205 T0", initial);
            REQUIRE(other != std::string::npos);
            CHECK(initial < other);
            CHECK(other < first_move);
            CHECK(count_exact_lines(lines, "M109 S220 T1") == 1);
        }
    } else {
        const size_t bed = first_exact_line(lines, "M190 S65", marker);
        const size_t initial = first_exact_line(lines, "M109 S220", marker);
        REQUIRE(bed != std::string::npos);
        REQUIRE(initial != std::string::npos);
        CHECK(marker < bed);
        CHECK(bed < initial);
        CHECK(initial < first_move);
        if (semm) {
            CHECK(count_exact_lines(lines, "M109 S220") == 1);
            for (size_t idx = 0; idx < first_layer; ++idx) {
                CHECK(lines[idx].rfind("M104 S205", 0) != 0);
                CHECK(lines[idx].rfind("M109 S205", 0) != 0);
                CHECK(lines[idx].rfind("M109 R205", 0) != 0);
                CHECK(lines[idx].rfind("SET_HEATER_TEMPERATURE", 0) != 0);
            }
        } else {
            const size_t other = first_exact_line(
                lines, "SET_HEATER_TEMPERATURE HEATER=extruder TARGET=205", initial);
            REQUIRE(other != std::string::npos);
            CHECK(initial < other);
            CHECK(other < first_move);
            CHECK(count_exact_lines(lines, "M109 S220") == 1);
            CHECK(first_exact_line(lines, "TEMPERATURE_WAIT SENSOR=extruder MINIMUM=205") ==
                  std::string::npos);
        }
    }
}

TEST_CASE("Temperature startup applies idle standby to later physical tools",
          "[plugins][temperature]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const std::string output = export_startup_fixture(
        "gcode.firmware.marlin2", false, true, false, true);
    INFO(output);
    const std::vector<std::string> lines = gcode_lines(output);
    const size_t standby = first_exact_line(lines, "M104 S150 T0");
    const size_t first_layer = first_exact_line(lines, ";LAYER_CHANGE");
    REQUIRE(standby != std::string::npos);
    REQUIRE(first_layer != std::string::npos);
    CHECK(count_exact_lines(lines, "M104 S150 T0") == 1);
    CHECK(standby < first_layer);
}

TEST_CASE("Temperature startup switches independently suppress automatic startup events",
          "[plugins][temperature]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const std::string manual_output = export_startup_fixture(
        "gcode.firmware.marlin2", false, false, true, true);
    const std::string autoemit_output = export_startup_fixture(
        "gcode.firmware.marlin2", false, false, false, false);
    const std::vector<std::string> manual_lines = gcode_lines(manual_output);
    const std::vector<std::string> autoemit_lines = gcode_lines(autoemit_output);
    INFO(manual_output);
    CHECK(first_exact_line(manual_lines, "M190 S65") == std::string::npos);
    CHECK(first_exact_line(manual_lines, "M191 S70") == std::string::npos);
    CHECK(first_exact_line(manual_lines, "M109 S220 T1") == std::string::npos);
    CHECK(first_exact_line(manual_lines, "M104 S205 T0") == std::string::npos);
    INFO(autoemit_output);
    CHECK(first_exact_line(autoemit_lines, "M190 S65") == std::string::npos);
    CHECK(first_exact_line(autoemit_lines, "M191 S70") == std::string::npos);
    CHECK(first_exact_line(autoemit_lines, "M109 S220 T1") == std::string::npos);
    CHECK(first_exact_line(autoemit_lines, "M104 S205 T0") == std::string::npos);
}

TEST_CASE("Temperature ooze prevention lowers the leaving tool before the change",
          "[plugins][temperature]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const std::string output = [&]() {
        PreparedPerimeterPrint prepared;
        prepare_cube_print(prepared, perimeter_config({}));
        Print &print = prepared.print;
        configure_standard_firmware(print, 2);
        configure_temperature_profile(print);
        PrintConfig &config = const_cast<PrintConfig &>(print.config());
        config.ooze_prevention.value = true;
        config.idle_temperature.set(std::vector<int>{150, -1});
        config.idle_temperature.set_enabled(true);
        PrintObject &object = print.object(0);
        REQUIRE(object.layer_count() > 1);
        LayerRegionIsland first_source({&object.layer(0).region(0)}, 0);
        LayerRegionIsland second_source({&object.layer(1).region(0)}, 0);
        PrintingLayerGroup &first_layer = append_temperature_layer(
            print, object.layer(0).scaled_print_z());
        append_temperature_visit(first_layer, 0, first_source, 0.0, 1.0);
        append_temperature_visit(first_layer, 1, first_source, 1.0, 2.0);
        PrintingLayerGroup &second_layer = append_temperature_layer(
            print, object.layer(1).scaled_print_z());
        append_temperature_visit(second_layer, 1, second_source, 2.0, 3.0);
        append_temperature_visit(second_layer, 0, second_source, 3.0, 4.0);
        run_temperature_plugins(print);
        return export_with_firmware(print, "gcode.firmware.marlin2");
    }();
    INFO(output);
    const std::vector<std::string> lines = gcode_lines(output);
    const size_t standby = first_exact_line(lines, "M104 S150 T0");
    const size_t change = first_exact_line(lines, "T1");
    const size_t wait = first_exact_line(lines, "M109 S220 T1", change);
    REQUIRE(standby != std::string::npos);
    REQUIRE(change != std::string::npos);
    REQUIRE(wait != std::string::npos);
    CHECK(standby < change);
    CHECK(change < wait);
}

TEST_CASE("Temperature tool-change wait follows the tool selection",
          "[plugins][temperature]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const std::string output = export_tool_change_order_fixture();
    INFO(output);
    const std::vector<std::string> lines = gcode_lines(output);
    const size_t standby = first_exact_line(lines, "M104 S150 T1");
    const size_t change = first_exact_line(lines, "T0");
    const size_t wait = first_exact_line(lines, "M109 S215 T0", change);
    const size_t movement = first_exact_line(lines, "G1 X2 E.03326", wait);
    REQUIRE(standby != std::string::npos);
    REQUIRE(change != std::string::npos);
    REQUIRE(wait != std::string::npos);
    REQUIRE(movement != std::string::npos);
    CHECK(standby < change);
    CHECK(change < wait);
    CHECK(wait < movement);
}

TEST_CASE("Temperature idle target zero is emitted at a tool change",
          "[plugins][temperature]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const std::string output = export_tool_change_order_fixture(0);
    INFO(output);
    const std::vector<std::string> lines = gcode_lines(output);
    const size_t standby = first_exact_line(lines, "M104 S0 T1");
    const size_t change = first_exact_line(lines, "T0");
    REQUIRE(standby != std::string::npos);
    REQUIRE(change != std::string::npos);
    CHECK(standby < change);
}

TEST_CASE("Temperature second layer updates the inactive tool and bed once",
          "[plugins][temperature]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const std::string output = export_temperature_fixture(perimeter_config({}));
    INFO(output);
    const std::vector<std::string> lines = gcode_lines(output);
    const size_t first_layer = first_exact_line(lines, ";LAYER_CHANGE");
    REQUIRE(first_layer != std::string::npos);
    const size_t second_layer = first_exact_line(lines, ";LAYER_CHANGE", first_layer + 1);
    REQUIRE(second_layer != std::string::npos);
    const size_t tool_target = first_exact_line(lines, "M104 S200 T0", second_layer);
    const size_t bed_target = first_exact_line(lines, "M140 S55", second_layer);
    REQUIRE(tool_target != std::string::npos);
    REQUIRE(bed_target != std::string::npos);
    CHECK(tool_target < bed_target);
    CHECK(count_exact_lines(lines, "M140 S55") == 1);
}

TEST_CASE("Temperature second layer skips inactive tools in SEMM and ooze prevention",
          "[plugins][temperature]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool semm = GENERATE(false, true);
    const bool ooze = semm ? false : true;
    const std::string output = export_second_layer_policy_fixture(semm, ooze);
    INFO(output);
    const std::vector<std::string> lines = gcode_lines(output);
    const size_t first_layer = first_exact_line(lines, ";LAYER_CHANGE");
    REQUIRE(first_layer != std::string::npos);
    const size_t second_layer = first_exact_line(lines, ";LAYER_CHANGE", first_layer + 1);
    REQUIRE(second_layer != std::string::npos);
    for (size_t idx = second_layer; idx < lines.size(); ++idx)
        CHECK(lines[idx].rfind("M104 S200", 0) != 0);
    size_t first_heater = std::string::npos;
    for (size_t idx = second_layer; idx < lines.size(); ++idx) {
        if (lines[idx].rfind("M104", 0) == 0 || lines[idx].rfind("M109", 0) == 0) {
            first_heater = idx;
            break;
        }
    }
    REQUIRE(first_heater != std::string::npos);
    CHECK(lines[first_heater].rfind("M104 S205", 0) == 0);
    CHECK(count_exact_lines(lines, "M140 S45") == 1);
}

TEST_CASE("Temperature plugin waits for SEMM changes without a tower",
          "[plugins][temperature]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({}));
    Print &print = prepared.print;
    configure_standard_firmware(print, 2);
    configure_temperature_profile(print);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.single_extruder_multi_material.value = true;
    config.first_layer_temperature.set(std::vector<int>{215, 220});
    config.temperature.set(std::vector<int>{200, 230});
    config.first_layer_bed_temperature.set(std::vector<int>{0, 0});
    config.chamber_temperature.set(std::vector<int>{0, 0});
    config.option<ConfigOptionString>("end_gcode")->value.clear();
    PrintObject &object = print.object(0);
    LayerRegionIsland source({&object.layer(0).region(0)}, 0);
    PrintingLayerGroup &layer = append_temperature_layer(print, object.layer(0).scaled_print_z());
    append_temperature_visit(layer, 0, source, 0.0, 1.0);
    append_temperature_visit(layer, 1, source, 1.0, 2.0);
    run_temperature_plugins(print);
    const std::string output = export_with_firmware(print, "gcode.firmware.marlin2");
    INFO(output);
    const std::vector<std::string> lines = gcode_lines(output);
    const size_t change = first_exact_line(lines, "T1");
    const size_t wait = first_exact_line(lines, "M109 S220");
    const size_t movement = first_exact_line(lines, "G1 X2 E10.03326 F1200");
    REQUIRE(change != std::string::npos);
    REQUIRE(wait != std::string::npos);
    REQUIRE(movement != std::string::npos);
    CHECK(change < wait);
    CHECK(wait < movement);
    CHECK(count_exact_lines(lines, "M109 S220") == 1);
    CHECK(first_exact_line(lines, "M109 S220 T1") == std::string::npos);
}

TEST_CASE("Temperature SEMM tower changes reset state before object roots",
          "[plugins][temperature]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({}));
    Print &print = prepared.print;
    configure_standard_firmware(print, 2);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.single_extruder_multi_material.value = true;
    config.first_layer_temperature.set(std::vector<int>{215, 215});
    config.temperature.set(std::vector<int>{200, 215});
    config.first_layer_bed_temperature.set(std::vector<int>{0, 0});
    config.chamber_temperature.set(std::vector<int>{0, 0});
    config.option<ConfigOptionString>("end_gcode")->value.clear();

    PrintObject &object = print.object(0);
    LayerRegionIsland first_source({&object.layer(0).region(0)}, 0);

    SECTION("matching target after the tower is requested") {
        LayerRegionIsland second_source({&object.layer(1).region(0)}, 0);
        PrintingLayerGroup &first_layer = append_temperature_layer(
            print, object.layer(0).scaled_print_z());
        append_temperature_visit(first_layer, 0, first_source, 0.0, 1.0);
        PrintingLayerGroup &second_layer = append_temperature_layer(
            print, object.layer(1).scaled_print_z());
        PrintingToolGroup &tower_visit = second_layer.tool_groups.emplace_back();
        tower_visit.extruder_id = 1;
        append_temperature_tower_root(tower_visit);
        append_temperature_root(tower_visit, second_source, 1.0, 2.0);
        run_temperature_layer_plugin_only(print);

        REQUIRE(tower_visit.extrusions.size() == 2);
        CHECK(count_heater_properties(tower_visit.events.before()) == 0);
        CHECK(tower_visit.extrusions[0].root->get_property<ExtrusionPropertyHeater>() == nullptr);
        REQUIRE(tower_visit.extrusions[1].root->get_property<ExtrusionPropertyHeater>() != nullptr);
        CHECK(tower_visit.extrusions[1].root->get_property<ExtrusionPropertyHeater>()->temperature_C == 215);
    }

    SECTION("regional override remains applied after the tower") {
        LayerRegionIsland override_source = append_temperature_override_region(print, object.layer(0), 230);
        PrintingLayerGroup &first_layer = append_temperature_layer(
            print, object.layer(0).scaled_print_z());
        append_temperature_visit(first_layer, 0, first_source, 0.0, 1.0);
        PrintingLayerGroup &second_layer = append_temperature_layer(
            print, object.layer(1).scaled_print_z());
        PrintingToolGroup &tower_visit = second_layer.tool_groups.emplace_back();
        tower_visit.extruder_id = 1;
        append_temperature_tower_root(tower_visit);
        append_temperature_root(tower_visit, override_source, 1.0, 2.0);
        run_temperature_layer_plugin_only(print);

        REQUIRE(tower_visit.extrusions.size() == 2);
        CHECK(count_heater_properties(tower_visit.events.before()) == 0);
        CHECK(tower_visit.extrusions[0].root->get_property<ExtrusionPropertyHeater>() == nullptr);
        REQUIRE(tower_visit.extrusions[1].root->get_property<ExtrusionPropertyHeater>() != nullptr);
        CHECK(tower_visit.extrusions[1].root->get_property<ExtrusionPropertyHeater>()->temperature_C == 230);
    }
}

TEST_CASE("Temperature SEMM tower visit without a change updates the second-layer root",
          "[plugins][temperature]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({}));
    Print &print = prepared.print;
    configure_standard_firmware(print, 1);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.single_extruder_multi_material.value = true;
    config.first_layer_temperature.set(std::vector<int>{215});
    config.temperature.set(std::vector<int>{200});
    config.first_layer_bed_temperature.set(std::vector<int>{0});
    config.chamber_temperature.set(std::vector<int>{0});
    config.option<ConfigOptionString>("end_gcode")->value.clear();

    PrintObject &object = print.object(0);
    LayerRegionIsland first_source({&object.layer(0).region(0)}, 0);
    LayerRegionIsland second_source({&object.layer(1).region(0)}, 0);
    PrintingLayerGroup &first_layer = append_temperature_layer(
        print, object.layer(0).scaled_print_z());
    append_temperature_visit(first_layer, 0, first_source, 0.0, 1.0);
    PrintingLayerGroup &second_layer = append_temperature_layer(
        print, object.layer(1).scaled_print_z());
    PrintingToolGroup &visit = second_layer.tool_groups.emplace_back();
    visit.extruder_id = 0;
    append_temperature_tower_root(visit);
    append_temperature_root(visit, second_source, 1.0, 2.0);
    run_temperature_layer_plugin_only(print);

    REQUIRE(visit.extrusions.size() == 2);
    CHECK(visit.extrusions[1].root->get_property<ExtrusionPropertyHeater>() != nullptr);
    CHECK(visit.extrusions[1].root->get_property<ExtrusionPropertyHeater>()->temperature_C == 200);
}

// Export a real two-object multi-nozzle tower five times and check only ordering properties stable across runs.
TEST_CASE("Temperature real tower export keeps waits before first tool use",
          "[plugins][temperature][wipe-tower]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    for (int iteration = 0; iteration < 5; ++iteration) {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.set_num_extruders(2);
        config.set_deserialize_strict({
            {"wipe_tower", "1"}, {"wipe_tower_x", "40"}, {"wipe_tower_y", "60"},
            {"wipe_tower_width", "20"}, {"wipe_tower_rotation_angle", "0"},
            {"wipe_tower_brim_width", "0"}, {"wiping_volumes_matrix", "0,8,8,0"},
            {"filament_minimal_purge_on_wipe_tower", "2,2"},
            {"filament_diameter", "1.75,1.75"}, {"extrusion_multiplier", "1,1"},
            {"perimeter_extruder", "1"}, {"infill_extruder", "1"},
            {"solid_infill_extruder", "1"}, {"temperature", "200,240"},
            {"first_layer_temperature", "210,250"}, {"bed_temperature", "0,0"},
            {"first_layer_bed_temperature", "0,0"}, {"chamber_temperature", "0,0"},
            {"single_extruder_multi_material", "0"}, {"filament_enable_toolchange_temp", "0,0"},
            {"layer_height", "0.3"}, {"fill_density", "10%"},
            {"step_perimeter_plugin", "perimeter.generator.classic"},
            {"step_surface_generation_plugin", "surface.initial_typed_surface_builder"},
            {"step_wipetower_plugin", "wipe_tower.manager.default"}, {"end_gcode", ""}});
        Print print;
        Model model;
        Test::init_print({Test::TestMesh::cube_20x20x20, Test::TestMesh::cube_20x20x20},
                         print, model, config);
        for (const char *key : {"perimeter_extruder", "infill_extruder", "solid_infill_extruder"})
            model.objects()[1].config.set_deserialize_strict(key, "2");
        print.apply(model, config);
        REQUIRE_NOTHROW(print.process());
        const boost::filesystem::path path = temporary_gcode_path();
        remove_output_pair(path);
        REQUIRE_NOTHROW(Orchestrator::instance().export_gcode(print, path.string(), nullptr));
        const std::string output = read_text_file(path);
        remove_output_pair(path);
        INFO(output);
        const std::vector<std::string> lines = gcode_lines(output);
        std::array<size_t, 2> first_selection{std::string::npos, std::string::npos};
        std::array<size_t, 2> first_wait{std::string::npos, std::string::npos};
        for (extruder_index_t tool = 0; tool < 2; ++tool) {
            const std::string selection = "T" + std::to_string(tool);
            first_selection[tool] = first_exact_line(lines, selection);
            REQUIRE(first_selection[tool] != std::string::npos);
            for (size_t idx = first_selection[tool] + 1; idx < lines.size(); ++idx) {
                if (is_wait_for_tool(lines[idx], tool)) {
                    first_wait[tool] = idx;
                    break;
                }
            }
            REQUIRE(first_wait[tool] != std::string::npos);
            for (size_t idx = first_selection[tool] + 1; idx < first_wait[tool]; ++idx)
                CHECK_FALSE(is_filament_move(lines[idx]));
        }
        for (size_t idx = 1; idx < lines.size(); ++idx) {
            const bool previous_heater = lines[idx - 1].compare(0, 4, "M104") == 0 ||
                lines[idx - 1].compare(0, 4, "M109") == 0;
            const bool current_heater = lines[idx].compare(0, 4, "M104") == 0 ||
                lines[idx].compare(0, 4, "M109") == 0;
            if (previous_heater && current_heater)
                CHECK(lines[idx] != lines[idx - 1]);
        }
    }
}

// Export a real cube whose upper floors are height ranges with their own print_temperature, like the temperature calibration tower, and check that each floor changes the nozzle temperature once, at its first layer, without any wait.
TEST_CASE("Height-range print_temperature drives the nozzle temperature by floor",
          "[plugins][temperature]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"layer_height", "0.2"}, {"first_layer_height", "0.2"},
        {"temperature", "195"}, {"first_layer_temperature", "200"},
        {"skirts", "0"}, {"start_gcode", ""}, {"end_gcode", ""},
        {"step_perimeter_plugin", "perimeter.generator.classic"},
        {"step_surface_generation_plugin", "surface.initial_typed_surface_builder"}});
    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
    ModelObject &object = model.objects()[0];
    // The object holds the bottom floor's temperature, and the first layer its own.
    object.config.set_key_value("print_temperature", new ConfigOptionInt(220));
    object.config.set_key_value("print_first_layer_temperature", new ConfigOptionInt(205));
    add_floor_height_range(object, 5., 10., 210);
    add_floor_height_range(object, 10., 15., 200);
    add_floor_height_range(object, 15., 20., 190);
    print.apply(model, config);
    REQUIRE_NOTHROW(print.process());
    const boost::filesystem::path path = temporary_gcode_path();
    remove_output_pair(path);
    REQUIRE_NOTHROW(Orchestrator::instance().export_gcode(print, path.string(), nullptr));
    const std::string output = read_text_file(path);
    remove_output_pair(path);
    INFO(output);
    const std::vector<std::string> lines = gcode_lines(output);

    // Startup waits for the filament first-layer temperature, and nothing else ever waits.
    const size_t first_layer = first_exact_line(lines, ";LAYER_CHANGE");
    REQUIRE(first_layer != std::string::npos);
    const size_t startup = first_command_line(lines, "M109 S200");
    REQUIRE(startup != std::string::npos);
    CHECK(startup < first_layer);
    CHECK(count_command_lines(lines, "M109") == 1);

    // The first layer takes the object's first-layer temperature without waiting.
    const size_t second_layer_z = first_exact_line(lines, ";Z:0.4");
    REQUIRE(second_layer_z != std::string::npos);
    const size_t first_layer_change = first_command_line(lines, "M104 S205", first_layer);
    REQUIRE(first_layer_change != std::string::npos);
    CHECK(first_layer_change < second_layer_z);
    const size_t first_layer_extrusion = first_extrusion_line(lines, first_layer);
    REQUIRE(first_layer_extrusion != std::string::npos);
    CHECK(first_layer_change < first_layer_extrusion);
    CHECK(count_command_lines(lines, "M104 S205") == 1);

    // The second layer takes the object's bottom-floor temperature before its first extrusion.
    const size_t bottom_floor_change = first_command_line(lines, "M104 S220", second_layer_z);
    const size_t bottom_floor_extrusion = first_extrusion_line(lines, second_layer_z);
    REQUIRE(bottom_floor_change != std::string::npos);
    REQUIRE(bottom_floor_extrusion != std::string::npos);
    CHECK(bottom_floor_change < bottom_floor_extrusion);
    CHECK(count_command_lines(lines, "M104 S220") == 1);

    // A range applies from the first layer whose middle is inside it: the layer at Z 5 (middle 4.9) keeps 220, the layer at Z 5.2 (middle 5.1) changes to 210.
    check_floor_change(lines, ";Z:5", ";Z:5.2", "M104 S210");
    check_floor_change(lines, ";Z:10", ";Z:10.2", "M104 S200");
    check_floor_change(lines, ";Z:15", ";Z:15.2", "M104 S190");
    CHECK(count_command_lines(lines, "M104") == 5);
}

TEST_CASE("Temperature sequential groups reuse first-layer targets",
          "[plugins][temperature]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({}));
    Print &print = prepared.print;
    configure_standard_firmware(print, 1);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.first_layer_temperature.set(std::vector<int>{215});
    config.temperature.set(std::vector<int>{200});
    config.first_layer_bed_temperature.set(std::vector<int>{0});
    config.bed_temperature.set(std::vector<int>{0});
    config.chamber_temperature.set(std::vector<int>{0});
    config.start_gcode_manual.value = true;

    PrintObject &object = print.object(0);
    REQUIRE(object.layer_count() > 1);
    LayerRegionIsland first_source({&object.layer(0).region(0)}, 0);
    LayerRegionIsland second_source({&object.layer(1).region(0)}, 0);
    PrintingLayerGroup &first_group_first_layer = append_temperature_layer(
        print, object.layer(0).scaled_print_z());
    append_temperature_visit(first_group_first_layer, 0, first_source, 0.0, 1.0);
    PrintingLayerGroup &first_group_second_layer = append_temperature_layer(
        print, object.layer(1).scaled_print_z());
    append_temperature_visit(first_group_second_layer, 0, second_source, 1.0, 2.0);
    PrintingLayerGroup &second_group_first_layer = append_temperature_group_layer(
        print, object.layer(0).scaled_print_z());
    append_temperature_visit(second_group_first_layer, 0, first_source, 2.0, 3.0);
    PrintingLayerGroup &second_group_second_layer = append_temperature_layer(
        print, object.layer(1).scaled_print_z());
    append_temperature_visit(second_group_second_layer, 0, second_source, 3.0, 4.0);
    run_temperature_plugins(print);
    const std::string output = export_with_firmware(print, "gcode.firmware.marlin2");
    INFO(output);
    const std::vector<std::string> lines = gcode_lines(output);
    CHECK(count_exact_lines(lines, "M104 S215 T0") == 2);
    CHECK(count_exact_lines(lines, "M104 S200 T0") == 2);
}
