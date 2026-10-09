#include <catch2/catch.hpp>

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
#include "libslic3r/Api/plugin/c/steps/slic3r_step_gcode.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_seam_placer.h"
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
#include "libslic3r/Steps/StepReadGeneratedGcode.hpp"
#include "test_data.hpp"

/*
PrintingPlan file-writer tests verify sequential scope traversal, firmware boundary calls and atomic publication.
Shared plan construction and output helpers live in gcode_test_helpers so this file keeps only component-specific behavior.
*/

namespace {
using namespace Slic3r;
using namespace Slic3r::Printing;
using namespace Slic3r::Test::GCode;

static_assert(std::is_same_v<decltype(run_ctx_generate_gcode{}.plan),
                             const printing_plan_handle *>);
static_assert(std::is_constructible_v<slic3r_api::ConstPrintingPlan,
                                      const printing_plan_handle *>);
static_assert(!std::is_constructible_v<slic3r_api::PrintingPlan,
                                       const printing_plan_handle *>);

struct FirmwareRecorder
{
    std::vector<std::string> calls;
    uint32_t destruction_count = 0;
    bool throw_on_extrusion = false;
    std::vector<bool> roots;
    std::vector<bool> region_islands;
    std::vector<raw_extrusion_role> roles;
    std::vector<uint32_t> printing_catalog_ids;
};

class RecordingFirmwareSession final : public slic3r_api::GCodeFirmwareSession
{
public:
    explicit RecordingFirmwareSession(FirmwareRecorder &recorder) : m_recorder(recorder) {}
    ~RecordingFirmwareSession() override { ++m_recorder.destruction_count; }

    std::string begin_print(const slic3r_api::Print &,
                            const slic3r_api::ConstPrintingPlan &plan) override
    {
        m_recorder.calls.emplace_back("begin_print");
        return std::string("begin_print\n") + (plan.events().has_before() ? write_event(plan.events().before()) : std::string());
    }

    std::string begin_group(const slic3r_api::ConstPrintingGroup &) override
    {
        m_recorder.calls.emplace_back("begin_group");
        return "begin_group\n";
    }

    std::string begin_layer(const slic3r_api::ConstPrintingLayerGroup &) override
    {
        m_recorder.calls.emplace_back("begin_layer");
        return "begin_layer\n";
    }

    std::string begin_tool_group(const slic3r_api::ConstPrintingToolGroup &) override
    {
        m_recorder.calls.emplace_back("begin_tool_group");
        return "begin_tool_group\n";
    }

    std::string write_extrusion(const slic3r_api::ConstPrintingExtrusion &extrusion) override
    {
        if (m_recorder.throw_on_extrusion)
            throw std::runtime_error("test firmware extrusion failure");

        m_recorder.calls.emplace_back("write_extrusion");
        m_recorder.roots.push_back(extrusion.root().valid());
        m_recorder.region_islands.push_back(extrusion.region_island().valid());
        m_recorder.roles.push_back(extrusion.role());
        m_recorder.printing_catalog_ids.push_back(extrusion.printing_catalog_id());
        return "write_extrusion\n";
    }

    std::string write_event(const slic3r_api::ExtrusionEntity &) override
    {
        m_recorder.calls.emplace_back("write_event");
        return "write_event\n";
    }

    std::string end_tool_group() override
    {
        m_recorder.calls.emplace_back("end_tool_group");
        return "end_tool_group\n";
    }

    std::string end_layer() override
    {
        m_recorder.calls.emplace_back("end_layer");
        return "end_layer\n";
    }

    std::string end_group() override
    {
        m_recorder.calls.emplace_back("end_group");
        return "end_group\n";
    }

    std::string end_print() override
    {
        m_recorder.calls.emplace_back("end_print");
        return "end_print\n";
    }

private:
    FirmwareRecorder &m_recorder;
};

/* Build printable integration geometry with explicit viewer marker attributes. */
std::unique_ptr<ExtrusionPath> make_integration_path(const ArcPolyline &polyline,
                                                     ExtrusionRole role,
                                                     float width,
                                                     float height)
{
    std::unique_ptr<ExtrusionPath> path(new ExtrusionPath(
        polyline,
        ExtrusionAttributes(role, ExtrusionFlow(0.08, width, height)),
        nullptr,
        true));
    path->add_property(ExtrusionPropertySpeed(20.f, 500.f));
    if (role == ExtrusionRole::ExternalPerimeter)
        path->get_or_add_property<slic3r_api::EPropertyPerimeter>().shell_count(0);
    return path;
}

} // namespace

TEST_CASE("STEP_GCODE PrintingPlan writer is the default and owns the printer UI slot", "[plugins][gcode][default-writer]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Orchestrator &orchestrator = Orchestrator::instance();
    Print print;

    const std::vector<Plugin *> gcode_plugins = orchestrator.get_active_plugins_for_step(STEP_GCODE);
    REQUIRE(gcode_plugins.size() == 1);
    CHECK(gcode_plugins.front()->get_id() == "gcode.printing_plan_file_writer");
    CHECK(orchestrator.get_plugin("gcode.legacy") == nullptr);

    Plugin *selected = Steps::selected_or_active_plugin_for_step(
        orchestrator, STEP_GCODE, &print.full_print_config());
    REQUIRE(selected != nullptr);
    CHECK(selected->get_id() == "gcode.printing_plan_file_writer");
    const ConfigOptionDef *definition = PrintConfigDef::instance().get("step_gcode_plugin");
    REQUIRE(definition != nullptr);
    CHECK(definition->option_preset_type == RAW_PRESET_TYPE_FFF_PRINTER);
    CHECK(PrintConfigDef::instance().option_keys(RAW_PRESET_TYPE_FFF_PRINTER).count(
              "step_gcode_plugin") == 1);

    const std::vector<Orchestrator::PluginUiFragment> printer_fragments =
        orchestrator.ui_fragments_for_file("printer_fff.ui");
    bool found_printer_fragment = false;
    for (const Orchestrator::PluginUiFragment &fragment : printer_fragments) {
        if (fragment.fragment_id != "step_gcode_plugin")
            continue;
        found_printer_fragment = true;
        CHECK(fragment.content.find("setting:insert$beforesetting$gcode_flavor:step_gcode_plugin") !=
              std::string::npos);
    }
    CHECK(found_printer_fragment);

    // Merge the insertion rule against a minimal printer page to verify the
    // user-visible order, not only the fragment registration.
    const std::string merged_printer_layout = orchestrator.merged_ui_layout(
        "printer_fff.ui",
        "page:General:printer\n"
        "group:Firmware\n"
        "\tsetting:gcode_flavor\n");
    const size_t gcode_selector_pos = merged_printer_layout.find("setting:step_gcode_plugin");
    const size_t flavor_pos = merged_printer_layout.find("setting:gcode_flavor");
    const size_t firmware_pos = merged_printer_layout.find("setting:gcode_firmware_plugin");
    REQUIRE(gcode_selector_pos != std::string::npos);
    REQUIRE(flavor_pos != std::string::npos);
    REQUIRE(firmware_pos != std::string::npos);
    CHECK(gcode_selector_pos < firmware_pos);
    CHECK(firmware_pos < flavor_pos);
    CHECK(gcode_selector_pos < flavor_pos);

    const std::vector<Orchestrator::PluginUiFragment> print_fragments =
        orchestrator.ui_fragments_for_file("print.ui");
    for (const Orchestrator::PluginUiFragment &fragment : print_fragments)
        CHECK(fragment.fragment_id != "step_gcode_plugin");
}
TEST_CASE("STEP_GCODE default writer rejects an invalid plan without publishing output", "[plugins][gcode][default-writer]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Orchestrator &orchestrator = Orchestrator::instance();
    orchestrator.reset_plugin_cancel();
    Print print;
    print.mutable_printing_plan().groups.emplace_back().layers.emplace_back().tool_groups.emplace_back().extrusions.emplace_back();
    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);

    CHECK_THROWS_AS(Steps::StepGenerateGcode::run_step(
                        orchestrator, print, output_path.string()), RuntimeError);
    orchestrator.reset_plugin_cancel();
    CHECK_FALSE(boost::filesystem::exists(output_path));
    CHECK_FALSE(boost::filesystem::exists(output_path.string() + ".tmp"));
}
TEST_CASE("Firmware C++ adapter validates tables and destroys its session once",
          "[plugins][gcode][firmware]")
{
    FirmwareRecorder recorder;
    {
        FirmwareInstanceOwner owner(slic3r_api::make_gcode_firmware_instance(
            std::unique_ptr<slic3r_api::GCodeFirmwareSession>(new RecordingFirmwareSession(recorder))));
        CHECK_NOTHROW(slic3r_api::GCodeFirmwareView(&owner.instance()));

        raw_gcode_firmware_instance truncated_instance = owner.instance();
        truncated_instance.struct_size = 0;
        CHECK_THROWS_AS(slic3r_api::GCodeFirmwareView(&truncated_instance), std::invalid_argument);

        raw_gcode_firmware_vtable truncated = *owner.instance().vtable;
        truncated.struct_size = 0;
        raw_gcode_firmware_instance invalid_size = owner.instance();
        invalid_size.vtable = &truncated;
        CHECK_THROWS_AS(slic3r_api::GCodeFirmwareView(&invalid_size), std::invalid_argument);

        raw_gcode_firmware_vtable missing_callback = *owner.instance().vtable;
        missing_callback.write_event = nullptr;
        raw_gcode_firmware_instance invalid_callback = owner.instance();
        invalid_callback.vtable = &missing_callback;
        CHECK_THROWS_AS(slic3r_api::GCodeFirmwareView(&invalid_callback), std::invalid_argument);
    }
    CHECK(recorder.destruction_count == 1);
}
TEST_CASE("PrintingPlan file writer serializes populated scope events inside their boundaries",
          "[plugins][gcode][firmware][printing][plan][events]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    FirmwareRecorder recorder;
    FirmwareInstanceOwner firmware(slic3r_api::make_gcode_firmware_instance(
        std::unique_ptr<slic3r_api::GCodeFirmwareSession>(new RecordingFirmwareSession(recorder))));
    Print print;
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    PrintingGroup &group = plan.groups.front();
    group.layers.emplace_back();
    PrintingLayerGroup &layer = group.layers.front();
    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool_group = layer.tool_groups.front();
    const LayerRegionIsland *fake_region =
        reinterpret_cast<const LayerRegionIsland *>(uintptr_t(0x3456));
    append_empty_extrusion(tool_group, ExtrusionRole::None, 0, fake_region);

    // One child makes each fixed event root observable without depending on a
    // specific event property. The recording firmware marks every callback.
    ExtrusionEntity event(true);
    plan.events.append_before(event);
    plan.events.append_after(event);
    group.events.append_before(event);
    group.events.append_after(event);
    layer.events.append_before(event);
    layer.events.append_after(event);
    tool_group.events.append_before(event);
    tool_group.events.append_after(event);

    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    try {
        run_file_writer_with_firmware(print, output_path, firmware.instance());

        const std::vector<std::string> expected_calls{
            "begin_print", "write_event",
            "begin_group", "write_event",
            "begin_layer", "write_event",
            "begin_tool_group", "write_event",
            "write_extrusion",
            "write_event", "end_tool_group",
            "write_event", "end_layer",
            "write_event", "end_group",
            "write_event", "end_print"
        };
        CHECK(recorder.calls == expected_calls);
        CHECK(read_text_file(output_path) ==
              "begin_print\nwrite_event\n"
              "begin_group\nwrite_event\n"
              "begin_layer\nwrite_event\n"
              "begin_tool_group\nwrite_event\n"
              "write_extrusion\n"
              "write_event\nend_tool_group\n"
              "write_event\nend_layer\n"
              "write_event\nend_group\n"
              "write_event\nend_print\n");
    } catch (...) {
        remove_output_pair(output_path);
        throw;
    }
    remove_output_pair(output_path);
}
TEST_CASE("PrintingPlan file writer serializes every firmware boundary in order",
          "[plugins][gcode][firmware]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    FirmwareRecorder recorder;
    FirmwareInstanceOwner firmware(slic3r_api::make_gcode_firmware_instance(
        std::unique_ptr<slic3r_api::GCodeFirmwareSession>(new RecordingFirmwareSession(recorder))));
    Print print;
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.front().layers.front();
    layer.tool_groups.emplace_back();
    layer.tool_groups.front().extruder_id = 2;
    const LayerRegionIsland *fake_region =
        reinterpret_cast<const LayerRegionIsland *>(uintptr_t(0x2345));
    append_empty_extrusion(
        layer.tool_groups.front(), ExtrusionRole::Perimeter, 7, fake_region);
    append_empty_extrusion(
        layer.tool_groups.front(), ExtrusionRole::None, 0, fake_region);

    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    try {
        run_file_writer_with_firmware(print, output_path, firmware.instance());

        const std::vector<std::string> expected_calls{
            "begin_print",
            "begin_group",
            "begin_layer",
            "begin_tool_group",
            "write_extrusion",
            "write_extrusion",
            "end_tool_group",
            "end_layer",
            "end_group",
            "end_print"
        };
        CHECK(recorder.calls == expected_calls);
        CHECK(read_text_file(output_path) ==
              "begin_print\n"
              "begin_group\n"
              "begin_layer\n"
              "begin_tool_group\n"
              "write_extrusion\n"
              "write_extrusion\n"
              "end_tool_group\n"
              "end_layer\n"
              "end_group\n"
              "end_print\n");
        REQUIRE(recorder.roots.size() == 2);
        CHECK(recorder.roots[0]);
        CHECK(recorder.roots[1]);
        REQUIRE(recorder.region_islands.size() == 2);
        CHECK(recorder.region_islands[0]);
        CHECK(recorder.region_islands[1]);
        REQUIRE(recorder.roles.size() == 2);
        CHECK(recorder.roles[0] == RAW_EXTRUSION_ROLE_PERIMETER);
        CHECK(recorder.roles[1] == RAW_EXTRUSION_ROLE_NONE);
        REQUIRE(recorder.printing_catalog_ids.size() == 2);
        CHECK(recorder.printing_catalog_ids[0] == 7);
        CHECK(recorder.printing_catalog_ids[1] == 0);
        CHECK_FALSE(boost::filesystem::exists(output_path.string() + ".tmp"));
    } catch (...) {
        remove_output_pair(output_path);
        throw;
    }
    remove_output_pair(output_path);
}
TEST_CASE("Default firmware selects the tool of an otherwise empty PrintingPlan", "[plugins][gcode]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    select_printing_plan_writer(print);
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    plan.groups.front().layers.front().tool_groups.emplace_back();
    plan.groups.front().layers.front().tool_groups.front().extruder_id = 0;
    append_empty_extrusion(plan.groups.front().layers.front().tool_groups.front());
    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);

    try {
        Steps::StepGenerateGcode::run_step(Orchestrator::instance(), print, output_path.string());
        REQUIRE(boost::filesystem::exists(output_path));
        CHECK(count_command_lines(read_text_file(output_path), "T0") == 1);
        CHECK_FALSE(boost::filesystem::exists(output_path.string() + ".tmp"));
    } catch (...) {
        remove_output_pair(output_path);
        throw;
    }
    remove_output_pair(output_path);
}
TEST_CASE("Default firmware publishes a preamble for an empty PrintingPlan", "[plugins][gcode]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    select_printing_plan_writer(print);
    print.mutable_printing_plan();
    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);

    try {
        Steps::StepGenerateGcode::run_step(Orchestrator::instance(), print, output_path.string());
        REQUIRE(boost::filesystem::exists(output_path));
        CHECK(read_text_file(output_path) == "; generated by SuperSlicer\n\nG21\nG90\nM82\n");
        CHECK(print.gcode_reader_source_path() == output_path.string());
        CHECK_FALSE(boost::filesystem::exists(output_path.string() + ".tmp"));

        Steps::StepGenerateGcode::clear(print);
        CHECK(print.gcode_reader_source_path().empty());
    } catch (...) {
        remove_output_pair(output_path);
        throw;
    }
    remove_output_pair(output_path);
}
TEST_CASE("Generated G-code is streamed into the viewer result",
          "[plugins][gcode][reader][viewer][writer]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    orchestrator.reset_plugin_cancel();

    Print print;
    configure_standard_firmware(print);
    print.mutable_printing_plan();

    // Padding exceeds the host buffer so this exercises the reusable chunk
    // source as well as the final unterminated line handled by the Reader.
    std::string source;
    while (source.size() <= 70u * 1024u)
        source += "; padding\n";
    source += "T0\nG21\nG90\nM83\nG1 X0 Y0 Z0.2 F1200\nG1 X10 Y0 E1 F1200";

    const boost::filesystem::path source_path = temporary_gcode_path();
    remove_output_pair(source_path);
    write_text_file(source_path, source);
    print.set_gcode_reader_source_path(source_path.string());

    GCodeProcessorResult result;
    try {
        Steps::StepReadGeneratedGcode::run(
            orchestrator, print, "gcode.reader.marlin2", result);
        CHECK(result.filename == source_path.string());
        CHECK(result.reader_plugin_id == "gcode.reader.marlin2");

        bool has_extrusion = false;
        for (const GCodeProcessorResult::MoveVertex &move : result.moves)
            has_extrusion = has_extrusion || move.type == EMoveType::Extrude;
        CHECK(has_extrusion);
    } catch (...) {
        remove_output_pair(source_path);
        orchestrator.reset_plugin_cancel();
        throw;
    }
    remove_output_pair(source_path);
    orchestrator.reset_plugin_cancel();
}

TEST_CASE("PrintingPlan markers survive generated file Reader integration",
          "[plugins][gcode][reader][viewer][writer][integration][markers]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    orchestrator.reset_plugin_cancel();

    Print print;
    configure_standard_firmware(print);
    select_printing_plan_writer(print, "gcode.firmware.marlin2");
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();

    plan.groups.front().layers.emplace_back();
    PrintingLayerGroup &first_layer = plan.groups.front().layers.back();
    first_layer.print_z = scale_i(0.2);
    first_layer.tool_groups.emplace_back();
    PrintingToolGroup &first_tool = first_layer.tool_groups.back();
    first_tool.extruder_id = 0;
    append_path_extrusion(first_tool, make_integration_path(
        ArcPolyline(Points{
            Point::new_scale(0.0, 0.0),
            Point::new_scale(10.0, 0.0),
            Point::new_scale(10.0, 10.0),
            Point::new_scale(0.0, 0.0)
        }),
        ExtrusionRole::ExternalPerimeter, 0.45f, 0.2f));

    std::unique_ptr<ExtrusionPath> wipe = make_firmware_travel(
        ArcPolyline(Points{
            Point::new_scale(0.0, 0.0),
            Point::new_scale(1.0, 0.0)
        }),
        40.f, 800.f);
    slic3r_api::MutableExtrusionEntity wipe_view(
        reinterpret_cast<extrusion_entity_handle *>(wipe.get()));
    wipe_view.get_mutable(slic3r_api::EPropertyAttributes::key)->extrusion_role(
        RAW_EXTRUSION_ROLE_TRAVEL | RAW_EXTRUSION_ROLE_WIPE);
    append_path_extrusion(first_tool, std::move(wipe));

    append_path_extrusion(first_tool, make_integration_path(
        ArcPolyline(Points{
            Point::new_scale(1.0, 0.0),
            Point::new_scale(6.0, 0.0)
        }),
        ExtrusionRole::InternalInfill, 0.50f, 0.25f));

    plan.groups.front().layers.emplace_back();
    PrintingLayerGroup &second_layer = plan.groups.front().layers.back();
    second_layer.print_z = scale_i(0.4);
    second_layer.tool_groups.emplace_back();
    PrintingToolGroup &second_tool = second_layer.tool_groups.back();
    second_tool.extruder_id = 0;
    append_path_extrusion(second_tool, make_integration_path(
        ArcPolyline(Points{
            Point::new_scale(6.0, 0.0),
            Point::new_scale(12.0, 0.0),
            Point::new_scale(12.0, 6.0),
            Point::new_scale(6.0, 0.0)
        }),
        ExtrusionRole::ExternalPerimeter, 0.42f, 0.2f));

    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    GCodeProcessorResult result;
    try {
        Steps::StepGenerateGcode::clear(print);
        const std::string reader_plugin_id = Steps::StepGenerateGcode::run_step(
            orchestrator, print, output_path.string());
        REQUIRE(reader_plugin_id == "gcode.reader.marlin2");
        REQUIRE(print.gcode_reader_source_path() == output_path.string());

        const std::string generated = read_text_file(output_path);
        INFO(generated);
        CHECK(count_occurrences(generated, ";LAYER_CHANGE\n") == 2);
        CHECK(count_occurrences(generated, ";Z:") == 2);
        CHECK(count_occurrences(generated, ";HEIGHT:") == 3);
        CHECK(count_occurrences(generated, ";TYPE:") == 3);
        CHECK(count_occurrences(generated, ";WIDTH:") == 3);
        CHECK(count_occurrences(generated, ";WIPE_START\n") == 1);
        CHECK(count_occurrences(generated, ";WIPE_END\n") == 1);
        CHECK(count_occurrences(generated, ";SEAM:") == 2);

        Steps::StepReadGeneratedGcode::run(
            orchestrator, print, reader_plugin_id, result);
    } catch (...) {
        remove_output_pair(output_path);
        orchestrator.reset_plugin_cancel();
        throw;
    }
    remove_output_pair(output_path);
    orchestrator.reset_plugin_cancel();

    CHECK(result.filename == output_path.string());
    CHECK(result.reader_plugin_id == "gcode.reader.marlin2");

    const GCodeProcessorResult::MoveVertex *external_first = nullptr;
    const GCodeProcessorResult::MoveVertex *infill_first = nullptr;
    const GCodeProcessorResult::MoveVertex *external_second = nullptr;
    size_t visible_extrusions = 0;
    size_t wipes = 0;
    size_t seams = 0;
    for (const GCodeProcessorResult::MoveVertex &move : result.moves) {
        if (move.type == EMoveType::Wipe)
            ++wipes;
        else if (move.type == EMoveType::Seam)
            ++seams;
        if (move.type != EMoveType::Extrude || move.internal_only)
            continue;
        CHECK(std::isfinite(move.mm3_per_mm));
        CHECK(move.mm3_per_mm > 0.0f);
        ++visible_extrusions;
        if (move.layer_id == 0 &&
            move.extrusion_role == GCodeExtrusionRole::ExternalPerimeter)
            external_first = &move;
        else if (move.layer_id == 0 &&
                 move.extrusion_role == GCodeExtrusionRole::InternalInfill)
            infill_first = &move;
        else if (move.layer_id == 1 &&
                 move.extrusion_role == GCodeExtrusionRole::ExternalPerimeter)
            external_second = &move;
    }

    CHECK(visible_extrusions >= 7);
    CHECK(wipes == 1);
    CHECK(seams == 2);
    REQUIRE(external_first != nullptr);
    REQUIRE(infill_first != nullptr);
    REQUIRE(external_second != nullptr);
    CHECK(external_first->width == Approx(0.45f));
    CHECK(external_first->height == Approx(0.2f));
    CHECK(infill_first->width == Approx(0.50f));
    CHECK(infill_first->height == Approx(0.25f));
    CHECK(external_second->width == Approx(0.42f));
    CHECK(external_second->height == Approx(0.2f));

    const std::vector<float> &layer_times =
        result.print_statistics.modes[0].layers_times;
    REQUIRE(layer_times.size() >= 2);
    CHECK(layer_times[0] > 0.0f);
    CHECK(layer_times[1] > 0.0f);
}
TEST_CASE("PrintingPlan firmware executes host-owned scripts",
          "[plugins][gcode][firmware][custom-gcode]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    select_printing_plan_writer(print);
    configure_standard_firmware(print);
    PrintingPlan &plan = print.mutable_printing_plan();
    ExtrusionNop script(ExtrusionPropertyCustomGcodeText(
        ExtrusionPropertyCustomGcodeText::Code::SCRIPT,
        "M117 X{position[0]}"));
    plan.events.append_before(script);

    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    try {
        Steps::StepGenerateGcode::run_step(
            Orchestrator::instance(), print, output_path.string());
        REQUIRE(boost::filesystem::exists(output_path));
        const std::string output = read_text_file(output_path);
        CHECK(output.find("M117 X0") != std::string::npos);
        CHECK(output.find("{position") == std::string::npos);
    } catch (...) {
        remove_output_pair(output_path);
        throw;
    }
    remove_output_pair(output_path);
}
TEST_CASE("PrintingPlan scripts cannot replace a previously published file",
          "[plugins][gcode][firmware][custom-gcode]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    select_printing_plan_writer(print);
    PrintingPlan &plan = print.mutable_printing_plan();
    ExtrusionNop script(ExtrusionPropertyCustomGcodeText(
        ExtrusionPropertyCustomGcodeText::Code::SCRIPT,
        "M117 {undefined_script_variable}"));
    plan.events.append_before(script);

    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    write_text_file(output_path, "previous output\n");
    CHECK_THROWS_AS(
        Steps::StepGenerateGcode::run_step(
            Orchestrator::instance(), print, output_path.string()),
        RuntimeError);
    Orchestrator::instance().reset_plugin_cancel();
    REQUIRE(boost::filesystem::exists(output_path));
    CHECK(read_text_file(output_path) == "previous output\n");
    CHECK_FALSE(boost::filesystem::exists(output_path.string() + ".tmp"));
    remove_output_pair(output_path);
}
TEST_CASE("Firmware exceptions prevent final output publication", "[plugins][gcode][firmware]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    FirmwareRecorder recorder;
    recorder.throw_on_extrusion = true;
    FirmwareInstanceOwner firmware(slic3r_api::make_gcode_firmware_instance(
        std::unique_ptr<slic3r_api::GCodeFirmwareSession>(new RecordingFirmwareSession(recorder))));
    Print print;
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    plan.groups.front().layers.front().tool_groups.emplace_back();
    append_empty_extrusion(plan.groups.front().layers.front().tool_groups.front());

    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    write_text_file(output_path, "previous output\n");
    CHECK_THROWS_AS(run_file_writer_with_firmware(print, output_path, firmware.instance()), RuntimeError);
    REQUIRE(boost::filesystem::exists(output_path));
    CHECK(read_text_file(output_path) == "previous output\n");
    CHECK_FALSE(boost::filesystem::exists(output_path.string() + ".tmp"));
    remove_output_pair(output_path);
}
TEST_CASE("STEP_GCODE fails when the output directory is unavailable", "[plugins][gcode]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Orchestrator &orchestrator = Orchestrator::instance();
    orchestrator.reset_plugin_cancel();
    Print print;
    select_printing_plan_writer(print);
    print.mutable_printing_plan();
    const boost::filesystem::path missing_directory =
        boost::filesystem::temp_directory_path() /
        boost::filesystem::unique_path("slic3r-missing-output-dir-%%%%-%%%%");
    const boost::filesystem::path output_path = missing_directory / "out.gcode";

    CHECK_THROWS_AS(Steps::StepGenerateGcode::run_step(orchestrator, print, output_path.string()),
                    RuntimeError);
    orchestrator.reset_plugin_cancel();
    CHECK_FALSE(boost::filesystem::exists(output_path));
    CHECK_FALSE(boost::filesystem::exists(output_path.string() + ".tmp"));
}
TEST_CASE("Orchestrator export_gcode still routes through ordering and STEP_GCODE", "[plugins][gcode]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    select_printing_plan_writer(print);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.option<ConfigOptionString>("start_gcode")->value.clear();
    config.option<ConfigOptionString>("end_gcode")->value.clear();
    for (slicing_step_t step : Steps::execution_order())
        print.mark_step_executed(step);

    GCodeProcessorResult result;
    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    std::vector<PrintBase::SlicingStatus> statuses;
    print.set_status_callback([&statuses](const PrintBase::SlicingStatus &status) {
        statuses.push_back(status);
    });

    try {
        const std::string generated_path =
            Orchestrator::instance().export_gcode(print, output_path.string(), &result, nullptr);
        CHECK(generated_path == output_path.string());
        CHECK(result.filename == output_path.string());
        CHECK(result.reader_plugin_id == "gcode.reader.marlin2");
        REQUIRE(boost::filesystem::exists(output_path));
        CHECK(read_text_file(output_path) == "; generated by SuperSlicer\n\nG21\nG90\nM82\n");
        REQUIRE(print.printing_plan() != nullptr);
        CHECK(print.printing_plan()->groups.size() == 1);

        // Step labels are not boost::format templates. A leftover path argument
        // would make the GUI throw boost::io::too_many_args while showing this
        // otherwise successful export.
        bool saw_ordering_status = false;
        for (const PrintBase::SlicingStatus &status : statuses) {
            if (status.main_text != "Ordering extrusions")
                continue;
            saw_ordering_status = true;
            CHECK(status.args.empty());
        }
        CHECK(saw_ordering_status);
    } catch (...) {
        remove_output_pair(output_path);
        throw;
    }
    remove_output_pair(output_path);
}

TEST_CASE("PrintingPlan writer exports a cube with cost seam placement",
          "[plugins][gcode][seam-placer][cost][checkpoint][integration]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Orchestrator &orchestrator = Orchestrator::instance();
    Plugin *cost_plugin = orchestrator.get_plugin("seam_placer.cost");
    REQUIRE(cost_plugin != nullptr);
    const bool cost_was_active = orchestrator.is_plugin_active(cost_plugin);
    REQUIRE(orchestrator.set_plugin_active(cost_plugin, true));

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict("step_gcode_plugin", "gcode.printing_plan_file_writer");
    config.set_deserialize_strict("gcode_firmware_plugin", "gcode.firmware.marlin2");
    const std::string seam_selector =
        Steps::plugin_exclusive_group_option_key(orchestrator, SEAM_PLACER, "seam_placer_plugin");
    REQUIRE_FALSE(seam_selector.empty());
    config.set_deserialize_strict(seam_selector, "seam_placer.cost");
    const std::string ordering_selector =
        Steps::plugin_exclusive_group_option_key(orchestrator, STEP_ORDERING, "ordering.extrusion_tree");
    REQUIRE_FALSE(ordering_selector.empty());
    config.set_deserialize_strict(ordering_selector, "ordering.extrusion_tree.advanced");

    Model model;
    Print print;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
    // Count the steps that really ran, to check which checkpoint each replay
    // resumed from.
    uint32_t slicing_count = 0;
    uint32_t finalize_slicing_count = 0;
    uint32_t ordering_count = 0;
    uint32_t ui_cancel_count = 0;
    bool cancel_after_ordering_restore = false;
    const Printing::PrintingPlan *plan_at_cancellation = nullptr;
    print.status().set_callback([&](const PrintBase::SlicingStatus &status) {
        if ((status.flags & PrintBase::SlicingStatus::MAIN_STATE) == 0)
            return;
        if (status.main_text == "StepSlicing")
            ++slicing_count;
        if (status.main_text == "Finalizing slicing")
            ++finalize_slicing_count;
        if (status.main_text == "Ordering extrusions")
            ++ordering_count;
        if (cancel_after_ordering_restore &&
            print.running_step() == Steps::checkpoint_resume_step(Steps::Checkpoint::Ordering)) {
            cancel_after_ordering_restore = false;
            plan_at_cancellation = print.printing_plan();
            print.cancel();
        }
    });

    // Ignore comments, not motion/extrusion values or command order.
    const auto normalized_commands = [](const std::string &gcode) {
        std::istringstream stream(gcode);
        std::vector<std::string> commands;
        std::string line;
        while (std::getline(stream, line)) {
            line = line.substr(0, line.find(';'));
            const size_t first = line.find_first_not_of(" \t\r");
            if (first != std::string::npos)
                commands.push_back(line.substr(first, line.find_last_not_of(" \t\r") - first + 1));
        }
        return commands;
    };

    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    try {
        print.process();
        CHECK_FALSE(print.is_step_done(STEP_WIPETOWER));
        CHECK(print.should_execute_step(STEP_WIPETOWER));
        CHECK_FALSE(print.tool_orderings().empty());
        const std::string generated_path =
            orchestrator.export_gcode(print, output_path.string(), nullptr, nullptr);
        CHECK(generated_path == output_path.string());
        CHECK(print.is_step_done(STEP_WIPETOWER));
        REQUIRE(boost::filesystem::exists(output_path));
        CHECK(read_text_file(output_path).find("G1") != std::string::npos);
        const std::vector<std::string> fresh_commands = normalized_commands(read_text_file(output_path));
        PrintStateBase::TimeStamp previous_pipeline_timestamp = 0;
        for (const slicing_step_t step : Steps::execution_order()) {
            const PrintStateBase::StateWithWarnings state = print.step_state_with_warnings(step);
            CAPTURE(step);
            CHECK(state.is_done());
            CHECK(state.enabled);
            CHECK(state.timestamp > previous_pipeline_timestamp);
            previous_pipeline_timestamp = state.timestamp;
        }
        bool has_extruding_motion = false;
        for (const std::string &command : fresh_commands) {
            if (command.find("G1 ") == 0 && command.find(" E") != std::string::npos &&
                (command.find(" X") != std::string::npos || command.find(" Y") != std::string::npos))
                has_extruding_motion = true;
        }
        REQUIRE(has_extruding_motion);
        CHECK(slicing_count == 1);
        CHECK(finalize_slicing_count == 1);
        CHECK(ordering_count == 1);
        remove_output_pair(output_path);

        // The read-only G-code stage reuses the final plan directly.
        const Printing::PrintingPlan *final_plan = print.printing_plan();
        REQUIRE(final_plan != nullptr);
        REQUIRE(print.is_step_done(STEP_GCODE));
        const size_t first_export_timestamp = print.step_state_with_timestamp(STEP_GCODE).timestamp;
        print.set_cancel_callback([&]() { ++ui_cancel_count; });
        const std::string replayed_gcode_path =
            orchestrator.export_gcode(print, output_path.string(), nullptr, nullptr);
        print.set_cancel_callback([]() {});
        CHECK(ui_cancel_count == 0);
        CHECK(replayed_gcode_path == output_path.string());
        CHECK(print.printing_plan() == final_plan);
        CHECK(read_text_file(output_path).find("G1") != std::string::npos);
        CHECK(normalized_commands(read_text_file(output_path)) == fresh_commands);
        CHECK(print.is_step_done(STEP_GCODE));
        CHECK(print.step_state_with_timestamp(STEP_GCODE).timestamp > first_export_timestamp);
        CHECK_FALSE(print.should_execute_step(STEP_GCODE));
        CHECK(slicing_count == 1);
        CHECK(finalize_slicing_count == 1);
        CHECK(ordering_count == 1);
        remove_output_pair(output_path);

        // A plan-editor change resumes from the ordering checkpoint. Background
        // slicing stops before plan replay, which belongs to export.
        print.request_replay_from_worker(Steps::checkpoint_before_step(STEP_EXTRUSION_EDIT));
        const Printing::PrintingPlan *plan_before_editor_replay = print.printing_plan();
        print.process();
        CHECK(print.printing_plan() == plan_before_editor_replay);
        // Mark the stale plan: its address may be reused by the restored clone.
        const size_t ordered_group_count = print.printing_plan()->groups.size();
        print.mutable_printing_plan().groups.emplace_back();
        const std::string replayed_editor_path =
            orchestrator.export_gcode(print, output_path.string(), nullptr, nullptr);
        CHECK(replayed_editor_path == output_path.string());
        REQUIRE(print.printing_plan() != nullptr);
        CHECK(print.printing_plan()->groups.size() == ordered_group_count);
        CHECK(normalized_commands(read_text_file(output_path)) == fresh_commands);
        CHECK(finalize_slicing_count == 1);
        CHECK(ordering_count == 1);
        CHECK(read_text_file(output_path).find("G1") != std::string::npos);
        remove_output_pair(output_path);

        SECTION("Cancellation after Ordering restoration leaves plan replay retryable") {
            REQUIRE(print.has_ordering_checkpoint());
            REQUIRE(print.is_step_done(STEP_GCODE));
            const size_t editor_export_timestamp = print.step_state_with_timestamp(STEP_GCODE).timestamp;
            const Printing::PrintingPlan *plan_before_cancelled_replay = print.printing_plan();
            REQUIRE(plan_before_cancelled_replay != nullptr);
            const size_t checkpoint_group_count = print.printing_plan()->groups.size();
            print.mutable_printing_plan().groups.emplace_back();
            print.request_replay_from_worker(Steps::checkpoint_before_step(STEP_EXTRUSION_EDIT));
            REQUIRE(print.current_checkpoint() == Steps::Checkpoint::Ordering);
            CHECK(print.printing_plan() == plan_before_cancelled_replay);
            cancel_after_ordering_restore = true;
            print.set_cancel_callback([&]() { ++ui_cancel_count; });
            CHECK_THROWS_AS(orchestrator.export_gcode(print, output_path.string(), nullptr, nullptr),
                            CanceledException);
            print.set_cancel_callback([]() {});
            CHECK(ui_cancel_count == 0);
            CHECK_FALSE(cancel_after_ordering_restore);
            REQUIRE(plan_at_cancellation != nullptr);
            CHECK(plan_at_cancellation->groups.size() == checkpoint_group_count);
            CHECK(print.printing_plan() == plan_at_cancellation);
            CHECK(print.canceled());
            CHECK(print.running_step() == STEP_NONE);
            CHECK(print.has_ordering_checkpoint());
            CHECK(print.current_checkpoint() == Steps::Checkpoint::Ordering);
            CHECK_FALSE(print.is_step_done(STEP_GCODE));
            for (const slicing_step_t step : Steps::execution_order()) {
                CAPTURE(step);
                CHECK(print.should_execute_step(step) ==
                      !Steps::executes_before(step, Steps::checkpoint_resume_step(Steps::Checkpoint::Ordering)));
            }
            CHECK(slicing_count == 1);
            CHECK(finalize_slicing_count == 1);
            CHECK(ordering_count == 1);
            CHECK_FALSE(boost::filesystem::exists(output_path));
            CHECK_FALSE(boost::filesystem::exists(output_path.string() + ".tmp"));

            print.mutable_printing_plan().groups.emplace_back();
            print.restart();
            print.set_cancel_callback([&]() { ++ui_cancel_count; });
            const std::string resumed_editor_path =
                orchestrator.export_gcode(print, output_path.string(), nullptr, nullptr);
            print.set_cancel_callback([]() {});
            CHECK(ui_cancel_count == 0);
            CHECK(resumed_editor_path == output_path.string());
            REQUIRE(boost::filesystem::exists(output_path));
            CHECK(normalized_commands(read_text_file(output_path)) == fresh_commands);
            REQUIRE(print.printing_plan() != nullptr);
            CHECK(print.printing_plan()->groups.size() == checkpoint_group_count);
            CHECK(print.has_ordering_checkpoint());
            CHECK(print.current_checkpoint() == Steps::Checkpoint::GCode);
            CHECK(print.is_step_done(STEP_GCODE));
            CHECK(print.step_state_with_timestamp(STEP_GCODE).timestamp > editor_export_timestamp);
            for (const slicing_step_t step : Steps::execution_order()) {
                CAPTURE(step);
                CHECK_FALSE(print.should_execute_step(step));
            }
            CHECK(slicing_count == 1);
            CHECK(finalize_slicing_count == 1);
            CHECK(ordering_count == 1);
            CHECK_FALSE(boost::filesystem::exists(output_path.string() + ".tmp"));
            remove_output_pair(output_path);
        }

        // A selector change can invalidate STEP_ORDERING after the downstream
        // layer extrusion edits were already marked done. Rebuilding the plan
        // must force those edits again so STEP_GCODE never sees speedless roots.
        print.request_replay_from_worker(Steps::checkpoint_before_step(STEP_ORDERING));
        const std::string regenerated_path =
            orchestrator.export_gcode(print, output_path.string(), nullptr, nullptr);
        CHECK(regenerated_path == output_path.string());
        REQUIRE(boost::filesystem::exists(output_path));
        CHECK(read_text_file(output_path).find("G1") != std::string::npos);
        CHECK(finalize_slicing_count == 1);
        CHECK(ordering_count == 2);

        // Without the ordered checkpoint, editor replay rebuilds Ordering from
        // the retained extrusion trees and then reruns all later steps.
        print.set_keep_ordering_checkpoint(false);
        print.request_replay_from_worker(Steps::checkpoint_before_step(STEP_EXTRUSION_EDIT));
        const std::string replayed_ordering_path =
            orchestrator.export_gcode(print, output_path.string(), nullptr, nullptr);
        CHECK(replayed_ordering_path == output_path.string());
        CHECK(read_text_file(output_path).find("G1") != std::string::npos);
        CHECK(finalize_slicing_count == 1);
        CHECK(ordering_count == 3);

        // Repeated cleanup must neither retain old extrusion edits nor erase the retained slices.
        for (uint32_t replay = 0; replay < 2; ++replay) {
            CAPTURE(replay);
            print.request_replay_from_worker(Steps::Checkpoint::Slicing);
            REQUIRE(print.current_checkpoint() == Steps::Checkpoint::Slicing);
            if (replay == 0)
                print.process();
            remove_output_pair(output_path);
            CHECK(orchestrator.export_gcode(print, output_path.string(), nullptr, nullptr) ==
                  output_path.string());
            REQUIRE(boost::filesystem::exists(output_path));
            CHECK(normalized_commands(read_text_file(output_path)) == fresh_commands);
            CHECK(slicing_count == 1);
            CHECK(finalize_slicing_count == 2 + replay);
            CHECK(ordering_count == 4 + replay);
            CHECK_FALSE(boost::filesystem::exists(output_path.string() + ".tmp"));
        }
    } catch (...) {
        print.set_cancel_callback([]() {});
        print.status().set_callback([](const PrintBase::SlicingStatus &) {});
        print.restart();
        orchestrator.reset_plugin_cancel();
        remove_output_pair(output_path);
        orchestrator.set_plugin_active(cost_plugin, cost_was_active);
        throw;
    }
    print.status().set_callback([](const PrintBase::SlicingStatus &) {});
    remove_output_pair(output_path);
    orchestrator.set_plugin_active(cost_plugin, cost_was_active);
}
