#include <catch2/catch.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
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
#include "libslic3r/Plugins/GCode/Firmware/BambuGCodeFirmware.hpp"
#include "libslic3r/Plugins/GCode/Firmware/BuiltinGCodeFirmwares.hpp"
#include "libslic3r/Plugins/GCode/Firmware/KlipperGCodeFirmware.hpp"
#include "libslic3r/Plugins/GCode/Firmware/Marlin1GCodeFirmware.hpp"
#include "libslic3r/Plugins/GCode/Firmware/Marlin2GCodeFirmware.hpp"
#include "libslic3r/Plugins/GCode/Firmware/PrusaGCodeFirmware.hpp"
#include "libslic3r/Plugins/GCode/Firmware/RepRapGCodeFirmware.hpp"
#include "libslic3r/Plugins/GCode/Firmware/SprinterGCodeFirmware.hpp"
#include "libslic3r/Plugins/GCode/Reader/BambuGCodeReaderItems.hpp"
#include "libslic3r/Plugins/GCode/Reader/RepRapGCodeReaderState.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/Config/Preset.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"
#include "libslic3r/Steps/StepExtrusionEdition.hpp"
#include "libslic3r/Steps/StepGenerateGcode.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"
#include "test_data.hpp"

/*
Compare the commands emitted by the built-in firmware dialects for the same
small plan or requested machine change. Literal expectations make differences
in tool, heater, fan and object-label syntax visible without deriving the oracle
from the encoder under test. The test owns each plan and session lifetime.

Firmware dialect tests keep cross-dialect command matrices together so protocol differences remain directly comparable.
Shared plan construction and output helpers live in gcode_test_helpers so this file keeps only component-specific behavior.
Chamber fan tests drive the session's real callbacks with a small plan; boundary
comments separate callback output from geometry to check deferred fan placement.

Representative direct calls (branches are not a timeline):

    TEST_CASE
        |-- configure_standard_firmware()
        |-- append_path_extrusion()
        |-- make_firmware_path()
        |-- Steps::StepExtrusionEdition::run_step()
        |-- events.append_after()
        `-- export_with_firmware()
*/

namespace {
using namespace Slic3r;
using namespace Slic3r::Printing;
using namespace Slic3r::Test::GCode;

// Run two sequential groups with three configured filaments (only 0 and 1 used).
// Owned text includes callback boundaries; an ordered TOOLCHANGE inside a tree
// checks selection independently of the tool-group id. Invalid setup throws.
std::string run_bambu_fans(bool platter, bool filtration, bool activated, int disabled_layers,
                           bool activate_second = true, bool reverse_exhaust = false);

// Build valid paths and session views from local owners, preserving their lifetime
// through every callback; no export file or production plugin state is changed.
std::string run_bambu_fans(bool platter, bool filtration, bool activated, int disabled_layers,
                           bool activate_second, bool reverse_exhaust)
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_extruders(3);
    config.set_deserialize_strict({
        {"platter_fan", platter ? "1" : "0"},
        {"support_air_filtration", filtration ? "1" : "0"},
        {"platter_fan_speed", "70,80,100"},
        {"activate_air_filtration", activated ? (activate_second ? "1,1,1" : "1,0,1") : "0,0,1"},
        {"during_print_exhaust_fan_speed", reverse_exhaust ? "70,80,100" : "60,70,100"},
        {"complete_print_exhaust_fan_speed", reverse_exhaust ? "60,70,100" : "70,80,100"},
        {"disable_fan_first_layers", std::to_string(disabled_layers) + ",0,0"},
        {"nozzle_diameter", "0.4,0.4,0.4"},
        {"filament_diameter", "1.75,1.75,1.75"},
        {"extrusion_multiplier", "1,1,1"},
        {"start_gcode", ";FAN_START"},
        {"gcode_label_objects", "disabled"}});
    Model model;
    Print print;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
    configure_standard_firmware(print, 3);
    PrintConfig &print_config = const_cast<PrintConfig &>(print.config());
    print_config.filament_diameter.set(std::vector<double>{1.75, 1.75, 1.75});
    print_config.extrusion_multiplier.set(std::vector<double>{1., 1., 1.});
    PrintingPlan plan;
    ExtrusionNop start;
    start.add_property(ExtrusionPropertyCustomGcodeText(ExtrusionPropertyCustomGcodeText::Code::SCRIPT,
        config.option<ConfigOptionString>("start_gcode")->value, GCODE_SCRIPT_TYPE_START_GCODE));
    plan.events.append_before(start);
    for (uint32_t group_idx = 0; group_idx < 2; ++group_idx) {
        plan.groups.emplace_back();
        for (uint32_t layer_idx = 0; layer_idx < 4; ++layer_idx) {
            plan.groups.back().layers.emplace_back();
            PrintingLayerGroup &layer = plan.groups.back().layers.back();
            layer.print_z = scale_i(0.2 * (layer_idx + 1));
            layer.tool_groups.emplace_back();
            PrintingToolGroup &tool = layer.tool_groups.back();
            tool.extruder_id = 0;
            const ArcPolyline path(Points{Point::new_scale(0., double(layer_idx)),
                                          Point::new_scale(1., double(layer_idx))});
            append_path_extrusion(tool, make_firmware_path(path, 20.f, 500.f));
            if (layer_idx == 2) {
                // The group's nominal filament stays zero; the ordered tree changes it.
                std::unique_ptr<ExtrusionEntity> root(new ExtrusionEntity(true));
                root->append_child(std::unique_ptr<ExtrusionEntity>(new ExtrusionNop(
                    ExtrusionPropertySpecialCommand(ExtrusionPropertySpecialCommand::Code::TOOLCHANGE, 1.))));
                std::unique_ptr<ExtrusionNop> marker(new ExtrusionNop());
                marker->add_property(ExtrusionPropertyCustomGcodeText(";AFTER_TOOLCHANGE"));
                root->append_child(std::move(marker));
                root->append_child(make_firmware_path(path, 20.f, 500.f));
                append_path_extrusion(tool, std::move(root));
                // A real extrusion makes filament 1 part of the plan's used set.
                layer.tool_groups.emplace_back();
                layer.tool_groups.back().extruder_id = 1;
                append_path_extrusion(layer.tool_groups.back(), make_firmware_path(path, 20.f, 500.f));
                // Empty configured tools and unresolved/nonexistent tools are not used filaments.
                layer.tool_groups.emplace_back();
                layer.tool_groups.back().extruder_id = 2;
                for (const extruder_index_t invalid : {EXTRUDER_ID_UNKNOWN, extruder_index_t(7)}) {
                    layer.tool_groups.emplace_back();
                    layer.tool_groups.back().extruder_id = invalid;
                    append_path_extrusion(layer.tool_groups.back(), make_firmware_path(path, 20.f, 500.f));
                }
            }
        }
    }
    GCodeScriptProcessor scripts(print, Orchestrator::instance());
    slic3r_api::GCodeGeneration::Firmware::BambuGCodeFirmwareSession session(
        slic3r_api::GCodeGeneration::GCodeScriptProcessorView(scripts.c_processor()), nullptr);
    const slic3r_api::ConstPrintingPlan view(reinterpret_cast<const printing_plan_handle *>(&plan));
    std::string output = session.begin_print(
        slic3r_api::Print(reinterpret_cast<const print_handle *>(&print)), view);
    for (uint32_t group_idx = 0; group_idx < view.group_count(); ++group_idx) {
        const slic3r_api::ConstPrintingGroup group = view.group(group_idx);
        output += ";GROUP " + std::to_string(group_idx) + "\n";
        output += session.begin_group(group);
        for (uint32_t layer_idx = 0; layer_idx < group.layer_group_count(); ++layer_idx) {
            const slic3r_api::ConstPrintingLayerGroup layer = group.layer_group(layer_idx);
            output += ";BEGIN_LAYER\n" + session.begin_layer(layer) + ";AFTER_LAYER\n";
            for (uint32_t tool_idx = 0; tool_idx < layer.tool_group_count(); ++tool_idx) {
                const slic3r_api::ConstPrintingToolGroup tool = layer.tool_group(tool_idx);
                if (tool.extrusion_count() == 0 || tool.extruder_id() < 0 || tool.extruder_id() >= 3)
                    continue;
                output += session.begin_tool_group(tool);
                for (uint32_t idx = 0; idx < tool.extrusion_count(); ++idx)
                    output += session.write_extrusion(tool.extrusion(idx));
                output += session.end_tool_group();
            }
            output += session.end_layer();
        }
        output += session.end_group();
    }
    output += ";END_GCODE\n";
    return output + session.end_print();
}

class SprinterFirmwareProbe final :
    public slic3r_api::GCodeGeneration::Firmware::SprinterGCodeFirmwareSession
{
public:
    using SprinterGCodeFirmwareSession::encode_chamber_temperature;
    using SprinterGCodeFirmwareSession::encode_extruder_current;
};

class KlipperFirmwareProbe final :
    public slic3r_api::GCodeGeneration::Firmware::KlipperGCodeFirmwareSession
{
public:
    using KlipperGCodeFirmwareSession::encode_chamber_temperature;
    using KlipperGCodeFirmwareSession::encode_extruder_current;
};

// Exposes only the private-state boundary used around custom scripts. This
// keeps the test independent of PrintingPlan traversal while still exercising
// the real RepRap firmware implementation and Gantry registers.
class RepRapReaderStateProbe final :
    public slic3r_api::GCodeGeneration::Firmware::RepRapGCodeFirmwareSession
{
public:
    using RepRapGCodeFirmwareSession::append_reader_private_state;
    using RepRapGCodeFirmwareSession::encode_fan;
    using RepRapGCodeFirmwareSession::encode_machine_envelope;
    using RepRapGCodeFirmwareSession::encode_pressure_advance;
    using RepRapGCodeFirmwareSession::synchronize_reader_private_state;

    void request_accelerations(uint32_t print, uint32_t travel)
    {
        gantry().request_print_acceleration(print);
        gantry().request_travel_acceleration(travel);
    }
    std::optional<uint32_t> requested_print() const
    {
        return gantry().requested_print_acceleration();
    }
    std::optional<uint32_t> requested_travel() const
    {
        return gantry().requested_travel_acceleration();
    }
    std::optional<uint32_t> encoded_print() const
    {
        return gantry().encoded_print_acceleration();
    }
    std::optional<uint32_t> encoded_travel() const
    {
        return gantry().encoded_travel_acceleration();
    }
};

// Build a print/travel/print sequence whose extrusion leaves deliberately
// carry no acceleration. The final print may be omitted when a test only
// needs to prove that an unsafe travel switch was suppressed.
void append_optional_acceleration_sequence(Print &print, bool append_final_print = true)
{
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.front().layers.front();
    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool_group = layer.tool_groups.front();
    tool_group.extruder_id = 0;
    append_path_extrusion(tool_group, make_firmware_path(
        ArcPolyline(Points{Point(scale_i(0.0), scale_i(0.0)), Point(scale_i(1.0), scale_i(0.0))}),
        20.f, -1.f));
    append_path_extrusion(tool_group, make_firmware_travel(
        ArcPolyline(Points{Point(scale_i(1.0), scale_i(0.0)), Point(scale_i(2.0), scale_i(0.0))}),
        80.f, 900.f));
    if (append_final_print)
        append_path_extrusion(tool_group, make_firmware_path(
            ArcPolyline(Points{Point(scale_i(2.0), scale_i(0.0)), Point(scale_i(3.0), scale_i(0.0))}),
            20.f, -1.f));
}

} // namespace

TEST_CASE("G-code firmware service exposes its printer selector and standard session",
          "[plugins][gcode][firmware]")
{
    STATIC_REQUIRE(std::is_base_of<
        slic3r_api::GCodeFirmwareSession,
        slic3r_api::GCodeGeneration::DefaultGCodeFirmwareSession>::value);

    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Orchestrator &orchestrator = Orchestrator::instance();
    Print print;

    const std::set<std::string> expected_firmwares = {
        "marlin",
        "marlin2",
        "prusa",
        "reprapfirmware",
        "sprinter",
        "klipper",
        "bambu"
    };
    std::set<std::string> registered_firmwares;
    for (Plugin *plugin : orchestrator.get_active_plugins_for_step(GCODE_FIRMWARE))
        registered_firmwares.insert(plugin->get_id());
    CHECK(registered_firmwares == expected_firmwares);
    CHECK(orchestrator.get_plugin("gcode.firmware.default") == nullptr);

    Plugin *selected = Steps::selected_or_active_plugin_for_step(
        orchestrator, GCODE_FIRMWARE, &print.full_print_config());
    REQUIRE(selected != nullptr);
    CHECK(selected->get_id() == "marlin2");

    std::set<std::string> used_keys;
    for (const Plugin::UsedConfigKey &key : selected->get_used_config_keys())
        used_keys.insert(key.key);
    CHECK(used_keys.count("extruder_offset") == 1);
    CHECK(used_keys.count("filament_diameter") == 1);
    CHECK(used_keys.count("machine_limits_usage") == 1);
    CHECK(used_keys.count("machine_max_acceleration_x") == 1);
    CHECK(used_keys.count("machine_max_feedrate_x") == 1);
    CHECK(used_keys.count("machine_max_jerk_x") == 1);
    CHECK(used_keys.count("travel_acceleration") == 0);
    CHECK(used_keys.count("travel_speed") == 1);
    CHECK(used_keys.count("z_offset") == 1);

    const ConfigOptionDef *definition = PrintConfigDef::instance().get("gcode_flavor");
    REQUIRE(definition != nullptr);
    CHECK(definition->option_preset_type == RAW_PRESET_TYPE_FFF_PRINTER);
    CHECK(definition->invalidates_step == STEP_ORDERING);

    // The selector keeps the place of the former option in the printer page.
    const std::string merged = orchestrator.merged_ui_layout(
        "printer_fff.ui",
        "page:General:printer\n"
        "group:Firmware\n"
        "\tsetting:gcode_flavor\n");
    const size_t firmware_pos = merged.find("setting:gcode_flavor");
    REQUIRE(firmware_pos != std::string::npos);
    CHECK(merged.find("setting:gcode_flavor", firmware_pos + 1) == std::string::npos);
    CHECK(merged.find("setting:step_gcode_plugin") == std::string::npos);
    // The firmware fragment marks that place, so the host adds no generic copy
    // of the selector to the print settings.
    CHECK(orchestrator.merged_ui_layout("print.ui", "").find("setting:gcode_flavor") == std::string::npos);

    STATIC_REQUIRE(std::is_base_of<
        slic3r_api::GCodeGeneration::Firmware::Marlin2GCodeFirmwareSession,
        slic3r_api::GCodeGeneration::Firmware::PrusaGCodeFirmwareSession>::value);
}

TEST_CASE("Built-in firmware dialects select their matching G-code readers",
          "[plugins][gcode][firmware][reader]")
{
    using namespace slic3r_api::GCodeGeneration::Firmware;
    CHECK(Marlin1GCodeFirmwareSession().reader_plugin_id() == "gcode.reader.marlin1");
    CHECK(Marlin2GCodeFirmwareSession().reader_plugin_id() == "gcode.reader.marlin2");
    CHECK(PrusaGCodeFirmwareSession().reader_plugin_id() == "gcode.reader.prusa");
    CHECK(SprinterGCodeFirmwareSession().reader_plugin_id() == "gcode.reader.sprinter");
    CHECK(RepRapGCodeFirmwareSession().reader_plugin_id() == "gcode.reader.reprap");
    CHECK(KlipperGCodeFirmwareSession().reader_plugin_id() == "gcode.reader.klipper");
    CHECK(BambuGCodeFirmwareSession().reader_plugin_id() == "gcode.reader.bambu");
}

TEST_CASE("RepRap firmware round-trips private Reader acceleration state",
          "[plugins][gcode][firmware][reader][reprap]")
{
    using slic3r_api::GCodeGeneration::MachineEnvelope;
    using slic3r_api::GCodeGeneration::Reader::RepRapGCodeReaderState;

    RepRapReaderStateProbe firmware;
    const orchestrator_runtime_id reader_state_id = orchestrator_register_runtime_id(
        reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()),
        GCODE_READER_STATE_ID_TYPE,
        slic3r_api::GCodeGeneration::Reader::REPRAP_READER_STATE_NAME);
    REQUIRE(reader_state_id != ORCHESTRATOR_RUNTIME_ID_INVALID);
    firmware.set_reader_state_id(reader_state_id);
    MachineEnvelope envelope;
    envelope.max_acceleration_x = 1000.0;
    envelope.max_acceleration_y = 1100.0;
    envelope.max_acceleration_z = 100.0;
    envelope.max_acceleration_e = 1200.0;
    envelope.max_feedrate_x = 300.0;
    envelope.max_print_acceleration = 800.0;
    envelope.max_travel_acceleration = 1500.0;
    (void) firmware.encode_machine_envelope(envelope);

    std::vector<raw_gcode_reader_event_item> items = {
        {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 0.0},
        {RAW_GCODE_READER_ITEM_READER_ID, 0, double(reader_state_id)}
    };
    firmware.append_reader_private_state(items);
    raw_gcode_reader_state raw = {items.data(), static_cast<uint32_t>(items.size())};
    slic3r_api::GCodeStateTracker tracker(raw);
    RepRapGCodeReaderState state =
        RepRapGCodeReaderState::from_tracker(tracker, reader_state_id);
    CHECK(state.max_acceleration[1] == 1100.0);
    CHECK(state.max_feedrate[0] == 300.0);
    CHECK(state.print_acceleration == 800.0);
    CHECK(state.travel_acceleration == 1500.0);

    // A script changes encoded firmware registers, but the plan's requested
    // values remain authoritative and will be re-emitted when they differ.
    firmware.request_accelerations(700u, 1400u);
    state.print_acceleration = 600.0;
    state.travel_acceleration = 1300.0;
    slic3r_api::GCodeGeneration::Reader::RepRapToolState &tool = state.tools[2];
    tool.drives = {3, 4};
    tool.fans = {6, 7};
    state.write_to(tracker, reader_state_id);
    firmware.synchronize_reader_private_state(tracker);
    CHECK(firmware.requested_print() == 700u);
    CHECK(firmware.requested_travel() == 1400u);
    CHECK(firmware.encoded_print() == 600u);
    CHECK(firmware.encoded_travel() == 1300u);
    CHECK(firmware.encode_fan(2, 25.0) ==
          "M106 P6 S0.25\nM106 P7 S0.25\n");
    CHECK(firmware.encode_pressure_advance(2, 0.04) ==
          "M572 D3 S0.04\nM572 D4 S0.04\n");
}

TEST_CASE("Built-in firmware dialects encode their machine envelope preamble",
          "[plugins][gcode][firmware][dialects][machine-envelope]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    struct DialectEnvelopeExpectation
    {
        const char *id;
        const char *output;
        // Text written after the envelope by the dialect (Bambu opens its detector).
        const char *after_start = "";
    };
    const DialectEnvelopeExpectation expectations[] = {
        {
            "marlin",
            "M201 X1000 Y1100 Z120 E1300 ; sets maximum accelerations, mm/sec^2\n"
            "M203 X200 Y210 Z12 E25 ; sets maximum feedrates, mm/sec\n"
            "M204 S500 T800 ; sets print and retract acceleration, mm/sec^2\n"
            "M205 X8.00 Y9.00 Z0.40 E2.50 ; sets the jerk limits, mm/sec\n"
            "M205 S3 T4 ; sets the minimum extruding and travel feed rate, mm/sec\n"
        },
        {
            "marlin2",
            "M201 X1000 Y1100 Z120 E1300 ; sets maximum accelerations, mm/sec^2\n"
            "M203 X200 Y210 Z12 E25 ; sets maximum feedrates, mm/sec\n"
            "M204 P500 R800 T900 ; sets print, retract and travel acceleration, mm/sec^2\n"
            "M205 X8.00 Y9.00 Z0.40 E2.50 ; sets the jerk limits, mm/sec\n"
            "M205 S3 T4 ; sets the minimum extruding and travel feed rate, mm/sec\n"
        },
        {
            "prusa",
            "M201 X1000 Y1100 Z120 E1300 ; sets maximum accelerations, mm/sec^2\n"
            "M203 X200 Y210 Z12 E25 ; sets maximum feedrates, mm/sec\n"
            "M204 P500 R800 T900 ; sets print, retract and travel acceleration, mm/sec^2\n"
            "M205 X8.00 Y9.00 Z0.40 E2.50 ; sets the jerk limits, mm/sec\n"
            "M205 S3 T4 ; sets the minimum extruding and travel feed rate, mm/sec\n"
        },
        {
            "reprapfirmware",
            "M201 X1000 Y1100 Z120 E1300 ; sets maximum accelerations, mm/sec^2\n"
            "M203 X12000 Y12600 Z720 E1500 I180 ; sets maximum feedrates, mm/min\n"
            "M204 P500 T900 ; sets print and travel acceleration, mm/sec^2\n"
            "M566 X480.00 Y540.00 Z24.00 E150.00 ; sets the jerk limits, mm/min\n"
        },
        {
            "sprinter",
            "M201 X1000 Y1100 Z120 E1300 ; sets maximum accelerations, mm/sec^2\n"
            "M203 X12000 Y12600 Z720 E1500 ; sets maximum feedrates, mm/min\n"
            "M204 P500 T900 ; sets print and travel acceleration, mm/sec^2\n"
        },
        {
            "klipper",
            "M204 P500 T900 ; sets the initial shared acceleration, mm/sec^2\n"
        },
        {
            "bambu",
            "M201 X1000 Y1100 Z120 E1300 ; sets maximum accelerations, mm/sec^2\n"
            "M203 X200 Y210 Z12 E25 ; sets maximum feedrates, mm/sec\n"
            "M204 P500 R800 T900 ; sets print, retract and travel acceleration, mm/sec^2\n"
            "M205 X8.00 Y9.00 Z0.40 E2.50 ; sets the jerk limits, mm/sec\n"
            "M205 S3 T4 ; sets the minimum extruding and travel feed rate, mm/sec\n",
            "M981 S1 P20000 ;open spaghetti detector\n"
        }
    };

    for (const DialectEnvelopeExpectation &expectation : expectations) {
        INFO(expectation.id);
        Print print;
        configure_standard_firmware(print);
        configure_machine_envelope(print);
        print.mutable_printing_plan();
        CHECK(export_with_firmware(print, expectation.id) ==
              std::string("G21\nG90\nM82\n") + expectation.output + expectation.after_start);
    }

    // The normal default keeps machine limits available for estimates without
    // inserting firmware commands into the file.
    Print disabled_print;
    configure_standard_firmware(disabled_print);
    disabled_print.mutable_printing_plan();
    CHECK(export_with_firmware(disabled_print, "marlin2") ==
          "G21\nG90\nM82\n");
}
TEST_CASE("Built-in firmware dialects encode their process commands",
          "[plugins][gcode][firmware][dialects]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    struct DialectExpectation
    {
        const char *id;
        const char *tool;
        const char *temperature;
        const char *wait;
        const char *fan;
        const char *pressure_advance;
        const char *acceleration;
    };
    const DialectExpectation expectations[] = {
        {"marlin", "T0\n", "M104 S200 T0\n", "M109 S200 T0\n",
         "M106 S128\n", "M900 K0.04\n", "M204 S500\n"},
        {"marlin2", "T0\n", "M104 S200 T0\n", "M109 S200 T0\n",
         "M106 S128\n", "M900 K0.04\n", "M204 P500\n"},
        {"prusa", "T0\n", "M104 S200 T0\n", "M109 S200 T0\n",
         "M106 S128\n", "M900 K0.04\n", "M204 P500\n"},
        {"reprapfirmware", "T0\n", "G10 P0 S200\n", "M116 P0\n",
         "M106 P0 S0.5\n", "M572 D0 S0.04\n", "M204 P500\n"},
        {"sprinter", "T0\n", "M104 S200 T0\n", "M109 S200 T0\n",
         "M106 S128\n", "M572 D0 S0.04\n", "M204 P500\n"},
        {"klipper", "ACTIVATE_EXTRUDER EXTRUDER=extruder\n",
         "M104 S200\n", "M109 S200\n", "M106 S128\n",
         "SET_PRESSURE_ADVANCE ADVANCE=0.04 EXTRUDER=extruder\n", "M204 S500\n"},
        {"bambu", "T0\n", "M104 S200 T0\n", "M109 S200 T0\n",
         "M106 S128\n", "M900 K0.04 L1000 M10 ; Override pressure advance value\n", "M204 P500\n"}
    };

    for (const DialectExpectation &expectation : expectations) {
        INFO(expectation.id);
        Print print;
        configure_standard_firmware(print);
    // Isolate runtime command encoding from automatic startup requests.
    const_cast<PrintConfig &>(print.config()).start_gcode_manual.value = true;
        PrintingPlan &plan = print.mutable_printing_plan();
        plan.groups.emplace_back();
        plan.groups.front().layers.emplace_back();
        PrintingLayerGroup &layer = plan.groups.front().layers.front();
        layer.tool_groups.emplace_back();
        PrintingToolGroup &tool_group = layer.tool_groups.front();
        tool_group.extruder_id = 0;
        ExtrusionEntity::Children children;
        children.push_back(make_firmware_path(
                ArcPolyline(Points{
                    Point(scale_i(0.0), scale_i(0.0)),
                    Point(scale_i(1.0), scale_i(0.0))
                }),
                20.f,
                500.f,
                0.04f,
                50.f,
                200.f));
        children.push_back(
            special_command_entity(ExtrusionPropertySpecialCommand::Code::WAIT_FOR_TEMP));
        std::unique_ptr<ExtrusionEntity> root(
            new ExtrusionEntity(std::move(children), false, true, true));
        root->add_property(ExtrusionPropertySpeed(20.f, 500.f, 0.04f));
        root->add_property(ExtrusionPropertyFan(50.f, FAN_REQUEST_MODE_OVERRIDE));
        root->add_property(ExtrusionPropertyHeater(
            C_EXTRUSION_HEATER_TOOL, EXTRUDER_ID_UNKNOWN, 200, false));
        append_path_extrusion(tool_group, std::move(root));

        const std::string output = export_with_firmware(print, expectation.id);
        INFO(output);
        CHECK(output.find(expectation.tool) != std::string::npos);
        CHECK(output.find(expectation.temperature) != std::string::npos);
        CHECK(output.find(expectation.wait) != std::string::npos);
        CHECK(output.find(expectation.fan) != std::string::npos);
        CHECK(output.find(expectation.pressure_advance) != std::string::npos);
        CHECK(output.find(expectation.acceleration) != std::string::npos);
        CHECK(output.find("G1 X1 E") != std::string::npos);
    }
}
// Feed speed-override events through locally owned sessions and compare exact
// output: Marlin backs up/restores, while other dialects finish at 100%.
TEST_CASE("Firmware speed-ratio save and restore encoding follows each dialect",
          "[plugins][gcode][firmware][dialects][speed-ratio]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    using namespace slic3r_api::GCodeGeneration;
    using namespace slic3r_api::GCodeGeneration::Firmware;

    DefaultGCodeFirmwareSession generic;
    Marlin1GCodeFirmwareSession marlin1;
    Marlin2GCodeFirmwareSession marlin2;
    PrusaGCodeFirmwareSession prusa;
    KlipperGCodeFirmwareSession klipper;
    RepRapGCodeFirmwareSession reprap;
    SprinterGCodeFirmwareSession sprinter;
    struct DialectExpectation
    {
        const char *name;
        DefaultGCodeFirmwareSession &session;
        bool saves_override;
    };
    const DialectExpectation expectations[] = {
        {"default", generic, false},
        {"marlin1", marlin1, true},
        {"marlin2", marlin2, true},
        {"prusa", prusa, true},
        {"klipper", klipper, false},
        {"reprap", reprap, false},
        {"sprinter", sprinter, false}
    };

    Print print;
    configure_standard_firmware(print);
    const slic3r_api::Print print_view(reinterpret_cast<const print_handle *>(&print));
    std::unique_ptr<ExtrusionEntity> save = special_command_entity(
        ExtrusionPropertySpecialCommand::Code::SAVE_AND_RESET_SPEED_RATIO, 0.8);
    std::unique_ptr<ExtrusionEntity> restore = special_command_entity(
        ExtrusionPropertySpecialCommand::Code::RESTORE_SPEED_RATIO);
    for (const DialectExpectation &expectation : expectations) {
        INFO(expectation.name);
        expectation.session.setup(print_view.config());
        CHECK(expectation.session.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(save.get()))) ==
            (expectation.saves_override ? "M220 B\nM220 S80\n" : "M220 S80\n"));
        CHECK(expectation.session.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(restore.get()))) ==
            (expectation.saves_override ? "M220 R\n" : "M220 S100\n"));
    }
}

// Export A/B/A process requests to distinguish a shared physical cache from per-tool caches.
TEST_CASE("Built-in firmware dialects address one physical extruder in SEMM",
          "[plugins][gcode][firmware][dialects][semm]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool semm = GENERATE(false, true);
    const char *firmware_id = GENERATE("marlin", "marlin2",
        "prusa", "sprinter", "reprapfirmware", "klipper",
        "bambu");
    CAPTURE(semm, firmware_id);
    Print print;
    configure_standard_firmware(print, 2);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.single_extruder_multi_material.value = semm;
    config.start_gcode_manual.value = true;
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.front().layers.front();
    layer.print_z = scale_i(0.2);
    for (size_t idx = 0; idx < 3; ++idx) {
        layer.tool_groups.emplace_back();
        PrintingToolGroup &group = layer.tool_groups.back();
        group.extruder_id = idx == 1 ? 1 : 0;
        const float temperature = idx == 1 ? 230.f : 200.f;
        const float pressure = idx == 1 ? 0.06f : 0.04f;
        const float fan = idx == 1 ? 60.f : 50.f;
        ExtrusionEntity::Children children;
        children.push_back(make_firmware_path(
            ArcPolyline(Points{Point::new_scale(double(idx), 0.0), Point::new_scale(double(idx + 1), 0.0)}),
            20.f, 500.f, pressure, fan, temperature));
        children.push_back(special_command_entity(ExtrusionPropertySpecialCommand::Code::WAIT_FOR_TEMP));
        std::unique_ptr<ExtrusionEntity> root(new ExtrusionEntity(std::move(children), false, true, true));
        root->add_property(ExtrusionPropertySpeed(20.f, 500.f, pressure));
        root->add_property(ExtrusionPropertyFan(fan, FAN_REQUEST_MODE_OVERRIDE));
        root->add_property(ExtrusionPropertyHeater(
            C_EXTRUSION_HEATER_TOOL, EXTRUDER_ID_UNKNOWN, int16_t(temperature), false));
        append_path_extrusion(group, std::move(root));
    }
    const std::string output = export_with_firmware(print, firmware_id);
    INFO(output);
    const bool reprap = std::string(firmware_id) == "reprapfirmware";
    const bool klipper = std::string(firmware_id) == "klipper";
    const bool sprinter = std::string(firmware_id) == "sprinter";
    const std::string first_heat = reprap ? "G10 P0 S200\n" :
        std::string("M104 S200") + (semm || klipper ? "\n" : " T0\n");
    const std::string second_heat = reprap ? "G10 P1 S230\n" :
        std::string("M104 S230") + (semm || klipper ? "\n" : " T1\n");
    CHECK(count_occurrences(output, first_heat) == (semm ? 2 : 1));
    CHECK(count_occurrences(output, second_heat) == 1);
    const size_t first = output.find(first_heat);
    const size_t second = output.find(second_heat);
    REQUIRE(first != std::string::npos);
    REQUIRE(second != std::string::npos);
    CHECK(first < second);
    if (semm)
        CHECK(output.find(first_heat, second) != std::string::npos);
    if (!reprap && semm) {
        std::istringstream lines(output);
        std::string line;
        while (std::getline(lines, line))
            if (line.rfind("M104 ", 0) == 0 || line.rfind("M109 ", 0) == 0)
                CHECK(line.find(" T") == std::string::npos);
        CHECK(output.find("M109 S230\n") != std::string::npos);
    }
    if (reprap || sprinter) {
        CHECK(output.find(semm ? "M572 S0.06\n" : "M572 D1 S0.06\n") != std::string::npos);
        if (semm)
            CHECK(output.find("M572 D") == std::string::npos);
    }
    if (reprap) {
        CHECK(output.find(semm ? "M106 P0 S0.6\n" : "M106 P1 S0.6\n") != std::string::npos);
        CHECK(output.find("M116 P1\n") != std::string::npos);
        if (semm)
            CHECK(output.find("M106 P1") == std::string::npos);
    }
    if (klipper) {
        CHECK(output.find(semm ? "T1\n" : "ACTIVATE_EXTRUDER EXTRUDER=extruder1\n") != std::string::npos);
        CHECK(output.find(semm ? "SET_PRESSURE_ADVANCE ADVANCE=0.06\n" :
            "SET_PRESSURE_ADVANCE ADVANCE=0.06 EXTRUDER=extruder1\n") != std::string::npos);
        if (semm) {
            CHECK(output.find("ACTIVATE_EXTRUDER") == std::string::npos);
            CHECK(output.find("EXTRUDER=") == std::string::npos);
        }
    } else {
        CHECK(output.find("T1\n") != std::string::npos);
    }
}

// Compare emitted coordinates across A/B/A selections in both physical-tool models and E modes.
TEST_CASE("Built-in firmware resets absolute E immediately after every emitted tool selection",
          "[plugins][gcode][firmware][dialects][toolchange-reset]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool semm = GENERATE(false, true);
    const bool relative_e = GENERATE(false, true);
    const char *firmware_id = GENERATE("marlin", "marlin2",
        "prusa", "sprinter", "reprapfirmware", "klipper",
        "bambu");
    CAPTURE(semm, relative_e, firmware_id);
    Print print;
    configure_standard_firmware(print, 2);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.single_extruder_multi_material.value = semm;
    config.use_relative_e_distances.value = relative_e;
    config.start_gcode_manual.value = true;
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.front().layers.front();
    const std::vector<double> requested_e{1.25, 2.5, 0.75};
    for (size_t idx = 0; idx < requested_e.size(); ++idx) {
        layer.tool_groups.emplace_back();
        PrintingToolGroup &group = layer.tool_groups.back();
        group.extruder_id = idx == 1 ? 1 : 0;
        std::unique_ptr<ExtrusionPath> path = make_firmware_path(
            ArcPolyline(Points{Point::new_scale(double(idx), 0.0), Point::new_scale(double(idx + 1), 0.0)}),
            20.f, 500.f);
        path->add_property(ExtrusionPropertySpecialCommand(
            ExtrusionPropertySpecialCommand::Code::EXTRUSION, requested_e[idx]));
        append_path_extrusion(group, std::move(path));
    }
    const std::string output = export_with_firmware(print, firmware_id);
    INFO(output);
    std::istringstream lines(output);
    std::string line;
    std::vector<double> encoded_e;
    size_t selections = 0;
    while (std::getline(lines, line)) {
        if (line == "T0" || line == "T1" || line.rfind("ACTIVATE_EXTRUDER ", 0) == 0) {
            ++selections;
            if (!relative_e) {
                std::string reset;
                REQUIRE(bool(std::getline(lines, reset)));
                CHECK(reset == "G92 E0");
            }
        }
        if (line.rfind("G1 ", 0) == 0) {
            const size_t e_pos = line.find(" E");
            if (e_pos != std::string::npos)
                encoded_e.push_back(std::stod(line.substr(e_pos + 2)));
        }
    }
    CHECK(selections == 3);
    CHECK(count_occurrences(output, "G92 E0\n") == (relative_e ? 0 : selections));
    REQUIRE(encoded_e.size() == requested_e.size());
    for (size_t idx = 0; idx < requested_e.size(); ++idx)
        CHECK(encoded_e[idx] == Approx(requested_e[idx]));
}

// Each built-in provider explicitly declares the typed setting used by its shared-extruder model.
TEST_CASE("Built-in firmware providers declare their SEMM dependency",
          "[plugins][gcode][firmware][semm]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    for (const char *id : {"marlin", "marlin2", "prusa",
                           "sprinter", "reprapfirmware", "klipper",
                           "bambu"}) {
        const Plugin *plugin = Orchestrator::instance().get_plugin(id);
        REQUIRE(plugin != nullptr);
        size_t declared = 0;
        for (const Plugin::UsedConfigKey &key : plugin->get_used_config_keys()) {
            if (key.key != "single_extruder_multi_material")
                continue;
            ++declared;
            CHECK(key.type == RAW_CO_BOOL);
            CHECK(key.container_type == RAW_CONTAINER_TYPE_NONE);
            CHECK(key.option_preset_type == RAW_PRESET_TYPE_NONE);
        }
        CHECK(declared == 1);
    }
}

// SEMM fan routing uses the physical tool's declared mapping rather than logical filament IDs.
TEST_CASE("RepRap SEMM fan output preserves the physical tool zero mapping",
          "[plugins][gcode][firmware][dialects][reprap][semm]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize("single_extruder_multi_material", "1");
    RepRapReaderStateProbe firmware;
    firmware.setup(slic3r_api::Config(ApiHost::to_config_handle(&config)));
    const orchestrator_runtime_id reader_state_id = orchestrator_register_runtime_id(
        reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()),
        GCODE_READER_STATE_ID_TYPE,
        slic3r_api::GCodeGeneration::Reader::REPRAP_READER_STATE_NAME);
    firmware.set_reader_state_id(reader_state_id);
    slic3r_api::GCodeGeneration::Reader::RepRapGCodeReaderState mapped;
    mapped.tools[0].fans = {2, 3};
    mapped.tools[1].fans = {4};
    slic3r_api::GCodeStateTracker tracker;
    tracker.set_reader_id(reader_state_id);
    mapped.write_to(tracker, reader_state_id);
    firmware.synchronize_reader_private_state(tracker);
    CHECK(firmware.encode_fan(1, 50.0) == "M106 P2 S0.5\nM106 P3 S0.5\n");
}

TEST_CASE("Built-in firmware acceleration acknowledgements match their physical registers",
          "[plugins][gcode][firmware][dialects][acceleration]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    const char *const independent_dialects[] = {
        "marlin2",
        "prusa",
        "reprapfirmware",
        "bambu"
    };
    for (const char *firmware_id : independent_dialects) {
        INFO(firmware_id);
        Print print;
        configure_standard_firmware(print);
        PrintingPlan &plan = print.mutable_printing_plan();
        plan.groups.emplace_back();
        plan.groups.front().layers.emplace_back();
        PrintingLayerGroup &layer = plan.groups.front().layers.front();
        layer.tool_groups.emplace_back();
        PrintingToolGroup &tool_group = layer.tool_groups.front();
        tool_group.extruder_id = 0;
        append_path_extrusion(tool_group, make_firmware_path(
            ArcPolyline(Points{Point(scale_i(0.0), scale_i(0.0)), Point(scale_i(1.0), scale_i(0.0))}),
            20.f, 500.f));
        append_path_extrusion(tool_group, make_firmware_travel(
            ArcPolyline(Points{Point(scale_i(1.0), scale_i(0.0)), Point(scale_i(2.0), scale_i(0.0))}),
            80.f, 900.f));
        append_path_extrusion(tool_group, make_firmware_path(
            ArcPolyline(Points{Point(scale_i(2.0), scale_i(0.0)), Point(scale_i(3.0), scale_i(0.0))}),
            20.f, 500.f));

        const std::string output = export_with_firmware(print, firmware_id);
        INFO(output);
        CHECK(count_occurrences(output, "M204 P500\n") == 1);
        CHECK(count_occurrences(output, "M204 T900\n") == 1);
    }

    struct SharedAccelerationExpectation
    {
        const char *id;
        const char *print_command;
        const char *travel_command;
    };
    const SharedAccelerationExpectation shared_dialects[] = {
        {"marlin", "M204 S500\n", "M204 S900\n"},
        {"sprinter", "M204 P500\n", "M204 P900\n"},
        {"klipper", "M204 S500\n", "M204 S900\n"}
    };
    for (const SharedAccelerationExpectation &expectation : shared_dialects) {
        INFO(expectation.id);
        Print print;
        configure_standard_firmware(print);
        PrintingPlan &plan = print.mutable_printing_plan();
        plan.groups.emplace_back();
        plan.groups.front().layers.emplace_back();
        PrintingLayerGroup &layer = plan.groups.front().layers.front();
        layer.tool_groups.emplace_back();
        PrintingToolGroup &tool_group = layer.tool_groups.front();
        tool_group.extruder_id = 0;
        append_path_extrusion(tool_group, make_firmware_path(
            ArcPolyline(Points{Point(scale_i(0.0), scale_i(0.0)), Point(scale_i(1.0), scale_i(0.0))}),
            20.f, 500.f));
        append_path_extrusion(tool_group, make_firmware_travel(
            ArcPolyline(Points{Point(scale_i(1.0), scale_i(0.0)), Point(scale_i(2.0), scale_i(0.0))}),
            80.f, 900.f));
        append_path_extrusion(tool_group, make_firmware_path(
            ArcPolyline(Points{Point(scale_i(2.0), scale_i(0.0)), Point(scale_i(3.0), scale_i(0.0))}),
            20.f, 500.f));

        const std::string output = export_with_firmware(print, expectation.id);
        INFO(output);
        CHECK(count_occurrences(output, expectation.print_command) == 2);
        CHECK(count_occurrences(output, expectation.travel_command) == 1);

        // Even equal logical values are encoded again after the movement
        // category changes: the shared command acknowledges the current
        // category and explicitly invalidates the opposite one.
        Print equal_print;
        configure_standard_firmware(equal_print);
        PrintingPlan &equal_plan = equal_print.mutable_printing_plan();
        equal_plan.groups.emplace_back();
        equal_plan.groups.front().layers.emplace_back();
        PrintingLayerGroup &equal_layer = equal_plan.groups.front().layers.front();
        equal_layer.tool_groups.emplace_back();
        PrintingToolGroup &equal_tool_group = equal_layer.tool_groups.front();
        equal_tool_group.extruder_id = 0;
        append_path_extrusion(equal_tool_group, make_firmware_path(
            ArcPolyline(Points{Point(scale_i(0.0), scale_i(0.0)), Point(scale_i(1.0), scale_i(0.0))}),
            20.f, 500.f));
        append_path_extrusion(equal_tool_group, make_firmware_travel(
            ArcPolyline(Points{Point(scale_i(1.0), scale_i(0.0)), Point(scale_i(2.0), scale_i(0.0))}),
            80.f, 500.f));
        append_path_extrusion(equal_tool_group, make_firmware_path(
            ArcPolyline(Points{Point(scale_i(2.0), scale_i(0.0)), Point(scale_i(3.0), scale_i(0.0))}),
            20.f, 500.f));

        const std::string equal_output = export_with_firmware(equal_print, expectation.id);
        INFO(equal_output);
        CHECK(count_occurrences(equal_output, expectation.print_command) == 3);
    }
}

TEST_CASE("Shared acceleration registers switch only when print acceleration can be restored",
          "[plugins][gcode][firmware][dialects][acceleration][optional]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    struct SharedAccelerationExpectation
    {
        const char *id;
        const char *print_command;
        const char *travel_command;
    };
    const SharedAccelerationExpectation shared_dialects[] = {
        {"marlin", "M204 S500\n", "M204 S900\n"},
        {"sprinter", "M204 P500\n", "M204 P900\n"},
        {"klipper", "M204 S500\n", "M204 S900\n"}
    };

    SECTION("the machine extrusion limit restores the shared register") {
        for (const SharedAccelerationExpectation &expectation : shared_dialects) {
            INFO(expectation.id);
            Print print;
            configure_standard_firmware(print);
            PrintConfig &config = const_cast<PrintConfig &>(print.config());
            config.machine_limits_usage.value = MachineLimitsUsage::Limits;
            config.machine_max_acceleration_extruding.set(std::vector<double>{500.0});
            append_optional_acceleration_sequence(print);

            const std::string output = export_with_firmware(print, expectation.id);
            INFO(output);
            const size_t travel_pos = output.find(expectation.travel_command);
            const size_t print_pos = output.find(expectation.print_command);
            REQUIRE(travel_pos != std::string::npos);
            REQUIRE(print_pos != std::string::npos);
            CHECK(travel_pos < print_pos);
            CHECK(count_occurrences(output, expectation.travel_command) == 1);
            CHECK(count_occurrences(output, expectation.print_command) == 1);
        }
    }

    SECTION("Ignore mode does not use the machine limit as a fallback") {
        for (const SharedAccelerationExpectation &expectation : shared_dialects) {
            INFO(expectation.id);
            Print print;
            configure_standard_firmware(print);
            PrintConfig &config = const_cast<PrintConfig &>(print.config());
            config.machine_limits_usage.value = MachineLimitsUsage::Ignore;
            config.machine_max_acceleration_extruding.set(std::vector<double>{500.0});
            append_optional_acceleration_sequence(print);

            const std::string output = export_with_firmware(print, expectation.id);
            INFO(output);
            CHECK(output.find("M204") == std::string::npos);
        }
    }

    SECTION("an independent travel register remains usable without print acceleration") {
        Print print;
        configure_standard_firmware(print);
        PrintConfig &config = const_cast<PrintConfig &>(print.config());
        config.machine_limits_usage.value = MachineLimitsUsage::Ignore;
        append_optional_acceleration_sequence(print);

        const std::string output = export_with_firmware(print, "marlin2");
        INFO(output);
        CHECK(count_occurrences(output, "M204 T900\n") == 1);
        CHECK(output.find("M204 P") == std::string::npos);
    }

    SECTION("a non-positive extrusion limit cannot restore the shared register") {
        Print print;
        configure_standard_firmware(print);
        PrintConfig &config = const_cast<PrintConfig &>(print.config());
        config.machine_limits_usage.value = MachineLimitsUsage::Limits;
        config.machine_max_acceleration_extruding.set(std::vector<double>{0.0});
        append_optional_acceleration_sequence(print, false);

        const std::string output = export_with_firmware(print, "marlin");
        INFO(output);
        CHECK(output.find("M204") == std::string::npos);
    }
}
TEST_CASE("Klipper uses configured extruder names and I J arc centers",
          "[plugins][gcode][firmware][dialects][klipper]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    configure_standard_firmware(print, 2);
    PrintConfig &print_config = const_cast<PrintConfig &>(print.config());
    print_config.tool_name.set(
        std::vector<std::string>{"left_extruder", "right_extruder"});
    print_config.tool_name.set_enabled(true);

    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.front().layers.front();
    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool_group = layer.tool_groups.front();
    tool_group.extruder_id = 1;
    Geometry::ArcWelder::Path arc_path;
    arc_path.emplace_back(Point(scale_i(0.0), scale_i(0.0)));
    arc_path.emplace_back(
        Point(scale_i(10.0), scale_i(0.0)),
        float(scale_i(5.0)),
        Geometry::ArcWelder::Orientation::CCW);
    append_path_extrusion(
        tool_group,
        make_firmware_path(ArcPolyline(arc_path), 25.f, 600.f, 0.03f));

    const std::string output = export_with_firmware(print, "klipper");
    INFO(output);
    CHECK(output.find("ACTIVATE_EXTRUDER EXTRUDER=right_extruder\n") != std::string::npos);
    CHECK(output.find("SET_PRESSURE_ADVANCE ADVANCE=0.03 EXTRUDER=right_extruder\n") !=
          std::string::npos);
    CHECK(output.find("G3 X10 I") != std::string::npos);
    CHECK(output.find(" J") != std::string::npos);
    CHECK(output.find(" R5") == std::string::npos);
}
TEST_CASE("Radius-arc firmware dialects preserve R-form movements",
          "[plugins][gcode][firmware][dialects][arcs]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    const char *const firmware_ids[] = {
        "marlin",
        "marlin2",
        "prusa",
        "reprapfirmware",
        "sprinter",
        "bambu"
    };
    for (const char *firmware_id : firmware_ids) {
        INFO(firmware_id);
        Print print;
        configure_standard_firmware(print);
        PrintingPlan &plan = print.mutable_printing_plan();
        plan.groups.emplace_back();
        plan.groups.front().layers.emplace_back();
        PrintingLayerGroup &layer = plan.groups.front().layers.front();
        layer.tool_groups.emplace_back();
        PrintingToolGroup &tool_group = layer.tool_groups.front();
        tool_group.extruder_id = 0;

        Geometry::ArcWelder::Path arc_path;
        arc_path.emplace_back(Point(scale_i(0.0), scale_i(0.0)));
        arc_path.emplace_back(
            Point(scale_i(10.0), scale_i(0.0)),
            float(scale_i(5.0)),
            Geometry::ArcWelder::Orientation::CCW);
        append_path_extrusion(
            tool_group,
            make_firmware_path(ArcPolyline(arc_path), 25.f, 600.f));

        const std::string output = export_with_firmware(print, firmware_id);
        INFO(output);
        CHECK(output.find("G3 X10 R5 E") != std::string::npos);
        CHECK(output.find(" I") == std::string::npos);
        CHECK(output.find(" J") == std::string::npos);
    }
}
TEST_CASE("Unsupported dialect operations remain explicit G-code comments",
          "[plugins][gcode][firmware][dialects]")
{
    SprinterFirmwareProbe sprinter;
    KlipperFirmwareProbe klipper;

    CHECK(sprinter.encode_chamber_temperature(50, false) ==
          "; Unsupported firmware operation: chamber temperature\n");
    CHECK(sprinter.encode_extruder_current(0, 650.0) ==
          "; Unsupported firmware operation: extruder current\n");
    CHECK(klipper.encode_chamber_temperature(50, false) ==
          "; Unsupported firmware operation: chamber temperature\n");
    CHECK(klipper.encode_extruder_current(0, 650.0) ==
          "; Unsupported firmware operation: extruder current\n");
}
// Shared fan history changes across tools; independent heater caches survive coordinate-only resets.
TEST_CASE("Marlin 2 shares fan state but keeps heater state per extruder",
          "[plugins][gcode][firmware]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    select_printing_plan_firmware(print);
    configure_standard_firmware(print, 2);
    // Isolate runtime command encoding from automatic startup requests.
    const_cast<PrintConfig &>(print.config()).start_gcode_manual.value = true;
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.front().layers.front();

    append_multi_tool_process_sequence(layer);

    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    try {
        Steps::StepGenerateGcode::run_step(Orchestrator::instance(), print, output_path.string());
        const std::string output = read_text_file(output_path);
        INFO(output);
        CHECK(count_command_lines(output, "T0") == 2);
        CHECK(count_command_lines(output, "T1") == 1);
        CHECK(count_occurrences(output, "M104 S200 T0\n") == 1);
        CHECK(count_occurrences(output, "M104 S210 T1\n") == 1);
        CHECK(count_occurrences(output, "M106 S51\n") == 2);
        CHECK(count_occurrences(output, "M106 S77\n") == 1);
        CHECK(count_occurrences(output, "M900 K0.01\n") == 1);
        CHECK(count_occurrences(output, "M900 K0.02\n") == 1);

        // Tool zero resumes its heater history but starts at a zero E origin.
        // The fan is re-encoded because M106 controls shared hardware whose
        // last value was changed while tool one was active.
        CHECK(output.find("G1 X2 E.03326") != std::string::npos);
        CHECK(output.find("G1 X3 E.03326") != std::string::npos);
    } catch (...) {
        remove_output_pair(output_path);
        throw;
    }
    remove_output_pair(output_path);
}
TEST_CASE("RepRapFirmware keeps addressed heaters and fans per extruder",
          "[plugins][gcode][firmware][dialects][reprap]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    configure_standard_firmware(print, 2);
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    append_multi_tool_process_sequence(plan.groups.front().layers.front());

    const std::string output = export_with_firmware(print, "reprapfirmware");
    INFO(output);
    CHECK(count_occurrences(output, "G10 P0 S200\n") == 1);
    CHECK(count_occurrences(output, "G10 P1 S210\n") == 1);
    CHECK(count_occurrences(output, "M106 P0 S0.2\n") == 1);
    CHECK(count_occurrences(output, "M106 P1 S0.3\n") == 1);
}

namespace {

// Exposes the protected pressure advance encoder of the Bambu firmware.
class BambuFirmwareProbe final :
    public slic3r_api::GCodeGeneration::Firmware::BambuGCodeFirmwareSession
{
public:
    using BambuGCodeFirmwareSession::encode_pressure_advance;
};

// Build a hand-made object catalog: entry i is a real object when
// real_instances[i] is true, a synthetic one otherwise (all real entries share the
// one source object of the cube Print).
PrintingPlan build_label_catalog(Print &print, const std::vector<bool> &real_instances)
{
    const Polygon outline(Points{Point::new_scale(0., 0.), Point::new_scale(1., 0.),
                                 Point::new_scale(1., 1.), Point::new_scale(0., 1.)});
    PrintingPlan plan;
    for (size_t idx = 0; idx < real_instances.size(); ++idx) {
        if (!real_instances[idx]) {
            plan.add_synthetic_object_instance("Tower " + std::to_string(idx), outline);
            continue;
        }
        PrintingObjectInstance instance;
        instance.object = &print.object(0);
        instance.object_instance_idx = 0;
        instance.name = "Object " + std::to_string(idx);
        instance.projected_outline_center = Point(0, 0);
        instance.projected_outline = outline;
        plan.object_instances.push_back(std::move(instance));
    }
    return plan;
}

// Create an owned support deposit at (1.25, 2.5), the only printed point at that Y, with printable process values.
std::unique_ptr<ExtrusionEntity> make_bambu_deposit()
{
    std::unique_ptr<ExtrusionEntity> leaf(new ExtrusionEntity(false,
        ArcPolyline(Points{Point::new_scale(1.25, 2.5)})));
    leaf->add_property(ExtrusionAttributes(ExtrusionRole::SupportMaterial, ExtrusionFlow(0.08, 0.4f, 0.2f)));
    leaf->add_property(ExtrusionPropertySpeed(20.f, 500.f));
    leaf->add_property(ExtrusionPropertySpecialCommand(ExtrusionPropertySpecialCommand::Code::DEPOSIT, 0.4));
    return leaf;
}

// Generate two labelled objects per layer with a fan threshold and an optional
// ordered tool event after object zero's geometry, optionally followed by another
// path (or, with deposit_after_change, a stationary deposit) in the same tree. Both tools use the supplied
// nonnegative retraction depth.
// Local owners outlive all views;
// setup or output failures propagate, and no plugin activation state is changed.
std::string run_bambu_cancel_fan(bool ordered_change, double retract_length = 0.,
                                 bool geometry_after_change = false, bool deposit_after_change = false);

// Build the catalogue and visit real session callbacks, deliberately leaving the
// previous layer's last object open until a command or transition closes it.
std::string run_bambu_cancel_fan(bool ordered_change, double retract_length,
                                 bool geometry_after_change, bool deposit_after_change)
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_extruders(2);
    config.set_deserialize_strict({{"platter_fan", "1"}, {"platter_fan_speed", "70,80"},
        {"disable_fan_first_layers", "1,0"}, {"gcode_label_objects", "firmware"}});
    Model model;
    Print print;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
    configure_standard_firmware(print, 2);
    PrintConfig &print_config = const_cast<PrintConfig &>(print.config());
    print_config.retract_length.set(std::vector<double>{retract_length, retract_length});
    print_config.use_relative_e_distances.value = false;
    PrintingPlan plan = build_label_catalog(print, {true, true});
    plan.groups.emplace_back();
    for (uint32_t layer_idx = 0; layer_idx < 2; ++layer_idx) {
        plan.groups.front().layers.emplace_back();
        PrintingLayerGroup &layer = plan.groups.front().layers.back();
        layer.print_z = scale_i(0.2 * (layer_idx + 1));
        layer.tool_groups.emplace_back();
        PrintingToolGroup &tool = layer.tool_groups.back();
        tool.extruder_id = 0;
        for (uint32_t object = 0; object < 2; ++object) {
            std::unique_ptr<ExtrusionEntity> root(new ExtrusionEntity(true));
            root->append_child(make_firmware_path(ArcPolyline(Points{
                Point::new_scale(double(object * 2), double(layer_idx)),
                Point::new_scale(double(object * 2 + 1), double(layer_idx))}), 20.f, 500.f));
            if (ordered_change && layer_idx == 1 && object == 0) {
                root->append_child(std::unique_ptr<ExtrusionEntity>(new ExtrusionNop(
                    ExtrusionPropertySpecialCommand(ExtrusionPropertySpecialCommand::Code::TOOLCHANGE, 1.))));
                if (geometry_after_change)
                    root->append_child(make_firmware_path(ArcPolyline(Points{
                        Point::new_scale(0., 1.), Point::new_scale(1., 1.)}), 20.f, 500.f));
                if (deposit_after_change)
                    root->append_child(make_bambu_deposit());
            }
            append_path_extrusion(tool, std::move(root));
            tool.extrusions.back().printing_catalog_id = object;
        }
    }
    slic3r_api::GCodeGeneration::Firmware::BambuGCodeFirmwareSession session;
    const slic3r_api::ConstPrintingPlan view(reinterpret_cast<const printing_plan_handle *>(&plan));
    std::string output = session.begin_print(
        slic3r_api::Print(reinterpret_cast<const print_handle *>(&print)), view);
    output += session.begin_group(view.group(0));
    for (uint32_t idx = 0; idx < 2; ++idx) {
        const slic3r_api::ConstPrintingLayerGroup layer = view.group(0).layer_group(idx);
        output += session.begin_layer(layer);
        const slic3r_api::ConstPrintingToolGroup tool = layer.tool_group(0);
        output += session.begin_tool_group(tool);
        for (uint32_t object = 0; object < tool.extrusion_count(); ++object)
            output += session.write_extrusion(tool.extrusion(object));
        output += session.end_tool_group();
        output += session.end_layer();
    }
    output += session.end_group();
    return output + session.end_print();
}

// Drive a Bambu session over a hand-built catalog and return its text. Each
// printed object gets one 1 mm extrusion. `print_order` lists the catalog ids in
// printing order; empty means every real entry in catalog order.
std::string run_bambu_labels(const std::vector<bool> &real_instances,
                             LabelObjectsStyle style,
                             bool relative_e,
                             std::vector<uint32_t> print_order = {})
{
    Model model;
    Print print;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model);
    configure_standard_firmware(print);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.gcode_label_objects.value = style;
    config.use_relative_e_distances.value = relative_e;
    // These tests are about the labels: no block retraction (it has its own tests).
    config.retract_length.set(std::vector<double>{0.0});
    PrintingPlan plan = build_label_catalog(print, real_instances);
    if (print_order.empty())
        for (uint32_t idx = 0; idx < real_instances.size(); ++idx)
            if (real_instances[idx])
                print_order.push_back(idx);

    PrintingToolGroup tool;
    tool.extruder_id = 0;
    size_t printed = 0;
    for (const uint32_t idx : print_order) {
        append_path_extrusion(tool, make_firmware_path(
            ArcPolyline(Points{Point::new_scale(double(printed), 0.), Point::new_scale(double(printed + 1), 0.)}),
            20.f, 500.f));
        tool.extrusions.back().printing_catalog_id = idx;
        ++printed;
    }

    slic3r_api::GCodeGeneration::Firmware::BambuGCodeFirmwareSession session;
    std::string output = session.begin_print(
        slic3r_api::Print(reinterpret_cast<const print_handle *>(&print)),
        slic3r_api::ConstPrintingPlan(reinterpret_cast<const printing_plan_handle *>(&plan)));
    output += session.begin_tool_group(slic3r_api::ConstPrintingToolGroup(
        reinterpret_cast<const printing_tool_group_handle *>(&tool)));
    for (const PrintingExtrusion &extrusion : tool.extrusions)
        output += session.write_extrusion(slic3r_api::ConstPrintingExtrusion(
            reinterpret_cast<const printing_extrusion_handle *>(&extrusion)));
    output += session.end_print();
    return output;
}

// Drive a Bambu session over one labelled object whose whole tree is a single stationary deposit, with the supplied
// retraction depth, absolute E and firmware labels. Owners outlive all views; setup or output failures propagate.
std::string run_bambu_deposit_block(double retract_length)
{
    Model model;
    Print print;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model);
    configure_standard_firmware(print);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.gcode_label_objects.value = LabelObjectsStyle::Firmware;
    config.use_relative_e_distances.value = false;
    config.retract_length.set(std::vector<double>{retract_length});
    PrintingPlan plan = build_label_catalog(print, {true});

    PrintingToolGroup tool;
    tool.extruder_id = 0;
    std::unique_ptr<ExtrusionEntity> root(new ExtrusionEntity(true));
    root->append_child(make_bambu_deposit());
    append_path_extrusion(tool, std::move(root));
    tool.extrusions.back().printing_catalog_id = 0;

    slic3r_api::GCodeGeneration::Firmware::BambuGCodeFirmwareSession session;
    std::string output = session.begin_print(
        slic3r_api::Print(reinterpret_cast<const print_handle *>(&print)),
        slic3r_api::ConstPrintingPlan(reinterpret_cast<const printing_plan_handle *>(&plan)));
    output += session.begin_tool_group(slic3r_api::ConstPrintingToolGroup(
        reinterpret_cast<const printing_tool_group_handle *>(&tool)));
    for (const PrintingExtrusion &extrusion : tool.extrusions)
        output += session.write_extrusion(slic3r_api::ConstPrintingExtrusion(
            reinterpret_cast<const printing_extrusion_handle *>(&extrusion)));
    output += session.end_print();
    return output;
}

// E-only semantic event: a retract to `value` mm or a full unretract, written as a
// travel-role event the way the retraction plugins do.
std::unique_ptr<ExtrusionNop> make_axis_event(bool retract, double value)
{
    std::unique_ptr<ExtrusionNop> event(new ExtrusionNop());
    event->add_property(ExtrusionPropertyExtrusionAxis(
        retract ? C_EXTRUSION_AXIS_OPERATION_RETRACT_TO : C_EXTRUSION_AXIS_OPERATION_UNRETRACT, value));
    event->add_property(ExtrusionAttributes(ExtrusionRole(ExtrusionRole::Travel |
        (retract ? ExtrusionRoleModifier::ERM_Retract : ExtrusionRoleModifier::ERM_Unretract))));
    return event;
}

// Options of run_bambu_second_layer().
struct SecondLayerRun
{
    std::string variables;                   // printer_custom_variables
    double retract_length = 2.;
    uint32_t groups = 1;                     // sequential groups of three layers each
    bool firmware_retraction = false;
    bool relative_e = false;
    bool repay_before_second_layer = false;  // a layer event repays the retraction first
    uint32_t objects = 2;                    // labelled objects per layer
    bool labels = true;                      // gcode_label_objects = firmware
    bool layer_script = true;                // a layer_gcode marker before each layer
    std::vector<double> lengths;             // retract_length per tool when not empty (one tool each)
    bool ordered_toolchange = false;         // layer 2: group of tool 1 whose first object starts with TOOLCHANGE 1
    bool second_tool_group = false;          // layer 2: an extra tool group of tool 1 after the first
};

// Drive a Bambu session over `groups` groups of three layers, each layer holding
// `objects` labelled paths and a layer script marker `;LAYER_GCODE <n>` (the script
// comes between begin_layer() and begin_tool_group(), as in the writer). Every group
// is announced by `;GROUP <n>`. The groups and the layers are real callbacks of the
// session; no plugin state is changed.
std::string run_bambu_second_layer(const SecondLayerRun &run)
{
    Model model;
    Print print;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model);
    configure_standard_firmware(print, run.lengths.empty() ? uint16_t(1) : uint16_t(run.lengths.size()));
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.gcode_label_objects.value = run.labels ? LabelObjectsStyle::Firmware : LabelObjectsStyle::Disabled;
    config.use_relative_e_distances.value = run.relative_e;
    config.use_firmware_retraction.value = run.firmware_retraction;
    config.retract_length.set(run.lengths.empty() ? std::vector<double>{run.retract_length} : run.lengths);
    config.printer_custom_variables.value = run.variables;
    PrintingPlan plan = build_label_catalog(print, std::vector<bool>(run.objects, true));
    for (uint32_t group_idx = 0; group_idx < run.groups; ++group_idx) {
        plan.groups.emplace_back();
        for (uint32_t layer_idx = 0; layer_idx < 3; ++layer_idx) {
            plan.groups.back().layers.emplace_back();
            PrintingLayerGroup &layer = plan.groups.back().layers.back();
            layer.print_z = scale_i(0.2 * (layer_idx + 1));
            ExtrusionNop marker;
            marker.add_property(ExtrusionPropertyCustomGcodeText(";LAYER_GCODE " + std::to_string(layer_idx)));
            if (run.layer_script)
                layer.events.append_before(marker);
            if (run.repay_before_second_layer && layer_idx == 1)
                layer.events.append_before(*make_axis_event(false, 0.0));
            layer.tool_groups.emplace_back();
            PrintingToolGroup &tool = layer.tool_groups.back();
            const bool ordered = run.ordered_toolchange && layer_idx == 1;
            tool.extruder_id = ordered ? 1 : 0;
            for (uint32_t object = 0; object < run.objects; ++object) {
                std::unique_ptr<ExtrusionEntity> root(new ExtrusionEntity(true));
                if (ordered && object == 0)
                    root->append_child(std::unique_ptr<ExtrusionEntity>(new ExtrusionNop(
                        ExtrusionPropertySpecialCommand(ExtrusionPropertySpecialCommand::Code::TOOLCHANGE, 1.))));
                root->append_child(make_firmware_path(ArcPolyline(Points{
                    Point::new_scale(double(object * 2), double(layer_idx)),
                    Point::new_scale(double(object * 2 + 1), double(layer_idx))}), 20.f, 500.f));
                append_path_extrusion(tool, std::move(root));
                tool.extrusions.back().printing_catalog_id = object;
            }
            if (run.second_tool_group && layer_idx == 1) {
                layer.tool_groups.emplace_back();
                PrintingToolGroup &extra = layer.tool_groups.back();
                extra.extruder_id = 1;
                append_path_extrusion(extra, make_firmware_path(ArcPolyline(Points{
                    Point::new_scale(8., double(layer_idx)), Point::new_scale(9., double(layer_idx))}), 20.f, 500.f));
                extra.extrusions.back().printing_catalog_id = 0;
            }
        }
    }
    slic3r_api::GCodeGeneration::Firmware::BambuGCodeFirmwareSession session;
    const slic3r_api::ConstPrintingPlan view(reinterpret_cast<const printing_plan_handle *>(&plan));
    std::string output = session.begin_print(
        slic3r_api::Print(reinterpret_cast<const print_handle *>(&print)), view);
    for (uint32_t group_idx = 0; group_idx < view.group_count(); ++group_idx) {
        const slic3r_api::ConstPrintingGroup group = view.group(group_idx);
        output += ";GROUP " + std::to_string(group_idx) + "\n" + session.begin_group(group);
        for (uint32_t layer_idx = 0; layer_idx < group.layer_group_count(); ++layer_idx) {
            const slic3r_api::ConstPrintingLayerGroup layer = group.layer_group(layer_idx);
            output += session.begin_layer(layer);
            if (layer.events().has_before())
                output += session.write_event(layer.events().before());
            for (uint32_t tool_idx = 0; tool_idx < layer.tool_group_count(); ++tool_idx) {
                const slic3r_api::ConstPrintingToolGroup tool = layer.tool_group(tool_idx);
                output += session.begin_tool_group(tool);
                for (uint32_t idx = 0; idx < tool.extrusion_count(); ++idx)
                    output += session.write_extrusion(tool.extrusion(idx));
                output += session.end_tool_group();
            }
            output += session.end_layer();
        }
        output += session.end_group();
    }
    return output + session.end_print();
}

// Lines of a text, without their line break.
std::vector<std::string> split_lines(const std::string &text)
{
    std::vector<std::string> lines;
    std::istringstream stream(text);
    std::string line;
    while (std::getline(stream, line))
        lines.push_back(line);
    return lines;
}

// Index of the first line equal to `line` at or after `from`, or the line count.
size_t find_line(const std::vector<std::string> &lines, const std::string &line, size_t from = 0)
{
    return size_t(std::find(lines.begin() + std::min(from, lines.size()), lines.end(), line) - lines.begin());
}

// Two labelled objects in absolute E: object 0, an outside event that retracts by
// 0.8 mm, then object 1 whose tree starts with the unretract and extrudes. The
// retraction is explicit E, or G10/G11 when `firmware_retraction` is set.
std::string run_bambu_retraction_between_objects(bool firmware_retraction)
{
    Model model;
    Print print;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model);
    configure_standard_firmware(print);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.gcode_label_objects.value = LabelObjectsStyle::Firmware;
    config.use_firmware_retraction.value = firmware_retraction;
    // The retraction under test is the explicit event, not the block retraction.
    config.retract_length.set(std::vector<double>{0.0});
    PrintingPlan plan = build_label_catalog(print, {true, true});

    PrintingToolGroup tool;
    tool.extruder_id = 0;
    append_path_extrusion(tool, make_firmware_path(
        ArcPolyline(Points{Point::new_scale(0., 0.), Point::new_scale(1., 0.)}), 20.f, 500.f));
    tool.extrusions.back().printing_catalog_id = 0;
    std::unique_ptr<ExtrusionEntity> second(new ExtrusionEntity(true));
    second->append_child(make_axis_event(false, 0.0));
    second->append_child(make_firmware_path(
        ArcPolyline(Points{Point::new_scale(1., 0.), Point::new_scale(2., 0.)}), 20.f, 500.f));
    append_path_extrusion(tool, std::move(second));
    tool.extrusions.back().printing_catalog_id = 1;
    const std::unique_ptr<ExtrusionNop> retract = make_axis_event(true, 0.8);

    slic3r_api::GCodeGeneration::Firmware::BambuGCodeFirmwareSession session;
    std::string output = session.begin_print(
        slic3r_api::Print(reinterpret_cast<const print_handle *>(&print)),
        slic3r_api::ConstPrintingPlan(reinterpret_cast<const printing_plan_handle *>(&plan)));
    output += session.begin_tool_group(slic3r_api::ConstPrintingToolGroup(
        reinterpret_cast<const printing_tool_group_handle *>(&tool)));
    output += session.write_extrusion(slic3r_api::ConstPrintingExtrusion(
        reinterpret_cast<const printing_extrusion_handle *>(&tool.extrusions[0])));
    output += session.write_event(slic3r_api::ExtrusionEntity(
        reinterpret_cast<const extrusion_entity_handle *>(retract.get())));
    output += session.write_extrusion(slic3r_api::ConstPrintingExtrusion(
        reinterpret_cast<const printing_extrusion_handle *>(&tool.extrusions[1])));
    output += session.end_print();
    return output;
}

// Filament pushed by a G-code text, as the sum of the signed E deltas of its
// G0/G1/G2/G3 moves. Absolute E is rebased by each `G92 E0`. `largest_delta`
// receives the biggest |delta| of one move.
double net_filament(const std::string &text, bool relative_e, double &largest_delta)
{
    double position = 0.0;
    double total = 0.0;
    largest_delta = 0.0;
    std::istringstream stream(text);
    std::string line;
    while (std::getline(stream, line)) {
        if (line.rfind("G92 E0", 0) == 0) {
            position = 0.0;
            continue;
        }
        const bool move = line.size() > 3 && line[0] == 'G' && line[1] >= '0' && line[1] <= '3' && line[2] == ' ';
        const size_t e_pos = line.find(" E");
        if (!move || e_pos == std::string::npos)
            continue;
        const double value = std::stod(line.substr(e_pos + 2));
        const double delta = relative_e ? value : value - position;
        position = value;
        total += delta;
        largest_delta = std::max(largest_delta, std::abs(delta));
    }
    return total;
}

// E value of the first extruding G1 line at or after `from`, or -1 when none.
double first_extrusion_e_after(const std::string &text, size_t from)
{
    size_t line = text.find("\nG1 ", from);
    while (line != std::string::npos) {
        const size_t line_end = text.find('\n', line + 1);
        const size_t e_pos = text.find(" E", line);
        if (e_pos != std::string::npos && e_pos < line_end)
            return std::stod(text.substr(e_pos + 2));
        line = text.find("\nG1 ", line + 1);
    }
    return -1.0;
}


// Activates the production plugins missing from the plugin test runtime (retraction
// wipe, travel, travel avoiding perimeters, terminal retraction) for the lifetime of
// the object, and restores their previous state. Create it before print.process().
class ScopedProductionPlugins
{
public:
    // An unknown plugin id throws after the plugins already activated are restored
    // (the destructor does not run when the constructor throws).
    ScopedProductionPlugins()
    {
        Orchestrator &orchestrator = Orchestrator::instance();
        for (const char *id : ids()) {
            const bool was_active = orchestrator.is_plugin_active(id);
            if (!orchestrator.set_plugin_active(id, true)) {
                restore();
                throw std::runtime_error(std::string("production plugin not registered: ") + id);
            }
            m_previous.push_back(was_active);
        }
    }
    ~ScopedProductionPlugins() { restore(); }
    ScopedProductionPlugins(const ScopedProductionPlugins &) = delete;
    ScopedProductionPlugins &operator=(const ScopedProductionPlugins &) = delete;

private:
    // Put every plugin activated so far back in its previous state.
    void restore()
    {
        Orchestrator &orchestrator = Orchestrator::instance();
        for (size_t idx = 0; idx < m_previous.size(); ++idx)
            orchestrator.set_plugin_active(ids()[idx], m_previous[idx]);
        m_previous.clear();
    }
    static const std::vector<const char *> &ids()
    {
        static const std::vector<const char *> plugin_ids = {
            "layer_extrusion_edit.wipe.default", "layer_extrusion_edit.travel.default",
            "layer_extrusion_edit.travel.avoid_crossing_perimeters",
            "extrusion_edit.terminal_retraction.default"};
        return plugin_ids;
    }
    std::vector<bool> m_previous;
};

// How a hand-driven scenario changes tool: as the plan does (a TOOLCHANGE command in
// the events of the target tool group, executed after the previous block was closed
// by the outside event), by the automatic selection of begin_tool_group() (it writes
// T itself), or as a custom toolchange script does: the group carries a toolchange
// script event, so begin_tool_group() selects the tool without writing T, and the
// script's effect (the T line) is written afterwards as a text event. The script is not
// processed (the hand-driven session has no script processor): only its effect is.
enum class ToolSwitch { Semantic, Automatic, Custom };

// One object of a hand-driven block scenario: its tool, an optional script and an
// optional retraction event written outside any block before it (prior_retract >= 0,
// target in mm), and the retraction events inside its tree (an unretract before the
// path, a retract to retract_after >= 0 after it). A synthetic entry is a wipe tower:
// its path is printed outside every block.
struct BlockObject
{
    extruder_index_t tool = 0;
    double prior_retract = -1.0;
    bool unretract_first = false;
    double retract_after = -1.0;
    bool synthetic = false;
    bool prior_script = false;
};

// Exposes the filament counted for each tool by the session.
class BambuBlockProbe final :
    public slic3r_api::GCodeGeneration::Firmware::BambuGCodeFirmwareSession
{
public:
    double used_filament(size_t tool) { return extruders().at(tool).extrusion_axis().used_filament(); }
    double retracted(size_t tool) { return extruders().at(tool).extrusion_axis().retracted(); }
};

// Drive a Bambu session over one object per entry of `objects` (one 1 mm path each,
// printed in order). `lengths` is retract_length per tool; its size is the number of
// tools. A tool change is written the way `switch_mode` says; a prior retraction or
// script is an outside event, which closes the previous block as the plan does;
// otherwise one object follows the other directly. `used_per_tool`, when given,
// receives the filament counted for each tool, `final_retracted_per_tool` the
// retraction debt each tool ends with.
std::string run_bambu_blocks(const std::vector<double> &lengths,
                             bool relative_e,
                             bool firmware_retraction,
                             const std::vector<BlockObject> &objects,
                             LabelObjectsStyle style = LabelObjectsStyle::Firmware,
                             ToolSwitch switch_mode = ToolSwitch::Semantic,
                             std::vector<double> *used_per_tool = nullptr,
                             std::vector<double> *final_retracted_per_tool = nullptr)
{
    Model model;
    Print print;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model);
    configure_standard_firmware(print, uint16_t(lengths.size()));
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.gcode_label_objects.value = style;
    config.use_relative_e_distances.value = relative_e;
    config.use_firmware_retraction.value = firmware_retraction;
    config.retract_length.set(lengths);
    std::vector<bool> real_entries;
    for (const BlockObject &object : objects)
        real_entries.push_back(!object.synthetic);
    PrintingPlan plan = build_label_catalog(print, real_entries);

    std::vector<PrintingToolGroup> groups(objects.size());
    for (uint32_t idx = 0; idx < objects.size(); ++idx) {
        PrintingToolGroup &group = groups[idx];
        group.extruder_id = objects[idx].tool;
        ArcPolyline path(Points{Point::new_scale(double(idx), 0.), Point::new_scale(double(idx + 1), 0.)});
        if (!objects[idx].unretract_first && objects[idx].retract_after < 0.0) {
            append_path_extrusion(group, make_firmware_path(path, 20.f, 500.f));
        } else {
            std::unique_ptr<ExtrusionEntity> root(new ExtrusionEntity(true));
            if (objects[idx].unretract_first)
                root->append_child(make_axis_event(false, 0.0));
            root->append_child(make_firmware_path(path, 20.f, 500.f));
            if (objects[idx].retract_after >= 0.0)
                root->append_child(make_axis_event(true, objects[idx].retract_after));
            append_path_extrusion(group, std::move(root));
        }
        group.extrusions.back().printing_catalog_id = idx;
        if (switch_mode == ToolSwitch::Semantic && idx > 0 && objects[idx].tool != objects[idx - 1].tool) {
            ExtrusionNop toolchange(ExtrusionPropertySpecialCommand(
                ExtrusionPropertySpecialCommand::Code::TOOLCHANGE, double(objects[idx].tool)));
            group.events.append_before(toolchange);
        }
        if (switch_mode == ToolSwitch::Custom && idx > 0 && objects[idx].tool != objects[idx - 1].tool) {
            ExtrusionNop script;
            script.add_property(ExtrusionPropertyCustomGcodeText(
                ExtrusionPropertyCustomGcodeText::Code::SCRIPT, "T[next_extruder]",
                GCODE_SCRIPT_TYPE_TOOLCHANGE_GCODE));
            group.events.append_before(script);
        }
    }

    BambuBlockProbe session;
    std::string output = session.begin_print(
        slic3r_api::Print(reinterpret_cast<const print_handle *>(&print)),
        slic3r_api::ConstPrintingPlan(reinterpret_cast<const printing_plan_handle *>(&plan)));
    std::vector<std::unique_ptr<ExtrusionNop>> events;
    const auto write_outside = [&](std::unique_ptr<ExtrusionNop> event) {
        events.push_back(std::move(event));
        output += session.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(events.back().get())));
    };
    for (uint32_t idx = 0; idx < objects.size(); ++idx) {
        const slic3r_api::ConstPrintingToolGroup group_view(
            reinterpret_cast<const printing_tool_group_handle *>(&groups[idx]));
        output += session.begin_tool_group(group_view);
        const bool custom_change = switch_mode == ToolSwitch::Custom && idx > 0 &&
            objects[idx].tool != objects[idx - 1].tool;
        if (custom_change) {
            // the effect of the script: the T line, after the block was closed
            std::unique_ptr<ExtrusionNop> select(new ExtrusionNop());
            select->add_property(ExtrusionPropertyCustomGcodeText("T" + std::to_string(objects[idx].tool)));
            write_outside(std::move(select));
        } else if (group_view.events().has_before()) {
            output += session.write_event(group_view.events().before());
        }
        if (objects[idx].prior_script) {
            std::unique_ptr<ExtrusionNop> script(new ExtrusionNop());
            script->add_property(ExtrusionPropertyCustomGcodeText("; SCRIPT"));
            write_outside(std::move(script));
        }
        if (objects[idx].prior_retract >= 0.0) {
            std::unique_ptr<ExtrusionNop> marker(new ExtrusionNop());
            marker->add_property(ExtrusionPropertyCustomGcodeText("; BOUNDARY"));
            write_outside(std::move(marker));
            write_outside(make_axis_event(true, objects[idx].prior_retract));
        }
        output += session.write_extrusion(slic3r_api::ConstPrintingExtrusion(
            reinterpret_cast<const printing_extrusion_handle *>(&groups[idx].extrusions.front())));
    }
    output += session.end_print();
    if (used_per_tool != nullptr) {
        used_per_tool->clear();
        for (size_t tool = 0; tool < lengths.size(); ++tool)
            used_per_tool->push_back(session.used_filament(tool));
    }
    if (final_retracted_per_tool != nullptr) {
        final_retracted_per_tool->clear();
        for (size_t tool = 0; tool < lengths.size(); ++tool)
            final_retracted_per_tool->push_back(session.retracted(tool));
    }
    return output;
}

// E value of the first extrusion (a G1 line with E and X) at or after `from`, or -1.
double first_deposit_e_after(const std::string &text, size_t from)
{
    size_t line = text.find("\nG1 ", from);
    while (line != std::string::npos) {
        const size_t line_end = text.find('\n', line + 1);
        const size_t e_pos = text.find(" E", line);
        const size_t x_pos = text.find(" X", line);
        if (e_pos != std::string::npos && e_pos < line_end && x_pos != std::string::npos && x_pos < line_end)
            return std::stod(text.substr(e_pos + 2));
        line = text.find("\nG1 ", line + 1);
    }
    return -1.0;
}
// Value of every E-only move of a text: a line `G1 E<value> ...` without X or Y. In
// relative E it is the signed delta; in absolute E it is the E coordinate.
std::vector<double> e_only_moves(const std::string &text)
{
    std::vector<double> values;
    std::istringstream stream(text);
    std::string line;
    while (std::getline(stream, line)) {
        if (line.rfind("G1 E", 0) != 0 || line.find(" X") != std::string::npos ||
            line.find(" Y") != std::string::npos)
            continue;
        values.push_back(std::stod(line.substr(4)));
    }
    return values;
}

// One retraction event seen by trace_debt: the tool that was selected, and either an
// E-only move (kind 'E', signed delta) or a native command (kind 'R' for G10, 'U' for G11).
struct RetractionEvent
{
    int tool = 0;
    char kind = 'E';
    double value = 0.0;
};

// Retraction debt seen at the boundaries of the labelled blocks of a G-code text.
struct DebtTrace
{
    std::vector<double> at_block_start;       // debt of the active tool when an M624 is reached
    std::vector<double> at_block_end;         // when an M625 is reached
    std::vector<double> before_first_deposit; // before the first extrusion of each block
    std::vector<int> tool_at_block_start;     // active tool at each M624
    std::vector<int> tool_at_block_end;       // active tool at each M625
    size_t tool_selections_in_blocks = 0;     // T<n> lines written between an M624 and its M625
    std::vector<RetractionEvent> events;      // every E-only move and G10/G11, in order
    double final_debt = 0.0;
};

// Small executor of the E axis: follows M82/M83, G92 E, T<n> (one debt per tool),
// G10/G11 (debt = native_length / 0) and E words. A negative E delta is a retraction;
// a positive one repays the debt when it is an E-only move or lies between
// ;WIPE_START and ;WIPE_END; any other positive delta is a deposit, which the base
// session also counts as repaying the debt on its flow (the debt is then reduced by
// the deposited amount, never below 0).
DebtTrace trace_debt(const std::string &text, double native_length)
{
    DebtTrace trace;
    bool relative = false;
    bool in_wipe = false;
    bool awaiting_first_deposit = false;
    bool in_block = false;
    int tool = 0;
    double position = 0.0;
    std::vector<double> debts(16, 0.0);
    std::istringstream stream(text);
    std::string line;
    while (std::getline(stream, line)) {
        double &debt = debts[size_t(tool)];
        if (line == "M82")
            relative = false;
        else if (line == "M83")
            relative = true;
        else if (line.rfind("G92 E", 0) == 0)
            position = std::stod(line.substr(5));
        else if (line.size() >= 2 && line.size() <= 3 && line[0] == 'T' && std::isdigit((unsigned char)line[1])) {
            tool = std::stoi(line.substr(1));
            if (in_block)
                ++trace.tool_selections_in_blocks;
        }
        else if (line == "G10") {
            debt = native_length;
            trace.events.push_back({tool, 'R', 0.0});
        } else if (line == "G11") {
            debt = 0.0;
            trace.events.push_back({tool, 'U', 0.0});
        } else if (line == ";WIPE_START")
            in_wipe = true;
        else if (line == ";WIPE_END")
            in_wipe = false;
        else if (line.rfind("M624 ", 0) == 0) {
            trace.at_block_start.push_back(debt);
            trace.tool_at_block_start.push_back(tool);
            awaiting_first_deposit = true;
            in_block = true;
        } else if (line == "M625") {
            trace.at_block_end.push_back(debt);
            trace.tool_at_block_end.push_back(tool);
            in_block = false;
        }
        else if (line.size() > 3 && line[0] == 'G' && line[1] >= '0' && line[1] <= '3' && line[2] == ' ' &&
                 line.find(" E") != std::string::npos) {
            const double value = std::stod(line.substr(line.find(" E") + 2));
            const double delta = relative ? value : value - position;
            position = value;
            const bool has_xy = line.find(" X") != std::string::npos || line.find(" Y") != std::string::npos;
            if (!has_xy)
                trace.events.push_back({tool, 'E', delta});
            if (delta < 0.0)
                debt += -delta;
            else if (delta > 0.0 && (!has_xy || in_wipe))
                debt = std::max(0.0, debt - delta);
            else if (delta > 0.0) {
                if (awaiting_first_deposit) {
                    trace.before_first_deposit.push_back(debt);
                    awaiting_first_deposit = false;
                }
                debt = std::max(0.0, debt - delta);
            }
        }
    }
    trace.final_debt = debts[size_t(tool)];
    return trace;
}

// Number of E-only moves between each M625 and the next M624 (the gaps outside the
// blocks), in order.
std::vector<size_t> e_only_moves_in_gaps(const std::string &text)
{
    std::vector<size_t> counts;
    bool in_gap = false;
    size_t count = 0;
    std::istringstream stream(text);
    std::string line;
    while (std::getline(stream, line)) {
        if (line == "M625") {
            in_gap = true;
            count = 0;
        } else if (line.rfind("M624 ", 0) == 0) {
            if (in_gap)
                counts.push_back(count);
            in_gap = false;
        } else if (in_gap && line.rfind("G1 E", 0) == 0 && line.find(" X") == std::string::npos &&
                   line.find(" Y") == std::string::npos) {
            ++count;
        }
    }
    return counts;
}

// Remove the k-th labelled block alone (its M624 ... M625 lines included). A printer
// skips whole objects (see cancel_object), so this is only a finer-grained check.
std::string cancel_block(const std::string &text, size_t block)
{
    std::string kept;
    std::istringstream stream(text);
    std::string line;
    size_t current = 0;
    bool skipping = false;
    while (std::getline(stream, line)) {
        if (line.rfind("M624 ", 0) == 0) {
            skipping = current == block;
            ++current;
        }
        if (!skipping)
            kept += line + "\n";
        if (line == "M625")
            skipping = false;
    }
    return kept;
}

// Unique label id of each labelled block of a text, in order.
std::vector<int> block_object_ids(const std::string &text)
{
    std::vector<int> ids;
    const std::string marker = "; start printing object, unique label id: ";
    std::istringstream stream(text);
    std::string line;
    while (std::getline(stream, line))
        if (line.rfind(marker, 0) == 0)
            ids.push_back(std::stoi(line.substr(marker.size())));
    return ids;
}

// Remove every labelled block of the object `object_id` (all its layers, M624 ... M625
// included), as a printer does when the object is cancelled.
std::string cancel_object(const std::string &text, int object_id)
{
    const std::string marker = "; start printing object, unique label id: ";
    std::string kept;
    std::istringstream stream(text);
    std::string line;
    int pending_id = -1;
    bool skipping = false;
    while (std::getline(stream, line)) {
        if (line.rfind(marker, 0) == 0)
            pending_id = std::stoi(line.substr(marker.size()));
        if (line.rfind("M624 ", 0) == 0)
            skipping = pending_id == object_id;
        if (!skipping)
            kept += line + "\n";
        if (line == "M625")
            skipping = false;
    }
    return kept;
}

// Identity of the Bambu private Reader state, as the firmware provider wires it
// into the sessions it creates.
uint32_t bambu_reader_state_id()
{
    const orchestrator_runtime_id id = orchestrator_register_runtime_id(
        reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()),
        GCODE_READER_STATE_ID_TYPE,
        slic3r_api::GCodeGeneration::Reader::BAMBU_READER_STATE_NAME);
    if (id == ORCHESTRATOR_RUNTIME_ID_INVALID)
        throw std::runtime_error("The Bambu Reader state identity could not be registered for the test.");
    return id;
}

// A firmware session over a script processor and a Reader service, exposing the
// per-tool fans and the script fan hook. `Firmware` is the dialect under test.
template <class Firmware>
class FanScriptProbe final : public Firmware
{
public:
    FanScriptProbe() = default;
    explicit FanScriptProbe(slic3r_api::GCodeGeneration::GCodeScriptProcessorView scripts) :
        Firmware(scripts, nullptr)
    {}
    using Firmware::append_reader_private_state;
    using Firmware::process_script;
    using Firmware::script_fan_tool;
    slic3r_api::GCodeGeneration::DefaultExtruder &machine_extruder(size_t idx)
    {
        return this->extruders().at(idx);
    }
};

using BambuFanProbe = FanScriptProbe<slic3r_api::GCodeGeneration::Firmware::BambuGCodeFirmwareSession>;
using Marlin2FanProbe = FanScriptProbe<slic3r_api::GCodeGeneration::Firmware::Marlin2GCodeFirmwareSession>;

// Comma-separated list of `count` copies of `value`.
std::string repeated_value(const std::string &value, uint16_t count)
{
    std::string list = value;
    for (uint16_t idx = 1; idx < count; ++idx)
        list += "," + value;
    return list;
}

// Select `tool` on a session of `tools` tools, run `script` as a layer script and
// let `check` inspect the session. With `reader` false only the script
// interpreter runs (process_script()); with `reader` true the script is written as
// an event, so the session also reads it back with its own Reader, as in an export.
// A written event resets the requested fan of the selected tool to its own
// inherited state, so only the encoded fan is meaningful in that mode.
template <class Probe>
void run_fan_script(uint16_t tools, uint16_t tool, const std::string &script, bool reader,
                    const std::function<void(Probe &)> &check)
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_extruders(tools);
    config.set_deserialize_strict({
        {"nozzle_diameter", repeated_value("0.4", tools)},
        {"filament_diameter", repeated_value("1.75", tools)},
        {"extrusion_multiplier", repeated_value("1", tools)}});
    Model model;
    Print print;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
    configure_standard_firmware(print, tools);
    GCodeScriptProcessor scripts(print, Orchestrator::instance());
    Probe session(slic3r_api::GCodeGeneration::GCodeScriptProcessorView(scripts.c_processor()));
    if (session.reader_plugin_id() == "gcode.reader.bambu")
        session.set_reader_state_id(bambu_reader_state_id());
    session.set_gcode_reader_context(
        reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()),
        reinterpret_cast<const print_handle *>(&print));
    session.setup(slic3r_api::Print(reinterpret_cast<const print_handle *>(&print)).config());
    const ExtrusionNop select(ExtrusionPropertySpecialCommand(
        ExtrusionPropertySpecialCommand::Code::TOOLCHANGE, double(tool)));
    session.write_event(slic3r_api::ExtrusionEntity(
        reinterpret_cast<const extrusion_entity_handle *>(&select)));
    const ExtrusionNop event(ExtrusionPropertyCustomGcodeText(
        ExtrusionPropertyCustomGcodeText::Code::SCRIPT, script, GCODE_SCRIPT_TYPE_LAYER_GCODE));
    if (reader)
        session.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(&event)));
    else
        (void)session.process_script(GCODE_SCRIPT_TYPE_LAYER_GCODE, script);
    check(session);
}

// Where the custom text of run_bambu_platter_script() is written, and the setup.
struct PlatterScriptRun
{
    std::string layer_script;        // raw text after begin_layer() of the second layer
    std::string interpreter_script;  // script given to the script interpreter alone at the second layer
    std::string tree_script;         // raw text inside the tree of object `tree_object` of layer `tree_layer`
    uint32_t tree_layer = 0;
    uint32_t tree_object = 0;
    bool geometry_after_script = false;  // a second path follows the tree script in its tree
    uint32_t objects = 0;            // labelled objects per layer; 0: one unlabelled path per layer
    double retract_length = 0.;      // retraction depth of the labelled blocks
    int disabled_layers = 0;         // disable_fan_first_layers
    bool platter_fan = true;
    // When given, receives the private Reader state the session would export after the run.
    std::vector<raw_gcode_reader_event_item> *exported = nullptr;
};

// Drive a one-tool Bambu session with the platter fan at 70 % (S178) over three
// layers. The custom text goes through the session's Reader, which gives the
// firmware the fan speeds the text set. The session callbacks are real; no plugin
// state is changed.
std::string run_bambu_platter_script(const PlatterScriptRun &run)
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_extruders(1);
    config.set_deserialize_strict({{"platter_fan", run.platter_fan ? "1" : "0"}, {"platter_fan_speed", "70"},
        {"disable_fan_first_layers", std::to_string(run.disabled_layers)},
        {"gcode_label_objects", run.objects > 0 ? "firmware" : "disabled"}});
    Model model;
    Print print;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
    configure_standard_firmware(print, 1);
    PrintConfig &print_config = const_cast<PrintConfig &>(print.config());
    print_config.retract_length.set(std::vector<double>{run.retract_length});
    print_config.use_relative_e_distances.value = false;
    PrintingPlan plan = run.objects > 0 ?
        build_label_catalog(print, std::vector<bool>(run.objects, true)) : PrintingPlan();
    plan.groups.emplace_back();
    for (uint32_t layer_idx = 0; layer_idx < 3; ++layer_idx) {
        plan.groups.back().layers.emplace_back();
        PrintingLayerGroup &layer = plan.groups.back().layers.back();
        layer.print_z = scale_i(0.2 * (layer_idx + 1));
        if (layer_idx == 1 && !run.layer_script.empty()) {
            ExtrusionNop script;
            script.add_property(ExtrusionPropertyCustomGcodeText(run.layer_script));
            layer.events.append_before(script);
        }
        layer.tool_groups.emplace_back();
        PrintingToolGroup &tool = layer.tool_groups.back();
        tool.extruder_id = 0;
        for (uint32_t object = 0; object < std::max<uint32_t>(run.objects, 1); ++object) {
            const auto path = [&](double offset) {
                return make_firmware_path(ArcPolyline(Points{
                    Point::new_scale(double(object * 2) + offset, double(layer_idx)),
                    Point::new_scale(double(object * 2 + 1) + offset, double(layer_idx))}), 20.f, 500.f);
            };
            std::unique_ptr<ExtrusionEntity> root(new ExtrusionEntity(true));
            root->append_child(path(0.));
            if (object == run.tree_object && layer_idx == run.tree_layer && !run.tree_script.empty()) {
                root->append_child(std::unique_ptr<ExtrusionEntity>(new ExtrusionNop(
                    ExtrusionPropertyCustomGcodeText(run.tree_script))));
                if (run.geometry_after_script)
                    root->append_child(path(0.5));
            }
            append_path_extrusion(tool, std::move(root));
            if (run.objects > 0)
                tool.extrusions.back().printing_catalog_id = object;
        }
    }
    GCodeScriptProcessor scripts(print, Orchestrator::instance());
    BambuFanProbe session(slic3r_api::GCodeGeneration::GCodeScriptProcessorView(scripts.c_processor()));
    session.set_reader_state_id(bambu_reader_state_id());
    session.set_gcode_reader_context(
        reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()),
        reinterpret_cast<const print_handle *>(&print));
    const slic3r_api::ConstPrintingPlan view(reinterpret_cast<const printing_plan_handle *>(&plan));
    std::string output = session.begin_print(
        slic3r_api::Print(reinterpret_cast<const print_handle *>(&print)), view);
    const slic3r_api::ConstPrintingGroup group = view.group(0);
    output += session.begin_group(group);
    for (uint32_t layer_idx = 0; layer_idx < group.layer_group_count(); ++layer_idx) {
        const slic3r_api::ConstPrintingLayerGroup layer = group.layer_group(layer_idx);
        output += ";LAYER " + std::to_string(layer_idx) + "\n" + session.begin_layer(layer);
        if (layer_idx == 1 && !run.interpreter_script.empty())
            (void)session.process_script(GCODE_SCRIPT_TYPE_LAYER_GCODE, run.interpreter_script);
        if (layer.events().has_before())
            output += session.write_event(layer.events().before());
        const slic3r_api::ConstPrintingToolGroup tool = layer.tool_group(0);
        output += session.begin_tool_group(tool);
        for (uint32_t idx = 0; idx < tool.extrusion_count(); ++idx)
            output += session.write_extrusion(tool.extrusion(idx));
        output += session.end_tool_group();
        output += session.end_layer();
    }
    output += session.end_group();
    output += session.end_print();
    if (run.exported != nullptr)
        session.append_reader_private_state(*run.exported);
    return output;
}

// Platter speed (the last `M106 P2 S`) at each deposit of the block of the
// object whose label mask is `mask`, when the text is executed from the start.
std::vector<int> platter_speeds_in_block(const std::string &text, const std::string &mask)
{
    std::vector<int> speeds;
    std::istringstream stream(text);
    std::string line;
    int speed = -1;
    bool inside = false;
    while (std::getline(stream, line)) {
        if (line.rfind("M106 P2 S", 0) == 0)
            speed = std::stoi(line.substr(9));
        if (line.rfind("M624 ", 0) == 0)
            inside = line == "M624 " + mask;
        if (line == "M625")
            inside = false;
        if (inside && line.rfind("G1 ", 0) == 0 && line.find(" X") != std::string::npos &&
            line.find(" E") != std::string::npos)
            speeds.push_back(speed);
    }
    return speeds;
}

// Export cubes through the real pipeline with the production plugins and firmware
// labels. `tools` is the number of extruders (two cubes on tools 1 and 2 with a wipe
// tower when it is 2, three cubes otherwise, with `two_tool_lengths` as retract_length);
// `toolchange_script` replaces the semantic tool change by a custom script (the real
// custom path: begin_tool_group() selects the tool without writing T, the script writes it).
std::string export_labelled_cubes(bool relative_e, uint16_t tools = 1,
                                  const std::string &toolchange_script = std::string(),
                                  const std::string &two_tool_lengths = "2,3")
{
    const ScopedProductionPlugins production_plugins;
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_extruders(tools);
    config.set_deserialize_strict({
        {"gcode_flavor", "bambu"},
        {"gcode_exporter_plugin", "gcode.exporter.simple"},
        {"binary_gcode", "1"},
        {"gcode_label_objects", "firmware"},
        {"use_relative_e_distances", relative_e ? "1" : "0"},
        {"retract_before_travel", "1"},
        {"skirts", "0"},
        {"brim_width", "0"}});
    Model model;
    Print print;
    if (tools == 2) {
        config.set_deserialize_strict({
            {"retract_length", two_tool_lengths},
            {"filament_diameter", "1.75,1.75"},
            {"nozzle_diameter", "0.4,0.4"},
            {"extrusion_multiplier", "1,1"},
            {"retract_speed", "40,40"},
            {"deretract_speed", "0,0"},
            {"wipe_tower", "1"}});
        if (!toolchange_script.empty())
            config.set_deserialize_strict("toolchange_gcode", toolchange_script);
        initialize_mixed_height_cube_print(print, model, config, 1, 2);
    } else {
        config.set_deserialize_strict("retract_length", "2");
        Test::init_print({Test::TestMesh::cube_20x20x20, Test::TestMesh::cube_20x20x20,
                          Test::TestMesh::cube_20x20x20}, print, model, config);
    }
    print.process();
    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    Orchestrator::instance().export_gcode(print, output_path.string(), nullptr);
    const std::string output = read_text_file(output_path);
    remove_output_pair(output_path);
    return output;
}} // namespace

// Inspect all six chamber settings and the shared physical calibration alongside
// printer/plate/software identity. Catch reports any missing or changed metadata.
TEST_CASE("Bambu firmware owns and declares its six chamber fan settings",
          "[plugins][gcode][firmware][bambu][config]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Plugin *plugin = Orchestrator::instance().get_plugin("bambu");
    REQUIRE(plugin != nullptr);
    const char *const keys[] = {"platter_fan", "support_air_filtration", "platter_fan_speed",
        "activate_air_filtration", "during_print_exhaust_fan_speed", "complete_print_exhaust_fan_speed"};
    const ConfigOptionType types[] = {coBool, coBool, coInts, coBools, coInts, coInts};
    const char *const defaults[] = {"0", "0", "0", "0", "60", "80"};
    std::set<std::string> used;
    std::set<std::string> defined;
    for (const Plugin::UsedConfigKey &key : plugin->get_used_config_keys())
        used.insert(key.key);
    for (const Plugin::DefinedConfigKey &key : plugin->get_defined_config_keys())
        defined.insert(key.key);
    // Printer model id, nozzle type and plate type have translation tests; software identity has exporter tests.
    CHECK(plugin->get_defined_config_keys().size() == 11);
    CHECK(defined.count("fan_printer_min_speed") == 1);
    const ConfigOptionDef *calibration = PrintConfigDef::instance().get("fan_printer_min_speed");
    REQUIRE(calibration != nullptr);
    CHECK(calibration->type == coInt);
    REQUIRE(calibration->default_value.get() != nullptr);
    CHECK(calibration->default_value->serialize() == "0");
    CHECK(calibration->container_type == ConfigOptionContainerType::Plater);
    CHECK(calibration->option_preset_type == RAW_PRESET_TYPE_FFF_PRINTER);
    CHECK(calibration->invalidates_step == STEP_GCODE);
    CHECK(calibration->min == 0);
    CHECK(calibration->max == 100);
    CHECK(calibration->sidetext == "%");
    for (size_t idx = 0; idx < 6; ++idx) {
        INFO(keys[idx]);
        const ConfigOptionDef *def = PrintConfigDef::instance().get(keys[idx]);
        REQUIRE(def != nullptr);
        CHECK(def->type == types[idx]);
        REQUIRE(def->default_value.get() != nullptr);
        CHECK(def->default_value->serialize() == defaults[idx]);
        CHECK(def->container_type == (idx < 2 ? ConfigOptionContainerType::Project : ConfigOptionContainerType::Plater));
        CHECK(def->option_preset_type == (idx < 2 ? RAW_PRESET_TYPE_FFF_PRINTER : RAW_PRESET_TYPE_FFF_FILAMENT));
        CHECK(def->invalidates_step == STEP_GCODE);
        if (idx >= 2) {
            CHECK(def->is_vector_extruder);
            const ConfigOptionVectorBase *values = dynamic_cast<const ConfigOptionVectorBase *>(def->default_value.get());
            REQUIRE(values != nullptr);
            CHECK(values->size() == 1);
            CHECK(values->is_extruder_size());
        }
        // The host generates the exclusive-firmware enable rule from these read declarations.
        CHECK(used.count(keys[idx]) == 1);
        CHECK(defined.count(keys[idx]) == 1);
        if (types[idx] == coInts) {
            CHECK(def->min == 0);
            CHECK(def->max == 100);
            CHECK(def->sidetext == "%");
        }
    }
    CHECK(used.count("disable_fan_first_layers") == 0);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_extruders(3);
    Preset::normalize(config);
    for (size_t idx = 2; idx < 6; ++idx) {
        const ConfigOptionVectorBase *values = dynamic_cast<const ConfigOptionVectorBase *>(config.optptr(keys[idx]));
        REQUIRE(values != nullptr);
        // Preset normalization resizes filament-preset vectors to the nozzle
        // count, independently of their extruder-array flag.
        CHECK(values->is_extruder_size());
        CHECK(values->size() == 3);
    }
    for (const char *firmware : {"bambu", "marlin2"}) {
        config.set_deserialize_strict("gcode_flavor", firmware);
        for (size_t idx = 0; idx < 6; ++idx) {
            INFO(keys[idx]);
            INFO(firmware);
            bool found = false;
            const raw_option_preset_type scope = idx < 2 ? RAW_PRESET_TYPE_FFF_PRINTER : RAW_PRESET_TYPE_FFF_FILAMENT;
            for (const Orchestrator::GuiRuleApplication &rule : Orchestrator::instance().gui_rules().evaluate(config, scope, -1)) {
                if (rule.target_key == keys[idx]) {
                    found = true;
                    CHECK(rule.enabled == (std::string(firmware) == "bambu"));
                }
            }
            CHECK(found);
        }
    }
}

TEST_CASE("Bambu platter fan follows layers and actual filament before geometry",
          "[plugins][gcode][firmware][bambu][fans]")
{
    const std::string output = run_bambu_fans(true, false, false, 2);
    CHECK(count_occurrences(output, "M106 P2 S0\n") == 3);
    CHECK(count_occurrences(output, "M106 P2 S178\n") == 4);
    CHECK(count_occurrences(output, "M106 P2 S204\n") == 2);
    CHECK(output.find("M106 P3") == std::string::npos);
    const size_t first_zero = output.find("M106 P2 S0\n");
    const size_t layer_one_end = output.find(";AFTER_LAYER\n");
    CHECK(first_zero > layer_one_end);
    CHECK(first_zero < output.find("G1 ", layer_one_end));
    const size_t third_layer = output.find(";BEGIN_LAYER\n", output.find(";BEGIN_LAYER\n", output.find(";BEGIN_LAYER\n") + 1) + 1);
    CHECK(output.find("M106 P2 S178\n") > third_layer);
    const size_t ordered_change = output.find("T1\n");
    REQUIRE(ordered_change != std::string::npos);
    CHECK(output.find("M106 P2 S204\n") > ordered_change);
    REQUIRE(output.find("; AFTER_TOOLCHANGE\n") != std::string::npos);
    CHECK(output.find("M106 P2 S204\n") > output.find("; AFTER_TOOLCHANGE\n"));
    CHECK(output.find("M106 P2 S204\n") < output.find("G1 ", ordered_change));
    const size_t final_stop = output.rfind("M106 P2 S0\n");
    CHECK(final_stop < output.find("M981 S0"));
    CHECK(final_stop > output.find(";GROUP 1\n"));
    CHECK(run_bambu_fans(false, false, false, 2).find("M106 P2") == std::string::npos);
    const std::string immediate = run_bambu_fans(true, false, false, 0);
    CHECK(immediate.find("M106 P2 S178\n") < immediate.find(";BEGIN_LAYER\n", immediate.find(";BEGIN_LAYER\n") + 1));
    CHECK(immediate.find("M106 P2 S178\n") < immediate.find("G1 ", immediate.find(";AFTER_LAYER\n")));
    const std::string ending_zero = run_bambu_fans(true, false, false, 4);
    CHECK(count_occurrences(ending_zero, "M106 P2 S0\n") == 3);
    CHECK(count_occurrences(ending_zero, "M106 P2 S204\n") == 2);
}

TEST_CASE("Bambu platter fan changes cannot be skipped with a cancelled object",
          "[plugins][gcode][firmware][bambu][fans]")
{
    const double depth = GENERATE(0., 2.);
    for (const bool ordered : {false, true}) {
        INFO("retraction depth " << depth << ", ordered change " << ordered);
        const std::string output = run_bambu_cancel_fan(ordered, depth);
        bool inside = false;
        std::istringstream source(output);
        std::string line;
        std::string cancelled;
        bool skip = false;
        size_t all_deposits = 0;
        while (std::getline(source, line)) {
            if (line.rfind("M624 ", 0) == 0) {
                CHECK_FALSE(inside);
                inside = true;
                skip = line == "M624 AQAAAAAAAAA=";
            }
            if (line.rfind("M106 P2 ", 0) == 0)
                CHECK_FALSE(inside);
            if (line.rfind("G1 ", 0) == 0 && line.find(" X") != std::string::npos &&
                line.find(" E") != std::string::npos) {
                CHECK(inside);
                ++all_deposits;
            }
            if (!skip)
                cancelled += line + "\n";
            if (line == "M625") {
                CHECK(inside);
                inside = false;
                skip = false;
            }
        }
        CHECK_FALSE(inside);
        CHECK(all_deposits == 4);
        CHECK(count_occurrences(output, "M624 ") == 4);
        CHECK(count_occurrences(output, "M625\nG92 E0\n") == 4);
        CHECK(cancelled.find("M624 AQAAAAAAAAA=") == std::string::npos);
        // Execute only the retained commands: fan speed before object one's
        // deposited geometry must be identical, even without object zero's blocks.
        const std::string *const executions[] = {&output, &cancelled};
        for (const std::string *executed : executions) {
            std::istringstream stream(*executed);
            int speed = -1;
            int layer = -1;
            bool second = false;
            int deposits = 0;
            while (std::getline(stream, line)) {
                if (line == ";LAYER_CHANGE")
                    ++layer;
                if (line.rfind("M106 P2 S", 0) == 0)
                    speed = std::stoi(line.substr(9));
                if (line.rfind("M624 ", 0) == 0)
                    second = line == "M624 AgAAAAAAAAA=";
                if (line == "M625")
                    second = false;
                if (second && line.rfind("G1 ", 0) == 0 && line.find(" X") != std::string::npos &&
                    line.find(" E") != std::string::npos) {
                    CHECK(speed == (layer == 0 ? 0 : ordered ? 204 : 178));
                    ++deposits;
                }
            }
            CHECK(deposits == 2);
            const DebtTrace trace = trace_debt(*executed, depth);
            const size_t blocks = executed == &output ? 4 : 2;
            REQUIRE(trace.at_block_start.size() == blocks);
            REQUIRE(trace.at_block_end.size() == blocks);
            REQUIRE(trace.before_first_deposit.size() == blocks);
            for (size_t idx = 0; idx < blocks; ++idx) {
                CHECK(trace.at_block_start[idx] == Approx(depth).margin(1e-4));
                CHECK(trace.at_block_end[idx] == Approx(depth).margin(1e-4));
                CHECK(trace.before_first_deposit[idx] == Approx(0.).margin(1e-4));
            }
            CHECK(trace.tool_selections_in_blocks == 0);
            CHECK(trace.final_debt == Approx(depth).margin(1e-4));
        }
    }
}

TEST_CASE("Bambu reopens an object's block after a tool change inside its tree",
          "[plugins][gcode][firmware][bambu][fans][retraction]")
{
    const std::string output = run_bambu_cancel_fan(true, 2., true);
    CHECK((block_object_ids(output) == std::vector<int>{0, 1, 0, 0, 1}));
    CHECK(count_occurrences(output, "M624 AQAAAAAAAAA=\n") == 3);
    const std::string cancelled = cancel_object(output, 0);
    // Start comments are outside M624/M625 and remain when the executable block is skipped.
    CHECK(cancelled.find("M624 AQAAAAAAAAA=\n") == std::string::npos);
    CHECK(count_occurrences(cancelled, "M624 AgAAAAAAAAA=\n") == 2);
    const std::string *const executions[] = {&output, &cancelled};
    for (const std::string *executed : executions) {
        const size_t blocks = executed == &output ? 5 : 2;
        std::istringstream stream(*executed);
        std::string line;
        bool inside = false;
        size_t deposits = 0;
        while (std::getline(stream, line)) {
            if (line.rfind("M624 ", 0) == 0) {
                CHECK_FALSE(inside);
                inside = true;
            }
            if (line == "M625") {
                CHECK(inside);
                inside = false;
            }
            if (line.rfind("M106 P2 ", 0) == 0)
                CHECK_FALSE(inside);
            if (line.rfind("G1 ", 0) == 0 && line.find(" X") != std::string::npos &&
                line.find(" E") != std::string::npos) {
                CHECK(inside);
                ++deposits;
            }
        }
        CHECK_FALSE(inside);
        CHECK(deposits == blocks);
        CHECK(count_occurrences(*executed, "M624 ") == blocks);
        CHECK(count_occurrences(*executed, "M625\n") == blocks);
        CHECK(count_occurrences(*executed, "M625\nG92 E0\n") == blocks);
        const DebtTrace trace = trace_debt(*executed, 2.);
        REQUIRE(trace.at_block_start.size() == blocks);
        REQUIRE(trace.at_block_end.size() == blocks);
        REQUIRE(trace.before_first_deposit.size() == blocks);
        for (size_t idx = 0; idx < blocks; ++idx) {
            CHECK(trace.at_block_start[idx] == Approx(2.).margin(1e-4));
            CHECK(trace.at_block_end[idx] == Approx(2.).margin(1e-4));
            CHECK(trace.before_first_deposit[idx] == Approx(0.).margin(1e-4));
            CHECK(trace.tool_at_block_start[idx] == trace.tool_at_block_end[idx]);
        }
        CHECK(trace.tool_selections_in_blocks == 0);
        CHECK(trace.final_debt == Approx(2.).margin(1e-4));
    }
}

// A stationary deposit that follows an in-tree tool change belongs to its object's block like geometry does: the
// closed block is reopened for it, and it disappears with the object when the object is cancelled.
TEST_CASE("Bambu reopens an object's block for a deposit after a tool change inside its tree",
          "[plugins][gcode][firmware][bambu][deposit][retraction]")
{
    const std::string output = run_bambu_cancel_fan(true, 2., false, true);
    INFO(output);
    CHECK((block_object_ids(output) == std::vector<int>{0, 1, 0, 0, 1}));
    CHECK(count_occurrences(output, " Y2.5") >= 1);
    const std::string cancelled = cancel_object(output, 0);
    CHECK(cancelled.find(" Y2.5") == std::string::npos);
    const std::string *const executions[] = {&output, &cancelled};
    for (const std::string *executed : executions) {
        std::istringstream stream(*executed);
        std::string line;
        bool inside = false;
        while (std::getline(stream, line)) {
            if (line.rfind("M624 ", 0) == 0)
                inside = true;
            if (line == "M625")
                inside = false;
            // The approach travel to the deposit point is the only line at Y2.5.
            if (line.find(" Y2.5") != std::string::npos)
                CHECK(inside);
        }
        const DebtTrace trace = trace_debt(*executed, 2.);
        REQUIRE(trace.at_block_start.size() == trace.at_block_end.size());
        for (size_t idx = 0; idx < trace.at_block_start.size(); ++idx) {
            CHECK(trace.at_block_start[idx] == Approx(2.).margin(1e-4));
            CHECK(trace.at_block_end[idx] == Approx(2.).margin(1e-4));
        }
        CHECK(trace.tool_selections_in_blocks == 0);
    }
}

// The first material of a block opened with a retraction depth is preceded by the explicit unretraction, a stationary
// deposit included; a block without depth writes none.
TEST_CASE("Bambu unretracts before the first deposit of a block",
          "[plugins][gcode][firmware][bambu][deposit][retraction]")
{
    SECTION("depth 2") {
        const std::string output = run_bambu_deposit_block(2.);
        INFO(output);
        REQUIRE((block_object_ids(output) == std::vector<int>{0}));
        const DebtTrace trace = trace_debt(output, 2.);
        // Entry retraction, explicit unretraction, the deposit, exit retraction.
        REQUIRE(trace.events.size() == 4);
        CHECK(trace.events[0].value == Approx(-2.).margin(1e-4));
        CHECK(trace.events[1].value == Approx(2.).margin(1e-4));
        CHECK(trace.events[2].value > 0.);
        CHECK(trace.events[3].value == Approx(-2.).margin(1e-4));
        REQUIRE(trace.at_block_start.size() == 1);
        CHECK(trace.at_block_start[0] == Approx(2.).margin(1e-4));
        CHECK(trace.at_block_end[0] == Approx(2.).margin(1e-4));
    }
    SECTION("depth 0") {
        const std::string output = run_bambu_deposit_block(0.);
        INFO(output);
        REQUIRE((block_object_ids(output) == std::vector<int>{0}));
        const DebtTrace trace = trace_debt(output, 0.);
        // Only the deposit itself moves the filament.
        REQUIRE(trace.events.size() == 1);
        CHECK(trace.events[0].value > 0.);
        CHECK(trace.final_debt == Approx(0.).margin(1e-4));
    }
}

TEST_CASE("Bambu exhaust fan uses activated used filaments at print boundaries",
          "[plugins][gcode][firmware][bambu][fans]")
{
    const std::string output = run_bambu_fans(false, true, true, 0);
    CHECK(count_occurrences(output, "M106 P3 S178\n") == 1);
    CHECK(count_occurrences(output, "M106 P3 S204\n") == 1);
    CHECK(output.find("M106 P3 S255") == std::string::npos);
    CHECK(output.find("M106 P3 S178\n") < output.find("M981 S1"));
    REQUIRE(output.find(";FAN_START") != std::string::npos);
    CHECK(output.find("M106 P3 S178\n") > output.find(";FAN_START"));
    CHECK(output.find("M106 P3 S204\n") > output.find(";END_GCODE\n"));
    CHECK(run_bambu_fans(false, false, true, 0).find("M106 P3") == std::string::npos);
    CHECK(run_bambu_fans(false, true, false, 0).find("M106 P3") == std::string::npos);
    const std::string first_only = run_bambu_fans(false, true, true, 0, false);
    CHECK(count_occurrences(first_only, "M106 P3 S153\n") == 1);
    CHECK(count_occurrences(first_only, "M106 P3 S178\n") == 1);
    CHECK(first_only.find("M106 P3 S153\n") < first_only.find("M981 S1"));
    CHECK(first_only.find("M106 P3 S178\n") > first_only.find(";END_GCODE\n"));
    const std::string reversed = run_bambu_fans(false, true, true, 0, false, true);
    CHECK(count_occurrences(reversed, "M106 P3 S153\n") == 1);
    CHECK(count_occurrences(reversed, "M106 P3 S178\n") == 1);
    CHECK(reversed.find("M106 P3 S178\n") < reversed.find("M981 S1"));
    CHECK(reversed.find("M106 P3 S153\n") > reversed.find(";END_GCODE\n"));
    const std::string reversed_max = run_bambu_fans(false, true, true, 0, true, true);
    CHECK(count_occurrences(reversed_max, "M106 P3 S178\n") == 1);
    CHECK(count_occurrences(reversed_max, "M106 P3 S204\n") == 1);
    CHECK(reversed_max.find("M106 P3 S204\n") < reversed_max.find("M981 S1"));
    CHECK(reversed_max.find("M106 P3 S178\n") > reversed_max.find(";END_GCODE\n"));
}

// The script interpreter asks the session which tool a `M106` / `M107` line
// addresses: Bambu reads P as a fan number, the other dialects keep P as a tool.
TEST_CASE("Bambu reads the P of a script M106 as a fan number",
          "[plugins][gcode][firmware][bambu][fans][script]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const std::optional<int32_t> no_fan;
    const std::optional<extruder_index_t> no_tool;
    const std::optional<extruder_index_t> tool_two = extruder_index_t(2);
    BambuFanProbe bambu;
    Marlin2FanProbe marlin;

    // No P and P1 are the part fan of the selected tool; P2 and P3 are not tool fans.
    CHECK(bambu.script_fan_tool(no_fan, tool_two) == 2);
    CHECK(bambu.script_fan_tool(1, tool_two) == 2);
    CHECK_FALSE(bambu.script_fan_tool(2, tool_two));
    CHECK_FALSE(bambu.script_fan_tool(3, tool_two));
    CHECK_FALSE(bambu.script_fan_tool(3, no_tool));
    // Any other P is still the tool P, and without a selected tool P1 is the tool 1.
    CHECK(bambu.script_fan_tool(0, tool_two) == 0);
    CHECK(bambu.script_fan_tool(4, tool_two) == 4);
    CHECK(bambu.script_fan_tool(1, no_tool) == 1);
    CHECK_FALSE(bambu.script_fan_tool(no_fan, no_tool));

    // The other dialects read P as a tool, so P2 and P3 stay the tools 2 and 3.
    CHECK(marlin.script_fan_tool(1, tool_two) == 1);
    CHECK(marlin.script_fan_tool(2, tool_two) == 2);
    CHECK(marlin.script_fan_tool(3, tool_two) == 3);
    CHECK(marlin.script_fan_tool(no_fan, tool_two) == 2);
    CHECK_FALSE(marlin.script_fan_tool(no_fan, no_tool));
}

// Requested and encoded part-fan percent of one tool.
struct ToolFan
{
    std::optional<double> requested;
    std::optional<double> encoded;
};

template <class Probe>
ToolFan tool_fan(Probe &session, size_t tool)
{
    const slic3r_api::GCodeGeneration::FanState &fan = session.machine_extruder(tool).fan();
    return {fan.requested_speed_percent(), fan.encoded_speed_percent()};
}

// Check the part fan of every tool after a script. ``expected`` holds, per tool,
// the fan percent the script must leave, or nothing for a tool it must not touch.
// The script runs through the interpreter alone, then through Reader
// reconciliation. Both observe physical output without inventing a logical
// request, since minimum/offset calibration cannot be inverted.
template <class Probe>
void check_script_fans(uint16_t tool, const std::string &script,
                       const std::vector<std::optional<double>> &expected)
{
    for (const bool reader : {false, true}) {
        INFO((reader ? "Reader reconciliation" : "script interpreter"));
        run_fan_script<Probe>(uint16_t(expected.size()), tool, script, reader,
            std::function<void(Probe &)>([&](Probe &session) {
                for (size_t idx = 0; idx < expected.size(); ++idx) {
                    INFO("tool " << idx);
                    const ToolFan fan = tool_fan(session, idx);
                    CHECK_FALSE(fan.requested);
                    if (!expected[idx]) {
                        CHECK_FALSE(fan.encoded);
                        continue;
                    }
                    REQUIRE(fan.encoded);
                    CHECK(*fan.encoded == Approx(*expected[idx]));
                }
            }));
    }
}

// A script M106 / M107 updates the addressed fan's physical observation, not a
// logical request. Check both interpreter and Reader paths with four tools:
// P2 and P3 are real tools for Marlin 2 and must not be for Bambu.
TEST_CASE("A Bambu script M106 changes only the part fan of the selected tool",
          "[plugins][gcode][firmware][bambu][fans][script]")
{
    using Fans = std::vector<std::optional<double>>;
    const std::optional<double> none;
    const double sixty = 153.0 * 100.0 / 255.0;
    const double half = 128.0 * 100.0 / 255.0;

    SECTION("P2 and P3 are not the fans of the tools 2 and 3") {
        const std::string script = "M106 P3 S153\nM106 P2 S0\n";
        check_script_fans<BambuFanProbe>(1, script, Fans{none, none, none, none});
        // Marlin 2 reads them as the fans of the tools 3 and 2.
        check_script_fans<Marlin2FanProbe>(1, script, Fans{none, none, 0.0, sixty});
    }

    SECTION("P1 is the part fan of the selected tool") {
        check_script_fans<BambuFanProbe>(2, "M106 P1 S255\n", Fans{none, none, 100.0, none});
        // Marlin 2 reads P1 as the tool 1.
        check_script_fans<Marlin2FanProbe>(2, "M106 P1 S255\n", Fans{none, 100.0, none, none});
    }

    SECTION("M106 and M107 without P follow the selected tool") {
        check_script_fans<BambuFanProbe>(2, "M106 S128\nM107 P3\n", Fans{none, none, half, none});
        check_script_fans<BambuFanProbe>(2, "M106 S255\nM107\n", Fans{none, none, 0.0, none});
    }

    SECTION("P0 stays the tool 0 and P4 addresses no tool") {
        check_script_fans<BambuFanProbe>(2, "M106 P0 S255\nM106 P4 S255\n", Fans{100.0, none, none, none});
    }
}
// The platter speed a script set is the speed of the fan: the firmware compares
// the speed it wants with it, so it writes a changed speed again and does not
// repeat an unchanged one. The exhaust fan is only published, never followed. The
// wanted speed is S178; scripts that leave exactly that speed write "S178.0" so
// that the firmware's own writes stay countable.
TEST_CASE("Bambu follows the platter fan speed set by a custom script",
          "[plugins][gcode][firmware][bambu][fans][script]")
{
    PlatterScriptRun run;
    SECTION("a script that stops the platter makes the next layer write it again") {
        for (const std::string &script : {std::string("M106 P2 S0"), std::string("M107 P2")}) {
            INFO(script);
            run.layer_script = script;
            const std::string output = run_bambu_platter_script(run);
            CHECK(count_occurrences(output, "M106 P2 S178\n") == 2);
            const size_t second_layer = output.find(";LAYER 1\n");
            const size_t script_pos = output.find(script + "\n", second_layer);
            REQUIRE(script_pos != std::string::npos);
            CHECK(output.find("M106 P2 S178\n", second_layer) > script_pos);
            CHECK(output.find("M106 P2 S178\n", second_layer) < output.find(";LAYER 2\n"));
            CHECK(output.find("M106 P2 S178\n", output.find(";LAYER 2\n")) == std::string::npos);
        }
    }
    SECTION("a script that already set the wanted speed makes the layer write nothing") {
        // The first layer is stopped (S0), the second wants S178: the script sets exactly
        // that speed, so only the adopted speed spares the layer's own write.
        run.disabled_layers = 1;
        run.layer_script = "M106 P2 S178.0";
        CHECK(count_occurrences(run_bambu_platter_script(run), "M106 P2 S178\n") == 0);
        run.layer_script.clear();
        CHECK(count_occurrences(run_bambu_platter_script(run), "M106 P2 S178\n") == 1);
    }
    SECTION("an exhaust fan script does not touch the platter cache") {
        run.layer_script = "M106 P3 S153";
        CHECK(count_occurrences(run_bambu_platter_script(run), "M106 P2 S178\n") == 1);
    }
    SECTION("the script interpreter alone does not touch the platter cache") {
        run.interpreter_script = "M106 P2 S0";
        CHECK(count_occurrences(run_bambu_platter_script(run), "M106 P2 S178\n") == 1);
    }
    SECTION("an unknown command makes the Reader lose the speed, which is written again") {
        run.layer_script = "M106 P2 S178.0\nM9999";
        CHECK(count_occurrences(run_bambu_platter_script(run), "M106 P2 S178\n") == 2);
    }
    SECTION("nothing is written for a disabled platter fan, not even a final stop") {
        run.platter_fan = false;
        run.layer_script = "M106 P2 S178.0";
        std::vector<raw_gcode_reader_event_item> exported;
        run.exported = &exported;
        const std::string output = run_bambu_platter_script(run);
        // Only the script's own line, and no state followed nor handed to the Reader.
        CHECK(count_occurrences(output, "M106 P2") == 1);
        CHECK(count_occurrences(output, "M981 S0") == 1);
        CHECK(exported.empty());
        // Enabled, the same run hands the speed to the Reader.
        run.platter_fan = true;
        exported.clear();
        (void)run_bambu_platter_script(run);
        CHECK(exported.size() == 1);
    }
    SECTION("an unknown speed at the end of the last group is stopped before the detector closes") {
        // The script sits in the last object's block of the last layer: its effect is unknown.
        run.objects = 2;
        run.tree_layer = 2;
        run.tree_object = 1;
        run.tree_script = "M106 P2 S0";
        const std::string output = run_bambu_platter_script(run);
        const size_t script_pos = output.find("M106 P2 S0\n", output.find(";LAYER 2\n"));
        REQUIRE(script_pos != std::string::npos);
        const size_t stop_pos = output.find("M106 P2 S0\n", script_pos + 1);
        REQUIRE(stop_pos != std::string::npos);
        CHECK(stop_pos > output.rfind("M625\n"));
        CHECK(stop_pos < output.find("M981 S0"));
    }
}

// A script inside an object block may be skipped with a cancelled object, so the
// speed it sets is unknown: the wanted speed is written again outside any block.
TEST_CASE("Bambu writes the platter speed again when a script inside an object block set it",
          "[plugins][gcode][firmware][bambu][fans][script]")
{
    // The first layer is stopped (the wanted speed is 0), then S178. The script sits in
    // the last object of the first layer and sets exactly the next layer's speed: without
    // the unknown state the firmware would skip the layer's write, although the script
    // is skipped when that object is cancelled.
    PlatterScriptRun run;
    run.objects = 2;
    run.disabled_layers = 1;
    run.tree_object = 1;
    run.tree_script = "M106 P2 S178.0";
    const std::string output = run_bambu_platter_script(run);
    CHECK(count_occurrences(output, "M106 P2 S178\n") == 1);
    CHECK(output.find("M106 P2 S178\n") > output.find(";LAYER 1\n"));
    for (const int cancelled : {-1, 0, 1}) {
        INFO("cancelled object " << cancelled);
        const std::string executed = cancelled < 0 ? output : cancel_object(output, cancelled);
        // Object 0 of the layers 0, 1 and 2, unless it is the cancelled one.
        if (cancelled != 0)
            CHECK(platter_speeds_in_block(executed, "AQAAAAAAAAA=") == std::vector<int>({0, 178, 178}));
        if (cancelled != 1)
            CHECK(platter_speeds_in_block(executed, "AgAAAAAAAAA=") == std::vector<int>({0, 178, 178}));
    }
}

// The platter speed is never written from inside an object block: that would close
// the block and leave the rest of its object outside it, where cancelling does not
// skip it. A script in the middle of an object's tree followed by more geometry
// keeps the object whole; the speed is written before the next object.
TEST_CASE("Bambu keeps an object whole when a script in its tree changes the platter speed",
          "[plugins][gcode][firmware][bambu][fans][script][retraction]")
{
    PlatterScriptRun run;
    run.objects = 2;
    run.retract_length = 2.;
    run.tree_script = "M106 P2 S0";
    run.geometry_after_script = true;
    const std::string output = run_bambu_platter_script(run);
    CHECK(count_occurrences(output, "M624 ") == 6);
    CHECK(count_occurrences(output, "M625\nG92 E0\n") == 6);
    // Every deposit is inside a block, and the fan is never written inside one.
    bool inside = false;
    size_t deposits = 0;
    std::istringstream stream(output);
    std::string line;
    while (std::getline(stream, line)) {
        if (line.rfind("M624 ", 0) == 0)
            inside = true;
        if (line == "M625")
            inside = false;
        if (line.rfind("M106 P2 ", 0) == 0 && line != "M106 P2 S0")
            CHECK_FALSE(inside);
        if (line.rfind("G1 ", 0) == 0 && line.find(" X") != std::string::npos &&
            line.find(" E") != std::string::npos) {
            CHECK(inside);
            ++deposits;
        }
    }
    CHECK(deposits == 7);
    // The script's S0 is inside object 0's block; the wanted speed is written again
    // before the second object, outside every block.
    const size_t first_block_end = output.find("M625\n");
    const size_t second_block = output.find("M624 AgAAAAAAAAA=");
    REQUIRE(first_block_end != std::string::npos);
    REQUIRE(second_block != std::string::npos);
    const size_t rewrite = output.find("M106 P2 S178\n", first_block_end);
    CHECK(rewrite != std::string::npos);
    CHECK(rewrite < second_block);
    // Cancelling the first object or not: balanced blocks, nothing owed before a deposit.
    const std::string cancelled = cancel_object(output, 0);
    for (const std::string *executed : {&output, &cancelled}) {
        const size_t blocks = executed == &output ? 6 : 3;
        const DebtTrace trace = trace_debt(*executed, 2.);
        REQUIRE(trace.at_block_start.size() == blocks);
        REQUIRE(trace.at_block_end.size() == blocks);
        REQUIRE(trace.before_first_deposit.size() == blocks);
        for (size_t idx = 0; idx < blocks; ++idx) {
            CHECK(trace.at_block_start[idx] == Approx(2.).margin(1e-4));
            CHECK(trace.at_block_end[idx] == Approx(2.).margin(1e-4));
            CHECK(trace.before_first_deposit[idx] == Approx(0.).margin(1e-4));
        }
        CHECK(trace.final_debt == Approx(2.).margin(1e-4));
        for (const int speed : platter_speeds_in_block(*executed, "AgAAAAAAAAA="))
            CHECK(speed == 178);
    }
}

// An ordered tool change inside the tree of a labelled object closes its block; the
// new filament's platter speed is then written outside the reopened block, before the
// geometry that follows the change, never inside the block.
TEST_CASE("Bambu writes the new filament's platter speed outside the reopened block",
          "[plugins][gcode][firmware][bambu][fans][retraction]")
{
    const std::string output = run_bambu_cancel_fan(true, 2., true);
    const std::vector<std::string> lines = split_lines(output);
    const size_t change = find_line(lines, "T1");
    REQUIRE(change < lines.size());
    const size_t speed = find_line(lines, "M106 P2 S204", change);
    REQUIRE(speed < lines.size());
    size_t deposit = change;
    for (; deposit < lines.size(); ++deposit)
        if (lines[deposit].rfind("G1 ", 0) == 0 && lines[deposit].find(" X") != std::string::npos &&
            lines[deposit].find(" E") != std::string::npos)
            break;
    REQUIRE(deposit < lines.size());
    CHECK(speed < deposit);
    // Every platter write is outside every block.
    int open_blocks = 0;
    for (const std::string &line : lines) {
        if (line.rfind("M624 ", 0) == 0)
            ++open_blocks;
        if (line == "M625")
            --open_blocks;
        if (line.rfind("M106 P2 ", 0) == 0)
            CHECK(open_blocks == 0);
    }
}
// The firmware provider gives the sessions it creates the identity of the Bambu
// Reader state: a script goes through the real Reader without an identity error.
TEST_CASE("The Bambu firmware provider wires the Reader state identity",
          "[plugins][gcode][firmware][bambu][fans][pipeline]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const std::string output = export_labelled_cubes(false, 2,
        "M106 P2 S0\\nM106 P1 S255\\nT[next_extruder]", "2,3");
    CHECK(output.find("M106 P2 S0\n") != std::string::npos);
    CHECK(output.find("M106 P1 S255\n") != std::string::npos);
}
TEST_CASE("Bambu firmware labels objects with a rank bitmask",
          "[plugins][gcode][firmware][bambu][labels]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    SECTION("two objects, absolute E, are labelled in order and E restarts after each") {
        const std::string output = run_bambu_labels({true, true}, LabelObjectsStyle::Firmware, false);
        INFO(output);
        const std::string start_first = "; start printing object, unique label id: 0\nM624 AQAAAAAAAAA=\n";
        const std::string stop_first = "; stop printing object, unique label id: 0\nM625\nG92 E0\n";
        const std::string start_second = "; start printing object, unique label id: 1\nM624 AgAAAAAAAAA=\n";
        const std::string stop_second = "; stop printing object, unique label id: 1\nM625\nG92 E0\n";
        const size_t start_first_pos = output.find(start_first);
        const size_t stop_first_pos = output.find(stop_first);
        const size_t start_second_pos = output.find(start_second);
        const size_t stop_second_pos = output.find(stop_second);
        REQUIRE(start_first_pos != std::string::npos);
        REQUIRE(stop_first_pos != std::string::npos);
        REQUIRE(start_second_pos != std::string::npos);
        REQUIRE(stop_second_pos != std::string::npos);
        CHECK(start_first_pos < stop_first_pos);
        CHECK(stop_first_pos < start_second_pos);
        CHECK(start_second_pos < stop_second_pos);
        // Layer printing: the travel to the object comes after its label, as in Orca.
        // (In sequential printing begin_group() writes the approach before the
        // label; that difference is a known to-do, not tested here.)
        REQUIRE(output.find("\nG1 ") != std::string::npos);
        CHECK(output.find("\nG1 ") > start_first_pos);
        CHECK(count_occurrences(output, "M624 ") == 2);
        CHECK(count_occurrences(output, "M625\nG92 E0\n") == 2);
        // The second object's first E is relative to the zero written after M625.
        const double first_e = first_extrusion_e_after(output, start_first_pos);
        const double second_e = first_extrusion_e_after(output, start_second_pos);
        CHECK(first_e > 0.0);
        CHECK(second_e == Approx(first_e));
    }

    SECTION("the bit follows the id, not the printing order") {
        const std::string output = run_bambu_labels({true, true}, LabelObjectsStyle::Firmware, false, {1, 0});
        INFO(output);
        const size_t second_pos = output.find("unique label id: 1\nM624 AgAAAAAAAAA=\n");
        const size_t first_pos = output.find("unique label id: 0\nM624 AQAAAAAAAAA=\n");
        REQUIRE(second_pos != std::string::npos);
        REQUIRE(first_pos != std::string::npos);
        CHECK(second_pos < first_pos);
    }

    SECTION("relative E writes no G92 E0 after M625") {
        const std::string output = run_bambu_labels({true, true}, LabelObjectsStyle::Firmware, true);
        INFO(output);
        CHECK(count_occurrences(output, "M625\n") == 2);
        CHECK(output.find("M625\nG92 E0") == std::string::npos);
        CHECK(output.find("G92 E0") == std::string::npos);
    }

    SECTION("64 objects use every bit of the mask, the last one being bit 63") {
        const std::string output = run_bambu_labels(std::vector<bool>(64, true), LabelObjectsStyle::Firmware, false);
        CHECK(count_occurrences(output, "M624 ") == 64);
        CHECK(count_occurrences(output, "M625\n") == 64);
        CHECK(output.find("unique label id: 63\nM624 AAAAAAAAAIA=\n") != std::string::npos);
    }

    SECTION("65 objects get no label and no E reset") {
        const std::string output = run_bambu_labels(std::vector<bool>(65, true), LabelObjectsStyle::Firmware, false);
        const std::string baseline = run_bambu_labels(std::vector<bool>(65, true), LabelObjectsStyle::Disabled, false);
        CHECK(count_occurrences(output, "M624") == 0);
        CHECK(count_occurrences(output, "M625") == 0);
        CHECK(count_occurrences(output, "unique label id") == 0);
        CHECK(count_occurrences(output, "G92 E0\n") == count_occurrences(baseline, "G92 E0\n"));
    }

    SECTION("the bit is the rank among real objects, not the catalog id") {
        const std::string output = run_bambu_labels({false, true, true}, LabelObjectsStyle::Firmware, false);
        INFO(output);
        CHECK(output.find("unique label id: 1\nM624 AQAAAAAAAAA=\n") != std::string::npos);
        CHECK(output.find("unique label id: 2\nM624 AgAAAAAAAAA=\n") != std::string::npos);
        CHECK(output.find("unique label id: 0") == std::string::npos);
        CHECK(count_occurrences(output, "M624 ") == 2);
    }

    SECTION("an Octoprint-only style writes no firmware label") {
        const std::string output = run_bambu_labels({true, true}, LabelObjectsStyle::Octoprint, false);
        const std::string baseline = run_bambu_labels({true, true}, LabelObjectsStyle::Disabled, false);
        CHECK(count_occurrences(output, "M624") == 0);
        CHECK(count_occurrences(output, "M625") == 0);
        CHECK(count_occurrences(output, "G92 E0\n") == count_occurrences(baseline, "G92 E0\n"));
    }
}

TEST_CASE("Bambu firmware sequences the E zero before work written after an object",
          "[plugins][gcode][firmware][bambu][labels][retraction]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    // E value of every E-bearing move line after `from`.
    const auto e_values_after = [](const std::string &text, size_t from) {
        std::vector<double> values;
        std::istringstream stream(text.substr(from));
        std::string line;
        while (std::getline(stream, line)) {
            const size_t e_pos = line.find(" E");
            if (line.rfind("G1 ", 0) == 0 && e_pos != std::string::npos)
                values.push_back(std::stod(line.substr(e_pos + 2)));
        }
        return values;
    };

    SECTION("an E-only retraction written after the object is relative to the new zero") {
        const std::string output = run_bambu_retraction_between_objects(false);
        INFO(output);
        const std::string reset = "M625\nG92 E0\n";
        const size_t reset_pos = output.find(reset);
        REQUIRE(reset_pos != std::string::npos);
        const size_t second_start = output.find("unique label id: 1\nM624 ");
        REQUIRE(second_start != std::string::npos);
        // The retraction follows the zero and sits outside both labelled blocks.
        const std::vector<double> after_reset = e_values_after(output, reset_pos + reset.size());
        REQUIRE(after_reset.size() == 3);
        CHECK(after_reset[0] == Approx(-0.8));
        CHECK(output.find(" E-.8") > reset_pos);
        CHECK(output.find(" E-.8") < second_start);
        // The unretract returns to the zero, then the object extrudes from there.
        CHECK(after_reset[1] == Approx(0.0));
        const double first_e = first_extrusion_e_after(output, output.find("M624 "));
        CHECK(first_e > 0.0);
        CHECK(after_reset[2] == Approx(first_e));
        CHECK(count_occurrences(output, "G92 E0\n") == 3);
    }

    SECTION("a native retraction needs no E and its unretract is not rebased") {
        const std::string output = run_bambu_retraction_between_objects(true);
        INFO(output);
        const size_t reset_pos = output.find("M625\nG92 E0\n; SLIC3R_OBJECT_INSTANCE_ID=-1\nG10\n");
        REQUIRE(reset_pos != std::string::npos);
        const size_t second_start = output.find("unique label id: 1\nM624 ");
        const size_t unretract = output.find("G11\n");
        REQUIRE(second_start != std::string::npos);
        REQUIRE(unretract != std::string::npos);
        CHECK(reset_pos < second_start);
        CHECK(second_start < unretract);
        CHECK(output.find(" E-") == std::string::npos);
        const std::vector<double> after_unretract = e_values_after(output, unretract);
        REQUIRE(after_unretract.size() == 1);
        CHECK(after_unretract[0] == Approx(first_extrusion_e_after(output, output.find("M624 "))));
    }
}

TEST_CASE("Bambu firmware labels the real pipeline output without losing filament",
          "[plugins][gcode][firmware][bambu][labels][pipeline]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();

    // Export two cubes with real retractions, in absolute or relative E.
    const auto export_two_cubes = [&orchestrator](bool relative_e) {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.set_num_extruders(1);
        config.set_deserialize_strict({
            {"gcode_flavor", "bambu"},
            {"gcode_exporter_plugin", "gcode.exporter.simple"},
            {"binary_gcode", "1"},
            {"gcode_label_objects", "firmware"},
            {"use_relative_e_distances", relative_e ? "1" : "0"},
            {"retract_length", "2"},
            {"retract_before_travel", "1"},
            // Skirt and brim are outside work: keep only the objects between labels.
            {"skirts", "0"},
            {"brim_width", "0"}});
        Model model;
        Print print;
        Test::init_print({Test::TestMesh::cube_20x20x20, Test::TestMesh::cube_20x20x20}, print, model, config);
        print.process();
        const boost::filesystem::path output_path = temporary_gcode_path();
        remove_output_pair(output_path);
        orchestrator.export_gcode(print, output_path.string(), nullptr);
        const std::string output = read_text_file(output_path);
        remove_output_pair(output_path);
        return output;
    };

    const std::string absolute = export_two_cubes(false);
    const std::string relative = export_two_cubes(true);

    const size_t labelled = count_occurrences(absolute, "M624 ");
    CHECK(labelled >= 2);
    CHECK(count_occurrences(absolute, "M625\n") == labelled);
    CHECK(count_occurrences(absolute, "M625\nG92 E0\n") == labelled);
    CHECK(count_occurrences(relative, "M625\n") == labelled);
    CHECK(count_occurrences(relative, "M625\nG92 E0") == 0);
    // Real retractions are present in the exports (E-only moves at the retract speed).
    CHECK(count_occurrences(absolute, " F2400\n") > 0);

    // Layer printing: the travel to an object is written after its label, so no
    // XY move (G0 or G1) sits between a M625 and the next M624. Moves after the
    // last M625 belong to end_gcode and are not counted.
    std::istringstream stream(absolute);
    std::string line;
    size_t gap_moves = 0;
    bool after_object_end = false;
    size_t xy_moves_between_objects = 0;
    while (std::getline(stream, line)) {
        if (line == "M625") {
            gap_moves = 0;
            after_object_end = true;
        } else if (line.rfind("M624 ", 0) == 0) {
            xy_moves_between_objects += gap_moves;
            gap_moves = 0;
        } else if (after_object_end && (line.rfind("G0 ", 0) == 0 || line.rfind("G1 ", 0) == 0) &&
                   (line.find(" X") != std::string::npos || line.find(" Y") != std::string::npos))
            ++gap_moves;
    }
    CHECK(xy_moves_between_objects == 0);

    // No aberrant E jump, and the same net filament as the relative export.
    double largest_absolute = 0.0;
    double largest_relative = 0.0;
    const double absolute_total = net_filament(absolute, false, largest_absolute);
    const double relative_total = net_filament(relative, true, largest_relative);
    CHECK(absolute_total > 100.0);
    CHECK(largest_absolute < 10.0);
    CHECK(largest_relative < 10.0);
    CHECK(absolute_total == Approx(relative_total).margin(0.05));
}

TEST_CASE("Bambu firmware opens and closes a labelled block with the tool's retraction depth",
          "[plugins][gcode][firmware][bambu][labels][retraction][block]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    SECTION("(a) first block: exact retraction before M624, exact unretraction before a complete first extrusion") {
        const std::vector<BlockObject> one_object = {BlockObject{0}};
        for (const bool relative_e : {true, false}) {
            INFO(relative_e);
            const std::string labelled = run_bambu_blocks({2.0}, relative_e, false, one_object);
            const std::string plain = run_bambu_blocks({2.0}, relative_e, false, one_object, LabelObjectsStyle::Disabled);
            const double expected = first_deposit_e_after(plain, 0);
            CHECK(expected > 0.0);
            CHECK(first_deposit_e_after(labelled, labelled.find("M624 ")) == Approx(expected));
            CHECK(e_only_moves(plain).empty());
            if (relative_e) {
                const std::vector<double> moves = e_only_moves(labelled);
                REQUIRE(moves.size() == 3);
                CHECK(moves[0] == Approx(-2.0));
                CHECK(moves[1] == Approx(2.0));
                CHECK(moves[2] == Approx(-2.0));
                CHECK(labelled.find("G1 E-2 F2400\n") < labelled.find("M624 "));
                CHECK(labelled.find("G1 E2 F1800\n") > labelled.find("M624 "));
            }
        }
    }

    SECTION("(a2) two tools: the retraction runs on the active tool, the tool change comes after the block is closed") {
        // Tool 1 (depth 3) then tool 0 (depth 2), changed as the plan does (a TOOLCHANGE
        // command in the events of the target group) or by automatic selection, in
        // relative E, absolute E and native retraction.
        const std::vector<BlockObject> objects = {BlockObject{1}, BlockObject{0}};
        const ToolSwitch mode = GENERATE(ToolSwitch::Semantic, ToolSwitch::Automatic, ToolSwitch::Custom);
        const int variant = GENERATE(0, 1, 2); // relative E, absolute E, native retraction
        CAPTURE(int(mode), variant);
        std::vector<double> used;
        std::vector<double> baseline_used;
        const std::string output = run_bambu_blocks({2.0, 3.0}, variant == 0, variant == 2, objects,
                                                    LabelObjectsStyle::Firmware, mode, &used);
        run_bambu_blocks({2.0, 3.0}, variant == 0, variant == 2, objects,
                         LabelObjectsStyle::Disabled, mode, &baseline_used);
        INFO(output);
        const DebtTrace trace = trace_debt(output, 99.0);
        // entry, unretraction and exit of each block, each on the tool of its block
        if (variant == 2) {
            REQUIRE(trace.events.size() == 6);
            const char kinds[] = {'R', 'U', 'R', 'R', 'U', 'R'};
            const int tools[] = {1, 1, 1, 0, 0, 0};
            for (size_t idx = 0; idx < 6; ++idx) {
                CHECK(trace.events[idx].kind == kinds[idx]);
                CHECK(trace.events[idx].tool == tools[idx]);
            }
        } else {
            REQUIRE(trace.events.size() == 6);
            const double values[] = {-3.0, 3.0, -3.0, -2.0, 2.0, -2.0};
            const int tools[] = {1, 1, 1, 0, 0, 0};
            for (size_t idx = 0; idx < 6; ++idx) {
                CHECK(trace.events[idx].kind == 'E');
                CHECK(trace.events[idx].tool == tools[idx]);
                if (variant == 0)
                    CHECK(trace.events[idx].value == Approx(values[idx]));
            }
        }
        // the block of tool 1 is closed before T0 is written
        CHECK(output.find("M625\n") < output.find("T0\n"));
        // the filament counted for each tool does not depend on the labels
        REQUIRE(used.size() == 2);
        REQUIRE(baseline_used.size() == 2);
        CHECK(used[0] == Approx(baseline_used[0]).margin(1e-4));
        CHECK(used[1] == Approx(baseline_used[1]).margin(1e-4));
    }

    SECTION("(b) no pair is added between two blocks that are already balanced") {
        const std::vector<BlockObject> objects = {
            BlockObject{0, -1.0, false, 2.0},
            BlockObject{0, -1.0, true, 2.0}};
        const std::string output = run_bambu_blocks({2.0}, true, false, objects);
        INFO(output);
        // entry complement, explicit unretract (first block), then the plan's
        // retract / unretract / retract: five E-only moves and nothing between blocks.
        const std::vector<double> moves = e_only_moves(output);
        REQUIRE(moves.size() == 5);
        CHECK(moves[0] == Approx(-2.0));
        CHECK(moves[1] == Approx(2.0));
        CHECK(moves[2] == Approx(-2.0));
        CHECK(moves[3] == Approx(2.0));
        CHECK(moves[4] == Approx(-2.0));
        const size_t first_end = output.find("M625\n");
        const size_t second_start = output.find("M624 ", first_end);
        REQUIRE(first_end != std::string::npos);
        REQUIRE(second_start != std::string::npos);
        CHECK(e_only_moves(output.substr(first_end, second_start - first_end)).empty());
    }

    SECTION("(c) a partial debt is completed, not replaced") {
        const std::vector<BlockObject> objects = {BlockObject{0, 0.5}};
        const std::string output = run_bambu_blocks({2.0}, true, false, objects);
        INFO(output);
        // prior retraction, completed entry retraction, unretract, exit retraction
        const std::vector<double> moves = e_only_moves(output);
        REQUIRE(moves.size() == 4);
        CHECK(moves[0] == Approx(-0.5));
        CHECK(moves[1] == Approx(-1.5));
        CHECK(moves[2] == Approx(2.0));
        CHECK(moves[3] == Approx(-2.0));
        CHECK(output.find("G1 E-1.5") < output.find("M624 "));
    }

    SECTION("(c2) a block entered deeper than retract_length leaves as deep as it entered") {
        // Debt 3 at entry (more than the depth 2): no complement, and the exit
        // retraction goes back to 3, not to 2.
        const std::vector<BlockObject> objects = {BlockObject{0, 3.0}};
        const std::string output = run_bambu_blocks({2.0}, true, false, objects);
        INFO(output);
        const std::vector<double> moves = e_only_moves(output);
        REQUIRE(moves.size() == 3);
        CHECK(moves[0] == Approx(-3.0));
        CHECK(moves[1] == Approx(3.0));
        CHECK(moves[2] == Approx(-3.0));
        const DebtTrace trace = trace_debt(output, 99.0);
        REQUIRE(trace.at_block_start.size() == 1);
        CHECK(trace.at_block_start[0] == Approx(3.0));
        CHECK(trace.at_block_end[0] == Approx(3.0));
    }

    SECTION("(d) native retraction: G10 before M624, G11 in the block, nothing extra") {
        const std::vector<BlockObject> objects = {
            BlockObject{0, -1.0, false, 2.0},
            BlockObject{0, -1.0, true, 2.0}};
        const std::string output = run_bambu_blocks({2.0}, true, true, objects);
        INFO(output);
        const size_t first_start = output.find("M624 ");
        const size_t first_move = output.find("\nG1 X");
        REQUIRE(first_start != std::string::npos);
        REQUIRE(first_move != std::string::npos);
        CHECK(output.find("G10\n") < first_start);
        CHECK(output.find("G11\n") > first_start);
        CHECK(output.find("G11\n") < first_move);
        // entry G10, the plan's retract and unretract, its final retract
        CHECK(count_occurrences(output, "G10\n") == 3);
        CHECK(count_occurrences(output, "G11\n") == 2);
        CHECK(output.find(" E-") == std::string::npos);
    }

    SECTION("(d2) native retraction already active: no complement, the firmware keeps its depth") {
        const std::vector<BlockObject> objects = {BlockObject{0, 3.0}};
        const std::string output = run_bambu_blocks({2.0}, true, true, objects);
        INFO(output);
        const size_t first_start = output.find("M624 ");
        REQUIRE(first_start != std::string::npos);
        // the prior retraction is the only G10 before the label; the block unretracts
        // (G11) and its exit retracts again (G10)
        CHECK(count_occurrences(output.substr(0, first_start), "G10\n") == 1);
        CHECK(count_occurrences(output, "G10\n") == 2);
        CHECK(count_occurrences(output, "G11\n") == 1);
        CHECK(output.find(" E-") == std::string::npos);
    }

    SECTION("(d3) native retraction active but shallower than retract_length: the firmware's depth is kept") {
        // A native retraction of 0.5 is active at entry: no complement (no G10 before
        // the label, no logical deepening to 2), and the block leaves it at 0.5.
        const std::vector<BlockObject> objects = {BlockObject{0, 0.5}};
        std::vector<double> final_retracted;
        const std::string output = run_bambu_blocks({2.0}, true, true, objects, LabelObjectsStyle::Firmware,
                                                    ToolSwitch::Semantic, nullptr, &final_retracted);
        INFO(output);
        const size_t first_start = output.find("M624 ");
        REQUIRE(first_start != std::string::npos);
        CHECK(count_occurrences(output.substr(0, first_start), "G10\n") == 1);
        REQUIRE(final_retracted.size() == 1);
        CHECK(final_retracted[0] == Approx(0.5));
    }

    SECTION("(g) a block that the plan leaves unretracted is closed retracted") {
        // No retraction event at all (a layer change without retraction): the base
        // retracts before each end label, so no block ends with a lower debt.
        const std::vector<BlockObject> objects = {BlockObject{0}, BlockObject{0}};
        const std::string output = run_bambu_blocks({2.0}, true, false, objects);
        INFO(output);
        const std::vector<double> moves = e_only_moves(output);
        REQUIRE(moves.size() == 5);
        CHECK(moves[0] == Approx(-2.0));
        CHECK(moves[1] == Approx(2.0));
        CHECK(moves[2] == Approx(-2.0));
        CHECK(moves[3] == Approx(2.0));
        CHECK(moves[4] == Approx(-2.0));
        // the exit retraction is inside the block, before its end label
        const size_t first_end = output.find("; stop printing object");
        REQUIRE(first_end != std::string::npos);
        CHECK(output.rfind("G1 E-2 F2400\n", first_end) > output.find("M624 "));
        const DebtTrace trace = trace_debt(output, 2.0);
        REQUIRE(trace.at_block_start.size() == 2);
        REQUIRE(trace.at_block_end.size() == 2);
        for (size_t idx = 0; idx < 2; ++idx) {
            CHECK(trace.at_block_start[idx] == Approx(2.0).margin(1e-3));
            CHECK(trace.at_block_end[idx] == Approx(2.0).margin(1e-3));
        }
    }

    SECTION("(h) a block after a wipe tower or after a script starts and ends at the depth") {
        const bool after_script = GENERATE(false, true);
        CAPTURE(after_script);
        std::vector<BlockObject> objects;
        if (after_script) {
            objects = {BlockObject{0}, BlockObject{0, -1.0, false, -1.0, false, true}};
        } else {
            BlockObject tower;
            tower.unretract_first = true;
            tower.retract_after = 2.0;
            tower.synthetic = true;
            objects = {BlockObject{0}, tower, BlockObject{0}};
        }
        const std::string output = run_bambu_blocks({2.0}, true, false, objects);
        INFO(output);
        const DebtTrace trace = trace_debt(output, 2.0);
        REQUIRE(trace.at_block_start.size() == 2);
        REQUIRE(trace.at_block_end.size() == 2);
        for (size_t idx = 0; idx < 2; ++idx) {
            CHECK(trace.at_block_start[idx] == Approx(2.0).margin(1e-3));
            CHECK(trace.at_block_end[idx] == Approx(2.0).margin(1e-3));
            CHECK(trace.before_first_deposit[idx] == Approx(0.0).margin(1e-3));
        }
        // No retraction is added outside the blocks: the gap holds the tower's own
        // unretract and retract (two E-only moves), or nothing for a script.
        const std::vector<size_t> gaps = e_only_moves_in_gaps(output);
        REQUIRE(gaps.size() == 1);
        CHECK(gaps[0] == (after_script ? size_t(0) : size_t(2)));
    }

    SECTION("(k) a tool change is never inside a block, even for a tool without retraction depth") {
        // Tool 0 has retract_length 0 (no block retraction at all), tool 1 has 3.
        const std::vector<BlockObject> objects = {BlockObject{0}, BlockObject{1}};
        const ToolSwitch mode = GENERATE(ToolSwitch::Semantic, ToolSwitch::Automatic, ToolSwitch::Custom);
        CAPTURE(int(mode));
        const std::string output = run_bambu_blocks({0.0, 3.0}, true, false, objects,
                                                    LabelObjectsStyle::Firmware, mode);
        INFO(output);
        const DebtTrace trace = trace_debt(output, 99.0);
        CHECK(trace.tool_selections_in_blocks == 0);
        REQUIRE(trace.at_block_start.size() == 2);
        CHECK(trace.tool_at_block_start[0] == 0);
        CHECK(trace.tool_at_block_end[0] == 0);
        CHECK(trace.tool_at_block_start[1] == 1);
        CHECK(trace.tool_at_block_end[1] == 1);
        // the first block is closed before T1 is written
        CHECK(output.find("M625\n") < output.find("T1\n"));
        // tool 0 gets no retraction, tool 1 its entry, unretraction and exit
        const std::vector<double> moves = e_only_moves(output);
        REQUIRE(moves.size() == 3);
        CHECK(moves[0] == Approx(-3.0));
        CHECK(moves[1] == Approx(3.0));
        CHECK(moves[2] == Approx(-3.0));
    }

    SECTION("(i) nothing is added when no firmware label is written") {
        const std::vector<BlockObject> two_objects = {BlockObject{0}, BlockObject{0}};
        const std::string octoprint = run_bambu_blocks({2.0}, true, false, two_objects, LabelObjectsStyle::Octoprint);
        CHECK(octoprint.find("M624") == std::string::npos);
        CHECK(e_only_moves(octoprint).empty());
        const std::string many = run_bambu_blocks({2.0}, true, false, std::vector<BlockObject>(65));
        CHECK(many.find("M624") == std::string::npos);
        CHECK(e_only_moves(many).empty());
    }

    SECTION("(j) the executor counts a deposit on a retracted tool as repaying the debt") {
        const DebtTrace trace = trace_debt(
            "M83\nG1 E-2 F2400\nM624 AQAAAAAAAAA=\nG1 X1 E1.5 F1200\nM625\n", 2.0);
        REQUIRE(trace.at_block_start.size() == 1);
        REQUIRE(trace.before_first_deposit.size() == 1);
        CHECK(trace.at_block_start[0] == Approx(2.0));
        CHECK(trace.before_first_deposit[0] == Approx(2.0));
        CHECK(trace.at_block_end[0] == Approx(0.5));
    }
}

TEST_CASE("Bambu firmware keeps the filament state when a block is cancelled",
          "[plugins][gcode][firmware][bambu][labels][retraction][block][pipeline]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool relative_e = GENERATE(false, true);
    CAPTURE(relative_e);
    const std::string output = export_labelled_cubes(relative_e);
    const DebtTrace baseline = trace_debt(output, 2.0);

    // (e) every block starts and ends with the retraction depth (the plan has no
    // retraction at some exits, e.g. a layer change: the block retracts itself before
    // its end label) and its first deposit follows a complete unretraction.
    REQUIRE(baseline.at_block_start.size() >= 3);
    CHECK(baseline.at_block_end.size() == baseline.at_block_start.size());
    CHECK(baseline.before_first_deposit.size() == baseline.at_block_start.size());
    for (const double debt : baseline.at_block_start)
        CHECK(debt == Approx(2.0).margin(1e-3));
    for (const double debt : baseline.at_block_end)
        CHECK(debt == Approx(2.0).margin(1e-3));
    for (const double debt : baseline.before_first_deposit)
        CHECK(debt == Approx(0.0).margin(1e-3));

    // Between two blocks of the same tool nothing but the blocks' own retractions is
    // written: no E-only move in the gaps.
    const std::vector<size_t> gaps = e_only_moves_in_gaps(output);
    CHECK(gaps.size() + 1 >= baseline.at_block_start.size());
    for (const size_t count : gaps)
        CHECK(count == 0);

    // (f) cancelling an object removes all its blocks (every layer), as the firmware
    // does: first, middle and last object. The debt at the following boundaries and
    // at the end is unchanged.
    const std::vector<int> ids = block_object_ids(output);
    REQUIRE(ids.size() == baseline.at_block_start.size());
    for (const int object_id : {0, 1, 2}) {
        INFO("cancelled object " << object_id);
        std::vector<double> expected_start;
        std::vector<double> expected_end;
        for (size_t idx = 0; idx < ids.size(); ++idx) {
            if (ids[idx] == object_id)
                continue;
            expected_start.push_back(baseline.at_block_start[idx]);
            expected_end.push_back(baseline.at_block_end[idx]);
        }
        REQUIRE(expected_start.size() + 1 < baseline.at_block_start.size());
        const DebtTrace trace = trace_debt(cancel_object(output, object_id), 2.0);
        REQUIRE(trace.at_block_start.size() == expected_start.size());
        REQUIRE(trace.at_block_end.size() == expected_end.size());
        for (size_t idx = 0; idx < expected_start.size(); ++idx) {
            CHECK(trace.at_block_start[idx] == Approx(expected_start[idx]).margin(1e-3));
            CHECK(trace.at_block_end[idx] == Approx(expected_end[idx]).margin(1e-3));
        }
        CHECK(trace.final_debt == Approx(baseline.final_debt).margin(1e-3));
    }

    // (f2) cancelling one block alone (a layer slice of an object, which a printer
    // does not do) leaves the same debt at the following boundaries and at the end.
    for (const size_t cancelled : {size_t(0), baseline.at_block_start.size() / 2,
                                   baseline.at_block_start.size() - 1}) {
        INFO("cancelled block " << cancelled);
        const DebtTrace trace = trace_debt(cancel_block(output, cancelled), 2.0);
        REQUIRE(trace.at_block_start.size() + 1 == baseline.at_block_start.size());
        std::vector<double> expected_start = baseline.at_block_start;
        std::vector<double> expected_end = baseline.at_block_end;
        expected_start.erase(expected_start.begin() + std::ptrdiff_t(cancelled));
        expected_end.erase(expected_end.begin() + std::ptrdiff_t(cancelled));
        for (size_t idx = 0; idx < expected_start.size(); ++idx) {
            CHECK(trace.at_block_start[idx] == Approx(expected_start[idx]).margin(1e-3));
            CHECK(trace.at_block_end[idx] == Approx(expected_end[idx]).margin(1e-3));
        }
        CHECK(trace.final_debt == Approx(baseline.final_debt).margin(1e-3));
    }
}
TEST_CASE("Bambu firmware retracts the active tool around the blocks of a two-tool print",
          "[plugins][gcode][firmware][bambu][labels][retraction][block][pipeline][tools]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    // The plan's tool change (a TOOLCHANGE command) or a custom script (the tool is
    // then selected by begin_tool_group() without writing T; the script writes it).
    const std::string script = GENERATE(std::string(), std::string("; USER_TOOLCHANGE\\nT[next_extruder]"));
    // retract_length per tool; "0,3" gives tool 0 no block retraction at all
    const std::string lengths = GENERATE(std::string("2,3"), std::string("0,3"));
    CAPTURE(script, lengths);
    const std::string output = export_labelled_cubes(false, 2, script, lengths);
    const DebtTrace trace = trace_debt(output, 99.0);
    REQUIRE(trace.at_block_start.size() >= 4);
    CHECK(trace.at_block_end.size() == trace.at_block_start.size());
    CHECK(trace.before_first_deposit.size() == trace.at_block_start.size());
    // both tools are used
    size_t blocks_of_tool[2] = {0, 0};
    for (size_t idx = 0; idx < trace.at_block_start.size(); ++idx) {
        const int tool = trace.tool_at_block_start[idx];
        REQUIRE((tool == 0 || tool == 1));
        ++blocks_of_tool[tool];
        const double depth = tool == 0 ? (lengths == "0,3" ? 0.0 : 2.0) : 3.0;
        // entered at least at the tool's depth, with no unretraction owed before its
        // first deposit
        CHECK(trace.at_block_start[idx] >= depth - 1e-3);
        CHECK(trace.before_first_deposit[idx] == Approx(0.0).margin(1e-3));
    }
    CHECK(blocks_of_tool[0] > 0);
    CHECK(blocks_of_tool[1] > 0);
    // A tool change (T written by the plan, or by the custom script after the block
    // was closed) is never inside a block, and each block ends on its own tool.
    CHECK(trace.tool_selections_in_blocks == 0);
    REQUIRE(trace.tool_at_block_end.size() == trace.tool_at_block_start.size());
    for (size_t idx = 0; idx < trace.tool_at_block_start.size(); ++idx)
        CHECK(trace.tool_at_block_end[idx] == trace.tool_at_block_start[idx]);
    // A block never ends shallower than it began. The plan's tool-change retraction
    // (retract_length_toolchange, deeper than retract_length) is written inside the
    // block that precedes the tool change, so those blocks end deeper than their
    // entry depth: this is the documented limit, such a block is not normalized.
    size_t equal_ends = 0;
    size_t deeper_ends = 0;
    for (size_t idx = 0; idx < trace.at_block_start.size(); ++idx) {
        CHECK(trace.at_block_end[idx] >= trace.at_block_start[idx] - 1e-3);
        if (std::abs(trace.at_block_end[idx] - trace.at_block_start[idx]) < 1e-3)
            ++equal_ends;
        else
            ++deeper_ends;
    }
    if (lengths == "2,3") {
        CHECK(equal_ends > 0);
        CHECK(deeper_ends > 0);
    }
}
TEST_CASE("Bambu firmware opens and closes the spaghetti detector around the print",
          "[plugins][gcode][firmware][bambu][detector]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    SECTION("M981 S1 follows the start sequence and M981 S0 precedes end_gcode, once for two groups") {
        Print print;
        configure_standard_firmware(print);
        PrintConfig &config = const_cast<PrintConfig &>(print.config());
        // The marker closing start_gcode is its last line, so the detector must
        // come after it, not merely after the first line.
        config.option<ConfigOptionString>("start_gcode")->value = "; START_BEGIN\n; START_MARK";
        config.option<ConfigOptionString>("end_gcode")->value = "; END_MARK";
        PrintingPlan &plan = print.mutable_printing_plan();
        // Two groups of two layers each.
        for (size_t group_idx = 0; group_idx < 2; ++group_idx) {
            plan.groups.emplace_back();
            for (size_t layer_idx = 0; layer_idx < 2; ++layer_idx) {
                plan.groups.back().layers.emplace_back();
                PrintingLayerGroup &layer = plan.groups.back().layers.back();
                layer.print_z = scale_i(0.2 * double(layer_idx + 1));
                layer.tool_groups.emplace_back();
                PrintingToolGroup &tool = layer.tool_groups.back();
                tool.extruder_id = 0;
                const double x = (group_idx == 0 ? 0.0 : 4.0) + double(layer_idx);
                append_path_extrusion(tool, make_firmware_path(
                    ArcPolyline(Points{Point::new_scale(x, 0.), Point::new_scale(x + 1., 0.)}),
                    20.f, 500.f));
            }
        }
        Steps::StepExtrusionEdition::clear(print);
        Steps::StepExtrusionEdition::run_step(Orchestrator::instance(), print);
        // Work written after the last layer and after the last group, to prove the
        // detector closes after both and still before end_gcode.
        ExtrusionNop layer_marker;
        layer_marker.add_property(ExtrusionPropertyCustomGcodeText("; LAST_LAYER_AFTER"));
        plan.groups.back().layers.back().events.append_after(layer_marker);
        ExtrusionNop group_marker;
        group_marker.add_property(ExtrusionPropertyCustomGcodeText("; LAST_GROUP_AFTER"));
        plan.groups.back().events.append_after(group_marker);

        const std::string output = export_with_firmware(print, "bambu");
        INFO(output);
        const size_t start_mark = output.find("; START_MARK");
        const size_t open_detector = output.find("M981 S1 P20000 ;open spaghetti detector\n");
        const size_t first_layer = output.find(";LAYER_CHANGE");
        const size_t last_move = output.rfind("G1 X6");
        const size_t last_layer_after = output.find("; LAST_LAYER_AFTER");
        const size_t last_group_after = output.find("; LAST_GROUP_AFTER");
        const size_t close_detector = output.find("M981 S0 P20000 ; close spaghetti detector\n");
        const size_t end_mark = output.find("; END_MARK");
        REQUIRE(start_mark != std::string::npos);
        REQUIRE(open_detector != std::string::npos);
        REQUIRE(first_layer != std::string::npos);
        REQUIRE(last_move != std::string::npos);
        REQUIRE(last_layer_after != std::string::npos);
        REQUIRE(last_group_after != std::string::npos);
        REQUIRE(close_detector != std::string::npos);
        REQUIRE(end_mark != std::string::npos);
        CHECK(start_mark < open_detector);
        CHECK(open_detector < first_layer);
        CHECK(last_move < close_detector);
        CHECK(last_layer_after < close_detector);
        CHECK(last_group_after < close_detector);
        CHECK(close_detector < end_mark);
        CHECK(count_occurrences(output, ";LAYER_CHANGE") == 4);
        CHECK(count_occurrences(output, "M981 S1") == 1);
        CHECK(count_occurrences(output, "M981 S0") == 1);
    }

    SECTION("a plan without group opens the detector and never closes it") {
        Print print;
        configure_standard_firmware(print);
        print.mutable_printing_plan();
        const std::string output = export_with_firmware(print, "bambu");
        CHECK(count_occurrences(output, "M981 S1 P20000 ;open spaghetti detector\n") == 1);
        CHECK(output.find("M981 S0") == std::string::npos);
    }
}

TEST_CASE("Bambu firmware writes pressure advance with four significant digits",
          "[plugins][gcode][firmware][bambu][pressure-advance]")
{
    BambuFirmwareProbe firmware;
    CHECK(firmware.encode_pressure_advance(0, 0.12345) ==
          "M900 K0.1235 L1000 M10 ; Override pressure advance value\n");
    CHECK(firmware.encode_pressure_advance(0, 0.02) ==
          "M900 K0.02 L1000 M10 ; Override pressure advance value\n");
    CHECK(firmware.encode_pressure_advance(0, 0.0225) ==
          "M900 K0.0225 L1000 M10 ; Override pressure advance value\n");
}

namespace {

// Index of the first move (a G1 with X, Y or Z) at or after `from`, or the line count.
size_t first_move_after(const std::vector<std::string> &lines, size_t from)
{
    for (size_t idx = from; idx < lines.size(); ++idx)
        if (lines[idx].rfind("G1 ", 0) == 0 && (lines[idx].find(" X") != std::string::npos ||
            lines[idx].find(" Y") != std::string::npos || lines[idx].find(" Z") != std::string::npos))
            return idx;
    return lines.size();
}

// True when the line at `idx` lies between an M624 and its M625.
bool inside_block(const std::vector<std::string> &lines, size_t idx)
{
    bool inside = false;
    for (size_t pos = 0; pos < idx; ++pos) {
        if (lines[pos].rfind("M624 ", 0) == 0)
            inside = true;
        else if (lines[pos] == "M625")
            inside = false;
    }
    return inside;
}

const std::string k_scan_line = "M976 S1 P1 ; scan model before printing 2nd layer";

// Exposes the retraction helper of the base session and the tool's retraction state.
class RetractionHelperProbe final :
    public slic3r_api::GCodeGeneration::Firmware::Marlin2GCodeFirmwareSession
{
public:
    using Marlin2GCodeFirmwareSession::write_with_retraction;
    double retracted() { return extruders().at(0).extrusion_axis().retracted(); }
    void plan_extra(double value, bool toolchange) { extruders().at(0).extrusion_axis().schedule_restart_extra(value, toolchange); }
    double extra() { return extruders().at(0).extrusion_axis().restart_extra(); }
    double extra_toolchange() { return extruders().at(0).extrusion_axis().restart_extra_toolchange(); }
};

// What run_retraction_helper() observed after the call.
struct HelperAfter
{
    double retracted = -1.0;
    double extra = -1.0;
    double extra_toolchange = -1.0;
    std::string next_unretraction;  // text of an unretraction event written right after (it carries its own plain restart extra, 0, so only the tool-change extra is deposited)
};

// Select tool 0, retract it by `initial` mm when positive, then wrap the content "X\n"
// with write_with_retraction(length). Returns the text of that call only and the
// retraction left on the tool. `extra` / `extra_toolchange` are restart extras scheduled
// before the call; `after` receives the state seen once the call returned.
std::string run_retraction_helper(double initial, double length, bool firmware_retraction, double &retracted_after,
                                  double extra = 0., double extra_toolchange = 0., HelperAfter *after = nullptr)
{
    Model model;
    Print print;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model);
    configure_standard_firmware(print);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.gcode_label_objects.value = LabelObjectsStyle::Disabled;
    config.use_relative_e_distances.value = true;
    config.use_firmware_retraction.value = firmware_retraction;
    config.retract_length.set(std::vector<double>{2.0});
    PrintingPlan plan;
    PrintingToolGroup tool;
    tool.extruder_id = 0;
    append_path_extrusion(tool, make_firmware_path(
        ArcPolyline(Points{Point::new_scale(0., 0.), Point::new_scale(1., 0.)}), 20.f, 500.f));
    RetractionHelperProbe session;
    session.begin_print(slic3r_api::Print(reinterpret_cast<const print_handle *>(&print)),
        slic3r_api::ConstPrintingPlan(reinterpret_cast<const printing_plan_handle *>(&plan)));
    session.begin_tool_group(slic3r_api::ConstPrintingToolGroup(
        reinterpret_cast<const printing_tool_group_handle *>(&tool)));
    if (initial > 0.0) {
        const std::unique_ptr<ExtrusionNop> event = make_axis_event(true, initial);
        session.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(event.get())));
    }
    if (extra != 0.)
        session.plan_extra(extra, false);
    if (extra_toolchange != 0.)
        session.plan_extra(extra_toolchange, true);
    const std::string output = session.write_with_retraction("X\n", length);
    retracted_after = session.retracted();
    if (after) {
        after->retracted = retracted_after;
        after->extra = session.extra();
        after->extra_toolchange = session.extra_toolchange();
        const std::unique_ptr<ExtrusionNop> repay = make_axis_event(false, 0.0);
        after->next_unretraction = session.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(repay.get())));
    }
    return output;
}

} // namespace

TEST_CASE("The base session frames dialect text with a retraction pair",
          "[plugins][gcode][firmware][retraction]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    double retracted = -1.0;

    SECTION("a tool that is not retracted is retracted, then repaid") {
        const std::vector<std::string> lines = split_lines(run_retraction_helper(0., 2., false, retracted));
        REQUIRE(lines.size() >= 3);
        CHECK(lines.front().rfind("G1 E-2", 0) == 0);
        const size_t content = find_line(lines, "X");
        REQUIRE(content == 1);
        CHECK(lines[content + 1].rfind("G1 E2", 0) == 0);
        CHECK(lines.size() == 3);
        CHECK(retracted == Approx(0.).margin(1e-6));
    }
    SECTION("a partly retracted tool only gets the complement, and keeps its debt") {
        const std::vector<std::string> lines = split_lines(run_retraction_helper(0.8, 2., false, retracted));
        REQUIRE(lines.size() == 2);
        CHECK(lines[0].rfind("G1 E-1.2", 0) == 0);
        CHECK(lines[1] == "X");
        CHECK(retracted == Approx(2.).margin(1e-6));
    }
    SECTION("a tool at least as deep as the length gets the content alone") {
        const std::vector<std::string> lines = split_lines(run_retraction_helper(3., 2., false, retracted));
        CHECK(lines == std::vector<std::string>{"X"});
        CHECK(retracted == Approx(3.).margin(1e-6));
    }
    SECTION("firmware retraction uses G10 and G11") {
        const std::vector<std::string> lines = split_lines(run_retraction_helper(0., 2., true, retracted));
        const size_t retract = find_line(lines, "G10");
        const size_t content = find_line(lines, "X");
        const size_t unretract = find_line(lines, "G11");
        REQUIRE(retract < content);
        REQUIRE(content < unretract);
        CHECK(unretract < lines.size());
        CHECK(retracted == Approx(0.).margin(1e-6));
    }
    SECTION("an active firmware retraction is kept as is, without a complement") {
        const std::vector<std::string> lines = split_lines(run_retraction_helper(0.8, 2., true, retracted));
        CHECK(lines == std::vector<std::string>{"X"});
        CHECK(retracted == Approx(0.8).margin(1e-6));
    }
    SECTION("restart extras stay scheduled for the next real unretraction") {
        for (const bool firmware : {false, true}) {
            INFO("firmware retraction " << firmware);
            HelperAfter after;
            const std::vector<std::string> lines =
                split_lines(run_retraction_helper(0., 2., firmware, retracted, 0.3, 0.5, &after));
            REQUIRE(lines.size() == 3);
            if (!firmware) {
                // the temporary pair moves exactly the retraction, without the extras
                CHECK(lines.front().rfind("G1 E-2", 0) == 0);
                CHECK(lines.back().rfind("G1 E2 ", 0) == 0);
            }
            CHECK(after.retracted == Approx(0.).margin(1e-6));
            CHECK(after.extra == Approx(0.3).margin(1e-6));
            CHECK(after.extra_toolchange == Approx(0.5).margin(1e-6));
            INFO(after.next_unretraction);
            if (!firmware)
                CHECK(after.next_unretraction.find("G1 E.5\n") != std::string::npos);
            else
                CHECK(after.next_unretraction.find("G11") != std::string::npos);
        }
    }
    SECTION("a length of zero writes the content alone") {
        CHECK(run_retraction_helper(0., 0., false, retracted) == "X\n");
        CHECK(retracted == Approx(0.).margin(1e-6));
    }
}

TEST_CASE("Bambu firmware writes nothing for M1003 and M976 without the variables",
          "[plugins][gcode][firmware][bambu][scan]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *const variables[] = {
        "", "other = 1", "scan_first_layer = 0\nscan_first_layer = false",
        "enable_power_loss_recovery = printer_configuration", "enable_power_loss_recovery = Enable",
        "enable_power_loss_recovery = enabled", "enable_power_loss_recovery = disabled"};
    for (const char *const text : variables) {
        INFO(text);
        SecondLayerRun run;
        run.variables = text;
        const std::string output = run_bambu_second_layer(run);
        CHECK(output.find("M1003") == std::string::npos);
        CHECK(output.find("M976") == std::string::npos);
        CHECK(output.find("M400 P100") == std::string::npos);
    }
}

TEST_CASE("Bambu firmware switches power loss recovery per copy across groups",
          "[plugins][gcode][firmware][bambu][scan]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    SECTION("enable: overlapping second-layer copies share one start and close after both finish") {
        SecondLayerRun run;
        run.variables = "enable_power_loss_recovery = enable";
        run.groups = 2;
        const std::vector<std::string> lines = split_lines(run_bambu_second_layer(run));
        CHECK(std::count(lines.begin(), lines.end(), "M1003 S1") == 1);
        CHECK(std::count(lines.begin(), lines.end(), "M1003 S0") == 1);
        size_t next_group = find_line(lines, ";GROUP 1");
        for (size_t group_begin : {find_line(lines, ";GROUP 0"), next_group}) {
            const size_t group_end = group_begin == next_group ? lines.size() : next_group;
            const size_t layer_one = find_line(lines, "; LAYER_GCODE 1", group_begin);
            const size_t layer_two = find_line(lines, "; LAYER_GCODE 2", group_begin);
            const size_t enable = find_line(lines, "M1003 S1", group_begin);
            const size_t disable = find_line(lines, "M1003 S0", group_begin);
            REQUIRE(layer_one < group_end);
            if (group_begin != next_group) {
                CHECK(layer_one < enable);
                CHECK(enable < first_move_after(lines, layer_one));
                CHECK(enable < layer_two);
            } else {
                CHECK(enable == lines.size());
            }
            // S0 is deferred until the active copies finish, not each group boundary.
            size_t last_move = group_begin;
            for (size_t idx = group_begin; idx < group_end; ++idx)
                if (lines[idx].rfind("G1 ", 0) == 0 && lines[idx].find(" X") != std::string::npos)
                    last_move = idx;
            CHECK(disable > last_move);
            if (group_begin == next_group) CHECK(disable < group_end);
            else CHECK(disable > group_end);
            CHECK_FALSE(inside_block(lines, enable));
            CHECK_FALSE(inside_block(lines, disable));
        }
        CHECK(find_line(lines, "M1003 S0", find_line(lines, ";GROUP 1")) < find_line(lines, "M981 S0 P20000 ; close spaghetti detector"));
    }
    SECTION("disable: one S0 for the shared second-layer pass") {
        SecondLayerRun run;
        run.variables = "enable_power_loss_recovery = disable";
        const std::vector<std::string> lines = split_lines(run_bambu_second_layer(run));
        CHECK(std::count(lines.begin(), lines.end(), "M1003 S0") == 1);
        CHECK(std::count(lines.begin(), lines.end(), "M1003 S1") == 0);
        const size_t layer_one = find_line(lines, "; LAYER_GCODE 1");
        const size_t command = find_line(lines, "M1003 S0");
        CHECK(layer_one < command);
        CHECK(command < first_move_after(lines, layer_one));
    }
}

TEST_CASE("Bambu firmware scans the second layer between a retraction pair",
          "[plugins][gcode][firmware][bambu][scan]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    SECTION("the scan follows layer_gcode and M1003, precedes the first move, outside the blocks") {
        const char *const spellings[] = {"scan_first_layer = 1", "scan_first_layer = true",
                                         " scan_first_layer = TRUE \r\n", "scan_first_layer = \"1\""};
        for (const char *const spelling : spellings) {
            INFO(spelling);
            SecondLayerRun run;
            run.variables = std::string("enable_power_loss_recovery = enable\n") + spelling;
            const std::string output = run_bambu_second_layer(run);
            const std::vector<std::string> lines = split_lines(output);
            REQUIRE(std::count(lines.begin(), lines.end(), k_scan_line) == 1);
            const size_t scan = find_line(lines, k_scan_line);
            const size_t layer_one = find_line(lines, "; LAYER_GCODE 1");
            CHECK(lines[scan + 1] == "M400 P100");
            CHECK(layer_one < find_line(lines, "M1003 S1"));
            CHECK(find_line(lines, "M1003 S1") < scan);
            CHECK(scan < first_move_after(lines, layer_one));
            CHECK_FALSE(inside_block(lines, scan));
            // Every labelled block (two objects, three layers) keeps its entry debt and
            // is complete when its first geometry is deposited.
            const DebtTrace trace = trace_debt(output, 2.);
            REQUIRE(trace.at_block_start.size() == 6);
            for (size_t idx = 0; idx < 6; ++idx) {
                CHECK(trace.at_block_start[idx] == Approx(2.).margin(1e-4));
                CHECK(trace.at_block_end[idx] == Approx(2.).margin(1e-4));
                CHECK(trace.before_first_deposit[idx] == Approx(0.).margin(1e-4));
            }
        }
    }
    SECTION("a tool left retracted by the previous layer gets no extra pair") {
        SecondLayerRun run;
        run.variables = "scan_first_layer = 1";
        const std::vector<std::string> lines = split_lines(run_bambu_second_layer(run));
        const size_t scan = find_line(lines, k_scan_line);
        REQUIRE(scan + 2 < lines.size());
        // The previous block ended at the entry depth: neither a retraction before the
        // scan nor an unretraction after it (E-only moves of the gap between the layers).
        size_t previous_e_move = scan;
        while (previous_e_move > 0 && lines[previous_e_move].rfind("G1 E", 0) != 0)
            --previous_e_move;
        CHECK(lines[previous_e_move].find(" X") == std::string::npos);
        CHECK(previous_e_move < find_line(lines, "; LAYER_GCODE 1"));
        for (size_t idx = scan + 2; idx < find_line(lines, "M624 AQAAAAAAAAA=", scan); ++idx)
            CHECK(lines[idx].rfind("G1 E", 0) != 0);
    }
    SECTION("a repaid tool is retracted and repaid once for the shared scan") {
        SecondLayerRun run;
        run.variables = "scan_first_layer = 1";
        run.relative_e = true;
        run.repay_before_second_layer = true;
        const std::string output = run_bambu_second_layer(run);
        const std::vector<std::string> lines = split_lines(output);
        const size_t scan = find_line(lines, k_scan_line);
        REQUIRE(scan + 3 < lines.size());
        CHECK(lines[scan - 1].rfind("G1 E-2", 0) == 0);
        CHECK(lines[scan + 1] == "M400 P100");
        CHECK(lines[scan + 2].rfind("G1 E2", 0) == 0);
        // Both copies share this framed scan, with no second retraction pair.
        CHECK(find_line(lines, k_scan_line, scan + 1) == lines.size());
        for (size_t idx = scan + 3; idx < find_line(lines, "M624 AQAAAAAAAAA=", scan); ++idx)
            if (lines[idx].rfind("G1 E", 0) == 0 && lines[idx].find(" X") == std::string::npos)
                CHECK(lines[idx].rfind("G1 E-", 0) == 0);
        const DebtTrace trace = trace_debt(output, 2.);
        for (size_t idx = 0; idx < trace.before_first_deposit.size(); ++idx)
            CHECK(trace.before_first_deposit[idx] == Approx(0.).margin(1e-4));
    }
    SECTION("firmware retraction frames the scan with G10 and G11") {
        SecondLayerRun run;
        run.variables = "scan_first_layer = 1";
        run.firmware_retraction = true;
        run.repay_before_second_layer = true;
        const std::vector<std::string> lines = split_lines(run_bambu_second_layer(run));
        const size_t scan = find_line(lines, k_scan_line);
        REQUIRE(scan + 2 < lines.size());
        CHECK(lines[scan - 1] == "G10");
        CHECK(lines[scan + 1] == "M400 P100");
        CHECK(lines[scan + 2] == "G11");
    }
    SECTION("a retract length of zero writes the scan alone") {
        SecondLayerRun run;
        run.variables = "scan_first_layer = 1";
        run.retract_length = 0.;
        const std::vector<std::string> lines = split_lines(run_bambu_second_layer(run));
        const size_t scan = find_line(lines, k_scan_line);
        REQUIRE(scan + 2 < lines.size());
        CHECK(lines[scan - 1].rfind("G1 E", 0) != 0);
        for (size_t idx = scan + 2; idx < find_line(lines, "M624 AQAAAAAAAAA=", scan); ++idx)
            CHECK(lines[idx].rfind("G1 E", 0) != 0);
    }
    SECTION("two tool groups do not duplicate the per-copy commands") {
        SecondLayerRun run;
        run.variables = "enable_power_loss_recovery = enable\nscan_first_layer = 1";
        run.lengths = {2., 2.};
        run.second_tool_group = true;
        const std::vector<std::string> lines = split_lines(run_bambu_second_layer(run));
        CHECK(std::count(lines.begin(), lines.end(), k_scan_line) == 1);
        CHECK(std::count(lines.begin(), lines.end(), "M1003 S1") == 1);
        const size_t scan = find_line(lines, k_scan_line);
        CHECK(find_line(lines, "; LAYER_GCODE 1") < scan);
        CHECK(scan < first_move_after(lines, find_line(lines, "; LAYER_GCODE 1")));
        CHECK(scan < find_line(lines, "T1"));
        CHECK(find_line(lines, "M1003 S1") < scan);
        CHECK(find_line(lines, "M1003 S1") < find_line(lines, "T1"));
    }
    SECTION("an ordered tool change at the start of the layer retracts the tool still selected") {
        SecondLayerRun run;
        run.variables = "scan_first_layer = 1";
        run.lengths = {2., 3.};
        run.labels = false;
        run.relative_e = true;
        run.repay_before_second_layer = true;
        run.ordered_toolchange = true;
        const std::vector<std::string> lines = split_lines(run_bambu_second_layer(run));
        const size_t scan = find_line(lines, k_scan_line);
        REQUIRE(scan + 2 < lines.size());
        CHECK(lines[scan - 1].rfind("G1 E-2 ", 0) == 0);
        CHECK(lines[scan + 2].rfind("G1 E2 ", 0) == 0);
        // the change itself comes after the scan
        CHECK(scan < find_line(lines, "T1"));
    }
    SECTION("the retraction pair of the scan stays outside the blocks when an object is cancelled") {
        SecondLayerRun run;
        run.variables = "scan_first_layer = 1";
        run.relative_e = true;
        run.repay_before_second_layer = true;
        const std::string output = run_bambu_second_layer(run);
        for (const int cancelled_object : {-1, 0, 1}) {
            INFO("cancelled object " << cancelled_object);
            const std::vector<std::string> lines = split_lines(cancel_object(output, cancelled_object));
            const size_t scan = find_line(lines, k_scan_line);
            REQUIRE(scan + 2 < lines.size());
            CHECK(lines[scan - 1].rfind("G1 E-2 ", 0) == 0);
            CHECK(lines[scan + 2].rfind("G1 E2 ", 0) == 0);
            CHECK_FALSE(inside_block(lines, scan - 1));
            CHECK_FALSE(inside_block(lines, scan + 2));
        }
    }
    SECTION("repeated groups do not scan the same copies again") {
        SecondLayerRun run;
        run.variables = "scan_first_layer = 1";
        run.groups = 2;
        const std::vector<std::string> lines = split_lines(run_bambu_second_layer(run));
        CHECK(std::count(lines.begin(), lines.end(), k_scan_line) == 1);
        const size_t second_group = find_line(lines, ";GROUP 1");
        CHECK(find_line(lines, "; LAYER_GCODE 1") < find_line(lines, k_scan_line));
        CHECK(find_line(lines, k_scan_line) < second_group);
        CHECK(find_line(lines, k_scan_line, second_group) == lines.size());
    }
    SECTION("the commands survive the cancellation of the first object of the layer") {
        SecondLayerRun run;
        run.variables = "enable_power_loss_recovery = enable\nscan_first_layer = 1";
        run.groups = 2;
        run.layer_script = false;
        const std::string output = run_bambu_second_layer(run);
        for (const int cancelled_object : {0, 1}) {
            const std::string kept = cancel_object(output, cancelled_object);
            CHECK(count_occurrences(kept, "M1003 S1\n") == 1);
            CHECK(count_occurrences(kept, "M1003 S0\n") == 1);
            CHECK(count_occurrences(kept, k_scan_line + "\nM400 P100\n") == 1);
            CHECK(kept.size() < output.size());
        }
        CHECK(count_occurrences(output, "M624 ") == 12);
    }
    SECTION("without layer_gcode the open block is closed before the commands") {
        SecondLayerRun run;
        run.variables = "enable_power_loss_recovery = enable\nscan_first_layer = 1";
        run.layer_script = false;
        const std::vector<std::string> lines = split_lines(run_bambu_second_layer(run));
        const size_t scan = find_line(lines, k_scan_line);
        REQUIRE(scan < lines.size());
        CHECK_FALSE(inside_block(lines, scan));
        CHECK_FALSE(inside_block(lines, find_line(lines, "M1003 S1")));
        // The block of the last object of the first layer ends before the commands.
        CHECK(find_line(lines, "M625", find_line(lines, "M624 AgAAAAAAAAA=")) < find_line(lines, "M1003 S1"));
    }
    SECTION("without labels the commands are the same") {
        SecondLayerRun run;
        run.variables = "scan_first_layer = 1";
        run.labels = false;
        const std::vector<std::string> lines = split_lines(run_bambu_second_layer(run));
        CHECK(std::count(lines.begin(), lines.end(), k_scan_line) == 1);
    }
}

TEST_CASE("Bambu firmware reads printer_custom_variables like the placeholder parser",
          "[plugins][gcode][firmware][bambu][scan]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    struct Case { const char *text; bool scan; };
    const Case cases[] = {
        {"scan_first_layer=1", true},
        {"  scan_first_layer \t=   true  ", true},
        {"scan_first_layer = \"true\"\r\nother = 3\r\n", true},
        {"first = 2\nscan_first_layer = 1\nlast = 5", true},
        {"scan_first_layer = 0\nscan_first_layer = 1", true},
        {"scan_first_layer = 1\nscan_first_layer = 0", false},
        {"scan_first_layer = True", true},
        {"scan_first_layer = 1.0", false},
        {"scan_first_layer\v=\f1\v\r\n", true},
        {"scan_first_layer == 1", false},
        {"scan_first_layer", false},
        {"scan_first_layer = yes", false},
        {"scan_first_layer = 2", false},
        {"scan_first_layer = ", false},
    };
    for (const Case &entry : cases) {
        INFO(entry.text);
        SecondLayerRun run;
        run.variables = entry.text;
        const std::string output = run_bambu_second_layer(run);
        CHECK((output.find(k_scan_line) != std::string::npos) == entry.scan);
    }
    SecondLayerRun quoted;
    quoted.variables = "enable_power_loss_recovery = \"enable\"";
    CHECK(run_bambu_second_layer(quoted).find("M1003 S1\n") != std::string::npos);
}
