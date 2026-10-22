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
#include "libslic3r/Plugins/GCode/Reader/RepRapGCodeReaderState.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
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

Representative direct calls (branches are not a timeline):

    TEST_CASE
        |-- Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized()
        |-- configure_standard_firmware()
        |-- print.config()
        |-- make_firmware_path()
        |-- append_path_extrusion()
        `-- export_with_firmware()
*/

namespace {
using namespace Slic3r;
using namespace Slic3r::Printing;
using namespace Slic3r::Test::GCode;

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
        "gcode.firmware.marlin1",
        "gcode.firmware.marlin2",
        "gcode.firmware.prusa",
        "gcode.firmware.reprap",
        "gcode.firmware.sprinter",
        "gcode.firmware.klipper"
    };
    std::set<std::string> registered_firmwares;
    for (Plugin *plugin : orchestrator.get_active_plugins_for_step(GCODE_FIRMWARE))
        registered_firmwares.insert(plugin->get_id());
    CHECK(registered_firmwares == expected_firmwares);
    CHECK(orchestrator.get_plugin("gcode.firmware.default") == nullptr);

    Plugin *selected = Steps::selected_or_active_plugin_for_step(
        orchestrator, GCODE_FIRMWARE, &print.full_print_config());
    REQUIRE(selected != nullptr);
    CHECK(selected->get_id() == "gcode.firmware.marlin2");

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

    const ConfigOptionDef *definition = PrintConfigDef::instance().get("gcode_firmware_plugin");
    REQUIRE(definition != nullptr);
    CHECK(definition->option_preset_type == RAW_PRESET_TYPE_FFF_PRINTER);
    CHECK(definition->invalidates_step == STEP_GCODE);

    const std::string merged = orchestrator.merged_ui_layout(
        "printer_fff.ui",
        "page:General:printer\n"
        "group:Firmware\n"
        "\tsetting:gcode_flavor\n");
    const size_t firmware_pos = merged.find("setting:gcode_firmware_plugin");
    const size_t flavor_pos = merged.find("setting:gcode_flavor");
    REQUIRE(firmware_pos != std::string::npos);
    REQUIRE(flavor_pos != std::string::npos);
    CHECK(firmware_pos < flavor_pos);
    CHECK(merged.find("setting:step_gcode_plugin") == std::string::npos);
    CHECK(std::string(slic3r_api::GCodeGeneration::Firmware::printer_ui_fragment())
              .find("gcode_firmware_plugin") != std::string::npos);

    STATIC_REQUIRE(std::is_base_of<
        slic3r_api::GCodeGeneration::Firmware::Marlin2GCodeFirmwareSession,
        slic3r_api::GCodeGeneration::Firmware::PrusaGCodeFirmwareSession>::value);

    // Printer presets that still name the removed generic provider resolve to
    // its concrete equivalent before the exclusive service is selected.
    Print migrated_print;
    DynamicPrintConfig &migrated_config =
        const_cast<DynamicPrintConfig &>(migrated_print.full_print_config());
    t_config_option_key migrated_key = "gcode_firmware_plugin";
    std::string migrated_value = "gcode.firmware.default";
    PrintConfigDef::handle_legacy_pair(migrated_key, migrated_value);
    CHECK(migrated_value == "gcode.firmware.marlin2");
    migrated_config.set_deserialize(migrated_key, migrated_value);
    Plugin *migrated_selection = Steps::selected_or_active_plugin_for_step(
        orchestrator, GCODE_FIRMWARE, &migrated_print.full_print_config());
    REQUIRE(migrated_selection != nullptr);
    CHECK(migrated_selection->get_id() == "gcode.firmware.marlin2");
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
    };
    const DialectEnvelopeExpectation expectations[] = {
        {
            "gcode.firmware.marlin1",
            "M201 X1000 Y1100 Z120 E1300 ; sets maximum accelerations, mm/sec^2\n"
            "M203 X200 Y210 Z12 E25 ; sets maximum feedrates, mm/sec\n"
            "M204 S500 T800 ; sets print and retract acceleration, mm/sec^2\n"
            "M205 X8.00 Y9.00 Z0.40 E2.50 ; sets the jerk limits, mm/sec\n"
            "M205 S3 T4 ; sets the minimum extruding and travel feed rate, mm/sec\n"
        },
        {
            "gcode.firmware.marlin2",
            "M201 X1000 Y1100 Z120 E1300 ; sets maximum accelerations, mm/sec^2\n"
            "M203 X200 Y210 Z12 E25 ; sets maximum feedrates, mm/sec\n"
            "M204 P500 R800 T900 ; sets print, retract and travel acceleration, mm/sec^2\n"
            "M205 X8.00 Y9.00 Z0.40 E2.50 ; sets the jerk limits, mm/sec\n"
            "M205 S3 T4 ; sets the minimum extruding and travel feed rate, mm/sec\n"
        },
        {
            "gcode.firmware.prusa",
            "M201 X1000 Y1100 Z120 E1300 ; sets maximum accelerations, mm/sec^2\n"
            "M203 X200 Y210 Z12 E25 ; sets maximum feedrates, mm/sec\n"
            "M204 P500 R800 T900 ; sets print, retract and travel acceleration, mm/sec^2\n"
            "M205 X8.00 Y9.00 Z0.40 E2.50 ; sets the jerk limits, mm/sec\n"
            "M205 S3 T4 ; sets the minimum extruding and travel feed rate, mm/sec\n"
        },
        {
            "gcode.firmware.reprap",
            "M201 X1000 Y1100 Z120 E1300 ; sets maximum accelerations, mm/sec^2\n"
            "M203 X12000 Y12600 Z720 E1500 I180 ; sets maximum feedrates, mm/min\n"
            "M204 P500 T900 ; sets print and travel acceleration, mm/sec^2\n"
            "M566 X480.00 Y540.00 Z24.00 E150.00 ; sets the jerk limits, mm/min\n"
        },
        {
            "gcode.firmware.sprinter",
            "M201 X1000 Y1100 Z120 E1300 ; sets maximum accelerations, mm/sec^2\n"
            "M203 X12000 Y12600 Z720 E1500 ; sets maximum feedrates, mm/min\n"
            "M204 P500 T900 ; sets print and travel acceleration, mm/sec^2\n"
        },
        {
            "gcode.firmware.klipper",
            "M204 P500 T900 ; sets the initial shared acceleration, mm/sec^2\n"
        }
    };

    for (const DialectEnvelopeExpectation &expectation : expectations) {
        INFO(expectation.id);
        Print print;
        configure_standard_firmware(print);
        configure_machine_envelope(print);
        print.mutable_printing_plan();
        CHECK(export_with_firmware(print, expectation.id) ==
              std::string("G21\nG90\nM82\n") + expectation.output);
    }

    // The normal default keeps machine limits available for estimates without
    // inserting firmware commands into the file.
    Print disabled_print;
    configure_standard_firmware(disabled_print);
    disabled_print.mutable_printing_plan();
    CHECK(export_with_firmware(disabled_print, "gcode.firmware.marlin2") ==
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
        {"gcode.firmware.marlin1", "T0\n", "M104 S200 T0\n", "M109 S200 T0\n",
         "M106 S128\n", "M900 K0.04\n", "M204 S500\n"},
        {"gcode.firmware.marlin2", "T0\n", "M104 S200 T0\n", "M109 S200 T0\n",
         "M106 S128\n", "M900 K0.04\n", "M204 P500\n"},
        {"gcode.firmware.prusa", "T0\n", "M104 S200 T0\n", "M109 S200 T0\n",
         "M106 S128\n", "M900 K0.04\n", "M204 P500\n"},
        {"gcode.firmware.reprap", "T0\n", "G10 P0 S200\n", "M116 P0\n",
         "M106 P0 S0.5\n", "M572 D0 S0.04\n", "M204 P500\n"},
        {"gcode.firmware.sprinter", "T0\n", "M104 S200 T0\n", "M109 S200 T0\n",
         "M106 S128\n", "M572 D0 S0.04\n", "M204 P500\n"},
        {"gcode.firmware.klipper", "ACTIVATE_EXTRUDER EXTRUDER=extruder\n",
         "M104 S200\n", "M109 S200\n", "M106 S128\n",
         "SET_PRESSURE_ADVANCE ADVANCE=0.04 EXTRUDER=extruder\n", "M204 S500\n"}
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
        root->add_property(ExtrusionPropertySpeed(20.f, 500.f, 0.04f, 50.f));
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
    const char *firmware_id = GENERATE("gcode.firmware.marlin1", "gcode.firmware.marlin2",
        "gcode.firmware.prusa", "gcode.firmware.sprinter", "gcode.firmware.reprap", "gcode.firmware.klipper");
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
        root->add_property(ExtrusionPropertySpeed(20.f, 500.f, pressure, fan));
        root->add_property(ExtrusionPropertyHeater(
            C_EXTRUSION_HEATER_TOOL, EXTRUDER_ID_UNKNOWN, int16_t(temperature), false));
        append_path_extrusion(group, std::move(root));
    }
    const std::string output = export_with_firmware(print, firmware_id);
    INFO(output);
    const bool reprap = std::string(firmware_id) == "gcode.firmware.reprap";
    const bool klipper = std::string(firmware_id) == "gcode.firmware.klipper";
    const bool sprinter = std::string(firmware_id) == "gcode.firmware.sprinter";
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
    const char *firmware_id = GENERATE("gcode.firmware.marlin1", "gcode.firmware.marlin2",
        "gcode.firmware.prusa", "gcode.firmware.sprinter", "gcode.firmware.reprap", "gcode.firmware.klipper");
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
    for (const char *id : {"gcode.firmware.marlin1", "gcode.firmware.marlin2", "gcode.firmware.prusa",
                           "gcode.firmware.sprinter", "gcode.firmware.reprap", "gcode.firmware.klipper"}) {
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
        "gcode.firmware.marlin2",
        "gcode.firmware.prusa",
        "gcode.firmware.reprap"
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
        {"gcode.firmware.marlin1", "M204 S500\n", "M204 S900\n"},
        {"gcode.firmware.sprinter", "M204 P500\n", "M204 P900\n"},
        {"gcode.firmware.klipper", "M204 S500\n", "M204 S900\n"}
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
        {"gcode.firmware.marlin1", "M204 S500\n", "M204 S900\n"},
        {"gcode.firmware.sprinter", "M204 P500\n", "M204 P900\n"},
        {"gcode.firmware.klipper", "M204 S500\n", "M204 S900\n"}
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

        const std::string output = export_with_firmware(print, "gcode.firmware.marlin2");
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

        const std::string output = export_with_firmware(print, "gcode.firmware.marlin1");
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

    const std::string output = export_with_firmware(print, "gcode.firmware.klipper");
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
        "gcode.firmware.marlin1",
        "gcode.firmware.marlin2",
        "gcode.firmware.prusa",
        "gcode.firmware.reprap",
        "gcode.firmware.sprinter"
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

    const std::string output = export_with_firmware(print, "gcode.firmware.reprap");
    INFO(output);
    CHECK(count_occurrences(output, "G10 P0 S200\n") == 1);
    CHECK(count_occurrences(output, "G10 P1 S210\n") == 1);
    CHECK(count_occurrences(output, "M106 P0 S0.2\n") == 1);
    CHECK(count_occurrences(output, "M106 P1 S0.3\n") == 1);
}
