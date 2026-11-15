#include <catch2/catch.hpp>

/*
Build small owned printing plans and drive real firmware sessions through the
public plugin views. Controlled machine state and event trees isolate traversal,
extrusion accounting, scripts, cancellation labels and syntax-extension hooks.
Assertions inspect the returned command text and the state it is meant to encode.

The test owns each plan and source object. Borrowed session views are used only
while their raw instance remains alive. Shared helpers construct neutral fixtures;
the probes here expose only the state needed by the behavior being checked.

Tests run firmware sessions through public plugin views and inspect generated
G-code. Export helpers select the text exporter with a borrowed Print and a
separate configuration copy, then read its published file for assertions.

Default firmware-session tests verify neutral machine-state interpretation, extrusion traversal and extension hooks.
Shared plan construction and output helpers live in gcode_test_helpers so this file keeps only component-specific behavior.
Seam export probes materialize an exact entry, consume rotation permission and
verify the writer's first extrusion and marker. Fixed closed sequences and their
canonical copies keep their entry even when a child remains rotatable.

Representative direct calls (branches are not a timeline):

    TEST_CASE
        |-- firmware.setup()
        |-- firmware.write_event()
        |-- make_firmware_path()
        |-- encoded_temperature()
        |-- klipper.setup()
        `-- klipper.write_event()
*/

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

#include <boost/beast/core/detail/base64.hpp>
#include <boost/filesystem.hpp>

#include "plugin_test_helpers.hpp"
#include "gcode_export_test_helpers.hpp"
#include "gcode_test_helpers.hpp"
#include "layer_extrusion_edit_transition_test_helpers.hpp"
#include "perimeter_test_helpers.hpp"

#include "libslic3r/Api/host/ApiHostUtils.hpp"
#include "libslic3r/Api/host/GCodeReaderService.hpp"
#include "libslic3r/Api/host/GCodeScriptProcessor.hpp"
#include "libslic3r/Api/host/GCodeReaderService.hpp"
#include "libslic3r/Api/host/GCodeThumbnailRenderer.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeThumbnailViews.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeTextEnvelope.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_gcode.h"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/properties/ExtrusionProperties.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/DefaultGCodeFirmwareSession.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeFirmwareViews.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeScriptProcessorViews.hpp"
#include "libslic3r/Api/plugin/cpp/PrintHelpers.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/MachineEnvelope.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Config/ConfigSnapshotSerialization.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityVisitors.hpp"
#include "libslic3r/GCode.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/libslic3r.h"
#include "libslic3r/Plugins/Ordering/ObjectInstanceCatalog.hpp"
#include "libslic3r/Plugins/Ordering/ExtrusionTreeOrderingGeometry.hpp"
#include "libslic3r/GCode/Thumbnails.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/Geometry/ArcWelder.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/SLA/SLAPrintConfig.hpp"
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
#include "libslic3r/ExtrusionEntityCanonical.hpp"
#include "libslic3r/Steps/StepExtrusionEdition.hpp"
#include "libslic3r/Steps/StepExtrusionOrdering.hpp"
#include "libslic3r/Steps/StepGenerateGcode.hpp"
#include "libslic3r/Steps/StepLayerHeightGeneration.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"
#include "libslic3r/Steps/StepSlicing.hpp"
#include "test_data.hpp"



namespace {
using namespace Slic3r;
using namespace Slic3r::Printing;
using namespace Slic3r::Test::GCode;

// Read E words from generated movement lines into an owned vector; malformed numbers throw.
std::vector<double> firmware_move_e_values(const std::string &gcode);
// Create an owned support deposit at (1.25, 2.5) with explicit printable process values.
std::unique_ptr<ExtrusionEntity> firmware_deposit(double volume = 0.4);

// Build a single-point owned entity, storing volume before the tool multiplier;
// callers may mutate its properties to exercise validation, and allocation failures propagate.
std::unique_ptr<ExtrusionEntity> firmware_deposit(const double volume)
{
    std::unique_ptr<ExtrusionEntity> leaf(new ExtrusionEntity(false,
        ArcPolyline(Points{Point::new_scale(1.25, 2.5)})));
    leaf->add_property(ExtrusionAttributes(ExtrusionRole::SupportMaterial, ExtrusionFlow(0.08, 0.4f, 0.2f)));
    leaf->add_property(ExtrusionPropertySpeed(20.f, 500.f));
    leaf->add_property(ExtrusionPropertySpecialCommand(ExtrusionPropertySpecialCommand::Code::DEPOSIT, volume));
    return leaf;
}

// Borrow generated G-code, scan G1/G2/G3 lines and copy their displayed E numbers;
// stod reports malformed numeric output, and lines without E leave no entry.
std::vector<double> firmware_move_e_values(const std::string &gcode)
{
    std::vector<double> values;
    std::istringstream stream(gcode);
    std::string line;
    while (std::getline(stream, line)) {
        if (line.rfind("G1 ", 0) != 0 && line.rfind("G2 ", 0) != 0 && line.rfind("G3 ", 0) != 0)
            continue;
        const size_t e_pos = line.find(" E");
        if (e_pos != std::string::npos)
            values.push_back(std::stod(line.substr(e_pos + 2)));
    }
    return values;
}

class CustomFanFirmwareSession final :
    public slic3r_api::GCodeGeneration::DefaultGCodeFirmwareSession
{
protected:
    std::string encode_fan(extruder_index_t tool_id, double speed_percent) const override
    {
        return "CUSTOM_FAN T" + std::to_string(tool_id) +
               " S" + std::to_string(int32_t(std::lround(speed_percent))) + "\n";
    }
};

// Expose the owned Klipper tools for precision accounting without changing encoding.
class DepositKlipperFirmwareProbe final :
    public slic3r_api::GCodeGeneration::Firmware::KlipperGCodeFirmwareSession
{
public:
    using KlipperGCodeFirmwareSession::extruders;
};

// Exposes the protected syntax encoder so kind dispatch can be tested without
// involving extrusion traversal or filesystem publication.
class CustomGCodeFirmwareProbe final :
    public slic3r_api::GCodeGeneration::DefaultGCodeFirmwareSession
{
public:
    using DefaultGCodeFirmwareSession::encode_custom_gcode;
};

// Exposes the parent begin-print orchestration without adding any dialect
// behavior beyond a visible marker for the machine-envelope hook.
class MachineEnvelopeFirmwareSession final :
    public slic3r_api::GCodeGeneration::DefaultGCodeFirmwareSession
{
public:
    uint32_t encode_count() const { return m_encode_count; }
    bool initialized_during_encoding() const { return m_initialized_during_encoding; }
    const std::optional<slic3r_api::GCodeGeneration::MachineEnvelope> &encoded_envelope() const
    {
        return m_encoded_envelope;
    }
    bool formatter_initialized() const
    {
        (void)gcode_formatter();
        return true;
    }

protected:
    std::string encode_machine_envelope(
        const slic3r_api::GCodeGeneration::MachineEnvelope &envelope) const override
    {
        // Accessing the formatter proves that setup() completed before the
        // virtual dialect hook was entered.
        (void)gcode_formatter();
        m_initialized_during_encoding = true;
        m_encoded_envelope = envelope;
        ++m_encode_count;
        return "machine-envelope\n";
    }

private:
    mutable uint32_t m_encode_count = 0;
    mutable bool m_initialized_during_encoding = false;
    mutable std::optional<slic3r_api::GCodeGeneration::MachineEnvelope> m_encoded_envelope;
};

class PreviewInspectingFirmwareSession final :
    public slic3r_api::GCodeGeneration::DefaultGCodeFirmwareSession
{
public:
    std::string end_print() override
    {
        return printer().preview_enabled() ? "PREVIEW_ON\n" : "PREVIEW_OFF\n";
    }
};

class TraversalHookFirmwareSession final :
    public slic3r_api::GCodeGeneration::DefaultGCodeFirmwareSession
{
public:
    explicit TraversalHookFirmwareSession(std::vector<std::string> &calls) : m_calls(calls) {}

protected:
    std::string enter_extrusion_node(const slic3r_api::ExtrusionEntity &) override
    {
        m_calls.emplace_back("enter");
        return {};
    }

    std::string visit_extrusion_leaf(const slic3r_api::ExtrusionEntity &) override
    {
        m_calls.emplace_back("leaf");
        return {};
    }

    std::string leave_extrusion_node(const slic3r_api::ExtrusionEntity &) override
    {
        m_calls.emplace_back("leave");
        return {};
    }

private:
    std::vector<std::string> &m_calls;
};

/* Build a short printable leaf with explicit semantic marker properties. */
std::unique_ptr<ExtrusionPath> marker_test_path(double start_x,
                                                double end_x,
                                                ExtrusionRole role,
                                                float width,
                                                float height)
{
    ArcPolyline polyline(Points{
        Point(scale_i(start_x), scale_i(0.0)),
        Point(scale_i(end_x), scale_i(0.0))
    });
    std::unique_ptr<ExtrusionPath> path(new ExtrusionPath(
        polyline,
        ExtrusionAttributes(role, ExtrusionFlow(0.08, width, height)),
        nullptr,
        true));
    path->add_property(ExtrusionPropertySpeed(20.f, 500.f));
    return path;
}

/* Append one manually ordered layer and its first tool group to a test plan. */
PrintingLayerGroup &append_marker_test_layer(Print &print, coord_t print_z)
{
    PrintingPlan &plan = print.mutable_printing_plan();
    if (plan.groups.empty())
        plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.front().layers.back();
    layer.print_z = print_z;
    layer.tool_groups.emplace_back();
    layer.tool_groups.back().extruder_id = 0;
    return layer;
}

/* Read a G-code string line by line, copy layer, Z, maximum-height and printable marker lines into an owned vector, and leave the input unchanged; allocation failures propagate. */
std::vector<std::string> semantic_marker_lines(const std::string &gcode)
{
    std::vector<std::string> markers;
    std::istringstream stream(gcode);
    std::string line;
    while (std::getline(stream, line)) {
        if (line == ";LAYER_CHANGE" || line.rfind(";Z:", 0) == 0 ||
            line.rfind(";MAX_LAYER_HEIGHT:", 0) == 0 ||
            line.rfind(";HEIGHT:", 0) == 0 || line.rfind(";TYPE:", 0) == 0 ||
            line.rfind(";WIDTH:", 0) == 0)
            markers.push_back(line);
    }
    return markers;
}

/* Return the first generated printable move, excluding approach and travel moves. */
size_t first_printable_move_offset(const std::string &gcode)
{
    size_t begin = 0;
    while (begin < gcode.size()) {
        const size_t end = gcode.find('\n', begin);
        const size_t length = end == std::string::npos ? gcode.size() - begin : end - begin;
        const std::string line = gcode.substr(begin, length);
        if (line.rfind("G1 ", 0) == 0 && line.find(" E") != std::string::npos)
            return begin;
        if (end == std::string::npos)
            break;
        begin = end + 1;
    }
    return std::string::npos;
}

/* Build one E-only retraction transition without introducing wipe markers. */
std::unique_ptr<ExtrusionEntity> marker_test_retraction(bool retract)
{
    std::unique_ptr<ExtrusionNop> event(new ExtrusionNop());
    slic3r_api::MutableExtrusionEntity view(
        reinterpret_cast<extrusion_entity_handle *>(event.get()));
    view.get_or_add(slic3r_api::EPropertyAttributes::key)
        .extrusion_role(retract ? RAW_EXTRUSION_ROLE_RETRACT : RAW_EXTRUSION_ROLE_UNRETRACT)
        .mm3_per_mm(0.0);
    if (retract)
        view.get_or_add(slic3r_api::EPropertyExtrusionAxis::key).retract_to(0.2);
    else
        view.get_or_add(slic3r_api::EPropertyExtrusionAxis::key).unretract(0.0);
    return std::unique_ptr<ExtrusionEntity>(std::move(event));
}

/* Export the same semantic fixture under both human-comment settings. */
std::string export_marker_comment_fixture(bool gcode_comments)
{
    Print print;
    configure_standard_firmware(print);
    const_cast<PrintConfig &>(print.config()).gcode_comments.value = gcode_comments;
    PrintingLayerGroup &layer = append_marker_test_layer(print, scale_i(0.2));
    append_path_extrusion(
        layer.tool_groups.front(),
        marker_test_path(0.0, 1.0, ExtrusionRole::Perimeter, 0.4f, 0.2f));
    return export_with_firmware(print, "gcode.firmware.marlin2");
}

std::unique_ptr<ExtrusionPath> marker_test_wipe(double start_x, double end_x)
{
    std::unique_ptr<ExtrusionPath> wipe = make_firmware_travel(
        ArcPolyline(Points{
            Point(scale_i(start_x), scale_i(0.0)),
            Point(scale_i(end_x), scale_i(0.0))
        }),
        40.f,
        800.f);
    slic3r_api::MutableExtrusionEntity view(
        reinterpret_cast<extrusion_entity_handle *>(wipe.get()));
    view.get_mutable(slic3r_api::EPropertyAttributes::key)->extrusion_role(
        RAW_EXTRUSION_ROLE_TRAVEL | RAW_EXTRUSION_ROLE_WIPE);
    return wipe;
}

std::vector<std::string> wipe_marker_lines(const std::string &gcode)
{
    std::vector<std::string> markers;
    std::istringstream stream(gcode);
    std::string line;
    while (std::getline(stream, line)) {
        if (line == ";WIPE_START" || line == ";WIPE_END")
            markers.push_back(line);
    }
    return markers;
}

std::vector<std::string> machine_command_lines(const std::string &gcode)
{
    std::vector<std::string> commands;
    std::istringstream stream(gcode);
    std::string line;
    while (std::getline(stream, line)) {
        if (!line.empty() && line.front() != ';')
            commands.push_back(line);
    }
    return commands;
}

std::vector<std::string> object_protocol_lines(const std::string &gcode)
{
    std::vector<std::string> lines;
    std::istringstream stream(gcode);
    std::string line;
    while (std::getline(stream, line)) {
        if (line.rfind("M486 ", 0) == 0 ||
            line.rfind("; printing object ", 0) == 0 ||
            line.rfind("; stop printing object ", 0) == 0 ||
            line.rfind("; SLIC3R_OBJECT_INSTANCE_ID=", 0) == 0)
            lines.push_back(line);
    }
    return lines;
}

std::string export_wipe_marker_fixture(bool gcode_comments, bool trailing_printable)
{
    Print print;
    configure_standard_firmware(print);
    const_cast<PrintConfig &>(print.config()).gcode_comments.value = gcode_comments;
    PrintingLayerGroup &layer = append_marker_test_layer(print, scale_i(0.2));
    PrintingToolGroup &tool_group = layer.tool_groups.front();
    append_path_extrusion(tool_group, marker_test_wipe(0.0, 1.0));
    append_path_extrusion(tool_group, marker_test_wipe(1.0, 2.0));
    if (trailing_printable) {
        append_path_extrusion(
            tool_group,
            marker_test_path(2.0, 3.0, ExtrusionRole::Perimeter, 0.4f, 0.2f));
    }
    return export_with_firmware(print, "gcode.firmware.marlin2");
}

std::unique_ptr<ExtrusionEntity> marker_test_composed_arc_loop()
{
    const Point start = Point::new_scale(1.2346, 2.3456);
    const Point arc_start = Point::new_scale(6.0, 2.3456);
    const Point arc_end = Point::new_scale(6.0, 6.3456);
    ExtrusionEntity::Children children;
    children.push_back(make_firmware_path(
        ArcPolyline(Points{start, arc_start}),
        20.f, 500.f, -1.f, -1.f, -1.f, ExtrusionRole::ExternalPerimeter));

    Geometry::ArcWelder::Path arc;
    arc.emplace_back(arc_start);
    arc.emplace_back(
        arc_end, float(scale_i(2.0)), Geometry::ArcWelder::Orientation::CCW);
    children.push_back(make_firmware_path(
        ArcPolyline(arc),
        20.f, 500.f, -1.f, -1.f, -1.f, ExtrusionRole::ExternalPerimeter));
    children.push_back(make_firmware_path(
        ArcPolyline(Points{arc_end, start}),
        20.f, 500.f, -1.f, -1.f, -1.f, ExtrusionRole::ExternalPerimeter));
    std::unique_ptr<ExtrusionEntity> root(
        new ExtrusionEntity(std::move(children), false, false, true));
    root->get_or_add_property<slic3r_api::EPropertyPerimeter>().shell_count(0);
    return root;
}

std::vector<std::string> seam_marker_lines(const std::string &gcode)
{
    std::vector<std::string> markers;
    std::istringstream stream(gcode);
    std::string line;
    while (std::getline(stream, line)) {
        if (line.rfind(";SEAM:", 0) == 0)
            markers.push_back(line);
    }
    return markers;
}

std::string export_composed_seam_fixture(bool gcode_comments,
                                         bool trailing_printable)
{
    Print print;
    configure_standard_firmware(print);
    const_cast<PrintConfig &>(print.config()).gcode_comments.value = gcode_comments;
    PrintingLayerGroup &layer = append_marker_test_layer(print, scale_i(0.2));
    PrintingToolGroup &tool_group = layer.tool_groups.front();
    append_path_extrusion(tool_group, marker_test_composed_arc_loop());
    if (trailing_printable) {
        append_path_extrusion(
            tool_group,
            make_firmware_path(
                ArcPolyline(Points{
                    Point::new_scale(1.2346, 2.3456),
                    Point::new_scale(2.2346, 2.3456)
                }),
                20.f, 500.f, -1.f, -1.f, -1.f,
                ExtrusionRole::ExternalPerimeter));
    }
    return export_with_firmware(print, "gcode.firmware.marlin2");
}

/* Add the minimum plan hierarchy needed by STEP_GCODE validation tests. */
PrintingToolGroup &append_speed_validation_tool_group(Print &print,
                                                      coord_t print_z = scale_i(0.42),
                                                      extruder_index_t extruder_id = 3)
{
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.back().layers.emplace_back();
    PrintingLayerGroup &layer_group = plan.groups.back().layers.back();
    layer_group.print_z = print_z;
    layer_group.tool_groups.emplace_back();
    layer_group.tool_groups.back().extruder_id = extruder_id;
    return layer_group.tool_groups.back();
}

/* Build one two-point geometric leaf with an optional direct speed property. */
std::unique_ptr<ExtrusionEntity> speed_validation_path(
    std::optional<float> speed,
    double start_x = 1.25,
    double end_x = 3.75)
{
    std::unique_ptr<ExtrusionPath> path(new ExtrusionPath(
        ArcPolyline(Points{
            Point(scale_i(start_x), scale_i(2.5)),
            Point(scale_i(end_x), scale_i(2.5))
        }),
        ExtrusionAttributes(ExtrusionRole::Perimeter, ExtrusionFlow(0.08, 0.4f, 0.2f)),
        nullptr,
        true));
    if (speed)
        path->add_property(ExtrusionPropertySpeed(*speed));
    return path->clone_canonical();
}

/* Acquire one canonical validation root, retaining an explicit missing-root probe. */
PrintingExtrusion &append_speed_validation_extrusion(
    PrintingToolGroup &tool_group,
    std::unique_ptr<ExtrusionEntity> root,
    const LayerRegionIsland *region_island = nullptr,
    uint32_t printing_catalog_id = 0)
{
    PrintingExtrusion extrusion;
    extrusion.region_island = region_island;
    extrusion.sregion_island_role = ExtrusionRole::Perimeter;
    extrusion.root = root ? CanonicalExtrusion::acquire(*root) : nullptr;
    extrusion.printing_catalog_id = printing_catalog_id;
    tool_group.extrusions.push_back(std::move(extrusion));
    return tool_group.extrusions.back();
}

/* Add one geometric event below a scope's fixed before or after root. */
void append_speed_validation_event(PrintingScopeEvents &events,
                                   const bool before,
                                   std::optional<float> speed)
{
    const std::unique_ptr<ExtrusionEntity> event = speed_validation_path(speed);
    if (before)
        events.append_before(*event);
    else
        events.append_after(*event);
}

/* Add one E-only command whose firmware path still emits a movement line. */
void append_special_extrusion_event(PrintingScopeEvents &events,
                                    const bool before,
                                    std::optional<float> speed,
                                    const double extrusion_delta = 0.5)
{
    ExtrusionNop event;
    event.add_property(ExtrusionPropertySpecialCommand(
        ExtrusionPropertySpecialCommand::Code::EXTRUSION, extrusion_delta));
    if (speed)
        event.add_property(ExtrusionPropertySpeed(*speed));
    if (before)
        events.append_before(event);
    else
        events.append_after(event);
}

} // namespace

TEST_CASE("Direct empty fan events encode once and preserve physical state",
          "[plugins][gcode][firmware][fan-request]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"extruder_fan_offset", "10%"}, {"fan_printer_min_speed", "30"}});
    CustomFanFirmwareSession firmware;
    firmware.setup(slic3r_api::Config(ApiHost::to_config_handle(&config)));
    const auto write = [&](const ExtrusionEntity &entity) {
        return firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(&entity)));
    };
    const auto select = special_command_entity(ExtrusionPropertySpecialCommand::Code::TOOLCHANGE, 0.);
    write(*select);

    ExtrusionNop neutral;
    neutral.get_or_add_property<ExtrusionPropertyFan>();
    CHECK(write(neutral).empty());
    ExtrusionEntityCollection inherited(false, false);
    inherited.add_property(ExtrusionPropertyFan(70.f, FAN_REQUEST_MODE_OVERRIDE));
    inherited.append(ExtrusionNop(ExtrusionPropertyHeater(
        C_EXTRUSION_HEATER_TOOL, EXTRUDER_ID_UNKNOWN, 200, true)));
    const auto inherited_output = write(inherited);
    CHECK(inherited_output.find("M109") != std::string::npos);
    CHECK(inherited_output.find("CUSTOM_FAN") == std::string::npos);

    ExtrusionNop stop;
    stop.add_property(ExtrusionPropertyFan(0.f, FAN_REQUEST_MODE_OVERRIDE));
    CHECK(write(stop) == "CUSTOM_FAN T0 S0\n");
    CHECK(write(stop).empty());
    // Neither exiting a parent nor the next request-less movement restores its
    // old lexical request. A fan-only event emits no motion/PA/heater commands.
    ExtrusionEntityCollection parent(false, false);
    parent.add_property(ExtrusionPropertyFan(80.f, FAN_REQUEST_MODE_AUTOMATIC));
    parent.append(ExtrusionNop(ExtrusionPropertyFan(20.f, FAN_REQUEST_MODE_OVERRIDE)));
    CHECK(write(parent) == "CUSTOM_FAN T0 S40\n"); // the minimum also applies to Override
    const auto travel = make_firmware_travel(
        ArcPolyline(Points{Point::new_scale(0., 0.), Point::new_scale(1., 0.)}), 20.f, 500.f);
    CHECK(write(*travel).find("CUSTOM_FAN") == std::string::npos);
    CHECK(write(stop) == "CUSTOM_FAN T0 S0\n");

    ExtrusionNop raw;
    auto &request = raw.get_or_add_property<ExtrusionPropertyFan>();
    request.percent = std::numeric_limits<float>::quiet_NaN();
    CHECK(write(raw).empty()); // Unspecified ignores even noncanonical bytes.
    request.mode = FAN_REQUEST_MODE_OVERRIDE;
    CHECK_THROWS_AS(write(raw), std::invalid_argument);
    request.percent = 101.f;
    CHECK_THROWS_AS(write(raw), std::invalid_argument);
    request.percent = 25.f;
    request.mode = orchestrator_register_runtime_id(
        reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()),
        FAN_REQUEST_MODE_ID_TYPE, "tests.firmware.plugin_mode");
    const fan_request_mode custom = request.mode;
    CHECK(write(raw) == "CUSTOM_FAN T0 S40\n"); // custom mode reaches firmware, calibration once
    CHECK(request.mode == custom); // no conversion to Automatic
    // Registry errors are rejected by STEP_GCODE, not by this standalone
    // encoder, which needs no orchestrator or reader service for fan events.
}

TEST_CASE("STEP_GCODE validates raw fan pairs without closing the runtime mode domain",
          "[plugins][gcode][validation][fan-request]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    configure_standard_firmware(print);
    auto &tool = append_speed_validation_tool_group(print, scale_i(0.2), 0);
    ExtrusionEntityUPtr root = std::make_unique<ExtrusionEntity>(false);
    auto &fan = root->get_or_add_property<ExtrusionPropertyFan>();
    // This fan-only event belongs to no model instance in the compact plan.
    append_speed_validation_extrusion(tool, std::move(root), nullptr, UINT32_MAX);
    std::string error;
    CHECK(Steps::StepGenerateGcode::validate_pre(Orchestrator::instance(), print, &error));
    fan.set(0.f, FAN_REQUEST_MODE_OVERRIDE);
    CHECK(Steps::StepGenerateGcode::validate_pre(Orchestrator::instance(), print, &error));
    const auto custom = orchestrator_register_runtime_id(
        reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()),
        FAN_REQUEST_MODE_ID_TYPE, "tests.validation.custom");
    fan.set(35.f, custom);
    CHECK(Steps::StepGenerateGcode::validate_pre(Orchestrator::instance(), print, &error));
    // Standard firmware encodes plugin-owned percentages without rewriting
    // their mode; policy identity has no effect on firmware command syntax.
    const std::string output = export_with_firmware(print, "gcode.firmware.marlin2");
    CHECK(output.find("M106 S89\n") != std::string::npos);
    CHECK(fan.mode == custom);
    // Forged or unregistered modes and malformed active C writes cannot export.
    fan.mode = 42;
    CHECK_FALSE(Steps::StepGenerateGcode::validate_pre(Orchestrator::instance(), print, &error));
    CHECK(error.find("EPropertyFan") != std::string::npos);
    CHECK_THROWS_AS(export_with_firmware(print, "gcode.firmware.marlin2"), RuntimeError);
    fan.mode = ORCHESTRATOR_RUNTIME_ID_INVALID;
    CHECK_FALSE(Steps::StepGenerateGcode::validate_pre(Orchestrator::instance(), print, &error));
    fan.mode = FAN_REQUEST_MODE_AUTOMATIC;
    for (float invalid : {-1.f, 101.f, std::numeric_limits<float>::infinity(),
                          std::numeric_limits<float>::quiet_NaN()}) {
        fan.percent = invalid;
        CHECK_FALSE(Steps::StepGenerateGcode::validate_pre(Orchestrator::instance(), print, &error));
    }
    fan.mode = FAN_REQUEST_MODE_UNSPECIFIED;
    CHECK(Steps::StepGenerateGcode::validate_pre(Orchestrator::instance(), print, &error));
    fan.clear();
    CHECK(Steps::StepGenerateGcode::validate_pre(Orchestrator::instance(), print, &error));
}

// A malformed owned child is diagnosed before any firmware output or unsafe traversal.
TEST_CASE("STEP_GCODE diagnoses null owned children without repairing the tree",
          "[plugins][gcode][validation][canonical-ingress]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    PrintingToolGroup &tool = append_speed_validation_tool_group(print);
    ExtrusionEntityUPtr root = std::make_unique<ExtrusionEntity>(ExtrusionEntity::Children(), false, false, false);
    root->append_child(speed_validation_path(22.f));
    root->children().emplace_back(nullptr);
    PrintingExtrusion &extrusion = append_speed_validation_extrusion(tool, std::move(root));
    const ExtrusionEntity *identity = extrusion.root.get();
    std::string error;
    CHECK_FALSE(Steps::StepGenerateGcode::validate_pre(Orchestrator::instance(), print, &error));
    CHECK(error.find("extrusions[0].root.children[1]") != std::string::npos);
    CHECK(error.find("null owned extrusion child") != std::string::npos);
    CHECK(extrusion.root.get() == identity);
    REQUIRE(extrusion.root->child_count() == 2);
    CHECK(extrusion.root->children()[1] == nullptr);
}

TEST_CASE("STEP_GCODE pre-validation resolves inherited geometric speeds",
          "[plugins][gcode][validation][speed]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    SECTION("a local positive speed is accepted") {
        Print print;
        PrintingToolGroup &tool_group = append_speed_validation_tool_group(print);
        append_speed_validation_extrusion(tool_group, speed_validation_path(22.f));

        std::string error = "stale";
        CHECK(Steps::StepGenerateGcode::validate_pre(Orchestrator::instance(), print, &error));
        CHECK(error.empty());
    }

    SECTION("a child inherits a positive parent speed") {
        Print print;
        PrintingToolGroup &tool_group = append_speed_validation_tool_group(print);
        ExtrusionEntity::Children children;
        children.push_back(speed_validation_path(std::nullopt));
        std::unique_ptr<ExtrusionEntity> root(new ExtrusionEntity(
            std::move(children), false, false, true));
        root->add_property(ExtrusionPropertySpeed(24.f));
        append_speed_validation_extrusion(tool_group, std::move(root));

        CHECK(Steps::StepGenerateGcode::validate_pre(Orchestrator::instance(), print));
    }

    SECTION("a non-positive child declaration keeps the inherited parent speed") {
        Print print;
        PrintingToolGroup &tool_group = append_speed_validation_tool_group(print);
        ExtrusionEntity::Children children;
        children.push_back(speed_validation_path(0.f));
        children.push_back(speed_validation_path(-1.f));
        std::unique_ptr<ExtrusionEntity> root(new ExtrusionEntity(
            std::move(children), false, false, true));
        root->add_property(ExtrusionPropertySpeed(26.f));
        append_speed_validation_extrusion(tool_group, std::move(root));

        CHECK(Steps::StepGenerateGcode::validate_pre(Orchestrator::instance(), print));
    }

    SECTION("an event leaf without geometry does not require speed") {
        Print print;
        PrintingToolGroup &tool_group = append_speed_validation_tool_group(print);
        append_speed_validation_extrusion(
            tool_group, std::unique_ptr<ExtrusionEntity>(new ExtrusionNop()));

        CHECK(Steps::StepGenerateGcode::validate_pre(Orchestrator::instance(), print));
    }
}

TEST_CASE("STEP_GCODE pre-validation reports missing and malformed speeds",
          "[plugins][gcode][validation][speed]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    SECTION("a PrintingPlan is required") {
        Print print;
        std::string error = "stale";
        CHECK_FALSE(Steps::StepGenerateGcode::validate_pre(Orchestrator::instance(), print, &error));
        CHECK(error.find("Print has no plan") != std::string::npos);
    }

    SECTION("a geometric leaf without any declaration has a complete diagnostic") {
        Print print;
        PrintingToolGroup &tool_group = append_speed_validation_tool_group(print);
        ExtrusionEntity::Children inner_children;
        inner_children.push_back(speed_validation_path(std::nullopt));
        ExtrusionEntity::Children root_children;
        root_children.push_back(std::unique_ptr<ExtrusionEntity>(new ExtrusionEntity(
            std::move(inner_children), false, false, true)));
        append_speed_validation_extrusion(tool_group, std::unique_ptr<ExtrusionEntity>(
            new ExtrusionEntity(std::move(root_children), false, false, true)));

        std::string error;
        CHECK_FALSE(Steps::StepGenerateGcode::validate_pre(Orchestrator::instance(), print, &error));
        CHECK(error.find("1 geometric extrusion leaf has no valid effective speed") != std::string::npos);
        CHECK(error.find(
            "groups[0].layers[0].tool_groups[0].extrusions[0].root.children[0].children[0]") !=
            std::string::npos);
        CHECK(error.find("print_z=0.42 mm") != std::string::npos);
        CHECK(error.find("extruder_id=3") != std::string::npos);
        CHECK(error.find("source_role=IPeri") != std::string::npos);
        CHECK(error.find("region_island=unavailable") != std::string::npos);
        CHECK(error.find("printing_catalog_id=0") != std::string::npos);
        CHECK(error.find("type=path") != std::string::npos);
        CHECK(error.find("points=2, segments=1") != std::string::npos);
        CHECK(error.find("length=2.5 mm") != std::string::npos);
        CHECK(error.find("start=(1.25, 2.5)") != std::string::npos);
        CHECK(error.find("end=(3.75, 2.5)") != std::string::npos);
        CHECK(error.find("contains_arcs=no") != std::string::npos);
        CHECK(error.find("Speed declarations: none on the leaf or its parents") != std::string::npos);
        CHECK(error.find("neither this geometric leaf nor any parent declares a speed") != std::string::npos);
    }

    SECTION("inherit sentinels without a positive ancestor are listed") {
        Print print;
        PrintingToolGroup &tool_group = append_speed_validation_tool_group(print);
        ExtrusionEntity::Children children;
        children.push_back(speed_validation_path(0.f));
        std::unique_ptr<ExtrusionEntity> root(new ExtrusionEntity(
            std::move(children), false, false, true));
        root->add_property(ExtrusionPropertySpeed(-1.f));
        append_speed_validation_extrusion(tool_group, std::move(root));

        std::string error;
        CHECK_FALSE(Steps::StepGenerateGcode::validate_pre(Orchestrator::instance(), print, &error));
        CHECK(error.find(".root: speed_mm_per_s=-1 (inherit)") != std::string::npos);
        CHECK(error.find(".root.children[0]: speed_mm_per_s=0 (inherit)") != std::string::npos);
        CHECK(error.find("every speed declaration on this path requests inheritance") != std::string::npos);
    }

    SECTION("non-finite declarations remain errors despite a valid ancestor") {
        const std::vector<std::pair<float, const char *>> invalid_values{
            {std::numeric_limits<float>::quiet_NaN(), "NaN"},
            {std::numeric_limits<float>::infinity(), "+infinity"},
            {-std::numeric_limits<float>::infinity(), "-infinity"}
        };
        for (const std::pair<float, const char *> &invalid : invalid_values) {
            CAPTURE(invalid.second);
            Print print;
            PrintingToolGroup &tool_group = append_speed_validation_tool_group(print);
            ExtrusionEntity::Children children;
            children.push_back(speed_validation_path(invalid.first));
            std::unique_ptr<ExtrusionEntity> root(new ExtrusionEntity(
                std::move(children), false, false, true));
            root->add_property(ExtrusionPropertySpeed(28.f));
            append_speed_validation_extrusion(tool_group, std::move(root));

            std::string error;
            CHECK_FALSE(Steps::StepGenerateGcode::validate_pre(Orchestrator::instance(), print, &error));
            CHECK(error.find(std::string("speed_mm_per_s=") + invalid.second) != std::string::npos);
            CHECK(error.find("Every explicit speed must be finite") != std::string::npos);
        }
    }

    SECTION("a missing extrusion root is rejected at its plan location") {
        Print print;
        PrintingToolGroup &tool_group = append_speed_validation_tool_group(print);
        append_speed_validation_extrusion(tool_group, nullptr);

        std::string error;
        CHECK_FALSE(Steps::StepGenerateGcode::validate_pre(Orchestrator::instance(), print, &error));
        CHECK(error.find("extrusions[0].root") != std::string::npos);
        CHECK(error.find("print_z=0.42 mm") != std::string::npos);
        CHECK(error.find("extruder_id=3") != std::string::npos);
        CHECK(error.find("Entity: unavailable") != std::string::npos);
        CHECK(error.find("Speed declarations: unavailable") != std::string::npos);
        CHECK(error.find("has no root extrusion tree") != std::string::npos);
    }
}

TEST_CASE("STEP_GCODE pre-validation rejects unsupported printable marker roles",
          "[plugins][gcode][validation][role][markers][m4-r1]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    PrintingToolGroup &tool_group = append_speed_validation_tool_group(print);
    std::unique_ptr<ExtrusionEntity> path = speed_validation_path(22.f);
    const raw_extrusion_role unsupported_role =
        RAW_EXTRUSION_ROLE_PERIMETER | RAW_EXTRUSION_ROLE_INFILL;
    slic3r_api::MutableExtrusionEntity path_view(
        reinterpret_cast<extrusion_entity_handle *>(path.get()));
    path_view.get_mutable(slic3r_api::EPropertyAttributes::key)
        ->extrusion_role(unsupported_role);
    append_speed_validation_extrusion(tool_group, std::move(path));

    std::string error;
    CHECK_FALSE(Steps::StepGenerateGcode::validate_pre(Orchestrator::instance(), print, &error));
    CHECK(error.find("unsupported effective extrusion role raw=3") !=
          std::string::npos);
    CHECK(error.find(
        "groups[0].layers[0].tool_groups[0].extrusions[0].root") !=
        std::string::npos);
}

TEST_CASE("STEP_GCODE pre-validation covers every before and after event scope",
          "[plugins][gcode][validation][speed][events]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    PrintingToolGroup &tool_group = append_speed_validation_tool_group(print);
    PrintingPlan &plan = print.mutable_printing_plan();
    PrintingGroup &group = plan.groups.front();
    PrintingLayerGroup &layer_group = group.layers.front();

    append_speed_validation_event(plan.events, true, std::nullopt);
    append_speed_validation_event(plan.events, false, std::nullopt);
    append_speed_validation_event(group.events, true, std::nullopt);
    append_speed_validation_event(group.events, false, std::nullopt);
    append_speed_validation_event(layer_group.events, true, std::nullopt);
    append_speed_validation_event(layer_group.events, false, std::nullopt);
    append_speed_validation_event(tool_group.events, true, std::nullopt);
    append_speed_validation_event(tool_group.events, false, std::nullopt);

    std::string error;
    CHECK_FALSE(Steps::StepGenerateGcode::validate_pre(Orchestrator::instance(), print, &error));
    CHECK(error.find("8 movement-producing before/after event entities are invalid") != std::string::npos);
    CHECK(error.find("plan.events.before.children[0]") != std::string::npos);
    CHECK(error.find("plan.events.after.children[0]") != std::string::npos);
    CHECK(error.find("groups[0].events.before.children[0]") != std::string::npos);
    CHECK(error.find("groups[0].events.after.children[0]") != std::string::npos);
    CHECK(error.find("groups[0].layers[0].events.before.children[0]") != std::string::npos);
    CHECK(error.find("groups[0].layers[0].events.after.children[0]") != std::string::npos);
    CHECK(error.find(
        "groups[0].layers[0].tool_groups[0].events.before.children[0]") !=
        std::string::npos);
    CHECK(error.find(
        "groups[0].layers[0].tool_groups[0].events.after.children[0]") !=
        std::string::npos);
    CHECK(error.find("source_role=unavailable (scope event)") != std::string::npos);
    CHECK(error.find("Source context: unavailable for a before/after scope event") !=
          std::string::npos);
}

TEST_CASE("STEP_GCODE pre-validation recognizes event movement kinds",
          "[plugins][gcode][validation][speed][events]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    SECTION("geometric and E-only events accept their own speed") {
        Print print;
        PrintingToolGroup &tool_group = append_speed_validation_tool_group(print, scale_i(0.42), 0);
        append_speed_validation_event(tool_group.events, true, 18.f);
        append_special_extrusion_event(tool_group.events, false, 12.f);

        CHECK(Steps::StepGenerateGcode::validate_pre(Orchestrator::instance(), print));
    }

    SECTION("event geometry inherits speed inside its own event root") {
        Print print;
        PrintingToolGroup &tool_group = append_speed_validation_tool_group(print);
        ExtrusionEntity::Children children;
        children.push_back(speed_validation_path(std::nullopt));
        ExtrusionEntity event(std::move(children), false, false, true);
        event.add_property(ExtrusionPropertySpeed(19.f));
        tool_group.events.append_before(event);

        CHECK(Steps::StepGenerateGcode::validate_pre(Orchestrator::instance(), print));
    }

    SECTION("an E-only special extrusion without speed is diagnosed") {
        Print print;
        PrintingToolGroup &tool_group = append_speed_validation_tool_group(print, scale_i(0.42), 0);
        append_special_extrusion_event(tool_group.events, false, std::nullopt);

        std::string error;
        CHECK_FALSE(Steps::StepGenerateGcode::validate_pre(Orchestrator::instance(), print, &error));
        CHECK(error.find("1 movement-producing before/after event entity is invalid") !=
              std::string::npos);
        CHECK(error.find("movement=E-only special extrusion") != std::string::npos);
        CHECK(error.find("neither this E-only event nor any parent declares a speed") !=
              std::string::npos);
    }

    SECTION("a non-movement scope event needs no speed") {
        Print print;
        PrintingToolGroup &tool_group = append_speed_validation_tool_group(print);
        tool_group.events.append_before(ExtrusionNop());

        CHECK(Steps::StepGenerateGcode::validate_pre(Orchestrator::instance(), print));
    }

    SECTION("an E-only command in a PrintingExtrusion is also rejected early") {
        Print print;
        PrintingToolGroup &tool_group = append_speed_validation_tool_group(print, scale_i(0.42), 0);
        std::unique_ptr<ExtrusionNop> event(new ExtrusionNop());
        event->add_property(ExtrusionPropertySpecialCommand(
            ExtrusionPropertySpecialCommand::Code::EXTRUSION, -15.0));
        append_speed_validation_extrusion(tool_group, std::move(event));

        std::string error;
        CHECK_FALSE(Steps::StepGenerateGcode::validate_pre(Orchestrator::instance(), print, &error));
        CHECK(error.find("1 E-only special extrusion event is invalid") != std::string::npos);
        CHECK(error.find("extrusions[0].root") != std::string::npos);
    }
}

TEST_CASE("STEP_GCODE pre-validation visits E-only commands in every event scope",
          "[plugins][gcode][validation][speed][events][e-only]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    PrintingToolGroup &tool_group = append_speed_validation_tool_group(
        print, scale_i(0.42), 0);
    PrintingPlan &plan = print.mutable_printing_plan();
    PrintingGroup &group = plan.groups.front();
    PrintingLayerGroup &layer_group = group.layers.front();

    // The plan-level before scope runs before begin_tool_group(), so establish
    // its tool with the same semantic command understood by the firmware.
    ExtrusionNop toolchange;
    toolchange.add_property(ExtrusionPropertySpecialCommand(
        ExtrusionPropertySpecialCommand::Code::TOOLCHANGE, 0.0));
    plan.events.append_before(toolchange);
    append_special_extrusion_event(plan.events, true, 12.f);
    append_special_extrusion_event(group.events, true, 12.f);
    append_special_extrusion_event(layer_group.events, true, 12.f);
    append_special_extrusion_event(tool_group.events, true, 12.f);
    append_special_extrusion_event(tool_group.events, false, 12.f);
    append_special_extrusion_event(layer_group.events, false, 12.f);
    append_special_extrusion_event(group.events, false, 12.f);
    append_special_extrusion_event(plan.events, false, 12.f);

    CHECK(Steps::StepGenerateGcode::validate_pre(Orchestrator::instance(), print));
}

TEST_CASE("STEP_GCODE pre-validation checks explicit E-only extrusion inputs",
          "[plugins][gcode][validation][speed][events][e-only]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    SECTION("a zero E delta needs no speed") {
        Print print;
        PrintingToolGroup &tool_group = append_speed_validation_tool_group(
            print, scale_i(0.42), 0);
        append_special_extrusion_event(tool_group.events, true, std::nullopt, 0.0);

        CHECK(Steps::StepGenerateGcode::validate_pre(Orchestrator::instance(), print));
    }

    SECTION("an E-only child inherits speed from its event parent") {
        Print print;
        PrintingToolGroup &tool_group = append_speed_validation_tool_group(
            print, scale_i(0.42), 0);
        ExtrusionEntity::Children children;
        std::unique_ptr<ExtrusionNop> extrusion(new ExtrusionNop());
        extrusion->add_property(ExtrusionPropertySpecialCommand(
            ExtrusionPropertySpecialCommand::Code::EXTRUSION, 0.5));
        children.push_back(std::move(extrusion));
        ExtrusionEntity event(std::move(children), false, false, true);
        event.add_property(ExtrusionPropertySpeed(17.f));
        tool_group.events.append_before(event);

        CHECK(Steps::StepGenerateGcode::validate_pre(Orchestrator::instance(), print));
    }

    SECTION("a non-finite speed remains invalid for a zero E delta") {
        Print print;
        PrintingToolGroup &tool_group = append_speed_validation_tool_group(
            print, scale_i(0.42), 0);
        append_special_extrusion_event(
            tool_group.events, true, std::numeric_limits<float>::quiet_NaN(), 0.0);

        std::string error;
        CHECK_FALSE(Steps::StepGenerateGcode::validate_pre(Orchestrator::instance(), print, &error));
        CHECK(error.find("a speed declaration on this path is NaN or infinite") !=
              std::string::npos);
    }

    SECTION("non-finite E deltas are rejected") {
        const std::vector<double> invalid_values{
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity()
        };
        for (const double invalid_value : invalid_values) {
            CAPTURE(invalid_value);
            Print print;
            PrintingToolGroup &tool_group = append_speed_validation_tool_group(
                print, scale_i(0.42), 0);
            append_special_extrusion_event(tool_group.events, true, 15.f, invalid_value);

            std::string error;
            CHECK_FALSE(Steps::StepGenerateGcode::validate_pre(Orchestrator::instance(), print, &error));
            CHECK(error.find("requested E delta is NaN or infinite") != std::string::npos);
        }
    }

    SECTION("an E-only command before the first tool reports the missing state") {
        Print print;
        PrintingPlan &plan = print.mutable_printing_plan();
        append_special_extrusion_event(plan.events, true, 15.f);

        std::string error;
        CHECK_FALSE(Steps::StepGenerateGcode::validate_pre(Orchestrator::instance(), print, &error));
        CHECK(error.find("plan.events.before.children[0]") != std::string::npos);
        CHECK(error.find("Active tool: unavailable") != std::string::npos);
        CHECK(error.find("no tool has been selected yet") != std::string::npos);
    }

    SECTION("a tool group selects the tool used by its events") {
        Print print;
        PrintingToolGroup &tool_group = append_speed_validation_tool_group(
            print, scale_i(0.42), 0);
        append_special_extrusion_event(tool_group.events, true, 15.f);

        CHECK(Steps::StepGenerateGcode::validate_pre(Orchestrator::instance(), print));
    }

    SECTION("an out-of-range tool is diagnosed with its source") {
        Print print;
        PrintingToolGroup &tool_group = append_speed_validation_tool_group(
            print, scale_i(0.42), 3);
        append_special_extrusion_event(tool_group.events, true, 15.f);

        std::string error;
        CHECK_FALSE(Steps::StepGenerateGcode::validate_pre(Orchestrator::instance(), print, &error));
        CHECK(error.find("Active tool: 3") != std::string::npos);
        CHECK(error.find("tool_groups[0].extruder_id") != std::string::npos);
        CHECK(error.find("outside the configured range") != std::string::npos);
    }

    SECTION("a semantic tool change supplies the tool to the following command") {
        Print print;
        PrintConfig &config = const_cast<PrintConfig &>(print.config());
        config.nozzle_diameter.set(std::vector<double>{0.4, 0.4});
        PrintingPlan &plan = print.mutable_printing_plan();

        ExtrusionEntity::Children commands;
        std::unique_ptr<ExtrusionNop> toolchange(new ExtrusionNop());
        toolchange->add_property(ExtrusionPropertySpecialCommand(
            ExtrusionPropertySpecialCommand::Code::TOOLCHANGE, 1.0));
        commands.push_back(std::move(toolchange));
        std::unique_ptr<ExtrusionNop> extrusion(new ExtrusionNop());
        extrusion->add_property(ExtrusionPropertySpecialCommand(
            ExtrusionPropertySpecialCommand::Code::EXTRUSION, 0.5));
        extrusion->add_property(ExtrusionPropertySpeed(15.f));
        commands.push_back(std::move(extrusion));
        plan.events.append_before(ExtrusionEntity(std::move(commands), false, false, true));

        CHECK(Steps::StepGenerateGcode::validate_pre(Orchestrator::instance(), print));
    }

    SECTION("a fractional semantic tool change is rejected instead of rounded") {
        Print print;
        PrintConfig &config = const_cast<PrintConfig &>(print.config());
        config.nozzle_diameter.set(std::vector<double>{0.4, 0.4});
        PrintingPlan &plan = print.mutable_printing_plan();

        ExtrusionEntity::Children commands;
        std::unique_ptr<ExtrusionNop> toolchange(new ExtrusionNop());
        toolchange->add_property(ExtrusionPropertySpecialCommand(
            ExtrusionPropertySpecialCommand::Code::TOOLCHANGE, 0.5));
        commands.push_back(std::move(toolchange));
        std::unique_ptr<ExtrusionNop> extrusion(new ExtrusionNop());
        extrusion->add_property(ExtrusionPropertySpecialCommand(
            ExtrusionPropertySpecialCommand::Code::EXTRUSION, 0.5));
        extrusion->add_property(ExtrusionPropertySpeed(15.f));
        commands.push_back(std::move(extrusion));
        plan.events.append_before(ExtrusionEntity(std::move(commands), false, false, true));

        std::string error;
        CHECK_FALSE(Steps::StepGenerateGcode::validate_pre(Orchestrator::instance(), print, &error));
        CHECK(error.find("invalid TOOLCHANGE value") != std::string::npos);
    }

    SECTION("the selected tool remains available to later after events") {
        Print print;
        append_speed_validation_tool_group(print, scale_i(0.42), 0);
        append_special_extrusion_event(print.mutable_printing_plan().events, false, 15.f);

        CHECK(Steps::StepGenerateGcode::validate_pre(Orchestrator::instance(), print));
    }
}

TEST_CASE("STEP_GCODE speed validation leaves retraction policy to the firmware",
          "[plugins][gcode][validation][speed][events][retraction]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    PrintingToolGroup &tool_group = append_speed_validation_tool_group(
        print, scale_i(0.42), 0);

    ExtrusionNop special_retract;
    special_retract.add_property(ExtrusionPropertySpecialCommand(
        ExtrusionPropertySpecialCommand::Code::RETRACT, -0.8));
    tool_group.events.append_before(special_retract);

    ExtrusionNop semantic_retract;
    semantic_retract.add_property(ExtrusionPropertyExtrusionAxis(
        C_EXTRUSION_AXIS_OPERATION_RETRACT_TO, 0.8));
    semantic_retract.add_property(ExtrusionAttributes(
        ExtrusionRole(ExtrusionRole::Travel | ExtrusionRoleModifier::ERM_Retract)));
    tool_group.events.append_after(semantic_retract);

    CHECK(Steps::StepGenerateGcode::validate_pre(Orchestrator::instance(), print));
}

TEST_CASE("STEP_GCODE speed diagnostics retain source object context",
          "[plugins][gcode][validation][speed]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    using namespace Slic3r::Test::PerimeterPluginTests;

    PreparedPerimeterPrint prepared;
    const DynamicPrintConfig config = perimeter_config({});
    prepare_cube_print(prepared, config);
    REQUIRE(prepared.print.objects().size() == 1);
    prepared.print.object(0).model_object()->name = "speed_validation_cube.stl";
    REQUIRE(prepared.print.object(0).layer_count() > 0);
    const Layer &source_layer = prepared.print.object(0).layer(0);
    REQUIRE(source_layer.region_count() > 0);
    const LayerRegionIsland source_island({&source_layer.region(0)}, 0);

    PrintingToolGroup &tool_group = append_speed_validation_tool_group(
        prepared.print, source_layer.scaled_print_z(), source_island.extruder_id());
    append_speed_validation_extrusion(
        tool_group, speed_validation_path(std::nullopt), &source_island, 0);

    std::string error;
    CHECK_FALSE(Steps::StepGenerateGcode::validate_pre(Orchestrator::instance(), prepared.print, &error));
    CHECK(error.find("region_count=1") != std::string::npos);
    CHECK(error.find("layer_id=0") != std::string::npos);
    CHECK(error.find("print_object_index=0") != std::string::npos);
    CHECK(error.find("object_name=\"speed_validation_cube.stl\"") != std::string::npos);
    CHECK(error.find("printing_catalog_id=0") != std::string::npos);
}

TEST_CASE("STEP_GCODE speed diagnostics are bounded without hiding the total",
          "[plugins][gcode][validation][speed]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    PrintingToolGroup &tool_group = append_speed_validation_tool_group(print);
    for (size_t extrusion_idx = 0; extrusion_idx < 12; ++extrusion_idx)
        append_speed_validation_extrusion(
            tool_group, speed_validation_path(std::nullopt, double(extrusion_idx), double(extrusion_idx + 1)));

    std::string error;
    CHECK_FALSE(Steps::StepGenerateGcode::validate_pre(Orchestrator::instance(), print, &error));
    CHECK(error.find("12 geometric extrusion leaves") != std::string::npos);
    CHECK(error.find("[10/12]") != std::string::npos);
    CHECK(error.find("[11/12]") == std::string::npos);
    CHECK(error.find("2 additional validation errors were omitted") != std::string::npos);
}

TEST_CASE("STEP_GCODE rejects an invalid speed before touching output",
          "[plugins][gcode][validation][speed]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    PrintingToolGroup &tool_group = append_speed_validation_tool_group(print);
    append_speed_validation_extrusion(tool_group, speed_validation_path(std::nullopt));

    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    write_text_file(output_path, "previous output\n");
    try {
        Steps::StepGenerateGcode::run_step(
            Orchestrator::instance(), print, output_path.string());
        FAIL("STEP_GCODE accepted a geometric leaf without speed");
    } catch (const RuntimeError &error) {
        CHECK(std::string(error.what()).find("PrintingPlan location") != std::string::npos);
    }
    CHECK(read_text_file(output_path) == "previous output\n");
    CHECK_FALSE(boost::filesystem::exists(output_path.string() + ".tmp"));
    remove_output_pair(output_path);
}

TEST_CASE("STEP_GCODE rejects an invalid E-only command before touching output",
          "[plugins][gcode][validation][speed][e-only]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    PrintingToolGroup &tool_group = append_speed_validation_tool_group(
        print, scale_i(0.42), 0);
    append_special_extrusion_event(tool_group.events, true, std::nullopt);

    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    write_text_file(output_path, "previous output\n");
    try {
        Steps::StepGenerateGcode::run_step(
            Orchestrator::instance(), print, output_path.string());
        FAIL("STEP_GCODE accepted an E-only extrusion command without speed");
    } catch (const RuntimeError &error) {
        CHECK(std::string(error.what()).find("movement=E-only special extrusion") !=
              std::string::npos);
    }
    CHECK(read_text_file(output_path) == "previous output\n");
    CHECK_FALSE(boost::filesystem::exists(output_path.string() + ".tmp"));
    remove_output_pair(output_path);
}

TEST_CASE("G-code formatter builds checked command parameters",
          "[plugins][gcode][formatter]")
{
    SECTION("integer and fixed parameters preserve their requested representation") {
        slic3r_api::GCodeGeneration::GCodeFormatter formatter(3, 5);
        formatter.emit_string("M204");
        formatter.emit_integer_parameter('P', 499.5);
        formatter.emit_fixed_parameter('T', 8.0, 2);
        formatter.emit_comment(true, "test command");
        CHECK(formatter.string() == "M204 P500 T8.00 ; test command\n");
    }

    SECTION("invalid values are rejected before encoding") {
        slic3r_api::GCodeGeneration::GCodeFormatter formatter(3, 5);
        CHECK_THROWS_AS(formatter.emit_integer_parameter('X', -1.0), std::invalid_argument);
        CHECK_THROWS_AS(
            formatter.emit_integer_parameter('X', std::numeric_limits<double>::quiet_NaN()),
            std::invalid_argument);
        CHECK_THROWS_AS(
            formatter.emit_fixed_parameter('X', std::numeric_limits<double>::infinity(), 2),
            std::invalid_argument);
        CHECK_THROWS_AS(
            formatter.emit_integer_parameter('X', std::numeric_limits<double>::max()),
            std::invalid_argument);
        CHECK_THROWS_AS(formatter.emit_fixed_parameter('X', 1.0, 10), std::invalid_argument);
    }
}
TEST_CASE("Default firmware owns machine envelope initialization",
          "[plugins][gcode][firmware][machine-envelope]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    SECTION("disabled limits initialize the session without calling the dialect hook") {
        Print print;
        configure_standard_firmware(print);
        const slic3r_api::Print print_view(
            reinterpret_cast<const print_handle *>(&print));
        MachineEnvelopeFirmwareSession session;
        CHECK(session.begin_print(print_view, slic3r_api::ConstPrintingPlan(reinterpret_cast<const printing_plan_handle *>(&print.mutable_printing_plan()))).find("machine-envelope") == std::string::npos);
        CHECK(session.formatter_initialized());
        CHECK(session.encode_count() == 0);
    }

    SECTION("enabled limits reach the dialect hook after setup") {
        Print print;
        configure_standard_firmware(print);
        configure_machine_envelope(print);
        const slic3r_api::Print print_view(
            reinterpret_cast<const print_handle *>(&print));
        MachineEnvelopeFirmwareSession session;
        CHECK(session.begin_print(print_view, slic3r_api::ConstPrintingPlan(reinterpret_cast<const printing_plan_handle *>(&print.mutable_printing_plan()))) .find("machine-envelope\n") != std::string::npos);
        CHECK(session.encode_count() == 1);
        CHECK(session.initialized_during_encoding());
        REQUIRE(session.encoded_envelope());
        CHECK(session.encoded_envelope()->max_acceleration_x == 1000.0);

        // The neutral base remains directly usable and deliberately has no
        // firmware-specific envelope syntax.
        slic3r_api::GCodeGeneration::DefaultGCodeFirmwareSession neutral_session;
        CHECK(neutral_session.begin_print(print_view, slic3r_api::ConstPrintingPlan(reinterpret_cast<const printing_plan_handle *>(&print.mutable_printing_plan()))).find("machine-envelope") == std::string::npos);
    }

    SECTION("invalid limits fail before entering the dialect hook") {
        Print print;
        configure_standard_firmware(print);
        configure_machine_envelope(print);
        PrintConfig &config = const_cast<PrintConfig &>(print.config());
        config.machine_max_feedrate_x.set(std::vector<double>{-1.0});
        const slic3r_api::Print print_view(
            reinterpret_cast<const print_handle *>(&print));
        MachineEnvelopeFirmwareSession session;
        CHECK_THROWS_AS(
            session.begin_print(print_view, slic3r_api::ConstPrintingPlan(reinterpret_cast<const printing_plan_handle *>(&print.mutable_printing_plan()))), std::invalid_argument);
        CHECK(session.formatter_initialized());
        CHECK(session.encode_count() == 0);
    }
}
TEST_CASE("Machine envelope reads and validates neutral printer limits",
          "[plugins][gcode][firmware][dialects][machine-envelope]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    // Presets may retain limits for estimates without asking firmware sessions
    // to publish those limits in the generated file.
    Print disabled_print;
    configure_standard_firmware(disabled_print);
    const slic3r_api::Print disabled_view(
        reinterpret_cast<const print_handle *>(&disabled_print));
    CHECK_FALSE(slic3r_api::GCodeGeneration::configured_machine_envelope(
        disabled_view.config()));

    // The public reader copies printer data without applying any dialect unit
    // conversion, leaving that decision to the selected firmware session.
    Print enabled_print;
    configure_standard_firmware(enabled_print);
    configure_machine_envelope(enabled_print);
    const slic3r_api::Print enabled_view(
        reinterpret_cast<const print_handle *>(&enabled_print));
    const std::optional<slic3r_api::GCodeGeneration::MachineEnvelope> envelope =
        slic3r_api::GCodeGeneration::configured_machine_envelope(enabled_view.config());
    REQUIRE(envelope);
    CHECK(envelope->max_acceleration_x == 1000.0);
    CHECK(envelope->max_acceleration_y == 1100.0);
    CHECK(envelope->max_acceleration_z == 120.0);
    CHECK(envelope->max_acceleration_e == 1300.0);
    CHECK(envelope->max_feedrate_x == 200.0);
    CHECK(envelope->max_feedrate_y == 210.0);
    CHECK(envelope->max_feedrate_z == 12.0);
    CHECK(envelope->max_feedrate_e == 25.0);
    CHECK(envelope->max_print_acceleration == 500.0);
    CHECK(envelope->max_retract_acceleration == 800.0);
    CHECK(envelope->max_travel_acceleration == 900.0);
    CHECK(envelope->max_jerk_x == 8.0);
    CHECK(envelope->max_jerk_y == 9.0);
    CHECK(envelope->max_jerk_z == 0.4);
    CHECK(envelope->max_jerk_e == 2.5);
    CHECK(envelope->min_extruding_feedrate == 3.0);
    CHECK(envelope->min_travel_feedrate == 4.0);

    SECTION("negative limits are rejected") {
        Print invalid_print;
        configure_standard_firmware(invalid_print);
        configure_machine_envelope(invalid_print);
        PrintConfig &config = const_cast<PrintConfig &>(invalid_print.config());
        config.machine_max_feedrate_x.set(std::vector<double>{-1.0});
        const slic3r_api::Print invalid_view(
            reinterpret_cast<const print_handle *>(&invalid_print));
        CHECK_THROWS_AS(
            slic3r_api::GCodeGeneration::configured_machine_envelope(invalid_view.config()),
            std::invalid_argument);
    }

    SECTION("non-finite limits are rejected") {
        Print invalid_print;
        configure_standard_firmware(invalid_print);
        configure_machine_envelope(invalid_print);
        PrintConfig &config = const_cast<PrintConfig &>(invalid_print.config());
        config.machine_max_feedrate_x.set(
            std::vector<double>{std::numeric_limits<double>::quiet_NaN()});
        const slic3r_api::Print invalid_view(
            reinterpret_cast<const print_handle *>(&invalid_print));
        CHECK_THROWS_AS(
            slic3r_api::GCodeGeneration::configured_machine_envelope(invalid_view.config()),
            std::invalid_argument);
    }
}
TEST_CASE("Default firmware interprets scope events with its extrusion visitor",
          "[plugins][gcode][firmware][printing][plan][events]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    select_printing_plan_firmware(print);
    PrintingPlan &plan = print.mutable_printing_plan();
    ExtrusionNop event;
    event.add_property(ExtrusionPropertyCustomGcodeText("M117 scope event"));
    plan.events.append_before(event);

    FirmwareInstanceOwner firmware(slic3r_api::make_gcode_firmware_instance(
        std::unique_ptr<slic3r_api::GCodeFirmwareSession>(
            new slic3r_api::GCodeGeneration::DefaultGCodeFirmwareSession())));
    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    try {
        run_file_writer_with_firmware(print, output_path, firmware.instance());
        CHECK(read_text_file(output_path) == "G21\nG90\nM82\nM117 scope event\n");
    } catch (...) {
        remove_output_pair(output_path);
        throw;
    }
    remove_output_pair(output_path);
}

TEST_CASE("Firmware object boundaries keep exit work with the source instance",
          "[plugins][gcode][firmware][printing][plan][object-instance]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Model model;
    Print print;
    Test::init_print({Test::TestMesh::cube_20x20x20,
                      Test::TestMesh::cube_20x20x20}, print, model);
    configure_standard_firmware(print);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.gcode_label_objects.value = LabelObjectsStyle::Both;
    PrintingPlan plan = build_printing_plan_by_object(print);
    PluginStorage storage;
    slic3r_api::Ordering::ObjectInstanceCatalogPlugin::populate_object_instance_catalog(
        reinterpret_cast<const print_handle *>(&print),
        reinterpret_cast<printing_plan_handle *>(&plan),
        reinterpret_cast<storage_handle *>(&storage));
    REQUIRE(plan.object_instances.size() == 2);

    const auto event_tree = [](std::initializer_list<const char *> commands) {
        std::unique_ptr<ExtrusionEntity> root(new ExtrusionEntity(true));
        for (const char *command : commands) {
            std::unique_ptr<ExtrusionNop> event(new ExtrusionNop());
            event->add_property(ExtrusionPropertyCustomGcodeText(command));
            root->append_child(std::move(event));
        }
        return root;
    };

    PrintingExtrusion first;
    first.printing_catalog_id = 0;
    first.root = event_tree({";A content", ";A retract", ";A wipe"});
    PrintingExtrusion second;
    second.printing_catalog_id = 1;
    second.root = event_tree({";B travel", ";B unretract", ";B content"});

    slic3r_api::GCodeGeneration::Firmware::Marlin2GCodeFirmwareSession session;
    std::string output = session.begin_print(
        slic3r_api::Print(reinterpret_cast<const print_handle *>(&print)),
        slic3r_api::ConstPrintingPlan(
            reinterpret_cast<const printing_plan_handle *>(&plan)));
    output += session.write_extrusion(slic3r_api::ConstPrintingExtrusion(
        reinterpret_cast<const printing_extrusion_handle *>(&first)));
    ExtrusionNop scope_event;
    scope_event.add_property(ExtrusionPropertyCustomGcodeText(";outside event"));
    output += session.write_event(slic3r_api::ExtrusionEntity(
        reinterpret_cast<const extrusion_entity_handle *>(&scope_event)));
    output += session.write_extrusion(slic3r_api::ConstPrintingExtrusion(
        reinterpret_cast<const printing_extrusion_handle *>(&second)));
    output += session.end_print();

    const size_t a_content = output.find("; A content");
    const size_t a_retract = output.find("; A retract");
    const size_t a_wipe = output.find("; A wipe");
    const size_t octoprint_close_a = output.find("; stop printing object", a_wipe);
    const size_t close_a = output.find("M486 S-1", a_wipe);
    const size_t outside_event = output.find("; outside event", close_a);
    const size_t octoprint_open_b = output.find("; printing object", outside_event);
    const size_t open_b = output.find("M486 S1", outside_event);
    const size_t b_travel = output.find("; B travel", open_b);
    const size_t b_unretract = output.find("; B unretract", b_travel);
    const size_t b_content = output.find("; B content", b_unretract);
    REQUIRE(a_content != std::string::npos);
    REQUIRE(a_retract != std::string::npos);
    REQUIRE(a_wipe != std::string::npos);
    REQUIRE(octoprint_close_a != std::string::npos);
    REQUIRE(close_a != std::string::npos);
    REQUIRE(outside_event != std::string::npos);
    REQUIRE(octoprint_open_b != std::string::npos);
    REQUIRE(open_b != std::string::npos);
    REQUIRE(b_travel != std::string::npos);
    REQUIRE(b_unretract != std::string::npos);
    REQUIRE(b_content != std::string::npos);
    CHECK(a_content < a_retract);
    CHECK(a_retract < a_wipe);
    CHECK(a_wipe < octoprint_close_a);
    CHECK(octoprint_close_a < close_a);
    CHECK(a_wipe < close_a);
    CHECK(close_a < outside_event);
    CHECK(outside_event < octoprint_open_b);
    CHECK(octoprint_open_b < open_b);
    CHECK(outside_event < open_b);
    CHECK(open_b < b_travel);
    CHECK(b_travel < b_unretract);
    CHECK(b_unretract < b_content);
    const std::string &first_name = plan.object_instances[0].name;
    const std::string &second_name = plan.object_instances[1].name;
    CHECK(object_protocol_lines(output) == std::vector<std::string>{
        "M486 T2",
        "; printing object " + first_name,
        "M486 S0",
        "; SLIC3R_OBJECT_INSTANCE_ID=0",
        "; stop printing object " + first_name,
        "M486 S-1",
        "; SLIC3R_OBJECT_INSTANCE_ID=-1",
        "; printing object " + second_name,
        "M486 S1",
        "; SLIC3R_OBJECT_INSTANCE_ID=1",
        "; stop printing object " + second_name,
        "M486 S-1",
        "; SLIC3R_OBJECT_INSTANCE_ID=-1"
    });
}

// Borrow a two-object catalog with an appended synthetic tower, then serialize
// real/tower/tower/real paths. Exact boundary prefixes prove tower motion is
// outside cancellation labels while real IDs and role markers remain intact.
TEST_CASE("Firmware prints global and instance wipe towers outside object labels",
          "[plugins][gcode][firmware][object-instance][wipe-tower][tower-labels]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const LabelObjectsStyle style = GENERATE(LabelObjectsStyle::Disabled,
        LabelObjectsStyle::Octoprint, LabelObjectsStyle::Firmware, LabelObjectsStyle::Both);
    const bool klipper = GENERATE(false, true);
    const bool instance_tower = GENERATE(false, true);
    CAPTURE(style, klipper, instance_tower);
    const bool enabled = style != LabelObjectsStyle::Disabled;
    const bool octoprint = style == LabelObjectsStyle::Octoprint || style == LabelObjectsStyle::Both;
    const bool firmware = style == LabelObjectsStyle::Firmware || style == LabelObjectsStyle::Both;

    Model model;
    Print print;
    Test::init_print({Test::TestMesh::cube_20x20x20, Test::TestMesh::cube_20x20x20}, print, model);
    ModelObject *first_object = const_cast<ModelObject *>(print.object(0).model_object());
    ModelObject *second_object = const_cast<ModelObject *>(print.object(1).model_object());
    REQUIRE(first_object != nullptr);
    REQUIRE(second_object != nullptr);
    first_object->name = "First object";
    second_object->name = "Second object";
    configure_standard_firmware(print);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.gcode_label_objects.value = style;
    config.gcode_flavor.value = klipper ? gcfKlipper : gcfMarlinFirmware;
    PrintingPlan plan = build_printing_plan_by_object(print);
    PluginStorage storage;
    slic3r_api::Ordering::ObjectInstanceCatalogPlugin::populate_object_instance_catalog(
        reinterpret_cast<const print_handle *>(&print),
        reinterpret_cast<printing_plan_handle *>(&plan),
        reinterpret_cast<storage_handle *>(&storage));
    REQUIRE(plan.object_instances.size() == 2);
    const std::string first_name = plan.object_instances[0].name;
    const std::string second_name = plan.object_instances[1].name;
    const std::string tower_name = instance_tower ? "Wipe tower First object #1" : "Wipe tower";
    const uint32_t tower_id = plan.add_synthetic_object_instance(tower_name,
        Polygon(Points{Point::new_scale(40., 40.), Point::new_scale(60., 40.),
                       Point::new_scale(60., 60.), Point::new_scale(40., 60.)}));
    REQUIRE(tower_id == 2);
    CHECK(plan.object_instances[0].name == first_name);
    CHECK(plan.object_instances[1].name == second_name);
    CHECK(plan.object_instances[0].object == &print.object(0));
    CHECK(plan.object_instances[1].object == &print.object(1));
    CHECK(plan.object_instances[tower_id].object == nullptr);

    PrintingToolGroup tool;
    tool.extruder_id = 0;
    const uint32_t ids[] = {0, tower_id, tower_id, 1};
    for (uint32_t idx = 0; idx < 4; ++idx) {
        append_path_extrusion(tool, make_firmware_path(
            ArcPolyline(Points{Point::new_scale(double(idx), 0.), Point::new_scale(double(idx + 1), 0.)}),
            20.f, 500.f, -1.f, -1.f, -1.f,
            ids[idx] == tower_id ? ExtrusionRole::WipeTower : ExtrusionRole::Perimeter));
        tool.extrusions.back().printing_catalog_id = ids[idx];
    }
    std::unique_ptr<slic3r_api::GCodeGeneration::DefaultGCodeFirmwareSession> session;
    if (klipper)
        session.reset(new slic3r_api::GCodeGeneration::Firmware::KlipperGCodeFirmwareSession());
    else
        session.reset(new slic3r_api::GCodeGeneration::Firmware::Marlin2GCodeFirmwareSession());
    std::string output = session->begin_print(
        slic3r_api::Print(reinterpret_cast<const print_handle *>(&print)),
        slic3r_api::ConstPrintingPlan(reinterpret_cast<const printing_plan_handle *>(&plan)));
    INFO(output);
    CHECK(count_occurrences(output, "EXCLUDE_OBJECT_DEFINE") == (firmware && klipper ? 2 : 0));
    CHECK(count_occurrences(output, "M486 T2\n") == (firmware && !klipper ? 1 : 0));
    if (firmware && klipper) {
        CHECK(output.find("EXCLUDE_OBJECT_DEFINE NAME='" + first_name + "'") != std::string::npos);
        CHECK(output.find("EXCLUDE_OBJECT_DEFINE NAME='" + second_name + "'") != std::string::npos);
    }
    output += session->begin_tool_group(slic3r_api::ConstPrintingToolGroup(
        reinterpret_cast<const printing_tool_group_handle *>(&tool)));
    const std::string first = session->write_extrusion(slic3r_api::ConstPrintingExtrusion(
        reinterpret_cast<const printing_extrusion_handle *>(&tool.extrusions[0])));
    const std::string tower = session->write_extrusion(slic3r_api::ConstPrintingExtrusion(
        reinterpret_cast<const printing_extrusion_handle *>(&tool.extrusions[1])));
    const std::string continued_tower = session->write_extrusion(slic3r_api::ConstPrintingExtrusion(
        reinterpret_cast<const printing_extrusion_handle *>(&tool.extrusions[2])));
    const std::string second = session->write_extrusion(slic3r_api::ConstPrintingExtrusion(
        reinterpret_cast<const printing_extrusion_handle *>(&tool.extrusions[3])));
    output += first + tower + continued_tower + second + session->end_print();
    INFO(output);

    std::string close_first;
    std::string open_first;
    std::string open_second;
    if (octoprint) {
        close_first += "; stop printing object " + first_name + "\n";
        open_first += "; printing object " + first_name + "\n";
        open_second += "; printing object " + second_name + "\n";
    }
    if (firmware) {
        close_first += klipper ? "EXCLUDE_OBJECT_END NAME='" + first_name + "'\n" : "M486 S-1\n";
        open_first += klipper ? "EXCLUDE_OBJECT_START NAME='" + first_name + "'\n" : "M486 S0\n";
        open_second += klipper ? "EXCLUDE_OBJECT_START NAME='" + second_name + "'\n" : "M486 S1\n";
    }
    if (enabled) {
        close_first += "; SLIC3R_OBJECT_INSTANCE_ID=-1\n";
        open_first += "; SLIC3R_OBJECT_INSTANCE_ID=0\n";
        open_second += "; SLIC3R_OBJECT_INSTANCE_ID=1\n";
    }
    CHECK(first.compare(0, open_first.size(), open_first) == 0);
    CHECK(tower.compare(0, close_first.size(), close_first) == 0);
    CHECK(second.compare(0, open_second.size(), open_second) == 0);
    const size_t tower_move = tower.find("G1 X2 E");
    REQUIRE(tower_move != std::string::npos);
    CHECK(tower_move >= close_first.size());
    CHECK(tower.find(";TYPE:Wipe tower\n") != std::string::npos);
    CHECK(continued_tower.find("G1 X3 E") != std::string::npos);
    CHECK(continued_tower.find("SLIC3R_OBJECT_INSTANCE_ID") == std::string::npos);
    CHECK(continued_tower.find("printing object") == std::string::npos);
    CHECK(continued_tower.find("M486") == std::string::npos);
    CHECK(continued_tower.find("EXCLUDE_OBJECT") == std::string::npos);
    CHECK(output.find("printing object Wipe tower") == std::string::npos);
    CHECK(output.find("M486 S2\n") == std::string::npos);
    CHECK(output.find("EXCLUDE_OBJECT_START NAME='Wipe tower") == std::string::npos);
    CHECK(output.find("EXCLUDE_OBJECT_DEFINE NAME='Wipe tower") == std::string::npos);
    CHECK(output.find("; SLIC3R_OBJECT_INSTANCE_ID=2\n") == std::string::npos);
    if (!enabled) {
        CHECK(output.find("printing object") == std::string::npos);
        CHECK(output.find("SLIC3R_OBJECT_INSTANCE_ID") == std::string::npos);
        CHECK(output.find("M486") == std::string::npos);
        CHECK(output.find("EXCLUDE_OBJECT") == std::string::npos);
    }
}

TEST_CASE("Klipper object declarations use the PrintingPlan instance catalog",
          "[plugins][gcode][firmware][printing][plan][object-instance][klipper]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Model model;
    Print print;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model);
    ModelObject *model_object = const_cast<ModelObject *>(print.object(0).model_object());
    REQUIRE(model_object != nullptr);
    model_object->name = "Single object";
    configure_standard_firmware(print);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.gcode_label_objects.value = LabelObjectsStyle::Firmware;
    config.gcode_flavor.value = gcfKlipper;
    PrintingPlan plan = build_printing_plan_by_object(print);
    PluginStorage storage;
    slic3r_api::Ordering::ObjectInstanceCatalogPlugin::populate_object_instance_catalog(
        reinterpret_cast<const print_handle *>(&print),
        reinterpret_cast<printing_plan_handle *>(&plan),
        reinterpret_cast<storage_handle *>(&storage));
    REQUIRE(plan.object_instances.size() == 1);
    CHECK(plan.object_instances.front().name == "Single_object");

    slic3r_api::GCodeGeneration::Firmware::KlipperGCodeFirmwareSession session;
    const std::string output = session.begin_print(
        slic3r_api::Print(reinterpret_cast<const print_handle *>(&print)),
        slic3r_api::ConstPrintingPlan(
            reinterpret_cast<const printing_plan_handle *>(&plan)));
    CHECK(output.find("EXCLUDE_OBJECT_DEFINE NAME='" +
                      plan.object_instances.front().name + "'") != std::string::npos);
    CHECK(output.find(" CENTER=") != std::string::npos);
    CHECK(output.find(" POLYGON=[[") != std::string::npos);
}
TEST_CASE("Default firmware emits inherited process state and ordered linear moves",
          "[plugins][gcode][firmware]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    select_printing_plan_firmware(print);
    configure_standard_firmware(print, 2);
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.front().layers.front();
    layer.print_z = scale_i(0.2);
    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool_group = layer.tool_groups.front();
    tool_group.extruder_id = 0;

    ArcPolyline polyline(Points{
        Point(scale_i(0.0), scale_i(0.0)),
        Point(scale_i(10.0), scale_i(0.0)),
        Point(scale_i(20.0), scale_i(0.0))
    });
    polyline.set_z_offset(0, scale_i(0.1));
    polyline.set_z_offset(1, scale_i(0.1));
    polyline.set_z_offset(2, scale_i(0.2));
    append_path_extrusion(tool_group, make_firmware_path(polyline, 20.f, 500.f, 0.04f, 50.f, 210.f));

    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    try {
        Steps::StepGenerateGcode::run_step(Orchestrator::instance(), print, output_path.string());
        const std::string output = read_text_file(output_path);
        INFO(output);

        const size_t temperature_pos = output.find("M104 S210 T0\n");
        const size_t fan_pos = output.find("M106 S128\n");
        const size_t pressure_pos = output.find("M900 K0.04\n");
        const size_t travel_pos = output.find("G0 X0 Y0 Z.3 F6000\n");
        const size_t print_acceleration_pos = output.find("M204 P500\n");
        const size_t first_extrusion_pos = output.find("G1 X10 E");
        const size_t second_extrusion_pos = output.find("G1 X20 Z.4 E");
        REQUIRE(temperature_pos != std::string::npos);
        REQUIRE(fan_pos != std::string::npos);
        REQUIRE(pressure_pos != std::string::npos);
        REQUIRE(travel_pos != std::string::npos);
        REQUIRE(print_acceleration_pos != std::string::npos);
        REQUIRE(first_extrusion_pos != std::string::npos);
        REQUIRE(second_extrusion_pos != std::string::npos);
        CHECK(temperature_pos < fan_pos);
        CHECK(fan_pos < pressure_pos);
        CHECK(pressure_pos < travel_pos);
        CHECK(travel_pos < print_acceleration_pos);
        CHECK(print_acceleration_pos < first_extrusion_pos);
        CHECK(first_extrusion_pos < second_extrusion_pos);

        // State remains emitted across both segments. Speed is attached only
        // to the first movement that needs the new value.
        CHECK(count_occurrences(output, "M104 S210 T0\n") == 1);
        CHECK(count_occurrences(output, "M106 S128\n") == 1);
        CHECK(count_occurrences(output, "M900 K0.04\n") == 1);
        CHECK(count_occurrences(output, "M204 P500\n") == 1);
        CHECK(output.find("M204 T") == std::string::npos);
        CHECK(count_occurrences(output, " F1200") == 1);
        CHECK_FALSE(boost::filesystem::exists(output_path.string() + ".tmp"));
    } catch (...) {
        remove_output_pair(output_path);
        throw;
    }
    remove_output_pair(output_path);
}

// Export two source-free layers and check that each maximum-height marker precedes the forced extrusion-height tag.
TEST_CASE("Default firmware emits ordered semantic marker triplets for layers",
          "[plugins][gcode][firmware][markers]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    configure_standard_firmware(print);
    PrintingLayerGroup &first_layer = append_marker_test_layer(print, scale_i(0.2));
    append_path_extrusion(
        first_layer.tool_groups.front(),
        marker_test_path(0.0, 1.0, ExtrusionRole::Perimeter, 0.4f, 0.2f));
    PrintingLayerGroup &second_layer = append_marker_test_layer(print, scale_i(0.4));
    append_path_extrusion(
        second_layer.tool_groups.front(),
        marker_test_path(1.0, 2.0, ExtrusionRole::Perimeter, 0.4f, 0.25f));

    const std::string output = export_with_firmware(
        print, "gcode.firmware.marlin2");
    INFO(output);
    const std::vector<std::string> expected{
        ";LAYER_CHANGE", ";Z:0.2", ";MAX_LAYER_HEIGHT:0.2",
        ";TYPE:Perimeter", ";WIDTH:0.4", ";HEIGHT:0.2",
        ";LAYER_CHANGE", ";Z:0.4", ";MAX_LAYER_HEIGHT:0.2",
        ";HEIGHT:0.25"
    };
    CHECK(semantic_marker_lines(output) == expected);
}

// Export two source-free layers of equal extrusion height and verify each first positive-E XY move follows a fresh HEIGHT marker.
TEST_CASE("Default firmware rewrites HEIGHT before same-height layer extrusions",
          "[plugins][gcode][firmware][markers]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    configure_standard_firmware(print);
    PrintingLayerGroup &first_layer = append_marker_test_layer(print, scale_i(0.2));
    append_path_extrusion(
        first_layer.tool_groups.front(),
        marker_test_path(0.0, 1.0, ExtrusionRole::Perimeter, 0.4f, 0.2f));
    PrintingLayerGroup &second_layer = append_marker_test_layer(print, scale_i(0.4));
    append_path_extrusion(
        second_layer.tool_groups.front(),
        marker_test_path(1.0, 2.0, ExtrusionRole::Perimeter, 0.4f, 0.2f));

    const std::string output = export_with_firmware(print, "gcode.firmware.marlin2");
    INFO(output);
    CHECK(semantic_marker_lines(output) == std::vector<std::string>{
        ";LAYER_CHANGE", ";Z:0.2", ";MAX_LAYER_HEIGHT:0.2",
        ";TYPE:Perimeter", ";WIDTH:0.4", ";HEIGHT:0.2",
        ";LAYER_CHANGE", ";Z:0.4", ";MAX_LAYER_HEIGHT:0.2",
        ";HEIGHT:0.2"
    });

    GCodeReader reader;
    int32_t current_layer = -1;
    bool height_seen = false;
    bool first_layer_extrusion_checked = false;
    bool second_layer_extrusion_checked = false;
    reader.parse_buffer(output, [&current_layer, &height_seen,
                                 &first_layer_extrusion_checked,
                                 &second_layer_extrusion_checked](
        GCodeReader &state, const GCodeReader::GCodeLine &line) {
        if (line.comment() == "LAYER_CHANGE") {
            ++current_layer;
            height_seen = false;
        } else if (line.comment().rfind("HEIGHT:", 0) == 0) {
            height_seen = true;
        }
        if (line.extruding(state) && line.dist_XY(state) > 0.) {
            if (current_layer == 0 && !first_layer_extrusion_checked) {
                CHECK(height_seen);
                first_layer_extrusion_checked = true;
            } else if (current_layer == 1 && !second_layer_extrusion_checked) {
                CHECK(height_seen);
                second_layer_extrusion_checked = true;
            }
        }
    });
    CHECK(first_layer_extrusion_checked);
    CHECK(second_layer_extrusion_checked);
}

// Export source-free layers at distinct Z gaps and verify MAX_LAYER_HEIGHT follows each gap instead of the extrusion height.
TEST_CASE("Default firmware uses the Z gap when a layer group has no source layers",
          "[plugins][gcode][firmware][markers]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    configure_standard_firmware(print);
    PrintingLayerGroup &first_layer = append_marker_test_layer(print, scale_i(0.2));
    append_path_extrusion(
        first_layer.tool_groups.front(),
        marker_test_path(0.0, 1.0, ExtrusionRole::Perimeter, 0.4f, 0.3f));
    PrintingLayerGroup &second_layer = append_marker_test_layer(print, scale_i(0.45));
    append_path_extrusion(
        second_layer.tool_groups.front(),
        marker_test_path(1.0, 2.0, ExtrusionRole::Perimeter, 0.4f, 0.3f));

    const std::string output = export_with_firmware(print, "gcode.firmware.marlin2");
    INFO(output);
    CHECK(semantic_marker_lines(output) == std::vector<std::string>{
        ";LAYER_CHANGE", ";Z:0.2", ";MAX_LAYER_HEIGHT:0.2",
        ";TYPE:Perimeter", ";WIDTH:0.4", ";HEIGHT:0.3",
        ";LAYER_CHANGE", ";Z:0.45", ";MAX_LAYER_HEIGHT:0.25",
        ";HEIGHT:0.3"
    });
}

// Borrow object/support sources and an auxiliary-only group in a manual plan, then export and assert maximum markers separately from extrusion heights; pipeline or output mismatches fail assertions.
TEST_CASE("Default firmware layer change describes mixed-height print Z groups",
          "[plugins][gcode][firmware][markers]")
{
    // A print Z may group object and auxiliary layers of different heights.
    // The layer marker describes their maximum; each extrusion keeps its own height.
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Model model;
    Print print;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model,
                     {{"layer_height", "0.2"}, {"first_layer_height", "0.2"}});
    Orchestrator &orchestrator = Orchestrator::instance();
    Steps::StepLayerHeightGeneration::run_step(orchestrator, print);
    Steps::StepSlicing::run_step(orchestrator, print);
    PrintObject &object = print.object(0);
    REQUIRE(object.layer_count() > 1);
    const Layer &object_layer = object.layer(1);
    REQUIRE(object_layer.scaled_print_z() == scale_i(0.4));
    REQUIRE(object_layer.scaled_height() == scale_i(0.2));

    // A thinner support layer at the object Z, listed before the object layer
    // so the choice cannot depend on the group order, then two support layers
    // alone at a higher Z.
    const coord_t support_z = scale_i(1.4);
    const Layer &thin_support = object.add_auxiliary_layer(0, scale_i(0.1), object_layer.scaled_print_z());
    const Layer &thick_support = object.add_auxiliary_layer(1, scale_i(0.3), support_z);
    const Layer &middle_support = object.add_auxiliary_layer(2, scale_i(0.15), support_z);

    configure_standard_firmware(print);
    PrintingLayerGroup &mixed_layer = append_marker_test_layer(print, object_layer.scaled_print_z());
    mixed_layer.layers = {&thin_support, &object_layer};
    append_path_extrusion(
        mixed_layer.tool_groups.front(),
        marker_test_path(0.0, 1.0, ExtrusionRole::SupportMaterial, 0.4f, 0.1f));
    append_path_extrusion(
        mixed_layer.tool_groups.front(),
        marker_test_path(1.0, 2.0, ExtrusionRole::Perimeter, 0.4f, 0.2f));
    PrintingLayerGroup &support_layer = append_marker_test_layer(print, support_z);
    support_layer.layers = {&thick_support, &middle_support};
    append_path_extrusion(
        support_layer.tool_groups.front(),
        marker_test_path(2.0, 3.0, ExtrusionRole::SupportMaterial, 0.4f, 0.3f));

    const std::string output = export_with_firmware(
        print, "gcode.firmware.marlin2");
    INFO(output);
    const std::vector<std::string> expected{
        ";LAYER_CHANGE", ";Z:0.4", ";MAX_LAYER_HEIGHT:0.2",
        ";TYPE:Support material", ";WIDTH:0.4", ";HEIGHT:0.1",
        ";TYPE:Perimeter", ";HEIGHT:0.2",
        ";LAYER_CHANGE", ";Z:1.4", ";MAX_LAYER_HEIGHT:0.3",
        ";TYPE:Support material", ";HEIGHT:0.3"
    };
    CHECK(semantic_marker_lines(output) == expected);
}

// Slice two owned objects on different tools, reverse the mixed group's source
// list, and compare before-layer and tool-change scripts with ordered source
// heights; slicing, plugin and firmware failures remain visible.
TEST_CASE("Default firmware accepts different object layer heights in the by-layer plan",
          "[plugins][gcode][firmware][markers]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Model model;
    Print print;
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"first_layer_height", "0.2"},
                                   {"support_material", "0"}});
    initialize_mixed_height_cube_print(print, model, config, 1, 2);
    Orchestrator &orchestrator = Orchestrator::instance();
    print.process();
    configure_standard_firmware(print, 2);
    PrintConfig &script_config = const_cast<PrintConfig &>(print.config());
    script_config.option<ConfigOptionString>("start_gcode")->value.clear();
    script_config.option<ConfigOptionString>("end_gcode")->value.clear();
    script_config.before_layer_gcode.value = "; MIXED_BEFORE H{layer_current_height}";
    script_config.layer_gcode.value.clear();
    script_config.option<ConfigOptionString>("toolchange_gcode")->value =
        "; MIXED_TOOL H{layer_current_height}";
    script_config.option<ConfigOptionStrings>("start_filament_gcode")->set(
        std::vector<std::string>{"", ""});
    script_config.option<ConfigOptionStrings>("end_filament_gcode")->set(
        std::vector<std::string>{"", ""});
    Steps::StepExtrusionOrdering::clear(print);
    Steps::StepExtrusionOrdering::prepare(print);
    Steps::StepExtrusionOrdering::run_step(orchestrator, print);
    PrintingPlan &plan = print.mutable_printing_plan();
    REQUIRE(plan.groups.size() == 1);

    // The builder merges only exactly equal scaled Z values stored as coord_t.
    // Keep the plan and print alive while this non-owning pointer is used.
    PrintingLayerGroup *mixed_layer = nullptr;
    for (PrintingLayerGroup &layer : plan.groups.front().layers) {
        if (layer.layers.size() == 2 &&
            layer.layers[0]->scaled_height() != layer.layers[1]->scaled_height()) {
            mixed_layer = &layer;
            break;
        }
    }
    REQUIRE(mixed_layer != nullptr);
    REQUIRE(mixed_layer->print_z == scale_i(0.8));
    std::swap(mixed_layer->layers[0], mixed_layer->layers[1]);
    const Layer &first_layer = *mixed_layer->layers[0];
    const Layer &second_layer = *mixed_layer->layers[1];
    INFO("shared scaled Z: " << mixed_layer->print_z << "; heights: "
         << first_layer.scaled_height() << ", " << second_layer.scaled_height());
    REQUIRE(first_layer.scaled_print_z() == second_layer.scaled_print_z());
    REQUIRE(first_layer.scaled_print_z() == mixed_layer->print_z);
    REQUIRE(first_layer.scaled_height() == scale_i(0.3));
    REQUIRE(second_layer.scaled_height() == scale_i(0.2));

    Steps::StepExtrusionEdition::clear(print);
    Slic3r::Test::TransitionPipeline::run_layer_plugins(
        print, {Slic3r::Test::TransitionPipeline::TOOLCHANGE_PLUGIN});
    Steps::StepExtrusionEdition::run_step(orchestrator, print);
    PrintingPlan &installed_plan = print.mutable_printing_plan();
    mixed_layer = nullptr;
    size_t mixed_layer_idx = 0;
    for (size_t layer_idx = 0; layer_idx < installed_plan.groups.front().layers.size(); ++layer_idx) {
        PrintingLayerGroup &layer = installed_plan.groups.front().layers[layer_idx];
        if (layer.print_z == scale_i(0.8)) {
            mixed_layer = &layer;
            mixed_layer_idx = layer_idx;
            break;
        }
    }
    REQUIRE(mixed_layer != nullptr);
    REQUIRE(mixed_layer_idx > 0);
    const PrintingLayerGroup &previous_layer =
        installed_plan.groups.front().layers[mixed_layer_idx - 1];
    REQUIRE(previous_layer.events.after().child_count() == 1);
    const ExtrusionEntity &before_script = previous_layer.events.after().child(0);
    const ExtrusionPropertyCustomGcode *before_property =
        before_script.get_property<ExtrusionPropertyCustomGcode>();
    REQUIRE(before_property != nullptr);
    REQUIRE(before_property->script_type == GCODE_SCRIPT_TYPE_BEFORE_LAYER_GCODE);

    GCodeScriptProcessor host_processor(print, orchestrator);
    const slic3r_api::GCodeGeneration::GCodeScriptProcessorView scripts(
        host_processor.c_processor());
    PluginStorage script_storage;
    storage_handle *script_storage_handle =
        reinterpret_cast<storage_handle *>(&script_storage);
    ScriptStateFirmwareProbe script_firmware(scripts, script_storage_handle);
    const slic3r_api::Print print_view(reinterpret_cast<const print_handle *>(&print));
    const slic3r_api::PrintingPlan plan_view = printing_plan_view(print);
    script_firmware.begin_print(print_view, plan_view);
    script_firmware.begin_layer(slic3r_api::ConstPrintingLayerGroup(
        reinterpret_cast<const printing_layer_group_handle *>(&previous_layer)));
    CHECK(script_firmware.write_event(slic3r_api::ExtrusionEntity(
              reinterpret_cast<const extrusion_entity_handle *>(&before_script))) ==
          "; MIXED_BEFORE H0.2\n");
    script_firmware.begin_layer(slic3r_api::ConstPrintingLayerGroup(
        reinterpret_cast<const printing_layer_group_handle *>(mixed_layer)));
    CHECK(script_firmware.process_script(
              GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM,
              "; MIXED_FIRMWARE H{layer_current_height}") ==
          "; MIXED_FIRMWARE H0.2\n");
    const PrintingToolGroup *thick_tool = nullptr;
    std::ostringstream ordered_tool_groups;
    for (const PrintingToolGroup &tool_group : mixed_layer->tool_groups)
    {
        const std::optional<coord_t> source_height = slic3r_api::first_source_layer_height(
            slic3r_api::ConstPrintingToolGroup(
                reinterpret_cast<const printing_tool_group_handle *>(&tool_group)));
        ordered_tool_groups << tool_group.extruder_id << ':'
                            << (source_height ? unscaled(*source_height) : -1.0) << '/'
                            << tool_group.events.before().child_count() << ' ';
        if (tool_group.extruder_id == 1)
            thick_tool = &tool_group;
    }
    INFO("Ordered tools, source heights and before-events: " << ordered_tool_groups.str());
    REQUIRE(thick_tool != nullptr);
    script_firmware.begin_tool_group(slic3r_api::ConstPrintingToolGroup(
        reinterpret_cast<const printing_tool_group_handle *>(thick_tool)));
    REQUIRE(thick_tool->events.before().child_count() == 1);
    const ExtrusionEntity &toolchange_script = thick_tool->events.before().child(0);
    const ExtrusionPropertyCustomGcode *toolchange_property =
        toolchange_script.get_property<ExtrusionPropertyCustomGcode>();
    REQUIRE(toolchange_property != nullptr);
    REQUIRE(toolchange_property->script_type == GCODE_SCRIPT_TYPE_TOOLCHANGE_GCODE);
    CHECK(script_firmware.write_event(slic3r_api::ExtrusionEntity(
              reinterpret_cast<const extrusion_entity_handle *>(&toolchange_script))) ==
          "; MIXED_TOOL H0.3\nG92 E0\n");

    slic3r_api::GCodeGeneration::Firmware::Marlin2GCodeFirmwareSession session;
    session.begin_print(
        slic3r_api::Print(reinterpret_cast<const print_handle *>(&print)),
        slic3r_api::ConstPrintingPlan(
            reinterpret_cast<const printing_plan_handle *>(&installed_plan)));
    std::string output;
    REQUIRE_NOTHROW(output = session.begin_layer(slic3r_api::ConstPrintingLayerGroup(
        reinterpret_cast<const printing_layer_group_handle *>(mixed_layer))));
    const std::vector<std::string> markers = semantic_marker_lines(output);
    CHECK(output == ";LAYER_CHANGE\n;Z:0.8\n;MAX_LAYER_HEIGHT:0.3\n");
    CHECK(markers == std::vector<std::string>{
        ";LAYER_CHANGE", ";Z:0.8", ";MAX_LAYER_HEIGHT:0.3"
    });
}

// Export two owned cubes with different layer heights, observe STEP_GCODE through
// progress and STEP_NONE after export, then verify object markers and reader output.
TEST_CASE("Default firmware exports different object layer heights in the by-layer plan",
          "[plugins][gcode][firmware][markers]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Model model;
    Print print;
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"first_layer_height", "0.2"},
                                   {"support_material", "0"},
                                   {"print_plan_builder", "ordering.plan_builder.by_layer"},
                                   {"complete_objects_step", "0"},
                                   {"seam_visibility", "0"}});
    Test::init_print({Test::TestMesh::cube_20x20x20, Test::TestMesh::cube_20x20x20},
                     print, model, config);
    model.objects()[0].config.set_deserialize_strict("layer_height", "0.2");
    model.objects()[1].config.set_deserialize_strict("layer_height", "0.3");
    print.apply(model, config);
    const_cast<PrintConfig &>(print.config()).gcode_label_objects.value =
        LabelObjectsStyle::Both;
    REQUIRE_NOTHROW(print.process());

    // Export resumes the slicing pipeline through production ordering and the
    // selected plugin writer. Always remove both output files, including on failure.
    select_printing_plan_firmware(print, "gcode.firmware.marlin2");
    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    std::string output;
    std::atomic<uint32_t> gcode_progress_calls{0};
    // Progress observes the reference-generation step without moving thumbnail
    // rendering back from STEP_EXPORT. Workers may report concurrently.
    print.set_status_callback([&print, &gcode_progress_calls](const PrintBase::SlicingStatus &) {
        if (print.running_step() == STEP_GCODE)
            gcode_progress_calls.fetch_add(1, std::memory_order_relaxed);
    });
    CHECK(print.running_step() == STEP_NONE);
    try {
        Orchestrator::instance().export_gcode(print, output_path.string(), nullptr);
        CHECK(gcode_progress_calls.load(std::memory_order_relaxed) > 0);
        CHECK(print.running_step() == STEP_NONE);
        print.set_status_callback(nullptr);
        output = read_text_file(output_path);
    } catch (...) {
        print.set_status_callback(nullptr);
        remove_output_pair(output_path);
        throw;
    }
    remove_output_pair(output_path);

    // Inspect the production plan after exporting: both sources must occupy
    // exactly the same scaled Z, with their different sliced heights intact.
    REQUIRE(print.printing_plan() != nullptr);
    const PrintingPlan &plan = *print.printing_plan();
    REQUIRE(plan.groups.size() == 1);
    const PrintingLayerGroup *mixed_layer = nullptr;
    for (const PrintingLayerGroup &layer : plan.groups.front().layers)
        if (layer.print_z == scale_i(0.8)) {
            mixed_layer = &layer;
            break;
        }
    REQUIRE(mixed_layer != nullptr);
    REQUIRE(mixed_layer->layers.size() == 2);
    REQUIRE(mixed_layer->layers[0]->scaled_print_z() == scale_i(0.8));
    REQUIRE(mixed_layer->layers[1]->scaled_print_z() == scale_i(0.8));
    REQUIRE(mixed_layer->layers[0]->scaled_height() == scale_i(0.2));
    REQUIRE(mixed_layer->layers[1]->scaled_height() == scale_i(0.3));

    GCodeReaderService reader_service(Orchestrator::instance(), nullptr);
    const GCodeProcessorResult read_result = reader_service.read_for_viewer(
        "gcode.reader.default", output);
    bool max_height_at_target_z = false;
    for (const GCodeProcessorResult::MoveVertex &move : read_result.moves) {
        if (move.position.z() == Approx(0.8) &&
            move.layer_id < read_result.layers_max_height.size() &&
            read_result.layers_max_height[move.layer_id] == Approx(0.3)) {
            max_height_at_target_z = true;
            break;
        }
    }
    CHECK(max_height_at_target_z);

    const std::string layer_start =
        ";LAYER_CHANGE\n;Z:0.8\n;MAX_LAYER_HEIGHT:0.3\n";
    const size_t begin = output.find(layer_start);
    REQUIRE(begin != std::string::npos);
    const size_t end = output.find(";LAYER_CHANGE\n", begin + layer_start.size());
    REQUIRE(end != std::string::npos);
    CHECK(output.find(";HEIGHT:0.2\n", begin + layer_start.size()) < end);
    CHECK(output.find(";HEIGHT:0.3\n", begin + layer_start.size()) < end);

    // Object labels identify which first positive-E XY movement must follow
    // its own extrusion-height marker within the shared print-Z group.
    GCodeReader object_reader;
    int32_t current_object = -1;
    double current_height = 0.0;
    bool first_object_height_seen = false;
    bool second_object_height_seen = false;
    bool first_object_extrusion_seen = false;
    bool second_object_extrusion_seen = false;
    object_reader.parse_buffer(output, [&current_object, &current_height,
                                        &first_object_height_seen,
                                        &second_object_height_seen,
                                        &first_object_extrusion_seen,
                                        &second_object_extrusion_seen](
        GCodeReader &state, const GCodeReader::GCodeLine &line) {
        const std::string comment(line.comment());
        const size_t object_marker = comment.find("SLIC3R_OBJECT_INSTANCE_ID=");
        if (object_marker != std::string::npos)
            current_object = std::stoi(comment.substr(object_marker + 26));
        else if (comment.rfind("HEIGHT:", 0) == 0) {
            current_height = std::stod(comment.substr(7));
            if (current_object == 0 && !first_object_extrusion_seen)
                first_object_height_seen = true;
            if (current_object == 1 && !second_object_extrusion_seen)
                second_object_height_seen = true;
        }
        if (line.extruding(state) && line.dist_XY(state) > 0.) {
            if (current_object == 0 && !first_object_extrusion_seen &&
                state.z() == Approx(0.8)) {
                CHECK(first_object_height_seen);
                CHECK(current_height == Approx(0.2));
                first_object_extrusion_seen = true;
            } else if (current_object == 1 && !second_object_extrusion_seen &&
                       state.z() == Approx(0.8)) {
                CHECK(second_object_height_seen);
                CHECK(current_height == Approx(0.3));
                second_object_extrusion_seen = true;
            }
        }
    });
    CHECK(first_object_extrusion_seen);
    CHECK(second_object_extrusion_seen);

    // Parse the whole file to retain absolute-E state and resets. Inside the
    // target layer, require XY motion with increasing E while height is 0.3;
    // travel and E-only unretraction cannot satisfy this check.
    GCodeReader reader;
    bool target_layer = false;
    bool thick_height = false;
    bool thick_extrusion = false;
    reader.parse_buffer(output, [&target_layer, &thick_height, &thick_extrusion](
        GCodeReader &state, const GCodeReader::GCodeLine &line) {
        if (line.comment() == "LAYER_CHANGE") {
            target_layer = false;
            thick_height = false;
        } else if (line.comment() == "Z:0.8") {
            target_layer = true;
        } else if (line.comment().substr(0, 7) == "HEIGHT:") {
            thick_height = line.comment() == "HEIGHT:0.3";
        }
        if (target_layer && thick_height && line.extruding(state) && line.dist_XY(state) > 0.)
            thick_extrusion = true;
    });
    CHECK(thick_extrusion);
}

// Export one source-free layer and verify its maximum-height tag and extrusion attributes all precede the first printable move.
TEST_CASE("Default firmware emits TYPE WIDTH and HEIGHT before printable geometry",
          "[plugins][gcode][firmware][markers]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    configure_standard_firmware(print);
    PrintingLayerGroup &layer = append_marker_test_layer(print, scale_i(0.2));
    append_path_extrusion(
        layer.tool_groups.front(),
        marker_test_path(0.0, 1.0, ExtrusionRole::Perimeter, 0.45f, 0.2f));

    const std::string output = export_with_firmware(
        print, "gcode.firmware.marlin2");
    INFO(output);
    const std::vector<std::string> expected{
        ";LAYER_CHANGE", ";Z:0.2", ";MAX_LAYER_HEIGHT:0.2",
        ";TYPE:Perimeter", ";WIDTH:0.45", ";HEIGHT:0.2"
    };
    CHECK(semantic_marker_lines(output) == expected);

    const size_t first_move = first_printable_move_offset(output);
    REQUIRE(first_move != std::string::npos);
    size_t previous_marker = 0;
    for (const std::string &marker : expected) {
        const size_t marker_position = output.find(marker + "\n", previous_marker);
        REQUIRE(marker_position != std::string::npos);
        CHECK(marker_position < first_move);
        previous_marker = marker_position + marker.size() + 1;
    }
}

// Export adjacent equal paths and verify role, width and height remain deduplicated after the layer maximum marker.
TEST_CASE("Default firmware deduplicates identical consecutive printable markers",
          "[plugins][gcode][firmware][markers]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    configure_standard_firmware(print);
    PrintingLayerGroup &layer = append_marker_test_layer(print, scale_i(0.2));
    PrintingToolGroup &tool_group = layer.tool_groups.front();
    append_path_extrusion(
        tool_group,
        marker_test_path(0.0, 1.0, ExtrusionRole::Perimeter, 0.4f, 0.2f));
    append_path_extrusion(
        tool_group,
        marker_test_path(1.0, 2.0, ExtrusionRole::Perimeter, 0.4f, 0.2f));

    const std::string output = export_with_firmware(
        print, "gcode.firmware.marlin2");
    INFO(output);
    const std::vector<std::string> expected{
        ";LAYER_CHANGE", ";Z:0.2", ";MAX_LAYER_HEIGHT:0.2",
        ";TYPE:Perimeter", ";WIDTH:0.4", ";HEIGHT:0.2"
    };
    CHECK(semantic_marker_lines(output) == expected);
    CHECK(count_occurrences(output, ";TYPE:Perimeter\n") == 1);
    CHECK(count_occurrences(output, ";WIDTH:0.4\n") == 1);
    CHECK(count_occurrences(output, ";HEIGHT:0.2\n") == 1);
}

TEST_CASE("Default firmware maps raw extrusion roles to G-code labels")
{
    CHECK(slic3r_api::extrusion_role_to_string(RAW_EXTRUSION_ROLE_NONE) == "Unknown");
    CHECK(slic3r_api::extrusion_role_to_string(RAW_EXTRUSION_ROLE_PERIMETER) == "Perimeter");
    CHECK(slic3r_api::extrusion_role_to_string(RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER) == "External perimeter");
    CHECK(slic3r_api::extrusion_role_to_string(RAW_EXTRUSION_ROLE_OVERHANG_PERIMETER) == "Overhang perimeter");
    CHECK(slic3r_api::extrusion_role_to_string(RAW_EXTRUSION_ROLE_OVERHANG_EXTERNAL_PERIMETER) == "Overhang perimeter");
    CHECK(slic3r_api::extrusion_role_to_string(RAW_EXTRUSION_ROLE_INFILL) == "Internal infill");
    CHECK(slic3r_api::extrusion_role_to_string(RAW_EXTRUSION_ROLE_SOLID_INFILL) == "Solid infill");
    CHECK(slic3r_api::extrusion_role_to_string(RAW_EXTRUSION_ROLE_TOP_SOLID_INFILL) == "Top solid infill");
    CHECK(slic3r_api::extrusion_role_to_string(RAW_EXTRUSION_ROLE_IRONING_INFILL) == "Ironing");
    CHECK(slic3r_api::extrusion_role_to_string(RAW_EXTRUSION_ROLE_BRIDGE_INFILL) == "Bridge infill");
    CHECK(slic3r_api::extrusion_role_to_string(RAW_EXTRUSION_ROLE_INTERNAL_BRIDGE_INFILL) == "Internal bridge infill");
    CHECK(slic3r_api::extrusion_role_to_string(RAW_EXTRUSION_ROLE_THIN_WALL) == "Thin wall");
    CHECK(slic3r_api::extrusion_role_to_string(RAW_EXTRUSION_ROLE_GAP_FILL) == "Gap fill");
    CHECK(slic3r_api::extrusion_role_to_string(RAW_EXTRUSION_ROLE_SKIRT) == "Skirt/Brim");
    CHECK(slic3r_api::extrusion_role_to_string(RAW_EXTRUSION_ROLE_SUPPORT) == "Support material");
    CHECK(slic3r_api::extrusion_role_to_string(RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL_INTERFACE) == "Support material interface");
    CHECK(slic3r_api::extrusion_role_to_string(RAW_EXTRUSION_ROLE_WIPE_TOWER) == "Wipe tower");
    CHECK(slic3r_api::extrusion_role_to_string(RAW_EXTRUSION_ROLE_WIPE_TOWER_RAMMING) == "Wipe tower");
    CHECK(slic3r_api::extrusion_role_to_string(RAW_EXTRUSION_ROLE_WIPE_TOWER_WIPE) == "Wipe tower");
    CHECK(slic3r_api::extrusion_role_to_string(RAW_EXTRUSION_ROLE_MILL) == "Milling");
    CHECK(slic3r_api::extrusion_role_to_string(RAW_EXTRUSION_ROLE_TRAVEL) == "Travel");
}

// Export paths with role, width and height changes and check only changed extrusion tags follow the layer maximum.
TEST_CASE("Default firmware emits only required intra-layer marker transitions",
          "[plugins][gcode][firmware][markers]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    configure_standard_firmware(print);
    PrintingLayerGroup &layer = append_marker_test_layer(print, scale_i(0.2));
    PrintingToolGroup &tool_group = layer.tool_groups.front();
    // Initial printable state.
    append_path_extrusion(
        tool_group,
        marker_test_path(0.0, 1.0, ExtrusionRole::ExternalPerimeter, 0.4f, 0.2f));
    // Width changes while role and height stay unchanged.
    append_path_extrusion(
        tool_group,
        marker_test_path(1.0, 2.0, ExtrusionRole::ExternalPerimeter, 0.45f, 0.2f));
    // Role changes while width and height stay unchanged.
    append_path_extrusion(
        tool_group,
        marker_test_path(2.0, 3.0, ExtrusionRole::InternalInfill, 0.45f, 0.2f));
    // Height changes while role and width stay unchanged.
    append_path_extrusion(
        tool_group,
        marker_test_path(3.0, 4.0, ExtrusionRole::InternalInfill, 0.45f, 0.25f));

    const std::string output = export_with_firmware(
        print, "gcode.firmware.marlin2");
    INFO(output);
    const std::vector<std::string> expected{
        ";LAYER_CHANGE", ";Z:0.2", ";MAX_LAYER_HEIGHT:0.2",
        ";TYPE:External perimeter", ";WIDTH:0.4", ";HEIGHT:0.2",
        ";WIDTH:0.45", ";TYPE:Internal infill", ";HEIGHT:0.25"
    };
    CHECK(semantic_marker_lines(output) == expected);
    CHECK(count_occurrences(output, ";TYPE:External perimeter\n") == 1);
    CHECK(count_occurrences(output, ";TYPE:Internal infill\n") == 1);
    CHECK(count_occurrences(output, ";WIDTH:0.4\n") == 1);
    CHECK(count_occurrences(output, ";WIDTH:0.45\n") == 1);
    CHECK(count_occurrences(output, ";HEIGHT:0.2\n") == 1);
    CHECK(count_occurrences(output, ";HEIGHT:0.25\n") == 1);
}

// Export printable geometry around travel and E-only transitions and check the layer's first HEIGHT remains before geometry.
TEST_CASE("Default firmware retains printable markers across travel and E transitions",
          "[plugins][gcode][firmware][markers]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    configure_standard_firmware(print);
    PrintingLayerGroup &layer = append_marker_test_layer(print, scale_i(0.2));
    PrintingToolGroup &tool_group = layer.tool_groups.front();

    ExtrusionEntity::Children children;
    children.push_back(
        marker_test_path(0.0, 1.0, ExtrusionRole::Perimeter, 0.4f, 0.2f));
    children.push_back(make_firmware_travel(
        ArcPolyline(Points{
            Point(scale_i(1.0), scale_i(0.0)),
            Point(scale_i(2.0), scale_i(0.0))
        }),
        40.f,
        800.f));
    children.push_back(marker_test_retraction(true));
    children.push_back(marker_test_retraction(false));
    children.push_back(
        marker_test_path(2.0, 3.0, ExtrusionRole::Perimeter, 0.4f, 0.2f));
    append_path_extrusion(tool_group, std::unique_ptr<ExtrusionEntity>(
        new ExtrusionEntity(std::move(children), false, true, true)));

    const std::string output = export_with_firmware(
        print, "gcode.firmware.marlin2");
    INFO(output);
    const std::vector<std::string> expected{
        ";LAYER_CHANGE", ";Z:0.2", ";MAX_LAYER_HEIGHT:0.2",
        ";TYPE:Perimeter", ";WIDTH:0.4", ";HEIGHT:0.2"
    };
    CHECK(semantic_marker_lines(output) == expected);
    CHECK(count_occurrences(output, ";TYPE:Perimeter\n") == 1);
    CHECK(count_occurrences(output, ";WIDTH:0.4\n") == 1);
    CHECK(count_occurrences(output, ";HEIGHT:0.2\n") == 1);
}

// Export one layer with and without human comments and require identical semantic marker sequences.
TEST_CASE("Default firmware semantic markers ignore gcode_comments",
          "[plugins][gcode][firmware][markers]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    const std::vector<std::string> expected{
        ";LAYER_CHANGE", ";Z:0.2", ";MAX_LAYER_HEIGHT:0.2",
        ";TYPE:Perimeter", ";WIDTH:0.4", ";HEIGHT:0.2"
    };
    const std::string comments_disabled = export_marker_comment_fixture(false);
    const std::string comments_enabled = export_marker_comment_fixture(true);
    INFO(comments_disabled);
    INFO(comments_enabled);

    const std::vector<std::string> disabled_markers =
        semantic_marker_lines(comments_disabled);
    const std::vector<std::string> enabled_markers =
        semantic_marker_lines(comments_enabled);
    CHECK(disabled_markers == expected);
    CHECK(enabled_markers == expected);
    CHECK(disabled_markers == enabled_markers);
}

TEST_CASE("Default firmware wipe_pair_is_balanced across multiple wipe leaves",
          "[plugins][gcode][firmware][markers][wipe]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    const std::string output = export_wipe_marker_fixture(false, true);
    INFO(output);
    CHECK(wipe_marker_lines(output) ==
          std::vector<std::string>{";WIPE_START", ";WIPE_END"});

    const size_t approach = output.find("G0 X0 Y0 Z.2");
    const size_t wipe_start = output.find(";WIPE_START\n");
    const size_t first_wipe_move = output.find("G1 X1");
    const size_t second_wipe_move = output.find("G1 X2");
    const size_t wipe_end = output.find(";WIPE_END\n");
    const size_t printable_move = first_printable_move_offset(output);
    REQUIRE(approach != std::string::npos);
    REQUIRE(wipe_start != std::string::npos);
    REQUIRE(first_wipe_move != std::string::npos);
    REQUIRE(second_wipe_move != std::string::npos);
    REQUIRE(wipe_end != std::string::npos);
    REQUIRE(printable_move != std::string::npos);
    CHECK(approach < wipe_start);
    CHECK(wipe_start < first_wipe_move);
    CHECK(first_wipe_move < second_wipe_move);
    CHECK(second_wipe_move < wipe_end);
    CHECK(wipe_end < printable_move);
}

TEST_CASE("Default firmware wipe_at_end_is_closed",
          "[plugins][gcode][firmware][markers][wipe]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    const std::string output = export_wipe_marker_fixture(false, false);
    INFO(output);
    CHECK(wipe_marker_lines(output) ==
          std::vector<std::string>{";WIPE_START", ";WIPE_END"});

    const size_t last_wipe_move = output.find("G1 X2");
    const size_t wipe_end = output.find(";WIPE_END\n");
    REQUIRE(last_wipe_move != std::string::npos);
    REQUIRE(wipe_end != std::string::npos);
    CHECK(last_wipe_move < wipe_end);
}

TEST_CASE("Default firmware wipe markers ignore gcode_comments and preserve commands",
          "[plugins][gcode][firmware][markers][wipe]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    const std::string comments_disabled = export_wipe_marker_fixture(false, true);
    const std::string comments_enabled = export_wipe_marker_fixture(true, true);
    INFO(comments_disabled);
    INFO(comments_enabled);
    const std::vector<std::string> expected{";WIPE_START", ";WIPE_END"};
    CHECK(wipe_marker_lines(comments_disabled) == expected);
    CHECK(wipe_marker_lines(comments_enabled) == expected);
    CHECK(machine_command_lines(comments_disabled) ==
          machine_command_lines(comments_enabled));
}

TEST_CASE("Default firmware seam_marks_loop_start using serialized coordinates",
          "[plugins][gcode][firmware][markers][seam]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    configure_standard_firmware(print);
    PrintingLayerGroup &layer = append_marker_test_layer(print, scale_i(0.2));
    std::unique_ptr<ExtrusionPath> path = make_firmware_path(
        ArcPolyline(Points{
            Point::new_scale(4.3216, 5.4321),
            Point::new_scale(6.0, 5.4321),
            Point::new_scale(6.0, 7.0),
            Point::new_scale(4.3216, 5.4321)
        }),
        20.f, 500.f, -1.f, -1.f, -1.f,
        ExtrusionRole::ExternalPerimeter);
    path->get_or_add_property<slic3r_api::EPropertyPerimeter>().shell_count(0);
    append_path_extrusion(layer.tool_groups.front(), std::move(path));

    const std::string output = export_with_firmware(
        print, "gcode.firmware.marlin2");
    INFO(output);
    CHECK(seam_marker_lines(output) ==
          std::vector<std::string>{";SEAM:4.322:5.432"});
    const size_t closing_move = output.find("G1 X4.322 Y5.432 E");
    const size_t seam = output.find(";SEAM:4.322:5.432\n");
    REQUIRE(closing_move != std::string::npos);
    REQUIRE(seam != std::string::npos);
    CHECK(seam < closing_move);
}

TEST_CASE("Default firmware emits one seam after a complete composed arc loop",
          "[plugins][gcode][firmware][markers][seam]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    const std::string output = export_composed_seam_fixture(false, true);
    INFO(output);
    CHECK(seam_marker_lines(output) ==
          std::vector<std::string>{";SEAM:1.235:2.346"});
    CHECK(count_occurrences(output, ";SEAM:") == 1);
    CHECK(output.find("G3 Y6.346 R2 E") != std::string::npos);
    CHECK(output.find(";SEAM:6:") == std::string::npos);

    const size_t closing_move = output.find("G1 X1.235 Y2.346 E");
    const size_t seam = output.find(";SEAM:1.235:2.346\n");
    const size_t following_move = output.find("G1 X2.235 E", seam);
    REQUIRE(closing_move != std::string::npos);
    REQUIRE(seam != std::string::npos);
    REQUIRE(following_move != std::string::npos);
    CHECK(seam < closing_move);
    CHECK(seam < following_move);
    CHECK(count_occurrences(output, ";TYPE:External perimeter\n") == 1);
    CHECK(count_occurrences(output, ";WIDTH:0.4\n") == 1);
    CHECK(count_occurrences(output, ";HEIGHT:0.2\n") == 1);
}

// The writer must serialize the chosen entry, not choose or cut a new seam.
TEST_CASE("Active writer preserves a finalized legal seam after rotation is consumed",
          "[plugins][gcode][firmware][markers][seam][seam-writer-preparation]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    configure_standard_firmware(print);
    PrintingLayerGroup &layer = append_marker_test_layer(print, scale_i(0.2));
    std::unique_ptr<ExtrusionPath> path = make_firmware_path(ArcPolyline(Points{
        Point::new_scale(1, 1), Point::new_scale(9, 1), Point::new_scale(9, 9),
        Point::new_scale(1, 9), Point::new_scale(1, 1)}),
        20.f, 500.f, -1.f, -1.f, -1.f, ExtrusionRole::ExternalPerimeter);
    path->get_or_add_property<slic3r_api::EPropertyPerimeter>().shell_count(0);
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    slic3r_api::MutableExtrusionEntity view(reinterpret_cast<extrusion_entity_handle *>(path.get()));
    const Point chosen = Point::new_scale(9, 5);
    slic3r_api::Ordering::TreeOrderingGeometry::rotate_loop_to_legal_seam(
        handle, view, c_point{chosen.x(), chosen.y()}, true);
    REQUIRE(path->ExtrusionEntity::first_point() == chosen);
    // Simulate the final pass's consumption, not a new exemption in the writer.
    path->set_can_rotate(false);
    append_path_extrusion(layer.tool_groups.front(), std::move(path));
    const ExtrusionEntity *root = layer.tool_groups.front().extrusions.back().root.get();
    const std::string output = export_with_firmware(print, "gcode.firmware.marlin2");
    INFO(output);
    CHECK(seam_marker_lines(output) == std::vector<std::string>{";SEAM:9:5"});
    bool checked_first_extrusion = false;
    GCodeReader reader;
    reader.parse_buffer(output, [&checked_first_extrusion](GCodeReader &state, const GCodeReader::GCodeLine &line) {
        if (!checked_first_extrusion && line.extruding(state) && line.dist_XY(state) > 0.) {
            CHECK(state.x() == 9.f);
            CHECK(state.y() == 5.f);
            checked_first_extrusion = true;
        }
    });
    CHECK(checked_first_extrusion);
    CHECK(root->ExtrusionEntity::first_point() == chosen);
    CHECK_FALSE(root->can_rotate());
}

// Fixed parent entry governs export independently of the child's own permission.
TEST_CASE("Active writer does not recut a closed fixed MultiPath or its canonical copy",
          "[plugins][gcode][firmware][markers][seam][seam-writer-preparation]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    for (const bool canonical : {false, true}) {
        CAPTURE(canonical);
        Print print;
        configure_standard_firmware(print);
        PrintingLayerGroup &layer = append_marker_test_layer(print, scale_i(0.2));
        std::unique_ptr<ExtrusionPath> path = make_firmware_path(ArcPolyline(Points{
            Point::new_scale(9, 9), Point::new_scale(1, 9), Point::new_scale(1, 1),
            Point::new_scale(9, 1), Point::new_scale(9, 9)}),
            20.f, 500.f, -1.f, -1.f, -1.f, ExtrusionRole::ExternalPerimeter);
        std::unique_ptr<ExtrusionMultiPath> fixed = std::make_unique<ExtrusionMultiPath>(*path);
        fixed->get_or_add_property<slic3r_api::EPropertyPerimeter>().shell_count(0);
        std::unique_ptr<ExtrusionEntity> root = canonical ? fixed->clone_canonical() :
                                                          std::unique_ptr<ExtrusionEntity>(std::move(fixed));
        REQUIRE_FALSE(root->can_rotate());
        REQUIRE(root->child(0).can_rotate());
        append_path_extrusion(layer.tool_groups.front(), std::move(root));
        const ExtrusionEntity *before = layer.tool_groups.front().extrusions.back().root.get();
        const std::string output = export_with_firmware(print, "gcode.firmware.marlin2");
        INFO(output);
        CHECK(seam_marker_lines(output) == std::vector<std::string>{";SEAM:9:9"});
        bool checked_first_extrusion = false;
        GCodeReader reader;
        reader.parse_buffer(output, [&checked_first_extrusion](GCodeReader &state, const GCodeReader::GCodeLine &line) {
            if (!checked_first_extrusion && line.extruding(state) && line.dist_XY(state) > 0.) {
                CHECK(state.x() == 9.f);
                CHECK(state.y() == 9.f);
                checked_first_extrusion = true;
            }
        });
        CHECK(checked_first_extrusion);
        CHECK(before->ExtrusionEntity::first_point() == Point::new_scale(9, 9));
        CHECK_FALSE(before->can_rotate());
        CHECK(before->child(0).can_rotate());
    }
}

TEST_CASE("Default firmware omits seam markers for ineligible roots",
          "[plugins][gcode][firmware][markers][seam]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    SECTION("open path") {
        Print print;
        configure_standard_firmware(print);
        PrintingLayerGroup &layer = append_marker_test_layer(print, scale_i(0.2));
        append_path_extrusion(
            layer.tool_groups.front(),
            marker_test_path(0.0, 1.0, ExtrusionRole::Perimeter, 0.4f, 0.2f));
        CHECK(seam_marker_lines(export_with_firmware(
                  print, "gcode.firmware.marlin2")).empty());
    }

    SECTION("empty root") {
        Print print;
        configure_standard_firmware(print);
        PrintingLayerGroup &layer = append_marker_test_layer(print, scale_i(0.2));
        append_path_extrusion(
            layer.tool_groups.front(),
            std::unique_ptr<ExtrusionEntity>(new ExtrusionEntity(true)));
        CHECK(seam_marker_lines(export_with_firmware(
                  print, "gcode.firmware.marlin2")).empty());
    }

    SECTION("closed point without a segment") {
        Print print;
        configure_standard_firmware(print);
        PrintingLayerGroup &layer = append_marker_test_layer(print, scale_i(0.2));
        append_path_extrusion(
            layer.tool_groups.front(),
            make_firmware_path(
                ArcPolyline(Points{Point::new_scale(1.0, 2.0)}),
                20.f, 500.f));
        CHECK(seam_marker_lines(export_with_firmware(
                  print, "gcode.firmware.marlin2")).empty());
    }

    SECTION("sortable non-loop collection") {
        Print print;
        configure_standard_firmware(print);
        PrintingLayerGroup &layer = append_marker_test_layer(print, scale_i(0.2));
        ExtrusionEntity::Children children;
        children.push_back(
            marker_test_path(0.0, 1.0, ExtrusionRole::Perimeter, 0.4f, 0.2f));
        children.push_back(
            marker_test_path(2.0, 3.0, ExtrusionRole::Perimeter, 0.4f, 0.2f));
        append_path_extrusion(
            layer.tool_groups.front(),
            std::unique_ptr<ExtrusionEntity>(
                new ExtrusionEntity(std::move(children), true, true, false)));
        CHECK(seam_marker_lines(export_with_firmware(
                  print, "gcode.firmware.marlin2")).empty());
    }
}

TEST_CASE("Default firmware seam markers ignore gcode_comments and preserve commands",
          "[plugins][gcode][firmware][markers][seam]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    const std::string comments_disabled = export_composed_seam_fixture(false, false);
    const std::string comments_enabled = export_composed_seam_fixture(true, false);
    INFO(comments_disabled);
    INFO(comments_enabled);
    const std::vector<std::string> expected{";SEAM:1.235:2.346"};
    CHECK(seam_marker_lines(comments_disabled) == expected);
    CHECK(seam_marker_lines(comments_enabled) == expected);
    CHECK(machine_command_lines(comments_disabled) ==
          machine_command_lines(comments_enabled));
}

TEST_CASE("Default firmware preserves arcs and computes their extrusion length",
          "[plugins][gcode][firmware]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    select_printing_plan_firmware(print);
    configure_standard_firmware(print);
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.front().layers.front();
    layer.print_z = scale_i(0.2);
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

    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    try {
        Steps::StepGenerateGcode::run_step(Orchestrator::instance(), print, output_path.string());
        const std::string output = read_text_file(output_path);
        const size_t arc_pos = output.find("G3 X10 R5 E");
        REQUIRE(arc_pos != std::string::npos);
        CHECK(output.find("G1 X10") == std::string::npos);

        // A 5 mm radius semicircle is longer than the 10 mm chord, so its E
        // output must exceed the straight-path value of about 0.333 mm.
        const size_t e_pos = output.find(" E", arc_pos);
        REQUIRE(e_pos != std::string::npos);
        CHECK(std::stod(output.substr(e_pos + 2)) > 0.5);
    } catch (...) {
        remove_output_pair(output_path);
        throw;
    }
    remove_output_pair(output_path);
}
TEST_CASE("Default firmware interprets one acceleration field from the movement role",
          "[plugins][gcode][firmware]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    select_printing_plan_firmware(print);
    configure_standard_firmware(print);
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.front().layers.front();
    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool_group = layer.tool_groups.front();
    tool_group.extruder_id = 0;

    // The travel and the following print path carry the same property type.
    // Their effective roles decide whether Marlin receives M204 T or M204 P.
    append_path_extrusion(
        tool_group,
        make_firmware_travel(
            ArcPolyline(Points{Point(scale_i(0.0), scale_i(0.0)), Point(scale_i(10.0), scale_i(0.0))}),
            80.f,
            1200.f));
    append_path_extrusion(
        tool_group,
        make_firmware_path(
            ArcPolyline(Points{Point(scale_i(10.0), scale_i(0.0)), Point(scale_i(11.0), scale_i(0.0))}),
            20.f,
            400.f));

    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    try {
        Steps::StepGenerateGcode::run_step(Orchestrator::instance(), print, output_path.string());
        const std::string output = read_text_file(output_path);
        INFO(output);
        CHECK(count_occurrences(output, "M204 T1200\n") == 1);
        CHECK(count_occurrences(output, "M204 P400\n") == 1);
        CHECK(output.find("G0 X10 F4800\n") != std::string::npos);
        CHECK(output.find("G0 X10 E") == std::string::npos);
        CHECK(output.find("G1 X11 E") != std::string::npos);
    } catch (...) {
        remove_output_pair(output_path);
        throw;
    }
    remove_output_pair(output_path);
}
TEST_CASE("Default firmware writes the E value prepared by the extruder state",
          "[plugins][gcode][firmware]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    select_printing_plan_firmware(print);
    configure_standard_firmware(print);
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.front().layers.front();
    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool_group = layer.tool_groups.front();
    tool_group.extruder_id = 0;

    // Two invisible deltas cross the output precision together. The last
    // command then returns the absolute coordinate to zero, which must remain
    // distinguishable from an absent E word.
    ExtrusionEntity::Children children;
    children.push_back(special_command_entity(ExtrusionPropertySpecialCommand::Code::EXTRUSION, 0.000004));
    children.push_back(special_command_entity(ExtrusionPropertySpecialCommand::Code::EXTRUSION, 0.000004));
    children.push_back(special_command_entity(ExtrusionPropertySpecialCommand::Code::EXTRUSION, -0.000008));
    std::unique_ptr<ExtrusionEntity> root(
        new ExtrusionEntity(std::move(children), false, true, true));
    root->add_property(ExtrusionPropertySpeed(12.f, 300.f));
    append_path_extrusion(tool_group, std::move(root));

    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    try {
        Steps::StepGenerateGcode::run_step(Orchestrator::instance(), print, output_path.string());
        const std::string output = read_text_file(output_path);
        INFO(output);
        CHECK(output.find("G1 E.00001") != std::string::npos);
        CHECK(output.find("G1 E0") != std::string::npos);
        CHECK(count_occurrences(output, "G1 E") == 2);
    } catch (...) {
        remove_output_pair(output_path);
        throw;
    }
    remove_output_pair(output_path);
}

// Check stationary volume conversion with the extrusion multiplier in both E
// modes: emitted E and F, and read-back deposit duration and volume.
TEST_CASE("Default firmware deposits volume at one point in both E modes",
          "[plugins][gcode][firmware][deposit]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool volumetric = GENERATE(false, true);
    Print print;
    print.mutable_printing_plan();
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.use_relative_e_distances.value = true;
    config.use_volumetric_e.value = volumetric;
    config.filament_diameter.set(std::vector<double>{1.75});
    config.extrusion_multiplier.set(std::vector<double>{1.5});
    ScriptStateFirmwareProbe firmware;
    firmware.setup(slic3r_api::Config(ApiHost::to_config_handle(&config)));
    const std::unique_ptr<ExtrusionEntity> select = special_command_entity(
        ExtrusionPropertySpecialCommand::Code::TOOLCHANGE, 0.0);
    firmware.write_event(slic3r_api::ExtrusionEntity(reinterpret_cast<const extrusion_entity_handle *>(select.get())));
    const std::unique_ptr<ExtrusionEntity> leaf = firmware_deposit();
    const std::string output = firmware.write_event(slic3r_api::ExtrusionEntity(
        reinterpret_cast<const extrusion_entity_handle *>(leaf.get())));
    INFO(output);
    const size_t type = output.find(";TYPE:Support material\n");
    const size_t width = output.find(";WIDTH:0.4\n");
    const size_t height = output.find(";HEIGHT:0.2\n");
    const size_t travel = output.find("G0 X1.25 Y2.5");
    const size_t deposit = output.find("G1 E");
    REQUIRE(type != std::string::npos);
    REQUIRE(width != std::string::npos);
    REQUIRE(height != std::string::npos);
    REQUIRE(travel != std::string::npos);
    REQUIRE(deposit != std::string::npos);
    CHECK(type < width);
    CHECK(width < height);
    CHECK(height < travel);
    CHECK(travel < deposit);
    const size_t print_acceleration = output.find("M204 P500\n");
    REQUIRE(print_acceleration != std::string::npos);
    CHECK(travel < print_acceleration);
    CHECK(print_acceleration < deposit);
    CHECK(output.find(";SEAM:") == std::string::npos);
    const std::string move = output.substr(deposit, output.find('\n', deposit) - deposit);
    CHECK(move.find(" X") == std::string::npos);
    CHECK(move.find(" Y") == std::string::npos);
    REQUIRE(move.find(" F") != std::string::npos);
    const double area = PI * 1.75 * 1.75 / 4.0;
    CHECK(std::stod(move.substr(move.find(" F") + 2)) == Approx(60.0 * 20.0 * 0.08 * 1.5 / area).margin(0.001));
    const std::vector<double> e = firmware_move_e_values(output);
    REQUIRE(e.size() == 1);
    CHECK(e.front() == Approx(0.4 * 1.5 / (volumetric ? 1.0 : area)).margin(0.00001));
    CHECK(firmware.machine_extruder(0).extrusion_axis().extruded_volume() == Approx(0.6));

    // The reader uses filament speed for stationary E in either mode. Its
    // duration and role volume therefore match the same flow-equivalent line.
    GCodeReaderService reader(Orchestrator::instance(), &print);
    const GCodeProcessorResult result = reader.read_for_viewer("gcode.reader.marlin2",
        "G90\nM83\nG92 X0 Y0 Z0 E0\n" + output);
    REQUIRE_FALSE(result.moves.empty());
    REQUIRE(result.moves.size() == 2);
    CHECK(result.moves.back().move_time - result.moves.front().move_time ==
          Approx(0.4 / (20.0 * 0.08)).margin(0.0001));
    CHECK(result.print_statistics.volumes_per_role_per_extruder.at(GCodeExtrusionRole::SupportMaterial).at(0) ==
          Approx(0.6).margin(0.0001));
}

// Compare reset-free support deposits and a line with reader volume at each
// precision in both E modes; copied-axis trials also decide the half-step tie.
TEST_CASE("Support deposits retain visible precision and filament accounting",
          "[plugins][gcode][firmware][deposit][deposit-precision]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool klipper = GENERATE(false, true);
    const bool relative = GENERATE(false, true);
    const bool volumetric = GENERATE(false, true);
    const int precision = GENERATE(2, 3, 5);
    INFO("klipper=" << klipper << ", relative=" << relative << ", volumetric=" << volumetric << ", precision=" << precision);
    Print print;
    print.mutable_printing_plan();
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.use_relative_e_distances.value = relative;
    config.use_volumetric_e.value = volumetric;
    config.gcode_precision_e.value = precision;
    config.filament_diameter.set(std::vector<double>{1.75});
    config.extrusion_multiplier.set(std::vector<double>{1.5});
    ScriptStateFirmwareProbe base;
    DepositKlipperFirmwareProbe dialect;
    slic3r_api::GCodeGeneration::DefaultGCodeFirmwareSession &firmware = klipper ?
        static_cast<slic3r_api::GCodeGeneration::DefaultGCodeFirmwareSession &>(dialect) : base;
    firmware.setup(slic3r_api::Config(ApiHost::to_config_handle(&config)));
    const std::unique_ptr<ExtrusionEntity> select = special_command_entity(
        ExtrusionPropertySpecialCommand::Code::TOOLCHANGE, 0.0);
    firmware.write_event(slic3r_api::ExtrusionEntity(reinterpret_cast<const extrusion_entity_handle *>(select.get())));
    slic3r_api::GCodeGeneration::ExtrusionAxisState &axis = klipper ?
        dialect.extruders().at(0).extrusion_axis() : base.machine_extruder(0).extrusion_axis();
    const double step = std::pow(10.0, -precision);
    const double area = PI * 1.75 * 1.75 / 4.0;
    const double filament_per_e = volumetric ? 1.0 / area : 1.0;
    std::string sequence = std::string("G90\n") + (relative ? "M83\n" : "M82\n") + "G92 X0 Y0 Z0 E0\n";
    GCodeReaderService reader(Orchestrator::instance(), &print);
    double previous_read_filament = 0.0;
    double previous_display = 0.0;

    SECTION("round-up, forced step, invisible sub-step and normal line") {
        for (size_t index = 0; index < 4; ++index) {
            const double used_before = axis.used_filament();
            const double remainder_before = axis.extruded_dE_left();
            const double fractions[] = {0.7, 0.6, 0.3, 2.1};
            std::unique_ptr<ExtrusionEntity> leaf;
            if (index < 3) {
                leaf = firmware_deposit(fractions[index] * step / axis.e_per_mm3());
            } else {
                // A real one-millimetre line consumes the carried positive remainder.
                leaf = marker_test_path(1.25, 2.25, ExtrusionRole::SupportMaterial, 0.4f, 0.2f);
                slic3r_api::MutableExtrusionEntity view(reinterpret_cast<extrusion_entity_handle *>(leaf.get()));
                view.get_mutable(slic3r_api::EPropertyAttributes::key)->mm3_per_mm(
                    fractions[index] * step / axis.e_per_mm3());
            }
            const std::string output = firmware.write_event(slic3r_api::ExtrusionEntity(
                reinterpret_cast<const extrusion_entity_handle *>(leaf.get())));
            INFO(output);
            sequence += output;
            const std::vector<double> words = firmware_move_e_values(output);
            if (index == 2) {
                CHECK(words.empty());
                CHECK(output.find("G4 P20\n") != std::string::npos);
            } else {
                REQUIRE(words.size() == 1);
                CHECK(output.find("G4 P20\n") == std::string::npos);
                const double displayed_delta = relative ? words.front() : words.front() - previous_display;
                CHECK(displayed_delta == Approx((index == 3 ? 2.0 : 1.0) * step));
                if (!relative) {
                    CHECK(words.front() >= previous_display);
                    previous_display = words.front();
                }
            }
            const double expected_remainders[] = {-0.3, 0.0, 0.3, 0.4};
            CHECK(axis.extruded_dE_left() == Approx(expected_remainders[index] * step).margin(step * 1e-9));
            CHECK(axis.used_filament() - used_before ==
                  Approx((index == 1 ? 1.3 : fractions[index]) * step * filament_per_e));
            CHECK(axis.position() == Approx(relative ? 0.0 :
                (index == 0 ? 0.7 : index == 1 ? 2.0 : index == 2 ? 2.3 : 4.4) * step));

            // Reader statistics derive volume from displayed E. Without a reset,
            // their difference from exact usage is precisely the carried remainder.
            const GCodeProcessorResult result = reader.read_for_viewer(
                klipper ? "gcode.reader.klipper" : "gcode.reader.marlin2", sequence);
            const auto role_volume = result.print_statistics.volumes_per_role_per_extruder.find(
                GCodeExtrusionRole::SupportMaterial);
            const bool has_volume = role_volume != result.print_statistics.volumes_per_role_per_extruder.end() &&
                role_volume->second.count(0) == 1;
            const double read_filament = has_volume ? role_volume->second.at(0) / area : 0.0;
            CHECK(std::abs(axis.extruded_dE_left() - remainder_before) < 0.5 * step);
            // Accepted reader limitation (GCodeReaderResultBuilder.cpp:808-818): a
            // stationary E move of at most EPSILON (1e-4 mm of filament) is ignored.
            // The writer still counts such deposits, so they are missing from
            // read-back statistics; the writer-side assertions above are unchanged.
            constexpr double reader_epsilon = 1e-4;
            const bool reader_sees_steps = step * filament_per_e > reader_epsilon;
            if (reader_sees_steps) {
                // An absent statistic means the reader counted zero volume; keep the
                // failed assertion visible while checking the rest of the sequence.
                CHECK(has_volume);
                CHECK(axis.used_filament() - used_before == Approx(read_filament - previous_read_filament)
                    .margin(0.5 * step * filament_per_e));
                CHECK(axis.used_filament() == Approx(read_filament).margin(0.5 * step * filament_per_e));
                CHECK(axis.used_filament() - read_filament ==
                    Approx(axis.extruded_dE_left() * filament_per_e).margin(step * filament_per_e * 1e-5));
                if (index == 1)
                    CHECK(axis.used_filament() == Approx(read_filament));
            } else if (index < 3) {
                // The reader omits the stationary deposits (E-only moves).
                CHECK(read_filament == Approx(0.0).margin(1e-12));
            } else {
                // The reader counts the printing line (it has an XY change): its
                // word carries the +0.3 step remainder, i.e. two steps.
                CHECK(has_volume);
                CHECK(read_filament - previous_read_filament ==
                    Approx(2.0 * step * filament_per_e).margin(0.1 * step * filament_per_e));
            }
            previous_read_filament = read_filament;
        }
    }
    SECTION("the half-step tie follows the axis trial") {
        const std::unique_ptr<ExtrusionEntity> first = firmware_deposit(0.7 * step / axis.e_per_mm3());
        firmware.write_event(slic3r_api::ExtrusionEntity(reinterpret_cast<const extrusion_entity_handle *>(first.get())));
        const double remainder = axis.extruded_dE_left();
        const double volume = (0.5 * step - remainder) / axis.e_per_mm3();
        const double delta = volume * axis.e_per_mm3();
        CHECK(remainder + delta == Approx(0.5 * step));
        slic3r_api::GCodeGeneration::ExtrusionAxisState trial = axis;
        std::optional<double> expected_word = trial.extrude(delta);
        if (!expected_word) {
            trial = axis;
            expected_word = trial.extrude(step - remainder);
        }
        const std::unique_ptr<ExtrusionEntity> tie = firmware_deposit(volume);
        const std::string output = firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(tie.get())));
        const std::vector<double> words = firmware_move_e_values(output);
        REQUIRE(expected_word);
        REQUIRE(words.size() == 1);
        CHECK(words.front() == Approx(*expected_word));
        CHECK(axis.used_filament() == Approx(trial.used_filament()));
        CHECK(axis.position() == Approx(trial.position()));
        CHECK(axis.extruded_dE_left() == Approx(trial.extruded_dE_left()).margin(step * 1e-9));
        CHECK(output.find("G4 P20\n") == std::string::npos);
    }
}

TEST_CASE("Deposits follow planned unretraction after tool selection",
          "[plugins][gcode][firmware][deposit][retraction]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool native = GENERATE(false, true);
    const bool volumetric = GENERATE(false, true);
    Print print;
    print.mutable_printing_plan();
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.nozzle_diameter.set(std::vector<double>{0.4, 0.4});
    config.filament_diameter.set(std::vector<double>{1.75, 1.75});
    config.extrusion_multiplier.set(std::vector<double>{1.5, 1.5});
    config.use_relative_e_distances.value = true;
    config.use_volumetric_e.value = volumetric;
    config.use_firmware_retraction.value = native;
    config.retract_speed.set(std::vector<double>{40.0, 40.0});
    config.deretract_speed.set(std::vector<double>{30.0, 30.0});
    ScriptStateFirmwareProbe firmware;
    firmware.setup(slic3r_api::Config(ApiHost::to_config_handle(&config)));
    for (const double id : {0.0, 1.0}) {
        const std::unique_ptr<ExtrusionEntity> tool = special_command_entity(ExtrusionPropertySpecialCommand::Code::TOOLCHANGE, id);
        firmware.write_event(slic3r_api::ExtrusionEntity(reinterpret_cast<const extrusion_entity_handle *>(tool.get())));
    }
    const std::unique_ptr<ExtrusionEntity> retract = marker_test_retraction(true);
    const std::string retract_output = firmware.write_event(slic3r_api::ExtrusionEntity(
        reinterpret_cast<const extrusion_entity_handle *>(retract.get())));
    const std::unique_ptr<ExtrusionEntity> unretract = marker_test_retraction(false);
    slic3r_api::MutableExtrusionEntity unretract_view(reinterpret_cast<extrusion_entity_handle *>(unretract.get()));
    unretract_view.get_mutable(slic3r_api::EPropertyAttributes::key)->extrusion_role(
        RAW_EXTRUSION_ROLE_UNRETRACT | RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL);
    unretract_view.get_mutable(slic3r_api::EPropertyExtrusionAxis::key)->unretract(0.05, true);
    const std::string unretract_output = firmware.write_event(unretract_view.readonly());
    slic3r_api::GCodeGeneration::ExtrusionAxisState &axis = firmware.machine_extruder(1).extrusion_axis();
    CHECK(axis.retracted() == 0.0);
    if (native) {
        CHECK(retract_output.find("G10\n") != std::string::npos);
        CHECK(unretract_output.find("G11\n") != std::string::npos);
    } else {
        CHECK(unretract_output.find(";TYPE:Support material\n") < unretract_output.find("G1 E"));
    }
    const double before = axis.extruded_volume();
    const std::unique_ptr<ExtrusionEntity> leaf = firmware_deposit();
    const std::string output = firmware.write_event(slic3r_api::ExtrusionEntity(
        reinterpret_cast<const extrusion_entity_handle *>(leaf.get())));
    CHECK(axis.extruded_volume() - before == Approx(0.6));
    CHECK(axis.retracted() == 0.0);
    // Isolate the deposit move for read-back so restart_extra is not included
    // in its volume or duration, and establish the marker if unretraction emitted it.
    GCodeReaderService reader(Orchestrator::instance(), &print);
    const GCodeProcessorResult result = reader.read_for_viewer("gcode.reader.marlin2",
        "G90\nM83\nT1\nG92 X0 Y0 Z0 E0\n;TYPE:Support material\n" + output);
    REQUIRE_FALSE(result.moves.empty());
    REQUIRE(result.moves.size() == 3);
    CHECK(result.moves.front().type == EMoveType::Tool_change);
    CHECK(result.moves[1].type == EMoveType::Travel);
    CHECK(result.moves.back().type == EMoveType::Unretract);
    CHECK(result.moves.back().move_time - result.moves[1].move_time == Approx(0.25).margin(0.0001));
    CHECK(result.print_statistics.volumes_per_role_per_extruder.at(GCodeExtrusionRole::SupportMaterial).at(1) ==
          Approx(0.6).margin(0.0001));
}

TEST_CASE("Default firmware rejects malformed stationary deposits",
          "[plugins][gcode][firmware][deposit]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    ScriptStateFirmwareProbe firmware;
    std::unique_ptr<ExtrusionEntity> leaf = firmware_deposit();
    slic3r_api::MutableExtrusionEntity view(reinterpret_cast<extrusion_entity_handle *>(leaf.get()));
    bool select_tool = true;
    SECTION("no point") { REQUIRE(view.clear_polyline()); }
    SECTION("two points") { REQUIRE(view.set_points(std::vector<c_point>{{0, 0}, {scale_i(1.), 0}})); }
    SECTION("children") {
        REQUIRE(view.clear_content());
        PluginStorage storage;
        slic3r_api::StoredExtrusionEntity child(reinterpret_cast<storage_handle *>(&storage));
        REQUIRE(view.append_child_copy(child.readonly()) != EXTRUSION_INDEX_INVALID);
    }
    SECTION("axis request") { view.get_or_add(slic3r_api::EPropertyExtrusionAxis::key).retract_to(0.8, false); }
    SECTION("zero volume") { view.get_mutable(slic3r_api::EPropertySpecialCommand::key)->extra_data = 0.0; }
    SECTION("negative volume") { view.get_mutable(slic3r_api::EPropertySpecialCommand::key)->extra_data = -1.0; }
    SECTION("non-finite volume") { view.get_mutable(slic3r_api::EPropertySpecialCommand::key)->extra_data = std::numeric_limits<double>::infinity(); }
    SECTION("no speed") { leaf->remove_property<ExtrusionPropertySpeed>(); }
    SECTION("no tool") { select_tool = false; }
    SECTION("invalid volumetric filament area") {
        config.set_deserialize("use_volumetric_e", "1");
        config.set_deserialize("filament_diameter", "0");
    }
    SECTION("travel role") { view.get_mutable(slic3r_api::EPropertyAttributes::key)->extrusion_role(RAW_EXTRUSION_ROLE_TRAVEL); }
    SECTION("zero flow") { view.get_mutable(slic3r_api::EPropertyAttributes::key)->mm3_per_mm(0.0); }
    SECTION("zero width") { view.get_mutable(slic3r_api::EPropertyAttributes::key)->width(0.f); }
    SECTION("zero height") { view.get_mutable(slic3r_api::EPropertyAttributes::key)->height(0.f); }
    firmware.setup(slic3r_api::Config(ApiHost::to_config_handle(&config)));
    if (select_tool) {
        const std::unique_ptr<ExtrusionEntity> select = special_command_entity(
            ExtrusionPropertySpecialCommand::Code::TOOLCHANGE, 0.0);
        firmware.write_event(slic3r_api::ExtrusionEntity(reinterpret_cast<const extrusion_entity_handle *>(select.get())));
    }
    CHECK_THROWS_AS(firmware.write_event(view.readonly()), std::invalid_argument);
}

TEST_CASE("STEP_GCODE pre-validation checks stationary deposit requirements",
          "[plugins][gcode][validation][deposit]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    PrintingToolGroup &tool = append_speed_validation_tool_group(print, scale_i(0.42), 0);
    std::unique_ptr<ExtrusionEntity> leaf = firmware_deposit();
    slic3r_api::MutableExtrusionEntity view(reinterpret_cast<extrusion_entity_handle *>(leaf.get()));
    bool valid = false;
    bool event_scope = false;
    SECTION("valid leaf") { valid = true; }
    SECTION("deposit in a tool-group event scope") { event_scope = true; }
    SECTION("inherited attributes and speed") {
        leaf->remove_property<ExtrusionAttributes>();
        leaf->remove_property<ExtrusionPropertySpeed>();
        ExtrusionEntity::Children children;
        children.push_back(std::move(leaf));
        leaf.reset(new ExtrusionEntity(std::move(children), false, false, true));
        leaf->add_property(ExtrusionAttributes(ExtrusionRole::SupportMaterial, ExtrusionFlow(0.08, 0.4f, 0.2f)));
        leaf->add_property(ExtrusionPropertySpeed(20.f));
        valid = true;
    }
    SECTION("no point") { REQUIRE(view.clear_polyline()); }
    SECTION("two points") { REQUIRE(view.set_points(std::vector<c_point>{{0, 0}, {scale_i(1.), 0}})); }
    SECTION("children") {
        REQUIRE(view.clear_content());
        leaf->append_child(std::unique_ptr<ExtrusionEntity>(new ExtrusionNop()));
    }
    SECTION("axis") { view.get_or_add(slic3r_api::EPropertyExtrusionAxis::key).retract_to(0.8, false); }
    SECTION("no speed") { leaf->remove_property<ExtrusionPropertySpeed>(); }
    SECTION("non-finite speed") { view.get_mutable(slic3r_api::EPropertySpeed::key)->speed_mm_per_s = std::numeric_limits<float>::quiet_NaN(); }
    SECTION("invalid tool") { tool.extruder_id = 3; }
    SECTION("non-positive volume") { view.get_mutable(slic3r_api::EPropertySpecialCommand::key)->extra_data = 0.0; }
    SECTION("non-finite volume") { view.get_mutable(slic3r_api::EPropertySpecialCommand::key)->extra_data = std::numeric_limits<double>::infinity(); }
    SECTION("no attributes") { leaf->remove_property<ExtrusionAttributes>(); }
    SECTION("zero flow") { view.get_mutable(slic3r_api::EPropertyAttributes::key)->mm3_per_mm(0.0); }
    SECTION("zero width") { view.get_mutable(slic3r_api::EPropertyAttributes::key)->width(0.f); }
    SECTION("zero height") { view.get_mutable(slic3r_api::EPropertyAttributes::key)->height(0.f); }
    SECTION("travel") { view.get_mutable(slic3r_api::EPropertyAttributes::key)->extrusion_role(RAW_EXTRUSION_ROLE_TRAVEL); }
    SECTION("wipe") { view.get_mutable(slic3r_api::EPropertyAttributes::key)->extrusion_role(RAW_EXTRUSION_ROLE_WIPE); }
    SECTION("retract") { view.get_mutable(slic3r_api::EPropertyAttributes::key)->extrusion_role(RAW_EXTRUSION_ROLE_RETRACT); }
    SECTION("unretract") { view.get_mutable(slic3r_api::EPropertyAttributes::key)->extrusion_role(RAW_EXTRUSION_ROLE_UNRETRACT); }
    SECTION("unsupported printable role") { view.get_mutable(slic3r_api::EPropertyAttributes::key)->extrusion_role(RAW_EXTRUSION_ROLE_PERIMETER | RAW_EXTRUSION_ROLE_INFILL); }
    if (event_scope)
        tool.events.append_before(*leaf);
    else
        append_speed_validation_extrusion(tool, std::move(leaf));
    std::string error;
    CHECK(Steps::StepGenerateGcode::validate_pre(Orchestrator::instance(), print, &error) == valid);
    if (!valid)
        CHECK(error.find("DEPOSIT needs") != std::string::npos);
    if (event_scope) {
        CHECK(error.find("events.before") != std::string::npos);
        CHECK(error.find("rather than an event scope") != std::string::npos);
    }
}

// Exercise signed fixed filament lengths without startup commands or file publication.
TEST_CASE("Default firmware distributes geometric EXTRUSION over planar segments",
          "[plugins][gcode][firmware][retraction][fixed-e]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool relative_e = GENERATE(false, true);
    const bool negative = GENERATE(false, true);
    const bool arc = GENERATE(false, true);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize("use_relative_e_distances", relative_e ? "1" : "0");
    config.set_deserialize("extrusion_multiplier", "2");
    ScriptStateFirmwareProbe firmware;
    firmware.setup(slic3r_api::Config(ApiHost::to_config_handle(&config)));
    std::unique_ptr<ExtrusionEntity> select = special_command_entity(
        ExtrusionPropertySpecialCommand::Code::TOOLCHANGE, 0.0);
    firmware.write_event(slic3r_api::ExtrusionEntity(
        reinterpret_cast<const extrusion_entity_handle *>(select.get())));
    slic3r_api::GCodeGeneration::ExtrusionAxisState &axis = firmware.machine_extruder(0).extrusion_axis();
    axis.set_retracted(negative ? 0.0 : 2.0, 0.0);

    Geometry::ArcWelder::Path segments;
    segments.emplace_back(Point::new_scale(0.0, 0.0));
    segments.emplace_back(Point::new_scale(1.0, 0.0));
    if (arc)
        segments.emplace_back(Point::new_scale(3.0, 0.0), float(scale_i(1.0)),
                              Geometry::ArcWelder::Orientation::CCW);
    else
        segments.emplace_back(Point::new_scale(3.0, 0.0));
    segments.emplace_back(Point::new_scale(7.0, 0.0));
    std::unique_ptr<ExtrusionPath> path = make_firmware_path(ArcPolyline(segments), 17.f, 500.f);
    slic3r_api::MutableExtrusionEntity view(reinterpret_cast<extrusion_entity_handle *>(path.get()));
    view.get_mutable(slic3r_api::EPropertyAttributes::key)->width(0.f).height(0.f).mm3_per_mm(0.0);
    const double delta = negative ? -1.00001 : 1.00001;
    view.get_or_add(slic3r_api::EPropertySpecialCommand::key).set(C_EXTRUSION_SPECIAL_COMMAND_EXTRUSION, delta);
    const std::string output = firmware.write_event(slic3r_api::ExtrusionEntity(
        reinterpret_cast<const extrusion_entity_handle *>(path.get())));
    INFO(output);
    const std::vector<double> values = firmware_move_e_values(output);
    REQUIRE(values.size() == 3);
    const double total_length = 5.0 + (arc ? PI : 2.0);
    CHECK(values[0] == Approx(delta / total_length).margin(0.00001));
    CHECK((relative_e ? values[1] : values[1] - values[0]) ==
          Approx(delta * (arc ? PI : 2.0) / total_length).margin(0.00001));
    CHECK((relative_e ? values[2] : values[2] - values[1]) ==
          Approx(delta * 4.0 / total_length).margin(0.00001));
    CHECK((relative_e ? values[0] + values[1] + values[2] : values[2]) == Approx(delta).margin(1e-12));
    // A fixed-E EXTRUSION is a raw filament move: it leaves the retraction state
    // exactly as the caller configured it.
    CHECK(axis.retracted() == Approx(negative ? 0.0 : 2.0));
    CHECK(output.find(";TYPE:Perimeter\n") != std::string::npos);
    CHECK(output.find(";WIDTH:") == std::string::npos);
    CHECK(output.find(";HEIGHT:") == std::string::npos);
    CHECK(output.find("F1020") != std::string::npos);
    CHECK(count_occurrences(output, "G1 E") == 0);
    if (arc)
        CHECK(output.find("G3 X3 R1 E") != std::string::npos);
}

// Logical filament selection transfers all live physical state but retains destination settings.
TEST_CASE("Default firmware shares complete extruder runtime state only in SEMM",
          "[plugins][gcode][firmware][state][semm]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool semm = GENERATE(false, true);
    const bool relative_e = GENERATE(false, true);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize("single_extruder_multi_material", semm ? "1" : "0");
    config.set_deserialize("use_relative_e_distances", relative_e ? "1" : "0");
    config.set_deserialize("filament_diameter", "1.75,2.85");
    config.set_deserialize("extrusion_multiplier", "1,2");
    config.set_deserialize("retract_speed", "40,55");
    config.set_deserialize("deretract_speed", "18,23");
    config.set_deserialize("extruder_temperature_offset", "0,5");
    config.set_deserialize("extruder_fan_offset", "0,10");
    ScriptStateFirmwareProbe firmware;
    firmware.setup(slic3r_api::Config(ApiHost::to_config_handle(&config)));
    std::unique_ptr<ExtrusionEntity> select = special_command_entity(
        ExtrusionPropertySpecialCommand::Code::TOOLCHANGE, 0.0);
    firmware.write_event(slic3r_api::ExtrusionEntity(
        reinterpret_cast<const extrusion_entity_handle *>(select.get())));
    slic3r_api::GCodeGeneration::DefaultExtruder &first = firmware.machine_extruder(0);
    first.heater().request_temperature(200);
    first.heater().mark_encoded_with_wait();
    first.fan().request_speed_percent(50.0);
    first.fan().mark_encoded();
    first.pressure_advance().request(0.04);
    first.pressure_advance().mark_encoded();
    first.extrusion_axis().extrude(0.000004);
    first.extrusion_axis().retract(2.0);
    first.extrusion_axis().schedule_restart_extra(0.1, false);
    first.extrusion_axis().schedule_restart_extra(0.2, true);

    std::unique_ptr<ExtrusionEntity> next = special_command_entity(
        ExtrusionPropertySpecialCommand::Code::TOOLCHANGE, 1.0);
    CHECK(firmware.write_event(slic3r_api::ExtrusionEntity(
        reinterpret_cast<const extrusion_entity_handle *>(next.get()))) ==
        (relative_e ? "T1\n" : "T1\nG92 E0\n"));
    slic3r_api::GCodeGeneration::DefaultExtruder &second = firmware.machine_extruder(1);
    CHECK(second.id() == 1);
    CHECK(second.extrusion_axis().filament_diameter() == Approx(2.85));
    CHECK(second.extrusion_axis().e_per_mm3() == Approx(2.0 / (PI * 2.85 * 2.85 / 4.0)));
    CHECK(second.extrusion_axis().retract_speed() == Approx(55.0));
    CHECK(second.extrusion_axis().deretract_speed() == Approx(23.0));
    if (semm) {
        // Selecting another SEMM filament copies the persistent heater request;
        // only inherited process values are cleared at the end of a node.
        CHECK(second.heater().requested_temperature() == 200);
        CHECK(second.heater().encoded_temperature() == 200);
        CHECK(second.heater().encoded_temperature_with_wait() == 200);
        second.heater().request_temperature(200);
        CHECK(second.heater().effective_temperature() == 205);
        CHECK_FALSE(second.fan().requested_speed_percent());
        CHECK(second.fan().encoded_speed_percent() == 50.0);
        second.fan().request_speed_percent(50.0);
        CHECK(second.fan().effective_speed_percent() == 60.0);
        CHECK_FALSE(second.pressure_advance().requested());
        CHECK(second.pressure_advance().encoded() == 0.04);
        CHECK(second.extrusion_axis().position() == 0.0);
        CHECK(second.extrusion_axis().extruded_dE_left() ==
              (relative_e ? first.extrusion_axis().extruded_dE_left() : 0.0));
        CHECK(second.extrusion_axis().retracted() == Approx(2.0));
        CHECK(second.extrusion_axis().restart_extra() == Approx(0.1));
        CHECK(second.extrusion_axis().used_filament() == first.extrusion_axis().used_filament());

        std::unique_ptr<ExtrusionEntity> full = marker_test_retraction(false);
        full->add_property(ExtrusionPropertyHeater(
            C_EXTRUSION_HEATER_TOOL, EXTRUDER_ID_UNKNOWN, 200, false));
        full->get_or_add_property<slic3r_api::EPropertyExtrusionAxis>().unretract(0.1);
        const std::string output = firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(full.get())));
        INFO(output);
        CHECK(output.find("M104 S205\n") != std::string::npos);
        const std::vector<double> values = firmware_move_e_values(output);
        REQUIRE(values.size() == 1);
        CHECK(values.front() == Approx(2.3));
        CHECK(output.find("F1380") != std::string::npos);
        CHECK_FALSE(second.extrusion_axis().need_unretract());
        firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(select.get())));
        CHECK(first.extrusion_axis().position() == 0.0);
        CHECK_FALSE(first.extrusion_axis().need_unretract());
    } else {
        CHECK_FALSE(second.heater().requested_temperature());
        CHECK_FALSE(second.fan().requested_speed_percent());
        CHECK_FALSE(second.pressure_advance().requested());
        CHECK(second.extrusion_axis().retracted() == 0.0);
        CHECK(second.extrusion_axis().position() == 0.0);
        CHECK(second.extrusion_axis().extruded_dE_left() == 0.0);
        CHECK(first.extrusion_axis().retracted() == Approx(2.0));
    }
}

// A native retraction remains active when SEMM selects another filament on the same extruder.
TEST_CASE("Default firmware shares native retraction protocol state in SEMM",
          "[plugins][gcode][firmware][retraction][semm]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize("single_extruder_multi_material", "1");
    config.set_deserialize("filament_diameter", "1.75,1.75");
    config.set_deserialize("use_firmware_retraction", "1");
    ScriptStateFirmwareProbe firmware;
    firmware.setup(slic3r_api::Config(ApiHost::to_config_handle(&config)));
    std::unique_ptr<ExtrusionEntity> select = special_command_entity(
        ExtrusionPropertySpecialCommand::Code::TOOLCHANGE, 0.0);
    firmware.write_event(slic3r_api::ExtrusionEntity(
        reinterpret_cast<const extrusion_entity_handle *>(select.get())));
    std::unique_ptr<ExtrusionEntity> retract = marker_test_retraction(true);
    retract->get_or_add_property<slic3r_api::EPropertyExtrusionAxis>().retract_to(2.0);
    CHECK(firmware.write_event(slic3r_api::ExtrusionEntity(
        reinterpret_cast<const extrusion_entity_handle *>(retract.get()))) == "G10\n");
    std::unique_ptr<ExtrusionEntity> next = special_command_entity(
        ExtrusionPropertySpecialCommand::Code::TOOLCHANGE, 1.0);
    CHECK(firmware.write_event(slic3r_api::ExtrusionEntity(
        reinterpret_cast<const extrusion_entity_handle *>(next.get()))) == "T1\nG92 E0\n");
    retract->get_or_add_property<slic3r_api::EPropertyExtrusionAxis>().retract_to(3.0);
    CHECK(firmware.write_event(slic3r_api::ExtrusionEntity(
        reinterpret_cast<const extrusion_entity_handle *>(retract.get()))).empty());
    std::unique_ptr<ExtrusionEntity> full = marker_test_retraction(false);
    CHECK(firmware.write_event(slic3r_api::ExtrusionEntity(
        reinterpret_cast<const extrusion_entity_handle *>(full.get()))) == "G11\n");
    firmware.write_event(slic3r_api::ExtrusionEntity(
        reinterpret_cast<const extrusion_entity_handle *>(select.get())));
    CHECK(firmware.write_event(slic3r_api::ExtrusionEntity(
        reinterpret_cast<const extrusion_entity_handle *>(retract.get()))) == "G10\n");
}

// Compare the first approach in the machine frame, including tool and printer Z corrections.
TEST_CASE("First approach transforms and clamps the absolute travel floor",
          "[plugins][gcode][firmware][lift][lift-min-frame]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool known = GENERATE(false, true);
    const double z_offset = GENERATE(-1.0, 1.0);
    const double tool_offset = GENERATE(-0.5, 0.5);
    const double minimum = GENERATE(5.0, 12.0);
    CAPTURE(known, z_offset, tool_offset, minimum);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"lift_min", std::to_string(minimum)}, {"max_print_height", "10"},
        {"z_offset", std::to_string(z_offset)}});
    ScriptStateFirmwareProbe firmware;
    firmware.setup(slic3r_api::Config(ApiHost::to_config_handle(&config)));
    struct ToolOffset : slic3r_api::GCodeGeneration::Tool {
        // Set only the protected correction for this test; no production setting exposes it.
        static void set(slic3r_api::GCodeGeneration::Tool &tool, double value) {
            tool.*(&ToolOffset::m_z_offset) = value;
        }
    };
    ToolOffset::set(firmware.machine_extruder(0), tool_offset);
    std::unique_ptr<ExtrusionEntity> select = special_command_entity(ExtrusionPropertySpecialCommand::Code::TOOLCHANGE, 0.0);
    firmware.write_event(slic3r_api::ExtrusionEntity(reinterpret_cast<const extrusion_entity_handle *>(select.get())));
    const double correction = z_offset - tool_offset;
    if (known)
        const_cast<slic3r_api::GCodeGeneration::Gantry &>(firmware.machine_gantry()).set_position({0.0, 0.0, correction});
    std::unique_ptr<ExtrusionPath> path = make_firmware_path(ArcPolyline(Points{
        Point(scale_i(20.), 0), Point(scale_i(21.), 0)}), 20.f, 500.f);
    const std::string output = firmware.write_event(slic3r_api::ExtrusionEntity(
        reinterpret_cast<const extrusion_entity_handle *>(path.get())));
    const double expected = std::min(minimum, 10.0) + correction;
    double z = correction, x = 0.0;
    bool approached = false;
    std::istringstream lines(output);
    std::string line;
    while (std::getline(lines, line)) {
        if (line.rfind("G1 ", 0) != 0 && line.rfind("G0 ", 0) != 0) continue;
        double next_z = z, next_x = x;
        std::istringstream words(line.substr(3));
        std::string word;
        while (words >> word) {
            if (word[0] == ';') break;
            if (word[0] == 'Z') next_z = std::stod(word.substr(1));
            if (word[0] == 'X') next_x = std::stod(word.substr(1));
        }
        CHECK(next_z <= 10.0 + correction + 0.001);
        if (!approached && next_x != x) {
            CHECK(z == Approx(expected));
            CHECK(next_z == Approx(expected));
            approached = true;
        }
        z = next_z; x = next_x;
    }
    CHECK(approached);
}

// Coordinate resets never repay the physical retraction or consume its pending restart extras.
TEST_CASE("Tool selection resets absolute E without changing retraction accounting",
          "[plugins][gcode][firmware][retraction][toolchange-reset]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool semm = GENERATE(false, true);
    const bool relative_e = GENERATE(false, true);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize("single_extruder_multi_material", semm ? "1" : "0");
    config.set_deserialize("use_relative_e_distances", relative_e ? "1" : "0");
    config.set_deserialize("filament_diameter", "1.75,1.75");
    ScriptStateFirmwareProbe firmware;
    firmware.setup(slic3r_api::Config(ApiHost::to_config_handle(&config)));
    std::unique_ptr<ExtrusionEntity> select = special_command_entity(
        ExtrusionPropertySpecialCommand::Code::TOOLCHANGE, 0.0);
    CHECK(firmware.write_event(slic3r_api::ExtrusionEntity(
        reinterpret_cast<const extrusion_entity_handle *>(select.get()))) ==
        (relative_e ? "T0\n" : "T0\nG92 E0\n"));
    slic3r_api::GCodeGeneration::ExtrusionAxisState &first = firmware.machine_extruder(0).extrusion_axis();
    first.extrude(10.0);
    first.retract(2.0);
    first.schedule_restart_extra(0.1, false);
    first.schedule_restart_extra(0.2, true);
    const double used_before = first.used_filament();
    std::unique_ptr<ExtrusionEntity> next = special_command_entity(
        ExtrusionPropertySpecialCommand::Code::TOOLCHANGE, 1.0);
    CHECK(firmware.write_event(slic3r_api::ExtrusionEntity(
        reinterpret_cast<const extrusion_entity_handle *>(next.get()))) ==
        (relative_e ? "T1\n" : "T1\nG92 E0\n"));
    CHECK(firmware.machine_extruder(1).extrusion_axis().position() == 0.0);
    CHECK(firmware.machine_extruder(1).extrusion_axis().retracted() == (semm ? 2.0 : 0.0));
    CHECK(firmware.write_event(slic3r_api::ExtrusionEntity(
        reinterpret_cast<const extrusion_entity_handle *>(select.get()))) ==
        (relative_e ? "T0\n" : "T0\nG92 E0\n"));
    CHECK(first.position() == 0.0);
    CHECK(first.retracted() == 2.0);
    CHECK(first.restart_extra() == Approx(0.1));
    CHECK(first.used_filament() == used_before);
    // Selecting the already active tool emits nothing and does not reset E.
    // A raw filament move changes the coordinate but not the retraction state;
    // only the explicit operation below extends the retraction.
    first.extrude(-1.0);
    const double retained_position = first.position();
    CHECK(firmware.write_event(slic3r_api::ExtrusionEntity(
        reinterpret_cast<const extrusion_entity_handle *>(select.get()))).empty());
    CHECK(first.position() == retained_position);
    CHECK(first.retracted() == 2.0);
    first.add_retraction(1.0);
    CHECK(first.retracted() == 3.0);
    std::unique_ptr<ExtrusionEntity> full = marker_test_retraction(false);
    full->get_or_add_property<slic3r_api::EPropertyExtrusionAxis>().unretract(0.1);
    const std::string output = firmware.write_event(slic3r_api::ExtrusionEntity(
        reinterpret_cast<const extrusion_entity_handle *>(full.get())));
    const std::vector<double> values = firmware_move_e_values(output);
    REQUIRE(values.size() == 1);
    CHECK(values.front() == Approx(relative_e ? 3.3 : 2.3));
    CHECK_FALSE(first.need_unretract());
    CHECK(first.used_filament() == Approx(used_before + 0.3));
}

// Point-less and parent EXTRUSION remain entry events; invalid geometric requests fail.
TEST_CASE("Default firmware keeps EXTRUSION entry events and rejects conflicting geometry",
          "[plugins][gcode][firmware][fixed-e]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    ScriptStateFirmwareProbe firmware;
    firmware.setup(slic3r_api::Config(ApiHost::to_config_handle(&config)));
    std::unique_ptr<ExtrusionEntity> select = special_command_entity(
        ExtrusionPropertySpecialCommand::Code::TOOLCHANGE, 0.0);
    firmware.write_event(slic3r_api::ExtrusionEntity(
        reinterpret_cast<const extrusion_entity_handle *>(select.get())));

    SECTION("point-less leaf") {
        std::unique_ptr<ExtrusionEntity> event = special_command_entity(
            ExtrusionPropertySpecialCommand::Code::EXTRUSION, -2.0);
        event->add_property(ExtrusionPropertySpeed(12.f, 300.f));
        const std::string output = firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(event.get())));
        CHECK(output.find("G1 E-2 F720\n") != std::string::npos);
        // The E-only EXTRUSION is raw: it does not add retraction.
        CHECK(firmware.machine_extruder(0).extrusion_axis().retracted() == 0.0);
    }
    SECTION("parent") {
        ExtrusionEntity::Children children;
        children.push_back(std::unique_ptr<ExtrusionNop>(new ExtrusionNop()));
        ExtrusionEntity parent(std::move(children), false, true, true);
        parent.add_property(ExtrusionPropertySpeed(12.f, 300.f));
        parent.add_property(ExtrusionPropertySpecialCommand(
            ExtrusionPropertySpecialCommand::Code::EXTRUSION, 2.0));
        const std::string output = firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(&parent)));
        CHECK(output.find("G1 E2 F720\n") != std::string::npos);
        CHECK(firmware_move_e_values(output).size() == 1);
    }
    SECTION("zero planar length") {
        std::unique_ptr<ExtrusionPath> path = make_firmware_path(
            ArcPolyline(Points{Point::new_scale(0.0, 0.0), Point::new_scale(0.0, 0.0)}), 12.f, 300.f);
        path->add_property(ExtrusionPropertySpecialCommand(
            ExtrusionPropertySpecialCommand::Code::EXTRUSION, 2.0));
        CHECK_THROWS_AS(firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(path.get()))), std::invalid_argument);
    }
    SECTION("simultaneous semantic axis") {
        std::unique_ptr<ExtrusionPath> path = make_firmware_path(
            ArcPolyline(Points{Point::new_scale(0.0, 0.0), Point::new_scale(1.0, 0.0)}), 12.f, 300.f);
        path->add_property(ExtrusionPropertySpecialCommand(
            ExtrusionPropertySpecialCommand::Code::EXTRUSION, 2.0));
        path->get_or_add_property<slic3r_api::EPropertyExtrusionAxis>().unretract();
        CHECK_THROWS_AS(firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(path.get()))), std::invalid_argument);
    }
}

// A fixed-E EXTRUSION never changes the retraction state, while the explicit
// RETRACT command keeps its retract/deretract meaning.
TEST_CASE("Default firmware keeps raw filament moves out of the retraction state",
          "[plugins][gcode][firmware][retraction][fixed-e]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    ScriptStateFirmwareProbe firmware;
    firmware.setup(slic3r_api::Config(ApiHost::to_config_handle(&config)));
    std::unique_ptr<ExtrusionEntity> select = special_command_entity(
        ExtrusionPropertySpecialCommand::Code::TOOLCHANGE, 0.0);
    firmware.write_event(slic3r_api::ExtrusionEntity(
        reinterpret_cast<const extrusion_entity_handle *>(select.get())));
    slic3r_api::GCodeGeneration::ExtrusionAxisState &axis =
        firmware.machine_extruder(0).extrusion_axis();

    SECTION("an explicit retraction survives a raw unload and is restored once") {
        std::unique_ptr<ExtrusionEntity> retract = marker_test_retraction(true);
        firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(retract.get())));
        CHECK(axis.retracted() == Approx(0.2));
        std::unique_ptr<ExtrusionEntity> unload = special_command_entity(
            ExtrusionPropertySpecialCommand::Code::EXTRUSION, -5.0);
        unload->add_property(ExtrusionPropertySpeed(20.f, 500.f));
        firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(unload.get())));
        CHECK(axis.retracted() == Approx(0.2));
        std::unique_ptr<ExtrusionEntity> full = marker_test_retraction(false);
        const std::string output = firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(full.get())));
        const std::vector<double> values = firmware_move_e_values(output);
        REQUIRE(values.size() == 1);
        CHECK(values.front() == Approx(-5.0));
        CHECK_FALSE(axis.need_unretract());
    }
    SECTION("a raw negative EXTRUSION alone needs no unretraction") {
        std::unique_ptr<ExtrusionEntity> unload = special_command_entity(
            ExtrusionPropertySpecialCommand::Code::EXTRUSION, -2.0);
        unload->add_property(ExtrusionPropertySpeed(20.f, 500.f));
        firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(unload.get())));
        CHECK(axis.retracted() == 0.0);
        CHECK_FALSE(axis.need_unretract());
        CHECK(axis.unretract() == 0.0);
    }
    SECTION("the explicit RETRACT command still retracts and deretracts") {
        std::unique_ptr<ExtrusionEntity> retract = special_command_entity(
            ExtrusionPropertySpecialCommand::Code::RETRACT, -2.0);
        retract->add_property(ExtrusionPropertySpeed(20.f, 500.f));
        firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(retract.get())));
        CHECK(axis.retracted() == Approx(2.0));
        std::unique_ptr<ExtrusionEntity> deretract = special_command_entity(
            ExtrusionPropertySpecialCommand::Code::RETRACT, 2.0);
        deretract->add_property(ExtrusionPropertySpeed(20.f, 500.f));
        firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(deretract.get())));
        CHECK(axis.retracted() == 0.0);
    }
}

// Restore a limited static amount, leaving all extras for the final moving restore.
TEST_CASE("Default firmware clamps partial static unretraction before a Wipe leaf",
          "[plugins][gcode][firmware][retraction][partial-unretract]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool relative_e = GENERATE(false, true);
    const double partial_length = GENERATE(8.0, 20.0, 0.0);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize("use_relative_e_distances", relative_e ? "1" : "0");
    config.set_deserialize("deretract_speed", "23");
    ScriptStateFirmwareProbe firmware;
    firmware.setup(slic3r_api::Config(ApiHost::to_config_handle(&config)));
    std::unique_ptr<ExtrusionEntity> select = special_command_entity(
        ExtrusionPropertySpecialCommand::Code::TOOLCHANGE, 0.0);
    firmware.write_event(slic3r_api::ExtrusionEntity(
        reinterpret_cast<const extrusion_entity_handle *>(select.get())));
    std::unique_ptr<ExtrusionEntity> retract = marker_test_retraction(true);
    retract->get_or_add_property<slic3r_api::EPropertyExtrusionAxis>().retract_to(10.0, true);
    firmware.write_event(slic3r_api::ExtrusionEntity(
        reinterpret_cast<const extrusion_entity_handle *>(retract.get())));
    slic3r_api::GCodeGeneration::ExtrusionAxisState &axis = firmware.machine_extruder(0).extrusion_axis();
    axis.schedule_restart_extra(0.3, true);

    std::unique_ptr<ExtrusionEntity> partial = marker_test_retraction(false);
    slic3r_api::EPropertyExtrusionAxis &request =
        partial->get_or_add_property<slic3r_api::EPropertyExtrusionAxis>().unretract_partial(partial_length, true);
    // Positive limits ignore even a supplied extra; zero retains full-event semantics.
    request.restart_extra = 0.7;
    const std::string static_output = firmware.write_event(slic3r_api::ExtrusionEntity(
        reinterpret_cast<const extrusion_entity_handle *>(partial.get())));
    INFO(static_output);
    const std::vector<double> static_e = firmware_move_e_values(static_output);
    REQUIRE(static_e.size() == 1);
    const double restored = partial_length == 0.0 ? 10.7 : std::min(partial_length, 10.0);
    CHECK(static_e[0] == Approx(relative_e ? restored : restored - 10.0));
    CHECK(static_output.find("F1380") != std::string::npos);
    CHECK(axis.retracted() == Approx(partial_length == 8.0 ? 2.0 : 0.0));
    CHECK(axis.need_unretract() == (partial_length > 0.0));

    std::unique_ptr<ExtrusionPath> wipe = marker_test_wipe(0.0, 5.0);
    slic3r_api::MutableExtrusionEntity wipe_view(reinterpret_cast<extrusion_entity_handle *>(wipe.get()));
    wipe_view.get_mutable(slic3r_api::EPropertyAttributes::key)->extrusion_role(
        RAW_EXTRUSION_ROLE_TRAVEL | RAW_EXTRUSION_ROLE_WIPE | RAW_EXTRUSION_ROLE_UNRETRACT);
    wipe_view.get_or_add(slic3r_api::EPropertyExtrusionAxis::key).unretract(0.3, true);
    const std::string moving_output = firmware.write_event(slic3r_api::ExtrusionEntity(
        reinterpret_cast<const extrusion_entity_handle *>(wipe.get())));
    INFO(moving_output);
    const std::vector<double> moving_e = firmware_move_e_values(moving_output);
    REQUIRE(moving_e.size() == 1);
    const double remaining = partial_length == 8.0 ? 2.3 : 0.3;
    CHECK(moving_e[0] == Approx(relative_e ? remaining : (partial_length == 0.0 ? 1.0 : 0.3)));
    CHECK(moving_output.find("G1 X5 E") != std::string::npos);
    CHECK_FALSE(axis.need_unretract());
}

// A partial request neither primes an idle tool nor splits a native G11 command.
TEST_CASE("Default firmware skips partial unretraction without explicit outstanding filament",
          "[plugins][gcode][firmware][retraction][partial-unretract]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool native = GENERATE(false, true);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize("use_firmware_retraction", native ? "1" : "0");
    ScriptStateFirmwareProbe firmware;
    firmware.setup(slic3r_api::Config(ApiHost::to_config_handle(&config)));
    std::unique_ptr<ExtrusionEntity> select = special_command_entity(
        ExtrusionPropertySpecialCommand::Code::TOOLCHANGE, 0.0);
    firmware.write_event(slic3r_api::ExtrusionEntity(
        reinterpret_cast<const extrusion_entity_handle *>(select.get())));
    std::unique_ptr<ExtrusionEntity> partial = marker_test_retraction(false);
    partial->get_or_add_property<slic3r_api::EPropertyExtrusionAxis>().unretract_partial(8.0);
    CHECK(firmware.write_event(slic3r_api::ExtrusionEntity(
        reinterpret_cast<const extrusion_entity_handle *>(partial.get()))).empty());
    if (native) {
        std::unique_ptr<ExtrusionEntity> retract = marker_test_retraction(true);
        retract->get_or_add_property<slic3r_api::EPropertyExtrusionAxis>().retract_to(10.0);
        CHECK(firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(retract.get()))).find("G10\n") != std::string::npos);
        CHECK(firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(partial.get()))).empty());
        CHECK(firmware.machine_extruder(0).extrusion_axis().retracted() == Approx(10.0));
        std::unique_ptr<ExtrusionEntity> full = marker_test_retraction(false);
        CHECK(firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(full.get()))).find("G11\n") != std::string::npos);
        CHECK_FALSE(firmware.machine_extruder(0).extrusion_axis().need_unretract());
    }
}

TEST_CASE("E-only unretraction resolves zero speed per extruder without inherited speed",
          "[plugins][gcode][firmware][retraction][deretract-zero]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const extruder_index_t tool_id = GENERATE(extruder_index_t(0), extruder_index_t(1));
    const bool explicit_speed = GENERATE(false, true);
    Print print;
    select_printing_plan_firmware(print);
    configure_standard_firmware(print, 2);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.use_firmware_retraction.value = false;
    config.use_relative_e_distances.value = false;
    config.retract_speed.set(std::vector<double>{40.0, 55.0});
    config.deretract_speed.set(std::vector<double>{explicit_speed ? 18.0 : 0.0,
                                                 explicit_speed ? 23.0 : 0.0});
    PrintingToolGroup &tool = append_speed_validation_tool_group(print, scale_i(0.2), tool_id);

    // Neither event nor parent declares speed: both E-only moves must resolve
    // their speed from the selected tool, independently of previous XY moves.
    ExtrusionEntity::Children children;
    for (const bool is_retract : {true, false}) {
        std::unique_ptr<ExtrusionNop> event(new ExtrusionNop());
        slic3r_api::MutableExtrusionEntity view(
            reinterpret_cast<extrusion_entity_handle *>(event.get()));
        view.get_or_add(slic3r_api::EPropertyAttributes::key)
            .extrusion_role(is_retract ? RAW_EXTRUSION_ROLE_RETRACT : RAW_EXTRUSION_ROLE_UNRETRACT)
            .mm3_per_mm(0.0);
        if (is_retract)
            view.get_or_add(slic3r_api::EPropertyExtrusionAxis::key).retract_to(2.0);
        else
            view.get_or_add(slic3r_api::EPropertyExtrusionAxis::key).unretract(0.0);
        children.push_back(std::move(event));
    }
    append_path_extrusion(tool, std::unique_ptr<ExtrusionEntity>(
        new ExtrusionEntity(std::move(children), false, true, true)));

    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    try {
        Steps::StepGenerateGcode::run_step(Orchestrator::instance(), print, output_path.string());
        const std::string output = read_text_file(output_path);
        INFO(output);
        const std::string retract_feed = tool_id == 0 ? "2400" : "3300";
        CHECK(output.find("G1 E-2 F" + retract_feed + "\n") != std::string::npos);
        if (explicit_speed) {
            const std::string unretract_feed = tool_id == 0 ? "1080" : "1380";
            CHECK(output.find("G1 E0 F" + unretract_feed + "\n") != std::string::npos);
        } else {
            // The same feed is already encoded by the retract command.
            CHECK(output.find("G1 E0\n") != std::string::npos);
        }
        CHECK(count_occurrences(output, "G1 E") == 2);
    } catch (...) {
        remove_output_pair(output_path);
        throw;
    }
    remove_output_pair(output_path);
}

TEST_CASE("Default firmware encodes semantic retract wipe and unretract leaves",
          "[plugins][gcode][firmware][retraction][wipe]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    select_printing_plan_firmware(print);
    configure_standard_firmware(print);
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.front().layers.front();
    layer.print_z = scale_i(0.2);
    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool_group = layer.tool_groups.front();
    tool_group.extruder_id = 0;

    ExtrusionEntity::Children children;
    std::unique_ptr<ExtrusionNop> retract(new ExtrusionNop());
    slic3r_api::MutableExtrusionEntity retract_view(
        reinterpret_cast<extrusion_entity_handle *>(retract.get()));
    retract_view.get_or_add(slic3r_api::EPropertyAttributes::key)
        .extrusion_role(RAW_EXTRUSION_ROLE_RETRACT).mm3_per_mm(0.0);
    retract_view.get_or_add(slic3r_api::EPropertyExtrusionAxis::key)
        .retract_to(0.2);
    children.push_back(std::move(retract));

    ArcPolyline wipe_polyline(Points{Point(scale_i(0.0), scale_i(0.0)),
                                     Point(scale_i(2.5), scale_i(0.0)),
                                     Point(scale_i(5.0), scale_i(0.0))});
    wipe_polyline.set_z_offset(0, 0);
    wipe_polyline.set_z_offset(1, scale_i(10.0));
    wipe_polyline.set_z_offset(2, 0);
    std::unique_ptr<ExtrusionPath> wipe = make_firmware_travel(
        wipe_polyline, 40.f, 800.f);
    slic3r_api::MutableExtrusionEntity wipe_view(
        reinterpret_cast<extrusion_entity_handle *>(wipe.get()));
    wipe_view.get_mutable(slic3r_api::EPropertyAttributes::key)->extrusion_role(
        RAW_EXTRUSION_ROLE_TRAVEL | RAW_EXTRUSION_ROLE_WIPE |
        RAW_EXTRUSION_ROLE_RETRACT);
    wipe_view.get_or_add(slic3r_api::EPropertyExtrusionAxis::key).retract_to(0.8);
    children.push_back(std::move(wipe));

    std::unique_ptr<ExtrusionPath> unretract = make_firmware_travel(
        ArcPolyline(Points{Point(scale_i(5.0), scale_i(0.0)),
                           Point(scale_i(6.0), scale_i(0.0))}),
        40.f,
        800.f);
    slic3r_api::MutableExtrusionEntity unretract_view(
        reinterpret_cast<extrusion_entity_handle *>(unretract.get()));
    unretract_view.get_or_add(slic3r_api::EPropertyAttributes::key)
        .extrusion_role(RAW_EXTRUSION_ROLE_TRAVEL | RAW_EXTRUSION_ROLE_WIPE |
                        RAW_EXTRUSION_ROLE_UNRETRACT).mm3_per_mm(0.0);
    unretract_view.get_or_add(slic3r_api::EPropertyExtrusionAxis::key)
        .unretract(-0.05);
    children.push_back(std::move(unretract));

    append_path_extrusion(tool_group, std::unique_ptr<ExtrusionEntity>(
        new ExtrusionEntity(std::move(children), false, true, true)));

    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    try {
        Steps::StepGenerateGcode::run_step(Orchestrator::instance(), print, output_path.string());
        const std::string output = read_text_file(output_path);
        INFO(output);
        const size_t retract_pos = output.find("G1 E-.2");
        const size_t wipe_start_pos = output.find(";WIPE_START\n");
        const size_t first_wipe_move_pos = output.find("G1 X2.5 Z10.2 E-.5");
        const size_t wipe_move_pos = output.find("G1 X5 Z.2 E-.8");
        const size_t wipe_end_pos = output.find(";WIPE_END\n");
        const size_t unretract_pos = output.find("G1 X6 E-.05");
        REQUIRE(retract_pos != std::string::npos);
        REQUIRE(wipe_start_pos != std::string::npos);
        REQUIRE(first_wipe_move_pos != std::string::npos);
        REQUIRE(wipe_move_pos != std::string::npos);
        REQUIRE(wipe_end_pos != std::string::npos);
        REQUIRE(unretract_pos != std::string::npos);
        CHECK(retract_pos < wipe_start_pos);
        CHECK(wipe_start_pos < first_wipe_move_pos);
        CHECK(first_wipe_move_pos < wipe_move_pos);
        CHECK(wipe_move_pos < unretract_pos);
        CHECK(unretract_pos < wipe_end_pos);
        CHECK(output.find("G0 X5 E") == std::string::npos);
    } catch (...) {
        remove_output_pair(output_path);
        throw;
    }
    remove_output_pair(output_path);
}

// Motion-only wipe segments have no E words even when tool selection resets the E coordinate.
TEST_CASE("Default firmware accepts a motion-only wipe without an E request",
          "[plugins][gcode][firmware][wipe]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    select_printing_plan_firmware(print);
    configure_standard_firmware(print);
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.front().layers.front();
    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool_group = layer.tool_groups.front();
    tool_group.extruder_id = 0;

    std::unique_ptr<ExtrusionPath> wipe = make_firmware_travel(
        ArcPolyline(Points{Point(scale_i(0.0), scale_i(0.0)),
                           Point(scale_i(5.0), scale_i(0.0))}),
        40.f,
        800.f);
    slic3r_api::MutableExtrusionEntity wipe_view(
        reinterpret_cast<extrusion_entity_handle *>(wipe.get()));
    wipe_view.get_mutable(slic3r_api::EPropertyAttributes::key)->extrusion_role(
        RAW_EXTRUSION_ROLE_TRAVEL | RAW_EXTRUSION_ROLE_WIPE);
    append_path_extrusion(tool_group, std::move(wipe));

    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    try {
        Steps::StepGenerateGcode::run_step(Orchestrator::instance(), print, output_path.string());
        const std::string output = read_text_file(output_path);
        INFO(output);
        CHECK(output.find(";WIPE_START\n") != std::string::npos);
        CHECK(output.find("G1 X5") != std::string::npos);
        CHECK(count_occurrences(output, "G92 E0\n") == 1);
        for (const std::string &line : machine_command_lines(output))
            if (line.rfind("G0 ", 0) == 0 || line.rfind("G1 ", 0) == 0 ||
                line.rfind("G2 ", 0) == 0 || line.rfind("G3 ", 0) == 0)
                CHECK(line.find(" E") == std::string::npos);
    } catch (...) {
        remove_output_pair(output_path);
        throw;
    }
    remove_output_pair(output_path);
}

TEST_CASE("Default firmware supports native semantic retraction",
          "[plugins][gcode][firmware][retraction]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    select_printing_plan_firmware(print);
    configure_standard_firmware(print);
    const_cast<PrintConfig &>(print.config()).use_firmware_retraction.value = true;
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.front().layers.front();
    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool_group = layer.tool_groups.front();
    tool_group.extruder_id = 0;

    ExtrusionEntity::Children children;
    std::unique_ptr<ExtrusionNop> retract(new ExtrusionNop());
    slic3r_api::MutableExtrusionEntity retract_view(
        reinterpret_cast<extrusion_entity_handle *>(retract.get()));
    retract_view.get_or_add(slic3r_api::EPropertyAttributes::key)
        .extrusion_role(RAW_EXTRUSION_ROLE_RETRACT).mm3_per_mm(0.0);
    retract_view.get_or_add(slic3r_api::EPropertyExtrusionAxis::key).retract_to(0.8);
    children.push_back(std::move(retract));
    std::unique_ptr<ExtrusionPath> wipe = make_firmware_travel(
        ArcPolyline(Points{Point(scale_i(0.0), scale_i(0.0)),
                           Point(scale_i(5.0), scale_i(0.0))}),
        40.f,
        800.f);
    slic3r_api::MutableExtrusionEntity wipe_view(
        reinterpret_cast<extrusion_entity_handle *>(wipe.get()));
    wipe_view.get_mutable(slic3r_api::EPropertyAttributes::key)->extrusion_role(
        RAW_EXTRUSION_ROLE_TRAVEL | RAW_EXTRUSION_ROLE_WIPE |
        RAW_EXTRUSION_ROLE_RETRACT);
    wipe_view.get_or_add(slic3r_api::EPropertyExtrusionAxis::key).retract_to(0.8);
    children.push_back(std::move(wipe));
    std::unique_ptr<ExtrusionPath> unretract = make_firmware_travel(
        ArcPolyline(Points{Point(scale_i(5.0), scale_i(0.0)),
                           Point(scale_i(6.0), scale_i(0.0))}),
        40.f,
        800.f);
    slic3r_api::MutableExtrusionEntity unretract_view(
        reinterpret_cast<extrusion_entity_handle *>(unretract.get()));
    unretract_view.get_or_add(slic3r_api::EPropertyAttributes::key)
        .extrusion_role(RAW_EXTRUSION_ROLE_TRAVEL | RAW_EXTRUSION_ROLE_WIPE |
                        RAW_EXTRUSION_ROLE_UNRETRACT).mm3_per_mm(0.0);
    unretract_view.get_or_add(slic3r_api::EPropertyExtrusionAxis::key).unretract();
    children.push_back(std::move(unretract));
    append_path_extrusion(tool_group, std::unique_ptr<ExtrusionEntity>(
        new ExtrusionEntity(std::move(children), false, true, true)));

    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    try {
        Steps::StepGenerateGcode::run_step(Orchestrator::instance(), print, output_path.string());
        const std::string output = read_text_file(output_path);
        INFO(output);
        CHECK(count_occurrences(output, "G10\n") == 1);
        CHECK(count_occurrences(output, "G11\n") == 1);
        CHECK(output.find("G1 E") == std::string::npos);
        const size_t wipe_start = output.find(";WIPE_START\n");
        const size_t wipe_move = output.find("G1 X5");
        const size_t approach_move = output.find("G1 X6");
        const size_t unretract = output.find("G11\n");
        REQUIRE(wipe_start != std::string::npos);
        REQUIRE(wipe_move != std::string::npos);
        REQUIRE(approach_move != std::string::npos);
        REQUIRE(unretract != std::string::npos);
        CHECK(wipe_start < wipe_move);
        CHECK(wipe_move < approach_move);
        CHECK(approach_move < unretract);
        CHECK(output.find("G0 X5") == std::string::npos);
    } catch (...) {
        remove_output_pair(output_path);
        throw;
    }
    remove_output_pair(output_path);
}
TEST_CASE("A derived firmware may replace only the fan encoder",
          "[plugins][gcode][firmware]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    configure_standard_firmware(print);
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.front().layers.front();
    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool_group = layer.tool_groups.front();
    tool_group.extruder_id = 0;
    append_path_extrusion(
        tool_group,
        make_firmware_path(
            ArcPolyline(Points{Point(scale_i(0.0), scale_i(0.0)), Point(scale_i(1.0), scale_i(0.0))}),
            15.f,
            400.f,
            -1.f,
            40.f));

    FirmwareInstanceOwner firmware(slic3r_api::make_gcode_firmware_instance(
        std::unique_ptr<slic3r_api::GCodeFirmwareSession>(new CustomFanFirmwareSession())));
    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    try {
        run_file_writer_with_firmware(print, output_path, firmware.instance());
        const std::string output = read_text_file(output_path);
        CHECK(output.find("CUSTOM_FAN T0 S40\n") != std::string::npos);
        CHECK(output.find("M106") == std::string::npos);
        CHECK(output.find("M204 P400\n") != std::string::npos);
        CHECK(output.find("G1 X1 E") != std::string::npos);
    } catch (...) {
        remove_output_pair(output_path);
        throw;
    }
    remove_output_pair(output_path);
}
TEST_CASE("A derived firmware may extend extrusion traversal without replacing it",
          "[plugins][gcode][firmware]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    configure_standard_firmware(print);
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.front().layers.front();
    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool_group = layer.tool_groups.front();
    tool_group.extruder_id = 0;

    // The collection and its path form two nested nodes. The derived session
    // observes both scopes and the leaf while the base visitor still performs
    // all standard state resolution and geometry serialization.
    ExtrusionEntity::Children children;
    children.push_back(make_firmware_path(
        ArcPolyline(Points{Point(scale_i(0.0), scale_i(0.0)), Point(scale_i(1.0), scale_i(0.0))}),
        15.f,
        400.f));
    append_path_extrusion(
        tool_group,
        std::unique_ptr<ExtrusionEntity>(new ExtrusionEntity(std::move(children), false, true, true)));

    std::vector<std::string> calls;
    FirmwareInstanceOwner firmware(slic3r_api::make_gcode_firmware_instance(
        std::unique_ptr<slic3r_api::GCodeFirmwareSession>(new TraversalHookFirmwareSession(calls))));
    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    try {
        run_file_writer_with_firmware(print, output_path, firmware.instance());
        const std::vector<std::string> expected{"enter", "enter", "leaf", "leave", "leave"};
        CHECK(calls == expected);
    } catch (...) {
        remove_output_pair(output_path);
        throw;
    }
    remove_output_pair(output_path);
}
TEST_CASE("Default firmware restores inherited requests between sibling nodes",
          "[plugins][gcode][firmware]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    select_printing_plan_firmware(print);
    configure_standard_firmware(print);
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.front().layers.front();
    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool_group = layer.tool_groups.front();
    tool_group.extruder_id = 0;

    ExtrusionEntity::Children children;
    children.push_back(make_firmware_path(
        ArcPolyline(Points{Point(scale_i(0.0), scale_i(0.0)), Point(scale_i(1.0), scale_i(0.0))}),
        -1.f,
        -1.f));
    children.push_back(make_firmware_path(
        ArcPolyline(Points{Point(scale_i(1.0), scale_i(0.0)), Point(scale_i(2.0), scale_i(0.0))}),
        30.f,
        700.f,
        0.08f,
        70.f,
        220.f));
    children.push_back(make_firmware_path(
        ArcPolyline(Points{Point(scale_i(2.0), scale_i(0.0)), Point(scale_i(3.0), scale_i(0.0))}),
        -1.f,
        -1.f));
    std::unique_ptr<ExtrusionEntity> root(
        new ExtrusionEntity(std::move(children), false, true, true));
    root->add_property(ExtrusionPropertySpeed(20.f, 500.f, 0.04f));
    root->add_property(ExtrusionPropertyFan(50.f, FAN_REQUEST_MODE_OVERRIDE));
    root->add_property(ExtrusionPropertyHeater(
        C_EXTRUSION_HEATER_TOOL, EXTRUDER_ID_UNKNOWN, 210, false));
    append_path_extrusion(tool_group, std::move(root));

    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    try {
        Steps::StepGenerateGcode::run_step(Orchestrator::instance(), print, output_path.string());
        const std::string output = read_text_file(output_path);
        INFO(output);

        const size_t first_temperature = output.find("M104 S210 T0\n");
        const size_t child_temperature = output.find("M104 S220 T0\n");
        REQUIRE(first_temperature != std::string::npos);
        REQUIRE(child_temperature != std::string::npos);
        CHECK(first_temperature < child_temperature);
        CHECK(count_occurrences(output, "M104 S210 T0\n") == 1);
        CHECK(count_occurrences(output, "M204 P500\n") == 2);
        CHECK(count_occurrences(output, "M204 P700\n") == 1);
        CHECK(count_occurrences(output, " F1200") == 2);
        CHECK(count_occurrences(output, " F1800") == 1);
        CHECK(count_occurrences(output, "M106 S128\n") == 2);
        CHECK(count_occurrences(output, "M106 S179\n") == 1);
    } catch (...) {
        remove_output_pair(output_path);
        throw;
    }
    remove_output_pair(output_path);
}

// Verify that heater entry events emit once, persist across descendants and can address every supported heater.
TEST_CASE("Heater events set targets once and persist", "[plugins][gcode][firmware][heater]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    SECTION("an active-tool event is emitted before its first movement") {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.set_deserialize("filament_diameter", "1.75,1.75");
        config.set_deserialize("extrusion_multiplier", "1,1");
        ScriptStateFirmwareProbe firmware;
        firmware.setup(slic3r_api::Config(ApiHost::to_config_handle(&config)));
        std::unique_ptr<ExtrusionEntity> select = special_command_entity(
            ExtrusionPropertySpecialCommand::Code::TOOLCHANGE, 1.0);
        firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(select.get())));

        ExtrusionEntity::Children children;
        children.push_back(make_firmware_path(
            ArcPolyline(Points{Point(scale_i(0.0), scale_i(0.0)), Point(scale_i(1.0), scale_i(0.0))}),
            20.f, 500.f));
        children.push_back(make_firmware_path(
            ArcPolyline(Points{Point(scale_i(1.0), scale_i(0.0)), Point(scale_i(2.0), scale_i(0.0))}),
            20.f, 500.f));
        std::unique_ptr<ExtrusionEntity> root(
            new ExtrusionEntity(std::move(children), false, true, true));
        root->add_property(ExtrusionPropertyHeater(
            C_EXTRUSION_HEATER_TOOL, EXTRUDER_ID_UNKNOWN, 205, false));

        const std::string output = firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(root.get())));
        const size_t temperature_pos = output.find("M104 S205 T1\n");
        const size_t movement_pos = first_printable_move_offset(output);
        REQUIRE(temperature_pos != std::string::npos);
        REQUIRE(movement_pos != std::string::npos);
        CHECK(temperature_pos < movement_pos);
        CHECK(count_occurrences(output, "M104 S205 T1\n") == 1);
        CHECK(count_occurrences(output, "M104 ") == 1);
        CHECK(count_occurrences(output, "M109 ") == 0);
        CHECK(firmware.machine_extruder(1).heater().requested_temperature() == 205);
        CHECK(firmware.machine_extruder(1).heater().encoded_temperature() == 205);
    }

    SECTION("an identical wait event is acknowledged only once") {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.set_deserialize("filament_diameter", "1.75,1.75");
        config.set_deserialize("extrusion_multiplier", "1,1");
        ScriptStateFirmwareProbe firmware;
        firmware.setup(slic3r_api::Config(ApiHost::to_config_handle(&config)));
        std::unique_ptr<ExtrusionEntity> select = special_command_entity(
            ExtrusionPropertySpecialCommand::Code::TOOLCHANGE, 1.0);
        firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(select.get())));

        std::unique_ptr<ExtrusionNop> first(new ExtrusionNop());
        first->add_property(ExtrusionPropertyHeater(
            C_EXTRUSION_HEATER_TOOL, EXTRUDER_ID_UNKNOWN, 210, true));
        CHECK(firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(first.get()))) == "M109 S210 T1\n");

        std::unique_ptr<ExtrusionNop> second(new ExtrusionNop());
        second->add_property(ExtrusionPropertyHeater(
            C_EXTRUSION_HEATER_TOOL, EXTRUDER_ID_UNKNOWN, 210, true));
        CHECK(firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(second.get()))).empty());
    }

    SECTION("a non-active Marlin tool keeps its own target after selection") {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.set_deserialize("filament_diameter", "1.75,1.75");
        config.set_deserialize("extrusion_multiplier", "1,1");
        ScriptStateFirmwareProbe firmware;
        firmware.setup(slic3r_api::Config(ApiHost::to_config_handle(&config)));
        std::unique_ptr<ExtrusionEntity> select_one = special_command_entity(
            ExtrusionPropertySpecialCommand::Code::TOOLCHANGE, 1.0);
        firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(select_one.get())));

        std::unique_ptr<ExtrusionNop> active_target(new ExtrusionNop());
        active_target->add_property(ExtrusionPropertyHeater(
            C_EXTRUSION_HEATER_TOOL, EXTRUDER_ID_UNKNOWN, 205, false));
        CHECK(firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(active_target.get()))) == "M104 S205 T1\n");

        std::unique_ptr<ExtrusionNop> event(new ExtrusionNop());
        event->add_property(ExtrusionPropertyHeater(
            C_EXTRUSION_HEATER_TOOL, extruder_index_t(0), 180, false));
        CHECK(firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(event.get()))) == "M104 S180 T0\n");
        CHECK(firmware.machine_extruder(1).heater().requested_temperature() == 205);
        CHECK(firmware.machine_extruder(1).heater().encoded_temperature() == 205);

        std::unique_ptr<ExtrusionEntity> select_zero = special_command_entity(
            ExtrusionPropertySpecialCommand::Code::TOOLCHANGE, 0.0);
        const std::string selection = firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(select_zero.get())));
        CHECK(selection.find("M104 S180") == std::string::npos);
        CHECK(count_occurrences(selection, "M104 ") == 0);
        CHECK(firmware.machine_extruder(0).heater().requested_temperature() == 180);
        CHECK(firmware.machine_extruder(0).heater().encoded_temperature() == 180);

        std::unique_ptr<ExtrusionPath> path = make_firmware_path(
            ArcPolyline(Points{Point(scale_i(0.0), scale_i(0.0)),
                               Point(scale_i(1.0), scale_i(0.0))}),
            20.f, 500.f);
        const std::string extrusion = firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(path.get())));
        CHECK(extrusion.find("M104") == std::string::npos);
    }

    SECTION("bed and chamber targets have separate wait encodings") {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        ScriptStateFirmwareProbe firmware;
        firmware.setup(slic3r_api::Config(ApiHost::to_config_handle(&config)));

        std::unique_ptr<ExtrusionNop> bed(new ExtrusionNop());
        bed->add_property(ExtrusionPropertyHeater(
            C_EXTRUSION_HEATER_BED, EXTRUDER_ID_UNKNOWN, 60, false));
        CHECK(firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(bed.get()))) == "M140 S60\n");
        std::unique_ptr<ExtrusionNop> bed_wait(new ExtrusionNop());
        bed_wait->add_property(ExtrusionPropertyHeater(
            C_EXTRUSION_HEATER_BED, EXTRUDER_ID_UNKNOWN, 60, true));
        CHECK(firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(bed_wait.get()))) == "M190 S60\n");

        std::unique_ptr<ExtrusionNop> chamber(new ExtrusionNop());
        chamber->add_property(ExtrusionPropertyHeater(
            C_EXTRUSION_HEATER_CHAMBER, EXTRUDER_ID_UNKNOWN, 40, false));
        CHECK(firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(chamber.get()))) == "M141 S40\n");
        std::unique_ptr<ExtrusionNop> chamber_wait(new ExtrusionNop());
        chamber_wait->add_property(ExtrusionPropertyHeater(
            C_EXTRUSION_HEATER_CHAMBER, EXTRUDER_ID_UNKNOWN, 40, true));
        CHECK(firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(chamber_wait.get()))) == "M191 S40\n");
        CHECK(firmware.machine_printer().bed_heater().requested_temperature() == 60);
        CHECK(firmware.machine_printer().chamber_heater().requested_temperature() == 40);

        slic3r_api::GCodeGeneration::Firmware::KlipperGCodeFirmwareSession klipper;
        klipper.setup(slic3r_api::Config(ApiHost::to_config_handle(&config)));
        std::unique_ptr<ExtrusionNop> klipper_chamber(new ExtrusionNop());
        klipper_chamber->add_property(ExtrusionPropertyHeater(
            C_EXTRUSION_HEATER_CHAMBER, EXTRUDER_ID_UNKNOWN, 40, true));
        const std::string klipper_output = klipper.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(klipper_chamber.get())));
        CHECK(klipper_output.find("M141") == std::string::npos);
        CHECK(klipper_output.find("M191") == std::string::npos);
    }

    SECTION("invalid heater events are rejected") {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.set_deserialize("filament_diameter", "1.75,1.75");
        config.set_deserialize("extrusion_multiplier", "1,1");
        ScriptStateFirmwareProbe firmware;
        firmware.setup(slic3r_api::Config(ApiHost::to_config_handle(&config)));
        std::unique_ptr<ExtrusionEntity> select = special_command_entity(
            ExtrusionPropertySpecialCommand::Code::TOOLCHANGE, 1.0);
        firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(select.get())));

        std::unique_ptr<ExtrusionNop> negative(new ExtrusionNop());
        negative->add_property(ExtrusionPropertyHeater(
            C_EXTRUSION_HEATER_TOOL, EXTRUDER_ID_UNKNOWN, -1, false));
        CHECK_THROWS_AS(firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(negative.get()))), std::invalid_argument);

        std::unique_ptr<ExtrusionNop> out_of_range(new ExtrusionNop());
        out_of_range->add_property(ExtrusionPropertyHeater(
            C_EXTRUSION_HEATER_TOOL, extruder_index_t(2), 180, false));
        CHECK_THROWS_AS(firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(out_of_range.get()))), std::out_of_range);

        DynamicPrintConfig no_active_config = DynamicPrintConfig::full_print_config();
        ScriptStateFirmwareProbe no_active;
        no_active.setup(slic3r_api::Config(ApiHost::to_config_handle(&no_active_config)));
        std::unique_ptr<ExtrusionNop> active_without_selection(new ExtrusionNop());
        active_without_selection->add_property(ExtrusionPropertyHeater(
            C_EXTRUSION_HEATER_TOOL, EXTRUDER_ID_UNKNOWN, 180, false));
        CHECK_THROWS_AS(no_active.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(active_without_selection.get()))), std::invalid_argument);

        DynamicPrintConfig semm_config = DynamicPrintConfig::full_print_config();
        semm_config.set_deserialize("single_extruder_multi_material", "1");
        semm_config.set_deserialize("filament_diameter", "1.75,1.75");
        semm_config.set_deserialize("extrusion_multiplier", "1,1");
        ScriptStateFirmwareProbe semm;
        semm.setup(slic3r_api::Config(ApiHost::to_config_handle(&semm_config)));
        std::unique_ptr<ExtrusionEntity> semm_select = special_command_entity(
            ExtrusionPropertySpecialCommand::Code::TOOLCHANGE, 1.0);
        semm.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(semm_select.get())));
        std::unique_ptr<ExtrusionNop> non_active_semm(new ExtrusionNop());
        non_active_semm->add_property(ExtrusionPropertyHeater(
            C_EXTRUSION_HEATER_TOOL, extruder_index_t(0), 180, false));
        CHECK_THROWS_AS(semm.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(non_active_semm.get()))), std::invalid_argument);

        std::unique_ptr<ExtrusionNop> unknown(new ExtrusionNop());
        ExtrusionPropertyHeater &unknown_property =
            unknown->get_or_add_property<ExtrusionPropertyHeater>();
        unknown_property.heater = static_cast<c_extrusion_heater_kind>(99);
        unknown_property.temperature_C = 180;
        CHECK_THROWS_AS(firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(unknown.get()))), std::invalid_argument);
    }

    SECTION("external G-code remains in force until a new heater event") {
        Print print;
        configure_standard_firmware(print);
        GCodeScriptProcessor processor(print, Orchestrator::instance());
        ScriptStateFirmwareProbe firmware(slic3r_api::GCodeGeneration::GCodeScriptProcessorView(
            processor.c_processor()));
        firmware.set_gcode_reader_context(
            reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()),
            reinterpret_cast<const print_handle *>(&print));
        firmware.setup(slic3r_api::Print(reinterpret_cast<const print_handle *>(&print)).config());
        std::unique_ptr<ExtrusionEntity> select = special_command_entity(
            ExtrusionPropertySpecialCommand::Code::TOOLCHANGE, 0.0);
        firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(select.get())));

        std::unique_ptr<ExtrusionNop> initial(new ExtrusionNop());
        initial->add_property(ExtrusionPropertyHeater(
            C_EXTRUSION_HEATER_TOOL, EXTRUDER_ID_UNKNOWN, 200, false));
        CHECK(firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(initial.get()))) == "M104 S200 T0\n");

        std::unique_ptr<ExtrusionNop> script(new ExtrusionNop());
        script->add_property(ExtrusionPropertyCustomGcodeText("M104 S215"));
        CHECK(firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(script.get()))) == "M104 S215\n");

        std::unique_ptr<ExtrusionPath> path = make_firmware_path(
            ArcPolyline(Points{Point(scale_i(0.0), scale_i(0.0)), Point(scale_i(1.0), scale_i(0.0))}),
            20.f, 500.f);
        const std::string extrusion = firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(path.get())));
        CHECK(extrusion.find("M104 S200") == std::string::npos);
        CHECK(count_occurrences(extrusion, "M104 ") == 0);
        CHECK(firmware.machine_extruder(0).heater().requested_temperature() == 215);
        CHECK(firmware.machine_extruder(0).heater().encoded_temperature() == 215);
        CHECK_FALSE(firmware.machine_extruder(0).heater().encoded_temperature_with_wait());

        std::unique_ptr<ExtrusionNop> replacement(new ExtrusionNop());
        replacement->add_property(ExtrusionPropertyHeater(
            C_EXTRUSION_HEATER_TOOL, EXTRUDER_ID_UNKNOWN, 205, false));
        CHECK(firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(replacement.get()))) == "M104 S205 T0\n");
    }

    SECTION("reader invalidates a wait acknowledgement after a different non-waiting target") {
        Print print;
        configure_standard_firmware(print);
        GCodeScriptProcessor processor(print, Orchestrator::instance());
        ScriptStateFirmwareProbe firmware(slic3r_api::GCodeGeneration::GCodeScriptProcessorView(
            processor.c_processor()));
        firmware.set_gcode_reader_context(
            reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()),
            reinterpret_cast<const print_handle *>(&print));
        firmware.setup(slic3r_api::Print(reinterpret_cast<const print_handle *>(&print)).config());
        std::unique_ptr<ExtrusionEntity> select = special_command_entity(
            ExtrusionPropertySpecialCommand::Code::TOOLCHANGE, 0.0);
        firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(select.get())));

        std::unique_ptr<ExtrusionNop> waited(new ExtrusionNop());
        waited->add_property(ExtrusionPropertyHeater(
            C_EXTRUSION_HEATER_TOOL, EXTRUDER_ID_UNKNOWN, 180, true));
        CHECK(firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(waited.get()))) == "M109 S180 T0\n");

        std::unique_ptr<ExtrusionNop> script(new ExtrusionNop());
        script->add_property(ExtrusionPropertyCustomGcodeText("M104 S200"));
        CHECK(firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(script.get()))) == "M104 S200\n");
        CHECK_FALSE(firmware.machine_extruder(0).heater().encoded_temperature_with_wait());

        std::unique_ptr<ExtrusionNop> normal(new ExtrusionNop());
        normal->add_property(ExtrusionPropertyHeater(
            C_EXTRUSION_HEATER_TOOL, EXTRUDER_ID_UNKNOWN, 180, false));
        CHECK(firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(normal.get()))) == "M104 S180 T0\n");

        std::unique_ptr<ExtrusionEntity> wait_again = special_command_entity(
            ExtrusionPropertySpecialCommand::Code::WAIT_FOR_TEMP);
        CHECK(firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(wait_again.get()))) == "M109 S180 T0\n");
    }

    SECTION("reader preserves a wait acknowledgement after the same non-waiting target") {
        Print print;
        configure_standard_firmware(print);
        GCodeScriptProcessor processor(print, Orchestrator::instance());
        ScriptStateFirmwareProbe firmware(slic3r_api::GCodeGeneration::GCodeScriptProcessorView(
            processor.c_processor()));
        firmware.set_gcode_reader_context(
            reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()),
            reinterpret_cast<const print_handle *>(&print));
        firmware.setup(slic3r_api::Print(reinterpret_cast<const print_handle *>(&print)).config());
        std::unique_ptr<ExtrusionEntity> select = special_command_entity(
            ExtrusionPropertySpecialCommand::Code::TOOLCHANGE, 0.0);
        firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(select.get())));

        std::unique_ptr<ExtrusionNop> waited(new ExtrusionNop());
        waited->add_property(ExtrusionPropertyHeater(
            C_EXTRUSION_HEATER_TOOL, EXTRUDER_ID_UNKNOWN, 180, true));
        CHECK(firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(waited.get()))) == "M109 S180 T0\n");

        std::unique_ptr<ExtrusionNop> script(new ExtrusionNop());
        script->add_property(ExtrusionPropertyCustomGcodeText("M104 S180"));
        CHECK(firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(script.get()))) == "M104 S180\n");
        CHECK(firmware.machine_extruder(0).heater().encoded_temperature_with_wait() == 180);

        std::unique_ptr<ExtrusionEntity> wait_again = special_command_entity(
            ExtrusionPropertySpecialCommand::Code::WAIT_FOR_TEMP);
        CHECK(firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(wait_again.get()))).empty());
    }

    SECTION("Klipper addresses active and non-active tools explicitly") {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.set_deserialize("filament_diameter", "1.75,1.75");
        config.set_deserialize("extrusion_multiplier", "1,1");
        config.set_deserialize("tool_name", "left_hotend,right_hotend");
        slic3r_api::GCodeGeneration::Firmware::KlipperGCodeFirmwareSession firmware;
        firmware.setup(slic3r_api::Config(ApiHost::to_config_handle(&config)));
        std::unique_ptr<ExtrusionEntity> select = special_command_entity(
            ExtrusionPropertySpecialCommand::Code::TOOLCHANGE, 1.0);
        firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(select.get())));

        std::unique_ptr<ExtrusionNop> active(new ExtrusionNop());
        active->add_property(ExtrusionPropertyHeater(
            C_EXTRUSION_HEATER_TOOL, EXTRUDER_ID_UNKNOWN, 205, false));
        CHECK(firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(active.get()))) == "M104 S205\n");
        std::unique_ptr<ExtrusionNop> first_wait(new ExtrusionNop());
        first_wait->add_property(ExtrusionPropertyHeater(
            C_EXTRUSION_HEATER_TOOL, extruder_index_t(0), 190, true));
        CHECK(firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(first_wait.get()))) ==
              "SET_HEATER_TEMPERATURE HEATER=left_hotend TARGET=190\n"
              "TEMPERATURE_WAIT SENSOR=left_hotend MINIMUM=190\n");
        std::unique_ptr<ExtrusionNop> other(new ExtrusionNop());
        other->add_property(ExtrusionPropertyHeater(
            C_EXTRUSION_HEATER_TOOL, extruder_index_t(0), 180, false));
        CHECK(firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(other.get()))) ==
              "SET_HEATER_TEMPERATURE HEATER=left_hotend TARGET=180\n");

        std::unique_ptr<ExtrusionNop> other_wait(new ExtrusionNop());
        other_wait->add_property(ExtrusionPropertyHeater(
            C_EXTRUSION_HEATER_TOOL, extruder_index_t(0), 180, true));
        CHECK(firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(other_wait.get()))) ==
              "SET_HEATER_TEMPERATURE HEATER=left_hotend TARGET=180\n"
              "TEMPERATURE_WAIT SENSOR=left_hotend MINIMUM=180\n");

        std::unique_ptr<ExtrusionNop> cooling(new ExtrusionNop());
        cooling->add_property(ExtrusionPropertyHeater(
            C_EXTRUSION_HEATER_TOOL, extruder_index_t(0), 170, true));
        CHECK(firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(cooling.get()))) ==
              "SET_HEATER_TEMPERATURE HEATER=left_hotend TARGET=170\n"
              "TEMPERATURE_WAIT SENSOR=left_hotend MAXIMUM=170\n");

        std::unique_ptr<ExtrusionNop> off_wait(new ExtrusionNop());
        off_wait->add_property(ExtrusionPropertyHeater(
            C_EXTRUSION_HEATER_TOOL, extruder_index_t(0), 0, true));
        CHECK(firmware.write_event(slic3r_api::ExtrusionEntity(
            reinterpret_cast<const extrusion_entity_handle *>(off_wait.get()))) ==
              "SET_HEATER_TEMPERATURE HEATER=left_hotend TARGET=0\n");
    }
}

// Ordered events retain their command sequence, including coordinate resets after emitted selections.
TEST_CASE("Default firmware handles ordered special commands and custom G-code",
          "[plugins][gcode][firmware]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    configure_standard_firmware(print, 2);
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
    children.push_back(special_command_entity(ExtrusionPropertySpecialCommand::Code::TOOLCHANGE, 1.0));
    children.push_back(special_command_entity(
        ExtrusionPropertySpecialCommand::Code::SAVE_AND_RESET_SPEED_RATIO, 0.8));
    children.push_back(special_command_entity(
        ExtrusionPropertySpecialCommand::Code::RESTORE_SPEED_RATIO));
    children.push_back(special_command_entity(
        ExtrusionPropertySpecialCommand::Code::FLUSH_PLANNER_QUEUE));
    children.push_back(special_command_entity(
        ExtrusionPropertySpecialCommand::Code::EXTRUSION, 0.5));
    children.push_back(special_command_entity(
        ExtrusionPropertySpecialCommand::Code::RETRACT, -0.2));
    children.push_back(special_command_entity(
        ExtrusionPropertySpecialCommand::Code::PAUSE, 25.0));
    children.push_back(special_command_entity(
        ExtrusionPropertySpecialCommand::Code::WAIT_FOR_TEMP));
    children.push_back(special_command_entity(
        ExtrusionPropertySpecialCommand::Code::WAIT_FOR_TEMP));
    children.push_back(special_command_entity(
        ExtrusionPropertySpecialCommand::Code::DISABLE_PREVIEW));
    children.push_back(special_command_entity(
        ExtrusionPropertySpecialCommand::Code::ENABLE_PREVIEW));
    children.push_back(special_command_entity(
        ExtrusionPropertySpecialCommand::Code::EXTRUDER_CURRENT, 650.0));
    std::unique_ptr<ExtrusionNop> custom(new ExtrusionNop());
    custom->add_property(ExtrusionPropertyCustomGcodeText("M117 ready"));
    children.push_back(std::move(custom));

    std::unique_ptr<ExtrusionEntity> root(
        new ExtrusionEntity(std::move(children), false, true, true));
    root->add_property(ExtrusionPropertySpeed(12.f, 300.f, 0.02f));
    root->add_property(ExtrusionPropertyFan(30.f, FAN_REQUEST_MODE_OVERRIDE));
    root->add_property(ExtrusionPropertyHeater(
        C_EXTRUSION_HEATER_TOOL, 1, 205, false));
    append_path_extrusion(tool_group, std::move(root));

    FirmwareInstanceOwner firmware(slic3r_api::make_gcode_firmware_instance(
        std::unique_ptr<slic3r_api::GCodeFirmwareSession>(new PreviewInspectingFirmwareSession())));
    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    try {
        run_file_writer_with_firmware(print, output_path, firmware.instance());
        const std::string output = read_text_file(output_path);
        INFO(output);
        CHECK(output.find("T0\nG92 E0\nM104 S205 T1\nT1\nG92 E0\n") != std::string::npos);
        CHECK(output.find("M220 S80\nM220 S100\n") != std::string::npos);
        CHECK(output.find("M220 B\n") == std::string::npos);
        CHECK(output.find("M220 R\n") == std::string::npos);
        CHECK(output.find("G4 S0\n") != std::string::npos);
        CHECK(output.find("G1 E") != std::string::npos);
        CHECK(output.find(" F720\n") != std::string::npos);
        CHECK(output.find(" F2400\n") != std::string::npos);
        CHECK(output.find("G4 P25\n") != std::string::npos);
        CHECK(output.find("M109 S205 T1\n") != std::string::npos);
        CHECK(count_occurrences(output, "M109 S205 T1\n") == 1);
        CHECK(output.find("M906 T1 E650\n") != std::string::npos);
        CHECK(output.find("M117 ready\n") != std::string::npos);
        CHECK(output.find("PREVIEW_ON\n") != std::string::npos);
    } catch (...) {
        remove_output_pair(output_path);
        throw;
    }
    remove_output_pair(output_path);
}

// Semantic tool selection emits one command/reset pair rather than duplicating the automatic selection.
TEST_CASE("Tool-group semantic selection suppresses the automatic firmware command",
          "[plugins][gcode][firmware][toolchange]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    configure_standard_firmware(print, 2);
    // Isolate runtime command encoding from automatic startup requests.
    const_cast<PrintConfig &>(print.config()).start_gcode_manual.value = true;
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.front().layers.front();
    layer.tool_groups.emplace_back();
    layer.tool_groups.back().extruder_id = 0;
    layer.tool_groups.emplace_back();
    PrintingToolGroup &target = layer.tool_groups.back();
    target.extruder_id = 1;

    // CreateToolChange stores the semantic command in the target tool-group's
    // before events. begin_tool_group() must update no physical selection of
    // its own; write_event() then emits the sole T1 command in sequence.
    std::unique_ptr<ExtrusionEntity> command = special_command_entity(
        ExtrusionPropertySpecialCommand::Code::TOOLCHANGE, 1.0);
    target.events.append_before(std::move(*command));

    const std::string output = export_with_firmware(
        print, "gcode.firmware.marlin2");
    INFO(output);
    CHECK(count_command_lines(output, "T0") == 1);
    CHECK(count_command_lines(output, "T1") == 1);
    CHECK(output.find("T0\nG92 E0\nT1\nG92 E0\n") != std::string::npos);
}
TEST_CASE("Default firmware distinguishes raw G-code, comments and scripts",
          "[plugins][gcode][firmware][custom-gcode]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    CustomGCodeFirmwareProbe firmware;

    CHECK(firmware.encode_custom_gcode(
              C_EXTRUSION_CUSTOM_GCODE_GCODE, "M117 raw") == "M117 raw\n");
    CHECK(firmware.encode_custom_gcode(
              C_EXTRUSION_CUSTOM_GCODE_COMMENT, "first\nsecond") ==
          "; first\n; second\n");
    CHECK_THROWS_AS(
        firmware.encode_custom_gcode(C_EXTRUSION_CUSTOM_GCODE_SCRIPT, "M117 {layer_num}"),
        std::invalid_argument);
    CHECK_THROWS_AS(
        firmware.encode_custom_gcode(
            static_cast<c_extrusion_custom_gcode_kind>(99), "M117 unknown"),
        std::invalid_argument);

    Print print;
    configure_standard_firmware(print);
    const slic3r_api::Print print_view(reinterpret_cast<const print_handle *>(&print));
    CHECK(firmware.begin_print(print_view, slic3r_api::ConstPrintingPlan(reinterpret_cast<const printing_plan_handle *>(&print.mutable_printing_plan()))).size() > 0);

    ExtrusionNop typed_raw(ExtrusionPropertyCustomGcodeText("M117 raw"));
    ExtrusionPropertyCustomGcode *typed_raw_property = typed_raw.get_property<ExtrusionPropertyCustomGcode>();
    REQUIRE(typed_raw_property != nullptr);
    typed_raw_property->script_type = GCODE_SCRIPT_TYPE_START_GCODE;
    CHECK_THROWS_AS(firmware.write_event(
                        slic3r_api::ExtrusionEntity(reinterpret_cast<const extrusion_entity_handle *>(&typed_raw))),
                    std::invalid_argument);

    ExtrusionNop untyped_script(
        ExtrusionPropertyCustomGcodeText(ExtrusionPropertyCustomGcodeText::Code::SCRIPT, "M117 script"));
    ExtrusionPropertyCustomGcode *untyped_script_property = untyped_script.get_property<ExtrusionPropertyCustomGcode>();
    REQUIRE(untyped_script_property != nullptr);
    untyped_script_property->script_type = GCODE_SCRIPT_TYPE_INVALID;
    CHECK_THROWS_AS(firmware.write_event(slic3r_api::ExtrusionEntity(
                        reinterpret_cast<const extrusion_entity_handle *>(&untyped_script))),
                    std::invalid_argument);
}
TEST_CASE("The generic firmware session keeps every tool environment independent",
          "[plugins][gcode][firmware]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    configure_standard_firmware(print, 2);
    // Isolate runtime command encoding from automatic startup requests.
    const_cast<PrintConfig &>(print.config()).start_gcode_manual.value = true;
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    append_multi_tool_process_sequence(plan.groups.front().layers.front());

    FirmwareInstanceOwner firmware(slic3r_api::make_gcode_firmware_instance(
        std::unique_ptr<slic3r_api::GCodeFirmwareSession>(
            new slic3r_api::GCodeGeneration::DefaultGCodeFirmwareSession())));
    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    try {
        run_file_writer_with_firmware(print, output_path, firmware.instance());
        const std::string output = read_text_file(output_path);
        INFO(output);
        CHECK(count_occurrences(output, "M104 S200 T0\n") == 1);
        CHECK(count_occurrences(output, "M104 S210 T1\n") == 1);
        CHECK(count_occurrences(output, "M106 S51\n") == 1);
        CHECK(count_occurrences(output, "M106 S77\n") == 1);
    } catch (...) {
        remove_output_pair(output_path);
        throw;
    }
    remove_output_pair(output_path);
}
TEST_CASE("Default firmware rejects missing process properties and unknown tools",
          "[plugins][gcode][firmware]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    SECTION("printable leaf without valid flow attributes") {
        Print print;
        select_printing_plan_firmware(print);
        configure_standard_firmware(print);
        PrintingPlan &plan = print.mutable_printing_plan();
        plan.groups.emplace_back();
        plan.groups.front().layers.emplace_back();
        PrintingLayerGroup &layer = plan.groups.front().layers.front();
        layer.tool_groups.emplace_back();
        PrintingToolGroup &tool_group = layer.tool_groups.front();
        tool_group.extruder_id = 0;

        // A printable role with default attributes has no usable mm3_per_mm,
        // even though its process speed and acceleration are otherwise valid.
        std::unique_ptr<ExtrusionPath> path(new ExtrusionPath(
            ArcPolyline(Points{Point(scale_i(0.0), scale_i(0.0)), Point(scale_i(1.0), scale_i(0.0))}),
            ExtrusionAttributes(ExtrusionRole::Perimeter),
            nullptr,
            true));
        path->add_property(ExtrusionPropertySpeed(20.f, 500.f));
        append_path_extrusion(tool_group, std::move(path));

        const boost::filesystem::path output_path = temporary_gcode_path();
        remove_output_pair(output_path);
        CHECK_THROWS_AS(
            Steps::StepGenerateGcode::run_step(Orchestrator::instance(), print, output_path.string()),
            RuntimeError);
        Orchestrator::instance().reset_plugin_cancel();
        CHECK_FALSE(boost::filesystem::exists(output_path));
        CHECK_FALSE(boost::filesystem::exists(output_path.string() + ".tmp"));
    }

    SECTION("printable leaf without speed") {
        Print print;
        select_printing_plan_firmware(print);
        configure_standard_firmware(print);
        PrintingPlan &plan = print.mutable_printing_plan();
        plan.groups.emplace_back();
        plan.groups.front().layers.emplace_back();
        PrintingLayerGroup &layer = plan.groups.front().layers.front();
        layer.tool_groups.emplace_back();
        PrintingToolGroup &tool_group = layer.tool_groups.front();
        tool_group.extruder_id = 0;
        append_path_extrusion(
            tool_group,
            make_firmware_path(
                ArcPolyline(Points{Point(scale_i(0.0), scale_i(0.0)), Point(scale_i(1.0), scale_i(0.0))}),
                -1.f,
                500.f));

        const boost::filesystem::path output_path = temporary_gcode_path();
        remove_output_pair(output_path);
        write_text_file(output_path, "previous output\n");
        CHECK_THROWS_AS(
            Steps::StepGenerateGcode::run_step(Orchestrator::instance(), print, output_path.string()),
            RuntimeError);
        Orchestrator::instance().reset_plugin_cancel();
        CHECK(read_text_file(output_path) == "previous output\n");
        CHECK_FALSE(boost::filesystem::exists(output_path.string() + ".tmp"));
        remove_output_pair(output_path);
    }

    SECTION("printable leaf without acceleration") {
        Print print;
        select_printing_plan_firmware(print);
        configure_standard_firmware(print);
        PrintingPlan &plan = print.mutable_printing_plan();
        plan.groups.emplace_back();
        plan.groups.front().layers.emplace_back();
        PrintingLayerGroup &layer = plan.groups.front().layers.front();
        layer.tool_groups.emplace_back();
        PrintingToolGroup &tool_group = layer.tool_groups.front();
        tool_group.extruder_id = 0;
        append_path_extrusion(
            tool_group,
            make_firmware_path(
                ArcPolyline(Points{Point(scale_i(0.0), scale_i(0.0)), Point(scale_i(1.0), scale_i(0.0))}),
                20.f,
                -1.f));

        const boost::filesystem::path output_path = temporary_gcode_path();
        remove_output_pair(output_path);
        Steps::StepGenerateGcode::run_step(
            Orchestrator::instance(), print, output_path.string());
        const std::string output = read_text_file(output_path);
        CHECK(output.find("M204") == std::string::npos);
        CHECK(output.find("G1 X1 E") != std::string::npos);
        remove_output_pair(output_path);
    }

    SECTION("tool group outside the configured range") {
        Print print;
        select_printing_plan_firmware(print);
        configure_standard_firmware(print);
        PrintingPlan &plan = print.mutable_printing_plan();
        plan.groups.emplace_back();
        plan.groups.front().layers.emplace_back();
        PrintingLayerGroup &layer = plan.groups.front().layers.front();
        layer.tool_groups.emplace_back();
        PrintingToolGroup &tool_group = layer.tool_groups.front();
        tool_group.extruder_id = 4;
        append_empty_extrusion(tool_group);

        const boost::filesystem::path output_path = temporary_gcode_path();
        remove_output_pair(output_path);
        CHECK_THROWS_AS(
            Steps::StepGenerateGcode::run_step(Orchestrator::instance(), print, output_path.string()),
            RuntimeError);
        Orchestrator::instance().reset_plugin_cancel();
        CHECK_FALSE(boost::filesystem::exists(output_path));
        CHECK_FALSE(boost::filesystem::exists(output_path.string() + ".tmp"));
    }
}

// Confirm that the preamble brackets start-script modes while startup heater commands come from the temperature plugin.
TEST_CASE("Preamble brackets start script before priming and restores profile state",
          "[plugins][gcode][firmware][preamble]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool manual = GENERATE(false, true);
    Print print;
    select_printing_plan_firmware(print);
    configure_standard_firmware(print);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.start_gcode_manual.value = manual;
    config.first_layer_temperature.set(std::vector<int>{215});
    config.first_layer_bed_temperature.set(std::vector<int>{60});
    config.chamber_temperature.set(std::vector<int>{0});
    config.use_relative_e_distances.value = false;
    append_speed_validation_tool_group(print, scale_i(0.2), 0);
    PrintingPlan &plan = print.mutable_printing_plan();
    ExtrusionNop start(ExtrusionPropertyCustomGcodeText(ExtrusionPropertyCustomGcodeText::Code::SCRIPT,
        "G20\nG91\nM83\nM104 S160\nM140 S35\n; UNIQUE_START"));
    start.get_property<ExtrusionPropertyCustomGcode>()->script_type = GCODE_SCRIPT_TYPE_START_GCODE;
    plan.events.append_before(std::move(start));
    // The next event is a real priming movement, owned by a preceding plugin.
    plan.events.append_before(*make_firmware_path(
        ArcPolyline(Points{Point(0, 0), Point(scale_i(1), 0)}), 20.f, 400.f));
    const boost::filesystem::path path = temporary_gcode_path();
    remove_output_pair(path);
    try {
        if (manual) {
            // A manual start must select its own initial tool before priming.
            // This test exercises commands only; no implicit setup is expected.
            plan.events.clear();
            ExtrusionNop event(ExtrusionPropertyCustomGcodeText(ExtrusionPropertyCustomGcodeText::Code::SCRIPT,
                "G20\nG91\nM83\n; UNIQUE_START"));
            event.get_property<ExtrusionPropertyCustomGcode>()->script_type = GCODE_SCRIPT_TYPE_START_GCODE;
            plan.events.append_before(std::move(event));
        }
        Steps::StepGenerateGcode::run_step(Orchestrator::instance(), print, path.string());
        const std::string text = read_text_file(path);
        INFO(text);
        CHECK(count_occurrences(text, "UNIQUE_START") == 1);
        if (manual) {
            CHECK(text.find("G21") == std::string::npos);
            CHECK(text.find("M109") == std::string::npos);
            CHECK(text.find("M190") == std::string::npos);
        } else {
            const size_t start_pos = text.find("UNIQUE_START");
            CHECK(text.find("G21", start_pos) != std::string::npos);
            CHECK(text.find("G90", start_pos) != std::string::npos);
            CHECK(text.find("M82", start_pos) != std::string::npos);
            CHECK(text.find("M109", start_pos) == std::string::npos);
            CHECK(text.find("M190", start_pos) == std::string::npos);
            CHECK(text.find("S215", start_pos) == std::string::npos);
            CHECK(text.find("S60", start_pos) == std::string::npos);
            CHECK(count_occurrences(text, "G1 X1") == 1);
        }
    } catch (...) { remove_output_pair(path); throw; }
    remove_output_pair(path);
}

TEST_CASE("Thumbnail renderer exposes borrowed RGBA8 pixels and render parameters",
          "[plugins][gcode][firmware][preamble][thumbnails]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    size_t calls = 0;
    GCodeThumbnailRenderer host(print, [&](const ThumbnailsParams &params) {
        ++calls;
        REQUIRE(params.sizes.size() == 1);
        CHECK(params.sizes[0] == Vec2d(4, 4));
        CHECK(params.show_bed);
        CHECK(params.camera == RAW_GCODE_THUMBNAIL_CAMERA_ISOMETRIC);
        CHECK(params.projection == RAW_GCODE_THUMBNAIL_PROJECTION_ORTHOGRAPHIC);
        CHECK(params.color_mode == RAW_GCODE_THUMBNAIL_COLOR_UNIFORM);
        ThumbnailData data;
        data.set(4, 4);
        std::fill(data.pixels.begin(), data.pixels.end(), 255);
        return ThumbnailsList{data};
    });
    raw_gcode_thumbnail_render_request request{};
    request.struct_size = sizeof(request);
    request.width = 4;
    request.height = 4;
    request.camera = RAW_GCODE_THUMBNAIL_CAMERA_REAR;
    request.projection = RAW_GCODE_THUMBNAIL_PROJECTION_PERSPECTIVE;
    request.framing = RAW_GCODE_THUMBNAIL_FRAMING_AUTO;
    request.color_mode = RAW_GCODE_THUMBNAIL_COLOR_UNIFORM;
    request.background = RAW_GCODE_THUMBNAIL_BACKGROUND_TRANSPARENT;
    request.show_bed = 1;
    request.custom_scale = 1.0;

    const slic3r_api::GCodeThumbnailRendererView view(host.c_renderer());
    const slic3r_api::GCodeThumbnailRenderResult rendered = view.render(request);
    REQUIRE(rendered.status == RAW_GCODE_THUMBNAIL_STATUS_SUCCESS);
    CHECK(rendered.image.width() == 4);
    CHECK(rendered.image.height() == 4);
    CHECK(rendered.image.stride_bytes() == 16);
    CHECK(rendered.image.pixels_size() == 64);
    CHECK(rendered.image.pixels()[0] == 255);
    CHECK(calls == 1);

    GCodeThumbnailRenderer absent(print, {});
    CHECK(slic3r_api::GCodeThumbnailRendererView(absent.c_renderer()).render(request).status ==
          RAW_GCODE_THUMBNAIL_STATUS_SKIPPED);
    request.width = 0;
    CHECK_THROWS(view.render(request));
    request.width = 4;
    GCodeThumbnailRenderer failing(print, [](const ThumbnailsParams &) -> ThumbnailsList {
        throw std::runtime_error("renderer failed");
    });
    CHECK_THROWS_WITH(slic3r_api::GCodeThumbnailRendererView(failing.c_renderer()).render(request),
                      "renderer failed");
    print.cancel();
    CHECK_THROWS(view.render(request));
}

TEST_CASE("Built-in thumbnail providers expose one compatible exclusive selector",
          "[plugins][gcode][firmware][preamble][thumbnails][metadata]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    const std::vector<std::string> provider_ids = {
        "PNG", "JPG",
        "QOI", "BIQU"
    };
    for (const std::string &provider_id : provider_ids) {
        const Plugin *plugin = orchestrator.get_plugin(provider_id);
        REQUIRE(plugin != nullptr);
        CHECK(plugin->get_step() == GCODE_THUMBNAIL);
        CHECK(plugin->get_exclusive_group() == "thumbnail_encoder_plugin");
        CHECK(orchestrator.is_plugin_active(plugin));
    }

    const Plugin *settings_owner = orchestrator.get_plugin(provider_ids.front());
    REQUIRE(settings_owner != nullptr);
    const std::vector<Plugin::UsedConfigKey> used = settings_owner->get_used_config_keys();
    CHECK_FALSE(used.empty()); // Typed metadata is required for definition ownership.
    for (const Orchestrator::PluginGuiRule &rule : orchestrator.gui_rules().rules())
        for (const Orchestrator::PluginGuiRuleClause &clause : rule.clauses)
            CHECK(clause.condition_key != "thumbnails_format");
    for (size_t provider_idx = 1; provider_idx < provider_ids.size(); ++provider_idx) {
        const Plugin *plugin = orchestrator.get_plugin(provider_ids[provider_idx]);
        REQUIRE(plugin != nullptr);
        CHECK(plugin->get_defined_config_keys().size() == 2);
    }

    for (const char *key : {"thumbnail_camera", "thumbnail_camera_position", "thumbnail_camera_target",
                            "thumbnail_camera_up", "thumbnail_projection", "thumbnail_framing", "thumbnail_custom_scale"}) {
        CHECK(PrintConfigDef::instance().get(key) == nullptr);
    }
    REQUIRE(PrintConfigDef::instance().get("thumbnail_background") != nullptr);
    REQUIRE(PrintConfigDef::instance().get("thumbnail_background_color") != nullptr);
    REQUIRE(PrintConfigDef::instance().get("thumbnail_color_mode") != nullptr);

    Print print;
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    CHECK(config.option("thumbnails_format")->serialize() == "PNG");
    const std::vector<std::string> expected = provider_ids;
    for (size_t format_idx = 0; format_idx < expected.size(); ++format_idx) {
        config.set_deserialize("thumbnails_format", expected[format_idx]);
        Plugin *selected = Steps::selected_active_plugin_from_group(
            orchestrator, GCODE_THUMBNAIL, "thumbnail_encoder_plugin", &print.config());
        REQUIRE(selected != nullptr);
        CHECK(selected->get_id() == expected[format_idx]);
    }

    config.set_deserialize("thumbnails_format", "JPG");
    CHECK(config.option("thumbnails_format")->serialize() == "JPG");
    CHECK(orchestrator.set_plugin_active("PNG", false));
    Plugin *without_png = Steps::selected_active_plugin_from_group(
        orchestrator, GCODE_THUMBNAIL, "thumbnail_encoder_plugin", &config);
    CHECK(without_png != nullptr);
    if (without_png) CHECK(without_png->get_id() == "JPG");
    config.set_deserialize("thumbnails_format", "PNG");
    CHECK(Steps::selected_active_plugin_from_group(
        orchestrator, GCODE_THUMBNAIL, "thumbnail_encoder_plugin", &config) == nullptr);
    CHECK(orchestrator.set_plugin_active("PNG", true));
    size_t selector_fragments = 0, background_fragments = 0;
    for (const Orchestrator::PluginUiFragment &fragment : orchestrator.ui_fragments_for_file("printer_fff.ui")) {
        if (fragment.fragment_id == "thumbnail_encoder_plugin") ++selector_fragments;
        if (fragment.fragment_id == "thumbnail_render_settings") ++background_fragments;
    }
    CHECK(selector_fragments == 1);
    CHECK(background_fragments == 1);
    for (const Orchestrator::PluginUiFragment &fragment : orchestrator.ui_fragments_for_file("print.ui"))
        CHECK(fragment.fragment_id != "thumbnail_encoder_plugin");

    config.thumbnails.set(std::vector<Vec2d>{Vec2d(4.5, 4)});
    slic3r_api::GCodeGeneration::DefaultGCodeFirmwareSession firmware;
    CHECK_THROWS_WITH(
        slic3r_api::default_gcode_thumbnail_requests(
            slic3r_api::Print(reinterpret_cast<const print_handle *>(&print)).config()),
        "Thumbnail dimensions must be finite positive integers.");
}

TEST_CASE("Thumbnail encoder adapter contains errors and owns its session once",
          "[plugins][gcode][firmware][preamble][thumbnails][adapter]")
{
    class RecordingSession final : public slic3r_api::GCodeThumbnailEncoderSession
    {
    public:
        explicit RecordingSession(uint32_t &destroyed) : m_destroyed(destroyed) {}
        ~RecordingSession() override { ++m_destroyed; }

        std::vector<uint8_t> encode(
            const raw_gcode_thumbnail_render_request &request) override
        {
            if (request.width == 7)
                throw std::invalid_argument("bad thumbnail request");
            if (request.width == 8)
                throw std::runtime_error("codec failed");
            return {uint8_t(request.width), 0, uint8_t(request.height)};
        }

    private:
        uint32_t &m_destroyed;
    };

    uint32_t destroyed = 0;
    raw_gcode_thumbnail_encoder_instance instance =
        slic3r_api::make_gcode_thumbnail_encoder_instance(
            std::unique_ptr<slic3r_api::GCodeThumbnailEncoderSession>(
                new RecordingSession(destroyed)));
    {
        const slic3r_api::GCodeThumbnailEncoderView view(&instance);
        raw_gcode_thumbnail_render_request request = {};
        request.struct_size = sizeof(request);
        request.width = 4;
        request.height = 3;
        request.custom_scale = 1.0;
        const slic3r_api::GCodeThumbnailEncodedBlock block = view.encode(request);
        REQUIRE(block.status == RAW_GCODE_THUMBNAIL_STATUS_SUCCESS);
        CHECK(std::vector<uint8_t>(block.bytes.begin(), block.bytes.end()) ==
              std::vector<uint8_t>{4, 0, 3});

        request.width = 0;
        CHECK_THROWS_WITH(view.encode(request),
                          "The G-code thumbnail render request is invalid.");
        request.width = 7;
        CHECK_THROWS_WITH(view.encode(request), "bad thumbnail request");
        request.width = 8;
        CHECK_THROWS_WITH(view.encode(request), "codec failed");
    }
    instance.vtable->destroy(instance.session);
    CHECK(destroyed == 1);

    raw_gcode_thumbnail_encoder_instance incomplete = {};
    incomplete.struct_size = sizeof(incomplete);
    CHECK_THROWS(slic3r_api::GCodeThumbnailEncoderView(&incomplete));
}

namespace {

// Build an RGBA image whose top and bottom halves differ, so that any vertical flip changes the compressed bytes.
Slic3r::ThumbnailData asymmetric_thumbnail(uint32_t width, uint32_t height)
{
    Slic3r::ThumbnailData data;
    data.set(width, height);
    for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
            unsigned char *pixel = data.pixels.data() + (size_t(y) * width + x) * 4;
            pixel[0] = y < height / 2 ? 255 : 0;
            pixel[1] = uint8_t(x * 20);
            pixel[2] = y < height / 2 ? 0 : 255;
            pixel[3] = 255;
        }
    }
    return data;
}

// Return the decoded payload of the base64 lines ("; <data>") of one framed thumbnail text block.
std::string decoded_framed_payload(const std::string &block)
{
    std::string encoded;
    std::istringstream lines(block);
    std::string line;
    while (std::getline(lines, line)) {
        if (line.size() > 2 && line[0] == ';' && line[1] == ' ' && line.find(' ', 2) == std::string::npos)
            encoded += line.substr(2);
    }
    std::string decoded(boost::beast::detail::base64::decoded_size(encoded.size()), '\0');
    const std::pair<std::size_t, std::size_t> sizes =
        boost::beast::detail::base64::decode(&decoded[0], encoded.data(), encoded.size());
    decoded.resize(sizes.first);
    return decoded;
}

// A render request of the given size with every other field at its neutral value.
raw_gcode_thumbnail_render_request raw_image_request(uint32_t width, uint32_t height)
{
    raw_gcode_thumbnail_render_request request = {};
    request.struct_size = sizeof(request);
    request.width = width;
    request.height = height;
    request.custom_scale = 1.0;
    return request;
}

} // namespace

// For PNG, JPG and QOI, the raw image is the compressed file of the rendered image: equal to the payload of the framed
// block of the same request and to a direct compression of the renderer's image (which fixes the orientation), with the
// signature, the format and the size of the request.
TEST_CASE("Built-in encoders return the compressed image without framing",
          "[plugins][gcode][thumbnails][raw-image]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    struct Codec { const char *id; Slic3r::GCodeThumbnailsFormat format; raw_gcode_thumbnail_encoded_format encoded; std::string signature; };
    const Codec codec = GENERATE(
        Codec{"PNG", Slic3r::GCodeThumbnailsFormat::PNG, RAW_GCODE_THUMBNAIL_ENCODED_FORMAT_PNG, std::string("\x89PNG", 4)},
        Codec{"JPG", Slic3r::GCodeThumbnailsFormat::JPG, RAW_GCODE_THUMBNAIL_ENCODED_FORMAT_JPG, std::string("\xFF\xD8", 2)},
        Codec{"QOI", Slic3r::GCodeThumbnailsFormat::QOI, RAW_GCODE_THUMBNAIL_ENCODED_FORMAT_QOI, std::string("qoif", 4)});
    INFO(codec.id);

    Print print;
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    const slic3r_api::Print print_view(reinterpret_cast<const print_handle *>(&print));
    const slic3r_api::Config config_view(ApiHost::to_config_handle(&config));
    const slic3r_api::OrchestratorView orch(reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
    GCodeThumbnailRenderer renderer(print, [](const ThumbnailsParams &params) {
        return ThumbnailsList{asymmetric_thumbnail(uint32_t(params.sizes.front().x()), uint32_t(params.sizes.front().y()))};
    });
    const slic3r_api::GCodeThumbnailEncoder encoder(orch, print_view, config_view, renderer.c_renderer(), codec.id);
    const slic3r_api::GCodeThumbnailEncoderView view = encoder.view();
    REQUIRE(view.supports_images());

    // A realistic thumbnail size.
    const raw_gcode_thumbnail_render_request request = raw_image_request(64, 48);
    const slic3r_api::GCodeThumbnailEncodedBlock framed = view.encode(request);
    REQUIRE(framed.status == RAW_GCODE_THUMBNAIL_STATUS_SUCCESS);
    const std::string framed_payload = decoded_framed_payload(std::string(framed.bytes));
    const slic3r_api::GCodeThumbnailEncodedImageBlock image = view.encode_image(request);
    REQUIRE(image.status == RAW_GCODE_THUMBNAIL_STATUS_SUCCESS);

    const std::string raw(image.bytes);
    const std::unique_ptr<GCodeThumbnails::CompressedImageBuffer> expected =
        GCodeThumbnails::compress_thumbnail(asymmetric_thumbnail(64, 48), codec.format);
    REQUIRE(expected->data != nullptr);
    CHECK(raw == std::string(static_cast<const char *>(expected->data), expected->size));
    CHECK(raw == framed_payload);
    CHECK(image.format == codec.encoded);
    CHECK(raw.compare(0, codec.signature.size(), codec.signature) == 0);
    CHECK(image.width == 64);
    CHECK(image.height == 48);
}

// A very small image compresses to more JPEG bytes than it has raw pixels, so the codec outgrows its first output buffer: two
// compressions still give the same complete file (start and end markers).
TEST_CASE("JPEG compression of a very small image is complete and repeatable",
          "[plugins][gcode][thumbnails]")
{
    const Slic3r::ThumbnailData image = asymmetric_thumbnail(8, 6);
    const std::unique_ptr<GCodeThumbnails::CompressedImageBuffer> first =
        GCodeThumbnails::compress_thumbnail(image, Slic3r::GCodeThumbnailsFormat::JPG);
    const std::unique_ptr<GCodeThumbnails::CompressedImageBuffer> second =
        GCodeThumbnails::compress_thumbnail(image, Slic3r::GCodeThumbnailsFormat::JPG);
    REQUIRE(first->data != nullptr);
    REQUIRE(second->data != nullptr);
    const std::string first_bytes(static_cast<const char *>(first->data), first->size);
    const std::string second_bytes(static_cast<const char *>(second->data), second->size);
    CHECK(first_bytes.size() > image.pixels.size());
    CHECK(first_bytes == second_bytes);
    CHECK(first_bytes.compare(0, 2, "\xFF\xD8", 2) == 0);
    REQUIRE(first_bytes.size() >= 2);
    CHECK(first_bytes.compare(first_bytes.size() - 2, 2, "\xFF\xD9", 2) == 0);
}

// BIQU is text only: its vtable has no raw-image callback and the view refuses to ask; a missing renderer skips the
// optional image of an encoder that supports them.
TEST_CASE("Raw images are unavailable for BIQU and skipped without a renderer",
          "[plugins][gcode][thumbnails][raw-image]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    const slic3r_api::Print print_view(reinterpret_cast<const print_handle *>(&print));
    const slic3r_api::Config config_view(ApiHost::to_config_handle(&config));
    const slic3r_api::OrchestratorView orch(reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
    const raw_gcode_thumbnail_render_request request = raw_image_request(8, 6);

    const slic3r_api::GCodeThumbnailEncoder biqu(orch, print_view, config_view, nullptr, "BIQU");
    CHECK_FALSE(biqu.view().supports_images());
    CHECK_THROWS_WITH(biqu.view().encode_image(request), "The G-code thumbnail encoder does not support raw images.");

    const slic3r_api::GCodeThumbnailEncoder jpg(orch, print_view, config_view, nullptr, "JPG");
    REQUIRE(jpg.view().supports_images());
    CHECK(jpg.view().encode_image(request).status == RAW_GCODE_THUMBNAIL_STATUS_SKIPPED);
}

namespace {

// Answer of a hand-written C encoder: whatever the test asks encode_image to return.
struct HandImageAnswer
{
    raw_gcode_thumbnail_status status = RAW_GCODE_THUMBNAIL_STATUS_SUCCESS;
    raw_gcode_thumbnail_encoded_format format = RAW_GCODE_THUMBNAIL_ENCODED_FORMAT_JPG;
    uint32_t width = 4;
    uint32_t height = 3;
    std::vector<uint8_t> bytes = {1, 2, 3};
    const char *error = nullptr;
};

void hand_destroy(void *) {}

void hand_encode(void *, const raw_gcode_thumbnail_render_request *, raw_gcode_thumbnail_encode_result *result)
{
    result->status = RAW_GCODE_THUMBNAIL_STATUS_SKIPPED;
}

void hand_encode_image(void *session, const raw_gcode_thumbnail_render_request *,
                       raw_gcode_thumbnail_encoded_image_result *result)
{
    const HandImageAnswer &answer = *static_cast<const HandImageAnswer *>(session);
    result->status = answer.status;
    result->format = answer.format;
    result->width = answer.width;
    result->height = answer.height;
    result->bytes = answer.bytes.empty() ? nullptr : answer.bytes.data();
    result->bytes_size = answer.bytes.size();
    result->error_message = answer.error;
}

// A session that implements encode_image and fails or skips according to the width of the request.
class RecordingImageSession final : public slic3r_api::GCodeThumbnailEncoderSession
{
public:
    explicit RecordingImageSession(uint32_t &destroyed) : m_destroyed(destroyed) {}
    ~RecordingImageSession() override { ++m_destroyed; }

    std::vector<uint8_t> encode(const raw_gcode_thumbnail_render_request &) override { return {1}; }
    bool supports_image() const noexcept override { return true; }
    slic3r_api::GCodeThumbnailEncodedImage encode_image(const raw_gcode_thumbnail_render_request &request) override
    {
        if (request.width == 7)
            throw std::invalid_argument("bad image request");
        if (request.width == 8)
            throw std::runtime_error("image codec failed");
        slic3r_api::GCodeThumbnailEncodedImage image;
        if (request.width == 9)
            return image;
        image.format = RAW_GCODE_THUMBNAIL_ENCODED_FORMAT_QOI;
        image.width = request.width;
        image.height = request.height;
        image.bytes = {uint8_t(request.width), 9, uint8_t(request.height)};
        return image;
    }

private:
    uint32_t &m_destroyed;
};

// A session that keeps the default: no raw image.
class TextOnlySession final : public slic3r_api::GCodeThumbnailEncoderSession
{
public:
    std::vector<uint8_t> encode(const raw_gcode_thumbnail_render_request &) override { return {1}; }
};

} // namespace

// The C++ adapter turns a session's answers and exceptions into statuses and the view into results or clear errors: a
// session without raw images publishes a null callback, and the view checks any provider's successful answer.
TEST_CASE("The thumbnail adapter and view check raw image answers",
          "[plugins][gcode][thumbnails][raw-image][adapter]")
{
    SECTION("a session that implements encode_image")
    {
        uint32_t destroyed = 0;
        raw_gcode_thumbnail_encoder_instance instance = slic3r_api::make_gcode_thumbnail_encoder_instance(
            std::unique_ptr<slic3r_api::GCodeThumbnailEncoderSession>(new RecordingImageSession(destroyed)));
        REQUIRE(instance.vtable->encode_image != nullptr);
        {
            const slic3r_api::GCodeThumbnailEncoderView view(&instance);
            REQUIRE(view.supports_images());
            const slic3r_api::GCodeThumbnailEncodedImageBlock image = view.encode_image(raw_image_request(4, 3));
            CHECK(image.status == RAW_GCODE_THUMBNAIL_STATUS_SUCCESS);
            CHECK(image.format == RAW_GCODE_THUMBNAIL_ENCODED_FORMAT_QOI);
            CHECK(image.width == 4);
            CHECK(image.height == 3);
            CHECK(std::vector<uint8_t>(image.bytes.begin(), image.bytes.end()) == std::vector<uint8_t>{4, 9, 3});

            CHECK_THROWS_WITH(view.encode_image(raw_image_request(7, 3)), "bad image request");
            CHECK_THROWS_WITH(view.encode_image(raw_image_request(8, 3)), "image codec failed");
            const slic3r_api::GCodeThumbnailEncodedImageBlock skipped = view.encode_image(raw_image_request(9, 3));
            CHECK(skipped.status == RAW_GCODE_THUMBNAIL_STATUS_SKIPPED);
            CHECK(skipped.bytes.empty());
        }
        instance.vtable->destroy(instance.session);
        CHECK(destroyed == 1);
    }

    SECTION("a session without raw images publishes a null callback")
    {
        raw_gcode_thumbnail_encoder_instance instance = slic3r_api::make_gcode_thumbnail_encoder_instance(
            std::unique_ptr<slic3r_api::GCodeThumbnailEncoderSession>(new TextOnlySession()));
        CHECK(instance.vtable->encode_image == nullptr);
        {
            const slic3r_api::GCodeThumbnailEncoderView view(&instance);
            CHECK_FALSE(view.supports_images());
            CHECK_THROWS_WITH(view.encode_image(raw_image_request(4, 3)),
                              "The G-code thumbnail encoder does not support raw images.");
        }
        instance.vtable->destroy(instance.session);
    }

    SECTION("the view checks the answer of any encoder")
    {
        HandImageAnswer answer;
        raw_gcode_thumbnail_encoder_vtable vtable = {sizeof(vtable), &hand_destroy, &hand_encode, &hand_encode_image};
        raw_gcode_thumbnail_encoder_instance instance = {sizeof(instance), &answer, &vtable};
        const slic3r_api::GCodeThumbnailEncoderView view(&instance);
        const raw_gcode_thumbnail_render_request request = raw_image_request(4, 3);

        CHECK(view.encode_image(request).format == RAW_GCODE_THUMBNAIL_ENCODED_FORMAT_JPG);
        answer.format = RAW_GCODE_THUMBNAIL_ENCODED_FORMAT_UNSET;
        CHECK_THROWS_WITH(view.encode_image(request), "The G-code thumbnail encoder returned an unknown image format.");
        answer.format = RAW_GCODE_THUMBNAIL_ENCODED_FORMAT_QOI;
        answer.bytes.clear();
        CHECK_THROWS_WITH(view.encode_image(request), "The G-code thumbnail encoder returned no image bytes.");
        answer.bytes = {1};
        answer.width = 5;
        CHECK_THROWS_WITH(view.encode_image(request),
                          "The G-code thumbnail encoder returned an image of unexpected dimensions.");
        answer.width = 4;
        answer.height = 2;
        CHECK_THROWS_WITH(view.encode_image(request),
                          "The G-code thumbnail encoder returned an image of unexpected dimensions.");
        answer.height = 3;
        answer.status = RAW_GCODE_THUMBNAIL_STATUS_ERROR;
        answer.error = "provider failed";
        CHECK_THROWS_WITH(view.encode_image(request), "provider failed");
    }

    SECTION("a null encode_image is accepted, a vtable of the version 1 size is refused")
    {
        HandImageAnswer answer;
        raw_gcode_thumbnail_encoder_vtable vtable = {sizeof(vtable), &hand_destroy, &hand_encode, nullptr};
        raw_gcode_thumbnail_encoder_instance instance = {sizeof(instance), &answer, &vtable};
        const slic3r_api::GCodeThumbnailEncoderView view(&instance);
        CHECK_FALSE(view.supports_images());
        CHECK_THROWS(view.encode_image(raw_image_request(4, 3)));

        vtable.struct_size = uint32_t(offsetof(raw_gcode_thumbnail_encoder_vtable, encode_image));
        CHECK_THROWS(slic3r_api::GCodeThumbnailEncoderView(&instance));
    }
}

// The firmware preamble selects only the initial tool and leaves automatic heating to the temperature plugin.
TEST_CASE("Preamble leaves automatic heating to the temperature plugin",
          "[plugins][gcode][firmware][preamble]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool mmu = GENERATE(false, true);
    const char *firmware_id = GENERATE("gcode.firmware.marlin2", "gcode.firmware.klipper");
    Print print;
    select_printing_plan_firmware(print);
    configure_standard_firmware(print, 2);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.single_extruder_multi_material.value = mmu;
    config.first_layer_temperature.set(std::vector<int>{205, 220});
    PrintingToolGroup &first = append_speed_validation_tool_group(print, scale_i(0.2), 1);
    (void)first;
    print.mutable_printing_plan().groups[0].layers[0].tool_groups.emplace_back();
    print.mutable_printing_plan().groups[0].layers[0].tool_groups.back().extruder_id = 0;
    const std::string text = export_with_firmware(print, firmware_id);
    INFO(text);
    const size_t first_layer = text.find(";LAYER_CHANGE\n");
    REQUIRE(first_layer != std::string::npos);
    const std::string preamble = text.substr(0, first_layer);
    CHECK(text.find("S220") == std::string::npos);
    CHECK(text.find("S205") == std::string::npos);
    CHECK(text.find("M109") == std::string::npos);
    CHECK(text.find("M190") == std::string::npos);
    const std::string initial_selection = std::string(firmware_id) == "gcode.firmware.klipper" && !mmu ?
        "ACTIVATE_EXTRUDER EXTRUDER=extruder1\n" : "T1\n";
    CHECK(preamble.find(initial_selection + "G92 E0\n") != std::string::npos);
    if (!mmu) {
        if (std::string(firmware_id) == "gcode.firmware.marlin2")
            CHECK(preamble.find("T0\n") == std::string::npos);
        else
            CHECK(preamble.find("ACTIVATE_EXTRUDER EXTRUDER=extruder\n") == std::string::npos);
    }
}

// Export a reference with the built-in text exporter, using a configuration copy and a render callback, and return the exported text.
static std::string export_with_simple_exporter(Print &print,
                                               const DynamicPrintConfig &config,
                                               const boost::filesystem::path &reference,
                                               const ThumbnailsGeneratorCallback &render)
{
    Orchestrator &orchestrator = Orchestrator::instance();
    const Orchestrator::GCodeExporterSelection selection = orchestrator.select_gcode_exporter(print, config);
    REQUIRE(selection.plugin_id == "gcode.exporter.simple");
    const boost::filesystem::path destination = temporary_gcode_path();
    remove_output_pair(destination);
    try {
        Test::export_print_to(orchestrator, print, config, reference.string(), destination.string(), selection, render);
        const std::string text = read_text_file(destination);
        remove_output_pair(destination);
        return text;
    } catch (...) {
        remove_output_pair(destination);
        throw;
    }
}

// Export each thumbnail format and placement to test-scoped files, then require a unique exact reference, adjacent configuration, and an end thumbnail immediately after that marker through EOF; Catch failures report boundary mismatches.
TEST_CASE("Exporter chooses thumbnail placement including BIQU at byte zero",
          "[plugins][gcode][exporter][thumbnails]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const std::string id = GENERATE("PNG", "JPG", "QOI", "BIQU");
    const bool at_end = GENERATE(false, true);
    Print print;
    select_printing_plan_firmware(print);
    configure_standard_firmware(print);
    append_speed_validation_tool_group(print, scale_i(0.2), 0);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"thumbnails_format", id}, {"thumbnails_end_file", at_end ? "1" : "0"},
        {"thumbnails_tag_format", "1"}, {"thumbnails", "4x4,3x2"}});
    uint32_t calls = 0;
    std::vector<Vec2d> rendered_sizes;
    const ThumbnailsGeneratorCallback render = [&calls, &rendered_sizes](const ThumbnailsParams &params) {
        ++calls;
        REQUIRE(params.sizes.size() == 1);
        rendered_sizes.push_back(params.sizes.front());
        ThumbnailData data;
        data.set(uint32_t(params.sizes.front().x()), uint32_t(params.sizes.front().y()));
        std::fill(data.pixels.begin(), data.pixels.end(), 255);
        return ThumbnailsList{data};
    };
    const boost::filesystem::path path = temporary_gcode_path();
    remove_output_pair(path);
    try {
        Steps::StepGenerateGcode::run_step(Orchestrator::instance(), print, path.string());
        const std::string reference = read_text_file(path);
        // The reference holds G-code only, whatever the thumbnail settings.
        CHECK(reference.find("generated by") == std::string::npos);
        CHECK(reference.find("thumbnail") == std::string::npos);
        CHECK(calls == 0);

        const std::string text = export_with_simple_exporter(print, config, path, render);
        INFO(text);
        CHECK(calls == 2);
        CHECK(rendered_sizes == std::vector<Vec2d>{Vec2d(4, 4), Vec2d(3, 2)});
        const bool biqu = id == "BIQU";
        const std::string marker = biqu ? ";00040004" :
            id == "PNG" ? "; thumbnail_PNG begin" :
            id == "JPG" ? "; thumbnail_JPG begin" : "; thumbnail_QOI begin";
        const std::string producer = "; generated by " SLIC3R_APP_KEY " " SLIC3R_VERSION " on\n\n";
        CHECK(count_occurrences(text, marker) == (biqu ? 1 : 2));
        if (biqu)
            CHECK(count_occurrences(text, ";00030002") == 1);
        if (biqu) {
            CHECK(text.find(marker) == 0);
            CHECK(text.find(producer) > 0);
        } else {
            CHECK(text.compare(0, producer.size(), producer) == 0);
            if (at_end) {
                CHECK(text.find(marker) > text.find("G21"));
            } else {
                CHECK(text.find(marker) < text.find("G21"));
            }
        }
        const size_t reference_position = text.find(reference, producer.size());
        REQUIRE(reference_position != std::string::npos);
        CHECK(text.compare(reference_position, reference.size(), reference) == 0);
        CHECK(text.find(reference, reference_position + 1) == std::string::npos);
        const size_t width_position = text.find("; external perimeters extrusion width = ", producer.size());
        if (width_position != std::string::npos)
            CHECK(width_position < reference_position);
        const std::string config_begin = "\n; " SLIC3R_APP_NAME "_config = begin\n";
        const size_t body_end = reference_position + reference.size();
        REQUIRE(text.compare(body_end, config_begin.size(), config_begin) == 0);
        const std::string config_end = "; " SLIC3R_APP_NAME "_config = end\n";
        const size_t config_end_position = text.find(config_end, body_end + config_begin.size());
        REQUIRE(config_end_position != std::string::npos);
        const std::string_view configuration_block(text.data() + body_end + config_begin.size(),
                                                   config_end_position - body_end - config_begin.size());
        std::string first_bad_configuration_line;
        CHECK_MESSAGE(Slic3r::Test::Plugins::GCodeExportTest::is_configuration_block(
                          configuration_block, first_bad_configuration_line),
                      "Malformed configuration line: " << first_bad_configuration_line);
        if (biqu || !at_end) {
            REQUIRE(text.size() >= config_end.size());
            CHECK(text.compare(text.size() - config_end.size(), config_end.size(), config_end) == 0);
        } else {
            const std::string thumbnail_end = "; thumbnail_" + id + " end\n;\n";
            REQUIRE(text.size() >= thumbnail_end.size());
            CHECK(text.compare(text.size() - thumbnail_end.size(), thumbnail_end.size(), thumbnail_end) == 0);
            const std::string thumbnail_separator = "\n;\n";
            const size_t end_thumbnail_start = config_end_position + config_end.size();
            CHECK(text.compare(end_thumbnail_start, thumbnail_separator.size(), thumbnail_separator) == 0);
            const size_t end_thumbnail_position = text.find(marker, body_end);
            CHECK(end_thumbnail_position == end_thumbnail_start + thumbnail_separator.size());
        }
    } catch (...) { remove_output_pair(path); throw; }
    remove_output_pair(path);
}

// Export requested thumbnails without a GUI renderer to test-scoped files; require the reference immediately after the producer line, adjacent configuration, a filtered footer at EOF, and no thumbnail block, reporting mismatches through Catch.
TEST_CASE("Optional thumbnails are skipped when no GUI renderer is available",
          "[plugins][gcode][exporter][thumbnails]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    select_printing_plan_firmware(print);
    configure_standard_firmware(print);
    append_speed_validation_tool_group(print, scale_i(0.2), 0);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict("thumbnails", "4x4");
    const boost::filesystem::path path = temporary_gcode_path();
    remove_output_pair(path);
    try {
        Steps::StepGenerateGcode::run_step(Orchestrator::instance(), print, path.string());
        const std::string reference = read_text_file(path);
        const std::string text = export_with_simple_exporter(print, config, path, ThumbnailsGeneratorCallback());
        CHECK(text.find("thumbnail begin") == std::string::npos);
        const std::string producer = "; generated by " SLIC3R_APP_KEY " " SLIC3R_VERSION " on\n\n";
        REQUIRE(text.compare(0, producer.size(), producer) == 0);
        const size_t reference_position = text.find(reference, producer.size());
        REQUIRE(reference_position != std::string::npos);
        CHECK(reference_position == producer.size());
        CHECK(text.compare(reference_position, reference.size(), reference) == 0);
        CHECK(text.find(reference, reference_position + 1) == std::string::npos);
        const size_t width_position = text.find("; external perimeters extrusion width = ", producer.size());
        if (width_position != std::string::npos)
            CHECK(width_position < reference_position);
        const std::string config_begin = "\n; " SLIC3R_APP_NAME "_config = begin\n";
        CHECK(text.compare(reference_position + reference.size(), config_begin.size(), config_begin) == 0);
        const std::string config_end = "; " SLIC3R_APP_NAME "_config = end\n";
        const size_t config_begin_position = reference_position + reference.size() + config_begin.size();
        const size_t config_end_position = text.find(config_end, config_begin_position);
        REQUIRE(config_end_position != std::string::npos);
        const std::string_view configuration_block(text.data() + config_begin_position,
                                                   config_end_position - config_begin_position);
        std::string first_bad_configuration_line;
        CHECK_MESSAGE(Slic3r::Test::Plugins::GCodeExportTest::is_configuration_block(
                          configuration_block, first_bad_configuration_line),
                      "Malformed configuration line: " << first_bad_configuration_line);
        REQUIRE(text.size() >= config_end.size());
        CHECK(text.compare(text.size() - config_end.size(), config_end.size(), config_end) == 0);
    } catch (...) { remove_output_pair(path); throw; }
    remove_output_pair(path);
}

TEST_CASE("Thumbnail rendering failure preserves an existing G-code file",
          "[plugins][gcode][exporter][thumbnails][atomic]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    select_printing_plan_firmware(print);
    configure_standard_firmware(print);
    append_speed_validation_tool_group(print, scale_i(0.2), 0);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict("thumbnails", "4x4");
    const boost::filesystem::path reference = temporary_gcode_path();
    const boost::filesystem::path destination = temporary_gcode_path();
    remove_output_pair(reference);
    remove_output_pair(destination);
    try {
        Steps::StepGenerateGcode::run_step(Orchestrator::instance(), print, reference.string());
        write_text_file(destination, "previous output\n");
        Orchestrator &orchestrator = Orchestrator::instance();
        const Orchestrator::GCodeExporterSelection selection = orchestrator.select_gcode_exporter(print, config);
        const ThumbnailsGeneratorCallback failing = [](const ThumbnailsParams &) -> ThumbnailsList {
            throw std::runtime_error("thumbnail rendering failed before publication");
        };
        CHECK_THROWS_WITH(Test::export_print_to(orchestrator, print, config, reference.string(), destination.string(), selection, failing),
                          Catch::Matchers::Contains("thumbnail rendering failed before publication"));
        CHECK(read_text_file(destination) == "previous output\n");
        // The failure happens before any output file exists, and no private file of this export stays beside the destination.
        const std::string private_prefix = destination.stem().string() + ".export-";
        std::string private_files;
        for (boost::filesystem::directory_iterator it(destination.parent_path()), end; it != end; ++it)
            if (it->path().filename().string().compare(0, private_prefix.size(), private_prefix) == 0)
                private_files += it->path().filename().string() + " ";
        INFO("private files: " << private_files);
        CHECK(private_files.empty());
        CHECK_FALSE(orchestrator.is_plugin_cancelled());
        // The same export succeeds once the renderer works.
        const ThumbnailsGeneratorCallback working = [](const ThumbnailsParams &params) {
            ThumbnailData data;
            data.set(uint32_t(params.sizes.front().x()), uint32_t(params.sizes.front().y()));
            std::fill(data.pixels.begin(), data.pixels.end(), 255);
            return ThumbnailsList{data};
        };
        Test::export_print_to(orchestrator, print, config, reference.string(), destination.string(), selection, working);
        CHECK(read_text_file(destination).find("thumbnail begin") != std::string::npos);
    } catch (...) { remove_output_pair(reference); remove_output_pair(destination); throw; }
    remove_output_pair(reference);
    remove_output_pair(destination);
}

TEST_CASE("Every export renders its own thumbnails",
          "[plugins][gcode][exporter][thumbnails]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    select_printing_plan_firmware(print);
    configure_standard_firmware(print);
    append_speed_validation_tool_group(print, scale_i(0.2), 0);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict("thumbnails", "4x4,3x2");
    uint32_t calls = 0;
    const ThumbnailsGeneratorCallback render = [&calls](const ThumbnailsParams &params) {
        ++calls;
        ThumbnailData data;
        data.set(uint32_t(params.sizes.front().x()), uint32_t(params.sizes.front().y()));
        std::fill(data.pixels.begin(), data.pixels.end(), 255);
        return ThumbnailsList{data};
    };
    const boost::filesystem::path path = temporary_gcode_path();
    remove_output_pair(path);
    try {
        Steps::StepGenerateGcode::run_step(Orchestrator::instance(), print, path.string());
        export_with_simple_exporter(print, config, path, render);
        CHECK(calls == 2);
        export_with_simple_exporter(print, config, path, render);
        CHECK(calls == 4);
    } catch (...) { remove_output_pair(path); throw; }
    remove_output_pair(path);
}

// Build an envelope from valid empty metadata and two renderer requests, then assert placement, normalized camera settings, and encoder failures without writing files.
TEST_CASE("Text envelope owns mixed thumbnail requests and ignores camera geometry",
          "[plugins][gcode][exporter][thumbnails]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    configure_standard_firmware(print);
    uint32_t calls = 0;
    GCodeThumbnailRenderer renderer(print, [&](const ThumbnailsParams &params) {
        ++calls;
        CHECK(params.camera == RAW_GCODE_THUMBNAIL_CAMERA_ISOMETRIC);
        CHECK(params.projection == RAW_GCODE_THUMBNAIL_PROJECTION_ORTHOGRAPHIC);
        CHECK(params.framing == RAW_GCODE_THUMBNAIL_FRAMING_AUTO);
        ThumbnailData image;
        image.set(4, 4);
        std::fill(image.pixels.begin(), image.pixels.end(), 255);
        return ThumbnailsList{image};
    });
    raw_gcode_thumbnail_render_request render = {};
    render.struct_size = sizeof(render);
    render.width = render.height = 4;
    render.with_format_tag = 1;
    render.camera = RAW_GCODE_THUMBNAIL_CAMERA_CUSTOM;
    render.projection = RAW_GCODE_THUMBNAIL_PROJECTION_PERSPECTIVE;
    render.framing = RAW_GCODE_THUMBNAIL_FRAMING_CUSTOM;
    // Zero custom vectors/scale are intentionally ignored by this host.
    const std::vector<slic3r_api::GCodeThumbnailRequest> requests = {
        {render, "JPG", RAW_GCODE_THUMBNAIL_FILE_PREFIX, true},
        {render, "QOI", RAW_GCODE_THUMBNAIL_FILE_SUFFIX, true}};

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    const slic3r_api::Print view(reinterpret_cast<const print_handle *>(&print));
    const slic3r_api::Config config_view(ApiHost::to_config_handle(&config));
    const slic3r_api::OrchestratorView orch(reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
    raw_gcode_export_metadata metadata = {};
    metadata.struct_size = uint32_t(sizeof(metadata));
    const slic3r_api::GCodeTextEnvelope envelope = slic3r_api::build_gcode_text_envelope(
        orch, view, config_view, renderer.c_renderer(), requests, &metadata);
    CHECK(envelope.header.find("thumbnail_JPG begin") < envelope.header.find("generated by"));
    CHECK(envelope.header.find("thumbnail_QOI") == std::string::npos);
    CHECK(envelope.trailer.find("thumbnail_QOI begin") != std::string::npos);
    CHECK(calls == 2);

    std::vector<slic3r_api::GCodeThumbnailRequest> unknown = {{render, "missing.encoder", RAW_GCODE_THUMBNAIL_FILE_PREFIX, true}};
    CHECK_THROWS(slic3r_api::build_gcode_text_envelope(orch, view, config_view, renderer.c_renderer(), unknown, &metadata));
    unknown.front().provider_id = "gcode.firmware.marlin2";
    CHECK_THROWS(slic3r_api::build_gcode_text_envelope(orch, view, config_view, renderer.c_renderer(), unknown, &metadata));
}
TEST_CASE("Thumbnail configuration preserves serialized ids and binary codec limits",
          "[plugins][gcode][thumbnails][metadata]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config;
    config.set_key_value("thumbnails", new ConfigOptionPoints(std::vector<Vec2d>{Vec2d(4, 4)}));
    for (const std::string &id : {"PNG", "JPG", "QOI", "BIQU"}) {
        config.set_deserialize("thumbnails_format", id);
        CHECK(config.option("thumbnails_format")->serialize() == id);
        DynamicPrintConfig copied;
        copied.apply(config);
        CHECK(copied.option("thumbnails_format")->serialize() == id);
        if (id == "BIQU") {
            CHECK_THROWS(GCodeThumbnails::make_and_check_thumbnail_list(copied));
        } else {
            const std::pair<GCodeThumbnails::GCodeThumbnailDefinitionsList, ThumbnailErrors> parsed =
                GCodeThumbnails::make_and_check_thumbnail_list(copied);
            REQUIRE(parsed.first.size() == 1);
        }
    }
    config.set_key_value("thumbnails_format", new ConfigOptionString("external.custom"));
    CHECK_THROWS(GCodeThumbnails::make_and_check_thumbnail_list(config));
    config.set_key_value("thumbnails", new ConfigOptionPoints());
    CHECK(GCodeThumbnails::make_and_check_thumbnail_list(config).first.empty());

    Print print;
    PrintConfig &settings = const_cast<PrintConfig &>(print.config());
    settings.thumbnails.set(std::vector<Vec2d>{Vec2d(4, 4)});
    settings.thumbnail_color_mode.value = ThumbnailColorMode::Uniform;
    const std::vector<slic3r_api::GCodeThumbnailRequest> requests =
        slic3r_api::default_gcode_thumbnail_requests(
            slic3r_api::Print(reinterpret_cast<const print_handle *>(&print)).config());
    REQUIRE(requests.size() == 1);
    CHECK(requests.front().render.color_mode == RAW_GCODE_THUMBNAIL_COLOR_UNIFORM);
}

#ifndef NDEBUG
// Deliberately enter the native exporter under this Print's active STEP_GCODE marker; the routing assertion
// must terminate before status or file effects. Explicit hidden-tag selection isolates the expected process failure.
TEST_CASE("Legacy export rejects the active pipeline G-code context",
          "[.][legacy-routing-assert]")
{
    const boost::filesystem::path output_path = boost::filesystem::absolute(
        boost::filesystem::path(TEST_DATA_DIR).parent_path().parent_path() /
        "build" / "temp" / boost::filesystem::unique_path("legacy-routing-%%%%-%%%%.gcode"));
    REQUIRE_FALSE(boost::filesystem::exists(output_path));
    REQUIRE_FALSE(boost::filesystem::exists(output_path.string() + ".tmp"));
    std::cout << "LEGACY_ROUTING_PROBE_PATH=" << output_path.string() << std::endl;
    Print print;
    print.set_running_step(STEP_GCODE);
    GCodeGenerator generator;
    generator.do_export(&print, output_path.string().c_str());
    FAIL("The active STEP_GCODE context must stop at the legacy exporter routing assertion");
}
#endif

TEST_CASE("Legacy text export uses the dynamic thumbnail service",
          "[plugins][gcode][legacy][thumbnails]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const std::string id = GENERATE("PNG", "JPG", "QOI", "BIQU");
    INFO("building full config");
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    INFO("deserializing config");
    config.set_deserialize_strict({
        {"thumbnails_format", id},
        {"thumbnails", "4x4"}, {"thumbnails_tag_format", "1"},
        {"thumbnails_end_file", id == "QOI" || id == "BIQU" ? "1" : "0"},
        {"start_gcode", ";thumbnail-test-start"}, {"end_gcode", ";thumbnail-test-end"}
    });
    Print print;
    Model model;
    INFO("initializing print");
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
    INFO("processing print");
    print.process();
    INFO("exporting print");
    uint32_t calls = 0;
    const ThumbnailsGeneratorCallback renderer = [&calls](const ThumbnailsParams &params) {
        ++calls;
        REQUIRE(params.sizes.size() == 1);
        ThumbnailData data;
        data.set(4, 4);
        std::fill(data.pixels.begin(), data.pixels.end(), 255);
        return ThumbnailsList{data};
    };
    const boost::filesystem::path path = temporary_gcode_path();
    remove_output_pair(path);
    try {
        GCodeGenerator generator;
        generator.do_export(&print, path.string().c_str(), nullptr, renderer);
        const std::string text = read_text_file(path);
        const std::string marker = id == "BIQU" ? ";00040004" : "; thumbnail_" + id + " begin";
        CHECK(calls == 1);
        CHECK(count_occurrences(text, marker) == 1);
        if (id == "BIQU") CHECK(text.find(marker) == 0);
        else if (id == "QOI") CHECK(text.find(marker) > text.find(";thumbnail-test-end"));
        else CHECK(text.find(marker) < text.find(";thumbnail-test-start"));
    } catch (...) { remove_output_pair(path); throw; }
    remove_output_pair(path);
}

TEST_CASE("Thumbnail palette migration preserves FFF and SLA profiles",
          "[plugins][gcode][thumbnails][config]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    for (const std::string &value : {"0", "1"}) {
        std::string key = "thumbnails_custom_color", converted = value;
        PrintConfigDef::handle_legacy_pair(key, converted);
        CHECK(key == "thumbnail_color_mode");
        CHECK(converted == (value == "1" ? "uniform" : "filament"));
        for (const std::string &palette : {"uniform", "extruder", "filament"}) {
            std::unordered_map<std::string, std::pair<std::string, std::string>> profile = {
                {"thumbnails_custom_color", {"thumbnails_custom_color", value}},
                {"thumbnail_color_mode", {"thumbnail_color_mode", palette}}
            };
            PrintConfigDef::handle_legacy_map(profile);
            CHECK(profile.at("thumbnails_custom_color").first.empty());
            CHECK(profile.at("thumbnail_color_mode").second == (value == "1" ? "uniform" : palette));
            DynamicPrintConfig loaded;
            loaded.load_from_ini_string("thumbnail_color_mode = " + palette +
                "\nthumbnails_custom_color = " + value + "\n", ForwardCompatibilitySubstitutionRule::Disable);
            CHECK(loaded.option("thumbnail_color_mode")->serialize() == (value == "1" ? "uniform" : palette));
            const boost::filesystem::path saved = temporary_gcode_path();
            loaded.save(saved.string(), ConfigWriteTarget::Local);
            const std::string serialized = read_text_file(saved);
            remove_output_pair(saved);
            CHECK(serialized.find("thumbnails_custom_color") == std::string::npos);
            CHECK(serialized.find("thumbnail_color_mode") != std::string::npos);
        }
    }
    CHECK(PrintConfigDef::instance().get("thumbnails_custom_color") == nullptr);
    for (const std::string &palette : {"uniform", "extruder", "filament"}) {
        DynamicPrintConfig profile;
        profile.set_deserialize("thumbnail_color_mode", palette);
        PrintConfig fff;
        SLAPrinterConfig sla;
        fff.apply(profile); sla.apply(profile);
        CHECK(fff.option("thumbnail_color_mode")->serialize() == palette);
        CHECK(sla.option("thumbnail_color_mode")->serialize() == palette);
        CHECK_FALSE(fff.has("thumbnails_custom_color"));
        CHECK_FALSE(sla.has("thumbnails_custom_color"));
        sla.thumbnails_color.value = "#123456";
        ThumbnailsParams params{{}, true, true, true, true};
        apply_thumbnail_colors(params, sla);
        CHECK(params.color_mode == (palette == "uniform" ? 0 : palette == "extruder" ? 1 : 2));
        CHECK(params.uniform_color == (std::array<unsigned char, 4>{0x12, 0x34, 0x56, 255}));
    }
    for (const char *obsolete : {"thumbnail_camera", "thumbnail_projection", "thumbnail_framing",
                                "thumbnail_custom_scale", "thumbnail_camera_position", "thumbnail_camera_target", "thumbnail_camera_up"}) {
        std::string key = obsolete, value = "custom";
        PrintConfigDef::handle_legacy_pair(key, value);
        CHECK(key.empty());
    }
}
