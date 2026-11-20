///|/ Copyright (c) NematXSlicer 2026 Durand Remi @supermerill
///|/
///|/ NematXSlicer is released under the terms of the AGPLv3 or higher

#include "NematXFirmwareSession.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <random>
#include <stdexcept>
#include <string>
#include <string_view>

#include "libslic3r/Api/plugin/cpp/ConfigViews.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/Api/plugin/cpp/VolumeViews.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/ArcGeometry.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeFormatter.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeReaderService.hpp"

#include "NematXObjectMetadata.hpp"
#include "NematXPressureSettings.hpp"
#include "../../pressure/src/PressureFlow.hpp"

// Provides the NematX export session to the package's firmware provider. The
// base produces nominal commands and finishes script synchronization first.
// Only the outer public call removes private movement hints, detects support,
// reads the clean text and passes it through the optional pressure Flow, then
// OutputFilter. Flow owns layer anticipation and a separate corrected E state;
// the base retains nominal extrusion accounting. A closed cycle has zero
// pressure offset at its rounded nominal E coordinate. A distinct Reader client
// owns its lock and final state, including private items, for this export only.
// Movement hints carry a per-export token so literal script comments are not hints.
// Host and Print are borrowed until session destruction. Exceptions propagate
// to the existing session ABI; the file writer withholds its temporary output.
//     provider -> set_pressure_reader_context: borrow host objects
//     begin_print -> setup_firmware: copy settings and reset export state
//     public wrappers -> filtered -> filter: process outer production once
//         PressureRuntime::consume -> Reader -> index_fragment -> Flow
//         OutputFilter::process: remove comments and number final commands
//     end_print -> separator -> OutputFilter::flush -> controller metadata

namespace nematx_firmware {

using namespace slic3r_api;
using namespace slic3r_api::GCodeGeneration;

struct NematXGCodeFirmwareSession::PressureRuntime
{
    // Owns an independent Reader and Flow and creates one private hint token;
    // the Reader borrows live host objects. Random-source failures propagate.
    PressureRuntime(orchestrator_handle *host, const print_handle *print,
                    const nematx_pressure::FlowConfig &config);
    // Removes internal hints before Reader input, carries the complete final
    // state and logs owned issues.
    // Current-fragment classification and control failures gain a one-based
    // source ordinal; deferred rewrite failures retain their own local context.
    // Reader/Flow failures propagate without returning bytes.
    std::string consume(std::string text, Boundary boundary);

    orchestrator_handle *host;
    GCodeReaderService reader;
    GCodeStateTracker prior;
    nematx_pressure::Flow flow;
    // Private text prefix shared only by this export's encoder and hint removal.
    std::string hint_prefix = ";NX_INTERNAL_MOVE=";
    // Count clean source records, including a final record without LF, like Flow.
    std::size_t source_rows = 0;
};

NematXGCodeFirmwareSession::PressureRuntime::PressureRuntime(
    orchestrator_handle *host, const print_handle *print, const nematx_pressure::FlowConfig &config)
    : host(host), reader(host, print), flow(config)
{
    std::random_device random;
    for (int word = 0; word < 4; ++word)
        hint_prefix += std::to_string(random()) + ":";
}

std::string NematXGCodeFirmwareSession::PressureRuntime::consume(std::string text, Boundary boundary)
{
    using namespace nematx_pressure;
    std::string clean;
    std::vector<MoveHint> hints;
    MoveHint pending = MoveHint::None;
    const std::string_view prefix = hint_prefix;
    for (size_t begin = 0; begin < text.size();) {
        const size_t lf = text.find('\n', begin);
        const size_t end = lf == std::string::npos ? text.size() : lf + 1;
        const std::string_view row(text.data() + begin, end - begin);
        if (row.substr(0, prefix.size()) == prefix) {
            if (pending != MoveHint::None)
                throw std::logic_error("Repeated NematX internal movement hint.");
            if (row == hint_prefix + "extrusion\n") pending = MoveHint::Extrusion;
            else if (row == hint_prefix + "travel\n") pending = MoveHint::Travel;
            else if (row == hint_prefix + "wipe\n") pending = MoveHint::Wipe;
            else throw std::logic_error("Malformed NematX internal movement hint.");
        } else {
            if (pending != MoveHint::None &&
                !(row.size() >= 3 && row[0] == 'G' && row[1] >= '0' && row[1] <= '3' && row[2] == ' '))
                throw std::logic_error("NematX internal hint is not followed by its movement.");
            hints.push_back(pending);
            pending = MoveHint::None;
            clean.append(row);
        }
        begin = end;
    }
    if (pending != MoveHint::None)
        throw std::logic_error("NematX internal movement hint has no movement.");
    ReaderFragment observed = read_fragment(reader, prior, clean);
    prior = observed.final;
    SourceFragment source = index_fragment(std::move(clean), std::move(observed));
    const std::size_t row_count = source.lines.size();
    FlowOutput output;
    try {
        output = flow.consume(std::move(source), hints,
            {boundary == Boundary::BeginLayer, boundary == Boundary::EndLayer, boundary == Boundary::EndPrint});
    } catch (const ClassificationError &error) {
        throw std::invalid_argument("NematX pressure source line " +
            std::to_string(source_rows + error.line_index + 1) + ": " + error.what());
    } catch (const ControlError &error) {
        throw std::invalid_argument("NematX pressure source line " +
            std::to_string(source_rows + error.line_index + 1) + ": " + error.what());
    }
    source_rows += row_count;
    for (const FlowIssue &issue : output.issues) {
        const char *cause = "Reader diagnostic";
        switch (issue.kind) {
        case FlowIssueKind::InvalidAcceleration: cause = "invalid ;M204 S acceleration ignored"; break;
        case FlowIssueKind::ReaderInvalidation: cause = "Reader state invalidated"; break;
        case FlowIssueKind::ReaderDiagnostic: break;
        case FlowIssueKind::HorizonInsufficient: cause = "pressure anticipation horizon insufficient"; break;
        case FlowIssueKind::GeometryMerged: cause = "pressure phases merged at output precision"; break;
        }
        for (const raw_gcode_reader_event_item &item : issue.reader_items) {
            if (item.type != RAW_GCODE_READER_ITEM_DIAGNOSTIC_TYPE)
                continue;
            switch (uint32_t(item.value)) {
            case RAW_GCODE_READER_DIAGNOSTIC_UNKNOWN_COMMAND: cause = "Reader unknown command"; break;
            case RAW_GCODE_READER_DIAGNOSTIC_INVALID_COMMAND_PARAMETERS: cause = "Reader invalid command parameters"; break;
            case RAW_GCODE_READER_DIAGNOSTIC_UNKNOWN_TOOL: cause = "Reader unknown tool"; break;
            case RAW_GCODE_READER_DIAGNOSTIC_AMBIGUOUS_TOOL: cause = "Reader ambiguous tool"; break;
            case RAW_GCODE_READER_DIAGNOSTIC_UNSUPPORTED_STATE_EFFECT: cause = "Reader unsupported state effect"; break;
            }
        }
        const std::string message = "NematX pressure: " + std::string(cause) + ", tool " +
            (issue.tool ? std::to_string(*issue.tool) : "unknown") + ", source line " +
            std::to_string(issue.source_ordinal + 1);
        orchestrator_send_log(host, RAW_PLUGIN_LOG_WARNING, message.c_str());
    }
    return std::move(output.bytes);
}

namespace {

// Rewrite the startup modes chosen by the base session for NematX: G71 selects
// millimetres (G21 for Marlin), and there is no M82/M83 because relative
// extrusion is written inside each move.
std::string nematx_startup_modes(const std::string &text)
{
    std::string output;
    size_t begin = 0;
    while (begin < text.size()) {
        size_t end = text.find('\n', begin);
        if (end == std::string::npos)
            end = text.size();
        const std::string_view line(text.data() + begin, end - begin);
        if (line == "G21")
            output += "G71\n";
        else if (line != "M82" && line != "M83") {
            output.append(line);
            output += '\n';
        }
        begin = end + 1;
    }
    return output;
}

// Apply a row-major 4x4 transformation to a point (millimetres).
Point3 transform_point(const c_matrix4d &m, double x, double y, double z)
{
    const double *v = m.value;
    return { v[0] * x + v[1] * y + v[2] * z + v[3],
             v[4] * x + v[5] * y + v[6] * z + v[7],
             v[8] * x + v[9] * y + v[10] * z + v[11] };
}

// Describe every object instance of the plan, in the order of the legacy
// slicer: objects of the print in order, then their instances.
std::vector<ObjectMetadata> collect_object_metadata(const Print &print, const ConstPrintingPlan &plan)
{
    std::vector<ObjectMetadata> objects;
    for (uint32_t catalog_id = 0; catalog_id < plan.object_instance_count(); ++catalog_id) {
        const PrintingObjectInstance instance = plan.object_instance(catalog_id);
        // Synthetic entries (a wipe tower for example) are not objects.
        if (!instance.has_object())
            continue;
        const Object object = instance.object();
        const uint32_t instance_idx = uint32_t(instance.object_instance_idx());

        // The id counts the instances of the objects before this one.
        size_t first_id = 0;
        uint32_t object_idx = 0;
        while (object_idx < print.object_count() && print.object(object_idx).handle() != object.handle())
            first_id += print.object(object_idx++).instance_count();
        if (object_idx == print.object_count())
            throw std::invalid_argument("The printing plan contains an object from another print.");

        ObjectMetadata metadata;
        metadata.name = instance.name();
        metadata.id = first_id + instance_idx;
        metadata.copy = instance_idx;
        const c_point center = instance.projected_outline_center();
        metadata.center = { unscaled(center.x), unscaled(center.y) };
        const Polygon outline = instance.projected_outline();
        for (uint32_t i = 0; i < outline.size(); ++i)
            metadata.outline.push_back({ unscaled(outline[i].x), unscaled(outline[i].y) });

        // Bounding box of the model parts, placed on the plater like the
        // object outline: object transformation, then the instance shift.
        const c_point shift = object.instance_shift(instance_idx);
        const c_matrix4d placement = matrix4d_mul(matrix4d_translation(unscaled(shift.x), unscaled(shift.y), 0.0),
                                                  object.transform_centered());
        for (uint32_t volume_idx = 0; volume_idx < object.volume_count(); ++volume_idx) {
            const Volume volume = object.volume(volume_idx);
            if (!volume.is_model_part())
                continue;
            const c_matrix4d to_plater = matrix4d_mul(placement, volume.matrix());
            const TriangleMesh mesh = volume.mesh();
            for (uint32_t vertex_idx = 0; vertex_idx < mesh.vertex_count(); ++vertex_idx) {
                const c_vec3f vertex = mesh.vertex(vertex_idx);
                metadata.box.merge(transform_point(to_plater, vertex.x, vertex.y, vertex.z));
            }
        }

        // Scale of each axis: the length of the matching column of the
        // object transformation (rotation and mirroring do not change it).
        const c_matrix4d transform = object.transform();
        const double *m = transform.value;
        for (size_t axis = 0; axis < 3; ++axis)
            metadata.scale[axis] = std::sqrt(m[axis] * m[axis] + m[4 + axis] * m[4 + axis] + m[8 + axis] * m[8 + axis]);

        objects.push_back(std::move(metadata));
    }
    std::sort(objects.begin(), objects.end(),
              [](const ObjectMetadata &a, const ObjectMetadata &b) { return a.id < b.id; });
    return objects;
}

} // namespace

NematXGCodeFirmwareSession::NematXGCodeFirmwareSession(GCodeScriptProcessorView scripts, storage_handle *storage)
    : Base(scripts, storage)
{}

NematXGCodeFirmwareSession::~NematXGCodeFirmwareSession() = default;

void NematXGCodeFirmwareSession::set_pressure_reader_context(orchestrator_handle *orchestrator,
                                                          const print_handle *print)
{
    m_pressure_orchestrator = orchestrator;
    m_pressure_print = print;
}

void NematXGCodeFirmwareSession::setup_firmware(const Config &config)
{
    Base::setup_firmware(config);

    const size_t tool_count = extruders().size();
    if (tool_count > k_max_tools)
        throw std::invalid_argument("The NematX firmware supports at most " + std::to_string(k_max_tools) +
                                    " extruders; this printer has " + std::to_string(tool_count) + ".");
    if (config.bool_or_default("use_firmware_retraction", false))
        throw std::invalid_argument("The NematX firmware has no firmware retraction (G10/G11): "
                                    "disable use_firmware_retraction.");

    std::vector<std::string> axes;
    for (uint32_t tool = 0; tool < tool_count; ++tool)
        axes.push_back(config.vector_string_or_default(k_extruder_axis_key, tool, ""));
    m_axis_letters = resolve_axis_letters(axes, tool_count);

    m_disable_unused_fan = config.bool_or_default(k_disable_unused_fan_key, false);
    m_fan_percentage = config.bool_or_default("fan_percentage", false);
    m_relative_e = config.bool_or_default("use_relative_e_distances", false);
    // gcode_label_objects: 0 means disabled, as in the base session.
    m_object_metadata = config.enum_or_default("gcode_label_objects", 0) != 0;
    m_filter = OutputFilter(config.bool_or_default(k_no_comment_key, false),
                            config.bool_or_default(k_line_number_key, false));
    m_pending.clear();
    m_pressure.reset();
    m_output_unterminated = false;
    nematx_pressure::FlowConfig pressure{};
    bool enabled = false;
    for (uint32_t tool = 0; tool < tool_count; ++tool) {
        const double k = config.vector_float_or_default(k_pressure_factor_key, tool, 0.0);
        if (!std::isfinite(k) || k < 0.0)
            throw std::invalid_argument("NematX extruder_pressure_factor must be finite and nonnegative, tool " +
                                        std::to_string(tool) + ".");
        const bool split = config.vector_bool_or_default(k_split_acceleration_key, tool, false);
        pressure.tools.push_back({k, split});
        enabled = enabled || k > 0.0 || split;
    }
    if (!enabled)
        return;
    pressure.axis_letters = m_axis_letters;
    pressure.xyz_digits = gcode_formatter().m_gcode_precision_xyz;
    pressure.e_digits = gcode_formatter().m_gcode_precision_e;
    pressure.volumetric = config.bool_or_default("use_volumetric_e", false);
    const ConfigOption usage = config.get("machine_limits_usage");
    if (usage.valid() && usage.is_enabled() && usage.serialize() != "ignore") {
        const char *speed_keys[] = {"machine_max_feedrate_x", "machine_max_feedrate_y",
                                    "machine_max_feedrate_z", "machine_max_feedrate_e"};
        const char *accel_keys[] = {"machine_max_acceleration_x", "machine_max_acceleration_y",
                                    "machine_max_acceleration_z", "machine_max_acceleration_e"};
        for (size_t axis = 0; axis < 4; ++axis) {
            pressure.limits.speed[axis] = config.vector_float_or_default(speed_keys[axis], 0, 0.0);
            pressure.limits.acceleration[axis] = config.vector_float_or_default(accel_keys[axis], 0, 0.0);
            if (!std::isfinite(pressure.limits.speed[axis]))
                throw std::invalid_argument(std::string("NematX pressure requires finite ") + speed_keys[axis]);
            if (!std::isfinite(pressure.limits.acceleration[axis]))
                throw std::invalid_argument(std::string("NematX pressure requires finite ") + accel_keys[axis]);
        }
        pressure.limits.max_extruding_acceleration =
            config.vector_float_or_default("machine_max_acceleration_extruding", 0, 0.0);
        if (!std::isfinite(pressure.limits.max_extruding_acceleration))
            throw std::invalid_argument("NematX pressure requires finite machine_max_acceleration_extruding");
    }
    pressure.initial_acceleration = pressure.limits.acceleration[0] > 0.0 ?
        pressure.limits.acceleration[0] : 100000.0;
    if (pressure.limits.max_extruding_acceleration > 0.0)
        pressure.initial_acceleration = std::min(pressure.initial_acceleration,
                                                pressure.limits.max_extruding_acceleration);
    m_pressure = std::make_unique<PressureRuntime>(m_pressure_orchestrator, m_pressure_print, pressure);
}

// ---- Output filtering and metadata -----------------------------------------

template<class Write>
std::string NematXGCodeFirmwareSession::filtered(Write write, Boundary boundary)
{
    ++m_filter_depth;
    std::string text;
    try {
        text = write();
    } catch (...) {
        --m_filter_depth;
        throw;
    }
    --m_filter_depth;
    return m_filter_depth == 0 ? filter(std::move(text), boundary) : text;
}

std::string NematXGCodeFirmwareSession::filter(std::string text, Boundary boundary)
{
    text += take_pending();
    if (m_pressure)
        text = m_pressure->consume(std::move(text), boundary);
    if (!text.empty())
        m_output_unterminated = text.back() != '\n';
    return m_filter.process(text);
}

std::string NematXGCodeFirmwareSession::begin_print(const Print &print, const ConstPrintingPlan &plan)
{
    return filtered([&] {
        // setup_firmware() runs inside Base::begin_print(), so the filter and the
        // metadata option are ready afterwards.
        std::string body = Base::begin_print(print, plan);
        // The session output starts after the file header that the exporter writes.
        // The object and plater descriptions read by the NematX controller (see
        // NematXObjectMetadata.hpp) come first, before the start G-code. As in the
        // legacy slicer, they go through the output filter like the G-code.
        if (!m_object_metadata)
            return body;
        return format_objects_metadata(collect_object_metadata(print, plan)) + body;
    });
}

std::string NematXGCodeFirmwareSession::begin_group(const ConstPrintingGroup &group)
{
    return filtered([&] { return Base::begin_group(group); });
}

std::string NematXGCodeFirmwareSession::begin_layer(const ConstPrintingLayerGroup &layer)
{
    return filtered([&] { return Base::begin_layer(layer); }, Boundary::BeginLayer);
}

std::string NematXGCodeFirmwareSession::begin_tool_group(const ConstPrintingToolGroup &tool_group)
{
    return filtered([&] { return Base::begin_tool_group(tool_group); });
}

std::string NematXGCodeFirmwareSession::write_extrusion(const ConstPrintingExtrusion &extrusion)
{
    return filtered([&] { return Base::write_extrusion(extrusion); });
}

std::string NematXGCodeFirmwareSession::write_event(const ExtrusionEntity &event_root)
{
    return filtered([&] { return Base::write_event(event_root); });
}

std::string NematXGCodeFirmwareSession::end_tool_group()
{
    return filtered([&] { return Base::end_tool_group(); });
}

std::string NematXGCodeFirmwareSession::end_layer()
{
    return filtered([&] { return Base::end_layer(); }, Boundary::EndLayer);
}

std::string NematXGCodeFirmwareSession::end_group()
{
    return filtered([&] { return Base::end_group(); });
}

std::string NematXGCodeFirmwareSession::end_print()
{
    // The layer count and has_support lines read by the NematX controller come from
    // the host statistics, which the exporter writes after the session output.
    std::string output = filtered([&] { return Base::end_print(); }, Boundary::EndPrint);
    output += flush_before_metadata(m_filter, m_pressure && m_output_unterminated);
    return output;
}

std::string NematXGCodeFirmwareSession::take_pending() const
{
    std::string text;
    text.swap(m_pending);
    return text;
}

// ---- Startup ------------------------------------------------------------------

std::string NematXGCodeFirmwareSession::write_pre_start(const std::string &resolved_script)
{
    return nematx_startup_modes(Base::write_pre_start(resolved_script));
}

std::string NematXGCodeFirmwareSession::write_post_start()
{
    return nematx_startup_modes(Base::write_post_start());
}

// ---- Tools ----------------------------------------------------------------------

char NematXGCodeFirmwareSession::axis_letter(extruder_index_t tool_id) const
{
    if (tool_id < 0 || size_t(tool_id) >= m_axis_letters.size())
        throw std::invalid_argument("A NematX command addresses an unknown extruder " +
                                    std::to_string(int(tool_id) + 1) + ".");
    return m_axis_letters[size_t(tool_id)];
}

char NematXGCodeFirmwareSession::current_axis_letter() const
{
    const std::optional<extruder_index_t> tool = current_extruder_id();
    return axis_letter(tool ? *tool : 0);
}

size_t NematXGCodeFirmwareSession::physical_tool(extruder_index_t tool_id) const
{
    return single_extruder_multi_material() || tool_id < 0 ? 0 : size_t(tool_id);
}

// The base session calls this hook while selecting a tool. NematX has an
// independent fan per extruder; with disable_fan_from_unused_extruder, the fan
// of the extruder being left is switched off. A hook cannot return text, so the
// command is queued and written before the next command.
void NematXGCodeFirmwareSession::synchronize_selected_extruder_state(const DefaultExtruder *previous,
                                                                    DefaultExtruder &selected)
{
    Base::synchronize_selected_extruder_state(previous, selected);
    if (!m_disable_unused_fan || previous == nullptr || previous->id() == selected.id() ||
        single_extruder_multi_material())
        return;
    const std::optional<double> encoded = previous->fan().encoded_speed_percent();
    if (encoded && *encoded == 0.0)
        return;
    m_pending += encode_fan(extruder_index_t(previous->id()), 0.0);
    // The firmware fan of that extruder is now off; a later return to it
    // writes its requested speed again.
    extruders()[previous->id()].fan().synchronize_encoded_after_external_gcode(0.0);
}

// NematX has no tool-change command: the next move simply uses the axis
// letter of the new extruder. Each axis keeps its own position, so the base
// session, which resets E only after a written tool-change command, leaves the
// positions of both extruders unchanged.
std::string NematXGCodeFirmwareSession::encode_tool_change(extruder_index_t) const
{
    return {};
}

// "A[SET_POSITION POS=0]" sets the extruder axis position (G92 E0 for Marlin).
std::string NematXGCodeFirmwareSession::encode_extrusion_axis_reset() const
{
    return take_pending() + current_axis_letter() + "[SET_POSITION POS=" +
           format_number(0.0, gcode_formatter().m_gcode_precision_e) + "]\n";
}

// ---- Heaters and fans -------------------------------------------------------------

// M104=215 (first tool) or M124=215 (second tool); a wait is a separate bare
// M109 / M129 that waits for the target set before.
std::string NematXGCodeFirmwareSession::encode_tool_temperature(extruder_index_t tool_id,
                                                                int16_t temperature, bool wait) const
{
    const size_t tool = physical_tool(tool_id);
    std::string output = take_pending() + tool_temperature_code(tool) + "=" + std::to_string(temperature) + "\n";
    if (wait)
        output += std::string(tool_wait_code(tool)) + "\n";
    return output;
}

// M140=60; the heater is switched off with M140=0 followed by M141, and a wait
// is a bare M190.
std::string NematXGCodeFirmwareSession::encode_bed_temperature(int16_t temperature, bool wait) const
{
    std::string output = take_pending() + "M140=" + std::to_string(temperature) + "\n";
    if (temperature == 0)
        output += "M141\n";
    if (wait)
        output += "M190\n";
    return output;
}

std::string NematXGCodeFirmwareSession::encode_chamber_temperature(int16_t, bool) const
{
    return take_pending() + encode_unsupported_operation("chamber temperature");
}

// M106=<value> (first tool) or M126=<value> (second tool), from 0 to 255, or
// from 0 to 100 with fan_percentage. Off is "=0" (never M107/M127).
std::string NematXGCodeFirmwareSession::encode_fan(extruder_index_t tool_id, double speed_percent) const
{
    const double percent = std::clamp(speed_percent, 0.0, 100.0);
    const double value = m_fan_percentage ? percent : percent * 255.0 / 100.0;
    return take_pending() + tool_fan_code(physical_tool(tool_id)) + "=" + format_number(value, 2) + "\n";
}

std::string NematXGCodeFirmwareSession::encode_pressure_advance(extruder_index_t, double) const
{
    return take_pending() + encode_unsupported_operation("pressure advance");
}

// The NematX firmware has no acceleration command: the print value is kept as
// a comment for information, as in the legacy slicer.
std::string NematXGCodeFirmwareSession::encode_acceleration(uint32_t acceleration, bool travel) const
{
    if (travel)
        return take_pending();
    return take_pending() + ";M204 S" + std::to_string(acceleration) + "\n";
}

// ---- Moves ------------------------------------------------------------------------

std::string NematXGCodeFirmwareSession::encode_move(const PreparedMove &move, bool include_speed) const
{
    const std::optional<c_vec3d> current = gantry().position();
    GCodeFormatter formatter(gcode_formatter());

    bool arc = move.destination && move.radius_mm != 0.f;
    if (arc && (!current || move.arc_orientation == RAW_EXTRUSION_ARC_ORIENTATION_UNKNOWN))
        throw std::invalid_argument("A NematX arc needs a known start position and an orientation.");
    if (arc) {
        // A chord of a few precision steps gives an unreliable center: write a
        // straight move instead, as the legacy slicer did.
        const double step = GCodeFormatter::pow_10_inv[formatter.m_gcode_precision_xyz];
        const double steps = (std::abs(move.destination->x - current->x) +
                              std::abs(move.destination->y - current->y)) / step;
        arc = steps > 10.0;
    }

    formatter.emit_string(!arc ? "G1" :
        move.arc_orientation == RAW_EXTRUSION_ARC_ORIENTATION_CCW ? "G3" : "G2");
    if (move.destination) {
        if (m_relative_e)
            formatter.emit_string(" G90");
        if (!current || current->x != move.destination->x)
            formatter.emit_axis('X', move.destination->x, formatter.m_gcode_precision_xyz);
        if (!current || current->y != move.destination->y)
            formatter.emit_axis('Y', move.destination->y, formatter.m_gcode_precision_xyz);
        if (!current || current->z != move.destination->z)
            formatter.emit_axis('Z', move.destination->z, formatter.m_gcode_precision_xyz);
        if (arc) {
            const c_vec2d start{ current->x, current->y };
            const c_vec2d center = arc_center(start, c_vec2d{ move.destination->x, move.destination->y },
                                              double(move.radius_mm),
                                              move.arc_orientation == RAW_EXTRUSION_ARC_ORIENTATION_CCW);
            formatter.emit_axis('I', center.x - start.x, formatter.m_gcode_precision_xyz);
            formatter.emit_axis('J', center.y - start.y, formatter.m_gcode_precision_xyz);
        }
    }
    if (move.extrusion) {
        if (m_relative_e)
            formatter.emit_string(" G91");
        formatter.emit_axis(current_axis_letter(), *move.extrusion, formatter.m_gcode_precision_e);
    }
    if (include_speed)
        formatter.emit_f(*gantry().requested_speed() * 60.0);
    std::string hint;
    if (m_pressure) {
        switch (move.kind) {
        case PreparedMove::Kind::Extrusion: hint = m_pressure->hint_prefix + "extrusion\n"; break;
        case PreparedMove::Kind::Travel: hint = m_pressure->hint_prefix + "travel\n"; break;
        case PreparedMove::Kind::Wipe: hint = m_pressure->hint_prefix + "wipe\n"; break;
        case PreparedMove::Kind::None: break;
        }
    }
    return take_pending() + hint + formatter.string();
}

// ---- Commands the NematX firmware does not have ------------------------------------

std::string NematXGCodeFirmwareSession::encode_save_speed_ratio(double) const { return take_pending(); }
std::string NematXGCodeFirmwareSession::encode_restore_speed_ratio() const { return take_pending(); }
std::string NematXGCodeFirmwareSession::encode_flush_planner() const { return take_pending(); }
std::string NematXGCodeFirmwareSession::encode_extruder_current(extruder_index_t, double) const
{
    return take_pending();
}
std::string NematXGCodeFirmwareSession::encode_machine_envelope(const MachineEnvelope &) const
{
    return take_pending();
}

// Firmware retraction is rejected in setup_firmware(); reaching this is a bug.
std::string NematXGCodeFirmwareSession::encode_firmware_retraction(bool) const
{
    throw std::logic_error("The NematX firmware has no firmware retraction (G10/G11).");
}

// G4 X<seconds> (3 decimals) instead of Marlin's G4 P<milliseconds>.
std::string NematXGCodeFirmwareSession::encode_pause(double milliseconds) const
{
    if (!std::isfinite(milliseconds) || milliseconds < 0.0)
        throw std::invalid_argument("A pause command contains an invalid duration.");
    return take_pending() + "G4 X" + format_number(milliseconds / 1000.0, 3) + "\n";
}

} // namespace nematx_firmware
