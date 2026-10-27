///|/ Copyright (c) NematXSlicer 2026 Durand Remi @supermerill
///|/
///|/ NematXSlicer is released under the terms of the AGPLv3 or higher

#include "NematXFirmwareSession.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
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

#include "NematXObjectMetadata.hpp"

namespace nematx_firmware {

using namespace slic3r_api;
using namespace slic3r_api::GCodeGeneration;

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
        const double *m = object.transform().value;
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
    m_layer_count = 0;
    m_has_support = false;
    m_pending.clear();
}

// ---- Output filtering and metadata -----------------------------------------

// The base session calls some of these public methods itself (write_event()
// for the startup scripts inside begin_print()). Only the outermost call
// filters, so no text is numbered twice.
template<class Write>
std::string NematXGCodeFirmwareSession::filtered(Write write)
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
    return m_filter_depth == 0 ? filter(std::move(text)) : text;
}

std::string NematXGCodeFirmwareSession::filter(std::string text)
{
    // Look for support in the unfiltered text: the TYPE comment may be removed below.
    if (!m_has_support && text.find(";TYPE:Support material") != std::string::npos)
        m_has_support = true;
    return m_filter.process(text + take_pending());
}

std::string NematXGCodeFirmwareSession::begin_print(const Print &print, const ConstPrintingPlan &plan)
{
    // setup_firmware() runs inside Base::begin_print(), so the filter is ready afterwards.
    return filtered([&] { return Base::begin_print(print, plan); });
}

std::string NematXGCodeFirmwareSession::begin_group(const ConstPrintingGroup &group)
{
    return filtered([&] { return Base::begin_group(group); });
}

std::string NematXGCodeFirmwareSession::begin_layer(const ConstPrintingLayerGroup &layer)
{
    ++m_layer_count;
    return filtered([&] { return Base::begin_layer(layer); });
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
    return filtered([&] { return Base::end_layer(); });
}

std::string NematXGCodeFirmwareSession::end_group()
{
    return filtered([&] { return Base::end_group(); });
}

std::string NematXGCodeFirmwareSession::end_print()
{
    std::string output = filtered([&] { return Base::end_print(); });
    output += m_filter.flush();
    // Metadata read by the NematX controller and web interface. As in the
    // legacy slicer, it is written after the filtered G-code: it keeps its
    // comment form and has no line number even when those options are set.
    output += "; total layers count = " + std::to_string(m_layer_count) + "\n";
    output += std::string("; has_support = ") + (m_has_support ? "1" : "0") + "\n";
    return output;
}

std::string NematXGCodeFirmwareSession::take_pending() const
{
    std::string text;
    text.swap(m_pending);
    return text;
}

// ---- Startup ------------------------------------------------------------------

// After the file header and the notes, before the start G-code: the object and
// plater descriptions read by the NematX controller (see NematXObjectMetadata.hpp).
// As in the legacy slicer, they go through the output filter like the G-code.
std::string NematXGCodeFirmwareSession::write_post_thumbnail(const Print &print, const ConstPrintingPlan &plan)
{
    std::string output = Base::write_post_thumbnail(print, plan);
    if (m_object_metadata)
        output += format_objects_metadata(collect_object_metadata(print, plan));
    return output;
}

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

// G1 X.. Y.. A.. F.. with the axis letter of the current extruder, or G2/G3
// with I/J center offsets for arcs. With relative extrusion, G90 precedes the
// coordinates and G91 the extruder distance: "G1 G90 X10 Y5 G91 A0.3".
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
    return take_pending() + formatter.string();
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
