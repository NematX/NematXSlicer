///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_plugin_cpp_properties_ExtrusionProperties_hpp_
#define slic3r_Api_plugin_cpp_properties_ExtrusionProperties_hpp_

/*
Built-in extrusion properties
=============================

Extrusion entities store small C payloads describing their physical process,
geometry classification, or generated commands. This header gives every fixed
payload a typed PluginPropertyKey and concise C++ accessors while preserving the
exact layout consumed by the C plugin ABI.

These helpers describe only one directly stored payload. Inherited lookup over
an extrusion tree remains the responsibility of ExtrusionTreeVisitors.

Developer guide: [Using Plugin Properties](/doc/plugins/properties.md)

Entity guide: [Using Unified Extrusion Entities](/doc/plugins/extrusions.md)
*/

#include <cstdint>
#include <limits>
#include <stdexcept>

#include "libslic3r/Api/plugin/c/slic3r_extrusion_property.h"
#include "libslic3r/Api/plugin/cpp/PluginPropertyKey.hpp"

namespace slic3r_api {

template<class Derived, class Payload, extrusion_property_type TypeValue>
using EPropertyPayload = BuiltInPluginPropertyPayload<Derived, Payload, TypeValue>;

/* Physical attributes inherited by printable descendants. */
struct EPropertyAttributes :
    EPropertyPayload<EPropertyAttributes, c_extrusion_property_attributes, EXTRUSION_PROPERTY_TYPE_ATTRIBUTES>
{
    EPropertyAttributes &extrusion_role(int32_t value) { role = value; return *this; }
    int32_t extrusion_role() const { return role; }
    EPropertyAttributes &no_seam_enabled(bool enabled = true) { no_seam = enabled ? 1 : 0; return *this; }
    bool no_seam_enabled() const { return no_seam != 0; }
    EPropertyAttributes &mm3_per_mm(double value) { c_extrusion_property_attributes::mm3_per_mm = value; return *this; }
    double mm3_per_mm() const { return c_extrusion_property_attributes::mm3_per_mm; }
    EPropertyAttributes &width(float value) { c_extrusion_property_attributes::width = value; return *this; }
    EPropertyAttributes &height(float value) { c_extrusion_property_attributes::height = value; return *this; }
};

/* Optional process values selected by earlier printing-plan steps. */
struct EPropertySpeed :
    EPropertyPayload<EPropertySpeed, c_extrusion_property_speed, EXTRUSION_PROPERTY_TYPE_SPEED>
{
    /* Set the inherited speed in millimeters per second. */
    EPropertySpeed &speed(float value) { speed_mm_per_s = value; return *this; }
    /* Set the inherited acceleration in millimeters per second squared. */
    EPropertySpeed &acceleration(float value) { accel_mm_per_s2 = value; return *this; }
    /* Set the inherited pressure-advance value. */
    EPropertySpeed &pressure_advance(float value) { pressure_adv = value; return *this; }
    /* Set the inherited fan percentage. */
    EPropertySpeed &fan_speed(float value) { fan_speed_percent = value; return *this; }
};

/* Travel, retraction, lift, and tool-change movement modifiers. */
struct EPropertyModifier :
    EPropertyPayload<EPropertyModifier, c_extrusion_property_modifier, EXTRUSION_PROPERTY_TYPE_MODIFIER>
{
    EPropertyModifier &set_enforce_travel(bool enabled = true) { enforce_travel = enabled ? 1 : 0; return *this; }
    EPropertyModifier &set_enforce_retraction(bool enabled = true) { enforce_retraction = enabled ? 1 : 0; return *this; }
    EPropertyModifier &set_enforce_unlift(bool enabled = true) { enforce_unlift = enabled ? 1 : 0; return *this; }
    EPropertyModifier &set_disable_retraction(bool enabled = true) { disable_retraction = enabled ? 1 : 0; return *this; }
    EPropertyModifier &set_disable_lift(bool enabled = true) { disable_lift = enabled ? 1 : 0; return *this; }
    EPropertyModifier &set_toolchange_retraction(bool enabled = true) { toolchange_retraction = enabled ? 1 : 0; return *this; }
};

/* Stored text, script metadata, and optional serialized script configuration. */
struct EPropertyCustomGcode :
    EPropertyPayload<EPropertyCustomGcode, c_extrusion_property_custom_gcode, EXTRUSION_PROPERTY_TYPE_CUSTOM_GCODE>
{
    EPropertyCustomGcode &set_kind(c_extrusion_custom_gcode_kind value) {
        kind = value;
        script_type = value == C_EXTRUSION_CUSTOM_GCODE_SCRIPT ? GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM :
                                                                 GCODE_SCRIPT_TYPE_INVALID;
        return *this;
    }
    EPropertyCustomGcode &set_script_type(gcode_script_type value) {
        script_type = value;
        return *this;
    }
    EPropertyCustomGcode &text(extrusion_data_id value) { text_id = value; return *this; }
    EPropertyCustomGcode &processing_extruder(int64_t value) {
        if (value < EXTRUDER_ID_UNKNOWN ||
            value > (std::numeric_limits<extruder_index_t>::max)())
            throw std::out_of_range("The script processing extruder does not fit in extruder_index_t.");
        processing_extruder_id = static_cast<extruder_index_t>(value);
        return *this;
    }
};

/*
An entry command embedded in the ordered extrusion stream. EXTRUSION on a leaf
with segments instead distributes its signed filament length over planar lengths
(including arcs), at the leaf's XY speed, ignoring flow and extrusion multiplier.
Zero width/height omit dimension markers; zero planar length or a simultaneous
extrusion-axis request is invalid. Parents and point-less leaves emit E-only.
Signed deltas update retraction accounting: negative retracts, positive repays.
*/
struct EPropertySpecialCommand :
    EPropertyPayload<EPropertySpecialCommand, c_extrusion_property_special_command, EXTRUSION_PROPERTY_TYPE_SPECIAL_COMMAND>
{
    EPropertySpecialCommand &set(c_extrusion_special_command value, double data = 0.) {
        code = value;
        extra_data = data;
        return *this;
    }
};

/* Entry event that sets one heater target, optionally waiting for it. */
struct EPropertyHeater :
    EPropertyPayload<EPropertyHeater, c_extrusion_property_heater, EXTRUSION_PROPERTY_TYPE_HEATER>
{
    /* Set a tool target, optionally addressing a selected tool. */
    EPropertyHeater &set_tool(int16_t temperature, extruder_index_t tool_id = EXTRUDER_ID_UNKNOWN,
                               bool wait_for_it = false) {
        heater = C_EXTRUSION_HEATER_TOOL;
        tool = tool_id;
        temperature_C = temperature;
        wait = wait_for_it ? 1 : 0;
        return *this;
    }
    /* Set the bed target and optional wait policy. */
    EPropertyHeater &set_bed(int16_t temperature, bool wait_for_it = false) {
        heater = C_EXTRUSION_HEATER_BED;
        tool = EXTRUDER_ID_UNKNOWN;
        temperature_C = temperature;
        wait = wait_for_it ? 1 : 0;
        return *this;
    }
    /* Set the chamber target and optional wait policy. */
    EPropertyHeater &set_chamber(int16_t temperature, bool wait_for_it = false) {
        heater = C_EXTRUSION_HEATER_CHAMBER;
        tool = EXTRUDER_ID_UNKNOWN;
        temperature_C = temperature;
        wait = wait_for_it ? 1 : 0;
        return *this;
    }
};

/* Support-distance and dynamic overhang processing metadata. */
struct EPropertyOverhang :
    EPropertyPayload<EPropertyOverhang, c_extrusion_property_overhang, EXTRUSION_PROPERTY_TYPE_OVERHANG>
{
    EPropertyOverhang &distance(float start, float end) {
        start_distance_from_prev_layer = start;
        end_distance_from_prev_layer = end;
        return *this;
    }
    EPropertyOverhang &curled_proximity(float value) { proximity_to_curled_lines = value; return *this; }
    EPropertyOverhang &full_overhangs_flow(bool enabled = true) { has_full_overhangs_flow = enabled ? 1 : 0; return *this; }
    EPropertyOverhang &full_overhangs_speed(bool enabled = true) { has_full_overhangs_speed = enabled ? 1 : 0; return *this; }
    EPropertyOverhang &dynamic_overhangs_flow(bool enabled = true) { has_dynamic_overhangs_flow = enabled ? 1 : 0; return *this; }
    EPropertyOverhang &dynamic_overhangs_speed(bool enabled = true) { has_dynamic_overhangs_speed = enabled ? 1 : 0; return *this; }
};

/* Constant Z offset applied while generating this entity. */
struct EPropertyZOffset :
    EPropertyPayload<EPropertyZOffset, c_extrusion_property_z_offset, EXTRUSION_PROPERTY_TYPE_Z_OFFSET>
{
    EPropertyZOffset &set(coord_t value) { z_offset = value; return *this; }
    coord_t get() const { return z_offset; }
};

/* Shell index and loop classification assigned by perimeter generators. */
struct EPropertyPerimeter :
    EPropertyPayload<EPropertyPerimeter, c_extrusion_property_perimeter, EXTRUSION_PROPERTY_TYPE_PERIMETER>
{
    EPropertyPerimeter &shell_count(int16_t value) { perimeter_idx = value; return *this; }
    int16_t shell_count() const { return perimeter_idx; }
    EPropertyPerimeter &perimeter_flags(uint16_t value) { c_extrusion_property_perimeter::perimeter_flags = value; return *this; }
    uint16_t perimeter_flags() const { return c_extrusion_property_perimeter::perimeter_flags; }
};

/* Source-surface provenance and raw fill recipe retained by generated infill trees. */
struct EPropertyInfill :
    EPropertyPayload<EPropertyInfill, c_extrusion_property_infill, EXTRUSION_PROPERTY_TYPE_INFILL>
{
    EPropertyInfill &surface_id(uint64_t value) { source_surface_id = value; return *this; }
    uint64_t surface_id() const { return source_surface_id; }
    EPropertyInfill &area_to_fill_mm2(double value) {
        c_extrusion_property_infill::area_to_fill_mm2 = value;
        return *this;
    }
    double area_to_fill_mm2() const { return c_extrusion_property_infill::area_to_fill_mm2; }
    EPropertyInfill &height(coord_t value) { c_extrusion_property_infill::height = value; return *this; }
    coord_t height() const { return c_extrusion_property_infill::height; }
    EPropertyInfill &spacing(coord_t value) { c_extrusion_property_infill::spacing = value; return *this; }
    coord_t spacing() const { return c_extrusion_property_infill::spacing; }
    EPropertyInfill &spacing_ratio(float value) {
        c_extrusion_property_infill::spacing_ratio = value;
        return *this;
    }
    float spacing_ratio() const { return c_extrusion_property_infill::spacing_ratio; }
    EPropertyInfill &density(float value) { c_extrusion_property_infill::density = value; return *this; }
    float density() const { return c_extrusion_property_infill::density; }
    EPropertyInfill &flow_mult(float value) { c_extrusion_property_infill::flow_mult = value; return *this; }
    float flow_mult() const { return c_extrusion_property_infill::flow_mult; }
    EPropertyInfill &surface_type(uint16_t value) { c_extrusion_property_infill::surface_type = value; return *this; }
    uint16_t surface_type() const { return c_extrusion_property_infill::surface_type; }
    EPropertyInfill &infill_flags(uint16_t value) {
        c_extrusion_property_infill::infill_flags = value;
        return *this;
    }
    uint16_t infill_flags() const { return c_extrusion_property_infill::infill_flags; }
};

/*
Firmware-neutral request for the physical extrusion axis.

retract_to() and unretract() may describe either an empty E-only event or a
geometric Wipe leaf. For geometry, the firmware session distributes the
remaining semantic operation over the complete planar path. The selected
firmware remains free to encode the same request as explicit E or G10/G11.
unretract_partial() describes an E-only event, clamped to outstanding retraction
without scheduling or consuming restart extras. Native G10/G11 cannot restore a
partial amount, so this event does nothing until a full unretraction follows.
*/
struct EPropertyExtrusionAxis :
    EPropertyPayload<EPropertyExtrusionAxis, c_extrusion_property_extrusion_axis, EXTRUSION_PROPERTY_TYPE_EXTRUSION_AXIS>
{
    /* Configure an event or Wipe leaf to reach an absolute retracted length. */
    EPropertyExtrusionAxis &retract_to(double target, bool is_toolchange = false) {
        operation = C_EXTRUSION_AXIS_OPERATION_RETRACT_TO;
        value = target;
        restart_extra = 0.0;
        toolchange = is_toolchange ? 1 : 0;
        return *this;
    }
    /* Configure an event or Wipe leaf to restore E and add optional extra prime. */
    EPropertyExtrusionAxis &unretract(double extra = 0.0, bool is_toolchange = false) {
        operation = C_EXTRUSION_AXIS_OPERATION_UNRETRACT;
        value = 0.0;
        restart_extra = extra;
        toolchange = is_toolchange ? 1 : 0;
        return *this;
    }
    /* Configure an E-only event to restore at most a positive filament length in mm. */
    EPropertyExtrusionAxis &unretract_partial(double length_mm, bool is_toolchange = false) {
        operation = C_EXTRUSION_AXIS_OPERATION_UNRETRACT;
        value = length_mm;
        restart_extra = 0.0;
        toolchange = is_toolchange ? 1 : 0;
        return *this;
    }
};

// Each helper is a direct typed view over the corresponding host-owned C bytes.
static_assert(sizeof(EPropertyAttributes) == sizeof(c_extrusion_property_attributes), "ABI payload mismatch");
static_assert(sizeof(EPropertySpeed) == sizeof(c_extrusion_property_speed), "ABI payload mismatch");
static_assert(sizeof(EPropertyModifier) == sizeof(c_extrusion_property_modifier), "ABI payload mismatch");
static_assert(sizeof(EPropertyCustomGcode) == sizeof(c_extrusion_property_custom_gcode), "ABI payload mismatch");
static_assert(sizeof(EPropertySpecialCommand) == sizeof(c_extrusion_property_special_command), "ABI payload mismatch");
static_assert(sizeof(EPropertyOverhang) == sizeof(c_extrusion_property_overhang), "ABI payload mismatch");
static_assert(sizeof(EPropertyZOffset) == sizeof(c_extrusion_property_z_offset), "ABI payload mismatch");
static_assert(sizeof(EPropertyPerimeter) == sizeof(c_extrusion_property_perimeter), "ABI payload mismatch");
static_assert(sizeof(EPropertyInfill) == sizeof(c_extrusion_property_infill), "ABI payload mismatch");
static_assert(sizeof(EPropertyExtrusionAxis) == sizeof(c_extrusion_property_extrusion_axis), "ABI payload mismatch");
static_assert(sizeof(EPropertyHeater) == sizeof(c_extrusion_property_heater), "ABI payload mismatch");

} // namespace slic3r_api

#endif // slic3r_Api_plugin_cpp_properties_ExtrusionProperties_hpp_
