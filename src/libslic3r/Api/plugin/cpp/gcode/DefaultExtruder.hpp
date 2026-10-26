///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_plugin_cpp_gcode_DefaultExtruder_hpp_
#define slic3r_Api_plugin_cpp_gcode_DefaultExtruder_hpp_

#include "ExtrusionAxisState.hpp"
#include "FanState.hpp"
#include "HeaterState.hpp"
#include "PressureAdvanceState.hpp"
#include "Tool.hpp"

/*
Standard firmware extruder composition
======================================

DefaultExtruder is an identified machine tool composed from independent
firmware states. It owns one fan, heater, pressure-advance register and E axis,
but does not inherit from those capabilities. This makes the physical model
visible and lets Printer reuse the same heater and fan components.

The standard firmware copies all component runtime state between logical tools
in single-extruder multi-material mode, where tools select filaments on shared
hardware. Device corrections and filament conversion settings remain configured
on the destination tool. Other shared components may be copied by a dialect.
*/

namespace slic3r_api { namespace GCodeGeneration {

class DefaultGCodeFirmwareSession;

class DefaultExtruder : public Tool
{
public:
    explicit DefaultExtruder(uint16_t id) : Tool(id) {}

    // Load configured tool and filament-density state, then reset owned runtime components.
    void setup(const Config &config) override;
    void reset_runtime_state() override;
    void synchronize_runtime_from(const DefaultExtruder &source);

    FanState &fan() { return m_fan; }
    const FanState &fan() const { return m_fan; }
    HeaterState &heater() { return m_heater; }
    const HeaterState &heater() const { return m_heater; }
    PressureAdvanceState &pressure_advance() { return m_pressure_advance; }
    const PressureAdvanceState &pressure_advance() const { return m_pressure_advance; }
    ExtrusionAxisState &extrusion_axis() { return m_extrusion_axis; }
    const ExtrusionAxisState &extrusion_axis() const { return m_extrusion_axis; }
    // Return filament density in g/cm³ for converting extruded volume to script weight.
    double filament_density() const { return m_filament_density; }

private:
    friend class DefaultGCodeFirmwareSession;

    FanState m_fan;
    HeaterState m_heater;
    PressureAdvanceState m_pressure_advance;
    ExtrusionAxisState m_extrusion_axis;
    // g/cm³, converts the extruded volume into a weight for scripts.
    double m_filament_density = 0.0;
};

}} // namespace slic3r_api::GCodeGeneration

#endif // slic3r_Api_plugin_cpp_gcode_DefaultExtruder_hpp_
