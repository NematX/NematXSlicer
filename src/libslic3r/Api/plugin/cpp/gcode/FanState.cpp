///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "FanState.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

/*
Firmware fan state implementation
=================================

The configured offset is applied only when a command value is evaluated.
Runtime synchronization therefore copies request/output history while keeping
the destination fan's own setup-derived correction.
*/

namespace slic3r_api { namespace GCodeGeneration {

void FanState::setup(double speed_offset_percent, double minimum_speed_percent)
{
    if (!std::isfinite(speed_offset_percent) || !std::isfinite(minimum_speed_percent))
        throw std::invalid_argument("Non-finite fan calibration.");
    m_speed_offset_percent = speed_offset_percent;
    m_minimum_speed_percent = std::clamp(minimum_speed_percent, 0.0, 100.0);
}

void FanState::reset_runtime_state()
{
    m_speed.reset_runtime_state();
}

void FanState::synchronize_runtime_from(const FanState &source)
{
    m_speed.synchronize_runtime_from(source.m_speed);
}

void FanState::synchronize_after_external_gcode(double effective_speed_percent)
{
    if (!std::isfinite(effective_speed_percent))
        throw std::invalid_argument("External G-code returned a non-finite fan speed.");
    const double effective = std::clamp(effective_speed_percent, 0.0, 100.0);
    // Calibration is non-injective: an observation cannot recover a logical
    // request. Keep only the physical output, never invert minimum/offset.
    m_speed.request(std::nullopt);
    m_speed.synchronize_encoded(effective);
}

void FanState::synchronize_encoded_after_external_gcode(
    std::optional<double> effective_speed_percent)
{
    if (effective_speed_percent && !std::isfinite(*effective_speed_percent))
        throw std::invalid_argument("External G-code returned a non-finite fan speed.");
    m_speed.synchronize_encoded(effective_speed_percent ?
        std::optional<double>(std::clamp(*effective_speed_percent, 0.0, 100.0)) :
        std::optional<double>());
}

double FanState::effective_speed_percent() const
{
    const std::optional<double> requested = m_speed.requested();
    if (!requested)
        throw std::logic_error("No fan speed is requested.");
    if (!std::isfinite(*requested) || *requested < 0.0 || *requested > 100.0)
        throw std::invalid_argument("Invalid logical fan percentage.");
    if (*requested == 0.0)
        return 0.0;
    // Legacy order: raise to the printer minimum, then add the tool offset.
    return std::clamp(std::max(*requested, m_minimum_speed_percent) + m_speed_offset_percent, 0.0, 100.0);
}

bool FanState::needs_encoding() const
{
    return m_speed.requested() && m_speed.needs_encoding_for(effective_speed_percent());
}

void FanState::mark_encoded()
{
    m_speed.mark_encoded_as(effective_speed_percent());
}

}} // namespace slic3r_api::GCodeGeneration
