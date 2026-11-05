///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "HeaterState.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>

/*
Firmware heater state implementation
====================================

Requests remain uncorrected so runtime state can be copied between devices.
Normal and wait encodings use the effective corrected target because they
describe output already generated for the physical machine. Requests are set by
heater entry events or imported external G-code. Effective targets add the
device offset and are limited to the heater range of 0 to 2000 degrees Celsius.
*/

namespace slic3r_api { namespace GCodeGeneration {

void HeaterState::setup(int16_t temperature_offset)
{
    m_temperature_offset = temperature_offset;
}

void HeaterState::reset_runtime_state()
{
    m_temperature.reset_runtime_state();
}

void HeaterState::synchronize_runtime_from(const HeaterState &source)
{
    m_temperature.synchronize_runtime_from(source.m_temperature);
}

// Import an externally emitted effective target, derive and store the uncorrected request from the external target, and clear an older wait acknowledgement when a non-waiting command changes the target; invalid corrected ranges throw and state is updated only for valid temperatures.
void HeaterState::synchronize_after_external_gcode(int16_t effective_temperature, bool waited)
{
    // Requests are stored before the device correction, while encoded values
    // describe the physical value which was present in the imported command.
    const int32_t requested = int32_t(effective_temperature) - int32_t(m_temperature_offset);
    if (requested < int32_t(std::numeric_limits<int16_t>::min()) ||
        requested > int32_t(std::numeric_limits<int16_t>::max()))
        throw std::invalid_argument("External G-code temperature is outside the corrected heater range.");
    m_temperature.request(int16_t(requested));
    if (waited)
        m_temperature.mark_encoded_with_wait_as(effective_temperature);
    else
        mark_encoded_preserving_matching_wait(effective_temperature);
}

// Add the device offset in a wide integer and return the target limited to [0,2000] without changing state; an absent request throws logic_error.
int16_t HeaterState::effective_temperature() const
{
    const std::optional<int16_t> requested = m_temperature.requested();
    if (!requested)
        throw std::logic_error("No heater temperature is requested.");
    const int32_t effective = int32_t(*requested) + int32_t(m_temperature_offset);
    return int16_t(std::clamp(effective, int32_t(0), int32_t(2000)));
}

bool HeaterState::needs_encoding() const
{
    return m_temperature.requested() &&
           m_temperature.needs_encoding_for(effective_temperature());
}

bool HeaterState::needs_wait_encoding() const
{
    return m_temperature.requested() &&
           m_temperature.needs_wait_encoding_for(effective_temperature());
}

// Record the current corrected target as normally emitted and clear a wait acknowledgement unless it names the same effective target; missing requests still fail through the generic state register.
void HeaterState::mark_encoded()
{
    mark_encoded_preserving_matching_wait(effective_temperature());
}

// Record one effective target as normally emitted and retain the separate wait acknowledgement only when it refers to the same target; the requested value must already exist.
void HeaterState::mark_encoded_preserving_matching_wait(int16_t effective_temperature)
{
    const std::optional<int16_t> encoded_with_wait = m_temperature.encoded_with_wait();
    m_temperature.mark_encoded_as(effective_temperature);
    if (!encoded_with_wait || *encoded_with_wait != effective_temperature)
        m_temperature.clear_encoded_with_wait();
}

void HeaterState::mark_encoded_with_wait()
{
    m_temperature.mark_encoded_with_wait_as(effective_temperature());
}

}} // namespace slic3r_api::GCodeGeneration
