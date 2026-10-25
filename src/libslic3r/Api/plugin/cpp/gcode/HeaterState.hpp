///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_plugin_cpp_gcode_HeaterState_hpp_
#define slic3r_Api_plugin_cpp_gcode_HeaterState_hpp_

#include <cstdint>

#include "EncodableState.hpp"

/*
Firmware heater state
=====================

HeaterState represents any independently controlled heater: tool, bed,
chamber or a future firmware-specific device. Heater targets come from entry
heater events or external G-code; the state distinguishes the requested target,
the encoded target and the target encoded with an explicit wait. Effective
targets add the device offset and are limited to [0,2000] degrees Celsius.
*/

namespace slic3r_api { namespace GCodeGeneration {

class HeaterState
{
public:
    void setup(int16_t temperature_offset);
    void reset_runtime_state();
    void synchronize_runtime_from(const HeaterState &source);
    // Import an external target and record whether that command explicitly waited for it.
    void synchronize_after_external_gcode(int16_t effective_temperature, bool waited);
    std::optional<int16_t> requested_temperature() const { return m_temperature.requested(); }
    std::optional<int16_t> encoded_temperature() const { return m_temperature.encoded(); }
    std::optional<int16_t> encoded_temperature_with_wait() const
    {
        return m_temperature.encoded_with_wait();
    }

    void request_temperature(std::optional<int16_t> temperature)
    {
        m_temperature.request(temperature);
    }
    // Return the requested target plus its device offset, limited to [0,2000]; zero effective temperature means off.
    int16_t effective_temperature() const;
    bool needs_encoding() const;
    bool needs_wait_encoding() const;
    // Record the current effective target and invalidate a wait acknowledgement for a different target.
    void mark_encoded();
    void mark_encoded_with_wait();
    void clear_encoded() { m_temperature.clear_encoded(); }
    void clear_encoded_with_wait() { m_temperature.clear_encoded_with_wait(); }

private:
    // Record a normal effective target while retaining a wait acknowledgement only for that same target.
    void mark_encoded_preserving_matching_wait(int16_t effective_temperature);

    // --- Runtime requested/encoded/wait-encoded state ---
    AwaitableEncodableState<int16_t> m_temperature;

    // --- Configuration cache populated by setup() ---
    // Per-device correction added before the effective target is limited and encoded.
    int16_t m_temperature_offset = 0;
};

}} // namespace slic3r_api::GCodeGeneration

#endif // slic3r_Api_plugin_cpp_gcode_HeaterState_hpp_
