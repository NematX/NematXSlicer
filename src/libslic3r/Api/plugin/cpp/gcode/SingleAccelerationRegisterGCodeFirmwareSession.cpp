///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "SingleAccelerationRegisterGCodeFirmwareSession.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include "libslic3r/Api/plugin/cpp/ConfigViews.hpp"

/*
Single acceleration-register firmware session implementation
==============================================================

The standard session performs the actual encoding and acknowledges the active
logical category. This specialization then clears the opposite acknowledgement
because both categories compete for the same physical firmware register.
*/

namespace slic3r_api { namespace GCodeGeneration {

void SingleAccelerationRegisterGCodeFirmwareSession::setup_firmware(const Config &config)
{
    DefaultGCodeFirmwareSession::setup_firmware(config);
    m_print_acceleration_fallback.reset();

    // Ignore means that machine limits must not influence generated motion.
    // Other policies may use the extrusion limit as a conservative value to
    // restore after a travel command changed the one shared register.
    const ConfigOption usage = config.get("machine_limits_usage");
    if (!usage.valid() || !usage.is_enabled() || usage.serialize() == "ignore")
        return;

    const double fallback = config.vector_float_or_default(
        "machine_max_acceleration_extruding", 0, 0.0);
    if (fallback <= 0.0)
        return;
    if (!std::isfinite(fallback) ||
        fallback > double(std::numeric_limits<uint32_t>::max()))
        throw std::invalid_argument(
            "The shared acceleration register needs a finite, representable "
            "machine_max_acceleration_extruding fallback.");
    const long long rounded = std::llround(fallback);
    if (rounded <= 0)
        return;
    m_print_acceleration_fallback = uint32_t(rounded);
}

std::string SingleAccelerationRegisterGCodeFirmwareSession::write_acceleration(
    PreparedMove::Kind kind)
{
    const bool is_travel =
        kind == PreparedMove::Kind::Travel || kind == PreparedMove::Kind::Wipe;

    // A travel command overwrites the same physical register later used for
    // extrusion. If no print value has ever been requested, establish the
    // configured machine fallback first; without one, suppress the travel
    // switch because the session could not safely restore print acceleration.
    if (is_travel && gantry().needs_travel_acceleration_encoding() &&
        !gantry().requested_print_acceleration()) {
        if (!m_print_acceleration_fallback)
            return {};
        gantry().request_print_acceleration(m_print_acceleration_fallback);
    }

    // Remember whether the standard implementation is about to encode a
    // transition. The returned text may legitimately be empty, so its size
    // cannot be used to decide whether the state was acknowledged.
    const bool encodes_travel =
        is_travel && gantry().needs_travel_acceleration_encoding();
    const bool encodes_print = kind == PreparedMove::Kind::Extrusion &&
                               gantry().needs_print_acceleration_encoding();

    std::string output = DefaultGCodeFirmwareSession::write_acceleration(kind);

    // The command just written replaced the one shared physical register.
    // Preserve both requests, but force the other category to be restored when
    // a later movement needs it again.
    if (encodes_travel)
        gantry().clear_print_acceleration_encoded();
    else if (encodes_print)
        gantry().clear_travel_acceleration_encoded();

    return output;
}

}} // namespace slic3r_api::GCodeGeneration
