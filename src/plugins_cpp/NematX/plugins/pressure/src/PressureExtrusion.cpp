///|/ Released under the GNU Affero General Public License, version 3 or later.

#include "PressureExtrusion.hpp"

#include <cmath>
#include <cstdint>
#include <stdexcept>

#include "libslic3r/Api/plugin/cpp/gcode/GCodeFormatter.hpp"

// Converts caller-owned cumulative nominal endpoints into corrected E values.
// The caller supplies already classified printing phases and the K copied for
// each movement. Charge C=K*v uses model speed, not filament speed or feed mm/min.
// Validation and quantization precede mutation so rejected inputs lose no charge.
// Output positions use the SDK's quantizer; the emitted relative E delta uses
// the last quantized output rather than independently rounded increments.
// Nominal and charge differences remain analytical, without quantization.
// State persists across cycles; only an explicit closed rebase changes its frame.
// The supported numeric domain is |N+C|*10^digits <= 1e14; no range clamp is used.
// No text or nominal writer state is read or changed, and no state is shared.
//
// Call tree:
//   Extrusion: validate digits and establish the quantized origin baseline.
//     quantize: check representability and use a local SDK formatter.
//   advance: validate inputs, compute charge/position/deltas, then store state.
//     quantize: quantize the combined nominal position and charge only once.
//   rebase: require zero charge and establish a new baseline without output.
//     quantize: validate the new origin before changing state.

namespace nematx_pressure {

Extrusion::Extrusion(int e_digits, double nominal_origin) : m_e_digits(e_digits)
{
    using slic3r_api::GCodeGeneration::GCodeFormatter;
    if (e_digits < 0 || e_digits >= static_cast<int>(GCodeFormatter::pow_10.size()))
        throw std::invalid_argument("E precision must be between 0 and 9");
    m_position = quantize(nominal_origin);
    m_nominal = nominal_origin;
}

ExtrusionResult Extrusion::advance(double cumulative_nominal_e_mm, double model_speed_mm_s,
                                   double pressure_factor_s)
{
    if (!std::isfinite(cumulative_nominal_e_mm) || !std::isfinite(model_speed_mm_s) ||
        !std::isfinite(pressure_factor_s) || model_speed_mm_s < 0. || pressure_factor_s < 0.)
        throw std::invalid_argument("Extrusion requires finite nominal and nonnegative finite speed and K");

    const double pressure = pressure_factor_s * model_speed_mm_s;
    const double position = quantize(cumulative_nominal_e_mm + pressure);
    const ExtrusionResult result = {position, position - m_position,
                                   cumulative_nominal_e_mm - m_nominal,
                                   pressure, pressure - m_pressure};
    if (!std::isfinite(result.delta) || !std::isfinite(result.nominal_delta))
        throw std::invalid_argument("Extrusion differences are not representable");

    m_nominal = cumulative_nominal_e_mm;
    m_position = position;
    m_pressure = pressure;
    return result;
}

void Extrusion::rebase(double new_nominal_origin)
{
    if (m_pressure != 0.)
        throw std::logic_error("Cannot rebase an open pressure charge");
    const double position = quantize(new_nominal_origin);
    m_nominal = new_nominal_origin;
    m_position = position;
}

double Extrusion::quantize(double position) const
{
    using slic3r_api::GCodeGeneration::GCodeFormatter;
    if (!std::isfinite(position) || !std::isfinite(position * GCodeFormatter::pow_10[m_e_digits]))
        throw std::invalid_argument("Extrusion position is not representable at this precision");
    const GCodeFormatter formatter(3, static_cast<std::uint16_t>(m_e_digits));
    return formatter.quantize_e(position);
}

} // namespace nematx_pressure
