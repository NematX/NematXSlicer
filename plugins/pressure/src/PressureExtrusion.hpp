///|/ Released under the GNU Affero General Public License, version 3 or later.

// Provides quantized filament positions for callers with classified printing phases.
// Construct Extrusion with fixed E precision and a nominal origin, then advance
// with cumulative nominal filament mm, dominant-axis model mm/s and K in seconds.
// Q is the SDK's E quantizer at that precision, with ties rounded away from zero.
// Each owned ExtrusionResult gives Q(N+K*v) and its delta from the previous output.
// Select either position or delta for output; neither is new nominal material.
// Keep the instance across closed cycles. Rebase only after charge has reached zero
// and the caller has proved a new nominal baseline; rebase emits nothing.
// Relative working sums start at zero, independently of external machine resets.
// Absolute origins may be off-grid; the initial output baseline is Q(origin).
// No G-code text, parser or writer state, geometry, configuration, nominal
// accounting or retraction state is accessed.
#pragma once

namespace nematx_pressure {

// All positions and deltas are filament mm; pressure is the algebraic charge K*v.
struct ExtrusionResult
{
    double position;
    double delta;
    double nominal_delta;
    double pressure;
    double pressure_delta;
};

// Owns one coordinate frame and its last emitted quantized position and charge.
class Extrusion
{
public:
    // Fixes precision (0..9) and baseline Q(origin), with zero initial charge.
    // Invalid precision or nonfinite/unrepresentable origin throws invalid_argument.
    Extrusion(int e_digits, double nominal_origin);
    // Computes Q(N+K*v), output delta and analytical nominal/charge differences.
    // N must be the caller's nondecreasing cumulative nominal in this frame.
    // Finite N, finite nonnegative speed/K and representable results are required;
    // invalid_argument leaves state unchanged. A zero K still releases old charge.
    ExtrusionResult advance(double cumulative_nominal_e_mm, double model_speed_mm_s,
                            double pressure_factor_s);
    // Sets a new nominal origin and Q(origin) baseline without emitting extrusion.
    // Nonzero charge throws logic_error; invalid origin throws invalid_argument.
    // Both failures preserve state; the caller must prove the baseline from a real
    // reset/mode change or the nominal start after an unchanged absolute E operation.
    void rebase(double new_nominal_origin);

private:
    // Uses the SDK's E quantizer with validated digits; rejects nonfinite input
    // or scaled input before returning a quantized filament position.
    double quantize(double position) const;

    int m_e_digits;
    double m_nominal = 0.;
    double m_position = 0.;
    double m_pressure = 0.;
};

} // namespace nematx_pressure
