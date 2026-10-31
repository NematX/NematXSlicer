///|/ Copyright (c) 2026 NematX
///|/
///|/ Released under the GNU Affero General Public License, version 3 or later.
#pragma once

// Legacy first-layer fan program, without any host dependency.
//
// With k the zero-based layer ordinal, E = enable_fan_first_layers,
// D = disable_fan_first_layers and G = fan_speed_layer_gradient:
// - k < E: normal fan (the fan is enabled on the first E layers);
// - from E, ramp down over G layers (at most D), factor 1 - (k + 1 - E) / (min(G, D - 1) + 1);
// - then off until E + D;
// - from E + D, ramp up over G layers, factor (k + 1 - (E + D)) / (G + 1);
// - then normal.
// These are the conditions of the legacy cooling buffer, including its overlaps
// when G > D. A ramp scales only the rampable roles, even at factor 0; off stops
// every role.

#include <algorithm>
#include <cstdint>
#include <optional>
#include <stdexcept>

namespace nematx_fan_layer_program {

enum class State { Normal, Off, Ramp };

struct Step
{
    State state = State::Normal;
    float factor = 1.f; // Ramp only
};

inline Step legacy_step(int32_t enabled, int32_t disabled, int32_t gradient, uint32_t layer)
{
    const int64_t k = layer, e = enabled, g = gradient, disabled_end = e + disabled;
    Step step;
    if (!(k < e + g || k >= disabled_end)) {
        step.state = State::Off;
    } else if (k >= e && k < e + g && k < disabled_end) {
        // Here disabled >= 1 and gradient >= 1, so the divisor is at least 1.
        step.state = State::Ramp;
        step.factor = 1.f - float(k + 1 - e) / float(std::min(g, int64_t(disabled) - 1) + 1);
    } else if (k >= disabled_end && k < disabled_end + g) {
        step.state = State::Ramp;
        step.factor = float(k + 1 - disabled_end) / float(g + 1);
    }
    return step;
}

// Resolve the gradient sentinel. With E = 0 and G = -1 no program is published:
// the host keeps its disable_fan_first_layers/full_fan_speed_layer program.
// With E > 0 and G = -1 the gradient takes the legacy default, 4 layers.
inline std::optional<Step> program_step(int32_t enabled, int32_t disabled, int32_t gradient, uint32_t layer)
{
    if (enabled < 0 || disabled < 0 || gradient < -1)
        throw std::invalid_argument("enable_fan_first_layers and disable_fan_first_layers must be >= 0, "
                                    "fan_speed_layer_gradient >= -1.");
    if (gradient == -1) {
        if (enabled == 0)
            return std::nullopt;
        gradient = 4;
    }
    return legacy_step(enabled, disabled, gradient, layer);
}

} // namespace nematx_fan_layer_program
