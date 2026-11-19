// Kernel tests of the legacy first-layer fan program; no host is needed.
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <vector>

#include "FanLayerProgram.hpp"

namespace {
using namespace nematx_fan_layer_program;

int failures = 0;

void check(bool condition, const char *message)
{
    if (!condition) {
        std::printf("FAILED: %s\n", message);
        ++failures;
    }
}

// Percent of a rampable 60% role on each layer, -1 for the layers without program.
std::vector<int> rampable(int32_t enabled, int32_t disabled, int32_t gradient, uint32_t layers)
{
    std::vector<int> out;
    for (uint32_t layer = 0; layer < layers; ++layer) {
        const std::optional<Step> step = program_step(enabled, disabled, gradient, layer);
        if (!step)
            out.push_back(-1);
        else if (step->state == State::Off)
            out.push_back(0);
        else if (step->state == State::Ramp)
            out.push_back(int(60.f * step->factor + 0.01f)); // legacy rounding
        else
            out.push_back(60);
    }
    return out;
}
} // namespace

int main()
{
    // Enabled first layer, ramp down, off, ramp up, normal.
    check(rampable(1, 3, 1, 7) == std::vector<int>{60, 30, 0, 0, 30, 60, 60}, "E=1 D=3 G=1");
    // G > D: the ramp down and the ramp up overlap the disabled layers.
    check(rampable(1, 2, 3, 8) == std::vector<int>{60, 30, 0, 15, 30, 45, 60, 60}, "E=1 D=2 G=3");
    // Layer 1 of E=1 D=2 G=3 ramps to 0 instead of stopping: only rampable roles stop.
    check(program_step(1, 2, 3, 2)->state == State::Ramp && program_step(1, 2, 3, 2)->factor == 0.f, "ramp to zero");
    // G = 0: instant changes.
    check(rampable(1, 2, 0, 5) == std::vector<int>{60, 0, 0, 60, 60}, "G=0");
    // D = 0: no disabled layer, but the legacy ramp up still starts after E.
    check(rampable(1, 0, 2, 5) == std::vector<int>{60, 20, 40, 60, 60}, "D=0");
    // Legacy defaults written explicitly (E=0, D=1, G=4).
    check(rampable(0, 1, 4, 7) == std::vector<int>{0, 12, 24, 36, 48, 60, 60}, "legacy defaults");
    check(program_step(0, 1, 4, 0)->state == State::Ramp, "legacy first layer is a zero ramp");
    // Sentinel: nothing published without enabled layers, 4 layers with them.
    check(rampable(0, 1, -1, 3) == std::vector<int>{-1, -1, -1}, "sentinel without enabled layers");
    check(rampable(1, 1, -1, 8) == std::vector<int>{60, 0, 12, 24, 36, 48, 60, 60}, "sentinel with enabled layers");
    for (const auto &invalid : {std::vector<int32_t>{-1, 0, 0}, {0, -1, 0}, {0, 0, -2}}) {
        bool thrown = false;
        try {
            program_step(invalid[0], invalid[1], invalid[2], 0);
        } catch (const std::invalid_argument &) {
            thrown = true;
        }
        check(thrown, "invalid settings are rejected");
    }
    if (failures == 0)
        std::printf("fan first-layer program kernel tests passed\n");
    return failures == 0 ? 0 : 1;
}
