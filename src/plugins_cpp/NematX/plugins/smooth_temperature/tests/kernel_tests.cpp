// Kernel tests of the instance temperature ramp; no host is needed. Exact
// examples protect the request-layer convention and rounding direction. A fixed
// matrix then checks each printed value, monotonicity and the final target;
// representative counter positions exercise long ramps without long loops.
//
//     main -> ramp_values / InstanceRamp::request, advance, value -> check
#include <algorithm>
#include <cstdio>
#include <cstdint>
#include <limits>
#include <vector>

#include "TemperatureRamp.hpp"

namespace {
using namespace nematx_smooth_temperature;

int failures = 0;
uint64_t checks = 0;

void check(bool condition, const char *message)
{
    ++checks;
    if (!condition) {
        if (failures < 20)
            std::printf("FAILED: %s\n", message);
        ++failures;
    }
}

// Values printed on the request layer and on the following layers without a new request.
std::vector<int> ramp_values(InstanceRamp ramp, int16_t target, uint32_t layers, int count)
{
    std::vector<int> out;
    ramp.request(target, layers);
    for (int layer = 0; layer < count; ++layer) {
        if (layer > 0)
            ramp.advance();
        out.push_back(ramp.value());
    }
    return out;
}
} // namespace

int main()
{
    // The example of the specification, N = 2: 230 on the first layer, then requests 210 and 180.
    InstanceRamp ramp = InstanceRamp::created(230);
    check(ramp.value() == 230, "a created ramp applies its temperature at once");
    ramp.request(210, 2);
    check(ramp.value() == 220, "the request layer takes the first step");
    ramp.request(180, 2);
    check(ramp.value() == 200, "a request restarts from the current value");
    ramp.advance();
    check(ramp.value() == 180, "the target is reached on the N-th layer");
    ramp.advance();
    check(ramp.value() == 180, "the target stays after the ramp");

    // Rounding to the nearest degree, halves towards the target.
    check(ramp_values(InstanceRamp::created(200), 230, 4, 5) == std::vector<int>{208, 215, 223, 230, 230}, "rise");
    check(ramp_values(InstanceRamp::created(230), 200, 4, 5) == std::vector<int>{222, 215, 207, 200, 200}, "fall");
    // A difference smaller than N repeats some values.
    check(ramp_values(InstanceRamp::created(200), 202, 5, 6) == std::vector<int>{200, 201, 201, 202, 202, 202},
          "small difference");
    // N = 0 and N = 1 apply the target at once.
    check(ramp_values(InstanceRamp::created(200), 230, 0, 2) == std::vector<int>{230, 230}, "N = 0");
    check(ramp_values(InstanceRamp::created(200), 230, 1, 2) == std::vector<int>{230, 230}, "N = 1");

    // A request during a ramp starts from the value printed on the previous layer.
    InstanceRamp restarted = InstanceRamp::created(200);
    restarted.request(230, 3);
    restarted.advance();
    check(restarted.value() == 220, "ramp before the restart");
    check(ramp_values(restarted, 190, 3, 4) == std::vector<int>{210, 200, 190, 190}, "restart during a ramp");

    // A new immediate request cancels the remaining steps, including when the target is unchanged.
    for (uint32_t immediate : {0u, 1u}) {
        InstanceRamp cancelled = InstanceRamp::created(200);
        cancelled.request(230, 8);
        cancelled.advance();
        cancelled.request(230, immediate);
        check(cancelled.value() == 230, "an immediate request cancels an active ramp to the same target");
        cancelled.advance();
        check(cancelled.value() == 230, "a cancelled ramp cannot resume");
    }

    // Check exact values and invariants across the temperature domain and short/long ramps.
    // The reference rounds a quotient with its remainder, independently of value()'s formula.
    const std::vector<int16_t> temperatures{0, 1, 100, 180, 199, 200, 201, 202, 230, 300, 32767};
    for (int16_t start : temperatures)
        for (int16_t target : temperatures)
            for (uint32_t n : {0u, 1u, 2u, 3u, 4u, 5u, 7u, 50u, 1000u}) {
                InstanceRamp sample = InstanceRamp::created(start);
                sample.request(target, n);
                int previous = start;
                for (uint32_t step = 1; step <= n + 2; ++step) {
                    const int value = sample.value();
                    const int64_t distance = std::abs(int(target) - int(start));
                    const int64_t numerator = distance * int64_t(step);
                    const int64_t rounded = n <= 1 || step >= n ? distance :
                        numerator / n + ((numerator % n) * 2 >= n ? 1 : 0);
                    const int expected = int(start) + (target < start ? -int(rounded) : int(rounded));
                    check(value == expected, "the value agrees with quotient/remainder rounding");
                    check(value >= std::min(start, target) && value <= std::max(start, target), "no overshoot");
                    check(target >= start ? value >= previous : value <= previous, "monotone progression");
                    check(step < n || value == target, "exact target at and after N");
                    previous = value;
                    sample.advance();
                }
            }

    // Restart at different positions, including a change of direction before the target.
    for (uint32_t position = 1; position <= 7; ++position) {
        InstanceRamp sample = InstanceRamp::created(200);
        sample.request(240, 8);
        for (uint32_t step = 1; step < position; ++step)
            sample.advance();
        const int printed = sample.value();
        sample.request(180, 2);
        check(sample.from == printed, "restart from the value actually printed, not the old target");
        check(sample.value() == printed - (printed - 180 + 1) / 2, "reverse direction with a rounded first step");
        sample.advance();
        check(sample.value() == 180, "reverse ramp finishes on its second layer");
    }

    // Probe a maximum-sized duration without iterating through billions of layers.
    InstanceRamp large = InstanceRamp::created(0);
    large.request(32767, std::numeric_limits<uint32_t>::max());
    check(large.value() == 0, "a very long ramp starts with a repeated value");
    large.done = large.layers / 2;
    check(large.value() == 16383, "large products use wide integer arithmetic");
    large.done = large.layers - 1;
    large.advance();
    check(large.value() == 32767 && large.done == large.layers, "maximum duration reaches its target");
    large.advance();
    check(large.done == large.layers, "the layer counter saturates without overflowing");

    if (failures == 0)
        std::printf("smooth temperature kernel tests passed (%llu checks)\n", static_cast<unsigned long long>(checks));
    return failures == 0 ? 0 : 1;
}
