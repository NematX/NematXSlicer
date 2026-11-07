// Kernel tests of the instance temperature ramp; no host is needed.
#include <cstdio>
#include <vector>

#include "TemperatureRamp.hpp"

namespace {
using namespace nematx_smooth_temperature;

int failures = 0;

void check(bool condition, const char *message)
{
    if (!condition) {
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

    if (failures == 0)
        std::printf("smooth temperature kernel tests passed\n");
    return failures == 0 ? 0 : 1;
}
