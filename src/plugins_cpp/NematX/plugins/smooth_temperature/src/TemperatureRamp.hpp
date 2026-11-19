///|/ Copyright (c) 2026 NematX
///|/
///|/ Released under the GNU Affero General Public License, version 3 or later.
#pragma once

/*
Temperature ramp of one object instance on one tool
====================================================

InstanceRamp follows the general temperature of one instance, without any host
dependency. A request restarts the ramp from the value printed on the previous
layer: the request layer already receives the first step, and the target is
reached on the N-th layer. With done the layers counted since the request,
request layer included, the printed value is

    from + (target - from) * done / N

rounded to the nearest degree, halves towards the target, and exactly the
target from done = N on. N = 0 or 1 applies the target at once.

    InstanceRamp ramp = InstanceRamp::created(230); // first layer: 230
    ramp.request(210, 2);                           // next layer: 220
    ramp.request(180, 2);                           // next layer: 200
    ramp.advance();                                 // next layer: 180
*/

#include <cstdint>
#include <cstdlib>

namespace nematx_smooth_temperature {

// General-temperature ramp of one object instance on one tool.
struct InstanceRamp
{
    // General value printed before the latest request.
    int16_t from = 0;
    // Latest requested general temperature.
    int16_t target = 0;
    // N captured with the request.
    uint32_t layers = 0;
    // Counted layers since the request, request layer included.
    uint32_t done = 0;

    // Start at a temperature applied at once, as on the first layer of an instance.
    static InstanceRamp created(const int16_t temperature)
    {
        InstanceRamp ramp;
        ramp.from = temperature;
        ramp.target = temperature;
        return ramp;
    }

    // Value printed on the current layer.
    int16_t value() const
    {
        if (done >= layers)
            return target;
        const int64_t step = (int64_t(target) - int64_t(from)) * int64_t(done);
        const int64_t rounded = (std::llabs(step) * 2 + int64_t(layers)) / (2 * int64_t(layers));
        return int16_t(from + (step < 0 ? -rounded : rounded));
    }

    // Restart from the current value towards a new target; the current layer takes the first step.
    void request(const int16_t temperature, const uint32_t ramp_layers)
    {
        from = value();
        target = temperature;
        layers = ramp_layers;
        done = 1;
    }

    // Count one more layer of the instance without a new request.
    void advance()
    {
        if (done < layers)
            ++done;
    }
};

} // namespace nematx_smooth_temperature
