// Public-SDK consumer: link the shared fan kernels from the standalone SDK
// archive, without private host headers or a copy of their implementation.
#include <iostream>
#include <stdexcept>

#include "libslic3r/Api/plugin/cpp/FanPolicy.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingLayerTimeProperty.hpp"

namespace {
// Keep assertions active regardless of build flags and identify a bad contract.
void require(bool condition, const char *message)
{
    if (!condition)
        throw std::runtime_error(message);
}
}

int main()
{
    using namespace slic3r_api;
    try {
        FanPolicy::Settings settings;
        settings.configured[RAW_FAN_ROLE_DEFAULT] = 20;
        settings.configured[RAW_FAN_ROLE_PERIMETER] = 40;
        settings.slowdown_seconds = 5;
        settings.fan_seconds = 10;
        const FanPolicy::Context base = FanPolicy::make_base(settings, 0);
        const FanPolicy::Context time = FanPolicy::apply_layer_time(base, settings, 7.5);
        const FanPolicy::Context closed = FanPolicy::close_first_layers(time);
        closed.validate(RAW_FAN_POLICY_PHASE_CLOSED);
        require(closed.roles[RAW_FAN_ROLE_DEFAULT].percent == 60, "default time interpolation");
        require(closed.roles[RAW_FAN_ROLE_PERIMETER].percent == 70, "role time interpolation");
        require(closed.roles[RAW_FAN_ROLE_PERIMETER].configured_percent == 40, "configured reference");
        EPropertyFan stop = {};
        stop.set(0, FAN_REQUEST_MODE_OVERRIDE);
        require(FanPolicy::transform_request(stop, 100, FAN_REQUEST_MODE_AUTOMATIC).percent == 0,
                "protected explicit stop");
        PrintingLayerTimeProperty timing = {};
        timing.duration_seconds = 7.5;
        timing.origin = RAW_PRINTING_LAYER_TIME_ORIGIN_RESOLVED;
        timing.validate();
        require(timing.is_resolved(), "resolved timing provenance");
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    std::cout << "Public fan policy SDK contracts passed\n";
    return 0;
}
