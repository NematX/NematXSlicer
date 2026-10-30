#include <cmath>

#include "libslic3r/Api/plugin/cpp/HoleCompensation.hpp"
#include "libslic3r/Api/plugin/cpp/FanPolicy.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingLayerTimeProperty.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/ArcGeometry.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/DefaultGCodeFirmwareSession.hpp"

/*
This fixture links a firmware session derived using only the copied public SDK.
Its exported probe constructs the complete session and calls an overridden
encoder, forcing the implementation and virtual table to resolve in a real DLL.
The same probe retains a reference to the shared hole-compensation core and
checks arc geometry without including any internal host header.

    gcode_sdk_probe()
    |-- ExternalFirmware::probe(): dispatch to the fixture's E reset encoder
    |-- compensate_convex_holes: require resolution from the SDK archive
    |-- FanPolicy stages: link the shared kernels and validate the public context
    `-- arc_center(): check the public signed-radius calculation
*/

using namespace slic3r_api::GCodeGeneration;

namespace {
class ExternalFirmware final : public DefaultGCodeFirmwareSession
{
public:
    // Exercise virtual dispatch through the same protected encoder a dialect customizes.
    std::string probe() const { return encode_extrusion_axis_reset(); }
protected:
    // Return a distinctive encoder result without modifying machine state.
    std::string encode_extrusion_axis_reset() const override
    { return "A[SET_POSITION POS=0]\n"; }
};
}

#ifdef _WIN32
#define PROBE_EXPORT __declspec(dllexport)
#else
#define PROBE_EXPORT __attribute__((visibility("default")))
#endif

// Constructing a derived session pulls its vtable and implementations into a
// real shared library. A compile-only test would miss unresolved SDK symbols.
extern "C" PROBE_EXPORT int gcode_sdk_probe();

extern "C" PROBE_EXPORT int gcode_sdk_probe()
{
    ExternalFirmware firmware;
    if (firmware.probe() != "A[SET_POSITION POS=0]\n")
        return 1;
    // The hole compensation core comes from the SDK archive: keep a real
    // reference so that the link fails if it is missing.
    std::vector<slic3r_api::StoredExPolygonCollection> (*volatile compensate)(
        storage_handle *, const std::vector<slic3r_api::ExPolygonCollection> &,
        const slic3r_api::ConvexHoleDelta &, bool &) = &slic3r_api::compensate_convex_holes;
    if (compensate == nullptr)
        return 3;
    slic3r_api::FanPolicy::Settings settings;
    settings.configured[RAW_FAN_ROLE_DEFAULT] = 20;
    settings.validate();
    const slic3r_api::FanPolicy::Context base = slic3r_api::FanPolicy::make_base(settings);
    const slic3r_api::FanPolicy::Context timed = slic3r_api::FanPolicy::apply_layer_time(base, settings, 10.);
    const slic3r_api::FanPolicy::Context closed = slic3r_api::FanPolicy::close_first_layers(timed, settings, 0);
    closed.validate(RAW_FAN_POLICY_PHASE_CLOSED);
    if (closed.roles[RAW_FAN_ROLE_DEFAULT].percent != 20)
        return 5;
    const slic3r_api::PrintingLayerTimeProperty duration{{10., RAW_PRINTING_LAYER_TIME_ORIGIN_RESOLVED}};
    duration.validate();
    // The arc helper is header-only: an external firmware uses it directly.
    const c_vec2d center = arc_center(c_vec2d{ 5.0, 0.0 }, c_vec2d{ 0.0, 5.0 }, 5.0, true);
    return std::abs(center.x) < 1e-9 && std::abs(center.y) < 1e-9 ? 0 : 2;
}
