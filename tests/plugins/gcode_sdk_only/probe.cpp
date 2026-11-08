#include <cmath>

#include "libslic3r/Api/plugin/cpp/gcode/ArcGeometry.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/DefaultGCodeFirmwareSession.hpp"

/*
This fixture links a firmware session derived using only the copied public SDK.
Its exported probe constructs the complete session and calls an overridden
encoder, forcing the implementation and virtual table to resolve in a real DLL.

    gcode_sdk_probe()
    `-- ExternalFirmware::probe(): dispatch to the fixture's E reset encoder
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
    // The arc helper is header-only: an external firmware uses it directly.
    const c_vec2d center = arc_center(c_vec2d{ 5.0, 0.0 }, c_vec2d{ 0.0, 5.0 }, 5.0, true);
    return std::abs(center.x) < 1e-9 && std::abs(center.y) < 1e-9 ? 0 : 2;
}
