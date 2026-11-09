/*
Run the isolated SDK DLL's firmware probe. The dynamic loader resolves its host
C ABI and SDK implementation before main() can report the probe's exit status.
The exported version table must also declare the fan/timing headers used in the
other translation unit, so package compatibility checks see the real requirements.

    main()
    |-- slic3r_plugin_abi_version(): verify the complete public requirement table
    `-- gcode_sdk_probe(): construct and exercise a public-SDK firmware session
*/

#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_extrusion_entity.h"
#include "libslic3r/Api/plugin/c/slic3r_fan_policy.h"
#include "libslic3r/Api/plugin/c/slic3r_printing_layer_time.h"
#include "libslic3r/Api/plugin/c/slic3r_plugin_types.h"

// Read the table exported by the fixture's registration translation unit.
extern "C" int32_t slic3r_plugin_abi_version(slic3r_major_minor_version *versions);
// Return zero only when the DLL's derived firmware encoder executes correctly.
extern "C" int gcode_sdk_probe();
// Forward the fixture result as the CTest process status without masking errors.
int main();

int main()
{
    const int32_t count = slic3r_plugin_abi_version(nullptr);
    if (count != SLIC3R_PLUGIN_API_COUNT)
        return 10;
    std::vector<slic3r_major_minor_version> versions(size_t(count), slic3r_major_minor_version{});
    if (slic3r_plugin_abi_version(versions.data()) != count)
        return 11;
    if (versions[SLIC3R_PLUGIN_API_FAN_POLICY].major != SLIC3R_PLUGIN_API_FAN_POLICY_MAJOR ||
        versions[SLIC3R_PLUGIN_API_FAN_POLICY].minor != SLIC3R_PLUGIN_API_FAN_POLICY_MINOR)
        return 12;
    if (versions[SLIC3R_PLUGIN_API_PRINTING_LAYER_TIME].major != SLIC3R_PLUGIN_API_PRINTING_LAYER_TIME_MAJOR ||
        versions[SLIC3R_PLUGIN_API_PRINTING_LAYER_TIME].minor != SLIC3R_PLUGIN_API_PRINTING_LAYER_TIME_MINOR)
        return 13;
    if (versions[SLIC3R_PLUGIN_API_EXTRUSION_ENTITY].major != SLIC3R_PLUGIN_API_EXTRUSION_ENTITY_MAJOR ||
        versions[SLIC3R_PLUGIN_API_EXTRUSION_ENTITY].minor != SLIC3R_PLUGIN_API_EXTRUSION_ENTITY_MINOR)
        return 14;
    return gcode_sdk_probe();
}
