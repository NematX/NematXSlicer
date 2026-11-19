# nematx_second_layer_flow

## Purpose

This NematXSlicer plugin scales deposited extrusion volume on the second printed layer. It applies the region-specific ratio to **mm3_per_mm** before the host's **DefaultSpeed** step calculates extrusion speeds.

It is intended to compensate on the second layer for intentional first-layer over-extrusion used to improve adhesion. Use the inverse ratio: for example, **125%** on the first layer corresponds to **80%** on the second.

The plugin is packaged as **nematx-extrusion** and has the plugin ID **nematx_second_layer_flow**.

## Setting

The region-level FFF print setting is **second_layer_flow_ratio**:

- Default: **100%**.
- GUI minimum: **1%**.
- **100%** preserves the extrusion attributes unchanged.
- **150%** multiplies the affected deposited volume by **1.5**.
- Runtime validation still requires a finite, strictly positive value; **0%** rejects the export.

## Affected paths

The plugin follows source-layer links to identify the second printed layer of each object. It applies the setting to eligible perimeter, infill, and gap-fill paths. Travel, wipe, retract, unretract, wipe-tower, and milling paths are excluded. Paths already expressed as E per millimeter are also excluded.

Support, skirt, and brim are printed from auxiliary layers, which are never changed. The legacy slicer did not apply the ratio to support either. Unlike the legacy slicer, a skirt taller than one layer keeps its flow on the second layer.

When an extrusion crosses a boundary between regions with different ratios, the plugin splits the path at the boundary and applies each region's ratio to its fragment. Regions at **100%** retain their original flow while regions at **150%** receive 50% more deposited volume. Instance offsets are included when matching paths to region geometry. Only the regions attached to the extrusion's source are considered, as in the host's first-layer flow plugin.

The plugin changes **mm3_per_mm**; it does not directly set feed rate. NematXSlicer's later speed calculation and volumetric limits remain in effect.

## Validation behavior

The setting's GUI minimum is **1%**. Runtime checks independently require the normalized ratio and computed flow to be finite and strictly positive; a profile containing **0%** still rejects the export. For regional paths, the plugin also checks that splitting preserves the source path length. If a later path or fragment fails validation, earlier changes in that export are not rolled back.

## Build and integration checks

Configure against the host source checkout and explicitly pass the matching host library, executable, and generated default-activation file. The executable is used for debugger launches and integration checks; it is not used for linking. For example, with Visual Studio 2022 on Windows x86_64:

    cmake -S . -B .build -G "Visual Studio 17 2022" -A x64 -DSUPERSLICER_SOURCE_DIR="C:/path/to/NematXSlicer_v3" -DSUPERSLICER_HOST_LIBRARY="C:/path/to/NematXSlicer_v3/build/src/RelWithDebInfo/Slic3r.lib" -DSUPERSLICER_HOST_EXECUTABLE="C:/path/to/NematXSlicer_v3/build/src/RelWithDebInfo/Slic3r_console.exe" -DSUPERSLICER_DEFAULT_ACTIVATED_FILE="C:/path/to/NematXSlicer_v3/build/src/generated-resources/plugins/default_activated.ini" -DSUPERSLICER_PLUGIN_REPOSITORY_DIR="C:/path/to/NematXSlicer_v3/build/src/RelWithDebInfo/resources/plugins" -DSUPERSLICER_VERSION_FULL="<host-version>"
    cmake --build .build --config RelWithDebInfo --target nematx_second_layer_flow

On Windows, the host library is the `Slic3r.lib` import library. On Linux and macOS, pass the matching linkable host `libslic3r` archive and list its required transitive libraries explicitly in **SUPERSLICER_HOST_LINK_LIBRARIES** as a semicolon-separated CMake list. Do not use an archive built for a different OS, architecture, ABI, compiler runtime, or host revision; a static host archive linked into the plugin must be built with position-independent code. The project does not discover Unix link dependencies automatically. **SUPERSLICER_PLUGIN_REPOSITORY_DIR** is optional; when omitted, the ZIP remains in the build output and is not copied to a host repository.

The target architecture comes from the CMake toolchain or generator platform. Outputs are placed under `package/<os>-<arch>/<config>` (for example, `package/windows-x86_64/RelWithDebInfo`). The archive base name remains `nematx-extrusion_0.1.0_<host-version>.zip`; the plugin inside uses the loader's platform filename (`plugin.dll`, `plugin.so`, or `plugin.dylib`).

Run the integration checks after building the plugin, passing both the host working directory and its executable:

    python tests/second_layer_flow/test_second_layer_flow.py --host-dir "C:/path/to/NematXSlicer_v3/build/src/RelWithDebInfo" --host-executable "C:/path/to/NematXSlicer_v3/build/src/RelWithDebInfo/Slic3r_console.exe" --build-dir .build

The checks cover uniform ratios, a zero-percent rejection, a global volumetric limit, adjacent **100%**/**150%** regions with a split at their boundary, and separate islands with different regional ratios. CMake prepares a host-only baseline datadir from the host's default plugin activations and stages the plugin datadir under **.build/test-data**. Generated models, logs, and G-code go under **.build/test-results**. Pass **--host-datadir** or **--plugin-datadir** when using custom CMake datadir paths.

The configured target matrix is Windows x86_64 and ARM64, Linux x86_64 and ARM64, and macOS ARM64. Windows x86_64 is the only combination built and exercised in this workspace; the remaining four combinations have not been validated here.
