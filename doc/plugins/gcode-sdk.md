# External G-code firmware and reader SDK

The C ABI remains the boundary between the slicer and a plugin DLL. Reusable
C++ implementations are compiled into a **static** library owned by each plugin:
no host C++ class is exported and no host internal header is needed.

## Build

```cmake
# The checkout (or SDK copy) must preserve libslic3r/Api/plugin below an include root.
add_subdirectory("${SLICER_SOURCE}/src/libslic3r/Api/plugin" sdk)
target_link_libraries(my_firmware PRIVATE slic3r::plugin_sdk_cpp "${SLICER_HOST_LIBRARY}")
```

The target requires C++17, Boost headers (Karma numeric formatting), and fast_float
headers. In the host checkout fast_float is bundled in `src/fast_float`. With a
standalone SDK copy, set `SLIC3R_SDK_FAST_FLOAT_INCLUDE_DIR` to the directory
containing `fast_float/fast_float.h` and use `CMAKE_PREFIX_PATH` / `Boost_INCLUDE_DIR`
to locate Boost. Neither dependency appears in the SDK's public headers.

The host incorporates the same compiled helper objects into its own archive, so
built-in and external firmware providers use the same implementation without
compiling it twice or introducing a circular static-library dependency.
Match compiler, architecture, runtime library and
configuration to the host. A static SDK library does not replace the host C ABI
import library; the latter resolves the SDK's calls to configuration, geometry,
orchestrator and PrintingPlan functions.

## Firmware

Include `libslic3r/Api/plugin/cpp/gcode/DefaultGCodeFirmwareSession.hpp` and derive
from `slic3r_api::GCodeGeneration::DefaultGCodeFirmwareSession`. Override only the
required syntax encoders and startup hooks. The SDK also contains
`SingleAccelerationRegisterGCodeFirmwareSession` for dialects sharing one
print/travel acceleration register.

Publish a fresh session through `make_gcode_firmware_instance()` from a
`GCODE_FIRMWARE` provider. The session owns export-local state. Select the provider
through `gcode_firmware_plugin`; no `gcode_flavor` compatibility is introduced here.

The default session is a reusable policy, not a restriction of the firmware ABI:
a provider may implement `GCodeFirmwareSession` directly when its state policy is
different. In particular, `encode_extrusion_axis_reset()` only changes syntax:
the default traversal still decides when it resets its E accounting to zero.
Returning empty text from that encoder does **not** suppress the accounting reset.

## Reader

Include `libslic3r/Api/plugin/cpp/gcode/BuiltinGCodeReaderBase.hpp` and derive from
`slic3r_api::GCodeGeneration::Reader::BuiltinGCodeReaderBase`. Its historical name
is retained, but this base now belongs to the public SDK and is usable externally.

The base reassembles arbitrary chunks, owns run-local state, dispatches common
commands, recognizes preview comments and conservatively invalidates unknown
state. The built-in readers use this same base, not a second implementation.

- Override a `process_*` handler to change an existing command's semantics.
- Override `consume_numeric_command`, delegating to the parent first, to add
  command numbers.
- Override `consume_text_command` to handle a different lexical form. The parsed
  line exposes `raw()` so a dialect can parse `=`, non-E extrusion axes, inline
  positioning modes or line-number prefixes itself. Return true only after
  publishing all required events; returning false continues normal dispatch.
- Use `publish_items()` for synchronous state/movement events. Preserve physical
  source line numbers and never retain borrowed callback arguments.
- Override probe/identity metadata in the derived provider. Portable reader state
  uses schema id zero; private state requires an orchestrator-registered schema.

Register the reader separately and return its id from the firmware session's
`reader_plugin_id()`. This supports preview reading and custom-script state
reconciliation without forwarding events through another reader.

## Isolated link and runtime test

`tests/plugins/gcode_sdk_only` is an independent CMake project. It copies only
the public SDK and bundled fast_float into a sandbox, builds the SDK without
the host PCH, and links a real DLL containing a derived firmware session and a
registered reader provider. A smoke executable loads the DLL and constructs the
firmware, catching unresolved symbols that a compile-only test cannot detect.

```powershell
cmake -S tests/plugins/gcode_sdk_only -B build/gcode-sdk-only `
  -DSLIC3R_HOST_LIBRARY=C:/path/build/debug-tests/src/Debug/Slic3r.lib `
  -DSLIC3R_HOST_RUNTIME_DIR=C:/path/build/debug-tests/src/Debug `
  -DSLIC3R_RUNTIME_DEPENDENCY_DIRS=C:/path/deps/usr/local/bin `
  -DCMAKE_PREFIX_PATH=C:/path/deps/usr/local
cmake --build build/gcode-sdk-only --config Debug
ctest --test-dir build/gcode-sdk-only -C Debug --output-on-failure
```

The main `plugins_tests` suite exercises the same base's reader events, dialect
overrides, preview markers and firmware/session behavior.
