# NematX temperature smoothing

Package `nematx-smooth-temperature`; plugin `nematx_smooth_temperature`.

The region setting `print_temperature_smooth_change` (N layers, default 0) spreads a change of the general nozzle temperature of each object over N layers instead of applying it at once. A change restarts the ramp from the value printed on the previous layer; the layer of the change already takes the first step and the new temperature is reached on the N-th layer. Values are rounded to the nearest degree, halves towards the target. N = 0 or 1 applies changes at once.

Example with N = 2: the first layer asks for 230 and prints 230; the second asks for 210 and prints 220; the third asks for 180 and prints 200; the fourth prints 180.

## Behavior

- The plugin is a member of the host's `layer_extrusion_edit.temperature` group, preferred to the host provider `layer_extrusion_edit.temperature.default` when both are active. It derives from that provider: the host planner computes every heater request, then the plugin rewrites the nozzle setpoints before they are written. The standalone package compiles the planner's two host sources, `DefaultTemperature.cpp` and `ExtrusionProcessParameterHelpers.cpp`.
- Each object instance has one ramp per tool. The temperature asked by all the regions of an instance on a layer is general: object settings, height ranges, or a modifier covering the whole instance on that layer. When the regions disagree, the general temperature comes from the object settings and the height range of the layer, and the other temperatures, for example those of a partial modifier, are overrides: they apply at once and the ramp continues underneath, as if they did not exist.
- N is read with the general temperature: from the regions on a layer where they agree, otherwise from the object settings and the height range.
- Setting N to 0 or 1 cancels an active ramp immediately, even when its target has not changed.
- The first layer of an instance applies its temperature at once, also with complete objects.
- A tool's ramp waits on the layers where the tool does not print the instance. With ooze prevention, the standby temperature is an override: a returning tool is heated to its next ramp value, with the wait decided by the host.
- The plugin adds no wait; the host waits stay, on the ramp value. Ramp steps never wait.
- With N <= 1 everywhere, the G-code is the host provider's.

## Limitations

- No smoothing with a single extruder multi-material printer: the wipe tower resets the shared heater there.
- When the host provider is explicitly selected in the layer temperature group, `print_temperature_smooth_change` is ignored without a message.
- Supports, skirt, brim and the wipe tower have no region: they keep the current setpoint, as with the host provider.
- Several instances at different temperatures on one layer switch the setpoint, without waiting, at each change of instance, as with the host provider.

## Differences with the legacy slicer

- The legacy slicer kept the layer of a change at the previous value, wrote N intermediate values and reached the target on the next layer, with a floating-point step and truncation. Here the change layer takes the first step and the target is reached on the N-th layer; N = 1 no longer smooths.
- The legacy ramp was per tool; here it is per object instance and tool, and modifier overrides do not restart it.
- The legacy first layer override was delayed by the ramp; here the first layer of an instance applies its temperature at once.
- In the current legacy slicer, the ramp only advanced with an active wipe tower; this plugin follows the original intent.

## Tests

The package gate is separate from the host's core suite. It runs the pure C++
ramp checks, the Python G-code reader/launcher checks, and the real host exports:

```
cmake --build .build --config Debug --target nematx_smooth_temperature smooth_temperature_tests
ctest --test-dir .build -C Debug -R smooth_temperature --output-on-failure
```

If the host needs dependency DLLs outside its executable/library directories,
configure `SMOOTH_TEMPERATURE_RUNTIME_PATH` with those directories (a CMake list).
The host executable and library paths come from the repository's existing
`SUPERSLICER_HOST_EXECUTABLE` and `SUPERSLICER_HOST_LIBRARY` settings.

### Coverage and cost

| Rule | Check | What must agree |
| --- | --- | --- |
| Ramp arithmetic | C++ kernel | Exact sequences, nearest rounding in both directions, small differences, monotonicity, no overshoot, exact finish, stable target |
| Restart / cancel | Kernel + shared main export | Current printed value is the restart point; N=0 also cancels at an unchanged target |
| Numerical limits | Kernel | Representative very large N positions and saturated counters, without a billion-layer loop |
| Setting / selection | Catalogue saved during first export + host comparison | Default 0, preferred provider, explicit host selection ignores smoothing |
| N=0/1 compatibility | One single-object range scene, two exports | Both immediate lengths in one scene; all executable G-code lines equal the host, not only temperatures |
| Height ranges / first layer | Shared main scene | Exact per-layer sequence, immediate first layer, N read beside the general temperature |
| Partial modifier | Common scene | Both the immediate override and the correct ramp value during actual extrusions |
| Covering modifier | Common scene | Smooth entry and exit, not an immediate partial override |
| Objects / real copies | Common scene + sequential scene | Independent temperatures and first layers; 3MF copies share one mesh and retain distinct copy ids |
| Synthetic paths | Common scene with a small skirt | A real regionless extrusion exists and adds no heater request or ramp step |
| Two tools / standby | Small base scene + shared main scene | Pause on absent-tool layers, immediate standby, next ramp value on return; identical wait locations |
| SEMM bypass | Base + smoothed SEMM scenes | Complete executable G-code equal, with both materials really used |
| Oracle failure visibility | Python unit tests | Missing temperatures/layers, ambiguous objects, malformed labels and partial exports fail |
| Timeout / process cleanup | Python unit tests | Failed commands retain logs; an expired budget launches nothing; a timed-out child's descendant is stopped |

The boxes are 4 × 4 mm, with only five to ten 0.3 mm layers as needed by each
sequence, and no infill or solid layers requested. Only the partial modifier needs two perimeters; only the synthetic
case needs a skirt. Placement is explicit, so the gate disables the unrelated
arranger and uses one host worker for these tiny scenes. It does not depend on
parallel CTest execution, a Release host or cached G-code. Arithmetic variations
stay in the cheap kernel instead of repeating slicer launches.

There are seven exports and seven host launches: saving the catalogue shares the
first export, and independent assertions reuse the main scene within that run.
The original gate used fifteen exports plus three catalogue-only launches.
Native SuperSlicer metadata avoids profile conversion, and a 3MF project-version
marker prevents the importer from splitting copies into separate objects.

The byte-for-byte compatibility scene deliberately has no modifier: repeated
exports of the host's own multi-object modifier scene can reorder regional
paths and their repeated heater commands, even with unchanged inputs. The main
scene checks the actual extrusion temperatures of both immediate and smoothed
partial regions instead; it does not mistake path-order variation for a ramp
regression. The strict full-command comparison remains on the range scene.

The E2E runner shares a 45-second wall-clock budget, including setup, exports and
parsing. Commands have a 12-second individual deadline (18 for the shared large
scene), always capped by the remaining total budget; a timeout is a visible
failure, not a skip or a retry. CTest safety limits are 2 seconds for the kernel,
3 for the reader/launcher and 50 for the E2E gate. These are limits, not claimed
performance measurements; the complete package must also pass below one minute
on the reference Debug machine.

Each invocation keeps an isolated datadir, all model/config/G-code/log files and
`timings.json` under `.build/test-results/smooth-temperature/run-*`. The report
separates host command durations from fixture creation and parsing. Runs do not
overwrite each other's inputs or reuse previous exports.

### Embedded-provider qualification

The normal gate uses the staged standalone package. To qualify a host rebuilt
with the embedded module, run the same assertions without installing that package:

```
python -B plugins/smooth_temperature/tests/test_smooth_temperature.py --host-executable "C:/path/to/nematxslicer_console.exe" --runtime-path "C:/path/to/deps/bin" --builtin
```

This second packaging qualification is explicit; it does not double the cost of
every normal CTest run. The known host firmware-priority diagnostic is kept in
the logs and excluded only by its existing specific message. Other host errors,
unknown options and disabled plugins fail the gate. Tool-temperature checks use
Marlin while the separate NematX firmware-selection problem remains unresolved.

Reference measurements on Windows with the Debug host (compilation excluded):
the complete three-test package passed in **35.62, 34.59 and 31.79 seconds** on
three consecutive serial runs. The original E2E-only suite took 96.96, 93.07 and
97.93 seconds. The separate embedded-provider E2E qualification took 24.39
seconds. These machine-specific measurements are not portable timing guarantees
or percentages of instrumented code coverage.
