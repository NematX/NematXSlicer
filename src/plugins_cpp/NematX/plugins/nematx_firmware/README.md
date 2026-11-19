# NematX firmware

The **nematx-firmware** package writes and reads G-code in the NematX dialect. It takes over the legacy `gcode_flavor = nematx` value.

It contains two plugins:

- **nematx**: the G-code firmware. Select it with the printer setting `gcode_flavor = nematx`.
- **gcode.reader.nematx**: the reader. The host uses it for the preview and statistics; the firmware uses it to follow the machine state through custom G-code scripts. It also recognizes NematX files opened on their own.

## Dialect

| Function | NematX G-code |
| --- | --- |
| Startup | `G90` and `G71` (metric units); no `M82`/`M83` |
| Extruder axes | one letter per extruder (`A`, `B`), no `E` axis and no `T` tool change |
| Moves | `G1` (also for travels); arcs `G2`/`G3` with `I`/`J` |
| Relative extrusion | `G90`/`G91` inside each move: `G1 G90 X10 Y5 G91 A0.3 F1200` |
| Axis position | `A[SET_POSITION POS=0]` |
| Extruder heaters | `M104=215` (first), `M124=215` (second); wait: bare `M109` / `M129` |
| Bed | `M140=60`; off: `M140=0` then `M141`; wait: bare `M190` |
| Fans | `M106=<v>` (first), `M126=<v>` (second); 0..255, or 0..100 with `fan_percentage`; off: `=0` |
| Dwell | `G4 X<seconds>` |
| Subprogram call | `L <program>`, for example in `toolchange_gcode`; the reader assumes it changes no tracked state |
| Acceleration | comment only: `;M204 S<value>` |
| Not supported | chamber heater and pressure advance (written as an "unsupported" comment); machine limits, speed ratio (`M220`), planner flush, extruder current, firmware retraction |

The firmware writes no per-command comments; the preview markers (`;TYPE:`, `;LAYER_CHANGE`...) are the same as for the other firmwares.

## Metadata for the NematX controller

When objects are labelled (`gcode_label_objects` is not disabled), the file header is followed by one JSON comment line per object instance and one for the whole plater, before the start G-code:

    ; object:{"name":"cube id:0 copy 0","id":"0","object_id":0,"copy":0,"object_center":[100.000,100.000,0.000],"boundingbox_center":[100.000,100.000,2.500],"boundingbox_size":[5.000,5.000,5.000],"scale":[1.000,1.000,1.000],"outline":[[102.500,102.500],...]}
    ; plater:{"object_center":[100.000,100.000,0.000],"boundingbox_center":[100.000,100.000,2.500],"boundingbox_size":[5.000,5.000,5.000],"outline":[[97.500,97.500],...]}

- `name` is the name used by the `; printing object` labels; `id` and `object_id` number the instances of the print in order, `copy` is the instance index inside its object.
- `object_center` is the centroid of the outline; the outline is the simplified projection of the instance on the plater. The bounding box covers the model parts of the instance, `scale` is its scale factor per axis.
- The plater outline is the convex hull of the object outlines, counter-clockwise; its bounding box covers all objects.
- The controller reads lines of at most 4096 characters. A long outline continues on new comment lines starting with `; `: the JSON is the concatenation of the lines without that prefix. An object line is cut once it exceeds 3000 characters, a plater line once it exceeds 4000.
- These lines go through the output settings like the G-code: they are numbered with `gcode_line_number` and removed with `gcode_no_comment`.

The controller also reads `; total layers count = <n>` and `; has_support = 0|1`. The host computes them for every firmware, and the exporter writes them with the other statistics after the G-code.

## Settings

The package defines these printer settings:

- **extruder_axis** (per extruder): axis letter of the extruder. Empty: `A` for the first extruder, `B` for the second. A letter must be free (not F, G, I, J, K, M, N, P, R, S, T, X, Y or Z) and different for each extruder.
- **disable_fan_from_unused_extruder**: switch off the fan of the extruder left by a tool change.
- **gcode_line_number**: number every non-blank line, comments included (`N10`, `N20`...).
- **gcode_no_comment**: remove all comments, including the preview markers. The preview then shows the moves without their extrusion types.

The lines written by the exporter around the G-code (header, statistics, configuration) keep their comment form and have no line number.

The provider also declares three historical profile settings, all with disabled default `0`:

- **extruder_pressure_factor**: nonnegative float per extruder, in seconds. The model uses the filament offset `C = K × model speed`, not filament speed.
- **split_extrusion_acceleration**: boolean per extruder, for phase splitting and annotations independently of pressure correction, including `K = 0`.
- **filament_compressibility_factor**: nonnegative float in the filament preset, with one value per extruder. Retained only for historical profile import/export; inactive, with no movement effect and no time-delay unit.

These keys are defined by the NematX provider, not the other firmwares. The session copies K and F09 for each logical tool at export setup. All K values must be finite and nonnegative. With every K zero and F09 disabled there is no pressure Reader, engine, movement hint or new script restriction; compressibility alone remains inert. The definitions request G-code invalidation. The slicing variation catalogue and same-Print invalidation matrix are separate qualification work.

## Pressure and phase splitting

One export owns one ordered pressure engine and a separate NematX Reader client.
The engine receives the complete nominal command stream before comment removal
and line numbering. It changes only eligible linear printing moves inside layers;
each layer closes at zero pressure offset. Startup and end scripts are read for
state but remain outside the corrected zone. Private movement hints are removed
before reading and never belong to the output protocol.

The legacy dominant-axis speed model and angular junction rule are retained.
The model is approximate, not a controller timing simulation. Its acceleration
comments (`;M204 S...`) are assumptions, not executed acceleration commands.
Machine limits apply for `emit_to_gcode`, `time_estimate_only` and `limits`; `ignore`
does not read them. Nonpositive axis limits retain the legacy 100000 fallback.
F09 emits `;NX_PRESSURE v=1` phase annotations with explicit units, not new F
commands. `gcode_no_comment` removes those annotations normally.

Both absolute and relative extrusion use `C = K × model speed`: K=0.02 s at
100 mm/s means a 2 mm offset, not the 0.08 mm produced by the old absolute
formula when filament speed was 4 mm/s. There is no automatic profile conversion.
The separate corrected E state rejoins nominal E at closed boundaries. It does
not change the producer's nominal volume, script variables or retraction debt.

An eligible Reader arc (`G2`/`G3`), volumetric extrusion, relative XYZ, inches or
unproved required state rejects the export; the existing file writer does not
publish its temporary output on failure. NematX already encodes an arc whose
sum of absolute X/Y endpoint deltas is at most ten XYZ output steps as G1,
independently of pressure.
Such an actually linear encoded move is treated as linear, not as a new pressure
approximation. Neutral tools and disabled pressure preserve the existing behavior.

`; PRESSURE_FACTOR=...` overrides K globally for configured participating tools;
an empty value restores their configured K. It never activates a neutral tool.
Invalid recognized directives fail even in the preamble when the engine exists.
Invalid acceleration comments retain the previous model acceleration and log a
warning. Other nonfatal Reader, horizon and precision-merge issues use the existing
application journal with a tool and source line; they do not open a GUI popup.

The NematX pressure-advance encoder is only an unsupported comment, so its setting
does not cause a false double-compensation refusal. Disable any pressure advance
hidden in controller configuration or proprietary macros yourself; it is not
observable through this Reader. No new compressibility physics is implemented.

The preview reads actual corrected E, not reconstructed nominal material. A
negative E delta during XY deceleration may therefore be classified as a
travel/retraction and change displayed width, flow or retraction statistics.
Producer accounting and viewer classification are distinct; no F06 correction
is included here.

In the focused 200 mm example with nominal filament 2 mm, K=0.02 s,
100 mm/s and 100 mm/s², emitted phase deltas are 2.5, 1 and -1.5 mm.
The NematX Reader ends at 2 mm; the viewer classifies the last XY move as
`Travel` and counts the positive 3.5 mm as consumed filament, retaining a
1.5 mm retraction debt. Its volume is therefore not the nominal 2 mm volume.

The export is rejected when the printer has more than two extruders (the NematX codes only address two tools) or uses firmware retraction.

## Known differences with the legacy slicer

- The extruder axis is reset (`SET_POSITION`) where the common firmware session resets it, not only at tool changes. A tool change writes nothing: the next move uses the new axis letter, and each axis keeps its own position.
- `gcode_allow_negative_e` is not supported yet: it needs the firmware to decide when the axis is reset and to what value.
- Scripts followed by generated moves must leave E in the mode selected by `use_relative_e_distances`. The existing base follows script mode changes, but the NematX encoder keeps its configured inline mode. A script that switches to absolute E while the encoder is configured relative can therefore produce wrong amounts even with pressure disabled. This F03 limitation is not corrected by pressure; closed pressure cycles rejoin the nominal encoded stream, not an independently reconstructed intended volume.
- Arcs (`G2`/`G3` with `I`/`J`) have focused PrintingPlan export/Reader checks, including the existing tiny-arc G1 conversion and rejection by active pressure. This does not qualify slicing arc fitting: the host currently fits no arcs on perimeters, whatever the firmware (checked with Marlin 2 and `arc_fitting = emit_center`).
- Corrected legacy defects: the startup no longer comments out the next line, waits no longer write a duplicated comment, a tool change no longer writes its reset twice, and the reader takes `G90`/`G91` inside a move into account. The plater JSON starts without a stray comma (`plater:{,"object_center"...` was invalid JSON), and a cut after the last outline point no longer leaves a trailing comma.

## Build and tests

The firmware derives from the slicer's public SDK classes. The plugin CMake adds the SDK directory (`src/libslic3r/Api/plugin` of `SUPERSLICER_SOURCE_DIR`), which needs the Boost headers: pass `CMAKE_PREFIX_PATH` pointing to the slicer dependencies when configuring.

    cmake --build .build --config Debug --target nematx_firmware nematx_firmware_syntax_tests
    .build/plugins/nematx_firmware/Debug/nematx_firmware_syntax_tests.exe

The syntax tests (axis letters, output filter, reader translation, metadata text and actual pressure option definitions) need no host. The end-to-end checks export models through the host executable and check the generated G-code:

    python plugins/nematx_firmware/tests/test_nematx_firmware.py --host-dir "C:/path/to/build/src/Debug" --host-executable "C:/path/to/build/src/Debug/Slic3r_console.exe" --runtime-path "C:/path/to/deps/usr/local/bin" --build-dir .build

`--runtime-path` adds directories to `PATH` for the host's DLL dependencies. Add `--legacy-executable <legacy nematxslicer_console.exe>` to also compare the command words and the object metadata with a legacy `gcode_flavor = nematx` export of the same cube.

Append `NematXPressureSettings` to select only the small profile persistence check: disabled defaults and distinct values for two extruders/filaments through save and reload/save in fresh host processes. It uses the staged real plugin and does not slice models or construct gears. The separate pressure export gate below covers the runtime catalogue and slicing matrix.

The focused native integration gate requires both real external providers and a
host test binary containing `[nematx-pressure]`; missing packages or tests fail:

    python plugins/nematx_firmware/tests/check_host_pressure.py --host-tests ../NematXSlicer_v3/build/tests/plugins/Debug/plugins_tests.exe --build build

On Windows, `plugins_tests.exe` exports the host ABI itself and intentionally
does not load `Slic3r.dll`. For this native gate, configure the existing package
target with `SUPERSLICER_HOST_LIBRARY` pointing to that executable's
`plugins_tests.lib`, then rebuild `nematx_firmware`. Restore the normal host
import library and rebuild for CLI/package delivery. This changes only which
real host supplies the same C ABI; neither firmware nor Reader is mocked.

### Small pressure export gate (Windows)

After building the Debug CLI and `plugins_tests` in the dedicated host checkout,
and staging the normal firmware package, run:

    ctest --test-dir build -C Debug -R "^pressure_exports$" --output-on-failure

The equivalent command, with explicit paths, is:

    python plugins/nematx_firmware/tests/check_pressure_exports.py --host ../NematXSlicer_v3/build/src/Debug/nematxslicer_console.exe --host-tests ../NematXSlicer_v3/build/tests/plugins/Debug/plugins_tests.exe --build build --native-fixture build/test-fixtures/pressure-native-host

The native fixture is a build prerequisite. Its absence fails, rather than
skipping tests. To prepare it with the existing package target:

1. Configure `SUPERSLICER_HOST_LIBRARY` to the dedicated host's
   import library:

       cmake -S . -B build -DSUPERSLICER_HOST_LIBRARY:FILEPATH="C:/local/NematXSlicer_repository/NematXSlicer_v3/build/tests/plugins/Debug/plugins_tests.lib"

   Replace the absolute dedicated-host path if the checkout is elsewhere.
2. Run `python build/CodexBuildWindows.py build --target nematx_firmware nematx_infill_gridvarspeed nematx_corner_stretch --config Debug -- /m:8 /v:minimal`.
3. Create `build/test-fixtures/pressure-native-host/test-data`, and copy the
    generated `build/test-data/nematx_firmware` directory into it.
   Then copy `build/test-data/infill_gridvarspeed/plugins/nematx-infill-gridvarspeed`
   and `build/test-data/corner_stretch/plugins/nematx-corner-stretch` into
   `build/test-fixtures/pressure-native-host/test-data/nematx_firmware/plugins`.
4. Configure `SUPERSLICER_HOST_LIBRARY` back to the normal host's
   import library, then rebuild the normal package with the same wrapper command:

       cmake -S . -B build -DSUPERSLICER_HOST_LIBRARY:FILEPATH="C:/local/NematXSlicer_repository/NematXSlicer_v3/build/src/Debug/Slic3r.lib"

Do not deliver the test-host DLL as an application package. The wrapper above
is a local, unversioned tool and is mandatory for builds on this machine. On
other hosts without that tool, the standard equivalent is
`cmake --build build --config Debug --target nematx_firmware nematx_infill_gridvarspeed nematx_corner_stretch`.
Repeat all four preparation steps after every production change under
`plugins/pressure`, `plugins/nematx_firmware`, `plugins/infill_gridvarspeed` or
`src/CornerStretch.cpp`. Matching ABI and package/host
versions are necessary checks, not proof that two DLLs used identical sources.

For all three packages the runner checks actual PE imports: the normal DLL
imports `Slic3r.dll`; the native fixture imports `plugins_tests.exe` and not
`Slic3r.dll`. It also
checks the staged DLL and its associated manifest against a generated ZIP,
compares the native fixture's ABI and package/host versions, and checks both activated
provider IDs and the dedicated embedded-source CMake setting. The two CLI
datadirs differ only by the installed NematX package. Startup logs must prove
the normal DLL load, or its absence with the embedded providers active; both
must actually emit pressure annotations. Marlin is a neutral control, not a
claim of pressure support on another firmware.

One sequential 90-second deadline includes preparation, all five CLI exports and all
seventeen native cases (four qualification cases plus the thirteen focused
pressure cases). Each run starts fresh under
`build/temp/pressure-exports/run-*/`, retaining the JSON report, effective
profiles, logs, STL files, G-code and Reader comparison measurements. A timeout
is a failure and retains partial diagnostics; it does not select a smaller
suite. The native child has its own shorter deadline so it exits before the
outer runner. Output verification shares the same deadline. CTest allows
105 seconds, including a 15-second margin for reporting a failure.
Run this test alone. Its `RUN_SERIAL` property prevents overlap with other tests
in the same CTest invocation; it does not isolate other checkouts or processes
on the machine and does not make a measured timeout successful.
The CLI/feature case runs first in lexical order. GridVarSpeed and CornerStretch
publish definitions before native selector/configuration freezing, then are
disabled outside their scoped feature pairs. The ordinary wipe provider is
temporarily enabled for these pairs to match the CLI profile and is restored
afterwards. The thirteen focused pressure cases keep their original execution
profiles; the normal focused runner does not enable feature fixtures.

The native cases reload a 4 × 4 × 0.4 mm ASCII STL and use real
`Print::apply`, `process`, PrintingPlan export and the NematX Reader. They cover:

- Four coupled K/F09 modes in absolute and relative extrusion, K=0/0.02 s,
  inert compressibility=0/3.5, fresh-Print comparisons after each changed
  pressure setting, return to neutral, and repeated active/neutral exports.
  A separate active compressibility export in each E mode must match the active
  zero-compressibility output without replacing the fresh/repeated exports.
  Slicing timestamps must remain valid for the pressure-only edits.
- A typed catalogue checked against the loaded provider's defined and used
  keys, including rejection of an artificially added uncovered key. Firmware
  inputs unrelated to pressure have individually named, typed exemptions.
- Linear variations of all nine machine limits, `ignore/time_estimate_only/limits`
  policies and coarse XYZ/E precision. A shared nominal baseline is used only
  within the current run; precision variants get their own quantized baseline.
  X/Y/E speed/acceleration and overall printing acceleration bounds must be exercised numerically;
  Z remains a travel-only exemption in this STL. `limits` and
  `time_estimate_only` have the same pressure-model effect; `emit_to_gcode` is
  not qualified here. Rewritten XYZ/A/B words must obey their configured quantum
  and decimal precision, with actual fractional examples.
  Volumetric extrusion remains deliberately rejected, as proved by the focused
  failure tests. A/B and SEMM already have focused coverage; a real two-tool STL
  additionally proves a neutral B tool, retraction and generated wipe markers.
  Withdrawals in wipe blocks and E-only restorations outside them must remain
  identical with pressure off/on and must not carry pressure annotations.
- Targeted native DLL GridVarSpeed and CornerStretch pairs with pressure off/on,
  actual activation and saved settings, as well as a cooling/slowdown pair.
  Matching nominal extrusion moves must show an actual slowdown while retaining
  the configured 3 mm/s minimum feedrate.
  GridVarSpeed uses top fill because the two-layer model has solid surfaces;
  its 30/60 mm/s normal/reduced-flow speeds must actually reach the Reader.
  These six feature/cooling exports use real STL loading, slicing, Print replay
  and the Reader in the same native process. Their saved settings must match the
  fresh embedded-CLI base profile except for the explicit feature/pressure
  overrides. This does not globally qualify these features or claim their
  embedded integration; the four mandatory NematX CLI exports still qualify
  normal-package versus embedded pressure firmware.
- Ordered endpoints and intermediate points at output precision, no reversal
  along a nominal segment, signed filament sums per tool/layer and at proven
  travel/retraction boundaries, and producer volume/retraction variables.
  An extrusion word rounded to zero does not prove a travel boundary. Positive
  viewer volume is never substituted for signed extrusion accounting.

The runner requires exactly 54 named qualification exports: 15 catalogue files,
26 Print replay files, two multi-tool files, five CLI files and six native
feature/cooling files. Each retained
file must have the expected metadata and annotation presence; its size, phase
count and SHA-256 are recorded by case. The thirteen focused native cases also
perform their existing internal exports. Reader measurements, native phase
timings and CLI plugin-library startup times are retained separately.
All 24 named Reader comparison reports are required, with finite measurements,
at least two checked cycle closures per two-layer mono-tool comparison and four
for two tools. Cycle E errors may not exceed the report's E quantum plus 1e-6 mm;
geometric deviations may not exceed two XYZ quanta. The total number of cycles
is measured rather than fixed. Raw durations, assertion counts
and run results belong in `build/temp/pressure-exports/run-*/report.json` and its
logs. A timeout remains a failure. These automated checks do not qualify the GUI
or packages on other operating systems and do not constitute final acceptance
of F08/F09.

The checks cover relative and absolute extrusion, two extruders (axes, heater and fan codes, fan of the unused extruder), line numbers without comments, the object metadata (a cube, and two instances of a 300-tooth gear whose outlines need several lines), the end metadata, the rejected configurations (firmware retraction, three extruders, invalid axis letter), and the absence of reader diagnostics.

Notes for a Debug host: the datadir must be a native absolute path (the script passes one), and seam visibility is disabled because it is very slow in Debug. The command line does not resize per-extruder settings to the extruder count as the GUI does, so the two-extruder case doubles them from the host's saved defaults.
