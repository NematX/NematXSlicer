# NematX firmware

The **nematx-firmware** package writes and reads G-code in the NematX dialect. It replaces the legacy `gcode_flavor = nematx` setting; there is no conversion from that setting.

It contains two plugins:

- **gcode.firmware.nematx**: the G-code firmware. Select it with the printer setting `gcode_firmware_plugin = gcode.firmware.nematx`.
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

At the end of the file, two more metadata lines are written: `; total layers count = <n>` and `; has_support = 0|1`.

## Settings

The package defines these printer settings:

- **extruder_axis** (per extruder): axis letter of the extruder. Empty: `A` for the first extruder, `B` for the second. A letter must be free (not F, G, I, J, K, M, N, P, R, S, T, X, Y or Z) and different for each extruder.
- **disable_fan_from_unused_extruder**: switch off the fan of the extruder left by a tool change.
- **gcode_line_number**: number every non-blank line, comments included (`N10`, `N20`...).
- **gcode_no_comment**: remove all comments, including the preview markers. The preview then shows the moves without their extrusion types.

The metadata lines at the end of the file keep their comment form and have no line number, as in the legacy slicer.

The export is rejected when the printer has more than two extruders (the NematX codes only address two tools) or uses firmware retraction.

## Known differences with the legacy slicer

- The extruder axis is reset (`SET_POSITION`) where the common firmware session resets it, not only at tool changes. A tool change writes nothing: the next move uses the new axis letter, and each axis keeps its own position.
- `gcode_allow_negative_e` is not supported yet: it needs the firmware to decide when the axis is reset and to what value.
- Arcs (`G2`/`G3` with `I`/`J`) are encoded, but not verified end to end: the host currently fits no arcs on perimeters, whatever the firmware (checked with Marlin 2 and `arc_fitting = emit_center`).
- Corrected legacy defects: the startup no longer comments out the next line, waits no longer write a duplicated comment, a tool change no longer writes its reset twice, and the reader takes `G90`/`G91` inside a move into account. The plater JSON starts without a stray comma (`plater:{,"object_center"...` was invalid JSON), and a cut after the last outline point no longer leaves a trailing comma.

## Build and tests

The firmware derives from the slicer's public SDK classes. The plugin CMake adds the SDK directory (`src/libslic3r/Api/plugin` of `SUPERSLICER_SOURCE_DIR`), which needs the Boost headers: pass `CMAKE_PREFIX_PATH` pointing to the slicer dependencies when configuring.

    cmake --build .build --config Debug --target nematx_firmware nematx_firmware_syntax_tests
    .build/plugins/nematx_firmware/Debug/nematx_firmware_syntax_tests.exe

The syntax tests (axis letters, output filter, reader translation, metadata text) need no host. The end-to-end checks export models through the host executable and check the generated G-code:

    python plugins/nematx_firmware/tests/test_nematx_firmware.py --host-dir "C:/path/to/build/src/Debug" --host-executable "C:/path/to/build/src/Debug/Slic3r_console.exe" --runtime-path "C:/path/to/deps/usr/local/bin" --build-dir .build

`--runtime-path` adds directories to `PATH` for the host's DLL dependencies. Add `--legacy-executable <legacy nematxslicer_console.exe>` to also compare the command words and the object metadata with a legacy `gcode_flavor = nematx` export of the same cube.

The checks cover relative and absolute extrusion, two extruders (axes, heater and fan codes, fan of the unused extruder), line numbers without comments, the object metadata (a cube, and two instances of a 300-tooth gear whose outlines need several lines), the end metadata, the rejected configurations (firmware retraction, three extruders, invalid axis letter), and the absence of reader diagnostics.

Notes for a Debug host: the datadir must be a native absolute path (the script passes one), and seam visibility is disabled because it is very slow in Debug. The command line does not resize per-extruder settings to the extruder count as the GUI does, so the two-extruder case doubles them from the host's saved defaults.
