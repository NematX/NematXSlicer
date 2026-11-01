# NematX fan anticipation and kickstart

Package `nematx-fan-speedup`; plugin `nematx_fan_speedup`.

A fan needs time to reach its speed. This plugin changes **when** the fan requests of the print take effect: it moves the rises of the fan speed earlier, and it can start a rise with a burst at 100%. It never changes a print speed, adds a wait or touches the geometry: the cuts it makes keep the path, the arcs, the Z and the extruded volume.

The three settings are defined by the host (printer settings, firmware category); the plugin defines none:

| Setting | Meaning |
| --- | --- |
| `fan_speedup_time` | A rise is moved earlier by up to this many seconds of modeled print time (its absolute value, at least 0.01 s when not zero). 0: off. |
| `fan_speedup_overhangs` | The anticipation only applies when the path that asks for the rise is an overhang perimeter. It has no effect alone. |
| `fan_kickstart` | A rise to a speed below 100% starts with 100% for `fan_kickstart * rise / 100` seconds, then the target. 0: off. |

The host needs a plugin of the `fan_temporal_plugin` group to apply `fan_speedup_time` and `fan_kickstart`: a non-zero value makes the export fail while this package is not active. With both at 0 the plugin walks nothing and changes nothing.

## Behavior

- The plugin runs at the temporal fan priority (40), after every fan policy has written its requests on the paths. It moves the instants at which requests take effect and never changes a print speed; the minimum fan speed and the fan offset of the firmware are applied later, to the 100% of a kick as to any request.
- **Anticipation.** Only rises are moved. A rise at instant t starts at t - delay, but never before the start of the tool group (so never before the start of its layer), the end of a barrier, the end of a kick, or the end of an earlier period whose target is at least as high. The lower requests inside the moved span are erased: their paths now belong to the rise. When the room is shorter than the delay the anticipation is partial; nothing is added to compensate.
- **Kick.** The rise to a target below 100% is preceded by 100% for `fan_kickstart * rise / 100` seconds. Without anticipation only rises of more than 10 points are kicked; with anticipation any rise is. A fall or a stop during a kick ends it there; a rise during a kick extends it by the time the extra rise needs.
- **Every mode.** The legacy slicer anticipated everything and so does this plugin: a moved request keeps the `{percent, mode}` pair of the request it comes from, an Override or a plugin mode included, and a kick is written in the mode of the request it serves. This differs from the common rule of the fan policies, where an Override is never modified by a later stage: the paths between the advanced instant and the original transition take the value of the request that was advanced, whatever they asked for (see Anticipation).
- **Barriers.** A custom G-code, a special command (tool change...), a heater event, a fan request on an empty leaf (a service event) and a path whose E is replaced by a raw `EXTRUSION` command are never crossed or edited. The first two and the raw `EXTRUSION` path also make the fan target unknown afterwards, so the first request after them is neither anticipated nor kicked; a service fan event makes it known. Wipes and approaches (extrusion axis) keep their real duration and are not cut; a rise falling inside one starts at its beginning.
- **Tool groups.** Nothing is anticipated across a tool group, and a kick never crosses the end of its tool group. A kick still running at the end of the last path is closed by an empty leaf with a direct fan request, appended to the "after" events of the tool group.
- **Entry state.** The target in force at the start of a tool group is the last target of the previous tool group of the plan, whatever its layer, if it prints with the same extruder; it is unknown for the first group, after a change of extruder and after an opaque event (custom G-code, special command) found at any level of the plan before it. A rise from an unknown state is neither anticipated nor kicked.
- **Placement.** A fan needs about 100 ms of precision, no more. A change is written at the start of a path or at a cut inside a cuttable path (a gap shares the position of the path that follows it), and a path is never cut in a piece shorter than the placement tolerance, 0.1 s (`k_placement_tolerance` in `FanSpeedupTree.hpp`, a constant that can become a setting). The piece is counted from the previous cut of the path, if any, and up to its end. A change that would leave a shorter piece goes to the nearest boundary in time: the previous cut or the start of that path, or the start of the next one. If a cut is impossible for another reason, the change goes to a boundary the same way: the host cannot split a path that contains a vertical move (a lift has no XY length, and a cut before it fails as well), so such a path is never cut and its changes go to the nearest boundary. A barrier is not a position where a change can be written: a change (ordinary or the return of a kick) that would land in the last 0.1 s of a path followed by a barrier or by the end of the tool group stays 0.1 s before it, or goes to the start of the path if that is not possible. Changes keep their order: a change cannot precede a position already written (for instance after the end of a kick was moved forward), it takes that position and is resolved as a collision. When two changes end at the same position, the later one wins; when the start of a kick and its own end (the return, or the transition that stops it) meet, as happens with a kick shorter than the tolerance or one that ends exactly where a path starts after a gap, the end moves to the next position: a cut 0.1 s later in the same path if the rest of the path lasts at least 0.1 s more, otherwise the start of the next path. If that position is behind a barrier or the end of the tool group, the kick is dropped and the transition keeps its place at the target. The durations of the kicks stay those of the planner, in seconds; only their placement is rounded.
- **Diagnostic.** One warning is reported per export when a kick is dropped, or shortened by a barrier beyond the placement tolerance: a barrier or a path that cannot be cut left no room for its return, or a kick shorter than the placement tolerance ended a tool group. Nothing else is reported: a return kept within 0.1 s before a barrier, and any change or kick moved to a boundary, even a far one (a path that cannot be cut or that has a vertical move), are silent.

## The time model

The timeline of a tool group follows the order of the G-code writer: its "before" events, then each extrusion root (a node's events at its entry, before its children), then its "after" events. A path lasts its 3D length (arcs included) over its inherited speed, as in the host time estimator; accelerations, retraction moves, scripts and waits are ignored. Between two paths separated by at least `SCALED_EPSILON` a synthetic travel at `travel_speed` is placed; it belongs to the path that follows, whose request is written before it. The first gap of a tool group is ignored: its starting position is unknown.

The delay is therefore the one of this model, not a guarantee of real time on the machine.

A time offset inside a path becomes a distance along its XY path (curved for an arc) in scaled units, found with the cumulative 3D lengths, so the cut is exact for arcs and Z changes. A purely vertical segment has no XY length: a path that contains one is never cut and the change goes to a boundary (see Placement). A cut perimeter loop keeps its perimeter property on the collection created by the cut only, so the seam marker is written once.

## Differences with the legacy slicer

- The legacy `FanMover` rewrote G-code lines; here the requests of the print tree are moved and the firmware writes the commands, so only the planned timing is shared. It stopped only at custom G-codes; here every barrier above stops it.
- The legacy slicer did not know the state at the start of a layer; here the previous tool group of the same extruder gives it, and the anticipation still never crosses the start of the layer.
- The M106 of scripts inserted after this step (layer, height and between-object G-code) are not seen, as in the legacy slicer for its own state.
- A kick that would cross a barrier is shortened to the last path that fits, or dropped (with the warning).

## Tests

The kernel tests cover the planning on a timeline of seconds. The SDK tests run the tree adapter on stored entities against the host library: a travel cut by an anticipated rise and the copy of the pair, an arc with Z changes at an inherited speed cut at its exact length, the placement rule (a cut, a change less than 0.1 s after the start or before the end of a path, an ordinary collision, the end of a kick moved past its start including a kick that starts at a cut and the case of a gap followed by a path, changes that keep their order after an end moved into the next path, a kick dropped before a barrier and at the end of the group with the flag, a return and an ordinary change kept 0.1 s before a barrier, a path with a vertical move, with a cut requested before, in and after the move, that is not cut), a rise inside a path that carries a request followed by the next paths, one-shot events never repeated, the barriers, the gap that belongs to the next path, and the restore event.

The end-to-end checks export generated models through the host executable and read the fan commands with the time model above (times are compared with the 0.1 s placement tolerance):

- a rise to 80% is moved one second earlier, or to the start of its layer when that comes first; with a 100 s delay, each layer either keeps its commands (no rise) or starts with the rise, together with its first path, or has none (the fan already ran at that speed);
- the kick lasts `fan_kickstart * rise / 100` seconds (rises of 60 and 30 points) and follows the anticipation;
- rises of 10 points or less are not kicked without anticipation, a rise of 11 is, and any rise is kicked with it;
- a kick longer than the layer is restored at the end of the layer;
- with two extruders (80% and 40%, raised layer by layer by the first-layer program of the host, so that every tool group is one request): the groups that follow a tool change, and the first group, are neither anticipated nor kicked; the tool that goes on from one layer to the next has its rise kicked from the start of its group; a kick of 60 s that is still running at the end of its group is closed in the window between the last path and the next tool change, with the target of the group;
- with `fan_speedup_overhangs`, only the rise to the overhang speed is moved;
- the extruded volume and the modeled time are kept, and every point added by a cut lies on the move it cuts, in order, with the same feed rate, length and extruded length (kicks, anticipation and overhang exports);
- the cuts of the perimeter loops do not repeat the `;SEAM:` marker (the SDK tests check that the perimeter property stays on the collection of the cut only);
- with both functions off the G-code is identical to the one without the package;
- the export is refused when `fan_speedup_time` or `fan_kickstart` is set and the plugin is not active (a copy of the host datadir with `nematx_fan_speedup = 0`, since a host can embed the plugin and activate it by default);
- two exports give the same G-code.

```
cmake --build .build --config Debug --target nematx_fan_speedup fan_speedup_kernel_tests fan_speedup_sdk_tests
ctest --test-dir .build -C Debug -R fan_speedup
python plugins/fan_speedup/tests/test_fan_speedup.py --host-executable "C:/path/to/build/src/Debug/Slic3r_console.exe" --runtime-path "C:/path/to/deps/usr/local/bin"
```

`--runtime-path` adds directories to `PATH` for the host's DLL dependencies. The refusal check uses a copy of the host datadir staged by the build (`.build/test-data/host`) with the plugin deactivated.
