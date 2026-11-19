# Internal seam attractor

`nematx-seam` adds the **NematX internal attractor** provider to the regional
seam selector. Its default preference is -50, before Cost (0); explicit selections
of Cost or another seam policy are still respected.

The provider asks the host Cost service for ordinary placement. It changes only
loops whose actual geometric segments are all internal perimeters. External,
mixed, support, skirt and other technical loops keep the host answer for the
same approach position. Changing an internal seam can naturally change later
approach positions; this does not freeze every later travel-dependent decision.

## Volume and constraints

Create **Seam → Seam (internal perimeter) cylinder attractor**, or convert an
existing volume to **Seam Position (internal perimeters, between min & max z)**.
The cyan volume is not printed. The provider uses the XY center of its
transformed cross-section at each layer's print height. Empty cross-sections
are inactive. Rotation, scaling, mirroring and source-instance translations use
the same object frame as the host constraint helper. A converted non-cylinder
uses its sliced bounding-box center, not a nearest triangle or a 3D centroid.

Among several active volumes, the nearest admissible seam wins; exact ties keep
model-volume order, then loop-segment order. Attractors are not distance-limited
and their radius is not a weight. Painted blockers exclude closed boundaries;
painted enforcers and inside-volume enforcers take precedence over attraction
to free space. If no admissible rounded point exists, the host fallback remains
unchanged, including its all-blocked policy. The analytic curve helper handles
both straight segments and real arcs; integer-rounded candidates are rechecked.

Projects write `SeamPositionInternalCenterZ`; old `SeamPositionInsideCenter`
projects load as the same type. Numeric volume identity 7 is unchanged. The
public C/Python/C++ alias remains available, with Volume API 2.1. Ordinary sphere
and unrestricted-cylinder attractors are not implemented by this package.

## Build and verification

Use the repository's normal host SDK/package configuration. The standalone DLL
compiles the existing `SeamPlacementConstraints.cpp` against the public C API;
it does **not** copy the host Cost engine. Cost is a required active service.
No additional configuration settings are defined. The three used settings are
the host's `seam_angle_cost`, `seam_travel_cost` and `seam_visibility`.

```
cmake --build build --config Debug --target nematx_seam
ctest --test-dir build -C Debug -R '^seam_' --output-on-failure
```

On the reference Windows workspace, compilation uses the local build wrapper
instead of direct `cmake --build`; configure `SEAM_RUNTIME_PATH` for dependency
DLL directories. The export runner has one 45-second budget, shorter per-command
deadlines, process-tree cleanup and no retries. Its artifacts go to
`build/test-results/seam/run-*` (`builtin-*` with `--builtin`): a passed export drops
its datadir, a passed run is removed and a failed one is kept, with at most two runs
per variant. CTest caps it at 50 seconds and the reader at 3.
On Windows, the runner passes `--sw-renderer` so the console wrapper does not
probe a windowed hardware OpenGL context before a headless export. Slicing and
all G-code assertions are unchanged.

The host also builds `nematx_seam_host_tests` from this package. It tests actual
initialized sessions, line/arc geometry, source transforms and instances, role
inheritance, constraints, ownership, cancellation and concurrent determinism.
Run the host CTest entry `nematx_seam_host` alongside the package gate. This is a
targeted suite, not the complete core test set.

The export matrix is linear: each declared cost setting varies over its domain
on an independent tiny object. The enabled visibility case uses one layer: the
host still traces its full 30,000-sample visibility model for that external loop,
but repeating it on five layers would dominate the gate without adding an
attractor rule. Five-layer boxes exercise canonical/legacy
names, true copies, a rotated instance, inactive heights, explicit host selection
and full-command no-volume parity. The reader checks actual extrusion starts;
the host's `;SEAM` comments alone describe only external perimeters.

After rebuilding an embedded host, qualify the identical export assertions
separately with `test_exports.py --host-executable ... --build-dir ... --builtin`.
This does not double every normal package run. All unknown options, disabled
plugins and host errors fail; only the pre-existing, exact firmware-priority
diagnostic is exempted and remains in the logs.

Reference Windows x64 Debug measurements, compilation excluded: three serial
runs of the complete gate (native checks plus package tests) passed in **30.86,
29.70 and 30.75 seconds**. Package CTest alone took 28.73, 28.05 and 29.11 seconds.
The separate embedded export qualification took 42.80 seconds. The native test
performs 131 checks, including fresh-versus-replayed complete ordered geometry;
the host seam/volume regression selection passed 4,355 assertions in 104 cases.
These are machine-specific timings, not portable performance guarantees or code
coverage percentages. GUI targets are compiled; interactive visual verification
is separate from these automated checks.
