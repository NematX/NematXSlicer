# NematX hole compensation by diameter

Package `nematx-hole-size-curve`; plugin `nematx_hole_size_curve`.

The region setting `hole_size_compensations_curve` is a curve that gives, for a hole diameter (mm), an XY compensation (mm) applied to the convex holes of that diameter. A negative value enlarges the hole, a positive value shrinks it. The curve can be linear or square (the value of the point at or before the diameter). An empty curve, the default, changes nothing.

## Behavior

- Only convex holes (bores) are changed, as for the host's `hole_size_compensation`. The diameter is approximated by twice the mean distance of the hole points to its centre.
- The curve adds to the other hole compensations:
  1. `slicing.hole_size_compensation` (host, priority -1100) applies `hole_size_compensation`, reduced by `hole_size_threshold`;
  2. this plugin (priority -1050) adds the curve value, not reduced by the threshold;
  3. `slicing.xy_size_compensation` (host, priority -1000) adds `xy_inner_size_compensation`.
- Regions: each region forming part of a hole wall moves its part with its own curve. The rules come from the SDK function `compensate_convex_holes()` (`Api/plugin/cpp/HoleCompensation.hpp`), shared with the host plugin.

## Differences with the legacy slicer

- The diameter is measured on the hole already moved by `hole_size_compensation`, so it can differ by twice that value; the legacy slicer measured it before any compensation. The curve and the hole compensation are applied as successive offsets instead of one.
- The setting is per region instead of per object.
- The plugin declares the legacy graph editor settings (title, labels, ranges, linear and square types). Editing the curve in the GUI has not been tested; profiles set it directly.

## Tests

The end-to-end checks export two plates with a 3 mm and a 10 mm round hole through the host executable, and compare the width of the external perimeter loop of each hole with an export without curve: linear and square curves, the sum with `hole_size_compensation` and `xy_inner_size_compensation`, and an empty curve.

    cmake --build .build --config Debug --target nematx_hole_size_curve
    python plugins/hole_size_curve/tests/test_hole_size_curve.py --host-executable "C:/path/to/build/src/Debug/Slic3r_console.exe" --runtime-path "C:/path/to/deps/usr/local/bin"

`--runtime-path` adds directories to `PATH` for the host's DLL dependencies. The regional rules themselves are tested with the host (`plugins_tests [hole-size-compensation]`).
