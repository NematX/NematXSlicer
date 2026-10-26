# NematX corner stretch

Package: `nematx-corner-stretch`; provider: `nematx_corner_stretch`.

The plugin edits ordered perimeter loops at `STEP_LAYER_EXTRUSION_EDIT`, priority
5: after the base speed and first/second-layer flow adjustments, before default
acceleration and fan calculation. It uses only the public plugin SDK.

Enable `stretch_corners` in a print/region profile. The twelve historical
`stretch_corners*` keys, defaults and the two legacy distance/deviation aliases
are registered by the plugin. Distances accept mm or percent of the perimeter
width; deviations accept mm or percent of the angle-adjusted stretch distance.
Entering/exiting influences and inner-perimeter influence are percentages.

Only the original corner selects its regional settings. The deformation may
cross a modifier boundary, including into a region where stretching is disabled.
An outside/ambiguous regional lookup leaves the corner intact. No geometry is
clipped to modifier masks. The seam and loop endpoints remain fixed.

Close corners share their straight branch proportionally. Both the windows and
corresponding displacement components shrink; the calculation is non-iterative.
Collinear, equivalent leaf splits are transparent. Changes of effective process
state, unknown property scope, arcs or events stop branch propagation.

The original volume of each corrected corner is preserved by changing mm³/mm;
speed changes by the reciprocal ratio (historical maximum multiplier x10), and
the advertised width changes by the flow ratio. This preserves nominal time
except at the speed cap; it does not model acceleration or enforce an additional
XY machine-speed cap. Invalid active flow/width/speed rejects export.

The wipe created upstream is deliberately left on the **undeformed** geometry.
Existing arcs are retained. New rounded transitions use circular fillets with
nominal straight extensions when the legs differ; impossible fillets fall back
to straight segments. Nonplanar segments and vase loops are not stretched.
With `arc_fitting = disabled`, rounded transitions are tessellated with a 0.01 mm
chord-error tolerance, and flow is corrected using the serialized chord lengths.
Native-arc firmware compatibility and arbitrary custom timing providers require
separate validation. There is no GUI panel in this migration.

## Focused verification

```powershell
cmake --build .build --config RelWithDebInfo --target nematx_corner_stretch corner_stretch_tests --parallel 2
ctest --test-dir .build -C RelWithDebInfo -R '^corner_stretch_geometry$' --output-on-failure
python tests/corner_stretch/test_export.py --host C:/path/to/Slic3r_console.exe
```

These commands do not rebuild the host. The packaging target writes an isolated
datadir `.build/test-data/corner_stretch` and a platform/configuration-specific
ZIP. It does not install the package into the host's shared plugin repository.

The export smoke test checks disabled, straight, native-arc and tessellated modes on the small
cube fixture, deposited E conservation (0.2% tolerance for serialization),
observable deformation and native arc output. This is not yet validation of the
entire migration test matrix (regional modifiers, wipe, holes and Z scenarios).
