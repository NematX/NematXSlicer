# Support interface perimeters

The `nematx-support-interface-perimeters` package wraps dense support contacts and
intermediate interfaces in contour loops before filling the remaining area with
their original pattern. Contours include the holes of each interface island.

`support_material_interface_perimeters` is an object-scoped print option:

- `0` (default) retains the ordinary support paths exactly.
- A positive integer requests that many available contour levels.
- `infill_overlap` of the object's default region controls the overlap with the
  innermost contour, limited to half the effective flow spacing.

The bed layer keeps its existing single contour and first-layer flow. Passes
printed as sparse support keep their existing paths. Removable contact loops
remain a separate setting: their footprint is removed before these contours are
generated. Narrow areas stop when another contour does not fit; no gap fill is
added. Contact patterns remain configurable, while intermediate interfaces keep
their rectilinear pattern. Flow, tool selection and height modulation belong to
the existing support pipeline.

## Providers and selection

The package supplies four ordinary providers at the native execution priorities:

| Provider | Exclusive group | Priority |
|---|---|---|
| `nematx.support.bottom_contact` | `support_toolpath_bottom_contact` | 90 |
| `nematx.support.bottom_interface` | `support_toolpath_bottom_interface` | 100 |
| `nematx.support.top_contact` | `support_toolpath_top_contact` | 190 |
| `nematx.support.top_interface` | `support_toolpath_top_interface` | 200 |

Their group rank is 50, preferred to legacy rank 100. Automatic dense-role
selection must use group ranking; family preferences are needed only for sparse
paths. Explicit per-object legacy selections still take precedence and do not
apply the perimeter option. Disabling the package restores the active legacy
fallback. The package does not replace the sparse provider or support families.

The host needs the optional dense-fill callback in `LegacySupportPaths.hpp`.
No public C API or ABI extension is required. The standalone library compiles the
shared support pipeline sources against the C API, without copying them; the
built-in module reuses the host's existing pipeline. Installed standalone
providers take precedence over the embedded copy.

## Checks

Build `nematx_support_interface_perimeters` and
`support_interface_geometry_tests`. Run CTest with label `support-interface`.
The host must contain the embedded module for the delivery-comparison E2E gate.
The gate keeps its profiles, STL files, G-code, logs and domain report under the
build's `temp` directory, in a new timestamped directory for each run. It never
overwrites a previous result.

The analytic SDK tests cover contour levels, hole offsets, split islands,
termination, inherited pattern properties and failure propagation. The E2E gate
checks the declared settings catalogue, varies its domains linearly, compares
zero with legacy, checks sparse geometry, reloads saved settings and compares
standalone with embedded delivery. The focused host support tests cover the
callback boundary, selection precedence, first-layer and bridge-flow handling.

The export gate explicitly disables the unrelated `nematx_second_layer_flow`
provider. Its current implementation accesses `region(0)` even on auxiliary
support sources with no regions, before checking the neutral 100% ratio. This
fails with the legacy support paths too. The isolation is recorded in each
gate report and activation file; compatibility with that provider is not claimed.
The native `--export-only` check uses the same declared isolation.

The raft probe also requests three raft layers at zero and two interface
perimeters. The current host grid pipeline does not emit a raft contact below
the model in this fixture; the report records `not_emitted_by_host`, not a
successful raft-contact check. Raft-contact coverage is therefore not claimed.
This package does not add raft generation.
