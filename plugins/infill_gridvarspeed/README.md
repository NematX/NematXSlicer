# Grid with varying speed and Z

External package `nematx-infill-gridvarspeed`. All its sources, copied geometry,
tests, maintenance tooling and metadata live here. Uses only the public host SDK;
no private host includes, nested pattern calls, host modification or host rebuild.

## Behavior / accepted migration choices

- Stable pattern id **`gridvarspeed`**, offered in `fill_pattern` and
  `top_fill_pattern` only, as in 2.7.
- Two ordinary **connected** rectilinear passes, in fixed order. Each keeps the
  recipe's full density (unlike standard Grid, which divides density between
  passes). No odd/even rotation; effective angle = recipe/bridge angle + π/2 +
  pass angle (0 or π/2).
- Preserves the experimental legacy multiplier **0.776284** for flow and width.
  Final full-fill normalization is disabled only for this pattern. This is NOT
  a guarantee of nominal-density volume, especially for sparse infill.
- `fill_gridvarspeed_width_diff`: regional Percent, default **50%**, range 0–100.
  Normal flow factor 1, crossing factor r, transition factor (1+r)/2. Crossing
  half-zone h = trunc((flow.spacing / 2) × r). **0% is a zero-length crossing
  zone, not zero extrusion**; both 0% and 100% publish uniform passes. Small r
  rounded to h=0 follows the same rule.
- Preserves the historical asymmetric first crossing and near-start accumulator
  branches. Later well-spaced crossings have symmetric transitions. This is a
  port, not a redesign of the deposition model.
- Speed is adjusted **after** DefaultSpeed, by 1/f, without changing fan,
  acceleration or pressure advance. Requests are bounded by existing
  print/filament/volumetric and first-layer limits, preserving an upstream
  minimum: `max(v, min(v/f, cap))`. For an uncapped 30 mm/s profile, factors
  1 / 0.75 / 0.5 yield 30 / 40 / 60 mm/s. Unlike the legacy slicer, compensation
  does not amplify speed beyond these caps. It does not reduce a baseline already
  exceeding a cap (for example due to an upstream minimum or provider defect).
  Actual motion may be slower.
- **Autospeed retains the legacy algebra:** the resolved baseline may already
  depend on reduced flow; this pattern still applies 1/f. No heuristic neighbor matching
  or promise of constant volumetric flow in autospeed mode.
- `fill_gridvarspeed_zhop`: regional FloatOrPercent, default **0 mm**; percentage
  base = extrusion height (including combined infill thickness). Rejects negative
  values or a non-positive lowered altitude. Offsets above half that height are
  clamped per extrusion, with one warning per generation run. This deliberate
  difference from the legacy tooltip-only recommendation accommodates variable
  layer heights without aborting an export.
  With `infill_first`, **P0 is lowered** and P1 stays nominal; otherwise P0 stays
  nominal and **P1 is raised**. This follows the historical tooltip, deliberately
  correcting the old code which lowered P1 instead. Flow/height remain unchanged.
- Ordered, non-reversible root; passes sortable/reversible; each split chain can
  reverse only as a whole. No independently reversed fragments.

### Deliberate robustness differences

Valid two-point paths/tails are retained (legacy `>2` could discard them).
Pre-cleanup preserves real corners: the old unconditional spacing-based point
removal could shortcut outside concave material. Only forward, quasi-collinear
points are removed now, so connected boundary walks retain their geometry.
Shared-vertex intersections are deduplicated; collinear overlaps remain ignored.
Zero-vector extension at a shared vertex is handled, near-start extension is
bounded to the source segment, and overtaken close crossings are consumed once.
Integer-rounding micro-edges are snapped across flow boundaries together before
publication: the Debug host rejects sub-epsilon ArcPolyline edges. Exact source
endpoints and continuous chains are retained; tiny cleanup changes are within
the slicer's coordinate tolerance. These are not claims of bitwise legacy parity.

## Pipeline / integration preconditions

One DLL registers three plugins in a joint activation group:

| Id | Step | Priority |
| --- | --- | ---: |
| `gridvarspeed` | `INFILL_PATTERN` | 30 |
| `nematx_gridvarspeed_recipe` | `INFILL_SURFACE_RECIPE_MODIFIER` | 1000 |
| `nematx_gridvarspeed_speed` | `STEP_LAYER_EXTRUSION_EDIT` | 2 |

The recipe modifier must run **after every modifier that changes the pattern**.
Priority 1000 is a supported-composition convention, not a universal guarantee
against arbitrary third-party plugins. The pattern rejects FILL_EXACTLY if a
later modifier leaves an incompatible recipe. DefaultSpeed at priority 0 must be
the selected speed provider; incompatible providers are diagnosed only when gridvarspeed
leaves are encountered. Corner stretch (priority 5) is unaffected.

Private versioned metadata stores the **factor**, not an absolute flow target.
Speed is resolved field by field from the PrintingExtrusion root. Process/travel
geometry inheriting metadata is excluded. A direct `applied` marker rejects a
second edit of the same leaf: replay must start from a fresh ordering-plan clone.
The host-side external replay fixture loads the real speed editor, changes a
speed and restores fresh ordering clones on the same Print. It verifies that
compensation does not accumulate. This is not a GUI event-handling test;
interactive GUI invalidation/re-slice remains a separate manual check.

`infill_first` is declared as a read-side dependency of the pattern so it
invalidates infill geometry, not only its historical later ordering step.
The native option's used-key declaration uses NONE container/preset (the SDK
does not label this native option as a plugin-created REGION option). Settings
are still read from the effective source region configuration.

Z offsets are inherited **by replacement**, not addition. Each pass publishes an
explicit offset (including zero). Composition with an ancestor's independent Z
transformation is unsupported without a composition test. A raised pass needs
adequate travel clearance and may affect wipe or subsequent low extrusions:
**exports are not proof of collision-free machine motion**. Do not print raised
infill without checking travels, first layer, variable heights and the firmware.
The current v3 gap-fill post-plugin is only a placeholder; revisit compatibility
if it begins generating additional geometry.

## Build / tests (existing matching host)

Configure the root project normally, with matching SDK/import library/version
and `SUPERSLICER_PLUGIN_REPOSITORY_DIR` empty. Use a build/datadir **inside this
repository**, not the host checkout. For example, with an already configured
Windows Debug tree:

```powershell
cmake --build .build --config Debug --target nematx_infill_gridvarspeed infill_gridvarspeed_tests infill_gridvarspeed_sdk_tests
ctest --test-dir .build -C Debug -R infill_gridvarspeed_ --output-on-failure
python plugins/infill_gridvarspeed/tests/smoke_export.py --host <host-console-executable> --build .build
```

SDK tests need the matching host library and its runtime dependencies available
to the system loader. Pure kernel tests need no
host runtime. The smoke script uses an isolated staged datadir and writes a
closed 30×30×1 mm STL, profiles, logs and G-code under the build tree. It checks
activation errors, speeds, E/mm ratios, caps, pass Z and unwanted retractions in
contiguous modulated paths. `--check-existing` rechecks saved exports without
running the host. It does not validate all collision scenarios or GUI replay.

Package output: `.build/package/<platform>/Debug/nematx-infill-gridvarspeed/`
and its ZIP in the parent directory. ABI metadata is extracted from the binary;
the configured slicer version must match the host used to build the package.

### Test coverage and known limits

Kernel tests cover crossing factors and geometry, degenerate intersections,
two-point tails, corner-preserving cleanup, speed compensation and Z bounds.
SDK tests cover rectangles, holes, concavities and offset-split islands at several
densities and angles, continuity, containment, clone/split properties and raw
filament command exclusion. Parser tests cover oriented fixtures and XYZE modes.
Import tests reproduce the four copied files from both LF and CRLF sources and
reject modified inputs.

End-to-end checks distinguish sparse from top fill, exercise uniform endpoint
ratios, print-speed caps, autospeed, pass order and Z limiting. These checks do
not establish bitwise legacy parity, physical collision safety, GUI replay,
multiple tools/objects, wipe, independent ancestor Z transforms, or compatibility
with arbitrary recipe/speed providers. First-layer compensation requires the
public SDK `DefaultSpeedLimits.hpp` contract: the default provider publishes the
absolute ceiling evaluated against the original role/autospeed base. The plugin
consumes that metadata without reevaluating percentages on a capped speed. A
host lacking that publication is rejected when modulated first-layer infill is
encountered; there is no heuristic fallback. An upstream explicit speed override
without resolved limit metadata is likewise unsupported on the first layer.
Regional projection must preserve
distinct ceilings even when final speeds coincide after a physical cap or
minimum. Zero infill-limit fallback is resolved by the host. The plugin cannot
repair an already excessive upstream baseline.

For hosts providing the optional `[external-grid-replay]` fixture in their
native `plugins_tests` executable, run:

```powershell
python plugins/infill_gridvarspeed/tests/check_host_replay.py --host-tests <host-plugins-tests-executable> --build .build
```

This fixture isolates the actual DLL speed editor on marked paths. A 200% limit
with a 40 mm/s first-layer minimum distinguishes correct ceiling reuse from
percentage reapplication; changing the role speed 30 → 40 → 30 mm/s must yield
60 → 80 → 60 mm/s without a double-application error. It is opt-in and does not
require the external package for ordinary host tests. Its native Print fixture
lives in the host because standalone Print construction is not part of the SDK.

## Vendored source / maintenance

`src/rectilinear/` is a locally namespaced subset of v3 commit
`6ea300e8b5`: raster, contour graph and ordinary
single-sweep traversal. No support connectors, monotonic solver, other patterns,
Boost/Eigen dependencies or runtime calls into a host rectilinear implementation.
Copyright/license notices are retained (AGPL-3.0-or-later).

`tools/import_rectilinear.py` reproduces the copied files from **hash-pinned**
upstream inputs after CRLF-to-LF normalization; it reads upstream and writes only
the local copies (or an explicit `--output` directory). It is not a
build step. `RectilinearFills.hpp` is a manually reduced adapter. Host algorithm
updates are not imported automatically; incompatible public SDK updates still
require rebuilding and reviewing ABI compatibility.

The modulation kernel adapts 2.7 `FillRectilinear.cpp:3425–3748` at commit
`aaf4f1ddf092249b1aab4e3c29f59771d31da2ba`. Geometry expectations are derived
from that source; comparisons require a legacy executable built from matching
sources, not an arbitrary older release.
