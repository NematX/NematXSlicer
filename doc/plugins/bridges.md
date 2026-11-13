# Prepared bridge surfaces

Bridge preparation targets external `BOTTOM | SOLID | BRIDGE` surfaces and the
first solid cover over internal sparse or void. A top cover can be that first
solid layer too. Later solids on solid remain ordinary. DenseInfill's
`INTERNAL | SPARSE | BRIDGE` is a separate sparse recipe, not a solid cover.

## Pipeline and configuration

The `bridge_preparation` activation group contains two built-in entry points:

- `surface.bridge_preparation`, surface generation priority 15, after SolidShells;
- `infill.recipe.bridge_preparation`, which requests disconnected parallel rows.

Fresh profiles also activate `bridge.finalize_surface_volume`, finalization
priority 20, after support generation and before ordering. The alternative
`bridge.finalize_volume` (also priority 20) is experimental and off by default.
Disabling it does not make the preparation activation group incomplete.
Preparation also depends
on the active `bridge_detector.default` service, which supplies automatic angles.

The existing `bridge_angle`, `bridged_infill_margin`, `bridge_precision`,
`bridge_type`, regional widths and tool assignments determine preparation.
Angle zero is a valid override; a disabled override invokes the bridge detector.
The native infill generator and the public bridge-flow adapter share one resolver.
The extrusion role selects the nozzle, while the `flow` bridge baseline uses the
normal solid-infill area.

The regional `bridged_infill_uniform_margin` adds a uniform margin before the
existing axial `bridged_infill_margin`. Its default is `25%`. Both accept mm or a
percentage of the effective external-perimeter width, not the bridge thread or
nozzle diameter. Zero disables only that margin; zero uniform margin restores the
axial-only geometry. The new setting appears under **Infill / Advanced**, beside
the anchor controls, and invalidates `STEP_PERIMETER` like the existing margin.
With zero infill density, their combined axial reach is capped by the existing
perimeter-shell limit, with the uniform margin applied first.

Internal bridges use neither external margin. `internal_bridge_min_width` uses
the normal solid-infill spacing as its percentage reference; components narrower
than that threshold remain ordinary solid. `internal_bridge_expansion` permits
end extension into this region's eligible internal infill, bounded by one lower
sparse pitch (normal sparse spacing divided by density). This surface-based reach
does not locate the phase of a particular sparse line and is not an exact contact
guarantee for nonuniform patterns. With void below, the reach is one solid spacing.
An enabled `bridge_angle`, including zero, is authoritative. An undetectable
internal angle uses lower sparse's configured layer direction +90 degrees,
including template, increment and alternating-layer settings; without a usable
lower density it uses the current normal fill direction +90 degrees. An
undetectable external angle uses the current normal fill direction itself and
reports a warning.

The recipe retains the native transport convention: width and spacing are scaled
coordinates, while height is rounded to the layer Z grid. For prepared bridges,
the generator restores the native circular dimensions on attributes that exactly
match that transport and its native area, before post-infill processing. It does
not infer a diameter from volume or rewrite a pattern's different physical model.

The providers belong to the native exclusive group `bridge_volume`. Its
project-scoped selector `bridge_volume_compensation` serializes provider IDs:
`bridge.finalize_surface_volume` (default preference 0) or
`bridge.finalize_volume` (experimental, preference 10). The existing group
infrastructure discovers active alternatives, builds the selector and dispatches
exactly one finalizer. With one active member it needs no selector and ignores a
stale stored choice, as for seams. There is no separate model enum or activation
guard in the bridge code.

The shared regional checkbox `bridge_volume_compensation_enabled` defaults to
true. Disabling it retains nominal flow without skipping direction, role and
provenance checks. Exact lower contact audits belong only to the experimental
provider. Both controls appear under **Width & Flow / Flow**. They invalidate
`STEP_FINALIZE_SLICING`: its Slicing checkpoint regenerates nominal infill and
support data. A Print checkpoint resumes at ordering and retains finalized paths.

Profile loading migrates `surface_density_v1` and `conservative_clearance_v1` to
their provider IDs; `disabled` becomes the regional checkbox set to false. Native
group resolution handles a stored provider that is no longer active.
If the singleton has no generated selector, loading an old project can report a
standard substitution for its unavailable choice; this does not block slicing.

## Geometry and physical model

Prepared destinations belong to one layer, island, physical LayerRegion and
resolved zero-based solid-infill tool. Uniform growth uses a round offset clipped
to ordinary internal receivers covered by linked lower slices, retaining only
components meeting the original core. Holes remain excluded and detached patches
are discarded; this is not a geodesic-distance calculation around obstacles.
Axial extension then grows this wider area along the bridge direction, stopping
at holes on each ray. Another region's receivers, protected top/bottom surfaces
and other bridge cores are not available anchors.
Both external and internal cores are identified before either family claims
receivers, so external margins cannot overwrite a neighboring internal core.

For external bridges, with prepared geometry `E`, anchors `A`, original core `B` and linked lower slices
`L`, preparation enforces `A` within `L` and reconstructs `B = E - L`. Complete
surface metadata is copied when surfaces are split. Top expansion and cleanup do
not consume prepared cores or their anchors. Compatibility splitting collects all
source groups before publishing, so converging region/tool destinations retain
both prepared bridges and ordinary infill.
Internal bridges use reconstruction rule 2: their entire prepared area is the
working core. Their unsupported material lies inside the lower model slices,
so `E - L` would incorrectly erase it. The top/internal position, shell metadata,
regional settings and solid-infill tool survive classification and publication.

Rectilinear and monotonic are the supported bridge patterns. The generator checks
the recipe after all modifiers. Finalization also checks actual strands: each is
straight, planar and monotone along the canonical angle. A row must traverse its
original core, or, with a positive uniform margin, traverse the uniformly expanded
core while staying inside its prepared surface. An isolated piece in an axial
anchor does not satisfy that rule. An incompatible recipe stops processing before
export. Finalization never stops slicing on a path: a refused row keeps its whole
infill group at nominal flow, a refused prepared surface or object keeps its own
infill, and one warning per object counts them. Broken internal invariants of the
finalizer also assert in debug builds. Preparation does not stop slicing either: an
island whose preparation fails keeps its unprepared surfaces, and one warning per
object counts these islands.

### Default: excess volume from surfaces

For a current area `S`, layer height `h` and nominal recipe volume `R*S*h`, the
extra equivalent empty height required below is `max(0, (R-1)*h)`. For a row,
`R*h` is its nominal cross section divided by its actual recipe pitch
(`spacing / density`), not the numeric value of `bridge_flow_ratio`.

Search downward through actual Z intervals. A lower sparse band of height `dz`
and density `rho` contributes `(1-rho)*dz`. Explicit void and air contribute the
whole interval. Each XY portion stops independently when enough room is found,
on a solid/perimeter, or at the finite platform boundary. Never credit void under
a stopping solid. The cap is `pitch*(h + credited_empty_height)`, bounded by the
incoming nominal section. For 160% deposition on a 0.2 mm layer, seek 0.12 mm;
a 0.1 mm lower band at 30% gives 0.07 mm, leaving 0.05 mm to seek farther down.

Use the lower surface's own region density, not the bridge's. Typed solids,
perimeter bands and explicit void override the configured sparse percentage;
DenseInfill's marked sparse-bridge recipe is 50%. Missing typed fill is unknown.
The lower extrusion paths, their phase and their angle are never inspected.
Splits occur only at changes of surface budget; equal adjacent results merge.

This is a mean volume estimate, not proof of clearance between individual sparse
lines. Nonuniform patterns use their configured average density. Auxiliary
support surface types identify a printing role but do not carry an exploitable
material-density payload here, so their affected areas remain explicitly unknown.
Unavailable support generation marks the requested areas unknown rather than
turning the request into material. Unknown portions retain nominal flow and warn.

### Experimental: actual extrusion clearance

When `bridge.finalize_volume` is selected, clearance is computed from actual lower extrusion footprints and their actual Z,
including auxiliary support and raft layers. A footprint is a capsule; arcs use
chords enlarged by their bounded chord error. Their vertical prisms conservatively
bound occupation. The highest intersecting prism limits each longitudinal interval;
model slices and support requests are not treated as deposited material.

For a nominal circular bridge of diameter `d`, compensation retains width `W=d`
and uses the flattened obround area

```
A(W,h) = W*h - (1 - pi/4)*h*h
```

The minimum printable height is the positive per-tool `min_layer_height`, capped
by nominal diameter and layer height. When the setting is zero, the layer height
is used instead. Bounded minorant ramps reduce sudden flow changes without raising
the local target. Unknown intervals remain strictly nominal; no ramp crosses them.
At a nominal fit, one ULP of the native float height is treated as roundoff against
the layer Z grid. Adjacent sections with identical deposited attributes can share
one path with their strictest cap. Split sections reuse the same stored endpoint;
a clipping gap larger than the host geometry epsilon is rejected, not joined.
Known sections shorter than the native splitting precision may be absorbed by an
adjacent section with no greater area or height. That neighbor's flow is unchanged;
only the tiny section loses material, and the strictest cap is retained. Unknown
sections and minima without a safe neighbor are not silently discarded.

- `Certified`: the section fits the modeled clearance, within numeric precision.
- `FloorLimited`: the printable floor exceeds the geometric cap; this is reported.
- `Unknown`: occupation or shape cannot be certified, so nominal attributes remain.
- `DensityEstimate`: fits the estimated surface volume budget, not a local gap.
- `DensityFloorLimited`: the printable floor exceeds that mean volume budget.

The experimental model may reduce flow excessively for an obstacle touching only one side of a
thread. It does not simulate pressure, sagging, adhesion or mechanical strength.

## Support completion and anchor diagnostics

`support.pipeline_state.v1` separates three facts: demand was computed, a matching
support family completed its surface calculation, and the required support
toolpath roles completed. A matching family can explicitly complete an empty
result. Merely having an active family, or an empty vector of paths, does not
establish material completion. Missing producers retain `Unavailable`. A known
empty result without producers requires support and raft to be disabled, with no
forced support layers, local request masks or existing support surfaces.

The finalization payload exposes read-only demand and material-completion flags,
and `get_requested_support_areas()` borrows the mask on the bridge's own layer
island. In the experimental model, a local request suppresses cantilever/missing-anchor diagnostics only on
the covered part of a deficient core. It never creates an occupied footprint.
Missing context produces a technical incomplete-compensation diagnostic, not a
claim that an unknown anchor is absent.

In the experimental model, for peripheral rows that do not cross the original core, lower slices still do
not prove support. Finalization measures gaps in actual nominal lower contact.
A continuously supported row needs no separate pair of end anchors. Otherwise,
each gap uses the normal anchor checks on its two sides. A row supported in the
middle with two free ends has two one-sided tails, not a wholly unanchored span.
Only each deficient gap's footprint participates in local request filtering.
Compensation and Unknown handling are unchanged for these rows.

## Properties, publication and ordering

All payloads are fixed-size POD values copied by the generic property containers:

- ID 17: `bridge_preparation.surface.v1`, surface identity and reconstruction policy;
- ID 18: `bridge_preparation.volume.v1`, each piece's nominal and effective values,
  confidence, and input/path signatures;
- ID 19: `bridge_preparation.atomic_strand.v1`, collective strand orientation;
- ID 20: `support.pipeline_state.v1`, support completion.

ID 16 remains reserved for `slic3r.layer.instance`, which identifies instance-owned
auxiliary layers independently of bridge metadata.

No property owns a polygon or retains a borrowed geometry pointer. Generated
infill refers to the final runtime Surface ID; the logical bridge key is separate.

The finalizer builds private infill copies bottom-up. The host
`replace_infill_buckets()` callback validates the whole batch before exchanging
content, flags and owned properties with nonthrowing swaps. Destinations must be
unique existing infill, ironing or gap-fill buckets of the current object. Donors
must be typed extrusion roots owned by that plugin's storage. Rejection changes no
destination; successful donors own the old content. Root identities remain stable.
Cancellation is checked before the first swap, not between committed swaps.

Each split strand is non-sortable. When reversible, it may reverse only as a whole;
its children retain the permissions needed for that collective reversal. Advanced,
XY-serpentine and default ordering are the built-in providers allowed for these
marked trees. Unmarked trees retain their existing provider choices.

Repeated finalization verifies signatures and effective attributes rather than
multiplying corrections. Changed inputs require nominal infill regeneration.
The finalization dispatcher also protects certificates against later plugins
changing or deleting paths, flow, supporting occupation or prepared surfaces.

The C API domains are `slic3r_def.h` 1.4, `slic3r_data_tree.h` 5.2 and
`slic3r_step_finalize_slicing.h` 2.0. C++, C and generated Python bindings use the
same payload layout; callback fields use named typedefs so the Python generator
preserves their positions.
The storage-owned `surface_collection_set_type()` reclassifies a scratch surface
without resetting its geometry, angle or properties; borrowed/live collections
cannot use it. C++ and Python expose it only on `StoredSurfaceCollection`.

## Verification

The `[bridge-preparation]` and `[bridge-volume]` tests cover geometry, native flow,
metadata, clearance, support availability and transactional publication. The
`[bridge-pipeline]` fixtures run real slicing and support generation, check the
emitted E against each planned bridge section, exercise all three built-in ordering
providers, and replay Print and Slicing checkpoints.
`[bridge-selector]` checks native exclusive dispatch, active alternatives,
singleton fallback, default preference and migration through real INI loading.
`[bridge-depth-budget]` slices 10x10 mm squares with 0.1 mm layers, a 0.4 mm
nozzle, 20% rectilinear sparse, two top/bottom layers and one perimeter. Four
layers have no bridge; five and six limit the first internal cover to 0.18 and
0.26 times its area. Seven and eight must retain the circular nominal section.
The matching external cases remove a central 10x10 mm part of a 20x10 mm mesh,
not internal infill: 0.1/0.2 mm gaps permit 0.2/0.3 times the bridge area, and
a 0.3 mm gap no longer limits the nominal section. Cases stop on both the
platform and a solid floor. The oracle checks local sections, actual surface
area, nonzero total volume and exported E, with anchor margins set to zero.

`[plugin-e2e][bridge-e2e]` is the fast settings gate. It reads the four bridge
entry points' and Ensuring's used/defined keys and checks an explicit variation
catalogue against them. The baseline enables the feature and uses each declared
setting's default.
Each case changes one setting, or a named coupled group such as tool/nozzle/height
limits; cases are never multiplied together. Enum values are enumerated, numeric
limits use representative values, and equivalent configurations run once. A new
declared key without a variation fails the gate instead of silently losing coverage.

Three tiny STL round trips cover a solid cube, a taller cube with internal sparse,
and a notched frame with a roof, opening and raised pad. Every case slices and exports, then checks bridge volume
and continuity against the actual final plan. Set `SLIC3R_E2E_REPORT` to a TSV path
for flushed per-case start/end records. The reusable catalogue and STL helpers are
in `tests/plugins/plugin_e2e_test_helpers.*`; the workflow rule is in `AGENT_TESTS.md`.
The `[plugin-e2e-catalog]` unit tests check linear growth, coupled groups,
deduplication, invalid changes and baseline preservation. Representative numeric
values do not claim exhaustive coverage of every possible value or interaction.

For a Windows Debug build, from `build/tests/plugins`:

```powershell
$env:SLIC3R_E2E_REPORT = Join-Path (Resolve-Path ../../temp) bridge-e2e-linear.tsv
./Debug/plugins_tests.exe '[plugin-e2e-catalog],[bridge-e2e]' --reporter compact -d yes
./Debug/plugins_tests.exe '[bridge-selector],[selector-snapshot]' --reporter compact -d yes
```

`[bridge-performance]` measures a fixed cube and two-pillar bridge with the feature
off and on. Set `SLIC3R_BRIDGE_PERFORMANCE_REPORT` to an output TSV path to record
wall time, path and segment counts, split pieces and unknown portions. These small
fixtures are not a large-model memory or mechanical qualification. Printed-part
tests are still needed to assess the conservative reduction near partial-width
obstacles, adhesion and the printable-height floor.
