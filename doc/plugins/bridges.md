# Prepared bridge surfaces

`surface.bridge_preparation` chooses the canonical angle and axial anchors
for external bottom bridges after solid shells. Its required recipe member
`infill.recipe.bridge_preparation` requests separate parallel rows; both
belong to the `bridge_preparation` activation group. An enabled bridge angle,
including zero, takes precedence over automatic detection.

Preparation preserves the full non-overlapping regional/tool partition and
surface metadata. Top expansion, cleanup and DenseInfill retain marked cores
and anchors. The final infill recipe is checked after all modifiers.

The regional `bridged_infill_uniform_margin` adds a uniform margin before the
existing axial `bridged_infill_margin`. Its default is `25%`. Both accept mm or a
percentage of the effective external-perimeter width, not the bridge thread or
nozzle diameter. Zero disables only that margin; zero uniform margin restores the
axial-only geometry. The new setting appears under **Infill / Advanced**, beside
the anchor controls, and invalidates `STEP_PERIMETER` like the existing margin.
With zero infill density, their combined axial reach is capped by the existing
perimeter-shell limit, with the uniform margin applied first.


The first solid cover over sparse/void becomes an internal bridge;
subsequent solids on solid stay ordinary. A top cover can be that first cover.

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
lower density it uses the current normal fill direction +90 degrees.


## Experimental volume correction

`bridge.finalize_volume` is optional and disabled by default. It shares
neither required activation nor an algorithm router with preparation.
The native `bridge_volume` exclusive group owns provider selection; one
active member requires no selector. The regional checkbox
`bridge_volume_compensation_enabled` keeps nominal flow when disabled,
without skipping strand and provenance checks.

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

- ID 16: `bridge_preparation.surface.v1`, surface identity and reconstruction policy;
- ID 17: `bridge_preparation.volume.v1`, each piece's nominal and effective values,
  confidence, and input/path signatures;
- ID 18: `bridge_preparation.atomic_strand.v1`, collective strand orientation;
- ID 19: `support.pipeline_state.v1`, support completion.

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

The C API domains are `slic3r_def.h` 1.3, `slic3r_data_tree.h` 5.2 and
`slic3r_step_finalize_slicing.h` 2.0. C++, C and generated Python bindings use the
same payload layout; callback fields use named typedefs so the Python generator
preserves their positions.
The storage-owned `surface_collection_set_type()` reclassifies a scratch surface
without resetting its geometry, angle or properties; borrowed/live collections
cannot use it. C++ and Python expose it only on `StoredSurfaceCollection`.
