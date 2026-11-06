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
