# Retraction over top surfaces

`nematx_top_surface_retraction` restores the legacy print setting
`enforce_retract_top_surface` (boolean, default **true**). With
`only_retract_when_crossing_perimeters` enabled, a travel over a published top
surface remains eligible for retraction even when it stays within an island.
With the setting off, ordinary interior suppression applies there too.

This is not `retract_lift_top`, and does not retract between contiguous printed
segments. The host still applies `retract_before_travel`, retraction length and
explicit disable modifiers. Tool changes, explicit requests and configured layer
changes keep their own rules. With the perimeter filter disabled, this policy
does nothing. Deactivating the plugin removes the exception on the next slice.

## Shared host contract

The plugin runs at layer-edit priority -95, after transition creation (-100) and
before retraction planning (-90). It publishes `RetractionCrossingPolicy` on each
printing root through the public SDK. The host classifies the selected built-in
travel path, including detours, with the existing perimeter engine and final
`LayerRegionIsland` surfaces. No second router or E-axis request is added.
The shared engine also detects a path wholly inside a top area, not only a
crossing of that area's boundary.

Layer-edit API **1.2** is required. The policy does not change the eight existing
scope flags or the C modifier layout. Endpoints are resolved through the plan's
instance catalog; absent/different provenance and unknown travel providers keep
ordinary retraction conservatively. Support surfaces are not treated as object
tops. Classification precedes wipe generation, as does the common decision.

## Qualification

The host's `top_surface_retraction_policy` gate checks a top strip between
endpoints outside the strip, a non-top interior, on/off, thresholds, and explicit
enforce/disable modifiers. It deliberately leaves the obsolete LayerRegion fill
container empty.

`tests/test_exports.py` runs a linear STL-to-G-code matrix with production
defaults, preserving logs and reports under `build/temp/f21`. It checks top-only
effects, deactivation, thresholds, zero length, avoidance, wipe, filter-off parity
and settings save/reload. The standalone CTest `top_surface_retraction_package`
uses the same matrix with the built DLL installed in private data directories;
the host's `top_surface_retraction_export` exercises the embedded module.
The production `top_surface_retraction_replay` gate reapplies on/off/on in the
same Print and compares it with fresh slicing, using true copies, supports and
two tools; it also retains their G-code exports.

No GUI qualification is implied by these headless gates.
