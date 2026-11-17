# Default straight travel

`layer_extrusion_edit.travel.default` is the simple travel provider of the
layer-extrusion-edit step. It belongs to the `layer_extrusion_edit.travel`
exclusive group with priority -50 and depends on
`layer_extrusion_edit.transition_scope.default` and
`layer_extrusion_edit.entry_state.default`.

The provider chooses a two-dimensional path for each ordered gap between
compact extrusion scopes. `ScopeTravelConnector` resolves the endpoints, snaps
nearby seams, inserts the travel phase, preserves Z offsets and sets the
materialization flags. The provider reads no print setting and adds no option.

## Path selection

The provider starts from the direct two-point path. Ordinary in-layer gaps at
the layer `print_z` are checked against the NoTravel volumes of the plan:

- Each NoTravel volume of every real plan instance is sliced at `print_z`,
  transformed by the object matrix, the volume matrix and the instance shift,
  then unioned in platter coordinates. Holes remain free.
- A connected union component containing either endpoint is ignored for that
  gap. The other components remain obstacles.
- If the direct path does not enter an obstacle, it is kept exactly.
- Otherwise the provider calls
  `clipper_detour_polyline_expolygons(..., CLIPPER_DETOUR_SHORTEST)` once along
  the original obstacle rings. The candidate is accepted only when its complete
  polyline avoids every non-exempt interior. A rejected candidate keeps the
  direct path.

Boundary contact and travel along an edge are allowed. Validation clips against
the obstacles inset by `SCALED_EPSILON`, the host's geometric tolerance rather
than a physical clearance, so features thinner than that tolerance are not
protected. Routing uses the union rings without an added margin.

`ScopeTravelConnector::prepare()` records the gaps after seam snapping, and
`apply()` calls the provider once per gap in order. A recorded gap is at least
`SCALED_EPSILON` long, so its endpoints differ and the detour primitive never
receives repeated consecutive points.

## Limits

- NoTravel avoidance is best-effort, not a strict barrier. An inadmissible
  candidate leaves the direct gap, which can still cross a NoTravel volume.
- Sections are two-dimensional at `print_z`, not the whole layer slab. A thin or
  inclined volume that misses that height is not avoided.
- The policy covers ordinary in-layer gaps at `print_z` only. Layer entries,
  gaps with another Z, lifts, wipes and scripts keep their paths; no vertical
  detour is added.
- An exempt component is crossed entirely, not only by a minimal entry or exit
  leg. Overlapping NoTravel volumes union into one component before exemption.
- There is no physical clearance. Enlarge the volume when nozzle clearance is
  required.
- One attempt is made. `CLIPPER_DETOUR_SHORTEST` chooses an arc locally on each
  ring, not a globally shortest path, and long admissible detours are accepted.
- NoTravel avoidance is independent of `avoid_crossing_perimeters`. The simple
  provider does not avoid printed perimeters, its detour can cross them, and no
  detour budget applies.
- Selecting another travel provider keeps that provider's paths unchanged.
  AdvancedTravel has its own policy and is not used here.

When the provider exempts an endpoint component or rejects a candidate, it
reports one warning per layer-edit execution with both counts. No per-gap
message is emitted.

The sections are rebuilt from the current plan instances on each run, so a model
edit needs no geometry cache invalidation. The host replays the plan on model
changes as it does for any other helper volume.

## Verification

`[no-travel-routing]` covers simple avoidance, holes, inactive height, endpoint
components, transformed instances, a rejected candidate, another provider,
independence from the perimeter setting, G-code publication and replay after
helper edits. `[plugins][layer-extrusion-edit]`, `[travel-zones]`,
`[zone-planner]` and `[clipper-detour]` keep the surrounding behavior checked.
