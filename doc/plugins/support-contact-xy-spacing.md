# Support contact XY separation

`support_material_contact_xy_spacing` lets the support contacts come closer to
the object than `support_material_xy_spacing` (`gap_xy`), while the rest of the
support keeps `gap_xy`. It helps sharp corners, small areas and the top of steep
slopes, whose overhang is narrower than `gap_xy` and would otherwise lose its
support.

The option is a distance or a percent of the external perimeter width, like
`support_material_xy_spacing`, and it can be disabled. It is disabled by
default.

## Activation

`Common::SupportParams` reads the option once per run:

- disabled, or not smaller than `gap_xy` (including `gap_xy` = 0): nothing
  changes, `gap_xy_contact` equals `gap_xy` and the slabs are the same as without
  the option;
- enabled and smaller than `gap_xy`: contact mode, `gap_xy_contact` is the
  option value (`mingap`).

In contact mode, `support_material_contact_cone_growth` sets the lateral change
per support layer. It is a distance in millimeters or a percentage of the
interface nozzle diameter, **not** the external perimeter width. Its default is
67%, which keeps the 0.67-nozzle step of profiles without this setting.

A positive value sets `contact_cone_step` (at least one scaled coordinate unit).
`contact_cone_layers`, `ceil((gap_xy - mingap) / contact_cone_step)`, is the
number of steps from the contact gap back to `gap_xy`; the merge below does not
use it as a limit. A smaller step makes longer transitions. A larger step
shortens them but increases the overhang between consecutive layers. It is a
distance per support layer, not an angle or a distance per millimeter of Z.

Zero adds no transition cones: `contact_cone_step` and `contact_cone_layers` are
zero, but the contact itself still keeps `mingap`. It may then lack support
below. Outside contact mode the growth value has no effect.

In the settings window, the contact gap and cone growth share the row directly
below the general XY separation. The growth control is inactive when supports or
contacts are disabled, or the style is tree or organic.

## Geometry

The contacts are the top contacts, under the object, and the bottom contacts,
on the object. They are the only slabs cut directly with `mingap`: the object
layers of their Z window, the no-support zones and the bridge footprints are
grown by `mingap` instead of `gap_xy`. The overhang margin of the grid top
contacts is limited by `mingap` too.

Every other slab, interface or sparse, keeps its usual area cut with `gap_xy`.
`Common::merge_contact_cones()` then merges the cone of each contact into the
slabs, one slab at a time, downwards from a top contact and upwards from a
bottom contact:

```
cone = C
for each slab L away from C:
    cone = cone - step                      (inward offset)
    stop if cone is empty
    touching = islands of L that meet cone
    stop if cone - touching is empty        (the support covers it)
    printed = cone - (obstacles(L) + mingap) - contacts beside L
    L = L + printed
    cone = printed + touching
```

The next cone grows from the printed part and the islands it joined, so a cone
that meets the support follows that support until the support covers it.

```
          wall
  layer   object
   C+1    ####|##############   object above the contact gap
   C      ####|.=============   contact, mingap from the wall
   C-1    ####| ..~~~~~~~~~~~   normal (gap_xy) + cone C - 1 step
   C-2    ####|    ..~~~~~~~~   + cone C - 2 steps
   C-3    ####|<-gap_xy->::::   the cone lies inside the normal area
```

Each cone overhangs the next one by at most one step; the chosen growth must
remain printable for the material and layer height. A cone that does not reach
the normal support is kept until it vanishes.

## Families

- Grid, snug and grid merged: the cones join the base slabs before
  `carve_interfaces()`, so the part of a cone in the interface window becomes
  interface. Grid merged clips its sparse body to the dense footprint grown by
  `mingap` instead of `gap_xy`, so the cones survive.
- Filled: the contact is projected again with `mingap`. Every layer keeps an
  interface slab into which the cones merge; the empty ones are dropped. The support carried to the layers below keeps `gap_xy`.
- Tree and organic ignore the option.
- The support column volumes keep `gap_xy`: they have no contact of their own.
- The first-layer foot is widened with `gap_xy` as before.

The contact detection threshold does not change: an overhang ribbon narrower
than the grid contact margin (about one external perimeter width) still gets no
contact.

## Verification

`[support-contact-xy]` covers the parameters, both growth units, zero growth,
profile/project persistence, invalidation and their fallbacks, the cone
merge on synthetic slabs (vanishing cones, cones covered by the support, cones
joining and following the support, bottom contacts, zero step), and sliced fixtures in both modes: a
tab narrower than `gap_xy`, a steep slope, a sharp lower edge, the normal
support of an overhanging roof with several interface layers, the settings that
must change nothing, the filled family and the tree family.
The default is compared with independently reconstructed fixed-step parameters;
the STL-to-G-code matrix varies growth linearly and checks real contact surfaces,
support interface paths and emitted extrusion moves.
