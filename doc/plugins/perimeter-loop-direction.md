# Perimeter loop direction

The `perimeter.post_process.loop_direction` plugin gives every closed perimeter
loop its printing direction, whatever generator made it (classic, simple,
Arachne, Python or third party). Generators publish loops in the direction their
geometry gives; this step, after overhang detection and before ordering, applies
three settings of the loop's region:

- `perimeter_direction`: counter-clockwise or clockwise, separately for contours
  and holes. The HOLE perimeter flag of the loop tells them apart.
- `perimeter_reverse`: on odd layers, every loop is reversed. The plugin defines
  this setting.
- `overhangs_reverse`, when `perimeter_reverse` is off: on odd layers, an island
  with a steep overhang reverses its loops that have an overhang path. The
  overhang is steep when the island area outside the layer below survives a
  shrink by half of `overhangs_reverse_threshold`. Overhang paths are the ones
  that `perimeter.post_process.detect_overhang` marked, so the reversal follows
  the overhang settings.

Layers are counted from zero, so the "even layers" of the tooltips are the odd
layer indices. Gap fill and thin walls keep their direction.

A loop with its direction is no longer reversible. Ordering providers reverse
with `ExtrusionTree::reverse_tree_keeping_loops()`: such a loop keeps its
direction when a collection around it is reversed, and only its seam may still
change.

The plugin reads the settings from the first region of each region island, so
perimeter generators keep apart the regions that disagree on any of the four
settings (`overhangs_reverse_threshold` included).
