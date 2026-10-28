# Regional dimensional compensation

FFF volume slicing produces raw geometry with only its technical closing
radius. `slicing.xy_size_compensation` runs at post-slicing priority -1000 and
applies the regional dimensional settings once, before perimeter generation.

- `xy_size_compensation` moves exterior boundaries: positive adds material.
- `xy_inner_size_compensation` moves hole walls: positive fills holes in.
- `first_layer_size_compensation` adds to both offsets and fades over
  `first_layer_size_compensation_layers` object-local layers. Raft layers are
  skipped without renumbering that fade.
- `first_layer_size_compensation_no_collapse` limits withdrawal near narrow
  walls using external-perimeter width and spacing. It does not thicken walls
  or promise a final minimum wall width.

Original material retains its region. Growth is allocated to regional
candidates; any unclaimed remainder is assigned by contact with the regional
masks. Significant remainders generate a warning rather than disappearing.
Different regional deltas can leave small notches or steps at their boundary.

`slicing.hole_size_compensation` runs first, at priority -1100. It changes only
convex holes of the merged material. Negative `hole_size_compensation` enlarges
them; positive values shrink them. `hole_size_threshold` gives the full distance
up to its area and fades to zero at four times that area; zero disables the
threshold. Inner XY offsets then add to this result. Regions move only their
own part of a shared wall; the first region wins where growth claims overlap.
Islands inside a hole remain untouched. XY first-layer no-collapse protection
does not account for the preceding hole compensation.

The geometry helpers operate on caller-owned SDK storage, without host headers.
Clipper 2.1 adds variable inward offsets with one distance per ring vertex.
Earlier Clipper 2.x packages remain compatible. Volume 2.0 and its
`c_mesh_slicing_params.extra_offset` field are unchanged: other SDK consumers
can still request a mesh offset. The FFF slicing provider explicitly requests
zero dimensional mesh offset to avoid double compensation.

SLA uses the distinct native settings `sla_first_layer_size_compensation` and
`material_ow_sla_first_layer_size_compensation`. Legacy conversion recognizes an
explicit SLA technology or SLA-only keys. It does not infer technology inherited
from a vendor section absent from the converted input map.
