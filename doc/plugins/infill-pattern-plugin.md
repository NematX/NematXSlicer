# Writing an infill pattern plugin

Infill patterns are `INFILL_PATTERN` service plugins. The selected `STEP_INFILL`
generator asks the host to run one pattern plugin per fill surface, and the
plugin writes its extrusion into the output entity it receives. A pattern
plugin only needs the public C ABI and the C++ view helpers; the gyroid pattern
in `src/libslic3r/Plugins/Infill/GyroidPattern.cpp` is the reference
implementation. Honeycomb, 3D honeycomb, Hilbert curve, Archimedean chords and
octagram spiral live in the same directory as separate plugins built on the
shared `PatternPluginHelpers.hpp` utilities.

## Initialization

Before the first run, the generator performs a discovery pass and reports the
work groups of every pattern through `report_pattern_assignments()`. The host
aggregates them and calls `setup()` once per used pattern with a
`run_ctx_infill_pattern_init` payload listing the objects and their groups. Use
it to precompute per-object state (grids, bounding boxes, synchronization); the
payload stays valid until the run for the same object completes.

## Step contract

A pattern plugin is a `PluginBase` subclass with `step_impl()` returning
`INFILL_PATTERN` and `run_impl()` reading `run_ctx_infill_pattern`:

```cpp
const run_ctx_infill_pattern *ctx = plugin_ctx_as_infill_pattern(run_ctx);
if (ctx == nullptr || ctx->surface == nullptr || ctx->params == nullptr || ctx->output == nullptr)
    return;

const raw_infill_pattern_params &params = *ctx->params;
const slic3r_api::Surface surface(ctx->surface);
const slic3r_api::ExPolygon area = surface.expolygon();
```

`params` is the complete per-surface recipe: role, flow, density, spacing,
angle, overlap and gap-fill flags. Distances stored as `coord_t` are scaled
integers; `double` fields are unscaled millimetres unless documented otherwise.
`params.object_bounding_box` is the scaled bounding box of the whole object,
used by patterns that align across layers.

The boolean recipe options are grouped in `params.infill_flags`, a bitmask of
`c_infill_flag` values (`steps/slic3r_step_infill.h`). This changed the layout of
`raw_infill_pattern_params`, so `steps/slic3r_step_infill.h` moved to MAJOR 2:
packages built against MAJOR 1 must be recompiled against the current headers.

Built-in plugins can use the helpers in
`src/libslic3r/Plugins/Infill/PatternPluginHelpers.hpp` instead of repeating the
legacy `Fill` frame handling: `infill_angle()`, `shrink_for_fill()`,
`append_clipped_pieces()`, `order_pieces()`, `append_polyline()`,
`publish_infill_paths()` and `add_self_to_pattern_list()`. The helpers use the
public plugin API and the ordering engine interface, so they are also a readable
reference for external plugins.

## Declaring membership

The pattern list settings (`fill_pattern`, `top_fill_pattern`, ...) are
plugin-defined enums of plugin ids. A pattern declares the lists it belongs to
during initialization; the host keeps the contribution even when the list is
created later. Built-in plugins call `add_self_to_pattern_list()`:

```cpp
void inilialize_impl(storage_handle *) const override
{
    add_self_to_pattern_list(m_orchestrator, k_id, "Honeycomb", { "fill_pattern" });
}
```

The underlying patch is the generic option-definition API:

```cpp
key_value_string_pair_t entry = { id_impl(), name_impl() };
key_value_string_pair_array_t entries = { &entry, 1 };

raw_option_def_patch patch = raw_option_def_patch_init();
patch.kind = RAW_OPTION_DEF_PATCH_ADD_ENUM_VALUES;
patch.opt_key = "fill_pattern";
patch.enum_values = entries;
orchestrator_modify_option_def(m_orchestrator, &patch);
```

Inactive plugins are pruned from the lists during finalization.

## Building geometry

Use the view helpers instead of internal slicer types:

- `slic3r_api::Surface`, `ExPolygon`, `Polygon`, `Polyline` from
  `SurfaceViews.hpp` / `GeometryViews.hpp`;
- `ClipperOperand`, `clipper_offset()`,
  `clipper_intersection_polyline_expolygons()` from `ClipperViews.hpp` for
  clipping and offsetting;
- `scale_i()`, `scale_d()` and `unscaled()` from `slic3r_def.h` for unit
  conversion;
- `slic3r_safe_rand()` from `slic3r_utils.h` for thread-safe inclusive random
  values when a pattern needs a host-seeded random sequence;
- the `c_bounding_box` helpers from `GeometryViews.hpp` (`bounding_box_empty()`,
  `bounding_box_rotate()`, `bounding_box_merge_rotated()`,
  `bounding_box_merge_aligned()`, ...) to build a pattern frame.

A pattern that generates its lines from `box.min` must anchor the lattice to the
world frame, otherwise the pattern shifts with the cross-section of every layer:
call `bounding_box_merge_aligned(box, {module_x, module_y})` before generating,
like the legacy `Fill` implementations did. The honeycomb, 3D honeycomb, gyroid
and plane-path patterns do this; sparse plane-path and gyroid frames stay tight
to the surface, and only their minimum corner is snapped.

`run_ctx->plugin_storage` owns every temporary `Stored*` object. The gyroid
plugin, for example, shrinks the surface by half the line spacing, clips each
generated wave against it and drops the pieces shorter than one extrusion
width.

Clipper ABI 1.1 also provides `clipper_intersection_polylines_expolygons()` and
`clipper_diff_polylines_expolygons()` for a whole `PolylineCollection` against an
`ExPolygonCollection`, with C, C++ and Python wrappers. The host executes the
core operation once for the batch, leaves both inputs unchanged and returns an
independent storage-owned polyline collection. An empty subject produces an
empty result; an empty mask produces an empty intersection or a copy of the
subjects for difference. The C entry points also accept a null mask with those
same semantics and return null for a null storage or subject. Collection order
and correspondence between output and source indices are unspecified; paths
remain open and retain their bends. Cubic and Lightning use batch intersection
once per inset component, then copy the results into their mutable connector
buffers; they do not embed a separate Clipper implementation.

`clipper_offset()` and `clipper_offset2()` never leave zero-width bridges: rings
meeting exactly along an edge are split as the native `offset_ex()` splits them.
For ExPolygon sources they give the native rings. The order of the resulting
components is unspecified.

## Publishing extrusion

Wrap `ctx->output` in a `MutableExtrusionEntity` and move a storage-owned child
into it. Set the printable attributes on each leaf:

```cpp
slic3r_api::StoredExtrusionEntity group(run_ctx->plugin_storage);
group.set_flags(group.flags() | RAW_EXTRUSION_FLAG_SORTABLE | RAW_EXTRUSION_FLAG_REVERSIBLE);

slic3r_api::StoredExtrusionEntity path(run_ctx->plugin_storage);
path.set_polyline(piece);   // piece is a Polyline view, transferred without a point copy
path.get_or_add(slic3r_api::EPropertyAttributes::key)
    .extrusion_role(int32_t(params.extrusion_role))
    .mm3_per_mm(params.flow.mm3_per_mm * double(params.flow_mult))
    .width(float(unscaled(params.flow.width) * double(params.flow_mult)))
    .height(float(unscaled(params.flow.height)));
group.append_child_move(path.mutable_view());

slic3r_api::MutableExtrusionEntity output(ctx->output);
output.append_child_move(group.mutable_view());
```

`set_polyline()` borrows the point array from another API polyline view instead
of building a `std::vector<c_point>`, which is what the helpers use to publish a
`StoredPolylineCollection` directly.

Mark the parent `SORTABLE`/`REVERSIBLE` when the order and direction of the
paths may be changed by `STEP_ORDERING`. A pattern that produces one fixed path
should clear those flags instead.

## Volume correction

The host stamps `c_extrusion_property_infill` on the subtree root of every
surface: `source_surface_id` identifies the surface and the other fields carry
its raw fill recipe. `area_to_fill_mm2` is the free-of-perimeter-overlap area
the surface expects when the recipe requests an enforced full fill
(`enforce_full_fill_volume`); it is 0 otherwise. A pattern neither reads nor
sets those fields.

The correction itself is a `STEP_POST_INFILL` service
(`infill.post_process.full_fill_volume`): it rebuilds the volume target and the
legacy spacing/width normalization from the stamped recipe, then scales the
`mm3_per_mm` and `width` of the leaves by the clamped ratio, exactly like the
legacy `Fill::fill_surface_extrusion()` did. A pattern therefore only generates
geometry and publishes `params.flow` scaled by `params.flow_mult`; it does not
implement `fill_exactly` or 100% density handling itself.

The normalizer is a no-op with the default `solid_infill_overlap = 100%`; a
reduced overlap stays an intentional void instead of being refilled by the
correction.

## Rectilinear family port

The family exposes eleven stable plugin ids:
`rectilinear`, `alignedrectilinear`, `monotonic`, `grid`,
`triangles`, `stars`, `cubic`, `scatteredrectilinear`, `sawtooth`,
`rectiwithperimeter` and `supportbase`. All eleven ids share the raster /
multi-sweep code ported from `FillRectilinear.cpp` into
`Plugins/Infill/Rectilinear`; `sawtooth`, `rectiwithperimeter` and `supportbase`
add or replace a stage on top of it:

- `RectilinearFills.cpp` implements the raster / multi-sweep algorithms on
  private work buffers while receiving the boundary as an API `ExPolygon` view
  and the fill recipe as raw plugin parameters.
- `RectilinearConnect.hpp` links the generated lines along the surface boundary
  (`PrusaSimpleConnect` / `FakePerimeterConnect`), so those patterns still
  produce connected paths. It consumes borrowed `BoundaryContours` views and
  materializes only contours that need cleanup or inserted junction points.
- `RectilinearPatterns.cpp` configures each filler from the recipe with
  `fills::configure_filler()`, then passes the API geometry view, raw recipe,
  and `plugin_storage` directly to the pattern implementation. The monotonic
  patterns publish their group without `SORTABLE` / `REVERSIBLE` so the path
  order and direction survive `STEP_ORDERING`; the other patterns stay
  sortable.
- `rectiwithperimeter` and `supportbase` are registered as `INFILL_PATTERN`
  providers but intentionally do not join any user option list. `supportbase`
  is selected by its stable plugin id for sparse support-base recipes.
  `rectiwithperimeter` is reachable through direct plugin-id selection, while
  normal brim and preset paths continue to use the core `Fill` implementation.
- `sawtooth` reuses the rectilinear XY raster and publishes a Z-profiled
  extrusion tree. Its z-hop offsets use `slic3r_safe_rand()`, and the
  generator sets `C_INFILL_FLAG_DONT_ADJUST` for the sawtooth recipe so later
  flow normalization does not alter the generated profile.

Every infill pattern is now a standalone plugin; the legacy pattern adapters
are gone, and the `smoothhilbert` and `smoothtriple` variants were removed (old
projects load them as `smooth`). Brim, support and the wipe tower still
instantiate the legacy core `Fill` classes. Dense infill uses recipe modifiers
and post-infill plugins instead of those core fillers.

## Line

`line` is a standalone `INFILL_PATTERN` provider (`LinePattern.cpp`) with
priority 7 and membership only in `fill_pattern`. It ports `FillLine`: odd lines
are tilted by the line spacing minus the extrusion spacing, and consecutive lines
are joined when the connecting segment stays inside the island grown by half a
spacing. `max_sparse_infill_spacing` is ignored, as before. On a full surface
without `C_INFILL_FLAG_DONT_ADJUST`, the line spacing is stretched (at most 20%)
to the object width. As in the legacy filler (which never initialized
`Fill::spacing_priv`), the half-spacing surface shrink is zero unless the spacing
was stretched; each island is then inset by 0.7 spacing when that keeps it whole.

`layer_id` is the index of the (combined) layer being filled, so the layer
alternation uses it directly. The current pipeline does not combine infill layers
(`thickness_layers` is always 1); a future combination step must store the
combined index there instead of changing each pattern.

Intentional difference: with `infill_connection` set to "not connected" the
legacy filler emitted no line at all; the plugin emits the chained lines without
joining them.

## Concentric

`concentric` is a standalone `INFILL_PATTERN` provider with priority 9, available
in `fill_pattern`, `top_fill_pattern`, `bottom_fill_pattern` and
`solid_fill_pattern`. It ports the classic `FillConcentric` onion offsets:
density spacing honors `max_sparse_infill_spacing`, and full adjustable surfaces
stretch spacing to the object width; that adjusted spacing also controls the
initial half-spacing inset, including the native millimeter/integer round-trip.

The initial half-spacing inset uses `clipper_offset()`, whose component order is
unspecified; each component restarts its seam reference at the origin, so its
paths do not depend on the other components. Each onion level is offset from the
previous one, so the plugin builds the ring nesting tree while generating it,
without a union of all loops: a contour lies inside the contour it was offset
from, a hole inside the deeper hole that grew around it or else inside its own
contour. The tree is printed depth first, every ring after the ring around it,
and among sibling branches the one nearest to the last seam comes next. Seam
selection uses native reduced-bit integer distances and keeps the first
candidate on ties; loop clipping follows the untrimmed seam reference.

The extrusion group is not sortable or reversible. Tests check, per component,
the native rings, that no ring follows a ring it encloses, and the native order
whenever every ring is nested in the next one; seams are not compared, as loops
are reversible and their seams are meant to change. Paths are reversible unless
`MONOTONIC` is set, and paths closed by zero loop clipping remain extrusion loops.

### Arachne walls

The generator sets `USE_ARACHNE` on concentric recipes when the region uses the
Arachne perimeter generator. On a full surface without `DONT_ADJUST`, the plugin
then ports the `FillConcentric` Arachne branch: each component of the initial
inset is grown back by half a spacing and given to `Arachne::WallToolPaths`
with the adjusted spacing as bead spacing, the recipe flow width as bead width,
and as many walls as fit in the component. The walls follow inset by inset,
outer first; each closed wall starts at its vertex nearest to the end of the
previous one, from the origin for each component, before loop clipping. Each
wall goes through `thick_polyline_extrusion()`: its widths are spacings, the
flow is rebuilt from the recipe spacing (ratio 1 when the recipe ratio is lower
and the surface is not a bridge), and `flow_mult` is applied.

`Plugins/Infill/ArachneInfill.hpp` holds the shared part: the seven settings
`WallToolPaths` reads (`min_feature_size`, `min_bead_width`,
`wall_transition_filter_deviation`, `wall_transition_length`,
`wall_transition_angle`, `wall_distribution_count`, and the smallest
`nozzle_diameter`), read from the region and print configurations into an
`Arachne::WallToolPathsSettings` and declared in `used_config_keys()`; the copy
of API expolygons into the Arachne outline; and the conversion of one wall.

The walls of all components go in one group that is not sortable, like the
classic loops. Tests compare every wall entity with the native one (points,
roles, flows, flags) and check that each component keeps the native wall order.

Intentional differences:

- the native collection was sortable, so its walls could lose their
  outside-in order;
- sparse or `DONT_ADJUST` Arachne recipes keep the classic loops of this plugin
  with the recipe flow; the native filler converted those loops with a flow
  rebuilt at spacing ratio 1;
- the legacy gap-fill branch is not ported. Gap fill and full-fill volume
  correction belong to post-infill plugins.

## Smooth (Ironing)

`smooth` (`SmoothPattern.cpp`, label "Ironing") is a standalone `INFILL_PATTERN`
provider with priority 17, available in `top_fill_pattern`,
`bottom_fill_pattern` and `solid_fill_pattern`. It ports `FillSmooth` with the
ported rectilinear fillers, configured from the recipe by
`fills::configure_filler()`:

- the deposit pass is a monotonic fill at the recipe density with
  `1 - fill_smooth_distribution` of the flow. When the surface has no-overlap
  areas and is not a bridge, it only fills those areas grown by half the
  overlap;
- the ironing pass covers the whole surface, a quarter turn further when
  `C_INFILL_FLAG_CAN_ANGLE_CROSS` is set, at the density divided by
  `fill_smooth_width`, with `RAW_EXTRUSION_ROLE_IRONING_INFILL` and
  `fill_smooth_distribution` of the flow. The line flow is divided by the same
  factor, so each pass keeps its share of the volume.

Both settings are read from the region of `primary_region` and declared in
`used_config_keys()`. The root keeps the deposit before the ironing; every path
is monotonic and not reversible. The direct test matches the native tree:
collection flags, points, roles and flows.

Intentional differences:

- the native deposit pass forced its own gap fill; gap fill belongs to the
  post-infill step;
- the native sub-fillers always rotated on odd layers; the plugin fillers follow
  `C_INFILL_FLAG_CAN_ANGLE_CROSS` (`fill_angle_cross`);
- the native filler corrected the volume of each pass separately; the full-fill
  volume step scales the whole surface tree with one factor, which keeps the
  flow split between the passes.

## Ensuring

`ensuring` (`EnsuringPattern.cpp`) is a standalone `INFILL_PATTERN` provider
with priority 24, available only in `solid_fill_pattern`, of which it is the
default. It ports `FillEnsuring` and always runs Arachne, whatever
`USE_ARACHNE` says:

1. the surface is rotated so the fill direction (fill angle, quarter turn on odd
   layers when the angle may cross, or the bridge angle) becomes vertical;
2. the area wide enough for lines, the surface intersected with its opening by
   two then three spacings, is shrunk by half a spacing minus the overlap;
3. vertical lines one spacing apart are cut by that area into sections longer
   than a spacing, and clusters of short sections that would make the nozzle
   vibrate are removed;
4. the area covered by the sections is traced back; `Arachne::WallToolPaths`
   fills every gap left in the surface (grown by the overlap) with walls of one
   spacing, and gap walls whose ends nearly meet in line are joined;
5. the lines are ordered, lines touching an already printed one first, then the
   nearest; consecutive lines whose ends are within two spacings are joined, and
   everything is rotated back.

Every line goes through `thick_polyline_extrusion()` like the concentric
Arachne walls, into one group that is sortable and reversible unless the recipe
is `MONOTONIC`, as the legacy collection was. The Clipper steps go through the
API; the boundary queries of `AABBTreeLines` (sorted line intersections,
inside test) and `Algorithm::sort_paths()` are ported in the plugin as linear
scans with the same arithmetic, and the endpoint search uses the API
`KDTreeIndirect` with the native tolerance. Tests compare the whole tree with
the native one on several shapes, angles, overlaps and Arachne settings.

A thin sliver can make the native filler, and thus this plugin, join gap walls
into consecutive duplicate points, which `thin_variable_width()` rejects with an
assertion in Debug builds; the tests leave that case out.

## Adaptive and Support Cubic

`adaptivecubic` and `supportcubic` are `INFILL_PATTERN` providers with priorities
15 and 16, respectively. Both belong only to `fill_pattern` and share the
implementation in `Plugins/Infill/AdaptiveCubic`, while retaining separate
per-instance caches and progress state.

Each plugin's `setup()` prepares one owned octree per assigned object, then
publishes the complete cache only after preparation succeeds. Repeated surfaces
for that object share the mode-specific immutable octree. The next `setup()`
starts a new generation and discards the previous caches; object handles are
generation-local lookup keys, not persistent cache identities. There is no
preparatory pipeline step or global cache registry.

Preparation reads model-part meshes, transforms, configuration and surfaces
through public views. Object-wide cell spacing is computed independently for
each mode and remains separate from per-surface flow spacing. It uses unweighted
mean widths and positive densities of configured regions matching that mode and
having nonempty fill surfaces. Width zero uses `1.125f * maximum_nozzle`; percent
widths use the maximum nozzle diameter as their base. An assigned object without
an eligible region fails explicitly, even if its first surface recipe requests
that pattern with positive density; there is no recipe-based spacing fallback.
Existing surfaces carrying `MOD_BRIDGE` contribute triangles at their layer's bottom Z;
`MOD_OVERBRIDGE` alone does not. No new bridge candidates are inferred, and
source surfaces and their metadata are never modified. This differs intentionally
from the legacy pre-bridge candidate filter.

Adaptive mode refines the octree with every model triangle. Support mode uses the
same complete mesh bounds and level spacing, but refines it only with faces whose
normal points upward by more than the legacy 0.707 threshold in octree coordinates.
The volume and object transform handedness are applied to triangle winding before
that test. Marked bridge triangles bypass the normal filter in both modes.

Private glu-libtess triangulates contours with holes without interpreting native
handles. API batch intersection delegates open-path coordinate preparation and
restoration to the host; Boost.Geometry indexes hook collisions. The host uses
its existing filtered determinant predicate with an exact 128-bit fallback.

`src/libslic3r/Plugins/Common/WorkBuffers.hpp` owns the shared mutable points, double vectors, bounding
boxes and polylines, including core-compatible rounding and Douglas-Peucker
simplification. The same `Plugins/Common/` directory provides polygon containers,
borrowed contour views and geometry helpers in `PolygonWorkBuffers.hpp`, Clipper
operations in `ClipperHelpers.hpp`, and borrowed contour grid queries in
`PluginEdgeGrid.hpp/.cpp`. These helpers use the neutral `slic3r_api::PluginCommon`
namespace, with `work` for point and path buffers, `polygon_work` for polygon
buffers and its `line_alg::distance_to_squared` segment-distance helper, `clipper` for Clipper helpers, and `EdgeGrid`
for grid queries. `InfillPaths.hpp` and `InfillChaining.cpp` remain in
`Plugins/Infill/`. `InfillPaths` consumes the neutral buffers directly and
converts between API-owned collections and
the private buffers without reinterpreting their layouts; the copies retain
all points and check cancellation while moving temporary stored payloads.

Surface generation applies the half-spacing inset once per surface, processes
each resulting component independently, builds Adaptive hooks, then chains or
connects paths using the selected connection and anchor settings. Publication
keeps paths sortable and reversible. Parity tests compare complete directed
polylines without requiring their order within the output group to match.
Adaptive parity covers both inputs without additional bridges and projected
bridge surfaces with holes across the connection and anchor settings.

Adaptive, Support Cubic and Lightning prepare their own caches in `setup()`;
none requires an additional pipeline preparation step.

## Lightning

`lightning` is a standalone `INFILL_PATTERN` provider with priority 23 and
membership only in `fill_pattern`. It reads all current internal sparse/void
surfaces of each assigned object, including non-Lightning regions, without
changing any source surface. It neither reconstructs the model mesh nor creates
internal bridge candidates.

`setup()` builds independent object jobs in parallel using per-job scratch
storage. Each object's trees are constructed from top to bottom: new branches,
root reconnection, deep copying, pruning, straightening and boundary realignment.
Only a fully successful generation is published. Caches own ordinary C++ memory
and are discarded at the next setup; no temporary views or scratch handles are
retained. Shared child nodes and weak parents allow rerooting without ownership
cycles.

The supporting radius uses unweighted mean widths and densities of configured
Lightning regions with positive density. Width zero uses `1.125f * maximum_nozzle`;
percent widths use that automatic width as their base. Selection through a recipe
modifier without a configured positive-density Lightning region fails explicitly.
Unlike the legacy first-region width and nominal-height calculation, wall support
uses each layer's actual height, while pruning and straightening use the difference
between consecutive print Z positions at the legacy 45-degree angles.

Unsupported points are sampled on a shared grid and stably ordered by fixed
distance bands, then the legacy point hash modulo 191. Contour intersection
queries select the nearest hit to their destination. These rules include the
separately tested corrections to the legacy comparison and intersection code.

`run()` reads immutable trees and converts them with `slic3r_safe_rand()` from the
host, without reseeding. Each inset surface component is clipped independently;
API batch intersection preserves complete branch bends, and `InfillPaths` copies
them into work buffers and chains paths, while the rectilinear connector handles
requested boundary connections. Junction
overlap, flow attributes and sortable/reversible publication follow the recipe.
Tests pin only their own thread-local seed; production paths need not be identical
under different random sequences or scheduling.

Multi-component parity feeds the native connector the same offset traversal
order as the API: the legacy offset can return the same regions in a different
order, assigning different random branch choices to each region. A cancellation
before final publication discards the whole surface group, including paths
already constructed for earlier components.

The mixed-region fixture uses unequal region areas, absolute and percent widths,
a non-Lightning region and a zero-density Lightning region. It compares the
whole-object outline union independently of the parameter means. A real height
profile changes between 0.2, 0.1 and 0.3 mm in the object's middle; its physical
deltas also drive analytic pruning and straightening checks on complete directed
three-point branches. These revised parameter contracts use independent oracles,
not legacy first-region or nominal-height behavior.

The multi-object integration fixture serializes upstream slicing/surface builders
that share mutable storage and restores concurrency before Lightning setup. This
isolates that existing upstream race; the Lightning migration does not fix it.

## Limits

The plugin API does not expose a polygon-aware `connect_infill()` substitute:
paths cannot be linked through the surface interior. A pattern therefore
publishes independent sortable paths and lets `STEP_ORDERING` fix their order,
direction and travels. The self-contained connect port used by the rectilinear
family and Adaptive Cubic supplies boundary linking without becoming part of
the public API.

`OrderingEngine.hpp` provides
`connect_infill(StoredPolylineCollection&, const OrderingEngine&)` for the
pattern-side pre-pass. It orders and orients the collection in place with the
supplied engine (for example
`connect_infill(pieces, NearestNeighborOrderingEngine{})`) but it never joins
two polylines. That is a hard requirement: clipping splits a path wherever it
leaves the surface, so a joined path would extrude a straight segment across the
hole or void between the pieces.

Plane-path patterns (`hilbertcurve`, `archimedeanchords`, `octagramspiral`)
reproduce the legacy frame handling in `fill_plane_path_surface()`: they
generate in a rotated grid, use `params.object_bounding_box` to align sparse
infill across layers, then clip and order in the surface frame.
