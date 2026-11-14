# Using Unified Extrusion Entities

> Plugin ABI version: `50`
>
> Extrusion-property ABI: `2.2` (minor 2 appends `DEPOSIT = 11`; payload sizes are unchanged)

The versions above identify the plugin API against which this guide and its
examples were checked. Update them whenever this guide is revised for a later
plugin API.

The plugin API represents every printable path, ordered path group, and
non-geometric event with the same `slic3r_api::ExtrusionEntity` tree model. A
plugin does not need to know which legacy C++ subclass the host may use
internally. It reads the node's geometry, children, flags, and properties
through one stable interface.

This guide uses the C++ API as the primary interface. The final section maps the
same concepts to the C ABI and Python views.

## One Node Type, Three Content States

An extrusion entity is one tree node in exactly one of these states:

1. **Empty:** no local polyline and no children. Empty nodes may still carry
   properties, for example a custom G-code event.
2. **Polyline leaf:** one local polyline and no children. The polyline contains
   points, optional arc metadata, and optional per-point Z offsets.
3. **Collection node:** ordered children and no local polyline. Properties on
   the parent may describe state inherited by its descendants.

A node cannot have a local polyline and children at the same time. An operation
that adds children to a polyline leaf, or sets a polyline on a collection, is
rejected. Call `clear_content()` first only when discarding the existing
content is intentional.

Do not use a C++ class name to decide what a node means. Inspect its structure:

```cpp
void inspect(const slic3r_api::ExtrusionEntity &entity)
{
    if (entity.has_polyline()) {
        // Local leaf geometry is available through points() and segments().
    } else if (entity.child_count() > 0) {
        // Children preserve their current traversal order.
    } else {
        // The node is empty, although it may still carry direct properties.
    }
}
```

`is_leaf()` means that the node has no children. An empty entity is therefore a
leaf even though it has no polyline.

## The Three C++ Views

The three wrappers expose the same entity but have different ownership and
mutability contracts.

| View | Mutability | Ownership | Typical source |
| --- | --- | --- | --- |
| `ExtrusionEntity` | Read-only | Borrowed | Input data, a const child, `PrintingExtrusion::root()` |
| `MutableExtrusionEntity` | Mutable | Borrowed | A mutable step output, a mutable child, `PrintingExtrusion::mutable_root()` |
| `StoredExtrusionEntity` | Mutable | Owned through `storage_handle` | A temporary tree created or cloned by a plugin |

A borrowed view never frees its handle. Its owner must outlive the view. A
`StoredExtrusionEntity` is move-only and calls `storage_free()` when destroyed.
Return a stored entity from a function that creates ownership; never return a
borrowed view into a local stored entity.

```cpp
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"

using namespace slic3r_api;

StoredExtrusionEntity clone_for_output(
    storage_handle *storage,
    const ExtrusionEntity &source)
{
    return StoredExtrusionEntity(storage, source);
}
```

Handles obtained from a host-owned layer or printing plan remain host-owned.
Wrapping such a handle in `MutableExtrusionEntity` does not transfer ownership.

## Creating A Printable Leaf

Coordinates use the scaled `coord_t` representation. Use `scale_i()` for points
and Z offsets. Width, height, speed, and volumetric values stored by extrusion
properties use the units documented by their payload, usually millimeters or
millimeters per second.

```cpp
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"

using namespace slic3r_api;

StoredExtrusionEntity make_perimeter(storage_handle *storage)
{
    StoredExtrusionEntity path(storage, {
        make_point(scale_i(0.0),  scale_i(0.0)),
        make_point(scale_i(20.0), scale_i(0.0)),
        make_point(scale_i(20.0), scale_i(10.0))
    });

    EPropertyAttributes &attributes =
        path.get_or_add(EPropertyAttributes::key);
    attributes
        .extrusion_role(RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER)
        .mm3_per_mm(0.08)
        .width(0.45f)
        .height(0.20f);

    path.reversible(true);
    return path;
}
```

`EPropertyAttributes` is normally required on printable leaves, either directly
or inherited from a parent. Other process decisions, such as speed,
acceleration, temperature, or fan speed, use additional properties. See
[Using Plugin Properties](/doc/plugins/properties.md) for direct and dynamic property access.

### Variable-Width Lines

`thick_polyline_extrusion()` converts one centerline with a width per point,
such as an Arachne wall, into a storage-owned entity: a loop when the line
closes on itself, a multi-path when the width changes, otherwise one path. The
widths are spacings between neighbor lines, except for thin walls. The
parameters give the role, the base flow (built from its width, or from its
spacing with `THICK_POLYLINE_EXTRUSION_FLOW_FROM_SPACING`), the segment
resolution and width tolerance (0 selects the defaults: 1/8 and 1/20 of the
flow width), a final `flow_mult`, and whether the entity can be reversed.

```cpp
c_thick_polyline_extrusion_params params = {};
params.role = RAW_EXTRUSION_ROLE_SOLID_INFILL;
params.flow = recipe.flow;
params.flow_mult = 1.f;
params.flags = THICK_POLYLINE_EXTRUSION_FLOW_FROM_SPACING | THICK_POLYLINE_EXTRUSION_CAN_REVERSE;
std::optional<StoredExtrusionEntity> line = thick_polyline_extrusion(storage, points, widths, params);
```

### Stationary Deposit Leaves

A stationary deposit is a leaf with exactly one polyline point, flags 0, ordinary
printable `EPropertyAttributes`, and `EPropertySpecialCommand` set to
`C_EXTRUSION_SPECIAL_COMMAND_DEPOSIT` (11). Its `extra_data` is a finite positive
volume in mm3 **before** the extrusion multiplier. `is_deposit()` identifies the
direct command; consumers validate its point count and process values separately.
Other one-point leaves are not deposits.

The leaf needs effective positive finite `mm3_per_mm`, width, height and speed,
an active tool and a printable role, with no `EPropertyExtrusionAxis`. It belongs
inside a `PrintingExtrusion` root, never a before/after event scope. The default
firmware session emits role/dimension markers, travels to the quantized point,
then emits E-only `G1 E F` with print acceleration. The tool converts the volume
using `delta_E = volume * e_per_mm3()`. Its filament speed is
`speed * mm3_per_mm * extrusion_multiplier / filament_area` in both classic and
volumetric E modes. The temporary speed is restored after the move. The plan
supplies any required unretraction; the deposit itself only repays existing debt.

For support roles, let `s = 10^-gcode_precision_e` and `r` be the carried E
rounding remainder (filament mm, or mm3 with volumetric E). The writer trials
`extrude(delta_E)` on a copy of the axis state. If it emits an E word, or
`delta_E < 0.5*s`, the ordinary result is retained. Otherwise a negative
remainder swallowed the deposit: the writer extrudes `s-r` instead, emits
exactly one step, clears the remainder and accounts for the adjusted amount.
The extra is below one step and is never recovered; absolute E does not decrease.
A support deposit with no emitted E word carries its remainder and emits
`G4 P20` at the deposit point. The estimator and reader omit this 20 ms dwell.
The G-code reader ignores stationary E moves of at most 1e-4 mm of filament, so
such deposits are missing from read-back statistics while the writer counts them.
The half-step trigger and dwell are first limits that may be tuned. They are
named in the default firmware writer header.

The one-point position survives cloning and instance translation. It contains
no segments, so the pre-G-code preview draws no geometry. The G-code reader
classifies stationary positive E as unretraction, although deposited volume is
accounted for after outstanding debt. Raw `EXTRUSION` retains its separate signed
filament-length meaning and does not apply flow or the extrusion multiplier.

### Retraction And Wipe Process Leaves

The ordered G-code pipeline also represents extrusion-axis preparation with
ordinary entities. Retract and Unretract requests may be empty E-only events or
geometric wipe leaves. An empty event carries only `RAW_EXTRUSION_ROLE_RETRACT`
or `RAW_EXTRUSION_ROLE_UNRETRACT`. A geometric event additionally carries
`RAW_EXTRUSION_ROLE_TRAVEL | RAW_EXTRUSION_ROLE_WIPE`.

Both forms use the same semantic axis request:

```cpp
MutableExtrusionEntity retract = /* ordered empty event leaf */;
retract.get_or_add(EPropertyAttributes::key)
    .extrusion_role(RAW_EXTRUSION_ROLE_RETRACT)
    .mm3_per_mm(0.0);
retract.get_or_add(EPropertyExtrusionAxis::key)
    .retract_to(0.8, false);

MutableExtrusionEntity unretract = /* ordered empty event leaf */;
unretract.get_or_add(EPropertyAttributes::key)
    .extrusion_role(RAW_EXTRUSION_ROLE_UNRETRACT)
    .mm3_per_mm(0.0);
unretract.get_or_add(EPropertyExtrusionAxis::key)
    .unretract(0.05, false);
```

These requests deliberately do not contain firmware syntax. A firmware session
may encode them as explicit E-only moves or native `G10`/`G11` commands while
its `ExtrusionAxisState` remains responsible for the physical E state. On a
geometric Wipe leaf, explicit E is distributed over the complete planar line
and arc length; Z lift does not increase the requested E movement. A Wipe that
only moves the nozzle has no `EPropertyExtrusionAxis`.

## Building An Ordered Tree

Build trees from the bottom up. Create complete leaves first, attach their
properties, then move them into parent entities in the desired order.

```cpp
StoredExtrusionEntity make_two_path_sequence(storage_handle *storage)
{
    StoredExtrusionEntity root(storage);
    root.disable_sort();
    root.disable_reverse();

    StoredExtrusionEntity first = make_perimeter(storage);
    StoredExtrusionEntity second(storage, {
        make_point(scale_i(20.0), scale_i(10.0)),
        make_point(scale_i(0.0),  scale_i(10.0))
    });

    EPropertyAttributes &attributes =
        second.get_or_add(EPropertyAttributes::key);
    attributes
        .extrusion_role(RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER)
        .mm3_per_mm(0.08)
        .width(0.45f)
        .height(0.20f);

    const uint32_t first_idx = root.append_child_move(first.mutable_view());
    const uint32_t second_idx = root.append_child_move(second.mutable_view());
    if (is_invalid_index(first_idx) || is_invalid_index(second_idx))
        throw std::runtime_error("The extrusion sequence could not be built.");

    return root;
}
```

After `append_child_move()`, the source handle remains valid but is empty. The
new child is owned by the parent tree. Use `append_child_copy()` when the source
must remain unchanged:

```cpp
void append_clone(
    StoredExtrusionEntity &root,
    const StoredExtrusionEntity &source)
{
    const uint32_t idx = root.append_child_copy(source.readonly());
    if (is_invalid_index(idx))
        throw std::runtime_error("The extrusion could not be cloned into the tree.");
}
```

Use `move_child_from()` when moving an already attached child between parents.
Do not combine an insertion with manual removal: the dedicated operation keeps
ownership and indices consistent.

### Inserting An Ordered Boundary Leaf

Use `emplace_ordered_leaf()` when a new empty leaf must execute immediately
before or after everything already represented by an entity. The method keeps
the current entity handle stable and makes the resulting outer node
non-sortable and non-reversible:

```cpp
MutableExtrusionEntity event = root.emplace_ordered_leaf(
    OrderedLeafPosition::Before,
    ExistingPropertyPlacement::MoveWithExistingContent);
if (!event.valid())
    throw std::runtime_error("The ordered event leaf could not be inserted.");

event.script_gcode(script, script_type);
```

`KeepOnParent` leaves the current node's direct properties on the outer parent.
The new leaf therefore inherits them. If the current entity is already a fixed
collection, the leaf is inserted directly; otherwise the previous content is
moved below a wrapper that preserves its former flags.

`MoveWithExistingContent` always creates that wrapper, even when the current
entity is already fixed or has no properties. Existing direct properties and
their variable-size resources move with the old content, so the new leaf does
not inherit them. This is the appropriate policy for inserting an event before
a leaf that already carries a different event or process state.

`Before` places the new leaf first and `After` places it last. Existing children
are transferred without cloning, so their handles remain stable. The returned
leaf is empty, non-sortable, non-reversible, and non-rotatable; populate it only
after checking that the returned view is valid. The parent becomes non-rotatable,
while wrapped old content and existing child objects retain their rotation permissions.

The operation is structurally atomic: invalid arguments or an allocation
failure leave the tree unchanged. A successful insertion invalidates borrowed
views of the modified node's direct structure and properties. Reacquire those
views before further use; borrowed views of transferred existing children stay
valid.

## Flags, Order, And Continuity

`RAW_EXTRUSION_FLAG_REVERSIBLE` allows an optimizer to reverse an entity.
`RAW_EXTRUSION_FLAG_SORTABLE` allows it to reorder the entity's direct children.
The C++ helpers expose these as `reversible()` and `sortable()`.

Since extrusion API 1.1, `extrusion_can_rotate()` / `extrusion_set_can_rotate()`
and the C++ / Python helpers `can_rotate()` / `set_can_rotate()` expose a separate
permission to change a closed contour's cyclic starting point. A true permission
does not prove geometric closure. Reading or rewriting the two legacy flags,
including `disable_sort()` / `disable_reverse()`, does not change this permission.
Copies, clones and moves preserve it; leaf splits preserve it on the stable
container and every fragment, and `clear_content()` does not reset it.

Continuity is computed, not stored as a mutable flag. A non-sortable collection
is continuous when all non-empty descendants are continuous and adjacent paths
touch end-to-start. A sortable collection is never reported as continuous
because its order may change later.

Use a non-sortable, non-reversible parent whenever child order carries semantic
meaning, such as a command that must execute immediately before a path. Explicitly
set its rotation permission to false as well; ordered boundary insertion does
this for both its parent and the new event leaf.

## Points, Segments, Arcs, And Z Offsets

Point operations create or edit straight geometry. Segment operations preserve
arc orientation, radius, and endpoint Z offsets.

```cpp
StoredExtrusionEntity make_arc(storage_handle *storage)
{
    StoredExtrusionEntity arc(storage);

    c_extrusion_segment segment = {};
    segment.point_a = make_point(scale_i(0.0),  scale_i(0.0));
    segment.point_b = make_point(scale_i(10.0), scale_i(10.0));
    segment.radius = static_cast<float>(scale_d(10.0));
    segment.orientation = RAW_EXTRUSION_ARC_ORIENTATION_CCW;
    segment.z_offset_a = scale_i(0.0);
    segment.z_offset_b = scale_i(0.2);

    if (!arc.set_segments({ segment }))
        throw std::runtime_error("The arc extrusion is invalid.");
    return arc;
}
```

For a straight segment, set `radius` to zero and use
`RAW_EXTRUSION_ARC_ORIENTATION_UNKNOWN`. Adjacent segments must agree on their
shared point and shared Z offset.

The local methods `point_count()`, `segments()`, `local_front()`, and
`local_length()` inspect only the current node's polyline. The tree-wide methods
`front()`, `back()`, `length()`, `empty()`, and `collect_points()` recursively
inspect descendants when the current node is a collection.

`collect_points()` deliberately flattens structure and arcs. Do not use it to
copy or transform an entity when preserving exact geometry matters; clone the
entity or operate on its segments instead.

## Traversal And Inherited Properties

`entity.get(key)` reads only a property stored directly on that entity. Use an
extrusion tree visitor when a parent property applies to its descendants.

```cpp
#include "libslic3r/Api/plugin/cpp/ExtrusionTreeVisitors.hpp"

class PrintableLeafVisitor : public slic3r_api::ExtrusionTreeConstVisitor<>
{
protected:
    void visit_leaf(slic3r_api::ExtrusionEntity leaf) override
    {
        if (!leaf.has_polyline())
            return;

        const slic3r_api::EPropertyAttributes *attributes =
            current_property(slic3r_api::EPropertyAttributes::key);
        if (attributes == nullptr)
            throw std::runtime_error("A printable leaf has no attributes.");

        process_leaf(leaf, *attributes);
    }

private:
    void process_leaf(
        slic3r_api::ExtrusionEntity leaf,
        const slic3r_api::EPropertyAttributes &attributes)
    {
        // The real plugin may inspect or aggregate the validated leaf here.
        (void) leaf;
        (void) attributes;
    }
};

void inspect_printable_leaves(slic3r_api::ExtrusionEntity root)
{
    PrintableLeafVisitor visitor;
    visitor.traverse(root);
}
```

`current_property()` searches from the current node toward the root and returns
the closest direct value. It works with built-in and dynamically registered
`PluginPropertyKey` instances.

The mutable visitor permits local edits to the current entity. Do not add or
remove siblings or ancestors during traversal. Replacing the current leaf by a
collection is supported, but the newly inserted children are not visited in
that same pass.

## Structural Mutation And Borrowed Views

Treat child views and property pointers like iterators into a mutable container.
They may be invalidated by:

- inserting, removing, moving, or reordering children;
- replacing an entity with `copy_from()` or `move_from()`;
- removing or recreating a property;
- clearing the owner, its storage, or its printing-plan container.

Read the needed value before mutation, then reacquire child views and property
pointers afterwards. Do not keep fragment handles returned by
`split_leaf_by_areas()` across a later structural mutation of their parent.

`split_leaf_at_lengths()` cuts a leaf at increasing distances from its start, each
piece longer than `SCALED_EPSILON`. Like the area splitter, the leaf becomes a
non-sortable collection of copies holding the exact intervals, so arcs, Z and the
volume per length are kept. A leaf carrying a custom G-code, special command
(including a raw E command), heater or extrusion-axis event is refused, because
every piece would repeat it.

`copy_from()` and `move_from()` preserve the destination handle identity while
replacing its content, properties, flags, children, and stored data. The move
operation leaves the source valid but empty.

## Editing A PrintingPlan Clone

`PrintingExtrusion` owns a clone of its source extrusion root. Ordering and
later printing-plan plugins should edit this clone, not the original
`LayerRegionIsland` tree.

```cpp
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"

void disable_reordering(slic3r_api::PrintingExtrusion extrusion)
{
    slic3r_api::MutableExtrusionEntity root = extrusion.mutable_root();
    root.disable_sort();
    root.disable_reverse();
}
```

Use `PrintingExtrusion::root()` for a read-only view and `mutable_root()` only
in a step whose contract allows tree mutation. `set_root_clone()` keeps its
source unchanged; `set_root_move()` consumes a standalone mutable entity.

The mutable `PrintingPlan`, `PrintingGroup`, `PrintingLayerGroup`,
`PrintingToolGroup`, and `PrintingExtrusion` views are accepted only by plan
construction and editing steps. `STEP_GCODE` receives a
`ConstPrintingPlan`; its child scopes and property containers remain const all
the way down. A G-code writer may inspect geometry and metadata, but it cannot
cast the final-plan handle back to a mutable C++ wrapper or add properties.

The host resumes an invalidated pipeline from fixed checkpoints:

| Checkpoint | Kept and frozen                                   | Resumes at        |
|------------|---------------------------------------------------|-------------------|
| None       | nothing                                           | `STEP_LAYER_HEIGHT` |
| Slicing    | layer heights and slices                          | `STEP_PRE_PERIMETER` |
| Print      | the whole data tree, after `STEP_FINALIZE_SLICING` | `STEP_ORDERING`   |
| Ordering   | optional private copy of the ordered plan         | `STEP_WIPETOWER`  |
| Plan       | the finished plan                                 | `STEP_GCODE`      |
| GCode      | everything                                        | nothing           |

An invalidated step resumes from the last checkpoint before it. The Ordering
copy is an implementation detail of `Print`: plugins receive only the active
working plan, restored as a fresh clone, and cannot access the saved copy.
Because the data tree is frozen from the Print checkpoint on, steps from
`STEP_ORDERING` onwards, including wipe tower and support-spot detection, must
write their results into the plan, never into objects, layers or islands.
Every setting listed through
`used_config_keys()` by an active plugin is constrained to invalidate no later
than that plugin's owning plan step, so replay cannot preserve annotations
produced from stale settings. The host applies this once, when plugin option
definitions are finalized at startup. Settings declared with `STEP_ANY` or with
an explicit `STEP_NONE` keep their declaration.

Extrusions attached directly to data-tree objects may use object-local
coordinates. Printing-plan roots are cloned and transformed according to their
plan instance. Check the contract of the step that supplied the handle before
applying an instance transform.

### Traversing PrintingPlan Entities By Direct Property

`PrintingEntityPropertyTraversal` streams the entities carrying one direct
property across a `PrintingLayerGroup`. It also supplies the owning tool group
and `PrintingExtrusion`, without allocating a flattened vector:

```cpp
#include "libslic3r/Api/plugin/cpp/PrintingEntityPropertyTraversal.hpp"

struct TransitionMarker
{
    uint32_t id;
};

using MarkerEntity = slic3r_api::PrintingEntity<TransitionMarker>;

void process_marked_entities(
    orchestrator_handle *orchestrator,
    slic3r_api::PrintingLayerGroup layer_group)
{
    const slic3r_api::PluginPropertyKey<TransitionMarker> marker_key =
        slic3r_api::PluginPropertyKey<TransitionMarker>::register_dynamic(
            orchestrator, "example.transition_marker");

    slic3r_api::PrintingEntityPropertyTraversal<TransitionMarker> traversal(
        marker_key,
        [](MarkerEntity *previous, MarkerEntity *next) {
            // previous is null at layer start; next is null at layer end.
            if (previous != nullptr && next != nullptr)
                previous->property->id = next->property->id;
        });
    traversal.process(layer_group);
}
```

The traversal uses depth-first pre-order inside each extrusion tree. Without a
state definition it only selects direct properties and does not resolve any
inherited process state. Its context and selecting-property pointers are
borrowed for the callback only.

When a consumer also needs inherited properties, describe exactly which keys
must be retained. The traversal resolves them lazily from the
`PrintingExtrusion` root through each matching entity, including direct values
on the matching entity:

```cpp
slic3r_api::ExtrusionPropertyStateDefinition state;
state.track(slic3r_api::EPropertyZOffset::key);
state.track(
    slic3r_api::EPropertySpeed::key,
    [](slic3r_api::EPropertySpeed &effective,
       const slic3r_api::EPropertySpeed &direct) {
        if (direct.speed_mm_per_s > 0.f)
            effective.speed_mm_per_s = direct.speed_mm_per_s;
    });

slic3r_api::PrintingEntityPropertyTraversal<TransitionMarker> traversal(
    marker_key, std::move(state),
    [](MarkerEntity *, MarkerEntity *next) {
        if (next == nullptr)
            return;
        const slic3r_api::EPropertyZOffset *z_offset =
            next->effective_properties.get(slic3r_api::EPropertyZOffset::key);
        if (z_offset != nullptr)
            use_z_offset(z_offset->get());
    },
    slic3r_api::MatchingEntityDescendants::Skip);
```

`track(key)` implements ordinary nearest-value inheritance: each direct value
replaces the complete effective payload. The callback overload is useful for a
payload whose individual fields override only when they satisfy a condition.
Its merge callback runs only after a first direct value has established an
effective payload.

The traversal keeps unresolved frames for the active tree path and materializes
them only when that branch reaches a match. Resolved parent state is reused by
matching siblings. `effective_properties` contains autonomous payload copies,
so the snapshot retained in `previous` remains valid after its branch is left.
Pointers returned by `effective_properties.get()` belong to that snapshot and
must not outlive the callback context.

Tracked payloads must be trivially copyable and self-contained. Do not track a
property containing an `extrusion_data_id`: copying such a payload would copy
only the resource identifier, not the variable data owned by the extrusion.
During a stateful traversal, callbacks must not modify tracked properties on a
matching entity or its active ancestors because later snapshots reuse that
already resolved path state.

A callback may update payloads, add unrelated properties, and edit a dedicated
descendant subtree which does not carry the selecting property. Transition
plugins use this rule to populate a scope's `travel`, `before`, or `after`
phase while keeping the marked scope root stable. Do not change a matching
root's child list or an active ancestor's child list, add or remove selecting
properties, reorder matching entities, or retain callback pointers. The
traversal reacquires a matching payload after callback-side property storage
changes, but structural identity and order must remain unchanged.

Separate `process()` calls do not share a previous entity, so their null
boundaries are layer-local.

By default, descendants of a matching entity are still traversed, so a direct
property on both a parent and a child produces two matches. When a property
defines non-nested logical roots, pass
`MatchingEntityDescendants::Skip` as the third constructor argument. The
traversal then reports the marked root and skips its complete subtree. This is
both faster for large scope contents and allows callbacks to populate dedicated
phase children without making those new descendants part of the active walk.

## Migrating Legacy Extrusion Classes

Migration from the old extrusion hierarchy is a change of representation, not
a call to a generic converter. Plugin code should stop branching on
`ExtrusionPath`, `ExtrusionEntityCollection`, `ExtrusionMultiPath`, or
`ExtrusionLoop` and instead use the unified node's content, flags, and
properties. An old path becomes a leaf with a local polyline and
`EPropertyAttributes`; an old collection or multi-path becomes a parent with
ordered children; sort and reverse capabilities become entity flags; and a
loop is represented by closed geometry or by an ordered non-sortable parent
when it contains multiple pieces. Empty command-like entities remain empty
nodes carrying command properties.

When porting construction code, migrate bottom-up:

1. Create each path as a `StoredExtrusionEntity`.
2. Copy its points or segments and add its attributes and process properties.
3. Create parent `StoredExtrusionEntity` nodes and set their flags.
4. Move children into each parent in final traversal order.
5. Move or clone the completed root into the host-owned output.

Legacy raw owning pointers and collection-specific `append()` calls should not
cross the plugin API. Use storage-owned entities and explicit copy/move methods
instead. If the host supplies an entity whose native implementation is still a
legacy subclass, its `extrusion_entity_handle` already exposes the unified base
view; no cast or wrapper conversion is required.

## C And Python Correspondence

| Operation | C++ | C ABI | Python |
| --- | --- | --- | --- |
| Borrow read-only | `ExtrusionEntity(handle)` | `const extrusion_entity_handle *` | `api.extrusion(handle)` |
| Borrow mutable | `MutableExtrusionEntity(handle)` | `extrusion_entity_handle *` | `api.mutable_extrusion(handle)` |
| Create owned | `StoredExtrusionEntity(storage)` | `extrusion_create_empty(storage)` | `api.new_extrusion(storage)` |
| Deep clone | `entity.clone(storage)` | `extrusion_clone(storage, entity)` | `entity.clone(storage)` |
| Read children | `entity.children()` | `extrusion_child_count()` / `extrusion_child()` | `entity.children()` |
| Append copy | `append_child_copy()` | `extrusion_insert_child_copy()` | `add_child_copy()` |
| Append move | `append_child_move()` | `extrusion_insert_child_move()` | `add_child_move()` |
| Set straight geometry | `set_points()` | `extrusion_polyline_set_points()` | `set_points()` |
| Set straight geometry from a polyline | `set_polyline()` | `extrusion_polyline_set_points()` | `set_polyline()` |
| Preserve arcs and Z | `set_segments()` | `extrusion_polyline_set_segments()` | `set_segments()` |
| Direct property lookup | `entity.get(key)` | `extrusion_property_data()` | `entity.property(Payload)` |

The C ABI is authoritative for binary compatibility. C callers must release
standalone storage-owned entities with `storage_free()`. Python
`StoredExtrusionEntity` supports explicit release through `free_from_storage()`
and automatic release with the `with` statement; borrowed Python wrappers never
own the host object.

Python mutable property payloads are live `ctypes` views, while read-only
property access returns a copy. Reacquire a mutable payload after structural
mutation for the same reasons as in C++.

## Travel provider selection

The exclusive group `layer_extrusion_edit.travel` selects how gaps between
ordered extrusion leaves become travels. The three bundled providers are enabled:
`layer_extrusion_edit.travel.advanced` (group priority -2, the default),
`layer_extrusion_edit.travel.avoid_crossing_perimeters` (-1), and
`layer_extrusion_edit.travel.default` (0, straight lines). Group priority orders
selector choices independently of execution priority. Presets keep their stored
provider IDs without migration; storing `layer_extrusion_edit.travel.default`
keeps straight travel. The advanced default applies when the selector is absent;
newly initialized configurations receive that ID from the generated definition.

The avoidance provider checks `avoid_crossing_perimeters` at both regional
endpoints. When an endpoint lies on a region boundary, the lookup can miss
because the point may belong to two regions; the provider then retries at a
point just inside the leaf, if that point is available. It keeps
straight travel if either endpoint disables avoidance, has no regional data,
or has no valid object instance (including global skirt and brim paths).
With `avoid_crossing_not_first_layer` enabled, it also excludes the target's
first object layer and every layer at that print Z, including support layers.
Both points are translated by the target instance shift for routing, and the
returned route restores the shift and exact original endpoints. The shared
connector handles insertion, epsilon snapping, Z interpolation and travel flags.

Before routing, `PerimeterCrossing` tests the direct segment against prepared
interior boundaries, not extrusion centerlines. When region settings assign
perimeters or infill to a tool other than the current one, object interiors are
clipped to the current tool so its material does not ooze over another tool's
area. `perimeter_extruder`, `infill_extruder` and `solid_infill_extruder` assign
the three roles independently. A region contributes `raw_slices - fill` for
perimeters, `fill - solid` for sparse infill, and `solid` for solid infill.
When the infill tools differ, `fill` is the region's `fill_expolygons()` area
restricted to `raw_slices`, and `solid` is its solid fill surfaces restricted to
that area. When infill and solid infill share a tool, that tool receives the
whole, unrestricted `fill_expolygons()` area instead. When the three
roles agree, the region contributes `raw_slices` to that tool instead.
Preparation keeps a base version and a copy inset by 1.5 nozzle radii,
then removes top surfaces from both. The provider tests the inset version.
Off-object targets use slices from each object's last normal layer strictly below
the query Z plus `SCALED_EPSILON`, expanded by 1.5 radii without tool clipping or
top removal. A target is off object when it has no region or its layer is not a
normal object layer: support, raft, skirt, brim and wipe-tower geometry qualify.
The wipe queries at the higher of the layer group's print Z and that Z plus the
target's inherited Z offset. The legacy planner queries its off-object targets
with a region (support and per-object brim) at the layer-group Z.
Real catalog instances in the current and earlier
printing groups count; later groups and synthetic entries such as wipe towers
do not supply blockers. These queries use platter coordinates, without removing
the target's shift. No object below means no crossing. Auxiliary geometry itself
is not a blocker. A path that does not
cross those boundaries remains straight.

Within one object instance, the router uses internal rings around inset object
interiors. Between objects or instances, it uses external rings: holes placed
at instance positions, rather than outer obstacle outlines. The support boundary
uses object slices at or below the target height; its purpose is undocumented,
and it does not route support travels around objects. Detection can see another
instance while the support routing boundary still belongs to the owner; such a
crossing may remain straight when no detour is available. Object areas are translated
by each instance's XY shift; differently rotated instances have separate sliced
objects. Detection includes finished instances in earlier groups, using their
last normal layer below the query Z rather than a full height history.
The compatible planner uses this detection for off-object targets with a region
and real instance, including support and per-object brim. Its router still uses
the owner's local geometry. Regionless and global targets keep straight routes,
but the wipe gate also tests their generated travels against platter blockers.
The router places endpoints near usable rings, follows
ring arcs and tries shortcuts. It may keep a direct path and does not guarantee
a shortest path or the removal of every crossing.

Top surfaces come from `LayerRegionIsland`; shared islands' tops are clipped to
each region's raw slices. Support has no tops. Crossing preparation removes all
tops regardless of `avoid_crossing_top`. Routing removes the tops selected by
that setting after insetting them by half a perimeter spacing, and retains those
inset areas in `to_avoid` to compare candidate paths. A top band is kept only where there are perimeters and no
ironing: a detour can follow the top perimeter, while ironing passes over its
edge. Perimeter spacing is averaged over regions with nonempty raw slices;
external spacing includes objects at that print Z. Missing spacing falls back
to the mean nozzle diameter. Departure and arrival placement use the same
search for a nearby point inside the island.

`avoid_crossing_perimeters_max_detour` limits extra length beyond the direct
distance, in millimetres or as a percentage of that distance. Zero is unlimited.
An over-budget route, or a result with fewer than two points, keeps straight
travel.

The implementation is internal to the host, under
`src/libslic3r/Plugins/LayerExtrusionEdit/AvoidCrossing/`, not a third-party API.
Each layer-group worker owns its copied inputs, routers and crossing caches;
inputs outlive routers, and temporary geometry uses run-context plugin storage.

```text
AvoidCrossingPerimetersTravel: choose a route for each eligible gap
|-- LayerInputCache::get: retain target geometry, settings and spacing
|   `-- build_input: copy public Print, Object and Layer views
|-- PerimeterCrossing: prepare interiors and test the direct path
`-- TravelRouter: build boundaries and choose a detour
    |-- TravelBoundary: retain rings and their search grids
    `-- build_external_input: copy other objects only when needed
```

### Advanced travel

`layer_extrusion_edit.travel.advanced` uses the same connector in two phases.
`ScopeTravelConnector::prepare` records a layer-local chronology of gaps and
snaps nearby seams, without inserting travels. `apply` requests one route per
recorded gap in order, inserts travels, interpolates Z and updates flags.
`T` is the zero-based index in `LayerTravelChronology::travels`, not a scope
index. The chronology also retains the target extrusion owner and tool for
every traversed pair, including pairs without a new gap. Its summary maps
consecutive runs of the same target owner and, separately, consecutive runs of
the same tool to half-open T intervals, including empty ones.

The exclusive group `layer_extrusion_edit.travel.advanced.planner` selects
`.legacy` (Legacy, priority -1, the default) or `.advanced` (Advanced, 0), using those suffixes
after the group ID. Legacy uses the same routes as
`travel.avoid_crossing_perimeters`. The host resolves the selector among active
members; if only Advanced is active, it selects Advanced. The advanced provider uses
Legacy when the resolved plugin is null or is not the Advanced member, or when
Advanced is selected but no gap after layer entry is eligible.
Planner members expose selection; the advanced provider performs the routing itself.

A dated zone has an attractor or forbidden role, polygon geometry in platter XY
coordinates, a half-open interval `[begin, end)` in T, and a scope. `SIZE_MAX`
means an open end for the rest of the layer. Scope can restrict the tool, require
either endpoint to match an instance in the plan catalog, or require either
endpoint extrusion to be support. Attractors form a routing domain; endpoint
attachments and inter-island jumps may leave it.

`TravelZoneView` collects contributor geometry once per worker, rejects reversed
dates and drops empty areas and intervals. Queries advance through dated events
in nondecreasing T order, filter by scope and cache unions by role for each
applicable set. Zone construction is skipped if there is no eligible gap after
layer entry.

Contributors are independently activated plugins under
`layer_extrusion_edit.travel.advanced.zones.`:

- `island_attractors` is active by default. It withdraws each perimeter slice by
  1.5 perimeter spacings with `elephant_foot_compensation`
  (`Plugins/ElephantFootCompensation.hpp`), shared with first-layer XY
  compensation. It withdraws material while keeping thin walls, limiting
  withdrawal near a minimum width of external perimeter width plus
  perimeter spacing. Support also includes slices from the last object layer
  below the support target's Z plus `SCALED_EPSILON`. Withdrawn parts are shifted
  per instance and dated by consecutive travels whose source
  or target belongs to the original slice on that layer and instance, including
  its border, not by membership in the withdrawn parts.
- `forbidden_tops` is active by default. It copies the boundary's excluded top
  components as separate forbidden zones for the whole layer, scoped per instance.
- `printed_tops` is opt-in and replaces `forbidden_tops` when active. It uses the
  same top components, but starts each after the last target arrival in its
  original island; a top without such an arrival is omitted.
- `support_obstacles` is opt-in. It copies separate object slice components from
  the layer at the group's Z or just below it, shifted for every instance in the
  plan catalog. These whole-layer forbidden zones apply only to travels touching
  support, regardless of endpoint instance or tool.

Activation is global through the plugin activation set, configured by
`activated.ini`, not a per-region contributor setting. The provider checks each
contributor with `orchestrator_find_plugin`. It checks `forbidden_tops` only when
`printed_tops` is inactive: disabling `printed_tops` restores static tops if
`forbidden_tops` is active, while disabling `forbidden_tops` changes nothing when
`printed_tops` is active. Otherwise, disabling a contributor removes only its
zones. With `printed_tops` inactive, disabling `forbidden_tops` removes top
avoidance from Advanced. Legacy routing and its own top avoidance are unchanged.
Platter blockers for outside-part travels apply independently of these contributor
activation switches.

The advanced planner uses the regional check described above, but an endpoint
without an object instance does not veto the other endpoint. When neither has
an instance, the default region configuration's
`avoid_crossing_perimeters` value governs eligibility (false if unavailable).
The incoming layer-entry travel has no source extrusion and stays straight.

Eligible XY gaps use a ranked candidate pipeline; Z-only moves stay straight.

1. **Classify and collect obstacles.** An endpoint with a region on a normal object
   layer is in-part. Regionless endpoints and auxiliary layers (support, raft,
   skirt, brim, tower) are outside-part. For every travel, ignore individual original
   forbidden zones containing either endpoint. If either endpoint is outside, also
   add `platter_blockers`: translated normal slices of the plan's objects at the
   layer-group Z or below, from printing groups up to the current one. The synthetic
   wipe tower is not a blocker. Ignore endpoint-containing components of these
   object slices too, then union and expand all remaining forbidden material by the
   tool's nozzle radius. This leaves the in-part endpoint's own component reachable
   on a mixed travel. Subtract the expanded
   obstacles from active attractors to form domain components.
2. **Rank hypotheses.** An in-part endpoint offers its containing domain, otherwise
   the nearest component within 1.5 perimeter spacings, plus the option without a
   domain (`D∅`). Outside-part endpoints offer only `D∅`, even near a domain.
   Contained points stay unchanged; other attachments use the nearest ring point.
   At most four pairs are ranked stably, domain choices first on equal costs.
   Same-component and attachment estimates cost `w * length`; bridges and other
   in-part connectors cost outside-domain length plus `w * inside-domain length`,
   measured across all components. An outside-part endpoint's leg counts its full
   length without a domain penalty or inside weight. Here `w` is
   `avoid_travel_island_weight` when island avoidance is active, otherwise zero.
   For an in-part travel whose endpoint candidates share one component, admit
   `D∅/D∅` only when its chord stays inside the components grown by 1.5 spacings
   (allowing `SCALED_EPSILON` outside). Between different components, or without
   an endpoint candidate, this material-proxy restriction does not apply.
3. **Construct the candidate.** Within one component, join the attachments by a
   chord that follows the component's rings only where it crosses them. Between
   components, select bridge points with the legacy weighted jump search, route
   within each component to those points and contour expanded
   obstacles along the bridge. A mixed domain/`D∅` choice uses the domain ring
   point nearest the free endpoint, then contours obstacles to that endpoint.
   A `D∅/D∅` candidate contours obstacles on the original chord. Estimates and
   construction share all selected bridge/entry/exit points. ZonePlanner never
   delegates to Legacy.
4. **Validate, then retry.** Require exact endpoints, no segment crossing original
   forbidden material and the complete-path detour budget. Actual outside-domain
   length must not exceed that of one obstacle-only reference detour by more than
   `SCALED_EPSILON`. Compute that reference at most once per blocker set; if unusable, skip only
   this bound. Rejection tries the next hypothesis. After exhaustion, only if no
   hypothesis passed forbidden-material validation, disable the remaining original
   forbidden component first crossed along the straight chord
   or, if that chord is clear, along the initial best-ranked estimate polyline (including
   attachments and bridge), and rebuild obstacles, domains and hypotheses.
   Budget-only or outside-length rejection keeps the counted straight fallback
   without deactivation. This includes zone components
   and slices of the plan's objects. Equal crossing positions prefer zone order,
   component order, then object-slice order. Disabled components remain in the
   ignored cache identity and are excluded from validation for this travel only.
   Each component is disabled at most once; the number crossed by the initial chord
   or initial best estimate (counting each component once) caps
   deactivations. No crossed component, or reaching that cap without acceptance,
   returns one counted straight fallback. `disabled_blockers` counts explicit
   deactivations across calls, not endpoint exclusions. An accepted `D∅` route is not a fallback. `domain_routes`
   counts accepted domain paths, `bridges` accepted inter-component bridges, and
   `detours` accepted paths longer than the chord by more than `SCALED_EPSILON`.

Spacing and island settings come from the target layer input, then the source
when the target has no region, with nozzle diameter as the spacing fallback.
The worker caches areas and component rings by active zones, tool, ignored zones
and spacing. Travels using platter blockers additionally key group, Z and ignored
platter components; the plan catalog and object slices remain unchanged during use.
Obstacle contours use first entry/last exit arcs; holes, tangencies and collinear
contacts do not count as material crossings. Missing contour entries/exits can
reject a candidate. `avoid_crossing_perimeters_max_detour` limits extra length in
millimetres or as a percentage of chord length; zero is unlimited. Routing is
heuristic, not shortest-path search; validation checks original material, not the
nozzle margin, and the final fallback remains straight.

```text
AdvancedTravel::run_impl: prepare and fill one layer's gaps
|-- ScopeTravelConnector::prepare: record chronology and snap seams
|-- orchestrator_select_plugin: choose Legacy or Advanced
|-- summarize: evaluate eligibility and date owner/tool runs
|-- TravelZoneView: collect active contributors and cache dated role unions
`-- ScopeTravelConnector::apply: request routes in T order and insert travels
    |-- CompatibilityPlanner::path: use perimeter routing
    `-- ZonePlanner::path: classify endpoints, cache geometry and rank hypotheses
        |-- is_off_object_layer / platter_blockers: resolve outside-part geometry
        |-- build_boundary / endpoint_candidates: index components and choose attachments
        |-- select_island_bridge / estimate_cost: retain shared points and rank estimates
        |-- append_component_path / route_on_boundary: follow each chosen component
        |-- detour_around_forbidden: contour bridges and free connectors
        `-- validate all segments: accept, retry hypotheses or remove a crossed blocker within the component bound, then fall back
```

### Crossing wipes and lift

Retraction with `wipe_only_crossing` uses the same copied-input and crossing
helpers on the generated travel, independently of travel-provider selection.
`layer_extrusion_edit.wipe.default` (-80) retains inside-start approaches and
plain outgoing wipes when this setting is off. With it on, gaps within a layer
where exactly one side is outside object material, or both sides are object
material on different instances or slice islands, use the normal outgoing
wipe with `wipe_return` and inside-end settings. Regions and tools sharing one
slice island do not establish a certain crossing. Object material excludes
global, regionless and auxiliary-layer geometry, including support, brim and
skirt. All other transitions, including those between two auxiliary paths and
inter-layer gaps, depend on the generated travel in the second pass. The independent
`layer_extrusion_edit.wipe.crossing` (-45) depends on that first pass and runs
after built-in travels (-50), before lift (-40). It adds a wipe only for a
travel phase whose Travel leaves, without Wipe, form one continuous path, with a
valid target region island and no departure geometry. The phase may be a single
leaf or an ordered subtree, such as a travel split for deceleration. A terminal
stationary deposit prevents the wipe. The lift provider accepts the same
travel shapes and applies one lift profile along all their leaves.

The crossing wipe returns to its source start, which must agree with the unchanged
travel start within `SCALED_EPSILON` in XY distance and absolute Z before lift.
The lift provider then applies one ramp over every outgoing Wipe
leaf, including motion-only leaves, returning wipes and inside-end segments.
`wipe_lift_length` is relative to the total wipe length when expressed as a
percentage and is capped to that length. The final wipe point reaches
`wipe_lift` above its original Z on planar wipes when both settings are positive.
For non-planar wipes, the first point keeps its original Z and each following
point takes the maximum of the preceding wipe Z and its original path Z plus
the ramp. The final Z is therefore the running maximum of these values across
all wipe leaves. The following travel starts at this lifted Z.
Complete source loops no longer than the configured wipe use whole turns rounded
up to the required length; longer loops and open paths go halfway out and return
over the same path. `wipe_return` and inside-end settings do not apply. Wipe and
retraction settings belong to the source tool; crossing preparation and nozzle
diameter belong to the target tool. Every off-object target uses the platter
crossing rule described above, so an off-object-to-off-object gap crossing the part
can wipe while a gap over void does not wipe for crossing alone.

Before editing, each target layer worker publishes its first travel's exact start
and crossing result as plain values. Source workers can thus handle inter-layer
gaps, skipping empty layer groups, without reading another worker's geometry.
Retraction remains split into stationary, moving and residual absolute targets.
