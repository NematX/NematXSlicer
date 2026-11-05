# Plugin-accessible storage: lifetime and mutability

This reference describes the native storage behind the plugin C API and its C++ views, as well as their publication boundaries. It serves to determine which data can be destroyed, kept or saved during a pipeline resume.

Member names are those of the native storage, not necessarily those of the API getters. Fields with the same owner and the same lifetime may be grouped on one line, but are named explicitly. Internal caches and associations needed to interpret a view are flagged as such; a line does not mean that a public getter exists for each field.

**This document separates the target contract from the current state of its implementation.** Host cleanup per checkpoint, the plugin `clear` notification, the optional deep `Ordering` clone and the snapshots of records and main-object properties at `None`/`Slicing` exist. `custom_gcode_per_print_z` is published by `STEP_ORDERING`. The `DATA_TREE` 4 API enforces the edit gates of properties, slices, configurations and source extrusions according to the freeze mode; surfaces use scratch capabilities. No general snapshot of properties or source geometry is planned. The required steps 5.1–5.6 are implemented, with the conservative routing and the limits below; only the surface amendment of 5.7 is included.

## Contents

1. [Conventions and general rules](#1-conventions-and-general-rules)
2. [Print roots and printable objects](#2-print-roots-and-printable-objects)
3. [Model, volumes, regions and slicing associations](#3-model-volumes-regions-and-slicing-associations)
4. [Layers, regions, islands and surfaces](#4-layers-regions-islands-and-surfaces)
5. [Printing plan and catalog](#5-printing-plan-and-catalog)
6. [Extrusions, geometry and extrusion properties](#6-extrusions-geometry-and-extrusion-properties)
7. [Generic geometry and plugin allocations](#7-generic-geometry-and-plugin-allocations)
8. [Generic properties, records and configurations](#8-generic-properties-records-and-configurations)
9. [Step contexts and temporary outputs](#9-step-contexts-and-temporary-outputs)
10. [Sessions, services and result buffers](#10-sessions-services-and-result-buffers)
11. [Current limits and consequences for checkpoints](#11-current-limits-and-consequences-for-checkpoints)
12. [API coverage](#12-api-coverage)

## 1. Conventions and general rules

### Boundaries

| Marker | Boundary / meaning |
|---|---|
| `apply` | Preparation of model/configuration inputs by the host, outside concurrent plugin execution. |
| `None` | Before `STEP_LAYER_HEIGHT`: current model and configuration; resumes at `STEP_LAYER_HEIGHT`. |
| `Slicing` | After `STEP_POST_SLICING`: profile, main layers and slicing geometry kept; resume at `STEP_SUPPORT_DEMAND`. |
| `Print` | After `STEP_FINALIZE_SLICING`: source tree finalized; resume at `STEP_ORDERING`. |
| `Ordering` | After `STEP_ORDERING`: optional save of the plan, which remains editable in its working copy. |
| `Plan` | After `STEP_EXTRUSION_SIMPLIFICATION`: final plan read-only for `STEP_GCODE`. |
| `GCode` | End of export and of the optional read-back; does not reopen any write right on the inputs. |
| Call / session / storage | Lifetime independent of checkpoints, specified for the relevant owner. |

The actual order is that of `Steps::execution_order()`, not the numeric order of the `slicing_step_t` values.

Sources: `src/libslic3r/Steps/StepPipeline.hpp:34`, `src/libslic3r/Steps/StepPipeline.cpp:69`.

### Routing audit of `STEP_SKIRT_BRIM` options

The **67 initial definitions** of `src/libslic3r/Config/FFFPrintConfig.cpp` give **8 reclassifications and 59 kept routes**. This inventory is the tracked reference; it does not mean that the 67 options were optimized. No key name, persisted alias, algorithm or ABI contract is changed.

| Reclassified batch | Keys | Resume step / checkpoint | Verified first consumer |
|---|---|---|---|
| Ordered catalog (1) | `gcode_flavor` | `STEP_ORDERING` / `Print` | Name sanitization, [ObjectInstanceCatalog.cpp:54–60](../../src/libslic3r/Plugins/Ordering/ObjectInstanceCatalog.cpp#L54), `used_config_keys` declaration at `:154–166` |
| Plan editing (1) | `travel_speed` | `STEP_LAYER_EXTRUSION_EDIT` / `Ordering` | [DefaultSpeed.cpp:958–962](../../src/libslic3r/Plugins/LayerExtrusionEdit/DefaultSpeed.cpp#L958), declaration at `:76`; [DefaultFan.cpp:1027](../../src/libslic3r/Plugins/LayerExtrusionEdit/DefaultFan.cpp#L1027) calls the time estimator, `Api/plugin/cpp/PrintingPlanTimeEstimator.cpp:150` |
| Export (4) | `first_layer_temperature`, `temperature`, `travel_speed_z`, `z_offset` | `STEP_GCODE` / `Plan` | [GCode.cpp:304–353](../../src/libslic3r/GCode.cpp#L304), `:1878–1992,8028–8031`; [GCodeWriter.cpp:717–723](../../src/libslic3r/GCode/GCodeWriter.cpp#L717), `:799–807`; `Gantry.cpp:22`, `Api/host/GCodeReaderResultBuilder.cpp:277` |
| Layer temperature editing (2) | `idle_temperature`, `ooze_prevention` | `STEP_LAYER_EXTRUSION_EDIT` / `Ordering` | [DefaultTemperature.cpp:339–355](../../src/libslic3r/Plugins/LayerExtrusionEdit/DefaultTemperature.cpp#L339), setup reads `ooze_prevention` at `:772` and calls `standby_target` at `:858–863`; startup fallback at [DefaultTemperature.cpp:435–438](../../src/libslic3r/Plugins/LayerExtrusionEdit/DefaultTemperature.cpp#L435) |

The temperature and travel readers of the old `GCode/WipeTower.cpp` are not treated as an active prefix: [Print.cpp:1733–1815](../../src/libslic3r/Print.cpp#L1733) uses `WipeTower2`, where these reads are absent/commented out (`GCode/WipeTower2.cpp:1207–1213,1340`). The events produced by [SettingsGCodeScripts.cpp:220–255](../../src/libslic3r/Plugins/GCode/SettingsGCodeScripts.cpp#L220) copy their structural arguments, not these Print values.

**9 adhesion options kept at SkirtBrim / Slicing:** `complete_objects`, `draft_shield`, `min_skirt_length`, `skirt_brim`, `skirt_distance`, `skirt_distance_from_brim`, `skirt_height`, `skirt_extrusion_width`, `skirts`. Readers and declarations are in [DefaultSkirtGenerator.cpp:71–106](../../src/libslic3r/Plugins/SkirtBrim/DefaultSkirtGenerator.cpp#L71), `:209–254,426,560–642,771` and `Plugins/SkirtBrim/DefaultBrimGenerator.cpp:672`. The `complete_objects` reader in Ordering does not replace its earlier adhesion need.

**40 native MMU/wipe controls kept at SkirtBrim / Slicing:**

- `cooling_tube_retraction`, `cooling_tube_length`, `filament_max_wipe_tower_speed`, `filament_loading_speed`, `filament_loading_speed_start`, `filament_enable_toolchange_temp`, `filament_use_fast_skinnydip`, `filament_enable_toolchange_part_fan`;
- `filament_toolchange_part_fan_speed`, `filament_use_skinnydip`, `filament_melt_zone_pause`, `filament_cooling_zone_pause`, `filament_dip_insertion_speed`, `filament_dip_extraction_speed`, `filament_skinnydip_distance`, `filament_toolchange_delay`;
- `filament_cooling_moves`, `filament_cooling_initial_speed`, `filament_cooling_final_speed`, `filament_minimal_purge_on_wipe_tower`, `filament_multitool_ramming`, `filament_multitool_ramming_volume`, `filament_multitool_ramming_flow`, `high_current_on_filament_swap`;
- `parking_pos_retraction`, `extra_loading_move`, `priming_position`, `single_extruder_multi_material`, `wipe_tower`, `wipe_tower_speed`, `wipe_tower_extrusion_width`, `wiping_volumes_matrix`;
- `wipe_tower_brim_width`, `wipe_tower_rest_in_middle`, `wipe_tower_x`, `wipe_tower_y`, `wipe_tower_width`, `wipe_tower_rotation_angle`, `wipe_tower_cone_angle`, `wipe_tower_extra_spacing`.

The native suffix after run_slice is real: [Print.cpp:1748–1815](../../src/libslic3r/Print.cpp#L1748), `GCode/ToolOrdering.cpp:247`, [WipeTower2.cpp:65–77](../../src/libslic3r/GCode/WipeTower2.cpp#L65), `:131,209–224,479–538,793–845,939,1036–1067,1113–1301,1717`. The canonical runner [StepGenerateWipeTower.cpp:19](../../src/libslic3r/Steps/StepGenerateWipeTower.cpp#L19) is still empty: assigning these parameters to it as first consumer could keep invalid native outputs.

**10 keys kept because no active/canonical first reader is proven:** `filament_unloading_speed`, `filament_unloading_speed_start`, `filament_ramming_parameters`, `filament_toolchange_temp`, `wipe_tower_wipe_starting_speed`, `wiping_volumes_extruders`, `wipe_tower_no_sparse_layers`, `wipe_tower_bridging`, `wipe_tower_extruder`, `filament_type`. Old/preview readers are in [WipeTower.cpp:747–811](../../src/libslic3r/GCode/WipeTower.cpp#L747) and following; some WipeTower2 references are commented out. `filament_type` is also used in support analyses (`PrintObject.cpp:1085,1241`), without an established canonical first caller. UI/normalization readers alone do not prove a replay step.

`used_config_keys` still tightens the definitions ([Orchestrator.cpp:626–636](../../src/libslic3r/Api/host/Orchestrator.cpp#L626)); services are attached to their calling step by `plan_owner_step` (`:533–551`, seam → Ordering, firmware/reader → GCode), never compared as execution enums. None of the 67 selected keys is a seam definition.

The [routing and re-export tests](../../tests/plugins/test_plugin_config.cpp#L1257) cover each reclassified key and `skirt_distance`, the checkpoint, the whole requested suffix and the absence of a new slicing of the kept prefix. The [conservative routes test](../../tests/plugins/test_plugin_config.cpp#L1350) checks the 59 final definitions after plugin initialization and their Slicing checkpoint.

### Meaning of "const"

- **Contractual const**: plugins must treat the data as a read-only input. The C++ storage is not necessarily declared `const`.
- **C++ const**: member actually declared constant, specified when relevant, for example `PrintRegion::m_config`.
- **Const view**: forbids certain operations through this access, without freezing other aliases.
- **Mutable cache**: may be recomputed during a read without changing the business data; does not mean that all concurrent accesses are safe.

For generic extrusions, properties and geometries, the write right depends on the origin:

| Storage origin | Write window | Read-only from |
|---|---|---|
| Main slicing geometry | Slicing and post-slicing | `Slicing` |
| Source subtree produced later | Authorized producer/editor steps before finalization | `Print`, or earlier if the callback treats this output as an input |
| New auxiliary geometry | Construction and authorized processing before finalization | `Print`, with local island locking that may occur earlier |
| Working plan | Ordering then plan editing within the scope entrusted to the callback | `Plan` |
| Ordering snapshot | Host construction only | As soon as the save is published |
| Temporary plugin allocation | As long as its owner keeps it and no consumer requires its stability | No automatic pipeline boundary |

### Target write windows for source metadata

The last allowed steps below are inclusive. Owner edit gates use the single `check_freeze` helper, including extrusion roots, their buckets and idle-only configurations. Surfaces follow the structural rule described after the table.

| Storage | Last allowed step | Handling on replay |
|---|---|---|
| Properties of main layers, regions and islands | `STEP_POST_SLICING` | Frozen after `Slicing`, no snapshot |
| Properties of main objects | `STEP_FINALIZE_SLICING` | Snapshot at `None` and `Slicing` only |
| Properties of auxiliary layers and their regions/islands, of the global auxiliary object and of region-islands | `STEP_FINALIZE_SLICING` | Destroyed with the downstream nodes on a `None`/`Slicing` replay, no snapshot |
| Properties of source surfaces | Editing payloads allowed before `Print`, no global write | Replacement from scratch collections; destroyed with the downstream nodes, no snapshot |
| Records written by plugins | `STEP_FINALIZE_SLICING` | Snapshot at `None` and `Slicing`, always kept, no preference |
| Geometry and properties of source extrusions | `STEP_FINALIZE_SLICING` | Frozen at `Print`, no snapshot; downstream steps edit the plan clones |

### Inventory of source edit gates (5.5)

| Data-tree entry | Window / protected effect |
|---|---|
| `layer_island_get_slice_mutable` | Main: PostSlicing; auxiliary: FinalizeSlicing; geometry borrow |
| `object_get_config_mutable`, `print_get_config_mutable` | `STEP_NONE` only, or object without a Print; no configuration change before refusal |
| `layer_region_island_get_mutable_extrusion` | FinalizeSlicing inclusive; the only mutable root getter of the data tree, which may create the role bucket; the root gives access to the geometry **and** the extrusion properties/resources |
| `layer_island_get_region_island_mutable` | FinalizeSlicing inclusive; borrow of a mutable source bucket |
| `layer_island_get_or_create_region_island` | FinalizeSlicing inclusive; lookup/creation of the owner of extrusion buckets, refused before creation |
| `layer_recompute_slices_and_islands_from_layer_regions` | Main/aux slice window; enclosing rebuild that destroys the islands and their buckets/extrusions; refused before destruction |
| `layer_recompute_slices_from_islands`, `layer_add_regions_to_islands` | Main/aux slice window; refused before updating merged slices or entering native island-lock assertions; allowed calls still obey the native locking invariant |

There is no separate global getter for extrusion properties and no dedicated global function to remove/clear a role bucket in `slic3r_data_tree`. Auxiliary removal is already guarded by 5.4. Native publications and removals of the perimeter, surface, infill and post-infill producers/payloads remain in their allowed steps; keeping a callback beyond that scope is not supported. Other intermediate node borrows and cache/attachment helpers remain available, with their structural invariants; no general removal of mutators from 5.7 is done.

The generic `extrusion_*` APIs (children, flags, points/arcs/Z, copy/move, remove/clear), `extrusion_property_*` (payloads and resources) and `config_*` have no Print context. They remain editable for scratch data and plan clones through their mutable capabilities; no owner registry is added and previously distributed aliases are not revoked. Source reads remain const. The audit of production consumers finds no call to `config_mutable()` or to the two mutable getters to read a configuration: `config()`/const getters are already used. C layouts and signatures are unchanged, with no additional bump beyond the 4.0 contract.

Implemented API: `*_get_properties(const ...*)` returns a const container; `*_get_properties_mutable(...*)` provides writing within its window for object, layer, layer_region, layer_island and layer_region_island (`DATA_TREE` 4 contract). The C++ and Python views split `properties()` for read-only access and `mutable_properties()` for writing. Reads remain possible after the freeze.

Surface amendment: no owning `Print*` link, no global registry and no global `surface_get_properties_mutable`. Source surfaces are read through const handles. Their current edits go through the publication callbacks of the perimeter and surface-generation payloads: scratch assembly/annotation, then replacement in the source tree. DenseInfill already writes its hints into this scratch; no current consumer needs an additional in-place editing callback. `surface_collection_get_properties_mutable(collection_mutable, index)` provides the scratch write capability, and `MutableSurface` explicitly receives this capability together with its mutable handle. Scratch stays editable in Ordering/GCode; their payloads grant no capability to edit source surfaces. The layouts of existing payloads are unchanged. Previously borrowed mutable aliases are not revoked; malicious const_casts are not defended against.

Implemented mode `SLIC3R_STRICT_FREEZE` (CMake option, default `OFF`, private define of `libslic3r`, no plugin ABI change): `OFF` keeps the existing Debug assertions; `ON`, in Debug **and** Release, refuses before any mutation with `nullptr` or a failure status, registers a diagnostic and requests plugin cancellation, without a freeze assertion. The registry id and the text are kept in the `PluginMessage` queue, with `Print::running_step()` and the `<unknown>` plugin: these getters receive no plugin context. The existing setup/run barriers stop the execution. The C++ wrappers for auxiliary creation and typed property creation on a null capability raise an error before any assertion/dereference; the Python helpers already propagate null returns as None or an exception. `STEP_NONE` and nodes without a Print stay editable; surface scratch does not use this helper. The ON branch does not depend on NDEBUG; Debug OFF/ON is executed, Release is checked statically but was neither built nor executed.

Strict refusal diagnostics are limited to the first refusal per Print and step execution attempt, including a retry of the same step. Every refused call still requests cancellation and returns failure; checkpoint output cleanup and metadata restoration do not reopen the diagnostic claim. Messages use readable step names, including `STEP_NONE` for idle-only access. Python `PluginProperties.get_or_add()` raises `RuntimeError` for a null capability or refused native creation; optional reads and `get_mutable()` still return `None` when absent.

### Handles and invalidation

- Borrowed handles are generally opaque native pointers, with no generation and no lifetime extension. Copying a C++ view does not copy the data; `valid()` does not prove that a pointer is still alive.
- `vector<unique_ptr<T>>`: adding/moving the pointers does not move the `T` objects; removing an owner destroys its object. Indices and iterators are not stable.
- `vector<T>`: reallocation, insertion, erasure, move or replacement can invalidate handles to elements and sub-objects.
- Moving **contents** through a callback generally does not transfer the root allocation registered in the storage; the source must still be freed normally.
- The public contract may require reacquiring a view after any mutation, even if a particular operation currently preserves its address.
- Any borrow from a layer, an island or a plan must be dropped before its owner is destroyed/rebuilt. No old pointer automatically becomes a pointer to the restored clone.

Sources: `src/libslic3r/Api/host/slic3r_data_tree.cpp:35`, `src/libslic3r/DataTreeFwd.hpp:43`, `src/libslic3r/Api/plugin/cpp/DataTreeViews.hpp:12`.

## 2. Print roots and printable objects

### `Print` and its base

| Member | Ownership, access and lifetime | Modifiable when | Const / end of borrow |
|---|---|---|---|
| `PrintBase::m_model` | Owned model; reached indirectly through source volumes and objects | `apply` / host | Pipeline input; its descendants are invalidated by their replacement or by `Print::clear()` |
| `Print::m_objects` | Owner of the main `PrintObject`s; traversal via `print_get_object*` | Host creation/removal/reordering | Structure stable during an execution; no general API for creating a main object |
| `Print::m_auxiliary_object` | Hidden global object, absent from `m_objects`; lazily-creating getter | Auxiliary construction before `Print` by contract | `Print`; also destroyed by some `apply()` calls, not only by `clear()` |
| `Print::m_config` | Embedded configuration; handle to the live value | `apply` / mutable getter outside a step (`STEP_NONE`) | Must stay stable during the pipeline; mutable getter guarded according to the freeze mode |
| `Print::m_default_object_config` | Embedded value, used notably to build the auxiliary object | Host | No dedicated editing getter in the tree API |
| `Print::m_default_region_config` | Embedded value, completes the registered regions | Host, outside concurrent registrations | Input of registrations; not an output to restore |
| `Print::m_region_registry` | Owns the immutable `PrintRegion`s, configuration interning | Added during `apply` or API registration | Each published region is immutable; stays alive until the Print's clear/destructor |
| `Print::m_print_regions` | Non-owning catalog of active regions | Host rebuild on association/auxiliary changes | Rebuilding the catalog does not invalidate the regions owned by the registry |
| `Print::m_records` | Embedded `PrintRecordStore`; configurations per channel | See §8 | Channels destroyed individually or at the Print's clear |
| `Print::m_printing_plan` | Owns the live plan; provided to step contexts | Plan construction/editing | Contents const at `Plan`; replacement/restoration invalidates old views |
| `Print::m_ordering_checkpoint` | Host clone not exposed as an editable plan | Host capture/replacement | Immutable from capture; freed on invalidation or when its retention is disabled |
| `Print::m_gcode_reader_source_path` | Copy of the path published by the G-code callback | During export/publication/reset | Export result, not a frozen Print input; does not own the file |
| `Print::m_pipeline_state` | Owned step states (done, timestamp, warnings), copied by `step_state_with_warnings()` under `state_mutex()` | Worker transitions or stopped-worker invalidation; callbacks are notified outside the mutex | Cleared by `Print::clear()`; scheduling state, not checkpoint metadata to restore |

The pipeline-state timestamps use the shared `PrintStateBase` clock and never rewind on replay. A step becomes done only after its successful, non-cancelled execution and progress checks; replay retains prefix timestamps and retires suffix warnings until a successful retry removes those not reissued. `m_running_step` is atomic for freeze borrows, while its publication and visible-state transitions share `state_mutex()`. Control replay callers already hold that mutex and capture the affected running step before the stop callback can clear it; the worker replay path acquires the mutex without invoking the stop callback. FFF Print state readers and GUI completion gates use canonical pipeline snapshots; source-layer, perimeter, infill, support and adhesion previews wait for `STEP_FINALIZE_SLICING`, while wipe-tower and support-spot views use their respective pipeline steps. SLA readers remain unchanged.

The host active-warning facade publishes into the running pipeline step through the private `add_step_warning()` helper, including its timestamp and Print warning notification; delivery occurs after the state mutex is released. Outside execution (`STEP_NONE`), host warnings are transient orchestrator messages, not current checkpoint warnings: non-critical messages have Warning level, critical messages Error level, without implicitly requesting cancellation. A null plugin at `STEP_NONE` is attributed to `<host>`; a null plugin at a real step stays `<unknown>`, including strict freeze diagnostics, and named plugins keep their identity even during initialization. FFF legacy state setters, invalidators and pipeline mirrors are removed; `PrintObject` derives from `PrintObjectBase` without per-object execution state. Native tool-ordering/wipe-tower preparation in `Print::process()` retains its calculations without publishing `STEP_WIPETOWER` completion; that checkpoint becomes done only in the export pipeline. FFF task bounds do not filter objects or disable steps: `set_task()` only stops the worker under the state mutex, and `finalize()` has no state to re-enable.

FFF model/configuration invalidation uses checkpoint replay; its return value reports an affected completed or running step, not merely the presence of a configured route. `apply()` captures the entry running-step enum under its state mutex before any stop, then includes that step only when an actual requested replay suffix contains it. This preserves affected-running status across explicit pre-stops and earlier, shallower replay requests without retaining owner pointers or treating an already-pending prefix as newly invalidated. Changes affecting only pending steps report `APPLY_STATUS_CHANGED`, while affected calculated or running outputs report `APPLY_STATUS_INVALIDATED`. Instance shifts replay from the checkpoint before adhesion, with an additional wipe-tower request for count changes; initial instance construction has no published instances to invalidate. Plugin cleanup still runs outside the state mutex before owner destruction, and failed cleanup retains owners for retry. Successful cleanup of a pending-at-entry invalidation with processing history reports `APPLY_STATUS_INVALIDATED` even without a new input diff, so the worker restarts; fresh pending state alone does not force that status. Metadata and Ordering restoration remain deferred until pipeline resume.

`Print` does not have the `PluginPropertyContainer` of layers/objects; do not invent a `Print::properties`.

Sources: `src/libslic3r/Print.hpp:454`, `src/libslic3r/Print.cpp:192`, `src/libslic3r/Print.cpp:263`, `src/libslic3r/Print.cpp:308`, `src/libslic3r/PrintApply.cpp:1582`.

### `PrintObject` and `PrintInstance`

The owner is `Print::m_objects` or `Print::m_auxiliary_object`. Destroying the object invalidates all its descendants and its properties; its embedded configuration is not a plugin allocation.

| Member | Storage / exposure | Write | Const / lifetime |
|---|---|---|---|
| `PrintObjectBase::m_model_object` | Borrowed from the model; null for the global auxiliary object | Host | Input during pipeline; model replaced ⇒ old views invalid |
| `PrintObject::m_print` | Borrowed from the parent Print, used by host operations and cancellation | Construction | No ownership of the Print and no lifetime extension |
| `m_size` | Value; height used notably for the object without a model | Host construction | No C setter |
| `m_config` | Effective configuration, parented to the Print one | `apply` / mutable getter outside a step, or object without a Print | Input to keep stable; no automatic invalidation by the getter |
| `m_trafo` | Matrix copied by `object_get_transform` | Host | Read-only on the tree API side |
| `m_center_offset` | Point copied by getter | Host | Read-only on the API side |
| `m_instances` | `vector<PrintInstance>`; count and shifts copied | `apply`; temporary RAII replacement in the current brim | No general C mutator; indices not persistent |
| `m_shared_regions` | Shared ownership of slicing associations; C handles do not retain a reference to it | `apply` | Associations stable during pipeline, but replaceable between executions |
| `m_layer_profile` | Computed `[top_z,height]` pairs, replaced by the layer-height callback | `STEP_LAYER_HEIGHT` | Slicing input; kept at the `Slicing` checkpoint |
| `m_layers` | Separately allocated main layers | Rebuilt by `STEP_SLICING` | Structure kept at `Slicing`; working descendants still modifiable afterwards |
| `m_auxiliary_layers` | Separately allocated auxiliary layers; insertion by Z, API removal | Before `Print`, in the authorized context | Source freeze at `Print`; removing a layer does not destroy the others |
| Base `PluginPropertyContainer::m_properties` | Object metadata, not per instance | Up to and including `STEP_FINALIZE_SLICING` | Freeze helper (§1); snapshot of the main object at `None`/`Slicing` (§8) |
| `PrintInstance::print_object` | Borrowed from the owner | Host | No public native instance handle |
| `PrintInstance::model_instance` | Borrowed from the model; null for the synthetic auxiliary instance | Host | Same restriction |
| `PrintInstance::shift` | XY value; getter returns a copy | Host; temporary brim copy/restore | The returned copy belongs to the caller |

The input profile `ModelObject::layer_height_profile` and the computed profile `PrintObject::m_layer_profile` are two distinct storages. The temporary mutation of instances by the brim is not a checkpoint output; it must be restored even on exception.

Sources: `src/libslic3r/PrintObject.hpp:80`, `src/libslic3r/PrintObject.hpp:282`, `src/libslic3r/PrintObject.cpp:130`, `src/libslic3r/Plugins/SkirtBrim/DefaultBrimGenerator.cpp:404`.

## 3. Model, volumes, regions and slicing associations

These storages are **inputs for plugins**. Returned transformations are copied values; configs/meshes/associations are borrows. Do not save them as outputs in order to undo a new `apply()`.

### Model and volumes

| Member | API access or dependency | Write / const | Lifetime |
|---|---|---|---|
| `Model::m_objects` | Owner of the source objects | Host only | Model replacement/clear |
| `ModelObject::volumes` | Owner of the volumes traversed by `object_volume_at` | Host; plugin read | Removing/replacing a volume invalidates its views |
| `ModelObject::instances` | Origin of printable instances and transformation computations | Host | No direct C handle for a model instance |
| `ModelObject::name` | `object_get_name` copies the bytes | Host | The copied buffer no longer depends on the source string |
| `ModelObject::config` | Source configuration, notably extruder fallback | Host; indirect read | Do not confuse with `PrintObject::m_config` |
| `ModelObject::layer_config_ranges` | Owns the range configs | Host; layer-height/slicing read | Replacing a range/config invalidates its borrows |
| `ModelObject::layer_height_profile` / `LayerHeightProfile::m_data` | Input converted into a context buffer | Host; plugin read of the copy | Context copy bounded to the step/call |
| `ModelObject::{m_bounding_box_exact,m_min_max_z_valid}` | Cache used by the `object_get_max_z` query | Lazy host computation during read | Rebuildable cache, not mutable published data |
| `ObjectBase::m_id` of the volume | Copy via `volume_get_id` | Host construction | ID does not extend the volume's lifetime |
| `ModelVolume::object` | Borrowed from the ModelObject, used for fallback settings | Host construction | Depends on the parent |
| `ModelVolume::m_type` | Type returned by value | Host; no C setter | Volume lifetime |
| `ModelVolume::config` / `ModelConfig::m_data` | Borrowed source config | Host; constant getter | Lifetime/replacement of the source config |
| `ModelVolume::m_transformation` / `Transformation::m_matrix` | Matrices returned by copy | Host; no API source write | Self-contained copies |
| `ModelVolume::m_mesh` | `shared_ptr<const TriangleMesh>`, borrowed C handle | Host replacement; source mesh const | The handle does not retain the shared_ptr |
| `TriangleMesh::its` | Contains the indexed mesh | Source const | Mesh lifetime |
| `indexed_triangle_set::{vertices,indices}` | Vertices and triangles read by copy | Source const; separate working copies in §10 | No mutable source buffer provided |
| `storage_new_triangle_mesh` handle | Owned `TriangleMesh` copied from plugin vertices and triangles, read by the `triangle_mesh_*` functions | Plugin fills the arrays before the call; host read-only afterwards | Until `storage_free()` or `storage_clear()`; the input arrays are not retained |
| `ModelVolume::{supported_facets,mm_segmentation_facets,m_generic_facets_annotations}` | Annotations queried/projected by key | Host; no annotation editing through this API | Lifetime of the volume's annotations |
| `FacetsAnnotation::m_data` | Facet encoding consumed by projections | Host; no raw exposure | Projection results copied to a separate destination |

`object_get_max_z` is not always a pure local height: for a model object, it uses the model's transformed max Z; for the auxiliary object without a model, it returns the object's native height.

Sources: `src/libslic3r/Model.hpp:358`, `src/libslic3r/Model.hpp:844`, `src/libslic3r/Model.hpp:994`, `src/libslic3r/Api/host/slic3r_volume.cpp:373`, `src/libslic3r/Api/host/slic3r_volume.cpp:726`, `src/libslic3r/Api/host/slic3r_data_tree.cpp:781`.

### Canonical regions and associations

| Member | Ownership / view | Write / transition to const | Invalidation |
|---|---|---|---|
| `PrintRegion::m_config` | Owned complete regional config, without parent | **C++ const from construction** | Region kept by the registry until the Print's clear |
| `PrintRegion::m_config_hash` | Internal interner identity, not a C getter | **C++ const from construction** | Same owner |
| `PrintObjectRegions::all_regions` | Vector of borrows into the registry; local indices | `apply` then read input | Reconfiguration possible without destroying the old canonical regions |
| `PrintObjectRegions::layer_ranges` | Owned `LayerRangeRegions` values | `apply` then read | Reallocation/rebuild invalidates range handles |
| `LayerRangeRegions::layer_height_range_` | Z bounds copied by getters | `apply` then read | Self-contained scalar copy |
| `LayerRangeRegions::config` | Borrow to a model range config, nullable | `apply` then read | Lifetime of the target configuration |
| `LayerRangeRegions::volumes` | Owned `VolumeExtents` values; parent of the referenced boxes | `apply` then indirect read | Rebuild invalidates box pointers |
| `VolumeExtents::bbox` | Box copied by the association getter | `apply` then read | Self-contained copy |
| `LayerRangeRegions::volume_regions` | Owned `VolumeRegion` values | `apply` then read | Rebuild invalidates association handles |
| `VolumeRegion::model_volume` | Source borrow | `apply` then read | Model volume lifetime |
| `VolumeRegion::parent` | Local index of the parent association | `apply` then read | Is neither a volume ID nor a LayerRegion index |
| `VolumeRegion::region` | Borrow to PrintRegion, nullable | `apply` then indirect read | Registry target, replaceable association |
| `VolumeRegion::bbox` | Borrowed from `VolumeExtents::bbox` | `apply` then copied read | Also depends on the `volumes` vector |
| `VolumeRegion::region_idx` | Dense index into `all_regions` | `apply` then read | Valid for these local associations only |

`apply()` can rewire `LayerRegion::m_region` without replacing its geometry. Restoring an old full copy of LayerRegion could therefore undo a valid current configuration. Painted associations, `prev_same_region`, volume caches and internal counters are not exposed as editable views of this API.

Sources: `src/libslic3r/PrintRegion.hpp:65`, `src/libslic3r/PrintObjectRegion.hpp:54`, `src/libslic3r/Api/host/slic3r_volume.cpp:302`, `src/libslic3r/PrintApply.cpp:1529`.

## 4. Layers, regions, islands and surfaces

### `Layer`

Owner: `PrintObject::m_layers` or `m_auxiliary_layers`. Regions and islands are allocated separately; rebuilding their collections can destroy them without destroying the layer.

| Member | Storage / access | Modifiable when | Const from / validity |
|---|---|---|---|
| `m_height` | Height, getter by copy | Construction | No C setter; fixed for the instance |
| `m_print_z` | Print Z, copy | Construction | Fixed for the instance |
| `slice_z` | Native slicing Z, double | Construction | No direct getter of this member in `layer_get_slice_z` |
| `m_id` | Native index, used indirectly for flows | Host | No layer ID exposed by this ABI |
| `m_object` | Borrowed parent, getter | Construction | Fixed association; the parent does not keep a deleted child alive |
| `upper_layer` | Borrowed neighbor, const/mutable getters | Host chaining | Main topology preserved from `Slicing` |
| `lower_layer` | Borrowed neighbor, const/mutable getters | Host chaining | Same rule; auxiliaries created by the API initially unlinked |
| `m_regions` | Owner of LayerRegion | Host construction/reset | Main structure preserved from `Slicing` |
| `m_lslices` | Merged ExPolygons, const/mutable borrow | Main: slicing/post-slicing; auxiliary: construction allowed | Main: `Slicing`; auxiliary: `Print` at the latest, with local locking |
| `m_islands` | Owner of LayerSliceIsland | Rebuild/assignment before locking | Main: `Slicing`; rebuild destroys the old islands |
| `m_islands_locked` | Internal guard, no public getter | Set by `add_regions_to_islands()` | Rebuild assertions; no transitive freeze of descendants |
| `curled_lines` | `vector<CurledLine>`, access by copies | Host analysis producer, no C setter | Input for consumers; do not assume a migrated producer |
| Base `m_properties` | PluginPropertyContainer | Main up to `STEP_POST_SLICING`, auxiliary up to `STEP_FINALIZE_SLICING`, inclusive | Freeze helper (§1), no snapshot (§8) |
| `CurledLine::a`, `CurledLine::b` inherited from `Line` | Points copied into `c_curled_line` | Host producer | Independent copies |
| `CurledLine::curled_height` | Copied height | Host producer | Independent copy |

`layer_get_slice_z()` computes `print_z - height / 2`, and does not read `Layer::slice_z`: the two values can differ for an auxiliary created with an explicit slicing plan.

Sources: `src/libslic3r/Layer.hpp:126`, `src/libslic3r/Layer.cpp:45`, `src/libslic3r/Layer.cpp:259`, `src/libslic3r/Api/host/slic3r_data_tree.cpp:364`.

### `LayerRegion`

| Member | Storage / access | Modifiable when | Const / lifetime |
|---|---|---|---|
| `m_layer` | Borrow of the parent layer | Construction | Fixed association |
| `m_region` | Borrow of the canonical region | Construction and reattachment by `apply` | Input during the pipeline, target truly const |
| `m_raw_slices` | Embedded ExPolygons; this is what `layer_region_get_slices` returns | Main: slicing/post-slicing; auxiliary: construction allowed | `Slicing` for the main one, `Print` at the latest for auxiliary |
| Base `m_properties` | Layer region metadata | Main up to `STEP_POST_SLICING`, auxiliary up to `STEP_FINALIZE_SLICING`, inclusive | Freeze helper (§1), no snapshot |
| `m_cache_fill_expolygons`, `m_cache_fill_expolygons_computed` | Native cache, no direct C getter | Host computation/invalidation | Recomputable, not a domain snapshot |
| `m_slices`, `m_fill_surfaces`, `m_fill_no_overlap_expolygons`, `m_unsupported_bridge_edges` | Legacy native storage, **not the surfaces exposed by the region-island API** | Relevant native paths | Do not attribute the C getters of the new tree to them |
| `surfaces_filled` in Debug | Native flag | Host attachment | May be true with empty processed surface collections |

The box and flows returned by the getters are computed and copied: there is no borrowed `LayerRegion::m_bbox` or `Flow` member to keep.

Sources: `src/libslic3r/LayerRegion.hpp:73`, `src/libslic3r/LayerRegion.hpp:123`, `src/libslic3r/Api/host/slic3r_data_tree.cpp:469`.

### `LayerSliceIsland` — `LayerIsland` in the API

Owner: `Layer::m_islands`. **Freezing `m_slice` does not freeze `m_extrusions`.**

| Member | Storage / access | Modifiable when | Const / lifetime |
|---|---|---|---|
| `m_slice` | ExPolygon defining the island; const and mutable getter | Main: up to post-slicing; auxiliary: construction allowed | Main: `Slicing`; auxiliary: `Print`; mutable getter guarded according to the freeze mode |
| `m_bbox` | XY cache built from the polygon; copy getter | Island construction | Stable with the geometry; editing `m_slice` does not recompute it automatically |
| `m_regions` | Set of borrows to LayerRegion; target getters | Region attachment | Main topology fixed from `Slicing`; a mutable target getter does not allow changing the set |
| `m_layer` | Borrowed parent, initially null | `fill_regions()` | Fixed after attachment; may be null during construction |
| `m_extrusions` | **Vector of owners of LayerRegionIsland**, not ExtrusionEntity | Perimeters, surfaces, groupings and post-infill allowed | Source frozen from `Print`; removing a group invalidates its descendants |
| `m_infill_areas` | Anchoring ExPolygons, const getters | Perimeter production, post-perimeter replacement | Input of surfaces/infill; global `Print` freeze |
| `m_infill_free_areas` | ExPolygons without overlap, const getters | Same production | Same freeze; empty may mean no restriction |
| `m_infill_areas_bboxes` | Native cache recomputed with the areas | Host setter of the areas | Not directly exposed; the C infill bbox getter computes the first area |
| `m_perimeter_slices` | Native derived contours, no dedicated C getter | Perimeter production | Derived source output, global `Print` freeze |
| `m_support_demand` | ExPolygons asking for support; const getter `layer_island_get_support_demand()` | `STEP_SUPPORT_DEMAND` builds it; any plugin from `STEP_SUPPORT_DEMAND` through `STEP_SUPPORT` may refine it (bridge removal in `STEP_POST_PERIMETER`, bridged infill removal in `STEP_POST_INFILL`) with `support_demand_get_island_mutable(run_ctx, island)`, which checks the running step | Emptied by the `STEP_SUPPORT_DEMAND` cleanup at a `Slicing` replay; read by support generation |
| `overlaps_above` | Vector of links borrowing the upper islands | Final post-slicing rebuild | Main topology fixed from `Slicing` |
| `overlaps_below` | Links to the layer below | Same phase | Same rule |
| `Link::to` | Pointer to another island, exposed | Graph construction | Invalid if the target is rebuilt |
| `Link::area` | Native area, no C getter | Graph construction | Same topology; the API exposes the neighbors, not this area |
| Base `m_properties` | Island metadata | Main up to `STEP_POST_SLICING`, auxiliary up to `STEP_FINALIZE_SLICING`, inclusive | Freeze helper (§1), no snapshot |

Adding a group does not move separately allocated groups. Perimeter preparation currently destroys all of them; the surface/post-infill passes can also remove empty groups after a callback. Do not keep a group merely because its parent island still exists.

Sources: `src/libslic3r/Layer.hpp:41`, `src/libslic3r/Layer.cpp:128`, `src/libslic3r/Layer.cpp:157`, `src/libslic3r/Api/host/slic3r_data_tree.cpp:510`, `src/libslic3r/Steps/StepSurfaceGeneration.cpp:171`.

### `LayerRegionIsland`

| Member | Storage / access | Write | Const / invalidation |
|---|---|---|---|
| `m_extruder_id` | Zero-based or unknown identity, copied | Group construction | No C setter; regrouping via another instance |
| `m_regions` | Set of borrows to LayerRegion | Construction | Constant associations for the published group |
| `m_fill_surfaces` | Owned SurfaceCollection, general const getter | Surface production/refinement by callbacks | Input for infill; post-infill must not modify it; global `Print` freeze |
| `m_extrusion_regions` | Map role → owned ExtrusionEntityCollection root | Producing/editing steps; mutable getter can create the bucket | `Print`; clearing a bucket/group invalidates its handles |
| Base `m_properties` | Group metadata | Up to and including `STEP_FINALIZE_SLICING` | Freeze helper (§1) through the owning regions; outside a Print: no phase guard; no snapshot |

Adding another `std::map` bucket does not move existing roots. `remove_empty_extrusions()` can remove an empty root; `has_extrusion()` means non-empty content, not the mere presence of a key. The generic getters are less restrictive than some step callbacks that filter editable roles.

Sources: `src/libslic3r/LayerRegion.hpp:170`, `src/libslic3r/LayerRegion.cpp:61`, `src/libslic3r/Api/host/slic3r_data_tree.cpp:674`.

### `SurfaceCollection` and `Surface`

A collection is either temporary in the storage, or embedded in a region-island. `SurfaceCollection::surfaces` is a `vector<Surface>` by value: append, replacement, clearing and moving can invalidate views of surfaces, geometries and properties.

| Member | Exposure / write | Const / lifetime |
|---|---|---|
| `SurfaceCollection::surfaces` | Create/append/clear on a mutable collection; publication by moving the content | Published source collection then read by consumers; the source handle stays with its storage |
| `Surface::m_id` | Value getter; construction/copy yields a new ID, move transfers it, copy assignment keeps the destination ID | No C setter; not a persistent identity across copies |
| `Surface::surface_type` | Chosen at append, read-only getters/flags | No direct C setter on an existing surface |
| `stModSupport` / `RAW_SURFACE_TYPE_MOD_SUPPORT` | Modifier bit identifying support auxiliary-layer surfaces; combines with position and density for bottom interface, sparse base and top interface | Type remains read-only after publication |
| `stModContact` / `RAW_SURFACE_TYPE_MOD_CONTACT` | Modifier bit identifying the dense areas of a support contact layer (under the object for a top surface, resting on it for a bottom surface): the area that touches the object above or below and the margin of that contact, that is the dense area of the contact layer around it; bridging bottom contacts carry it. A dense support surface without it is an interface; a contact merged into the base is sparse base without it | Type remains read-only after publication |
| `Surface::expolygon` | Owned geometry; constant C getter | Replacement by publishing new surfaces, not generic geometric editing via MutableSurface |
| `Surface::m_thickness` | Copied indirectly by the native callback `append_surface_like` | No individual C getter/setter |
| `Surface::thickness_layers` | Same indirect copy | Same restriction |
| `Surface::bridge_angle` | Same indirect copy | Same restriction |
| `Surface::extra_perimeters` | Same indirect copy | Same restriction |
| `Surface::maxNbSolidLayersOnTop` | Same indirect copy | Same restriction |
| `Surface::priority` | Same indirect copy | Same restriction |
| Base `Surface::m_properties` | Const source getter; explicit scratch capability | Source: editing by publication in the allowed payloads; scratch: editable in any phase, no snapshot |
| `c_surface::{type,id}` | Copied values | Self-contained |
| `c_surface::expolygon` | Borrowed handle | Still depends on the source surface |

Dense top or bottom support surfaces without the contact modifier are interfaces of their side. On a side without interface layers, interfaces and contacts both use the base settings and bucket; the contact bit selects contact patterns and loops, while the bridge bit independently selects bridge flow. A contact provider may hand an area back to its interface provider by republishing the surface with only contact cleared, preserving geometry, order, properties and other flags. Support packages built against step ABI 1.2 do not mark contacts, so their contact areas are classified as interfaces: they use interface settings when enabled on that side, otherwise base settings and bucket without contact handling.

Sparse support includes base slabs and tree branch bodies; bit 10 is reserved in data-tree ABI 5.0.
Legacy contact providers hand components back when no contact-specific setting is active, when effective contact and interface patterns agree without active contact loops, or when the component is narrower than one extrusion width. Active top-contact loops keep wide contacts and their loops even with equal patterns; narrow components are still handed back. Equal-pattern bottom bridges can still be handed back while retaining bridge flow.
Support-toolpath ABI 2.0 has five roles: sparse, bottom contact, bottom interface, top contact and top interface. Each object's
automatic selector follows its support family's style preference, unless the object chooses an explicit
provider. `support.toolpath.branch_sheath` is a sparse provider preferred by organic support, not a separate role.

`MutableSurface` currently offers neither a type setter nor mutable geometry. The C++ helper `StoredSurfaceCollection::append_like()` copies type/geometry/plugin properties, but is not equivalent to the native callback, which also copies the hidden fields. `StoredSurface::{m_expolygon,m_type}` is a temporary geometry helper, not a native Surface allocation; its synthetic view has a null ID.

Sources: `src/libslic3r/Surface.hpp:89`, `src/libslic3r/SurfaceCollection.hpp:20`, `src/libslic3r/Steps/StepSurfaceGeneration.cpp:142`, `src/libslic3r/Api/plugin/cpp/SurfaceViews.hpp:298`.

## 5. Printing plan and catalog

### Common ownership and phases

The plan owns its hierarchy, its extrusion copies, its events and its catalog values. It **borrows** the source objects, layers and region-islands. These must stay alive as long as the live plan or its snapshot references them.

All the working members below are editable only before `Plan`, in an allowed phase. Parallel per-layer editing can neither restructure the plan/group vectors nor modify another layer or the shared catalog; the serialized setup can prepare global data and resolve extruders. The mutable handles of the context do not lift these restrictions.

### Hierarchy members

| Class and member | Ownership / editing | End of validity / const |
|---|---|---|
| `PrintingPlan::groups` | Vector of PrintingGroup by value; structural append/reorder/clear | Handles invalidated by vector changes; `Plan` freeze |
| `PrintingPlan::object_instances` | Catalog owned by value; append then metadata | Reallocation/clear/replacement invalidate entries; `Plan` freeze |
| `PrintingPlan::events` | Embedded scope events | Lifetime of the plan, with possible child clearing before `Plan` |
| `PrintingPlan::properties` | Owned metadata, separate const/mutable getters | API freeze from `Plan` |
| `PrintingPlan::identity` | Unique owner of an empty `PrintingPlanIdentity` token | Token not editable; new on clone, kept by `printing_plan_clear()` |
| `PrintingGroup::plan_identity` | Borrow of the parent token, initialized by host | Not a guarantee of a handle's temporal validity |
| `PrintingGroup::events` | Owned scope | `Plan` freeze; follows group moves |
| `PrintingGroup::properties` | Owned metadata | `Plan` freeze |
| `PrintingGroup::printing_catalog_ids` | Index values into the catalog | `Plan` freeze; IDs local to the plan |
| `PrintingGroup::layers` | Vector of PrintingLayerGroup by value | Invalidating structural changes; `Plan` freeze |
| `PrintingLayerGroup::plan_identity` | Borrow of the token | Lifetime of the plan |
| `PrintingLayerGroup::events` | Layer-local scope | Local editing allowed, then `Plan` freeze |
| `PrintingLayerGroup::properties` | Local metadata, notably time/input state | Same window and concurrency restrictions |
| `PrintingLayerGroup::print_z` | Plan schedule value, getter/setter | Structural editing allowed, not arbitrary rescheduling in a parallel pass; `Plan` freeze |
| `PrintingLayerGroup::layers` | Owned vector of borrows to the source Layers | List built before `Plan` freeze; targets frozen from `Print` |
| `PrintingLayerGroup::tool_groups` | Vector of PrintingToolGroup by value | Invalidating structural mutations; `Plan` freeze |
| `PrintingToolGroup::plan_identity` | Borrow of the token | Lifetime of the plan |
| `PrintingToolGroup::events` | Tool visit scope | `Plan` freeze |
| `PrintingToolGroup::properties` | Visit metadata | `Plan` freeze |
| `PrintingToolGroup::extruder_id` | Scalar, notably resolved in serialized setup | `Plan` freeze |
| `PrintingToolGroup::region_islands` | Vector of source borrows | `Plan` freeze of the list; sources frozen from `Print` |
| `PrintingToolGroup::extrusions` | Vector of PrintingExtrusion by value | Append/reorder/transfer invalidate descriptors; `Plan` freeze |
| `PrintingExtrusion::region_island` | Borrow of the source context, nullable | Fixed on addition; no dedicated API setter |
| `PrintingExtrusion::sregion_island_role` | Provenance role, distinct from the clone's role attribute | Getter/setter before `Plan` |
| `PrintingExtrusion::root` | Unique owner of a tree copy | Replacement destroys the old root/descendants; `Plan` freeze |
| `PrintingExtrusion::printing_catalog_id` | Local ID, fixed on addition | No dedicated setter; not a model ID |

`PrintingExtrusion` has no `properties` member: its extrusion metadata lives on `root` and its descendants. Moving the root unique_ptr can keep the tree address, but does not stabilize the moved PrintingExtrusion descriptor in its vector.

Sources: `src/libslic3r/Printing/PrintingPlan.hpp:134`, `src/libslic3r/Printing/PrintingPlan.hpp:165`, `src/libslic3r/Api/plugin/c/steps/slic3r_step_layer_extrusion_edit.h:18`.

### Catalog and events

| Class and member | Ownership / mutation | Const / invalidation |
|---|---|---|
| `PrintingObjectInstance::object` | Source borrow, null if synthetic | Fixed at registration; source not owned |
| `PrintingObjectInstance::object_instance_idx` | Copied source index | Fixed at registration, not stable across a source rebuild |
| `PrintingObjectInstance::name` | Owned string, metadata setter | `Plan` freeze; C pointer invalidated by replacement/reallocation |
| `PrintingObjectInstance::projected_outline` | Owned Polygon, independent of the model | Copied at publication; `Plan` freeze |
| `PrintingObjectInstance::projected_outline_center` | Point computed/copied by getter | `Plan` freeze |
| `c_printing_object_instance::id` | Copied catalog index, not an extra native member | Valid in this catalog only |
| Other fields of `c_printing_object_instance` | Copied scalars; `object`, `name`, `projected_outline` remain borrowed | Copying the descriptor does not copy text/geometry |
| `PrintingScopeEvents::m_before` | Embedded root; const root API, append of mutable children | `Plan` freeze; children destroyed by clear |
| `PrintingScopeEvents::m_after` | Same model after the content | Same rule |

The C++ catalog wrapper copies the descriptor; it is its `name()` call that copies the text. Scope roots must be non-sortable/non-reversible by contract; do not attach arbitrary data to them by bypassing const access.

An explicit call to `StepExtrusionEdition::clear()` clears the children of the scope events. The pipeline no longer does this when preparing this step: replay replaces the working plan, without clearing events newly produced by previous steps.

Sources: `src/libslic3r/Printing/PrintingPlan.hpp:46`, `src/libslic3r/Printing/PrintingPlan.hpp:111`, `src/libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp:80`, `src/libslic3r/Steps/StepExtrusionEdition.cpp:31`.

### Snapshot and coordinates

`PrintingPlan::clone()` copies the catalog, properties and owned trees, rewrites the identity tokens, but keeps the source borrows. Restoring Ordering replaces the live plan with a new clone: all old plan handles must be abandoned. The retained snapshot is not consumed.

Printable extrusions are expected in bed coordinates. The builder clones then applies the instance shift; the low-level append only clones what it is given. The copy is not a promise of automatic translation of all geometry passed to the API.

Sources: `src/libslic3r/Printing/PrintingPlan.cpp:472`, `src/libslic3r/Print.cpp:233`, `src/libslic3r/Plugins/Ordering/DefaultPlanBuilder.cpp:508`.

## 6. Extrusions, geometry and extrusion properties

### `ExtrusionEntity` and subtypes

Write window: **source before `Print`**, **plan copy before `Plan`**, **temporary while owned**, always depending on the callback. Generic mutators do not know which origin the handle has.

| Member | Storage and access | Invalidation / read-only |
|---|---|---|
| `ExtrusionEntity::m_id` | Internal native identity, copied during clones | No general C ID for the entity; clone does not mean new global ID |
| `m_can_reverse` | Permission exposed indirectly through flags | Modifiable within the owner's window |
| `m_can_sort` | Sort permission | Same window; some content changes also change it |
| `m_content` | Variant: empty, `ArcPolyline`, or `Children` | Replacement destroys the previous alternative |
| `Children` alternative | `vector<unique_ptr<ExtrusionEntity>>` | Adding/moving a pointer preserves the objects; removal destroys the subtree |
| `ExtrusionPropertyContainer` base | Owned properties and resources | Same window as the node, detailed rules below |
| `ExtrusionEntityCollection::m_entities_cache` | Non-owning cache of child pointers | Rebuilt by native read, not a second owner |
| `ExtrusionNop::m_role` | Legacy native role | No specialized setter in the unified ABI |
| `ExtrusionNop::position` | Legacy native position | Distinct from an ArcPolyline |

`ExtrusionPath`, `ExtrusionMultiEntity`, `ExtrusionMultiPath` and `ExtrusionLoop` use the inherited content and properties; do not invent `m_children`, `m_polyline`, `m_attributes` or `m_loop_role` as current separate storages. Attributes and loop role are properties.

Copy/move into a root keeps its address but replaces its previous contents; handles to descendants/properties must be reacquired. `clear_content()` does not remove properties: it removes geometry/children and currently resets `m_can_sort` to false. Moving a child transfers its unique_ptr; moving the content of a storage root does not transfer its allocation record.

Sources: `src/libslic3r/ExtrusionEntity.hpp:63`, `src/libslic3r/ExtrusionEntityCollection.hpp:34`, `src/libslic3r/ExtrusionEntity.cpp:419`, `src/libslic3r/Api/host/slic3r_extrusion_entity.cpp:68`.

### `ArcPolyline` and segments

The ArcPolyline is embedded in `m_content`, not a generic Polyline to be freed separately.

| Member | Write / exposure | Lifetime and const |
|---|---|---|
| `ArcPolyline::m_path` | Vector of segments, geometric setters | Owner node's window; disappears when content is replaced |
| `ArcPolyline::m_z_offset` | Owned optional vector, one offset per point | Allocation/resize/reset aligned with the geometry |
| `ArcPolyline::m_only_strait` | Native cache, actual spelling | Recomputed by operations, not a public payload |
| `ArcPolyline::is_3D` in Debug | Internal flag | No public getter/setter |
| `ArcWelder::Segment::point` | Endpoint of the incoming segment | Value copied in getters |
| `ArcWelder::Segment::radius` | Signed radius, zero for a straight line | Same window as m_path |
| `ArcWelder::Segment::orientation` | Arc direction | Same window |
| `ArcWelder::Segment::{length,center}` in Debug | Internal caches | No additional public storage |
| `c_extrusion_segment::{point_a,point_b,radius,orientation,z_offset_a,z_offset_b}` | Value copies of two native points/segment | Belong to the requester, no borrow of a native buffer |

`set_points` replaces arcs and Z offsets; modifying a point may linearize adjacent arcs. Geometric splits into other entities do not automatically copy flags/properties. Freeze follows the owner, not the ArcPolyline type.

Sources: `src/libslic3r/Polyline.hpp:234`, `src/libslic3r/Geometry/ArcWelder.hpp:442`, `src/libslic3r/Api/host/slic3r_extrusion_polyline.cpp:247`.

### Extrusion properties and resources

| Member | Ownership / mutation | Validity |
|---|---|---|
| `ExtrusionPropertyContainer::m_properties` | Direct slots, one payload per type | Removal/replacement/clear/copy-over invalidate the affected payloads |
| `m_data_resources` | Auxiliary buffers owned by the entity | Explicit release or via owner property/field; replacement/destruction of the entity |
| `m_next_data_resource_id` | Counter local to the container | Copied with the container; no global identity |
| `DataResource::id` | Local ID | To be interpreted with its owner entity |
| `DataResource::owner_type` | Owner type, or invalid for independent storage | Internal association, not an ownership pointer |
| `DataResource::owner_field_offset` | Offset of the owner ID field, or sentinel | Used to replace a field's resources |
| `DataResource::data` | Owned PropertyRawBuffer | Borrowed, constant data getter |

A full container copy copies slots **and** resources. `clone_properties()` alone only copies the slots. Copying a `text_id` ID into another entity does not copy its text. `clear_properties()` releases resources attached to the removed properties, but not necessarily independent resources. `extrusion_free_data()` does not fix up fields still holding the released ID.

Sources: `src/libslic3r/ExtrusionProperty.hpp:371`, `src/libslic3r/ExtrusionProperty.hpp:463`, `src/libslic3r/ExtrusionProperty.cpp:113`, `src/libslic3r/ExtrusionProperty.cpp:288`.

### Built-in payload fields

All follow the window of the node carrying them. Field lists group values having exactly the same lifetime; IDs do not own their targets.

| C payload | Members |
|---|---|
| `c_extrusion_property_attributes` | `mm3_per_mm`, `width`, `height`, `role`, `no_seam` |
| `c_extrusion_property_speed` | `speed_mm_per_s`, `accel_mm_per_s2`, `pressure_adv`, `fan_speed_percent` |
| `c_extrusion_property_modifier` | `enforce_travel`, `enforce_retraction`, `enforce_unlift`, `disable_retraction`, `disable_lift`, `toolchange_retraction` |
| `c_extrusion_property_custom_gcode` | `kind`, `script_type`, `processing_extruder_id`; `text_id`, `config_id` reference buffers local to the entity, not live Config handles |
| `c_extrusion_property_special_command` | `code`, `extra_data` |
| `c_extrusion_property_overhang` | `start_distance_from_prev_layer`, `end_distance_from_prev_layer`, `proximity_to_curled_lines`, `has_full_overhangs_flow`, `has_full_overhangs_speed`, `has_dynamic_overhangs_flow`, `has_dynamic_overhangs_speed` |
| `c_extrusion_property_z_offset` | `z_offset` |
| `c_extrusion_property_perimeter` | `perimeter_idx`, `reserved`, `perimeter_flags` |
| `c_extrusion_property_infill` | `source_surface_id`, numeric reference without ownership of the Surface |
| `c_extrusion_property_extrusion_axis` | `operation`, `value`, `restart_extra`, `toolchange` |
| `c_extrusion_property_heater` | `heater`, `tool`, `temperature_C`, `wait`; entry event, not inherited |

Plugin-registered payloads are open: copying them preserves all bytes, not only the known built-in fields. Examples used in the plan: time (`duration_seconds`, `origin`), input (`x`, `y`, `z`, `state`, and tool payload `extruder_id`), endpoints (`entry`, `exit`), scope marker (`flags`). They follow the plan's freeze; they are not new geometry owners.

Sources: `src/libslic3r/Api/plugin/c/slic3r_extrusion_property.h:237`, `src/libslic3r/Plugins/PrintingPlan/PrintingLayerTimeProperty.h:33`, `src/libslic3r/Plugins/PrintingPlan/EntryPointProperty.h:13`.

## 7. Generic geometry and plugin allocations

### `PluginStorage`

The storage provided per context belongs to `Orchestrator::m_plugin_storage` and survives callbacks. Parallel calls of the same plugin share it. A central mutex protects allocation registration, individual release, membership/count queries and clearing. Object addresses remain stable across registry growth; accessing an existing object through its handle does not take the storage lock. Plugins must coordinate edits of the same object and finish reading a handle before freeing it. Do not call `storage_clear()` during `setup_run()` or `run()`, or assert that the shared storage is empty: other callbacks may still have live allocations. Temporary owners release their allocations individually. A worker storage provided by `slic3r_parallel_for_storage`, by contrast, is local to the job and destroyed when it returns.

Plugin instances are singletons: `plugin_ctx` and this storage designate the plugin, not a particular run or a Print. The `plugin_run_context::print` member identifies the Print for setup/setup_run/run; the optional callback `clear(plugin_ctx, print, plugin_storage)` receives the same key directly. A run-dependent cache must be partitioned by this identity, not by the temporary address of the context.

Participant tracking belongs to the Print. Their presence does not mean their caches are invalid: a separate atomic flag signals cleanups caused by an invalidation or a failure. An unchanged apply does not traverse this tracking and does not cancel a valid worker; tracking reads needed for cleanup happen after the workers have stopped. An apply retry after a failed cleanup is still treated as an invalidation, even if the inputs have already been applied.

### Independent host executions

For each independent print, the host creates an `Orchestrator::RunContext`.
It holds the progress counters, stop flag and first known stop cause.
The host keeps both this context and the scope returned by `enter_run(context, &print)`
alive until all execution workers have finished.

A `plugin_host_context` carries a borrowed pointer to that context. Copying the
pointer does not extend its lifetime. Plugin wrappers and C parallel loops associate
it with each worker thread for the duration of a callback. A check with an explicit
`Print` selects that Print's registered context. A check without a `Print` selects
the calling thread's context. If that lookup finds none, it uses the shared stop state.
Without an execution context, native C++ plugins use their own progress counters.

`PLUGIN_RUN_CONTEXT` 2.1 keeps the 2.0 prefix and places `progress_context` after it.
Native `PluginBase` callbacks use these host-owned counters when supplied.
Returning from a nested callback restores the previous counter selection.
On the callback thread, obtain `PluginProgress &counters = progress()` before starting
other threads. Pass that reference to them and wait for them before returning.

Older native plugin binaries and Python's `PluginProgress` helper do not use these
C++ counters. This mechanism does not make concurrent calls to those helpers safe.

Ending an execution scope does not replace or clear `plugin_storage`.
Its allocations remain available to `clear(print, storage)` afterwards.
Print calls this cleanup when replaying from a saved pipeline boundary, called a
checkpoint, or during destruction. Plugins must still separate caches by Print
and protect access to shared mutable data. RunScope does not protect those caches.

### Planned rework

Replace the central storage lock with private storage per executed element (object, layer location or sub-plugin call), shared by that element's `setup_run()` and `run()`. Keep persistent storage for `setup()`/`clear()` and print-wide calls: G-code firmware sessions retain a config allocated during their factory `run()` and use it later while writing the print. A future per-object pipeline must prevent object steps from writing print-wide data (`print_records_*` or the print's auxiliary object). The current `setup_run()` discovery barrier across objects must also be reviewed before changing that scheduling.

### Scheduling ownership and invalidation barriers

The set of steps to execute, the progress, and the snapshot pointers are not concurrent storages. The legacy milestone lock is not enough to protect the worker's reads of this data: the control thread must wait for the worker to stop before modifying them.

| Entry point | Synchronization contract |
|---|---|
| `Print::request_replay_from()` | Control thread, `state_mutex` already held; calls the cancellation barrier before any modification of the schedule or of the Ordering copy. Do not call from the worker. |
| `Print::invalidate_all_steps()` | Same prior stop, before resetting the schedule, the progress and the snapshot. |
| `Print::request_replay_from_worker()` | Worker owning the run, or idle Print; briefly locks the milestones and invalidates them without a cancellation callback. Used by export and resume preparation. |
| `request_steps_after`, `mark_step*`, `reset_step_execution_plan_all` | Primitives without cancellation: owning worker or already-stopped worker; not UI invalidation entry points. |

The GUI cancellation callback may release and then reacquire `state_mutex` while it waits for the worker to finish; the UI entry point must therefore not lock this non-recursive mutex a second time. The worker must never use this callback to request its own replay. Both paths share the same checkpoint/milestone mapping, without temporarily replacing the global callback.

Invalidation of slicing parameters remains a control operation: the worker path does not invalidate parameters already prepared by apply when it refreshes the milestones. This prevents an unchanged apply from subsequently trying to recompute them during execution.

A request handled by the worker is not left as a pending UI cleanup request: an identical apply must not interrupt the preparation of its export. A physical printer change also respects the barrier, before replacing its configuration. An apply that is only CHANGED without a confirmed stop does not consult the schedule to decide on a cleanup.

`StepExtrusionOrdering::clear()` only releases the plan and its Ordering copy; it marks no step. Replay policy belongs to the preceding entry points, not to output destructors.

Invalidation keeps the plan alive until plugins are notified; object replacements by apply and full clear also notify before destroying the owners. Callbacks run outside the state lock. A failure keeps the participant for retry and blocks normal resume with `PluginCleanupError`, displayed as a validation error by the GUI. By contrast, a full clear attempts all notifications, reports failures and then drops their tracking to allow a reset; the host does not claim to release the opaque private caches of a failing plugin itself. A new setup must rebuild its state rather than reuse borrows from the previous project.

A participant's missing storage is an explicit error, not a successful cleanup; the participant remains tracked for a normal resume. This is not a hot-reload mechanism: Print instances must stay within the lifetime of the orchestrator and of the registered plugins.

### Error diagnostic registry

Plugin callbacks return an integer diagnostic id defined by `slic3r_plugin_types.h`. `orchestrator_set_error_message(orchestrator, message)` copies a UTF-8 message of arbitrary length into a registry owned by the orchestrator and returns a unique, increasing id that is never reused. Ids are valid for the lifetime of the process: they must be neither serialized nor replayed after a restart. The call returns `PLUGIN_ERROR_GENERAL` if the orchestrator is missing or if the message is NULL or empty.

| Returned value | Meaning | Host interpretation |
|---|---|---|
| `PLUGIN_ERROR_OK` (0) | Success | No diagnostic |
| `PLUGIN_ERROR_GENERAL` (1) | Failure without a specific diagnostic | Generic diagnostic |
| 2..`PLUGIN_ERROR_RESERVED_LAST` (127) | Reserved for future host codes | Generic diagnostic until a text is defined |
| `PLUGIN_ERROR_FIRST_DYNAMIC` (128) and above | Id registered by `orchestrator_set_error_message()` | The host resolves the registered text; an unknown id gets a generic diagnostic |
| Negative | Invalid | Treated as `PLUGIN_ERROR_GENERAL` |

The Python generator exposes the `PLUGIN_ERROR_OK` / `PLUGIN_ERROR_GENERAL` constants and binds `orchestrator_set_error_message` for `store_error_message()`.

### Explicit result of the clear callback

The `slic3r_plugin_types.h` 3.0 contract replaces the void return with an integer diagnostic id, without a returned structure or a thread-local context:

```c
typedef int32_t (*plugin_clear_fn)(
    void *plugin_ctx,
    const print_handle *print,
    storage_handle *plugin_storage);
```

`PLUGIN_ERROR_OK` removes the participant from tracking; any other positive value is a diagnostic resolved through the registry described above, and a negative code is treated as `PLUGIN_ERROR_GENERAL`.

The C++ hook `clear_impl()` returns this integer; its wrapper registers the exception text (`what()`) before the exception is destroyed and returns the obtained id, or `PLUGIN_ERROR_GENERAL` if registration fails. The Python hook `clear(print, plugin_storage)` returns a signed 32-bit integer (`None` means success, bool is excluded) and calls `store_error_message()` to attach a diagnostic. Any other type, a negative or out-of-range integer, and Python exceptions produce a diagnostic registered by the bridge before references and the GIL are released.

Failed participants remain tracked and the other callbacks continue. No call to `orchestrator_plugin_report_error(nullptr, ...)` implicitly marks clear as failed: without an explicit context this function only logs an unattributed error. The rules for ordinary setup/run callbacks with a context remain unchanged.

This return change breaks the ABI: native plugins must be recompiled and manifests regenerated; 2.x versions of the contract are rejected.

| Member | Owned allocations | Write / lifetime |
|---|---|---|
| `PluginStorage::m_entries` | One `Entry` per allocation, keyed by the exported handle: polylines, polygons, expolygons, their collections, surface collections, extrusion roots, configs (keyed by the adjusted `ConfigBase` handle), triangle meshes built by `storage_new_triangle_mesh`, Clipper adapters/results | Mutable scratch (meshes: read-only after creation), until free/clear/destruction. Not a recursive owner nor a reference counter; an owned Clipper adapter does not mean an owned geometric source; children of an extrusion root are owned by that root, not separate allocations; a config's options are destroyed with the config; the by-value elements of a collection (polygons, polylines, expolygons, surfaces) are invalidated by a structural change of that collection, while the root pointer stays stable |
| `PluginStorage::Entry` | The object and its type-erased destroy function | Destroying the entry destroys the object, wherever the registry node dies |
| `PluginStorage::m_first_extrusion` / `m_last_extrusion` | Ends of the insertion-ordered chain of extrusion entries | `stored_extrusions()` walks it; a free unlinks in O(1) |

The registry bookkeeping of `store`, `contains`, `size` and `free` is average O(1) whatever the type and number of allocations, so releasing N allocations costs O(N) bookkeeping; this excludes the cost of destroying an object itself, which is proportional to its content (an extrusion tree, a PolyTree). `free()` finds the entry under the lock, unlinks it from the extrusion chain, extracts its registry node, and destroys the object after unlocking; a destructor never runs under the storage lock. From the unlock on, `contains()`, `size()` and `stored_extrusions()` no longer report the allocation. This does not protect a reader that still uses the object while another callback frees it, which the run-context contract forbids. `clear()` swaps the whole registry out under the lock and destroys the objects after unlocking. Destruction order between objects is unspecified: no destructor reads another stored object nor the storage. Registering a handle twice is a programming error (asserted).

`storage_free` must only receive roots of this storage, never a child, a contour, an option or a source/plan object. A null, unknown or already freed handle asserts in Debug and returns false. `storage_clear` destroys all allocations; handle copies are not reset to zero. The `Stored*` RAII wrappers carry `m_storage` and `m_handle`: they own the responsibility for releasing the handle, not the storage, which must outlive them. The registry lock protects allocation and release, not simultaneous access to the stored objects.

Sources: `src/libslic3r/Api/host/Orchestrator.hpp:629`, `src/libslic3r/Api/host/Orchestrator.cpp:2530`, `src/libslic3r/Api/host/slic3r_utils.cpp:39`.

### Geometric members

| Class / member | Ownership / views | Write and freeze |
|---|---|---|
| `MultiPoint::points`, inherited by Polygon/Polyline | Owned vector of Point; direct views of the buffer | Follows the source/auxiliary/plan/scratch owner, no intrinsic freeze |
| `ExPolygon::contour` | Embedded Polygon | Same window as the owner ExPolygon |
| `ExPolygon::holes` | Vector of Polygon by value | Resize/erase invalidate the affected holes |
| `Polygons`, `Polylines`, `ExPolygons`: vector elements | Owned values | Structural change may invalidate elements, contours, holes and points |
| `multipoint_view::array` | Mutable borrow of `points.data()` | Direct writes cannot be intercepted; reacquire after a mutable call according to the ABI contract |
| `multipoint_const_view::array` | Constant borrow of the same buffer | Not an immutable snapshot; another alias may modify/destroy the source |
| Multipoint views `size`, `flags` | Descriptor values | Changing them neither resizes nor reconfigures the native MultiPoint |
| `BoundingBoxBase::{min,max}` | C getters return the bounds by copy | No mutable handle to the source bbox; recompute after editing geometry |
| `c_point`, `c_vec2d`, `c_vec3f`, `c_vec3d`, `c_matrix4d::value` | Copied values unless an array is explicitly borrowed | Copies freely modifiable by their owner, with no effect on the source |

Geometric mutators ignore the pipeline owner. A mutable handle borrowed before a boundary is not revoked afterwards. Contracts forbidding source/destination aliasing must be respected; do not assume that self-append or insertion from its own buffer is safe.

Sources: `src/libslic3r/MultiPoint.hpp:417`, `src/libslic3r/ExPolygon.hpp:59`, `src/libslic3r/Api/plugin/c/slic3r_geometry.h:152`, `src/libslic3r/Api/host/slic3r_geometry.cpp:474`.

### Clipper adapters

| Class / member | Ownership and lifetime | Mutation / const |
|---|---|---|
| `PolygonShapes::m_polygon` | Borrow of a Polygon | Source stable while the adapter is in use |
| `PolylineShapes::m_polyline` | Borrow of a Polyline | Same rule |
| `PolygonsShapes::m_polygons` | Borrow of a collection | Same rule |
| `ExPolygonsShapes::m_expolygons` | Borrow of a collection | Same rule |
| `ExPolygonShapes::m_expolygon` | Borrow of an ExPolygon | Same rule |
| `PathListShapes::m_paths` | Owned geometry of the flat result | Lifetime of the result in storage |
| `PolyTreeShapes::m_tree` | Owned geometry of the hierarchical result | Lifetime of the result in storage |

Owning an adapter does not extend the lifetime of its source. `clipper_concat_replace` may replace/release the previous accumulator: use the returned handle. Conversions to collections create independent results in the requested storage.

Sources: `src/libslic3r/Api/host/ClipperShapes.cpp:123`, `src/libslic3r/Api/host/slic3r_clipper.cpp:339`.

## 8. Generic properties, records and configurations

### `PluginPropertyContainer` and shared backend

Source getters are const; the mutable getters of owner nodes enforce the windows according to the mode of §1, while surfaces use structural capabilities. Only records and main-object properties are saved at `None` and `Slicing`.

Production write inventory (September 2026) justifying this choice: no step writes a plugin property on a main object, layer, region or island. Existing writes target nodes created after `Slicing`: `LayerAdhesionProperty` on auxiliary layers in `SKIRT_BRIM` (`src/libslic3r/Plugins/SkirtBrim/AdhesionLayerHelpers.cpp:63`) and `SurfaceDenseInfillHint` on surfaces in `SURFACE_GENERATION` (`src/plugins_cpp/DenseInfill/DenseInfill.cpp:558–581`). From `STEP_ORDERING` onwards, plugins edit the plan, not the source tree or the records. No production plugin currently writes a record; only the host publishes `custom_gcode_per_print_z` in `StepExtrusionOrdering::prepare()` and empties the store in `clear()`.

| Member | Ownership / mutation | End of validity / freeze |
|---|---|---|
| `PluginPropertyContainer::m_properties` | Sole owner of a vector of slots; base of PrintObject, Layer, LayerRegion, LayerSliceIsland, LayerRegionIsland, Surface; member of plan scopes | Source: Debug windows/capabilities of §1; plan scopes: `Plan` through const views |
| `PropertyStorageSlot::m_type` | Numeric identity of the payload | Not a stable identity across orchestrators |
| `PropertyStorageSlot::m_buffer` | Owned buffer | Slot removal/replacement |
| `PropertyRawBuffer::m_data` | Owned aligned allocation, borrowed direct access | Removal/replacement/clear/copy of the owner |
| `PropertyRawBuffer::m_byte_count` | Owned size | Buffer lifetime |
| `PropertyRawBuffer::m_alignment` | Owned alignment | Buffer lifetime |
| `Orchestrator::m_custom_property_infos` | Registry of PropertyInfo layouts | Host/plugin registrations; reallocation may invalidate borrowed names |
| `PropertyInfo::{type,name,byte_count,alignment}` | Identity and layout; owned name | Copy a name to keep it beyond a later registration |
| `c_layer_support_property::{interface_id,reserved}` | Layer payload | Same lifetime as its property |
| `c_layer_brim_property::reserved` | Layer payload | Same rule |
| `c_layer_adhesion_property::{kind,flags}` | Layer payload | Same rule |

The backend copies independent bytes, not arbitrary objects pointed to by those bytes: no owning pointer, C++ string or vector may be placed in a generic payload. Reallocating the slot envelopes may preserve the payload allocations, but the contract requires reacquiring views after relevant mutations. The mutators have neither a phase guard nor an internal lock.

Sources: `src/libslic3r/PluginProperty.hpp:94`, `src/libslic3r/PropertyStorage.hpp:109`, `src/libslic3r/Api/host/slic3r_data_tree.cpp:217`, `src/libslic3r/Api/plugin/c/slic3r_data_tree.h:412`.

### `PrintRecordStore`

| Member / view | Ownership and mutation | Freeze / validity |
|---|---|---|
| `m_channels` | Map of DynamicConfig per channel; empty channel allowed | Sequential writes until `Print` per contract; addition/removal guarded only by phase assertions |
| `m_next_id` | Atomic counter of positive IDs | No controlled freeze boundary on allocate_id; only this access is thread-safe |
| Config of a channel | Value owned by the map, borrowed by the API | Stable when other channels are added, invalidated on its removal/clear/destruction |
| `print_records_channels` list | Sorted array in a thread-local buffer | Until the next call to this function on the same thread |

`clear()` destroys the channels and resets the counter to 1. A checkpoint restore must not use this ID reset as a simple rollback if it promises not to reuse the IDs of the previous attempt. An old mutable Config/Option handle can still bypass the store's access guard; the generic mutators do not know its owner.

Source: `src/libslic3r/PrintRecords.hpp:22`, `src/libslic3r/Api/host/slic3r_data_tree.cpp:959`.

Implemented snapshots: a `CheckpointMetadataSnapshot` owned by `Print` for each of `None` and `Slicing` holds a copy of the `DynamicConfig` of every channel and a copy of the `PluginPropertyContainer` of each main object, indexed by object. `None` is captured on entry to `STEP_LAYER_HEIGHT`, before `prepare()` and the plugins; `Slicing` is captured at the successful, non-cancelled end of `STEP_POST_SLICING`. Restoration happens in `prepare_checkpoint_resume()` after the `clear` notification and host cleanup, without consuming the copy; missing snapshot: nothing to restore; object without a copy: properties cleared. A cleanup failure prevents host cleanup and restoration, and keeps the snapshot of the resumed checkpoint for retry. For a channel that still exists, the value is restored in the same object to preserve its `config_handle`; removed channels are recreated and added channels removed, without rewinding `m_next_id`. Option/payload handles must be reacquired. A `None` replay releases the `Slicing` snapshot; replacing/removing an object releases its copy before the object; `Print::clear()` releases both snapshots.

The `None` baseline is a snapshot of records and main-object properties before LayerHeight, not a live mirror of metadata written outside the pipeline. Writes after capture do not update this baseline and are rolled back at the next `None` resume, before the next LayerHeight capture. Current model and configuration inputs are never rolled back by these metadata snapshots. Future host input records must therefore be published by their first consumer step, rather than by `apply()` in the expectation that post-capture writes will survive restoration. Invalidation alone does not restore the metadata baseline; restoration waits until the next run (§11).

Host publication: each input record is published by the first step that consumes it, without any special channel marking. `custom_gcode_per_print_z` is published from the model by `StepExtrusionOrdering::prepare()` (`STEP_ORDERING`), restoring the channel value in place if it exists. This is a deliberate host operation directly on `PrintRecordStore`, not a relaxation of the plugin record-write window, which ends at `STEP_FINALIZE_SLICING` inclusive. `Print::apply()` does not publish it; a change of this table, including the mode of an empty table, requests a `Print` replay. Its readers are in `ORDERING` and `EXTRUSION_EDIT`, none before `STEP_ORDERING`. Created after `Slicing`, the channel disappears on `Slicing` restoration and is republished by Ordering; direct export from `Plan` keeps the published table.

### Live and temporary configurations, and options

A `config_handle` targets the `ConfigBase` subobject, with native pointer adjustment; do not treat it as any raw DynamicConfig.

| Storage / member | Ownership and lifetime | Write / freeze |
|---|---|---|
| `DynamicConfig::options` | Map of ConfigOption owners; scratch, record or model config depending on origin | Scratch mutable; record until `Print` contractually; model input read-only |
| Embedded values of static configs | ConfigOption members of Print/Object/Region instances | "Static" does not mean immutable value; region truly const, Print/Object technically editable through mutable getter |
| `StaticPrintConfig::m_dynamic_options` | Plugin options owned by the static instance | Same window as its owning configuration |
| `StaticPrintConfig::{m_keys_with_dynamic,m_keys_with_dynamic_dirty}` | Native cache of the key list | May change during a const read; not a domain snapshot |
| `ConfigOption::flags` | Enabled, can-be-disabled, phony, extruder-array, placeholder-temp | Same lifetime/rights as the option |
| `ConfigOptionSingle<T>::value` | Float/int/string/bool and scalar derivatives | API mutation if the option is mutable |
| `ConfigOptionFloatOrPercent::percent` and inherited `value` | Value + unit | Same window |
| `ConfigOptionPoint::value`, `ConfigOptionPoint3::value` | Floating-point config coordinates | Same window; not a source MultiPoint |
| `ConfigOptionEnum<T>::value` | Enum value | Same window |
| `ConfigOptionEnumGeneric::value`, `keys_map` | Value + borrow of a name table, possibly null in scratch | Does not own the borrowed table |
| `ConfigOptionVector<T>::m_values` | Float/int/string/percent/points/bools/FloatOrPercent vectors | Resize/clear/replacement invalidate their borrowed descendants |
| `ConfigOptionVectorBase::m_enabled` | Per-element enablement | Same window as the vector |
| `FloatOrPercent::{value,percent}` | Owned element | Same owner |
| `ConfigOptionGraph::value`, `ConfigOptionGraphs::m_values` | Owned GraphData | Const graph getter; replacement/deserialization of the option may modify/invalidate |
| `GraphData::{graph_points,begin_idx,end_idx,type}` | Graph values | Graph/option lifetime; no independent global freeze |

Adding another option does not invalidate a separately allocated option. However, `config_clear()` destroys all of them, and a successful `config_deserialize_all()` replaces the entire map after merging on a copy: **all old option handles must be reacquired**, not only those for keys present in the document.

`config_get_or_add_mutable`, clear and global deserialization require DynamicConfig; mutable access to an existing option may work on a static config. The keys returned by `config_keys` are in a thread-local snapshot, valid until the next call on the same thread. The text/SCFG getters copy into the caller's buffer. `RAW_CO_VECTOR_ENUM` does not currently correspond to a supported native family.

Sources: `src/libslic3r/Config/ConfigDef.hpp:904`, `src/libslic3r/Config/ConfigOption.hpp:473`, `src/libslic3r/Config/PrintConfig.hpp:305`, `src/libslic3r/Api/host/slic3r_config.cpp:74`.

### Setting descriptors: schema, not user values

Owner: `PrintConfigDef` / `ConfigDef::options` catalog. Construction/patching before schema finalization; no link with the `Print` checkpoint. `PrintConfigDef::instance_mutable()` refuses access after finalization. The GUI `readonly` flag does not lock the generic value setters.

| Native storage | Exposed members / mapping | Lifetime |
|---|---|---|
| `ConfigOptionDef`: identity/classification | `opt_key`, `type`, `nature`, `category`, `container_type`, `option_preset_type`, `gui_type`, `printer_technology`, `invalidates_step` | Catalog, read after finalization |
| ABI level/flags | `plugin_api_level`, `plugin_api_flags` | Distinct from ConfigOption::flags |
| Presentation/editing | `is_optional`, `multiline`, `full_width`, `is_code`, `is_vector_extruder`, `readonly`, `can_phony`, `can_be_disabled`, `aligned_label_left`, `contains_plugin_id` | Schema values |
| Owned texts | `gui_flags`, `label`, `full_label`, `tooltip`, `translation_domain`, `sidetext`, `cli`, `ratio_over` | Copies at registration |
| Dimensions/precision | `height`, `width`, `label_width`, `sidetext_width`, `precision`, `mode` | Schema values |
| Limits | `min`, `max`, `max_literal` | ABI also exposes presence of limits and unit of the literal |
| Dependencies | `aliases`, `shortcut`, `depends_on` | Owned collections |
| `default_value` | Owned constant option, built from the serialized text | Not the value chosen in each Print |
| `enum_def` | Owned ConfigOptionEnumDef | Borrowed static built-in tables or owned script table |
| `ConfigOptionEnumDef::{m_values,m_labels}` | Values/labels copied into the ABI's `enum_def.value_label_pairs` pairs | Descriptor lifetime; ABI view in the thread-local snapshot |
| `ConfigOptionEnumDef::{m_enum_names,m_enum_keys_map,m_enum_keys_map_storage_for_script}` | Native conversion tables, **not exposed in the ABI descriptor**; borrowed static tables or owned script table | Lifetime of the descriptor/source table |
| `ConfigOptionEnumDef::{m_labels_translated,m_values_ordinary}` | Internal translation cache and ordering indicator, not carried as ABI descriptor members | The translation cache may change after schema finalization |
| `graph_settings` | Owned/shared GraphSettings | Descriptor lifetime |
| `GraphSettings` | `title`, `description`, `y_label`, `x_label`, `null_label`, `label_min_x`, `label_max_x`, `label_min_y`, `label_max_y`, `min_x`, `max_x`, `step_x`, `min_y`, `max_y`, `step_y`, `allowed_types`, `has_reset_vals`, `reset_vals`, `enforced_values` | Same schema contract; reset_vals is a GraphData |
| `Orchestrator::m_option_def_patches` / `OptionDefPatch` | `kind`, `opt_key`, `enum_values`, `remove_values`, `default_serialized_value`, `source_option_key`, `prepend`, `plugin_id` | Copies of requests applied at finalization, not slicing data |

`orchestrator_get_option_def` returns a structure containing pointers into a thread-local snapshot, invalidated at the next call on that thread. The `ConfigOptionDefinition` wrapper makes its own copies; the current wrapper does not represent `contains_plugin_id`. Registrations copy their inputs, unlike the borrowed read descriptors.

Sources: `src/libslic3r/Config/ConfigDef.hpp:153`, `src/libslic3r/Api/host/slic3r_config_def.cpp:56`, `src/libslic3r/Api/plugin/cpp/ConfigDefinitionViews.hpp:75`.

## 9. Step contexts and temporary outputs

A `plugin_run_context` is a callback borrow. Its members `step`, `print`, `plugin_storage`, `data`, `host_context` and callbacks do not own the pointed-to objects. Copying the context does not extend any lifetime; the opaque host_context must not be inspected or retained. Step wrappers may live during setup_run/run, but the public contract may limit the borrow to the current call.

`plugin_storage` still designates the plugin's shared persistent storage, not a private storage created by the context copy. Its allocation registry is locked as described in section 7; a callback must not clear it while parallel callbacks may use it. Use `slic3r_parallel_for_storage` when each task needs an independent temporary lifetime.

### Layer-height and perimeters

| Class / member | Storage and write rights | End of validity / publication |
|---|---|---|
| `LayerHeightRunContext::enforced_layer_profile` | Converted copy of the model profile, backing of `enforce_layer_zs` | Context lifetime, not an alias of the model input |
| `LayerHeightRunContext::layer_config_ranges` | c_layer_config_range values, copied bounds and borrowed configs | Lifetime of the context array; config targets tied to the model |
| `LayerHeightRunContext::context_step` | Payload/callbacks borrowing both buffers | Not an object backup |
| `SlicingRunContext::context_step` | Sole member, no additional private geometric storage | Context lifetime |
| `PerimeterTree::root` | Root owning the temporary processing tree | Destroyed when the group processing returns |
| `PerimeterTree::last_span_nodes` | Pointer array backing the split span | Replaced at the next span/rebuild result |
| `PerimeterTreeNode::node` | Embedded C descriptor | Temporary node lifetime |
| `PerimeterTreeNode::area` | Owned ExPolygon exposed through node.area | Editable during callback, working copy independent of the source island |
| `PerimeterTreeNode::infill_areas` | **One ExPolygon**, exposed through node.fill_area | Same window |
| `PerimeterTreeNode::extrusions` | Exposed temporary root | Filled/consumed during generation/publication |
| `PerimeterTreeNode::children` | Node owners | Rebuild destroys old descendants |
| `PerimeterTreeNode::child_nodes` | Non-owning array exposed through node.children | Recreated by structural synchronization |
| `perimeter_node::{parent,area,fill_area,extrusions,children,child_count}` | Links/handles to the storages above | Structure managed via callbacks, no free editing of links |
| `perimeter_node::{perimeter_idx,perimeter_needed}` | Counters editable per module contract | Node lifetime |
| Local buffers `inner_areas`, `inner_infill_areas` | Output ExPolygons of generate_node | Synchronous call; consumed to create the children |
| `PerimeterRunContext::{infill_areas,infill_free_areas}` | Owned accumulations of domains | Moved to the island at publication; no persistent borrow of the context |

The native member `PerimeterRunContext::perimeter_slices` is declared but is not the published buffer used in this implementation; the persistent output is computed differently. Do not invent a `PerimeterOutputs` owner.

Sources: `src/libslic3r/Api/host/steps/LayerHeightStep.hpp:20`, `src/libslic3r/Steps/StepGeneratePerimeter.cpp:72`, `src/libslic3r/Steps/StepGeneratePerimeter.cpp:1017`.

### Infill and support demands

| Class / member | Write / ownership | End of editing / lifetime |
|---|---|---|
| `SurfaceInfillRecipe::pattern_plugin_id` | Owned selection string | Lifetime of generate_surface |
| `SurfaceInfillRecipe::raw` | Recipe by value, modified in place by successive recipe modifiers | Const view for the pattern; destroyed with the recipe |
| `SurfaceInfillRecipe::no_overlap_areas` | Computed local geometry, passed const | Lifetime of generate_surface |
| `generate_surface()::output` | Local root initially empty, filled by the pattern | Content published by move; local handle not transferred |
| `InfillPatternService::m_runtime_ids_by_plugin_id`, `m_patterns` | Local name ↔ ID/plugin registries, built once by the constructor and read-only afterwards | Lifetime of the step call that owns the service (`STEP_INFILL`, `STEP_SUPPORT` or `STEP_SUPPORT_TOOLPATH`), non-persistent IDs |
| `InfillStepHostContext::recipe_modifier_plugins` | List of borrowed providers | Step context lifetime |
| `run_ctx_generate_infill::{host_context,resolve_pattern_id,pattern_plugin_id,report_pattern_assignments,modify_surface_recipe,generate_pattern,append_region_island_extrusion}` | Borrowed `InfillStepHostContext` with the pattern service, copied discovery groups and recipe-modifier contexts; no exception crosses these callbacks. `resolve_pattern_id` returns INVALID on a contained failure, while normal inactive-id fallback remains a warning; `pattern_plugin_id` is never NULL and returns "" for an unknown id. `report_pattern_assignments` copies generator-owned arrays during `setup_run()`; a contained failure requests a stop before pattern initialization. `modify_surface_recipe` runs modifiers in priority order against one mutable recipe; the chain stops on failure and preserves earlier edits, so the recipe is not consumed. `generate_pattern` returns 0 and empties caller-owned output on a contained failure or pending stop; ordinary refusals keep their existing result and message. `append_region_island_extrusion` stages moved children before appending to the role bucket; on failure, previously published paths remain unchanged, while the caller-owned source may be partly consumed. That callback has no print handle, so print cancellation is handled by the generator and runner. Other failures add a plugin error message and request cancellation | The context and runtime IDs live only for the current `STEP_INFILL` call. The host retains copied assignments and their borrowed print-geometry handles until the call ends; source arrays need only survive their report callback |
| `InfillPatternService::Pattern::{host_context,execution_error,run_context}`, `InfillStepHostContext::recipe_modifier_host_contexts` | Owned contexts for the callbacks; pattern contexts and plugin storage entries are created before any call, so calls may run concurrently | Lifetime of the step call |
| `support_demand_get_island_mutable(run_ctx, island)` | Host function returning the island-owned demand (`LayerSliceIsland::m_support_demand`); refused outside `STEP_SUPPORT_DEMAND`..`STEP_SUPPORT` | The collection lives as long as the island |
| `run_ctx_support_toolpath::{print,object}` | Borrowed whole-object context at `STEP_SUPPORT_TOOLPATH`, after `STEP_SUPPORT` and before `STEP_SKIRT_BRIM`; selected role algorithms read support surfaces and write auxiliary region-island extrusions through data-tree mutators. A contact provider may replace a region-island's surface collection with `layer_region_island_set_fill_surfaces()` to hand areas back: geometry, order, properties and other flags are preserved, and only contact is cleared | Replacement invalidates all old surface and geometry handles for that collection; only the recreated surfaces remain valid. Auxiliary editing remains allowed through `STEP_FINALIZE_SLICING`; outputs are destroyed with auxiliary layers at a `None`/`Slicing` replay, with no separate dispatcher cleanup |
| `run_ctx_generate_support::{host_context,resolve_pattern_id,generate_pattern}` (ABI 1.2) | Borrowed holder of an `InfillPatternService`, built the first time a callback is used (`std::call_once`) and shared by all objects of `STEP_SUPPORT`; `layer`, `island`, `region_island`, `primary_region` and `no_overlap_areas` may be NULL (rectilinear, supportbase and honeycomb do not read them), and `output` is never published (the caller only reads it). No exception crosses the callbacks: an error or failed service construction makes `generate_pattern` return 0 (`resolve_pattern_id` returns the invalid id), empties `output` (children and a path written directly on it), records a plugin error and requests plugin cancellation. A print cancellation returns the failure value for the runner to rethrow; a requested stop returns immediately without a new message. The run count advertised to patterns is the object count | Valid during the family callback; runtime IDs are valid only for this step run |
| `run_ctx_support_toolpath::{host_context,resolve_pattern_id,generate_pattern}` | Borrowed `InfillPatternService` shared by all objects of the step; the caller builds the whole `raw_infill_pattern_params` and owns `output`, which the pattern only fills; no recipe modifier; lightning, adaptive cubic and support cubic are refused. No exception crosses either callback; a contained generation failure or pending stop returns 0 with empty output, while ordinary refusals keep the service's message and let the step continue. A print cancellation is left for the runner to rethrow | Valid during the algorithm callback; runtime IDs are valid only for this step run |

By-value fields of `raw_infill_pattern_params`, all from the same recipe: `pattern_id`, `surface_type`, `extrusion_role`, `flow`, `density`, `flow_mult`, `connection`, `add_gap_fill`, `gap_fill_enabled`, `dont_adjust`, `monotonic`, `fill_exactly`, `complete`, `use_arachne`, `can_angle_cross`, `extruder`, `priority`, `spacing`, `angle`, `bridge_angle`, `bridge_type`, `layer_height`, `z`, `layer_id`, `overlap`, `bridge_offset`, `fill_resolution`, `link_max_length`, `loop_clipping`, `anchor_length`, `anchor_length_max`, `max_sparse_infill_spacing`.

`c_flow` owns `width`, `spacing`, `height`, `nozzle_diameter`, `is_bridge`, `spacing_ratio`, `mm3_per_mm`. This record does not own the regions from which its values are computed.

The pattern generation callback borrows the provided buffers, without allocating or clearing the output itself. The default generator provides a fresh output. The support demand is island data (see the `LayerSliceIsland` table): `STEP_SUPPORT_DEMAND` runs right after `STEP_POST_SLICING`, so every later step can read it.

`STEP_SUPPORT` calls every active support family in priority order, once per object, after infill and before `STEP_SUPPORT_TOOLPATH`. It has no host selector. Each family checks the object's support enablement and `support_material_style`, leaving other objects intact. Families read island demand and publish auxiliary layers marked with `LayerSupportProperty` and typed surfaces: `support | top | solid` for upper contacts/interfaces, `support | bottom | solid` for lower contacts/interfaces (plus `bridge` for a lower contact using bridge flow), and `support | internal | sparse` for the base. The dense areas of a contact layer carry `RAW_SURFACE_TYPE_MOD_CONTACT`; a dense top or bottom surface without it is an interface, and a contact merged into the base is sparse base without it. They do not write extrusions, but they may sample `INFILL_PATTERN` plugins (for example to place tree tips) through `resolve_pattern_id()` and `generate_pattern()` of their payload (ABI 1.2; the pattern service is built by the first callback; a failing call returns 0 with its output emptied and stops the step through the runner once the family returns). The built-in `support.family.grid` family handles the grid and snug styles.

`support.family.filled` projects final island demand downward at exact object-layer heights, cumulatively subtracting the XY-grown object slices. It requires contact distance type `none` and no raft; incompatible settings fail generation only on objects with demand. Splitting at the next layer's slices produces top contacts and intermediate interfaces, with a sparse first-layer expansion ring. Run storage owns the slabs until Common publication creates the auxiliary layers and marks contact surfaces. Initialization declares all five shared legacy toolpath preferences, including `legacy_sparse` for the ring. Native top-contact patterns and contact loops remain available; shared automatic fusion can return contact area to rectilinear interface paths. Zero interface layers retains the common base-setting behavior. Buildplate-only demand filtering also applies to filled, without new GUI restrictions.

`STEP_SUPPORT_TOOLPATH` resumes from `Checkpoint::Slicing` like `STEP_SUPPORT`. Its five plugin-defined role groups (sparse, bottom contact, bottom interface, top contact and top interface) are selected per object through a `PrintObject` option; selected plugins run sequentially by priority, each on the objects that selected it. The dispatcher does nothing when no provider is active. The host registers `support.toolpath.legacy_sparse`, `support.toolpath.legacy_bottom_contact`, `support.toolpath.legacy_bottom`, `support.toolpath.legacy_top_contact` and `support.toolpath.legacy_top` at priorities 0/90/100/190/200. `support.toolpath.branch_sheath` shares the sparse role. The host imposes no contact/interface role order: a contact provider handing areas back must run before its interface provider, or those areas remain unprinted.

Sources: `src/libslic3r/Plugins/Infill/DefaultInfillGenerator.cpp:279`, `src/libslic3r/Api/plugin/c/steps/slic3r_step_infill.h:40`, `src/libslic3r/Steps/StepSupportDemand.hpp`.

### C++ computed views

| Class / members | Owned or borrowed | Validity / mutation |
|---|---|---|
| `RegionSettingsValue::m_keys` | Owned names | Value lifetime |
| `RegionSettingsValue::m_options` | Const pointers to source options | Not a snapshot of the values; configs/options must remain stable |
| `RegionSettingsClip::m_storage` | Storage borrow | Must outlive the allocations |
| `RegionSettingsClip::m_expolygons` | RAII ownership of a scratch geometry | Null means accept everything, not an empty collection |
| `RegionSettings::{m_default_config,m_regions}` | Borrowed sources | Lifetime/stability of the source configurations and regions |
| `RegionSettings::{m_option_groups,m_key_areas,m_key_regions}` | Owned results/indexes, keys depending on borrowed values | Rebuild invalidates their references; sources stable during use |

Source: `src/libslic3r/Api/plugin/cpp/RegionSettingsViews.hpp:126`.

## 10. Sessions, services and result buffers

A service may be provided by an external plugin: the API cannot name the members of its opaque private state. The tables list the ABI fields and, where they exist, the storage of the shipped provider/host. Sessions have their own dedicated destructors; they must not be freed with `storage_free`.

### Geometry services

| Class / member | Ownership / write | Const / lifetime |
|---|---|---|
| `NativeBridgeDetector::{expolygons_owned,lower_slices_owned,detector}` | Geometry copies and owned detector | Session lifetime, destroyed via vtable |
| `BridgeDetector::{expolygons_owned,expolygons,lower_slices}` | Geometry and references depending on the provider's constructor | Stable session inputs |
| `BridgeDetector::{spacing,precision,angle,max_bridge_length,layer_id}` | Analysis state; setters/detection during use | Not frozen at the checkpoint; frozen according to session call/read |
| `mesh_query_mesh::triangles` | Owned working mesh, vertices/indices copied | Append/decimate modify it; specialized destroy |
| `mesh_query_scene::{triangles,labels,tree}` | Scene owning a transformed copy and index | **Immutable after creation**, may outlive the work meshes |
| `mesh_query_samples::{values,area}` | Owned result | Immutable, may outlive the scene; copies to caller buffers |
| `mesh_query_hits::rays` | Owned hit vectors | Same rule |
| `mesh_query_sample::{position,normal,geometry_id,triangle_id}` | Result copied to the requester | Self-contained values; IDs interpreted in the originating scene |
| `mesh_query_hit::{distance,normal,geometry_id,triangle_id}` | Same ownership by copy | No owned triangle handle |
| `mesh_query_ray::{origin,direction,min_distance,max_distance}` | Input by value in the caller's array | Borrowed only during the query call |
| `mesh_query_part::{mesh,transform,geometry_id}` | Borrowed mesh, transform/label by value | Copied into the scene at its construction; no persistent dependency on the work mesh |
| `mesh_query_diagnostic::message` | Fixed buffer supplied by the requester | Overwritten by calls that publish a diagnostic |
| `mesh_query_cancel::{context,is_cancelled}` | Borrowed context and callback | Duration of the synchronous call; no ownership transferred |

The built-in bridge detector replaces the coverage/unsupported edges output collections; do not assume an append that keeps the previous elements. Mesh scenes/results can be read concurrently, but not during their destruction.

Sources: `src/libslic3r/Plugins/BridgeDetector.cpp:76`, `src/libslic3r/Api/host/slic3r_mesh_queries.cpp:24`, `src/libslic3r/Api/plugin/c/slic3r_mesh_queries.h`.

### Seam placement

| ABI field | Ownership / write | Lifetime |
|---|---|---|
| `run_ctx_seam_placer::plan` | Working plan borrowed during initialization | Does not transfer ownership of the plan |
| `run_ctx_seam_placer::{regions,region_count}` | Borrowed array of const PrintRegion | Array during run only; regions tied to the Print |
| `run_ctx_seam_placer::instance` | Session published by the provider | Transferred to the consumer, destroyed exactly once |
| `raw_seam_placer_instance::session` | Transferred opaque state | Session lifetime |
| `raw_seam_placer_instance::vtable` | Borrowed static table | Must outlive the session |
| `struct_size`, `status`, `point` or `priority` results | Requester memory, filled by callback | Self-contained values after return |
| `error_message` results | Borrowed text | Until the next priority/place callback on the same thread or destruction |

After initialization, priority/place may run concurrently. They modify neither the borrowed loop nor the borrowed region; shared state/cache must support this concurrent read. The session depends on the plan for which it was created and is not a snapshot of that plan.

Source: `src/libslic3r/Api/plugin/c/steps/slic3r_step_seam_placer.h:21`.

### Scripts, firmware, thumbnails and G-code reader

| Storage / members | Ownership / write | Validity |
|---|---|---|
| `GCodeScriptProcessor::Impl::m_public_config` | Config copied/prepared, then mutable by the processor's user | Until the next prepare/destruction; **modifiable during G-code**, distinct from the frozen input configs |
| `GCodeScriptProcessor::Impl::{m_output,m_error}` | Owned result texts | Borrows until the next prepare/process/destruction |
| `GCodeFirmwareSessionHolder::{session,output,error}` | Provider state and C++ helper buffers | Until the next callback/destruction; a const interface does not make the firmware state immutable |
| `GCodeThumbnailRenderer::{m_image,m_error}` | Owned image/diagnostic | Next render/destruction |
| `ThumbnailData::{pixels,width,height}` | Owned pixels and dimensions copied into the view | Borrowed const pixels, never storage_free |
| `GCodeThumbnailEncoderHolder::{session,bytes,error,mutex}` | Owned session and encoded output | Next encode/destruction; the callback lock does not extend the borrow |
| `ViewerReaderInvocation::m_active_chunk` | string_view of the chunk supplied by the source | Next chunk/end of call; not the owner of the bytes |
| `ProbeInvocation::m_prefix` | Borrowed prefix | Probe call |
| `raw_gcode_reader_event::items`, `raw_gcode_reader_state::items` | Synchronous borrowed arrays | Recipient copies what it keeps |
| `GCodeStateTracker::{m_state_items,m_state}` | Owned array and the view referencing it | Tracker update invalidates/rewrites previous views |
| `run_ctx_gcode::{print,plan,firmware,output_path,reader_source_context}` | Borrows and publication context | Export call; plan/source read-only, firmware session with its own state |

Streams, reader services and per-export host buffers are temporary; the published path is copied into Print. The preview result is built then moved to the caller, and is not a plugin storage allocation. Final statistics and host operational state are outputs separate from the plan: the statistics do not reopen its final properties for writing.

Sources: `src/libslic3r/Api/host/GCodeScriptProcessor.cpp:181`, `src/libslic3r/Api/plugin/cpp/gcode/GCodeFirmwareViews.hpp:118`, `src/libslic3r/Api/host/GCodeThumbnailRenderer.hpp:34`, `src/libslic3r/Api/host/GCodeReaderService.cpp:74`.

### Registration metadata and service values

| Storage | Copy or borrow | Lifetime / editing |
|---|---|---|
| `Plugin::{m_id,m_name,m_description,m_exclusive_group,m_exclusive_group_priority,m_dependencies,m_used_config_keys,m_defined_config_keys}` | Owned copies of the provider's responses | Plugin catalog, not a checkpoint output |
| `Orchestrator::m_elected_group_definitions` | Owned complete group text, traits, declaring-member domain, untranslated marker and authenticated package id/root | Retained for the selector generation; later activation changes do not re-elect its definition. A missing package catalogue marks both retained text and generated options for verbatim display, bypassing default/foreign catalogues, and records a package warning |
| `Plugin::m_c_api::{ctx,vt}` | Borrowed provider context and pointer to the host-owned normalized callback table | Context and callback code must outlive calls; the host does not own provider memory |
| ABI structures `struct_size`, enums, flags, IDs, callbacks | Copied values or function pointers depending on the field | No slicing freeze of their own; respect the lifetime of the designated table/session |
| `c_medial_axis_extrusion_params` | Owned scalar parameters; `extension_area` is a borrow | The operation produces an extrusion in the requested storage; does not extend the lifetime of the borrowed area |

Medial-axis scalar fields: `min_medial_width`, `max_medial_width`, `min_extrusion_width`, `max_extrusion_width`, `min_centerline_length`, `endpoint_extension_length`, `endpoint_taper_length`, `role`, `flow`, `variable_width_resolution`, `width_change_tolerance`, `min_extrusion_length`, `flags`. Same lifetime for these values; `extension_area` must remain valid while it is used.

Sources: `src/libslic3r/Api/host/Plugin.cpp:33`, `src/libslic3r/Api/plugin/c/slic3r_extrusions.h:253`.

## 11. Current limits and consequences for checkpoints

### What the current checks do not guarantee

1. The data tree guards are mainly assertions depending on the current step; `running_step_may_edit()` also accepts `STEP_NONE`. This is not a permanent frozen state of each object.
2. OFF guards the property windows only in Debug; ON refuses without a freeze assertion in every configuration. Source surfaces expose no global mutable getter; scratch editing does not depend on a Print. Snapshots remain limited to records and main-object properties at `None`/`Slicing`.
3. The generic geometry/extrusion/config/property mutators ignore their owner. A mutable alias handed out earlier is not revoked when a checkpoint is passed.
4. Print/Object configurations are editable through their getters only outside a step, according to the freeze mode, without automatic invalidation; this is not a transactional user-editing API. Older Config/Option aliases stay outside this entry check.
5. The C++ const of a unique_ptr or of a container of unique_ptr does not make the pointed-to objects deeply const. The const views of the final plan are more restrictive than some native accesses.
6. Plan handles do not all guarantee their identity/provenance: in particular, transfer between tool groups does not currently enforce all the announced same-layer/same-plan checks. This is not a permission for inter-plan transfer.
7. Event roots have a non-sortable/non-reversible contract; the current clear/append cycle can re-enable sorting via `ensure_children()`. The announced invariant is therefore not a guarantee for all current native paths.

Static findings, not runtime test results. References: `src/libslic3r/Print.cpp:528`, `src/libslic3r/Api/host/slic3r_data_tree.cpp:510`, `src/libslic3r/Api/host/slic3r_printing_plan.cpp:830`, `src/libslic3r/Printing/PrintingPlan.cpp:464`, `src/libslic3r/ExtrusionEntity.cpp:301`.

### Storage decisions that can be inferred

Destruction happens at invalidation, restoration at the next run. `Print::apply()` stops the worker, calls the plugin `clear` callbacks and drops the live plan as soon as a step is invalidated; at the end of `apply()`, once every object has its regions again, `StepPipeline::clear_checkpoint_outputs()` destroys the host outputs after the final checkpoint (from `Ordering`, only the live plan; the copy is kept). Saved data (records and main-object properties at `None`/`Slicing`, the `Ordering` clone) is restored only by `run_slice()`/`run_gcode()`, because several invalidations may accumulate and only the run knows the final checkpoint. A failed plugin `clear` destroys nothing. The empty `PrintBase::cleanup()` hook and its FFF/SLA overrides are removed; invalidation and resume own this lifecycle.

`Print::prepare_checkpoint_resume()` repeats notification and host destruction as a safety net (no-op after `apply()`; needed after a canceled run or an invalidation made outside `apply()`), then restores. From `None`, the auxiliaries are destroyed before the main layers and the profile; from `Slicing`, the region-islands and the fill caches of the retained regions are cleared. The hash of these caches no longer uses a variable shift likely to exceed the integer width; it remains address-based and requires explicit invalidation after in-place editing.

A direct call to `run_gcode()` at the `None` checkpoint is rejected before any destruction: the caller must run `run_slice()` to produce the missing prefix. Cleanup must never erase the layers and then skip their reconstruction.

| Situation | Consistent handling |
|---|---|
| Owner created after the checkpoint | Destroy this owner, without separately purging each child/property |
| Main layer/region/island kept at `Slicing` | Freeze its properties after `STEP_POST_SLICING`, no snapshot |
| Main object and records kept at `None`/`Slicing` | Restore only this metadata from its target snapshot |
| Full source tree kept at `Print` and later | Target freeze; no additional snapshot of source metadata |
| Derived cache | Invalidate/recompute, do not save as authoritative data |
| Plan modified after Ordering | Restore a complete clone; properties alone do not undo structural modifications |
| Source data borrowed by a plan | Release the plans/snapshots before destroying the source owner |
| Input updated by apply | Keep the current input; do not restore an old pointer/config by copying a complete object |
| Temporary/service | Respect free/destroy/scope and publish the necessary contents before expiration |

Step callbacks may impose a stricter boundary than the global freeze: a surface consumed by infill is already an input, even though `Print` has not yet been reached. A snapshot must not be used to legitimize a mutation forbidden by the local contract.

## 12. API coverage

Scope: owned or borrowed storage accessible through the shipped ABI, its step callbacks and its C++ views. Scalars returned by copy are grouped; arbitrary private states of external plugins, unexposed algorithm details and GUI internals cannot be enumerated as host members in this reference.

| Header family under `src/libslic3r/Api/plugin/c/` | Documented storage |
|---|---|
| `slic3r_data_tree.h`, `slic3r_volume.h` | Print/objects, model and regions, layers/islands/surfaces, properties and records (§2–4, §8) |
| `slic3r_printing_plan.h` | Hierarchy, catalog, events, identities, source borrows (§5) |
| `slic3r_extrusion_entity.h`, `slic3r_extrusion_polyline.h`, `slic3r_extrusion_property.h`, `slic3r_extrusions.h` | Trees, geometry, payloads and resources, generation parameters (§6, §9–10) |
| `slic3r_geometry.h`, `slic3r_geometry_types.h`, `slic3r_clipper.h`, `slic3r_utils.h` | Allocations, geometries, adapters, copies and buffers (§7) |
| `slic3r_config*.h` | Configs, option families, schema/descriptors and thread-local views (§8) |
| `slic3r_plugin*.h`, `slic3r_orchestrator.h` | Storage, registry/descriptors, contexts and provider pointers (§7–10) |
| `slic3r_bridge_detector.h`, `slic3r_mesh_queries.h` | Geometry sessions/results and ownership of copies (§10) |
| `slic3r_gcode_script.h`, `slic3r_gcode_firmware.h`, `slic3r_gcode_thumbnail.h`, `slic3r_gcode_reader.h` | Runtime config, sessions, buffers and result views (§10) |
| `steps/slic3r_step_*.h` | Write windows of the preceding families; own layer-height/perimeter/infill/support-demand buffers; seam/firmware/reader/thumbnail services (§1, §9–10) |
| `slic3r_def.h`, `slic3r_slicing_step.h`, ABI version | Contract types/values/tables; no additional slicing result storage |

The `Stored*`, `RegionSettings*` views and service wrappers are included when their ownership or copies differ from the underlying C handle. Any newly exposed allocation/collection must add its members, its mutation boundary and its invalidations here, rather than receiving by default the lifetime of its calling context.
