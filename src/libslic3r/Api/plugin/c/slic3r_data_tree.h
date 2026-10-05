///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_data_tree_h_
#define slic3r_data_tree_h_

#define SLIC3R_PLUGIN_API_DATA_TREE_MAJOR 4u
#define SLIC3R_PLUGIN_API_DATA_TREE_MINOR 5u

/*
Data-tree access for a sliced Print.

The main ownership path is Print -> Object -> Layer. A Layer owns LayerRegions
(the raw slices for each PrintRegion) and LayerIslands (the merged geometric
islands). Each LayerIsland references the LayerRegions that intersect it and
owns LayerRegionIslands, which group generated surfaces and extrusions by
region set and extruder. SurfaceCollections own Surface records; each Surface
owns one ExPolygon and carries a type, a runtime id and plugin properties.

Unless a function name starts with storage_new_ or explicitly says that it
creates owned storage, every pointer returned by this header is borrowed. Do
not free borrowed handles. A mutable handle grants mutation only through the
public functions accepting that handle; it does not transfer ownership.

Borrowed handles remain valid while their owning node exists and its containing
collection is not structurally changed. Clearing, replacing, rebuilding or
removing a collection invalidates handles to its elements and descendants.
Indices are traversal positions, not persistent identifiers. Geometry mutation
may also make derived slices, bounding boxes, islands, surfaces or extrusions
stale; use the documented recomputation function before consuming such caches.
Unless a function documents another result, null input is accepted by query
functions and produces zero, an empty value or null, while indexed accessors
return null for an out-of-range index.

Developer guide: [Using Plugin Properties](/doc/plugins/properties.md)

Extrusion guide: [Using Unified Extrusion Entities](/doc/plugins/extrusions.md)
*/

#include <stddef.h>
#include <stdint.h>

#include "slic3r_def.h"
#include "slic3r_extrusions.h"
#include "slic3r_geometry.h"
#include "slic3r_utils.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================= HANDLES ========================= */

typedef struct print_handle print_handle;
typedef struct orchestrator_handle orchestrator_handle;
typedef struct print_region_handle print_region_handle;
typedef struct object_handle object_handle;
typedef struct layer_handle layer_handle;
typedef struct layer_region_handle layer_region_handle;
typedef struct layer_island_handle layer_island_handle;
typedef struct layer_region_island_handle layer_region_island_handle;
typedef struct surface_handle surface_handle;
typedef struct surface_collection_handle surface_collection_handle;
typedef struct plugin_property_container_handle plugin_property_container_handle;
typedef struct config_handle config_handle;

typedef slic3r_property_type plugin_property_type;
typedef uint32_t print_record_id;

#define PLUGIN_PROPERTY_TYPE_INVALID ((plugin_property_type)SLIC3R_PROPERTY_TYPE_INVALID)
#define PRINT_RECORD_ID_INVALID ((print_record_id)0)
#define PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT ((plugin_property_type)SLIC3R_PROPERTY_TYPE_LAYER_SUPPORT)
#define PLUGIN_PROPERTY_TYPE_LAYER_BRIM ((plugin_property_type)SLIC3R_PROPERTY_TYPE_LAYER_BRIM)
#define PLUGIN_PROPERTY_TYPE_LAYER_ADHESION ((plugin_property_type)SLIC3R_PROPERTY_TYPE_LAYER_ADHESION)

/*
Built-in payload stored on auxiliary Layers that represent generated support.

Support layers are ordinary Layer objects in the final data tree. The property
is the part that says "this auxiliary layer is support" and carries the interface_id
value used to alternate support-interface angles.
If a Layer does not have this property, plugin code must treat it as a generic
auxiliary layer, not as support.
*/
typedef struct c_layer_support_property {
    uint32_t interface_id;
    uint32_t reserved;
} c_layer_support_property;

/*
Built-in marker stored on auxiliary Layers that contain object-owned brim.

The payload deliberately has no behavior fields. Its presence says that this
auxiliary layer was created to host brim extrusions belonging to one object.
The reserved field keeps the binary payload non-empty, which is required by the
generic plugin-property backend.
*/
typedef struct c_layer_brim_property {
    uint32_t reserved;
} c_layer_brim_property;

typedef uint32_t raw_layer_adhesion_kind;
typedef uint32_t raw_layer_adhesion_flag;

#define RAW_LAYER_ADHESION_KIND_BRIM  1u
#define RAW_LAYER_ADHESION_KIND_SKIRT 2u

#define RAW_LAYER_ADHESION_FLAG_FIRST_LAYER_ONLY 1u

/*
Built-in marker stored on auxiliary Layers that contain first-layer adhesion.

Auxiliary layers are generic; this payload tells later code whether the layer
contains brim or skirt geometry. `flags` refines the behavior inside that kind.
For skirt, FIRST_LAYER_ONLY means the layer carries the special first-layer
skirt loops that replace the normal skirt only on the first printed Z.
*/
typedef struct c_layer_adhesion_property {
    raw_layer_adhesion_kind kind;
    raw_layer_adhesion_flag flags;
} c_layer_adhesion_property;

typedef struct c_curled_line
{
    c_point a;
    c_point b;
    float curled_height;
} c_curled_line;

/* ========================= SURFACE TYPE ========================= */

/*
Surface type bitmask (mapped from SurfaceType).
Multiple flags can be combined.
*/
typedef uint16_t raw_surface_type;

/* No surface */
#define RAW_SURFACE_TYPE_NONE 0

/* Position flags */

/* Top horizontal surface, visible from the top */
#define RAW_SURFACE_TYPE_POS_TOP (1 << 0)

/* Bottom horizontal surface, visible from the bottom */
#define RAW_SURFACE_TYPE_POS_BOTTOM (1 << 1)

/* Internal horizontal surface position. Its density is selected separately. */
#define RAW_SURFACE_TYPE_POS_INTERNAL (1 << 2)

/* Inner/outer perimeters (mainly for coloring) */
#define RAW_SURFACE_TYPE_POS_PERIMETER (1 << 3)

/* Density flags */

/* Solid infill (100%) */
#define RAW_SURFACE_TYPE_DENS_SOLID (1 << 4)

/* Sparse infill (>0% & <100%) */
#define RAW_SURFACE_TYPE_DENS_SPARSE (1 << 5)

/* Void / combined sparse layers */
#define RAW_SURFACE_TYPE_DENS_VOID (1 << 6)

/* Modifier flags */

/* First bridging layer */
#define RAW_SURFACE_TYPE_MOD_BRIDGE (1 << 7)

/* Second layer over bridge */
#define RAW_SURFACE_TYPE_MOD_OVERBRIDGE (1 << 8)

/* Support modifier: the surface belongs to a support auxiliary layer (support base or interface), not to the object. */
#define RAW_SURFACE_TYPE_MOD_SUPPORT (1 << 9)

/* Check if flag is set */
#define RAW_SURFACE_TYPE_HAS(type, flag) (((type) & (flag)) != 0)

/* Add flag */
#define RAW_SURFACE_TYPE_ADD(type, flag) ((type) |= (flag))

/* Remove flag */
#define RAW_SURFACE_TYPE_REMOVE(type, flag) ((type) &= ~(flag))

/* ========================= PLUGIN PROPERTIES ========================= */

/*
Generic typed metadata attached to data-tree objects.

A plugin property is a numeric-keyed byte payload. It is meant for small C
structs such as:

    typedef struct my_surface_info {
        uint32_t priority;
        uint32_t max_layers;
    } my_surface_info;

Register a stable, namespaced key such as "my_plugin.surface_info" with
orchestrator_register_property(), then use the returned numeric id here. The
host copies these payloads when it copies or splits the owning object, so later
plugins can read metadata created by earlier plugins without knowing native C++
classes.

Normal creation flow:

    plugin_property_type type = orchestrator_register_property(
        orch, "my_plugin.surface_info", sizeof(my_surface_info), alignof(my_surface_info));

    my_surface_info *info = (my_surface_info *)plugin_property_get_or_add_data_mutable(
        orch, surface_collection_get_properties_mutable(scratch_surfaces, index), type);

The host looks up the registered size and alignment from orch. A plugin should
not invent its own size/alignment at the point of use, because that would make
two plugins with the same property name disagree silently.

Const property getters expose reads only. Explicit mutable getters check the
node's pipeline window in Debug. Source surfaces expose only const reads;
authorized step payloads publish replacements assembled in mutable scratch
collections. Scratch metadata does not depend on the current pipeline phase.

Threading rule: the host does not lock individual property containers. Parallel
steps usually assign one layer or island to one worker, which is safe. If a
plugin deliberately lets several workers write properties on the same object at
the same time, that plugin must add its own synchronization.

Important rules:
- store plain C-layout data only;
- do not store owning pointers or strings inside the payload;
- use the same orchestrator that registered the property when creating it;
- if get_or_add returns NULL, the type is unknown to the orchestrator or the
  existing payload layout does not match the registered one;
- data pointers are borrowed from the container and must never be freed.
  Removing a property invalidates its payload pointer; clearing the container
  or copying properties into it invalidates all its previous payload pointers,
  even though the container handle itself remains valid. Reacquire payload
  pointers after structural mutations and never retain them past the owner's
  lifetime.
*/
SLIC3R_HOST_API uint32_t plugin_property_count(const plugin_property_container_handle *me);
SLIC3R_HOST_API plugin_property_type plugin_property_type_at(const plugin_property_container_handle *me, uint32_t idx);
SLIC3R_HOST_API int32_t plugin_property_has(const plugin_property_container_handle *me, plugin_property_type type);
SLIC3R_HOST_API uint32_t plugin_property_data_size(const plugin_property_container_handle *me, plugin_property_type type);
SLIC3R_HOST_API const void *plugin_property_data(const plugin_property_container_handle *me, plugin_property_type type);
SLIC3R_HOST_API void *plugin_property_data_mutable(plugin_property_container_handle *me, plugin_property_type type);
SLIC3R_HOST_API void *plugin_property_get_or_add_data_mutable(orchestrator_handle *orch,
                                                              plugin_property_container_handle *me,
                                                              plugin_property_type type);
SLIC3R_HOST_API int32_t plugin_property_remove(plugin_property_container_handle *me, plugin_property_type type);
SLIC3R_HOST_API void plugin_property_clear(plugin_property_container_handle *me);

/*
Replace every property in dst with an independent byte-for-byte copy of src.
Both containers remain owned by their data-tree entities; this function neither
transfers nor frees either handle. A null dst or src is ignored. Copying an
empty source clears dst, and later mutations of either container are independent.
*/
SLIC3R_HOST_API void plugin_property_copy_all(plugin_property_container_handle *dst,
                                              const plugin_property_container_handle *src);

/* ========================= SURFACE ========================= */

/*
Value snapshot of a Surface.

expolygon is borrowed from the source Surface: c_surface does not own or copy
that geometry, and the pointer must not be freed. It remains valid only until
the source Surface or its containing SurfaceCollection is mutated, cleared or
destroyed. Do not retain this value beyond that lifetime.
*/
typedef struct c_surface
{
    const expolygon_handle *expolygon;
    raw_surface_type type;
    uint64_t id;
} c_surface;
/* Return a zero-initialized value for null; otherwise return the borrowed snapshot above. */
SLIC3R_HOST_API c_surface surface_c_view(const surface_handle *me);

/* Borrow the geometry owned by a read-only Surface view. */
SLIC3R_HOST_API const expolygon_handle *surface_get_expolygon(const surface_handle *me);

/* Surface type bitmask access. Surfaces are read-only views in the public API. */
SLIC3R_HOST_API raw_surface_type surface_get_type(const surface_handle *me);

/*
Runtime id of this Surface.

The id is unique only inside the current host process. It lets generated
extrusion trees remember which Surface produced them during the same slice.
It is not stable across copies, project saves, reloads, or different runs. A
value of 0 means "no host Surface id", which is possible for plugin-owned
temporary surface snapshots.
*/
SLIC3R_HOST_API uint64_t surface_get_id(const surface_handle *me);

/* Convenience helper for one flag inside the surface type bitmask. */
SLIC3R_HOST_API int32_t surface_get_flag(const surface_handle *me, raw_surface_type flag);

/*
Surface properties.

Source surfaces and their metadata are read-only through the global API.
Plugins annotate owned scratch collections before an authorized step payload
publishes a replacement. A const surface handle alone grants no editing rights.
*/
SLIC3R_HOST_API const plugin_property_container_handle *surface_get_properties(const surface_handle *me);
// A mutable collection is an owned scratch capability; source collections are exposed const.
SLIC3R_HOST_API plugin_property_container_handle *surface_collection_get_properties_mutable(surface_collection_handle *me, uint32_t idx);

/*
Surface collection storage.

Surface remains a read-only data-tree view. Plugins create temporary
SurfaceCollections in their storage, fill them with copied or moved ExPolygons,
then pass the whole collection to a step callback that moves it into the host
data tree. The returned handle is owned by storage and must be released through
the storage API unless ownership is explicitly transferred. Null storage
returns null.
*/
SLIC3R_HOST_API surface_collection_handle *storage_new_surface_collection(storage_handle *storage);
/* Clear the collection and invalidate every Surface and ExPolygon handle obtained from it. */
SLIC3R_HOST_API void surface_collection_clear(surface_collection_handle *me);
/* Copy area into a new Surface. The source and destination geometries are independent. */
SLIC3R_HOST_API void surface_collection_append_expolygon_copy(surface_collection_handle *me,
                                                              const expolygon_handle *area,
                                                              raw_surface_type surface_type);
/*
Move area into a new Surface and clear the source ExPolygon. The source handle
remains valid, empty and owned by its original storage, so the caller must still
release that storage normally. ExPolygons carry no Surface properties; the new
Surface starts with an empty property container. Appending may invalidate
previous Surface handles from me.
*/
SLIC3R_HOST_API void surface_collection_append_expolygon_move(surface_collection_handle *me,
                                                              expolygon_handle *area,
                                                              raw_surface_type surface_type);
/* Copy every source ExPolygon into a new Surface with surface_type. */
SLIC3R_HOST_API void surface_collection_append_expolygons_copy(surface_collection_handle *me,
                                                               const expolygon_collection_handle *areas,
                                                               raw_surface_type surface_type);
/*
Move all ExPolygons into new Surfaces and leave areas empty. The source
collection handle remains valid and must still be released by its owner.
No Surface properties are transferred. Appending may invalidate previously
borrowed Surface handles from me.
*/
SLIC3R_HOST_API void surface_collection_append_expolygons_move(surface_collection_handle *me,
                                                               expolygon_collection_handle *areas,
                                                               raw_surface_type surface_type);
/* Null collections have size zero. */
SLIC3R_HOST_API uint32_t surface_collection_size(const surface_collection_handle *me);
/* Borrow element idx; return null for null or out-of-range input. Never free the result. */
SLIC3R_HOST_API const surface_handle *surface_collection_at(const surface_collection_handle *me, uint32_t idx);
/* Mutable borrowed element access with the same null, lifetime and invalidation rules. */
SLIC3R_HOST_API surface_handle *surface_collection_at_mutable(surface_collection_handle *me, uint32_t idx);

/* ========================= LAYER ========================= */

/* Return the layer thickness in scaled coord_t units. Null returns zero. */
SLIC3R_HOST_API coord_t layer_get_height(const layer_handle *me);
/* Return the printing Z position in scaled coord_t units. Null returns zero. */
SLIC3R_HOST_API coord_t layer_get_print_z(const layer_handle *me);
/* Center Z of the slicing plane. It is exactly print_z - height / 2. */
SLIC3R_HOST_API coord_t layer_get_slice_z(const layer_handle *me);

/*
Borrow the Object that owns this normal or auxiliary layer. The returned handle
must not be freed and remains valid only while the owning Object and its layer
storage remain alive. A null layer returns null. This function does not mutate
the layer or object.
*/
SLIC3R_HOST_API const object_handle *layer_get_object(const layer_handle *me);
/*
Borrow the merged Layer slices. The collection may be empty but is non-null for
a valid Layer. Its elements become stale when Layer slices are edited or
recomputed; the collection and its elements must not be freed.
*/
SLIC3R_HOST_API const expolygon_collection_handle *layer_get_slices(const layer_handle *me);

/*
Return the number of curled-line estimates attached to this layer by the host
curl-estimation step. These estimates may be absent when that step has not run.
A null layer returns zero. This function does not allocate or mutate data.
*/
SLIC3R_HOST_API uint32_t layer_count_curled_line(const layer_handle *me);

/*
Copy the curled-line estimate at idx. Endpoints use scaled XY coordinates and
curled_height is expressed in millimeters. The returned structure owns no host
memory and needs no release. A null layer or an out-of-range index returns a
zero-initialized structure; use layer_count_curled_line() to validate idx.
*/
SLIC3R_HOST_API c_curled_line layer_get_curled_line(const layer_handle *me, uint32_t idx);

/*
Borrow an adjacent layer when the host linked one. Boundary layers and unlinked
auxiliary layers return null. These links and handles remain valid only while
the owning Object's relevant layer collection is unchanged. Mutable variants
do not transfer ownership.
*/
SLIC3R_HOST_API layer_handle *layer_get_upper_layer_mutable(layer_handle *me);
SLIC3R_HOST_API const layer_handle *layer_get_upper_layer(const layer_handle *me);

SLIC3R_HOST_API layer_handle *layer_get_lower_layer_mutable(layer_handle *me);
SLIC3R_HOST_API const layer_handle *layer_get_lower_layer(const layer_handle *me);

/*
Borrow the Layer's property container; it dies with its owner and must not be freed.
Main layer/region/island properties are writable through STEP_POST_SLICING;
object, auxiliary and region-island properties through STEP_FINALIZE_SLICING.
Boundaries are inclusive and checked by mutable getters in Debug; reads remain
available afterwards. Steps from Ordering onwards write their results into the plan.
*/
SLIC3R_HOST_API const plugin_property_container_handle *layer_get_properties(const layer_handle *me);
// Main layers are writable through PostSlicing; auxiliary layers through FinalizeSlicing, inclusive.
SLIC3R_HOST_API plugin_property_container_handle *layer_get_properties_mutable(layer_handle *me);

/*
Enumerate LayerRegions owned by this Layer. A null Layer has count zero;
accessors return null for invalid indices. Returned handles are borrowed and
must not be freed. Reinitializing or destroying the Layer's region collection
invalidates all of them; mutable access does not transfer ownership.
*/
SLIC3R_HOST_API uint32_t layer_count_region(const layer_handle *me);
SLIC3R_HOST_API layer_region_handle *layer_get_region_mutable(layer_handle *me, uint32_t idx);
SLIC3R_HOST_API const layer_region_handle *layer_get_region(const layer_handle *me, uint32_t idx);

/*
Enumerate merged LayerIslands owned by this Layer, with the same null and index
rules as LayerRegions. Rebuilding islands invalidates every previously returned
LayerIsland and descendant LayerRegionIsland handle.
*/
SLIC3R_HOST_API uint32_t layer_count_island(const layer_handle *me);
SLIC3R_HOST_API layer_island_handle *layer_get_island_mutable(layer_handle *me, uint32_t idx);
SLIC3R_HOST_API const layer_island_handle *layer_get_island(const layer_handle *me, uint32_t idx);

/* ========================= LAYER REGION ========================= */

/* Borrow this LayerRegion's metadata container; null returns null. */
SLIC3R_HOST_API const plugin_property_container_handle *layer_region_get_properties(const layer_region_handle *me);
SLIC3R_HOST_API plugin_property_container_handle *layer_region_get_properties_mutable(layer_region_handle *me);

/*
Compute normal flow at this LayerRegion's layer height. Support and support
interface use the object's support flow settings; both use first-layer support
flow when the layer's bottom touches the bed, whatever its id (an auxiliary
layer's id is its creation rank). Other recognized
flags select perimeter, top or solid flow; remaining values select infill.
The result needs no release. A null region or a flow that cannot be calculated
returns all zero fields; no exception crosses this C function. Use
layer_region_get_bridging_flow() for bridge/overhang rules.
*/
SLIC3R_HOST_API c_flow layer_region_get_flow(const layer_region_handle *me, raw_extrusion_role flow_role);

/*
Compute bridge/overhang flow for this LayerRegion. For support and support
interface, the bridge diameter is the layer height and the nozzle comes from
the corresponding object support flow. Other roles use regional bridge rules;
unrecognized flags select infill. The result needs no release. A null region or
a flow that cannot be calculated returns all zero fields; no exception crosses
this C function.
*/
SLIC3R_HOST_API c_flow layer_region_get_bridging_flow(const layer_region_handle *me, raw_extrusion_role flow_role);
/*
Borrow the raw slices assigned to this LayerRegion. The collection is owned by
the region and may be empty. Any direct mutation or replacement invalidates its
element handles and leaves Layer slices/islands stale until recomputed.
*/
SLIC3R_HOST_API const expolygon_collection_handle *layer_region_get_slices(const layer_region_handle *me);
/* Copy the XY bounding box of all current raw slices; null returns a zero value. */
SLIC3R_HOST_API c_bounding_box layer_region_get_bounding_box(const layer_region_handle *me);

/* Borrow the owning Layer; null input returns null. Never free the result. */
SLIC3R_HOST_API const layer_handle *layer_region_get_layer(const layer_region_handle *me);
/* Borrow the shared PrintRegion supplying this region's configuration. */
SLIC3R_HOST_API const print_region_handle *layer_region_get_print_region(const layer_region_handle *me);

/* ========================= LAYER ISLAND ========================= */

/*
Borrow the island's defining ExPolygon. The mutable form edits the source used
to rebuild Layer slices; after mutation, call layer_recompute_slices_from_islands()
to synchronize that Layer collection. This call does not refresh the island's
cached bounding box, infill areas or other derived perimeter data. Code that
needs those caches must arrange for the owning pipeline step to rebuild the
island. Main slices are editable through STEP_POST_SLICING, auxiliary slices
through STEP_FINALIZE_SLICING, inclusive. Neither form transfers ownership.
Null input returns null.
*/
SLIC3R_HOST_API expolygon_handle *layer_island_get_slice_mutable(layer_island_handle *me);
SLIC3R_HOST_API const expolygon_handle *layer_island_get_slice(const layer_island_handle *me);
/* Copy the cached island bounding box; null returns a zero value. */
SLIC3R_HOST_API c_bounding_box layer_island_get_bounding_box(const layer_island_handle *me);
/*
Borrow the first ExPolygon from infill_areas, or return null when no infill area
exists. This is a legacy single-area convenience, not a union of all areas.
*/
SLIC3R_HOST_API const expolygon_handle *layer_island_get_infill_slice(const layer_island_handle *me);
/*
Borrow every area where infill may be generated. Perimeter processing may split
one island into multiple disconnected areas, and these areas may overlap the
perimeter domain to provide infill anchoring. A valid island returns a borrowed
collection owned by that island; it may be empty and must not be freed. Later perimeter/infill
reprocessing may replace it and invalidate its element handles.
*/
SLIC3R_HOST_API const expolygon_collection_handle *layer_island_get_infill_areas(const layer_island_handle *me);
/* Copy the bounding box of the first infill area, or an empty box when absent. */
SLIC3R_HOST_API c_bounding_box layer_island_get_infill_bounding_box(const layer_island_handle *me);
/*
Borrow the first ExPolygon from infill_no_overlap_areas, or return null when the
collection is empty. It is a convenience view, not the complete free domain.
*/
SLIC3R_HOST_API const expolygon_handle *layer_island_get_infill_no_overlap_slice(const layer_island_handle *me);
/*
Borrow all areas where infill may be generated without encroaching on
perimeters. Unlike infill_areas, these areas exclude the anchoring overlap.
An empty collection means there is no overlap restriction to apply; consumers
normally keep infill_areas unchanged rather than treating it as no infill.
Never free or retain element handles across perimeter/infill reprocessing.
*/
SLIC3R_HOST_API const expolygon_collection_handle *layer_island_get_infill_no_overlap_areas(const layer_island_handle *me);
/*
Borrow the areas of this island that ask for support. STEP_SUPPORT_DEMAND
writes them (it runs right after STEP_POST_SLICING); plugins up to STEP_SUPPORT
may refine them with support_demand_get_island_mutable() (bridge removal in
STEP_POST_PERIMETER); support generation reads them. Any step may read them;
before STEP_SUPPORT_DEMAND the collection is empty. A valid island
returns a collection owned by that island; it may be empty and must not be
freed. Null returns null.
*/
SLIC3R_HOST_API const expolygon_collection_handle *layer_island_get_support_demand(const layer_island_handle *me);

/* Borrow this island's metadata container; null returns null. */
SLIC3R_HOST_API const plugin_property_container_handle *layer_island_get_properties(const layer_island_handle *me);
SLIC3R_HOST_API plugin_property_container_handle *layer_island_get_properties_mutable(layer_island_handle *me);

/*
Enumerate the borrowed LayerRegions intersecting this island. Null has count
zero and invalid indices return null. Mutable access changes the referenced
LayerRegion, not the membership of this island.
*/
SLIC3R_HOST_API uint32_t layer_island_count_region(const layer_island_handle *me);
SLIC3R_HOST_API layer_region_handle *layer_island_get_region_mutable(layer_island_handle *me, uint32_t idx);
SLIC3R_HOST_API const layer_region_handle *layer_island_get_region(const layer_island_handle *me, uint32_t idx);

/*
Enumerate LayerRegionIsland work buckets owned by this island. Returned handles
are borrowed; adding or rebuilding buckets may invalidate earlier handles.
*/
SLIC3R_HOST_API uint32_t layer_island_count_region_island(const layer_island_handle *me);
/* Source editing capabilities are available through STEP_FINALIZE_SLICING inclusive. */
SLIC3R_HOST_API layer_region_island_handle *layer_island_get_region_island_mutable(layer_island_handle *me, uint32_t idx);
SLIC3R_HOST_API const layer_region_island_handle *layer_island_get_region_island(const layer_island_handle *me, uint32_t idx);

/*
Return the LayerRegionIsland for one island, one region set, and one extruder.

LayerRegionIslands are the mutable work buckets shared by perimeter, surface,
infill, and post-process plugins. A plugin uses this function when it already
knows which regions and which extruder must own the output. The function only
validates and mutates the data tree; it does not decide which extruder a role
should use.

regions must contain LayerRegion handles that belong to island. Passing
regions == NULL or region_count == 0 means "all regions of the island".
extruder_id is a zero-based extruder index. EXTRUDER_ID_UNKNOWN stores the
generic "no single extruder" value. Any other negative value is rejected.

Editing is allowed through STEP_FINALIZE_SLICING inclusive, before any bucket creation.
Returns NULL when island is invalid, when any region is invalid or belongs to a
different island, or when the final region set is empty.
*/
SLIC3R_HOST_API layer_region_island_handle *layer_island_get_or_create_region_island(
    layer_island_handle *island,
    const layer_region_handle *const *regions,
    uint32_t region_count,
    extruder_index_t extruder_id);

/* Borrow the Layer that owns this island; null input returns null. */
SLIC3R_HOST_API const layer_handle *layer_island_get_layer(const layer_island_handle *me);

/*
Borrow overlap links to islands on adjacent object layers. Counts are zero for
null input; indexed access returns null out of range. Links are derived caches
and become stale when either layer's islands or overlap graph is rebuilt.
*/
SLIC3R_HOST_API uint32_t layer_island_count_lower_island(const layer_island_handle *me);
SLIC3R_HOST_API const layer_island_handle *layer_island_get_lower_island(const layer_island_handle *me, uint32_t idx);
SLIC3R_HOST_API uint32_t layer_island_count_upper_island(const layer_island_handle *me);
SLIC3R_HOST_API const layer_island_handle *layer_island_get_upper_island(const layer_island_handle *me, uint32_t idx);

/* ========================= LAYER REGION ISLAND ========================= */

/* Return the zero-based extruder id, or EXTRUDER_ID_UNKNOWN for null/no single extruder. */
SLIC3R_HOST_API extruder_index_t layer_region_island_extruder_id(const layer_region_island_handle *me);
/* Null handles report no extrusions. */
SLIC3R_HOST_API int32_t layer_region_island_has_extrusions(const layer_region_island_handle *me);
/* Report whether this bucket contains an extrusion root for role. */
SLIC3R_HOST_API int32_t layer_region_island_has_extrusion(const layer_region_island_handle *me, raw_extrusion_role role);
/*
Borrow the mutable extrusion collection for role, creating the role bucket when
needed, through STEP_FINALIZE_SLICING inclusive. The same root capability grants
geometry and extrusion-property editing. Null returns null. Mutating its tree may invalidate descendant extrusion
handles; do not free the returned root.
*/
SLIC3R_HOST_API extrusion_entity_handle *layer_region_island_get_mutable_extrusion(layer_region_island_handle *me, raw_extrusion_role role);
/* Borrow the role's extrusion root, or return null when that role is absent. */
SLIC3R_HOST_API const extrusion_entity_handle *layer_region_island_get_extrusion(const layer_region_island_handle *me,
                                                          raw_extrusion_role role);

/*
Island-level fill surfaces produced by STEP_SURFACE_GENERATION and refined by
later surface-processing steps. LayerRegion no longer exposes these caches in
the plugin ABI; plugins should read/write fill surfaces through the
LayerRegionIsland that owns the infill work.
*/
/*
Borrow the complete fill-surface collection. A valid group may return an empty
collection; null input returns null. Replacing or editing fill surfaces may
invalidate Surface handles previously obtained from this group.
*/
SLIC3R_HOST_API const surface_collection_handle *layer_region_island_get_fill_surfaces(const layer_region_island_handle *me);
/* Null groups have zero fill surfaces. */
SLIC3R_HOST_API uint32_t layer_region_island_count_fill_surface(const layer_region_island_handle *me);
/* Borrow one fill Surface; return null for null or out-of-range input. */
SLIC3R_HOST_API const surface_handle *layer_region_island_get_fill_surface(const layer_region_island_handle *me, uint32_t idx);

/*
Replace an auxiliary layer's island-region fill surfaces. Build surfaces with
storage_new_surface_collection(); a successful call moves the complete content
into the data tree and leaves the source valid and empty. NULL surfaces clears
the destination. Replacing surfaces invalidates its previous Surface handles.

Only island-regions associated with their object's auxiliary layers are
accepted. A null destination or an object-layer island-region returns 0 and
changes neither collection. Auxiliary editing is allowed through
STEP_FINALIZE_SLICING inclusive, or while idle. Outside that window the standard
freeze check applies (Debug assertion, or rejection and cancellation in strict
freeze mode). Returns 1 on success. Never pass a collection from the data tree
as the source; its container must remain in plugin storage.
*/
SLIC3R_HOST_API int32_t layer_region_island_set_fill_surfaces(layer_region_island_handle *region_island,
                                                           surface_collection_handle *surfaces);

/* Borrow this work bucket's metadata container; null returns null. */
SLIC3R_HOST_API const plugin_property_container_handle *layer_region_island_get_properties(const layer_region_island_handle *me);
SLIC3R_HOST_API plugin_property_container_handle *layer_region_island_get_properties_mutable(layer_region_island_handle *me);

/*
Return the number of LayerRegion objects grouped by this LayerRegionIsland.
Surface-generation plugins use these regions when refining an existing group
instead of starting again from the whole LayerIsland. A null handle returns
zero. This function does not allocate or mutate the group.
*/
SLIC3R_HOST_API uint32_t layer_region_island_count_region(const layer_region_island_handle *me);

/*
Borrow the LayerRegion at idx within this group. The returned
handle must not be freed and may be invalidated by later data-tree mutations.
A null group or out-of-range index returns null. Region ordering is host-owned;
callers must not treat idx as a persistent identifier.
*/
SLIC3R_HOST_API const layer_region_handle *layer_region_island_get_region(const layer_region_island_handle *me,
                                                                          uint32_t idx);

/* ========================= PRINT REGION ========================= */

/*
Borrow an immutable PrintRegion configuration. Null returns null. A registered
region and its configuration remain valid across apply() and invalidation until
Print::clear() or Print destruction. Neither handle may be freed or mutated.
The region can remain alive after it stops being used by any object or layer.
Only regional options belong to this snapshot; resolve object/print options
through their respective configs rather than a mutable parent-config fallback.
*/
SLIC3R_HOST_API const config_handle *print_region_get_config(const print_region_handle *me);

/* ========================= OBJECT ========================= */

/*
Borrow the Object configuration. Mutable access edits the live configuration,
without making a copy or persisting it. Null returns null; neither handle may
be freed or retained after its Object dies. Mutable access requires STEP_NONE
on the owning Print; objects without a Print remain editable.
*/
SLIC3R_HOST_API config_handle *object_get_config_mutable(object_handle *me);
SLIC3R_HOST_API const config_handle *object_get_config(const object_handle *me);
/* Borrow read-only metadata; the container remains owned by the Object. */
SLIC3R_HOST_API const plugin_property_container_handle *object_get_properties(const object_handle *me);
/* Explicit editing through STEP_FINALIZE_SLICING inclusive, checked in Debug. */
SLIC3R_HOST_API plugin_property_container_handle *object_get_properties_mutable(object_handle *me);

/* Copy the UTF-8 ModelObject name. Print-level auxiliary Objects have no
   ModelObject and therefore report an empty name. The return value excludes
   the null terminator; out may be NULL for a sizing query. */
SLIC3R_HOST_API uint32_t object_get_name(const object_handle *me, char *out, uint32_t capacity);

/*
Deprecated legacy object-local maximum Z in scaled coord_t units. Model-backed Objects use
the source ModelObject maximum; the hidden auxiliary Object uses its current
height. The value does not include instance placement and mixes two definitions,
so new slicing code should use the step-specific maximum Z or the last Layer's
print_z/height according to its purpose. Null returns zero.
*/
SLIC3R_HOST_API coord_t object_get_max_z(const object_handle *me);

/*
Transformation from the source model object into this printable Object.

The row-major c_matrix4d uses unscaled millimeter coordinates and is applied to
a model-space homogeneous column vector. It is the native PrintObject::trafo():
rotation, scaling, mirroring and Z translation are included, but the temporary
XY centering offset used to keep Clipper coordinates small is not. For a source
point p_model in millimeters, reproduce trafo_centered() as:

    center_mm = (unscaled(center.x), unscaled(center.y), 0)
    p_centered = translation(-center_mm) * object_get_transform(me) * p_model

where center is object_get_center_offset(). Instance XY shifts are separate and
remain scaled c_point values. Null returns a zero-initialized matrix.
*/
SLIC3R_HOST_API c_matrix4d object_get_transform(const object_handle *me);

/*
Scaled XY offset used by the host while slicing to center the mesh before it is
sent to Clipper. This is a slicer-space 2D value, so it uses coord_t through
c_point. Convert with unscaled() before composing it with c_matrix4d.
*/
SLIC3R_HOST_API c_point object_get_center_offset(const object_handle *me);

/*
Return the number of physical instances of this printable Object. Extrusion
trees stored on its Layers remain in object-local coordinates and are not
duplicated by this call. A null object returns zero.
*/
SLIC3R_HOST_API uint32_t object_count_instance(const object_handle *me);

/*
Copy the scaled XY translation of instance idx. Add this shift when cloning an
object-local extrusion tree for that physical instance. The returned c_point
owns no host memory. A null object or out-of-range index returns (0, 0); use
object_count_instance() to validate idx. Indices are stable only for the current
Print and are not persistent object identifiers.
*/
SLIC3R_HOST_API c_point object_get_instance_shift(const object_handle *me, uint32_t idx);

/*
Enumerate normal object Layers. Null has count zero and indexed access returns
null out of range. Handles must not be freed and remain valid only while the
Object's layer collection is unchanged; mutable access does not transfer
ownership.
*/
SLIC3R_HOST_API uint32_t object_count_layer(const object_handle *me);
SLIC3R_HOST_API layer_handle *object_get_layer_mutable(object_handle *me, uint32_t idx);
SLIC3R_HOST_API const layer_handle *object_get_layer(const object_handle *me, uint32_t idx);
/*
Auxiliary layers share the Layer shape used by normal object layers: print_z, height,
slices(), islands(), and plugin properties work the same way. A plugin should
treat them as geometry already produced by earlier steps and must not assume
that their indices match normal object layer indices.

Support is represented by the built-in PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT
property on the Layer. Read it through layer_get_properties() and the generic
plugin_property_* functions; an auxiliary layer without that property is not a
support layer.
*/
/* Null Objects contain zero auxiliary layers. Indices follow ascending print Z. */
SLIC3R_HOST_API uint32_t object_count_auxiliary_layer(const object_handle *me);
/*
Return a mutable auxiliary Layer.

Auxiliary layers are plugin-owned working layers. They are reachable through a
const object_handle because plugins often receive read-only Object views while
still being responsible for generated helper geometry such as support, skirt,
brim or wipe tower. Mutability is intentionally limited to the returned Layer
and its plugin/public layer APIs; it does not make normal object layers mutable.
The handle is borrowed, must not be freed, and is null for an invalid index.
Adding or removing a layer may change indices, but handles to other auxiliary
Layer objects remain valid until those particular layers are removed.
*/
SLIC3R_HOST_API layer_handle *object_get_auxiliary_layer(const object_handle *me, uint32_t idx);
/*
Create an empty auxiliary Layer and initialize its LayerRegion list from the
Object's PrintRegions.

height, print_z and slice_z are scaled slicer coordinates. slice_z may equal
print_z when the helper geometry lives on the print plane. The new layer is not
automatically filled; callers should write raw LayerRegion slices and then call
layer_recompute_slices_and_islands_from_layer_regions(). The returned handle is
borrowed from the Object. Null Object, non-positive height or non-positive
print_z returns null. Insertion keeps print-Z order and may change indices of
existing auxiliary layers. Only steps up to STEP_FINALIZE_SLICING may add or
remove auxiliary layers (debug builds assert it).
*/
SLIC3R_HOST_API layer_handle *object_add_auxiliary_layer(const object_handle *me,
                                                         coord_t height,
                                                         coord_t print_z,
                                                         coord_t slice_z);
/*
Remove an auxiliary Layer previously returned by object_get_auxiliary_layer() or
object_add_auxiliary_layer(). Returns non-zero on success and immediately
invalidates layer plus every descendant handle borrowed from it. Other
auxiliary Layer objects remain alive, but indices after the removed position
shift down by one. Null, foreign or already removed handles return zero.
Only steps up to STEP_FINALIZE_SLICING may call it (debug builds assert it).
*/
SLIC3R_HOST_API int32_t object_remove_auxiliary_layer(const object_handle *me, layer_handle *layer);

/*
Generic raw-slice mutation helpers.

These functions are intentionally not tied to STEP_POST_SLICING. A plugin that
creates an auxiliary layer in a later step still has to rebuild the Layer's
cached island list after writing LayerRegion raw slices. If the layer should be
used by perimeter/surface/infill code immediately, call
layer_add_regions_to_islands() after recomputing; it ignores layers without
islands. Reconstruction must run before the island layout is locked by
layer_add_regions_to_islands(). Do not mutate this geometry while later steps
are consuming it.
*/
/*
Borrow the mutable merged-slice collection owned by me. Null returns null.
Editing it invalidates handles to its ExPolygons and may desynchronize islands
and LayerRegion slices. layer_recompute_slices_from_islands() overwrites these
edits with the current island geometry; it does not propagate them to islands.
To rebuild islands, edit raw LayerRegion slices instead and call
layer_recompute_slices_and_islands_from_layer_regions().
*/
SLIC3R_HOST_API expolygon_collection_handle *layer_borrow_mutable_slices(layer_handle *me);
/*
Borrow mutable raw slices from a LayerRegion, including through a const region
view. Null returns null. Editing invalidates element handles and leaves the
owning Layer's merged slices and islands stale. Slices of normal layers are
frozen after STEP_POST_SLICING, those of auxiliary layers after
STEP_FINALIZE_SLICING (debug builds assert it).
*/
SLIC3R_HOST_API expolygon_collection_handle *layer_region_borrow_mutable_slices(const layer_region_handle *me);
/*
Replace Layer merged slices with copies of the current island slices. Existing
LayerIsland objects and their handles remain valid; handles to elements of the
Layer slice collection become invalid. This does not recompute island bounding
boxes, region membership, infill areas or extrusions. Null is ignored.
*/
SLIC3R_HOST_API void layer_recompute_slices_from_islands(layer_handle *me);
/*
Rebuild and replace the Layer merged slices and every LayerIsland from current
LayerRegion raw slices. All previously borrowed LayerIsland, LayerRegionIsland
and Layer-slice element handles become invalid. LayerRegion objects remain the
inputs and are not replaced. Null is ignored.
*/
SLIC3R_HOST_API void layer_recompute_slices_and_islands_from_layer_regions(layer_handle *me);
/*
Populate each existing island's region membership after reconstruction. This
locks the island layout for downstream processing. Existing LayerRegionIsland
work buckets are not replaced. Null Layers and Layers without islands are
ignored.
*/
SLIC3R_HOST_API void layer_add_regions_to_islands(layer_handle *me);

/*
Enumerate PrintRegions used by this Object. Counts are zero for null; accessors
return null for invalid indices. Handles and configurations are borrowed from
the Print and must not be freed. Regions are immutable and remain valid until
Print::clear()/destruction, even if their local associations are replaced.
*/
SLIC3R_HOST_API uint32_t object_count_region(const object_handle *me);
SLIC3R_HOST_API const print_region_handle *object_get_print_region(const object_handle *me, uint32_t idx);

/*
Flags of object_get_support_flow(). They are applied in this order: the base
flow, the bottom-contact conversion, then the bridge or the print height.
*/
/*
Use the support first-layer flow settings for the base flow, whether the role
asks for base or interface. The role must still carry the support bit.
*/
#define OBJECT_SUPPORT_FLOW_FIRST_LAYER ((uint32_t)(1u << 0))
/*
Convert the base flow into the flow of a bottom contact, which rests on the
object. The bridge flow ratio is the average bridge_flow_ratio of the object's
regions. With soluble support (support_material_contact_distance_type "none")
the base flow keeps its spacing and its cross section is multiplied by that
ratio. Otherwise the result is a round bridge thread of diameter
sqrt(ratio) * nozzle, so its height is the bottom-contact thickness.
*/
#define OBJECT_SUPPORT_FLOW_BOTTOM_CONTACT ((uint32_t)(1u << 1))
/*
Return a round bridge thread whose diameter is print_height_mm, extruded by the
nozzle of the base flow. This is how a contact bridging over air is printed at
the height of its own layer.
*/
#define OBJECT_SUPPORT_FLOW_BRIDGE ((uint32_t)(1u << 2))

/*
Compute a support flow of this Object at any height, natively, from the same
settings and functions as the legacy support generator. Use it when the flow is
needed before a support layer exists or at another height than the layer's own,
for example the nominal support layer height. layer_region_get_flow() remains
the flow of an existing layer at its own height.

flow_role selects the base flow: RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL_INTERFACE
(support bit plus external bit) for the interface flow, any other role with the
support bit for the support base flow; other roles return a zero flow.
flow_height_mm is the height the base flow is computed for, in millimeters; a
value <= 0 uses the native default (the configured support layer height, or
the object's first layer height with OBJECT_SUPPORT_FLOW_FIRST_LAYER). These
defaults are not rounded to the machine Z step, whereas the legacy support
parameters round their first layer and support heights (check_z_step()):
callers needing exact parity pass the height they computed themselves. Like
the native functions, the base and interface heights are capped at the
extrusion width.

After the flags above, a print_height_mm > 0 keeps the width and recomputes the
spacing and section for that printed height (Flow::with_height()), as when a
nominal flow prints a thinner or thicker layer. It is ignored for bridge
threads; it is required with OBJECT_SUPPORT_FLOW_BRIDGE.

A null Object, an unknown role, a bridge without print height or settings the
host cannot compute return a flow with all fields zero; no exception crosses
this C function. The result is a value and needs no release.
*/
SLIC3R_HOST_API c_flow object_get_support_flow(const object_handle *me,
                                               raw_extrusion_role flow_role,
                                               double flow_height_mm,
                                               double print_height_mm,
                                               uint32_t flags);

/*
Compute the flow of one of this Object's PrintRegions (index as for
object_get_print_region()) for a perimeter or infill role at any height, with
the native PrintRegion::flow() rules. layer_id selects the per-layer width
rules: 0 is the first layer (first-layer widths), an odd id applies the
"change on odd layers" settings, and an even id >= 2 gives the ordinary width.
The support generator, for example, reads the external perimeter width with
layer_id 2. Role flags map as in layer_region_get_flow(); support roles are not
regional and return a zero flow. A null Object, an invalid index, a height
<= 0 or settings the host cannot compute also return a zero flow; no exception
crosses this C function.
*/
SLIC3R_HOST_API c_flow object_get_region_flow(const object_handle *me,
                                              uint32_t region_idx,
                                              raw_extrusion_role flow_role,
                                              double height_mm,
                                              uint32_t layer_id);
/*
The millimeter values of a flow, as the host computed them.

c_flow stores scaled integers, truncated from the host's float millimeters, so
unscaled(c_flow.spacing) can differ from the host spacing by up to one scaled
unit. A plugin that must repeat a host computation exactly (for example the
size of a grid cell, the configured gap plus the flow spacing) reads
these values instead. width, spacing, height and nozzle_diameter are the float
values of the host flow widened to double; zero means the host could not
compute the flow.
*/
typedef struct c_flow_mm
{
    double width;
    double spacing;
    double height;
    double nozzle_diameter;
} c_flow_mm;

/*
The same flows as object_get_support_flow() and object_get_region_flow(), with
the same arguments and the same rules, returned in millimeters. Input or
settings the host cannot compute return all fields zero.
*/
SLIC3R_HOST_API c_flow_mm object_get_support_flow_mm(const object_handle *me,
                                                     raw_extrusion_role flow_role,
                                                     double flow_height_mm,
                                                     double print_height_mm,
                                                     uint32_t flags);
SLIC3R_HOST_API c_flow_mm object_get_region_flow_mm(const object_handle *me,
                                                    uint32_t region_idx,
                                                    raw_extrusion_role flow_role,
                                                    double height_mm,
                                                    uint32_t layer_id);

/* ========================= PRINT ========================= */

/*
Copy the current Print regional defaults and apply config's supplied options.
Unknown/non-regional keys, mismatched option types, null input or allocation
failure return null without publishing a partial region. Equal completed configs
share a handle within this Print (including registered plugin options).

The returned immutable handle is Print-owned until clear()/destruction; never
free it. Changing config or Print defaults later does not change this snapshot.
Registration does not attach it to geometry or add it to active region traversal.
Concurrent registrations and reads are allowed; apply(), clear(), destruction
and changes to Print defaults require that all workers have stopped.
*/
SLIC3R_HOST_API const print_region_handle *print_register_print_region_config(
    print_handle *me, const config_handle *config);

/*
Borrow the Print configuration; null returns null and ownership stays with Print.
Mutable access edits the live configuration without cloning or persisting it.
It is available only outside pipeline steps (STEP_NONE).
*/
SLIC3R_HOST_API config_handle *print_get_config_mutable(print_handle *me);
SLIC3R_HOST_API const config_handle *print_get_config(const print_handle *me);

/*
Variable record channels owned by one Print.

Each channel contains one DynamicConfig. An empty channel name is valid and is
reserved for generic print metadata; named channels should be namespaced by
their producer. Config handles are borrowed and remain valid until that channel
is removed or the Print is cleared.

Channel mutation is not thread-safe. Build or update records from a sequential
pipeline callback, then keep them unchanged while parallel callbacks read them.
Only print_records_allocate_id() may be called concurrently.
*/
/*
Return a borrowed thread-local snapshot of channel names. Its array and strings
remain valid until the next call on the same thread; copy them to retain them
longer. Do not free them. A null Print returns an empty list.
*/
SLIC3R_HOST_API const_strings_t print_records_channels(const print_handle *me);
/* Borrow the channel configuration, or return null if absent. */
SLIC3R_HOST_API const config_handle *print_records_get(const print_handle *me, const char *channel);
/* Create or borrow the mutable channel configuration. Records are frozen
   after STEP_FINALIZE_SLICING (debug builds assert it). */
SLIC3R_HOST_API config_handle *print_records_get_or_add(const print_handle *me, const char *channel);
/* Remove the channel and invalidate its previously returned config handle. */
SLIC3R_HOST_API int32_t print_records_remove(const print_handle *me, const char *channel);
SLIC3R_HOST_API print_record_id print_records_allocate_id(const print_handle *me);

/* Null Prints contain zero printable Objects. */
SLIC3R_HOST_API uint32_t print_count_object(const print_handle *me);

/*
Borrow printable Object idx. Null or out-of-range input returns null. The handle
must not be freed and remains valid only while the Print owns that Object;
mutable access does not transfer ownership.
*/
SLIC3R_HOST_API object_handle *print_get_object_mutable(print_handle *me, uint32_t idx);
SLIC3R_HOST_API const object_handle *print_get_object(const print_handle *me, uint32_t idx);
/*
Return the hidden Object that owns print-level auxiliary layers.

This Object is not counted by print_count_object() and is never returned by
print_get_object(). It exists only so plugins can store global helper geometry
with the same Object auxiliary-layer API used by object-local support, brim,
skirt and future generated layers. The returned Object is mutable because
auxiliary layers are plugin-owned work data; normal model-derived objects stay
reachable only through the regular object list above. The handle is borrowed
from Print and becomes invalid when that Print is cleared or destroyed.
*/
SLIC3R_HOST_API object_handle *print_get_auxiliary_object(const print_handle *me);

#ifdef __cplusplus
}
#endif

#endif // slic3r_data_tree_h_
