///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_Plugins_Ordering_PlanBuilderCommon_hpp_
#define slic3r_Plugins_Ordering_PlanBuilderCommon_hpp_

#include <cstdint>
#include <map>
#include <optional>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"

/*
Shared construction helpers for ordering plugins turn borrowed Print views into
plan-owned extrusion clones. SourceInstance caches placements for object copies;
source_instances preserves source order and ordered_source_instances_for_object_plan
applies complete_objects_sort. Group helpers merge source layers by scaled print Z
and extruder, while append helpers copy role buckets and translate their geometry.
Source views must remain alive throughout construction. Mutating groups can
invalidate child views, so retain indices and fetch fresh views after appends.
Cancellation and allocation failures propagate; partially built plans remain owned
by their caller.

Typical use after obtaining a plan, group, source layer and instance placement:
    std::map<coord_t, uint32_t> indices;
    PrintingLayerGroup layers = layer_group_for_print_z(group, indices, layer.print_z(), ctx);
    append_layer_once(layers, layer, ctx);
    append_layer_instance_extrusions(plan, layers, layer, shift, catalog_id, ctx);
    sort_layer_groups_by_print_z(group, ctx);
The optional LayerExtrusionFilter restricts only the extrusion traversal; callers
still choose which layer groups and source-layer context entries to create.
The by-layer object and global auxiliary helpers also select layer context by the
filter's Z interval, allowing a shared final group above an object-step limit.
*/

namespace slic3r_api { namespace Ordering { namespace PlanBuilderCommon {

struct SourceInstance
{
    // Borrowed object view. The plan stores the corresponding opaque object
    // handle only as source context; append helpers clone the extrusion geometry.
    Object object;
    // Stable tie-breaks used by the plugin-side complete-object ordering.
    uint32_t object_idx = 0;
    uint32_t object_instance_idx = 0;
    // Platter-space translation applied to every cloned extrusion root.
    c_point shift = {};
    // Cached object height used by complete_objects_sort=lowz.
    coord_t max_z = 0;
};

// Select source layers by scaled Z in [min_z, max_z), and optionally by island
// indices of the layer passed to append_layer_instance_extrusions. An absent
// interval or index list imposes no restriction; an empty list selects nothing.
// The list is borrowed only for the duration of the append call.
struct LayerExtrusionFilter
{
    std::optional<std::pair<coord_t, coord_t>> print_z_interval;
    const std::vector<uint32_t> *island_indices = nullptr;
};

// Append object and local auxiliary layers selected by Z to a shared by-layer group.
// Optionally omit layer zero and its auxiliaries already emitted by the first pass.
void append_by_layer_object_layers(const Print &print, PrintingPlan &plan, PrintingGroup &group,
    std::map<coord_t, uint32_t> &indices, const plugin_run_context *run_ctx,
    const LayerExtrusionFilter &filter = {}, bool omit_first_layers = false);

/* Return all object instances in model/source order with their shift cached. */
std::vector<SourceInstance> source_instances(
    const Print &print,
    const plugin_run_context *run_ctx);

/*
Return the object-instance order for a complete-object plan.

This intentionally stays plugin-side. Exact host model order is not exposed as a
callback; the plugin uses source order, object height, increasing or decreasing
instance X/Y, or a greedy nearest-neighbor walk over instance shifts depending on complete_objects_sort.
*/
std::vector<SourceInstance> ordered_source_instances_for_object_plan(
    const Print &print,
    const plugin_run_context *run_ctx);

/*
Find or create the PrintingLayerGroup for print_z.

Appending to a group may invalidate older child handles, so callers keep only
indices in the map and request a fresh view for each use.
*/
PrintingLayerGroup layer_group_for_print_z(PrintingGroup &group,
                                           std::map<coord_t, uint32_t> &layer_index_by_print_z,
                                           coord_t print_z,
                                           const plugin_run_context *run_ctx);

/* Store each source Layer pointer once in a PrintingLayerGroup. */
void append_layer_once(
    PrintingLayerGroup &layer_group,
    const Layer &layer,
    const plugin_run_context *run_ctx);

/* Find or create the tool bucket for one extruder inside one layer group. */
PrintingToolGroup tool_group_for_extruder(
    PrintingLayerGroup &layer_group,
    extruder_index_t extruder_id,
    const plugin_run_context *run_ctx);

/* Store each source LayerRegionIsland pointer once in a tool group. */
void append_region_island_once(
    PrintingToolGroup &tool_group,
    const LayerRegionIsland &region_island,
    const plugin_run_context *run_ctx);

/*
Clone every printable role bucket of region_island for one object instance.

The source tree is object-local. The appended plan clone is translated by
instance_shift immediately, so later ordering and G-code code can read plan
coordinates directly.
*/
void append_region_island_instance_extrusions(PrintingPlan &plan,
                                              PrintingLayerGroup &layer_group,
                                              const LayerRegionIsland &region_island,
                                              c_point instance_shift,
                                              uint32_t printing_catalog_id,
                                              const plugin_run_context *run_ctx);

/*
Append printable LayerRegionIslands selected from one Layer. The default filter
copies everything; an interval selects [min_z, max_z) in scaled print Z, and an
island list selects indices in this source layer. Filtering creates no context
entries itself and leaves source geometry unchanged.

Normal object layers and auxiliary layers share the same LayerIsland tree once
their raw slices have been recomputed. Keeping this traversal shared prevents
print-level helper layers from needing their own ordering model.
*/
void append_layer_instance_extrusions(PrintingPlan &plan,
                                      PrintingLayerGroup &layer_group,
                                      const Layer &layer,
                                      c_point instance_shift,
                                      uint32_t printing_catalog_id,
                                      const plugin_run_context *run_ctx,
                                      const LayerExtrusionFilter &filter = {});

/*
Append the hidden print-level auxiliary object to an existing by-layer group.

The auxiliary object has exactly one logical instance at shift zero. Its layers
are already in platter coordinates, so they must not be duplicated for real
object instances.
The optional filter selects source layers by its half-open Z interval.
*/
void append_print_auxiliary_layers_to_group(const Print &print,
                                            PrintingPlan &plan,
                                            PrintingGroup &group,
                                            std::map<coord_t, uint32_t> &layer_index_by_print_z,
                                             const plugin_run_context *run_ctx,
                                             const LayerExtrusionFilter &filter = {});

/*
Create the leading group used by complete-object plans for global helper
layers. Return false if there is no auxiliary work or the group cannot be created.
The optional filter selects source layers by its half-open Z interval.
*/
bool append_print_auxiliary_group(
    const Print &print,
    PrintingPlan &plan,
    const plugin_run_context *run_ctx,
    const LayerExtrusionFilter &filter = {});

/*
Translate stored geometry in the owned canonical clone through the core bridge
shared with the native builder. Commands without a polyline retain the implicit
current position; the source tree and public ABI remain unchanged.
*/
void translate_extrusion_tree(
    MutableExtrusionEntity entity,
    c_point shift,
    const plugin_run_context *run_ctx);

/* Sort the layer groups after a construction walk that discovered them object by object. */
void sort_layer_groups_by_print_z(
    PrintingGroup &group,
    const plugin_run_context *run_ctx);

}}} // namespace slic3r_api::Ordering::PlanBuilderCommon

#endif // slic3r_Plugins_Ordering_PlanBuilderCommon_hpp_
