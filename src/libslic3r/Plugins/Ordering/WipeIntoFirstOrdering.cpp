///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
/* ordering.wipe_into_first.default moves the geometry the wipe tower can purge to the front of the
   visit that follows a tool change, so the tool-change service measures it there:
    - first the PrintingExtrusions whose region island belongs to an object with wipe_into_objects,
      keeping their relative order; support has no region island and never moves;
    - then the infill bucket of the first island after that block, when a region of it has
      wipe_into_infill and its perimeters are already printed (infill_first, the perimeter tool is
      the incoming tool, or a visit of that tool precedes the incoming visit in the layer); inside
      that bucket the direct children whose printable leaves are all sparse infill move to the
      front, keeping their relative order.
   Nothing is reassigned to another tool, and no root geometry or direction changes: only the
   chaining between the roots at the start of the visit gets worse, which keeps this simple. The
   entry points published by the tree ordering stay correct because they describe one root, not its
   position. A visit without a tool change is never touched, and so is a change with a soluble
   outgoing or incoming filament. The plugin only orders; the tower measurement and the wipe
   themselves stay in the wipe-tower service. */
#include "DefaultOrdering.hpp"

#include <algorithm>
#include <cstdint>
#include <stdexcept>

#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/WipeTowerFootprint.hpp"

namespace slic3r_api::Ordering::DefaultOrderingPlugin {
namespace {
const char *const k_no_dependencies[] = { nullptr };

/* The plugin decides from the wipe-into options and from the presence of a tower. The tower
   footprint geometry is deliberately not declared: it cannot change the decision, only the
   envelope of a tower that already exists. */
const raw_used_config_key k_used_keys[] = {
    {"wipe_into_infill", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"wipe_into_objects", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"filament_soluble", RAW_CO_VECTOR_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"perimeter_extruder", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"infill_first", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"wipe_tower", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"wipe_tower_per_object", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"print_plan_builder", RAW_CO_ENUM, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"complete_objects_step", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"nozzle_diameter", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"spiral_vase", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE}
};

// Return true when the island belongs to an object with wipe_into_objects. Reads borrowed views.
bool object_wipes_into_itself(const LayerRegionIsland &island)
{
    return island.region_count() != 0 &&
        island.region(0).layer().object().config().bool_or_default("wipe_into_objects", false);
}

// Return true when at least one region of the island has the given boolean option.
bool island_has_region_option(const LayerRegionIsland &island, const char *key)
{
    for (uint32_t region = 0; region < island.region_count(); ++region)
        if (island.region(region).print_region().config().bool_or_default(key, false))
            return true;
    return false;
}

// Return true when the island's perimeters are already printed before the incoming visit: a region
// asks for infill_first, or the island's perimeter tool is the incoming tool, or a visit of that
// tool precedes the incoming visit in the same layer. The perimeter tool is the first region's
// one-based perimeter_extruder; an invalid value keeps the island untouched.
bool perimeters_printed_first(const ConstPrintingLayerGroup &layer, const uint32_t visit_index,
                              const LayerRegionIsland &island, const extruder_index_t incoming)
{
    if (island_has_region_option(island, "infill_first"))
        return true;
    if (island.region_count() == 0)
        return false;
    const int32_t configured =
        island.region(0).print_region().config().int_or_default("perimeter_extruder", 1);
    const extruder_index_t perimeter_tool = configured > 0 ?
        extruder_index_t(configured - 1) : EXTRUDER_ID_UNKNOWN;
    if (perimeter_tool == EXTRUDER_ID_UNKNOWN)
        return false;
    if (perimeter_tool == incoming)
        return true;
    for (uint32_t t = 0; t < visit_index; ++t)
        if (layer.tool_group(t).extruder_id() == perimeter_tool)
            return true;
    return false;
}

// Return true when the tree has at least one printable leaf (points and a positive inherited
// mm3_per_mm) and every printable leaf is sparse infill. Roles are read raw because role() asserts
// on process-only roles.
bool all_leaves_sparse_infill(const Slic3r::ExtrusionEntity &entity, const Slic3r::ExtrusionAttributes *inherited,
                              bool &has_leaf)
{
    if (const Slic3r::ExtrusionAttributes *attributes = entity.get_property<Slic3r::ExtrusionAttributes>())
        inherited = attributes;
    if (entity.child_count() == 0) {
        if (inherited == nullptr || !(inherited->mm3_per_mm > 0.) || !entity.has_polyline())
            return true;
        has_leaf = true;
        return inherited->role == uint16_t(RAW_EXTRUSION_ROLE_INTERNAL_INFILL);
    }
    for (uint32_t idx = 0; idx < entity.child_count(); ++idx)
        if (!all_leaves_sparse_infill(entity.child(idx), inherited, has_leaf))
            return false;
    return true;
}

// Move the direct children of one root whose printable leaves are all sparse infill to its front,
// keeping their relative order. Reads and moves borrowed children only.
void move_sparse_children_first(MutableExtrusionEntity root, const plugin_run_context *run)
{
    uint32_t fixed = 0;
    for (uint32_t idx = 0; idx < root.child_count(); ++idx) {
        throw_if_cancelled(run);
        const MutableExtrusionEntity child = root.child_mutable(idx);
        bool has_leaf = false;
        if (!all_leaves_sparse_infill(
                *reinterpret_cast<const Slic3r::ExtrusionEntity *>(child.handle()), nullptr, has_leaf) ||
            !has_leaf)
            continue;
        if (idx != fixed && root.move_child_from(fixed, root, idx) == EXTRUSION_INDEX_INVALID)
            throw std::runtime_error("Cannot move a sparse infill child to the front of the bucket.");
        ++fixed;
    }
}

// Order one visit: the wipe-into objects block first, then the sparse infill of the first island
// that wipe_into_infill can use; a visit without such an island stays untouched.
void order_visit(PrintingToolGroup visit, const ConstPrintingLayerGroup &layer, const uint32_t visit_index,
                 const extruder_index_t incoming, const plugin_run_context *run)
{
    throw_if_cancelled(run);
    uint32_t fixed = 0;
    for (uint32_t e = 0; e < visit.extrusion_count(); ++e) {
        const LayerRegionIsland island = visit.extrusion(e).region_island();
        if (!island.valid() || !object_wipes_into_itself(island))
            continue;
        if (e != fixed && !visit.move_extrusion(e, fixed))
            throw std::runtime_error("Cannot move a wipe-into extrusion to the front of the visit.");
        ++fixed;
    }
    if (fixed == visit.extrusion_count())
        return;
    // Only the first island after the objects block is considered, and only its infill bucket.
    const LayerRegionIsland island = visit.extrusion(fixed).region_island();
    if (!island.valid() || island.region_count() == 0 || object_wipes_into_itself(island) ||
        !island_has_region_option(island, "wipe_into_infill") ||
        !perimeters_printed_first(layer, visit_index, island, incoming))
        return;
    for (uint32_t e = fixed; e < visit.extrusion_count(); ++e) {
        PrintingExtrusion extrusion = visit.extrusion(e);
        if (extrusion.role() != uint16_t(RAW_EXTRUSION_ROLE_INTERNAL_INFILL) ||
            extrusion.region_island().handle() != island.handle())
            continue;
        throw_if_cancelled(run);
        if (e != fixed && !visit.move_extrusion(e, fixed))
            throw std::runtime_error("Cannot move the infill bucket to the front of the visit.");
        // Moving an extrusion moves its PrintingExtrusion object inside the visit vector, so the
        // bucket view taken above is stale: fetch the bucket again at its new index.
        move_sparse_children_first(visit.extrusion(fixed).mutable_root(), run);
        return;
    }
}

class WipeIntoFirstOrdering final : public PluginBase
{
public:
    static WipeIntoFirstOrdering &instance(orchestrator_handle *orchestrator)
    {
        static WipeIntoFirstOrdering instance(orchestrator);
        return instance;
    }

    explicit WipeIntoFirstOrdering(orchestrator_handle *orchestrator) : PluginBase(orchestrator) {}

private:
    const char *id_impl() const noexcept override { return "ordering.wipe_into_first.default"; }
    const char *name_impl() const noexcept override { return "Wipe into first ordering"; }
    const char *description_impl() const noexcept override
    {
        return "Moves the geometry the wipe tower can purge to the front of the visit after a tool change.";
    }
    const char *exclusive_group_impl() const noexcept override { return ""; }
    slicing_step_t step_impl() const noexcept override { return STEP_ORDERING; }
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }
    int32_t priority_impl() const noexcept override { return 1200; }

    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override
    {
        const int32_t count = int32_t(sizeof(k_used_keys) / sizeof(k_used_keys[0]));
        if (keys != nullptr)
            std::copy(k_used_keys, k_used_keys + count, keys);
        return count;
    }

    void run_impl(const plugin_run_context *run_ctx) const override
    {
        const run_ctx_extrusion_ordering *ctx = plugin_ctx_as_extrusion_ordering(run_ctx);
        if (ctx == nullptr || ctx->print == nullptr || ctx->plan == nullptr)
            return;
        const Slic3r::Print &print = *reinterpret_cast<const Slic3r::Print *>(ctx->print);
        // Without a wipe tower there is nothing to purge into the objects.
        if (Slic3r::wipe_tower_footprints(print).empty())
            return;
        const Slic3r::PrintConfig &print_config = print.config();
        const size_t tool_count = print_config.nozzle_diameter.size();
        PrintingPlan plan(ctx->plan);
        extruder_index_t previous = EXTRUDER_ID_UNKNOWN;
        for (uint32_t group_idx = 0; group_idx < plan.group_count(); ++group_idx)
            for (uint32_t layer_idx = 0; layer_idx < plan.group(group_idx).layer_group_count(); ++layer_idx) {
                throw_if_cancelled(run_ctx);
                PrintingLayerGroup layer = plan.group(group_idx).layer_group(layer_idx);
                for (uint32_t tool_idx = 0; tool_idx < layer.tool_group_count(); ++tool_idx) {
                    PrintingToolGroup visit = layer.tool_group(tool_idx);
                    const extruder_index_t tool = visit.extruder_id();
                    // An unresolved or unconfigured visit cannot be purged; like the tower
                    // service, it is skipped without resetting the previous concrete tool.
                    if (tool < 0 || size_t(tool) >= tool_count)
                        continue;
                    // A real change A->B with neither filament soluble is the only place the
                    // ordering may help the purge.
                    if (previous != EXTRUDER_ID_UNKNOWN && previous != tool &&
                        !print_config.filament_soluble.get_at(size_t(previous)) &&
                        !print_config.filament_soluble.get_at(size_t(tool)))
                        order_visit(visit, layer, tool_idx, tool, run_ctx);
                    previous = tool;
                }
            }
    }
};

} // namespace

void register_wipe_into_first_ordering_plugin(orchestrator_handle *orchestrator)
{
    orchestrator_register_plugin(
        orchestrator, WipeIntoFirstOrdering::instance(orchestrator).c_instance());
}

} // namespace slic3r_api::Ordering::DefaultOrderingPlugin
