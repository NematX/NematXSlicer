///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "libslic3r/Api/plugin/c/slic3r_printing_plan.h"
#include "libslic3r/Api/plugin/c/slic3r_extrusion_entity.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <type_traits>
#include <utility>

#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCanonical.hpp"
#include "libslic3r/ExtrusionRole.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"

namespace Slic3r {
namespace {

/*
These casts are the C ABI boundary for PrintingPlan. The C side only sees
opaque handles; the host side always immediately converts them back to the C++
objects that own the vectors and extrusion clones.
Plan copies publish canonical base nodes; plan moves acquire new canonical roots
after reserving publication capacity. Conforming child handles survive a move;
legacy trees are copied before their source content is consumed. Root setters
prepare their result before replacing the owner, so self/descendant aliases stay
valid throughout preparation. Allocation errors propagate to the guarded plugin
caller without publishing a partial tree. All handles remain borrowed from the
live plan or source storage; callers exclude concurrent structural mutation.

    printing_tool_group_append_extrusion_clone() -> clone_or_empty()
        `-- clone_canonical(): independently copy stored content and properties
    printing_tool_group_append_extrusion_move() -> move_or_empty()
        `-- CanonicalExtrusion::acquire(): consume only after preparation
*/
Printing::PrintingPlan *to_plan(printing_plan_handle *me)
{
    return reinterpret_cast<Printing::PrintingPlan *>(me);
}

const Printing::PrintingPlan *to_plan(const printing_plan_handle *me)
{
    return reinterpret_cast<const Printing::PrintingPlan *>(me);
}

Printing::PrintingScopeEvents *to_scope_events(printing_scope_events_handle *me)
{
    return reinterpret_cast<Printing::PrintingScopeEvents *>(me);
}

const Printing::PrintingScopeEvents *to_scope_events(const printing_scope_events_handle *me)
{
    return reinterpret_cast<const Printing::PrintingScopeEvents *>(me);
}

Printing::PrintingGroup *to_group(printing_group_handle *me)
{
    return reinterpret_cast<Printing::PrintingGroup *>(me);
}

const Printing::PrintingGroup *to_group(const printing_group_handle *me)
{
    return reinterpret_cast<const Printing::PrintingGroup *>(me);
}

Printing::PrintingLayerGroup *to_layer_group(printing_layer_group_handle *me)
{
    return reinterpret_cast<Printing::PrintingLayerGroup *>(me);
}

const Printing::PrintingLayerGroup *to_layer_group(const printing_layer_group_handle *me)
{
    return reinterpret_cast<const Printing::PrintingLayerGroup *>(me);
}

Printing::PrintingToolGroup *to_tool_group(printing_tool_group_handle *me)
{
    return reinterpret_cast<Printing::PrintingToolGroup *>(me);
}

const Printing::PrintingToolGroup *to_tool_group(const printing_tool_group_handle *me)
{
    return reinterpret_cast<const Printing::PrintingToolGroup *>(me);
}

Printing::PrintingExtrusion *to_printing_extrusion(printing_extrusion_handle *me)
{
    return reinterpret_cast<Printing::PrintingExtrusion *>(me);
}

const Printing::PrintingExtrusion *to_printing_extrusion(const printing_extrusion_handle *me)
{
    return reinterpret_cast<const Printing::PrintingExtrusion *>(me);
}

const Print *to_print(const print_handle *me)
{
    return reinterpret_cast<const Print *>(me);
}

const PrintObject *to_object(const object_handle *me)
{
    return reinterpret_cast<const PrintObject *>(me);
}

const Layer *to_layer(const layer_handle *me)
{
    return reinterpret_cast<const Layer *>(me);
}

const LayerRegionIsland *to_region_island(const layer_region_island_handle *me)
{
    return reinterpret_cast<const LayerRegionIsland *>(me);
}

/* Recover the common source object through the island's LayerRegions.

LayerRegionIsland intentionally has no parent pointer. Every member region of
one island nevertheless belongs to the same Layer and PrintObject. Reject an
empty or inconsistent source instead of assigning an ID from guessed context.
*/
const PrintObject *object_for_region_island(const LayerRegionIsland *island)
{
    if (island == nullptr || island->regions().empty())
        return nullptr;
    const PrintObject *object = nullptr;
    for (const LayerRegion *region : island->regions()) {
        const PrintObject *candidate = region == nullptr || region->layer() == nullptr ?
            nullptr : region->layer()->object();
        if (candidate == nullptr || (object != nullptr && object != candidate))
            return nullptr;
        object = candidate;
    }
    return object;
}

/*
Print-level auxiliary layers use a real PrintObject as their data-tree owner,
but they do not represent a physical PrintInstance. Keep that distinction in
one place so every plan mutation maps them to the reserved outside ID
UINT32_MAX.
*/
bool is_print_auxiliary_object(const PrintObject *object)
{
    return object != nullptr && object->print() != nullptr &&
           object->print()->auxiliary_object() == object;
}

const Polygon *to_polygon(const polygon_handle *me)
{
    return reinterpret_cast<const Polygon *>(me);
}

bool plan_owns_group(const Printing::PrintingPlan *plan,
                     const Printing::PrintingGroup *group)
{
    return plan != nullptr && group != nullptr &&
           group->plan_identity == plan->identity_token();
}

bool plan_owns_tool_group(const Printing::PrintingPlan *plan,
                          const Printing::PrintingToolGroup *tool_group)
{
    return plan != nullptr && tool_group != nullptr &&
           tool_group->plan_identity == plan->identity_token();
}

ExtrusionEntity *to_extrusion(extrusion_entity_handle *me)
{
    return reinterpret_cast<ExtrusionEntity *>(me);
}

const ExtrusionEntity *to_extrusion(const extrusion_entity_handle *me)
{
    return reinterpret_cast<const ExtrusionEntity *>(me);
}

/* Rebuild the compact source-context list from the extrusions that remain in
   a tool group. Ordering mutations use this instead of trying to update the
   list incrementally, which also removes duplicates left by merged sources. */
std::vector<const LayerRegionIsland *> region_islands_for_extrusions(
    const std::vector<Printing::PrintingExtrusion> &extrusions)
{
    std::vector<const LayerRegionIsland *> result;
    result.reserve(extrusions.size());
    for (const Printing::PrintingExtrusion &extrusion : extrusions)
        if (extrusion.region_island != nullptr &&
            std::find(result.begin(), result.end(), extrusion.region_island) == result.end())
            result.push_back(extrusion.region_island);
    return result;
}

/* Build the source context remaining after one extrusion is removed. Keeping
   this helper pointer-only avoids constructing temporary ownership objects
   while the transfer is still in its strongly guaranteed preparation phase. */
std::vector<const LayerRegionIsland *> region_islands_without_extrusion(
    const Printing::PrintingToolGroup &source,
    uint32_t removed_idx)
{
    std::vector<const LayerRegionIsland *> result;
    result.reserve(source.extrusions.size() - 1);
    for (uint32_t idx = 0; idx < source.extrusions.size(); ++idx) {
        if (idx == removed_idx)
            continue;
        const LayerRegionIsland *region_island = source.extrusions[idx].region_island;
        if (region_island != nullptr &&
            std::find(result.begin(), result.end(), region_island) == result.end())
            result.push_back(region_island);
    }
    return result;
}

/* Prepare the context list that a destination will expose after receiving one
   more extrusion. This allocation happens before ownership is moved so an
   allocation failure leaves both live groups unchanged. */
std::vector<const LayerRegionIsland *> region_islands_after_append(
    const Printing::PrintingToolGroup &destination,
    const Printing::PrintingExtrusion &extrusion)
{
    std::vector<const LayerRegionIsland *> result =
        region_islands_for_extrusions(destination.extrusions);
    if (extrusion.region_island != nullptr &&
        std::find(result.begin(), result.end(), extrusion.region_island) == result.end())
        result.push_back(extrusion.region_island);
    return result;
}

ExtrusionRole to_extrusion_role(raw_extrusion_role role)
{
    return ExtrusionRole(static_cast<ExtrusionRoleModifier>(role));
}

raw_extrusion_role to_raw_role(ExtrusionRole role)
{
    return static_cast<raw_extrusion_role>(role());
}

template<class Vector>
bool move_vector_item(Vector &items, uint32_t from_idx, uint32_t to_idx)
{
    // Reordering APIs move owned vector elements rather than copying them.
    // This preserves cloned extrusion trees and any properties stored on those
    // trees while giving plugins simple index-based ordering primitives.
    if (from_idx >= items.size() || to_idx >= items.size())
        return false;
    if (from_idx == to_idx)
        return true;

    typename Vector::value_type moved = std::move(items[from_idx]);
    items.erase(items.begin() + from_idx);
    items.insert(items.begin() + to_idx, std::move(moved));
    return true;
}

std::unique_ptr<ExtrusionEntity> clone_or_empty(const extrusion_entity_handle *root)
{
    if (root == nullptr)
        return std::make_unique<ExtrusionEntity>(true);
    return to_extrusion(root)->clone_canonical();
}

std::unique_ptr<ExtrusionEntity> move_or_empty(extrusion_entity_handle *root)
{
    return root != nullptr ? CanonicalExtrusion::acquire(*to_extrusion(root)) :
        std::make_unique<ExtrusionEntity>(true);
}

} // namespace
} // namespace Slic3r

extern "C" {

int32_t printing_scope_events_has_before(const printing_scope_events_handle *me)
{
    return me != nullptr && Slic3r::to_scope_events(me)->has_before();
}

int32_t printing_scope_events_has_after(const printing_scope_events_handle *me)
{
    return me != nullptr && Slic3r::to_scope_events(me)->has_after();
}

const extrusion_entity_handle *printing_scope_events_get_before(const printing_scope_events_handle *me)
{
    if (me == nullptr)
        return nullptr;
    return reinterpret_cast<const extrusion_entity_handle *>(&Slic3r::to_scope_events(me)->before());
}

const extrusion_entity_handle *printing_scope_events_get_after(const printing_scope_events_handle *me)
{
    if (me == nullptr)
        return nullptr;
    return reinterpret_cast<const extrusion_entity_handle *>(&Slic3r::to_scope_events(me)->after());
}

extrusion_entity_handle *printing_scope_events_append_before_clone(
    printing_scope_events_handle *me,
    const extrusion_entity_handle *event)
{
    if (me == nullptr || event == nullptr)
        return nullptr;
    Slic3r::ExtrusionEntity &appended = Slic3r::to_scope_events(me)->append_before(*Slic3r::to_extrusion(event));
    return reinterpret_cast<extrusion_entity_handle *>(&appended);
}

extrusion_entity_handle *printing_scope_events_append_before_move(
    printing_scope_events_handle *me,
    extrusion_entity_handle *event)
{
    if (me == nullptr || event == nullptr)
        return nullptr;
    Slic3r::ExtrusionEntity &appended =
        Slic3r::to_scope_events(me)->append_before(std::move(*Slic3r::to_extrusion(event)));
    return reinterpret_cast<extrusion_entity_handle *>(&appended);
}

extrusion_entity_handle *printing_scope_events_append_after_clone(
    printing_scope_events_handle *me,
    const extrusion_entity_handle *event)
{
    if (me == nullptr || event == nullptr)
        return nullptr;
    Slic3r::ExtrusionEntity &appended = Slic3r::to_scope_events(me)->append_after(*Slic3r::to_extrusion(event));
    return reinterpret_cast<extrusion_entity_handle *>(&appended);
}

extrusion_entity_handle *printing_scope_events_append_after_move(
    printing_scope_events_handle *me,
    extrusion_entity_handle *event)
{
    if (me == nullptr || event == nullptr)
        return nullptr;
    Slic3r::ExtrusionEntity &appended =
        Slic3r::to_scope_events(me)->append_after(std::move(*Slic3r::to_extrusion(event)));
    return reinterpret_cast<extrusion_entity_handle *>(&appended);
}

void printing_plan_clear(printing_plan_handle *me)
{
    if (me != nullptr) {
        Slic3r::to_plan(me)->events.clear();
        Slic3r::to_plan(me)->properties.clear_properties();
        Slic3r::to_plan(me)->object_instances.clear();
        Slic3r::to_plan(me)->groups.clear();
    }
}

printing_scope_events_handle *printing_plan_get_events_mutable(printing_plan_handle *me)
{
    return me == nullptr ? nullptr : reinterpret_cast<printing_scope_events_handle *>(&Slic3r::to_plan(me)->events);
}

const printing_scope_events_handle *printing_plan_get_events(const printing_plan_handle *me)
{
    return me == nullptr ?
               nullptr : reinterpret_cast<const printing_scope_events_handle *>(&Slic3r::to_plan(me)->events);
}

const plugin_property_container_handle *printing_plan_get_properties(
    const printing_plan_handle *me)
{
    return me == nullptr ? nullptr :
        reinterpret_cast<const plugin_property_container_handle *>(
            &Slic3r::to_plan(me)->properties);
}

plugin_property_container_handle *printing_plan_get_properties_mutable(printing_plan_handle *me)
{
    return me == nullptr ? nullptr :
        reinterpret_cast<plugin_property_container_handle *>(
            &Slic3r::to_plan(me)->properties);
}

uint32_t printing_plan_register_object_instance(
    printing_plan_handle *me,
    const object_handle *object,
    uint64_t object_instance_idx,
    const char *synthetic_name,
    const polygon_handle *synthetic_outline)
{
    if (me == nullptr)
        return Slic3r::Printing::PRINTING_OBJECT_INSTANCE_ID_NONE;
    try {
        if (object != nullptr && synthetic_name == nullptr && synthetic_outline == nullptr &&
            object_instance_idx <= uint64_t((std::numeric_limits<size_t>::max)()))
            return Slic3r::to_plan(me)->find_or_add_object_instance(
                *Slic3r::to_object(object), static_cast<size_t>(object_instance_idx));

        if (object == nullptr && object_instance_idx == (std::numeric_limits<uint64_t>::max)() &&
            synthetic_name != nullptr && synthetic_name[0] != '\0' &&
            synthetic_outline != nullptr && !Slic3r::to_polygon(synthetic_outline)->empty())
            return Slic3r::to_plan(me)->add_synthetic_object_instance(
                synthetic_name, *Slic3r::to_polygon(synthetic_outline));
    } catch (...) {
    }
    return Slic3r::Printing::PRINTING_OBJECT_INSTANCE_ID_NONE;
}

int32_t printing_plan_set_object_instance_metadata(
    printing_plan_handle *me, uint32_t id, const char *name,
    const polygon_handle *outline)
{
    if (me == nullptr || name == nullptr || name[0] == '\0' || outline == nullptr)
        return 0;
    try {
        const Slic3r::Polygon *polygon = Slic3r::to_polygon(outline);
        return polygon != nullptr && !polygon->empty() &&
            Slic3r::to_plan(me)->set_object_instance_metadata(id, name, *polygon);
    } catch (...) {
        return 0;
    }
}

uint32_t printing_plan_count_object_instance(const printing_plan_handle *me)
{
    return me == nullptr ? 0u : uint32_t(Slic3r::to_plan(me)->object_instances.size());
}

c_printing_object_instance printing_plan_get_object_instance_by_printing_catalog_id(
    const printing_plan_handle *me, uint32_t id)
{
    c_printing_object_instance out = {};
    out.id = Slic3r::Printing::PRINTING_OBJECT_INSTANCE_ID_NONE;
    if (me == nullptr)
        return out;
    const Slic3r::Printing::PrintingObjectInstance *instance =
        Slic3r::to_plan(me)->object_instance(id);
    if (instance == nullptr)
        return out;
    out.object = reinterpret_cast<const object_handle *>(instance->object);
    out.object_instance_idx = static_cast<uint64_t>(instance->object_instance_idx);
    out.id = id;
    out.name = instance->name.empty() ? nullptr : instance->name.c_str();
    out.projected_outline_center = {
        instance->projected_outline_center.x(), instance->projected_outline_center.y()};
    out.projected_outline = instance->projected_outline.empty() ? nullptr :
        reinterpret_cast<const polygon_handle *>(&instance->projected_outline);
    return out;
}

uint32_t printing_plan_count_group(const printing_plan_handle *me)
{
    return me == nullptr ? 0u : uint32_t(Slic3r::to_plan(me)->groups.size());
}

printing_group_handle *printing_plan_get_group_mutable(printing_plan_handle *me, uint32_t idx)
{
    if (me == nullptr || idx >= Slic3r::to_plan(me)->groups.size())
        return nullptr;
    return reinterpret_cast<printing_group_handle *>(&Slic3r::to_plan(me)->groups[idx]);
}

const printing_group_handle *printing_plan_get_group(const printing_plan_handle *me, uint32_t idx)
{
    if (me == nullptr || idx >= Slic3r::to_plan(me)->groups.size())
        return nullptr;
    return reinterpret_cast<const printing_group_handle *>(&Slic3r::to_plan(me)->groups[idx]);
}

printing_group_handle *printing_plan_append_group(printing_plan_handle *me)
{
    if (me == nullptr)
        return nullptr;
    Slic3r::Printing::PrintingGroup &group = Slic3r::to_plan(me)->groups.emplace_back();
    group.plan_identity = Slic3r::to_plan(me)->identity_token();
    return reinterpret_cast<printing_group_handle *>(&group);
}

int32_t printing_plan_move_group(printing_plan_handle *me, uint32_t from_idx, uint32_t to_idx)
{
    return me != nullptr && Slic3r::move_vector_item(Slic3r::to_plan(me)->groups, from_idx, to_idx);
}

void printing_group_clear(printing_group_handle *me)
{
    if (me != nullptr) {
        Slic3r::Printing::PrintingGroup *group = Slic3r::to_group(me);
        const Slic3r::Printing::PrintingPlanIdentity *identity = group->plan_identity;
        *group = Slic3r::Printing::PrintingGroup();
        group->plan_identity = identity;
    }
}

printing_scope_events_handle *printing_group_get_events_mutable(printing_group_handle *me)
{
    return me == nullptr ? nullptr : reinterpret_cast<printing_scope_events_handle *>(&Slic3r::to_group(me)->events);
}

const printing_scope_events_handle *printing_group_get_events(const printing_group_handle *me)
{
    return me == nullptr ?
               nullptr : reinterpret_cast<const printing_scope_events_handle *>(&Slic3r::to_group(me)->events);
}

const plugin_property_container_handle *printing_group_get_properties(
    const printing_group_handle *me)
{
    return me == nullptr ? nullptr :
        reinterpret_cast<const plugin_property_container_handle *>(
            &Slic3r::to_group(me)->properties);
}

plugin_property_container_handle *printing_group_get_properties_mutable(printing_group_handle *me)
{
    return me == nullptr ? nullptr :
        reinterpret_cast<plugin_property_container_handle *>(
            &Slic3r::to_group(me)->properties);
}

uint32_t printing_group_count_object_instance(const printing_group_handle *me)
{
    return me == nullptr ? 0u : uint32_t(Slic3r::to_group(me)->printing_catalog_ids.size());
}

uint32_t printing_group_get_printing_catalog_id(
    const printing_group_handle *me, uint32_t idx)
{
    return me == nullptr || idx >= Slic3r::to_group(me)->printing_catalog_ids.size() ?
        Slic3r::Printing::PRINTING_OBJECT_INSTANCE_ID_NONE :
        Slic3r::to_group(me)->printing_catalog_ids[idx];
}

int32_t printing_group_append_printing_catalog_id(
    const printing_plan_handle *plan, printing_group_handle *me, uint32_t id)
{
    if (!Slic3r::plan_owns_group(Slic3r::to_plan(plan), Slic3r::to_group(me)) ||
        Slic3r::to_plan(plan)->object_instance(id) == nullptr)
        return 0;
    Slic3r::to_group(me)->printing_catalog_ids.push_back(id);
    return 1;
}

uint32_t printing_group_count_layer_group(const printing_group_handle *me)
{
    return me == nullptr ? 0u : uint32_t(Slic3r::to_group(me)->layers.size());
}

printing_layer_group_handle *printing_group_get_layer_group_mutable(printing_group_handle *me, uint32_t idx)
{
    if (me == nullptr || idx >= Slic3r::to_group(me)->layers.size())
        return nullptr;
    return reinterpret_cast<printing_layer_group_handle *>(&Slic3r::to_group(me)->layers[idx]);
}

const printing_layer_group_handle *printing_group_get_layer_group(const printing_group_handle *me, uint32_t idx)
{
    if (me == nullptr || idx >= Slic3r::to_group(me)->layers.size())
        return nullptr;
    return reinterpret_cast<const printing_layer_group_handle *>(&Slic3r::to_group(me)->layers[idx]);
}

printing_layer_group_handle *printing_group_append_layer_group(printing_group_handle *me, coord_t print_z)
{
    if (me == nullptr)
        return nullptr;
    Slic3r::Printing::PrintingLayerGroup &layer_group = Slic3r::to_group(me)->layers.emplace_back();
    layer_group.plan_identity = Slic3r::to_group(me)->plan_identity;
    layer_group.print_z = print_z;
    return reinterpret_cast<printing_layer_group_handle *>(&layer_group);
}

int32_t printing_group_move_layer_group(printing_group_handle *me, uint32_t from_idx, uint32_t to_idx)
{
    return me != nullptr && Slic3r::move_vector_item(Slic3r::to_group(me)->layers, from_idx, to_idx);
}

printing_scope_events_handle *printing_layer_group_get_events_mutable(printing_layer_group_handle *me)
{
    return me == nullptr ?
               nullptr : reinterpret_cast<printing_scope_events_handle *>(&Slic3r::to_layer_group(me)->events);
}

const printing_scope_events_handle *printing_layer_group_get_events(const printing_layer_group_handle *me)
{
    return me == nullptr ?
               nullptr : reinterpret_cast<const printing_scope_events_handle *>(&Slic3r::to_layer_group(me)->events);
}

const plugin_property_container_handle *printing_layer_group_get_properties(
    const printing_layer_group_handle *me)
{
    return me == nullptr ? nullptr :
        reinterpret_cast<const plugin_property_container_handle *>(
            &Slic3r::to_layer_group(me)->properties);
}

plugin_property_container_handle *printing_layer_group_get_properties_mutable(
    printing_layer_group_handle *me)
{
    return me == nullptr ? nullptr :
        reinterpret_cast<plugin_property_container_handle *>(
            &Slic3r::to_layer_group(me)->properties);
}

coord_t printing_layer_group_get_print_z(const printing_layer_group_handle *me)
{
    return me == nullptr ? 0 : Slic3r::to_layer_group(me)->print_z;
}

void printing_layer_group_set_print_z(printing_layer_group_handle *me, coord_t print_z)
{
    if (me != nullptr)
        Slic3r::to_layer_group(me)->print_z = print_z;
}

uint32_t printing_layer_group_count_layer(const printing_layer_group_handle *me)
{
    return me == nullptr ? 0u : uint32_t(Slic3r::to_layer_group(me)->layers.size());
}

const layer_handle *printing_layer_group_get_layer(const printing_layer_group_handle *me, uint32_t idx)
{
    if (me == nullptr || idx >= Slic3r::to_layer_group(me)->layers.size())
        return nullptr;
    return reinterpret_cast<const layer_handle *>(Slic3r::to_layer_group(me)->layers[idx]);
}

void printing_layer_group_append_layer(printing_layer_group_handle *me, const layer_handle *layer)
{
    if (me != nullptr)
        Slic3r::to_layer_group(me)->layers.push_back(Slic3r::to_layer(layer));
}

uint32_t printing_layer_group_count_tool_group(const printing_layer_group_handle *me)
{
    return me == nullptr ? 0u : uint32_t(Slic3r::to_layer_group(me)->tool_groups.size());
}

printing_tool_group_handle *printing_layer_group_get_tool_group_mutable(printing_layer_group_handle *me,
                                                                        uint32_t idx)
{
    if (me == nullptr || idx >= Slic3r::to_layer_group(me)->tool_groups.size())
        return nullptr;
    return reinterpret_cast<printing_tool_group_handle *>(&Slic3r::to_layer_group(me)->tool_groups[idx]);
}

const printing_tool_group_handle *printing_layer_group_get_tool_group(const printing_layer_group_handle *me,
                                                                      uint32_t idx)
{
    if (me == nullptr || idx >= Slic3r::to_layer_group(me)->tool_groups.size())
        return nullptr;
    return reinterpret_cast<const printing_tool_group_handle *>(&Slic3r::to_layer_group(me)->tool_groups[idx]);
}

printing_tool_group_handle *printing_layer_group_append_tool_group(printing_layer_group_handle *me,
                                                                   extruder_index_t extruder_id)
{
    if (me == nullptr || extruder_id < EXTRUDER_ID_UNKNOWN)
        return nullptr;
    Slic3r::Printing::PrintingToolGroup &tool_group = Slic3r::to_layer_group(me)->tool_groups.emplace_back();
    tool_group.plan_identity = Slic3r::to_layer_group(me)->plan_identity;
    tool_group.extruder_id = extruder_id;
    return reinterpret_cast<printing_tool_group_handle *>(&tool_group);
}

int32_t printing_layer_group_move_tool_group(printing_layer_group_handle *me, uint32_t from_idx, uint32_t to_idx)
{
    return me != nullptr && Slic3r::move_vector_item(Slic3r::to_layer_group(me)->tool_groups, from_idx, to_idx);
}

int32_t printing_layer_group_remove_empty_tool_group(printing_layer_group_handle *me, uint32_t idx)
{
    if (me == nullptr || idx >= Slic3r::to_layer_group(me)->tool_groups.size())
        return 0;

    Slic3r::Printing::PrintingToolGroup &tool_group = Slic3r::to_layer_group(me)->tool_groups[idx];
    if (!tool_group.extrusions.empty() || tool_group.events.has_before() || tool_group.events.has_after() ||
        tool_group.properties.has_properties())
        return 0;

    Slic3r::to_layer_group(me)->tool_groups.erase(
        Slic3r::to_layer_group(me)->tool_groups.begin() + idx);
    return 1;
}

printing_scope_events_handle *printing_tool_group_get_events_mutable(printing_tool_group_handle *me)
{
    return me == nullptr ?
               nullptr : reinterpret_cast<printing_scope_events_handle *>(&Slic3r::to_tool_group(me)->events);
}

const printing_scope_events_handle *printing_tool_group_get_events(const printing_tool_group_handle *me)
{
    return me == nullptr ?
               nullptr : reinterpret_cast<const printing_scope_events_handle *>(&Slic3r::to_tool_group(me)->events);
}

const plugin_property_container_handle *printing_tool_group_get_properties(
    const printing_tool_group_handle *me)
{
    return me == nullptr ? nullptr :
        reinterpret_cast<const plugin_property_container_handle *>(
            &Slic3r::to_tool_group(me)->properties);
}

plugin_property_container_handle *printing_tool_group_get_properties_mutable(
    printing_tool_group_handle *me)
{
    return me == nullptr ? nullptr :
        reinterpret_cast<plugin_property_container_handle *>(
            &Slic3r::to_tool_group(me)->properties);
}

extruder_index_t printing_tool_group_get_extruder_id(const printing_tool_group_handle *me)
{
    return me == nullptr ? EXTRUDER_ID_UNKNOWN : Slic3r::to_tool_group(me)->extruder_id;
}

void printing_tool_group_set_extruder_id(printing_tool_group_handle *me, extruder_index_t extruder_id)
{
    if (me != nullptr && extruder_id >= EXTRUDER_ID_UNKNOWN)
        Slic3r::to_tool_group(me)->extruder_id = extruder_id;
}

uint32_t printing_tool_group_count_region_island(const printing_tool_group_handle *me)
{
    return me == nullptr ? 0u : uint32_t(Slic3r::to_tool_group(me)->region_islands.size());
}

const layer_region_island_handle *printing_tool_group_get_region_island(const printing_tool_group_handle *me,
                                                                        uint32_t idx)
{
    if (me == nullptr || idx >= Slic3r::to_tool_group(me)->region_islands.size())
        return nullptr;
    return reinterpret_cast<const layer_region_island_handle *>(Slic3r::to_tool_group(me)->region_islands[idx]);
}

void printing_tool_group_append_region_island(printing_tool_group_handle *me,
                                              const layer_region_island_handle *region_island)
{
    if (me != nullptr)
        Slic3r::to_tool_group(me)->region_islands.push_back(Slic3r::to_region_island(region_island));
}

uint32_t printing_tool_group_count_extrusion(const printing_tool_group_handle *me)
{
    return me == nullptr ? 0u : uint32_t(Slic3r::to_tool_group(me)->extrusions.size());
}

printing_extrusion_handle *printing_tool_group_get_extrusion_mutable(printing_tool_group_handle *me, uint32_t idx)
{
    if (me == nullptr || idx >= Slic3r::to_tool_group(me)->extrusions.size())
        return nullptr;
    return reinterpret_cast<printing_extrusion_handle *>(&Slic3r::to_tool_group(me)->extrusions[idx]);
}

const printing_extrusion_handle *printing_tool_group_get_extrusion(const printing_tool_group_handle *me,
                                                                   uint32_t idx)
{
    if (me == nullptr || idx >= Slic3r::to_tool_group(me)->extrusions.size())
        return nullptr;
    return reinterpret_cast<const printing_extrusion_handle *>(&Slic3r::to_tool_group(me)->extrusions[idx]);
}

printing_extrusion_handle *printing_tool_group_append_extrusion_clone(
    const printing_plan_handle *plan,
    printing_tool_group_handle *me,
    const layer_region_island_handle *source_region_island,
    raw_extrusion_role source_role,
    const extrusion_entity_handle *root,
    uint32_t printing_catalog_id)
{
    const Slic3r::Printing::PrintingPlan *printing_plan = Slic3r::to_plan(plan);
    const Slic3r::Printing::PrintingToolGroup *tool_group = Slic3r::to_tool_group(me);
    const Slic3r::LayerRegionIsland *region_island = Slic3r::to_region_island(source_region_island);
    const Slic3r::ExtrusionRole role = Slic3r::to_extrusion_role(source_role);
    if (!Slic3r::plan_owns_tool_group(printing_plan, tool_group) ||
        (printing_catalog_id != Slic3r::Printing::PRINTING_OBJECT_INSTANCE_ID_NONE &&
         printing_plan->object_instance(printing_catalog_id) == nullptr))
        return nullptr;

    Slic3r::Printing::PrintingExtrusion extrusion;
    extrusion.region_island = region_island;
    extrusion.sregion_island_role = Slic3r::to_extrusion_role(source_role);
    extrusion.root = Slic3r::clone_or_empty(root);
    extrusion.printing_catalog_id = printing_catalog_id;
    Slic3r::to_tool_group(me)->extrusions.push_back(std::move(extrusion));
    return reinterpret_cast<printing_extrusion_handle *>(&Slic3r::to_tool_group(me)->extrusions.back());
}

printing_extrusion_handle *printing_tool_group_append_extrusion_clone_from_region_island(
    const printing_plan_handle *plan,
    printing_tool_group_handle *me,
    const layer_region_island_handle *source_region_island,
    raw_extrusion_role source_role,
    uint32_t printing_catalog_id)
{
    const Slic3r::LayerRegionIsland *region_island = Slic3r::to_region_island(source_region_island);
    const Slic3r::ExtrusionRole role = Slic3r::to_extrusion_role(source_role);
    if (me == nullptr || region_island == nullptr || !region_island->has_extrusion(role))
        return nullptr;

    return printing_tool_group_append_extrusion_clone(
        plan, me,
        source_region_island,
        source_role,
        reinterpret_cast<const extrusion_entity_handle *>(&region_island->extrusion(role)),
        printing_catalog_id);
}

printing_extrusion_handle *printing_tool_group_append_extrusion_move(
    const printing_plan_handle *plan,
    printing_tool_group_handle *me,
    const layer_region_island_handle *source_region_island,
    raw_extrusion_role source_role,
    extrusion_entity_handle *root,
    uint32_t printing_catalog_id)
{
    const Slic3r::Printing::PrintingPlan *printing_plan = Slic3r::to_plan(plan);
    const Slic3r::Printing::PrintingToolGroup *tool_group = Slic3r::to_tool_group(me);
    const Slic3r::LayerRegionIsland *region_island = Slic3r::to_region_island(source_region_island);
    const Slic3r::ExtrusionRole role = Slic3r::to_extrusion_role(source_role);
    if (!Slic3r::plan_owns_tool_group(printing_plan, tool_group) ||
        (printing_catalog_id != Slic3r::Printing::PRINTING_OBJECT_INSTANCE_ID_NONE &&
         printing_plan->object_instance(printing_catalog_id) == nullptr))
        return nullptr;

    Slic3r::Printing::PrintingExtrusion extrusion;
    extrusion.region_island = region_island;
    extrusion.sregion_island_role = Slic3r::to_extrusion_role(source_role);
    // Root handles point to heap-owned nodes and survive this vector reserve.
    Slic3r::to_tool_group(me)->extrusions.reserve(tool_group->extrusions.size() + 1);
    extrusion.root = Slic3r::move_or_empty(root);
    extrusion.printing_catalog_id = printing_catalog_id;
    Slic3r::to_tool_group(me)->extrusions.push_back(std::move(extrusion));
    return reinterpret_cast<printing_extrusion_handle *>(&Slic3r::to_tool_group(me)->extrusions.back());
}

int32_t printing_tool_group_move_extrusion(printing_tool_group_handle *me, uint32_t from_idx, uint32_t to_idx)
{
    return me != nullptr && Slic3r::move_vector_item(Slic3r::to_tool_group(me)->extrusions, from_idx, to_idx);
}

int32_t printing_tool_group_transfer_extrusion(printing_tool_group_handle *source,
                                               uint32_t source_idx,
                                               printing_tool_group_handle *destination)
{
    static_assert(std::is_nothrow_move_constructible_v<Slic3r::Printing::PrintingExtrusion>);
    static_assert(std::is_nothrow_move_assignable_v<Slic3r::Printing::PrintingExtrusion>);
    if (source == nullptr || destination == nullptr || source == destination ||
        source_idx >= Slic3r::to_tool_group(source)->extrusions.size())
        return 0;

    try {
        Slic3r::Printing::PrintingToolGroup &source_group = *Slic3r::to_tool_group(source);
        Slic3r::Printing::PrintingToolGroup &destination_group = *Slic3r::to_tool_group(destination);

        /* Build every allocation-prone result first. Once reserve succeeds,
           moving the unique extrusion root and erasing its old slot are
           noexcept operations, so callers never observe a half transfer. */
        std::vector<const Slic3r::LayerRegionIsland *> source_islands =
            Slic3r::region_islands_without_extrusion(source_group, source_idx);
        std::vector<const Slic3r::LayerRegionIsland *> destination_islands =
            Slic3r::region_islands_after_append(destination_group, source_group.extrusions[source_idx]);
        destination_group.extrusions.reserve(destination_group.extrusions.size() + 1);

        destination_group.extrusions.push_back(std::move(source_group.extrusions[source_idx]));
        source_group.extrusions.erase(source_group.extrusions.begin() + source_idx);
        source_group.region_islands.swap(source_islands);
        destination_group.region_islands.swap(destination_islands);
        return 1;
    } catch (...) {
        return 0;
    }
}

const layer_region_island_handle *printing_extrusion_get_region_island(const printing_extrusion_handle *me)
{
    return me == nullptr ?
               nullptr :
               reinterpret_cast<const layer_region_island_handle *>(Slic3r::to_printing_extrusion(me)->region_island);
}

raw_extrusion_role printing_extrusion_get_role(const printing_extrusion_handle *me)
{
    return me == nullptr ? RAW_EXTRUSION_ROLE_NONE : Slic3r::to_raw_role(Slic3r::to_printing_extrusion(me)->sregion_island_role);
}

void printing_extrusion_set_role(printing_extrusion_handle *me, raw_extrusion_role role)
{
    if (me != nullptr)
        Slic3r::to_printing_extrusion(me)->sregion_island_role = Slic3r::to_extrusion_role(role);
}

uint32_t printing_extrusion_get_printing_catalog_id(const printing_extrusion_handle *me)
{
    return me == nullptr ?
        Slic3r::Printing::PRINTING_OBJECT_INSTANCE_ID_NONE :
        Slic3r::to_printing_extrusion(me)->printing_catalog_id;
}

extrusion_entity_handle *printing_extrusion_get_root_mutable(printing_extrusion_handle *me)
{
    if (me == nullptr || !Slic3r::to_printing_extrusion(me)->root)
        return nullptr;
    return reinterpret_cast<extrusion_entity_handle *>(Slic3r::to_printing_extrusion(me)->root.get());
}

const extrusion_entity_handle *printing_extrusion_get_root(const printing_extrusion_handle *me)
{
    if (me == nullptr || !Slic3r::to_printing_extrusion(me)->root)
        return nullptr;
    return reinterpret_cast<const extrusion_entity_handle *>(Slic3r::to_printing_extrusion(me)->root.get());
}

int32_t printing_extrusion_set_root_clone(printing_extrusion_handle *me, const extrusion_entity_handle *root)
{
    if (me == nullptr)
        return 0;
    Slic3r::to_printing_extrusion(me)->root = Slic3r::clone_or_empty(root);
    return 1;
}

int32_t printing_extrusion_set_root_move(printing_extrusion_handle *me, extrusion_entity_handle *root)
{
    if (me == nullptr)
        return 0;
    Slic3r::to_printing_extrusion(me)->root = Slic3r::move_or_empty(root);
    return 1;
}

} // extern "C"
