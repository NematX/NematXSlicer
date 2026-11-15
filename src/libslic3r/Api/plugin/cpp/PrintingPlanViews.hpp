///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_plugin_cpp_PrintingPlanViews_hpp_
#define slic3r_Api_plugin_cpp_PrintingPlanViews_hpp_

#include <cassert>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

#include "libslic3r/Api/plugin/c/steps/slic3r_step_ordering.h"
#include "libslic3r/Api/plugin/c/slic3r_printing_plan.h"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"

namespace slic3r_api {

class PrintingPlan;
class PrintingScopeEvents;
class PrintingGroup;
class PrintingLayerGroup;
class PrintingToolGroup;
class PrintingExtrusion;
class ConstPrintingPlan;
class ConstPrintingScopeEvents;
class ConstPrintingGroup;
class ConstPrintingLayerGroup;
class ConstPrintingToolGroup;
class ConstPrintingExtrusion;

/*
PrintingPlan C++ views
======================

STEP_ORDERING plugins receive two pieces of data: the read-only Print and a
mutable PrintingPlan. The plan is a work copy owned by the host for the duration
of the ordering step. It points back to source Layers and LayerRegionIslands for
context, but every PrintingExtrusion owns a cloned extrusion root that plugins
may reorder.

Ownership rules:
- These classes are borrowed views. They never free the handles.
- Child views are invalidated when their parent vector is cleared, appended to
  or reordered.
- PrintingExtrusion::root() is the mutable clone that ordering plugins should
  modify. It is not the source LayerRegionIsland extrusion tree.

Typical step flow:

    const run_ctx_extrusion_ordering *ctx = plugin_ctx_as_extrusion_ordering(run_ctx);
    PrintingPlan plan(ctx->plan);
    plan.clear();                                // builder plugin
    PrintingGroup group = plan.append_group();
    // append layer/tool/extrusion groups with the helpers below
    for (uint32_t idx = 0; idx < plan.group_count(); ++idx)
        my_plugin_order_tool_groups(plan.group(idx));
    for (uint32_t idx = 0; idx < plan.group_count(); ++idx)
        my_plugin_order_extrusion_trees(plan.group(idx));

The views expose plan access and editing primitives. Algorithms such as by-layer or
by-object construction belong in STEP_ORDERING plugins, where they can be
replaced independently.

ConstPrintingPlan::uses_single_extruder_multi_material() resolves the configured
SEMM mode against concrete tool visits. Call it after ordering, so tools selected
by height markers are included even when their visits contain no extrusion.
A color replacement on the same tool does not introduce another logical tool.

Property guide: [Using Plugin Properties](/doc/plugins/properties.md)

Extrusion guide: [Using Unified Extrusion Entities](/doc/plugins/extrusions.md)
*/

/*
Complete descriptor copied from the plan-level instance catalog.

The numeric fields and name() are copied values. object() and
projected_outline() remain borrowed from the plan; do not retain them after the
source Print or plan is destroyed or structurally changed.
*/
class PrintingObjectInstance
{
public:
    explicit PrintingObjectInstance(c_printing_object_instance value) : m_value(value) {}

    bool has_object() const { return m_value.object != nullptr; }
    Object object() const { return Object(m_value.object); }
    uint64_t object_instance_idx() const { return m_value.object_instance_idx; }
    uint32_t id() const { return m_value.id; }
    std::string name() const { return m_value.name == nullptr ? std::string() : std::string(m_value.name); }
    c_point projected_outline_center() const { return m_value.projected_outline_center; }
    bool has_projected_outline() const { return m_value.projected_outline != nullptr; }
    Polygon projected_outline() const { return Polygon(m_value.projected_outline); }

private:
    c_printing_object_instance m_value;
};

/* Read-only event sequence used by G-code writers and firmware sessions. */
class ConstPrintingScopeEvents
{
public:
    explicit ConstPrintingScopeEvents(const printing_scope_events_handle *handle) : m_handle(handle) {}
    ConstPrintingScopeEvents(const PrintingScopeEvents &events);

    bool valid() const { return m_handle != nullptr; }
    const printing_scope_events_handle *handle() const { return m_handle; }
    bool has_before() const { return printing_scope_events_has_before(handle()) != 0; }
    bool has_after() const { return printing_scope_events_has_after(handle()) != 0; }
    ExtrusionEntity before() const { return ExtrusionEntity(printing_scope_events_get_before(handle())); }
    ExtrusionEntity after() const { return ExtrusionEntity(printing_scope_events_get_after(handle())); }

private:
    const printing_scope_events_handle *m_handle = nullptr;
};

/* Read-only view of one finalized plan extrusion. */
class ConstPrintingExtrusion
{
public:
    explicit ConstPrintingExtrusion(const printing_extrusion_handle *handle) : m_handle(handle) {}
    ConstPrintingExtrusion(const PrintingExtrusion &extrusion);

    bool valid() const { return m_handle != nullptr; }
    const printing_extrusion_handle *handle() const { return m_handle; }
    LayerRegionIsland region_island() const {
        const layer_region_island_handle *source = printing_extrusion_get_region_island(handle());
        return source != nullptr ? LayerRegionIsland(source) : LayerRegionIsland();
    }
    raw_extrusion_role role() const { return printing_extrusion_get_role(handle()); }
    uint32_t printing_catalog_id() const {
        return printing_extrusion_get_printing_catalog_id(handle());
    }
    ExtrusionEntity root() const {
        return ExtrusionEntity(printing_extrusion_get_root(handle()));
    }

private:
    const printing_extrusion_handle *m_handle = nullptr;
};

/* Read-only visit of one finalized tool section. */
class ConstPrintingToolGroup
{
public:
    explicit ConstPrintingToolGroup(const printing_tool_group_handle *handle) : m_handle(handle) {}
    ConstPrintingToolGroup(const PrintingToolGroup &tool_group);

    bool valid() const { return m_handle != nullptr; }
    const printing_tool_group_handle *handle() const { return m_handle; }
    ConstPrintingScopeEvents events() const {
        return ConstPrintingScopeEvents(printing_tool_group_get_events(handle()));
    }
    ConstPluginProperties properties() const {
        return ConstPluginProperties(printing_tool_group_get_properties(handle()));
    }
    extruder_index_t extruder_id() const { return printing_tool_group_get_extruder_id(handle()); }
    uint32_t region_island_count() const {
        return printing_tool_group_count_region_island(handle());
    }
    LayerRegionIsland region_island(uint32_t idx) const {
        return LayerRegionIsland(printing_tool_group_get_region_island(handle(), idx));
    }
    uint32_t extrusion_count() const { return printing_tool_group_count_extrusion(handle()); }
    ConstPrintingExtrusion extrusion(uint32_t idx) const {
        return ConstPrintingExtrusion(printing_tool_group_get_extrusion(handle(), idx));
    }

private:
    const printing_tool_group_handle *m_handle = nullptr;
};

/* Read-only visit of one finalized plan layer. */
class ConstPrintingLayerGroup
{
public:
    explicit ConstPrintingLayerGroup(const printing_layer_group_handle *handle) : m_handle(handle) {}
    ConstPrintingLayerGroup(const PrintingLayerGroup &layer_group);

    bool valid() const { return m_handle != nullptr; }
    const printing_layer_group_handle *handle() const { return m_handle; }
    ConstPrintingScopeEvents events() const {
        return ConstPrintingScopeEvents(printing_layer_group_get_events(handle()));
    }
    ConstPluginProperties properties() const {
        return ConstPluginProperties(printing_layer_group_get_properties(handle()));
    }
    coord_t print_z() const { return printing_layer_group_get_print_z(handle()); }
    uint32_t layer_count() const { return printing_layer_group_count_layer(handle()); }
    Layer layer(uint32_t idx) const {
        return Layer(printing_layer_group_get_layer(handle(), idx));
    }
    uint32_t tool_group_count() const {
        return printing_layer_group_count_tool_group(handle());
    }
    ConstPrintingToolGroup tool_group(uint32_t idx) const {
        return ConstPrintingToolGroup(printing_layer_group_get_tool_group(handle(), idx));
    }

private:
    const printing_layer_group_handle *m_handle = nullptr;
};

/* Read-only visit of one finalized independent plan batch. */
class ConstPrintingGroup
{
public:
    explicit ConstPrintingGroup(const printing_group_handle *handle) : m_handle(handle) {}
    ConstPrintingGroup(const PrintingGroup &group);

    bool valid() const { return m_handle != nullptr; }
    const printing_group_handle *handle() const { return m_handle; }
    ConstPrintingScopeEvents events() const {
        return ConstPrintingScopeEvents(printing_group_get_events(handle()));
    }
    ConstPluginProperties properties() const {
        return ConstPluginProperties(printing_group_get_properties(handle()));
    }
    uint32_t object_instance_count() const {
        return printing_group_count_object_instance(handle());
    }
    uint32_t printing_catalog_id(uint32_t idx) const {
        return printing_group_get_printing_catalog_id(handle(), idx);
    }
    uint32_t layer_group_count() const {
        return printing_group_count_layer_group(handle());
    }
    ConstPrintingLayerGroup layer_group(uint32_t idx) const {
        return ConstPrintingLayerGroup(printing_group_get_layer_group(handle(), idx));
    }

private:
    const printing_group_handle *m_handle = nullptr;
};

/* Read-only view of the finalized plan supplied to STEP_GCODE. */
class ConstPrintingPlan
{
public:
    ConstPrintingPlan() = default;
    explicit ConstPrintingPlan(const printing_plan_handle *handle) : m_handle(handle) {}
    ConstPrintingPlan(const PrintingPlan &plan);

    bool valid() const { return m_handle != nullptr; }
    const printing_plan_handle *handle() const { return m_handle; }
    ConstPrintingScopeEvents events() const {
        return ConstPrintingScopeEvents(printing_plan_get_events(handle()));
    }
    ConstPluginProperties properties() const {
        return ConstPluginProperties(printing_plan_get_properties(handle()));
    }
    uint32_t object_instance_count() const {
        return printing_plan_count_object_instance(handle());
    }
    PrintingObjectInstance object_instance(uint32_t id) const {
        return PrintingObjectInstance(
            printing_plan_get_object_instance_by_printing_catalog_id(handle(), id));
    }
    c_point object_instance_shift(const ConstPrintingExtrusion &extrusion) const;
    uint32_t group_count() const { return printing_plan_count_group(handle()); }
    ConstPrintingGroup group(uint32_t idx) const {
        return ConstPrintingGroup(printing_plan_get_group(handle(), idx));
    }

    // Enable shared-nozzle operation only for an enabled setting and two distinct
    // concrete tool visits anywhere in the borrowed plan. Scripts are not parsed.
    bool uses_single_extruder_multi_material(bool configured) const {
        if (!configured || !valid())
            return false;
        extruder_index_t first = EXTRUDER_ID_UNKNOWN;
        for (uint32_t group_idx = 0; group_idx < group_count(); ++group_idx) {
            const ConstPrintingGroup current_group = group(group_idx);
            for (uint32_t layer_idx = 0; layer_idx < current_group.layer_group_count(); ++layer_idx) {
                const ConstPrintingLayerGroup layer = current_group.layer_group(layer_idx);
                for (uint32_t tool_idx = 0; tool_idx < layer.tool_group_count(); ++tool_idx) {
                    const extruder_index_t tool = layer.tool_group(tool_idx).extruder_id();
                    if (tool == EXTRUDER_ID_UNKNOWN)
                        continue;
                    if (first == EXTRUDER_ID_UNKNOWN)
                        first = tool;
                    else if (tool != first)
                        return true;
                }
            }
        }
        return false;
    }

private:
    const printing_plan_handle *m_handle = nullptr;
};

/*
Borrowed view over the fixed before/after sequences of one plan scope.

The roots are intentionally read-only so plugin code cannot make the enclosing
sequence sortable or replace it. Mutation is limited to appending complete
event trees in their execution order.
*/
class PrintingScopeEvents
{
public:
    explicit PrintingScopeEvents(printing_scope_events_handle *handle) : m_handle(handle) {}

    bool valid() const { return m_handle != nullptr; }
    printing_scope_events_handle *mutable_handle() { return m_handle; }
    const printing_scope_events_handle *handle() const { return m_handle; }

    bool has_before() const { return printing_scope_events_has_before(handle()) != 0; }
    bool has_after() const { return printing_scope_events_has_after(handle()) != 0; }
    ExtrusionEntity before() const { return ExtrusionEntity(printing_scope_events_get_before(handle())); }
    ExtrusionEntity after() const { return ExtrusionEntity(printing_scope_events_get_after(handle())); }

    MutableExtrusionEntity append_before_clone(const ExtrusionEntity &event) {
        return MutableExtrusionEntity(
            printing_scope_events_append_before_clone(mutable_handle(), event.handle()));
    }
    MutableExtrusionEntity append_before_move(MutableExtrusionEntity event) {
        return MutableExtrusionEntity(
            printing_scope_events_append_before_move(mutable_handle(), event.mutable_handle()));
    }
    MutableExtrusionEntity append_after_clone(const ExtrusionEntity &event) {
        return MutableExtrusionEntity(
            printing_scope_events_append_after_clone(mutable_handle(), event.handle()));
    }
    MutableExtrusionEntity append_after_move(MutableExtrusionEntity event) {
        return MutableExtrusionEntity(
            printing_scope_events_append_after_move(mutable_handle(), event.mutable_handle()));
    }

private:
    printing_scope_events_handle *m_handle = nullptr;
};

class PrintingExtrusion
{
public:
    explicit PrintingExtrusion(printing_extrusion_handle *handle) : m_handle(handle) {}

    bool valid() const { return m_handle != nullptr; }
    printing_extrusion_handle *mutable_handle() { return m_handle; }
    const printing_extrusion_handle *handle() const { return m_handle; }

    LayerRegionIsland region_island() const {
        const layer_region_island_handle *region_island =
            printing_extrusion_get_region_island(handle());
        // Support and print-level auxiliary extrusions may have no regional
        // owner. Preserve that valid model state as an invalid optional view
        // so callers can test valid() before asking for region data.
        return region_island != nullptr ? LayerRegionIsland(region_island) :
                                          LayerRegionIsland();
    }

    raw_extrusion_role role() const { return printing_extrusion_get_role(handle()); }
    void set_role(raw_extrusion_role role) { printing_extrusion_set_role(mutable_handle(), role); }

    uint32_t printing_catalog_id() const { return printing_extrusion_get_printing_catalog_id(handle()); }

    ExtrusionEntity root() const {
        return ExtrusionEntity(printing_extrusion_get_root(handle()));
    }

    MutableExtrusionEntity mutable_root() {
        return MutableExtrusionEntity(printing_extrusion_get_root_mutable(mutable_handle()));
    }

    bool set_root_clone(const ExtrusionEntity &root) {
        return printing_extrusion_set_root_clone(mutable_handle(), root.handle()) != 0;
    }

    bool set_root_move(MutableExtrusionEntity root) {
        return printing_extrusion_set_root_move(mutable_handle(), root.mutable_handle()) != 0;
    }

private:
    printing_extrusion_handle *m_handle = nullptr;
};

class PrintingToolGroup
{
public:
    explicit PrintingToolGroup(printing_tool_group_handle *handle) : m_handle(handle) {}

    bool valid() const { return m_handle != nullptr; }
    printing_tool_group_handle *mutable_handle() { return m_handle; }
    const printing_tool_group_handle *handle() const { return m_handle; }

    PrintingScopeEvents events() {
        return PrintingScopeEvents(printing_tool_group_get_events_mutable(mutable_handle()));
    }
    ConstPrintingScopeEvents events() const {
        return ConstPrintingScopeEvents(printing_tool_group_get_events(handle()));
    }

    PluginProperties properties() {
        return PluginProperties(printing_tool_group_get_properties_mutable(mutable_handle()));
    }
    ConstPluginProperties properties() const {
        return ConstPluginProperties(printing_tool_group_get_properties(handle()));
    }

    extruder_index_t extruder_id() const { return printing_tool_group_get_extruder_id(handle()); }
    void set_extruder_id(int64_t extruder_id) {
        if (extruder_id < EXTRUDER_ID_UNKNOWN ||
            extruder_id > (std::numeric_limits<extruder_index_t>::max)())
            throw std::out_of_range("The extruder id does not fit in extruder_index_t.");
        printing_tool_group_set_extruder_id(
            mutable_handle(), static_cast<extruder_index_t>(extruder_id));
    }

    uint32_t region_island_count() const { return printing_tool_group_count_region_island(handle()); }
    LayerRegionIsland region_island(uint32_t idx) const {
        return LayerRegionIsland(printing_tool_group_get_region_island(handle(), idx));
    }
    void append_region_island(const LayerRegionIsland &region_island) {
        printing_tool_group_append_region_island(mutable_handle(), region_island.handle());
    }

    uint32_t extrusion_count() const { return printing_tool_group_count_extrusion(handle()); }
    PrintingExtrusion extrusion(uint32_t idx) {
        return PrintingExtrusion(printing_tool_group_get_extrusion_mutable(mutable_handle(), idx));
    }
    ConstPrintingExtrusion extrusion(uint32_t idx) const {
        return ConstPrintingExtrusion(printing_tool_group_get_extrusion(handle(), idx));
    }

    PrintingExtrusion append_extrusion_clone(PrintingPlan &plan,
                                             const LayerRegionIsland &source,
                                             raw_extrusion_role role,
                                             const ExtrusionEntity &root,
                                             uint32_t printing_catalog_id);

    PrintingExtrusion append_extrusion_clone_from_region_island(
        PrintingPlan &plan, const LayerRegionIsland &source,
        raw_extrusion_role role, uint32_t printing_catalog_id);

    PrintingExtrusion append_extrusion_move(PrintingPlan &plan,
                                            const LayerRegionIsland &source,
                                            raw_extrusion_role role,
                                            MutableExtrusionEntity root,
                                            uint32_t printing_catalog_id);

    bool move_extrusion(uint32_t from_idx, uint32_t to_idx) {
        return printing_tool_group_move_extrusion(mutable_handle(), from_idx, to_idx) != 0;
    }

    bool transfer_extrusion_to(uint32_t source_idx, PrintingToolGroup &destination) {
        return printing_tool_group_transfer_extrusion(
                   mutable_handle(), source_idx, destination.mutable_handle()) != 0;
    }

private:
    printing_tool_group_handle *m_handle = nullptr;
};

class PrintingLayerGroup
{
public:
    explicit PrintingLayerGroup(printing_layer_group_handle *handle) : m_handle(handle) {}

    bool valid() const { return m_handle != nullptr; }
    printing_layer_group_handle *mutable_handle() { return m_handle; }
    const printing_layer_group_handle *handle() const { return m_handle; }

    PrintingScopeEvents events() {
        return PrintingScopeEvents(printing_layer_group_get_events_mutable(mutable_handle()));
    }
    ConstPrintingScopeEvents events() const {
        return ConstPrintingScopeEvents(printing_layer_group_get_events(handle()));
    }

    PluginProperties properties() {
        return PluginProperties(printing_layer_group_get_properties_mutable(mutable_handle()));
    }
    ConstPluginProperties properties() const {
        return ConstPluginProperties(printing_layer_group_get_properties(handle()));
    }

    coord_t print_z() const { return printing_layer_group_get_print_z(handle()); }
    void set_print_z(coord_t print_z) { printing_layer_group_set_print_z(mutable_handle(), print_z); }

    uint32_t layer_count() const { return printing_layer_group_count_layer(handle()); }
    Layer layer(uint32_t idx) const { return Layer(printing_layer_group_get_layer(handle(), idx)); }
    void append_layer(const Layer &layer) {
        printing_layer_group_append_layer(mutable_handle(), layer.handle());
    }

    uint32_t tool_group_count() const { return printing_layer_group_count_tool_group(handle()); }
    PrintingToolGroup tool_group(uint32_t idx) {
        return PrintingToolGroup(printing_layer_group_get_tool_group_mutable(mutable_handle(), idx));
    }
    ConstPrintingToolGroup tool_group(uint32_t idx) const {
        return ConstPrintingToolGroup(printing_layer_group_get_tool_group(handle(), idx));
    }
    PrintingToolGroup append_tool_group(int64_t extruder_id) {
        if (extruder_id < EXTRUDER_ID_UNKNOWN ||
            extruder_id > (std::numeric_limits<extruder_index_t>::max)())
            throw std::out_of_range("The extruder id does not fit in extruder_index_t.");
        return PrintingToolGroup(printing_layer_group_append_tool_group(
            mutable_handle(), static_cast<extruder_index_t>(extruder_id)));
    }
    bool move_tool_group(uint32_t from_idx, uint32_t to_idx) {
        return printing_layer_group_move_tool_group(mutable_handle(), from_idx, to_idx) != 0;
    }
    bool remove_empty_tool_group(uint32_t idx) {
        return printing_layer_group_remove_empty_tool_group(mutable_handle(), idx) != 0;
    }

private:
    printing_layer_group_handle *m_handle = nullptr;
};

class PrintingGroup
{
public:
    explicit PrintingGroup(printing_group_handle *handle) : m_handle(handle) {}

    bool valid() const { return m_handle != nullptr; }
    printing_group_handle *mutable_handle() { return m_handle; }
    const printing_group_handle *handle() const { return m_handle; }

    PrintingScopeEvents events() {
        return PrintingScopeEvents(printing_group_get_events_mutable(mutable_handle()));
    }
    ConstPrintingScopeEvents events() const {
        return ConstPrintingScopeEvents(printing_group_get_events(handle()));
    }

    PluginProperties properties() {
        return PluginProperties(printing_group_get_properties_mutable(mutable_handle()));
    }
    ConstPluginProperties properties() const {
        return ConstPluginProperties(printing_group_get_properties(handle()));
    }

    void clear() { printing_group_clear(mutable_handle()); }

    uint32_t object_instance_count() const { return printing_group_count_object_instance(handle()); }
    uint32_t printing_catalog_id(uint32_t idx) const {
        return printing_group_get_printing_catalog_id(handle(), idx);
    }
    void append_printing_catalog_id(const PrintingPlan &plan, uint32_t id);

    uint32_t layer_group_count() const { return printing_group_count_layer_group(handle()); }
    PrintingLayerGroup layer_group(uint32_t idx) {
        return PrintingLayerGroup(printing_group_get_layer_group_mutable(mutable_handle(), idx));
    }
    ConstPrintingLayerGroup layer_group(uint32_t idx) const {
        return ConstPrintingLayerGroup(printing_group_get_layer_group(handle(), idx));
    }
    PrintingLayerGroup append_layer_group(coord_t print_z) {
        return PrintingLayerGroup(printing_group_append_layer_group(mutable_handle(), print_z));
    }
    bool move_layer_group(uint32_t from_idx, uint32_t to_idx) {
        return printing_group_move_layer_group(mutable_handle(), from_idx, to_idx) != 0;
    }

private:
    printing_group_handle *m_handle = nullptr;
};

class PrintingPlan
{
public:
    PrintingPlan() = default;
    explicit PrintingPlan(printing_plan_handle *handle) : m_handle(handle) {}

    bool valid() const { return m_handle != nullptr; }
    printing_plan_handle *mutable_handle() { return m_handle; }
    const printing_plan_handle *handle() const { return m_handle; }

    PrintingScopeEvents events() {
        return PrintingScopeEvents(printing_plan_get_events_mutable(mutable_handle()));
    }
    ConstPrintingScopeEvents events() const {
        return ConstPrintingScopeEvents(printing_plan_get_events(handle()));
    }

    PluginProperties properties() {
        return PluginProperties(printing_plan_get_properties_mutable(mutable_handle()));
    }
    ConstPluginProperties properties() const {
        return ConstPluginProperties(printing_plan_get_properties(handle()));
    }

    void clear() { printing_plan_clear(mutable_handle()); }

    uint32_t find_or_add_object_instance(const Object &object, uint64_t object_instance_idx) {
        const uint32_t id = printing_plan_register_object_instance(
            mutable_handle(), object.handle(), object_instance_idx, nullptr, nullptr);
        if (id == UINT32_MAX)
            throw std::invalid_argument(
                "The object instance is invalid or cannot be added to the PrintingPlan catalog.");
        return id;
    }
    uint32_t add_synthetic_object_instance(const std::string &name,
                                           const Polygon &outline) {
        const uint32_t id = printing_plan_register_object_instance(
            mutable_handle(), nullptr, (std::numeric_limits<uint64_t>::max)(),
            name.c_str(), outline.handle());
        if (id == UINT32_MAX)
            throw std::invalid_argument("The synthetic object instance is invalid.");
        return id;
    }
    void set_object_instance_metadata(uint32_t id, const std::string &name,
                                      const Polygon &outline) {
        if (!printing_plan_set_object_instance_metadata(
                mutable_handle(), id, name.c_str(), outline.handle()))
            throw std::invalid_argument("The object instance metadata is invalid.");
    }
    uint32_t object_instance_count() const {
        return printing_plan_count_object_instance(handle());
    }
    PrintingObjectInstance object_instance(uint32_t id) const {
        return PrintingObjectInstance(printing_plan_get_object_instance_by_printing_catalog_id(handle(), id));
    }
    c_point object_instance_shift(const PrintingExtrusion &extrusion) const;
    c_point object_instance_shift(const ConstPrintingExtrusion &extrusion) const;

    uint32_t group_count() const { return printing_plan_count_group(handle()); }
    PrintingGroup group(uint32_t idx) {
        return PrintingGroup(printing_plan_get_group_mutable(mutable_handle(), idx));
    }
    ConstPrintingGroup group(uint32_t idx) const {
        return ConstPrintingGroup(printing_plan_get_group(handle(), idx));
    }
    PrintingGroup append_group() {
        return PrintingGroup(printing_plan_append_group(mutable_handle()));
    }
    bool move_group(uint32_t from_idx, uint32_t to_idx) {
        return printing_plan_move_group(mutable_handle(), from_idx, to_idx) != 0;
    }

private:
    printing_plan_handle *m_handle = nullptr;
};

inline ConstPrintingScopeEvents::ConstPrintingScopeEvents(const PrintingScopeEvents &events)
    : m_handle(events.handle())
{}

inline ConstPrintingExtrusion::ConstPrintingExtrusion(const PrintingExtrusion &extrusion)
    : m_handle(extrusion.handle())
{}

inline ConstPrintingToolGroup::ConstPrintingToolGroup(const PrintingToolGroup &tool_group)
    : m_handle(tool_group.handle())
{}

inline ConstPrintingLayerGroup::ConstPrintingLayerGroup(const PrintingLayerGroup &layer_group)
    : m_handle(layer_group.handle())
{}

inline ConstPrintingGroup::ConstPrintingGroup(const PrintingGroup &group)
    : m_handle(group.handle())
{}

inline ConstPrintingPlan::ConstPrintingPlan(const PrintingPlan &plan)
    : m_handle(plan.handle())
{}

inline PrintingExtrusion PrintingToolGroup::append_extrusion_clone(
    PrintingPlan &plan, const LayerRegionIsland &source,
    raw_extrusion_role role, const ExtrusionEntity &root,
    uint32_t printing_catalog_id)
{
    return PrintingExtrusion(printing_tool_group_append_extrusion_clone(
        plan.mutable_handle(), mutable_handle(), source.handle(), role,
        root.handle(), printing_catalog_id));
}

inline PrintingExtrusion PrintingToolGroup::append_extrusion_clone_from_region_island(
    PrintingPlan &plan, const LayerRegionIsland &source,
    raw_extrusion_role role, uint32_t printing_catalog_id)
{
    return PrintingExtrusion(printing_tool_group_append_extrusion_clone_from_region_island(
        plan.mutable_handle(), mutable_handle(), source.handle(), role,
        printing_catalog_id));
}

inline PrintingExtrusion PrintingToolGroup::append_extrusion_move(
    PrintingPlan &plan, const LayerRegionIsland &source,
    raw_extrusion_role role, MutableExtrusionEntity root,
    uint32_t printing_catalog_id)
{
    return PrintingExtrusion(printing_tool_group_append_extrusion_move(
        plan.mutable_handle(), mutable_handle(), source.handle(), role,
        root.mutable_handle(), printing_catalog_id));
}

/* Resolve the local Object instance index only when a consumer needs source
   geometry or settings. PrintingExtrusion itself stores only the stable ID. */
inline size_t printing_plan_object_instance_idx(const PrintingPlan &plan,
                                           const PrintingExtrusion &extrusion)
{
    const uint32_t printing_catalog_id = extrusion.printing_catalog_id();
    // none -> auxiliary -> only one instance per auxiliary object
    if (printing_catalog_id == SLIC3R_PRINTING_OBJECT_INSTANCE_ID_NONE)
        return 0;
    if (printing_catalog_id >= plan.object_instance_count())
        return size_t(-1);
    return static_cast<size_t>(plan.object_instance(printing_catalog_id).object_instance_idx());
}

inline size_t printing_plan_object_instance_idx(const ConstPrintingPlan &plan,
                                           const ConstPrintingExtrusion &extrusion)
{
    const uint32_t printing_catalog_id = extrusion.printing_catalog_id();
    if (printing_catalog_id == SLIC3R_PRINTING_OBJECT_INSTANCE_ID_NONE)
        return 0;
    if (printing_catalog_id >= plan.object_instance_count())
        return size_t(-1);
    return static_cast<size_t>(plan.object_instance(printing_catalog_id).object_instance_idx());
}

inline c_point PrintingPlan::object_instance_shift(const PrintingExtrusion &extrusion) const
{
    const uint32_t printing_catalog_id = extrusion.printing_catalog_id();
    // if the PrintingExtrusion doesn't have an object, it means it's already in the  platter coordinates.
    if (printing_catalog_id == SLIC3R_PRINTING_OBJECT_INSTANCE_ID_NONE)
        return {0,0};
    if (printing_catalog_id >= object_instance_count())
        throw std::invalid_argument(
            "The PrintingExtrusion references an unknown catalog entry.");

    const PrintingObjectInstance instance = object_instance(printing_catalog_id);
    if (!instance.has_object())
        throw std::invalid_argument(
            "A synthetic PrintingObjectInstance has no source-object shift.");

    const uint64_t object_instance_idx = instance.object_instance_idx();
    const Object object = instance.object();
    if (object_instance_idx > UINT32_MAX ||
        object_instance_idx >= object.instance_count())
        throw std::invalid_argument(
            "The PrintingObjectInstance references an invalid local instance.");

    return object.instance_shift(static_cast<uint32_t>(object_instance_idx));
}

inline c_point ConstPrintingPlan::object_instance_shift(
    const ConstPrintingExtrusion &extrusion) const
{
    const uint32_t printing_catalog_id = extrusion.printing_catalog_id();
    if (printing_catalog_id == SLIC3R_PRINTING_OBJECT_INSTANCE_ID_NONE)
        return {0, 0};
    if (printing_catalog_id >= object_instance_count())
        throw std::invalid_argument(
            "The PrintingExtrusion references an unknown catalog entry.");

    const PrintingObjectInstance instance = object_instance(printing_catalog_id);
    if (!instance.has_object())
        throw std::invalid_argument(
            "A synthetic PrintingObjectInstance has no source-object shift.");

    const uint64_t object_instance_idx = instance.object_instance_idx();
    const Object object = instance.object();
    if (object_instance_idx > UINT32_MAX || object_instance_idx >= object.instance_count())
        throw std::invalid_argument(
            "The PrintingObjectInstance references an invalid local instance.");

    return object.instance_shift(static_cast<uint32_t>(object_instance_idx));
}

inline c_point PrintingPlan::object_instance_shift(
    const ConstPrintingExtrusion &extrusion) const
{
    return ConstPrintingPlan(handle()).object_instance_shift(extrusion);
}

inline void PrintingGroup::append_printing_catalog_id(
    const PrintingPlan &plan, uint32_t id)
{
    if (printing_group_append_printing_catalog_id(plan.handle(), mutable_handle(), id) == 0)
        throw std::invalid_argument("The object instance ID does not belong to the PrintingPlan.");
}

} // namespace slic3r_api

#endif // slic3r_Api_plugin_cpp_PrintingPlanViews_hpp_
