///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "CustomGCodePerPrintZ.hpp"

#include <cassert>
#include <iterator>
#include <map>
#include <optional>
#include <stdexcept>
#include <vector>

#include "CustomGCodePerPrintZRecord.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_ordering.h"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"

/*
Custom height-marker ordering
=============================

This file contains two STEP_ORDERING plugins that prepare the PrintingPlan for
custom height markers. They handle different marker effects and run in a fixed
priority order: normal object extrusions are assigned to the selected tool
first, then the tool required by a color-change event is placed first on its
layer.

The normal execution flow is:

    CustomGCodeToolOverrides (priority 50)
    |-- index ToolChange rows by their scaled Z
    |-- keep each group's first object tool below all effective ToolChanges
    |-- select the last MultiAsSingle ToolChange at or below each layer's Z
    |-- move only normal object extrusion roots to the selected tool group
    `-- remove empty tool groups that no longer contain work or events

    default ordering plugins
    `-- order the resulting tool and extrusion groups

    CustomGCodeEventTools (priority 150)
    |-- resolve every ColorChange or converted ToolChange target in advance
    |-- reject conflicting targets on the same layer before mutation
    |-- create the target tool group when it is absent
    `-- move the target tool group to the front of its layer

The two plugins accept only marker-table and PrintingPlan modes that describe
the same single- or multi-extruder family. Tool IDs are zero-based inside the
plan. `CustomGCodeToolOverrides` leaves support and auxiliary extrusions in
their original groups, while `CustomGCodeEventTools` may create an empty tool
group because its presence is a required structural event for the later
G-code stage.

The plan API can invalidate views when groups are moved or erased, so object
extrusions are transferred while source views are reacquired for each
operation. These plugins only prepare ordering and tool visits; they do not
emit the final custom G-code events themselves.
*/

namespace slic3r_api::GCodeGeneration::CustomGCodePerPrintZPlugin {
namespace {

const char *const k_no_dependencies[] = {nullptr};

/*
Find the first tool that prints a normal object extrusion in one group. This is
the initial state of the ToolChange timeline; support and auxiliary extrusions
must not choose the object tool.
*/
std::optional<extruder_index_t> first_object_tool(const ConstPrintingPlan &plan,
                                           const ConstPrintingGroup &group);

/* Return the index of one tool visit, or uint32_t(-1) when it is absent. */
uint32_t find_tool_group(const ConstPrintingLayerGroup &layer, extruder_index_t extruder_id);

/*
Move every normal object extrusion on a layer to the selected tool while
leaving support and auxiliary roots untouched. The plan API updates source
contexts during each transfer, then obsolete empty source visits are removed.
*/
void move_object_extrusions_to_tool(PrintingPlan &plan,
                                     PrintingGroup &group,
                                     PrintingLayerGroup &layer,
                                     extruder_index_t extruder_id);

/*
Assign each layer's object roots to the last MultiAsSingle ToolChange
at or below its Z, regardless of group order. Below all changes, retain the group's
first object tool. Borrowed table rows remain unchanged; conversion and plan API
failures propagate. Other modes leave the plan unchanged.
*/
void apply_tool_overrides(const CustomGCodeTable &table,
                           PrintingPlan &plan,
                          Slic3r::CustomGCode::Mode print_mode,
                          uint32_t extruder_count);

/*
Prepare the first tool visit of each layer for color-change execution. All
rows map once across the complete plan, with pointers borrowing the table. All
targets are validated before mutation so conflicting markers cannot leave a
partially reordered plan; a missing visit is created intentionally as an empty
structural tool change.
*/
void apply_color_event_tools(const CustomGCodeTable &table,
                              PrintingPlan &plan,
                             Slic3r::CustomGCode::Mode print_mode,
                             uint32_t extruder_count);

std::optional<extruder_index_t> first_object_tool(const ConstPrintingPlan &plan,
                                           const ConstPrintingGroup &group)
{
    for (uint32_t layer_idx = 0; layer_idx < group.layer_group_count(); ++layer_idx) {
        const ConstPrintingLayerGroup layer = group.layer_group(layer_idx);
        for (uint32_t tool_idx = 0; tool_idx < layer.tool_group_count(); ++tool_idx) {
            const ConstPrintingToolGroup tool = layer.tool_group(tool_idx);
            for (uint32_t extrusion_idx = 0; extrusion_idx < tool.extrusion_count(); ++extrusion_idx)
                if (printing_extrusion_is_object(
                        plan, group, tool.extrusion(extrusion_idx)))
                    return tool.extruder_id();
        }
    }
    return std::nullopt;
}

uint32_t find_tool_group(const ConstPrintingLayerGroup &layer, extruder_index_t extruder_id)
{
    for (uint32_t idx = 0; idx < layer.tool_group_count(); ++idx)
        if (layer.tool_group(idx).extruder_id() == extruder_id)
            return idx;
    return uint32_t(-1);
}

void move_object_extrusions_to_tool(PrintingPlan &plan,
                                     PrintingGroup &group,
                                     PrintingLayerGroup &layer,
                                     extruder_index_t extruder_id)
{
    uint32_t destination_idx = find_tool_group(layer, extruder_id);
    if (destination_idx == uint32_t(-1)) {
        if (!layer.append_tool_group(extruder_id).valid())
            throw std::runtime_error("Cannot create the tool group required by a custom tool change.");
        destination_idx = layer.tool_group_count() - 1;
    }

    /* Transfer only roots sourced from normal object layers. Re-fetch views on
       every operation because vector edits may invalidate earlier child views. */
    for (uint32_t source_idx = 0; source_idx < layer.tool_group_count(); ++source_idx) {
        if (source_idx == destination_idx)
            continue;
        uint32_t extrusion_idx = 0;
        while (extrusion_idx < layer.tool_group(source_idx).extrusion_count()) {
            const PrintingExtrusion extrusion = layer.tool_group(source_idx).extrusion(extrusion_idx);
            if (!printing_extrusion_is_object(plan, group, extrusion)) {
                ++extrusion_idx;
                continue;
            }
            PrintingToolGroup source = layer.tool_group(source_idx);
            PrintingToolGroup destination = layer.tool_group(destination_idx);
            if (!source.transfer_extrusion_to(extrusion_idx, destination))
                throw std::runtime_error("Cannot transfer an object extrusion for a custom tool change.");
        }
    }

    /* Empty source visits no longer carry work. Remove them backwards so the
       destination index does not need to be tracked across each erase. */
    for (uint32_t idx = layer.tool_group_count(); idx > 0; --idx) {
        const uint32_t tool_idx = idx - 1;
        if (layer.tool_group(tool_idx).extrusion_count() == 0 &&
            !layer.tool_group(tool_idx).events().has_before() &&
            !layer.tool_group(tool_idx).events().has_after())
            layer.remove_empty_tool_group(tool_idx);
    }
}

void apply_tool_overrides(const CustomGCodeTable &table,
                           PrintingPlan &plan,
                          Slic3r::CustomGCode::Mode print_mode,
                          uint32_t extruder_count)
{
    const Slic3r::CustomGCode::Mode record_mode = effective_record_mode(table);
    if (print_mode != Slic3r::CustomGCode::MultiAsSingle ||
        record_mode != Slic3r::CustomGCode::MultiAsSingle)
        return;

    std::map<coord_t, extruder_index_t> changes;
    for (const CustomGCodeRow &row : table.rows)
        if (row.type == Slic3r::CustomGCode::ToolChange)
            changes[scale_to_layer_coord(row.print_z_mm)] = custom_gcode_row_extruder_index(row, extruder_count);
    for (uint32_t group_idx = 0; group_idx < plan.group_count(); ++group_idx) {
        PrintingGroup group = plan.group(group_idx);
        const std::optional<extruder_index_t> initial_tool = first_object_tool(plan, group);
        for (uint32_t layer_idx = 0; layer_idx < group.layer_group_count(); ++layer_idx) {
            std::optional<extruder_index_t> selected_tool = initial_tool;
            const std::map<coord_t, extruder_index_t>::const_iterator next = changes.upper_bound(group.layer_group(layer_idx).print_z());
            if (next != changes.begin())
                selected_tool = std::prev(next)->second;
            if (selected_tool) {
                PrintingLayerGroup layer = group.layer_group(layer_idx);
                move_object_extrusions_to_tool(plan, group, layer, *selected_tool);
            }
        }
    }
}

void apply_color_event_tools(const CustomGCodeTable &table,
                              PrintingPlan &plan,
                             Slic3r::CustomGCode::Mode print_mode,
                             uint32_t extruder_count)
{
    const CustomGCodeColorTargets targets = color_event_targets(table, plan, print_mode, extruder_count);

    for (uint32_t group_idx = 0; group_idx < plan.group_count(); ++group_idx) {
        PrintingGroup group = plan.group(group_idx);
        for (uint32_t layer_idx = 0; layer_idx < group.layer_group_count(); ++layer_idx) {
            if (!targets[group_idx][layer_idx])
                continue;
            PrintingLayerGroup layer = group.layer_group(layer_idx);
            uint32_t tool_idx = find_tool_group(layer, *targets[group_idx][layer_idx]);
            if (tool_idx == uint32_t(-1)) {
                if (!layer.append_tool_group(*targets[group_idx][layer_idx]).valid())
                    throw std::runtime_error("Cannot create the tool visit required by a color change.");
                tool_idx = layer.tool_group_count() - 1;
            }
            if (tool_idx != 0 && !layer.move_tool_group(tool_idx, 0))
                throw std::runtime_error("Cannot place the color-change tool first on its layer.");
        }
    }
}

class CustomGCodeToolOverrides final : public PluginBase
{
public:
    static CustomGCodeToolOverrides &instance(orchestrator_handle *orchestrator)
    {
        static CustomGCodeToolOverrides plugin(orchestrator);
        return plugin;
    }
    explicit CustomGCodeToolOverrides(orchestrator_handle *orchestrator) : PluginBase(orchestrator) {}

private:
    const char *id_impl() const noexcept override { return "ordering.custom_gcode_tool_overrides"; }
    const char *name_impl() const noexcept override { return "Custom height tool overrides"; }
    const char *description_impl() const noexcept override {
        return "Moves normal object extrusions according to MultiAsSingle tool-change markers.";
    }
    slicing_step_t step_impl() const noexcept override { return STEP_ORDERING; }
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }
    int32_t priority_impl() const noexcept override { return 50; }
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override {
        if (keys != nullptr)
            keys[0] = raw_used_config_key{"nozzle_diameter", RAW_CO_VECTOR_FLOAT,
                                          RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE};
        return 1;
    }
    void run_impl(const plugin_run_context *run_context) const override {
        const run_ctx_extrusion_ordering *context = plugin_ctx_as_extrusion_ordering(run_context);
        assert(context != nullptr && context->print != nullptr && context->plan != nullptr);
        if (context == nullptr || context->print == nullptr || context->plan == nullptr)
            return;
        const Print print(context->print);
        PrintingPlan plan(context->plan);
        const uint32_t extruder_count = print.config().get("nozzle_diameter").size();
        const std::optional<CustomGCodeTable> table = read_custom_gcode_per_print_z(print);
        if (table)
            apply_tool_overrides(*table, plan, printing_plan_mode(print, plan), extruder_count);
    }
};

class CustomGCodeEventTools final : public PluginBase
{
public:
    static CustomGCodeEventTools &instance(orchestrator_handle *orchestrator)
    {
        static CustomGCodeEventTools plugin(orchestrator);
        return plugin;
    }
    explicit CustomGCodeEventTools(orchestrator_handle *orchestrator) : PluginBase(orchestrator) {}

private:
    const char *id_impl() const noexcept override { return "ordering.custom_gcode_event_tools"; }
    const char *name_impl() const noexcept override { return "Custom height event tools"; }
    const char *description_impl() const noexcept override {
        return "Places the tool required by a color-change marker first on its layer.";
    }
    slicing_step_t step_impl() const noexcept override { return STEP_ORDERING; }
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }
    int32_t priority_impl() const noexcept override { return 150; }
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override {
        if (keys != nullptr)
            keys[0] = raw_used_config_key{"nozzle_diameter", RAW_CO_VECTOR_FLOAT,
                                          RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE};
        return 1;
    }
    void run_impl(const plugin_run_context *run_context) const override {
        const run_ctx_extrusion_ordering *context = plugin_ctx_as_extrusion_ordering(run_context);
        assert(context != nullptr && context->print != nullptr && context->plan != nullptr);
        if (context == nullptr || context->print == nullptr || context->plan == nullptr)
            return;
        const Print print(context->print);
        PrintingPlan plan(context->plan);
        const uint32_t extruder_count = print.config().get("nozzle_diameter").size();
        const std::optional<CustomGCodeTable> table = read_custom_gcode_per_print_z(print);
        if (table)
            apply_color_event_tools(*table, plan, printing_plan_mode(print, plan), extruder_count);
    }
};

} // namespace

void register_custom_gcode_per_print_z_ordering_plugins(orchestrator_handle *orchestrator)
{
    orchestrator_register_plugin(orchestrator, CustomGCodeToolOverrides::instance(orchestrator).c_instance());
    orchestrator_register_plugin(orchestrator, CustomGCodeEventTools::instance(orchestrator).c_instance());
}

} // namespace slic3r_api::GCodeGeneration::CustomGCodePerPrintZPlugin
