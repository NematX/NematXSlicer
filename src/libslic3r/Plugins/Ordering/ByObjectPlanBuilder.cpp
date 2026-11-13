///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "DefaultOrdering.hpp"
#include "IslandStacks.hpp"
#include "PlanBuilderCommon.hpp"

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <vector>

#include "libslic3r/Api/plugin/c/steps/slic3r_step_ordering.h"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Api/plugin/cpp/GuiRuleHelpers.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"

/*
Provides the By object plan builder to the ordering chain. Borrowed print layers
are cloned through PlanBuilderCommon into owned groups per physical instance.
With a positive height step, the first object layer is printed for every instance,
then each instance contributes a group for each nonempty Z interval. The interval
height is capped by the printer clearance height. Auxiliary layers at or below
the first object layer belong to its first group; others follow their Z interval.
With a positive step and height limit, intervals stop at that limit. Remaining
layers from all instances share a final group in increasing Z, using the same
construction as By layer. Layer zero and its auxiliaries always stay in the first
pass, even above the limit. With island scheduling, object-wide layers form stacks
once per object, then each stack is cloned for every physical instance before
the next stack starts. Support islands join the same clearance and lower-island
searches as object islands.
Instance-owned auxiliaries do not change the shared stacks and are attached by Z.
When no stack exists below the height limit, they join their instance's first pass.
Global auxiliary geometry below the height limit precedes the instance groups;
geometry at or above the limit joins the final group. Without a limit, all global
auxiliary geometry precedes the instance groups. Local auxiliary layers
marked with an owner are included only for that instance; unmarked local layers
are reproduced for every copy. Sources remain unchanged.
The provider owns the object order, height step, height limit and island settings.
Their activation rules keep them editable only for the selected By object mode.

    register_by_object_plan_builder_plugin() -> register the alternative provider
        ByObjectPlanBuilder::inilialize_impl() -> define settings and activation rules
        ByObjectPlanBuilder::run_impl() -> replace the plan
            build_plan_by_object() -> order instances and clone their layers
                append_print_auxiliary_group() -> retain global auxiliaries
                append_layer_instance_extrusions() -> copy and shift geometry
                sort_layer_groups_by_print_z() -> order each instance's layers
                append_by_layer_object_layers() -> merge the remaining instance layers
                append_island_groups() -> clone object-wide stacks for every instance
                    IslandStacks::separate_islands() -> group and follow source islands
*/
namespace slic3r_api { namespace Ordering { namespace DefaultOrderingPlugin {

// Register the permanent borrowed provider; rejected registration or allocation failures propagate.
void register_by_object_plan_builder_plugin(orchestrator_handle *orch);

namespace {
using namespace PlanBuilderCommon;
const char *k_no_dependencies[] = { nullptr };
const char *k_defined_config_keys[] = {
    "complete_objects_sort", "complete_objects_step", "complete_objects_step_max_z", "complete_islands"
};

// Publish copied option definitions and activation rules to the borrowed host; registration failures follow the host API.
void initialize_by_object_settings(orchestrator_handle *orch);

// Replace the plan with cloned instance groups, selecting layer zero first and then
// half-open Z intervals when the step is positive. A positive height limit caps
// intervals and merges the remainder by Z. Invalid auxiliary owners, cancellation
// and allocation failures propagate; the borrowed print is unchanged.
void build_plan_by_object(const Print &print, PrintingPlan &plan, const plugin_run_context *run_ctx);

// Build stacks once per object from non-first layers below the optional limit,
// then clone each stack for all instances before advancing to the next stack.
// Owned auxiliaries follow stack Z ranges, or the instance's first pass when
// no stack exists. They never change the shared search. Source views remain borrowed;
// cancellation, invalid owners and allocation failures propagate.
void append_island_groups(const Print &print, PrintingPlan &plan,
    const std::vector<SourceInstance> &instances, coord_t height_step_range,
    coord_t object_step_max_z, const plugin_run_context *run_ctx);

void build_plan_by_object(const Print &print, PrintingPlan &plan, const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    /* Do not clear a usable plan after cancellation has already been requested. */
    plan.clear();
    throw_if_cancelled(run_ctx);
    const std::vector<SourceInstance> instances =
        ordered_source_instances_for_object_plan(print, run_ctx);

    const double object_step = print.config().float_or_default("complete_objects_step", 0.);
    const coord_t height_step_range = object_step > 0. ? scale_to_layer_coord(std::min(
        object_step, print.config().float_or_default("extruder_clearance_height", 0.))) : 0;
    // Validation rejects positive steps whose capped height scales to zero.
    const bool with_step = height_step_range > 0;
    const bool with_islands = with_step && print.config().bool_or_default("complete_islands", false);
    const double max_z = print.config().float_or_default("complete_objects_step_max_z", 0.);
    const coord_t object_step_max_z = with_step && max_z > 0. ? scale_to_layer_coord(max_z) : 0;
    LayerExtrusionFilter lower_filter;
    if (object_step_max_z > 0)
        lower_filter.print_z_interval = std::make_pair(std::numeric_limits<coord_t>::lowest(), object_step_max_z);
    append_print_auxiliary_group(print, plan, run_ctx, lower_filter);
    bool first_layers = true;
    coord_t z_start = 0, z_end = height_step_range;
    bool is_layers = true;
    while (is_layers && (first_layers || object_step_max_z == 0 || z_start + EPSILON < object_step_max_z)) {
        if (with_islands && !first_layers) {
            append_island_groups(print, plan, instances, height_step_range, object_step_max_z, run_ctx);
            break;
        }
        if (object_step_max_z > 0)
            z_end = std::min(z_end, object_step_max_z);
        is_layers = false;
        for (const SourceInstance &source : instances) {
            throw_if_cancelled(run_ctx);
            const coord_t first_layer_z = source.object.layer_count() == 0 ? 0 : source.object.layer(0).print_z();
            std::vector<Layer> layers_to_print_range;
            for (uint32_t layer_idx = 0; layer_idx < source.object.layer_count(); ++layer_idx) {
                const Layer layer = source.object.layer(layer_idx);
                if (with_step) {
                    // PrintObjectSlice.cpp assigns object layer ids from their sorted list index.
                    // if first_layer then only id==0, else only id != 0
                    if (first_layers != (layer_idx == 0))
                        continue;
                    if (!first_layers && layer.print_z() >= z_end)
                        is_layers = true;
                    // keep only layers that are in our current printing range
                    if (!first_layers && (layer.print_z() < z_start || layer.print_z() >= z_end))
                        continue;
                }
                layers_to_print_range.push_back(layer);
            }
            // Auxiliary ids describe insertion order, not their sorted Z position.
            // Their first-layer geometry travels with layer zero; the rest follows Z.
            for (uint32_t layer_idx = 0; layer_idx < source.object.auxiliary_layer_count(); ++layer_idx) {
                const Layer layer = source.object.auxiliary_layer(layer_idx);
                const std::optional<uint32_t> owner = layer_instance_owner(layer);
                if (owner && *owner >= source.object.instance_count())
                    throw std::out_of_range("Auxiliary layer owner instance index is outside the Object.");
                if (owner && *owner != source.object_instance_idx)
                    continue;
                if (with_step) {
                    if (first_layers != (layer.print_z() <= first_layer_z))
                        continue;
                    if (!first_layers && layer.print_z() >= z_end)
                        is_layers = true;
                    if (!first_layers && (layer.print_z() < z_start || layer.print_z() >= z_end))
                        continue;
                }
                layers_to_print_range.push_back(layer);
            }
            if (with_step && layers_to_print_range.empty())
                continue;
            /* Each physical copy has a group that later stages consume independently. */
            throw_if_cancelled(run_ctx);
            PrintingGroup group = plan.append_group();
            if (!group.valid())
                continue;

            const uint32_t printing_catalog_id =
                plan.find_or_add_object_instance(source.object, source.object_instance_idx);
            group.append_printing_catalog_id(plan, printing_catalog_id);
            std::map<coord_t, uint32_t> layer_index_by_print_z;

            /* Shift object-local geometry into this physical instance's position. */
            LayerExtrusionFilter filter;
            if (with_step && !first_layers)
                filter.print_z_interval = std::make_pair(z_start, z_end);
            for (const Layer &layer : layers_to_print_range) {
                throw_if_cancelled(run_ctx);
                PrintingLayerGroup layer_group =
                    layer_group_for_print_z(
                        group, layer_index_by_print_z, layer.print_z(), run_ctx);
                append_layer_once(layer_group, layer, run_ctx);
                append_layer_instance_extrusions(
                    plan, layer_group, layer, source.shift, printing_catalog_id, run_ctx, filter);
            }

            sort_layer_groups_by_print_z(group, run_ctx);
        }
        if (!with_step)
            break;
        if (first_layers) {
            first_layers = false;
            is_layers = true;
        } else {
            z_start = z_end;
            z_end += height_step_range;
        }
    }
    if (object_step_max_z > 0) {
        // The first pass is unconditional; do not clone it again when the limit is below it.
        bool has_remaining = false;
        for (const SourceInstance &source : instances) {
            const coord_t first_z = source.object.layer_count() == 0 ? 0 : source.object.layer(0).print_z();
            for (uint32_t idx = 1; idx < source.object.layer_count(); ++idx)
                has_remaining |= source.object.layer(idx).print_z() >= object_step_max_z;
            for (uint32_t idx = 0; idx < source.object.auxiliary_layer_count(); ++idx) {
                const coord_t z = source.object.auxiliary_layer(idx).print_z();
                has_remaining |= z > first_z && z >= object_step_max_z;
            }
        }
        const Object auxiliary = print.auxiliary_object();
        for (uint32_t idx = 0; idx < auxiliary.auxiliary_layer_count(); ++idx)
            has_remaining |= auxiliary.auxiliary_layer(idx).print_z() >= object_step_max_z;
        if (has_remaining) {
            PrintingGroup group = plan.append_group();
            if (!group.valid())
                return;
            std::map<coord_t, uint32_t> indices;
            LayerExtrusionFilter filter;
            filter.print_z_interval = std::make_pair(object_step_max_z, std::numeric_limits<coord_t>::max());
            append_by_layer_object_layers(print, plan, group, indices, run_ctx, filter, true);
            append_print_auxiliary_layers_to_group(print, plan, group, indices, run_ctx, filter);
            sort_layer_groups_by_print_z(group, run_ctx);
        }
    }
}

void append_island_groups(const Print &print, PrintingPlan &plan,
    const std::vector<SourceInstance> &instances, coord_t height_step_range,
    coord_t object_step_max_z, const plugin_run_context *run_ctx)
{
    using IslandStacks::ObjectLayerToPrint;
    using IslandStacks::ObjectsLayerToPrint;
    std::vector<GraphData> extruder_clearance;
    const ConfigOption clearance = print.config().get("extruder_clearance");
    for (uint32_t idx = 0; idx < clearance.size(); ++idx)
        extruder_clearance.push_back(clearance.graph(idx).value());
    std::set<const object_handle *> separate_islands_done;
    for (const SourceInstance &first_source : instances) {
        throw_if_cancelled(run_ctx);
        if (!separate_islands_done.insert(first_source.object.handle()).second)
            continue;
        const Object object = first_source.object;
        const coord_t first_z = object.layer_count() == 0 ? 0 : object.layer(0).print_z();
        std::vector<Layer> object_layers, auxiliary_layers;
        for (uint32_t idx = 1; idx < object.layer_count(); ++idx) {
            const Layer layer = object.layer(idx);
            if (object_step_max_z == 0 || layer.print_z() < object_step_max_z)
                object_layers.push_back(layer);
        }
        for (uint32_t idx = 0; idx < object.auxiliary_layer_count(); ++idx) {
            const Layer layer = object.auxiliary_layer(idx);
            const std::optional<uint32_t> owner = layer_instance_owner(layer);
            if (owner && *owner >= object.instance_count())
                throw std::out_of_range("Auxiliary layer owner instance index is outside the Object.");
            if (!owner && layer.print_z() > first_z &&
                (object_step_max_z == 0 || layer.print_z() < object_step_max_z))
                auxiliary_layers.push_back(layer);
        }
        // Pair object and support layers at equal Z, retaining the source order.
        ObjectsLayerToPrint object_and_support_layers;
        size_t object_idx = 0, auxiliary_idx = 0;
        while (object_idx < object_layers.size() || auxiliary_idx < auxiliary_layers.size()) {
            ObjectLayerToPrint layer;
            if (object_idx < object_layers.size() && (auxiliary_idx == auxiliary_layers.size() ||
                    object_layers[object_idx].print_z() <= auxiliary_layers[auxiliary_idx].print_z()))
                layer.object_layer = object_layers[object_idx++];
            if (auxiliary_idx < auxiliary_layers.size() && (!layer.object_layer.valid() ||
                    auxiliary_layers[auxiliary_idx].print_z() == layer.object_layer.print_z()))
                layer.auxiliary_layer = auxiliary_layers[auxiliary_idx++];
            object_and_support_layers.push_back(std::move(layer));
        }
        // No writer exists at ordering time, so no current tool is known.
        const std::vector<ObjectsLayerToPrint> stacks = IslandStacks::separate_islands(
            object_and_support_layers, 0, height_step_range, extruder_clearance,
            EXTRUDER_ID_UNKNOWN, run_ctx->plugin_storage);
        if (stacks.empty()) {
            // Without a stack, higher owned auxiliaries join the copy's first pass.
            for (const SourceInstance &source : instances) {
                if (source.object.handle() != object.handle())
                    continue;
                throw_if_cancelled(run_ctx);
                const uint32_t catalog_id = plan.find_or_add_object_instance(object, source.object_instance_idx);
                for (uint32_t idx = 0; idx < plan.group_count(); ++idx) {
                    PrintingGroup group = plan.group(idx);
                    if (group.object_instance_count() != 1 || group.printing_catalog_id(0) != catalog_id)
                        continue;
                    std::map<coord_t, uint32_t> indices;
                    for (uint32_t layer_idx = 0; layer_idx < group.layer_group_count(); ++layer_idx)
                        indices.emplace(group.layer_group(layer_idx).print_z(), layer_idx);
                    for (uint32_t layer_idx = 0; layer_idx < object.auxiliary_layer_count(); ++layer_idx) {
                        const Layer layer = object.auxiliary_layer(layer_idx);
                        const std::optional<uint32_t> owner = layer_instance_owner(layer);
                        if (!owner || *owner != source.object_instance_idx || layer.print_z() <= first_z ||
                            (object_step_max_z > 0 && layer.print_z() >= object_step_max_z))
                            continue;
                        PrintingLayerGroup layers = layer_group_for_print_z(group, indices, layer.print_z(), run_ctx);
                        append_layer_once(layers, layer, run_ctx);
                        append_layer_instance_extrusions(plan, layers, layer, source.shift, catalog_id, run_ctx);
                    }
                    sort_layer_groups_by_print_z(group, run_ctx);
                    break;
                }
            }
            continue;
        }
        std::vector<const SourceInstance *> object_instances;
        std::vector<std::vector<std::vector<Layer>>> owned_layers(object.instance_count(),
            std::vector<std::vector<Layer>>(stacks.size()));
        for (const SourceInstance &source : instances) {
            if (source.object.handle() != object.handle())
                continue;
            object_instances.push_back(&source);
            // Owned auxiliaries never affect shared stacks. In print order choose
            // the first range containing Z, else the first starting above Z,
            // else the last stack. Layer-zero auxiliaries remain in the first pass.
            for (uint32_t idx = 0; idx < object.auxiliary_layer_count(); ++idx) {
                const Layer layer = object.auxiliary_layer(idx);
                const std::optional<uint32_t> owner = layer_instance_owner(layer);
                if (!owner || *owner != source.object_instance_idx || layer.print_z() <= first_z ||
                    (object_step_max_z > 0 && layer.print_z() >= object_step_max_z))
                    continue;
                size_t selected = stacks.size();
                for (size_t stack_idx = 0; stack_idx < stacks.size(); ++stack_idx)
                    if (layer.print_z() >= stacks[stack_idx].front().layer().print_z() &&
                        layer.print_z() <= stacks[stack_idx].back().layer().print_z()) {
                        selected = stack_idx;
                        break;
                    }
                if (selected == stacks.size()) {
                    for (size_t stack_idx = 0; stack_idx < stacks.size(); ++stack_idx)
                        if (stacks[stack_idx].front().layer().print_z() > layer.print_z()) {
                            selected = stack_idx;
                            break;
                        }
                }
                if (selected == stacks.size())
                    selected = stacks.size() - 1;
                owned_layers[source.object_instance_idx][selected].push_back(layer);
            }
        }
        // Complete each stack for all copies before any copy advances to the next one.
        for (size_t stack_idx = 0; stack_idx < stacks.size(); ++stack_idx) {
            for (const SourceInstance *source_ptr : object_instances) {
                const SourceInstance &source = *source_ptr;
                throw_if_cancelled(run_ctx);
                const uint32_t catalog_id = plan.find_or_add_object_instance(object, source.object_instance_idx);
                PrintingGroup group = plan.append_group();
                if (!group.valid())
                    continue;
                group.append_printing_catalog_id(plan, catalog_id);
                std::map<coord_t, uint32_t> indices;
                for (const ObjectLayerToPrint &selection : stacks[stack_idx]) {
                    for (const Layer &layer : {selection.object_layer, selection.auxiliary_layer}) {
                        if (!layer.valid())
                            continue;
                        std::vector<uint32_t> island_indices;
                        for (uint32_t idx = 0; idx < layer.island_count(); ++idx)
                            if (selection.islands.count(layer.island(idx).handle()) != 0)
                                island_indices.push_back(idx);
                        LayerExtrusionFilter filter;
                        filter.island_indices = &island_indices;
                        PrintingLayerGroup layers = layer_group_for_print_z(group, indices, layer.print_z(), run_ctx);
                        append_layer_once(layers, layer, run_ctx);
                        append_layer_instance_extrusions(plan, layers, layer, source.shift, catalog_id, run_ctx, filter);
                    }
                }
                for (const Layer &layer : owned_layers[source.object_instance_idx][stack_idx]) {
                    PrintingLayerGroup layers = layer_group_for_print_z(group, indices, layer.print_z(), run_ctx);
                    append_layer_once(layers, layer, run_ctx);
                    append_layer_instance_extrusions(plan, layers, layer, source.shift, catalog_id, run_ctx);
                }
                sort_layer_groups_by_print_z(group, run_ctx);
            }
        }
    }
}

void initialize_by_object_settings(orchestrator_handle *orch)
{
    const key_value_string_pair_t order_values[] = {
        { "object", "Right panel" }, { "lowy", "lowest Y" },
        { "lowz", "lowest Z" }, { "nearest", "Nearest" }
    };
    raw_config_option_def def = raw_config_option_def_init();
    def.opt_key = "complete_objects_sort";
    def.type = RAW_CO_ENUM;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_ORDERING;
    def.category = RAW_OPTION_CATEGORY_OUTPUT;
    def.mode = RAW_CONFIG_OPTION_MODE_ADV_EXP | RAW_CONFIG_OPTION_MODE_SUSI;
    def.precision = 6;
    def.label = "Object sort";
    def.tooltip = "When printing multiple objects or copies on after another, this will help you to choose how it's ordered."
        "\nObject will sort them by the order of the right panel."
        "\nLowest Y will sort them by their lowest Y point. Useful for printers with a X-bar."
        "\nLowest Z will sort them by their height, useful for delta printers."
        "\nNearest will try to jump to the nearest.";
    def.enum_def.value_label_pairs = { order_values, 4 };
    def.default_serialized_value = "object";
    orchestrator_create_option_def(orch, &def);

    const char *step_alias = "parallel_objects_step";
    def = raw_config_option_def_init();
    def.opt_key = "complete_objects_step";
    def.type = RAW_CO_FLOAT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    // The skirt reader runs before ordering and does not require this optional definition.
    def.invalidates_step = STEP_SUPPORT;
    def.category = RAW_OPTION_CATEGORY_OUTPUT;
    def.mode = RAW_CONFIG_OPTION_MODE_ADV_EXP | RAW_CONFIG_OPTION_MODE_SUSI;
    def.precision = 6;
    def.default_serialized_value = "0";
    def.aliases = { &step_alias, 1 };
    def.label = "Object step height";
    def.tooltip = "When multiple objects are present, instead of jumping from one to another at each layer"
        " the printer will continue to print the current object layers up to this height before moving to the next object."
        " (first layers will be still printed one by one)."
        "\nThe step is capped by 'extruder_clearance_height'. The 'extruder_clearance' graph describes the nozzle clearance."
        " If the step exceeds the graph height, islands cannot be separated."
        "\nIn By object mode, set to 0 to complete each object before moving to the next.";
    def.sidetext = "mm";
    orchestrator_create_option_def(orch, &def);

    const char *limit_alias = "parallel_objects_step_max_z";
    def = raw_config_option_def_init();
    def.opt_key = "complete_objects_step_max_z";
    def.type = RAW_CO_FLOAT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_ORDERING;
    def.category = RAW_OPTION_CATEGORY_OUTPUT;
    def.mode = RAW_CONFIG_OPTION_MODE_ADV_EXP | RAW_CONFIG_OPTION_MODE_SUSI;
    def.precision = 6;
    def.default_serialized_value = "0";
    def.aliases = { &limit_alias, 1 };
    def.label = "Object step max height";
    def.tooltip = "Above this height, printing switches back to layer-by-layer printing. Set to 0 for no height limit.";
    def.sidetext = "mm";
    orchestrator_create_option_def(orch, &def);

    const char *islands_alias = "parallel_islands";
    def = raw_config_option_def_init();
    def.opt_key = "complete_islands";
    def.type = RAW_CO_BOOL;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_ORDERING;
    def.category = RAW_OPTION_CATEGORY_OUTPUT;
    def.mode = RAW_CONFIG_OPTION_MODE_ADV_EXP | RAW_CONFIG_OPTION_MODE_SUSI;
    def.precision = 6;
    def.default_serialized_value = "0";
    def.aliases = { &islands_alias, 1 };
    def.label = "Print islands separately";
    def.tooltip = "When using 'complete_objects_step', consider each object island as a separate object, if far enough."
        "\nTwo islands are considered separate if they are farther than the extruder clearance.";
    orchestrator_create_option_def(orch, &def);

    const GuiCondition by_object = string_equals("print_plan_builder", "ordering.plan_builder.by_object");
    for (const char *key : { "complete_objects_sort", "complete_objects_step" })
        GuiRule::activate(key).when(by_object).scope(RAW_PRESET_TYPE_FFF_PRINT).add_to(orch);
    const GuiCondition with_step = all_of({ by_object, non_zero("complete_objects_step") });
    for (const char *key : { "complete_objects_step_max_z", "complete_islands" })
        GuiRule::activate(key).when(with_step).scope(RAW_PRESET_TYPE_FFF_PRINT).add_to(orch);
}

class ByObjectPlanBuilder : public PluginBase
{
public:
    // Borrow the permanent provider; initialization failures propagate.
    static ByObjectPlanBuilder &instance(orchestrator_handle *orch)
    {
        static ByObjectPlanBuilder s_instance(orch);
        return s_instance;
    }
    // Bind the borrowed orchestrator without modifying print data.
    explicit ByObjectPlanBuilder(orchestrator_handle *orch) : PluginBase(orch) {}

private:
    // Return permanent registration strings without changing state.
    const char *id_impl() const noexcept override { return "ordering.plan_builder.by_object"; }
    // Return the display name without effects.
    const char *name_impl() const noexcept override { return "By object"; }
    // Describe the owned group arrangement without effects.
    const char *description_impl() const noexcept override { return "Creates object instance groups for whole objects or height steps."; }
    // Join the plan builder alternatives without declaring another selector.
    const char *exclusive_group_impl() const noexcept override { return "ordering.plan_builder"; }
    // Run in ordering without effects.
    slicing_step_t step_impl() const noexcept override { return STEP_ORDERING; }
    // Return the empty dependency list borrowed for the application lifetime.
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }
    // Build before the other ordering phases.
    int32_t priority_impl() const noexcept override { return 0; }
    // Leave By layer as the preferred default.
    int32_t exclusive_group_priority_impl() const noexcept override { return 10; }
    // Report the count or copy the four owned setting names into caller-owned storage.
    int32_t defined_config_keys(const char **keys) const noexcept override
    {
        if (keys != nullptr)
            for (size_t idx = 0; idx < 4; ++idx)
                keys[idx] = k_defined_config_keys[idx];
        return 4;
    }
    // Report setting types so changes to the step cap or nozzle graph invalidate ordering too.
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override
    {
        if (keys != nullptr) {
            keys[0] = raw_used_config_key{ "complete_objects_sort", RAW_CO_ENUM, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE };
            keys[1] = raw_used_config_key{ "complete_objects_step", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE };
            keys[2] = raw_used_config_key{ "complete_objects_step_max_z", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE };
            keys[3] = raw_used_config_key{ "complete_islands", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE };
            keys[4] = raw_used_config_key{ "extruder_clearance_height", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE };
            keys[5] = raw_used_config_key{ "extruder_clearance", RAW_CO_VECTOR_GRAPH, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE };
        }
        return 6;
    }
    // Define the provider's settings and rules once without taking ownership of host storage.
    void inilialize_impl(storage_handle *) const override
    {
        initialize_by_object_settings(m_orchestrator);
    }
    // Replace the plan from borrowed ordering context; source and cancellation failures propagate.
    void run_impl(const plugin_run_context *run_ctx) const override
    {
        const run_ctx_extrusion_ordering *ctx = plugin_ctx_as_extrusion_ordering(run_ctx);
        assert(ctx != nullptr);
        assert(ctx->print != nullptr);
        assert(ctx->plan != nullptr);
        if (ctx == nullptr || ctx->print == nullptr || ctx->plan == nullptr)
            return;
        throw_if_cancelled(run_ctx);
        PrintingPlan plan(ctx->plan);
        const Print print(ctx->print);
        build_plan_by_object(print, plan, run_ctx);
    }
};
} // namespace

void register_by_object_plan_builder_plugin(orchestrator_handle *orch)
{
    orchestrator_register_plugin(orch, ByObjectPlanBuilder::instance(orch).c_instance());
}

}}} // namespace slic3r_api::Ordering::DefaultOrderingPlugin
