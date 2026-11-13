///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "DefaultOrdering.hpp"
#include "PlanBuilderCommon.hpp"

#include <cassert>
#include <cstdint>
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
are cloned through PlanBuilderCommon into one owned group per physical instance.
Global auxiliary geometry precedes the instance groups; local auxiliary layers
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

// Replace the plan with instance groups owning cloned sources. Invalid auxiliary owners,
// cancellation and allocation failures propagate; the borrowed print is unchanged.
void build_plan_by_object(const Print &print, PrintingPlan &plan, const plugin_run_context *run_ctx);

void build_plan_by_object(const Print &print, PrintingPlan &plan, const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    /* Do not clear a usable plan after cancellation has already been requested. */
    plan.clear();
    throw_if_cancelled(run_ctx);
    append_print_auxiliary_group(print, plan, run_ctx);
    const std::vector<SourceInstance> instances =
        ordered_source_instances_for_object_plan(print, run_ctx);

    for (const SourceInstance &source : instances) {
        throw_if_cancelled(run_ctx);
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
        for (uint32_t layer_idx = 0; layer_idx < source.object.layer_count(); ++layer_idx) {
            throw_if_cancelled(run_ctx);
            const Layer layer = source.object.layer(layer_idx);
            PrintingLayerGroup layer_group =
                layer_group_for_print_z(
                    group, layer_index_by_print_z, layer.print_z(), run_ctx);
            append_layer_once(layer_group, layer, run_ctx);
            append_layer_instance_extrusions(
                plan, layer_group, layer, source.shift, printing_catalog_id, run_ctx);
        }

        /* Common local layers apply to every copy, marked layers only to their owner. */
        for (uint32_t layer_idx = 0; layer_idx < source.object.auxiliary_layer_count(); ++layer_idx) {
            throw_if_cancelled(run_ctx);
            const Layer layer = source.object.auxiliary_layer(layer_idx);
            const std::optional<uint32_t> owner = layer_instance_owner(layer);
            if (owner && *owner >= source.object.instance_count())
                throw std::out_of_range("Auxiliary layer owner instance index is outside the Object.");
            if (owner && *owner != source.object_instance_idx)
                continue;
            PrintingLayerGroup layer_group =
                layer_group_for_print_z(
                    group, layer_index_by_print_z, layer.print_z(), run_ctx);
            append_layer_once(layer_group, layer, run_ctx);
            append_layer_instance_extrusions(
                plan, layer_group, layer, source.shift, printing_catalog_id, run_ctx);
        }

        sort_layer_groups_by_print_z(group, run_ctx);
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
    const char *description_impl() const noexcept override { return "Creates one printing group per object instance."; }
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
    // Report owned setting types so the host can validate definitions and invalidate ordering on edits.
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override
    {
        if (keys != nullptr) {
            keys[0] = raw_used_config_key{ "complete_objects_sort", RAW_CO_ENUM, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE };
            keys[1] = raw_used_config_key{ "complete_objects_step", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE };
            keys[2] = raw_used_config_key{ "complete_objects_step_max_z", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE };
            keys[3] = raw_used_config_key{ "complete_islands", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE };
        }
        return 4;
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
