///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "DefaultOrdering.hpp"
#include "PlanBuilderCommon.hpp"

#include <cassert>
#include <cstdint>
#include <map>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/c/steps/slic3r_step_ordering.h"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"

/*
Representative direct calls (branches are not a timeline):

    build_plan_by_layer()
        |-- append_layer_instance_extrusions()
        |-- append_print_auxiliary_layers_to_group()
        `-- sort_layer_groups_by_print_z()

    build_plan_by_object()
        |-- append_print_auxiliary_group()
        |-- ordered_source_instances_for_object_plan()
        `-- append_layer_instance_extrusions()
*/

namespace slic3r_api { namespace Ordering { namespace DefaultOrderingPlugin {
// Register the permanent borrowed plugin and copy its alternative-selection setting description into the application.
// Print data is unchanged; allocation failures or rejected descriptions throw.
void register_default_plan_builder_plugin(orchestrator_handle *orch);

namespace {

using namespace PlanBuilderCommon;

/*
DefaultPlanBuilder is the first plugin in the ordering chain.

It creates the mutable PrintingPlan from the read-only Print through plugin
views and the shared PlanBuilderCommon construction helpers. The source data tree
stays untouched: each printable role bucket from a LayerRegionIsland is cloned
into the plan, and the clone is shifted
for the object instance it belongs to. Later ordering plugins can reorder those
clones without changing perimeter, infill or support generation results.

The plan built here has this shape:
  - by-layer printing: one PrintingGroup for the whole print;
  - complete-object printing: one PrintingGroup per object instance;
  - inside a group, PrintingLayerGroups are keyed by scaled print_z;
  - inside a layer group, PrintingToolGroups are keyed by extruder id;
  - each stored PrintingExtrusion owns one cloned role bucket.

Object-local auxiliary layers marked with LayerInstanceProperty are cloned
only for their owner. Unmarked layers remain common patterns for every copy;
the separate global auxiliary object already stores plate-space geometry.

    build_plan_by_layer() -> merge Z buckets, then place each local auxiliary
        layer_instance_projections() -> validate the owner and choose its shifts
    build_plan_by_object() -> form one group per instance and skip other owners

PlanBuilderCommon keeps its copied role buckets in sync with
LayerRegionIsland::has_extrusion(). Do not call native PrintingPlan builders
from this plugin: the point of this file is to make construction replaceable by
another STEP_ORDERING plugin.
*/

const char *k_no_dependencies[] = { nullptr };
const char *k_group_builder = "ordering.plan_builder";

/*
Build the normal by-layer plan: one PrintingGroup for the whole print, with all
objects contributing to the same Z buckets.
Clone borrowed geometry through validated auxiliary owner placements. The plan
owns new roots; cancellation, invalid-owner and allocation failures propagate.
*/
void build_plan_by_layer(
    const Print &print,
    PrintingPlan &plan,
    const plugin_run_context *run_ctx);

/*
Build the complete-object style plan: one PrintingGroup per object instance.
The group order is chosen from complete_objects_sort when the plugin can express
that choice from public data.
Clone borrowed local sources once in each instance group, excluding auxiliaries
owned by another copy. The plan owns roots; invalid-owner, cancellation and
allocation failures propagate.
*/
void build_plan_by_object(
    const Print &print,
    PrintingPlan &plan,
    const plugin_run_context *run_ctx);

/*
Read one bool setting from the Print config exposed to the plugin API.

Missing keys are treated as disabled. This keeps the default builder usable in
small tests that construct a Print without a fully populated preset.
*/
bool config_bool_value(const Config &config, const char *key);

/* Same helper as above, but for floating-point switches such as parallel object mode. */
double config_float_value(const Config &config, const char *key);

/*
Choose the initial plan shape.

Complete-object and parallel-object modes need one high-level PrintingGroup per
object instance so later G-code code can keep one object batch independent from
the next. Normal printing uses a single by-layer group so tools and islands can
be ordered across the whole Z.
*/
bool use_object_plan(const Print &print);

bool config_bool_value(const Config &config, const char *key)
{
    const config_option_handle *option = config_get(config.handle(), key);
    return option != nullptr && ConfigOption(option).get_bool();
}

double config_float_value(const Config &config, const char *key)
{
    const config_option_handle *option = config_get(config.handle(), key);
    return option == nullptr ? 0.0 : ConfigOption(option).get_float();
}

bool use_object_plan(const Print &print)
{
    const Config config = print.config();
    return config_bool_value(config, "complete_objects") ||
           config_float_value(config, "parallel_objects_step") > 0.0;
}

void build_plan_by_layer(
    const Print &print,
    PrintingPlan &plan,
    const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    /* Cancellation before clear prevents discarding a previously valid plan. */
    plan.clear();
    throw_if_cancelled(run_ctx);
    PrintingGroup group = plan.append_group();
    if (!group.valid())
        return;

    std::map<coord_t, uint32_t> layer_index_by_print_z;
    for (uint32_t object_idx = 0; object_idx < print.object_count(); ++object_idx) {
        throw_if_cancelled(run_ctx);
        const Object object = print.object(object_idx);

        /*
        The global by-layer group records every physical instance represented by
        the batch. Geometry is duplicated below; this list is only context for
        later ordering and debugging.
        */
        for (uint32_t object_instance_idx = 0; object_instance_idx < object.instance_count(); ++object_instance_idx) {
            throw_if_cancelled(run_ctx);
            const uint32_t printing_catalog_id =
                plan.find_or_add_object_instance(object, object_instance_idx);
            group.append_printing_catalog_id(plan, printing_catalog_id);
        }

        for (uint32_t layer_idx = 0; layer_idx < object.layer_count(); ++layer_idx) {
            throw_if_cancelled(run_ctx);
            const Layer layer = object.layer(layer_idx);
            /*
            The map merges layers from several objects when their scaled print_z
            matches. That is the normal by-layer printing contract: all geometry
            at the same Z belongs to one layer batch before tool/island ordering.
            */
            PrintingLayerGroup layer_group =
                layer_group_for_print_z(
                    group, layer_index_by_print_z, layer.print_z(), run_ctx);
            append_layer_once(layer_group, layer, run_ctx);
            /*
            The source layer exists once per object, so clone it once per
            physical instance with that instance's bed shift applied.
            */
            for (uint32_t object_instance_idx = 0; object_instance_idx < object.instance_count(); ++object_instance_idx) {
                throw_if_cancelled(run_ctx);
                const uint32_t printing_catalog_id =
                    plan.find_or_add_object_instance(object, object_instance_idx);
                append_layer_instance_extrusions(plan, layer_group,
                                                 layer,
                                                 object.instance_shift(object_instance_idx),
                                                 printing_catalog_id,
                                                 run_ctx);
            }
        }

        /*
        Real-object auxiliary layers are local. Merge them by print Z and clone
        a marked layer only for its owner, or a common layer for every copy.
        The separate global auxiliary object already stores plateau geometry.
        */
        for (uint32_t layer_idx = 0; layer_idx < object.auxiliary_layer_count(); ++layer_idx) {
            throw_if_cancelled(run_ctx);
            const Layer layer = object.auxiliary_layer(layer_idx);
            PrintingLayerGroup layer_group =
                layer_group_for_print_z(
                    group, layer_index_by_print_z, layer.print_z(), run_ctx);
            append_layer_once(layer_group, layer, run_ctx);
            for (const std::pair<uint32_t, c_point> &projection : layer_instance_projections(object, layer)) {
                throw_if_cancelled(run_ctx);
                const uint32_t printing_catalog_id =
                    plan.find_or_add_object_instance(object, projection.first);
                append_layer_instance_extrusions(plan, layer_group,
                                                 layer,
                                                 projection.second,
                                                 printing_catalog_id,
                                                 run_ctx);
            }
        }
    }

    append_print_auxiliary_layers_to_group(
        print, plan, group, layer_index_by_print_z, run_ctx);
    sort_layer_groups_by_print_z(group, run_ctx);
}

void build_plan_by_object(
    const Print &print,
    PrintingPlan &plan,
    const plugin_run_context *run_ctx)
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
        /*
        Complete-object style printing isolates each physical object copy in its
        own group. Later stages may consume one group completely before moving
        to the next group.
        */
        throw_if_cancelled(run_ctx);
        PrintingGroup group = plan.append_group();
        if (!group.valid())
            continue;

        const uint32_t printing_catalog_id =
            plan.find_or_add_object_instance(source.object, source.object_instance_idx);
        group.append_printing_catalog_id(plan, printing_catalog_id);
        std::map<coord_t, uint32_t> layer_index_by_print_z;

        /*
        A complete-object group owns only one object instance. The source layers
        still use object-local geometry, so each cloned root receives this
        instance shift during append_region_island_instance_extrusions().
        */
        for (uint32_t layer_idx = 0; layer_idx < source.object.layer_count(); ++layer_idx) {
            throw_if_cancelled(run_ctx);
            const Layer layer = source.object.layer(layer_idx);
            PrintingLayerGroup layer_group =
                layer_group_for_print_z(
                    group, layer_index_by_print_z, layer.print_z(), run_ctx);
            append_layer_once(layer_group, layer, run_ctx);
            /*
            Unlike by-layer mode, only the selected object instance is cloned
            into this group. That keeps each complete-object batch independent.
            */
            append_layer_instance_extrusions(
                plan, layer_group, layer, source.shift, printing_catalog_id, run_ctx);
        }

        /*
        Complete-object groups own one physical instance. Common local layers
        apply to each group; marked local layers apply only to their owner.
        Global plateau layers remain in their separate leading group.
        */
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

class DefaultPlanBuilder : public PluginBase
{
public:
    static DefaultPlanBuilder &instance(orchestrator_handle *orch)
    {
        static DefaultPlanBuilder s_instance(orch);
        return s_instance;
    }

    explicit DefaultPlanBuilder(orchestrator_handle *orch) : PluginBase(orch) {}

private:
    const char *id_impl() const noexcept override { return "ordering.plan_builder.default"; }
    const char *name_impl() const noexcept override { return "Default printing-plan builder"; }
    const char *description_impl() const noexcept override
    {
        return "Creates the initial ordering work plan from the generated layer-region-island extrusions.";
    }
    const char *exclusive_group_impl() const noexcept override { return k_group_builder; }
    slicing_step_t step_impl() const noexcept override { return STEP_ORDERING; }
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }
    int32_t priority_impl() const noexcept override { return 0; }

    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override
    {
        if (keys == nullptr)
            return 3;
        keys[0] = raw_used_config_key{"complete_objects", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE};
        keys[1] = raw_used_config_key{"parallel_objects_step", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE};
        keys[2] = raw_used_config_key{"complete_objects_sort", RAW_CO_ENUM, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE};
        return 3;
    }

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
        /*
        The builder is the only default ordering plugin that clears the plan.
        Later plugins must preserve the plan and only reorder or annotate it.
        */
        if (use_object_plan(print))
            build_plan_by_object(print, plan, run_ctx);
        else
            build_plan_by_layer(print, plan, run_ctx);
    }
};

} // namespace

void register_default_plan_builder_plugin(orchestrator_handle *orch)
{
    orchestrator_register_plugin(orch, DefaultPlanBuilder::instance(orch).c_instance());
    const plugin_instance member = DefaultPlanBuilder::instance(orch).c_instance();
    raw_exclusive_group_def definition = raw_exclusive_group_def_init();
    definition.exclusive_group = member.vt->get_exclusive_group(member.ctx);
    definition.category = RAW_OPTION_CATEGORY_OUTPUT;
    definition.label = "Printing-plan builder";
    definition.tooltip = "Selects how STEP_ORDERING creates the initial printing plan.";
    DefaultPlanBuilder::instance(orch).register_exclusive_group(STEP_ORDERING, definition);
}

}}} // namespace slic3r_api::Ordering::DefaultOrderingPlugin
