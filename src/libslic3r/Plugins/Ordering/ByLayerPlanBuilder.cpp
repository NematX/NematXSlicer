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
ByLayerPlanBuilder creates the initial mutable PrintingPlan for the ordering chain
from read-only Print views and PlanBuilderCommon helpers. Each printable role
bucket is cloned and shifted to its instance placement; later orderers own and
edit those clones without changing the sliced source geometry.

One PrintingGroup contains the whole print. Its layer groups are keyed by scaled
print_z and their tool groups by extruder ID; each extrusion owns a cloned role
bucket. LayerInstanceProperty restricts object-local auxiliary layers to their
owner. Unmarked layers repeat for every copy, while the global auxiliary object
already holds plate-space geometry. PlanBuilderCommon keeps role buckets in sync
with LayerRegionIsland::has_extrusion(). Construction remains replaceable through
STEP_ORDERING rather than calling native plan builders.

Representative direct calls (branches are not a timeline):

    build_plan_by_layer()
        |-- append_by_layer_object_layers() -> clone object layers at instance placements
        |       `-- append_layer_instance_extrusions()
        |               `-- layer_instance_projections() -> validate owners and choose shifts
        |-- append_print_auxiliary_layers_to_group()
        `-- sort_layer_groups_by_print_z()

*/

namespace slic3r_api { namespace Ordering { namespace DefaultOrderingPlugin {
// Register the permanent borrowed plugin and copy its alternative-selection setting description into the application.
// Print data is unchanged; allocation failures or rejected descriptions throw.
void register_by_layer_plan_builder_plugin(orchestrator_handle *orch);

namespace {

using namespace PlanBuilderCommon;

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
    append_by_layer_object_layers(print, plan, group, layer_index_by_print_z, run_ctx);

    append_print_auxiliary_layers_to_group(
        print, plan, group, layer_index_by_print_z, run_ctx);
    sort_layer_groups_by_print_z(group, run_ctx);
}

class ByLayerPlanBuilder : public PluginBase
{
public:
    // Borrow the permanent builder associated with the application orchestrator; initialization failures propagate.
    static ByLayerPlanBuilder &instance(orchestrator_handle *orch)
    {
        static ByLayerPlanBuilder s_instance(orch);
        return s_instance;
    }

    // Bind the borrowed orchestrator without changing print data.
    explicit ByLayerPlanBuilder(orchestrator_handle *orch) : PluginBase(orch) {}

private:
    // Return the permanent provider id without changing state or transferring ownership.
    const char *id_impl() const noexcept override { return "ordering.plan_builder.by_layer"; }
    // Return the permanent display name without effects or failures.
    const char *name_impl() const noexcept override { return "By layer"; }
    const char *description_impl() const noexcept override
    {
        return "Creates the initial ordering work plan from the generated layer-region-island extrusions.";
    }
    const char *exclusive_group_impl() const noexcept override { return k_group_builder; }
    slicing_step_t step_impl() const noexcept override { return STEP_ORDERING; }
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }
    int32_t priority_impl() const noexcept override { return 0; }
    // Prefer this builder by default without changing its execution order or application state.
    int32_t exclusive_group_priority_impl() const noexcept override { return 0; }

    // Replace the plan with owned by-layer geometry; cancellation and invalid source failures propagate.
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
        build_plan_by_layer(print, plan, run_ctx);
    }
};

} // namespace

void register_by_layer_plan_builder_plugin(orchestrator_handle *orch)
{
    orchestrator_register_plugin(orch, ByLayerPlanBuilder::instance(orch).c_instance());
    const plugin_instance member = ByLayerPlanBuilder::instance(orch).c_instance();
    raw_exclusive_group_def definition = raw_exclusive_group_def_init();
    definition.exclusive_group = member.vt->get_exclusive_group(member.ctx);
    definition.category = RAW_OPTION_CATEGORY_OUTPUT;
    definition.option_key = "print_plan_builder";
    definition.label = "Print sequence";
    definition.tooltip = "Chooses whether the print is built layer by layer or object by object.";
    definition.show_with_one_provider = 1;
    // Individual setting rules govern the object controls instead of a generated provider rule.
    definition.provider_settings_rules = 0;
    definition.has_mode = 1;
    definition.mode = RAW_CONFIG_OPTION_MODE_SIM_ADV_EXP | RAW_CONFIG_OPTION_MODE_PRUSA;
    definition.has_invalidates_step = 1;
    definition.invalidates_step = STEP_SKIRT_BRIM;
    definition.ui_page = "Output options";
    definition.ui_group = "Sequential printing";
    ByLayerPlanBuilder::instance(orch).register_exclusive_group(STEP_ORDERING, definition);
}

}}} // namespace slic3r_api::Ordering::DefaultOrderingPlugin
