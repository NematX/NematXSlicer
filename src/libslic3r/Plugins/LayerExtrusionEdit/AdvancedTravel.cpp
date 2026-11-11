///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

#include "AdvancedTravel.hpp"

#include <algorithm>
#include <iterator>

#include "libslic3r/Api/plugin/c/steps/slic3r_step_layer_extrusion_edit.h"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "AvoidCrossing/CompatibilityPlanner.hpp"
#include "TravelConnectionHelpers.hpp"

/*
Advanced travel supplies XY routes between ordered extrusions to the layer
editor. Setup allocates connector summaries, and setup_run fills one summary per
layer group before the host's worker barrier. Each run first prepares a chronology:
an ordered record of borrowed extrusion gaps with their seams already snapped.
It then applies that record using a CompatibilityPlanner owned by this worker
callback. The planner checks regional avoidance settings and chooses direct paths
or perimeter detours; CompatibilityPlanner.hpp defines its routing interface.
The connector inserts travels, interpolates Z and updates flags and progress.
The plan owns all borrowed extrusions throughout both phases; no chronology or
planner survives the callback. Cancellation and routing errors propagate.

    register_advanced_travel_plugin: register the permanent provider
    setup_impl: reset and allocate connector summaries
    setup_run_impl: publish this layer group's summary
    run_impl: own the chronology and planner for one worker callback
        |-- ScopeTravelConnector::prepare: record ordered gaps and snap seams
        `-- ScopeTravelConnector::apply: insert routes and update flags
            `-- CompatibilityPlanner::path: choose straight travel or a detour
*/

namespace slic3r_api { namespace LayerExtrusionEdit { namespace AdvancedTravelPlugin {
// Register a permanent provider borrowing the orchestrator; allocation errors propagate and registration rejections are logged by the host.
void register_advanced_travel_plugin(orchestrator_handle *orchestrator);

namespace {
using namespace TravelConnection;

const char *const k_dependencies[] = {
    "layer_extrusion_edit.transition_scope.default",
    "layer_extrusion_edit.entry_state.default",
    nullptr
};

class AdvancedTravel : public PluginBase
{
public:
    // Bind the provider and connector to the borrowed orchestrator.
    explicit AdvancedTravel(orchestrator_handle *orchestrator);
private:
    // Return the static provider ID for explicit selection.
    const char *id_impl() const noexcept override;
    // Return the static display name.
    const char *name_impl() const noexcept override;
    // Return the static routing description.
    const char *description_impl() const noexcept override;
    // Return the existing travel group ID.
    const char *exclusive_group_impl() const noexcept override;
    // Return the layer editing step that inserts travel.
    slicing_step_t step_impl() const noexcept override;
    // Return the static null-terminated scope and entry-state dependencies.
    const char *const *dependencies_impl() const noexcept override;
    // Run after the default wipe (-80), before the crossing wipe (-45) and lift (-40).
    int32_t priority_impl() const noexcept override;
    // Keep avoidance and straight travel ahead of advanced for implicit selection.
    int32_t exclusive_group_priority_impl() const noexcept override;
    // Return the count, and copy static key descriptions when the caller supplies an array of that size.
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override;
    // Return the static progress format.
    const char *progress_message_format_impl() const noexcept override;
    // Reset summaries and allocate them for a valid print and plan; allocation errors propagate.
    void setup_impl(const plugin_run_context *run_ctx, uint32_t run_count) const override;
    // Fill this layer group's summary for a valid setup and layer; connector errors propagate.
    void setup_run_impl(const plugin_run_context *run_ctx) const override;
    // Prepare gaps then apply worker-local routes; invalid contexts do nothing and cancellation or errors propagate.
    void run_impl(const plugin_run_context *run_ctx) const override;

    mutable ScopeTravelConnector m_connector;
    mutable bool m_setup_valid = false;
};

AdvancedTravel::AdvancedTravel(orchestrator_handle *orchestrator) :
    PluginBase(orchestrator), m_connector(orchestrator)
{}

const char *AdvancedTravel::id_impl() const noexcept
{
    return "layer_extrusion_edit.travel.advanced";
}

const char *AdvancedTravel::name_impl() const noexcept
{
    return "Advanced travel";
}

const char *AdvancedTravel::description_impl() const noexcept
{
    return "Reroutes eligible travels that would cross a perimeter.";
}

const char *AdvancedTravel::exclusive_group_impl() const noexcept
{
    return "layer_extrusion_edit.travel";
}

slicing_step_t AdvancedTravel::step_impl() const noexcept
{
    return STEP_LAYER_EXTRUSION_EDIT;
}

const char *const *AdvancedTravel::dependencies_impl() const noexcept
{
    return k_dependencies;
}

int32_t AdvancedTravel::priority_impl() const noexcept
{
    return -50;
}

int32_t AdvancedTravel::exclusive_group_priority_impl() const noexcept
{
    return 1;
}

int32_t AdvancedTravel::used_config_keys(raw_used_config_key *keys) const noexcept
{
    static const raw_used_config_key used_keys[] = {
        {"avoid_crossing_perimeters", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"avoid_crossing_not_first_layer", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"avoid_crossing_perimeters_max_detour", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"nozzle_diameter", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"avoid_crossing_top", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"avoid_travel_island", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"avoid_travel_island_weight", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"infill_extruder", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"ironing", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"perimeter_extruder", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"perimeters", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"solid_infill_extruder", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE}
    };
    if (keys != nullptr)
        std::copy(std::begin(used_keys), std::end(used_keys), keys);
    return int32_t(sizeof(used_keys) / sizeof(used_keys[0]));
}

const char *AdvancedTravel::progress_message_format_impl() const noexcept
{
    return "Connecting advanced travel paths: %u / %u trees";
}

void AdvancedTravel::setup_impl(const plugin_run_context *run_ctx, const uint32_t run_count) const
{
    m_connector.reset();
    const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run_ctx);
    m_setup_valid = ctx != nullptr && ctx->print != nullptr && ctx->plan != nullptr;
    if (m_setup_valid)
        m_connector.setup(PrintingPlan(ctx->plan), run_count);
}

void AdvancedTravel::setup_run_impl(const plugin_run_context *run_ctx) const
{
    if (!m_setup_valid)
        return;
    const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run_ctx);
    if (ctx == nullptr || ctx->layer_group == nullptr)
        return;
    PrintingLayerGroup layer_group(ctx->layer_group);
    m_connector.setup_run(layer_group, ctx->group_idx, ctx->layer_group_idx, progress());
}

void AdvancedTravel::run_impl(const plugin_run_context *run_ctx) const
{
    if (!m_setup_valid)
        return;
    const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run_ctx);
    if (ctx == nullptr || ctx->print == nullptr || ctx->layer_group == nullptr || ctx->plan == nullptr)
        return;
    PrintingLayerGroup layer_group(ctx->layer_group);
    LayerTravelChronology chronology = m_connector.prepare(layer_group, ctx->group_idx, ctx->layer_group_idx);
    PrintingPlan plan(ctx->plan);
    AvoidCrossing::CompatibilityPlanner planner(Print(ctx->print), plan, run_ctx);
    const TravelPathPlanner route = [&planner](const TravelEndpoint &source,
                                               const TravelEndpoint &target,
                                               const extruder_index_t extruder_id) {
        return planner.path(source, target, extruder_id);
    };
    m_connector.apply(chronology, route, progress());
}
} // namespace

void register_advanced_travel_plugin(orchestrator_handle *orchestrator)
{
    static AdvancedTravel plugin(orchestrator);
    orchestrator_register_plugin(orchestrator, plugin.c_instance());
}
}}} // namespace slic3r_api::LayerExtrusionEdit::AdvancedTravelPlugin
