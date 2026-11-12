///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
Perimeter-avoiding travel generation
===================================

This provider supplies XY paths for gaps between ordered extrusion leaves in a
PrintingPlan. ScopeTravelConnector, defined in TravelConnectionHelpers.hpp,
finds those gaps and inserts travels into the reserved phases. It also handles
epsilon snapping, Z interpolation and final travel flags; this provider chooses
only the XY route, through AvoidCrossing::CompatibilityPlanner::path.

setup_impl resets the connector and allocates its per-layer summaries.
setup_run_impl prepares one layer group's summary before run_impl connects its
scopes. The host finishes all setup_run_impl callbacks before starting run_impl
callbacks, so a run can read the next non-empty layer's summary to set its
outgoing flag. Each run owns an AvoidCrossing::CompatibilityPlanner for that
layer group; CompatibilityPlanner.hpp defines its routing interface.

Travel group priorities select advanced (-2), this provider (-1) when advanced
is inactive, then straight travel (0). Explicit active provider IDs take precedence.

    setup_impl: reset and allocate connector summaries
    setup_run_impl: prepare the current layer group's summary
    run_impl: own the planner for this worker callback
    `-- ScopeTravelConnector::run: find gaps and insert the returned XY paths
        `-- CompatibilityPlanner::path: select straight travel or a detour
*/

#include "AvoidCrossingPerimetersTravel.hpp"
#include "PerimeterTravelPlanner.hpp"

#include <algorithm>
#include <cstdint>
#include <iterator>

#include "libslic3r/Api/plugin/c/steps/slic3r_step_layer_extrusion_edit.h"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "AvoidCrossing/CompatibilityPlanner.hpp"
#include "TravelConnectionHelpers.hpp"

namespace slic3r_api { namespace LayerExtrusionEdit { namespace AvoidCrossingPerimetersTravelPlugin {
// Register the permanent borrowed plugin and copy its alternative-selection setting description into the application.
// Print data is unchanged; allocation failures or rejected descriptions throw.
void register_avoid_crossing_perimeters_travel_plugin(orchestrator_handle *orchestrator);

namespace {

using namespace TravelConnection;
namespace AC = AvoidCrossing;

// Publish the router inputs read by both route generation and retraction planning.
int32_t perimeter_travel_config_keys(raw_used_config_key *keys) noexcept;

const char *const k_dependencies[] = {
    "layer_extrusion_edit.transition_scope.default",
    "layer_extrusion_edit.entry_state.default",
    nullptr
};

class AvoidCrossingPerimetersTravel : public PluginBase
{
public:
    // Return the permanent provider instance borrowing the first orchestrator supplied.
    static AvoidCrossingPerimetersTravel &instance(orchestrator_handle *orchestrator);
    // Bind the provider and its connector to the borrowed orchestrator.
    explicit AvoidCrossingPerimetersTravel(orchestrator_handle *orchestrator);

private:
    // Return the static provider ID used by the travel selector.
    const char *id_impl() const noexcept override;
    // Return the static display name for the provider.
    const char *name_impl() const noexcept override;
    // Return the static description of perimeter-avoiding travel.
    const char *description_impl() const noexcept override;
    // Return the shared travel-generation group ID.
    const char *exclusive_group_impl() const noexcept override;
    // Select the layer-extrusion-edit step that owns travel insertion.
    slicing_step_t step_impl() const noexcept override;
    // Return the static null-terminated list of required scope and entry-state providers.
    const char *const *dependencies_impl() const noexcept override;
    // Run after the entry-state, retraction and wipe providers and before lift insertion.
    int32_t priority_impl() const noexcept override;
    // Rank after advanced (-2) and before straight travel (0); explicit active provider IDs take precedence.
    int32_t exclusive_group_priority_impl() const noexcept override;
    // Report key types for host validation and GUI activation. Call with null to get the count,
    // then supply a caller-owned array of that size; copied entries borrow static key strings.
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override;
    // Return the static progress format for connected layer trees.
    const char *progress_message_format_impl() const noexcept override;
    // Reset connector state and allocate summaries when the run has a print and plan.
    void setup_impl(const plugin_run_context *run_ctx, uint32_t run_count) const override;
    // Publish this layer group summary when setup and the layer context are valid.
    void setup_run_impl(const plugin_run_context *run_ctx) const override;
    // Connect this layer group through a worker-local planner; invalid contexts do nothing and cancellation or errors propagate.
    void run_impl(const plugin_run_context *run_ctx) const override;

    mutable ScopeTravelConnector m_connector;
    mutable bool m_setup_valid = false;
};

AvoidCrossingPerimetersTravel &AvoidCrossingPerimetersTravel::instance(orchestrator_handle *orchestrator)
{
    static AvoidCrossingPerimetersTravel plugin(orchestrator);
    return plugin;
}

AvoidCrossingPerimetersTravel::AvoidCrossingPerimetersTravel(orchestrator_handle *orchestrator) :
    PluginBase(orchestrator),
    m_connector(orchestrator)
{}

const char *AvoidCrossingPerimetersTravel::id_impl() const noexcept
{
    return "layer_extrusion_edit.travel.avoid_crossing_perimeters";
}

const char *AvoidCrossingPerimetersTravel::name_impl() const noexcept
{
    return "Avoid crossing perimeters";
}

const char *AvoidCrossingPerimetersTravel::description_impl() const noexcept
{
    return "Reroutes eligible travels that would cross a perimeter.";
}

const char *AvoidCrossingPerimetersTravel::exclusive_group_impl() const noexcept
{
    return "layer_extrusion_edit.travel";
}

slicing_step_t AvoidCrossingPerimetersTravel::step_impl() const noexcept
{
    return STEP_LAYER_EXTRUSION_EDIT;
}

const char *const *AvoidCrossingPerimetersTravel::dependencies_impl() const noexcept
{
    return k_dependencies;
}

int32_t AvoidCrossingPerimetersTravel::priority_impl() const noexcept
{
    return -50;
}

int32_t AvoidCrossingPerimetersTravel::exclusive_group_priority_impl() const noexcept
{
    return -1;
}

int32_t AvoidCrossingPerimetersTravel::used_config_keys(raw_used_config_key *keys) const noexcept
{
    return perimeter_travel_config_keys(keys);
}

int32_t perimeter_travel_config_keys(raw_used_config_key *keys) noexcept
{
    static const raw_used_config_key used_keys[] = {
        {"complete_objects", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"parallel_objects_step", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
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

const char *AvoidCrossingPerimetersTravel::progress_message_format_impl() const noexcept
{
    return "Connecting perimeter-avoiding travel paths: %u / %u trees";
}

void AvoidCrossingPerimetersTravel::setup_impl(
    const plugin_run_context *run_ctx, const uint32_t run_count) const
{
    m_connector.reset();
    const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run_ctx);
    m_setup_valid = ctx != nullptr && ctx->print != nullptr && ctx->plan != nullptr;
    if (m_setup_valid)
        m_connector.setup(PrintingPlan(ctx->plan), run_count);
}

void AvoidCrossingPerimetersTravel::setup_run_impl(const plugin_run_context *run_ctx) const
{
    if (!m_setup_valid)
        return;
    const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run_ctx);
    if (ctx == nullptr || ctx->layer_group == nullptr)
        return;
    PrintingLayerGroup layer_group(ctx->layer_group);
    m_connector.setup_run(layer_group,
                          ctx->group_idx, ctx->layer_group_idx, progress());
}

void AvoidCrossingPerimetersTravel::run_impl(const plugin_run_context *run_ctx) const
{
    if (!m_setup_valid)
        return;
    const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run_ctx);
    if (ctx == nullptr || ctx->print == nullptr || ctx->layer_group == nullptr)
        return;

    if (ctx->plan == nullptr)
        return;
    PrintingPlan plan(ctx->plan);
    AC::CompatibilityPlanner planner(Print(ctx->print), plan, run_ctx);
    const TravelPathPlanner route = [&planner](const TravelEndpoint &source,
                                               const TravelEndpoint &target,
                                               const extruder_index_t extruder_id) {
        return planner.path(source, target, extruder_id);
    };
    PrintingLayerGroup layer_group(ctx->layer_group);
    m_connector.run(layer_group,
                    ctx->group_idx, ctx->layer_group_idx,
                    route, progress());
}

} // namespace

// Keep the provider's planner private while sharing its worker caches.
struct PerimeterTravelPlanner::Impl
{
    AC::CompatibilityPlanner planner;
    // Keep the planner alive for all queries from one layer worker.
    Impl(const Print &print, const PrintingPlan &plan, const plugin_run_context *ctx) : planner(print, plan, ctx) {}
};

PerimeterTravelPlanner::PerimeterTravelPlanner(const Print &print, const PrintingPlan &plan,
    const plugin_run_context *ctx) : m_impl(std::make_unique<Impl>(print, plan, ctx)) {}
PerimeterTravelPlanner::~PerimeterTravelPlanner() = default;

int32_t PerimeterTravelPlanner::used_config_keys(raw_used_config_key *keys) noexcept
{
    return perimeter_travel_config_keys(keys);
}

std::vector<c_point> PerimeterTravelPlanner::path(const TravelEndpoint &source,
    const TravelEndpoint &target, extruder_index_t tool)
{
    return m_impl->planner.path(source, target, tool);
}

bool PerimeterTravelPlanner::crosses(const TravelEndpoint &source, const TravelEndpoint &target,
    extruder_index_t tool, const std::vector<c_point> &path, bool exclude_top_surfaces)
{
    return m_impl->planner.crosses(source, target, tool, path, exclude_top_surfaces);
}

void register_avoid_crossing_perimeters_travel_plugin(orchestrator_handle *orchestrator)
{
    orchestrator_register_plugin(
        orchestrator, AvoidCrossingPerimetersTravel::instance(orchestrator).c_instance());
    const plugin_instance member = AvoidCrossingPerimetersTravel::instance(orchestrator).c_instance();
    raw_exclusive_group_def definition = raw_exclusive_group_def_init();
    definition.exclusive_group = member.vt->get_exclusive_group(member.ctx);
    definition.label = "Travel generation";
    definition.tooltip = "Selects how discontinuities between ordered extrusion leaves become travels.";
    AvoidCrossingPerimetersTravel::instance(orchestrator).register_exclusive_group(STEP_LAYER_EXTRUSION_EDIT, definition);
}

}}} // namespace slic3r_api::LayerExtrusionEdit::AvoidCrossingPerimetersTravelPlugin
