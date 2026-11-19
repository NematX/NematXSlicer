///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "DefaultTravel.hpp"

#include <atomic>
#include <cstdint>
#include <stdexcept>
#include <string>

#include "libslic3r/Api/plugin/c/steps/slic3r_step_layer_extrusion_edit.h"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "NoTravelRouting.hpp"
#include "TravelConnectionHelpers.hpp"

/*
Default straight travels
========================

CreateTransitionScope identifies discontinuities and reserves the target
scope's travel phase. This provider selects the direct two-point path, or one
Clipper detour when that path enters a NoTravel section, while
ScopeTravelConnector handles endpoint resolution, epsilon snapping, Z offsets
and final materialization flags.

setup_run publishes the connector summary of one layer; run_impl then prepares
that layer's gaps and asks the planner for each path. The plan is read without
structural changes: NoTravelRouting.hpp slices the NoTravel volumes of every
real instance at the layer print_z in the worker's plugin storage and unions
them in platter coordinates. The sections, Clipper operands and API collections
live only for that call; no geometry cache survives a run, and the provider
keeps only execution-wide counters. Layer workers share no mutable state, and
the host barrier orders the summaries.

The detour policy is deliberately local: a component containing an endpoint is
exempt for that gap, one CLIPPER_DETOUR_SHORTEST arc is validated against all
other components, and a rejected candidate keeps the direct path. Only ordinary
in-layer gaps at print_z are routed; layer entries, Z moves, scripts and wipes
keep their paths. No setting is read and no physical clearance is added. The
last finished layer reports the exemption and fallback counts once. The contract
is [Default straight travel](/doc/plugins/default-travel.md).

    DefaultTravel::setup_impl             reset the connector and counters, then allocate layer summaries
        `-- ScopeTravelConnector::setup   one summary slot per final layer

    DefaultTravel::setup_run_impl         publish one layer's boundary summary
        `-- ScopeTravelConnector::setup_run

    DefaultTravel::run_impl
        |-- NoTravel::sections            slice NoTravel instances at print_z and union them in platter space
        |-- ScopeTravelConnector::prepare snap seams and collect the ordered gaps
        |-- ScopeTravelConnector::apply   plan each gap, then write travels and flags
        |   `-- NoTravel::path            direct gap or one validated Clipper detour
        `-- report_warning                execution summary from the last finished layer
*/

namespace slic3r_api { namespace LayerExtrusionEdit { namespace DefaultTravelPlugin {
// Register the permanent borrowed plugin and copy its alternative-selection setting description into the application.
// Print data is unchanged; allocation failures or rejected descriptions throw.
void register_default_travel_plugin(orchestrator_handle *orchestrator);

namespace {

using namespace TravelConnection;

const char *const k_dependencies[] = {
    "layer_extrusion_edit.transition_scope.default",
    "layer_extrusion_edit.entry_state.default",
    nullptr
};

class DefaultTravel : public PluginBase
{
public:
    static DefaultTravel &instance(orchestrator_handle *orchestrator);
    explicit DefaultTravel(orchestrator_handle *orchestrator);

private:
    const char *id_impl() const noexcept override;
    const char *name_impl() const noexcept override;
    const char *description_impl() const noexcept override;
    const char *exclusive_group_impl() const noexcept override;
    slicing_step_t step_impl() const noexcept override;
    const char *const *dependencies_impl() const noexcept override;
    int32_t priority_impl() const noexcept override;
    // Return the preference for straight travel. Nothing is changed or transferred, and the call cannot fail.
    int32_t exclusive_group_priority_impl() const noexcept override { return 0; }
    const char *progress_message_format_impl() const noexcept override;
    void setup_impl(const plugin_run_context *run_ctx, uint32_t run_count) const override;
    void setup_run_impl(const plugin_run_context *run_ctx) const override;
    void run_impl(const plugin_run_context *run_ctx) const override;

    mutable ScopeTravelConnector m_connector;
    mutable bool m_setup_valid = false;
    // Remaining layer runs and execution-wide counts for the final summary.
    mutable std::atomic<uint32_t> m_remaining{0};
    mutable std::atomic<uint64_t> m_exemptions{0}, m_fallbacks{0};
};

DefaultTravel &DefaultTravel::instance(orchestrator_handle *orchestrator)
{
    static DefaultTravel plugin(orchestrator);
    return plugin;
}

DefaultTravel::DefaultTravel(orchestrator_handle *orchestrator) :
    PluginBase(orchestrator),
    m_connector(orchestrator)
{}

const char *DefaultTravel::id_impl() const noexcept
{
    return "layer_extrusion_edit.travel.default";
}

const char *DefaultTravel::name_impl() const noexcept
{
    return "Default straight travel";
}

const char *DefaultTravel::description_impl() const noexcept
{
    return "Connects ordered extrusion leaves with straight travels, with best-effort NoTravel volume avoidance. "
           "No perimeter avoidance or physical clearance; layer-entry and Z moves are unchanged.";
}

const char *DefaultTravel::exclusive_group_impl() const noexcept
{
    return "layer_extrusion_edit.travel";
}

slicing_step_t DefaultTravel::step_impl() const noexcept
{
    return STEP_LAYER_EXTRUSION_EDIT;
}

const char *const *DefaultTravel::dependencies_impl() const noexcept
{
    return k_dependencies;
}

int32_t DefaultTravel::priority_impl() const noexcept
{
    return -50;
}

const char *DefaultTravel::progress_message_format_impl() const noexcept
{
    return "Connecting ordered extrusion paths: %u / %u trees";
}

void DefaultTravel::setup_impl(const plugin_run_context *run_ctx, const uint32_t run_count) const
{
    m_connector.reset();
    m_remaining.store(run_count);
    m_exemptions.store(0);
    m_fallbacks.store(0);
    const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run_ctx);
    m_setup_valid = ctx != nullptr && ctx->plan != nullptr;
    if (m_setup_valid)
        m_connector.setup(PrintingPlan(ctx->plan), run_count);
}

void DefaultTravel::setup_run_impl(const plugin_run_context *run_ctx) const
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

void DefaultTravel::run_impl(const plugin_run_context *run_ctx) const
{
    if (!m_setup_valid)
        return;
    const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run_ctx);
    if (ctx == nullptr || ctx->layer_group == nullptr)
        return;

    PrintingLayerGroup layer_group(ctx->layer_group);
    const auto sections = NoTravel::sections(ConstPrintingPlan(ctx->plan), layer_group.print_z(), run_ctx);
    auto chronology = m_connector.prepare(layer_group, ctx->group_idx, ctx->layer_group_idx);
    size_t index = 0;
    NoTravel::Counts counts;
    m_connector.apply(chronology, [&](const TravelEndpoint &source, const TravelEndpoint &target, extruder_index_t tool) {
        const PendingTravel &gap = chronology.travels[index++];
        return NoTravel::path(source, target, tool, !gap.source_extrusion.valid(), layer_group.print_z(),
                              sections, run_ctx, counts);
    }, progress());
    m_exemptions.fetch_add(counts.exemptions, std::memory_order_relaxed);
    m_fallbacks.fetch_add(counts.fallbacks, std::memory_order_relaxed);
    // The last completed layer reports one execution-wide summary, not one
    // warning per segment. Failed/cancelled executions never publish success.
    if (m_remaining.fetch_sub(1, std::memory_order_acq_rel) == 1) {
        const std::string message = NoTravel::warning_message(m_exemptions.load(), m_fallbacks.load());
        if (!message.empty())
            report_warning(run_ctx, message.c_str());
    }
}

} // namespace

void register_default_travel_plugin(orchestrator_handle *orchestrator)
{
    orchestrator_register_plugin(orchestrator, DefaultTravel::instance(orchestrator).c_instance());
    const plugin_instance member = DefaultTravel::instance(orchestrator).c_instance();
    raw_exclusive_group_def definition = raw_exclusive_group_def_init();
    definition.exclusive_group = member.vt->get_exclusive_group(member.ctx);
    definition.label = "Travel generation";
    definition.tooltip = "Selects how discontinuities between ordered extrusion leaves become travels.";
    DefaultTravel::instance(orchestrator).register_exclusive_group(STEP_LAYER_EXTRUSION_EDIT, definition);
}

}}} // namespace slic3r_api::LayerExtrusionEdit::DefaultTravelPlugin
