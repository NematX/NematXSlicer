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
#include "libslic3r/Api/plugin/cpp/VolumeViews.hpp"
#include "libslic3r/Plugins/Common/ClipperHelpers.hpp"
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
structural changes: the NoTravel volumes of every real instance are sliced at
the layer print_z in the worker's plugin storage, placed with the object, volume
and instance transforms, and unioned in platter coordinates. The sections,
Clipper operands and API collections live only for that call; no geometry cache
survives a run, and the provider keeps only execution-wide counters. Layer
workers share no mutable state, and the host barrier orders the summaries.

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
        |-- no_travel_sections            slice NoTravel instances at print_z and union them in platter space
        |-- ScopeTravelConnector::prepare snap seams and collect the ordered gaps
        |-- ScopeTravelConnector::apply   plan each gap, then write travels and flags
        |   `-- no_travel_path            direct gap or one validated Clipper detour
        `-- report_warning                execution summary from the last finished layer
*/

namespace slic3r_api { namespace LayerExtrusionEdit { namespace DefaultTravelPlugin {
// Register the permanent borrowed plugin and copy its alternative-selection setting description into the application.
// Print data is unchanged; allocation failures or rejected descriptions throw.
void register_default_travel_plugin(orchestrator_handle *orchestrator);

namespace {

using namespace TravelConnection;
namespace work = PluginCommon::polygon_work;
namespace clipper = PluginCommon::clipper;

// Slice the plan instances' NoTravel volumes at print_z into platter-space
// sections, unioned with holes preserved. API allocations belong to the
// worker's plugin storage and do not outlive the call.
work::ExPolygons no_travel_sections(const ConstPrintingPlan &plan, coord_t print_z,
                                    const plugin_run_context *run_ctx);

// Return the direct gap when it avoids the obstacles, else one shortest-ring
// detour accepted against every non-exempt component, else the direct gap.
// Whole connected components containing an endpoint are exempt for this gap.
// Increment exemptions and fallbacks for the execution summary.
std::vector<c_point> no_travel_path(const TravelEndpoint &source, const TravelEndpoint &target,
    extruder_index_t tool, const work::ExPolygons &sections, const plugin_run_context *run_ctx,
    uint64_t &exemptions, uint64_t &fallbacks);

work::ExPolygons no_travel_sections(const ConstPrintingPlan &plan, coord_t print_z,
                                   const plugin_run_context *run_ctx)
{
    StoredPolygonCollection rings(run_ctx->plugin_storage);
    for (uint32_t i = 0; i < plan.object_instance_count(); ++i) {
        throw_if_cancelled(run_ctx);
        const PrintingObjectInstance instance = plan.object_instance(i);
        if (!instance.has_object()) continue;
        const Object object = instance.object();
        const c_point center = object.center_offset();
        const c_point shift = object.instance_shift(uint32_t(instance.object_instance_idx()));
        const c_matrix4d transform = matrix4d_mul(matrix4d_translation(
            unscaled(shift.x) - unscaled(center.x), unscaled(shift.y) - unscaled(center.y), 0), object.transform());
        for (uint32_t v = 0; v < object.volume_count(); ++v) {
            throw_if_cancelled(run_ctx);
            const Volume volume = object.volume(v);
            if (!volume.is_no_travel()) continue;
            c_mesh_slicing_params params{};
            params.transform = matrix4d_mul(transform, volume_get_matrix(volume.handle()));
            auto slices = volume.mesh().slice_to_polygons(run_ctx->plugin_storage, params,
                {float(unscaled(print_z))}, run_ctx);
            rings.append_move_from(slices.front());
        }
    }
    if (rings.empty()) return {};
    const auto merged = clipper_union_ex(run_ctx->plugin_storage, rings.readonly(), CLIPPER_FILL_NONZERO);
    return clipper::from_expolygon_collection_view(merged.readonly());
}

std::vector<c_point> no_travel_path(const TravelEndpoint &source, const TravelEndpoint &target,
    extruder_index_t tool, const work::ExPolygons &sections, const plugin_run_context *run_ctx,
    uint64_t &exemptions, uint64_t &fallbacks)
{
    const auto direct = straight_path(source, target, tool);
    work::ExPolygons obstacles;
    bool exempt = false;
    for (const auto &section : sections) {
        throw_if_cancelled(run_ctx);
        if (section.contains(work::Point(direct.front()), false) || section.contains(work::Point(direct.back()), false))
            exempt = true;
        else
            obstacles.push_back(section);
    }
    exemptions += exempt;
    if (obstacles.empty()) return direct;
    const auto stored = clipper::to_stored_expolygons(run_ctx->plugin_storage, obstacles);
    const auto interior = clipper_offset_ex(run_ctx->plugin_storage, stored.readonly(), -double(SCALED_EPSILON));
    StoredPolyline line(run_ctx->plugin_storage);
    for (const c_point p : direct) line.push_back(p);
    if (clipper_intersection_polyline_expolygons(run_ctx->plugin_storage, line.readonly(), interior.readonly()).empty())
        return direct;
    const ClipperOperand rings(run_ctx->plugin_storage, stored.readonly());
    const auto candidate = clipper_detour_polyline_expolygons(
        run_ctx->plugin_storage, line.readonly(), rings, CLIPPER_DETOUR_SHORTEST);
    throw_if_cancelled(run_ctx);
    if (!clipper_intersection_polyline_expolygons(run_ctx->plugin_storage, candidate.readonly(), interior.readonly()).empty()) {
        ++fallbacks;
        return direct;
    }
    std::vector<c_point> result;
    for (uint32_t i = 0; i < candidate.size(); ++i) result.push_back(candidate.at(i));
    return result;
}

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
    const auto sections = no_travel_sections(ConstPrintingPlan(ctx->plan), layer_group.print_z(), run_ctx);
    auto chronology = m_connector.prepare(layer_group, ctx->group_idx, ctx->layer_group_idx);
    size_t index = 0;
    uint64_t exemptions = 0, fallbacks = 0;
    m_connector.apply(chronology, [&](const TravelEndpoint &source, const TravelEndpoint &target, extruder_index_t tool) {
        const PendingTravel &gap = chronology.travels[index++];
        if (sections.empty() || !gap.source_extrusion.valid() ||
            source.position.z != layer_group.print_z() || target.position.z != layer_group.print_z())
            return straight_path(source, target, tool);
        return no_travel_path(source, target, tool, sections, run_ctx, exemptions, fallbacks);
    }, progress());
    m_exemptions.fetch_add(exemptions, std::memory_order_relaxed);
    m_fallbacks.fetch_add(fallbacks, std::memory_order_relaxed);
    // The last completed layer reports one execution-wide summary, not one
    // warning per segment. Failed/cancelled executions never publish success.
    if (m_remaining.fetch_sub(1, std::memory_order_acq_rel) == 1 && (m_exemptions.load() || m_fallbacks.load())) {
        const std::string message = "NoTravel best-effort: " + std::to_string(m_exemptions.load()) +
            " travels exempted endpoint-containing components; " + std::to_string(m_fallbacks.load()) +
            " inadmissible detours kept their straight path. NoTravel is not a strict barrier.";
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
