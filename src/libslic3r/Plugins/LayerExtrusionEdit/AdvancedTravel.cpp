///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

#include "AdvancedTravel.hpp"

#include <algorithm>
#include <atomic>
#include <iterator>
#include <stdexcept>
#include <string>

#include "libslic3r/Api/plugin/c/steps/slic3r_step_layer_extrusion_edit.h"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "AvoidCrossing/CompatibilityPlanner.hpp"
#include "AdvancedTravel/DefaultZoneContributors.hpp"
#include "AdvancedTravel/ZonePlanner.hpp"
#include "NoTravelRouting.hpp"
#include "TravelConnectionHelpers.hpp"

/*
Advanced travel supplies XY routes between ordered extrusions to the layer
editor. Setup allocates connector summaries, and setup_run fills one summary per
layer group before the host's worker barrier. Each run first prepares a chronology:
an ordered record of borrowed extrusion gaps with their seams already snapped.
It then slices the layer's NoTravel volumes once (NoTravelRouting.hpp).
A gap is routed by the zone planner when perimeter avoidance applies to it
(ZonePlanner::eligible); every other gap, and every zone route that ends in a
straight fallback, takes the shared NoTravel path: direct, or one best-effort
detour around the NoTravel sections. Disabling avoid_crossing_perimeters therefore
keeps NoTravel avoidance.
The zone planner first checks whether any gap is eligible, then collects active
attractors, forbidden tops and the NoTravel sections, and routes in chronology order.
Printed tops are opt-in and replace static forbidden tops when active; their dates
start after the last arrival in each original island. Attractors publish raw
perimeter slices dated by source-or-target membership. NoTravel sections are
forbidden for the whole layer, for every tool and instance. The planner builds a fixed
initial platter from the scaled bed_shape contour, groups endpoint-containing
original forbidden and outside-part pieces transitively by overlap or contact.
For each combination of keeping or ignoring these groups, it unions the remaining
pieces as Z3 and forms the attractor and platter routing domains described in
ZonePlanner.hpp. It expands only the initial platter by SCALED_EPSILON for
containment checks because diff_pl has no reliable side for a path exactly on a border.
Attractor and platter hypotheses are ranked by estimated outside length plus weighted
inside length. Construction retains exact endpoints; validation checks the expanded
initial platter and the budget. Straight exits are checked against the inset of other
kept obstacles; the rest is checked against the inset of Z3. Only material checks
shrink obstacles by SCALED_EPSILON; domains, detours, the reference and the score use
the original geometry. When a usable platter-only reference
exists, validation compares actual outside length against that reference. Rejected hypotheses
leave the next choice available; only exhaustion counts a straight fallback.
Each chosen component supplies its own route along crossed W_a rings; bridges and
connectors detour on W_p rings, including obstacle-facing Z4 rings and the platter
border, between their selected points.
Endpoints without a region or on auxiliary layers offer only a platter attachment
when W_p is nonempty.
Travels touching such endpoints include slices of the plan's objects as forbidden
components, not the synthetic wipe tower. The first passing hypothesis per
combination competes by length, with length inside any endpoint group counted
three times. On ties the first combination wins. The planner counts only the
winning route, or counts a straight fallback if none passes.
The planner borrows this layer group's plan, index and Z to cache those slices.
Layers without eligible gaps bypass zone construction. Permanent no-op plugins expose
global contributor activation. Active means present in the host activation set,
configured by activated.ini. Computation objects are worker-local. The private
implementation checks each contributor ID with orchestrator_find_plugin. Active
contributors share an input cache owned by advanced travel for boundary geometry
and routing settings; the compatibility planner, used only for regional eligibility,
keeps its own cache, so a layer can be copied twice.
The incoming layer-entry travel has no source extrusion and remains straight.
The connector inserts travels, interpolates Z and updates flags and progress.
The plan owns all borrowed extrusions throughout both phases; no chronology or
planner survives the callback. Cancellation and routing errors propagate.
NoTravel exemptions and rejected detours are added to execution-wide counters; the
last finished layer reports them in one warning.

    register_advanced_travel_plugin: register the permanent provider
    register_advanced_travel_contributors: register three non-exclusive no-op zone members
    setup_impl: reset counters, then reset and allocate connector summaries
    setup_run_impl: publish this layer group's summary
    run_impl: own the chronology and planners for one worker callback
        |-- ScopeTravelConnector::prepare: record ordered gaps and snap seams
        |-- NoTravel::sections: slice NoTravel volumes at print_z
        |-- eligibility guard: evaluate ZonePlanner::eligible without building zones
        |-- TravelZoneView: collect active attractors, forbidden tops and NoTravel zones when eligible
        |-- ScopeTravelConnector::apply: call the planner once per gap in order and insert routes
        |   |-- ZonePlanner::path: group endpoint obstacles and score accepted routes
        |   |   |-- outside_part / platter_blockers: classify endpoints and collect outside-part geometry
        |   |   |-- touches: group original pieces by overlap or contact
        |   |   `-- route: use cached domains for one combination and try ranked hypotheses
        |   |       |-- endpoint_candidates / exit_candidates: attach endpoints to W
        |   |       |-- detour: route connectors on W_p rings with the public Clipper operation
        |   |       `-- blocked: check exits and the remaining path against their respective obstacle masks
        |   `-- NoTravel::path: ineligible gaps and zone fallbacks
        `-- report_warning: NoTravel summary from the last finished layer
*/

namespace slic3r_api { namespace LayerExtrusionEdit { namespace AdvancedTravelPlugin {
// Register a permanent provider borrowing the orchestrator; allocation errors propagate and registration rejections are logged by the host.
void register_advanced_travel_plugin(orchestrator_handle *orchestrator);
// Register three permanent no-op zone contributors; allocation errors propagate and host rejections are logged.
void register_advanced_travel_contributors(orchestrator_handle *orchestrator);

namespace {
using namespace TravelConnection;
namespace zones = slic3r_api::LayerExtrusionEdit::AdvancedTravel;

const char *const k_attractors_id = "layer_extrusion_edit.travel.advanced.zones.island_attractors";
const char *const k_tops_id = "layer_extrusion_edit.travel.advanced.zones.forbidden_tops";
const char *const k_printed_tops_id = "layer_extrusion_edit.travel.advanced.zones.printed_tops";
const char *const k_contributor_dependencies[] = {"layer_extrusion_edit.travel.advanced", nullptr};

class ContributorChoice : public PluginBase
{
public:
    // Borrow the orchestrator and retain static identity and description strings for this permanent activation member.
    ContributorChoice(orchestrator_handle *orchestrator, const char *id, const char *name,
                      const char *description = "Enables zones collected by Advanced travel.") :
        PluginBase(orchestrator), m_id(id), m_name(name), m_description(description) {}
private:
    // Return the static contributor identity.
    const char *id_impl() const noexcept override { return m_id; }
    // Return the static display name.
    const char *name_impl() const noexcept override { return m_name; }
    // Describe the computation controlled by this otherwise inert member.
    const char *description_impl() const noexcept override { return m_description; }
    // Allow independent activation without a selector.
    const char *exclusive_group_impl() const noexcept override { return ""; }
    // Participate in layer editing alongside the provider.
    slicing_step_t step_impl() const noexcept override { return STEP_LAYER_EXTRUSION_EDIT; }
    // Require the provider that collects these zones.
    const char *const *dependencies_impl() const noexcept override { return k_contributor_dependencies; }
    // Keep the provider's step priority.
    int32_t priority_impl() const noexcept override { return -50; }
    // Leave geometry unchanged; only AdvancedTravel calls the contributor computation.
    void run_impl(const plugin_run_context *) const override {}
    const char *m_id;
    const char *m_name;
    const char *m_description;
};

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
    // Select advanced travel before straight travel (0) when no ID is stored.
    int32_t exclusive_group_priority_impl() const noexcept override;
    // Return the count, and copy static key descriptions when the caller supplies an array of that size.
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override;
    // Return the static progress format.
    const char *progress_message_format_impl() const noexcept override;
    // Reset counters and summaries, then allocate summaries for a valid print and plan; allocation errors propagate.
    void setup_impl(const plugin_run_context *run_ctx, uint32_t run_count) const override;
    // Fill this layer group's summary for a valid setup and layer; connector errors propagate.
    void setup_run_impl(const plugin_run_context *run_ctx) const override;
    // Prepare gaps, apply worker-local zone or NoTravel routes and publish the NoTravel summary from the last
    // layer; invalid contexts do nothing and cancellation or errors propagate.
    void run_impl(const plugin_run_context *run_ctx) const override;

    mutable ScopeTravelConnector m_connector;
    mutable bool m_setup_valid = false;
    // Remaining layer runs and execution-wide NoTravel counts for the final summary.
    mutable std::atomic<uint32_t> m_remaining{0};
    mutable std::atomic<uint64_t> m_exemptions{0}, m_fallbacks{0};
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
    return "Routes travels around perimeters, tops and other islands where avoid_crossing_perimeters applies, "
           "and around NoTravel volumes everywhere (best effort).";
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
    return -2;
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
        {"solid_infill_extruder", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        // Attractor rings are withdrawn by this distance from each island outline.
        {"avoid_crossing_perimeters_inset", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        // Advanced routes use the fixed printable contour as their initial platter.
        {"bed_shape", RAW_CO_VECTOR_POINT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE}
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
    m_remaining.store(run_count);
    m_exemptions.store(0);
    m_fallbacks.store(0);
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
    const Print print(ctx->print);
    const Config config = print.config();
    const coord_t print_z = layer_group.print_z();
    const PluginCommon::polygon_work::ExPolygons no_travel =
        NoTravel::sections(ConstPrintingPlan(ctx->plan), print_z, run_ctx);
    NoTravel::Counts counts;
    const auto no_travel_path = [&](const PendingTravel &travel) {
        return NoTravel::path(travel.source, travel.target, travel.extruder_id,
                              travel.source_extrusion.handle() == nullptr, print_z, no_travel, run_ctx, counts);
    };
    AvoidCrossing::CompatibilityPlanner planner(print, plan, run_ctx, ctx->group_idx, print_z);
    const Config region_defaults(print_get_default_region_config(ctx->print));
    const bool auxiliary_avoidance = region_defaults.bool_or_default("avoid_crossing_perimeters", false);
    bool any_avoidance_active = false;
    for (const PendingTravel &travel : chronology.travels)
        if (travel.source_extrusion.handle() != nullptr &&
            zones::ZonePlanner::eligible(planner, config, auxiliary_avoidance, travel.source, travel.target)) {
            any_avoidance_active = true;
            break;
        }
    size_t t = 0;
    if (!any_avoidance_active) {
        m_connector.apply(chronology, [&no_travel_path, &chronology, &t](
            const TravelEndpoint &, const TravelEndpoint &, extruder_index_t) {
            return no_travel_path(chronology.travels.at(t++));
        }, progress());
    } else {
        // The contributors share this cache; the compatibility planner owns its separate cache.
        AvoidCrossing::LayerInputCache inputs;
        const zones::ZoneContributorInput input{print, plan, *run_ctx,
                                              inputs, chronology.travels};
        const zones::DefaultAttractorContributor attractors;
        const zones::DefaultForbiddenTopsContributor tops;
        const zones::PrintedTopsContributor printed_tops;
        const zones::NoTravelContributor no_travel_zones(no_travel);
        std::vector<std::reference_wrapper<const zones::ZoneContributor>> active;
        if (orchestrator_find_plugin(m_orchestrator, k_attractors_id) != nullptr)
            active.push_back(std::cref(attractors));
        if (orchestrator_find_plugin(m_orchestrator, k_printed_tops_id) != nullptr)
            active.push_back(std::cref(printed_tops));
        else if (orchestrator_find_plugin(m_orchestrator, k_tops_id) != nullptr)
            active.push_back(std::cref(tops));
        active.push_back(std::cref(no_travel_zones));
        zones::TravelZoneView view(active, input);
        zones::ZonePlanner zone_planner(planner, view, config, auxiliary_avoidance, run_ctx, print, inputs,
            plan, ctx->group_idx, print_z);
        // apply calls this exactly once per PendingTravel, in chronology order.
        m_connector.apply(chronology, [&zone_planner, &no_travel_path, &chronology, &t](
            const TravelEndpoint &, const TravelEndpoint &, extruder_index_t) {
            const size_t current = t++;
            const PendingTravel &travel = chronology.travels.at(current);
            if (travel.source_extrusion.handle() == nullptr || !zone_planner.eligible(travel.source, travel.target))
                return no_travel_path(travel);
            // A counted fallback is the zone planner's direct gap; NoTravel may still detour it.
            const size_t fallbacks = zone_planner.fallbacks();
            std::vector<c_point> route = zone_planner.path(current, travel);
            return zone_planner.fallbacks() == fallbacks ? route : no_travel_path(travel);
        }, progress());
    }
    if (t != chronology.travels.size())
        throw std::logic_error("Advanced travel did not consume every pending travel.");
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

void register_advanced_travel_plugin(orchestrator_handle *orchestrator)
{
    static AdvancedTravel plugin(orchestrator);
    orchestrator_register_plugin(orchestrator, plugin.c_instance());
}

void register_advanced_travel_contributors(orchestrator_handle *orchestrator)
{
    static ContributorChoice attractors(orchestrator, k_attractors_id, "Island attractor zones");
    static ContributorChoice tops(orchestrator, k_tops_id, "Forbidden top zones");
    static ContributorChoice printed_tops(orchestrator, k_printed_tops_id, "Printed top zones (replace static tops)",
        "Replaces static forbidden tops with tops avoided after the last arrival in their island by the advanced planner.");
    orchestrator_register_plugin(orchestrator, attractors.c_instance());
    orchestrator_register_plugin(orchestrator, tops.c_instance());
    orchestrator_register_plugin(orchestrator, printed_tops.c_instance());
}
}}} // namespace slic3r_api::LayerExtrusionEdit::AdvancedTravelPlugin
