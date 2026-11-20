///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

#include "AdvancedTravel.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <iterator>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "libslic3r/Api/plugin/c/steps/slic3r_step_layer_extrusion_edit.h"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/Api/plugin/cpp/RegionSettingsViews.hpp"
#include "libslic3r/Plugins/Common/ClipperHelpers.hpp"
#include "AvoidCrossing/AvoidCrossingInput.hpp"
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
The incoming layer-entry travel has no source extrusion. With the provider-owned
setting avoid_crossing_layer_entry = straight (the default) it keeps the NoTravel path.
With routed, the zone planner routes it from a synthesized start: setup_run records,
per layer group, the last plan instance of a real object printed there, and the next
layer looks for an island of that instance's normal layer at its own print_z whose
published perimeter slices (the areas island attractors are made of) contain the entry
point or lie within four nozzle diameters of it. The entry then starts "in piece" from
a point just inside those slices, with that island's regional provenance, after a
straight approach when it was outside. Starting inside the attractor area lets the zone
planner attach the start to the island ring instead of the platter. Without such an island it
starts "from outside", as a regionless endpoint that only attaches to the platter.
The connector still interpolates Z along the whole route.
The connector inserts travels, interpolates Z and updates flags and progress.
The plan owns all borrowed extrusions throughout both phases; no chronology or
planner survives the callback. Cancellation and routing errors propagate.
NoTravel exemptions and rejected detours are added to execution-wide counters; the
last finished layer reports them in one warning.
With gcode_comments (Verbose G-code), each travel also receives one comment that
names its route: layer entry, outside perimeter avoidance, zone route with the
attachment of each end (island ring or platter, straight exit from an obstacle,
link from the island band, obstacle crossed), inter-island bridge, outside-part
endpoint, the zone planner's fallback reason, and the NoTravel outcome. The
connector writes these comments after apply(), in the targets' before phases.

    register_advanced_travel_plugin: register the permanent provider
    register_advanced_travel_contributors: register three non-exclusive no-op zone members
    inilialize_impl: define avoid_crossing_layer_entry
    setup_impl: reset counters, then reset and allocate connector and last-instance summaries
    setup_run_impl: publish this layer group's summaries
    run_impl: own the chronology and planners for one worker callback
        |-- ScopeTravelConnector::prepare: record ordered gaps and snap seams
        |-- entry_in_piece: synthesize a routed layer-entry start from the previous layer's last instance
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
        |-- describe_zone_route / describe_no_travel: verbose travel comments
        |-- ScopeTravelConnector::annotate: insert the verbose comments
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
// Provider-owned choice between a straight and a routed layer-entry travel.
const char *const k_layer_entry_key = "avoid_crossing_layer_entry";
const char *const k_defined_keys[] = {k_layer_entry_key};

// Return a copy of the layer-entry endpoint that starts inside the perimeter slices of an island of the catalog
// instance's normal layer at print_z, with that island's regional provenance, when the entry point lies in those
// slices or within max_distance of them; the nearest such area wins. Return nothing without such an island, or when no region
// island resolves avoid_crossing_perimeters at the start point. The plan and its sliced layers are only read.
std::optional<TravelEndpoint> entry_in_piece(const PrintingPlan &plan, uint32_t catalog, coord_t print_z,
                                             const TravelEndpoint &entry, coord_t max_distance,
                                             const plugin_run_context *run_ctx);

// Describe the zone planner's latest decision for a verbose comment, without the NoTravel outcome.
std::string describe_zone_route(const zones::ZonePlanner::Decision &decision);
// Describe one endpoint attachment of an admitted zone route.
std::string describe_attachment(bool island, bool in_obstacle, bool obstacle_ignored, bool exit, bool link);
// Describe the NoTravel outcome of one gap from the counters before and after NoTravel::path and its point count.
std::string describe_no_travel(const NoTravel::Counts &before, const NoTravel::Counts &after, size_t points);

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
    // Report the provider-owned layer-entry setting; the caller supplies storage for the returned count.
    int32_t defined_config_keys(const char **keys) const noexcept override;
    // Define avoid_crossing_layer_entry once; rejected definitions are logged by the host.
    void inilialize_impl(storage_handle *storage) const override;
    // Return the static progress format.
    const char *progress_message_format_impl() const noexcept override;
    // Reset counters and summaries, then allocate summaries for a valid print and plan; allocation errors propagate.
    void setup_impl(const plugin_run_context *run_ctx, uint32_t run_count) const override;
    // Fill this layer group's connector summary and its last printed object instance; connector errors propagate.
    void setup_run_impl(const plugin_run_context *run_ctx) const override;
    // Prepare gaps, apply worker-local zone or NoTravel routes and publish the NoTravel summary from the last
    // layer; invalid contexts do nothing and cancellation or errors propagate.
    void run_impl(const plugin_run_context *run_ctx) const override;

    mutable ScopeTravelConnector m_connector;
    mutable bool m_setup_valid = false;
    // Last plan instance of a real object in each group and layer group, written by its own setup_run before the
    // host barrier and read by the next layer's run.
    mutable std::vector<std::vector<uint32_t>> m_last_instances;
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
        {k_layer_entry_key, RAW_CO_ENUM, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        // Advanced routes use the fixed printable contour as their initial platter.
        {"bed_shape", RAW_CO_VECTOR_POINT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE}
    };
    if (keys != nullptr)
        std::copy(std::begin(used_keys), std::end(used_keys), keys);
    return int32_t(sizeof(used_keys) / sizeof(used_keys[0]));
}

int32_t AdvancedTravel::defined_config_keys(const char **keys) const noexcept
{
    if (keys != nullptr)
        std::copy(std::begin(k_defined_keys), std::end(k_defined_keys), keys);
    return int32_t(std::size(k_defined_keys));
}

void AdvancedTravel::inilialize_impl(storage_handle *) const
{
    const key_value_string_pair_t values[] = {{"straight", "Straight"}, {"routed", "Routed"}};
    raw_config_option_def def = raw_config_option_def_init();
    def.opt_key = k_layer_entry_key;
    def.type = RAW_CO_ENUM;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_LAYER_EXTRUSION_EDIT;
    def.category = RAW_OPTION_CATEGORY_PERIMETER;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
    def.label = "Avoid crossing perimeters - Layer entry";
    def.tooltip = "How the first travel of a layer, from where the previous layer ended, avoids crossing perimeters."
        "\nStraight: keep the direct travel."
        "\nRouted: route it like the other travels. It starts inside the island of the last object printed on the "
        "previous layer when its start lies in that island or within four nozzle diameters of it, otherwise from "
        "outside the part. Z still rises along the whole route.";
    def.enum_def.value_label_pairs = {values, 2};
    def.default_serialized_value = "straight";
    orchestrator_create_option_def(m_orchestrator, &def);
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
    m_last_instances.clear();
    if (!m_setup_valid)
        return;
    const PrintingPlan plan(ctx->plan);
    m_connector.setup(plan, run_count);
    m_last_instances.resize(plan.group_count());
    for (uint32_t group = 0; group < plan.group_count(); ++group)
        m_last_instances[group].assign(plan.group(group).layer_group_count(), SLIC3R_PRINTING_OBJECT_INSTANCE_ID_NONE);
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
    // The next layer group's entry starts where this one's last real object finished.
    const PrintingPlan plan(ctx->plan);
    uint32_t last = SLIC3R_PRINTING_OBJECT_INSTANCE_ID_NONE;
    for (uint32_t t = 0; t < layer_group.tool_group_count(); ++t) {
        PrintingToolGroup tool = layer_group.tool_group(t);
        for (uint32_t e = 0; e < tool.extrusion_count(); ++e) {
            const uint32_t catalog = tool.extrusion(e).printing_catalog_id();
            if (catalog < plan.object_instance_count() && plan.object_instance(catalog).has_object())
                last = catalog;
        }
    }
    m_last_instances.at(ctx->group_idx).at(ctx->layer_group_idx) = last;
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
    // Verbose output names every travel's route; comments stay empty otherwise.
    const bool verbose = config.bool_or_default("gcode_comments", false);
    std::vector<std::string> comments(verbose ? chronology.travels.size() : 0);
    const auto no_travel_path = [&](const size_t index, const std::string &reason) {
        const PendingTravel &travel = chronology.travels.at(index);
        const NoTravel::Counts before = counts;
        std::vector<c_point> route = NoTravel::path(travel.source, travel.target, travel.extruder_id,
            travel.source_extrusion.handle() == nullptr, print_z, no_travel, run_ctx, counts);
        if (verbose)
            comments[index] = "advanced travel: " + reason + "; " + describe_no_travel(before, counts, route.size());
        return route;
    };
    AvoidCrossing::CompatibilityPlanner planner(print, plan, run_ctx, ctx->group_idx, print_z);
    const Config region_defaults(print_get_default_region_config(ctx->print));
    const bool auxiliary_avoidance = region_defaults.bool_or_default("avoid_crossing_perimeters", false);
    // The zone planner reads planned travels: a routed layer entry may start in piece, the others are unchanged.
    const bool routed_entry = config.has(k_layer_entry_key) && config.get(k_layer_entry_key).serialize() == "routed";
    std::vector<PendingTravel> planned = chronology.travels;
    std::vector<bool> in_piece(planned.size(), false);
    if (routed_entry) {
        const uint32_t group = ctx->group_idx, index = ctx->layer_group_idx;
        const uint32_t previous = index > 0 ? m_last_instances.at(group).at(index - 1) :
            group > 0 && !m_last_instances.at(group - 1).empty() ? m_last_instances.at(group - 1).back() :
            SLIC3R_PRINTING_OBJECT_INSTANCE_ID_NONE;
        for (size_t i = 0; i < planned.size(); ++i) {
            if (planned[i].source_extrusion.handle() != nullptr) continue;
            const coord_t reach = scale_i(4. * config.vector_float_or_default("nozzle_diameter", planned[i].extruder_id, 0.4));
            if (const std::optional<TravelEndpoint> start =
                    entry_in_piece(plan, previous, print_z, planned[i].source, reach, run_ctx)) {
                planned[i].source = *start;
                in_piece[i] = true;
            }
        }
    }
    bool any_avoidance_active = false;
    for (const PendingTravel &travel : planned)
        if ((travel.source_extrusion.handle() != nullptr || routed_entry) &&
            zones::ZonePlanner::eligible(planner, config, auxiliary_avoidance, travel.source, travel.target)) {
            any_avoidance_active = true;
            break;
        }
    size_t t = 0;
    if (!any_avoidance_active) {
        m_connector.apply(chronology, [&no_travel_path, &chronology, &t](
            const TravelEndpoint &, const TravelEndpoint &, extruder_index_t) {
            const size_t current = t++;
            return no_travel_path(current, chronology.travels[current].source_extrusion.handle() == nullptr ?
                "layer entry" : "outside perimeter avoidance");
        }, progress());
    } else {
        // The contributors share this cache; the compatibility planner owns its separate cache.
        AvoidCrossing::LayerInputCache inputs;
        const zones::ZoneContributorInput input{print, plan, *run_ctx,
                                              inputs, planned};
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
        m_connector.apply(chronology, [&zone_planner, &no_travel_path, &chronology, &planned, &in_piece, &t,
                                       &comments, verbose, routed_entry](
            const TravelEndpoint &, const TravelEndpoint &, extruder_index_t) {
            const size_t current = t++;
            const PendingTravel &travel = planned.at(current);
            const bool entry = travel.source_extrusion.handle() == nullptr;
            if (entry && !routed_entry)
                return no_travel_path(current, "layer entry");
            const std::string prefix = !entry ? "" : in_piece[current] ? "layer entry in piece, " :
                                                                         "layer entry from outside, ";
            if (!zone_planner.eligible(travel.source, travel.target))
                return no_travel_path(current, prefix + "outside perimeter avoidance");
            // A counted fallback is the zone planner's direct gap; NoTravel may still detour it.
            const size_t fallbacks = zone_planner.fallbacks();
            std::vector<c_point> route = zone_planner.path(current, travel);
            if (zone_planner.fallbacks() != fallbacks)
                return no_travel_path(current, prefix + describe_zone_route(zone_planner.last_decision()));
            // An in-piece start moved into its island keeps the recorded entry point first, then approaches straight.
            const TravelEndpoint &recorded = chronology.travels[current].source;
            if (route.front().x != recorded.position.x || route.front().y != recorded.position.y)
                route.insert(route.begin(), c_point{recorded.position.x, recorded.position.y});
            if (verbose)
                comments[current] = "advanced travel: " + prefix + describe_zone_route(zone_planner.last_decision());
            return route;
        }, progress());
    }
    if (t != chronology.travels.size())
        throw std::logic_error("Advanced travel did not consume every pending travel.");
    if (verbose)
        m_connector.annotate(chronology, comments);
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

std::optional<TravelEndpoint> entry_in_piece(const PrintingPlan &plan, const uint32_t catalog, const coord_t print_z,
                                             const TravelEndpoint &entry, const coord_t max_distance,
                                             const plugin_run_context *run_ctx)
{
    namespace work = PluginCommon::polygon_work;
    if (catalog >= plan.object_instance_count()) return std::nullopt;
    const PrintingObjectInstance instance = plan.object_instance(catalog);
    if (!instance.has_object()) return std::nullopt;
    const Object object = instance.object();
    const uint64_t copy = instance.object_instance_idx();
    if (copy >= object.instance_count()) return std::nullopt;
    const c_point shift = object.instance_shift(static_cast<size_t>(copy));
    const work::Point local(entry.position.x - shift.x, entry.position.y - shift.y);
    for (uint32_t layer_idx = 0; layer_idx < object.layer_count(); ++layer_idx) {
        throw_if_cancelled(run_ctx);
        const Layer layer = object.layer(layer_idx);
        if (std::abs(layer.print_z() - print_z) > SCALED_EPSILON || AvoidCrossing::is_off_object_layer(layer))
            continue;
        std::optional<TravelEndpoint> best;
        double best_distance = double(max_distance);
        for (uint32_t island_idx = 0; island_idx < layer.island_count(); ++island_idx) {
            throw_if_cancelled(run_ctx);
            const LayerIsland island = layer.island(island_idx);
            // Island attractors are built from these slices, so a start inside them can attach to the island ring.
            const work::ExPolygons areas = PluginCommon::clipper::from_expolygon_collection_view(island.perimeter_slices());
            work::Point start = local;
            double distance = std::numeric_limits<double>::max();
            for (const work::ExPolygon &area : areas) {
                if (area.contains(local, true)) {
                    start = local;
                    distance = 0.;
                    break;
                }
                // Project onto the nearest contour or hole edge, then continue a little past the border
                // so the start is unambiguously inside.
                work::Point nearest = local;
                double nearest_distance = std::numeric_limits<double>::max();
                const auto project = [&](const work::Points &ring) {
                    for (size_t i = 0; i < ring.size(); ++i) {
                        const work::Point &a = ring[i], &b = ring[(i + 1) % ring.size()];
                        const double dx = double(b.x() - a.x()), dy = double(b.y() - a.y());
                        const double length2 = dx * dx + dy * dy;
                        const double u = length2 <= 0. ? 0. : std::clamp(
                            (double(local.x() - a.x()) * dx + double(local.y() - a.y()) * dy) / length2, 0., 1.);
                        const work::Point q(coord_t(std::llround(a.x() + u * dx)), coord_t(std::llround(a.y() + u * dy)));
                        const double d = std::hypot(double(local.x() - q.x()), double(local.y() - q.y()));
                        if (d < nearest_distance) { nearest_distance = d; nearest = q; }
                    }
                };
                project(area.contour.points);
                for (const work::Polygon &hole : area.holes) project(hole.points);
                if (nearest_distance <= 0. || nearest_distance >= distance) continue;
                const double step = 4. * SCALED_EPSILON / nearest_distance;
                const work::Point inside(coord_t(std::llround(nearest.x() + (nearest.x() - local.x()) * step)),
                                         coord_t(std::llround(nearest.y() + (nearest.y() - local.y()) * step)));
                if (!area.contains(inside, false)) continue;
                start = inside;
                distance = nearest_distance;
            }
            if (distance > best_distance) continue;
            if (best && distance >= best_distance) continue;
            for (uint32_t group = 0; group < island.region_island_count(); ++group) {
                const LayerRegionIsland region_island = island.region_island(group);
                if (!region_island.valid() || region_island.region_count() == 0) continue;
                if (!RegionSettings::lookup_at_point(region_island, {"avoid_crossing_perimeters"},
                                                     c_point{start.x(), start.y()}).found()) continue;
                TravelEndpoint result = entry;
                result.position.x = start.x() + shift.x;
                result.position.y = start.y() + shift.y;
                result.region_island = region_island;
                result.printing_catalog_id = catalog;
                result.interior_point = c_point{};
                result.has_interior_point = false;
                best = result;
                best_distance = distance;
                break;
            }
        }
        return best;
    }
    return std::nullopt;
}

std::string describe_zone_route(const zones::ZonePlanner::Decision &decision)
{
    using Outcome = zones::ZonePlanner::Decision::Outcome;
    switch (decision.outcome) {
    case Outcome::Ineligible: return "outside perimeter avoidance";
    case Outcome::NoMotion: return "zone planner, no planar motion";
    case Outcome::OutsideBed: return "zone planner fallback, endpoint outside the bed";
    case Outcome::Exhausted: return "zone planner fallback, no admissible route";
    case Outcome::Routed: break;
    }
    std::string text = "zone route; start: " +
        describe_attachment(decision.source_island, decision.source_in_obstacle, decision.source_obstacle_ignored,
                            decision.source_exit, decision.source_link) +
        "; end: " +
        describe_attachment(decision.target_island, decision.target_in_obstacle, decision.target_obstacle_ignored,
                            decision.target_exit, decision.target_link);
    if (decision.bridge) text += "; inter-island bridge";
    if (decision.outside_part) text += "; endpoint outside the part";
    return text;
}

std::string describe_attachment(const bool island, const bool in_obstacle, const bool obstacle_ignored,
                                const bool exit, const bool link)
{
    std::string text = island ? "island ring" : "platter";
    if (in_obstacle)
        text += obstacle_ignored ? ", inside an obstacle crossed by the route" : ", inside an obstacle";
    if (exit) text += ", straight exit from the obstacle";
    if (link) text += ", link from the island band to its ring";
    return text;
}

std::string describe_no_travel(const NoTravel::Counts &before, const NoTravel::Counts &after, const size_t points)
{
    std::string text = after.exemptions != before.exemptions ? "endpoint in a NoTravel volume, volume ignored; " : "";
    if (after.fallbacks != before.fallbacks) return text + "NoTravel detour rejected, straight";
    return text + (points > 2 ? "NoTravel detour" : "straight");
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
