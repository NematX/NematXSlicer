///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

/*
ZonePlanner supplies platter-space XY routes for one worker. Construct it with the
borrowed compatibility planner, dated zones, print plan, layer cache and run context;
pass Print::config() as its print settings and resolve auxiliary_avoidance from
print_get_default_region_config(). This flag governs travels whose endpoints have no
valid object instance, such as skirt and brim. Call the static eligible(...) guard
before constructing TravelZoneView and ZonePlanner, then call path(T, travel) in
nondecreasing T order.
Attractors publish original slices. The planner subtracts expanded forbidden material
to form M, withdraws each M component into W rings, and caches M/W plus optional
printed platter components by active zones, tool, endpoint exclusions and spacing.
Endpoint provenance selects an attractor component; regionless and auxiliary endpoints
use only the obstacle-only choice. Candidate pairs are ranked by typed inside/outside
estimates and built with their selected attachment points. Construction retains exact
endpoints; validation checks original-material crossings, the complete-path detour
budget and the obstacle-only reference bound. If no candidate passes, path returns and
counts the straight line.
*/
#ifndef slic3r_plugins_advanced_zone_planner_hpp_
#define slic3r_plugins_advanced_zone_planner_hpp_

#include <map>
#include <optional>
#include <tuple>

#include "TravelZones.hpp"
#include "../AvoidCrossing/CompatibilityPlanner.hpp"
#include "../AvoidCrossing/TravelRouter.hpp"

namespace slic3r_api { namespace LayerExtrusionEdit {
namespace AvoidCrossing { class LayerInputCache; }
namespace AdvancedTravel {

class ZonePlanner
{
public:
    // Copy the Config view, the auxiliary flag and layer-group identity/Z; borrow its configuration, the print, plan, worker-local planners, cache, zones and run context.
    // Borrowed dependencies outlive this planner; the plan catalog and object slices stay unchanged while cached platter blockers are used.
    ZonePlanner(AvoidCrossing::CompatibilityPlanner &compatibility, TravelZoneView &zones,
                const Config &print_config, bool auxiliary_avoidance, const plugin_run_context *context,
                const Print &print, AvoidCrossing::LayerInputCache &layer_inputs,
                const PrintingPlan &plan, uint32_t group_idx, coord_t print_z);
    // Evaluate eligibility without constructing zones; settings and planner are borrowed for this call.
    static bool eligible(const AvoidCrossing::CompatibilityPlanner &compatibility, const Config &print_config,
                         bool auxiliary_avoidance, const TravelConnection::TravelEndpoint &source,
                         const TravelConnection::TravelEndpoint &target);
    // Check regional activation only for valid object instances; other endpoints do not veto, and object targets respect first-layer exclusion.
    bool eligible(const TravelConnection::TravelEndpoint &source,
                  const TravelConnection::TravelEndpoint &target) const;
    // Return the first ranked route whose construction retains exact endpoints and that passes material, budget and reference-bound checks.
    // A direct result, a domain route or the counted straight fallback may be returned.
    // Cancellation and geometry errors propagate.
    std::vector<c_point> path(size_t t, const PendingTravel &travel);
    // Count accepted paths constructed with at least one component's attachment or ring route, including interior chords.
    size_t domain_routes() const { return m_domain_routes; }
    // Count accepted paths connecting two distinct components through selected bridge points.
    size_t bridges() const { return m_bridges; }
    // Return cumulative ring-routing diagnostics; advanced routing does not update island_jumps.
    const AvoidCrossing::RouterCounts &router_counts() const { return m_router_counts; }
    // Return the number of accepted domain or obstacle paths longer than their chord by more than SCALED_EPSILON.
    size_t detours() const { return m_detours; }
    // Return the number of straight fallbacks after all ranked hypotheses fail.
    size_t fallbacks() const { return m_fallbacks; }

private:
    struct Obstacles {
        polygon_work::ExPolygons original;
        polygon_work::ExPolygons expanded;
        // Unioned attractor components after subtracting expanded forbidden material.
        polygon_work::ExPolygons components;
        // One W component per M component, used only for ring routing.
        polygon_work::ExPolygons withdrawn_components;
        // Every active raw attractor index contributing filled area to each M component.
        std::vector<std::vector<size_t>> component_sources;
        // Independently indexed W rings for each component, in components order.
        std::vector<std::unique_ptr<AvoidCrossing::Boundary>> component_boundaries;
        // First ring index of each component in the combined domain boundary.
        std::vector<size_t> component_rings;
        std::unique_ptr<AvoidCrossing::Boundary> domain;
    };
    // Present only for travels using slices of the immutable plan's objects; retain group, Z and ignored component identities.
    using PlatterKey = std::tuple<uint32_t, coord_t, std::vector<size_t>>;
    // Cache entries use active zones, tool, endpoint exclusions, spacing and optional platter context.
    using CacheKey = std::tuple<size_t, extruder_index_t, std::vector<size_t>, coord_t, std::optional<PlatterKey>>;
    AvoidCrossing::CompatibilityPlanner &m_compatibility;
    TravelZoneView &m_zones;
    Config m_config;
    bool m_auxiliary_avoidance;
    const plugin_run_context *m_context;
    const Print &m_print;
    AvoidCrossing::LayerInputCache &m_layer_inputs;
    // One immutable plan/group/Z geometry context per worker callback.
    const PrintingPlan &m_plan;
    uint32_t m_group_idx;
    coord_t m_print_z;
    // Original object-slice components are collected once and shared by the tool-specific expanded cache entries.
    bool m_platter_ready = false;
    polygon_work::ExPolygons m_platter_blockers;
    std::map<CacheKey, Obstacles> m_obstacles;
    size_t m_domain_routes = 0;
    size_t m_bridges = 0;
    AvoidCrossing::RouterCounts m_router_counts;
    size_t m_detours = 0;
    size_t m_fallbacks = 0;
};

}}}
#endif
