///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

/*
ZonePlanner supplies platter-space XY routes to travel providers. Construct one
per worker with a compatibility planner, dated zone view, a copied Config view,
the borrowed Print, PrintingPlan and worker LayerInputCache, layer-group index/Z,
and a live run context. These
borrowed inputs must outlive the planner. Pass Print::config() for print settings and resolve the
auxiliary avoidance flag from print_get_default_region_config(). The flag governs
travels whose two endpoints have no valid object instance, such as skirt and brim.
Config borrows its referenced
configuration; that configuration must outlive the planner.
Call static eligible(compatibility, config, auxiliary_avoidance, source, target)
for the eligibility guard, then build TravelZoneView and ZonePlanner, then call path(T, travel)
in nondecreasing T order. Instance eligible(source, target) uses the same test
with the planner's settings. Dependencies and context storage must outlive this object.
Attractors publish original slices. The planner first subtracts expanded forbidden
material to form M, then withdraws each M component into one non-empty W used only
for ring routing. A component keeps the indexes of every source slice that
contributes filled area to it; endpoint provenance must match one of those sources.
An endpoint in M has physical membership in that component; a link is the
connector from that endpoint to its selected W point, built inside M.
The planner borrows the print and worker layer-input cache to resolve spacing and
island-jump settings. It owns M/W geometry cached by active set, tool, ignored zones
and spacing. Outside-part and mixed travels also use slices of the plan's objects
at the group Z or below, from groups up to the current one; the synthetic wipe
tower is not a blocker. Their containing endpoint components
are ignored before expansion; the cache additionally keys group, Z and ignored
platter components. The plan catalog and object slices remain immutable during use.
A normal object-layer endpoint offers its containing M component only when its
instance, source layer and original slice all match; otherwise it offers only the
option without a domain. An endpoint in M but outside W is connected to the nearest
W ring through the M rings. Regionless or auxiliary-layer endpoints (support, raft,
skirt, brim, tower) are outside the part and offer only the option without a domain.
Candidate pairs are
ranked by estimated outside length plus weighted inside length, with stable ties.
Inside estimates use weighted length within one component; other connectors use
clipped inside and outside portions across all M components. Every outside segment
uses the same outside-plus-weighted-inside formula, including an outside-part endpoint.
Same-component paths use that component alone. Paths between distinct components
reach selected bridge points within each one and detour around forbidden rings on the bridge.
Mixed choices use the nearest ring exit/entry to the free endpoint and detour around
rings on the remaining connector. Estimates and routes share selected points. The
option without domains follows expanded obstacle rings, including holes.
For in-part travels whose endpoints have the same M component, that option requires
its chord to stay in that component, with a SCALED_EPSILON outside-length allowance.
Every route is checked against original forbidden material, exact endpoints and
the complete-path detour budget. Its actual outside-domain length cannot exceed
the obstacle-only reference detour's outside length by more than SCALED_EPSILON;
when a source or target is strictly inside expanded forbidden material, that
reference bound is multiplied by three.
The reference is computed at most once per blocker set; when it crosses original material,
the option without a domain is unavailable and domain hypotheses are tried without the
outside-length bound. Exhausted hypotheses with no geometrically valid route disable
the first original component crossed along the chord, or the initial best-ranked estimate
when the chord is clear, rebuild geometry and retry. Budget and outside-length failures
alone do not authorize deactivation. Equal entry positions prefer zone and
component order before platter components. Each original component is disabled at most
once per travel, bounding deactivations by the component count crossed by the initial
chord or best estimate. Disabled components are excluded from
validation and retained in the cache identity. No crossed blocker or the retry bound
ends in one counted straight fallback. disabled_blockers() counts explicit removals only.
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
    // Return the first ranked route passing forbidden-material, endpoint and budget checks, plus the outside-length bound when a usable reference exists.
    // Geometrically exhausted hypotheses disable the first original component crossed by the chord, otherwise the initial best estimate, within a fixed crossing-count cap.
    // A direct result may be accepted, a Z-only move, or the counted fallback when no further crossed blocker can be removed.
    // Cancellation and geometry errors propagate.
    std::vector<c_point> path(size_t t, const PendingTravel &travel);
    // Count accepted paths constructed with at least one component's attachment or ring route, including interior chords.
    size_t domain_routes() const { return m_domain_routes; }
    // Count accepted paths connecting two distinct components through selected bridge points.
    size_t bridges() const { return m_bridges; }
    // Return cumulative ring-routing diagnostics; advanced routing does not update island_jumps. Use bridges() for accepted bridges.
    const AvoidCrossing::RouterCounts &router_counts() const { return m_router_counts; }
    // Return the number of accepted domain or obstacle paths longer than their chord by more than SCALED_EPSILON.
    size_t detours() const { return m_detours; }
    // Return the number of straight fallbacks after all ranked hypotheses and bounded blocker retries fail.
    size_t fallbacks() const { return m_fallbacks; }
    // Count original zone or platter components explicitly disabled after exhausted hypotheses; endpoint exclusions do not count.
    size_t disabled_blockers() const { return m_disabled_blockers; }

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
    using CacheKey = std::tuple<size_t, extruder_index_t, std::vector<size_t>, coord_t, std::optional<PlatterKey>, std::vector<std::pair<size_t, size_t>>>;
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
    size_t m_disabled_blockers = 0;
};

}}}
#endif
