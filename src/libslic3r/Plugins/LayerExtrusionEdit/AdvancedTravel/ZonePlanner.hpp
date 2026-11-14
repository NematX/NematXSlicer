///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

/*
ZonePlanner supplies platter-space XY routes to travel providers. Construct one
per worker with a compatibility planner, dated zone view, a copied Config view,
the borrowed Print and worker LayerInputCache, and a live run context. These
borrowed inputs must outlive the planner. Pass Print::config() for print settings and resolve the
auxiliary avoidance flag from print_get_default_region_config(). The flag governs
travels whose two endpoints have no valid object instance, such as skirt and brim.
Config borrows its referenced
configuration; that configuration must outlive the planner.
Call static eligible(compatibility, config, auxiliary_avoidance, source, target)
for summarize(), then build TravelZoneView and ZonePlanner, then call path(T, travel)
in nondecreasing T order. Instance eligible(source, target) uses the same test
with the planner's settings. Dependencies and context storage must outlive this object.
Attractors form a routing domain after expanded forbidden material is removed.
The planner borrows the print and worker layer-input cache to resolve spacing and
island-jump settings. It owns geometry cached by active set, tool, ignored zones
and spacing. Each endpoint offers its containing component (or nearest within
1.5 spacings) and an option without a domain. Outside endpoints attach at the
nearest ring point; contained endpoints remain unchanged. Candidate pairs are
ranked by estimated outside length plus weighted inside length, with stable ties.
Inside estimates use weighted chord length wholly inside their component;
endpoint attachments within 1.5 spacings are likewise priced as inside material;
other connectors use clipped inside and outside portions across all components.
Same-component paths use that component alone. Paths between distinct components
reach selected bridge points within each one and contour forbidden obstacles on the bridge.
Mixed choices use the nearest ring exit/entry to the free endpoint and contour
obstacles on the remaining connector. Estimates and routes share selected points.
The option without domains follows expanded obstacle contours.
If both endpoints have the same domain candidate, that option requires its chord to stay
within components grown by 1.5 spacings, with a SCALED_EPSILON outside-length allowance.
Every route is checked against original forbidden material, exact endpoints and
the complete-path detour budget. Its actual outside-domain length cannot exceed
the obstacle-only reference detour's outside length by more than SCALED_EPSILON.
The reference is computed at most once; an unavailable reference skips only the
outside-length bound. Rejection tries the next hypothesis, then counts one straight fallback.
*/
#ifndef slic3r_plugins_advanced_zone_planner_hpp_
#define slic3r_plugins_advanced_zone_planner_hpp_

#include <map>
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
    // Copy the Config view and the auxiliary flag; borrow its configuration, the print, worker-local planners, input cache, zones and cancellation storage.
    // The borrowed Print and worker LayerInputCache must outlive this planner, as must its other dependencies.
    ZonePlanner(AvoidCrossing::CompatibilityPlanner &compatibility, TravelZoneView &zones,
                const Config &print_config, bool auxiliary_avoidance, const plugin_run_context *context,
                const Print &print, AvoidCrossing::LayerInputCache &layer_inputs);
    // Evaluate eligibility without constructing zones; settings and planner are borrowed for this call.
    static bool eligible(const AvoidCrossing::CompatibilityPlanner &compatibility, const Config &print_config,
                         bool auxiliary_avoidance, const TravelConnection::TravelEndpoint &source,
                         const TravelConnection::TravelEndpoint &target);
    // Check regional activation only for valid object instances; other endpoints do not veto, and object targets respect first-layer exclusion.
    bool eligible(const TravelConnection::TravelEndpoint &source,
                  const TravelConnection::TravelEndpoint &target) const;
    // Return the first ranked route passing forbidden-material, endpoint and budget checks, plus the outside-length bound when a usable reference exists.
    // A direct result may be an accepted hypothesis, a Z-only move, or the counted fallback after all hypotheses fail.
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
    // Return the number of straight fallbacks after all ranked hypotheses fail validation or the budget.
    size_t fallbacks() const { return m_fallbacks; }

private:
    struct Obstacles {
        polygon_work::ExPolygons original;
        polygon_work::ExPolygons expanded;
        // Unioned attractor components after subtracting expanded forbidden material.
        polygon_work::ExPolygons components;
        // Components grown by 1.5 perimeter spacings, used to reject chords that leave the near-domain material proxy.
        polygon_work::ExPolygons material_proxy;
        // Independently indexed rings for each component, in components order.
        std::vector<std::unique_ptr<AvoidCrossing::Boundary>> component_boundaries;
        // First ring index of each component in the combined domain boundary.
        std::vector<size_t> component_rings;
        std::unique_ptr<AvoidCrossing::Boundary> domain;
    };
    using CacheKey = std::tuple<size_t, extruder_index_t, std::vector<size_t>, coord_t>;
    AvoidCrossing::CompatibilityPlanner &m_compatibility;
    TravelZoneView &m_zones;
    Config m_config;
    bool m_auxiliary_avoidance;
    const plugin_run_context *m_context;
    const Print &m_print;
    AvoidCrossing::LayerInputCache &m_layer_inputs;
    std::map<CacheKey, Obstacles> m_obstacles;
    size_t m_domain_routes = 0;
    size_t m_bridges = 0;
    AvoidCrossing::RouterCounts m_router_counts;
    size_t m_detours = 0;
    size_t m_fallbacks = 0;
};

}}}
#endif
