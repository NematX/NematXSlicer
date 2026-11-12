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
and spacing. Domain routes are validated against original forbidden material;
blocked routes use obstacle routing. If obstacle routing stays straight without
failure and its chord passes validation against every remaining original forbidden
area, that chord is accepted without counting a fallback or a domain route.
A chord inside only the nozzle margin may still require a detour or fallback.
Failed obstacle routing or an excessive detour instead
returns a counted straight fallback.
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
    // Return exact endpoints; a direct result may be a valid domain or obstacle route, a Z-only move, or a counted fallback.
    // This includes a clear straight chord accepted after a rejected domain path.
    // Cancellation and geometry errors propagate.
    std::vector<c_point> path(size_t t, const PendingTravel &travel);
    // Return the number of accepted domain paths, including direct segments.
    size_t domain_routes() const { return m_domain_routes; }
    // Return cumulative ring-routing diagnostics for this worker.
    const AvoidCrossing::RouterCounts &router_counts() const { return m_router_counts; }
    // Return the number of accepted domain or obstacle paths longer than their chord by more than SCALED_EPSILON.
    size_t detours() const { return m_detours; }
    // Return the number of straight fallbacks after failed obstacle routing or a rejected detour budget.
    size_t fallbacks() const { return m_fallbacks; }

private:
    struct Obstacles {
        polygon_work::ExPolygons original;
        polygon_work::ExPolygons expanded;
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
    AvoidCrossing::RouterCounts m_router_counts;
    size_t m_detours = 0;
    size_t m_fallbacks = 0;
};

}}}
#endif
