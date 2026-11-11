///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

/*
ZonePlanner supplies platter-space XY routes to travel providers. Construct one
per worker with a compatibility planner, dated zone view, a copied Config view
and a live run context. Pass Print::config() for print settings and resolve the
auxiliary avoidance flag from print_get_default_region_config(). The flag governs
travels whose two endpoints have no valid object instance, such as skirt and brim.
Config borrows its referenced
configuration; that configuration must outlive the planner.
Call static eligible(compatibility, config, auxiliary_avoidance, source, target)
for summarize(), then build TravelZoneView and ZonePlanner, then call path(T, travel)
in nondecreasing T order. Instance eligible(source, target) uses the same test
with the planner's settings. Dependencies and context storage must outlive this object.
Attractors select compatibility candidates; forbidden material takes precedence.
The planner owns expanded geometry cached by active set, tool and ignored zones.
Counters describe completed delegations, obstacle detours and straight fallbacks.
*/
#ifndef slic3r_plugins_advanced_zone_planner_hpp_
#define slic3r_plugins_advanced_zone_planner_hpp_

#include <map>
#include <tuple>

#include "TravelZones.hpp"
#include "../AvoidCrossing/CompatibilityPlanner.hpp"

namespace slic3r_api { namespace LayerExtrusionEdit { namespace AdvancedTravel {

class ZonePlanner
{
public:
    // Copy the print configuration view and resolved auxiliary flag; borrow worker-local planners, zones and cancellation storage.
    ZonePlanner(AvoidCrossing::CompatibilityPlanner &compatibility, TravelZoneView &zones,
                const Config &print_config, bool auxiliary_avoidance, const plugin_run_context *context);
    // Evaluate eligibility without constructing zones; settings and planner are borrowed for this call.
    static bool eligible(const AvoidCrossing::CompatibilityPlanner &compatibility, const Config &print_config,
                         bool auxiliary_avoidance, const TravelConnection::TravelEndpoint &source,
                         const TravelConnection::TravelEndpoint &target);
    // Check regional activation only for valid object instances; other endpoints do not veto, and object targets respect first-layer exclusion.
    bool eligible(const TravelConnection::TravelEndpoint &source,
                  const TravelConnection::TravelEndpoint &target) const;
    // Return exact endpoints and an accepted XY route, or a direct fallback; cancellation and geometry errors propagate.
    std::vector<c_point> path(size_t t, const PendingTravel &travel);
    // Return the number of compatibility candidates accepted without crossing forbidden material.
    size_t same_attractor_delegations() const { return m_delegations; }
    // Return the number of accepted obstacle routes.
    size_t detours() const { return m_detours; }
    // Return the number of obstacle routes rejected by validation, bounds or the detour budget.
    size_t fallbacks() const { return m_fallbacks; }

private:
    struct Obstacles {
        polygon_work::ExPolygons original;
        polygon_work::ExPolygons expanded;
    };
    using CacheKey = std::tuple<size_t, extruder_index_t, std::vector<size_t>>;
    AvoidCrossing::CompatibilityPlanner &m_compatibility;
    TravelZoneView &m_zones;
    Config m_config;
    bool m_auxiliary_avoidance;
    const plugin_run_context *m_context;
    std::map<CacheKey, Obstacles> m_obstacles;
    size_t m_delegations = 0;
    size_t m_detours = 0;
    size_t m_fallbacks = 0;
};

}}}
#endif
