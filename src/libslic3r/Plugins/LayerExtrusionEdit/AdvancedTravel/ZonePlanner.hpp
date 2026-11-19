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
Attractors publish original slices. The planner groups original forbidden and platter pieces
by overlap or contact and tries keeping or ignoring each group containing an
endpoint. Each combination caches the remaining union Z3 and its routing domains:
M_a and M_p subtract Z3 from the attractor union and bed contour, respectively;
W_a withdraws M_a and W_p subtracts the nozzle offset of Z3 from the bed contour.
An endpoint in a kept group can exit it by a straight attachment to W_a or W_p.
The planner checks exits against other kept obstacles and the rest against all of Z3,
shrinking those masks by SCALED_EPSILON so paths on the original border stay outside.
Walls thinner than twice that inset can disappear from the check. Domains, detours,
the reference and the score use the original obstacle geometry. The planner
then checks the assembled route against the expanded bed, budget and platter-only
reference. It compares the first admissible route from each combination by length,
counting length inside endpoint groups three times and keeping the first on ties.
It publishes only the winner's counts or returns and counts the straight fallback.
last_decision() describes how the latest path() call chose its result, for verbose
G-code annotations; it does not influence routing.
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

// Identify one original obstacle piece: the first flag is true for a platter piece
// and false for a forbidden zone, followed by its source and component indexes.
using ObstacleIdentity = std::tuple<bool, size_t, size_t>;

class ZonePlanner
{
public:
    // How one path() call produced its result.
    struct Decision {
        enum class Outcome {
            // Perimeter avoidance does not apply to this travel.
            Ineligible,
            // The endpoints coincide within SCALED_EPSILON.
            NoMotion,
            // An endpoint lies outside the configured bed contour.
            OutsideBed,
            // No combination produced an admissible route.
            Exhausted,
            // A route was admitted.
            Routed
        };
        Outcome outcome = Outcome::Ineligible;
        // Either endpoint is outside the part: no region, or support, raft, skirt, brim or tower geometry.
        bool outside_part = false;
        // The endpoint lies in a forbidden or platter obstacle group, and the admitted route ignored that group.
        bool source_in_obstacle = false, target_in_obstacle = false;
        bool source_obstacle_ignored = false, target_obstacle_ignored = false;
        // The admitted route attaches the endpoint to an island ring; otherwise to the free platter.
        bool source_island = false, target_island = false;
        // A straight exit leaves a kept obstacle group to reach the ring.
        bool source_exit = false, target_exit = false;
        // A link moves the endpoint from the island band, between the outline and its withdrawn ring, to that ring.
        bool source_link = false, target_link = false;
        // The admitted route joins two distinct island components.
        bool bridge = false;
    };

    // Copy the Config view, the auxiliary flag and layer-group identity/Z, and cache an expanded bed_shape copy for route containment; borrow its configuration, the print, plan, worker-local planners, cache, zones and run context.
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
    // Return the lowest-scoring admissible route across endpoint-group combinations, retaining exact endpoints.
    // When none passes, return and count the straight fallback.
    // Ineligible travels and travels without XY motion return straight without counting a fallback.
    // Cancellation and geometry errors propagate.
    std::vector<c_point> path(size_t t, const PendingTravel &travel);
    // Count accepted paths that use an attractor attachment or ring route; platter-only paths do not count.
    size_t domain_routes() const { return m_domain_routes; }
    // Count accepted paths joining distinct attractor components, including joins at a common point.
    size_t bridges() const { return m_bridges; }
    // Return the number of straight fallbacks when no admissible route was found.
    size_t fallbacks() const { return m_fallbacks; }
    // Describe the latest path() call; the value is replaced by the next call.
    const Decision &last_decision() const { return m_last; }

private:
    // The geometry of one obstacle piece and its stable identity for cache keys.
    struct OriginalObstacle {
        polygon_work::ExPolygon area;
        ObstacleIdentity identity;
    };
    // One admitted route with the measurements and diagnostics to publish if it wins.
    struct Route {
        polygon_work::Points points;
        double length;
        bool domain_route, bridge;
        // Attachment kind of each endpoint, as reported by Decision.
        bool source_island, target_island, source_exit, target_exit, source_link, target_link;
    };
    struct Obstacles {
        polygon_work::ExPolygons original;
        // Shrink original material by SCALED_EPSILON for crossing checks only.
        polygon_work::ExPolygons blocking;
        // Unioned attractor components after subtracting original Z3 material.
        polygon_work::ExPolygons components;
        // One W_a component per M_a component, used only for attractor ring routing.
        polygon_work::ExPolygons withdrawn_components;
        // Every active raw attractor index contributing filled area to each M_a component.
        std::vector<std::vector<size_t>> component_sources;
        // Independently indexed W_a rings for each component, in components order.
        std::vector<std::unique_ptr<AvoidCrossing::Boundary>> component_boundaries;
        // First ring index of each component in the combined domain boundary.
        std::vector<size_t> component_rings;
        std::unique_ptr<AvoidCrossing::Boundary> domain;
        // M_p is the initial bed contour minus Z3; W_p is the same contour minus Z4.
        polygon_work::ExPolygons platter_components;
        polygon_work::ExPolygons platter_free_components;
        // Map each W_p component to its overlapping M_p component, or SIZE_MAX if none overlaps.
        std::vector<size_t> platter_component_sources;
    };
    // Present only for travels using slices of the immutable plan's objects.
    using PlatterKey = std::tuple<uint32_t, coord_t>;
    // Cache entries use active zones, tool, ignored source components, spacing and platter context.
    using CacheKey = std::tuple<size_t, extruder_index_t, std::vector<ObstacleIdentity>, coord_t, std::optional<PlatterKey>>;
    // Find the first admissible route for one set of ignored pieces without changing the published counters.
    std::optional<Route> route(const PendingTravel &pending, const ActiveZones &active,
        const std::vector<OriginalObstacle> &pieces, const std::vector<bool> &ignored,
        const std::vector<size_t> &source_group, const std::vector<size_t> &target_group,
        const std::optional<PlatterKey> &platter_key, coord_t spacing, double nozzle_diameter, double jump_weight);
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
    // Original object-slice components are collected once and shared by the cache entries that use them.
    bool m_platter_ready = false;
    polygon_work::ExPolygons m_platter_blockers;
    // Fixed bed_shape contour copied to scaled coordinates when this planner is constructed.
    polygon_work::ExPolygon m_platter_initial;
    // Fixed bed_shape contour expanded by SCALED_EPSILON for inside/outside checks.
    polygon_work::ExPolygons m_platter_expanded;
    std::map<CacheKey, Obstacles> m_obstacles;
    size_t m_domain_routes = 0;
    size_t m_bridges = 0;
    size_t m_fallbacks = 0;
    Decision m_last;
};

}}}
#endif
