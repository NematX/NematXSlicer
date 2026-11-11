///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

/*
CompatibilityPlanner supplies perimeter-avoiding XY paths to travel providers.
Construct one planner per worker callback with the unchanged print, plan and a
non-null run context with valid plugin storage, then call path() for each gap.
The borrowed plan and run context must outlive the planner. Copied inputs outlive
the routers borrowing them; mutable routing and crossing caches are worker-local.
Travel providers need only the constructor and path(); retraction planning also asks
crosses() whether a route crosses a perimeter contour:
    CompatibilityPlanner planner(print, plan, run_ctx);
    std::vector<c_point> points = planner.path(source, target, extruder_id);
    bool retract = planner.crosses(source, target, extruder_id, points, false);
*/
#ifndef slic3r_plugins_compatibility_planner_hpp_
#define slic3r_plugins_compatibility_planner_hpp_

#include <functional>
#include <map>
#include <memory>
#include <vector>

#include "LayerInputCache.hpp"
#include "PerimeterCrossing.hpp"
#include "TravelRouter.hpp"
#include "../TravelConnectionHelpers.hpp"

namespace slic3r_api { namespace LayerExtrusionEdit { namespace AvoidCrossing {

class CompatibilityPlanner
{
public:
    // Retain borrowed views of the print and its settings, plus the plan and run context, during this worker callback.
    CompatibilityPlanner(const Print &print, const PrintingPlan &plan, const plugin_run_context *run_ctx);

    // Return print-space routing for an eligible gap, otherwise its direct segment; cancellation propagates.
    std::vector<c_point> path(const TravelConnection::TravelEndpoint &source,
                              const TravelConnection::TravelEndpoint &target,
                              extruder_index_t extruder_id);

    // Classify a print-space route against the perimeter contours of the target layer. With
    // exclude_top_surfaces, its tops count as exposed: a route over a top crosses even without
    // reaching a contour. Missing or different object provenance counts as a crossing.
    bool crosses(const TravelConnection::TravelEndpoint &source,
                 const TravelConnection::TravelEndpoint &target,
                 extruder_index_t extruder_id,
                 const std::vector<c_point> &path,
                 bool exclude_top_surfaces);

private:
    /* Identify one reusable router inside a layer-group worker. */
    struct RouterKey
    {
        const layer_handle *layer = nullptr;
        extruder_index_t extruder_id = EXTRUDER_ID_UNKNOWN;

        // Order borrowed public layer handles and zero-based tools for worker-local lookup.
        bool operator<(const RouterKey &rhs) const
        {
            return std::less<const layer_handle *>()(layer, rhs.layer) ||
                (layer == rhs.layer && extruder_id < rhs.extruder_id);
        }
    };

    // Own one layer/tool's router and crossing cache on the heap because TravelBoundary cannot move.
    struct RoutingState
    {
        TravelRouter router;
        PerimeterCrossing crossing;
    };

    // Resolve regional activation only for valid object instances, probing inside on a boundary.
    bool endpoint_enables_avoidance(const TravelConnection::TravelEndpoint &endpoint) const;

    /* Convert a final-plan point into the source object's local coordinates. */
    c_point local_point(c_point point, const TravelConnection::TravelEndpoint &endpoint) const;

    // Copy target data once after finding its normal or auxiliary index through public views; missing identities throw.
    const Input &input_for(const Layer &layer);

    // Initialize routing state for one target/tool, borrowing the cached input and run storage; failures propagate.
    RoutingState &router_for(const Layer &layer, const Input &input, extruder_index_t extruder_id);

    /* Return a straight path whenever perimeter avoidance cannot be used. */
    std::vector<c_point> straight(const TravelConnection::TravelEndpoint &source,
                                  const TravelConnection::TravelEndpoint &target,
                                  extruder_index_t extruder_id) const;

    Print m_print;
    Config m_print_config;
    const PrintingPlan &m_plan;
    const plugin_run_context *m_run_ctx = nullptr;
    LayerInputCache m_inputs;
    std::map<RouterKey, std::unique_ptr<RoutingState>> m_routers;
    // Retraction filtering chooses whether tops are exposed, so it uses its own crossing caches
    // per layer, tool and top choice.
    std::map<std::pair<RouterKey, bool>, PerimeterCrossing> m_perimeter_crossings;
};

}}} // namespace slic3r_api::LayerExtrusionEdit::AvoidCrossing
#endif
