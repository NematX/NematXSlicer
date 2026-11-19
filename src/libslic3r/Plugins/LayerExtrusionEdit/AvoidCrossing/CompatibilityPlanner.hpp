///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

/*
CompatibilityPlanner answers the regional avoid_crossing_perimeters questions of
travel and retraction planning. Construct one planner per worker callback with the
unchanged print, plan and a non-null run context with valid plugin storage. The
borrowed plan and run context must outlive the planner; its copied inputs and
crossing caches are worker-local.
avoidance_enabled() checks both endpoints' regional activation and valid object
instances, then the target's first-layer exclusion, without building geometry.
Advanced travel's zone planner uses the endpoint checks for its own eligibility.
Retraction planning asks crosses() whether a route crosses a perimeter contour:
    CompatibilityPlanner planner(print, plan, run_ctx, group_idx, print_z);
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
#include "../TravelConnectionHelpers.hpp"

namespace slic3r_api { namespace LayerExtrusionEdit { namespace AvoidCrossing {

class CompatibilityPlanner
{
public:
    // Borrow print, plan and context; group_idx and print_z identify the current printing group and layer-group Z.
    CompatibilityPlanner(const Print &print, const PrintingPlan &plan, const plugin_run_context *run_ctx,
                         uint32_t group_idx, coord_t print_z);

    // Report the avoidance settings shared by advanced travel and retraction planning. Call with null to
    // get the count, then supply a caller-owned array of that size; copied entries borrow static key strings.
    static int32_t used_config_keys(raw_used_config_key *keys) noexcept;

    // Check regional activation at both endpoints, valid instances and the target's first-layer exclusion.
    bool avoidance_enabled(const TravelConnection::TravelEndpoint &source,
                           const TravelConnection::TravelEndpoint &target) const;

    // Return false without a valid region island, object instance and regional avoid_crossing_perimeters value; retry at the interior probe.
    bool endpoint_enables_avoidance(const TravelConnection::TravelEndpoint &endpoint) const;

    // Require a valid nonempty region island and a catalog entry naming an existing instance of that island's object.
    bool endpoint_has_object_instance(const TravelConnection::TravelEndpoint &endpoint) const;

    // Classify a print-space route against the perimeter contours of the target layer. With
    // exclude_top_surfaces, its tops count as exposed: a route over a top crosses even without
    // reaching a contour. Missing or different object provenance counts as a crossing.
    bool crosses(const TravelConnection::TravelEndpoint &source,
                 const TravelConnection::TravelEndpoint &target,
                 extruder_index_t extruder_id,
                 const std::vector<c_point> &path,
                 bool exclude_top_surfaces);

private:
    /* Identify one reusable crossing cache inside a layer-group worker. */
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

    /* Convert a final-plan point into the source object's local coordinates. */
    c_point local_point(c_point point, const TravelConnection::TravelEndpoint &endpoint) const;

    // Copy target data once after finding its normal or auxiliary index through public views; missing identities throw.
    const Input &input_for(const Layer &layer);

    Print m_print;
    Config m_print_config;
    const PrintingPlan &m_plan;
    const plugin_run_context *m_run_ctx = nullptr;
    uint32_t m_group_idx = 0;
    coord_t m_print_z = 0;
    LayerInputCache m_inputs;
    // Retraction filtering chooses whether tops are exposed, so it uses its own crossing caches
    // per layer, tool and top choice.
    std::map<std::pair<RouterKey, bool>, PerimeterCrossing> m_perimeter_crossings;
};

}}} // namespace slic3r_api::LayerExtrusionEdit::AvoidCrossing
#endif
