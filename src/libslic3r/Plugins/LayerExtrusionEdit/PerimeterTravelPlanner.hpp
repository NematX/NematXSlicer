///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_Plugins_LayerExtrusionEdit_PerimeterTravelPlanner_hpp_
#define slic3r_Plugins_LayerExtrusionEdit_PerimeterTravelPlanner_hpp_

#include <memory>
#include "TravelConnectionHelpers.hpp"

/* Share the host's perimeter router between travel generation and retraction
   decisions. Construct one planner per layer worker, ask path() for the proposed
   route, then crosses() for its perimeter classification. The implementation
   owns its geometry caches and borrows immutable Print/PrintingPlan data only
   for the worker's lifetime. For example:

       PerimeterTravelPlanner planner(print, plan, context, group_idx, print_z);
       const auto route = planner.path(source, target, tool);
       const bool crossing = planner.crosses(source, target, tool, route);

   Missing or different object provenance is conservatively classified as a
   crossing; such travels must not lose their ordinary retraction. */
namespace slic3r_api { namespace LayerExtrusionEdit { namespace AvoidCrossingPerimetersTravelPlugin {

class PerimeterTravelPlanner
{
public:
    // Own worker-local routing caches for one plan group at its layer-group Z, which places the
    // objects already printed; the borrowed plan must outlive the planner.
    PerimeterTravelPlanner(const Print &, const PrintingPlan &, const plugin_run_context *,
                           uint32_t group_idx, coord_t print_z);
    // Release caches after the worker has finished its routing queries.
    ~PerimeterTravelPlanner();
    // Declare the settings read by the shared router and crossing caches.
    static int32_t used_config_keys(raw_used_config_key *) noexcept;
    // Select the same path as the built-in perimeter-avoiding travel provider.
    std::vector<c_point> path(const TravelConnection::TravelEndpoint &,
        const TravelConnection::TravelEndpoint &, extruder_index_t);
    // Classify the path, optionally treating top surfaces as exposed interior.
    bool crosses(const TravelConnection::TravelEndpoint &, const TravelConnection::TravelEndpoint &,
        extruder_index_t, const std::vector<c_point> &, bool exclude_top_surfaces = false);
private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

}}}
#endif
