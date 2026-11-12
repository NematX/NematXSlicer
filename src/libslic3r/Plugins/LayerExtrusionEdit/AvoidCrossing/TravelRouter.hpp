///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#ifndef slic3r_plugins_avoid_crossing_travel_router_hpp_
#define slic3r_plugins_avoid_crossing_travel_router_hpp_

/*
TravelRouter supplies two-dimensional detours to plugin workers after callers
have decided that avoidance is active and routing is needed. Callers retain
responsibility for crossing tests, path insertion and Z motion. This heuristic
does not find a shortest path or guarantee that a returned path avoids every boundary.
A detour budget may replace any computed detour with the direct segment.
route_on_boundary also exposes the ring-following operation without layer input
or a detour budget. Use build_boundary first and pass endpoints in its coordinates.

TravelBoundary.hpp provides internal rings in object-local coordinates and
external rings in print coordinates. Internal rings follow insets of filled
object areas, with regional tops handled during boundary construction. External
rings are built from object holes placed at their instance shifts, not from the
outlines of outer obstacles. An initial interior
inset decides whether object routing has usable geometry. Support uses the object
insets supplied as additional input: its travel may remain straight or enter an
object inset; it does not route support travels around objects.

Both endpoints and all returned points use object-local coordinates. External
routing adds context.origin while planning, then subtracts it from every returned
point. init_layer builds the initial interior inset. Routing rings are built on
demand when the selected mode needs them; island grids are built only for island jumps.

Routing chooses the requested rings lazily, clips the working endpoints to their
bounding box when needed, and restores the requested endpoints afterward.
Internal routing first tries to place endpoints in the retained insets. Fallback
placements may compare how much of a proposed shortcut lies outside retained top
areas. Grid crossings and nearby projected points select where to follow rings;
the nearby projections are protected from ordinary shortcut removal. Between
islands, bridge scores combine bridge length with weighted endpoint approaches.
Ring direction compares float scores derived from accumulated distances after
subtracting four endpoint-to-vertex distances, choosing backward on a tie.
Simplification coalesces epsilon-near neighbours, even protected ones, then skips
unprotected points only when the shortcut misses the rings. The final extra
length is checked against the absolute-mm or percentage detour budget.

The caller keeps input and storage alive and unchanged during use. Each worker
owns a router for each layer/tool pair: the first internal boundary build selects
its tool-specific geometry. Callbacks are owned; their borrowed captures
must remain valid. External geometry is requested only when needed. Cancellation
and allocation failures propagate without returning a partial path.

Use init_layer before travel_to; use_external_mp chooses external rings. Endpoints
must be distinct under coincides_with_epsilon. For example, with caller-supplied
storage, input, print view, identity, context and end:
    TravelRouter router;
    router.init_layer(storage, input, [&] { return build_external_input(print, identity); });
    auto path = router.travel_to(context, end);
*/
#include <functional>
#include "TravelBoundary.hpp"

namespace slic3r_api { namespace LayerExtrusionEdit { namespace AvoidCrossing {

struct TravelContext
{
    // Scaled start position in the active object's coordinate system.
    PluginCommon::work::Point start;
    // Scaled instance shift from object coordinates to print coordinates.
    PluginCommon::work::Point origin;
    // Zero-based extruder index, fixed for this router's layer queries.
    int extruder_id = 0;
    // Maximum extra distance in millimetres, or a percentage of direct distance; nonpositive values disable the limit.
    double max_detour = 0.;
    // Interpret max_detour as a percentage rather than millimetres when true.
    bool max_detour_is_percent = false;
};

// Coverage counters reset by init_layer; deviations are counted before the detour limit is applied.
struct RouterCounts
{
    size_t deviated = 0, island_jumps = 0, boundary_fallback = 0;
    size_t to_avoid = 0, outside_bbox = 0, detour_rejected = 0;
    size_t arrival_epsilon = 0;
    size_t brute_force = 0;
    size_t epsilon_merge = 0;
};

// Return an owned path in boundary's coordinates that follows rings where the placed segment crosses or nears them;
// otherwise the path is a direct chord, as for empty geometry or failed placement.
// placement_spacing controls endpoint placement (zero disables it); segment_radius selects nearby edges.
// Distinct endpoints are required; island_jump enables weighted bridges. Update counts and lazy grids; cancellation propagates.
PluginCommon::work::Polyline route_on_boundary(Boundary &boundary, PluginCommon::work::Point start,
    PluginCommon::work::Point end, coord_t placement_spacing, coord_t segment_radius,
    bool island_jump, double jump_weight, storage_handle *storage, RouterCounts &counts,
    const std::function<void()> &cancel);

class TravelRouter
{
public:
    // Reset geometry and diagnostics, borrowing input and storage and owning the supplied callbacks.
    // Captured borrows must remain valid; callback failures propagate and external snapshots are requested lazily.
    void init_layer(storage_handle *storage, const Input &input,
                    std::function<ExternalInput()> external_input,
                    std::function<void()> throw_if_canceled = []() {});
    // Select print-space external boundaries or local internal boundaries for subsequent travels.
    void use_external_mp(bool use = true) { m_use_external_mp = use; }
    // Report whether init_layer has supplied the current immutable layer.
    bool is_init() const { return m_input != nullptr; }
    // Return an owned path from context.start to point, with an optional absolute-mm or percentage detour limit.
    // init_layer and endpoints distinct under coincides_with_epsilon are required; callback and allocation failures propagate.
    PluginCommon::work::Polyline travel_to(const TravelContext &context, const PluginCommon::work::Point &point);
    // Return cumulative branch counts since the most recent initialization.
    const RouterCounts &counts() const { return m_counts; }

private:
    storage_handle *m_storage = nullptr;
    const Input *m_input = nullptr;
    std::function<ExternalInput()> m_external_input;
    std::function<void()> m_throw_if_canceled;
    bool m_use_external_mp = false;
    TravelBoundary m_boundaries;
    RouterCounts m_counts;
};

}}}
#endif
