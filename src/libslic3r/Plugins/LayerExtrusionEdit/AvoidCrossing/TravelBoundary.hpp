///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

#ifndef slic3r_plugins_avoid_crossing_travel_boundary_hpp_
#define slic3r_plugins_avoid_crossing_travel_boundary_hpp_

/*
Travel boundaries for one layer and one tool
==========================================
TravelRouter (TravelRouter.hpp) owns one TravelBoundary to obtain rings along
which it can detour instead of crossing perimeters. The travel provider in
AvoidCrossingPerimetersTravel.cpp keeps
one router per layer and tool. This component builds geometry, not travel paths.
The internal boundary follows inner perimeters in one object's coordinates.
The external boundary follows holes of all objects, placed per instance in print
coordinates, for travels between objects or instances. For support targets it also
includes holes of each object's last layer at or below the target height. TravelBoundary.cpp explains
their construction and the exact correspondence with the core implementation.

Call init_layer once with the target layer copied by AvoidCrossingInput.hpp.
On the first route needing a boundary, call build_internal with a zero-based tool
or build_external with the external input (AvoidCrossingInput.hpp). Read the results
through internal_boundary() and external_boundary(). The non-const internal accessor
lets routing lazily add a grid for the ring selected for an island jump: its contour
or the hole enclosing the other island, keyed by its index in boundaries.
Each grid is created when a jump first uses that ring.
The non-const external accessor gives routing its geometry.
Boundary owns its rings and excluded top areas; grids borrow their point arrays.
These arrays must not change after indexing. Neither Boundary nor TravelBoundary
can be copied or moved, so their owners must keep them at a stable address.
For caller-supplied geometry, build_boundary fills a stable Boundary directly;
route_on_boundary in TravelRouter.hpp routes in the same coordinate system.
*/
#include <map>

#include "AvoidCrossingInput.hpp"
#include "libslic3r/Plugins/Common/PluginEdgeGrid.hpp"

namespace slic3r_api { namespace LayerExtrusionEdit { namespace AvoidCrossing {

struct Boundary
{
    // Boundary rings; internally, each island's contour is followed by its holes.
    PluginCommon::polygon_work::Polygons boundaries;
    // For each ring, its island's contour index in boundaries; empty externally.
    std::vector<size_t> islands;
    // Box of all rings, padded by the supplied extra space plus epsilon; TravelRouter::travel_to clips travels to it.
    PluginCommon::polygon_work::BoundingBoxf bbox;
    // Per-ring boxes, used to find which hole of an island encloses another island; empty externally.
    std::vector<PluginCommon::work::BoundingBox> bboxes;
    // Cumulative float length at each vertex, then the whole ring length, for choosing the shorter detour.
    std::vector<std::vector<float>> boundaries_params;
    // Edge grid over all rings, with 1 mm cells, for crossings and nearest points.
    PluginCommon::EdgeGrid::Grid grid;
    // Grid of the ring selected for an island jump: its contour or the hole enclosing the other island.
    // Keyed by its index in boundaries; created when a jump first uses that ring.
    std::map<int, PluginCommon::EdgeGrid::Grid> island_to_grid;
    // Original island/inset pairs before top removal and tool clipping; empty externally.
    // Domain callers fill {component, component} to keep interior endpoints in place.
    // Routing tests inset containment and uses the original island to find an inset edge beyond grid reach.
    std::vector<std::pair<PluginCommon::polygon_work::ExPolygon, PluginCommon::polygon_work::ExPolygon>> boundary_growth;
    // Unioned inset top areas, checked before shortcuts when an endpoint cannot reach a boundary.
    PluginCommon::polygon_work::ExPolygons to_avoid;
    // Edge grid borrowing to_avoid's contours and holes, with 1 mm cells.
    PluginCommon::EdgeGrid::Grid to_avoid_grid;

    // Create empty geometry and grids.
    Boundary() = default;
    // Disallow copying and moving: grids borrow this owner's point arrays at a stable address.
    Boundary(const Boundary &) = delete;
    Boundary &operator=(const Boundary &) = delete;
    Boundary(Boundary &&) = delete;
    Boundary &operator=(Boundary &&) = delete;
    // Release borrowed grids before clearing their owned geometry.
    void clear();
};

// Move islands into out and index their rings in the caller's coordinate system; pad boxes by extra_space plus epsilon.
// Preserve supplied out.to_avoid areas even for empty islands. Keep out alive during routing; storage is only borrowed for this call.
void build_boundary(storage_handle *storage, PluginCommon::polygon_work::ExPolygons &&islands,
                    coord_t extra_space, Boundary &out);

// Tests read these counts to identify reached inset branches; they never affect geometry.
// Both init_layer and build_internal reset them; build_internal then fills them.
struct InnerOffsetCounts
{
    // Count islands whose epsilon-grown bounding box has area below (0.1 mm)^2; keep the resampled candidate.
    size_t small = 0;
    // Count single-piece insets with unchanged hole count, and attempts retried with a wider threshold.
    // Final outcomes: one piece with changed holes, largest of several pieces, or no piece (keep pre-inset island).
    size_t same_holes = 0, retry = 0, changed_holes = 0, multiple = 0, empty_original = 0;
};

class TravelBoundary
{
public:
    // Reset boundaries and counts; inset perimeter slices by half an external perimeter width, without prior union.
    void init_layer(storage_handle *storage, const LayerInput &layer);
    // Build and index internal insets for the zero-based tool, with top exclusions and tool clipping.
    // For support, use the object layer at or below the target.
    void build_internal(storage_handle *storage, const Input &input, int extruder);
    // Build and index holes of all objects at this height, placed per instance in print coordinates.
    // Support input also includes holes of each object's last layer at or below the target height.
    void build_external(storage_handle *storage, const LayerInput &layer, const ExternalInput &external);
    // Return internal geometry, empty until build_internal.
    const Boundary &internal_boundary() const { return m_internal; }
    // Borrow internal geometry so routing can lazily add selected-ring grids keyed by their index in boundaries.
    Boundary &internal_boundary() { return m_internal; }
    // Return external geometry, empty until build_external.
    const Boundary &external_boundary() const { return m_external; }
    // Borrow external geometry for routing queries.
    Boundary &external_boundary() { return m_external; }
    // Return init_layer's inset; routing only tests emptiness to leave travels direct when no interior exists (except support).
    const PluginCommon::polygon_work::ExPolygons &layer_slices_offset() const { return m_lslices_offset; }
    // Return inset branch counts for diagnostics without changing geometry construction.
    const InnerOffsetCounts &inner_offset_counts() const { return m_counts; }

private:
    Boundary m_internal;
    Boundary m_external;
    PluginCommon::polygon_work::ExPolygons m_lslices_offset;
    InnerOffsetCounts m_counts;
};

}}}
#endif
