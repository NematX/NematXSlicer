// Crossing tests for callers deciding whether to retract or request a travel detour.
// Prepare base and offset area boundaries, then query paths in the target's local
// coordinates, or prepare_platter() and query in platter coordinates. For objects, when any region assigns a perimeter or infill role to
// another tool, preparation clips the slices to the selected tool's area.
// Objects use an inset of 1.5 nozzle radii in the offset version and, by default,
// no tops in either version. Platter queries use the last normal object areas below Z plus epsilon,
// expanded by 1.5 radii, with neither tool clipping nor top removal. Real instances
// in current and earlier printing groups count. Empty areas cross nothing.
// Both versions are simplified by radius.
// A false result is not proof of containment: paths wholly outside or in a hole
// can miss all boundaries. This class does not route or insert travels.
// Keep the immutable LayerInput alive until preparing another input. The cache key
// is its address, tool and radius; platter keys are group, Z and radius.
// Keep the plan unchanged too: edits do not refresh cached geometry.
// Each worker owns an instance because queries populate cached hole boxes.
// Example for an object target (storage, input and path already supplied by the caller):
//   PerimeterCrossing crossing;
//   crossing.prepare(storage, *input.target, 0, radius);
//   bool crosses = crossing.can_cross_perimeter(path, true);
#ifndef slic3r_plugins_perimeter_crossing_hpp_
#define slic3r_plugins_perimeter_crossing_hpp_

#include <functional>
#include "AvoidCrossingInput.hpp"

namespace slic3r_api { namespace LayerExtrusionEdit { namespace AvoidCrossing {

class PerimeterCrossing
{
    using ExPolygon = PluginCommon::polygon_work::ExPolygon;
    using BoundingBox = PluginCommon::work::BoundingBox;
    struct SliceIsland {
        ExPolygon expolygon;
        BoundingBox boundingbox;
        std::vector<BoundingBox> hole_boundingboxes;
    };
    struct CrossingCache {
        const LayerInput *layer = nullptr;
        size_t extruder = 0;
        coord_t radius = 0;
        bool prepared = false;
        bool exclude_top_surfaces = true;
        uint32_t group_idx = 0;
        coord_t print_z = 0;
        std::vector<SliceIsland> slices;
        std::vector<SliceIsland> offset_slices;
        // Object tops removed from both versions; empty when tops are kept.
        PluginCommon::polygon_work::ExPolygons top_surfaces;
    } m_cache;
    std::function<void()> m_cancel;
    // Return owned simplified rings with their original contour boxes; geometry failures propagate.
    std::vector<SliceIsland> slice_islands(storage_handle *storage,
        const PluginCommon::polygon_work::ExPolygons &slices, coord_t radius);
public:
    // Union and tool-clip normal object slices, remove tops unless exclude_top_surfaces is false, and cache
    // simplified base and inset rings for this zero-based tool and scaled radius. Travel routing removes object
    // tops, which crossing could mark; a retraction filter keeps them to test only real perimeter contours.
    // cancel may throw: preparation calls it per region before top subtraction, queries per examined hole.
    void prepare(storage_handle *storage, const LayerInput &layer, size_t extruder,
                 coord_t radius, std::function<void()> cancel = {}, bool exclude_top_surfaces = true);
    // Collect platter blockers only on a group/Z/radius miss and cache base and expanded rings; geometry failures propagate.
    // Retain cancel for queries even on a cache hit; collection does not invoke it.
    void prepare_platter(storage_handle *storage, const PrintingPlan &plan,
        uint32_t group_idx, coord_t print_z, coord_t radius, std::function<void()> cancel = {});
    // Return whether a path in prepared coordinates crosses an outer contour or hole; invoke cancellation per examined hole.
    bool can_cross_perimeter(const PluginCommon::work::Polyline &travel, bool offset);
    // Return whether an object-local path runs over a removed top surface, even wholly inside it without
    // crossing a boundary. Always false when tops are kept or for support.
    bool touches_top_surface(storage_handle *storage, const PluginCommon::work::Polyline &travel) const;
};

}}}
#endif
