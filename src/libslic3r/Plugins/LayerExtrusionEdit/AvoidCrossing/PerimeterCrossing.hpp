// Crossing tests for callers deciding whether to retract or request a travel detour.
// Prepare base and offset area boundaries, then query paths in the target's local
// coordinates. For objects, when any region assigns a perimeter or infill role to
// another tool, preparation clips the slices to the selected tool's area.
// Objects use an inset of 1.5 nozzle radii in the offset version and, by default,
// no tops in either version. Support uses the last object areas below its Z plus epsilon,
// expanded by 1.5 radii, with neither tool clipping nor top removal. All catalog
// instances count, or only the target in a per-object plan. Empty areas cross nothing.
// Both versions are simplified by radius.
// A false result is not proof of containment: paths wholly outside or in a hole
// can miss all boundaries. This class does not route or insert travels.
// Keep the immutable LayerInput alive until preparing another input. The cache key
// is its address, the tool and radius, plus support target id, shift and plan mode;
// edits at that address do not refresh it. Keep support areas unchanged too.
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
        bool exclude_top_surfaces = true;
        uint32_t target_instance = 0;
        PluginCommon::work::Point target_shift;
        bool per_object = false;
        std::vector<SliceIsland> slices;
        std::vector<SliceIsland> offset_slices;
        // Object tops removed from both versions; empty when tops are kept.
        PluginCommon::polygon_work::ExPolygons top_surfaces;
    } m_cache;
    std::function<void()> m_cancel;
public:
    // Cache simplified interiors for this layer, zero-based tool and scaled nozzle radius.
    // Travel routing removes object tops, which crossing could mark; a retraction filter
    // keeps them to test only real perimeter contours.
    // Support areas are shifted object slices and are moved into target-local coordinates.
    // cancel may throw: object preparation calls it per region before top subtraction;
    // support preparation does not call it, and queries call it per examined hole.
    void prepare(storage_handle *storage, const LayerInput &layer, size_t extruder,
                 coord_t radius, std::function<void()> cancel = {},
                 const std::vector<SupportObjectInput> &support_areas = {},
                 uint32_t target_instance = 0, PluginCommon::work::Point target_shift = {},
                 bool per_object = false, bool exclude_top_surfaces = true);
    // Return whether an object-local path crosses an outer contour or hole of the prepared interiors.
    bool can_cross_perimeter(const PluginCommon::work::Polyline &travel, bool offset);
    // Return whether an object-local path runs over a removed top surface, even wholly inside it without
    // crossing a boundary. Always false when tops are kept or for support.
    bool touches_top_surface(storage_handle *storage, const PluginCommon::work::Polyline &travel) const;
};

}}}
#endif
