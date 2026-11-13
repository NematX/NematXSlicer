// Crossing tests for callers deciding whether to retract or request a travel detour.
// Prepare base and offset area boundaries, then query paths in the target's local
// coordinates. For objects, when any region assigns a perimeter or infill role to
// another tool, preparation clips the slices to the selected tool's area.
// Objects use an inset of 1.5 nozzle radii in the offset version and, by default,
// no tops in either version. Support uses its own slices and
// an expansion of 1.5 radii, with neither tool clipping nor top removal; already
// printed object layers are not included. Both versions are simplified by radius.
// A false result is not proof of containment: paths wholly outside or in a hole
// can miss all boundaries. This class does not route or insert travels.
// Keep the immutable LayerInput alive until preparing another input. The cache key
// is its address, the tool and radius; edits at that address do not refresh it.
// Each worker owns an instance because queries populate cached hole boxes.
// Example (storage, input and path already supplied by the caller):
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
        std::vector<SliceIsland> slices;
        std::vector<SliceIsland> offset_slices;
    } m_cache;
    std::function<void()> m_cancel;
public:
    // Cache simplified interiors for this layer, zero-based tool and scaled nozzle radius.
    // Travel routing removes object tops, which crossing could mark; a retraction filter
    // keeps them to test only real perimeter contours. Support considers the target
    // layer's own geometry. cancel may throw; it runs once per object region before top
    // subtraction during preparation and per queried hole.
    void prepare(storage_handle *storage, const LayerInput &layer, size_t extruder,
                 coord_t radius, std::function<void()> cancel = {}, bool exclude_top_surfaces = true);
    // Return whether an object-local path crosses an outer contour or hole of the prepared interiors.
    bool can_cross_perimeter(const PluginCommon::work::Polyline &travel, bool offset);
};

}}}
#endif
