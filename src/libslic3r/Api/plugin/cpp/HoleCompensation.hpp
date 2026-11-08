///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_plugin_cpp_HoleCompensation_hpp_
#define slic3r_Api_plugin_cpp_HoleCompensation_hpp_

#include <cstddef>
#include <functional>
#include <vector>

#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"

/*
Regional compensation of convex holes
=====================================

compensate_convex_holes() moves the walls of the convex holes of one layer.
The caller decides how far, per region and per hole, through a function; the
rules shared by every hole compensation plugin are implemented here once:

- the holes are those of the merged material of all regions (a region that
  only surrounds another region does not make a hole);
- only convex holes are changed (see is_convex_hole() in HoleGeometry.hpp);
- a region acts on a hole only when its material forms part of the hole wall;
- each region moves its own part of the wall with its own distance:
  - negative distance (enlarge the hole): only that region's material is
    removed, so the wall can show a step where two regions meet;
  - positive distance (shrink the hole): the region receives the new
    material next to its part of the wall; where two regions could claim the
    same material, the first region in order keeps it;
- islands inside a hole are never touched.

Used by the built-in slicing.hole_size_compensation plugin and by external
plugins (for example a compensation by hole diameter), at STEP_POST_SLICING.

    bool changed = false;
    auto output = compensate_convex_holes(storage, region_slices,
        [](std::size_t region, const ConvexHoleInfo &hole) { return coord_t(0); }, changed);

The callback runs synchronously and its hole view must not escape the call.
Input slices stay borrowed and unchanged. Returned geometry belongs to storage,
which must outlive the output views. Concurrent calls require stable input
views and a callback safe for the caller's thread use.
*/

namespace slic3r_api {

// What the distance function knows about a hole.
struct ConvexHoleInfo
{
    // The hole ring, in the clockwise orientation of ExPolygon holes.
    Polygon hole;
    // Area of the hole, in scaled units squared (always positive).
    double area = 0.;
    // approximate_hole_diameter() of the hole, in scaled units.
    double diameter = 0.;
};

// Distance by which `region` moves its part of the wall of a convex hole, in
// scaled units. Negative enlarges the hole, positive shrinks it, zero leaves
// that region's part unchanged. Called only for regions with material.
using ConvexHoleDelta = std::function<coord_t(std::size_t region, const ConvexHoleInfo &hole)>;

// region_slices[i] are the slices of region i on one layer (the regions do not
// overlap). Returns the new slices of every region, in the same order, and
// sets `changed` to false when no region was modified (the result is then a
// copy). All geometry is allocated in `storage`.
// A null storage or empty callback throws invalid_argument. Callback, allocation
// and geometry errors propagate; no host slices are changed on failure.
std::vector<StoredExPolygonCollection> compensate_convex_holes(storage_handle *storage,
                                                               const std::vector<ExPolygonCollection> &region_slices,
                                                               const ConvexHoleDelta &delta,
                                                               bool &changed);

} // namespace slic3r_api

#endif // slic3r_Api_plugin_cpp_HoleCompensation_hpp_
