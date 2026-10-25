///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_internal_LayerIslandAccess_hpp_
#define slic3r_Api_internal_LayerIslandAccess_hpp_

#include <vector>

namespace Slic3r {

class LayerSliceIsland;
class ExPolygon;
using ExPolygons = std::vector<ExPolygon>;

namespace ApiInternal {

struct LayerIslandAccess
{
    static ExPolygon &slice_mutable(LayerSliceIsland &island);
    // Sets the infill areas and their bounding boxes; transactional.
    static void set_infill_areas(LayerSliceIsland &island, ExPolygons &&infill_areas);
    static ExPolygons &infill_free_areas_mutable(LayerSliceIsland &island);
    static ExPolygons &perimeter_slices_mutable(LayerSliceIsland &island);
};

} // namespace ApiInternal

} // namespace Slic3r


#endif // slic3r_Api_internal_LayerIslandAccess_hpp_
