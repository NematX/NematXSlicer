///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_internal_LayerRegionAccess_hpp_
#define slic3r_Api_internal_LayerRegionAccess_hpp_

#include <cstdint>
#include <vector>

namespace Slic3r {

class LayerRegion;
class LayerRegionIsland;
class PrintRegion;
class SurfaceCollection;
class ExPolygon;
using ExPolygons = std::vector<ExPolygon>;

namespace ApiInternal {

struct LayerRegionAccess
{
    // Replace an island-region's surfaces, emptying the source; null source clears, null destination returns zero.
    static int32_t set_fill_surfaces(LayerRegionIsland *island, SurfaceCollection *surfaces);
    // Host-only reassociation; caller guarantees registry ownership and invalidation.
    static void set_print_region(LayerRegion &layer_region, const PrintRegion &region);
    static ExPolygons &slices_mutable(LayerRegion &layer_region);
    static SurfaceCollection &surfaces_mutable(LayerRegion &layer_region);
    static ExPolygons &fill_no_overlap_expolygons_mutable(LayerRegion &layer_region);
};

} // namespace ApiInternal

} // namespace Slic3r


#endif // slic3r_Api_internal_LayerRegionAccess_hpp_
