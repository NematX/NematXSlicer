///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_internal_LayerAccess_hpp_
#define slic3r_Api_internal_LayerAccess_hpp_

#include <vector>

namespace Slic3r {

class Layer;
class PrintRegion;
class ExPolygon;
using ExPolygons = std::vector<ExPolygon>;

namespace ApiInternal {

struct LayerAccess
{
    static ExPolygons &slices_mutable(Layer &layer);
    // Replaces the layer slices and rebuilds the islands; transactional, the source is moved only on success.
    static void set_islands(Layer &layer, ExPolygons &&new_islands);
    // Rebuilds the layer slices cache from the islands' slices; transactional.
    static void recompute_slices_from_islands(Layer &layer);
    static void recompute_slices_from_layer_regions(Layer &layer);
    static void init_regions_from_object(Layer &layer);
    static void add_region(Layer &layer, const PrintRegion &region);
};

} // namespace ApiInternal

} // namespace Slic3r


#endif // slic3r_Api_internal_LayerAccess_hpp_
