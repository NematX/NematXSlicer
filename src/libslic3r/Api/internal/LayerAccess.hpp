///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_internal_LayerAccess_hpp_
#define slic3r_Api_internal_LayerAccess_hpp_

#include <vector>

#include "libslic3r/libslic3r.h"

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
    // The height of a layer must be above this (scaled units), as the constructor of Layer requires.
    static constexpr coord_t min_scaled_height = 100;
    // Moves a layer to a new print z and height, both in scaled layer units (print_z on the layer grid, height > min_scaled_height), and slice_z (mm)
    // to its middle; the id is kept. The caller restores the order of the layer container.
    static void set_z(Layer &layer, coord_t print_z, coord_t height);
};

} // namespace ApiInternal

} // namespace Slic3r


#endif // slic3r_Api_internal_LayerAccess_hpp_
