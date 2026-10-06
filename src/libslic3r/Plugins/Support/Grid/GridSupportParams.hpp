///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* Support parameters translate object, print and region settings into geometry.
   Z distances use millimetres until converted to the integer layer-Z grid;
   XY geometry uses scaled coordinates. A zero-distance interface is soluble.
   No raft parameters or slicer-owned parameter bundle are used here. */
#ifndef plugins_support_grid_params_hpp_
#define plugins_support_grid_params_hpp_
#include "libslic3r/Flow.hpp"
#include "libslic3r/Config/FFFPrintConfig.hpp"

namespace Slic3r { class PrintObject; }
namespace Slic3r::SupportGrid {
struct GridSupportParams {
    // Derive contact gaps, layer limits and geometry flows from the object's settings.
    explicit GridSupportParams(const PrintObject &object);
    double first_layer_height = 0, layer_height = 0, max_support_height = 0;
    double gap_above = 0, gap_below = 0, z_step = 0;
    bool soluble = false;
    size_t top_interfaces = 0, bottom_interfaces = 0;
    Flow first_flow, base_flow, interface_flow, bottom_flow;
    FloatOrPercent bottom_expansion;
    coord_t min_height = 0, gap_xy = 0, resolution = SCALED_EPSILON;
    double interface_density = 0;
};
}
#endif
