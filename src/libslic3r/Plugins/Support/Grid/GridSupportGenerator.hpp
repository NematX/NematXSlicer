///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* The generator turns per-island demand into temporary contacts and columns.
   It projects contacts down, places slabs along Z, trims them against the
   object and separates dense interfaces from sparse base. The caller publishes
   these slabs through the data-tree API; no extrusion paths are generated. */
#ifndef plugins_support_grid_generator_hpp_
#define plugins_support_grid_generator_hpp_
#include "GridSupportLayers.hpp"
#include "GridSupportParams.hpp"

namespace Slic3r::SupportGrid {
struct GeneratedSupport {
    LayerStorage storage;
    GeneratorLayers top, bottom, base, interfaces;
};
// Populate temporary slabs for the sliced object's demand, leaving publication to the caller.
void generate_support(const PrintObject &object, const GridSupportParams &params, GeneratedSupport &result);
}
#endif
