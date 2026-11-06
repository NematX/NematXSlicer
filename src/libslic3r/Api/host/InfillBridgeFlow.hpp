///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#ifndef slic3r_InfillBridgeFlow_hpp_
#define slic3r_InfillBridgeFlow_hpp_
/* Calculate the same bridge thread size for infill generation and the plugin API.

   Pass the owning layer, its region and the extrusion role. The role selects
   the nozzle. The configured bridge mode starts from nozzle diameter, layer
   height or the amount extruded by normal solid infill; bridge_flow_ratio then
   scales the circular cross-sectional area. Inputs are read without mutation.

   Example, in namespace Slic3r:
       const Flow bridge = infill_bridge_flow(layer, region, frSolidInfill);
*/
#include "libslic3r/Flow.hpp"
namespace Slic3r {
class Layer;
class LayerRegion;
// Return nozzle-, height- or flow-based bridge dimensions in mm before coordinate
// quantization. Requires a region of this layer and a configured tool for role.
Flow infill_bridge_flow(const Layer &layer, const LayerRegion &region, FlowRole role);
}
#endif
