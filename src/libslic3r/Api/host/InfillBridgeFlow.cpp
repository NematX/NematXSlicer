///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
/* Calculate the round bridge thread's diameter and extrusion amount from the
   selected bridge mode and flow ratio. Infill generation and the C API share it.

   The role selects the nozzle. The configured mode starts from nozzle
   diameter, layer height, or the circular diameter equivalent to normal solid
   infill's area. Multiplying that diameter by sqrt(bridge_flow_ratio) scales
   the circular area by the requested ratio.

   Selected direct calls after reading those settings:

       infill_bridge_flow()
           |-- region.flow(frSolidInfill) [flow-based mode only]
           `-- Flow::bridging_flow() [construct the circular reference flow]

   The result is a Flow containing width, height, spacing and area per unit length.
   Dimensions are in mm, before conversion to integer coordinates. Layer, region
   and printer settings are read only for the call; none are changed or retained.
*/
#include "InfillBridgeFlow.hpp"
#include <cmath>
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/PrintRegion.hpp"

namespace Slic3r {
Flow infill_bridge_flow(const Layer &layer, const LayerRegion &region, FlowRole role)
{
    const PrintRegionConfig &config = region.region().config();
    const float nozzle = layer.object()->print()->config().nozzle_diameter.get_at(
        region.region().extruder(role, *layer.object()) - 1);
    double diameter = nozzle;
    // Flow-based mode takes the amount from normal solid infill, even when the
    // requested extrusion role selects a different nozzle.
    if (config.bridge_type == BridgeType::btFromFlow)
        diameter = std::sqrt(4. * region.flow(frSolidInfill).mm3_per_mm() / PI);
    else if (config.bridge_type == BridgeType::btFromHeight)
        diameter = layer.unscaled_height();
    return Flow::bridging_flow(float(diameter * std::sqrt(config.bridge_flow_ratio.get_effective_value(1))), nozzle);
}
}
