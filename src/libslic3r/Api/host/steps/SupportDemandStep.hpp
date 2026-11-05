///|/ Copyright (c) SuperSlicer 2026 Durand R?mi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_host_steps_SupportDemandStep_hpp_
#define slic3r_Api_host_steps_SupportDemandStep_hpp_

/*
Host side of the support demand.

The support demand of a layer island is stored in the island itself
(LayerSliceIsland::support_demand()). STEP_SUPPORT_DEMAND builds it; plugins of
the steps up to STEP_SUPPORT refine it with the C function
support_demand_get_island_mutable(), which checks the running step; every step
reads it with layer_island_get_support_demand().
*/

#include "libslic3r/Api/plugin/c/steps/slic3r_step_support_demand.h"

namespace Slic3r {

class Print;

namespace ApiHost::Steps {

// Empty the support demand of every island of every object of print.
void clear_support_demand(Print &print);

} // namespace ApiHost::Steps
} // namespace Slic3r

#endif // slic3r_Api_host_steps_SupportDemandStep_hpp_
