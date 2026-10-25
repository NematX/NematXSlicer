///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_Plugins_LayerExtrusionEdit_FirstLayerFlow_hpp_
#define slic3r_Plugins_LayerExtrusionEdit_FirstLayerFlow_hpp_

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"

namespace slic3r_api { namespace LayerExtrusionEdit { namespace FirstLayerFlowPlugin {

/*
Register the STEP_LAYER_EXTRUSION_EDIT first-layer flow plugin.

On the first layer of each object, the plugin multiplies the deposited flow
(mm3_per_mm) of the cloned PrintingPlan extrusions by the regional
first_layer_flow_ratio and by the extruder's filament_first_layer_flow_ratio.
It runs before the default speed plugin, so speed limits see the final flow.
*/
void register_first_layer_flow_plugin(orchestrator_handle *orch);

}}} // namespace slic3r_api::LayerExtrusionEdit::FirstLayerFlowPlugin

#endif // slic3r_Plugins_LayerExtrusionEdit_FirstLayerFlow_hpp_
