///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

#ifndef slic3r_Plugins_LayerExtrusionEdit_AdvancedTravel_hpp_
#define slic3r_Plugins_LayerExtrusionEdit_AdvancedTravel_hpp_

/*
Register the advanced travel alternative during built-in plugin registration.
The layer_extrusion_edit.travel selector can choose it explicitly. It uses
ScopeTravelConnector (TravelConnectionHelpers.hpp) to record ordered extrusion
gaps, then fills them with XY routes from AvoidCrossing::CompatibilityPlanner.
The connector owns insertion, Z interpolation and materialized travel flags.
Avoid-crossing remains the default when active; straight travel is next.
*/

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"

namespace slic3r_api { namespace LayerExtrusionEdit { namespace AdvancedTravelPlugin {

// Register the built-in advanced travel provider without redeclaring its group.
void register_advanced_travel_plugin(orchestrator_handle *orchestrator);

}}}
#endif
