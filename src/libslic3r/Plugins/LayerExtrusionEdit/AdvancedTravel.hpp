///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

#ifndef slic3r_Plugins_LayerExtrusionEdit_AdvancedTravel_hpp_
#define slic3r_Plugins_LayerExtrusionEdit_AdvancedTravel_hpp_

/*
Register the advanced travel alternative during built-in plugin registration.
The layer_extrusion_edit.travel selector can choose it explicitly. It uses
ScopeTravelConnector (TravelConnectionHelpers.hpp) to record ordered extrusion
gaps, then fills them with XY routes from the selected advanced planner.
Compatibility is the default and the fallback for absent or unknown choices.
The opt-in zones planner collects active island attractors, forbidden tops and support obstacles.
Active means present in the host activation set, configured by activated.ini.
Collection occurs only when the chronology contains an eligible travel after layer entry. The
layer-entry travel stays straight. Compatibility owns its worker-local input
cache; the three boundary contributors share a second cache owned by advanced
travel, so layer geometry may be copied twice. Support obstacles read slices directly.
The permanent contributor plugins are non-exclusive no-op members, dependent on
advanced travel. Its private implementation checks each ID with
orchestrator_find_plugin before passing its borrowed worker-local computation.
The four members are island_attractors, forbidden_tops, printed_tops and support_obstacles. When active,
printed_tops replaces forbidden_tops, starting each top after the last arrival
in its original island rather than at the beginning of the layer.
Support obstacles forbid separate object components at or below travel Z only
for travels touching support, using the instances in the plan catalog.
Disabling a member
removes only its zones; same-attractor compatibility routing retains its own top
avoidance. An inactive advanced provider blocks its dependent members even when
their IDs are enabled in the profile.
The connector owns insertion, Z interpolation and materialized travel flags.
Avoid-crossing remains the default when active; straight travel is next.
*/

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"

namespace slic3r_api { namespace LayerExtrusionEdit { namespace AdvancedTravelPlugin {

// Register the built-in advanced travel provider without redeclaring its group.
void register_advanced_travel_plugin(orchestrator_handle *orchestrator);
// Register the two no-op planner choices and declare their exclusive selection group.
void register_advanced_travel_planners(orchestrator_handle *orchestrator);
// Register four independently activated no-op contributors: island_attractors, forbidden_tops, printed_tops and support_obstacles.
void register_advanced_travel_contributors(orchestrator_handle *orchestrator);

}}}
#endif
