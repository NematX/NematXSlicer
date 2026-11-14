///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

#ifndef slic3r_Plugins_LayerExtrusionEdit_AdvancedTravel_hpp_
#define slic3r_Plugins_LayerExtrusionEdit_AdvancedTravel_hpp_

/*
Register the default advanced travel provider during built-in plugin registration.
The layer_extrusion_edit.travel selector honours explicit active provider IDs. It uses
ScopeTravelConnector (TravelConnectionHelpers.hpp) to record ordered extrusion
gaps, then fills them with XY routes from the selected advanced planner.
Legacy is the planner group's default and the fallback for absent or unknown planner choices.
The opt-in advanced planner collects active island attractors, forbidden tops and support obstacles.
Active means present in the host activation set, configured by activated.ini.
Collection occurs only when the chronology contains an eligible travel after layer entry. The
layer-entry travel stays straight. The legacy planner owns its worker-local input
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
removes only its zones. The advanced planner routes on withdrawn attractors minus
expanded forbidden zones, validating all segments against original forbidden
material; it does not inherit the legacy planner's own top avoidance. Its spacing
and island-jump settings come from the shared worker layer-input cache.
Normal object-layer endpoints may attach to a domain. Regionless and auxiliary-layer
endpoints offer no domain attachment; travels touching them include slices of the
plan's objects as blockers independently of contributor activation. The synthetic
wipe tower is not a blocker. The worker borrows the
plan, group index and Z for that geometry, excluding components containing an endpoint
so mixed travels can enter or leave their in-part endpoint's component.
An inactive advanced provider blocks its dependent members even when
their IDs are enabled in the profile.
The connector owns insertion, Z interpolation and materialized travel flags.
Travel group priorities select advanced (-2), then avoid-crossing (-1) when
advanced is inactive, then straight travel (0).
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
