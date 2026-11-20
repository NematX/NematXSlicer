///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

#ifndef slic3r_Plugins_LayerExtrusionEdit_AdvancedTravel_hpp_
#define slic3r_Plugins_LayerExtrusionEdit_AdvancedTravel_hpp_

/*
Register the default advanced travel provider during built-in plugin registration.
The layer_extrusion_edit.travel selector honours explicit active provider IDs. It uses
ScopeTravelConnector (TravelConnectionHelpers.hpp) to record ordered extrusion
gaps, then fills them with XY routes. Gaps where avoid_crossing_perimeters applies
use the zone planner; the other gaps, and zone routes that end in a straight
fallback, use the best-effort NoTravel path of NoTravelRouting.hpp. NoTravel
volumes are therefore avoided whatever the avoidance settings.
The zone planner collects active island attractors, forbidden tops and the
layer's NoTravel sections, which are forbidden for every tool and instance.
Active means present in the host activation set, configured by activated.ini.
Collection occurs only when the chronology contains an eligible travel. The layer-entry
travel is routed unless avoid_crossing_layer_entry is straight; it then starts in the
island of the previous layer's last printed object when that island is within four nozzle
diameters, otherwise from outside the part. The compatibility planner, used for regional
eligibility, owns its worker-local input cache; the three boundary contributors share
a second cache owned by advanced travel, so layer geometry may be copied twice.
The permanent contributor plugins are non-exclusive no-op members, dependent on
advanced travel. Its private implementation checks each ID with
orchestrator_find_plugin before passing its borrowed worker-local computation.
The three members are island_attractors, forbidden_tops and printed_tops. When active,
printed_tops replaces forbidden_tops, starting each top after the last arrival
in its original island rather than at the beginning of the layer. NoTravel zones
have no activation member: the presence of a NoTravel volume enables them.
When an endpoint is outside the part, the planner also collects platter obstacles
from the current and earlier groups, uniting translated slices before grouping
the endpoint obstacles.
Disabling a member
removes only its zones. The advanced planner groups original endpoint obstacles,
forms M_a/W_a and M_p/W_p from original and offset masks, and expands only the initial
platter for complete-route containment; it checks routes against forbidden material
shrunk by SCALED_EPSILON, where an exit may cross only its own endpoint group. Its
spacing and island-jump settings come from the shared worker layer-input cache.
Normal object-layer endpoints may attach to an attractor or platter domain. Regionless
and auxiliary-layer endpoints offer only the platter attachment; travels touching them
include slices of the plan's objects as blockers independently of contributor activation.
The synthetic wipe tower is not a blocker. The worker borrows the
plan, group index and Z for that geometry. Pieces containing an endpoint, with every
piece touching them transitively, form endpoint groups; each group is either kept and
left by a straight exit or omitted, and the best-scored route wins, so mixed travels
can enter or leave their in-part endpoint's component.
An inactive advanced provider blocks its dependent members even when
their IDs are enabled in the profile.
The connector owns insertion, Z interpolation and materialized travel flags.
Travel group priorities select advanced (-2), then straight travel (0) when
advanced is inactive.
*/

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"

namespace slic3r_api { namespace LayerExtrusionEdit { namespace AdvancedTravelPlugin {

// Register the built-in advanced travel provider without redeclaring its group.
void register_advanced_travel_plugin(orchestrator_handle *orchestrator);
// Register three independently activated no-op contributors: island_attractors, forbidden_tops and printed_tops.
void register_advanced_travel_contributors(orchestrator_handle *orchestrator);

}}}
#endif
