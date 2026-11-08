///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_Plugins_LayerExtrusionEdit_AvoidCrossingPerimetersTravel_hpp_
#define slic3r_Plugins_LayerExtrusionEdit_AvoidCrossingPerimetersTravel_hpp_

/*
Avoid-crossing travel provider
=============================

Call register_avoid_crossing_perimeters_travel_plugin during plugin registration
to add the built-in perimeter-avoiding alternative to layer_extrusion_edit.travel.
The selector chooses this provider or the straight-travel provider for gaps in
the ordered PrintingPlan. The connector in TravelConnectionHelpers.hpp owns
insertion, Z interpolation and travel flags; this provider supplies XY only.

Both regional endpoints must enable avoidance, the first-layer setting must
allow it, and the direct path must cross the prepared interior boundaries.
TravelRouter then seeks a detour using copied public API inputs. The detour
limit can retain straight travel; this is neither a shortest-path service nor
a guarantee that every perimeter crossing is removed.

When both travel providers are active, straight travel remains the group default.
Explicitly select layer_extrusion_edit.travel.avoid_crossing_perimeters to use
this provider.
*/

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"

namespace slic3r_api { namespace LayerExtrusionEdit { namespace AvoidCrossingPerimetersTravelPlugin {

/* Register the optional perimeter-avoiding travel producer. */
void register_avoid_crossing_perimeters_travel_plugin(orchestrator_handle *orchestrator);

}}} // namespace slic3r_api::LayerExtrusionEdit::AvoidCrossingPerimetersTravelPlugin

#endif // slic3r_Plugins_LayerExtrusionEdit_AvoidCrossingPerimetersTravel_hpp_
