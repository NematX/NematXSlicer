///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_Plugins_LayerExtrusionEdit_DefaultTemperature_hpp_
#define slic3r_Plugins_LayerExtrusionEdit_DefaultTemperature_hpp_

/*
Default temperature plugins
============================

This module registers the plan-level startup-temperature producer and the
layer-level extrusion-temperature producer.  The first appends heater events
after start_gcode; the second records sequential temperature decisions and
materializes only the decisions belonging to each layer run.  Its initial
heater state is known only when the default startup provider is the selected
active provider for its exclusive group.
*/

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"

namespace slic3r_api { namespace LayerExtrusionEdit { namespace DefaultTemperaturePlugin {

/* Register the startup and layer heater providers in the supplied orchestrator. */
void register_default_temperature_plugins(orchestrator_handle *orchestrator);

}}} // namespace slic3r_api::LayerExtrusionEdit::DefaultTemperaturePlugin

#endif // slic3r_Plugins_LayerExtrusionEdit_DefaultTemperature_hpp_
