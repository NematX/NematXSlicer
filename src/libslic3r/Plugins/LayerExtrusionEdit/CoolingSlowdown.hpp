#pragma once
// Public entry point of the built-in plugin `layer_extrusion_edit.speed.slowdown`.
//
// On a layer shorter than `slowdown_below_layer_time` (per extruder, seconds),
// the plugin lowers the speed of its slowable extrusions (never below
// `min_print_speed`, never beyond `max_speed_reduction`) so that the layer lasts
// that long when possible, then publishes the final layer duration as a RESOLVED
// PrintingLayerTimeProperty, which the fan cooling policy reads. It runs in
// STEP_LAYER_EXTRUSION_EDIT at priority 16 and changes no geometry. The
// algorithm is in SlowdownSolver.hpp; the plan handling is in CoolingSlowdown.cpp.
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
namespace slic3r_api { namespace LayerExtrusionEdit { namespace CoolingSlowdownPlugin {
// Register the plugin instance with `orch`.
void register_cooling_slowdown_plugin(orchestrator_handle *orch);
}}}