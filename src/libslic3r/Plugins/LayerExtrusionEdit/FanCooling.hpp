#pragma once
// Register the optional time-based fan policy; it never changes motion speeds.
// The host owns the registry and must outlive the borrowed permanent provider.
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
namespace slic3r_api { namespace LayerExtrusionEdit { namespace FanCoolingPlugin {
// Register once before initialization; rejected registration follows host diagnostics.
void register_fan_cooling_plugin(orchestrator_handle *orch);
}}}
