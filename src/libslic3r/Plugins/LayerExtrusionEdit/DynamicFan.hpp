#pragma once
// This registration interface adds bounded geometric overhang cooling to a host.
// Register before initialization; request pairs change, not geometry or ordering.
// Register geometric overhang fan increases after CLOSED reference publication.
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
namespace slic3r_api { namespace LayerExtrusionEdit { namespace DynamicFanPlugin {
// Lend the permanent dynamic policy to a host that outlives it.
void register_dynamic_fan_plugin(orchestrator_handle *orch);
}}}
