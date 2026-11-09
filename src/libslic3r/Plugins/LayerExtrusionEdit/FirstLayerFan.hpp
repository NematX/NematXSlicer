#pragma once
// This registration interface adds the layer program and CLOSED role fallbacks
// to a host. Register before initialization; no geometry is owned by the provider.
// Register the host D/F applicator and the sole late-fallback closure stage.
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
namespace slic3r_api { namespace LayerExtrusionEdit { namespace FirstLayerFanPlugin {
// Lend the permanent first-layer policy to a host that outlives it.
void register_first_layer_fan_plugin(orchestrator_handle *orch);
}}}
