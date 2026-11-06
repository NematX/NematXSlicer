///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#ifndef slic3r_BridgeVolume_hpp_
#define slic3r_BridgeVolume_hpp_
/* Register the experimental extrusion-clearance plugin.

   register_plugin() supplies the independent experimental footprint model.
   The selected provider runs after model/support generation and publishes paths atomically.
   The host uses path_signature() to detect later changes to a corrected path.
   Registration borrows the host; path_signature() borrows its extrusion handle.
   Neither operation transfers ownership.
*/
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
namespace slic3r_api { namespace BridgeVolume {
// Register the experimental extrusion-footprint corrector, disabled by default.
// The plugin keeps a reference to the first host, which must outlive the plugin.
void register_plugin(orchestrator_handle *orch);
// Hash local segment coordinates, arcs and per-point Z without traversing children
// or inherited properties. Requires a non-null borrowed handle; allocation errors propagate.
uint64_t path_signature(const extrusion_entity_handle *entity);
}}
#endif
