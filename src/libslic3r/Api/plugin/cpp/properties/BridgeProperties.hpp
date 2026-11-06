///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

#ifndef slic3r_BridgeProperties_hpp_
#define slic3r_BridgeProperties_hpp_

/* Carry bridge information from surface extension through flow correction and
   path ordering, using typed C++ keys for the properties in slic3r_bridge.h.

   Read BridgeSurfaceInfo on a Surface, BridgeVolumeInfo on an extrusion piece,
   and BridgeAtomicStrand on its containing extrusion collection. Read the
   host-maintained SupportPipelineInfo through Object::properties(), not through
   a surface or extrusion. Payload fields and states are defined in slic3r_bridge.h.

   Example with plugin API views, in namespace slic3r_api:
       const BridgeSurfaceInfo *info = surface.get(BridgeSurfaceInfo::key);
       const SupportPipelineInfo *state = object.properties().get(SupportPipelineInfo::key);

   A lookup can return null. Returned pointers are borrowed from their owner and
   must not survive its replacement or mutation of its property storage. These
   fixed-size values own no geometry; the assertions check their size against the
   C payloads and require trivial copying.
*/
#include <type_traits>
#include "libslic3r/Api/plugin/c/slic3r_bridge.h"
#include "libslic3r/Api/plugin/cpp/PluginPropertyKey.hpp"

namespace slic3r_api {
// Identify a bridge group and the rule for interpreting its prepared surface area.
// With core_rule 1, subtract linked lower-island slices to recover the original
// unsupported core. With core_rule 2, use the entire prepared internal bridge,
// including its extensions, as the working core. See slic3r_bridge.h.
struct BridgeSurfaceInfo : BuiltInPluginPropertyPayload<BridgeSurfaceInfo,
    c_bridge_surface_property, SLIC3R_PROPERTY_TYPE_BRIDGE_SURFACE> {};
// Record one path piece's incoming/adjusted flow and its gap or mean-volume state.
struct BridgeVolumeInfo : BuiltInPluginPropertyPayload<BridgeVolumeInfo,
    c_bridge_volume_property, SLIC3R_PROPERTY_TYPE_BRIDGE_VOLUME> {};
// Keep connected pieces in order and reverse them together, never independently.
struct BridgeAtomicStrand : BuiltInPluginPropertyPayload<BridgeAtomicStrand,
    c_bridge_strand_property, SLIC3R_PROPERTY_TYPE_BRIDGE_STRAND> {};
// Distinguish support calculations that finished empty from those that have not finished.
struct SupportPipelineInfo : BuiltInPluginPropertyPayload<SupportPipelineInfo,
    c_support_pipeline_property, SLIC3R_PROPERTY_TYPE_SUPPORT_PIPELINE> {};
static_assert(sizeof(BridgeSurfaceInfo)==sizeof(c_bridge_surface_property));
static_assert(sizeof(BridgeVolumeInfo)==sizeof(c_bridge_volume_property));
static_assert(sizeof(BridgeAtomicStrand)==sizeof(c_bridge_strand_property));
static_assert(sizeof(SupportPipelineInfo)==sizeof(c_support_pipeline_property));
static_assert(std::is_trivially_copyable_v<BridgeSurfaceInfo> && std::is_trivially_copyable_v<BridgeVolumeInfo> &&
    std::is_trivially_copyable_v<BridgeAtomicStrand> && std::is_trivially_copyable_v<SupportPipelineInfo>);
}
#endif
