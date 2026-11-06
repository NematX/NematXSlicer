///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#ifndef slic3r_BridgePreparation_hpp_
#define slic3r_BridgePreparation_hpp_
/* Register the passes that choose bridge directions and expand their fill areas.

   The loader first registers the pass that changes surfaces, then the pass that
   sets their infill parameters. Both passes keep native nominal bridge flow.
   The infill generator calls validate_recipe() after all parameter modifiers, so
   a later plugin cannot silently replace the selected direction or parallel rows.
   expansion_margins() resolves uniform and axial distances for preparation and
   callers, using the same external-perimeter width and shell limit.
   Handles are borrowed for the call; invalid parameters stop generation.
*/
#include "libslic3r/Api/plugin/c/steps/slic3r_step_infill.h"
namespace slic3r_api { namespace BridgePreparation {
// Resolved distances in scaled XY units, with the uniform expansion applied first.
struct ExpansionMargins { coord_t axial, uniform; };
// Resolve both settings on one region; zero infill caps their combined axial reach
// to the shell width. The borrowed region must be valid; configuration errors propagate.
ExpansionMargins expansion_margins(const layer_region_handle *region);
// Register external/internal classification and direction/extension, with its
// recipe activation partner. Registration failure throws; the first host must outlive it.
void register_surface(orchestrator_handle *orch);
// Register the pass that requests parallel bridge rows, using the same long-lived host.
void register_recipe(orchestrator_handle *orch);
// For surfaces marked with BridgeSurfaceInfo, reject incompatible direction,
// pattern, flow or tool settings before caching. Borrow the region and pattern id
// for this check; a null surface or a surface without that marker needs no check.
void validate_recipe(const surface_handle *surface, const raw_infill_pattern_params &params,
    const char *pattern_id, const layer_region_handle *region);
}}
#endif
