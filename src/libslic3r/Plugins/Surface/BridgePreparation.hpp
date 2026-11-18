///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#ifndef slic3r_BridgePreparation_hpp_
#define slic3r_BridgePreparation_hpp_
/* Register the passes that choose bridge directions and expand external bridge fill areas.

   The loader first registers the pass that changes surfaces, then the pass that
   sets their infill parameters. Volume correction is registered in BridgeVolume.hpp.
   The infill generator calls validate_recipe() after all parameter modifiers, so
   a later plugin cannot silently replace the selected direction or parallel rows.
   expansion_margins() resolves uniform and axial distances for preparation and
   final path checks, using the same external-perimeter width and shell limit.
   Handles are borrowed for the call; invalid parameters stop generation.

   Preparation fits the rows of each prepared bridge (BridgeRows.hpp) and stores them on its surfaces as
   PreparedRows, direction included; the recipe pass reads them back through prepared_rows_key():

       const PreparedRows *rows = prepared_rows_key(orch).get(Surface(surface));
       if (rows != nullptr) fit = BridgeRows::anchored(Surface(surface).expolygon(), rows->angle, spacing, rows->pitch, rows->anchor);
*/
#include <cstdint>
#include "libslic3r/Api/plugin/c/steps/slic3r_step_infill.h"
#include "libslic3r/Api/plugin/cpp/PluginPropertyKey.hpp"
namespace slic3r_api { namespace BridgePreparation {
// Resolved distances in scaled XY units, with the uniform expansion applied first.
struct ExpansionMargins { coord_t axial, uniform; };
// The rows fitted to a prepared bridge: their direction in radians, in [0, PI), their pitch in millimeters and the
// scaled position across the rows of one of them, from BridgeRows::anchor() along that direction. The position is
// absolute, so it stays valid for any outline of the same bridge; the direction is the one the rows were fitted for.
struct PreparedRows { double angle; double pitch; int64_t anchor; };
// The property key of PreparedRows, registered by name on orch; every call returns the same key for one orchestrator.
PluginPropertyKey<PreparedRows> prepared_rows_key(orchestrator_handle *orch);
// Resolve both settings on one region; zero infill caps their combined axial reach
// to the shell width. The borrowed region must be valid; configuration errors propagate.
ExpansionMargins expansion_margins(const layer_region_handle *region);
// Register external/internal classification and direction/extension, with its
// recipe activation partner. Registration failure throws; the first host must outlive it.
void register_surface(orchestrator_handle *orch);
// Register the pass that requests parallel bridge rows at the pitch and positions stored by preparation, using the same
// long-lived host.
void register_recipe(orchestrator_handle *orch);
// For surfaces marked with BridgeSurfaceInfo, reject incompatible direction,
// pattern, flow or tool settings, or a row pitch outside the overlap settings, before caching. A surface with
// PreparedRows must get exactly these rows: their direction, pitch and positions. orch is the host that registered
// PreparedRows. Borrow the region and pattern id for this check; a null surface or a surface without that marker
// needs no check.
void validate_recipe(orchestrator_handle *orch, const surface_handle *surface, const raw_infill_pattern_params &params,
    const char *pattern_id, const layer_region_handle *region);
}}
#endif
