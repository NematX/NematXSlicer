///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_plugin_cpp_PrintHelpers_hpp_
#define slic3r_Api_plugin_cpp_PrintHelpers_hpp_

#include <cstdint>
#include <set>
#include <vector>

#include "Views.hpp"

namespace slic3r_api {

class ConstPrintingLayerGroup;

/*
Print helper functions for plugin algorithms
============================================

These helpers reimplement small pieces of classic Print/PrintObject logic using
only C API views. They are meant for plugins that need the same derived values
as the host, but must not include host-only classes such as Print, Flow or
Extruder.
They also derive the actual height of one finalized plan layer group from its
source layers or its Z gap in the plan.

All extruder ids returned by these helpers are zero-based, matching the C++ host
convention used by nozzle_diameter and filament vector settings. Config settings
such as perimeter_extruder remain one-based in the user config and are converted
inside the helpers when needed.
*/

void collect_object_printing_extruders(const Print &print,
                                       const Object &object,
                                       const PrintRegion &region,
                                       std::set<extruder_index_t> &object_extruders);

std::set<extruder_index_t> object_extruders(const Print &print, const Object &object);

/*
Return the object first-layer height in scaled coordinates.

If first_layer_height is an absolute value, the result is just that value. If it
is a percentage, the helper follows the classic host logic: it evaluates the
percentage against every nozzle used by the object, then keeps the smallest
height so every involved extruder can print the first layer safely.
*/
coord_t get_object_first_layer_height(const Print &print, const Object &object);

/*
Return the smallest first-layer height among all objects in the print.

Skirt and brim are print-level first-layer features, so they use the most
restrictive object height. Throws std::runtime_error when called on an empty
print because there is no meaningful first-layer context.
*/
coord_t get_min_first_layer_height(const Print &print);

/*
Collect the extruders that may be used while printing skirt.

It starts from object printing extruders and adds support extruders. If support
is configured to use the "current extruder" (setting value 0), the object
extruders are kept as possible support tools because the concrete tool is chosen
later by ordering/G-code logic.
*/
std::set<extruder_index_t> collect_print_extruders_for_skirt(const Print &print,
                                                             const std::vector<Object> &objects);

/*
Convert volumetric extrusion (mm3 per mm of path) into filament E per mm.

This mirrors Extruder::e_per_mm(): for normal E values, divide by filament
cross-section and multiply by extrusion_multiplier. In volumetric E mode, the
G-code E axis already uses mm3, so the conversion factor is only the extrusion
multiplier.
*/
double e_per_mm(const Print &print, extruder_index_t extruder_id, double mm3_per_mm);

coord_t check_z_step(coord_t val, coord_t z_step);

// Return the shared actual height of a finalized layer group; previous_print_z is the print_z of the preceding plan layer or 0 for the first layer, DefaultGCodeFirmwareSession::reliable_layer_height verifies that source layers in one group agree, and a source-free group uses its Z gap.
coord_t printing_layer_height(const ConstPrintingLayerGroup &layer, coord_t previous_print_z);

} // namespace slic3r_api


#endif // slic3r_Api_plugin_cpp_PrintHelpers_hpp_
