///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

#ifndef slic3r_Api_plugin_cpp_TemperatureTargets_hpp_
#define slic3r_Api_plugin_cpp_TemperatureTargets_hpp_

#include <cstdint>
#include <optional>

#include "libslic3r/Api/plugin/c/slic3r_def.h"

/* Resolve nozzle targets for C++ temperature providers using borrowed public
   Config views. These functions only select a target; callers own heater history,
   waiting decisions and event publication. No plan or provider cache is required.

   Pass the print's filament settings, the source extrusion's resolved region,
   its concrete tool and its source first-layer status:

       const std::optional<int16_t> target = TemperatureTargets::extrusion_target(
           print_config, region_config, tool, first_layer);

   ConfigViews.hpp supplies the views. Configurations must remain alive and
   unchanged for the duration of a call; no handles are retained. Conversion of
   a selected value above INT16_MAX throws std::invalid_argument. Exceptions must
   be caught before returning through a C plugin entry point, as PluginBase does.
*/

namespace slic3r_api {

class Config;

namespace TemperatureTargets {

// Prefer a positive first-layer filament target on the first layer, then a
// positive normal target. Missing, disabled, out-of-vector and non-positive
// settings produce no request unless the next fallback supplies a target.
std::optional<int16_t> filament_target(const Config &print_config,
                                     extruder_index_t tool,
                                     bool first_layer);

// Prefer an enabled non-negative first-layer regional override, then a normal
// regional override, then filament_target(). Regional zero is a valid target;
// negative, missing and disabled overrides inherit the next fallback.
std::optional<int16_t> extrusion_target(const Config &print_config,
                                      const Config &region_config,
                                      extruder_index_t tool,
                                      bool first_layer);

} // namespace TemperatureTargets
} // namespace slic3r_api

#endif // slic3r_Api_plugin_cpp_TemperatureTargets_hpp_
