///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_plugin_cpp_LayerHeightLimits_hpp_
#define slic3r_Api_plugin_cpp_LayerHeightLimits_hpp_

/*
Layer height limits
===================

These helpers give the smallest and the largest layer height that an extruder
may print, from its min_layer_height and max_layer_height settings. The host
slicing parameters, the layer-height generators and the support plugin all use
them, so that every step works with the same limits.

Both functions take and return millimeters. The caller resolves the settings
first: a percentage applies to the nozzle diameter of the same extruder, and
the enabled state of max_layer_height comes from the config. The caller then
rounds both results to z_step and keeps the maximum at or above the rounded
minimum; this stays with the caller because the host and the plugins round in
different units.

    const double min_mm = min_layer_height_mm(configured_min_mm, nozzle_mm);
    const double max_mm = max_layer_height_mm(configured_max_mm, max_enabled, nozzle_mm);
    const coord_t min_height = check_z_step(scale_i(min_mm), z_step);
    const coord_t max_height = check_z_step(std::max(min_height, scale_i(max_mm)), z_step);
*/

#include <algorithm>

namespace slic3r_api {

// Smallest layer height accepted for any extruder, in mm (1 um); a smaller minimum is raised to it.
constexpr double k_min_layer_height_floor_mm = 0.001;
// Fraction of the nozzle diameter used when min_layer_height is 0 (not set).
constexpr double k_min_layer_height_unset_nozzle_ratio = 1. / 8.;

// Return the minimum layer height for a resolved min_layer_height in mm: an eighth
// of the nozzle diameter when it is 0, and never less than k_min_layer_height_floor_mm.
inline double min_layer_height_mm(double configured_mm, double nozzle_diameter_mm)
{
    const double minimum = configured_mm == 0. ? k_min_layer_height_unset_nozzle_ratio * nozzle_diameter_mm : configured_mm;
    return std::max(k_min_layer_height_floor_mm, minimum);
}

// Return the maximum layer height for a resolved max_layer_height in mm: three quarters
// of the nozzle diameter when the setting is 0 or disabled, otherwise the setting.
// The caller keeps the result at or above the minimum layer height.
inline double max_layer_height_mm(double configured_mm, bool enabled, double nozzle_diameter_mm)
{
    return configured_mm == 0. || !enabled ? 0.75 * nozzle_diameter_mm : configured_mm;
}

} // namespace slic3r_api

#endif // slic3r_Api_plugin_cpp_LayerHeightLimits_hpp_
