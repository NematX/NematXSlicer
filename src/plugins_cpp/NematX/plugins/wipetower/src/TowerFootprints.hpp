///|/ Copyright (c) NematX 2026
///|/ Released under the terms of the AGPLv3 or higher
/* Wipe towers of a Print, read through the plugin SDK.

The host enumerates the towers with Slic3r::wipe_tower_footprints(), which is not
part of the SDK. towers_of_print() repeats it from settings and instances:
- one global tower, from the Print's default object settings, when the printer
  has several nozzles, wipe_tower is on, and neither complete_objects nor the
  vase mode is active;
- one tower per printable instance of an object that asks for it
  (wipe_tower_per_object, or wipe_tower with complete_objects), placed at the
  instance's XY translation plus wipe_tower_object_x/y, with that object's settings.
A tower is a square of wipe_tower_width plus its brim, rotated by
wipe_tower_rotation_angle about its local origin, then moved to its origin.

    const std::vector<TowerFootprint> towers = towers_of_print(print, storage);
    for (const TowerFootprint &tower : towers)
        tower.config.get("wipe_tower_cone_angle").get_float();
*/
#ifndef nematx_wipetower_TowerFootprints_hpp_
#define nematx_wipetower_TowerFootprints_hpp_

#include <cstdint>
#include <vector>

#include "libslic3r/Api/plugin/cpp/ConfigViews.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"

namespace nematx_wipetower {

struct TowerFootprint {
    // Platter translation in mm, applied after the local rotation.
    double origin_x = 0.;
    double origin_y = 0.;
    // Square side and resolved brim width in mm, rotation in degrees.
    double width = 0.;
    double brim_width = 0.;
    double rotation = 0.;
    // Square plus brim, in scaled platter coordinates.
    std::vector<c_point> polygon;
    // Owner object of an instance tower; null for the global tower.
    const object_handle *object = nullptr;
    // Index of the owner instance among the object's printable instances.
    uint32_t instance_idx = 0;
    // Object settings of the tower: the owner's settings, or the Print's default object settings.
    slic3r_api::Config config;
};

// Return the global tower first, then the instance towers in object and instance order.
// The configs are borrowed from the Print and stay valid until its next apply().
std::vector<TowerFootprint> towers_of_print(const slic3r_api::Print &print);

} // namespace nematx_wipetower

#endif
