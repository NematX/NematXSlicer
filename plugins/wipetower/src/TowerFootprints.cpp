///|/ Copyright (c) NematX 2026
///|/ Released under the terms of the AGPLv3 or higher
#include "TowerFootprints.hpp"

#include <cmath>

/* Tower enumeration from the Print's settings, as Slic3r::wipe_tower_footprints().

The settings are read from the Print config (nozzles, complete_objects, vase
mode), from its default object config (global tower) and from each object's
config (instance towers). An instance's XY translation is not in the SDK, but
object_get_instance_shift() is that translation plus the object's centring
offset (PrintObject::set_instances), so the centring offset is subtracted.
Both are rounded to the coordinate grid, so an origin may differ from the
host's by one coordinate unit when the translation is not on that grid.

    towers_of_print
        append_tower    square, brim, rotation, translation */

namespace nematx_wipetower {
namespace {
// Append one tower at origin_x/origin_y (mm) with the given object settings and owner.
void append_tower(std::vector<TowerFootprint> &towers, const slic3r_api::Config &print_config,
                  const slic3r_api::Config &config, double origin_x, double origin_y,
                  const object_handle *object, uint32_t instance_idx);
// Scale a point in mm to the coordinate grid with rounding, like Slic3r::Point::new_scale().
c_point new_scale(double x, double y);
}

std::vector<TowerFootprint> towers_of_print(const slic3r_api::Print &print)
{
    std::vector<TowerFootprint> towers;
    const slic3r_api::Config print_config = print.config();
    if (print_config.size("nozzle_diameter", 0) <= 1 || print_config.get("spiral_vase").get_bool())
        return towers;
    const bool complete_objects = print_config.get("complete_objects").get_bool();
    const slic3r_api::Config defaults = print.default_object_config();
    if (defaults.get("wipe_tower").get_bool() && !complete_objects)
        append_tower(towers, print_config, defaults, defaults.get("wipe_tower_x").get_float(),
                     defaults.get("wipe_tower_y").get_float(), nullptr, 0);
    for (uint32_t object_idx = 0; object_idx < print.object_count(); ++object_idx) {
        const slic3r_api::Object object = print.object(object_idx);
        const slic3r_api::Config config = object.config();
        if (!((complete_objects && config.get("wipe_tower").get_bool()) || config.get("wipe_tower_per_object").get_bool()))
            continue;
        const c_point center = object.center_offset();
        for (uint32_t instance_idx = 0; instance_idx < object.instance_count(); ++instance_idx) {
            const c_point shift = object.instance_shift(instance_idx);
            append_tower(towers, print_config, config,
                         unscaled(shift.x - center.x) + config.get("wipe_tower_object_x").get_float(),
                         unscaled(shift.y - center.y) + config.get("wipe_tower_object_y").get_float(),
                         object.handle(), instance_idx);
        }
    }
    return towers;
}

namespace {
void append_tower(std::vector<TowerFootprint> &towers, const slic3r_api::Config &print_config,
                  const slic3r_api::Config &config, const double origin_x, const double origin_y,
                  const object_handle *object, const uint32_t instance_idx)
{
    // The brim is relative to the nozzle of the contour tool, or of the first nozzle when it is automatic.
    const int32_t contour_tool = config.get("wipe_tower_extruder").get_int();
    const uint32_t nozzle = contour_tool > 0 ? uint32_t(contour_tool - 1) : 0;
    const double brim = config.get("wipe_tower_brim_width")
                            .get_effective_value(print_config.get("nozzle_diameter").get_float(nozzle));
    TowerFootprint tower;
    tower.origin_x = origin_x;
    tower.origin_y = origin_y;
    tower.width = config.get("wipe_tower_width").get_float();
    tower.brim_width = brim;
    tower.rotation = config.get("wipe_tower_rotation_angle").get_float();
    tower.object = object;
    tower.instance_idx = instance_idx;
    tower.config = config;
    const double angle = 3.141592653589793238 * tower.rotation / 180.;
    const double c = std::cos(angle), s = std::sin(angle);
    const c_point origin = new_scale(origin_x, origin_y);
    for (const c_point corner : {new_scale(-brim, -brim), new_scale(tower.width + brim, -brim),
                                 new_scale(tower.width + brim, tower.width + brim), new_scale(-brim, tower.width + brim)}) {
        const double x = double(corner.x), y = double(corner.y);
        tower.polygon.push_back({coord_t(std::round(c * x - s * y)) + origin.x, coord_t(std::round(c * y + s * x)) + origin.y});
    }
    towers.push_back(std::move(tower));
}

c_point new_scale(const double x, const double y)
{
    return {coord_t(std::round(scale_d(x))), coord_t(std::round(scale_d(y)))};
}
}

} // namespace nematx_wipetower
