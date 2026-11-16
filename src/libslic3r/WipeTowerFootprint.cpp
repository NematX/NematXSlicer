///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/* Generic square-plus-brim envelopes do not depend on generated wipe paths.
   Coordinates are millimeters until converted into scaled Polygon points.
   Borrowed instance owners identify towers for GUI dragging and clearance;
   their ModelInstance XY translations are shared by Print and GUI enumeration. */
#include "WipeTowerFootprint.hpp"
#include "Geometry.hpp"
#include "Model.hpp"
#include "Print.hpp"
#include "PrintObject.hpp"

namespace Slic3r {
namespace {
// Append one owned envelope from borrowed configuration/owners; allocation failures propagate.
void append_tower(std::vector<WipeTowerFootprint> &towers, const PrintConfig &print_config,
                  const PrintObjectConfig &config, const ModelObject *object,
                  const ModelInstance *instance, const PrintObject *print_object);
}

// Borrow a Print and copy global and complete-object/explicit-instance envelopes independent of parallel scheduling; single-extruder/vase prints return no towers and allocation failures propagate.
std::vector<WipeTowerFootprint> wipe_tower_footprints(const Print &print)
{
    std::vector<WipeTowerFootprint> towers;
    if (print.config().nozzle_diameter.size() <= 1 || print.config().spiral_vase.value)
        return towers;
    if (print.has_global_wipe_tower())
        append_tower(towers, print.config(), print.default_object_config(), nullptr, nullptr, nullptr);
    for (const PrintObject &object : print.objects())
        if ((print_complete_objects(print.config()) && object.config().wipe_tower.value) || object.config().wipe_tower_per_object.value)
            for (const PrintInstance &instance : object.instances())
                append_tower(towers, print.config(), object.config(), object.model_object(), instance.model_instance, &object);
    return towers;
}

// Borrow unsliced model/config, merge overrides and copy eligible global and complete-object/explicit-instance envelopes independent of parallel scheduling; config/allocation failures propagate without ownership transfer.
std::vector<WipeTowerFootprint> wipe_tower_footprints(const Model &model, const DynamicPrintConfig &config)
{
    PrintConfig print_config;
    print_config.apply(config, true);
    PrintObjectConfig defaults;
    defaults.apply(config, true);
    std::vector<WipeTowerFootprint> towers;
    if (print_config.nozzle_diameter.size() <= 1 || print_config.spiral_vase.value)
        return towers;
    if (defaults.wipe_tower.value && !print_complete_objects(print_config))
        append_tower(towers, print_config, defaults, nullptr, nullptr, nullptr);
    for (const ModelObject &object : model.objects()) {
        PrintObjectConfig effective = defaults;
        effective.apply(object.config.get(), true);
        if ((print_complete_objects(print_config) && effective.wipe_tower.value) || effective.wipe_tower_per_object.value)
            for (const ModelInstance *instance : object.instances)
                if (object.printable && instance->is_printable())
                    append_tower(towers, print_config, effective, &object, instance, nullptr);
    }
    return towers;
}

namespace {
// Resolve brim against the selected contour nozzle (first nozzle for automatic selection), then copy geometry; borrowed owners remain caller-owned and allocation failures propagate.
void append_tower(std::vector<WipeTowerFootprint> &towers, const PrintConfig &print_config,
                  const PrintObjectConfig &config, const ModelObject *object,
                  const ModelInstance *instance, const PrintObject *print_object)
{
    const Vec2d origin = instance == nullptr ? Vec2d(config.wipe_tower_x.value, config.wipe_tower_y.value) :
        Vec2d(instance->get_offset().x() + config.wipe_tower_object_x.value,
              instance->get_offset().y() + config.wipe_tower_object_y.value);
    const size_t nozzle = config.wipe_tower_extruder.value > 0 ? size_t(config.wipe_tower_extruder.value - 1) : 0;
    const double brim = config.wipe_tower_brim_width.get_effective_value(print_config.nozzle_diameter.get_at(nozzle));
    towers.push_back({origin, config.wipe_tower_width.value, brim, config.wipe_tower_rotation_angle.value,
        wipe_tower_footprint(origin, config.wipe_tower_width.value, brim, config.wipe_tower_rotation_angle.value),
        object, instance, print_object});
}
}

// Convert nonnegative millimeter width/brim and finite pose into owned scaled points, rotate about the local origin, then translate; no input ownership changes and allocation failures propagate.
Polygon wipe_tower_footprint(const Vec2d &origin, const double width, const double brim_width, const double rotation)
{
    Polygon polygon({Point::new_scale(-brim_width, -brim_width),
                     Point::new_scale(width + brim_width, -brim_width),
                     Point::new_scale(width + brim_width, width + brim_width),
                     Point::new_scale(-brim_width, width + brim_width)});
    polygon.rotate(Geometry::deg2rad(rotation));
    polygon.translate(Point::new_scale(origin.x(), origin.y()));
    return polygon;
}
}
