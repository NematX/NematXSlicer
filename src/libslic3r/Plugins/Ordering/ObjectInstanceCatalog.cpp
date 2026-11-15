///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "ObjectInstanceCatalog.hpp"

#include <algorithm>
#include <map>
#include <regex>
#include <set>
#include <stdexcept>
#include <string>

#include "libslic3r/Api/plugin/c/steps/slic3r_step_ordering.h"
#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/ConfigViews.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/Api/plugin/cpp/VolumeViews.hpp"
#include "libslic3r/Api/plugin/c/slic3r_def.h"

/*
Object-instance catalog provider
================================

This STEP_ORDERING provider completes entries registered by a plan builder. It
uses only plugin-facing views and services, so the same catalog rules are
available to native and future non-native plan builders without importing the
host Print, Model, or mesh slicer implementation.
After filling missing metadata, it resolves collisions between physical-copy
names. Every original name is reserved before colliding copies receive catalog-ID
suffixes, so already unique names remain unchanged, including suffix-like names.
The plan owns all names and outlines; IDs and source associations never change.

    populate_object_instance_catalog()
        |-- object_instance_name() -> apply label-style and firmware sanitization
        |-- project_object_instance_outline() -> copy the placed footprint
        `-- reserve names and disambiguate colliding physical copies
*/

namespace slic3r_api { namespace Ordering { namespace ObjectInstanceCatalogPlugin {
namespace {

const char *const k_no_dependencies[] = {nullptr};

std::string object_instance_name(const Print &print, const Object &object,
                                 uint32_t object_idx, uint32_t instance_idx)
{
    static const std::regex invalid_name("[^\\w]+", std::regex_constants::ECMAScript);
    std::string name = std::regex_replace(object.name(), invalid_name, std::string("_"));
    const Config config = print.config();
    const std::string label_style = config.has("gcode_label_objects") ?
        config.get("gcode_label_objects").serialize() : std::string();
    if (label_style == "firmware") {
        if (object.instance_count() > 1)
            name += " (Instance " + std::to_string(instance_idx + 1) + ")";
    } else {
        name += " id:" + std::to_string(object_idx) + " copy " +
                std::to_string(instance_idx);
    }

    const std::string gcode_flavor = config.has("gcode_flavor") ?
        config.get("gcode_flavor").serialize() : std::string();
    if (gcode_flavor == "klipper") {
        const std::string banned = "\b\t\n\v\f\r \"#%&'*-./:;<>\\";
        std::replace_if(name.begin(), name.end(), [&banned](char character) {
            return banned.find(character) != std::string::npos;
        }, '_');
    }
    return name;
}

StoredPolygon project_object_instance_outline(
    storage_handle *storage, const Print &print, const Object &object,
    uint32_t instance_idx)
{
    if (instance_idx >= object.instance_count())
        throw std::out_of_range("The object-instance index is outside the Object.");

    const c_point shift = object.instance_shift(instance_idx);
    const c_matrix4d instance_transform = matrix4d_mul(
        matrix4d_translation(unscaled(shift.x), unscaled(shift.y), 0.0),
        object.transform_centered());
    ClipperOperand positive = ClipperOperand::create_empty(storage);
    ClipperOperand material = ClipperOperand::create_empty(storage);

    for (uint32_t volume_idx = 0; volume_idx < object.volume_count(); ++volume_idx) {
        const Volume volume = object.volume(volume_idx);
        if (!volume.is_model_part() && !volume.is_negative())
            continue;

        const c_matrix4d transform = matrix4d_mul(instance_transform, volume.matrix());
        const StoredPolygonCollection projected =
            volume.mesh().project_to_polygons(storage, transform);
        const PolygonCollection polygons(projected.handle());
        if (polygons.empty())
            continue;

        const ClipperOperand shape = ClipperOperand::from_polygons(storage, polygons);
        if (volume.is_model_part()) {
            positive = clipper_union2(positive, shape);
            material = clipper_union2(material, shape);
        } else {
            material = clipper_diff(material, shape);
        }
    }

    if (positive.empty())
        throw std::invalid_argument("The object instance has no projected printable geometry.");

    const StoredExPolygonCollection resolved = material.to_expolygon_collection();
    StoredPolygon outline(storage);
    if (resolved.size() == 1) {
        outline.copy_from(resolved.front().contour());
    } else {
        const StoredPolygonCollection positive_polygons = positive.to_polygon_collection();
        outline = StoredPolygon::adopt(
            storage, polygons_convex_hull(storage, positive_polygons.handle()));
    }

    const coord_t resolution = scale_d(
        print.config().float_or_default("resolution_internal", 0.0));
    outline.simplify(resolution, resolution);
    if (outline.empty())
        throw std::invalid_argument("The simplified object-instance outline is empty.");
    return outline;
}

class ObjectInstanceCatalog final : public PluginBase
{
public:
    static ObjectInstanceCatalog &instance(orchestrator_handle *orchestrator)
    {
        static ObjectInstanceCatalog instance(orchestrator);
        return instance;
    }

    explicit ObjectInstanceCatalog(orchestrator_handle *orchestrator) :
        PluginBase(orchestrator)
    {}

private:
    const char *id_impl() const noexcept override
    {
        return "ordering.object_instance_catalog";
    }
    const char *name_impl() const noexcept override
    {
        return "Printing-plan object-instance catalog";
    }
    const char *description_impl() const noexcept override
    {
        return "Completes object-instance metadata after the plan builder.";
    }
    slicing_step_t step_impl() const noexcept override { return STEP_ORDERING; }
    const char *const *dependencies_impl() const noexcept override
    {
        return k_no_dependencies;
    }
    int32_t priority_impl() const noexcept override { return 25; }

    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override
    {
        if (keys == nullptr)
            return 3;
        keys[0] = raw_used_config_key{
            "gcode_label_objects", RAW_CO_ENUM,
            RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE};
        keys[1] = raw_used_config_key{
            "gcode_flavor", RAW_CO_ENUM,
            RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE};
        keys[2] = raw_used_config_key{
            "resolution_internal", RAW_CO_FLOAT,
            RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE};
        return 3;
    }

    void run_impl(const plugin_run_context *run_context) const override
    {
        const run_ctx_extrusion_ordering *context =
            plugin_ctx_as_extrusion_ordering(run_context);
        if (run_context == nullptr || context == nullptr || context->print == nullptr ||
            context->plan == nullptr || run_context->plugin_storage == nullptr)
            throw std::invalid_argument(
                "The object-instance catalog requires Print, PrintingPlan and storage.");
        populate_object_instance_catalog(
            context->print, context->plan, run_context->plugin_storage);
    }
};

} // namespace

void populate_object_instance_catalog(
    const print_handle *print_handle_value,
    printing_plan_handle *plan_handle_value,
    storage_handle *storage)
{
    if (print_handle_value == nullptr || plan_handle_value == nullptr || storage == nullptr)
        throw std::invalid_argument("Print, PrintingPlan and storage are required.");

    const Print print(print_handle_value);
    PrintingPlan plan(plan_handle_value);
    const uint32_t count = plan.object_instance_count();
    for (uint32_t id = 0; id < count; ++id) {
        const PrintingObjectInstance instance = plan.object_instance(id);
        if (!instance.name().empty() && instance.has_projected_outline())
            continue;
        if (!instance.has_object())
            throw std::invalid_argument(
                "A synthetic object instance must provide its metadata at registration.");

        const Object source = instance.object();
        uint32_t object_idx = 0;
        while (object_idx < print.object_count() &&
               print.object(object_idx).handle() != source.handle())
            ++object_idx;
        if (object_idx == print.object_count())
            throw std::invalid_argument(
                "The PrintingPlan catalog contains an object from another Print.");

        const uint64_t instance_idx = instance.object_instance_idx();
        if (instance_idx >= source.instance_count())
            throw std::out_of_range("The catalog instance index is outside its Object.");
        const std::string name = object_instance_name(
            print, source, object_idx, uint32_t(instance_idx));
        const StoredPolygon outline = project_object_instance_outline(
            storage, print, source, uint32_t(instance_idx));
        if (!printing_plan_set_object_instance_metadata(
                plan.mutable_handle(), id, name.c_str(), outline.handle()))
            throw std::invalid_argument(
                "The PrintingPlan rejected object-instance metadata.");
    }
    std::map<std::string, uint32_t> name_counts;
    std::set<std::string> reserved_names;
    for (uint32_t id = 0; id < count; ++id) {
        const PrintingObjectInstance instance = plan.object_instance(id);
        reserved_names.insert(instance.name());
        if (instance.has_object()) ++name_counts[instance.name()];
    }
    // Reserve originals first so a generated suffix cannot steal an existing unique name.
    for (uint32_t id = 0; id < count; ++id) {
        const PrintingObjectInstance instance = plan.object_instance(id);
        if (!instance.has_object() || name_counts[instance.name()] < 2) continue;
        const std::string base = instance.name() + "_id_" + std::to_string(id);
        std::string name = base;
        uint32_t suffix = 0;
        while (reserved_names.count(name) != 0) name = base + "_" + std::to_string(++suffix);
        reserved_names.insert(name);
        if (!printing_plan_set_object_instance_metadata(plan.mutable_handle(), id, name.c_str(),
                instance.projected_outline().handle()))
            throw std::invalid_argument("The PrintingPlan rejected a unique object-instance name.");
    }
}

void register_object_instance_catalog_plugin(orchestrator_handle *orchestrator)
{
    orchestrator_register_plugin(
        orchestrator, ObjectInstanceCatalog::instance(orchestrator).c_instance());
}

}}} // namespace slic3r_api::Ordering::ObjectInstanceCatalogPlugin
