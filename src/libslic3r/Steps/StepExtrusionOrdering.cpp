/*
STEP_ORDERING preflight and plugin execution
=============================================

The source LayerRegionIsland perimeter trees are cloned into PrintingPlan by
the ordering step. The preflight below checks the loop metadata on those
source trees before any plugin can consume or reorder the clones.
Seam placement chooses the start of closed extrusion loops from regional settings.
Its selector belongs to this module because a changed placement replays ordering.
*/
#include "StepExtrusionOrdering.hpp"

#include <algorithm>
#include <sstream>
#include <string>

#include <boost/log/trivial.hpp>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_ordering.h"
#include "libslic3r/CustomGCode.hpp"
#include "libslic3r/Exception.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionProperty.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"
#include "libslic3r/Steps/StepRunner.hpp"

namespace Slic3r::Steps::StepExtrusionOrdering {

// Install extrusion-ordering traits and the exact regional seam selector declaration on the supplied host.
void register_selector_traits(Orchestrator &orchestrator);

// Copy output traits and exact regional seam traits into the borrowed host, which owns the strings.
// Seam selection invalidates ordering and keeps stored-id reading disabled for a singleton; no print/configuration data changes.
// Duplicate/conflicting installation asserts and allocation errors propagate without transferring provider ownership.
void register_selector_traits(Orchestrator &orchestrator)
{
    StepSelectorTraits traits;
    traits.category = RAW_OPTION_CATEGORY_OUTPUT;
    traits.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    const int32_t registered = orchestrator.register_builtin_step_selector(STEP_ORDERING, std::move(traits));
    assert(registered == 1);
    (void)registered;

    StepSelectorTraits seam;
    seam.exclusive_group = "seam_placer_plugin";
    seam.container_type = RAW_CONTAINER_TYPE_REGION;
    seam.invalidates_step = STEP_ORDERING;
    seam.category = RAW_OPTION_CATEGORY_SLICING;
    seam.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    seam.ui_page = "Perimeters & Shell";
    seam.ui_group = "Seam";
    seam.strict_selection = true;
    seam.read_with_one_provider = false;
    const int32_t seam_registered = orchestrator.register_builtin_step_selector(SEAM_PLACER, std::move(seam));
    assert(seam_registered == 1);
    (void)seam_registered;
}

namespace {

enum class SourceObjectStatus {
    NoSource,
    Valid,
    Invalid
};

struct SourceObjectResult {
    SourceObjectStatus status;
    const PrintObject *object;
};

/* Distinguish an absent source from an island whose source context is corrupt. */
SourceObjectResult source_object(const LayerRegionIsland *region_island)
{
    if (region_island == nullptr)
        return { SourceObjectStatus::NoSource, nullptr };
    if (region_island->regions().empty())
        return { SourceObjectStatus::Invalid, nullptr };

    const PrintObject *object = nullptr;
    for (const LayerRegion *region : region_island->regions()) {
        if (region == nullptr || region->layer() == nullptr || region->layer()->object() == nullptr)
            return { SourceObjectStatus::Invalid, nullptr };
        const PrintObject *candidate = region->layer()->object();
        if (object != nullptr && object != candidate)
            return { SourceObjectStatus::Invalid, nullptr };
        object = candidate;
    }
    return object == nullptr ?
        SourceObjectResult{ SourceObjectStatus::Invalid, nullptr } :
        SourceObjectResult{ SourceObjectStatus::Valid, object };
}

/* Format one post-ordering contract violation without hiding its tree path. */
bool validation_failure(std::string *error,
                        const std::string &path,
                        const std::string &cause)
{
    if (error != nullptr)
        *error = path + ": " + cause;
    return false;
}

/*
STEP_ORDERING source validation follows this call flow:
validate_pre -> validate_layer_perimeters -> validate_perimeter_entity.
Keep the declarations and definitions in that order so the outer source-layer
walk is visible before the recursive tree details.
*/
bool validate_layer_perimeters(const Layer &layer,
                               const std::string &layer_path,
                               std::string *error);
bool validate_perimeter_entity(const ExtrusionEntity &entity,
                               const std::string &path,
                               std::string *error,
                               bool inherited_loop_property);

/* Validate every perimeter root stored by one source layer. */
bool validate_layer_perimeters(const Layer &layer,
                               const std::string &layer_path,
                               std::string *error)
{
    for (size_t island_idx = 0; island_idx < layer.islands().size(); ++island_idx) {
        const LayerSliceIsland &island = layer.island(island_idx);
        for (size_t region_island_idx = 0;
             region_island_idx < island.regions_islands().size();
             ++region_island_idx) {
            const LayerRegionIsland &region_island = island.regions_island(region_island_idx);
            if (!region_island.has_extrusion(LayerRegionIsland::PERIMETERS))
                continue;

            const std::string path =
                layer_path + ".island[" + std::to_string(island_idx) + "]"
                ".region_island[" + std::to_string(region_island_idx) + "]"
                ".perimeters.root";
            if (!validate_perimeter_entity(
                    region_island.extrusion(LayerRegionIsland::PERIMETERS), path, error, false))
                return false;
        }
    }
    return true;
}

/* Validate one perimeter tree using the same depth-first order as its source storage. */
bool validate_perimeter_entity(const ExtrusionEntity &entity,
                               const std::string &path,
                               std::string *error,
                               const bool inherited_loop_property)
{
    const ExtrusionPropertyLoopRole *property =
        entity.get_property<ExtrusionPropertyLoopRole>();
    if (property != nullptr &&
        (property->perimeter_flags & C_EXTRUSION_PERIMETER_FLAG_LOOP) != 0 &&
        !entity.is_loop())
        return validation_failure(
            error, path,
            "entity has C_EXTRUSION_PERIMETER_FLAG_LOOP but is_loop() is false");

    bool has_direct_loop_child = false;
    for (size_t child_idx = 0; child_idx < entity.child_count(); ++child_idx)
        if (entity.child(child_idx).is_loop()) {
            has_direct_loop_child = true;
            break;
        }

    // A loop that directly owns another loop is a structural container. Only
    // terminal ordinary-perimeter loops need their own perimeter property;
    // thin-wall loops share the PERIMETERS bucket but carry a different role.
    const ExtrusionRole role = entity.role();
    const bool is_ordinary_perimeter =
        role.has(ExtrusionRoleModifier::ERM_Perimeter) &&
        !role.has(ExtrusionRoleModifier::ERM_Thin);
    if (entity.is_loop() && !has_direct_loop_child && is_ordinary_perimeter) {
        if (property == nullptr && !inherited_loop_property)
            return validation_failure(
                error, path,
                "terminal loop is missing direct ExtrusionPropertyLoopRole/EPropertyPerimeter");
        if (property != nullptr &&
            (property->perimeter_flags & C_EXTRUSION_PERIMETER_FLAG_LOOP) == 0)
            return validation_failure(
                error, path,
                "terminal loop has direct ExtrusionPropertyLoopRole/EPropertyPerimeter but is missing "
                "C_EXTRUSION_PERIMETER_FLAG_LOOP");
    }

    const bool has_valid_loop_property =
        property != nullptr &&
        (property->perimeter_flags & C_EXTRUSION_PERIMETER_FLAG_LOOP) != 0;
    const bool child_inherits_loop_property =
        inherited_loop_property ||
        (entity.is_loop() && is_ordinary_perimeter && has_valid_loop_property);
    for (size_t child_idx = 0; child_idx < entity.child_count(); ++child_idx) {
        const std::string child_path = path + ".child[" + std::to_string(child_idx) + "]";
        if (!validate_perimeter_entity(
                entity.child(child_idx), child_path, error, child_inherits_loop_property))
            return false;
    }
    return true;
}

/* Describe the catalog state used by an extrusion in a diagnostic. */
const char *instance_kind(const Printing::PrintingObjectInstance *instance,
                          const uint32_t printing_catalog_id)
{
    if (printing_catalog_id == Printing::PRINTING_OBJECT_INSTANCE_ID_NONE)
        return "NONE";
    return instance != nullptr && instance->object != nullptr ? "real" : "synthetic";
}

} // namespace

void clear(Print &print)
{
    // Cleanup discards output only; replay scheduling belongs to Print's checkpoint entry points.
    print.reset_printing_plan();
}

void prepare(Print &print)
{
    DynamicConfig record = CustomGCode::make_print_record(print.model().custom_gcode_per_print_z);
    // Keep the channel object alive for consumers retaining borrowed Config handles.
    print.records().get_or_add(CustomGCode::PrintRecordChannel).swap(record);
}

bool validate_pre(const Print &print, std::string *error)
{
    if (error != nullptr)
        error->clear();

    for (size_t object_idx = 0; object_idx < print.objects().size(); ++object_idx) {
        const PrintObject &object = print.object(object_idx);
        for (size_t layer_idx = 0; layer_idx < object.layer_count(); ++layer_idx) {
            const std::string layer_path =
                "object[" + std::to_string(object_idx) + "]"
                ".normal_layer[" + std::to_string(layer_idx) + "]";
            if (!validate_layer_perimeters(object.layer(layer_idx), layer_path, error))
                return false;
        }
        for (size_t layer_idx = 0; layer_idx < object.auxiliary_layer_count(); ++layer_idx) {
            const std::string layer_path =
                "object[" + std::to_string(object_idx) + "]"
                ".auxiliary_layer[" + std::to_string(layer_idx) + "]";
            if (!validate_layer_perimeters(object.auxiliary_layer(layer_idx), layer_path, error))
                return false;
        }
    }

    const PrintObject *auxiliary_object = print.auxiliary_object();
    if (auxiliary_object != nullptr) {
        for (size_t layer_idx = 0; layer_idx < auxiliary_object->auxiliary_layer_count(); ++layer_idx) {
            const std::string layer_path =
                "print.auxiliary_layer[" + std::to_string(layer_idx) + "]";
            if (!validate_layer_perimeters(
                    auxiliary_object->auxiliary_layer(layer_idx), layer_path, error))
                return false;
        }
    }
    return true;
}

bool validate_post(const Print &print, std::string *error)
{
    const Printing::PrintingPlan *plan = print.printing_plan();
    if (plan == nullptr)
        return validation_failure(error, "plan", "the PrintingPlan is missing");

    for (size_t object_instance_idx = 0; object_instance_idx < plan->object_instances.size(); ++object_instance_idx) {
        const Printing::PrintingObjectInstance &instance = plan->object_instances[object_instance_idx];
        const std::string path = "plan.catalog[" + std::to_string(object_instance_idx) + "]";
        if (instance.object != nullptr) {
            if (instance.object->print() != &print)
                return validation_failure(error, path, "real catalog object belongs to another Print");
            if (instance.object_instance_idx >= instance.object->instances().size())
                return validation_failure(error, path, "real catalog instance index is out of range");
        } else {
            if (instance.object_instance_idx != size_t(-1))
                return validation_failure(error, path, "synthetic catalog entry has a real instance index");
            if (instance.name.empty() || instance.projected_outline.empty() ||
                instance.projected_outline.area() == 0.0)
                return validation_failure(error, path, "synthetic catalog metadata is incomplete");
        }
    }

    for (size_t group_idx = 0; group_idx < plan->groups.size(); ++group_idx) {
        const Printing::PrintingGroup &group = plan->groups[group_idx];
        const std::string group_path = "group[" + std::to_string(group_idx) + "]";
        if (group.plan_identity != plan->identity_token())
            return validation_failure(error, group_path, "group belongs to another PrintingPlan");
        for (const Printing::PrintingObjectInstanceId printing_catalog_id : group.printing_catalog_ids) {
            if (printing_catalog_id == Printing::PRINTING_OBJECT_INSTANCE_ID_NONE ||
                printing_catalog_id >= plan->object_instances.size())
                return validation_failure(error, group_path,
                                          "printing catalog ID is not a valid catalog entry");
        }

        for (size_t layer_idx = 0; layer_idx < group.layers.size(); ++layer_idx) {
            const Printing::PrintingLayerGroup &layer = group.layers[layer_idx];
            const std::string layer_path = group_path + ".layer[" + std::to_string(layer_idx) + "]";
            if (layer.plan_identity != plan->identity_token())
                return validation_failure(error, layer_path, "layer group belongs to another PrintingPlan");

            for (size_t tool_idx = 0; tool_idx < layer.tool_groups.size(); ++tool_idx) {
                const Printing::PrintingToolGroup &tool = layer.tool_groups[tool_idx];
                const std::string tool_path = layer_path + ".tool[" + std::to_string(tool_idx) + "]";
                if (tool.plan_identity != plan->identity_token())
                    return validation_failure(error, tool_path, "tool group belongs to another PrintingPlan");
                if (tool.extruder_id == EXTRUDER_ID_UNKNOWN)
                    return validation_failure(error, tool_path,
                                              "tool group has no concrete extruder after STEP_ORDERING");

                for (size_t extrusion_idx = 0; extrusion_idx < tool.extrusions.size(); ++extrusion_idx) {
                    const Printing::PrintingExtrusion &extrusion = tool.extrusions[extrusion_idx];
                    const std::string extrusion_path =
                        tool_path + ".extrusion[" + std::to_string(extrusion_idx) + "]";
                    if (extrusion.root == nullptr)
                        return validation_failure(error, extrusion_path, "root is not owned by the plan");

                    const uint32_t printing_catalog_id = extrusion.printing_catalog_id;
                    const Printing::PrintingObjectInstance *instance =
                        printing_catalog_id == Printing::PRINTING_OBJECT_INSTANCE_ID_NONE ?
                            nullptr : plan->object_instance(printing_catalog_id);
                    std::ostringstream details;
                    details << extrusion_path << " (instance="
                            << instance_kind(instance, printing_catalog_id)
                            << ", id=" << printing_catalog_id << ")";
                    if (printing_catalog_id != Printing::PRINTING_OBJECT_INSTANCE_ID_NONE && instance == nullptr)
                        return validation_failure(error, details.str(), "extrusion ID is not in the plan catalog");

                    const SourceObjectResult source = source_object(extrusion.region_island);
                    if (source.status == SourceObjectStatus::Invalid)
                        return validation_failure(error, details.str(), "extrusion source region island is invalid");
                    if (source.status == SourceObjectStatus::Valid && source.object->print() != &print)
                        return validation_failure(error, details.str(), "extrusion source belongs to another Print");
                    if (instance != nullptr && instance->object != nullptr &&
                        (source.status != SourceObjectStatus::Valid || source.object != instance->object))
                        return validation_failure(error, details.str(),
                                                  "real instance does not match the extrusion source PrintObject");
                    if (instance != nullptr && instance->object == nullptr && source.status == SourceObjectStatus::Valid &&
                        source.object != print.auxiliary_object())
                        return validation_failure(error, details.str(),
                                                  "synthetic instance is associated with a real object region");
                    if (printing_catalog_id == Printing::PRINTING_OBJECT_INSTANCE_ID_NONE &&
                        source.status == SourceObjectStatus::Valid && source.object != print.auxiliary_object())
                        return validation_failure(error, details.str(),
                                                  "NONE instance is associated with a real object region");

                    if (printing_catalog_id != Printing::PRINTING_OBJECT_INSTANCE_ID_NONE &&
                        std::find(group.printing_catalog_ids.begin(), group.printing_catalog_ids.end(), printing_catalog_id) ==
                            group.printing_catalog_ids.end())
                        return validation_failure(error, details.str(),
                                                  "extrusion ID is not listed by its enclosing group");
                }
            }
        }
    }
    return true;
}

void run_step(Orchestrator &orchestrator, Print &print)
{
    std::string pre_validation_error;
    if (!validate_pre(print, &pre_validation_error))
        throw RuntimeError("STEP_ORDERING pre-validation failed before the plugin pipeline: " + pre_validation_error);

    // STEP_ORDERING is a small sequential pipeline. The first plugin normally
    // builds the PrintingPlan, and later plugins refine that same mutable plan.
    // Running these plugins in parallel would make the plan order depend on
    // scheduler timing, so every selected plugin receives the same payload and
    // runs to completion before the next plugin starts.
    Printing::PrintingPlan &plan = print.mutable_printing_plan();
    run_ctx_extrusion_ordering payload = {};
    payload.print = reinterpret_cast<const print_handle *>(&print);
    payload.plan = reinterpret_cast<printing_plan_handle *>(&plan);

    std::vector<Plugin *> plugins =
        selected_or_active_plugins_for_step(orchestrator, STEP_ORDERING, &print.full_print_config());
    std::string last_plugin_id = "<plugin pipeline>";
    for (Plugin *plugin : plugins) {
        if (plugin == nullptr)
            continue;
        last_plugin_id = plugin->get_id();

        std::atomic_bool execution_error { false };
        plugin_host_context host_context =
            Detail::prepare_guarded_plugin_host_context(orchestrator, STEP_ORDERING, *plugin, print, execution_error);
        plugin_run_context run_context = orchestrator.prepare_plugin_run_context(STEP_ORDERING, plugin, &host_context);
        run_context.data = &payload;

        plugin->setup(run_context, 1);
        Detail::throw_if_plugin_stopped(orchestrator, print, STEP_ORDERING, *plugin, execution_error, "setup");
        plugin->setup_run(run_context);
        Detail::throw_if_plugin_stopped(orchestrator, print, STEP_ORDERING, *plugin, execution_error, "setup_run");
        plugin->run(run_context);
        Detail::throw_if_plugin_stopped(orchestrator, print, STEP_ORDERING, *plugin, execution_error, "run");
        print.pipeline_progress().complete_plugin(STEP_ORDERING, plugin->get_id());
    }

    std::string validation_error;
    if (!validate_post(print, &validation_error))
        throw RuntimeError("STEP_ORDERING post-validation failed after the plugin pipeline "
                           "(last executed plugin: '" + last_plugin_id + "'): " + validation_error);

}

} // namespace Slic3r::Steps::StepExtrusionOrdering
