/* This file validates source extrusions and runs plugins that order their output movements.
   An extrusion is a planned movement that deposits material. A perimeter is
   an object's boundary extrusion; a seam is the starting point of a closed loop.

   The host is the slicer application. Its Orchestrator manages plugins,
   components that perform jobs. register_selector_traits() installs protected
   descriptions for seam selection and extrusion-tree ordering. A selector is
   a setting naming an alternative plugin. Changing seam placement recomputes
   ordering, the processing stage identified by STEP_ORDERING.

   LayerRegionIsland groups source geometry for one material region within a
   layer. Its perimeter trees organize nested extrusion entities. prepare()
   prepares ordering data; validate_pre() checks source-loop information before
   plugins clone or reorder it into PrintingPlan, the output movement plan.
   run_step() rejects unrecognized tree orderers when source trees carry atomic
   bridge strands, before any provider setup or run can change them. It then
    invokes enabled ordering plugins sequentially. Only after all have succeeded,
    finalization disables sorting, reversal and rotation on every stored tree,
    including empty nodes and event descendants at every scope. Provider failures
    propagate before this boundary. validate_post() checks the resulting plan and
    reports any remaining permission with its storage path. clear() releases this
    step's results before recomputation. The Print owns the plan throughout;
    finalization changes no geometry, properties, identities or ownership.
    Debug validation also rejects legacy node types at the canonical plan boundary.

   Representative direct calls:

       prepare()
           `-- CustomGCode::make_print_record()

       run_step()
           |-- validate_pre()
           |   `-- validate_layer_perimeters()
           |       `-- validate_perimeter_entity()
           |-- selected_or_active_plugins_for_step()
           |-- plugin->setup()
           |-- plugin->setup_run()
            |-- plugin->run()
            |-- finalize_ordering_plan()
            |   |-- lock_ordering_events() [every plan/group/layer/tool scope]
            |   `-- lock_ordering_tree() [disable permissions without changing content]
            `-- validate_post()

   Provider lifecycle calls occur only after the atomic-strand compatibility
   check. Their order is setup, setup_run, then run; tree branches show callers,
   not an additional processing pipeline. */
#include "StepExtrusionOrdering.hpp"

#include <algorithm>
#include <functional>
#include <sstream>
#include <string>

#include <boost/log/trivial.hpp>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_ordering.h"
#include "libslic3r/Api/plugin/cpp/properties/BridgeProperties.hpp"
#include "libslic3r/CustomGCode.hpp"
#include "libslic3r/Exception.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionProperty.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"
#include "libslic3r/Printing/PrintingPlanValidation.hpp"
#include "libslic3r/Steps/StepRunner.hpp"

namespace Slic3r::Steps::StepExtrusionOrdering {

// Copy application text, catalogue names and output/regional seam selection rules into the borrowed host without changing print data.
// Seam selection recomputes ordering and ignores stored singleton ids; conflicts assert, and allocation errors propagate.
void register_selector_traits(Orchestrator &orchestrator);

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
    Orchestrator::ExclusiveGroupDefinition seam_definition;
    seam_definition.traits = std::move(seam);
    seam_definition.label = "Seam placement";
    seam_definition.tooltip = "Selects the service used to choose the starting point of closed extrusion loops.";
    seam_definition.translation_domain = "Slic3r";
    const int32_t seam_registered = orchestrator.register_builtin_exclusive_group(SEAM_PLACER, std::move(seam_definition));
    assert(seam_registered == 1);
    (void)seam_registered;
    Orchestrator::ExclusiveGroupDefinition ordering;
    ordering.traits.exclusive_group = "ordering.extrusion_tree";
    ordering.traits.category = RAW_OPTION_CATEGORY_OUTPUT;
    ordering.traits.invalidates_step = STEP_ORDERING;
    ordering.traits.ui_page = "Output options";
    ordering.traits.ui_group = "Advanced";
    ordering.label = "Extrusion-tree ordering";
    ordering.tooltip = "Selects how STEP_ORDERING fixes the order inside each PrintingExtrusion tree.";
    ordering.translation_domain = "Slic3r";
    const int32_t ordering_registered = orchestrator.register_builtin_exclusive_group(STEP_ORDERING, std::move(ordering));
    assert(ordering_registered == 1);
    (void)ordering_registered;
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

// Disable all permissions on borrowed nodes without materializing child storage.
// Cancellation propagates between nodes and may leave a partially locked tree.
void lock_ordering_tree(ExtrusionEntity &entity, const Print &print);
// Lock owned event descendants; the borrowed event roots are permanently fixed.
void lock_ordering_events(const Printing::PrintingScopeEvents &events, const Print &print);
// Lock all extrusion and event trees once, after every ordering plugin succeeds.
void finalize_ordering_plan(Printing::PrintingPlan &plan, const Print &print);
// Report the first enabled permission, even on empty nodes, at its exact storage path.
bool validate_locked_tree(const ExtrusionEntity &entity, const std::string &path, std::string *error);
// Validate both event roots and their descendants with optional path diagnostics.
bool validate_locked_events(const Printing::PrintingScopeEvents &events, const std::string &path, std::string *error);

/*
Distinguish an absent source from an island whose source context is corrupt.
A valid source has at least one region, and all its regions are on the same
layer of the same object: later steps read the source layer from any region.
*/
SourceObjectResult source_object(const LayerRegionIsland *region_island)
{
    if (region_island == nullptr)
        return { SourceObjectStatus::NoSource, nullptr };
    if (region_island->regions().empty())
        return { SourceObjectStatus::Invalid, nullptr };

    const Layer *layer = nullptr;
    for (const LayerRegion *region : region_island->regions()) {
        if (region == nullptr || region->layer() == nullptr || region->layer()->object() == nullptr)
            return { SourceObjectStatus::Invalid, nullptr };
        if (layer != nullptr && layer != region->layer())
            return { SourceObjectStatus::Invalid, nullptr };
        layer = region->layer();
    }
    // One layer belongs to one object, so the regions share their object too.
    return { SourceObjectStatus::Valid, layer->object() };
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

void lock_ordering_tree(ExtrusionEntity &entity, const Print &print)
{
    print.throw_if_canceled();
    const ExtrusionEntity &view = entity;
    for (const ExtrusionEntityUPtr &child : view.children())
        if (child)
            lock_ordering_tree(*child, print);
    entity.set_can_sort_reverse(false, false);
    entity.set_can_rotate(false);
}

void lock_ordering_events(const Printing::PrintingScopeEvents &events, const Print &print)
{
    for (const ExtrusionEntity *root : {&events.before(), &events.after()}) {
        print.throw_if_canceled();
        for (const ExtrusionEntityUPtr &child : root->children())
            if (child)
                lock_ordering_tree(*child, print);
    }
}

void finalize_ordering_plan(Printing::PrintingPlan &plan, const Print &print)
{
    lock_ordering_events(plan.events, print);
    for (Printing::PrintingGroup &group : plan.groups) {
        lock_ordering_events(group.events, print);
        for (Printing::PrintingLayerGroup &layer : group.layers) {
            lock_ordering_events(layer.events, print);
            for (Printing::PrintingToolGroup &tool : layer.tool_groups) {
                lock_ordering_events(tool.events, print);
                for (Printing::PrintingExtrusion &extrusion : tool.extrusions)
                    if (extrusion.root)
                        lock_ordering_tree(*extrusion.root, print);
            }
        }
    }
}

bool validate_locked_tree(const ExtrusionEntity &entity, const std::string &path, std::string *error)
{
    if (entity.can_sort())
        return validation_failure(error, path, "can_sort remains enabled after STEP_ORDERING");
    if (entity.can_reverse())
        return validation_failure(error, path, "can_reverse remains enabled after STEP_ORDERING");
    if (entity.can_rotate())
        return validation_failure(error, path, "can_rotate remains enabled after STEP_ORDERING");
    for (size_t i = 0; i < entity.children().size(); ++i) {
        const ExtrusionEntityUPtr &child = entity.children()[i];
        if (child && !validate_locked_tree(*child, path + ".child[" + std::to_string(i) + "]", error))
            return false;
    }
    return true;
}

bool validate_locked_events(const Printing::PrintingScopeEvents &events,
                            const std::string &path, std::string *error)
{
    return validate_locked_tree(events.before(), path + ".events.before", error) &&
           validate_locked_tree(events.after(), path + ".events.after", error);
}

/*
STEP_ORDERING source validation follows this call flow:
validate_pre -> validate_layer_perimeters -> validate_perimeter_entity.
Keep the declarations and definitions in that order so the outer source-layer
walk is visible before the recursive tree details.
*/
/* Validate every perimeter root stored by one source layer. */
bool validate_layer_perimeters(const Layer &layer,
                               const std::string &layer_path,
                               std::string *error);
/* Validate one perimeter tree in storage order, retaining inherited process attributes. */
bool validate_perimeter_entity(const ExtrusionEntity &entity,
                               const std::string &path,
                               std::string *error,
                               bool inherited_loop_property,
                               const ExtrusionAttributes *inherited_attributes = nullptr);

// Inspect effective attributes on stored geometry: Travel and a parent's legacy
// role summary must not hide a perimeter child or discard inherited attributes.
bool has_ordinary_perimeter_geometry(const ExtrusionEntity &entity, const ExtrusionAttributes *inherited);

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

bool validate_perimeter_entity(const ExtrusionEntity &entity,
                               const std::string &path,
                               std::string *error,
                               const bool inherited_loop_property,
                               const ExtrusionAttributes *inherited_attributes)
{
    const ExtrusionAttributes *attributes = entity.get_property<ExtrusionAttributes>();
    if (attributes == nullptr)
        attributes = inherited_attributes;
    const ExtrusionPropertyLoopRole *property =
        entity.get_property<ExtrusionPropertyLoopRole>();
    if (property != nullptr &&
        (property->perimeter_flags & C_EXTRUSION_PERIMETER_FLAG_LOOP) != 0 &&
        !entity.ExtrusionEntity::is_loop())
        return validation_failure(
            error, path,
            "entity has C_EXTRUSION_PERIMETER_FLAG_LOOP but is_loop() is false");

    bool has_direct_loop_child = false;
    for (size_t child_idx = 0; child_idx < entity.child_count(); ++child_idx)
        if (entity.child(child_idx).ExtrusionEntity::is_loop()) {
            has_direct_loop_child = true;
            break;
        }

    // A loop that directly owns another loop is a structural container. Only
    // terminal ordinary-perimeter loops need their own perimeter property;
    // thin-wall loops share the PERIMETERS bucket but carry a different role.
    const bool is_ordinary_perimeter = has_ordinary_perimeter_geometry(entity, attributes);
    if (entity.ExtrusionEntity::is_loop() && !has_direct_loop_child && is_ordinary_perimeter) {
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
        (entity.ExtrusionEntity::is_loop() && is_ordinary_perimeter && has_valid_loop_property);
    for (size_t child_idx = 0; child_idx < entity.child_count(); ++child_idx) {
        const std::string child_path = path + ".child[" + std::to_string(child_idx) + "]";
        if (!validate_perimeter_entity(
                entity.child(child_idx), child_path, error, child_inherits_loop_property, attributes))
            return false;
    }
    return true;
}

bool has_ordinary_perimeter_geometry(const ExtrusionEntity &entity, const ExtrusionAttributes *inherited)
{
    const ExtrusionAttributes *attributes = entity.get_property<ExtrusionAttributes>();
    if (attributes == nullptr)
        attributes = inherited;
    if (const ArcPolyline *path = entity.polyline_or_null()) {
        if (path->empty() || attributes == nullptr)
            return false;
        const ExtrusionRole role = attributes->extrusion_role();
        return role.has(ExtrusionRoleModifier::ERM_Perimeter) && !role.has(ExtrusionRoleModifier::ERM_Thin);
    }
    for (const ExtrusionEntityUPtr &child : entity.children())
        if (child && has_ordinary_perimeter_geometry(*child, attributes))
            return true;
    return false;
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
#ifndef NDEBUG
    if (!Printing::validate_canonical_plan(*plan, error))
        return false;
#endif
    if (!validate_locked_events(plan->events, "plan", error))
        return false;

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
        if (!validate_locked_events(group.events, group_path, error))
            return false;
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
            if (!validate_locked_events(layer.events, layer_path, error))
                return false;

            for (size_t tool_idx = 0; tool_idx < layer.tool_groups.size(); ++tool_idx) {
                const Printing::PrintingToolGroup &tool = layer.tool_groups[tool_idx];
                const std::string tool_path = layer_path + ".tool[" + std::to_string(tool_idx) + "]";
                if (tool.plan_identity != plan->identity_token())
                    return validation_failure(error, tool_path, "tool group belongs to another PrintingPlan");
                if (tool.extruder_id == EXTRUDER_ID_UNKNOWN)
                    return validation_failure(error, tool_path,
                                              "tool group has no concrete extruder after STEP_ORDERING");
                if (!validate_locked_events(tool.events, tool_path, error))
                    return false;

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
                    if (!validate_locked_tree(*extrusion.root, extrusion_path + ".root", error))
                        return false;
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
    // Only the listed built-in tree providers are recognized as preserving
    // collective strand orientation; ordinary unmarked trees need no such check.
    const std::function<bool(const ExtrusionEntity &)> contains_atomic = [&](const ExtrusionEntity &entity) {
        if (entity.get_property<slic3r_api::BridgeAtomicStrand>()!=nullptr) return true;
        for (const ExtrusionEntityUPtr &child : entity.children())
            if (child && contains_atomic(*child)) return true;
        return false;
    };
    bool requires_atomic=false;
    for (const PrintObject &object : print.objects())
        for (const Layer &layer : object.layers())
            for (const LayerSliceIsland &island : layer.islands())
                for (const LayerRegionIsland &region : island.regions_islands())
                    for (const auto &[role, root] : region.extrusion_buckets())
                        requires_atomic=requires_atomic || contains_atomic(root);
    if (requires_atomic)
        for (const Plugin *plugin : plugins)
            if (plugin->get_exclusive_group()=="ordering.extrusion_tree" &&
                plugin->get_id()!="ordering.extrusion_tree.advanced" &&
                plugin->get_id()!="ordering.extrusion_tree.xy_serpentine" &&
                plugin->get_id()!="ordering.extrusion_tree.default")
                throw RuntimeError("The selected ordering provider does not preserve atomic bridge strands.");
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

    finalize_ordering_plan(plan, print);

    std::string validation_error;
    if (!validate_post(print, &validation_error))
        throw RuntimeError("STEP_ORDERING post-validation failed after the plugin pipeline "
                           "(last executed plugin: '" + last_plugin_id + "'): " + validation_error);

}

} // namespace Slic3r::Steps::StepExtrusionOrdering
