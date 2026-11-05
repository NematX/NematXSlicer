#include <catch2/catch.hpp>

#include "gcode_test_helpers.hpp"
#include "layer_extrusion_edit_transition_test_helpers.hpp"
#include "plugin_test_helpers.hpp"
#include "test_data.hpp"

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/host/ApiHostUtils.hpp"
#include "libslic3r/Api/internal/LayerIslandAccess.hpp"
#include "libslic3r/Config/FFFPrintConfig.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/Api/plugin/c/slic3r_data_tree.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_layer_extrusion_edit.h"
#include "libslic3r/Api/plugin/cpp/AuxiliaryLayerHelpers.hpp"
#include "libslic3r/Api/plugin/cpp/ConfigViews.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanTimeEstimator.hpp"
#include "libslic3r/Api/plugin/cpp/RegionSettingsViews.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Exception.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/ExtrusionProperty.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"
#include "libslic3r/Plugins/PrintingPlan/PrintingLayerTimeProperty.h"
#include "libslic3r/Steps/StepExtrusionOrdering.hpp"
#include "libslic3r/Steps/StepLayerHeightGeneration.hpp"
#include "libslic3r/Steps/StepLayerExtrusionEdition.hpp"
#include "libslic3r/Steps/StepPostSlicing.hpp"
#include "libslic3r/Steps/StepSlicing.hpp"

/*
STEP_LAYER_EXTRUSION_EDIT process-parameter tests
=================================================

The production step edits clones owned by PrintingPlan, not Layer output. Each
fixture therefore creates a small auxiliary layer with real object and region
configuration, publishes native extrusion paths, and lets STEP_ORDERING create
the same clone hierarchy that the future G-code pipeline consumes.

Tests inspect effective process values through the extrusion tree because the
default plugins deliberately hoist identical speed and acceleration values to
parents independently. Direct-property checks are used only where the storage
placement is itself the behavior under test.
*/

namespace {
using namespace Slic3r;
using namespace Slic3r::Printing;
using slic3r_api::PluginPropertyKey;
using slic3r_api::PrintingLayerTimeProperty;
using slic3r_api::printing_layer_time_property_key;

// Borrow the complete function table with its byte count; optional cleanup/preference functions are null, and no changes or failures occur.
const plugin_vtable *recording_vtable();
// Ignore borrowed initialization arguments and return success without state changes or failure.
int32_t recording_initialize(void *, storage_handle *);

constexpr const char *DEFAULT_SPEED_PLUGIN = "layer_extrusion_edit.speed.default";
constexpr const char *DEFAULT_ACCELERATION_PLUGIN = "layer_extrusion_edit.acceleration.default";
constexpr const char *DEFAULT_FAN_PLUGIN = "layer_extrusion_edit.fan.default";
constexpr const char *DEFAULT_ENTRY_STATE_PLUGIN = "layer_extrusion_edit.entry_state.default";
constexpr const char *DEFAULT_TRANSITION_SCOPE_PLUGIN =
    "layer_extrusion_edit.transition_scope.default";
constexpr const char *DEFAULT_TRAVEL_PLUGIN = "layer_extrusion_edit.travel.default";

struct ExtrusionSpec
{
    ExtrusionRole role = ExtrusionRole::Perimeter;
    double mm3_per_mm = 0.2;
    float existing_speed = -1.f;
    float existing_acceleration = -1.f;
    float pressure_advance = -1.f;
    float fan_speed = -1.f;
    bool force_e_per_mm = false;
};

struct ObservedLeaf
{
    ExtrusionRole role = ExtrusionRole::None;
    float speed = -1.f;
    float acceleration = -1.f;
    float fan_speed = -1.f;
    const ExtrusionPropertySpeed *direct_process = nullptr;
    size_t direct_property_count = 0;
};

struct PreparedSpeedPrint
{
    Model model;
    Print print;
    PluginStorage storage;
    std::vector<ExtrusionEntityCollection *> source_roots;
    std::vector<Layer *> source_layers;
};

struct ProcessState
{
    float speed = -1.f;
    float acceleration = -1.f;
    float fan_speed = -1.f;
};

// Return the resolved mm/s speed of one skirt/brim leaf with the supplied settings.
float resolved_brim_speed(std::initializer_list<ConfigBase::SetDeserializeItem> overrides);

// Return the resolved mm/s² acceleration of one skirt/brim leaf with the supplied settings.
float resolved_brim_acceleration(
    std::initializer_list<ConfigBase::SetDeserializeItem> overrides);

// Preserve and restore the process-wide active plugin set around runner tests.
class ScopedActivePlugins
{
public:
    explicit ScopedActivePlugins(std::initializer_list<const char *> plugin_ids)
        : m_orchestrator(Orchestrator::instance())
    {
        for (Plugin *plugin : m_orchestrator.active_plugins())
            m_previous.push_back(plugin);
        m_orchestrator.clear_active_plugins();
        for (const char *plugin_id : plugin_ids)
            REQUIRE(m_orchestrator.set_plugin_active(plugin_id, true));
    }

    ~ScopedActivePlugins()
    {
        m_orchestrator.clear_active_plugins();
        for (Plugin *plugin : m_previous)
            m_orchestrator.set_plugin_active(plugin, true);
    }

private:
    Orchestrator &m_orchestrator;
    std::vector<Plugin *> m_previous;
};

// Build one rectangular subject used to attach a source region and object.
ExPolygon test_subject()
{
    return ExPolygon(Polygon({
        Point(scale_i(10.), scale_i(10.)),
        Point(scale_i(30.), scale_i(10.)),
        Point(scale_i(30.), scale_i(30.)),
        Point(scale_i(10.), scale_i(30.))
    }));
}

// Start from the complete FFF config so every legacy option read by the plugin
// exists, then override only the values relevant to one scenario.
DynamicPrintConfig speed_config(std::initializer_list<ConfigBase::SetDeserializeItem> overrides)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict(overrides);
    return config;
}

// Create one printable path with attributes and optional pre-existing process
// values. Distinct Y coordinates keep ordering deterministic in diagnostics.
std::unique_ptr<ExtrusionPath> make_path(const ExtrusionSpec &spec, const size_t path_idx)
{
    ExtrusionAttributes attributes(spec.role, ExtrusionFlow(spec.mm3_per_mm, 0.4f, 0.2f));
    if (spec.force_e_per_mm)
        attributes.set_force_e_per_mm();

    std::unique_ptr<ExtrusionPath> path = std::make_unique<ExtrusionPath>(attributes, nullptr, true);
    const coord_t y = scale_i(11. + double(path_idx));
    path->polyline().append(Point(scale_i(11.), y));
    path->polyline().append(Point(scale_i(29.), y));
    if (spec.existing_speed > 0.f || spec.existing_acceleration > 0.f ||
        spec.pressure_advance > 0.f || spec.fan_speed > 0.f) {
        path->add_property(ExtrusionPropertySpeed(spec.existing_speed,
                                                  spec.existing_acceleration,
                                                  spec.pressure_advance,
                                                  spec.fan_speed));
    }
    return path;
}

// Add one auxiliary layer and publish the requested leaves into its perimeter
// bucket. The layer keeps a real source RegionIsland for config lookup.
void append_source_layer(PreparedSpeedPrint &prepared,
                         const std::vector<ExtrusionSpec> &specs,
                         const double print_z_mm)
{
    const ExPolygons subject{test_subject()};
    const slic3r_api::ExPolygonCollection subject_view(
        reinterpret_cast<const expolygon_collection_handle *>(&subject));
    const slic3r_api::Print print_view(reinterpret_cast<const print_handle *>(&prepared.print));
    const slic3r_api::Object object_view(
        reinterpret_cast<const object_handle *>(&prepared.print.object(0)));
    slic3r_api::AuxiliaryLayerBuildResult result =
        slic3r_api::build_auxiliary_layer_regions_from_subject(
            reinterpret_cast<storage_handle *>(&prepared.storage),
            print_view,
            object_view,
            subject_view,
            scale_i(0.2),
            scale_i(print_z_mm),
            scale_i(print_z_mm - 0.1));
    REQUIRE(result.created);
    REQUIRE(result.layer.island_count() == 1);

    layer_region_island_handle *region_island = layer_island_get_or_create_region_island(
        const_cast<layer_island_handle *>(result.layer.island(0).handle()), nullptr, 0, 0);
    REQUIRE(region_island != nullptr);
    extrusion_entity_handle *root_handle = layer_region_island_get_mutable_extrusion(
        region_island, RAW_EXTRUSION_ROLE_PERIMETER);
    REQUIRE(root_handle != nullptr);

    ExtrusionEntityCollection &root = *reinterpret_cast<ExtrusionEntityCollection *>(root_handle);
    for (size_t idx = 0; idx < specs.size(); ++idx)
        root.append(make_path(specs[idx], idx));
    prepared.source_roots.push_back(&root);
    prepared.source_layers.push_back(reinterpret_cast<Layer *>(
        const_cast<layer_handle *>(result.layer.handle())));
}

// Apply the config to a real PrintObject, create the requested source layers,
// then let ordering build the clone-only PrintingPlan.
void prepare_ordered_plan(PreparedSpeedPrint &prepared,
                          const DynamicPrintConfig &config,
                          const std::vector<std::vector<ExtrusionSpec>> &layers)
{
    const TriangleMesh cube = Slic3r::make_cube(20., 20., 20.);
    Slic3r::Test::init_print({cube}, prepared.print, prepared.model, config);
    for (size_t layer_idx = 0; layer_idx < layers.size(); ++layer_idx)
        append_source_layer(prepared, layers[layer_idx], 0.2 + 0.2 * double(layer_idx));

    Steps::StepExtrusionOrdering::clear(prepared.print);
    Steps::StepExtrusionOrdering::prepare(prepared.print);
    Steps::StepExtrusionOrdering::run_step(Orchestrator::instance(), prepared.print);
    REQUIRE(prepared.print.printing_plan() != nullptr);
}

// Build a model part with a centered parameter modifier. The auxiliary layer
// receives the resulting region masks, while its test paths can either cross
// base/modifier/base or remain wholly inside the modifier.
void prepare_regional_process_plan(
    PreparedSpeedPrint &prepared,
    const DynamicPrintConfig &config,
    std::initializer_list<ConfigBase::SetDeserializeItem> modifier_overrides,
    const char *region_setting_key,
    const std::vector<ExtrusionSpec> &specs,
    bool path_inside_modifier = false)
{
    ModelObject *model_object = prepared.model.add_object();
    model_object->name = "regional_process_parameter.stl";

    ModelVolume *part = model_object->add_volume(
        Slic3r::make_cube(20., 20., 10.), ModelVolumeType::MODEL_PART, false);
    part->set_type(ModelVolumeType::MODEL_PART);

    TriangleMesh modifier_mesh = Slic3r::make_cube(4., 20., 10.);
    modifier_mesh.translate(Vec3f(8.f, 0.f, 0.f));
    ModelVolume *modifier = model_object->add_volume(
        std::move(modifier_mesh), ModelVolumeType::PARAMETER_MODIFIER, false);
    modifier->set_type(ModelVolumeType::PARAMETER_MODIFIER);
    DynamicPrintConfig modifier_config;
    modifier_config.set_deserialize_strict(modifier_overrides);
    modifier->config.assign_config(modifier_config);

    model_object->add_instance();
    prepared.model.center_instances_around_point({100., 100.});
    model_object->ensure_on_bed();
    prepared.print.auto_assign_extruders(model_object);
    prepared.print.apply(prepared.model, config);
    prepared.print.validate();
    prepared.print.set_status_silent();

    Orchestrator &orchestrator = Orchestrator::instance();
    Steps::StepLayerHeightGeneration::run_step(orchestrator, prepared.print);
    Steps::StepSlicing::run_step(orchestrator, prepared.print);
    Steps::StepPostSlicing::run_step(orchestrator, prepared.print);

    PrintObject &print_object = prepared.print.object(0);
    REQUIRE(print_object.layer_count() > 0);
    const Layer &source_layer = print_object.layer(0);
    REQUIRE_FALSE(source_layer.lslices().empty());

    const ExPolygons subject = source_layer.lslices();
    const slic3r_api::AuxiliaryLayerBuildResult result =
        slic3r_api::build_auxiliary_layer_regions_from_subject(
            reinterpret_cast<storage_handle *>(&prepared.storage),
            slic3r_api::Print(reinterpret_cast<const print_handle *>(&prepared.print)),
            slic3r_api::Object(reinterpret_cast<const object_handle *>(&print_object)),
            slic3r_api::ExPolygonCollection(
                reinterpret_cast<const expolygon_collection_handle *>(&subject)),
            source_layer.scaled_height(),
            source_layer.scaled_print_z(),
            scale_i(source_layer.slice_z));
    REQUIRE(result.created);
    REQUIRE(result.layer.island_count() == 1);
    const slic3r_api::LayerIsland island = result.layer.island(0);
    REQUIRE(island.region_count() >= 2);

    layer_region_island_handle *region_island = layer_island_get_or_create_region_island(
        const_cast<layer_island_handle *>(island.handle()), nullptr, 0, 0);
    REQUIRE(region_island != nullptr);
    extrusion_entity_handle *root_handle = layer_region_island_get_mutable_extrusion(
        region_island, RAW_EXTRUSION_ROLE_PERIMETER);
    REQUIRE(root_handle != nullptr);

    const BoundingBox bbox = get_extents(subject);
    const Point center = bbox.center();
    const coord_t inset = path_inside_modifier ? scale_i(1.) : scale_i(1.5);
    const Point start = path_inside_modifier ?
        Point(center.x() - inset, center.y()) :
        Point(bbox.min.x() + inset, center.y());
    const Point end = path_inside_modifier ?
        Point(center.x() + inset, center.y()) :
        Point(bbox.max.x() - inset, center.y());

    // Keep the fixture honest: the region masks supplied to the process plugin
    // must collectively cover the exact paths added below.
    const Polyline test_line(start, end);
    double covered_length = 0.;
    for (uint32_t region_idx = 0; region_idx < island.region_count(); ++region_idx) {
        const slic3r_api::ExPolygonCollection slices = island.region(region_idx).slices();
        const ExPolygons &native_slices = *reinterpret_cast<const ExPolygons *>(slices.handle());
        for (const Polyline &covered : intersection_pl(test_line, native_slices))
            covered_length += covered.length();
    }
    REQUIRE(covered_length == Approx(test_line.length()).margin(double(SCALED_EPSILON) * 2.));

    slic3r_api::RegionSettings fixture_settings(
        reinterpret_cast<storage_handle *>(&prepared.storage),
        island,
        {{region_setting_key}});
    fixture_settings.segregate(island.slice());
    const slic3r_api::RegionSettings::AreaMap &fixture_areas =
        fixture_settings.get_areas(region_setting_key);
    REQUIRE(fixture_areas.size() == 2);
    double segregated_length = 0.;
    for (const auto &entry : fixture_areas)
        for (const slic3r_api::Polyline &covered : entry.second.intersections(
                 slic3r_api::Polyline(reinterpret_cast<const polyline_handle *>(&test_line))))
            segregated_length += covered.length();
    REQUIRE(segregated_length == Approx(test_line.length()).margin(double(SCALED_EPSILON) * 2.));

    ExtrusionEntityCollection &root = *reinterpret_cast<ExtrusionEntityCollection *>(root_handle);
    for (size_t spec_idx = 0; spec_idx < specs.size(); ++spec_idx) {
        std::unique_ptr<ExtrusionPath> path = make_path(specs[spec_idx], spec_idx);
        path->polyline() = ArcPolyline();
        path->polyline().append(start);
        path->polyline().append(end);
        root.append(std::move(path));
    }
    prepared.source_roots.push_back(&root);
    prepared.source_layers.push_back(reinterpret_cast<Layer *>(
        const_cast<layer_handle *>(result.layer.handle())));

    Steps::StepExtrusionOrdering::clear(prepared.print);
    Steps::StepExtrusionOrdering::prepare(prepared.print);
    Steps::StepExtrusionOrdering::run_step(orchestrator, prepared.print);
    REQUIRE(prepared.print.printing_plan() != nullptr);
}

// Preserve the concise acceleration fixture calls while sharing all regional
// setup and validation with the speed scenarios below.
void prepare_regional_acceleration_plan(
    PreparedSpeedPrint &prepared,
    const DynamicPrintConfig &config,
    std::initializer_list<ConfigBase::SetDeserializeItem> modifier_overrides,
    const ExtrusionSpec &spec,
    bool path_inside_modifier = false)
{
    prepare_regional_process_plan(
        prepared, config, modifier_overrides, "default_acceleration", {spec},
        path_inside_modifier);
}

// Run any selected editor set through the host runner rather than invoking
// callbacks directly, preserving setup and parallel layer-run semantics.
void run_editors(PreparedSpeedPrint &prepared,
                 std::initializer_list<const char *> plugin_ids)
{
    ScopedActivePlugins active(plugin_ids);
    Orchestrator &orchestrator = Orchestrator::instance();
    orchestrator.reset_plugin_cancel();
    Steps::StepLayerExtrusionEdition::run_step(orchestrator, prepared.print);
    REQUIRE_FALSE(orchestrator.is_plugin_cancelled());
}

// Reproduce the default pipeline with speed first and acceleration second.
void run_default_editors(PreparedSpeedPrint &prepared)
{
    run_editors(prepared, {DEFAULT_SPEED_PLUGIN, DEFAULT_ACCELERATION_PLUGIN});
}

// Read effective values while walking the parent-inherited process state.
void collect_observed_leaves(const ExtrusionEntity &entity,
                             ProcessState state,
                             std::vector<ObservedLeaf> &out)
{
    const ExtrusionPropertySpeed *process = entity.get_property<ExtrusionPropertySpeed>();
    if (process != nullptr) {
        if (process->speed_mm_per_s > 0.f)
            state.speed = process->speed_mm_per_s;
        if (process->accel_mm_per_s2 > 0.f)
            state.acceleration = process->accel_mm_per_s2;
        if (process->fan_speed_percent >= 0.f)
            state.fan_speed = process->fan_speed_percent;
    }

    if (entity.child_count() > 0) {
        for (size_t child_idx = 0; child_idx < entity.child_count(); ++child_idx)
            collect_observed_leaves(entity.child(child_idx), state, out);
        return;
    }

    const ExtrusionAttributes *attributes = entity.get_property<ExtrusionAttributes>();
    if (attributes != nullptr && entity.has_polyline())
        out.push_back(ObservedLeaf{
            attributes->extrusion_role(), state.speed, state.acceleration, state.fan_speed,
            process,
            slic3r_api::ExtrusionEntity(
                reinterpret_cast<const extrusion_entity_handle *>(&entity)).property_count()});
}

// Collect every plan leaf. This helper intentionally preserves duplicate roles
// for autospeed tests where flow, rather than role, distinguishes the leaves.
std::vector<ObservedLeaf> observed_plan_leaves(const Print &print)
{
    std::vector<ObservedLeaf> out;
    REQUIRE(print.printing_plan() != nullptr);
    for (const PrintingGroup &group : print.printing_plan()->groups)
        for (const PrintingLayerGroup &layer : group.layers)
            for (const PrintingToolGroup &tool : layer.tool_groups)
                for (const PrintingExtrusion &extrusion : tool.extrusions) {
                    REQUIRE(extrusion.root != nullptr);
                    collect_observed_leaves(*extrusion.root, ProcessState{}, out);
                }
    return out;
}

// Return the only cloned root, ignoring empty layer-groups produced by slicing
// fixtures whose model height spans more than one physical layer.
ExtrusionEntity &single_plan_root(Print &print)
{
    REQUIRE(print.printing_plan() != nullptr);
    ExtrusionEntity *root = nullptr;
    size_t root_count = 0;
    for (PrintingGroup &group : print.mutable_printing_plan().groups)
        for (PrintingLayerGroup &layer : group.layers)
            for (PrintingToolGroup &tool : layer.tool_groups)
                for (PrintingExtrusion &extrusion : tool.extrusions) {
                    REQUIRE(extrusion.root != nullptr);
                    root = extrusion.root.get();
                    ++root_count;
                }
    REQUIRE(root_count == 1);
    REQUIRE(root != nullptr);
    return *root;
}

// Resolve observed values by role for scenarios containing one leaf per role.
std::map<uint16_t, ObservedLeaf> observed_by_role(const std::vector<ObservedLeaf> &leaves)
{
    std::map<uint16_t, ObservedLeaf> out;
    for (const ObservedLeaf &leaf : leaves)
        out[uint16_t(leaf.role())] = leaf;
    return out;
}

/*
Run speed editing on one `Skirt` leaf with `brim_speed` overrides and return
mm/s; the local Print owns the plan and failed REQUIRE checks abort the test.
*/
float resolved_brim_speed(std::initializer_list<ConfigBase::SetDeserializeItem> overrides)
{
    PreparedSpeedPrint prepared;
    DynamicPrintConfig config = speed_config({
        {"first_layer_speed", "100%"}, {"max_print_speed", "73"},
        {"max_volumetric_speed", "0"}, {"filament_max_speed", "0"},
        {"filament_max_volumetric_speed", "0"}, {"support_material_speed", "40"}
    });
    config.set_deserialize_strict(overrides);
    prepare_ordered_plan(prepared, config, {{{ExtrusionRole::Skirt, 0.2}}});
    run_editors(prepared, {DEFAULT_SPEED_PLUGIN});
    const std::vector<ObservedLeaf> leaves = observed_plan_leaves(prepared.print);
    REQUIRE(leaves.size() == 1);
    REQUIRE(leaves.front().role == ExtrusionRole::Skirt);
    return leaves.front().speed;
}

/*
Run acceleration editing on one `Skirt` leaf with support/brim overrides and
return mm/s²; the local Print owns the plan and failed REQUIRE checks abort it.
*/
float resolved_brim_acceleration(
    std::initializer_list<ConfigBase::SetDeserializeItem> overrides)
{
    PreparedSpeedPrint prepared;
    DynamicPrintConfig config = speed_config({
        {"default_acceleration", "1000"}, {"first_layer_acceleration", "100%"},
        {"machine_limits_usage", "ignore"}
    });
    config.set_deserialize_strict(overrides);
    prepare_ordered_plan(prepared, config, {{{ExtrusionRole::Skirt, 0.2}}});
    run_editors(prepared, {DEFAULT_ACCELERATION_PLUGIN});
    const std::vector<ObservedLeaf> leaves = observed_plan_leaves(prepared.print);
    REQUIRE(leaves.size() == 1);
    REQUIRE(leaves.front().role == ExtrusionRole::Skirt);
    return leaves.front().acceleration;
}

struct RecordingEditorState
{
    const char *id = nullptr;
    int32_t priority = 0;
    bool writes_marker = false;
    std::atomic_uint32_t setup_count{0};
    std::atomic_uint32_t setup_run_count{0};
    std::atomic_uint32_t run_count{0};
    std::atomic_uint32_t saw_previous_marker{0};
    std::atomic_bool payload_valid{true};
};

RecordingEditorState g_first_editor{"test.layer_edit.first", -20, true};
RecordingEditorState g_second_editor{"test.layer_edit.second", 20, false};

const char *recording_id(void *ctx) { return static_cast<RecordingEditorState *>(ctx)->id; }
const char *recording_name(void *ctx) { return static_cast<RecordingEditorState *>(ctx)->id; }
const char *recording_description(void *) { return "Records layer editor scheduling."; }
const char *recording_group(void *) { return ""; }
slicing_step_t recording_step(void *) { return STEP_LAYER_EXTRUSION_EDIT; }
int32_t recording_priority(void *ctx) { return static_cast<RecordingEditorState *>(ctx)->priority; }
const_strings_t recording_dependencies(void *) { return {}; }
int32_t recording_used_keys(void *, raw_used_config_key *) { return 0; }
int32_t recording_defined_keys(void *, const char **) { return 0; }
int32_t recording_initialize(void *, storage_handle *) { return PLUGIN_ERROR_OK; }

// Validate the setup payload and count the single setup call for one plugin.
void recording_setup(void *ctx, const plugin_run_context *run_ctx, uint32_t run_count)
{
    RecordingEditorState &state = *static_cast<RecordingEditorState *>(ctx);
    const run_ctx_layer_extrusion_edition *payload = plugin_ctx_as_layer_extrusion_edition(run_ctx);
    state.payload_valid = payload != nullptr && payload->print != nullptr && payload->plan != nullptr &&
                          payload->group != nullptr && payload->layer_group != nullptr && run_count == 2;
    ++state.setup_count;
}

// Every layer group receives setup_run before its plugin's parallel run phase.
void recording_setup_run(void *ctx, const plugin_run_context *run_ctx)
{
    RecordingEditorState &state = *static_cast<RecordingEditorState *>(ctx);
    const run_ctx_layer_extrusion_edition *payload = plugin_ctx_as_layer_extrusion_edition(run_ctx);
    if (payload == nullptr || payload->layer_group == nullptr || payload->group_idx != 0 ||
        payload->layer_group_idx >= 2)
        state.payload_valid = false;
    ++state.setup_run_count;
}

// The first plugin writes a harmless marker on every clone root. The second
// verifies it, proving the host completes one plugin before starting the next.
void recording_run(void *ctx, const plugin_run_context *run_ctx)
{
    RecordingEditorState &state = *static_cast<RecordingEditorState *>(ctx);
    const run_ctx_layer_extrusion_edition *payload = plugin_ctx_as_layer_extrusion_edition(run_ctx);
    if (payload == nullptr || payload->layer_group == nullptr) {
        state.payload_valid = false;
        return;
    }

    PrintingLayerGroup &layer = *reinterpret_cast<PrintingLayerGroup *>(payload->layer_group);
    for (PrintingToolGroup &tool : layer.tool_groups)
        for (PrintingExtrusion &extrusion : tool.extrusions) {
            if (extrusion.root == nullptr)
                continue;
            ExtrusionPropertySpeed &process = extrusion.root->get_or_add_property<ExtrusionPropertySpeed>();
            if (state.writes_marker)
                process.fan_speed_percent = 23.f;
            else if (process.fan_speed_percent == 23.f)
                ++state.saw_previous_marker;
        }
    ++state.run_count;
}

const plugin_vtable *recording_vtable()
{
    static const plugin_vtable table = {
        sizeof(plugin_vtable),
        &recording_id,
        &recording_name,
        &recording_description,
        &recording_group,
        &recording_step,
        &recording_dependencies,
        &recording_priority,
        &recording_used_keys,
        &recording_defined_keys,
        &recording_initialize,
        &recording_setup,
        &recording_setup_run,
        &recording_run
    };
    return &table;
}

// Register the two process-local scheduling probes once.
void register_recording_editors()
{
    Orchestrator &orchestrator = Orchestrator::instance();
    RecordingEditorState *states[] = {&g_first_editor, &g_second_editor};
    for (RecordingEditorState *state : states) {
        if (orchestrator.get_plugin(state->id) != nullptr)
            continue;
        plugin_instance instance = {state, recording_vtable()};
        REQUIRE(orchestrator.register_plugin(instance));
    }
}

// Reset atomics between tests because plugin instances live process-wide.
void reset_recording_editor(RecordingEditorState &state)
{
    state.setup_count = 0;
    state.setup_run_count = 0;
    state.run_count = 0;
    state.saw_previous_marker = 0;
    state.payload_valid = true;
}

/*
Obtain the timing contract id from the same orchestrator used by the plugins.
The id is deliberately looked up by name instead of cached process-wide so the
test follows the contract required by independent orchestrator instances.
*/
PluginPropertyKey<PrintingLayerTimeProperty> registered_layer_time_property_key()
{
    orchestrator_handle *orchestrator =
        reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance());
    return printing_layer_time_property_key(orchestrator);
}

} // namespace

TEST_CASE("Layer extrusion editors expose independent config contracts",
          "[plugins][layer-extrusion-edit][speed][acceleration][api]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Orchestrator &orchestrator = Orchestrator::instance();
    Plugin *speed_plugin = orchestrator.get_plugin(DEFAULT_SPEED_PLUGIN);
    Plugin *acceleration_plugin = orchestrator.get_plugin(DEFAULT_ACCELERATION_PLUGIN);
    REQUIRE(speed_plugin != nullptr);
    REQUIRE(acceleration_plugin != nullptr);
    CHECK(orchestrator.get_plugin("layer_extrusion_edit.speed_acceleration.default") == nullptr);
    CHECK(speed_plugin->get_step() == STEP_LAYER_EXTRUSION_EDIT);
    CHECK(acceleration_plugin->get_step() == STEP_LAYER_EXTRUSION_EDIT);
    CHECK(speed_plugin->get_priority() == 0);
    CHECK(acceleration_plugin->get_priority() == 10);
    CHECK(speed_plugin->get_exclusive_group() == "layer_extrusion_edit.speed");
    CHECK(acceleration_plugin->get_exclusive_group() == "layer_extrusion_edit.acceleration");
    // Keys the speed step reads; the plugin publishes exactly this set.
    std::vector<std::string> speed_defined;
    for (const Plugin::DefinedConfigKey &key : speed_plugin->get_defined_config_keys())
        speed_defined.push_back(key.key);
    CHECK(speed_defined.size() == 21);
    for (const char *expected : { "perimeter_speed", "bridge_speed", "max_print_speed",
                                  "filament_max_speed", "first_layer_flow_ratio", "milling_speed" })
        CHECK(std::find(speed_defined.begin(), speed_defined.end(), expected) != speed_defined.end());
    CHECK(std::find(speed_defined.begin(), speed_defined.end(), "travel_speed") == speed_defined.end());

    // Keys the acceleration step reads; the plugin publishes exactly this set.
    std::vector<std::string> acceleration_defined;
    for (const Plugin::DefinedConfigKey &key : acceleration_plugin->get_defined_config_keys())
        acceleration_defined.push_back(key.key);
    CHECK(acceleration_defined.size() == 16);
    for (const char *expected : { "default_acceleration", "bridge_acceleration",
                                  "machine_max_acceleration_travel", "travel_acceleration" })
        CHECK(std::find(acceleration_defined.begin(), acceleration_defined.end(), expected) !=
              acceleration_defined.end());
    CHECK(std::find(acceleration_defined.begin(), acceleration_defined.end(), "machine_limits_usage") ==
          acceleration_defined.end());

    std::vector<std::string> speed_keys;
    for (const Plugin::UsedConfigKey &key : speed_plugin->get_used_config_keys())
        speed_keys.push_back(key.key);
    CHECK(std::find(speed_keys.begin(), speed_keys.end(), "perimeter_speed") != speed_keys.end());
    CHECK(std::find(speed_keys.begin(), speed_keys.end(), "autospeed_min_thin_flow") != speed_keys.end());
    CHECK(std::find(speed_keys.begin(), speed_keys.end(), "filament_max_volumetric_speed") != speed_keys.end());
    CHECK(std::find(speed_keys.begin(), speed_keys.end(), "travel_speed") != speed_keys.end());
    CHECK(std::find(speed_keys.begin(), speed_keys.end(), "support_material_speed") != speed_keys.end());
    CHECK(std::find(speed_keys.begin(), speed_keys.end(), "default_acceleration") == speed_keys.end());

    std::vector<std::string> acceleration_keys;
    for (const Plugin::UsedConfigKey &key : acceleration_plugin->get_used_config_keys())
        acceleration_keys.push_back(key.key);
    CHECK(std::find(acceleration_keys.begin(), acceleration_keys.end(), "default_acceleration") !=
          acceleration_keys.end());
    CHECK(std::find(acceleration_keys.begin(), acceleration_keys.end(), "machine_limits_usage") !=
          acceleration_keys.end());
    CHECK(std::find(acceleration_keys.begin(), acceleration_keys.end(), "travel_acceleration") !=
          acceleration_keys.end());
    CHECK(std::find(acceleration_keys.begin(), acceleration_keys.end(), "machine_max_acceleration_travel") !=
          acceleration_keys.end());
    CHECK(std::find(acceleration_keys.begin(), acceleration_keys.end(), "support_material_acceleration") !=
          acceleration_keys.end());
    CHECK(std::find(acceleration_keys.begin(), acceleration_keys.end(), "perimeter_speed") ==
          acceleration_keys.end());
    CHECK(std::find(acceleration_keys.begin(), acceleration_keys.end(), "autospeed_min_thin_flow") ==
          acceleration_keys.end());

    DynamicPrintConfig config = speed_config({{"default_speed", "80"}, {"perimeter_speed", "50%"}});
    const double native_value = config.get_computed_value("perimeter_speed", 0);
    const slic3r_api::Config config_view(Slic3r::ApiHost::to_config_handle(&config));
    CHECK(config_view.computed_float_or_default("perimeter_speed", 0, -1.0) == Approx(native_value));
    CHECK(config_view.computed_float_or_default("missing_speed", 0, 17.0) == Approx(17.0));
}

TEST_CASE("Layer extrusion speed and acceleration plugins operate independently",
          "[plugins][layer-extrusion-edit][speed][acceleration][independence]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    SECTION("speed does not fill acceleration") {
        PreparedSpeedPrint prepared;
        const DynamicPrintConfig config = speed_config({
            {"perimeter_speed", "40"}, {"default_acceleration", "900"},
            {"first_layer_speed", "100%"}, {"first_layer_acceleration", "100%"},
            {"max_volumetric_speed", "0"}, {"filament_max_speed", "0"},
            {"filament_max_volumetric_speed", "0"}
        });
        prepare_ordered_plan(prepared, config, {{{ExtrusionRole::Perimeter, 0.2}}});

        run_editors(prepared, {DEFAULT_SPEED_PLUGIN});
        const std::vector<ObservedLeaf> leaves = observed_plan_leaves(prepared.print);
        REQUIRE(leaves.size() == 1);
        CHECK(leaves.front().speed == 40.f);
        CHECK(leaves.front().acceleration == -1.f);
    }

    SECTION("acceleration does not require flow or fill speed") {
        PreparedSpeedPrint prepared;
        const DynamicPrintConfig config = speed_config({
            {"perimeter_speed", "40"}, {"default_acceleration", "900"},
            {"perimeter_acceleration", "700"}, {"first_layer_acceleration", "100%"},
            {"machine_max_acceleration_extruding", "0"}
        });
        prepare_ordered_plan(prepared, config, {{{ExtrusionRole::Perimeter, 0.0}}});

        run_editors(prepared, {DEFAULT_ACCELERATION_PLUGIN});
        const std::vector<ObservedLeaf> leaves = observed_plan_leaves(prepared.print);
        REQUIRE(leaves.size() == 1);
        CHECK(leaves.front().speed == -1.f);
        CHECK(leaves.front().acceleration == 700.f);
    }

    SECTION("an unresolved travel acceleration remains optional") {
        PreparedSpeedPrint prepared;
        const DynamicPrintConfig config = speed_config({
            {"travel_speed", "80"}, {"default_acceleration", "0"},
            {"travel_acceleration", "0"}, {"machine_limits_usage", "ignore"}
        });
        prepare_ordered_plan(prepared, config, {{{ExtrusionRole::Travel, 0.0}}});

        run_editors(prepared, {DEFAULT_ACCELERATION_PLUGIN});
        const std::vector<ObservedLeaf> leaves = observed_plan_leaves(prepared.print);
        REQUIRE(leaves.size() == 1);
        CHECK(leaves.front().speed == -1.f);
        CHECK(leaves.front().acceleration == -1.f);
    }
}

TEST_CASE("Layer speed follows final values across region boundaries",
          "[plugins][layer-extrusion-edit][speed][regions]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    SECTION("different final values split a crossing leaf in traversal order") {
        PreparedSpeedPrint prepared;
        const DynamicPrintConfig config = speed_config({
            {"perimeter_speed", "70"}, {"first_layer_speed", "100%"},
            {"max_print_speed", "100"}, {"max_volumetric_speed", "0"},
            {"filament_max_speed", "0"}, {"filament_max_volumetric_speed", "0"}
        });
        ExtrusionSpec spec{ExtrusionRole::Perimeter, 0.2};
        spec.existing_acceleration = 321.f;
        spec.pressure_advance = 0.05f;
        spec.fan_speed = 42.f;
        prepare_regional_process_plan(
            prepared, config, {{"perimeter_speed", "30"}}, "perimeter_speed", {spec});

        run_editors(prepared, {DEFAULT_SPEED_PLUGIN});

        const std::vector<ObservedLeaf> leaves = observed_plan_leaves(prepared.print);
        REQUIRE(leaves.size() == 3);
        CHECK(leaves[0].speed == 70.f);
        CHECK(leaves[1].speed == 30.f);
        CHECK(leaves[2].speed == 70.f);
        for (const ObservedLeaf &leaf : leaves) {
            REQUIRE(leaf.direct_process != nullptr);
            CHECK(leaf.acceleration == 321.f);
            CHECK(leaf.direct_process->pressure_adv == 0.05f);
            CHECK(leaf.direct_process->fan_speed_percent == 42.f);
        }

        ExtrusionEntity &root = single_plan_root(prepared.print);
        REQUIRE(root.child_count() == 1);
        const ExtrusionEntity &split_leaf = root.child(0);
        REQUIRE(split_leaf.child_count() == 3);
        CHECK(split_leaf.child(0).last_point() == split_leaf.child(1).first_point());
        CHECK(split_leaf.child(1).last_point() == split_leaf.child(2).first_point());

        // Speed edits belong to the ordered plan clone; the source layer keeps
        // its original unsplit geometry and unrelated process fields.
        REQUIRE(prepared.source_roots.size() == 1);
        REQUIRE(prepared.source_roots.front()->child_count() == 1);
        CHECK(prepared.source_roots.front()->child(0).child_count() == 0);
        const ExtrusionPropertySpeed *source_process =
            prepared.source_roots.front()->child(0).get_property<ExtrusionPropertySpeed>();
        REQUIRE(source_process != nullptr);
        CHECK(source_process->speed_mm_per_s == -1.f);
        CHECK(source_process->accel_mm_per_s2 == 321.f);
    }

    SECTION("different raw settings capped to one final float do not split") {
        PreparedSpeedPrint prepared;
        const DynamicPrintConfig config = speed_config({
            {"perimeter_speed", "100"}, {"first_layer_speed", "100%"},
            {"max_print_speed", "50"}, {"max_volumetric_speed", "0"},
            {"filament_max_speed", "0"}, {"filament_max_volumetric_speed", "0"}
        });
        prepare_regional_process_plan(
            prepared, config, {{"perimeter_speed", "80"}}, "perimeter_speed",
            {ExtrusionSpec{ExtrusionRole::Perimeter, 0.2}});

        run_editors(prepared, {DEFAULT_SPEED_PLUGIN});

        const std::vector<ObservedLeaf> leaves = observed_plan_leaves(prepared.print);
        REQUIRE(leaves.size() == 1);
        CHECK(leaves.front().speed == 50.f);
        ExtrusionEntity &root = single_plan_root(prepared.print);
        REQUIRE(root.child_count() == 1);
        CHECK(root.child(0).child_count() == 0);
    }

    SECTION("base autospeed and explicit modifier are resolved independently") {
        PreparedSpeedPrint prepared;
        const DynamicPrintConfig config = speed_config({
            {"perimeter_speed", "0"}, {"first_layer_speed", "100%"},
            {"max_print_speed", "100"}, {"max_volumetric_speed", "12"},
            {"autospeed_min_thin_flow", "!0"}, {"filament_max_speed", "0"},
            {"filament_max_volumetric_speed", "0"}
        });
        prepare_regional_process_plan(
            prepared, config, {{"perimeter_speed", "30"}}, "perimeter_speed",
            {ExtrusionSpec{ExtrusionRole::Perimeter, 0.2}});

        run_editors(prepared, {DEFAULT_SPEED_PLUGIN});

        const std::vector<ObservedLeaf> leaves = observed_plan_leaves(prepared.print);
        REQUIRE(leaves.size() == 3);
        CHECK(leaves[0].speed == 60.f);
        CHECK(leaves[1].speed == 30.f);
        CHECK(leaves[2].speed == 60.f);
    }

    SECTION("an autospeed modifier contributes when the base setting is explicit") {
        PreparedSpeedPrint prepared;
        const DynamicPrintConfig config = speed_config({
            {"perimeter_speed", "30"}, {"first_layer_speed", "100%"},
            {"max_print_speed", "100"}, {"max_volumetric_speed", "12"},
            {"autospeed_min_thin_flow", "!0"}, {"filament_max_speed", "0"},
            {"filament_max_volumetric_speed", "0"}
        });
        prepare_regional_process_plan(
            prepared, config, {{"perimeter_speed", "0"}}, "perimeter_speed",
            {ExtrusionSpec{ExtrusionRole::Perimeter, 0.2}});

        run_editors(prepared, {DEFAULT_SPEED_PLUGIN});

        const std::vector<ObservedLeaf> leaves = observed_plan_leaves(prepared.print);
        REQUIRE(leaves.size() == 3);
        CHECK(leaves[0].speed == 30.f);
        CHECK(leaves[1].speed == 60.f);
        CHECK(leaves[2].speed == 30.f);
    }

    SECTION("an untouched autospeed region does not contribute to the group target") {
        PreparedSpeedPrint prepared;
        const DynamicPrintConfig config = speed_config({
            {"perimeter_speed", "0"}, {"infill_speed", "0"},
            {"solid_infill_speed", "0"},
            {"first_layer_speed", "100%"}, {"first_layer_infill_speed", "100%"},
            {"max_print_speed", "100"}, {"max_volumetric_speed", "50"},
            {"autospeed_min_thin_flow", "!0"}, {"filament_max_speed", "0"},
            {"filament_max_volumetric_speed", "0"}
        });
        prepare_regional_process_plan(
            prepared, config, {{"perimeter_speed", "30"}}, "perimeter_speed",
            {ExtrusionSpec{ExtrusionRole::Perimeter, 0.2},
             ExtrusionSpec{ExtrusionRole::InternalInfill, 0.4}}, true);

        run_editors(prepared, {DEFAULT_SPEED_PLUGIN});

        const std::map<uint16_t, ObservedLeaf> leaves =
            observed_by_role(observed_plan_leaves(prepared.print));
        REQUIRE(leaves.count(uint16_t(ExtrusionRole::Perimeter)) == 1);
        REQUIRE(leaves.count(uint16_t(ExtrusionRole::InternalInfill)) == 1);
        CHECK(leaves.at(uint16_t(ExtrusionRole::Perimeter)).speed == 30.f);
        CHECK(leaves.at(uint16_t(ExtrusionRole::InternalInfill)).speed == 100.f);
    }

    SECTION("a leaf contained in one region keeps its structure") {
        PreparedSpeedPrint prepared;
        const DynamicPrintConfig config = speed_config({
            {"perimeter_speed", "70"}, {"first_layer_speed", "100%"},
            {"max_print_speed", "100"}, {"max_volumetric_speed", "0"},
            {"filament_max_speed", "0"}, {"filament_max_volumetric_speed", "0"}
        });
        prepare_regional_process_plan(
            prepared, config, {{"perimeter_speed", "30"}}, "perimeter_speed",
            {ExtrusionSpec{ExtrusionRole::Perimeter, 0.2}}, true);

        run_editors(prepared, {DEFAULT_SPEED_PLUGIN});

        const std::vector<ObservedLeaf> leaves = observed_plan_leaves(prepared.print);
        REQUIRE(leaves.size() == 1);
        CHECK(leaves.front().speed == 30.f);
        ExtrusionEntity &root = single_plan_root(prepared.print);
        REQUIRE(root.child_count() == 1);
        CHECK(root.child(0).child_count() == 0);
    }

    SECTION("an existing effective speed bypasses regional splitting") {
        PreparedSpeedPrint prepared;
        const DynamicPrintConfig config = speed_config({
            {"perimeter_speed", "70"}, {"first_layer_speed", "100%"},
            {"max_print_speed", "100"}, {"max_volumetric_speed", "0"},
            {"filament_max_speed", "0"}, {"filament_max_volumetric_speed", "0"}
        });
        ExtrusionSpec spec{ExtrusionRole::Perimeter, 0.2};
        spec.existing_speed = 41.f;
        prepare_regional_process_plan(
            prepared, config, {{"perimeter_speed", "30"}}, "perimeter_speed", {spec});

        run_editors(prepared, {DEFAULT_SPEED_PLUGIN});

        const std::vector<ObservedLeaf> leaves = observed_plan_leaves(prepared.print);
        REQUIRE(leaves.size() == 1);
        CHECK(leaves.front().speed == 41.f);
        ExtrusionEntity &root = single_plan_root(prepared.print);
        REQUIRE(root.child_count() == 1);
        CHECK(root.child(0).child_count() == 0);
    }

    SECTION("flow-dependent caps produce distinct cached partitions") {
        PreparedSpeedPrint prepared;
        const DynamicPrintConfig config = speed_config({
            {"perimeter_speed", "100"}, {"first_layer_speed", "100%"},
            {"max_print_speed", "100"}, {"max_volumetric_speed", "12"},
            {"filament_max_speed", "0"}, {"filament_max_volumetric_speed", "0"}
        });
        prepare_regional_process_plan(
            prepared, config, {{"perimeter_speed", "50"}}, "perimeter_speed",
            {ExtrusionSpec{ExtrusionRole::Perimeter, 0.2},
             ExtrusionSpec{ExtrusionRole::Perimeter, 0.4}});

        run_editors(prepared, {DEFAULT_SPEED_PLUGIN});

        const std::vector<ObservedLeaf> leaves = observed_plan_leaves(prepared.print);
        REQUIRE(leaves.size() == 4);
        CHECK(leaves[0].speed == 60.f);
        CHECK(leaves[1].speed == 50.f);
        CHECK(leaves[2].speed == 60.f);
        CHECK(leaves[3].speed == 30.f);
        ExtrusionEntity &root = single_plan_root(prepared.print);
        REQUIRE(root.child_count() == 2);
        CHECK(root.child(0).child_count() == 3);
        CHECK(root.child(1).child_count() == 0);
    }
}

TEST_CASE("Deposits select regional process values without splitting their volume",
          "[plugins][deposit][speed][acceleration][regions][retraction][lift][travel][travel-deceleration]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const int location = GENERATE(0, 1, 2, 3);
    const bool base_deceleration = GENERATE(false, true);
    INFO("Deposit location index: " << location);
    PreparedSpeedPrint prepared;
    const DynamicPrintConfig config = speed_config({
        {"perimeter_speed", "70"}, {"default_acceleration", "700"},
        {"perimeter_acceleration", "100%"}, {"first_layer_speed", "100%"},
        {"first_layer_acceleration", "100%"}, {"max_print_speed", "100"},
        {"max_volumetric_speed", "0"}, {"filament_max_speed", "0"},
        {"filament_max_volumetric_speed", "0"}, {"retract_before_travel", "0"},
        {"retract_layer_change", "0"}, {"retract_length", "2"},
        {"print_retract_length", "1.5"}, {"print_retract_lift", "0.6"},
        {"travel_deceleration_use_target", base_deceleration ? "1" : "0"},
        {"avoid_crossing_perimeters", "1"}, {"avoid_crossing_not_first_layer", "0"},
        {"avoid_crossing_perimeters_max_detour", "0"}, {"machine_limits_usage", "ignore"},
        {"retract_lift_above", "0"}, {"retract_lift_below", "0"},
        {"retract_lift_first_layer", "0"}, {"retract_lift_top", "All surfaces"},
        {"retract_lift_before_travel", "0"}, {"travel_ramping_lift", "1"},
        {"travel_slope", "0"}, {"wipe", "0"}
    });
    prepare_regional_process_plan(prepared, config,
        {{"perimeter_speed", "30"}, {"default_acceleration", "300"},
         {"print_retract_length", "0.8"}, {"print_retract_lift", "0.3"},
         {"travel_deceleration_use_target", base_deceleration ? "0" : "1"},
         {"avoid_crossing_perimeters", "0"}},
        "perimeter_speed", {ExtrusionSpec{}, ExtrusionSpec{}, ExtrusionSpec{}});
    ExtrusionEntity &root = single_plan_root(prepared.print);
    REQUIRE(root.child_count() == 3);
    ExtrusionPath &deposit = dynamic_cast<ExtrusionPath &>(root.child(1));
    const Point center = (deposit.first_point() + deposit.last_point()) / 2;
    // The modifier is the central four-millimeter strip. Other choices test
    // the base region, the shared border, and a point outside every source area.
    Point point = center;
    if (location == 1)
        point.x() -= scale_i(5.0);
    else if (location == 2)
        point.x() -= scale_i(2.0);
    else if (location == 3)
        point.x() -= scale_i(30.0);
    const slic3r_api::PrintingPlan plan_view(
        reinterpret_cast<printing_plan_handle *>(&prepared.print.mutable_printing_plan()));
    const slic3r_api::ConstPrintingExtrusion extrusion =
        plan_view.group(0).layer_group(0).tool_group(0).extrusion(0);
    const slic3r_api::Config fallback = extrusion.region_island().region(0).print_region().config();
    const c_point shift = plan_view.object_instance_shift(extrusion);
    CHECK((shift.x != 0 || shift.y != 0));
    if (location == 2) {
        // Use the sliced modifier's exact edge rather than the nominal mesh
        // width, which can differ by a scaled coordinate after slicing.
        for (uint32_t region_idx = 0; region_idx < extrusion.region_island().region_count(); ++region_idx) {
            const slic3r_api::LayerRegion region = extrusion.region_island().region(region_idx);
            if (region.print_region().config().float_or_default("perimeter_speed", 0.0) != 30.0)
                continue;
            for (const slic3r_api::ExPolygon area : region.slices()) {
                coord_t min_x = area.contour().at(0).x;
                for (uint32_t idx = 1; idx < area.contour().size(); ++idx)
                    min_x = std::min(min_x, area.contour().at(idx).x);
                point.x() = min_x + shift.x;
            }
        }
        const slic3r_api::RegionSettingsPointResult lookup =
            slic3r_api::RegionSettings::lookup_at_point(extrusion.region_island(),
                {"perimeter_speed"}, c_point{point.x() - shift.x, point.y() - shift.y});
        REQUIRE(lookup.status == slic3r_api::RegionSettingsPointStatus::Ambiguous);
    }
    deposit.polyline() = ArcPolyline();
    deposit.polyline().append(point);
    deposit.add_property(ExtrusionPropertySpecialCommand(
        ExtrusionPropertySpecialCommand::Code::DEPOSIT, 0.5));

    run_default_editors(prepared);
    const std::vector<ObservedLeaf> leaves = observed_plan_leaves(prepared.print);
    REQUIRE(leaves.size() == 7); // Two crossing paths split into three; the deposit stays one.
    CHECK(leaves[3].speed == Approx(location == 0 ? 30.0 : location == 1 ? 70.0 :
                                   fallback.float_or_default("perimeter_speed", 0.0)));
    CHECK(leaves[3].acceleration == Approx(location == 0 ? 300.0 : location == 1 ? 700.0 :
                                          fallback.float_or_default("default_acceleration", 0.0)));
    CHECK(deposit.child_count() == 0);
    CHECK(deposit.polyline().size() == 1);
    CHECK(deposit.get_property<ExtrusionPropertySpecialCommand>()->extra_data == 0.5);
    CHECK(deposit.first_point() == point);

    // Perimeter routing sees a hole in the enabled base region. Region masks
    // stay unchanged so the same source still exercises ambiguous point lookup.
    Layer &routing_layer = *reinterpret_cast<Layer *>(
        const_cast<layer_handle *>(extrusion.region_island().region(0).layer().handle()));
    ExPolygons perimeter = routing_layer.lslices();
    REQUIRE(perimeter.size() == 1);
    const Point source_center(center.x() - shift.x, center.y() - shift.y);
    Polygon obstacle({
        source_center + Point(scale_i(4.0), scale_i(-1.0)),
        source_center + Point(scale_i(6.0), scale_i(-1.0)),
        source_center + Point(scale_i(6.0), scale_i(1.0)),
        source_center + Point(scale_i(4.0), scale_i(1.0))});
    obstacle.make_clockwise();
    perimeter.front().holes.push_back(std::move(obstacle));
    Slic3r::ApiInternal::LayerIslandAccess::perimeter_slices_mutable(
        routing_layer.islands().front()) = perimeter;
    Slic3r::ApiInternal::LayerIslandAccess::slice_mutable(
        routing_layer.islands().front()) = perimeter.front();
    const_cast<ExPolygons &>(routing_layer.lslices()) = perimeter;
    // Auxiliary support-layer routing also reads the nearest object layer's
    // perimeter boundary; expose the same obstacle in that borrowed layer.
    Layer *below = const_cast<Layer *>(prepared.print.object(0).get_first_layer_below_printz(
        routing_layer.scaled_print_z() + SCALED_EPSILON));
    REQUIRE(below != nullptr);
    Slic3r::ApiInternal::LayerIslandAccess::perimeter_slices_mutable(
        below->islands().front()) = perimeter;
    Slic3r::ApiInternal::LayerIslandAccess::slice_mutable(
        below->islands().front()) = perimeter.front();
    const_cast<ExPolygons &>(below->lslices()) = perimeter;
    PrintingToolGroup &tool = prepared.print.mutable_printing_plan().groups.front()
        .layers.front().tool_groups.front();
    const slic3r_api::LayerRegionIsland source_regions = extrusion.region_island();
    for (int control_idx = 0; control_idx < 2; ++control_idx) {
        const slic3r_api::RegionSettingsPointResult enabled =
            slic3r_api::RegionSettings::lookup_at_point(source_regions,
                {"avoid_crossing_perimeters"}, c_point{
                    source_center.x() + scale_i(control_idx == 0 ? 3.0 : 8.0), source_center.y()});
        REQUIRE(enabled.found());
        REQUIRE(enabled.value.get_bool());
        ExtrusionSpec control_spec;
        control_spec.existing_speed = 20.f;
        control_spec.existing_acceleration = 700.f;
        std::unique_ptr<ExtrusionPath> path = make_path(control_spec, 0);
        path->polyline() = ArcPolyline();
        path->polyline().append(center + Point(scale_i(control_idx == 0 ? 2.0 : 8.0), 0));
        path->polyline().append(center + Point(scale_i(control_idx == 0 ? 3.0 : 9.0), 0));
        PrintingExtrusion control;
        control.region_island = tool.extrusions.back().region_island;
        control.printing_catalog_id = tool.extrusions.back().printing_catalog_id;
        control.sregion_island_role = ExtrusionRole::Perimeter;
        control.root = std::move(path);
        tool.extrusions.insert(tool.extrusions.begin() + control_idx, std::move(control));
    }

    // Run the remaining consumers with real non-uniform source provenance.
    REQUIRE_NOTHROW(run_editors(prepared, {
        DEFAULT_TRANSITION_SCOPE_PLUGIN, "layer_extrusion_edit.retraction.default",
        DEFAULT_ENTRY_STATE_PLUGIN, "layer_extrusion_edit.travel.avoid_crossing_perimeters",
    }));
    const slic3r_api::LayerExtrusionEdit::ExtrusionScope::OrderedExtrusionScope scope(
        Slic3r::Test::TransitionPipeline::find_scope(
            Slic3r::Test::TransitionPipeline::entity_view(deposit)),
        Slic3r::Test::TransitionPipeline::scope_property_key());
    REQUIRE(scope.content().is_deposit());
    const slic3r_api::LayerExtrusionEdit::ExtrusionScope::OrderedExtrusionScope control(
        Slic3r::Test::TransitionPipeline::find_scope(
            Slic3r::Test::TransitionPipeline::entity_view(*tool.extrusions[1].root)),
        Slic3r::Test::TransitionPipeline::scope_property_key());
    REQUIRE(control.travel().point_count() > 2);
    CHECK(control.travel().point(0).x == center.x() + scale_i(3.0));
    CHECK(control.travel().point(control.travel().point_count() - 1).x == center.x() + scale_i(8.0));
    const slic3r_api::EPropertyExtrusionAxis *retract =
        scope.after().get(slic3r_api::EPropertyExtrusionAxis::key);
    REQUIRE(retract != nullptr);
    CHECK(retract->value == Approx(location == 0 ? 0.8 : location == 1 ? 1.5 :
                                  fallback.float_or_default("print_retract_length", 2.0)));
    if (location >= 2) {
        // Outside or ambiguous endpoints have no probe, so routing stays straight.
        REQUIRE(scope.travel().point_count() == 2);
        CHECK(scope.travel().point(1).x == point.x());
        CHECK(scope.travel().point(1).y == point.y());
    }
    slic3r_api::EPropertySpeed &travel_process = scope.travel().get_or_add(slic3r_api::EPropertySpeed::key);
    travel_process.speed_mm_per_s = 100.f;
    travel_process.accel_mm_per_s2 = 2000.f;
    REQUIRE_NOTHROW(run_editors(prepared, {"layer_extrusion_edit.lift.default"}));
    const slic3r_api::LayerExtrusionEdit::ExtrusionScope::OrderedExtrusionScope next(
        Slic3r::Test::TransitionPipeline::find_scope(
            Slic3r::Test::TransitionPipeline::entity_view(root.child(2))),
        Slic3r::Test::TransitionPipeline::scope_property_key());
    REQUIRE(next.travel().child_count() == 2);
    const slic3r_api::ExtrusionEntity ascending = next.travel().child(0);
    CHECK(unscaled(ascending.z_offset(ascending.point_count() - 1)) ==
          Approx(location == 0 ? 0.3 : location == 1 ? 0.6 :
                 fallback.float_or_default("print_retract_lift", 0.0)).margin(1e-6));
    REQUIRE_NOTHROW(run_editors(prepared, {"layer_extrusion_edit.travel_deceleration.default"}));
    if (location >= 2) {
        const bool use_target = fallback.bool_or_default("travel_deceleration_use_target", false);
        CHECK(source_regions.region(1).print_region().config().bool_or_default(
            "travel_deceleration_use_target", false) != use_target);
        std::vector<ProcessState> travel_values;
        // Lift and deceleration hoist attributes to wrappers. Resolve process
        // inheritance without requiring direct attributes on each geometric leaf.
        const auto collect_travel = [&travel_values](const auto &self,
            const ExtrusionEntity &entity, ProcessState state) -> void {
            if (const ExtrusionPropertySpeed *process = entity.get_property<ExtrusionPropertySpeed>()) {
                if (process->speed_mm_per_s > 0.f)
                    state.speed = process->speed_mm_per_s;
                if (process->accel_mm_per_s2 > 0.f)
                    state.acceleration = process->accel_mm_per_s2;
            }
            if (entity.child_count() == 0 && entity.has_polyline())
                travel_values.push_back(state);
            for (size_t idx = 0; idx < entity.child_count(); ++idx)
                self(self, entity.child(idx), state);
        };
        collect_travel(collect_travel,
            *reinterpret_cast<const ExtrusionEntity *>(scope.travel().handle()), ProcessState{});
        REQUIRE_FALSE(travel_values.empty());
        CHECK(travel_values.back().acceleration == Approx(use_target ? leaves[3].acceleration : 2000.f));
        CHECK(travel_values.back().speed == Approx(100.f));
    }
}

TEST_CASE("Layer acceleration follows final values across region boundaries",
          "[plugins][layer-extrusion-edit][acceleration][regions]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    SECTION("different final values split a crossing leaf in traversal order") {
        PreparedSpeedPrint prepared;
        const DynamicPrintConfig config = speed_config({
            {"default_acceleration", "1000"},
            {"perimeter_acceleration", "100%"},
            {"first_layer_acceleration", "100%"},
            {"machine_limits_usage", "ignore"}
        });
        ExtrusionSpec spec{ExtrusionRole::Perimeter, 0.2};
        spec.existing_speed = 37.f;
        spec.pressure_advance = 0.05f;
        spec.fan_speed = 42.f;
        prepare_regional_acceleration_plan(
            prepared, config, {{"default_acceleration", "400"}}, spec);

        run_editors(prepared, {DEFAULT_ACCELERATION_PLUGIN});

        const std::vector<ObservedLeaf> leaves = observed_plan_leaves(prepared.print);
        REQUIRE(leaves.size() == 3);
        CHECK(leaves[0].acceleration == 1000.f);
        CHECK(leaves[1].acceleration == 400.f);
        CHECK(leaves[2].acceleration == 1000.f);
        for (const ObservedLeaf &leaf : leaves) {
            REQUIRE(leaf.direct_process != nullptr);
            CHECK(leaf.speed == 37.f);
            CHECK(leaf.direct_process->pressure_adv == 0.05f);
            CHECK(leaf.direct_process->fan_speed_percent == 42.f);
        }

        ExtrusionEntity &root = single_plan_root(prepared.print);
        REQUIRE(root.child_count() == 1);
        const ExtrusionEntity &split_leaf = root.child(0);
        REQUIRE(split_leaf.child_count() == 3);
        CHECK(split_leaf.child(0).last_point() == split_leaf.child(1).first_point());
        CHECK(split_leaf.child(1).last_point() == split_leaf.child(2).first_point());

        // Ordering cloned the source tree before the acceleration plugin split
        // its plan copy. The layer-owned extrusion remains one unsplit path.
        REQUIRE(prepared.source_roots.size() == 1);
        REQUIRE(prepared.source_roots.front()->child_count() == 1);
        CHECK(prepared.source_roots.front()->child(0).child_count() == 0);
        const ExtrusionPropertySpeed *source_process =
            prepared.source_roots.front()->child(0).get_property<ExtrusionPropertySpeed>();
        REQUIRE(source_process != nullptr);
        CHECK(source_process->accel_mm_per_s2 == -1.f);
    }

    SECTION("different raw settings with one final float do not split") {
        PreparedSpeedPrint prepared;
        const DynamicPrintConfig config = speed_config({
            {"default_acceleration", "1000"},
            {"perimeter_acceleration", "100%"},
            {"first_layer_acceleration", "100%"},
            {"machine_limits_usage", "limits"},
            {"machine_max_acceleration_extruding", "500"}
        });
        prepare_regional_acceleration_plan(
            prepared, config, {{"default_acceleration", "800"}},
            ExtrusionSpec{ExtrusionRole::Perimeter, 0.2});

        run_editors(prepared, {DEFAULT_ACCELERATION_PLUGIN});

        const std::vector<ObservedLeaf> leaves = observed_plan_leaves(prepared.print);
        REQUIRE(leaves.size() == 1);
        CHECK(leaves.front().acceleration == 500.f);
        ExtrusionEntity &root = single_plan_root(prepared.print);
        REQUIRE(root.child_count() == 1);
        CHECK(root.child(0).child_count() == 0);
        CHECK(root.child(0).has_polyline());
    }

    SECTION("an unresolved region remains distinct from a positive value") {
        PreparedSpeedPrint prepared;
        const DynamicPrintConfig config = speed_config({
            {"default_acceleration", "0"},
            {"perimeter_acceleration", "100%"},
            {"first_layer_acceleration", "100%"},
            {"machine_limits_usage", "ignore"}
        });
        prepare_regional_acceleration_plan(
            prepared, config, {{"default_acceleration", "400"}},
            ExtrusionSpec{ExtrusionRole::Perimeter, 0.2});

        run_editors(prepared, {DEFAULT_ACCELERATION_PLUGIN});

        const std::vector<ObservedLeaf> leaves = observed_plan_leaves(prepared.print);
        REQUIRE(leaves.size() == 3);
        CHECK(leaves[0].acceleration == -1.f);
        CHECK(leaves[1].acceleration == 400.f);
        CHECK(leaves[2].acceleration == -1.f);
    }

    SECTION("a leaf contained in one region keeps its structure") {
        PreparedSpeedPrint prepared;
        const DynamicPrintConfig config = speed_config({
            {"default_acceleration", "1000"},
            {"perimeter_acceleration", "100%"},
            {"first_layer_acceleration", "100%"},
            {"machine_limits_usage", "ignore"}
        });
        prepare_regional_acceleration_plan(
            prepared, config, {{"default_acceleration", "400"}},
            ExtrusionSpec{ExtrusionRole::Perimeter, 0.2}, true);

        run_editors(prepared, {DEFAULT_ACCELERATION_PLUGIN});

        const std::vector<ObservedLeaf> leaves = observed_plan_leaves(prepared.print);
        REQUIRE(leaves.size() == 1);
        CHECK(leaves.front().acceleration == 400.f);
        ExtrusionEntity &root = single_plan_root(prepared.print);
        REQUIRE(root.child_count() == 1);
        CHECK(root.child(0).child_count() == 0);
    }

    SECTION("an existing effective acceleration bypasses regional splitting") {
        PreparedSpeedPrint prepared;
        const DynamicPrintConfig config = speed_config({
            {"default_acceleration", "1000"},
            {"perimeter_acceleration", "100%"},
            {"first_layer_acceleration", "100%"},
            {"machine_limits_usage", "ignore"}
        });
        ExtrusionSpec spec{ExtrusionRole::Perimeter, 0.2};
        spec.existing_acceleration = 321.f;
        prepare_regional_acceleration_plan(
            prepared, config, {{"default_acceleration", "400"}}, spec);

        run_editors(prepared, {DEFAULT_ACCELERATION_PLUGIN});

        const std::vector<ObservedLeaf> leaves = observed_plan_leaves(prepared.print);
        REQUIRE(leaves.size() == 1);
        CHECK(leaves.front().acceleration == 321.f);
        ExtrusionEntity &root = single_plan_root(prepared.print);
        REQUIRE(root.child_count() == 1);
        CHECK(root.child(0).child_count() == 0);
    }
}

TEST_CASE("Layer extrusion editor mutates plan clones and hoists uniform fields",
          "[plugins][layer-extrusion-edit][speed][acceleration][hoisting]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedSpeedPrint prepared;
    const DynamicPrintConfig config = speed_config({
        {"perimeter_speed", "40"},
        {"default_acceleration", "1000"},
        {"perimeter_acceleration", "0"},
        {"first_layer_speed", "100%"},
        {"first_layer_acceleration", "100%"},
        {"max_volumetric_speed", "0"},
        {"filament_max_speed", "0"},
        {"filament_max_volumetric_speed", "0"}
    });
    prepare_ordered_plan(prepared, config, {{
        ExtrusionSpec{ExtrusionRole::Perimeter, 0.2},
        ExtrusionSpec{ExtrusionRole::Perimeter, 0.3}
    }});

    REQUIRE(prepared.source_roots.size() == 1);
    CHECK(prepared.source_roots.front()->get_property<ExtrusionPropertySpeed>() == nullptr);
    CHECK(prepared.source_roots.front()->child(0).get_property<ExtrusionPropertySpeed>() == nullptr);
    REQUIRE(prepared.source_layers.size() == 1);
    CHECK(layer_get_object(reinterpret_cast<const layer_handle *>(prepared.source_layers.front())) ==
          reinterpret_cast<const object_handle *>(&prepared.print.object(0)));

    run_default_editors(prepared);

    ExtrusionEntity &root = single_plan_root(prepared.print);
    const ExtrusionPropertySpeed *root_process = root.get_property<ExtrusionPropertySpeed>();
    REQUIRE(root_process != nullptr);
    CHECK(root_process->speed_mm_per_s == 40.f);
    CHECK(root_process->accel_mm_per_s2 == 1000.f);
    CHECK(root_process->pressure_adv == -1.f);
    CHECK(root_process->fan_speed_percent == -1.f);
    REQUIRE(root.child_count() == 2);
    CHECK(root.child(0).get_property<ExtrusionPropertySpeed>() == nullptr);
    CHECK(root.child(1).get_property<ExtrusionPropertySpeed>() == nullptr);

    // Ordering cloned the source before editing, so generated Layer output is
    // still free of process values owned by this downstream plan step.
    CHECK(prepared.source_roots.front()->get_property<ExtrusionPropertySpeed>() == nullptr);
    CHECK(prepared.source_roots.front()->child(0).get_property<ExtrusionPropertySpeed>() == nullptr);
}

TEST_CASE("Travel leaves receive independent speed and acceleration",
          "[plugins][layer-extrusion-edit][travel][speed][acceleration]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    SECTION("generated travel uses travel settings and its own machine limit") {
        PreparedSpeedPrint prepared;
        const DynamicPrintConfig config = speed_config({
            {"perimeter_speed", "0"}, {"travel_speed", "123"},
            {"default_acceleration", "1000"}, {"perimeter_acceleration", "800"},
            {"travel_acceleration", "1500"}, {"first_layer_speed", "100%"},
            {"first_layer_acceleration", "100%"}, {"max_print_speed", "50"},
            {"max_volumetric_speed", "12"},
            {"filament_max_speed", "0"}, {"filament_max_volumetric_speed", "0"},
            {"machine_limits_usage", "limits"},
            {"machine_max_acceleration_extruding", "700"},
            {"machine_max_acceleration_travel", "900"}
        });
        prepare_ordered_plan(prepared, config, {{
            ExtrusionSpec{ExtrusionRole::Perimeter, 0.2},
            ExtrusionSpec{ExtrusionRole::Perimeter, 0.2}
        }});

        run_editors(prepared, {
            DEFAULT_TRANSITION_SCOPE_PLUGIN, DEFAULT_ENTRY_STATE_PLUGIN, DEFAULT_TRAVEL_PLUGIN,
            DEFAULT_SPEED_PLUGIN, DEFAULT_ACCELERATION_PLUGIN
        });

        const std::vector<ObservedLeaf> leaves = observed_plan_leaves(prepared.print);
        size_t travel_count = 0;
        for (const ObservedLeaf &leaf : leaves) {
            if (leaf.role == ExtrusionRole::Travel) {
                ++travel_count;
                CHECK(leaf.speed == 123.f);
                CHECK(leaf.acceleration == 900.f);
            } else {
                CHECK(leaf.speed == 50.f);
                CHECK(leaf.acceleration == 700.f);
            }
        }
        CHECK(travel_count == 1);

        // Ordering cloned before travel generation. The source layer therefore
        // remains two printable leaves with no generated connector or process data.
        REQUIRE(prepared.source_roots.size() == 1);
        REQUIRE(prepared.source_roots.front()->child_count() == 2);
        for (size_t child_idx = 0; child_idx < prepared.source_roots.front()->child_count(); ++child_idx) {
            const ExtrusionEntity &source = prepared.source_roots.front()->child(child_idx);
            const ExtrusionAttributes *attributes = source.get_property<ExtrusionAttributes>();
            REQUIRE(attributes != nullptr);
            CHECK(attributes->extrusion_role() == ExtrusionRole::Perimeter);
            CHECK(source.get_property<ExtrusionPropertySpeed>() == nullptr);
        }
    }

    SECTION("pre-existing travel process values remain authoritative") {
        PreparedSpeedPrint prepared;
        const DynamicPrintConfig config = speed_config({
            {"travel_speed", "123"}, {"default_acceleration", "1000"},
            {"travel_acceleration", "1500"}, {"machine_limits_usage", "limits"},
            {"machine_max_acceleration_travel", "900"}
        });
        ExtrusionSpec travel{ExtrusionRole::Travel, 0.0};
        travel.existing_speed = 33.f;
        travel.existing_acceleration = 444.f;
        prepare_ordered_plan(prepared, config, {{travel}});

        run_editors(prepared, {DEFAULT_SPEED_PLUGIN, DEFAULT_ACCELERATION_PLUGIN});

        const std::vector<ObservedLeaf> leaves = observed_plan_leaves(prepared.print);
        REQUIRE(leaves.size() == 1);
        CHECK(leaves.front().role == ExtrusionRole::Travel);
        CHECK(leaves.front().speed == 33.f);
        CHECK(leaves.front().acceleration == 444.f);
    }

    SECTION("travel acceleration resolves percentages against the default") {
        PreparedSpeedPrint prepared;
        const DynamicPrintConfig config = speed_config({
            {"travel_speed", "80"}, {"default_acceleration", "1000"},
            {"travel_acceleration", "150%"}, {"machine_limits_usage", "ignore"}
        });
        prepare_ordered_plan(prepared, config, {{
            ExtrusionSpec{ExtrusionRole::Travel, 0.0}
        }});

        run_editors(prepared, {DEFAULT_SPEED_PLUGIN, DEFAULT_ACCELERATION_PLUGIN});

        const std::vector<ObservedLeaf> leaves = observed_plan_leaves(prepared.print);
        REQUIRE(leaves.size() == 1);
        CHECK(leaves.front().speed == 80.f);
        CHECK(leaves.front().acceleration == 1500.f);
    }
}

TEST_CASE("Layer extrusion editor resolves role speed and acceleration fallbacks",
          "[plugins][layer-extrusion-edit][speed][acceleration][roles]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedSpeedPrint prepared;
    const DynamicPrintConfig config = speed_config({
        {"perimeter_speed", "31"}, {"external_perimeter_speed", "29"},
        {"overhangs", "1"}, {"overhangs_speed", "23"},
        {"bridge_speed", "27"}, {"internal_bridge_speed", "19"},
        {"infill_speed", "61"}, {"solid_infill_speed", "53"},
        {"top_solid_infill_speed", "47"}, {"thin_walls_speed", "17"},
        {"gap_fill_speed", "13"}, {"ironing_speed", "11"}, {"brim_speed", "7"},
        {"default_acceleration", "1000"}, {"perimeter_acceleration", "900"},
        {"external_perimeter_acceleration", "800"}, {"overhangs_acceleration", "700"},
        {"bridge_acceleration", "600"}, {"internal_bridge_acceleration", "500"},
        {"infill_acceleration", "400"}, {"solid_infill_acceleration", "300"},
        {"top_solid_infill_acceleration", "250"}, {"thin_walls_acceleration", "200"},
        {"gap_fill_acceleration", "150"}, {"ironing_acceleration", "100"},
        {"brim_acceleration", "50"}, {"first_layer_speed", "100%"},
        {"first_layer_acceleration", "100%"}, {"max_volumetric_speed", "0"},
        {"filament_max_speed", "0"}, {"filament_max_volumetric_speed", "0"}
    });
    prepare_ordered_plan(prepared, config, {{
        {ExtrusionRole::Perimeter, 0.2}, {ExtrusionRole::ExternalPerimeter, 0.2},
        {ExtrusionRole::OverhangPerimeter, 0.2}, {ExtrusionRole::BridgeInfill, 0.2},
        {ExtrusionRole::InternalBridgeInfill, 0.2}, {ExtrusionRole::InternalInfill, 0.2},
        {ExtrusionRole::SolidInfill, 0.2}, {ExtrusionRole::TopSolidInfill, 0.2},
        {ExtrusionRole::ThinWall, 0.2}, {ExtrusionRole::GapFill, 0.2},
        {ExtrusionRole::Ironing, 0.2}, {ExtrusionRole::Skirt, 0.2}
    }});

    run_default_editors(prepared);
    const std::map<uint16_t, ObservedLeaf> leaves = observed_by_role(observed_plan_leaves(prepared.print));
    const auto check_role = [&leaves](ExtrusionRole role, float speed, float acceleration) {
        const std::map<uint16_t, ObservedLeaf>::const_iterator found = leaves.find(uint16_t(role()));
        REQUIRE(found != leaves.end());
        CHECK(found->second.speed == speed);
        CHECK(found->second.acceleration == acceleration);
    };
    check_role(ExtrusionRole::Perimeter, 31.f, 900.f);
    check_role(ExtrusionRole::ExternalPerimeter, 29.f, 800.f);
    check_role(ExtrusionRole::OverhangPerimeter, 23.f, 700.f);
    check_role(ExtrusionRole::BridgeInfill, 27.f, 600.f);
    check_role(ExtrusionRole::InternalBridgeInfill, 19.f, 500.f);
    check_role(ExtrusionRole::InternalInfill, 61.f, 400.f);
    check_role(ExtrusionRole::SolidInfill, 53.f, 300.f);
    check_role(ExtrusionRole::TopSolidInfill, 47.f, 250.f);
    check_role(ExtrusionRole::ThinWall, 17.f, 200.f);
    check_role(ExtrusionRole::GapFill, 13.f, 150.f);
    check_role(ExtrusionRole::Ironing, 11.f, 100.f);
    check_role(ExtrusionRole::Skirt, 7.f, 50.f);
}

/*
Initialize the plugin runtime, build one in-memory `Skirt` role leaf (the role
used by skirt/brim paths), run the speed editor, and check fixed, percentage,
default and zero values in mm/s. The local print owns each plan; Catch2 reports
setup or assertion failures and no files are exported.
*/
TEST_CASE("Layer extrusion editor resolves brim speed over the support speed",
          "[plugins][layer-extrusion-edit][speed][brim]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    const ConfigOptionDef *def = PrintConfigDef::instance().get("brim_speed");
    REQUIRE(def != nullptr);
    CHECK(def->ratio_over == "support_material_speed");
    CHECK(def->default_value->serialize() == "50%");

    SECTION("a fixed brim speed is used as is") {
        CHECK(resolved_brim_speed({{"brim_speed", "25"}}) == 25.f);
    }

    SECTION("a brim percentage applies to the support speed") {
        CHECK(resolved_brim_speed({{"brim_speed", "50%"}}) == 20.f);
    }

    SECTION("the default brim speed is half the support speed") {
        CHECK(resolved_brim_speed({}) == 20.f);
    }

    SECTION("a zero brim speed falls back to max print speed without autospeed") {
        CHECK(resolved_brim_speed({{"brim_speed", "0"}}) == 73.f);
    }

    SECTION("a zero brim speed uses autospeed, not the support speed") {
        CHECK(resolved_brim_speed({{"brim_speed", "0"}, {"max_print_speed", "100"},
                                   {"max_volumetric_speed", "20"}}) == 100.f);
    }

    SECTION("zero brim speed uses the autospeed base, not max_print_speed") {
        CHECK(resolved_brim_speed({{"brim_speed", "0"}, {"max_print_speed", "100"},
                                   {"max_volumetric_speed", "20"},
                                   {"autospeed_min_thin_flow", "8"}}) == 40.f);
    }
}

/*
Initialize the plugin runtime, build one in-memory `Skirt` role leaf (the role
used by skirt/brim paths), run the acceleration editor, and check fixed,
percentage, zero-inheritance and first-layer results in mm/s². The local print
owns each plan; Catch2 reports setup or assertion failures.
*/
TEST_CASE("Layer extrusion editor resolves brim acceleration over the support acceleration",
          "[plugins][layer-extrusion-edit][acceleration][brim]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    SECTION("a brim percentage applies to support acceleration resolved from default") {
        CHECK(resolved_brim_acceleration({{"support_material_acceleration", "60%"},
                                          {"brim_acceleration", "50%"}}) == 300.f);
    }

    SECTION("a fixed brim acceleration is used as is") {
        CHECK(resolved_brim_acceleration({{"brim_acceleration", "700"}}) == 700.f);
    }

    SECTION("zero brim acceleration inherits support acceleration") {
        CHECK(resolved_brim_acceleration({{"support_material_acceleration", "600"},
                                          {"brim_acceleration", "0"}}) == 600.f);
    }

    SECTION("zero brim acceleration inherits 80 percent support acceleration") {
        CHECK(resolved_brim_acceleration({{"support_material_acceleration", "80%"},
                                          {"brim_acceleration", "0"}}) == 800.f);
    }

    SECTION("a zero support acceleration leaves the brim percentage based on default") {
        CHECK(resolved_brim_acceleration({{"support_material_acceleration", "0"},
                                          {"brim_acceleration", "40%"}}) == 400.f);
    }

    SECTION("the first-layer ceiling caps brim acceleration") {
        CHECK(resolved_brim_acceleration({{"support_material_acceleration", "150%"},
                                          {"brim_acceleration", "150%"},
                                          {"first_layer_acceleration", "1800"}}) == 1800.f);
    }
}

TEST_CASE("Layer extrusion editor preserves overrides and applies first-layer caps",
          "[plugins][layer-extrusion-edit][speed][acceleration][limits]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedSpeedPrint prepared;
    const DynamicPrintConfig config = speed_config({
        {"perimeter_speed", "100"}, {"default_acceleration", "2000"},
        {"perimeter_acceleration", "1500"}, {"first_layer_speed", "50%"},
        {"first_layer_min_speed", "60"}, {"first_layer_acceleration", "50%"},
        {"first_layer_flow_ratio", "200%"}, {"max_volumetric_speed", "20"},
        {"filament_max_volumetric_speed", "12"}, {"filament_max_speed", "25"},
        {"machine_limits_usage", "emit_to_gcode"},
        {"machine_max_acceleration_extruding", "500"}
    });
    ExtrusionSpec spec{ExtrusionRole::Perimeter, 0.2};
    spec.existing_speed = 18.f;
    spec.pressure_advance = 0.07f;
    spec.fan_speed = 35.f;
    ExtrusionSpec acceleration_override{ExtrusionRole::Perimeter, 0.2};
    acceleration_override.existing_acceleration = 321.f;
    prepare_ordered_plan(prepared, config, {{
        spec,
        acceleration_override,
        ExtrusionSpec{ExtrusionRole::Perimeter, 0.2}
    }});

    run_default_editors(prepared);
    const std::vector<ObservedLeaf> leaves = observed_plan_leaves(prepared.print);
    REQUIRE(leaves.size() == 3);

    bool found_existing_speed = false;
    bool found_existing_acceleration = false;
    bool found_fully_resolved = false;
    for (const ObservedLeaf &leaf : leaves) {
        REQUIRE(leaf.direct_process != nullptr);
        if (leaf.direct_process->pressure_adv == 0.07f) {
            found_existing_speed = true;
            CHECK(leaf.speed == 18.f);
            CHECK(leaf.acceleration == 500.f);
            CHECK(leaf.direct_process->fan_speed_percent == 35.f);
        } else if (leaf.acceleration == 321.f) {
            found_existing_acceleration = true;
            CHECK(leaf.speed == 25.f);
        } else {
            found_fully_resolved = true;
            CHECK(leaf.speed == 25.f);
            CHECK(leaf.acceleration == 500.f);
        }
    }
    CHECK(found_existing_speed);
    CHECK(found_existing_acceleration);
    CHECK(found_fully_resolved);
}

TEST_CASE("Layer extrusion editor caps explicit role speed with max print speed",
          "[plugins][layer-extrusion-edit][speed][limits]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedSpeedPrint prepared;
    const DynamicPrintConfig config = speed_config({
        {"perimeter_speed", "100"}, {"default_acceleration", "1000"},
        {"max_print_speed", "42"}, {"max_volumetric_speed", "0"},
        {"first_layer_speed", "100%"}, {"first_layer_acceleration", "100%"},
        {"filament_max_speed", "0"}, {"filament_max_volumetric_speed", "0"}
    });
    prepare_ordered_plan(prepared, config, {{{ExtrusionRole::Perimeter, 0.2}}});

    run_default_editors(prepared);
    const std::vector<ObservedLeaf> leaves = observed_plan_leaves(prepared.print);
    REQUIRE(leaves.size() == 1);
    CHECK(leaves.front().speed == 42.f);
}

TEST_CASE("Layer extrusion editor computes autospeed from real eligible flow",
          "[plugins][layer-extrusion-edit][speed][autospeed]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedSpeedPrint prepared;
    const DynamicPrintConfig config = speed_config({
        {"perimeter_speed", "0"}, {"default_acceleration", "1000"},
        {"max_print_speed", "100"}, {"max_volumetric_speed", "50"},
        {"autospeed_min_thin_flow", "!0"}, {"first_layer_speed", "100%"},
        {"first_layer_acceleration", "100%"}, {"filament_max_speed", "0"},
        {"filament_max_volumetric_speed", "0"}
    });
    ExtrusionSpec forced{ExtrusionRole::Perimeter, 0.001};
    forced.force_e_per_mm = true;
    prepare_ordered_plan(prepared, config, {{
        {ExtrusionRole::Perimeter, 0.2},
        {ExtrusionRole::Perimeter, 0.4},
        {ExtrusionRole::SupportMaterial, 0.01},
        {ExtrusionRole::WipeTower, 0.01},
        forced
    }});

    run_default_editors(prepared);
    const std::vector<ObservedLeaf> leaves = observed_plan_leaves(prepared.print);
    REQUIRE(leaves.size() == 5);
    std::vector<float> editable_speeds;
    size_t excluded_count = 0;
    for (const ObservedLeaf &leaf : leaves) {
        if (leaf.speed > 0.f)
            editable_speeds.push_back(leaf.speed);
        else
            ++excluded_count;
    }
    std::sort(editable_speeds.begin(), editable_speeds.end());
    REQUIRE(editable_speeds.size() == 3);
    CHECK(editable_speeds[0] == 50.f);
    CHECK(editable_speeds[1] == 100.f);
    CHECK(editable_speeds[2] == 100.f);
    CHECK(excluded_count == 2);
}

TEST_CASE("Layer extrusion editor supports autospeed percentages and thin-flow floor",
          "[plugins][layer-extrusion-edit][speed][autospeed]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedSpeedPrint prepared;
    const DynamicPrintConfig config = speed_config({
        {"perimeter_speed", "50%"}, {"gap_fill_speed", "0"},
        {"default_acceleration", "1000"}, {"max_print_speed", "100"},
        {"max_volumetric_speed", "20"}, {"autospeed_min_thin_flow", "50%"},
        {"first_layer_speed", "100%"}, {"first_layer_acceleration", "100%"},
        {"filament_max_speed", "0"}, {"filament_max_volumetric_speed", "0"}
    });
    prepare_ordered_plan(prepared, config, {{
        {ExtrusionRole::Perimeter, 0.2},
        {ExtrusionRole::GapFill, 0.01}
    }});

    run_default_editors(prepared);
    const std::map<uint16_t, ObservedLeaf> leaves = observed_by_role(observed_plan_leaves(prepared.print));
    REQUIRE(leaves.count(uint16_t(ExtrusionRole::Perimeter)) == 1);
    REQUIRE(leaves.count(uint16_t(ExtrusionRole::GapFill)) == 1);
    CHECK(leaves.at(uint16_t(ExtrusionRole::Perimeter)).speed == 25.f);
    CHECK(leaves.at(uint16_t(ExtrusionRole::GapFill)).speed == 50.f);
}

TEST_CASE("Layer extrusion editor isolates autospeed by printing group and extruder",
          "[plugins][layer-extrusion-edit][speed][autospeed]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedSpeedPrint prepared;
    const DynamicPrintConfig config = speed_config({
        {"perimeter_speed", "0"}, {"default_acceleration", "1000"},
        {"max_print_speed", "100"}, {"max_volumetric_speed", "100"},
        {"autospeed_min_thin_flow", "!0"}, {"first_layer_speed", "100%"},
        {"first_layer_acceleration", "100%"}, {"filament_max_speed", "0,0"},
        {"filament_max_volumetric_speed", "0,12"}, {"nozzle_diameter", "0.4,0.4"}
    });
    prepare_ordered_plan(prepared, config, {
        {{ExtrusionRole::Perimeter, 0.2}, {ExtrusionRole::Perimeter, 0.4}},
        {{ExtrusionRole::Perimeter, 0.4}, {ExtrusionRole::Perimeter, 0.4}}
    });

    PrintingPlan &plan = prepared.print.mutable_printing_plan();
    REQUIRE(plan.groups.size() == 1);
    REQUIRE(plan.groups.front().layers.size() == 2);

    // Complete-object printing computes autospeed independently per group. Move
    // the second prepared layer into a second group and assign another actual
    // extruder so both dimensions of the target lookup are exercised.
    PrintingLayerGroup second_layer = std::move(plan.groups.front().layers.back());
    plan.groups.front().layers.pop_back();
    REQUIRE(second_layer.tool_groups.size() == 1);
    second_layer.tool_groups.front().extruder_id = 1;
    PrintingGroup second_group;
    second_group.layers.push_back(std::move(second_layer));
    plan.groups.push_back(std::move(second_group));

    run_default_editors(prepared);
    REQUIRE(plan.groups.size() == 2);

    std::vector<ObservedLeaf> first_group_leaves;
    for (const PrintingExtrusion &extrusion : plan.groups[0].layers[0].tool_groups[0].extrusions)
        collect_observed_leaves(*extrusion.root, ProcessState{}, first_group_leaves);
    std::vector<ObservedLeaf> second_group_leaves;
    for (const PrintingExtrusion &extrusion : plan.groups[1].layers[0].tool_groups[0].extrusions)
        collect_observed_leaves(*extrusion.root, ProcessState{}, second_group_leaves);

    REQUIRE(first_group_leaves.size() == 2);
    REQUIRE(second_group_leaves.size() == 2);
    std::vector<float> first_speeds{first_group_leaves[0].speed, first_group_leaves[1].speed};
    std::sort(first_speeds.begin(), first_speeds.end());
    CHECK(first_speeds[0] == 50.f);
    CHECK(first_speeds[1] == 100.f);
    CHECK(second_group_leaves[0].speed == 30.f);
    CHECK(second_group_leaves[1].speed == 30.f);
}

TEST_CASE("Layer extrusion editor falls back to max print speed without autospeed",
          "[plugins][layer-extrusion-edit][speed][autospeed]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedSpeedPrint prepared;
    const DynamicPrintConfig config = speed_config({
        {"perimeter_speed", "0"}, {"default_acceleration", "1000"},
        {"max_print_speed", "73"}, {"max_volumetric_speed", "0"},
        {"first_layer_speed", "100%"}, {"first_layer_acceleration", "100%"},
        {"filament_max_speed", "0"}, {"filament_max_volumetric_speed", "0"}
    });
    prepare_ordered_plan(prepared, config, {{{ExtrusionRole::Perimeter, 0.2}}});

    run_default_editors(prepared);
    const std::vector<ObservedLeaf> leaves = observed_plan_leaves(prepared.print);
    REQUIRE(leaves.size() == 1);
    CHECK(leaves.front().speed == 73.f);
}

TEST_CASE("Layer extrusion editor hoists speed and acceleration independently",
          "[plugins][layer-extrusion-edit][speed][acceleration][hoisting]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    SECTION("uniform speed is hoisted across distinct accelerations") {
        PreparedSpeedPrint prepared;
        const DynamicPrintConfig config = speed_config({
            {"perimeter_speed", "40"}, {"default_acceleration", "500"},
            {"first_layer_speed", "100%"}, {"first_layer_acceleration", "100%"}
        });
        ExtrusionSpec first{ExtrusionRole::Perimeter, 0.2};
        first.existing_acceleration = 300.f;
        ExtrusionSpec second{ExtrusionRole::Perimeter, 0.2};
        second.existing_acceleration = 400.f;
        prepare_ordered_plan(prepared, config, {{first, second}});

        run_default_editors(prepared);
        ExtrusionEntity &root = single_plan_root(prepared.print);
        const ExtrusionPropertySpeed *root_process = root.get_property<ExtrusionPropertySpeed>();
        REQUIRE(root_process != nullptr);
        CHECK(root_process->speed_mm_per_s == 40.f);
        CHECK(root_process->accel_mm_per_s2 == -1.f);
        REQUIRE(root.child_count() == 2);
        CHECK(root.child(0).get_property<ExtrusionPropertySpeed>()->speed_mm_per_s == -1.f);
        CHECK(root.child(1).get_property<ExtrusionPropertySpeed>()->speed_mm_per_s == -1.f);
    }

    SECTION("uniform acceleration is hoisted across distinct speeds") {
        PreparedSpeedPrint prepared;
        const DynamicPrintConfig config = speed_config({
            {"perimeter_speed", "40"}, {"default_acceleration", "500"},
            {"first_layer_speed", "100%"}, {"first_layer_acceleration", "100%"}
        });
        ExtrusionSpec first{ExtrusionRole::Perimeter, 0.2};
        first.existing_speed = 30.f;
        ExtrusionSpec second{ExtrusionRole::Perimeter, 0.2};
        second.existing_speed = 35.f;
        prepare_ordered_plan(prepared, config, {{first, second}});

        run_default_editors(prepared);
        ExtrusionEntity &root = single_plan_root(prepared.print);
        const ExtrusionPropertySpeed *root_process = root.get_property<ExtrusionPropertySpeed>();
        REQUIRE(root_process != nullptr);
        CHECK(root_process->speed_mm_per_s == -1.f);
        CHECK(root_process->accel_mm_per_s2 == 500.f);
        REQUIRE(root.child_count() == 2);
        CHECK(root.child(0).get_property<ExtrusionPropertySpeed>()->accel_mm_per_s2 == -1.f);
        CHECK(root.child(1).get_property<ExtrusionPropertySpeed>()->accel_mm_per_s2 == -1.f);
    }
}

TEST_CASE("Layer extrusion editor reports invalid printable flow before mutation",
          "[plugins][layer-extrusion-edit][speed][validation]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedSpeedPrint prepared;
    const DynamicPrintConfig config = speed_config({
        {"perimeter_speed", "40"}, {"default_acceleration", "1000"},
        {"first_layer_speed", "100%"}, {"first_layer_acceleration", "100%"}
    });
    prepare_ordered_plan(prepared, config, {{{ExtrusionRole::Perimeter, 0.0}}});

    ScopedActivePlugins active({DEFAULT_SPEED_PLUGIN});
    Orchestrator &orchestrator = Orchestrator::instance();
    orchestrator.reset_plugin_cancel();
    (void)orchestrator.consume_plugin_messages();
    CHECK_THROWS_AS(Steps::StepLayerExtrusionEdition::run_step(orchestrator, prepared.print), RuntimeError);

    CHECK(orchestrator.is_plugin_cancelled());
    CHECK(single_plan_root(prepared.print).get_property<ExtrusionPropertySpeed>() == nullptr);
    bool found_flow_error = false;
    for (const Orchestrator::PluginMessage &message : orchestrator.consume_plugin_messages())
        found_flow_error = found_flow_error || message.message.find("positive volumetric flow") != std::string::npos;
    CHECK(found_flow_error);
    orchestrator.reset_plugin_cancel();
}

TEST_CASE("Layer extrusion edit runner orders plugins and runs once per layer group",
          "[plugins][layer-extrusion-edit][speed][runner]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    register_recording_editors();
    reset_recording_editor(g_first_editor);
    reset_recording_editor(g_second_editor);

    PreparedSpeedPrint prepared;
    const DynamicPrintConfig config = speed_config({
        {"perimeter_speed", "40"}, {"default_acceleration", "1000"},
        {"first_layer_speed", "100%"}, {"first_layer_acceleration", "100%"}
    });
    prepare_ordered_plan(prepared, config, {
        {{ExtrusionRole::Perimeter, 0.2}},
        {{ExtrusionRole::Perimeter, 0.2}}
    });

    ScopedActivePlugins active({g_second_editor.id, g_first_editor.id});
    Steps::StepLayerExtrusionEdition::run_step(Orchestrator::instance(), prepared.print);

    CHECK(g_first_editor.payload_valid.load());
    CHECK(g_second_editor.payload_valid.load());
    CHECK(g_first_editor.setup_count.load() == 1);
    CHECK(g_second_editor.setup_count.load() == 1);
    CHECK(g_first_editor.setup_run_count.load() == 2);
    CHECK(g_second_editor.setup_run_count.load() == 2);
    CHECK(g_first_editor.run_count.load() == 2);
    CHECK(g_second_editor.run_count.load() == 2);
    CHECK(g_second_editor.saw_previous_marker.load() == 2);
}

TEST_CASE("Layer fan plugin exposes its independent contract",
          "[plugins][layer-extrusion-edit][fan][api]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Plugin *fan_plugin = Orchestrator::instance().get_plugin(DEFAULT_FAN_PLUGIN);
    REQUIRE(fan_plugin != nullptr);
    CHECK(fan_plugin->get_step() == STEP_LAYER_EXTRUSION_EDIT);
    CHECK(fan_plugin->get_priority() == 20);
    CHECK(fan_plugin->get_exclusive_group() == "layer_extrusion_edit.fan");

    // Keys the fan step reads; the plugin publishes exactly this set.
    std::vector<std::string> defined_keys;
    for (const Plugin::DefinedConfigKey &key : fan_plugin->get_defined_config_keys())
        defined_keys.push_back(key.key);
    CHECK(defined_keys.size() == 18);
    for (const char *expected : { "bridge_fan_speed", "default_fan_speed", "disable_fan_first_layers",
                                  "fan_below_layer_time", "fan_printer_min_speed", "max_fan_speed",
                                  "top_fan_speed" })
        CHECK(std::find(defined_keys.begin(), defined_keys.end(), expected) != defined_keys.end());

    const ConfigOptionDef *fan_def = PrintConfigDef::instance().get("bridge_fan_speed");
    REQUIRE(fan_def != nullptr);
    CHECK(fan_def->type == coInts);
    CHECK(fan_def->is_vector_extruder);
    CHECK(fan_def->can_be_disabled);
    CHECK(fan_def->invalidates_step == STEP_LAYER_EXTRUSION_EDIT);
    CHECK(fan_def->default_value->serialize() == "100");

    fan_def = PrintConfigDef::instance().get("default_fan_speed");
    REQUIRE(fan_def != nullptr);
    CHECK(fan_def->type == coInts);
    CHECK(fan_def->default_value->serialize() == "!100");
    REQUIRE(fan_def->aliases.size() == 1);
    CHECK(fan_def->aliases.front() == "min_fan_speed");

    fan_def = PrintConfigDef::instance().get("fan_printer_min_speed");
    REQUIRE(fan_def != nullptr);
    CHECK(fan_def->type == coInt);
    CHECK(fan_def->default_value->serialize() == "0");

    std::vector<std::string> keys;
    for (const Plugin::UsedConfigKey &key : fan_plugin->get_used_config_keys())
        keys.push_back(key.key);
    CHECK(std::find(keys.begin(), keys.end(), "default_fan_speed") != keys.end());
    CHECK(std::find(keys.begin(), keys.end(), "overhangs_dynamic_fan_speed") != keys.end());
    CHECK(std::find(keys.begin(), keys.end(), "travel_speed") != keys.end());
    CHECK(std::find(keys.begin(), keys.end(), "perimeter_speed") == keys.end());

    orchestrator_handle *orchestrator =
        reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance());
    const PluginPropertyKey<PrintingLayerTimeProperty> first_key =
        printing_layer_time_property_key(orchestrator);
    const PluginPropertyKey<PrintingLayerTimeProperty> second_key =
        printing_layer_time_property_key(orchestrator);
    CHECK(first_key.type() >= SLIC3R_PROPERTY_TYPE_CUSTOM_BEGIN);
    CHECK(second_key.type() == first_key.type());
    CHECK(first_key.orchestrator() == orchestrator);

    // A plugin compiled against another private payload revision must fail at
    // registration instead of reusing the id and corrupting timing metadata.
    CHECK(orchestrator_register_property(
              orchestrator,
              PRINTING_LAYER_TIME_PROPERTY_NAME,
              uint32_t(sizeof(c_printing_layer_time_property) + 1),
              uint32_t(alignof(c_printing_layer_time_property))) == SLIC3R_PROPERTY_TYPE_INVALID);
}

TEST_CASE("Layer fan plugin assigns roles and preserves upstream overrides",
          "[plugins][layer-extrusion-edit][fan]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    PreparedSpeedPrint prepared;
    const DynamicPrintConfig config = speed_config({
        {"perimeter_speed", "40"},
        {"external_perimeter_speed", "40"},
        {"first_layer_speed", "100%"},
        {"max_volumetric_speed", "0"},
        {"filament_max_speed", "0"},
        {"filament_max_volumetric_speed", "0"},
        {"default_fan_speed", "20"},
        {"perimeter_fan_speed", "35"},
        {"external_perimeter_fan_speed", "70"},
        {"disable_fan_first_layers", "0"},
        {"full_fan_speed_layer", "0"},
        {"fan_below_layer_time", "0"},
        {"slowdown_below_layer_time", "0"},
        {"fan_printer_min_speed", "0"}
    });
    ExtrusionSpec overridden{ExtrusionRole::Perimeter, 0.2};
    overridden.fan_speed = 63.f;
    ExtrusionSpec equal_override{ExtrusionRole::Perimeter, 0.2};
    equal_override.fan_speed = 35.f;
    prepare_ordered_plan(prepared, config, {{
        {ExtrusionRole::Perimeter, 0.2},
        {ExtrusionRole::ExternalPerimeter, 0.2},
        overridden,
        equal_override
    }});

    run_editors(prepared, {DEFAULT_SPEED_PLUGIN, DEFAULT_FAN_PLUGIN});
    std::vector<ObservedLeaf> leaves = observed_plan_leaves(prepared.print);
    REQUIRE(leaves.size() == 4);
    std::vector<float> fan_speeds;
    for (const ObservedLeaf &leaf : leaves)
        fan_speeds.push_back(leaf.fan_speed);
    std::sort(fan_speeds.begin(), fan_speeds.end());
    const std::vector<float> expected_fan_speeds{35.f, 35.f, 63.f, 70.f};
    CHECK(fan_speeds == expected_fan_speeds);
    CHECK(std::count_if(leaves.begin(), leaves.end(), [](const ObservedLeaf &leaf) {
        return leaf.direct_process != nullptr && leaf.direct_process->fan_speed_percent == 35.f &&
               leaf.direct_property_count == 2;
    }) == 1);

    // A second execution removes only values marked by the first fan pass,
    // recomputes them, and leaves the upstream 63% override authoritative.
    run_editors(prepared, {DEFAULT_FAN_PLUGIN});
    leaves = observed_plan_leaves(prepared.print);
    REQUIRE(leaves.size() == 4);
    fan_speeds.clear();
    for (const ObservedLeaf &leaf : leaves)
        fan_speeds.push_back(leaf.fan_speed);
    std::sort(fan_speeds.begin(), fan_speeds.end());
    CHECK(fan_speeds == expected_fan_speeds);
    CHECK(std::count_if(leaves.begin(), leaves.end(), [](const ObservedLeaf &leaf) {
        return leaf.direct_process != nullptr && leaf.direct_process->fan_speed_percent == 35.f &&
               leaf.direct_property_count == 2;
    }) == 1);

    REQUIRE(prepared.print.printing_plan() != nullptr);
    PrintingLayerGroup &native_layer = prepared.print.mutable_printing_plan().groups.front().layers.front();
    const slic3r_api::PrintingLayerGroup layer_view(
        reinterpret_cast<printing_layer_group_handle *>(&native_layer));
    const PluginPropertyKey<PrintingLayerTimeProperty> layer_time_key =
        registered_layer_time_property_key();
    const PrintingLayerTimeProperty *time = layer_time_key.get(layer_view.properties());
    REQUIRE(time != nullptr);
    CHECK(time->duration_seconds > 0.0);
    CHECK_FALSE(time->is_final());
}

TEST_CASE("Layer fan rerun preserves unrelated process fields",
          "[plugins][layer-extrusion-edit][fan][regression]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    PreparedSpeedPrint prepared;
    const DynamicPrintConfig config = speed_config({
        {"default_fan_speed", "20"},
        {"perimeter_fan_speed", "35"},
        {"external_perimeter_fan_speed", "70"},
        {"disable_fan_first_layers", "0"},
        {"full_fan_speed_layer", "0"},
        {"fan_below_layer_time", "0"},
        {"slowdown_below_layer_time", "0"},
        {"fan_printer_min_speed", "0"}
    });
    ExtrusionSpec preserved{ExtrusionRole::Perimeter, 0.2};
    preserved.existing_speed = 27.f;
    preserved.existing_acceleration = 321.f;
    preserved.pressure_advance = 0.047f;
    ExtrusionSpec external{ExtrusionRole::ExternalPerimeter, 0.2};
    external.existing_speed = 29.f;
    prepare_ordered_plan(prepared, config, {{
        preserved,
        external
    }});

    run_editors(prepared, {DEFAULT_FAN_PLUGIN});
    run_editors(prepared, {DEFAULT_FAN_PLUGIN});

    const std::map<uint16_t, ObservedLeaf> leaves =
        observed_by_role(observed_plan_leaves(prepared.print));
    REQUIRE(leaves.count(uint16_t(ExtrusionRole::Perimeter)) == 1);
    const ObservedLeaf &perimeter =
        leaves.at(uint16_t(ExtrusionRole::Perimeter));
    REQUIRE(perimeter.direct_process != nullptr);
    CHECK(perimeter.direct_process->speed_mm_per_s == Approx(27.f));
    CHECK(perimeter.direct_process->accel_mm_per_s2 == Approx(321.f));
    CHECK(perimeter.direct_process->pressure_adv == Approx(0.047f));
    CHECK(perimeter.direct_process->fan_speed_percent == Approx(35.f));
}

TEST_CASE("PrintingPlan time estimator counts geometry and connecting travel",
          "[plugins][layer-extrusion-edit][fan][time-estimator]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = speed_config({{"travel_speed", "10"}});
    PrintingPlan plan;
    plan.groups.emplace_back();
    plan.groups.back().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.back().layers.back();
    layer.print_z = scale_i(0.2);
    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool = layer.tool_groups.back();
    tool.extruder_id = 0;

    std::unique_ptr<ExtrusionEntityCollection> root =
        std::make_unique<ExtrusionEntityCollection>(false, false);
    ExtrusionSpec first{ExtrusionRole::Perimeter, 0.2};
    first.existing_speed = 20.f;
    ExtrusionSpec second = first;
    root->append(make_path(first, 0));
    root->append(make_path(second, 1));

    PrintingExtrusion extrusion;
    extrusion.root = std::move(root);
    tool.extrusions.push_back(std::move(extrusion));

    const slic3r_api::Config config_view(Slic3r::ApiHost::to_config_handle(&config));
    const slic3r_api::PrintingPlan plan_view(
        reinterpret_cast<printing_plan_handle *>(&plan));
    const std::vector<slic3r_api::PrintingLayerTimeEstimate> estimates =
        slic3r_api::PrintingPlanTimeEstimator(config_view).estimate(plan_view);

    REQUIRE(estimates.size() == 1);
    // Two 18 mm paths at 20 mm/s plus the 18.027... mm connection at
    // 10 mm/s. The first machine position is intentionally free.
    CHECK(estimates.front().duration_seconds == Approx(3.6027756).margin(1e-5));
}

TEST_CASE("PrintingPlan time estimator inherits deposit flow and counts both gaps",
          "[plugins][deposit][fan][time-estimator]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = speed_config({{"travel_speed", "10"}});
    PrintingPlan plan;
    plan.groups.emplace_back();
    plan.groups.back().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.back().layers.back();
    layer.print_z = scale_i(0.2);
    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool = layer.tool_groups.back();
    tool.extruder_id = 0;
    std::unique_ptr<ExtrusionEntityCollection> root =
        std::make_unique<ExtrusionEntityCollection>(false, false);
    root->add_property(ExtrusionAttributes(ExtrusionRole::SupportMaterial,
                                         ExtrusionFlow(0.2, 0.4f, 0.2f)));
    root->add_property(ExtrusionPropertySpeed(20.f));
    root->append(make_path(ExtrusionSpec{ExtrusionRole::Perimeter, 0.2}, 0));
    std::unique_ptr<ExtrusionPath> deposit = make_path(ExtrusionSpec{}, 0);
    deposit->remove_property<ExtrusionAttributes>();
    deposit->polyline() = ArcPolyline();
    deposit->polyline().append(Point(scale_i(39.0), scale_i(11.0)));
    deposit->add_property(ExtrusionPropertySpecialCommand(
        ExtrusionPropertySpecialCommand::Code::DEPOSIT, 1.0));
    root->append(std::move(deposit));
    // A non-DEPOSIT point is not a movement and must not change timing or position.
    std::unique_ptr<ExtrusionPath> ignored = make_path(ExtrusionSpec{}, 0);
    ignored->polyline() = ArcPolyline();
    ignored->polyline().append(Point(scale_i(1000.0), 0));
    root->append(std::move(ignored));
    std::unique_ptr<ExtrusionPath> last = make_path(ExtrusionSpec{}, 0);
    last->polyline() = ArcPolyline();
    last->polyline().append(Point(scale_i(49.0), scale_i(11.0)));
    last->polyline().append(Point(scale_i(59.0), scale_i(11.0)));
    root->append(std::move(last));
    PrintingExtrusion extrusion;
    extrusion.root = std::move(root);
    tool.extrusions.push_back(std::move(extrusion));
    const slic3r_api::Config config_view(Slic3r::ApiHost::to_config_handle(&config));
    const slic3r_api::ConstPrintingPlan plan_view(
        reinterpret_cast<const printing_plan_handle *>(&plan));
    const std::vector<slic3r_api::PrintingLayerTimeEstimate> estimates =
        slic3r_api::PrintingPlanTimeEstimator(config_view).estimate(plan_view);
    REQUIRE(estimates.size() == 1);
    // 18/20 + 10/10 + 1/(20*0.2) + 10/10 + 10/20 seconds.
    CHECK(estimates.front().duration_seconds == Approx(3.65));
}

TEST_CASE("PrintingPlan time estimator rejects malformed deposits before timing",
          "[plugins][deposit][fan][time-estimator]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const int invalid = GENERATE(0, 1, 2, 3, 4, 5);
    PreparedSpeedPrint prepared;
    const DynamicPrintConfig config = speed_config({{"travel_speed", "10"}});
    ExtrusionSpec spec;
    spec.existing_speed = 20.f;
    prepare_ordered_plan(prepared, config, {{spec}});
    ExtrusionPath &deposit = dynamic_cast<ExtrusionPath &>(single_plan_root(prepared.print).child(0));
    deposit.polyline() = ArcPolyline();
    if (invalid != 0)
        deposit.polyline().append(Point(0, 0));
    if (invalid == 1)
        deposit.polyline().append(Point(scale_i(1.0), 0));
    const double volume = invalid == 2 ? 0.0 : invalid == 3 ? -1.0 :
        invalid == 4 ? std::numeric_limits<double>::quiet_NaN() :
        invalid == 5 ? std::numeric_limits<double>::infinity() : 0.5;
    deposit.add_property(ExtrusionPropertySpecialCommand(
        ExtrusionPropertySpecialCommand::Code::DEPOSIT, volume));
    const slic3r_api::ConstPrintingPlan plan_view(
        reinterpret_cast<const printing_plan_handle *>(&prepared.print.mutable_printing_plan()));
    const slic3r_api::PrintingPlanTimeEstimator estimator(
        slic3r_api::Config(Slic3r::ApiHost::to_config_handle(&config)));
    CHECK_THROWS_AS(estimator.estimate(plan_view), std::invalid_argument);
    CHECK_THROWS_WITH(estimator.estimate(plan_view),
                      "A deposit needs exactly one point and a finite positive volume.");
    slic3r_api::LayerExtrusionEdit::ExtrusionScope::LeafEndpoints endpoints;
    CHECK(slic3r_api::LayerExtrusionEdit::ExtrusionScope::leaf_endpoints(
        Slic3r::Test::TransitionPipeline::entity_view(deposit).readonly(), endpoints) == (invalid >= 2));
}

TEST_CASE("PrintingPlan time estimator treats sub-epsilon gaps as continuous",
          "[plugins][layer-extrusion-edit][fan][time-estimator][travel]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = speed_config({{"travel_speed", "1"}});
    PrintingPlan plan;
    plan.groups.emplace_back();
    plan.groups.back().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.back().layers.back();
    layer.print_z = scale_i(0.2);
    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool = layer.tool_groups.back();
    tool.extruder_id = 0;

    ExtrusionSpec spec{ExtrusionRole::Perimeter, 0.2};
    spec.existing_speed = 20.f;
    std::unique_ptr<ExtrusionPath> first = make_path(spec, 0);
    std::unique_ptr<ExtrusionPath> second = make_path(spec, 0);
    const Point first_end = first->last_point();
    second->polyline().set_front(Point(
        first_end.x() + SCALED_EPSILON - 1, first_end.y()));

    std::unique_ptr<ExtrusionEntityCollection> root =
        std::make_unique<ExtrusionEntityCollection>(false, false);
    const double expected_duration = unscaled(first->length() + second->length()) / 20.0;
    root->append(std::move(first));
    root->append(std::move(second));
    PrintingExtrusion extrusion;
    extrusion.root = std::move(root);
    tool.extrusions.push_back(std::move(extrusion));

    const slic3r_api::Config config_view(Slic3r::ApiHost::to_config_handle(&config));
    const slic3r_api::PrintingPlan plan_view(
        reinterpret_cast<printing_plan_handle *>(&plan));
    const std::vector<slic3r_api::PrintingLayerTimeEstimate> estimates =
        slic3r_api::PrintingPlanTimeEstimator(config_view).estimate(plan_view);

    REQUIRE(estimates.size() == 1);
    CHECK(estimates.front().duration_seconds == Approx(expected_duration).margin(1e-8));
}

TEST_CASE("Layer fan plugin consumes final timing without estimating geometry",
          "[plugins][layer-extrusion-edit][fan][time-estimator]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedSpeedPrint prepared;
    const DynamicPrintConfig config = speed_config({
        {"default_fan_speed", "42"},
        {"perimeter_fan_speed", "42"},
        {"support_material_fan_speed", "55"},
        {"disable_fan_first_layers", "0"},
        {"full_fan_speed_layer", "0"},
        {"fan_below_layer_time", "0"},
        {"slowdown_below_layer_time", "0"}
    });
    // No speed is provided: estimating this geometric leaf would fail. A
    // Final duration must therefore let the fan plugin skip estimation.
    prepare_ordered_plan(prepared, config, {{
        {ExtrusionRole::Perimeter, 0.2},
        {ExtrusionRole::SupportMaterial, 0.2},
        {ExtrusionRole::WipeTower, 0.2}
    }});

    const PluginPropertyKey<PrintingLayerTimeProperty> layer_time_key =
        registered_layer_time_property_key();
    REQUIRE(prepared.print.printing_plan() != nullptr);
    for (PrintingGroup &group : prepared.print.mutable_printing_plan().groups) {
        for (PrintingLayerGroup &layer : group.layers) {
            slic3r_api::PrintingLayerGroup layer_view(
                reinterpret_cast<printing_layer_group_handle *>(&layer));
            PrintingLayerTimeProperty &time = layer_time_key.get_or_add(layer_view.properties());
            time.duration_seconds = 123.0;
            time.origin = RAW_PRINTING_LAYER_TIME_ORIGIN_FINAL;
        }
    }

    run_editors(prepared, {DEFAULT_FAN_PLUGIN});
    const std::vector<ObservedLeaf> leaves = observed_plan_leaves(prepared.print);
    REQUIRE(leaves.size() == 3);
    const std::map<uint16_t, ObservedLeaf> by_role = observed_by_role(leaves);
    CHECK(by_role.at(uint16_t(ExtrusionRole::Perimeter)).fan_speed == 42.f);
    CHECK(by_role.at(uint16_t(ExtrusionRole::SupportMaterial)).fan_speed == 55.f);
    CHECK(by_role.at(uint16_t(ExtrusionRole::WipeTower)).fan_speed == -1.f);

    PrintingLayerGroup &layer = prepared.print.mutable_printing_plan().groups.front().layers.front();
    const slic3r_api::PrintingLayerGroup layer_view(
        reinterpret_cast<printing_layer_group_handle *>(&layer));
    const PrintingLayerTimeProperty *time = layer_time_key.get(layer_view.properties());
    REQUIRE(time != nullptr);
    CHECK(time->duration_seconds == Approx(123.0));
    CHECK(time->is_final());
}

TEST_CASE("Acceleration plugin owns its settings and activation rules",
          "[plugins][layer-extrusion-edit][acceleration][gui-rules]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();

    const ConfigOptionDef *def = PrintConfigDef::instance().get("default_acceleration");
    REQUIRE(def != nullptr);
    CHECK(def->type == coFloatOrPercent);
    CHECK(def->invalidates_step == STEP_LAYER_EXTRUSION_EDIT);
    CHECK(def->default_value->serialize() == "0");

    def = PrintConfigDef::instance().get("machine_max_acceleration_travel");
    REQUIRE(def != nullptr);
    CHECK(def->type == coFloats);
    CHECK(def->default_value->serialize() == "1500,1250");

    // Every rule this plugin owns applies to the print scope and keeps the
    // default_acceleration gate. Each any_of() disjunction expands to one rule
    // per activation reason, so the counts below cover all of them.
    size_t owned_rule_count = 0;
    size_t bridge_rule_count = 0;
    size_t perimeter_rule_count = 0;
    for (const Orchestrator::PluginGuiRule &rule : orchestrator.gui_rules().rules()) {
        if (rule.owner != DEFAULT_ACCELERATION_PLUGIN)
            continue;
        CHECK(rule.scope == RAW_PRESET_TYPE_FFF_PRINT);
        owned_rule_count++;
        REQUIRE(!rule.clauses.empty());
        CHECK(rule.clauses.front().condition == RAW_GUI_RULE_CONDITION_VALUE_NON_ZERO);
        CHECK(rule.clauses.front().condition_key == "default_acceleration");
        if (rule.target_key == "bridge_acceleration") {
            bridge_rule_count++;
            REQUIRE(rule.clauses.size() == 1);
        }
        if (rule.target_key == "external_perimeter_acceleration") {
            perimeter_rule_count++;
            REQUIRE(rule.clauses.size() == 2);
            CHECK(rule.clauses[1].condition_key == "perimeters");
        }
    }
    CHECK(owned_rule_count == 27);
    CHECK(bridge_rule_count == 1);
    CHECK(perimeter_rule_count == 1);

    const auto enabled = [&orchestrator](const DynamicPrintConfig &config, const char *target) {
        for (const Orchestrator::GuiRuleApplication &application :
             orchestrator.gui_rules().evaluate(config, RAW_PRESET_TYPE_FFF_PRINT, -1,
                                               DEFAULT_ACCELERATION_PLUGIN))
            if (application.target_key == target)
                return application.enabled;
        return false;
    };

    const DynamicPrintConfig no_default = speed_config({{"default_acceleration", "0"}, {"perimeters", "2"}});
    CHECK_FALSE(enabled(no_default, "bridge_acceleration"));
    CHECK_FALSE(enabled(no_default, "external_perimeter_acceleration"));

    const DynamicPrintConfig no_perimeters = speed_config({{"default_acceleration", "1000"}, {"perimeters", "0"}});
    CHECK(enabled(no_perimeters, "bridge_acceleration"));
    CHECK_FALSE(enabled(no_perimeters, "external_perimeter_acceleration"));

    const DynamicPrintConfig all_set = speed_config({{"default_acceleration", "1000"}, {"perimeters", "2"}});
    CHECK(enabled(all_set, "bridge_acceleration"));
    CHECK(enabled(all_set, "external_perimeter_acceleration"));
}

TEST_CASE("Speed plugin owns its settings and activation rules",
          "[plugins][layer-extrusion-edit][speed][gui-rules]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();

    const ConfigOptionDef *def = PrintConfigDef::instance().get("perimeter_speed");
    REQUIRE(def != nullptr);
    CHECK(def->type == coFloatOrPercent);
    CHECK(def->invalidates_step == STEP_LAYER_EXTRUSION_EDIT);
    CHECK(def->default_value->serialize() == "60%");

    def = PrintConfigDef::instance().get("first_layer_flow_ratio");
    REQUIRE(def != nullptr);
    CHECK(def->type == coPercent);
    CHECK(def->default_value->serialize() == "100%");

    def = PrintConfigDef::instance().get("filament_max_speed");
    REQUIRE(def != nullptr);
    CHECK(def->type == coFloats);
    CHECK(def->is_vector_extruder);

    // Every rule this plugin owns applies to the print scope and keeps its
    // condition keys. Each any_of() disjunction expands to one rule per
    // activation reason, so the counts below cover all of them.
    size_t owned_rule_count = 0;
    size_t perimeter_rule_count = 0;
    size_t milling_rule_count = 0;
    size_t max_print_rule_count = 0;
    for (const Orchestrator::PluginGuiRule &rule : orchestrator.gui_rules().rules()) {
        if (rule.owner != DEFAULT_SPEED_PLUGIN)
            continue;
        CHECK(rule.scope == RAW_PRESET_TYPE_FFF_PRINT);
        owned_rule_count++;
        REQUIRE(!rule.clauses.empty());
        if (rule.target_key == "perimeter_speed") {
            perimeter_rule_count++;
            REQUIRE(rule.clauses.size() == 1);
            CHECK(rule.clauses.front().condition_key == "perimeters");
        }
        if (rule.target_key == "milling_speed") {
            milling_rule_count++;
            REQUIRE(rule.clauses.size() == 1);
            CHECK(rule.clauses.front().condition_key == "milling_post_process");
        }
        if (rule.target_key == "max_print_speed") {
            max_print_rule_count++;
            REQUIRE(rule.clauses.size() == 1);
            CHECK(rule.clauses.front().condition_key == "max_volumetric_speed");
        }
    }
    CHECK(owned_rule_count == 30);
    CHECK(perimeter_rule_count == 1);
    CHECK(milling_rule_count == 1);
    CHECK(max_print_rule_count == 1);

    const auto enabled = [&orchestrator](const DynamicPrintConfig &config, const char *target) {
        for (const Orchestrator::GuiRuleApplication &application :
             orchestrator.gui_rules().evaluate(config, RAW_PRESET_TYPE_FFF_PRINT, -1,
                                               DEFAULT_SPEED_PLUGIN))
            if (application.target_key == target)
                return application.enabled;
        return false;
    };

    CHECK_FALSE(enabled(speed_config({{"perimeters", "0"}}), "perimeter_speed"));
    CHECK(enabled(speed_config({{"perimeters", "2"}}), "perimeter_speed"));

    CHECK_FALSE(enabled(speed_config({{"milling_post_process", "0"}}), "milling_speed"));
    CHECK(enabled(speed_config({{"milling_post_process", "1"}}), "milling_speed"));

    CHECK_FALSE(enabled(speed_config({{"max_volumetric_speed", "0"}}), "max_print_speed"));
    CHECK(enabled(speed_config({{"max_volumetric_speed", "5"}}), "max_print_speed"));
}
