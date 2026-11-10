#include <catch2/catch.hpp>

#include "gcode_test_helpers.hpp"
#include "layer_extrusion_edit_transition_test_helpers.hpp"
#include "plugin_test_helpers.hpp"
#include "plugin_e2e_test_helpers.hpp"
#include "support_grid_test_helpers.hpp"

#include "test_data.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <initializer_list>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/host/ApiHostUtils.hpp"
#include "libslic3r/Api/internal/LayerIslandAccess.hpp"
#include "libslic3r/Api/internal/LayerAccess.hpp"
#include "libslic3r/Api/internal/LayerRegionAccess.hpp"
#include "libslic3r/Config/FFFPrintConfig.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/Api/plugin/c/slic3r_data_tree.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_layer_extrusion_edit.h"
#include "libslic3r/Api/plugin/cpp/AuxiliaryLayerHelpers.hpp"
#include "libslic3r/Api/plugin/cpp/ConfigViews.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/FanPolicy.hpp"
#include "libslic3r/Api/plugin/cpp/DefaultSpeedLimits.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanTimeEstimator.hpp"
#include "libslic3r/Api/plugin/cpp/RegionSettingsViews.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Exception.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/ExtrusionProperty.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/PrintRegion.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingLayerTimeProperty.hpp"
#include "libslic3r/Steps/StepExtrusionOrdering.hpp"
#include "libslic3r/Steps/StepLayerHeightGeneration.hpp"
#include "libslic3r/Steps/StepLayerExtrusionEdition.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"
#include "libslic3r/Steps/StepPostSlicing.hpp"
#include "libslic3r/Steps/StepSlicing.hpp"

/*
STEP_LAYER_EXTRUSION_EDIT process-parameter tests
=================================================

The production step edits clones owned by PrintingPlan, not Layer output. Each
fixture therefore creates a small auxiliary layer with real object and region
configuration, publishes native extrusion paths, and lets STEP_ORDERING create
the same clone hierarchy that the future G-code pipeline consumes. One support
test instead slices a real object with grid support and exports it through
STEP_GCODE, as the pipeline refuses any geometric leaf left without a speed.

Tests inspect effective process values through the extrusion tree because the
default plugins deliberately hoist identical speed and acceleration values to
parents independently. Direct-property checks are used only where the storage
placement is itself the behavior under test.

Fan replay changes the standard reference, cooling threshold, geometry graph and
first-layer ramp on one Print, restoring its ordering checkpoint each time. A
coexisting fresh Print must match every context field, duration, speed and fan
pair; returning to the initial configuration must recover the initial result.

The late-editor replay test registers a test-only speed editor with no package
dependency. It scales cloned flow and respects the published absolute ceiling.
Ordering checkpoints must restore clean flow and metadata on the same Print;
fresh Prints provide an independent comparison after each configuration change.

Representative direct calls (branches are not a timeline):

    TEST_CASE
        |-- prepare_regional_process_plan()
        |-- append()
        |-- run_default_editors()
        |-- obstacle.make_clockwise()
        |-- make_path()
        `-- run_editors()
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
constexpr const char *FAN_COOLING_PLUGIN = "layer_extrusion_edit.fan.cooling";
constexpr const char *SLOWDOWN_PLUGIN = "layer_extrusion_edit.speed.slowdown";
constexpr const char *FIRST_LAYER_FAN_PLUGIN = "layer_extrusion_edit.fan.first_layers";
constexpr const char *DYNAMIC_FAN_PLUGIN = "layer_extrusion_edit.fan.dynamic";
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
    // Add a collinear point halfway, making a two-segment path of the same length.
    bool middle_point = false;
};

struct ObservedLeaf
{
    ExtrusionRole role = ExtrusionRole::None;
    float speed = -1.f;
    float acceleration = -1.f;
    float fan_speed = -1.f;
    const ExtrusionPropertySpeed *direct_process = nullptr;
    size_t direct_property_count = 0;
    double first_layer_limit = -1.;
    const ExtrusionPropertyFan *direct_fan = nullptr;
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

using SettingOverrides = std::vector<std::pair<std::string, std::string>>;

// Resolve one first-layer leaf per role with distinct explicit parent speeds,
// then apply overrides so accidental inheritance of a zero role speed is visible.
// The local Print owns the plan; setup or assertion failures abort the test.
std::map<uint16_t, float> resolved_role_speeds(const std::vector<ExtrusionSpec> &specs,
                                               const SettingOverrides &overrides);

// Return the resolved mm/s² acceleration of one skirt/brim leaf with the supplied settings.
float resolved_brim_acceleration(std::initializer_list<ConfigBase::SetDeserializeItem> overrides);

// Preserve and restore the process-wide active plugin set around runner tests.
class ScopedActivePlugins
{
public:
    // Retain the full pipeline for export tests, or isolate the supplied editors.
    explicit ScopedActivePlugins(std::initializer_list<const char *> plugin_ids, bool keep_existing = false)
        : m_orchestrator(Orchestrator::instance())
    {
        for (Plugin *plugin : m_orchestrator.active_plugins())
            m_previous.push_back(plugin);
        if (!keep_existing)
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
    if (spec.middle_point)
        path->polyline().append(Point(scale_i(20.), y));
    path->polyline().append(Point(scale_i(29.), y));
    if (spec.existing_speed > 0.f || spec.existing_acceleration > 0.f ||
        spec.pressure_advance > 0.f) {
        path->add_property(ExtrusionPropertySpeed(spec.existing_speed,
                                                  spec.existing_acceleration,
                                                   spec.pressure_advance));
    }
    if (spec.fan_speed >= 0.f)
        path->add_property(ExtrusionPropertyFan(spec.fan_speed, FAN_REQUEST_MODE_OVERRIDE));
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
    bool path_inside_modifier = false,
    std::initializer_list<ConfigBase::SetDeserializeItem> part_overrides = {})
{
    ModelObject *model_object = prepared.model.add_object();
    model_object->name = "regional_process_parameter.stl";

    ModelVolume *part = model_object->add_volume(
        Slic3r::make_cube(20., 20., 10.), ModelVolumeType::MODEL_PART, false);
    part->set_type(ModelVolumeType::MODEL_PART);
    DynamicPrintConfig part_config;
    part_config.set_deserialize_strict(part_overrides);
    part->config.assign_config(part_config);

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

// Activate the complete fan chain explicitly. Passing the reverse activation
// order checks the host's priority sort rather than relying on insertion order.
void run_fan_editors(PreparedSpeedPrint &prepared, bool include_speed = false)
{
    if (include_speed)
        run_editors(prepared, {DYNAMIC_FAN_PLUGIN, FIRST_LAYER_FAN_PLUGIN, FAN_COOLING_PLUGIN, DEFAULT_FAN_PLUGIN, DEFAULT_SPEED_PLUGIN});
    else
        run_editors(prepared, {DYNAMIC_FAN_PLUGIN, FIRST_LAYER_FAN_PLUGIN, FAN_COOLING_PLUGIN, DEFAULT_FAN_PLUGIN});
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
    }
    const auto *fan = entity.get_property<ExtrusionPropertyFan>();
    if (fan && fan->is_set())
        state.fan_speed = fan->percent;

    if (entity.child_count() > 0) {
        for (size_t child_idx = 0; child_idx < entity.child_count(); ++child_idx)
            collect_observed_leaves(entity.child(child_idx), state, out);
        return;
    }

    const ExtrusionAttributes *attributes = entity.get_property<ExtrusionAttributes>();
    if (attributes != nullptr && entity.has_polyline()) {
        const auto key = slic3r_api::default_speed_limits_key(reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
        const auto *limits = slic3r_api::ExtrusionEntity(reinterpret_cast<const extrusion_entity_handle *>(&entity)).get(key);
        out.push_back(ObservedLeaf{
            attributes->extrusion_role(), state.speed, state.acceleration, state.fan_speed,
            process,
            slic3r_api::ExtrusionEntity(
                reinterpret_cast<const extrusion_entity_handle *>(&entity)).property_count(), limits ? limits->first_layer_mm_per_s : -1., fan});
    }
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

std::map<uint16_t, float> resolved_role_speeds(const std::vector<ExtrusionSpec> &specs,
                                               const SettingOverrides &overrides)
{
    PreparedSpeedPrint prepared;
    DynamicPrintConfig config = speed_config({
        {"perimeter_speed", "31"}, {"external_perimeter_speed", "19"},
        {"solid_infill_speed", "53"}, {"bridge_speed", "27"},
        {"overhangs", "1"}, {"first_layer_speed", "100%"}, {"first_layer_min_speed", "0"},
        {"filament_max_speed", "0"}, {"filament_max_volumetric_speed", "0"}
    });
    for (const std::pair<std::string, std::string> &item : overrides)
        config.set_deserialize_strict(item.first, item.second);
    prepare_ordered_plan(prepared, config, {specs});
    run_editors(prepared, {DEFAULT_SPEED_PLUGIN});
    const std::vector<ObservedLeaf> leaves = observed_plan_leaves(prepared.print);
    REQUIRE(leaves.size() == specs.size());
    std::map<uint16_t, float> speeds;
    for (const ObservedLeaf &leaf : leaves)
        speeds[uint16_t(leaf.role())] = leaf.speed;
    REQUIRE(speeds.size() == specs.size());
    return speeds;
}

/*
Run acceleration editing on one `Skirt` leaf with support/brim overrides and
return mm/s²; the local Print owns the plan and failed REQUIRE checks abort it.
*/
float resolved_brim_acceleration(std::initializer_list<ConfigBase::SetDeserializeItem> overrides)
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
            ExtrusionPropertyFan &process = extrusion.root->get_or_add_property<ExtrusionPropertyFan>();
            if (state.writes_marker)
                process.set(23.f, FAN_REQUEST_MODE_OVERRIDE);
            else if (process.is_set() && process.percent == 23.f)
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
constexpr const char *FIRST_LAYER_FLOW_PLUGIN = "layer_extrusion_edit.first_layer_flow";

// One printed leaf of the plan with its effective flow and length in mm.
// layer_idx counts the plan's layer groups from the bottom.
struct ObservedFlow
{
    size_t layer_idx = 0;
    ExtrusionRole role = ExtrusionRole::None;
    double mm3_per_mm = 0.;
    double length = 0.;
};

// Walk one cloned tree, passing the parent's attributes down to its children.
void collect_flows(const ExtrusionEntity &entity,
                   const ExtrusionAttributes *inherited,
                   size_t layer_idx,
                   std::vector<ObservedFlow> &out)
{
    const ExtrusionAttributes *attributes = entity.get_property<ExtrusionAttributes>();
    if (attributes == nullptr)
        attributes = inherited;
    if (entity.child_count() > 0) {
        for (size_t child_idx = 0; child_idx < entity.child_count(); ++child_idx)
            collect_flows(entity.child(child_idx), attributes, layer_idx, out);
        return;
    }
    if (attributes != nullptr && entity.has_polyline())
        out.push_back(ObservedFlow{ layer_idx, attributes->extrusion_role(), attributes->mm3_per_mm,
                                    unscaled(entity.length()) });
}

// Collect the effective flow of every printed leaf of the plan.
std::vector<ObservedFlow> observed_plan_flows(const Print &print)
{
    std::vector<ObservedFlow> out;
    REQUIRE(print.printing_plan() != nullptr);
    size_t layer_idx = 0;
    for (const PrintingGroup &group : print.printing_plan()->groups)
        for (const PrintingLayerGroup &layer : group.layers) {
            for (const PrintingToolGroup &tool : layer.tool_groups)
                for (const PrintingExtrusion &extrusion : tool.extrusions) {
                    REQUIRE(extrusion.root != nullptr);
                    collect_flows(*extrusion.root, nullptr, layer_idx, out);
                }
            ++layer_idx;
        }
    return out;
}

PluginPropertyKey<PrintingLayerTimeProperty> registered_layer_time_property_key()
{
    orchestrator_handle *orchestrator =
        reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance());
    return printing_layer_time_property_key(orchestrator);
}

} // namespace

TEST_CASE("Regional segregation excludes empty and distant regions from matching values",
          "[plugins][regions][region-settings]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedSpeedPrint prepared;
    const DynamicPrintConfig config = speed_config({{"perimeter_speed", "70"}});
    prepare_regional_process_plan(prepared, config,
        {{"perimeter_speed", "30"}}, "perimeter_speed",
        {ExtrusionSpec{ExtrusionRole::Perimeter, 0.2}});
    REQUIRE(prepared.source_layers.size() == 1);
    Layer &layer = *prepared.source_layers.front();
    REQUIRE(layer.lslices().size() == 1);

    // These extra regions share a value with a region inside the candidate.
    // They must not be returned as owners merely because that value exists.
    const LayerRegion *near_region = nullptr;
    for (const LayerRegion &region : layer.regions()) {
        const slic3r_api::Config region_config(ApiHost::to_config_handle(&region.region().config()));
        if (!region.get_raw_slices().empty() && region_config.get("perimeter_speed").get_float() == 30.)
            near_region = &region;
    }
    REQUIRE(near_region != nullptr);
    const size_t original_count = layer.region_count();
    ApiInternal::LayerAccess::add_region(layer, near_region->region());
    ExPolygons &far_slices = ApiInternal::LayerRegionAccess::slices_mutable(layer.region(original_count));
    far_slices = near_region->get_raw_slices();
    for (ExPolygon &slice : far_slices)
        slice.translate(Point(scale_i(100.), 0));
    ApiInternal::LayerAccess::add_region(layer, near_region->region());
    REQUIRE(layer.region(original_count + 1).get_raw_slices().empty());

    slic3r_api::RegionSettings settings(
        reinterpret_cast<storage_handle *>(&prepared.storage),
        slic3r_api::Config(ApiHost::to_config_handle(&near_region->region().config())),
        {{"perimeter_speed"}});
    for (const LayerRegion &region : layer.regions())
        settings.add_region(slic3r_api::LayerRegion(reinterpret_cast<const layer_region_handle *>(&region)));
    settings.segregate(slic3r_api::ExPolygon(reinterpret_cast<const expolygon_handle *>(&layer.lslices().front())));

    const slic3r_api::RegionSettings::AreaMap &areas = settings.get_areas("perimeter_speed");
    REQUIRE(areas.size() == 2);
    for (const auto &entry : areas) {
        const std::vector<slic3r_api::LayerRegion> &regions = settings.get_regions("perimeter_speed", entry.first);
        REQUIRE(regions.size() == 1);
        CHECK_FALSE(regions.front().slices().empty());
        CHECK(regions.front().handle() != reinterpret_cast<const layer_region_handle *>(&layer.region(original_count)));
        CHECK(regions.front().handle() != reinterpret_cast<const layer_region_handle *>(&layer.region(original_count + 1)));
    }
}

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
    CHECK(speed_defined.size() == 20);
    for (const char *expected : { "perimeter_speed", "bridge_speed", "max_print_speed",
                                  "filament_max_speed", "milling_speed" })
        CHECK(std::find(speed_defined.begin(), speed_defined.end(), expected) != speed_defined.end());
    CHECK(std::find(speed_defined.begin(), speed_defined.end(), "travel_speed") == speed_defined.end());
    // The first-layer flow plugin owns this ratio and applies it to the flow itself.
    CHECK(std::find(speed_defined.begin(), speed_defined.end(), "first_layer_flow_ratio") == speed_defined.end());

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
            REQUIRE(leaf.direct_fan != nullptr);
            CHECK(leaf.direct_fan->percent == 42.f);
            CHECK(leaf.direct_fan->mode == FAN_REQUEST_MODE_OVERRIDE);
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

    SECTION("different raw settings with equal final speed and ceiling do not split") {
        PreparedSpeedPrint prepared;
        const DynamicPrintConfig config = speed_config({
            {"perimeter_speed", "100"}, {"first_layer_speed", "200"},
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
            {"perimeter_speed", "100"}, {"first_layer_speed", "200"},
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
    ExtrusionEntity &deposit = root.child(1);
    REQUIRE(deposit.has_polyline());
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
    deposit.polyline_ref() = ArcPolyline();
    deposit.polyline_ref().append(point);
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
    CHECK(deposit.polyline_ref().size() == 1);
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
        control.root = path->clone_canonical();
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
            REQUIRE(leaf.direct_fan != nullptr);
            CHECK(leaf.direct_fan->percent == 42.f);
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
    CHECK(root.get_property<ExtrusionPropertyFan>() == nullptr);
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

TEST_CASE("Layer extrusion editor resolves support and interface speeds",
          "[plugins][layer-extrusion-edit][speed][support]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    // Run the default editors on one support leaf and one interface leaf, then
    // return their effective speeds in that order.
    const auto support_speeds = [](std::initializer_list<ConfigBase::SetDeserializeItem> overrides) {
        PreparedSpeedPrint prepared;
        DynamicPrintConfig config = speed_config({
            {"default_acceleration", "1000"}, {"first_layer_speed", "100%"},
            {"first_layer_acceleration", "100%"}, {"max_print_speed", "73"},
            {"max_volumetric_speed", "0"}, {"filament_max_speed", "0"},
            {"filament_max_volumetric_speed", "0"}
        });
        config.set_deserialize_strict(overrides);
        prepare_ordered_plan(prepared, config, {{
            {ExtrusionRole::SupportMaterial, 0.2}, {ExtrusionRole::SupportMaterialInterface, 0.2}
        }});
        run_default_editors(prepared);
        const std::map<uint16_t, ObservedLeaf> leaves =
            observed_by_role(observed_plan_leaves(prepared.print));
        REQUIRE(leaves.count(uint16_t(ExtrusionRole::SupportMaterial)) == 1);
        REQUIRE(leaves.count(uint16_t(ExtrusionRole::SupportMaterialInterface)) == 1);
        return std::make_pair(leaves.at(uint16_t(ExtrusionRole::SupportMaterial)).speed,
                              leaves.at(uint16_t(ExtrusionRole::SupportMaterialInterface)).speed);
    };

    SECTION("a fixed support speed is the base of the interface percentage") {
        const std::pair<float, float> speeds = support_speeds({
            {"support_material_speed", "40"}, {"support_material_interface_speed", "50%"}
        });
        CHECK(speeds.first == 40.f);
        CHECK(speeds.second == 20.f);
    }

    SECTION("a zero interface speed falls back to max print speed without autospeed") {
        const std::pair<float, float> speeds = support_speeds({
            {"support_material_speed", "40"}, {"support_material_interface_speed", "0"}
        });
        CHECK(speeds.first == 40.f);
        CHECK(speeds.second == 73.f);
    }

    SECTION("a zero interface speed uses autospeed, not the support speed") {
        // Only the interface leaf joins autospeed: 0.2 mm3/mm at 100 mm/s
        // reaches the 20 mm3/s target.
        const std::pair<float, float> speeds = support_speeds({
            {"support_material_speed", "40"}, {"support_material_interface_speed", "0"},
            {"max_print_speed", "100"}, {"max_volumetric_speed", "20"}
        });
        CHECK(speeds.first == 40.f);
        CHECK(speeds.second == 100.f);
    }

    SECTION("a zero support speed falls back to max print speed without autospeed") {
        const std::pair<float, float> speeds = support_speeds({
            {"support_material_speed", "0"}, {"support_material_interface_speed", "0"}
        });
        CHECK(speeds.first == 73.f);
        CHECK(speeds.second == 73.f);
    }

    SECTION("a zero support speed uses autospeed as the interface base") {
        // The 0.2 mm3/mm flow at 100 mm/s reaches the 20 mm3/s target.
        const std::pair<float, float> speeds = support_speeds({
            {"support_material_speed", "0"}, {"support_material_interface_speed", "50%"},
            {"max_print_speed", "100"}, {"max_volumetric_speed", "20"}
        });
        CHECK(speeds.first == 100.f);
        CHECK(speeds.second == 50.f);
    }

    SECTION("the first layer speed caps support paths") {
        const std::pair<float, float> speeds = support_speeds({
            {"support_material_speed", "40"}, {"support_material_interface_speed", "50%"},
            {"first_layer_speed", "30"}, {"first_layer_min_speed", "0"}
        });
        CHECK(speeds.first == 30.f);
        CHECK(speeds.second == 20.f);
    }
}

TEST_CASE("Layer extrusion editor resolves brim speed over the support speed",
          "[plugins][layer-extrusion-edit][speed][support][brim]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    // The plugin publishes brim_speed as a percentage of the support speed.
    const ConfigOptionDef *def = PrintConfigDef::instance().get("brim_speed");
    REQUIRE(def != nullptr);
    CHECK(def->ratio_over == "support_material_speed");
    CHECK(def->default_value->serialize() == "50%");

    // Run the default editors on one skirt/brim leaf and return its speed.
    const auto brim_speed = [](std::initializer_list<ConfigBase::SetDeserializeItem> overrides) {
        PreparedSpeedPrint prepared;
        DynamicPrintConfig config = speed_config({
            {"default_acceleration", "1000"}, {"first_layer_speed", "100%"},
            {"first_layer_acceleration", "100%"}, {"max_print_speed", "73"},
            {"max_volumetric_speed", "0"}, {"filament_max_speed", "0"},
            {"filament_max_volumetric_speed", "0"}, {"support_material_speed", "40"}
        });
        config.set_deserialize_strict(overrides);
        prepare_ordered_plan(prepared, config, {{{ExtrusionRole::Skirt, 0.2}}});
        run_default_editors(prepared);
        const std::vector<ObservedLeaf> leaves = observed_plan_leaves(prepared.print);
        REQUIRE(leaves.size() == 1);
        return leaves.front().speed;
    };

    SECTION("a fixed brim speed is used as is") {
        CHECK(brim_speed({{"brim_speed", "25"}}) == 25.f);
    }

    SECTION("a brim percentage applies to the support speed") {
        CHECK(brim_speed({{"brim_speed", "50%"}}) == 20.f);
    }

    SECTION("the default brim speed is half the support speed") {
        CHECK(brim_speed({}) == 20.f);
    }

    SECTION("a zero brim speed falls back to max print speed without autospeed") {
        CHECK(brim_speed({{"brim_speed", "0"}}) == 73.f);
    }

    SECTION("a zero brim speed uses autospeed, not the support speed") {
        // 0.2 mm3/mm at 100 mm/s reaches the 20 mm3/s target.
        CHECK(brim_speed({{"brim_speed", "0"}, {"max_print_speed", "100"},
                          {"max_volumetric_speed", "20"}}) == 100.f);
    }

    SECTION("zero brim speed uses the autospeed base, not max_print_speed") {
        CHECK(resolved_brim_speed({{"brim_speed", "0"}, {"max_print_speed", "100"},
                                   {"max_volumetric_speed", "20"},
                                   {"autospeed_min_thin_flow", "8"}}) == 40.f);
    }
}

/*
Check that a zero value (and a zero percentage) of each role speed that refines a
parent speed selects autospeed, or max_print_speed without autospeed, rather than
the parent speed, and that the first-layer, filament speed and filament
volumetric ceilings then apply as for an explicit speed. One plan per scenario
holds the seven roles. The local Prints own their plans; Catch2 reports setup or
assertion failures.
*/
TEST_CASE("Layer extrusion editor resolves a zero role speed as autospeed",
          "[plugins][layer-extrusion-edit][speed][roles][zero-speed]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    const SettingOverrides without_autospeed = {{"max_print_speed", "73"}, {"max_volumetric_speed", "0"}};
    const SettingOverrides with_autospeed = {{"max_print_speed", "100"}, {"max_volumetric_speed", "20"},
                                             {"autospeed_min_thin_flow", "8"}};
    struct Scenario { const SettingOverrides &base; SettingOverrides ceiling; float expected; };
    const Scenario scenarios[] = {
        {without_autospeed, {}, 73.f},
        {without_autospeed, {{"first_layer_speed", "30"}}, 30.f},
        {without_autospeed, {{"filament_max_speed", "25"}}, 25.f},
        {without_autospeed, {{"filament_max_volumetric_speed", "6"}}, 30.f},
        {with_autospeed, {}, 40.f},
        {with_autospeed, {{"first_layer_speed", "30"}}, 30.f},
        {with_autospeed, {{"filament_max_speed", "25"}}, 25.f},
        {with_autospeed, {{"filament_max_volumetric_speed", "6"}}, 30.f}
    };
    const std::vector<ExtrusionSpec> specs = {
        {ExtrusionRole::ExternalPerimeter, 0.2}, {ExtrusionRole::GapFill, 0.2},
        {ExtrusionRole::ThinWall, 0.2}, {ExtrusionRole::TopSolidInfill, 0.2},
        {ExtrusionRole::InternalBridgeInfill, 0.2}, {ExtrusionRole::InternalInfill, 0.2},
        {ExtrusionRole::OverhangPerimeter, 0.2}
    };
    for (const char *zero : {"0", "0%"})
        for (const Scenario &scenario : scenarios) {
            CAPTURE(zero, scenario.expected, scenario.ceiling.size());
            SettingOverrides overrides = scenario.base;
            overrides.insert(overrides.end(), scenario.ceiling.begin(), scenario.ceiling.end());

            // Every role speed is zero: no role may take its parent's speed.
            SettingOverrides all_zero = overrides;
            for (const char *option : {"external_perimeter_speed", "gap_fill_speed", "thin_walls_speed",
                                       "top_solid_infill_speed", "internal_bridge_speed",
                                       "infill_speed", "overhangs_speed"})
                all_zero.emplace_back(option, zero);
            for (const std::pair<const uint16_t, float> &role_speed : resolved_role_speeds(specs, all_zero)) {
                CAPTURE(role_speed.first);
                CHECK(role_speed.second == scenario.expected);
            }

            // The thin wall parent (external perimeter, 19 mm/s) stays explicit here.
            SettingOverrides thin_only = overrides;
            thin_only.emplace_back("thin_walls_speed", zero);
            CHECK(resolved_role_speeds({{ExtrusionRole::ThinWall, 0.2}}, thin_only)
                      .at(uint16_t(ExtrusionRole::ThinWall) ) == scenario.expected);
        }
}

/*
With the thin-flow floor disabled, gap fill and thin walls take part in the
automatic volumetric target; enabled at zero they are excluded from it. The
target is min(smallest flow * max_print_speed, max_volumetric_speed), and each
leaf gets min(target / its flow, max_print_speed). Local Prints own the plans.
*/
TEST_CASE("Layer extrusion editor derives each zero role speed from its own flow",
          "[plugins][layer-extrusion-edit][speed][roles][zero-speed][autospeed]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    const std::vector<ExtrusionSpec> specs = {
        {ExtrusionRole::ExternalPerimeter, 0.1}, {ExtrusionRole::BridgeInfill, 0.4},
        {ExtrusionRole::GapFill, 0.05}
    };
    const SettingOverrides base = {
        {"external_perimeter_speed", "0"}, {"bridge_speed", "0"}, {"gap_fill_speed", "0"},
        {"max_print_speed", "100"}, {"max_volumetric_speed", "50"}
    };
    const auto speeds = [&specs, &base](const char *thin_flow) {
        SettingOverrides overrides = base;
        overrides.emplace_back("autospeed_min_thin_flow", thin_flow);
        return resolved_role_speeds(specs, overrides);
    };

    // Excluded gap fill: target = min(0.1 * 100, 50) = 10 mm3/s.
    const std::map<uint16_t, float> excluded = speeds("0");
    CHECK(excluded.at(uint16_t(ExtrusionRole::ExternalPerimeter)) == Approx(100.));
    CHECK(excluded.at(uint16_t(ExtrusionRole::BridgeInfill)) == Approx(25.));
    CHECK(excluded.at(uint16_t(ExtrusionRole::GapFill)) == Approx(100.));

    // Gap fill included: target = min(0.05 * 100, 50) = 5 mm3/s.
    const std::map<uint16_t, float> included = speeds("!0");
    CHECK(included.at(uint16_t(ExtrusionRole::ExternalPerimeter)) == Approx(50.));
    CHECK(included.at(uint16_t(ExtrusionRole::BridgeInfill)) == Approx(12.5));
    CHECK(included.at(uint16_t(ExtrusionRole::GapFill)) == Approx(100.));
}

/*
A percentage of a zero parent applies to the autospeed base, not to the parent's
own fallback and not to the whole base. Without autospeed there is no base, so the
speed falls back to max_print_speed itself. This covers the perimeter -> external perimeter and top solid -> ironing chains.
Local Prints own the plans; Catch2 reports failures.
*/
TEST_CASE("Layer extrusion editor applies a percentage of a zero parent to the autospeed base",
          "[plugins][layer-extrusion-edit][speed][roles][zero-speed]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    const SettingOverrides without_autospeed = {{"max_print_speed", "73"}, {"max_volumetric_speed", "0"}};
    const SettingOverrides with_autospeed = {{"max_print_speed", "100"}, {"max_volumetric_speed", "20"},
                                             {"autospeed_min_thin_flow", "8"}};
    const std::vector<ExtrusionSpec> perimeters = {
        {ExtrusionRole::Perimeter, 0.2}, {ExtrusionRole::ExternalPerimeter, 0.2}
    };
    const std::vector<ExtrusionSpec> top_and_ironing = {
        {ExtrusionRole::TopSolidInfill, 0.2}, {ExtrusionRole::Ironing, 0.2}
    };

    SECTION("external perimeter percentage of a zero perimeter speed, without autospeed") {
        SettingOverrides overrides = without_autospeed;
        overrides.emplace_back("perimeter_speed", "0");
        overrides.emplace_back("external_perimeter_speed", "50%");
        const std::map<uint16_t, float> speeds = resolved_role_speeds(perimeters, overrides);
        CHECK(speeds.at(uint16_t(ExtrusionRole::Perimeter)) == 73.f);
        CHECK(speeds.at(uint16_t(ExtrusionRole::ExternalPerimeter)) == 73.f);
    }

    SECTION("external perimeter percentage of a zero perimeter speed, with autospeed") {
        SettingOverrides overrides = with_autospeed;
        overrides.emplace_back("perimeter_speed", "0");
        overrides.emplace_back("external_perimeter_speed", "50%");
        const std::map<uint16_t, float> speeds = resolved_role_speeds(perimeters, overrides);
        CHECK(speeds.at(uint16_t(ExtrusionRole::Perimeter)) == 40.f);
        CHECK(speeds.at(uint16_t(ExtrusionRole::ExternalPerimeter)) == 20.f);
    }

    SECTION("ironing percentage of a zero top solid speed, without autospeed") {
        SettingOverrides overrides = without_autospeed;
        overrides.emplace_back("top_solid_infill_speed", "0");
        overrides.emplace_back("ironing_speed", "50%");
        const std::map<uint16_t, float> speeds = resolved_role_speeds(top_and_ironing, overrides);
        CHECK(speeds.at(uint16_t(ExtrusionRole::TopSolidInfill)) == 73.f);
        CHECK(speeds.at(uint16_t(ExtrusionRole::Ironing)) == 73.f);
    }

    SECTION("ironing percentage of a zero top solid speed, with autospeed") {
        SettingOverrides overrides = with_autospeed;
        overrides.emplace_back("top_solid_infill_speed", "0");
        overrides.emplace_back("ironing_speed", "50%");
        const std::map<uint16_t, float> speeds = resolved_role_speeds(top_and_ironing, overrides);
        CHECK(speeds.at(uint16_t(ExtrusionRole::TopSolidInfill)) == 40.f);
        CHECK(speeds.at(uint16_t(ExtrusionRole::Ironing)) == 20.f);
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

TEST_CASE("A print with support exports support moves at the support speeds",
          "[plugins][layer-extrusion-edit][speed][support][gcode]")
{
    // The fan layer-time estimate and STEP_GCODE reject a geometric leaf
    // without a speed, so this export fails unless the speed plugin resolves
    // the support and interface paths. Support layers are not synchronized,
    // so some print Z groups also mix object and support layer heights.
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = Slic3r::Test::SupportGrid::config("grid", false, 3, true);
    config.set_deserialize_strict({
        {"step_gcode_plugin", "gcode.printing_plan_file_writer"},
        {"gcode_flavor", "marlin2"},
        {"support_material_speed", "40"}, {"support_material_interface_speed", "50%"},
        {"max_print_speed", "80"}, {"first_layer_speed", "100%"}, {"first_layer_min_speed", "0"},
        {"max_volumetric_speed", "0"}, {"filament_max_speed", "0,0"},
        {"filament_max_volumetric_speed", "0,0"}, {"slowdown_below_layer_time", "0,0"},
        {"use_relative_e_distances", "1"}, {"wipe", "0,0"}, {"filament_diameter", "1.75,1.75"}
    });
    Model model;
    Print print;
    Slic3r::Test::init_print({Slic3r::Test::SupportGrid::slab(true)}, print, model, config);
    const boost::filesystem::path path = Slic3r::Test::GCode::temporary_gcode_path();
    std::string code;
    try {
        print.process();
        Orchestrator::instance().export_gcode(print, path.string(), nullptr);
        code = Slic3r::Test::GCode::read_text_file(path);
    } catch (...) {
        Slic3r::Test::GCode::remove_output_pair(path);
        throw;
    }
    Slic3r::Test::GCode::remove_output_pair(path);

    // Follow the modal feedrate and the ;TYPE: marker. With relative E, a
    // printing move is a G1 with X or Y and a positive E; collect the feedrate
    // (mm/min) of every such move in the two support sections.
    std::set<double> base_feedrates;
    std::set<double> interface_feedrates;
    std::string type;
    double feedrate = 0.;
    std::istringstream stream(code);
    std::string line;
    while (std::getline(stream, line)) {
        if (line.rfind(";TYPE:", 0) == 0) {
            type = line.substr(6);
            continue;
        }
        if (line.rfind("G1 ", 0) != 0 && line.rfind("G0 ", 0) != 0)
            continue;
        const std::string command = line.substr(0, line.find(';'));
        const auto word = [&command](char letter, double &value) {
            const size_t pos = command.find(std::string(" ") + letter);
            if (pos == std::string::npos)
                return false;
            value = std::stod(command.substr(pos + 2));
            return true;
        };
        double value = 0.;
        if (word('F', value))
            feedrate = value;
        double e = 0.;
        const bool moves_xy = word('X', value) || word('Y', value);
        if (!moves_xy || !word('E', e) || e <= 0.)
            continue;
        if (type == "Support material")
            base_feedrates.insert(feedrate);
        else if (type == "Support material interface")
            interface_feedrates.insert(feedrate);
    }
    CHECK(base_feedrates == std::set<double>{40. * 60.});
    CHECK(interface_feedrates == std::set<double>{20. * 60.});
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
            REQUIRE(leaf.direct_fan != nullptr);
            CHECK(leaf.direct_fan->percent == 35.f);
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

TEST_CASE("First-layer infill speed zero falls back to the general first-layer limit",
          "[plugins][layer-extrusion-edit][speed][limits][first-layer-fallback]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    for (const char *infill_limit : {"0", "0%", "30", "25%"}) {
        CAPTURE(infill_limit);
        PreparedSpeedPrint prepared;
        const DynamicPrintConfig config = speed_config({
            {"infill_speed", "200"}, {"solid_infill_speed", "200"},
            {"first_layer_speed", "20"}, {"first_layer_infill_speed", infill_limit},
            {"first_layer_min_speed", "0"}, {"max_print_speed", "300"},
            {"max_volumetric_speed", "0"}, {"filament_max_speed", "0"},
            {"filament_max_volumetric_speed", "0"}
        });
        prepare_ordered_plan(prepared, config, {{
            {ExtrusionRole::InternalInfill, 0.2}, {ExtrusionRole::SolidInfill, 0.2}
        }});
        run_default_editors(prepared);
        const auto leaves = observed_plan_leaves(prepared.print);
        REQUIRE(leaves.size() == 2);
        const float expected = std::string(infill_limit) == "30" ? 30.f :
                               std::string(infill_limit) == "25%" ? 50.f : 20.f;
        for (const auto &leaf : leaves)
            CHECK(leaf.speed == expected);
    }
}

TEST_CASE("Default speed publishes percentage ceilings before physical caps and minimums",
          "[plugins][layer-extrusion-edit][speed][resolved-limits]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedSpeedPrint prepared;
    const auto config = speed_config({{"infill_speed", "200"}, {"first_layer_infill_speed", "50%"},
        {"first_layer_min_speed", "0"}, {"max_print_speed", "40"}, {"max_volumetric_speed", "0"},
        {"filament_max_speed", "0"}, {"filament_max_volumetric_speed", "0"}});
    prepare_ordered_plan(prepared, config, {{{ExtrusionRole::InternalInfill,0.2}}});
    run_editors(prepared, {DEFAULT_SPEED_PLUGIN});
    const auto leaves = observed_plan_leaves(prepared.print);
    REQUIRE(leaves.size()==1);
    CHECK(leaves.front().speed==40.f);
    CHECK(leaves.front().first_layer_limit==100.);
}

TEST_CASE("Autospeed publishes per-flow first-layer ceilings before minimums and filament caps",
          "[plugins][layer-extrusion-edit][speed][resolved-limits][autospeed]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedSpeedPrint prepared;
    const auto config=speed_config({{"infill_speed","0"}, {"solid_infill_speed","0"},
        {"first_layer_infill_speed","50%"}, {"first_layer_min_speed","60"},
        {"max_print_speed","100"}, {"max_volumetric_speed","50"},
        {"autospeed_min_thin_flow","!0"}, {"filament_max_speed","20"}, {"filament_max_volumetric_speed","0"}});
    prepare_ordered_plan(prepared,config,{{{ExtrusionRole::InternalInfill,0.2},{ExtrusionRole::InternalInfill,0.4}}});
    run_editors(prepared,{DEFAULT_SPEED_PLUGIN});
    const auto leaves=observed_plan_leaves(prepared.print);
    REQUIRE(leaves.size()==2);
    CHECK(leaves[0].speed==20.f);
    CHECK(leaves[1].speed==20.f);
    CHECK(leaves[0].first_layer_limit==50.);
    CHECK(leaves[1].first_layer_limit==25.);
}

TEST_CASE("Regional speed projection keeps distinct ceilings even when final speeds match",
          "[plugins][layer-extrusion-edit][speed][resolved-limits]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedSpeedPrint prepared;
    const auto config = speed_config({{"infill_speed", "100"}, {"first_layer_infill_speed", "50%"},
        {"first_layer_min_speed", "0"}, {"max_print_speed", "40"}, {"max_volumetric_speed", "0"},
        {"filament_max_speed", "0"}, {"filament_max_volumetric_speed", "0"}});
    prepare_regional_process_plan(prepared, config, {{"infill_speed","200"}}, "infill_speed",
                                  {{ExtrusionRole::InternalInfill,0.2}});
    run_editors(prepared, {DEFAULT_SPEED_PLUGIN});
    const auto leaves = observed_plan_leaves(prepared.print);
    REQUIRE(leaves.size()==3);
    CHECK(leaves[0].first_layer_limit==50.);
    CHECK(leaves[1].first_layer_limit==100.);
    CHECK(leaves[2].first_layer_limit==50.);
    for (const auto &leaf:leaves) CHECK(leaf.speed==40.f);
}

TEST_CASE("Late speed editing replays from fresh ordering clones on the same Print",
          "[plugins][layer-extrusion-edit][speed][resolved-limits][replay-options]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    class LateSpeedEditor final : public slic3r_api::PluginBase {
    public:
        // Bind the test editor to the same host as the production speed provider.
        explicit LateSpeedEditor(orchestrator_handle *host) : PluginBase(host) {}
    private:
        // Identify the fixture independently of any external plugin package.
        const char *id_impl() const noexcept override { return "test.layer_edit.late_speed"; }
        // Run on ordering clones after the default speed provider.
        slicing_step_t step_impl() const noexcept override { return STEP_LAYER_EXTRUSION_EDIT; }
        // No dependency is required; priority orders the two explicitly selected editors.
        const char *const *dependencies_impl() const noexcept override {
            static const char *none[] = {nullptr};
            return none;
        }
        // Consume the resolved metadata only after default speed has published it.
        int32_t priority_impl() const noexcept override { return 20; }
        // Scale each cloned leaf once and preserve all unrelated process fields.
        void run_impl(const plugin_run_context *context) const override {
            const run_ctx_layer_extrusion_edition *payload = plugin_ctx_as_layer_extrusion_edition(context);
            if (payload == nullptr || payload->layer_group == nullptr)
                throw std::runtime_error("Missing late-editor layer context.");
            PrintingLayerGroup &layer = *reinterpret_cast<PrintingLayerGroup *>(payload->layer_group);
            const auto key = slic3r_api::default_speed_limits_key(
                reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
            for (PrintingToolGroup &tool : layer.tool_groups)
                for (PrintingExtrusion &extrusion : tool.extrusions) {
                    if (extrusion.root == nullptr)
                        continue;
                    // The fixture has one flat tree; inherit the hoisted speed from its root.
                    const ExtrusionPropertySpeed *parent = extrusion.root->get_property<ExtrusionPropertySpeed>();
                    for (size_t idx = 0; idx < extrusion.root->child_count(); ++idx) {
                        ExtrusionEntity &leaf = extrusion.root->child(idx);
                        const slic3r_api::ExtrusionEntity view(reinterpret_cast<const extrusion_entity_handle *>(&leaf));
                        const slic3r_api::DefaultSpeedLimits *limits = view.get(key);
                        if (limits == nullptr || limits->version != 1)
                            throw std::runtime_error("Missing resolved first-layer ceiling.");
                        ExtrusionPropertySpeed &process = leaf.get_or_add_property<ExtrusionPropertySpeed>();
                        const double speed = process.speed_mm_per_s > 0. ? process.speed_mm_per_s :
                                             parent != nullptr ? parent->speed_mm_per_s : -1.;
                        if (speed <= 0.)
                            throw std::runtime_error("Missing resolved speed.");
                        process.speed_mm_per_s = float(limits->first_layer_mm_per_s > 0. ?
                            std::min(speed * 2., limits->first_layer_mm_per_s) : speed * 2.);
                        ExtrusionAttributes *attributes = leaf.get_property<ExtrusionAttributes>();
                        if (attributes == nullptr)
                            throw std::runtime_error("Missing leaf flow.");
                        attributes->mm3_per_mm *= 0.5;
                    }
                }
        }
    };
    static LateSpeedEditor editor(reinterpret_cast<orchestrator_handle *>(&orchestrator));
    if (orchestrator.get_plugin("test.layer_edit.late_speed") == nullptr)
        REQUIRE(orchestrator.register_plugin(editor.c_instance()));
    PreparedSpeedPrint prepared;
    DynamicPrintConfig config=speed_config({{"infill_speed","30"}, {"first_layer_infill_speed","200%"},
        {"first_layer_min_speed","40"}, {"max_print_speed","300"}, {"max_volumetric_speed","0"},
        {"filament_max_speed","0"}, {"filament_max_volumetric_speed","0"}});
    prepare_ordered_plan(prepared,config,{{{ExtrusionRole::InternalInfill,0.04}}});
    Steps::StepExtrusionOrdering::clear(prepared.print);
    Steps::StepExtrusionOrdering::prepare(prepared.print);
    Steps::StepExtrusionOrdering::run_step(orchestrator,prepared.print);
    for (auto step:Steps::execution_order()) {
        prepared.print.mark_step_executed(step);
        if (step==STEP_ORDERING) break;
    }
    REQUIRE(prepared.print.save_ordering_checkpoint());
    bool first_run=true;
    for (double role_speed:{30.,40.,30.}) {
        if (!first_run) {
            config.set_deserialize_strict("infill_speed",std::to_string(role_speed));
            REQUIRE(prepared.print.apply(prepared.model,config)==Print::APPLY_STATUS_INVALIDATED);
            REQUIRE(prepared.print.has_ordering_checkpoint());
            prepared.print.prepare_checkpoint_resume();
        }
        run_editors(prepared,{DEFAULT_SPEED_PLUGIN,"test.layer_edit.late_speed"});
        prepared.print.mark_step_executed(STEP_LAYER_EXTRUSION_EDIT);
        const auto leaves=observed_plan_leaves(prepared.print);
        REQUIRE(leaves.size()==1);
        CHECK(leaves.front().speed==float(role_speed*2.));
        CHECK(leaves.front().first_layer_limit==role_speed*2.);
        const std::vector<ObservedFlow> flows = observed_plan_flows(prepared.print);
        REQUIRE(flows.size() == 1);
        CHECK(flows.front().mm3_per_mm == Approx(0.02));
        REQUIRE(prepared.source_roots.front()->child_count() == 1);
        const ExtrusionEntity &source = prepared.source_roots.front()->child(0);
        REQUIRE(source.get_property<ExtrusionAttributes>() != nullptr);
        CHECK(source.get_property<ExtrusionAttributes>()->mm3_per_mm == Approx(0.04));
        CHECK(source.get_property<ExtrusionPropertySpeed>() == nullptr);

        PreparedSpeedPrint fresh;
        prepare_ordered_plan(fresh, config, {{{ExtrusionRole::InternalInfill, 0.04}}});
        run_editors(fresh, {DEFAULT_SPEED_PLUGIN, "test.layer_edit.late_speed"});
        const std::vector<ObservedLeaf> fresh_leaves = observed_plan_leaves(fresh.print);
        REQUIRE(fresh_leaves.size() == 1);
        CHECK(fresh_leaves.front().speed == leaves.front().speed);
        CHECK(fresh_leaves.front().first_layer_limit == leaves.front().first_layer_limit);
        const std::vector<ObservedFlow> fresh_flows = observed_plan_flows(fresh.print);
        REQUIRE(fresh_flows.size() == 1);
        CHECK(fresh_flows.front().mm3_per_mm == flows.front().mm3_per_mm);
        first_run=false;
    }
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
        {"filament_max_volumetric_speed", "0"}, {"support_material_speed", "35"}
    });
    // The explicit support speed keeps the thin support flow out of the
    // shared target, and the wipe tower is never edited by this plugin.
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
    REQUIRE(editable_speeds.size() == 4);
    CHECK(editable_speeds[0] == 35.f);
    CHECK(editable_speeds[1] == 50.f);
    CHECK(editable_speeds[2] == 100.f);
    CHECK(editable_speeds[3] == 100.f);
    CHECK(excluded_count == 1);
}

TEST_CASE("Layer extrusion editor supports autospeed percentages and thin-flow floor",
          "[plugins][layer-extrusion-edit][speed][autospeed]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedSpeedPrint prepared;
    const DynamicPrintConfig config = speed_config({
        {"perimeter_speed", "50%"}, {"gap_fill_speed", "100%"},
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

    // Layer policies own logical settings, while firmware owns calibration.
    std::vector<std::string> defined_keys;
    for (const Plugin::DefinedConfigKey &key : fan_plugin->get_defined_config_keys())
        defined_keys.push_back(key.key);
    CHECK(defined_keys.size() == 12);
    for (const char *expected : { "bridge_fan_speed", "default_fan_speed", "top_fan_speed" })
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
    CHECK(fan_def->invalidates_step == STEP_GCODE);
    CHECK(std::find(defined_keys.begin(), defined_keys.end(), "fan_printer_min_speed") == defined_keys.end());
    CHECK(fan_def->translation_domain == "Slic3r");
    for (const char *id : {"marlin2", "marlin",
                           "klipper", "prusa",
                           "reprapfirmware", "sprinter"}) {
        INFO(id);
        Plugin *firmware_plugin = Orchestrator::instance().get_plugin(id);
        REQUIRE(firmware_plugin != nullptr);
        const std::vector<Plugin::DefinedConfigKey> &firmware_keys = firmware_plugin->get_defined_config_keys();
        REQUIRE(firmware_keys.size() == 1);
        CHECK(firmware_keys.front().key == "fan_printer_min_speed");
    }

    std::vector<std::string> keys;
    for (const Plugin::UsedConfigKey &key : fan_plugin->get_used_config_keys())
        keys.push_back(key.key);
    CHECK(std::find(keys.begin(), keys.end(), "default_fan_speed") != keys.end());
    CHECK(std::find(keys.begin(), keys.end(), "overhangs_dynamic_fan_speed") != keys.end());
    CHECK(std::find(keys.begin(), keys.end(), "travel_speed") == keys.end());
    CHECK(std::find(keys.begin(), keys.end(), "fan_below_layer_time") == keys.end());
    CHECK(std::find(keys.begin(), keys.end(), "perimeter_speed") == keys.end());
    CHECK(std::find(keys.begin(), keys.end(), "fan_printer_min_speed") == keys.end());

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

    run_fan_editors(prepared, true);
    std::vector<ObservedLeaf> leaves = observed_plan_leaves(prepared.print);
    REQUIRE(leaves.size() == 4);
    std::vector<float> fan_speeds;
    for (const ObservedLeaf &leaf : leaves)
        fan_speeds.push_back(leaf.fan_speed);
    std::sort(fan_speeds.begin(), fan_speeds.end());
    const std::vector<float> expected_fan_speeds{35.f, 35.f, 63.f, 70.f};
    CHECK(fan_speeds == expected_fan_speeds);
    CHECK(std::count_if(leaves.begin(), leaves.end(), [](const ObservedLeaf &leaf) {
        return leaf.direct_fan != nullptr && leaf.direct_fan->percent == 35.f &&
               leaf.direct_fan->mode == FAN_REQUEST_MODE_OVERRIDE &&
               leaf.direct_property_count == 3 && leaf.first_layer_limit > 0.;
    }) == 1);

    // A second execution removes only values marked by the first fan pass,
    // recomputes them, and leaves the upstream 63% override authoritative.
    run_fan_editors(prepared);
    leaves = observed_plan_leaves(prepared.print);
    REQUIRE(leaves.size() == 4);
    fan_speeds.clear();
    for (const ObservedLeaf &leaf : leaves)
        fan_speeds.push_back(leaf.fan_speed);
    std::sort(fan_speeds.begin(), fan_speeds.end());
    CHECK(fan_speeds == expected_fan_speeds);
    CHECK(std::count_if(leaves.begin(), leaves.end(), [](const ObservedLeaf &leaf) {
        return leaf.direct_fan != nullptr && leaf.direct_fan->percent == 35.f &&
               leaf.direct_fan->mode == FAN_REQUEST_MODE_OVERRIDE &&
               leaf.direct_property_count == 3 && leaf.first_layer_limit > 0.;
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
    CHECK_FALSE(time->is_resolved());
}

TEST_CASE("Layer fan defaults preserve effective modes including zero and plugin requests",
          "[plugins][layer-extrusion-edit][fan][fan-request]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedSpeedPrint prepared;
    const auto config = speed_config({
        {"default_fan_speed", "35"}, {"disable_fan_first_layers", "0"},
        {"full_fan_speed_layer", "0"}, {"fan_below_layer_time", "0"},
        {"slowdown_below_layer_time", "0"}});
    const ExtrusionSpec spec{ExtrusionRole::Perimeter, 0.2, 20.f};
    prepare_ordered_plan(prepared, config, {{spec, spec, spec}});
    auto &root = single_plan_root(prepared.print);
    REQUIRE(root.child_count() == 3);
    root.add_property(ExtrusionPropertyFan(70.f, FAN_REQUEST_MODE_OVERRIDE));
    root.child(0).get_or_add_property<ExtrusionPropertyFan>().clear();
    root.child(1).add_property(ExtrusionPropertyFan(0.f, FAN_REQUEST_MODE_AUTOMATIC));
    const auto mode = orchestrator_register_runtime_id(
        reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()),
        FAN_REQUEST_MODE_ID_TYPE, "tests.default_fan.custom");
    root.child(2).add_property(ExtrusionPropertyFan(0.f, mode));
    run_fan_editors(prepared);
    const auto leaves = observed_plan_leaves(prepared.print);
    REQUIRE(leaves.size() == 3);
    CHECK(leaves[0].fan_speed == 70.f);
    CHECK(leaves[1].fan_speed == 0.f);
    CHECK(leaves[2].fan_speed == 0.f);
    REQUIRE(leaves[0].direct_fan != nullptr);
    REQUIRE(leaves[1].direct_fan != nullptr);
    REQUIRE(leaves[2].direct_fan != nullptr);
    CHECK(leaves[0].direct_fan->mode == FAN_REQUEST_MODE_OVERRIDE);
    CHECK(root.get_property<ExtrusionPropertyFan>() == nullptr);
    CHECK(leaves[1].direct_fan->mode == FAN_REQUEST_MODE_AUTOMATIC);
    CHECK(leaves[2].direct_fan->mode == mode);
}

TEST_CASE("Layer fan normalization does not create requests on service leaves or raw extrusion connectors",
          "[plugins][layer-extrusion-edit][fan][fan-request]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedSpeedPrint prepared;
    const auto config = speed_config({
        {"default_fan_speed", "35"}, {"disable_fan_first_layers", "0"},
        {"full_fan_speed_layer", "0"}, {"fan_below_layer_time", "0"},
        {"slowdown_below_layer_time", "0"}});
    const ExtrusionSpec spec{ExtrusionRole::Perimeter, 0.2, 20.f};
    prepare_ordered_plan(prepared, config, {{spec, spec, spec}});
    auto &root = single_plan_root(prepared.print);
    // A zero-flow connector between two deposits keeps their physical fan state.
    root.child(1).add_property(ExtrusionPropertySpecialCommand(
        ExtrusionPropertySpecialCommand::Code::EXTRUSION, 0.));
    ExtrusionEntityUPtr event = std::make_unique<ExtrusionEntity>(false);
    event->add_property(ExtrusionPropertySpecialCommand(
        ExtrusionPropertySpecialCommand::Code::EXTRUSION, 0.5));
    root.append_child(std::move(event));
    run_fan_editors(prepared);
    CHECK(root.get_property<ExtrusionPropertyFan>() == nullptr);
    REQUIRE(root.child_count() == 4);
    CHECK(root.child(3).get_property<ExtrusionPropertyFan>() == nullptr);
    const auto leaves = observed_plan_leaves(prepared.print);
    REQUIRE(leaves.size() == 3);
    CHECK(leaves[1].direct_fan == nullptr);
    CHECK(leaves[1].fan_speed == -1.f);
    for (const size_t idx : {size_t(0), size_t(2)}) {
        CHECK(leaves[idx].fan_speed == 35.f);
        REQUIRE(leaves[idx].direct_fan != nullptr);
        CHECK(leaves[idx].direct_fan->mode == FAN_REQUEST_MODE_AUTOMATIC);
    }
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

    run_fan_editors(prepared);
    run_fan_editors(prepared);

    const std::map<uint16_t, ObservedLeaf> leaves =
        observed_by_role(observed_plan_leaves(prepared.print));
    REQUIRE(leaves.count(uint16_t(ExtrusionRole::Perimeter)) == 1);
    const ObservedLeaf &perimeter =
        leaves.at(uint16_t(ExtrusionRole::Perimeter));
    REQUIRE(perimeter.direct_process != nullptr);
    CHECK(perimeter.direct_process->speed_mm_per_s == Approx(27.f));
    CHECK(perimeter.direct_process->accel_mm_per_s2 == Approx(321.f));
    CHECK(perimeter.direct_process->pressure_adv == Approx(0.047f));
    REQUIRE(perimeter.direct_fan != nullptr);
    CHECK(perimeter.direct_fan->percent == Approx(35.f));
    CHECK(perimeter.direct_fan->mode == FAN_REQUEST_MODE_AUTOMATIC);
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
    const std::vector<slic3r_api::PrintingLayerTimeDetail> details =
        slic3r_api::PrintingPlanTimeEstimator(config_view).estimate_detailed(plan_view);
    REQUIRE(details.size() == 1);
    REQUIRE(details.front().leaves.size() == 2);
    CHECK(details.front().leaves[0].leaf_rank == 0);
    CHECK(details.front().leaves[0].duration_seconds == Approx(0.9));
    CHECK(details.front().leaves[1].leaf_rank == 1);
    CHECK(details.front().leaves[1].duration_seconds == Approx(0.5));
    // The fixed deposit and both synthetic gaps belong to no geometric leaf.
    CHECK(details.front().unattributed_seconds == Approx(2.25));
    double detailed_total = details.front().unattributed_seconds;
    for (const slic3r_api::PrintingLeafTime &leaf : details.front().leaves)
        detailed_total += leaf.duration_seconds;
    CHECK(detailed_total == Approx(estimates.front().duration_seconds));
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
    ExtrusionEntity &deposit = single_plan_root(prepared.print).child(0);
    REQUIRE(deposit.has_polyline());
    deposit.polyline_ref() = ArcPolyline();
    if (invalid != 0)
        deposit.polyline_ref().append(Point(0, 0));
    if (invalid == 1)
        deposit.polyline_ref().append(Point(scale_i(1.0), 0));
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
            time.origin = RAW_PRINTING_LAYER_TIME_ORIGIN_RESOLVED;
        }
    }

    run_fan_editors(prepared);
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
    CHECK(time->is_resolved());
}

TEST_CASE("Public fan context publishes references despite protected leaves and copies with tools",
          "[plugins][layer-extrusion-edit][fan-policy][fan]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedSpeedPrint prepared;
    const DynamicPrintConfig config = speed_config({
        {"default_fan_speed", "20"}, {"perimeter_fan_speed", "40"},
        {"disable_fan_first_layers", "0"}, {"full_fan_speed_layer", "0"},
        {"fan_below_layer_time", "10"}, {"slowdown_below_layer_time", "5"}
    });
    ExtrusionSpec protected_leaf{ExtrusionRole::Perimeter, 0.2};
    protected_leaf.fan_speed = 0.f;
    prepare_ordered_plan(prepared, config, {{protected_leaf}});
    orchestrator_handle *orch = reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance());
    const PluginPropertyKey<slic3r_api::FanPolicy::Context> context_key = slic3r_api::FanPolicy::context_property_key(orch);
    CHECK(slic3r_api::FanPolicy::context_property_key(orch).type() == context_key.type());
    CHECK(orchestrator_register_property(orch, FAN_POLICY_CONTEXT_PROPERTY_NAME,
        uint32_t(sizeof(c_fan_policy_context) + 1), uint32_t(alignof(c_fan_policy_context))) == SLIC3R_PROPERTY_TYPE_INVALID);
    const PluginPropertyKey<PrintingLayerTimeProperty> time_key = registered_layer_time_property_key();
    for (PrintingGroup &group : prepared.print.mutable_printing_plan().groups)
        for (PrintingLayerGroup &layer : group.layers) {
            slic3r_api::PrintingLayerGroup view(reinterpret_cast<printing_layer_group_handle *>(&layer));
            PrintingLayerTimeProperty &time = time_key.get_or_add(view.properties());
            time.duration_seconds = 7.5;
            time.origin = RAW_PRINTING_LAYER_TIME_ORIGIN_RESOLVED;
        }
    run_fan_editors(prepared);
    const std::vector<ObservedLeaf> leaves = observed_plan_leaves(prepared.print);
    REQUIRE(leaves.size() == 1);
    CHECK(leaves.front().fan_speed == 0.f);
    size_t contexts = 0;
    for (PrintingGroup &group : prepared.print.mutable_printing_plan().groups)
        for (PrintingLayerGroup &layer : group.layers)
            for (PrintingToolGroup &tool : layer.tool_groups) {
                const slic3r_api::PrintingToolGroup view(reinterpret_cast<printing_tool_group_handle *>(&tool));
                const slic3r_api::FanPolicy::Context *context = context_key.get(view.properties());
                REQUIRE(context != nullptr);
                CHECK_NOTHROW(context->validate(RAW_FAN_POLICY_PHASE_CLOSED));
                CHECK(context->roles[RAW_FAN_ROLE_DEFAULT].percent == 60);
                CHECK(context->roles[RAW_FAN_ROLE_PERIMETER].configured_percent == 40);
                CHECK(context->roles[RAW_FAN_ROLE_PERIMETER].percent == 70);
                // A property-container copy must retain the complete ABI record,
                // not borrow storage from the original tool group.
                PrintingToolGroup copy;
                copy.properties = tool.properties;
                const slic3r_api::PrintingToolGroup copy_view(reinterpret_cast<printing_tool_group_handle *>(&copy));
                const slic3r_api::FanPolicy::Context *copied = context_key.get(copy_view.properties());
                REQUIRE(copied != nullptr);
                CHECK(copied != context);
                CHECK(copied->roles[RAW_FAN_ROLE_PERIMETER].percent == 70);
                ++contexts;
            }
    CHECK(contexts > 0);
    for (const ExtrusionEntity *root : prepared.source_roots)
        CHECK(root->get_property<ExtrusionPropertyFan>() == nullptr);
}

TEST_CASE("Fan setup is read only and every tool consumes total layer duration",
          "[plugins][layer-extrusion-edit][fan-policy][printing-plan]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedSpeedPrint prepared;
    const DynamicPrintConfig config = speed_config({
        {"default_fan_speed", "20"}, {"perimeter_fan_speed", "40"},
        {"disable_fan_first_layers", "0"}, {"full_fan_speed_layer", "0"},
        {"fan_below_layer_time", "10"}, {"slowdown_below_layer_time", "5"}
    });
    ExtrusionSpec spec{ExtrusionRole::Perimeter, 0.2};
    spec.existing_speed = 20.f;
    prepare_ordered_plan(prepared, config, {{spec}});
    PrintingPlan &plan = prepared.print.mutable_printing_plan();
    PrintingLayerGroup &layer = plan.groups.front().layers.front();
    REQUIRE(layer.tool_groups.size() == 1);
    // Separate visits to the same physical tool still share the layer total.
    // Moving a second root into another visit also exercises inter-leaf travel.
    layer.tool_groups.emplace_back();
    layer.tool_groups.back().extruder_id = 0;
    PrintingExtrusion second;
    second.root.reset(layer.tool_groups.front().extrusions.front().root->clone());
    layer.tool_groups.back().extrusions.emplace_back(std::move(second));
    const slic3r_api::PrintingPlan plan_view(reinterpret_cast<printing_plan_handle *>(&plan));
    const slic3r_api::Config config_view(Slic3r::ApiHost::to_config_handle(&prepared.print.config()));
    const std::vector<slic3r_api::PrintingLayerTimeEstimate> estimates =
        slic3r_api::PrintingPlanTimeEstimator(config_view).estimate(plan_view);
    REQUIRE_FALSE(estimates.empty());
    const double total = estimates.front().duration_seconds;
    const PluginPropertyKey<PrintingLayerTimeProperty> time_key = registered_layer_time_property_key();
    const PluginPropertyKey<slic3r_api::FanPolicy::Context> context_key = slic3r_api::FanPolicy::context_property_key(
        reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
    Plugin *plugin = Orchestrator::instance().get_plugin(FAN_COOLING_PLUGIN);
    REQUIRE(plugin != nullptr);
    run_ctx_layer_extrusion_edition payload = {};
    payload.print = reinterpret_cast<const print_handle *>(&prepared.print);
    payload.plan = reinterpret_cast<printing_plan_handle *>(&plan);
    payload.group = reinterpret_cast<printing_group_handle *>(&plan.groups.front());
    payload.layer_group = reinterpret_cast<printing_layer_group_handle *>(&layer);
    plugin_run_context run = {};
    run.step = STEP_LAYER_EXTRUSION_EDIT;
    run.data = &payload;
    plugin->setup(run, uint32_t(estimates.size()));
    const slic3r_api::PrintingLayerGroup layer_view(payload.layer_group);
    CHECK(time_key.get(layer_view.properties()) == nullptr);
    for (PrintingToolGroup &tool : layer.tool_groups) {
        const slic3r_api::PrintingToolGroup tool_view(reinterpret_cast<printing_tool_group_handle *>(&tool));
        CHECK(context_key.get(tool_view.properties()) == nullptr);
        CHECK(tool.extrusions.front().root->get_property<ExtrusionPropertyFan>() == nullptr);
    }
    run_fan_editors(prepared);
    // Read the finished pass through newly acquired views of its owning scopes.
    PrintingLayerGroup &published_layer = prepared.print.mutable_printing_plan().groups.front().layers.front();
    const slic3r_api::PrintingLayerGroup published_view(reinterpret_cast<printing_layer_group_handle *>(&published_layer));
    const PrintingLayerTimeProperty *published = time_key.get(published_view.properties());
    REQUIRE(published != nullptr);
    CHECK(published->origin == RAW_PRINTING_LAYER_TIME_ORIGIN_ESTIMATED);
    CHECK(published->duration_seconds == Approx(total));
    const slic3r_api::FanPolicy::Settings settings = slic3r_api::FanPolicy::read_settings(config_view, 0);
    const slic3r_api::FanPolicy::Context expected = slic3r_api::FanPolicy::close_first_layers(
        slic3r_api::FanPolicy::apply_layer_time(slic3r_api::FanPolicy::make_base(settings), settings, total),
        slic3r_api::FanPolicy::first_layer_program(settings, 0));
    REQUIRE(published_layer.tool_groups.size() == 2);
    for (PrintingToolGroup &tool : published_layer.tool_groups) {
        const slic3r_api::PrintingToolGroup view(reinterpret_cast<printing_tool_group_handle *>(&tool));
        const slic3r_api::FanPolicy::Context *context = context_key.get(view.properties());
        REQUIRE(context != nullptr);
        CHECK(context->roles[RAW_FAN_ROLE_PERIMETER].percent == expected.roles[RAW_FAN_ROLE_PERIMETER].percent);
    }
}

TEST_CASE("Fan references and estimated timing replay from the checkpoint on the same Print",
          "[plugins][layer-extrusion-edit][fan-policy][fan]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedSpeedPrint prepared;
    DynamicPrintConfig config = speed_config({
        {"perimeter_speed", "20"}, {"first_layer_speed", "100%"}, {"max_volumetric_speed", "0"},
        {"filament_max_speed", "0"}, {"filament_max_volumetric_speed", "0"},
        {"default_fan_speed", "20"}, {"perimeter_fan_speed", "40"},
        {"disable_fan_first_layers", "0"}, {"full_fan_speed_layer", "4"},
        {"fan_below_layer_time", "1.5"}, {"slowdown_below_layer_time", "0.5"}
    });
    prepare_ordered_plan(prepared, config, {{{ExtrusionRole::Perimeter, 0.2}}});
    for (slicing_step_t step : Steps::execution_order()) {
        prepared.print.mark_step_executed(step);
        if (step == STEP_ORDERING)
            break;
    }
    REQUIRE(prepared.print.save_ordering_checkpoint());
    const PluginPropertyKey<PrintingLayerTimeProperty> time_key = registered_layer_time_property_key();
    const PluginPropertyKey<slic3r_api::FanPolicy::Context> context_key = slic3r_api::FanPolicy::context_property_key(
        reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
    double first_duration = 0;
    size_t iteration = 0;
    for (int speed : {20, 40, 20}) {
        if (iteration > 0) {
            config.set_deserialize_strict("perimeter_speed", std::to_string(speed));
            REQUIRE(prepared.print.apply(prepared.model, config) == Print::APPLY_STATUS_INVALIDATED);
            prepared.print.prepare_checkpoint_resume();
        }
        run_fan_editors(prepared, true);
        prepared.print.mark_step_executed(STEP_LAYER_EXTRUSION_EDIT);
        PrintingLayerGroup &layer = prepared.print.mutable_printing_plan().groups.front().layers.front();
        const slic3r_api::PrintingLayerGroup layer_view(reinterpret_cast<printing_layer_group_handle *>(&layer));
        const PrintingLayerTimeProperty *time = time_key.get(layer_view.properties());
        REQUIRE(time != nullptr);
        if (iteration == 0)
            first_duration = time->duration_seconds;
        CHECK(time->duration_seconds == Approx(first_duration * 20 / speed));
        const slic3r_api::PrintingToolGroup tool_view(reinterpret_cast<printing_tool_group_handle *>(&layer.tool_groups.front()));
        const slic3r_api::FanPolicy::Context *context = context_key.get(tool_view.properties());
        REQUIRE(context != nullptr);
        CHECK(context->phase == RAW_FAN_POLICY_PHASE_CLOSED);
        CHECK(context->roles[RAW_FAN_ROLE_PERIMETER].configured_percent == 40);
        const std::vector<ObservedLeaf> leaves = observed_plan_leaves(prepared.print);
        REQUIRE(leaves.size() == 1);
        // The ramp is not idempotent. Matching a freshly computed reference
        // proves this pass did not transform the preceding pass's leaf request.
        CHECK(leaves.front().fan_speed == context->roles[RAW_FAN_ROLE_PERIMETER].percent);
        CHECK(leaves.front().speed == speed);
        ++iteration;
    }
}

// One printed leaf as observed in the plan: role id, effective speed, effective
// fan percent and the mode of its direct fan property (0 when it has none).
struct FanLeaf
{
    uint16_t role = 0;
    float speed = -1.f;
    float fan_speed = -1.f;
    uint32_t mode = 0;

    // Compare copied process state without retaining pointers into either plan.
    bool operator==(const FanLeaf &other) const
    {
        return role == other.role && speed == other.speed && fan_speed == other.fan_speed && mode == other.mode;
    }
};

// The fan results of the first layer, copied by value so they outlive a plan
// replacement and can be compared across Prints. Compared fields: the fan
// context (struct_size, phase, dynamic_min/max and every role's
// configured_percent/percent), the layer duration, and the ordered leaves.
struct FanResult
{
    slic3r_api::FanPolicy::Context context;
    double duration = -1.;
    std::vector<FanLeaf> leaves;

    // Require every published reference, duration and ordered leaf to match exactly.
    bool operator==(const FanResult &other) const
    {
        if (context.struct_size != other.context.struct_size || context.phase != other.context.phase ||
            context.dynamic_min != other.context.dynamic_min || context.dynamic_max != other.context.dynamic_max)
            return false;
        for (size_t idx = 0; idx < RAW_FAN_ROLE_COUNT; ++idx)
            if (context.roles[idx].configured_percent != other.context.roles[idx].configured_percent ||
                context.roles[idx].percent != other.context.roles[idx].percent)
                return false;
        return duration == other.duration && leaves == other.leaves;
    }
};

// Validate and copy the first tool's context after the chain has completed.
slic3r_api::FanPolicy::Context first_fan_context(PreparedSpeedPrint &prepared);

// Catch prints this when a comparison fails.
std::ostream &operator<<(std::ostream &os, const FanResult &r)
{
    os << "phase " << r.context.phase << " dyn " << r.context.dynamic_min << ".." << r.context.dynamic_max << " t " << r.duration << " roles";
    for (const auto &role : r.context.roles) os << " " << role.configured_percent << "/" << role.percent;
    os << " leaves";
    for (const FanLeaf &leaf : r.leaves)
        os << " [" << leaf.role << " " << leaf.speed << " " << leaf.fan_speed << " m" << leaf.mode << "]";
    return os;
}

// Precondition: the fan chain has run on `prepared`, so the first tool group
// holds a fan context and the first layer holds a layer-time property. Every
// value is copied; no view into the plan is kept.
FanResult capture_fan_result(PreparedSpeedPrint &prepared)
{
    REQUIRE(prepared.print.printing_plan() != nullptr);
    FanResult out;
    out.context = first_fan_context(prepared);
    const PrintingLayerGroup &layer = prepared.print.printing_plan()->groups.front().layers.front();
    const slic3r_api::ConstPrintingLayerGroup view(reinterpret_cast<const printing_layer_group_handle *>(&layer));
    const PrintingLayerTimeProperty *time = registered_layer_time_property_key().get(view.properties());
    REQUIRE(time != nullptr);
    out.duration = time->duration_seconds;
    for (const ObservedLeaf &leaf : observed_plan_leaves(prepared.print))
        out.leaves.push_back({uint16_t(leaf.role()), leaf.speed, leaf.fan_speed,
                              leaf.direct_fan ? uint32_t(leaf.direct_fan->mode) : 0u});
    return out;
}

// One Perimeter and one OverhangPerimeter on the first layer; the overhang
// distances make the leaf modifiable by the dynamic stage. They are added to
// the source before ordering so every clone carries them.
void prepare_fan_settings_plan(PreparedSpeedPrint &prepared, const DynamicPrintConfig &config)
{
    const TriangleMesh cube = Slic3r::make_cube(20., 20., 20.);
    Slic3r::Test::init_print({cube}, prepared.print, prepared.model, config);
    append_source_layer(prepared, {{ExtrusionRole::Perimeter, 0.2}, {ExtrusionRole::OverhangPerimeter, 0.2}}, 0.2);
    ExtrusionEntityCollection &root = *prepared.source_roots.front();
    REQUIRE(root.child_count() == 2);
    slic3r_api::MutableExtrusionEntity overhang(reinterpret_cast<extrusion_entity_handle *>(&root.child(1)));
    slic3r_api::EPropertyOverhang &property = overhang.get_or_add(slic3r_api::EPropertyOverhang::key);
    property.start_distance_from_prev_layer = 0.5f;
    property.end_distance_from_prev_layer = 0.5f;
    Steps::StepExtrusionOrdering::clear(prepared.print);
    Steps::StepExtrusionOrdering::prepare(prepared.print);
    Steps::StepExtrusionOrdering::run_step(Orchestrator::instance(), prepared.print);
    REQUIRE(prepared.print.printing_plan() != nullptr);
}

// Baseline of the settings test. Every policy reads a value that differs from
// the full-config default, so a config that was not applied (first_layer_speed
// 30, disable_fan_first_layers 1, no fan speeds) cannot give the same result.
// The layer lasts about 1.8 s: fan_below_layer_time 3 makes cooling active
// until it is lowered. perimeter_fan_speed is disabled, so default_fan_speed
// gives the Perimeter's configured reference, which cooling then transforms.
// With the dynamic graph enabled the overhang has no own request (reference
// -1): it inherits the closed Perimeter value, cooled to about 74 %. The 60 %
// graph is below that and changes nothing; raising it to 90 % does.
DynamicPrintConfig fan_settings_config()
{
    DynamicPrintConfig config = speed_config({
        {"perimeter_speed", "20"}, {"overhangs_speed", "20"}, {"first_layer_speed", "100%"},
        {"max_volumetric_speed", "0"}, {"filament_max_speed", "0"}, {"filament_max_volumetric_speed", "0"},
        {"default_fan_speed", "20"}, {"perimeter_fan_speed", "-1"}, {"overhangs_fan_speed", "50"},
        {"disable_fan_first_layers", "0"}, {"full_fan_speed_layer", "0"},
        {"fan_below_layer_time", "3"}, {"slowdown_below_layer_time", "0.5"}});
    config.set_key_value("overhangs_dynamic_fan_speed", new ConfigOptionGraphs({
        GraphData(0, 2, GraphData::GraphType::LINEAR, {{0., 60.}, {100., 60.}})}));
    return config;
}

TEST_CASE("Fan setting changes on the same Print match a fresh Print and the initial result returns",
          "[plugins][layer-extrusion-edit][fan-policy][fan]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    // One setting per host policy: standard, cooling, dynamic, first layers.
    // Each change is cumulative and must visibly change the published result.
    const std::vector<std::function<void(DynamicPrintConfig &)>> changes = {
        [](DynamicPrintConfig &c) { c.set_deserialize_strict("default_fan_speed", "30"); },
        [](DynamicPrintConfig &c) { c.set_deserialize_strict("fan_below_layer_time", "1"); },
        [](DynamicPrintConfig &c) {
            c.set_key_value("overhangs_dynamic_fan_speed", new ConfigOptionGraphs({
                GraphData(0, 2, GraphData::GraphType::LINEAR, {{0., 90.}, {100., 90.}})}));
        },
        [](DynamicPrintConfig &c) { c.set_deserialize_strict("full_fan_speed_layer", "4"); }};
    std::vector<DynamicPrintConfig> configs{fan_settings_config()};
    for (const auto &change : changes) {
        configs.push_back(configs.back());
        change(configs.back());
    }

    PreparedSpeedPrint reused;
    prepare_fan_settings_plan(reused, configs.front());
    for (slicing_step_t step : Steps::execution_order()) {
        reused.print.mark_step_executed(step);
        if (step == STEP_ORDERING)
            break;
    }
    REQUIRE(reused.print.save_ordering_checkpoint());
    REQUIRE(reused.print.has_ordering_checkpoint());
    // `config` is a const reference; never assign a config to an alias of
    // itself, since DynamicConfig self-assignment clears it.
    const auto replay = [&](const DynamicPrintConfig &config, Print::ApplyStatus expected) {
        REQUIRE(reused.print.apply(reused.model, config) == expected);
        if (expected == Print::APPLY_STATUS_INVALIDATED)
            reused.print.prepare_checkpoint_resume();
        run_fan_editors(reused, true);
        reused.print.mark_step_executed(STEP_LAYER_EXTRUSION_EDIT);
        return capture_fan_result(reused);
    };
    // Built while `reused` is still alive, so two Prints coexist. An empty
    // config would also report "unchanged" (apply only compares present keys),
    // so the proof that the config reached the fresh Print is the sentinel
    // values read back from its effective config; the status is a second check.
    const auto fresh_result = [](const DynamicPrintConfig &config) {
        REQUIRE_FALSE(config.empty());
        PreparedSpeedPrint fresh;
        prepare_fan_settings_plan(fresh, config);
        const ConfigOption *fan_speed = fresh.print.config().option("default_fan_speed");
        REQUIRE(fan_speed != nullptr);
        CHECK(fan_speed->serialize() == config.option("default_fan_speed")->serialize());
        CHECK(fresh.print.config().opt_serialize("fan_below_layer_time") == config.opt_serialize("fan_below_layer_time"));
        CHECK(fresh.print.default_region_config().opt_serialize("perimeter_speed") == config.opt_serialize("perimeter_speed"));
        REQUIRE(fresh.print.apply(fresh.model, config) == Print::APPLY_STATUS_UNCHANGED);
        run_fan_editors(fresh, true);
        return capture_fan_result(fresh);
    };

    FanResult initial;
    FanResult previous;
    for (size_t idx = 0; idx < configs.size(); ++idx) {
        INFO(idx);
        const FanResult resumed = replay(configs[idx], idx == 0 ? Print::APPLY_STATUS_UNCHANGED : Print::APPLY_STATUS_INVALIDATED);
        REQUIRE(resumed.leaves.size() == 2);
        CHECK(resumed == fresh_result(configs[idx]));
        if (idx == 0) {
            initial = resumed;
            CHECK(resumed.context.roles[RAW_FAN_ROLE_DEFAULT].configured_percent == 20);
        } else {
            CHECK_FALSE(resumed == previous);
        }
        if (idx == 1)
            CHECK(resumed.context.roles[RAW_FAN_ROLE_DEFAULT].configured_percent == 30);
        previous = resumed;
    }

    // Restoring every changed setting gives back the initial published result.
    CHECK(replay(configs.front(), Print::APPLY_STATUS_INVALIDATED) == initial);

}

TEST_CASE("Invalid public timing fails in setup without publishing cached fan data",
          "[plugins][layer-extrusion-edit][fan-policy][fan]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    const DynamicPrintConfig config = speed_config({
        {"default_fan_speed", "20"}, {"disable_fan_first_layers", "0"},
        {"full_fan_speed_layer", "0"}, {"fan_below_layer_time", "0"}, {"slowdown_below_layer_time", "0"}
    });
    ExtrusionSpec spec{ExtrusionRole::Perimeter, 0.2};
    spec.existing_speed = 20.f;
    // Warm the shared plugin's setup cache using a different Print first.
    PreparedSpeedPrint previous;
    prepare_ordered_plan(previous, config, {{spec}});
    run_fan_editors(previous);

    PreparedSpeedPrint invalid;
    prepare_ordered_plan(invalid, config, {{spec}});
    const PluginPropertyKey<PrintingLayerTimeProperty> time_key = registered_layer_time_property_key();
    PrintingLayerGroup &layer = invalid.print.mutable_printing_plan().groups.front().layers.front();
    slic3r_api::PrintingLayerGroup view(reinterpret_cast<printing_layer_group_handle *>(&layer));
    PrintingLayerTimeProperty &time = time_key.get_or_add(view.properties());
    time.duration_seconds = -1;
    time.origin = RAW_PRINTING_LAYER_TIME_ORIGIN_RESOLVED;
    {
        ScopedActivePlugins active({DEFAULT_FAN_PLUGIN, FAN_COOLING_PLUGIN, FIRST_LAYER_FAN_PLUGIN, DYNAMIC_FAN_PLUGIN});
        orchestrator.reset_plugin_cancel();
        CHECK_THROWS(Steps::StepLayerExtrusionEdition::run_step(orchestrator, invalid.print));
        CHECK(orchestrator.is_plugin_cancelled());
        orchestrator.reset_plugin_cancel();
    }
    const PluginPropertyKey<slic3r_api::FanPolicy::Context> context_key = slic3r_api::FanPolicy::context_property_key(
        reinterpret_cast<orchestrator_handle *>(&orchestrator));
    for (PrintingToolGroup &tool : layer.tool_groups) {
        const slic3r_api::PrintingToolGroup tool_view(reinterpret_cast<printing_tool_group_handle *>(&tool));
        const slic3r_api::FanPolicy::Context *context = context_key.get(tool_view.properties());
        REQUIRE(context != nullptr);
        CHECK(context->phase == RAW_FAN_POLICY_PHASE_BASE);
        // The completed BASE pass remains, but failed cooling publishes no TIME.
        CHECK(observed_plan_leaves(invalid.print).front().fan_speed == 20.f);
    }
}

// Read the first tool's public record by value, never retaining a property view
// across another plugin run or a PrintingPlan replacement.
slic3r_api::FanPolicy::Context first_fan_context(PreparedSpeedPrint &prepared)
{
    REQUIRE(prepared.print.printing_plan() != nullptr);
    const PluginPropertyKey<slic3r_api::FanPolicy::Context> key = slic3r_api::FanPolicy::context_property_key(
        reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
    const PrintingToolGroup &tool = prepared.print.printing_plan()->groups.front().layers.front().tool_groups.front();
    const slic3r_api::ConstPrintingToolGroup view(reinterpret_cast<const printing_tool_group_handle *>(&tool));
    const slic3r_api::FanPolicy::Context *context = key.get(view.properties());
    REQUIRE(context != nullptr);
    return *context;
}

// Inject one authoritative whole-layer total so policy tests do not depend on
// incidental path lengths, travel placement or the geometric timing fallback.
void publish_fan_test_duration(PreparedSpeedPrint &prepared, double seconds)
{
    const PluginPropertyKey<PrintingLayerTimeProperty> key = registered_layer_time_property_key();
    for (PrintingGroup &group : prepared.print.mutable_printing_plan().groups)
        for (PrintingLayerGroup &layer : group.layers) {
            slic3r_api::PrintingLayerGroup view(reinterpret_cast<printing_layer_group_handle *>(&layer));
            PrintingLayerTimeProperty &time = key.get_or_add(view.properties());
            time.duration_seconds = seconds;
            time.origin = RAW_PRINTING_LAYER_TIME_ORIGIN_RESOLVED;
        }
}

TEST_CASE("Independent fan stages expose phases and transform only modifiable pairs",
          "[plugins][layer-extrusion-edit][fan-policy][fan-request]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedSpeedPrint prepared;
    const DynamicPrintConfig config = speed_config({
        {"default_fan_speed", "20"}, {"perimeter_fan_speed", "40"},
        {"fan_below_layer_time", "10"}, {"slowdown_below_layer_time", "5"},
        {"disable_fan_first_layers", "0"}, {"full_fan_speed_layer", "4"}});
    const ExtrusionSpec spec{ExtrusionRole::Perimeter, 0.2, 20.f};
    prepare_ordered_plan(prepared, config, {{spec, spec, spec, spec, spec, spec}});
    ExtrusionEntity &root = single_plan_root(prepared.print);
    REQUIRE(root.child_count() == 6);
    std::vector<double> lengths;
    for (size_t idx = 0; idx < root.child_count(); ++idx)
        lengths.push_back(root.child(idx).length());
    const fan_request_mode custom = orchestrator_register_runtime_id(
        reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()), FAN_REQUEST_MODE_ID_TYPE, "tests.fan.stage.other");
    const fan_request_mode modes[] = {FAN_REQUEST_MODE_AUTOMATIC, FAN_REQUEST_MODE_AUTOMATIC,
        FAN_REQUEST_MODE_OVERRIDE, FAN_REQUEST_MODE_OVERRIDE, custom};
    const float initial[] = {40, 0, 40, 0, 40};
    for (size_t idx = 0; idx < 5; ++idx)
        root.child(idx).add_property(ExtrusionPropertyFan(initial[idx], modes[idx]));
    publish_fan_test_duration(prepared, 7.5);

    run_editors(prepared, {DEFAULT_FAN_PLUGIN});
    CHECK(first_fan_context(prepared).phase == RAW_FAN_POLICY_PHASE_BASE);
    const std::vector<ObservedLeaf> base = observed_plan_leaves(prepared.print);
    CHECK(base[0].fan_speed == 40);
    CHECK(base[1].fan_speed == 0);
    CHECK(base[5].fan_speed == 40);
    run_editors(prepared, {FAN_COOLING_PLUGIN});
    const slic3r_api::FanPolicy::Context timed = first_fan_context(prepared);
    CHECK(timed.phase == RAW_FAN_POLICY_PHASE_TIME);
    CHECK(timed.roles[RAW_FAN_ROLE_DEFAULT].percent == 60);
    const std::vector<ObservedLeaf> time = observed_plan_leaves(prepared.print);
    const float expected_time[] = {70, 50, 40, 0, 40, 70};
    for (size_t idx = 0; idx < 6; ++idx) CHECK(time[idx].fan_speed == expected_time[idx]);
    run_editors(prepared, {FIRST_LAYER_FAN_PLUGIN});
    const slic3r_api::FanPolicy::Context closed = first_fan_context(prepared);
    CHECK(closed.phase == RAW_FAN_POLICY_PHASE_CLOSED);
    CHECK(closed.roles[RAW_FAN_ROLE_DEFAULT].percent == 15);
    CHECK(closed.roles[RAW_FAN_ROLE_PERIMETER].percent == 17);
    run_editors(prepared, {DYNAMIC_FAN_PLUGIN}); // disabled graph is an identity
    const std::vector<ObservedLeaf> final = observed_plan_leaves(prepared.print);
    const float expected_final[] = {17, 12, 40, 0, 40, 17};
    for (size_t idx = 0; idx < 6; ++idx) {
        CHECK(final[idx].fan_speed == expected_final[idx]);
        REQUIRE(final[idx].direct_fan != nullptr);
        CHECK(final[idx].direct_fan->mode == (idx < 5 ? modes[idx] : FAN_REQUEST_MODE_AUTOMATIC));
        CHECK(final[idx].speed == 20.f);
        CHECK(root.child(idx).length() == Approx(lengths[idx]));
    }
    CHECK(root.get_property<ExtrusionPropertyFan>() == nullptr);
}

TEST_CASE("Fan closure works without optional time cooling and retains late family fallbacks",
          "[plugins][layer-extrusion-edit][fan-policy]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedSpeedPrint prepared;
    const DynamicPrintConfig config = speed_config({
        {"default_fan_speed", "!100"}, {"perimeter_fan_speed", "40"},
        {"solid_infill_fan_speed", "60"}, {"top_fan_speed", "!100"},
        {"fan_below_layer_time", "10"}, {"slowdown_below_layer_time", "5"},
        {"disable_fan_first_layers", "0"}, {"full_fan_speed_layer", "0"}});
    // No speeds: absence of cooling must not invoke the timing estimator.
    prepare_ordered_plan(prepared, config, {{{ExtrusionRole::ExternalPerimeter, 0.2},
        {ExtrusionRole::ThinWall, 0.2}, {ExtrusionRole::Ironing, 0.2}}});
    run_editors(prepared, {DEFAULT_FAN_PLUGIN, FIRST_LAYER_FAN_PLUGIN});
    CHECK(first_fan_context(prepared).phase == RAW_FAN_POLICY_PHASE_CLOSED);
    const std::map<uint16_t, ObservedLeaf> leaves = observed_by_role(observed_plan_leaves(prepared.print));
    CHECK(leaves.at(uint16_t(ExtrusionRole::ExternalPerimeter)).fan_speed == 40);
    CHECK(leaves.at(uint16_t(ExtrusionRole::ThinWall)).fan_speed == 40);
    CHECK(leaves.at(uint16_t(ExtrusionRole::Ironing)).fan_speed == 60);
    const PrintingLayerGroup &layer = prepared.print.printing_plan()->groups.front().layers.front();
    const slic3r_api::ConstPrintingLayerGroup view(reinterpret_cast<const printing_layer_group_handle *>(&layer));
    CHECK(registered_layer_time_property_key().get(view.properties()) == nullptr);
}

// Publish one first-layer program on every tool group, as an optional provider does.
void publish_first_layer_program(PreparedSpeedPrint &prepared, const slic3r_api::FanPolicy::FirstLayerProgram &program)
{
    const PluginPropertyKey<slic3r_api::FanPolicy::FirstLayerProgram> key = slic3r_api::FanPolicy::first_layer_program_key(
        reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
    for (PrintingGroup &group : prepared.print.mutable_printing_plan().groups)
        for (PrintingLayerGroup &layer : group.layers)
            for (PrintingToolGroup &tool : layer.tool_groups) {
                slic3r_api::PrintingToolGroup view(reinterpret_cast<printing_tool_group_handle *>(&tool));
                key.get_or_add(view.properties()) = program;
            }
}

TEST_CASE("First-layer fan stage applies a published program instead of its D F settings",
          "[plugins][layer-extrusion-edit][fan-policy]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    // Without a program, disable_fan_first_layers = 1 would stop both paths.
    const DynamicPrintConfig config = speed_config({
        {"default_fan_speed", "80"}, {"bridge_fan_speed", "60"}, {"fan_below_layer_time", "0"},
        {"slowdown_below_layer_time", "0"}, {"disable_fan_first_layers", "1"}, {"full_fan_speed_layer", "0"}});
    slic3r_api::FanPolicy::FirstLayerProgram program;
    program.state = slic3r_api::FanPolicy::FirstLayerProgram::Ramp;
    program.factor = 0.5f;
    PreparedSpeedPrint prepared;
    prepare_ordered_plan(prepared, config, {{{ExtrusionRole::Perimeter, 0.2}, {ExtrusionRole::BridgeInfill, 0.2}}});
    publish_first_layer_program(prepared, program);
    run_editors(prepared, {DEFAULT_FAN_PLUGIN, FIRST_LAYER_FAN_PLUGIN});
    const std::map<uint16_t, ObservedLeaf> leaves = observed_by_role(observed_plan_leaves(prepared.print));
    CHECK(leaves.at(uint16_t(ExtrusionRole::Perimeter)).fan_speed == 40);
    CHECK(leaves.at(uint16_t(ExtrusionRole::BridgeInfill)).fan_speed == 60); // bridges do not ramp
    CHECK(first_fan_context(prepared).dynamic_max == 40);

    program.factor = 2.f;
    PreparedSpeedPrint invalid;
    prepare_ordered_plan(invalid, config, {{{ExtrusionRole::Perimeter, 0.2}}});
    publish_first_layer_program(invalid, program);
    ScopedActivePlugins active({DEFAULT_FAN_PLUGIN, FIRST_LAYER_FAN_PLUGIN});
    Orchestrator &orch = Orchestrator::instance();
    orch.reset_plugin_cancel();
    CHECK_THROWS(Steps::StepLayerExtrusionEdition::run_step(orch, invalid.print));
    orch.reset_plugin_cancel();
    orch.consume_plugin_messages();
}

TEST_CASE("Standard fan honors the configured overhang role when the dynamic graph is disabled",
          "[plugins][layer-extrusion-edit][fan-policy]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedSpeedPrint prepared;
    const DynamicPrintConfig config = speed_config({
        {"default_fan_speed", "20"}, {"perimeter_fan_speed", "40"}, {"overhangs_fan_speed", "75"},
        {"fan_below_layer_time", "0"}, {"slowdown_below_layer_time", "0"},
        {"disable_fan_first_layers", "0"}, {"full_fan_speed_layer", "0"}});
    prepare_ordered_plan(prepared, config, {{{ExtrusionRole::OverhangPerimeter, 0.2, 20.f}}});
    run_fan_editors(prepared);
    REQUIRE(observed_plan_leaves(prepared.print).size() == 1);
    CHECK(observed_plan_leaves(prepared.print).front().fan_speed == 75);
}

TEST_CASE("Dynamic fan only increases modifiable overhangs within closed first-layer bounds",
          "[plugins][layer-extrusion-edit][fan-policy][fan-request]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const fan_request_mode other = orchestrator_register_runtime_id(
        reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()), FAN_REQUEST_MODE_ID_TYPE, "tests.dynamic.fan.other");
    for (int program : {0, 1, 2}) {
        INFO(program);
        PreparedSpeedPrint prepared;
        DynamicPrintConfig config = speed_config({
            {"default_fan_speed", "20"}, {"perimeter_fan_speed", "40"},
            {"overhangs_fan_speed", "75"}, {"fan_below_layer_time", "0"}, {"slowdown_below_layer_time", "0"},
            {"disable_fan_first_layers", "0"}, {"full_fan_speed_layer", "0"}});
        if (program == 1) config.set_deserialize_strict("disable_fan_first_layers", "1");
        if (program == 2) config.set_deserialize_strict("full_fan_speed_layer", "4");
        config.set_key_value("overhangs_dynamic_fan_speed", new ConfigOptionGraphs({
            GraphData(0, 2, GraphData::GraphType::LINEAR, {{0., 80.}, {100., 80.}})}));
        const ExtrusionSpec spec{ExtrusionRole::OverhangPerimeter, 0.2, 20.f};
        prepare_ordered_plan(prepared, config, {{spec, spec, spec, spec, spec, spec, spec}});
        ExtrusionEntity &root = single_plan_root(prepared.print);
        REQUIRE(root.child_count() == 7);
        const float initial[] = {90, 40, 40, 0, 40};
        const fan_request_mode modes[] = {FAN_REQUEST_MODE_AUTOMATIC, FAN_REQUEST_MODE_AUTOMATIC,
            FAN_REQUEST_MODE_OVERRIDE, FAN_REQUEST_MODE_OVERRIDE, other};
        for (size_t idx = 0; idx < 7; ++idx) {
            if (idx < 5) root.child(idx).add_property(ExtrusionPropertyFan(initial[idx], modes[idx]));
            if (idx < 6) {
                slic3r_api::MutableExtrusionEntity leaf(reinterpret_cast<extrusion_entity_handle *>(&root.child(idx)));
                slic3r_api::EPropertyOverhang &overhang = leaf.get_or_add(slic3r_api::EPropertyOverhang::key);
                overhang.start_distance_from_prev_layer = 0.5f;
                overhang.end_distance_from_prev_layer = 0.5f;
            }
        }
        publish_fan_test_duration(prepared, 7.5);
        run_fan_editors(prepared);
        // Overhang requests share the bridge ramp exception. Dynamic cooling
        // never lowers an existing Automatic above its own closed ceiling.
        const float expected[3][7] = {{90, 80, 40, 0, 40, 80, 40}, {0, 0, 40, 0, 40, 0, 0}, {90, 40, 40, 0, 40, 10, 10}};
        const std::vector<ObservedLeaf> leaves = observed_plan_leaves(prepared.print);
        REQUIRE(leaves.size() == 7);
        for (size_t idx = 0; idx < 7; ++idx) {
            CHECK(leaves[idx].fan_speed == expected[program][idx]);
            REQUIRE(leaves[idx].direct_fan != nullptr);
            CHECK(leaves[idx].direct_fan->mode == (idx < 5 ? modes[idx] : FAN_REQUEST_MODE_AUTOMATIC));
        }
    }
}

TEST_CASE("Fan consumers reject missing malformed and out of order published contexts",
          "[plugins][layer-extrusion-edit][fan-policy]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedSpeedPrint prepared;
    const DynamicPrintConfig config = speed_config({
        {"default_fan_speed", "20"}, {"disable_fan_first_layers", "0"}, {"full_fan_speed_layer", "0"}});
    prepare_ordered_plan(prepared, config, {{{ExtrusionRole::Perimeter, 0.2, 20.f}}});
    publish_fan_test_duration(prepared, 7.5);
    Orchestrator &orch = Orchestrator::instance();
    const PluginPropertyKey<slic3r_api::FanPolicy::Context> key = slic3r_api::FanPolicy::context_property_key(
        reinterpret_cast<orchestrator_handle *>(&orch));
    PrintingToolGroup &tool = prepared.print.mutable_printing_plan().groups.front().layers.front().tool_groups.front();
    slic3r_api::PrintingToolGroup view(reinterpret_cast<printing_tool_group_handle *>(&tool));
    for (const char *consumer : {FAN_COOLING_PLUGIN, FIRST_LAYER_FAN_PLUGIN, DYNAMIC_FAN_PLUGIN}) {
        INFO(consumer);
        ScopedActivePlugins active({consumer});
        orch.reset_plugin_cancel();
        CHECK_THROWS(Steps::StepLayerExtrusionEdition::run_step(orch, prepared.print));
        CHECK(orch.is_plugin_cancelled());
        orch.reset_plugin_cancel();
    }
    run_editors(prepared, {DEFAULT_FAN_PLUGIN});
    key.get_or_add(view.properties()).struct_size = 0;
    {
        ScopedActivePlugins active({FAN_COOLING_PLUGIN});
        CHECK_THROWS(Steps::StepLayerExtrusionEdition::run_step(orch, prepared.print));
        orch.reset_plugin_cancel();
    }
    run_editors(prepared, {DEFAULT_FAN_PLUGIN, FIRST_LAYER_FAN_PLUGIN});
    {
        ScopedActivePlugins active({FAN_COOLING_PLUGIN});
        CHECK_THROWS(Steps::StepLayerExtrusionEdition::run_step(orch, prepared.print));
        orch.reset_plugin_cancel();
    }
    CHECK(first_fan_context(prepared).phase == RAW_FAN_POLICY_PHASE_CLOSED);
    orch.consume_plugin_messages();
}

TEST_CASE("Fan host policies keep separate priorities configuration owners and read sets",
          "[plugins][layer-extrusion-edit][fan-policy][api]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *ids[] = {DEFAULT_FAN_PLUGIN, FAN_COOLING_PLUGIN, FIRST_LAYER_FAN_PLUGIN, DYNAMIC_FAN_PLUGIN};
    const int priorities[] = {20, 22, 24, 26};
    // slowdown_below_layer_time is a base host setting: FanCooling only reads it.
    const size_t defined_counts[] = {12, 2, 2, 0};
    std::set<std::string> owners;
    for (size_t idx = 0; idx < 4; ++idx) {
        Plugin *plugin = Orchestrator::instance().get_plugin(ids[idx]);
        REQUIRE(plugin != nullptr);
        CHECK(plugin->get_priority() == priorities[idx]);
        CHECK(plugin->get_step() == STEP_LAYER_EXTRUSION_EDIT);
        CHECK(plugin->get_defined_config_keys().size() == defined_counts[idx]);
        if (idx > 0) CHECK(plugin->get_exclusive_group().empty());
        for (const Plugin::DefinedConfigKey &key : plugin->get_defined_config_keys()) {
            CHECK(owners.insert(key.key).second);
            const ConfigOptionDef *definition = PrintConfigDef::instance().get(key.key);
            REQUIRE(definition != nullptr);
            CHECK(definition->invalidates_step == STEP_LAYER_EXTRUSION_EDIT);
        }
    }
    CHECK(owners.size() == 16);
    CHECK(owners.count("slowdown_below_layer_time") == 0);
    const ConfigOptionDef *slowdown_def = PrintConfigDef::instance().get("slowdown_below_layer_time");
    REQUIRE(slowdown_def != nullptr);
    CHECK(slowdown_def->invalidates_step == STEP_LAYER_EXTRUSION_EDIT);
    Plugin *slowdown = Orchestrator::instance().get_plugin(SLOWDOWN_PLUGIN);
    REQUIRE(slowdown != nullptr);
    // After every speed editor (max 15) and before the first fan policy (20).
    CHECK(slowdown->get_priority() == 16);
    CHECK(slowdown->get_step() == STEP_LAYER_EXTRUSION_EDIT);
    CHECK(slowdown->get_exclusive_group().empty());
    CHECK(slowdown->get_defined_config_keys().empty());
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


TEST_CASE("First-layer flow plugin owns the first-layer flow ratio",
          "[plugins][layer-extrusion-edit][first-layer-flow][api]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    Plugin *flow_plugin = orchestrator.get_plugin(FIRST_LAYER_FLOW_PLUGIN);
    Plugin *speed_plugin = orchestrator.get_plugin(DEFAULT_SPEED_PLUGIN);
    REQUIRE(flow_plugin != nullptr);
    REQUIRE(speed_plugin != nullptr);
    CHECK(flow_plugin->get_step() == STEP_LAYER_EXTRUSION_EDIT);
    // The speed plugin must see the final flow.
    CHECK(flow_plugin->get_priority() < speed_plugin->get_priority());

    std::vector<std::string> defined;
    for (const Plugin::DefinedConfigKey &key : flow_plugin->get_defined_config_keys())
        defined.push_back(key.key);
    CHECK(defined == std::vector<std::string>{ "first_layer_flow_ratio" });

    const ConfigOptionDef *def = PrintConfigDef::instance().get("first_layer_flow_ratio");
    REQUIRE(def != nullptr);
    CHECK(def->type == coPercent);
    CHECK(def->default_value->serialize() == "100%");
    CHECK(def->invalidates_step == STEP_LAYER_EXTRUSION_EDIT);
}

TEST_CASE("First-layer flow scales deposited paths of the first layer only",
          "[plugins][layer-extrusion-edit][first-layer-flow]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedSpeedPrint prepared;
    const DynamicPrintConfig config = speed_config({
        {"first_layer_flow_ratio", "125%"}, {"filament_first_layer_flow_ratio", "120%"}
    });
    ExtrusionSpec force_e{ExtrusionRole::Perimeter, 0.05};
    force_e.force_e_per_mm = true;
    prepare_ordered_plan(prepared, config, {
        { ExtrusionSpec{ExtrusionRole::Perimeter, 0.2}, ExtrusionSpec{ExtrusionRole::Skirt, 0.2},
          ExtrusionSpec{ExtrusionRole::SolidInfill, 0.1}, force_e },
        { ExtrusionSpec{ExtrusionRole::Perimeter, 0.2} }
    });

    run_editors(prepared, {FIRST_LAYER_FLOW_PLUGIN});
    size_t first_layer_leaves = 0;
    size_t upper_layer_leaves = 0;
    for (const ObservedFlow &leaf : observed_plan_flows(prepared.print)) {
        INFO("layer " << leaf.layer_idx << ", role " << uint16_t(leaf.role()) << ", flow " << leaf.mm3_per_mm);
        if (leaf.layer_idx > 0) {
            ++upper_layer_leaves;
            CHECK(leaf.mm3_per_mm == Approx(0.2));
            continue;
        }
        ++first_layer_leaves;
        if (leaf.role == ExtrusionRole::SolidInfill)
            CHECK(leaf.mm3_per_mm == Approx(0.1 * 1.25 * 1.2));
        else if (leaf.mm3_per_mm < 0.1)
            CHECK(leaf.mm3_per_mm == Approx(0.05)); // already E per mm: untouched
        else
            CHECK(leaf.mm3_per_mm == Approx(0.2 * 1.25 * 1.2));
    }
    CHECK(first_layer_leaves == 4);
    CHECK(upper_layer_leaves == 1);
}

TEST_CASE("First-layer flow leaves paths untouched at 100%",
          "[plugins][layer-extrusion-edit][first-layer-flow]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedSpeedPrint prepared;
    prepare_ordered_plan(prepared, speed_config({}), {{ExtrusionSpec{ExtrusionRole::Perimeter, 0.2}}});
    run_editors(prepared, {FIRST_LAYER_FLOW_PLUGIN});
    const std::vector<ObservedFlow> leaves = observed_plan_flows(prepared.print);
    REQUIRE(leaves.size() == 1);
    CHECK(leaves.front().mm3_per_mm == 0.2);
}

TEST_CASE("First-layer flow splits a path at the border of regions with different ratios",
          "[plugins][layer-extrusion-edit][first-layer-flow][regional]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedSpeedPrint prepared;
    // The path crosses base / modifier / base; only the 4 mm wide modifier uses 200%.
    // Support and skirt are printed outside the region masks: they are not split
    // and use the object's default region (the base part, 100%), whatever the
    // order of the source regions.
    prepare_regional_process_plan(prepared, speed_config({}), {{"first_layer_flow_ratio", "200%"}},
                                  "first_layer_flow_ratio",
                                  {ExtrusionSpec{ExtrusionRole::Perimeter, 0.2},
                                   ExtrusionSpec{ExtrusionRole::SupportMaterial, 0.1},
                                   ExtrusionSpec{ExtrusionRole::Skirt, 0.05}});
    run_editors(prepared, {FIRST_LAYER_FLOW_PLUGIN});

    double base_length = 0.;
    double modifier_length = 0.;
    size_t outside_leaves = 0;
    for (const ObservedFlow &leaf : observed_plan_flows(prepared.print)) {
        INFO("flow " << leaf.mm3_per_mm << ", length " << leaf.length);
        if (leaf.role == ExtrusionRole::SupportMaterial || leaf.role == ExtrusionRole::Skirt) {
            ++outside_leaves;
            CHECK(leaf.mm3_per_mm == Approx(leaf.role == ExtrusionRole::Skirt ? 0.05 : 0.1));
            CHECK(leaf.length == Approx(17.).margin(0.01));
        } else if (leaf.mm3_per_mm == Approx(0.4))
            modifier_length += leaf.length;
        else {
            CHECK(leaf.mm3_per_mm == Approx(0.2));
            base_length += leaf.length;
        }
    }
    CHECK(outside_leaves == 2);
    CHECK(modifier_length == Approx(4.).margin(0.01));
    CHECK(base_length == Approx(13.).margin(0.01));
}

TEST_CASE("First-layer exterior flow uses object defaults even for a uniform material partition",
          "[plugins][layer-extrusion-edit][first-layer-flow][regional]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *material_ratio = GENERATE("100%", "200%");
    PreparedSpeedPrint prepared;
    prepare_regional_process_plan(prepared,
        speed_config({{"first_layer_flow_ratio", "150%"}, {"perimeter_speed", "70"}}),
        {{"first_layer_flow_ratio", material_ratio}, {"perimeter_speed", "30"}}, "perimeter_speed",
        {ExtrusionSpec{ExtrusionRole::Perimeter, 0.2}, ExtrusionSpec{ExtrusionRole::SupportMaterial, 0.1},
         ExtrusionSpec{ExtrusionRole::Skirt, 0.05}}, false, {{"first_layer_flow_ratio", material_ratio}});
    run_editors(prepared, {FIRST_LAYER_FLOW_PLUGIN});
    const std::vector<ObservedFlow> flows = observed_plan_flows(prepared.print);
    REQUIRE(flows.size() == 3);
    for (const ObservedFlow &flow : flows) {
        if (flow.role == ExtrusionRole::SupportMaterial)
            CHECK(flow.mm3_per_mm == Approx(0.15));
        else if (flow.role == ExtrusionRole::Skirt)
            CHECK(flow.mm3_per_mm == Approx(0.075));
        else
            CHECK(flow.mm3_per_mm == Approx(material_ratio[0] == '1' ? 0.2 : 0.4));
    }
}

TEST_CASE("First-layer flow selects one regional value for a deposit without duplicating its command",
          "[plugins][layer-extrusion-edit][first-layer-flow][regional][deposit]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool inside_modifier = GENERATE(false, true);
    PreparedSpeedPrint prepared;
    prepare_regional_process_plan(prepared,
        speed_config({{"first_layer_flow_ratio", "125%"}, {"filament_first_layer_flow_ratio", "120%"}}),
        {{"first_layer_flow_ratio", "200%"}}, "first_layer_flow_ratio", {ExtrusionSpec{ExtrusionRole::Perimeter, 0.2}});
    ExtrusionEntity &root = single_plan_root(prepared.print);
    REQUIRE(root.child_count() == 1);
    ExtrusionEntity &deposit = root.child(0);
    REQUIRE(deposit.has_polyline());
    Point point = (deposit.first_point() + deposit.last_point()) / 2;
    if (!inside_modifier)
        point.x() -= scale_i(5.);
    deposit.polyline_ref() = ArcPolyline();
    deposit.polyline_ref().append(point);
    deposit.add_property(ExtrusionPropertySpecialCommand(
        ExtrusionPropertySpecialCommand::Code::DEPOSIT, 0.5));
    run_editors(prepared, {FIRST_LAYER_FLOW_PLUGIN});
    const std::vector<ObservedFlow> flows = observed_plan_flows(prepared.print);
    REQUIRE(flows.size() == 1);
    CHECK(flows.front().mm3_per_mm == Approx(0.2 * (inside_modifier ? 2. : 1.25) * 1.2));
    CHECK(deposit.child_count() == 0);
    CHECK(deposit.polyline_ref().size() == 1);
    CHECK(deposit.first_point() == point);
    REQUIRE(deposit.get_property<ExtrusionPropertySpecialCommand>() != nullptr);
    CHECK(deposit.get_property<ExtrusionPropertySpecialCommand>()->extra_data == 0.5);
}

TEST_CASE("Composable fan stages pass their declared linear STL export catalogue",
          "[plugins][plugin-e2e][fan-policy][fan-e2e]")
{
    namespace E2E = Test::PluginE2E;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    ScopedActivePlugins active({DEFAULT_FAN_PLUGIN, FAN_COOLING_PLUGIN, FIRST_LAYER_FAN_PLUGIN,
        DYNAMIC_FAN_PLUGIN, DEFAULT_SPEED_PLUGIN, "ordering.extrusion_tree.advanced"}, true);
    Orchestrator &host = Orchestrator::instance();
    DynamicPrintConfig baseline = DynamicPrintConfig::full_print_config();
    baseline.set_deserialize_strict({{"layer_height", "0.2"}, {"first_layer_height", "0.2"},
        {"default_fan_speed", "20"}, {"disable_fan_first_layers", "0"}, {"full_fan_speed_layer", "0"},
        {"fan_below_layer_time", "0"}, {"slowdown_below_layer_time", "0"},
        {"step_gcode_plugin", "gcode.printing_plan_file_writer"}, {"gcode_flavor", "marlin2"}});
    const std::string ordering_key = Steps::plugin_exclusive_group_option_key(host, STEP_ORDERING, "ordering.extrusion_tree");
    REQUIRE_FALSE(ordering_key.empty());
    baseline.set_deserialize_strict(ordering_key, "ordering.extrusion_tree.advanced");
    std::vector<E2E::Domain> domains;
    const auto add = [&domains](const std::string &key, const std::vector<std::string> &values) {
        E2E::Domain domain{{key}, {}};
        for (const std::string &value : values)
            domain.cases.push_back({key + "=" + value, {{key, value}}});
        domains.push_back(std::move(domain));
    };
    for (const char *key : {"default_fan_speed", "bridge_fan_speed", "external_perimeter_fan_speed",
            "gap_fill_fan_speed", "infill_fan_speed", "internal_bridge_fan_speed", "overhangs_fan_speed",
            "perimeter_fan_speed", "solid_infill_fan_speed", "support_material_fan_speed",
            "support_material_interface_fan_speed", "top_fan_speed"})
        add(key, {"!0", "0", "1", "100"});
    add("disable_fan_first_layers", {"0", "2"});
    add("full_fan_speed_layer", {"0", "4"});
    add("fan_below_layer_time", {"0", "60"});
    add("slowdown_below_layer_time", {"0", "5"});
    add("max_fan_speed", {"0", "100"});
    add("travel_speed", {"50", "150"});
    const ConfigOptionGraphs low({GraphData(0, 2, GraphData::GraphType::LINEAR, {{0., 0.}, {100., 0.}})});
    const ConfigOptionGraphs high({GraphData(0, 2, GraphData::GraphType::LINEAR, {{0., 100.}, {100., 100.}})});
    add("overhangs_dynamic_fan_speed", {baseline.optptr("overhangs_dynamic_fan_speed")->serialize(),
        low.serialize(), high.serialize()});
    const TriangleMesh mesh = E2E::stl_fixture(true);
    for (const E2E::Case &test : E2E::linear_cases(host,
            {DEFAULT_FAN_PLUGIN, FAN_COOLING_PLUGIN, FIRST_LAYER_FAN_PLUGIN, DYNAMIC_FAN_PLUGIN}, baseline, domains)) {
        INFO(test.name);
        Print print;
        Model model;
        Test::init_print({mesh}, print, model, E2E::apply_case(baseline, test));
        print.process();
        const boost::filesystem::path path = Test::GCode::temporary_gcode_path();
        std::string code;
        try {
            code = Test::GCode::read_text_file(host.export_gcode_cli(print, path.string()));
            Test::GCode::remove_output_pair(path);
        } catch (...) {
            Test::GCode::remove_output_pair(path);
            throw;
        }
        CHECK(code.find("G1 ") != std::string::npos);
        CHECK(code.find(" E") != std::string::npos);
        const std::vector<ObservedLeaf> leaves = observed_plan_leaves(print);
        REQUIRE_FALSE(leaves.empty());
        for (const ObservedLeaf &leaf : leaves) {
            CHECK(std::isfinite(leaf.fan_speed));
            CHECK(leaf.fan_speed >= -1.f);
            CHECK(leaf.fan_speed <= 100.f);
        }
    }
}

TEST_CASE("Short-layer slowdown and cooling pass their declared linear STL export catalogue",
          "[plugins][plugin-e2e][slowdown-e2e]")
{
    namespace E2E = Test::PluginE2E;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    ScopedActivePlugins active({SLOWDOWN_PLUGIN, DEFAULT_SPEED_PLUGIN, DEFAULT_FAN_PLUGIN,
        FAN_COOLING_PLUGIN, FIRST_LAYER_FAN_PLUGIN, DYNAMIC_FAN_PLUGIN, "ordering.extrusion_tree.advanced"}, true);
    Orchestrator &host = Orchestrator::instance();
    DynamicPrintConfig baseline = DynamicPrintConfig::full_print_config();
    baseline.set_deserialize_strict({{"layer_height", "0.2"}, {"first_layer_height", "0.2"},
        {"slowdown_below_layer_time", "5"}, {"min_print_speed", "10"}, {"max_speed_reduction", "90%"},
        {"travel_speed", "100"}, {"default_fan_speed", "20"}, {"max_fan_speed", "100"},
        {"fan_below_layer_time", "60"}, {"disable_fan_first_layers", "0"}, {"full_fan_speed_layer", "0"},
        {"step_gcode_plugin", "gcode.printing_plan_file_writer"}, {"gcode_flavor", "marlin2"}});
    const std::string ordering_key = Steps::plugin_exclusive_group_option_key(host, STEP_ORDERING, "ordering.extrusion_tree");
    REQUIRE_FALSE(ordering_key.empty());
    baseline.set_deserialize_strict(ordering_key, "ordering.extrusion_tree.advanced");
    const std::vector<E2E::Domain> domains = {
        {{"max_speed_reduction"}, {{"unbounded-zero", {{"max_speed_reduction", "0%"}}},
            {"half-reduction", {{"max_speed_reduction", "50%"}}}, {"unbounded-full", {{"max_speed_reduction", "100%"}}}}},
        {{"min_print_speed"}, {{"no-floor", {{"min_print_speed", "0"}}},
            {"low-floor", {{"min_print_speed", "1"}}}, {"high-floor", {{"min_print_speed", "80"}}}}},
        {{"slowdown_below_layer_time"}, {{"disabled", {{"slowdown_below_layer_time", "0"}}},
            {"already-long", {{"slowdown_below_layer_time", "0.001"}}}, {"long-goal", {{"slowdown_below_layer_time", "30"}}}}},
        {{"travel_speed"}, {{"slow-travel", {{"travel_speed", "40"}}}, {"fast-travel", {{"travel_speed", "200"}}}}},
        {{"fan_below_layer_time"}, {{"no-time-fan", {{"fan_below_layer_time", "0"}}},
            {"short-time-fan", {{"fan_below_layer_time", "10"}}}}},
        {{"max_fan_speed"}, {{"fan-stop", {{"max_fan_speed", "0"}}}, {"fan-half", {{"max_fan_speed", "50"}}}}}
    };
    const TriangleMesh mesh = E2E::stl_fixture(false);
    std::vector<std::string> baseline_moves;
    bool saw_reduced_speed = false;
    for (const E2E::Case &test : E2E::linear_cases(host, {SLOWDOWN_PLUGIN, FAN_COOLING_PLUGIN}, baseline, domains)) {
        INFO(test.name);
        const DynamicPrintConfig config = E2E::apply_case(baseline, test);
        Print print;
        Model model;
        Test::init_print({mesh}, print, model, config);
        print.process();
        const boost::filesystem::path path = Test::GCode::temporary_gcode_path();
        std::string code;
        try {
            code = Test::GCode::read_text_file(host.export_gcode_cli(print, path.string()));
            Test::GCode::remove_output_pair(path);
        } catch (...) {
            Test::GCode::remove_output_pair(path);
            throw;
        }
        REQUIRE(print.printing_plan() != nullptr);
        CHECK((code.find("M106") != std::string::npos || code.find("M107") != std::string::npos));
        // Only feedrates may differ: compare every emitted motion/extrusion token
        // in order, skipping commands that set a feedrate without any movement.
        std::vector<std::string> moves;
        std::istringstream lines(code);
        std::string line;
        while (std::getline(lines, line)) {
            std::istringstream words(line.substr(0, line.find(';')));
            std::string command;
            words >> command;
            if (command != "G0" && command != "G1" && command != "G2" && command != "G3")
                continue;
            std::string motion = command;
            std::string word;
            while (words >> word)
                if (word.front() != 'F') motion += " " + word;
            if (motion != command) moves.push_back(std::move(motion));
        }
        REQUIRE_FALSE(moves.empty());
        if (baseline_moves.empty()) baseline_moves = moves;
        CHECK(moves == baseline_moves);
        const slic3r_api::Config config_view(Slic3r::ApiHost::to_config_handle(&print.config()));
        const slic3r_api::ConstPrintingPlan plan_view(
            reinterpret_cast<const printing_plan_handle *>(print.printing_plan()));
        const std::vector<slic3r_api::PrintingLayerTimeEstimate> totals =
            slic3r_api::PrintingPlanTimeEstimator(config_view).estimate(plan_view);
        size_t resolved_count = 0;
        for (const slic3r_api::PrintingLayerTimeEstimate &total : totals) {
            const PrintingLayerGroup &layer = print.printing_plan()->groups[total.group_idx].layers[total.layer_group_idx];
            if (layer.tool_groups.empty()) continue;
            const slic3r_api::ConstPrintingLayerGroup layer_view(
                reinterpret_cast<const printing_layer_group_handle *>(&layer));
            const PrintingLayerTimeProperty *time = registered_layer_time_property_key().get(layer_view.properties());
            REQUIRE(time != nullptr);
            CHECK(time->duration_seconds == Approx(total.duration_seconds).epsilon(1e-6));
            if (time->is_resolved()) ++resolved_count;
        }
        if (config.opt_float("slowdown_below_layer_time", 0) == 0.0)
            CHECK(resolved_count == 0);
        else
            CHECK(resolved_count > 0);
        for (const ObservedLeaf &leaf : observed_plan_leaves(print))
            if (leaf.role == ExtrusionRole::Perimeter && leaf.speed < 10.f)
                saw_reduced_speed = true;
    }
    CHECK(saw_reduced_speed);
}

TEST_CASE("Default speed declared settings pass a linear STL export matrix",
          "[plugins][plugin-e2e][layer-extrusion-edit][speed-e2e]")
{
    namespace E2E = Test::PluginE2E;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    ScopedActivePlugins active({DEFAULT_SPEED_PLUGIN, "ordering.extrusion_tree.advanced"}, true);
    Orchestrator &host = Orchestrator::instance();
    DynamicPrintConfig baseline = DynamicPrintConfig::full_print_config();
    baseline.set_deserialize_strict({{"layer_height", "0.2"}, {"first_layer_height", "0.2"},
        {"step_gcode_plugin", "gcode.printing_plan_file_writer"}, {"gcode_flavor", "marlin2"}});
    const std::string ordering_key = Steps::plugin_exclusive_group_option_key(host, STEP_ORDERING, "ordering.extrusion_tree");
    REQUIRE_FALSE(ordering_key.empty());
    baseline.set_deserialize_strict(ordering_key, "ordering.extrusion_tree.advanced");
    std::vector<E2E::Domain> domains;
    for (const char *key : {"bridge_speed", "brim_speed", "external_perimeter_speed", "first_layer_infill_speed",
             "first_layer_speed", "gap_fill_speed", "infill_speed", "internal_bridge_speed", "ironing_speed",
             "overhangs_speed", "perimeter_speed", "solid_infill_speed", "support_material_interface_speed",
             "support_material_speed", "thin_walls_speed", "top_solid_infill_speed"}) {
        E2E::Domain domain{{key}, {}};
        for (const char *value : {"0", "25", "50%"})
            domain.cases.push_back({std::string(key) + "=" + value, {{key, value}}});
        domains.push_back(std::move(domain));
    }
    const auto add = [&domains](const std::string &key, const std::vector<std::string> &values) {
        E2E::Domain domain{{key}, {}};
        for (const std::string &value : values)
            domain.cases.push_back({key + "=" + value, {{key, value}}});
        domains.push_back(std::move(domain));
    };
    add("autospeed_min_thin_flow", {"0", "!0", "50%"});
    add("filament_max_speed", {"0", "25"});
    add("filament_max_volumetric_speed", {"0", "2"});
    add("first_layer_min_speed", {"0", "30"});
    add("max_print_speed", {"20", "100"});
    add("max_volumetric_speed", {"0", "2"});
    add("milling_speed", {"10", "30"});
    add("overhangs", {"0", "1"});
    add("travel_speed", {"50", "150"});
    const TriangleMesh mesh = E2E::stl_fixture(true);
    for (const E2E::Case &test : E2E::linear_cases(host, {DEFAULT_SPEED_PLUGIN}, baseline, domains)) {
        INFO(test.name);
        Print print;
        Model model;
        Test::init_print({mesh}, print, model, E2E::apply_case(baseline, test));
        print.process();
        const boost::filesystem::path path = Test::GCode::temporary_gcode_path();
        std::string code;
        try {
            const std::string exported = host.export_gcode_cli(print, path.string());
            code = Test::GCode::read_text_file(exported);
            Test::GCode::remove_output_pair(path);
        } catch (...) {
            Test::GCode::remove_output_pair(path);
            throw;
        }
        CHECK(code.find("G1 ") != std::string::npos);
        CHECK(code.find(" E") != std::string::npos);
        const std::vector<ObservedLeaf> leaves = observed_plan_leaves(print);
        REQUIRE_FALSE(leaves.empty());
        for (const ObservedLeaf &leaf : leaves) {
            CHECK(std::isfinite(leaf.speed));
            CHECK(leaf.speed > 0.);
        }
    }
}

TEST_CASE("First-layer flow declared settings pass a linear STL export matrix",
          "[plugins][plugin-e2e][first-layer-flow][first-layer-flow-e2e]")
{
    namespace E2E = Test::PluginE2E;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    ScopedActivePlugins active({FIRST_LAYER_FLOW_PLUGIN, "ordering.extrusion_tree.advanced"}, true);
    Orchestrator &host = Orchestrator::instance();
    DynamicPrintConfig baseline = DynamicPrintConfig::full_print_config();
    baseline.set_deserialize_strict({{"layer_height", "0.2"}, {"first_layer_height", "0.2"},
        {"first_layer_flow_ratio", "125%"}, {"filament_first_layer_flow_ratio", "120%"},
        {"step_gcode_plugin", "gcode.printing_plan_file_writer"}, {"gcode_flavor", "marlin2"}});
    const std::string ordering_key = Steps::plugin_exclusive_group_option_key(host, STEP_ORDERING, "ordering.extrusion_tree");
    REQUIRE_FALSE(ordering_key.empty());
    baseline.set_deserialize_strict(ordering_key, "ordering.extrusion_tree.advanced");
    const std::vector<E2E::Domain> domains = {
        {{"first_layer_flow_ratio"}, {{"region 50%", {{"first_layer_flow_ratio", "50%"}}},
            {"region 100%", {{"first_layer_flow_ratio", "100%"}}}, {"region 150%", {{"first_layer_flow_ratio", "150%"}}}}},
        {{"filament_first_layer_flow_ratio"}, {{"filament 50%", {{"filament_first_layer_flow_ratio", "50%"}}},
            {"filament 100%", {{"filament_first_layer_flow_ratio", "100%"}}},
            {"filament 150%", {{"filament_first_layer_flow_ratio", "150%"}}}}}
    };
    const std::vector<E2E::Case> cases = E2E::linear_cases(host, {FIRST_LAYER_FLOW_PLUGIN}, baseline, domains);
    const TriangleMesh mesh = E2E::stl_fixture(false);
    for (const E2E::Case &test : cases) {
        INFO(test.name);
        Print print;
        Model model;
        Test::init_print({mesh}, print, model, E2E::apply_case(baseline, test));
        print.process();
        const boost::filesystem::path path = Test::GCode::temporary_gcode_path();
        std::string code;
        try {
            const std::string exported = host.export_gcode_cli(print, path.string());
            code = Test::GCode::read_text_file(exported);
            Test::GCode::remove_output_pair(path);
        } catch (...) {
            Test::GCode::remove_output_pair(path);
            throw;
        }
        CHECK(code.find("G1 ") != std::string::npos);
        CHECK(code.find(" E") != std::string::npos);
        size_t material_leaves = 0;
        for (const ObservedFlow &flow : observed_plan_flows(print))
            if (flow.role == ExtrusionRole::Perimeter || flow.role == ExtrusionRole::ExternalPerimeter ||
                flow.role == ExtrusionRole::SolidInfill || flow.role == ExtrusionRole::InternalInfill) {
                ++material_leaves;
                CHECK(std::isfinite(flow.mm3_per_mm));
                CHECK(flow.mm3_per_mm > 0.);
            }
        CHECK(material_leaves > 0);
    }
}

TEST_CASE("First-layer flow rejects a zero ratio",
          "[plugins][layer-extrusion-edit][first-layer-flow][validation]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedSpeedPrint prepared;
    prepare_ordered_plan(prepared, speed_config({{"first_layer_flow_ratio", "0%"}}),
                         {{ExtrusionSpec{ExtrusionRole::Perimeter, 0.2}}});

    ScopedActivePlugins active({FIRST_LAYER_FLOW_PLUGIN});
    Orchestrator &orchestrator = Orchestrator::instance();
    orchestrator.reset_plugin_cancel();
    (void)orchestrator.consume_plugin_messages();
    CHECK_THROWS_AS(Steps::StepLayerExtrusionEdition::run_step(orchestrator, prepared.print), RuntimeError);
    bool found_ratio_error = false;
    for (const Orchestrator::PluginMessage &message : orchestrator.consume_plugin_messages())
        found_ratio_error = found_ratio_error || message.message.find("first_layer_flow_ratio") != std::string::npos;
    CHECK(found_ratio_error);
    orchestrator.reset_plugin_cancel();
}

TEST_CASE("First-layer flow feeds the volumetric speed limit",
          "[plugins][layer-extrusion-edit][first-layer-flow][speed]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    // 12 mm3/s over 0.2 mm3/mm gives 60 mm/s; at 200% the flow is 0.4 mm3/mm: 30 mm/s.
    const DynamicPrintConfig config = speed_config({
        {"perimeter_speed", "100"}, {"first_layer_speed", "100%"}, {"first_layer_flow_ratio", "200%"},
        {"max_print_speed", "0"}, {"max_volumetric_speed", "0"}, {"filament_max_speed", "0"},
        {"filament_max_volumetric_speed", "12"}
    });
    SECTION("with the first-layer flow plugin") {
        PreparedSpeedPrint prepared;
        prepare_ordered_plan(prepared, config, {{ExtrusionSpec{ExtrusionRole::Perimeter, 0.2}}});
        run_editors(prepared, {FIRST_LAYER_FLOW_PLUGIN, DEFAULT_SPEED_PLUGIN});
        const std::vector<ObservedLeaf> leaves = observed_plan_leaves(prepared.print);
        REQUIRE(leaves.size() == 1);
        CHECK(leaves.front().speed == Approx(30.f));
    }
    SECTION("the speed plugin no longer applies the ratio itself") {
        PreparedSpeedPrint prepared;
        prepare_ordered_plan(prepared, config, {{ExtrusionSpec{ExtrusionRole::Perimeter, 0.2}}});
        run_editors(prepared, {DEFAULT_SPEED_PLUGIN});
        const std::vector<ObservedLeaf> leaves = observed_plan_leaves(prepared.print);
        REQUIRE(leaves.size() == 1);
        CHECK(leaves.front().speed == Approx(60.f));
    }
}

// ---------------------------------------------------------------------------
// Short-layer slowdown plugin (layer_extrusion_edit.speed.slowdown)
// ---------------------------------------------------------------------------
// The numerical rules are covered by test_layer_extrusion_edit_slowdown_solver.cpp;
// the cases below check the adapter: leaf classification, speed writing, the
// published RESOLVED duration, its use by FanCooling and the invalidation.
// Leaves carry their own speed (100 mm/s) because no speed plugin runs here.

namespace {

// Printing config with an explicit slowdown goal. min_print_speed 10 and
// max_speed_reduction 90 % are the defaults, written here so that the tests do
// not depend on them.
DynamicPrintConfig slowdown_config(const char *goal = "5")
{
    return speed_config({{"slowdown_below_layer_time", goal}, {"min_print_speed", "10"},
                         {"max_speed_reduction", "90"}, {"travel_speed", "100"}});
}

// A leaf that already carries its own speed.
ExtrusionSpec slowdown_spec(ExtrusionRole role, float speed = 100.f)
{
    ExtrusionSpec spec{role, 0.2};
    spec.existing_speed = speed;
    return spec;
}

// Printed layers of the plan (the fixture may also create empty layer groups).
std::vector<PrintingLayerGroup *> printed_layers(PreparedSpeedPrint &prepared)
{
    std::vector<PrintingLayerGroup *> layers;
    for (PrintingGroup &group : prepared.print.mutable_printing_plan().groups)
        for (PrintingLayerGroup &layer : group.layers)
            if (!layer.tool_groups.empty())
                layers.push_back(&layer);
    return layers;
}

// Duration of the n-th printed layer according to the shared time model.
double estimate_printed_layer(PreparedSpeedPrint &prepared, size_t n = 0)
{
    PrintingPlan &plan = prepared.print.mutable_printing_plan();
    const slic3r_api::PrintingPlan plan_view(reinterpret_cast<printing_plan_handle *>(&plan));
    const slic3r_api::Config config_view(Slic3r::ApiHost::to_config_handle(&prepared.print.config()));
    const std::vector<slic3r_api::PrintingLayerTimeEstimate> estimates =
        slic3r_api::PrintingPlanTimeEstimator(config_view).estimate(plan_view);
    size_t printed = 0;
    for (const slic3r_api::PrintingLayerTimeEstimate &estimate : estimates)
        if (!plan.groups[estimate.group_idx].layers[estimate.layer_group_idx].tool_groups.empty() && printed++ == n)
            return estimate.duration_seconds;
    FAIL("printed layer not found");
    return -1.;
}

// Time property of the n-th printed layer, or nullptr when none is published.
const PrintingLayerTimeProperty *layer_time(PreparedSpeedPrint &prepared, size_t n = 0)
{
    PrintingLayerGroup &layer = *printed_layers(prepared).at(n);
    const slic3r_api::PrintingLayerGroup view(reinterpret_cast<printing_layer_group_handle *>(&layer));
    return registered_layer_time_property_key().get(view.properties());
}

// Effective speed of every plan leaf in plan order.
std::vector<float> leaf_speeds(PreparedSpeedPrint &prepared)
{
    std::vector<float> speeds;
    for (const ObservedLeaf &leaf : observed_plan_leaves(prepared.print))
        speeds.push_back(leaf.speed);
    return speeds;
}

// Published duration and leaf speeds, copied so they can be compared across Prints.
struct SlowdownResult
{
    std::vector<float> speeds;
    double duration = -1.;
    // Compare owned process values exactly across replayed and fresh plans.
    bool operator==(const SlowdownResult &other) const { return speeds == other.speeds && duration == other.duration; }
};

// Describe copied slowdown results in Catch diagnostics without plan access.
std::ostream &operator<<(std::ostream &os, const SlowdownResult &result)
{
    os << "t " << result.duration << " speeds";
    for (float speed : result.speeds)
        os << " " << speed;
    return os;
}

// Copy ordered speeds and the optional whole-layer duration for replay checks.
SlowdownResult capture_slowdown(PreparedSpeedPrint &prepared)
{
    SlowdownResult out;
    out.speeds = leaf_speeds(prepared);
    if (const PrintingLayerTimeProperty *time = layer_time(prepared))
        out.duration = time->duration_seconds;
    return out;
}

// Three 100 mm/s leaves of 18 mm each: about 0.55 s per layer before slowdown.
std::vector<std::vector<ExtrusionSpec>> short_layer_specs()
{
    return {{slowdown_spec(ExtrusionRole::Perimeter), slowdown_spec(ExtrusionRole::InternalInfill),
             slowdown_spec(ExtrusionRole::SolidInfill)}};
}

// Append a second visit (tool group) of `extruder_id` to the layer, holding a
// copy of the first extrusion; returns the new tool group.
PrintingToolGroup &append_tool_visit(PrintingLayerGroup &layer, uint32_t extruder_id)
{
    layer.tool_groups.emplace_back();
    layer.tool_groups.back().extruder_id = extruder_id;
    PrintingExtrusion copy;
    copy.root.reset(layer.tool_groups.front().extrusions.front().root->clone());
    layer.tool_groups.back().extrusions.emplace_back(std::move(copy));
    return layer.tool_groups.back();
}
} // namespace

TEST_CASE("Slowdown stretches a short layer to its goal and publishes the resolved duration",
          "[plugins][layer-extrusion-edit][slowdown][printing-plan]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedSpeedPrint prepared;
    prepare_ordered_plan(prepared, slowdown_config(), short_layer_specs());
    REQUIRE(estimate_printed_layer(prepared) < 1.);
    REQUIRE(layer_time(prepared) == nullptr);

    run_editors(prepared, {SLOWDOWN_PLUGIN});

    const PrintingLayerTimeProperty *time = layer_time(prepared);
    REQUIRE(time != nullptr);
    CHECK(time->is_resolved());
    // The 10 mm/s floor would allow about 5.4 s, so the goal is reachable and reached.
    CHECK(time->duration_seconds == Approx(5.0).epsilon(1e-5));
    // The published value is what the time model gives on the edited plan.
    CHECK(estimate_printed_layer(prepared) == Approx(time->duration_seconds).epsilon(1e-6));
    // Equal leaves are slowed together to one common speed, within the bounds.
    const std::vector<float> speeds = leaf_speeds(prepared);
    REQUIRE(speeds.size() == 3);
    CHECK(speeds[0] == Approx(speeds[1]));
    CHECK(speeds[0] == Approx(speeds[2]));
    CHECK(speeds[0] < 100.f);
    CHECK(speeds[0] >= 10.f);
}

TEST_CASE("Slowdown keeps an interleaved deposit fixed and pairs both geometric leaves",
          "[plugins][layer-extrusion-edit][slowdown][deposit][time-estimator]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = slowdown_config("10");
    config.set_deserialize_strict("min_print_speed", "1");
    config.set_deserialize_strict("max_speed_reduction", "99");
    PreparedSpeedPrint prepared;
    prepare_ordered_plan(prepared, config, {{slowdown_spec(ExtrusionRole::Perimeter, 20.f),
        slowdown_spec(ExtrusionRole::SupportMaterial, 40.f), slowdown_spec(ExtrusionRole::InternalInfill)}});
    PrintingLayerGroup &layer = *printed_layers(prepared).front();
    ExtrusionEntity &root = *layer.tool_groups.front().extrusions.front().root;
    REQUIRE(root.child_count() == 3);
    ExtrusionEntity &deposit = root.child(1);
    const Point center = deposit.first_point();
    deposit.polyline_ref() = ArcPolyline();
    deposit.polyline_ref().append(center);
    deposit.add_property(ExtrusionPropertySpecialCommand(ExtrusionPropertySpecialCommand::Code::DEPOSIT, 0.5));
    const double flow = deposit.get_property<ExtrusionAttributes>()->mm3_per_mm;
    REQUIRE(flow > 0.0);
    const double fixed_seconds = 0.5 / (40.0 * flow);
    const slic3r_api::ConstPrintingPlan plan_view(
        reinterpret_cast<const printing_plan_handle *>(prepared.print.printing_plan()));
    const slic3r_api::Config config_view(Slic3r::ApiHost::to_config_handle(&prepared.print.config()));
    const slic3r_api::PrintingPlanTimeEstimator estimator(config_view);
    const std::vector<slic3r_api::PrintingLayerTimeDetail> before = estimator.estimate_detailed(plan_view);
    const slic3r_api::PrintingLayerTimeDetail *measured = nullptr;
    for (const slic3r_api::PrintingLayerTimeDetail &detail : before)
        if (!detail.leaves.empty()) {
            REQUIRE(measured == nullptr);
            measured = &detail;
        }
    REQUIRE(measured != nullptr);
    REQUIRE(measured->leaves.size() == 2);
    CHECK(measured->leaves[0].leaf_rank == 0);
    CHECK(measured->leaves[0].speed_mm_per_s == 20.f);
    CHECK(measured->leaves[1].leaf_rank == 1);
    CHECK(measured->leaves[1].speed_mm_per_s == 100.f);
    CHECK(measured->unattributed_seconds >= fixed_seconds);

    run_editors(prepared, {SLOWDOWN_PLUGIN});

    REQUIRE(root.child_count() == 3);
    CHECK(deposit.polyline_ref().size() == 1);
    CHECK(deposit.first_point() == center);
    CHECK(deposit.get_property<ExtrusionPropertySpecialCommand>()->extra_data == 0.5);
    CHECK(deposit.get_property<ExtrusionPropertySpeed>()->speed_mm_per_s == 40.f);
    CHECK(deposit.get_property<ExtrusionAttributes>()->mm3_per_mm == flow);
    CHECK(root.child(0).get_property<ExtrusionPropertySpeed>()->speed_mm_per_s < 20.f);
    CHECK(root.child(2).get_property<ExtrusionPropertySpeed>()->speed_mm_per_s < 100.f);
    const PrintingLayerTimeProperty *resolved = layer_time(prepared);
    REQUIRE(resolved != nullptr);
    CHECK(resolved->is_resolved());
    CHECK(resolved->duration_seconds == Approx(10.0).epsilon(1e-5));
    CHECK(estimate_printed_layer(prepared) == Approx(resolved->duration_seconds).epsilon(1e-6));
    const std::vector<slic3r_api::PrintingLayerTimeDetail> after = estimator.estimate_detailed(plan_view);
    REQUIRE(after.size() == before.size());
    for (size_t idx = 0; idx < before.size(); ++idx) {
        CHECK(after[idx].unattributed_seconds == before[idx].unattributed_seconds);
        CHECK(after[idx].leaves.size() == before[idx].leaves.size());
    }
}

TEST_CASE("Slowdown keeps the speed of non-slowable roles and counts their time",
          "[plugins][layer-extrusion-edit][slowdown][printing-plan]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedSpeedPrint prepared;
    // Unreachable goal: every slowable leaf goes to the 10 mm/s floor, the others stay at 100.
    prepare_ordered_plan(prepared, slowdown_config("1000"), {{
        slowdown_spec(ExtrusionRole::Perimeter), slowdown_spec(ExtrusionRole::OverhangPerimeter),
        slowdown_spec(ExtrusionRole::BridgeInfill), slowdown_spec(ExtrusionRole::InternalBridgeInfill),
        slowdown_spec(ExtrusionRole::Travel), slowdown_spec(ExtrusionRole::None),
        slowdown_spec(ExtrusionRole::WipeTower), slowdown_spec(ExtrusionRole::Milling)}});

    run_editors(prepared, {SLOWDOWN_PLUGIN});

    const std::map<uint16_t, ObservedLeaf> by_role = observed_by_role(observed_plan_leaves(prepared.print));
    for (const ExtrusionRole role : {ExtrusionRole::Perimeter, ExtrusionRole::InternalBridgeInfill}) {
        INFO(role());
        REQUIRE(by_role.count(uint16_t(role())) == 1);
        CHECK(by_role.at(uint16_t(role())).speed == Approx(10.f).margin(1e-3));
    }
    for (const ExtrusionRole role : {ExtrusionRole::OverhangPerimeter, ExtrusionRole::BridgeInfill, ExtrusionRole::Travel,
                                     ExtrusionRole::None, ExtrusionRole::WipeTower, ExtrusionRole::Milling}) {
        INFO(role());
        REQUIRE(by_role.count(uint16_t(role())) == 1);
        CHECK(by_role.at(uint16_t(role())).speed == 100.f);
    }
    // The fixed leaves and the synthetic travels stay in the published total.
    const PrintingLayerTimeProperty *time = layer_time(prepared);
    REQUIRE(time != nullptr);
    CHECK(time->duration_seconds == Approx(estimate_printed_layer(prepared)).epsilon(1e-6));
    CHECK(time->duration_seconds > 2 * 18. / 10. + 6 * 18. / 100.);
}

TEST_CASE("Slowdown leaves paths already under the minimum speed untouched",
          "[plugins][layer-extrusion-edit][slowdown][printing-plan]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedSpeedPrint prepared;
    prepare_ordered_plan(prepared, slowdown_config("1000"), {{
        slowdown_spec(ExtrusionRole::Perimeter, 8.f), slowdown_spec(ExtrusionRole::InternalInfill)}});

    run_editors(prepared, {SLOWDOWN_PLUGIN});

    const std::vector<float> speeds = leaf_speeds(prepared);
    REQUIRE(speeds.size() == 2);
    CHECK(speeds[0] == Approx(8.f).margin(1e-6));
    CHECK(speeds[1] == Approx(10.f).margin(1e-3));
}

TEST_CASE("Slowdown does nothing and publishes nothing when every threshold is zero",
          "[plugins][layer-extrusion-edit][slowdown][printing-plan]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedSpeedPrint prepared;
    prepare_ordered_plan(prepared, slowdown_config("0"), short_layer_specs());

    run_editors(prepared, {SLOWDOWN_PLUGIN});

    CHECK(layer_time(prepared) == nullptr);
    for (float speed : leaf_speeds(prepared))
        CHECK(speed == 100.f);
}

TEST_CASE("Slowdown keeps acceleration and pressure advance and writes a direct speed on inheriting leaves",
          "[plugins][layer-extrusion-edit][slowdown][printing-plan]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedSpeedPrint prepared;
    ExtrusionSpec own = slowdown_spec(ExtrusionRole::Perimeter);
    own.existing_acceleration = 1000.f;
    own.pressure_advance = 0.05f;
    // The second leaf has no process property: it inherits it from the root.
    prepare_ordered_plan(prepared, slowdown_config(), {{own, ExtrusionSpec{ExtrusionRole::InternalInfill, 0.2}}});
    PrintingLayerGroup &layer = *printed_layers(prepared).front();
    REQUIRE(layer.tool_groups.size() == 1);
    REQUIRE(layer.tool_groups.front().extrusions.size() == 1);
    slic3r_api::MutableExtrusionEntity root(
        reinterpret_cast<extrusion_entity_handle *>(layer.tool_groups.front().extrusions.front().root.get()));
    root.get_or_add(slic3r_api::EPropertySpeed::key).speed(100.f).acceleration(800.f);

    run_editors(prepared, {SLOWDOWN_PLUGIN});

    const std::vector<ObservedLeaf> leaves = observed_plan_leaves(prepared.print);
    REQUIRE(leaves.size() == 2);
    for (const ObservedLeaf &leaf : leaves)
        CHECK(leaf.speed < 100.f);
    // Own property: only the speed changed.
    REQUIRE(leaves[0].direct_process != nullptr);
    CHECK(leaves[0].acceleration == 1000.f);
    CHECK(leaves[0].direct_process->pressure_adv == Approx(0.05f));
    // Inheriting leaf: a new direct property with the other fields unset.
    REQUIRE(leaves[1].direct_process != nullptr);
    CHECK(leaves[1].direct_process->accel_mm_per_s2 < 0.f);
    CHECK(leaves[1].direct_process->pressure_adv < 0.f);
    CHECK(leaves[1].acceleration == 800.f);
    // The root keeps what it had: only leaves are edited.
    const slic3r_api::EPropertySpeed *root_speed = root.get(slic3r_api::EPropertySpeed::key);
    REQUIRE(root_speed != nullptr);
    CHECK(root_speed->speed_mm_per_s == 100.f);
}

TEST_CASE("Slowdown handles a multi-point path like a two-point path of the same length",
          "[plugins][layer-extrusion-edit][slowdown][printing-plan]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    // A path is one leaf (one legacy block): both plans must get the same speed.
    SlowdownResult results[2];
    for (const bool middle_point : {false, true}) {
        PreparedSpeedPrint prepared;
        ExtrusionSpec spec = slowdown_spec(ExtrusionRole::Perimeter);
        spec.middle_point = middle_point;
        prepare_ordered_plan(prepared, slowdown_config("1"), {{spec}});
        run_editors(prepared, {SLOWDOWN_PLUGIN});
        results[middle_point ? 1 : 0] = capture_slowdown(prepared);
        REQUIRE(results[middle_point ? 1 : 0].speeds.size() == 1);
    }
    CHECK(results[0].speeds[0] < 100.f);
    CHECK(results[0].speeds[0] == Approx(results[1].speeds[0]));
    CHECK(results[0].duration == Approx(results[1].duration));
    // 18 mm in 1 s.
    CHECK(results[1].speeds[0] == Approx(18.f).epsilon(1e-4));
}

TEST_CASE("Slowdown shares the goal between separate visits of the same extruder",
          "[plugins][layer-extrusion-edit][slowdown][printing-plan]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedSpeedPrint prepared;
    DynamicPrintConfig config = slowdown_config();
    config.set_deserialize_strict("min_print_speed", "1");
    config.set_deserialize_strict("max_speed_reduction", "99");
    prepare_ordered_plan(prepared, config, {{slowdown_spec(ExtrusionRole::Perimeter)}});
    append_tool_visit(*printed_layers(prepared).front(), 0);

    run_editors(prepared, {SLOWDOWN_PLUGIN});

    const std::vector<float> speeds = leaf_speeds(prepared);
    REQUIRE(speeds.size() == 2);
    CHECK(speeds[0] == Approx(speeds[1]));
    CHECK(speeds[0] < 100.f);
    const PrintingLayerTimeProperty *time = layer_time(prepared);
    REQUIRE(time != nullptr);
    CHECK(time->duration_seconds == Approx(5.0).epsilon(1e-5));
    CHECK(estimate_printed_layer(prepared) == Approx(5.0).epsilon(1e-5));
}

TEST_CASE("Slowdown with a disabled extruder keeps its leaves and counts their time",
          "[plugins][layer-extrusion-edit][slowdown][printing-plan]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedSpeedPrint prepared;
    // Extruder 0 has a 5 s goal, extruder 1 none.
    DynamicPrintConfig config = slowdown_config("5,0");
    config.set_deserialize_strict("nozzle_diameter", "0.4,0.4");
    config.set_deserialize_strict("min_print_speed", "1,1");
    config.set_deserialize_strict("max_speed_reduction", "99,99");
    prepare_ordered_plan(prepared, config, {{slowdown_spec(ExtrusionRole::Perimeter)}});
    append_tool_visit(*printed_layers(prepared).front(), 1);

    run_editors(prepared, {SLOWDOWN_PLUGIN});

    const std::vector<float> speeds = leaf_speeds(prepared);
    REQUIRE(speeds.size() == 2);
    CHECK(speeds[0] < 100.f);
    CHECK(speeds[1] == 100.f);
    const PrintingLayerTimeProperty *time = layer_time(prepared);
    REQUIRE(time != nullptr);
    CHECK(time->is_resolved());
    CHECK(time->duration_seconds == Approx(5.0).epsilon(1e-5));
}

TEST_CASE("Slowdown handles an extruder with a goal that prints nothing in the layer",
          "[plugins][layer-extrusion-edit][slowdown][printing-plan]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedSpeedPrint prepared;
    // Extruder 1 has a higher goal but is absent from the layer: only extruder 0 counts.
    DynamicPrintConfig config = slowdown_config("5,50");
    config.set_deserialize_strict("min_print_speed", "1");
    config.set_deserialize_strict("max_speed_reduction", "99");
    prepare_ordered_plan(prepared, config, {{slowdown_spec(ExtrusionRole::Perimeter)}});

    run_editors(prepared, {SLOWDOWN_PLUGIN});

    const PrintingLayerTimeProperty *time = layer_time(prepared);
    REQUIRE(time != nullptr);
    CHECK(time->duration_seconds == Approx(5.0).epsilon(1e-5));
}

TEST_CASE("Fan cooling follows the layer duration after the slowdown",
          "[plugins][layer-extrusion-edit][slowdown][fan-policy][fan][printing-plan]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedSpeedPrint prepared;
    // Extruder 0 slows layers under 3 s and cools under 10 s; extruder 1 wants 10 s.
    // The layer ends at 10 s because of extruder 1, so extruder 0 cools at its
    // minimum fan speed; the estimate before the slowdown (0.5 s) would give the maximum.
    DynamicPrintConfig config = slowdown_config("3,10");
    config.set_deserialize_strict("nozzle_diameter", "0.4,0.4");
    config.set_deserialize_strict("min_print_speed", "1,1");
    config.set_deserialize_strict("max_speed_reduction", "99,99");
    config.set_deserialize_strict("default_fan_speed", "20");
    config.set_deserialize_strict("perimeter_fan_speed", "-1");
    config.set_deserialize_strict("max_fan_speed", "100");
    config.set_deserialize_strict("fan_below_layer_time", "10");
    config.set_deserialize_strict("disable_fan_first_layers", "0");
    config.set_deserialize_strict("full_fan_speed_layer", "0");
    prepare_ordered_plan(prepared, config, {{slowdown_spec(ExtrusionRole::Perimeter)}});
    append_tool_visit(*printed_layers(prepared).front(), 1);
    const double before = estimate_printed_layer(prepared);

    run_editors(prepared, {SLOWDOWN_PLUGIN, DYNAMIC_FAN_PLUGIN, FIRST_LAYER_FAN_PLUGIN, FAN_COOLING_PLUGIN,
                           DEFAULT_FAN_PLUGIN});

    const PrintingLayerTimeProperty *time = layer_time(prepared);
    REQUIRE(time != nullptr);
    CHECK(time->is_resolved());
    CHECK(time->duration_seconds == Approx(10.0).epsilon(1e-5));

    const slic3r_api::Config config_view(Slic3r::ApiHost::to_config_handle(&prepared.print.config()));
    const slic3r_api::FanPolicy::Settings settings = slic3r_api::FanPolicy::read_settings(config_view, 0);
    const auto expected = [&](double seconds) {
        return slic3r_api::FanPolicy::close_first_layers(
            slic3r_api::FanPolicy::apply_layer_time(slic3r_api::FanPolicy::make_base(settings), settings, seconds),
            slic3r_api::FanPolicy::first_layer_program(settings, 0));
    };
    const int after_percent = expected(time->duration_seconds).roles[RAW_FAN_ROLE_PERIMETER].percent;
    const int estimated_percent = expected(before).roles[RAW_FAN_ROLE_PERIMETER].percent;
    REQUIRE(after_percent != estimated_percent);

    const PluginPropertyKey<slic3r_api::FanPolicy::Context> context_key = slic3r_api::FanPolicy::context_property_key(
        reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
    PrintingToolGroup &tool = printed_layers(prepared).front()->tool_groups.front();
    const slic3r_api::PrintingToolGroup tool_view(reinterpret_cast<printing_tool_group_handle *>(&tool));
    const slic3r_api::FanPolicy::Context *context = context_key.get(tool_view.properties());
    REQUIRE(context != nullptr);
    CHECK(context->roles[RAW_FAN_ROLE_PERIMETER].percent == after_percent);
}

TEST_CASE("Fan cooling keeps resolved layers and estimates the layers without a resolved duration",
          "[plugins][layer-extrusion-edit][slowdown][fan-policy][fan][printing-plan]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedSpeedPrint prepared;
    // Extruder 0 has a goal, extruder 1 none; the second layer is printed by extruder 1 only.
    DynamicPrintConfig config = slowdown_config("5,0");
    config.set_deserialize_strict("nozzle_diameter", "0.4,0.4");
    config.set_deserialize_strict("min_print_speed", "1,1");
    config.set_deserialize_strict("max_speed_reduction", "99,99");
    config.set_deserialize_strict("fan_below_layer_time", "10");
    prepare_ordered_plan(prepared, config, {{slowdown_spec(ExtrusionRole::Perimeter)},
                                            {slowdown_spec(ExtrusionRole::Perimeter)}});
    REQUIRE(printed_layers(prepared).size() == 2);
    printed_layers(prepared).back()->tool_groups.front().extruder_id = 1;

    run_editors(prepared, {SLOWDOWN_PLUGIN, DYNAMIC_FAN_PLUGIN, FIRST_LAYER_FAN_PLUGIN, FAN_COOLING_PLUGIN,
                           DEFAULT_FAN_PLUGIN});

    const PrintingLayerTimeProperty *first = layer_time(prepared, 0);
    REQUIRE(first != nullptr);
    CHECK(first->is_resolved());
    CHECK(first->duration_seconds == Approx(5.0).epsilon(1e-5));
    const PrintingLayerTimeProperty *second = layer_time(prepared, 1);
    REQUIRE(second != nullptr);
    CHECK_FALSE(second->is_resolved());
    CHECK(second->duration_seconds == Approx(estimate_printed_layer(prepared, 1)).epsilon(1e-6));
}

TEST_CASE("Slowdown settings changes on the same Print match a fresh Print and the initial result returns",
          "[plugins][layer-extrusion-edit][slowdown][printing-plan]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    // Each change is cumulative and must visibly change the result: the floor
    // (30 mm/s, goal no longer reachable), the reduction cap (50 %), then no slowdown.
    const std::vector<std::pair<const char *, const char *>> changes = {
        {"min_print_speed", "30"}, {"max_speed_reduction", "50"}, {"slowdown_below_layer_time", "0"}};
    std::vector<DynamicPrintConfig> configs{slowdown_config()};
    for (const auto &[key, value] : changes) {
        configs.push_back(configs.back());
        configs.back().set_deserialize_strict(key, value);
    }

    PreparedSpeedPrint reused;
    prepare_ordered_plan(reused, configs.front(), short_layer_specs());
    for (slicing_step_t step : Steps::execution_order()) {
        reused.print.mark_step_executed(step);
        if (step == STEP_ORDERING)
            break;
    }
    REQUIRE(reused.print.save_ordering_checkpoint());
    const auto replay = [&](const DynamicPrintConfig &config, Print::ApplyStatus expected) {
        REQUIRE(reused.print.apply(reused.model, config) == expected);
        if (expected == Print::APPLY_STATUS_INVALIDATED)
            reused.print.prepare_checkpoint_resume();
        run_editors(reused, {SLOWDOWN_PLUGIN});
        reused.print.mark_step_executed(STEP_LAYER_EXTRUSION_EDIT);
        return capture_slowdown(reused);
    };
    const auto fresh_result = [](const DynamicPrintConfig &config) {
        PreparedSpeedPrint fresh;
        prepare_ordered_plan(fresh, config, short_layer_specs());
        REQUIRE(fresh.print.apply(fresh.model, config) == Print::APPLY_STATUS_UNCHANGED);
        run_editors(fresh, {SLOWDOWN_PLUGIN});
        return capture_slowdown(fresh);
    };

    SlowdownResult initial;
    SlowdownResult previous;
    for (size_t idx = 0; idx < configs.size(); ++idx) {
        INFO(idx);
        const SlowdownResult resumed = replay(configs[idx], idx == 0 ? Print::APPLY_STATUS_UNCHANGED : Print::APPLY_STATUS_INVALIDATED);
        REQUIRE(resumed.speeds.size() == 3);
        CHECK(resumed == fresh_result(configs[idx]));
        if (idx == 0)
            initial = resumed;
        else
            CHECK_FALSE(resumed == previous);
        previous = resumed;
    }
    // The last change disables the slowdown: original speeds, nothing published.
    CHECK(previous.speeds == std::vector<float>(3, 100.f));
    CHECK(previous.duration < 0.);
    // Restoring every setting gives back the initial result.
    CHECK(replay(configs.front(), Print::APPLY_STATUS_INVALIDATED) == initial);

    // The three settings are read by the plugin, so each invalidates the edit step.
    for (const char *key : {"min_print_speed", "max_speed_reduction", "slowdown_below_layer_time"}) {
        INFO(key);
        const ConfigOptionDef *def = reused.print.config().def()->get(key);
        REQUIRE(def != nullptr);
        CHECK(def->invalidates_step == STEP_LAYER_EXTRUSION_EDIT);
    }
}

TEST_CASE("Detailed time estimate splits a layer into leaves and unattributed time that sum to the total",
          "[plugins][layer-extrusion-edit][slowdown][printing-plan]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedSpeedPrint prepared;
    prepare_ordered_plan(prepared, slowdown_config(), short_layer_specs());
    append_tool_visit(*printed_layers(prepared).front(), 0);
    PrintingPlan &plan = prepared.print.mutable_printing_plan();
    const slic3r_api::PrintingPlan plan_view(reinterpret_cast<printing_plan_handle *>(&plan));
    const slic3r_api::Config config_view(Slic3r::ApiHost::to_config_handle(&prepared.print.config()));
    const slic3r_api::PrintingPlanTimeEstimator estimator(config_view);
    const std::vector<slic3r_api::PrintingLayerTimeEstimate> totals = estimator.estimate(plan_view);
    const std::vector<slic3r_api::PrintingLayerTimeDetail> details = estimator.estimate_detailed(plan_view);
    REQUIRE(totals.size() == details.size());
    size_t checked = 0;
    for (size_t idx = 0; idx < totals.size(); ++idx) {
        double sum = details[idx].unattributed_seconds;
        for (const slic3r_api::PrintingLeafTime &leaf : details[idx].leaves)
            sum += leaf.duration_seconds;
        CHECK(details[idx].group_idx == totals[idx].group_idx);
        CHECK(details[idx].layer_group_idx == totals[idx].layer_group_idx);
        CHECK(sum == Approx(totals[idx].duration_seconds).epsilon(1e-9));
        checked += details[idx].leaves.size();
    }
    // 3 leaves in the first visit, 3 in the copied visit, ranked per extrusion.
    CHECK(checked == 6);
    for (const slic3r_api::PrintingLayerTimeDetail &detail : details)
        if (!detail.leaves.empty()) {
            CHECK(detail.unattributed_seconds > 0.);
            CHECK(detail.leaves.back().tool_idx == 1);
            CHECK(detail.leaves.back().leaf_rank == 2);
        }
}

namespace {
// Collect, in tree order, the entities that own segments and no child.
void collect_leaf_handles(slic3r_api::MutableExtrusionEntity entity, std::vector<slic3r_api::MutableExtrusionEntity> &out)
{
    for (uint32_t idx = 0; idx < entity.child_count(); ++idx)
        collect_leaf_handles(entity.child_mutable(idx), out);
    if (entity.child_count() == 0 && entity.segment_count() > 0)
        out.push_back(entity);
}

// All the leaves of the printed layer, in plan order.
std::vector<slic3r_api::MutableExtrusionEntity> printed_leaf_handles(PreparedSpeedPrint &prepared)
{
    std::vector<slic3r_api::MutableExtrusionEntity> leaves;
    for (PrintingToolGroup &tool : printed_layers(prepared).front()->tool_groups)
        for (PrintingExtrusion &extrusion : tool.extrusions)
            collect_leaf_handles(slic3r_api::MutableExtrusionEntity(
                reinterpret_cast<extrusion_entity_handle *>(extrusion.root.get())), leaves);
    return leaves;
}

// A straight segment between two points given in mm, with Z offsets in mm.
c_extrusion_segment straight_segment(double x0, double y0, double x1, double y1, double z0 = 0., double z1 = 0.)
{
    c_extrusion_segment segment = {};
    segment.point_a = {scale_i(x0), scale_i(y0)};
    segment.point_b = {scale_i(x1), scale_i(y1)};
    segment.z_offset_a = scale_i(z0);
    segment.z_offset_b = scale_i(z1);
    return segment;
}

// The three connected segments used by the multi-segment test: a line, a 60 degree
// arc (chord 3 mm, radius 3 mm) and a line that rises by 0.3 mm. About 9.16 mm.
std::array<c_extrusion_segment, 3> line_arc_rise_segments()
{
    std::array<c_extrusion_segment, 3> segments = {
        straight_segment(11., 12., 14., 12.), straight_segment(14., 12., 17., 12.),
        straight_segment(17., 12., 20., 12., 0., 0.3)};
    segments[1].radius = float(scale_i(3.)); // radii are scaled coordinates
    segments[1].orientation = RAW_EXTRUSION_ARC_ORIENTATION_CCW;
    return segments;
}
} // namespace

TEST_CASE("Slowdown gives a multi-segment path with an arc and a Z change the speed of the equivalent paths",
          "[plugins][layer-extrusion-edit][slowdown][printing-plan]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const std::array<c_extrusion_segment, 3> segments = line_arc_rise_segments();
    SlowdownResult results[2];
    for (const bool split : {false, true}) {
        PreparedSpeedPrint prepared;
        std::vector<ExtrusionSpec> specs(split ? 3 : 1, slowdown_spec(ExtrusionRole::Perimeter));
        // Goal 0.8 s: the 9.16 mm path lasts 0.092 s at 100 mm/s and 0.916 s at the 10 mm/s floor.
        prepare_ordered_plan(prepared, slowdown_config("0.8"), {specs});
        std::vector<slic3r_api::MutableExtrusionEntity> leaves = printed_leaf_handles(prepared);
        REQUIRE(leaves.size() == specs.size());
        if (split)
            for (size_t idx = 0; idx < 3; ++idx)
                REQUIRE(leaves[idx].set_segments(&segments[idx], 1));
        else
            REQUIRE(leaves.front().set_segments(segments.data(), 3));
        run_editors(prepared, {SLOWDOWN_PLUGIN});
        results[split ? 1 : 0] = capture_slowdown(prepared);
        const PrintingLayerTimeProperty *time = layer_time(prepared);
        REQUIRE(time != nullptr);
        CHECK(time->duration_seconds == Approx(0.8).epsilon(1e-5));
        CHECK(estimate_printed_layer(prepared) == Approx(time->duration_seconds).epsilon(1e-6));
    }
    // One path is one block: it gets the speed that three equal-speed pieces get.
    REQUIRE(results[0].speeds.size() == 1);
    REQUIRE(results[1].speeds.size() == 3);
    for (float speed : results[1].speeds)
        CHECK(speed == Approx(results[0].speeds[0]));
    CHECK(results[0].speeds[0] == Approx(100.f * 0.0916f / 0.8f).epsilon(2e-2));
    CHECK(results[0].duration == Approx(results[1].duration));
}

TEST_CASE("Slowdown keeps leaves of non-eligible roles, including roles outside its table",
          "[plugins][layer-extrusion-edit][slowdown][printing-plan]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedSpeedPrint prepared;
    const auto raw_role = [](uint32_t raw) { return ExtrusionRole(static_cast<ExtrusionRoleModifier>(raw)); };
    const ExtrusionRole overhang_external = raw_role(RAW_EXTRUSION_ROLE_OVERHANG_EXTERNAL_PERIMETER);
    const ExtrusionRole wipe = raw_role(RAW_EXTRUSION_ROLE_TRAVEL | RAW_EXTRUSION_ROLE_WIPE);
    // A perimeter that also carries the "solid" modifier is not in the table.
    const ExtrusionRole modified = raw_role(RAW_EXTRUSION_ROLE_PERIMETER | RAW_EXTRUSION_ROLE_SOLID);
    prepare_ordered_plan(prepared, slowdown_config("1000"), {{
        slowdown_spec(ExtrusionRole::Perimeter), slowdown_spec(overhang_external), slowdown_spec(wipe),
        slowdown_spec(ExtrusionRole::Travel), slowdown_spec(modified)}});

    run_editors(prepared, {SLOWDOWN_PLUGIN});

    const std::map<uint16_t, ObservedLeaf> by_role = observed_by_role(observed_plan_leaves(prepared.print));
    REQUIRE(by_role.size() == 5);
    CHECK(by_role.at(uint16_t(ExtrusionRole(ExtrusionRole::Perimeter)())).speed == Approx(10.f).margin(1e-3));
    for (const ExtrusionRole role : {overhang_external, wipe, ExtrusionRole(ExtrusionRole::Travel), modified}) {
        INFO(role());
        REQUIRE(by_role.count(uint16_t(role())) == 1);
        CHECK(by_role.at(uint16_t(role())).speed == Approx(100.f).margin(1e-6));
    }
}

TEST_CASE("Slowdown handles layers in two sequential printing groups, first layer included",
          "[plugins][layer-extrusion-edit][slowdown][printing-plan]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedSpeedPrint prepared;
    prepare_ordered_plan(prepared, slowdown_config(), {short_layer_specs().front(), short_layer_specs().front()});
    // Move the second printed layer into its own group, after the first one.
    PrintingPlan &plan = prepared.print.mutable_printing_plan();
    PrintingGroup &first = plan.groups.front();
    size_t second_idx = first.layers.size();
    size_t printed = 0;
    for (size_t idx = 0; idx < first.layers.size(); ++idx)
        if (!first.layers[idx].tool_groups.empty() && ++printed == 2)
            second_idx = idx;
    REQUIRE(second_idx < first.layers.size());
    PrintingGroup second;
    second.plan_identity = first.plan_identity;
    second.printing_catalog_ids = first.printing_catalog_ids;
    second.layers.push_back(std::move(first.layers[second_idx]));
    first.layers.erase(first.layers.begin() + std::ptrdiff_t(second_idx));
    plan.groups.push_back(std::move(second));
    REQUIRE(plan.groups.size() == 2);

    run_editors(prepared, {SLOWDOWN_PLUGIN});

    REQUIRE(printed_layers(prepared).size() == 2);
    for (size_t layer = 0; layer < 2; ++layer) {
        INFO(layer);
        const PrintingLayerTimeProperty *time = layer_time(prepared, layer);
        REQUIRE(time != nullptr);
        CHECK(time->is_resolved());
        CHECK(time->duration_seconds == Approx(5.0).epsilon(1e-5));
        // The layer entered from the other group is measured with the carried position.
        CHECK(estimate_printed_layer(prepared, layer) == Approx(time->duration_seconds).epsilon(1e-6));
    }
}

TEST_CASE("Slowdown reduction cap decides when it is tighter than the minimum speed, per extruder",
          "[plugins][layer-extrusion-edit][slowdown][printing-plan]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    // Unreachable goal, minimum speed 1 mm/s: the lowest speed is max(1, 100 * (1 - reduction)).
    struct Case { const char *reduction; float expected_speed; };
    for (const Case &c : {Case{"0,0", 1.f}, Case{"90,90", 10.f}, Case{"100,100", 1.f}}) {
        INFO(c.reduction);
        PreparedSpeedPrint prepared;
        DynamicPrintConfig config = slowdown_config("1000,1000");
        config.set_deserialize_strict("nozzle_diameter", "0.4,0.4");
        config.set_deserialize_strict("min_print_speed", "1,1");
        config.set_deserialize_strict("max_speed_reduction", c.reduction);
        prepare_ordered_plan(prepared, config, {{slowdown_spec(ExtrusionRole::Perimeter)}});
        append_tool_visit(*printed_layers(prepared).front(), 1);
        // The Print must hold two values for each setting, so both tools read them.
        const PrintConfig &print_config = prepared.print.config();
        REQUIRE(print_config.max_speed_reduction.size() == 2);
        REQUIRE(print_config.min_print_speed.size() == 2);
        REQUIRE(print_config.slowdown_below_layer_time.size() == 2);
        CHECK(print_config.max_speed_reduction.get_at(0) == print_config.max_speed_reduction.get_at(1));

        run_editors(prepared, {SLOWDOWN_PLUGIN});

        const std::vector<float> speeds = leaf_speeds(prepared);
        REQUIRE(speeds.size() == 2);
        CHECK(speeds[0] == Approx(c.expected_speed).margin(1e-3));
        CHECK(speeds[1] == Approx(c.expected_speed).margin(1e-3));
    }
}
