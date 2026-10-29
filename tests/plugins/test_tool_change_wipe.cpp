/*
The production wipe service is exercised through borrowed phase payloads and
through the whole export pipeline. Compact owned plans isolate visit boundaries,
volume fallbacks, rounded-flow rows and speeds; sliced prints check tool commands,
platter positions and tower Z offsets without replacing downstream plugins.
Reader-derived statistics are checked against role/tool volumes and an
explicit post-export CLI read, preserving source-layer areas and metadata.
Overlapping model parts exercise typed surfaces and closed overhang fragments;
materialized Travel connectors into synthetic towers need machine travel speed.
Pressure-advance command checks use short 20 mm-wide boxes: they retain the first
and normal-layer tool changes without generating a tall object for each option.

Representative direct calls (branches are not a timeline):

    TEST_CASE
        |-- Test::Plugins::ensure_plugin_test_runtime_initialized()
        |-- configure_print()
        |-- host.prepare_plugin_host_context()
        |-- host.prepare_plugin_run_context()
        |-- run_service()
        `-- check_leaf()
*/
#include <catch2/catch.hpp>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>
#include <tbb/global_control.h>
#include <boost/scope_exit.hpp>
#include "gcode_test_helpers.hpp"
#include "plugin_test_helpers.hpp"
#include "test_data.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_wipe_tower_wipe.h"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/PrintRegion.hpp"
#include "libslic3r/Plugins/WipeTower/WipeTowerLayout.hpp"
#include "libslic3r/Steps/StepGenerateWipeTower.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"
#include "libslic3r/Steps/StepReadGeneratedGcode.hpp"
#include "libslic3r/Utils.hpp"

namespace {
using namespace Slic3r;
using namespace slic3r_api::WipeTower;
const char *const plugin_id = "wipe_tower.wipe.toolchange.default";

// Initialize an owned print with consistent tool vectors and a small model, then clear its plan.
void configure_print(Print &print, Model &model, size_t tools = 2);
// Append Print-owned source height and optionally plan-owned source paths to concrete visits.
void append_layer(Print &print, slic3r_api::PrintingGroup group, double z,
                  const std::vector<extruder_index_t> &tools, bool paths = false);
// Copy one reported value into the caller's request array.
int32_t report_request(void *context, const c_wipe_tower_request *request);
// Invoke the registered production service and require no hidden callback error.
void run_service(Print &print, run_ctx_wipe_tower_wipe &payload);
// Collect owned request copies from the production service.
std::vector<c_wipe_tower_request> collect(Print &print);
// Sum the volume of the leaves of a tree whose stored attribute role equals raw_role.
double raw_role_volume(const ExtrusionEntity &entity, uint16_t raw_role);
// Extend the platter Y extent of a tower tree's leaves, and separately the maximum Y of its
// purge and ramming leaves, in millimeters.
void tower_leaf_y_extent(const ExtrusionEntity &entity, double &min_y, double &max_y, double &purge_max);
// Find the first leaf whose stored attribute role equals raw_role, with the Z offset in effect on it.
const ExtrusionEntity *first_leaf_with_raw_role(const ExtrusionEntity &entity, uint16_t raw_role,
                                                coord_t inherited_offset, coord_t &z_offset);
// Find the last leaf whose stored attribute role equals raw_role, or null when none matches.
const ExtrusionEntity *last_leaf_with_raw_role(const ExtrusionEntity &entity, uint16_t raw_role);
// Collect the geometric EXTRUSION leaves of a tree in order (point-less ones are not travelling).
std::vector<const ExtrusionEntity *> travelling_filament_leaves(const ExtrusionEntity &entity);
// Collect every leaf holding a polyline in tree order, whatever its role.
std::vector<const ExtrusionEntity *> geometric_leaves(const ExtrusionEntity &entity);
// Sum the EXTRUSION distances of a tree, point-less commands included.
double filament_delta_sum(const ExtrusionEntity &entity);
// Expected filament E of one plan tree: flow leaves convert their volume with e_per_mm3 and
// fixed EXTRUSION commands carry their own signed length. Explicit retraction operations are
// expected to net to their restart extras, so they are not added here.
double plan_net_e(const ExtrusionEntity &entity, double e_per_mm3);
// Sum the explicit tool-change retraction requests of one plan tree.
double plan_toolchange_retraction(const ExtrusionEntity &entity);
// Verify effective speeds on geometric leaves and count resolved tower travels.
void require_movement_speeds(const ExtrusionEntity &entity, float inherited_speed, bool tower,
                             size_t &tower_travels, float travel_speed);
// Leaf speeds of one tree, classified by role: purge, ramming, tower maintenance and other filament.
struct TowerSpeeds
{
    std::vector<double> purge;
    std::vector<double> ramming;
    std::vector<double> maintenance;
    std::vector<double> filament;
};
// Count heater entry events of a borrowed tree, on collections and leaves alike.
size_t temperature_request_count(const ExtrusionEntity &entity);
// Collect the leaf speeds of a borrowed tree into caller-owned buckets, in tree order.
void collect_tower_speeds(const ExtrusionEntity &entity, TowerSpeeds &speeds);

// Traverse a borrowed final plan tree through generic children, checking that
// inherited speed is positive and finite for each geometric leaf. Count tower
// Travel leaves and require machine travel speed there; retain no views or change
// properties, and report all mismatches through test assertions.
void require_movement_speeds(const ExtrusionEntity &entity, float inherited_speed, const bool tower,
                             size_t &tower_travels, const float travel_speed)
{
    if (const ExtrusionPropertySpeed *speed = entity.get_property<ExtrusionPropertySpeed>())
        if (speed->speed_mm_per_s > 0.f)
            inherited_speed = speed->speed_mm_per_s;
    for (size_t index = 0; index < entity.child_count(); ++index)
        require_movement_speeds(entity.child(index), inherited_speed, tower, tower_travels, travel_speed);
    if (!entity.has_polyline() || entity.polyline_or_null()->size() < 2)
        return;
    CHECK(inherited_speed > 0.f);
    CHECK(std::isfinite(inherited_speed));
    const ExtrusionAttributes *attributes = entity.get_property<ExtrusionAttributes>();
    if (tower && attributes != nullptr && attributes->extrusion_role() == ExtrusionRole::Travel) {
        ++tower_travels;
        CHECK(inherited_speed == Approx(travel_speed));
    }
}

// Count heater entry events of a borrowed tree, on collections and leaves alike.
size_t temperature_request_count(const ExtrusionEntity &entity)
{
    size_t count = 0;
    if (entity.get_property<ExtrusionPropertyHeater>() != nullptr)
        ++count;
    for (size_t index = 0; index < entity.child_count(); ++index)
        count += temperature_request_count(entity.child(index));
    return count;
}

// Classify every leaf carrying a positive speed by its role: a tower leaf with a real flow is
// maintenance, one without is a filament move of the SEMM unload or load. Retains no views.
void collect_tower_speeds(const ExtrusionEntity &entity, TowerSpeeds &speeds)
{
    const ExtrusionPropertySpeed *speed = entity.get_property<ExtrusionPropertySpeed>();
    if (speed != nullptr && speed->speed_mm_per_s > 0.f && entity.child_count() == 0) {
        if (entity.role() == ExtrusionRole::WipeTowerWipe)
            speeds.purge.push_back(double(speed->speed_mm_per_s));
        else if (entity.role() == ExtrusionRole::WipeTowerRamming)
            speeds.ramming.push_back(double(speed->speed_mm_per_s));
        else if (entity.role() == ExtrusionRole::WipeTower) {
            const ExtrusionAttributes *attributes = entity.get_property<ExtrusionAttributes>();
            if (attributes != nullptr && attributes->mm3_per_mm > 0.)
                speeds.maintenance.push_back(double(speed->speed_mm_per_s));
            else
                speeds.filament.push_back(double(speed->speed_mm_per_s));
        }
    }
    for (size_t index = 0; index < entity.child_count(); ++index)
        collect_tower_speeds(entity.child(index), speeds);
}

// Apply complete default configuration with explicit tower/tool vectors to caller-owned model/print, retaining no views; initialization failures remain test failures.
void configure_print(Print &print, Model &model, const size_t tools)
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_extruders(uint16_t(tools));
    config.set_deserialize_strict({{"wipe_tower", "1"}, {"wipe_tower_x", "40"}, {"wipe_tower_y", "60"},
        {"wipe_tower_width", "20"}, {"wipe_tower_extra_spacing", "150%"}, {"wipe_tower_speed", "80"},
        {"wipe_tower_wipe_starting_speed", "25%"}, {"step_wipetower_plugin", "wipe_tower.manager.default"}});
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
    Test::GCode::printing_plan_view(print).clear();
}

// Append a positive-Z layer with 0.2-mm auxiliary source owned by Print and concrete visits owned by the plan; optional paths have no source owner and API/allocation failures remain assertions.
void append_layer(Print &print, slic3r_api::PrintingGroup group, const double z,
                  const std::vector<extruder_index_t> &tools, const bool paths)
{
    slic3r_api::PrintingLayerGroup layer = group.append_layer_group(scale_i(z));
    PrintObject &object = print.mutable_auxiliary_object();
    Layer &source = object.add_auxiliary_layer(object.auxiliary_layer_count(), scale_i(0.2), scale_i(z));
    layer.append_layer(slic3r_api::Layer(reinterpret_cast<const layer_handle *>(&source)));
    for (const extruder_index_t tool : tools) {
        slic3r_api::PrintingToolGroup visit = layer.append_tool_group(tool);
        if (paths) {
            std::unique_ptr<ExtrusionPath> path = Test::GCode::make_firmware_path(
                ArcPolyline(Points{Point::new_scale(1., 1.), Point::new_scale(2., 1.)}), 20.f, 500.f);
            REQUIRE(printing_tool_group_append_extrusion_clone(Test::GCode::printing_plan_view(print).handle(), visit.mutable_handle(),
                nullptr, RAW_EXTRUSION_ROLE_PERIMETER, reinterpret_cast<const extrusion_entity_handle *>(path.get()), UINT32_MAX) != nullptr);
        }
    }
}

// Copy a nonnull borrowed request into caller-owned storage synchronously, return success and retain no pointers; test allocation failures propagate to the service's error bridge.
int32_t report_request(void *context, const c_wipe_tower_request *request)
{
    static_cast<std::vector<c_wipe_tower_request> *>(context)->push_back(*request);
    return 1;
}

// Lend caller-owned print/plan/payload to the registered plugin with invocation-local error tracking; callbacks retain no test pointers and any bridge error or cancellation fails the test.
void run_service(Print &print, run_ctx_wipe_tower_wipe &payload)
{
    Orchestrator &host = Orchestrator::instance();
    Plugin *plugin = host.get_plugin(plugin_id);
    REQUIRE(plugin != nullptr);
    host.reset_plugin_cancel();
    plugin_host_context context = host.prepare_plugin_host_context(WIPE_TOWER_WIPE, plugin, &print);
    std::atomic_bool error{false};
    context.execution_error = &error;
    plugin_run_context run = host.prepare_plugin_run_context(WIPE_TOWER_WIPE, plugin, &context);
    plugin->setup(run, 1);
    payload.print = reinterpret_cast<const print_handle *>(&print);
    payload.plan = reinterpret_cast<const printing_plan_handle *>(print.printing_plan());
    run.data = &payload;
    plugin->setup_run(run);
    plugin->run(run);
    REQUIRE_FALSE(error.load());
    REQUIRE_FALSE(host.is_plugin_cancelled());
}

// Collect value-only anchors from the borrowed plan into a returned owned vector; reporting or callback errors fail without mutating source visits.
std::vector<c_wipe_tower_request> collect(Print &print)
{
    std::vector<c_wipe_tower_request> requests;
    run_ctx_wipe_tower_wipe payload = {};
    payload.phase = C_WIPE_TOWER_COLLECT;
    payload.report_context = &requests;
    payload.report_request = &report_request;
    run_service(print, payload);
    return requests;
}

// Compare stored attribute roles without converting them to ExtrusionRole (process-only entities have no base
// type); a matching leaf adds its length times its own mm3_per_mm.
double raw_role_volume(const ExtrusionEntity &entity, const uint16_t raw_role)
{
    if (entity.child_count() == 0) {
        const ExtrusionAttributes *attributes = entity.get_property<ExtrusionAttributes>();
        return attributes != nullptr && attributes->role == raw_role ? unscaled(entity.length()) * attributes->mm3_per_mm : 0.;
    }
    double volume = 0.;
    for (size_t idx = 0; idx < entity.child_count(); ++idx)
        volume += raw_role_volume(entity.child(idx), raw_role);
    return volume;
}

// Walk a borrowed tower tree and extend the caller's platter Y extent; purge and ramming leaves
// also extend purge_max. Travels and other connectors are not tower geometry. Roles are read raw
// because role() asserts on process-only entities.
void tower_leaf_y_extent(const ExtrusionEntity &entity, double &min_y, double &max_y,
                         double &purge_max)
{
    if (entity.child_count() == 0) {
        const ExtrusionAttributes *attributes = entity.get_property<ExtrusionAttributes>();
        if (attributes == nullptr || !entity.has_polyline() ||
            !RAW_EXTRUSION_ROLE_HAS(attributes->role, uint16_t(RAW_EXTRUSION_ROLE_WIPE_TOWER)))
            return;
        const Polyline line = entity.as_polyline().to_polyline();
        for (const Point &point : line.points) {
            const double y = unscaled(point.y());
            min_y = std::min(min_y, y);
            max_y = std::max(max_y, y);
            if (attributes->role == uint16_t(RAW_EXTRUSION_ROLE_WIPE_TOWER_WIPE) ||
                attributes->role == uint16_t(RAW_EXTRUSION_ROLE_WIPE_TOWER_RAMMING))
                purge_max = std::max(purge_max, y);
        }
        return;
    }
    for (size_t idx = 0; idx < entity.child_count(); ++idx)
        tower_leaf_y_extent(entity.child(idx), min_y, max_y, purge_max);
}

// Search the tree depth first and compare the stored attribute role without converting it to
// ExtrusionRole: after the downstream plugins, a root also holds process-only entities (travel,
// retraction) whose roles have no base type. Returns null when no leaf matches. z_offset receives
// the Z offset property in effect on the found leaf: like the firmware, the nearest one on the
// path from the root wins (inherited_offset is the value in effect above entity).
const ExtrusionEntity *first_leaf_with_raw_role(const ExtrusionEntity &entity, const uint16_t raw_role,
                                                coord_t inherited_offset, coord_t &z_offset)
{
    if (const ExtrusionPropertyZOffset *offset = entity.get_property<ExtrusionPropertyZOffset>())
        inherited_offset = offset->z_offset;
    if (entity.child_count() == 0) {
        const ExtrusionAttributes *attributes = entity.get_property<ExtrusionAttributes>();
        if (attributes == nullptr || attributes->role != raw_role)
            return nullptr;
        z_offset = inherited_offset;
        return &entity;
    }
    for (size_t idx = 0; idx < entity.child_count(); ++idx)
        if (const ExtrusionEntity *leaf = first_leaf_with_raw_role(entity.child(idx), raw_role, inherited_offset, z_offset))
            return leaf;
    return nullptr;
}

// Search the whole tree depth first and keep the last leaf whose stored attribute role matches.
const ExtrusionEntity *last_leaf_with_raw_role(const ExtrusionEntity &entity, const uint16_t raw_role)
{
    const ExtrusionEntity *found = nullptr;
    if (entity.child_count() == 0) {
        const ExtrusionAttributes *attributes = entity.get_property<ExtrusionAttributes>();
        if (attributes != nullptr && attributes->role == raw_role)
            found = &entity;
        return found;
    }
    for (size_t idx = 0; idx < entity.child_count(); ++idx)
        if (const ExtrusionEntity *leaf = last_leaf_with_raw_role(entity.child(idx), raw_role))
            found = leaf;
    return found;
}

// Collect the geometric leaves carrying an EXTRUSION command in tree order: they travel while
// extruding a fixed filament length, unlike the point-less in-place filament commands.
std::vector<const ExtrusionEntity *> travelling_filament_leaves(const ExtrusionEntity &entity)
{
    std::vector<const ExtrusionEntity *> leaves;
    if (entity.child_count() == 0) {
        const ExtrusionPropertySpecialCommand *command = entity.get_property<ExtrusionPropertySpecialCommand>();
        const ArcPolyline *polyline = entity.polyline_or_null();
        if (command != nullptr && command->command_code() == ExtrusionPropertySpecialCommand::Code::EXTRUSION &&
            polyline != nullptr && polyline->size() >= 2)
            leaves.push_back(&entity);
        return leaves;
    }
    for (size_t idx = 0; idx < entity.child_count(); ++idx) {
        const std::vector<const ExtrusionEntity *> child_leaves = travelling_filament_leaves(entity.child(idx));
        leaves.insert(leaves.end(), child_leaves.begin(), child_leaves.end());
    }
    return leaves;
}

// Sum every EXTRUSION distance of a tree in order, so the point-less and travelling filament
// commands of a root can be compared with the configured load and unload totals.
double filament_delta_sum(const ExtrusionEntity &entity)
{
    double sum = 0.;
    if (const ExtrusionPropertySpecialCommand *command = entity.get_property<ExtrusionPropertySpecialCommand>())
        if (command->command_code() == ExtrusionPropertySpecialCommand::Code::EXTRUSION)
            sum += command->extra_data;
    for (size_t idx = 0; idx < entity.child_count(); ++idx)
        sum += filament_delta_sum(entity.child(idx));
    return sum;
}

// Expected filament E of one plan tree: flow leaves convert their volume with e_per_mm3 and
// fixed EXTRUSION commands carry their own signed length. Explicit retraction operations are
// expected to net to their restart extras, so they are not added here.
double plan_net_e(const ExtrusionEntity &entity, const double e_per_mm3)
{
    double sum = 0.;
    if (const ExtrusionPropertySpecialCommand *command = entity.get_property<ExtrusionPropertySpecialCommand>())
        if (command->command_code() == ExtrusionPropertySpecialCommand::Code::EXTRUSION)
            sum += command->extra_data;
    const ExtrusionAttributes *attributes = entity.get_property<ExtrusionAttributes>();
    if (attributes != nullptr && attributes->mm3_per_mm > 0.)
        sum += unscaled(entity.length()) * attributes->mm3_per_mm * e_per_mm3;
    for (size_t idx = 0; idx < entity.child_count(); ++idx)
        sum += plan_net_e(entity.child(idx), e_per_mm3);
    return sum;
}

// Sum the explicit tool-change retraction requests of one plan tree.
double plan_toolchange_retraction(const ExtrusionEntity &entity)
{
    double sum = 0.;
    if (const ExtrusionPropertyExtrusionAxis *axis = entity.get_property<ExtrusionPropertyExtrusionAxis>())
        if (axis->operation == C_EXTRUSION_AXIS_OPERATION_RETRACT_TO && axis->toolchange != 0)
            sum += axis->value;
    for (size_t idx = 0; idx < entity.child_count(); ++idx)
        sum += plan_toolchange_retraction(entity.child(idx));
    return sum;
}

// Collect every leaf holding a polyline in tree order, whatever its role; a later plan pass may
// attach process leaves, so callers must filter by role when they need printable geometry.
std::vector<const ExtrusionEntity *> geometric_leaves(const ExtrusionEntity &entity)
{
    std::vector<const ExtrusionEntity *> leaves;
    if (entity.child_count() == 0) {
        if (entity.polyline_or_null() != nullptr)
            leaves.push_back(&entity);
        return leaves;
    }
    for (size_t idx = 0; idx < entity.child_count(); ++idx) {
        const std::vector<const ExtrusionEntity *> child_leaves = geometric_leaves(entity.child(idx));
        leaves.insert(leaves.end(), child_leaves.begin(), child_leaves.end());
    }
    return leaves;
}
}

// Walk concrete empty/nonempty visits in owned groups, requiring every real boundary and no initial purge, with
// incoming index zero. Without EDIT, SEMM adds the outgoing part only where the old visit directly precedes the new
// one in the same layer, and a final outgoing-only floating unload at the end of the last visit; every tool-change
// request is not floating.
TEST_CASE("Tool-change wipe collects every real visit boundary", "[plugins][wipe-tower][toolchange]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    configure_print(print, model, 3);
    const bool semm = GENERATE(false, true);
    const_cast<PrintConfig &>(print.config()).single_extruder_multi_material.value = semm;
    const_cast<PrintConfig &>(print.config()).parallel_objects_step.value = 5.;
    slic3r_api::PrintingPlan plan = Test::GCode::printing_plan_view(print);
    slic3r_api::PrintingGroup first = plan.append_group();
    append_layer(print, first, 0.2, {0, 0, 1});
    append_layer(print, first, 0.4, {2, 2, 0});
    slic3r_api::PrintingGroup second = plan.append_group();
    append_layer(print, second, 0.2, {1, 1});
    const std::vector<c_wipe_tower_request> requests = collect(print);
    REQUIRE(requests.size() == (semm ? 5 : 4));
    const c_wipe_tower_anchor expected[] = {{0, 0, 2, 0}, {0, 1, 0, 0}, {0, 1, 2, 0}, {1, 0, 0, 0}};
    const c_wipe_tower_anchor outgoing[] = {{0, 0, 1, 0}, {}, {0, 1, 1, 0}, {}};
    const bool adjacent[] = {true, false, true, false};
    for (size_t i = 0; i < 4; ++i) {
        CHECK(requests[i].private_id == i + 1);
        CHECK(requests[i].has_incoming == 1);
        CHECK(requests[i].incoming.group_idx == expected[i].group_idx);
        CHECK(requests[i].incoming.layer_group_idx == expected[i].layer_group_idx);
        CHECK(requests[i].incoming.tool_group_idx == expected[i].tool_group_idx);
        CHECK(requests[i].incoming.extrusion_idx == 0);
        CHECK(requests[i].floating == 0);
        CHECK(requests[i].has_outgoing == (semm && adjacent[i] ? 1 : 0));
        if (requests[i].has_outgoing) {
            CHECK(requests[i].outgoing.group_idx == outgoing[i].group_idx);
            CHECK(requests[i].outgoing.layer_group_idx == outgoing[i].layer_group_idx);
            CHECK(requests[i].outgoing.tool_group_idx == outgoing[i].tool_group_idx);
            CHECK(requests[i].outgoing.extrusion_idx == 0);
        }
    }
    if (semm) {
        CHECK(requests[4].has_incoming == 0);
        CHECK(requests[4].has_outgoing == 1);
        CHECK(requests[4].floating == 1);
        CHECK(requests[4].outgoing.group_idx == 1);
        CHECK(requests[4].outgoing.layer_group_idx == 0);
        CHECK(requests[4].outgoing.tool_group_idx == 1);
    }
}

// A SEMM print that never changes tool has nothing to purge, so it reports no request at all: no
// tower is created and the end G-code unloads the filament. One real change still reports the
// floating final unload after it.
TEST_CASE("SEMM final unload needs a reported tool change", "[plugins][wipe-tower][toolchange][semm]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    configure_print(print, model);
    const_cast<PrintConfig &>(print.config()).single_extruder_multi_material.value = true;
    SECTION("one tool only reports nothing") {
        append_layer(print, Test::GCode::printing_plan_view(print).append_group(), 0.2, {1, 1});
        CHECK(collect(print).empty());
    }
    SECTION("one change still unloads at the end") {
        append_layer(print, Test::GCode::printing_plan_view(print).append_group(), 0.2, {0, 1});
        const std::vector<c_wipe_tower_request> requests = collect(print);
        REQUIRE(requests.size() == 2);
        CHECK(requests[0].floating == 0);
        CHECK(requests[1].has_incoming == 0);
        CHECK(requests[1].floating == 1);
    }
}

// SEMM EDIT inserts, at the start of a layer opened by a tool change, an empty visit of the previous tool so its
// unload is printed at that layer; priming inserts one visit per other used tool before the first visit. After EDIT,
// every change carries an outgoing part at the end of the previous visit.
TEST_CASE("SEMM edit adds outgoing and priming visits", "[plugins][wipe-tower][toolchange][semm]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    configure_print(print, model, 3);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.single_extruder_multi_material.value = true;
    const bool priming = GENERATE(false, true);
    config.single_extruder_multi_material_priming.value = priming;
    slic3r_api::PrintingPlan plan = Test::GCode::printing_plan_view(print);
    slic3r_api::PrintingGroup group = plan.append_group();
    append_layer(print, group, 0.2, {0, 1});
    append_layer(print, group, 0.4, {2});
    run_ctx_wipe_tower_wipe payload = {};
    payload.phase = C_WIPE_TOWER_EDIT;
    payload.mutable_plan = reinterpret_cast<printing_plan_handle *>(&print.mutable_printing_plan());
    run_service(print, payload);
    const Printing::PrintingGroup &edited = print.printing_plan()->groups[0];
    std::vector<extruder_index_t> first_layer;
    for (const Printing::PrintingToolGroup &tool : edited.layers[0].tool_groups)
        first_layer.push_back(tool.extruder_id);
    std::vector<extruder_index_t> second_layer;
    for (const Printing::PrintingToolGroup &tool : edited.layers[1].tool_groups)
        second_layer.push_back(tool.extruder_id);
    CHECK(first_layer == (priming ? std::vector<extruder_index_t>{1, 2, 0, 1} : std::vector<extruder_index_t>{0, 1}));
    CHECK(second_layer == std::vector<extruder_index_t>{1, 2});
    const std::vector<c_wipe_tower_request> requests = collect(print);
    REQUIRE(requests.size() == (priming ? 5 : 3));
    for (size_t i = 0; i + 1 < requests.size(); ++i) {
        CHECK(requests[i].has_incoming == 1);
        CHECK(requests[i].has_outgoing == 1);
        CHECK(requests[i].outgoing.group_idx == requests[i].incoming.group_idx);
        CHECK(requests[i].outgoing.layer_group_idx == requests[i].incoming.layer_group_idx);
        CHECK(requests[i].outgoing.tool_group_idx + 1 == requests[i].incoming.tool_group_idx);
    }
    CHECK(requests.back().has_incoming == 0);
    CHECK(requests.back().has_outgoing == 1);
}

// An unresolved visit between two concrete tools is skipped: it neither reports a purge nor
// hides the real change from the tool before it to the tool after it.
TEST_CASE("Tool-change wipe skips unresolved tool visits", "[plugins][wipe-tower][toolchange]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    configure_print(print, model);
    append_layer(print, Test::GCode::printing_plan_view(print).append_group(), 0.2, {0, EXTRUDER_ID_UNKNOWN, 1});
    const std::vector<c_wipe_tower_request> requests = collect(print);
    REQUIRE(requests.size() == 1);
    CHECK(requests[0].incoming.tool_group_idx == 2);
    CHECK(requests[0].incoming.extrusion_idx == 0);
}

// Exercise asymmetric pair/component volumes and incoming/both-tool minimum floors on an owned two-visit plan; each branch preserves its exact expected volume.
TEST_CASE("Tool-change wipe uses matrix components and minimum purge fallbacks", "[plugins][wipe-tower][toolchange]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    configure_print(print, model);
    append_layer(print, Test::GCode::printing_plan_view(print).append_group(), 0.2, {0, 1, 0});
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.filament_minimal_purge_on_wipe_tower.set(std::vector<double>{30., 15.});
    config.wiping_volumes_matrix.set(std::vector<double>{0., 50., 5., 0.});
    double incoming = 50.;
    double returning = 30.;
    SECTION("asymmetric matrix and incoming minimum") {}
    SECTION("empty matrix uses unload plus load") {
        config.wiping_volumes_matrix.set(std::vector<double>{});
        config.wiping_volumes_extruders.set(std::vector<double>{10., 40., 20., 25.});
        incoming = 35.;
        returning = 60.;
    }
    SECTION("wrong-size matrix uses components and minimum") {
        config.wiping_volumes_matrix.set(std::vector<double>{0., 1000., 1000.});
        config.wiping_volumes_extruders.set(std::vector<double>{1., 2., 3., 4.});
        incoming = 15.;
    }
    SECTION("absent matrix and components use larger minimum") {
        config.wiping_volumes_matrix.set(std::vector<double>{});
        config.wiping_volumes_extruders.set(std::vector<double>{});
        incoming = 30.;
    }
    const std::vector<c_wipe_tower_request> requests = collect(print);
    REQUIRE(requests.size() == 2);
    CHECK(requests[0].purge_volume == incoming);
    CHECK(requests[1].purge_volume == returning);
}

// Generate a long owned incoming purge at either band entry side, checking exact rounded flow/pitch, connected local geometry, volume within one row and the explicit four-line ramp scaled by the filament purge factor.
TEST_CASE("Tool-change wipe fills rounded-flow bands with scaled four-line speeds", "[plugins][wipe-tower][toolchange]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    configure_print(print, model);
    append_layer(print, Test::GCode::printing_plan_view(print).append_group(), 0.2, {0, 1});
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    const bool right = GENERATE(false, true);
    const int speed_case = GENERATE(0, 1, 2, 3);
    config.filament_max_wipe_tower_speed.set(std::vector<double>{1., speed_case == 1 ? 45. : 0.});
    if (speed_case == 2)
        config.wipe_tower_wipe_starting_speed.value = 0.;
    if (speed_case == 3) {
        config.wipe_tower_wipe_starting_speed.value = 10.;
        config.wipe_tower_wipe_starting_speed.percent = false;
    }
    const double requested = 500.;
    const TowerLayout layout = layout_tower(20., 1.5, false, 0., {{0.2, 0.2, 0.3, {{0, requested, 0.6, 0.4}}}});
    const LayoutBand &reserved = layout.layers[0].bands[0];
    Orchestrator &host = Orchestrator::instance();
    Plugin *plugin = host.get_plugin(plugin_id);
    plugin_host_context context = host.prepare_plugin_host_context(WIPE_TOWER_WIPE, plugin, &print);
    plugin_run_context run = host.prepare_plugin_run_context(WIPE_TOWER_WIPE, plugin, &context);
    slic3r_api::StoredExtrusionEntity root(run.plugin_storage);
    const c_wipe_tower_request request{1, {0, 0, 1, 0}, {}, 1, 0, requested};
    c_wipe_tower_band band = {};
    band.min = {0., reserved.y0};
    band.max = {20., reserved.y1};
    band.entry = {right ? 20. : 0., reserved.y0};
    band.layer_height = 0.2;
    band.extrusion_width = 0.6;
    band.incoming_root = root.mutable_handle();
    band.line_spacing = reserved.line_spacing;
    c_vec2d exit = {};
    run_ctx_wipe_tower_wipe payload = {};
    payload.phase = C_WIPE_TOWER_GENERATE;
    payload.request = &request;
    payload.band = &band;
    payload.exit = &exit;
    run_service(print, payload);
    const Flow flow = Flow::new_from_width(0.6f, 0.4f, 0.2f, 1.f);
    const double pitch = flow.spacing() * 1.5;
    const ExtrusionEntity &native = *reinterpret_cast<const ExtrusionEntity *>(root.handle());
    REQUIRE(native.child_count() > 4);
    CHECK(native.first_point() == Point::new_scale(band.entry.x, band.entry.y));
    double deposited = 0.;
    for (size_t i = 0; i < native.child_count(); ++i) {
        const ExtrusionEntity &leaf = native.child(i);
        REQUIRE(leaf.get_property<ExtrusionAttributes>() != nullptr);
        const ExtrusionAttributes &attributes = *leaf.get_property<ExtrusionAttributes>();
        CHECK(leaf.role() == ExtrusionRole::WipeTowerWipe);
        CHECK(attributes.width == Approx(0.6));
        CHECK(attributes.height == Approx(0.2));
        CHECK(attributes.mm3_per_mm == Approx(flow.mm3_per_mm()));
        REQUIRE(leaf.get_property<ExtrusionPropertySpeed>() != nullptr);
        const double start = speed_case == 3 ? 10. : 20.;
        const double ramp = speed_case == 2 ? 80. : start + (80. - start) * double(std::min<size_t>(i, 3)) / 3.;
        // filament_max_wipe_tower_speed is a percentage of the purge speed: 45 % here.
        CHECK(leaf.get_property<ExtrusionPropertySpeed>()->speed_mm_per_s ==
            Approx(ramp * (speed_case == 1 ? 0.45 : 1.)));
        CHECK_FALSE(leaf.can_reverse());
        CHECK_FALSE(leaf.can_sort());
        const Polyline line = leaf.as_polyline().to_polyline();
        CHECK(line.points.back().y() == scale_i(double(i) * pitch));
        CHECK(std::abs(unscaled(line.points.back().x() - line.points[line.points.size() - 2].x())) == Approx(20.));
        for (const Point &point : line.points) {
            CHECK(point.x() >= scale_i(band.min.x));
            CHECK(point.x() <= scale_i(band.max.x));
            CHECK(point.y() >= scale_i(band.min.y));
            CHECK(point.y() <= scale_i(band.max.y));
        }
        if (i != 0)
            CHECK(leaf.first_point() == native.child(i - 1).last_point());
        deposited += unscaled(leaf.length()) * attributes.mm3_per_mm;
    }
    CHECK(deposited >= requested - 1e-4);
    CHECK(deposited - requested <= flow.mm3_per_mm() * (20. + pitch) + 1e-4);
    CHECK(native.last_point() == Point::new_scale(exit.x, exit.y));
    CHECK(reserved.y1 == Approx(double(native.child_count()) * pitch));
}

// filament_max_wipe_tower_speed is a percentage of the incoming purge speed: it multiplies the whole
// four-line ramp, 0 falls back to the filament type (soluble 35 %, PVA 60 % below 0.8 mm of tower and
// 80 % above, SCAFF 35 %, FLEX 35 %, others 100 %, matched without case), a positive value wins over
// the type, and the ramming and the SEMM unload/load keep their own speeds.
TEST_CASE("Tool-change purge speed follows the filament percentage and its fallbacks",
          "[plugins][wipe-tower][toolchange]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    configure_print(print, model);
    append_layer(print, Test::GCode::printing_plan_view(print).append_group(), 0.2, {0, 1});
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.single_extruder_multi_material.value = true;
    config.cooling_tube_retraction.value = 91.5;
    config.cooling_tube_length.value = 1.;
    config.parking_pos_retraction.value = 92.;
    config.extra_loading_move.value = -2.;
    config.filament_cooling_moves.set(std::vector<int>{1, 1});
    config.filament_cooling_initial_speed.set(std::vector<double>{20., 20.});
    config.filament_cooling_final_speed.set(std::vector<double>{30., 30.});
    config.filament_loading_speed.set(std::vector<double>{30., 30.});
    config.filament_loading_speed_start.set(std::vector<double>{40., 40.});
    config.filament_unloading_speed.set(std::vector<double>{30., 30.});
    config.filament_unloading_speed_start.set(std::vector<double>{40., 40.});
    config.filament_use_skinnydip.set(std::vector<unsigned char>{0, 0});
    config.filament_enable_toolchange_temp.set(std::vector<unsigned char>{0, 0});
    config.high_current_on_filament_swap.value = false;
    // Generate the same two-part SEMM request with one filament percentage, material, solubility and
    // tower Z, returning the leaf speeds of both roots.
    const auto run_case = [&](const double percentage, const std::string &material, const bool soluble,
                              const double print_z) {
        config.filament_max_wipe_tower_speed.set(std::vector<double>{0., percentage});
        config.filament_type.set(std::vector<std::string>{"PLA", material});
        config.filament_soluble.set(std::vector<unsigned char>{0, static_cast<unsigned char>(soluble ? 1 : 0)});
        const double requested = 100.;
        const TowerLayout layout = layout_tower(20., 1., false, 0., {{0.2, 0.2, 0.3, {{0, requested, 0.6, 0.4}}}});
        const LayoutBand &reserved = layout.layers[0].bands[0];
        Orchestrator &host = Orchestrator::instance();
        Plugin *plugin = host.get_plugin(plugin_id);
        plugin_host_context context = host.prepare_plugin_host_context(WIPE_TOWER_WIPE, plugin, &print);
        plugin_run_context run = host.prepare_plugin_run_context(WIPE_TOWER_WIPE, plugin, &context);
        slic3r_api::StoredExtrusionEntity outgoing_root(run.plugin_storage);
        slic3r_api::StoredExtrusionEntity incoming_root(run.plugin_storage);
        const c_wipe_tower_request request{1, {0, 0, 1, 0}, {0, 0, 0, 0}, 1, 1, requested};
        c_wipe_tower_band band = {};
        band.min = {0., reserved.y0};
        band.max = {20., reserved.y1};
        band.entry = {0., reserved.y0};
        band.print_z = print_z;
        band.layer_height = 0.2;
        band.extrusion_width = 0.6;
        band.outgoing_root = outgoing_root.mutable_handle();
        band.incoming_root = incoming_root.mutable_handle();
        band.line_spacing = reserved.line_spacing;
        c_vec2d exit = {};
        run_ctx_wipe_tower_wipe payload = {};
        payload.phase = C_WIPE_TOWER_GENERATE;
        payload.request = &request;
        payload.band = &band;
        payload.exit = &exit;
        run_service(print, payload);
        TowerSpeeds speeds;
        collect_tower_speeds(*reinterpret_cast<const ExtrusionEntity *>(outgoing_root.handle()), speeds);
        collect_tower_speeds(*reinterpret_cast<const ExtrusionEntity *>(incoming_root.handle()), speeds);
        return speeds;
    };
    const double main_speed = config.wipe_tower_speed.value > 0. ? config.wipe_tower_speed.value : 80.;
    const double configured_start = config.wipe_tower_wipe_starting_speed.get_effective_value(main_speed);
    const double start_speed = configured_start > 0. ? std::min(configured_start, main_speed) : main_speed;
    // The purge ramps from the configured start to wipe_tower_speed over four lines, then holds it;
    // the filament factor multiplies the whole ramp.
    const auto require_purge = [&](const TowerSpeeds &speeds, const double factor) {
        REQUIRE_FALSE(speeds.purge.empty());
        for (size_t i = 0; i < speeds.purge.size(); ++i) {
            const double ramp = start_speed + (main_speed - start_speed) * double(std::min<size_t>(i, 3)) / 3.;
            CHECK(speeds.purge[i] == Approx(ramp * factor));
        }
    };
    // A positive percentage multiplies the whole ramp, 50 % halves it and it wins over the material.
    const TowerSpeeds full = run_case(0., "PLA", false, 0.2);
    // More than four purge leaves: the four ramp lines and the held speed are all checked.
    CHECK(full.purge.size() > 4);
    require_purge(full, 1.);
    const TowerSpeeds half = run_case(50., "SCAFF", true, 0.2);
    require_purge(half, 0.5);
    REQUIRE(half.purge.size() == full.purge.size());
    for (size_t i = 0; i < full.purge.size(); ++i)
        CHECK(half.purge[i] == Approx(0.5 * full.purge[i]));
    // The ramming and the unload/load filament moves keep their own speeds.
    REQUIRE_FALSE(full.ramming.empty());
    REQUIRE_FALSE(full.filament.empty());
    REQUIRE(half.ramming.size() == full.ramming.size());
    REQUIRE(half.filament.size() == full.filament.size());
    for (size_t i = 0; i < full.ramming.size(); ++i)
        CHECK(half.ramming[i] == Approx(full.ramming[i]));
    for (size_t i = 0; i < full.filament.size(); ++i)
        CHECK(half.filament[i] == Approx(full.filament[i]));
    // Zero uses the filament type fallback, without case, and PVA depends on the tower Z.
    require_purge(run_case(0., "pva", false, 0.2), 0.60);
    require_purge(run_case(0., "PVA", false, 0.8), 0.80);
    require_purge(run_case(0., "SCAFF", false, 0.2), 0.35);
    require_purge(run_case(0., "FLEX", false, 0.2), 0.35);
    require_purge(run_case(0., "", true, 0.2), 0.35);
    require_purge(run_case(0., "PETG", false, 0.2), 1.);
}

// The exported tower keeps the maintenance, the ramming and the SEMM unload/load speeds when only
// the purge factor changes, and the purge follows it: export a two-tool print at 100 % and at 50 %.
TEST_CASE("Tool-change exported tower scales only its purge speed", "[plugins][wipe-tower][toolchange]")
{
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    // Export the same two-tool SEMM print with one purge percentage, returning the tower leaf speeds.
    const auto exported_tower_speeds = [](const double percentage) {
        DynamicPrintConfig export_config = DynamicPrintConfig::full_print_config();
        export_config.set_num_extruders(2);
        export_config.set_deserialize_strict({{"wipe_tower", "1"}, {"wipe_tower_x", "40"}, {"wipe_tower_y", "60"},
            {"wipe_tower_width", "20"}, {"wipe_tower_rotation_angle", "0"}, {"wipe_tower_brim_width", "0"},
            {"wiping_volumes_matrix", "0,8,8,0"}, {"filament_minimal_purge_on_wipe_tower", "2,2"},
            {"filament_diameter", "1.75,1.75"}, {"extrusion_multiplier", "1,1"}, {"filament_type", "PLA,PLA"},
            {"perimeter_extruder", "1"}, {"infill_extruder", "1"}, {"solid_infill_extruder", "1"},
            {"step_perimeter_plugin", "perimeter.generator.classic"}, {"step_surface_generation_plugin", "surface.initial_typed_surface_builder"},
            {"step_wipetower_plugin", "wipe_tower.manager.default"}, {"layer_height", "0.3"}, {"fill_density", "10%"},
            {"single_extruder_multi_material", "1"},
            {"filament_max_wipe_tower_speed", percentage > 0. ? "50,50" : "0,0"}});
        Print export_print;
        Model export_model;
        Test::init_print({Test::TestMesh::cube_20x20x20, Test::TestMesh::cube_20x20x20}, export_print, export_model, export_config);
        for (const char *key : {"perimeter_extruder", "infill_extruder", "solid_infill_extruder"})
            export_model.objects()[1].config.set_deserialize_strict(key, "2");
        export_print.apply(export_model, export_config);
        REQUIRE_NOTHROW(export_print.process());
        const boost::filesystem::path checkout = boost::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
        const std::string path = (checkout / "build" / "temp" /
            (std::string("toolchange_purge_factor_") + (percentage > 0. ? "half" : "full") + ".gcode")).string();
        GCodeProcessorResult result;
        REQUIRE_NOTHROW(Orchestrator::instance().export_gcode(export_print, path, &result));
        TowerSpeeds speeds;
        for (const Printing::PrintingGroup &group : export_print.printing_plan()->groups)
            for (const Printing::PrintingLayerGroup &layer : group.layers)
                for (const Printing::PrintingToolGroup &tool : layer.tool_groups)
                    for (const Printing::PrintingExtrusion &extrusion : tool.extrusions)
                        if (extrusion.root != nullptr && extrusion.sregion_island_role.is_wipetower())
                            collect_tower_speeds(*extrusion.root, speeds);
        return speeds;
    };
    const TowerSpeeds exported_full = exported_tower_speeds(0.);
    const TowerSpeeds exported_half = exported_tower_speeds(50.);
    REQUIRE_FALSE(exported_full.maintenance.empty());
    REQUIRE_FALSE(exported_full.ramming.empty());
    REQUIRE_FALSE(exported_full.purge.empty());
    CHECK(exported_half.maintenance.size() == exported_full.maintenance.size());
    CHECK(exported_half.ramming.size() == exported_full.ramming.size());
    CHECK(exported_half.filament.size() == exported_full.filament.size());
    for (size_t i = 0; i < exported_full.maintenance.size(); ++i)
        CHECK(exported_half.maintenance[i] == Approx(exported_full.maintenance[i]));
    for (size_t i = 0; i < exported_full.ramming.size(); ++i)
        CHECK(exported_half.ramming[i] == Approx(exported_full.ramming[i]));
    for (size_t i = 0; i < exported_full.filament.size(); ++i)
        CHECK(exported_half.filament[i] == Approx(exported_full.filament[i]));
    REQUIRE(exported_half.purge.size() == exported_full.purge.size());
    for (size_t i = 0; i < exported_full.purge.size(); ++i)
        CHECK(exported_half.purge[i] == Approx(0.5 * exported_full.purge[i]));
}

// During a SEMM swap the unload requests the incoming filament temperature (first_layer_temperature on
// the first layer of the plan, temperature above it) without waiting and without a tool address, after
// the pulls and before the cooling moves. With filament_enable_toolchange_temp the toolchange
// temperature is waited for before the pulls (normal mode) or after the cooling moves (fast
// skinnydip), with the toolchange part fan, and nothing requests a temperature after the tool change.
// The final unload requests none, and a multi-nozzle tower requests none at all.
// The no-request check is limited to the layer of the first change because later layers legitimately
// receive root-temperature requests from the layer plugin.
TEST_CASE("SEMM tool change temperatures follow the swap phases",
          "[plugins][wipe-tower][toolchange][semm]")
{
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const int mode = GENERATE(0, 1, 2, 3);  // 0: no toolchange temp, 1: normal, 2: fast skinnydip, 3: multi-nozzle
    CAPTURE(mode);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_extruders(2);
    const bool toolchange = mode == 1 || mode == 2;
    config.set_deserialize_strict({{"wipe_tower", "1"}, {"wipe_tower_x", "40"}, {"wipe_tower_y", "60"},
        {"wipe_tower_width", "20"}, {"wipe_tower_rotation_angle", "0"}, {"wipe_tower_brim_width", "0"},
        {"wiping_volumes_matrix", "0,8,8,0"}, {"filament_minimal_purge_on_wipe_tower", "2,2"},
        {"filament_diameter", "1.75,1.75"}, {"extrusion_multiplier", "1,1"},
        {"perimeter_extruder", "1"}, {"infill_extruder", "1"}, {"solid_infill_extruder", "1"},
        {"step_perimeter_plugin", "perimeter.generator.classic"}, {"step_surface_generation_plugin", "surface.initial_typed_surface_builder"},
        {"step_wipetower_plugin", "wipe_tower.manager.default"}, {"layer_height", "0.3"}, {"fill_density", "10%"},
        {"temperature", "200,240"}, {"first_layer_temperature", "210,250"},
        {"single_extruder_multi_material", mode == 3 ? "0" : "1"},
        {"filament_multitool_ramming", "1,1"},
        {"filament_multitool_ramming_volume", "10,10"}, {"filament_multitool_ramming_flow", "5,5"},
        {"filament_enable_toolchange_temp", toolchange ? "1,1" : "0,0"},
        {"filament_toolchange_temp", "180,190"},
        {"filament_use_fast_skinnydip", mode == 2 ? "1,1" : "0,0"},
        {"filament_use_skinnydip", mode == 2 ? "1,1" : "0,0"},
        {"filament_enable_toolchange_part_fan", toolchange ? "1,1" : "0,0"},
        {"filament_toolchange_part_fan_speed", "50,50"}});
    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20, Test::TestMesh::cube_20x20x20}, print, model, config);
    for (const char *key : {"perimeter_extruder", "infill_extruder", "solid_infill_extruder"})
        model.objects()[1].config.set_deserialize_strict(key, "2");
    print.apply(model, config);
    REQUIRE_NOTHROW(print.process());
    const boost::filesystem::path checkout = boost::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
    const std::string path = (checkout / "build" / "temp" /
        (std::string("toolchange_temperature_") + std::to_string(mode) + ".gcode")).string();
    GCodeProcessorResult result;
    REQUIRE_NOTHROW(Orchestrator::instance().export_gcode(print, path, &result));

    // The first change's ramming end opens its unload, whose pulls are E-only moves and whose cooling
    // moves are geometric; every temperature or fan command is recorded with the phase and the
    // counters in effect when it is emitted.
    Point ramming_end{};
    bool has_ramming = false;
    for (const Printing::PrintingGroup &group : print.printing_plan()->groups)
        for (const Printing::PrintingLayerGroup &layer : group.layers)
            for (const Printing::PrintingToolGroup &tool : layer.tool_groups)
                for (const Printing::PrintingExtrusion &extrusion : tool.extrusions) {
                    if (extrusion.root == nullptr || !extrusion.sregion_island_role.is_wipetower())
                        continue;
                    if (const ExtrusionEntity *ramming = last_leaf_with_raw_role(
                            *extrusion.root, uint16_t(RAW_EXTRUSION_ROLE_WIPE_TOWER_RAMMING)))
                        if (!has_ramming) {
                            ramming_end = ramming->last_point();
                            has_ramming = true;
                        }
                }
    REQUIRE(has_ramming);
    const auto matches = [](const double x, const double y, const Point &point) {
        return std::abs(x - unscaled(point.x())) < 0.002 && std::abs(y - unscaled(point.y())) < 0.002;
    };
    // One temperature or fan command with the unload state seen before it.
    struct Event {
        bool fan = false;
        bool wait = false;
        int value = 0;
        int phase = 0;             // 1 pulls, 2 cooling, 3 after the cooling, 4 after the tool change
        size_t pulls = 0;
        size_t geometric = 0;
        size_t tower_changes = 0;  // tool changes after the first tower marker, before this command
        size_t layer_changes = 0;
        bool tower = false;
    };
    std::vector<Event> events;
    const std::string gcode = Test::GCode::read_text_file(path);
    GCodeReader reader;
    reader.apply_config(print.config());
    bool tower = false;
    bool in_unload = false;
    int phase = 0;
    size_t pulls = 0, geometric = 0;
    size_t first_pulls = 0, first_geometric = 0;
    size_t tower_changes = 0, layer_changes = 0;
    int first_change_tool = -1;
    size_t first_change_layer_changes = 0;
    reader.parse_buffer(gcode, [&](GCodeReader &state, const GCodeReader::GCodeLine &line) {
        if (line.comment() == "TYPE:Wipe tower") {
            tower = true;
            return;
        }
        if (line.comment() == "LAYER_CHANGE") {
            ++layer_changes;
            return;
        }
        if (line.cmd_is("T0") || line.cmd_is("T1")) {
            if (tower) {
                if (first_change_tool < 0) {
                    first_change_tool = line.cmd_is("T0") ? 0 : 1;
                    first_pulls = pulls;
                    first_geometric = geometric;
                    first_change_layer_changes = layer_changes;
                }
                ++tower_changes;
            }
            if (phase != 0)
                phase = 4;
            in_unload = false;
            return;
        }
        float value = 0.f;
        const bool temperature = line.cmd_is("M104") || line.cmd_is("M109");
        const bool fan = line.cmd_is("M106");
        if (temperature || fan) {
            Event event;
            event.fan = fan;
            event.wait = line.cmd_is("M109");
            // Marlin waits use M109 R, other firmwares M109 S.
            if (!line.has_value('S', value))
                line.has_value('R', value);
            event.value = int(std::lround(value));
            event.phase = phase;
            event.pulls = pulls;
            event.geometric = geometric;
            event.tower_changes = tower_changes;
            event.layer_changes = layer_changes;
            event.tower = tower;
            events.push_back(event);
            return;
        }
        if (!line.cmd_is("G0") && !line.cmd_is("G1"))
            return;
        if (!in_unload) {
            if (matches(line.new_X(state), line.new_Y(state), ramming_end)) {
                in_unload = true;
                phase = 1;
            }
            return;
        }
        const bool is_geometric = line.has(X) || line.has(Y);
        if (phase == 1 && is_geometric) {
            phase = 2;
            ++geometric;
        } else if (phase == 1) {
            ++pulls;
        } else if (phase == 2) {
            if (is_geometric)
                ++geometric;
            else
                phase = 3;
        }
    });
    // The tower's own temperature requests: one incoming request per change without a toolchange
    // temperature, the toolchange and incoming ones with it, plus the final unload's toolchange one.
    size_t requested = 0;
    for (const Printing::PrintingGroup &group : print.printing_plan()->groups)
        for (const Printing::PrintingLayerGroup &layer : group.layers)
            for (const Printing::PrintingToolGroup &tool : layer.tool_groups)
                for (const Printing::PrintingExtrusion &extrusion : tool.extrusions)
                    if (extrusion.root != nullptr && extrusion.sregion_island_role.is_wipetower())
                        requested += temperature_request_count(*extrusion.root);
    if (mode == 3) {
        // The multi-nozzle tower requests no temperature of its own.
        CHECK(requested == 0);
        return;
    }
    // Each two-part request asks the incoming temperature on its cooling part and again on its
    // parking part (the same value, emitted once); normal mode adds the toolchange one on the pulls,
    // fast skinnydip on the cooling part, and the final unload keeps its toolchange request.
    const size_t expected_requests = mode == 0 ? 2 * tower_changes :
        (mode == 1 ? 3 * tower_changes + 1 : 2 * tower_changes + 1);
    CHECK(requested == expected_requests);
    // No temperature command carries a tool address in SEMM.
    {
        size_t addressed = 0;
        std::istringstream stream(gcode);
        std::string line;
        while (std::getline(stream, line))
            if ((line.compare(0, 6, "M104 S") == 0 || line.compare(0, 6, "M109 S") == 0) &&
                line.find(" T") != std::string::npos)
                ++addressed;
        CHECK(addressed == 0);
    }
    REQUIRE(first_change_tool >= 0);
    const int outgoing_tool = 1 - first_change_tool;
    const int incoming_first = first_change_tool == 0 ? 210 : 250;
    const int incoming_normal = first_change_tool == 0 ? 200 : 240;
    const int toolchange_temp = outgoing_tool == 0 ? 180 : 190;
    // The four pulls and the cooling moves of the first change exist, so the placement checks below
    // cannot pass vacuously.
    CHECK(first_pulls == 4);
    CHECK(first_geometric > 0);
    // The temperature commands of the first change's unload, in order.
    std::vector<Event> first;
    for (const Event &event : events)
        if (event.tower && event.tower_changes == 0 && !event.fan)
            first.push_back(event);
    if (mode == 0) {
        REQUIRE(first.size() == 1);
        CHECK(first[0].value == incoming_first);
        CHECK_FALSE(first[0].wait);
        CHECK(first[0].phase == 1);
        CHECK(first[0].pulls == first_pulls);   // after the pulls, before the cooling moves
        CHECK(first[0].geometric == 0);
    } else if (mode == 1) {
        REQUIRE(first.size() == 2);
        CHECK(first[0].wait);
        CHECK(first[0].value == toolchange_temp);
        CHECK(first[0].phase == 1);
        CHECK(first[0].pulls == 0);             // the wait precedes the pulls
        CHECK(first[1].value == incoming_first);
        CHECK_FALSE(first[1].wait);
        CHECK(first[1].phase == 1);
        CHECK(first[1].pulls == first_pulls);   // after the pulls, before the cooling moves
        CHECK(first[1].geometric == 0);
    } else {
        REQUIRE(first.size() == 3);
        CHECK_FALSE(first[0].wait);
        CHECK(first[0].value == toolchange_temp);
        CHECK(first[0].phase == 1);
        CHECK(first[0].pulls == first_pulls);   // after the pulls, before the cooling moves
        CHECK(first[1].wait);
        CHECK(first[1].value == toolchange_temp);
        CHECK(first[1].geometric == first_geometric);   // after the cooling moves
        CHECK(first[2].value == incoming_first);
        CHECK_FALSE(first[2].wait);
        CHECK(first[2].geometric == first_geometric);
    }
    if (toolchange) {
        // The toolchange part fan stays with the toolchange temperature: with the wait in normal
        // mode, and with the cooling part in fast skinnydip mode.
        size_t fan_index = events.size();
        for (size_t i = 0; i < events.size(); ++i)
            if (events[i].fan && events[i].tower && events[i].tower_changes == 0) {
                fan_index = i;
                break;
            }
        REQUIRE(fan_index < events.size());
        const size_t expected_fan_pulls = mode == 1 ? 0 : first_pulls;
        CHECK(events[fan_index].value == 128);  // 50 % of 255
        CHECK(events[fan_index].phase == 1);
        CHECK(events[fan_index].pulls == expected_fan_pulls);
        // The fan is requested after the heater command (blocking in normal
        // mode, nonblocking in fast skinnydip), never during the normal wait.
        REQUIRE(fan_index > 0);
        CHECK_FALSE(events[fan_index - 1].fan);
        CHECK(events[fan_index - 1].value == toolchange_temp);
        CHECK(events[fan_index - 1].wait == (mode == 1));
    }
    // The first layer of the plan uses first_layer_temperature, every later layer temperature.
    for (const Event &event : events)
        if (event.tower && !event.fan &&
            (event.value == 200 || event.value == 210 || event.value == 240 || event.value == 250)) {
            if (event.layer_changes <= 1)
                CHECK((event.value == 210 || event.value == 250));
            else
                CHECK((event.value == 200 || event.value == 240));
        }
    // On the first-change layer nothing requests a temperature between the change and the load; later
    // layers may legitimately receive root-temperature requests from the layer plugin. The final
    // unload, which runs after the last change, requests no incoming temperature either.
    for (const Event &event : events) {
        const bool after_change = event.tower && event.phase == 4 && event.tower_changes == 1 &&
            event.layer_changes == first_change_layer_changes;
        const bool incoming_after_last_change = event.tower && event.tower_changes == tower_changes &&
            (event.value == 200 || event.value == 210 || event.value == 240 || event.value == 250);
        CHECK_FALSE(after_change);
        CHECK_FALSE(incoming_after_last_change);
    }
    CHECK(tower_changes > 1);
}

// Generate a two-part request through the service alone: the manager owns the speed-ratio commands,
// so this call must place none. SEMM: the outgoing root rams the default profile volume (10 steps of
// 0.25 s) at the start of the band, unloads the filament by exactly parking_pos_retraction (the
// cooling moves cancel out) and flushes the planner; the incoming root loads parking_pos_retraction +
// extra_loading_move and purges the rest of the band. Multi-nozzle with multitool ramming:
// only the configured ramming volume at the configured flow, then the flush, and the purge.
TEST_CASE("Two-part tool change rams, unloads and loads in its band", "[plugins][wipe-tower][toolchange][semm]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    configure_print(print, model);
    append_layer(print, Test::GCode::printing_plan_view(print).append_group(), 0.2, {0, 1});
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    const bool semm = GENERATE(false, true);
    config.single_extruder_multi_material.value = semm;
    config.filament_multitool_ramming.set(std::vector<unsigned char>{1, 0});
    config.filament_multitool_ramming_volume.set(std::vector<double>{10., 10.});
    config.filament_multitool_ramming_flow.set(std::vector<double>{5., 5.});
    config.cooling_tube_retraction.value = 91.5;
    config.cooling_tube_length.value = 5.;
    config.parking_pos_retraction.value = 92.;
    config.extra_loading_move.value = -2.;
    config.filament_cooling_moves.set(std::vector<int>{2, 2});
    config.filament_use_skinnydip.set(std::vector<unsigned char>{0, 0});
    config.filament_enable_toolchange_temp.set(std::vector<unsigned char>{0, 0});
    const double ramming = semm ? 0.25 * (6.6 + 6.8 + 7.2 + 7.6 + 7.9 + 8.2 + 8.7 + 9.4 + 9.9 + 10.0) : 10.;
    const double purge = 50.;
    const double requested = purge + ramming;
    const TowerLayout layout = layout_tower(20., 1., false, 0., {{0.2, 0.2, 0.3, {{0, requested, 0.6, 0.4}}}});
    const LayoutBand &reserved = layout.layers[0].bands[0];
    Orchestrator &host = Orchestrator::instance();
    Plugin *plugin = host.get_plugin(plugin_id);
    plugin_host_context context = host.prepare_plugin_host_context(WIPE_TOWER_WIPE, plugin, &print);
    plugin_run_context run = host.prepare_plugin_run_context(WIPE_TOWER_WIPE, plugin, &context);
    slic3r_api::StoredExtrusionEntity outgoing_root(run.plugin_storage);
    slic3r_api::StoredExtrusionEntity incoming_root(run.plugin_storage);
    const c_wipe_tower_request request{1, {0, 0, 1, 0}, {0, 0, 0, 0}, 1, 1, requested};
    c_wipe_tower_band band = {};
    band.min = {0., reserved.y0};
    band.max = {20., reserved.y1};
    band.entry = {0., reserved.y0};
    band.layer_height = 0.2;
    band.extrusion_width = 0.6;
    band.outgoing_root = outgoing_root.mutable_handle();
    band.incoming_root = incoming_root.mutable_handle();
    band.line_spacing = reserved.line_spacing;
    c_vec2d exit = {};
    run_ctx_wipe_tower_wipe payload = {};
    payload.phase = C_WIPE_TOWER_GENERATE;
    payload.request = &request;
    payload.band = &band;
    payload.exit = &exit;
    run_service(print, payload);

    struct Summary { double rammed = 0.; double purged = 0.; double filament = 0.; std::vector<ExtrusionPropertySpecialCommand::Code> commands; };
    // Sum the geometric volumes per role and the E-only filament moves of a borrowed tree, in order.
    std::function<void(const ExtrusionEntity &, Summary &)> summarize = [&summarize](const ExtrusionEntity &entity, Summary &summary) {
        if (const ExtrusionPropertySpecialCommand *command = entity.get_property<ExtrusionPropertySpecialCommand>()) {
            summary.commands.push_back(command->command_code());
            if (command->command_code() == ExtrusionPropertySpecialCommand::Code::EXTRUSION)
                summary.filament += command->extra_data;
        }
        if (entity.child_count() == 0) {
            const ExtrusionAttributes *attributes = entity.get_property<ExtrusionAttributes>();
            if (attributes != nullptr && attributes->role == uint16_t(RAW_EXTRUSION_ROLE_WIPE_TOWER_RAMMING))
                summary.rammed += unscaled(entity.length()) * attributes->mm3_per_mm;
            if (attributes != nullptr && attributes->role == uint16_t(RAW_EXTRUSION_ROLE_WIPE_TOWER_WIPE))
                summary.purged += unscaled(entity.length()) * attributes->mm3_per_mm;
        }
        for (size_t idx = 0; idx < entity.child_count(); ++idx)
            summarize(entity.child(idx), summary);
    };
    Summary out;
    summarize(*reinterpret_cast<const ExtrusionEntity *>(outgoing_root.handle()), out);
    Summary in;
    summarize(*reinterpret_cast<const ExtrusionEntity *>(incoming_root.handle()), in);
    const Flow flow = Flow::new_from_width(0.6f, 0.4f, 0.2f, 1.f);
    CHECK(out.rammed == Approx(ramming).margin(1e-3));
    CHECK(out.purged == 0.);
    CHECK(in.rammed == 0.);
    CHECK(in.purged >= purge - 1e-3);
    CHECK(out.rammed + in.purged - requested <= flow.mm3_per_mm() * (20. + reserved.line_spacing) + 1e-3);
    REQUIRE_FALSE(out.commands.empty());
    CHECK(out.commands.back() == ExtrusionPropertySpecialCommand::Code::FLUSH_PLANNER_QUEUE);
    const bool saves = std::find(out.commands.begin(), out.commands.end(),
        ExtrusionPropertySpecialCommand::Code::SAVE_AND_RESET_SPEED_RATIO) != out.commands.end();
    const bool restores = std::find(in.commands.begin(), in.commands.end(),
        ExtrusionPropertySpecialCommand::Code::RESTORE_SPEED_RATIO) != in.commands.end();
    // The manager adds these around the whole request, whatever the printer type.
    CHECK_FALSE(saves);
    CHECK_FALSE(restores);
    CHECK(out.filament == Approx(semm ? -92. : 0.).margin(1e-6));
    CHECK(in.filament == Approx(semm ? 90. : 0.).margin(1e-6));
}

// Pressure advance is a request carried by the two band roots: the outgoing root asks for zero while
// it rams and unloads when the outgoing filament enables filament_pressure_advance, and the incoming
// root asks for the incoming filament value when it enables it. A disabled filament leaves its root
// without a request, so a value written by the filament G-code stays.
TEST_CASE("Tool change roots request pressure advance", "[plugins][wipe-tower][toolchange][pressure-advance]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    configure_print(print, model);
    append_layer(print, Test::GCode::printing_plan_view(print).append_group(), 0.2, {0, 1});
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    const bool semm = GENERATE(false, true);
    const bool enabled_outgoing = GENERATE(false, true);
    const bool enabled_incoming = GENERATE(false, true);
    CAPTURE(semm, enabled_outgoing, enabled_incoming);
    config.single_extruder_multi_material.value = semm;
    config.filament_multitool_ramming.set(std::vector<unsigned char>{1, 0});
    config.filament_multitool_ramming_volume.set(std::vector<double>{10., 10.});
    config.filament_multitool_ramming_flow.set(std::vector<double>{5., 5.});
    config.filament_pressure_advance.set(std::vector<double>{0.04, 0.09});
    config.filament_pressure_advance.set_enabled(enabled_outgoing, 0);
    config.filament_pressure_advance.set_enabled(enabled_incoming, 1);
    const double ramming = semm ? 0.25 * (6.6 + 6.8 + 7.2 + 7.6 + 7.9 + 8.2 + 8.7 + 9.4 + 9.9 + 10.0) : 10.;
    const double purge = 50.;
    const double requested = purge + ramming;
    const TowerLayout layout = layout_tower(20., 1., false, 0., {{0.2, 0.2, 0.3, {{0, requested, 0.6, 0.4}}}});
    const LayoutBand &reserved = layout.layers[0].bands[0];
    Orchestrator &host = Orchestrator::instance();
    Plugin *plugin = host.get_plugin(plugin_id);
    plugin_host_context context = host.prepare_plugin_host_context(WIPE_TOWER_WIPE, plugin, &print);
    plugin_run_context run = host.prepare_plugin_run_context(WIPE_TOWER_WIPE, plugin, &context);
    slic3r_api::StoredExtrusionEntity outgoing_root(run.plugin_storage);
    slic3r_api::StoredExtrusionEntity incoming_root(run.plugin_storage);
    const c_wipe_tower_request request{1, {0, 0, 1, 0}, {0, 0, 0, 0}, 1, 1, requested};
    c_wipe_tower_band band = {};
    band.min = {0., reserved.y0};
    band.max = {20., reserved.y1};
    band.entry = {0., reserved.y0};
    band.layer_height = 0.2;
    band.extrusion_width = 0.6;
    band.outgoing_root = outgoing_root.mutable_handle();
    band.incoming_root = incoming_root.mutable_handle();
    band.line_spacing = reserved.line_spacing;
    c_vec2d exit = {};
    run_ctx_wipe_tower_wipe payload = {};
    payload.phase = C_WIPE_TOWER_GENERATE;
    payload.request = &request;
    payload.band = &band;
    payload.exit = &exit;
    run_service(print, payload);

    const slic3r_api::ExtrusionEntity out(outgoing_root.handle());
    const slic3r_api::ExtrusionEntity in(incoming_root.handle());
    const slic3r_api::EPropertySpeed *out_speed = out.get(slic3r_api::EPropertySpeed::key);
    const slic3r_api::EPropertyHeater *out_heater = out.get(slic3r_api::EPropertyHeater::key);
    const slic3r_api::EPropertySpeed *in_speed = in.get(slic3r_api::EPropertySpeed::key);
    if (enabled_outgoing) {
        REQUIRE(out_speed != nullptr);
        CHECK(out_speed->pressure_adv == 0.f);
        // Only pressure advance is overridden; the other requested values stay unset.
        CHECK(out_speed->speed_mm_per_s < 0.f);
        CHECK(out_heater == nullptr);
    } else {
        CHECK(out_speed == nullptr);
    }
    if (enabled_incoming) {
        REQUIRE(in_speed != nullptr);
        CHECK(in_speed->pressure_adv == Approx(0.09f));
    } else {
        CHECK(in_speed == nullptr);
    }
}

// In SEMM, the unload cooling moves and the load fast parts travel along the band's work line:
// one band line above the ramming end, capped at the band top, inside band.min.x..band.max.x. Each
// move lasts |E| / configured filament speed, its XY speed is the travelled distance over that
// duration, and the machine ceiling only shortens the travel without slowing the filament.
TEST_CASE("SEMM unload and load travel along the band work line", "[plugins][wipe-tower][toolchange][semm]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    configure_print(print, model);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.single_extruder_multi_material.value = true;
    config.cooling_tube_retraction.value = 91.5;
    config.cooling_tube_length.value = 1.;
    config.parking_pos_retraction.value = 92.;
    config.extra_loading_move.value = -2.;
    config.filament_cooling_moves.set(std::vector<int>{1, 1});
    config.filament_cooling_initial_speed.set(std::vector<double>{20., 20.});
    config.filament_cooling_final_speed.set(std::vector<double>{30., 30.});
    config.filament_loading_speed.set(std::vector<double>{30., 30.});
    config.filament_loading_speed_start.set(std::vector<double>{40., 40.});
    config.filament_unloading_speed.set(std::vector<double>{30., 30.});
    config.filament_unloading_speed_start.set(std::vector<double>{40., 40.});
    config.filament_use_skinnydip.set(std::vector<unsigned char>{0, 0});
    config.filament_enable_toolchange_temp.set(std::vector<unsigned char>{0, 0});
    config.high_current_on_filament_swap.value = false;
    double expected_cap = 0.;       // 0 means no ceiling applies
    bool expect_shortened = false;  // set when the unshortened travel must exceed the ceiling
    SECTION("machine limits are high enough") {
        config.machine_limits_usage.value = MachineLimitsUsage::EmitToGCode;
        config.machine_max_feedrate_x.set(std::vector<double>{500.});
        config.machine_max_feedrate_y.set(std::vector<double>{500.});
        expected_cap = 500.;
    }
    SECTION("the X ceiling shortens the travel") {
        config.machine_limits_usage.value = MachineLimitsUsage::EmitToGCode;
        config.machine_max_feedrate_x.set(std::vector<double>{5.});
        config.machine_max_feedrate_y.set(std::vector<double>{500.});
        expected_cap = 5.;
        expect_shortened = true;
    }
    SECTION("ignored machine limits never shorten the travel") {
        config.machine_limits_usage.value = MachineLimitsUsage::Ignore;
        config.machine_max_feedrate_x.set(std::vector<double>{5.});
        config.machine_max_feedrate_y.set(std::vector<double>{5.});
    }
    SECTION("no positive limit falls back to 100 mm/s") {
        config.machine_limits_usage.value = MachineLimitsUsage::Limits;
        config.machine_max_feedrate_x.set(std::vector<double>{0.});
        config.machine_max_feedrate_y.set(std::vector<double>{0.});
        config.filament_cooling_initial_speed.set(std::vector<double>{200., 200.});
        config.filament_cooling_final_speed.set(std::vector<double>{200., 200.});
        expected_cap = 100.;
        expect_shortened = true;
    }
    SECTION("the slower Y axis caps the X travel") {
        config.machine_limits_usage.value = MachineLimitsUsage::EmitToGCode;
        config.machine_max_feedrate_x.set(std::vector<double>{500.});
        config.machine_max_feedrate_y.set(std::vector<double>{30.});
        config.filament_cooling_initial_speed.set(std::vector<double>{200., 200.});
        config.filament_cooling_final_speed.set(std::vector<double>{200., 200.});
        expected_cap = 30.;
        expect_shortened = true;
    }
    const double requested = 100.;
    const TowerLayout layout = layout_tower(20., 1., false, 0., {{0.2, 0.2, 0.3, {{0, requested, 0.6, 0.4}}}});
    const LayoutBand &reserved = layout.layers[0].bands[0];
    Orchestrator &host = Orchestrator::instance();
    Plugin *plugin = host.get_plugin(plugin_id);
    plugin_host_context context = host.prepare_plugin_host_context(WIPE_TOWER_WIPE, plugin, &print);
    plugin_run_context run = host.prepare_plugin_run_context(WIPE_TOWER_WIPE, plugin, &context);
    slic3r_api::StoredExtrusionEntity outgoing_root(run.plugin_storage);
    slic3r_api::StoredExtrusionEntity incoming_root(run.plugin_storage);
    const c_wipe_tower_request request{1, {0, 0, 1, 0}, {0, 0, 0, 0}, 1, 1, requested};
    c_wipe_tower_band band = {};
    band.min = {0., reserved.y0};
    band.max = {20., reserved.y1};
    band.entry = {0., reserved.y0};
    band.layer_height = 0.2;
    band.extrusion_width = 0.6;
    band.outgoing_root = outgoing_root.mutable_handle();
    band.incoming_root = incoming_root.mutable_handle();
    band.line_spacing = reserved.line_spacing;
    c_vec2d exit = {};
    run_ctx_wipe_tower_wipe payload = {};
    payload.phase = C_WIPE_TOWER_GENERATE;
    payload.request = &request;
    payload.band = &band;
    payload.exit = &exit;
    run_service(print, payload);

    const ExtrusionEntity &outgoing = *reinterpret_cast<const ExtrusionEntity *>(outgoing_root.handle());
    const ExtrusionEntity &incoming = *reinterpret_cast<const ExtrusionEntity *>(incoming_root.handle());
    // The work line is one band line above the ramming end, capped at the band top.
    const ExtrusionEntity *ramming_end = last_leaf_with_raw_role(outgoing, uint16_t(RAW_EXTRUSION_ROLE_WIPE_TOWER_RAMMING));
    REQUIRE(ramming_end != nullptr);
    const double work_y = std::min(unscaled(ramming_end->last_point().y()) + band.line_spacing, band.max.y);
    const std::vector<const ExtrusionEntity *> cooling = travelling_filament_leaves(outgoing);
    const std::vector<const ExtrusionEntity *> loading = travelling_filament_leaves(incoming);
    REQUIRE(cooling.size() == 2);
    REQUIRE(loading.size() == 2);
    // E is untouched: the cooling nets zero, the travelling loading parts are the 70 % and 10 %
    // parts, and the complete load and unload totals stay as before.
    CHECK(filament_delta_sum(outgoing) == Approx(-92.).margin(1e-6));
    CHECK(filament_delta_sum(incoming) == Approx(90.).margin(1e-6));
    const double loading_distance = config.parking_pos_retraction.value + config.extra_loading_move.value;
    CHECK(filament_delta_sum(*cooling[0]) == Approx(config.cooling_tube_length.value));
    CHECK(filament_delta_sum(*cooling[1]) == Approx(-config.cooling_tube_length.value));
    CHECK(filament_delta_sum(*loading[0]) == Approx(0.7 * loading_distance));
    CHECK(filament_delta_sum(*loading[1]) == Approx(0.1 * loading_distance));
    // Every travelling leaf stays on the work line inside the band, lasts |E| / filament speed and
    // is shortened to the ceiling when one applies.
    const auto check_leaf = [&](const ExtrusionEntity *leaf, const double filament_speed, const double target_x) {
        const Polyline line = leaf->as_polyline().to_polyline();
        REQUIRE(line.points.size() >= 2);
        const double from_x = unscaled(line.points.front().x());
        const double to_x = unscaled(line.points.back().x());
        CHECK(unscaled(line.points.front().y()) == Approx(work_y));
        CHECK(unscaled(line.points.back().y()) == Approx(work_y));
        CHECK(from_x >= band.min.x - 1e-6);
        CHECK(from_x <= band.max.x + 1e-6);
        CHECK(to_x >= band.min.x - 1e-6);
        CHECK(to_x <= band.max.x + 1e-6);
        const ExtrusionPropertySpecialCommand *command = leaf->get_property<ExtrusionPropertySpecialCommand>();
        REQUIRE(command != nullptr);
        const double duration = std::abs(command->extra_data) / filament_speed;
        const double full = std::abs(target_x - from_x);
        const double travel = expected_cap > 0. ? std::min(full, expected_cap * duration) : full;
        CHECK(std::abs(to_x - from_x) == Approx(travel).margin(1e-6));
        const ExtrusionPropertySpeed *speed = leaf->get_property<ExtrusionPropertySpeed>();
        REQUIRE(speed != nullptr);
        CHECK(speed->speed_mm_per_s == Approx(travel / duration));
        if (expected_cap > 0.)
            CHECK(speed->speed_mm_per_s <= expected_cap + 1e-6);
    };
    const double cooling_start_x = unscaled(cooling[0]->first_point().x());
    const double cooling_far_x = cooling_start_x - band.min.x < band.max.x - cooling_start_x ? band.max.x : band.min.x;
    check_leaf(cooling[0], config.filament_cooling_initial_speed.get_at(0), cooling_far_x);
    check_leaf(cooling[1], config.filament_cooling_final_speed.get_at(0), cooling_start_x);
    const double loading_start_x = unscaled(cooling[1]->last_point().x());
    const double loading_far_x = loading_start_x - band.min.x < band.max.x - loading_start_x ? band.max.x : band.min.x;
    check_leaf(loading[0], config.filament_loading_speed.get_at(0), loading_far_x);
    check_leaf(loading[1], 0.1 * config.filament_loading_speed.get_at(0), loading_start_x);
    if (expect_shortened)
        CHECK(std::abs(unscaled(cooling[0]->last_point().x()) - cooling_start_x) <
              std::abs(cooling_far_x - cooling_start_x) - 1e-9);
    if (config.machine_limits_usage.value == MachineLimitsUsage::Ignore) {
        // The configured 5 mm/s ceiling must not have been applied.
        CHECK(cooling[0]->get_property<ExtrusionPropertySpeed>()->speed_mm_per_s > 5.f);
    }
}

// Slice two tool-specific cubes in SEMM and export them, then slice the same model again with
// machine limits set to ignore: the travelling filament moves stay inside the tower footprint and
// keep their role, zero flow, speed rule and fixed order, the retraction wipe never follows them,
// a travel reaches every purge start after a load, their E is still accounted under the tower role,
// the ceiling shortens them unless it is ignored, and changing only that geometry leaves the tower
// volume measured by the reader unchanged.
TEST_CASE("SEMM band travel stays inside the exported tower", "[plugins][wipe-tower][toolchange][semm]")
{
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_extruders(2);
    config.set_deserialize_strict({{"wipe_tower", "1"}, {"wipe_tower_x", "40"}, {"wipe_tower_y", "60"},
        {"wipe_tower_width", "20"}, {"wipe_tower_rotation_angle", "0"}, {"wipe_tower_brim_width", "0"},
        {"wiping_volumes_matrix", "0,8,8,0"}, {"filament_minimal_purge_on_wipe_tower", "2,2"},
        {"filament_diameter", "1.75,1.75"}, {"extrusion_multiplier", "1,1"},
        {"perimeter_extruder", "1"}, {"infill_extruder", "1"}, {"solid_infill_extruder", "1"},
        {"step_perimeter_plugin", "perimeter.generator.classic"}, {"step_surface_generation_plugin", "surface.initial_typed_surface_builder"},
        {"step_wipetower_plugin", "wipe_tower.manager.default"}, {"layer_height", "0.3"}, {"fill_density", "10%"},
        {"single_extruder_multi_material", "1"},
        {"machine_limits_usage", "emit_to_gcode"},
        {"machine_max_feedrate_x", "5"}, {"machine_max_feedrate_y", "5"}});
    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20, Test::TestMesh::cube_20x20x20}, print, model, config);
    for (const char *key : {"perimeter_extruder", "infill_extruder", "solid_infill_extruder"})
        model.objects()[1].config.set_deserialize_strict(key, "2");
    print.apply(model, config);
    REQUIRE_NOTHROW(print.process());
    const boost::filesystem::path checkout = boost::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
    const std::string path = (checkout / "build" / "temp" / "semm_band_travel.gcode").string();
    GCodeProcessorResult result;
    REQUIRE_NOTHROW(Orchestrator::instance().export_gcode(print, path, &result));

    const Printing::PrintingPlan &plan = *print.printing_plan();
    Points outline;
    for (const Printing::PrintingObjectInstance &instance : plan.object_instances)
        if (instance.name.find("Wipe tower") != std::string::npos)
            outline = instance.projected_outline.points;
    REQUIRE_FALSE(outline.empty());
    // The configured filament speeds of the travelling moves: the cooling moves ramp from the
    // initial to the final cooling speed, and the loading parts use the loading speeds.
    const double cooling_initial = print.config().filament_cooling_initial_speed.get_at(0);
    const double cooling_final = print.config().filament_cooling_final_speed.get_at(0);
    const double loading_fast = print.config().filament_loading_speed.get_at(0);
    const size_t cooling_moves = size_t(std::max(print.config().filament_cooling_moves.get_at(0), 0));
    const double cooling_increment = cooling_moves > 0 ? (cooling_final - cooling_initial) / (2. * cooling_moves - 1.) : 0.;
    std::vector<double> configured_speeds;
    for (size_t move = 0; move < 2 * cooling_moves; ++move)
        configured_speeds.push_back(cooling_initial + cooling_increment * double(move));
    configured_speeds.push_back(loading_fast);
    configured_speeds.push_back(0.1 * loading_fast);
    // One scan of the current plan's tower extrusions, checking the travelling filament moves.
    struct TravelScan {
        size_t travelling = 0;
        size_t wipe_bit_leaves = 0;
        size_t loads = 0;
        bool uncapped = false;
        double flow_volume = 0.;
        std::vector<Point> purge_starts;
        std::vector<std::pair<Point, Point>> moves;
    };
    // Walk the tower extrusions of the current plan: every printable tower point stays inside the
    // synthetic footprint, no retraction wipe leaf was added inside the tower, and every
    // travelling filament move keeps its role, zero flow, fixed order and configured E speed.
    const auto scan_tower = [&](Print &tower_print, const bool ignored) {
        TravelScan scan;
        coord_t min_x = outline.front().x(), max_x = min_x;
        coord_t min_y = outline.front().y(), max_y = min_y;
        for (const Point &point : outline) {
            min_x = std::min(min_x, point.x());
            max_x = std::max(max_x, point.x());
            min_y = std::min(min_y, point.y());
            max_y = std::max(max_y, point.y());
        }
        for (const Printing::PrintingGroup &group : tower_print.printing_plan()->groups)
            for (const Printing::PrintingLayerGroup &layer : group.layers)
                for (const Printing::PrintingToolGroup &tool : layer.tool_groups)
                    for (const Printing::PrintingExtrusion &extrusion : tool.extrusions) {
                        if (extrusion.root == nullptr || !extrusion.sregion_island_role.is_wipetower())
                            continue;
                        for (const ExtrusionEntity *leaf : geometric_leaves(*extrusion.root)) {
                            const ExtrusionAttributes *attributes = leaf->get_property<ExtrusionAttributes>();
                            if (attributes != nullptr && RAW_EXTRUSION_ROLE_IS_WIPE(attributes->role))
                                ++scan.wipe_bit_leaves;
                            // Every printable tower move stays inside the synthetic footprint.
                            if (attributes == nullptr ||
                                !RAW_EXTRUSION_ROLE_HAS(attributes->role, RAW_EXTRUSION_ROLE_WIPE_TOWER))
                                continue;
                            if (attributes->mm3_per_mm > 0.)
                                scan.flow_volume += unscaled(leaf->length()) * attributes->mm3_per_mm;
                            for (const Point &point : leaf->as_polyline().to_polyline().points) {
                                CHECK(point.x() >= min_x - 1);
                                CHECK(point.x() <= max_x + 1);
                                CHECK(point.y() >= min_y - 1);
                                CHECK(point.y() <= max_y + 1);
                            }
                        }
                        // An incoming root carries the load of one tool change.
                        const double load_distance = tower_print.config().parking_pos_retraction.value +
                            tower_print.config().extra_loading_move.value;
                        if (load_distance > 0. &&
                            std::abs(filament_delta_sum(*extrusion.root) - load_distance) < 1e-6)
                            ++scan.loads;
                        // The travelling filament moves keep their zero flow, fixed order, role
                        // and the filament speed imposed by E.
                        for (const ExtrusionEntity *leaf : travelling_filament_leaves(*extrusion.root)) {
                            ++scan.travelling;
                            CHECK_FALSE(leaf->can_reverse());
                            const ExtrusionAttributes *attributes = leaf->get_property<ExtrusionAttributes>();
                            REQUIRE(attributes != nullptr);
                            CHECK(attributes->role == uint16_t(RAW_EXTRUSION_ROLE_WIPE_TOWER));
                            CHECK(attributes->width == 0.f);
                            CHECK(attributes->height == 0.f);
                            CHECK(attributes->mm3_per_mm == 0.);
                            const ExtrusionPropertySpeed *speed = leaf->get_property<ExtrusionPropertySpeed>();
                            REQUIRE(speed != nullptr);
                            CHECK(speed->speed_mm_per_s > 0.f);
                            if (ignored)
                                scan.uncapped = scan.uncapped || speed->speed_mm_per_s > 5.f + 1e-4;
                            else
                                CHECK(speed->speed_mm_per_s <= 5.f + 1e-4);
                            const Polyline line = leaf->as_polyline().to_polyline();
                            scan.moves.emplace_back(line.points.front(), line.points.back());
                            const double dx = std::abs(unscaled(line.points.back().x() - line.points.front().x()));
                            if (dx > 1e-6) {
                                const ExtrusionPropertySpecialCommand *command =
                                    leaf->get_property<ExtrusionPropertySpecialCommand>();
                                REQUIRE(command != nullptr);
                                const double implied = std::abs(command->extra_data) * double(speed->speed_mm_per_s) / dx;
                                bool known = false;
                                for (const double configured : configured_speeds)
                                    known = known || std::abs(implied - configured) <= 0.01 * configured;
                                INFO("implied=" << implied << " e=" << command->extra_data << " dx=" << dx
                                                << " v=" << speed->speed_mm_per_s);
                                CHECK(known);
                            }
                        }
                        coord_t z_offset = 0;
                        const ExtrusionEntity *purge = first_leaf_with_raw_role(*extrusion.root,
                            uint16_t(RAW_EXTRUSION_ROLE_WIPE_TOWER_WIPE), 0, z_offset);
                        if (purge != nullptr)
                            scan.purge_starts.push_back(purge->first_point());
                    }
        return scan;
    };
    const TravelScan scan = scan_tower(print, false);
    CHECK(scan.travelling > 0);
    CHECK(scan.wipe_bit_leaves == 0);
    REQUIRE_FALSE(scan.purge_starts.empty());
    // In the text, a non-extruding move must reach every purge start after a load, and every
    // travelling move must be emitted while the tower role is active, so the reader accounts its
    // E under the tower.
    const std::string gcode = Test::GCode::read_text_file(path);
    std::vector<bool> reached(scan.purge_starts.size(), false);
    std::vector<bool> attributed(scan.moves.size(), false);
    std::string role;
    GCodeReader reader;
    reader.parse_buffer(gcode, [&](GCodeReader &state, const GCodeReader::GCodeLine &line) {
        if (!line.comment().empty()) {
            if (line.comment() == "TYPE:Wipe tower")
                role = "Wipe tower";
            else if (line.comment().rfind("TYPE:", 0) == 0)
                role = line.comment();
        }
        if (!line.cmd_is("G0") && !line.cmd_is("G1"))
            return;
        if (!line.has(E)) {
            for (size_t idx = 0; idx < scan.purge_starts.size(); ++idx)
                if (std::abs(line.new_X(state) - unscaled(scan.purge_starts[idx].x())) < 0.002 &&
                    std::abs(line.new_Y(state) - unscaled(scan.purge_starts[idx].y())) < 0.002)
                    reached[idx] = true;
            return;
        }
        for (size_t idx = 0; idx < scan.moves.size(); ++idx)
            if (std::abs(state.x() - unscaled(scan.moves[idx].first.x())) < 0.002 &&
                std::abs(state.y() - unscaled(scan.moves[idx].first.y())) < 0.002 &&
                std::abs(line.new_X(state) - unscaled(scan.moves[idx].second.x())) < 0.002 &&
                std::abs(line.new_Y(state) - unscaled(scan.moves[idx].second.y())) < 0.002)
                attributed[idx] = attributed[idx] || role == "Wipe tower";
    });
    for (const bool connected : reached)
        CHECK(connected);
    for (const bool under_tower : attributed)
        CHECK(under_tower);
    // The tower volume the reader accounts from the emitted E; its exact value also depends on the
    // retraction debt around the tower, so only positivity is required here.
    const auto tower_volume = [](const GCodeProcessorResult &processor_result) {
        double volume = 0.;
        const std::map<GCodeExtrusionRole, std::map<extruder_index_t, double>> &volumes =
            processor_result.print_statistics.volumes_per_role_per_extruder;
        const std::map<GCodeExtrusionRole, std::map<extruder_index_t, double>>::const_iterator wipe =
            volumes.find(GCodeExtrusionRole::WipeTower);
        REQUIRE(wipe != volumes.end());
        for (const std::pair<const extruder_index_t, double> &entry : wipe->second)
            volume += entry.second;
        return volume;
    };
    const double measured_volume = tower_volume(result);
    CHECK(measured_volume > 0.);
    // The reader accounts the tower flow minus the extra_loading_move remainder of
    // every load; the final unload stays unpaid and is not counted.
    const double filament_area = PI * std::pow(print.config().filament_diameter.get_at(0) * 0.5, 2.);
    CHECK(measured_volume == Approx(scan.flow_volume -
        std::abs(print.config().extra_loading_move.value) * filament_area * double(scan.loads)).margin(1.));
    // Slice the same model again with ignored machine limits: only the travelling geometry
    // changes, not the E stream, so the tower volume measured by the reader must not change.
    DynamicPrintConfig ignore_config = config;
    ignore_config.set_deserialize_strict("machine_limits_usage", "ignore");
    Print ignore_print;
    Model ignore_model;
    Test::init_print({Test::TestMesh::cube_20x20x20, Test::TestMesh::cube_20x20x20}, ignore_print, ignore_model, ignore_config);
    for (const char *key : {"perimeter_extruder", "infill_extruder", "solid_infill_extruder"})
        ignore_model.objects()[1].config.set_deserialize_strict(key, "2");
    ignore_print.apply(ignore_model, ignore_config);
    REQUIRE_NOTHROW(ignore_print.process());
    const std::string ignore_path = (checkout / "build" / "temp" / "semm_band_travel_ignore.gcode").string();
    GCodeProcessorResult ignore_result;
    REQUIRE_NOTHROW(Orchestrator::instance().export_gcode(ignore_print, ignore_path, &ignore_result));
    const TravelScan ignore_scan = scan_tower(ignore_print, true);
    CHECK(ignore_scan.travelling > 0);
    CHECK(ignore_scan.uncapped);
    CHECK(ignore_scan.wipe_bit_leaves == 0);
    CHECK(tower_volume(ignore_result) == Approx(measured_volume).margin(1e-6));
}

// The SEMM unload must stay retracted until the tool change: with a retraction
// scope boundary inside the unload (the work-line travel exceeds
// retract_before_travel), no positive E move may restore it before the T, and
// the reader must account only the tower flow minus the extra_loading_move
// remainder of every load.
TEST_CASE("SEMM unload stays retracted through a retraction boundary",
          "[plugins][wipe-tower][toolchange][semm][retraction]")
{
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_extruders(2);
    config.set_deserialize_strict({{"wipe_tower", "1"}, {"wipe_tower_x", "40"}, {"wipe_tower_y", "60"},
        {"wipe_tower_width", "20"}, {"wipe_tower_rotation_angle", "0"}, {"wipe_tower_brim_width", "0"},
        {"wiping_volumes_matrix", "0,8,8,0"}, {"filament_minimal_purge_on_wipe_tower", "2,2"},
        {"filament_diameter", "1.75,1.75"}, {"extrusion_multiplier", "1,1"},
        {"perimeter_extruder", "1"}, {"infill_extruder", "1"}, {"solid_infill_extruder", "1"},
        {"step_perimeter_plugin", "perimeter.generator.classic"}, {"step_surface_generation_plugin", "surface.initial_typed_surface_builder"},
        {"step_wipetower_plugin", "wipe_tower.manager.default"}, {"layer_height", "0.3"}, {"fill_density", "10%"},
        {"single_extruder_multi_material", "1"},
        // The work-line travel exceeds this threshold, so CreateRetraction places a
        // scope boundary inside the SEMM unload.
        {"retract_before_travel", "0.2,0.2"}});
    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20, Test::TestMesh::cube_20x20x20}, print, model, config);
    for (const char *key : {"perimeter_extruder", "infill_extruder", "solid_infill_extruder"})
        model.objects()[1].config.set_deserialize_strict(key, "2");
    print.apply(model, config);
    REQUIRE_NOTHROW(print.process());
    const boost::filesystem::path checkout = boost::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
    const std::string path = (checkout / "build" / "temp" / "semm_retraction_boundary.gcode").string();
    GCodeProcessorResult result;
    REQUIRE_NOTHROW(Orchestrator::instance().export_gcode(print, path, &result));

    // No positive E move may restore the unload after its last pull and before T.
    const double last_pull = -0.1 * (print.config().cooling_tube_retraction.value +
        print.config().cooling_tube_length.value / 2. - 15.);
    const double cooling_length = print.config().cooling_tube_length.value;
    size_t unloads = 0;
    bool after_pull = false;
    double restored = 0.;
    const std::string gcode = Test::GCode::read_text_file(path);
    GCodeReader reader;
    reader.parse_buffer(gcode, [&](GCodeReader &state, const GCodeReader::GCodeLine &line) {
        if (line.cmd_is("T0") || line.cmd_is("T1")) {
            if (after_pull)
                ++unloads;
            after_pull = false;
            return;
        }
        if (!line.has(E))
            return;
        const double delta = double(line.dist_E(state));
        if (after_pull)
            restored = std::max(restored, delta);
        if (std::abs(delta - last_pull) < 1e-3)
            after_pull = true;
    });
    CHECK(unloads > 0);
    CHECK(restored <= cooling_length + 1e-3);
}

// The SEMM swap is itself the retraction: the unload and load collections disable retraction, so
// CreateRetraction adds no retract/unretract pair inside the swap or at the tool-change boundary,
// while the travel retraction before the ramming and its unretraction stay, and an outgoing-only
// final unload gets no terminal retraction.
TEST_CASE("SEMM swap disables the retraction around it",
          "[plugins][wipe-tower][toolchange][semm][retraction]")
{
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool relative_e = GENERATE(false, true);
    const bool cooling_moves = GENERATE(true, false);
    CAPTURE(relative_e, cooling_moves);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_extruders(2);
    config.set_deserialize_strict({{"wipe_tower", "1"}, {"wipe_tower_x", "40"}, {"wipe_tower_y", "60"},
        {"wipe_tower_width", "20"}, {"wipe_tower_rotation_angle", "0"}, {"wipe_tower_brim_width", "0"},
        {"wiping_volumes_matrix", "0,8,8,0"}, {"filament_minimal_purge_on_wipe_tower", "2,2"},
        {"filament_diameter", "1.75,1.75"}, {"extrusion_multiplier", "1,1"},
        {"perimeter_extruder", "1"}, {"infill_extruder", "1"}, {"solid_infill_extruder", "1"},
        {"step_perimeter_plugin", "perimeter.generator.classic"}, {"step_surface_generation_plugin", "surface.initial_typed_surface_builder"},
        {"step_wipetower_plugin", "wipe_tower.manager.default"}, {"layer_height", "0.3"}, {"fill_density", "10%"},
        {"single_extruder_multi_material", "1"},
        {"use_relative_e_distances", relative_e ? "1" : "0"},
        {"filament_cooling_moves", cooling_moves ? "4,4" : "0,0"},
        {"filament_use_skinnydip", "0,0"}, {"filament_enable_toolchange_temp", "0,0"},
        {"high_current_on_filament_swap", "0"},
        {"retract_length", "2"}, {"retract_length_toolchange", "10"},
        // The work-line travel exceeds this threshold, so CreateRetraction would place a
        // scope boundary inside the swap without the disable_retraction modifier.
        {"retract_before_travel", "0.2,0.2"}});
    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20, Test::TestMesh::cube_20x20x20}, print, model, config);
    for (const char *key : {"perimeter_extruder", "infill_extruder", "solid_infill_extruder"})
        model.objects()[1].config.set_deserialize_strict(key, "2");
    print.apply(model, config);
    REQUIRE_NOTHROW(print.process());
    const boost::filesystem::path checkout = boost::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
    const std::string path = (checkout / "build" / "temp" /
        (std::string("semm_swap_retraction_") + (relative_e ? "rel" : "abs") +
         (cooling_moves ? "_cool" : "_nocool") + ".gcode")).string();
    GCodeProcessorResult result;
    REQUIRE_NOTHROW(Orchestrator::instance().export_gcode(print, path, &result));

    // Plan anchors: the first change's ramming end, its load end and its first purge point.
    const double load_distance =
        print.config().parking_pos_retraction.value + print.config().extra_loading_move.value;
    Point ramming_end{}, load_end{}, purge_start{};
    bool has_ramming_end = false;
    for (const Printing::PrintingGroup &group : print.printing_plan()->groups)
        for (const Printing::PrintingLayerGroup &layer : group.layers)
            for (const Printing::PrintingToolGroup &tool : layer.tool_groups)
                for (const Printing::PrintingExtrusion &extrusion : tool.extrusions) {
                    if (extrusion.root == nullptr || !extrusion.sregion_island_role.is_wipetower())
                        continue;
                    if (const ExtrusionEntity *ramming = last_leaf_with_raw_role(
                            *extrusion.root, uint16_t(RAW_EXTRUSION_ROLE_WIPE_TOWER_RAMMING))) {
                        if (!has_ramming_end) {
                            ramming_end = ramming->last_point();
                            has_ramming_end = true;
                        }
                    }
                    const std::vector<const ExtrusionEntity *> travelling =
                        travelling_filament_leaves(*extrusion.root);
                    if (travelling.empty() || load_distance <= 0. ||
                        std::abs(filament_delta_sum(*extrusion.root) - load_distance) > 1e-6)
                        continue;
                    load_end = travelling.back()->last_point();
                    coord_t z_offset = 0;
                    if (const ExtrusionEntity *purge = first_leaf_with_raw_role(
                            *extrusion.root, uint16_t(RAW_EXTRUSION_ROLE_WIPE_TOWER_WIPE), 0, z_offset))
                        purge_start = purge->first_point();
                }
    REQUIRE(has_ramming_end);
    REQUIRE(load_distance > 0.);

    const auto matches = [](const double x, const double y, const Point &point) {
        return std::abs(x - unscaled(point.x())) < 0.002 && std::abs(y - unscaled(point.y())) < 0.002;
    };
    const double expected_swap_e = -print.config().parking_pos_retraction.value;
    const int cooling = print.config().filament_cooling_moves.get_at(0);
    const size_t expected_swap_moves = 4 + 2 * size_t(cooling) + 1;
    const double last_pull = -15.;
    const std::string gcode = Test::GCode::read_text_file(path);
    GCodeReader reader;
    reader.apply_config(print.config());
    double swap_e = 0.;
    size_t swap_moves = 0;
    int swap_state = 0;  // 0 before the first ramming end, 1 inside the first swap, 2 done
    bool tower_marker = false;
    bool ramming_pair_ok = false;
    std::vector<double> history;
    int load_state = 0;  // 0 before the first load end, 1 between load and purge, 2 done
    bool load_armed = false;
    size_t moves_before_purge = 0;
    bool terminal_seen = false;
    double terminal_e = 0.;
    size_t terminal_moves = 0;
    reader.parse_buffer(gcode, [&](GCodeReader &state, const GCodeReader::GCodeLine &line) {
        if (line.comment() == "TYPE:Wipe tower") {
            tower_marker = true;
            return;
        }
        if (line.cmd_is("T0") || line.cmd_is("T1")) {
            // The first tool change closes the first swap and opens its load.
            if (swap_state == 1) {
                swap_state = 2;
                load_armed = true;
            }
            return;
        }
        if (!line.cmd_is("G0") && !line.cmd_is("G1"))
            return;
        if (!line.has(E))
            return;
        const double delta = double(line.dist_E(state));
        // The travel retraction before the ramming keeps its unretract pair: the tower role
        // marker sits between the retract and its unretract.
        if (tower_marker && delta > 0. && !history.empty() &&
            std::abs(history.back() + delta) < 1e-3)
            ramming_pair_ok = true;
        tower_marker = false;
        if (swap_state == 1) {
            swap_e += delta;
            ++swap_moves;
        }
        if (load_state == 1) {
            if (matches(state.x(), state.y(), purge_start))
                load_state = 2;
            else
                ++moves_before_purge;
        }
        // The last first-pull of the unload is the final unload's: accumulate from there to the
        // end of the file, where a terminal retraction would follow the parking move.
        if (terminal_seen) {
            terminal_e += delta;
            ++terminal_moves;
        }
        if (std::abs(delta - last_pull) < 1e-3) {
            terminal_seen = true;
            terminal_e = delta;
            terminal_moves = 1;
        }
        if (swap_state == 0 && matches(line.new_X(state), line.new_Y(state), ramming_end))
            swap_state = 1;
        // The load end point also lies on the outgoing work line, so the match is armed only
        // after the first tool change and only on a move with a coordinate: the in-place part
        // of the load stays at the work line start and would match otherwise.
        if (load_state == 0 && load_armed && (line.has(X) || line.has(Y)) &&
            matches(line.new_X(state), line.new_Y(state), load_end))
            load_state = 1;
        history.push_back(delta);
    });
    // No retraction inside the swap: the emitted E is exactly the unload, and no extra
    // retract/unretract pair is added.
    CHECK(swap_moves == expected_swap_moves);
    CHECK(swap_e == Approx(expected_swap_e).margin(0.01));
    // The travel retraction before the ramming keeps its unretract pair.
    CHECK(ramming_pair_ok);
    // No unretraction between the load and the first purge line.
    CHECK(moves_before_purge == 0);
    // The outgoing-only final unload is not followed by a terminal retraction.
    CHECK(terminal_moves == expected_swap_moves);
    CHECK(terminal_e == Approx(expected_swap_e).margin(0.01));
}

// Let the real manager add empty contour visits to an owned three-tool plan, requiring both forced-change purges, no first-visit purge and no outgoing source mutation.
TEST_CASE("Production wipe includes manager-forced contour visits and can be disabled", "[plugins][wipe-tower][toolchange]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    configure_print(print, model, 3);
    const_cast<PrintObjectConfig &>(print.default_object_config()).wipe_tower_extruder.value = 3;
    slic3r_api::PrintingPlan plan = Test::GCode::printing_plan_view(print);
    slic3r_api::PrintingGroup group = plan.append_group();
    append_layer(print, group, 0.2, {0, 1}, true);
    const bool enabled = GENERATE(false, true);
    Orchestrator &host = Orchestrator::instance();
    REQUIRE(host.set_plugin_active(plugin_id, enabled));
    try {
        Steps::StepGenerateWipeTower::run_step(host, print);
    } catch (...) {
        host.set_plugin_active(plugin_id, true);
        throw;
    }
    REQUIRE(host.set_plugin_active(plugin_id, true));
    const Printing::PrintingLayerGroup &layer = print.printing_plan()->groups[0].layers[0];
    REQUIRE(layer.tool_groups.size() == (enabled ? 3 : 2));
    size_t purges = 0;
    for (size_t t = 0; t < layer.tool_groups.size(); ++t)
        for (const Printing::PrintingExtrusion &extrusion : layer.tool_groups[t].extrusions)
            if (extrusion.root->child_count() != 0 && extrusion.root->child(0).role() == ExtrusionRole::WipeTowerWipe) {
                ++purges;
                CHECK(t > 0);
            }
    CHECK(purges == (enabled ? 2 : 0));
    CHECK(print.printing_plan()->object_instances.size() == (enabled ? 1 : 0));
}

// Inspect all declared dependencies after initialization, requiring service-owner tightening no later than the tower step and exact tower invalidation for purge-only options.
TEST_CASE("Tool-change wipe declares early invalidation for every option read", "[plugins][wipe-tower][toolchange]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Plugin *plugin = Orchestrator::instance().get_plugin(plugin_id);
    REQUIRE(plugin != nullptr);
    CHECK(plugin->get_step() == WIPE_TOWER_WIPE);
    CHECK(plugin->get_exclusive_group().empty());
    REQUIRE(plugin->get_used_config_keys().size() == 49);
    for (const Plugin::UsedConfigKey &key : plugin->get_used_config_keys()) {
        const ConfigOptionDef *definition = PrintConfigDef::instance().get(key.key);
        REQUIRE(definition != nullptr);
        CHECK(Steps::tightened_invalidation_step(definition->invalidates_step, STEP_WIPETOWER) == definition->invalidates_step);
        CHECK(definition->invalidates_step != STEP_NONE);
    }
    for (const char *key : {"wiping_volumes_matrix", "wiping_volumes_extruders", "filament_minimal_purge_on_wipe_tower", "filament_max_wipe_tower_speed"})
        CHECK(PrintConfigDef::instance().get(key)->invalidates_step == STEP_WIPETOWER);
}

// Slice an owned two-part box with overlapping tool regions and export its tower;
// require material from both tools, tower geometry and resolved connector speeds,
// leaving pipeline failures visible and restoring travel-plugin activation on exit.
TEST_CASE("Two overlapping model parts export both tools and a wipe tower", "[plugins][wipe-tower][mmu-parts]")
{
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    Plugin *travel = host.get_plugin("layer_extrusion_edit.travel.default");
    REQUIRE(travel != nullptr);
    const bool travel_was_active = host.active_plugins().count(travel) != 0;
    // Restore the borrowed plugin's activation even when an export assertion fails.
    BOOST_SCOPE_EXIT((&host)(travel)(travel_was_active)) {
        host.set_plugin_active(travel, travel_was_active);
    } BOOST_SCOPE_EXIT_END
    REQUIRE(host.set_plugin_active(travel, true));
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_extruders(2);
    config.set_deserialize_strict({{"nozzle_diameter", "0.4,0.4"}, {"single_extruder_multi_material", "0"},
        {"wipe_tower", "1"}, {"wipe_tower_x", "57.75"}, {"wipe_tower_y", "107.48"}, {"wipe_tower_width", "60"},
        {"wiping_volumes_matrix", "0,140,140,0"}, {"filament_minimal_purge_on_wipe_tower", "15,15"},
        {"wipe_tower_speed", "80"}, {"wipe_tower_wipe_starting_speed", "33%"}, {"first_layer_speed", "30"},
        {"first_layer_height", "75%"}, {"layer_height", "0.2"}, {"filament_diameter", "1.75,1.75"},
        {"extrusion_multiplier", "1,1"}, {"perimeter_extruder", "1"}, {"infill_extruder", "1"},
        {"solid_infill_extruder", "1"}, {"support_material_extruder", "0"},
        {"step_perimeter_plugin", "perimeter.generator.simple"},
        {"gcode_flavor", "marlin"}});
    Print print;
    Model model;
    ModelObject *object = model.add_object();
    object->name = "Shape-Box";
    object->config.set_deserialize_strict("extruder", "1");
    TriangleMesh first = make_cube(3., 3., 3.);
    ModelVolume *part = object->add_volume(std::move(first), ModelVolumeType::MODEL_PART, false);
    part->name = "Shape-Box";
    part->config.set_deserialize_strict("extruder", "0");
    TriangleMesh second = make_cube(3., 3., 3.);
    second.translate(Vec3f(2.0685f, -0.1366f, -1.5f));
    part = object->add_volume(std::move(second), ModelVolumeType::MODEL_PART, false);
    part->name = "Generic-Box";
    part->config.set_deserialize_strict("extruder", "2");
    object->add_instance();
    model.center_instances_around_point({83.69, 98.70});
    model.objects().front().ensure_on_bed();
    print.apply(model, config);
    print.set_status_silent();
    REQUIRE_NOTHROW(print.process());
    const boost::filesystem::path checkout = boost::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
    const std::string path = (checkout / "build" / "temp" / "mmu_parts_export.gcode").string();
    REQUIRE_NOTHROW(Orchestrator::instance().export_gcode(print, path, nullptr));
    double object_volume[2] = {0., 0.};
    double tower_volume = 0.;
    size_t tower_travels = 0;
    for (const Printing::PrintingGroup &group : print.printing_plan()->groups)
        for (const Printing::PrintingLayerGroup &layer : group.layers)
            for (const Printing::PrintingToolGroup &tool : layer.tool_groups)
                for (const Printing::PrintingExtrusion &extrusion : tool.extrusions) {
                    if (!extrusion.root)
                        continue;
                    require_movement_speeds(*extrusion.root, -1.f, extrusion.sregion_island_role.is_wipetower(),
                                            tower_travels, float(print.config().travel_speed.value));
                    if (extrusion.sregion_island_role.is_wipetower())
                        tower_volume += raw_role_volume(*extrusion.root, RAW_EXTRUSION_ROLE_WIPE_TOWER_WIPE);
                    else if (tool.extruder_id < 2)
                        object_volume[tool.extruder_id] += raw_role_volume(*extrusion.root, RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
                }
    CHECK(object_volume[0] > 0.);
    CHECK(object_volume[1] > 0.);
    CHECK(tower_volume > 0.);
    CHECK(tower_travels > 0);
}

// The user-visible requirement on the mmu.3mf-equivalent print: in the tower layer with the
// largest used depth, at most 10 % of that depth is sparse. The tower rectangle and the purge
// bands are measured from the final plan geometry in platter Y; the last layer of a tie is
// used, so the first layer's brim never enlarges the rectangle.
TEST_CASE("Two overlapping model parts keep the tower sparse share below ten percent",
          "[plugins][wipe-tower][sparse-share]")
{
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    Plugin *travel = host.get_plugin("layer_extrusion_edit.travel.default");
    REQUIRE(travel != nullptr);
    const bool travel_was_active = host.active_plugins().count(travel) != 0;
    BOOST_SCOPE_EXIT((&host)(travel)(travel_was_active)) {
        host.set_plugin_active(travel, travel_was_active);
    } BOOST_SCOPE_EXIT_END
    REQUIRE(host.set_plugin_active(travel, true));
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_extruders(2);
    config.set_deserialize_strict({{"nozzle_diameter", "0.4,0.4"}, {"single_extruder_multi_material", "0"},
        {"wipe_tower", "1"}, {"wipe_tower_x", "57.75"}, {"wipe_tower_y", "107.48"}, {"wipe_tower_width", "60"},
        {"wiping_volumes_matrix", "0,140,140,0"}, {"filament_minimal_purge_on_wipe_tower", "15,15"},
        {"wipe_tower_speed", "80"}, {"wipe_tower_wipe_starting_speed", "33%"}, {"first_layer_speed", "30"},
        {"first_layer_height", "75%"}, {"layer_height", "0.2"}, {"filament_diameter", "1.75,1.75"},
        {"extrusion_multiplier", "1,1"}, {"perimeter_extruder", "1"}, {"infill_extruder", "1"},
        {"solid_infill_extruder", "1"}, {"support_material_extruder", "0"},
        {"step_perimeter_plugin", "perimeter.generator.simple"},
        {"gcode_flavor", "marlin"}});
    Print print;
    Model model;
    ModelObject *object = model.add_object();
    object->name = "Shape-Box";
    object->config.set_deserialize_strict("extruder", "1");
    TriangleMesh first = make_cube(3., 3., 3.);
    ModelVolume *part = object->add_volume(std::move(first), ModelVolumeType::MODEL_PART, false);
    part->name = "Shape-Box";
    part->config.set_deserialize_strict("extruder", "0");
    TriangleMesh second = make_cube(3., 3., 3.);
    second.translate(Vec3f(2.0685f, -0.1366f, -1.5f));
    part = object->add_volume(std::move(second), ModelVolumeType::MODEL_PART, false);
    part->name = "Generic-Box";
    part->config.set_deserialize_strict("extruder", "2");
    object->add_instance();
    model.center_instances_around_point({83.69, 98.70});
    model.objects().front().ensure_on_bed();
    print.apply(model, config);
    print.set_status_silent();
    REQUIRE_NOTHROW(print.process());
    const boost::filesystem::path checkout =
        boost::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
    const std::string path =
        (checkout / "build" / "temp" / "mmu_parts_sparse_share.gcode").string();
    REQUIRE_NOTHROW(Orchestrator::instance().export_gcode(print, path, nullptr));

    // One measure per tower layer: the used depth from its purge and ramming bands, the
    // rectangle depth from all its tower leaves, and its number of purge bands.
    struct LayerMeasure { size_t index; double used; double depth; size_t bands; };
    std::vector<LayerMeasure> measures;
    size_t layer_index = 0;
    for (const Printing::PrintingGroup &group : print.printing_plan()->groups)
        for (const Printing::PrintingLayerGroup &layer : group.layers) {
            double min_y = (std::numeric_limits<double>::max)();
            double max_y = -min_y;
            double purge_max = -min_y;
            size_t bands = 0;
            for (const Printing::PrintingToolGroup &tool : layer.tool_groups)
                for (const Printing::PrintingExtrusion &extrusion : tool.extrusions) {
                    if (!extrusion.root || !extrusion.sregion_island_role.is_wipetower())
                        continue;
                    tower_leaf_y_extent(*extrusion.root, min_y, max_y, purge_max);
                    coord_t z_offset = 0;
                    if (first_leaf_with_raw_role(*extrusion.root,
                            uint16_t(RAW_EXTRUSION_ROLE_WIPE_TOWER_WIPE), 0, z_offset) != nullptr)
                        ++bands;
                }
            if (max_y > min_y && purge_max > min_y)
                measures.push_back({layer_index, purge_max - min_y, max_y - min_y, bands});
            ++layer_index;
        }
    REQUIRE_FALSE(measures.empty());
    const LayerMeasure *most_used = nullptr;
    for (const LayerMeasure &measure : measures)
        if (most_used == nullptr || measure.used >= most_used->used)
            most_used = &measure;
    REQUIRE(most_used != nullptr);
    REQUIRE(most_used->depth > 0.);
    CAPTURE(most_used->index, most_used->used, most_used->depth, most_used->bands, measures.size());
    CHECK((most_used->depth - most_used->used) / most_used->depth <= 0.1);
}

// Slice two owned tool-specific cubes and export through real plugins in shared/restarting-Z mode, requiring one selection per real change and every sampled purge at its local tower row's platter Z.
TEST_CASE("Production two-nozzle wipe exports tool changes and tower offsets", "[plugins][wipe-tower][toolchange]")
{
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool parallel = GENERATE(false, true);
    const bool semm = GENERATE(false, true);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_extruders(2);
    config.set_deserialize_strict({{"wipe_tower", "1"}, {"wipe_tower_x", "40"}, {"wipe_tower_y", "60"},
        {"wipe_tower_width", "20"}, {"wipe_tower_rotation_angle", "0"}, {"wipe_tower_brim_width", "0"},
        {"wiping_volumes_matrix", "0,8,8,0"}, {"filament_minimal_purge_on_wipe_tower", "2,2"},
        {"filament_diameter", "1.75,1.75"}, {"extrusion_multiplier", "1,1"},
        {"perimeter_extruder", "1"}, {"infill_extruder", "1"}, {"solid_infill_extruder", "1"},
        {"step_perimeter_plugin", "perimeter.generator.classic"}, {"step_surface_generation_plugin", "surface.initial_typed_surface_builder"},
        {"step_wipetower_plugin", "wipe_tower.manager.default"}, {"layer_height", "0.3"}, {"fill_density", "10%"},
        {"parallel_objects_step", parallel ? "5" : "0"}, {"gcode_comments", "1"},
        {"single_extruder_multi_material", semm ? "1" : "0"}});
    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20, Test::TestMesh::cube_20x20x20}, print, model, config);
    for (const char *key : {"perimeter_extruder", "infill_extruder", "solid_infill_extruder"})
        model.objects()[1].config.set_deserialize_strict(key, "2");
    print.apply(model, config);
    REQUIRE_NOTHROW(print.process());
    const boost::filesystem::path checkout = boost::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
    const std::string path = (checkout / "build" / "temp" /
        (std::string("tool_change_wipe_export") + (parallel ? "_parallel" : "") + (semm ? "_semm" : "") + ".gcode")).string();
    REQUIRE_NOTHROW(Orchestrator::instance().export_gcode(print, path, nullptr));
    size_t changes = 0;
    size_t purge_roots = 0;
    size_t elevated = 0;
    extruder_index_t previous = EXTRUDER_ID_UNKNOWN;
    struct Sample { Point from; Point to; double z; size_t seen; };
    std::vector<Sample> samples;
    for (const Printing::PrintingGroup &group : print.printing_plan()->groups)
        for (const Printing::PrintingLayerGroup &layer : group.layers)
            for (const Printing::PrintingToolGroup &tool : layer.tool_groups) {
                if (previous != EXTRUDER_ID_UNKNOWN && previous != tool.extruder_id)
                    ++changes;
                previous = tool.extruder_id;
                for (const Printing::PrintingExtrusion &extrusion : tool.extrusions) {
                    if (!extrusion.root || !extrusion.sregion_island_role.is_wipetower())
                        continue;
                    coord_t z_offset = 0;
                    const ExtrusionEntity *purge = first_leaf_with_raw_role(*extrusion.root,
                        uint16_t(RAW_EXTRUSION_ROLE_WIPE_TOWER_WIPE), 0, z_offset);
                    if (purge == nullptr)
                        continue;
                    ++purge_roots;
                    const double z = unscaled(layer.print_z + z_offset);
                    if (z_offset != 0)
                        ++elevated;
                    const Polyline line = purge->as_polyline().to_polyline();
                    samples.push_back({line.points[line.points.size() - 2], line.points.back(), z, 0});
                }
            }
    REQUIRE(changes > 0);
    CHECK(purge_roots == changes);
    if (parallel)
        CHECK(elevated > 0);
    size_t selections = 0;
    const std::string gcode = Test::GCode::read_text_file(path);
    GCodeReader reader;
    // The start sequence selects the tools before the first layer; only later selections are changes.
    bool printing = false;
    reader.parse_buffer(gcode, [&samples, &selections, &printing](GCodeReader &state, const GCodeReader::GCodeLine &line) {
        if (line.comment().find("LAYER_CHANGE") != std::string::npos)
            printing = true;
        if (printing && (line.cmd_is("T0") || line.cmd_is("T1")))
            ++selections;
        if (!line.extruding(state) || line.dist_X(state) == 0.)
            return;
        for (Sample &sample : samples)
            if (std::abs(state.x() - unscaled(sample.from.x())) < 0.002 &&
                std::abs(state.y() - unscaled(sample.from.y())) < 0.002 &&
                std::abs(line.new_X(state) - unscaled(sample.to.x())) < 0.002 &&
                std::abs(line.new_Y(state) - unscaled(sample.to.y())) < 0.002 &&
                std::abs(state.z() - sample.z) < 0.002)
                ++sample.seen;
    });
    CHECK(selections == changes);
    for (const Sample &sample : samples)
        CHECK(sample.seen > 0);
}

// Slice two cubes printed by different tools, export the same plan on Marlin 2 and Klipper, and
// check the manager's speed override in the M220 lines around every tower tool change: Marlin
// backs the override up before forcing 100% and restores it once, Klipper forces 100% on both sides.
TEST_CASE("Tower tool changes export the manager speed override", "[plugins][wipe-tower][toolchange]")
{
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool semm = GENERATE(false, true);
    CAPTURE(semm);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_extruders(2);
    config.set_deserialize_strict({{"wipe_tower", "1"}, {"wipe_tower_x", "40"}, {"wipe_tower_y", "60"},
        {"wipe_tower_width", "20"}, {"wipe_tower_rotation_angle", "0"}, {"wipe_tower_brim_width", "0"},
        {"wiping_volumes_matrix", "0,8,8,0"}, {"filament_minimal_purge_on_wipe_tower", "2,2"},
        {"filament_diameter", "1.75,1.75"}, {"extrusion_multiplier", "1,1"},
        {"perimeter_extruder", "1"}, {"infill_extruder", "1"}, {"solid_infill_extruder", "1"},
        {"step_perimeter_plugin", "perimeter.generator.classic"}, {"step_surface_generation_plugin", "surface.initial_typed_surface_builder"},
        {"step_wipetower_plugin", "wipe_tower.manager.default"}, {"layer_height", "0.3"}, {"fill_density", "10%"},
        {"single_extruder_multi_material", semm ? "1" : "0"}});
    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20, Test::TestMesh::cube_20x20x20}, print, model, config);
    for (const char *key : {"perimeter_extruder", "infill_extruder", "solid_infill_extruder"})
        model.objects()[1].config.set_deserialize_strict(key, "2");
    print.apply(model, config);
    REQUIRE_NOTHROW(print.process());
    const boost::filesystem::path checkout = boost::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
    // Keep only the speed override and tool selection lines so their bracket structure is visible.
    const auto selection_lines = [](const std::string &gcode) {
        std::vector<std::string> lines;
        GCodeReader reader;
        reader.parse_buffer(gcode, [&lines](GCodeReader &, const GCodeReader::GCodeLine &line) {
            // Klipper selects a multi-nozzle tool with ACTIVATE_EXTRUDER; other dialects use T.
            if (line.cmd_is("M220") || line.cmd_is("T0") || line.cmd_is("T1") ||
                line.cmd_is("ACTIVATE_EXTRUDER"))
                lines.emplace_back(line.raw());
        });
        return lines;
    };
    Test::GCode::select_printing_plan_firmware(print, "gcode.firmware.marlin2");
    const std::string marlin_path = (checkout / "build" / "temp" /
        (std::string("tower_speed_override") + (semm ? "_semm" : "") + ".gcode")).string();
    REQUIRE_NOTHROW(Orchestrator::instance().export_gcode(print, marlin_path, nullptr));
    // Export ran the ordering and wipe-tower steps, so the printing plan now exists; every change
    // between consecutive visits is a tower tool change.
    size_t changes = 0;
    extruder_index_t previous = EXTRUDER_ID_UNKNOWN;
    for (const Printing::PrintingGroup &group : print.printing_plan()->groups)
        for (const Printing::PrintingLayerGroup &layer : group.layers)
            for (const Printing::PrintingToolGroup &tool : layer.tool_groups) {
                if (previous != EXTRUDER_ID_UNKNOWN && previous != tool.extruder_id)
                    ++changes;
                previous = tool.extruder_id;
            }
    REQUIRE(changes > 0);
    const std::vector<std::string> marlin = selection_lines(Test::GCode::read_text_file(marlin_path));
    size_t saves = 0, restores = 0, open = 0, covered = 0;
    bool nested = false;
    std::string last_m220;
    for (const std::string &line : marlin) {
        if (line == "M220 B") {
            ++saves;
            nested = nested || open != 0;
            ++open;
            last_m220 = line;
        } else if (line == "M220 R") {
            ++restores;
            nested = nested || open != 1;
            --open;
            last_m220 = line;
        } else if (line == "M220 S100") {
            last_m220 = line;
        } else if ((line == "T0" || line == "T1") && open == 1) {
            ++covered;
            CHECK(last_m220 == "M220 S100");
        }
    }
    CHECK(saves > 0);
    CHECK(saves == restores);
    CHECK(open == 0);
    CHECK_FALSE(nested);
    CHECK(covered == changes);
    Test::GCode::select_printing_plan_firmware(print, "gcode.firmware.klipper");
    const std::string klipper_path = (checkout / "build" / "temp" /
        (std::string("tower_speed_override_klipper") + (semm ? "_semm" : "") + ".gcode")).string();
    REQUIRE_NOTHROW(Orchestrator::instance().export_gcode(print, klipper_path, nullptr));
    const std::vector<std::string> klipper = selection_lines(Test::GCode::read_text_file(klipper_path));
    size_t klipper_covered = 0;
    for (size_t i = 0; i < klipper.size(); ++i) {
        const bool selection = klipper[i] == "T0" || klipper[i] == "T1" ||
            klipper[i].rfind("ACTIVATE_EXTRUDER", 0) == 0;
        if (selection && i > 0 && i + 1 < klipper.size() &&
            klipper[i - 1] == "M220 S100" && klipper[i + 1] == "M220 S100")
            ++klipper_covered;
    }
    CHECK(klipper_covered == changes);
}

// Slice a short tool-2 cube below a tall tool-1 cube and export the SEMM print: the tower stops at
// the last tool change and the floating final unload is laid at the top Z over the tower footprint,
// with no tower layer between them.
TEST_CASE("SEMM final unload floats above a tower that stopped lower", "[plugins][wipe-tower][toolchange][semm]")
{
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_extruders(2);
    config.set_deserialize_strict({{"wipe_tower", "1"}, {"wipe_tower_x", "40"}, {"wipe_tower_y", "60"},
        {"wipe_tower_width", "20"}, {"wipe_tower_rotation_angle", "0"}, {"wipe_tower_brim_width", "0"},
        {"wiping_volumes_matrix", "0,8,8,0"}, {"filament_minimal_purge_on_wipe_tower", "2,2"},
        {"filament_diameter", "1.75,1.75"}, {"extrusion_multiplier", "1,1"},
        {"perimeter_extruder", "1"}, {"infill_extruder", "1"}, {"solid_infill_extruder", "1"},
        {"step_perimeter_plugin", "perimeter.generator.classic"}, {"step_surface_generation_plugin", "surface.initial_typed_surface_builder"},
        {"step_wipetower_plugin", "wipe_tower.manager.default"}, {"layer_height", "0.3"}, {"fill_density", "10%"},
        {"single_extruder_multi_material", "1"}});
    Print print;
    Model model;
    Test::init_print({make_cube(20., 20., 10.), make_cube(20., 20., 20.)}, print, model, config);
    for (const char *key : {"perimeter_extruder", "infill_extruder", "solid_infill_extruder"})
        model.objects()[0].config.set_deserialize_strict(key, "2");
    print.apply(model, config);
    REQUIRE_NOTHROW(print.process());
    const boost::filesystem::path checkout = boost::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
    const std::string path = (checkout / "build" / "temp" / "semm_final_unload_float.gcode").string();
    REQUIRE_NOTHROW(Orchestrator::instance().export_gcode(print, path, nullptr));
    // Find the last tool change and the top of the print in the ordered plan.
    double last_change_z = 0.;
    double top_z = 0.;
    extruder_index_t previous = EXTRUDER_ID_UNKNOWN;
    for (const Printing::PrintingGroup &group : print.printing_plan()->groups)
        for (const Printing::PrintingLayerGroup &layer : group.layers)
            for (const Printing::PrintingToolGroup &tool : layer.tool_groups) {
                if (previous != EXTRUDER_ID_UNKNOWN && previous != tool.extruder_id)
                    last_change_z = std::max(last_change_z, unscaled(layer.print_z));
                previous = tool.extruder_id;
                top_z = std::max(top_z, unscaled(layer.print_z));
            }
    REQUIRE(top_z > last_change_z + 1.);
    // Above the last change only the floating final unload remains; its ramming line is sampled so
    // the export can be checked at that Z inside the tower footprint.
    struct Sample { Point from; Point to; double z; size_t seen; };
    std::vector<Sample> samples;
    size_t above = 0;
    for (const Printing::PrintingGroup &group : print.printing_plan()->groups)
        for (const Printing::PrintingLayerGroup &layer : group.layers)
            for (const Printing::PrintingToolGroup &tool : layer.tool_groups)
                for (const Printing::PrintingExtrusion &extrusion : tool.extrusions) {
                    if (!extrusion.root || !extrusion.sregion_island_role.is_wipetower())
                        continue;
                    coord_t z_offset = 0;
                    const ExtrusionEntity *ramming = first_leaf_with_raw_role(*extrusion.root,
                        uint16_t(RAW_EXTRUSION_ROLE_WIPE_TOWER_RAMMING), 0, z_offset);
                    const double z = unscaled(layer.print_z + z_offset);
                    if (z <= last_change_z + 0.01)
                        continue;
                    ++above;
                    CHECK(z == Approx(top_z));
                    if (ramming == nullptr)
                        continue;
                    const Polyline line = ramming->as_polyline().to_polyline();
                    REQUIRE(line.points.size() >= 2);
                    samples.push_back({line.points[0], line.points[1], z, 0});
                }
    CHECK(above == 1);
    REQUIRE(samples.size() == 1);
    const std::string gcode = Test::GCode::read_text_file(path);
    GCodeReader reader;
    reader.parse_buffer(gcode, [&samples](GCodeReader &state, const GCodeReader::GCodeLine &line) {
        if (!line.extruding(state) || line.dist_X(state) == 0.)
            return;
        for (Sample &sample : samples)
            if (std::abs(state.x() - unscaled(sample.from.x())) < 0.002 &&
                std::abs(state.y() - unscaled(sample.from.y())) < 0.002 &&
                std::abs(line.new_X(state) - unscaled(sample.to.x())) < 0.002 &&
                std::abs(line.new_Y(state) - unscaled(sample.to.y())) < 0.002 &&
                std::abs(state.z() - sample.z) < 0.002)
                ++sample.seen;
    });
    for (const Sample &sample : samples)
        CHECK(sample.seen > 0);
}

// Slice one cube whose sparse infill is printed by tool 2 while its perimeters and solid infill use tool 1:
// the sparse infill is then the first geometry of tool 2's visit, so wipe_into_infill removes its volume
// from the tower purge and the reader accounts less tower material. Nothing is moved or reassigned.
TEST_CASE("Wipe into infill measures the sparse infill printed after the change",
          "[plugins][wipe-tower][toolchange][wipe-into]")
{
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    double purged[2] = {0., 0.};
    double tower_volume[2] = {0., 0.};
    size_t changes[2] = {0, 0};
    size_t extrusions[2] = {0, 0};
    for (int into = 0; into < 2; ++into) {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.set_num_extruders(2);
        config.set_deserialize_strict({{"wipe_tower", "1"}, {"wipe_tower_x", "40"}, {"wipe_tower_y", "60"},
            {"wipe_tower_width", "20"}, {"wipe_tower_rotation_angle", "0"}, {"wipe_tower_brim_width", "0"},
            {"wiping_volumes_matrix", "0,140,140,0"}, {"filament_minimal_purge_on_wipe_tower", "15,15"},
            {"filament_diameter", "1.75,1.75"}, {"extrusion_multiplier", "1,1"},
            {"perimeter_extruder", "1"}, {"infill_extruder", "2"}, {"solid_infill_extruder", "1"},
            {"step_perimeter_plugin", "perimeter.generator.classic"}, {"step_surface_generation_plugin", "surface.initial_typed_surface_builder"},
            {"step_wipetower_plugin", "wipe_tower.manager.default"}, {"layer_height", "0.3"}, {"fill_density", "20%"},
            {"wipe_into_infill", into != 0 ? "1" : "0"}});
        Print print;
        Model model;
        Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
        print.apply(model, config);
        REQUIRE_NOTHROW(print.process());
        const boost::filesystem::path checkout = boost::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
        const std::string path = (checkout / "build" / "temp" /
            (into != 0 ? "wipe_into_infill.gcode" : "wipe_into_infill_off.gcode")).string();
        GCodeProcessorResult result;
        REQUIRE_NOTHROW(Orchestrator::instance().export_gcode(print, path, &result));
        extruder_index_t previous = EXTRUDER_ID_UNKNOWN;
        for (const Printing::PrintingGroup &group : print.printing_plan()->groups)
            for (const Printing::PrintingLayerGroup &layer : group.layers)
                for (const Printing::PrintingToolGroup &tool : layer.tool_groups) {
                    if (previous != EXTRUDER_ID_UNKNOWN && previous != tool.extruder_id)
                        ++changes[into];
                    previous = tool.extruder_id;
                    for (const Printing::PrintingExtrusion &extrusion : tool.extrusions) {
                        if (!extrusion.root)
                            continue;
                        ++extrusions[into];
                        if (extrusion.sregion_island_role.is_wipetower())
                            purged[into] += raw_role_volume(*extrusion.root, uint16_t(RAW_EXTRUSION_ROLE_WIPE_TOWER_WIPE));
                    }
                }
        const std::map<GCodeExtrusionRole, std::map<extruder_index_t, double>>::const_iterator wipe =
            result.print_statistics.volumes_per_role_per_extruder.find(GCodeExtrusionRole::WipeTower);
        if (wipe != result.print_statistics.volumes_per_role_per_extruder.end())
            for (const std::pair<const extruder_index_t, double> &entry : wipe->second)
                tower_volume[into] += entry.second;
    }
    REQUIRE(changes[0] > 0);
    CHECK(changes[1] == changes[0]);
    // The same visits and extrusions with and without the option: nothing is moved or reassigned.
    CHECK(extrusions[1] == extrusions[0]);
    CHECK(purged[0] >= 140. * double(changes[0]) - 1e-3);
    CHECK(purged[1] < purged[0] - 1.);
    CHECK(purged[1] >= 15. * double(changes[1]) - 1e-3);
    CHECK(tower_volume[1] < tower_volume[0] - 1.);
}

// Measure the eligible geometry at the start of the incoming visit on hand-built plans: the object and
// region options, the sparse-infill role, the consecutive prefix, the soluble guard and the
// perimeters-first rule decide the volume removed from the tower purge, and the plan is not changed.
TEST_CASE("Wipe into infill and objects measures the printed prefix",
          "[plugins][wipe-tower][toolchange][wipe-into]")
{
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    // Slice two small objects once to own real region islands, then build the measured plans by hand.
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_extruders(2);
    config.set_deserialize_strict({{"layer_height", "0.3"}, {"fill_density", "20%"},
        {"perimeter_extruder", "1"}, {"infill_extruder", "1"}, {"solid_infill_extruder", "1"},
        {"step_perimeter_plugin", "perimeter.generator.classic"},
        {"step_surface_generation_plugin", "surface.initial_typed_surface_builder"}});
    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20, Test::TestMesh::cube_20x20x20}, print, model, config);
    print.apply(model, config);
    REQUIRE_NOTHROW(print.process());
    PrintConfig &print_config = const_cast<PrintConfig &>(print.config());
    print_config.wiping_volumes_matrix.set(std::vector<double>{0., 100., 100., 0.});
    print_config.filament_minimal_purge_on_wipe_tower.set(std::vector<double>{10., 10.});
    const LayerRegionIsland &island0 = print.object(0).layer(0).islands().front().regions_islands().front();
    const LayerRegionIsland &island1 = print.object(1).layer(0).islands().front().regions_islands().front();
    // One leaf of a hand-built plan: a role, a length in mm and the region island it belongs to. The
    // path helper gives every leaf a 0.08 mm3/mm flow, so a leaf volume is length * 0.08.
    struct Leaf { ExtrusionRole role; double length; const LayerRegionIsland *island; };
    const double flow = 0.08;
    // Rebuild the plan with the given visits and their leaves, in print order.
    const auto build_plan = [&](const std::vector<std::pair<extruder_index_t, std::vector<Leaf>>> &visits) {
        Printing::PrintingPlan &plan = print.mutable_printing_plan();
        plan.groups.clear();
        plan.groups.emplace_back();
        Printing::PrintingGroup &group = plan.groups.front();
        group.layers.clear();
        group.layers.emplace_back();
        Printing::PrintingLayerGroup &layer = group.layers.front();
        layer.print_z = scale_i(0.3);
        layer.tool_groups.clear();
        for (const std::pair<const extruder_index_t, std::vector<Leaf>> &visit_leaves : visits) {
            layer.tool_groups.emplace_back();
            Printing::PrintingToolGroup &visit = layer.tool_groups.back();
            visit.extruder_id = visit_leaves.first;
            visit.extrusions.clear();
            for (const Leaf &leaf : visit_leaves.second) {
                Printing::PrintingExtrusion extrusion;
                extrusion.root = Test::GCode::make_firmware_path(
                    ArcPolyline(Points{Point::new_scale(0., 0.), Point::new_scale(leaf.length, 0.)}),
                    20.f, 500.f, -1.f, -1.f, -1.f, leaf.role);
                extrusion.sregion_island_role = ExtrusionRole::Perimeter;
                extrusion.region_island = leaf.island;
                visit.extrusions.push_back(std::move(extrusion));
            }
        }
    };
    // Set the region options of one island, with the perimeter extruder as the user configures it.
    const auto set_region = [](const LayerRegionIsland &island, const bool into_infill, const bool infill_first,
                               const int perimeter_extruder) {
        PrintRegionConfig &region =
            const_cast<PrintRegionConfig &>((*island.regions().begin())->region().config());
        region.wipe_into_infill.value = into_infill;
        region.infill_first.value = infill_first;
        region.perimeter_extruder.value = perimeter_extruder;
    };
    const auto set_object = [](const LayerRegionIsland &island, const bool into_objects) {
        const_cast<PrintObjectConfig &>(
            (*island.regions().begin())->layer()->object()->config()).wipe_into_objects.value = into_objects;
    };
    // The purge of the first change of a plan, with the matrix above.
    const auto first_purge = [&](const std::vector<c_wipe_tower_request> &requests) {
        REQUIRE_FALSE(requests.empty());
        return requests[0].purge_volume;
    };
    const std::vector<Leaf> perimeter_leaf{{ExtrusionRole::Perimeter, 10., &island0}};
    const std::vector<Leaf> sparse_leaf{{ExtrusionRole::InternalInfill, 10., &island0}};
    set_object(island0, false);
    set_object(island1, false);
    set_region(island0, false, false, 1);
    set_region(island1, false, false, 1);
    // The consecutive sparse infill at the start of the incoming visit is removed from the purge,
    // whole leaves only, up to the incoming minimum purge.
    build_plan({{0, perimeter_leaf}, {1, {{ExtrusionRole::InternalInfill, 10., &island0},
                                          {ExtrusionRole::InternalInfill, 20., &island0},
                                          {ExtrusionRole::Perimeter, 10., &island0}}}});
    set_region(island0, true, false, 1);
    CHECK(first_purge(collect(print)) == Approx(100. - 30. * flow));
    // A first leaf that is solid infill or a perimeter is not eligible: nothing is removed.
    build_plan({{0, perimeter_leaf}, {1, {{ExtrusionRole::SolidInfill, 10., &island0},
                                          {ExtrusionRole::InternalInfill, 20., &island0}}}});
    CHECK(first_purge(collect(print)) == Approx(100.));
    build_plan({{0, perimeter_leaf}, {1, {{ExtrusionRole::Perimeter, 10., &island0},
                                          {ExtrusionRole::InternalInfill, 20., &island0}}}});
    CHECK(first_purge(collect(print)) == Approx(100.));
    // The measured volume is capped by the purge above the incoming minimum purge.
    build_plan({{0, perimeter_leaf}, {1, {{ExtrusionRole::InternalInfill, 2000., &island0}}}});
    CHECK(first_purge(collect(print)) == Approx(10.));
    // A soluble outgoing or incoming filament is never wiped into another object.
    build_plan({{0, perimeter_leaf}, {1, sparse_leaf}});
    CHECK(first_purge(collect(print)) == Approx(100. - 10. * flow));
    print_config.filament_soluble.set(std::vector<unsigned char>{1, 0});
    CHECK(first_purge(collect(print)) == Approx(100.));
    print_config.filament_soluble.set(std::vector<unsigned char>{0, 1});
    CHECK(first_purge(collect(print)) == Approx(100.));
    print_config.filament_soluble.set(std::vector<unsigned char>{0, 0});
    // wipe_into_objects accepts every leaf of the object, up to the first leaf of an object without
    // the option; a leaf without a region island (support) is never eligible.
    build_plan({{0, perimeter_leaf}, {1, {{ExtrusionRole::Perimeter, 10., &island0},
                                          {ExtrusionRole::SolidInfill, 20., &island0},
                                          {ExtrusionRole::InternalInfill, 10., &island1}}}});
    set_region(island0, false, false, 1);
    set_object(island0, true);
    CHECK(first_purge(collect(print)) == Approx(100. - 30. * flow));
    build_plan({{0, perimeter_leaf}, {1, {{ExtrusionRole::Perimeter, 10., &island0},
                                          {ExtrusionRole::InternalInfill, 10., nullptr},
                                          {ExtrusionRole::InternalInfill, 10., &island0}}}});
    CHECK(first_purge(collect(print)) == Approx(100. - 10. * flow));
    set_object(island0, false);
    // Perimeters first: with wipe_into_infill alone and no infill_first, the sparse infill counts
    // only when the island's perimeter tool is the incoming tool or prints earlier in the layer.
    const std::vector<Leaf> sparse_10{{ExtrusionRole::InternalInfill, 10., &island0}};
    set_region(island0, true, false, 3);                        // perimeter tool 3, after the incoming tool 2
    build_plan({{0, perimeter_leaf}, {1, sparse_10}, {2, perimeter_leaf}});
    CHECK(first_purge(collect(print)) == Approx(100.));
    set_region(island0, true, false, 1);                        // perimeter tool 1, before the incoming tool 2
    build_plan({{0, perimeter_leaf}, {1, sparse_10}});
    CHECK(first_purge(collect(print)) == Approx(100. - 10. * flow));
    set_region(island0, true, false, 2);                        // perimeter tool 2 is the incoming tool
    build_plan({{0, perimeter_leaf}, {1, sparse_10}});
    CHECK(first_purge(collect(print)) == Approx(100. - 10. * flow));
    set_region(island0, true, true, 1);                         // infill_first skips the rule
    build_plan({{0, perimeter_leaf}, {1, sparse_10}});
    CHECK(first_purge(collect(print)) == Approx(100. - 10. * flow));
    // Measuring changes no visit and no extrusion.
    set_region(island0, true, false, 1);
    build_plan({{0, perimeter_leaf}, {1, sparse_10}});
    size_t visits = 0, extrusions = 0;
    for (const Printing::PrintingGroup &group : print.printing_plan()->groups)
        for (const Printing::PrintingLayerGroup &layer : group.layers) {
            visits += layer.tool_groups.size();
            for (const Printing::PrintingToolGroup &tool : layer.tool_groups)
                extrusions += tool.extrusions.size();
        }
    REQUIRE(visits == 2);
    REQUIRE(extrusions == 2);
    CHECK(first_purge(collect(print)) == Approx(100. - 10. * flow));
    size_t visits_after = 0, extrusions_after = 0;
    for (const Printing::PrintingGroup &group : print.printing_plan()->groups)
        for (const Printing::PrintingLayerGroup &layer : group.layers) {
            visits_after += layer.tool_groups.size();
            for (const Printing::PrintingToolGroup &tool : layer.tool_groups)
                extrusions_after += tool.extrusions.size();
        }
    CHECK(visits_after == visits);
    CHECK(extrusions_after == extrusions);
}

// Convert owned synthetic reader totals with borrowed config, checking normal
// time, per-tool properties and tower shares. Unrelated area/export metadata
// remains intact, and a second fill replaces rather than adds to measured totals.
TEST_CASE("Reader statistics fill preserves source areas and export metadata",
          "[plugins][gcode][wipe-tower][print-statistics]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    PrintConfig config;
    config.filament_diameter.set(std::vector<double>{1.75, 2.85});
    config.filament_density.set(std::vector<double>{1.1, 1.3});
    config.filament_cost.set(std::vector<double>{20.0, 30.0});
    config.time_cost.value = 2.0;
    const double area0 = PI * std::pow(1.75 * 0.5, 2.0);
    const double area1 = PI * std::pow(2.85 * 0.5, 2.0);
    GCodeProcessorResult result;
    result.print_statistics.modes[0].time = 3600.f;
    result.print_statistics.volumes_per_extruder = {{0, 10.0 * area0}, {1, 20.0 * area1}};
    result.print_statistics.volumes_per_role_per_extruder[GCodeExtrusionRole::WipeTower] =
        {{0, area0}, {1, 2.0 * area1}};
    PrintStatistics stats;
    stats._layer_area_stats = {{scale_i(0.2), 12.f}};
    stats.initial_extruder_id = 1;
    stats.total_toolchanges = 4;
    stats.printing_extruders = {0, 1};
    stats.object_layer_count = 7;
    stats.total_layer_count = 9;
    stats.is_computing_gcode = true;
    stats.estimated_print_time[1] = 123.0;
    const double expected_mass = 10.0 * area0 * 1.1 * 0.001 + 20.0 * area1 * 1.3 * 0.001;
    const double expected_cost = 2.0 + 10.0 * area0 * 1.1 * 20.0 * 1e-6 + 20.0 * area1 * 1.3 * 30.0 * 1e-6;
    for (size_t repeat = 0; repeat < 2; ++repeat) {
        Steps::StepReadGeneratedGcode::fill_print_statistics(result, config, stats);
        CHECK(stats.filament_stats == result.print_statistics.volumes_per_extruder);
        CHECK(stats.total_extruded_volume == Approx(10.0 * area0 + 20.0 * area1));
        CHECK(stats.total_used_filament == Approx(30.0));
        CHECK(stats.total_weight == Approx(expected_mass));
        CHECK(stats.total_cost == Approx(expected_cost));
        CHECK(stats.total_wipe_tower_filament == Approx(3.0));
        CHECK(stats.total_wipe_tower_filament_weight == Approx(area0 * 1.1 * 0.001 + 2.0 * area1 * 1.3 * 0.001));
        CHECK(stats.total_wipe_tower_volume == Approx(area0 + 2.0 * area1));
        CHECK(stats.total_wipe_tower_cost == Approx(area0 * 1.1 * 20.0 * 1e-6 + 2.0 * area1 * 1.3 * 30.0 * 1e-6));
        REQUIRE(stats.estimated_print_time.size() == 1);
        CHECK(stats.estimated_print_time.at(0) == Approx(3600.0));
        CHECK(stats.estimated_print_time_str.at(0) == get_time_dhms(3600.f));
        REQUIRE(stats._layer_area_stats.size() == 1);
        CHECK(stats._layer_area_stats[0].second == 12.f);
        CHECK(stats.initial_extruder_id == 1);
        CHECK(stats.total_toolchanges == 4);
        CHECK(stats.printing_extruders == std::vector<extruder_index_t>{0, 1});
        CHECK(stats.object_layer_count == 7);
        CHECK(stats.total_layer_count == 9);
        CHECK(stats.statistics_available.load());
        CHECK(stats.is_computing_gcode.load());
    }
    result.print_statistics.volumes_per_role_per_extruder.clear();
    Steps::StepReadGeneratedGcode::fill_print_statistics(result, config, stats);
    CHECK(stats.total_wipe_tower_filament == 0.0);
    CHECK(stats.total_wipe_tower_filament_weight == 0.0);
    CHECK(stats.total_wipe_tower_volume == 0.0);
    CHECK(stats.total_wipe_tower_cost == 0.0);
    stats.clear();
    CHECK(stats.total_wipe_tower_filament == 0.0);
    CHECK(stats.total_wipe_tower_filament_weight == 0.0);
    CHECK(stats.total_wipe_tower_volume == 0.0);
    CHECK(stats.total_wipe_tower_cost == 0.0);
}

// Export a sliced single-tool print or a two-tool tower print through production
// plugins. Reconcile reader volumes independently, then export without a viewer
// result (statistics unavailable) and export again with a local result, as the CLI does.
TEST_CASE("Export statistics include the tower share for preview and explicit CLI reads",
          "[plugins][gcode][wipe-tower][print-statistics][integration]")
{
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool tower = GENERATE(false, true);
    CAPTURE(tower);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_extruders(tower ? 2 : 1);
    config.set_deserialize_strict({{"wipe_tower", tower ? "1" : "0"}, {"wipe_tower_x", "40"}, {"wipe_tower_y", "60"},
        {"wipe_tower_width", "20"}, {"wipe_tower_rotation_angle", "0"}, {"wipe_tower_brim_width", "0"},
        {"single_extruder_multi_material", "0"}, {"filament_diameter", tower ? "1.75,2.85" : "1.75"},
        {"filament_density", tower ? "1.1,1.3" : "1.1"}, {"filament_cost", tower ? "20,30" : "20"}, {"time_cost", "2"},
        {"perimeter_extruder", "1"}, {"infill_extruder", "1"}, {"solid_infill_extruder", "1"},
        {"step_perimeter_plugin", "perimeter.generator.classic"}, {"step_surface_generation_plugin", "surface.initial_typed_surface_builder"},
        {"step_wipetower_plugin", "wipe_tower.manager.default"}, {"gcode_firmware_plugin", "gcode.firmware.marlin2"},
        {"layer_height", "0.3"}, {"fill_density", "20%"}});
    if (tower)
        config.set_deserialize_strict({{"wiping_volumes_matrix", "0,140,140,0"}, {"filament_minimal_purge_on_wipe_tower", "15,15"}});
    Print print;
    Model model;
    if (tower) {
        Test::init_print({Test::TestMesh::cube_20x20x20, Test::TestMesh::cube_20x20x20}, print, model, config);
        for (const char *key : {"perimeter_extruder", "infill_extruder", "solid_infill_extruder"})
            model.objects()[1].config.set_deserialize_strict(key, "2");
        print.apply(model, config);
    } else
        Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
    REQUIRE_NOTHROW(print.process());
    const boost::filesystem::path checkout = boost::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
    const std::string preview_path = (checkout / "build" / "temp" /
        (tower ? "stats_tower_preview.gcode" : "stats_single_preview.gcode")).string();
    GCodeProcessorResult result;
    REQUIRE_NOTHROW(Orchestrator::instance().export_gcode(print, preview_path, &result));
    const PrintStatistics &stats = print.print_statistics();
    REQUIRE(result.print_statistics.volumes_per_extruder.size() == (tower ? 2 : 1));
    CHECK(stats.filament_stats == result.print_statistics.volumes_per_extruder);
    double expected_volume = 0.0, expected_length = 0.0, expected_mass = 0.0, expected_cost = 0.0;
    for (const auto &[tool, volume] : result.print_statistics.volumes_per_extruder) {
        const double area = PI * std::pow(print.config().filament_diameter.get_at(tool) * 0.5, 2.0);
        const double mass = volume * print.config().filament_density.get_at(tool) * 0.001;
        expected_volume += volume;
        expected_length += volume / area;
        expected_mass += mass;
        expected_cost += mass * print.config().filament_cost.get_at(tool) * 0.001;
    }
    expected_cost += print.config().time_cost.value * (result.print_statistics.modes[0].time / 3600.f);
    CHECK(stats.total_extruded_volume == Approx(expected_volume));
    CHECK(stats.total_used_filament == Approx(expected_length));
    CHECK(stats.total_weight == Approx(expected_mass));
    CHECK(stats.total_cost == Approx(expected_cost));
    REQUIRE(stats.estimated_print_time.count(0) == 1);
    CHECK(stats.estimated_print_time.at(0) > 0.0);
    CHECK(stats.estimated_print_time_str.at(0) == get_time_dhms(result.print_statistics.modes[0].time));
    CHECK(stats.estimated_print_time.count(1) == 0);
    double tower_length = 0.0, tower_mass = 0.0, tower_volume = 0.0, tower_cost = 0.0;
    if (tower) {
        REQUIRE(result.print_statistics.volumes_per_role_per_extruder.count(GCodeExtrusionRole::WipeTower) == 1);
        for (const auto &[tool, volume] : result.print_statistics.volumes_per_role_per_extruder.at(GCodeExtrusionRole::WipeTower)) {
            tower_length += volume / (PI * std::pow(print.config().filament_diameter.get_at(tool) * 0.5, 2.0));
            tower_volume += volume;
            const double mass = volume * print.config().filament_density.get_at(tool) * 0.001;
            tower_mass += mass;
            tower_cost += mass * print.config().filament_cost.get_at(tool) * 0.001;
        }
        CHECK(stats.total_wipe_tower_filament > 0.0);
    }
    CHECK(stats.total_wipe_tower_filament == Approx(tower_length));
    CHECK(stats.total_wipe_tower_filament_weight == Approx(tower_mass));
    CHECK(stats.total_wipe_tower_volume == Approx(tower_volume));
    CHECK(stats.total_wipe_tower_cost == Approx(tower_cost));
    const DynamicConfig placeholders = stats.config();
    CHECK(placeholders.opt_float("total_wipe_tower_filament") == Approx(tower_length));
    CHECK(placeholders.opt_float("total_wipe_tower_cost") == Approx(tower_cost));
    const std::map<uint8_t, double> preview_times = stats.estimated_print_time;
    const std::vector<std::pair<coord_t, float>> preview_areas = stats._layer_area_stats;
    REQUIRE_FALSE(preview_areas.empty());
    const std::string cli_path = (checkout / "build" / "temp" /
        (tower ? "stats_tower_cli.gcode" : "stats_single_cli.gcode")).string();
    REQUIRE_NOTHROW(Orchestrator::instance().export_gcode(print, cli_path, nullptr));
    CHECK(print.print_statistics().estimated_print_time.empty());
    CHECK(print.print_statistics().total_used_filament == 0.0);
    CHECK_FALSE(print.print_statistics().statistics_available);
    CHECK(boost::filesystem::exists(print.gcode_reader_source_path()));
    CHECK(print.gcode_reader_plugin_id() == result.reader_plugin_id);
    GCodeProcessorResult local_result;
    // The reference of the export above is valid and would be reused without reading: ask for a new pass that reads it.
    print.request_replay_from_worker(Steps::Checkpoint::Plan);
    REQUIRE_NOTHROW(Orchestrator::instance().export_gcode(print, cli_path, &local_result));
    CHECK(stats.statistics_available);
    CHECK(stats.total_extruded_volume == Approx(expected_volume));
    CHECK(stats.total_used_filament == Approx(expected_length));
    CHECK(stats.total_weight == Approx(expected_mass));
    CHECK(stats.total_cost == Approx(expected_cost));
    CHECK(stats.total_wipe_tower_filament == Approx(tower_length));
    CHECK(stats.total_wipe_tower_filament_weight == Approx(tower_mass));
    CHECK(stats.total_wipe_tower_volume == Approx(tower_volume));
    CHECK(stats.total_wipe_tower_cost == Approx(tower_cost));
    CHECK(stats.estimated_print_time == preview_times);
    CHECK(stats._layer_area_stats == preview_areas);
    CHECK(stats.filament_stats == result.print_statistics.volumes_per_extruder);
    CHECK_FALSE(stats.finalize_output_path("stats_{total_wipe_tower_filament}_{total_wipe_tower_cost}.gcode").empty());
}

// Every G-code script sees has_wipe_tower true only when the plan holds tower extrusions, and
// has_single_extruder_multi_material_priming only with a tower, SEMM and priming.
TEST_CASE("Every G-code script knows whether a wipe tower is printed", "[plugins][wipe-tower][toolchange][placeholders]")
{
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool tower = GENERATE(false, true);
    const bool semm = GENERATE(false, true);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_extruders(2);
    config.set_deserialize_strict({{"wipe_tower", tower ? "1" : "0"}, {"wipe_tower_x", "40"}, {"wipe_tower_y", "60"},
        {"wipe_tower_width", "20"}, {"wipe_tower_brim_width", "0"},
        {"wiping_volumes_matrix", "0,8,8,0"}, {"filament_minimal_purge_on_wipe_tower", "2,2"},
        {"filament_diameter", "1.75,1.75"}, {"extrusion_multiplier", "1,1"},
        {"perimeter_extruder", "1"}, {"infill_extruder", "1"}, {"solid_infill_extruder", "1"},
        {"step_perimeter_plugin", "perimeter.generator.classic"}, {"step_surface_generation_plugin", "surface.initial_typed_surface_builder"},
        {"step_wipetower_plugin", "wipe_tower.manager.default"}, {"layer_height", "0.3"}, {"fill_density", "10%"},
        {"single_extruder_multi_material", semm ? "1" : "0"}, {"single_extruder_multi_material_priming", "1"},
        {"start_gcode", "{if has_wipe_tower}M117 START_TOWER{else}M117 START_NO_TOWER{endif}\n"},
        {"end_gcode", "{if has_wipe_tower}M117 END_TOWER{else}M117 END_NO_TOWER{endif}\n"
                      "{if has_single_extruder_multi_material_priming}M117 END_PRIMING{else}M117 END_NOPRIMING{endif}\n"},
        {"layer_gcode", "{if has_wipe_tower}M117 LAYER_TOWER{else}M117 LAYER_NO_TOWER{endif}\n"},
        {"toolchange_gcode", "{if has_wipe_tower}M117 CHANGE_TOWER{else}M117 CHANGE_NO_TOWER{endif}\n"}});
    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20, Test::TestMesh::cube_20x20x20}, print, model, config);
    for (const char *key : {"perimeter_extruder", "infill_extruder", "solid_infill_extruder"})
        model.objects()[1].config.set_deserialize_strict(key, "2");
    print.apply(model, config);
    REQUIRE_NOTHROW(print.process());
    const boost::filesystem::path checkout = boost::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
    const std::string path = (checkout / "build" / "temp" / "wipe_tower_placeholders.gcode").string();
    REQUIRE_NOTHROW(Orchestrator::instance().export_gcode(print, path, nullptr));
    const std::string gcode = Test::GCode::read_text_file(path);
    CHECK((gcode.find("M117 START_TOWER") != std::string::npos) == tower);
    CHECK((gcode.find("M117 START_NO_TOWER") != std::string::npos) == !tower);
    CHECK((gcode.find("M117 END_TOWER") != std::string::npos) == tower);
    CHECK((gcode.find("M117 END_NO_TOWER") != std::string::npos) == !tower);
    CHECK((gcode.find("M117 LAYER_TOWER") != std::string::npos) == tower);
    CHECK((gcode.find("M117 LAYER_NO_TOWER") != std::string::npos) == !tower);
    CHECK((gcode.find("M117 CHANGE_TOWER") != std::string::npos) == tower);
    CHECK((gcode.find("M117 CHANGE_NO_TOWER") != std::string::npos) == !tower);
    CHECK((gcode.find("M117 END_PRIMING") != std::string::npos) == (tower && semm));
    CHECK((gcode.find("M117 END_NOPRIMING") != std::string::npos) == !(tower && semm));
}

// A SEMM print that uses one filament has no tool change and therefore no tower: the plan holds no
// tower extrusion, the start G-code sees has_wipe_tower false, and the profile's end G-code unloads
// the filament instead of the tower.
TEST_CASE("Single-filament SEMM print reports no wipe tower", "[plugins][wipe-tower][toolchange][placeholders]")
{
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_extruders(2);
    config.set_deserialize_strict({{"wipe_tower", "1"}, {"wipe_tower_x", "40"}, {"wipe_tower_y", "60"},
        {"wipe_tower_width", "20"}, {"wipe_tower_brim_width", "0"},
        {"wiping_volumes_matrix", "0,8,8,0"}, {"filament_minimal_purge_on_wipe_tower", "2,2"},
        {"filament_diameter", "1.75,1.75"}, {"extrusion_multiplier", "1,1"},
        {"perimeter_extruder", "1"}, {"infill_extruder", "1"}, {"solid_infill_extruder", "1"},
        {"step_perimeter_plugin", "perimeter.generator.classic"}, {"step_surface_generation_plugin", "surface.initial_typed_surface_builder"},
        {"step_wipetower_plugin", "wipe_tower.manager.default"}, {"layer_height", "0.3"}, {"fill_density", "10%"},
        {"single_extruder_multi_material", "1"}, {"single_extruder_multi_material_priming", "1"},
        {"start_gcode", "{if has_wipe_tower}M117 TOWER{endif}\n"
                        "{if has_single_extruder_multi_material_priming}M117 PRIMING{endif}\n"},
        {"end_gcode", "{if has_wipe_tower}M117 END_TOWER{else}M117 END_UNLOAD{endif}\n"}});
    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
    print.apply(model, config);
    REQUIRE_NOTHROW(print.process());
    const boost::filesystem::path checkout = boost::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
    const std::string path = (checkout / "build" / "temp" / "wipe_tower_single_filament.gcode").string();
    REQUIRE_NOTHROW(Orchestrator::instance().export_gcode(print, path, nullptr));
    // No service produced a tower extrusion and no synthetic tower catalog entry exists.
    for (const Printing::PrintingGroup &group : print.printing_plan()->groups)
        for (const Printing::PrintingLayerGroup &layer : group.layers)
            for (const Printing::PrintingToolGroup &tool : layer.tool_groups)
                for (const Printing::PrintingExtrusion &extrusion : tool.extrusions)
                    CHECK_FALSE(extrusion.sregion_island_role.is_wipetower());
    for (const Printing::PrintingObjectInstance &instance : print.printing_plan()->object_instances)
        CHECK(instance.name.find("Wipe tower") == std::string::npos);
    const std::string gcode = Test::GCode::read_text_file(path);
    CHECK(gcode.find("M117 TOWER") == std::string::npos);
    CHECK(gcode.find("M117 PRIMING") == std::string::npos);
    CHECK(gcode.find("M220") == std::string::npos);
    // The print still reaches its end G-code, which takes the no-tower unload branch.
    CHECK(gcode.find("M117 END_TOWER") == std::string::npos);
    CHECK(gcode.find("M117 END_UNLOAD") != std::string::npos);
}

// Every explicit retraction is restored: the exported E stream nets to the plan's deposited
// filament for single-tool retract-with-wipe, multi-nozzle toolchange retraction, firmware
// retraction, and both E addressing modes.
TEST_CASE("Exports restore every explicit retraction",
          "[plugins][gcode][firmware][retraction]")
{
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const int variant = GENERATE(0, 1, 2, 3);
    CAPTURE(variant);
    const bool relative_e = variant == 1;
    const bool two_tools = variant == 2;
    const bool firmware_retraction = variant == 3;
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_extruders(2);
    config.set_deserialize_strict({{"filament_diameter", "1.75,1.75"}, {"extrusion_multiplier", "1,1"},
        {"perimeter_extruder", "1"}, {"infill_extruder", "1"}, {"solid_infill_extruder", "1"},
        {"step_perimeter_plugin", "perimeter.generator.classic"}, {"step_surface_generation_plugin", "surface.initial_typed_surface_builder"},
        {"layer_height", "0.3"}, {"fill_density", "10%"},
        {"wipe", "1,1"}, {"retract_length", "2"}, {"retract_length_toolchange", "10"},
        {"single_extruder_multi_material", "0"},
        {"use_relative_e_distances", relative_e ? "1" : "0"},
        {"use_firmware_retraction", firmware_retraction ? "1" : "0"}});
    Print print;
    Model model;
    if (two_tools)
        Test::init_print({Test::TestMesh::cube_20x20x20, Test::TestMesh::cube_20x20x20}, print, model, config);
    else
        Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
    if (two_tools)
        for (const char *key : {"perimeter_extruder", "infill_extruder", "solid_infill_extruder"})
            model.objects()[1].config.set_deserialize_strict(key, "2");
    print.apply(model, config);
    REQUIRE_NOTHROW(print.process());
    const boost::filesystem::path checkout = boost::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
    const std::string path = (checkout / "build" / "temp" /
        ("retraction_restore_" + std::to_string(variant) + ".gcode")).string();
    GCodeProcessorResult result;
    REQUIRE_NOTHROW(Orchestrator::instance().export_gcode(print, path, &result));

    // The expected deposited filament of the plan; explicit retraction operations net to their
    // restart extras, which stay zero here.
    const double filament_area = PI * std::pow(print.config().filament_diameter.get_at(0) * 0.5, 2.);
    const double e_per_mm3 = print.config().extrusion_multiplier.get_at(0) / filament_area;
    double expected_e = 0.;
    for (const Printing::PrintingGroup &group : print.printing_plan()->groups)
        for (const Printing::PrintingLayerGroup &layer : group.layers)
            for (const Printing::PrintingToolGroup &tool : layer.tool_groups)
                for (const Printing::PrintingExtrusion &extrusion : tool.extrusions)
                    if (extrusion.root != nullptr)
                        expected_e += plan_net_e(*extrusion.root, e_per_mm3);
    // A multi-nozzle tool-change retraction has no incoming unretract event at the layer
    // boundary, so the next printing extrusion repays it (P3b) and the stream is shorter by
    // that retraction. Count those requests from the plan.
    double repaid_retraction = 0.;
    if (two_tools)
        for (const Printing::PrintingGroup &group : print.printing_plan()->groups)
            for (const Printing::PrintingLayerGroup &layer : group.layers)
                for (const Printing::PrintingToolGroup &tool : layer.tool_groups) {
                    if (tool.events.has_before())
                        repaid_retraction += plan_toolchange_retraction(tool.events.before());
                    if (tool.events.has_after())
                        repaid_retraction += plan_toolchange_retraction(tool.events.after());
                    for (const Printing::PrintingExtrusion &extrusion : tool.extrusions)
                        if (extrusion.root != nullptr)
                            repaid_retraction += plan_toolchange_retraction(*extrusion.root);
                }
    // The exported stream nets to the same filament once every explicit retraction is restored
    // or repaid by printing.
    double exported_e = 0.;
    for (const GCodeProcessorResult::MoveVertex &move : result.moves)
        exported_e += double(move.delta_extruder);
    CHECK(exported_e == Approx(expected_e - repaid_retraction).margin(0.5));
}

// The exported band asks the firmware for no pressure advance before the ramming of an enabled
// outgoing filament, and for the incoming filament value before its load and purge. A disabled
// filament emits no pressure command at all, so a value written by the filament G-code stays.
TEST_CASE("Tool change exports the requested pressure advance",
          "[plugins][wipe-tower][toolchange][pressure-advance]")
{
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool semm = GENERATE(false, true);
    const bool enabled_tool0 = GENERATE(false, true);
    const bool enabled_tool1 = GENERATE(false, true);
    CAPTURE(semm, enabled_tool0, enabled_tool1);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_extruders(2);
    config.set_deserialize_strict({{"wipe_tower", "1"}, {"wipe_tower_x", "40"}, {"wipe_tower_y", "60"},
        {"wipe_tower_width", "20"}, {"wipe_tower_rotation_angle", "0"}, {"wipe_tower_brim_width", "0"},
        {"wiping_volumes_matrix", "0,8,8,0"}, {"filament_minimal_purge_on_wipe_tower", "2,2"},
        {"filament_diameter", "1.75,1.75"}, {"extrusion_multiplier", "1,1"},
        {"perimeter_extruder", "1"}, {"infill_extruder", "1"}, {"solid_infill_extruder", "1"},
        {"step_perimeter_plugin", "perimeter.generator.classic"}, {"step_surface_generation_plugin", "surface.initial_typed_surface_builder"},
        {"step_wipetower_plugin", "wipe_tower.manager.default"}, {"layer_height", "0.3"}, {"fill_density", "10%"},
        {"single_extruder_multi_material", semm ? "1" : "0"},
        {"filament_multitool_ramming", "1,1"},
        {"filament_multitool_ramming_volume", "10,10"}, {"filament_multitool_ramming_flow", "5,5"},
        {"filament_pressure_advance", "0.05,0.15"},
        {"gcode_flavor", "marlin2"},
        {"gcode_firmware_plugin", "gcode.firmware.marlin2"}});
    ConfigOptionFloats *pressure_advance = config.option<ConfigOptionFloats>("filament_pressure_advance");
    REQUIRE(pressure_advance != nullptr);
    pressure_advance->set_enabled(enabled_tool0, 0);
    pressure_advance->set_enabled(enabled_tool1, 1);
    Print print;
    Model model;
    Test::init_print({make_cube(20., 20., 1.4), make_cube(20., 20., 1.4)}, print, model, config);
    for (const char *key : {"perimeter_extruder", "infill_extruder", "solid_infill_extruder"})
        model.objects()[1].config.set_deserialize_strict(key, "2");
    print.apply(model, config);
    REQUIRE_NOTHROW(print.process());
    REQUIRE(print.object(0).layer_count() >= 3);
    const boost::filesystem::path checkout = boost::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
    const std::string path = (checkout / "build" / "temp" /
        (std::string("toolchange_pressure_advance_") + (semm ? "semm" : "multi") +
         (enabled_tool0 ? "_t0on" : "_t0off") + (enabled_tool1 ? "_t1on" : "_t1off") + ".gcode")).string();
    GCodeProcessorResult result;
    REQUIRE_NOTHROW(Orchestrator::instance().export_gcode(print, path, &result));

    // The first change rams its outgoing tool, then loads or purges the tool of its T command.
    // Each request is encoded before the first movement that follows its root, so an enabled
    // outgoing tool writes M900 K0 before the change and an enabled incoming tool writes its own
    // value after the change but before its load or purge movement.
    std::vector<std::string> lines;
    {
        std::istringstream stream(Test::GCode::read_text_file(path));
        std::string line;
        while (std::getline(stream, line))
            lines.push_back(line);
    }
    std::vector<double> values;
    std::vector<size_t> positions;
    size_t m900_before_change = 0;
    size_t m900_after_change = 0;
    const size_t not_found = lines.size();
    size_t tower_line = not_found;
    size_t change_line = not_found;
    size_t move_after_change = not_found;
    for (size_t i = 0; i < lines.size(); ++i) {
        if (lines[i] == ";TYPE:Wipe tower" && tower_line == not_found)
            tower_line = i;
        if (tower_line != not_found && change_line == not_found && lines[i].size() >= 2 &&
            lines[i][0] == 'T' && lines[i][1] >= '0' && lines[i][1] <= '9')
            change_line = i;
        if (lines[i].compare(0, 6, "M900 K") == 0) {
            if (i < change_line)
                ++m900_before_change;
            else if (i < move_after_change)
                ++m900_after_change;
            if (values.size() < 2) {
                positions.push_back(i);
                values.push_back(std::stod(lines[i].substr(6)));
            }
        }
        if (change_line != not_found && move_after_change == not_found &&
            lines[i].compare(0, 3, "G1 ") == 0 && lines[i].find('E') != std::string::npos)
            move_after_change = i;
    }
    REQUIRE(tower_line != not_found);
    REQUIRE(change_line != not_found);
    REQUIRE(move_after_change != not_found);
    const int incoming_tool = lines[change_line][1] - '0';
    REQUIRE(incoming_tool < 2);
    // The first change rams the other tool and then loads or purges the tool of its T command.
    const bool incoming_enabled = incoming_tool == 0 ? enabled_tool0 : enabled_tool1;
    const bool outgoing_enabled = incoming_tool == 0 ? enabled_tool1 : enabled_tool0;
    const double incoming_value = pressure_advance->get_at(size_t(incoming_tool));
    if (outgoing_enabled && incoming_enabled) {
        CHECK(m900_before_change == 1);
        CHECK(m900_after_change == 1);
        REQUIRE(values.size() >= 2);
        CHECK(values[0] == 0.);
        CHECK(positions[0] > tower_line);
        CHECK(positions[0] < change_line);
        CHECK(values[1] == Approx(incoming_value));
        CHECK(positions[1] > change_line);
        CHECK(positions[1] < move_after_change);
    } else if (outgoing_enabled) {
        // The incoming filament does not enable the option, so nothing is written for its load or
        // purge: the machine keeps the zero written before the ramming.
        CHECK(m900_before_change == 1);
        CHECK(m900_after_change == 0);
        REQUIRE(values.size() >= 1);
        CHECK(values[0] == 0.);
        CHECK(positions[0] > tower_line);
        CHECK(positions[0] < change_line);
    } else if (incoming_enabled) {
        CHECK(m900_before_change == 0);
        CHECK(m900_after_change == 1);
        REQUIRE(values.size() >= 1);
        CHECK(values[0] == Approx(incoming_value));
        CHECK(positions[0] > change_line);
        CHECK(positions[0] < move_after_change);
    } else {
        CHECK(m900_before_change == 0);
        CHECK(m900_after_change == 0);
        CHECK(values.empty());
    }
}

// A SEMM printer shares one extruder, so its pressure-advance command carries no drive address.
TEST_CASE("SEMM pressure advance carries no drive address",
          "[plugins][wipe-tower][toolchange][pressure-advance][semm]")
{
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_extruders(2);
    config.set_deserialize_strict({{"wipe_tower", "1"}, {"wipe_tower_x", "40"}, {"wipe_tower_y", "60"},
        {"wipe_tower_width", "20"}, {"wipe_tower_rotation_angle", "0"}, {"wipe_tower_brim_width", "0"},
        {"wiping_volumes_matrix", "0,8,8,0"}, {"filament_minimal_purge_on_wipe_tower", "2,2"},
        {"filament_diameter", "1.75,1.75"}, {"extrusion_multiplier", "1,1"},
        {"perimeter_extruder", "1"}, {"infill_extruder", "1"}, {"solid_infill_extruder", "1"},
        {"step_perimeter_plugin", "perimeter.generator.classic"}, {"step_surface_generation_plugin", "surface.initial_typed_surface_builder"},
        {"step_wipetower_plugin", "wipe_tower.manager.default"}, {"layer_height", "0.3"}, {"fill_density", "10%"},
        {"single_extruder_multi_material", "1"},
        {"filament_pressure_advance", "0.05,0.15"},
        {"gcode_flavor", "reprapfirmware"},
        {"gcode_firmware_plugin", "gcode.firmware.reprap"}});
    ConfigOptionFloats *pressure_advance = config.option<ConfigOptionFloats>("filament_pressure_advance");
    REQUIRE(pressure_advance != nullptr);
    pressure_advance->set_enabled(true, 0);
    pressure_advance->set_enabled(true, 1);
    Print print;
    Model model;
    Test::init_print({make_cube(20., 20., 1.4), make_cube(20., 20., 1.4)}, print, model, config);
    for (const char *key : {"perimeter_extruder", "infill_extruder", "solid_infill_extruder"})
        model.objects()[1].config.set_deserialize_strict(key, "2");
    print.apply(model, config);
    REQUIRE_NOTHROW(print.process());
    REQUIRE(print.object(0).layer_count() >= 3);
    const boost::filesystem::path checkout = boost::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
    const std::string path = (checkout / "build" / "temp" / "toolchange_pressure_advance_reprap.gcode").string();
    GCodeProcessorResult result;
    REQUIRE_NOTHROW(Orchestrator::instance().export_gcode(print, path, &result));

    std::vector<std::string> commands;
    {
        std::istringstream stream(Test::GCode::read_text_file(path));
        std::string line;
        while (std::getline(stream, line))
            if (line.compare(0, 5, "M572 ") == 0)
                commands.push_back(line);
    }
    REQUIRE(commands.size() >= 2);
    CHECK(commands[0] == "M572 S0");
    CHECK(commands[1] == "M572 S0.05");
    for (const std::string &command : commands)
        CHECK(command.find(" D") == std::string::npos);
}
