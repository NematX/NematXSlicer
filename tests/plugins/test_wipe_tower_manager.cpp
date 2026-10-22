///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
Wipe manager tests lend a test-only service to the host. The service reports
real changes between consecutive concrete tool visits and writes one local
straight line per band. Constructed plans isolate insertion and ownership;
the sliced-print test exercises the export-side ordering and downstream path.

Representative direct calls (branches are not a timeline):

    TEST_CASE
        |-- Test::init_print()
        |-- print.apply()
        |-- print.process()
        |-- Steps::StepExtrusionOrdering::run_step()
        |-- Steps::StepGenerateWipeTower::run_step()
        `-- check_wipes()
*/
#include <catch2/catch.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <tbb/global_control.h>

#include "gcode_test_helpers.hpp"
#include "plugin_test_helpers.hpp"
#include "test_data.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_wipetower.h"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/ExtrusionProcessParameterHelpers.hpp"
#include "libslic3r/Plugins/PluginApiCompatibility.hpp"
#include "libslic3r/Steps/StepExtrusionOrdering.hpp"
#include "libslic3r/Steps/StepGenerateWipeTower.hpp"
#include "libslic3r/Steps/StepGenerateGcode.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"
#include "libslic3r/WipeTowerFootprint.hpp"
#include "layer_extrusion_edit_transition_test_helpers.hpp"

namespace {
using namespace Slic3r;

struct BandRecord {
    uint64_t private_id;
    double z;
    double height;
    double y0;
    double y1;
    double width;
    uint32_t catalog_id;
    c_vec2d entry;
};

class TestWipe final : public slic3r_api::PluginBase {
public:
    // Borrow the orchestrator and stable metadata for this process-lifetime test plugin.
    TestWipe(orchestrator_handle *orchestrator, const char *id, int32_t priority);
    mutable std::vector<c_wipe_tower_phase> phases;
    mutable std::vector<c_wipe_tower_request> requests;
    mutable std::vector<BandRecord> bands;
    bool outgoing = false;
    bool duplicate = false;
    bool empty = false;
    bool fail = false;
    c_wipe_tower_phase failure_phase = C_WIPE_TOWER_GENERATE;
    bool last_layer_only = false;
    bool supplied_requests = false;
    std::vector<c_wipe_tower_request> supplied;
    mutable std::vector<std::vector<extruder_index_t>> edit_tools;
    const char *plugin_id;
private:
    // Return the stable test activation id.
    const char *id_impl() const noexcept override;
    // Return a constant test display name.
    const char *name_impl() const noexcept override;
    // Register as a non-exclusive wipe service.
    slicing_step_t step_impl() const noexcept override;
    // Declare no dependencies.
    const char *const *dependencies_impl() const noexcept override;
    // Keep independent wipe services outside exclusive selector groups.
    const char *exclusive_group_impl() const noexcept override;
    // Return the supplied service priority.
    int32_t priority_impl() const noexcept override;
    // Check that service initialization has no phase payload.
    void setup_impl(const plugin_run_context *run, uint32_t count) const override;
    // Record phases, request tool-change bands, and fill lent roots with one local line.
    void run_impl(const plugin_run_context *run) const override;
    int32_t m_priority;
};

class WipeActivation {
public:
    // Register static test services once and enable only the requested number.
    explicit WipeActivation(uint32_t count = 1);
    // Disable test services and reset host cancellation before another test runs.
    ~WipeActivation();
    TestWipe *first = nullptr;
    TestWipe *second = nullptr;
    bool production_wipe_active = false;
};

// Configure a two-extruder print and build two layers with concrete visits and independent source roots.
void build_plan(Print &print, bool enabled = true);
// Inspect every inserted wipe and assert synthetic provenance, fixed order, and transformed line endpoints.
size_t check_wipes(const Print &print);
// Collect maintenance roots from a borrowed layer in execution order.
std::vector<const Printing::PrintingExtrusion *> maintenance_paths(const Printing::PrintingLayerGroup &layer);
// Find the first leaf whose stored raw attribute role matches, without converting process-only roles.
const ExtrusionEntity *first_leaf_with_raw_role(const ExtrusionEntity &entity, uint16_t raw_role);
// Return the roots of one visit that contain a tower wipe leaf, in insertion order.
std::vector<const Printing::PrintingExtrusion *> wipe_bands(const Printing::PrintingToolGroup &tool);
// Return true when the direct child at idx of a root carries the given special command code.
bool child_has_command(const ExtrusionEntity &root, uint32_t idx, ExtrusionPropertySpecialCommand::Code code);
// Count special commands with the given code in a borrowed tree, children included.
size_t count_commands(const ExtrusionEntity &entity, ExtrusionPropertySpecialCommand::Code code);
// Count the given special command in every plan root and scope event, in print order.
size_t count_plan_commands(const Printing::PrintingPlan &plan, ExtrusionPropertySpecialCommand::Code code);
// Build small real-owner plans with two copies and independently configured tower offsets.
void build_instance_plan(Print &print, Model &model, bool separate_groups, bool global_tower);
// Attach a Print-owned source layer with an explicit height to a compact plan group.
void attach_source_layer(Print &print, slic3r_api::PrintingLayerGroup group, double height);

// Initialize borrowed metadata without allocating output or transferring plugin ownership.
TestWipe::TestWipe(orchestrator_handle *orchestrator, const char *id, const int32_t priority)
    : PluginBase(orchestrator), plugin_id(id), m_priority(priority) {}
// Return test-owned static metadata without changing state.
const char *TestWipe::id_impl() const noexcept { return plugin_id; }
// Return a borrowed constant name without allocation or failure.
const char *TestWipe::name_impl() const noexcept { return "Test straight wipe"; }
// Return the non-exclusive service step identity.
slicing_step_t TestWipe::step_impl() const noexcept { return WIPE_TOWER_WIPE; }
// Return no dependencies; the manager explicitly calls this service.
const char *const *TestWipe::dependencies_impl() const noexcept { return nullptr; }
// Return an empty group so multiple providers remain independently active.
const char *TestWipe::exclusive_group_impl() const noexcept { return ""; }
// Return the supplied priority used by the active service snapshot.
int32_t TestWipe::priority_impl() const noexcept { return m_priority; }

// Inspect the borrowed initialization context without retaining it; non-null data or an unexpected run count throws.
void TestWipe::setup_impl(const plugin_run_context *run, const uint32_t count) const
{
    if (run->data != nullptr || count != 1)
        throw std::runtime_error("Wipe service setup requires null data and one manager run.");
}

// Read borrowed phase data, record edited tools, report owned supplied/change anchors and move scaled local lines with explicit speeds into lent roots; the selected failure phase throws the service's own cause, and invalid phase access or reporter failures throw without retaining borrowed data.
void TestWipe::run_impl(const plugin_run_context *run) const
{
    const run_ctx_wipe_tower_wipe *ctx = plugin_ctx_as_wipe_tower_wipe(run);
    if (ctx == nullptr)
        throw std::runtime_error("Missing wipe service payload.");
    phases.push_back(ctx->phase);
    if (fail && ctx->phase == failure_phase)
        throw std::runtime_error("Intentional wipe service failure.");
    if (ctx->phase == C_WIPE_TOWER_EDIT) {
        if (ctx->mutable_plan == nullptr || ctx->report_request != nullptr || ctx->report_context != nullptr ||
            ctx->request != nullptr || ctx->band != nullptr || ctx->exit != nullptr)
            throw std::runtime_error("EDIT requires only the plan fields.");
        const slic3r_api::ConstPrintingPlan plan(ctx->plan);
        for (uint32_t g = 0; g < plan.group_count(); ++g)
            for (uint32_t l = 0; l < plan.group(g).layer_group_count(); ++l) {
                std::vector<extruder_index_t> tools;
                const slic3r_api::ConstPrintingLayerGroup layer = plan.group(g).layer_group(l);
                for (uint32_t t = 0; t < layer.tool_group_count(); ++t)
                    tools.push_back(layer.tool_group(t).extruder_id());
                edit_tools.push_back(std::move(tools));
            }
        return;
    }
    if (ctx->phase == C_WIPE_TOWER_COLLECT) {
        if (ctx->mutable_plan != nullptr || ctx->report_request == nullptr || ctx->report_context == nullptr ||
            ctx->request != nullptr || ctx->band != nullptr || ctx->exit != nullptr)
            throw std::runtime_error("COLLECT must be read-only and provide a reporter.");
        const slic3r_api::ConstPrintingPlan plan(ctx->plan);
        if (supplied_requests) {
            for (const c_wipe_tower_request &request : supplied) {
                requests.push_back(request);
                if (!ctx->report_request(ctx->report_context, &request))
                    throw std::runtime_error("Cannot report supplied wipe request.");
            }
            return;
        }
        extruder_index_t previous = EXTRUDER_ID_UNKNOWN;
        c_wipe_tower_anchor previous_end = {};
        for (uint32_t g = 0; g < plan.group_count(); ++g)
            for (uint32_t l = 0; l < plan.group(g).layer_group_count(); ++l) {
                const slic3r_api::ConstPrintingLayerGroup layer = plan.group(g).layer_group(l);
                for (uint32_t t = 0; t < layer.tool_group_count(); ++t) {
                    const slic3r_api::ConstPrintingToolGroup tool = layer.tool_group(t);
                    if (previous != EXTRUDER_ID_UNKNOWN && previous != tool.extruder_id() &&
                        (!last_layer_only || l + 1 == plan.group(g).layer_group_count())) {
                        c_wipe_tower_request request = {};
                        request.private_id = requests.size() + 1;
                        request.incoming = {g, l, t, 0};
                        request.has_incoming = 1;
                        request.purge_volume = 8.0;
                        if (outgoing && previous_end.group_idx == g && previous_end.layer_group_idx == l) {
                            request.has_outgoing = 1;
                            request.outgoing = previous_end;
                        }
                        requests.push_back(request);
                        if (!ctx->report_request(ctx->report_context, &request))
                            throw std::runtime_error("Cannot report wipe request.");
                        if (duplicate) {
                            request.private_id = requests.size() + 1;
                            requests.push_back(request);
                            if (!ctx->report_request(ctx->report_context, &request))
                                throw std::runtime_error("Cannot report duplicate wipe request.");
                        }
                    }
                    previous = tool.extruder_id();
                    previous_end = {g, l, t, tool.extrusion_count()};
                }
            }
        return;
    }
    if (ctx->phase != C_WIPE_TOWER_GENERATE || ctx->mutable_plan != nullptr ||
        ctx->request == nullptr || ctx->band == nullptr || ctx->exit == nullptr || ctx->report_request != nullptr)
        throw std::runtime_error("GENERATE must only edit supplied roots and return an exit.");
    const c_wipe_tower_band &band = *ctx->band;
    bands.push_back({ctx->request->private_id, band.print_z, band.layer_height,
                      band.min.y, band.max.y, band.extrusion_width, band.tower_catalog_id, band.entry});
    *ctx->exit = {band.max.x, band.min.y};
    if (empty)
        return;
    for (extrusion_entity_handle *handle : {band.outgoing_root, band.incoming_root}) {
        if (handle == nullptr)
            continue;
        slic3r_api::StoredExtrusionEntity line(run->plugin_storage);
        line.set_points({{scale_i(band.min.x), scale_i(band.min.y)},
                         {scale_i(band.max.x), scale_i(band.min.y)}});
        line.get_or_add(slic3r_api::EPropertyAttributes::key)
            .extrusion_role(RAW_EXTRUSION_ROLE_WIPE_TOWER_WIPE)
            .width(float(band.extrusion_width)).height(float(band.layer_height))
            .mm3_per_mm(band.extrusion_width * band.layer_height);
        line.get_or_add(slic3r_api::EPropertySpeed::key).speed_mm_per_s = 20.f;
        slic3r_api::MutableExtrusionEntity root(handle);
        root.append_child_move(line.mutable_view());
    }
}

// Register borrowed test services and reset probes, temporarily disabling the production service to isolate manager geometry; activation failures remain visible and no plan ownership is retained.
WipeActivation::WipeActivation(const uint32_t count)
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    orchestrator.reset_plugin_cancel();
    production_wipe_active = orchestrator.is_plugin_active("wipe_tower.wipe.toolchange.default");
    REQUIRE(orchestrator.set_plugin_active("wipe_tower.wipe.toolchange.default", false));
    orchestrator_handle *handle = reinterpret_cast<orchestrator_handle *>(&orchestrator);
    static TestWipe first_plugin(handle, "test.wipe.low", -10);
    static TestWipe second_plugin(handle, "test.wipe.high", 10);
    first = &first_plugin;
    second = &second_plugin;
    uint32_t idx = 0;
    for (TestWipe *plugin : {first, second}) {
        if (orchestrator.get_plugin(plugin->plugin_id) == nullptr)
            REQUIRE(orchestrator.register_plugin(plugin->c_instance()));
        plugin->phases.clear();
        plugin->requests.clear();
        plugin->bands.clear();
        plugin->outgoing = plugin->duplicate = plugin->empty = plugin->fail = false;
        plugin->failure_phase = C_WIPE_TOWER_GENERATE;
        plugin->last_layer_only = plugin->supplied_requests = false;
        plugin->supplied.clear();
        plugin->edit_tools.clear();
        REQUIRE(orchestrator.set_plugin_active(plugin->plugin_id, idx++ < count));
    }
}

// Leave test services registered but inactive, restore production activation and clear cancellation; no borrowed plan data is retained.
WipeActivation::~WipeActivation()
{
    Orchestrator &orchestrator = Orchestrator::instance();
    orchestrator.set_plugin_active(first->plugin_id, false);
    orchestrator.set_plugin_active(second->plugin_id, false);
    orchestrator.set_plugin_active("wipe_tower.wipe.toolchange.default", production_wipe_active);
    orchestrator.reset_plugin_cancel();
}

// Create plan-owned paths and Print-owned 0.2/0.3 mm source layers through editing primitives with consistent printer vectors; source/allocation failures propagate without borrowing temporary layers.
void build_plan(Print &print, const bool enabled)
{
    Test::GCode::configure_standard_firmware(print, 2);
    PrintObjectConfig &config = const_cast<PrintObjectConfig &>(print.default_object_config());
    config.wipe_tower.value = enabled;
    config.wipe_tower_x.value = 40.0;
    config.wipe_tower_y.value = 60.0;
    config.wipe_tower_width.value = 20.0;
    config.wipe_tower_rotation_angle.value = 90.0;
    config.wipe_tower_extrusion_width.value = 150.0;
    config.wipe_tower_extrusion_width.percent = true;
    DynamicPrintConfig &full = const_cast<DynamicPrintConfig &>(print.full_print_config());
    full = DynamicPrintConfig::full_print_config();
    full.set_num_extruders(2);
    full.set_deserialize_strict("nozzle_diameter", "0.4,0.4");
    full.set_deserialize("step_wipetower_plugin", "wipe_tower.manager.default");
    slic3r_api::PrintingPlan plan = Test::GCode::printing_plan_view(print);
    slic3r_api::PrintingGroup group = plan.append_group();
    for (const double z : {0.2, 0.5}) {
        slic3r_api::PrintingLayerGroup layer = group.append_layer_group(scale_i(z));
        attach_source_layer(print, layer, z == 0.2 ? 0.2 : 0.3);
        for (const extruder_index_t extruder : {extruder_index_t(0), extruder_index_t(1)}) {
            slic3r_api::PrintingToolGroup tool = layer.append_tool_group(extruder);
            std::unique_ptr<ExtrusionPath> path = Test::GCode::make_firmware_path(
                ArcPolyline(Points{Point(scale_i(1.0), scale_i(1.0)), Point(scale_i(2.0), scale_i(1.0))}),
                20.f, 500.f);
            REQUIRE(printing_tool_group_append_extrusion_clone(plan.handle(), tool.mutable_handle(), nullptr,
                RAW_EXTRUSION_ROLE_PERIMETER, reinterpret_cast<const extrusion_entity_handle *>(path.get()),
                SLIC3R_PRINTING_OBJECT_INSTANCE_ID_NONE) != nullptr);
        }
    }
}

// Validate borrowed band/maintenance roots and synthetic provenance, counting only service bands; maintenance leaves must carry rounded-flow attributes and explicit speed, and assertions expose missing output.
size_t check_wipes(const Print &print)
{
    const Printing::PrintingPlan &plan = *print.printing_plan();
    size_t count = 0;
    for (const Printing::PrintingGroup &group : plan.groups)
        for (const Printing::PrintingLayerGroup &layer : group.layers)
            for (const Printing::PrintingToolGroup &tool : layer.tool_groups)
                for (const Printing::PrintingExtrusion &extrusion : tool.extrusions) {
                    if (!extrusion.sregion_island_role.is_wipetower())
                        continue;
                    REQUIRE(extrusion.root != nullptr);
                    CHECK_FALSE(extrusion.root->can_sort());
                    CHECK_FALSE(extrusion.root->can_reverse());
                    CHECK(extrusion.region_island == nullptr);
                    REQUIRE(extrusion.printing_catalog_id < plan.object_instances.size());
                    const Printing::PrintingObjectInstance &instance = plan.object_instances[extrusion.printing_catalog_id];
                    CHECK(instance.object == nullptr);
                    CHECK(instance.name == "Wipe tower");
                    CHECK(instance.projected_outline_center.x() == scale_i(30.0));
                    CHECK(instance.projected_outline_center.y() == scale_i(70.0));
                    CHECK(std::find(group.printing_catalog_ids.begin(), group.printing_catalog_ids.end(),
                                    extrusion.printing_catalog_id) != group.printing_catalog_ids.end());
                    REQUIRE(extrusion.root->child_count() > 0);
                    if (extrusion.root->child(0).role() == ExtrusionRole::WipeTower) {
                        for (size_t c = 0; c < extrusion.root->child_count(); ++c) {
                            const ExtrusionEntity &path = extrusion.root->child(c);
                            const ExtrusionAttributes *attributes = path.get_property<ExtrusionAttributes>();
                            REQUIRE(attributes != nullptr);
                            CHECK(path.role() == ExtrusionRole::WipeTower);
                            CHECK(attributes->width > 0.f);
                            CHECK(attributes->height > 0.f);
                            const Flow flow = Flow::new_from_width(attributes->width, 0.4f, attributes->height, 1.f);
                            CHECK(attributes->mm3_per_mm == Approx(flow.mm3_per_mm()));
                            REQUIRE(path.get_property<ExtrusionPropertySpeed>() != nullptr);
                            CHECK(path.get_property<ExtrusionPropertySpeed>()->speed_mm_per_s > 0.f);
                        }
                        continue;
                    }
                    ++count;
                    // The manager adds at most one speed save and one restore around the band; the
                    // other leaf is the service wipe line.
                    CHECK(count_commands(*extrusion.root, ExtrusionPropertySpecialCommand::Code::SAVE_AND_RESET_SPEED_RATIO) <= 1);
                    CHECK(count_commands(*extrusion.root, ExtrusionPropertySpecialCommand::Code::RESTORE_SPEED_RATIO) <= 1);
                    const ExtrusionEntity *line = first_leaf_with_raw_role(*extrusion.root,
                        uint16_t(RAW_EXTRUSION_ROLE_WIPE_TOWER_WIPE));
                    REQUIRE(line != nullptr);
                    REQUIRE(line->get_property<ExtrusionPropertySpeed>() != nullptr);
                    CHECK(line->get_property<ExtrusionPropertySpeed>()->speed_mm_per_s == 20.f);
                    CHECK(line->first_point().y() == scale_i(60.0));
                    CHECK(line->last_point().y() == scale_i(80.0));
                    CHECK(line->first_point().x() == line->last_point().x());
                }
    return count;
}

// Return borrowed roots whose first child is manager maintenance, in tool/extrusion order; no ownership or geometry changes occur and allocation failures propagate.
std::vector<const Printing::PrintingExtrusion *> maintenance_paths(const Printing::PrintingLayerGroup &layer)
{
    std::vector<const Printing::PrintingExtrusion *> paths;
    for (const Printing::PrintingToolGroup &tool : layer.tool_groups)
        for (const Printing::PrintingExtrusion &extrusion : tool.extrusions)
            if (extrusion.sregion_island_role.is_wipetower() && extrusion.root != nullptr &&
                extrusion.root->child_count() > 0 && extrusion.root->child(0).role() == ExtrusionRole::WipeTower)
                paths.push_back(&extrusion);
    return paths;
}

// Search a borrowed tree depth first for the first leaf whose stored raw attribute role matches;
// process-only roles have no ExtrusionRole value, so raw comparison avoids the asserting conversion.
const ExtrusionEntity *first_leaf_with_raw_role(const ExtrusionEntity &entity, const uint16_t raw_role)
{
    if (entity.child_count() == 0) {
        const ExtrusionAttributes *attributes = entity.get_property<ExtrusionAttributes>();
        return attributes != nullptr && attributes->role == raw_role ? &entity : nullptr;
    }
    for (size_t idx = 0; idx < entity.child_count(); ++idx)
        if (const ExtrusionEntity *leaf = first_leaf_with_raw_role(entity.child(idx), raw_role))
            return leaf;
    return nullptr;
}

// Return the roots of one visit that contain a tower wipe leaf, in insertion order; roots stay
// plan-owned and no geometry is changed.
std::vector<const Printing::PrintingExtrusion *> wipe_bands(const Printing::PrintingToolGroup &tool)
{
    std::vector<const Printing::PrintingExtrusion *> bands;
    for (const Printing::PrintingExtrusion &extrusion : tool.extrusions)
        if (extrusion.root != nullptr &&
            first_leaf_with_raw_role(*extrusion.root, uint16_t(RAW_EXTRUSION_ROLE_WIPE_TOWER_WIPE)) != nullptr)
            bands.push_back(&extrusion);
    return bands;
}

// Return true when the direct child at idx of a borrowed root carries the given special command.
bool child_has_command(const ExtrusionEntity &root, const uint32_t idx,
                       const ExtrusionPropertySpecialCommand::Code code)
{
    if (idx >= root.child_count())
        return false;
    const ExtrusionPropertySpecialCommand *command = root.child(idx).get_property<ExtrusionPropertySpecialCommand>();
    return command != nullptr && command->command_code() == code;
}

// Count special commands with the given code in a borrowed tree, children included.
size_t count_commands(const ExtrusionEntity &entity, const ExtrusionPropertySpecialCommand::Code code)
{
    size_t count = 0;
    if (const ExtrusionPropertySpecialCommand *command = entity.get_property<ExtrusionPropertySpecialCommand>())
        count += command->command_code() == code ? 1 : 0;
    for (size_t idx = 0; idx < entity.child_count(); ++idx)
        count += count_commands(entity.child(idx), code);
    return count;
}

// Count the given special command in every plan-owned root and scope event, in print order; no
// borrowed view is retained and no tree is modified.
size_t count_plan_commands(const Printing::PrintingPlan &plan, const ExtrusionPropertySpecialCommand::Code code)
{
    size_t count = 0;
    for (const Printing::PrintingGroup &group : plan.groups)
        for (const Printing::PrintingLayerGroup &layer : group.layers)
            for (const Printing::PrintingToolGroup &tool : layer.tool_groups) {
                count += count_commands(tool.events.before(), code);
                count += count_commands(tool.events.after(), code);
                for (const Printing::PrintingExtrusion &extrusion : tool.extrusions)
                    if (extrusion.root != nullptr)
                        count += count_commands(*extrusion.root, code);
            }
    return count;
}

// Apply two real model copies then own two compact source layers per owner, either in one group or separate restarting-Z groups; ids link PrintObject instances, configuration/allocation failures propagate.
void build_instance_plan(Print &print, Model &model, const bool separate_groups, const bool global_tower)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_extruders(2);
    config.set_deserialize_strict({{"nozzle_diameter", "0.4,0.4"}, {"wipe_tower", global_tower ? "1" : "0"},
        {"wipe_tower_width", "20"}, {"wipe_tower_brim_width", "1"}, {"wipe_tower_rotation_angle", "0"},
        {"step_wipetower_plugin", "wipe_tower.manager.default"}, {"complete_objects", separate_groups ? "1" : "0"}});
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
    model.objects()[0].name = "Owner";
    model.objects()[0].instances[0]->set_offset(Vec3d(10., 20., 0.));
    model.objects()[0].add_instance()->set_offset(Vec3d(110., 20., 0.));
    model.objects()[0].config.set_deserialize_strict("wipe_tower_per_object", separate_groups ? "0" : "1");
    model.objects()[0].config.set_deserialize_strict("wipe_tower_object_x", "50");
    print.apply(model, config);
    slic3r_api::PrintingPlan plan = Test::GCode::printing_plan_view(print);
    plan.clear();
    const slic3r_api::Object object(reinterpret_cast<const object_handle *>(&print.object(0)));
    const uint32_t ids[] = {plan.find_or_add_object_instance(object, 0), plan.find_or_add_object_instance(object, 1)};
    const uint32_t groups = separate_groups ? 2 : 1;
    for (uint32_t g = 0; g < groups; ++g) {
        slic3r_api::PrintingGroup group = plan.append_group();
        for (uint32_t owner = 0; owner < 2; ++owner)
            if (!separate_groups || owner == g)
                group.append_printing_catalog_id(plan, ids[owner]);
        for (const double z : {0.2, 0.4}) {
            slic3r_api::PrintingLayerGroup layer = group.append_layer_group(scale_i(z));
            attach_source_layer(print, layer, 0.2);
            for (uint32_t owner = 0; owner < 2; ++owner) {
                if (separate_groups && owner != g)
                    continue;
                slic3r_api::PrintingToolGroup tool = layer.append_tool_group(extruder_index_t(owner));
                std::unique_ptr<ExtrusionPath> path = Test::GCode::make_firmware_path(
                    ArcPolyline(Points{Point::new_scale(10. + owner * 100., 20.), Point::new_scale(11. + owner * 100., 20.)}), 20.f, 500.f);
                REQUIRE(printing_tool_group_append_extrusion_clone(plan.handle(), tool.mutable_handle(), nullptr,
                    RAW_EXTRUSION_ROLE_PERIMETER, reinterpret_cast<const extrusion_entity_handle *>(path.get()), ids[owner]) != nullptr);
            }
        }
    }
}

// Append an auxiliary source layer owned by Print and lend its stable address to the plan group; positive millimeter heights become scaled coordinates and allocation/API failures propagate.
void attach_source_layer(Print &print, slic3r_api::PrintingLayerGroup group, const double height)
{
    PrintObject &object = print.mutable_auxiliary_object();
    const size_t id = object.auxiliary_layer_count();
    Layer &layer = object.add_auxiliary_layer(id, scale_i(height), group.print_z());
    group.append_layer(slic3r_api::Layer(reinterpret_cast<const layer_handle *>(&layer)));
    REQUIRE(group.layer_count() > 0);
}

} // namespace

TEST_CASE("Wipe manager ABI versions advertise the new payload and service", "[plugins][wipe-tower][abi]")
{
    const std::vector<PluginApiRequirement> &versions = host_plugin_api_contracts();
    REQUIRE(versions.size() > SLIC3R_PLUGIN_API_STEP_WIPE_TOWER_WIPE);
    CHECK(versions[SLIC3R_PLUGIN_API_STEP_WIPETOWER].version.major == 2);
    CHECK(versions[SLIC3R_PLUGIN_API_STEP_WIPE_TOWER_WIPE].version.major == 2);
    CHECK(versions[SLIC3R_PLUGIN_API_STEP_WIPE_TOWER_WIPE].version.minor == 0);
    CHECK(versions[SLIC3R_PLUGIN_API_STEP_WIPE_TOWER_WIPE].header == "steps/slic3r_step_wipe_tower_wipe.h");
    CHECK_FALSE(validate_plugin_api_requirements({versions.front(),
        {"steps/slic3r_step_wipetower.h", {1, 0}}}).compatible());
    CHECK(validate_plugin_api_requirements({versions.front(), versions[SLIC3R_PLUGIN_API_STEP_WIPETOWER],
        versions[SLIC3R_PLUGIN_API_STEP_WIPE_TOWER_WIPE]}).compatible());
}

// Inspect borrowed service bands and plan-owned maintenance before source extrusions, asserting layer heights, rounded reservations, provenance and rigid XY transformation; failures remain visible.
TEST_CASE("Wipe manager inserts transformed bands at collected incoming anchors", "[plugins][wipe-tower]")
{
    WipeActivation active;
    Print print;
    build_plan(print);
    Steps::StepGenerateWipeTower::run_step(Orchestrator::instance(), print);
    REQUIRE(active.first->requests.size() == 3);
    REQUIRE(active.first->bands.size() == 3);
    CHECK(active.first->phases[0] == C_WIPE_TOWER_EDIT);
    CHECK(active.first->phases[1] == C_WIPE_TOWER_COLLECT);
    CHECK(check_wipes(print) == 3);
    CHECK(active.first->bands[0].height == Approx(0.2));
    CHECK(active.first->bands[1].height == Approx(0.3));
    const double lower_pitch = Flow::new_from_width(0.6f, 0.4f, 0.2f, 1.f).spacing();
    const double upper_pitch = Flow::new_from_width(0.6f, 0.4f, 0.3f, 1.f).spacing();
    CHECK(active.first->bands[0].y1 == Approx(4. * lower_pitch));
    CHECK(active.first->bands[1].y1 == Approx(3. * upper_pitch));
    CHECK(active.first->bands[2].y0 == Approx(3. * upper_pitch));
    CHECK(active.first->bands[0].width == Approx(0.6));
    for (const c_wipe_tower_request &request : active.first->requests) {
        const Printing::PrintingToolGroup &tool = print.printing_plan()->groups[request.incoming.group_idx]
            .layers[request.incoming.layer_group_idx].tool_groups[request.incoming.tool_group_idx];
        const bool maintenance = request.incoming.tool_group_idx == (request.incoming.layer_group_idx == 0 ? 1 : 0);
        REQUIRE(tool.extrusions.size() == (maintenance ? 3 : 2));
        CHECK(tool.extrusions[maintenance ? 1 : 0].root->child(0).role() == ExtrusionRole::WipeTowerWipe);
        CHECK(tool.extrusions.back().sregion_island_role == ExtrusionRole::Perimeter);
        if (maintenance)
            CHECK(tool.extrusions[0].root->child(0).role() == ExtrusionRole::WipeTower);
    }
    std::string error;
    CHECK(Steps::StepGenerateWipeTower::validate_post(print, &error));
    CHECK(error.empty());
}

TEST_CASE("Wipe manager leaves disabled and unserved plans untouched", "[plugins][wipe-tower]")
{
    const bool enabled = GENERATE(false, true);
    WipeActivation active(enabled ? 0 : 1);
    Print print;
    build_plan(print, enabled);
    const ExtrusionEntity *retained = print.printing_plan()->groups[0].layers[0].tool_groups[0].extrusions[0].root.get();
    Steps::StepGenerateWipeTower::run_step(Orchestrator::instance(), print);
    CHECK(print.printing_plan()->object_instances.empty());
    CHECK(active.first->phases.empty());
    CHECK(print.printing_plan()->groups[0].layers[0].tool_groups[0].extrusions[0].root.get() == retained);
    for (const Printing::PrintingLayerGroup &layer : print.printing_plan()->groups[0].layers)
        for (const Printing::PrintingToolGroup &tool : layer.tool_groups)
            CHECK(tool.extrusions.size() == 1);
}

// Collect duplicate requests from borrowed priority-ordered services and inspect the resulting owned roots, asserting equal-anchor ordering and exit chaining without hiding empty/outgoing parts.
TEST_CASE("Wipe service priority and equal anchors retain band and insertion order", "[plugins][wipe-tower]")
{
    WipeActivation active(2);
    active.first->duplicate = true;
    active.first->outgoing = true;
    Print print;
    build_plan(print);
    Steps::StepGenerateWipeTower::run_step(Orchestrator::instance(), print);
    REQUIRE(active.first->bands.size() == 6);
    REQUIRE(active.second->bands.size() == 3);
    const double depth = 4. * Flow::new_from_width(0.6f, 0.4f, 0.2f, 1.f).spacing();
    CHECK(active.second->bands[0].y0 == Approx(2. * depth));
    CHECK(active.first->bands[1].entry.x == Approx(20.));
    CHECK(active.first->bands[1].entry.y == Approx(active.first->bands[0].y0));
    CHECK(active.second->bands[0].entry.y == Approx(active.first->bands[1].y0));
    CHECK(check_wipes(print) == 13);
    const Printing::PrintingToolGroup &incoming = print.printing_plan()->groups[0].layers[0].tool_groups[1];
    REQUIRE(incoming.extrusions.size() == 4);
    CHECK(incoming.extrusions[0].root->first_point().x() == scale_i(40.0));
    // Band edges are rounded to scaled coordinates by the rotation, hence the one-unit tolerance.
    CHECK(std::abs(incoming.extrusions[1].root->first_point().x() - scale_i(40. - depth)) <= 1);
    CHECK(std::abs(incoming.extrusions[2].root->first_point().x() - scale_i(40. - 2. * depth)) <= 1);
    CHECK(incoming.extrusions[3].sregion_island_role == ExtrusionRole::Perimeter);
    const Printing::PrintingToolGroup &outgoing = print.printing_plan()->groups[0].layers[0].tool_groups[0];
    REQUIRE(outgoing.extrusions.size() == 4);
    CHECK(outgoing.extrusions[0].sregion_island_role == ExtrusionRole::Perimeter);
    CHECK(outgoing.extrusions[1].sregion_island_role == ExtrusionRole::WipeTower);
    CHECK(outgoing.extrusions[1].root->child(0).role() == ExtrusionRole::WipeTower);
    // The outgoing band starts with the manager's speed save, so look for its wipe leaf.
    CHECK(first_leaf_with_raw_role(*outgoing.extrusions[2].root, uint16_t(RAW_EXTRUSION_ROLE_WIPE_TOWER_WIPE)) != nullptr);
}

// Run eligibility/zero-change variants against owned source visits, requiring no synthetic entries or geometry; a configured contour tool cannot manufacture the first tool change.
TEST_CASE("Global tower eligibility and an empty request list do not modify the plan", "[plugins][wipe-tower]")
{
    WipeActivation active;
    Print print;
    build_plan(print);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    SECTION("single extruder") { config.nozzle_diameter.set(std::vector<double>{0.4}); }
    SECTION("complete objects") { config.complete_objects.value = true; }
    SECTION("spiral vase") { config.spiral_vase.value = true; }
    SECTION("no actual tool change") {
        const_cast<PrintObjectConfig &>(print.default_object_config()).wipe_tower_extruder.value = 2;
        for (Printing::PrintingLayerGroup &layer : print.mutable_printing_plan().groups[0].layers)
            for (Printing::PrintingToolGroup &tool : layer.tool_groups)
                tool.extruder_id = 0;
    }
    Steps::StepGenerateWipeTower::run_step(Orchestrator::instance(), print);
    CHECK(print.printing_plan()->object_instances.empty());
    CHECK(active.first->requests.empty());
    for (const Printing::PrintingLayerGroup &layer : print.printing_plan()->groups[0].layers)
        for (const Printing::PrintingToolGroup &tool : layer.tool_groups)
            CHECK(tool.extrusions.size() == 1);
}

// Run empty output or a selected service failure against an owned plan; inspect the step exception and consume diagnostics to require the service's sole cause without a manager error.
TEST_CASE("Empty service output is not inserted and service errors retain their identity", "[plugins][wipe-tower]")
{
    WipeActivation active;
    Print print;
    build_plan(print);
    SECTION("empty roots") {
        active.first->empty = true;
        Steps::StepGenerateWipeTower::run_step(Orchestrator::instance(), print);
        CHECK(check_wipes(print) == 0);
        CHECK(active.first->bands.size() == 3);
    }
    SECTION("service error") {
        const c_wipe_tower_phase phase = GENERATE(C_WIPE_TOWER_COLLECT, C_WIPE_TOWER_GENERATE);
        active.first->fail = true;
        active.first->failure_phase = phase;
        Orchestrator &host = Orchestrator::instance();
        (void) host.consume_plugin_messages();
        CHECK_THROWS_WITH(Steps::StepGenerateWipeTower::run_step(Orchestrator::instance(), print),
                         "WIPE_TOWER_WIPE stopped after plugin 'test.wipe.low' reported an error during run.");
        const std::vector<Orchestrator::PluginMessage> messages = host.consume_plugin_messages();
        size_t errors = 0;
        for (const Orchestrator::PluginMessage &message : messages) {
            INFO(message.plugin_id << ": " << message.message);
            CHECK(message.message.find("Wipe service ") == std::string::npos);
            if (message.level != Orchestrator::PluginMessageLevel::Error)
                continue;
            ++errors;
            CHECK(message.plugin_id == "test.wipe.low");
            CHECK(message.message == "Intentional wipe service failure.");
        }
        CHECK(errors == 1);
    }
}

// Inspect a two-part request built from supplied anchors: the manager saves speed as the first leaf
// of the outgoing root and restores it as the last leaf of the incoming one, with no visit event.
TEST_CASE("Manager brackets a two-part tool change with one speed save and restore", "[plugins][wipe-tower]")
{
    WipeActivation active;
    Print print;
    build_plan(print);
    active.first->supplied_requests = true;
    active.first->supplied = {{1, {0, 0, 1, 0}, {0, 0, 0, 1}, 1, 1, 8.}};
    Steps::StepGenerateWipeTower::run_step(Orchestrator::instance(), print);
    const Printing::PrintingPlan &plan = *print.printing_plan();
    CHECK(count_plan_commands(plan, ExtrusionPropertySpecialCommand::Code::SAVE_AND_RESET_SPEED_RATIO) == 1);
    CHECK(count_plan_commands(plan, ExtrusionPropertySpecialCommand::Code::RESTORE_SPEED_RATIO) == 1);
    const Printing::PrintingLayerGroup &layer = plan.groups[0].layers[0];
    REQUIRE(layer.tool_groups.size() == 2);
    CHECK_FALSE(layer.tool_groups[1].events.has_before());
    const std::vector<const Printing::PrintingExtrusion *> outgoing = wipe_bands(layer.tool_groups[0]);
    const std::vector<const Printing::PrintingExtrusion *> incoming = wipe_bands(layer.tool_groups[1]);
    REQUIRE(outgoing.size() == 1);
    REQUIRE(incoming.size() == 1);
    CHECK(child_has_command(*outgoing[0]->root, 0, ExtrusionPropertySpecialCommand::Code::SAVE_AND_RESET_SPEED_RATIO));
    CHECK(count_commands(*outgoing[0]->root, ExtrusionPropertySpecialCommand::Code::SAVE_AND_RESET_SPEED_RATIO) == 1);
    CHECK(count_commands(*outgoing[0]->root, ExtrusionPropertySpecialCommand::Code::RESTORE_SPEED_RATIO) == 0);
    CHECK(count_commands(*incoming[0]->root, ExtrusionPropertySpecialCommand::Code::SAVE_AND_RESET_SPEED_RATIO) == 0);
    CHECK(count_commands(*incoming[0]->root, ExtrusionPropertySpecialCommand::Code::RESTORE_SPEED_RATIO) == 1);
    CHECK(child_has_command(*incoming[0]->root, uint32_t(incoming[0]->root->child_count() - 1),
                            ExtrusionPropertySpecialCommand::Code::RESTORE_SPEED_RATIO));
}

// An incoming-only request (multi-nozzle without ramming) has no outgoing root: the save becomes a
// before event of the target visit, emitted before the tool-change command, and the restore closes
// the incoming root.
TEST_CASE("Manager saves before the tool change of an incoming-only request", "[plugins][wipe-tower]")
{
    WipeActivation active;
    Print print;
    build_plan(print);
    active.first->supplied_requests = true;
    active.first->supplied = {{1, {0, 0, 1, 0}, {}, 1, 0, 8.}};
    Steps::StepGenerateWipeTower::run_step(Orchestrator::instance(), print);
    const Printing::PrintingPlan &plan = *print.printing_plan();
    CHECK(count_plan_commands(plan, ExtrusionPropertySpecialCommand::Code::SAVE_AND_RESET_SPEED_RATIO) == 1);
    CHECK(count_plan_commands(plan, ExtrusionPropertySpecialCommand::Code::RESTORE_SPEED_RATIO) == 1);
    const Printing::PrintingLayerGroup &layer = plan.groups[0].layers[0];
    REQUIRE(layer.tool_groups.size() == 2);
    const Printing::PrintingToolGroup &target = layer.tool_groups[1];
    REQUIRE(target.events.has_before());
    const ExtrusionEntity &before = target.events.before();
    REQUIRE(before.child_count() == 1);
    CHECK(child_has_command(before, 0, ExtrusionPropertySpecialCommand::Code::SAVE_AND_RESET_SPEED_RATIO));
    const std::vector<const Printing::PrintingExtrusion *> incoming = wipe_bands(target);
    REQUIRE(incoming.size() == 1);
    CHECK(count_commands(*incoming[0]->root, ExtrusionPropertySpecialCommand::Code::SAVE_AND_RESET_SPEED_RATIO) == 0);
    CHECK(count_commands(*incoming[0]->root, ExtrusionPropertySpecialCommand::Code::RESTORE_SPEED_RATIO) == 1);
    CHECK(child_has_command(*incoming[0]->root, uint32_t(incoming[0]->root->child_count() - 1),
                            ExtrusionPropertySpecialCommand::Code::RESTORE_SPEED_RATIO));
}

// The outgoing-only final unload has no incoming part, so the manager saves first and restores last
// inside the same outgoing root, with no visit event anywhere.
TEST_CASE("Manager brackets an outgoing-only unload with one speed save and restore", "[plugins][wipe-tower]")
{
    WipeActivation active;
    Print print;
    build_plan(print);
    active.first->supplied_requests = true;
    active.first->supplied = {{1, {}, {0, 0, 0, 1}, 0, 1, 8.}};
    Steps::StepGenerateWipeTower::run_step(Orchestrator::instance(), print);
    const Printing::PrintingPlan &plan = *print.printing_plan();
    CHECK(count_plan_commands(plan, ExtrusionPropertySpecialCommand::Code::SAVE_AND_RESET_SPEED_RATIO) == 1);
    CHECK(count_plan_commands(plan, ExtrusionPropertySpecialCommand::Code::RESTORE_SPEED_RATIO) == 1);
    for (const Printing::PrintingLayerGroup &layer : plan.groups[0].layers)
        for (const Printing::PrintingToolGroup &tool : layer.tool_groups)
            CHECK_FALSE(tool.events.has_before());
    const Printing::PrintingToolGroup &target = plan.groups[0].layers[0].tool_groups[0];
    const std::vector<const Printing::PrintingExtrusion *> outgoing = wipe_bands(target);
    REQUIRE(outgoing.size() == 1);
    CHECK(child_has_command(*outgoing[0]->root, 0, ExtrusionPropertySpecialCommand::Code::SAVE_AND_RESET_SPEED_RATIO));
    CHECK(count_commands(*outgoing[0]->root, ExtrusionPropertySpecialCommand::Code::SAVE_AND_RESET_SPEED_RATIO) == 1);
    CHECK(count_commands(*outgoing[0]->root, ExtrusionPropertySpecialCommand::Code::RESTORE_SPEED_RATIO) == 1);
    CHECK(child_has_command(*outgoing[0]->root, uint32_t(outgoing[0]->root->child_count() - 1),
                            ExtrusionPropertySpecialCommand::Code::RESTORE_SPEED_RATIO));
}

// A request whose enabled roots all stayed empty prints nothing, so it must not leave a save in the
// target visit's before events nor any other speed command.
TEST_CASE("Manager adds no speed command for empty service output", "[plugins][wipe-tower]")
{
    WipeActivation active;
    Print print;
    build_plan(print);
    active.first->empty = true;
    active.first->supplied_requests = true;
    active.first->supplied = {{1, {0, 0, 1, 0}, {0, 0, 0, 1}, 1, 1, 8.}};
    Steps::StepGenerateWipeTower::run_step(Orchestrator::instance(), print);
    const Printing::PrintingPlan &plan = *print.printing_plan();
    CHECK(count_plan_commands(plan, ExtrusionPropertySpecialCommand::Code::SAVE_AND_RESET_SPEED_RATIO) == 0);
    CHECK(count_plan_commands(plan, ExtrusionPropertySpecialCommand::Code::RESTORE_SPEED_RATIO) == 0);
    for (const Printing::PrintingGroup &group : plan.groups)
        for (const Printing::PrintingLayerGroup &layer : group.layers)
            for (const Printing::PrintingToolGroup &tool : layer.tool_groups)
                CHECK_FALSE(tool.events.has_before());
}

// A supplied floating request above the tower top is inserted on its own layer at that Z, while the
// regular request below keeps the tower and its maintenance.
TEST_CASE("Manager floats a supplied request above the tower top without maintenance", "[plugins][wipe-tower]")
{
    WipeActivation active;
    Print print;
    build_plan(print);
    active.first->supplied_requests = true;
    active.first->supplied = {{1, {0, 0, 1, 0}, {}, 1, 0, 8., 0}, {2, {}, {0, 1, 0, 1}, 0, 1, 8., 1}};
    Steps::StepGenerateWipeTower::run_step(Orchestrator::instance(), print);
    const Printing::PrintingPlan &plan = *print.printing_plan();
    const Printing::PrintingLayerGroup &lower = plan.groups[0].layers[0];
    const Printing::PrintingLayerGroup &upper = plan.groups[0].layers[1];
    // The regular band keeps its tower layer and maintenance.
    REQUIRE(wipe_bands(lower.tool_groups[1]).size() == 1);
    CHECK(maintenance_paths(lower).size() == 1);
    // Above it, only the floating band is inserted at the upper layer's Z, with no maintenance.
    const std::vector<const Printing::PrintingExtrusion *> floating = wipe_bands(upper.tool_groups[0]);
    REQUIRE(floating.size() == 1);
    CHECK(maintenance_paths(upper).empty());
    CHECK(floating[0]->root->get_property<ExtrusionPropertyZOffset>() == nullptr);
    // Both requests keep the manager's one speed bracket each.
    CHECK(count_plan_commands(plan, ExtrusionPropertySpecialCommand::Code::SAVE_AND_RESET_SPEED_RATIO) == 2);
    CHECK(count_plan_commands(plan, ExtrusionPropertySpecialCommand::Code::RESTORE_SPEED_RATIO) == 2);
}

// Corrupt one plan-owned maintenance root or its metadata, checking each validation failure names the exact extrusion while retaining every independent provenance/flag assertion.
TEST_CASE("Wipe post-validation rejects malformed synthetic provenance and root flags", "[plugins][wipe-tower]")
{
    WipeActivation active;
    Print print;
    build_plan(print);
    Steps::StepGenerateWipeTower::run_step(Orchestrator::instance(), print);
    REQUIRE(print.printing_plan()->object_instances.size() == 1);
    REQUIRE(print.printing_plan()->groups[0].layers[0].tool_groups[1].extrusions.size() == 3);
    Printing::PrintingExtrusion &wipe = print.mutable_printing_plan().groups[0].layers[0].tool_groups[1].extrusions[0];
    SECTION("unknown catalog id") { wipe.printing_catalog_id = UINT32_MAX; }
    SECTION("real catalog entry") { print.mutable_printing_plan().object_instances[0].object = reinterpret_cast<const PrintObject *>(&print); }
    SECTION("incomplete catalog metadata") { print.mutable_printing_plan().object_instances[0].name.clear(); }
    SECTION("source island") { wipe.region_island = reinterpret_cast<const LayerRegionIsland *>(&print); }
    SECTION("reversible root") { wipe.root->set_can_sort_reverse(false, true); }
    SECTION("sortable root") { wipe.root->set_can_sort_reverse(true, false); }
    SECTION("missing root") { wipe.root.reset(); }
    SECTION("wipe role on child only") {
        wipe.sregion_island_role = ExtrusionRole::Perimeter;
        wipe.printing_catalog_id = UINT32_MAX;
    }
    std::string error;
    CHECK_FALSE(Steps::StepGenerateWipeTower::validate_post(print, &error));
    CHECK(error.find("group[0].layer[0].tool[1].extrusion[0]") != std::string::npos);
}

// Borrow inserted maintenance and a source extrusion, checking synthetic classification before/after a real-owner corruption; missing real region data must still throw with no source ownership fabricated.
TEST_CASE("Process settings distinguish synthetic wipes from missing real source islands", "[plugins][wipe-tower]")
{
    WipeActivation active;
    Print print;
    build_plan(print);
    Steps::StepGenerateWipeTower::run_step(Orchestrator::instance(), print);
    slic3r_api::PrintingPlan plan = Test::GCode::printing_plan_view(print);
    slic3r_api::PrintingToolGroup tool = plan.group(0).layer_group(0).tool_group(1);
    const slic3r_api::PrintingExtrusion wipe = tool.extrusion(0);
    using namespace slic3r_api::LayerExtrusionEdit::ProcessParameterHelpers;
    CHECK(is_regionless_synthetic(plan, wipe));
    CHECK_FALSE(is_regionless_synthetic(plan, tool.extrusion(2)));
    // A real catalog owner without a source island must still reach source validation.
    print.mutable_printing_plan().object_instances[0].object = reinterpret_cast<const PrintObject *>(&print);
    CHECK_FALSE(is_regionless_synthetic(plan, wipe));
    CHECK_THROWS_WITH(settings_context(slic3r_api::Print(reinterpret_cast<const print_handle *>(&print)), tool, wipe),
                      "Printing extrusion has no source LayerRegionIsland region.");
    print.mutable_printing_plan().object_instances[0].object = nullptr;
}

// Slice owned two-tool models in shared or restarting-Z groups and export in the checkout, checking maintenance and elevated bands through real downstream plugins; preparation/export failures remain assertions.
TEST_CASE("Sliced two-extruder ordering reaches the wipe manager and full G-code export", "[plugins][wipe-tower]")
{
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    WipeActivation active;
    Print print;
    Model model;
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    const bool parallel = GENERATE(false, true);
    config.set_deserialize_strict("parallel_objects_step", parallel ? "5" : "0");
    config.set_num_extruders(2);
    config.set_deserialize_strict("step_perimeter_plugin", "perimeter.generator.classic");
    config.set_deserialize_strict("step_surface_generation_plugin", "surface.initial_typed_surface_builder");
    config.set_deserialize_strict("step_wipetower_plugin", "wipe_tower.manager.default");
    config.set_deserialize_strict("nozzle_diameter", "0.4,0.4");
    config.set_deserialize_strict("filament_diameter", "1.75,1.75");
    config.set_deserialize_strict("extrusion_multiplier", "1,1");
    config.set_deserialize_strict("wipe_tower", "1");
    config.set_deserialize_strict("wipe_tower_x", "40");
    config.set_deserialize_strict("wipe_tower_y", "60");
    config.set_deserialize_strict("wipe_tower_width", "20");
    config.set_deserialize_strict("wipe_tower_rotation_angle", "90");
    config.set_deserialize_strict("perimeter_extruder", "1");
    config.set_deserialize_strict("infill_extruder", "1");
    config.set_deserialize_strict("solid_infill_extruder", "1");
    config.set_deserialize_strict("fill_density", "10%");
    config.set_deserialize_strict("layer_height", "0.3");
    Test::init_print({Test::TestMesh::cube_20x20x20, Test::TestMesh::cube_20x20x20}, print, model, config);
    // Assign each complete object's perimeter and infill to the same concrete tool.
    model.objects()[1].config.set_deserialize_strict("perimeter_extruder", "2");
    model.objects()[1].config.set_deserialize_strict("infill_extruder", "2");
    model.objects()[1].config.set_deserialize_strict("solid_infill_extruder", "2");
    print.apply(model, config);
    try {
        print.process();
    } catch (...) {
        // Keep source geometry counts visible when the sliced-print preparation fails.
        size_t islands = 0;
        size_t fill_areas = 0;
        size_t region_islands = 0;
        for (const Layer &layer : print.object(0).layers())
            for (const LayerSliceIsland &island : layer.islands()) {
                ++islands;
                fill_areas += island.infill_areas().size();
                region_islands += island.regions_islands().size();
            }
        WARN("Source layers=" << print.object(0).layer_count() << " islands=" << islands <<
             " fill areas=" << fill_areas << " region islands=" << region_islands);
        throw;
    }
    // Direct step calls have no export progress plan; the normal export initializes its own accounting below.
    print.pipeline_progress().reset();
    Steps::StepExtrusionOrdering::run_step(Orchestrator::instance(), print);
    Steps::StepGenerateWipeTower::run_step(Orchestrator::instance(), print);
    REQUIRE_FALSE(active.first->requests.empty());
    CHECK(check_wipes(print) == active.first->requests.size());
    std::string error;
    CHECK(Steps::StepGenerateWipeTower::validate_post(print, &error));
    if (parallel) {
        // A leading skirt/brim batch precedes the two restarting-Z object batches.
        REQUIRE(print.printing_plan()->groups.size() == 3);
        CHECK(print.has_global_wipe_tower());
        REQUIRE(print.printing_plan()->object_instances.size() == 3);
        CHECK(active.first->bands.back().z > 0.2);
    }

    // Export replays ordering and wipe generation normally, then every downstream plugin.
    // Compiler source paths are absolute in this build, so the export remains
    // inside the checkout even when CTest runs from its binary test directory.
    const boost::filesystem::path checkout = boost::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
    REQUIRE(checkout.is_absolute());
    const std::string path = (checkout / "build" / "temp" / "wipe_tower_smoke.gcode").string();
    REQUIRE_NOTHROW(Orchestrator::instance().export_gcode(print, path, nullptr));
    const std::string gcode = Test::GCode::read_text_file(path);
    CHECK(gcode.find("Wipe tower") != std::string::npos);
    CHECK(gcode.find("T1") != std::string::npos);
    // Service paths are vertical after the 90-degree rotation. A positive-E
    // horizontal move within the tower bounds can only come from maintenance.
    bool exported_maintenance = false;
    GCodeReader reader;
    reader.parse_buffer(gcode, [&exported_maintenance](GCodeReader &state, const GCodeReader::GCodeLine &line) {
        if (line.extruding(state) && line.dist_X(state) != 0. &&
            state.x() >= 19. && state.x() <= 41. && state.y() >= 59. && state.y() <= 81. &&
            line.new_X(state) >= 19. && line.new_X(state) <= 41. && line.new_Y(state) >= 59. && line.new_Y(state) <= 81.)
            exported_maintenance = true;
    });
    CHECK(exported_maintenance);
}

// Build two owned restarting-Z batches with supplied requests, inspect fused/stacked bands and root offsets, then use the real file writer to require separate no-E approach/return Z moves before extrusion.
TEST_CASE("Parallel groups share one global tower with fused and stacked Z offsets", "[plugins][wipe-tower]")
{
    WipeActivation active;
    Print print;
    build_plan(print);
    const_cast<PrintConfig &>(print.config()).parallel_objects_step.value = 5.;
    slic3r_api::PrintingPlan view = Test::GCode::printing_plan_view(print);
    view.clear();
    for (uint32_t g = 0; g < 2; ++g) {
        slic3r_api::PrintingGroup group = view.append_group();
        for (const double z : {0.2, 0.4}) {
            slic3r_api::PrintingLayerGroup layer = group.append_layer_group(scale_i(z));
            attach_source_layer(print, layer, 0.2);
            slic3r_api::PrintingToolGroup tool = layer.append_tool_group(0);
            std::unique_ptr<ExtrusionPath> path = Test::GCode::make_firmware_path(
                ArcPolyline(Points{Point::new_scale(1., 1.), Point::new_scale(2., 1.)}), 20.f, 500.f);
            REQUIRE(printing_tool_group_append_extrusion_clone(view.handle(), tool.mutable_handle(), nullptr,
                RAW_EXTRUSION_ROLE_PERIMETER, reinterpret_cast<const extrusion_entity_handle *>(path.get()), UINT32_MAX) != nullptr);
        }
    }
    active.first->supplied_requests = true;
    active.first->supplied = {{1, {0, 0, 0, 0}, {}, 1, 0, 8.}, {2, {0, 1, 0, 0}, {}, 1, 0, 16.},
        {3, {1, 0, 0, 0}, {}, 1, 0, 8.}, {4, {1, 1, 0, 0}, {}, 1, 0, 80.}};
    REQUIRE_NOTHROW(Steps::StepGenerateWipeTower::run_step(Orchestrator::instance(), print));
    REQUIRE(active.first->bands.size() == 4);
    // Each lower-Z batch stacks above the previous tower top; the bands keep print order.
    const double zs[] = {0.2, 0.4, 0.6, 0.8};
    for (size_t i = 0; i < 4; ++i) {
        CHECK(active.first->bands[i].z == Approx(zs[i]));
        CHECK(active.first->bands[i].height == Approx(0.2));
        CHECK(active.first->bands[i].catalog_id == active.first->bands[0].catalog_id);
    }
    REQUIRE(print.printing_plan()->object_instances.size() == 1);
    CHECK(maintenance_paths(print.printing_plan()->groups[1].layers[0]).size() == 1);
    REQUIRE(maintenance_paths(print.printing_plan()->groups[1].layers[1]).size() == 1);
    for (uint32_t l = 0; l < 2; ++l) {
        slic3r_api::PrintingToolGroup tool = view.group(1).layer_group(l).tool_group(0);
        for (uint32_t e = 0; e + 1 < tool.extrusion_count(); ++e) {
            const slic3r_api::EPropertyZOffset *offset = tool.extrusion(e).root().get(slic3r_api::EPropertyZOffset::key);
            REQUIRE(offset != nullptr);
            CHECK(offset->get() == scale_i(0.4));
        }
        CHECK(tool.extrusion(tool.extrusion_count() - 1).root().get(slic3r_api::EPropertyZOffset::key) == nullptr);
    }
    Test::GCode::select_printing_plan_firmware(print, "gcode.firmware.marlin2");
    const boost::filesystem::path checkout = boost::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
    const std::string path = (checkout / "build" / "temp" / "wipe_tower_parallel.gcode").string();
    REQUIRE_NOTHROW(Steps::StepGenerateGcode::run_step(Orchestrator::instance(), print, path));
    const std::string gcode = Test::GCode::read_text_file(path);
    bool raised = false;
    bool returned = false;
    bool elevated_extrusion = false;
    GCodeReader reader;
    reader.parse_buffer(gcode, [&raised, &returned, &elevated_extrusion](GCodeReader &state, const GCodeReader::GCodeLine &line) {
        if (line.cmd_is("G0") && line.has(Z) && !line.has(E) && line.new_Z(state) > state.z())
            raised = true;
        if (line.cmd_is("G0") && line.has(Z) && !line.has(E) && state.z() > 0.59 && line.new_Z(state) < 0.41)
            returned = true;
        if (line.extruding(state) && state.z() > 0.59) {
            CHECK(raised);
            CHECK_FALSE(line.has(Z));
            elevated_extrusion = true;
        }
    });
    CHECK(raised);
    CHECK(returned);
    CHECK(elevated_extrusion);
}

// Borrow multiple source layers at a close support Z, checking that their minimum height drives fusion and that a single maintenance root serves the shared top layer without extra catalog entries.
TEST_CASE("Close source groups share a tower layer using their thinnest source height", "[plugins][wipe-tower]")
{
    WipeActivation active;
    Print print;
    build_plan(print);
    print.mutable_printing_plan().groups[0].layers[1].print_z = scale_i(0.3);
    slic3r_api::PrintingPlan view = Test::GCode::printing_plan_view(print);
    attach_source_layer(print, view.group(0).layer_group(1), 0.2);
    active.first->supplied_requests = true;
    active.first->supplied = {{1, {0, 0, 0, 0}, {}, 1, 0, 8.}, {2, {0, 1, 0, 0}, {}, 1, 0, 8.}};
    Steps::StepGenerateWipeTower::run_step(Orchestrator::instance(), print);
    REQUIRE(active.first->bands.size() == 2);
    CHECK(active.first->bands[0].z == Approx(0.2));
    CHECK(active.first->bands[1].z == Approx(0.2));
    CHECK(active.first->bands[1].height == Approx(0.2));
    CHECK(maintenance_paths(print.printing_plan()->groups[0].layers[0]).size() == 1);
    CHECK(maintenance_paths(print.printing_plan()->groups[0].layers[1]).empty());
    const slic3r_api::EPropertyZOffset *offset = view.group(0).layer_group(1).tool_group(0).extrusion(0).root().get(slic3r_api::EPropertyZOffset::key);
    REQUIRE(offset != nullptr);
    CHECK(offset->get() == scale_i(-0.1));
}

// Generate a single late request against an owned four-layer plan, checking maintenance continuity, first-layer brim/solid flow and no output above the last request.
TEST_CASE("Global tower maintenance spans the first layer through the last request", "[plugins][wipe-tower]")
{
    WipeActivation active;
    Print print;
    build_plan(print);
    slic3r_api::PrintingPlan view = Test::GCode::printing_plan_view(print);
    for (const double z : {0.8, 1.1}) {
        slic3r_api::PrintingLayerGroup layer = view.group(0).append_layer_group(scale_i(z));
        attach_source_layer(print, layer, 0.3);
        for (const extruder_index_t tool_id : {extruder_index_t(0), extruder_index_t(1)}) {
            slic3r_api::PrintingToolGroup tool = layer.append_tool_group(tool_id);
            std::unique_ptr<ExtrusionPath> path = Test::GCode::make_firmware_path(
                ArcPolyline(Points{Point::new_scale(1., 1.), Point::new_scale(2., 1.)}), 20.f, 500.f);
            REQUIRE(printing_tool_group_append_extrusion_clone(view.handle(), tool.mutable_handle(), nullptr,
                RAW_EXTRUSION_ROLE_PERIMETER, reinterpret_cast<const extrusion_entity_handle *>(path.get()), UINT32_MAX) != nullptr);
        }
    }
    PrintObjectConfig &config = const_cast<PrintObjectConfig &>(print.default_object_config());
    config.wipe_tower_rotation_angle.value = 0.;
    config.wipe_tower_brim_width.value = 1.;
    config.wipe_tower_brim_width.percent = false;
    const_cast<PrintConfig &>(print.config()).wipe_tower_speed.value = 50.;
    const_cast<DynamicPrintConfig &>(print.full_print_config()).set_deserialize_strict("first_layer_speed", "50%");
    active.first->supplied_requests = true;
    active.first->supplied.push_back({1, {0, 2, 1, 0}, {}, 1, 0, 8.});
    Steps::StepGenerateWipeTower::run_step(Orchestrator::instance(), print);
    REQUIRE(active.first->bands.size() == 1);
    for (size_t l = 0; l < 4; ++l) {
        const Printing::PrintingLayerGroup &layer = print.printing_plan()->groups[0].layers[l];
        const std::vector<const Printing::PrintingExtrusion *> paths = maintenance_paths(layer);
        REQUIRE(paths.size() == (l < 3 ? 1 : 0));
        if (l == 3)
            continue;
        const Printing::PrintingToolGroup &tool = layer.tool_groups[l == 2 ? 1 : 0];
        CHECK(&tool.extrusions[0] == paths[0]);
        const ExtrusionEntity &root = *paths[0]->root;
        // Layers 0-1 keep a fill area; the band layer's depth equals its used depth, so its
        // sparse area is empty and only its perimeter remains.
        REQUIRE(root.child_count() >= 1);
        if (l == 2)
            CHECK(root.child_count() == 1);
        else
            CHECK(root.child_count() > 1);
        double fill_length = 0.;
        for (size_t c = 0; c < root.child_count(); ++c) {
            const ExtrusionEntity &path = root.child(c);
            REQUIRE(path.get_property<ExtrusionAttributes>() != nullptr);
            CHECK(path.get_property<ExtrusionAttributes>()->height == Approx(l == 0 ? 0.2 : 0.3));
            REQUIRE(path.get_property<ExtrusionPropertySpeed>() != nullptr);
            CHECK(path.get_property<ExtrusionPropertySpeed>()->speed_mm_per_s == Approx(l == 0 ? 25. : 50.));
            CHECK_FALSE(path.can_reverse());
            fill_length += path.length();
        }
        if (l == 0) {
            const Polyline brim = root.child(0).as_polyline().to_polyline();
            coord_t min_x = brim.points.front().x();
            coord_t max_x = min_x;
            for (const Point &point : brim.points) {
                min_x = std::min(min_x, point.x());
                max_x = std::max(max_x, point.x());
            }
            CHECK(min_x < scale_i(40.));
            CHECK(max_x > scale_i(60.));
            CHECK(brim.points.front() == brim.points.back());
            // The depth follows the band usage, so the solid fill spans the thin first layer.
            CHECK(fill_length > scale_d(160.));
        } else {
            CHECK(root.child(0).first_point() == Point::new_scale(40., 60.));
            CHECK(fill_length < scale_d(500.));
        }
    }
}

// Use an original cross-layer change to bound forced visits in owned lower layers, checking EDIT visibility, minimum-cost placement, disabled fallback and empty-visit cleanup; real downstream selection failures remain visible.
TEST_CASE("Manager contour visits precede service edits and obey the toolchange flag", "[plugins][wipe-tower]")
{
    WipeActivation active;
    Print print;
    build_plan(print);
    for (Printing::PrintingLayerGroup &layer : print.mutable_printing_plan().groups[0].layers)
        layer.tool_groups.erase(layer.tool_groups.begin() + 1);
    slic3r_api::PrintingPlan view = Test::GCode::printing_plan_view(print);
    slic3r_api::PrintingLayerGroup upper = view.group(0).append_layer_group(scale_i(0.8));
    attach_source_layer(print, upper, 0.3);
    upper.append_tool_group(1);
    PrintObjectConfig &config = const_cast<PrintObjectConfig &>(print.default_object_config());
    config.wipe_tower_extruder.value = 2;
    bool forced = true;
    bool no_requests = false;
    SECTION("forced tool visible to services") {}
    SECTION("disabled forced change uses first visit") {
        config.wipe_tower_extruder_toolchange.value = false;
        forced = false;
        active.first->supplied_requests = true;
        active.first->supplied.push_back({1, {0, 1, 0, 0}, {}, 1, 0, 8.});
    }
    SECTION("unused marked visits are removed") {
        active.first->supplied_requests = true;
        no_requests = true;
    }
    Steps::StepGenerateWipeTower::run_step(Orchestrator::instance(), print);
    REQUIRE(active.first->edit_tools.size() == 3);
    CHECK(active.first->edit_tools[0] == (forced ? std::vector<extruder_index_t>{1, 0} : std::vector<extruder_index_t>{0}));
    for (size_t l = 0; l < 2; ++l) {
        const Printing::PrintingLayerGroup &layer = print.printing_plan()->groups[0].layers[l];
        REQUIRE(layer.tool_groups.size() == (forced && !no_requests ? 2 : 1));
        const std::vector<const Printing::PrintingExtrusion *> paths = maintenance_paths(layer);
        REQUIRE(paths.size() == (no_requests ? 0 : 1));
        if (!no_requests) {
            const size_t visit = forced && l != 0 ? 1 : 0;
            CHECK(layer.tool_groups[visit].extruder_id == (forced ? 1 : 0));
            CHECK(&layer.tool_groups[visit].extrusions[0] == paths[0]);
        }
    }
    const Printing::PrintingLayerGroup &last = print.printing_plan()->groups[0].layers[2];
    REQUIRE(last.tool_groups.size() == 1);
    CHECK(last.tool_groups[0].extruder_id == 1);
    CHECK(last.tool_groups[0].extrusions.empty());
    CHECK(maintenance_paths(last).empty());
    if (forced && !no_requests) {
        Test::TransitionPipeline::run_layer_plugins(print, {Test::TransitionPipeline::TOOLCHANGE_PLUGIN});
        CHECK_FALSE(print.printing_plan()->groups[0].layers[0].tool_groups[0].events.has_before());
        CHECK(print.printing_plan()->groups[0].layers[0].tool_groups[1].events.has_before());
        CHECK(print.printing_plan()->groups[0].layers[1].tool_groups[1].events.has_before());
    }
}

// Route borrowed source ids to printed instance towers across shared/sequential groups, checking appended tower IDs, origins, heights and absence of an unused global catalog entry; outputs remain plan-owned.
TEST_CASE("Manager selects independently catalogued instance towers", "[plugins][wipe-tower]")
{
    const bool separate = GENERATE(false, true);
    WipeActivation active;
    Print print;
    Model model;
    build_instance_plan(print, model, separate, true);
    REQUIRE(print.printing_plan()->object_instances.size() == 2);
    const PrintObject *owner = print.printing_plan()->object_instances[0].object;
    REQUIRE(owner != nullptr);
    CHECK(print.printing_plan()->object_instances[1].object == owner);
    active.first->supplied_requests = true;
    active.first->supplied = separate ? std::vector<c_wipe_tower_request>{
        {1, {0, 1, 0, 0}, {}, 1, 0, 8.}, {2, {1, 1, 0, 0}, {}, 1, 0, 8.}} :
        std::vector<c_wipe_tower_request>{{1, {0, 1, 0, 0}, {}, 1, 0, 8.}, {2, {0, 1, 1, 0}, {}, 1, 0, 8.}};
    Steps::StepGenerateWipeTower::run_step(Orchestrator::instance(), print);
    REQUIRE(active.first->bands.size() == 2);
    const uint32_t first_id = active.first->bands[0].catalog_id;
    const uint32_t second_id = active.first->bands[1].catalog_id;
    CHECK(first_id != second_id);
    const Printing::PrintingPlan &plan = *print.printing_plan();
    REQUIRE(plan.object_instances.size() == 4);
    CHECK(first_id == 2);
    CHECK(second_id == 3);
    CHECK(plan.object_instances[0].object == owner);
    CHECK(plan.object_instances[0].object_instance_idx == 0);
    CHECK(plan.object_instances[1].object == owner);
    CHECK(plan.object_instances[1].object_instance_idx == 1);
    CHECK(plan.object_instances[first_id].name == "Wipe tower Owner #1");
    CHECK(plan.object_instances[second_id].name == "Wipe tower Owner #2");
    CHECK(plan.object_instances[first_id].projected_outline_center == Point::new_scale(70., 30.));
    CHECK(plan.object_instances[second_id].projected_outline_center == Point::new_scale(170., 30.));
    const std::vector<WipeTowerFootprint> footprints = wipe_tower_footprints(print);
    for (const Printing::PrintingGroup &group : plan.groups) {
        REQUIRE(group.layers.size() == 2);
        for (size_t l = 0; l < group.layers.size(); ++l) {
            const std::vector<const Printing::PrintingExtrusion *> paths = maintenance_paths(group.layers[l]);
            REQUIRE(paths.size() == (separate ? 1 : 2));
            for (const Printing::PrintingExtrusion *maintenance : paths) {
                const Printing::PrintingObjectInstance &catalog = plan.object_instances[maintenance->printing_catalog_id];
                CHECK(catalog.object == nullptr);
                CHECK(catalog.object_instance_idx == uint64_t(-1));
                CHECK(maintenance->root->child(0).get_property<ExtrusionAttributes>()->height == Approx(0.2));
                CHECK(std::find(group.printing_catalog_ids.begin(), group.printing_catalog_ids.end(), maintenance->printing_catalog_id) != group.printing_catalog_ids.end());
                const uint32_t owner = maintenance->printing_catalog_id == first_id ? 0 : 1;
                CHECK(std::find(group.printing_catalog_ids.begin(), group.printing_catalog_ids.end(), owner) != group.printing_catalog_ids.end());
                const WipeTowerFootprint &footprint = footprints[(separate ? 0 : 1) + owner];
                CHECK(catalog.projected_outline.points == footprint.polygon.points);
            }
        }
    }
}

// Route two borrowed owners to a mix of explicit-instance and global towers, requiring distinct printed catalogs while an unrequested instance stays absent; source ownership and output geometry remain plan-owned.
TEST_CASE("Global and instance towers coexist without cataloguing unused towers", "[plugins][wipe-tower]")
{
    WipeActivation active;
    Print print;
    Model model;
    build_instance_plan(print, model, false, true);
    print.mutable_printing_plan().groups[0].layers[1].tool_groups[1].extrusions[0].printing_catalog_id = UINT32_MAX;
    active.first->supplied_requests = true;
    active.first->supplied = {{1, {0, 1, 0, 0}, {}, 1, 0, 8.}, {2, {0, 1, 1, 0}, {}, 1, 0, 8.}};
    Steps::StepGenerateWipeTower::run_step(Orchestrator::instance(), print);
    REQUIRE(active.first->bands.size() == 2);
    const Printing::PrintingPlan &plan = *print.printing_plan();
    REQUIRE(plan.object_instances.size() == 4);
    const uint32_t instance = active.first->bands[0].catalog_id;
    const uint32_t global = active.first->bands[1].catalog_id;
    CHECK(instance != global);
    CHECK(plan.object_instances[instance].name == "Wipe tower Owner #1");
    CHECK(plan.object_instances[global].name == "Wipe tower");
    CHECK(plan.groups[0].printing_catalog_ids.size() == 4);
    for (const Printing::PrintingObjectInstance &catalog : plan.object_instances)
        CHECK(catalog.name != "Wipe tower Owner #2");
    for (const Printing::PrintingLayerGroup &layer : plan.groups[0].layers)
        CHECK(maintenance_paths(layer).size() == 2);
}

// Inspect incoming-first and outgoing-end anchors against borrowed owners, requiring only the selected instance/global catalog entry or a silent drop when neither tower exists; source ownership remains unchanged.
TEST_CASE("Tower choice at end anchors stays in the layer and supports global fallback", "[plugins][wipe-tower]")
{
    WipeActivation active;
    Print print;
    Model model;
    build_instance_plan(print, model, false, true);
    bool fallback = false;
    bool dropped = false;
    bool incoming = false;
    uint32_t tool = 0;
    SECTION("following visit owns the tower") {}
    SECTION("last end anchor uses previous extrusion") { tool = 1; }
    SECTION("incoming anchor uses the visit's first owner rather than the anchor's following owner") {
        incoming = true;
        slic3r_api::PrintingPlan view = Test::GCode::printing_plan_view(print);
        slic3r_api::PrintingToolGroup visit = view.group(0).layer_group(1).tool_group(0);
        std::unique_ptr<ExtrusionPath> path = Test::GCode::make_firmware_path(
            ArcPolyline(Points{Point::new_scale(110., 20.), Point::new_scale(111., 20.)}), 20.f, 500.f);
        REQUIRE(printing_tool_group_append_extrusion_clone(view.handle(), visit.mutable_handle(), nullptr,
            RAW_EXTRUSION_ROLE_PERIMETER, reinterpret_cast<const extrusion_entity_handle *>(path.get()), 1) != nullptr);
    }
    SECTION("source-less following extrusion uses global tower") {
        print.mutable_printing_plan().groups[0].layers[1].tool_groups[1].extrusions[0].printing_catalog_id = UINT32_MAX;
        fallback = true;
    }
    SECTION("no tower for source-less following extrusion drops request") {
        const_cast<PrintObjectConfig &>(print.default_object_config()).wipe_tower.value = false;
        print.mutable_printing_plan().groups[0].layers[1].tool_groups[1].extrusions[0].printing_catalog_id = UINT32_MAX;
        dropped = true;
    }
    active.first->supplied_requests = true;
    active.first->supplied = incoming ? std::vector<c_wipe_tower_request>{{1, {0, 1, 0, 1}, {}, 1, 0, 8.}} :
        std::vector<c_wipe_tower_request>{{1, {}, {0, 1, tool, 1}, 0, 1, 8.}};
    Steps::StepGenerateWipeTower::run_step(Orchestrator::instance(), print);
    if (dropped) {
        CHECK(active.first->bands.empty());
        CHECK(print.printing_plan()->object_instances.size() == 2);
        CHECK(maintenance_paths(print.printing_plan()->groups[0].layers[1]).empty());
    } else {
        REQUIRE(active.first->bands.size() == 1);
        REQUIRE(print.printing_plan()->object_instances.size() == 3);
        const uint32_t id = active.first->bands[0].catalog_id;
        CHECK(print.printing_plan()->object_instances[id].name == (fallback ? "Wipe tower" :
            incoming ? "Wipe tower Owner #1" : "Wipe tower Owner #2"));
    }
}

// Select safe skipped layers in a compact owned plan and inspect the actual generated flow thickness; resolved nozzle percentages bound accumulated gaps without removing requested bands.
TEST_CASE("Manager sparse-layer skipping produces thicker bounded maintenance", "[plugins][wipe-tower]")
{
    WipeActivation active;
    Print print;
    build_plan(print);
    slic3r_api::PrintingPlan view = Test::GCode::printing_plan_view(print);
    view.clear();
    slic3r_api::PrintingGroup group = view.append_group();
    for (uint32_t n = 1; n <= 7; ++n) {
        slic3r_api::PrintingLayerGroup layer = group.append_layer_group(scale_i(0.1 * n));
        attach_source_layer(print, layer, 0.1);
        layer.append_tool_group(0);
    }
    const_cast<PrintConfig &>(print.config()).wipe_tower_no_sparse_layers.value = true;
    const_cast<DynamicPrintConfig &>(print.full_print_config()).set_deserialize_strict("max_layer_height", "75%,75%");
    active.first->supplied_requests = true;
    active.first->supplied = {{1, {0, 3, 0, 0}, {}, 1, 0, 8.}, {2, {0, 6, 0, 0}, {}, 1, 0, 8.}};
    Steps::StepGenerateWipeTower::run_step(Orchestrator::instance(), print);
    REQUIRE(active.first->bands.size() == 2);
    CHECK(active.first->bands[0].height == Approx(0.3));
    CHECK(active.first->bands[1].height == Approx(0.3));
    for (size_t l = 0; l < 7; ++l) {
        const std::vector<const Printing::PrintingExtrusion *> paths = maintenance_paths(print.printing_plan()->groups[0].layers[l]);
        REQUIRE(paths.size() == (l == 0 || l == 3 || l == 6 ? 1 : 0));
        if (!paths.empty()) {
            REQUIRE(paths[0]->root->child(0).get_property<ExtrusionAttributes>() != nullptr);
            CHECK(paths[0]->root->child(0).get_property<ExtrusionAttributes>()->height == Approx(l == 0 ? 0.1 : 0.3));
        }
    }
}

// Bound forced visits by an original cross-layer change but collect an earlier request, proving later empty-visit cleanup retains owned source paths and post-EDIT anchors.
TEST_CASE("Manager removes unused forced contour visits above the last request", "[plugins][wipe-tower]")
{
    WipeActivation active;
    Print print;
    build_plan(print);
    for (Printing::PrintingLayerGroup &layer : print.mutable_printing_plan().groups[0].layers)
        layer.tool_groups.erase(layer.tool_groups.begin() + 1);
    slic3r_api::PrintingPlan view = Test::GCode::printing_plan_view(print);
    slic3r_api::PrintingLayerGroup upper = view.group(0).append_layer_group(scale_i(0.8));
    attach_source_layer(print, upper, 0.3);
    upper.append_tool_group(1);
    const_cast<PrintObjectConfig &>(print.default_object_config()).wipe_tower_extruder.value = 2;
    const ExtrusionEntity *retained = print.printing_plan()->groups[0].layers[1].tool_groups[0].extrusions[0].root.get();
    active.first->supplied_requests = true;
    active.first->supplied = {{1, {0, 0, 1, 0}, {}, 1, 0, 8.}};
    Steps::StepGenerateWipeTower::run_step(Orchestrator::instance(), print);
    REQUIRE(active.first->edit_tools.size() == 3);
    CHECK(active.first->edit_tools[1].size() == 2);
    CHECK(print.printing_plan()->groups[0].layers[0].tool_groups.size() == 2);
    const Printing::PrintingLayerGroup &last = print.printing_plan()->groups[0].layers[1];
    REQUIRE(last.tool_groups.size() == 1);
    REQUIRE(last.tool_groups[0].extrusions.size() == 1);
    CHECK(last.tool_groups[0].extrusions[0].root.get() == retained);
    CHECK(maintenance_paths(last).empty());
}

// Keep real changes in the two lower source layers, requiring no forced visits or tower above their original cutoff; a supplied later request may extend maintenance only on the existing tool, with assertions exposing all geometry.
TEST_CASE("Forced contour visits stop at the last original tool-change layer", "[plugins][wipe-tower]")
{
    WipeActivation active;
    Print print;
    build_plan(print);
    // The last original change is T1 -> T0 at the second layer boundary.
    print.mutable_printing_plan().groups[0].layers[1].tool_groups.erase(
        print.mutable_printing_plan().groups[0].layers[1].tool_groups.begin() + 1);
    slic3r_api::PrintingPlan view = Test::GCode::printing_plan_view(print);
    for (const double z : {0.8, 1.1}) {
        slic3r_api::PrintingLayerGroup layer = view.group(0).append_layer_group(scale_i(z));
        attach_source_layer(print, layer, 0.3);
        slic3r_api::PrintingToolGroup tool = layer.append_tool_group(0);
        std::unique_ptr<ExtrusionPath> path = Test::GCode::make_firmware_path(
            ArcPolyline(Points{Point::new_scale(1., 1.), Point::new_scale(2., 1.)}), 20.f, 500.f);
        REQUIRE(printing_tool_group_append_extrusion_clone(view.handle(), tool.mutable_handle(), nullptr,
            RAW_EXTRUSION_ROLE_PERIMETER, reinterpret_cast<const extrusion_entity_handle *>(path.get()), UINT32_MAX) != nullptr);
    }
    const_cast<PrintObjectConfig &>(print.default_object_config()).wipe_tower_extruder.value = 2;
    bool late_request = false;
    SECTION("real tool changes only") {}
    SECTION("later non-tool-change wipe keeps the existing tool") {
        late_request = true;
        active.first->supplied_requests = true;
        active.first->supplied = {{1, {0, 3, 0, 0}, {}, 1, 0, 8.}};
    }
    Steps::StepGenerateWipeTower::run_step(Orchestrator::instance(), print);
    REQUIRE(active.first->edit_tools.size() == 4);
    CHECK(active.first->edit_tools[0] == std::vector<extruder_index_t>{0, 1});
    CHECK(active.first->edit_tools[1] == std::vector<extruder_index_t>{1, 0});
    CHECK(active.first->edit_tools[2] == std::vector<extruder_index_t>{0});
    CHECK(active.first->edit_tools[3] == std::vector<extruder_index_t>{0});
    REQUIRE(active.first->bands.size() == (late_request ? 1 : 2));
    for (const BandRecord &band : active.first->bands)
        CHECK(band.z == Approx(late_request ? 1.1 : band.private_id == 1 ? 0.2 : 0.5));
    for (size_t l = 2; l < 4; ++l) {
        const Printing::PrintingLayerGroup &layer = print.printing_plan()->groups[0].layers[l];
        REQUIRE(layer.tool_groups.size() == 1);
        CHECK(layer.tool_groups[0].extruder_id == 0);
        const std::vector<const Printing::PrintingExtrusion *> paths = maintenance_paths(layer);
        REQUIRE(paths.size() == (late_request ? 1 : 0));
        if (late_request)
            CHECK(&layer.tool_groups[0].extrusions[0] == paths[0]);
        else {
            REQUIRE(layer.tool_groups[0].extrusions.size() == 1);
            CHECK(layer.tool_groups[0].extrusions[0].sregion_island_role == ExtrusionRole::Perimeter);
        }
    }
}

// Supply a deep band, checking real maintenance cone outlines and suffix depth against the disabled cone case; geometry remains plan-owned.
TEST_CASE("Extended tower maintenance has optional tapered cone rings", "[plugins][wipe-tower]")
{
    const bool cone = GENERATE(false, true);
    WipeActivation active;
    Print print;
    build_plan(print);
    PrintObjectConfig &config = const_cast<PrintObjectConfig &>(print.default_object_config());
    config.wipe_tower_rotation_angle.value = 0.;
    config.wipe_tower_brim_width.value = 0.;
    config.wipe_tower_cone_angle.value = cone ? 60. : 0.;
    active.first->supplied_requests = true;
    active.first->supplied = {{1, {0, 1, 1, 0}, {}, 1, 0, 300.}};
    Steps::StepGenerateWipeTower::run_step(Orchestrator::instance(), print);
    REQUIRE(active.first->bands.size() == 1);
    CHECK(active.first->bands[0].y1 > 40.);
    double prior_ring_area = 0.;
    for (const Printing::PrintingLayerGroup &layer : print.printing_plan()->groups[0].layers) {
        const std::vector<const Printing::PrintingExtrusion *> paths = maintenance_paths(layer);
        REQUIRE(paths.size() == 1);
        const ExtrusionEntity &root = *paths[0]->root;
        CHECK(root.child(0).first_point() == Point::new_scale(40., 60.));
        if (layer.print_z == scale_i(0.2)) {
            if (cone)
                REQUIRE(root.child_count() > 2);
            else
                REQUIRE(root.child_count() == 2);
        }
        bool has_ring = false;
        for (size_t c = 1; c < root.child_count(); ++c) {
            const ExtrusionEntity &path = root.child(c);
            if (path.first_point() == path.last_point()) {
                has_ring = true;
                CHECK(path.length() > root.child(0).length());
            }
        }
        CHECK(has_ring == cone);
        const Polyline rectangle = root.child(0).as_polyline().to_polyline();
        CHECK(rectangle.points[2].y() == scale_i(60. + active.first->bands[0].y1));
        if (cone) {
            REQUIRE(root.child_count() > 1);
            const Polyline ring = root.child(1).as_polyline().to_polyline();
            CHECK(ring.points.size() > 5);
            double min_x = double(scale_i(40.));
            for (const Point &point : ring.points)
                min_x = std::min(min_x, double(point.x()));
            CHECK(min_x < scale_i(40.));
            const double area = Polygon(ring.points).area();
            if (prior_ring_area != 0.)
                CHECK(area < prior_ring_area);
            prior_ring_area = area;
        }
    }
}

// Run the real tool-selection plugin against empty visits, verifying that geometry-free visits remain real selections with before events and no source ownership is fabricated.
TEST_CASE("CreateToolChange emits selections for empty contour visits", "[plugins][wipe-tower]")
{
    WipeActivation active(0);
    Print print;
    build_plan(print);
    for (Printing::PrintingLayerGroup &layer : print.mutable_printing_plan().groups[0].layers)
        for (Printing::PrintingToolGroup &tool : layer.tool_groups)
            tool.extrusions.clear();
    Test::TransitionPipeline::run_layer_plugins(print, {Test::TransitionPipeline::TOOLCHANGE_PLUGIN});
    for (size_t l = 0; l < 2; ++l)
        for (size_t t = 0; t < 2; ++t) {
            const Printing::PrintingToolGroup &tool = print.printing_plan()->groups[0].layers[l].tool_groups[t];
            CHECK(tool.extrusions.empty());
            CHECK(tool.events.has_before() == (l != 0 || t != 0));
        }
}
