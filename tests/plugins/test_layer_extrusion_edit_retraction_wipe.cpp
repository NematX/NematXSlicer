///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
CreateRetractionWipe tests
==========================

These cases exercise the wipe through the real ordered plugin step. They use
separate PrintingExtrusion owners and layer boundaries. Path checks cover
source-owner boundaries and wipes across layer transitions; the crossing cases
compare wipe presence only.

The crossing cases slice cubes, obtain region and instance identities from the
ordered result, then replace the printing plan with controlled printable paths.
Controlled paths enter fresh canonical plan storage before editor execution.
check_wipe_gaps prepares the core AvoidCrossingPerimeters on the target layer
alone and compares its crossing result with explicit expectations and emitted
wipes. Twelve gaps cover interior and outgoing moves, two nozzle radii, a
nonzero tool with distinct perimeter and fill ownership, a shifted second
instance, and the global skirt layer. Several gaps in each plan reuse one
worker's caches. The agreement between plugin and core covers these gaps only;
it does not prove the core rule elsewhere. No case prepares a support target,
so support preparation is not checked here.

    crossing cases: obtain sliced geometry and identities
    |-- Print::process and StepExtrusionOrdering: build masks and ordered owners
    `-- check_wipe_gaps: replace the plan with controlled source/target pairs
        |-- prepare_crossing_test/can_cross_perimeter: compute core decisions
        |-- run_layer_plugins: create scopes, retractions and wipes
        `-- find_role: compare wipe presence with each expected decision
*/

#include "layer_extrusion_edit_transition_test_helpers.hpp"

#include <algorithm>
#include <optional>
#include "libslic3r/ExtrusionEntityCanonical.hpp"
#include "libslic3r/GCode/AvoidCrossingPerimeters.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Steps/StepExtrusionOrdering.hpp"
#include "libslic3r/Steps/PipelineProgress.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/AvoidCrossing/LayerInputCache.hpp"

namespace {

using namespace Slic3r;
using namespace Slic3r::Printing;
using namespace Slic3r::Test::TransitionPipeline;
using slic3r_api::LayerExtrusionEdit::ExtrusionScope::OrderedExtrusionScope;

constexpr const char *WIPE_PLUGIN = "layer_extrusion_edit.wipe.default";
constexpr const char *ENTRY_STATE_PLUGIN =
    "layer_extrusion_edit.entry_state.default";
constexpr const char *TRAVEL_PLUGIN = "layer_extrusion_edit.travel.default";

// One gap in the target layer's XY coordinates and its independently stated expected crossing decision.
struct CrossingGap { Point start; Point end; bool crosses; };

/* Find the first subtree whose effective direct role contains one flag. */
slic3r_api::MutableExtrusionEntity find_role(
    slic3r_api::MutableExtrusionEntity entity,
    raw_extrusion_role role);

// Replace the printing plan and run all gaps in one worker; region must be non-null, expose region(0) with a valid layer, and remain alive through plugin execution.
// Compare source wipes with core crossing decisions on that region's layer alone and with explicit expectations.
void check_wipe_gaps(Print &print, const LayerRegionIsland *region, uint32_t catalog,
                     extruder_index_t extruder, const std::vector<CrossingGap> &gaps);

void check_wipe_gaps(Print &print, const LayerRegionIsland *region, uint32_t catalog,
                     extruder_index_t extruder, const std::vector<CrossingGap> &gaps)
{
    const slic3r_api::LayerRegionIsland region_view(reinterpret_cast<const layer_region_island_handle *>(region));
    const Layer *target_layer = reinterpret_cast<const Layer *>(region_view.region(0).layer().handle());
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.clear();
    plan.groups.emplace_back();
    if (catalog != PRINTING_OBJECT_INSTANCE_ID_NONE)
        plan.groups.back().printing_catalog_ids.push_back(catalog);
    plan.groups.back().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.back().layers.back();
    layer.print_z = target_layer->scaled_print_z();
    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool = layer.tool_groups.back();
    tool.extruder_id = extruder;
    const slic3r_api::Print print_view(reinterpret_cast<const print_handle *>(&print));
    const double nozzle = print_view.config().vector_float_or_default("nozzle_diameter", extruder, 0.4);
    AvoidCrossingPerimeters oracle;
    oracle.init_layer(*target_layer);
    const std::vector<const Layer *> printed{target_layer};
    oracle.prepare_crossing_test({*target_layer, printed, catalog, extruder,
        std::max<coord_t>(1, scale_i(nozzle * 0.5)), []() {}});
    std::vector<bool> expected;
    for (const CrossingGap &gap : gaps) {
        CAPTURE(extruder, nozzle, gap.start.x(), gap.start.y(), gap.end.x(), gap.end.y());
        Polyline direct;
        direct.points = {gap.start, gap.end};
        expected.push_back(oracle.can_cross_perimeter(direct, true));
        CHECK(expected.back() == gap.crosses);
        // Connected source tails ensure crossing, rather than lack of wipe geometry, selects the result.
        for (const Points &points : {Points{gap.start - Point(0, scale_i(5.)), gap.start},
                                    Points{gap.end, gap.end + Point(0, scale_i(5.))}}) {
            PrintingExtrusion extrusion;
            extrusion.region_island = region;
            extrusion.printing_catalog_id = catalog;
            const slic3r_api::ConstPrintingPlan plan_view(reinterpret_cast<const printing_plan_handle *>(&plan));
            const c_point shift = plan_view.object_instance_shift(slic3r_api::ConstPrintingExtrusion(
                reinterpret_cast<const printing_extrusion_handle *>(&extrusion)));
            if (catalog == PRINTING_OBJECT_INSTANCE_ID_NONE) {
                CHECK(shift.x == 0);
                CHECK(shift.y == 0);
            }
            ArcPolyline line;
            for (const Point &point : points)
                line.append(point + Point(shift.x, shift.y));
            ExtrusionPath path(line,
                ExtrusionAttributes(ExtrusionRole::Perimeter, ExtrusionFlow(0.2, 0.4f, 0.2f)), nullptr, true);
            extrusion.root = CanonicalExtrusion::acquire(path);
            tool.extrusions.push_back(std::move(extrusion));
        }
    }
    run_layer_plugins(print, {TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN, WIPE_PLUGIN});
    for (size_t idx = 0; idx < gaps.size(); ++idx) {
        CAPTURE(idx, extruder, nozzle);
        CHECK(find_role(entity_view(*tool.extrusions[2 * idx].root), RAW_EXTRUSION_ROLE_WIPE).valid() == expected[idx]);
    }
}

/* Effective information retained for one geometric leaf in output order. */
struct GeometricLeaf
{
    slic3r_api::ExtrusionEntity entity;
    std::optional<slic3r_api::EPropertyAttributes> attributes;
    std::optional<slic3r_api::EPropertyExtrusionAxis> extrusion_axis;
};

/* Build one external square loop with the seam at its first point. */
std::unique_ptr<ExtrusionEntity> make_external_loop(double origin_x);

/* Build one curved printable leaf with distinct endpoint Z offsets. */
std::unique_ptr<ExtrusionPath> make_arc_path();

/* Build the standard two-path plan with wipe settings enabled. */
void prepare_wipe_print(Print &print,
                        Model &model,
                        const char *wipe_min = "8",
                        const char *wipe_only_crossing = "0",
                        const char *inside_start = "0",
                        const char *inside_end = "0",
                        const char *wipe_return = "0",
                        const char *retract_before_wipe = "20%");

/* Read direct and inherited process properties from geometric leaves. */
void collect_geometric_leaves(
    const slic3r_api::ExtrusionEntity &entity,
    std::optional<slic3r_api::EPropertyAttributes> inherited_attributes,
    std::vector<GeometricLeaf> &leaves);

/* Find an E-only request which reaches one exact semantic target. */
slic3r_api::MutableExtrusionEntity find_axis_target(
    slic3r_api::MutableExtrusionEntity entity,
    c_extrusion_axis_operation operation,
    double value,
    bool toolchange);

std::unique_ptr<ExtrusionEntity> make_external_loop(const double origin_x)
{
    std::unique_ptr<ExtrusionPath> path = std::make_unique<ExtrusionPath>(
        ExtrusionAttributes(
            ExtrusionRole::ExternalPerimeter,
            ExtrusionFlow(0.2, 0.4f, 0.2f)),
        nullptr,
        true);
    path->polyline().append(Point(scale_i(origin_x), 0));
    path->polyline().append(Point(scale_i(origin_x + 10.0), 0));
    path->polyline().append(Point(scale_i(origin_x + 10.0), scale_i(10.0)));
    path->polyline().append(Point(scale_i(origin_x), scale_i(10.0)));
    path->polyline().append(Point(scale_i(origin_x), 0));

    ExtrusionEntity::Children children;
    children.push_back(std::move(path));
    std::unique_ptr<ExtrusionEntity> loop = std::make_unique<ExtrusionEntity>(
        std::move(children), false, false, false);
    loop->get_or_add_property<ExtrusionPropertyLoopRole>()
        .set_perimeter_role(elrDefault);
    return loop;
}

std::unique_ptr<ExtrusionPath> make_arc_path()
{
    ArcPolyline polyline;
    polyline.append(Point(0, 0));
    polyline.append(Geometry::ArcWelder::Segment(
        Point(scale_i(1.0), scale_i(1.0)),
        float(scale_i(1.0)),
        Geometry::ArcWelder::Orientation::CCW));
    polyline.set_z_offset(0, scale_i(0.03));
    polyline.set_z_offset(1, scale_i(0.04));
    return std::make_unique<ExtrusionPath>(
        polyline,
        ExtrusionAttributes(
            ExtrusionRole::Perimeter,
            ExtrusionFlow(0.2, 0.4f, 0.2f)),
        nullptr,
        true);
}

void prepare_wipe_print(
    Print &print,
    Model &model,
    const char *wipe_min,
    const char *wipe_only_crossing,
    const char *inside_start,
    const char *inside_end,
    const char *wipe_return,
    const char *retract_before_wipe)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"retract_before_travel", "0"},
        {"retract_layer_change", "0"},
        {"retract_length", "2"},
        {"retract_length_toolchange", "2"},
        {"retract_restart_extra", "0.1"},
        {"retract_restart_extra_toolchange", "0.1"},
        {"retract_speed", "40"},
        {"deretract_speed", "35"},
        {"wipe", "1"},
        {"wipe_speed", "20"},
        {"wipe_min", wipe_min},
        {"retract_before_wipe", retract_before_wipe},
        {"wipe_return", wipe_return},
        {"wipe_only_crossing", wipe_only_crossing},
        {"wipe_inside_start", inside_start},
        {"wipe_inside_end", inside_end},
        {"wipe_inside_depth", "50%"},
        {"wipe_extra_perimeter", "0"},
        {"use_firmware_retraction", "0"}
    });
    Slic3r::Test::init_print(
        {Slic3r::Test::TestMesh::cube_20x20x20}, print, model, config);

    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.clear();
    plan.groups.emplace_back();
    plan.groups.back().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.back().layers.back();
    layer.print_z = scale_i(0.2);
    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool = layer.tool_groups.back();
    tool.extruder_id = 0;
    append_printing_extrusion(tool, make_print_path(0.0, 10.0));
    append_printing_extrusion(tool, make_print_path(20.0, 30.0));
}

void collect_geometric_leaves(
    const slic3r_api::ExtrusionEntity &entity,
    std::optional<slic3r_api::EPropertyAttributes> inherited_attributes,
    std::vector<GeometricLeaf> &leaves)
{
    if (const slic3r_api::EPropertyAttributes *direct =
            entity.get(slic3r_api::EPropertyAttributes::key))
        inherited_attributes = *direct;
    if (entity.segment_count() > 0) {
        std::optional<slic3r_api::EPropertyExtrusionAxis> axis;
        if (const slic3r_api::EPropertyExtrusionAxis *direct =
                entity.get(slic3r_api::EPropertyExtrusionAxis::key))
            axis = *direct;
        leaves.push_back(GeometricLeaf{entity, inherited_attributes, axis});
        return;
    }
    for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx)
        collect_geometric_leaves(
            entity.child(child_idx), inherited_attributes, leaves);
}

slic3r_api::MutableExtrusionEntity find_role(
    slic3r_api::MutableExtrusionEntity entity,
    const raw_extrusion_role role)
{
    if (const slic3r_api::EPropertyAttributes *attributes =
            entity.get(slic3r_api::EPropertyAttributes::key))
        if (RAW_EXTRUSION_ROLE_HAS(attributes->extrusion_role(), role))
            return entity;
    for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx) {
        const slic3r_api::MutableExtrusionEntity found = find_role(
            entity.child_mutable(child_idx), role);
        if (found.valid())
            return found;
    }
    return slic3r_api::MutableExtrusionEntity();
}

slic3r_api::MutableExtrusionEntity find_axis_target(
    slic3r_api::MutableExtrusionEntity entity,
    const c_extrusion_axis_operation operation,
    const double value,
    const bool toolchange)
{
    const slic3r_api::EPropertyExtrusionAxis *axis =
        entity.get(slic3r_api::EPropertyExtrusionAxis::key);
    if (axis != nullptr && entity.segment_count() == 0 &&
        axis->operation == operation && axis->value == Approx(value) &&
        (axis->toolchange != 0) == toolchange)
        return entity;
    for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx) {
        const slic3r_api::MutableExtrusionEntity found = find_axis_target(
            entity.child_mutable(child_idx), operation, value, toolchange);
        if (found.valid())
            return found;
    }
    return slic3r_api::MutableExtrusionEntity();
}

TEST_CASE("CreateRetractionWipe is registered after retraction",
          "[plugins][layer-extrusion-edit][wipe][registration]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Plugin *plugin = Orchestrator::instance().get_plugin(WIPE_PLUGIN);
    REQUIRE(plugin != nullptr);
    CHECK(plugin->get_priority() == -80);
    CHECK(plugin->get_exclusive_group() == "layer_extrusion_edit.wipe");
    REQUIRE(plugin->get_dependencies().size() == 1);
    CHECK(plugin->get_dependencies().front() == TRANSITION_SCOPE_PLUGIN);
}

TEST_CASE("Retraction wipe retraces connected source content across a layer boundary",
          "[plugins][layer-extrusion-edit][retraction][wipe]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_wipe_print(print, model);

    PrintingPlan &plan = print.mutable_printing_plan();
    PrintingToolGroup &first_tool =
        plan.groups.front().layers.front().tool_groups.front();
    first_tool.extrusions.clear();
    ExtrusionEntity::Children connected;
    connected.push_back(make_print_path(0.0, 5.0));
    connected.push_back(make_print_path(5.0, 10.0));
    append_printing_extrusion(
        first_tool,
        std::make_unique<ExtrusionEntity>(
            std::move(connected), false, false, false));

    plan.groups.front().layers.emplace_back();
    PrintingLayerGroup &second_layer = plan.groups.front().layers.back();
    second_layer.print_z = scale_i(0.4);
    second_layer.tool_groups.emplace_back();
    second_layer.tool_groups.back().extruder_id = 0;
    append_printing_extrusion(
        second_layer.tool_groups.back(), make_print_path(20.0, 30.0));

    run_layer_plugins(print, {
        TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN, WIPE_PLUGIN});

    const slic3r_api::MutableExtrusionEntity source = entity_view(
        *plan.groups.front().layers.front().tool_groups.front()
            .extrusions.front().root);
    const slic3r_api::MutableExtrusionEntity wipe =
        find_role(source, RAW_EXTRUSION_ROLE_WIPE);
    REQUIRE(wipe.valid());
    std::vector<GeometricLeaf> leaves;
    collect_geometric_leaves(wipe, std::nullopt, leaves);
    REQUIRE(leaves.size() >= 2);
    CHECK(leaves.front().entity.segment(0).point_a.x == scale_i(10.0));
    CHECK(leaves.back().entity.segment(
        leaves.back().entity.segment_count() - 1).point_b.x == scale_i(2.0));
    REQUIRE(leaves.front().extrusion_axis.has_value());
    CHECK(leaves.front().extrusion_axis->operation ==
          C_EXTRUSION_AXIS_OPERATION_RETRACT_TO);
    CHECK(leaves.front().extrusion_axis->value == Approx(2.0));
    CHECK_FALSE(leaves.back().extrusion_axis.has_value());
}

TEST_CASE("Wipe-only-crossing matches the core decision on sliced masks",
          "[plugins][layer-extrusion-edit][retraction][wipe][crossing][avoid-crossing-parity]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_wipe_print(print, model, "2", "1");
    print.set_status_silent();
    print.process();
    print.pipeline_progress().reset();
    Steps::StepExtrusionOrdering::clear(print);
    Steps::StepExtrusionOrdering::prepare(print);
    Steps::StepExtrusionOrdering::run_step(Orchestrator::instance(), print);
    PrintingPlan &plan = print.mutable_printing_plan();
    const LayerRegionIsland *region = nullptr;
    uint32_t catalog = PRINTING_OBJECT_INSTANCE_ID_NONE;
    const Layer *target_layer = nullptr;
    // Layer three is above the bottom solid layers and below the top surfaces of the cube.
    constexpr size_t interior_layer_id = 3;
    for (const PrintingGroup &group : plan.groups)
        for (const PrintingLayerGroup &layer : group.layers)
            for (const PrintingToolGroup &tool : layer.tool_groups)
                for (const PrintingExtrusion &extrusion : tool.extrusions) {
                    if (region != nullptr || extrusion.region_island == nullptr ||
                        extrusion.printing_catalog_id == PRINTING_OBJECT_INSTANCE_ID_NONE)
                        continue;
                    const slic3r_api::LayerRegionIsland view(
                        reinterpret_cast<const layer_region_island_handle *>(extrusion.region_island));
                    if (view.region_count() == 0 ||
                        reinterpret_cast<const Layer *>(view.region(0).layer().handle())->id() != interior_layer_id)
                        continue;
                    region = extrusion.region_island;
                    catalog = extrusion.printing_catalog_id;
                    target_layer = reinterpret_cast<const Layer *>(view.region(0).layer().handle());
                }
    REQUIRE(target_layer != nullptr);
    const BoundingBox bounds = get_extents(target_layer->lslices());
    const Point center = bounds.center();
    check_wipe_gaps(print, region, catalog, 0, {
        {center, center + Point(scale_i(2.), 0), false},
        {center, Point(bounds.max.x() + scale_i(2.), center.y()), true},
        {center, center + Point(0, scale_i(2.)), false},
        {center, Point(center.x(), bounds.min.y() - scale_i(2.)), true}});
}

TEST_CASE("Wipe crossing distinguishes inset radii tools instances and global skirt layers",
          "[plugins][layer-extrusion-edit][retraction][wipe][crossing][avoid-crossing-parity][avoid-crossing-input]")
{
    enum class Scene { SmallNozzle, LargeNozzle, SecondTool, Skirt };
    const Scene scene = GENERATE(Scene::SmallNozzle, Scene::LargeNozzle, Scene::SecondTool, Scene::Skirt);
    CAPTURE(static_cast<int>(scene));
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"layer_height", "0.2"}, {"first_layer_height", "0.2"},
        {"perimeters", "4"}, {"top_solid_layers", "1"}, {"bottom_solid_layers", "1"},
        {"skirts", scene == Scene::Skirt ? "1" : "0"}, {"skirt_height", "1"}, {"brim_width", "0"},
        {"nozzle_diameter", scene == Scene::LargeNozzle ? "0.8,0.8" : "0.4,0.4"},
        {"min_layer_height", "0.05,0.05"}, {"max_layer_height", "0.3,0.3"},
        {"perimeter_extruder", scene == Scene::SecondTool ? "2" : "1"},
        {"infill_extruder", "1"}, {"solid_infill_extruder", "1"},
        {"retract_before_travel", "0,0"}, {"retract_layer_change", "0,0"},
        {"retract_length", "2,2"}, {"retract_speed", "40,40"},
        {"wipe", "1,1"}, {"wipe_speed", "20,20"}, {"wipe_min", "2,2"},
        {"wipe_only_crossing", "1,1"}, {"wipe_inside_start", "0,0"}, {"wipe_inside_end", "0,0"},
        {"wipe_return", "0,0"}, {"retract_before_wipe", "20%,20%"}, {"use_firmware_retraction", "0"}});
    Model model;
    ModelObject *object = model.add_object();
    object->add_volume(make_cube(20., 20., 1.2), ModelVolumeType::MODEL_PART, false);
    object->add_instance();
    object->add_instance()->set_offset(Vec3d(40., 10., 0.));
    model.center_instances_around_point({100., 100.});
    Print print;
    print.auto_assign_extruders(object);
    print.apply(model, config);
    print.set_status_silent();
    print.process();
    print.pipeline_progress().reset();
    Steps::StepExtrusionOrdering::clear(print);
    Steps::StepExtrusionOrdering::prepare(print);
    Steps::StepExtrusionOrdering::run_step(Orchestrator::instance(), print);
    PrintingPlan &plan = print.mutable_printing_plan();
    const slic3r_api::ConstPrintingPlan plan_view(reinterpret_cast<const printing_plan_handle *>(&plan));
    const LayerRegionIsland *region = nullptr;
    uint32_t catalog = PRINTING_OBJECT_INSTANCE_ID_NONE;
    const Layer *target_layer = nullptr;
    // Interior layer three avoids top/bottom masks; the skirt instead lives on its global auxiliary layer.
    constexpr size_t interior_layer_id = 3;
    for (const PrintingGroup &group : plan.groups)
        for (const PrintingLayerGroup &layer : group.layers)
            for (const PrintingToolGroup &tool : layer.tool_groups)
                for (const PrintingExtrusion &extrusion : tool.extrusions) {
                    if (region != nullptr || extrusion.region_island == nullptr)
                        continue;
                    const bool global = extrusion.printing_catalog_id == PRINTING_OBJECT_INSTANCE_ID_NONE;
                    if (global != (scene == Scene::Skirt))
                        continue;
                    if (!global && plan_view.object_instance(extrusion.printing_catalog_id).object_instance_idx() != 1)
                        continue;
                    const slic3r_api::LayerRegionIsland view(reinterpret_cast<const layer_region_island_handle *>(extrusion.region_island));
                    if (view.region_count() == 0)
                        continue;
                    const Layer *candidate = reinterpret_cast<const Layer *>(view.region(0).layer().handle());
                    if (!global && candidate->id() != interior_layer_id)
                        continue;
                    region = extrusion.region_island;
                    catalog = extrusion.printing_catalog_id;
                    target_layer = candidate;
                }
    REQUIRE(target_layer != nullptr);
    const BoundingBox bounds = get_extents(target_layer->lslices());
    const Point center = bounds.center();
    if (scene == Scene::Skirt) {
        namespace AC = slic3r_api::LayerExtrusionEdit::AvoidCrossing;
        const slic3r_api::Print print_view(reinterpret_cast<const print_handle *>(&print));
        const slic3r_api::Object auxiliary = print_view.auxiliary_object();
        CHECK(auxiliary.layer_count() == 0);
        REQUIRE(auxiliary.auxiliary_layer_count() > 0);
        CHECK_FALSE(AC::is_first_layer(auxiliary, slic3r_api::Layer(reinterpret_cast<const layer_handle *>(target_layer))));
        bool copied = false;
        for (uint32_t idx = 0; idx < auxiliary.auxiliary_layer_count(); ++idx) {
            if (auxiliary.auxiliary_layer(idx).handle() != reinterpret_cast<const layer_handle *>(target_layer))
                continue;
            PluginStorage storage;
            const AC::Input input = AC::build_input(reinterpret_cast<storage_handle *>(&storage),
                print_view, {0, idx, true, true});
            CHECK(input.target->identity.global_object);
            CHECK(input.target->identity.object == 0);
            CHECK_FALSE(input.target->first_layer);
            CHECK(input.target->print_z == target_layer->scaled_print_z());
            CHECK_FALSE(input.target->lslices.empty());
            CHECK_FALSE(input.target->regions.empty());
            plugin_run_context ctx{};
            ctx.plugin_storage = reinterpret_cast<storage_handle *>(&storage);
            AC::LayerInputCache cache;
            const AC::Input &cached = cache.get(print_view, auxiliary.auxiliary_layer(idx), &ctx);
            CHECK(cached.target->identity.global_object);
            CHECK(cached.target->identity.object == input.target->identity.object);
            CHECK(cached.target->identity.layer == input.target->identity.layer);
            CHECK(cached.target->identity.auxiliary == input.target->identity.auxiliary);
            CHECK(cached.target->print_z == input.target->print_z);
            CHECK(&cache.get(print_view, auxiliary.auxiliary_layer(idx), &ctx) == &cached);
            copied = true;
        }
        REQUIRE(copied);
        // Both gaps on this thin skirt domain are noncrossing in the prepared core oracle, including the outgoing chord.
        // NONE keeps both controlled paths in platter space and must still resolve the global layer.
        check_wipe_gaps(print, region, catalog, 0, {
            {center, Point(bounds.max.x() + scale_i(2.), center.y()), false},
            {center, center + Point(scale_i(1.), 0), false}});
    } else if (scene == Scene::SecondTool) {
        // Tool one owns the perimeter band; tool zero owns the fill and cannot admit this near-edge endpoint.
        AvoidCrossingPerimeters other_tool;
        other_tool.init_layer(*target_layer);
        const std::vector<const Layer *> printed{target_layer};
        other_tool.prepare_crossing_test({*target_layer, printed, catalog, 0,
            std::max<coord_t>(1, scale_i(slic3r_api::Print(reinterpret_cast<const print_handle *>(&print))
                .config().vector_float_or_default("nozzle_diameter", 0, 0.4) * 0.5)), []() {}});
        Polyline near_edge;
        near_edge.points = {Point(bounds.max.x() - scale_i(0.1), center.y()),
                            Point(bounds.max.x() - scale_i(0.5), center.y())};
        CHECK_FALSE(other_tool.can_cross_perimeter(near_edge, true));
        check_wipe_gaps(print, region, catalog, 1, {
            {Point(bounds.max.x() - scale_i(0.1), center.y()),
             Point(bounds.max.x() - scale_i(0.5), center.y()), true},
            {center, center + Point(scale_i(1.), 0), false}});
    } else {
        // At 0.45 mm from the edge, the 0.3 mm inset admits the endpoint but the 0.6 mm inset excludes it.
        const Point band(bounds.max.x() - scale_i(0.45), center.y());
        AvoidCrossingPerimeters unshifted;
        unshifted.init_layer(*target_layer);
        const std::vector<const Layer *> printed{target_layer};
        unshifted.prepare_crossing_test({*target_layer, printed, catalog, 0,
            std::max<coord_t>(1, scale_i(slic3r_api::Print(reinterpret_cast<const print_handle *>(&print))
                .config().vector_float_or_default("nozzle_diameter", 0, 0.4) * 0.5)), []() {}});
        Polyline direct;
        direct.points = {center, band};
        CHECK_FALSE(unshifted.can_cross_perimeter(direct, false));
        check_wipe_gaps(print, region, catalog, 0, {
            {center, band, scene == Scene::LargeNozzle},
            {center, center + Point(scale_i(1.), 0), false}});
    }
}

TEST_CASE("Wipe-only-crossing preserves the E-only retraction without a local target",
          "[plugins][layer-extrusion-edit][retraction][wipe][crossing]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_wipe_print(print, model, "2", "1");

    PrintingPlan &plan = print.mutable_printing_plan();
    PrintingExtrusion target = std::move(
        plan.groups.front().layers.front().tool_groups.front().extrusions.back());
    plan.groups.front().layers.front().tool_groups.front().extrusions.pop_back();
    plan.groups.front().layers.emplace_back();
    PrintingLayerGroup &second = plan.groups.front().layers.back();
    second.print_z = scale_i(0.4);
    second.tool_groups.emplace_back();
    second.tool_groups.back().extruder_id = 0;
    second.tool_groups.back().extrusions.push_back(std::move(target));

    run_layer_plugins(print, {
        TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN, WIPE_PLUGIN});

    const slic3r_api::MutableExtrusionEntity source = entity_view(
        *plan.groups.front().layers.front().tool_groups.front()
            .extrusions.front().root);
    CHECK(find_role(source, RAW_EXTRUSION_ROLE_RETRACT).valid());
    CHECK_FALSE(find_role(source, RAW_EXTRUSION_ROLE_WIPE).valid());
}

TEST_CASE("Inside-start unretracts from the travel target to an external seam",
          "[plugins][layer-extrusion-edit][retraction][wipe][travel][inside-start]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_wipe_print(print, model, "2", "0", "1", "0");
    PrintingToolGroup &tool = print.mutable_printing_plan().groups.front()
        .layers.front().tool_groups.front();
    tool.extrusions.clear();
    append_printing_extrusion(tool, make_external_loop(0.0));
    append_printing_extrusion(tool, make_external_loop(20.0));

    run_layer_plugins(print, {
        TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN, WIPE_PLUGIN,
        ENTRY_STATE_PLUGIN, TRAVEL_PLUGIN});

    OrderedExtrusionScope target(
        find_scope(entity_view(*tool.extrusions.back().root)),
        scope_property_key());
    const slic3r_api::MutableExtrusionEntity approach =
        find_role(target.before(), RAW_EXTRUSION_ROLE_WIPE);
    REQUIRE(approach.valid());
    REQUIRE(approach.segment_count() == 1);
    CHECK(approach.segment(0).point_b.x == scale_i(20.0));
    CHECK(approach.segment(0).point_b.y == 0);
    const slic3r_api::EPropertyExtrusionAxis *axis =
        approach.get(slic3r_api::EPropertyExtrusionAxis::key);
    REQUIRE(axis != nullptr);
    CHECK(axis->operation == C_EXTRUSION_AXIS_OPERATION_UNRETRACT);

    const slic3r_api::MutableExtrusionEntity travel = target.travel();
    REQUIRE(travel.segment_count() > 0);
    CHECK(travel.segment(travel.segment_count() - 1).point_b.x ==
          approach.segment(0).point_a.x);
    CHECK(travel.segment(travel.segment_count() - 1).point_b.y ==
          approach.segment(0).point_a.y);
}

TEST_CASE("Inside-end leaves a non-returning loop wipe on the material side",
          "[plugins][layer-extrusion-edit][retraction][wipe][inside-end]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_wipe_print(print, model, "2", "0", "0", "1");
    PrintingToolGroup &tool = print.mutable_printing_plan().groups.front()
        .layers.front().tool_groups.front();
    tool.extrusions.clear();
    append_printing_extrusion(tool, make_external_loop(0.0));
    append_printing_extrusion(tool, make_print_path(20.0, 30.0));

    run_layer_plugins(print, {
        TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN, WIPE_PLUGIN});

    OrderedExtrusionScope source(
        find_scope(entity_view(*tool.extrusions.front().root)),
        scope_property_key());
    const slic3r_api::MutableExtrusionEntity wipe =
        find_role(source.after(), RAW_EXTRUSION_ROLE_WIPE);
    REQUIRE(wipe.valid());
    std::vector<GeometricLeaf> leaves;
    collect_geometric_leaves(wipe, std::nullopt, leaves);
    REQUIRE(!leaves.empty());
    const slic3r_api::ExtrusionEntity final_leaf = leaves.back().entity;
    const c_extrusion_segment final_segment =
        final_leaf.segment(final_leaf.segment_count() - 1);
    CHECK(final_segment.point_b.y > final_segment.point_a.y);
}

TEST_CASE("Retraction wipe stops at its PrintingExtrusion boundary",
          "[plugins][layer-extrusion-edit][retraction][wipe][owner-boundary]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_wipe_print(print, model);

    PrintingToolGroup &tool = print.mutable_printing_plan().groups.front()
        .layers.front().tool_groups.front();
    tool.extrusions.clear();
    append_printing_extrusion(tool, make_print_path(0.0, 9.5));
    append_printing_extrusion(tool, make_print_path(9.5, 10.0));
    append_printing_extrusion(tool, make_print_path(20.0, 30.0));

    run_layer_plugins(print, {
        TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN, WIPE_PLUGIN});

    OrderedExtrusionScope source(
        find_scope(entity_view(*tool.extrusions[1].root)),
        scope_property_key());
    const slic3r_api::MutableExtrusionEntity wipe =
        find_role(source.after(), RAW_EXTRUSION_ROLE_WIPE);
    REQUIRE(wipe.valid());
    std::vector<GeometricLeaf> leaves;
    collect_geometric_leaves(wipe, std::nullopt, leaves);
    REQUIRE(!leaves.empty());
    const GeometricLeaf &last = leaves.back();
    CHECK(last.entity.segment(last.entity.segment_count() - 1).point_b.x ==
          scale_i(9.5));

    // Five millimetres of source geometry cannot finish the requested wipe,
    // so the original semantic target is completed by a final E-only event.
    CHECK(find_axis_target(
        source.after(), C_EXTRUSION_AXIS_OPERATION_RETRACT_TO, 2.0, false)
        .valid());
}

TEST_CASE("Wipe return retraces half the configured distance back to the seam",
          "[plugins][layer-extrusion-edit][retraction][wipe][return]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_wipe_print(print, model, "8", "0", "0", "0", "1");

    run_layer_plugins(print, {
        TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN, WIPE_PLUGIN});

    PrintingToolGroup &tool = print.mutable_printing_plan().groups.front()
        .layers.front().tool_groups.front();
    OrderedExtrusionScope source(
        find_scope(entity_view(*tool.extrusions.front().root)),
        scope_property_key());
    const slic3r_api::MutableExtrusionEntity wipe =
        find_role(source.after(), RAW_EXTRUSION_ROLE_WIPE);
    REQUIRE(wipe.valid());
    std::vector<GeometricLeaf> leaves;
    collect_geometric_leaves(wipe, std::nullopt, leaves);
    REQUIRE(!leaves.empty());
    const GeometricLeaf &first = leaves.front();
    const GeometricLeaf &last = leaves.back();
    CHECK(first.entity.segment(0).point_a.x == scale_i(10.0));
    CHECK(last.entity.segment(last.entity.segment_count() - 1).point_b.x ==
          scale_i(10.0));
    distf_t total_length = 0.0;
    for (const GeometricLeaf &leaf : leaves)
        total_length += leaf.entity.local_length();
    CHECK(unscaled(total_length) == Approx(8.0));
}

TEST_CASE("Retraction wipe preserves the semantic tool-change request",
          "[plugins][layer-extrusion-edit][retraction][wipe][toolchange]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_wipe_print(print, model, "2");

    PrintingLayerGroup &layer = print.mutable_printing_plan().groups.front()
        .layers.front();
    PrintingExtrusion target = std::move(
        layer.tool_groups.front().extrusions.back());
    layer.tool_groups.front().extrusions.pop_back();
    layer.tool_groups.emplace_back();
    layer.tool_groups.back().extruder_id = 1;
    layer.tool_groups.back().extrusions.push_back(std::move(target));

    run_layer_plugins(print, {
        TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN, WIPE_PLUGIN});

    OrderedExtrusionScope source(
        find_scope(entity_view(
            *layer.tool_groups.front().extrusions.front().root)),
        scope_property_key());
    const slic3r_api::MutableExtrusionEntity wipe =
        find_role(source.after(), RAW_EXTRUSION_ROLE_WIPE);
    REQUIRE(wipe.valid());
    std::vector<GeometricLeaf> leaves;
    collect_geometric_leaves(wipe, std::nullopt, leaves);
    const std::vector<GeometricLeaf>::const_iterator retracting = std::find_if(
        leaves.begin(), leaves.end(), [](const GeometricLeaf &leaf) {
            return leaf.extrusion_axis.has_value();
        });
    REQUIRE(retracting != leaves.end());
    CHECK(retracting->extrusion_axis->operation ==
          C_EXTRUSION_AXIS_OPERATION_RETRACT_TO);
    CHECK(retracting->extrusion_axis->toolchange == 1);
}

TEST_CASE("Retraction wipe reverses arcs and interpolates their Z offsets",
          "[plugins][layer-extrusion-edit][retraction][wipe][arc][z-offset]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_wipe_print(print, model, "10", "0", "0", "0", "0", "100%");

    PrintingToolGroup &tool = print.mutable_printing_plan().groups.front()
        .layers.front().tool_groups.front();
    tool.extrusions.clear();
    append_printing_extrusion(tool, make_arc_path());
    append_printing_extrusion(tool, make_print_path(20.0, 30.0));

    run_layer_plugins(print, {
        TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN, WIPE_PLUGIN});

    OrderedExtrusionScope source(
        find_scope(entity_view(*tool.extrusions.front().root)),
        scope_property_key());
    const slic3r_api::MutableExtrusionEntity wipe =
        find_role(source.after(), RAW_EXTRUSION_ROLE_WIPE);
    REQUIRE(wipe.valid());
    std::vector<GeometricLeaf> leaves;
    collect_geometric_leaves(wipe, std::nullopt, leaves);
    REQUIRE(!leaves.empty());
    const c_extrusion_segment reversed = leaves.front().entity.segment(0);
    CHECK(reversed.point_a.x == scale_i(1.0));
    CHECK(reversed.point_a.y == scale_i(1.0));
    CHECK((reversed.point_b.x != reversed.point_a.x ||
           reversed.point_b.y != reversed.point_a.y));
    CHECK(reversed.radius != 0.f);
    CHECK(reversed.orientation == RAW_EXTRUSION_ARC_ORIENTATION_CW);
    CHECK(reversed.z_offset_a == scale_i(0.04));
    CHECK(reversed.z_offset_b > scale_i(0.03));
    CHECK(reversed.z_offset_b < scale_i(0.04));
}

} // namespace
