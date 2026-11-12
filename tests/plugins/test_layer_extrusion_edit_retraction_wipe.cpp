///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
CreateRetractionWipe tests
==========================

These cases exercise the wipe through the real ordered plugin step. They use
separate PrintingExtrusion owners and layer boundaries. Path checks cover
source-owner boundaries, complete fragmented loops, and wipes across layer transitions.
The crossing cases run both wipe passes around entry-state and travel generation.
They check returning geometry, exact joins and stationary or residual retraction
targets for open paths and loops, including an empty intermediate layer group.
Percent cases check stationary and moving retraction targets and nozzle-relative
inside depths through the same ordered step.

The crossing cases slice cubes, obtain region and instance identities from the
ordered result, then replace the printing plan with controlled printable paths.
Controlled paths enter fresh canonical plan storage before editor execution.
check_wipe_gaps prepares the core AvoidCrossingPerimeters on the target layer
alone and compares its crossing result with explicit expectations and emitted
wipes. Twelve gaps cover interior and outgoing moves, two nozzle radii, a
nonzero tool with distinct perimeter and fill ownership, a shifted second
instance, and the global skirt layer. Several gaps in each plan reuse one
worker's caches. The agreement between plugin and core covers these gaps only;
it does not prove the core rule elsewhere. A sliced roof on a narrow post also
supplies a real support layer with its support property and region identity.
Support crossing witnesses use the last object layer below support Z plus epsilon,
not the support subject. Support-to-support paths over the part wipe, while paths
across the unsupported overhang do not. Other-instance witnesses wipe in a layer
plan but not with complete_objects or positive parallel_objects_step.
Controlled deposits, occupied departures and non-crossing detours suppress the
second wipe; tool-change, lift and missing-dependency cases check pass ordering.
Split cases cut the generated travel into two ordered leaves: a gap crossed only
by the whole path still wipes, while a split detour or a broken junction does not.
Certain local crossings use sliced instance and island identities, support in
both directions, per-object brim published as Perimeter, and global skirt.
Both passes run together: departure lengths and leaf counts exclude a second
wipe, while the travel starts at the normal non-returning wipe's endpoint.
Pairs of support or brim paths on different instances remain for the returning
pass. A global path carrying an object region paired with support distinguishes
global ownership from that region; a regionless departure checks absent ownership.
Seam-gap cases retain the LOOP marker on an open path and require an explicit
non-extruding wipe from its printed endpoint to the seam before each forward pass.
Closed paths and return-to-seam settings share the same continuity and phase checks.

    crossing cases: obtain sliced geometry and identities
    |-- Print::process and StepExtrusionOrdering: build masks and ordered owners
    `-- check_wipe_gaps: replace the plan with controlled source/target pairs
        |-- prepare_crossing_test/can_cross_perimeter: compute core decisions
        |-- run_layer_plugins: create scopes, retractions and wipes
        `-- find_role: compare wipe presence with each expected decision
*/

#include "layer_extrusion_edit_transition_test_helpers.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <optional>
#include <utility>
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
constexpr const char *CROSSING_WIPE_PLUGIN = "layer_extrusion_edit.wipe.crossing";
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
    run_layer_plugins(print, {TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN, WIPE_PLUGIN,
        ENTRY_STATE_PLUGIN, TRAVEL_PLUGIN, CROSSING_WIPE_PLUGIN});
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

// Build a canonical square from four ordered sides; optionally inherit its loop property through a wrapper or mark the terminal or numbered side as Bridge.
std::unique_ptr<ExtrusionEntity> make_fragmented_loop(bool wrapped, bool terminal_bridge = false, double origin_x = 0., size_t bridge_side = 0);

/* Build one curved printable leaf with distinct endpoint Z offsets. */
std::unique_ptr<ExtrusionPath> make_arc_path();

/* Build the standard two-path plan with wipe settings enabled and the requested extra perimeter distance. */
void prepare_wipe_print(Print &print,
                        Model &model,
                        const char *wipe_min = "8",
                        const char *wipe_only_crossing = "0",
                        const char *inside_start = "0",
                        const char *inside_end = "0",
                        const char *wipe_return = "0",
                        const char *retract_before_wipe = "20%",
                        const char *wipe_extra_perimeter = "0");

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

std::unique_ptr<ExtrusionEntity> make_fragmented_loop(const bool wrapped, const bool terminal_bridge, const double origin_x, const size_t bridge_side)
{
    const Points corners{Point(0, 0), Point(scale_i(10.), 0),
        Point(scale_i(10.), scale_i(10.)), Point(0, scale_i(10.)), Point(0, 0)};
    ExtrusionEntity::Children fragments;
    for (size_t idx = 1; idx < corners.size(); ++idx) {
        ArcPolyline line;
        line.append(corners[idx - 1] + Point(scale_i(origin_x), 0));
        line.append(corners[idx] + Point(scale_i(origin_x), 0));
        fragments.push_back(std::make_unique<ExtrusionPath>(line,
            ExtrusionAttributes((terminal_bridge && idx + 1 == corners.size()) || idx == bridge_side ?
                ExtrusionRole::OverhangPerimeter : ExtrusionRole::ExternalPerimeter,
                ExtrusionFlow(0.2, 0.4f, 0.2f)), nullptr, true));
    }
    std::unique_ptr<ExtrusionEntity> loop = std::make_unique<ExtrusionEntity>(
        std::move(fragments), false, false, false);
    if (wrapped) {
        ExtrusionEntity::Children children;
        children.push_back(std::move(loop));
        loop = std::make_unique<ExtrusionEntity>(std::move(children), false, false, false);
    }
    loop->get_or_add_property<ExtrusionPropertyLoopRole>().set_perimeter_role(elrDefault);
    return CanonicalExtrusion::acquire(*loop);
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
    const char *retract_before_wipe,
    const char *wipe_extra_perimeter)
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
        {"wipe_extra_perimeter", wipe_extra_perimeter},
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
    Plugin *crossing = Orchestrator::instance().get_plugin(CROSSING_WIPE_PLUGIN);
    REQUIRE(crossing != nullptr);
    CHECK(crossing->get_priority() == -45);
    CHECK(crossing->get_exclusive_group().empty());
    const std::vector<std::string> dependencies = crossing->get_dependencies();
    REQUIRE(dependencies.size() == 2);
    CHECK(std::find(dependencies.begin(), dependencies.end(), WIPE_PLUGIN) != dependencies.end());
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

// Exercise both wipe passes on controlled plans with sliced identities and masks, checking normal departures for certain crossings and returning departures otherwise.
TEST_CASE("Crossing wipes preserve generated travel starts and absolute retraction targets",
          "[plugins][layer-extrusion-edit][wipe][crossing]")
{
    enum class Shape { Open, LongLoop, EqualLoop, ShortLoop, Fragmented, Residual,
        InterLayer, Deposit, OccupiedAfter, Detour, Blocked, Lift, Toolchange, Support,
        RoutedDetour, LimitedDetour, Instances, ZOffsets, WrongZ, ExtraLoop,
        Islands, SupportToObject, ObjectToSupport, BrimToObject, SkirtToObject,
        OutsideSupports, OutsideBrims, GlobalToSupport, RegionlessToObject,
        SplitTravel, SplitDetour, BrokenSplit, SupportVoid,
        OtherSupport, OtherSupportComplete, OtherSupportParallel };
    const Shape shape = GENERATE(Shape::Open, Shape::LongLoop, Shape::EqualLoop, Shape::ShortLoop,
        Shape::Fragmented, Shape::Residual, Shape::InterLayer, Shape::Deposit, Shape::OccupiedAfter,
        Shape::Detour, Shape::Blocked, Shape::Lift, Shape::Toolchange, Shape::Support,
        Shape::RoutedDetour, Shape::LimitedDetour, Shape::Instances, Shape::ZOffsets, Shape::WrongZ, Shape::ExtraLoop,
        Shape::Islands, Shape::SupportToObject, Shape::ObjectToSupport, Shape::BrimToObject, Shape::SkirtToObject,
        Shape::OutsideSupports, Shape::OutsideBrims, Shape::GlobalToSupport, Shape::RegionlessToObject,
        Shape::SplitTravel, Shape::SplitDetour, Shape::BrokenSplit, Shape::SupportVoid,
        Shape::OtherSupport, Shape::OtherSupportComplete, Shape::OtherSupportParallel);
    // Split shapes cut the generated travel in two ordered leaves, as the deceleration provider does.
    const bool split = shape == Shape::SplitTravel || shape == Shape::SplitDetour || shape == Shape::BrokenSplit;
    const bool detour = shape == Shape::Detour || shape == Shape::SplitDetour;
    const bool other_support = shape == Shape::OtherSupport || shape == Shape::OtherSupportComplete || shape == Shape::OtherSupportParallel;
    const bool outside_supports = shape == Shape::OutsideSupports || other_support;
    const bool outside_pair = outside_supports || shape == Shape::OutsideBrims || shape == Shape::GlobalToSupport;
    const bool certain = shape == Shape::Instances || shape == Shape::Islands ||
        shape == Shape::SupportToObject || shape == Shape::ObjectToSupport ||
        shape == Shape::BrimToObject || shape == Shape::SkirtToObject || shape == Shape::RegionlessToObject;
    const char *before = GENERATE("0%", "20%", "100%");
    CAPTURE(static_cast<int>(shape), before);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    const double wanted = shape == Shape::EqualLoop || shape == Shape::ExtraLoop ? 40. : shape == Shape::ShortLoop ? 45. :
        shape == Shape::Residual ? 0.1 : 8.;
    config.set_deserialize_strict({{"layer_height", "0.2"}, {"first_layer_height", "0.2"},
        {"perimeters", "2"}, {"top_solid_layers", "0"}, {"bottom_solid_layers", "0"},
        {"skirts", "0"}, {"brim_width", "0"}, {"retract_before_travel", "0,0"},
        {"retract_layer_change", "0,0"}, {"retract_length", "2,3"}, {"retract_length_toolchange", "2,3"},
        {"retract_speed", "40,50"}, {"wipe", "1,0"}, {"wipe_only_crossing", "1,0"},
        {"wipe_speed", "20,30"}, {"wipe_min", std::to_string(wanted) + ",1"},
        {"wipe_return", "0,0"}, {"wipe_inside_start", "0,0"}, {"wipe_inside_end", "1,1"},
        {"wipe_extra_perimeter", shape == Shape::ExtraLoop ? "1,0" : "0,0"},
        {"retract_before_wipe", std::string(before) + ",100%"},
        {"nozzle_diameter", "0.4,0.8"}, {"min_layer_height", "0.05,0.05"}, {"max_layer_height", "0.3,0.3"},
        {"retract_lift", shape == Shape::Lift ? "0.6,0" : "0,0"},
        {"wipe_lift", "0,0"}, {"use_firmware_retraction", "0"}});
    if (shape == Shape::Toolchange)
        config.set_deserialize_strict({{"perimeter_extruder", "2"}, {"infill_extruder", "2"}, {"solid_infill_extruder", "2"}});
    if (shape == Shape::OtherSupportComplete)
        config.set_deserialize_strict("complete_objects", "1");
    if (shape == Shape::OtherSupportParallel)
        config.set_deserialize_strict("parallel_objects_step", "1");
    if (shape == Shape::BrimToObject || shape == Shape::OutsideBrims)
        config.set_deserialize_strict({{"brim_width", "2"}});
    if (shape == Shape::SkirtToObject)
        config.set_deserialize_strict({{"skirts", "1"}});
    if (shape == Shape::Support || shape == Shape::SupportVoid || shape == Shape::SupportToObject || shape == Shape::ObjectToSupport || outside_supports || shape == Shape::GlobalToSupport)
        config.set_deserialize_strict({{"support_material", "1"}, {"support_material_style", "grid"},
            {"support_material_extruder", "1"}, {"support_material_interface_extruder", "1"},
            {"support_material_contact_distance", "0.2"}, {"dont_support_bridges", "0"}});
    const bool routed = shape == Shape::RoutedDetour || shape == Shape::LimitedDetour;
    if (routed)
        config.set_deserialize_strict({{"avoid_crossing_perimeters", "1"}, {"avoid_crossing_top", "0"},
            {"avoid_crossing_not_first_layer", "0"},
            {"avoid_crossing_perimeters_max_detour", shape == Shape::LimitedDetour ? "0.001" : "0"}});
    Model model;
    ModelObject *object = model.add_object();
    TriangleMesh mesh = make_cube(20., 20., 1.2);
    if (routed) {
        mesh = make_cube(20., 4., 1.2);
        TriangleMesh top = make_cube(20., 4., 1.2);
        top.translate(Vec3f(0.f, 16.f, 0.f));
        mesh.merge(top);
        TriangleMesh left = make_cube(4., 12., 1.2);
        left.translate(Vec3f(0.f, 4.f, 0.f));
        mesh.merge(left);
        TriangleMesh right = make_cube(4., 12., 1.2);
        right.translate(Vec3f(16.f, 4.f, 0.f));
        mesh.merge(right);
    }
    if (shape == Shape::Support || shape == Shape::SupportVoid || shape == Shape::SupportToObject || shape == Shape::ObjectToSupport || outside_supports || shape == Shape::GlobalToSupport) {
        mesh = make_cube(4., 8., 2.);
        TriangleMesh roof = make_cube(12., 8., 0.6);
        roof.translate(Vec3f(0.f, 0.f, 2.f));
        mesh.merge(roof);
    }
    if (shape == Shape::Islands) {
        TriangleMesh other = make_cube(20., 20., 1.2);
        other.translate(Vec3f(30.f, 0.f, 0.f));
        mesh.merge(other);
    }
    object->add_volume(mesh, ModelVolumeType::MODEL_PART, false);
    object->add_instance();
    if (shape == Shape::Instances || outside_supports || shape == Shape::OutsideBrims)
        object->add_instance()->set_offset(Vec3d(40., 0., 0.));
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
    const slic3r_api::ConstPrintingPlan original_plan(reinterpret_cast<const printing_plan_handle *>(&plan));
    const LayerRegionIsland *region = nullptr;
    uint32_t catalog = PRINTING_OBJECT_INSTANCE_ID_NONE;
    uint32_t source_catalog = PRINTING_OBJECT_INSTANCE_ID_NONE;
    const Layer *mask = nullptr;
    for (const PrintingGroup &group : plan.groups)
        for (const PrintingLayerGroup &layer : group.layers)
            for (const PrintingToolGroup &tool : layer.tool_groups)
                for (const PrintingExtrusion &extrusion : tool.extrusions) {
                    if (region != nullptr || extrusion.region_island == nullptr)
                        continue;
                    if (shape == Shape::Instances && extrusion.printing_catalog_id != PRINTING_OBJECT_INSTANCE_ID_NONE) {
                        if (original_plan.object_instance(extrusion.printing_catalog_id).object_instance_idx() == 0) {
                            source_catalog = extrusion.printing_catalog_id;
                            continue;
                        }
                    }
                    const slic3r_api::LayerRegionIsland view(reinterpret_cast<const layer_region_island_handle *>(extrusion.region_island));
                    if (view.region_count() == 0)
                        continue;
                    const Layer *candidate = reinterpret_cast<const Layer *>(view.region(0).layer().handle());
                    const bool support = candidate->get_property<LayerSupportProperty>() != nullptr;
                    if (shape == Shape::Support || shape == Shape::SupportVoid ? support : !support && candidate->id() == 3) {
                        region = extrusion.region_island;
                        catalog = extrusion.printing_catalog_id;
                        mask = reinterpret_cast<const Layer *>(view.region(0).layer().handle());
                    }
                }
    REQUIRE(mask != nullptr);
    const LayerRegionIsland *source_region = region;
    const LayerRegionIsland *target_region = region;
    if (shape == Shape::Islands) {
        REQUIRE(mask->islands().size() == 2);
        source_region = &mask->islands().front().regions_islands().front();
        target_region = &mask->islands().back().regions_islands().front();
    }
    if (shape == Shape::SupportToObject || shape == Shape::ObjectToSupport ||
        shape == Shape::BrimToObject || shape == Shape::SkirtToObject || outside_pair) {
        const LayerRegionIsland *auxiliary_region = nullptr;
        uint32_t auxiliary_catalog = catalog;
        for (const PrintingGroup &group : plan.groups)
            for (const PrintingLayerGroup &layer : group.layers)
                for (const PrintingToolGroup &tool : layer.tool_groups)
                    for (const PrintingExtrusion &extrusion : tool.extrusions) {
                        if (extrusion.region_island == nullptr)
                            continue;
                        const slic3r_api::LayerRegionIsland view(reinterpret_cast<const layer_region_island_handle *>(extrusion.region_island));
                        if (view.region_count() == 0)
                            continue;
                        const slic3r_api::Layer owner = view.region(0).layer();
                        const bool selected = shape == Shape::BrimToObject || shape == Shape::OutsideBrims ? slic3r_api::layer_is_brim(owner) :
                            shape == Shape::SkirtToObject ? extrusion.printing_catalog_id == PRINTING_OBJECT_INSTANCE_ID_NONE :
                            owner.properties().get(slic3r_api::LayerSupportProperty::key) != nullptr;
                        if (selected) {
                            auxiliary_region = extrusion.region_island;
                            auxiliary_catalog = extrusion.printing_catalog_id;
                        }
                    }
        REQUIRE(auxiliary_region != nullptr);
        if (outside_pair) {
            target_region = auxiliary_region;
            mask = reinterpret_cast<const Layer *>(slic3r_api::LayerRegionIsland(reinterpret_cast<const layer_region_island_handle *>(auxiliary_region)).region(0).layer().handle());
            catalog = auxiliary_catalog;
            if (shape != Shape::GlobalToSupport)
                source_region = auxiliary_region;
            source_catalog = shape == Shape::GlobalToSupport ? PRINTING_OBJECT_INSTANCE_ID_NONE :
                (catalog == 0 ? 1 : 0);
            if (shape != Shape::GlobalToSupport) {
                REQUIRE(original_plan.object_instance_count() == 2);
                REQUIRE(source_catalog != catalog);
                CHECK(slic3r_api::LayerRegionIsland(reinterpret_cast<const layer_region_island_handle *>(source_region)).region(0).layer().print_z() ==
                    slic3r_api::LayerRegionIsland(reinterpret_cast<const layer_region_island_handle *>(target_region)).region(0).layer().print_z());
            }
        } else if (shape == Shape::ObjectToSupport)
            target_region = auxiliary_region;
        else {
            source_region = auxiliary_region;
            source_catalog = auxiliary_catalog;
        }
    }
    if (shape == Shape::RegionlessToObject)
        source_region = nullptr;
    CHECK((mask->get_property<LayerSupportProperty>() != nullptr) ==
        (shape == Shape::Support || shape == Shape::SupportVoid || outside_supports || shape == Shape::GlobalToSupport));
    const Layer *crossing_mask = mask;
    if (mask->get_property<LayerSupportProperty>() != nullptr) {
        crossing_mask = nullptr;
        for (const Layer &candidate : mask->object()->layers()) {
            if (candidate.scaled_print_z() >= mask->scaled_print_z() + SCALED_EPSILON)
                break;
            crossing_mask = &candidate;
        }
        REQUIRE(crossing_mask != nullptr);
    }
    const BoundingBox bounds = get_extents(crossing_mask->lslices());
    Point start = bounds.center();
    Point end(bounds.max.x() + scale_i(3.), start.y());
    if (shape == Shape::SupportVoid) {
        start = Point(bounds.max.x() + scale_i(2.), bounds.min.y() - scale_i(3.));
        end = Point(bounds.max.x() + scale_i(2.), bounds.max.y() + scale_i(3.));
    }
    if (shape == Shape::Instances)
        end = bounds.center();
    if (routed) {
        start.x() = bounds.min.x() + scale_i(2.);
        end.x() = bounds.max.x() - scale_i(2.);
    }
    if (detour)
        start.x() = bounds.min.x() - scale_i(3.);
    const slic3r_api::ConstPrintingPlan plan_view(reinterpret_cast<const printing_plan_handle *>(&plan));
    PrintingExtrusion identity;
    identity.region_island = region;
    identity.printing_catalog_id = catalog;
    const c_point shift = plan_view.object_instance_shift(slic3r_api::ConstPrintingExtrusion(
        reinterpret_cast<const printing_extrusion_handle *>(&identity)));
    Point oracle_shift;
    if (shape == Shape::Instances || outside_supports || shape == Shape::OutsideBrims) {
        REQUIRE(source_catalog != PRINTING_OBJECT_INSTANCE_ID_NONE);
        identity.printing_catalog_id = source_catalog;
        const c_point source_shift = plan_view.object_instance_shift(slic3r_api::ConstPrintingExtrusion(
            reinterpret_cast<const printing_extrusion_handle *>(&identity)));
        start += Point(source_shift.x, source_shift.y);
        if (other_support) {
            oracle_shift = Point(source_shift.x - shift.x, source_shift.y - shift.y);
            end += oracle_shift;
        }
    } else {
        if (shape != Shape::SkirtToObject && !outside_pair)
            source_catalog = catalog;
        start += Point(shift.x, shift.y);
    }
    const Point target_start = end + Point(shift.x, shift.y);
    AvoidCrossingPerimeters oracle;
    oracle.init_layer(*mask);
    const std::vector<const Layer *> printed{crossing_mask};
    oracle.prepare_crossing_test({*mask, printed, catalog,
        static_cast<extruder_index_t>(shape == Shape::Toolchange ? 1 : 0),
        scale_i(shape == Shape::Toolchange ? 0.4 : 0.2), []() {}});
    Polyline direct;
    direct.points = {start - Point(shift.x, shift.y), end};
    // The core has no instance placement; query the other instance's copied object area in its own coordinates.
    for (Point &point : direct.points)
        point -= oracle_shift;
    REQUIRE(oracle.can_cross_perimeter(direct, true) == (shape != Shape::SupportVoid));
    plan.groups.clear();
    plan.groups.emplace_back();
    plan.groups.back().printing_catalog_ids.push_back(catalog);
    if (source_catalog != catalog)
        plan.groups.back().printing_catalog_ids.push_back(source_catalog);
    plan.groups.back().layers.emplace_back();
    plan.groups.back().layers.back().print_z = mask->scaled_print_z();
    plan.groups.back().layers.back().tool_groups.emplace_back();
    PrintingToolGroup &source_tool = plan.groups.back().layers.back().tool_groups.back();
    source_tool.extruder_id = 0;
    const bool loop = shape == Shape::LongLoop || shape == Shape::EqualLoop ||
        shape == Shape::ShortLoop || shape == Shape::Fragmented || shape == Shape::ExtraLoop;
    std::unique_ptr<ExtrusionEntity> source;
    if (loop) {
        source = shape == Shape::Fragmented ? make_fragmented_loop(true) : make_external_loop(0.);
        // Translate each fragment without changing the loop's inherited properties.
        std::function<void(slic3r_api::MutableExtrusionEntity)> shift_loop =
            [&shift_loop, &start](slic3r_api::MutableExtrusionEntity entity) {
                if (entity.point_count() != 0)
                    REQUIRE(entity.translate({start.x(), start.y()}));
                for (uint32_t idx = 0; idx < entity.child_count(); ++idx)
                    shift_loop(entity.child_mutable(idx));
            };
        shift_loop(entity_view(*source));
    } else {
        ArcPolyline line;
        line.append(start - Point(0, scale_i(5.)));
        line.append(start);
        source = std::make_unique<ExtrusionPath>(line,
            ExtrusionAttributes(ExtrusionRole::Perimeter, ExtrusionFlow(0.2, 0.4f, 0.2f)), nullptr, true);
    }
    append_printing_extrusion(source_tool, std::move(source));
    if (shape == Shape::ZOffsets)
        entity_view(*source_tool.extrusions.back().root).get_or_add(slic3r_api::EPropertyZOffset::key).set(scale_i(0.03));
    source_tool.extrusions.back().region_island = source_region;
    source_tool.extrusions.back().printing_catalog_id = source_catalog;
    if (shape == Shape::InterLayer) {
        // An empty intermediate group must not hide the next printable scope.
        plan.groups.front().layers.emplace_back();
        plan.groups.front().layers.back().print_z = mask->scaled_print_z() + scale_i(0.1);
        plan.groups.emplace_back();
        plan.groups.back().printing_catalog_ids.push_back(catalog);
        plan.groups.back().layers.emplace_back();
        plan.groups.back().layers.back().print_z = mask->scaled_print_z() + scale_i(0.2);
        plan.groups.back().layers.back().tool_groups.emplace_back();
        plan.groups.back().layers.back().tool_groups.back().extruder_id = 0;
    } else if (shape == Shape::Toolchange) {
        plan.groups.back().layers.back().tool_groups.emplace_back();
        plan.groups.back().layers.back().tool_groups.back().extruder_id = 1;
    }
    PrintingToolGroup &target_tool = plan.groups.back().layers.back().tool_groups.back();
    ArcPolyline target_line;
    target_line.append(target_start);
    target_line.append(target_start + Point(0, scale_i(5.)));
    append_printing_extrusion(target_tool, std::make_unique<ExtrusionPath>(target_line,
        ExtrusionAttributes(ExtrusionRole::Perimeter, ExtrusionFlow(0.2, 0.4f, 0.2f)), nullptr, true));
    target_tool.extrusions.back().region_island = target_region;
    target_tool.extrusions.back().printing_catalog_id = catalog;
    if (shape == Shape::ZOffsets)
        entity_view(*target_tool.extrusions.back().root).get_or_add(slic3r_api::EPropertyZOffset::key).set(scale_i(0.07));
    if (certain)
        run_layer_plugins(print, {TRANSITION_SCOPE_PLUGIN, TOOLCHANGE_PLUGIN, RETRACTION_PLUGIN,
            WIPE_PLUGIN, CROSSING_WIPE_PLUGIN, ENTRY_STATE_PLUGIN, TRAVEL_PLUGIN});
    else
        run_layer_plugins(print, {TRANSITION_SCOPE_PLUGIN, TOOLCHANGE_PLUGIN, RETRACTION_PLUGIN,
            WIPE_PLUGIN, ENTRY_STATE_PLUGIN, routed ?
                "layer_extrusion_edit.travel.avoid_crossing_perimeters" : TRAVEL_PLUGIN});
    const slic3r_api::MutableExtrusionEntity source_root = find_scope(entity_view(
        *plan.groups.front().layers.front().tool_groups.front().extrusions.front().root));
    const slic3r_api::MutableExtrusionEntity target_root = find_scope(entity_view(*target_tool.extrusions.back().root));
    const OrderedExtrusionScope source_scope(source_root, scope_property_key());
    const OrderedExtrusionScope target_scope(target_root, scope_property_key());
    const slic3r_api::MutableExtrusionEntity first_wipe = find_role(source_root, RAW_EXTRUSION_ROLE_WIPE);
    REQUIRE(first_wipe.valid() == certain);
    if (certain) {
        std::vector<GeometricLeaf> before_leaves;
        collect_geometric_leaves(source_scope.after(), std::nullopt, before_leaves);
        REQUIRE(before_leaves.size() == (std::string(before) == "100%" ? 1 : 2));
        const c_extrusion_segment tail = before_leaves.back().entity.segment(before_leaves.back().entity.segment_count() - 1);
        CHECK(c_point_distance_to(tail.point_b, target_scope.travel().segment(0).point_a) == Approx(0.));
        CHECK(c_point_distance_to(tail.point_b, {start.x(), start.y()}) > SCALED_EPSILON);
        double length = 0.;
        for (const GeometricLeaf &leaf : before_leaves)
            length += unscaled(leaf.entity.local_length());
        CHECK(length == Approx(5.));
        return;
    }
    const c_extrusion_segment original = target_scope.travel().segment(0);
    CHECK(original.point_a.x == start.x());
    CHECK(original.point_a.y == start.y());
    if (routed) {
        Polyline generated;
        generated.points.emplace_back(original.point_a.x - shift.x, original.point_a.y - shift.y);
        for (const c_extrusion_segment &segment : target_scope.travel().segments())
            generated.points.emplace_back(segment.point_b.x - shift.x, segment.point_b.y - shift.y);
        CHECK(oracle.can_cross_perimeter(generated, true) == (shape == Shape::LimitedDetour));
        CHECK((target_scope.travel().segment_count() == 1) == (shape == Shape::LimitedDetour));
    }
    if (shape == Shape::OccupiedAfter) {
        slic3r_api::MutableExtrusionEntity occupied = slic3r_api::LayerExtrusionEdit::ExtrusionScope::append_phase_leaf(source_scope.after());
        REQUIRE(occupied.set_segments({original}));
    }
    if (shape == Shape::Deposit) {
        // A one-point deposit is added after the segmented source at the same nozzle position.
        slic3r_api::MutableExtrusionEntity deposit = slic3r_api::LayerExtrusionEdit::ExtrusionScope::append_phase_leaf(source_scope.content());
        deposit.get_or_add(slic3r_api::EPropertyAttributes::key).extrusion_role(RAW_EXTRUSION_ROLE_PERIMETER);
        REQUIRE(deposit.set_points({original.point_a}));
        deposit.get_or_add(slic3r_api::EPropertySpecialCommand::key).set(C_EXTRUSION_SPECIAL_COMMAND_DEPOSIT, 0.1);
    }
    if (shape == Shape::WrongZ) {
        c_extrusion_segment moved = original;
        moved.z_offset_a += scale_i(0.01);
        REQUIRE(target_scope.travel().set_segments({moved}));
    }
    if (detour) {
        const coord_t outside_y = bounds.max.y() + shift.y + scale_i(3.);
        c_extrusion_segment a = original, b = original, c = original;
        a.point_b = {original.point_a.x, outside_y};
        b.point_a = a.point_b;
        b.point_b = {original.point_b.x, outside_y};
        c.point_a = b.point_b;
        REQUIRE(target_scope.travel().set_segments({a, b, c}));
        Polyline detour;
        detour.points = {Point(a.point_a.x - shift.x, a.point_a.y - shift.y),
            Point(a.point_b.x - shift.x, a.point_b.y - shift.y),
            Point(b.point_b.x - shift.x, b.point_b.y - shift.y),
            Point(c.point_b.x - shift.x, c.point_b.y - shift.y)};
        CHECK_FALSE(oracle.can_cross_perimeter(detour, true));
    }
    if (split) {
        slic3r_api::MutableExtrusionEntity final_part = split_travel_leaf(
            target_scope.travel(), 0.5 * unscaled(target_scope.travel().local_length()));
        const slic3r_api::ExtrusionEntity first_part = target_scope.travel().child(0);
        if (shape == Shape::SplitTravel) {
            // Only the whole ordered path crosses: the first part stays inside the object.
            Polyline head;
            for (uint32_t idx = 0; idx < first_part.point_count(); ++idx)
                head.points.emplace_back(first_part.point(idx).x - shift.x, first_part.point(idx).y - shift.y);
            CHECK_FALSE(oracle.can_cross_perimeter(head, true));
        }
        if (shape == Shape::BrokenSplit) {
            std::vector<c_extrusion_segment> moved = final_part.segments();
            moved.front().point_a.y += scale_i(1.);
            REQUIRE(final_part.set_segments(moved));
        }
    }
    if (shape == Shape::Blocked) {
        ScopedActivePlugins active({TRANSITION_SCOPE_PLUGIN, CROSSING_WIPE_PLUGIN});
        Orchestrator &host = Orchestrator::instance();
        host.block_unsatisfied_plugin_dependencies();
        CHECK(host.blocked_plugin_activations().count(CROSSING_WIPE_PLUGIN) == 1);
        CHECK_FALSE(host.is_plugin_active(CROSSING_WIPE_PLUGIN));
        Steps::StepLayerExtrusionEdition::run_step(host, print);
    } else
        run_layer_plugins(print, {TRANSITION_SCOPE_PLUGIN, WIPE_PLUGIN, CROSSING_WIPE_PLUGIN});
    const slic3r_api::MutableExtrusionEntity wipe = find_role(source_root, RAW_EXTRUSION_ROLE_WIPE);
    if (shape == Shape::Deposit || shape == Shape::OccupiedAfter || detour ||
        shape == Shape::RoutedDetour || shape == Shape::Blocked || shape == Shape::WrongZ ||
        shape == Shape::BrokenSplit || shape == Shape::SupportVoid ||
        shape == Shape::OtherSupportComplete || shape == Shape::OtherSupportParallel) {
        CHECK_FALSE(wipe.valid());
        return;
    }
    REQUIRE(wipe.valid());
    std::vector<GeometricLeaf> leaves;
    collect_geometric_leaves(wipe, std::nullopt, leaves);
    REQUIRE_FALSE(leaves.empty());
    const c_extrusion_segment first = leaves.front().entity.segment(0);
    const c_extrusion_segment last = leaves.back().entity.segment(leaves.back().entity.segment_count() - 1);
    CHECK(first.point_a.x == original.point_a.x);
    CHECK(first.point_a.y == original.point_a.y);
    CHECK(last.point_b.x == original.point_a.x);
    CHECK(last.point_b.y == original.point_a.y);
    CHECK(last.z_offset_b == first.z_offset_a);
    const slic3r_api::EPropertyZOffset *source_z = source_root.get(slic3r_api::EPropertyZOffset::key);
    const slic3r_api::EPropertyZOffset *target_z = target_root.get(slic3r_api::EPropertyZOffset::key);
    const coord_t source_base_z = mask->scaled_print_z() + (source_z == nullptr ? 0 : source_z->get());
    const coord_t target_base_z = plan.groups.back().layers.back().print_z + (target_z == nullptr ? 0 : target_z->get());
    CHECK(source_base_z + first.z_offset_a == target_base_z + original.z_offset_a);
    CHECK(first_leaf_segment(target_scope.travel().readonly()).point_a.x == original.point_a.x);
    CHECK(first_leaf_segment(target_scope.travel().readonly()).point_a.y == original.point_a.y);
    double length = 0.;
    for (size_t idx = 0; idx < leaves.size(); ++idx) {
        length += unscaled(leaves[idx].entity.local_length());
        if (idx > 0) {
            const slic3r_api::ExtrusionEntity &previous = leaves[idx - 1].entity;
            const c_extrusion_segment tail = previous.segment(previous.segment_count() - 1);
            CHECK(c_point_distance_to(tail.point_b, leaves[idx].entity.segment(0).point_a) == Approx(0.));
        }
    }
    const double expected_length = shape == Shape::ShortLoop || shape == Shape::ExtraLoop ? 80. :
        shape == Shape::EqualLoop ? 40. : wanted;
    CHECK(length == Approx(expected_length).margin(0.00001));
    if (shape == Shape::EqualLoop || shape == Shape::ExtraLoop) {
        // A whole counter-clockwise turn encloses the square; an out-and-back path has zero signed area.
        double twice_area = 0.;
        for (const GeometricLeaf &leaf : leaves)
            for (const c_extrusion_segment &segment : leaf.entity.segments()) {
                const double ax = unscaled(segment.point_a.x - original.point_a.x);
                const double ay = unscaled(segment.point_a.y - original.point_a.y);
                const double bx = unscaled(segment.point_b.x - original.point_a.x);
                const double by = unscaled(segment.point_b.y - original.point_a.y);
                twice_area += ax * by - bx * ay;
            }
        CHECK(twice_area == Approx(shape == Shape::EqualLoop ? 200. : 400.).margin(0.00001));
    }
    const double stationary = std::string(before) == "100%" ? 2. : std::string(before) == "20%" ? 0.4 : 0.;
    CHECK(find_axis_target(source_scope.after(), C_EXTRUSION_AXIS_OPERATION_RETRACT_TO,
        stationary, shape == Shape::Toolchange).valid());
    if (shape == Shape::Residual && stationary < 2.)
        CHECK(find_axis_target(source_scope.after(), C_EXTRUSION_AXIS_OPERATION_RETRACT_TO,
            2., false).valid());
    if (shape == Shape::Lift) {
        run_layer_plugins(print, {TRANSITION_SCOPE_PLUGIN, WIPE_PLUGIN, CROSSING_WIPE_PLUGIN,
            "layer_extrusion_edit.lift.default"});
        CHECK(target_scope.travel().segment(0).point_a.x == original.point_a.x);
        CHECK(target_scope.travel().segment(0).point_a.y == original.point_a.y);
        std::vector<GeometricLeaf> lifted;
        collect_geometric_leaves(target_scope.travel(), std::nullopt, lifted);
        coord_t maximum_z = 0;
        for (const GeometricLeaf &leaf : lifted)
            for (const c_extrusion_segment &segment : leaf.entity.segments())
                maximum_z = std::max(maximum_z, std::max(segment.z_offset_a, segment.z_offset_b));
        CHECK(maximum_z == scale_i(0.6));
    }
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
                print_view, {0, idx, true, true}, nullptr);
            CHECK(input.target->identity.global_object);
            CHECK(input.target->identity.object == 0);
            CHECK_FALSE(input.target->first_layer);
            CHECK(input.target->print_z == target_layer->scaled_print_z());
            CHECK_FALSE(input.target->lslices.empty());
            CHECK_FALSE(input.target->regions.empty());
            plugin_run_context ctx{};
            ctx.plugin_storage = reinterpret_cast<storage_handle *>(&storage);
            AC::LayerInputCache cache;
            const AC::Input &cached = cache.get(print_view, auxiliary.auxiliary_layer(idx), &ctx, nullptr);
            CHECK(cached.target->identity.global_object);
            CHECK(cached.target->identity.object == input.target->identity.object);
            CHECK(cached.target->identity.layer == input.target->identity.layer);
            CHECK(cached.target->identity.auxiliary == input.target->identity.auxiliary);
            CHECK(cached.target->print_z == input.target->print_z);
            CHECK(&cache.get(print_view, auxiliary.auxiliary_layer(idx), &ctx, nullptr) == &cached);
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

// A fragmented square must continue from the nozzle seam and use its complete turn for return rounding.
TEST_CASE("Fragmented loop wipes start at the seam and follow the complete loop",
          "[plugins][layer-extrusion-edit][retraction][wipe][fragmented-loop]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    for (const bool wrapped : {false, true})
    for (const bool repeat_turn : {false, true})
    for (const bool returning : {false, true}) {
        CAPTURE(returning, repeat_turn, wrapped);
        Print print;
        Model model;
        prepare_wipe_print(print, model, repeat_turn ? "45" : "8", "0", "0", "0", returning ? "1" : "0");
        PrintingToolGroup &tool = print.mutable_printing_plan().groups.front()
            .layers.front().tool_groups.front();
        tool.extrusions.front().root = make_fragmented_loop(wrapped);
        run_layer_plugins(print, {TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN, WIPE_PLUGIN});
        OrderedExtrusionScope source(find_scope(entity_view(*tool.extrusions.front().root)),
                                     scope_property_key());
        const slic3r_api::MutableExtrusionEntity wipe = find_role(source.after(), RAW_EXTRUSION_ROLE_WIPE);
        REQUIRE(wipe.valid());
        std::vector<GeometricLeaf> leaves;
        collect_geometric_leaves(wipe, std::nullopt, leaves);
        REQUIRE(!leaves.empty());
        const c_extrusion_segment first = leaves.front().entity.segment(0);
        CHECK(first.point_a.x == 0);
        CHECK(first.point_a.y == 0);
        CHECK(first.point_b.x > first.point_a.x);
        CHECK(first.point_b.y == 0);
        distf_t length = 0.;
        for (const GeometricLeaf &leaf : leaves)
            length += leaf.entity.local_length();
        const double requested = repeat_turn ? 45. : 8.;
        const double expected_length = returning ? (repeat_turn ? 80. : 40.) : requested;
        CHECK(unscaled(length) == Approx(expected_length));
        const slic3r_api::ExtrusionEntity last = leaves.back().entity;
        const c_point end = last.segment(last.segment_count() - 1).point_b;
        CHECK(end.x == (returning ? 0 : scale_i(repeat_turn ? 5. : 8.)));
        CHECK(end.y == 0);
    }
}

// A terminal Bridge side prevents an outgoing wipe even when earlier sides belong to the same loop.
TEST_CASE("Fragmented loop with a terminal Bridge has no wipe",
          "[plugins][layer-extrusion-edit][retraction][wipe][fragmented-loop]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_wipe_print(print, model);
    PrintingToolGroup &tool = print.mutable_printing_plan().groups.front()
        .layers.front().tool_groups.front();
    tool.extrusions.front().root = make_fragmented_loop(true, true);
    run_layer_plugins(print, {TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN, WIPE_PLUGIN});
    OrderedExtrusionScope source(find_scope(entity_view(*tool.extrusions.front().root)),
                                 scope_property_key());
    CHECK_FALSE(find_role(source.after(), RAW_EXTRUSION_ROLE_WIPE).valid());
}

// Complete square orientation keeps the departure inside material and places the incoming approach inside its seam.
TEST_CASE("Fragmented loop inside approaches use the complete turn",
          "[plugins][layer-extrusion-edit][retraction][wipe][fragmented-loop]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    for (const bool wrapped : {false, true}) {
        CAPTURE(wrapped);
        Print print;
        Model model;
        prepare_wipe_print(print, model, "8", "0", "1", "1");
        PrintingToolGroup &tool = print.mutable_printing_plan().groups.front()
            .layers.front().tool_groups.front();
        tool.extrusions.front().root = make_fragmented_loop(wrapped);
        tool.extrusions.back().root = make_fragmented_loop(wrapped, false, 20.);
        run_layer_plugins(print, {TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN, WIPE_PLUGIN});
        OrderedExtrusionScope source(find_scope(entity_view(*tool.extrusions.front().root)),
                                     scope_property_key());
        const slic3r_api::MutableExtrusionEntity wipe = find_role(source.after(), RAW_EXTRUSION_ROLE_WIPE);
        REQUIRE(wipe.valid());
        std::vector<GeometricLeaf> leaves;
        collect_geometric_leaves(wipe, std::nullopt, leaves);
        REQUIRE(!leaves.empty());
        const slic3r_api::ExtrusionEntity last = leaves.back().entity;
        const c_point end = last.segment(last.segment_count() - 1).point_b;
        CHECK(end.x == scale_i(8.));
        // Half the 0.4 mm nozzle gives a 0.2 mm inward end offset and a 0.2 mm start displacement along the corner bisector.
        const coord_t depth = scale_i(0.2);
        CHECK(end.y == depth);
        OrderedExtrusionScope target(find_scope(entity_view(*tool.extrusions.back().root)),
                                     scope_property_key());
        leaves.clear();
        collect_geometric_leaves(target.before(), std::nullopt, leaves);
        CHECK(leaves.size() == 1);
        if (leaves.empty())
            continue;
        const c_extrusion_segment approach = leaves.front().entity.segment(0);
        const coord_t diagonal = coord_t(std::llround(double(depth) / std::sqrt(2.)));
        CHECK(approach.point_a.x == scale_i(20.) + diagonal);
        CHECK(approach.point_a.y == diagonal);
        CHECK(approach.point_b.x == scale_i(20.));
        CHECK(approach.point_b.y == 0);
    }
}

// A bridge cuts the forward prefix, which retains the extra perimeter distance but cannot repeat, round up or end inside as a complete loop.
TEST_CASE("Fragmented loop wipes stop before an intermediate Bridge",
          "[plugins][layer-extrusion-edit][retraction][wipe][fragmented-loop]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    for (const bool extra_distance : {false, true})
    for (const bool wrapped : {false, true})
    for (const bool returning : {false, true}) {
        CAPTURE(wrapped, returning, extra_distance);
        Print print;
        Model model;
        prepare_wipe_print(print, model, extra_distance ? "8" : "45", "0", "0", "1",
                           returning ? "1" : "0", "20%", extra_distance ? "4" : "0");
        PrintingToolGroup &tool = print.mutable_printing_plan().groups.front()
            .layers.front().tool_groups.front();
        tool.extrusions.front().root = make_fragmented_loop(wrapped, false, 0., 2);
        run_layer_plugins(print, {TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN, WIPE_PLUGIN});
        OrderedExtrusionScope source(find_scope(entity_view(*tool.extrusions.front().root)),
                                     scope_property_key());
        const slic3r_api::MutableExtrusionEntity wipe = find_role(source.after(), RAW_EXTRUSION_ROLE_WIPE);
        REQUIRE(wipe.valid());
        std::vector<GeometricLeaf> leaves;
        collect_geometric_leaves(wipe, std::nullopt, leaves);
        REQUIRE(!leaves.empty());
        distf_t length = 0.;
        for (const GeometricLeaf &leaf : leaves) {
            length += leaf.entity.local_length();
            for (uint32_t idx = 0; idx < leaf.entity.segment_count(); ++idx) {
                CHECK(leaf.entity.segment(idx).point_a.y == 0);
                CHECK(leaf.entity.segment(idx).point_b.y == 0);
            }
        }
        CHECK(unscaled(length) == Approx(returning ? (extra_distance ? 12. : 20.) : 10.));
        const c_point start = leaves.front().entity.segment(0).point_a;
        CHECK(start.x == 0);
        CHECK(start.y == 0);
        const slic3r_api::ExtrusionEntity last = leaves.back().entity;
        const c_point end = last.segment(last.segment_count() - 1).point_b;
        CHECK(end.x == (returning ? 0 : scale_i(10.)));
        CHECK(end.y == 0);
    }
}

// Percent settings divide the absolute retraction target between the stationary request and moving leaves.
TEST_CASE("Before-wipe percent sets the stationary retraction target",
          "[plugins][layer-extrusion-edit][retraction][wipe]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    for (const std::pair<const char *, double> &setting : {std::pair<const char *, double>{"0%", 0.}, {"20%", 0.4}, {"100%", 2.}}) {
        CAPTURE(setting.first);
        Print print;
        Model model;
        prepare_wipe_print(print, model, "8", "0", "0", "0", "0", setting.first);
        PrintingToolGroup &tool = print.mutable_printing_plan().groups.front()
            .layers.front().tool_groups.front();
        run_layer_plugins(print, {TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN, WIPE_PLUGIN});
        OrderedExtrusionScope source(find_scope(entity_view(*tool.extrusions.front().root)),
                                     scope_property_key());
        CHECK(find_axis_target(source.after(), C_EXTRUSION_AXIS_OPERATION_RETRACT_TO, setting.second, false).valid());
        const slic3r_api::MutableExtrusionEntity wipe = find_role(source.after(), RAW_EXTRUSION_ROLE_WIPE);
        REQUIRE(wipe.valid());
        std::vector<GeometricLeaf> leaves;
        collect_geometric_leaves(wipe, std::nullopt, leaves);
        REQUIRE(!leaves.empty());
        size_t retracting_leaves = 0;
        for (const GeometricLeaf &leaf : leaves) {
            if (!leaf.extrusion_axis)
                continue;
            ++retracting_leaves;
            CHECK(leaf.extrusion_axis->operation == C_EXTRUSION_AXIS_OPERATION_RETRACT_TO);
            CHECK(leaf.extrusion_axis->value == Approx(2.));
        }
        CHECK(retracting_leaves == (setting.second == 2. ? 0 : 1));
    }
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

TEST_CASE("Retraction wipe joins an open seam gap before repeating a marked loop",
          "[plugins][layer-extrusion-edit][retraction][wipe][wipe-seam-gap]")
{
    const bool closed = GENERATE(false, true);
    const bool return_to_seam = GENERATE(false, true);
    const bool fragmented = GENERATE(false, true);
    CAPTURE(closed, return_to_seam, fragmented);
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_wipe_print(print, model, "0.75", "0", "0", "0", return_to_seam ? "1" : "0");
    PrintingLayerGroup &layer = print.mutable_printing_plan().groups.front().layers.front();
    layer.print_z = 4800000;
    PrintingToolGroup &tool = layer.tool_groups.front();
    tool.extrusions.clear();
    Points points{Point(-17824989, 2135139), Point(-17824989, 2209927), Point(-17824989, 2284716)};
    if (closed) {
        points.push_back(points.back() + Point(100000, 0));
        points.push_back(points.front() + Point(100000, 0));
        points.push_back(points.front());
    }
    const Point tip = points.back();
    std::unique_ptr<ExtrusionEntity> perimeter = std::make_unique<ExtrusionEntity>(true, ArcPolyline(points));
    if (fragmented) {
        ExtrusionEntity::Children children;
        for (size_t idx = 1; idx < points.size(); ++idx)
            children.push_back(std::make_unique<ExtrusionEntity>(true,
                ArcPolyline(Points{points[idx - 1], points[idx]})));
        perimeter = std::make_unique<ExtrusionEntity>(std::move(children), false, false, false);
    }
    perimeter->add_property(ExtrusionAttributes(ExtrusionRole::ExternalPerimeter, ExtrusionFlow(.08, .4f, .2f)));
    entity_view(*perimeter).get_or_add(slic3r_api::EPropertyPerimeter::key)
        .perimeter_flags(C_EXTRUSION_PERIMETER_FLAG_LOOP);
    append_printing_extrusion(tool, std::move(perimeter));
    append_printing_extrusion(tool, make_print_path(20., 30.));

    // The legacy iterates stored points from the current tip, making the gap move explicit before the first point.
    ArcPolyline turn;
    turn.append(tip);
    for (const Point &point : points)
        if (turn.back() != point) turn.append(point);
    const double turn_mm = unscaled(turn.length());
    const double requested_mm = return_to_seam ? std::ceil(.75 / turn_mm) * turn_mm : .75;
    ArcPolyline expected;
    expected.append(tip);
    while (unscaled(expected.length()) < requested_mm)
        for (const Point &point : points)
            if (expected.back() != point) expected.append(point);
    expected.clip_end(expected.length() - scale_d(requested_mm));

    REQUIRE_NOTHROW(run_layer_plugins(print, {TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN, WIPE_PLUGIN}));
    OrderedExtrusionScope source(find_scope(entity_view(*tool.extrusions.front().root)), scope_property_key());
    const slic3r_api::MutableExtrusionEntity wipe = find_role(source.after(), RAW_EXTRUSION_ROLE_WIPE);
    REQUIRE(wipe.valid());
    std::vector<GeometricLeaf> leaves;
    collect_geometric_leaves(wipe, std::nullopt, leaves);
    REQUIRE_FALSE(leaves.empty());
    c_point previous{tip.x(), tip.y()};
    coord_t previous_z = 0;
    double length_mm = 0.;
    size_t segments = 0;
    for (const GeometricLeaf &leaf : leaves) {
        REQUIRE(leaf.attributes.has_value());
        CHECK(leaf.attributes->mm3_per_mm() == 0.);
        CHECK(RAW_EXTRUSION_ROLE_IS_WIPE(leaf.attributes->extrusion_role()));
        CHECK(RAW_EXTRUSION_ROLE_IS_TRAVEL(leaf.attributes->extrusion_role()));
        for (const c_extrusion_segment &segment : leaf.entity.segments()) {
            CHECK(segment.point_a.x == previous.x);
            CHECK(segment.point_a.y == previous.y);
            CHECK(segment.z_offset_a == previous_z);
            CHECK(segment.radius == 0.f);
            CHECK(segment.z_offset_b == 0);
            previous = segment.point_b;
            previous_z = segment.z_offset_b;
            ++segments;
        }
        length_mm += unscaled(leaf.entity.local_length());
    }
    CHECK(segments > points.size() - 1);
    CHECK(length_mm == Approx(requested_mm).margin(0.000002));
    CHECK(previous.x == expected.back().x());
    CHECK(previous.y == expected.back().y());
    const c_extrusion_segment first = leaves.front().entity.segment(0);
    CHECK(first.point_b.x == points[closed ? 1 : 0].x());
    CHECK(first.point_b.y == points[closed ? 1 : 0].y());
    const slic3r_api::MutableExtrusionEntity initial = find_role(source.after(), RAW_EXTRUSION_ROLE_RETRACT);
    REQUIRE(initial.valid());
    REQUIRE(initial.get(slic3r_api::EPropertyExtrusionAxis::key) != nullptr);
    CHECK(initial.get(slic3r_api::EPropertyExtrusionAxis::key)->value == Approx(.4));
    double retracted = .4;
    for (const GeometricLeaf &leaf : leaves) {
        if (!leaf.extrusion_axis)
            continue;
        CHECK(leaf.extrusion_axis->operation == C_EXTRUSION_AXIS_OPERATION_RETRACT_TO);
        CHECK_FALSE(leaf.extrusion_axis->toolchange);
        CHECK(leaf.extrusion_axis->value >= retracted);
        CHECK(leaf.extrusion_axis->value <= 2.);
        retracted = leaf.extrusion_axis->value;
    }
    if (find_axis_target(source.after(), C_EXTRUSION_AXIS_OPERATION_RETRACT_TO, 2., false).valid())
        retracted = 2.;
    CHECK(retracted == Approx(2.));
    CHECK_FALSE(source.before().valid());
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
    // A short motion-only wipe ends within the arc, so its final Z offset must be interpolated.
    prepare_wipe_print(print, model, "0.5", "0", "0", "0", "0", "100%");

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
