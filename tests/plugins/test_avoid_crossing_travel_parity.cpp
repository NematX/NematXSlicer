#include <catch2/catch.hpp>

#include <algorithm>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <vector>

#include "plugin_test_helpers.hpp"
#include "layer_extrusion_edit_transition_test_helpers.hpp"
#include "test_data.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/Api/plugin/cpp/RegionSettingsViews.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCanonical.hpp"
#include "libslic3r/GCode/AvoidCrossingPerimeters.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/PrintRegion.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/ExtrusionScopeHelpers.hpp"
#include "libslic3r/Steps/StepExtrusionOrdering.hpp"
#include "libslic3r/Steps/StepLayerExtrusionEdition.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"
#include "libslic3r/Steps/PipelineProgress.hpp"

/*
Travel parity tests check that the travel plugin integrates the core perimeter
crossing test and router without changing their route coordinates. Small
meshes are sliced into surfaces and extrusion paths, then ordered for
printing. The selected editors identify the gaps between ordered paths, carry
the position at each gap's entry, and insert travel paths. Wipe and lift
editors do not run, so they cannot change the measured connectors. A leaf here
is an extrusion entity with a polyline of at least two points and no children;
its owning PrintingExtrusion identifies its source region and object instance.
The reserved travel phase is the connector slot in an ordered extrusion.
collect_leaves marks its paths, separating inserted connectors from other
travel paths.

oracle_path computes the expected route with the core class, independently of
the travel editor. After handling travel across layer groups, it checks
activation at both endpoints and the first-layer setting, then removes the
target instance shift to work in object coordinates. The core crossing test
decides whether to call travel_to; a different source object or instance
selects external routing. The result is shifted back and its original
endpoints restored before point-by-point checks. Agreement checks integration
with core behaviour, not the core's correctness. The boundary test uses two
controlled paths on real sliced region masks, so a shared modifier edge
exercises the endpoint lookup and its interior retry. Controlled paths enter
fresh canonical plan storage before editor execution. Repeated scene passes
check detour units and stable multi-tool ordering.

Core-only checks are separate from these plugin comparisons. top_oracle_routes
slices and orders a ring without travel editing, then computes core routes
between successive leaves and copies surface counts and ordering diagnostics.
The ring checks require top-avoidance sensitivity without changing ordering.
Controlled chords across a cube test regional top avoidance directly through
the core router. A same-tool modifier disables top avoidance on part of a
shared island. Intersections are measured against top areas inset by 0.5 mm,
excluding the perimeter band where detours may pass. The crossing test still
detects the top interior when top avoidance is off, because top surfaces are
excluded from the areas treated as safe to cross; routing and crossing are
separate checks.

The core-only spacing scene routes across a ring hole. The region's slice
surfaces are empty while its raw slices remain, and a 0.8 mm perimeter width
on a 0.4 mm nozzle separates the flow-spacing inset from the nozzle fallback.
The test measures intermediate route points against the expected flow-based
inset.

The shared-top check also inspects the stored excluded top areas and verifies
that their grid points refer to the boundary's own stored contour points.

A core-only notched plate tests a narrow passage opposite a long edge. Routing
builds the core's internal boundary (the inset rings used for routes inside
the object) lazily; one resulting ring must contain points on both sides of
the passage, checking that the width-aware inset keeps it joined.

A core-only square with a distant island tests endpoint placement. The island
widens that internal boundary's bounding box so the departure stays inside it,
while grid queries within half the travel length find a ring only for arrival.
The next route point must be the departure's projection onto the square
boundary edge, rather than an arbitrary vertex.

When avoid_crossing_not_first_layer is enabled, the expected route stays
direct at the first object layer's Z, including support at that height. Scene
checks verify this exemption by height rather than by layer identifier.

    Travel tests
    |-- Plugin/core comparisons
    |   |-- Print::process: slice meshes and generate source extrusions
    |   |-- StepExtrusionOrdering, StepLayerExtrusionEdition: order paths and insert travels
    |   |-- collect_leaves: retain ordered geometry, provenance and travel-phase marks
    |   |-- enabled_endpoint, oracle_path: compute core routes and compare every point
    |   |-- lookup_at_point: check the controlled modifier-edge retry
    |   `-- oracle_path, check_scene: check enabled first-layer exemption for support by Z
    `-- Core-only checks
        |-- top_oracle_routes: copy routes, top counts and ordering from a sliced ring
        |-- travel_to, intersection_pl, can_cross_perimeter: check regional tops
        |-- travel_to: measure the raw-slice flow-spacing inset
        |-- internal_boundary: check excluded tops and borrowed grid points
        |-- travel_to, internal_boundary: check the notched passage remains joined
        `-- travel_to, internal_boundary: check grid misses and the edge projection

The Print owns borrowed leaves and regions until comparisons finish; copied
route diagnostics survive each helper's Print. A PrintingLayerGroup groups
ordered tool paths at one printing height. Across such groups the connector
has an entry position but no preceding region, so oracle_path clears the
router map and returns a direct path. Router maps are local to a scene pass or
a top_oracle_routes call, not owned by an individual layer group.
*/

namespace {
using namespace Slic3r;
using namespace Slic3r::Printing;

// Borrowed geometry and its source provenance in the final execution stream.
struct Leaf
{
    const ExtrusionEntity *entity;
    const PrintingExtrusion *owner;
    extruder_index_t tool;
    const PrintingLayerGroup *layer;
    bool generated_travel = false;
};
// Own core routers by target layer and tool; oracle_path clears the map across layer groups.
using Routers = std::map<std::pair<const Layer *, extruder_index_t>, std::unique_ptr<AvoidCrossingPerimeters>>;

// Named geometry and setting variations exercised independently by the pipeline.
enum class Scene { Hole, Islands, Top, Absolute, Percentage, FirstLayer, Instances, Tools, Support, Regional, Units };

// Own the oracle routes, ordered endpoint summary and surface counts of one sliced ring.
struct TopRoutes
{
    std::vector<Points> routes;
    std::string ordered;
    size_t region_tops = 0;
    size_t island_tops = 0;
};

// Collect polyline leaves in execution order with borrowed provenance; mark the reserved travel phase to identify inserted connectors; ignore commands and single-point deposits.
void collect_leaves(const ExtrusionEntity &entity, const PrintingExtrusion &owner,
                    extruder_index_t tool, const PrintingLayerGroup &layer, std::vector<Leaf> &out);
// Read avoid_crossing_perimeters at the first or last endpoint; after any failed lookup retry within SCALED_EPSILON along the leaf.
// Return false without source-region provenance; resolved counts only ambiguous lookups resolved by the retry.
bool enabled_endpoint(const Leaf &leaf, bool first, const PrintingPlan &plan, size_t *resolved = nullptr);
// Return the core route, optionally bypassing activation to prove a disabled endpoint prevented a detour.
// With avoid_crossing_not_first_layer enabled, the first object layer Z also exempts support at that height.
Points oracle_path(const Leaf &source, const Leaf &target, const PrintingPlan &plan,
                   const Print &print, Routers &routers, bool ignore_activation = false);
// Build a shallow square ring from four joined bars; the returned mesh owns its triangles.
TriangleMesh ring_mesh();
// Slice and compare one scene, including paired settings or repeated ordering where requested; failed checks stay visible to Catch.
void check_scene(Scene scenario);
// Slice and order a ring, returning core routes and copied diagnostics without running travel editing or making Catch assertions.
TopRoutes top_oracle_routes(bool avoid_top);

// A notch faces the middle of a long edge, where local width needs additional vertices before insetting.
TEST_CASE("Core travel boundary preserves the passage beneath a notch",
          "[plugins][travel][avoid-crossing-parity]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"layer_height", "0.2"}, {"first_layer_height", "0.2"},
        {"perimeters", "2"}, {"fill_density", "0%"}, {"top_solid_layers", "0"},
        {"bottom_solid_layers", "0"}, {"skirts", "0"}, {"brim_width", "0"},
        {"gap_fill_enabled", "0"}, {"thin_walls", "0"}, {"thin_perimeters", "0"},
        {"avoid_crossing_top", "0"}});
    TriangleMesh mesh = make_cube(30., 0.5, 0.6);
    for (double x : {0., 16.}) {
        TriangleMesh side = make_cube(14., 9.5, 0.6);
        side.translate(float(x), 0.5f, 0.f);
        mesh.merge(side);
    }
    Model model;
    ModelObject *object = model.add_object();
    object->add_volume(mesh, ModelVolumeType::MODEL_PART, false);
    object->add_instance();
    object->ensure_on_bed();
    Print print;
    print.apply(model, config);
    print.set_status_silent();
    print.process();
    const Layer &layer = print.object(0).layers().front();
    REQUIRE(layer.lslices().size() == 1);
    const BoundingBox bounds = get_extents(layer.lslices());
    const Point left = bounds.min + Point(scale_i(5.), scale_i(5.));
    const Point right = bounds.min + Point(scale_i(25.), scale_i(5.));
    AvoidCrossingPerimeters router;
    router.init_layer(layer);
    router.travel_to({layer, left, Point::Zero(), extruder_index_t(0), 0., false}, right);
    const AvoidCrossingPerimeters::Boundary &boundary = router.internal_boundary();
    size_t spanning = 0;
    for (const Polygon &ring : boundary.boundaries)
        if (ring.contains(left) && ring.contains(right)) ++spanning;
    std::cout << "notch: boundaries=" << boundary.boundaries.size() << " spanning=" << spanning << std::endl;
    CHECK(spanning == 1);
}

// A distant island keeps the outside endpoint in the routing box while only the square is close to the arrival.
TEST_CASE("Core travel endpoint fallback projects onto a boundary edge",
          "[plugins][travel][avoid-crossing-parity]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"layer_height", "0.2"}, {"first_layer_height", "0.2"},
        {"perimeters", "2"}, {"fill_density", "0%"}, {"top_solid_layers", "0"},
        {"bottom_solid_layers", "0"}, {"skirts", "0"}, {"brim_width", "0"},
        {"gap_fill_enabled", "0"}, {"thin_walls", "0"}, {"thin_perimeters", "0"},
        {"avoid_crossing_top", "0"}, {"avoid_travel_island", "0"}});
    TriangleMesh mesh = make_cube(20., 20., 0.6);
    TriangleMesh distant = make_cube(5., 5., 0.6);
    distant.translate(100.f, 100.f, 0.f);
    mesh.merge(distant);
    Model model;
    ModelObject *object = model.add_object();
    object->add_volume(mesh, ModelVolumeType::MODEL_PART, false);
    object->add_instance();
    object->ensure_on_bed();
    Print print;
    print.apply(model, config);
    print.set_status_silent();
    print.process();
    const Layer &layer = print.object(0).layers().front();
    const BoundingBox bounds = get_extents(layer.lslices());
    const Point from = bounds.min + Point(scale_i(7.123), scale_i(60.));
    const Point to = bounds.min + Point(scale_i(7.123), scale_i(19.));
    AvoidCrossingPerimeters router;
    router.init_layer(layer);
    const Polyline route = router.travel_to({layer, from, Point::Zero(), extruder_index_t(0), 0., false}, to);
    const AvoidCrossingPerimeters::Boundary &boundary = router.internal_boundary();
    const double radius = from.distance_to(to) / 2;
    CHECK(boundary.grid.closest_point_signed_distance(from, radius).contour_idx == size_t(-1));
    REQUIRE(boundary.grid.closest_point_signed_distance(to, radius).contour_idx != size_t(-1));
    const Polygon *square = nullptr;
    for (const Polygon &ring : boundary.boundaries)
        if (ring.contains(to)) square = &ring;
    REQUIRE(square != nullptr);
    const Point projection(from.x(), get_extents(*square).max.y());
    std::cout << "endpoint projection core:";
    for (const Point &point : route.points) std::cout << " (" << point.x() << "," << point.y() << ")";
    std::cout << " expected=(" << projection.x() << "," << projection.y() << ")" << std::endl;
    REQUIRE(route.size() > 2);
    CHECK(route.points[1] == projection);
}

TEST_CASE("Avoid-crossing resolves a sliced modifier boundary with an interior probe",
          "[plugins][layer-extrusion-edit][travel][avoid-crossing-perimeters][avoid-crossing-parity]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"avoid_crossing_perimeters", "1"}, {"avoid_crossing_top", "0"},
        {"avoid_crossing_not_first_layer", "0"}, {"avoid_crossing_perimeters_max_detour", "0"},
        {"nozzle_diameter", "0.4"}, {"layer_height", "0.2"}, {"first_layer_height", "0.2"},
        {"perimeters", "2"}, {"fill_density", "20%"}, {"top_solid_layers", "1"}, {"bottom_solid_layers", "1"},
        {"skirts", "0"}, {"brim_width", "0"}});
    Model model;
    ModelObject *object = model.add_object();
    object->add_volume(ring_mesh(), ModelVolumeType::MODEL_PART, false);
    ModelVolume *modifier = object->add_volume(make_cube(10., 20., 0.6), ModelVolumeType::PARAMETER_MODIFIER, false);
    DynamicPrintConfig disabled;
    disabled.set_deserialize_strict({{"avoid_crossing_perimeters", "0"}});
    modifier->config.assign_config(disabled);
    object->add_instance();
    model.center_instances_around_point({100., 100.});
    object->ensure_on_bed();
    Print print;
    print.auto_assign_extruders(object);
    print.apply(model, config);
    print.set_status_silent();
    print.process();
    print.pipeline_progress().reset();
    Orchestrator &orchestrator = Orchestrator::instance();
    Steps::StepExtrusionOrdering::clear(print);
    Steps::StepExtrusionOrdering::prepare(print);
    Steps::StepExtrusionOrdering::run_step(orchestrator, print);
    PrintingPlan &plan = print.mutable_printing_plan();
    const LayerRegionIsland *native_region = nullptr;
    uint32_t catalog = PRINTING_OBJECT_INSTANCE_ID_NONE;
    for (const PrintingGroup &group : plan.groups)
        for (const PrintingLayerGroup &layer : group.layers)
            for (const PrintingToolGroup &tool : layer.tool_groups)
                for (const PrintingExtrusion &extrusion : tool.extrusions) {
                    const slic3r_api::LayerRegionIsland region(reinterpret_cast<const layer_region_island_handle *>(extrusion.region_island));
                    if (native_region == nullptr && region.valid() && region.region_count() > 1) {
                        native_region = extrusion.region_island;
                        catalog = extrusion.printing_catalog_id;
                    }
                }
    REQUIRE(native_region != nullptr);
    const slic3r_api::LayerRegionIsland region(reinterpret_cast<const layer_region_island_handle *>(native_region));
    coord_t boundary = 0, bottom = 0;
    for (uint32_t idx = 0; idx < region.region_count(); ++idx) {
        const slic3r_api::LayerRegion part = region.region(idx);
        if (part.print_region().config().bool_or_default("avoid_crossing_perimeters", true))
            continue;
        const ExPolygons &slices = *reinterpret_cast<const ExPolygons *>(part.slices().handle());
        const BoundingBox bounds = get_extents(slices);
        boundary = bounds.max.x();
        bottom = bounds.min.y();
    }
    const c_point shift = region.region(0).layer().object().instance_shift(0);
    const Point source(boundary + shift.x, bottom + scale_i(2.) + shift.y);
    const Point target(boundary + scale_i(8.) + shift.x, bottom + scale_i(18.) + shift.y);
    const slic3r_api::RegionSettingsPointResult at_boundary = slic3r_api::RegionSettings::lookup_at_point(
        region, {"avoid_crossing_perimeters"}, c_point{source.x() - shift.x, source.y() - shift.y});
    REQUIRE(at_boundary.status == slic3r_api::RegionSettingsPointStatus::Ambiguous);
    const slic3r_api::RegionSettingsPointResult inside = slic3r_api::RegionSettings::lookup_at_point(
        region, {"avoid_crossing_perimeters"}, c_point{source.x() - shift.x + SCALED_EPSILON, source.y() - shift.y});
    REQUIRE(inside.found());
    REQUIRE(inside.value.get_bool());

    // Controlled leaves use the real masks, with the source ending exactly on their shared edge.
    plan.groups.clear();
    plan.groups.emplace_back();
    plan.groups.back().printing_catalog_ids.push_back(catalog);
    plan.groups.back().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.back().layers.back();
    layer.print_z = scale_i(0.2);
    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool = layer.tool_groups.back();
    tool.extruder_id = 0;
    for (const Points &points : {Points{source + Point(scale_i(1.), 0), source},
                                Points{target, target + Point(scale_i(1.), 0)}}) {
        ArcPolyline line;
        for (const Point &point : points)
            line.append(point);
        PrintingExtrusion extrusion;
        ExtrusionPath path(line,
            ExtrusionAttributes(ExtrusionRole::Perimeter, ExtrusionFlow(0.2, 0.4f, 0.2f)), nullptr, true);
        extrusion.root = CanonicalExtrusion::acquire(path);
        extrusion.region_island = native_region;
        extrusion.printing_catalog_id = catalog;
        tool.extrusions.push_back(std::move(extrusion));
    }
    {
        Slic3r::Test::TransitionPipeline::ScopedActivePlugins active({
            "layer_extrusion_edit.transition_scope.default", "layer_extrusion_edit.entry_state.default",
            "layer_extrusion_edit.travel.avoid_crossing_perimeters"});
        Steps::StepLayerExtrusionEdition::run_step(orchestrator, print);
    }
    std::vector<Leaf> leaves;
    for (const PrintingExtrusion &extrusion : tool.extrusions)
        collect_leaves(*extrusion.root, extrusion, 0, layer, leaves);
    REQUIRE(leaves.size() == 3);
    size_t resolved = 0;
    REQUIRE(enabled_endpoint(leaves.front(), false, plan, &resolved));
    REQUIRE(resolved == 1);
    Routers routers;
    const Points expected = oracle_path(leaves.front(), leaves.back(), plan, print, routers);
    REQUIRE(expected.size() > 2);
    const ArcPolyline &actual = leaves[1].entity->polyline_ref();
    REQUIRE(actual.size() == expected.size());
    for (size_t idx = 0; idx < expected.size(); ++idx)
        CHECK(actual.get_point(idx) == expected[idx]);
    std::cout << "boundary: compared=1 deviated=1 resolved=" << resolved << std::endl;
}

TEST_CASE("Avoid-crossing plugin matches core routes on sliced scenes",
          "[plugins][layer-extrusion-edit][travel][avoid-crossing-perimeters][avoid-crossing-parity]")
{
    const Scene scenario = GENERATE(Scene::Hole, Scene::Islands, Scene::Top, Scene::Absolute,
        Scene::Percentage, Scene::FirstLayer, Scene::Instances, Scene::Tools, Scene::Support,
        Scene::Regional, Scene::Units);
    check_scene(scenario);
}

TEST_CASE("Top travel scene has stable ordering and surfaces published per island",
          "[plugins][layer-extrusion-edit][travel][avoid-crossing-perimeters][avoid-crossing-parity]")
{
    const TopRoutes off = top_oracle_routes(false);
    const TopRoutes on = top_oracle_routes(true);
    REQUIRE_FALSE(off.routes.empty());
    REQUIRE_FALSE(on.routes.empty());
    CHECK(off.island_tops > 0);
    CHECK(on.island_tops > 0);
    CHECK(off.region_tops == 0);
    CHECK(on.region_tops == 0);
    CHECK(off.ordered == on.ordered);
}

// The core router uses top surfaces published per island even when regional surfaces are empty.
TEST_CASE("Core travel router avoids top surfaces published per island",
          "[plugins][layer-extrusion-edit][travel][avoid-crossing-perimeters][avoid-crossing-parity]")
{
    const TopRoutes off = top_oracle_routes(false);
    const TopRoutes on = top_oracle_routes(true);
    const bool different = off.routes != on.routes;
    CHECK(different);
}

// Route across a sliced ring hole and measure the flow-dependent inset of the detour.
TEST_CASE("Core travel detour uses raw-sliced regional perimeter spacing",
          "[plugins][travel][avoid-crossing-parity][raw-slice-spacing]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"avoid_crossing_perimeters", "1"}, {"avoid_crossing_top", "0"},
        {"avoid_crossing_perimeters_max_detour", "0"}, {"nozzle_diameter", "0.4"},
        {"perimeter_extrusion_width", "0.8"}, {"layer_height", "0.2"}, {"first_layer_height", "0.2"},
        {"perimeters", "1"}, {"fill_density", "0%"}, {"thin_walls", "0"}, {"gap_fill_enabled", "0"},
        {"skirts", "0"}, {"brim_width", "0"}});
    Model model;
    Print print;
    Slic3r::Test::init_print({ring_mesh()}, print, model, config);
    print.process();
    const Layer &layer = print.object(0).layers().back();
    REQUIRE(layer.regions().size() == 1);
    const LayerRegion &region = *layer.regions().begin();
    REQUIRE(region.slices().empty());
    REQUIRE_FALSE(region.get_raw_slices().empty());
    const BoundingBox bbox = get_extents(layer.lslices());
    const Point from(bbox.min.x() + scale_i(2.), bbox.min.y() + scale_i(10.));
    const Point to(bbox.max.x() - scale_i(2.), from.y());
    AvoidCrossingPerimeters router;
    router.init_layer(layer);
    const Polyline route = router.travel_to({layer, from, Point::Zero(), 0, 0., false}, to);
    REQUIRE(route.size() > 2);
    // get_boundary insets the internal boundary by 1.5 perimeter spacings through inner_offset.
    const double expected = 1.5 * region.flow(frPerimeter).scaled_spacing();
    // Separate the roughly 1.14 mm flow inset from the 0.6 mm nozzle-based fallback.
    REQUIRE(expected > scale_i(1.));
    double inset = double(bbox.max.y() - bbox.min.y());
    for (size_t idx = 1; idx + 1 < route.size(); ++idx)
        inset = std::min(inset, double(std::min(route.points[idx].y() - bbox.min.y(),
                                              bbox.max.y() - route.points[idx].y())));
    CHECK(inset == Approx(expected).margin(scale_i(0.02)));
}

// A same-tool modifier shares generated surfaces but must retain its own top-avoidance setting.
// The internal boundary retains excluded tops, and its grid borrows their stored points.
TEST_CASE("Shared island tops obey regional top avoidance",
          "[plugins][travel][avoid-crossing-parity]")
{
    const bool avoid_top = GENERATE(false, true);
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"avoid_crossing_perimeters", "1"}, {"avoid_crossing_top", avoid_top ? "1" : "0"},
        {"nozzle_diameter", "0.4"}, {"layer_height", "0.2"}, {"first_layer_height", "0.2"},
        {"perimeters", "2"}, {"fill_density", "20%"}, {"top_solid_layers", "1"},
        {"bottom_solid_layers", "1"}, {"skirts", "0"}, {"brim_width", "0"}});
    Model model;
    ModelObject *object = model.add_object();
    object->add_volume(make_cube(20., 20., 0.6), ModelVolumeType::MODEL_PART, false);
    ModelVolume *modifier = object->add_volume(make_cube(10., 20., 0.6), ModelVolumeType::PARAMETER_MODIFIER, false);
    DynamicPrintConfig disabled;
    disabled.set_deserialize_strict({{"avoid_crossing_top", "0"}});
    modifier->config.assign_config(disabled);
    object->add_instance();
    object->ensure_on_bed();
    Print print;
    print.apply(model, config);
    print.set_status_silent();
    print.process();
    const Layer &layer = print.object(0).layers().back();
    const BoundingBox bbox = get_extents(layer.lslices());
    bool shared = false;
    ExPolygons enabled_tops, disabled_tops;
    for (const LayerSliceIsland &island : layer.islands()) {
        for (const LayerRegionIsland &region_island : island.regions_islands()) {
            shared = shared || region_island.regions().size() > 1;
            for (const Surface &surface : region_island.fill_surfaces()) {
                if (!surface.has_pos_top())
                    continue;
                for (const LayerRegion *region : region_island.regions()) {
                    ExPolygons part = intersection_ex(ExPolygons{surface.expolygon}, region->get_raw_slices());
                    const bool avoided = region->region().config().avoid_crossing_top;
                    append(!avoid_top || avoided ? enabled_tops : disabled_tops,
                           offset_ex(part, -scale_i(0.5)));
                }
            }
        }
    }
    if (avoid_top)
        REQUIRE(shared);
    REQUIRE_FALSE(enabled_tops.empty());
    if (avoid_top)
        REQUIRE_FALSE(disabled_tops.empty());
    // Keep endpoints in the perimeter band, so only the chord crosses the fill interior.
    size_t enabled_chords = 0, disabled_chords = 0;
    for (int eighth : {1, 3, 5, 7}) {
        const coord_t x = bbox.min.x() + (bbox.max.x() - bbox.min.x()) * eighth / 8;
        const Point from(x, bbox.min.y() + scale_i(0.2));
        const Point to(x, bbox.max.y() - scale_i(0.2));
        AvoidCrossingPerimeters router;
        router.init_layer(layer);
        if (!avoid_top) {
            // Endpoints stay in the perimeter band; the chord crosses the excluded top interior.
            const Polyline interior_chord(from, to);
            REQUIRE_FALSE(intersection_pl(interior_chord, enabled_tops).empty());
            const std::vector<const Layer *> printed_layers{&layer};
            router.prepare_crossing_test({layer, printed_layers, 0, 0, scale_i(0.2), []() {}});
            CHECK(router.can_cross_perimeter(interior_chord, false));
        }
        const Polyline route = router.travel_to({layer, from, Point::Zero(), 0, 0., false}, to);
        REQUIRE(route.size() >= 2);
        if (avoid_top) {
            REQUIRE_FALSE(router.internal_boundary().to_avoid.empty());
            REQUIRE_FALSE(router.internal_boundary().to_avoid_grid.contours().empty());
            CHECK(router.internal_boundary().to_avoid_grid.contours().front().begin() ==
                  router.internal_boundary().to_avoid.front().contour.points.data());
        }
        if (!intersection_pl(Polyline(from, to), enabled_tops).empty()) {
            ++enabled_chords;
            CHECK(intersection_pl(route, enabled_tops).empty() == avoid_top);
        } else if (avoid_top) {
            ++disabled_chords;
            REQUIRE_FALSE(intersection_pl(Polyline(from, to), disabled_tops).empty());
            CHECK_FALSE(intersection_pl(route, disabled_tops).empty());
        }
    }
    CHECK(enabled_chords > 0);
    if (avoid_top)
        CHECK(disabled_chords > 0);
}

void check_scene(const Scene scenario)
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    CAPTURE(static_cast<int>(scenario));
    Points selected_route;
    double discriminant = 0.;
    const int passes = scenario == Scene::Units ? 3 : scenario == Scene::Tools ? 2 : 1;
    std::vector<std::string> ordered_runs;
    for (int pass = 0; pass < passes; ++pass) {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        const std::string selector = Steps::plugin_exclusive_group_option_key(
            Orchestrator::instance(), STEP_LAYER_EXTRUSION_EDIT, "layer_extrusion_edit.travel");
        config.set_deserialize_strict(selector, "layer_extrusion_edit.travel.avoid_crossing_perimeters");
        config.set_deserialize_strict({
            {"avoid_crossing_perimeters", "1"}, {"avoid_crossing_not_first_layer", "0"}, {"avoid_crossing_top", "0"},
            {"avoid_crossing_perimeters_max_detour", "0"}, {"nozzle_diameter", "0.4"},
            {"layer_height", "0.2"}, {"first_layer_height", "0.2"},
            {"perimeters", "2"}, {"fill_density", "20%"}, {"top_solid_layers", "1"},
            {"bottom_solid_layers", "1"}, {"skirts", "0"}, {"brim_width", "0"}
        });
        if (scenario == Scene::Islands)
            config.set_deserialize_strict({{"avoid_travel_island", "1"}, {"avoid_travel_island_weight", "5"}});
        if (scenario == Scene::Top)
            config.set_deserialize_strict({{"avoid_crossing_top", "1"}});
        if (scenario == Scene::Absolute)
            config.set_deserialize_strict({{"avoid_crossing_perimeters_max_detour", "0.001"}});
        if (scenario == Scene::Percentage)
            config.set_deserialize_strict({{"avoid_crossing_perimeters_max_detour", "0.001%"}});
        if (scenario == Scene::FirstLayer || scenario == Scene::Support)
            config.set_deserialize_strict({{"avoid_crossing_not_first_layer", "1"}});
        if (scenario == Scene::Units && pass > 0) {
            std::ostringstream value;
            value.precision(17);
            value << discriminant;
            if (pass == 2)
                value << '%';
            config.set_deserialize_strict("avoid_crossing_perimeters_max_detour", value.str());
        }
        if (scenario == Scene::Tools)
            config.set_deserialize_strict({{"nozzle_diameter", "0.4,0.4"},
                {"min_layer_height", "0.05,0.05"}, {"max_layer_height", "0.3,0.3"}});
        if (scenario == Scene::Support)
            config.set_deserialize_strict({{"support_material", "1"}, {"support_material_style", "grid"},
                {"support_material_extruder", "1"}, {"support_material_interface_extruder", "1"},
                {"support_material_contact_distance", "0.2"}, {"dont_support_bridges", "0"}});

        Model model;
        Print print;
        TriangleMesh mesh = ring_mesh();
        if (scenario == Scene::Islands) {
            TriangleMesh island = make_cube(3., 3., 0.6);
            island.translate(Vec3f(8.f, 8.f, 0.f));
            mesh.merge(island);
        }
        if (scenario == Scene::Support) {
            // The roof extends beyond the narrow post and creates the support demand.
            TriangleMesh post = make_cube(4., 8., 2.);
            TriangleMesh roof = make_cube(12., 8., 0.6);
            roof.translate(Vec3f(0.f, 0.f, 2.f));
            post.merge(roof);
            mesh = std::move(post);
        }
        if (scenario == Scene::Instances || scenario == Scene::Tools || scenario == Scene::Regional) {
            ModelObject *object = model.add_object();
            object->add_volume(mesh, ModelVolumeType::MODEL_PART, false);
            object->add_instance();
            if (scenario == Scene::Instances) {
                ModelInstance *instance = object->add_instance();
                instance->set_offset(Vec3d(30., 0., 0.));
            } else {
                TriangleMesh part = make_cube(10., 20., 0.6);
                ModelVolume *volume = object->add_volume(part, ModelVolumeType::PARAMETER_MODIFIER, false);
                DynamicPrintConfig regional;
                if (scenario == Scene::Regional)
                    regional.set_deserialize_strict({{"avoid_crossing_perimeters", "0"}});
                else
                    // Shared perimeters use one tool; only the region's infill changes tools.
                    regional.set_deserialize_strict({{"perimeter_extruder", "1"}, {"infill_extruder", "2"},
                                                     {"solid_infill_extruder", "2"}});
                volume->config.assign_config(regional);
            }
            model.center_instances_around_point({100., 100.});
            object->ensure_on_bed();
            print.auto_assign_extruders(object);
            print.apply(model, config);
            print.set_status_silent();
        } else
            Slic3r::Test::init_print({mesh}, print, model, config);
        print.process();
        // The focused editor selection is a separate run from the slicing progress plan.
        print.pipeline_progress().reset();
        Orchestrator &orchestrator = Orchestrator::instance();
        Steps::StepExtrusionOrdering::clear(print);
        Steps::StepExtrusionOrdering::prepare(print);
        Steps::StepExtrusionOrdering::run_step(orchestrator, print);
        // Record the ordered source stream before any editor inserts travel geometry.
        std::ostringstream ordered;
        std::vector<Leaf> before;
        for (const PrintingGroup &group : print.printing_plan()->groups)
            for (const PrintingLayerGroup &layer : group.layers)
                for (const PrintingToolGroup &tool : layer.tool_groups)
                    for (const PrintingExtrusion &extrusion : tool.extrusions)
                        collect_leaves(*extrusion.root, extrusion, tool.extruder_id, layer, before);
        for (const Leaf &leaf : before)
            ordered << leaf.layer->print_z << ' ' << leaf.tool << ' ' << leaf.owner->printing_catalog_id << ' '
                    << leaf.entity->first_point().transpose() << ' ' << leaf.entity->last_point().transpose() << '\n';
        ordered_runs.push_back(ordered.str());
        {
            Slic3r::Test::TransitionPipeline::ScopedActivePlugins active({
                "layer_extrusion_edit.transition_scope.default", "layer_extrusion_edit.entry_state.default",
                "layer_extrusion_edit.travel.avoid_crossing_perimeters"});
            REQUIRE(Steps::selected_active_plugin_from_group(orchestrator, STEP_LAYER_EXTRUSION_EDIT,
                "layer_extrusion_edit.travel", &print.config())->get_id() ==
                "layer_extrusion_edit.travel.avoid_crossing_perimeters");
            Steps::StepLayerExtrusionEdition::run_step(orchestrator, print);
        }
        REQUIRE_FALSE(orchestrator.is_plugin_cancelled());
        REQUIRE(print.printing_plan() != nullptr);
        const PrintingPlan &plan = *print.printing_plan();
        std::vector<Leaf> leaves;
        for (const PrintingGroup &group : plan.groups)
            for (const PrintingLayerGroup &layer : group.layers)
                for (const PrintingToolGroup &tool : layer.tool_groups)
                    for (const PrintingExtrusion &extrusion : tool.extrusions)
                        collect_leaves(*extrusion.root, extrusion, tool.extruder_id, layer, leaves);

        size_t compared = 0, deviated = 0, first_layer_travels = 0;
        size_t external = 0, second_tool = 0, support = 0, first_layer_support_travels = 0;
        size_t disabled_detours = 0, resolved = 0, selected = 0;
        Routers routers;
        for (size_t idx = 0; idx < leaves.size(); ++idx) {
            const ExtrusionAttributes *attributes = leaves[idx].entity->get_property<ExtrusionAttributes>();
            if (attributes == nullptr || attributes->extrusion_role() != ExtrusionRole::Travel ||
                !leaves[idx].generated_travel)
                continue;
            REQUIRE(idx > 0);
            REQUIRE(idx + 1 < leaves.size());
            const Points expected = oracle_path(leaves[idx - 1], leaves[idx + 1], plan, print, routers);
            if (scenario == Scene::Regional) {
                const bool source_enabled = enabled_endpoint(leaves[idx - 1], false, plan, &resolved);
                const bool target_enabled = enabled_endpoint(leaves[idx + 1], true, plan, &resolved);
                if ((!source_enabled || !target_enabled) && leaves[idx - 1].layer == leaves[idx + 1].layer) {
                    Routers forced;
                    if (oracle_path(leaves[idx - 1], leaves[idx + 1], plan, print, forced, true).size() > 2) {
                        CHECK(expected.size() == 2);
                        ++disabled_detours;
                    }
                }
            }
            if (scenario == Scene::Units) {
                if (pass == 0 && selected_route.empty() && expected.size() > 2) {
                    const double direct = (expected.back() - expected.front()).cast<double>().norm() * SCALING_FACTOR;
                    const double extra = Polyline(expected).length() * SCALING_FACTOR - direct;
                    if (extra > 0.01 && direct < 90. && direct > 1.) {
                        // This midpoint lies between the extra mm and its percent equivalent: accepted in mm, rejected in % for direct lengths below 100 mm.
                        discriminant = (extra + 100. * extra / direct) / 2.;
                        selected_route = expected;
                        std::cout << "unit discriminator: direct_mm=" << direct << " extra_mm=" << extra
                                  << " value=" << discriminant << std::endl;
                    }
                } else if (pass > 0 && expected.front() == selected_route.front() && expected.back() == selected_route.back()) {
                    CHECK((expected.size() > 2) == (pass == 1));
                    ++selected;
                }
            }
            const ArcPolyline &actual = leaves[idx].entity->polyline_ref();
            INFO("leaf=" << idx << " source=" << leaves[idx - 1].entity->last_point()
                 << " target=" << leaves[idx + 1].entity->first_point()
                 << " actual from=" << actual.front() << " actual to=" << actual.back());
            REQUIRE(actual.size() == expected.size());
            for (size_t point_idx = 0; point_idx < expected.size(); ++point_idx)
                CHECK(actual.get_point(point_idx) == expected[point_idx]);
            ++compared;
            deviated += actual.size() > 2;
            external += leaves[idx - 1].layer == leaves[idx + 1].layer &&
                leaves[idx - 1].owner->printing_catalog_id != leaves[idx + 1].owner->printing_catalog_id;
            second_tool += leaves[idx + 1].tool == 1;
            const ExtrusionAttributes *target_attributes = leaves[idx + 1].entity->get_property<ExtrusionAttributes>();
            support += target_attributes != nullptr && target_attributes->extrusion_role().is_support();
            // This scene enables avoid_crossing_not_first_layer, so support at the first object layer Z stays direct.
            if (scenario == Scene::Support && target_attributes != nullptr &&
                target_attributes->extrusion_role().is_support() &&
                leaves[idx + 1].layer->print_z == print.object(0).layers().front().scaled_print_z()) {
                CHECK(actual.size() == 2);
                ++first_layer_support_travels;
            }
            const LayerRegionIsland *region = leaves[idx + 1].owner->region_island;
            const slic3r_api::LayerRegionIsland region_view(reinterpret_cast<const layer_region_island_handle *>(region));
            if (scenario == Scene::FirstLayer && region != nullptr &&
                region_view.region(0).layer().print_z() == region_view.region(0).layer().object().layer(0).print_z()) {
                CHECK(actual.size() == 2);
                ++first_layer_travels;
            }
        }
        std::cout << "avoid-crossing parity: scene=" << static_cast<int>(scenario) << " pass=" << pass << ": compared=" << compared
                  << " deviated=" << deviated << " disabled_detours=" << disabled_detours << " resolved=" << resolved
                  << " first_layer_support=" << first_layer_support_travels << std::endl;
        REQUIRE(compared > 0);
        if (scenario == Scene::Hole || scenario == Scene::Islands || scenario == Scene::Top)
            CHECK(deviated > 0);
        if (scenario == Scene::Absolute || scenario == Scene::Percentage)
            CHECK(deviated == 0);
        if (scenario == Scene::FirstLayer)
            CHECK(first_layer_travels > 0);
        if (scenario == Scene::Instances) {
            CHECK(external > 0);
            CHECK(deviated > 0);
        }
        if (scenario == Scene::Tools) {
            CHECK(second_tool > 0);
            CHECK(deviated > 0);
        }
        if (scenario == Scene::Support) {
            CHECK(support > 0);
            CHECK(first_layer_support_travels > 0);
            CHECK(deviated > 0);
        }
        if (scenario == Scene::Regional) {
            CHECK(disabled_detours > 0);
            CHECK(deviated > 0);
        }
        if (scenario == Scene::Units) {
            REQUIRE_FALSE(selected_route.empty());
            if (pass > 0)
                CHECK(selected > 0);
            if (pass > 0)
                CHECK(ordered_runs[0] == ordered_runs[pass]);
        }
    }
    if (scenario == Scene::Tools) {
        std::cout << "multi-tool ordered sequences equal=" << (ordered_runs[0] == ordered_runs[1]) << std::endl;
        CHECK(ordered_runs[0] == ordered_runs[1]);
    }
}

TopRoutes top_oracle_routes(const bool avoid_top)
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"avoid_crossing_perimeters", "1"}, {"avoid_crossing_not_first_layer", "0"},
        {"avoid_crossing_top", avoid_top ? "1" : "0"},
        {"avoid_crossing_perimeters_max_detour", "0"}, {"nozzle_diameter", "0.4"},
        {"layer_height", "0.2"}, {"first_layer_height", "0.2"},
        {"perimeters", "2"}, {"fill_density", "20%"}, {"top_solid_layers", "1"},
        {"bottom_solid_layers", "1"}, {"skirts", "0"}, {"brim_width", "0"}
    });
    Model model;
    Print print;
    Slic3r::Test::init_print({ring_mesh()}, print, model, config);
    print.process();
    TopRoutes result;
    for (const Layer &layer : print.object(0).layers()) {
        for (const LayerRegion &region : layer.regions())
            for (const Surface &surface : region.fill_surfaces())
                result.region_tops += surface.has_pos_top();
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands())
                for (const Surface &surface : region.fill_surfaces())
                    result.island_tops += surface.has_pos_top();
    }
    print.pipeline_progress().reset();
    Orchestrator &orchestrator = Orchestrator::instance();
    Steps::StepExtrusionOrdering::clear(print);
    Steps::StepExtrusionOrdering::prepare(print);
    Steps::StepExtrusionOrdering::run_step(orchestrator, print);
    const PrintingPlan &plan = *print.printing_plan();
    std::vector<Leaf> leaves;
    for (const PrintingGroup &group : plan.groups)
        for (const PrintingLayerGroup &layer : group.layers)
            for (const PrintingToolGroup &tool : layer.tool_groups)
                for (const PrintingExtrusion &extrusion : tool.extrusions)
                    collect_leaves(*extrusion.root, extrusion, tool.extruder_id, layer, leaves);
    std::ostringstream ordered;
    for (const Leaf &leaf : leaves)
        ordered << leaf.layer->print_z << ' ' << leaf.tool << ' ' << leaf.owner->printing_catalog_id << ' '
                << leaf.entity->first_point().transpose() << ' ' << leaf.entity->last_point().transpose() << '\n';
    result.ordered = ordered.str();
    Routers routers;
    for (size_t idx = 1; idx < leaves.size(); ++idx) {
        const Point source = leaves[idx - 1].entity->last_point();
        const Point target = leaves[idx].entity->first_point();
        if ((target - source).cast<double>().norm() >= SCALED_EPSILON)
            result.routes.push_back(oracle_path(leaves[idx - 1], leaves[idx], plan, print, routers));
    }
    return result;
}

void collect_leaves(const ExtrusionEntity &entity, const PrintingExtrusion &owner,
                    const extruder_index_t tool, const PrintingLayerGroup &layer, std::vector<Leaf> &out)
{
    if (entity.child_count() != 0) {
        const size_t begin = out.size();
        for (size_t idx = 0; idx < entity.child_count(); ++idx)
            collect_leaves(entity.child(idx), owner, tool, layer, out);
        const slic3r_api::PluginPropertyKey<slic3r_api::PrintingExtrusionScopeProperty> key =
            slic3r_api::printing_extrusion_scope_property_key(
                reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
        const slic3r_api::MutableExtrusionEntity view(reinterpret_cast<extrusion_entity_handle *>(
            const_cast<ExtrusionEntity *>(&entity)));
        if (view.get(key) != nullptr) {
            const slic3r_api::LayerExtrusionEdit::ExtrusionScope::OrderedExtrusionScope scope(view, key);
            for (size_t idx = begin; idx < out.size(); ++idx)
                if (scope.travel().valid() &&
                    reinterpret_cast<const extrusion_entity_handle *>(out[idx].entity) == scope.travel().handle())
                    out[idx].generated_travel = true;
        }
    } else if (entity.has_polyline() && entity.polyline_ref().size() > 1)
        out.push_back(Leaf{&entity, &owner, tool, &layer});
}

bool enabled_endpoint(const Leaf &leaf, const bool first, const PrintingPlan &plan, size_t *resolved)
{
    if (leaf.owner->region_island == nullptr)
        return false;
    const slic3r_api::LayerRegionIsland region(reinterpret_cast<const layer_region_island_handle *>(leaf.owner->region_island));
    if (region.region_count() == 0)
        return false;
    const slic3r_api::ConstPrintingPlan view(reinterpret_cast<const printing_plan_handle *>(&plan));
    const c_point shift = region.region(0).layer().object().instance_shift(
        view.object_instance(leaf.owner->printing_catalog_id).object_instance_idx());
    const Point point = first ? leaf.entity->first_point() : leaf.entity->last_point();
    slic3r_api::RegionSettingsPointResult result = slic3r_api::RegionSettings::lookup_at_point(
        region, {"avoid_crossing_perimeters"}, c_point{point.x() - shift.x, point.y() - shift.y});
    if (result.found())
        return result.value.get_bool();
    const bool ambiguous = result.status == slic3r_api::RegionSettingsPointStatus::Ambiguous;
    const slic3r_api::ExtrusionEntity entity(reinterpret_cast<const extrusion_entity_handle *>(leaf.entity));
    const double distance = std::min(double(SCALED_EPSILON), entity.length() / 2.);
    if (distance > 0.) {
        const c_point probe = first ? entity.point_from_begin(distance) : entity.point_from_end(distance);
        result = slic3r_api::RegionSettings::lookup_at_point(region, {"avoid_crossing_perimeters"},
            c_point{probe.x - shift.x, probe.y - shift.y});
    }
    if (resolved != nullptr && ambiguous && result.found())
        ++*resolved;
    return result.found() && result.value.get_bool();
}

Points oracle_path(const Leaf &source, const Leaf &target, const PrintingPlan &plan,
                   const Print &print, Routers &routers, const bool ignore_activation)
{
    const Points straight{source.entity->last_point(), target.entity->first_point()};
    // At a layer boundary the connector carries only the entry position, not the preceding region.
    if (source.layer != target.layer) {
        routers.clear();
        return straight;
    }
    if (!ignore_activation && (!enabled_endpoint(source, false, plan) || !enabled_endpoint(target, true, plan)))
        return straight;
    const slic3r_api::LayerRegionIsland region(reinterpret_cast<const layer_region_island_handle *>(target.owner->region_island));
    const slic3r_api::Layer layer_view = region.region(0).layer();
    const Layer &layer = *reinterpret_cast<const Layer *>(layer_view.handle());
    // When the exemption is enabled, use the first object layer Z for both object and support targets.
    if (print.config().avoid_crossing_not_first_layer &&
        layer.scaled_print_z() == layer.object()->layers().front().scaled_print_z())
        return straight;
    const slic3r_api::ConstPrintingPlan view(reinterpret_cast<const printing_plan_handle *>(&plan));
    const c_point api_shift = layer_view.object().instance_shift(
        view.object_instance(target.owner->printing_catalog_id).object_instance_idx());
    const Point shift(api_shift.x, api_shift.y);
    const Point from = straight.front() - shift, to = straight.back() - shift;
    std::unique_ptr<AvoidCrossingPerimeters> &entry = routers[{&layer, target.tool}];
    if (!entry) {
        entry = std::make_unique<AvoidCrossingPerimeters>();
        entry->init_layer(layer);
    }
    AvoidCrossingPerimeters &router = *entry;
    const std::vector<const Layer *> printed_layers{&layer};
    router.prepare_crossing_test({layer, printed_layers, target.owner->printing_catalog_id,
        target.tool, std::max<coord_t>(1, scale_i(print.config().nozzle_diameter.get_at(target.tool) * 0.5)), []() {}});
    if (!router.can_cross_perimeter(Polyline(from, to), true))
        return straight;
    const slic3r_api::LayerRegionIsland source_region(reinterpret_cast<const layer_region_island_handle *>(source.owner->region_island));
    router.use_external_mp(source_region.region(0).layer().object().handle() != layer_view.object().handle() ||
                           source.owner->printing_catalog_id != target.owner->printing_catalog_id);
    const ConfigOptionFloatOrPercent &detour = print.config().avoid_crossing_perimeters_max_detour;
    Polyline routed = router.travel_to({layer, from, shift, target.tool, detour.value, detour.percent}, to);
    if (routed.size() < 2)
        return straight;
    for (Point &point : routed.points)
        point += shift;
    routed.points.front() = straight.front();
    routed.points.back() = straight.back();
    return routed.points;
}

TriangleMesh ring_mesh()
{
    TriangleMesh mesh = make_cube(20., 4., 0.6);
    TriangleMesh top = make_cube(20., 4., 0.6);
    top.translate(Vec3f(0.f, 16.f, 0.f));
    mesh.merge(top);
    TriangleMesh left = make_cube(4., 12., 0.6);
    left.translate(Vec3f(0.f, 4.f, 0.f));
    mesh.merge(left);
    TriangleMesh right = make_cube(4., 12., 0.6);
    right.translate(Vec3f(16.f, 4.f, 0.f));
    mesh.merge(right);
    return mesh;
}

} // namespace
