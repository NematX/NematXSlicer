#include <catch2/catch.hpp>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>
#include "plugin_test_helpers.hpp"
#include "avoid_crossing_test_helpers.hpp"
#include "test_data.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/AuxiliaryLayerHelpers.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/GCode/AvoidCrossingPerimeters.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/AvoidCrossing/TravelRouter.hpp"

/*
These tests compare every ordered point returned by the plugin TravelRouter
with the core router on sliced frames, islands, regional tops, two tools and
narrow-neck dumbbells. Deterministic endpoint pairs include near-boundary points;
internal and external routing are checked with disabled, absolute and percentage
detour limits. Agreement checks the port against core behaviour, not independent
shortest-path quality or a guarantee of no crossings.

The core borrows its Layer. The plugin borrows a copied Input and requests copied
external geometry through a callback into the public print view. Print, Input
and storage outlive each comparison; each router owns its lazy boundary caches.
The support scene manually attaches an auxiliary subject from a sliced frame.
It checks agreement with the core's support rule, which may keep travels straight
or enter object insets rather than route around objects.

Focused cases check direct fallback outside the boundary, arrival-side epsilon
placement, full bridge-edge search, coalescing near-corner projections, and
absolute detour thresholds at adjacent floating-point values. A chord across tops
checks the fallback shortcut comparison against retained top areas. A square and
distant island leave one endpoint without a nearby ring, exercising projection
onto consecutive edges of the other endpoint's ring in both travel directions.

    test cases: slice scenes and compare owned routes
    |-- compare_layer: sample endpoints with both boundary choices and budgets
    |   |-- core travel_to: route using the borrowed Layer
    |   `-- plugin travel_to: route using Input and the external-copy callback
    |-- check_scene: vary tops, tools, support, island weight and neck width
    |   `-- compare_layer: compare each applicable layer/tool pair
    |-- epsilon/threshold case: compare near projections and measured budgets
    |-- endpoint-projection case: compare unmatched start and end edge scans
    `-- top-shortcut case: compare fallback routing across retained tops

Successful queries report aggregate counts. Catch retains endpoint context for
ordered-coordinate failures; counters establish the explicitly asserted branches.
*/
namespace {
using namespace Slic3r;
namespace port = slic3r_api::LayerExtrusionEdit::AvoidCrossing;
namespace work = slic3r_api::PluginCommon::work;

// Build a shallow rectangular frame with a disconnected island.
static TriangleMesh frame();
using Test::Plugins::dumbbell;
// Slice one independent setting or geometry variation and compare all applicable layers and tools.
static void check_scene(int scene);
// Compare deterministic ordered routes on one layer, retaining both router caches across queries.
static size_t compare_layer(const Print &print, const Layer &layer, port::LayerIdentity identity, port::RouterCounts &total,
                            int tool = 0, bool regressions = false);

TEST_CASE("Plugin travel router matches exact sliced core routes", "[plugins][avoid-crossing-router]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"layer_height", "0.2"}, {"first_layer_height", "0.2"},
        {"perimeters", "2"}, {"fill_density", "20%"}, {"top_solid_layers", "1"}, {"bottom_solid_layers", "1"},
        {"skirts", "0"}, {"brim_width", "0"}, {"gap_fill_enabled", "0"}, {"thin_walls", "0"},
        {"thin_perimeters", "0"}, {"avoid_crossing_top", "1"}, {"avoid_travel_island", "1"},
        {"avoid_travel_island_weight", "0.4"}});
    Model model;
    ModelObject *object = model.add_object();
    object->add_volume(frame(), ModelVolumeType::MODEL_PART, false);
    object->add_instance();
    object->add_instance()->set_offset(Vec3d(50., 0., 0.));
    object->ensure_on_bed();
    ModelObject *second = model.add_object();
    second->add_volume(frame(), ModelVolumeType::MODEL_PART, false);
    second->add_instance()->set_offset(Vec3d(0., 50., 0.));
    second->ensure_on_bed();
    Print print;
    print.apply(model, config); print.set_status_silent(); print.process();
    port::RouterCounts counts;
    for (uint32_t o = 0; o < print.objects().size(); ++o) {
        size_t routes = 0;
        uint32_t l = 0;
        for (const Layer &layer : print.object(o).layers()) routes += compare_layer(print, layer, {o, l++, false}, counts, 0, true);
        std::cout << "router object=" << o << " routes=" << routes << std::endl;
        REQUIRE(routes > 0);
    }
    std::cout << "router deviated=" << counts.deviated << " jumps=" << counts.island_jumps
              << " fallback=" << counts.boundary_fallback << " to_avoid=" << counts.to_avoid
               << " outside=" << counts.outside_bbox << " rejected=" << counts.detour_rejected
               << " arrival_epsilon=" << counts.arrival_epsilon << " brute_force=" << counts.brute_force << std::endl;
    CHECK(counts.deviated > 0);
    CHECK(counts.island_jumps > 0);
    CHECK(counts.boundary_fallback > 0);
    CHECK(counts.outside_bbox > 0);
    CHECK(counts.detour_rejected > 0);
    CHECK(counts.arrival_epsilon > 0);
    CHECK(counts.brute_force > 0);
}

TEST_CASE("Plugin travel router sliced scene matrix", "[plugins][avoid-crossing-router]")
{
    SECTION("regional tops") { check_scene(0); }
    SECTION("two tools") { check_scene(1); }
    SECTION("support") { check_scene(2); }
    SECTION("island weight five") { check_scene(3); }
    SECTION("island avoidance disabled") { check_scene(4); }
    SECTION("dumbbells") { check_scene(5); }
}

TEST_CASE("Plugin travel router preserves epsilon merging and absolute detour thresholds", "[plugins][avoid-crossing-router]")
{
    const bool threshold = GENERATE(false, true);
    CAPTURE(threshold);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"layer_height", "0.2"}, {"first_layer_height", "0.2"},
        {"perimeters", "2"}, {"fill_density", "20%"}, {"top_solid_layers", "1"}, {"bottom_solid_layers", "1"},
        {"skirts", "0"}, {"brim_width", "0"}, {"avoid_crossing_top", "0"}, {"avoid_travel_island", "0"}});
    Model model;
    ModelObject *object = model.add_object();
    object->add_volume(threshold ? frame() : make_cube(20., 20., 0.6), ModelVolumeType::MODEL_PART, false);
    object->add_instance(); object->ensure_on_bed();
    Print print;
    print.apply(model, config); print.set_status_silent(); print.process();
    const Layer &layer = print.object(0).layers().front();
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    const slic3r_api::Print view(reinterpret_cast<const print_handle *>(&print));
    const port::LayerIdentity identity{0, 0, false};
    const port::Input input = port::build_input(handle, view, identity);
    port::TravelRouter actual;
    actual.init_layer(handle, input, [&view, identity]() { return port::build_external_input(view, identity); });
    AvoidCrossingPerimeters expected;
    expected.init_layer(layer);
    size_t routes = 0, discriminators = 0;
    if (!threshold) {
        const BoundingBox box = get_extents(layer.lslices());
        for (coord_t offset : {128, 256, 1000, 2000}) {
            for (coord_t delta : {1024, 2048, 4096}) {
                const Point from = box.min + Point(offset, offset), to = from + Point(delta, 0);
                CAPTURE(offset, delta);
                const Polyline reference = expected.travel_to({layer, from, Point(0, 0), extruder_index_t(0), 0., false}, to);
                const work::Polyline result = actual.travel_to({work::Point(from.x(), from.y()), work::Point(0, 0), 0, 0., false}, work::Point(to.x(), to.y()));
                REQUIRE(result.size() == reference.size());
                for (size_t i = 0; i < result.size(); ++i) {
                    REQUIRE(result.points[i].x() == reference.points[i].x());
                    REQUIRE(result.points[i].y() == reference.points[i].y());
                }
                ++routes;
            }
        }
        CHECK(actual.counts().epsilon_merge > 0);
        std::cout << "router epsilon routes=" << routes << " merged=" << actual.counts().epsilon_merge << std::endl;
    } else {
        Points points;
        for (double x : {2., 14., 28.})
            for (double y : {2., 8., 14., 28.}) points.emplace_back(scale_i(x), scale_i(y));
        for (size_t a = 0; a < points.size(); ++a) {
            for (size_t b = a + 1; b < points.size(); ++b) {
                const Point from = points[a], to = points[b];
                const Polyline unlimited = expected.travel_to({layer, from, Point(0, 0), extruder_index_t(0), 0., false}, to);
                const double detour = unlimited.length() - Line(from, to).length();
                if (detour <= 0.) continue;
                double lower = detour * SCALING_FACTOR, upper = lower;
                std::vector<double> limits{lower};
                for (int i = 0; i < 4; ++i) {
                    lower = std::nextafter(lower, -std::numeric_limits<double>::infinity());
                    upper = std::nextafter(upper, std::numeric_limits<double>::infinity());
                    limits.push_back(lower); limits.push_back(upper);
                }
                for (double limit : limits) {
                    CAPTURE(a, b, from.x(), from.y(), to.x(), to.y(), detour, limit);
                    if ((detour > scale_d(limit)) != (detour > limit / SCALING_FACTOR)) ++discriminators;
                    const Polyline reference = expected.travel_to({layer, from, Point(0, 0), extruder_index_t(0), limit, false}, to);
                    const work::Polyline result = actual.travel_to({work::Point(from.x(), from.y()), work::Point(0, 0), 0, limit, false}, work::Point(to.x(), to.y()));
                    REQUIRE(result.size() == reference.size());
                    for (size_t i = 0; i < result.size(); ++i) {
                        CAPTURE(i);
                        REQUIRE(result.points[i].x() == reference.points[i].x());
                        REQUIRE(result.points[i].y() == reference.points[i].y());
                    }
                    ++routes;
                }
            }
        }
        REQUIRE(routes > 0);
        CHECK(discriminators > 0);
        std::cout << "router absolute thresholds routes=" << routes << " discriminating_limits=" << discriminators << std::endl;
    }
}

// The distant island expands the routing box without supplying a nearby ring for the outside endpoint.
TEST_CASE("Plugin travel router projects an unmatched endpoint onto the core boundary edge", "[plugins][avoid-crossing-router]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
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
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    const slic3r_api::Print view(reinterpret_cast<const print_handle *>(&print));
    const port::LayerIdentity identity{0, 0, false};
    const port::Input input = port::build_input(handle, view, identity);
    port::TravelRouter actual;
    actual.init_layer(handle, input, [&view, identity]() { return port::build_external_input(view, identity); });
    AvoidCrossingPerimeters expected;
    expected.init_layer(layer);
    // Exercise both unmatched-start and unmatched-end calls of the same boundary scan.
    for (bool reverse : {false, true}) {
        const Point start = reverse ? to : from, end = reverse ? from : to;
        const Polyline reference = expected.travel_to({layer, start, Point::Zero(), extruder_index_t(0), 0., false}, end);
        const work::Polyline result = actual.travel_to({work::Point(start.x(), start.y()), work::Point(0, 0), 0, 0., false}, work::Point(end.x(), end.y()));
        CAPTURE(reverse);
        REQUIRE(reference.size() > 2);
        REQUIRE(result.size() == reference.size());
        for (size_t i = 0; i < result.size(); ++i) {
            REQUIRE(result.points[i].x() == reference.points[i].x());
            REQUIRE(result.points[i].y() == reference.points[i].y());
        }
        std::cout << "endpoint projection router: reverse=" << reverse << " points=" << result.size() << std::endl;
    }
}

TEST_CASE("Plugin travel router compares fallback shortcuts across top surfaces", "[plugins][avoid-crossing-router]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"layer_height", "0.2"}, {"first_layer_height", "0.2"},
        {"perimeters", "2"}, {"fill_density", "20%"}, {"top_solid_layers", "1"}, {"bottom_solid_layers", "1"},
        {"skirts", "0"}, {"brim_width", "0"}, {"avoid_crossing_top", "1"}, {"avoid_travel_island", "0"}});
    Model model;
    ModelObject *object = model.add_object();
    object->add_volume(make_cube(20., 20., 0.6), ModelVolumeType::MODEL_PART, false);
    object->add_instance(); object->ensure_on_bed();
    Print print;
    print.apply(model, config); print.set_status_silent(); print.process();
    const Layer &layer = print.object(0).layers().back();
    const uint32_t layer_index = uint32_t(print.object(0).layers().size() - 1);
    const BoundingBox box = get_extents(layer.lslices());
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    const slic3r_api::Print view(reinterpret_cast<const print_handle *>(&print));
    const port::LayerIdentity identity{0, layer_index, false};
    const port::Input input = port::build_input(handle, view, identity);
    port::TravelRouter actual;
    actual.init_layer(handle, input, [&view, identity]() { return port::build_external_input(view, identity); });
    AvoidCrossingPerimeters expected;
    expected.init_layer(layer);
    size_t routes = 0;
    for (int side = 0; side < 4; ++side) {
        for (double offset : {0.3, 0.6, 0.8, 1.2}) {
            for (int position : {2, 5, 8}) {
                const coord_t x = box.min.x() + (box.max.x() - box.min.x()) * position / 10;
                const coord_t y = box.min.y() + (box.max.y() - box.min.y()) * position / 10;
                const Point from = side == 0 ? Point(box.min.x() - scale_i(offset), y) :
                                   side == 1 ? Point(box.max.x() + scale_i(offset), y) :
                                   side == 2 ? Point(x, box.min.y() - scale_i(offset)) : Point(x, box.max.y() + scale_i(offset));
                // The opposite top endpoint stays close enough to a boundary for both half-distance grid searches to succeed.
                const Point to = side == 0 ? Point(box.max.x() - scale_i(2.), y) :
                                 side == 1 ? Point(box.min.x() + scale_i(2.), y) :
                                 side == 2 ? Point(x, box.max.y() - scale_i(2.)) : Point(x, box.min.y() + scale_i(2.));
                for (int limit : {0, 1, 2}) {
                    CAPTURE(side, offset, position, limit);
                    const double maximum = limit == 0 ? 0. : limit == 1 ? 0.1 : 5.;
                    const Polyline reference = expected.travel_to({layer, from, Point(0, 0), extruder_index_t(0), maximum, limit == 2}, to);
                    const work::Polyline result = actual.travel_to({work::Point(from.x(), from.y()), work::Point(0, 0), 0, maximum, limit == 2}, work::Point(to.x(), to.y()));
                    REQUIRE(result.size() == reference.size());
                    for (size_t i = 0; i < result.size(); ++i) {
                        REQUIRE(result.points[i].x() == reference.points[i].x());
                        REQUIRE(result.points[i].y() == reference.points[i].y());
                    }
                    ++routes;
                }
            }
        }
    }
    REQUIRE_FALSE(expected.internal_boundary().to_avoid.empty());
    CHECK(actual.counts().to_avoid > 0);
    std::cout << "router targeted tops routes=" << routes << " to_avoid=" << actual.counts().to_avoid
              << " fallback=" << actual.counts().boundary_fallback << " rejected=" << actual.counts().detour_rejected << std::endl;
}

static void check_scene(int scene)
{
    CAPTURE(scene);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"layer_height", "0.2"}, {"first_layer_height", "0.2"},
        {"perimeters", "2"}, {"fill_density", "20%"}, {"top_solid_layers", "1"}, {"bottom_solid_layers", "1"},
        {"skirts", "0"}, {"brim_width", "0"}, {"gap_fill_enabled", "0"}, {"thin_walls", "0"},
        {"thin_perimeters", "0"}, {"avoid_crossing_top", "1"},
        {"avoid_travel_island", scene == 4 ? "0" : "1"}, {"avoid_travel_island_weight", scene == 3 ? "5" : "0.4"}});
    if (scene == 1) config.set_deserialize_strict({{"nozzle_diameter", "0.4,0.6"},
        {"min_layer_height", "0.05,0.05"}, {"max_layer_height", "0.3,0.3"}});
    Model model;
    ModelObject *object = model.add_object();
    object->add_volume(scene == 0 ? make_cube(20., 20., 0.6) : scene == 5 ? dumbbell(0.6) : frame(), ModelVolumeType::MODEL_PART, false);
    if (scene == 0 || scene == 1) {
        ModelVolume *modifier = object->add_volume(make_cube(10., 20., 0.6), ModelVolumeType::PARAMETER_MODIFIER, false);
        DynamicPrintConfig regional;
        if (scene == 0) regional.set_deserialize_strict({{"avoid_crossing_top", "0"}});
        else regional.set_deserialize_strict({{"infill_extruder", "2"}, {"solid_infill_extruder", "2"}});
        modifier->config.assign_config(regional);
    }
    object->add_instance(); object->ensure_on_bed();
    if (scene == 5) {
        for (double width : {1., 1.5, 2.5}) {
            ModelObject *next = model.add_object();
            next->add_volume(dumbbell(width), ModelVolumeType::MODEL_PART, false);
            next->add_instance()->set_offset(Vec3d(0., 30. * model.objects().size(), 0.)); next->ensure_on_bed();
        }
    }
    Print print;
    print.apply(model, config); print.set_status_silent(); print.process();
    port::RouterCounts counts;
    size_t routes = 0;
    if (scene == 2) {
        PluginStorage storage;
        storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
        const slic3r_api::Print view(reinterpret_cast<const print_handle *>(&print));
        const Layer &lower = print.object(0).layers().front();
        const coord_t z = view.object(0).layer(0).print_z();
        const slic3r_api::AuxiliaryLayerBuildResult support = slic3r_api::build_auxiliary_layer_regions_from_subject(
            handle, view, view.object(0), slic3r_api::ExPolygonCollection(reinterpret_cast<const expolygon_collection_handle *>(&lower.lslices())),
            scale_i(0.2), z, z - scale_i(0.1));
        REQUIRE(support.created);
        support.layer.mutable_properties().get_or_add(slic3r_api::LayerSupportProperty::key);
        routes += compare_layer(print, *reinterpret_cast<const Layer *>(support.layer.handle()), {0, 0, true}, counts);
    }
    for (uint32_t o = 0; o < print.objects().size(); ++o) {
        uint32_t l = 0;
        for (const Layer &layer : print.object(o).layers()) {
            for (int tool = 0; tool < (scene == 1 ? 2 : 1); ++tool)
                routes += compare_layer(print, layer, {o, l, false}, counts, tool);
            ++l;
        }
    }
    std::cout << "router scene=" << scene << " routes=" << routes << " deviated=" << counts.deviated
              << " jumps=" << counts.island_jumps << " fallback=" << counts.boundary_fallback << " to_avoid=" << counts.to_avoid
              << " outside=" << counts.outside_bbox << " rejected=" << counts.detour_rejected
              << " arrival_epsilon=" << counts.arrival_epsilon << " brute_force=" << counts.brute_force << std::endl;
    REQUIRE(routes > 0);
}

static TriangleMesh frame()
{
    TriangleMesh mesh = make_cube(30., 4., 0.6);
    TriangleMesh top = make_cube(30., 4., 0.6); top.translate(0., 26., 0.); mesh.merge(top);
    TriangleMesh left = make_cube(4., 22., 0.6); left.translate(0., 4., 0.); mesh.merge(left);
    TriangleMesh right = make_cube(4., 22., 0.6); right.translate(26., 4., 0.); mesh.merge(right);
    TriangleMesh island = make_cube(6., 6., 0.6); island.translate(11., 11., 0.); mesh.merge(island);
    return mesh;
}

static size_t compare_layer(const Print &print, const Layer &layer, port::LayerIdentity identity, port::RouterCounts &total,
                            int tool, bool regressions)
{
    CAPTURE(identity.object, identity.layer);
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    const slic3r_api::Print view(reinterpret_cast<const print_handle *>(&print));
    const port::Input input = port::build_input(handle, view, identity);
    port::TravelRouter actual;
    actual.init_layer(handle, input, [&view, identity]() { return port::build_external_input(view, identity); });
    AvoidCrossingPerimeters expected;
    expected.init_layer(layer);
    size_t routes = 0;
    if (regressions && identity.object == 0 && identity.layer == 0) {
        // A segment wholly outside the boundary must reach the direct-path fallback before the size assertion.
        const Point from(2000000, 28000000), to(2000000, 29990000);
        const Polyline reference = expected.travel_to({layer, from, Point(0, 0), extruder_index_t(0), 0., false}, to);
        const work::Polyline result = actual.travel_to({work::Point(from.x(), from.y()), work::Point(0, 0), 0, 0., false}, work::Point(to.x(), to.y()));
        REQUIRE((reference.points == Points{from, to}));
        REQUIRE(result.size() == reference.size());
        for (size_t i = 0; i < result.size(); ++i) {
            CHECK(result.points[i].x() == reference.points[i].x());
            CHECK(result.points[i].y() == reference.points[i].y());
        }
        ++routes;
    }
    if (regressions && identity.object == 0 && identity.layer == 1) {
        // The arrival-side epsilon visitor must move the arrival endpoint, not the departure endpoint.
        const Point from(-1000000, 2000000), to(8000000, 8000000);
        const Polyline reference = expected.travel_to({layer, from, Point(0, 0), extruder_index_t(0), 0., false}, to);
        const work::Polyline result = actual.travel_to({work::Point(from.x(), from.y()), work::Point(0, 0), 0, 0., false}, work::Point(to.x(), to.y()));
        REQUIRE(result.size() == reference.size());
        for (size_t i = 0; i < result.size(); ++i) {
            CHECK(result.points[i].x() == reference.points[i].x());
            CHECK(result.points[i].y() == reference.points[i].y());
        }
        CHECK(actual.counts().arrival_epsilon > 0);
        ++routes;
    }
    if (regressions && identity.object == 0 && identity.layer == 2) {
        // A failed arrival-grid search must reach the full edge search without requiring an epsilon intersection.
        const Point from(2000000, 10000), to(14000000, 2000000);
        const Polyline reference = expected.travel_to({layer, from, Point(0, 0), extruder_index_t(0), 0., false}, to);
        const work::Polyline result = actual.travel_to({work::Point(from.x(), from.y()), work::Point(0, 0), 0, 0., false}, work::Point(to.x(), to.y()));
        REQUIRE(result.size() == reference.size());
        for (size_t i = 0; i < result.size(); ++i) {
            CHECK(result.points[i].x() == reference.points[i].x());
            CHECK(result.points[i].y() == reference.points[i].y());
        }
        CHECK(actual.counts().brute_force > 0);
        ++routes;
    }
    Points points;
    for (double x : {2., 14., 28., 4., 8., 16., 26., 0.01, 0.3, 29.7, 29.99, -1., 31.})
        for (double y : {0.01, 2., 8., 14., 28., 29.99}) points.emplace_back(scale_i(x), scale_i(y));
    for (bool external : {false, true}) {
        actual.use_external_mp(external); expected.use_external_mp(external);
        for (size_t i = 0; i < points.size(); ++i) {
            for (size_t step : {size_t(1), size_t(7), size_t(19), size_t(37)}) {
                const Point start = points[i], end = points[(i + step) % points.size()];
                for (int limit : {0, 1, 2}) {
                    CAPTURE(external, tool, identity.auxiliary, i, step, limit, start.x(), start.y(), end.x(), end.y());
                    const slic3r_api::Object object = view.object(identity.object);
                    const c_point shift = object.instance_shift(uint32_t(i % object.instance_count()));
                    const Point origin(shift.x, shift.y);
                    const double maximum = limit == 0 ? 0. : limit == 1 ? 0.1 : 5.;
                    const Polyline reference = expected.travel_to({layer, start, origin, extruder_index_t(tool), maximum, limit == 2}, end);
                    const work::Polyline result = actual.travel_to({work::Point(start.x(), start.y()), work::Point(origin.x(), origin.y()), tool, maximum, limit == 2}, work::Point(end.x(), end.y()));
                    REQUIRE(result.size() == reference.size());
                    for (size_t p = 0; p < result.size(); ++p) {
                        CAPTURE(p);
                        REQUIRE(result.points[p].x() == reference.points[p].x());
                        REQUIRE(result.points[p].y() == reference.points[p].y());
                    }
                    ++routes;
                }
            }
        }
    }
    const port::RouterCounts &counts = actual.counts();
    total.deviated += counts.deviated; total.island_jumps += counts.island_jumps;
    total.boundary_fallback += counts.boundary_fallback; total.to_avoid += counts.to_avoid;
    total.outside_bbox += counts.outside_bbox; total.detour_rejected += counts.detour_rejected;
    total.arrival_epsilon += counts.arrival_epsilon;
    total.brute_force += counts.brute_force;
    std::cout << "router layer object=" << identity.object << " layer=" << identity.layer << " auxiliary=" << identity.auxiliary
              << " tool=" << tool << " routes=" << routes << " deviated=" << counts.deviated << " jumps=" << counts.island_jumps
              << " fallback=" << counts.boundary_fallback << " to_avoid=" << counts.to_avoid << " outside=" << counts.outside_bbox
              << " rejected=" << counts.detour_rejected << " arrival_epsilon=" << counts.arrival_epsilon
              << " brute_force=" << counts.brute_force << std::endl;
    return routes;
}
}
