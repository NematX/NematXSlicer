/*
Core travel fallback tests
=========================

These cases route fixed endpoint pairs through sliced frame layers to check
direct fallback and endpoint preservation. Print owns the layers borrowed by
the router throughout each query. Each case owns its model, print and router.

    cases -> prepare_scene -> sliced frame layers -> core travel_to
*/

#include <catch2/catch.hpp>
#include "plugin_test_helpers.hpp"
#include "test_data.hpp"
#include "libslic3r/GCode/AvoidCrossingPerimeters.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"

namespace {
using namespace Slic3r;

// Slice the same frame, instances and neighbouring object used by router parity.
static void prepare_scene(Print &print, Model &model);
// Build a shallow frame with one disconnected central island.
static TriangleMesh frame();

TEST_CASE("Core travel router keeps an outside-box travel direct", "[plugins][avoid-crossing-router]")
{
    Print print;
    Model model;
    prepare_scene(print, model);
    const Layer &layer = print.object(0).layers().front();
    AvoidCrossingPerimeters router;
    router.init_layer(layer);
    const Point from(2000000, 28000000), to(2000000, 29990000);
    const Polyline result = router.travel_to({layer, from, Point(0, 0), extruder_index_t(0), 0., false}, to);
    REQUIRE((result.points == Points{from, to}));
}


static void prepare_scene(Print &print, Model &model)
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"layer_height", "0.2"}, {"first_layer_height", "0.2"},
        {"perimeters", "2"}, {"fill_density", "20%"}, {"top_solid_layers", "1"}, {"bottom_solid_layers", "1"},
        {"skirts", "0"}, {"brim_width", "0"}, {"gap_fill_enabled", "0"}, {"thin_walls", "0"},
        {"thin_perimeters", "0"}, {"avoid_crossing_top", "1"}, {"avoid_travel_island", "1"},
        {"avoid_travel_island_weight", "0.4"}});
    ModelObject *object = model.add_object();
    object->add_volume(frame(), ModelVolumeType::MODEL_PART, false);
    object->add_instance();
    object->add_instance()->set_offset(Vec3d(50., 0., 0.));
    object->ensure_on_bed();
    ModelObject *second = model.add_object();
    second->add_volume(frame(), ModelVolumeType::MODEL_PART, false);
    second->add_instance()->set_offset(Vec3d(0., 50., 0.));
    second->ensure_on_bed();
    print.apply(model, config);
    print.set_status_silent();
    print.process();
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
}
