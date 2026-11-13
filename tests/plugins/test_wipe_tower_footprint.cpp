///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/* Core-only tower tests compare configuration envelopes across unsliced Model
   and Print representations. Origins use instance translations, not centered
   slicing coordinates or instance rotations. Clearance includes the owner's
   tower and profile loading ignores the removed per-color legacy option. */
#include <catch2/catch.hpp>
#include <tbb/global_control.h>
#include "plugin_test_helpers.hpp"
#include "test_data.hpp"
#include "libslic3r/Api/internal/PrintAccess.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Geometry/ConvexHull.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/WipeTowerFootprint.hpp"
#include "libslic3r/Steps/StepLayerHeightGeneration.hpp"
#include "libslic3r/Steps/StepSlicing.hpp"

namespace {
using namespace Slic3r;
// Return complete two-tool configuration with a compact generic tower.
DynamicPrintConfig tower_config();
// Create two printable cubes at deterministic model-instance translations.
void prepare_tower_print(Print &print, Model &model, const DynamicPrintConfig &config);
// Compare the sliced-only cached hull and the export envelope that also includes tower corners.
void check_first_layer_hull(Print &print, const std::vector<WipeTowerFootprint> &towers);

// Initialize the shared runtime and return owned configuration; dimensions are millimeters and config/allocation failures propagate.
DynamicPrintConfig tower_config()
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_extruders(2);
    config.set_deserialize_strict({{"nozzle_diameter", "0.4,0.6"}, {"filament_diameter", "1.75,1.75"},
        {"wipe_tower", "1"}, {"wipe_tower_width", "20"}, {"wipe_tower_brim_width", "2"},
        {"wipe_tower_x", "10"}, {"wipe_tower_y", "15"}, {"wipe_tower_rotation_angle", "0"},
        {"wipe_tower_object_x", "50"}, {"wipe_tower_object_y", "0"}, {"extruder_clearance_radius", "10"},
        {"step_layer_height_plugin", "standard_layer_height_generator"}, {"step_slicing_plugin", "slice_volume"}});
    return config;
}

// Populate caller-owned Model/Print through normal apply, using translations independent of mesh centering; configuration and allocation failures propagate.
void prepare_tower_print(Print &print, Model &model, const DynamicPrintConfig &config)
{
    Test::init_print({Test::TestMesh::cube_20x20x20, Test::TestMesh::cube_20x20x20}, print, model, config);
    model.objects()[0].instances[0]->set_offset(Vec3d(30., 50., 0.));
    model.objects()[1].instances[0]->set_offset(Vec3d(150., 50., 0.));
    print.apply(model, config);
}

// Generate caller-owned layers/slices, assert the cached source hull and its tower-enriched export copy, and retain borrowed source ownership; stage/allocation failures propagate.
void check_first_layer_hull(Print &print, const std::vector<WipeTowerFootprint> &towers)
{
    Steps::StepLayerHeightGeneration::run_step(Orchestrator::instance(), print);
    Steps::StepSlicing::run_step(Orchestrator::instance(), print);
    Points expected;
    for (const PrintObject &object : print.objects()) {
        REQUIRE(object.layer_count() > 0);
        for (const PrintInstance &instance : object.instances())
            for (const ExPolygon &slice : object.layer(0).lslices()) {
                Polygon polygon = slice.contour;
                polygon.translate(instance.shift);
                append(expected, polygon.points);
            }
    }
    ApiInternal::PrintAccess::rebuild_first_layer_convex_hull_after_skirt_brim(print);
    CHECK(print.first_layer_convex_hull().points == Geometry::convex_hull(expected).points);
    for (const WipeTowerFootprint &tower : towers)
        append(expected, tower.polygon.points);
    CHECK(print.first_layer_convex_hull_with_wipe_towers().points == Geometry::convex_hull(expected).points);
}
}

TEST_CASE("Generic tower footprint includes brim then rotates and translates", "[plugins][wipe-tower][footprint]")
{
    const Slic3r::Polygon square = Slic3r::wipe_tower_footprint({0., 0.}, 20., 2., 0.);
    CHECK(square.area() == Approx(Slic3r::scale_d(24.) * Slic3r::scale_d(24.)));
    REQUIRE(square.points.size() == 4);
    CHECK(square.points[0] == Slic3r::Point::new_scale(-2., -2.));
    CHECK(square.points[2] == Slic3r::Point::new_scale(22., 22.));
    const Slic3r::Polygon posed = Slic3r::wipe_tower_footprint({40., 60.}, 20., 2., 90.);
    CHECK(posed.points[0] == Slic3r::Point::new_scale(42., 58.));
    CHECK(posed.points[2] == Slic3r::Point::new_scale(18., 82.));
    CHECK(posed.area() == Approx(square.area()));
}

// Apply owned model/config variants and compare borrowed tower origins, eligibility and both first-layer hulls; slicing/configuration failures propagate and all assertions remain visible.
TEST_CASE("Tower enumeration shares model origins and object settings", "[plugins][wipe-tower][footprint]")
{
    const tbb::global_control serial(tbb::global_control::max_allowed_parallelism, 1);
    using namespace Slic3r;
    DynamicPrintConfig config = tower_config();
    Print print;
    Model model;
    SECTION("global") {
        prepare_tower_print(print, model, config);
        const std::vector<WipeTowerFootprint> towers = wipe_tower_footprints(print);
        REQUIRE(towers.size() == 1);
        CHECK(towers[0].object == nullptr);
        CHECK(towers[0].origin == Vec2d(10., 15.));
        check_first_layer_hull(print, towers);
        CHECK(print.has_global_wipe_tower());
        const Polygon cached = print.first_layer_convex_hull();
        config.set_deserialize_strict("wipe_tower_x", "210");
        print.apply(model, config);
        Points expected = cached.points;
        for (const WipeTowerFootprint &tower : wipe_tower_footprints(print))
            append(expected, tower.polygon.points);
        CHECK(print.first_layer_convex_hull().points == cached.points);
        CHECK(print.first_layer_convex_hull_with_wipe_towers().points == Geometry::convex_hull(expected).points);
    }
    SECTION("sequential instances") {
        config.set_deserialize_strict("print_plan_builder", "ordering.plan_builder.by_object");
        prepare_tower_print(print, model, config);
        model.objects()[0].instances[0]->set_rotation(Vec3d(0., 0., Geometry::deg2rad(90.)));
        model.objects()[0].config.set_deserialize_strict("wipe_tower_rotation_angle", "30");
        model.objects()[0].config.set_deserialize_strict("wipe_tower_brim_width", "200%");
        model.objects()[0].config.set_deserialize_strict("wipe_tower_extruder", "2");
        print.apply(model, config);
        const std::vector<WipeTowerFootprint> towers = wipe_tower_footprints(print);
        const std::vector<WipeTowerFootprint> model_towers = wipe_tower_footprints(model, config);
        REQUIRE(towers.size() == 2);
        REQUIRE(model_towers.size() == towers.size());
        CHECK_FALSE(print.has_global_wipe_tower());
        CHECK(print.has_wipe_tower());
        CHECK(towers[0].origin == Vec2d(80., 50.));
        CHECK(towers[0].rotation == 30.);
        CHECK(towers[0].brim_width == Approx(1.2));
        for (size_t i = 0; i < towers.size(); ++i) {
            CHECK(towers[i].origin == model_towers[i].origin);
            CHECK(towers[i].polygon.points == model_towers[i].polygon.points);
            CHECK(towers[i].instance != nullptr);
        }
        Points corners;
        for (const WipeTowerFootprint &tower : towers)
            append(corners, tower.polygon.points);
        REQUIRE(corners.size() == 8);
        check_first_layer_hull(print, towers);
    }
    SECTION("own tower without global enable") {
        config.set_deserialize_strict("wipe_tower", "0");
        prepare_tower_print(print, model, config);
        model.objects()[0].config.set_deserialize_strict("wipe_tower_per_object", "1");
        ModelInstance *copy = model.objects()[0].add_instance();
        copy->set_offset(Vec3d(30., 100., 0.));
        print.apply(model, config);
        const std::vector<WipeTowerFootprint> towers = wipe_tower_footprints(print);
        REQUIRE(towers.size() == 2);
        CHECK(towers[0].origin == Vec2d(80., 50.));
        CHECK(towers[1].origin == Vec2d(80., 100.));
        CHECK(print.has_wipe_tower());
        CHECK_FALSE(print.has_global_wipe_tower());
    }
    SECTION("global plus explicit instance towers") {
        config.set_deserialize_strict("wipe_tower_per_object", "1");
        prepare_tower_print(print, model, config);
        CHECK(wipe_tower_footprints(print).size() == 3);
        CHECK(wipe_tower_footprints(model, config).size() == 3);
    }
    SECTION("single extruder") {
        config.set_deserialize_strict("nozzle_diameter", "0.4");
        config.set_deserialize_strict("wipe_tower_per_object", "1");
        prepare_tower_print(print, model, config);
        CHECK(wipe_tower_footprints(print).empty());
        CHECK(wipe_tower_footprints(model, config).empty());
        CHECK_FALSE(print.has_wipe_tower());
    }
    SECTION("spiral vase") {
        config.set_deserialize_strict("wipe_tower_per_object", "1");
        prepare_tower_print(print, model, config);
        // Eligibility needs only the configured flag, independent of vase path generation.
        const_cast<PrintConfig &>(print.config()).spiral_vase.value = true;
        config.set_deserialize_strict("spiral_vase", "1");
        CHECK(wipe_tower_footprints(print).empty());
        CHECK(wipe_tower_footprints(model, config).empty());
        CHECK_FALSE(print.has_wipe_tower());
    }
    SECTION("parallel objects keep the global tower without implicit instance towers") {
        config.set_deserialize_strict("parallel_objects_step", "5");
        config.set_deserialize_strict("print_plan_builder", "ordering.plan_builder.by_object");
        config.set_deserialize_strict("parallel_islands", "0");
        prepare_tower_print(print, model, config);
        CHECK(print.has_global_wipe_tower());
        CHECK(print.has_wipe_tower());
        const std::vector<WipeTowerFootprint> towers = wipe_tower_footprints(print);
        const std::vector<WipeTowerFootprint> model_towers = wipe_tower_footprints(model, config);
        REQUIRE(towers.size() == 1);
        REQUIRE(model_towers.size() == 1);
        CHECK(towers[0].instance == nullptr);
        CHECK(towers[0].polygon.points == model_towers[0].polygon.points);
        config.set_deserialize_strict("parallel_islands", "1");
        print.apply(model, config);
        CHECK(print.has_global_wipe_tower());
        CHECK(wipe_tower_footprints(print).size() == 1);
        CHECK(wipe_tower_footprints(model, config).size() == 1);
    }
}

TEST_CASE("Sequential clearance includes each instance tower with its owner", "[plugins][wipe-tower][footprint]")
{
    const tbb::global_control serial(tbb::global_control::max_allowed_parallelism, 1);
    using namespace Slic3r;
    DynamicPrintConfig config = tower_config();
    config.set_deserialize_strict("print_plan_builder", "ordering.plan_builder.by_object");
    Print print;
    Model model;
    prepare_tower_print(print, model, config);
    CHECK(Print::sequential_print_horizontal_clearance_valid(print));
    model.objects()[1].instances[0]->set_offset(Vec3d(85., 50., 0.));
    print.apply(model, config);
    Polygons collisions;
    CHECK_FALSE(Print::sequential_print_horizontal_clearance_valid(print, &collisions));
    CHECK_FALSE(collisions.empty());
    config.set_deserialize_strict("wipe_tower", "0");
    print.apply(model, config);
    CHECK(Print::sequential_print_horizontal_clearance_valid(print));
}

// Apply caller-owned three-tool settings to two cubes and compare global/sequential tool lists; the default contour tool belongs only to the global legacy tower and configuration failures remain visible.
TEST_CASE("Legacy extruder list adds only the global tower contour tool", "[plugins][wipe-tower][config]")
{
    using namespace Slic3r;
    DynamicPrintConfig config = tower_config();
    config.set_num_extruders(3);
    config.set_deserialize_strict({{"nozzle_diameter", "0.4,0.4,0.4"}, {"wipe_tower_extruder", "2"},
        {"perimeter_extruder", "1"}, {"infill_extruder", "3"}, {"solid_infill_extruder", "3"}});
    Print print;
    Model model;
    prepare_tower_print(print, model, config);
    CHECK(print.extruders() == std::set<extruder_index_t>{0, 1, 2});
    config.set_deserialize_strict("print_plan_builder", "ordering.plan_builder.by_object");
    print.apply(model, config);
    CHECK(print.has_wipe_tower());
    CHECK_FALSE(print.has_global_wipe_tower());
    CHECK(print.extruders() == std::set<extruder_index_t>{0, 2});
}

// Inspect registered defaults/routing and normalize/load owned partial configs, asserting exact values and ignored legacy keys; unexpected load or normalization failures fail the test.
TEST_CASE("Tower options expose defaults, consumer invalidation, and legacy loading", "[plugins][wipe-tower][config]")
{
    using namespace Slic3r;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const PrintObjectConfig defaults;
    CHECK_FALSE(defaults.wipe_tower_per_object.value);
    CHECK(defaults.wipe_tower_object_x.value == 50.);
    CHECK(defaults.wipe_tower_object_y.value == 0.);
    CHECK(defaults.wipe_tower_extruder_toolchange.value);
    for (const char *key : {"wipe_tower_speed", "wipe_tower_wipe_starting_speed", "wipe_tower_extrusion_width",
         "wipe_tower_no_sparse_layers", "wipe_tower_rest_in_middle", "wipe_tower_cone_angle", "wipe_tower_extra_spacing",
         "wipe_tower_bridging", "wipe_tower_extruder_toolchange"}) {
        INFO(key);
        REQUIRE(PrintConfigDef::instance().get(key) != nullptr);
        CHECK(PrintConfigDef::instance().get(key)->invalidates_step == STEP_WIPETOWER);
    }
    // The wipe-into ordering plugin reads the tower presence at STEP_ORDERING, so these two resume
    // there; the earlier step still replays the tower step and everything after it.
    for (const char *key : {"wipe_tower", "wipe_tower_per_object"}) {
        INFO(key);
        CHECK(PrintConfigDef::instance().get(key)->invalidates_step == STEP_ORDERING);
    }
    for (const char *key : {"wipe_tower_x", "wipe_tower_y", "wipe_tower_width", "wipe_tower_rotation_angle",
         "wipe_tower_brim_width", "wipe_tower_extruder", "wipe_tower_object_x", "wipe_tower_object_y"}) {
        INFO(key);
        CHECK(PrintConfigDef::instance().get(key)->invalidates_step == STEP_WIPETOWER);
    }
    CHECK(PrintConfigDef::instance().get("wipe_into_infill")->invalidates_step == STEP_SURFACE_GENERATION);
    DynamicPrintConfig object_override;
    object_override.set_deserialize_strict("wipe_tower_extruder", "2");
    REQUIRE_NOTHROW(object_override.normalize_fdm());
    CHECK(object_override.opt_int("wipe_tower_extruder") == 2);
    object_override.set_deserialize_strict("nozzle_diameter", "0.4");
    object_override.normalize_fdm();
    CHECK(object_override.opt_int("wipe_tower_extruder") == 0);
    DynamicPrintConfig loaded;
    REQUIRE_NOTHROW(loaded.load_from_ini_string("wipe_tower = 1\nwipe_tower_per_color_wipe = 25\n",
        ForwardCompatibilitySubstitutionRule::Disable));
    CHECK(loaded.opt_bool("wipe_tower"));
    CHECK_FALSE(loaded.has("wipe_tower_per_color_wipe"));
}
