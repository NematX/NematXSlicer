/*
 * Direct tests for REST scene placement helpers.
 *
 * Small models exercise arrangement selection, bed reporting, GUI distance
 * defaults, and per-instance automatic orientation without HTTP transport.
 * Explicit By object gaps below the preferred clearance remain valid requests.
 */
#include <catch2/catch.hpp>

#include "placement.hpp"
#include "rest_models_test_helpers.hpp"
#include "scene_edit.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Config/FFFPrintConfig.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Orient.hpp"
#include "libslic3r/Print.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <string>

using namespace RestServerTest;
using namespace Slic3r;
using namespace Slic3r::Rest;

namespace {

// Compare all affine coefficients exactly for engine no-op assertions.
bool same_transform(const Transform3d &left, const Transform3d &right)
{
    for (int row = 0; row < 4; ++row)
        for (int column = 0; column < 4; ++column)
            if (left.matrix()(row, column) != right.matrix()(row, column))
                return false;
    return true;
}

// Start with the same complete FFF configuration used by REST-created sessions.
DynamicPrintConfig default_placement_config()
{
    return DynamicPrintConfig::full_print_config();
}

// Build a printable model with one instance for geometry and distance checks.
Model make_cube_scene(double x, double y, double z)
{
    Model scene;
    ModelObject *object = scene.add_object("placement", "placement.stl", make_cube(x, y, z));
    object->add_instance();
    object->ensure_on_bed();
    return scene;
}

// Return the axis-aligned XY bounds of an instance's printable solid volumes.
std::pair<Vec2d, Vec2d> instance_xy_bounds(const ModelInstance &instance)
{
    Vec2d minimum = Vec2d::Constant(std::numeric_limits<double>::max());
    Vec2d maximum = Vec2d::Constant(std::numeric_limits<double>::lowest());
    for (const ModelVolume *volume : instance.get_object()->volumes) {
        const Transform3d matrix = instance.get_matrix() * volume->get_matrix();
        for (const Vec3f &vertex : volume->mesh().its.vertices) {
            const Vec3d point = matrix * vertex.cast<double>();
            minimum = minimum.cwiseMin(point.head<2>());
            maximum = maximum.cwiseMax(point.head<2>());
        }
    }
    return {minimum, maximum};
}

// Measure the shortest axis-aligned XY gap between two arranged instances.
double xy_gap(const ModelInstance &left, const ModelInstance &right)
{
    const std::pair<Vec2d, Vec2d> left_bounds = instance_xy_bounds(left);
    const std::pair<Vec2d, Vec2d> right_bounds = instance_xy_bounds(right);
    const double x_gap = std::max({0., left_bounds.first.x() - right_bounds.second.x(),
        right_bounds.first.x() - left_bounds.second.x()});
    const double y_gap = std::max({0., left_bounds.first.y() - right_bounds.second.y(),
        right_bounds.first.y() - left_bounds.second.y()});
    return std::hypot(x_gap, y_gap);
}

// Rotate only the first instance in a copied model and keep the second as a fixed obstacle.
Model make_two_overlapping_instances()
{
    Model scene = make_cube_scene(20., 20., 20.);
    ModelObject *object = scene.object_ptrs().front();
    object->add_instance(*object->instances.front());
    object->instances.back()->set_offset(Vec3d::Zero());
    object->invalidate_bounding_box();
    return scene;
}

} // namespace

TEST_CASE("Arrange defaults follow the GUI distance rules", "[placement]")
{
    ensure_engine_initialized();
    CHECK(maximum_arrange_distance_mm() > 100.);
    DynamicPrintConfig config = default_placement_config();
    config.set_deserialize_strict({{"duplicate_distance", "6"}, {"print_plan_builder", "ordering.plan_builder.by_layer"},
        {"complete_objects_step", "0"}});
    CHECK(default_arrange_distance(config) == Approx(6.));

    config.set_deserialize_strict("duplicate_distance", "0");
    CHECK(default_arrange_distance(config) == Approx(6.));

    config.set_deserialize_strict({{"duplicate_distance", "6"}, {"print_plan_builder", "ordering.plan_builder.by_object"}});
    const double clearance = min_object_distance(&config, 0.);
    CHECK(clearance > 0.);
    CHECK(default_arrange_distance(config) == Approx(6. + clearance));
}

// Explicit distances smaller than the preferred radius are valid for both object
// modes; the engine still enforces the requested non-negative gap.
TEST_CASE("By object arrangement permits a gap below the preferred clearance", "[placement]")
{
    ensure_engine_initialized();
    for (const double step : {0., 0.5}) {
        DynamicPrintConfig config = default_placement_config();
        config.set_deserialize_strict({{"print_plan_builder", "ordering.plan_builder.by_object"},
            {"complete_objects_step", std::to_string(step)}, {"extruder_clearance_radius", "20"},
            {"duplicate_distance", "6"}, {"skirts", "0"}, {"brim_width", "0"}});
        CHECK(default_arrange_distance(config) >= 26.);
        Model scene = make_two_overlapping_instances();
        ArrangeRequest request;
        request.distance = 2.;
        const ArrangeOutcome outcome = arrange_instances(scene, config, {}, request);
        REQUIRE(outcome.instances.size() == 2);
        const ModelObject &object = scene.objects().front();
        CHECK(xy_gap(*object.instances[0], *object.instances[1]) == Approx(2.).margin(0.02));
        request.distance = -1.;
        CHECK_THROWS_AS(arrange_instances(scene, config, {}, request), SceneEditError);
    }
}

TEST_CASE("Arrange separates instances and keeps unselected ones fixed", "[placement]")
{
    ensure_engine_initialized();
    DynamicPrintConfig config = default_placement_config();
    config.set_deserialize_strict({{"duplicate_distance", "0"}, {"skirts", "0"}, {"brim_width", "0"}});
    Model scene = make_two_overlapping_instances();
    ModelObject *object = scene.object_ptrs().front();
    const Transform3d fixed_before = object->instances[1]->get_matrix();
    ArrangeRequest request;
    request.distance = 6.;
    const InstanceTargets targets{static_cast<std::uint64_t>(object->instances[0]->id().id)};

    const ArrangeOutcome outcome = arrange_instances(scene, config, targets, request);
    REQUIRE(outcome.instances.size() == 1);
    CHECK(outcome.changed);
    CHECK(outcome.instances.front().instance_id == targets.front());
    CHECK(outcome.instances.front().bed == 0);
    CHECK(same_transform(object->instances[1]->get_matrix(), fixed_before));
    CHECK(xy_gap(*object->instances[0], *object->instances[1]) >= 5.99);

    ArrangeRequest unrepresentable = request;
    unrepresentable.distance = maximum_arrange_distance_mm() * 2.;
    Model invalid_scene = make_two_overlapping_instances();
    CHECK_THROWS_AS(arrange_instances(invalid_scene, config, targets, unrepresentable), SceneEditError);

    Model empty;
    const ArrangeOutcome no_op = arrange_instances(empty, config, {}, request);
    CHECK_FALSE(no_op.changed);
    CHECK(no_op.instances.empty());
}

TEST_CASE("Arrange keeps the requested object distance and insets brims from the bed edge", "[placement]")
{
    ensure_engine_initialized();
    DynamicPrintConfig config = default_placement_config();
    config.set_deserialize_strict({{"bed_shape", "0x0,210x0,210x210,0x210"},
        {"duplicate_distance", "0"}, {"skirts", "0"}, {"brim_width", "5"},
        {"print_plan_builder", "ordering.plan_builder.by_layer"}, {"complete_objects_step", "0"},
        {"skirt_distance_from_brim", "0"}});
    Model scene;
    ModelObject *first_object = scene.add_object("first", "first.stl", make_cube(90., 90., 10.));
    ModelInstance *first_instance = first_object->add_instance();
    ModelObject *second_object = scene.add_object("second", "second.stl", make_cube(90., 90., 10.));
    ModelInstance *second_instance = second_object->add_instance();
    ArrangeRequest request;
    request.distance = 6.;

    const ArrangeOutcome outcome = arrange_instances(scene, config, {}, request);
    REQUIRE(outcome.instances.size() == 2);
    const std::pair<Vec2d, Vec2d> first_bounds = instance_xy_bounds(*first_instance);
    const std::pair<Vec2d, Vec2d> second_bounds = instance_xy_bounds(*second_instance);
    INFO("beds=" << outcome.instances[0].bed.value_or(-1) << "," << outcome.instances[1].bed.value_or(-1));
    INFO("first XY bounds=" << first_bounds.first.x() << "," << first_bounds.first.y() << " to "
        << first_bounds.second.x() << "," << first_bounds.second.y());
    INFO("second XY bounds=" << second_bounds.first.x() << "," << second_bounds.first.y() << " to "
        << second_bounds.second.x() << "," << second_bounds.second.y());
    INFO("case 1 XY gap=" << xy_gap(*first_instance, *second_instance));
    for (const ArrangedInstance &instance : outcome.instances) {
        REQUIRE(instance.bed.has_value());
        CHECK(*instance.bed == 0);
    }
    CHECK(xy_gap(*first_instance, *second_instance) >= 5.99);
    for (const ModelInstance *instance : {first_instance, second_instance}) {
        const std::pair<Vec2d, Vec2d> bounds = instance_xy_bounds(*instance);
        CHECK(bounds.first.x() >= 5. - 0.01);
        CHECK(bounds.first.y() >= 5. - 0.01);
        CHECK(bounds.second.x() <= 205. + 0.01);
        CHECK(bounds.second.y() <= 205. + 0.01);
    }

    // With the default setting, the engine measures the skirt from the brim and counts the brim twice.
    DynamicPrintConfig default_brim_distance_config = config;
    default_brim_distance_config.set_deserialize_strict("skirt_distance_from_brim", "1");
    Model default_brim_distance_scene;
    default_brim_distance_scene.add_object("first", "first.stl", make_cube(90., 90., 10.))->add_instance();
    default_brim_distance_scene.add_object("second", "second.stl", make_cube(90., 90., 10.))->add_instance();
    const ArrangeOutcome default_brim_distance_outcome = arrange_instances(
        default_brim_distance_scene, default_brim_distance_config, {}, request);
    REQUIRE(default_brim_distance_outcome.instances.size() == 2);
    INFO("default skirt_distance_from_brim beds=" << default_brim_distance_outcome.instances[0].bed.value_or(-1)
        << "," << default_brim_distance_outcome.instances[1].bed.value_or(-1));
    CHECK(std::all_of(default_brim_distance_outcome.instances.begin(), default_brim_distance_outcome.instances.end(),
        [](const ArrangedInstance &instance) { return instance.bed.has_value() && *instance.bed == 0; }));
    for (const ModelInstance *instance : default_brim_distance_scene.object_ptrs()[0]->instances) {
        const std::pair<Vec2d, Vec2d> bounds = instance_xy_bounds(*instance);
        CHECK(bounds.first.x() >= 10. - 0.01);
        CHECK(bounds.first.y() >= 10. - 0.01);
        CHECK(bounds.second.x() <= 200. + 0.01);
        CHECK(bounds.second.y() <= 200. + 0.01);
    }
    for (const ModelInstance *instance : default_brim_distance_scene.object_ptrs()[1]->instances) {
        const std::pair<Vec2d, Vec2d> bounds = instance_xy_bounds(*instance);
        CHECK(bounds.first.x() >= 10. - 0.01);
        CHECK(bounds.first.y() >= 10. - 0.01);
        CHECK(bounds.second.x() <= 200. + 0.01);
        CHECK(bounds.second.y() <= 200. + 0.01);
    }

    DynamicPrintConfig no_brim_config = config;
    no_brim_config.set_deserialize_strict("brim_width", "0");
    Model no_brim_scene;
    no_brim_scene.add_object("first", "first.stl", make_cube(90., 90., 10.))->add_instance();
    no_brim_scene.add_object("second", "second.stl", make_cube(90., 90., 10.))->add_instance();
    const ArrangeOutcome no_brim_outcome = arrange_instances(no_brim_scene, no_brim_config, {}, request);
    REQUIRE(no_brim_outcome.instances.size() == 2);
    CHECK(std::all_of(no_brim_outcome.instances.begin(), no_brim_outcome.instances.end(),
        [](const ArrangedInstance &instance) { return instance.bed.has_value() && *instance.bed == 0; }));
}

TEST_CASE("Arrange reports virtual beds and unplaced instances", "[placement]")
{
    ensure_engine_initialized();
    DynamicPrintConfig config = default_placement_config();
    config.set_deserialize_strict({{"duplicate_distance", "0"}, {"skirts", "0"}, {"brim_width", "0"}});
    Model scene;
    std::vector<std::uint64_t> regular_ids;
    for (int index = 0; index < 20; ++index) {
        const std::string name = "cube-" + std::to_string(index);
        ModelObject *object = scene.add_object(name.c_str(), name.c_str(), make_cube(80., 80., 20.));
        regular_ids.push_back(static_cast<std::uint64_t>(object->add_instance()->id().id));
    }
    ModelObject *oversized = scene.add_object("oversized", "oversized.stl", make_cube(1000., 1000., 20.));
    ModelInstance *oversized_instance = oversized->add_instance();
    oversized_instance->set_offset(Vec3d(1500., 1500., 0.));
    const std::uint64_t oversized_id = static_cast<std::uint64_t>(oversized_instance->id().id);
    const Transform3d oversized_before = oversized_instance->get_matrix();

    const ArrangeOutcome outcome = arrange_instances(scene, config, {}, ArrangeRequest{});
    REQUIRE(outcome.instances.size() == 21);
    CHECK(outcome.changed);
    CHECK(std::any_of(outcome.instances.begin(), outcome.instances.end(), [](const ArrangedInstance &instance) {
        return instance.bed.has_value() && *instance.bed > 0;
    }));
    const std::vector<ArrangedInstance>::const_iterator too_large = std::find_if(
        outcome.instances.begin(), outcome.instances.end(),
        [oversized_id](const ArrangedInstance &instance) { return instance.instance_id == oversized_id; });
    REQUIRE(too_large != outcome.instances.end());
    CHECK_FALSE(too_large->bed.has_value());
    CHECK(same_transform(oversized_instance->get_matrix(), oversized_before));
    CHECK(std::all_of(regular_ids.begin(), regular_ids.end(), [&outcome](std::uint64_t id) {
        return std::any_of(outcome.instances.begin(), outcome.instances.end(), [id](const ArrangedInstance &instance) {
            return instance.instance_id == id;
        });
    }));
}

TEST_CASE("Applied arrangement Print exposes skirt metadata", "[placement]")
{
    ensure_engine_initialized();
    DynamicPrintConfig config = default_placement_config();
    config.set_deserialize_strict({{"skirts", "1"}, {"skirt_height", "1"}, {"skirt_distance", "3"}});
    Model scene = make_cube_scene(20., 20., 20.);
    Orchestrator::RunContext run_context;
    Print print;
    Orchestrator::RunScope run_scope = Orchestrator::instance().enter_run(run_context, &print);
    for (ModelObject *object : scene.object_ptrs())
        print.auto_assign_extruders(object);
    print.apply(scene, config);

    CHECK(print.has_skirt());
    const std::set<extruder_index_t> extruders = print.object_extruders();
    REQUIRE_FALSE(extruders.empty());
    const Flow flow = print.skirt_flow(*extruders.begin());
    CHECK(flow.width() > 0.);
}

TEST_CASE("Orient lays only the oriented instance on the bed", "[placement]")
{
    ensure_engine_initialized();
    Model scene = make_cube_scene(10., 20., 40.);
    ModelObject *object = scene.object_ptrs().front();
    const double half_pi = std::acos(-1.) / 2.;
    object->instances.front()->set_rotation(Vec3d(half_pi, 0., 0.));
    object->ensure_on_bed();
    ModelInstance *sibling = object->add_instance(*object->instances.front());
    sibling->set_offset(sibling->get_offset() + Vec3d(60., 0., -3.));
    object->invalidate_bounding_box();
    REQUIRE(object->get_instance_min_z(1) == Approx(-3.).margin(1e-6));
    const Transform3d sibling_before = sibling->get_matrix();
    const std::uint64_t selected_id = static_cast<std::uint64_t>(object->instances.front()->id().id);

    const OrientOutcome outcome = orient_instances(scene, {selected_id}, OrientRequest{});

    REQUIRE(outcome.instances.size() == 1);
    CHECK(outcome.changed);
    CHECK(object->get_instance_min_z(0) == Approx(0.).margin(1e-6));
    CHECK(same_transform(sibling->get_matrix(), sibling_before));
    CHECK(object->get_instance_min_z(1) == Approx(-3.).margin(1e-6));
}

TEST_CASE("Orient rotates each instance from its own mesh", "[placement]")
{
    ensure_engine_initialized();
    Model single = make_cube_scene(10., 20., 40.);
    ModelObject *single_object = single.object_ptrs().front();
    const double half_pi = std::acos(-1.) / 2.;
    single_object->instances.front()->set_rotation(Vec3d(half_pi, 0., 0.));
    single_object->ensure_on_bed();
    Model together(single);
    ModelObject *together_object = together.object_ptrs().front();
    together_object->add_instance(*together_object->instances.front());
    together_object->instances.back()->set_rotation(Vec3d(0., half_pi, 0.));
    together_object->instances.back()->set_offset(Vec3d(60., 0., 0.));
    together_object->invalidate_bounding_box();

    const std::uint64_t selected_id = static_cast<std::uint64_t>(together_object->instances.front()->id().id);
    const OrientOutcome together_result = orient_instances(together, {selected_id}, OrientRequest{});
    const OrientOutcome single_result = orient_instances(single,
        {static_cast<std::uint64_t>(single_object->instances.front()->id().id)}, OrientRequest{});
    REQUIRE(together_result.instances.size() == 1);
    REQUIRE(single_result.instances.size() == 1);
    CHECK(together_result.changed);
    CHECK(together_result.instances.front().matrix.linear().isApprox(single_result.instances.front().matrix.linear(), 1e-8));
    CHECK(together_object->get_instance_min_z(0) == Approx(0.).margin(1e-5));

    Model volume_mode = make_cube_scene(10., 20., 40.);
    OrientRequest volume_request;
    volume_request.min_area = false;
    const OrientOutcome volume_result = orient_instances(volume_mode, {}, volume_request);
    REQUIRE(volume_result.instances.size() == 1);

    for (double angle : {45., 90.}) {
        Model empty;
        OrientRequest accepted;
        accepted.overhang_angle = angle;
        CHECK_NOTHROW(orient_instances(empty, {}, accepted));
    }
    for (double angle : {44.9, 90.1}) {
        Model empty;
        OrientRequest rejected;
        rejected.overhang_angle = angle;
        CHECK_THROWS_AS(orient_instances(empty, {}, rejected), SceneEditError);
    }

    Model lower_threshold;
    ModelObject *lower_object = lower_threshold.add_object(
        "steep-pyramid", "steep-pyramid.stl", make_pyramid(10.f, 30.f));
    lower_object->add_instance();
    Model upper_threshold(lower_threshold);
    OrientRequest lower_request;
    lower_request.overhang_angle = 45.;
    lower_request.min_area = false;
    OrientRequest upper_request;
    upper_request.overhang_angle = 90.;
    upper_request.min_area = false;
    orient_instances(lower_threshold, {}, lower_request);
    orient_instances(upper_threshold, {}, upper_request);
    CHECK_FALSE(same_transform(
        lower_threshold.object_ptrs().front()->instances.front()->get_matrix(),
        upper_threshold.object_ptrs().front()->instances.front()->get_matrix()));
}
