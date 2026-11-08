// Direct tests cover facet selection, coordinate conversion, and channel storage.
#include <catch2/catch.hpp>

#include "paint.hpp"
#include "rest_models_test_helpers.hpp"

#include "libslic3r/TriangleSelector.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <utility>
#include <vector>

using namespace Slic3r;
using namespace Slic3r::Rest;

namespace {

// Build a centered cube with one instance and place its lowest model-part vertex on the platter.
Model make_cube_scene()
{
    Model scene;
    ModelObject *object = scene.add_object("cube", "cube.stl", make_cube(20., 20., 20.));
    object->add_instance();
    object->ensure_on_bed();
    return scene;
}

// Return the transformed bounds of one volume in the selected instance's platter coordinates.
std::pair<Vec3d, Vec3d> volume_bounds(const ModelObject &object, const ModelVolume &volume,
    std::size_t instance_index = 0)
{
    Vec3d minimum = Vec3d::Constant(std::numeric_limits<double>::max());
    Vec3d maximum = Vec3d::Constant(std::numeric_limits<double>::lowest());
    const Transform3d matrix = object.instances.at(instance_index)->get_matrix() * volume.get_matrix();
    for (const Vec3f &vertex : volume.mesh().its.vertices) {
        const Vec3d point = matrix * vertex.cast<double>();
        minimum = minimum.cwiseMin(point);
        maximum = maximum.cwiseMax(point);
    }
    return {minimum, maximum};
}

// Return the center of the upper or lower planar cube face in platter coordinates.
Vec3d cube_face_center(const ModelObject &object, const ModelVolume &volume, bool upper)
{
    const std::pair<Vec3d, Vec3d> bounds = volume_bounds(object, volume);
    return Vec3d((bounds.first.x() + bounds.second.x()) * .5,
        (bounds.first.y() + bounds.second.y()) * .5, upper ? bounds.second.z() : bounds.first.z());
}

// Create the common one-point painting request for a channel and state.
PaintRequest point_request(const std::string &kind, EnforcerBlockerType state, const Vec3d &point)
{
    PaintRequest request;
    request.kind = kind;
    request.state = state;
    request.strokes = {{PaintPoint{false, point, Vec3d::Zero()}}};
    return request;
}

// Sum painted triangle area, optionally filtering by face normal.
double painted_area(const ModelInstance &instance, const ModelVolume &volume, const std::string &kind,
    EnforcerBlockerType state, std::optional<Vec3d> normal = {})
{
    const std::vector<PaintedState> states = painted_triangles(instance, volume, kind, PaintSpace::platter);
    double area = 0.;
    for (const PaintedState &painted : states) {
        if (painted.state != state)
            continue;
        for (const stl_triangle_vertex_indices &face : painted.triangles.indices) {
            const Vec3d a = painted.triangles.vertices[face[0]].cast<double>();
            const Vec3d b = painted.triangles.vertices[face[1]].cast<double>();
            const Vec3d c = painted.triangles.vertices[face[2]].cast<double>();
            const Vec3d cross = (b - a).cross(c - a);
            if (normal.has_value() && cross.norm() > 0. &&
                cross.normalized().dot(normal->normalized()) < .9)
                continue;
            area += cross.norm() * .5;
        }
    }
    return area;
}

// Return the maximum distance of vertices in one painted state from a reference point.
double painted_vertex_distance(const ModelInstance &instance, const ModelVolume &volume,
    const std::string &kind, EnforcerBlockerType state, const Vec3d &reference)
{
    double distance = 0.;
    for (const PaintedState &painted : painted_triangles(instance, volume, kind, PaintSpace::platter)) {
        if (painted.state != state)
            continue;
        for (const stl_triangle_vertex_indices &face : painted.triangles.indices) {
            const Vec3d a = painted.triangles.vertices[face[0]].cast<double>();
            const Vec3d b = painted.triangles.vertices[face[1]].cast<double>();
            const Vec3d c = painted.triangles.vertices[face[2]].cast<double>();
            if ((b - a).cross(c - a).normalized().dot(Vec3d::UnitZ()) < .9)
                continue;
            distance = std::max(distance, (a - reference).norm());
            distance = std::max(distance, (b - reference).norm());
            distance = std::max(distance, (c - reference).norm());
        }
    }
    return distance;
}

// Create a transformed cube whose instance has translation, rotation, and non-uniform scale.
Model make_transformed_scene()
{
    Model scene = make_cube_scene();
    ModelObject *object = scene.object_ptrs().front();
    object->volumes.front()->set_rotation(Vec3d(0., 0., .5));
    object->volumes.front()->set_offset(Vec3d(3., -2., 1.));
    object->instances.front()->set_scaling_factor(Vec3d(2., 1., 1.));
    object->instances.front()->set_rotation(Vec3d(.2, .3, .4));
    object->instances.front()->set_offset(Vec3d(30., 20., 5.));
    object->invalidate_bounding_box();
    return scene;
}

// Transform a platter-space point or direction into one request coordinate space.
Vec3d request_point(const ModelObject &object, const ModelVolume &volume, PaintSpace space,
    const Vec3d &platter_point)
{
    Transform3d mapping = Transform3d::Identity();
    if (space == PaintSpace::object)
        mapping = object.instances.front()->get_matrix();
    else if (space == PaintSpace::volume)
        mapping = object.instances.front()->get_matrix() * volume.get_matrix();
    return mapping.inverse() * platter_point;
}

// Transform a platter direction without treating it as a surface normal.
Vec3d request_direction(const ModelObject &object, const ModelVolume &volume, PaintSpace space,
    const Vec3d &platter_direction)
{
    Transform3d mapping = Transform3d::Identity();
    if (space == PaintSpace::object)
        mapping = object.instances.front()->get_matrix();
    else if (space == PaintSpace::volume)
        mapping = object.instances.front()->get_matrix() * volume.get_matrix();
    return (mapping.linear().inverse() * platter_direction).normalized();
}

// Return the transformed center of a volume's original upper planar face.
Vec3d transformed_face_center(const ModelObject &object, const ModelVolume &volume)
{
    Vec3d minimum = Vec3d::Constant(std::numeric_limits<double>::max());
    Vec3d maximum = Vec3d::Constant(std::numeric_limits<double>::lowest());
    for (const Vec3f &vertex : volume.mesh().its.vertices) {
        const Vec3d point = vertex.cast<double>();
        minimum = minimum.cwiseMin(point);
        maximum = maximum.cwiseMax(point);
    }
    const Vec3d local((minimum.x() + maximum.x()) * .5,
        (minimum.y() + maximum.y()) * .5, maximum.z());
    return object.instances.front()->get_matrix() * volume.get_matrix() * local;
}

// Serialize one request-space surface circle on the same transformed physical face.
PaintRequest transformed_circle(ModelObject &object, ModelVolume &volume, PaintSpace space,
    const Vec3d &platter_point)
{
    PaintRequest request = point_request("support", EnforcerBlockerType::ENFORCER,
        request_point(object, volume, space, platter_point));
    request.space = space;
    return request;
}

// Return a cube's one original top-face area after a direct triangle selection.
double top_face_area(const ModelObject &object, const ModelVolume &volume)
{
    return painted_area(*object.instances.front(), volume, "support", EnforcerBlockerType::ENFORCER,
        Vec3d::UnitZ());
}

} // namespace

// Verify brush visibility, capsule coverage, spherical edge behavior, and erasing.
TEST_CASE("Brush strokes paint the expected area", "[paint]")
{
    Model scene = make_cube_scene();
    ModelObject *object = scene.object_ptrs().front();
    ModelVolume *volume = object->volumes.front();
    const Vec3d top = cube_face_center(*object, *volume, true);
    PaintRequest circle = point_request("support", EnforcerBlockerType::ENFORCER, top);
    validate_paint_request(circle, 1);
    REQUIRE(paint(*object, volume, 0, circle).changed == std::vector<ModelVolume *>{volume});
    CHECK(painted_area(*object->instances.front(), *volume, "support", EnforcerBlockerType::ENFORCER,
        Vec3d::UnitZ()) == Approx(std::acos(-1.) * 4.).epsilon(.2));
    CHECK(painted_vertex_distance(*object->instances.front(), *volume, "support",
        EnforcerBlockerType::ENFORCER, top) < 2.4);
    CHECK(painted_area(*object->instances.front(), *volume, "support", EnforcerBlockerType::ENFORCER,
        Vec3d::UnitX()) == Approx(0.));

    REQUIRE(clear_paint(*object, volume, "support") == std::vector<ModelVolume *>{volume});
    PaintRequest edge = point_request("support", EnforcerBlockerType::ENFORCER,
        Vec3d(volume_bounds(*object, *volume).second.x() - 1., top.y(), top.z()));
    REQUIRE(paint(*object, volume, 0, edge).changed == std::vector<ModelVolume *>{volume});
    CHECK(painted_area(*object->instances.front(), *volume, "support", EnforcerBlockerType::ENFORCER,
        Vec3d::UnitX()) == Approx(0.));
    REQUIRE(clear_paint(*object, volume, "support") == std::vector<ModelVolume *>{volume});
    edge.kind = "support";
    edge.tool = PaintTool::sphere;
    REQUIRE(paint(*object, volume, 0, edge).changed == std::vector<ModelVolume *>{volume});
    CHECK(painted_area(*object->instances.front(), *volume, "support", EnforcerBlockerType::ENFORCER,
        Vec3d::UnitX()) > 0.);

    REQUIRE(clear_paint(*object, volume, "support") == std::vector<ModelVolume *>{volume});
    PaintRequest capsule = point_request("support", EnforcerBlockerType::ENFORCER,
        top + Vec3d(-4., 0., 0.));
    capsule.strokes.front().push_back(PaintPoint{false, top + Vec3d(4., 0., 0.), Vec3d::Zero()});
    REQUIRE(paint(*object, volume, 0, capsule).changed == std::vector<ModelVolume *>{volume});
    CHECK(painted_area(*object->instances.front(), *volume, "support", EnforcerBlockerType::ENFORCER,
        Vec3d::UnitZ()) == Approx(8. * 4. + std::acos(-1.) * 4.).epsilon(.2));

    REQUIRE(clear_paint(*object, volume, "support") == std::vector<ModelVolume *>{volume});
    PaintRequest fill = point_request("support", EnforcerBlockerType::ENFORCER, top);
    fill.tool = PaintTool::smart_fill;
    REQUIRE(paint(*object, volume, 0, fill).changed == std::vector<ModelVolume *>{volume});
    PaintRequest erase = point_request("support", EnforcerBlockerType::NONE, top);
    erase.radius = 2.;
    REQUIRE(paint(*object, volume, 0, erase).changed == std::vector<ModelVolume *>{volume});
    CHECK(painted_area(*object->instances.front(), *volume, "support", EnforcerBlockerType::ENFORCER,
        Vec3d::UnitZ()) == Approx(400. - std::acos(-1.) * 4.).epsilon(.2));
}

// Verify triangle, smart-fill, bucket, overhang-only, and angle-based GUI operations.
TEST_CASE("Fill tools follow the GUI semantics", "[paint]")
{
    Model scene = make_cube_scene();
    ModelObject *object = scene.object_ptrs().front();
    ModelVolume *volume = object->volumes.front();
    const Vec3d top = cube_face_center(*object, *volume, true);
    const Vec3d bottom = cube_face_center(*object, *volume, false);

    PaintRequest triangle = point_request("support", EnforcerBlockerType::ENFORCER, top);
    triangle.tool = PaintTool::triangle;
    REQUIRE(paint(*object, volume, 0, triangle).changed == std::vector<ModelVolume *>{volume});
    CHECK(top_face_area(*object, *volume) == Approx(200.).margin(1e-3));

    REQUIRE(clear_paint(*object, volume, "support") == std::vector<ModelVolume *>{volume});
    PaintRequest smart = triangle;
    smart.tool = PaintTool::smart_fill;
    REQUIRE(paint(*object, volume, 0, smart).changed == std::vector<ModelVolume *>{volume});
    CHECK(top_face_area(*object, *volume) == Approx(400.).margin(1e-3));

    PaintRequest bucket = point_request("support", EnforcerBlockerType::BLOCKER, top);
    bucket.tool = PaintTool::bucket_fill;
    REQUIRE(paint(*object, volume, 0, bucket).changed == std::vector<ModelVolume *>{volume});
    CHECK(painted_area(*object->instances.front(), *volume, "support", EnforcerBlockerType::BLOCKER,
        Vec3d::UnitZ()) == Approx(400.).margin(1e-3));
    CHECK(painted_area(*object->instances.front(), *volume, "support", EnforcerBlockerType::ENFORCER,
        Vec3d::UnitZ()) == Approx(0.));

    REQUIRE(clear_paint(*object, volume, "support") == std::vector<ModelVolume *>{volume});
    PaintRequest angle = point_request("support", EnforcerBlockerType::ENFORCER, Vec3d::Zero());
    angle.tool = PaintTool::by_angle;
    angle.strokes.clear();
    angle.angle = 30.;
    REQUIRE(paint(*object, volume, 0, angle).changed == std::vector<ModelVolume *>{volume});
    CHECK(painted_area(*object->instances.front(), *volume, "support", EnforcerBlockerType::ENFORCER,
        -Vec3d::UnitZ()) == Approx(400.).margin(1e-3));
    CHECK(painted_area(*object->instances.front(), *volume, "support", EnforcerBlockerType::ENFORCER,
        Vec3d::UnitZ()) == Approx(0.));

    REQUIRE(clear_paint(*object, volume, "support") == std::vector<ModelVolume *>{volume});
    PaintRequest lower = point_request("support", EnforcerBlockerType::ENFORCER, bottom);
    lower.tool = PaintTool::smart_fill;
    lower.overhangs_only = 30.;
    REQUIRE(paint(*object, volume, 0, lower).changed == std::vector<ModelVolume *>{volume});
    CHECK(painted_area(*object->instances.front(), *volume, "support", EnforcerBlockerType::ENFORCER,
        -Vec3d::UnitZ()) == Approx(400.).margin(1e-3));
    REQUIRE(clear_paint(*object, volume, "support") == std::vector<ModelVolume *>{volume});
    PaintRequest upper = lower;
    upper.strokes.front().front().position = top;
    REQUIRE(paint(*object, volume, 0, upper).changed.empty());
}

// Verify surface points and rays agree across platter, object, and volume spaces under scaling.
TEST_CASE("Points rays and spaces select the same paint", "[paint]")
{
    Model platter_scene = make_transformed_scene();
    Model object_scene = make_transformed_scene();
    Model volume_scene = make_transformed_scene();
    Model *scenes[] = {&platter_scene, &object_scene, &volume_scene};
    const PaintSpace spaces[] = {PaintSpace::platter, PaintSpace::object, PaintSpace::volume};
    std::vector<std::pair<std::vector<std::pair<int, int>>, std::vector<bool>>> serialized;
    for (std::size_t index = 0; index < 3; ++index) {
        ModelObject *object = scenes[index]->object_ptrs().front();
        ModelVolume *volume = object->volumes.front();
        const Vec3d center = transformed_face_center(*object, *volume);
        PaintRequest request = transformed_circle(*object, *volume, spaces[index], center);
        request.radius = 2.;
        validate_paint_request(request, 1);
        REQUIRE(paint(*object, volume, 0, request).changed == std::vector<ModelVolume *>{volume});
        serialized.push_back(volume->supported_facets.get_data());
        CHECK(painted_area(*object->instances.front(), *volume, "support", EnforcerBlockerType::ENFORCER,
            Vec3d::UnitZ()) == Approx(std::acos(-1.) * 4.).epsilon(.2));
    }
    CHECK(serialized[0] == serialized[1]);
    CHECK(serialized[0] == serialized[2]);

    Model ray_scene = make_transformed_scene();
    ModelObject *ray_object = ray_scene.object_ptrs().front();
    ModelVolume *ray_volume = ray_object->volumes.front();
    const Vec3d ray_hit = transformed_face_center(*ray_object, *ray_volume);
    const Transform3d ray_matrix = ray_object->instances.front()->get_matrix() * ray_volume->get_matrix();
    const Vec3d platter_view = -(ray_matrix.linear().inverse().transpose() * Vec3d::UnitZ()).normalized();
    PaintRequest ray;
    ray.kind = "support";
    ray.state = EnforcerBlockerType::ENFORCER;
    ray.space = PaintSpace::object;
    ray.strokes = {{PaintPoint{true,
        request_point(*ray_object, *ray_volume, PaintSpace::object, ray_hit - platter_view * 100.),
        request_direction(*ray_object, *ray_volume, PaintSpace::object, platter_view)}}};
    REQUIRE(paint(*ray_object, ray_volume, 0, ray).changed == std::vector<ModelVolume *>{ray_volume});
    CHECK(ray_volume->supported_facets.get_data() == serialized[0]);
}

// Verify that an explicit oblique ray view changes circle projection and painted top area.
TEST_CASE("Ray view directions change circle projection", "[paint]")
{
    Model scene_a = make_cube_scene();
    ModelObject *object_a = scene_a.object_ptrs().front();
    ModelVolume *volume_a = object_a->volumes.front();
    const Vec3d top = cube_face_center(*object_a, *volume_a, true);
    PaintRequest request_a;
    request_a.kind = "support";
    request_a.state = EnforcerBlockerType::ENFORCER;
    request_a.radius = 2.;
    request_a.strokes = {{PaintPoint{true, top + Vec3d(0., 0., 100.), -Vec3d::UnitZ()}}};
    validate_paint_request(request_a, 1);
    REQUIRE(paint(*object_a, volume_a, 0, request_a).changed == std::vector<ModelVolume *>{volume_a});
    const std::pair<std::vector<std::pair<int, int>>, std::vector<bool>> data_a = volume_a->supported_facets.get_data();
    const double area_a = painted_area(*object_a->instances.front(), *volume_a, "support",
        EnforcerBlockerType::ENFORCER, Vec3d::UnitZ());

    Model scene_b = make_cube_scene();
    ModelObject *object_b = scene_b.object_ptrs().front();
    ModelVolume *volume_b = object_b->volumes.front();
    PaintRequest request_b = request_a;
    request_b.view_direction = Vec3d(1., 0., -.3).normalized();
    validate_paint_request(request_b, 1);
    REQUIRE(paint(*object_b, volume_b, 0, request_b).changed == std::vector<ModelVolume *>{volume_b});
    const std::pair<std::vector<std::pair<int, int>>, std::vector<bool>> data_b = volume_b->supported_facets.get_data();
    const double area_b = painted_area(*object_b->instances.front(), *volume_b, "support",
        EnforcerBlockerType::ENFORCER, Vec3d::UnitZ());
    CAPTURE(area_a, area_b);
    CHECK(data_a != data_b);
    CHECK(std::abs(area_b - area_a) > area_a * .01);
}

// Verify object targets split one stroke into nearest-part runs and preserve changed-part order.
TEST_CASE("Object targets paint the nearest part", "[paint]")
{
    Model scene;
    ModelObject *object = scene.add_object("parts", "parts.stl", make_cube(20., 20., 20.));
    ModelVolume *first = object->volumes.front();
    ModelVolume *second = object->add_volume(make_cube(20., 20., 20.));
    second->set_offset(Vec3d(30., 0., 0.));
    object->add_instance();
    object->ensure_on_bed();
    const Vec3d first_center = cube_face_center(*object, *first, true);
    const Vec3d second_center = cube_face_center(*object, *second, true);
    PaintRequest request = point_request("support", EnforcerBlockerType::ENFORCER,
        first_center + Vec3d(-4., 0., 0.));
    request.strokes.front().push_back(PaintPoint{false, first_center + Vec3d(4., 0., 0.), Vec3d::Zero()});
    request.strokes.front().push_back(PaintPoint{false, second_center, Vec3d::Zero()});
    const PaintOutcome outcome = paint(*object, nullptr, 0, request);
    REQUIRE(outcome.changed == std::vector<ModelVolume *>{first, second});
    CHECK(painted_area(*object->instances.front(), *first, "support", EnforcerBlockerType::ENFORCER,
        Vec3d::UnitZ()) == Approx(8. * 4. + std::acos(-1.) * 4.).epsilon(.2));
    CHECK(painted_area(*object->instances.front(), *second, "support", EnforcerBlockerType::ENFORCER,
        Vec3d::UnitZ()) == Approx(std::acos(-1.) * 4.).epsilon(.2));
}

// Verify MMU ranges, generic annotation storage, and removal of empty generic channels.
TEST_CASE("MMU and generic channels store their states", "[paint]")
{
    Model scene = make_cube_scene();
    ModelObject *object = scene.object_ptrs().front();
    ModelVolume *volume = object->volumes.front();
    const Vec3d top = cube_face_center(*object, *volume, true);
    PaintRequest mmu = point_request("mmu", static_cast<EnforcerBlockerType>(2), top);
    validate_paint_request(mmu, 3);
    REQUIRE(paint(*object, volume, 0, mmu).changed == std::vector<ModelVolume *>{volume});
    CHECK(painted_area(*object->instances.front(), *volume, "mmu", static_cast<EnforcerBlockerType>(2),
        Vec3d::UnitZ()) > 0.);
    CHECK_THROWS_AS(validate_paint_request(mmu, 1), SceneEditError);
    mmu.state = static_cast<EnforcerBlockerType>(4);
    CHECK_THROWS_AS(validate_paint_request(mmu, 3), SceneEditError);

    PaintRequest generic = point_request("builtin:seam", EnforcerBlockerType::BLOCKER, top);
    REQUIRE(paint(*object, volume, 0, generic).changed == std::vector<ModelVolume *>{volume});
    CHECK(volume->has_facets_annotation("builtin:seam"));
    const std::vector<std::string> keys = volume->facets_annotation_keys();
    CHECK(std::find(keys.begin(), keys.end(), "builtin:seam") != keys.end());
    REQUIRE(clear_paint(*object, volume, "builtin:seam") == std::vector<ModelVolume *>{volume});
    CHECK_FALSE(volume->has_facets_annotation("builtin:seam"));
}

// Verify point errors and request bounds occur before any selector data is changed.
TEST_CASE("Paint validation leaves the scene unchanged", "[paint]")
{
    Model scene = make_cube_scene();
    ModelObject *object = scene.object_ptrs().front();
    ModelVolume *volume = object->volumes.front();
    const Vec3d top = cube_face_center(*object, *volume, true);
    const std::pair<std::vector<std::pair<int, int>>, std::vector<bool>> before = volume->supported_facets.get_data();
    PaintRequest off_surface = point_request("support", EnforcerBlockerType::ENFORCER, top + Vec3d(0., 0., 1.));
    validate_paint_request(off_surface, 1);
    try {
        paint(*object, volume, 0, off_surface);
        FAIL("An off-surface point was accepted.");
    } catch (const PaintPointError &error) {
        CHECK(error.stroke() == 0);
        CHECK(error.point() == 0);
        CHECK(error.reason() == "off_surface");
    }
    CHECK(volume->supported_facets.get_data() == before);

    PaintRequest no_hit = off_surface;
    no_hit.strokes.front().front() = PaintPoint{true, Vec3d(100., 100., 100.), Vec3d::UnitZ()};
    try {
        paint(*object, volume, 0, no_hit);
        FAIL("A ray without an impact was accepted.");
    } catch (const PaintPointError &error) {
        CHECK(error.reason() == "no_hit");
    }
    CHECK(volume->supported_facets.get_data() == before);

    for (double radius : {0.04, 2001.}) {
        PaintRequest invalid = off_surface;
        invalid.radius = radius;
        CHECK_THROWS_AS(validate_paint_request(invalid, 1), SceneEditError);
    }
    PaintRequest invalid = off_surface;
    invalid.radius = 2.;
    invalid.tolerance = 0.;
    CHECK_THROWS_AS(validate_paint_request(invalid, 1), SceneEditError);
    invalid.tolerance = .05;
    invalid.view_direction = Vec3d::Zero();
    CHECK_THROWS_AS(validate_paint_request(invalid, 1), SceneEditError);
    invalid.view_direction.reset();
    invalid.strokes.front().push_back(PaintPoint{true, top, Vec3d::UnitZ()});
    CHECK_THROWS_AS(validate_paint_request(invalid, 1), SceneEditError);
    invalid.strokes.front().pop_back();
    invalid.strokes.front().resize(10001, invalid.strokes.front().front());
    CHECK_THROWS_AS(validate_paint_request(invalid, 1), SceneEditError);

    PaintRequest by_angle = invalid;
    by_angle.strokes.clear();
    by_angle.tool = PaintTool::by_angle;
    by_angle.radius = 2.;
    CHECK_NOTHROW(validate_paint_request(by_angle, 1));
    CHECK(paint(*object, volume, 0, by_angle).changed.empty());
}
