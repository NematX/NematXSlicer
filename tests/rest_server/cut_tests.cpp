// Direct plane-cut tests cover geometry and scene bookkeeping without HTTP transport.
#include <catch2/catch.hpp>

#include "cut.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/TriangleSelector.hpp"
#include "rest_models_test_helpers.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

using namespace Slic3r;
using namespace Slic3r::Rest;

namespace {

// Build a centered 20 mm cube with one instance and place its lowest point at
// platter Z zero; the returned model owns the object and its volume.
Model make_cube_scene()
{
    Model scene;
    ModelObject *object = scene.add_object("cube", "cube.stl", make_cube(20., 20., 20.));
    object->add_instance();
    object->ensure_on_bed();
    return scene;
}

// Return the platter-space axis-aligned bounds of every mesh vertex in one
// object instance; the object and selected instance must remain alive.
std::pair<Vec3d, Vec3d> object_bounds(const ModelObject &object, std::size_t instance_index = 0)
{
    Vec3d minimum = Vec3d::Constant(std::numeric_limits<double>::max());
    Vec3d maximum = Vec3d::Constant(std::numeric_limits<double>::lowest());
    const Transform3d instance = object.instances.at(instance_index)->get_matrix();
    for (const ModelVolume *volume : object.volumes)
        for (const Vec3f &vertex : volume->mesh().its.vertices) {
            const Vec3d point = instance * volume->get_matrix() * vertex.cast<double>();
            minimum = minimum.cwiseMin(point);
            maximum = maximum.cwiseMax(point);
        }
    return {minimum, maximum};
}

// Return the platter-space bounds of one volume in an object's selected
// instance; the volume must belong to that object and both must remain alive.
std::pair<Vec3d, Vec3d> volume_bounds(const ModelObject &object, const ModelVolume &volume,
    std::size_t instance_index = 0)
{
    Vec3d minimum = Vec3d::Constant(std::numeric_limits<double>::max());
    Vec3d maximum = Vec3d::Constant(std::numeric_limits<double>::lowest());
    const Transform3d instance = object.instances.at(instance_index)->get_matrix();
    for (const Vec3f &vertex : volume.mesh().its.vertices) {
        const Vec3d point = instance * volume.get_matrix() * vertex.cast<double>();
        minimum = minimum.cwiseMin(point);
        maximum = maximum.cwiseMax(point);
    }
    return {minimum, maximum};
}

// Create the common platter-space request used by the cube cases; the point
// is at the requested height and the normal points toward the upper side.
CutRequest horizontal_cut(double z)
{
    CutRequest request;
    request.point = Vec3d(0., 0., z);
    request.normal = Vec3d::UnitZ();
    return request;
}

// Build the transformed cube used to compare equivalent plane coordinate
// spaces; its instance has translation, rotation, and non-uniform scaling.
Model make_transformed_scene()
{
    Model scene = make_cube_scene();
    ModelObject *object = scene.object_ptrs().front();
    object->instances.front()->set_scaling_factor(Vec3d(1., 2., .5));
    object->instances.front()->set_rotation(Vec3d(.2, .3, .4));
    object->instances.front()->set_offset(Vec3d(30., 20., 5.));
    object->invalidate_bounding_box();
    return scene;
}

// Check a part cut against an object-space plane oracle for every unchanged instance.
void check_part_cut_frame(const std::size_t reference_index, const bool keep_both)
{
    constexpr double tolerance = 1e-3;
    Model scene;
    ModelObject *object = scene.add_object("parts", "parts.stl", make_cube(20., 20., 20.));
    ModelVolume *part = object->volumes.front();
    part->set_offset(Vec3d(3., -2., 4.));
    part->set_rotation(Vec3d(.1, .2, .3));
    ModelVolume *sibling = object->add_volume(make_cube(6., 6., 6.));
    sibling->set_offset(Vec3d(50., 0., 0.));
    object->add_instance();
    object->add_instance();
    object->instances[0]->set_rotation(Vec3d(.2, .3, .4));
    object->instances[0]->set_scaling_factor(Vec3d(1., 2., .5));
    object->instances[0]->set_offset(Vec3d(30., 20., 5.));
    object->instances[1]->set_rotation(Vec3d(-.3, .1, 1.));
    object->instances[1]->set_scaling_factor(Vec3d(1.5, 1., 1.));
    object->instances[1]->set_offset(Vec3d(-40., 10., 7.));

    const ObjectID sibling_id = sibling->id();
    const Transform3d sibling_matrix = sibling->get_matrix();
    const std::vector<ObjectID> instance_ids = [&object]() {
        std::vector<ObjectID> ids;
        for (const ModelInstance *instance : object->instances)
            ids.push_back(instance->id());
        return ids;
    }();
    const std::vector<Transform3d> instance_matrices = [&object]() {
        std::vector<Transform3d> matrices;
        for (const ModelInstance *instance : object->instances)
            matrices.push_back(instance->get_matrix());
        return matrices;
    }();
    REQUIRE(reference_index < object->instances.size());

    const std::vector<Vec3f> source_vertices = part->mesh().its.vertices;
    const Transform3d source_matrix = part->get_matrix();
    Vec3d local_minimum = Vec3d::Constant(std::numeric_limits<double>::max());
    Vec3d local_maximum = Vec3d::Constant(std::numeric_limits<double>::lowest());
    Vec3d object_minimum = Vec3d::Constant(std::numeric_limits<double>::max());
    Vec3d object_maximum = Vec3d::Constant(std::numeric_limits<double>::lowest());
    std::vector<Vec3d> source_object_vertices;
    for (const Vec3f &vertex : source_vertices) {
        const Vec3d local_vertex = vertex.cast<double>();
        const Vec3d object_vertex = source_matrix * local_vertex;
        local_minimum = local_minimum.cwiseMin(local_vertex);
        local_maximum = local_maximum.cwiseMax(local_vertex);
        object_minimum = object_minimum.cwiseMin(object_vertex);
        object_maximum = object_maximum.cwiseMax(object_vertex);
        source_object_vertices.push_back(object_vertex);
    }
    const Vec3d object_center = (object_minimum + object_maximum) * .5;
    const Transform3d reference_matrix = object->instances[reference_index]->get_matrix();
    const Vec3d platter_point = reference_matrix * object_center;
    const Vec3d platter_normal = Vec3d(1., .4, 1.).normalized();
    const Vec3d object_point = reference_matrix.inverse() * platter_point;
    const Vec3d object_normal = (reference_matrix.linear().transpose() * platter_normal).normalized();

    CutRequest request;
    request.point = platter_point;
    request.normal = platter_normal;
    request.keep_upper = keep_both;
    const CutOutcome outcome = cut_part(*object, *part, reference_index, request);
    REQUIRE(outcome.parts.size() == (keep_both ? 2U : 1U));
    REQUIRE(outcome.parts.size() == outcome.sides.size());

    ModelVolume *upper = nullptr;
    ModelVolume *lower = nullptr;
    for (std::size_t index = 0; index < outcome.parts.size(); ++index) {
        if (outcome.sides[index] == "upper")
            upper = outcome.parts[index];
        else if (outcome.sides[index] == "lower")
            lower = outcome.parts[index];
    }
    if (keep_both) {
        REQUIRE(upper != nullptr);
        REQUIRE(lower != nullptr);
    } else {
        REQUIRE(upper == nullptr);
        REQUIRE(lower != nullptr);
        CHECK(std::count(outcome.sides.begin(), outcome.sides.end(), "upper") == 0);
    }
    CHECK(sibling->id() == sibling_id);
    CHECK(sibling->get_matrix().matrix().isApprox(sibling_matrix.matrix(), 1e-9));
    REQUIRE(object->instances.size() == instance_ids.size());
    for (std::size_t index = 0; index < object->instances.size(); ++index) {
        CHECK(object->instances[index]->id() == instance_ids[index]);
        CHECK(object->instances[index]->get_matrix().matrix().isApprox(instance_matrices[index].matrix(), 1e-9));
    }

    const auto check_half = [&](ModelVolume &volume, const bool is_upper, const std::size_t instance_index) {
        const Transform3d instance_matrix = object->instances[instance_index]->get_matrix();
        const Transform3d inverse_instance = instance_matrix.inverse();
        const Transform3d platter_volume = instance_matrix * volume.get_matrix();
        std::size_t face_vertices = 0;
        for (const Vec3f &vertex : volume.mesh().its.vertices) {
            const Vec3d object_vertex = inverse_instance * (platter_volume * vertex.cast<double>());
            const double side = object_normal.dot(object_vertex - object_point);
            if (is_upper)
                CHECK(side >= -tolerance);
            else
                CHECK(side <= tolerance);
            if (std::abs(side) <= tolerance)
                ++face_vertices;
            const Vec3d local_vertex = source_matrix.inverse() * object_vertex;
            for (int axis = 0; axis < 3; ++axis) {
                CHECK(local_vertex(axis) >= local_minimum(axis) - tolerance);
                CHECK(local_vertex(axis) <= local_maximum(axis) + tolerance);
            }
        }
        CHECK(face_vertices >= 3);
        for (const Vec3d &source_vertex : source_object_vertices) {
            const Vec3d object_source_vertex = inverse_instance * (instance_matrix * source_vertex);
            const double side = object_normal.dot(object_source_vertex - object_point);
            if ((is_upper && side > tolerance) || (!is_upper && side < -tolerance)) {
                bool found = false;
                for (const Vec3f &vertex : volume.mesh().its.vertices) {
                    const Vec3d object_vertex = inverse_instance * (platter_volume * vertex.cast<double>());
                    if ((object_vertex - object_source_vertex).norm() <= tolerance) {
                        found = true;
                        break;
                    }
                }
                CHECK(found);
            }
        }
    };

    if (upper != nullptr) {
        check_half(*upper, true, 0);
        check_half(*upper, true, 1);
    }
    check_half(*lower, false, 0);
    check_half(*lower, false, 1);
}

} // namespace

// Verify part replacements stay in the object frame for both reference instances.
TEST_CASE("Part cuts keep the cut geometry in place for every instance", "[cut]")
{
    check_part_cut_frame(0, true);
    check_part_cut_frame(1, true);
    check_part_cut_frame(0, false);
}

// Verify both object sides, one-side retention, parts mode, pose options, and
// an inclined plane all use the existing GUI cut geometry operation.
TEST_CASE("Object cuts split the geometry as the GUI does", "[cut]")
{
    Model scene = make_cube_scene();
    ModelObject *source = scene.object_ptrs().front();
    const CutOutcome both = cut_object(scene, *source, 0, horizontal_cut(10.));
    REQUIRE(both.objects.size() == 2);
    REQUIRE(both.sides == std::vector<std::string>{"upper", "lower"});
    for (const ModelObject *object : both.objects) {
        const std::pair<Vec3d, Vec3d> bounds = object_bounds(*object);
        CHECK(bounds.first.z() == Approx(0.).margin(1e-6));
        CHECK(bounds.second.z() == Approx(10.).margin(1e-6));
    }

    Model upper_scene = make_cube_scene();
    CutRequest upper_request = horizontal_cut(10.);
    upper_request.keep_lower = false;
    const CutOutcome upper = cut_object(upper_scene, *upper_scene.object_ptrs().front(), 0, upper_request);
    REQUIRE(upper.objects.size() == 1);
    CHECK(upper.sides == std::vector<std::string>{"upper"});

    Model parts_scene = make_cube_scene();
    CutRequest parts_request = horizontal_cut(10.);
    parts_request.keep_as_parts = true;
    const CutOutcome parts = cut_object(parts_scene, *parts_scene.object_ptrs().front(), 0, parts_request);
    REQUIRE(parts.objects.size() == 1);
    REQUIRE(parts.objects.front()->volumes.size() == 2);
    CHECK(parts.sides == std::vector<std::string>{"parts"});
    CHECK(parts.objects.front()->volumes[0]->name.find("_A") != std::string::npos);
    CHECK(parts.objects.front()->volumes[1]->name.find("_B") != std::string::npos);

    Model pose_scene = make_cube_scene();
    CutRequest pose_request = horizontal_cut(10.);
    pose_request.flip_upper = true;
    pose_request.place_on_cut_lower = true;
    const CutOutcome pose = cut_object(pose_scene, *pose_scene.object_ptrs().front(), 0, pose_request);
    REQUIRE(pose.objects.size() == 2);
    for (const ModelObject *object : pose.objects)
        CHECK(object_bounds(*object).first.z() == Approx(0.).margin(1e-6));
    for (const ModelObject *object : pose.objects)
        CHECK(object_bounds(*object).second.z() == Approx(10.).margin(1e-6));

    Model inclined_scene = make_cube_scene();
    ModelObject *inclined_source = inclined_scene.object_ptrs().front();
    const std::pair<Vec3d, Vec3d> inclined_bounds = object_bounds(*inclined_source);
    TriangleMesh inclined_source_mesh = inclined_source->volumes.front()->mesh();
    const double inclined_original_volume = std::abs(inclined_source_mesh.volume()) *
        inclined_source->instances.front()->get_scaling_factor().prod();
    CutRequest inclined;
    inclined.point = (inclined_bounds.first + inclined_bounds.second) * .5;
    inclined.normal = Vec3d(1., 0., 1.);
    const CutOutcome inclined_result = cut_object(inclined_scene, *inclined_source, 0, inclined);
    REQUIRE(inclined_result.objects.size() == 2);
    double inclined_total_volume = 0.;
    for (const ModelObject *object : inclined_result.objects) {
        double object_volume = 0.;
        for (const ModelVolume *volume : object->volumes) {
            if (!volume->is_model_part())
                continue;
            TriangleMesh result_mesh = volume->mesh();
            object_volume += std::abs(result_mesh.volume()) * object->instances.front()->get_scaling_factor().prod();
        }
        inclined_total_volume += object_volume;
        CHECK(object_volume == Approx(inclined_original_volume * .5).epsilon(1e-4));
    }
    CHECK(inclined_total_volume == Approx(inclined_original_volume).epsilon(1e-4));

    const auto has_modifier = [](const ModelObject *object) {
        return std::any_of(object->volumes.begin(), object->volumes.end(),
            [](const ModelVolume *volume) { return volume->is_modifier(); });
    };
    Model crossing_modifier_scene = make_cube_scene();
    ModelObject *crossing_source = crossing_modifier_scene.object_ptrs().front();
    ModelVolume *crossing_modifier = crossing_source->add_volume(make_cube(4., 4., 4.), ModelVolumeType::PARAMETER_MODIFIER);
    crossing_modifier->set_offset(Vec3d(0., 0., 10.));
    const CutOutcome crossing_modifier_result = cut_object(crossing_modifier_scene, *crossing_source, 0, horizontal_cut(10.));
    REQUIRE(crossing_modifier_result.objects.size() == 2);
    CHECK(has_modifier(crossing_modifier_result.objects[0]));
    CHECK(has_modifier(crossing_modifier_result.objects[1]));

    Model upper_modifier_scene = make_cube_scene();
    ModelObject *upper_source = upper_modifier_scene.object_ptrs().front();
    ModelVolume *upper_modifier = upper_source->add_volume(make_cube(4., 4., 4.), ModelVolumeType::PARAMETER_MODIFIER);
    upper_modifier->set_offset(Vec3d(0., 0., 16.));
    const CutOutcome upper_modifier_result = cut_object(upper_modifier_scene, *upper_source, 0, horizontal_cut(10.));
    REQUIRE(upper_modifier_result.objects.size() == 2);
    CHECK(has_modifier(upper_modifier_result.objects[0]));
    CHECK_FALSE(has_modifier(upper_modifier_result.objects[1]));

    Model painted_scene = make_cube_scene();
    ModelObject *painted_source = painted_scene.object_ptrs().front();
    ModelVolume *painted_upper = painted_source->add_volume(make_cube(4., 4., 4.));
    painted_upper->set_offset(Vec3d(0., 0., 16.));
    for (ModelVolume *volume : painted_source->volumes) {
        TriangleSelector selector(volume->mesh());
        selector.set_facet(0, EnforcerBlockerType::ENFORCER);
        volume->facets_annotation_mutable("builtin:seam").set(selector);
    }
    ModelConfig range_config;
    range_config.set_key_value("layer_height", new ConfigOptionFloat(.2));
    painted_source->layer_config_ranges.emplace(t_layer_height_range{2., 6.}, std::move(range_config));
    const CutOutcome painted_result = cut_object(painted_scene, *painted_source, 0, horizontal_cut(10.));
    REQUIRE(painted_result.objects.size() == 2);
    for (const ModelObject *object : painted_result.objects) {
        const t_layer_height_range range{2., 6.};
        REQUIRE(object->layer_config_ranges.find(range) != object->layer_config_ranges.end());
        CHECK(object->layer_config_ranges.at(range).opt_float("layer_height") == Approx(.2));
        for (const ModelVolume *volume : object->volumes)
            CHECK(volume->facets_annotation_keys().empty());
    }
}

// Verify new objects remain outside a cut group on the first cut, while an
// existing group's checksum propagates to its retained peer objects.
TEST_CASE("Cut groups follow the GUI bookkeeping", "[cut]")
{
    Model fresh_scene = make_cube_scene();
    const CutOutcome fresh = cut_object(fresh_scene, *fresh_scene.object_ptrs().front(), 0, horizontal_cut(10.));
    REQUIRE(fresh.objects.size() == 2);
    CHECK(std::none_of(fresh.objects.begin(), fresh.objects.end(),
        [](const ModelObject *object) { return object->is_cut(); }));

    Model grouped_scene = make_cube_scene();
    ModelObject *source = grouped_scene.object_ptrs().front();
    ModelObject *peer = grouped_scene.add_object("peer", "peer.stl", make_cube(8., 8., 8.));
    peer->add_instance();
    peer->ensure_on_bed();
    source->cut_id.init();
    peer->cut_id = source->cut_id;
    const CutObjectBase original_group = source->cut_id;
    const CutOutcome grouped = cut_object(grouped_scene, *source, 0, horizontal_cut(10.));
    REQUIRE(grouped.objects.size() == 2);
    CHECK(grouped.objects[0]->is_cut());
    CHECK(grouped.objects[1]->is_cut());
    CHECK(grouped.objects[0]->cut_id.has_same_id(original_group));
    CHECK(grouped.objects[1]->cut_id.has_same_id(original_group));
    CHECK(grouped.objects[0]->cut_id.check_sum() == original_group.check_sum() + 1);
    CHECK(grouped.objects[1]->cut_id.check_sum() == original_group.check_sum() + 1);
    CHECK(peer->cut_id.is_equal(grouped.objects.front()->cut_id));
    CHECK(peer->cut_id.check_sum() == grouped.objects.front()->cut_id.check_sum());

    Model part_scene = make_cube_scene();
    const CutOutcome object_cut = cut_object(part_scene, *part_scene.object_ptrs().front(), 0, horizontal_cut(10.));
    REQUIRE(object_cut.objects.size() == 2);
    ModelObject *part_object = object_cut.objects.front();
    ModelObject *other_object = object_cut.objects.back();
    ModelVolume *part = part_object->volumes.front();
    part_object->cut_id.init();
    other_object->cut_id = part_object->cut_id;
    CutRequest restricted = horizontal_cut(5.);
    try {
        cut_part(*part_object, *part, 0, restricted);
        FAIL("A part of a cut object must require cut invalidation.");
    } catch (const SceneEditError &error) {
        CHECK(error.status() == 422);
        CHECK(error.code() == "cut_object");
    }
    restricted.invalidate_cut = true;
    const CutOutcome invalidated = cut_part(*part_object, *part, 0, restricted);
    REQUIRE(!invalidated.parts.empty());
    CHECK(!other_object->is_cut());
}

// Verify that equivalent platter/object and platter/volume requests produce
// the same transformed geometry, including inverse-transpose normals under
// non-uniform instance and volume transforms.
TEST_CASE("Cut planes are equivalent in platter object and volume spaces", "[cut]")
{
    Model platter_scene = make_transformed_scene();
    Model object_scene = make_transformed_scene();
    const Transform3d instance = platter_scene.object_ptrs().front()->instances.front()->get_matrix();
    const Vec3d local_point(0.73, 0.41, -0.37);
    const Vec3d platter_point = instance * local_point;
    const Vec3d platter_normal = Vec3d(1., .37, 1.).normalized();

    CutRequest platter = horizontal_cut(0.);
    platter.point = platter_point;
    platter.normal = platter_normal;
    CutRequest object = platter;
    object.space = CutSpace::object;
    object.point = instance.inverse() * platter_point;
    object.normal = instance.linear().transpose() * platter_normal;

    const CutOutcome platter_result = cut_object(platter_scene, *platter_scene.object_ptrs().front(), 0, platter);
    const CutOutcome object_result = cut_object(object_scene, *object_scene.object_ptrs().front(), 0, object);
    REQUIRE(platter_result.objects.size() == 2);
    REQUIRE(object_result.objects.size() == 2);
    for (std::size_t index = 0; index < 2; ++index) {
        CHECK(object_bounds(*platter_result.objects[index]).first.isApprox(
            object_bounds(*object_result.objects[index]).first, 1e-6));
        CHECK(object_bounds(*platter_result.objects[index]).second.isApprox(
            object_bounds(*object_result.objects[index]).second, 1e-6));
    }

    Model volume_platter_scene = make_transformed_scene();
    Model volume_scene = make_transformed_scene();
    ModelVolume *platter_volume = volume_platter_scene.object_ptrs().front()->volumes.front();
    ModelVolume *local_volume = volume_scene.object_ptrs().front()->volumes.front();
    platter_volume->set_offset(Vec3d(4., -3., 2.));
    platter_volume->set_rotation(Vec3d(0., 0., .3));
    local_volume->set_offset(Vec3d(4., -3., 2.));
    local_volume->set_rotation(Vec3d(0., 0., .3));
    ModelObject *platter_part_object = volume_platter_scene.object_ptrs().front();
    ModelObject *volume_part_object = volume_scene.object_ptrs().front();
    const Transform3d volume_mapping = platter_part_object->instances.front()->get_matrix() * platter_volume->get_matrix();
    const Vec3d volume_point = volume_mapping * local_point;
    const Vec3d volume_normal = Vec3d::UnitZ();
    CutRequest volume_platter = horizontal_cut(0.);
    volume_platter.point = volume_point;
    volume_platter.normal = volume_normal;
    CutRequest volume_local = volume_platter;
    volume_local.space = CutSpace::volume;
    volume_local.point = volume_mapping.inverse() * volume_point;
    volume_local.normal = volume_mapping.linear().transpose() * volume_normal;

    const CutOutcome volume_platter_result = cut_part(*platter_part_object, *platter_volume, 0, volume_platter);
    const CutOutcome volume_result = cut_part(*volume_part_object, *local_volume, 0, volume_local);
    REQUIRE(volume_platter_result.parts.size() == 2);
    REQUIRE(volume_result.parts.size() == 2);
    for (std::size_t index = 0; index < 2; ++index) {
        CHECK(volume_bounds(*platter_part_object, *volume_platter_result.parts[index]).first.isApprox(
            volume_bounds(*volume_part_object, *volume_result.parts[index]).first, 1e-6));
        CHECK(volume_bounds(*platter_part_object, *volume_platter_result.parts[index]).second.isApprox(
            volume_bounds(*volume_part_object, *volume_result.parts[index]).second, 1e-6));
    }
}

// Verify that replacing a part leaves sibling volumes and every instance intact,
// including the one-part case where ModelObject::delete_volume would rehome IDs.
TEST_CASE("Part cuts replace only the cut part", "[cut]")
{
    Model scene = make_cube_scene();
    ModelObject *object = scene.object_ptrs().front();
    ModelVolume *original = object->volumes.front();
    ModelVolume *sibling = object->add_volume(make_cube(6., 6., 6.));
    sibling->set_offset(Vec3d(30., 0., 0.));
    object->sort_volumes(true);
    const ObjectID sibling_id = sibling->id();
    const Transform3d sibling_matrix = sibling->get_matrix();
    const Transform3d instance_matrix = object->instances.front()->get_matrix();
    const ObjectID original_id = original->id();

    const CutOutcome outcome = cut_part(*object, *original, 0, horizontal_cut(10.));
    REQUIRE(outcome.parts.size() == 2);
    REQUIRE(object->volumes.size() == 3);
    CHECK(std::find_if(object->volumes.begin(), object->volumes.end(),
        [sibling_id](const ModelVolume *volume) { return volume->id() == sibling_id; }) != object->volumes.end());
    CHECK(sibling->id() == sibling_id);
    CHECK(sibling->get_matrix().matrix().isApprox(sibling_matrix.matrix(), 1e-6));
    CHECK(object->instances.front()->get_matrix().matrix().isApprox(instance_matrix.matrix(), 1e-6));
    CHECK(std::all_of(outcome.parts.begin(), outcome.parts.end(),
        [original_id](const ModelVolume *volume) { return volume->id() != original_id; }));

    Model single_scene = make_cube_scene();
    ModelObject *single = single_scene.object_ptrs().front();
    ModelVolume *single_original = single->volumes.front();
    const ObjectID single_id = single_original->id();
    const Transform3d single_instance = single->instances.front()->get_matrix();
    CutRequest one_side = horizontal_cut(10.);
    one_side.keep_lower = false;
    const CutOutcome one = cut_part(*single, *single_original, 0, one_side);
    REQUIRE(one.parts.size() == 1);
    CHECK(single->volumes.front()->id() != single_id);
    CHECK(single->instances.front()->get_matrix().matrix().isApprox(single_instance.matrix(), 1e-6));

    Model lower_scene = make_cube_scene();
    ModelObject *lower_object = lower_scene.object_ptrs().front();
    ModelVolume *lower_original = lower_object->volumes.front();
    CutRequest lower_request = horizontal_cut(10.);
    lower_request.keep_upper = false;
    const CutOutcome lower = cut_part(*lower_object, *lower_original, 0, lower_request);
    REQUIRE(lower.parts.size() == 1);
    CHECK(lower.sides == std::vector<std::string>{"lower"});
}

// Reject invalid combinations and planes without modifying the candidate scene.
TEST_CASE("Cut validation rejects degenerate requests", "[cut]")
{
    Model scene = make_cube_scene();
    ModelObject *object = scene.object_ptrs().front();
    const ObjectID object_id = object->id();
    CutRequest zero_normal = horizontal_cut(10.);
    zero_normal.normal = Vec3d::Zero();
    CHECK_THROWS_AS(cut_object(scene, *object, 0, zero_normal), SceneEditError);

    CutRequest no_side = horizontal_cut(10.);
    no_side.keep_upper = false;
    no_side.keep_lower = false;
    CHECK_THROWS_AS(cut_object(scene, *object, 0, no_side), SceneEditError);

    CutRequest invalid_parts = horizontal_cut(10.);
    invalid_parts.keep_as_parts = true;
    invalid_parts.flip_upper = true;
    CHECK_THROWS_AS(cut_object(scene, *object, 0, invalid_parts), SceneEditError);

    CutRequest outside = horizontal_cut(40.);
    CHECK_THROWS_AS(cut_object(scene, *object, 0, outside), SceneEditError);
    CHECK(scene.object_ptrs().size() == 1);
    CHECK(scene.object_ptrs().front()->id() == object_id);

    Model modifier_scene = make_cube_scene();
    ModelObject *modifier_object = modifier_scene.object_ptrs().front();
    ModelVolume *modifier = modifier_object->add_volume(make_cube(4., 4., 4.), ModelVolumeType::PARAMETER_MODIFIER);
    modifier->set_offset(Vec3d(0., 0., 10.));
    CutOutcome modifier_cut = cut_object(modifier_scene, *modifier_object, 0, horizontal_cut(10.));
    CHECK(modifier_cut.objects.size() == 2);
}
