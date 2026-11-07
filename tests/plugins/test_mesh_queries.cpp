///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
// Public/native comparisons cover ABI ownership, immutable snapshots, geometry,
// cancellation and the numerical visibility policy used by CostSeamPlacer.
#include <catch2/catch.hpp>
#include <algorithm>
#include <future>
#include <type_traits>
#include "libslic3r/Api/plugin/cpp/MeshQueryViews.hpp"
#include "libslic3r/Api/plugin/cpp/VolumeViews.hpp"
#include "libslic3r/Plugins/SeamPlacer/SeamCostModel.hpp"
#include "libslic3r/AABBTreeIndirect.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/TriangleSetSampling.hpp"
#include "plugin_test_helpers.hpp"
#include "test_data.hpp"
extern "C" int test_mesh_queries_c(void);
namespace Slic3r { namespace MeshQueryVisibilityReference {
std::vector<float> calculate_cost_visibility(const slic3r_api::Object &, const std::vector<c_vec3f> &);
}} // namespace Slic3r::MeshQueryVisibilityReference
using namespace slic3r_api;

static c_matrix4d identity() {
    c_matrix4d m{};
    for (int i = 0; i < 4; ++i)
        m.value[i * 4 + i] = 1;
    return m;
}

static bool native_hit_order_less(const igl::Hit &lhs, const igl::Hit &rhs)
{
    return lhs.t < rhs.t || (lhs.t == rhs.t && lhs.id < rhs.id);
}

static bool mesh_hit_order_less(const mesh_query_hit &lhs, const mesh_query_hit &rhs)
{
    return lhs.distance < rhs.distance ||
           (lhs.distance == rhs.distance && lhs.triangle_id < rhs.triangle_id);
}

TEST_CASE("Mesh query ownership errors and native ray geometry", "[plugins][mesh-queries]") {
    REQUIRE(test_mesh_queries_c() == 1);
    static_assert(!std::is_copy_constructible<WorkingMesh>::value, "owner");
    static_assert(std::is_move_constructible<WorkingMesh>::value, "movable");
    Slic3r::TriangleMesh native(Slic3r::its_make_cube(10, 10, 10));
    const triangle_mesh_handle *source = reinterpret_cast<const triangle_mesh_handle *>(&native);
    WorkingMesh mesh;
    mesh.append(source, identity());
    c_matrix4d shift = identity();
    shift.value[3] = 20;
    MeshQueryScene scene({{mesh.handle(), identity(), 17}, {mesh.handle(), shift, 42}});
    const std::vector<mesh_query_sample> samples = scene.sample(128).values();
    REQUIRE(samples.size() == 128);
    CHECK(scene.sample(0).area() == Approx(1200));
    for (const mesh_query_sample &sample : samples)
        CHECK((sample.geometry_id == 17 || sample.geometry_id == 42));
    const std::vector<mesh_query_sample> again = scene.sample(128).values();
    for (size_t i = 0; i < samples.size(); ++i)
        CHECK(samples[i].position.x == again[i].position.x);
    const mesh_query_ray ray{{-1, 3, 4}, {2, 0, 0}, 0, 100};
    const MeshRayHits all = scene.cast({ray}, true);
    const std::vector<mesh_query_hit> hits = all.ray(0);
    REQUIRE(hits.size() == 4);
    CHECK(hits[0].distance == Approx(1));
    CHECK(hits[1].distance == Approx(11));
    CHECK(hits[2].distance == Approx(21));
    CHECK(hits[3].distance == Approx(31));
    CHECK(hits[0].normal.x == Approx(-1));
    CHECK(hits[0].geometry_id == 17);
    CHECK(hits[2].geometry_id == 42);
    mesh_query_ray clipped = ray;
    clipped.min_distance = 5;
    clipped.max_distance = 25;
    CHECK(scene.cast({clipped}, false).ray(0).front().distance == Approx(11));
    CHECK(scene.cast({clipped}, true).ray(0).size() == 2);
    clipped = ray;
    clipped.origin = {0, 3, 4};
    clipped.max_distance = 0;
    REQUIRE(scene.cast({clipped}, true).ray(0).size() == 1);
    CHECK(scene.cast({clipped}, false).ray(0).front().distance == 0);

    std::future<double> parallel = std::async(std::launch::async,
                                              [&] { return scene.cast({ray}, false).ray(0).front().distance; });
    CHECK(parallel.get() == Approx(1));
    // Mutation/destruction of construction inputs cannot change a published scene.
    mesh.decimate(4);
    mesh = WorkingMesh();
    native = Slic3r::TriangleMesh();
    CHECK(scene.cast({ray}, true).ray(0).size() == 4);
    const mesh_query_cancel cancelled{nullptr, [](void *) -> int32_t { return 1; }};
    CHECK_THROWS_AS(scene.sample(10, &cancelled), MeshQueryError);
    CHECK_THROWS_AS(scene.cast({ray}, true, &cancelled), MeshQueryError);
    mesh_query_ray invalid = ray;
    invalid.direction = {0, 0, 0};
    CHECK_THROWS_AS(scene.cast({invalid}, true), MeshQueryError);
    invalid = ray;
    invalid.max_distance = -1;
    CHECK_THROWS_AS(scene.cast({invalid}, true), MeshQueryError);
    mesh_query_scene *failed = nullptr;
    mesh_query_diagnostic diagnostic{};
    CHECK(mesh_query_scene_create(nullptr, 1, nullptr, &failed, &diagnostic) == MESH_QUERY_INVALID);
    CHECK(failed == nullptr);
    // Cancellation after a worker batch discards every partially computed hit.
    int polls = 0;
    const mesh_query_cancel late{&polls, [](void *context) -> int32_t { return ++*static_cast<int *>(context) >= 2; }};
    mesh_query_hits *partial = nullptr;
    CHECK(mesh_query_scene_cast(nullptr, &ray, 1, 1, nullptr, &partial, &diagnostic) == MESH_QUERY_INVALID);
    CHECK(partial == nullptr);
    CHECK_THROWS_AS(scene.cast({ray}, true, &late), MeshQueryError);
    CHECK(polls == 2);
    c_matrix4d bad = identity();
    bad.value[0] = std::numeric_limits<double>::infinity();
    CHECK_THROWS_AS(mesh.append(source, bad), MeshQueryError);
    CHECK_THROWS_AS(mesh.decimate(0), MeshQueryError);
    const MeshQueryScene empty({});
    CHECK(empty.sample(10).values().empty());
    CHECK(empty.sample(10).area() == 0);
    CHECK(empty.cast({ray}, true).ray(0).empty());
}
TEST_CASE("Mesh query sampling and hits agree with the native engine", "[plugins][mesh-queries]") {
    Slic3r::TriangleMesh native(Slic3r::its_make_cube(10, 10, 10));
    WorkingMesh mesh;
    mesh.append(reinterpret_cast<const triangle_mesh_handle *>(&native), identity());
    const MeshQueryScene scene({{mesh.handle(), identity(), 5}});
    const Slic3r::TriangleSetSamples expected = Slic3r::sample_its_uniform_parallel(100, native.its);
    const std::vector<mesh_query_sample> actual = scene.sample(100).values();
    for (size_t i = 0; i < actual.size(); ++i) {
        CHECK(actual[i].triangle_id == expected.triangle_indices[i]);
        CHECK(actual[i].position.x == expected.positions[i].x());
        CHECK(actual[i].normal.z == expected.normals[i].z());
    }
    const Slic3r::AABBTreeIndirect::Tree<3, float> tree =
        Slic3r::AABBTreeIndirect::build_aabb_tree_over_indexed_triangle_set(native.its.vertices, native.its.indices);
    for (int i = 1; i < 10; ++i) {
        const Slic3r::Vec3d origin(-2, i, .321), direction(1, 0, 0);
        std::vector<igl::Hit> reference;
        Slic3r::AABBTreeIndirect::intersect_ray_all_hits(native.its.vertices, native.its.indices, tree, origin,
                                                         direction, reference);
        std::sort(reference.begin(), reference.end(), native_hit_order_less);
        const std::vector<mesh_query_hit> got = scene.cast({{{-2, double(i), .321}, {1, 0, 0}, 0, 100}}, true).ray(0);
        REQUIRE(got.size() == reference.size());
        for (size_t j = 0; j < got.size(); ++j) {
            CHECK(got[j].distance == Approx(reference[j].t).margin(1e-6));
            CHECK(got[j].triangle_id == static_cast<uint64_t>(reference[j].id));
        }
    }

    Slic3r::TriangleMesh traversal_sensitive(
        std::vector<Slic3r::Vec3f>{
            {0.f, -1.f, -1.f}, {0.f, 1.f, -1.f}, {0.f, 0.f, 1.f},
            {10.f, -1.f, -1.f}, {10.f, 1.f, -1.f}, {10.f, 0.f, 1.f}},
        std::vector<Slic3r::Vec3i32>{{0, 1, 2}, {3, 4, 5}});
    WorkingMesh traversal_mesh;
    traversal_mesh.append(reinterpret_cast<const triangle_mesh_handle *>(&traversal_sensitive), identity());
    const MeshQueryScene traversal_scene({{traversal_mesh.handle(), identity(), 9}});
    const std::vector<mesh_query_hit> traversal_hits =
        traversal_scene.cast({{{20, 0, 0}, {-1, 0, 0}, 0, 100}}, true).ray(0);
    REQUIRE(traversal_hits.size() == 2);
    CHECK(std::is_sorted(traversal_hits.begin(), traversal_hits.end(), mesh_hit_order_less));
    CHECK(traversal_hits[0].distance == Approx(10));
    CHECK(traversal_hits[0].triangle_id == 1);
    CHECK(traversal_hits[1].distance == Approx(20));
    CHECK(traversal_hits[1].triangle_id == 0);
}
TEST_CASE("Mesh query visibility retains the native cost policy", "[plugins][mesh-queries][seam-cost][seam-visibility]") {
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Slic3r::Model model;
    Slic3r::Print print;
    Slic3r::Test::init_print({Slic3r::Test::TestMesh::cube_20x20x20}, print, model);
    const bool negative = GENERATE(false, true);
    if (negative) {
        Slic3r::ModelObject *object = const_cast<Slic3r::ModelObject *>(print.object(0).model_object());
        object->add_volume(Slic3r::TriangleMesh(Slic3r::its_make_cube(8, 8, 25)),
                           Slic3r::ModelVolumeType::NEGATIVE_VOLUME, false);
    }
    const Object object(reinterpret_cast<const object_handle *>(&print.object(0)));
    const std::vector<c_vec3f> points{{0, 0, 1}, {5, 5, 5}, {-10, -10, 3}, {10, 0, 10}, {0, 10, 15}};
    const std::vector<float> expected = Slic3r::MeshQueryVisibilityReference::calculate_cost_visibility(object,
                                                                                                        points);
    const std::vector<float> actual = Slic3r::SeamCostModel::calculate_cost_visibility(object, points);
    REQUIRE(actual.size() == expected.size());
    for (size_t i = 0; i < actual.size(); ++i)
        CHECK(actual[i] == Approx(expected[i]).margin(.0001));
}
