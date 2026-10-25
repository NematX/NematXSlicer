/* Storage-owned triangle meshes created from plugin arrays. The fixture is a
   10 mm cube whose triangles face outward, so POSITIVE slicing keeps its contour.
   The tests check exact readback, the regular and POSITIVE slicing entry points at
   several Z values, rejection of invalid arrays, release through the registry, and
   the owner's release across scopes and moves. */
#include <catch2/catch.hpp>

#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/c/slic3r_volume.h"
#include "libslic3r/Api/plugin/cpp/VolumeViews.hpp"

namespace {

const std::vector<c_vec3f> cube_vertices {
    {0.f, 0.f, 0.f}, {10.f, 0.f, 0.f}, {10.f, 10.f, 0.f}, {0.f, 10.f, 0.f},
    {0.f, 0.f, 10.f}, {10.f, 0.f, 10.f}, {10.f, 10.f, 10.f}, {0.f, 10.f, 10.f}};

// Each triangle is counter-clockwise when seen from outside the cube.
const std::vector<c_triangle_indices> cube_triangles {
    {0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7}, {0, 1, 5}, {0, 5, 4},
    {2, 3, 7}, {2, 7, 6}, {0, 4, 7}, {0, 7, 3}, {1, 2, 6}, {1, 6, 5}};

// Unscaled identity transform passed to the slicing and projection entry points.
c_matrix4d identity_transform()
{
    c_matrix4d transform{};
    for (int i = 0; i < 4; ++i)
        transform.value[i * 4 + i] = 1;
    return transform;
}

// Mode 0 (REGULAR) with the identity transform, which must be set explicitly.
c_mesh_slicing_params regular_params()
{
    c_mesh_slicing_params params{};
    params.transform = identity_transform();
    return params;
}

// RAW_MESH_SLICING_MODE_POSITIVE is the CloseHoles slicing mode of the native object slicing.
c_mesh_slicing_params positive_params()
{
    c_mesh_slicing_params params{};
    params.mode = RAW_MESH_SLICING_MODE_POSITIVE;
    params.mode_below = RAW_MESH_SLICING_MODE_POSITIVE;
    params.transform = identity_transform();
    return params;
}

// Exact comparison: readback must return the stored values unchanged.
bool same_vertex(const c_vec3f &lhs, const c_vec3f &rhs)
{
    return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

bool same_triangle(const c_triangle_indices &lhs, const c_triangle_indices &rhs)
{
    return lhs.a == rhs.a && lhs.b == rhs.b && lhs.c == rhs.c;
}

} // namespace

TEST_CASE("Storage-owned triangle mesh reads back arrays and slices through every entry point",
          "[plugins][triangle-mesh][volume]")
{
    Slic3r::PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    slic3r_api::StoredTriangleMesh mesh(handle, cube_vertices, cube_triangles);
    REQUIRE(storage.size() == 1);

    CHECK(mesh.vertex_count() == 8);
    CHECK(mesh.triangle_count() == 12);
    CHECK_FALSE(mesh.empty());
    for (uint32_t idx = 0; idx < cube_vertices.size(); ++idx)
        CHECK(same_vertex(mesh.vertex(idx), cube_vertices[idx]));
    for (uint32_t idx = 0; idx < cube_triangles.size(); ++idx)
        CHECK(same_triangle(mesh.triangle(idx), cube_triangles[idx]));

    // The last height is above the cube, so every entry point returns nothing there.
    const std::vector<float> zs {2.5f, 5.f, 7.5f, 11.f};
    const c_mesh_slicing_params positive = positive_params();

    const std::vector<slic3r_api::StoredExPolygonCollection> expolygons =
        mesh.slice_to_expolygons(handle, positive, zs, nullptr);
    REQUIRE(expolygons.size() == zs.size());
    for (size_t idx = 0; idx + 1 < zs.size(); ++idx) {
        REQUIRE(expolygons[idx].size() == 1);
        CHECK(unscaled(unscaled(expolygons[idx].at(0).area())) == Approx(100.));
    }
    CHECK(expolygons.back().empty());

    const std::vector<slic3r_api::StoredPolygonCollection> regular =
        mesh.slice_to_polygons(handle, regular_params(), zs, nullptr);
    REQUIRE(regular.size() == zs.size());
    CHECK(regular[1].size() == 1);
    CHECK(regular.back().empty());

    const std::vector<slic3r_api::StoredExPolygonCollection> regular_expolygons =
        mesh.slice_to_expolygons(handle, regular_params(), {5.f}, nullptr);
    REQUIRE(regular_expolygons.size() == 1);
    REQUIRE(regular_expolygons.front().size() == 1);
    CHECK(unscaled(unscaled(regular_expolygons.front().at(0).area())) == Approx(100.));

    std::vector<slic3r_api::StoredPolygonCollection> positive_polygons;
    std::vector<polygon_collection_handle *> positive_handles;
    for (size_t idx = 0; idx < zs.size(); ++idx) {
        positive_polygons.emplace_back(handle);
        positive_handles.push_back(positive_polygons.back().mutable_handle());
    }
    CHECK(triangle_mesh_slice_to_polygons(mesh.handle(), &positive, zs.data(), positive_handles.data(),
                                          uint32_t(zs.size()), nullptr) == RAW_MESH_SLICE_STATUS_DONE);
    REQUIRE(positive_polygons.size() == zs.size());
    CHECK(positive_polygons[1].size() == 1);
    CHECK(unscaled(unscaled(positive_polygons[1].at(0).area())) == Approx(100.));
    CHECK(positive_polygons.back().empty());

    CHECK(mesh.project_to_polygons(handle, identity_transform()).size() > 0);
}

TEST_CASE("Storage-owned triangle mesh rejects invalid arrays and releases its allocation",
          "[plugins][triangle-mesh][volume]")
{
    Slic3r::PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    CHECK(storage_new_triangle_mesh(nullptr, cube_vertices.data(), 8, cube_triangles.data(), 12) == nullptr);
    CHECK(storage_new_triangle_mesh(handle, nullptr, 8, cube_triangles.data(), 12) == nullptr);
    CHECK(storage_new_triangle_mesh(handle, cube_vertices.data(), 8, nullptr, 12) == nullptr);

    std::vector<c_triangle_indices> outside = cube_triangles;
    outside.back().c = 8;
    CHECK(storage_new_triangle_mesh(handle, cube_vertices.data(), 8, outside.data(), 12) == nullptr);

    std::vector<c_triangle_indices> repeated = cube_triangles;
    repeated.front() = {0, 0, 1};
    CHECK(storage_new_triangle_mesh(handle, cube_vertices.data(), 8, repeated.data(), 12) == nullptr);
    repeated.front() = {0, 1, 0};
    CHECK(storage_new_triangle_mesh(handle, cube_vertices.data(), 8, repeated.data(), 12) == nullptr);
    repeated.front() = {0, 1, 1};
    CHECK(storage_new_triangle_mesh(handle, cube_vertices.data(), 8, repeated.data(), 12) == nullptr);

    std::vector<c_vec3f> non_finite = cube_vertices;
    non_finite[3].y = std::numeric_limits<float>::quiet_NaN();
    CHECK(storage_new_triangle_mesh(handle, non_finite.data(), 8, cube_triangles.data(), 12) == nullptr);
    CHECK(storage.size() == 0);

    triangle_mesh_handle *empty = storage_new_triangle_mesh(handle, nullptr, 0, nullptr, 0);
    REQUIRE(empty != nullptr);
    CHECK(triangle_mesh_vertex_count(empty) == 0);
    CHECK(triangle_mesh_triangle_count(empty) == 0);
    CHECK(storage_free(handle, empty) == 1);
    CHECK(storage.size() == 0);

    triangle_mesh_handle *freed = storage_new_triangle_mesh(handle, cube_vertices.data(), 8,
                                                            cube_triangles.data(), 12);
    REQUIRE(freed != nullptr);
    CHECK(storage.contains(freed));
    CHECK(storage_free(handle, freed) == 1);
    CHECK_FALSE(storage.contains(freed));

    triangle_mesh_handle *cleared = storage_new_triangle_mesh(handle, cube_vertices.data(), 8,
                                                              cube_triangles.data(), 12);
    REQUIRE(cleared != nullptr);
    storage_clear(handle);
    CHECK_FALSE(storage.contains(cleared));
    CHECK(storage.size() == 0);

    CHECK_THROWS_AS(slic3r_api::StoredTriangleMesh(nullptr, cube_vertices, cube_triangles), std::invalid_argument);
    CHECK_THROWS_AS(slic3r_api::StoredTriangleMesh(handle, non_finite, cube_triangles), std::invalid_argument);
    CHECK(storage.size() == 0);
}

TEST_CASE("Storage-owned triangle mesh releases exactly once across scopes and moves",
          "[plugins][triangle-mesh][volume]")
{
    Slic3r::PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    {
        slic3r_api::StoredTriangleMesh mesh(handle, cube_vertices, cube_triangles);
        REQUIRE(storage.size() == 1);
    }
    CHECK(storage.size() == 0);

    {
        slic3r_api::StoredTriangleMesh source(handle, cube_vertices, cube_triangles);
        const triangle_mesh_handle *source_handle = source.handle();
        slic3r_api::StoredTriangleMesh moved(std::move(source));
        CHECK(moved.handle() == source_handle);
        CHECK_FALSE(source.valid());
        CHECK(storage.size() == 1);

        slic3r_api::StoredTriangleMesh target(handle, cube_vertices, cube_triangles);
        const triangle_mesh_handle *target_handle = target.handle();
        REQUIRE(storage.size() == 2);
        target = std::move(moved);
        CHECK(target.handle() == source_handle);
        CHECK_FALSE(moved.valid());
        CHECK_FALSE(storage.contains(const_cast<triangle_mesh_handle *>(target_handle)));
        CHECK(storage.size() == 1);
    }
    CHECK(storage.size() == 0);
}
