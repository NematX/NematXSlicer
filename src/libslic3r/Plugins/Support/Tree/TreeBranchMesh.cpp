///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#include "TreeBranchMesh.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/cpp/ParallelFor.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Api/plugin/cpp/VolumeViews.hpp"
#include "TreeGeometry.hpp"

/* Implementation of the organic branch meshes (see TreeBranchMesh.hpp).

   The paths of a tree are cut at its splits and tips (node_paths). Each path is extruded once into a mesh (extrude_branch):
   between two consecutive nodes the mesh is a strip of triangles joining the circles of the nodes, each circle turned by
   the bisector of the two stretches meeting at its node (path_ring_normals), and the two ends get a half sphere each. The mesh
   is held as plain arrays of the plugin API (BranchMesh) and handed to the host as a StoredTriangleMesh, which refuses a
   repeated vertex index. The mesh is positive, so its Positive slicing mode gives the solid section of the branch at each Z.
   slice_branch_meshes() slices every mesh at the Z values of the slabs it crosses, in one call per path.

   Every tree is one parallel job with its own scratch storage: the meshes and the raw slices live in it, and the non-empty
   sections move into the output storage (out_storage, owned by the caller) before the job ends. The sections of the trees are joined in tree order afterwards. */

namespace Slic3r::TreeFamily {
namespace {
// Sagitta of the circles of the tube, in millimeters: the discretization keeps every circle this close to the true one.
constexpr float tube_eps = 0.015f;

// The mesh of one path, in the arrays of the plugin API (millimeters).
struct BranchMesh {
    std::vector<c_vec3f> vertices;
    std::vector<c_triangle_indices> triangles;
};

// A section of one tree: the polygons (in out_storage) that its meshes give at the slice Z number z_index.
struct TreeSection {
    size_t z_index;
    slic3r_api::StoredPolygonCollection polygons;
};

// What the job of every tree reads, and where it leaves its sections (one vector per tree, written by the job of that tree only).
struct TreeSlicing {
    storage_handle *out_storage;
    const std::vector<RoutedTree> &trees;
    const std::vector<coord_t> &slice_z;
    const plugin_run_context *context;
    std::vector<std::vector<TreeSection>> sections;
};

// The job of one tree: slices each of its paths (the cancellation is checked before each path) and keeps the sections of the tree.
void slice_tree(uint32_t tree_idx, storage_handle *scratch, TreeSlicing *slicing);
// Extrudes one path into a mesh, slices it at the Z values it crosses in one call, and appends its non-empty sections to the tree's.
void slice_path(storage_handle *scratch, const TreeSlicing &slicing, const RoutedTree &tree, const std::vector<size_t> &path,
                std::vector<TreeSection> &tree_sections);
// The paths of one tree, each as node indices listed from the lower node up. A path starts at a node with no node below
// (a trunk bottom) or with several nodes above (a split), and runs up through single links until the next split or a tip.
std::vector<std::vector<size_t>> node_paths(const RoutedTree &tree);
// The ring normal of each node of a path: the direction of the first and last stretch, and at each node in between the
// bisector of its two stretches.
std::vector<Vec3d> path_ring_normals(const RoutedTree &tree, const std::vector<size_t> &path);
// The centre of a node in millimeters, at the print Z of its layer.
Vec3d node_centre(const TreeNode &node);
// Appends the tube of one path to result in millimeters, with a half sphere at each end; the rings of the nodes face the given
// normals. Returns the Z span of the mesh.
std::pair<float, float> extrude_branch(const RoutedTree &tree, const std::vector<size_t> &path, const std::vector<Vec3d> &normals,
                                       BranchMesh &result);
// Adds the triangles of the fan from vertex ifan to the closed ring of vertices [ibegin, iend).
template<bool flip_normals>
void triangulate_fan(BranchMesh &mesh, int ifan, int ibegin, int iend);
// Adds the zig-zag triangles between two rings of vertices, each ring closed on itself, taking the shorter diagonal first.
void triangulate_strip(BranchMesh &mesh, int ibegin1, int iend1, int ibegin2, int iend2);
// Appends a circle of the given radius around the centre, in the plane of the normal; returns the index range of its points.
std::pair<int, int> discretize_circle(const Vec3f &center, const Vec3f &normal, float radius, float eps, std::vector<c_vec3f> &pts);
// Conversions between the Eigen vectors of the computations and the vertices of the API.
Vec3f to_vec3f(const c_vec3f &vertex);
c_vec3f to_c_vec3f(const Vec3f &vertex);
// Appends the triangle (a, b, c) to the mesh.
void add_triangle(BranchMesh &mesh, int a, int b, int c);
// The vertex after i on the closed ring [ibegin, iend).
int next_in_ring(int i, int ibegin, int iend);
} // namespace

// Slices every path of every tree at the Z values it crosses; a path is sliced in one call for all of them.
std::vector<slic3r_api::StoredPolygonCollection> slice_branch_meshes(storage_handle *out_storage, const std::vector<RoutedTree> &trees,
                                                                     const std::vector<coord_t> &slice_z, const plugin_run_context *context)
{
    TreeSlicing slicing{ out_storage, trees, slice_z, context, std::vector<std::vector<TreeSection>>(trees.size()) };
    // Each tree is a separate job that draws its branch paths.
    slic3r_api::parallel_for_storage_with_progress(0, uint32_t(trees.size()), context, nullptr, &slice_tree, &slicing);

    // Joins the sections in tree order, then in path order: the same order as a serial loop over the trees.
    std::vector<slic3r_api::StoredPolygonCollection> sections;
    sections.reserve(slice_z.size());
    for (size_t idx = 0; idx < slice_z.size(); ++idx)
        sections.emplace_back(out_storage);
    for (std::vector<TreeSection> &tree_sections : slicing.sections)
        for (TreeSection &section : tree_sections)
            sections[section.z_index].append_move_from(section.polygons);
    return sections;
}

namespace {
void slice_tree(uint32_t tree_idx, storage_handle *scratch, TreeSlicing *slicing)
{
    const RoutedTree &tree = slicing->trees[tree_idx];
    for (const std::vector<size_t> &path : node_paths(tree)) {
        slic3r_api::throw_if_cancelled(slicing->context);
        slice_path(scratch, *slicing, tree, path, slicing->sections[tree_idx]);
    }
}

void slice_path(storage_handle *scratch, const TreeSlicing &slicing, const RoutedTree &tree, const std::vector<size_t> &path,
                std::vector<TreeSection> &tree_sections)
{
    BranchMesh mesh;
    const std::pair<float, float> span = extrude_branch(tree, path, path_ring_normals(tree, path), mesh);
    // The slice Z values inside the Z span of the mesh, with the index of the slice Z they come from.
    std::vector<std::pair<float, size_t>> cuts;
    for (size_t idx = 0; idx < slicing.slice_z.size(); ++idx) {
        const float z = float(unscaled(slicing.slice_z[idx]));
        if (z >= span.first && z <= span.second)
            cuts.emplace_back(z, idx);
    }
    if (cuts.empty())
        return;
    std::sort(cuts.begin(), cuts.end());
    std::vector<float> zs;
    zs.reserve(cuts.size());
    for (const std::pair<float, size_t> &cut : cuts)
        zs.push_back(cut.first);

    // The host refuses a mesh with a repeated vertex index (std::invalid_argument): the error stops the step, no branch is skipped.
    const slic3r_api::StoredTriangleMesh stored_mesh(scratch, mesh.vertices, mesh.triangles);
    c_mesh_slicing_params params = {};
    params.mode       = RAW_MESH_SLICING_MODE_POSITIVE;
    params.mode_below = RAW_MESH_SLICING_MODE_POSITIVE;
    params.transform  = matrix4d_identity();
    // The identity transform is required (a zeroed matrix would crush the mesh). mode_below is inert here (slicing_mode_normal_below_layer
    // stays 0); closing_radius, extra_offset and the resolutions are left 0 and have no effect on this path: the host slices with the
    // base MeshSlicingParams and gives the raw loops, as the legacy slice_mesh() did.
    std::vector<slic3r_api::StoredPolygonCollection> cut_sections = stored_mesh.slice_to_polygons(scratch, params, zs, slicing.context);
    for (size_t cut = 0; cut < cuts.size(); ++cut) {
        if (cut_sections[cut].empty())
            continue;
        tree_sections.push_back(TreeSection{ cuts[cut].second, slic3r_api::StoredPolygonCollection(slicing.out_storage) });
        tree_sections.back().polygons.append_move_from(cut_sections[cut]);
    }
}

std::vector<std::vector<size_t>> node_paths(const RoutedTree &tree)
{
    const std::vector<size_t> below = nodes_below(tree);
    std::vector<std::vector<size_t>> paths;
    for (size_t start = 0; start < tree.nodes.size(); ++start) {
        const std::vector<size_t> &carried = tree.nodes[start].carries;
        // A node with one node above and one below it lies inside the path that reaches it from below.
        if (carried.empty() || (carried.size() == 1 && below[start] != no_node))
            continue;
        for (const size_t first : carried) {
            std::vector<size_t> path{ start, first };
            while (tree.nodes[path.back()].carries.size() == 1)
                path.push_back(tree.nodes[path.back()].carries.front());
            paths.push_back(std::move(path));
        }
    }
    return paths;
}

std::vector<Vec3d> path_ring_normals(const RoutedTree &tree, const std::vector<size_t> &path)
{
    std::vector<Vec3d> stretches;
    for (size_t k = 1; k < path.size(); ++k)
        stretches.push_back((node_centre(tree.nodes[path[k]]) - node_centre(tree.nodes[path[k - 1]])).normalized());
    std::vector<Vec3d> normals{ stretches.front() };
    for (size_t k = 1; k < stretches.size(); ++k)
        normals.push_back((stretches[k - 1] + stretches[k]).normalized());
    normals.push_back(stretches.back());
    return normals;
}

Vec3d node_centre(const TreeNode &node)
{
    return Vec3d(unscaled(node.position.x), unscaled(node.position.y), node.print_z);
}

// The tube joins the rings of the nodes in path order. The first node gets a bottom half sphere and the last a top half sphere;
// the rings in between are the circles of the nodes, turned by their normals.
std::pair<float, float> extrude_branch(const RoutedTree &tree, const std::vector<size_t> &path, const std::vector<Vec3d> &normals,
                                       BranchMesh &result)
{
    assert(path.size() >= 2);
    std::pair<int, int> prev_strip;
    const size_t first_vertex = result.vertices.size();
    for (size_t ipath = 1; ipath < path.size(); ++ ipath) {
        const TreeNode &prev    = tree.nodes[path[ipath - 1]];
        const TreeNode &current = tree.nodes[path[ipath]];
        assert(prev.layer + 1 == current.layer);
        const Vec3d p1 = node_centre(prev);
        const Vec3d p2 = node_centre(current);
        const Vec3d &nprev = normals[ipath - 1];
        const Vec3d &ncurrent = normals[ipath];
        if (ipath == 1) {
            // Bottom half sphere: circles shrinking from the equator of the node down to its lowest point.
            const float radius = float(unscaled(prev.radius));
            float angle_step = float(2. * std::acos(1. - tube_eps / radius));
            const int nsteps = int(std::ceil(PI / (2. * angle_step)));
            angle_step = float(PI / (2. * nsteps));
            const int ifan = int(result.vertices.size());
            result.vertices.push_back(to_c_vec3f((p1 - nprev * radius).cast<float>()));
            float angle = angle_step;
            for (int i = 1; i < nsteps; ++ i, angle += angle_step) {
                const std::pair<int, int> strip = discretize_circle((p1 - nprev * radius * cos(angle)).cast<float>(), nprev.cast<float>(), radius * sin(angle), tube_eps, result.vertices);
                if (i == 1)
                    triangulate_fan<false>(result, ifan, strip.first, strip.second);
                else
                    triangulate_strip(result, prev_strip.first, prev_strip.second, strip.first, strip.second);
                prev_strip = strip;
            }
        }
        if (ipath + 1 == path.size()) {
            // Top half sphere: circles shrinking from the equator of the tip up to its highest point.
            const float radius = float(unscaled(current.radius));
            float angle_step = float(2. * std::acos(1. - tube_eps / radius));
            const int nsteps = int(std::ceil(PI / (2. * angle_step)));
            angle_step = float(PI / (2. * nsteps));
            float angle = float(PI / 2.);
            for (int i = 0; i < nsteps; ++ i, angle -= angle_step) {
                const std::pair<int, int> strip = discretize_circle((p2 + ncurrent * radius * cos(angle)).cast<float>(), ncurrent.cast<float>(), radius * sin(angle), tube_eps, result.vertices);
                triangulate_strip(result, prev_strip.first, prev_strip.second, strip.first, strip.second);
                prev_strip = strip;
            }
            const int ifan = int(result.vertices.size());
            result.vertices.push_back(to_c_vec3f((p2 + ncurrent * radius).cast<float>()));
            triangulate_fan<true>(result, ifan, prev_strip.first, prev_strip.second);
        } else {
            const float radius = float(unscaled(current.radius));
            const std::pair<int, int> strip = discretize_circle(p2.cast<float>(), ncurrent.cast<float>(), radius, tube_eps, result.vertices);
            triangulate_strip(result, prev_strip.first, prev_strip.second, strip.first, strip.second);
            prev_strip = strip;
        }
    }
    float zmin = std::numeric_limits<float>::max();
    float zmax = std::numeric_limits<float>::lowest();
    for (size_t idx = first_vertex; idx < result.vertices.size(); ++idx) {
        zmin = std::min(zmin, result.vertices[idx].z);
        zmax = std::max(zmax, result.vertices[idx].z);
    }
    return { zmin, zmax };
}

template<bool flip_normals>
void triangulate_fan(BranchMesh &mesh, int ifan, int ibegin, int iend)
{
    // at least 3 vertices, increasing order.
    assert(ibegin + 3 <= iend);
    assert(ibegin >= 0 && iend <= int(mesh.vertices.size()));
    assert(ifan >= 0 && ifan < int(mesh.vertices.size()));
    int num_faces = iend - ibegin;
    mesh.triangles.reserve(mesh.triangles.size() + num_faces * 3);
    for (int v = ibegin, u = iend - 1; v < iend; u = v ++) {
        if (flip_normals)
            add_triangle(mesh, ifan, u, v);
        else
            add_triangle(mesh, ifan, v, u);
    }
}

void triangulate_strip(BranchMesh &mesh, int ibegin1, int iend1, int ibegin2, int iend2)
{
    // at least 3 vertices, increasing order.
    assert(ibegin1 + 3 <= iend1);
    assert(ibegin1 >= 0 && iend1 <= int(mesh.vertices.size()));
    assert(ibegin2 + 3 <= iend2);
    assert(ibegin2 >= 0 && iend2 <= int(mesh.vertices.size()));
    int n1 = iend1 - ibegin1;
    int n2 = iend2 - ibegin2;
    mesh.triangles.reserve(mesh.triangles.size() + (n1 + n2) * 3);

    // For the first vertex of 1st strip, find the closest vertex on the 2nd strip.
    int istart2 = ibegin2;
    {
        const Vec3f p1    = to_vec3f(mesh.vertices[ibegin1]);
        float       d2min = std::numeric_limits<float>::max();
        for (int i = ibegin2; i < iend2; ++ i) {
            const Vec3f p2 = to_vec3f(mesh.vertices[i]);
            const float d2 = (p2 - p1).squaredNorm();
            if (d2 < d2min) {
                d2min = d2;
                istart2 = i;
            }
        }
    }

    // Now triangulate the strip zig-zag fashion taking always the shortest connection if possible.
    for (int u = ibegin1, v = istart2; n1 > 0 || n2 > 0;) {
        bool take_first;
        int u2, v2;
        // u2 is the vertex after u on the first ring, v2 the vertex after v on the second ring (each ring is closed on itself).
        u2 = next_in_ring(u, ibegin1, iend1);
        v2 = next_in_ring(v, ibegin2, iend2);
        if (n1 == 0) {
            take_first = false;
        } else if (n2 == 0) {
            take_first = true;
        } else {
            float l1 = (to_vec3f(mesh.vertices[u2]) - to_vec3f(mesh.vertices[v])).squaredNorm();
            float l2 = (to_vec3f(mesh.vertices[v2]) - to_vec3f(mesh.vertices[u])).squaredNorm();
            take_first = l1 < l2;
        }
        if (take_first) {
            add_triangle(mesh, u, u2, v);
            -- n1;
            u = u2;
        } else {
            add_triangle(mesh, u, v2, v);
            -- n2;
            v = v2;
        }
    }
}

// Discretize 3D circle, append to output vector, return ranges of indices of the points added.
std::pair<int, int> discretize_circle(const Vec3f &center, const Vec3f &normal, const float radius, const float eps, std::vector<c_vec3f> &pts)
{
    // Calculate discretization step and number of steps.
    float angle_step = 2. * acos(1. - eps / radius);
    int   nsteps     = int(std::ceil(2 * PI / angle_step));
    angle_step = 2 * PI / nsteps;

    // Prepare coordinate system for the circle plane.
    Vec3f x = normal.cross(Vec3f(0.f, -1.f, 0.f)).normalized();
    Vec3f y = normal.cross(x).normalized();
    assert(std::abs(x.cross(y).dot(normal) - 1.f) < EPSILON);

    // Discretize the circle.
    int begin = int(pts.size());
    pts.reserve(pts.size() + nsteps);
    float angle = 0;
    x *= radius;
    y *= radius;
    for (int i = 0; i < nsteps; ++ i) {
        pts.push_back(to_c_vec3f(center + x * cos(angle) + y * sin(angle)));
        angle += angle_step;
    }
    return { begin, int(pts.size()) };
}

Vec3f to_vec3f(const c_vec3f &vertex)
{
    return Vec3f(vertex.x, vertex.y, vertex.z);
}

c_vec3f to_c_vec3f(const Vec3f &vertex)
{
    return c_vec3f{ vertex.x(), vertex.y(), vertex.z() };
}

void add_triangle(BranchMesh &mesh, int a, int b, int c)
{
    mesh.triangles.push_back(c_triangle_indices{ uint32_t(a), uint32_t(b), uint32_t(c) });
}

int next_in_ring(int i, int ibegin, int iend)
{
    return i + 1 == iend ? ibegin : i + 1;
}
} // namespace
} // namespace Slic3r::TreeFamily
