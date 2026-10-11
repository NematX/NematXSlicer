///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#include "TreeBranchMesh.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/TriangleMeshSlicer.hpp"

/* Implementation of the organic branch meshes (see TreeBranchMesh.hpp).

   The paths of a tree are cut at its splits and tips (node_paths). Each path is extruded once into a mesh (extrude_branch):
   between two consecutive nodes the mesh is a strip of triangles joining the circles of the nodes, each circle turned by
   the bisector of the two stretches meeting at its node (path_ring_normals), and the two ends get a half sphere each. The mesh is positive, so its
   Positive slicing mode gives the solid section of the branch at each Z. slice_branch_meshes() slices every mesh at the
   Z values of the slabs it crosses, in one call per path. */

namespace Slic3r::TreeFamily {
namespace {
// Sagitta of the circles of the tube, in millimeters: the discretization keeps every circle this close to the true one.
constexpr float tube_eps = 0.015f;

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
                                       indexed_triangle_set &result);
// Adds the triangles of the fan from vertex ifan to the closed ring of vertices [ibegin, iend).
template<bool flip_normals>
void triangulate_fan(indexed_triangle_set &its, int ifan, int ibegin, int iend);
// Adds the zig-zag triangles between two rings of vertices, each ring closed on itself, taking the shorter diagonal first.
void triangulate_strip(indexed_triangle_set &its, int ibegin1, int iend1, int ibegin2, int iend2);
// Appends a circle of the given radius around the centre, in the plane of the normal; returns the index range of its points.
std::pair<int, int> discretize_circle(const Vec3f &center, const Vec3f &normal, float radius, float eps, std::vector<Vec3f> &pts);
} // namespace

// Slices every path of every tree at the Z values it crosses; a path is sliced in one call for all of them.
std::vector<Polygons> slice_branch_meshes(const std::vector<RoutedTree> &trees, const std::vector<coord_t> &slice_z,
                                          const std::function<void()> &throw_on_cancel)
{
    std::vector<Polygons> sections(slice_z.size());
    MeshSlicingParams params;
    params.mode = MeshSlicingParams::SlicingMode::Positive;
    for (const RoutedTree &tree : trees)
        for (const std::vector<size_t> &path : node_paths(tree)) {
            indexed_triangle_set mesh;
            const std::pair<float, float> span = extrude_branch(tree, path, path_ring_normals(tree, path), mesh);
            std::vector<std::pair<float, size_t>> cuts;
            for (size_t idx = 0; idx < slice_z.size(); ++idx) {
                const float z = float(unscaled(slice_z[idx]));
                if (z >= span.first && z <= span.second)
                    cuts.emplace_back(z, idx);
            }
            if (cuts.empty())
                continue;
            std::sort(cuts.begin(), cuts.end());
            std::vector<float> zs;
            zs.reserve(cuts.size());
            for (const std::pair<float, size_t> &cut : cuts)
                zs.push_back(cut.first);
            const std::vector<Polygons> cut_sections = slice_mesh(mesh, zs, params, throw_on_cancel);
            for (size_t cut = 0; cut < cuts.size(); ++cut) {
                Polygons &section = sections[cuts[cut].second];
                section.insert(section.end(), cut_sections[cut].begin(), cut_sections[cut].end());
            }
        }
    return sections;
}

namespace {
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
    return Vec3d(unscaled(node.position.x()), unscaled(node.position.y()), node.print_z);
}

// The tube joins the rings of the nodes in path order. The first node gets a bottom half sphere and the last a top half sphere;
// the rings in between are the circles of the nodes, turned by their normals.
std::pair<float, float> extrude_branch(const RoutedTree &tree, const std::vector<size_t> &path, const std::vector<Vec3d> &normals,
                                       indexed_triangle_set &result)
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
            const int nsteps = int(std::ceil(M_PI / (2. * angle_step)));
            angle_step = float(M_PI / (2. * nsteps));
            const int ifan = int(result.vertices.size());
            result.vertices.emplace_back((p1 - nprev * radius).cast<float>());
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
            const int nsteps = int(std::ceil(M_PI / (2. * angle_step)));
            angle_step = float(M_PI / (2. * nsteps));
            float angle = float(M_PI / 2.);
            for (int i = 0; i < nsteps; ++ i, angle -= angle_step) {
                const std::pair<int, int> strip = discretize_circle((p2 + ncurrent * radius * cos(angle)).cast<float>(), ncurrent.cast<float>(), radius * sin(angle), tube_eps, result.vertices);
                triangulate_strip(result, prev_strip.first, prev_strip.second, strip.first, strip.second);
                prev_strip = strip;
            }
            const int ifan = int(result.vertices.size());
            result.vertices.emplace_back((p2 + ncurrent * radius).cast<float>());
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
        zmin = std::min(zmin, result.vertices[idx].z());
        zmax = std::max(zmax, result.vertices[idx].z());
    }
    return { zmin, zmax };
}

template<bool flip_normals>
void triangulate_fan(indexed_triangle_set &its, int ifan, int ibegin, int iend)
{
    // at least 3 vertices, increasing order.
    assert(ibegin + 3 <= iend);
    assert(ibegin >= 0 && iend <= its.vertices.size());
    assert(ifan >= 0 && ifan < its.vertices.size());
    int num_faces = iend - ibegin;
    its.indices.reserve(its.indices.size() + num_faces * 3);
    for (int v = ibegin, u = iend - 1; v < iend; u = v ++) {
        if (flip_normals)
            its.indices.push_back({ ifan, u, v });
        else
            its.indices.push_back({ ifan, v, u });
    }
}

void triangulate_strip(indexed_triangle_set &its, int ibegin1, int iend1, int ibegin2, int iend2)
{
    // at least 3 vertices, increasing order.
    assert(ibegin1 + 3 <= iend1);
    assert(ibegin1 >= 0 && iend1 <= its.vertices.size());
    assert(ibegin2 + 3 <= iend2);
    assert(ibegin2 >= 0 && iend2 <= its.vertices.size());
    int n1 = iend1 - ibegin1;
    int n2 = iend2 - ibegin2;
    its.indices.reserve(its.indices.size() + (n1 + n2) * 3);

    // For the first vertex of 1st strip, find the closest vertex on the 2nd strip.
    int istart2 = ibegin2;
    {
        const Vec3f &p1    = its.vertices[ibegin1];
        float        d2min = std::numeric_limits<float>::max();
        for (int i = ibegin2; i < iend2; ++ i) {
            const Vec3f &p2 = its.vertices[i];
            const float d2  = (p2 - p1).squaredNorm();
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
        auto update_u2 = [&u2, u, ibegin1, iend1]() {
            u2 = u;
            if (++ u2 == iend1)
                u2 = ibegin1;
        };
        auto update_v2 = [&v2, v, ibegin2, iend2]() {
            v2 = v;
            if (++ v2 == iend2)
                v2 = ibegin2;
        };
        if (n1 == 0) {
            take_first = false;
            update_v2();
        } else if (n2 == 0) {
            take_first = true;
            update_u2();
        } else {
            update_u2();
            update_v2();
            float l1 = (its.vertices[u2] - its.vertices[v]).squaredNorm();
            float l2 = (its.vertices[v2] - its.vertices[u]).squaredNorm();
            take_first = l1 < l2;
        }
        if (take_first) {
            its.indices.push_back({ u, u2, v });
            -- n1;
            u = u2;
        } else {
            its.indices.push_back({ u, v2, v });
            -- n2;
            v = v2;
        }
    }
}

// Discretize 3D circle, append to output vector, return ranges of indices of the points added.
std::pair<int, int> discretize_circle(const Vec3f &center, const Vec3f &normal, const float radius, const float eps, std::vector<Vec3f> &pts)
{
    // Calculate discretization step and number of steps.
    float angle_step = 2. * acos(1. - eps / radius);
    int   nsteps     = int(std::ceil(2 * M_PI / angle_step));
    angle_step = 2 * M_PI / nsteps;

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
        pts.emplace_back(center + x * cos(angle) + y * sin(angle));
        angle += angle_step;
    }
    return { begin, int(pts.size()) };
}
} // namespace
} // namespace Slic3r::TreeFamily
