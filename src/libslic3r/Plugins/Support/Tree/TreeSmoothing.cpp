///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#include "TreeSmoothing.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <numeric>
#include <utility>
#include <vector>

#include <oneapi/tbb/parallel_for.h>

#include "libslic3r/AABBTreeIndirect.hpp"
#include "libslic3r/AABBTreeLines.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Line.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/PointUtils.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/PrintObject.hpp"

/* Implementation of the organic branch smoothing (see TreeSmoothing.hpp).

   Every node becomes a sphere of the relaxation: its circle in one layer, its radius, and its links to the spheres above
   it and to the sphere below it. The steps of smooth_organic_branches() are:
   1. make_spheres() creates one sphere per node, with the links of its tree;
   2. set_layer_spans() limits the layers that each sphere can reach, from the bottom of the branch below it up to the
      lowest tip above it, and from its own radius;
   3. layer_collisions() indexes the outline of every layer as lines, in millimeters;
   4. relax() repeats, for every free sphere, a push out of the outline and a pull toward the links. The push measures the
      overlap at the start of the iteration, before the pull of that iteration moves the sphere, so the last pull is not
      measured and a free sphere can end slightly inside the outline (see settled_displacement). The tips and the roots that
      rest on the model are locked and never move. The relaxation stops when no free sphere overlapped in an iteration and no
      node moved more than settled_displacement in it, or after max_iterations iterations;
   5. write_back() stores the positions of the free spheres in the nodes; the other nodes keep their positions.
   The constants are the legacy ones, except settled_displacement; the layer heights are the real heights of the object layers. */

namespace Slic3r::TreeFamily {
namespace {
// The legacy parameters, in millimeters: the gap kept after a push out of the outline, the largest push out, and the
// largest pull toward the links in one iteration.
constexpr double collision_extra_gap = 0.1;
constexpr double max_nudge_collision_avoidance = 0.5;
constexpr double max_nudge_smoothing = 0.2;
// Weight of the mean of the links in the pull: half of the way toward it.
constexpr double smoothing_factor = 0.5;
constexpr size_t max_iterations = 100;
// Stop threshold of the relaxation, in millimeters: an iteration in which no node moves more than this, and no free sphere
// overlaps the outline, ends it. It is a local rule on the motion of the nodes, not a bound on the bend that remains in the
// branch. Known limits of the rule: a long, gently curved branch can stop after one iteration, because each node moves far less
// than this threshold when its local curvature is small (the rule removes kinks, not gentle arcs); and the last pull is not
// measured against the outline, so the circle of a free sphere can reach up to this distance into the keep-out (the legacy stop
// has the same gap). The drawing trims the slabs against the keep-out afterwards.
constexpr double settled_displacement = 0.01;

// The outline of one layer as lines in millimeters, indexed for the nearest-line queries.
struct LayerCollision {
    std::vector<Linef> lines;
    AABBTreeIndirect::Tree<2, double> tree;
};

// A node as a sphere of the relaxation: its layer, its radius and position in millimeters, the spheres it is linked to
// (global indices), and the layers that its circle can reach. The spheres of the tips and of the roots that rest on the
// model are locked and do not move.
struct Sphere {
    size_t layer = 0;
    double radius = 0.;
    Vec2d position = Vec2d::Zero();
    Vec2d prev_position = Vec2d::Zero();
    Vec2d last_collision = Vec2d::Zero();
    double last_collision_depth = 0.;
    double min_z = 0., max_z = 0.;
    size_t layer_begin = 0, layer_end = 0;
    size_t below = no_node;
    std::vector<size_t> above;
    bool locked = false;
};

// The print Z of every layer of the object, in millimeters.
std::vector<double> layer_print_zs(const PrintObject &object);
// One sphere per node of every tree, the nodes of a tree following the nodes of the tree before it.
std::vector<Sphere> make_spheres(const std::vector<RoutedTree> &trees, const std::vector<double> &print_z);
// The Z span and the layers of every sphere, see the file header.
void set_layer_spans(std::vector<Sphere> &spheres, const std::vector<double> &print_z);
// The outline of every layer, as lines indexed by an AABB tree.
std::vector<LayerCollision> layer_collisions(const std::vector<Polygons> &keep_out);
// Runs the iterations of the relaxation, at most max_iterations, and stops once no free sphere overlapped in an iteration and no
// node moved more than settled_displacement in it. Returns the number of iterations that ran.
size_t relax(std::vector<Sphere> &spheres, const std::vector<double> &print_z, const std::vector<LayerCollision> &collisions,
             const std::function<void()> &throw_on_cancel);
// Pushes the sphere away from the closest outline point of the layers it reaches; returns true when it overlapped by more than EPSILON.
bool push_out_of_outline(Sphere &sphere, const std::vector<double> &print_z, const std::vector<LayerCollision> &collisions);
// Moves the sphere half of the way toward the radius-weighted mean of its links, by at most max_nudge_smoothing.
void pull_toward_links(Sphere &sphere, const std::vector<Sphere> &spheres);
// Stores the positions of the free spheres in the nodes of the trees, in the order that make_spheres() used.
void write_back(const std::vector<Sphere> &spheres, std::vector<RoutedTree> &trees);
} // namespace

// Relaxes the free nodes of the trees and returns the number of iterations that ran. The spheres are built from the trees,
// relaxed, and only then written back, so an exception from throw_on_cancel leaves the trees unchanged.
size_t smooth_organic_branches(const PrintObject &object, const std::vector<Polygons> &keep_out, std::vector<RoutedTree> &trees,
                               const std::function<void()> &throw_on_cancel)
{
    const std::vector<double> print_z = layer_print_zs(object);
    std::vector<Sphere> spheres = make_spheres(trees, print_z);
    set_layer_spans(spheres, print_z);
    const std::vector<LayerCollision> collisions = layer_collisions(keep_out);
    const size_t iterations = relax(spheres, print_z, collisions, throw_on_cancel);
    write_back(spheres, trees);
    return iterations;
}

// Builds a sphere reaching only its own layer, runs the shared push and returns its center; layer indexes print_z and keep_out.
Vec2d push_sphere_out_of_outline(size_t layer, double radius, const Vec2d &position, const std::vector<double> &print_z,
                                 const std::vector<Polygons> &keep_out)
{
    Sphere sphere;
    sphere.layer = layer;
    sphere.radius = radius;
    sphere.position = position;
    sphere.layer_begin = layer;
    sphere.layer_end = layer + 1;
    push_out_of_outline(sphere, print_z, layer_collisions(keep_out));
    return sphere.position;
}

namespace {
std::vector<double> layer_print_zs(const PrintObject &object)
{
    std::vector<double> print_z;
    print_z.reserve(object.layer_count());
    for (size_t id = 0; id < object.layer_count(); ++id)
        print_z.push_back(object.layer(id).unscaled_print_z());
    return print_z;
}

std::vector<Sphere> make_spheres(const std::vector<RoutedTree> &trees, const std::vector<double> &print_z)
{
    std::vector<Sphere> spheres;
    for (const RoutedTree &tree : trees) {
        const size_t offset = spheres.size();
        const std::vector<size_t> below = nodes_below(tree);
        for (size_t idx = 0; idx < tree.nodes.size(); ++idx) {
            const TreeNode &node = tree.nodes[idx];
            Sphere sphere;
            sphere.layer = node.layer;
            sphere.radius = unscaled(node.radius);
            sphere.position = Vec2d(unscaled(node.position.x), unscaled(node.position.y));
            sphere.below = below[idx] == no_node ? no_node : offset + below[idx];
            for (const size_t upper : node.carries)
                sphere.above.push_back(offset + upper);
            sphere.locked = node.carries.empty() || (below[idx] == no_node && node.layer > 0);
            spheres.push_back(std::move(sphere));
        }
    }
    return spheres;
}

// The bottom of a sphere is the bottom of the branch below it, so the bottoms are set from the lowest layer up; the top is
// the lowest tip above it, so the tops are set from the highest layer down.
void set_layer_spans(std::vector<Sphere> &spheres, const std::vector<double> &print_z)
{
    std::vector<size_t> by_layer(spheres.size());
    std::iota(by_layer.begin(), by_layer.end(), size_t(0));
    std::stable_sort(by_layer.begin(), by_layer.end(), [&spheres](const size_t a, const size_t b) { return spheres[a].layer < spheres[b].layer; });
    for (const size_t id : by_layer) {
        Sphere &sphere = spheres[id];
        sphere.min_z = sphere.below == no_node ? print_z[sphere.layer] : spheres[sphere.below].min_z;
    }
    for (size_t pos = by_layer.size(); pos-- > 0;) {
        Sphere &sphere = spheres[by_layer[pos]];
        sphere.max_z = sphere.above.empty() ? print_z[sphere.layer] : std::numeric_limits<double>::max();
        for (const size_t above : sphere.above)
            sphere.max_z = std::min(sphere.max_z, spheres[above].max_z);
    }
    for (Sphere &sphere : spheres) {
        const double z = print_z[sphere.layer];
        sphere.min_z = std::max(sphere.min_z, z - sphere.radius);
        sphere.max_z = std::min(sphere.max_z, z + sphere.radius);
        const size_t bottom = size_t(std::lower_bound(print_z.begin(), print_z.end(), sphere.min_z) - print_z.begin());
        const size_t top = size_t(std::upper_bound(print_z.begin(), print_z.end(), sphere.max_z) - print_z.begin());
        sphere.layer_begin = std::min(sphere.layer, bottom);
        sphere.layer_end = std::min(print_z.size(), std::max(sphere.layer + 1, top));
    }
}

std::vector<LayerCollision> layer_collisions(const std::vector<Polygons> &keep_out)
{
    std::vector<LayerCollision> collisions(keep_out.size());
    for (size_t layer = 0; layer < keep_out.size(); ++layer) {
        LayerCollision &collision = collisions[layer];
        for (const Line &line : to_lines(keep_out[layer]))
            collision.lines.push_back({ unscale_p(line.a), unscale_p(line.b) });
        collision.tree = AABBTreeLines::build_aabb_tree_over_indexed_lines(collision.lines);
    }
    return collisions;
}

// Each sphere's push and pull read only its own position, the outlines and the previous positions of its links, so the spheres
// of an iteration are moved in parallel. The iterations go on while a free sphere overlaps the outline or a node moves more than
// settled_displacement in an iteration, and stop after max_iterations. The largest move is gathered serially after the parallel
// pass.
size_t relax(std::vector<Sphere> &spheres, const std::vector<double> &print_z, const std::vector<LayerCollision> &collisions,
             const std::function<void()> &throw_on_cancel)
{
    for (size_t iteration = 0; iteration < max_iterations; ++iteration) {
        for (Sphere &sphere : spheres)
            sphere.prev_position = sphere.position;
        std::atomic<size_t> num_overlapping{ 0 };
        tbb::parallel_for(tbb::blocked_range<size_t>(0, spheres.size()), [&](const tbb::blocked_range<size_t> &range) {
            for (size_t id = range.begin(); id < range.end(); ++id)
                if (!spheres[id].locked) {
                    if (push_out_of_outline(spheres[id], print_z, collisions))
                        ++num_overlapping;
                    pull_toward_links(spheres[id], spheres);
                }
        });
        throw_on_cancel();
        double largest_move = 0.;
        for (const Sphere &sphere : spheres)
            largest_move = std::max(largest_move, (sphere.position - sphere.prev_position).norm());
        if (num_overlapping == 0 && largest_move <= settled_displacement)
            return iteration + 1;
    }
    return max_iterations;
}

bool push_out_of_outline(Sphere &sphere, const std::vector<double> &print_z, const std::vector<LayerCollision> &collisions)
{
    sphere.last_collision_depth = -std::numeric_limits<double>::max();
    for (size_t layer = sphere.layer_begin; layer < sphere.layer_end; ++layer) {
        const double dz = print_z[layer] - print_z[sphere.layer];
        const double r2 = sqr(sphere.radius) - sqr(dz);
        const LayerCollision &collision = collisions[layer];
        if (r2 <= 0. || collision.lines.empty())
            continue;
        size_t hit_idx = 0;
        Vec2d hit_point;
        const double squared_distance = AABBTreeLines::squared_distance_to_indexed_lines(collision.lines, collision.tree, sphere.position, hit_idx, hit_point, r2);
        if (squared_distance >= 0.) {
            const double depth = std::sqrt(r2) - std::sqrt(squared_distance);
            if (depth > sphere.last_collision_depth) {
                sphere.last_collision_depth = depth;
                sphere.last_collision = hit_point;
            }
        }
    }
    if (sphere.last_collision_depth <= 0.)
        return false;
    // nudge_dist is the penetration plus the gap, capped. nudge_vector already has that length, so adding it once is the whole shift.
    const double nudge_dist = std::min(std::max(0., sphere.last_collision_depth + collision_extra_gap), max_nudge_collision_avoidance);
    const Vec2d nudge_vector = (sphere.position - sphere.last_collision).normalized() * nudge_dist;
    sphere.position += nudge_vector;
    return sphere.last_collision_depth > EPSILON;
}

void pull_toward_links(Sphere &sphere, const std::vector<Sphere> &spheres)
{
    Vec2d mean = Vec2d::Zero();
    double weight = 0.;
    for (const size_t above : sphere.above) {
        mean += sphere.radius * spheres[above].prev_position;
        weight += sphere.radius;
    }
    if (sphere.below != no_node) {
        const double below_weight = weight;
        mean += below_weight * spheres[sphere.below].prev_position;
        weight += below_weight;
    }
    mean /= weight;
    const Vec2d old_position = sphere.position;
    const Vec2d new_position = (1. - smoothing_factor) * old_position + smoothing_factor * mean;
    const Vec2d shift = new_position - old_position;
    sphere.position += shift.normalized() * std::min(shift.norm(), max_nudge_smoothing);
}

void write_back(const std::vector<Sphere> &spheres, std::vector<RoutedTree> &trees)
{
    size_t id = 0;
    for (RoutedTree &tree : trees)
        for (TreeNode &node : tree.nodes) {
            if (!spheres[id].locked)
                {
                    // The sphere is in millimeters and native; the node holds an API point.
                    const Point scaled = Point::new_scale(spheres[id].position);
                    node.position = c_point{ scaled.x(), scaled.y() };
                }
            ++id;
        }
}
} // namespace
} // namespace Slic3r::TreeFamily
