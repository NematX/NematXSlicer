///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#include "TreeSmoothing.hpp"

#include <algorithm>
#include <atomic>
#include <cassert>
#include <cmath>
#include <limits>
#include <numeric>
#include <optional>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_utils.h"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/DeferredCallbackException.hpp"
#include "libslic3r/Api/plugin/cpp/LineDistancer.hpp"

/* Implementation of the organic branch smoothing (see TreeSmoothing.hpp).

   Every node becomes a sphere of the relaxation: its circle in one layer, its radius, and its links to the spheres above
   it and to the sphere below it. The steps of smooth_organic_branches() are:
   1. make_spheres() creates one sphere per node, with the links of its tree;
   2. set_layer_spans() limits the layers that each sphere can reach, from the bottom of the branch below it up to the
      lowest tip above it, and from its own radius;
   3. layer_collisions() indexes the outline of every layer as lines, in scaled units (the units of the stored outlines);
   4. relax() repeats, for every free sphere, a push out of the outline and a pull toward the links. The pull aims at the point
       of the straight line between the sphere below and the mean of the spheres above that lies at the Z of the sphere, so a
       straight inclined branch stays straight on unequal layer heights. The push measures the
      overlap at the start of the iteration, before the pull of that iteration moves the sphere, so the last pull is not
      measured and a free sphere can end slightly inside the outline (see settled_displacement). The tips and the roots that
      rest on the model are locked and never move. The relaxation stops when no free sphere overlapped in an iteration and no
      node moved more than settled_displacement in it, or after max_iterations iterations;
   5. a final pass pushes every free sphere that still overlaps the outline out of it, once, with the same push as the
      iterations (no pull follows it); the context is checked after it, before anything is written back;
   6. write_back() stores the positions of the free spheres in the nodes; the other nodes keep their positions.
   The spheres work in millimeters; the outline queries scale the sphere position to the units of the lines and unscale the
   answer. The relaxation of an iteration runs on chunks of spheres, one chunk being one job of slic3r_parallel_for.
   The constants are the legacy ones, except settled_displacement; the layer heights are the real heights of the object layers. */

namespace Slic3r::TreeFamily {
namespace {
// The legacy parameters, in millimeters: the gap kept after a push out of the outline, the largest push out, and the
// largest pull toward the links in one iteration.
constexpr double collision_extra_gap = 0.1;
constexpr double max_nudge_collision_avoidance = 0.5;
constexpr double max_nudge_smoothing = 0.2;
// Weight of the target of the links in the pull: half of the way toward it.
constexpr double smoothing_factor = 0.5;
constexpr size_t max_iterations = 100;
// Stop threshold of the relaxation, in millimeters: an iteration in which no node moves more than this, and no free sphere
// overlaps the outline, ends it. It is a local rule on the motion of the nodes, not a bound on the bend that remains in the
// branch. Known limits of the rule: a long, gently curved branch can stop after one iteration, because each node moves far less
// than this threshold when its local curvature is small (the rule removes kinks, not gentle arcs); and the last pull is not
// measured against the outline, so the circle of a free sphere can reach up to this distance into the keep-out (the legacy stop
// has the same gap); the final pass of smooth_organic_branches() pushes such a sphere out.
constexpr double settled_displacement = 0.01;
// Number of spheres moved by one job of the relaxation. The relaxation time was measured flat from 1 to 64 spheres per job (5000
// spheres, 16 threads) and only degrades above that, when there are fewer jobs than threads; 16 is a middle value, with no gain claimed.
constexpr size_t spheres_per_job = 16;

// The outline of one layer as lines in scaled units, and an index over the same lines that finds the lines near a point.
struct LayerCollision {
    std::vector<LineSegment> lines;
    slic3r_api::LineDistancer index;
};

// A node as a sphere of the relaxation: its layer, its radius and position in millimeters, the spheres it is linked to
// (global indices), and the layers that its circle can reach. The spheres of the tips and of the roots that rest on the
// model are locked and do not move.
struct Sphere {
    size_t layer = 0;
    // The print Z of the layer, in scaled integer units (the exact value of the object layer, so Z differences between layers are exact).
    coord_t z = 0;
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
std::vector<double> layer_print_zs(const slic3r_api::Object &object);
// One sphere per node of every tree, the nodes of a tree following the nodes of the tree before it.
std::vector<Sphere> make_spheres(const std::vector<RoutedTree> &trees, const std::vector<double> &print_z);
// The Z span and the layers of every sphere, see the file header.
void set_layer_spans(std::vector<Sphere> &spheres, const std::vector<double> &print_z);
// The outline of every layer, as lines with a spatial index.
std::vector<LayerCollision> layer_collisions(const std::vector<slic3r_api::StoredPolygonCollection> &keep_out);
// What the jobs of one iteration share: the spheres they move, the outlines they read and the count of overlapping spheres.
struct RelaxPass {
    std::vector<Sphere> *spheres;
    const std::vector<double> *print_z;
    const std::vector<LayerCollision> *collisions;
    std::atomic<size_t> *num_overlapping;
    // First exception of a job (an allocation can fail); the jobs after it return at once and relax() rethrows it after the loop.
    slic3r_api::DeferredCallbackException failure;
};
// Runs the iterations of the relaxation, at most max_iterations, and stops once no free sphere overlapped in an iteration and no
// node moved more than settled_displacement in it. Returns the number of iterations that ran.
size_t relax(std::vector<Sphere> &spheres, const std::vector<double> &print_z, const std::vector<LayerCollision> &collisions,
             const plugin_run_context *context);
// The job of slic3r_parallel_for: pushes and pulls the free spheres of chunk number `job` (spheres_per_job spheres from job * spheres_per_job).
void relax_chunk(uint32_t job, void *user_data);
// Pushes the sphere away from the closest outline point of the layers it reaches; returns true when it overlapped by more than EPSILON.
bool push_out_of_outline(Sphere &sphere, const std::vector<double> &print_z, const std::vector<LayerCollision> &collisions);
// Moves the sphere half of the way toward the point of the straight line between the sphere below and the mean of the spheres
// above that lies at the Z of the sphere (the midpoint when the layer heights are equal), by at most max_nudge_smoothing.
void pull_toward_links(Sphere &sphere, const std::vector<Sphere> &spheres);
// Stores the positions of the free spheres in the nodes of the trees, in the order that make_spheres() used.
void write_back(const std::vector<Sphere> &spheres, std::vector<RoutedTree> &trees);
} // namespace

// Relaxes the free nodes of the trees and returns the number of iterations that ran. The spheres are built from the trees,
// relaxed, pushed out of the outline once more, and only then written back. The context is checked after every iteration and after
// the final pass, so a cancellation leaves the trees unchanged.
size_t smooth_organic_branches(const slic3r_api::Object &object, const std::vector<slic3r_api::StoredPolygonCollection> &keep_out,
                               std::vector<RoutedTree> &trees, const plugin_run_context *context)
{
    const std::vector<double> print_z = layer_print_zs(object);
    std::vector<Sphere> spheres = make_spheres(trees, print_z);
    set_layer_spans(spheres, print_z);
    const std::vector<LayerCollision> collisions = layer_collisions(keep_out);
    const size_t iterations = relax(spheres, print_z, collisions, context);
    // Final pass: the last pull of the relaxation was not measured against the outline, so a free sphere can still overlap it. Push it
    // out once. TODO: a push is capped (max_nudge_collision_avoidance) and can leave the circle in another part of the outline (a
    // concave corner, another layer of its window); such a sphere is left as it is. The printed geometry is still trimmed by the
    // object outline grown by the XY distance (trim_slab).
    for (Sphere &sphere : spheres)
        if (! sphere.locked)
            push_out_of_outline(sphere, print_z, collisions);
    slic3r_api::throw_if_cancelled(context);
    write_back(spheres, trees);
    return iterations;
}

// Builds a sphere reaching only its own layer, runs the shared push and returns its center; layer indexes print_z and keep_out.
Vec2d push_sphere_out_of_outline(size_t layer, double radius, const Vec2d &position, const std::vector<double> &print_z,
                                 const std::vector<slic3r_api::StoredPolygonCollection> &keep_out)
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
std::vector<double> layer_print_zs(const slic3r_api::Object &object)
{
    std::vector<double> print_z;
    print_z.reserve(object.layer_count());
    for (uint32_t id = 0; id < object.layer_count(); ++id)
        print_z.push_back(unscaled(object.layer(id).print_z()));
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
            sphere.z = scale_to_layer_coord(print_z[node.layer]);
            sphere.radius = unscaled(node.radius);
            sphere.position = unscale_p(node.position);
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

// The index is built before the relaxation starts, so that its const queries can run from several threads.
std::vector<LayerCollision> layer_collisions(const std::vector<slic3r_api::StoredPolygonCollection> &keep_out)
{
    std::vector<LayerCollision> collisions(keep_out.size());
    for (size_t layer = 0; layer < keep_out.size(); ++layer) {
        LayerCollision &collision = collisions[layer];
        collision.lines = to_lines(keep_out[layer]);
        for (const LineSegment &line : collision.lines)
            collision.index.append_segment(line.a, line.b);
        collision.index.build_index();
    }
    return collisions;
}

// Each sphere's push and pull read only its own position, the outlines and the previous positions of its links, so the spheres
// of an iteration are moved in parallel. The iterations go on while a free sphere overlaps the outline or a node moves more than
// settled_displacement in an iteration, and stop after max_iterations. The largest move is gathered serially after the parallel
// pass. The parallel pass has one job per chunk of spheres_per_job spheres, without API scratch storage (a job creates none; each nearest_on_lines query still allocates its candidate vector).
size_t relax(std::vector<Sphere> &spheres, const std::vector<double> &print_z, const std::vector<LayerCollision> &collisions,
             const plugin_run_context *context)
{
    for (size_t iteration = 0; iteration < max_iterations; ++iteration) {
        for (Sphere &sphere : spheres)
            sphere.prev_position = sphere.position;
        std::atomic<size_t> num_overlapping{ 0 };
        RelaxPass pass{ &spheres, &print_z, &collisions, &num_overlapping, {} };
        slic3r_parallel_for(0, uint32_t((spheres.size() + spheres_per_job - 1) / spheres_per_job), &pass, &relax_chunk);
        pass.failure.rethrow_if_captured();
        slic3r_api::throw_if_cancelled(context);
        double largest_move = 0.;
        for (const Sphere &sphere : spheres)
            largest_move = std::max(largest_move, (sphere.position - sphere.prev_position).norm());
        if (num_overlapping == 0 && largest_move <= settled_displacement)
            return iteration + 1;
    }
    return max_iterations;
}

void relax_chunk(uint32_t job, void *user_data)
{
    // slic3r_parallel_for is an extern "C" host function: no exception may leave the job.
    RelaxPass &pass = *static_cast<RelaxPass *>(user_data);
    if (pass.failure.captured())
        return;
    try {
        std::vector<Sphere> &spheres = *pass.spheres;
        const size_t end = std::min(spheres.size(), (size_t(job) + 1) * spheres_per_job);
        for (size_t id = size_t(job) * spheres_per_job; id < end; ++id)
            if (!spheres[id].locked) {
                if (push_out_of_outline(spheres[id], *pass.print_z, *pass.collisions))
                    ++*pass.num_overlapping;
                pull_toward_links(spheres[id], spheres);
            }
    } catch (...) {
        pass.failure.capture_current();
    }
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
        // The lines are in scaled units: the query point and the limit are scaled, the distance and the hit point are unscaled.
        // Measured (5000 spheres, median of 5): this search is slower than the native AABB tree only in multithreaded Debug, where the
        // checked iterators of MSVC take a global lock (12.4 s against 2.5 s on 16 threads); in Release it is faster than native on 1 and
        // 16 threads (92.6 against 134 ms, 18.7 against 24.8 ms). A reusable candidate buffer, a nearest-only LineDistancer search and a
        // fast rejection were measured and rejected: none of them helped in Release or in single-thread Debug enough to justify API code.
        const std::optional<LineHit> hit = nearest_on_lines(collision.index, collision.lines, sphere.position * UNSCALING_FACTOR, r2 * sqr(UNSCALING_FACTOR));
        if (hit) {
            const double depth = std::sqrt(r2) - unscaled(std::sqrt(hit->squared_distance));
            if (depth > sphere.last_collision_depth) {
                sphere.last_collision_depth = depth;
                sphere.last_collision = hit->point * SCALING_FACTOR;
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
    // A free sphere always has spheres above (a node without any is a locked tip), and they all lie on the next layer.
    assert(!sphere.above.empty());
    Vec2d mean = Vec2d::Zero();
    for (const size_t above : sphere.above) {
        assert(spheres[above].z == spheres[sphere.above.front()].z);
        mean += spheres[above].prev_position;
    }
    mean /= double(sphere.above.size());
    if (sphere.below != no_node) {
        // The target is the point of the line from the sphere below to the mean above that lies at the Z of the sphere. Equal
        // layer heights give exactly t = 0.5, the midpoint.
        const Vec2d &below_position = spheres[sphere.below].prev_position;
        const double dz_below = double(sphere.z - spheres[sphere.below].z);
        const double dz_above = double(spheres[sphere.above.front()].z - sphere.z);
        const double t = dz_below / (dz_below + dz_above);
        mean = below_position + t * (mean - below_position);
    }
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
                node.position = scaled_point(spheres[id].position);
            ++id;
        }
}
} // namespace
} // namespace Slic3r::TreeFamily
