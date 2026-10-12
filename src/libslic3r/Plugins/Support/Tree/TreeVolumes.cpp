///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/*
Tree supports by Thomas Rahm, losely based on Tree Supports by CuraEngine.
   Original CuraEngine copyright:
   Copyright (c) 2021 Ultimaker B.V.
   CuraEngine is released under the terms of the AGPLv3 or higher. */

#include "TreeVolumes.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <string_view>
#include <unordered_map>

#include <boost/log/trivial.hpp>

#include <oneapi/tbb/parallel_for.h>
#include <oneapi/tbb/task_group.h>

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Utils.hpp"

// Errors about a missing cache entry are logged at the error level.
#define error_level_not_in_cache error

/* Collision, avoidance and placeable areas of the tree branches, computed per object layer and branch radius.

   The object layers are read once, from their outlines and the support blockers. precalculate() then computes, for the
   radii the tips can reach, the layers where a branch of that radius may stand:
   - the collision: the outline grown by the radius and the XY distance, merged over the Z gaps above and below;
   - the avoidance: the collision propagated upwards, one layer at a time, by the maximum move of each layer, so that a
     branch which avoids it can still reach the build plate (or the model);
   - the placeable areas: the top surfaces where a branch of that radius fits;
   - the wall restrictions: the walls that a branch must not cross between two consecutive layers.

   A getter that misses a key computes it itself, from any thread: every calculate*() function computes the whole range of its key
   (from layer 0 for the collisions, from layer 1 for the placeable areas, the avoidances and the wall restrictions) without
   looking at what the caches hold, outside of any lock, and inserts the result. The caches keep the first area inserted for a key,
   so two threads that compute the same key only duplicate work. The functions that read other getters (the holefree collisions,
   the avoidances, the placeable areas, the wall restrictions) first request the highest layer they need, so that a cold cache is
   filled by one computation instead of one per layer. See the Concurrency note in TreeVolumes.hpp.
*/

namespace Slic3r::TreeFamily {

using namespace std::literals;

// ClipperUtils rejects a zero offset of polygons, which a zero XY distance or a zero branch move gives; a zero distance keeps them.
static Polygons offset_polygons(const Polygons &polygons, double distance, ClipperLib::JoinType join_type = DefaultJoinType, double miter_limit = DefaultMiterLimit)
{
    return distance == 0. ? polygons : offset(polygons, distance, join_type, miter_limit);
}

TreeVolumes::TreeVolumes(const PrintObject &object, const TreeSettings &settings) :
    m_min_resolution(settings.resolution),
    m_current_min_xy_dist(settings.xy_min_distance),
    m_current_min_xy_dist_delta(settings.xy_distance - settings.xy_min_distance),
    m_support_rests_on_model(settings.support_rests_on_model),
    m_increase_until_radius(settings.increase_radius_until_radius),
    m_radius_0(settings.getRadius(0)),
    m_z_distance_bottom_layers(int(settings.z_distance_bottom_layers)),
    m_z_distance_top_layers(int(settings.z_distance_top_layers)),
    m_settings(settings)
{
    m_anti_overhang = object.slice_support_blockers();
    m_layer_outlines.reserve(object.layer_count());
    for (size_t layer_idx = 0; layer_idx < object.layer_count(); ++ layer_idx)
        m_layer_outlines.push_back(polygons_simplify(to_polygons(object.layer(layer_idx).lslices()), settings.resolution, polygons_strictly_simple));
    // The maximum move keeps a margin of two microns against rounding errors.
    for (coord_t move : settings.max_move)
        m_max_move.push_back(std::max<coord_t>(move - 2, 0));
    for (coord_t move : settings.max_move_slow)
        m_max_move_slow.push_back(std::max<coord_t>(move - 2, 0));
}

// Computes the caches for the keys (ceiled radius, layer) that a tree whose top layer is max_layer can request: each radius is
// computed from the lowest layer up to the highest layer where it can occur. It also sets the members that the getters read without
// a lock (m_precalculated, m_ignorable_radii), so it must complete before the getters are called concurrently. The areas stay in the
// caches, which own them: the getters return references to them. An exception thrown by throw_on_cancel propagates to the caller;
// the keys published before it stay, and a key that was not computed is computed by the getter that asks for it.
void TreeVolumes::precalculate(const coord_t max_layer, std::function<void()> throw_on_cancel)
{
    m_precalculated = true;
    const TreeSettings &config = m_settings;

    // Radii that a tip can have while it grows from the radius at distance to top 0 to the branch radius. The radii that no tip
    // can reach are skipped by ceilRadius(), as they will not be requested.
    {
        std::vector<coord_t> possible_tip_radiis;
        for (size_t distance_to_top = 0; distance_to_top <= config.tip_layers; ++ distance_to_top) {
            possible_tip_radiis.emplace_back(ceilRadius(config.getRadius(distance_to_top)));
            possible_tip_radiis.emplace_back(ceilRadius(config.getRadius(distance_to_top) + m_current_min_xy_dist_delta));
        }
        sort_remove_duplicates(possible_tip_radiis);
        for (coord_t radius_eval = m_radius_0; radius_eval <= config.branch_radius; radius_eval = ceilRadius(radius_eval + 1))
            if (! std::binary_search(possible_tip_radiis.begin(), possible_tip_radiis.end(), radius_eval))
                m_ignorable_radii.emplace_back(radius_eval);
    }

    throw_on_cancel();

    // The radius each distance to top may have, keyed by the highest layer where it is needed. The simulation runs from the top
    // layer down, so that the caches cover exactly the layers a tree can reach.
    std::unordered_map<coord_t, LayerIndex> radius_until_layer;
    for (LayerIndex distance_to_top = 0; distance_to_top <= max_layer; ++ distance_to_top) {
        const LayerIndex current_layer = max_layer - distance_to_top;
        auto update_radius_until_layer = [&radius_until_layer, current_layer](coord_t r) {
            // A zero radius needs no area.
            if (r <= 0)
                return;
            auto it = radius_until_layer.find(r);
            if (it == radius_until_layer.end())
                radius_until_layer.emplace_hint(it, r, current_layer);
            else
                assert(it->second >= current_layer);
        };
        update_radius_until_layer(ceilRadius(config.getRadius(distance_to_top, 0) + m_current_min_xy_dist_delta));
        update_radius_until_layer(ceilRadius(config.getRadius(distance_to_top, 0)));
        update_radius_until_layer(ceilRadius(config.recommendedMinRadius(current_layer) + m_current_min_xy_dist_delta));
        // The wider base of a branch near the build plate, as the routing asks for it with the minimum XY distance (no delta).
        update_radius_until_layer(ceilRadius(config.recommendedMinRadius(current_layer)));
    }

    throw_on_cancel();

    std::vector<RadiusLayerPair> relevant_avoidance_radiis{ radius_until_layer.begin(), radius_until_layer.end() };

    // The collision of the increase radius is needed for the holefree collisions, and the one of radius 0 for the xy distance checks.
    radius_until_layer[ceilRadius(m_increase_until_radius + m_current_min_xy_dist_delta)] = max_layer;
    radius_until_layer[0] = max_layer;
    if (m_current_min_xy_dist_delta != 0)
        radius_until_layer[m_current_min_xy_dist_delta] = max_layer;

    std::vector<RadiusLayerPair> relevant_collision_radiis{ radius_until_layer.begin(), radius_until_layer.end() };
    calculateCollision(relevant_collision_radiis, throw_on_cancel);

    std::vector<RadiusLayerPair> relevant_hole_collision_radiis;
    for (RadiusLayerPair key : relevant_avoidance_radiis)
        if (key.first < m_increase_until_radius + m_current_min_xy_dist_delta)
            relevant_hole_collision_radiis.emplace_back(key);
    calculateCollisionHolefree(relevant_hole_collision_radiis, throw_on_cancel);

    // Placeable areas are computed from the avoidances, which request them for the layers they need.
    if (m_support_rests_on_model)
        calculatePlaceables(relevant_avoidance_radiis, throw_on_cancel);

    {
        tbb::task_group task_group;
        task_group.run([this, relevant_avoidance_radiis, throw_on_cancel]{ calculateAvoidance(relevant_avoidance_radiis, true, m_support_rests_on_model, throw_on_cancel); });
        task_group.run([this, relevant_avoidance_radiis, throw_on_cancel]{ calculateWallRestrictions(relevant_avoidance_radiis, throw_on_cancel); });
        task_group.wait();
    }
}

void TreeVolumes::calculateCollision(const std::vector<RadiusLayerPair> &keys, std::function<void()> throw_on_cancel)
{
    tbb::parallel_for(tbb::blocked_range<size_t>(0, keys.size()),
        [&](const tbb::blocked_range<size_t> &range) {
        for (size_t ikey = range.begin(); ikey != range.end(); ++ ikey) {
            const LayerIndex radius        = keys[ikey].first;
            const size_t     max_layer_idx = keys[ikey].second;
            // recursive call to parallel_for.
            calculateCollision(radius, max_layer_idx, throw_on_cancel);
        }
    });
}

// Computes the collision of one radius on layers 0 to max_layer_idx: the outline grown by the radius and the XY distance, then
// merged over the Z gaps. The placeable areas of radius 0 come from the same outlines. The whole range is computed whatever the
// cache holds, and the result is inserted without replacing what another thread inserted meanwhile.
void TreeVolumes::calculateCollision(const coord_t radius, const LayerIndex max_layer_idx, std::function<void()> throw_on_cancel)
{
    const std::vector<Polygons> &outlines = m_layer_outlines;
    const coord_t xy_distance = m_current_min_xy_dist;
    const int z_distance_bottom_layers = m_z_distance_bottom_layers;
    const int z_distance_top_layers = m_z_distance_top_layers;

    // Layer range for which the collisions will be calculated.
    LayerPolygonCache data;
    data.allocate(0, max_layer_idx + 1);

    const bool calculate_placable = m_support_rests_on_model && radius == 0;
    LayerPolygonCache data_placeable;
    if (calculate_placable)
        data_placeable.allocate(data.begin(), data.end());

    // 1) Offset the outlines of the layers that the sums below use.
    LayerPolygonCache collision_areas_offsetted;
    collision_areas_offsetted.allocate(
        std::max<LayerIndex>(0, data.begin() - z_distance_bottom_layers),
        std::min<LayerIndex>(outlines.size(), data.end() + z_distance_top_layers));
    tbb::parallel_for(tbb::blocked_range<LayerIndex>(collision_areas_offsetted.begin(), collision_areas_offsetted.end()),
        [&outlines, offset_value = radius + xy_distance, &collision_areas_offsetted, &throw_on_cancel]
        (const tbb::blocked_range<LayerIndex> &range) {
            for (LayerIndex layer_idx = range.begin(); layer_idx != range.end(); ++ layer_idx) {
                // jtRound is not needed here, as the overshoot can not cause errors in the algorithm, because no assumptions are made about the model.
                collision_areas_offsetted[layer_idx] = offset_value == 0 ?
                    union_(outlines[layer_idx]) :
                    offset(union_ex(outlines[layer_idx]), offset_value, ClipperLib::jtMiter, 1.2);
                throw_on_cancel();
            }
        });

    // 2) Sum the outlines over the Z gaps of each layer. The outlines above are offset by a distance that shrinks with the
    // layer distance, so that a branch may come as close to the layers above as the xy distance of the overhang requires.
    // The Z gaps are counts of nominal layers, rounded as the legacy does (TreeSettings.hpp).
    tbb::parallel_for(tbb::blocked_range<LayerIndex>(data.begin(), data.end()),
        [&collision_areas_offsetted, &outlines, &anti_overhang = m_anti_overhang, radius, xy_distance, z_distance_bottom_layers, z_distance_top_layers,
            min_resolution = m_min_resolution, &data, &throw_on_cancel]
        (const tbb::blocked_range<LayerIndex>& range) {
            for (LayerIndex layer_idx = range.begin(); layer_idx != range.end(); ++ layer_idx) {
                Polygons collisions;
                for (int i = - z_distance_bottom_layers; i <= 0; ++ i)
                    if (int j = layer_idx + i; collision_areas_offsetted.has(j))
                        append(collisions, collision_areas_offsetted[j]);
                for (int i = 1; i <= z_distance_top_layers; ++ i)
                    if (int j = layer_idx + i; j < int(outlines.size())) {
                        // The conditional -0.5 keeps plastic from touching diagonally below a layer when the Z gap is one layer:
                        // an overhang below 90 degrees is then left unsupported rather than fused to the object.
                        const coord_t required_range_x =
                            (xy_distance - ((i - (z_distance_top_layers == 1 ? 0.5 : 0)) * xy_distance / z_distance_top_layers));
                        append(collisions, offset(union_ex(outlines[j]), radius + required_range_x, ClipperLib::jtMiter, 1.2));
                    }
                collisions = layer_idx < int(anti_overhang.size()) ?
                    union_(collisions, offset(anti_overhang[layer_idx], radius, ClipperLib::jtMiter, 1.2)) :
                    union_(collisions);
                data[layer_idx] = polygons_simplify(collisions, min_resolution, polygons_strictly_simple);
                throw_on_cancel();
            }
        });

    // 3) The placeable areas: the top surface of the layer below, minus the collision of this layer, inset by the XY distance.
    if (calculate_placable) {
        tbb::parallel_for(tbb::blocked_range<LayerIndex>(std::max(z_distance_bottom_layers + 1, data.begin()), data.end()),
            [&collision_areas_offsetted, &outlines, &anti_overhang = m_anti_overhang, min_resolution = m_min_resolution,
                z_distance_bottom_layers, xy_distance, &data_placeable, &throw_on_cancel]
            (const tbb::blocked_range<LayerIndex>& range) {
                for (LayerIndex layer_idx = range.begin(); layer_idx != range.end(); ++ layer_idx) {
                    const LayerIndex layer_idx_below = layer_idx - z_distance_bottom_layers - 1;
                    const Polygons &current = collision_areas_offsetted[layer_idx];
                    const Polygons &below   = outlines[layer_idx_below];
                    data_placeable[layer_idx] = polygons_simplify(diff(
                        // Inflate the surface to sit on by the separation distance to increase chance of a support being placed on a sloped surface.
                        offset_polygons(below, xy_distance),
                        layer_idx_below < int(anti_overhang.size()) ? union_(current, to_polygons(anti_overhang[layer_idx_below])) : current),
                        min_resolution, polygons_strictly_simple);
                    throw_on_cancel();
                }
            });
    }

    throw_on_cancel();
    m_collision_cache.insert(std::move(data), radius);
    if (calculate_placable)
        m_placeable_areas_cache.insert(std::move(data_placeable), radius);
}

// Computes, for each key (radius, last layer), the collision of that radius on layers 0 to the last layer without the holes too small
// for a branch of the radius until which it always widens. The radii are ceiled and smaller than m_increase_until_radius plus the
// XY distance delta. The layers are published in chunks, each chunk with the layers of all the keys; an exception thrown by
// throw_on_cancel propagates, and the chunks published before it stay (a missing layer is computed by getCollisionHolefree).
void TreeVolumes::calculateCollisionHolefree(const std::vector<RadiusLayerPair> &keys, std::function<void()> throw_on_cancel)
{
    // Without any key nothing is to compute, and the key count is the grainsize of the parallel loop below, which TBB requires to be
    // positive. There is no key when every radius reaches m_increase_until_radius plus the XY distance delta. For instance, the tip
    // radius is clamped to the branch radius when the tip diameter is at least the branch diameter (TreeSettings.cpp), so every
    // radius is at least the branch radius, which is m_increase_until_radius, and the delta is 0.
    if (keys.empty())
        return;

    LayerIndex max_layer = 0;
    for (long long unsigned int i = 0; i < keys.size(); i++)
        max_layer = std::max(max_layer, keys[i].second);

    // The layers below grow the collision of m_increase_until_radius. When it is not cached, one computation of its whole range
    // fills it, instead of one computation per layer in the loop below.
    getCollision(m_increase_until_radius, max_layer, false);

    tbb::parallel_for(tbb::blocked_range<LayerIndex>(0, max_layer + 1, keys.size()),
        [&](const tbb::blocked_range<LayerIndex> &range) {
        std::vector<std::pair<RadiusLayerPair, Polygons>> data;
        data.reserve(range.size() * keys.size());
        for (LayerIndex layer_idx = range.begin(); layer_idx < range.end(); ++ layer_idx) {
            for (RadiusLayerPair key : keys)
                if (layer_idx <= key.second) {
                    // Logically increase the collision by m_increase_until_radius
                    coord_t radius = key.first;
                    assert(radius == this->ceilRadius(radius));
                    assert(radius < m_increase_until_radius + m_current_min_xy_dist_delta);
                    coord_t increase_radius_ceil = ceilRadius(m_increase_until_radius, false) - radius;
                    assert(increase_radius_ceil > 0);
                    // this union is important as otherwise holes(in form of lines that will increase to holes in a later step) can get unioned onto the area.
                    data.emplace_back(RadiusLayerPair(radius, layer_idx), polygons_simplify(
                        offset(union_ex(this->getCollision(m_increase_until_radius, layer_idx, false)),
                            5 - increase_radius_ceil, ClipperLib::jtRound, m_min_resolution),
                        m_min_resolution, polygons_strictly_simple));
                    throw_on_cancel();
                }
        }
        m_collision_cache_holefree.insert(std::move(data));
    });
}

// One cut of a maximum move into steps no longer than a step size: the count, the step, and the last step, which takes the rest.
struct MoveSteps {
    int   count;
    float step;
    float last;
};

// Cuts a maximum move into steps. A very short last step is avoided by stretching the other steps instead.
static MoveSteps split_move(const float max_move, const float move_step)
{
    MoveSteps steps{ std::max(1, round_up_divide<int>(max_move, move_step)), move_step, 0.f };
    steps.last = max_move - float(steps.count - 1) * move_step;
    if (steps.last < scale_d(0.05) && steps.count > 1) {
        steps.step = max_move / float(-- steps.count);
        steps.last = steps.step;
    }
    return steps;
}

// Propagates the avoidance of each key upwards, one object layer at a time. Between layer_idx - 1 and layer_idx the avoidance may
// shrink by the maximum move of layer_idx, which is the height of that layer times the branch angle. The propagation of a key
// starts from the collision of layer 0, which is the avoidance of layer 0, and computes layers 1 up to the requested layer.
void TreeVolumes::calculateAvoidance(const std::vector<RadiusLayerPair> &keys, bool to_build_plate, bool to_model, std::function<void()> throw_on_cancel)
{
    // For every RadiusLayer pair there are 3 avoidances that have to be calculated.
    // Prepare tasks for parallelization.
    struct AvoidanceTask {
        AvoidanceType   type;
        coord_t         radius;
        LayerIndex      max_required_layer;
        bool            to_model;

        bool slow()     const { return this->type == AvoidanceType::Slow; }
        bool holefree() const { return this->type == AvoidanceType::FastSafe; }
    };

    std::vector<AvoidanceTask> avoidance_tasks;
    avoidance_tasks.reserve((int(to_build_plate) + int(to_model)) * keys.size() * size_t(AvoidanceType::Count));

    // Layer 0 has no avoidance of its own, it is the collision: the propagation starts above it.
    constexpr LayerIndex first_layer = 1;
    for (int iter_idx = 0; iter_idx < 2 * int(keys.size()) * int(AvoidanceType::Count); ++ iter_idx) {
        AvoidanceTask task{
            AvoidanceType(iter_idx % int(AvoidanceType::Count)),
            keys[iter_idx / 6].first,  // radius
            keys[iter_idx / 6].second, // max_layer
            ((iter_idx / 3) & 1) != 0  // to_model
        };
        if (task.max_required_layer < first_layer)
            continue;
        if ((task.to_model ? to_model : to_build_plate) &&
            (! task.holefree() || task.radius < m_increase_until_radius + m_current_min_xy_dist_delta))
            avoidance_tasks.emplace_back(task);
    }

    throw_on_cancel();

    tbb::parallel_for(tbb::blocked_range<size_t>(0, avoidance_tasks.size(), 1),
        [this, &avoidance_tasks, &throw_on_cancel, first_layer](const tbb::blocked_range<size_t> &range) {
        for (size_t task_idx = range.begin(); task_idx < range.end(); ++ task_idx) {
            const AvoidanceTask &task = avoidance_tasks[task_idx];
            assert(! task.holefree() || task.radius < m_increase_until_radius + m_current_min_xy_dist_delta);
            if (task.to_model)
                getPlaceableAreas(task.radius, task.max_required_layer, throw_on_cancel);
            // The following loop propagating avoidance regions bottom up is inherently serial.
            const bool collision_holefree = (task.slow() || task.holefree()) && task.radius < m_increase_until_radius + m_current_min_xy_dist_delta;
            // The collision that the avoidance of this task merges with, on a layer.
            const auto collision_of_layer = [this, &task, collision_holefree](LayerIndex layer_idx) -> const Polygons& {
                return collision_holefree ? getCollisionHolefree(task.radius, layer_idx) : getCollision(task.radius, layer_idx, true);
            };
            // When the collisions are not cached, one computation of their whole range fills them, instead of one per layer below.
            collision_of_layer(task.max_required_layer);
            const std::vector<coord_t> &max_moves = task.slow() ? m_max_move_slow : m_max_move;
            // Limiting the offset step so that unioning the shrunk latest_avoidance with the current layer collisions
            // will not create gaps in the resulting avoidance region letting a tree support branch tunneling through an object wall.
            const float move_step = 1.9f * float(std::max(task.radius, m_current_min_xy_dist));
            // Threshold must be >= 0.5 to prevent division by zero when move_step is cast to int in round_up_divide. A branch below
            // it (no radius and no XY distance) has no step to propagate with: its avoidance is its collision. It is published
            // like the others, as the getter that asked for it reads it back.
            const bool can_move = move_step >= 0.5f;
            // minDist as the delta was already added, also avoidance for layer 0 will return the collision.
            Polygons latest_avoidance = getAvoidance(task.radius, first_layer - 1, task.type, task.to_model, true);
            std::vector<std::pair<RadiusLayerPair, Polygons>> data;
            data.reserve(task.max_required_layer + 1 - first_layer);
            for (LayerIndex layer_idx = first_layer; layer_idx <= task.max_required_layer; ++ layer_idx) {
                // Merge current layer collisions with shrunk last_avoidance.
                const Polygons &current_layer_collisions = collision_of_layer(layer_idx);
                if (can_move) {
                    // For mildly steep branch angles only one step will be taken.
                    const MoveSteps steps = split_move(float(max_moves[layer_idx]), move_step);
                    for (int istep = 0; istep < steps.count; ++ istep)
                        latest_avoidance = union_(current_layer_collisions,
                            offset_polygons(latest_avoidance, istep + 1 == steps.count ? - steps.last : - steps.step, ClipperLib::jtRound, m_min_resolution));
                } else
                    latest_avoidance = current_layer_collisions;
                if (task.to_model)
                    latest_avoidance = diff(latest_avoidance, getPlaceableAreas(task.radius, layer_idx, throw_on_cancel));
                latest_avoidance = polygons_simplify(latest_avoidance, m_min_resolution, polygons_strictly_simple);
                data.emplace_back(RadiusLayerPair{task.radius, layer_idx}, latest_avoidance);
                throw_on_cancel();
            }
            avoidance_cache(task.type, task.to_model).insert(std::move(data));
        }
    });
}

void TreeVolumes::calculatePlaceables(const std::vector<RadiusLayerPair> &keys, std::function<void()> throw_on_cancel)
{
    tbb::parallel_for(tbb::blocked_range<size_t>(0, keys.size()),
        [&, keys](const tbb::blocked_range<size_t>& range) {
            for (size_t key_idx = range.begin(); key_idx < range.end(); ++ key_idx)
                this->calculatePlaceables(keys[key_idx].first, keys[key_idx].second, throw_on_cancel);
        });
}

// Computes the placeable areas of one radius on layers 0 to max_required_layer, from the ones of radius 0 shrunk by the radius and
// by the regular XY distance. Layer 0 has no placeable area, as nothing lies below it: its entry stays empty, so that a request for
// it finds its key.
void TreeVolumes::calculatePlaceables(const coord_t radius, const LayerIndex max_required_layer, std::function<void()> throw_on_cancel)
{
    std::vector<Polygons> data(max_required_layer + 1, Polygons{});

    // When the placeable areas of radius 0 are not cached, one computation of their whole range fills them (they come with the
    // collision of radius 0), instead of one per layer in the loop below.
    getPlaceableAreas(0, max_required_layer, throw_on_cancel);

    tbb::parallel_for(tbb::blocked_range<LayerIndex>(1, max_required_layer + 1),
        [this, &data, radius, &throw_on_cancel](const tbb::blocked_range<LayerIndex>& range) {
            for (LayerIndex layer_idx = range.begin(); layer_idx < range.end(); ++ layer_idx) {
                data[layer_idx] = offset(
                    union_ex(getPlaceableAreas(0, layer_idx, throw_on_cancel)),
                    // As a placeable area is calculated by (collision of the layer below) - (collision of the current layer) and the collision is offset by xy_distance,
                    // it can happen that a small line is considered a flat area to place something onto, even though it is mostly
                    // xy_distance that cant support it. Making the area smaller by xy_distance fixes this.
                    - (radius + m_current_min_xy_dist + m_current_min_xy_dist_delta),
                    jtMiter, 1.2);
                throw_on_cancel();
            }
        });
    m_placeable_areas_cache.insert(std::move(data), 0, radius);
}

// Computes, for each key (radius, last layer), the wall restrictions of that radius on layers 1 to the last layer, with the regular
// XY distance and, when it differs from the minimum one, with the minimum XY distance as well. The radii are ceiled. A key is
// published whole, after its loop; an exception thrown by throw_on_cancel propagates, and the keys published before it stay.
void TreeVolumes::calculateWallRestrictions(const std::vector<RadiusLayerPair> &keys, std::function<void()> throw_on_cancel)
{
    // Wall restrictions are mainly important when they represent actual walls that are printed, and not "just" the configured z_distance, because technically valid placement is no excuse for moving through a wall.
    // As they exist to prevent accidentially moving though a wall at high speed between layers like thie (x = wall,i = influence area,o= empty space,d = blocked area because of z distance) Assume maximum movement distance is two characters and maximum safe movement distance of one character

    /* Potential issue addressed by the wall restrictions: Influence area may lag through a wall
     *  layer z+1:iiiiiiiiiiioooo
     *  layer z+0:xxxxxiiiiiiiooo
     *  layer z-1:ooooixxxxxxxxxx
     */

    // The radius for the upper collission has to be 0 as otherwise one may not enter areas that may be forbidden on layer_idx but not one below (c = not an influence area even though it should ):
    /*
     *  layer z+1:xxxxxiiiiiioo
     *  layer z+0:dddddiiiiiiio
     *  layer z-1:dddocdddddddd
     */
    // Also there can not just the collision of the lower layer be used because if it were:
    /*
     *  layer z+1:dddddiiiiiiiiiio
     *  layer z+0:xxxxxddddddddddc
     *  layer z-1:dddddxxxxxxxxxxc
     */
    // Or of the upper layer be used because if it were:
    /*
     *  layer z+1:dddddiiiiiiiiiio
     *  layer z+0:xxxxcddddddddddc
     *  layer z-1:ddddcxxxxxxxxxxc
     */

    // And just offseting with maximum movement distance (and not in multiple steps) could cause:
    /*
     *  layer z:   oxiiiiiiiiioo
     *  layer z-1: ixiiiiiiiiiii
     */

    tbb::parallel_for(tbb::blocked_range<size_t>(0, keys.size()),
        [&, keys](const tbb::blocked_range<size_t> &range) {
        for (size_t key_idx = range.begin(); key_idx < range.end(); ++ key_idx) {
            const coord_t    radius             = keys[key_idx].first;
            const LayerIndex max_required_layer = keys[key_idx].second;
            // A wall restriction needs the layer below, so the whole range starts at layer 1.
            const LayerIndex first_layer        = 1;
            if (max_required_layer < first_layer)
                continue;
            const size_t     buffer_size        = max_required_layer + 1 - first_layer;
            std::vector<Polygons> data(buffer_size, Polygons{});
            std::vector<Polygons> data_min;
            if (m_current_min_xy_dist_delta > 0)
                data_min.assign(buffer_size, Polygons{});
            // When the collisions are not cached, one computation of their whole range fills them, instead of one per layer below.
            getCollision(0, max_required_layer, false);
            if (! data_min.empty())
                getCollision(0, max_required_layer, true);
            getCollision(radius, max_required_layer - 1, true);
            tbb::parallel_for(tbb::blocked_range<LayerIndex>(first_layer, max_required_layer + 1),
                [this, &data, &data_min, radius, first_layer, &throw_on_cancel](const tbb::blocked_range<LayerIndex> &range) {
                for (LayerIndex layer_idx = range.begin(); layer_idx < range.end(); ++ layer_idx) {
                    data[layer_idx - first_layer] = polygons_simplify(
                        // radius contains m_current_min_xy_dist_delta already if required
                        intersection(getCollision(0, layer_idx, false), getCollision(radius, layer_idx - 1, true)),
                        m_min_resolution, polygons_strictly_simple);
                    if (! data_min.empty())
                        data_min[layer_idx - first_layer] =
                            polygons_simplify(
                                intersection(getCollision(0, layer_idx, true), getCollision(radius, layer_idx - 1, true)),
                                m_min_resolution, polygons_strictly_simple);
                    throw_on_cancel();
                }
            });
            m_wall_restrictions_cache.insert(std::move(data), first_layer, radius);
            if (! data_min.empty())
                m_wall_restrictions_cache_min.insert(std::move(data_min), first_layer, radius);
        }
    });
}

const Polygons& TreeVolumes::getCollision(const coord_t orig_radius, LayerIndex layer_idx, bool min_xy_dist) const
{
    const coord_t radius = this->ceilRadius(orig_radius, min_xy_dist);
    if (std::optional<std::reference_wrapper<const Polygons>> result = m_collision_cache.getArea({ radius, layer_idx }); result)
        return (*result).get();
    if (m_precalculated) {
        BOOST_LOG_TRIVIAL(error_level_not_in_cache) << "Had to calculate collision at radius " << radius << " and layer " << layer_idx << ", but precalculate was called. Performance may suffer!";
        BOOST_LOG_TRIVIAL(error) << "Not precalculated Collision requested."sv;
    }
    const_cast<TreeVolumes*>(this)->calculateCollision(radius, layer_idx, []{});
    return getCollision(orig_radius, layer_idx, min_xy_dist);
}

const Polygons& TreeVolumes::getCollisionHolefree(coord_t radius, LayerIndex layer_idx) const
{
    assert(radius == this->ceilRadius(radius));
    assert(radius < m_increase_until_radius + m_current_min_xy_dist_delta);
    if (std::optional<std::reference_wrapper<const Polygons>> result = m_collision_cache_holefree.getArea({ radius, layer_idx }); result)
        return (*result).get();
    if (m_precalculated) {
        BOOST_LOG_TRIVIAL(error_level_not_in_cache) << "Had to calculate collision holefree at radius " << radius << " and layer " << layer_idx << ", but precalculate was called. Performance may suffer!";
        BOOST_LOG_TRIVIAL(error) << "Not precalculated Holefree Collision requested."sv;
    }
    const_cast<TreeVolumes*>(this)->calculateCollisionHolefree({ radius, layer_idx });
    return getCollisionHolefree(radius, layer_idx);
}

const Polygons& TreeVolumes::getAvoidance(const coord_t orig_radius, LayerIndex layer_idx, AvoidanceType type, bool to_model, bool min_xy_dist) const
{
    if (layer_idx == 0) // What on the layer directly above buildplate do i have to avoid to reach the buildplate ...
        return getCollision(orig_radius, layer_idx, min_xy_dist);

    const coord_t radius = this->ceilRadius(orig_radius, min_xy_dist);
    if (type == AvoidanceType::FastSafe && radius >= m_increase_until_radius + m_current_min_xy_dist_delta)
        // no holes anymore by definition at this request
        type = AvoidanceType::Fast;

    if (std::optional<std::reference_wrapper<const Polygons>> result =
            this->avoidance_cache(type, to_model).getArea({ radius, layer_idx });
        result)
        return (*result).get();

    if (m_precalculated) {
        if (to_model) {
            BOOST_LOG_TRIVIAL(error_level_not_in_cache) << "Had to calculate Avoidance to model at radius " << radius << " and layer " << layer_idx << ", but precalculate was called. Performance may suffer!";
            BOOST_LOG_TRIVIAL(error) << "Not precalculated Avoidance(to model) requested."sv;
        } else {
            BOOST_LOG_TRIVIAL(error_level_not_in_cache) << "Had to calculate Avoidance at radius " << radius << " and layer " << layer_idx << ", but precalculate was called. Performance may suffer!";
            BOOST_LOG_TRIVIAL(error) << "Not precalculated Avoidance(to buildplate) requested."sv;
        }
    }
    const_cast<TreeVolumes*>(this)->calculateAvoidance({ radius, layer_idx }, ! to_model, to_model);
    // Retrive failed and correct result was calculated. Now it has to be retrived.
    return getAvoidance(orig_radius, layer_idx, type, to_model, min_xy_dist);
}

// Gives the placeable areas of a branch of orig_radius (rounded as the other getters do, without the XY distance delta) on layer
// layer_idx >= 0. On a cache miss it computes the key, from layer 0 to layer_idx, then reads the cache again; the reference is
// owned by the cache. An exception thrown by throw_on_cancel propagates, and what was published before it stays.
const Polygons& TreeVolumes::getPlaceableAreas(const coord_t orig_radius, LayerIndex layer_idx, std::function<void()> throw_on_cancel) const
{
    const coord_t radius = ceilRadius(orig_radius);
    if (std::optional<std::reference_wrapper<const Polygons>> result = m_placeable_areas_cache.getArea({ radius, layer_idx }); result)
        return (*result).get();
    if (m_precalculated) {
        BOOST_LOG_TRIVIAL(error_level_not_in_cache) << "Had to calculate Placeable Areas at radius " << radius << " and layer " << layer_idx << ", but precalculate was called. Performance may suffer!";
        BOOST_LOG_TRIVIAL(error) << format("Not precalculated Placeable areas requested, radius %1%, layer %2%", radius, layer_idx);
    }
    if (orig_radius == 0) {
        // Placeable areas for radius 0 are calculated in the general collision code, which inserts them with the collisions. The
        // collision may be cached already while the thread that computed it is still inserting the placeable areas, so this runs
        // the computation (a duplicate is harmless) instead of reading the collision, and reads the placeable areas again below.
        if (! m_support_rests_on_model)
            // Nothing rests on the model, so that code computes no placeable areas, and the routing does not ask for them: the
            // collision is returned, which ends the request.
            return this->getCollision(0, layer_idx, true);
        const_cast<TreeVolumes*>(this)->calculateCollision(0, layer_idx, throw_on_cancel);
    } else
        const_cast<TreeVolumes*>(this)->calculatePlaceables(radius, layer_idx, throw_on_cancel);
    // The computation above inserted the area, or another thread did before.
    return getPlaceableAreas(orig_radius, layer_idx, throw_on_cancel);
}

const Polygons& TreeVolumes::getWallRestriction(const coord_t orig_radius, LayerIndex layer_idx, bool min_xy_dist) const
{
    assert(layer_idx > 0);
    if (layer_idx == 0)
        // Should never be requested as there will be no going below layer 0 ...,
        // but just to be sure some semi-sane catch. Alternative would be empty Polygon.
        return getCollision(orig_radius, layer_idx, min_xy_dist);

    min_xy_dist &= m_current_min_xy_dist_delta > 0;

    const coord_t radius = ceilRadius(orig_radius);
    if (std::optional<std::reference_wrapper<const Polygons>> result =
        (min_xy_dist ? m_wall_restrictions_cache_min : m_wall_restrictions_cache).getArea({ radius, layer_idx });
        result)
        return (*result).get();
    if (m_precalculated) {
        BOOST_LOG_TRIVIAL(error_level_not_in_cache) << "Had to calculate Wall restrictions at radius " << radius << " and layer " << layer_idx << ", but precalculate was called. Performance may suffer!";
        BOOST_LOG_TRIVIAL(error) << (min_xy_dist ?
            "Not precalculated Wall restriction of minimum xy distance requested )." :
            "Not precalculated Wall restriction requested )."sv);
    }
    const_cast<TreeVolumes*>(this)->calculateWallRestrictions({ radius, layer_idx });
    return getWallRestriction(orig_radius, layer_idx, min_xy_dist); // Retrieve failed and correct result was calculated. Now it has to be retrieved.
}

coord_t TreeVolumes::ceilRadius(const coord_t radius) const
{
    if (radius == 0)
        return 0;

    coord_t out = m_radius_0;
    if (radius > m_radius_0) {
        // generate SUPPORT_TREE_PRE_EXPONENTIAL_STEPS of radiis before starting to exponentially increase them.
        coord_t initial_radius_delta = SUPPORT_TREE_EXPONENTIAL_THRESHOLD - m_radius_0;
        auto ignore = [this](coord_t r) { return std::binary_search(m_ignorable_radii.begin(), m_ignorable_radii.end(), r); };
        if (initial_radius_delta > SUPPORT_TREE_COLLISION_RESOLUTION) {
            const int num_steps = round_up_divide(initial_radius_delta, SUPPORT_TREE_EXPONENTIAL_THRESHOLD);
            const int stepsize  = initial_radius_delta / num_steps;
            out += stepsize;
            for (auto step = 0; step < num_steps; ++ step) {
                if (out >= radius && ! ignore(out))
                    return out;
                out += stepsize;
            }
        } else
            out += SUPPORT_TREE_COLLISION_RESOLUTION;
        while (out < radius || ignore(out)) {
            assert(out * SUPPORT_TREE_EXPONENTIAL_FACTOR > out + SUPPORT_TREE_COLLISION_RESOLUTION);
            out = out * SUPPORT_TREE_EXPONENTIAL_FACTOR;
        }
    }
    return out;
}

} // namespace Slic3r::TreeFamily
