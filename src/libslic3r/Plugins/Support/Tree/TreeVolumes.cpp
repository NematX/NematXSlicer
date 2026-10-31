///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/*
Tree supports by Thomas Rahm, losely based on Tree Supports by CuraEngine.
   Original CuraEngine copyright:
   Copyright (c) 2021 Ultimaker B.V.
   CuraEngine is released under the terms of the AGPLv3 or higher. */

#include "TreeVolumes.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_map>

#include <boost/log/trivial.hpp>

#include <oneapi/tbb/parallel_for.h>
#include <oneapi/tbb/task_group.h>

#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/PolygonOps.hpp"
#include "libslic3r/Api/plugin/cpp/VolumeViews.hpp"
#include "libslic3r/Plugins/Support/Common/SupportParams.hpp"
#include "TreeGeometry.hpp"

/* Collision, avoidance and placeable areas of the tree branches, computed per object layer and branch radius.

   The object layers are read once, from their outlines and the support blockers. precalculate() then computes, for the
   radii the tips can reach, the layers where a branch of that radius may stand:
   - the collision: the outline grown by the radius and the XY distance, merged over the Z gaps above and below;
   - the avoidance: the collision propagated upwards, one layer at a time, by the maximum move of each layer, so that a
     branch which avoids it can still reach the build plate (or the model);
   - the placeable areas: the top surfaces where a branch of that radius fits;
   - the wall restrictions: the walls that a branch must not cross between two consecutive layers.
   A radius is always rounded by ceilRadius() to a step of a table built by the constructor (build_radius_steps()): the scale of the
   tips, with a step of TreeSettings::foot_cache_step where the branches widen into their foot on the build plate. The areas are cached
   per step, so the table sets how finely the routing can check that a wider branch still fits.

   A getter that misses a key computes it itself, from any thread: every calculate*() function computes the whole range of its key
   (from layer 0 for the collisions, from layer 1 for the placeable areas, the avoidances and the wall restrictions) without
   looking at what the caches hold, outside of any lock, and inserts the result. The caches keep the first area inserted for a key,
   so two threads that compute the same key only duplicate work. The functions that read other getters (the holefree collisions,
   the avoidances, the placeable areas, the wall restrictions) first request the highest layer they need, so that a cold cache is
   filled by one computation instead of one per layer. See the Concurrency note in TreeVolumes.hpp.

   All the geometry is written on the plugin API: the areas are StoredPolygonCollection, the boolean operations and offsets are the
   wrappers of TreeGeometry.hpp, and the simplification is slic3r_api::polygons_simplify. Each per-radius computation allocates its temporary
   collections in a scratch storage (with_scratch_storage), which is destroyed when the computation returns; the areas to keep are
   moved into the run storage by the caches when they are published. Per-layer immutable outline and radius-zero surface
   unions are built lazily in run storage instead: call_once publishes each complete union to readers, and local
   temporary collections are released on return or exception so failed initialization can be retried.
*/

namespace Slic3r::TreeFamily {

using slic3r_api::PolygonCollection;
using slic3r_api::StoredExPolygonCollection;
using slic3r_api::StoredPolygonCollection;
namespace Common = slic3r_api::Support::Common;

namespace {
// One cut of a maximum move into steps no longer than a step size: the count, the step, and the last step, which takes the rest.
struct MoveSteps {
    int   count;
    float step;
    float last;
};

// Cuts a maximum move into steps. A very short last step is avoided by stretching the other steps instead.
MoveSteps split_move(const float max_move, const float move_step);

MoveSteps split_move(const float max_move, const float move_step)
{
    MoveSteps steps{ std::max(1, round_up_divide<int>(max_move, move_step)), move_step, 0.f };
    steps.last = max_move - float(steps.count - 1) * move_step;
    if (steps.last < scale_d(0.05) && steps.count > 1) {
        steps.step = max_move / float(-- steps.count);
        steps.last = steps.step;
    }
    return steps;
}

// The table of the radii that TreeVolumes::ceilRadius() rounds to (sorted, ascending).
//
// It starts from the scale of the tips: min_radius, then steps of equal size up to and including the first step beyond
// SUPPORT_TREE_EXPONENTIAL_THRESHOLD (0.4; 1.5; 2.6 mm with the defaults, steps of 1.1 mm), then a factor of 1.5 for each
// following step (3.9; 5.85; 8.775 mm ...). The scale is generated until it passes both band_end and branch_radius.
//
// The branches widen into their foot on the build plate between branch_radius and bp_radius. The validity of a foot (and of the
// cone that leads to it) is checked on the table steps, so the table has a step every foot_step in the band [band_start, band_end].
// foot_step is TreeSettings::foot_cache_step, half the diameter of the largest support nozzle (0.2 mm for 0.4 mm nozzles).
// Without a positive foot_step there is no band. The band is delimited by:
// - band_end = bp_radius: the foot is at most bp_radius;
// - band_start = the scale step that branch_radius rounds up to (1.5 mm with the defaults), so that a branch of radius branch_radius
//   never has fine steps.
// The table is the scale steps strictly below band_start (the radii below it keep the scale steps, which the merging and the
// radius growth of the tips are tuned on), then band_start, band_start + foot_step, ... up to band_end inclusive (the last step is
// band_end even when it is not aligned), then the scale steps strictly above band_end.
// Without a band (bp_radius <= branch_radius, or band_start >= band_end) the table is the scale.
std::vector<coord_t> build_radius_steps(const coord_t min_radius, const coord_t branch_radius, const coord_t bp_radius, const coord_t foot_step);

std::vector<coord_t> build_radius_steps(const coord_t min_radius, const coord_t branch_radius, const coord_t bp_radius, const coord_t foot_step)
{
    const coord_t band_end = bp_radius;
    // The scale is generated a step beyond the band: the lookup continues the factor of 1.5 itself above the table.
    std::vector<coord_t> scale{ min_radius };
    const coord_t initial_radius_delta = SUPPORT_TREE_EXPONENTIAL_THRESHOLD - min_radius;
    coord_t       out = min_radius;
    if (initial_radius_delta > SUPPORT_TREE_COLLISION_RESOLUTION) {
        const int num_steps = round_up_divide(initial_radius_delta, SUPPORT_TREE_EXPONENTIAL_THRESHOLD);
        const int stepsize  = initial_radius_delta / num_steps;
        // num_steps + 1 steps of the same size, the last one being the start of the factor.
        for (int step = 0; step <= num_steps; ++ step) {
            out += stepsize;
            scale.emplace_back(out);
        }
    } else {
        out += SUPPORT_TREE_COLLISION_RESOLUTION;
        scale.emplace_back(out);
    }
    while (out <= std::max(band_end, branch_radius)) {
        out = coord_t(out * SUPPORT_TREE_EXPONENTIAL_FACTOR);
        scale.emplace_back(out);
    }
    if (bp_radius <= branch_radius || foot_step <= 0)
        return scale;
    // The scale step that branch_radius rounds up to (the scale was generated beyond it).
    const coord_t band_start = *std::lower_bound(scale.begin(), scale.end(), branch_radius);
    if (band_start >= band_end)
        return scale;

    std::vector<coord_t> steps;
    for (coord_t radius : scale)
        if (radius < band_start)
            steps.emplace_back(radius);
    for (coord_t radius = band_start; radius < band_end; radius += foot_step)
        steps.emplace_back(radius);
    steps.emplace_back(band_end);
    for (coord_t radius : scale)
        if (radius > band_end)
            steps.emplace_back(radius);
    return steps;
}
} // namespace

// Copies settings and caches simplified real-layer rings in run_storage, reserving empty per-layer union collections for lazy initialization; views are borrowed during construction. Cancellation
// and allocation failures propagate; the supplied storage and the context, when non-null, must outlive the resulting volumes.
TreeVolumes::TreeVolumes(const slic3r_api::Object &object, const slic3r_api::Print &print, const TreeSettings &settings, storage_handle *run_storage,
                         const plugin_run_context *context) :
    m_context(context),
    m_run_storage(run_storage),
    m_min_resolution(settings.resolution),
    m_support_rests_on_model(settings.support_rests_on_model),
    m_radius_steps(build_radius_steps(settings.min_radius, settings.branch_radius, settings.bp_radius,
                                      settings.foot_cache_step)),
    m_settings(settings),
    m_collision_cache(run_storage),
    m_collision_cache_holefree(run_storage),
    m_avoidance_cache(run_storage),
    m_avoidance_cache_slow(run_storage),
    m_avoidance_cache_to_model(run_storage),
    m_avoidance_cache_to_model_slow(run_storage),
    m_placeable_areas_cache(run_storage),
    m_avoidance_cache_holefree(run_storage),
    m_avoidance_cache_holefree_to_model(run_storage),
    m_wall_restrictions_cache(run_storage)
{
    // The constructor does the two once-only readings of the object, in the run storage: it slices the support blockers and it
    // simplifies the outline of every layer. Unions for offsets are computed at first use of each layer.

    // The support blockers of every layer: the union of all the blocker volumes, which are sliced at the slice Z of the layers.
    // The slices use the print resolution to clean the outlines, where the native PrintObject::slice_support_blockers() uses the
    // default one: a contour lies within `resolution` of the native contour.
    const std::vector<float> slice_zs = Common::layer_slice_zs(object);
    const c_mesh_slicing_params slicing_params = Common::support_modifier_slicing_params(print, object);
    // One scratch storage holds blocker-union and outline-simplification temporaries; their results move to run storage.
    const uint32_t layer_count = object.layer_count();
    m_layer_outlines.reserve(layer_count);
    m_layer_outline_regions.reserve(layer_count);
    m_layer_outline_regions_once.resize(layer_count);
    m_placeable_regions.reserve(layer_count);
    m_placeable_regions_once.resize(layer_count);
    for (uint32_t layer_idx = 0; layer_idx < layer_count; ++layer_idx) {
        m_layer_outline_regions.emplace_back(run_storage);
        m_placeable_regions.emplace_back(run_storage);
    }
    with_scratch_storage([&](storage_handle *scratch_storage) {
        for (uint32_t volume_idx = 0; volume_idx < object.volume_count(); ++ volume_idx) {
            const slic3r_api::Volume volume = object.volume(volume_idx);
            if (volume.type() != RAW_VOLUME_TYPE_SUPPORT_BLOCKER)
                continue;
            // The helper checks the context before and after slicing the volume and between its cleaned layers; the slicing of one mesh checks it too, at the coarse granularity of the native slicer.
            std::vector<StoredExPolygonCollection> slices = Common::slice_support_modifier_volume(volume, slice_zs, slicing_params, run_storage, m_context);
            if (slices.empty())
                continue;
            if (m_anti_overhang.empty()) {
                m_anti_overhang = std::move(slices);
                continue;
            }
            // Every volume is sliced at the same Z values, so the sizes are equal; only the common layers are merged if they ever differ.
            assert(slices.size() == m_anti_overhang.size());
            const size_t merged_layers = std::min(slices.size(), m_anti_overhang.size());
            for (size_t layer_idx = 0; layer_idx < merged_layers; ++ layer_idx) {
                if (m_anti_overhang[layer_idx].empty())
                    m_anti_overhang[layer_idx] = std::move(slices[layer_idx]);
                else if (! slices[layer_idx].empty()) {
                    // The union is computed in the scratch storage and its content moved into the collection of the run storage.
                    StoredExPolygonCollection both(scratch_storage);
                    both.append_copy_from(m_anti_overhang[layer_idx]);
                    both.append_copy_from(slices[layer_idx]);
                    StoredExPolygonCollection united = expolygons_union(scratch_storage, both);
                    m_anti_overhang[layer_idx].move_from(united);
                }
            }
        }

        // The outline of every layer, simplified with the resolution of the settings, moved to the run storage.
        for (uint32_t layer_idx = 0; layer_idx < layer_count; ++ layer_idx) {
            const StoredPolygonCollection rings = to_polygons(scratch_storage, object.layer(layer_idx).slices());
            StoredPolygonCollection simplified = slic3r_api::polygons_simplify(scratch_storage, rings, double(settings.resolution));
            m_layer_outlines.emplace_back(moved_to_storage(run_storage, simplified));
            slic3r_api::throw_if_cancelled(m_context);
        }
    });
    // The helper that slices the blockers may drop a cancellation at the boundary of its job: no half-built object leaves the constructor.
    throw_if_interrupted();
    // The maximum move keeps a margin of 2 scaled units (2 nm) against rounding errors.
    for (coord_t move : settings.max_move)
        m_max_move.push_back(std::max<coord_t>(move - 2, 0));
    for (coord_t move : settings.max_move_slow)
        m_max_move_slow.push_back(std::max<coord_t>(move - 2, 0));
}

// Unites a valid layer's borrowed simplified rings once in run storage and returns the immutable result; no cache mutex is held,
// local temporaries are released on exit, and cancellation/allocation failures leave call_once retryable without publishing a partial union.
const StoredExPolygonCollection &TreeVolumes::outline_regions(LayerIndex layer_idx)
{
    std::call_once(m_layer_outline_regions_once[layer_idx], [this, layer_idx] {
        throw_if_interrupted();
        StoredExPolygonCollection united = expolygons_union(m_run_storage, m_layer_outlines[layer_idx]);
        throw_if_interrupted();
        m_layer_outline_regions[layer_idx] = std::move(united);
    });
    return m_layer_outline_regions[layer_idx];
}

// Computes the caches for the keys (ceiled radius, layer) that a tree whose top layer is max_layer can request: each radius is
// computed from the lowest layer up to the highest layer where it can occur. For each layer L, the radii a branch can have there
// run from the path radius (TreeSettings::path_gain() on the real layer heights: a tip on max_layer that grew at every step) to
// that radius plus the foot excess (foot_target[L] - branch_radius, 0 away from the bed); every step of the table (ceilRadius())
// in that range is a key. The range covers the catch-up of a branch and the partial widening of
// its foot, which stop on table steps. A branch that pauses its growth has a radius below the path radius: those keys, and any
// other that the range misses, are computed on demand by the getter. The function also sets the members that the getters read
// without a lock (m_precalculated), so it must complete before the getters are called concurrently. The areas stay in the
// caches, which own them: the getters return references to them. A cancellation of the context (PluginCancelled) propagates to
// the caller; the keys published before it stay, and a key that was not computed is computed by the getter that asks for it.
// Precondition: max_layer is a valid object layer.
void TreeVolumes::precalculate(const coord_t max_layer)
{
    m_precalculated = true;
    const TreeSettings &config = m_settings;

    slic3r_api::throw_if_cancelled(m_context);

    // The radius each distance to top may have, keyed by the highest layer where it is needed. The simulation runs from the top
    // layer down, so that the caches cover exactly the layers a tree can reach.
    std::unordered_map<coord_t, LayerIndex> radius_until_layer;
    for (LayerIndex distance_to_top = 0; distance_to_top <= max_layer; ++ distance_to_top) {
        const LayerIndex current_layer = max_layer - distance_to_top;
        auto update_radius_until_layer = [&radius_until_layer, current_layer](coord_t r) {
            // A zero radius needs no area.
            if (r <= 0)
                return;
            std::unordered_map<coord_t, LayerIndex>::iterator it = radius_until_layer.find(r);
            if (it == radius_until_layer.end())
                radius_until_layer.emplace_hint(it, r, current_layer);
            else
                assert(it->second >= current_layer);
        };
        const coord_t path_radius = coord_t(config.min_radius + config.path_gain(current_layer, distance_to_top));
        // Upper bound of the radius a branch can have here: the path without pause, plus all the foot the layer allows (the foot
        // excess, foot_target - branch_radius; the foot only grows from a drawn radius of branch_radius), and a couple of scaled units of margin against the roundings.
        const coord_t foot_excess = config.foot_target[current_layer] > double(config.branch_radius) && path_radius >= config.branch_radius ?
            coord_t(config.foot_target[current_layer]) - config.branch_radius : 0;
        const coord_t upper_radius = path_radius + foot_excess + 2;
        // Every step of the table between the path radius and the bound: the catch-up and the partial foot of a branch can stop on any
        // of them.
        const coord_t last_step = ceilRadius(upper_radius);
        for (coord_t step = ceilRadius(path_radius); ; step = ceilRadius(step + 1)) {
            update_radius_until_layer(step);
            if (step >= last_step)
                break;
        }
    }

    slic3r_api::throw_if_cancelled(m_context);

    std::vector<RadiusLayerPair> relevant_avoidance_radiis{ radius_until_layer.begin(), radius_until_layer.end() };

    // The collision of the branch radius is needed for the holefree collisions, and the one of radius 0 for the xy distance checks.
    radius_until_layer[ceilRadius(m_settings.branch_radius)] = max_layer;
    radius_until_layer[0] = max_layer;

    std::vector<RadiusLayerPair> relevant_collision_radiis{ radius_until_layer.begin(), radius_until_layer.end() };
    calculateCollision(relevant_collision_radiis);

    std::vector<RadiusLayerPair> relevant_hole_collision_radiis;
    for (RadiusLayerPair key : relevant_avoidance_radiis)
        if (key.first < m_settings.branch_radius)
            relevant_hole_collision_radiis.emplace_back(key);
    calculateCollisionHolefree(relevant_hole_collision_radiis);

    // Placeable areas are computed from the avoidances, which request them for the layers they need.
    if (m_support_rests_on_model)
        calculatePlaceables(relevant_avoidance_radiis);

    {
        // Avoidance and wall restrictions run as separate tasks and join here with no lock held.
        tbb::task_group task_group;
        task_group.run([this, relevant_avoidance_radiis]{ calculateAvoidance(relevant_avoidance_radiis, true, m_support_rests_on_model); });
        task_group.run([this, relevant_avoidance_radiis]{ calculateWallRestrictions(relevant_avoidance_radiis); });
        task_group.wait();
    }
}

void TreeVolumes::calculateCollision(const std::vector<RadiusLayerPair> &keys)
{
    // Work per radius key varies with its layer range; the default partitioner balances these ranges.
    tbb::parallel_for(tbb::blocked_range<size_t>(0, keys.size()),
        [&](const tbb::blocked_range<size_t> &range) {
        for (size_t ikey = range.begin(); ikey != range.end(); ++ ikey) {
            const LayerIndex radius        = keys[ikey].first;
            const size_t     max_layer_idx = keys[ikey].second;
            // recursive call to parallel_for.
            calculateCollision(radius, max_layer_idx);
        }
    });
}

// Computes the collision of one radius on layers 0 to max_layer_idx: the outline grown by the radius and the XY distance, then
// merged over the Z gaps. The placeable areas of radius 0 come from the same outlines. The whole range is computed whatever the
// cache holds, and the result is inserted without replacing what another thread inserted meanwhile. max_layer_idx must name an
// object layer; cache storage owns published polygons, and cancellation or allocation failures propagate to the caller.
void TreeVolumes::calculateCollision(const coord_t radius, const LayerIndex max_layer_idx)
{
    // The layers are computed by parallel loops. Each chunk of a loop works in a scratch storage of its own, so that the chunks do not wait
    // for each other on the lock of a storage; the results are moved into the collections of result_storage, which are allocated before
    // the loops and hold them until they are published.
    with_scratch_storage([&](storage_handle *result_storage) {
        const std::vector<StoredPolygonCollection> &outlines = m_layer_outlines;
        const coord_t xy_distance = m_settings.xy_distance;
        const std::vector<uint32_t> &bottom_windows = m_settings.z_distance_bottom_layers;
        const std::vector<uint32_t> &top_windows = m_settings.z_distance_top_layers;
        assert(max_layer_idx >= 0 && max_layer_idx < LayerIndex(top_windows.size()) && max_layer_idx < LayerIndex(bottom_windows.size()));

        // Layer range for which the collisions will be calculated.
        LayerPolygonCache data;
        data.allocate(result_storage, 0, max_layer_idx + 1);

        const bool calculate_placable = m_support_rests_on_model && radius == 0;
        LayerPolygonCache data_placeable;
        if (calculate_placable)
            data_placeable.allocate(result_storage, data.begin(), data.end());

        // 1) Offset the outlines of the layers that the sums below use.
        LayerPolygonCache collision_areas_offsetted;
        const LayerIndex max_bottom = LayerIndex(*std::max_element(bottom_windows.begin() + data.begin(), bottom_windows.begin() + data.end()));
        const LayerIndex max_top = LayerIndex(*std::max_element(top_windows.begin() + data.begin(), top_windows.begin() + data.end()));
        collision_areas_offsetted.allocate(result_storage,
            std::max<LayerIndex>(0, data.begin() - max_bottom),
            std::min<LayerIndex>(LayerIndex(outlines.size()), data.end() + max_top));
        tbb::parallel_for(tbb::blocked_range<LayerIndex>(collision_areas_offsetted.begin(), collision_areas_offsetted.end()),
            [&outlines, offset_value = radius + xy_distance, &collision_areas_offsetted, this]
            (const tbb::blocked_range<LayerIndex> &range) {
                with_scratch_storage([&](storage_handle *storage) {
                    for (LayerIndex layer_idx = range.begin(); layer_idx != range.end(); ++ layer_idx) {
                        // jtRound is not needed here, as the overshoot can not cause errors in the algorithm, because no assumptions are made about the model.
                        if (offset_value == 0)
                            collision_areas_offsetted[layer_idx].move_from(polygons_union(storage, outlines[layer_idx]));
                        else {
                            const StoredExPolygonCollection &regions = outline_regions(layer_idx);
                            collision_areas_offsetted[layer_idx].move_from(polygons_offset(storage, regions, offset_value, CLIPPER_JOIN_MITER, 1.2));
                        }
                        slic3r_api::throw_if_cancelled(m_context);
                    }
                });
            });

        // 2) Sum the outlines over the Z gaps of each layer. The outlines above are offset by a distance that shrinks with the
        // layer distance, so that a branch may come as close to the layers above as the xy distance of the overhang requires.
        // Each real-Z window includes layers closer than the gap, within Z rounding tolerance;
        // the native XY ramp uses n = top[L], virtual layers included, as its denominator.
        tbb::parallel_for(tbb::blocked_range<LayerIndex>(data.begin(), data.end()),
            [&collision_areas_offsetted, &outlines, &anti_overhang = m_anti_overhang, radius, xy_distance, &bottom_windows, &top_windows,
                min_resolution = m_min_resolution, &data, this]
            (const tbb::blocked_range<LayerIndex>& range) {
                with_scratch_storage([&](storage_handle *storage) {
                    for (LayerIndex layer_idx = range.begin(); layer_idx != range.end(); ++ layer_idx) {
                        StoredPolygonCollection collisions(storage);
                        assert(layer_idx >= 0 && layer_idx < LayerIndex(top_windows.size()) && layer_idx < LayerIndex(bottom_windows.size()));
                        const int z_distance_bottom_layers = int(bottom_windows[layer_idx]);
                        const int z_distance_top_layers = int(top_windows[layer_idx]);
                        for (int i = - z_distance_bottom_layers; i <= 0; ++ i)
                            if (int j = layer_idx + i; collision_areas_offsetted.has(j))
                                collisions.append_copy_from(collision_areas_offsetted[j]);
                        for (int i = 1; i <= z_distance_top_layers; ++ i)
                            if (int j = layer_idx + i; j < int(outlines.size())) {
                                // The conditional -0.5 keeps plastic from touching diagonally below a layer when the Z gap is one layer:
                                // an overhang below 90 degrees is then left unsupported rather than fused to the object.
                                const coord_t required_range_x =
                                    (xy_distance - ((i - (z_distance_top_layers == 1 ? 0.5 : 0)) * xy_distance / z_distance_top_layers));
                                const StoredExPolygonCollection &regions = outline_regions(j);
                                StoredPolygonCollection grown = polygons_offset(storage, regions, radius + required_range_x, CLIPPER_JOIN_MITER, 1.2);
                                collisions.append_move_from(grown);
                            }
                        StoredPolygonCollection merged = layer_idx < int(anti_overhang.size()) ?
                            polygons_union(storage, collisions, polygons_offset(storage, anti_overhang[layer_idx], radius, CLIPPER_JOIN_MITER, 1.2)) :
                            polygons_union(storage, collisions);
                        data[layer_idx].move_from(slic3r_api::polygons_simplify(storage, merged, double(min_resolution)));
                        slic3r_api::throw_if_cancelled(m_context);
                    }
                });
            });

        // 3) The placeable areas: the top surface of the layer below, minus the collision of this layer, inset by the XY distance.
        if (calculate_placable) {
            tbb::parallel_for(tbb::blocked_range<LayerIndex>(data.begin(), data.end()),
                [&collision_areas_offsetted, &outlines, &anti_overhang = m_anti_overhang, min_resolution = m_min_resolution,
                    &bottom_windows, xy_distance, &data_placeable, this]
                (const tbb::blocked_range<LayerIndex>& range) {
                    with_scratch_storage([&](storage_handle *storage) {
                        for (LayerIndex layer_idx = range.begin(); layer_idx != range.end(); ++ layer_idx) {
                            assert(layer_idx >= 0 && layer_idx < LayerIndex(bottom_windows.size()));
                            const LayerIndex layer_idx_below = layer_idx - LayerIndex(bottom_windows[layer_idx]) - 1;
                            if (layer_idx_below < 0)
                                continue;
                            const StoredPolygonCollection &current = collision_areas_offsetted[layer_idx];
                            const StoredPolygonCollection &below   = outlines[layer_idx_below];
                            // Inflate the surface to sit on by the separation distance to increase chance of a support being placed on a sloped surface.
                            // A zero distance keeps the surface: the offset of a clean list by 0 is not supported.
                            const StoredPolygonCollection surface = xy_distance == 0 ? below.readonly().clone(storage) : polygons_offset(storage, below, xy_distance);
                            const StoredPolygonCollection free_area = layer_idx_below < int(anti_overhang.size()) ?
                                polygons_diff(storage, surface, polygons_union(storage, current, anti_overhang[layer_idx_below])) :
                                polygons_diff(storage, surface, current);
                            data_placeable[layer_idx].move_from(slic3r_api::polygons_simplify(storage, free_area, double(min_resolution)));
                            slic3r_api::throw_if_cancelled(m_context);
                        }
                    });
                });
        }

        throw_if_interrupted();
        m_collision_cache.insert(std::move(data), radius);
        if (calculate_placable)
            m_placeable_areas_cache.insert(std::move(data_placeable), radius);
    });
}

// Computes, for each key (radius, last layer), the collision of that radius on layers 0 to the last layer without the holes too small
// for a branch of the branch radius. The radii are ceiled and smaller than the branch radius. The layers are published in chunks, each
// chunk with the layers of all the keys; a cancellation of the context (PluginCancelled) propagates, and the chunks published before
// it stay (a missing layer is computed by getCollisionHolefree).
void TreeVolumes::calculateCollisionHolefree(const std::vector<RadiusLayerPair> &keys)
{
    // Without any key nothing is to compute, and the key count is the grainsize of the parallel loop below, which TBB requires to be
    // positive. There is no key when every radius reaches the branch radius. For instance, the tip radius is clamped to the
    // branch radius when the tip diameter is at least the branch diameter (TreeSettings.cpp), so every radius is at least the
    // branch radius.
    if (keys.empty())
        return;

    LayerIndex max_layer = 0;
    for (size_t i = 0; i < keys.size(); i++)
        max_layer = std::max(max_layer, keys[i].second);

    // The layers below grow the collision of the branch radius. When it is not cached, one computation of its whole range
    // fills it, instead of one computation per layer in the loop below.
    getCollision(m_settings.branch_radius, max_layer);

    // The grain is the key count; each chunk computes and publishes its layer results together.
    tbb::parallel_for(tbb::blocked_range<LayerIndex>(0, max_layer + 1, keys.size()),
        [&](const tbb::blocked_range<LayerIndex> &range) {
        // The scratch storage of the chunk holds its results and its temporaries; the cache moves the results to the run storage.
        with_scratch_storage([&](storage_handle *storage) {
            std::vector<std::pair<RadiusLayerPair, StoredPolygonCollection>> data;
            data.reserve(range.size() * keys.size());
            for (LayerIndex layer_idx = range.begin(); layer_idx < range.end(); ++ layer_idx) {
                for (RadiusLayerPair key : keys)
                    if (layer_idx <= key.second) {
                        // Logically increase the collision by the branch radius
                        coord_t radius = key.first;
                        assert(radius == this->ceilRadius(radius));
                        assert(radius < m_settings.branch_radius);
                        coord_t increase_radius_ceil = ceilRadius(m_settings.branch_radius) - radius;
                        assert(increase_radius_ceil > 0);
                        // this union is important as otherwise holes(in form of lines that will increase to holes in a later step) can get unioned onto the area.
                        data.emplace_back(RadiusLayerPair(radius, layer_idx), StoredPolygonCollection(storage));
                        const StoredExPolygonCollection regions = expolygons_union(storage, this->getCollision(m_settings.branch_radius, layer_idx));
                        const StoredPolygonCollection shrunk = polygons_offset(storage, regions, 5 - increase_radius_ceil, CLIPPER_JOIN_ROUND, m_min_resolution);
                        data.back().second.move_from(slic3r_api::polygons_simplify(storage, shrunk, double(m_min_resolution)));
                        slic3r_api::throw_if_cancelled(m_context);
                    }
            }
            // A throw in the loop leaves before this point; the guard keeps a chunk that was cancelled (host flag or cancelled TBB group,
            // whose nested loops are cut short) from being published.
            throw_if_interrupted();
            m_collision_cache_holefree.insert(std::move(data));
        });
    });
}

// Propagates the avoidance of each key upwards, one object layer at a time. Between layer_idx - 1 and layer_idx the avoidance may
// shrink by the maximum move of layer_idx, which is the height of that layer times the branch angle. The propagation of a key
// starts from the collision of layer 0, which is the avoidance of layer 0, and computes layers 1 up to the requested layer.
void TreeVolumes::calculateAvoidance(const std::vector<RadiusLayerPair> &keys, bool to_build_plate, bool to_model)
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
            (! task.holefree() || task.radius < m_settings.branch_radius))
            avoidance_tasks.emplace_back(task);
    }

    slic3r_api::throw_if_cancelled(m_context);

    // Grain-one ranges let the default partitioner distribute whole propagation tasks.
    tbb::parallel_for(tbb::blocked_range<size_t>(0, avoidance_tasks.size(), 1),
        [this, &avoidance_tasks, first_layer](const tbb::blocked_range<size_t> &range) {
        for (size_t task_idx = range.begin(); task_idx < range.end(); ++ task_idx) {
            const AvoidanceTask &task = avoidance_tasks[task_idx];
            assert(! task.holefree() || task.radius < m_settings.branch_radius);
            if (task.to_model)
                getPlaceableAreas(task.radius, task.max_required_layer);
            // The following loop propagating avoidance regions bottom up is inherently serial.
            const bool collision_holefree = (task.slow() || task.holefree()) && task.radius < m_settings.branch_radius;
            // The collision that the avoidance of this task merges with, on a layer.
            const auto collision_of_layer = [this, &task, collision_holefree](LayerIndex layer_idx) -> const StoredPolygonCollection& {
                return collision_holefree ? getCollisionHolefree(task.radius, layer_idx) : getCollision(task.radius, layer_idx);
            };
            // When the collisions are not cached, one computation of their whole range fills them, instead of one per layer below.
            collision_of_layer(task.max_required_layer);
            const std::vector<coord_t> &max_moves = task.slow() ? m_max_move_slow : m_max_move;
            // Limiting the offset step so that unioning the shrunk latest_avoidance with the current layer collisions
            // will not create gaps in the resulting avoidance region letting a tree support branch tunneling through an object wall.
            // For a wall of thickness w, this stays below w + 2 * (radius + printed XY clearance), the collision band.
            const float move_step = 1.9f * float(std::max(task.radius, m_settings.xy_distance));
            // Threshold must be >= 0.5 to prevent division by zero when move_step is cast to int in round_up_divide. A branch below
            // it (zero radius and zero printed XY clearance) has no step to propagate with: its avoidance is its collision. It is published
            // like the others, as the getter that asked for it reads it back.
            const bool can_move = move_step >= 0.5f;
            // The scratch storage of the task holds the avoidance being propagated, the temporaries of every layer and the results;
            // the cache moves the results to the run storage. Each result is a copy of latest_avoidance, which the next layer reads.
            with_scratch_storage([&](storage_handle *storage) {
                std::vector<std::pair<RadiusLayerPair, StoredPolygonCollection>> data;
                data.reserve(task.max_required_layer + 1 - first_layer);
                // The avoidance of layer 0 is the collision.
                StoredPolygonCollection latest_avoidance = getAvoidance(task.radius, first_layer - 1, task.type, task.to_model).readonly().clone(storage);
                for (LayerIndex layer_idx = first_layer; layer_idx <= task.max_required_layer; ++ layer_idx) {
                    // Merge current layer collisions with shrunk last_avoidance.
                    const StoredPolygonCollection &current_layer_collisions = collision_of_layer(layer_idx);
                    // The collection the next operation reads: each result is assigned to latest_avoidance after it is materialized.
                    const StoredPolygonCollection *operand = &latest_avoidance;
                    if (can_move) {
                        // For mildly steep branch angles only one step will be taken.
                        const MoveSteps steps = split_move(float(max_moves[layer_idx]), move_step);
                        for (int istep = 0; istep < steps.count; ++ istep) {
                            const float shrink = istep + 1 == steps.count ? - steps.last : - steps.step;
                            if (shrink == 0.f)
                                // A zero move (a branch that may not move on this layer) keeps the avoidance: the offset by 0 is not supported.
                                latest_avoidance = polygons_union(storage, current_layer_collisions, latest_avoidance);
                            else {
                                const StoredPolygonCollection shrunk = polygons_offset(storage, latest_avoidance, shrink, CLIPPER_JOIN_ROUND, m_min_resolution);
                                latest_avoidance = polygons_union(storage, current_layer_collisions, shrunk);
                            }
                        }
                    } else
                        operand = &current_layer_collisions;
                    if (task.to_model) {
                        latest_avoidance = polygons_diff(storage, *operand, getPlaceableAreas(task.radius, layer_idx));
                        operand = &latest_avoidance;
                    }
                    latest_avoidance = slic3r_api::polygons_simplify(storage, *operand, double(m_min_resolution));
                    data.emplace_back(RadiusLayerPair{task.radius, layer_idx}, latest_avoidance.readonly().clone(storage));
                    slic3r_api::throw_if_cancelled(m_context);
                }
                throw_if_interrupted();
                avoidance_cache(task.type, task.to_model).insert(std::move(data));
            });
        }
    });
}

void TreeVolumes::calculatePlaceables(const std::vector<RadiusLayerPair> &keys)
{
    // The default partitioner balances radius keys whose layer ranges have unequal cost.
    tbb::parallel_for(tbb::blocked_range<size_t>(0, keys.size()),
        [&, keys](const tbb::blocked_range<size_t>& range) {
            for (size_t key_idx = range.begin(); key_idx < range.end(); ++ key_idx)
                this->calculatePlaceables(keys[key_idx].first, keys[key_idx].second);
        });
}

// Computes the placeable areas of one radius on layers 0 to max_required_layer, from the ones of radius 0 shrunk by the radius and
// by the XY distance. Layer 0 has no placeable area, as nothing lies below it: its entry stays empty, so that a request for
// it finds its key.
void TreeVolumes::calculatePlaceables(const coord_t radius, const LayerIndex max_required_layer)
{
    with_scratch_storage([&](storage_handle *result_storage) {
        std::vector<StoredPolygonCollection> data;
        data.reserve(max_required_layer + 1);
        for (LayerIndex layer_idx = 0; layer_idx <= max_required_layer; ++ layer_idx)
            data.emplace_back(result_storage);

        // Request only the radius-zero range needed by this calculation, preserving on-demand cache boundaries.
        getPlaceableAreas(0, max_required_layer);
        tbb::parallel_for(tbb::blocked_range<LayerIndex>(1, max_required_layer + 1),
            [this, &data, radius](const tbb::blocked_range<LayerIndex>& range) {
                with_scratch_storage([&](storage_handle *storage) {
                    for (LayerIndex layer_idx = range.begin(); layer_idx < range.end(); ++ layer_idx) {
                        // Publishes this layer's union once in run storage before readers proceed; getters hold no cache lock
                        // during computation, and call_once retries after cancellation or allocation failures.
                        std::call_once(m_placeable_regions_once[layer_idx], [this, layer_idx] {
                            StoredExPolygonCollection united = expolygons_union(m_run_storage, getPlaceableAreas(0, layer_idx));
                            throw_if_interrupted();
                            m_placeable_regions[layer_idx] = std::move(united);
                        });
                        const StoredExPolygonCollection &regions = m_placeable_regions[layer_idx];
                        data[layer_idx].move_from(polygons_offset(storage, regions,
                            // As a placeable area is calculated by (collision of the layer below) - (collision of the current layer) and the collision is offset by xy_distance,
                            // it can happen that a small line is considered a flat area to place something onto, even though it is mostly
                            // xy_distance that cant support it. Making the area smaller by xy_distance fixes this.
                            - (radius + m_settings.xy_distance),
                            CLIPPER_JOIN_MITER, 1.2));
                        slic3r_api::throw_if_cancelled(m_context);
                    }
                });
            });
        // The inner loop is skipped, not failed, when the enclosing TBB group is cancelled: it must not publish its empty layers.
        throw_if_interrupted();
        m_placeable_areas_cache.insert(std::move(data), 0, radius);
    });
}

// Computes, for each key (radius, last layer), the wall restrictions of that radius on layers 1 to the last layer. The radii are ceiled. A key is
// published whole, after its loop; a cancellation of the context propagates, and the keys published before it stay.
void TreeVolumes::calculateWallRestrictions(const std::vector<RadiusLayerPair> &keys)
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

    // The default partitioner balances radius keys whose layer ranges have unequal cost.
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
            // When the collisions are not cached, one computation of their whole range fills them, instead of one per layer below.
            getCollision(0, max_required_layer);
            getCollision(radius, max_required_layer - 1);
            with_scratch_storage([&](storage_handle *result_storage) {
                std::vector<StoredPolygonCollection> data;
                data.reserve(buffer_size);
                for (size_t idx = 0; idx < buffer_size; ++ idx)
                    data.emplace_back(result_storage);
                tbb::parallel_for(tbb::blocked_range<LayerIndex>(first_layer, max_required_layer + 1),
                    [this, &data, radius, first_layer](const tbb::blocked_range<LayerIndex> &range) {
                    with_scratch_storage([&](storage_handle *storage) {
                        for (LayerIndex layer_idx = range.begin(); layer_idx < range.end(); ++ layer_idx) {
                            const StoredPolygonCollection walls = polygons_intersection(storage, getCollision(0, layer_idx), getCollision(radius, layer_idx - 1));
                            data[layer_idx - first_layer].move_from(slic3r_api::polygons_simplify(storage, walls, double(m_min_resolution)));
                            slic3r_api::throw_if_cancelled(m_context);
                        }
                    });
                });
                // The inner loop is skipped, not failed, when the enclosing TBB group is cancelled: it must not publish its empty layers.
                throw_if_interrupted();
                m_wall_restrictions_cache.insert(std::move(data), first_layer, radius);
            });
        }
    });
}

// Called after a computation, before its result is read or published. A nested tbb::parallel_for (also the one of
// slic3r_parallel_for_storage inside with_scratch_storage) is skipped or cut short, the exceptions of its tasks dropped, when the TBB group
// that runs the caller is cancelled, so the computation may have produced nothing: a getter would then miss its cache again and recurse without end,
// and a calculate*() function would publish layers that were never filled. A cancellation of the context throws PluginCancelled. A
// group cancelled by another exception (TBB keeps the first one and drops this one) throws a std::runtime_error, which only ends
// the call. No cache lock is held here.
void TreeVolumes::throw_if_interrupted() const
{
    slic3r_api::throw_if_cancelled(m_context);
    if (tbb::is_current_task_group_canceling())
        throw std::runtime_error("TreeVolumes: the enclosing TBB task group is cancelled");
}

// Gives the collision of a branch of orig_radius (>= 0), rounded up to its cache step by ceilRadius(), on the object layer layer_idx
// (0 to the layer count minus one). The reference is owned by the cache and stays valid until the TreeVolumes is destroyed. On a cache
// miss it computes the whole range of the key, from layer 0 to layer_idx, then reads the cache again; a cancellation of the context
// propagates, and what was published before it stays.
const StoredPolygonCollection& TreeVolumes::getCollision(const coord_t orig_radius, LayerIndex layer_idx) const
{
    const coord_t radius = this->ceilRadius(orig_radius);
    if (std::optional<std::reference_wrapper<const StoredPolygonCollection>> result = m_collision_cache.getArea({ radius, layer_idx }); result)
        return (*result).get();
    if (m_precalculated)
        BOOST_LOG_TRIVIAL(error) << "Had to calculate collision at radius " << radius << " and layer " << layer_idx << ", but precalculate was called. Performance may suffer!";
    const_cast<TreeVolumes*>(this)->calculateCollision(radius, layer_idx);
    throw_if_interrupted();
    return getCollision(orig_radius, layer_idx);
}

const StoredPolygonCollection& TreeVolumes::getCollisionHolefree(coord_t radius, LayerIndex layer_idx) const
{
    assert(radius == this->ceilRadius(radius));
    assert(radius < m_settings.branch_radius);
    if (std::optional<std::reference_wrapper<const StoredPolygonCollection>> result = m_collision_cache_holefree.getArea({ radius, layer_idx }); result)
        return (*result).get();
    if (m_precalculated)
        BOOST_LOG_TRIVIAL(error) << "Had to calculate collision holefree at radius " << radius << " and layer " << layer_idx << ", but precalculate was called. Performance may suffer!";
    const_cast<TreeVolumes*>(this)->calculateCollisionHolefree({ radius, layer_idx });
    throw_if_interrupted();
    return getCollisionHolefree(radius, layer_idx);
}

// Gives the avoidance of a branch of orig_radius (>= 0), rounded up to its cache step by ceilRadius(), on the object layer layer_idx
// (0 to the layer count minus one), to the build plate or, with to_model, to the model. Layer 0 gives the collision, and FastSafe falls
// back to Fast from the branch radius up, where there is no hole left. The reference is owned by the cache and stays valid until the
// TreeVolumes is destroyed. On a cache miss it computes the key from layer 1 to layer_idx, then reads the cache again; a cancellation of
// the context propagates, and what was published before it stays.
const StoredPolygonCollection& TreeVolumes::getAvoidance(const coord_t orig_radius, LayerIndex layer_idx, AvoidanceType type, bool to_model) const
{
    if (layer_idx == 0) // What on the layer directly above buildplate do i have to avoid to reach the buildplate ...
        return getCollision(orig_radius, layer_idx);

    const coord_t radius = this->ceilRadius(orig_radius);
    if (type == AvoidanceType::FastSafe && radius >= m_settings.branch_radius)
        // no holes anymore by definition at this request
        type = AvoidanceType::Fast;

    if (std::optional<std::reference_wrapper<const StoredPolygonCollection>> result =
            this->avoidance_cache(type, to_model).getArea({ radius, layer_idx });
        result)
        return (*result).get();

    if (m_precalculated) {
        if (to_model) {
            BOOST_LOG_TRIVIAL(error) << "Had to calculate Avoidance to model at radius " << radius << " and layer " << layer_idx << ", but precalculate was called. Performance may suffer!";
        } else {
            BOOST_LOG_TRIVIAL(error) << "Had to calculate Avoidance at radius " << radius << " and layer " << layer_idx << ", but precalculate was called. Performance may suffer!";
        }
    }
    const_cast<TreeVolumes*>(this)->calculateAvoidance({ radius, layer_idx }, ! to_model, to_model);
    throw_if_interrupted();
    // Retrive failed and correct result was calculated. Now it has to be retrived.
    return getAvoidance(orig_radius, layer_idx, type, to_model);
}

// Gives the placeable areas of a branch of orig_radius (rounded as the other getters do) on layer
// layer_idx >= 0. On a cache miss it computes the key, from layer 0 to layer_idx, then reads the cache again; the reference is
// owned by the cache. A cancellation of the context propagates, and what was published before it stays.
const StoredPolygonCollection& TreeVolumes::getPlaceableAreas(const coord_t orig_radius, LayerIndex layer_idx) const
{
    const coord_t radius = ceilRadius(orig_radius);
    if (std::optional<std::reference_wrapper<const StoredPolygonCollection>> result = m_placeable_areas_cache.getArea({ radius, layer_idx }); result)
        return (*result).get();
    if (m_precalculated)
        BOOST_LOG_TRIVIAL(error) << "Had to calculate Placeable Areas at radius " << radius << " and layer " << layer_idx << ", but precalculate was called. Performance may suffer!";
    if (orig_radius == 0) {
        // Placeable areas for radius 0 are calculated in the general collision code, which inserts them with the collisions. The
        // collision may be cached already while the thread that computed it is still inserting the placeable areas, so this runs
        // the computation (a duplicate is harmless) instead of reading the collision, and reads the placeable areas again below.
        if (! m_support_rests_on_model)
            // Nothing rests on the model, so that code computes no placeable areas, and the routing does not ask for them: the
            // collision is returned, which ends the request.
            return this->getCollision(0, layer_idx);
        const_cast<TreeVolumes*>(this)->calculateCollision(0, layer_idx);
    } else
        const_cast<TreeVolumes*>(this)->calculatePlaceables(radius, layer_idx);
    // The computation above inserted the area, or another thread did before.
    throw_if_interrupted();
    return getPlaceableAreas(orig_radius, layer_idx);
}

// Returns a cache-owned wall restriction for a nonnegative radius on layer > 0; missing entries are computed, and interruption propagates through the cache calculation.
const StoredPolygonCollection& TreeVolumes::getWallRestriction(const coord_t orig_radius, LayerIndex layer_idx) const
{
    assert(layer_idx > 0);

    const coord_t radius = ceilRadius(orig_radius);
    if (std::optional<std::reference_wrapper<const StoredPolygonCollection>> result =
        m_wall_restrictions_cache.getArea({ radius, layer_idx });
        result)
        return (*result).get();
    if (m_precalculated)
        BOOST_LOG_TRIVIAL(error) << "Had to calculate Wall restrictions at radius " << radius << " and layer " << layer_idx << ", but precalculate was called. Performance may suffer!";
    const_cast<TreeVolumes*>(this)->calculateWallRestrictions({ radius, layer_idx });
    throw_if_interrupted();
    return getWallRestriction(orig_radius, layer_idx); // Retrieve failed and correct result was calculated. Now it has to be retrieved.
}

// Rounds a radius (>= 0) up to the smallest step of the table of cache radii that is not below it; 0 stays 0. Above the last step of
// the table the steps keep growing by SUPPORT_TREE_EXPONENTIAL_FACTOR. It reads the table only, so any thread may call it.
coord_t TreeVolumes::ceilRadius(const coord_t radius) const
{
    if (radius == 0)
        return 0;

    std::vector<coord_t>::const_iterator it = std::lower_bound(m_radius_steps.begin(), m_radius_steps.end(), radius);
    if (it != m_radius_steps.end())
        return *it;
    // Above the table the exponential scale goes on from its last step.
    coord_t out = m_radius_steps.back();
    while (out < radius) {
        assert(out * SUPPORT_TREE_EXPONENTIAL_FACTOR > out + SUPPORT_TREE_COLLISION_RESOLUTION);
        out = coord_t(out * SUPPORT_TREE_EXPONENTIAL_FACTOR);
    }
    return out;
}

} // namespace Slic3r::TreeFamily
