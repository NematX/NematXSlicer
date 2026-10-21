///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* Collision, avoidance and placeable areas of the tree branches, cached per radius and object layer.

   A tree branch is a disc of some radius centred on a node placed on an object layer. TreeVolumes answers, for a
   radius and a layer, which 2D areas the node must not enter:
   - getCollision: the object and its support blockers grown by the radius and the XY distance, with the Z gaps
     above and below the layer. Each real-Z window includes layers closer than its gap, within Z rounding tolerance,
     and keeps the native XY ramp above the layer;
   - getAvoidance: the places from which a node cannot descend to the build plate (or to the model) within the
     maximum move of each layer. It is built upwards from the collision of the layers below, one layer at a time;
   - getPlaceableAreas: the top surfaces where a branch of this radius fits and may stand on the model;
   - getWallRestriction: the object walls a branch must not cross between two consecutive layers.

   Every area is computed for a (radius, layer) key and kept in a cache. precalculate() computes the keys a tree can
   request up to its top layer, so the router normally only reads the caches. A key that precalculate() did not cover is
   computed by the getter that misses it (see Concurrency).

   Cancellation: the constructor takes the plugin_run_context of the step (nullptr never cancels) and keeps it. Every computation,
   the ones of precalculate() and the lazy ones of the getters, calls slic3r_api::throw_if_cancelled() on it after each layer, so a
   cancelled step throws PluginCancelled from the call that is running. A computation that runs in a TBB group cancelled by another
   exception has its nested parallel loops skipped or cut short, and the exceptions of their tasks dropped; it detects the group with
   tbb::is_current_task_group_canceling() before it publishes or reads its result, and throws a std::runtime_error (TBB keeps the first
   exception of the group). Nothing partial is published when the host cancels (PluginCancelled); when the group is cancelled by another
   error, the step fails and these volumes are not reused. The context must outlive the TreeVolumes.

       TreeVolumes volumes(object, print, settings, run_storage, context);
       volumes.precalculate(max_layer);
       const StoredPolygonCollection &free = volumes.getAvoidance(radius, layer, TreeVolumes::AvoidanceType::Fast, false, min_xy_dist);

   Storage: the areas are StoredPolygonCollection values owned by the run storage given to the constructor, which must outlive the
   TreeVolumes (declare it first). A computation runs on a scratch storage of its own (slic3r_parallel_for_storage); its result is
   moved into the run storage when it is published, so that no handle of a scratch storage survives the computation.

   Concurrency: once the constructor and precalculate() have returned, the getters are called from many threads at once, and any
   of them may have to compute a missing key. The constructor and precalculate() must complete before the first concurrent getter:
   precalculate() writes members that the getters read without a lock (m_precalculated). Destroying a TreeVolumes
   needs exclusive access as well.
   - Each cache guards its maps with a short lock, taken only for a lookup or an insertion. No lock is held while an area is
     computed or while waiting for TBB tasks, as the computations run parallel_for and call other getters.
   - A getter that misses a key computes the whole range of that key, from its first layer to the requested layer, without
     reading what the caches hold, then inserts it. Insertion never replaces a value: the first one published wins. Two threads
     that compute the same key at the same time duplicate work, and both get the same (deterministic) areas. After its own
     computation, a getter finds its key in the cache.
   - A published area is never modified or moved while its TreeVolumes is alive: the layers are kept in a std::deque, which does
     not relocate its elements when it grows. A reference returned by a getter stays valid while other threads insert, until the
     TreeVolumes is destroyed.
   - The moves into the run storage happen before the cache lock is taken, and the areas that lose the insertion are freed after
     it is released: a cache lock is never held while a storage lock is taken. */
#ifndef plugins_support_tree_volumes_hpp_
#define plugins_support_tree_volumes_hpp_

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "TreeGeometry.hpp"
#include "TreeSettings.hpp"

namespace Slic3r {

namespace TreeFamily {

// Growth factor of the radii above the exponential threshold, and the radius step below it.
static constexpr const double  SUPPORT_TREE_EXPONENTIAL_FACTOR = 1.5;
static constexpr const coord_t SUPPORT_TREE_EXPONENTIAL_THRESHOLD = scale_i(1. * SUPPORT_TREE_EXPONENTIAL_FACTOR);
static constexpr const coord_t SUPPORT_TREE_COLLISION_RESOLUTION = scale_i(0.5);

class TreeVolumes
{
public:
    // Reads the object once, in run_storage: it slices the support blockers and simplifies the outline of every layer. No area is
    // computed until precalculate(). The areas are owned by run_storage, which must outlive this object. The context, which must
    // outlive it too, is checked before and after the slicing of each blocker volume and between its cleaned layers
    // (the slicing of one mesh checks it too, at the coarse granularity of the native slicer)
    // and by every later computation.
    TreeVolumes(const slic3r_api::Object &object, const slic3r_api::Print &print, const TreeSettings &settings, storage_handle *run_storage,
                const plugin_run_context *context);
    TreeVolumes(TreeVolumes&&) = delete;
    TreeVolumes& operator=(TreeVolumes&&) = delete;
    TreeVolumes(const TreeVolumes&) = delete;
    TreeVolumes& operator=(const TreeVolumes&) = delete;

    enum class AvoidanceType : int8_t
    {
        Slow,
        FastSafe,
        Fast,
        Count
    };

    /*!
     * \brief Precalculate the collisions, avoidances and placeable areas of every radius a tree can request up to max_layer.
     *
     * Knowledge about the branch angles and the radii of the tips limits the keys to the ones that may actually be used.
     * Requests for other keys still work from any thread, but they are computed on demand and are slower.
     */
    void precalculate(const coord_t max_layer);

    /*!
     * \brief Provides the areas that a node of the given radius must not enter on this layer, because they collide with the model.
     * \param radius The radius of the node of interest
     * \param layer_idx An object layer index, from zero through object layer count minus one.
     * \param min_xy_dist Is the minimum xy distance used.
     */
    const slic3r_api::StoredPolygonCollection& getCollision(const coord_t radius, LayerIndex layer_idx, bool min_xy_dist) const;

    /*!
     * \brief Provides the areas that a node of the given radius must avoid on this layer to still reach the build plate,
     * or the model when to_model is set.
     * \param radius The radius of the node of interest
     * \param layer_idx An object layer index, from zero through object layer count minus one.
     * \param type The avoidance speed and hole handling
     * \param to_model Does the avoidance allow good connections with the model.
     * \param min_xy_dist is the minimum xy distance used.
     */
    const slic3r_api::StoredPolygonCollection& getAvoidance(coord_t radius, LayerIndex layer_idx, AvoidanceType type, bool to_model, bool min_xy_dist) const;

    /*!
     * \brief Provides the area where a branch of this radius completely fits on the model on this layer.
     * Radius 0 gives the top surfaces of the object, which are computed together with the collision of radius 0.
     * Without resting on the model there is no such area, and the collision is returned for radius 0.
     * \param radius The radius of the node of interest
     * \param layer_idx An object layer index, from zero through object layer count minus one.
     */
    const slic3r_api::StoredPolygonCollection& getPlaceableAreas(coord_t radius, LayerIndex layer_idx) const;

    /*!
     * \brief Provides the abstract walls that a branch must not pass when it moves between two layers.
     * \param radius The radius of the node of interest.
     * \param layer_idx An object layer index, from zero through object layer count minus one.
     * \param min_xy_dist is the minimum xy distance used.
     */
    const slic3r_api::StoredPolygonCollection& getWallRestriction(coord_t radius, LayerIndex layer_idx, bool min_xy_dist) const;

    /*!
     * \brief Round \p radius upwards to a multiple of SUPPORT_TREE_COLLISION_RESOLUTION or to an exponentially increasing value.
     * The XY distance delta between min_xy_dist and the regular distance is added when min_xy_dist is not used.
     */
    coord_t ceilRadius(const coord_t radius, const bool min_xy_dist) const {
        assert(radius >= 0);
        return min_xy_dist ?
            this->ceilRadius(radius) :
            // A zero radius still needs the regular XY distance, so the collision stays as close to the configured values as possible.
            radius > 0 ? this->ceilRadius(radius + m_current_min_xy_dist_delta) : m_current_min_xy_dist_delta;
    }

    /*!
     * \brief Round \p radius upwards to the maximum that still rounds up to the same value as the provided one.
     */
    coord_t getRadiusNextCeil(coord_t radius, bool min_xy_dist) const {
        assert(radius > 0);
        return min_xy_dist ?
            this->ceilRadius(radius) :
            this->ceilRadius(radius + m_current_min_xy_dist_delta) - m_current_min_xy_dist_delta;
    }

private:
    // Areas of a range of layers computed on a scratch storage, indexed by layer. A collection stays in the storage that allocated it, so
    // the cache that receives them moves their content into new collections of the run storage when it publishes them; the range must
    // be destroyed before its scratch storage.
    class LayerPolygonCache {
    public:
        void allocate(storage_handle *storage, LayerIndex aidx_begin, LayerIndex aidx_end) {
            m_idx_begin = aidx_begin;
            m_idx_end = aidx_end;
            m_polygons.clear();
            m_polygons.reserve(size_t(aidx_end - aidx_begin));
            for (LayerIndex idx = aidx_begin; idx < aidx_end; ++ idx)
                m_polygons.emplace_back(storage);
        }

        LayerIndex begin() const { return m_idx_begin; }
        LayerIndex end()   const { return m_idx_end; }
        size_t     size()  const { return m_polygons.size(); }

        bool has(LayerIndex idx) const { return idx >= m_idx_begin && idx < m_idx_end; }
        slic3r_api::StoredPolygonCollection& operator[](LayerIndex idx) { assert(idx >= m_idx_begin && idx < m_idx_end); return m_polygons[idx - m_idx_begin]; }
        const slic3r_api::StoredPolygonCollection& operator[](LayerIndex idx) const { assert(idx >= m_idx_begin && idx < m_idx_end); return m_polygons[idx - m_idx_begin]; }
        std::vector<slic3r_api::StoredPolygonCollection>& polygons_mutable() { return m_polygons; }

    private:
        std::vector<slic3r_api::StoredPolygonCollection> m_polygons;
        LayerIndex                                       m_idx_begin = 0;
        LayerIndex                                       m_idx_end = 0;
    };

    // A cache key: the radius and the layer of an area.
    using RadiusLayerPair             = std::pair<coord_t, LayerIndex>;
    // Cache of areas by layer, then by radius, shared by all the threads. Every access takes the mutex for its own duration only.
    // An area is never replaced or moved once inserted: the layers are in a std::deque, which keeps its elements in place when it
    // grows (a std::vector of maps would copy them on MSVC, where a map is not nothrow movable), and std::map nodes do not move.
    // So a reference returned by getArea() stays valid while other threads insert.
    // The insert functions receive collections of a scratch storage and move them into the run storage first, outside the lock.
    class RadiusLayerPolygonCache {
        using LayerData = std::map<coord_t, slic3r_api::StoredPolygonCollection>;
        using Layers = std::deque<LayerData>;
        // One area to publish, with the key it is published for.
        using Entry = std::pair<RadiusLayerPair, slic3r_api::StoredPolygonCollection>;
    public:
        explicit RadiusLayerPolygonCache(storage_handle *run_storage) : m_run_storage(run_storage) {}
        RadiusLayerPolygonCache(const RadiusLayerPolygonCache&) = delete;
        RadiusLayerPolygonCache& operator=(const RadiusLayerPolygonCache&) = delete;

        // The insertions below keep the area that is already cached for a key: a thread that computed the same key at the same
        // time loses nothing, as both computed the same area.
        void insert(std::vector<std::pair<RadiusLayerPair, slic3r_api::StoredPolygonCollection>> &&in) {
            std::vector<Entry> entries;
            entries.reserve(in.size());
            for (Entry &d : in)
                entries.emplace_back(d.first, moved_to_storage(m_run_storage, d.second));
            publish(entries);
        }
        void insert(std::vector<slic3r_api::StoredPolygonCollection> &&in, coord_t first_layer_idx, coord_t radius) {
            std::vector<Entry> entries;
            entries.reserve(in.size());
            for (slic3r_api::StoredPolygonCollection &d : in)
                entries.emplace_back(RadiusLayerPair(radius, first_layer_idx ++), moved_to_storage(m_run_storage, d));
            publish(entries);
        }
        void insert(LayerPolygonCache &&in, coord_t radius) {
            std::vector<Entry> entries;
            entries.reserve(in.size());
            LayerIndex i = in.begin();
            for (slic3r_api::StoredPolygonCollection &d : in.polygons_mutable())
                entries.emplace_back(RadiusLayerPair(radius, i ++), moved_to_storage(m_run_storage, d));
            publish(entries);
        }
        // Return the area of a key when it was already computed, nothing otherwise.
        std::optional<std::reference_wrapper<const slic3r_api::StoredPolygonCollection>> getArea(const RadiusLayerPair &key) const {
            std::lock_guard<std::mutex> guard(m_mutex);

            if (key.second >= LayerIndex(m_data.size()))
                return std::nullopt;

            const LayerData &layer = m_data[key.second];
            std::map<coord_t, slic3r_api::StoredPolygonCollection>::const_iterator it = layer.find(key.first);
            if (it == layer.end())
                return std::nullopt;

            return std::optional<std::reference_wrapper<const slic3r_api::StoredPolygonCollection>>{it->second};
        }

    private:
        // Inserts the entries under the lock. try_emplace leaves the entry alone when its key is taken: the area that lost is
        // freed with the vector of the caller, after the lock is released.
        void publish(std::vector<Entry> &entries) {
            std::lock_guard<std::mutex> guard(m_mutex);
            for (Entry &d : entries)
                get_allocate_layer_data(d.first.second).try_emplace(d.first.first, std::move(d.second));
        }
        LayerData&          get_allocate_layer_data(LayerIndex layer_idx) {
            allocate_layers(layer_idx + 1);
            return m_data[layer_idx];
        }
        void                allocate_layers(size_t num_layers) {
            if (num_layers > m_data.size())
                m_data.resize(num_layers);
        }

        Layers              m_data;
        mutable std::mutex  m_mutex;
        storage_handle     *m_run_storage = nullptr;
    };

    /*!
     * \brief Creates the collision areas of every radius and layer up to max_layer_idx and stores them in the cache.
     * \param keys RadiusLayerPairs of all requested areas. Every radius will be calculated up to the provided layer.
     *
     * The calculate*() functions always compute the whole range of a key: from layer 0 for the collisions, from layer 1 for the
     * placeable areas, the avoidances and the wall restrictions, which have nothing to compute on layer 0 (the placeable areas
     * insert an empty layer 0, so that every layer of the range is found). They never look at what the caches hold, so a getter
     * can call them from any thread once precalculate() has returned (see Concurrency above); what a cache already holds for a
     * key is kept, and is equal to the new area.
     */
    void calculateCollision(const std::vector<RadiusLayerPair> &keys);
    // Populate collisions through max_layer_idx using each layer's real-Z gap windows and their native XY ramp.
    void calculateCollision(const coord_t radius, const LayerIndex max_layer_idx);
    // Collisions that additionally keep out the holes too small for a branch of the radius until which it always widens.
    void calculateCollisionHolefree(const std::vector<RadiusLayerPair> &keys);
    void calculateCollisionHolefree(RadiusLayerPair key) { calculateCollisionHolefree(std::vector<RadiusLayerPair>{ RadiusLayerPair(key) }); }
    // Avoidances are propagated upwards from the collisions, with the maximum move of each layer.
    void calculateAvoidance(const std::vector<RadiusLayerPair> &keys, bool to_build_plate, bool to_model);
    void calculateAvoidance(RadiusLayerPair key, bool to_build_plate, bool to_model) {
        calculateAvoidance(std::vector<RadiusLayerPair>{ RadiusLayerPair(key) }, to_build_plate, to_model);
    }
    // Placeable areas of every key, each up to the layer of its key.
    void calculatePlaceables(const std::vector<RadiusLayerPair> &keys);
    // Placeable areas of one radius on layers 0 to max_required_layer, from the ones of radius 0; layer 0 stays empty.
    void calculatePlaceables(const coord_t radius, const LayerIndex max_required_layer);
    // Wall restrictions are the collision of one layer intersected with the collision of the layer below.
    void calculateWallRestrictions(const std::vector<RadiusLayerPair> &keys);
    void calculateWallRestrictions(RadiusLayerPair key) { calculateWallRestrictions(std::vector<RadiusLayerPair>{ RadiusLayerPair(key) }); }

    // Throws PluginCancelled when the context is cancelled, and std::runtime_error when the enclosing TBB group is cancelled; called
    // after each computation, before its result is published or read again.
    void throw_if_interrupted() const;

    // Return the collision of a radius and layer, computed on demand when precalculate() did not cover it.
    const slic3r_api::StoredPolygonCollection& getCollisionHolefree(coord_t radius, LayerIndex layer_idx) const;
    // Rounds a radius up to the scale of cached radii (from min_radius, then fine steps, then exponential). The list of ignorable
    // radii is empty, so every step of the scale is a possible result.
    coord_t ceilRadius(const coord_t radius) const;

    RadiusLayerPolygonCache& avoidance_cache(const AvoidanceType type, const bool to_model) {
        if (to_model) {
            switch (type) {
            case AvoidanceType::Fast:       return m_avoidance_cache_to_model;
            case AvoidanceType::Slow:       return m_avoidance_cache_to_model_slow;
            case AvoidanceType::Count:      assert(false);
            case AvoidanceType::FastSafe:   return m_avoidance_cache_holefree_to_model;
            }
        } else {
            switch (type) {
            case AvoidanceType::Fast:       return m_avoidance_cache;
            case AvoidanceType::Slow:       return m_avoidance_cache_slow;
            case AvoidanceType::Count:      assert(false);
            case AvoidanceType::FastSafe:   return m_avoidance_cache_holefree;
            }
        }
        assert(false);
        return m_avoidance_cache;
    }
    const RadiusLayerPolygonCache& avoidance_cache(const AvoidanceType type, const bool to_model) const {
        return const_cast<TreeVolumes*>(this)->avoidance_cache(type, to_model);
    }

    // The context of the step, checked by every computation; not owned, nullptr never cancels.
    const plugin_run_context *m_context = nullptr;
    // Per layer maximum move of the branch centres (scaled), fast and slow, with a small margin against rounding.
    std::vector<coord_t> m_max_move;
    std::vector<coord_t> m_max_move_slow;
    coord_t m_min_resolution;
    bool m_precalculated = false;
    // Minimum XY distance between the model and the branches, and its difference from the regular distance.
    coord_t m_current_min_xy_dist;
    coord_t m_current_min_xy_dist_delta;
    // False when every object only allows branches on the build plate.
    bool m_support_rests_on_model;
    // Radius from which the radius of a branch is always increased, even if the avoidance does not allow it.
    coord_t m_increase_until_radius;
    // Smallest radius a branch can have: the radius of a tip (min_radius), the first step of the ceilRadius() scale.
    coord_t m_radius_0;
    // Object outline of each layer.
    std::vector<slic3r_api::StoredPolygonCollection> m_layer_outlines;
    // Support blockers of each layer, avoided like the object; empty when the object has no blocker.
    std::vector<slic3r_api::StoredExPolygonCollection> m_anti_overhang;
    // Radii that ceilRadius skips, sorted. It is empty: with a radius gain that follows the real layer heights and a continuous
    // catch up, no radius of the scale can be shown to be unreachable.
    std::vector<coord_t> m_ignorable_radii;
    // The settings that give the radii of the tips and the branch sizes.
    TreeSettings m_settings;

    RadiusLayerPolygonCache     m_collision_cache;
    RadiusLayerPolygonCache     m_collision_cache_holefree;
    RadiusLayerPolygonCache     m_avoidance_cache;
    RadiusLayerPolygonCache     m_avoidance_cache_slow;
    RadiusLayerPolygonCache     m_avoidance_cache_to_model;
    RadiusLayerPolygonCache     m_avoidance_cache_to_model_slow;
    RadiusLayerPolygonCache     m_placeable_areas_cache;
    // Avoidances that do not cross holes narrower than the radius until which the radius is always increased.
    RadiusLayerPolygonCache     m_avoidance_cache_holefree;
    RadiusLayerPolygonCache     m_avoidance_cache_holefree_to_model;
    // Walls a branch must not pass, with and without the extra minimum XY distance.
    RadiusLayerPolygonCache     m_wall_restrictions_cache;
    RadiusLayerPolygonCache     m_wall_restrictions_cache_min;
};

} // namespace TreeFamily
} // namespace Slic3r

#endif // plugins_support_tree_volumes_hpp_
