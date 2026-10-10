///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* Collision, avoidance and placeable areas of the tree branches, cached per radius and object layer.

   A tree branch is a disc of some radius centred on a node placed on an object layer. TreeVolumes answers, for a
   radius and a layer, which 2D areas the node must not enter:
   - getCollision: the object and its support blockers grown by the radius and the XY distance, with the Z gaps
     above and below the layer;
   - getAvoidance: the places from which a node cannot descend to the build plate (or to the model) within the
     maximum move of each layer. It is built upwards from the collision of the layers below, one layer at a time;
   - getPlaceableAreas: the top surfaces where a branch of this radius fits and may stand on the model;
   - getWallRestriction: the object walls a branch must not cross between two consecutive layers.

   Every area is computed for a (radius, layer) key and kept in a cache. precalculate() computes the keys a tree can
   request up to its top layer, so the router only reads the caches.

       TreeVolumes volumes(object, settings);
       volumes.precalculate(max_layer, throw_on_cancel);
       const Polygons &free = volumes.getAvoidance(radius, layer, TreeVolumes::AvoidanceType::Fast, false, min_xy_dist); */
#ifndef plugins_support_tree_volumes_hpp_
#define plugins_support_tree_volumes_hpp_

#include <cassert>
#include <cstddef>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <utility>
#include <vector>

#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "TreeSettings.hpp"

namespace Slic3r {

class PrintObject;

namespace TreeFamily {

// Growth factor of the radii above the exponential threshold, and the radius step below it.
static constexpr const double  SUPPORT_TREE_EXPONENTIAL_FACTOR = 1.5;
static constexpr const coord_t SUPPORT_TREE_EXPONENTIAL_THRESHOLD = scale_i(1. * SUPPORT_TREE_EXPONENTIAL_FACTOR);
static constexpr const coord_t SUPPORT_TREE_COLLISION_RESOLUTION = scale_i(0.5);

class TreeVolumes
{
public:
    TreeVolumes() = default;
    // Read the object layers, its support blockers and the settings; no area is computed until precalculate().
    TreeVolumes(const PrintObject &object, const TreeSettings &settings);
    TreeVolumes(TreeVolumes&&) = default;
    TreeVolumes& operator=(TreeVolumes&&) = default;

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
     * Requests for other keys still work, but they are computed on demand and are slower.
     */
    void precalculate(const coord_t max_layer, std::function<void()> throw_on_cancel);

    /*!
     * \brief Provides the areas that a node of the given radius must not enter on this layer, because they collide with the model.
     * \param radius The radius of the node of interest
     * \param layer_idx The layer of interest
     * \param min_xy_dist Is the minimum xy distance used.
     */
    const Polygons& getCollision(const coord_t radius, LayerIndex layer_idx, bool min_xy_dist) const;

    /*!
     * \brief Provides the areas that a node of the given radius must avoid on this layer to still reach the build plate,
     * or the model when to_model is set.
     * \param radius The radius of the node of interest
     * \param layer_idx The layer of interest
     * \param type The avoidance speed and hole handling
     * \param to_model Does the avoidance allow good connections with the model.
     * \param min_xy_dist is the minimum xy distance used.
     */
    const Polygons& getAvoidance(coord_t radius, LayerIndex layer_idx, AvoidanceType type, bool to_model, bool min_xy_dist) const;

    /*!
     * \brief Provides the area where a branch of this radius completely fits on the model on this layer.
     * \param radius The radius of the node of interest
     * \param layer_idx The layer of interest
     */
    const Polygons& getPlaceableAreas(coord_t radius, LayerIndex layer_idx, std::function<void()> throw_on_cancel) const;

    /*!
     * \brief Provides the abstract walls that a branch must not pass when it moves between two layers.
     * \param radius The radius of the node of interest.
     * \param layer_idx The layer of interest.
     * \param min_xy_dist is the minimum xy distance used.
     */
    const Polygons& getWallRestriction(coord_t radius, LayerIndex layer_idx, bool min_xy_dist) const;

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
    // Polygons of a range of layers, indexed by layer.
    class LayerPolygonCache {
    public:
        void allocate(LayerIndex aidx_begin, LayerIndex aidx_end) {
            m_idx_begin = aidx_begin;
            m_idx_end = aidx_end;
            m_polygons.assign(aidx_end - aidx_begin, {});
        }

        LayerIndex begin() const { return m_idx_begin; }
        LayerIndex end()   const { return m_idx_end; }
        size_t     size()  const { return m_polygons.size(); }

        bool      has(LayerIndex idx) const { return idx >= m_idx_begin && idx < m_idx_end; }
        Polygons& operator[](LayerIndex idx) { assert(idx >= m_idx_begin && idx < m_idx_end); return m_polygons[idx - m_idx_begin]; }
        std::vector<Polygons>& polygons_mutable() { return m_polygons; }

    private:
        std::vector<Polygons> m_polygons;
        LayerIndex            m_idx_begin;
        LayerIndex            m_idx_end;
    };

    // A cache key: the radius and the layer of an area.
    using RadiusLayerPair             = std::pair<coord_t, LayerIndex>;
    // Cache of areas by layer, then by radius. References returned by getArea() stay valid after later insertions.
    class RadiusLayerPolygonCache {
        using LayerData = std::map<coord_t, Polygons>;
        using Layers = std::vector<LayerData>;
    public:
        RadiusLayerPolygonCache() = default;
        RadiusLayerPolygonCache(RadiusLayerPolygonCache &&rhs) : m_data(std::move(rhs.m_data)) {}
        RadiusLayerPolygonCache& operator=(RadiusLayerPolygonCache &&rhs) { m_data = std::move(rhs.m_data); return *this; }

        RadiusLayerPolygonCache(const RadiusLayerPolygonCache&) = delete;
        RadiusLayerPolygonCache& operator=(const RadiusLayerPolygonCache&) = delete;

        void insert(std::vector<std::pair<RadiusLayerPair, Polygons>> &&in) {
            std::lock_guard<std::mutex> guard(m_mutex);
            for (auto &d : in)
                this->get_allocate_layer_data(d.first.second).emplace(d.first.first, std::move(d.second));
        }
        void insert(std::vector<Polygons> &&in, coord_t first_layer_idx, coord_t radius) {
            std::lock_guard<std::mutex> guard(m_mutex);
            allocate_layers(first_layer_idx + in.size());
            for (auto &d : in)
                m_data[first_layer_idx ++].emplace(radius, std::move(d));
        }
        void insert(LayerPolygonCache &&in, coord_t radius) {
            std::lock_guard<std::mutex> guard(m_mutex);
            LayerIndex i = in.begin();
            allocate_layers(i + LayerIndex(in.size()));
            for (auto &d : in.polygons_mutable())
                m_data[i ++].emplace(radius, std::move(d));
        }
        // Return the area of a key when it was already computed, nothing otherwise.
        std::optional<std::reference_wrapper<const Polygons>> getArea(const RadiusLayerPair &key) const {
            std::lock_guard<std::mutex> guard(m_mutex);

            if (key.second >= LayerIndex(m_data.size()))
                return std::nullopt;

            const LayerData &layer = m_data[key.second];
            auto it = layer.find(key.first);
            if (it == layer.end())
                return std::nullopt;

            return std::optional<std::reference_wrapper<const Polygons>>{it->second};
        }
        // Return the highest computed layer for this radius, or -1 when none is computed above layer 0.
        LayerIndex getMaxCalculatedLayer(coord_t radius) const {
            std::lock_guard<std::mutex> guard(m_mutex);
            auto layer_idx = LayerIndex(m_data.size()) - 1;
            for (; layer_idx > 0; -- layer_idx)
                if (const auto &layer = m_data[layer_idx]; layer.find(radius) != layer.end())
                    break;
            // Placeable areas do not exist on layer 0, so layer 0 may be missing while layer 1 exists.
            return layer_idx == 0 ? -1 : layer_idx;
        }

        void clear() { m_data.clear(); }

    private:
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
    };

    /*!
     * \brief Creates the collision areas of every radius and layer up to max_layer_idx and stores them in the cache.
     * \param keys RadiusLayerPairs of all requested areas. Every radius will be calculated up to the provided layer.
     */
    void calculateCollision(const std::vector<RadiusLayerPair> &keys, std::function<void()> throw_on_cancel);
    void calculateCollision(const coord_t radius, const LayerIndex max_layer_idx, std::function<void()> throw_on_cancel);
    // Collisions that additionally keep out the holes too small for a branch of the radius until which it always widens.
    void calculateCollisionHolefree(const std::vector<RadiusLayerPair> &keys, std::function<void()> throw_on_cancel);
    void calculateCollisionHolefree(RadiusLayerPair key) { calculateCollisionHolefree(std::vector<RadiusLayerPair>{ RadiusLayerPair(key) }, []{}); }
    // Avoidances are propagated upwards from the collisions, with the maximum move of each layer.
    void calculateAvoidance(const std::vector<RadiusLayerPair> &keys, bool to_build_plate, bool to_model, std::function<void()> throw_on_cancel);
    void calculateAvoidance(RadiusLayerPair key, bool to_build_plate, bool to_model) {
        calculateAvoidance(std::vector<RadiusLayerPair>{ RadiusLayerPair(key) }, to_build_plate, to_model, []{});
    }
    void calculatePlaceables(const std::vector<RadiusLayerPair> &keys, std::function<void()> throw_on_cancel);
    void calculatePlaceables(const coord_t radius, const LayerIndex max_required_layer, std::function<void()> throw_on_cancel);
    // Wall restrictions are the collision of one layer intersected with the collision of the layer below.
    void calculateWallRestrictions(const std::vector<RadiusLayerPair> &keys, std::function<void()> throw_on_cancel);
    void calculateWallRestrictions(RadiusLayerPair key) { calculateWallRestrictions(std::vector<RadiusLayerPair>{ RadiusLayerPair(key) }, []{}); }

    // Return the collision of a radius and layer, computed on demand when precalculate() did not cover it.
    const Polygons& getCollisionHolefree(coord_t radius, LayerIndex layer_idx) const;
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
    // Smallest radius a branch can have, at distance to top 0.
    coord_t m_radius_0;
    // Object outline of each layer.
    std::vector<Polygons> m_layer_outlines;
    // Support blockers of each layer, avoided like the object.
    std::vector<ExPolygons> m_anti_overhang;
    // Radii that ceilRadius skips, as no tree tip can request them, sorted.
    std::vector<coord_t> m_ignorable_radii;
    // Layers count of the Z gaps used by the collisions, from the nominal layer height.
    int m_z_distance_bottom_layers;
    int m_z_distance_top_layers;
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
