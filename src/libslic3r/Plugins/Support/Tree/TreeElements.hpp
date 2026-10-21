///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* State of the tree branch elements while the influence areas are propagated down the object layers.

   A SupportElement is one influence area of a branch on one object layer: its SupportElementState says where the
   branch must end up (target_position, target_height), how it reaches the bed or the model (to_buildplate,
   to_model_gracious), how far it is from its tip (distance_to_top) and how its radius grows. The influence area is
   the polygon the branch centre may occupy on its layer. Elements of the layer above that rest on an element are
   listed in its parents, so the branches form a forest that is read back when the nodes are placed.

   The influence area of an element is a StoredPolygonCollection allocated in the run storage of the support. The elements
   are the local data of route_trees(): their collections are freed when route_trees() returns (the destructor of each
   element frees its handle), before the returned trees are used; the trees hold no handle. The positions of the states are
   API points (c_point, scaled units); the positions of the nodes of the routed trees are copied from them.

   Tree supports by Thomas Rahm, losely based on Tree Supports by CuraEngine.
   Original CuraEngine copyright:
   Copyright (c) 2021 Ultimaker B.V.
   CuraEngine is released under the terms of the AGPLv3 or higher. */
#ifndef plugins_support_tree_elements_hpp_
#define plugins_support_tree_elements_hpp_

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <deque>
#include <limits>
#include <vector>

#include <boost/container/small_vector.hpp>

#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"
#include "TreeSettings.hpp"
#include "TreeVolumes.hpp"

namespace Slic3r::TreeFamily {

enum class LineStatus
{
    INVALID,
    TO_MODEL,
    TO_MODEL_GRACIOUS,
    TO_MODEL_GRACIOUS_SAFE,
    TO_BP,
    TO_BP_SAFE
};

// The points of a line (the API point type) and the avoidance each one is valid in.
using LineInformation = std::vector<std::pair<c_point, LineStatus>>;
using LineInformations = std::vector<LineInformation>;

// The maximum chord error of the circles of the branches, in scaled units (25 nm): the target largest distance between a side of the polygon and the true circle. It is a target
// before the truncation of the coordinates to integers, so the actual polygon may exceed it slightly. It is used by the branch circles and
// by the small tip-base circle.
static constexpr const size_t SUPPORT_TREE_CIRCLE_RESOLUTION = 25;

struct AreaIncreaseSettings
{
    AreaIncreaseSettings(
        TreeVolumes::AvoidanceType type = TreeVolumes::AvoidanceType::Fast, coord_t increase_speed = 0,
        bool increase_radius = false, bool no_error = false, bool use_min_distance = false, bool move = false) :
        increase_speed{ increase_speed }, type{ type }, increase_radius{ increase_radius }, no_error{ no_error }, use_min_distance{ use_min_distance }, move{ move } {}

    coord_t         increase_speed;
    // Packing for smaller memory footprint of SupportElementState && SupportElementMerging
    TreeVolumes::AvoidanceType type;
    bool            increase_radius  : 1;
    bool            no_error         : 1;
    bool            use_min_distance : 1;
    bool            move             : 1;
    bool operator==(const AreaIncreaseSettings& other) const
    {
        return type             == other.type               &&
               increase_speed   == other.increase_speed     &&
               increase_radius  == other.increase_radius    &&
               no_error         == other.no_error           &&
               use_min_distance == other.use_min_distance   &&
               move             == other.move;
    }
};

// C++17 does not support in place initializers of bit values, thus a constructor zeroing the bits is provided.
struct SupportElementStateBits {
    SupportElementStateBits() :
        to_buildplate(false),
        to_model_gracious(false),
        use_min_xy_dist(false),
        can_use_safe_radius(false),
        skip_ovalisation(false),
        lost(false),
        verylost(false),
        deleted(false),
        marked(false)
        {}

    /*!
     * \brief The element trys to reach the buildplate
     */
    bool to_buildplate : 1;

    /*!
     * \brief Will the branch be able to rest completely on a flat surface, be it buildplate or model ?
     */
    bool to_model_gracious : 1;

    /*!
     * \brief Whether the min_xy_distance can be used to get avoidance or similar. Will only be true if support_xy_overrides_z=Z overrides X/Y.
     */
    bool use_min_xy_dist : 1;

    /*!
     * \brief An influence area is considered safe when it can use the holefree avoidance <=> It will not have to encounter holes on its way downward.
     */
    bool can_use_safe_radius : 1;

    /*!
     * \brief Skip the ovalisation to parent and children when generating the final circles.
     */
    bool skip_ovalisation : 1;

    // Likely a lost branch, debugging information.
    bool lost : 1;
    bool verylost : 1;

    // Not valid anymore, to be deleted.
    bool deleted : 1;

    // General purpose flag marking a visited element.
    bool marked : 1;
};

struct SupportElementState : public SupportElementStateBits
{
    /*!
     * \brief The layer this support elements wants reach
     */
    LayerIndex  target_height;

    /*!
     * \brief The position this support elements wants to support on layer=target_height
     */
    c_point     target_position{ 0, 0 };

    /*!
     * \brief The next position this support elements wants to reach. NOTE: This is mainly a suggestion regarding direction inside the influence area.
     */
    c_point     next_position{ 0, 0 };

    /*!
     * \brief The next height this support elements wants to reach
     */
    LayerIndex  layer_idx;

    /*!
     * \brief The radius gain (scaled, above min_radius, without the foot) that the branch has acquired for its radius increases and
     * collision calculations. It grows by TreeSettings::advance() each time a layer step increases the radius, and by the catch-up of
     * the routing (bounded by the path gain of one step less); it stays when the radius is not increased. The drawn radius
     * (support_element_radius()) follows the path gain without pause, capped at the tip total gain, during the tip, then the
     * acquired gain.
     */
    double      radius_gain;

    /*!
     * \brief The amount of layers this element is below the topmost layer of this branch.
     */
    uint32_t    distance_to_top;

    /*!
     * \brief The resulting center point around which a circle will be drawn later.
     * Will be set by setPointsOnAreas
     */
    c_point result_on_layer { std::numeric_limits<coord_t>::max(), std::numeric_limits<coord_t>::max() };
    // Whether result_on_layer holds a position, i.e. is not the {max, max} sentinel.
    bool    result_on_layer_is_set() const
        { return this->result_on_layer.x != std::numeric_limits<coord_t>::max() || this->result_on_layer.y != std::numeric_limits<coord_t>::max(); }
    // Restores the {max, max} sentinel: the position is no longer set.
    void    result_on_layer_reset() { this->result_on_layer = c_point{ std::numeric_limits<coord_t>::max(), std::numeric_limits<coord_t>::max() }; }
    /*!
     * \brief The amount of extra radius we got from merging branches that could have reached the buildplate, but merged with ones that can not.
     */
    coord_t     increased_to_model_radius; // how much to model we increased only relevant for merging

    /*!
     * \brief The foot widening (scaled length) that the branch has acquired near the build plate, on top of radius_gain. It
     * is part of both the collision and the drawn radius. It only grows by foot_step_gain() (and is set by a merge so that the
     * drawn radius never decreases).
     */
    double      foot_gain;

    /*!
     * \brief The element tries to not move until this dtt is reached, is set to 0 if the element had to move.
     */
    uint32_t    dont_move_until;

    /*!
     * \brief Settings used to increase the influence area to its current state.
     */
    AreaIncreaseSettings last_area_increase;

    // called by increase_single_area() and increaseAreas()
    [[nodiscard]] static SupportElementState propagate_down(const SupportElementState &src)
    {
        SupportElementState dst{ src };
        ++ dst.distance_to_top;
        -- dst.layer_idx;
        // set to invalid as we are a new node on a new layer
        dst.result_on_layer_reset();
        dst.skip_ovalisation = false;
        return dst;
    }

};

/*!
 * \brief The radius (scaled) of a branch with the given radius gain and foot gain: min_radius + gain + foot_gain. This is the
 * single conversion to coord_t of the radii of the elements and of the routed nodes (collision and drawn radius);
 * TreeVolumes::precalculate() converts the radii it simulates itself.
 * \param gain[in] The radius gain above min_radius (scaled, without the foot).
 * \param foot_gain[in] The foot widening (see SupportElementState::foot_gain).
 */
[[nodiscard]] inline coord_t radius_from_gain(const TreeSettings &settings, const double gain, const double foot_gain)
{
    return coord_t(settings.min_radius + gain + foot_gain);
}

/*!
 * \brief Get the Radius, that this element will have: the one that is drawn. During the tip it follows the path without pause
 * (the collision may have paused), afterwards it is the acquired gain.
 * \param elem[in] The Element, on layer elem.layer_idx, whose tip is on layer elem.layer_idx + elem.distance_to_top.
 * \return The radius the element has.
 */
[[nodiscard]] inline coord_t support_element_radius(const TreeSettings &settings, const SupportElementState &elem)
{
    const double path = std::min(settings.path_gain(elem.layer_idx, elem.distance_to_top), settings.tip_total_gain);
    return radius_from_gain(settings, std::max(elem.radius_gain, path), elem.foot_gain);
}

/*!
 * \brief Get the collision Radius of this Element. This can be smaller then the actual radius, as the drawAreas will cut off areas that may collide with the model.
 * \param elem[in] The Element.
 * \return The collision radius the element has.
 */
[[nodiscard]] inline coord_t support_element_collision_radius(const TreeSettings &settings, const SupportElementState &elem)
{
    return radius_from_gain(settings, elem.radius_gain, elem.foot_gain);
}

/*!
 * \brief The foot widening that a branch adds on the step from layer upper_layer to upper_layer - 1, resting on the bed.
 *
 * The step budget of the foot is settings.bp_gain[upper_layer]: the foot adds what the normal growth of the step (the drawn
 * radius difference between grown and parent) leaves of it, never more than what is missing to settings.foot_target[upper_layer
 * - 1], and never a negative amount. The foot only grows for a branch that goes to the build plate, whose drawn radius has
 * reached branch_radius, on a layer where the foot is active (foot_target > branch_radius). The routing step
 * (increase_single_area()) calls this function; the foot gives no lateral speed.
 * \param parent[in] The element on upper_layer.
 * \param grown[in] The element on upper_layer - 1 after its normal growth (and catch-up), with the foot of the parent.
 * \param upper_layer[in] The upper layer of the step, an object layer above layer 0.
 * \return The foot widening to add (scaled), 0 when the foot does not grow.
 */
[[nodiscard]] inline double foot_step_gain(const TreeSettings &settings, const SupportElementState &parent, const SupportElementState &grown, const LayerIndex upper_layer)
{
    assert(upper_layer > 0 && size_t(upper_layer) < settings.foot_target.size());
    const coord_t drawn = support_element_radius(settings, grown);
    const double  target = settings.foot_target[upper_layer - 1];
    if (! grown.to_buildplate || target <= double(settings.branch_radius) || drawn < settings.branch_radius)
        return 0.;
    const double step_growth = double(drawn - support_element_radius(settings, parent));
    return std::min(std::max(0., settings.bp_gain[upper_layer] - step_growth), std::max(0., target - double(drawn)));
}

/*!
 * \brief The gain a branch gets from the catch-up of the radius: the largest gain whose collision radius stays within the ceiled
 * radius found by the cache search and within the drawn radius, bounded by the path gain of one step less.
 *
 * The gain never decreases. The foot acquired so far is part of both radii, so it is removed from the reachable radius.
 * \param gain[in] The current gain (scaled, above min_radius, without the foot).
 * \param ceil_radius[in] The largest cache radius found valid (scaled).
 * \param drawn_radius[in] The drawn radius of the element (scaled, foot included).
 * \param min_radius[in] The tip radius (scaled).
 * \param foot_gain[in] The foot widening acquired so far (scaled).
 * \param path_bound[in] The path gain of one step less than the distance to the tip.
 */
[[nodiscard]] inline double catch_up_gain(
    const double gain, const coord_t ceil_radius, const coord_t drawn_radius, const coord_t min_radius, const double foot_gain, const double path_bound)
{
    const double reachable_gain = double(std::min(ceil_radius, drawn_radius)) - double(min_radius) - foot_gain;
    return std::max(gain, std::min(reachable_gain, path_bound));
}

/*!
 * \brief True when merging two elements must be refused because the collision radius of the merged state needs a higher cache step
 * (ceilRadius()) than both elements: their areas were validated with their own collision radii only.
 *
 * The three radii are the ceiled collision radii, all taken with the same XY variant (use_min_xy_dist) of the merge.
 */
[[nodiscard]] inline bool merge_crosses_cache_step(const coord_t merged_ceil_radius, const coord_t first_ceil_radius, const coord_t second_ceil_radius)
{
    return merged_ceil_radius > std::max(first_ceil_radius, second_ceil_radius);
}

/*!
 * \brief Combines the states of two elements that merge into one, on layer layer_idx (both elements are on that layer).
 *
 * The radius gain and the distance to top are the larger ones. The foot is what the merged gain and path do not give of the
 * largest drawn radius of the two elements, so the drawn radius never decreases through a merge. to_buildplate and
 * to_model_gracious are conjunctions: the merged branch may lose its access to the bed. last_area_increase takes the smaller type
 * and speed and the or / and of the flags, as the best out of both elements. The caller sets increased_to_model_radius.
 */
[[nodiscard]] inline SupportElementState merge_support_element_states(
    const SupportElementState &first, const SupportElementState &second, const c_point &next_position, const coord_t layer_idx,
    const TreeSettings &config)
{
    SupportElementState out;
    out.next_position   = next_position;
    out.layer_idx       = layer_idx;
    out.use_min_xy_dist = first.use_min_xy_dist || second.use_min_xy_dist;
    out.dont_move_until = std::max(first.dont_move_until, second.dont_move_until);
    out.can_use_safe_radius = first.can_use_safe_radius || second.can_use_safe_radius;
    out.skip_ovalisation = false;
    if (first.target_height > second.target_height) {
        out.target_height   = first.target_height;
        out.target_position = first.target_position;
    } else {
        out.target_height   = second.target_height;
        out.target_position = second.target_position;
    }
    out.radius_gain = std::max(first.radius_gain, second.radius_gain);
    out.distance_to_top = std::max(first.distance_to_top, second.distance_to_top);

    out.to_buildplate = first.to_buildplate && second.to_buildplate;
    out.to_model_gracious = first.to_model_gracious && second.to_model_gracious; // valid as we do not merge non-gracious with gracious

    // The foot carries the missing part in whole scaled units: the merged drawn radius is then at least the larger one of the parents.
    out.foot_gain = 0;
    const coord_t parents_drawn = std::max(support_element_radius(config, first), support_element_radius(config, second));
    out.foot_gain = double(std::max<coord_t>(0, parents_drawn - support_element_radius(config, out)));

    // set last settings to the best out of both parents. If this is wrong, it will only cause a small performance penalty instead of weird behavior.
    out.last_area_increase = {
        std::min(first.last_area_increase.type, second.last_area_increase.type),
        std::min(first.last_area_increase.increase_speed, second.last_area_increase.increase_speed),
        first.last_area_increase.increase_radius || second.last_area_increase.increase_radius,
        first.last_area_increase.no_error || second.last_area_increase.no_error,
        first.last_area_increase.use_min_distance && second.last_area_increase.use_min_distance,
        first.last_area_increase.move || second.last_area_increase.move };

    return out;
}

struct SupportElement
{
    using ParentIndices =
#ifdef NDEBUG
        // To reduce memory allocation in release mode.
        boost::container::small_vector<int32_t, 4>;
#else // NDEBUG
        // To ease debugging.
        std::vector<int32_t>;
#endif // NDEBUG

    // An element without parents (a tip). The influence area is moved in: the element owns its handle from now on. It must have been
    // allocated in the run storage, because the element outlives every scratch storage.
    SupportElement(const SupportElementState &state, slic3r_api::StoredPolygonCollection &&influence_area) :
        state(state), influence_area(std::move(influence_area)) {}
    // An element resting on the elements of the layer above listed in parents (the result of a layer step or of a merge); the
    // influence area is moved in as above.
    SupportElement(const SupportElementState &state, ParentIndices &&parents, slic3r_api::StoredPolygonCollection &&influence_area) :
        state(state), parents(std::move(parents)), influence_area(std::move(influence_area)) {}

    SupportElementState         state;

    /*!
     * \brief All elements in the layer above the current one that are supported by this element
     */
    ParentIndices               parents;

    /*!
     * \brief The influence area: always set for a kept element.
     * It is allocated in the run storage, because the element outlives every scratch storage. The element owns the handle and its
     * destructor frees it, so the storage must outlive the element. The copy is deleted; a move transfers the handle, and a
     * moved-from element holds no handle (the routing tests it with has_rings()).
     */
    slic3r_api::StoredPolygonCollection influence_area;

    /*!
     * \brief The tip source that inserted this element as a tip, or the largest size_t for every other element.
     */
    size_t                      tip_source = std::numeric_limits<size_t>::max();
};

using SupportElements = std::deque<SupportElement>;

[[nodiscard]] inline coord_t support_element_radius(const TreeSettings &settings, const SupportElement &elem)
{
    return support_element_radius(settings, elem.state);
}

[[nodiscard]] inline coord_t support_element_collision_radius(const TreeSettings &settings, const SupportElement &elem)
{
    return support_element_collision_radius(settings, elem.state);
}

} // namespace Slic3r::TreeFamily

#endif // plugins_support_tree_elements_hpp_
