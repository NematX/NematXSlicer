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
     * \brief The Effective distance to top of this element regarding radius increases and collision calculations.
     */
    uint32_t    effective_radius_height;

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
     * \brief Counter about the times the elephant foot was increased. Can be fractions for merge reasons.
     */
    double      elephant_foot_increases;

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
 * \brief Get the Distance to top regarding the real radius this part will have. This is different from distance_to_top, which is can be used to calculate the top most layer of the branch.
 * \param elem[in] The SupportElement one wants to know the effectiveDTT
 * \return The Effective DTT.
 */
[[nodiscard]] inline size_t getEffectiveDTT(const TreeSettings &settings, const SupportElementState &elem)
{
    return elem.effective_radius_height < settings.increase_radius_until_layer ?
        (elem.distance_to_top < settings.increase_radius_until_layer ? elem.distance_to_top : settings.increase_radius_until_layer) :
        elem.effective_radius_height;
}

/*!
 * \brief Get the Radius, that this element will have.
 * \param elem[in] The Element.
 * \return The radius the element has.
 */
[[nodiscard]] inline coord_t support_element_radius(const TreeSettings &settings, const SupportElementState &elem)
{
    return settings.getRadius(getEffectiveDTT(settings, elem), elem.elephant_foot_increases);
}

/*!
 * \brief Get the collision Radius of this Element. This can be smaller then the actual radius, as the drawAreas will cut off areas that may collide with the model.
 * \param elem[in] The Element.
 * \return The collision radius the element has.
 */
[[nodiscard]] inline coord_t support_element_collision_radius(const TreeSettings &settings, const SupportElementState &elem)
{
    return settings.getRadius(elem.effective_radius_height, elem.elephant_foot_increases);
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
