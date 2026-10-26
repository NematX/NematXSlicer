///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* Settings of the tree branches of one object: radii, distances, branch angles and the maximum moves per layer.

   The routing works on the object layers. Most quantities are counts of layers: the radius a branch gains with its
   distance to the top, the length of the tip, the Z gaps of the collisions. They use the nominal layer height of the
   object, as the legacy tree did. The one quantity that follows the real layer heights is the maximum move of the
   branch centres: between two consecutive layers it is tan(effective angle) times the height of the upper layer.

   The effective angle is the branch angle, capped so that the horizontal overhang of a branch per support layer does not
   exceed support_tree_max_overhang at the minimum layer height of the support extruder:
       effective angle = min(support_tree_angle, atan(max_overhang / support_min_height)).

       const TreeSettings settings(object, print, params);
       const coord_t move = settings.maximum_move_distance(layer_idx); */
#ifndef plugins_support_tree_settings_hpp_
#define plugins_support_tree_settings_hpp_

#include <cstddef>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_def.h"

namespace slic3r_api {
class Object;
class Print;
}

namespace slic3r_api::Support::Common {
struct SupportParams;
}

namespace Slic3r {

namespace TreeFamily {

// Index of an object layer, bottom first.
using LayerIndex = int;

// Areas smaller than this are dropped, as too small to hold a branch.
static constexpr const double tiny_area_threshold = scale_d(0.001) * scale_d(0.001);

struct TreeSettings
{
    TreeSettings() = default;
    // Read the object and print settings through the plugin API views; params gives the minimum layer height, the Z gaps, the machine Z step
    // and the support flows. The views must be those params was built from.
    TreeSettings(const slic3r_api::Object &object, const slic3r_api::Print &print, const slic3r_api::Support::Common::SupportParams &params);

    // Nominal layer height, used by the layer counts (scaled).
    coord_t layer_height = 0;
    // Simplification tolerance and smallest feature size of the object outlines (scaled).
    coord_t resolution = 0;
    coord_t min_feature_size = 0;
    // Width of one support line, and the radii of the tip and of the branches (scaled).
    coord_t support_line_width = 0;
    coord_t branch_radius = 0;
    coord_t min_radius = 0;
    // Number of layers over which the tip grows from min_radius to branch_radius.
    size_t tip_layers = 0;
    // Radius gained per layer by a branch, and the one gained per layer when it rests on the bed.
    double branch_radius_increase_per_layer = 0;
    double bp_radius_increase_per_layer = 0;
    // Radius that a branch tries to reach on the build plate, and the layer from which that growth starts.
    coord_t bp_radius = 0;
    LayerIndex layer_start_bp_radius = 0;
    // Radius from which the radius is always increased, and the number of layers needed to reach it.
    coord_t increase_radius_until_radius = 0;
    size_t increase_radius_until_layer = 0;
    // Layers that a branch must stay in place when it rests on the model.
    size_t min_dtt_to_model = 0;
    // Radius increase allowed when a model branch merges with branches that reach the bed.
    coord_t max_to_model_radius_increase = 0;
    // False when the object only allows branches on the build plate.
    bool support_rests_on_model = true;
    // XY distance from the object, and the smaller one used for overhang outlines (scaled).
    coord_t xy_distance = 0;
    coord_t xy_min_distance = 0;
    // Layers between the object and the top and bottom of the support: the Z gaps divided by the nominal layer height, rounded.
    size_t z_distance_top_layers = 0;
    size_t z_distance_bottom_layers = 0;
    // Spacing of the tip points along the overhang lines, and the top rate that sets their density (percent).
    coord_t branch_distance = 0;
    double top_rate = 0;
    // Effective branch angle (radians), after the cap of support_tree_max_overhang.
    double angle = 0;
    // Largest horizontal overhang of a branch over one support slab (scaled), from support_tree_max_overhang.
    coord_t max_overhang = 0;
    // Thinnest and thickest slabs of the branch middle, and the dense interface slab, on the machine Z step (scaled).
    coord_t min_slab_height = 0, max_slab_height = 0, dense_slab_height = 0;
    // Machine Z step of the support (scaled); the slab boundaries snap to it.
    coord_t z_step = 0;

    // Radius of a branch with the given distance to top, and the radius a branch should have at this layer on the bed.
    [[nodiscard]] coord_t getRadius(size_t distance_to_top, double elephant_foot_increases = 0) const;
    [[nodiscard]] coord_t recommendedMinRadius(LayerIndex layer_idx) const;

    // Maximum move of a branch centre from the layer below into layer_idx, fast and slow (scaled).
    [[nodiscard]] coord_t maximum_move_distance(LayerIndex layer_idx) const { return max_move[layer_idx]; }
    [[nodiscard]] coord_t maximum_move_distance_slow(LayerIndex layer_idx) const { return max_move_slow[layer_idx]; }

    // The per layer maximum moves, indexed by object layer.
    std::vector<coord_t> max_move;
    std::vector<coord_t> max_move_slow;
};

} // namespace TreeFamily
} // namespace Slic3r

#endif // plugins_support_tree_settings_hpp_
