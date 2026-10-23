///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* Settings of the tree branches of one object: radii, distances, branch angles and the maximum moves per layer.

   The routing works on the object layers. The radius a branch gains between two consecutive layers depends on the real
   height of the upper layer (tip_gain, cone_gain): the tip grows by at most a third of a line width per layer and
   follows a cone of height branch_radius, then the branch widens by tan(diameter angle) times the layer height. The
   foot widening near the build plate follows the real layer heights too: a branch may widen by at most bp_gain[L]
   per step (40 degrees, and at most half a line width), and the radius it tries to reach on layer L is foot_target[L], a
   cone anchored at the top of layer 0 whose radius is bp_radius there. Collision Z gaps use per-layer windows
   from the real Z: layers closer than the gap enter the window, within the Z rounding tolerance. The minimum height of a
   branch above the model and the search depth of the roots are physical distances measured on the real layer tops
   (reaches_min_height_to_model, lowest_layer_within). Between two consecutive
   layers, the maximum move of the branch centres is tan(effective angle) times the height of the upper layer.

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

    // Nominal layer height of the object (scaled).
    coord_t layer_height = 0;
    // Simplification tolerance and smallest feature size of the object outlines (scaled).
    coord_t resolution = 0;
    coord_t min_feature_size = 0;
    // Width of one support line, and the radii of the tip and of the branches (scaled).
    coord_t support_line_width = 0;
    coord_t branch_radius = 0;
    coord_t min_radius = 0;
    // Radius that a branch tries to reach on the build plate (top of layer 0).
    coord_t bp_radius = 0;
    // Spacing of the cache radii (TreeVolumes::ceilRadius()) in the band where a branch widens into its foot: half the diameter of
    // the largest nozzle of the support (the support material and the support interface extruders), 0.2 mm for 0.4 mm nozzles.
    coord_t foot_cache_step = 0;
    // Radius from which the radius is always increased.
    coord_t increase_radius_until_radius = 0;
    // Radius gain (scaled, above min_radius) of a branch: it is gained step by step, a step going from a layer to the one
    // below it and being indexed by its upper layer L, whose height is h(L) = layer_print_z[L] - layer_print_z[L - 1]
    // (layer_print_z[-1] = 0). While the gain is below the tip total (branch_radius - min_radius) the branch is a tip and
    // gains tip_gain[L] = min(support_line_width / 3, h(L) * (branch_radius - min_radius) / branch_radius), a cone of
    // height branch_radius capped by the deposition limit of a layer; then it gains cone_gain[L] = tan(diameter angle) * h(L).
    // A tip whose gain comes within 1 scaled unit of the tip total is ended (its gain becomes the tip total). With
    // min_radius == branch_radius there is no tip: the cone starts at the first step.
    std::vector<double> tip_gain;
    std::vector<double> cone_gain;
    // Foot widening of a branch that rests on the bed, per step indexed by its upper layer L >= 1 (scaled):
    // bp_gain[L] = min(tan(0.7) * h(L), support_line_width / 2), i.e. 40 degrees and at most half a line overlap per layer.
    // It is the foot budget of a step after the normal growth (tip or cone) of that step: the foot adds what the normal growth
    // leaves of it, and the budget does not cap the normal growth. bp_gain[0] is 0 (no step has layer 0 as its upper layer).
    std::vector<double> bp_gain;
    // The radius (scaled) that a branch resting on the bed should have on layer L: branch_radius plus what is left of
    // (bp_radius - branch_radius) after removing bp_gain[1 .. L], never below branch_radius. The foot is active on the layers
    // where foot_target[L] > branch_radius; with branch_radius >= bp_radius there is no foot.
    std::vector<double> foot_target;
    // branch_radius - min_radius (scaled): the gain at which the tip ends.
    double tip_total_gain = 0;
    // tan(diameter angle): the radius gained by the cone per unit of height.
    double cone_slope = 0;
    // Minimum height (scaled) between the tip of a branch and an element that rests on the model.
    coord_t min_height_to_model = 0;
    // Real top Z (scaled) of every object layer, bottom first.
    std::vector<coord_t> layer_print_z;
    // Radius increase allowed when a model branch merges with branches that reach the bed.
    coord_t max_to_model_radius_increase = 0;
    // False when the object only allows branches on the build plate.
    bool support_rests_on_model = true;
    // XY distance from the object (scaled): the user's support_material_xy_spacing, at least 0.1 mm for a soluble support. The tree uses it
    // everywhere, so xy_min_distance is kept equal to it; the min-XY variants of the routing and of the volumes are dormant code kept for
    // a later cleanup.
    coord_t xy_distance = 0;
    coord_t xy_min_distance = 0;
    // Collision windows from real Z, indexed by object layer; the top continues virtually at the last layer height.
    // Clearances in [gap - eps, gap), with eps = scale_to_layer_coord(EPSILON), are deliberately excluded: Z rounding must not
    // add a layer when an exact-gap boundary is rounded slightly closer, preserving integer gap/height counts.
    std::vector<uint32_t> z_distance_top_layers;
    std::vector<uint32_t> z_distance_bottom_layers;
    // Gaps within the Z rounding tolerance are zero gaps; true means both gaps exceed it, so every layer
    // has its existing neighbours in its windows (the last layer uses a virtual layer above, the first has none below).
    bool z_distance_both_sides = false;
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

    // The gain (scaled) of a branch that has gained radius in every step so far, after one more step that goes from upper_layer to
    // upper_layer - 1: a tip gains tip_gain[upper_layer] (and ends when it comes within 1 unit of tip_total_gain), a branch whose
    // tip ended gains cone_gain[upper_layer]. A gain that is not advanced (a pause) stays as it is.
    // Precondition: upper_layer is a valid object layer above layer 0 (a step goes down to a layer that exists; asserted).
    [[nodiscard]] double advance(double gain, LayerIndex upper_layer) const;
    // The gain (scaled) of a branch that gained radius at every step since its tip, on layer layer_idx + distance_to_top, down to
    // layer layer_idx: the result of advance() applied to the layers layer_idx + distance_to_top .. layer_idx + 1, in O(1) from
    // prefix sums. It is 0 for distance_to_top == 0.
    // Preconditions: layer_idx >= 0 and layer_idx + distance_to_top is a valid object layer (both checked by asserts).
    [[nodiscard]] double path_gain(LayerIndex layer_idx, size_t distance_to_top) const;

    // True when the tip of an element on layer_idx, distance_to_top layers above it (so on layer layer_idx + distance_to_top),
    // is at least min_height_to_model above the top of layer_idx, measured on the real layer tops (layer_print_z). eps
    // (scale_to_layer_coord(EPSILON), 0.1 um) absorbs the rounding of the two layer tops. Const, no side effect.
    // Preconditions: layer_idx is a valid object layer and the tip layer layer_idx + distance_to_top is below layer_print_z.size()
    // (checked by an assert, as the routing never puts a tip above the last layer).
    [[nodiscard]] bool reaches_min_height_to_model(LayerIndex layer_idx, size_t distance_to_top) const;
    // Lowest layer j <= layer_idx whose top is at most depth below the top of layer_idx (within the same eps); never below layer 0,
    // and layer_idx itself when the layer below is farther than depth. Const, no side effect.
    // Preconditions: layer_idx is a valid object layer (no assert) and depth >= 0, in scaled units.
    [[nodiscard]] LayerIndex lowest_layer_within(LayerIndex layer_idx, coord_t depth) const;

    // Maximum move of a branch centre from the layer below into layer_idx, fast and slow (scaled).
    [[nodiscard]] coord_t maximum_move_distance(LayerIndex layer_idx) const { return max_move[layer_idx]; }
    [[nodiscard]] coord_t maximum_move_distance_slow(LayerIndex layer_idx) const { return max_move_slow[layer_idx]; }

    // The per layer maximum moves, indexed by object layer.
    std::vector<coord_t> max_move;
    std::vector<coord_t> max_move_slow;

private:
    // Sums of tip_gain: tip_gain_prefix[k] is the sum of tip_gain[0 .. k - 1] (size layer_count + 1).
    std::vector<double> tip_gain_prefix;
    // For a tip on layer T, the first layer E (counted downwards) at which the gain of a branch that gained at every step has
    // reached tip_total_gain (within 1 unit). E == T when the tip is already ended at its first layer; -1 when it does not end
    // on a layer >= 0. Below E, the gain is tip_total_gain plus the cone.
    std::vector<LayerIndex> tip_end_layer;
};

} // namespace TreeFamily
} // namespace Slic3r

#endif // plugins_support_tree_settings_hpp_
