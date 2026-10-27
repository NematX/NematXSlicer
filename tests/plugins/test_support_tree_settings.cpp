#include "support_grid_test_helpers.hpp"
#include "support_tree_test_helpers.hpp"
#include "libslic3r/Plugins/Support/Common/SupportZPlacement.hpp"
#include "libslic3r/Plugins/Support/Tree/TreeElements.hpp"
#include "libslic3r/Support/TreeSupportCommon.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

/* TreeSettings tests. The plugin TreeSettings (built from the plugin API views) is compared with the native TreeSupportSettings of the
   legacy tree generator (src/libslic3r/Support/TreeSupportCommon.cpp), field by field and exactly, for the default configuration and
   for a table of overrides that exercise the places where the two derivations could part: percent and absolute slab heights above and
   below the cap, a thin branch, a layer height above the maximum layer height, and a variable layer-height profile.

   Only fields that exist on both sides are compared with the native settings. These quantities differ and are checked explicitly:
   - collision windows: the plugin uses real Z and a strict physical gap, with EPSILON for Z rounding; native counts are nominal.
   - the radius law: the plugin gains radius per layer step from the real height of the layer (tip_gain, cone_gain, advance,
     path_gain), where the native settings count steps of the nominal height (tip_layers, increase_radius_until_layer). They agree on
     uniform layers with the default settings, where the drawn radius of a branch that grew at every step equals getRadius(n); the
     per-layer gains have hand-computed values on the variable profile.
   - maximum moves: the native settings hold one move for the nominal layer height; the plugin holds one per real layer height. They
     are compared on the layers whose height is the nominal one, and every layer is compared with tan(angle) * its own height.
   - the minimum height to the model and the root search depth: the plugin measures them on the real layer tops, the native
     settings count nominal layers (min_dtt_to_model). They agree only on uniform layers above the first, the identity domain
     checked against the native count; the variable profile has hand-computed values.
   - the branch angle: the plugin caps it by support_tree_max_overhang; the cases set it high enough for the cap to be inactive.
   - the foot near the bed: bp_gain[L] and foot_target[L] follow the real layer heights; the native settings count nominal layers
     (bp_radius_increase_per_layer, layer_start_bp_radius). The step budget agrees with the native one on nominal layers; the
     targets have hand-computed values on a profile of three bands. foot_step_gain() and merge_support_element_states() are
     tested on hand-built states.
   The slab heights (min, max and dense) exist only in the plugin: they are compared with the rule written from the configuration.
   Zero and sub-EPSILON branch angles are checked only on the plugin: its slow angle stays nonnegative, while the native slow-angle
   clamp has reversed bounds for these angles. The slow angle is local to construction, so its zero value is checked through every move. */

using namespace Slic3r;
namespace SG = Slic3r::Test::SupportGrid;
namespace ST = Slic3r::Test::SupportTree;
namespace TF = Slic3r::TreeFamily;

namespace {
// One configuration to compare: the options to override, the layer height profile (empty: uniform layers) and whether the native
// settings can be built for it (they assert a positive tip_layers).
struct SettingsCase {
    const char *name;
    std::vector<std::pair<std::string, std::string>> overrides;
    std::vector<coordf_t> profile;
    bool native;
};

// The nozzle diameter and the maximum layer height that SG::config() sets, in millimeters.
constexpr double nozzle_mm = 0.4, max_layer_mm = 0.3;

// Slice an owned print for item and check its settings against native values, physical Z windows and slab heights; failures reach Catch.
void check_case(const SettingsCase &item);

// The slab height rule written from the configuration: 0 is the maximum, a percent is of the nozzle, the maximum caps it.
double expected_slab_mm(const std::string &setting)
{
    const bool percent = !setting.empty() && setting.back() == '%';
    const double value = std::stod(setting);
    if (value == 0.)
        return max_layer_mm;
    return std::min(max_layer_mm, percent ? nozzle_mm * value / 100. : value);
}

// Builds and owns a sliced print, compares settings with native values and physical-gap enumeration, and reports mismatches through
// Catch. The case supplies configuration and optional profile; slicing and allocation failures propagate without shared-state changes.
void check_case(const SettingsCase &item)
{
    INFO("case " << item.name);
    DynamicPrintConfig config = SG::config("tree", false, 3);
    // A large maximum overhang keeps its cap on the branch angle inactive, as the native settings have no such cap.
    config.set_deserialize_strict({{"support_tree_max_overhang", "100"}});
    for (const std::pair<std::string, std::string> &option : item.overrides)
        config.set_deserialize_strict({{option.first, option.second}});
    Print print;
    Model model;
    if (item.profile.empty())
        Test::init_print({SG::slab(true)}, print, model, config);
    else
        SG::init_print_with_profile(print, model, SG::slab(true), item.profile, config);
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    const slic3r_api::Support::Common::SupportParams params = SG::params(print, object);
    REQUIRE(params.max_support_height == Approx(max_layer_mm));
    const TF::TreeSettings settings = ST::tree_settings(print, object, params);

    if (item.native) {
        const FFFTreeSupport::TreeSupportMeshGroupSettings mesh_group(object);
        const FFFTreeSupport::TreeSupportSettings native(mesh_group, object.slicing_parameters());
        CHECK(settings.support_line_width == native.support_line_width);
        CHECK(settings.resolution == native.resolution);
        CHECK(settings.min_feature_size == native.min_feature_size);
        CHECK(settings.branch_radius == native.branch_radius);
        CHECK(settings.min_radius == native.min_radius);
        CHECK(settings.bp_radius == native.bp_radius);
        // The foot step budget of every layer of the nominal height above the first is the native one.
        for (size_t layer = 1; layer < object.layer_count(); ++layer)
            if (object.layer(layer).scaled_height() == native.layer_height)
                CHECK(settings.bp_gain[layer] == Approx(native.bp_radius_increase_per_layer).margin(1e-6));
        CHECK(settings.min_height_to_model == scale_i(1.));
        // Uniform layers above the first: the physical test equals the native layer count, whatever the starting layer.
        bool uniform = item.profile.empty();
        for (size_t layer = 1; layer < object.layer_count(); ++layer)
            uniform = uniform && object.layer(layer).scaled_height() == native.layer_height;
        if (uniform) {
            for (size_t layer : {size_t(1), size_t(2), object.layer_count() / 2})
                for (size_t dtt = 0; dtt <= native.min_dtt_to_model + 2 && layer + dtt < object.layer_count(); ++dtt)
                    CHECK(settings.reaches_min_height_to_model(TF::LayerIndex(layer), dtt) == (dtt >= native.min_dtt_to_model));
            // lowest_layer_within is the layer count that the depth covers, from the layer down (an exact multiple counts), clamped at layer 0.
            const TF::LayerIndex top = TF::LayerIndex(object.layer_count() - 1);
            for (const coord_t depth : {coord_t(0), native.layer_height, scale_i(0.5), scale_i(1.), scale_i(1.3), scale_i(1000.)})
                CHECK(settings.lowest_layer_within(top, depth) == std::max<TF::LayerIndex>(0, top - TF::LayerIndex(depth / native.layer_height)));
            // The radius of a branch that grew at every step since its tip (min_radius + path_gain) is the native radius of the same
            // number of steps, within one unit. The steps above layer 0 all have the nominal height.
            for (size_t steps = 0; steps <= native.tip_layers + 5 && steps < object.layer_count(); ++steps) {
                CAPTURE(steps);
                CHECK(std::abs(double(settings.min_radius) + settings.path_gain(0, steps) - double(native.getRadius(steps))) <= 1.);
            }
        }
        CHECK(settings.max_to_model_radius_increase == native.max_to_model_radius_increase);
        CHECK(settings.support_rests_on_model == native.support_rests_on_model);
        CHECK(settings.xy_distance == native.xy_distance);
        if (item.profile.empty()) {
            for (size_t layer = 0; layer < object.layer_count(); ++layer) {
                CHECK(settings.z_distance_top_layers[layer] == native.z_distance_top_layers);
                CHECK(settings.z_distance_bottom_layers[layer] == std::min<size_t>(native.z_distance_bottom_layers, layer));
            }
        }
        CHECK(settings.branch_distance == native.settings.support_tree_branch_distance);
        CHECK(settings.top_rate == native.settings.support_tree_top_rate);
        CHECK(settings.angle == native.settings.support_tree_angle);
        // The native move is for the nominal height: it is the plugin move of every layer that has that height.
        size_t nominal = 0;
        for (size_t layer = 0; layer < object.layer_count(); ++layer)
            if (object.layer(layer).scaled_height() == native.layer_height) {
                CHECK(settings.max_move[layer] == native.maximum_move_distance);
                CHECK(settings.max_move_slow[layer] == native.maximum_move_distance_slow);
                ++nominal;
            }
        // A nominal height above the maximum layer height is never sliced (the layers are cut at the maximum): no layer is then comparable,
        // and the moves of the plugin follow the real heights, which the formula check below covers.
        CHECK((nominal > 0) == (native.layer_height <= scale_to_layer_coord(max_layer_mm)));
    }

    if (! item.native) {
        // The tip is a single step: tip_gain (a third of a line width or the cone) exceeds the tip total, so one step ends it.
        const TF::LayerIndex layer = 2;
        REQUIRE(size_t(layer) + 1 < object.layer_count());
        CHECK(settings.tip_total_gain > 0.);
        CHECK(settings.path_gain(layer, 1) == settings.tip_total_gain);
        CHECK(settings.advance(0., layer) == settings.tip_total_gain);
    }

    // Every layer gains radius by its own height: the tip by the cone of height branch_radius capped by a third of a line width, then
    // the cone of the diameter angle.
    const double diameter_slope = std::tan(config.opt_float("support_tree_branch_diameter_angle") * PI / 180.);
    CHECK(settings.tip_total_gain == double(settings.branch_radius - settings.min_radius));
    REQUIRE(settings.tip_gain.size() == object.layer_count());
    REQUIRE(settings.cone_gain.size() == object.layer_count());
    for (size_t layer = 0; layer < object.layer_count(); ++layer) {
        const double height = double(object.layer(layer).scaled_print_z() - (layer > 0 ? object.layer(layer - 1).scaled_print_z() : 0));
        CHECK(settings.tip_gain[layer] == Approx(std::min(double(settings.support_line_width) / 3., height * settings.tip_total_gain / double(settings.branch_radius))).margin(1e-6));
        CHECK(settings.cone_gain[layer] == Approx(diameter_slope * height).margin(1e-6));
    }

    // Every layer moves by tan(angle) times its own height, fast and slow.
    REQUIRE(settings.max_move.size() == object.layer_count());
    REQUIRE(settings.z_distance_top_layers.size() == object.layer_count());
    REQUIRE(settings.z_distance_bottom_layers.size() == object.layer_count());
    size_t other_heights = 0;
    const coord_t nominal_height = scale_to_layer_coord(object.config().layer_height.value);
    for (size_t layer = 0; layer < object.layer_count(); ++layer) {
        const coord_t height = object.layer(layer).scaled_height();
        CHECK(settings.max_move[layer] == coord_t(std::tan(settings.angle) * double(height)));
        other_heights += height != nominal_height;
        // Independent enumeration checks strict clearance, including virtual layers above the last real layer.
        const coord_t eps = scale_to_layer_coord(EPSILON);
        const coord_t top_limit = object.layer(layer).scaled_print_z() + scale_to_layer_coord(params.gap_above) - eps;
        uint32_t top = 0, bottom = 0;
        for (size_t j = layer + 1; j < object.layer_count(); ++j)
            top += object.layer(j).scaled_bottom_z() < top_limit;
        const Layer &last = object.layer(object.layer_count() - 1);
        for (coord_t z = last.scaled_print_z(); z < top_limit; z += last.scaled_height())
            ++top;
        const coord_t bottom_limit = object.layer(layer).scaled_bottom_z() - scale_to_layer_coord(params.gap_below) + eps;
        for (size_t j = 0; j < layer; ++j)
            bottom += object.layer(j).scaled_print_z() > bottom_limit;
        CHECK(settings.z_distance_top_layers[layer] == top);
        CHECK(settings.z_distance_bottom_layers[layer] == bottom);
    }
    if (!item.profile.empty())
        CHECK(other_heights > 0);

    // The slab heights follow the rule written from the configuration.
    std::string max_setting = "0", interface_setting = "0";
    for (const std::pair<std::string, std::string> &option : item.overrides) {
        if (option.first == "support_material_layer_height")
            max_setting = option.second;
        else if (option.first == "support_material_interface_layer_height")
            interface_setting = option.second;
    }
    CHECK(settings.min_slab_height == scale_to_layer_coord(params.support_min_height));
    CHECK(settings.max_slab_height == scale_to_layer_coord(expected_slab_mm(max_setting)));
    CHECK(settings.dense_slab_height ==
          slic3r_api::Support::Common::handed_over_dense_height(scale_to_layer_coord(expected_slab_mm(interface_setting)), settings.min_slab_height, settings.z_step));
}
}

// The plugin TreeSettings equal the native TreeSupportSettings, and the slab heights follow their rule, for a table of overrides.
TEST_CASE("Tree settings equal the native tree support settings", "[plugins][support-tree-settings]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const std::vector<SettingsCase> cases{
        {"defaults", {}, {}, true},
        {"slab heights as percents", {{"support_material_layer_height", "50%"}, {"support_material_interface_layer_height", "60%"}}, {}, true},
        {"slab heights as percents above the cap", {{"support_material_layer_height", "200%"}, {"support_material_interface_layer_height", "300%"}}, {}, true},
        {"slab heights absolute below the cap", {{"support_material_layer_height", "0.1"}, {"support_material_interface_layer_height", "0.15"}}, {}, true},
        {"slab heights absolute above the cap", {{"support_material_layer_height", "5"}, {"support_material_interface_layer_height", "6"}}, {}, true},
        {"branch under three support line widths", {{"support_tree_branch_diameter", "0.8"}, {"support_tree_tip_diameter", "0.4"}}, {}, true},
        {"branch and tip that leave no tip layer", {{"support_tree_branch_diameter", "0.3"}, {"support_tree_tip_diameter", "0.2"}}, {}, false},
        {"layer height above the maximum layer height", {{"layer_height", "0.4"}}, {}, true},
        {"variable layer height profile", {}, {0., 0.2, 1., 0.05, 2., 0.2, 2.4, 0.05, 2.6, 0.3, 5., 0.3}, true},
    };
    for (const SettingsCase &item : cases)
        check_case(item);
}

// Checks real-Z windows in constant-height bands and at their boundary, then checks the zero-gap rounding tolerance on the same owned print.
TEST_CASE("Tree settings use real Z at a height-band transition and treat tiny gaps as zero", "[plugins][support-tree-settings]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = SG::config("tree", false, 3);
    Print print;
    Model model;
    SG::init_print_with_profile(print, model, make_cube(2., 2., 6.), {0., 0.05, 2., 0.05, 2.01, 0.3, 6., 0.3}, config);
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    slic3r_api::Support::Common::SupportParams params = SG::params(print, object);
    const TF::TreeSettings settings = ST::tree_settings(print, object, params);
    size_t transition = 1;
    while (transition < object.layer_count() && object.layer(transition).scaled_height() == scale_to_layer_coord(0.05))
        ++transition;
    REQUIRE(transition >= 6);
    REQUIRE(transition + 2 < object.layer_count());
    CAPTURE(transition, object.layer(transition).unscaled_height(), object.layer(transition).unscaled_print_z());
    REQUIRE(object.layer(transition).scaled_height() == scale_to_layer_coord(0.3));
    // At the first 0.3 layer, the previous thin layers have clearances 0, 0.05, 0.10, 0.15: four below gap=0.2.
    // Above it only the neighbour has clearance zero; the next clearance is 0.3. At the last thin layer the same counts apply.
    CHECK(settings.z_distance_bottom_layers[transition] == 4);
    CHECK(settings.z_distance_top_layers[transition] == 1);
    CHECK(settings.z_distance_bottom_layers[transition - 1] == 4);
    CHECK(settings.z_distance_top_layers[transition - 1] == 1);
    // Away from the boundary, four thin layers or one thick layer fit in the 0.2 gap in either direction.
    CHECK(settings.z_distance_top_layers[transition - 6] == 4);
    CHECK(settings.z_distance_bottom_layers[transition - 6] == 4);
    CHECK(settings.z_distance_top_layers[transition + 2] == 1);
    CHECK(settings.z_distance_bottom_layers[transition + 2] == 1);
    for (const double gap : {0., EPSILON / 2., EPSILON, 2. * EPSILON}) {
        params.gap_above = gap;
        params.gap_below = gap;
        const TF::TreeSettings tiny = ST::tree_settings(print, object, params);
        const bool positive = scale_to_layer_coord(gap) > scale_to_layer_coord(EPSILON);
        CHECK(tiny.z_distance_both_sides == positive);
        for (size_t layer = 0; layer < object.layer_count(); ++layer) {
            CHECK(tiny.z_distance_top_layers[layer] == uint32_t(positive));
            CHECK(tiny.z_distance_bottom_layers[layer] == uint32_t(positive && layer > 0));
        }
    }
}

// Checks the physical minimum height to the model and the physical root search depth in the 0.05 band, the 0.3 band and across the
// transition between them, with expectations computed by hand from the layer heights.
TEST_CASE("Tree settings measure the height to the model and the root depth in real Z", "[plugins][support-tree-settings]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = SG::config("tree", false, 3);
    Print print;
    Model model;
    SG::init_print_with_profile(print, model, make_cube(2., 2., 6.), {0., 0.05, 2., 0.05, 2.01, 0.3, 6., 0.3}, config);
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    const TF::TreeSettings settings = ST::tree_settings(print, object, SG::params(print, object));
    size_t transition = 1;
    while (transition < object.layer_count() && object.layer(transition).scaled_height() == scale_to_layer_coord(0.05))
        ++transition;
    REQUIRE(transition >= 12);
    REQUIRE(transition + 8 < object.layer_count());
    REQUIRE(object.layer(transition).scaled_height() == scale_to_layer_coord(0.3));
    REQUIRE(object.layer(transition + 8).scaled_height() == scale_to_layer_coord(0.3));
    REQUIRE(settings.layer_print_z.size() == object.layer_count());
    const TF::LayerIndex t = TF::LayerIndex(transition);
    CHECK(settings.min_height_to_model == scale_i(1.));

    // Thin band: 1 mm is 20 layers of 0.05, so 19 layers are too low and 20 are enough.
    CHECK(! settings.reaches_min_height_to_model(2, 19));
    CHECK(settings.reaches_min_height_to_model(2, 20));
    // Thick band: 1 mm is 4 layers of 0.3 (1.2 mm); 3 layers (0.9 mm) are too low.
    CHECK(! settings.reaches_min_height_to_model(t + 1, 3));
    CHECK(settings.reaches_min_height_to_model(t + 1, 4));
    // Across the transition: layer t - 1 ends at z_t and layer t - 10 ends 9 thin layers (0.45 mm) lower. The tip on layer t is
    // 0.45 + 0.3 = 0.75 mm up (too low) and the tip on layer t + 1 is 1.05 mm up (enough).
    CHECK(! settings.reaches_min_height_to_model(t - 10, 10));
    CHECK(settings.reaches_min_height_to_model(t - 10, 11));

    // Thin band: 0.5 mm down is 10 layers of 0.05.
    CHECK(settings.lowest_layer_within(t - 1, scale_i(0.5)) == t - 11);
    // Thick band: 0.6 mm down is 2 layers of 0.3, and 0.59 mm only one.
    CHECK(settings.lowest_layer_within(t + 5, scale_i(0.6)) == t + 3);
    CHECK(settings.lowest_layer_within(t + 5, scale_i(0.59)) == t + 4);
    // Across the transition: the top of layer t + 1 is z_t + 0.6; 0.9 mm down reaches z_t - 0.3, the top of layer t - 7 (the thick
    // layers t and t + 1 cover 0.6 mm and 6 thin layers 0.3 mm).
    CHECK(settings.lowest_layer_within(t + 1, scale_i(0.9)) == t - 7);
    // A depth longer than the object stops at layer 0.
    CHECK(settings.lowest_layer_within(3, scale_i(100.)) == 0);
}

// The radius gains per layer step on a profile of three bands, thick at the bottom, then 0.2, then 0.05 at the top (the branches
// grow downwards, from the thin band into the thick one). The default tip is 0.8 mm wide (radius 0.4) and the branch 2 mm wide
// (radius 1), so the tip gains 0.6 mm of radius: min(line width / 3, h * 0.6) per layer, then tan(5 degrees) * h.
TEST_CASE("Tree settings gain radius from the real height of each layer", "[plugins][support-tree-settings]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = SG::config("tree", false, 3);
    Print print;
    Model model;
    SG::init_print_with_profile(print, model, make_cube(2., 2., 4.2), {0., 0.3, 2., 0.3, 2.01, 0.2, 3., 0.2, 3.01, 0.05, 4.2, 0.05}, config);
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    const TF::TreeSettings settings = ST::tree_settings(print, object, SG::params(print, object));
    REQUIRE(settings.branch_radius == scale_i(1.));
    REQUIRE(settings.min_radius == scale_i(0.4));
    REQUIRE(config.opt_float("support_tree_branch_diameter_angle") == Approx(5.));
    const double slope = std::tan(5. * PI / 180.);
    const double third_line = double(settings.support_line_width) / 3.;
    // The tip cones (h * 0.6) must be below a third of a line width in the thin band and above it in no case that the hand values hide.
    REQUIRE(third_line > scale_d(0.05) * 0.6);

    const size_t layer_count = object.layer_count();
    REQUIRE(settings.tip_gain.size() == layer_count);
    std::vector<double> heights;
    for (size_t layer = 0; layer < layer_count; ++layer)
        heights.push_back(double(object.layer(layer).scaled_print_z() - (layer > 0 ? object.layer(layer - 1).scaled_print_z() : 0)));
    // The layers of the bands: the first 0.2 layer (above the thick band, it has thick layers below it), a thick layer, a thin layer.
    size_t mid_first = 2;
    while (mid_first < layer_count && object.layer(mid_first).scaled_height() != scale_to_layer_coord(0.2))
        ++mid_first;
    REQUIRE(mid_first + 3 < layer_count);
    REQUIRE(mid_first >= 4);
    const size_t thick = mid_first - 2, mid = mid_first + 1, thin = layer_count - 1;
    CHECK(object.layer(thick).scaled_height() == scale_to_layer_coord(0.3));
    CHECK(object.layer(mid_first - 1).scaled_height() == scale_to_layer_coord(0.3));
    CHECK(object.layer(mid).scaled_height() == scale_to_layer_coord(0.2));
    CHECK(object.layer(mid_first + 1).scaled_height() == scale_to_layer_coord(0.2));
    CHECK(object.layer(thin).scaled_height() == scale_to_layer_coord(0.05));
    REQUIRE(thin > mid + 22);
    // Hand values of one layer of each band: the thin band is limited by the cone of the tip (0.05 * 0.6 mm), the others by a third of
    // a line width or by their own cone (0.2 * 0.6, 0.3 * 0.6 mm).
    CHECK(settings.tip_gain[thin] == Approx(scale_d(0.05) * 0.6).margin(1.));
    CHECK(settings.tip_gain[mid] == Approx(std::min(third_line, scale_d(0.2) * 0.6)).margin(1.));
    CHECK(settings.tip_gain[thick] == Approx(std::min(third_line, scale_d(0.3) * 0.6)).margin(1.));
    CHECK(settings.cone_gain[thin] == Approx(slope * scale_d(0.05)).margin(1.));
    CHECK(settings.cone_gain[mid] == Approx(slope * scale_d(0.2)).margin(1.));
    CHECK(settings.cone_gain[thick] == Approx(slope * scale_d(0.3)).margin(1.));
    for (size_t layer = 1; layer < layer_count; ++layer) {
        CHECK(settings.tip_gain[layer] == Approx(std::min(third_line, heights[layer] * 0.6)).margin(1e-6));
        CHECK(settings.cone_gain[layer] == Approx(slope * heights[layer]).margin(1e-6));
    }

    // A tip that is five thin layers old has gained five thin tip gains.
    const TF::LayerIndex top = TF::LayerIndex(thin);
    CHECK(settings.path_gain(top - 5, 5) == Approx(5. * scale_d(0.05) * 0.6).margin(5.));
    CHECK(settings.path_gain(top, 0) == 0.);
    // Across the transition from the 0.2 band to the thick one: a tip on the second 0.2 layer has two 0.2 steps and then one thick step.
    CHECK(settings.path_gain(TF::LayerIndex(mid_first) - 2, 3) == Approx(2. * std::min(third_line, scale_d(0.2) * 0.6) + std::min(third_line, scale_d(0.3) * 0.6)).margin(2.));
    // Once the tip has ended in the thin band, the gain grows by the cone over the real height, whatever the heights crossed: from
    // the thin band down to the 0.2 band and down to the thick band.
    const TF::LayerIndex mid_layer = TF::LayerIndex(mid), thick_layer = TF::LayerIndex(thick);
    const double tip_total = double(settings.branch_radius - settings.min_radius);
    REQUIRE(settings.path_gain(mid_layer, size_t(top - mid_layer)) > tip_total);
    CHECK(settings.path_gain(thick_layer, size_t(top - thick_layer)) - settings.path_gain(mid_layer, size_t(top - mid_layer)) ==
          Approx(slope * double(object.layer(mid).scaled_print_z() - object.layer(thick).scaled_print_z())).margin(1.));
    // Every (tip layer, layer) pair equals the gain of a branch that grew step by step, written from the layer heights.
    for (size_t tip = 0; tip < layer_count; ++tip) {
        double gain = 0.;
        for (size_t upper = tip; upper > 0; --upper) {
            if (gain >= tip_total - 1.)
                gain = std::max(gain, tip_total) + slope * heights[upper];
            else {
                gain += std::min(third_line, heights[upper] * tip_total / double(settings.branch_radius));
                if (gain >= tip_total - 1.)
                    gain = tip_total;
            }
            CAPTURE(tip, upper);
            CHECK(settings.path_gain(TF::LayerIndex(upper - 1), tip - (upper - 1)) == Approx(gain).margin(1e-3));
        }
    }

    // advance() follows the same steps one at a time; a pause (no advance) leaves the gain as it is, and the branch then grows less
    // than the path without pause.
    const TF::LayerIndex tip = top;
    const double first = settings.advance(0., tip);
    CHECK(first == Approx(settings.tip_gain[tip]));
    CHECK(first == Approx(settings.path_gain(tip - 1, 1)));
    const double resumed = settings.advance(first, tip - 2);
    CHECK(resumed == Approx(first + settings.tip_gain[tip - 2]));
    CHECK(resumed < settings.path_gain(tip - 3, 3));
    double gain = 0.;
    for (TF::LayerIndex upper = tip; upper > mid_layer; --upper)
        gain = settings.advance(gain, upper);
    CHECK(gain == Approx(settings.path_gain(mid_layer, size_t(tip - mid_layer))).margin(1e-3));
}

// A branch as wide as its tip has no tip: the cone starts at the first step, whatever the layer.
TEST_CASE("Tree settings start the cone at the first step when the tip equals the branch", "[plugins][support-tree-settings]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = SG::config("tree", false, 3);
    config.set_deserialize_strict({{"support_tree_branch_diameter", "1"}, {"support_tree_tip_diameter", "1"}});
    Print print;
    Model model;
    Test::init_print({SG::slab(true)}, print, model, config);
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    const TF::TreeSettings settings = ST::tree_settings(print, object, SG::params(print, object));
    REQUIRE(settings.min_radius == settings.branch_radius);
    CHECK(settings.tip_total_gain == 0.);
    REQUIRE(object.layer_count() > 6);
    for (TF::LayerIndex layer = 1; layer < TF::LayerIndex(object.layer_count()); ++layer)
        CHECK(settings.advance(0., layer) == Approx(settings.cone_gain[layer]).margin(1e-6));
    for (TF::LayerIndex layer = 0; layer + 5 < TF::LayerIndex(object.layer_count()); ++layer)
        for (size_t steps = 1; steps <= 5; ++steps) {
            const double height = double(settings.layer_print_z[size_t(layer) + steps] - settings.layer_print_z[size_t(layer)]);
            CHECK(settings.path_gain(layer, steps) == Approx(settings.cone_slope * height).margin(1e-3));
        }
}

// Uniform layers of the maximum height (0.3 mm): the plugin tip gains min(line width / 3, h * (B - m) / B) per step, which is a third
// of a line width here. The native tip uses integer divisions (tip_layers = max((B - m) / (w / 3), B / h) = 4 steps of (B - m) / 4),
// so its steps exceed a third of a line width: the plugin tip needs 5 steps, and after 4 steps it is shorter than the native one by
// B - (m + 4 * w / 3): measured 40001 scaled units (0.04 mm) with the default 0.42 mm line (plugin radius 959998.7, native 1000000).
TEST_CASE("Tree settings tip gain is limited by a third of a line width at the maximum layer height", "[plugins][support-tree-settings]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = SG::config("tree", false, 3);
    config.set_deserialize_strict({{"layer_height", "0.3"}});
    Print print;
    Model model;
    Test::init_print({SG::slab(true)}, print, model, config);
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    const TF::TreeSettings settings = ST::tree_settings(print, object, SG::params(print, object));
    const FFFTreeSupport::TreeSupportMeshGroupSettings mesh_group(object);
    const FFFTreeSupport::TreeSupportSettings native(mesh_group, object.slicing_parameters());
    const double third_line = double(settings.support_line_width) / 3.;
    REQUIRE(native.layer_height == scale_to_layer_coord(0.3));
    REQUIRE(object.layer_count() > 8);
    for (size_t layer = 1; layer < object.layer_count(); ++layer)
        CHECK(settings.tip_gain[layer] == Approx(std::min(third_line, double(object.layer(layer).scaled_height()) * settings.tip_total_gain / double(settings.branch_radius))).margin(1e-3));
    // The deposition limit is the one that applies: h * (B - m) / B = 0.18 mm is above a third of the line.
    CHECK(settings.tip_gain[2] == Approx(third_line).margin(1e-3));
    CHECK(native.tip_layers == 4);
    const double plugin_radius = double(settings.min_radius) + settings.path_gain(2, 4);
    CHECK(plugin_radius == Approx(double(settings.min_radius) + 4. * third_line).margin(1e-3));
    CHECK(double(native.getRadius(4)) - plugin_radius == Approx(double(settings.branch_radius) - (double(settings.min_radius) + 4. * third_line)).margin(1.));
    CAPTURE(plugin_radius, native.getRadius(4));
    CHECK(settings.path_gain(2, 5) == settings.tip_total_gain);
}

// A tip that starts in a thick (0.3) band and ends in the thin (0.05) band below it: thick steps first, then thin ones.
TEST_CASE("Tree settings path gain crosses a transition from a thick band to a thin one", "[plugins][support-tree-settings]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = SG::config("tree", false, 3);
    Print print;
    Model model;
    SG::init_print_with_profile(print, model, make_cube(2., 2., 4.2), {0., 0.05, 1., 0.05, 1.01, 0.3, 4.2, 0.3}, config);
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    const TF::TreeSettings settings = ST::tree_settings(print, object, SG::params(print, object));
    REQUIRE(settings.branch_radius == scale_i(1.));
    REQUIRE(settings.min_radius == scale_i(0.4));
    size_t first_thick = 2;
    while (first_thick < object.layer_count() && object.layer(first_thick).scaled_height() != scale_to_layer_coord(0.3))
        ++first_thick;
    REQUIRE(first_thick >= 6);
    REQUIRE(first_thick + 3 < object.layer_count());
    CHECK(object.layer(first_thick - 1).scaled_height() == scale_to_layer_coord(0.05));
    CHECK(object.layer(first_thick + 1).scaled_height() == scale_to_layer_coord(0.3));
    // Tip on the second thick layer, four steps down to three layers below the first one: two thick steps (min(w / 3, 0.18 mm)) and
    // two thin steps (0.05 * 0.6 = 0.03 mm).
    const double thick_step = std::min(double(settings.support_line_width) / 3., scale_d(0.3) * 0.6);
    CHECK(settings.path_gain(TF::LayerIndex(first_thick) - 3, 4) == Approx(2. * thick_step + 2. * scale_d(0.05) * 0.6).margin(3.));
}
// Builds owned prints at zero and sub-EPSILON branch angles and checks all slow moves are zero; setup failures propagate to Catch.
TEST_CASE("Tree settings keep slow moves zero at zero and tiny branch angles", "[plugins][support-tree-settings]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    for (const std::string &angle_degrees : {std::string("0"), std::string("0.001")}) {
        INFO("branch angle in degrees " << angle_degrees);
        DynamicPrintConfig config = SG::config("tree", false, 3);
        config.set_deserialize_strict({{"support_tree_angle", angle_degrees}, {"support_tree_angle_slow", "25"}});
        Print print;
        Model model;
        Test::init_print({SG::slab(true)}, print, model, config);
        SG::prefix(print);
        const PrintObject &object = print.object(0);
        const TF::TreeSettings settings = ST::tree_settings(print, object, SG::params(print, object));
        CHECK(settings.angle == Approx(std::stod(angle_degrees) * PI / 180.));
        REQUIRE(object.layer_count() > 0);
        REQUIRE(settings.max_move_slow.size() == object.layer_count());
        for (const coord_t move : settings.max_move_slow)
            CHECK(move == 0);
    }
}

namespace {
// The settings of a 6 mm cube sliced with a profile of three bands above a first layer of 0.3 mm: 0.05 up to 1.3 mm, 0.2 up to
// 3.3 mm, then 0.3. The foot (7.5 mm diameter, 2.75 mm of radius above the 2 mm branch) is spent in the first two bands and the
// start of the third one.
TF::TreeSettings foot_profile_settings(DynamicPrintConfig config)
{
    config.set_deserialize_strict({{"first_layer_height", "0.3"}});
    Print print;
    Model model;
    SG::init_print_with_profile(print, model, make_cube(2., 2., 6.), {0., 0.05, 1.3, 0.05, 1.31, 0.2, 3.3, 0.2, 3.31, 0.3, 6., 0.3}, config);
    SG::prefix(print);
    return ST::tree_settings(print, print.object(0), SG::params(print, print.object(0)));
}

// The height of layer L (scaled): the top Z of the layer minus the one of the layer below (the bed for layer 0).
double layer_height_of(const TF::TreeSettings &settings, size_t layer)
{
    return double(settings.layer_print_z[layer] - (layer > 0 ? settings.layer_print_z[layer - 1] : 0));
}

// A state of a branch on layer layer_idx whose tip is dtt layers above, going to the bed.
TF::SupportElementState foot_state(const TF::LayerIndex layer_idx, const uint32_t dtt, const double radius_gain, const double foot_gain)
{
    TF::SupportElementState state{};
    state.layer_idx = layer_idx;
    state.target_height = layer_idx + TF::LayerIndex(dtt);
    state.distance_to_top = dtt;
    state.radius_gain = radius_gain;
    state.foot_gain = foot_gain;
    state.to_buildplate = true;
    return state;
}

// The state of the step from layer upper_layer to the one below, as increase_single_area() builds it before the foot: the parent
// propagated down, with the gain of the normal growth of that step (advance()).
TF::SupportElementState grown_state(const TF::TreeSettings &settings, const TF::SupportElementState &parent, const TF::LayerIndex upper_layer)
{
    TF::SupportElementState grown = TF::SupportElementState::propagate_down(parent);
    grown.radius_gain = settings.advance(parent.radius_gain, upper_layer);
    return grown;
}
}

// bp_gain is the widening a branch is allowed per layer step: 40 degrees of the real layer height, at most half a line width.
// foot_target is the radius to reach on each layer: the cone anchored at the top of layer 0, the height of layer 0 not counting.
// Hand values on the three bands (0.05 and 0.2 mm layers are below the cap, 0.3 mm layers are capped), and a first layer of
// another height than the band above it.
TEST_CASE("Tree settings foot gain and target follow the real height of each layer", "[plugins][support-tree-settings]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const TF::TreeSettings settings = foot_profile_settings(SG::config("tree", false, 3));
    const size_t layer_count = settings.layer_print_z.size();
    REQUIRE(settings.bp_gain.size() == layer_count);
    REQUIRE(settings.foot_target.size() == layer_count);
    REQUIRE(settings.bp_radius == scale_i(3.75));
    REQUIRE(settings.branch_radius == scale_i(1.));
    const double tan40 = std::tan(0.7), half_line = 0.5 * double(settings.support_line_width);
    const double bp_radius = double(settings.bp_radius), branch_radius = double(settings.branch_radius);
    // The bands: layer 1 is thin, then the first 0.2 layer and the first 0.3 layer above it.
    const double first = layer_height_of(settings, 0);
    REQUIRE(first == double(scale_to_layer_coord(0.3)));
    REQUIRE(layer_height_of(settings, 1) == double(scale_to_layer_coord(0.05)));
    size_t mid = 1, thick = 1;
    while (mid < layer_count && layer_height_of(settings, mid) != double(scale_to_layer_coord(0.2)))
        ++mid;
    while (thick < layer_count && layer_height_of(settings, thick) != double(scale_to_layer_coord(0.3)))
        ++thick;
    REQUIRE(mid >= 6);
    REQUIRE(thick >= mid + 3);
    REQUIRE(thick + 2 < layer_count);
    // Step budget: below the cap in the thin and the 0.2 bands, at the cap (half a line width) in the 0.3 band; nothing for layer 0.
    REQUIRE(tan40 * scale_d(0.2) < half_line);
    REQUIRE(tan40 * scale_d(0.3) > half_line);
    CHECK(settings.bp_gain[0] == 0.);
    CHECK(settings.bp_gain[1] == Approx(tan40 * scale_d(0.05)).margin(1e-6));
    CHECK(settings.bp_gain[mid] == Approx(tan40 * scale_d(0.2)).margin(1e-6));
    CHECK(settings.bp_gain[thick] == Approx(half_line).margin(1e-6));
    // Targets: bp_radius on layer 0 whatever its height, then the budget of each step is removed going up.
    CHECK(settings.foot_target[0] == bp_radius);
    CHECK(settings.foot_target[1] == Approx(bp_radius - tan40 * scale_d(0.05)).margin(1e-6));
    CHECK(settings.foot_target[1] - settings.foot_target[5] == Approx(4. * tan40 * scale_d(0.05)).margin(1e-3));
    CHECK(settings.foot_target[mid - 1] - settings.foot_target[mid + 2] == Approx(3. * tan40 * scale_d(0.2)).margin(1e-3));
    // 20 thin layers (0.05) and 10 layers of 0.2 spend 20 * 0.0421 + 10 * 0.1685 = 2.527 mm of the 2.75 mm; the first 0.3 layer
    // spends 0.21 mm of the 0.223 mm left, and the next one cannot go below the branch radius.
    CHECK(thick == 31);
    CHECK(settings.foot_target[thick - 1] == Approx(branch_radius + scale_d(2.75) - 20. * tan40 * scale_d(0.05) - 10. * tan40 * scale_d(0.2)).margin(1.));
    CHECK(settings.foot_target[thick] == Approx(settings.foot_target[thick - 1] - half_line).margin(1e-3));
    CHECK(settings.foot_target[thick] > branch_radius);
    CHECK(settings.foot_target[thick + 1] == branch_radius);
    // Written independently: running remainder of the foot, never below the branch radius, never increasing going up.
    double remaining = bp_radius - branch_radius;
    for (size_t layer = 0; layer < layer_count; ++layer) {
        if (layer > 0)
            remaining -= std::min(tan40 * layer_height_of(settings, layer), half_line);
        remaining = std::max(remaining, 0.);
        CHECK(settings.foot_target[layer] == Approx(branch_radius + remaining).margin(1e-3));
        CHECK(settings.foot_target[layer] >= branch_radius);
        if (layer > 0)
            CHECK(settings.foot_target[layer] <= settings.foot_target[layer - 1]);
    }
}

// A branch as wide as the foot target or wider has no foot: the target is the branch radius on every layer.
TEST_CASE("Tree settings have no foot when the branch is as wide as the bed radius", "[plugins][support-tree-settings]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = SG::config("tree", false, 3);
    config.set_deserialize_strict({{"support_tree_branch_diameter", "8"}});
    const TF::TreeSettings settings = foot_profile_settings(config);
    REQUIRE(settings.branch_radius >= settings.bp_radius);
    REQUIRE(settings.foot_target.size() == settings.layer_print_z.size());
    for (const double target : settings.foot_target)
        CHECK(target == double(settings.branch_radius));
}

// The foot of one step: what the step budget bp_gain leaves after the normal growth, up to foot_target, never negative, and only for
// a branch that goes to the bed and has reached the branch radius. increase_single_area() reads this function.
TEST_CASE("Tree foot step adds what the step budget leaves, up to the target", "[plugins][support-tree-settings]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const TF::TreeSettings settings = foot_profile_settings(SG::config("tree", false, 3));
    const TF::LayerIndex upper = 3;
    REQUIRE(settings.foot_target[upper - 1] - double(settings.branch_radius) > scale_d(1.));
    const double tip_total = settings.tip_total_gain;
    const double branch_radius = double(settings.branch_radius);

    // A branch whose tip has ended: the cone grows by cone_gain, the rest of bp_gain goes to the foot.
    const TF::SupportElementState parent = foot_state(upper, 1, tip_total, 0.);
    const TF::SupportElementState grown = grown_state(settings, parent, upper);
    const double budget = settings.bp_gain[upper] - settings.cone_gain[upper];
    REQUIRE(budget > 0.);
    CHECK(TF::foot_step_gain(settings, parent, grown, upper) == Approx(budget).margin(2.));

    // A branch that does not go to the bed has no foot; neither one whose drawn radius did not reach the branch radius.
    TF::SupportElementState not_bed = grown;
    not_bed.to_buildplate = false;
    CHECK(TF::foot_step_gain(settings, parent, not_bed, upper) == 0.);
    const TF::SupportElementState young = foot_state(upper, 1, 0., 0.);
    CHECK(TF::foot_step_gain(settings, young, grown_state(settings, young, upper), upper) == 0.);

    // Near the target the foot only adds what is missing: 7 units here.
    const double near_target = settings.foot_target[upper - 1] - 7.;
    const TF::SupportElementState near_parent = foot_state(upper, 1, tip_total, near_target - branch_radius - settings.cone_gain[upper]);
    CHECK(TF::foot_step_gain(settings, near_parent, grown_state(settings, near_parent, upper), upper) == Approx(7.).margin(2.));

    // A branch already above the target adds nothing, never a negative amount.
    const TF::SupportElementState above = foot_state(upper, 1, tip_total, settings.foot_target[upper - 1] - branch_radius + 50.);
    CHECK(TF::foot_step_gain(settings, above, grown_state(settings, above, upper), upper) == 0.);

    // A cone faster than the step budget leaves nothing to the foot; a cone at three quarters of the budget leaves a quarter.
    TF::TreeSettings fast = settings;
    fast.cone_gain[upper] = 2. * fast.bp_gain[upper];
    CHECK(TF::foot_step_gain(fast, parent, grown_state(fast, parent, upper), upper) == 0.);
    fast.cone_gain[upper] = 0.75 * fast.bp_gain[upper];
    CHECK(TF::foot_step_gain(fast, parent, grown_state(fast, parent, upper), upper) == Approx(0.25 * fast.bp_gain[upper]).margin(2.));

    // The last step of the tip ends it: the drawn radius reaches the branch radius and the foot starts with the rest of the budget.
    // One step earlier the drawn radius is still below the branch radius and the foot does not grow.
    const TF::SupportElementState last_tip = foot_state(upper, 1, tip_total - 0.5 * settings.tip_gain[upper], 0.);
    const TF::SupportElementState last_grown = grown_state(settings, last_tip, upper);
    REQUIRE(last_grown.radius_gain == tip_total);
    CHECK(TF::foot_step_gain(settings, last_tip, last_grown, upper) == Approx(settings.bp_gain[upper] - 0.5 * settings.tip_gain[upper]).margin(2.));
    const TF::SupportElementState before_last = foot_state(upper, 1, tip_total - 1.5 * settings.tip_gain[upper], 0.);
    CHECK(TF::foot_step_gain(settings, before_last, grown_state(settings, before_last, upper), upper) == 0.);

    // Where the foot is over (target at the branch radius) nothing is added.
    const TF::LayerIndex over = TF::LayerIndex(settings.layer_print_z.size()) - 2;
    REQUIRE(settings.foot_target[over - 1] == branch_radius);
    const TF::SupportElementState high = foot_state(over, 1, tip_total, 0.);
    CHECK(TF::foot_step_gain(settings, high, grown_state(settings, high, over), over) == 0.);
}

// The catch-up gain: the largest gain whose radius stays within min(ceiled radius, drawn radius), the foot acquired so far removed,
// bounded by the path gain of one step less; it never decreases. A partial foot (0 < foot < target) is part of both radii.
TEST_CASE("Tree catch-up gain respects the radii, the foot and the path bound", "[plugins][support-tree-settings]")
{
    const coord_t min_radius = 400000;
    const double gain = 500000., partial_foot = 300000.;
    // Drawn radius 1300000 and ceiled radius 2600000 above it: reachable = 1300000 - 400000 - 300000 = 600000, below the path bound.
    CHECK(TF::catch_up_gain(gain, 2600000, 1300000, min_radius, partial_foot, 900000.) == 600000.);
    // The path bound is the limit when it is lower than the reachable gain, and never below the current gain.
    CHECK(TF::catch_up_gain(gain, 2600000, 1300000, min_radius, partial_foot, 550000.) == 550000.);
    CHECK(TF::catch_up_gain(gain, 2600000, 1300000, min_radius, partial_foot, 450000.) == gain);
    // A ceiled radius under the drawn one limits the gain (1000000 - 400000 - 300000 = 300000), but the gain does not decrease.
    CHECK(TF::catch_up_gain(250000., 1000000, 1300000, min_radius, partial_foot, 900000.) == 300000.);
    CHECK(TF::catch_up_gain(gain, 1000000, 1300000, min_radius, partial_foot, 900000.) == gain);
    // Without a foot the whole radius above the tip is gain.
    CHECK(TF::catch_up_gain(0., 2600000, 1000000, min_radius, 0., 900000.) == 600000.);
    // A fine cache step of the foot band (1.7 mm, 0.2 mm after 1.5 mm) under the drawn radius stops the catch-up on it: 1700000 - 400000 = 1300000.
    CHECK(TF::catch_up_gain(0., 1700000, 1800000, min_radius, 0., 1.e9) == 1300000.);
    // The collision radius of the result stays within both radii whenever the gain grows.
    for (const coord_t ceil_radius : {coord_t(900000), coord_t(1100000), coord_t(1500000)}) {
        const double caught = TF::catch_up_gain(0., ceil_radius, 1300000, min_radius, partial_foot, 1.e9);
        CHECK(double(min_radius) + caught + partial_foot <= double(std::min<coord_t>(ceil_radius, 1300000)));
    }
}

// A merge keeps the largest drawn radius of the two branches (the foot carries the difference), and the larger gain and distance
// to top. The collision radius of the merged branch can exceed the one of both branches when one of them paused its growth while
// its drawn radius followed the path: the routing refuses such a merge (merge_influence_areas_two_elements).
TEST_CASE("Tree merge keeps the largest drawn radius of the merged branches", "[plugins][support-tree-settings]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const TF::TreeSettings settings = foot_profile_settings(SG::config("tree", false, 3));
    const double tip_total = settings.tip_total_gain;
    const TF::LayerIndex layer = 5;
    const c_point position{ 0, 0 };
    const uint32_t long_dtt = 25;
    REQUIRE(size_t(layer) + long_dtt < settings.layer_print_z.size());
    // Gains from the tip to well above the cone, with and without a foot, with a long and a short path.
    const std::vector<TF::SupportElementState> states{
        foot_state(layer, 4, tip_total + 300000., 0.), foot_state(layer, 4, tip_total, 400000.), foot_state(layer, 25, 0.5 * tip_total, 0.),
        foot_state(layer, 25, 0., 300000.), foot_state(layer, 1, 0.5 * tip_total, 0.), foot_state(layer, 12, tip_total + 1000., 123456.)};
    for (const TF::SupportElementState &first : states)
        for (const TF::SupportElementState &second : states) {
            const TF::SupportElementState merged = TF::merge_support_element_states(first, second, position, layer, settings);
            CAPTURE(first.radius_gain, first.foot_gain, first.distance_to_top, second.radius_gain, second.foot_gain, second.distance_to_top);
            CHECK(merged.radius_gain == std::max(first.radius_gain, second.radius_gain));
            CHECK(merged.distance_to_top == std::max(first.distance_to_top, second.distance_to_top));
            CHECK(merged.foot_gain >= 0.);
            CHECK(TF::support_element_radius(settings, merged) >= std::max(TF::support_element_radius(settings, first), TF::support_element_radius(settings, second)));
        }
    // Both branches within one cache step: without foot to carry, the merged foot is zero.
    const TF::SupportElementState plain = TF::merge_support_element_states(states[0], states[0], position, layer, settings);
    CHECK(plain.foot_gain == 0.);
    CHECK(TF::support_element_collision_radius(settings, plain) == TF::support_element_collision_radius(settings, states[0]));
    // A branch that paused (gain half the tip, a long path) with a foot, merged with a branch of a larger gain whose drawn radius
    // is the same: the merged collision radius is larger than both collision radii.
    const TF::SupportElementState paused = foot_state(layer, 25, 0.5 * tip_total, 0.);
    const TF::SupportElementState footed = foot_state(layer, 25, 0., 300000.);
    const TF::SupportElementState both = TF::merge_support_element_states(paused, footed, position, layer, settings);
    CHECK(TF::support_element_collision_radius(settings, both) > std::max(TF::support_element_collision_radius(settings, paused), TF::support_element_collision_radius(settings, footed)));
}

// A planned foot widening that the caches refuse is cut at a limit of a cache step: the current step is always filled (same key,
// no validation), then the limit of each following step that the widening crosses is tried, and the largest valid one is kept. The
// cache steps here are a synthetic grid (0.25 mm, then 0.1 mm, from 1 mm; the limit of a step is the step itself), and the radius
// that the validity function receives is the radius of the widening that would be added.
TEST_CASE("Tree partial foot fills the cache step and the valid steps it crosses", "[plugins][support-tree-settings]")
{
    const coord_t first = 1000000;
    const auto grid = [&](coord_t step) {
        return [=](coord_t radius) { return radius <= first ? first : first + ((radius - first + step - 1) / step) * step; };
    };
    const auto next_ceil = grid(250000), fine_ceil = grid(100000);
    const auto valid_until = [&](coord_t limit) { return [=](coord_t radius) { return radius <= limit; }; };
    const auto gain = [](double base, double planned, const auto &ceil, const auto &valid) {
        return TF::partial_foot_gain(base, planned, [=](double increase) { return coord_t(base + increase); }, ceil, valid);
    };
    // A half line width of 0.6 mm: the largest foot widening of one step.
    const double planned = 300000.;

    // The full widening is valid: it is added whole.
    CHECK(gain(1300000., planned, next_ceil, valid_until(1750000)) == planned);
    // The full radius stays in the current cache step: valid, so the widening is added whole.
    CHECK(gain(1260000., 200000., next_ceil, valid_until(1500000)) == 200000.);
    // One step crossed: 1.3 mm + 0.5 mm reaches the step 2 mm, refused; the limit 1.75 mm is crossed and valid.
    CHECK(gain(1300000., 500000., next_ceil, valid_until(1750000)) == Approx(450000.));
    // The crossed limit is refused too: the foot only fills the current step (1.5 mm).
    CHECK(gain(1300000., 500000., next_ceil, valid_until(1500000)) == Approx(200000.));
    // Nothing is valid above the current step and the radius is already at its limit: no widening.
    CHECK(gain(1500000., 300000., next_ceil, valid_until(1500000)) == 0.);

    // Several steps crossed (steps of 0.1 mm): 1.05 mm + 0.3 mm = 1.35 mm, step limit 1.4 mm; crossed limits 1.2 and 1.3 mm.
    CHECK(gain(1050000., planned, fine_ceil, valid_until(1300000)) == Approx(250000.));
    CHECK(gain(1050000., planned, fine_ceil, valid_until(1200000)) == Approx(150000.));
    CHECK(gain(1050000., planned, fine_ceil, valid_until(1100000)) == Approx(50000.));
    // The largest valid limit is kept even when a lower one is refused.
    CHECK(gain(1050000., planned, fine_ceil, [](coord_t radius) { return radius <= 1100000 || radius == 1300000; }) == Approx(250000.));

    // The validity function only sees the radii that the widening gives: with a radius that is not base + increase (a collision
    // radius that the caller computes differently), the radii checked are the ones of radius_of.
    std::vector<coord_t> checked;
    static_cast<void>(TF::partial_foot_gain(1300000., 500000., [](double increase) { return coord_t(1300000. + increase) + 3; }, next_ceil,
        [&](coord_t radius) { checked.push_back(radius); return false; }));
    CHECK(checked == std::vector<coord_t>{ 1800003, 1750003 });
}

// The step of the cache radii in the foot band is half the diameter of the largest nozzle of the support: the extruder of the
// support material or the one of the support interface.
TEST_CASE("Tree settings cache step is half the largest support nozzle", "[plugins][support-tree-settings]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const auto step_of = [](const auto &override_config) {
        DynamicPrintConfig config = SG::config("tree", false, 3);
        override_config(config);
        Print print;
        Model model;
        Test::init_print({make_cube(2., 2., 6.)}, print, model, config);
        SG::prefix(print);
        return ST::tree_settings(print, print.object(0), SG::params(print, print.object(0))).foot_cache_step;
    };
    using Config = DynamicPrintConfig;
    CHECK(step_of([](Config &) {}) == scale_i(0.2));
    CHECK(step_of([](Config &c) { c.set_deserialize_strict({{"nozzle_diameter", "0.8"}}); }) == scale_i(0.4));
    // Two extruders: the larger nozzle of the two decides, whichever of them prints the interface.
    CHECK(step_of([](Config &c) { c.set_deserialize_strict({{"nozzle_diameter", "0.4,0.8"}, {"support_material_extruder", "1"}, {"support_material_interface_extruder", "2"}}); }) == scale_i(0.4));
    CHECK(step_of([](Config &c) { c.set_deserialize_strict({{"nozzle_diameter", "0.4,0.8"}, {"support_material_extruder", "2"}, {"support_material_interface_extruder", "1"}}); }) == scale_i(0.4));
    CHECK(step_of([](Config &c) { c.set_deserialize_strict({{"nozzle_diameter", "0.4,0.8"}, {"support_material_extruder", "1"}, {"support_material_interface_extruder", "1"}}); }) == scale_i(0.2));
}
