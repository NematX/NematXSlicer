#include "support_grid_test_helpers.hpp"
#include "support_tree_test_helpers.hpp"
#include "libslic3r/Plugins/Support/Common/SupportZPlacement.hpp"
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

   Only fields that exist on both sides are compared with the native settings. Three things differ on purpose, each checked explicitly:
   - tip_layers: the plugin never gives less than one tip layer (max(1, ...) at TreeSettings.cpp), where the native settings assert
     that it is positive. A configuration that makes the native value zero is therefore only checked on the plugin side.
   - maximum moves: the native settings hold one move for the nominal layer height; the plugin holds one per real layer height. They
     are compared on the layers whose height is the nominal one, and every layer is compared with tan(angle) * its own height.
   - the branch angle: the plugin caps it by support_tree_max_overhang; the cases set it high enough for the cap to be inactive.
   The slab heights (min, max and dense) exist only in the plugin: they are compared with the rule written from the configuration. */

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

// The slab height rule written from the configuration: 0 is the maximum, a percent is of the nozzle, the maximum caps it.
double expected_slab_mm(const std::string &setting)
{
    const bool percent = !setting.empty() && setting.back() == '%';
    const double value = std::stod(setting);
    if (value == 0.)
        return max_layer_mm;
    return std::min(max_layer_mm, percent ? nozzle_mm * value / 100. : value);
}

// Builds the print of a case, then compares the plugin settings with the native ones and with the slab rule.
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
        CHECK(settings.layer_height == native.layer_height);
        CHECK(settings.resolution == native.resolution);
        CHECK(settings.min_feature_size == native.min_feature_size);
        CHECK(settings.branch_radius == native.branch_radius);
        CHECK(settings.min_radius == native.min_radius);
        // The plugin keeps at least one tip layer; the native value is positive here, so they are equal.
        CHECK(settings.tip_layers == std::max<size_t>(1, native.tip_layers));
        CHECK(settings.branch_radius_increase_per_layer == native.branch_radius_increase_per_layer);
        CHECK(settings.bp_radius == native.bp_radius);
        CHECK(settings.bp_radius_increase_per_layer == native.bp_radius_increase_per_layer);
        CHECK(settings.layer_start_bp_radius == native.layer_start_bp_radius);
        CHECK(settings.increase_radius_until_radius == native.increase_radius_until_radius);
        CHECK(settings.increase_radius_until_layer == native.increase_radius_until_layer);
        CHECK(settings.min_dtt_to_model == native.min_dtt_to_model);
        CHECK(settings.max_to_model_radius_increase == native.max_to_model_radius_increase);
        CHECK(settings.support_rests_on_model == native.support_rests_on_model);
        CHECK(settings.xy_distance == native.xy_distance);
        CHECK(settings.xy_min_distance == native.xy_min_distance);
        CHECK(settings.z_distance_top_layers == native.z_distance_top_layers);
        CHECK(settings.z_distance_bottom_layers == native.z_distance_bottom_layers);
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
        CHECK((nominal > 0) == (settings.layer_height <= scale_to_layer_coord(max_layer_mm)));
    } else {
        // The native settings cannot be built: only the floor of the tip layers is checked.
        CHECK(settings.tip_layers == 1);
    }

    // Every layer moves by tan(angle) times its own height, fast and slow.
    REQUIRE(settings.max_move.size() == object.layer_count());
    size_t other_heights = 0;
    for (size_t layer = 0; layer < object.layer_count(); ++layer) {
        const coord_t height = object.layer(layer).scaled_height();
        CHECK(settings.max_move[layer] == coord_t(std::tan(settings.angle) * double(height)));
        other_heights += height != settings.layer_height;
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
