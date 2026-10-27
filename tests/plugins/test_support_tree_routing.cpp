#include "support_grid_test_helpers.hpp"
#include "support_tree_test_helpers.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Plugins/Support/Tree/TreeRouting.hpp"
#include "libslic3r/Plugins/Support/Tree/TreeSettings.hpp"
#include "libslic3r/Plugins/Support/Tree/TreeVolumes.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <tbb/global_control.h>
#include <tbb/task_arena.h>

#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"

/* Tree-family routing tests. The native routing copy is checked on properties of its routed branches, not on the legacy
   output: the tips sit under the top interface stack, every branch reaches the build plate or rests on the object, no node
   comes closer to the object than the XY distance, and each branch move stays within the maximum move of its layers.
   The tests run the production slicing prefix, so the island demand is the one the family receives. */

using namespace Slic3r;
namespace SG = Slic3r::Test::SupportGrid;
namespace TF = Slic3r::TreeFamily;

namespace {
// Tolerance for the properties checked on routed nodes, in millimeters.
constexpr double check_tolerance_mm = 0.05;

using Routed = Slic3r::Test::SupportTree::Routed;
using Slic3r::Test::SupportTree::to_point;

// Distance in millimeters from a point to the closed outlines of the polygons.
double distance_to_outline_mm(const Point &point, const Polygons &polygons)
{
    double best = std::numeric_limits<double>::max();
    for (const Polygon &polygon : polygons)
        for (size_t idx = 0; idx < polygon.size(); ++ idx) {
            const Vec2d a = polygon.points[idx].cast<double>();
            const Vec2d ab = polygon.points[(idx + 1) % polygon.size()].cast<double>() - a;
            const Vec2d ap = point.cast<double>() - a;
            const double len2 = ab.squaredNorm();
            const double t = len2 > 0. ? std::clamp(ap.dot(ab) / len2, 0., 1.) : 0.;
            best = std::min(best, (ap - ab * t).norm());
        }
    return unscaled(best);
}

// The object outlines of one layer, as they are sliced.
Polygons layer_outline(const PrintObject &object, size_t layer)
{
    return to_polygons(object.layer(layer).lslices());
}


// A low platform with a roof on a post beside it: the roof overhangs past the platform's far side, so its branches must go
// around the platform to reach the bed. The branch centres cross the thin layers of the profile on the way down.
TriangleMesh narrow_platform_overhang()
{
    TriangleMesh platform = make_cube(10., 10., 3.);
    TriangleMesh post = make_cube(2., 10., 3.);
    post.translate(7., 0., 3.);
    TriangleMesh roof = make_cube(10., 10., 1.);
    roof.translate(7., 0., 6.);
    platform.merge(post);
    platform.merge(roof);
    return platform;
}

// A wide platform, and a roof on a post at its far side, 2.5 mm above the platform: the tips under the roof above the platform
// may rest on it, which build plate only forbids, and the tips past the platform edge reach the bed.
TriangleMesh wide_platform_overhang()
{
    TriangleMesh platform = make_cube(20., 10., 4.);
    TriangleMesh post = make_cube(2., 10., 2.5);
    post.translate(15., 0., 4.);
    TriangleMesh roof = make_cube(13., 10., 1.);
    roof.translate(12., 0., 6.5);
    platform.merge(post);
    platform.merge(roof);
    return platform;
}
}

// The overhang of the slab starts at the roof bottom (z = 4 mm), so the top contact lies at 3.8 mm. Three dense interface
// slabs of the maximum layer height (0.3 mm) stack under it, so the stack bottom is at 2.9 mm and the tips start on the layer at
// 2.8 mm, which must reach the build plate.
TEST_CASE("Tree tips sit under the interface stack and reach the build plate", "[plugins][support-tree-routing]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    Test::init_print({SG::slab()}, print, model, SG::config("organic", false, 3));
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    const Routed routed(print);
    REQUIRE_FALSE(routed.trees.empty());
    const size_t tip_layer = 13;
    REQUIRE(unscaled(object.layer(tip_layer).scaled_print_z()) == Approx(2.8));
    for (const TF::RoutedTree &tree : routed.trees) {
        REQUIRE_FALSE(tree.nodes.empty());
        CHECK(tree.nodes.front().layer == tip_layer);
        CHECK(tree.nodes.front().print_z == Approx(2.8));
        const TF::TreeNode &bottom = tree.nodes.back();
        CHECK(bottom.layer == 0);
        CHECK(bottom.to_buildplate);
    }
}

// Every routed node must keep the XY distance from the object outlines of its layer, minus a tolerance. The distance is
// measured to the layer that the node is printed on, which is where the legacy collisions are computed.
TEST_CASE("Tree nodes keep their XY distance to the object", "[plugins][support-tree-routing]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool platform = GENERATE(false, true);
    Print print;
    Model model;
    Test::init_print({platform ? SG::slab(true) : SG::slab()}, print, model, SG::config("organic"));
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    const TF::TreeSettings settings = Slic3r::Test::SupportTree::tree_settings(print, object, SG::params(print, object));
    const double xy_distance_mm = unscaled(double(settings.xy_distance));
    const Routed routed(print);
    REQUIRE_FALSE(routed.trees.empty());
    double closest_mm = std::numeric_limits<double>::max();
    for (const TF::RoutedTree &tree : routed.trees)
        for (const TF::TreeNode &node : tree.nodes) {
            const Polygons outline = layer_outline(object, node.layer);
            CAPTURE(node.layer, node.print_z, unscaled(node.position.x), unscaled(node.position.y));
            const double distance_mm = distance_to_outline_mm(to_point(node.position), outline);
            closest_mm = std::min(closest_mm, distance_mm);
            CHECK_FALSE(contains(outline, to_point(node.position)));
            CHECK(distance_mm >= xy_distance_mm - check_tolerance_mm);
        }
    CAPTURE(xy_distance_mm, closest_mm);
    CHECK(closest_mm >= xy_distance_mm - check_tolerance_mm);
}

// A branch centre moves between two consecutive layers by at most tan(angle) times the real height of the upper layer, plus the
// radius growth that the routing grants on the way down (the legacy extra speed for radius growth, which widens branch feet).
// The layers have variable heights, so the bound must follow each layer, not one nominal height.
TEST_CASE("Tree branch moves follow the real height of each layer", "[plugins][support-tree-routing]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = SG::config("organic");
    config.set_deserialize_strict({{"support_material_buildplate_only", "1"}});
    Print print;
    Model model;
    SG::init_print_with_profile(print, model, narrow_platform_overhang(), std::vector<coordf_t>{0., 0.2, 1., 0.05, 2., 0.2, 2.4, 0.05, 2.6, 0.3, 7., 0.3}, config);
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    const TF::TreeSettings settings = Slic3r::Test::SupportTree::tree_settings(print, object, SG::params(print, object));
    const Routed routed(print);
    REQUIRE_FALSE(routed.trees.empty());
    double largest_ratio = 0.;
    size_t checked = 0, thin_moves = 0;
    for (const TF::RoutedTree &tree : routed.trees)
        for (const TF::TreeNode &lower : tree.nodes)
            for (size_t upper_idx : lower.carries) {
                const TF::TreeNode &upper = tree.nodes[upper_idx];
                REQUIRE(upper.layer == lower.layer + 1);
                const double height_mm = object.layer(upper.layer).unscaled_height();
                const double allowed_mm = std::tan(settings.angle) * height_mm + unscaled(double(std::max<coord_t>(0, lower.radius - upper.radius)));
                const double move_mm = unscaled((to_point(lower.position) - to_point(upper.position)).cast<double>().norm());
                CAPTURE(upper.layer, height_mm, move_mm, allowed_mm);
                CHECK(move_mm <= allowed_mm + check_tolerance_mm);
                largest_ratio = std::max(largest_ratio, move_mm / (std::tan(settings.angle) * height_mm));
                thin_moves += height_mm < 0.1 && move_mm > 0.;
                ++ checked;
            }
    CAPTURE(checked, largest_ratio, thin_moves);
    REQUIRE(checked > 0);
    // The thin layers must lie on the branch paths, and the branches must move across them.
    CHECK(thin_moves > 0);
    // The sliced heights only approximate the profile, so the thinnest and the thickest layers are compared.
    size_t thin = 0, thick = 0;
    for (size_t layer = 0; layer < object.layer_count(); ++ layer) {
        if (object.layer(layer).unscaled_height() < object.layer(thin).unscaled_height())
            thin = layer;
        if (object.layer(layer).unscaled_height() > object.layer(thick).unscaled_height())
            thick = layer;
    }
    REQUIRE(object.layer(thick).unscaled_height() > 2. * object.layer(thin).unscaled_height());
    CHECK(settings.maximum_move_distance(TF::LayerIndex(thin)) < settings.maximum_move_distance(TF::LayerIndex(thick)) / 2);
}

// Between two consecutive routed nodes of a branch above the foot, whose tip has ended, the radius grows by tan(diameter angle) times
// the real height between the two layers. The profile has a thin band (0.05) and thick ones (0.3) on the branch paths, above the
// layers where the foot widens the branches (foot_target above the branch radius). Forks and merges are skipped: their radius is the larger one of the
// merged branches. A law counted in layers of the nominal height would grow by the same amount in every layer.
TEST_CASE("Tree branch radius grows with the real height between layers", "[plugins][support-tree-routing]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = SG::config("organic");
    config.set_deserialize_strict({{"support_material_buildplate_only", "1"}});
    TriangleMesh post = make_cube(2., 10., 10.);
    TriangleMesh roof = make_cube(8., 10., 1.);
    roof.translate(0., 0., 10.);
    post.merge(roof);
    Print print;
    Model model;
    SG::init_print_with_profile(print, model, std::move(post), std::vector<coordf_t>{0., 0.3, 7., 0.3, 7.01, 0.05, 8.5, 0.05, 8.51, 0.3, 11., 0.3}, config);
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    const TF::TreeSettings settings = Slic3r::Test::SupportTree::tree_settings(print, object, SG::params(print, object));
    const double slope = std::tan(config.opt_float("support_tree_branch_diameter_angle") * M_PI / 180.);
    // The first layer above the foot: where the foot target is back to the branch radius.
    TF::LayerIndex foot_end = 0;
    while (size_t(foot_end) < settings.foot_target.size() && settings.foot_target[foot_end] > double(settings.branch_radius))
        ++ foot_end;
    const Routed routed(print);
    REQUIRE_FALSE(routed.trees.empty());
    size_t thin_pairs = 0, thick_pairs = 0;
    for (const TF::RoutedTree &tree : routed.trees)
        for (const TF::TreeNode &lower : tree.nodes) {
            // A plain continuation: one node above, on the next layer, above the foot, and a radius that grows.
            if (lower.carries.size() != 1 || TF::LayerIndex(lower.layer) < foot_end + 2)
                continue;
            const TF::TreeNode &upper = tree.nodes[lower.carries.front()];
            if (upper.layer != lower.layer + 1 || upper.radius < settings.branch_radius || lower.radius <= upper.radius)
                continue;
            const double height_mm = object.layer(upper.layer).unscaled_height();
            const double growth = double(lower.radius - upper.radius) / (upper.print_z - lower.print_z) / 1e6;
            CHECK_FALSE((lower.verylost || upper.verylost));
            CAPTURE(lower.layer, height_mm, lower.radius, upper.radius, growth, slope);
            CHECK(growth == Approx(slope).margin(0.002));
            thin_pairs += height_mm < 0.1;
            thick_pairs += height_mm > 0.25;
        }
    CAPTURE(thin_pairs, thick_pairs);
    CHECK(thin_pairs >= 5);
    CHECK(thick_pairs >= 5);
}

// A roof overhangs a wide platform, and its post stands on the platform edge, so the only landing for the branches is the platform
// top, a gap below the roof. With thin 0.05 layers around the gap, even a short branch spans many layers: the layer count
// alone (ceil(1 mm / 0.2) = 5) would let a branch of 0.7 mm rest on the platform, but the minimum height to the model is 1 mm of
// real Z, so it must not. A branch of 1.3 mm (control) rests on the platform.
TEST_CASE("The minimum height to the model follows the real Z of thin layers", "[plugins][support-tree-routing]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const double gap_mm = GENERATE(0.9, 1.5);
    CAPTURE(gap_mm);
    DynamicPrintConfig config = SG::config("organic", false, 0);
    config.set_deserialize_strict({{"support_material_buildplate_only", "0"}});
    TriangleMesh platform = make_cube(20., 10., 4.);
    TriangleMesh post = make_cube(2., 10., gap_mm);
    post.translate(18., 0., 4.);
    TriangleMesh roof = make_cube(12., 10., 1.);
    roof.translate(8., 0., 4. + float(gap_mm));
    platform.merge(post);
    platform.merge(roof);
    const double top = 5. + gap_mm;
    Print print;
    Model model;
    SG::init_print_with_profile(print, model, std::move(platform), std::vector<coordf_t>{0., 0.2, 3.4, 0.2, 3.5, 0.05, top - 0.5, 0.05, top - 0.4, 0.2, top, 0.2}, config);
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    const Routed routed(print);
    // The top layer of the platform: the last layer whose slice plane (the middle of the layer) cuts the platform.
    constexpr double platform_height_mm = 4.;
    size_t platform_layer = 0;
    for (size_t layer = 0; layer < object.layer_count(); ++ layer)
        if (object.layer(layer).unscaled_print_z() - 0.5 * object.layer(layer).unscaled_height() < platform_height_mm)
            platform_layer = layer;
    const double platform_top_mm = object.layer(platform_layer).unscaled_print_z();
    const double eps_mm = unscaled(double(scale_to_layer_coord(EPSILON)));
    // The tips are one contact distance (0.2 mm) under the roof, so the branches span gap - 0.2 mm down to the platform: 0.7 mm
    // (gap 0.9) or 1.3 mm (gap 1.5).
    // The elements that cannot rest on the model are dropped: the short case routes no branch, as none can reach the bed.
    size_t on_model = 0;
    for (const TF::RoutedTree &tree : routed.trees) {
        const TF::TreeNode &bottom = tree.nodes.back();
        // A root rests on the platform: its last node lies above the platform top and at most the bottom gap above it. The routed root
        // may stay inside that gap, under the floor: clear_under_floors() removes the floor footprint from the slab polygons, not the
        // routed nodes, so the node may lie up to platform_top + gap_below and the branch then rests on its floor.
        CAPTURE(bottom.print_z, platform_top_mm, routed.params.gap_below);
        CHECK(bottom.print_z > platform_top_mm);
        CHECK(bottom.print_z <= platform_top_mm + routed.params.gap_below + eps_mm);
        on_model += ! bottom.to_buildplate;
    }
    CAPTURE(routed.trees.size(), on_model);
    if (gap_mm < 1.) {
        CHECK(routed.trees.empty());
        CHECK(on_model == 0);
    } else {
        REQUIRE_FALSE(routed.trees.empty());
        CHECK(on_model == routed.trees.size());
    }
}
// A small support_tree_max_overhang caps the branch angle at atan(max overhang / minimum layer height), where the minimum
// layer height is the thinnest of the configured minimums and of the sliced layers. With the branch angle at 80 degrees, an
// overhang of 0.05 mm must set the angle to that cap.
TEST_CASE("A small max branch overhang lowers the branch angle", "[plugins][support-tree-routing]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = SG::config("organic");
    config.set_deserialize_strict({{"support_tree_max_overhang", "0.05"}, {"support_tree_angle", "80"}});
    Print print;
    Model model;
    Test::init_print({SG::slab(true)}, print, model, config);
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    const TF::TreeSettings settings = Slic3r::Test::SupportTree::tree_settings(print, object, SG::params(print, object));
    const double min_height_mm = SG::params(print, object).support_min_height;
    CAPTURE(min_height_mm, settings.angle);
    CHECK(settings.angle == Approx(std::atan2(0.05, min_height_mm)).epsilon(1e-6));
    CHECK(settings.angle < 80. * M_PI / 180.);
    for (size_t layer = 0; layer < object.layer_count(); ++ layer) {
        const double height_mm = object.layer(layer).unscaled_height();
        CHECK(unscaled(double(settings.maximum_move_distance(TF::LayerIndex(layer)))) <= std::tan(settings.angle) * height_mm + 1e-4);
    }
    const Routed routed(print);
    REQUIRE_FALSE(routed.trees.empty());
    for (const TF::RoutedTree &tree : routed.trees)
        for (const TF::TreeNode &lower : tree.nodes)
            for (size_t upper_idx : lower.carries) {
                const TF::TreeNode &upper = tree.nodes[upper_idx];
                const double height_mm = object.layer(upper.layer).unscaled_height();
                const double move_mm = unscaled((to_point(lower.position) - to_point(upper.position)).cast<double>().norm());
                CHECK(move_mm <= std::tan(settings.angle) * height_mm + unscaled(double(std::max<coord_t>(0, lower.radius - upper.radius))) + check_tolerance_mm);
            }
}

// With build plate only, the branches that support the overhang beside the platform must not rest on the platform: they go
// down diagonally, around it, and reach the build plate. Without the restriction, some branch rests on the platform.
TEST_CASE("Build plate only branches reach the bed around the object", "[plugins][support-tree-routing]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool buildplate_only = GENERATE(false, true);
    DynamicPrintConfig config = SG::config("organic");
    config.set_deserialize_strict({{"support_material_buildplate_only", buildplate_only ? "1" : "0"}});
    CAPTURE(buildplate_only);
    Print print;
    Model model;
    Test::init_print({wide_platform_overhang()}, print, model, config);
    SG::prefix(print);
    const Routed routed(print);
    REQUIRE_FALSE(routed.trees.empty());
    if (buildplate_only) {
        bool diagonal = false;
        for (const TF::RoutedTree &tree : routed.trees) {
            const TF::TreeNode &bottom = tree.nodes.back();
            CHECK(bottom.layer == 0);
            CHECK(bottom.to_buildplate);
            coord_t min_x = std::numeric_limits<coord_t>::max(), max_x = std::numeric_limits<coord_t>::min();
            for (const TF::TreeNode &node : tree.nodes) {
                min_x = std::min(min_x, node.position.x);
                max_x = std::max(max_x, node.position.x);
            }
            diagonal = diagonal || unscaled(double(max_x - min_x)) >= 1.;
        }
        CHECK(diagonal);
    } else {
        bool rests_on_model = false;
        for (const TF::RoutedTree &tree : routed.trees)
            rests_on_model = rests_on_model || ! tree.nodes.back().to_buildplate;
        CHECK(rests_on_model);
    }
}

// A zero XY spacing or a zero branch overhang gives zero offsets of the collision and move polygons, which ClipperUtils rejects.
// The routing must still run, and every branch reaches the bed.
TEST_CASE("Tree routing accepts zero XY spacing and zero branch overhang", "[plugins][support-tree-routing]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool zero_xy = GENERATE(true, false);
    DynamicPrintConfig config = SG::config("organic");
    config.set_deserialize_strict({{zero_xy ? "support_material_xy_spacing" : "support_tree_max_overhang", "0"}});
    Print print;
    Model model;
    Test::init_print({SG::slab()}, print, model, config);
    SG::prefix(print);
    const Routed routed(print);
    REQUIRE_FALSE(routed.trees.empty());
    for (const TF::RoutedTree &tree : routed.trees)
        CHECK(tree.nodes.back().layer == 0);
}

// The branch angle cap follows the minimum layer height of the support extruder, not the thinnest layer of the other extruders.
TEST_CASE("The branch angle cap follows the minimum layer height of the support extruder", "[plugins][support-tree-routing]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool thin_support = GENERATE(false, true);
    DynamicPrintConfig config = SG::config("organic");
    config.set_deserialize_strict({{"min_layer_height", thin_support ? "0.05,0.2" : "0.2,0.05"}, {"support_tree_max_overhang", "0.05"},
        {"support_tree_angle", "85"}, {"support_tree_angle_slow", "80"}});
    Print print;
    Model model;
    Test::init_print({SG::slab(true)}, print, model, config);
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    const TF::TreeSettings settings = Slic3r::Test::SupportTree::tree_settings(print, object, SG::params(print, object));
    const double support_min_mm = thin_support ? 0.05 : 0.2;
    const double cap = std::atan2(0.05, support_min_mm);
    CAPTURE(thin_support, settings.angle);
    CHECK(SG::params(print, object).support_min_height == Approx(support_min_mm).epsilon(1e-6));
    CHECK(settings.angle == Approx(cap).epsilon(1e-6));
    for (size_t layer = 0; layer < object.layer_count(); ++ layer) {
        const double height_mm = object.layer(layer).unscaled_height();
        CHECK(unscaled(double(settings.maximum_move_distance_slow(TF::LayerIndex(layer)))) == Approx(std::tan(cap) * height_mm).margin(1e-4));
        CHECK(unscaled(double(settings.maximum_move_distance(TF::LayerIndex(layer)))) == Approx(std::tan(cap) * height_mm).margin(1e-4));
    }
}

// A soluble support touches the object, so with zero XY spacing the safe offset must still step by a positive distance, and the
// tips reach the build plate through several layers.
TEST_CASE("Soluble tree routing with zero XY spacing propagates the tips", "[plugins][support-tree-routing]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = SG::config("organic", true, 3);
    config.set_deserialize_strict({{"support_material_xy_spacing", "0"}});
    Print print;
    Model model;
    Test::init_print({SG::slab()}, print, model, config);
    SG::prefix(print);
    const Routed routed(print);
    REQUIRE_FALSE(routed.trees.empty());
    size_t propagated = 0;
    for (const TF::RoutedTree &tree : routed.trees) {
        CHECK(tree.nodes.back().layer == 0);
        propagated += tree.nodes.size() > 1 ? 1 : 0;
    }
    CHECK(propagated > 0);
}

// A thin layer with a bridge contact keeps its normal contact apart from the bridge after the merge, and the layer above it is
// merged into that normal contact. The shelf of that layer is supported by the tips of the merged contact only.
TEST_CASE("Tips keep the demand that a merged top contact absorbs next to a bridge contact", "[plugins][support-tree-routing]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = SG::config("organic");
    config.set_deserialize_strict({{"support_material_contact_distance_type", "filament"}, {"bridge_type", "nozzle"},
        {"bridge_flow_ratio", "25%"}});
    TriangleMesh post = make_cube(2., 8., 4.);
    TriangleMesh roof = make_cube(6., 8., 0.05);
    roof.translate(0.f, 0.f, 4.f);
    TriangleMesh shelf = make_cube(2., 8., 0.5);
    shelf.translate(6.f, 0.f, 4.05f);
    post.merge(roof);
    post.merge(shelf);
    Print print;
    Model model;
    SG::init_print_with_profile(print, model, std::move(post), std::vector<coordf_t>{0., 0.2, 3.999, 0.2, 4.0, 0.05, 4.05, 0.05, 4.051, 0.2, 4.55, 0.2}, config);
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    // The thin layer is the one that the profile dips to; the layer above it keeps the bridge height, so it has no bridge contact.
    size_t thin_layer = object.layer_count();
    for (size_t layer = 0; layer < object.layer_count(); ++ layer)
        if (std::abs(object.layer(layer).unscaled_height() - 0.05) < 1e-4)
            thin_layer = layer;
    REQUIRE(thin_layer + 1 < object.layer_count());
    const size_t shelf_layer = thin_layer + 1;
    REQUIRE(object.layer(shelf_layer).unscaled_height() >= 0.2 - 1e-4);
    ExPolygons shelf_demand;
    for (const LayerSliceIsland &island : object.layer(shelf_layer).islands())
        append(shelf_demand, island.support_demand());
    const Polygons shelf_area = to_polygons(union_ex(shelf_demand));
    const Routed routed(print);
    REQUIRE_FALSE(routed.trees.empty());
    bool tips_on_shelf = false;
    for (const TF::RoutedTree &tree : routed.trees)
        tips_on_shelf = tips_on_shelf || contains(shelf_area, to_point(tree.nodes.front().position));
    CHECK(tips_on_shelf);
}

// The tree keeps the XY spacing of the user everywhere, tips included: with a spacing well above half the external perimeter width
// (1 mm), the circle of every routed node, tips and branches on any layer, stays at least that spacing from the object slices of its
// layer. The routing has no reduced distance next to the object.
TEST_CASE("Tips and branches keep the full XY distance next to the object", "[plugins][support-tree-routing]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = SG::config("organic");
    config.set_deserialize_strict({{"support_material_xy_spacing", "1"}});
    TriangleMesh lip = make_cube(4., 8., 4.);
    // The roof overhangs the post by 3 mm: a smaller overhang leaves no room for a branch at 1 mm from the post.
    TriangleMesh roof = make_cube(7., 8., 1.);
    roof.translate(0.f, 0.f, 4.f);
    lip.merge(roof);
    Print print;
    Model model;
    Test::init_print({lip}, print, model, config);
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    const TF::TreeSettings settings = Slic3r::Test::SupportTree::tree_settings(print, object, SG::params(print, object));
    const double xy_mm = unscaled(double(settings.xy_distance));
    CAPTURE(xy_mm);
    REQUIRE(xy_mm > 0.5);
    const Routed routed(print);
    REQUIRE_FALSE(routed.trees.empty());
    size_t checked = 0, near_object = 0;
    for (const TF::RoutedTree &tree : routed.trees)
        for (const TF::TreeNode &node : tree.nodes) {
            const Polygons outline = layer_outline(object, node.layer);
            const double distance_mm = distance_to_outline_mm(to_point(node.position), outline);
            const double radius_mm = unscaled(double(node.radius));
            CAPTURE(node.layer, unscaled(node.position.x), unscaled(node.position.y), distance_mm, radius_mm);
            // The distance to the outline is unsigned: a node centre inside the object must not pass for a node far from it.
            CHECK_FALSE(contains(outline, to_point(node.position)));
            CHECK(distance_mm - radius_mm >= xy_mm - check_tolerance_mm);
            // The distance is reached, not only exceeded: some node lies next to the clearance.
            near_object += distance_mm - radius_mm <= xy_mm + 0.1;
            ++checked;
        }
    REQUIRE(checked > 0);
    CHECK(near_object > 0);
}

// The tips are the points of the lines of the base pattern of the support, generated by the pattern plugin through the callbacks of the
// support step: supportbase for a sparse base without sheath, rectilinear for a sheathed one (the default of the tests).
// The slab of the default case has the 6 tips of the reference dump (support-tree-dump, slab-tree).
TEST_CASE("Tips are sampled with the rectilinear and the supportbase patterns", "[plugins][support-tree-routing]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool sparse = GENERATE(false, true);
    CAPTURE(sparse);
    DynamicPrintConfig config = SG::config("tree", false, 3);
    if (sparse)
        config.set_deserialize_strict({{"support_material_with_sheath", "0"}});
    // One thread, as the dump does: the tip counts below do not depend on the interleaving of the jobs (the first point inserted by
    // TipPlacement wins a duplicate).
    tbb::global_control limit(tbb::global_control::max_allowed_parallelism, 1);
    tbb::task_arena arena(1);
    arena.execute([&] {
    Print print;
    Model model;
    Test::init_print({SG::slab()}, print, model, config);
    SG::prefix(print);
    const Routed routed(print);
    const std::string used = sparse ? "supportbase" : "rectilinear";
    const std::string other = sparse ? "rectilinear" : "supportbase";
    REQUIRE(routed.params.base_pattern_id == used);
    // The pattern of the base was called, and only that one; no pattern reported an error.
    CHECK(routed.patterns.calls(used) > 0);
    CHECK(routed.patterns.calls(other) == 0);
    CHECK(routed.patterns.errors().empty());
    REQUIRE_FALSE(routed.trees.empty());
    size_t tips = 0;
    for (const TF::RoutedTree &tree : routed.trees)
        tips += tree.nodes.empty() ? 0 : 1;
    CAPTURE(tips, routed.trees.size());
    // Each tree has at least one tip, which is its first node, on the tip layer.
    const size_t tip_layer = routed.trees.front().nodes.front().layer;
    size_t tips_on_tip_layer = 0;
    for (const TF::RoutedTree &tree : routed.trees)
        for (const TF::TreeNode &node : tree.nodes)
            tips_on_tip_layer += node.layer == tip_layer && node.carries.empty() ? 1 : 0;
    CAPTURE(tips_on_tip_layer);
    CHECK(tips_on_tip_layer >= 2);
    if (!sparse)
        CHECK(tips_on_tip_layer == 6);
    });
}

// The honeycomb plugin does not connect its clipped pieces (the honeycomb gap accepted on 2026-10-08). This case characterizes the tips and contacts of a honeycomb base on the slab: the
// tips exist, lie next to the overhang they support, on one layer, and the contacts are kept.
TEST_CASE("Tips of a honeycomb base lie under the overhang", "[plugins][support-tree-routing]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = SG::config("tree", false, 3);
    config.set_deserialize_strict({{"support_material_pattern", "honeycomb"}});
    // One thread, as the dump does: the tip counts below do not depend on the interleaving of the jobs (the first point inserted by
    // TipPlacement wins a duplicate).
    tbb::global_control limit(tbb::global_control::max_allowed_parallelism, 1);
    tbb::task_arena arena(1);
    arena.execute([&] {
    Print print;
    Model model;
    Test::init_print({SG::slab()}, print, model, config);
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    const Routed routed(print);
    REQUIRE(routed.params.base_pattern_id == "honeycomb");
    CHECK(routed.patterns.calls("honeycomb") > 0);
    CHECK(routed.patterns.errors().empty());
    REQUIRE_FALSE(routed.trees.empty());
    REQUIRE_FALSE(routed.support.top.empty());
    // The overhang of the slab: the demand of its islands, with the margin the tips may be grown into.
    ExPolygons demand;
    for (size_t layer = 0; layer < object.layer_count(); ++layer)
        for (const LayerSliceIsland &island : object.layer(layer).islands())
            append(demand, island.support_demand());
    const BoundingBox box = get_extents(demand).inflated(scale_i(1.));
    const size_t tip_layer = routed.trees.front().nodes.front().layer;
    size_t tips = 0;
    for (const TF::RoutedTree &tree : routed.trees)
        for (const TF::TreeNode &node : tree.nodes)
            if (node.carries.empty() && node.layer == tip_layer) {
                ++tips;
                CHECK(box.contains(to_point(node.position)));
            }
    CAPTURE(tips);
    // The plugin does not connect its honeycomb pieces, so it places more tips than the chained legacy lines would (accepted gap).
    CHECK(tips == 15);
    // The contacts that remain are the parts that a tip reaches.
    for (const slic3r_api::Support::Common::GeneratorLayer *contact : routed.support.top)
        CHECK_FALSE(contact->polygons.empty());
    });
}

// A pattern call that cancels the step is the last one: the job checks the cancellation after each call, before the next part or the contour
// fallback, and no job starts afterwards. One thread, so that no other job is inside a fill when the flag is raised.
TEST_CASE("Tip sampling stops after the pattern call that cancels the step", "[plugins][support-tree-routing]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    tbb::global_control limit(tbb::global_control::max_allowed_parallelism, 1);
    tbb::task_arena arena(1);
    arena.execute([] {
        Print print;
        Model model;
        Test::init_print({Slic3r::Test::SupportTree::comparison_mesh(1)}, print, model, SG::config("tree", false, 3));
        SG::prefix(print);
        const auto route = [&](size_t cancel_after) {
            PluginStorage storage;
            storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
            slic3r_api::Support::Common::GeneratedSupport support(handle);
            const slic3r_api::Support::Common::SupportParams params = SG::params(print, print.object(0));
            const TF::TreeSettings settings = Slic3r::Test::SupportTree::tree_settings(print, print.object(0), params);
            Slic3r::Test::SupportTree::SupportPatternContext patterns(print, handle);
            patterns.cancel_after_calls(cancel_after);
            bool cancelled = false;
            try {
                std::vector<TF::RestingPart> resting;
                TF::route_trees(SG::print_view(print), SG::object_view(print.object(0)), params, settings, support, patterns.context(), resting);
            } catch (const slic3r_api::PluginCancelled &) {
                cancelled = true;
            }
            return std::make_pair(cancelled, patterns.total_calls());
        };
        const std::pair<bool, size_t> complete = route(0);
        CHECK_FALSE(complete.first);
        // The T has an overhang on each side, so it needs several fills, so that stopping after the first one means something.
        REQUIRE(complete.second > 1);
        const std::pair<bool, size_t> stopped = route(1);
        CHECK(stopped.first);
        CHECK(stopped.second == 1);
    });
}

// The branches of a tall overhang, far from the object once past the post: the tips keep the tip radius, the drawn radius never
// decreases going down (the foot and the merges only add), and the roots on the bed are widened by the foot up to foot_target[0]
// (the radius of the bed). The routing may stop a foot short of the target where the next cache radius is not free; the roots are
// asserted never above the target and the widest one at the target (within 2 scaled units), since the branches stand in free space.
TEST_CASE("Tree branches widen at the bed to the foot target without ever losing radius", "[plugins][support-tree-routing]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = SG::config("organic");
    config.set_deserialize_strict({{"support_material_buildplate_only", "1"}});
    TriangleMesh post = make_cube(4., 8., 8.);
    TriangleMesh roof = make_cube(12., 8., 1.);
    roof.translate(0., 0., 8.);
    post.merge(roof);
    Print print;
    Model model;
    Test::init_print({post}, print, model, config);
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    const TF::TreeSettings settings = Slic3r::Test::SupportTree::tree_settings(print, object, SG::params(print, object));
    const Routed routed(print);
    REQUIRE_FALSE(routed.trees.empty());
    REQUIRE(settings.foot_target[0] == double(settings.bp_radius));
    double widest_root = 0.;
    size_t roots = 0, tips = 0, links = 0;
    for (const TF::RoutedTree &tree : routed.trees) {
        for (const TF::TreeNode &lower : tree.nodes) {
            if (lower.carries.empty()) {
                CHECK(lower.radius == settings.min_radius);
                ++ tips;
            }
            for (const size_t upper_idx : lower.carries) {
                CAPTURE(lower.layer, lower.radius, tree.nodes[upper_idx].radius);
                CHECK(lower.radius >= tree.nodes[upper_idx].radius);
                ++ links;
            }
        }
        const TF::TreeNode &bottom = tree.nodes.back();
        REQUIRE(bottom.layer == 0);
        ++ roots;
        CAPTURE(bottom.radius, settings.foot_target[0]);
        CHECK(double(bottom.radius) <= settings.foot_target[0] + 2.);
        widest_root = std::max(widest_root, double(bottom.radius));
    }
    CAPTURE(roots, tips, links, widest_root);
    CHECK(tips > 0);
    CHECK(links > 0);
    CHECK(widest_root >= settings.foot_target[0] - 2.);
}

namespace {
// Two tall walls with a slot between them, and a small plate floating in the middle of the slot at the top. The branches of the
// plate can only go down the slot, so the room for the foot of the root on the bed is the half width of the slot minus the XY
// distance: the walls take the room the foot needs (a lone wall would let the branches drift away from it). The walls are thick
// and the plate is small, so the branches of its tips merge into one trunk long before the foot zone. room_mm is the free room
// for the foot at the bed. Returns the widest root radius (scaled), counts the trees, gives in table_limit the largest radius
// below the room that the cache steps allow (the last step of the cache radius table that is not above the room) and
// in foot_step the spacing of the cache steps in the foot band.
double slot_root_radius(const char *xy_spacing, const double room_mm, size_t &tree_count, coord_t &table_limit, coord_t &foot_step)
{
    DynamicPrintConfig config = SG::config("organic");
    config.set_deserialize_strict({{"support_material_buildplate_only", "1"}, {"support_material_xy_spacing", xy_spacing},
                                   {"support_tree_branch_diameter", "2"}});
    // The XY distance is a setting of the print: read it from a small seed object with the same configuration.
    Print seed;
    Model seed_model;
    Test::init_print({make_cube(1., 1., 1.)}, seed, seed_model, config);
    SG::prefix(seed);
    const TF::TreeSettings seed_settings = Slic3r::Test::SupportTree::tree_settings(seed, seed.object(0), SG::params(seed, seed.object(0)));
    const double slot_mm = 2. * (room_mm + unscaled(double(seed_settings.xy_distance)));
    TriangleMesh wall_a = make_cube(2., 10., 10.);
    TriangleMesh wall_b = make_cube(2., 10., 10.);
    wall_b.translate(2. + slot_mm, 0., 0.);
    TriangleMesh plate = make_cube(1., 1., 1.);
    plate.translate(2. + slot_mm / 2. - 0.5, 4.5, 8.);
    wall_a.merge(wall_b);
    wall_a.merge(plate);
    Print print;
    Model model;
    Test::init_print({wall_a}, print, model, config);
    SG::prefix(print);
    const Routed routed(print);
    REQUIRE(routed.settings.branch_radius == scale_i(1.));
    REQUIRE(routed.settings.bp_radius == scale_i(3.75));
    tree_count = routed.trees.size();
    foot_step = routed.settings.foot_cache_step;
    // The steps of the cache radii, with the same settings as the routing.
    PluginStorage storage;
    const TF::TreeVolumes volumes(SG::object_view(print.object(0)), SG::print_view(print), routed.settings, SG::handle(storage), nullptr);
    table_limit = 0;
    for (coord_t radius = 1, limit; (limit = volumes.ceilRadius(radius)) <= scale_i(room_mm); radius = limit + 1)
        table_limit = limit;
    double widest = 0.;
    for (const TF::RoutedTree &tree : routed.trees) {
        const TF::TreeNode &bottom = tree.nodes.back();
        REQUIRE(bottom.layer == 0);
        widest = std::max(widest, double(bottom.radius));
    }
    return widest;
}
}

// The foot of a mature branch widens up to the room it has, to the resolution of the cache steps of the foot band: the root on the
// bed ends in the last cache step that fits in the free room of 3.05 mm (above the limit of the step before, up to the limit of that
// step: the widening of the last layers can stop inside a step that is valid), and never exceeds the room. The table must offer a
// step from 2.85 mm on in the room (REQUIRE below): the plain scale of the tips does not (its last limit below the room is 2.6 mm).
// The XY spacing is generated so that the foot must stop at the same cache step whatever the XY distance.
TEST_CASE("A branch foot widens to the room it has at the resolution of the cache steps", "[plugins][support-tree-routing]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *xy_spacing = GENERATE("0", "3");
    constexpr double room_mm = 3.05;
    CAPTURE(xy_spacing);
    size_t tree_count = 0;
    coord_t table_limit = 0, foot_step = 0;
    const double widest = slot_root_radius(xy_spacing, room_mm, tree_count, table_limit, foot_step);
    CAPTURE(tree_count, widest, table_limit, foot_step);
    REQUIRE(tree_count > 0);
    REQUIRE(table_limit >= scale_i(2.85));
    CHECK(widest <= scale_d(room_mm) + 2.);
    CHECK(widest > double(table_limit - foot_step));
}

// The routing asks for no hold of the tips in a straight column: the interface stack above them is always placed. On this model
// every tip that leaves its vertical leaves it at its first step (the node at distance 1 below the tip has another XY), with 3
// interfaces over the thin and thick bands of the profile. The unsafe tips still try not to move first, as in the native support.
// The tips that stay on their vertical down to the bed are free to do so and are skipped.
TEST_CASE("Tips are free to move from their first step", "[plugins][support-tree-routing]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const double band_mm = GENERATE(0.05, 0.3);
    CAPTURE(band_mm);
    DynamicPrintConfig config = SG::config("organic", false, 3);
    config.set_deserialize_strict({{"support_material_buildplate_only", "1"}});
    Print print;
    Model model;
    SG::init_print_with_profile(print, model, narrow_platform_overhang(), std::vector<coordf_t>{0., 0.3, 3.6, 0.3, 3.61, band_mm, 5.2, band_mm, 5.21, 0.3, 7., 0.3}, config);
    SG::prefix(print);
    const Routed routed(print);
    REQUIRE_FALSE(routed.trees.empty());
    size_t leaving = 0;
    for (const TF::RoutedTree &tree : routed.trees) {
        const std::vector<size_t> below = TF::nodes_below(tree);
        for (size_t tip = 0; tip < tree.nodes.size(); ++ tip) {
            if (! tree.nodes[tip].carries.empty())
                continue;
            // The run of nodes under the tip that keep its XY.
            size_t still = 0, node = tip;
            while (below[node] != TF::no_node && tree.nodes[below[node]].position.x == tree.nodes[tip].position.x &&
                   tree.nodes[below[node]].position.y == tree.nodes[tip].position.y) {
                node = below[node];
                ++ still;
            }
            // A tip that stays on its vertical down to the root does not leave it.
            if (below[node] == TF::no_node)
                continue;
            CAPTURE(tree.nodes[tip].layer, still);
            CHECK(still == 0);
            ++ leaving;
        }
    }
    CAPTURE(leaving);
    CHECK(leaving >= 3);
}
