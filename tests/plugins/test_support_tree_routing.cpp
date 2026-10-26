#include "support_grid_test_helpers.hpp"
#include "support_tree_test_helpers.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Plugins/Support/Tree/TreeRouting.hpp"
#include "libslic3r/Plugins/Support/Tree/TreeSettings.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

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

// The routed trees of the first object of a sliced print, with the storage that owns the contacts they support.
struct Routed {
    PluginStorage storage;
    slic3r_api::Support::Common::GeneratedSupport support{reinterpret_cast<storage_handle *>(&storage)};
    const slic3r_api::Support::Common::SupportParams params;
    const TF::TreeSettings settings;
    std::vector<TF::RoutedTree> trees;
    explicit Routed(const Print &print)
        : params(SG::params(print, print.object(0))), settings(Slic3r::Test::SupportTree::tree_settings(print, print.object(0), params)),
          trees(TF::route_trees(print, print.object(0), params, settings, support)) {}
};

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
            CAPTURE(node.layer, node.print_z, unscaled(node.position.x()), unscaled(node.position.y()));
            const double distance_mm = distance_to_outline_mm(node.position, outline);
            closest_mm = std::min(closest_mm, distance_mm);
            CHECK_FALSE(contains(outline, node.position));
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
                const double move_mm = unscaled((lower.position - upper.position).cast<double>().norm());
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
                const double move_mm = unscaled((lower.position - upper.position).cast<double>().norm());
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
                min_x = std::min(min_x, node.position.x());
                max_x = std::max(max_x, node.position.x());
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
        tips_on_shelf = tips_on_shelf || contains(shelf_area, tree.nodes.front().position);
    CHECK(tips_on_shelf);
}

// A XY spacing well above half the external perimeter width: next to the object the tips may keep the reduced distance, and
// the branches that have moved away keep the full spacing on the lower layers, where the post is.
TEST_CASE("Tips use the reduced XY distance next to the object and the full distance lower down", "[plugins][support-tree-routing]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = SG::config("organic");
    config.set_deserialize_strict({{"support_material_xy_spacing", "1"}});
    TriangleMesh lip = make_cube(4., 8., 4.);
    TriangleMesh roof = make_cube(4.5, 8., 1.);
    roof.translate(0.f, 0.f, 4.f);
    lip.merge(roof);
    Print print;
    Model model;
    Test::init_print({lip}, print, model, config);
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    const TF::TreeSettings settings = Slic3r::Test::SupportTree::tree_settings(print, object, SG::params(print, object));
    const double xy_min_mm = unscaled(double(settings.xy_min_distance));
    const double xy_mm = unscaled(double(settings.xy_distance));
    CAPTURE(xy_min_mm, xy_mm);
    REQUIRE(xy_min_mm < 0.5 * xy_mm);
    const Routed routed(print);
    REQUIRE_FALSE(routed.trees.empty());
    double closest_tip_mm = std::numeric_limits<double>::max();
    for (const TF::RoutedTree &tree : routed.trees) {
        const TF::LayerIndex tip = TF::LayerIndex(tree.nodes.front().layer);
        for (const TF::TreeNode &node : tree.nodes) {
            const TF::LayerIndex layer = TF::LayerIndex(node.layer);
            const double distance_mm = distance_to_outline_mm(node.position, layer_outline(object, node.layer));
            CAPTURE(layer, unscaled(node.position.x()), unscaled(node.position.y()), distance_mm);
            if (layer + 2 >= tip) {
                CHECK(distance_mm >= xy_min_mm - check_tolerance_mm);
                closest_tip_mm = std::min(closest_tip_mm, distance_mm);
            } else if (layer <= 3) {
                CHECK(distance_mm >= xy_mm - check_tolerance_mm);
            }
        }
    }
    CHECK(closest_tip_mm < xy_mm - 0.1);
}
