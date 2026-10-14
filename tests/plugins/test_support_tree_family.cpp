#include "support_grid_test_helpers.hpp"
#include "support_tree_test_helpers.hpp"
#include "gcode_test_helpers.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/GCode/ToolOrdering.hpp"
#include "libslic3r/Plugins/Support/Common/SupportPublication.hpp"
#include "libslic3r/Plugins/Support/Common/SupportZPlacement.hpp"
#include "libslic3r/Plugins/Support/Tree/TreeBranchMesh.hpp"
#include "libslic3r/Plugins/Support/Tree/TreeRouting.hpp"
#include "libslic3r/Plugins/Support/Tree/TreeSettings.hpp"
#include "libslic3r/Plugins/Support/Tree/TreeStorage.hpp"
#include "libslic3r/Plugins/Support/Tree/TreeSupportGenerator.hpp"
#include "libslic3r/Plugins/Support/Tree/TreeSmoothing.hpp"
#include "libslic3r/Plugins/Support/Tree/TreeZPlanner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>

/* Tree-family tests: the slabs that the tree and organic styles publish. The branch slab planner is checked on synthetic
   overhang layers, the slabs of small objects on their geometry (interfaces, floors, heights, XY distance, branch overhang),
   then the published surfaces and the G-code of a whole print. The grid is unchanged by this family, which the grid
   comparison tests cover. The Print::validate checks of the tree family settings (the [validate] case) are checked here too. */

using namespace Slic3r;
namespace SG = Slic3r::Test::SupportGrid;
namespace TF = Slic3r::TreeFamily;
namespace Common = slic3r_api::Support::Common;

namespace {
// Tolerance of the geometric checks, in millimeters.
constexpr double tolerance_mm = 0.01;

// The tree slabs of the first object of a sliced print, generated directly. The storage is declared first so that it
// outlives the slabs that live in it.
struct TreeSlabs {
    PluginStorage storage;
    Common::GeneratedSupport support{reinterpret_cast<storage_handle *>(&storage)};
    TreeSlabs(Print &print, bool organic)
    {
        const Slic3r::Test::SupportTree::SupportPatternContext patterns(print, reinterpret_cast<storage_handle *>(&storage));
        TF::generate_tree_support(patterns.context(), print, print.object(0), organic, support);
    }
};

using Routed = Slic3r::Test::SupportTree::Routed;
using Slic3r::Test::SupportTree::to_c_point;
using Slic3r::Test::SupportTree::to_point;

// Distance in millimeters from a point to the region of the polygons: zero inside, else to the nearest outline.
double distance_to_region_mm(const Point &point, const Polygons &polygons)
{
    double best = std::numeric_limits<double>::max();
    for (const Polygon &polygon : polygons) {
        if (polygon.contains(point))
            return 0.;
        for (size_t idx = 0; idx < polygon.size(); ++idx) {
            const Vec2d a = polygon.points[idx].cast<double>();
            const Vec2d ab = polygon.points[(idx + 1) % polygon.size()].cast<double>() - a;
            const Vec2d ap = point.cast<double>() - a;
            const double len2 = ab.squaredNorm();
            const double t = len2 > 0. ? std::clamp(ap.dot(ab) / len2, 0., 1.) : 0.;
            best = std::min(best, (ap - ab * t).norm());
        }
    }
    return unscaled(best);
}

// The largest outline overhang in millimeters of an upper slab over a lower one: the farthest vertex of the upper slab
// outside the lower slab.
double overhang_mm(const Polygons &upper, const Polygons &lower)
{
    double largest = 0.;
    for (const Polygon &polygon : upper)
        for (const Point &point : polygon.points)
            largest = std::max(largest, distance_to_region_mm(point, lower));
    return largest;
}

// A block with a post and a roof above its middle. The roof needs support, and the build plate is the only place a branch
// may stand, so the branch of its underside leaves the block diagonally to reach the bed.
TriangleMesh diagonal_roof()
{
    TriangleMesh block = make_cube(10., 10., 2.);
    TriangleMesh post = make_cube(1., 10., 6.);
    post.translate(3., 0., 2.);
    TriangleMesh roof = make_cube(3., 10., 1.);
    roof.translate(3., 0., 8.);
    block.merge(post);
    block.merge(roof);
    return block;
}

// The slabs of the base layers that the branches fill, bottom first, with the native polygons of each.
struct BaseSlab {
    coord_t bottom_z, print_z;
    Polygons polygons;
};
std::vector<BaseSlab> base_slabs(const Common::GeneratedSupport &support)
{
    std::vector<BaseSlab> slabs;
    for (const Common::GeneratorLayer *slab : support.base)
        slabs.push_back(BaseSlab{slab->bottom_z, slab->print_z, SG::native(slab->polygons)});
    std::sort(slabs.begin(), slabs.end(), [](const BaseSlab &a, const BaseSlab &b) { return a.print_z < b.print_z; });
    return slabs;
}

// Checks a plan of branch slabs: each slab is within the height limits, and its overhang, the rate of the largest layer it
// reaches times its height, is within the limit. The last top is the upper extreme.
void check_plan(const std::vector<coord_t> &tops, coord_t from, coord_t to, const std::vector<TF::OverhangLayer> &layers, coord_t limit,
    coord_t minimum, coord_t maximum)
{
    REQUIRE_FALSE(tops.empty());
    CHECK(tops.back() == to);
    coord_t bottom = from;
    for (const coord_t top : tops) {
        double rate = 0.;
        for (const TF::OverhangLayer &layer : layers)
            if (layer.bottom_z < top && layer.top_z > bottom)
                rate = std::max(rate, layer.rate);
        CAPTURE(bottom, top, rate);
        CHECK(top - bottom >= minimum);
        CHECK(top - bottom <= maximum);
        CHECK(rate * double(top - bottom) <= double(limit));
        bottom = top;
    }
}

// One object of a print: its mesh, the offset of its instance on the bed, and its layer height when it has its own.
struct PlacedObject {
    TriangleMesh mesh;
    Vec3d offset;
    std::string layer_height;
};

// Builds a print of the objects at the places they are given: they are not arranged, so the test knows where its geometry lies.
void place_print(const std::vector<PlacedObject> &objects, const DynamicPrintConfig &config, Print &print, Model &model)
{
    for (const PlacedObject &placed : objects) {
        ModelObject *object = model.add_object();
        object->name += "object.stl";
        object->add_volume(placed.mesh);
        object->add_instance();
        object->instances.front()->set_offset(placed.offset);
        if (!placed.layer_height.empty())
            object->config.set_deserialize_strict("layer_height", placed.layer_height);
    }
    for (ModelObject &object : model.objects()) {
        object.ensure_on_bed();
        print.auto_assign_extruders(&object);
    }
    print.apply(model, config);
    print.validate();
    print.set_status_silent();
}
} // namespace

// The planner gives every slab a height on the Z grid, within the minimum and the maximum, and the last slab ends at the
// upper extreme. Without any branch the slabs take the maximum height.
TEST_CASE("Branch slab planner keeps heights on the grid within the limits", "[plugins][support-tree-family][planner]")
{
    const coord_t step = scale_to_layer_coord(0.01);
    const coord_t minimum = scale_to_layer_coord(0.07), maximum = scale_to_layer_coord(0.3);
    const coord_t from = scale_to_layer_coord(0.2), to = scale_to_layer_coord(5.);
    const std::vector<TF::OverhangLayer> no_branch;
    const std::vector<coord_t> tops = TF::plan_branch_slabs(from, to, no_branch, scale_to_layer_coord(0.1), minimum, maximum, step);
    REQUIRE(tops.size() >= 2);
    CHECK(tops.back() == to);
    coord_t bottom = from;
    for (size_t idx = 0; idx < tops.size(); ++idx) {
        const coord_t top = tops[idx];
        CHECK(top > bottom);
        CHECK(top - bottom <= maximum);
        CHECK(top - bottom >= minimum);
        // The last slab takes the rest up to the upper extreme, which need not lie on the step.
        if (idx + 1 < tops.size())
            CHECK((top - bottom) % step == 0);
        bottom = top;
    }
    CHECK(tops.front() - from == maximum / step * step);
}

// A branch leaning with a rate of one (a horizontal shift equal to its height) allows a slab as thick as the overhang limit,
// on the grid and rounded down.
TEST_CASE("Branch slab planner limits each slab to the overhang of the branches", "[plugins][support-tree-family][planner]")
{
    const coord_t step = scale_to_layer_coord(0.01);
    const coord_t minimum = scale_to_layer_coord(0.07), maximum = scale_to_layer_coord(0.3);
    const coord_t limit = scale_to_layer_coord(0.1);
    const coord_t from = 0, to = scale_to_layer_coord(2.);
    std::vector<TF::OverhangLayer> leaning;
    for (coord_t z = 0; z < to; z += scale_to_layer_coord(0.2))
        leaning.push_back(TF::OverhangLayer{ z, z + scale_to_layer_coord(0.2), 1. });
    const std::vector<coord_t> tops = TF::plan_branch_slabs(from, to, leaning, limit, minimum, maximum, step);
    CHECK(tops.back() == to);
    coord_t bottom = from;
    for (const coord_t top : tops) {
        CHECK(top - bottom <= limit);
        CHECK(top - bottom >= limit - step);
        bottom = top;
    }
    CHECK(tops.size() >= 15);
}

// A rest thinner than the minimum height shares the last two slabs, so that both stay printable.
TEST_CASE("Branch slab planner shares the last two slabs when the rest is too thin", "[plugins][support-tree-family][planner]")
{
    const coord_t step = scale_to_layer_coord(0.01);
    const coord_t minimum = scale_to_layer_coord(0.07), maximum = scale_to_layer_coord(0.3);
    const coord_t from = 0, to = maximum + scale_to_layer_coord(0.03);
    const std::vector<coord_t> tops = TF::plan_branch_slabs(from, to, {}, scale_to_layer_coord(1.), minimum, maximum, step);
    REQUIRE(tops.size() == 2);
    CHECK(tops.back() == to);
    CHECK(tops.front() - from >= minimum);
    CHECK(to - tops.front() >= minimum);
    CHECK(tops.front() - from <= maximum);
}

// The rest below the upper extreme takes the last tops back: with a rate of one and a 0.10 mm overhang limit, the tops 0.10 and
// 0.20 leave a 0.03 mm rest, and the 0.13 mm span they then share overhangs, while 0.07 + 0.08 + 0.08 fits.
TEST_CASE("Branch slab planner backs off the last tops when the shared span would overhang", "[plugins][support-tree-family][planner]")
{
    const coord_t step = scale_to_layer_coord(0.01);
    const coord_t minimum = scale_to_layer_coord(0.07), maximum = scale_to_layer_coord(0.3);
    const coord_t limit = scale_to_layer_coord(0.1), to = scale_to_layer_coord(0.23);
    const std::vector<TF::OverhangLayer> leaning{ TF::OverhangLayer{ 0, to, 1. } };
    check_plan(TF::plan_branch_slabs(0, to, leaning, limit, minimum, maximum, step), 0, to, leaning, limit, minimum, maximum);
}

// A steep stretch under the upper extreme: the slabs cut from the span must keep the limit of the layers they reach.
TEST_CASE("Branch slab planner keeps the overhang limit next to a rate change", "[plugins][support-tree-family][planner]")
{
    const coord_t step = scale_to_layer_coord(0.01);
    const coord_t minimum = scale_to_layer_coord(0.07), maximum = scale_to_layer_coord(0.3);
    const coord_t limit = scale_to_layer_coord(0.1), to = scale_to_layer_coord(0.21);
    const std::vector<TF::OverhangLayer> layers{ TF::OverhangLayer{ 0, scale_to_layer_coord(0.15), 0. },
        TF::OverhangLayer{ scale_to_layer_coord(0.15), scale_to_layer_coord(0.23), 1. } };
    check_plan(TF::plan_branch_slabs(0, to, layers, limit, minimum, maximum, step), 0, to, layers, limit, minimum, maximum);
    // The climb reaches 0.30 with a 0.03 rest, and every uniform cut of the 0.33 span overhangs: the top slab must be the
    // constrained one, 0.26 + 0.07.
    const coord_t top = scale_to_layer_coord(0.33);
    const std::vector<TF::OverhangLayer> stepped{ TF::OverhangLayer{ 0, scale_to_layer_coord(0.30), 0. },
        TF::OverhangLayer{ scale_to_layer_coord(0.30), top, 1. } };
    const coord_t overhang_limit = scale_to_layer_coord(0.07);
    check_plan(TF::plan_branch_slabs(0, top, stepped, overhang_limit, minimum, maximum, step), 0, top, stepped, overhang_limit, minimum,
        maximum);
}

// With no machine Z step, the only cut that keeps the rate-one layers within the overhang limit starts the top slab at 0.2601 mm.
// That position lies on the grid measured back from the upper extreme 0.3301 mm, not on the grid measured from 0.
TEST_CASE("Branch slab planner cuts from the upper extreme when the Z step is zero", "[plugins][support-tree-family][planner]")
{
    const coord_t minimum = scale_to_layer_coord(0.07), maximum = scale_to_layer_coord(0.3);
    const coord_t limit = scale_to_layer_coord(0.07), top = scale_to_layer_coord(0.3301);
    const std::vector<TF::OverhangLayer> stepped{ TF::OverhangLayer{ 0, scale_to_layer_coord(0.30), 0. },
        TF::OverhangLayer{ scale_to_layer_coord(0.30), top, 1. } };
    const std::vector<coord_t> tops = TF::plan_branch_slabs(0, top, stepped, limit, minimum, maximum, 0);
    check_plan(tops, 0, top, stepped, limit, minimum, maximum);
    REQUIRE(tops.size() == 2);
    CHECK(tops.front() == scale_to_layer_coord(0.2601));
}

// Infeasible settings never assert: a minimum above the maximum keeps the minimum, a middle thinner than the minimum stays one
// slab, and a zero minimum is raised to the Z step.
TEST_CASE("Branch slab planner degrades gracefully on infeasible settings", "[plugins][support-tree-family][planner]")
{
    const coord_t step = scale_to_layer_coord(0.01);
    const coord_t minimum = scale_to_layer_coord(0.07), maximum = scale_to_layer_coord(0.3);
    SECTION("a minimum above the maximum keeps the minimum") {
        const std::vector<coord_t> tops = TF::plan_branch_slabs(0, scale_to_layer_coord(1.), {}, scale_to_layer_coord(1.),
            scale_to_layer_coord(0.4), maximum, step);
        REQUIRE_FALSE(tops.empty());
        CHECK(tops.front() >= scale_to_layer_coord(0.4));
        CHECK(tops.back() == scale_to_layer_coord(1.));
    }
    SECTION("a middle thinner than the minimum stays one slab") {
        const std::vector<coord_t> tops = TF::plan_branch_slabs(0, scale_to_layer_coord(0.05), {}, scale_to_layer_coord(1.), minimum,
            maximum, step);
        CHECK(tops == std::vector<coord_t>{ scale_to_layer_coord(0.05) });
    }
    SECTION("a zero minimum and a zero step still tile the span") {
        const std::vector<coord_t> tops = TF::plan_branch_slabs(0, scale_to_layer_coord(1.), {}, scale_to_layer_coord(1.), 0, maximum, 0);
        REQUIRE_FALSE(tops.empty());
        CHECK(tops.back() == scale_to_layer_coord(1.));
    }
}

// A branch that leans out of the block must go diagonally to the bed. With two maximum overhangs, the smaller one cuts the
// inclined part into more and thinner slabs, and the overhang between consecutive slabs of that part stays within the setting.
TEST_CASE("A diagonal branch with build plate only takes thinner slabs for a smaller max overhang",
    "[plugins][support-tree-family][diagonal]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    struct Measure {
        size_t slabs = 0;
        double mean_height_mm = 0., largest_overhang_mm = 0.;
    };
    std::map<double, Measure> measures;
    for (const double limit_mm : {0.2, 0.1}) {
        DynamicPrintConfig config = SG::config("organic", false, 0);
        config.set_deserialize_strict({{"support_material_buildplate_only", "1"},
            {"support_tree_max_overhang", std::to_string(limit_mm)}});
        Print print;
        Model model;
        Test::init_print({diagonal_roof()}, print, model, config);
        SG::prefix(print);
        const Routed routed(print);
        REQUIRE_FALSE(routed.trees.empty());
        // The inclined part is where the routed branches move at least half their height sideways.
        double inclined_bottom = std::numeric_limits<double>::max(), inclined_top = 0.;
        double largest_travel_mm = 0.;
        for (const TF::RoutedTree &tree : routed.trees) {
            if (tree.nodes.empty()) continue;
            largest_travel_mm = std::max(largest_travel_mm, unscaled((to_point(tree.nodes.front().position) - to_point(tree.nodes.back().position)).cast<double>().norm()));
            for (const TF::TreeNode &lower : tree.nodes)
                for (const size_t upper_idx : lower.carries) {
                    const TF::TreeNode &upper = tree.nodes[upper_idx];
                    const double shift_mm = unscaled((to_point(upper.position) - to_point(lower.position)).cast<double>().norm());
                    const double height_mm = upper.print_z - lower.print_z;
                    if (height_mm > 0. && shift_mm >= 0.5 * height_mm) {
                        inclined_bottom = std::min(inclined_bottom, lower.print_z);
                        inclined_top = std::max(inclined_top, upper.print_z);
                    }
                }
        }
        CAPTURE(limit_mm, inclined_bottom, inclined_top, largest_travel_mm);
        CHECK(largest_travel_mm >= 2.);
        REQUIRE(inclined_top > inclined_bottom);
        const TreeSlabs slabs(print, true);
        const std::vector<BaseSlab> base = base_slabs(slabs.support);
        Measure &measure = measures[limit_mm];
        double height_total = 0.;
        const BaseSlab *previous = nullptr;
        for (const BaseSlab &slab : base) {
            const double middle_mm = unscaled(slab.bottom_z + slab.print_z) / 2.;
            if (slab.polygons.empty() || middle_mm < inclined_bottom || middle_mm > inclined_top) {
                previous = nullptr;
                continue;
            }
            ++measure.slabs;
            height_total += unscaled(slab.print_z - slab.bottom_z);
            if (previous != nullptr)
                measure.largest_overhang_mm = std::max(measure.largest_overhang_mm, overhang_mm(slab.polygons, previous->polygons));
            previous = &slab;
        }
        REQUIRE(measure.slabs > 0);
        measure.mean_height_mm = height_total / double(measure.slabs);
        CAPTURE(limit_mm, measure.slabs, measure.mean_height_mm, measure.largest_overhang_mm);
        CHECK(measure.largest_overhang_mm <= limit_mm + tolerance_mm);
    }
    CHECK(measures.at(0.1).slabs > measures.at(0.2).slabs);
    CHECK(measures.at(0.1).mean_height_mm < measures.at(0.2).mean_height_mm);
}

// Every top contact lies on its interface stack: the configured number of interface slabs (the contact included) under it,
// at the dense step of the interfaces, and the contact has its printed polygons.
TEST_CASE("Tree contacts carry their interface slabs at the interface step", "[plugins][support-tree-family][slabs]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool organic = GENERATE(false, true);
    CAPTURE(organic);
    DynamicPrintConfig config = SG::config(organic ? "organic" : "tree", false, 3);
    Print print;
    Model model;
    Test::init_print({SG::slab()}, print, model, config);
    SG::prefix(print);
    const TF::TreeSettings settings = Slic3r::Test::SupportTree::tree_settings(print, print.object(0), SG::params(print, print.object(0)));
    const TreeSlabs slabs(print, organic);
    REQUIRE_FALSE(slabs.support.top.empty());
    const coord_t dense = settings.dense_slab_height;
    for (const Common::GeneratorLayer *contact : slabs.support.top) {
        CHECK(contact->type == Common::LayerType::TopContact);
        CHECK_FALSE(SG::native(contact->polygons).empty());
        CHECK(contact->print_z - contact->bottom_z == dense);
        for (const coord_t z : {contact->bottom_z, contact->bottom_z - dense}) {
            const bool found = std::any_of(slabs.support.interfaces.begin(), slabs.support.interfaces.end(),
                [z, dense](const Common::GeneratorLayer *layer) {
                    return layer->type == Common::LayerType::TopInterface && layer->print_z == z && layer->height == dense;
                });
            CHECK(found);
        }
    }
}

// The area in square millimeters of the branch sections that a contact at the Z holds, as the generator draws them there: the
// discs of the tree stretches, each from a node up to the node it carries or from a tip up to the layer above it, with the radius
// interpolated at that Z; or the slices of the organic branch meshes. The organic tip circles that draw_tip_contacts adds for tips
// at or below the middle are not counted: the tips of this fixture lie above the middles, so none applies.
double contact_branch_mm2(const PrintObject &object, const std::vector<TF::RoutedTree> &trees, bool organic, coord_t z)
{
    if (organic) {
        const std::vector<Polygons> sections = TF::slice_branch_meshes(trees, { z }, [] {});
        return unscaled(unscaled(area(union_(sections.front()))));
    }
    double total = 0.;
    for (const TF::RoutedTree &tree : trees)
        for (const TF::TreeNode &node : tree.nodes) {
            const coord_t bottom_z = object.layer(node.layer).scaled_print_z();
            const auto add = [&](coord_t top_z, coord_t top_radius) {
                if (bottom_z > z || top_z < z)
                    return;
                const double t = double(z - bottom_z) / double(top_z - bottom_z);
                total += M_PI * sqr(unscaled(double(node.radius) + double(top_radius - node.radius) * t));
            };
            if (node.carries.empty() && node.layer + 1 < object.layer_count())
                add(object.layer(node.layer + 1).scaled_print_z(), node.radius);
            for (const size_t upper : node.carries)
                add(object.layer(tree.nodes[upper].layer).scaled_print_z(), tree.nodes[upper].radius);
        }
    return total;
}

// Without interface layers the top contact is no roof: its branches run through it, and it publishes the branch sections drawn at
// its middle, typed as the branches of its style, instead of the footprint of the overhang. Every tip lies on the contact, and the
// published area matches the branch sections that the generator draws there.
TEST_CASE("Tree contacts without interface layers publish the branches under them alone", "[plugins][support-tree-family][slabs]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool organic = GENERATE(false, true);
    CAPTURE(organic);
    Print print;
    Model model;
    Test::init_print({SG::slab()}, print, model, SG::config(organic ? "organic" : "tree", false, 0));
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    Routed routed(print);
    TF::generate_tree_slabs(nullptr, print, object, routed.trees, organic, routed.support);
    REQUIRE_FALSE(routed.support.top.empty());
    double overhang_mm2 = 0.;
    for (size_t id = 1; id < object.layer_count(); ++id)
        overhang_mm2 += unscaled(unscaled(area(diff_ex(object.layer(id).lslices(), object.layer(id - 1).lslices()))));
    std::vector<Point> tips;
    for (const TF::RoutedTree &tree : routed.trees)
        for (const TF::TreeNode &node : tree.nodes)
            if (node.carries.empty() && node.layer + 1 < object.layer_count())
                tips.push_back(to_point(node.position));
    REQUIRE_FALSE(tips.empty());
    Polygons contact_area;
    std::vector<coord_t> contact_zs;
    double expected_mm2 = 0.;
    for (const Common::GeneratorLayer *contact : routed.support.top) {
        CHECK(contact->type == (organic ? Common::LayerType::Branch : Common::LayerType::Base));
        const Polygons &polygons = SG::native(contact->polygons);
        contact_area.insert(contact_area.end(), polygons.begin(), polygons.end());
        contact_zs.push_back(contact->print_z);
        expected_mm2 += contact_branch_mm2(object, routed.trees, organic, contact->bottom_z + (contact->print_z - contact->bottom_z) / 2);
    }
    for (const Point &tip : tips)
        CHECK(distance_to_region_mm(tip, contact_area) <= tolerance_mm);
    SG::family(print);
    const SurfaceType branch_type = stModSupport | stModBranch | stPosInternal | stDensSparse;
    const SurfaceType sparse_type = stModSupport | stPosInternal | stDensSparse;
    double published_mm2 = 0., typed_mm2 = 0.;
    for (const Layer &layer : object.auxiliary_layers()) {
        if (std::find(contact_zs.begin(), contact_zs.end(), layer.scaled_print_z()) == contact_zs.end())
            continue;
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands())
                for (const Surface &surface : region.fill_surfaces()) {
                    const double surface_mm2 = unscaled(unscaled(area(surface.expolygon)));
                    published_mm2 += surface_mm2;
                    if (surface.surface_type == (organic ? branch_type : sparse_type))
                        typed_mm2 += surface_mm2;
                }
    }
    CAPTURE(overhang_mm2, expected_mm2, published_mm2, tips.size());
    CHECK(published_mm2 <= 0.5 * overhang_mm2);
    CHECK(published_mm2 == Approx(expected_mm2).epsilon(0.1));
    CHECK(typed_mm2 == Approx(published_mm2));
}

// An organic tip ends in a half sphere on its own node, which can stop below the middle of a contact that is thin next to the
// object layers above it. A tip below the middle of a contact reaches it, so the tips are published at the contact Z in both styles.
TEST_CASE("Tips below a thin contact are published at its Z", "[plugins][support-tree-organic][slabs]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool organic = GENERATE(false, true);
    CAPTURE(organic);
    DynamicPrintConfig config = SG::config(organic ? "organic" : "tree", false, 0);
    config.set_deserialize_strict({{"layer_height", "0.3"}, {"support_material_layer_height", "0.05"},
        {"support_material_contact_distance", "0.05"}, {"support_tree_tip_diameter", "0.4"}, {"support_material_extrusion_width", "0.4"}});
    TriangleMesh post = make_cube(4., 8., 4.1);
    TriangleMesh roof = make_cube(12., 8., 1.);
    roof.translate(0.f, 0.f, 4.1f);
    post.merge(roof);
    Print print;
    Model model;
    Test::init_print({ post }, print, model, config);
    SG::prefix(print);
    PrintObject &object = print.object(0);
    Routed routed(print);
    TF::generate_tree_slabs(nullptr, print, object, routed.trees, organic, routed.support);
    REQUIRE_FALSE(routed.support.top.empty());
    const coord_t contact_z = routed.support.top.front()->print_z;
    REQUIRE(unscaled(contact_z) == Approx(4.05));
    Common::publish(nullptr, SG::object_view(object), routed.support, reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
    Polygons published;
    for (const Layer &layer : object.auxiliary_layers())
        if (layer.scaled_print_z() == contact_z)
            for (const LayerSliceIsland &island : layer.islands())
                for (const LayerRegionIsland &region : island.regions_islands())
                    for (const Surface &surface : region.fill_surfaces()) {
                        const Polygons polygons = to_polygons(surface.expolygon);
                        published.insert(published.end(), polygons.begin(), polygons.end());
                    }
    REQUIRE_FALSE(published.empty());
    size_t tips = 0;
    for (const TF::RoutedTree &tree : routed.trees)
        for (const TF::TreeNode &node : tree.nodes)
            if (node.carries.empty() && node.layer + 1 < object.layer_count()) {
                ++tips;
                CHECK(distance_to_region_mm(to_point(node.position), published) <= tolerance_mm);
            }
    REQUIRE(tips > 0);
}

// The base and dense slabs lie between the minimum and the largest thickness, and every slab edge sits on the layer Z grid,
// where the object layers and the contacts already sit.
TEST_CASE("Tree slabs keep their heights within the support limits", "[plugins][support-tree-family][slabs]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool organic = GENERATE(false, true);
    CAPTURE(organic);
    DynamicPrintConfig config = SG::config(organic ? "organic" : "tree", false, 3);
    Print print;
    Model model;
    Test::init_print({SG::slab()}, print, model, config);
    SG::prefix(print);
    const Common::SupportParams params = SG::params(print, print.object(0));
    const TF::TreeSettings settings = Slic3r::Test::SupportTree::tree_settings(print, print.object(0), params);
    const TreeSlabs slabs(print, organic);
    const coord_t largest = std::max(settings.max_slab_height, settings.dense_slab_height);
    for (const Common::GeneratorLayer *slab : slabs.support.base) {
        const coord_t height = slab->print_z - slab->bottom_z;
        CAPTURE(slab->print_z, slab->bottom_z, height);
        CHECK(height >= settings.min_slab_height);
        CHECK(height <= largest);
        CHECK(slab->bottom_z >= 0);
        CHECK(slab->print_z % SCALED_LAYER_Z_STEP == 0);
        CHECK(slab->bottom_z % SCALED_LAYER_Z_STEP == 0);
    }
}

// No branch area comes closer to the object than the XY distance: every slab, contact and interface keeps clear of the object
// outlines of the layers it overlaps.
TEST_CASE("Tree slabs keep the XY distance from the object", "[plugins][support-tree-family][slabs]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool organic = GENERATE(false, true);
    CAPTURE(organic);
    DynamicPrintConfig config = SG::config(organic ? "organic" : "tree", false, 3);
    Print print;
    Model model;
    Test::init_print({SG::slab(true)}, print, model, config);
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    const TF::TreeSettings settings = Slic3r::Test::SupportTree::tree_settings(print, object, SG::params(print, object));
    const TreeSlabs slabs(print, organic);
    const auto check_slab = [&](const Common::GeneratorLayer &slab) {
        const Polygons &area = SG::native(slab.polygons);
        if (area.empty()) return;
        for (size_t id = 0; id < object.layer_count(); ++id) {
            const Layer &layer = object.layer(id);
            if (layer.scaled_bottom_z() >= slab.print_z || layer.scaled_print_z() <= slab.bottom_z) continue;
            const Polygons keep_out = to_polygons(offset_ex(layer.lslices(), float(settings.xy_min_distance - scale_d(tolerance_mm))));
            CHECK(intersection(area, keep_out).empty());
        }
    };
    for (const Common::GeneratorLayer *slab : slabs.support.top) check_slab(*slab);
    for (const Common::GeneratorLayer *slab : slabs.support.bottom) check_slab(*slab);
    for (const Common::GeneratorLayer *slab : slabs.support.base) check_slab(*slab);
    for (const Common::GeneratorLayer *slab : slabs.support.interfaces) check_slab(*slab);
}

// A branch that rests on the platform gets a bottom contact on its top surface, and dense bottom interfaces above it.
TEST_CASE("Tree floors put a bottom contact and interfaces on the object", "[plugins][support-tree-family][slabs]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool organic = GENERATE(false, true);
    CAPTURE(organic);
    DynamicPrintConfig config = SG::config(organic ? "organic" : "tree", false, 3);
    Print print;
    Model model;
    Test::init_print({SG::slab(true)}, print, model, config);
    SG::prefix(print);
    const TreeSlabs slabs(print, organic);
    REQUIRE_FALSE(slabs.support.bottom.empty());
    for (const Common::GeneratorLayer *contact : slabs.support.bottom) {
        CHECK(contact->type == Common::LayerType::BottomContact);
        CHECK_FALSE(SG::native(contact->polygons).empty());
        CHECK(contact->object_below < print.object(0).layer_count());
    }
    const bool interfaces = std::any_of(slabs.support.interfaces.begin(), slabs.support.interfaces.end(),
        [](const Common::GeneratorLayer *layer) { return layer->type == Common::LayerType::BottomInterface; });
    CHECK(interfaces);
}

// With build plate only, the demand that cannot reach the bed gets no contact, while the demand that reaches the bed keeps its
// contact. The roof hangs on a pillar over a block that a branch cannot leave in the height between them.
TEST_CASE("Build plate only keeps the contacts of the demands that reach the bed", "[plugins][support-tree-family][routing]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    TriangleMesh post = make_cube(4., 10., 6.);
    post.translate(-12., 0., 0.);
    TriangleMesh arm = make_cube(12., 10., 1.);
    arm.translate(-20., 0., 6.);
    TriangleMesh block = make_cube(30., 30., 2.);
    block.translate(6., -10., 0.);
    TriangleMesh pillar = make_cube(2., 2., 4.);
    pillar.translate(19., 4., 2.);
    TriangleMesh roof = make_cube(10., 10., 1.);
    roof.translate(14., 0., 6.);
    post.merge(arm);
    post.merge(block);
    post.merge(pillar);
    post.merge(roof);
    DynamicPrintConfig config = SG::config("organic", false, 0);
    config.set_deserialize_strict({{"support_material_buildplate_only", "1"}});
    Print print;
    Model model;
    place_print({ { post, Vec3d(100., 100., 0.), "" } }, config, print, model);
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    BoundingBox extents = get_extents(object.layer(0).lslices());
    for (size_t id = 1; id < object.layer_count(); ++id)
        extents.merge(get_extents(object.layer(id).lslices()));
    const TreeSlabs slabs(print, true);
    REQUIRE_FALSE(slabs.support.top.empty());
    // The arm reaches the bed within 8 mm of the left edge of the object; the roof starts 34 mm from that edge.
    for (const Common::GeneratorLayer *contact : slabs.support.top)
        for (const Polygon &polygon : SG::native(contact->polygons))
            for (const Point &point : polygon.points)
                CHECK(unscaled(point.x() - extents.min.x()) < 16.);
}

// The drawing trims the object at the XY distance that the routing places the tips at, so a tip keeps its circle when the XY
// spacing of the object is larger than half the external perimeter width.
TEST_CASE("Tree tips keep their circle at the routing XY distance", "[plugins][support-tree-family][slabs]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = SG::config("tree", false, 3);
    config.set_deserialize_strict({{"support_material_xy_spacing", "3"}});
    Print print;
    Model model;
    Test::init_print({SG::slab(true)}, print, model, config);
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    const Routed routed(print);
    const TreeSlabs slabs(print, false);
    size_t tips = 0;
    for (const TF::RoutedTree &tree : routed.trees)
        for (const TF::TreeNode &node : tree.nodes) {
            if (!node.carries.empty() || node.layer + 1 >= object.layer_count())
                continue;
            ++tips;
            // A tip is drawn as a circle in every base slab whose middle lies within the stretch above the tip.
            const coord_t bottom = object.layer(node.layer).scaled_print_z(), top = object.layer(node.layer + 1).scaled_print_z();
            size_t drawn = 0;
            for (const Common::GeneratorLayer *slab : slabs.support.base) {
                const coord_t middle = slab->bottom_z + (slab->print_z - slab->bottom_z) / 2;
                if (middle < bottom || middle > top)
                    continue;
                ++drawn;
                CHECK(distance_to_region_mm(to_point(node.position), SG::native(slab->polygons)) <= tolerance_mm);
            }
            CHECK(drawn > 0);
        }
    REQUIRE(tips > 0);
}

// A root over free air extends down to the object layer that takes its unsupported part: its bottom contact lies on that layer,
// and the stem of its branch runs from that layer up to the root.
TEST_CASE("Tree roots over free air extend down to the layer they rest on", "[plugins][support-tree-family][floors]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    TriangleMesh block = make_cube(10., 10., 4.);
    TriangleMesh column = make_cube(2., 10., 7.);
    column.translate(50., 0., 0.);
    block.merge(column);
    DynamicPrintConfig config = SG::config("tree", false, 0);
    Print print;
    Model model;
    place_print({ { block, Vec3d(100., 100., 0.), "" } }, config, print, model);
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    const TF::TreeSettings settings = Slic3r::Test::SupportTree::tree_settings(print, object, SG::params(print, object));
    const auto layer_at = [&object](double z_mm) {
        size_t id = 0;
        while (id + 1 < object.layer_count() && std::abs(object.layer(id).unscaled_print_z() - z_mm) > 0.01)
            ++id;
        return id;
    };
    const size_t block_top = layer_at(4.), root_layer = layer_at(6.);
    REQUIRE(root_layer > block_top + 2);
    const ExPolygons block_slices = object.layer(block_top).lslices();
    const ExPolygon &block_slice = *std::max_element(block_slices.begin(), block_slices.end(),
        [](const ExPolygon &a, const ExPolygon &b) { return a.area() < b.area(); });
    const Point centre = get_extents(block_slice).center();
    CAPTURE(block_top, root_layer, centre.x(), centre.y(), settings.branch_radius, settings.layer_height);
    TF::TreeNode upper;
    upper.layer = root_layer + 1;
    upper.print_z = object.layer(root_layer + 1).unscaled_print_z();
    upper.position = to_c_point(centre);
    upper.radius = settings.branch_radius;
    TF::TreeNode root;
    root.layer = root_layer;
    root.print_z = object.layer(root_layer).unscaled_print_z();
    root.position = to_c_point(centre);
    root.radius = settings.branch_radius;
    root.carries = { 0 };
    PluginStorage storage;
    Common::GeneratedSupport support{ reinterpret_cast<storage_handle *>(&storage) };
    std::vector<TF::RoutedTree> trees{ TF::RoutedTree{ { upper, root } } };
    TF::generate_tree_slabs(nullptr, print, object, trees, false, support);
    const coord_t block_z = object.layer(block_top).scaled_print_z(), root_z = object.layer(root_layer).scaled_print_z();
    REQUIRE_FALSE(support.bottom.empty());
    for (const Common::GeneratorLayer *contact : support.bottom) {
        CHECK(contact->print_z > block_z);
        CHECK(contact->print_z < root_z);
        CHECK(distance_to_region_mm(centre, SG::native(contact->polygons)) <= tolerance_mm);
    }
}

// A slab bound within the layer merge tolerance of a layer of another object takes that layer's Z: the tool ordering asserts on
// distinct print Z values that close, so no support Z may lie that close to an object layer Z without being equal to it.
TEST_CASE("Tree slab bounds join the print Z of the object layers of another object", "[plugins][support-tree-family][slabs]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = SG::config("tree", false, 0);
    config.set_deserialize_strict({{"z_step", "0"}});
    const coord_t first = scale_to_layer_coord(0.2);
    const auto print_zs = [](const Print &print) {
        std::vector<coord_t> zs;
        for (const PrintObject &object : print.objects())
            for (size_t id = 0; id < object.layer_count(); ++id)
                zs.push_back(object.layer(id).scaled_print_z());
        std::sort(zs.begin(), zs.end());
        zs.erase(std::unique(zs.begin(), zs.end()), zs.end());
        return zs;
    };
    const auto distant = [](const std::vector<coord_t> &zs, coord_t z) {
        return std::all_of(zs.begin(), zs.end(), [z](coord_t other) { return std::abs(other - z) > 1000; });
    };
    // The bounds of the support of the slab object alone. The second object gets uniform layers from the first layer, one of
    // which lies a few hundred nanometres above a bound; the other layers keep clear of the first object.
    Print probe;
    Model probe_model;
    place_print({ { SG::slab(true), Vec3d(100., 100., 0.), "" } }, config, probe, probe_model);
    SG::prefix(probe);
    const std::vector<coord_t> own_zs = print_zs(probe);
    std::vector<coord_t> bounds;
    {
        const TreeSlabs slabs(probe, false);
        for (const Common::GeneratorLayer *slab : slabs.support.base)
            bounds.push_back(slab->print_z);
    }
    std::sort(bounds.begin(), bounds.end());
    coord_t bound = 0, height = 0, count = 0;
    for (const coord_t candidate : bounds) {
        constexpr coord_t gap = 256;
        if (candidate < scale_to_layer_coord(0.5) || candidate > scale_to_layer_coord(4.) || !distant(own_zs, candidate))
            continue;
        const coord_t span = candidate + gap - first;
        const coord_t steps = (span + scale_to_layer_coord(0.3) - 1) / scale_to_layer_coord(0.3);
        const coord_t step = span / steps;
        bool clear = true;
        for (coord_t layer = 1; layer <= steps; ++layer)
            clear = clear && distant(own_zs, first + layer * step);
        if (clear) {
            bound = candidate;
            height = step;
            count = steps;
            break;
        }
    }
    REQUIRE(bound != 0);
    const coord_t target = first + count * height;
    TriangleMesh second = make_cube(10., 10., unscaled(target) + 0.5);
    Print print;
    Model model;
    place_print({ { SG::slab(true), Vec3d(100., 100., 0.), "" }, { second, Vec3d(160., 100., 0.), std::to_string(unscaled(height)) } },
        config, print, model);
    SG::prefix(print);
    const std::vector<coord_t> zs = print_zs(print);
    const TreeSlabs slabs(print, false);
    for (const Common::GeneratorLayer &slab : slabs.support.slabs) {
        const auto clear_of_objects = [&zs](coord_t z) {
            return std::all_of(zs.begin(), zs.end(), [z](coord_t other) { return other == z || std::abs(other - z) > 4 * SCALED_EPSILON; });
        };
        CHECK(clear_of_objects(slab.print_z));
        CHECK(clear_of_objects(slab.bottom_z));
    }
    const auto snapped = std::min_element(zs.begin(), zs.end(),
        [bound](coord_t a, coord_t b) { return std::abs(a - bound) < std::abs(b - bound); });
    REQUIRE(std::abs(*snapped - bound) <= 4 * SCALED_EPSILON);
    const TF::TreeSettings settings = Slic3r::Test::SupportTree::tree_settings(print, print.object(0), SG::params(print, print.object(0)));
    std::vector<const Common::GeneratorLayer *> base(slabs.support.base.begin(), slabs.support.base.end());
    std::sort(base.begin(), base.end(), [](const Common::GeneratorLayer *a, const Common::GeneratorLayer *b) { return a->print_z < b->print_z; });
    size_t joined = 0;
    for (size_t idx = 0; idx + 1 < base.size(); ++idx) {
        if (base[idx]->print_z != *snapped)
            continue;
        ++joined;
        CHECK(base[idx + 1]->bottom_z == *snapped);
        CHECK(base[idx]->print_z - base[idx]->bottom_z >= settings.min_slab_height - 4 * SCALED_EPSILON);
        CHECK(base[idx + 1]->print_z - base[idx + 1]->bottom_z >= settings.min_slab_height - 4 * SCALED_EPSILON);
    }
    CHECK(joined > 0);
}

// Two bounds within the layer merge tolerance of one object top both snap onto it, which collapses the slab between them. Its
// polygons join the slab below it, so the published layers stay on the layer grid with positive heights, keep clear of the close
// print Z values of the object, and still print the collapsed area as sparse support.
TEST_CASE("Slab bounds that snap onto one object top keep their slabs printable", "[plugins][support-tree-family][slabs]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    Test::init_print({ SG::slab() }, print, model, SG::config("tree", false, 0));
    SG::prefix(print);
    PrintObject &object = print.object(0);
    std::vector<coord_t> object_zs;
    for (size_t id = 0; id < object.layer_count(); ++id)
        object_zs.push_back(object.layer(id).scaled_print_z());
    const coord_t top = scale_to_layer_coord(1.6);
    REQUIRE(std::find(object_zs.begin(), object_zs.end(), top) != object_zs.end());
    const coord_t below = scale_to_layer_coord(1.6001), above = scale_to_layer_coord(1.6004), roof = scale_to_layer_coord(2.);
    PluginStorage storage;
    Common::GeneratedSupport support{ reinterpret_cast<storage_handle *>(&storage) };
    Common::GeneratorLayer &lower = Common::add_layer(support, Common::LayerType::Intermediate, below, 0, below);
    Common::GeneratorLayer &thin = Common::add_layer(support, Common::LayerType::Intermediate, above, below, above - below);
    Common::GeneratorLayer &upper = Common::add_layer(support, Common::LayerType::Intermediate, roof, above, roof - above);
    support.base = { &lower, &thin, &upper };
    const Point centre(coord_t(0), scale_i(3.));
    TF::native_polygons(lower.polygons) = Polygons{ make_circle(scale_d(2.), 1000.) };
    Polygon thin_circle = make_circle(scale_d(0.5), 1000.);
    thin_circle.translate(centre);
    TF::native_polygons(thin.polygons) = Polygons{ thin_circle };
    TF::native_polygons(upper.polygons) = Polygons{ make_circle(scale_d(1.), 1000.) };
    TF::align_slab_bounds(object_zs, support);
    const auto on_grid = [](coord_t z) { return scale_to_layer_coord(unscaled(z)) == z; };
    REQUIRE(support.base.size() == 2);
    for (const Common::GeneratorLayer *slab : support.base) {
        REQUIRE(slab->height > 0);
        REQUIRE(on_grid(slab->print_z));
        REQUIRE(on_grid(slab->bottom_z));
    }
    CHECK(lower.print_z == top);
    CHECK(upper.bottom_z == top);
    Common::publish(nullptr, SG::object_view(object), support, reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
    const SurfaceType sparse_type = stModSupport | stPosInternal | stDensSparse;
    bool thin_published = false;
    for (const Layer &layer : object.auxiliary_layers()) {
        CHECK(on_grid(layer.scaled_print_z()));
        CHECK(layer.scaled_height() > 0);
        for (const coord_t z : object_zs)
            CHECK((z == layer.scaled_print_z() || std::abs(z - layer.scaled_print_z()) > 4 * SCALED_EPSILON));
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands())
                for (const Surface &surface : region.fill_surfaces())
                    thin_published = thin_published || (surface.surface_type == sparse_type && surface.expolygon.contains(centre));
    }
    CHECK(thin_published);
}

// Two independent support zones of one object: the bottom contact of a floor ends at 1.1 mm and the top contact of an overhang
// starts at 1.1004 mm, 400 scaled units higher, with no object layer near either. Publication would print them at two Z that the
// tool ordering refuses, so their bounds must share one Z before publication.
TEST_CASE("Support bounds of one object closer than the merge tolerance share one print Z", "[plugins][support-tree-family][slabs]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    Test::init_print({ SG::slab() }, print, model, SG::config("tree", false, 0));
    SG::prefix(print);
    PrintObject &object = print.object(0);
    std::vector<coord_t> object_zs;
    for (size_t id = 0; id < object.layer_count(); ++id)
        object_zs.push_back(object.layer(id).scaled_print_z());
    PluginStorage storage;
    Common::GeneratedSupport support{ reinterpret_cast<storage_handle *>(&storage) };
    const coord_t floor_top = scale_to_layer_coord(1.1), floor_bottom = scale_to_layer_coord(0.8);
    const coord_t contact_top = scale_to_layer_coord(1.1004), contact_bottom = scale_to_layer_coord(0.9004);
    Common::GeneratorLayer &floor_contact = Common::add_layer(support, Common::LayerType::BottomContact, floor_top, floor_bottom,
        floor_top - floor_bottom);
    Common::GeneratorLayer &overhang_contact = Common::add_layer(support, Common::LayerType::TopContact, contact_top, contact_bottom,
        contact_top - contact_bottom);
    support.bottom = { &floor_contact };
    support.top = { &overhang_contact };
    const Point floor_centre(scale_i(-4.), coord_t(0)), overhang_centre(scale_i(4.), coord_t(0));
    Polygon floor_circle = make_circle(scale_d(1.), 1000.);
    floor_circle.translate(floor_centre);
    Polygon overhang_circle = make_circle(scale_d(1.), 1000.);
    overhang_circle.translate(overhang_centre);
    TF::native_polygons(floor_contact.polygons) = Polygons{ floor_circle };
    TF::native_polygons(overhang_contact.polygons) = Polygons{ overhang_circle };
    TF::align_slab_bounds(object_zs, support);
    Common::publish(nullptr, SG::object_view(object), support, reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
    SG::toolpaths(print);
    REQUIRE(object.auxiliary_layers().size() >= 2);
    std::vector<coord_t> zs = object_zs;
    for (const Layer &layer : object.auxiliary_layers()) {
        REQUIRE(layer.has_extrusions());
        zs.push_back(layer.scaled_print_z());
    }
    std::sort(zs.begin(), zs.end());
    zs.erase(std::unique(zs.begin(), zs.end()), zs.end());
    for (size_t idx = 0; idx + 1 < zs.size(); ++idx)
        REQUIRE(zs[idx + 1] - zs[idx] > 4 * SCALED_EPSILON);
    const ToolOrdering ordering(object, EXTRUDER_ID_UNKNOWN);
    CHECK(ordering.tools_for_layer(scale_to_layer_coord(1.1)) != nullptr);
    bool floor_printed = false, overhang_printed = false;
    for (const Layer &layer : object.auxiliary_layers()) {
        if (layer.scaled_print_z() != floor_top)
            continue;
        for (const ExPolygon &expolygon : layer.lslices()) {
            floor_printed = floor_printed || expolygon.contains(floor_centre);
            overhang_printed = overhang_printed || expolygon.contains(overhang_centre);
        }
    }
    CHECK(floor_printed);
    CHECK(overhang_printed);
}

namespace {
// The first layer whose print Z is the given height in millimeters, or the top layer when none is.
size_t layer_at_height(const PrintObject &object, double z_mm)
{
    size_t id = 0;
    while (id + 1 < object.layer_count() && std::abs(object.layer(id).unscaled_print_z() - z_mm) > 0.01)
        ++id;
    return id;
}
} // namespace

// A bottom contact stands above the object with a gap, and the branch above it starts on its top. Where a base slab and the contact
// share their top Z with other heights, the slab must not print over the contact footprint, or that area would be printed twice.
TEST_CASE("Tree floors keep the gap and the contact footprint free of branch surfaces", "[plugins][support-tree-family][floors]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    TriangleMesh block = make_cube(10., 10., 4.);
    TriangleMesh column = make_cube(2., 10., 7.);
    column.translate(50., 0., 0.);
    block.merge(column);
    DynamicPrintConfig config = SG::config("tree", false, 0);
    Print print;
    Model model;
    place_print({ { block, Vec3d(100., 100., 0.), "" } }, config, print, model);
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    const TF::TreeSettings settings = Slic3r::Test::SupportTree::tree_settings(print, object, SG::params(print, object));
    const ExPolygons block_slices = object.layer(layer_at_height(object, 4.)).lslices();
    const ExPolygon &block_slice = *std::max_element(block_slices.begin(), block_slices.end(),
        [](const ExPolygon &a, const ExPolygon &b) { return a.area() < b.area(); });
    const Point centre = get_extents(block_slice).center();
    const size_t root_layer = layer_at_height(object, 6.);
    TF::TreeNode upper;
    upper.layer = root_layer + 1;
    upper.print_z = object.layer(root_layer + 1).unscaled_print_z();
    upper.position = to_c_point(centre);
    upper.radius = settings.branch_radius;
    TF::TreeNode root;
    root.layer = root_layer;
    root.print_z = object.layer(root_layer).unscaled_print_z();
    root.position = to_c_point(centre);
    root.radius = settings.branch_radius;
    root.carries = { 0 };
    PluginStorage storage;
    Common::GeneratedSupport support{ reinterpret_cast<storage_handle *>(&storage) };
    std::vector<TF::RoutedTree> trees{ TF::RoutedTree{ { upper, root } } };
    TF::generate_tree_slabs(nullptr, print, object, trees, false, support);
    REQUIRE_FALSE(support.bottom.empty());
    const Common::GeneratorLayer &floor = *support.bottom.front();
    const Polygons footprint = SG::native(floor.polygons);
    REQUIRE_FALSE(footprint.empty());
    const coord_t object_top = object.layer(floor.object_below).scaled_print_z();
    for (const Common::GeneratorLayer *slab : support.base) {
        const Polygons &area = SG::native(slab->polygons);
        if (area.empty() || slab->print_z <= object_top || slab->bottom_z >= floor.print_z)
            continue;
        CAPTURE(slab->bottom_z, slab->print_z, slab->height, floor.bottom_z, floor.print_z, floor.height);
        CHECK(intersection(area, footprint).empty());
        if (slab->print_z == floor.print_z)
            CHECK(slab->height == floor.height);
    }
}

// A floor under an organic foot: the foot's half sphere reaches into the Z span of the floor. A contact of another overhang that has
// the print Z of that floor, with another thickness, samples the foot. The contact must leave the footprint of the floor: the
// published layers that overlap the floor share no area with the floor layer.
TEST_CASE("Tip contacts keep out of the footprints of floors that share their print Z", "[plugins][support-tree-family][floors]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    TriangleMesh block = make_cube(10., 10., 4.);
    TriangleMesh column = make_cube(2., 10., 7.);
    column.translate(50., 0., 0.);
    block.merge(column);
    Print print;
    Model model;
    place_print({ { block, Vec3d(100., 100., 0.), "" } }, SG::config("organic", false, 0), print, model);
    SG::prefix(print);
    PrintObject &object = print.object(0);
    const TF::TreeSettings settings = Slic3r::Test::SupportTree::tree_settings(print, object, SG::params(print, object));
    const size_t foot_layer = layer_at_height(object, 4.4), tip_layer = layer_at_height(object, 4.6);
    const ExPolygons block_slices = object.layer(layer_at_height(object, 4.)).lslices();
    const ExPolygon &block_slice = *std::max_element(block_slices.begin(), block_slices.end(),
        [](const ExPolygon &a, const ExPolygon &b) { return a.area() < b.area(); });
    const Point centre = get_extents(block_slice).center();
    const auto make_trees = [&]() {
        TF::TreeNode upper;
        upper.layer = tip_layer;
        upper.print_z = object.layer(tip_layer).unscaled_print_z();
        upper.position = to_c_point(centre);
        upper.radius = settings.branch_radius;
        TF::TreeNode foot;
        foot.layer = foot_layer;
        foot.print_z = object.layer(foot_layer).unscaled_print_z();
        foot.position = to_c_point(centre);
        foot.radius = settings.branch_radius;
        foot.carries = { 0 };
        TF::RoutedTree pillar;
        for (size_t layer = tip_layer + 1; layer-- > 0;) {
            TF::TreeNode node;
            node.layer = layer;
            node.print_z = object.layer(layer).unscaled_print_z();
            node.position = to_c_point(Point::new_scale(30., 30.));
            node.radius = settings.branch_radius;
            if (layer < tip_layer)
                node.carries = { pillar.nodes.size() - 1 };
            pillar.nodes.push_back(node);
        }
        return std::vector<TF::RoutedTree>{ TF::RoutedTree{ { upper, foot } }, pillar };
    };
    PluginStorage probe_storage;
    Common::GeneratedSupport probe{ reinterpret_cast<storage_handle *>(&probe_storage) };
    std::vector<TF::RoutedTree> probe_trees = make_trees();
    TF::generate_tree_slabs(nullptr, print, object, probe_trees, true, probe);
    REQUIRE(probe.bottom.size() == 1);
    const coord_t floor_top = probe.bottom.front()->print_z, floor_height = probe.bottom.front()->height;
    REQUIRE(unscaled(floor_top) == Approx(4.4));
    PluginStorage storage;
    Common::GeneratedSupport support{ reinterpret_cast<storage_handle *>(&storage) };
    Common::GeneratorLayer &other = Common::add_layer(support, Common::LayerType::TopContact, floor_top,
        floor_top - floor_height - scale_to_layer_coord(0.1), floor_height + scale_to_layer_coord(0.1));
    support.top = { &other };
    std::vector<TF::RoutedTree> trees = make_trees();
    TF::generate_tree_slabs(nullptr, print, object, trees, true, support);
    Common::publish(nullptr, SG::object_view(object), support, reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
    const auto surfaces_of = [](const Layer &layer) {
        ExPolygons surfaces;
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands())
                for (const Surface &surface : region.fill_surfaces())
                    surfaces.push_back(surface.expolygon);
        return surfaces;
    };
    const Layer *floor_layer = nullptr;
    for (const Layer &layer : object.auxiliary_layers())
        if (layer.scaled_print_z() == floor_top && layer.scaled_height() == floor_height)
            floor_layer = &layer;
    REQUIRE(floor_layer != nullptr);
    const ExPolygons floor_surfaces = surfaces_of(*floor_layer);
    size_t overlaps = 0;
    for (const Layer &layer : object.auxiliary_layers()) {
        if (&layer == floor_layer)
            continue;
        const coord_t shared_bottom = std::max(layer.scaled_bottom_z(), floor_layer->scaled_bottom_z());
        const coord_t shared_top = std::min(layer.scaled_print_z(), floor_layer->scaled_print_z());
        if (shared_top <= shared_bottom)
            continue;
        ++overlaps;
        const double shared_mm2 = unscaled(unscaled(area(intersection_ex(surfaces_of(layer), floor_surfaces))));
        CAPTURE(layer.scaled_bottom_z(), layer.scaled_print_z(), shared_mm2);
        CHECK(shared_mm2 <= 1e-4);
    }
    REQUIRE(overlaps > 0);
}

// Organic branches are published as branch surfaces, which the branch sheath prints, and tree branches as plain sparse surfaces.
TEST_CASE("Organic branches publish branch surfaces and tree branches plain sparse surfaces", "[plugins][support-tree-family][publication]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool organic = GENERATE(false, true);
    CAPTURE(organic);
    Print print;
    Model model;
    Test::init_print({SG::slab()}, print, model, SG::config(organic ? "organic" : "tree", false, 3));
    SG::prefix(print);
    SG::family(print);
    REQUIRE_FALSE(print.object(0).auxiliary_layers().empty());
    const SurfaceType branch_type = stModSupport | stModBranch | stPosInternal | stDensSparse;
    const SurfaceType sparse_type = stModSupport | stPosInternal | stDensSparse;
    size_t branch = 0, sparse = 0;
    for (const Layer &layer : print.object(0).auxiliary_layers())
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands())
                for (const Surface &surface : region.fill_surfaces()) {
                    branch += surface.surface_type == branch_type;
                    sparse += surface.surface_type == sparse_type;
                }
    if (organic) {
        CHECK(branch > 0);
        CHECK(sparse == 0);
    } else {
        CHECK(branch == 0);
        CHECK(sparse > 0);
    }
}

// A print with tree or organic support exports the support paths, and the interface paths only when interface layers are set.
TEST_CASE("Tree and organic prints export support and interface paths", "[plugins][support-tree-family][gcode]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *style = GENERATE("tree", "organic");
    const int interfaces = GENERATE(0, 3);
    CAPTURE(style, interfaces);
    DynamicPrintConfig config = Slic3r::Test::SupportGrid::config(style, false, interfaces, true);
    config.set_deserialize_strict({
        {"step_gcode_plugin", "gcode.printing_plan_file_writer"},
        {"gcode_firmware_plugin", "gcode.firmware.marlin2"},
        {"support_material_speed", "40"}, {"support_material_interface_speed", "50%"},
        {"max_print_speed", "80"}, {"first_layer_speed", "100%"}, {"first_layer_min_speed", "0"},
        {"max_volumetric_speed", "0"}, {"filament_max_speed", "0,0"},
        {"filament_max_volumetric_speed", "0,0"}, {"slowdown_below_layer_time", "0,0"},
        {"use_relative_e_distances", "1"}, {"wipe", "0,0"}, {"filament_diameter", "1.75,1.75"}
    });
    Model model;
    Print print;
    Slic3r::Test::init_print({Slic3r::Test::SupportGrid::slab(true)}, print, model, config);
    const boost::filesystem::path path = Slic3r::Test::GCode::temporary_gcode_path();
    std::string code;
    try {
        print.process();
        Orchestrator::instance().export_gcode(print, path.string(), nullptr, nullptr);
        code = Slic3r::Test::GCode::read_text_file(path);
    } catch (...) {
        Slic3r::Test::GCode::remove_output_pair(path);
        throw;
    }
    Slic3r::Test::GCode::remove_output_pair(path);
    std::set<std::string> types;
    std::istringstream stream(code);
    for (std::string line; std::getline(stream, line);)
        if (line.rfind(";TYPE:", 0) == 0)
            types.insert(line);
    CAPTURE(types);
    CHECK(types.count(";TYPE:Support material") == 1);
    CHECK(types.count(";TYPE:Support material interface") == (interfaces > 0 ? 1u : 0u));
}

// The family declares its geometry and tree branch keys, and it warns once for a requested raft without generating raft layers.
TEST_CASE("Tree family declares its keys and warns once for a raft", "[plugins][support-tree-family][family]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    REQUIRE(Orchestrator::instance().is_plugin_active("support.family.tree"));
    const Plugin &provider = *Orchestrator::instance().get_plugin("support.family.tree");
    for (const char *expected_key : {"support_tree_max_overhang", "support_tree_branch_diameter", "support_tree_top_rate",
             "support_material_interface_layers", "support_material_buildplate_only"}) {
        CAPTURE(expected_key);
        CHECK(std::any_of(provider.get_used_config_keys().begin(), provider.get_used_config_keys().end(),
            [expected_key](const Plugin::UsedConfigKey &key) { return key.key == expected_key; }));
    }
    DynamicPrintConfig config = SG::config("tree");
    config.set_deserialize_strict({{"raft_layers", "3"}});
    Print print;
    Model model;
    Test::init_print({SG::slab()}, print, model, config);
    SG::prefix(print);
    Orchestrator::instance().consume_plugin_messages();
    SG::family(print);
    REQUIRE_FALSE(print.object(0).auxiliary_layers().empty());
    size_t warnings = 0;
    for (const Orchestrator::PluginMessage &message : Orchestrator::instance().consume_plugin_messages())
        if (message.plugin_id == "support.family.tree" && message.level == Orchestrator::PluginMessageLevel::Warning) ++warnings;
    CHECK(warnings == 1);
}

// The outline of every layer of the object grown by the XY distance, which the branches keep clear of.
namespace {
std::vector<Polygons> keep_out_outlines(const PrintObject &object, coord_t gap_xy)
{
    std::vector<Polygons> outlines;
    outlines.reserve(object.layer_count());
    for (size_t id = 0; id < object.layer_count(); ++id)
        outlines.push_back(to_polygons(offset_ex(object.layer(id).lslices(), float(gap_xy), ClipperLib::jtRound)));
    return outlines;
}

// The volume in cubic millimeters of the base slabs: the area of each slab times its height.
double base_volume_mm3(const std::vector<BaseSlab> &slabs)
{
    double volume = 0.;
    for (const BaseSlab &slab : slabs)
        volume += unscaled(unscaled(area(slab.polygons))) * unscaled(slab.print_z - slab.bottom_z);
    return volume;
}
// The tolerance of the drawn checks in millimeters: the sections are measured where they are printed, so only rounding is left.
constexpr double section_tolerance_mm = 0.002;

// The middle Z of a base slab, in scaled units, where its branches are drawn.
coord_t middle_z(const BaseSlab &slab)
{
    return slab.bottom_z + (slab.print_z - slab.bottom_z) / 2;
}

// The number of gaps in a stack of base slabs, bottom first. Two neighbouring slabs that hold branches must overlap. A run of empty
// slabs between two slabs that hold branches is a gap when they overlap, since the branch is then present below and above the run.
size_t branch_gaps(const std::vector<BaseSlab> &slabs)
{
    size_t gaps = 0;
    size_t previous = slabs.size();
    for (size_t idx = 0; idx < slabs.size(); ++idx) {
        if (slabs[idx].polygons.empty())
            continue;
        if (previous < slabs.size()) {
            const bool overlap = !intersection(slabs[previous].polygons, slabs[idx].polygons).empty();
            const bool run = idx > previous + 1;
            if (run ? overlap : !overlap)
                ++gaps;
        }
        previous = idx;
    }
    return gaps;
}

// A square with the given corner and side, in millimeters.
Polygons square_mm(double x, double y, double side)
{
    Polygon polygon;
    polygon.points = { Point::new_scale(x, y), Point::new_scale(x + side, y), Point::new_scale(x + side, y + side), Point::new_scale(x, y + side) };
    return Polygons{ polygon };
}
// A node of a chain in millimeters: its XY position and the object layer it lies on.
struct ChainPoint {
    double x_mm, y_mm;
    size_t layer;
};
// A chain of nodes listed from the tip down to the root, as RoutedTree lists its nodes: each node after the first carries the
// node before it, the one above it. The tip and the root are locked, and the nodes between them are free.
TF::RoutedTree chain_tree(const PrintObject &object, const std::vector<ChainPoint> &points)
{
    TF::RoutedTree tree;
    for (size_t idx = 0; idx < points.size(); ++idx) {
        TF::TreeNode node;
        node.layer = points[idx].layer;
        node.print_z = object.layer(node.layer).unscaled_print_z();
        node.position = to_c_point(Point::new_scale(points[idx].x_mm, points[idx].y_mm));
        node.radius = scale_to_layer_coord(0.4);
        if (idx > 0)
            node.carries = { idx - 1 };
        tree.nodes.push_back(node);
    }
    return tree;
}
// A node in millimeters: its XY position and its print Z.
Vec3d node_mm(const TF::TreeNode &node)
{
    return Vec3d(unscaled(node.position.x), unscaled(node.position.y), node.print_z);
}
// The sharpest turning angle of a tree in degrees: at each node with one node below and one above, the angle between the
// segment from the node below and the segment to the node above.
double sharpest_bend_deg(const TF::RoutedTree &tree)
{
    const std::vector<size_t> below = TF::nodes_below(tree);
    double sharpest = 0.;
    for (size_t idx = 0; idx < tree.nodes.size(); ++idx) {
        const TF::TreeNode &node = tree.nodes[idx];
        if (below[idx] == TF::no_node || node.carries.size() != 1)
            continue;
        const Vec3d in = node_mm(node) - node_mm(tree.nodes[below[idx]]);
        const Vec3d out = node_mm(tree.nodes[node.carries.front()]) - node_mm(node);
        const double cosine = std::clamp(in.dot(out) / (in.norm() * out.norm()), -1., 1.);
        sharpest = std::max(sharpest, std::acos(cosine) * 180. / M_PI);
    }
    return sharpest;
}
// A bent branch of five nodes from a tip at (0, 0) mm to a root at (8, 0) mm: the bend sits at the middle node.
TF::RoutedTree bent_branch(const PrintObject &object)
{
    return chain_tree(object, { { 0., 0., 20 }, { 0., 0., 16 }, { 0., 0., 12 }, { 4., 0., 8 }, { 8., 0., 4 } });
}
// A branch whose only free node stands 25 mm off the line of its tip and its root.
TF::RoutedTree stray_node_branch(const PrintObject &object)
{
    return chain_tree(object, { { 0., 0., 12 }, { 25., 0., 8 }, { 0., 0., 4 } });
}
} // namespace

// A sphere whose circle reaches d into the outline is pushed out by min(d + the gap, the cap) in one iteration: its center moves
// by that distance along the normal of the outline. The gap is 0.1 mm and the cap 0.5 mm, the values of TreeSmoothing.cpp.
TEST_CASE("Organic push moves a penetrating sphere by the nudge distance in one iteration", "[plugins][support-tree-organic][smoothing]")
{
    const double radius_mm = 0.5, gap_mm = 0.1, cap_mm = 0.5;
    const double penetration_mm = GENERATE(0.1, 0.3, 0.45);
    CAPTURE(penetration_mm);
    const std::vector<double> print_z{ 0.2 };
    const std::vector<Polygons> keep_out{ square_mm(0., 0., 10.) };
    const Vec2d center(10. + radius_mm - penetration_mm, 5.);
    const Vec2d pushed = TF::push_sphere_out_of_outline(0, radius_mm, center, print_z, keep_out);
    const double expected_mm = std::min(penetration_mm + gap_mm, cap_mm);
    CAPTURE(pushed.x(), pushed.y(), expected_mm);
    CHECK(std::abs((pushed.x() - center.x()) - expected_mm) <= tolerance_mm);
    CHECK(std::abs(pushed.y() - center.y()) <= tolerance_mm);
}

// Smoothing moves the free nodes of a bent organic branch, and no node ends up inside the object outline grown by the XY
// distance. A node with a radius keeps its circle clear of the outline, within the tolerance.
TEST_CASE("Organic smoothing moves the nodes of a bent branch and keeps them out of the object",
    "[plugins][support-tree-organic][smoothing]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = SG::config("organic", false, 0);
    config.set_deserialize_strict({{"support_material_buildplate_only", "1"}, {"support_tree_max_overhang", "0.2"}});
    Print print;
    Model model;
    Test::init_print({diagonal_roof()}, print, model, config);
    SG::prefix(print);
    const Routed routed(print);
    const PrintObject &object = print.object(0);
    const std::vector<Polygons> keep_out = keep_out_outlines(object, routed.settings.xy_min_distance);
    std::vector<TF::RoutedTree> smoothed = routed.trees;
    TF::smooth_organic_branches(object, keep_out, smoothed, [] {});
    size_t moved = 0;
    double largest_move_mm = 0.;
    for (size_t tree_id = 0; tree_id < smoothed.size(); ++tree_id) {
        const std::vector<size_t> below = TF::nodes_below(routed.trees[tree_id]);
        for (size_t idx = 0; idx < smoothed[tree_id].nodes.size(); ++idx) {
            const TF::TreeNode &raw = routed.trees[tree_id].nodes[idx];
            const TF::TreeNode &node = smoothed[tree_id].nodes[idx];
            if (to_point(node.position) != to_point(raw.position)) {
                ++moved;
                largest_move_mm = std::max(largest_move_mm, unscaled((to_point(node.position) - to_point(raw.position)).cast<double>().norm()));
            }
            const double clearance_mm = distance_to_region_mm(to_point(node.position), keep_out[node.layer]);
            const bool free = !node.carries.empty() && (below[idx] != TF::no_node || node.layer == 0);
            CAPTURE(tree_id, idx, node.layer, clearance_mm);
            CHECK(clearance_mm > 0.);
            if (free)
                CHECK(clearance_mm >= unscaled(node.radius) - tolerance_mm);
        }
    }
    CAPTURE(moved, largest_move_mm);
    CHECK(moved > 0);
    CHECK(largest_move_mm > 0.01);
}

// A bent branch that no sphere touches is relaxed until its bend settles: its free nodes move, and its sharpest bend falls to at
// most half of its value.
TEST_CASE("Organic relaxation smooths a collision-free bent branch until its bend settles", "[plugins][support-tree-organic][smoothing]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    Test::init_print({SG::slab()}, print, model, SG::config("organic", false, 0));
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    const std::vector<Polygons> keep_out(object.layer_count(), square_mm(-30., -30., 10.));
    std::vector<TF::RoutedTree> trees{ bent_branch(object) };
    // Guard: the fixture starts clear of the outline, so the branch is collision-free.
    for (const TF::TreeNode &node : trees.front().nodes)
        REQUIRE(distance_to_region_mm(to_point(node.position), keep_out[node.layer]) > unscaled(node.radius));
    const std::vector<TF::TreeNode> raw = trees.front().nodes;
    const double bend_before = sharpest_bend_deg(trees.front());
    const size_t iterations = TF::smooth_organic_branches(object, keep_out, trees, [] {});
    const double bend_after = sharpest_bend_deg(trees.front());
    size_t moved = 0;
    for (size_t idx = 0; idx < raw.size(); ++idx)
        moved += to_point(trees.front().nodes[idx].position) != to_point(raw[idx].position) ? 1 : 0;
    CAPTURE(iterations, bend_before, bend_after, moved);
    CHECK(moved == 3);
    CHECK(bend_after <= 0.5 * bend_before);
}

// The relaxation stops once a pass moves no node more than 0.01 mm, and never runs more than 100 iterations: a node that keeps
// moving by the step cap runs to the cap.
TEST_CASE("Organic relaxation stops once the nodes settle and never runs past its iteration cap", "[plugins][support-tree-organic][smoothing]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    Test::init_print({SG::slab()}, print, model, SG::config("organic", false, 0));
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    const std::vector<Polygons> keep_out(object.layer_count(), square_mm(-30., -30., 10.));
    // The count is exact: the relaxation is deterministic, so a change of the threshold or of the rule moves it.
    SECTION("a bend that settles stops after its exact count") {
        std::vector<TF::RoutedTree> trees{ bent_branch(object) };
        const size_t iterations = TF::smooth_organic_branches(object, keep_out, trees, [] {});
        CAPTURE(iterations);
        CHECK(iterations == 33);
    }
    SECTION("a node that keeps moving runs to the cap") {
        std::vector<TF::RoutedTree> trees{ stray_node_branch(object) };
        const size_t iterations = TF::smooth_organic_branches(object, keep_out, trees, [] {});
        const double x_mm = unscaled(trees.front().nodes[1].position.x);
        CAPTURE(iterations, x_mm);
        CHECK(iterations == 100);
        CHECK(std::abs(x_mm - 5.) <= tolerance_mm);
    }
}

// An exception from the cancel callback ends the relaxation before anything is written back, so the trees keep their positions.
TEST_CASE("Organic relaxation leaves the trees unchanged when the cancel callback throws", "[plugins][support-tree-organic][smoothing]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    Test::init_print({SG::slab()}, print, model, SG::config("organic", false, 0));
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    const std::vector<Polygons> keep_out(object.layer_count(), square_mm(-30., -30., 10.));
    std::vector<TF::RoutedTree> trees{ bent_branch(object) };
    const std::vector<TF::TreeNode> raw = trees.front().nodes;
    REQUIRE_THROWS_AS(TF::smooth_organic_branches(object, keep_out, trees, [] { throw std::runtime_error("cancelled"); }),
        std::runtime_error);
    for (size_t idx = 0; idx < raw.size(); ++idx)
        CHECK(to_point(trees.front().nodes[idx].position) == to_point(raw[idx].position));
}

// The branch meshes of the organic style fill the branches that the circles of the tree style fill. Their volume stays within a
// quarter of the circles: a tilted tube is cut into wider sections than its circles, and the end caps close the tubes. No branch is
// lost between the slabs, so the branches stay connected from the contact to the bed (branch_gaps()).
TEST_CASE("Organic branch meshes fill the volume of the tree circles and stay connected", "[plugins][support-tree-organic][drawing]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    Test::init_print({SG::slab()}, print, model, SG::config("organic", false, 0));
    SG::prefix(print);
    const TreeSlabs circles(print, false);
    const TreeSlabs meshes(print, true);
    const std::vector<BaseSlab> circle_base = base_slabs(circles.support);
    const std::vector<BaseSlab> mesh_base = base_slabs(meshes.support);
    const double circle_volume = base_volume_mm3(circle_base);
    const double mesh_volume = base_volume_mm3(mesh_base);
    CAPTURE(circle_volume, mesh_volume);
    REQUIRE(circle_volume > 0.);
    CHECK(std::abs(mesh_volume / circle_volume - 1.) <= 0.25);
    CAPTURE(mesh_base.size());
    CHECK(branch_gaps(mesh_base) == 0);
}

// A branch that is present below and above a slab must be present in it. A removed slab in a drawn stack is a gap, while a branch that
// ends below an empty slab and another that starts above it are not.
TEST_CASE("A branch present below and above a slab must be present in it", "[plugins][support-tree-organic][continuity]")
{
    const Polygons column = square_mm(0., 0., 1.);
    const Polygons elsewhere = square_mm(5., 5., 1.);
    CHECK(branch_gaps({ BaseSlab{ 0, scale_to_layer_coord(0.2), column }, BaseSlab{ scale_to_layer_coord(0.2), scale_to_layer_coord(0.4), {} }, BaseSlab{ scale_to_layer_coord(0.4), scale_to_layer_coord(0.6), column } }) == 1);
    CHECK(branch_gaps({ BaseSlab{ 0, scale_to_layer_coord(0.2), column }, BaseSlab{ scale_to_layer_coord(0.2), scale_to_layer_coord(0.4), {} }, BaseSlab{ scale_to_layer_coord(0.4), scale_to_layer_coord(0.6), elsewhere } }) == 0);
    CHECK(branch_gaps({ BaseSlab{ 0, scale_to_layer_coord(0.2), column }, BaseSlab{ scale_to_layer_coord(0.2), scale_to_layer_coord(0.4), elsewhere } }) == 1);

    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    Test::init_print({SG::slab()}, print, model, SG::config("organic", false, 0));
    SG::prefix(print);
    const TreeSlabs meshes(print, true);
    std::vector<BaseSlab> stack = base_slabs(meshes.support);
    CHECK(branch_gaps(stack) == 0);
    size_t removed = 0;
    for (size_t idx = 1; idx + 1 < stack.size() && removed == 0; ++idx)
        if (!stack[idx - 1].polygons.empty() && !stack[idx].polygons.empty() && !stack[idx + 1].polygons.empty() &&
            !intersection(stack[idx - 1].polygons, stack[idx + 1].polygons).empty()) {
            CAPTURE(idx);
            stack[idx].polygons.clear();
            removed = idx;
        }
    REQUIRE(removed > 0);
    CHECK(branch_gaps(stack) == 1);
}

// Every root that stands on the bed has a circular stem below it and a tilted branch above it. The drawn slabs on either side of the root's Z
// are both present and overlap, and their outline overhang stays within the limit.
TEST_CASE("Circular stems join the tilted branch meshes without a gap or a step above the limit", "[plugins][support-tree-organic][junction]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const double limit_mm = 0.2;
    DynamicPrintConfig config = SG::config("organic", false, 0);
    config.set_deserialize_strict({{"support_material_buildplate_only", "1"}, {"support_tree_max_overhang", std::to_string(limit_mm)}});
    Print print;
    Model model;
    Test::init_print({diagonal_roof()}, print, model, config);
    SG::prefix(print);
    const Routed routed(print);
    const TreeSlabs slabs(print, true);
    const std::vector<BaseSlab> base = base_slabs(slabs.support);
    size_t junctions = 0;
    for (const TF::RoutedTree &tree : routed.trees) {
        const std::vector<size_t> below = TF::nodes_below(tree);
        for (size_t idx = 0; idx < tree.nodes.size(); ++idx) {
            if (below[idx] != TF::no_node || tree.nodes[idx].layer != 0)
                continue;
            const coord_t root_z = scale_to_layer_coord(tree.nodes[idx].print_z);
            for (size_t k = 1; k < base.size(); ++k) {
                if (middle_z(base[k - 1]) >= root_z || middle_z(base[k]) <= root_z)
                    continue;
                ++junctions;
                CAPTURE(idx, root_z, k);
                CHECK(!base[k - 1].polygons.empty());
                CHECK(!base[k].polygons.empty());
                if (!base[k - 1].polygons.empty() && !base[k].polygons.empty()) {
                    CHECK(!intersection(base[k].polygons, base[k - 1].polygons).empty());
                    CHECK(overhang_mm(base[k].polygons, base[k - 1].polygons) <= limit_mm + section_tolerance_mm);
                }
            }
        }
    }
    CAPTURE(junctions);
    CHECK(junctions >= 2);
}

// The overhang rate that the planner starts from is measured on the layer boundaries, but the branches are drawn at the middle of each slab.
// A bent organic branch with several slabs inside one object layer keeps the drawn overhang between consecutive slabs within the limit.
TEST_CASE("Bent organic slabs keep the drawn overhang within the limit inside each object layer", "[plugins][support-tree-organic][planner]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const double limit_mm = GENERATE(0.2, 0.1);
    CAPTURE(limit_mm);
    DynamicPrintConfig config = SG::config("organic", false, 0);
    config.set_deserialize_strict({{"support_material_buildplate_only", "1"}, {"support_tree_max_overhang", std::to_string(limit_mm)}});
    Print print;
    Model model;
    Test::init_print({diagonal_roof()}, print, model, config);
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    const TreeSlabs slabs(print, true);
    const std::vector<BaseSlab> base = base_slabs(slabs.support);
    size_t layers_with_several = 0;
    for (size_t id = 0; id < object.layer_count(); ++id) {
        size_t inside = 0;
        for (const BaseSlab &slab : base)
            inside += middle_z(slab) > object.layer(id).scaled_bottom_z() && middle_z(slab) < object.layer(id).scaled_print_z() ? 1 : 0;
        layers_with_several += inside >= 2 ? 1 : 0;
    }
    CAPTURE(layers_with_several, base.size());
    REQUIRE(layers_with_several > 0);
    for (size_t idx = 1; idx < base.size(); ++idx)
        if (!base[idx].polygons.empty() && !base[idx - 1].polygons.empty()) {
            const double overhang = overhang_mm(base[idx].polygons, base[idx - 1].polygons);
            CAPTURE(idx, overhang);
            CHECK(overhang <= limit_mm + section_tolerance_mm);
        }
}

// The rings of the end caps of a tilted branch reach beyond the poles of the caps. A slice between the bottom pole and the lowest
// point of the bottom cap, or above the top pole, still cuts the branch, so its section is not empty.
TEST_CASE("Branch meshes slice where their tilted end caps reach beyond the poles", "[plugins][support-tree-organic][mesh]")
{
    TF::TreeNode upper;
    upper.layer = 6;
    upper.print_z = 1.2;
    upper.position = to_c_point(Point::new_scale(0.28, 0.));
    upper.radius = scale_to_layer_coord(0.4);
    TF::TreeNode lower;
    lower.layer = 5;
    lower.print_z = 1.0;
    lower.position = to_c_point(Point::new_scale(0., 0.));
    lower.radius = scale_to_layer_coord(0.4);
    lower.carries = { 0 };
    TF::RoutedTree tree;
    tree.nodes = { upper, lower };
    const std::vector<coord_t> zs = { scale_to_layer_coord(0.7), scale_to_layer_coord(1.5) };
    const std::vector<Polygons> sections = TF::slice_branch_meshes({ tree }, zs, [] {});
    REQUIRE(sections.size() == zs.size());
    // Each cap is the half of a sphere of radius 0.4 mm on the side of its normal, cut 0.3 mm from its centre: a disc of radius 0.2646
    // mm, of which the tilted half keeps about 0.21 mm2. The faceted rings of the cap keep at least half of the disc.
    for (size_t idx = 0; idx < zs.size(); ++idx) {
        const double area_mm2 = unscaled(unscaled(area(sections[idx])));
        CAPTURE(idx, area_mm2);
        CHECK(area_mm2 >= 0.5 * M_PI * 0.07);
        CHECK(area_mm2 <= M_PI * 0.07);
    }
}

namespace {
// A post, a thin cantilever roof (0.05 mm) on top of it and a shelf past the roof tip. The roof layer overhangs the post, so it
// carries a bridge, and the shelf is a second demand just above it. The layer heights follow the profile: 0.2 mm, 0.05 mm for the roof.
TriangleMesh bridge_shelf_mesh()
{
    TriangleMesh post = make_cube(2., 8., 4.);
    TriangleMesh roof = make_cube(6., 8., 0.05);
    roof.translate(0.f, 0.f, 4.f);
    TriangleMesh shelf = make_cube(2., 8., 0.5);
    shelf.translate(6.f, 0.f, 4.05f);
    post.merge(roof);
    post.merge(shelf);
    return post;
}

// The filament contact distance gives a bridge contact at the bridge height: nozzle bridges of a quarter of the flow are 0.2 mm high.
DynamicPrintConfig bridge_contact_config(bool organic, int interfaces, const char *bridge_flow_ratio = "25%")
{
    DynamicPrintConfig config = SG::config(organic ? "organic" : "tree", false, interfaces);
    config.set_deserialize_strict({{"support_material_contact_distance_type", "filament"}, {"bridge_type", "nozzle"},
        {"bridge_flow_ratio", bridge_flow_ratio}});
    return config;
}

// The object layer of the roof: the layer that the profile thins to 0.05 mm.
size_t thin_layer_of(const PrintObject &object)
{
    size_t thin_layer = object.layer_count();
    for (size_t layer = 0; layer < object.layer_count(); ++layer)
        if (std::abs(object.layer(layer).unscaled_height() - 0.05) < 1e-4)
            thin_layer = layer;
    return thin_layer;
}

// The bridge threads of one object layer, hanging in [print_z - bridge height, print_z], and the footprint they cover.
struct ThreadBand {
    coord_t start = 0, end = 0;
    ExPolygons footprint;
};

// The thread band of an object layer and its footprint, measured with a probe slab whose top plus the gap above lies 1 um over the start of
// the threads of the layer: the footprint holds the bridges of that layer only, and its threads hang over the probe slab.
ThreadBand thread_band(const Print &print, const PrintObject &object, size_t bridge_layer)
{
    const slic3r_api::Object object_view = SG::object_view(object);
    std::vector<slic3r_api::Layer> layers;
    for (uint32_t id = 0; id < object_view.layer_count(); ++id)
        layers.push_back(object_view.layer(id));
    const Common::SupportParams params = SG::params(print, object);
    const slic3r_api::Layer layer = layers[bridge_layer];
    ThreadBand band;
    band.end = object.layer(bridge_layer).scaled_print_z();
    band.start = band.end - scale_to_layer_coord(Common::bridging_height_avg_mm(SG::print_view(print).config(), layer, layer.region(0)));
    PluginStorage storage;
    Common::GeneratedSupport probe{ reinterpret_cast<storage_handle *>(&storage) };
    const coord_t probe_z = band.start - scale_to_layer_coord(params.gap_above) + scale_to_layer_coord(0.001);
    Common::GeneratorLayer &slab = Common::add_layer(probe, Common::LayerType::TopContact, probe_z, probe_z - scale_to_layer_coord(0.2),
        scale_to_layer_coord(0.2));
    const slic3r_api::StoredExPolygonCollection footprint = Common::bridge_footprint(probe.storage, layers, SG::print_view(print).config(),
        params, slab.print_z, slab);
    band.footprint = union_ex(TF::native_expolygons(footprint));
    return band;
}

// Every slab that publication prints: the contacts, the base slabs and the dense interface slabs.
std::vector<const Common::GeneratorLayer *> printed_slabs(const Common::GeneratedSupport &support)
{
    std::vector<const Common::GeneratorLayer *> slabs;
    for (const Common::GeneratorLayers *list : { &support.top, &support.bottom, &support.base, &support.interfaces })
        slabs.insert(slabs.end(), list->begin(), list->end());
    return slabs;
}

// Whether the Z span of a slab meets the thread band shifted down by the gap above, where the object layer can no longer take support
// under the threads. A slab that ends at the bottom of that band stands under the threads and supports them.
bool in_thread_band(const Common::GeneratorLayer &slab, const ThreadBand &band, coord_t gap_above)
{
    return slab.print_z > band.start - gap_above + SCALED_EPSILON && slab.bottom_print_z() < band.end - gap_above;
}

// The area in mm2 that a slab shares with the bridge footprint.
double footprint_overlap_mm2(const Common::GeneratorLayer &slab, const ThreadBand &band)
{
    return unscaled(unscaled(area(intersection_ex(SG::native(slab.polygons), to_polygons(band.footprint)))));
}
// Checks that no printed slab in the thread band carries the footprint of the bridge, and returns the number of slabs in the band.
size_t count_slabs_in_band(const Common::GeneratedSupport &support, const ThreadBand &band, coord_t gap_above)
{
    size_t in_band = 0;
    for (const Common::GeneratorLayer *slab : printed_slabs(support))
        if (in_thread_band(*slab, band, gap_above)) {
            ++in_band;
            CAPTURE(slab->type, unscaled(slab->bottom_print_z()), unscaled(slab->print_z));
            CHECK(footprint_overlap_mm2(*slab, band) <= 1e-4);
        }
    return in_band;
}
}

// A thin roof layer with a bridge makes two top contacts, one at the normal contact distance and one at the bridge height under the
// bridge. Both survive the merge of the contacts. No printed slab that meets the band where the bridge threads hang carries the
// bridge footprint, but the bridge contact under the band keeps it, so the bridge stays supported. The case runs 8 combinations:
// tree or organic style, 0 or 3 interface layers, and a bridge flow ratio of 25 % or 200 %.
TEST_CASE("Tree slabs keep the bridge footprint out of the bridge thread band", "[plugins][support-tree-family][bridge]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool organic = GENERATE(false, true);
    const int interfaces = GENERATE(0, 3);
    // A bridge of a quarter of the flow hangs 0.2 mm high; a double flow bridge hangs 0.57 mm high, which puts base slabs in its band.
    const bool thick_bridge = GENERATE(false, true);
    CAPTURE(organic, interfaces, thick_bridge);
    Print print;
    Model model;
    SG::init_print_with_profile(print, model, bridge_shelf_mesh(), std::vector<coordf_t>{0., 0.2, 3.999, 0.2, 4.0, 0.05, 4.05, 0.05, 4.051, 0.2, 4.55, 0.2},
        bridge_contact_config(organic, interfaces, thick_bridge ? "200%" : "25%"));
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    const size_t thin_layer = thin_layer_of(object);
    REQUIRE(thin_layer + 1 < object.layer_count());
    const Common::SupportParams params = SG::params(print, object);
    const coord_t gap_above = scale_to_layer_coord(params.gap_above);
    const ThreadBand band = thread_band(print, object, thin_layer);
    REQUIRE(band.start < object.layer(thin_layer).scaled_bottom_z());
    REQUIRE_FALSE(band.footprint.empty());
    Routed routed(print);
    TF::generate_tree_slabs(nullptr, print, object, routed.trees, organic, routed.support);
    // Both contacts of the roof layer survive the merge: one at the bridge height, one at the normal contact distance.
    const coord_t bridge_z = band.start - gap_above, normal_z = object.layer(thin_layer).scaled_bottom_z() - gap_above;
    const Common::GeneratorLayer *bridge_contact = nullptr, *normal_contact = nullptr;
    for (const Common::GeneratorLayer *contact : routed.support.top) {
        if (contact->object_above != thin_layer)
            continue;
        if (std::abs(contact->print_z - bridge_z) <= 4 * SCALED_EPSILON)
            bridge_contact = contact;
        if (std::abs(contact->print_z - normal_z) <= 4 * SCALED_EPSILON)
            normal_contact = contact;
    }
    REQUIRE(bridge_contact != nullptr);
    REQUIRE(normal_contact != nullptr);
    CHECK(count_slabs_in_band(routed.support, band, gap_above) > 0);
    // The thick bridge has base slabs of its own in the band, besides the normal contact that is typed as a base slab.
    if (thick_bridge && interfaces == 0) {
        size_t base_in_band = 0;
        for (const Common::GeneratorLayer *slab : routed.support.base)
            base_in_band += in_thread_band(*slab, band, gap_above) && !slab->polygons.empty() ? 1 : 0;
        CHECK(base_in_band > 0);
    }
    // The bridge is supported: the contact under the band keeps the footprint.
    CHECK(footprint_overlap_mm2(*bridge_contact, band) > 0.);
}

// A root that rests on the block under a bridge: the floor is a bottom contact on the block, as thick as the bridge, so its Z span
// meets the band where the threads of the bridge above hang. The floor does not print the footprint of the bridge. In the second
// scenario the bridge is 0.8 mm high and the floor (4.2 to 5.0) ends at the bottom of the bridge layer: the layer starts below the
// floor top plus the gap above, so only the bridge search from the slab top reaches it.
TEST_CASE("Tree floors keep the bridge footprint out of the bridge thread band", "[plugins][support-tree-family][bridge]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool organic = GENERATE(false, true);
    const int interfaces = GENERATE(0, 3);
    // The roof height, the bridge flow ratio, and the heights of the tip and of the root of the branch.
    struct Scenario { double roof_z; const char *ratio; double tip_z, root_z; };
    const Scenario scenario = GENERATE(Scenario{ 4.8, "100%", 4.6, 4.2 }, Scenario{ 5.0, "400%", 4.8, 4.4 });
    CAPTURE(organic, interfaces, scenario.roof_z, scenario.ratio);
    TriangleMesh block = make_cube(10., 10., 4.);
    TriangleMesh post = make_cube(2., 10., scenario.roof_z);
    post.translate(18.f, 0.f, 0.f);
    TriangleMesh roof = make_cube(14., 10., 0.05);
    roof.translate(6.f, 0.f, float(scenario.roof_z));
    block.merge(post);
    block.merge(roof);
    Print print;
    Model model;
    SG::init_print_with_profile(print, model, std::move(block),
        std::vector<coordf_t>{0., 0.2, scenario.roof_z - 0.001, 0.2, scenario.roof_z, 0.05, scenario.roof_z + 0.05, 0.05},
        bridge_contact_config(organic, interfaces, scenario.ratio));
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    const size_t thin_layer = thin_layer_of(object);
    REQUIRE(thin_layer < object.layer_count());
    const coord_t gap_above = scale_to_layer_coord(SG::params(print, object).gap_above);
    const ThreadBand band = thread_band(print, object, thin_layer);
    REQUIRE_FALSE(band.footprint.empty());
    // One branch stands on the block under the roof, 3 mm from the block centre, toward the post.
    const TF::TreeSettings settings = Slic3r::Test::SupportTree::tree_settings(print, object, SG::params(print, object));
    const ExPolygons block_slices = object.layer(layer_at_height(object, 4.)).lslices();
    const ExPolygon &block_slice = *std::max_element(block_slices.begin(), block_slices.end(),
        [](const ExPolygon &a, const ExPolygon &b) { return a.area() < b.area(); });
    const Point position = get_extents(block_slice).center() + Point::new_scale(3., 0.);
    // The branch runs from a tip under the roof contact down to a root over the block, which rests on the block.
    TF::RoutedTree branch;
    for (size_t layer = layer_at_height(object, scenario.tip_z) + 1; layer-- > layer_at_height(object, scenario.root_z);) {
        TF::TreeNode node;
        node.layer = layer;
        node.print_z = object.layer(layer).unscaled_print_z();
        node.position = to_c_point(position);
        node.radius = settings.branch_radius;
        if (!branch.nodes.empty())
            node.carries = { branch.nodes.size() - 1 };
        branch.nodes.push_back(node);
    }
    Routed routed(print);
    routed.trees = { branch };
    TF::generate_tree_slabs(nullptr, print, object, routed.trees, organic, routed.support);
    REQUIRE_FALSE(routed.support.bottom.empty());
    for (const Common::GeneratorLayer *floor : routed.support.bottom)
        REQUIRE(in_thread_band(*floor, band, gap_above));
    CHECK(count_slabs_in_band(routed.support, band, gap_above) > 0);
}

// Thin interface layers under a thick bridge put the dense interface slabs of the roof contact in the band where the bridge threads
// hang. The interfaces are projected from the top contact, so they must not print the footprint of the bridge either.
TEST_CASE("Tree interface slabs keep the bridge footprint out of the bridge thread band", "[plugins][support-tree-family][bridge]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool organic = GENERATE(false, true);
    CAPTURE(organic);
    DynamicPrintConfig config = bridge_contact_config(organic, 3, "200%");
    config.set_deserialize_strict({{"support_material_interface_layer_height", "0.05"}});
    Print print;
    Model model;
    SG::init_print_with_profile(print, model, bridge_shelf_mesh(), std::vector<coordf_t>{0., 0.2, 3.999, 0.2, 4.0, 0.05, 4.05, 0.05, 4.051, 0.2, 4.55, 0.2}, config);
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    const size_t thin_layer = thin_layer_of(object);
    REQUIRE(thin_layer + 1 < object.layer_count());
    const coord_t gap_above = scale_to_layer_coord(SG::params(print, object).gap_above);
    const ThreadBand band = thread_band(print, object, thin_layer);
    REQUIRE_FALSE(band.footprint.empty());
    Routed routed(print);
    TF::generate_tree_slabs(nullptr, print, object, routed.trees, organic, routed.support);
    size_t interfaces_in_band = 0;
    for (const Common::GeneratorLayer *slab : routed.support.interfaces)
        interfaces_in_band += in_thread_band(*slab, band, gap_above) && !slab->polygons.empty() ? 1 : 0;
    REQUIRE(interfaces_in_band > 1);
    CHECK(count_slabs_in_band(routed.support, band, gap_above) >= interfaces_in_band);
}

// Print::validate rejects a support branch diameter that the support extrusion width cannot print or that is smaller than the
// tip, for the tree and the organic styles alike (both run the tree family), and accepts sane diameters for both. A tip thinner
// than the extrusion width is rejected for the tree style only: its sections are sparse-filled Base surfaces, which drop areas
// narrower than half a spacing, while the organic Branch surfaces print such sections as a deposit or along their medial axis.
TEST_CASE("Print validation checks the support tree diameters for the tree and organic styles", "[plugins][support-tree-family][validate]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    struct Diameters { const char *tip, *branch, *message; };
    const Diameters invalid[] = {
        {"0.4", "0.7", "branch diameter must not be smaller than 2x support material extrusion width"},
        {"1.5", "1", "branch diameter must not be smaller than support tree tip diameter"},
    };
    for (const char *style : {"tree", "organic"}) {
        const auto validation = [style](const Diameters &diameters) {
            DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
            config.set_deserialize_strict({{"support_material", "1"}, {"support_material_style", style},
                {"support_material_extrusion_width", "0.4"},
                {"support_tree_tip_diameter", diameters.tip}, {"support_tree_branch_diameter", diameters.branch}});
            Model model;
            Print print;
            Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
            return print.validate();
        };
        for (const Diameters &diameters : invalid) {
            INFO("style " << style << " tip " << diameters.tip << " branch " << diameters.branch);
            const std::pair<PrintBase::PrintValidationError, std::string> result = validation(diameters);
            CHECK(result.first == PrintBase::PrintValidationError::pveWrongSettings);
            CHECK(result.second.find("Organic support") == 0);
            CHECK(result.second.find(diameters.message) != std::string::npos);
        }
        // A tip of 0.1 mm is below the 0.4 mm extrusion width: only the tree style refuses it.
        const Diameters thin_tip{"0.1", "2", "tip diameter must not be smaller than support material extrusion width"};
        const bool thin_tip_refused = std::string(style) == "tree";
        {
            INFO("style " << style << " tip " << thin_tip.tip << " branch " << thin_tip.branch);
            const std::pair<PrintBase::PrintValidationError, std::string> result = validation(thin_tip);
            CHECK((result.first == PrintBase::PrintValidationError::pveWrongSettings) == thin_tip_refused);
            CHECK(result.second.empty() == !thin_tip_refused);
            if (thin_tip_refused) {
                CHECK(result.second.find("Organic support") == 0);
                CHECK(result.second.find(thin_tip.message) != std::string::npos);
            }
        }
        INFO("style " << style);
        const std::pair<PrintBase::PrintValidationError, std::string> result = validation({"0.8", "2", ""});
        CHECK(result.first == PrintBase::PrintValidationError::pveNone);
        CHECK(result.second.empty());
    }
}

// Tree and organic supports validate with a variable layer height profile on the object.
TEST_CASE("Print validation accepts tree and organic supports with a variable layer height", "[plugins][support-tree-family][validate][variable-layer-height]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *style = GENERATE("tree", "organic");
    CAPTURE(style);
    Print print;
    Model model;
    SG::init_print_with_profile(print, model, SG::slab(), std::vector<coordf_t>{0., 0.2, 2., 0.2, 2.01, 0.1, 4., 0.1, 4.01, 0.3, 5., 0.3},
        SG::config(style, false, 3));
    REQUIRE(print.object(0).model_object()->has_custom_layering());
    const std::pair<PrintBase::PrintValidationError, std::string> result = print.validate();
    CHECK(result.first == PrintBase::PrintValidationError::pveNone);
    CHECK(result.second.empty());
}

// A tree or organic support under an object with thin and thick layer bands is generated and exported on the real layer heights.
TEST_CASE("Tree and organic supports are generated and exported on a variable layer height", "[plugins][support-tree-family][variable-layer-height][gcode]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *style = GENERATE("tree", "organic");
    CAPTURE(style);
    // A tall post under a wide roof: the branches stand on the bed and cross the 0.2, 0.1 and 0.3 mm bands of the profile.
    TriangleMesh mesh = make_cube(4., 8., 6.);
    TriangleMesh roof = make_cube(12., 8., 1.);
    roof.translate(0., 0., 6.);
    mesh.merge(roof);
    DynamicPrintConfig config = SG::config(style, false, 3, true);
    config.set_deserialize_strict({
        {"step_gcode_plugin", "gcode.printing_plan_file_writer"},
        {"gcode_firmware_plugin", "gcode.firmware.marlin2"},
        {"seam_visibility", "0"},
        {"support_material_speed", "40"}, {"support_material_interface_speed", "50%"},
        {"max_print_speed", "80"}, {"first_layer_speed", "100%"}, {"first_layer_min_speed", "0"},
        {"max_volumetric_speed", "0"}, {"filament_max_speed", "0,0"},
        {"filament_max_volumetric_speed", "0,0"}, {"slowdown_below_layer_time", "0,0"},
        {"use_relative_e_distances", "1"}, {"wipe", "0,0"}, {"filament_diameter", "1.75,1.75"}
    });
    Print print;
    Model model;
    SG::init_print_with_profile(print, model, std::move(mesh),
        std::vector<coordf_t>{0., 0.2, 2., 0.2, 2.01, 0.1, 4., 0.1, 4.01, 0.3, 7., 0.3}, config);
    REQUIRE(print.validate().first == PrintBase::PrintValidationError::pveNone);
    const boost::filesystem::path path = Slic3r::Test::GCode::temporary_gcode_path();
    std::string code;
    try {
        print.process();
        Orchestrator::instance().export_gcode(print, path.string(), nullptr, nullptr);
        code = Slic3r::Test::GCode::read_text_file(path);
    } catch (...) {
        Slic3r::Test::GCode::remove_output_pair(path);
        throw;
    }
    Slic3r::Test::GCode::remove_output_pair(path);
    CHECK(code.find(";TYPE:Support material") != std::string::npos);

    // The object really has layers of the three heights.
    const PrintObject &object = print.object(0);
    std::set<long> object_heights;
    for (const Layer &layer : object.layers()) {
        object_heights.insert(std::lround(layer.unscaled_height() * 1000.));
    }
    CAPTURE(object_heights);
    CHECK(object_heights.size() >= 3);

    // The support lives on several layers of different heights, each with a non-empty section and a height that fits its Z.
    std::set<long> support_heights;
    size_t supported_layers = 0;
    double previous_top = 0., highest_support_top = 0.;
    for (const Layer &layer : object.auxiliary_layers()) {
        // The support layers stack without gap or overlap, from the bed to the roof.
        CHECK(layer.unscaled_print_z() - layer.unscaled_height() == Approx(previous_top).margin(1e-4));
        previous_top = layer.unscaled_print_z();
        if (layer.islands().empty())
            continue;
        ++supported_layers;
        highest_support_top = std::max(highest_support_top, layer.unscaled_print_z());
        support_heights.insert(std::lround(layer.unscaled_height() * 1000.));
        CHECK(layer.unscaled_height() > 0.);
        CHECK(layer.unscaled_print_z() > 0.);
        CHECK(layer.unscaled_print_z() - layer.unscaled_height() >= -1e-6);
    }
    CAPTURE(support_heights, supported_layers);
    CHECK(supported_layers > 5);
    CHECK(support_heights.size() >= 2);
    // The support follows the object: the highest support layer stops under the first object layer that contains the roof (sliced
    // at its middle, so its bottom is not the 6 mm of the mesh) by the 0.2 mm top contact distance. The post section is 32 mm2
    // and the roof section 96 mm2: the first layer wider than 60 mm2 is the roof layer.
    double roof_layer_bottom = -1.;
    for (const Layer &layer : object.layers())
        if (unscaled(unscaled(area(layer.lslices()))) > 60.) {
            roof_layer_bottom = layer.unscaled_print_z() - layer.unscaled_height();
            break;
        }
    INFO("style " << style << " roof layer bottom " << roof_layer_bottom << " highest support top " << highest_support_top);
    REQUIRE(roof_layer_bottom > 0.);
    CHECK(highest_support_top == Approx(roof_layer_bottom - 0.2).margin(1e-3));
}


namespace {
// A hollow block whose cavity is open at the bed (a hole in the first layer), closed by a lid at 4 mm, and a low shelf on each side
// of it. The shelf at 0.4 mm gets a top contact on the first layer; the contact of the shelf at 0.6 mm lies at 0.4 mm over a first
// layer base slab, which makes a first layer interface slab. The supports of the shelves stand within the brim width of the walls.
TriangleMesh brim_hollow_mesh()
{
    // The origin and size of each box: the four walls, the lid and the two shelves.
    const std::vector<std::pair<Vec3f, Vec3f>> boxes = {
        {Vec3f(0.f, 0.f, 0.f), Vec3f(14.f, 3.f, 4.f)}, {Vec3f(0.f, 11.f, 0.f), Vec3f(14.f, 3.f, 4.f)},
        {Vec3f(0.f, 3.f, 0.f), Vec3f(3.f, 8.f, 4.f)}, {Vec3f(11.f, 3.f, 0.f), Vec3f(3.f, 8.f, 4.f)},
        {Vec3f(0.f, 0.f, 4.f), Vec3f(14.f, 14.f, 1.f)},
        {Vec3f(13.f, 4.f, 0.4f), Vec3f(6.f, 6.f, 0.4f)}, {Vec3f(-5.f, 4.f, 0.6f), Vec3f(6.f, 6.f, 0.4f)}};
    TriangleMesh mesh;
    for (const std::pair<Vec3f, Vec3f> &box : boxes) {
        TriangleMesh part = make_cube(box.second.x(), box.second.y(), box.second.z());
        part.translate(box.first);
        mesh.merge(part);
    }
    return mesh;
}

// A print of the hollow block sliced with the tree slabs generated, with the given brim_width_interior (the brim width is 2 mm).
struct BrimRun {
    Print print;
    Model model;
    std::unique_ptr<Routed> routed;
    BrimRun(TriangleMesh mesh, bool organic, int interfaces, const char *interior, const char *expansion = "0")
    {
        DynamicPrintConfig config = SG::config(organic ? "organic" : "tree", false, interfaces);
        config.set_deserialize_strict({{"brim_width", "2"}, {"brim_width_interior", interior}, {"raft_first_layer_expansion", expansion}});
        Test::init_print({std::move(mesh)}, print, model, config);
        SG::prefix(print);
        routed = std::make_unique<Routed>(print);
        TF::generate_tree_slabs(nullptr, print, print.object(0), routed->trees, organic, routed->support);
    }
};

// The brim area of the first layer, built independently of Common::remove_first_layer_brim() with the same recipe: the slices grown
// by the brim width, as the native offset does (miter joins).
Polygons brim_mask(const PrintObject &object, double width)
{
    return to_polygons(offset_ex(object.layer(0).lslices(), float(scale_d(width))));
}

// The area in mm2 of the symmetric difference of two sets of polygons.
double xor_mm2(const Polygons &a, const Polygons &b)
{
    return unscaled(unscaled(area(diff_ex(a, b)) + area(diff_ex(b, a))));
}
}

// The brim area is removed from the top, base and interface slabs of the first layer, for the tree and organic styles: each such slab
// equals the slab of the same print without the brim minus the brim area; every other slab (floors included) and every dimension is
// unchanged. With a brim_width_interior of 0 the brim is not removed (a documented limit) and nothing changes.
TEST_CASE("Tree first layer slabs lose the brim area", "[plugins][support-tree-family][brim]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool organic = GENERATE(false, true);
    const int interfaces = GENERATE(0, 3);
    const bool interior = GENERATE(true, false);
    CAPTURE(organic, interfaces, interior);
    // The foot is widened by 3 mm: without interface layers the foot loses the area of the contacts on the bed, and the widening gives
    // the first base slab the area that the brim removal is then checked on.
    const BrimRun plain(brim_hollow_mesh(), organic, interfaces, "0", "3"), brimmed(brim_hollow_mesh(), organic, interfaces, interior ? "1" : "0", "3");
    // Both prints have brim_width 2; the plain one has an interior width of 0, which disables the removal of the brim area (a
    // documented limit), so its slabs are the ones before the cut.
    const Polygons mask = interior ? brim_mask(brimmed.print.object(0), 2.) : Polygons();
    ExPolygons holes;
    for (const ExPolygon &slice : plain.print.object(0).layer(0).lslices())
        for (const Polygon &hole : slice.holes) {
            ExPolygon area_of_hole;
            area_of_hole.contour = hole;
            area_of_hole.contour.reverse();
            holes.push_back(std::move(area_of_hole));
        }
    REQUIRE_FALSE(holes.empty());
    const coord_t first = scale_to_layer_coord(plain.routed->params.first_layer_height);
    const Common::GeneratedSupport &before = plain.routed->support, &after = brimmed.routed->support;
    // The lists that the brim touches, then the floors that it never touches.
    struct List { const char *name; const Common::GeneratorLayers &before, &after; bool brimmed; };
    const std::vector<List> lists = {{"top", before.top, after.top, true}, {"base", before.base, after.base, true},
                                     {"interfaces", before.interfaces, after.interfaces, true}, {"bottom", before.bottom, after.bottom, false}};
    size_t first_slabs[3] = {0, 0, 0};
    double removed = 0., support_in_hole = 0., removed_in_hole = 0., in_mask[3] = {0., 0., 0.};
    for (size_t kind = 0; kind < lists.size(); ++kind) {
        const List &list = lists[kind];
        REQUIRE(list.before.size() == list.after.size());
        for (size_t i = 0; i < list.before.size(); ++i) {
            const Common::GeneratorLayer &slab_before = *list.before[i], &slab_after = *list.after[i];
            CAPTURE(list.name, i, unscaled(slab_before.print_z));
            CHECK(slab_before.type == slab_after.type);
            CHECK(slab_before.print_z == slab_after.print_z);
            CHECK(slab_before.bottom_z == slab_after.bottom_z);
            CHECK(slab_before.height == slab_after.height);
            const Polygons polygons_before = SG::native(slab_before.polygons), polygons_after = SG::native(slab_after.polygons);
            if (!list.brimmed || slab_before.print_z > first) {
                CHECK(xor_mm2(polygons_after, polygons_before) < 1e-4);
                continue;
            }
            CHECK(xor_mm2(polygons_after, diff(polygons_before, mask)) < 1e-3);
            removed += unscaled(unscaled(area(union_ex(polygons_before)))) - unscaled(unscaled(area(union_ex(polygons_after))));
            if (polygons_before.empty())
                continue;
            ++first_slabs[kind];
            support_in_hole += unscaled(unscaled(area(intersection_ex(polygons_before, to_polygons(holes)))));
            in_mask[kind] += unscaled(unscaled(area(intersection_ex(polygons_before, mask))));
            removed_in_hole += unscaled(unscaled(area(intersection_ex(intersection_ex(polygons_before, mask), holes))));
        }
    }
    // The case is not vacuous: the first layer has top contact, base and (with interface layers) interface slabs, and the hole holds support.
    CHECK(first_slabs[0] > 0);
    CHECK(first_slabs[1] > 0);
    CHECK((first_slabs[2] > 0) == (interfaces > 0));
    CHECK(support_in_hole > 0.);
    CAPTURE(removed, removed_in_hole, in_mask[0], in_mask[1], in_mask[2]);
    if (interior) {
        // Each list present has first-layer support inside the brim area before the cut, and the hole loses some.
        CHECK(in_mask[0] > 0.);
        CHECK(in_mask[1] > 0.);
        CHECK((in_mask[2] > 0.) == (interfaces > 0));
        CHECK(removed_in_hole > 0.);
        CHECK(removed > 0.1);
    } else
        CHECK(removed == Approx(0.).margin(1e-6));
}

// Publication and toolpaths run on the sliced print with the brim, and the extrusions of the first support layer (the organic flange
// included) stay out of the brim area.
TEST_CASE("Tree first layer extrusions keep out of the brim area", "[plugins][support-tree-family][brim]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool organic = GENERATE(false, true);
    CAPTURE(organic);
    DynamicPrintConfig config = SG::config(organic ? "organic" : "tree", false, 3);
    config.set_deserialize_strict({{"brim_width", "2"}, {"brim_width_interior", "1"}});
    Print print;
    Model model;
    Test::init_print({brim_hollow_mesh()}, print, model, config);
    SG::prefix(print);
    REQUIRE_NOTHROW(SG::family(print));
    REQUIRE_NOTHROW(SG::toolpaths(print));
    const PrintObject &object = print.object(0);
    REQUIRE_FALSE(object.auxiliary_layers().empty());
    coord_t previous_z = 0;
    for (const Layer &layer : object.auxiliary_layers()) {
        INFO("auxiliary layer id " << layer.id() << " print_z " << unscaled(layer.scaled_print_z()) << " previous " << previous_z);
        CHECK(layer.scaled_print_z() >= previous_z);
        previous_z = layer.scaled_print_z();
    }
    // The support layers of the bed layer: their extrusion polylines, measured whole, against the brim area shrunk by 0.05 mm. A
    // path centre lies at least half an extrusion width (0.2 mm) inside the surface it fills, and the clipping of the surfaces and
    // the mask rounds by micrometres, so only a path that really enters the brim area exceeds the 0.01 mm tolerance.
    const ExPolygons mask = offset_ex(brim_mask(object, 2.), -float(scale_d(0.05)));
    // A mask grown by 1 mm shows that the measure sees paths: the support stands close to the brim area.
    const ExPolygons grown_mask = offset_ex(brim_mask(object, 3.), -float(scale_d(0.05)));
    size_t points = 0, branch_surfaces = 0;
    double inside_mm = 0., grown_inside_mm = 0.;
    const SurfaceType branch_type = stModSupport | stModBranch | stPosInternal | stDensSparse;
    for (const Layer &layer : object.auxiliary_layers()) {
        if (layer.scaled_print_z() > object.layer(0).scaled_print_z())
            continue;
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands()) {
                for (const Surface &surface : region.fill_surfaces())
                    branch_surfaces += surface.surface_type == branch_type && !surface.expolygon.empty();
                for (const ExtrusionRole bucket : {LayerRegionIsland::SUPPORT, LayerRegionIsland::SUPPORT_INTERFACE}) {
                    if (!region.has_extrusion(bucket))
                        continue;
                    ExtrusionPaths paths;
                    SG::legacy_paths(region.extrusion(bucket), paths);
                    Polylines polylines;
                    for (const ExtrusionPath &path : paths) {
                        polylines.push_back(path.polyline_ref().to_polyline());
                        points += polylines.back().points.size();
                    }
                    for (const Polyline &clipped : intersection_pl(polylines, mask))
                        inside_mm += unscaled(clipped.length());
                    for (const Polyline &clipped : intersection_pl(polylines, grown_mask))
                        grown_inside_mm += unscaled(clipped.length());
                }
            }
    }
    CAPTURE(points, branch_surfaces, inside_mm, grown_inside_mm);
    CHECK(points > 0);
    CHECK(grown_inside_mm > 0.);
    // The organic flange is printed on the Branch surfaces that touch the bed; the tree style has none.
    if (organic)
        CHECK(branch_surfaces > 0);
    else
        CHECK(branch_surfaces == 0);
    CHECK(inside_mm < 0.01);
}

namespace {
// A block with a 1.5 mm shelf at 0.4 mm: the only support is under the shelf, a foot that lies entirely within the 2 mm of the brim
// width from the wall.
TriangleMesh brim_foot_mesh()
{
    TriangleMesh mesh = make_cube(10., 10., 4.);
    TriangleMesh shelf = make_cube(1.5, 6., 0.4);
    shelf.translate(9.5f, 2.f, 0.4f);
    mesh.merge(shelf);
    return mesh;
}
}

// A first layer slab that lies entirely in the brim area ends empty. It stays listed, and publication and toolpaths run without error
// and print nothing for it: no support layer of the bed layer holds an island.
TEST_CASE("Tree first layer slabs inside the brim area end empty and publish nothing", "[plugins][support-tree-family][brim]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool organic = GENERATE(false, true);
    const int interfaces = GENERATE(0, 3);
    CAPTURE(organic, interfaces);
    const BrimRun plain(brim_foot_mesh(), organic, interfaces, "0"), brimmed(brim_foot_mesh(), organic, interfaces, "1");
    const coord_t first = scale_to_layer_coord(plain.routed->params.first_layer_height);
    const Polygons mask = brim_mask(brimmed.print.object(0), 2.);
    size_t listed = 0, emptied = 0;
    double before_mm2 = 0., outside_mm2 = 0.;
    const Common::GeneratedSupport &before = plain.routed->support, &after = brimmed.routed->support;
    const std::vector<std::pair<const Common::GeneratorLayers *, const Common::GeneratorLayers *>> lists = {
        {&before.top, &after.top}, {&before.base, &after.base}, {&before.interfaces, &after.interfaces}};
    for (const std::pair<const Common::GeneratorLayers *, const Common::GeneratorLayers *> &list : lists) {
        REQUIRE(list.first->size() == list.second->size());
        for (size_t i = 0; i < list.first->size(); ++i) {
            const Common::GeneratorLayer &slab = *(*list.first)[i];
            if (slab.print_z > first || slab.polygons.empty())
                continue;
            ++listed;
            const Polygons polygons_before = SG::native(slab.polygons);
            before_mm2 += unscaled(unscaled(area(union_ex(polygons_before))));
            outside_mm2 += unscaled(unscaled(area(diff_ex(polygons_before, mask))));
            emptied += (*list.second)[i]->polygons.empty();
        }
    }
    CAPTURE(listed, emptied, before_mm2, outside_mm2);
    // The foot exists before the cut, lies in the brim area, and every first layer slab ends empty.
    REQUIRE(listed > 0);
    CHECK(before_mm2 > 0.);
    CHECK(outside_mm2 < 1e-3);
    CHECK(emptied == listed);
    for (const Common::GeneratorLayers *list : {&after.top, &after.base, &after.interfaces})
        for (const Common::GeneratorLayer *slab : *list)
            if (slab->print_z <= first)
                REQUIRE(slab->polygons.empty());
    // The same print goes through publication and toolpaths.
    DynamicPrintConfig config = SG::config(organic ? "organic" : "tree", false, interfaces);
    config.set_deserialize_strict({{"brim_width", "2"}, {"brim_width_interior", "1"}});
    Print print;
    Model model;
    Test::init_print({brim_foot_mesh()}, print, model, config);
    SG::prefix(print);
    REQUIRE_NOTHROW(SG::family(print));
    REQUIRE_NOTHROW(SG::toolpaths(print));
    size_t islands = 0;
    for (const Layer &layer : print.object(0).auxiliary_layers())
        if (layer.scaled_print_z() <= print.object(0).layer(0).scaled_print_z())
            islands += layer.islands().size();
    CHECK(islands == 0);
}

namespace {
// The foot of a run: the first base slab, which must start on the bed.
const Common::GeneratorLayer &foot_of(const BrimRun &run)
{
    REQUIRE_FALSE(run.routed->support.base.empty());
    const Common::GeneratorLayer &foot = *run.routed->support.base.front();
    REQUIRE(foot.bottom_print_z() == 0);
    return foot;
}

// The footprints of the top contacts and interface slabs that start on the bed, grown by EPSILON: the foot keeps clear of them.
Polygons bed_interfaces(const Common::GeneratedSupport &support)
{
    Polygons grown;
    for (const Common::GeneratorLayers *list : {&support.top, &support.interfaces})
        for (const Common::GeneratorLayer *slab : *list)
            if (slab->bottom_print_z() == 0)
                append(grown, offset(SG::native(slab->polygons), float(scale_d(EPSILON)), ClipperLib::jtSquare, 0.));
    return grown;
}
}

// The foot, the first base slab on the bed, grows by raft_first_layer_expansion in the tree and organic styles (a tree base slab and
// an organic branch slab), never crosses the first object layer grown by the XY gap, and the other slabs are unchanged. An expansion of
// 0 only cuts the foot. The expected foot is the growth recipe applied independently to the foot of the run without expansion.
TEST_CASE("Tree first layer foot grows by the expansion", "[plugins][support-tree-family][foot]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool organic = GENERATE(false, true);
    const int interfaces = GENERATE(0, 3);
    CAPTURE(organic, interfaces);
    const BrimRun narrow(Test::SupportGrid::slab(), organic, interfaces, "0", "0"), wide(Test::SupportGrid::slab(), organic, interfaces, "0", "3");
    const Common::GeneratorLayer &foot0 = foot_of(narrow), &foot3 = foot_of(wide);
    CAPTURE(foot0.type, foot3.type);
    CHECK(foot3.type == (organic ? Common::LayerType::Branch : Common::LayerType::Base));
    const Common::SupportParams &params = wide.routed->params;
    // The joins of the production recipe: square for the object trimming and the dense footprints (CLIPPER_JOIN_SQUARE, miter limit 0), the
    // default miter join (limit 3) for the growth steps.
    const Polygons trimming = offset(to_polygons(wide.print.object(0).layer(0).lslices()), float(params.gap_xy), ClipperLib::jtSquare, 0.);
    const Polygons polygons0 = SG::native(foot0.polygons), polygons3 = SG::native(foot3.polygons);
    const Polygons clear_of = bed_interfaces(wide.routed->support);
    REQUIRE_FALSE(polygons0.empty());
    // With no expansion the foot is only cut: it is clear of the object and of the dense slabs, and cutting it again changes nothing.
    CHECK(xor_mm2(polygons0, diff(diff(polygons0, trimming), bed_interfaces(narrow.routed->support))) < 1e-4);
    // The expanded foot is clear of the object grown by the XY gap and of the dense slabs, and contains the narrow one.
    const double crossing = unscaled(unscaled(area(intersection_ex(polygons3, trimming))));
    const double lost = unscaled(unscaled(area(diff_ex(polygons0, polygons3))));
    const double growth = unscaled(unscaled(area(union_ex(polygons3)))) - unscaled(unscaled(area(union_ex(polygons0))));
    CAPTURE(crossing, lost, growth);
    CHECK(crossing < 1e-3);
    CHECK(unscaled(unscaled(area(intersection_ex(polygons3, clear_of)))) < 1e-3);
    CHECK(lost < 1e-3);
    CHECK(growth > 1.);
    // The recipe applied to the narrow foot: at least 5 steps of at most one first layer width, each cut by the object grown by the XY gap.
    const double expansion = scale_d(3.) - scale_d(EPSILON);
    const int steps = std::max(5, int(std::ceil(expansion / params.first_flow.width)));
    Polygons expected = polygons0;
    for (int i = 0; i < steps; ++i)
        expected = diff(offset(expected, float(expansion / steps)), trimming);
    expected = diff(expected, clear_of);
    const double error = xor_mm2(polygons3, expected), expected_area = unscaled(unscaled(area(union_ex(expected))));
    CAPTURE(error, expected_area);
    CHECK(error < 0.02 * expected_area);
    // Every other slab and every dimension is unchanged.
    const std::vector<std::pair<const Common::GeneratorLayers *, const Common::GeneratorLayers *>> lists = {
        {&narrow.routed->support.top, &wide.routed->support.top}, {&narrow.routed->support.base, &wide.routed->support.base},
        {&narrow.routed->support.interfaces, &wide.routed->support.interfaces}, {&narrow.routed->support.bottom, &wide.routed->support.bottom}};
    for (const std::pair<const Common::GeneratorLayers *, const Common::GeneratorLayers *> &list : lists) {
        REQUIRE(list.first->size() == list.second->size());
        for (size_t i = 0; i < list.first->size(); ++i) {
            const Common::GeneratorLayer &a = *(*list.first)[i], &b = *(*list.second)[i];
            CHECK(a.print_z == b.print_z);
            CHECK(a.bottom_z == b.bottom_z);
            CHECK(a.height == b.height);
            CHECK(a.type == b.type);
            if (&a != &foot0)
                CHECK(xor_mm2(SG::native(a.polygons), SG::native(b.polygons)) < 1e-4);
        }
    }
}

// The widening comes before the brim removal: the widened foot loses the reserved brim area, which reaches into the part it gained.
TEST_CASE("Tree widened foot loses the brim area", "[plugins][support-tree-family][foot]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool organic = GENERATE(false, true);
    CAPTURE(organic);
    const BrimRun plain(Test::SupportGrid::slab(), organic, 3, "0", "3"), brimmed(Test::SupportGrid::slab(), organic, 3, "1", "3");
    const Polygons mask = brim_mask(brimmed.print.object(0), 2.);
    const Polygons before = SG::native(foot_of(plain).polygons), after = SG::native(foot_of(brimmed).polygons);
    const double in_mask = unscaled(unscaled(area(intersection_ex(before, mask))));
    CAPTURE(in_mask);
    CHECK(in_mask > 0.1);
    CHECK(xor_mm2(after, diff(before, mask)) < 1e-3);
    // The part of the widened foot that the narrow foot does not have lies in the brim area as well.
    const BrimRun narrow(Test::SupportGrid::slab(), organic, 3, "0", "0");
    const double gained_in_mask = unscaled(unscaled(area(intersection_ex(diff(before, SG::native(foot_of(narrow).polygons)), mask))));
    CAPTURE(gained_in_mask);
    CHECK(gained_in_mask > 0.);
}

namespace {
// The published surfaces and the support toolpaths of the bed layer of a print that went through publication and toolpaths.
struct BedLayerPrint {
    Print print;
    Model model;
    ExPolygons surfaces, branch_surfaces;
    Polylines support_paths;
    BedLayerPrint(bool organic, const char *expansion)
    {
        DynamicPrintConfig config = SG::config(organic ? "organic" : "tree", false, 3);
        config.set_deserialize_strict({{"raft_first_layer_expansion", expansion}});
        Test::init_print({Test::SupportGrid::slab()}, print, model, config);
        SG::prefix(print);
        REQUIRE_NOTHROW(SG::family(print));
        REQUIRE_NOTHROW(SG::toolpaths(print));
        const PrintObject &object = print.object(0);
        const SurfaceType branch_type = stModSupport | stModBranch | stPosInternal | stDensSparse;
        for (const Layer &layer : object.auxiliary_layers()) {
            if (layer.scaled_print_z() > object.layer(0).scaled_print_z())
                continue;
            for (const LayerSliceIsland &island : layer.islands())
                for (const LayerRegionIsland &region : island.regions_islands()) {
                    for (const Surface &surface : region.fill_surfaces()) {
                        surfaces.push_back(surface.expolygon);
                        if (surface.surface_type == branch_type)
                            branch_surfaces.push_back(surface.expolygon);
                    }
                    if (!region.has_extrusion(LayerRegionIsland::SUPPORT))
                        continue;
                    ExtrusionPaths paths;
                    SG::legacy_paths(region.extrusion(LayerRegionIsland::SUPPORT), paths);
                    for (const ExtrusionPath &path : paths)
                        support_paths.push_back(path.polyline_ref().to_polyline());
                }
        }
        surfaces = union_ex(surfaces);
        branch_surfaces = union_ex(branch_surfaces);
    }
};

// The length in mm of the polylines inside the areas.
double length_inside_mm(const Polylines &polylines, const ExPolygons &areas)
{
    double length = 0.;
    for (const Polyline &clipped : intersection_pl(polylines, areas))
        length += unscaled(clipped.length());
    return length;
}
}

// The widened part of the foot is published and printed, in both styles: with an expansion of 3 mm the bed layer publishes more area
// than with none, the support toolpaths run inside the gained area, they stay out of the first object layer, and for the organic style
// the Branch surfaces of the bed carry the flange paths, in the gained area too.
TEST_CASE("Tree widened foot is published and printed", "[plugins][support-tree-family][foot]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool organic = GENERATE(false, true);
    CAPTURE(organic);
    const BedLayerPrint narrow(organic, "0"), wide(organic, "3");
    const double narrow_mm2 = unscaled(unscaled(area(narrow.surfaces))), wide_mm2 = unscaled(unscaled(area(wide.surfaces)));
    // The gained area, shrunk by 0.05 mm so that a path on its border is not counted.
    const ExPolygons gained = offset_ex(diff_ex(wide.surfaces, narrow.surfaces), -float(scale_d(0.05)));
    const double gained_mm = length_inside_mm(wide.support_paths, gained);
    const double inside_object_mm = length_inside_mm(wide.support_paths, wide.print.object(0).layer(0).lslices());
    CAPTURE(narrow_mm2, wide_mm2, gained_mm, inside_object_mm, wide.support_paths.size());
    REQUIRE_FALSE(narrow.support_paths.empty());
    CHECK(wide_mm2 > narrow_mm2 + 1.);
    CHECK(gained_mm > 1.);
    CHECK(inside_object_mm < 0.01);
    // The flange of the organic style follows the Branch surfaces of the bed layer; the tree style has none.
    const double branch_mm2 = unscaled(unscaled(area(wide.branch_surfaces)));
    const double flange_mm = length_inside_mm(wide.support_paths, offset_ex(wide.branch_surfaces, float(scale_d(0.05))));
    const double gained_branch_mm = length_inside_mm(wide.support_paths, intersection_ex(gained, wide.branch_surfaces));
    CAPTURE(branch_mm2, flange_mm, gained_branch_mm);
    if (organic) {
        CHECK(branch_mm2 > 1.);
        CHECK(flange_mm > 1.);
        CHECK(gained_branch_mm > 1.);
    } else
        CHECK(branch_mm2 == Approx(0.).margin(1e-9));
}
