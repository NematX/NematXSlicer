#include "polygon_metrics_test_helpers.hpp"
#include "support_grid_test_helpers.hpp"
#include "support_tree_test_helpers.hpp"
#include "gcode_test_helpers.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/GCode/ToolOrdering.hpp"
#include "libslic3r/Api/internal/LayerAccess.hpp"
#include "libslic3r/Steps/StepGenerateSupport.hpp"
#include "libslic3r/Plugins/Support/Common/SupportPublication.hpp"
#include "libslic3r/Plugins/Support/Common/SupportCounters.hpp"
#include "libslic3r/Plugins/Support/Common/SupportZPlacement.hpp"
#include "libslic3r/Plugins/Support/Toolpath/BranchIslandClass.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/Plugins/Support/Tree/TreeBranchMesh.hpp"
#include "libslic3r/Plugins/Support/Tree/TreeRouting.hpp"
#include "libslic3r/Plugins/Support/Tree/TreeSettings.hpp"
#include "libslic3r/Plugins/Support/Tree/TreeSupportGenerator.hpp"
#include "libslic3r/Plugins/Support/Tree/TreeSmoothing.hpp"
#include "libslic3r/Plugins/Support/Tree/TreeZPlanner.hpp"
#include "libslic3r/Format/OBJ.hpp"
#include "libslic3r/Support/SupportMaterial.hpp"
#include "libslic3r/Support/TreeSupport.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <oneapi/tbb/global_control.h>

/*
Tree-family tests: the slabs that the tree and organic styles publish. The branch slab planner is checked on synthetic
overhang layers, the slabs of small objects on their geometry (interfaces, floors, heights, XY distance, branch overhang),
then the published surfaces and the G-code of a whole print. The grid is unchanged by this family, which the grid
comparison tests cover. Tip-material checks copy published paths and fused islands once per run, then lazily measure
islands without changing the geometry. The Print::validate checks of the tree family settings (the [validate] case) are checked here too.

Representative direct calls (branches are not a timeline):

    TEST_CASE
        |-- Test::Plugins::ensure_plugin_test_runtime_initialized()
        |-- Test::init_print()
        |-- SG::prepare_legacy_inputs()
        |-- generator.generate()
        |-- TF::generate_tree_slabs()
        `-- Common::publish()
*/

using namespace Slic3r;
namespace SG = Slic3r::Test::SupportGrid;
namespace TF = Slic3r::TreeFamily;
namespace Common = slic3r_api::Support::Common;

namespace {
// Tolerance of the geometric checks, in millimeters.
constexpr double tolerance_mm = 0.01;

// Supplies the native layer-top table to isolated smoothing tests, without routing support.
std::vector<double> layer_print_z_mm(const PrintObject &object);

// Checks final slab XY clearance for one style/interface combination.
void check_normal_xy_clearance(const char *style, int interfaces);
// Checks downward-point object and published-support island ordering for one thickness variant and layer height.
void check_downward_point_island_order(bool thin, const char *layer_height);
struct GapScenario;
// Checks one shelf/low-roof/lobe gap scenario using the selected support style.
void check_shelf_clearance(const char *style, const GapScenario &scenario);
// Checks each routed organic tip against published support toolpaths for one mesh/configuration.
void check_isolated_tips(int fixture, bool thin, int interfaces, const char *layer_height);

// The tree slabs of the first object of a sliced print, generated directly. The storage is declared first so that it
// outlives the slabs that live in it.
struct TreeSlabs {
    PluginStorage storage;
    Common::GeneratedSupport support{reinterpret_cast<storage_handle *>(&storage)};
    // Borrows a sliced print and fills owned slabs using one parameter construction; organic selects the branch style and generation failures propagate.
    TreeSlabs(Print &print, bool organic)
    {
        const Slic3r::Test::SupportTree::SupportPatternContext patterns(print, reinterpret_cast<storage_handle *>(&storage));
        const Common::SupportParams params = SG::params(print, print.object(0));
        TF::generate_tree_support(patterns.context(), SG::print_view(print), SG::object_view(print.object(0)), params, organic, support);
    }
};

using Routed = Slic3r::Test::SupportTree::Routed;
using Slic3r::Test::SupportTree::to_c_point;
using Slic3r::Test::SupportTree::to_point;
using Slic3r::Test::SupportTree::to_native;
using Slic3r::Test::SupportTree::assign_native;

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
        slabs.push_back(BaseSlab{slab->bottom_z, slab->print_z, to_native(slab->polygons)});
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
            {"support_max_overhang", std::to_string(limit_mm)}});
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
        CHECK_FALSE(to_native(contact->polygons).empty());
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
double contact_branch_mm2(storage_handle *storage, const PrintObject &object, const std::vector<TF::RoutedTree> &trees, bool organic, coord_t z)
{
    if (organic) {
        const std::vector<slic3r_api::StoredPolygonCollection> sections = TF::slice_branch_meshes(storage, trees, { z }, nullptr);
        return unscaled(unscaled(area(union_(to_native(sections.front())))));
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
    TF::generate_tree_slabs(nullptr, SG::print_view(print), SG::object_view(object), routed.params, routed.settings, routed.trees, organic, routed.support, routed.resting);
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
        const Polygons &polygons = to_native(contact->polygons);
        contact_area.insert(contact_area.end(), polygons.begin(), polygons.end());
        contact_zs.push_back(contact->print_z);
        expected_mm2 += contact_branch_mm2(routed.support.storage, object, routed.trees, organic, contact->bottom_z + (contact->print_z - contact->bottom_z) / 2);
    }
    for (const Point &tip : tips)
        CHECK(distance_to_region_mm(tip, contact_area) <= tolerance_mm);
    SG::family(print);
    const SurfaceType branch_type = stModSupport | stPosInternal | stDensSparse;
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
    TF::generate_tree_slabs(nullptr, SG::print_view(print), SG::object_view(object), routed.params, routed.settings, routed.trees, organic, routed.support, routed.resting);
    REQUIRE_FALSE(routed.support.top.empty());
    const coord_t contact_z = routed.support.top.front()->print_z;
    REQUIRE(unscaled(contact_z) == Approx(4.05));
    Common::publish(nullptr, SG::object_view(object), routed.params, routed.support, reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
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

namespace {
// One geometric extrusion entity, including one-point deposits; line order and vertices are copied without flattening leaves together.
struct TipOracleLeaf {
    ArcPolylines lines;
    bool deposit = false;
};
// Lazily computed, tip-independent properties of one fused island; the region owns its unchanged polygon separately.
struct TipOracleIsland {
    bool ready = false;
    slic3r_api::Support::Toolpath::BranchIslandMeasure measure;
    size_t leaves = 0, deposits = 0;
};
// Region-owned immutable geometry/path copies and lazy island measurements, in published region and fusion order.
struct TipOracleRegion {
    ExPolygons fused;
    std::vector<TipOracleLeaf> paths;
    std::vector<TipOracleIsland> islands;
};
// Layer bounds retain the original floating conversions; regions follow published island/region traversal order.
struct TipOracleLayer {
    double bottom_z = 0., print_z = 0.;
    std::vector<TipOracleRegion> regions;
};

// Copies geometric entities into owned leaves, preserving recursive traversal and deposit identity.
void copy_tip_paths(const ExtrusionEntity &entity, std::vector<TipOracleLeaf> &paths);
// Measures the nearest cached segment or singleton point to a query point.
double distance_to_paths_mm(const std::vector<TipOracleLeaf> &paths, const Point &point);
// Counts cached vertices within the query radius, including singleton deposits.
size_t path_points_within(const std::vector<TipOracleLeaf> &paths, const Point &point, double radius_mm);
// Counts cached leaves and deposits with at least one vertex in the supplied area.
void leaves_in_area(const std::vector<TipOracleLeaf> &paths, const ExPolygons &area, size_t &leaves, size_t &deposits);

// Copies each borrowed geometric entity's polylines once into caller-owned storage; recursive order and deposit detection
// match the entity tree, including geometric parents with children. No source mutation occurs; allocation failures propagate.
void copy_tip_paths(const ExtrusionEntity &entity, std::vector<TipOracleLeaf> &paths)
{
    if (entity.has_polyline()) {
        paths.emplace_back();
        paths.back().lines = entity.as_polylines();
        paths.back().deposit = slic3r_api::ExtrusionEntity(reinterpret_cast<const extrusion_entity_handle *>(&entity)).is_deposit();
    }
    for (const ExtrusionEntityUPtr &child : entity.children())
        copy_tip_paths(*child, paths);
}

// Smallest distance in millimeters from the point to the extrusion paths of an extrusion tree (a deposit is a point path), or
// the largest double when the borrowed cached leaves have none; uses unchanged point/segment arithmetic without mutation.
double distance_to_paths_mm(const std::vector<TipOracleLeaf> &paths, const Point &point)
{
    double best = std::numeric_limits<double>::max();
    for (const TipOracleLeaf &leaf : paths)
        for (const ArcPolyline &line : leaf.lines) {
            if (line.size() == 1)
                best = std::min(best, unscaled(point.distance_to(line.get_point(0))));
            for (size_t idx = 0; idx + 1 < line.size(); ++idx)
                best = std::min(best, unscaled(Line(line.get_point(idx), line.get_point(idx + 1)).distance_to(point)));
        }
    return best;
}

// Counts vertices in borrowed cached leaves with the original inclusive millimeter-distance comparison; singleton deposits
// count as one point, geometry is unchanged and no references escape.
size_t path_points_within(const std::vector<TipOracleLeaf> &paths, const Point &point, double radius_mm)
{
    size_t count = 0;
    for (const TipOracleLeaf &leaf : paths)
        for (const ArcPolyline &line : leaf.lines)
            for (size_t idx = 0; idx < line.size(); ++idx)
                count += unscaled(point.distance_to(line.get_point(idx))) <= radius_mm ? 1 : 0;
    return count;
}

// Adds one count per borrowed cached leaf with any vertex inside the borrowed area, plus its stored deposit flag; containment
// order and boundary semantics are unchanged, caller-owned scalar counts accumulate and geometry remains immutable.
void leaves_in_area(const std::vector<TipOracleLeaf> &paths, const ExPolygons &area, size_t &leaves, size_t &deposits)
{
    for (const TipOracleLeaf &leaf : paths) {
        bool inside = false;
        for (const ArcPolyline &line : leaf.lines)
            for (size_t idx = 0; idx < line.size() && !inside; ++idx)
                for (const ExPolygon &polygon : area)
                    inside = inside || polygon.contains(line.get_point(idx));
        if (inside) {
            ++leaves;
            deposits += leaf.deposit ? 1 : 0;
        }
    }
}

// Number of extrusion leaves of the tree, and of them how many are deposits.
void count_leaves(const ExtrusionEntity &entity, size_t &leaves, size_t &deposits)
{
    if (entity.has_polyline()) {
        ++leaves;
        deposits += slic3r_api::ExtrusionEntity(reinterpret_cast<const extrusion_entity_handle *>(&entity)).is_deposit() ? 1 : 0;
    }
    for (const ExtrusionEntityUPtr &child : entity.children())
        count_leaves(*child, leaves, deposits);
}

// The width in millimeters of the widest part of the island: twice the largest erosion that leaves something (bisection).
double island_width_mm(const ExPolygon &island)
{
    double low = 0., high = 4.;
    for (int step = 0; step < 14; ++step) {
        const double mid = 0.5 * (low + high);
        (offset_ex(island, -float(scale_d(mid))).empty() ? high : low) = mid;
    }
    return 2. * low;
}

// What the support step made of the area around a tip, on one auxiliary layer: a description of the fused branch islands that touch
// the circle of the tip (class, measures, paths that print inside them) and the paths that reach the circle.
struct TipLayerFact {
    double bottom_z = 0., print_z = 0.;
    std::string islands;
    double nearest_path_mm = std::numeric_limits<double>::max();
    size_t points_in_circle = 0;
};

// Builds the arm (0), tab (1) or downward-point (2) fixture mesh.
TriangleMesh isolated_tips_mesh(int fixture)
{
    TriangleMesh mesh = make_cube(4., 4., fixture == 2 ? 6. : 4.);
    if (fixture == 0 || fixture == 1) {
        // A bar 1 mm wide and 1 mm thick, 8 mm out of the post, or a tab of 1.5 x 1.5 mm; both 0.5 mm into the post.
        TriangleMesh arm = fixture == 0 ? make_cube(8.5, 1., 1.) : make_cube(2., 1.5, 1.);
        arm.translate(3.5f, fixture == 0 ? 1.5f : 1.25f, 3.f);
        mesh.merge(arm);
    } else {
        // A flat cone with its apex down, under the top of the post.
        TriangleMesh cone = make_cone(3., 1.5);
        cone.rotate_x(float(PI));
        cone.translate(6.f, 2.f, 6.f);
        mesh.merge(cone);
    }
    return mesh;
}
const char *isolated_tips_name(int fixture) { return fixture == 0 ? "arm" : fixture == 1 ? "tab" : "point"; }
} // namespace

// Every tip of an organic tree must leave some support material at its place. Two criteria are checked per tip: the loose one,
// an extrusion path (line, centreline or stationary deposit) within the tip radius + 1.5 flow widths of the tip centre on a layer
// whose top is at or above the tip node; and the strict one, at least one extrusion in the fused island of the tip's own top slab
// (the first auxiliary layer whose top is at or above the tip node) that holds the tip centre, else the nearest island within the
// tip radius. The fixtures have isolated tips: a thin arm, a small tab and a downward
// point. The tips and the slabs come from one routing, and the materials are read
// from the support toolpaths of the published layers. The thin case is a thin organic tip: tip, branch and extrusion width
// of 0.42 mm; the control keeps the default tip and branch. The facts are printed as warnings; a failing CHECK below is a defect
// of the tip material, not a fixture problem, and must not be loosened.
namespace {
// Generates owned support and extrusion trees for one mesh/configuration, then checks every routed tip against locally cached
// immutable published geometry. Lazy island measurements do not alter polygons; generation/allocation failures reach Catch.
void check_isolated_tips(int fixture, bool thin, int interfaces, const char *layer_height)
{
    CAPTURE(isolated_tips_name(fixture), thin, interfaces, layer_height);
    DynamicPrintConfig config = SG::config("organic", false, interfaces, true);
    config.set_deserialize_strict({{"layer_height", layer_height}, {"first_layer_height", layer_height}});
    if (thin)
        config.set_deserialize_strict({{"support_tree_tip_diameter", "0.42"}, {"support_tree_branch_diameter", "0.42"},
                                       {"support_material_extrusion_width", "0.42"}});
    const std::chrono::steady_clock::time_point started = std::chrono::steady_clock::now();
    Print print;
    Model model;
    Test::init_print({isolated_tips_mesh(fixture)}, print, model, config);
    SG::prefix(print);
    // The routing varies from run to run, so the tips and the slabs come from the same routing: the trees are routed once, their
    // slabs are published, and the support toolpaths run on those layers.
    Routed routed(print);
    TF::generate_tree_slabs(nullptr, SG::print_view(print), SG::object_view(print.object(0)), routed.params, routed.settings, routed.trees, true, routed.support, routed.resting);
    Common::publish(nullptr, SG::object_view(print.object(0)), routed.params, routed.support, reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
    SG::toolpaths(print);
    const double process_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    const PrintObject &object = print.object(0);
    const coord_t width = SG::params(print, object).base_flow.width;
    const c_flow flow = SG::params(print, object).base_flow;
    const double tolerance = 1.5 * unscaled(width);
    // Cache only after toolpaths finish: vector ownership preserves fused polygons without moving individual populated rings.
    std::vector<TipOracleLayer> cached_layers(object.auxiliary_layers().size());
    std::vector<size_t> top_order, bottom_order;
    size_t layer_index = 0;
    for (const Layer &layer : object.auxiliary_layers()) {
        TipOracleLayer &cached = cached_layers[layer_index];
        cached.bottom_z = unscaled(layer.scaled_bottom_z());
        cached.print_z = layer.unscaled_print_z();
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands()) {
                cached.regions.emplace_back();
                TipOracleRegion &copy = cached.regions.back();
                ExPolygons surfaces;
                for (const Surface &surface : region.fill_surfaces())
                    surfaces.push_back(surface.expolygon);
                copy.fused = offset2_ex(surfaces, float(SCALED_EPSILON), -float(SCALED_EPSILON));
                copy.islands.resize(copy.fused.size());
                for (const ExtrusionRole role : {LayerRegionIsland::SUPPORT, LayerRegionIsland::SUPPORT_INTERFACE})
                    if (region.has_extrusion(role))
                        copy_tip_paths(region.extrusion(role), copy.paths);
            }
        top_order.push_back(layer_index);
        bottom_order.push_back(layer_index++);
    }
    // Sort indices, not geometry; equal bounds retain publication order and slab bottoms need not be monotone with tops.
    std::stable_sort(top_order.begin(), top_order.end(), [&](size_t a, size_t b) { return cached_layers[a].print_z < cached_layers[b].print_z; });
    std::stable_sort(bottom_order.begin(), bottom_order.end(), [&](size_t a, size_t b) { return cached_layers[a].bottom_z < cached_layers[b].bottom_z; });
    size_t tips = 0, loose_ok = 0, island_ok = 0, centreline_deposit_tips = 0;
    std::ostringstream report;
    report << isolated_tips_name(fixture) << " thin=" << thin << " interfaces=" << interfaces << " layer_height=" << layer_height
           << " width=" << unscaled(width) << " min_medial_width="
           << unscaled(slic3r_api::Support::Toolpath::centreline_widths(flow, coord_t(scale_d(0.1))).min_medial_width) << " routed trees=" << routed.trees.size();
    for (const TF::RoutedTree &tree : routed.trees)
        for (const TF::TreeNode &node : tree.nodes) {
            if (!node.carries.empty() || node.layer + 1 >= object.layer_count())
                continue;
            ++tips;
            const Point centre = to_point(node.position);
            const double radius = unscaled(node.radius), reach = radius + tolerance;
            // The own slab: the lowest layer top at or above the tip node.
            double own_top = std::numeric_limits<double>::max();
            const auto own = std::lower_bound(top_order.begin(), top_order.end(), node.print_z - 0.01,
                [&](size_t id, double z) { return cached_layers[id].print_z < z; });
            if (own != top_order.end())
                own_top = cached_layers[*own].print_z;
            const auto window_top = std::lower_bound(top_order.begin(), top_order.end(), node.print_z - 0.4,
                [&](size_t id, double z) { return cached_layers[id].print_z < z; });
            const auto window_bottom = std::upper_bound(bottom_order.begin(), bottom_order.end(), node.print_z + 1.0,
                [&](double z, size_t id) { return z < cached_layers[id].bottom_z; });
            std::vector<size_t> window;
            // Filter the shorter bound-index range using the original inclusive window, then restore publication traversal.
            if (top_order.end() - window_top <= window_bottom - bottom_order.begin()) {
                for (auto it = window_top; it != top_order.end(); ++it)
                    if (!(cached_layers[*it].bottom_z > node.print_z + 1.0)) window.push_back(*it);
            } else {
                for (auto it = bottom_order.begin(); it != window_bottom; ++it)
                    if (!(cached_layers[*it].print_z < node.print_z - 0.4)) window.push_back(*it);
            }
            std::sort(window.begin(), window.end());
            std::vector<TipLayerFact> facts;
            double top_material = -1.;
            // The fused island of the own slab that holds the tip centre, else the nearest within the tip radius, and its paths.
            bool own_island = false;
            double own_island_distance = std::numeric_limits<double>::max();
            size_t own_island_paths = 0, own_island_deposits = 0;
            bool own_island_centreline = false;
            // Borrows cached geometry, filling lazy island measurements once; only described tips collect text/point facts.
            // Scalar results retain the original traversal and nearest-island tie order; failures propagate without mutation of paths.
            const auto scan = [&](bool describe) {
            facts.clear();
            top_material = -1.;
            own_island = false;
            own_island_distance = std::numeric_limits<double>::max();
            own_island_paths = 0;
            own_island_deposits = 0;
            own_island_centreline = false;
            for (const size_t id : window) {
                TipOracleLayer &layer = cached_layers[id];
                const double top_z = layer.print_z, bottom_z = layer.bottom_z;
                double nearest_path_mm = std::numeric_limits<double>::max();
                TipLayerFact *fact = nullptr;
                if (describe) {
                    facts.emplace_back();
                    fact = &facts.back();
                    fact->bottom_z = bottom_z;
                    fact->print_z = top_z;
                }
                for (TipOracleRegion &region : layer.regions) {
                    for (size_t island_id = 0; island_id < region.fused.size(); ++island_id) {
                        const ExPolygon &fused = region.fused[island_id];
                        if (distance_to_region_mm(centre, to_polygons(fused)) > reach)
                            continue;
                        size_t leaves = 0, deposits = 0;
                        TipOracleIsland &cached = region.islands[island_id];
                        if (describe || top_z == own_top) {
                            if (!cached.ready) {
                                // Grow once by the same boundary tolerance; count leaves individually, including deposits.
                                const ExPolygons grown = offset_ex(fused, float(scale_d(0.005)));
                                leaves_in_area(region.paths, grown, cached.leaves, cached.deposits);
                                PluginStorage class_storage;
                                cached.measure = slic3r_api::Support::Toolpath::classify_island(
                                    reinterpret_cast<storage_handle *>(&class_storage),
                                    slic3r_api::ExPolygon(reinterpret_cast<const expolygon_handle *>(&fused)), width);
                                cached.ready = true;
                            }
                            leaves = cached.leaves;
                            deposits = cached.deposits;
                        }
                        bool centreline = false;
                        if (describe || top_z == own_top) {
                            const slic3r_api::Support::Toolpath::BranchIslandMeasure &measure = cached.measure;
                            centreline = measure.kind == slic3r_api::Support::Toolpath::BranchIslandClass::Centreline;
                            if (describe) {
                                std::ostringstream text;
                                text << " [island " << unscaled(unscaled(fused.area())) << " mm2 "
                                     << (centreline ? "Centreline" : measure.kind == slic3r_api::Support::Toolpath::BranchIslandClass::Blob ? "Blob" : "Sheath")
                                     << " thin fraction " << measure.thin_fraction << " compactness " << measure.compactness;
                                if (centreline)
                                    text << " width " << island_width_mm(fused);
                                text << " paths in island " << leaves << " (deposits " << deposits << ")"
                                     << (centreline && leaves == 0 ? " CENTRELINE PRINTED NOTHING" : "") << "]";
                                fact->islands += text.str();
                            }
                        }
                        const double distance = fused.contains(centre) ? 0. : distance_to_region_mm(centre, to_polygons(fused));
                        if (top_z == own_top && distance <= radius && distance < own_island_distance) {
                            own_island = true;
                            own_island_distance = distance;
                            own_island_paths = leaves;
                            own_island_deposits = deposits;
                            own_island_centreline = centreline;
                        }
                    }
                    nearest_path_mm = std::min(nearest_path_mm, distance_to_paths_mm(region.paths, centre));
                    if (describe)
                        fact->points_in_circle += path_points_within(region.paths, centre, radius);
                }
                if (nearest_path_mm <= reach)
                    top_material = std::max(top_material, top_z);
                if (describe)
                    fact->nearest_path_mm = nearest_path_mm;
            }
            };
            scan(false);
            const bool loose = top_material >= node.print_z - 0.01, island_prints = own_island && own_island_paths > 0;
            loose_ok += loose ? 1 : 0;
            island_ok += island_prints ? 1 : 0;
            centreline_deposit_tips += own_island && own_island_centreline && own_island_deposits > 0 ? 1 : 0;
            if (!loose || !island_prints || tips == 1) {
                scan(true);
                report << "\n  tip layer " << node.layer << " z " << node.print_z << " r " << radius << " xy " << unscaled(centre.x()) << ","
                       << unscaled(centre.y()) << " loose " << loose << " island prints " << island_prints << " (own slab top " << own_top
                       << ", island found " << own_island << ", paths in it " << own_island_paths << ")";
                for (const TipLayerFact &fact : facts)
                    report << "\n    aux slab " << fact.bottom_z << ".." << fact.print_z << " nearest path "
                           << (fact.nearest_path_mm == std::numeric_limits<double>::max() ? -1. : fact.nearest_path_mm)
                           << " mm, points in circle " << fact.points_in_circle << fact.islands;
            }
        }
    size_t leaves = 0, deposits = 0;
    for (const Layer &layer : object.auxiliary_layers())
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands())
                for (const ExtrusionRole role : {LayerRegionIsland::SUPPORT, LayerRegionIsland::SUPPORT_INTERFACE})
                    if (region.has_extrusion(role))
                        count_leaves(region.extrusion(role), leaves, deposits);
    report << "\n  TIME routing and processing " << process_seconds << " s, whole case "
           << std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count() << " s";
    report << "\n  RESULT tips " << tips << " loose " << loose_ok << " island prints " << island_ok << " centreline islands with a deposit " << centreline_deposit_tips << " (support leaves " << leaves << ", deposits " << deposits << ")";
    // Printed unwrapped: Catch wraps its warnings at 80 columns, which makes the per-layer facts unreadable.
    std::fprintf(stderr, "%s\n", report.str().c_str());
    REQUIRE(tips > 0);
    // These fail when a tip has no material at its contact. The loose reach is the tip radius + 1.5 flow widths: the
    // medial line is trimmed where an island narrows, so the narrow end of a band that holds a tip may stay without material over
    // about 0.3 mm (an accepted limit).
    CHECK(loose_ok == tips);
    CHECK(island_ok == tips);
    // Integration proof of the deposit fallback: on the downward point without interface layers, some tip lies in a Centreline
    // island that has no line and prints a deposit instead (the facts show 14 to 18 such tips per run at these layer heights).
    if (fixture == 2 && thin && interfaces == 0 && (std::string(layer_height) == "0.2" || std::string(layer_height) == "0.15"))
        REQUIRE(centreline_deposit_tips > 0);
}
} // namespace

// Each thickness/interface combination is independently schedulable; every case retains all three layer heights.
TEST_CASE("Organic tips of an isolated arm leave material (thin, 0 interfaces)", "[plugins][support-tree-organic][support-tree-thin-tips]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *layer_height = GENERATE("0.2", "0.3", "0.15");
    check_isolated_tips(0, true, 0, layer_height);
}

// Runs owned thin-arm support at all three heights with three interfaces; helper assertions and failures reach Catch.
TEST_CASE("Organic tips of an isolated arm leave material (thin, 3 interfaces)", "[plugins][support-tree-organic][support-tree-thin-tips]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *layer_height = GENERATE("0.2", "0.3", "0.15");
    check_isolated_tips(0, true, 3, layer_height);
}

// Runs owned control-arm support at all three heights without interfaces; helper assertions and failures reach Catch.
TEST_CASE("Organic tips of an isolated arm leave material (control, 0 interfaces)", "[plugins][support-tree-organic][support-tree-thin-tips]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *layer_height = GENERATE("0.2", "0.3", "0.15");
    check_isolated_tips(0, false, 0, layer_height);
}

// Runs owned control-arm support at all three heights with three interfaces; helper assertions and failures reach Catch.
TEST_CASE("Organic tips of an isolated arm leave material (control, 3 interfaces)", "[plugins][support-tree-organic][support-tree-thin-tips]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *layer_height = GENERATE("0.2", "0.3", "0.15");
    check_isolated_tips(0, false, 3, layer_height);
}

// Runs owned thin-tab support at all three heights without interfaces; helper assertions and failures reach Catch.
TEST_CASE("Organic tips of an isolated tab leave material (thin, 0 interfaces)", "[plugins][support-tree-organic][support-tree-thin-tips]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *layer_height = GENERATE("0.2", "0.3", "0.15");
    check_isolated_tips(1, true, 0, layer_height);
}

// Runs owned thin-tab support at all three heights with three interfaces; helper assertions and failures reach Catch.
TEST_CASE("Organic tips of an isolated tab leave material (thin, 3 interfaces)", "[plugins][support-tree-organic][support-tree-thin-tips]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *layer_height = GENERATE("0.2", "0.3", "0.15");
    check_isolated_tips(1, true, 3, layer_height);
}

// Runs owned control-tab support at all three heights without interfaces; helper assertions and failures reach Catch.
TEST_CASE("Organic tips of an isolated tab leave material (control, 0 interfaces)", "[plugins][support-tree-organic][support-tree-thin-tips]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *layer_height = GENERATE("0.2", "0.3", "0.15");
    check_isolated_tips(1, false, 0, layer_height);
}

// Runs owned control-tab support at all three heights with three interfaces; helper assertions and failures reach Catch.
TEST_CASE("Organic tips of an isolated tab leave material (control, 3 interfaces)", "[plugins][support-tree-organic][support-tree-thin-tips]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *layer_height = GENERATE("0.2", "0.3", "0.15");
    check_isolated_tips(1, false, 3, layer_height);
}

// Runs owned thin-point support at all three heights without interfaces, including the helper's required deposit checks.
TEST_CASE("Organic tips of an isolated downward point leave material (thin, 0 interfaces)", "[plugins][support-tree-organic][support-tree-thin-tips]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *layer_height = GENERATE("0.2", "0.3", "0.15");
    check_isolated_tips(2, true, 0, layer_height);
}

// Runs owned thin-point support at all three heights with three interfaces; helper assertions and failures reach Catch.
TEST_CASE("Organic tips of an isolated downward point leave material (thin, 3 interfaces)", "[plugins][support-tree-organic][support-tree-thin-tips]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *layer_height = GENERATE("0.2", "0.3", "0.15");
    check_isolated_tips(2, true, 3, layer_height);
}

// Runs owned control-point support at all three heights without interfaces; helper assertions and failures reach Catch.
TEST_CASE("Organic tips of an isolated downward point leave material (control, 0 interfaces)", "[plugins][support-tree-organic][support-tree-thin-tips]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *layer_height = GENERATE("0.2", "0.3", "0.15");
    check_isolated_tips(2, false, 0, layer_height);
}

// Runs owned control-point support at all three heights with three interfaces; helper assertions and failures reach Catch.
TEST_CASE("Organic tips of an isolated downward point leave material (control, 3 interfaces)", "[plugins][support-tree-organic][support-tree-thin-tips]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *layer_height = GENERATE("0.2", "0.3", "0.15");
    check_isolated_tips(2, false, 3, layer_height);
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
        const Polygons &area = to_native(slab.polygons);
        if (area.empty()) return;
        for (size_t id = 0; id < object.layer_count(); ++id) {
            const Layer &layer = object.layer(id);
            if (layer.scaled_bottom_z() >= slab.print_z || layer.scaled_print_z() <= slab.bottom_z) continue;
            const Polygons keep_out = to_polygons(offset_ex(layer.lslices(), float(settings.xy_distance - scale_d(tolerance_mm))));
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
        CHECK_FALSE(to_native(contact->polygons).empty());
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
        for (const Polygon &polygon : to_native(contact->polygons))
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
                CHECK(distance_to_region_mm(to_point(node.position), to_native(slab->polygons)) <= tolerance_mm);
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
    const Common::SupportParams params = SG::params(print, object);
    const TF::TreeSettings settings = Slic3r::Test::SupportTree::tree_settings(print, object, params);
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
    CAPTURE(block_top, root_layer, centre.x(), centre.y(), settings.branch_radius);
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
    TF::generate_tree_slabs(nullptr, SG::print_view(print), SG::object_view(object), params, settings, trees, false, support, std::vector<TF::RestingPart>{});
    const coord_t block_z = object.layer(block_top).scaled_print_z(), root_z = object.layer(root_layer).scaled_print_z();
    REQUIRE_FALSE(support.bottom.empty());
    for (const Common::GeneratorLayer *contact : support.bottom) {
        CHECK(contact->print_z > block_z);
        CHECK(contact->print_z < root_z);
        CHECK(distance_to_region_mm(centre, to_native(contact->polygons)) <= tolerance_mm);
    }
}

// A root over free air that nothing ever supports goes down five root radii of real Z, whatever the layer heights. In a 0.05 mm
// band that is many more layers than the nominal 0.2 mm height gives: with a root radius of 0.4 mm (the branch radius) the depth is
// 2 mm, which is 40 layers of 0.05 mm (10 nominal layers). The root keeps its full area (it never meets the object, and its area
// is above the stop area), so its stem ends at the lowest layer whose top is within 2 mm of the root layer top; that layer is
// computed here from the real Z of the layers (about 5.5 - 2 = 3.5 mm, in the thin band), and must differ from the nominal
// reconstruction (10 layers down, which are only 0.5 mm of thin layers: about 5.0 mm). The stem is observed in the base slabs:
// the lowest slab that draws it has its middle between the stem bottom and one maximum slab height above.
TEST_CASE("Tree roots in thin layers go down five root radii of real Z", "[plugins][support-tree-family][floors]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    // Only a far column gives the object layers; the root stands in free air away from it.
    TriangleMesh column = make_cube(2., 10., 7.);
    DynamicPrintConfig config = SG::config("tree", false, 0);
    config.set_deserialize_strict({{"support_tree_branch_diameter", "0.8"}, {"support_tree_tip_diameter", "0.4"}});
    Print print;
    Model model;
    SG::init_print_with_profile(print, model, std::move(column), std::vector<coordf_t>{0., 0.2, 2.9, 0.2, 3., 0.05, 6., 0.05, 6.1, 0.2, 7., 0.2}, config);
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    const Common::SupportParams params = SG::params(print, object);
    const TF::TreeSettings settings = Slic3r::Test::SupportTree::tree_settings(print, object, params);
    // The root layer is the last thin layer under z = 5.5 mm, 2.5 mm of thin layers above the start of the band.
    size_t root_layer = 0;
    for (size_t id = 0; id < object.layer_count(); ++id)
        if (object.layer(id).unscaled_height() < 0.1 && object.layer(id).unscaled_print_z() < 5.5)
            root_layer = id;
    REQUIRE(root_layer > 40);
    const coord_t radius = settings.branch_radius, depth = coord_t(5. * double(radius));
    // The real depth: the lowest layer whose top is within the depth below the root layer top, counted from the real Z.
    size_t expected = root_layer;
    while (expected > 0 && object.layer(root_layer).scaled_print_z() - object.layer(expected - 1).scaled_print_z() <= depth)
        --expected;
    // The nominal reconstruction counts the nominal layer heights instead.
    const size_t nominal = root_layer - size_t(5. * double(radius) / double(scale_to_layer_coord(object.config().layer_height.value)));
    CAPTURE(root_layer, expected, nominal);
    REQUIRE(expected + 20 < nominal);
    REQUIRE(expected > 0);
    const BoundingBox extents = get_extents(object.layer(0).lslices());
    TF::TreeNode upper;
    upper.layer = root_layer + 1;
    upper.print_z = object.layer(root_layer + 1).unscaled_print_z();
    upper.position = to_c_point(Point(extents.min.x() - scale_i(10.), extents.center().y()));
    upper.radius = radius;
    TF::TreeNode root = upper;
    root.layer = root_layer;
    root.print_z = object.layer(root_layer).unscaled_print_z();
    root.carries = { 0 };
    PluginStorage storage;
    Common::GeneratedSupport support{ reinterpret_cast<storage_handle *>(&storage) };
    // The top contact over the tip gives the base slabs below it, as the routing would: the stem is drawn in those slabs.
    const coord_t contact_top = object.layer(root_layer + 1).scaled_print_z();
    Common::GeneratorLayer &contact = Common::add_layer(support, Common::LayerType::TopContact, contact_top,
        contact_top - scale_to_layer_coord(0.1), scale_to_layer_coord(0.1));
    support.top = { &contact };
    Polygon tip_circle = make_circle(double(radius), 1000.);
    tip_circle.translate(to_point(upper.position));
    assign_native(contact.polygons, Polygons{ tip_circle });
    std::vector<TF::RoutedTree> trees{ TF::RoutedTree{ { upper, root } } };
    TF::generate_tree_slabs(nullptr, SG::print_view(print), SG::object_view(object), params, settings, trees, false, support, std::vector<TF::RestingPart>{});
    REQUIRE_FALSE(support.base.empty());
    // The lowest slab that draws the stem: its middle lies within the stem, so the slab below it has its middle under the stem
    // bottom. The slabs are at most the maximum slab height thick.
    const coord_t stem_bottom = object.layer(expected).scaled_bottom_z(), nominal_bottom = object.layer(nominal).scaled_bottom_z();
    coord_t lowest_drawn = std::numeric_limits<coord_t>::max();
    for (const Common::GeneratorLayer *slab : support.base)
        if (distance_to_region_mm(to_point(upper.position), to_native(slab->polygons)) <= tolerance_mm)
            lowest_drawn = std::min(lowest_drawn, slab->bottom_z + (slab->print_z - slab->bottom_z) / 2);
    REQUIRE(lowest_drawn != std::numeric_limits<coord_t>::max());
    CAPTURE(stem_bottom, nominal_bottom, lowest_drawn, settings.max_slab_height);
    CHECK(lowest_drawn >= stem_bottom);
    CHECK(lowest_drawn <= stem_bottom + settings.max_slab_height);
    CHECK(lowest_drawn < nominal_bottom);
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
    assign_native(lower.polygons, Polygons{ make_circle(scale_d(2.), 1000.) });
    Polygon thin_circle = make_circle(scale_d(0.5), 1000.);
    thin_circle.translate(centre);
    assign_native(thin.polygons, Polygons{ thin_circle });
    assign_native(upper.polygons, Polygons{ make_circle(scale_d(1.), 1000.) });
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
    Common::publish(nullptr, SG::object_view(object), SG::params(print, object), support, reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
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
    assign_native(floor_contact.polygons, Polygons{ floor_circle });
    assign_native(overhang_contact.polygons, Polygons{ overhang_circle });
    TF::align_slab_bounds(object_zs, support);
    Common::publish(nullptr, SG::object_view(object), SG::params(print, object), support, reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
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
    const Common::SupportParams params = SG::params(print, object);
    const TF::TreeSettings settings = Slic3r::Test::SupportTree::tree_settings(print, object, params);
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
    TF::generate_tree_slabs(nullptr, SG::print_view(print), SG::object_view(object), params, settings, trees, false, support, std::vector<TF::RestingPart>{});
    REQUIRE_FALSE(support.bottom.empty());
    const Common::GeneratorLayer &floor = *support.bottom.front();
    const Polygons footprint = to_native(floor.polygons);
    REQUIRE_FALSE(footprint.empty());
    const coord_t object_top = object.layer(floor.object_below).scaled_print_z();
    for (const Common::GeneratorLayer *slab : support.base) {
        const Polygons &area = to_native(slab->polygons);
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
    const Common::SupportParams params = SG::params(print, object);
    const TF::TreeSettings settings = Slic3r::Test::SupportTree::tree_settings(print, object, params);
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
    TF::generate_tree_slabs(nullptr, SG::print_view(print), SG::object_view(object), params, settings, probe_trees, true, probe, std::vector<TF::RestingPart>{});
    REQUIRE(probe.bottom.size() == 1);
    const coord_t floor_top = probe.bottom.front()->print_z, floor_height = probe.bottom.front()->height;
    REQUIRE(unscaled(floor_top) == Approx(4.4));
    PluginStorage storage;
    Common::GeneratedSupport support{ reinterpret_cast<storage_handle *>(&storage) };
    Common::GeneratorLayer &other = Common::add_layer(support, Common::LayerType::TopContact, floor_top,
        floor_top - floor_height - scale_to_layer_coord(0.1), floor_height + scale_to_layer_coord(0.1));
    support.top = { &other };
    std::vector<TF::RoutedTree> trees = make_trees();
    TF::generate_tree_slabs(nullptr, SG::print_view(print), SG::object_view(object), params, settings, trees, true, support, std::vector<TF::RestingPart>{});
    Common::publish(nullptr, SG::object_view(object), params, support, reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
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

// Both organic and tree branch bodies publish the same sparse support type, leaving algorithm selection to the object.
TEST_CASE("Organic and tree branches publish sparse support surfaces", "[plugins][support-tree-family][publication]")
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
    const SurfaceType sparse_type = stModSupport | stPosInternal | stDensSparse;
    size_t sparse = 0;
    for (const Layer &layer : print.object(0).auxiliary_layers())
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands())
                for (const Surface &surface : region.fill_surfaces()) {
                    sparse += surface.surface_type == sparse_type;
                    CHECK((uint16_t(surface.surface_type) & (1 << 10)) == 0);
                }
    CHECK(sparse > 0);
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
        {"gcode_flavor", "marlin2"},
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
        Orchestrator::instance().export_gcode(print, path.string(), nullptr);
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

// The family declares its geometry and tree branch keys, and it ignores a requested raft without a warning and without raft layers.
TEST_CASE("Tree family declares its keys and ignores a raft without a warning", "[plugins][support-tree-family][family]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    REQUIRE(Orchestrator::instance().is_plugin_active("support.family.tree"));
    const Plugin &provider = *Orchestrator::instance().get_plugin("support.family.tree");
    for (const char *expected_key : {"support_max_overhang", "support_tree_branch_diameter", "support_tree_top_rate",
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
    CHECK(warnings == 0);
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
// The outlines as plugin collections of the storage, the keep-out form that smooth_organic_branches() reads.
std::vector<slic3r_api::StoredPolygonCollection> stored_outlines(PluginStorage &storage, const std::vector<Polygons> &outlines)
{
    std::vector<slic3r_api::StoredPolygonCollection> stored;
    stored.reserve(outlines.size());
    for (const Polygons &outline : outlines) {
        stored.emplace_back(reinterpret_cast<storage_handle *>(&storage));
        assign_native(stored.back(), outline);
    }
    return stored;
}
// Copies native layer tops into an owned millimeter table for borrowed sliced objects; layer order is preserved and allocation failures propagate.
std::vector<double> layer_print_z_mm(const PrintObject &object)
{
    std::vector<double> tops;
    tops.reserve(object.layer_count());
    for (size_t id = 0; id < object.layer_count(); ++id)
        tops.push_back(object.layer(id).unscaled_print_z());
    return tops;
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
    PluginStorage storage;
    const std::vector<slic3r_api::StoredPolygonCollection> keep_out = stored_outlines(storage, { square_mm(0., 0., 10.) });
    const Vec2d center(10. + radius_mm - penetration_mm, 5.);
    const Vec2d pushed = TF::push_sphere_out_of_outline(0, radius_mm, center, print_z, keep_out);
    const double expected_mm = std::min(penetration_mm + gap_mm, cap_mm);
    CAPTURE(pushed.x(), pushed.y(), expected_mm);
    CHECK(std::abs((pushed.x() - center.x()) - expected_mm) <= tolerance_mm);
    CHECK(std::abs(pushed.y() - center.y()) <= tolerance_mm);
}

// At the centre of a square all four sides are exactly as close. nearest_on_lines keeps the first line of the outline (the bottom side,
// the ring starts at its lower-left corner), so the sphere is pushed up by the same nudge distance as before; the native tree met another
// side first (accepted difference, 2026-10-13). The distance of the push does not depend on the side chosen.
TEST_CASE("Organic push at the centre of a square follows the first side of the outline", "[plugins][support-tree-organic][smoothing]")
{
    const double radius_mm = 5.2;
    const std::vector<double> print_z{ 0.2 };
    PluginStorage storage;
    const std::vector<slic3r_api::StoredPolygonCollection> keep_out = stored_outlines(storage, { square_mm(0., 0., 10.) });
    const Vec2d pushed = TF::push_sphere_out_of_outline(0, radius_mm, Vec2d(5., 5.), print_z, keep_out);
    CAPTURE(pushed.x(), pushed.y());
    CHECK(std::abs(pushed.x() - 5.) <= tolerance_mm);
    CHECK(std::abs(pushed.y() - 5.3) <= tolerance_mm);
}

// Smoothing moves the free nodes of a bent organic branch, and no node ends up inside the object outline grown by the XY
// distance. A node with a radius keeps its circle clear of the outline, within the tolerance.
TEST_CASE("Organic smoothing moves the nodes of a bent branch and keeps them out of the object",
    "[plugins][support-tree-organic][smoothing]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = SG::config("organic", false, 0);
    config.set_deserialize_strict({{"support_material_buildplate_only", "1"}, {"support_max_overhang", "0.2"}});
    Print print;
    Model model;
    Test::init_print({diagonal_roof()}, print, model, config);
    SG::prefix(print);
    const Routed routed(print);
    const PrintObject &object = print.object(0);
    const std::vector<Polygons> keep_out = keep_out_outlines(object, routed.settings.xy_distance);
    PluginStorage storage;
    const std::vector<slic3r_api::StoredPolygonCollection> stored_keep_out = stored_outlines(storage, keep_out);
    std::vector<TF::RoutedTree> smoothed = routed.trees;
    TF::smooth_organic_branches(layer_print_z_mm(object), stored_keep_out, smoothed, nullptr);
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
    PluginStorage storage;
    const std::vector<slic3r_api::StoredPolygonCollection> stored_keep_out = stored_outlines(storage, keep_out);
    std::vector<TF::RoutedTree> trees{ bent_branch(object) };
    // Guard: the fixture starts clear of the outline, so the branch is collision-free.
    for (const TF::TreeNode &node : trees.front().nodes)
        REQUIRE(distance_to_region_mm(to_point(node.position), keep_out[node.layer]) > unscaled(node.radius));
    const std::vector<TF::TreeNode> raw = trees.front().nodes;
    const double bend_before = sharpest_bend_deg(trees.front());
    const size_t iterations = TF::smooth_organic_branches(layer_print_z_mm(object), stored_keep_out, trees, nullptr);
    const double bend_after = sharpest_bend_deg(trees.front());
    size_t moved = 0;
    for (size_t idx = 0; idx < raw.size(); ++idx)
        moved += to_point(trees.front().nodes[idx].position) != to_point(raw[idx].position) ? 1 : 0;
    CAPTURE(iterations, bend_before, bend_after, moved);
    CHECK(moved == 3);
    CHECK(bend_after <= 0.5 * bend_before);
}

namespace {
// A straight inclined branch (1.5 mm of X per mm of Z) through the given layers, listed from the tip down to the root, far from any
// outline. The smoothing aims at the line between the links at the Z of the node, so a branch already on a straight line stays where
// it is within 1 um, whatever the spacing of its layers. Returns the largest distance in mm between a node and its raw position.
double straight_branch_drift_mm(const Print &print, const std::vector<size_t> &layers)
{
    const PrintObject &object = print.object(0);
    std::vector<ChainPoint> points;
    for (const size_t layer : layers)
        points.push_back({ 1.5 * object.layer(layer).unscaled_print_z(), 0., layer });
    const std::vector<Polygons> keep_out(object.layer_count(), square_mm(-30., -30., 10.));
    PluginStorage storage;
    const std::vector<slic3r_api::StoredPolygonCollection> stored_keep_out = stored_outlines(storage, keep_out);
    std::vector<TF::RoutedTree> trees{ chain_tree(object, points) };
    // Guard: every node between the tip and the root is free, with a node above and a node below, so the pull moves it.
    const std::vector<size_t> below = TF::nodes_below(trees.front());
    for (size_t idx = 1; idx + 1 < points.size(); ++idx) {
        REQUIRE(!trees.front().nodes[idx].carries.empty());
        REQUIRE(below[idx] != TF::no_node);
    }
    const std::vector<TF::TreeNode> raw = trees.front().nodes;
    TF::smooth_organic_branches(layer_print_z_mm(object), stored_keep_out, trees, nullptr);
    double drift_mm = 0.;
    for (size_t idx = 0; idx < raw.size(); ++idx)
        drift_mm = std::max(drift_mm, unscaled((to_point(trees.front().nodes[idx].position) - to_point(raw[idx].position)).cast<double>().norm()));
    return drift_mm;
}
} // namespace

// The pull aims at the point of the line between the node below and the node above that lies at the Z of the node, not at their
// midpoint, so a straight inclined branch stays straight when its layers are unequal: here a 0.05 mm band then a 0.3 mm band, with
// the free nodes on both sides of the change. On uniform layers the target is the midpoint and the branch stays too.
TEST_CASE("Organic smoothing keeps a straight inclined branch straight on unequal layer heights", "[plugins][support-tree-organic][smoothing]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    SECTION("a 0.05 mm band then a 0.3 mm band") {
        Print print;
        Model model;
        SG::init_print_with_profile(print, model, make_cube(10., 10., 9.), { 0., 0.05, 2., 0.05, 2.01, 0.3, 9., 0.3 }, SG::config("organic", false, 0));
        SG::prefix(print);
        const PrintObject &object = print.object(0);
        size_t band = 0;
        while (object.layer(band).unscaled_print_z() < 2.)
            ++band;
        REQUIRE(band > 6);
        REQUIRE(band + 3 < object.layer_count());
        // Tip to root; the free nodes lie at band - 2, band and band + 1. From the root up the four gaps are 0.2, 0.1, 0.3 and
        // 0.6 mm, so the node at band has 0.1 mm below and 0.3 mm above.
        const std::vector<size_t> layers{ band + 3, band + 1, band, band - 2, band - 6 };
        const double gap_a = object.layer(layers[1]).unscaled_print_z() - object.layer(layers[2]).unscaled_print_z();
        const double gap_b = object.layer(layers[2]).unscaled_print_z() - object.layer(layers[3]).unscaled_print_z();
        REQUIRE(std::abs(gap_a - gap_b) > 0.05);
        const double drift_mm = straight_branch_drift_mm(print, layers);
        CAPTURE(drift_mm);
        CHECK(drift_mm <= 0.001);
    }
    SECTION("uniform layers") {
        Print print;
        Model model;
        Test::init_print({SG::slab()}, print, model, SG::config("organic", false, 0));
        SG::prefix(print);
        const double drift_mm = straight_branch_drift_mm(print, { 20, 16, 12, 8, 4 });
        CAPTURE(drift_mm);
        CHECK(drift_mm <= 0.001);
    }
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
    PluginStorage storage;
    const std::vector<slic3r_api::StoredPolygonCollection> stored_keep_out = stored_outlines(storage, keep_out);
    // The count is exact: the relaxation is deterministic, so a change of the threshold or of the rule moves it.
    SECTION("a bend that settles stops after its exact count") {
        std::vector<TF::RoutedTree> trees{ bent_branch(object) };
        const size_t iterations = TF::smooth_organic_branches(layer_print_z_mm(object), stored_keep_out, trees, nullptr);
        CAPTURE(iterations);
        CHECK(iterations == 33);
    }
    SECTION("a node that keeps moving runs to the cap") {
        std::vector<TF::RoutedTree> trees{ stray_node_branch(object) };
        const size_t iterations = TF::smooth_organic_branches(layer_print_z_mm(object), stored_keep_out, trees, nullptr);
        const double x_mm = unscaled(trees.front().nodes[1].position.x);
        CAPTURE(iterations, x_mm);
        CHECK(iterations == 100);
        CHECK(std::abs(x_mm - 5.) <= tolerance_mm);
    }
}

// A cancellation of the context ends the relaxation before anything is written back, so the trees keep their positions.
TEST_CASE("Organic relaxation leaves the trees unchanged when the context is cancelled", "[plugins][support-tree-organic][smoothing]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    Test::init_print({SG::slab()}, print, model, SG::config("organic", false, 0));
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    const std::vector<Polygons> keep_out(object.layer_count(), square_mm(-30., -30., 10.));
    PluginStorage storage;
    const std::vector<slic3r_api::StoredPolygonCollection> stored_keep_out = stored_outlines(storage, keep_out);
    std::vector<TF::RoutedTree> trees{ bent_branch(object) };
    const std::vector<TF::TreeNode> raw = trees.front().nodes;
    // The context cancels at its second check, after the second iteration: one full pass ran and nothing was written back.
    int calls = 0;
    plugin_run_context context = {};
    context.host_context = reinterpret_cast<plugin_host_context *>(&calls);
    context.is_cancelled = [](plugin_host_context *host_context) { return ++*reinterpret_cast<int *>(host_context) >= 2 ? 1 : 0; };
    REQUIRE_THROWS_AS(TF::smooth_organic_branches(layer_print_z_mm(object), stored_keep_out, trees, &context), slic3r_api::PluginCancelled);
    CHECK(calls == 2);
    for (size_t idx = 0; idx < raw.size(); ++idx)
        CHECK(to_point(trees.front().nodes[idx].position) == to_point(raw[idx].position));
}

// The last pull of the relaxation is not measured against the outline. The free node of the stray branch walks 0.2 mm per iteration
// from x = 25 mm to x = 5.0 mm in the 100 iterations of the cap, and its circle (0.4 mm) clears a wall whose face is at x = 4.7 mm by 0.5 mm
// at the start of the last iteration (no push), then reaches 0.1 mm into it with the last pull. The final pass pushes it out: its
// clearance is then at least its radius, within the tolerance.
TEST_CASE("Organic smoothing pushes out a node that the last pull left inside the outline", "[plugins][support-tree-organic][smoothing]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    Test::init_print({SG::slab()}, print, model, SG::config("organic", false, 0));
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    const std::vector<Polygons> keep_out(object.layer_count(), square_mm(2.7, -1., 2.));
    PluginStorage storage;
    const std::vector<slic3r_api::StoredPolygonCollection> stored_keep_out = stored_outlines(storage, keep_out);
    std::vector<TF::RoutedTree> trees{ stray_node_branch(object) };
    const TF::TreeNode &free_node = trees.front().nodes[1];
    const size_t iterations = TF::smooth_organic_branches(layer_print_z_mm(object), stored_keep_out, trees, nullptr);
    const double clearance_mm = distance_to_region_mm(to_point(free_node.position), keep_out[free_node.layer]);
    CAPTURE(iterations, unscaled(free_node.position.x), clearance_mm);
    // The cap ended the relaxation, so the last pull was the one that went into the wall.
    CHECK(iterations == 100);
    CHECK(clearance_mm >= unscaled(free_node.radius) - tolerance_mm);
}

// A cancellation at the check that follows the final pass, the last one of the function, ends it before anything is written back.
TEST_CASE("Organic smoothing leaves the trees unchanged when cancelled after its final pass", "[plugins][support-tree-organic][smoothing]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    Test::init_print({SG::slab()}, print, model, SG::config("organic", false, 0));
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    const std::vector<Polygons> keep_out(object.layer_count(), square_mm(-30., -30., 10.));
    PluginStorage storage;
    const std::vector<slic3r_api::StoredPolygonCollection> stored_keep_out = stored_outlines(storage, keep_out);
    // The relaxation of this branch settles after a known number of iterations, each of them followed by one check.
    std::vector<TF::RoutedTree> reference{ bent_branch(object) };
    const size_t iterations = TF::smooth_organic_branches(layer_print_z_mm(object), stored_keep_out, reference, nullptr);
    REQUIRE(iterations > 1);
    std::vector<TF::RoutedTree> trees{ bent_branch(object) };
    const std::vector<TF::TreeNode> raw = trees.front().nodes;
    struct Counter { size_t calls = 0, cancel_at = 0; } counter{ 0, iterations + 1 };
    plugin_run_context context = {};
    context.host_context = reinterpret_cast<plugin_host_context *>(&counter);
    context.is_cancelled = [](plugin_host_context *host_context) {
        Counter &c = *reinterpret_cast<Counter *>(host_context);
        return ++c.calls >= c.cancel_at ? 1 : 0;
    };
    REQUIRE_THROWS_AS(TF::smooth_organic_branches(layer_print_z_mm(object), stored_keep_out, trees, &context), slic3r_api::PluginCancelled);
    CHECK(counter.calls == iterations + 1);
    for (size_t idx = 0; idx < raw.size(); ++idx)
        CHECK(to_point(trees.front().nodes[idx].position) == to_point(raw[idx].position));
    // The trees of the reference run moved: the check above is not about an unmoved branch.
    CHECK(to_point(reference.front().nodes[3].position) != to_point(raw[3].position));
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
    config.set_deserialize_strict({{"support_material_buildplate_only", "1"}, {"support_max_overhang", std::to_string(limit_mm)}});
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
    config.set_deserialize_strict({{"support_material_buildplate_only", "1"}, {"support_max_overhang", std::to_string(limit_mm)}});
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
    PluginStorage storage;
    const std::vector<slic3r_api::StoredPolygonCollection> sections = TF::slice_branch_meshes(reinterpret_cast<storage_handle *>(&storage), { tree }, zs, nullptr);
    REQUIRE(sections.size() == zs.size());
    // Each cap is the half of a sphere of radius 0.4 mm on the side of its normal, cut 0.3 mm from its centre: a disc of radius 0.2646
    // mm, of which the tilted half keeps about 0.21 mm2. The faceted rings of the cap keep at least half of the disc.
    for (size_t idx = 0; idx < zs.size(); ++idx) {
        const double area_mm2 = unscaled(unscaled(area(to_native(sections[idx]))));
        CAPTURE(idx, area_mm2);
        CHECK(area_mm2 >= 0.5 * M_PI * 0.07);
        CHECK(area_mm2 <= M_PI * 0.07);
    }
}

namespace {
// A tilted tree of three nodes (one path, one ring in the middle) whose lowest node is at the given position, in millimeters.
TF::RoutedTree chain_tree(double x_mm, double y_mm)
{
    TF::RoutedTree tree;
    tree.nodes.resize(3);
    for (size_t idx = 0; idx < 3; ++idx) {
        TF::TreeNode &node = tree.nodes[idx];
        const size_t level = 2 - idx;
        node.layer = 5 + level;
        node.print_z = 1.0 + 0.2 * double(level);
        node.position = to_c_point(Point::new_scale(x_mm + 0.1 * double(level), y_mm));
        node.radius = scale_to_layer_coord(0.4);
        if (idx > 0)
            node.carries = { idx - 1 };
    }
    return tree;
}

// The slice Z values (scaled) that cross the chain trees, from the bottom cap to the top cap.
std::vector<coord_t> chain_zs()
{
    std::vector<coord_t> zs;
    for (const double z : { 0.7, 0.9, 1.1, 1.3, 1.5, 1.7 })
        zs.push_back(scale_to_layer_coord(z));
    return zs;
}

// True when both lists have the same rings in the same order, each pair differing by less than 1e-4 square millimeters. The
// fixtures give one ring per tree and per Z, so the order of the rings is the order of the trees. The slicer does not give the same start point and the same last digit of a ring on every call (its parallel workers add the
// segments in a varying order), so the rings are not compared point by point.
bool same_rings(const Polygons &a, const Polygons &b)
{
    if (a.size() != b.size())
        return false;
    for (size_t idx = 0; idx < a.size(); ++idx)
        if (unscaled(unscaled(area(diff(Polygons{ a[idx] }, Polygons{ b[idx] })) + area(diff(Polygons{ b[idx] }, Polygons{ a[idx] })))) > 1e-4)
            return false;
    return true;
}
} // namespace

// The sections of every slice Z do not depend on how the Z values are grouped: one call with all of them gives what one call per Z
// gives (the mesh of a path is sliced once for all its Z values, which is the point of the single call).
TEST_CASE("Branch meshes slice at many Z in one call as at one Z per call", "[plugins][support-tree-organic][mesh]")
{
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    const std::vector<TF::RoutedTree> trees = { chain_tree(0., 0.) };
    const std::vector<coord_t> zs = chain_zs();
    const std::vector<slic3r_api::StoredPolygonCollection> all = TF::slice_branch_meshes(handle, trees, zs, nullptr);
    REQUIRE(all.size() == zs.size());
    for (size_t idx = 0; idx < zs.size(); ++idx) {
        const std::vector<slic3r_api::StoredPolygonCollection> one = TF::slice_branch_meshes(handle, trees, { zs[idx] }, nullptr);
        REQUIRE(one.size() == 1);
        CAPTURE(idx);
        CHECK_FALSE(all[idx].empty());
        CHECK(same_rings(to_native(all[idx]), to_native(one.front())));
    }
}

// The trees are sliced in parallel but the sections are joined in tree order: the polygons of the first tree come first at every Z,
// whatever the thread that finishes first, and a Z that no tree crosses has an empty collection.
TEST_CASE("Branch meshes of several trees are joined in tree order", "[plugins][support-tree-organic][mesh]")
{
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    std::vector<TF::RoutedTree> trees;
    for (int idx = 0; idx < 6; ++idx)
        trees.push_back(chain_tree(3. * idx, 0.));
    std::vector<coord_t> zs = chain_zs();
    zs.push_back(scale_to_layer_coord(5.));
    const std::vector<slic3r_api::StoredPolygonCollection> together = TF::slice_branch_meshes(handle, trees, zs, nullptr);
    REQUIRE(together.size() == zs.size());
    std::vector<Polygons> expected(zs.size());
    for (const TF::RoutedTree &tree : trees) {
        const std::vector<slic3r_api::StoredPolygonCollection> alone = TF::slice_branch_meshes(handle, { tree }, zs, nullptr);
        for (size_t idx = 0; idx < zs.size(); ++idx) {
            const Polygons polygons = to_native(alone[idx]);
            expected[idx].insert(expected[idx].end(), polygons.begin(), polygons.end());
        }
    }
    for (size_t idx = 0; idx < zs.size(); ++idx) {
        CAPTURE(idx);
        CHECK(same_rings(to_native(together[idx]), expected[idx]));
    }
    CHECK(together.back().empty());
}

// A cancelled context stops the slicing before the first job.
TEST_CASE("Branch meshes stop when the context is cancelled", "[plugins][support-tree-organic][mesh]")
{
    PluginStorage storage;
    plugin_run_context context = {};
    context.is_cancelled = [](plugin_host_context *) { return 1; };
    const std::vector<TF::RoutedTree> trees = { chain_tree(0., 0.), chain_tree(3., 0.) };
    CHECK_THROWS_AS(TF::slice_branch_meshes(reinterpret_cast<storage_handle *>(&storage), trees, chain_zs(), &context), slic3r_api::PluginCancelled);
}

// The cancellation is also checked before each path of a tree. One tree of two paths (a split) is sliced with a callback that
// allows the check of the job and the check before the first path, then asks to stop: the check before the second path throws.
// Without the per-path check the callback would be called once and the slicing would finish.
TEST_CASE("Branch meshes check the cancellation before each path", "[plugins][support-tree-organic][mesh]")
{
    TF::RoutedTree tree;
    tree.nodes.resize(3);
    for (size_t idx = 0; idx < 3; ++idx) {
        TF::TreeNode &node = tree.nodes[idx];
        node.layer = idx == 0 ? 5 : 6;
        node.print_z = idx == 0 ? 1.0 : 1.2;
        node.position = to_c_point(Point::new_scale(idx == 0 ? 0. : (idx == 1 ? 0.2 : -0.2), 0.));
        node.radius = scale_to_layer_coord(0.4);
    }
    tree.nodes[0].carries = { 1, 2 };
    PluginStorage storage;
    int calls = 0;
    plugin_run_context context = {};
    context.host_context = reinterpret_cast<plugin_host_context *>(&calls);
    context.is_cancelled = [](plugin_host_context *host_context) { return ++*reinterpret_cast<int *>(host_context) > 2 ? 1 : 0; };
    CHECK_THROWS_AS(TF::slice_branch_meshes(reinterpret_cast<storage_handle *>(&storage), { tree }, chain_zs(), &context), slic3r_api::PluginCancelled);
    // The job check, the first path, then the second path that stopped.
    CHECK(calls == 3);
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
        params, slab.print_z, slab, params.gap_xy);
    band.footprint = union_ex(to_native(footprint));
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
    return unscaled(unscaled(area(intersection_ex(to_native(slab.polygons), to_polygons(band.footprint)))));
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
    TF::generate_tree_slabs(nullptr, SG::print_view(print), SG::object_view(object), routed.params, routed.settings, routed.trees, organic, routed.support, routed.resting);
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
    TF::generate_tree_slabs(nullptr, SG::print_view(print), SG::object_view(object), routed.params, routed.settings, routed.trees, organic, routed.support, routed.resting);
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
    TF::generate_tree_slabs(nullptr, SG::print_view(print), SG::object_view(object), routed.params, routed.settings, routed.trees, organic, routed.support, routed.resting);
    size_t interfaces_in_band = 0;
    for (const Common::GeneratorLayer *slab : routed.support.interfaces)
        interfaces_in_band += in_thread_band(*slab, band, gap_above) && !slab->polygons.empty() ? 1 : 0;
    REQUIRE(interfaces_in_band > 1);
    CHECK(count_slabs_in_band(routed.support, band, gap_above) >= interfaces_in_band);
}

// Print::validate rejects a support branch diameter that the support extrusion width cannot print or that is smaller than the
// tip, for the tree and the organic styles alike (both run the tree family), and accepts sane diameters for both. A tip thinner
// than the extrusion width is rejected for the tree style only: its sections are sparse-filled Base surfaces, which drop areas
// narrower than half a spacing, while the organic sparse surfaces print such sections as a deposit or along their medial axis.
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

namespace {
// The Z gaps of one clearance scenario, as the support_material_contact_distance (above the support) and
// support_material_bottom_contact_distance (under it) settings, and whether the object has a variable layer height profile.
struct GapScenario {
    const char *above, *below;
    bool variable_layers;
    // Whether the object holds a floor in this scenario: a gap thinner than the layers leaves no bottom contact.
    bool floors;
};

// The short band of 0.05 mm layers of the variable profile, in mm: it holds the low roof of the shelf mesh (2.0 to 2.4 mm), so the
// gap window of its top contact falls on thin layers, and ends on the 0.05 to 0.3 mm transition.
constexpr double thin_band_start_mm = 1.7, thin_band_end_mm = 2.3;

// Slices the mesh with the given style, interface layer count and gaps, routes the trees and generates the final slabs. Every
// non-empty slab is then compared with every object layer: a layer that overlaps the slab in XY must lie above or below it, at
// least the gap on that side away (a gap not larger than the Z tolerance counts as zero, a larger one is shortened by it; for the
// layer a top contact supports or a floor rests on, the tolerance is the one of the bound snapping), and never within its range.
// The roles the configuration produces must hold slabs (the floors only if scenario.floors), the stacked base slabs must not
// leave a hole in the branches, and without interface layers the top contacts must have been rebuilt from the branches.
// require_bands asks for slabs in both the thin band and above it; with a variable profile, a top contact or a floor must also
// have its gap window on the thin band. Slicing and allocation failures propagate to Catch.
void check_final_slab_clearance(TriangleMesh mesh, const char *style, int interfaces, const GapScenario &scenario, bool require_bands)
{
    DynamicPrintConfig config = SG::config(style, false, interfaces);
    config.set_deserialize_strict({{"support_material_synchronize_layers", "0"},
        {"support_material_contact_distance", scenario.above}, {"support_material_bottom_contact_distance", scenario.below}});
    Print print;
    Model model;
    if (scenario.variable_layers)
        SG::init_print_with_profile(print, model, std::move(mesh),
            {0., 0.2, 1.7, 0.2, 1.71, 0.05, 2.3, 0.05, 2.31, 0.3, 9., 0.3}, config);
    else
        Test::init_print({mesh}, print, model, config);
    REQUIRE(print.validate().first == PrintBase::PrintValidationError::pveNone);
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    const Common::SupportParams params = SG::params(print, object);
    const coord_t gap_above = scale_to_layer_coord(params.gap_above), gap_below = scale_to_layer_coord(params.gap_below);
    const coord_t eps = scale_to_layer_coord(EPSILON);
    // The bound snapping of align_slab_bounds() can move a contact bound by up to this much from its object layer.
    const coord_t snap_tolerance = 4 * SCALED_EPSILON;
    const coord_t thin_start = scale_to_layer_coord(thin_band_start_mm), thin_end = scale_to_layer_coord(thin_band_end_mm);
    Routed routed(print);
    TF::generate_tree_slabs(nullptr, SG::print_view(print), SG::object_view(object), routed.params, routed.settings, routed.trees, std::string(style) == "organic", routed.support, routed.resting);
    const std::vector<std::pair<const char *, const Common::GeneratorLayers *>> roles{
        {"base", &routed.support.base}, {"interfaces", &routed.support.interfaces}, {"top", &routed.support.top}, {"bottom", &routed.support.bottom}};
    constexpr double overlap_tolerance_mm2 = 1e-6;
    size_t checked = 0, thin_band = 0, thick_band = 0, thin_windows = 0, violations = 0;
    std::string non_empty_counts;
    for (const std::pair<const char *, const Common::GeneratorLayers *> &role : roles) {
        const size_t non_empty = size_t(std::count_if(role.second->begin(), role.second->end(),
            [](const Common::GeneratorLayer *slab) { return !slab->polygons.empty(); }));
        non_empty_counts += std::string(role.first) + "=" + std::to_string(non_empty) + " ";
        // Every role that the configuration produces must be present, or the check below would pass on nothing.
        const std::string name = role.first;
        if ((name != "interfaces" || interfaces > 0) && (name != "bottom" || scenario.floors)) {
            INFO("role " << role.first);
            CHECK(non_empty > 0);
        }
    }
    INFO("non-empty slabs per role: " << non_empty_counts);
    if (interfaces == 0)
        for (const Common::GeneratorLayer *contact : routed.support.top)
            CHECK(contact->type != Common::LayerType::TopContact);
    CHECK(branch_gaps(base_slabs(routed.support)) == 0);
    const coord_t near_window = std::max(gap_above, gap_below) + scale_to_layer_coord(0.4);
    double minimum_clearance_mm = std::numeric_limits<double>::infinity();
    for (const std::pair<const char *, const Common::GeneratorLayers *> &role : roles) {
        const std::string name = role.first;
        for (size_t index = 0; index < role.second->size(); ++index) {
            const Common::GeneratorLayer &slab = *(*role.second)[index];
            ++checked;
            if (slab.polygons.empty())
                continue;
            thin_band += slab.bottom_z < thin_end && slab.print_z > thin_start;
            thick_band += slab.print_z > thin_end;
            // The Z window kept free by the gap: over a top contact, under a floor.
            if (name == "top")
                thin_windows += slab.print_z < thin_end && slab.print_z + gap_above > thin_start;
            else if (name == "bottom")
                thin_windows += slab.bottom_z - gap_below < thin_end && slab.bottom_z > thin_start;
            for (size_t j = 0; j < object.layer_count(); ++j) {
                const Layer &layer = object.layer(j);
                const bool above = layer.scaled_bottom_z() >= slab.print_z;
                const bool below = layer.scaled_print_z() <= slab.bottom_z;
                const coord_t clearance = above ? layer.scaled_bottom_z() - slab.print_z : below ? slab.bottom_z - layer.scaled_print_z() :
                    std::max(slab.bottom_z, layer.scaled_bottom_z()) - std::min(slab.print_z, layer.scaled_print_z());
                // Intersecting polygons is the costly part: the layers far beyond the gaps cannot violate them and are skipped.
                if (clearance > near_window)
                    continue;
                const double overlap_mm2 = unscaled(unscaled(area(intersection_ex(SG::native(slab.polygons), layer.lslices()))));
                if (overlap_mm2 <= overlap_tolerance_mm2)
                    continue;
                minimum_clearance_mm = std::min(minimum_clearance_mm, unscaled(clearance));
                // The layer a top contact supports or a floor rests on has its clearance set by the contact placement and the bound snap.
                const bool supported = (name == "top" && j == slab.object_above) || (name == "bottom" && j == slab.object_below);
                const coord_t tolerance = supported ? snap_tolerance : eps;
                // Above and below are directional; layers intersecting the slab's own Z are not excluded, since trim_slab must clear them too.
                const bool forbidden = (!above && !below) || (above && clearance < gap_above - tolerance) || (below && clearance < gap_below - tolerance);
                if (forbidden) {
                    ++violations;
                    std::printf("CLEARANCE FAILURE style=%s interfaces=%d gaps=%s/%s role=%s slab=%zu z=[%.9f,%.9f] object_layer=%zu z=[%.9f,%.9f] overlap_mm2=%.12g clearance_mm=%.9f\n",
                        style, interfaces, scenario.above, scenario.below, role.first, index, unscaled(slab.bottom_z), unscaled(slab.print_z), j,
                        layer.unscaled_print_z() - layer.unscaled_height(), layer.unscaled_print_z(), overlap_mm2, unscaled(clearance));
                }
            }
        }
    }
    INFO("slabs=" << checked << " thin_band=" << thin_band << " thick_band=" << thick_band << " thin_windows=" << thin_windows
        << " minimum_clearance_mm=" << minimum_clearance_mm << " violations=" << violations);
    REQUIRE(checked > 0);
    if (require_bands) {
        REQUIRE(thin_band > 0);
        REQUIRE(thick_band > 0);
    }
    if (scenario.variable_layers)
        REQUIRE(thin_windows > 0);
    REQUIRE(std::isfinite(minimum_clearance_mm));
    REQUIRE(violations == 0);
}

} // namespace

// Runs owned tree shelf geometry for all four gap scenarios; slab assertions and generation failures reach Catch.
TEST_CASE("Final tree slabs respect small large asymmetric and variable physical Z gaps", "[plugins][support-tree-family][support-tree-slab-clearance]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const GapScenario scenario = GENERATE(GapScenario{ "0.08", "0.08", false, false }, GapScenario{ "0.5", "0.5", false, true },
        GapScenario{ "0.5", "0.15", false, true }, GapScenario{ "0.15", "0.15", true, true });
    check_shelf_clearance("tree", scenario);
}

// Checks owned organic shelf slabs with symmetric 0.08 mm gaps; helper assertions and generation failures reach Catch.
TEST_CASE("Final organic slabs respect physical Z gaps (small)", "[plugins][support-tree-family][support-tree-slab-clearance]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    check_shelf_clearance("organic", GapScenario{ "0.08", "0.08", false, false });
}

// Checks owned organic shelf slabs with symmetric 0.5 mm gaps and floors; helper failures reach Catch.
TEST_CASE("Final organic slabs respect physical Z gaps (large)", "[plugins][support-tree-family][support-tree-slab-clearance]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    check_shelf_clearance("organic", GapScenario{ "0.5", "0.5", false, true });
}

// Checks owned organic shelf slabs with 0.5/0.15 mm gaps and floors; helper failures reach Catch.
TEST_CASE("Final organic slabs respect physical Z gaps (asymmetric)", "[plugins][support-tree-family][support-tree-slab-clearance]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    check_shelf_clearance("organic", GapScenario{ "0.5", "0.15", false, true });
}

// Checks owned organic shelf slabs with variable heights and symmetric 0.15 mm gaps; helper failures reach Catch.
TEST_CASE("Final organic slabs respect physical Z gaps (variable layers)", "[plugins][support-tree-family][support-tree-slab-clearance]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    check_shelf_clearance("organic", GapScenario{ "0.15", "0.15", true, true });
}

namespace {
// Builds owned shelf/low-roof/lobe geometry for the borrowed style and gap scenario, then checks every final slab with three
// interfaces. The 8 mm roof and 2 mm roof exercise thick/thin profile bands; generation failures propagate to Catch.
void check_shelf_clearance(const char *style, const GapScenario &scenario)
{
    CAPTURE(style, scenario.above, scenario.below, scenario.variable_layers);
    TriangleMesh mesh = Slic3r::Test::SupportTree::comparison_mesh(2);
    TriangleMesh low_roof = make_cube(12., 8., 0.4);
    low_roof.translate(0., 0., 2.);
    mesh.merge(low_roof);
    check_final_slab_clearance(std::move(mesh), style, 3, scenario, true);
}
} // namespace

// Without interface layers the top contacts are rebuilt from the branches drawn at their middle and the tip stretches that cross
// them. Under a sloped overhang (a staircase whose roofs fall on consecutive layers) the tip stretch of a neighbouring roof then
// reaches the next contact, which must stay a full gap away from it.
TEST_CASE("Rebuilt tip contacts of a sloped overhang respect the physical Z gaps", "[plugins][support-tree-family][support-tree-slab-clearance]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *style = GENERATE("tree", "organic");
    const GapScenario scenario = GENERATE(GapScenario{ "0.5", "0.5", false, false }, GapScenario{ "0.5", "0.15", false, false });
    CAPTURE(style, scenario.above, scenario.below);
    TriangleMesh mesh = make_cube(3., 4., 0.4);
    mesh.translate(0., 0., 1.);
    for (int step = 1; step < 6; ++step) {
        TriangleMesh block = make_cube(3. + 0.5 * step, 4., 0.4);
        block.translate(0., 0., 1. + 0.2 * step);
        mesh.merge(block);
    }
    check_final_slab_clearance(std::move(mesh), style, 0, scenario, false);
}

// The routing and the final trim both use the XY distance of the user (support_material_xy_spacing). With a spacing of 1 mm no
// final slab may come closer than that to the layers of its Z span or of its gap windows, except to the layer a top contact supports
// or a floor rests on, whose clearance is set by the contact placement.
TEST_CASE("Final tree slabs keep the normal XY distance from the object (3 interfaces)", "[plugins][support-tree-family][support-tree-xy-clearance]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    check_normal_xy_clearance("tree", 3);
}

// Checks owned tree slabs without interfaces against normal XY clearance; helper failures reach Catch.
TEST_CASE("Final tree slabs keep the normal XY distance from the object (0 interfaces)", "[plugins][support-tree-family][support-tree-xy-clearance]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    check_normal_xy_clearance("tree", 0);
}

// Checks owned organic slabs with three interfaces against normal XY clearance; helper failures reach Catch.
TEST_CASE("Final organic slabs keep the normal XY distance from the object (3 interfaces)", "[plugins][support-tree-family][support-tree-xy-clearance]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    check_normal_xy_clearance("organic", 3);
}

// Checks owned organic slabs without interfaces against normal XY clearance; helper failures reach Catch.
TEST_CASE("Final organic slabs keep the normal XY distance from the object (0 interfaces)", "[plugins][support-tree-family][support-tree-xy-clearance]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    check_normal_xy_clearance("organic", 0);
}

namespace {
// Routes owned shelf/low-roof/lobe geometry for the borrowed style and interface count, then checks final slabs against
// 1 mm expanded object outlines in their Z/gap windows. Print/model own geometry; generation failures and checks reach Catch.
void check_normal_xy_clearance(const char *style, int interfaces)
{
    CAPTURE(style, interfaces);
    DynamicPrintConfig config = SG::config(style, false, interfaces);
    config.set_deserialize_strict({{"support_material_synchronize_layers", "0"}, {"support_material_xy_spacing", "1"}});
    TriangleMesh mesh = Slic3r::Test::SupportTree::comparison_mesh(2);
    TriangleMesh low_roof = make_cube(12., 8., 0.4);
    low_roof.translate(0., 0., 2.);
    mesh.merge(low_roof);
    Print print;
    Model model;
    Test::init_print({mesh}, print, model, config);
    REQUIRE(print.validate().first == PrintBase::PrintValidationError::pveNone);
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    const Common::SupportParams params = SG::params(print, object);
    const TF::TreeSettings settings = Slic3r::Test::SupportTree::tree_settings(print, object, params);
    const coord_t gap_above = scale_to_layer_coord(params.gap_above), gap_below = scale_to_layer_coord(params.gap_below);
    // The Z gaps count as in trim_slab: not above the tolerance they are zero, above it they are shortened by it.
    const coord_t eps = scale_to_layer_coord(EPSILON);
    const coord_t above = gap_above > eps ? gap_above - eps : 0, below = gap_below > eps ? gap_below - eps : 0;
    Routed routed(print);
    TF::generate_tree_slabs(nullptr, SG::print_view(print), SG::object_view(object), routed.params, routed.settings, routed.trees, std::string(style) == "organic", routed.support, routed.resting);
    // The outline of every layer grown by the normal distance, less the tolerance of the drawn checks.
    const std::vector<Polygons> keep_out = keep_out_outlines(object, settings.xy_distance - coord_t(scale_d(tolerance_mm)));
    const std::vector<std::pair<const char *, const Common::GeneratorLayers *>> roles{
        {"base", &routed.support.base}, {"interfaces", &routed.support.interfaces}, {"top", &routed.support.top}, {"bottom", &routed.support.bottom}};
    size_t non_empty = 0, checked_pairs = 0;
    for (const std::pair<const char *, const Common::GeneratorLayers *> &role : roles)
        for (const Common::GeneratorLayer *slab : *role.second) {
            const Polygons area = to_native(slab->polygons);
            if (area.empty())
                continue;
            ++non_empty;
            for (size_t id = 0; id < object.layer_count(); ++id) {
                const Layer &layer = object.layer(id);
                const bool supported = (std::string(role.first) == "top" && id == slab->object_above) ||
                                       (std::string(role.first) == "bottom" && id == slab->object_below);
                if (supported || layer.scaled_bottom_z() >= slab->print_z + above || layer.scaled_print_z() <= slab->bottom_z - below)
                    continue;
                ++checked_pairs;
                INFO("role " << role.first << " layer " << id);
                CHECK(intersection(area, keep_out[id]).empty());
            }
        }
    // The check must not pass on nothing.
    REQUIRE(non_empty > 0);
    REQUIRE(checked_pairs > 0);
}
} // namespace

// Generates and publishes soluble supports below a roof beside its post; final trimmed/smoothed surfaces must occupy positive
// area in the outer 0.08 mm band of a vertically overlapping object layer. Print/model own inputs and published output;
// generation/allocation failures reach Catch, and the band check excludes contacts against an object layer only above/below it.
TEST_CASE("Final soluble tree and organic supports approach the object at zero XY clearance", "[plugins][support-tree-family][support-tree-xy-clearance]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *style = GENERATE("tree", "organic");
    CAPTURE(style);
    TriangleMesh mesh = make_cube(4., 8., 6.);
    TriangleMesh roof = make_cube(12., 8., 1.);
    roof.translate(-4., 0., 6.);
    mesh.merge(roof);
    DynamicPrintConfig config = SG::config(style, true, 3);
    config.set_deserialize_strict({{"support_material_xy_spacing", "0"}, {"support_material_synchronize_layers", "0"}});
    Print print;
    Model model;
    Test::init_print({mesh}, print, model, config);
    SG::prefix(print);
    SG::family(print);
    const PrintObject &object = print.object(0);
    REQUIRE_FALSE(object.auxiliary_layers().empty());
    double close_area = 0.;
    size_t overlapping_pairs = 0;
    for (const Layer &support : object.auxiliary_layers()) {
        ExPolygons printed;
        for (const LayerSliceIsland &island : support.islands())
            for (const LayerRegionIsland &region : island.regions_islands())
                for (const Surface &surface : region.fill_surfaces())
                    printed.push_back(surface.expolygon);
        if (printed.empty()) continue;
        for (size_t id = 0; id < object.layer_count(); ++id) {
            const Layer &layer = object.layer(id);
            if (layer.scaled_bottom_z() >= support.scaled_print_z() || layer.scaled_print_z() <= support.scaled_bottom_z())
                continue;
            ++overlapping_pairs;
            const ExPolygons band = diff_ex(offset_ex(layer.lslices(), scale_i(0.08)), layer.lslices());
            close_area += area(intersection_ex(printed, band));
        }
    }
    REQUIRE(overlapping_pairs > 0);
    CHECK(close_area > scale_d(0.001) * scale_d(0.001));
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
        {"gcode_flavor", "marlin2"},
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
        Orchestrator::instance().export_gcode(print, path.string(), nullptr);
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

// A print of a mesh sliced with the tree slabs generated, with the given brim widths and raft_first_layer_expansion.
struct BrimRun {
    Print print;
    Model model;
    std::unique_ptr<Routed> routed;
    BrimRun(TriangleMesh mesh, bool organic, int interfaces, const char *brim_width, const char *interior, const char *expansion = "0", const char *skirts = "0")
    {
        DynamicPrintConfig config = SG::config(organic ? "organic" : "tree", false, interfaces);
        config.set_deserialize_strict({{"brim_width", brim_width}, {"brim_width_interior", interior}, {"raft_first_layer_expansion", expansion},
                                       {"skirts", skirts}, {"skirt_distance", "1"}});
        Test::init_print({std::move(mesh)}, print, model, config);
        SG::prefix(print);
        routed = std::make_unique<Routed>(print);
        TF::generate_tree_slabs(nullptr, SG::print_view(print), SG::object_view(print.object(0)), routed->params, routed->settings, routed->trees, organic, routed->support, routed->resting);
    }
};

// The area in mm2 of the symmetric difference of two sets of polygons, without any filter of small areas.
double xor_mm2(const Polygons &a, const Polygons &b)
{
    return Slic3r::Test::PolygonMetrics::sym_diff_mm2(a, b);
}
}

// The support ignores the brim and the skirt: with a brim_width of 2, a brim_width_interior of 1 and a skirt, every slab of every list, floors included, equals the slab of the same print without a brim.
TEST_CASE("Tree first layer slabs ignore the brim", "[plugins][support-tree-family][brim]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool organic = GENERATE(false, true);
    const int interfaces = GENERATE(0, 3);
    CAPTURE(organic, interfaces);
    const BrimRun plain(brim_hollow_mesh(), organic, interfaces, "0", "0", "3"), brimmed(brim_hollow_mesh(), organic, interfaces, "2", "1", "3", "1");
    // Only the second print has a brim and a skirt.
    REQUIRE_FALSE(plain.print.object(0).has_brim());
    REQUIRE(brimmed.print.object(0).has_brim());
    const Common::GeneratedSupport &before = plain.routed->support, &after = brimmed.routed->support;
    const std::vector<std::pair<const Common::GeneratorLayers *, const Common::GeneratorLayers *>> lists = {
        {&before.top, &after.top}, {&before.base, &after.base}, {&before.interfaces, &after.interfaces}, {&before.bottom, &after.bottom}};
    // The plain print has the slabs that the brim used to cut.
    REQUIRE_FALSE(before.top.empty());
    REQUIRE_FALSE(before.base.empty());
    if (interfaces > 0)
        REQUIRE_FALSE(before.interfaces.empty());
    const coord_t first = scale_to_layer_coord(plain.routed->params.first_layer_height);
    double first_layer_mm2 = 0.;
    for (const std::pair<const Common::GeneratorLayers *, const Common::GeneratorLayers *> &list : lists) {
        REQUIRE(list.first->size() == list.second->size());
        for (size_t i = 0; i < list.first->size(); ++i) {
            const Common::GeneratorLayer &a = *(*list.first)[i], &b = *(*list.second)[i];
            const Polygons polygons_a = to_native(a.polygons), polygons_b = to_native(b.polygons);
            CHECK(a.print_z == b.print_z);
            CHECK(a.bottom_z == b.bottom_z);
            CHECK(a.height == b.height);
            CHECK(a.bridging == b.bridging);
            CHECK(a.type == b.type);
            CHECK(xor_mm2(polygons_a, polygons_b) < 1e-4);
            if (a.print_z <= first)
                first_layer_mm2 += unscaled(unscaled(area(union_ex(polygons_b))));
        }
    }
    // The case is not vacuous: the first layer holds support.
    CHECK(first_layer_mm2 > 0.);
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
                append(grown, offset(to_native(slab->polygons), float(scale_d(EPSILON)), ClipperLib::jtSquare, 0.));
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
    const BrimRun narrow(Test::SupportGrid::slab(), organic, interfaces, "0", "0", "0"), wide(Test::SupportGrid::slab(), organic, interfaces, "0", "0", "3");
    const Common::GeneratorLayer &foot0 = foot_of(narrow), &foot3 = foot_of(wide);
    CAPTURE(foot0.type, foot3.type);
    CHECK(foot3.type == (organic ? Common::LayerType::Branch : Common::LayerType::Base));
    const Common::SupportParams &params = wide.routed->params;
    // The joins of the production recipe: square for the object trimming and the dense footprints (CLIPPER_JOIN_SQUARE, miter limit 0), the
    // default miter join (limit 3) for the growth steps.
    const Polygons trimming = offset(to_polygons(wide.print.object(0).layer(0).lslices()), float(params.gap_xy), ClipperLib::jtSquare, 0.);
    const Polygons polygons0 = to_native(foot0.polygons), polygons3 = to_native(foot3.polygons);
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
                CHECK(xor_mm2(to_native(a.polygons), to_native(b.polygons)) < 1e-4);
        }
    }
}

namespace {
// The published surfaces and the support toolpaths of the bed layer of a print that went through publication and toolpaths.
struct BedLayerPrint {
    Print print;
    Model model;
    ExPolygons surfaces, sparse_surfaces;
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
        const SurfaceType sparse_type = stModSupport | stPosInternal | stDensSparse;
        for (const Layer &layer : object.auxiliary_layers()) {
            if (layer.scaled_print_z() > object.layer(0).scaled_print_z())
                continue;
            for (const LayerSliceIsland &island : layer.islands())
                for (const LayerRegionIsland &region : island.regions_islands()) {
                    for (const Surface &surface : region.fill_surfaces()) {
                        surfaces.push_back(surface.expolygon);
                        if (surface.surface_type == sparse_type)
                            sparse_surfaces.push_back(surface.expolygon);
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
        sparse_surfaces = union_ex(sparse_surfaces);
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
// than with none, the support toolpaths run inside the gained area and stay out of the first object layer.
// Both styles' sparse support surfaces carry the flange paths, in the gained area too.
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
    // The flange follows sparse support on the bed in either style.
    const double sparse_mm2 = unscaled(unscaled(area(wide.sparse_surfaces)));
    const double flange_mm = length_inside_mm(wide.support_paths, offset_ex(wide.sparse_surfaces, float(scale_d(0.05))));
    const double gained_sparse_mm = length_inside_mm(wide.support_paths, intersection_ex(gained, wide.sparse_surfaces));
    CAPTURE(sparse_mm2, flange_mm, gained_sparse_mm);
    CHECK(sparse_mm2 > 1.);
    CHECK(flange_mm > 1.);
    CHECK(gained_sparse_mm > 1.);
}

// Hidden benchmark of smooth_organic_branches() on 5000 spheres (1000 branches of 5 nodes), whole call timed. SLIC3R_SMOOTH_THREADS
// sets the thread count (tbb::global_control, default 16), SLIC3R_SMOOTH_RUNS the number of runs (default 5). One line per run, then
// the min and the median. Run it with "[smoothing-benchmark]".
TEST_CASE("Organic smoothing relaxation benchmark", "[.][smoothing-benchmark]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *threads_env = std::getenv("SLIC3R_SMOOTH_THREADS");
    const char *runs_env = std::getenv("SLIC3R_SMOOTH_RUNS");
    const size_t threads = threads_env ? size_t(std::atoi(threads_env)) : size_t(16);
    const int runs = runs_env ? std::atoi(runs_env) : 5;
    tbb::global_control thread_limit(tbb::global_control::max_allowed_parallelism, threads);
    Print print;
    Model model;
    Test::init_print({SG::slab()}, print, model, SG::config("organic", false, 0));
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    // 40 small squares along X so that a share of the spheres overlaps an outline.
    Polygons squares;
    for (int k = 0; k < 40; ++k) {
        const Polygons square = square_mm(double(k) * 25. + 6., -2., 4.);
        squares.insert(squares.end(), square.begin(), square.end());
    }
    const std::vector<Polygons> keep_out(object.layer_count(), squares);
    PluginStorage storage;
    const std::vector<slic3r_api::StoredPolygonCollection> stored_keep_out = stored_outlines(storage, keep_out);
    std::vector<double> times;
    for (int run = 0; run < runs; ++run) {
        std::vector<TF::RoutedTree> trees;
        for (int t = 0; t < 1000; ++t) {
            const double x = double(t) * 1.0;
            trees.push_back(chain_tree(object, { { x, 0., 20 }, { x, 0., 16 }, { x, 0., 12 }, { x + 4., 0., 8 }, { x + 8., 0., 4 } }));
        }
        const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
        const size_t iterations = TF::smooth_organic_branches(layer_print_z_mm(object), stored_keep_out, trees, nullptr);
        times.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count());
        std::printf("SMOOTH-BENCH threads=%zu run=%d spheres=5000 iterations=%zu total_ms=%.1f\n", threads, run, iterations, times.back());
        CHECK(iterations > 0);
    }
    std::sort(times.begin(), times.end());
    std::printf("SMOOTH-BENCH-SUMMARY threads=%zu runs=%d min_ms=%.1f median_ms=%.1f\n", threads, runs, times.front(), times[times.size() / 2]);
}

// Hidden benchmark of the tree routing, the tool to measure a change of the merge cadence of create_layer_pathing (the merges are on
// every layer). It times, separately (ms, 5 runs each), the construction of Routed (parameters, settings, pattern context and
// route_trees()) and generate_tree_slabs(), on the three comparison objects (see comparison_mesh), for the tree and organic styles
// with three interface layers, with every available thread. It prints one line per fixture and style: the node count, then the min
// and the median of each step. No timing is asserted. The merge order depends on the TBB thread count, so compare two builds on the
// same machine only. Build Release and run it from build/tests/plugins with: .\Release\plugins_tests.exe "[tree-routing-benchmark]"
TEST_CASE("Tree routing and slab generation benchmark", "[.][tree-routing-benchmark]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    constexpr int runs = 5;
    const auto median = [](std::vector<double> &times) {
        std::sort(times.begin(), times.end());
        return times[times.size() / 2];
    };
    for (const int model_id : {0, 1, 2})
        for (const char *style : {"tree", "organic"}) {
            Print print;
            Model model;
            Test::init_print({Slic3r::Test::SupportTree::comparison_mesh(model_id)}, print, model, SG::config(style, false, 3, false));
            SG::prefix(print);
            const bool organic = std::string(style) == "organic";
            std::vector<double> route_times, slab_times;
            size_t nodes = 0;
            for (int run = 0; run < runs; ++run) {
                const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
                Routed routed(print);
                const std::chrono::steady_clock::time_point routed_at = std::chrono::steady_clock::now();
                TF::generate_tree_slabs(nullptr, SG::print_view(print), SG::object_view(print.object(0)), routed.params, routed.settings, routed.trees, organic, routed.support, routed.resting);
                const std::chrono::steady_clock::time_point done = std::chrono::steady_clock::now();
                route_times.push_back(std::chrono::duration<double, std::milli>(routed_at - start).count());
                slab_times.push_back(std::chrono::duration<double, std::milli>(done - routed_at).count());
                nodes = 0;
                for (const TF::RoutedTree &tree : routed.trees)
                    nodes += tree.nodes.size();
            }
            const double route_min = *std::min_element(route_times.begin(), route_times.end());
            const double slab_min = *std::min_element(slab_times.begin(), slab_times.end());
            std::printf("ROUTING-BENCH model=%d style=%s nodes=%zu route_min_ms=%.1f route_median_ms=%.1f slabs_min_ms=%.1f slabs_median_ms=%.1f\n",
                        model_id, style, nodes, route_min, median(route_times), slab_min, median(slab_times));
            CHECK(nodes > 0);
        }
}

namespace {
struct PerfModel {
    std::string  name;
    TriangleMesh mesh;
};

// The comma list of an environment variable, empty when it is not set.
std::vector<std::string> perf_list(const char *variable)
{
    std::vector<std::string> result;
    if (const char *value = std::getenv(variable)) {
        std::istringstream in(value);
        for (std::string item; std::getline(in, item, ',');)
            if (!item.empty())
                result.push_back(item);
    }
    return result;
}

bool perf_selected(const std::vector<std::string> &list, const std::string &name)
{
    return list.empty() || std::find(list.begin(), list.end(), name) != list.end();
}

// An obj of tests/data, scaled so that its longest side is 120 mm when it is smaller than 100 mm.
TriangleMesh perf_obj(const char *file)
{
    TriangleMesh mesh;
    REQUIRE(load_obj((std::string(TEST_DATA_DIR) + "/" + file).c_str(), &mesh));
    const Vec3d size = mesh.bounding_box().size();
    const double longest = std::max({ size.x(), size.y(), size.z() });
    if (longest > 0. && longest < 100.)
        mesh.scale(float(120. / longest));
    return mesh;
}

struct PerfStat {
    std::vector<double> values;
    void add(double value) { values.push_back(value); }
    double min() const { return values.empty() ? 0. : *std::min_element(values.begin(), values.end()); }
    double median() const
    {
        std::vector<double> sorted = values;
        std::sort(sorted.begin(), sorted.end());
        return sorted.empty() ? 0. : sorted[sorted.size() / 2];
    }
};

double perf_ms(const std::chrono::steady_clock::time_point &from, const std::chrono::steady_clock::time_point &to)
{
    return std::chrono::duration<double, std::milli>(to - from).count();
}
} // namespace

// Runs only the plugin support and toolpath steps after the slicing prefix, using the benchmark's model loading and optional
// SLIC3R_PERF_MODELS / SLIC3R_PERF_STYLES filters. Each joined run resets process counters and prints every named total;
// the print owns its support and temporaries, loading/generation failures propagate, and only support existence is asserted.
TEST_CASE("Support work counters on comparison models", "[.][support-counters]")
{
    if (!Common::Counters::enabled) {
        WARN("Support counters are disabled: SLIC3R_SUPPORT_COUNTERS=1 must be set before the process starts");
        return;
    }
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const std::vector<std::string> model_filter = perf_list("SLIC3R_PERF_MODELS"), style_filter = perf_list("SLIC3R_PERF_STYLES");
    std::vector<PerfModel> models;
    if (perf_selected(model_filter, "frog_legs"))
        models.push_back({"frog_legs", perf_obj("frog_legs.obj")});
    if (perf_selected(model_filter, "comparison2"))
        models.push_back({"comparison2", Slic3r::Test::SupportTree::comparison_mesh(2)});
    if (perf_selected(model_filter, "A_upsidedown"))
        models.push_back({"A_upsidedown", perf_obj("A_upsidedown.obj")});
    for (const PerfModel &entry : models) {
        for (const char *style : {"grid", "snug", "tree", "organic"}) {
            if (!perf_selected(style_filter, style)) continue;
            Print print;
            Model model;
            Test::init_print({entry.mesh}, print, model, SG::config(style, false, 3, false));
            SG::prefix(print);
            Common::Counters::reset();
            SG::family(print);
            SG::toolpaths(print);
            const std::vector<std::pair<const char *, uint64_t>> counts = Common::Counters::snapshot();
            std::printf("SUPPORT-COUNTERS model=%s style=%s", entry.name.c_str(), style);
            for (const std::pair<const char *, uint64_t> &count : counts)
                std::printf(" %s=%llu", count.first, static_cast<unsigned long long>(count.second));
            std::printf("\n");
            std::fflush(stdout);
            CHECK_FALSE(print.object(0).auxiliary_layers().empty());
        }
    }
}

// Hidden benchmark of the support generation on large objects, the plugin (the STEP_SUPPORT family run and its print Z host pass, then
// the STEP_SUPPORT_TOOLPATH run) against the native legacy generator, in the same binary. Models: frog_legs.obj and A_upsidedown.obj of
// tests/data (scaled to a longest side of 120 mm when smaller than 100 mm) and comparison_mesh(2); styles grid, snug, tree and organic,
// with three interface layers and a layer height of 0.2. Every run builds its print and runs the slicing prefix (SG::prefix) untimed.
// Plugin side: support_ms is SG::family() (the family run and the host pass), toolpath_ms SG::toolpaths(). Legacy side: the whole native
// generation, which includes its toolpaths (PrintObjectSupportMaterial::generate() for grid and snug, fff_tree_support_generate() for
// tree and organic, both end with generate_support_toolpaths()); the untimed prepare_legacy_inputs() precedes it. The plugin total
// (support + toolpath) is therefore the one to compare with the legacy total. For tree and organic a third print is timed apart, split as
// Routed construction (volumes precalculation and routing), generate_tree_slabs() and Common::publish().
// Environment: SLIC3R_PERF_THREADS (tbb::global_control, default every thread), SLIC3R_PERF_RUNS (default 3), SLIC3R_PERF_MODELS
// (frog_legs, A_upsidedown, comparison2) and SLIC3R_PERF_STYLES (grid, snug, tree, organic) as comma lists (default all), and
// SLIC3R_PERF_LEGACY=0 to skip the legacy side. No timing is asserted; the support must exist. Build Release and run it from
// build/tests/plugins with: .\Release\plugins_tests.exe "[support-perf-benchmark]"
TEST_CASE("Support generation performance benchmark, plugin against legacy", "[.][support-perf-benchmark]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *threads_env = std::getenv("SLIC3R_PERF_THREADS");
    const char *runs_env = std::getenv("SLIC3R_PERF_RUNS");
    const char *legacy_env = std::getenv("SLIC3R_PERF_LEGACY");
    const int runs = std::max(1, runs_env ? std::atoi(runs_env) : 3);
    const bool with_legacy = legacy_env == nullptr || std::atoi(legacy_env) != 0;
    std::unique_ptr<tbb::global_control> thread_limit;
    if (threads_env && std::atoi(threads_env) > 0)
        thread_limit = std::make_unique<tbb::global_control>(tbb::global_control::max_allowed_parallelism, size_t(std::atoi(threads_env)));
    const size_t threads = tbb::global_control::active_value(tbb::global_control::max_allowed_parallelism);
    const std::vector<std::string> model_filter = perf_list("SLIC3R_PERF_MODELS"), style_filter = perf_list("SLIC3R_PERF_STYLES");

    std::vector<PerfModel> models;
    if (perf_selected(model_filter, "frog_legs"))
        models.push_back({ "frog_legs", perf_obj("frog_legs.obj") });
    if (perf_selected(model_filter, "A_upsidedown"))
        models.push_back({ "A_upsidedown", perf_obj("A_upsidedown.obj") });
    if (perf_selected(model_filter, "comparison2"))
        models.push_back({ "comparison2", Slic3r::Test::SupportTree::comparison_mesh(2) });
    REQUIRE_FALSE(models.empty());

    for (const PerfModel &model_entry : models) {
        const Vec3d size = model_entry.mesh.bounding_box().size();
        std::printf("PERF-MODEL model=%s triangles=%zu size_mm=%.2fx%.2fx%.2f threads=%zu runs=%d\n", model_entry.name.c_str(),
                    model_entry.mesh.facets_count(), size.x(), size.y(), size.z(), threads, runs);
        for (const char *style : { "grid", "snug", "tree", "organic" }) {
            if (!perf_selected(style_filter, style))
                continue;
            const bool tree_family = std::strcmp(style, "tree") == 0 || std::strcmp(style, "organic") == 0;
            const bool organic = std::strcmp(style, "organic") == 0;
            const DynamicPrintConfig config = SG::config(style, false, 3, false);
            PerfStat plugin_support, plugin_toolpath, plugin_total, legacy_total, routed_stat, slabs_stat, publish_stat, split_total;
            for (int run = 0; run < runs; ++run) {
                {
                    Print print;
                    Model model;
                    Test::init_print({ model_entry.mesh }, print, model, config);
                    SG::prefix(print);
                    const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
                    SG::family(print);
                    const std::chrono::steady_clock::time_point supported = std::chrono::steady_clock::now();
                    SG::toolpaths(print);
                    const std::chrono::steady_clock::time_point done = std::chrono::steady_clock::now();
                    plugin_support.add(perf_ms(start, supported));
                    plugin_toolpath.add(perf_ms(supported, done));
                    plugin_total.add(perf_ms(start, done));
                    const size_t layers = print.object(0).auxiliary_layers().size();
                    std::printf("PERF-RUN model=%s style=%s run=%d side=plugin layers=%zu support_ms=%.1f toolpath_ms=%.1f total_ms=%.1f\n",
                                model_entry.name.c_str(), style, run, layers, plugin_support.values.back(), plugin_toolpath.values.back(), plugin_total.values.back());
                    CHECK(layers > 0);
                }
                if (with_legacy) {
                    Print print;
                    Model model;
                    Test::init_print({ model_entry.mesh }, print, model, config);
                    SG::prefix(print);
                    PrintObject &object = print.object(0);
                    REQUIRE(object.auxiliary_layers().empty());
                    SG::prepare_legacy_inputs(object);
                    const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
                    if (tree_family)
                        fff_tree_support_generate(object);
                    else {
                        PrintObjectSupportMaterial generator(&object, std::make_shared<SlicingParameters>(object.slicing_parameters()));
                        generator.generate(object);
                    }
                    legacy_total.add(perf_ms(start, std::chrono::steady_clock::now()));
                    const size_t layers = object.auxiliary_layers().size();
                    std::printf("PERF-RUN model=%s style=%s run=%d side=legacy layers=%zu total_ms=%.1f\n", model_entry.name.c_str(), style, run,
                                layers, legacy_total.values.back());
                    CHECK(layers > 0);
                }
                if (tree_family) {
                    Print print;
                    Model model;
                    Test::init_print({ model_entry.mesh }, print, model, config);
                    SG::prefix(print);
                    PrintObject &object = print.object(0);
                    const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
                    Routed routed(print);
                    const std::chrono::steady_clock::time_point routed_at = std::chrono::steady_clock::now();
                    TF::generate_tree_slabs(nullptr, SG::print_view(print), SG::object_view(object), routed.params, routed.settings, routed.trees, organic, routed.support, routed.resting);
                    const std::chrono::steady_clock::time_point slabs_at = std::chrono::steady_clock::now();
                    Common::publish(nullptr, SG::object_view(object), routed.params, routed.support, reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
                    const std::chrono::steady_clock::time_point done = std::chrono::steady_clock::now();
                    routed_stat.add(perf_ms(start, routed_at));
                    slabs_stat.add(perf_ms(routed_at, slabs_at));
                    publish_stat.add(perf_ms(slabs_at, done));
                    split_total.add(perf_ms(start, done));
                    size_t nodes = 0;
                    for (const TF::RoutedTree &tree : routed.trees)
                        nodes += tree.nodes.size();
                    std::printf("PERF-SPLIT model=%s style=%s run=%d nodes=%zu routed_ms=%.1f slabs_ms=%.1f publish_ms=%.1f total_ms=%.1f\n",
                                model_entry.name.c_str(), style, run, nodes, routed_stat.values.back(), slabs_stat.values.back(),
                                publish_stat.values.back(), split_total.values.back());
                    CHECK(nodes > 0);
                }
            }
            const double ratio_min = legacy_total.min() > 0. ? plugin_total.min() / legacy_total.min() : 0.;
            const double ratio_median = legacy_total.median() > 0. ? plugin_total.median() / legacy_total.median() : 0.;
            std::printf("PERF-SUMMARY model=%s style=%s threads=%zu runs=%d plugin_support_min_ms=%.1f plugin_support_median_ms=%.1f "
                        "plugin_toolpath_min_ms=%.1f plugin_toolpath_median_ms=%.1f plugin_total_min_ms=%.1f plugin_total_median_ms=%.1f "
                        "legacy_total_min_ms=%.1f legacy_total_median_ms=%.1f ratio_plugin_over_legacy_min=%.2f ratio_plugin_over_legacy_median=%.2f\n",
                        model_entry.name.c_str(), style, threads, runs, plugin_support.min(), plugin_support.median(), plugin_toolpath.min(),
                        plugin_toolpath.median(), plugin_total.min(), plugin_total.median(), legacy_total.min(), legacy_total.median(), ratio_min, ratio_median);
            if (tree_family)
                std::printf("PERF-SUMMARY-SPLIT model=%s style=%s routed_min_ms=%.1f routed_median_ms=%.1f slabs_min_ms=%.1f slabs_median_ms=%.1f "
                            "publish_min_ms=%.1f publish_median_ms=%.1f split_total_min_ms=%.1f split_total_median_ms=%.1f\n",
                            model_entry.name.c_str(), style, routed_stat.min(), routed_stat.median(), slabs_stat.min(), slabs_stat.median(),
                            publish_stat.min(), publish_stat.median(), split_total.min(), split_total.median());
        }
    }
}


namespace {
// The island-order condition that the Debug build asserts in Layer.cpp (recompute_slices_from_layer_regions). A violation is an island
// listed after an earlier island whose bounding box it contains and whose first point its contour contains, i.e. a container listed
// after an island lying in one of its holes; the correct order lists the container first. Returns true and fills earlier and later with
// the indexes of the pair on a violation; returns false and leaves them unchanged otherwise.
bool island_order_violation(const ExPolygons &islands, size_t &earlier, size_t &later)
{
    std::vector<BoundingBox> boxes;
    for (size_t check = 0; check < islands.size(); ++check) {
        boxes.emplace_back(islands[check].contour.points);
        for (size_t bigger = 0; bigger < check; ++bigger)
            if (boxes[check].contains(boxes[bigger]) && islands[check].contour.contains(islands[bigger].contour.first_point())) {
                earlier = bigger;
                later = check;
                return true;
            }
    }
    return false;
}
} // namespace

// The API union lists the islands from outside to inside, as the native union_ex does: an island nested in a hole comes after the
// island that holds it (ClipperShapes::to_expolygons walks the Clipper tree with traverse_pt, ClipperUtils.hpp). The API union of three
// nested squares lists the outer ring first.
TEST_CASE("The API union lists nested islands from outside to inside", "[plugins][support-island-order]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const auto square = [](double from, double to) {
        Polygon polygon;
        polygon.points = { Point::new_scale(from, from), Point::new_scale(to, from), Point::new_scale(to, to), Point::new_scale(from, to) };
        return polygon;
    };
    // An outer square, a hole in it and a square inside the hole.
    Polygons nested{ square(0., 10.), square(2., 8.), square(4., 6.) };
    nested[1].make_clockwise();
    const ExPolygons native_result = union_ex(nested);
    size_t earlier = 0, later = 0;
    REQUIRE(native_result.size() == 2);
    CHECK_FALSE(island_order_violation(native_result, earlier, later));
    PluginStorage storage;
    slic3r_api::StoredPolygonCollection collection(SG::handle(storage));
    assign_native(collection, nested);
    const slic3r_api::ClipperContext clipper(SG::handle(storage));
    const slic3r_api::StoredExPolygonCollection api = slic3r_api::clipper_union(clipper(collection)).to_expolygon_collection();
    const ExPolygons &api_result = *reinterpret_cast<const ExPolygons *>(api.handle());
    REQUIRE(api_result.size() == 2);
    CHECK_FALSE(island_order_violation(api_result, earlier, later));
}

// The islands of every object layer, and of every support layer
// that the publication builds (the union of the slabs of one print Z and thickness, as publish() does), of the downward point with
// 3 interface layers must be ordered outside to inside, which the Debug build asserts when a layer is published (Layer.cpp,
// recompute_slices_from_layer_regions). The support layers are rebuilt here as publish() builds them. The interface slabs of the point are rings around the cone section, with
// branch discs in their holes.
TEST_CASE("The islands of the layers of the downward point are ordered from outside to inside (thin)", "[plugins][support-island-order]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *layer_height = GENERATE("0.2", "0.3", "0.15");
    check_downward_point_island_order(true, layer_height);
}

// Checks owned control-point islands at all three heights through the ordering helper; failures reach Catch.
TEST_CASE("The islands of the layers of the downward point are ordered from outside to inside (control)", "[plugins][support-island-order]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *layer_height = GENERATE("0.2", "0.3", "0.15");
    check_downward_point_island_order(false, layer_height);
}

namespace {
// Slices owned downward-point geometry with the selected thickness and borrowed height string, then checks object islands
// and support islands unioned by publication Z/thickness. Geometry stays locally owned; generation failures reach Catch.
void check_downward_point_island_order(bool thin, const char *layer_height)
{
    CAPTURE(thin, layer_height);
    DynamicPrintConfig config = SG::config("organic", false, 3, true);
    config.set_deserialize_strict({{"layer_height", layer_height}, {"first_layer_height", layer_height}});
    if (thin)
        config.set_deserialize_strict({{"support_tree_tip_diameter", "0.42"}, {"support_tree_branch_diameter", "0.42"},
                                       {"support_material_extrusion_width", "0.42"}});
    Print print;
    Model model;
    Test::init_print({isolated_tips_mesh(2)}, print, model, config);
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    size_t earlier = 0, later = 0, object_violations = 0;
    for (size_t layer_idx = 0; layer_idx < object.layer_count(); ++layer_idx)
        object_violations += island_order_violation(object.layer(layer_idx).lslices(), earlier, later) ? 1 : 0;
    CHECK(object_violations == 0);
    Routed routed(print);
    TF::generate_tree_slabs(nullptr, SG::print_view(print), SG::object_view(object), routed.params, routed.settings, routed.trees, true, routed.support, routed.resting);
    Common::GeneratorLayers sorted = routed.support.top;
    for (const Common::GeneratorLayers *list : { &routed.support.bottom, &routed.support.base, &routed.support.interfaces })
        sorted.insert(sorted.end(), list->begin(), list->end());
    std::sort(sorted.begin(), sorted.end(), [](const Common::GeneratorLayer *a, const Common::GeneratorLayer *b) { return *a < *b; });
    const slic3r_api::ClipperContext clipper(SG::handle(routed.storage));
    size_t groups = 0, violations = 0;
    std::ostringstream report;
    for (size_t begin = 0; begin < sorted.size();) {
        size_t end = begin + 1;
        while (end < sorted.size() && sorted[end]->print_z == sorted[begin]->print_z && sorted[end]->height == sorted[begin]->height)
            ++end;
        slic3r_api::StoredPolygonCollection all(SG::handle(routed.storage));
        for (size_t idx = begin; idx < end; ++idx)
            all.append_copy_from(sorted[idx]->polygons);
        if (!all.empty()) {
            ++groups;
            const slic3r_api::StoredExPolygonCollection islands = Common::native_expolygons(slic3r_api::clipper_union(clipper(all)));
            if (island_order_violation(*reinterpret_cast<const ExPolygons *>(islands.handle()), earlier, later)) {
                ++violations;
                report << " z=" << unscaled(sorted[begin]->print_z) << " (island " << earlier << " listed before the island " << later << " that holds it)";
            }
        }
        begin = end;
    }
    REQUIRE(groups > 0);
    CAPTURE(groups, violations, report.str());
    CHECK(violations == 0);
}
} // namespace

// Close print Z: the print z of the supports are kept more than 4 * SCALED_EPSILON (512 units, 0.5 um) apart from the other z of the print by a host pass
// at the end of STEP_SUPPORT (Steps::StepGenerateSupport). The reproductions list the Z involved (on stderr, and in CLOSEZ_DUMP_DIR when that
// variable is set), establish on the slabs that the family generates that the close pair exists before the pass, then check the layers after the
// step and the export up to the G-code.
namespace {
namespace CloseZ = Slic3r::Steps::StepGenerateSupport::Detail;

struct CloseZLayer {
    size_t  object;
    bool    support;
    coord_t z;
    bool    extrusions;
};

struct CloseZObject {
    TriangleMesh                                      mesh;
    Vec3d                                             offset;
    std::vector<std::pair<std::string, std::string>> options;
};

void close_z_place(const std::vector<CloseZObject> &objects, const DynamicPrintConfig &config, Print &print, Model &model)
{
    for (const CloseZObject &placed : objects) {
        ModelObject *object = model.add_object();
        object->name += "object.stl";
        object->add_volume(placed.mesh);
        object->add_instance();
        object->instances.front()->set_offset(placed.offset);
        for (const std::pair<std::string, std::string> &option : placed.options)
            object->config.set_deserialize_strict(option.first, option.second);
    }
    for (ModelObject &object : model.objects()) {
        object.ensure_on_bed();
        print.auto_assign_extruders(&object);
    }
    print.apply(model, config);
    print.validate();
    print.set_status_silent();
}

std::vector<CloseZLayer> close_z_layers(const Print &print)
{
    std::vector<CloseZLayer> zs;
    for (size_t idx = 0; idx < print.objects().size(); ++idx) {
        const PrintObject &object = print.object(idx);
        for (const Layer &layer : object.layers())
            zs.push_back({ idx, false, layer.scaled_print_z(), layer.has_extrusions() });
        for (const Layer &layer : object.auxiliary_layers())
            zs.push_back({ idx, true, layer.scaled_print_z(), layer.has_extrusions() });
    }
    std::sort(zs.begin(), zs.end(), [](const CloseZLayer &a, const CloseZLayer &b) { return a.z < b.z; });
    return zs;
}

// The pairs of layers of the print whose Z differ by more than zero and by at most the merge tolerance, as the tool ordering sees them
// (only the layers with extrusions count when with_extrusions is set). Only the layers of the object are compared when object is given.
std::vector<std::pair<CloseZLayer, CloseZLayer>> close_z_pairs(const Print &print, bool with_extrusions, const size_t *object = nullptr)
{
    std::vector<CloseZLayer> zs;
    for (const CloseZLayer &layer : close_z_layers(print))
        if ((!with_extrusions || layer.extrusions) && (object == nullptr || layer.object == *object))
            zs.push_back(layer);
    std::vector<std::pair<CloseZLayer, CloseZLayer>> pairs;
    for (size_t i = 0; i < zs.size(); ++i)
        for (size_t j = i + 1; j < zs.size() && zs[j].z <= zs[i].z + 4 * SCALED_EPSILON; ++j)
            if (zs[j].z != zs[i].z)
                pairs.push_back({ zs[i], zs[j] });
    return pairs;
}

// Writes a report on stderr, and in CLOSEZ_DUMP_DIR/closez_<name>.txt (appended) when that variable is set. The stderr of a test case is
// lost when an assertion ends the process, the file is not.
void close_z_report(const std::string &name, const std::string &text)
{
    std::fputs(text.c_str(), stderr);
    std::fflush(stderr);
    if (const char *directory = std::getenv("CLOSEZ_DUMP_DIR")) {
        const std::string path = std::string(directory) + "/closez_" + name + ".txt";
        if (FILE *file = std::fopen(path.c_str(), "ab")) {
            std::fwrite(text.data(), 1, text.size(), file);
            std::fclose(file);
        }
    }
}

// Lists the Z of the supports of every object and the close pairs.
void close_z_dump(const char *label, const Print &print)
{
    std::ostringstream out;
    out << "\n[support-close-z] " << label << "\n";
    for (size_t idx = 0; idx < print.objects().size(); ++idx) {
        const PrintObject &object = print.object(idx);
        out << "  object " << idx << " support z (units):";
        for (const Layer &layer : object.auxiliary_layers())
            out << " " << layer.scaled_print_z() << (layer.has_extrusions() ? "" : "(empty)");
        out << "\n";
    }
    for (const auto &pair : close_z_pairs(print, false))
        out << "  close pair: object " << pair.first.object << " " << (pair.first.support ? "support" : "layer") << " z=" << pair.first.z
            << "  object " << pair.second.object << " " << (pair.second.support ? "support" : "layer") << " z=" << pair.second.z
            << "  (diff " << pair.second.z - pair.first.z << ")\n";
    close_z_report(label, out.str());
}

DynamicPrintConfig close_z_gcode_config(const char *style, int interfaces)
{
    DynamicPrintConfig config = SG::config(style, false, interfaces, true);
    config.set_deserialize_strict({
        {"z_step", "0"},
        {"step_gcode_plugin", "gcode.printing_plan_file_writer"},
        {"gcode_flavor", "marlin2"},
        {"support_material_speed", "40"}, {"support_material_interface_speed", "50%"},
        {"max_print_speed", "80"}, {"first_layer_speed", "100%"}, {"first_layer_min_speed", "0"},
        {"max_volumetric_speed", "0"}, {"filament_max_speed", "0,0"},
        {"filament_max_volumetric_speed", "0,0"}, {"slowdown_below_layer_time", "0,0"},
        {"use_relative_e_distances", "1"}, {"wipe", "0,0"}, {"filament_diameter", "1.75,1.75"}
    });
    return config;
}

// The Z (tops and published bottoms, print_z - height, the values that the host pass moves) of the slabs that the family generates for an object of the sliced print, before the host pass of STEP_SUPPORT.
std::vector<coord_t> close_z_generated(Print &print, size_t object_idx, const std::string &style)
{
    PluginStorage storage;
    Common::GeneratedSupport support{ reinterpret_cast<storage_handle *>(&storage) };
    const PrintObject &object = print.object(object_idx);
    if (style == "grid") {
        slic3r_api::Support::Grid::generate_support(nullptr, SG::print_view(print), SG::object_view(object), SG::params(print, object), support);
    } else {
        const Slic3r::Test::SupportTree::SupportPatternContext patterns(print, reinterpret_cast<storage_handle *>(&storage));
        TF::generate_tree_support(patterns.context(), SG::print_view(print), SG::object_view(object), SG::params(print, object), style == "organic", support);
    }
    std::vector<coord_t> zs;
    for (const auto *list : { &support.top, &support.bottom, &support.base, &support.interfaces })
        for (const Common::GeneratorLayer *slab : *list) {
            zs.push_back(slab->print_z);
            zs.push_back(slab->print_z - slab->height);
        }
    std::sort(zs.begin(), zs.end());
    zs.erase(std::unique(zs.begin(), zs.end()), zs.end());
    return zs;
}

// A pair of distinct Z within the merge tolerance, among the generated slabs of the objects (support) and the object layers.
struct CloseZGeneratedPair {
    size_t  object_a, object_b;
    bool    support_a, support_b;
    coord_t z_a, z_b;
};

// The pairs of distinct Z within the merge tolerance of the slabs that the family generates for each object and of the object layers (each
// listed with its object), with at least one slab in the pair.
std::vector<CloseZGeneratedPair> close_z_generated_pairs(Print &print, const std::string &style)
{
    struct Entry { coord_t z; size_t object; bool support; };
    std::vector<Entry> zs;
    for (size_t idx = 0; idx < print.objects().size(); ++idx) {
        for (const coord_t z : close_z_generated(print, idx, style))
            zs.push_back({ z, idx, true });
        for (const Layer &layer : print.object(idx).layers())
            zs.push_back({ layer.scaled_print_z(), idx, false });
    }
    std::sort(zs.begin(), zs.end(), [](const Entry &a, const Entry &b) { return a.z < b.z; });
    std::vector<CloseZGeneratedPair> pairs;
    for (size_t i = 0; i < zs.size(); ++i)
        for (size_t j = i + 1; j < zs.size() && zs[j].z <= zs[i].z + 4 * SCALED_EPSILON; ++j)
            if (zs[j].z != zs[i].z && (zs[i].support || zs[j].support))
                pairs.push_back({ zs[i].object, zs[j].object, zs[i].support, zs[j].support, zs[i].z, zs[j].z });
    return pairs;
}

// True when a pair has a slab and a slab of another object (cross_support), or a slab and an object layer of any object (otherwise).
bool close_z_generated_pair(Print &print, const std::string &style, bool cross_support)
{
    for (const CloseZGeneratedPair &pair : close_z_generated_pairs(print, style))
        if (cross_support ? (pair.support_a && pair.support_b && pair.object_a != pair.object_b) : (pair.support_a != pair.support_b))
            return true;
    return false;
}
// Runs the production steps up to the support toolpaths (STEP_SUPPORT included, so with the host pass on the support z).
void close_z_run_supports(Print &print)
{
    SG::prefix(print);
    SG::family(print);
    SG::toolpaths(print);
}

// The auxiliary layers of every object are in increasing z, the thickest first at an equal z.
void close_z_check_order(const Print &print)
{
    for (size_t idx = 0; idx < print.objects().size(); ++idx) {
        const PrintObject &object = print.object(idx);
        for (size_t id = 1; id < object.auxiliary_layers().size(); ++id) {
            const Layer &before = object.auxiliary_layer(id - 1), &after = object.auxiliary_layer(id);
            CHECK(before.scaled_print_z() <= after.scaled_print_z());
            if (before.scaled_print_z() == after.scaled_print_z())
                CHECK(before.scaled_height() >= after.scaled_height());
        }
    }
}

// The G-code of the print. The text is also kept in the directory of CLOSEZ_DUMP_DIR when it is set.
std::string close_z_export(Print &print, const char *name)
{
    const boost::filesystem::path path = Slic3r::Test::GCode::temporary_gcode_path();
    std::string code;
    try {
        print.process();
        Orchestrator::instance().export_gcode(print, path.string(), nullptr);
        code = Slic3r::Test::GCode::read_text_file(path);
    } catch (...) {
        Slic3r::Test::GCode::remove_output_pair(path);
        throw;
    }
    Slic3r::Test::GCode::remove_output_pair(path);
    if (const char *directory = std::getenv("CLOSEZ_DUMP_DIR")) {
        const std::string copy = std::string(directory) + "/closez_" + name + ".gcode";
        if (FILE *file = std::fopen(copy.c_str(), "wb")) {
            std::fwrite(code.data(), 1, code.size(), file);
            std::fclose(file);
        }
    }
    return code;
}

// The Z of the layer changes of the G-code, in the order they are printed.
std::vector<double> close_z_layer_changes(const std::string &code)
{
    std::vector<double> zs;
    std::istringstream stream(code);
    for (std::string line; std::getline(stream, line);)
        if (line.rfind(";Z:", 0) == 0)
            zs.push_back(std::stod(line.substr(3)));
    return zs;
}

// Checks that the layer changes of the G-code never go back (one object at a time, a sequential print restarts from the bottom for each).
void close_z_check_gcode(const std::string &code, const char *name, bool sequential = false)
{
    const std::vector<double> zs = close_z_layer_changes(code);
    REQUIRE(zs.size() > 2);
    size_t backwards = 0;
    for (size_t i = 1; i < zs.size(); ++i)
        if (zs[i] < zs[i - 1] - 1e-9 && !(sequential && zs[i] < 0.5))
            ++backwards;
    close_z_report(name, "[support-close-z] " + std::to_string(zs.size()) + " layer changes, " + std::to_string(backwards) + " going backwards\n");
    CHECK(backwards == 0);
}

// Two objects with the contact distances distance_a and distance_b: the contacts (and with them the slabs that start or end at them) lie
// 100 units apart, away from every object layer, so the bound snapping of the tree cannot help.
void close_z_two_objects(const char *style, const char *name, const char *distance_a, const char *distance_b)
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    close_z_place({
        { SG::slab(true), Vec3d(100., 100., 0.), { {"support_material_contact_distance", distance_a}, {"support_material_bottom_contact_distance", distance_a} } },
        { SG::slab(true), Vec3d(160., 100., 0.), { {"support_material_contact_distance", distance_b}, {"support_material_bottom_contact_distance", distance_b} } } },
        close_z_gcode_config(style, 0), print, model);
    SG::prefix(print);
    // The close pair exists between the slabs that the families generate for the two objects (a support against a support of the other one).
    REQUIRE(close_z_generated_pair(print, style, true));
    SG::family(print);
    SG::toolpaths(print);
    close_z_dump(name, print);
    CHECK(close_z_pairs(print, false).empty());
    close_z_check_order(print);
    const std::string code = close_z_export(print, name);
    close_z_check_gcode(code, name);
}
} // namespace

// The mapping of the host pass: fixed values never move, a movable value within 512 units of one takes the nearest (the lower on a tie),
// the other movable values are merged in ascending order onto the first of a series.
TEST_CASE("Close Z mapping near a fixed value", "[plugins][support-tree-family][support-close-z][support-close-z-pure]")
{
    const coord_t fixed = 1000000;
    const CloseZ::CloseZMapping mapping = CloseZ::close_z_mapping({ fixed, fixed + 5000 }, { fixed - 512, fixed - 513, fixed + 512, fixed + 513, fixed + 2000 });
    CHECK(CloseZ::close_z_target(mapping, fixed - 512) == fixed);
    CHECK(CloseZ::close_z_target(mapping, fixed - 513) == fixed - 513);
    CHECK(CloseZ::close_z_target(mapping, fixed + 512) == fixed);
    CHECK(CloseZ::close_z_target(mapping, fixed + 513) == fixed + 513);
    CHECK(CloseZ::close_z_target(mapping, fixed + 2000) == fixed + 2000);
    // Equal to a fixed value: unchanged and not listed.
    CHECK(CloseZ::close_z_mapping({ fixed }, { fixed }).empty());
    // Both sides of a fixed value, the nearest wins and the lower one wins a tie.
    const CloseZ::CloseZMapping sides = CloseZ::close_z_mapping({ fixed, fixed + 1024 }, { fixed + 300, fixed + 724, fixed + 512 });
    CHECK(CloseZ::close_z_target(sides, fixed + 300) == fixed);
    CHECK(CloseZ::close_z_target(sides, fixed + 724) == fixed + 1024);
    CHECK(CloseZ::close_z_target(sides, fixed + 512) == fixed);
}

TEST_CASE("Close Z mapping merges chains of movable values", "[plugins][support-tree-family][support-close-z][support-close-z-pure]")
{
    const coord_t z = 2000000;
    // z, z + 400, z + 800: the second takes the first, the third is more than 512 from the kept one and stays.
    const CloseZ::CloseZMapping chain = CloseZ::close_z_mapping({}, { z + 800, z, z + 400, z + 400 });
    CHECK(CloseZ::close_z_target(chain, z) == z);
    CHECK(CloseZ::close_z_target(chain, z + 400) == z);
    CHECK(CloseZ::close_z_target(chain, z + 800) == z + 800);
    // The kept value is the first of the series, not the nearest of the values taken.
    const CloseZ::CloseZMapping series = CloseZ::close_z_mapping({}, { z, z + 512, z + 513, z + 1024 });
    CHECK(CloseZ::close_z_target(series, z + 512) == z);
    CHECK(CloseZ::close_z_target(series, z + 513) == z + 513);
    CHECK(CloseZ::close_z_target(series, z + 1024) == z + 513);
    // A value taken by a fixed value does not become the kept value of the sweep.
    const CloseZ::CloseZMapping taken = CloseZ::close_z_mapping({ z }, { z + 300, z + 700 });
    CHECK(CloseZ::close_z_target(taken, z + 300) == z);
    CHECK(CloseZ::close_z_target(taken, z + 700) == z + 700);
    // Empty inputs.
    CHECK(CloseZ::close_z_mapping({}, {}).empty());
    CHECK(CloseZ::close_z_mapping({ z }, {}).empty());
    CHECK(CloseZ::close_z_target({}, z) == z);
}

// Guarantee: each value moves by at most 512 units, and two distinct results (fixed values included) are more than 512 units apart.
TEST_CASE("Close Z mapping guarantee on many values", "[plugins][support-tree-family][support-close-z][support-close-z-pure]")
{
    std::vector<coord_t> fixed, movable;
    for (coord_t z = 200000; z <= 220000; z += 2000)
        fixed.push_back(z);
    uint32_t state = 12345;
    for (int i = 0; i < 600; ++i) {
        state = state * 1664525u + 1013904223u;
        movable.push_back(coord_t(200000 + 100 * (state >> 16) % 21000));
    }
    const CloseZ::CloseZMapping mapping = CloseZ::close_z_mapping(fixed, movable);
    std::vector<coord_t> results = fixed;
    for (const coord_t z : movable) {
        const coord_t target = CloseZ::close_z_target(mapping, z);
        CHECK(std::abs(target - z) <= 4 * SCALED_EPSILON);
        results.push_back(target);
    }
    std::sort(results.begin(), results.end());
    results.erase(std::unique(results.begin(), results.end()), results.end());
    for (size_t i = 1; i < results.size(); ++i)
        CHECK(results[i] - results[i - 1] > 4 * SCALED_EPSILON);
}

TEST_CASE("Close Z two objects tree", "[plugins][support-tree-family][support-close-z][support-close-z-tree]")
{
    close_z_two_objects("tree", "tree", "0.25", "0.2501");
}

TEST_CASE("Close Z two objects organic", "[plugins][support-tree-family][support-close-z][support-close-z-organic]")
{
    close_z_two_objects("organic", "organic", "0.25", "0.2501");
}

TEST_CASE("Close Z two objects grid", "[plugins][support-tree-family][support-close-z][support-close-z-grid]")
{
    close_z_two_objects("grid", "grid", "0.25", "0.2501");
}

// One grid object whose floor contact lies 62 um under its top contact, with a support layer height (60 um) not much above the minimum
// height (50 um): the 62 um between the two extremes are split in two slabs thinner than the minimum height, and the top contact is one of them.
// The host pass does not change this (a known limit): the slabs print as they are.
TEST_CASE("Close Z grid contact thinner than the minimum", "[plugins][support-tree-family][support-close-z][support-close-z-thin]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = close_z_gcode_config("grid", 0);
    config.set_deserialize_strict({ {"support_material_contact_distance", "2.738"}, {"support_material_layer_height", "0.06"} });
    Print print;
    Model model;
    close_z_place({ { SG::slab(true), Vec3d(100., 100., 0.), {} } }, config, print, model);
    SG::prefix(print);
    std::ostringstream out;
    const coord_t minimum = SG::params(print, print.object(0)).min_height;
    out << "\n[support-close-z] thin\n  minimum height " << minimum << "\n";
    size_t thinner = 0;
    {
        const SG::Generated generated(print, print.object(0));
        for (const auto &role : { std::make_pair("top", &generated.support.top), std::make_pair("bottom", &generated.support.bottom),
                                  std::make_pair("base", &generated.support.base) })
            for (const Common::GeneratorLayer *slab : *role.second) {
                out << "  " << role.first << " " << slab->bottom_z << ".." << slab->print_z << " height " << slab->height << (slab->height < minimum ? "  THINNER" : "") << "\n";
                thinner += slab->height < minimum;
            }
    }
    close_z_report("thin", out.str());
    REQUIRE(thinner > 0);
    const std::string code = close_z_export(print, "thin");
    close_z_check_gcode(code, "thin");
}

namespace {
// One object whose top contact lies 0.3 um under an object layer (contact distance 0.3997 mm under the roof at 4 mm, so at 3.6003 mm and the
// object layer is at 3.6 mm). The tree snaps its bounds onto the object layer; the grid does not, so the host pass moves its slabs.
void close_z_one_object(const char *style, const char *name)
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = close_z_gcode_config(style, 0);
    config.set_deserialize_strict({ {"support_material_contact_distance", "0.3997"} });
    Print print;
    Model model;
    close_z_place({ { SG::slab(true), Vec3d(100., 100., 0.), {} } }, config, print, model);
    SG::prefix(print);
    // The grid generates slabs close to its own object layers; the tree already snaps them.
    CHECK(close_z_generated_pair(print, style, false) == (std::string(style) == "grid"));
    SG::family(print);
    SG::toolpaths(print);
    close_z_dump(name, print);
    CHECK(close_z_pairs(print, false).empty());
    close_z_check_order(print);
    const std::string code = close_z_export(print, name);
    close_z_check_gcode(code, name);
}

// The path measures of the auxiliary layers of an object at a print z, per tool and role.
SG::PathMeasures close_z_measure_support(const Print &print, coord_t z)
{
    SG::PathMeasures total;
    for (const Layer &layer : print.object(0).auxiliary_layers())
        if (layer.scaled_print_z() == z)
        for (const auto &entry : SG::measure_layer(layer)) {
            SG::PathMeasure &measure = total[entry.first];
            measure.length += entry.second.length;
            measure.volume += entry.second.volume;
            measure.count += entry.second.count;
        }
    return total;
}
} // namespace

TEST_CASE("Close Z one object grid contact near an object layer", "[plugins][support-tree-family][support-close-z][support-close-z-one-grid]")
{
    close_z_one_object("grid", "one-grid");
}

TEST_CASE("Close Z one object tree contact near an object layer", "[plugins][support-tree-family][support-close-z][support-close-z-one-tree]")
{
    close_z_one_object("tree", "one-tree");
}

// With interface layers, the contact that the pass moves onto the object layer is still recognised as a contact by the toolpaths: the
// paths of its layer (count and length per tool and role, removable contact loops included; the volume depends on the slab thickness) match those of the print whose contact
// lies exactly on the object layer (contact distance 0.4 mm, no close z). The other slabs are not compared: their planning differs.
TEST_CASE("Close Z moved contact stays a contact for the toolpaths", "[plugins][support-tree-family][support-close-z][support-close-z-contact]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const auto measure = [](const char *distance, bool close) {
        DynamicPrintConfig config = close_z_gcode_config("grid", 3);
        config.set_deserialize_strict({ {"support_material_contact_distance", distance}, {"support_material_interface_contact_loops", "1"} });
        Print print;
        Model model;
        close_z_place({ { SG::slab(true), Vec3d(100., 100., 0.), {} } }, config, print, model);
        SG::prefix(print);
        CHECK(close_z_generated_pair(print, "grid", false) == close);
        SG::family(print);
        SG::toolpaths(print);
        CHECK(close_z_pairs(print, false).empty());
        return close_z_measure_support(print, scale_to_layer_coord(3.6));
    };
    const SG::PathMeasures moved = measure("0.3997", true), reference = measure("0.4", false);
    REQUIRE_FALSE(reference.empty());
    REQUIRE(moved.size() == reference.size());
    for (const auto &entry : reference) {
        const auto found = moved.find(entry.first);
        REQUIRE(found != moved.end());
        CAPTURE(entry.first.second, entry.second.count, found->second.count, entry.second.length, found->second.length);
        CHECK(found->second.count == entry.second.count);
        CHECK(found->second.length == Approx(entry.second.length).epsilon(0.02));
    }
}

namespace {
// A print using the By object builder with two objects whose layer heights are 0.2 and
// 0.2001 mm: the layers of the two objects are close to one another, which only the layer-by-layer plan would mind. Each object gets its
// own pass: no close pair inside an object, and no support z of one object was moved onto a layer z of the other.
void close_z_object_by_object(const char *option, const char *value, const char *name)
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = close_z_gcode_config("grid", 0);
    config.set_deserialize_strict("print_plan_builder", "ordering.plan_builder.by_object");
    config.set_deserialize_strict({ {option, value} });
    Print print;
    Model model;
    close_z_place({
        { SG::slab(true), Vec3d(100., 100., 0.), {} },
        { SG::slab(true), Vec3d(160., 100., 0.), { {"layer_height", "0.2001"} } } },
        config, print, model);
    SG::prefix(print);
    std::vector<std::vector<coord_t>> generated;
    for (size_t idx = 0; idx < 2; ++idx)
        generated.push_back(close_z_generated(print, idx, "grid"));
    SG::family(print);
    SG::toolpaths(print);
    close_z_dump(name, print);
    for (size_t idx = 0; idx < 2; ++idx) {
        CHECK(close_z_pairs(print, false, &idx).empty());
        std::set<coord_t> own;
        for (const Layer &layer : print.object(idx).layers())
            own.insert(layer.scaled_print_z());
        std::set<coord_t> other;
        for (const Layer &layer : print.object(1 - idx).layers())
            other.insert(layer.scaled_print_z());
        for (const Layer &layer : print.object(idx).auxiliary_layers()) {
            const coord_t z = layer.scaled_print_z();
            // Not equal to a layer of the other object, unless it already was (or equals a layer of its own object).
            CHECK((other.count(z) == 0 || own.count(z) == 1 || std::binary_search(generated[idx].begin(), generated[idx].end(), z)));
            // Every z is one of the generated slabs, or a z of the object (or moved by at most the tolerance onto an object layer).
            const bool known = std::binary_search(generated[idx].begin(), generated[idx].end(), z) ||
                std::any_of(own.begin(), own.end(), [z](coord_t o) { return std::abs(o - z) <= 4 * SCALED_EPSILON; });
            CHECK(known);
        }
    }
    close_z_check_order(print);
    const std::string code = close_z_export(print, name);
    close_z_check_gcode(code, name, true);
}
} // namespace

TEST_CASE("Close Z sequential print passes per object", "[plugins][support-tree-family][support-close-z][support-close-z-sequential]")
{
    close_z_object_by_object("print_plan_builder", "ordering.plan_builder.by_object", "sequential");
}

TEST_CASE("Close Z parallel objects step passes per object", "[plugins][support-tree-family][support-close-z][support-close-z-parallel]")
{
    close_z_object_by_object("complete_objects_step", "5", "parallel");
}

// The internal setter moves a layer and its slice z and keeps its id.
TEST_CASE("Close Z layer setter keeps the id and moves the slice z", "[plugins][support-tree-family][support-close-z][support-close-z-setter]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    close_z_place({ { SG::slab(true), Vec3d(100., 100., 0.), {} } }, close_z_gcode_config("grid", 0), print, model);
    close_z_run_supports(print);
    PrintObject &object = print.object(0);
    REQUIRE(object.auxiliary_layers().size() > 12);
    Layer &layer = *object.mutable_auxiliary_layers()[12];
    const size_t id = layer.id();
    const coord_t z = layer.scaled_print_z(), height = layer.scaled_height();
    ApiInternal::LayerAccess::set_z(layer, z - 300, height - 300);
    CHECK(layer.scaled_print_z() == z - 300);
    CHECK(layer.scaled_height() == height - 300);
    CHECK(layer.scaled_bottom_z() == z - height);
    CHECK(layer.slice_z == Approx(unscaled(z - 300 - (height - 300) / 2)));
    CHECK(layer.id() == id);
}

// The sort of the pass: layers put out of order with set_z come back by increasing print z, and the thickest first at an equal print z.
TEST_CASE("Close Z support layers are sorted by z then thickness", "[plugins][support-tree-family][support-close-z][support-close-z-sort]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    close_z_place({ { SG::slab(true), Vec3d(100., 100., 0.), {} } }, close_z_gcode_config("grid", 0), print, model);
    close_z_run_supports(print);
    LayerUPtrs &layers = print.object(0).mutable_auxiliary_layers();
    REQUIRE(layers.size() > 12);
    const Layer *thin = layers[3].get(), *middle = layers[9].get(), *thick = layers[10].get();
    const coord_t z = middle->scaled_print_z(), height = middle->scaled_height();
    // Layer 3 jumps to the z of layer 9 with a thinner height, layer 10 comes down to it with a thicker one.
    ApiInternal::LayerAccess::set_z(*layers[3], z, height / 100 / 2 * 100);
    ApiInternal::LayerAccess::set_z(*layers[10], z, height + 100);
    Slic3r::Steps::StepGenerateSupport::Detail::sort_support_layers(layers);
    std::vector<const Layer *> order;
    for (const LayerUPtr &layer : layers)
        order.push_back(layer.get());
    const auto position = [&order](const Layer *layer) { return std::find(order.begin(), order.end(), layer) - order.begin(); };
    CHECK(position(thick) + 1 == position(middle));
    CHECK(position(middle) + 1 == position(thin));
    close_z_check_order(print);
}

// The tree's own snapping and the host pass can both move the same contact. A full print cannot show it (a floor contact and a top contact
// 0.4 um apart leave no body between them, and the families publish no support there), so the two moves are chained on a synthetic support:
// the tree merges a floor contact and a top contact 0.4 um above it (align_slab_bounds), then the mapping of the host pass takes the merged z to
// the z of a support of another object, 0.5 um lower. The contact moves by more than 640 units in all, and still by at most the contact match
// tolerance of the toolpaths (SCALED_EPSILON + 2 * the close-Z tolerance limit, which is 4 * SCALED_EPSILON without a Z step; LegacySupportPaths.cpp).
TEST_CASE("Close Z contact moved by the tree and by the pass stays within the contact match", "[plugins][support-tree-family][support-close-z][support-close-z-cumulative]")
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
    const coord_t floor_top = scale_to_layer_coord(1.25), contact_top = scale_to_layer_coord(1.2504);
    Common::GeneratorLayer &floor_contact = Common::add_layer(support, Common::LayerType::BottomContact, floor_top, scale_to_layer_coord(1.05), scale_to_layer_coord(0.2));
    Common::GeneratorLayer &overhang_contact = Common::add_layer(support, Common::LayerType::TopContact, contact_top, scale_to_layer_coord(1.0504), scale_to_layer_coord(0.2));
    support.bottom = { &floor_contact };
    support.top = { &overhang_contact };
    Polygon floor_circle = make_circle(scale_d(1.), 1000.), overhang_circle = make_circle(scale_d(1.), 1000.);
    floor_circle.translate(Point(scale_i(-4.), coord_t(0)));
    overhang_circle.translate(Point(scale_i(4.), coord_t(0)));
    assign_native(floor_contact.polygons, Polygons{ floor_circle });
    assign_native(overhang_contact.polygons, Polygons{ overhang_circle });
    TF::align_slab_bounds(object_zs, support);
    const coord_t merged = overhang_contact.print_z;
    REQUIRE(floor_contact.print_z == merged);
    const coord_t by_tree = merged - contact_top;
    // A support of another object, 500 units under the merged z on the far side from the contact.
    const coord_t other = merged + (by_tree < 0 ? -500 : 500);
    const Slic3r::Steps::StepGenerateSupport::Detail::CloseZMapping mapping =
        Slic3r::Steps::StepGenerateSupport::Detail::close_z_mapping(object_zs, { other, merged });
    const coord_t final_z = Slic3r::Steps::StepGenerateSupport::Detail::close_z_target(mapping, merged);
    close_z_report("cumulative", "[support-close-z] contact z " + std::to_string(contact_top) + ", merged by the tree " + std::to_string(merged) +
        ", after the pass " + std::to_string(final_z) + "\n");
    CHECK(std::abs(by_tree) > 0);
    const coord_t cumulative = std::abs(final_z - contact_top);
    CHECK(cumulative > 640);
    CHECK(cumulative <= 1024);
    CHECK(cumulative <= SCALED_EPSILON + 2 * Common::close_z_tolerance_limit(0.));
}

// The tolerance of the close-Z passes is max(z_step / 2, 0.5 um), never below the 512 units that the tool ordering requires, and bounded by half the
// distance between two object Z when the object layers are closer than the Z step suggests.
TEST_CASE("Close Z tolerance follows the Z step", "[plugins][support-tree-family][support-close-z][support-close-z-pure]")
{
    CHECK(Common::min_close_z_tolerance == 512);
    // Without a Z step, or with one smaller than a micron, the tolerance stays 0.5 um.
    CHECK(Common::close_z_tolerance_limit(0.) == 512);
    CHECK(Common::close_z_tolerance_limit(0.001) == 512);
    CHECK(Common::close_z_tolerance_limit(0.005) == 2500);
    CHECK(Common::close_z_tolerance_limit(0.04) == 20000);
    const std::vector<coord_t> layers{ 0, 200000, 400000 };
    CHECK(Common::close_z_tolerance(0., layers) == 512);
    CHECK(Common::close_z_tolerance(0.005, layers) == 2500);
    // Object Z closer than the step: half their distance, but never below 512.
    CHECK(Common::close_z_tolerance(0.005, { 0, 1500, 200000 }) == 750);
    CHECK(Common::close_z_tolerance(0.005, { 0, 600, 200000 }) == 512);
    CHECK(Common::close_z_tolerance(0.005, { 100, 100, 200000 }) == 2500);
}

// The mapping of the host pass with a tolerance larger than 0.5 um: a movable value 2 um from a fixed one takes it, two movable values 2 um
// apart merge, and fixed values stay; with the default tolerance (no Z step) none of them is merged.
TEST_CASE("Close Z mapping with the tolerance of a Z step", "[plugins][support-tree-family][support-close-z][support-close-z-pure]")
{
    const coord_t fixed = 1000000, tolerance = Common::close_z_tolerance(0.005, { fixed, fixed + 200000 });
    REQUIRE(tolerance == 2500);
    const CloseZ::CloseZMapping mapping = CloseZ::close_z_mapping({ fixed, fixed + 200000 },
        { fixed - 2000, fixed + 2500, fixed + 2501, fixed + 100000, fixed + 102000 }, tolerance);
    CHECK(CloseZ::close_z_target(mapping, fixed - 2000) == fixed);
    CHECK(CloseZ::close_z_target(mapping, fixed + 2500) == fixed);
    CHECK(CloseZ::close_z_target(mapping, fixed + 2501) == fixed + 2501);
    CHECK(CloseZ::close_z_target(mapping, fixed + 100000) == fixed + 100000);
    CHECK(CloseZ::close_z_target(mapping, fixed + 102000) == fixed + 100000);
    const CloseZ::CloseZMapping plain = CloseZ::close_z_mapping({ fixed, fixed + 200000 },
        { fixed - 2000, fixed + 100000, fixed + 102000 }, Common::close_z_tolerance(0., { fixed, fixed + 200000 }));
    CHECK(CloseZ::close_z_target(plain, fixed - 2000) == fixed - 2000);
    CHECK(CloseZ::close_z_target(plain, fixed + 102000) == fixed + 102000);
}

namespace {
// Two base slabs that meet at 1.598 mm, 2 um under the object layer at 1.6 mm, on a print with the given Z step; the tree alignment runs with the tolerance
// that generate_tree_slabs() computes for it. Returns the tolerance, the top of the lower slab, the bottom of the upper one and the object layer top.
struct SnapResult {
    coord_t tolerance, lower_top, upper_bottom, object_top;
};
SnapResult tree_snap_near_object_layer(const char *z_step)
{
    DynamicPrintConfig config = SG::config("tree", false, 0);
    config.set_deserialize_strict({ {"z_step", z_step} });
    Print print;
    Model model;
    Test::init_print({ SG::slab() }, print, model, config);
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    std::vector<coord_t> object_zs;
    for (size_t id = 0; id < object.layer_count(); ++id)
        object_zs.push_back(object.layer(id).scaled_print_z());
    const coord_t top = scale_to_layer_coord(1.6), bound = scale_to_layer_coord(1.598);
    REQUIRE(std::find(object_zs.begin(), object_zs.end(), top) != object_zs.end());
    const coord_t tolerance = Common::close_z_tolerance(SG::params(print, object).z_step, object_zs);
    PluginStorage storage;
    Common::GeneratedSupport support{ reinterpret_cast<storage_handle *>(&storage) };
    Common::GeneratorLayer &lower = Common::add_layer(support, Common::LayerType::Intermediate, bound, scale_to_layer_coord(1.), bound - scale_to_layer_coord(1.));
    Common::GeneratorLayer &upper = Common::add_layer(support, Common::LayerType::Intermediate, scale_to_layer_coord(2.), bound, scale_to_layer_coord(2.) - bound);
    support.base = { &lower, &upper };
    assign_native(lower.polygons, Polygons{ make_circle(scale_d(2.), 1000.) });
    assign_native(upper.polygons, Polygons{ make_circle(scale_d(1.), 1000.) });
    TF::align_slab_bounds(object_zs, support, tolerance);
    return { tolerance, lower.print_z, upper.bottom_z, top };
}
} // namespace

// A tree bound 2 um from an object Z is snapped onto it when the printer has a Z step of 0.005 mm (tolerance 2.5 um), and stays where it is without one (0.5 um).
TEST_CASE("Close Z tree bound near an object layer follows the Z step", "[plugins][support-tree-family][support-close-z][support-close-z-step]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const SnapResult stepped = tree_snap_near_object_layer("0.005");
    CHECK(stepped.tolerance == 2500);
    CHECK(stepped.lower_top == stepped.object_top);
    CHECK(stepped.upper_bottom == stepped.object_top);
    const SnapResult plain = tree_snap_near_object_layer("0");
    CHECK(plain.tolerance == 512);
    CHECK(plain.lower_top == scale_to_layer_coord(1.598));
    CHECK(plain.upper_bottom == scale_to_layer_coord(1.598));
}

// Two support bounds 2 um apart (a floor contact top and an overhang contact top, no object layer near) share one Z with a Z step of 0.005 mm,
// and keep their distance without one.
TEST_CASE("Close Z support bounds 2 um apart merge with a Z step", "[plugins][support-tree-family][support-close-z][support-close-z-step]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const auto merged = [](const char *z_step) {
        DynamicPrintConfig config = SG::config("tree", false, 0);
        config.set_deserialize_strict({ {"z_step", z_step} });
        Print print;
        Model model;
        Test::init_print({ SG::slab() }, print, model, config);
        SG::prefix(print);
        const PrintObject &object = print.object(0);
        std::vector<coord_t> object_zs;
        for (size_t id = 0; id < object.layer_count(); ++id)
            object_zs.push_back(object.layer(id).scaled_print_z());
        PluginStorage storage;
        Common::GeneratedSupport support{ reinterpret_cast<storage_handle *>(&storage) };
        const coord_t floor_top = scale_to_layer_coord(1.1), contact_top = scale_to_layer_coord(1.102);
        REQUIRE(contact_top - floor_top == 2000);
        Common::GeneratorLayer &floor_contact = Common::add_layer(support, Common::LayerType::BottomContact, floor_top, scale_to_layer_coord(0.8), scale_to_layer_coord(0.3));
        Common::GeneratorLayer &overhang_contact = Common::add_layer(support, Common::LayerType::TopContact, contact_top, scale_to_layer_coord(0.902), scale_to_layer_coord(0.2));
        support.bottom = { &floor_contact };
        support.top = { &overhang_contact };
        Polygon floor_circle = make_circle(scale_d(1.), 1000.), overhang_circle = make_circle(scale_d(1.), 1000.);
        floor_circle.translate(Point(scale_i(-4.), coord_t(0)));
        overhang_circle.translate(Point(scale_i(4.), coord_t(0)));
        assign_native(floor_contact.polygons, Polygons{ floor_circle });
        assign_native(overhang_contact.polygons, Polygons{ overhang_circle });
        TF::align_slab_bounds(object_zs, support, Common::close_z_tolerance(SG::params(print, object).z_step, object_zs));
        return std::make_pair(floor_contact.print_z, overhang_contact.print_z);
    };
    const std::pair<coord_t, coord_t> stepped = merged("0.005");
    CHECK(stepped.first == stepped.second);
    const std::pair<coord_t, coord_t> plain = merged("0");
    CHECK(plain.second - plain.first == 2000);
}

// The host pass of STEP_SUPPORT uses the same rule: a support layer 2 um under an object layer is moved onto it with a Z step of 0.005 mm, and stays
// where it is without one. The layer is added to the object before the pass, and recognised by its id (the pass does not renumber).
TEST_CASE("Close Z host pass follows the Z step", "[plugins][support-tree-family][support-close-z][support-close-z-step]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const auto final_z = [](const char *z_step) {
        DynamicPrintConfig config = SG::config("grid", false, 0);
        config.set_deserialize_strict({ {"z_step", z_step} });
        Print print;
        Model model;
        Test::init_print({ SG::slab() }, print, model, config);
        SG::prefix(print);
        PrintObject &object = print.object(0);
        constexpr size_t marker = 99999;
        object.add_auxiliary_layer(marker, scale_to_layer_coord(0.123), scale_to_layer_coord(1.598));
        SG::family(print);
        for (const Layer &layer : object.auxiliary_layers())
            if (layer.id() == marker)
                return layer.scaled_print_z();
        FAIL("the marker layer disappeared");
        return coord_t(0);
    };
    CHECK(final_z("0.005") == scale_to_layer_coord(1.6));
    CHECK(final_z("0") == scale_to_layer_coord(1.598));
}

// With a Z step of 0.005 mm the contact that lies on an object layer is still recognised as a contact by the toolpaths (the match tolerance is
// computed from the Z step), also when the contact layer was moved by 3000 units (3 um) after the families published it, more than the old
// match window (SCALED_EPSILON + 2 * 512 = 1152 units) allowed and within the new one (SCALED_EPSILON + 2 * 2500 = 5128): the paths of its layer
// match those of the print without a Z step and without a move.
TEST_CASE("Close Z contact stays a contact with a Z step", "[plugins][support-tree-family][support-close-z][support-close-z-step]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    constexpr coord_t moved_by = 3000;
    CHECK(moved_by > SCALED_EPSILON + 2 * Common::min_close_z_tolerance);
    CHECK(moved_by <= SCALED_EPSILON + 2 * Common::close_z_tolerance_limit(0.005));
    const auto measure = [](const char *z_step, coord_t move) {
        DynamicPrintConfig config = close_z_gcode_config("grid", 3);
        config.set_deserialize_strict({ {"z_step", z_step}, {"support_material_contact_distance", "0.4"}, {"support_material_interface_contact_loops", "1"} });
        Print print;
        Model model;
        close_z_place({ { SG::slab(true), Vec3d(100., 100., 0.), {} } }, config, print, model);
        SG::prefix(print);
        SG::family(print);
        const coord_t contact_z = scale_to_layer_coord(3.6);
        // Like the host pass would: the top of the contact layer moves, its bottom stays.
        size_t moved = 0;
        for (const LayerUPtr &layer : print.object(0).mutable_auxiliary_layers())
            if (move != 0 && layer->scaled_print_z() == contact_z) {
                ApiInternal::LayerAccess::set_z(*layer, contact_z + move, layer->scaled_height() + move);
                ++moved;
            }
        CHECK((move == 0 || moved > 0));
        SG::toolpaths(print);
        CHECK(close_z_pairs(print, false).empty());
        return close_z_measure_support(print, contact_z + move);
    };
    const SG::PathMeasures stepped = measure("0.005", moved_by), reference = measure("0", 0);
    REQUIRE_FALSE(reference.empty());
    REQUIRE(stepped.size() == reference.size());
    for (const auto &entry : reference) {
        const auto found = stepped.find(entry.first);
        REQUIRE(found != stepped.end());
        CAPTURE(entry.first.second, entry.second.count, found->second.count, entry.second.length, found->second.length);
        CHECK(found->second.count == entry.second.count);
        CHECK(found->second.length == Approx(entry.second.length).epsilon(0.02));
    }
}

namespace {
// Whether the segment ab meets the island, by the host polyline/polygon intersection that the legacy toolpaths use to assign a path to the islands it
// crosses (an end inside, a pass through the contour or through vertices): the part of the segment inside the island is not empty.
bool segment_meets_island(Point a, Point b, const ExPolygon &island)
{
    return !intersection_pl(Polyline(a, b), island).empty();
}
// Counts the polyline leaves of an extrusion tree and those that meet an island other than their own (a segment that has a part inside it).
void count_island_crossings(const ExtrusionEntity &entity, const std::vector<const LayerSliceIsland *> &islands, size_t own, size_t &leaves, size_t &crossing)
{
    if (entity.has_polyline()) {
        ++leaves;
        const ArcPolyline &line = entity.polyline_ref();
        bool crosses = false;
        for (size_t idx = 0; idx < line.size() && !crosses; ++idx)
            for (size_t other = 0; other < islands.size() && !crosses; ++other)
                if (other != own)
                    crosses = segment_meets_island(line.get_point(idx), line.get_point(idx > 0 ? idx - 1 : idx), islands[other]->get_slice());
        crossing += crosses ? 1 : 0;
    }
    for (const ExtrusionEntityUPtr &child : entity.children())
        count_island_crossings(*child, islands, own, leaves, crossing);
}

// The result of publishing slabs of one height group and running the legacy toolpaths: the layers and islands, the area of each island in
// publication order, the number of support paths, those that have a point in an island other than their own, and those that have points in two
// of the given zones.
struct Published {
    size_t layers = 0, islands = 0, leaves = 0, crossing = 0, spanning = 0;
    std::vector<double> areas;
};
// Whether a leaf has points in two different zones.
void count_zone_spans(const ExtrusionEntity &entity, const ExPolygons &zones, size_t &spanning)
{
    if (entity.has_polyline()) {
        const ArcPolyline &line = entity.polyline_ref();
        std::vector<bool> reached(zones.size(), false);
        for (size_t idx = 0; idx < line.size(); ++idx)
            for (size_t zone = 0; zone < zones.size(); ++zone)
                if (zones[zone].contains(line.get_point(idx)))
                    reached[zone] = true;
        spanning += std::count(reached.begin(), reached.end(), true) > 1 ? 1 : 0;
    }
    for (const ExtrusionEntityUPtr &child : entity.children())
        count_zone_spans(*child, zones, spanning);
}
// Publishes one base slab per entry of slab_rings (same print Z and height), runs the toolpaths and measures the result.
Published publish_pieces(const std::vector<Polygons> &slab_rings, const ExPolygons &zones)
{
    DynamicPrintConfig config = SG::config("tree", false, 0);
    Print print;
    Model model;
    Test::init_print({ SG::slab() }, print, model, config);
    SG::prefix(print);
    PrintObject &object = print.object(0);
    PluginStorage storage;
    Common::GeneratedSupport support{ reinterpret_cast<storage_handle *>(&storage) };
    const coord_t top = scale_to_layer_coord(1.0), height = scale_to_layer_coord(0.2);
    for (const Polygons &rings : slab_rings) {
        Common::GeneratorLayer &slab = Common::add_layer(support, Common::LayerType::Intermediate, top, top - height, height);
        support.base.push_back(&slab);
        assign_native(slab.polygons, rings);
    }
    Common::publish(nullptr, SG::object_view(object), SG::params(print, object), support, reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
    SG::toolpaths(print);
    Published result;
    result.layers = object.auxiliary_layers().size();
    for (const Layer &layer : object.auxiliary_layers()) {
        std::vector<const LayerSliceIsland *> islands;
        for (const LayerSliceIsland &island : layer.islands()) {
            islands.push_back(&island);
            result.areas.push_back(island.get_slice().area());
        }
        result.islands += islands.size();
        for (size_t own = 0; own < islands.size(); ++own)
            for (const LayerRegionIsland &work : islands[own]->regions_islands())
                for (const ExtrusionRole bucket : { LayerRegionIsland::SUPPORT, LayerRegionIsland::SUPPORT_INTERFACE })
                    if (work.has_extrusion(bucket)) {
                        count_island_crossings(work.extrusion(bucket), islands, own, result.leaves, result.crossing);
                        count_zone_spans(work.extrusion(bucket), zones, result.spanning);
                    }
    }
    return result;
}
// Two 2 mm squares, `gap` scaled units apart in x, as two slabs; the zones are the squares.
Published publish_two_squares(coord_t gap)
{
    const coord_t side = scale_i(2.);
    const auto square = [side](coord_t x) {
        return Polygon(Points{ Point(x, coord_t(0)), Point(x + side, coord_t(0)), Point(x + side, side), Point(x, side) });
    };
    return publish_pieces({ Polygons{ square(coord_t(0)) }, Polygons{ square(side + gap) } }, ExPolygons{ ExPolygon(square(coord_t(0))), ExPolygon(square(side + gap)) });
}
// A 10 mm square with a 4 mm square hole (a ring slab), and a square slab in the hole, `gap` scaled units from the edge of the hole on every side.
Published publish_nested(coord_t gap)
{
    const auto square = [](double from, double to, bool clockwise) {
        Points points{ Point(scale_i(from), scale_i(from)), Point(scale_i(to), scale_i(from)), Point(scale_i(to), scale_i(to)), Point(scale_i(from), scale_i(to)) };
        if (clockwise)
            std::reverse(points.begin(), points.end());
        return Polygon(points);
    };
    const coord_t margin = gap;
    Polygon inner(Points{ Point(scale_i(3.) + margin, scale_i(3.) + margin), Point(scale_i(7.) - margin, scale_i(3.) + margin),
                          Point(scale_i(7.) - margin, scale_i(7.) - margin), Point(scale_i(3.) + margin, scale_i(7.) - margin) });
    return publish_pieces({ Polygons{ square(0., 10., false), square(3., 7., true) }, Polygons{ inner } }, ExPolygons{});
}
} // namespace

// Positive control of the crossing detection: a segment with both ends outside an island but passing through it is found, as is a segment that ends
// inside it, and a segment that stays outside is not.
TEST_CASE("Island crossing detection finds a segment that passes through an island", "[plugins][support-tree-family][support-island]")
{
    const coord_t side = scale_i(2.), mm = scale_i(1.);
    const ExPolygon island(Polygon(Points{ Point(coord_t(0), coord_t(0)), Point(side, coord_t(0)), Point(side, side), Point(coord_t(0), side) }));
    CHECK(segment_meets_island(Point(-mm, mm), Point(3 * mm, mm), island));
    CHECK(!island.contains(Point(-mm, mm)));
    CHECK(!island.contains(Point(3 * mm, mm)));
    CHECK(segment_meets_island(Point(-mm, mm), Point(mm, mm), island));
    // Through two opposite vertices of the square: no proper crossing of its edges, yet the segment passes through the island.
    CHECK(segment_meets_island(Point(-mm, -mm), Point(3 * mm, 3 * mm), island));
    CHECK(!segment_meets_island(Point(-mm, 3 * mm), Point(3 * mm, 3 * mm), island));
    CHECK(!segment_meets_island(Point(-2 * mm, -mm), Point(-mm, 3 * mm), island));
}

// Two joined pieces and a distant one in one layer: the distant piece is published point by point as the plain union makes it (in the raw slice
// of region 0, before the host simplifies the islands) although the island list is rebuilt. A union output never keeps a vertex within
// SCALED_EPSILON of another (Clipper merges them), so the near-closing vertex that a Polygon move would drop cannot reach the rebuild: the
// distant piece is a notched ring, so a rebuilt, rotated or simplified contour would show.
TEST_CASE("Joined support pieces leave a distant piece identical to the plain union", "[plugins][support-tree-family][support-island]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    Test::init_print({ SG::slab() }, print, model, SG::config("tree", false, 0));
    SG::prefix(print);
    PrintObject &object = print.object(0);
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    Common::GeneratedSupport support{ handle };
    const coord_t top = scale_to_layer_coord(1.0), height = scale_to_layer_coord(0.2), side = scale_i(2.);
    const auto square = [side](coord_t x) {
        return Polygon(Points{ Point(x, coord_t(0)), Point(x + side, coord_t(0)), Point(x + side, side), Point(x, side) });
    };
    // The distant ring is a notched square with an off-axis start.
    const coord_t far_x = scale_i(20.);
    Polygon far_ring;
    far_ring.points = Points{ Point(far_x, coord_t(0)), Point(far_x + side, coord_t(0)), Point(far_x + side, side / 2), Point(far_x + side / 2, side / 2),
                              Point(far_x + side / 2, side), Point(far_x, side) };
    const std::vector<Polygons> rings{ Polygons{ square(coord_t(0)) }, Polygons{ square(side + 50) } };
    slic3r_api::StoredPolygonCollection all(handle);
    for (const Polygons &ring : rings) {
        Common::GeneratorLayer &slab = Common::add_layer(support, Common::LayerType::Intermediate, top, top - height, height);
        support.base.push_back(&slab);
        assign_native(slab.polygons, ring);
        all.append_copy_from(slab.polygons);
    }
    // The distant ring is injected by the exact multipoint copy that the island ring filter uses (assign_native copies, then moves into the
    // collection).
    Common::GeneratorLayer &far_slab = Common::add_layer(support, Common::LayerType::Intermediate, top, top - height, height);
    support.base.push_back(&far_slab);
    polygons_resize(far_slab.polygons.mutable_handle(), 1);
    multipoint_copy(polygon_as_multipoint(polygons_at(far_slab.polygons.mutable_handle(), 0)),
                    polygon_as_multipoint_const(reinterpret_cast<const polygon_handle *>(&far_ring)));
    all.append_copy_from(far_slab.polygons);
    // The ring reached the slab with all its points.
    REQUIRE(support.base.back()->polygons.at(0).points().size() == far_ring.points.size());
    const slic3r_api::ClipperContext clipper(handle);
    const slic3r_api::StoredExPolygonCollection plain = Common::native_expolygons(slic3r_api::clipper_union(clipper(all)));
    const std::vector<c_point> *expected = nullptr;
    std::vector<c_point> expected_points;
    for (const slic3r_api::ExPolygon piece : plain)
        if (piece.contour().points().front().x > scale_i(10.)) {
            expected_points = piece.contour().points();
            expected = &expected_points;
        }
    REQUIRE(expected != nullptr);
    // The union keeps the notch: six vertices.
    REQUIRE(expected->size() == far_ring.points.size());
    Common::publish(nullptr, SG::object_view(object), SG::params(print, object), support, reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
    REQUIRE(object.auxiliary_layers().size() == 1);
    const Layer &layer = object.auxiliary_layers().front();
    REQUIRE(layer.islands().size() == 2);
    // The raw slice of region 0 is what publication copied, before the host's ensure_valid simplifies the contours of the islands.
    const ExPolygons &raw = layer.regions().front().get_raw_slices();
    const Points *published = nullptr;
    for (const ExPolygon &slice : raw)
        if (slice.contour.points.front().x() > scale_i(10.))
            published = &slice.contour.points;
    REQUIRE(published != nullptr);
    REQUIRE(published->size() == expected->size());
    for (size_t idx = 0; idx < expected->size(); ++idx) {
        CHECK((*published)[idx].x() == (*expected)[idx].x);
        CHECK((*published)[idx].y() == (*expected)[idx].y);
    }
}
namespace {
// Publishes one base slab per entry of `slabs` (one height group), then compares the islands that the publication chose, read from the raw slice of region 0
// (before the host simplifies the islands), with the whole-layer reference computed here: the closing of the union of the slabs, by +SCALED_EPSILON then
// -SCALED_EPSILON with mitered joins (limit 3), through native_expolygons(). The two rectangles of the cases are the pieces; the reference joins them into
// the one closed island that contains `probe`, which must be the only island published. `other_probe` lies in the other rectangle, and the closing gives
// `closed_count` islands.
void check_joined_rectangles_published(const std::vector<Polygons> &slabs, c_point probe, c_point other_probe, size_t closed_count)
{
    Print print;
    Model model;
    Test::init_print({ SG::slab() }, print, model, SG::config("tree", false, 0));
    SG::prefix(print);
    PrintObject &object = print.object(0);
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    Common::GeneratedSupport support{ handle };
    const coord_t top = scale_to_layer_coord(1.0), height = scale_to_layer_coord(0.2);
    slic3r_api::StoredPolygonCollection all(handle);
    for (const Polygons &rings : slabs) {
        Common::GeneratorLayer &slab = Common::add_layer(support, Common::LayerType::Intermediate, top, top - height, height);
        support.base.push_back(&slab);
        assign_native(slab.polygons, rings);
        all.append_copy_from(slab.polygons);
    }
    const slic3r_api::ClipperContext clipper(handle);
    const slic3r_api::ClipperOperand united = slic3r_api::clipper_union(clipper(all));
    const slic3r_api::StoredExPolygonCollection closed = Common::native_expolygons(slic3r_api::clipper_offset2(
        united, double(SCALED_EPSILON), -double(SCALED_EPSILON), CLIPPER_JOIN_MITER, 3.));
    // The case needs the three areas in the union, and the sliver (or the U) dropped by the filter of the plain union, which leaves the two rectangles as
    // the only pieces.
    REQUIRE(united.to_expolygon_collection().size() == 3);
    const slic3r_api::StoredExPolygonCollection plain = Common::native_expolygons(united);
    REQUIRE(plain.size() == 2);
    CHECK(((plain.at(0).contains(probe) && plain.at(1).contains(other_probe)) || (plain.at(1).contains(probe) && plain.at(0).contains(other_probe))));
    REQUIRE(closed.size() == closed_count);
    Common::publish(nullptr, SG::object_view(object), SG::params(print, object), support, reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
    REQUIRE(object.auxiliary_layers().size() == 1);
    const ExPolygons &published = object.auxiliary_layers().front().regions().front().get_raw_slices();
    // The reference island that holds the rectangles.
    const slic3r_api::ExPolygon *joined = nullptr;
    std::vector<slic3r_api::ExPolygon> closed_islands;
    for (const slic3r_api::ExPolygon island : closed)
        closed_islands.push_back(island);
    for (const slic3r_api::ExPolygon &island : closed_islands)
        if (island.contains(probe))
            joined = &island;
    REQUIRE(joined != nullptr);
    REQUIRE(published.size() == 1);
    const std::vector<c_point> expected = joined->contour().points();
    REQUIRE(published.front().contour.points.size() == expected.size());
    for (size_t idx = 0; idx < expected.size(); ++idx) {
        CHECK(published.front().contour.points[idx].x() == expected[idx].x);
        CHECK(published.front().contour.points[idx].y() == expected[idx].y);
    }
    CHECK(published.front().holes.size() == joined->hole_size());
}
} // namespace

// Two rectangles 450 units apart with a thin triangle between them, whose area is under the small-area filter of the plain union but which the closing
// still sees: it bridges the gaps (125 units on each side), so the reference publishes the rectangles as one joined island.
TEST_CASE("A dropped sliver between two support pieces still joins them", "[plugins][support-tree-family][support-island]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const auto rectangle = [](coord_t x0, coord_t x1, coord_t y1) {
        return Polygon(Points{ Point(x0, coord_t(0)), Point(x1, coord_t(0)), Point(x1, y1), Point(x0, y1) });
    };
    const Polygon triangle(Points{ Point(coord_t(125), coord_t(0)), Point(coord_t(325), coord_t(0)), Point(coord_t(225), coord_t(10000)) });
    check_joined_rectangles_published({ Polygons{ rectangle(-1000, 0, 10000) }, Polygons{ rectangle(450, 1450, 10000) }, Polygons{ triangle } },
                                      c_point{ -500, 5000 }, c_point{ 950, 5000 }, 1);
}

// Two rectangles 100 units apart and a far thin U whose ring is dropped by the filter of the plain union but whose closing survives as an island of its
// own, without piece. The count of closed islands equals the count of pieces, which must not hide the join of the rectangles.
TEST_CASE("A closed island without piece does not hide a join", "[plugins][support-tree-family][support-island]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const auto rectangle = [](coord_t x0, coord_t x1, coord_t y1) {
        return Polygon(Points{ Point(x0, coord_t(0)), Point(x1, coord_t(0)), Point(x1, y1), Point(x0, y1) });
    };
    const Polygon u_shape(Points{ Point(coord_t(10000), coord_t(0)), Point(coord_t(10440), coord_t(0)), Point(coord_t(10440), coord_t(10000)),
                                  Point(coord_t(10290), coord_t(200)), Point(coord_t(10150), coord_t(200)), Point(coord_t(10000), coord_t(10000)) });
    check_joined_rectangles_published({ Polygons{ rectangle(-1000, 0, 10000) }, Polygons{ rectangle(100, 1100, 10000) }, Polygons{ u_shape } },
                                      c_point{ -500, 5000 }, c_point{ 600, 5000 }, 2);
}

// The islands of a support layer are what the legacy toolpaths unite (the sparse surfaces are closed by +SCALED_EPSILON then -SCALED_EPSILON): two
// slabs of one height group 50 units apart publish one island, and the support paths cross no other island. The united fill makes paths that
// run over both squares: they would have crossed two islands.
TEST_CASE("Support slabs closer than the closing publish one island", "[plugins][support-tree-family][support-island]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const Published close = publish_two_squares(50);
    CHECK(close.layers == 1);
    CHECK(close.islands == 1);
    CHECK(close.leaves > 0);
    REQUIRE(close.spanning > 0);
    CHECK(close.crossing == 0);
}

// A ring with a hole and a square 50 units from the edge of the hole: the closing fills the gap, one island. A square 1 mm from it stays its own
// island, listed after the ring (outside to inside, which the host asserts in Debug), and no path crosses the other island.
TEST_CASE("Support slab nested in the hole of another publishes joined or separate islands", "[plugins][support-tree-family][support-island]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const Published joined = publish_nested(50);
    CHECK(joined.layers == 1);
    CHECK(joined.islands == 1);
    CHECK(joined.leaves > 0);
    CHECK(joined.crossing == 0);
    const Published apart = publish_nested(scale_i(1.));
    CHECK(apart.layers == 1);
    REQUIRE(apart.islands == 2);
    CHECK(apart.areas[0] > apart.areas[1]);
    CHECK(apart.leaves > 0);
    CHECK(apart.crossing == 0);
}
// A slab with no other piece near it publishes exactly the plain union of its rings (same points in the same order), so the joining of pieces
// changes the geometry only where pieces are joined.
TEST_CASE("An isolated support slab publishes the plain union geometry", "[plugins][support-tree-family][support-island]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    Test::init_print({ SG::slab() }, print, model, SG::config("tree", false, 0));
    SG::prefix(print);
    PrintObject &object = print.object(0);
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    Common::GeneratedSupport support{ handle };
    const coord_t top = scale_to_layer_coord(1.0), height = scale_to_layer_coord(0.2);
    Common::GeneratorLayer &slab = Common::add_layer(support, Common::LayerType::Intermediate, top, top - height, height);
    support.base = { &slab };
    // A square with an off-axis start and a notch, so that a ring that is rebuilt or rotated would show.
    const Points ring{ Point(scale_i(1.), scale_i(1.)), Point(scale_i(3.), scale_i(1.)), Point(scale_i(3.), scale_i(2.)), Point(scale_i(2.5), scale_i(2.)),
                       Point(scale_i(2.5), scale_i(3.)), Point(scale_i(1.), scale_i(3.)) };
    assign_native(slab.polygons, Polygons{ Polygon(ring) });
    const slic3r_api::ClipperContext clipper(handle);
    const slic3r_api::StoredExPolygonCollection plain = Common::native_expolygons(slic3r_api::clipper_union(clipper(slab.polygons)));
    REQUIRE(plain.size() == 1);
    Common::publish(nullptr, SG::object_view(object), SG::params(print, object), support, reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
    REQUIRE(object.auxiliary_layers().size() == 1);
    const Layer &layer = object.auxiliary_layers().front();
    REQUIRE(layer.islands().size() == 1);
    const Points &published = layer.islands().front().get_slice().contour.points;
    const std::vector<c_point> expected = plain.at(0).contour().points();
    REQUIRE(published.size() == expected.size());
    for (size_t idx = 0; idx < expected.size(); ++idx) {
        CHECK(published[idx].x() == expected[idx].x);
        CHECK(published[idx].y() == expected[idx].y);
    }
}

// Pieces farther apart than the closing stay two islands, each with its own paths.
TEST_CASE("Support slabs 1 mm apart publish two islands", "[plugins][support-tree-family][support-island]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const Published far = publish_two_squares(scale_i(1.));
    CHECK(far.layers == 1);
    CHECK(far.islands == 2);
    CHECK(far.leaves > 0);
    CHECK(far.crossing == 0);
}

namespace {
const SurfaceType roof_rest_top_type = stModSupport | stPosTop | stDensSolid;
const SurfaceType roof_rest_floor_type = stModSupport | stPosBottom | stDensSolid | stModBridge;

// A block of a roof scene: x, y, width, depth, z and height in mm, in the frame where the roof footprint is 12 x 8 mm from the origin.
struct RoofBlock {
    double x, y, width, depth, z, height;
};

// A post (4 x 8 mm from the origin) with a roof (12 x 8 x 1 mm) at roof_z that overhangs the post by 8 mm on one side, and the given blocks.
TriangleMesh roof_scene(double roof_z, const std::vector<RoofBlock> &blocks)
{
    TriangleMesh mesh = make_cube(4., 8., roof_z), roof = make_cube(12., 8., 1.);
    roof.translate(0., 0., roof_z);
    mesh.merge(roof);
    for (const RoofBlock &block : blocks) {
        TriangleMesh part = make_cube(block.width, block.depth, block.height);
        part.translate(block.x, block.y, block.z);
        mesh.merge(part);
    }
    return mesh;
}

// The roof scene over a platform from the bed to platform_z, of the given width in x (centred on platform_centre_x) and depth in y (centred under the
// roof). A platform of 12 x 8 mm centred on 6 mm is the footprint of the roof.
TriangleMesh roof_over_platform(double roof_z, double platform_z, double platform_width, double platform_depth, double platform_centre_x = 6.)
{
    return roof_scene(roof_z, { { platform_centre_x - platform_width / 2., 4. - platform_depth / 2., platform_width, platform_depth, 0., platform_z } });
}

// The model preparation of the roof tests: a print of a roof scene sliced up to the support demand, with the bottom contact distance (mm), the
// interface layers and the print options given. The roof footprint is the one of the top layer.
struct RoofRest {
    Print print;
    Model model;
    RoofRest(const char *style, TriangleMesh mesh, const char *bottom_distance, int interfaces,
             const std::vector<std::pair<std::string, std::string>> &print_options = {})
    {
        DynamicPrintConfig config = close_z_gcode_config(style, interfaces);
        for (const auto &option : print_options)
            config.set_deserialize_strict({ {option.first, option.second} });
        close_z_place({ { std::move(mesh), Vec3d(100., 100., 0.), { {"support_material_bottom_contact_distance", bottom_distance} } } }, config, print, model);
        SG::prefix(print);
    }
    RoofRest(const char *style, double roof_z, double platform_z, const char *bottom_distance, int interfaces, double platform_width = 12.,
             double platform_depth = 8., double platform_centre_x = 6.)
        : RoofRest(style, roof_over_platform(roof_z, platform_z, platform_width, platform_depth, platform_centre_x), bottom_distance, interfaces) {}
    BoundingBox roof_extents() const
    {
        const PrintObject &object = print.object(0);
        return get_extents(object.layer(object.layer_count() - 1).lslices());
    }
    // A point under the roof at x mm from its left edge (the post is the first 4 mm), in the middle in y.
    Point under_roof_at(double x_mm) const
    {
        const BoundingBox extents = roof_extents();
        return Point(extents.min.x() + scale_to_layer_coord(x_mm), extents.center().y());
    }
    // A point under the part of the roof that overhangs the post.
    Point under_overhang() const { return under_roof_at(9.); }
};

// What the published auxiliary layers of the first object hold under the overhang of the roof.
struct RoofRestPublished {
    // Layers (bottom, top) whose top surfaces (the contact and the interfaces) contain the point under the overhang.
    std::vector<std::pair<coord_t, coord_t>> top_layers;
    // Layers (bottom, top) with a branch or sparse surface that lies entirely beyond the post.
    std::vector<std::pair<coord_t, coord_t>> branch_layers;
};

RoofRestPublished roof_rest_published(const RoofRest &fixture, bool organic)
{
    const SurfaceType branch_type = stModSupport | stPosInternal | stDensSparse;
    const BoundingBox extents = fixture.roof_extents();
    const Point point = fixture.under_overhang();
    const coord_t beyond_post = extents.min.x() + scale_to_layer_coord(4.);
    RoofRestPublished published;
    for (const Layer &layer : fixture.print.object(0).auxiliary_layers()) {
        bool top = false, branch = false;
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands())
                for (const Surface &surface : region.fill_surfaces()) {
                    top = top || ((surface.surface_type & ~stModContact) == roof_rest_top_type && surface.expolygon.contains(point));
                    branch = branch || (surface.surface_type == branch_type && get_extents(surface.expolygon).min.x() >= beyond_post);
                }
        if (top)
            published.top_layers.push_back({ layer.scaled_bottom_z(), layer.scaled_print_z() });
        if (branch)
            published.branch_layers.push_back({ layer.scaled_bottom_z(), layer.scaled_print_z() });
    }
    return published;
}

// The published surfaces of a type in the auxiliary layers of the first object that span [bottom, top], united.
ExPolygons roof_rest_surfaces(const Print &print, SurfaceType type, coord_t bottom, coord_t top)
{
    ExPolygons surfaces;
    for (const Layer &layer : print.object(0).auxiliary_layers())
        if (layer.scaled_bottom_z() == bottom && layer.scaled_print_z() == top)
            for (const LayerSliceIsland &island : layer.islands())
                for (const LayerRegionIsland &region_island : island.regions_islands())
                    for (const Surface &surface : region_island.fill_surfaces())
                        if ((surface.surface_type & ~stModContact) == type)
                            surfaces.push_back(surface.expolygon);
    return union_ex(surfaces);
}

// The area (mm2) of the published surfaces of a type spanning [bottom, top], within a region.
double roof_rest_area(const Print &print, SurfaceType type, coord_t bottom, coord_t top, const ExPolygons &region)
{
    return unscaled(unscaled(Slic3r::area(intersection_ex(roof_rest_surfaces(print, type, bottom, top), region))));
}

// Whether a published surface of a type spanning [bottom, top] contains the point.
bool roof_rest_covers(const Print &print, SurfaceType type, coord_t bottom, coord_t top, const Point &point)
{
    for (const ExPolygon &surface : roof_rest_surfaces(print, type, bottom, top))
        if (surface.contains(point))
            return true;
    return false;
}
} // namespace

// Branches that carry the interface stack rest on the model whatever the height of their tip above it (they are exempt from
// min_height_to_model, like the roof branches of the legacy tree). The roof is at 2.8 mm over a platform of 0.8 mm, the bottom contact distance
// is 0.25 mm and the interfaces are 3 layers of 0.3 mm: the contact is published at 2.6 mm (gap 0.2 mm), the stack under it from 1.7 mm, the tips
// start on the object layer at 1.6 mm, and the first layer that may rest on the platform (0.8 + 0.25 mm) is the one at 1.2 mm. The tips are 0.4 mm
// above it, less than the minimum of 1 mm. The support must still be routed, kept, and published under the roof: the contact and the interfaces
// over the overhang, and the branch bodies between the platform and the stack, which rest on the platform. The platform is the footprint of the
// roof (12 x 8 mm) or 40 x 40 mm; the second variant is the control where no branch has a way down to the bed past the platform edge.
TEST_CASE("Tree branches that carry the interface stack rest on the model under a low tip", "[plugins][support-tree-family][support-tree-roof-rest]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *style = GENERATE("tree", "organic");
    const auto [platform_width, platform_depth] = GENERATE(table<double, double>({ { 12., 8. }, { 40., 40. } }));
    CAPTURE(style, platform_width, platform_depth);
    RoofRest fixture(style, 2.8, 0.8, "0.25", 3, platform_width, platform_depth);
    // The key heights: the tips are less than the minimum height above the first layer that may rest on the platform.
    const PrintObject &object = fixture.print.object(0);
    const coord_t stack_bottom = scale_to_layer_coord(1.7), first_rest = scale_to_layer_coord(0.8 + 0.25);
    coord_t tip_layer = 0, rest_layer = 0;
    for (size_t id = 0; id < object.layer_count(); ++id) {
        if (object.layer(id).scaled_print_z() <= stack_bottom)
            tip_layer = object.layer(id).scaled_print_z();
        if (rest_layer == 0 && object.layer(id).scaled_print_z() >= first_rest)
            rest_layer = object.layer(id).scaled_print_z();
    }
    CHECK(tip_layer == scale_to_layer_coord(1.6));
    CHECK(rest_layer == scale_to_layer_coord(1.2));
    CHECK(tip_layer - rest_layer < scale_to_layer_coord(1.));
    {
        const Slic3r::Test::SupportTree::Routed routed(fixture.print);
        CHECK(routed.trees.size() > 0);
        CHECK(routed.support.top.size() > 0);
    }
    SG::family(fixture.print);
    const RoofRestPublished published = roof_rest_published(fixture, std::string(style) == "organic");
    // The contact (top at 2.6 mm) and the two interfaces under it cover the overhang; the stack starts at 1.7 mm.
    REQUIRE(published.top_layers.size() == 3);
    std::vector<std::pair<coord_t, coord_t>> top_layers = published.top_layers;
    std::sort(top_layers.begin(), top_layers.end(), [](const auto &a, const auto &b) { return a.second < b.second; });
    CHECK(top_layers[0] == std::make_pair(stack_bottom, scale_to_layer_coord(2.0)));
    CHECK(top_layers[1] == std::make_pair(scale_to_layer_coord(2.0), scale_to_layer_coord(2.3)));
    CHECK(top_layers[2] == std::make_pair(scale_to_layer_coord(2.3), scale_to_layer_coord(2.6)));
    // A branch body under the stack, over the platform and beyond the post: it rests on the platform.
    coord_t lowest_bottom = std::numeric_limits<coord_t>::max();
    for (const auto &layer : published.branch_layers)
        if (layer.second <= stack_bottom)
            lowest_bottom = std::min(lowest_bottom, layer.first);
    REQUIRE(lowest_bottom != std::numeric_limits<coord_t>::max());
    CHECK(lowest_bottom > scale_to_layer_coord(0.8));
    CHECK(lowest_bottom <= rest_layer);
}

// Without interface layers a branch does not carry a roof, so the rule of min_height_to_model holds: a roof at 2.0 mm over a platform of 0.8 mm
// puts the tips (on the layer at 1.8 mm) 0.8 mm above the first layer that may rest on the platform (1.0 mm), and they are dropped, with their
// contact. The platform is 40 mm wide: a branch cannot slant down to the bed past its edge (14 mm away, which needs more than 16 mm of height at
// the largest branch angle of 40 degrees, and the tips are less than 2 mm high).
TEST_CASE("Tree branches without interface layers keep the minimum height to the model", "[plugins][support-tree-family][support-tree-roof-rest]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *style = GENERATE("tree", "organic");
    CAPTURE(style);
    RoofRest fixture(style, 2.0, 0.8, "0.2", 0, 40., 40.);
    const Slic3r::Test::SupportTree::Routed routed(fixture.print);
    CHECK(routed.trees.empty());
    CHECK(routed.support.top.empty());
    // Control: a roof 1.2 mm higher leaves the tips 2.0 mm above the platform, and they are kept.
    RoofRest high(style, 3.2, 0.8, "0.2", 0, 40., 40.);
    const Slic3r::Test::SupportTree::Routed routed_high(high.print);
    CHECK(routed_high.trees.size() > 0);
    CHECK(routed_high.support.top.size() > 0);
}

// The reported case: a roof at 2.0 mm over a platform of 0.8 mm, a bottom contact distance of 0.2 mm and 3 interface layers of 0.3 mm. The
// contact is at 1.8 mm and the nominal stack under it would start at 0.9 mm, on the platform layer itself, so no tip can start there. The part of
// the contact that lies over the object on the tip layer needs no tip: the stack fills the interval down to the object, as the grid does. The
// floor is the bottom contact on the platform ([1.0, 1.2], the platform top 0.8 + the gap 0.2), and the stack the interface [1.2, 1.5] and the
// contact [1.5, 1.8] (the interval of 0.6 mm holds two dense slabs of 0.3 mm). No branch is drawn under the part.
// Inside the raw slices of the platform, the tree matches the grid within 1 percent: both are the overhang of the roof over the platform. The floor
// of the tree also has a rim outside the platform, on the top surfaces grown by the XY distance (the rule of the floors of the roots): it lies in the
// band of the XY distance around the outline of the platform, and nowhere else.
TEST_CASE("Tree roof 2 mm over a platform with 3 interface layers rests its stack on the platform", "[plugins][support-tree-family][support-tree-roof-rest]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *style = GENERATE("tree", "organic");
    CAPTURE(style);
    const coord_t floor_bottom = scale_to_layer_coord(1.0), floor_top = scale_to_layer_coord(1.2), middle = scale_to_layer_coord(1.5), roof_top = scale_to_layer_coord(1.8);
    RoofRest tree(style, 2.0, 0.8, "0.2", 3);
    RoofRest grid("grid", 2.0, 0.8, "0.2", 3);
    {
        const Slic3r::Test::SupportTree::Routed routed(tree.print);
        CHECK(routed.trees.empty());
        CHECK(routed.support.top.size() > 0);
        CHECK(routed.resting.size() == 1);
    }
    const coord_t gap_xy = SG::params(tree.print, tree.print.object(0)).gap_xy;
    SG::family(tree.print);
    SG::family(grid.print);
    const ExPolygons platform_slices = tree.print.object(0).layer(0).lslices();
    const struct { const char *name; SurfaceType type; coord_t bottom, top; } parts[] = {
        { "floor", roof_rest_floor_type, floor_bottom, floor_top }, { "interface", roof_rest_top_type, floor_top, middle }, { "contact", roof_rest_top_type, middle, roof_top } };
    for (const auto &part : parts) {
        const double tree_area = roof_rest_area(tree.print, part.type, part.bottom, part.top, platform_slices);
        const double grid_area = roof_rest_area(grid.print, part.type, part.bottom, part.top, platform_slices);
        CAPTURE(part.name, tree_area, grid_area);
        CHECK(tree_area > 0.);
        CHECK(grid_area > 0.);
        CHECK(tree_area == Approx(grid_area).epsilon(0.01));
    }
    // The rim of the floor outside the platform: within the band of the XY distance around its outline.
    const ExPolygons rim = diff_ex(roof_rest_surfaces(tree.print, roof_rest_floor_type, floor_bottom, floor_top), platform_slices);
    const ExPolygons band = offset_ex(platform_slices, float(gap_xy + SCALED_EPSILON));
    CHECK(unscaled(unscaled(Slic3r::area(diff_ex(rim, band)))) < 1e-4);
    CHECK(unscaled(unscaled(Slic3r::area(rim))) > 0.);
    // The same z ranges cover the point under the overhang, and no branch is published there.
    const Point point = tree.under_overhang();
    CHECK(roof_rest_covers(tree.print, roof_rest_floor_type, floor_bottom, floor_top, point) == roof_rest_covers(grid.print, roof_rest_floor_type, floor_bottom, floor_top, point));
    CHECK(roof_rest_covers(tree.print, roof_rest_top_type, floor_top, middle, point));
    CHECK(roof_rest_covers(tree.print, roof_rest_top_type, middle, roof_top, point));
    CHECK(roof_rest_published(tree, std::string(style) == "organic").branch_layers.empty());
}

// A roof half over the platform and half over the bed: the platform is under the right 4 mm of the roof (x from 8 to 12 mm), the post is under the
// first 4 mm. The part over the platform has its floor and its stack on it; the rest routes as before, with branches down to the bed.
TEST_CASE("Tree roof half over a platform and half over the bed", "[plugins][support-tree-family][support-tree-roof-rest]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *style = GENERATE("tree", "organic");
    CAPTURE(style);
    RoofRest fixture(style, 2.0, 0.8, "0.2", 3, 4., 8., 10.);
    const coord_t floor_bottom = scale_to_layer_coord(1.0), floor_top = scale_to_layer_coord(1.2);
    {
        const Slic3r::Test::SupportTree::Routed routed(fixture.print);
        CHECK(routed.trees.size() > 0);
        coord_t lowest = std::numeric_limits<coord_t>::max();
        for (const TF::RoutedTree &tree : routed.trees)
            for (const TF::TreeNode &node : tree.nodes)
                lowest = std::min<coord_t>(lowest, coord_t(node.layer));
        CHECK(lowest == 0);
    }
    SG::family(fixture.print);
    // Over the platform (x = 10 mm): the floor on it, and the stack. Over the bed (x = 6 mm): no floor, and the contact still covers it.
    const Point on_platform = fixture.under_roof_at(10.), on_bed = fixture.under_roof_at(6.);
    CHECK(roof_rest_covers(fixture.print, roof_rest_floor_type, floor_bottom, floor_top, on_platform));
    CHECK_FALSE(roof_rest_covers(fixture.print, roof_rest_floor_type, floor_bottom, floor_top, on_bed));
    CHECK(roof_rest_covers(fixture.print, roof_rest_top_type, scale_to_layer_coord(1.5), scale_to_layer_coord(1.8), on_platform));
    CHECK(roof_rest_covers(fixture.print, roof_rest_top_type, scale_to_layer_coord(1.5), scale_to_layer_coord(1.8), on_bed));
    // Branches reach the bed under the part that is not over the platform.
    coord_t lowest_bottom = std::numeric_limits<coord_t>::max();
    for (const auto &layer : roof_rest_published(fixture, std::string(style) == "organic").branch_layers)
        lowest_bottom = std::min(lowest_bottom, layer.first);
    CHECK(lowest_bottom <= scale_to_layer_coord(0.2));
}

// Two superposed surfaces: a shelf (0.4 mm high) under a platform (from 1.0 to 1.2 mm, held by the post and by a pillar at the other end), under
// the roof at 2.4 mm. The cavity between them is 0.6 mm high. The platform has a support of its own (its underside, whose floor stands on the shelf),
// and the roof has its resting part on the platform (dont_support_bridges narrows the underside part to the strips that are not bridged). The floors of the roof stop at the first support, the platform: the floor of the shelf is the one
// of the underside of the platform alone, which is narrower in y than the roof part (about 5.6 mm against 8 mm), so it does not cover the point
// at 3.6 mm from the middle of the roof in y, while the floor of the platform does. The check is made on the floors generated for the slabs (the
// published ones are widened by the bottom expansion). The floor of the platform is [1.4, 1.6] (top 1.2 + gap 0.2) and the shelf [0.6, 0.8] (top
// 0.4 + gap 0.2).
// The platform underside keeps its demand because the bridge infill prints diagonally at the default fill angle, so the bridge infill removal
// does not apply there (only the perimeters narrow it). Once bridges print in their detected direction, this test will need a geometry whose
// underside is not a two-sided bridge.
TEST_CASE("Tree resting part floors stop at the first support", "[plugins][support-tree-family][support-tree-roof-rest]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool organic = GENERATE(false, true);
    CAPTURE(organic);
    RoofRest fixture(organic ? "organic" : "tree", roof_scene(2.4, { { 0., 0., 12., 8., 0., 0.4 }, { 0., 0., 12., 8., 1.0, 0.2 }, { 11., 0., 1., 8., 0., 1.0 } }), "0.2", 3,
                       { {"dont_support_bridges", "1"} });
    const Point wide = fixture.under_roof_at(9.) + Point(coord_t(0), scale_to_layer_coord(3.6));
    {
        Slic3r::Test::SupportTree::Routed routed(fixture.print);
        CHECK(routed.trees.empty());
        REQUIRE(routed.resting.size() == 2);
        TF::generate_tree_slabs(nullptr, SG::print_view(fixture.print), SG::object_view(fixture.print.object(0)), routed.params, routed.settings, routed.trees, organic, routed.support, routed.resting);
        const Common::GeneratorLayer *shelf = nullptr, *platform = nullptr;
        for (const Common::GeneratorLayer *floor : routed.support.bottom) {
            if (floor->print_z == scale_to_layer_coord(0.8))
                shelf = floor;
            if (floor->print_z == scale_to_layer_coord(1.6))
                platform = floor;
        }
        REQUIRE(shelf != nullptr);
        REQUIRE(platform != nullptr);
        const auto covers = [](const Common::GeneratorLayer &floor, const Point &point) {
            for (const ExPolygon &expolygon : union_ex(to_native(floor.polygons)))
                if (expolygon.contains(point))
                    return true;
            return false;
        };
        CHECK(covers(*platform, wide));
        CHECK_FALSE(covers(*shelf, wide));
    }
    SG::family(fixture.print);
    // The roof stack rests on the platform, and the shelf has the floor of the underside of the platform.
    const Point near_post = fixture.under_roof_at(6.);
    CHECK(roof_rest_covers(fixture.print, roof_rest_floor_type, scale_to_layer_coord(0.6), scale_to_layer_coord(0.8), near_post));
    CHECK(roof_rest_covers(fixture.print, roof_rest_floor_type, scale_to_layer_coord(1.4), scale_to_layer_coord(1.6), near_post));
    CHECK(roof_rest_covers(fixture.print, roof_rest_top_type, scale_to_layer_coord(1.6), scale_to_layer_coord(1.9), near_post));
    CHECK(roof_rest_covers(fixture.print, roof_rest_top_type, scale_to_layer_coord(1.9), scale_to_layer_coord(2.2), near_post));
}
// With a support restricted to the build plate, the resting rule is off: the demand over the object is removed by the demand plugin before the
// routing, so the roof over the platform has no contact at all, nothing is routed or published, and no resting part exists.
TEST_CASE("Tree resting part is off for a build plate only support", "[plugins][support-tree-family][support-tree-roof-rest]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *style = GENERATE("tree", "organic");
    CAPTURE(style);
    RoofRest fixture(style, roof_over_platform(2.0, 0.8, 12., 8.), "0.2", 3, { {"support_material_buildplate_only", "1"} });
    {
        const Slic3r::Test::SupportTree::Routed routed(fixture.print);
        CHECK(routed.trees.empty());
        CHECK(routed.support.top.empty());
        CHECK(routed.resting.empty());
    }
    SG::family(fixture.print);
    CHECK(fixture.print.object(0).auxiliary_layers().empty());
}

// A soluble support (contact distance type none, distances 0): the contact fills the object layer under the roof ([1.8, 2.0]) and the floor lies on
// the platform itself ([0.8, 1.0], no gap), under a bottom interface. The configuration is valid for the low roof; the part over the platform rests
// there as well, with no branch, and its stack is the contact, two interfaces and the bottom interface.
TEST_CASE("Tree soluble support rests its stack on the platform", "[plugins][support-tree-family][support-tree-roof-rest]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *style = GENERATE("tree", "organic");
    CAPTURE(style);
    RoofRest fixture(style, roof_over_platform(2.0, 0.8, 12., 8.), "0.2", 3, { {"support_material_contact_distance_type", "none"}, {"support_material_contact_distance", "0"} });
    {
        const Slic3r::Test::SupportTree::Routed routed(fixture.print);
        CHECK(routed.trees.empty());
        CHECK(routed.resting.size() == 1);
    }
    SG::family(fixture.print);
    const SurfaceType soluble_floor_type = stModSupport | stPosBottom | stDensSolid;
    CHECK(roof_rest_covers(fixture.print, soluble_floor_type, scale_to_layer_coord(0.8), scale_to_layer_coord(1.0), fixture.under_overhang()));
    const RoofRestPublished published = roof_rest_published(fixture, std::string(style) == "organic");
    REQUIRE(published.top_layers.size() == 3);
    coord_t lowest = std::numeric_limits<coord_t>::max(), highest = 0;
    for (const auto &layer : published.top_layers) {
        lowest = std::min(lowest, layer.first);
        highest = std::max(highest, layer.second);
    }
    CHECK(highest == scale_to_layer_coord(2.0));
    CHECK(lowest > scale_to_layer_coord(1.2));
    CHECK(published.branch_layers.empty());
}

// A contact split by the resting part: two platforms under the roof (x from 4.5 to 6.0 mm and from 6.6 to 9.0 mm) leave a slot of 0.6 mm between
// them over the bed and a component beyond them (x from 9 to 12 mm). The slot is too narrow for a tip, so its component is trimmed alone; the far
// component has tips and branches down to the bed; the platforms have their floors and stacks. (Before the components were split, the whole remainder
// was one tip source, and the slot stayed in the contact because of the tips of the far component.)
TEST_CASE("Tree contact split by the resting part trims the component without a tip", "[plugins][support-tree-family][support-tree-roof-rest]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *style = GENERATE("tree", "organic");
    CAPTURE(style);
    RoofRest fixture(style, roof_scene(2.0, { { 4.5, 0., 1.5, 8., 0., 0.8 }, { 6.6, 0., 2.4, 8., 0., 0.8 } }), "0.2", 3);
    {
        const Slic3r::Test::SupportTree::Routed routed(fixture.print);
        CHECK(routed.trees.size() > 0);
        CHECK(routed.resting.size() == 1);
    }
    SG::family(fixture.print);
    const Point first = fixture.under_roof_at(5.2), slot = fixture.under_roof_at(6.3), second = fixture.under_roof_at(7.8), far = fixture.under_roof_at(10.5);
    const coord_t floor_bottom = scale_to_layer_coord(1.0), floor_top = scale_to_layer_coord(1.2), middle = scale_to_layer_coord(1.5), roof_top = scale_to_layer_coord(1.8);
    for (const Point &platform : { first, second }) {
        CHECK(roof_rest_covers(fixture.print, roof_rest_floor_type, floor_bottom, floor_top, platform));
        CHECK(roof_rest_covers(fixture.print, roof_rest_top_type, floor_top, middle, platform));
        CHECK(roof_rest_covers(fixture.print, roof_rest_top_type, middle, roof_top, platform));
    }
    CHECK(roof_rest_covers(fixture.print, roof_rest_top_type, middle, roof_top, far));
    CHECK_FALSE(roof_rest_covers(fixture.print, roof_rest_top_type, middle, roof_top, slot));
    CHECK_FALSE(roof_rest_covers(fixture.print, roof_rest_top_type, floor_top, middle, slot));
    CHECK_FALSE(roof_rest_covers(fixture.print, roof_rest_floor_type, floor_bottom, floor_top, far));
    coord_t lowest_bottom = std::numeric_limits<coord_t>::max();
    for (const auto &layer : roof_rest_published(fixture, std::string(style) == "organic").branch_layers)
        lowest_bottom = std::min(lowest_bottom, layer.first);
    CHECK(lowest_bottom <= scale_to_layer_coord(0.2));
}
