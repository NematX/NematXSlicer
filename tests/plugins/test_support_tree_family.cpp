#include "support_grid_test_helpers.hpp"
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
   comparison tests cover. */

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
    TreeSlabs(const Print &print, bool organic)
    {
        TF::generate_tree_support(nullptr, print, print.object(0), organic, support);
    }
};

// The routed trees of the first object of a sliced print, with the storage that owns the contacts they support.
struct Routed {
    PluginStorage storage;
    Common::GeneratedSupport support{reinterpret_cast<storage_handle *>(&storage)};
    const Common::SupportParams params;
    const TF::TreeSettings settings;
    std::vector<TF::RoutedTree> trees;
    explicit Routed(const Print &print)
        : params(SG::params(print, print.object(0))), settings(print.object(0), params),
          trees(TF::route_trees(print, print.object(0), params, settings, support)) {}
};

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
            largest_travel_mm = std::max(largest_travel_mm, unscaled((tree.nodes.front().position - tree.nodes.back().position).cast<double>().norm()));
            for (const TF::TreeNode &lower : tree.nodes)
                for (const size_t upper_idx : lower.carries) {
                    const TF::TreeNode &upper = tree.nodes[upper_idx];
                    const double shift_mm = unscaled((upper.position - lower.position).cast<double>().norm());
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
    const TF::TreeSettings settings(print.object(0), SG::params(print, print.object(0)));
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
    const TF::TreeSettings settings(print.object(0), params);
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
    const TF::TreeSettings settings(object, SG::params(print, object));
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
                CHECK(distance_to_region_mm(node.position, SG::native(slab->polygons)) <= tolerance_mm);
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
    const TF::TreeSettings settings(object, SG::params(print, object));
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
    upper.position = centre;
    upper.radius = settings.branch_radius;
    TF::TreeNode root;
    root.layer = root_layer;
    root.print_z = object.layer(root_layer).unscaled_print_z();
    root.position = centre;
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
    const TF::TreeSettings settings(print.object(0), SG::params(print, print.object(0)));
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
    const TF::TreeSettings settings(object, SG::params(print, object));
    const ExPolygons block_slices = object.layer(layer_at_height(object, 4.)).lslices();
    const ExPolygon &block_slice = *std::max_element(block_slices.begin(), block_slices.end(),
        [](const ExPolygon &a, const ExPolygon &b) { return a.area() < b.area(); });
    const Point centre = get_extents(block_slice).center();
    const size_t root_layer = layer_at_height(object, 6.);
    TF::TreeNode upper;
    upper.layer = root_layer + 1;
    upper.print_z = object.layer(root_layer + 1).unscaled_print_z();
    upper.position = centre;
    upper.radius = settings.branch_radius;
    TF::TreeNode root;
    root.layer = root_layer;
    root.print_z = object.layer(root_layer).unscaled_print_z();
    root.position = centre;
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
        node.position = Point::new_scale(points[idx].x_mm, points[idx].y_mm);
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
    return Vec3d(unscaled(node.position.x()), unscaled(node.position.y()), node.print_z);
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
            if (node.position != raw.position) {
                ++moved;
                largest_move_mm = std::max(largest_move_mm, unscaled((node.position - raw.position).cast<double>().norm()));
            }
            const double clearance_mm = distance_to_region_mm(node.position, keep_out[node.layer]);
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
        REQUIRE(distance_to_region_mm(node.position, keep_out[node.layer]) > unscaled(node.radius));
    const std::vector<TF::TreeNode> raw = trees.front().nodes;
    const double bend_before = sharpest_bend_deg(trees.front());
    const size_t iterations = TF::smooth_organic_branches(object, keep_out, trees, [] {});
    const double bend_after = sharpest_bend_deg(trees.front());
    size_t moved = 0;
    for (size_t idx = 0; idx < raw.size(); ++idx)
        moved += trees.front().nodes[idx].position != raw[idx].position ? 1 : 0;
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
        const double x_mm = unscaled(trees.front().nodes[1].position.x());
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
        CHECK(trees.front().nodes[idx].position == raw[idx].position);
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
    upper.position = Point::new_scale(0.28, 0.);
    upper.radius = scale_to_layer_coord(0.4);
    TF::TreeNode lower;
    lower.layer = 5;
    lower.print_z = 1.0;
    lower.position = Point::new_scale(0., 0.);
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
