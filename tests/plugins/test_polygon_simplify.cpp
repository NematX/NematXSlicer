/* slic3r_api::polygons_simplify (Api/plugin/cpp/PolygonOps.hpp) replaces the native Slic3r::polygons_simplify for plugins.
   It cleans every ring alone (polygons_ensure_valid, a NonZero clipper_union, the clockwise orientation restored), then
   unites all the cleaned rings. The tests compare it with the native function (strictly_simple = false) on:
   - random star-shaped rings with holes, several tolerances;
   - the lslices of a few test meshes (Test::TestMesh), several tolerances;
   - self-intersecting rings in both orientations, a self-intersecting contour over a hole, an orphan clockwise ring;
   and check the documented behavior: orientation (contours CCW, holes CW), empty input, rings that collapse, and touching
   rings that merge (the accepted difference). Results are compared as NonZero regions and by ring count, never ring by
   ring: the order of the rings and their first point differ from the native result.
   Canonical ring lists keep orientation and ignore start points and collection order; identical lists have zero symmetric difference.
   A symmetric difference below 1e-3 mm2 per collection is required. */
#include <catch2/catch.hpp>

#include "plugin_test_helpers.hpp"
#include "clipper_offset_test_helpers.hpp"
#include "test_data.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <string>
#include <vector>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/cpp/PolygonOps.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Steps/StepLayerHeightGeneration.hpp"
#include "libslic3r/Steps/StepPostSlicing.hpp"
#include "libslic3r/Steps/StepSlicing.hpp"

using namespace Slic3r;

namespace {
// Largest accepted area of the symmetric difference with the native result, in mm2.
constexpr double max_sym_diff_mm2 = 1e-3;
// Tolerances (mm) of the comparisons.
const std::vector<double> tolerances_mm = {0.005, 0.0125, 0.05, 0.2};

// Scaled point from millimeters.
Point point_mm(double x, double y);
// Area in mm2 from an area in scaled units squared.
double mm2(double scaled_area);
// Area in mm2 of the symmetric difference of two ring sets (rings use the NonZero fill: CCW contours, CW holes).
double sym_diff_mm2(const Polygons &a, const Polygons &b);
// Run the API helper on native polygons and return the result as native polygons.
Polygons simplify_api(const Polygons &polygons, double tolerance_mm);
// Run the native function on the same input.
Polygons simplify_native(const Polygons &polygons, double tolerance_mm);
// Number of rings whose orientation is wrong for their nesting depth (contour CCW at an even depth, hole CW at an odd one).
size_t wrong_orientation_count(const Polygons &polygons);
// Star-shaped CCW ring around a center, with a random radius per vertex.
Polygon random_star(std::mt19937 &rng, const Point &center, double radius_mm, size_t points);
// Sliced lslices of a test mesh (Test::TestMesh), one Polygons (contours CCW, holes CW) per layer.
std::vector<Polygons> sliced_layers(Test::TestMesh mesh);
// Compare the API helper with the native function on one input; returns the symmetric difference (mm2).
double compare_with_native(const Polygons &input, double tolerance_mm, bool expect_same_ring_count);

Point point_mm(double x, double y)
{
    return Point(scale_i(x), scale_i(y));
}

double mm2(double scaled_area)
{
    return scaled_area * SCALING_FACTOR * SCALING_FACTOR;
}

double sym_diff_mm2(const Polygons &a, const Polygons &b)
{
    return mm2(area(diff_ex(a, b)) + area(diff_ex(b, a)));
}

Polygons simplify_api(const Polygons &polygons, double tolerance_mm)
{
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    const slic3r_api::PolygonCollection view(reinterpret_cast<const polygon_collection_handle *>(&polygons));
    slic3r_api::StoredPolygonCollection result = slic3r_api::polygons_simplify(handle, view, scale_d(tolerance_mm));
    // Copy out before the storage (which owns the result) is destroyed.
    return *reinterpret_cast<const Polygons *>(result.handle());
}

Polygons simplify_native(const Polygons &polygons, double tolerance_mm)
{
    return polygons_simplify(polygons, scale_d(tolerance_mm), false);
}

size_t wrong_orientation_count(const Polygons &polygons)
{
    size_t wrong = 0;
    for (size_t i = 0; i < polygons.size(); ++i) {
        // Depth of ring i: number of other rings that contain its first point.
        size_t depth = 0;
        for (size_t j = 0; j < polygons.size(); ++j)
            if (j != i && polygons[j].contains(polygons[i].points.front()))
                ++depth;
        const bool is_hole = depth % 2 == 1;
        if (polygons[i].is_counter_clockwise() == is_hole)
            ++wrong;
    }
    return wrong;
}

Polygon random_star(std::mt19937 &rng, const Point &center, double radius_mm, size_t points)
{
    std::uniform_real_distribution<double> jitter(0.8, 1.0);
    Polygon polygon;
    for (size_t i = 0; i < points; ++i) {
        const double angle = 2. * PI * double(i) / double(points);
        const double radius = radius_mm * jitter(rng);
        polygon.points.emplace_back(center.x() + scale_i(radius * std::cos(angle)), center.y() + scale_i(radius * std::sin(angle)));
    }
    return polygon;
}

// Slice one test-owned model centered at (100, 100) with default active plugins; return copied layers, with exceptions propagated; validation diagnostics are not checked.
std::vector<Polygons> sliced_layers(Test::TestMesh mesh)
{
    // The Print needs the plugin runtime.
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    Print print;
    Model model;
    config.set_deserialize_strict({{"support_material", "0"}, {"layer_height", "0.3"}, {"first_layer_height", "0.3"}});
    ModelObject *object = model.add_object();
    object->name = "object.stl";
    object->add_volume(Test::mesh(mesh));
    object->add_instance();
    // Default arrangement allows no rotation; centering this sole instance fixes the same XY position without packing.
    model.center_instances_around_point({100, 100});
    object->ensure_on_bed();
    print.auto_assign_extruders(object);
    print.apply(model, config);
    print.validate();
    print.set_status_silent();
    Orchestrator &host = Orchestrator::instance();
    Steps::StepLayerHeightGeneration::prepare(print);
    Steps::StepLayerHeightGeneration::run_step(host, print);
    Steps::StepSlicing::run_step(host, print);
    Steps::StepPostSlicing::prepare(print);
    Steps::StepPostSlicing::run_step(host, print);
    std::vector<Polygons> layers;
    for (const Layer &layer : print.object(0).layers())
        layers.push_back(to_polygons(layer.lslices()));
    return layers;
}

// Simplify borrowed rings at a millimeter tolerance; return area difference and report failed geometry checks, retaining both owned results locally.
double compare_with_native(const Polygons &input, double tolerance_mm, bool expect_same_ring_count)
{
    const Polygons api = simplify_api(input, tolerance_mm);
    const Polygons native = simplify_native(input, tolerance_mm);
    // Canonical rings preserve orientation; equal sorted ring lists describe exactly the same NonZero region.
    std::vector<clipper_offset_test::CanonicalRing> api_rings, native_rings;
    for (const Polygon &ring : api)
        api_rings.push_back(clipper_offset_test::canonical_ring(ring));
    for (const Polygon &ring : native)
        native_rings.push_back(clipper_offset_test::canonical_ring(ring));
    std::sort(api_rings.begin(), api_rings.end());
    std::sort(native_rings.begin(), native_rings.end());
    const double diff = api_rings == native_rings ? 0. : sym_diff_mm2(api, native);
    CHECK(diff < max_sym_diff_mm2);
    // The comparison is not vacuous: something is kept.
    if (!input.empty())
        CHECK(!api.empty());
    CHECK(wrong_orientation_count(api) == 0);
    if (expect_same_ring_count)
        CHECK(api.size() == native.size());
    return diff;
}
} // namespace

TEST_CASE("polygons_simplify matches the native function on random rings", "[plugins][polygon-simplify]")
{
    std::mt19937 rng(12345);
    double worst = 0.;
    for (const double tolerance : tolerances_mm) {
        for (int iteration = 0; iteration < 20; ++iteration) {
            // Three islands far from each other, the first one with a hole (clockwise ring inside the contour).
            Polygons input;
            input.push_back(random_star(rng, point_mm(0., 0.), 20., 150));
            Polygon hole = random_star(rng, point_mm(0., 0.), 6., 80);
            hole.make_clockwise();
            input.push_back(hole);
            input.push_back(random_star(rng, point_mm(60., 0.), 10., 120));
            input.push_back(random_star(rng, point_mm(0., 60.), 12., 60));
            worst = std::max(worst, compare_with_native(input, tolerance, true));
            // The rings really are simplified (the stars have 410 points).
            if (tolerance >= 0.05) {
                size_t points = 0;
                for (const Polygon &ring : simplify_api(input, tolerance))
                    points += ring.size();
                CHECK(points < 410);
            }
        }
    }
    WARN("random rings: largest symmetric difference " << worst << " mm2");
}

TEST_CASE("polygons_simplify matches the native function on sliced test meshes", "[plugins][polygon-simplify]")
{
    // The ring count is expected to be equal to the native one: the islands of these layers do not touch each other
    // after the simplification, so the final union has nothing to merge.
    double worst = 0.;
    size_t layer_count = 0;
    for (const Test::TestMesh mesh : {Test::TestMesh::sphere_50mm, Test::TestMesh::cube_with_hole, Test::TestMesh::gt2_teeth,
                                     Test::TestMesh::two_hollow_squares, Test::TestMesh::ipadstand}) {
        const std::vector<Polygons> layers = sliced_layers(mesh);
        REQUIRE(!layers.empty());
        for (const double tolerance : tolerances_mm)
            for (size_t i = 0; i < layers.size(); i += 3) {
                ++layer_count;
                worst = std::max(worst, compare_with_native(layers[i], tolerance, true));
            }
    }
    WARN("test meshes lslices: " << layer_count << " comparisons, largest symmetric difference " << worst << " mm2");
}

TEST_CASE("polygons_simplify orientation: contours CCW, holes CW", "[plugins][polygon-simplify]")
{
    Polygon outer({point_mm(0., 0.), point_mm(20., 0.), point_mm(20., 20.), point_mm(0., 20.)});
    Polygon inner({point_mm(5., 5.), point_mm(15., 5.), point_mm(15., 15.), point_mm(5., 15.)});
    // Rings in the right orientations (contour CCW, hole CW) keep them.
    outer.make_counter_clockwise();
    inner.make_clockwise();
    Polygons result = simplify_api({outer, inner}, 0.0125);
    REQUIRE(result.size() == 2);
    CHECK(std::count_if(result.begin(), result.end(), [](const Polygon &p) { return p.is_counter_clockwise(); }) == 1);
    CHECK(std::count_if(result.begin(), result.end(), [](const Polygon &p) { return p.is_clockwise(); }) == 1);
    CHECK(wrong_orientation_count(result) == 0);
    CHECK(mm2(area(result)) == Approx(300.).margin(1e-6));

    // The inverted pair (contour CW, hole CCW) describes the same region under the NonZero rule, and the final union
    // normalizes it: the native function would keep the inverted orientations.
    outer.make_clockwise();
    inner.make_counter_clockwise();
    result = simplify_api({outer, inner}, 0.0125);
    REQUIRE(result.size() == 2);
    CHECK(std::count_if(result.begin(), result.end(), [](const Polygon &p) { return p.is_counter_clockwise(); }) == 1);
    CHECK(std::count_if(result.begin(), result.end(), [](const Polygon &p) { return p.is_clockwise(); }) == 1);
    CHECK(wrong_orientation_count(result) == 0);
    CHECK(mm2(area(result)) == Approx(300.).margin(1e-6));
}

TEST_CASE("polygons_simplify cleans each ring alone, like the native function", "[plugins][polygon-simplify]")
{
    // Compare the NonZero regions with the native function and check the orientation of the output.
    auto compare = [](const Polygons &input) {
        const Polygons api = simplify_api(input, 0.0125);
        const Polygons native = simplify_native(input, 0.0125);
        CHECK(!api.empty());
        CHECK(sym_diff_mm2(api, native) < max_sym_diff_mm2);
        CHECK(wrong_orientation_count(api) == 0);
    };
    // Self-intersecting contour: its two lobes have opposite windings.
    const Polygon bow_tie({point_mm(0., 0.), point_mm(10., 10.), point_mm(0., 10.), point_mm(10., 0.)});
    // Figure-eight (lemniscate of Gerono) sampled with 60 points.
    Polygon eight;
    for (int i = 0; i < 60; ++i) {
        const double t = 2. * PI * double(i) / 60.;
        eight.points.push_back(point_mm(10. * std::cos(t), 5. * std::sin(2. * t)));
    }
    for (const Polygon &ring : {bow_tie, eight}) {
        Polygon one_way = ring;
        Polygon other_way = ring;
        std::reverse(other_way.points.begin(), other_way.points.end());
        SECTION("self-intersecting ring, one orientation")
        {
            compare({one_way});
        }
        SECTION("self-intersecting ring, the other orientation")
        {
            compare({other_way});
        }
    }
    SECTION("self-intersecting contour over a hole")
    {
        // Contour A, with the bottom lobe (a triangle around the point (4.4, 4.4)) holding the clockwise hole H.
        // The native function normalizes the lobes of A alone and keeps H as a hole. A union of all the rings at
        // once sums their windings: the lobe of A holding H has the negative winding, so H adds a second negative
        // winding and H is filled. The previous version of the helper (a single union over the ensure_valid result)
        // gave a region differing from the native one by 1 mm2 on this input (measured; the area of H).
        const Polygon contour({point_mm(0., 0.), point_mm(10., 10.), point_mm(0., 10.), point_mm(8., 0.)});
        Polygon hole({point_mm(4., 1.), point_mm(4., 2.), point_mm(5., 2.), point_mm(5., 1.)});
        hole.make_clockwise();
        const Polygons input{contour, hole};
        compare(input);
        // The same input through a single global union, as the previous version did.
        Polygons global_only = input;
        ensure_valid(global_only, coord_t(scale_d(0.0125)));
        const Polygons native = simplify_native(input, 0.0125);
        const double previous_diff = sym_diff_mm2(union_(global_only), native);
        WARN("previous global-only union on the counter-example: " << previous_diff << " mm2 away from the native region");
        CHECK(previous_diff > 0.5);
        // The helper keeps the hole: the region is the native one.
        CHECK(sym_diff_mm2(simplify_api(input, 0.0125), native) < max_sym_diff_mm2);
    }
    SECTION("orphan clockwise ring")
    {
        // A clockwise ring with no contour around it: a filled area for the NonZero rule, in the native output too.
        Polygon orphan({point_mm(0., 0.), point_mm(10., 0.), point_mm(10., 10.), point_mm(0., 10.)});
        orphan.make_clockwise();
        const Polygons api = simplify_api({orphan}, 0.0125);
        REQUIRE(api.size() == 1);
        CHECK(api.front().is_counter_clockwise());
        CHECK(mm2(area(api)) == Approx(100.).margin(1e-6));
        compare({orphan});
    }
}

TEST_CASE("polygons_simplify edge cases", "[plugins][polygon-simplify]")
{
    const Polygon square({point_mm(20., 0.), point_mm(30., 0.), point_mm(30., 10.), point_mm(20., 10.)});

    SECTION("empty input")
    {
        CHECK(simplify_api({}, 0.0125).empty());
        CHECK(simplify_native({}, 0.0125).empty());
    }
    SECTION("a ring collapsing below 3 points disappears")
    {
        Polygon sliver({point_mm(0., 0.), point_mm(10., 0.), point_mm(5., 0.001)});
        sliver.make_counter_clockwise();
        const Polygons input{square, sliver};
        const Polygons api = simplify_api(input, 0.0125);
        const Polygons native = simplify_native(input, 0.0125);
        CHECK(api.size() == 1);
        CHECK(native.size() == 1);
        CHECK(sym_diff_mm2(api, native) < max_sym_diff_mm2);
    }
    SECTION("only collapsing rings give an empty result")
    {
        Polygon sliver({point_mm(0., 0.), point_mm(10., 0.), point_mm(5., 0.001)});
        CHECK(simplify_api({sliver}, 0.0125).empty());
    }
    SECTION("a hole collapsing is filled, the contour stays")
    {
        Polygon outer({point_mm(0., 0.), point_mm(10., 0.), point_mm(10., 10.), point_mm(0., 10.)});
        Polygon thin_hole({point_mm(2., 5.), point_mm(8., 5.), point_mm(8., 5.002), point_mm(2., 5.002)});
        outer.make_counter_clockwise();
        thin_hole.make_clockwise();
        const Polygons input{outer, thin_hole};
        const Polygons api = simplify_api(input, 0.0125);
        const Polygons native = simplify_native(input, 0.0125);
        REQUIRE(api.size() == 1);
        CHECK(native.size() == 1);
        CHECK(api.front().is_counter_clockwise());
        CHECK(sym_diff_mm2(api, native) < max_sym_diff_mm2);
    }
    SECTION("rings touching after the simplification merge (accepted difference)")
    {
        // Two squares sharing the edge x = 10: the native function keeps two rings, the union gives one.
        Polygon left({point_mm(0., 0.), point_mm(10., 0.), point_mm(10., 10.), point_mm(0., 10.)});
        Polygon right({point_mm(10., 0.), point_mm(20., 0.), point_mm(20., 10.), point_mm(10., 10.)});
        const Polygons input{left, right};
        const Polygons api = simplify_api(input, 0.0125);
        const Polygons native = simplify_native(input, 0.0125);
        CHECK(native.size() == 2);
        CHECK(api.size() == 1);
        CHECK(sym_diff_mm2(api, native) < max_sym_diff_mm2);
    }
    SECTION("the input is not modified")
    {
        // A square with a vertex nudged 0.002 mm off the middle of each edge: the simplification (0.0125 mm) removes them.
        const Polygon noisy({point_mm(0., 0.), point_mm(5., 0.002), point_mm(10., 0.), point_mm(9.998, 5.), point_mm(10., 10.),
                             point_mm(5., 9.998), point_mm(0., 10.), point_mm(0.002, 5.)});
        const Polygons input{noisy};
        const Polygons copy = input;
        const Polygons result = simplify_api(input, 0.0125);
        REQUIRE(result.size() == 1);
        // The simplification really changed the ring, and the input stays as it was.
        CHECK(result.front().size() == 4);
        CHECK(input.front().size() == 8);
        CHECK(input == copy);
    }
}

// ---- slic3r_api::remove_collinear: identity with the native Polygon::remove_collinear ----

namespace {
// The points of a ring, in the C layout of the API.
using Ring = std::vector<c_point>;

// Run the native Polygon::remove_collinear on a copy of the ring; removed receives its return value.
Ring remove_collinear_native(const Ring &ring, coord_t max_offset, size_t &removed)
{
    Polygon polygon;
    for (const c_point &point : ring)
        polygon.points.emplace_back(point.x, point.y);
    removed = polygon.remove_collinear(max_offset);
    Ring out;
    for (const Point &point : polygon.points)
        out.push_back(c_point{point.x(), point.y()});
    return out;
}

// Run the API port on a ring stored in a storage; removed receives the number of deleted points.
Ring remove_collinear_api(const Ring &ring, coord_t max_offset, size_t &removed)
{
    PluginStorage storage;
    slic3r_api::StoredPolygon polygon(reinterpret_cast<storage_handle *>(&storage));
    if (!ring.empty())
        polygon.insert_array(0, ring.data(), uint32_t(ring.size()));
    removed = slic3r_api::remove_collinear(polygon, max_offset);
    return polygon.points();
}

// True when the API port and the native function give the same points, in the same order, and the same count.
bool remove_collinear_same(const Ring &ring, coord_t max_offset)
{
    size_t native_removed = 0, api_removed = 0;
    const Ring native = remove_collinear_native(ring, max_offset, native_removed);
    const Ring api = remove_collinear_api(ring, max_offset, api_removed);
    return native_removed == api_removed && native.size() == api.size() &&
           std::equal(native.begin(), native.end(), api.begin(), [](const c_point &a, const c_point &b) { return a.x == b.x && a.y == b.y; });
}

c_point cp(coord_t x, coord_t y) { return c_point{x, y}; }
} // namespace

TEST_CASE("remove_collinear matches the native function on the listed cases", "[plugins][polygon-ops][remove-collinear]")
{
    const std::vector<coord_t> offsets{0, 1, 3, 100, 5000};
    const std::vector<Ring> rings{
        {},                                                      // empty
        {cp(0, 0)},                                              // one point
        {cp(0, 0), cp(10, 10)},                                  // two points
        {cp(0, 0), cp(10, 0), cp(20, 0)},                        // three aligned points
        {cp(0, 0), cp(10, 0), cp(20, 0), cp(30, 0), cp(40, 0)},  // all aligned: reduces to two points
        {cp(0, 0), cp(10, 0), cp(20, 0), cp(30, 0), cp(30, 40)}, // several removals at the head side
        {cp(0, 0), cp(0, 40), cp(10, 40), cp(20, 40), cp(30, 40)}, // several removals at the tail side
        {cp(0, 0), cp(5, 0), cp(10, 0), cp(10, 10), cp(10, 20), cp(0, 20), cp(0, 10)}, // head, middle and tail
        {cp(0, 0), cp(0, 0), cp(10, 0), cp(10, 0), cp(10, 10), cp(0, 10)},             // consecutive duplicates
        {cp(0, 0), cp(0, 0), cp(0, 0), cp(0, 0)},                                       // only duplicates
        {cp(0, 0), cp(100, 0), cp(50, 1), cp(200, 0), cp(0, 100)},                      // projection beyond the segment end
        {cp(0, 0), cp(100, 0), cp(-50, 1), cp(200, 0), cp(0, 100)},                     // projection before the segment start
        {cp(0, 0), cp(1000, 99), cp(1100, -9), cp(2000, 0), cp(0, 1000)},               // a removal changes the next test
        {cp(0, 0), cp(10, 0), cp(10, 10), cp(0, 10)},                                   // a square: nothing to remove
        {cp(0, 0), cp(10, 0), cp(0, 0), cp(10, 0), cp(0, 0)},                           // a back and forth stick
    };
    for (const Ring &ring : rings)
        for (const coord_t offset : offsets)
            CHECK(remove_collinear_same(ring, offset));
}

TEST_CASE("remove_collinear on the case where a removal hides the next one", "[plugins][polygon-ops][remove-collinear]")
{
    // With max_offset 100, the native function removes (1100,-9) and keeps (1000,99): the point kept is not
    // examined again after the removal, and (1100,-9) is compared with the next ORIGINAL point.
    const Ring ring{cp(0, 0), cp(1000, 99), cp(1100, -9), cp(2000, 0), cp(0, 1000)};
    size_t native_removed = 0, api_removed = 0;
    const Ring native = remove_collinear_native(ring, 100, native_removed);
    const Ring api = remove_collinear_api(ring, 100, api_removed);
    const Ring expected{cp(0, 0), cp(1000, 99), cp(2000, 0), cp(0, 1000)};
    REQUIRE(native.size() == expected.size());
    REQUIRE(api.size() == expected.size());
    for (size_t idx = 0; idx < expected.size(); ++idx) {
        CHECK(native[idx].x == expected[idx].x);
        CHECK(native[idx].y == expected[idx].y);
        CHECK(api[idx].x == expected[idx].x);
        CHECK(api[idx].y == expected[idx].y);
    }
    CHECK(native_removed == 1);
    CHECK(api_removed == 1);
}

TEST_CASE("remove_collinear matches the native function on random rings", "[plugins][polygon-ops][remove-collinear]")
{
    std::mt19937 random(20261019);
    auto uniform = [&random](int low, int high) { return std::uniform_int_distribution<int>(low, high)(random); };
    // The coordinates are small so that many points are aligned, repeated or within a few units of a line.
    const std::vector<coord_t> offsets{0, 1, 2, 5, 40, 300};
    size_t compared = 0, with_removal = 0;
    for (int test = 0; test < 6000; ++test) {
        Ring ring;
        const int count = uniform(0, 14);
        const int style = test % 3;
        c_point current = cp(uniform(-60, 60), uniform(-60, 60));
        for (int idx = 0; idx < count; ++idx) {
            if (style == 0) {
                ring.push_back(cp(uniform(-60, 60), uniform(-60, 60)));
            } else if (style == 1) {
                // A walk along a few directions, with a small jitter: near aligned points.
                current = cp(current.x + uniform(-2, 2) * 20 + uniform(-1, 1), current.y + uniform(-2, 2) * 20 + uniform(-1, 1));
                ring.push_back(current);
            } else {
                // Points on the sides of a square, plus some duplicates and far outliers.
                const int side = uniform(0, 3), along = uniform(0, 10) * 10, jitter = uniform(-2, 2);
                ring.push_back(side == 0 ? cp(along, jitter) : side == 1 ? cp(100 + jitter, along) :
                               side == 2 ? cp(100 - along, 100 + jitter) : cp(jitter, 100 - along));
                if (uniform(0, 5) == 0)
                    ring.push_back(ring.back());
                if (uniform(0, 9) == 0)
                    ring.push_back(cp(uniform(-500, 500), uniform(-500, 500)));
            }
        }
        for (const coord_t offset : offsets) {
            size_t native_removed = 0;
            remove_collinear_native(ring, offset, native_removed);
            with_removal += native_removed > 0 ? 1 : 0;
            ++compared;
            REQUIRE(remove_collinear_same(ring, offset));
        }
    }
    // The random rings must exercise the removals, or the comparison would prove nothing.
    CHECK(with_removal > compared / 10);
}

TEST_CASE("remove_collinear of a collection and of real slices", "[plugins][polygon-ops][remove-collinear]")
{
    // The sliced layers of the test meshes, with the model precision of the slicer: every ring of the collection.
    for (const Test::TestMesh mesh : {Test::TestMesh::A, Test::TestMesh::sloping_hole, Test::TestMesh::overhang}) {
        const std::vector<Polygons> layers = sliced_layers(mesh);
        for (const Polygons &layer : layers) {
            for (const coord_t offset : {coord_t(1), coord_t(100), coord_t(scale_i(0.01))}) {
                PluginStorage storage;
                slic3r_api::StoredPolygonCollection rings(reinterpret_cast<storage_handle *>(&storage));
                for (const Polygon &polygon : layer) {
                    slic3r_api::StoredPolygon ring(reinterpret_cast<storage_handle *>(&storage));
                    for (const Point &point : polygon.points)
                        ring.push_back(c_point{point.x(), point.y()});
                    rings.push_back_move(std::move(ring));
                }
                Polygons native = layer;
                size_t native_removed = 0;
                for (Polygon &polygon : native)
                    native_removed += polygon.remove_collinear(offset);
                CHECK(slic3r_api::remove_collinear(rings, offset) == native_removed);
                REQUIRE(rings.size() == native.size());
                for (uint32_t idx = 0; idx < rings.size(); ++idx) {
                    const std::vector<c_point> points = rings.at(idx).points();
                    REQUIRE(points.size() == native[idx].points.size());
                    for (size_t point = 0; point < points.size(); ++point)
                        REQUIRE((points[point].x == native[idx].points[point].x() && points[point].y == native[idx].points[point].y()));
                }
            }
        }
    }
}
