#include "polygon_metrics_test_helpers.hpp"
#include "support_tree_test_helpers.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include "libslic3r/AABBTreeIndirect.hpp"
#include "libslic3r/AABBTreeLines.hpp"
#include "libslic3r/PointUtils.hpp"
#include "libslic3r/Plugins/Support/Tree/TreeGeometry.hpp"

/* Tests of the geometry helpers of the tree support family (Plugins/Support/Tree/TreeGeometry.hpp). Each helper that replaces a native
   function is compared with that function, on random input with fixed seeds and on edge cases:

   - scalars, point conversions, products and hash: exact equality with the native Point operations and PointHash;
   - bounding boxes, area, length, closed polylines, lines, flat polygons of expolygons: exact equality (same operations, same order);
   - point_in_polygons: equality with the native contains(Polygons, Point, border_result) on random points, on the vertices and on the
     edges of the rings (the border cases), with holes, overlapping rings, rings under 3 points;
   - nearest_on_lines: the squared distance and the hit point equal those of AABBTreeLines::squared_distance_to_indexed_lines(), the hit
     line too unless two lines tie (the first line of the vector wins here, which is checked on its own);
   - spatial_order: the exact order of AABBTreeIndirect::Tree::build_modify_input(), including many ties;
   - the Clipper wrappers: compared as regions (symmetric difference below 1e-3 mm2) and by ring or expolygon count, never ring by
     ring: the order of the rings and their first point differ from the native result. The largest differences are measured but not asserted;
     the hidden case [.][tree-geometry-report] prints them (WARN), and the hidden benchmark [.][tree-geometry-benchmark] prints timings.
     The INFO lines of the other cases give context to a check and show when it fails.

   The tests build no Print, so the plugin runtime does not need to be initialized. */

using namespace Slic3r;
namespace PM = Slic3r::Test::PolygonMetrics;
namespace SG = Slic3r::Test::SupportGrid;
namespace ST = Slic3r::Test::SupportTree;
namespace TF = Slic3r::TreeFamily;

// Written inside the TreeFamily namespace on purpose: the c_point operators of TreeGeometry.hpp must be usable from there, next to the
// operators of the native types, in the same function.
namespace Slic3r::TreeFamily {
inline bool native_and_c_point_operators_coexist()
{
    const Point native_a(1, 2), native_b(3, 4);
    const c_point c_a{ 1, 2 }, c_b{ 3, 4 };
    const Polygon polygon_a({ Point(0, 0), Point(100000, 0), Point(0, 100000) });
    const Polygon polygon_b = polygon_a;
    const ExPolygon expolygon_a(polygon_a), expolygon_b(polygon_b);
    const BoundingBox box_a(native_a, native_b), box_b(native_a, native_b);
    const Linef line_a(Vec2d(0., 0.), Vec2d(1., 1.)), line_b(Vec2d(0., 0.), Vec2d(1., 1.));
    const std::pair<Point, int> pair_a(native_a, 1), pair_b(native_b, 1);
    const std::vector<Point> points_a{ native_a, native_b }, points_b{ native_a, native_b };
    bool ok = true;
    // native types, with the lookups that find the native operators of Slic3r
    ok = ok && native_a != native_b && (native_a + native_b) == Point(4, 6) && (native_b - native_a) == Point(2, 2);
    ok = ok && !(native_a == native_b);
    ok = ok && polygon_a == polygon_b && expolygon_a == expolygon_b && box_a == box_b && line_a.a == line_b.a;
    ok = ok && !(pair_a == pair_b) && points_a == points_b;
    // c_point, with the operators of TreeGeometry.hpp, in the same function
    ok = ok && c_a != c_b && (c_a + c_b) == c_point{ 4, 6 } && (c_b - c_a) == c_point{ 2, 2 } && (c_a * 2) == c_point{ 2, 4 } && (c_b / 2) == c_point{ 1, 2 };
    return ok;
}

struct PointOperatorResults {
    c_point sum, difference, tripled, middle;
    bool equal, different;
    double length;
};
inline PointOperatorResults apply_point_operators(c_point p, c_point q)
{
    return PointOperatorResults{ p + q, p - q, p * 3, (p + q) / 2, p == q, p != q, slic3r_api::norm(p - q) };
}
} // namespace Slic3r::TreeFamily

namespace {
// Largest accepted area of the symmetric difference with the native result, in mm2.
constexpr double max_sym_diff_mm2 = 1e-3;
// The radius of a vertex of the stars is between this fraction of the radius of the star and the radius itself.
constexpr double star_min_fraction = 0.6;

// Read-only API views over native collections: the native containers are what the API handles point to.
slic3r_api::PolygonCollection view_of(const Polygons &polygons);
slic3r_api::ExPolygonCollection view_of(const ExPolygons &expolygons);
slic3r_api::PolylineCollection view_of(const Polylines &polylines);
// c_bounding_box from a native box, and back (ST::to_c_point and ST::to_point do the points; there is no shared helper for boxes).
c_bounding_box c_box_of(const BoundingBox &box);
BoundingBox box_of(const c_bounding_box &box);

// A random set of overlapping stars, the first one with a clockwise star hole; 3 to 5 rings in a square of about 40 mm.
Polygons random_scene(std::mt19937 &rng);
// A random point in the box of the scene, grown by margin_mm.
Point random_point(std::mt19937 &rng, const BoundingBox &box, double margin_mm);

// An item to order, for the native tree: the interface of SupportElementMerging used by AABBTreeIndirect.
struct NativeItem {
    size_t id = 0;
    Eigen::AlignedBox<coord_t, 2> box;
    const Eigen::AlignedBox<coord_t, 2> &bbox() const { return box; }
    Point centroid() const { return (box.min() + box.max()) / 2; }
    static size_t idx() { return 0; }
};
// The same item for the helper.
struct ApiItem {
    size_t id = 0;
    c_bounding_box box;
};

// The result of an operation as regions: both as rings of the native polygons.
struct Largest {
    double sym_diff = 0.;
    size_t identical_rings = 0, compared = 0;
};

// Rings that tell the fill rules apart: contours counter-clockwise (+1), holes clockwise (-1), so a region can have a winding of -2 to 2.
// `clean` scenes are valid non-zero regions, for which polygons_offset must equal the native function; the others are measured only.
struct WindingScene {
    std::string name;
    Polygons rings;
    bool clean;
};

// The ids of both orderings of the same boxes, and whether they are equal.
bool same_order(const std::vector<c_bounding_box> &boxes);
// Random boxes with min in [-range, range] and a side of at most max_side (0 included), on a grid of the given step.
std::vector<c_bounding_box> random_boxes(std::mt19937 &rng, size_t count, coord_t range, coord_t max_side, coord_t step);
// Check one Clipper result against the native one: same region and, unless told otherwise, same ring count; track the largest difference.
void check_polygons(const Polygons &mine, const Polygons &native, Largest &largest, bool same_ring_count = true);
// Check Clipper wrappers on deterministic scenes and return their difference report.
std::string run_clipper_wrappers();
// The same for expolygons: same region, same expolygon count, same ring count.
void check_expolygons(const ExPolygons &mine, const ExPolygons &native, Largest &largest);
// Rings that tell the fill rules apart, clean or not.
std::vector<WindingScene> winding_scenes(std::mt19937 &rng);

// A point from coordinates in micrometers. The rings of the fixed cases are built from such points: the Polygon constructors drop the
// last point of a ring that comes back within SCALED_EPSILON of the first one, which would change a ring of a few units.
Point P(coord_t x, coord_t y);
c_point CP(coord_t x, coord_t y);
// A grid of count_x * count_y one-millimeter squares 3 mm apart, each with a clockwise hole of 0.4 mm when with_holes.
Polygons square_grid(size_t count_x, size_t count_y, bool with_holes);
// The time in milliseconds of the best of three runs of fn.
template<typename Fn> double best_of_three_ms(Fn &&fn);
// One input of the large cases.
struct LargeCase {
    const char *name;
    Polygons rings;
    double delta_mm;
};
// The inputs of the large cases: 2000 squares, then with a hole in each (4000 rings), grown and shrunk.
std::vector<LargeCase> large_cases();

Point P(coord_t x, coord_t y)
{
    return Point(x * 1000, y * 1000);
}

c_point CP(coord_t x, coord_t y)
{
    return c_point{ x * 1000, y * 1000 };
}

slic3r_api::PolygonCollection view_of(const Polygons &polygons)
{
    return slic3r_api::PolygonCollection(reinterpret_cast<const polygon_collection_handle *>(&polygons));
}

slic3r_api::ExPolygonCollection view_of(const ExPolygons &expolygons)
{
    return slic3r_api::ExPolygonCollection(reinterpret_cast<const expolygon_collection_handle *>(&expolygons));
}

slic3r_api::PolylineCollection view_of(const Polylines &polylines)
{
    return slic3r_api::PolylineCollection(reinterpret_cast<const polyline_collection_handle *>(&polylines));
}

c_bounding_box c_box_of(const BoundingBox &box)
{
    return c_bounding_box{ ST::to_c_point(box.min), ST::to_c_point(box.max) };
}

BoundingBox box_of(const c_bounding_box &box)
{
    return BoundingBox(ST::to_point(box.min), ST::to_point(box.max));
}

Polygons random_scene(std::mt19937 &rng)
{
    std::uniform_real_distribution<double> position(-15., 15.);
    std::uniform_real_distribution<double> radius(5., 14.);
    std::uniform_int_distribution<int> count(3, 5);
    std::uniform_int_distribution<int> vertices(5, 40);
    Polygons scene;
    for (int i = 0, n = count(rng); i < n; ++i)
        scene.push_back(PM::random_star(rng, Point(scale_i(position(rng)), scale_i(position(rng))), radius(rng), size_t(vertices(rng)), star_min_fraction));
    // A clockwise hole inside the first star (its radius is always above 0.6 * 5 mm).
    Polygon hole = PM::random_star(rng, scene.front().centroid(), 1.5, 12, star_min_fraction);
    hole.make_clockwise();
    scene.push_back(hole);
    return scene;
}

Point random_point(std::mt19937 &rng, const BoundingBox &box, double margin_mm)
{
    const coord_t margin = scale_i(margin_mm);
    std::uniform_int_distribution<coord_t> x(box.min.x() - margin, box.max.x() + margin);
    std::uniform_int_distribution<coord_t> y(box.min.y() - margin, box.max.y() + margin);
    return Point(x(rng), y(rng));
}

bool same_order(const std::vector<c_bounding_box> &boxes)
{
    std::vector<NativeItem> native(boxes.size());
    std::vector<ApiItem> mine(boxes.size());
    for (size_t i = 0; i < boxes.size(); ++i) {
        native[i].id = i;
        native[i].box = Eigen::AlignedBox<coord_t, 2>(Point(boxes[i].min.x, boxes[i].min.y), Point(boxes[i].max.x, boxes[i].max.y));
        mine[i].id = i;
        mine[i].box = boxes[i];
    }
    AABBTreeIndirect::Tree<2, coord_t> tree;
    tree.build_modify_input(native);
    TF::spatial_order(mine, [](const ApiItem &item) { return item.box; });
    REQUIRE(native.size() == mine.size());
    bool same = true;
    for (size_t i = 0; i < native.size(); ++i)
        same = same && native[i].id == mine[i].id;
    return same;
}

std::vector<c_bounding_box> random_boxes(std::mt19937 &rng, size_t count, coord_t range, coord_t max_side, coord_t step)
{
    std::uniform_int_distribution<coord_t> position(-range, range);
    std::uniform_int_distribution<coord_t> side(0, max_side);
    std::vector<c_bounding_box> boxes;
    for (size_t i = 0; i < count; ++i) {
        const coord_t x = position(rng) * step, y = position(rng) * step;
        boxes.push_back(c_bounding_box{ c_point{ x, y }, c_point{ x + side(rng) * step, y + side(rng) * step } });
    }
    return boxes;
}

// Compare borrowed rings by canonical form or NonZero difference; record statistics and failed area/count checks without changing inputs.
void check_polygons(const Polygons &mine, const Polygons &native, Largest &largest, bool same_ring_count)
{
    const bool identical_rings = ST::rings_of(mine) == ST::rings_of(native);
    const double diff = identical_rings ? 0. : PM::sym_diff_mm2(mine, native);
    CHECK(diff < max_sym_diff_mm2);
    if (same_ring_count)
        CHECK(mine.size() == native.size());
    largest.sym_diff = std::max(largest.sym_diff, diff);
    ++largest.compared;
    if (identical_rings)
        ++largest.identical_rings;
}

void check_expolygons(const ExPolygons &mine, const ExPolygons &native, Largest &largest)
{
    CHECK(mine.size() == native.size());
    // Flattened by hand: the native to_polygons() asserts on the order of nested islands and holes, which is not what is compared.
    auto flatten = [](const ExPolygons &expolygons) {
        Polygons polygons;
        for (const ExPolygon &expolygon : expolygons) {
            polygons.push_back(expolygon.contour);
            polygons.insert(polygons.end(), expolygon.holes.begin(), expolygon.holes.end());
        }
        return polygons;
    };
    check_polygons(flatten(mine), flatten(native), largest);
}

std::vector<WindingScene> winding_scenes(std::mt19937 &rng)
{
    auto ccw = [&](double x, double y, double radius) { return PM::random_star(rng, Point(scale_i(x), scale_i(y)), radius, 24, star_min_fraction); };
    auto cw = [&](double x, double y, double radius) {
        Polygon ring = PM::random_star(rng, Point(scale_i(x), scale_i(y)), radius, 24, star_min_fraction);
        ring.make_clockwise();
        return ring;
    };
    std::vector<WindingScene> scenes;
    // Clean lists. Each radius of the nested rings is under 0.6 times the previous one: the stars have a radius between 0.6 and 1 times
    // the given one, so the rings do not cross.
    scenes.push_back({ "nested rings", { ccw(0, 0, 40), cw(0, 0, 24), ccw(0, 0, 14), cw(0, 0, 8) }, true });
    scenes.push_back({ "a contour with a hole, and a separate contour", { ccw(0, 0, 12), cw(1, 0, 4), ccw(40, 0, 10) }, true });
    scenes.push_back({ "two separate contours", { ccw(0, 0, 10), ccw(40, 0, 10) }, true });
    // Not clean: the native function offsets each ring alone, which a union cannot reproduce.
    scenes.push_back({ "clockwise orphan ring", { cw(0, 0, 10) }, false });
    scenes.push_back({ "clockwise orphan and a separate counter-clockwise ring", { cw(0, 0, 10), ccw(40, 0, 10) }, false });
    scenes.push_back({ "clockwise ring half over a counter-clockwise one", { ccw(0, 0, 12), cw(8, 0, 10) }, false });
    scenes.push_back({ "two clockwise rings overlapping", { cw(0, 0, 10), cw(7, 0, 10) }, false });
    scenes.push_back({ "two counter-clockwise rings overlapping", { ccw(0, 0, 10), ccw(7, 0, 10) }, false });
    scenes.push_back({ "hole straddling two contours", { ccw(0, 0, 12), ccw(10, 0, 12), cw(5, 0, 4) }, false });
    scenes.push_back({ "two clockwise rings nested (winding -2)", { cw(0, 0, 12), cw(0, 0, 5) }, false });
    scenes.push_back({ "clockwise ring inside two overlapping contours", { ccw(0, 0, 12), ccw(3, 0, 12), cw(1, 0, 3) }, false });
    Polygon left({ Point(0, 0), Point(scale_i(10.), 0), Point(scale_i(10.), scale_i(10.)), Point(0, scale_i(10.)) });
    Polygon right({ Point(scale_i(10.), 0), Point(scale_i(20.), 0), Point(scale_i(20.), scale_i(10.)), Point(scale_i(10.), scale_i(10.)) });
    scenes.push_back({ "squares sharing an edge", { left, right }, false });
    return scenes;
}
} // namespace

TEST_CASE("tree geometry scalars match the native ones", "[plugins][tree-geometry]")
{
    std::mt19937 rng(1);
    std::uniform_real_distribution<double> real(-1e6, 1e6);
    for (int i = 0; i < 1000; ++i) {
        const double value = real(rng);
        CHECK(TF::sqr(value) == Slic3r::sqr(value));
        CHECK(TF::sqr(float(value)) == Slic3r::sqr(float(value)));
    }
    std::uniform_int_distribution<int> small(0, 100000);
    for (int i = 0; i < 1000; ++i) {
        const int dividend = small(rng), divisor = small(rng) + 1;
        CHECK(TF::round_up_divide(dividend, divisor) == Slic3r::round_up_divide(dividend, divisor));
        CHECK(TF::round_up_divide(size_t(dividend), size_t(divisor)) == Slic3r::round_up_divide(size_t(dividend), size_t(divisor)));
    }
    // The call of TreeVolumes with an explicit type, and the exact cases.
    CHECK(TF::round_up_divide<int>(7, 2) == 4);
    CHECK(TF::round_up_divide<int>(8, 2) == 4);
    CHECK(TF::round_up_divide<int>(0, 5) == 0);
}

TEST_CASE("tree geometry point conversions, products and hash match the native ones", "[plugins][tree-geometry]")
{
    std::mt19937 rng(2);
    std::uniform_int_distribution<coord_t> coord(-200000000, 200000000);
    std::uniform_real_distribution<double> real(-300., 300.);
    // EPSILON and the scale functions are the ones of slic3r_def.h, which the native code imports (libslic3r.h): nothing to compare.
    for (int i = 0; i < 2000; ++i) {
        const Point p(coord(rng), coord(rng)), q(coord(rng), coord(rng));
        const c_point cp = ST::to_c_point(p), cq = ST::to_c_point(q);
        // to_vec2d and unscale_p
        CHECK(TF::to_vec2d(cp) == p.cast<double>());
        CHECK(TF::unscale_p(cp) == unscale_p(p));
        // integer products
        CHECK(TF::dot(cp, cq) == p.cast<int64_t>().dot(q.cast<int64_t>()));
        CHECK(TF::squared_norm(cp) == p.cast<int64_t>().squaredNorm());
        CHECK(TF::squared_norm(c_point{ cp.x - cq.x, cp.y - cq.y }) == (p - q).cast<int64_t>().squaredNorm());
        // the API operators, found in the TreeFamily namespace
        const TF::PointOperatorResults ops = TF::apply_point_operators(cp, cq);
        CHECK(ST::to_point(ops.sum) == p + q);
        CHECK(ST::to_point(ops.difference) == p - q);
        CHECK(ST::to_point(ops.tripled) == Point(p.x() * 3, p.y() * 3));
        CHECK(ST::to_point(ops.middle) == (p + q) / 2);
        CHECK(ops.equal == (p == q));
        CHECK(ops.different == (p != q));
        CHECK(ops.length == (p - q).cast<double>().norm());
        // vectors of doubles back to points: truncation and millimeter scaling
        const Vec2d v(real(rng), real(rng));
        CHECK(ST::to_point(TF::truncated_point(v)) == Point(v.cast<coord_t>()));
        CHECK(ST::to_point(TF::scaled_point(v)) == Point::new_scale(v));
    }

    SECTION("PointHash and PointEqual")
    {
        std::unordered_set<c_point, TF::PointHash, TF::PointEqual> mine;
        std::unordered_set<Point, Slic3r::PointHash> native;
        for (int i = 0; i < 5000; ++i) {
            // Few distinct values, and some close ones (hash collisions on purpose).
            const Point p(coord(rng) / 100000 * 7, coord(rng) / 100000 * 7);
            CHECK(TF::PointHash()(ST::to_c_point(p)) == Slic3r::PointHash()(p));
            const bool mine_new = mine.insert(ST::to_c_point(p)).second;
            const bool native_new = native.insert(p).second;
            CHECK(mine_new == native_new);
        }
        CHECK(mine.size() == native.size());
        CHECK(TF::PointHash()(c_point{ -1, -1 }) == Slic3r::PointHash()(Point(-1, -1)));
        CHECK(TF::PointHash()(c_point{ 0, 0 }) == Slic3r::PointHash()(Point(0, 0)));
    }
}

TEST_CASE("tree geometry bounding boxes match the native ones", "[plugins][tree-geometry]")
{
    std::mt19937 rng(3);
    for (int iteration = 0; iteration < 50; ++iteration) {
        const Polygons scene = random_scene(rng);
        const BoundingBox native = get_extents(scene);
        const c_bounding_box mine = TF::polygons_bounding_box(view_of(scene));
        CHECK(box_of(mine) == native);
        for (const coord_t delta : { coord_t(0), coord_t(1), SCALED_EPSILON, scale_i(0.5) })
            CHECK(box_of(TF::bounding_box_inflated(mine, delta)) == native.inflated(delta));
    }

    SECTION("empty collection and empty ring")
    {
        const Polygons none;
        CHECK(!slic3r_api::bounding_box_valid(TF::polygons_bounding_box(view_of(none))));
        // An empty box stays empty when inflated, and the merge of the first box initializes it.
        const c_bounding_box inflated = TF::bounding_box_inflated(TF::polygons_bounding_box(view_of(none)), 5);
        CHECK(!slic3r_api::bounding_box_valid(inflated));
        // A ring without points does not change the box.
        const Polygons with_empty{ Polygon(), Polygon({ P(0, 0), P(10, 0), P(10, 20) }) };
        const c_bounding_box with_empty_box = TF::polygons_bounding_box(view_of(with_empty));
        INFO("box " << with_empty_box.min.x << "," << with_empty_box.min.y << " " << with_empty_box.max.x << "," << with_empty_box.max.y);
        CHECK(box_of(with_empty_box) == BoundingBox(P(0, 0), P(10, 20)));
    }
    SECTION("ClipperOperand::bounding_box, the API function that replaces clipper_shapes_bounding_box, gives the same box")
    {
        const Polygons scene = random_scene(rng);
        PluginStorage owner;
        const slic3r_api::ClipperContext clipper(SG::handle(owner));
        CHECK(box_of(clipper(view_of(scene)).bounding_box()) == get_extents(scene));
    }
}

TEST_CASE("tree geometry polygon collection helpers match the native ones", "[plugins][tree-geometry]")
{
    std::mt19937 rng(4);
    PluginStorage owner;
    storage_handle *storage = SG::handle(owner);
    for (int iteration = 0; iteration < 50; ++iteration) {
        const Polygons scene = random_scene(rng);
        const slic3r_api::PolygonCollection view = view_of(scene);
        CHECK(TF::polygons_area(view) == area(scene));
        CHECK(TF::total_length(view) == total_length(scene));

        const Polylines native_polylines = to_polylines(scene);
        const Polylines mine_polylines = SG::native(TF::to_polylines(storage, view));
        REQUIRE(mine_polylines.size() == native_polylines.size());
        for (size_t i = 0; i < native_polylines.size(); ++i)
            CHECK(mine_polylines[i].points == native_polylines[i].points);

        const Lines native_lines = to_lines(scene);
        const std::vector<TF::LineSegment> mine_lines = TF::to_lines(view);
        REQUIRE(mine_lines.size() == native_lines.size());
        for (size_t i = 0; i < native_lines.size(); ++i) {
            CHECK(ST::to_point(mine_lines[i].a) == native_lines[i].a);
            CHECK(ST::to_point(mine_lines[i].b) == native_lines[i].b);
        }

        // to_polygons of expolygons: contour then holes, expolygon after expolygon
        const ExPolygons expolygons = union_ex(scene);
        CHECK(SG::native(TF::to_polygons(storage, view_of(expolygons))) == to_polygons(expolygons));
    }

    SECTION("empty collection")
    {
        const Polygons none;
        const ExPolygons no_expolygons;
        CHECK(TF::polygons_area(view_of(none)) == 0.);
        CHECK(TF::total_length(view_of(none)) == 0.);
        CHECK(TF::to_polylines(storage, view_of(none)).empty());
        CHECK(TF::to_lines(view_of(none)).empty());
        CHECK(TF::to_polygons(storage, view_of(no_expolygons)).empty());
    }
    SECTION("degenerate rings")
    {
        // A ring of two points has two lines and twice the segment length; a ring without points gives nothing. (The native
        // to_lines() reads before the start of an empty ring, and a ring of one point cannot be held in native Polygons without
        // tripping their assertions: those two cases are not compared.)
        const Polygons two{ Polygon({ P(0, 0), P(30, 40) }) };
        CHECK(TF::total_length(view_of(two)) == total_length(two));
        CHECK(TF::total_length(view_of(two)) == 100000.);
        const std::vector<TF::LineSegment> lines = TF::to_lines(view_of(two));
        const Lines native = to_lines(two);
        REQUIRE(lines.size() == 2);
        REQUIRE(native.size() == 2);
        CHECK(ST::to_point(lines[0].a) == native[0].a);
        CHECK(ST::to_point(lines[1].b) == native[1].b);
        const Polygons none_points{ Polygon(), Polygon({ P(0, 0), P(10, 0), P(10, 10) }) };
        CHECK(TF::to_lines(view_of(none_points)).size() == 3);
        CHECK(TF::to_polylines(storage, view_of(none_points)).size() == 1);
        CHECK(TF::total_length(view_of(none_points)) == total_length(Polygons{ none_points[1] }));
    }
    SECTION("clipper_offset_polylines, the API function used with to_polylines, gives the native offset of polylines")
    {
        const Polygons scene = random_scene(rng);
        const Polylines closed = to_polylines(scene);
        const Polylines from_helper = SG::native(TF::to_polylines(storage, view_of(scene)));
        const Polygons native = offset(closed, scale_d(0.002), ClipperLib::jtMiter, 1.2);
        const slic3r_api::StoredPolylineCollection helper_polylines = TF::to_polylines(storage, view_of(scene));
        const Polygons api = SG::native(slic3r_api::clipper_offset_polylines(storage, helper_polylines, scale_d(0.002), CLIPPER_JOIN_MITER, 1.2)
                                           .to_polygon_collection());
        CHECK(from_helper.size() == closed.size());
        CHECK(PM::sym_diff_mm2(api, native) < max_sym_diff_mm2);
        CHECK(!api.empty());
    }
}

TEST_CASE("point_in_polygons matches the native contains on random points and on borders", "[plugins][tree-geometry]")
{
    std::mt19937 rng(5);
    size_t border_cases = 0, inside_cases = 0, outside_cases = 0;
    auto compare = [&](const Polygons &polygons, const Point &point) {
        const slic3r_api::PolygonCollection view = view_of(polygons);
        for (const bool border_result : { true, false }) {
            const bool native = contains(polygons, point, border_result);
            CHECK(TF::point_in_polygons(view, ST::to_c_point(point), border_result) == native);
        }
        // Is the point on a border? With a different border_result the answers differ exactly then.
        if (contains(polygons, point, true) != contains(polygons, point, false))
            ++border_cases;
        else if (contains(polygons, point, true))
            ++inside_cases;
        else
            ++outside_cases;
    };
    for (int iteration = 0; iteration < 40; ++iteration) {
        const Polygons scene = random_scene(rng);
        const BoundingBox box = get_extents(scene);
        for (int i = 0; i < 100; ++i)
            compare(scene, random_point(rng, box, 2.));
        // The vertices, and the midpoints of the edges (on the border when the edge has an even delta), of every ring.
        for (const Polygon &ring : scene)
            for (size_t i = 0; i < ring.size(); ++i) {
                compare(scene, ring[i]);
                compare(scene, (ring[i] + ring[(i + 1) % ring.size()]) / 2);
            }
    }
    CHECK(border_cases > 1000);
    CHECK(inside_cases > 100);
    CHECK(outside_cases > 100);

    SECTION("a square with a square hole, border points of both rings")
    {
        Polygon outer({ P(0, 0), P(100, 0), P(100, 100), P(0, 100) });
        Polygon hole({ P(40, 40), P(40, 60), P(60, 60), P(60, 40) });
        outer.make_counter_clockwise();
        hole.make_clockwise();
        const Polygons polygons{ outer, hole };
        for (const Point &point : { P(10, 10), P(50, 50), P(0, 0), P(0, 50), P(100, 100), P(50, 100), P(40, 50),
                                    P(60, 60), P(40, 40), P(-1, 50), P(101, 50), P(50, 39), P(50, 40) })
            compare(polygons, point);
        // Inside the contour but outside the hole is inside; inside the hole is outside.
        CHECK(TF::point_in_polygons(view_of(polygons), CP(10, 10)));
        CHECK(!TF::point_in_polygons(view_of(polygons), CP(50, 50)));
        // The border follows border_result.
        CHECK(TF::point_in_polygons(view_of(polygons), CP(0, 50), true));
        CHECK(!TF::point_in_polygons(view_of(polygons), CP(0, 50), false));
        CHECK(TF::point_in_polygons(view_of(polygons), CP(40, 50), true));
        CHECK(!TF::point_in_polygons(view_of(polygons), CP(40, 50), false));
    }
    SECTION("diagonal edges: points on the edge, at the vertex height and beside the edge")
    {
        const Polygons diamond{ Polygon({ P(0, -100), P(100, 0), P(0, 100), P(-100, 0) }) };
        for (coord_t y = -110; y <= 110; y += 5)
            for (coord_t x = -110; x <= 110; x += 5)
                compare(diamond, P(x, y));
    }
    SECTION("overlapping rings count twice, like the native parity")
    {
        const Polygons overlapping{ Polygon({ P(0, 0), P(100, 0), P(100, 100), P(0, 100) }),
                                    Polygon({ P(50, 50), P(150, 50), P(150, 150), P(50, 150) }) };
        for (const Point &point : { P(25, 25), P(75, 75), P(125, 125), P(75, 25), P(50, 50), P(150, 150), P(200, 200) })
            compare(overlapping, point);
        CHECK(!TF::point_in_polygons(view_of(overlapping), CP(75, 75)));
    }
    SECTION("empty collection, rings under 3 points")
    {
        const Polygons none;
        CHECK(!TF::point_in_polygons(view_of(none), CP(0, 0)));
        CHECK(!contains(none, P(0, 0), true));
        const Polygons short_rings{ Polygon(), Polygon({ P(0, 0), P(10, 10) }) };
        compare(short_rings, P(0, 0));
        compare(short_rings, P(5, 5));
        CHECK(!TF::point_in_polygons(view_of(short_rings), CP(5, 5)));
        // A short ring does not hide a real one.
        const Polygons mixed{ Polygon({ P(0, 0), P(10, 10) }), Polygon({ P(0, 0), P(100, 0), P(100, 100), P(0, 100) }) };
        compare(mixed, P(50, 50));
        CHECK(TF::point_in_polygons(view_of(mixed), CP(50, 50)));
    }
}

TEST_CASE("nearest_on_lines matches the native indexed lines distance", "[plugins][tree-geometry]")
{
    std::mt19937 rng(6);
    std::uniform_real_distribution<double> unit(0., 1.);
    size_t hits = 0, misses = 0, other_line = 0;
    double worst_distance_diff = 0., worst_point_diff = 0.;

    // One comparison: the native tree and the helper, built on the same lines.
    auto compare = [&](const std::vector<TF::LineSegment> &lines, const slic3r_api::LineDistancer &index, const Vec2d &from, double max_squared_distance,
                       bool expect_same_point) {
        std::vector<Linef> native_lines;
        for (const TF::LineSegment &line : lines)
            native_lines.push_back(Linef(Vec2d(double(line.a.x), double(line.a.y)), Vec2d(double(line.b.x), double(line.b.y))));
        const AABBTreeIndirect::Tree<2, double> tree = AABBTreeLines::build_aabb_tree_over_indexed_lines(native_lines);
        size_t native_index = 0;
        Vec2d native_point = Vec2d::Zero();
        const double native_distance = AABBTreeLines::squared_distance_to_indexed_lines(native_lines, tree, from, native_index, native_point, max_squared_distance);
        const std::optional<TF::LineHit> mine = TF::nearest_on_lines(index, lines, from, max_squared_distance);
        if (native_lines.empty()) {
            CHECK(!mine.has_value());
            return;
        }
        // The native function returns the limit itself when no line is strictly closer.
        if (!(native_distance < max_squared_distance)) {
            CHECK(!mine.has_value());
            ++misses;
            return;
        }
        REQUIRE(mine.has_value());
        ++hits;
        worst_distance_diff = std::max(worst_distance_diff, std::abs(mine->squared_distance - native_distance));
        worst_point_diff = std::max(worst_point_diff, (mine->point - native_point).norm());
        CHECK(mine->squared_distance == native_distance);
        // The nearest point is the native one. The line can differ when two lines are as close, which happens when the nearest point is
        // a vertex shared by two consecutive lines: the first line wins here, the native tree keeps the one it meets first.
        if (expect_same_point)
            CHECK(mine->point == native_point);
        if (mine->line_index != native_index)
            ++other_line;
    };

    SECTION("random scenes, query points and limits")
    {
        for (int iteration = 0; iteration < 30; ++iteration) {
            const Polygons scene = random_scene(rng);
            const std::vector<TF::LineSegment> lines = TF::to_lines(view_of(scene));
            slic3r_api::LineDistancer index;
            for (const TF::LineSegment &line : lines)
                index.append_segment(line.a, line.b);
            index.build_index();
            slic3r_api::LineDistancer unbuilt;
            for (const TF::LineSegment &line : lines)
                unbuilt.append_segment(line.a, line.b);
            const BoundingBox box = get_extents(scene);
            for (int i = 0; i < 20; ++i) {
                // A query point between integer positions, inside and around the scene.
                const Point base = random_point(rng, box, 3.);
                const Vec2d from = base.cast<double>() + Vec2d(unit(rng), unit(rng));
                for (const double max_distance_mm : { 0.01, 0.5, 5., 50. }) {
                    const double max_squared = sqr(scale_d(max_distance_mm));
                    compare(lines, index, from, max_squared, true);
                    // The linear scan of an index that was not built gives the same answer.
                    compare(lines, unbuilt, from, max_squared, true);
                }
                compare(lines, index, from, std::numeric_limits<double>::infinity(), true);
            }
        }
        INFO("nearest_on_lines: " << hits << " hits, " << misses << " misses, largest squared distance difference " << worst_distance_diff
                                  << ", largest hit point difference " << worst_point_diff << " (scaled units), " << other_line << " hits on another line than the native one (ties)");
        CHECK(hits > 1000);
        CHECK(misses > 100);
    }
    SECTION("a distance exactly at the limit is not a hit, as in the native function")
    {
        const std::vector<TF::LineSegment> lines{ { c_point{ 0, 0 }, c_point{ 100, 0 } } };
        slic3r_api::LineDistancer index;
        index.append_segment(lines[0].a, lines[0].b);
        index.build_index();
        const Vec2d from(50., 30.);
        CHECK(!TF::nearest_on_lines(index, lines, from, 900.).has_value());
        compare(lines, index, from, 900., true);
        const std::optional<TF::LineHit> hit = TF::nearest_on_lines(index, lines, from, 900.0001);
        REQUIRE(hit.has_value());
        CHECK(hit->squared_distance == 900.);
        CHECK(hit->point == Vec2d(50., 0.));
    }
    SECTION("a tie: the first line of the vector wins")
    {
        // Two parallel lines at the same distance of the origin, in both orders.
        const TF::LineSegment up{ c_point{ -10, 10 }, c_point{ 10, 10 } };
        const TF::LineSegment down{ c_point{ -10, -10 }, c_point{ 10, -10 } };
        for (const std::vector<TF::LineSegment> &lines : { std::vector<TF::LineSegment>{ up, down }, std::vector<TF::LineSegment>{ down, up } }) {
            slic3r_api::LineDistancer index;
            for (const TF::LineSegment &line : lines)
                index.append_segment(line.a, line.b);
            index.build_index();
            const std::optional<TF::LineHit> hit = TF::nearest_on_lines(index, lines, Vec2d::Zero());
            REQUIRE(hit.has_value());
            CHECK(hit->line_index == 0);
            CHECK(hit->squared_distance == 100.);
            // The distance is the native one whichever line the native tree keeps.
            compare(lines, index, Vec2d::Zero(), std::numeric_limits<double>::infinity(), false);
        }
    }
    SECTION("zero-length segment, point on a line, end points")
    {
        const std::vector<TF::LineSegment> lines{ { c_point{ 7, 7 }, c_point{ 7, 7 } }, { c_point{ 0, 0 }, c_point{ 100, 0 } },
                                                  { c_point{ 100, 0 }, c_point{ 100, 100 } } };
        slic3r_api::LineDistancer index;
        for (const TF::LineSegment &line : lines)
            index.append_segment(line.a, line.b);
        index.build_index();
        for (const Vec2d &from : { Vec2d(7., 7.), Vec2d(50., 0.), Vec2d(-20., -20.), Vec2d(100., 0.), Vec2d(130.5, 50.5), Vec2d(100., 100.), Vec2d(3.25, 4.5) })
            compare(lines, index, from, std::numeric_limits<double>::infinity(), true);
        const std::optional<TF::LineHit> on_line = TF::nearest_on_lines(index, lines, Vec2d(50., 0.));
        REQUIRE(on_line.has_value());
        CHECK(on_line->squared_distance == 0.);
        CHECK(on_line->line_index == 1);
    }
    SECTION("no lines")
    {
        const std::vector<TF::LineSegment> lines;
        slic3r_api::LineDistancer index;
        index.build_index();
        CHECK(!TF::nearest_on_lines(index, lines, Vec2d(1., 1.)).has_value());
        CHECK(!TF::nearest_on_lines(index, lines, Vec2d(1., 1.), 1e12).has_value());
    }
}

TEST_CASE("spatial_order gives the exact order of the native AABB tree", "[plugins][tree-geometry]")
{
    std::mt19937 rng(7);
    SECTION("sizes from 0 to 70, spread boxes")
    {
        for (size_t count = 0; count <= 70; ++count)
            for (int repeat = 0; repeat < 5; ++repeat)
                CHECK(same_order(random_boxes(rng, count, 1000000, 500000, 1)));
    }
    SECTION("many ties: small grid, equal centroids, zero-size boxes")
    {
        for (size_t count : { 2, 3, 4, 5, 8, 17, 64, 200 })
            for (const coord_t range : { 1, 2, 5 })
                for (int repeat = 0; repeat < 10; ++repeat)
                    CHECK(same_order(random_boxes(rng, count, range, 2, 3)));
    }
    SECTION("identical boxes")
    {
        for (size_t count : { 1, 2, 3, 10, 33 })
            CHECK(same_order(std::vector<c_bounding_box>(count, c_bounding_box{ c_point{ 5, 5 }, c_point{ 9, 9 } })));
    }
    SECTION("boxes in a line along one axis, then the other, with negative coordinates")
    {
        std::vector<c_bounding_box> along_x, along_y;
        for (coord_t i = 0; i < 40; ++i) {
            along_x.push_back(c_bounding_box{ c_point{ (i * 37) % 41 - 20, 0 }, c_point{ (i * 37) % 41 - 18, 1 } });
            along_y.push_back(c_bounding_box{ c_point{ 0, (i * 29) % 43 - 21 }, c_point{ 1, (i * 29) % 43 - 19 } });
        }
        CHECK(same_order(along_x));
        CHECK(same_order(along_y));
    }
    SECTION("sorted and reverse sorted input")
    {
        std::vector<c_bounding_box> boxes = random_boxes(rng, 100, 1000, 10, 100);
        std::sort(boxes.begin(), boxes.end(), [](const c_bounding_box &a, const c_bounding_box &b) { return std::make_pair(a.min.x, a.min.y) < std::make_pair(b.min.x, b.min.y); });
        CHECK(same_order(boxes));
        std::reverse(boxes.begin(), boxes.end());
        CHECK(same_order(boxes));
    }
    SECTION("the order really changes the input")
    {
        std::vector<ApiItem> items;
        for (size_t i = 0; i < 16; ++i)
            items.push_back(ApiItem{ i, c_bounding_box{ c_point{ coord_t((i * 7) % 16) * 100, 0 }, c_point{ coord_t((i * 7) % 16) * 100 + 10, 10 } } });
        const std::vector<ApiItem> before = items;
        TF::spatial_order(items, [](const ApiItem &item) { return item.box; });
        bool moved = false;
        for (size_t i = 0; i < items.size(); ++i)
            moved = moved || items[i].id != before[i].id;
        CHECK(moved);
        // The first half holds the left boxes.
        for (size_t i = 0; i < 8; ++i)
            CHECK(items[i].box.min.x < 800);
    }
    SECTION("empty input")
    {
        std::vector<ApiItem> none;
        TF::spatial_order(none, [](const ApiItem &item) { return item.box; });
        CHECK(none.empty());
    }
}

namespace {
// Compare locally owned deterministic scenes with native operations, reporting failed checks and returning the largest differences as text.
std::string run_clipper_wrappers()
{
    std::string report;
    std::mt19937 rng(8);
    PluginStorage owner;
    storage_handle *storage = SG::handle(owner);
    Largest unions, diffs, intersections, offsets, offsets_round, expolygon_ops, expolygon_offsets, clips, diff_clippeds;
    SECTION("random scenes")
    {
        for (int iteration = 0; iteration < 40; ++iteration) {
            const Polygons a = random_scene(rng);
            const Polygons b = random_scene(rng);
            const ExPolygons a_ex = union_ex(a);
            const ExPolygons b_ex = union_ex(b);
            const slic3r_api::PolygonCollection a_view = view_of(a), b_view = view_of(b);
            const Polygons clean = union_(a);

            check_polygons(SG::native(TF::polygons_union(storage, a_view)), clean, unions);
            check_polygons(SG::native(TF::polygons_union(storage, a_view, b_view)), union_(a, b), unions);
            check_polygons(SG::native(TF::polygons_union(storage, view_of(a_ex))), union_(a_ex), unions);
            check_polygons(SG::native(TF::polygons_diff(storage, a_view, b_view)), diff(a, b), diffs);
            check_polygons(SG::native(TF::polygons_diff(storage, a_view, view_of(b_ex))), diff(a, b_ex), diffs);
            check_polygons(SG::native(TF::polygons_intersection(storage, a_view, b_view)), intersection(a, b), intersections);
            check_polygons(SG::native(TF::polygons_intersection(storage, view_of(a_ex), view_of(b_ex))), intersection(a_ex, b_ex), intersections);

            // The rings of a without overlap (a union), the counter-clockwise rings of a alone (overlapping, no hole), and expolygons.
            Polygons ccw_only;
            for (const Polygon &ring : a)
                if (ring.is_counter_clockwise())
                    ccw_only.push_back(ring);
            // Expolygons that overlap: each counter-clockwise ring of a alone, none united.
            ExPolygons overlapping_ex;
            for (const Polygon &ring : ccw_only)
                overlapping_ex.emplace_back(ring);
            for (const double delta_mm : { 0., 0.005, 0.3, 2., -0.005, -0.3, -1.5, -30. }) {
                const double delta = scale_d(delta_mm);
                INFO("delta " << delta_mm << " mm, iteration " << iteration);
                // Miter joins, then round joins with the arc tolerance of the volumes (passed in the miter_limit slot). Same region and same
                // ring count as the native function in every case below. A delta of 0 is only for the expolygons (the ring lists
                // assert on it, natively too).
                if (delta != 0.) {
                    check_polygons(SG::native(TF::polygons_offset(storage, view_of(clean), delta, CLIPPER_JOIN_MITER, 1.2)), offset(clean, delta, ClipperLib::jtMiter, 1.2), offsets);
                    check_polygons(SG::native(TF::polygons_offset(storage, view_of(clean), delta, CLIPPER_JOIN_ROUND, scale_d(0.01))),
                                   offset(clean, delta, ClipperLib::jtRound, scale_d(0.01)), offsets_round);
                }
                // Overlapping rings are supported for a positive delta when no ring is clockwise.
                if (delta > 0) {
                    check_polygons(SG::native(TF::polygons_offset(storage, view_of(ccw_only), delta, CLIPPER_JOIN_MITER, 1.2)), offset(ccw_only, delta, ClipperLib::jtMiter, 1.2), offsets);
                    check_polygons(SG::native(TF::polygons_offset(storage, view_of(ccw_only), delta, CLIPPER_JOIN_ROUND, scale_d(0.01))),
                                   offset(ccw_only, delta, ClipperLib::jtRound, scale_d(0.01)), offsets_round);
                }
                // Expolygons are supported for both signs, overlapping ones for a positive delta (the native function documents that they
                // must not overlap: for a negative delta it keeps each shrunk expolygon, the API unites them). A delta of 0 is tested on
                // the expolygons that do not overlap.
                for (const ExPolygons *expolygons : std::vector<const ExPolygons *>{ &a_ex, &overlapping_ex }) {
                    if (expolygons == &overlapping_ex && delta <= 0)
                        continue;
                    check_polygons(SG::native(TF::polygons_offset(storage, view_of(*expolygons), delta, CLIPPER_JOIN_MITER, 1.2)),
                                   offset(*expolygons, delta, ClipperLib::jtMiter, 1.2), offsets);
                    check_polygons(SG::native(TF::polygons_offset(storage, view_of(*expolygons), delta, CLIPPER_JOIN_ROUND, scale_d(0.01))),
                                   offset(*expolygons, delta, ClipperLib::jtRound, scale_d(0.01)), offsets_round);
                    check_expolygons(SG::native(TF::expolygons_offset(storage, view_of(*expolygons), delta, CLIPPER_JOIN_MITER, 1.2)),
                                     offset_ex(*expolygons, delta, ClipperLib::jtMiter, 1.2), expolygon_offsets);
                    check_expolygons(SG::native(TF::expolygons_offset(storage, view_of(*expolygons), delta, CLIPPER_JOIN_ROUND, scale_d(0.01))),
                                     offset_ex(*expolygons, delta, ClipperLib::jtRound, scale_d(0.01)), expolygon_offsets);
                }
            }

            check_expolygons(SG::native(TF::expolygons_union(storage, a_view)), a_ex, expolygon_ops);
            check_expolygons(SG::native(TF::expolygons_union(storage, view_of(a_ex))), union_ex(a_ex), expolygon_ops);
            check_expolygons(SG::native(TF::expolygons_diff(storage, a_view, b_view)), diff_ex(a, b), expolygon_ops);
            check_expolygons(SG::native(TF::expolygons_diff(storage, view_of(a_ex), view_of(b_ex))), diff_ex(a_ex, b_ex), expolygon_ops);

            // The rings that can touch a small box: compared by the region they leave after a union (the rings are not merged).
            const BoundingBox box = get_extents(a);
            const BoundingBox small(box.min + Point(scale_i(8.), scale_i(8.)), box.min + Point(scale_i(14.), scale_i(14.)));
            check_polygons(SG::native(TF::polygons_union(storage, TF::clip_polygons_with_subject_bbox(storage, b_view, c_box_of(small)))),
                           union_(ClipperUtils::clip_clipper_polygons_with_subject_bbox(b, small)), clips);
            // A small subject against a large clip, and the large one against the small, as the routing does.
            const Polygons small_subject{ PM::random_star(rng, Point(box.min.x() + scale_i(10.), box.min.y() + scale_i(10.)), 3., 20, star_min_fraction) };
            check_polygons(SG::native(TF::diff_clipped(storage, view_of(small_subject), b_view)), diff_clipped(small_subject, b), diff_clippeds);
            check_polygons(SG::native(TF::diff_clipped(storage, a_view, b_view)), diff_clipped(a, b), diff_clippeds);
        }
        std::ostringstream text;
        text << "Clipper wrappers, largest symmetric difference (mm2) / rings identical to the native ones:"
             << "\n  union " << unions.sym_diff << " / " << unions.identical_rings << " of " << unions.compared
             << "\n  diff " << diffs.sym_diff << " / " << diffs.identical_rings << " of " << diffs.compared
             << "\n  intersection " << intersections.sym_diff << " / " << intersections.identical_rings << " of " << intersections.compared
             << "\n  offset miter " << offsets.sym_diff << " / " << offsets.identical_rings << " of " << offsets.compared
             << "\n  offset round " << offsets_round.sym_diff << " / " << offsets_round.identical_rings << " of " << offsets_round.compared
             << "\n  union_ex, diff_ex " << expolygon_ops.sym_diff << " / " << expolygon_ops.identical_rings << " of " << expolygon_ops.compared
             << "\n  offset_ex " << expolygon_offsets.sym_diff << " / " << expolygon_offsets.identical_rings << " of " << expolygon_offsets.compared
             << "\n  clip with bbox " << clips.sym_diff << " / " << clips.identical_rings << " of " << clips.compared
             << "\n  diff_clipped " << diff_clippeds.sym_diff << " / " << diff_clippeds.identical_rings << " of " << diff_clippeds.compared;
        report = text.str();
    }

    SECTION("empty subject and clip")
    {
        const Polygons scene = random_scene(rng);
        const Polygons none;
        const ExPolygons no_expolygons;
        CHECK(TF::polygons_union(storage, view_of(none)).empty());
        CHECK(TF::polygons_diff(storage, view_of(none), view_of(scene)).empty());
        CHECK(TF::polygons_intersection(storage, view_of(none), view_of(scene)).empty());
        CHECK(TF::polygons_intersection(storage, view_of(scene), view_of(none)).empty());
        CHECK(TF::polygons_offset(storage, view_of(none), scale_d(1.)).empty());
        CHECK(TF::expolygons_union(storage, view_of(none)).empty());
        CHECK(TF::expolygons_diff(storage, view_of(none), view_of(scene)).empty());
        CHECK(TF::expolygons_offset(storage, view_of(no_expolygons), scale_d(1.)).empty());
        CHECK(TF::diff_clipped(storage, view_of(none), view_of(scene)).empty());
        // An empty clip leaves the region of the subject.
        Largest ignored;
        check_polygons(SG::native(TF::polygons_diff(storage, view_of(scene), view_of(none))), diff(scene, none), ignored);
        check_polygons(SG::native(TF::diff_clipped(storage, view_of(scene), view_of(none))), diff_clipped(scene, none), ignored);
        check_polygons(SG::native(TF::clip_polygons_with_subject_bbox(storage, view_of(none), c_box_of(get_extents(scene)))), Polygons{}, ignored);
        // A union of nothing with something.
        check_polygons(SG::native(TF::polygons_union(storage, view_of(none), view_of(scene))), union_(none, scene), ignored);
    }
    SECTION("shrinking past nothing, and shapes far from the clip")
    {
        const Polygons square{ Polygon({ Point(0, 0), Point(scale_i(2.), 0), Point(scale_i(2.), scale_i(2.)), Point(0, scale_i(2.)) }) };
        CHECK(SG::native(TF::polygons_offset(storage, view_of(square), -scale_d(5.))).empty());
        CHECK(offset(square, -scale_d(5.)).empty());
        const Polygons far_clip{ Polygon({ Point(scale_i(100.), 0), Point(scale_i(102.), 0), Point(scale_i(102.), scale_i(2.)), Point(scale_i(100.), scale_i(2.)) }) };
        Largest ignored;
        check_polygons(SG::native(TF::diff_clipped(storage, view_of(square), view_of(far_clip))), diff_clipped(square, far_clip), ignored);
        CHECK(!SG::native(TF::diff_clipped(storage, view_of(square), view_of(far_clip))).empty());
    }
    return report;
}
} // namespace

TEST_CASE("tree geometry Clipper wrappers match the native operations", "[plugins][tree-geometry]")
{
    run_clipper_wrappers();
}

TEST_CASE("tree geometry operators coexist with the native ones", "[plugins][tree-geometry]")
{
    // The function is compiled inside Slic3r::TreeFamily, where the c_point operators are declared: it uses native Point, Polygon,
    // ExPolygon, BoundingBox, Line, std::pair and std::vector comparisons and c_point arithmetic side by side.
    CHECK(TF::native_and_c_point_operators_coexist());
}

namespace {
// The checks of the winding edge cases; returns the largest differences with the native result, for the report case.
std::string run_winding_edge_cases()
{
    std::mt19937 rng(9);
    PluginStorage owner;
    storage_handle *storage = SG::handle(owner);
    Largest offsets, booleans;
    const Polygons other = random_scene(rng);
    std::string unsupported_report;
    for (const WindingScene &scene : winding_scenes(rng)) {
        INFO("scene: " << scene.name);
        const slic3r_api::PolygonCollection view = view_of(scene.rings);
        bool counter_clockwise_only = true;
        for (const Polygon &ring : scene.rings)
            counter_clockwise_only = counter_clockwise_only && ring.is_counter_clockwise();
        double unsupported_worst = 0.;
        for (const double delta_mm : { 0.05, 0.5, 2., -0.05, -0.5, -2. }) {
            const double delta = scale_d(delta_mm);
            INFO("delta " << delta_mm << " mm");
            const Polygons mine_miter = SG::native(TF::polygons_offset(storage, view, delta, CLIPPER_JOIN_MITER, 1.2));
            const Polygons mine_round = SG::native(TF::polygons_offset(storage, view, delta, CLIPPER_JOIN_ROUND, scale_d(0.01)));
            const Polygons native_miter = offset(scene.rings, delta, ClipperLib::jtMiter, 1.2);
            const Polygons native_round = offset(scene.rings, delta, ClipperLib::jtRound, scale_d(0.01));
            // Supported: clean lists with both signs, counter-clockwise rings (overlapping or not) with a positive delta.
            if (scene.clean || (counter_clockwise_only && delta > 0)) {
                check_polygons(mine_miter, native_miter, offsets);
                check_polygons(mine_round, native_round, offsets);
            } else {
                // Documented as unsupported: measured, not compared.
                unsupported_worst = std::max({ unsupported_worst, PM::sym_diff_mm2(mine_miter, native_miter), PM::sym_diff_mm2(mine_round, native_round) });
            }
        }
        if (!scene.clean)
            unsupported_report += "\n  " + scene.name + ": " + std::to_string(unsupported_worst) + " mm2";
        // The boolean operations are exact whatever the windings.
        check_polygons(SG::native(TF::polygons_union(storage, view)), union_(scene.rings), booleans);
        check_polygons(SG::native(TF::polygons_diff(storage, view, view_of(other))), diff(scene.rings, other), booleans);
        check_polygons(SG::native(TF::polygons_diff(storage, view_of(other), view)), diff(other, scene.rings), booleans);
        check_polygons(SG::native(TF::polygons_intersection(storage, view, view_of(other))), intersection(scene.rings, other), booleans);
        check_expolygons(SG::native(TF::expolygons_union(storage, view)), union_ex(scene.rings), booleans);
        check_expolygons(SG::native(TF::expolygons_diff(storage, view, view_of(other))), diff_ex(scene.rings, other), booleans);
    }
    std::ostringstream text;
    text << "winding edge cases, largest symmetric difference (mm2): supported offsets " << offsets.sym_diff << " (" << offsets.compared << " compared, "
         << offsets.identical_rings << " with identical rings), union/diff/intersection " << booleans.sym_diff << " (" << booleans.compared
         << " compared)\nUnsupported offsets (measured, not compared), largest difference per scene:" << unsupported_report;
    return text.str();
}
} // namespace

TEST_CASE("tree geometry Clipper wrappers match the native operations on winding edge cases", "[plugins][tree-geometry]")
{
    run_winding_edge_cases();
}

namespace {
// A grid of count_x * count_y one-millimeter squares 3 mm apart, each with a clockwise hole of 0.4 mm when with_holes.
Polygons square_grid(size_t count_x, size_t count_y, bool with_holes)
{
    Polygons rings;
    for (size_t j = 0; j < count_y; ++j)
        for (size_t i = 0; i < count_x; ++i) {
            const coord_t x = scale_i(3. * double(i)), y = scale_i(3. * double(j));
            rings.push_back(Polygon({ Point(x, y), Point(x + scale_i(1.), y), Point(x + scale_i(1.), y + scale_i(1.)), Point(x, y + scale_i(1.)) }));
            if (with_holes) {
                const coord_t h = scale_i(0.3);
                Polygon hole({ Point(x + h, y + h), Point(x + h, y + h + scale_i(0.4)), Point(x + h + scale_i(0.4), y + h + scale_i(0.4)), Point(x + h + scale_i(0.4), y + h) });
                rings.push_back(hole);
            }
        }
    return rings;
}

template<typename Fn> double best_of_three_ms(Fn &&fn)
{
    double best = std::numeric_limits<double>::max();
    for (int run = 0; run < 3; ++run) {
        const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
        fn();
        best = std::min(best, std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count());
    }
    return best;
}

std::vector<LargeCase> large_cases()
{
    std::vector<LargeCase> cases;
    cases.push_back({ "2000 squares, outward (flat route)", square_grid(50, 40, false), 0.3 });
    cases.push_back({ "2000 squares with holes (4000 rings), outward (union route)", square_grid(50, 40, true), 0.3 });
    cases.push_back({ "2000 squares with holes (4000 rings), inward (union route)", square_grid(50, 40, true), -0.1 });
    return cases;
}
} // namespace

namespace {
// The checks of the large inputs; returns the largest difference with the native result, for the report case.
std::string run_large_inputs()
{
    PluginStorage owner;
    storage_handle *storage = SG::handle(owner);
    Largest largest;
    for (const LargeCase &entry : large_cases()) {
        INFO(entry.name);
        const double delta = scale_d(entry.delta_mm);
        check_polygons(SG::native(TF::polygons_offset(storage, view_of(entry.rings), delta, CLIPPER_JOIN_ROUND, scale_d(0.01))),
                       offset(entry.rings, delta, ClipperLib::jtRound, scale_d(0.01)), largest);
    }
    std::ostringstream text;
    text << "large inputs, largest symmetric difference with the native result " << largest.sym_diff << " mm2 over " << largest.compared << " comparisons";
    return text.str();
}
} // namespace

TEST_CASE("tree geometry polygons_offset on a large input: result", "[plugins][tree-geometry]")
{
    run_large_inputs();
}

// The largest differences with the native result that the cases above measure (they are shown nowhere else, since a passing case prints
// nothing); hidden, run it by its tag: plugins_tests "[tree-geometry-report]". It repeats the checks of those cases.
TEST_CASE("tree geometry report of the largest differences with the native operations", "[.][tree-geometry-report]")
{
    // The sections of the first run make Catch2 run the case again: each report is printed once.
    static std::set<std::string> printed;
    for (const std::string &report : { run_clipper_wrappers(), run_winding_edge_cases(), run_large_inputs() })
        if (!report.empty() && printed.insert(report).second)
            WARN(report);
}

// Cost of polygons_offset on the large inputs; hidden, run it by its tag: plugins_tests "[tree-geometry-benchmark]".
TEST_CASE("tree geometry polygons_offset cost on a large input", "[.][tree-geometry-benchmark]")
{
    PluginStorage owner;
    storage_handle *storage = SG::handle(owner);
    std::string report;
    for (const LargeCase &entry : large_cases()) {
        const slic3r_api::PolygonCollection view = view_of(entry.rings);
        const double delta = scale_d(entry.delta_mm);
        const slic3r_api::ClipperContext clipper(storage);
        // The helper, one global API offset of the same rings (not the native result), and the native function.
        const double helper_ms = best_of_three_ms([&] { TF::polygons_offset(storage, view, delta, CLIPPER_JOIN_ROUND, scale_d(0.01)); });
        const double global_ms = best_of_three_ms([&] { slic3r_api::clipper_offset(clipper(view), delta, CLIPPER_JOIN_ROUND, scale_d(0.01)).to_polygon_collection(); });
        const double native_ms = best_of_three_ms([&] { offset(entry.rings, delta, ClipperLib::jtRound, scale_d(0.01)); });
        report += std::string("\n  ") + entry.name + ": helper " + std::to_string(helper_ms) + " ms, one global API offset " + std::to_string(global_ms) +
                  " ms, native " + std::to_string(native_ms) + " ms";
    }
    WARN("polygons_offset cost (best of three, Debug build):" << report);
}

TEST_CASE("tree geometry polygons_offset branches: a lone ring, a list with a short ring", "[plugins][tree-geometry]")
{
    std::mt19937 rng(12);
    PluginStorage owner;
    storage_handle *storage = SG::handle(owner);
    Largest largest;
    // A lone counter-clockwise ring, grown and shrunk (the positive route and the union route with one ring).
    const Polygons lone{ PM::random_star(rng, Point(0, 0), 10., 30, star_min_fraction) };
    // A ring of two points (not an area, ignored) next to a valid ring.
    const Polygons mixed{ Polygon({ P(0, 0), P(30, 40) }), PM::random_star(rng, Point(scale_i(50.), 0), 8., 20, star_min_fraction) };
    for (const Polygons *rings : { &lone, &mixed })
        for (const double delta_mm : { 0.3, -0.3 }) {
            INFO("rings " << rings->size() << ", delta " << delta_mm << " mm");
            const double delta = scale_d(delta_mm);
            check_polygons(SG::native(TF::polygons_offset(storage, view_of(*rings), delta, CLIPPER_JOIN_MITER, 1.2)), offset(*rings, delta, ClipperLib::jtMiter, 1.2), largest);
            check_polygons(SG::native(TF::polygons_offset(storage, view_of(*rings), delta, CLIPPER_JOIN_ROUND, scale_d(0.01))),
                           offset(*rings, delta, ClipperLib::jtRound, scale_d(0.01)), largest);
        }
    // Only short rings: nothing to offset.
    const Polygons only_short{ Polygon({ P(0, 0), P(30, 40) }) };
    INFO("lone ring and list with a short ring: largest symmetric difference " << largest.sym_diff << " mm2 over " << largest.compared << " comparisons");
    CHECK(TF::polygons_offset(storage, view_of(only_short), scale_d(0.3)).empty());
}

TEST_CASE("tree geometry expolygons_offset keeps the small areas like offset_ex", "[plugins][tree-geometry]")
{
    PluginStorage owner;
    storage_handle *storage = SG::handle(owner);
    Largest largest;
    // A thin parallelogram whose area is under the threshold of remove_small_areas(): offset_ex() keeps it, union_ex() would not.
    const ExPolygons parallelogram{ ExPolygon(Polygon({ Point(0, 0), Point(10000000, 0), Point(10500000, 136), Point(500000, 136) })) };
    // A rectangle shrunk to a residue of a few hundred units wide (the width of the residue is 500000 + 2 * delta): around the same threshold.
    const ExPolygons strip{ ExPolygon(Polygon({ Point(0, 0), Point(10000000, 0), Point(10000000, 500000), Point(0, 500000) })) };
    const std::vector<std::pair<const ExPolygons *, double>> cases{ { &parallelogram, 0. }, { &parallelogram, 1000. }, { &strip, -249900. },
                                                                       { &strip, -249940. }, { &strip, -249960. }, { &strip, -100000. } };
    size_t kept_small = 0;
    for (const std::pair<const ExPolygons *, double> &entry : cases) {
        INFO("delta " << entry.second);
        const ExPolygons native = offset_ex(*entry.first, entry.second, ClipperLib::jtMiter, 1.2);
        const ExPolygons mine = SG::native(TF::expolygons_offset(storage, view_of(*entry.first), entry.second, CLIPPER_JOIN_MITER, 1.2));
        // The same expolygons after the filter of union_ex(): what would be lost if the conversion dropped small areas.
        const ExPolygons filtered = union_ex(to_polygons(native));
        if (!native.empty() && filtered.size() < native.size())
            ++kept_small;
        INFO("delta " << entry.second << ": offset_ex gives " << native.size() << " expolygons, " << filtered.size() << " after the filter of union_ex");
        check_expolygons(mine, native, largest);
        CHECK(SG::native(TF::polygons_offset(storage, view_of(*entry.first), entry.second, CLIPPER_JOIN_MITER, 1.2)).size() == mine.size());
    }
    // The test discriminates: the native offset_ex result has small areas that the filter of union_ex would drop.
    INFO("small areas kept, largest symmetric difference " << largest.sym_diff << " mm2 over " << largest.compared << " comparisons");
    CHECK(kept_small >= 1);
    // The union of the same thin shape, with the native filter of union_ex() (the thin shape is dropped natively and here).
    const Polygons thin = to_polygons(parallelogram);
    CHECK(union_ex(thin).size() == SG::native(TF::expolygons_union(storage, view_of(thin))).size());
}

TEST_CASE("nearest_on_lines on an exact tie", "[plugins][tree-geometry]")
{
    // A counter-clockwise square of +-1 mm that starts with its right edge, queried at its centre: the four sides are equally close.
    const coord_t one = scale_i(1.);
    Polygon square({ Point(one, -one), Point(one, one), Point(-one, one), Point(-one, -one) });
    REQUIRE(square.is_counter_clockwise());
    const Polygons rings{ square };
    const std::vector<TF::LineSegment> lines = TF::to_lines(view_of(rings));
    REQUIRE(lines.size() == 4);
    slic3r_api::LineDistancer index;
    std::vector<Linef> native_lines;
    for (const TF::LineSegment &line : lines) {
        index.append_segment(line.a, line.b);
        native_lines.push_back(Linef(Vec2d(double(line.a.x), double(line.a.y)), Vec2d(double(line.b.x), double(line.b.y))));
    }
    index.build_index();
    const AABBTreeIndirect::Tree<2, double> tree = AABBTreeLines::build_aabb_tree_over_indexed_lines(native_lines);
    size_t native_index = 0;
    Vec2d native_point = Vec2d::Zero();
    const Vec2d origin = Vec2d::Zero();
    const double native_distance = AABBTreeLines::squared_distance_to_indexed_lines(native_lines, tree, origin, native_index, native_point, std::numeric_limits<double>::infinity());

    const std::optional<TF::LineHit> mine = TF::nearest_on_lines(index, lines, origin);
    REQUIRE(mine.has_value());
    INFO("tie at the centre of a square: helper line " << mine->line_index << " point (" << mine->point.x() << ", " << mine->point.y() << "), native line " << native_index
         << " point (" << native_point.x() << ", " << native_point.y() << ")");
    // The distance is the same, the closest point is the middle of a side in both cases, but the helper takes the first line (the right
    // side) where the native traversal takes another one: the direction of a push away from the hit point differs.
    CHECK(mine->squared_distance == native_distance);
    CHECK(mine->squared_distance == double(one) * double(one));
    CHECK(mine->line_index == 0);
    CHECK(mine->point == Vec2d(double(one), 0.));
    const bool native_on_a_side_middle = std::abs(native_point.x()) == double(one) && native_point.y() == 0. || std::abs(native_point.y()) == double(one) && native_point.x() == 0.;
    CHECK(native_on_a_side_middle);
}

namespace {
// Verbatim copy of the native move_inside_if_outside() and move_inside() (Support/TreeSupport.cpp, static there), the reference of the
// tests below.
unsigned int native_move_inside(const Polygons &polygons, Point &from, int distance, int64_t maxDist2)
{
    Point  ret = from;
    double bestDist2 = std::numeric_limits<double>::max();
    auto   bestPoly = static_cast<unsigned int>(-1);
    bool   is_already_on_correct_side_of_boundary = false; // whether [from] is already on the right side of the boundary
    for (unsigned int poly_idx = 0; poly_idx < polygons.size(); ++ poly_idx) {
        const Polygon &poly = polygons[poly_idx];
        if (poly.size() < 2)
            continue;
        Point p0 = poly[poly.size() - 2];
        Point p1 = poly.back();
        // because we compare with vSize2 here (no division by zero), we also need to compare by vSize2 inside the loop
        // to avoid integer rounding edge cases
        bool projected_p_beyond_prev_segment = (p1 - p0).cast<int64_t>().dot((from - p0).cast<int64_t>()) >= (p1 - p0).cast<int64_t>().squaredNorm();
        for (const Point& p2 : poly) {
            // X = A + Normal(B-A) * (((B-A) dot (P-A)) / VSize(B-A));
            //   = A +       (B-A) *  ((B-A) dot (P-A)) / VSize2(B-A);
            // X = P projected on AB
            const Point& a = p1;
            const Point& b = p2;
            const Point& p = from;
            auto ab = (b - a).cast<int64_t>();
            auto ap = (p - a).cast<int64_t>();
            int64_t ab_length2 = ab.squaredNorm();
            if (ab_length2 <= 0) { //A = B, i.e. the input polygon had two adjacent points on top of each other.
                p1 = p2; //Skip only one of the points.
                continue;
            }
            int64_t dot_prod = ab.dot(ap);
            if (dot_prod <= 0) { // x is projected to before ab
                if (projected_p_beyond_prev_segment) {
                    //  case which looks like:   > .
                    projected_p_beyond_prev_segment = false;
                    Point& x = p1;

                    auto dist2 = (x - p).cast<int64_t>().squaredNorm();
                    if (dist2 < bestDist2) {
                        bestDist2 = dist2;
                        bestPoly = poly_idx;
                        if (distance == 0)
                            ret = x;
                        else {
                            Vec2d  abd   = ab.cast<double>();
                            Vec2d  p1p2  = (p1 - p0).cast<double>();
                            double lab   = abd.norm();
                            double lp1p2 = p1p2.norm();
                            // inward direction irrespective of sign of [distance]
                            auto inward_dir = perp(abd * (scale_d(10.0) / lab) + p1p2 * (scale_d(10.0) / lp1p2));
                            // MM2INT(10.0) to retain precision for the eventual normalization
                            ret = x + (inward_dir * (distance / inward_dir.norm())).cast<coord_t>();
                            is_already_on_correct_side_of_boundary = inward_dir.dot((p - x).cast<double>()) * distance >= 0;
                        }
                    }
                } else {
                    projected_p_beyond_prev_segment = false;
                    p0 = p1;
                    p1 = p2;
                    continue;
                }
            } else if (dot_prod >= ab_length2) {
                // x is projected to beyond ab
                projected_p_beyond_prev_segment = true;
                p0 = p1;
                p1 = p2;
                continue;
            } else {
                // x is projected to a point properly on the line segment (not onto a vertex). The case which looks like | .
                projected_p_beyond_prev_segment = false;
                Point x = a + (ab.cast<double>() * (double(dot_prod) / double(ab_length2))).cast<coord_t>();
                auto dist2 = (p - x).cast<int64_t>().squaredNorm();
                if (dist2 < bestDist2) {
                    bestDist2 = dist2;
                    bestPoly = poly_idx;
                    if (distance == 0)
                        ret = x;
                    else {
                        Vec2d abd = ab.cast<double>();
                        Vec2d inward_dir = perp(abd * (distance / abd.norm())); // inward or outward depending on the sign of [distance]
                        ret = x + inward_dir.cast<coord_t>();
                        is_already_on_correct_side_of_boundary = inward_dir.dot((p - x).cast<double>()) >= 0;
                    }
                }
            }
            p0 = p1;
            p1 = p2;
        }
    }
    // when the best point is already inside and we're moving inside, or when the best point is already outside and we're moving outside
    if (is_already_on_correct_side_of_boundary) {
        if (bestDist2 < distance * distance)
            from = ret;
        else {
            // from = from; // original point stays unaltered. It is already inside by enough distance
        }
        return bestPoly;
    } else if (bestDist2 < maxDist2) {
        from = ret;
        return bestPoly;
    }
    return -1;
}

Point native_move_inside_if_outside(const Polygons &polygons, Point from, int distance = 0, int64_t maxDist2 = std::numeric_limits<int64_t>::max())
{
    if (! contains(polygons, from))
        native_move_inside(polygons, from, distance, maxDist2);
    return from;
}
}

TEST_CASE("make_circle matches the native one", "[plugins][tree-geometry]")
{
    size_t compared = 0;
    for (const double radius : { 10000., 5000., 123456., 1000000., 77777.7 })
        for (const double error : { 25., 1., 100., 25.5 }) {
            const Polygon native = make_circle(radius, error);
            const std::vector<c_point> mine = TF::make_circle(radius, error);
            REQUIRE(mine.size() == native.points.size());
            for (size_t i = 0; i < mine.size(); ++i) {
                CHECK(mine[i].x == native.points[i].x());
                CHECK(mine[i].y == native.points[i].y());
                ++compared;
            }
        }
    CHECK(compared > 400);
    // The tip circle of the routing: 0.01 mm radius, 25 scaled units of error.
    CHECK(TF::make_circle(scale_d(0.01), 25.).size() == 45);
}

TEST_CASE("move_inside_if_outside matches the native function", "[plugins][tree-geometry]")
{
    std::mt19937 rng(11);
    size_t moved = 0, inside = 0;
    auto compare = [&](const Polygons &polygons, const Point &point) {
        const Point native = native_move_inside_if_outside(polygons, point);
        const c_point mine = TF::move_inside_if_outside(view_of(polygons), ST::to_c_point(point));
        CHECK(mine.x == native.x());
        CHECK(mine.y == native.y());
        if (native == point)
            ++inside;
        else
            ++moved;
    };
    for (int iteration = 0; iteration < 40; ++iteration) {
        Polygons scene = random_scene(rng);
        const BoundingBox box = get_extents(scene);
        for (int i = 0; i < 200; ++i)
            compare(scene, random_point(rng, box, 2.));
        // The rings in another order, and with their start points rotated: the same algorithm sees the same input as the native one.
        std::shuffle(scene.begin(), scene.end(), rng);
        for (Polygon &ring : scene)
            std::rotate(ring.points.begin(), ring.points.begin() + ring.points.size() / 2, ring.points.end());
        for (int i = 0; i < 100; ++i)
            compare(scene, random_point(rng, box, 2.));
        // The vertices themselves and the points a hair outside of them.
        for (const Polygon &ring : scene)
            for (const Point &vertex : ring.points) {
                compare(scene, vertex);
                compare(scene, vertex + Point(1, 1));
                compare(scene, vertex - Point(1, 1));
            }
    }
    CHECK(moved > 3000);
    CHECK(inside > 300);

    SECTION("no ring, a ring under 2 points, equal adjacent points")
    {
        const Polygons none;
        compare(none, P(5, 5));
        CHECK(TF::move_inside_if_outside(view_of(none), CP(5, 5)).x == P(5, 5).x());
        // An empty ring and a ring of one point (built in place: the Polygon constructors refuse it).
        Polygons short_rings(2);
        short_rings[1].points.push_back(P(0, 0));
        compare(short_rings, P(5, 5));
        // Two equal adjacent points: native code skips only one of them.
        const Polygons doubled{ Polygon({ P(0, 0), P(100, 0), P(100, 0), P(100, 100), P(0, 100), P(0, 100) }) };
        for (const Point &point : { P(150, 50), P(50, 150), P(-50, -50), P(150, 150), P(50, -10) })
            compare(doubled, point);
    }
    SECTION("a point inside stays where it is, a point outside goes to the border")
    {
        const Polygons square{ Polygon({ P(0, 0), P(100, 0), P(100, 100), P(0, 100) }) };
        CHECK(TF::move_inside_if_outside(view_of(square), CP(50, 50)).x == P(50, 50).x());
        const c_point moved_point = TF::move_inside_if_outside(view_of(square), CP(150, 50));
        CHECK(moved_point.x == P(100, 50).x());
        CHECK(moved_point.y == P(100, 50).y());
        // Beyond a corner the closest point is the vertex.
        const c_point corner = TF::move_inside_if_outside(view_of(square), CP(130, 130));
        CHECK(corner.x == P(100, 100).x());
        CHECK(corner.y == P(100, 100).y());
    }
}

// The rule on an exact tie: the candidate replaces the best point only when it is strictly closer, so the first ring (then the first edge)
// wins. The rings of a Clipper output are not in the order of the native one: two rings at the same distance from the point, at different
// points, give a result that depends on that order.
TEST_CASE("move_inside_if_outside keeps the first ring on an exact tie", "[plugins][tree-geometry]")
{
    const Polygon left({ P(0, 0), P(10000, 0), P(10000, 10000), P(0, 10000) });
    const Polygon right({ P(20000, 0), P(30000, 0), P(30000, 10000), P(20000, 10000) });
    // (15 mm, 5 mm) is 5 mm from the right side of the left square and 5 mm from the left side of the right square.
    const Point query = P(15000, 5000);
    const Polygons left_first{ left, right };
    const Polygons right_first{ right, left };
    const c_point first = TF::move_inside_if_outside(view_of(left_first), ST::to_c_point(query));
    const c_point second = TF::move_inside_if_outside(view_of(right_first), ST::to_c_point(query));
    CHECK(first.x == P(10000, 5000).x());
    CHECK(first.y == P(10000, 5000).y());
    CHECK(second.x == P(20000, 5000).x());
    CHECK(second.y == P(20000, 5000).y());
    // The native function has the same rule: it agrees with the helper on both orders.
    CHECK(native_move_inside_if_outside(left_first, query) == ST::to_point(first));
    CHECK(native_move_inside_if_outside(right_first, query) == ST::to_point(second));
}
// The scratch storage lives for the work only: a result leaves it by being moved to another storage, an exception leaves the work.
TEST_CASE("tree geometry with_scratch_storage runs the work once and lets a moved result leave", "[plugins][tree-geometry]")
{
    PluginStorage run_storage;
    slic3r_api::StoredPolygonCollection kept(SG::handle(run_storage));
    int runs = 0;
    const coord_t side = scale_i(5.);
    const Polygon native_square({ Point(0, 0), Point(side, 0), Point(side, side), Point(0, side) });
    TF::with_scratch_storage([&](storage_handle *scratch) {
        ++runs;
        slic3r_api::StoredPolygon ring(scratch);
        for (const Point &point : native_square.points)
            ring.push_back(ST::to_c_point(point));
        slic3r_api::StoredPolygonCollection rings(scratch);
        rings.push_back(ring);
        slic3r_api::StoredPolygonCollection united = TF::polygons_union(scratch, rings);
        kept.move_from(united);
    });
    CHECK(runs == 1);
    REQUIRE(kept.size() == 1);
    CHECK(TF::polygons_area(kept) == native_square.area());

    CHECK_THROWS_AS(TF::with_scratch_storage([](storage_handle *) { throw std::runtime_error("work failed"); }), std::runtime_error);
}
