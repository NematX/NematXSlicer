/*
Test the geometry operations available to plugins through the C API and its
C++ wrappers: polygon unions and offsets, polyline clipping, conversion of
polygon trees, and distance queries with LineDistancer.

The tests build their geometry in memory, without a Print or slicing pipeline.
Coordinates use the slicer's scaled integer units. Helpers such as point_mm()
and rectangle_mm() accept millimeters and perform that conversion.

Polygon operations are compared with the corresponding native slicer functions.
Conversion tests also inspect Clipper's polygon tree directly. An island is an
outer contour together with its holes. These tests check that conversion keeps
every contour, hole and vertex intact while placing containing islands before
the islands inside them. Independent islands have no promised relative order.

Polyline comparisons preserve every bend and each path's direction. They may
ignore the order of separate paths, but tests of collection indices or result
ordering check that order explicitly. Other cases verify geometry cleanup,
unchanged inputs, independent results and error reporting through C and C++.

LineDistancer finds distances and nearby line segments. Its tests compare
indexed queries with an exhaustive scan, then check copied geometry, index
invalidation and concurrent reads. A separate opt-in benchmark measures time
without imposing a timing assertion.

Representative calls in the boolean-conversion tests:

    TEST_CASE
        `-- check_sdk_boolean_geometry()
            |-- ClipperLib::Clipper::Execute()
            |-- append_engine_islands()
            |   `-- append_engine_islands() [recursive]
            |-- clipper_execute()
            |-- clipper_shapes_to_expolygons()
            |-- island_signatures()
            |-- check_union_island_order()
            `-- Slic3r::union_ex() [union case only]

Input geometry belongs to local test values; views borrow it during calls.
PluginStorage owns results created through the C API and remains alive while
they are inspected. Ownership tests also clear the inputs before reading the
results again, so sharing input memory cannot pass unnoticed.
*/
#include <catch2/catch.hpp>

#include <chrono>
#include <future>
#include <iostream>
#include <limits>
#include <random>
#include <set>
#include <sstream>
#include <algorithm>
#include "libslic3r/Api/plugin/cpp/LineDistancer.hpp"

#include "libslic3r/Api/host/ClipperShapes.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Polygon.hpp"

namespace {
using namespace Slic3r;

Point point_mm(const double x, const double y)
{
    return Point(scale_i(x), scale_i(y));
}

Polygon rectangle_mm(const double min_x, const double min_y, const double max_x, const double max_y)
{
    Polygon polygon({
        point_mm(min_x, min_y),
        point_mm(max_x, min_y),
        point_mm(max_x, max_y),
        point_mm(min_x, max_y)
    });
    polygon.make_counter_clockwise();
    return polygon;
}

ExPolygon rectangle_with_two_holes()
{
    ExPolygon expolygon(rectangle_mm(0., 0., 20., 20.));
    expolygon.holes.push_back(rectangle_mm(2., 2., 5., 5.));
    expolygon.holes.push_back(rectangle_mm(8., 8., 11., 11.));
    for (Polygon &hole : expolygon.holes)
        hole.make_clockwise();
    return expolygon;
}

slic3r_api::Polygon view(const Polygon &polygon)
{
    return slic3r_api::Polygon(reinterpret_cast<const polygon_handle *>(&polygon));
}

slic3r_api::ExPolygon view(const ExPolygon &expolygon)
{
    return slic3r_api::ExPolygon(reinterpret_cast<const expolygon_handle *>(&expolygon));
}

void check_bbox(const c_bounding_box &bbox,
                const double min_x,
                const double min_y,
                const double max_x,
                const double max_y)
{
    CHECK(bbox.min.x == scale_i(min_x));
    CHECK(bbox.min.y == scale_i(min_y));
    CHECK(bbox.max.x == scale_i(max_x));
    CHECK(bbox.max.y == scale_i(max_y));
}

} // namespace

namespace {
using DirectedPath = std::vector<std::pair<coord_t, coord_t>>;
using DirectedPaths = std::vector<DirectedPath>;

// Only collection order is ignored: every bend and each path's direction remains significant.
DirectedPaths directed_paths(const Slic3r::Polylines &paths);
// Read coordinates through the public API without interpreting host geometry handles.
DirectedPaths directed_paths(const slic3r_api::PolylineCollection &paths);

DirectedPaths directed_paths(const Slic3r::Polylines &paths)
{
    DirectedPaths result;
    for (const Slic3r::Polyline &path : paths) {
        DirectedPath &coordinates = result.emplace_back();
        for (const Slic3r::Point &point : path.points)
            coordinates.emplace_back(point.x(), point.y());
    }
    std::sort(result.begin(), result.end());
    return result;
}

DirectedPaths directed_paths(const slic3r_api::PolylineCollection &paths)
{
    DirectedPaths result;
    for (const slic3r_api::Polyline path : paths) {
        DirectedPath &coordinates = result.emplace_back();
        for (uint32_t index = 0; index < path.size(); ++index) {
            const c_point point = path.at(index);
            coordinates.emplace_back(point.x, point.y);
        }
    }
    std::sort(result.begin(), result.end());
    return result;
}
}

namespace {
// Construct owned literal fragments in internal scaled units; the first has one hole and the other three have none, with allocation failures propagated and no Print or external data dependency.
Slic3r::ExPolygons recorded_union_fragments();
// Inspect borrowed contour containment in original result order and emit Catch assertions matching layer reconstruction; never reorder or mutate geometry, and diagnostics/allocation failures remain visible.
void check_union_island_order(const Slic3r::ExPolygons &islands, const char *stage);
// Check actual contour inclusion, not bounding boxes or a possibly touching first
// vertex. Holes stay attached to their enclosing contour; independent islands are free.
bool has_enclosure_order(const Slic3r::ExPolygons &islands);
// Recursively copy borrowed engine contours before descendants into caller-owned polygons, retaining hole membership; no engine/storage mutation occurs and allocation failures propagate.
void append_engine_islands(const Slic3r::ClipperLib::PolyNode &node, Slic3r::ExPolygons &out);

Slic3r::ExPolygons recorded_union_fragments()
{
    using Slic3r::Point;
    using Slic3r::Polygon;
    return {
        Slic3r::ExPolygon(Polygon{
            Point(-11012588, 10233305), Point(-11003483, 10279077), Point(-10906900, 10512252), Point(-10880968, 10551061), Point(-10551064, 10880969),
            Point(-10512252, 10906902), Point(-10279080, 11003486), Point(-10233304, 11012591), Point(10233305, 11012591), Point(10279081, 11003486),
            Point(10512251, 10906903), Point(10551065, 10880968), Point(10880968, 10551065), Point(10906903, 10512251), Point(11003486, 10279081),
            Point(11012591, 10233305), Point(11012591, -10233304), Point(11003486, -10279080), Point(10906902, -10512252), Point(10880969, -10551064),
            Point(10551061, -10880968), Point(10512252, -10906900), Point(10279077, -11003483), Point(10233305, -11012588), Point(11, -11012588),
            Point(11, -12606380), Point(10324365, -12605756), Point(10324348, -12325628), Point(10355223, -12604049), Point(10462139, -12592193),
            Point(10474032, -12590354), Point(10660460, -12553272), Point(10672151, -12550419), Point(10782694, -12518362), Point(10796899, -12513402),
            Point(11188065, -12352770), Point(11199370, -12347529), Point(11301535, -12294522), Point(11314140, -12287123), Point(11495441, -12167579),
            Point(11505866, -12159995), Point(11596113, -12087766), Point(11607442, -12077644), Point(12059409, -11627484), Point(12068843, -11617096),
            Point(12158172, -11508250), Point(12166366, -11497202), Point(12272926, -11337721), Point(12279997, -11325924), Point(12346399, -11201693),
            Point(12352442, -11188866), Point(12513404, -10796900), Point(12518364, -10782696), Point(12550423, -10672151), Point(12553276, -10660457),
            Point(12590358, -10474027), Point(12592196, -10462135), Point(12604905, -10347536), Point(12605759, -10332095), Point(12607005, 10305853),
            Point(12606331, 10319599), Point(12592543, 10459582), Point(12590525, 10473185), Point(12553108, 10661301), Point(12549766, 10674644),
            Point(12508871, 10809454), Point(12504074, 10822799), Point(12340727, 11213776), Point(12334190, 11227329), Point(12278695, 11328157),
            Point(12272445, 11338440), Point(12166843, 11496488), Point(12159732, 11506199), Point(12087676, 11596229), Point(12077356, 11607757),
            Point(11607757, 12077356), Point(11596229, 12087676), Point(11505864, 12160000), Point(11495438, 12167585), Point(11314141, 12287124),
            Point(11301537, 12294522), Point(11199378, 12347526), Point(11188073, 12352768), Point(10796902, 12513404), Point(10782697, 12518364),
            Point(10672152, 12550423), Point(10660458, 12553276), Point(10473185, 12590525), Point(10459582, 12592543), Point(10319603, 12606330),
            Point(10305866, 12607005), Point(-10305865, 12607005), Point(-10319602, 12606330), Point(-10459581, 12592543), Point(-10473184, 12590525),
            Point(-10661300, 12553108), Point(-10674643, 12549766), Point(-10809453, 12508871), Point(-10822798, 12504074), Point(-11213773, 12340729),
            Point(-11227323, 12334192), Point(-11328151, 12278699), Point(-11338435, 12272449), Point(-11496487, 12166843), Point(-11506199, 12159733),
            Point(-11596233, 12087672), Point(-11607761, 12077352), Point(-12077359, 11607750), Point(-12087679, 11596222), Point(-12159995, 11505866),
            Point(-12167579, 11495441), Point(-12287123, 11314140), Point(-12294522, 11301535), Point(-12347529, 11199370), Point(-12352770, 11188065),
            Point(-12513402, 10796899), Point(-12518362, 10782694), Point(-12550419, 10672151), Point(-12553272, 10660460), Point(-12590522, 10473189),
            Point(-12592540, 10459584), Point(-12606327, 10319604), Point(-12607002, 10305867), Point(-12607002, 11), Point(-11012588, 11)},
            Polygon{Point(10360791, -11272845), Point(10451468, -11491759), Point(10440706, -11496217), Point(10418443, -11502970), Point(10407018, -11505243)}),
        Slic3r::ExPolygon(Polygon{
            Point(-9978430, 9978433), Point(9978433, 9978433), Point(9978433, -9978430), Point(11, -9978430), Point(11, -10538686),
            Point(10162833, -10538686), Point(10265791, -10496039), Point(10456641, -10305191), Point(10258561, -10107110), Point(10538689, -10107110),
            Point(10538689, 10162833), Point(10496042, 10265792), Point(10265792, 10496042), Point(10162833, 10538689), Point(-10162831, 10538689),
            Point(-10265790, 10496042), Point(-10496039, 10265791), Point(-10538686, 10162833), Point(-10538686, 11), Point(-9978430, 11)}),
        Slic3r::ExPolygon(Polygon{
            Point(11, -12606380), Point(11, -11012588), Point(-10233303, -11012588), Point(-10279079, -11003483), Point(-10512249, -10906900),
            Point(-10551063, -10880965), Point(-10880965, -10551063), Point(-10906900, -10512249), Point(-11003483, -10279079), Point(-11012588, -10233303),
            Point(-11012588, 11), Point(-12607002, 11), Point(-12607002, -10305864), Point(-12606327, -10319601), Point(-12592540, -10459580),
            Point(-12590522, -10473183), Point(-12553105, -10661299), Point(-12549763, -10674642), Point(-12508868, -10809452), Point(-12504071, -10822797),
            Point(-12340724, -11213774), Point(-12334187, -11227327), Point(-12278692, -11328155), Point(-12272442, -11338438), Point(-12166840, -11496486),
            Point(-12159729, -11506197), Point(-12087673, -11596227), Point(-12077353, -11607755), Point(-11607755, -12077353), Point(-11596227, -12087673),
            Point(-11505862, -12159997), Point(-11495436, -12167582), Point(-11314139, -12287121), Point(-11301535, -12294519), Point(-11199376, -12347523),
            Point(-11188071, -12352765), Point(-10796900, -12513401), Point(-10782695, -12518361), Point(-10672150, -12550420), Point(-10660456, -12553273),
            Point(-10473183, -12590522), Point(-10459580, -12592540), Point(-10319597, -12606328), Point(-10305851, -12607002)}),
        Slic3r::ExPolygon(Polygon{
            Point(11, -9978430), Point(-9978430, -9978430), Point(-9978430, 11), Point(-10538686, 11), Point(-10538686, -10162831),
            Point(-10496039, -10265790), Point(-10265790, -10496039), Point(-10162831, -10538686), Point(11, -10538686)})
    };
}

void check_union_island_order(const Slic3r::ExPolygons &islands, const char *stage)
{
    INFO(stage);
    CHECK(has_enclosure_order(islands));
}

bool has_enclosure_order(const Slic3r::ExPolygons &islands)
{
    for (size_t later = 1; later < islands.size(); ++later)
        for (size_t earlier = 0; earlier < later; ++earlier) {
            const Slic3r::Polygon &outer = islands[later].contour, &inner = islands[earlier].contour;
            const bool vertices_inside = !inner.points.empty() && std::all_of(inner.points.begin(), inner.points.end(),
                [&outer](const Slic3r::Point &point) { return Slic3r::contains(outer, point, true); });
            // Edges may leave a concave contour even when all vertices are inside.
            if (vertices_inside && std::abs(inner.area()) < std::abs(outer.area()) &&
                Slic3r::diff(Slic3r::Polygons{inner}, Slic3r::Polygons{outer}).empty())
                return false;
        }
    return true;
}

void append_engine_islands(const Slic3r::ClipperLib::PolyNode &node, Slic3r::ExPolygons &out)
{
    if (!node.IsHole()) {
        Slic3r::ExPolygon island(Slic3r::Polygon(node.Contour));
        for (const Slic3r::ClipperLib::PolyNode *hole : node.Childs)
            island.holes.emplace_back(hole->Contour);
        out.push_back(std::move(island));
    }
    for (const Slic3r::ClipperLib::PolyNode *child : node.Childs)
        append_engine_islands(*child, out);
}
}

// Union four owned scaled fragments without Print, compare SDK/native order and independently inspect the same engine options before storage conversion; results remain owned by local storage/values and failures are ordinary Catch diagnostics.
TEST_CASE("SDK union preserves layer island order for four recorded fragments", "[plugins][clipper][union-sdk-order]")
{
    const Slic3r::ExPolygons fragments = recorded_union_fragments();
    REQUIRE(fragments.size() == 4);
    CHECK(fragments[0].contour.size() == 125);
    CHECK(fragments[0].holes.size() == 1);
    CHECK(fragments[1].contour.size() == 20);
    CHECK(fragments[2].contour.size() == 44);
    CHECK(fragments[3].contour.size() == 9);
    const Slic3r::ExPolygons native = Slic3r::union_ex(fragments);
    REQUIRE(native.size() == 2);
    check_union_island_order(native, "native union_ex");
    Slic3r::PluginStorage owned_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&owned_storage);
    const slic3r_api::ExPolygonCollection input(reinterpret_cast<const expolygon_collection_handle *>(&fragments));
    const slic3r_api::ClipperContext clipper(storage);
    const slic3r_api::StoredExPolygonCollection sdk = slic3r_api::clipper_union(clipper(input)).to_expolygon_collection();
    const Slic3r::ExPolygons &sdk_islands = *reinterpret_cast<const Slic3r::ExPolygons *>(sdk.handle());
    REQUIRE(sdk_islands.size() == 2);
    check_union_island_order(sdk_islands, "SDK clipper_union to_expolygon_collection");
    CHECK(Slic3r::area(sdk_islands) == Approx(Slic3r::area(native)).epsilon(1e-12));

    Slic3r::ClipperLib::Clipper engine;
    for (const Slic3r::ExPolygon &fragment : fragments) {
        engine.AddPath(fragment.contour.points, Slic3r::ClipperLib::ptSubject, true);
        for (const Slic3r::Polygon &hole : fragment.holes)
            engine.AddPath(hole.points, Slic3r::ClipperLib::ptSubject, true);
    }
    Slic3r::ClipperLib::PolyTree tree;
    REQUIRE(engine.Execute(Slic3r::ClipperLib::ctUnion, tree, Slic3r::ClipperLib::pftNonZero, Slic3r::ClipperLib::pftNonZero));
    Slic3r::ExPolygons engine_islands;
    append_engine_islands(tree, engine_islands);
    REQUIRE(engine_islands.size() == 2);
    CAPTURE(tree.ChildCount());
    // Two engine roots preserve the recorded inverse containment order; only SDK materialization promises the repaired order.
    REQUIRE(tree.ChildCount() == 2);
    CHECK(Slic3r::BoundingBox(engine_islands[1].contour.points).contains(Slic3r::BoundingBox(engine_islands[0].contour.points)));
    CHECK(engine_islands[1].contour.contains(engine_islands[0].contour.first_point()));
    const std::unique_ptr<Slic3r::ApiClipper::ClipperShapes> converted = Slic3r::ApiClipper::make_polytree_shapes(std::move(tree));
    const Slic3r::ExPolygons converted_islands = converted->to_expolygons();
    REQUIRE(converted_islands.size() == 2);
    check_union_island_order(converted_islands, "direct engine tree, SDK conversion");

    // The native ExPolygon union first materializes flat Paths, then unions those Paths into its PolyTree.
    Slic3r::ClipperLib::Clipper flat_engine;
    for (const Slic3r::ExPolygon &fragment : fragments) {
        flat_engine.AddPath(fragment.contour.points, Slic3r::ClipperLib::ptSubject, true);
        for (const Slic3r::Polygon &hole : fragment.holes)
            flat_engine.AddPath(hole.points, Slic3r::ClipperLib::ptSubject, true);
    }
    Slic3r::ClipperLib::Paths normalized_paths;
    REQUIRE(flat_engine.Execute(Slic3r::ClipperLib::ctUnion, normalized_paths,
        Slic3r::ClipperLib::pftNonZero, Slic3r::ClipperLib::pftNonZero));
    Slic3r::ClipperLib::Clipper hierarchy_engine;
    hierarchy_engine.AddPaths(normalized_paths, Slic3r::ClipperLib::ptSubject, true);
    Slic3r::ClipperLib::PolyTree normalized_tree;
    REQUIRE(hierarchy_engine.Execute(Slic3r::ClipperLib::ctUnion, normalized_tree,
        Slic3r::ClipperLib::pftNonZero, Slic3r::ClipperLib::pftNonZero));
    CHECK(normalized_tree.ChildCount() == 1);
    Slic3r::ExPolygons normalized_islands;
    append_engine_islands(normalized_tree, normalized_islands);
    REQUIRE(normalized_islands.size() == 2);
    check_union_island_order(normalized_islands, "two-pass engine tree, explicit parent-first walk");
    const std::unique_ptr<Slic3r::ApiClipper::ClipperShapes> normalized_conversion =
        Slic3r::ApiClipper::make_polytree_shapes(std::move(normalized_tree));
    const Slic3r::ExPolygons normalized_converted = normalized_conversion->to_expolygons();
    REQUIRE(normalized_converted.size() == 2);
    check_union_island_order(normalized_converted, "two-pass engine tree, unchanged SDK conversion");
    CHECK(Slic3r::area(normalized_converted) == Approx(Slic3r::area(native)).epsilon(1e-12));
}

namespace {
// Serialize contours with their holes and multiplicity. Unordered comparison
// accepts cyclic ring starts and independent island/hole permutations, not winding
// or coordinate changes. Ordered mode checks plain borrowed collection copies.
std::vector<std::string> island_signatures(const Slic3r::ExPolygons &islands, bool ignore_order = true);
// Build disjoint translated copies with at most one hole each, restore their exact borrowed rings on engine-owned nodes, and permute the roots; node ownership remains with the tree and invalid test inputs fail through Catch.
Slic3r::ClipperLib::PolyTree independent_test_roots(const Slic3r::ExPolygons &islands, const std::vector<size_t> &permutation);
// Construct six owned disjoint islands with explicit contour boxes 600/500/400/300/100/100 mm2 and holes on A/C/F; every ring is literal via rectangle_mm, all allocations are local, and failures propagate.
Slic3r::ExPolygons independent_cycle_islands();
// Run subjects/clips through separate NonZero engines and SDK storage, compare
// geometry and hole membership, and check enclosure order plus native union area/count.
void check_sdk_boolean_geometry(const Slic3r::ExPolygons &subjects, const Slic3r::ExPolygons &clips, clipper_operation_t operation);

std::vector<std::string> island_signatures(const Slic3r::ExPolygons &islands, bool ignore_order)
{
    std::vector<std::string> signatures;
    for (const Slic3r::ExPolygon &island : islands) {
        const auto ring_key = [ignore_order](const Slic3r::Polygon &ring) {
            Slic3r::Points points = ring.points;
            if (ignore_order && !points.empty()) {
                const Slic3r::Points::iterator first = std::min_element(points.begin(), points.end(),
                    [](const Slic3r::Point &a, const Slic3r::Point &b) { return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y()); });
                std::rotate(points.begin(), first, points.end());
            }
            std::ostringstream key;
            for (const Slic3r::Point &point : points) key << point.x() << ',' << point.y() << ';';
            return key.str();
        };
        std::string key = "C:" + ring_key(island.contour);
        std::vector<std::string> holes;
        for (const Slic3r::Polygon &hole : island.holes) {
            holes.push_back(ring_key(hole));
        }
        if (ignore_order) std::sort(holes.begin(), holes.end());
        for (const std::string &hole : holes) key += "H:" + hole;
        signatures.push_back(std::move(key));
    }
    if (ignore_order) std::sort(signatures.begin(), signatures.end());
    return signatures;
}

Slic3r::ClipperLib::PolyTree independent_test_roots(const Slic3r::ExPolygons &islands, const std::vector<size_t> &permutation)
{
    Slic3r::ClipperLib::Clipper engine;
    std::vector<Slic3r::BoundingBox> translated_boxes;
    for (size_t index = 0; index < islands.size(); ++index) {
        REQUIRE(islands[index].holes.size() <= 1);
        Slic3r::ExPolygon translated = islands[index];
        const Slic3r::BoundingBox box(translated.contour.points);
        translated.translate(Slic3r::Point(Slic3r::scale_i(1000. * double(index + 1)), 0) - box.min);
        translated_boxes.emplace_back(translated.contour.points);
        engine.AddPath(translated.contour.points, Slic3r::ClipperLib::ptSubject, true);
        for (const Slic3r::Polygon &hole : translated.holes) engine.AddPath(hole.points, Slic3r::ClipperLib::ptSubject, true);
    }
    Slic3r::ClipperLib::PolyTree tree;
    REQUIRE(engine.Execute(Slic3r::ClipperLib::ctUnion, tree, Slic3r::ClipperLib::pftNonZero, Slic3r::ClipperLib::pftNonZero));
    REQUIRE(tree.Childs.size() == islands.size());
    REQUIRE(permutation.size() == islands.size());
    Slic3r::ClipperLib::PolyNodes roots;
    for (const size_t index : permutation) {
        REQUIRE(index < islands.size());
        const Slic3r::ClipperLib::PolyNodes::iterator found = std::find_if(tree.Childs.begin(), tree.Childs.end(), [&translated_boxes, index](const Slic3r::ClipperLib::PolyNode *node) { return Slic3r::BoundingBox(node->Contour) == translated_boxes[index]; });
        REQUIRE(found != tree.Childs.end());
        Slic3r::ClipperLib::PolyNode *node = *found;
        REQUIRE(node->Childs.size() == islands[index].holes.size());
        node->Contour = islands[index].contour.points;
        for (size_t hole = 0; hole < node->Childs.size(); ++hole) node->Childs[hole]->Contour = islands[index].holes[hole].points;
        roots.push_back(node);
    }
    tree.Childs = std::move(roots);
    return tree;
}

Slic3r::ExPolygons independent_cycle_islands()
{
    Slic3r::ExPolygons source{
        Slic3r::ExPolygon(rectangle_mm(0., 0., 60., 10.)),
        Slic3r::ExPolygon(rectangle_mm(100., 0., 150., 10.)),
        Slic3r::ExPolygon(rectangle_mm(200., 0., 240., 10.)),
        Slic3r::ExPolygon(rectangle_mm(300., 0., 330., 10.)),
        Slic3r::ExPolygon(rectangle_mm(400., 0., 410., 10.)),
        Slic3r::ExPolygon(rectangle_mm(500., 0., 505., 20.))};
    source[0].holes.push_back(rectangle_mm(1., 1., 59., 9.));
    source[2].holes.push_back(rectangle_mm(201., 1., 239., 9.));
    source[5].holes.push_back(rectangle_mm(501., 1., 504., 19.));
    for (Slic3r::ExPolygon &island : source)
        for (Slic3r::Polygon &hole : island.holes) hole.make_clockwise();
    return source;
}

void check_sdk_boolean_geometry(const Slic3r::ExPolygons &subjects, const Slic3r::ExPolygons &clips, clipper_operation_t operation)
{
    Slic3r::ClipperLib::Clipper engine;
    for (const Slic3r::ExPolygon &island : subjects) {
        engine.AddPath(island.contour.points, Slic3r::ClipperLib::ptSubject, true);
        for (const Slic3r::Polygon &hole : island.holes) engine.AddPath(hole.points, Slic3r::ClipperLib::ptSubject, true);
    }
    if (operation != CLIPPER_OPERATION_UNION)
        for (const Slic3r::ExPolygon &island : clips) {
            engine.AddPath(island.contour.points, Slic3r::ClipperLib::ptClip, true);
            for (const Slic3r::Polygon &hole : island.holes) engine.AddPath(hole.points, Slic3r::ClipperLib::ptClip, true);
        }
    const Slic3r::ClipperLib::ClipType type = operation == CLIPPER_OPERATION_UNION ? Slic3r::ClipperLib::ctUnion : operation == CLIPPER_OPERATION_INTERSECTION ? Slic3r::ClipperLib::ctIntersection : operation == CLIPPER_OPERATION_DIFFERENCE ? Slic3r::ClipperLib::ctDifference : Slic3r::ClipperLib::ctXor;
    Slic3r::ClipperLib::PolyTree tree;
    REQUIRE(engine.Execute(type, tree, Slic3r::ClipperLib::pftNonZero, Slic3r::ClipperLib::pftNonZero));
    Slic3r::ExPolygons raw;
    append_engine_islands(tree, raw);
    Slic3r::PluginStorage storage_owner;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&storage_owner);
    const slic3r_api::ClipperContext context(storage);
    const slic3r_api::ExPolygonCollection subject_view(reinterpret_cast<const expolygon_collection_handle *>(&subjects));
    const slic3r_api::ExPolygonCollection clip_view(reinterpret_cast<const expolygon_collection_handle *>(&clips));
    const clipper_shapes_handle *shape = ::clipper_execute(storage, operation, context(subject_view).handle(), operation == CLIPPER_OPERATION_UNION ? nullptr : context(clip_view).handle());
    REQUIRE(shape != nullptr);
    const expolygon_collection_handle *output = ::clipper_shapes_to_expolygons(storage, shape);
    REQUIRE(output != nullptr);
    const Slic3r::ExPolygons &converted = *reinterpret_cast<const Slic3r::ExPolygons *>(output);
    CHECK(island_signatures(converted) == island_signatures(raw));
    check_union_island_order(converted, "SDK boolean preserves geometry and layer order");
    if (operation == CLIPPER_OPERATION_UNION) {
        const Slic3r::ExPolygons native = Slic3r::union_ex(subjects);
        CHECK(converted.size() == native.size());
        CHECK(Slic3r::area(converted) == Approx(Slic3r::area(native)).epsilon(1.e-12));
    }
}
}

// Nested rings must follow their containers; disjoint and equal-box islands
// retain their geometry without a promised relative position.
TEST_CASE("SDK multi-root conversion preserves nested and independent islands", "[plugins][clipper][union-sdk-order]")
{
    Slic3r::ExPolygons source{
        Slic3r::ExPolygon(rectangle_mm(0., 0., 100., 100.)),
        Slic3r::ExPolygon(rectangle_mm(2., 2., 98., 98.)),
        Slic3r::ExPolygon(rectangle_mm(4., 4., 96., 96.)),
        Slic3r::ExPolygon(rectangle_mm(200., 0., 220., 20.)),
        Slic3r::ExPolygon(rectangle_mm(240., 0., 250., 40.)),
        Slic3r::ExPolygon(rectangle_mm(280., 0., 285., 5.))};
    source[0].holes.push_back(rectangle_mm(1., 1., 99., 99.));
    source[1].holes.push_back(rectangle_mm(3., 3., 97., 97.));
    source[0].holes.front().make_clockwise();
    source[1].holes.front().make_clockwise();
    REQUIRE(source[0].area() < source[2].area());
    Slic3r::ClipperLib::PolyTree tree = independent_test_roots(source, {2, 5, 4, 1, 3, 0});
    Slic3r::ExPolygons raw;
    append_engine_islands(tree, raw);
    const std::unique_ptr<Slic3r::ApiClipper::ClipperShapes> shapes = Slic3r::ApiClipper::make_polytree_shapes(std::move(tree));
    const Slic3r::ExPolygons converted = shapes->to_expolygons();
    REQUIRE(converted.size() == source.size());
    CHECK(island_signatures(converted) == island_signatures(raw));
    check_union_island_order(converted, "nested islands with net-area trap and equal boxes");
}

// A scrambled forest must preserve every contour/hole, including equal-size
// disjoint islands. No particular permutation is part of the result contract.
TEST_CASE("SDK conversion preserves scrambled independent islands", "[plugins][clipper][union-sdk-order][union-sdk-cycles]")
{
    const Slic3r::ExPolygons source = independent_cycle_islands();
    Slic3r::ClipperLib::PolyTree tree = independent_test_roots(source, {1, 0, 4, 2, 3, 5});
    REQUIRE(tree.ChildCount() == 6);
    Slic3r::ExPolygons raw;
    append_engine_islands(tree, raw);
    const std::vector<std::string> raw_rings = island_signatures(raw, false);
    const std::vector<std::string> expected_rings = island_signatures(source, false);
    const std::vector<size_t> expected_sources{1, 0, 3, 4, 2, 5};
    REQUIRE(raw_rings.size() == expected_sources.size());
    REQUIRE(expected_rings.size() == expected_sources.size());
    for (size_t index = 0; index < expected_sources.size(); ++index)
        CHECK(raw_rings[expected_sources[index]] == expected_rings[index]);
    const Slic3r::BoundingBox e(source[4].contour.points), f(source[5].contour.points);
    CHECK(e.size().x() * e.size().y() == f.size().x() * f.size().y());
    const std::unique_ptr<Slic3r::ApiClipper::ClipperShapes> shapes = Slic3r::ApiClipper::make_polytree_shapes(std::move(tree));
    const Slic3r::ExPolygons converted = shapes->to_expolygons();
    REQUIRE(converted.size() == source.size());
    CHECK(island_signatures(converted) == island_signatures(raw));
    check_union_island_order(converted, "scrambled independent islands and equal boxes");
}

// Ordered inputs still require full geometry and hole membership, not retention
// of their ranks after operation-backed materialization.
TEST_CASE("SDK conversion preserves independent islands from ordered input", "[plugins][clipper][union-sdk-order][union-sdk-sorted]")
{
    const Slic3r::ExPolygons source = independent_cycle_islands();
    Slic3r::ClipperLib::PolyTree tree = independent_test_roots(source, {0, 1, 2, 3, 4, 5});
    REQUIRE(tree.ChildCount() == 6);
    Slic3r::ExPolygons raw;
    append_engine_islands(tree, raw);
    const std::vector<std::string> expected_rings = island_signatures(source, false);
    REQUIRE(island_signatures(raw, false) == expected_rings);
    const std::unique_ptr<Slic3r::ApiClipper::ClipperShapes> shapes = Slic3r::ApiClipper::make_polytree_shapes(std::move(tree));
    const Slic3r::ExPolygons converted = shapes->to_expolygons();
    REQUIRE(converted.size() == source.size());
    CHECK(island_signatures(converted) == island_signatures(raw));
    check_union_island_order(converted, "ordered input with independent equal-size islands");
}

// C must follow both containing contours even when the engine places it under
// A's hole. Preserve geometry without requiring unrelated subtree ranks.
TEST_CASE("SDK multi-root ordering interleaves containing roots and descendants", "[plugins][clipper][union-sdk-order][union-sdk-interleaving]")
{
    Slic3r::ExPolygon a(rectangle_mm(0., 0., 100., 100.));
    a.holes.push_back(rectangle_mm(1., 1., 99., 99.));
    a.holes.front().make_clockwise();
    Slic3r::ExPolygon b(rectangle_mm(10., 10., 90., 90.));
    b.holes.push_back(rectangle_mm(11., 11., 89., 89.));
    b.holes.front().make_clockwise();
    const Slic3r::ExPolygons source{a, b, Slic3r::ExPolygon(rectangle_mm(20., 20., 80., 80.))};
    Slic3r::ClipperLib::PolyTree tree = independent_test_roots(source, {0, 2, 1});
    Slic3r::ClipperLib::PolyNode *root_a = tree.Childs[0];
    Slic3r::ClipperLib::PolyNode *child_c = tree.Childs[1];
    Slic3r::ClipperLib::PolyNode *root_b = tree.Childs[2];
    REQUIRE(root_a->ChildCount() == 1);
    root_a->Childs.front()->Childs.push_back(child_c);
    child_c->Parent = root_a->Childs.front();
    tree.Childs = {root_a, root_b};
    REQUIRE(tree.ChildCount() == 2);
    Slic3r::ExPolygons raw;
    append_engine_islands(tree, raw);
    REQUIRE(raw.size() == 3);
    CHECK(Slic3r::BoundingBox(raw[1].contour.points) == Slic3r::BoundingBox(source[2].contour.points));
    const std::unique_ptr<Slic3r::ApiClipper::ClipperShapes> shapes = Slic3r::ApiClipper::make_polytree_shapes(std::move(tree));
    const Slic3r::ExPolygons converted = shapes->to_expolygons();
    REQUIRE(converted.size() == 3);
    CHECK(island_signatures(converted) == island_signatures(raw));
    check_union_island_order(converted, "interleaving A/B/C instead of root-grouped A/C/B");
}

// A single-root tree still promises containment order and preserved rings, not
// a fixed traversal of its independent siblings.
TEST_CASE("SDK single-root conversion preserves all levels and branches", "[plugins][clipper][union-sdk-order]")
{
    Slic3r::ExPolygon outer(rectangle_mm(0., 0., 100., 100.));
    outer.holes.push_back(rectangle_mm(1., 1., 99., 99.));
    outer.holes.front().make_clockwise();
    Slic3r::ExPolygon branch(rectangle_mm(20., 20., 60., 60.));
    branch.holes.push_back(rectangle_mm(22., 22., 58., 58.));
    branch.holes.front().make_clockwise();
    const Slic3r::ExPolygons source{outer, Slic3r::ExPolygon(rectangle_mm(5., 5., 7., 7.)), branch, Slic3r::ExPolygon(rectangle_mm(24., 24., 56., 56.))};
    Slic3r::ClipperLib::Clipper engine;
    for (const Slic3r::ExPolygon &island : source) {
        engine.AddPath(island.contour.points, Slic3r::ClipperLib::ptSubject, true);
        for (const Slic3r::Polygon &hole : island.holes) engine.AddPath(hole.points, Slic3r::ClipperLib::ptSubject, true);
    }
    Slic3r::ClipperLib::PolyTree tree;
    REQUIRE(engine.Execute(Slic3r::ClipperLib::ctUnion, tree, Slic3r::ClipperLib::pftNonZero, Slic3r::ClipperLib::pftNonZero));
    REQUIRE(tree.ChildCount() == 1);
    REQUIRE(tree.Childs.front()->ChildCount() == 1);
    Slic3r::ClipperLib::PolyNodes &siblings = tree.Childs.front()->Childs.front()->Childs;
    REQUIRE(siblings.size() == 2);
    std::sort(siblings.begin(), siblings.end(), [](const Slic3r::ClipperLib::PolyNode *a, const Slic3r::ClipperLib::PolyNode *b) { return Slic3r::BoundingBox(a->Contour).size().x() < Slic3r::BoundingBox(b->Contour).size().x(); });
    Slic3r::ExPolygons raw;
    append_engine_islands(tree, raw);
    REQUIRE(raw.size() == 4);
    const std::unique_ptr<Slic3r::ApiClipper::ClipperShapes> shapes = Slic3r::ApiClipper::make_polytree_shapes(std::move(tree));
    const Slic3r::ExPolygons converted = shapes->to_expolygons();
    CHECK(island_signatures(converted) == island_signatures(raw));
    check_union_island_order(converted, "single root with multiple levels and unequal sibling sizes");
}

// Exercise concave disjoint contours whose first vertex touches the other's boundary, point contacts and overlapping inputs; exact stored rings and layer order must survive conversion and every SDK boolean.
TEST_CASE("SDK conversion handles concave bays and boundary contacts", "[plugins][clipper][union-sdk-order]")
{
    const Slic3r::Polygon bay({point_mm(0., 0.), point_mm(30., 0.), point_mm(30., 5.), point_mm(5., 5.), point_mm(5., 25.), point_mm(30., 25.), point_mm(30., 30.), point_mm(0., 30.)});
    const Slic3r::Polygon triangle({point_mm(5., 15.), point_mm(10., 10.), point_mm(10., 20.)});
    REQUIRE(bay.contains(triangle.front()));
    REQUIRE_FALSE(Slic3r::contains(bay, point_mm(8., 15.), false));
    const Slic3r::ExPolygons source{Slic3r::ExPolygon(bay), Slic3r::ExPolygon(triangle)};
    Slic3r::ClipperLib::PolyTree tree = independent_test_roots(source, {1, 0});
    Slic3r::ExPolygons raw;
    append_engine_islands(tree, raw);
    const std::unique_ptr<Slic3r::ApiClipper::ClipperShapes> shapes = Slic3r::ApiClipper::make_polytree_shapes(std::move(tree));
    const Slic3r::ExPolygons converted = shapes->to_expolygons();
    CHECK(island_signatures(converted) == island_signatures(raw));
    REQUIRE(converted.size() == 2);
    check_union_island_order(converted, "concave bay with first vertex on boundary");
    const Slic3r::ExPolygons touching{Slic3r::ExPolygon(rectangle_mm(0., 0., 20., 20.)), Slic3r::ExPolygon(rectangle_mm(20., 20., 25., 25.)), Slic3r::ExPolygon(rectangle_mm(10., -5., 30., 5.))};
    const Slic3r::ExPolygons clip{Slic3r::ExPolygon(rectangle_mm(5., -10., 22., 22.))};
    for (const clipper_operation_t operation : {CLIPPER_OPERATION_UNION, CLIPPER_OPERATION_INTERSECTION, CLIPPER_OPERATION_DIFFERENCE, CLIPPER_OPERATION_XOR}) {
        CAPTURE(operation);
        check_sdk_boolean_geometry(touching, clip, operation);
    }
}

TEST_CASE("Island comparison accepts only geometry-preserving enclosure orders", "[plugins][clipper][union-sdk-order][clipper-comparison]")
{
    Slic3r::ExPolygon outer(rectangle_mm(0., 0., 20., 20.));
    outer.holes.push_back(rectangle_mm(2., 2., 18., 18.));
    outer.holes.front().make_clockwise();
    const Slic3r::ExPolygon inner(rectangle_mm(4., 4., 6., 6.));
    const Slic3r::ExPolygon independent(rectangle_mm(40., 0., 45., 5.));
    const Slic3r::ExPolygons expected{outer, inner, independent};
    Slic3r::ExPolygons permuted{independent, outer, inner};
    std::rotate(permuted[1].contour.points.begin(), permuted[1].contour.points.begin() + 1, permuted[1].contour.points.end());
    CHECK(island_signatures(permuted) == island_signatures(expected));
    CHECK(has_enclosure_order(expected));
    CHECK(has_enclosure_order(permuted));
    CHECK_FALSE(has_enclosure_order(Slic3r::ExPolygons{inner, independent, outer}));
    const Slic3r::ExPolygon with_holes = rectangle_with_two_holes();
    Slic3r::ExPolygon reordered_holes(with_holes);
    std::reverse(reordered_holes.holes.begin(), reordered_holes.holes.end());
    std::rotate(reordered_holes.holes[0].points.begin(), reordered_holes.holes[0].points.begin() + 1, reordered_holes.holes[0].points.end());
    CHECK(island_signatures({reordered_holes}) == island_signatures({with_holes}));
    Slic3r::ExPolygons changed(expected);
    SECTION("missing island") { changed.pop_back(); }
    SECTION("duplicate island") { changed.push_back(inner); }
    SECTION("duplicate hole") { changed[0].holes.push_back(changed[0].holes.front()); }
    SECTION("coordinate change") { changed[1].contour.points.front().x() += 1; }
    SECTION("hole assigned to another contour") {
        changed[2].holes.push_back(changed[0].holes.front());
        changed[0].holes.clear();
    }
    SECTION("reversed winding") { changed[1].contour.reverse(); }
    CHECK(island_signatures(changed) != island_signatures(expected));
}

TEST_CASE("Enclosure comparison distinguishes contacts from concave overlaps", "[plugins][clipper][union-sdk-order][clipper-comparison]")
{
    const Slic3r::ExPolygon bay(Slic3r::Polygon{point_mm(0., 0.), point_mm(30., 0.), point_mm(30., 5.),
        point_mm(5., 5.), point_mm(5., 25.), point_mm(30., 25.), point_mm(30., 30.), point_mm(0., 30.)});
    const Slic3r::ExPolygon crossing(Slic3r::Polygon{point_mm(1., 1.), point_mm(29., 1.), point_mm(1., 29.)});
    for (const Slic3r::Point &point : crossing.contour.points) REQUIRE(bay.contour.contains(point));
    CHECK(has_enclosure_order(Slic3r::ExPolygons{crossing, bay}));
    CHECK(has_enclosure_order(Slic3r::ExPolygons{bay, crossing}));
    const Slic3r::ExPolygon touching(rectangle_mm(0., 1., 2., 3.));
    CHECK(has_enclosure_order(Slic3r::ExPolygons{bay, touching}));
    CHECK_FALSE(has_enclosure_order(Slic3r::ExPolygons{touching, bay}));
}

// Keep borrowed ExPolygon collections in exact source order, including hole membership; only operation-backed trees may reorder their independent owned materializations.
TEST_CASE("SDK borrowed ExPolygon conversion preserves exact source order", "[plugins][clipper][union-sdk-order]")
{
    const Slic3r::ExPolygons source{Slic3r::ExPolygon(rectangle_mm(50., 0., 55., 5.)), rectangle_with_two_holes(), Slic3r::ExPolygon(rectangle_mm(80., 0., 90., 10.))};
    const std::vector<std::string> before = island_signatures(source, false);
    Slic3r::PluginStorage storage_owner;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&storage_owner);
    const slic3r_api::ClipperContext context(storage);
    const slic3r_api::ExPolygonCollection input(reinterpret_cast<const expolygon_collection_handle *>(&source));
    const slic3r_api::StoredExPolygonCollection output = context(input).to_expolygon_collection();
    const Slic3r::ExPolygons &copied = *reinterpret_cast<const Slic3r::ExPolygons *>(output.handle());
    CHECK(island_signatures(copied, false) == before);
    CHECK(island_signatures(source, false) == before);
}

// Compare every offset ring before ExPolygon conversion with all converted contours/holes; list order may change but no vertex, direction or ring is allowed to change or disappear.
TEST_CASE("SDK offset tree conversion preserves every ring exactly", "[plugins][clipper][union-sdk-order]")
{
    const Slic3r::ExPolygons source{rectangle_with_two_holes(), Slic3r::ExPolygon(rectangle_mm(50., 0., 55., 5.))};
    Slic3r::PluginStorage storage_owner;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&storage_owner);
    const slic3r_api::ClipperContext context(storage);
    const slic3r_api::ExPolygonCollection input(reinterpret_cast<const expolygon_collection_handle *>(&source));
    const slic3r_api::ClipperOperand offset = slic3r_api::clipper_offset(context(input), Slic3r::scale_i(.25));
    const slic3r_api::StoredPolygonCollection flat = offset.to_polygon_collection();
    const slic3r_api::StoredExPolygonCollection grouped = offset.to_expolygon_collection();
    const Slic3r::Polygons &raw = *reinterpret_cast<const Slic3r::Polygons *>(flat.handle());
    const Slic3r::ExPolygons &converted = *reinterpret_cast<const Slic3r::ExPolygons *>(grouped.handle());
    Slic3r::ExPolygons raw_rings, converted_rings;
    for (const Slic3r::Polygon &ring : raw) raw_rings.emplace_back(ring);
    for (const Slic3r::ExPolygon &island : converted) {
        converted_rings.emplace_back(island.contour);
        for (const Slic3r::Polygon &hole : island.holes) converted_rings.emplace_back(hole);
    }
    REQUIRE(converted.size() == 2);
    CHECK(island_signatures(converted_rings) == island_signatures(raw_rings));
    check_union_island_order(converted, "offset tree conversion order without ring changes");
}

// Generate fixed-seed rings, islands, concave components, point contacts and overlaps for all four booleans; compare every coordinate/hole against raw engine output and native union area/island count without weakening empty-result checks.
TEST_CASE("SDK boolean conversion preserves generated geometry and topological order", "[plugins][clipper][union-sdk-order]")
{
    std::mt19937 random(0xC7102026u);
    for (size_t iteration = 0; iteration < 128; ++iteration) {
        CAPTURE(iteration);
        const double jitter = double(random() % 5);
        Slic3r::ExPolygon outer(rectangle_mm(0., 0., 60., 60.));
        outer.holes.push_back(rectangle_mm(5., 5., 55., 55.));
        outer.holes.front().make_clockwise();
        Slic3r::ExPolygons subjects{outer, Slic3r::ExPolygon(rectangle_mm(10. + jitter, 10., 20. + jitter, 20.)),
            Slic3r::ExPolygon(Slic3r::Polygon{point_mm(80., 0.), point_mm(110., 0.), point_mm(110., 5.), point_mm(85., 5.), point_mm(85., 25.), point_mm(110., 25.), point_mm(110., 30.), point_mm(80., 30.)})};
        if (iteration % 3 == 0) subjects.emplace_back(rectangle_mm(60., 60., 65. + jitter, 65. + jitter));
        if (iteration % 3 == 1) subjects.emplace_back(rectangle_mm(50., -2. - jitter, 80., 10.));
        if (iteration % 3 == 2) subjects.emplace_back(rectangle_mm(30., 30., 40., 40.));
        std::shuffle(subjects.begin(), subjects.end(), random);
        const double x = double(int(random() % 110) - 5);
        const double y = double(int(random() % 55) - 5);
        const Slic3r::ExPolygons clips{Slic3r::ExPolygon(rectangle_mm(x, y, x + 15. + double(random() % 40), y + 15. + double(random() % 40)))};
        for (const clipper_operation_t operation : {CLIPPER_OPERATION_UNION, CLIPPER_OPERATION_INTERSECTION, CLIPPER_OPERATION_DIFFERENCE, CLIPPER_OPERATION_XOR}) {
            CAPTURE(operation);
            check_sdk_boolean_geometry(subjects, clips, operation);
        }
    }
}

TEST_CASE("Batch polyline clipping matches core directed paths and owns results", "[plugins][clipper][clipper-batch]")
{
    const bool difference = GENERATE(false, true);
    Slic3r::Polylines subjects{
        Slic3r::Polyline{point_mm(-5., 3.), point_mm(4., 8.), point_mm(15., 8.), point_mm(45., 3.)},
        Slic3r::Polyline{point_mm(-5., 10.), point_mm(45., 10.)},
        Slic3r::Polyline{point_mm(12., 12.), point_mm(17., 17.)},
        Slic3r::Polyline{point_mm(-5., -5.), point_mm(-1., -1.)},
        Slic3r::Polyline{point_mm(0., 0.), point_mm(20., 0.), point_mm(20., 20.)}
    };
    Slic3r::ExPolygons clips{rectangle_with_two_holes(), Slic3r::ExPolygon(rectangle_mm(30., 0., 40., 20.))};
    const DirectedPaths original = directed_paths(subjects);
    const Slic3r::ExPolygons original_clips = clips;
    const DirectedPaths expected = directed_paths(difference ? Slic3r::diff_pl(subjects, clips) : Slic3r::intersection_pl(subjects, clips));
    PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    const slic3r_api::PolylineCollection subject(reinterpret_cast<const polyline_collection_handle *>(&subjects));
    const slic3r_api::ExPolygonCollection clip(reinterpret_cast<const expolygon_collection_handle *>(&clips));
    slic3r_api::StoredPolylineCollection result = difference ?
        slic3r_api::clipper_diff_polylines_expolygons(storage, subject, clip) :
        slic3r_api::clipper_intersection_polylines_expolygons(storage, subject, clip);
    REQUIRE_FALSE(expected.empty());
    if (!difference)
        CHECK(std::any_of(expected.begin(), expected.end(), [](const DirectedPath &path) { return path.size() > 2; }));
    CHECK(directed_paths(result.readonly()) == expected);
    CHECK(directed_paths(subjects) == original);
    REQUIRE(clips.size() == original_clips.size());
    for (size_t index = 0; index < clips.size(); ++index) {
        CHECK(clips[index].contour.points == original_clips[index].contour.points);
        REQUIRE(clips[index].holes.size() == original_clips[index].holes.size());
        for (size_t hole = 0; hole < clips[index].holes.size(); ++hole)
            CHECK(clips[index].holes[hole].points == original_clips[index].holes[hole].points);
    }
    subjects.clear();
    clips.clear();
    CHECK(directed_paths(result.readonly()) == expected);
}

TEST_CASE("Batch polyline C ABI handles null and empty inputs", "[plugins][clipper][clipper-batch]")
{
    const bool difference = GENERATE(false, true);
    using Operation = polyline_collection_handle *(*)(storage_handle *, const polyline_collection_handle *, const expolygon_collection_handle *);
    const Operation operation = difference ? ::clipper_diff_polylines_expolygons : ::clipper_intersection_polylines_expolygons;
    PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    Slic3r::Polylines subjects{Slic3r::Polyline{point_mm(-5., 10.), point_mm(25., 10.)},
                              Slic3r::Polyline{}, Slic3r::Polyline{point_mm(1., 1.)}};
    Slic3r::ExPolygons clips{rectangle_with_two_holes()};
    const polyline_collection_handle *subject = reinterpret_cast<const polyline_collection_handle *>(&subjects);
    const expolygon_collection_handle *clip = reinterpret_cast<const expolygon_collection_handle *>(&clips);
    CHECK(operation(nullptr, subject, clip) == nullptr);
    CHECK(operation(storage, nullptr, clip) == nullptr);
    const DirectedPaths original = directed_paths(subjects);
    Slic3r::ExPolygons empty;
    const expolygon_collection_handle *empty_clips[] = {nullptr, reinterpret_cast<const expolygon_collection_handle *>(&empty)};
    for (const expolygon_collection_handle *mask : empty_clips) {
        polyline_collection_handle *handle = operation(storage, subject, mask);
        REQUIRE(handle != nullptr);
        CHECK(handle != subject);
        slic3r_api::StoredPolylineCollection result = slic3r_api::StoredPolylineCollection::adopt(storage, handle);
        CHECK(directed_paths(result.readonly()) == (difference ? original : DirectedPaths{}));
        subjects.front().points.front().x() += 1;
        CHECK(directed_paths(result.readonly()) == (difference ? original : DirectedPaths{}));
        subjects.front().points.front().x() -= 1;
        if (mask != nullptr) {
            const slic3r_api::PolylineCollection source(subject);
            const slic3r_api::ExPolygonCollection areas(mask);
            slic3r_api::StoredPolylineCollection cpp_result = difference ?
                slic3r_api::clipper_diff_polylines_expolygons(storage, source, areas) :
                slic3r_api::clipper_intersection_polylines_expolygons(storage, source, areas);
            CHECK(directed_paths(cpp_result.readonly()) == (difference ? original : DirectedPaths{}));
        }
    }
    {
        const DirectedPaths expected = directed_paths(difference ? Slic3r::diff_pl(subjects, clips) : Slic3r::intersection_pl(subjects, clips));
        slic3r_api::StoredPolylineCollection result = slic3r_api::StoredPolylineCollection::adopt(storage, operation(storage, subject, clip));
        CHECK(directed_paths(result.readonly()) == expected);
    }
    subjects.clear();
    for (const expolygon_collection_handle *mask : {clip, empty_clips[0], empty_clips[1]}) {
        polyline_collection_handle *handle = operation(storage, subject, mask);
        REQUIRE(handle != nullptr);
        slic3r_api::StoredPolylineCollection result = slic3r_api::StoredPolylineCollection::adopt(storage, handle);
        CHECK(result.empty());
    }
    const slic3r_api::PolylineCollection subject_view(subject);
    const slic3r_api::ExPolygonCollection clip_view(clip);
    CHECK(slic3r_api::clipper_diff_polylines_expolygons(storage, subject_view, clip_view).empty());
    CHECK(slic3r_api::clipper_intersection_polylines_expolygons(storage, subject_view, clip_view).empty());
}

TEST_CASE("ClipperOperand path_count counts raw non-empty paths", "[plugins][clipper]")
{
    PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    slic3r_api::ClipperContext clipper(storage);

    slic3r_api::ClipperOperand empty = clipper.empty();
    CHECK(empty.path_count() == 0);
    CHECK(empty.empty());

    Polygon polygon = rectangle_mm(0., 0., 10., 10.);
    slic3r_api::ClipperOperand polygon_operand = clipper(view(polygon));
    CHECK(polygon_operand.path_count() == 1);
    CHECK(!polygon_operand.empty());

    ExPolygon expolygon = rectangle_with_two_holes();
    slic3r_api::ClipperOperand expolygon_operand = clipper(view(expolygon));
    CHECK(expolygon_operand.path_count() == 3);

    Polygon other_polygon = rectangle_mm(30., 0., 40., 10.);
    slic3r_api::ClipperOperand concatenated = clipper.empty();
    concatenated += polygon_operand;
    CHECK(concatenated.path_count() == 1);
    concatenated += clipper(view(other_polygon));
    CHECK(concatenated.path_count() == 2);

    slic3r_api::ClipperOperand unioned = slic3r_api::clipper_union(concatenated);
    CHECK(unioned.path_count() == 2);
    CHECK(!unioned.empty());
    // A Clipper result is a PolyTree whose root has an empty contour; only its children are paths.
    CHECK(unioned.to_polygon_collection().size() == 2);
}

TEST_CASE("ClipperOperand concatenation treats null operands as empty", "[plugins][clipper]")
{
    PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    slic3r_api::ClipperContext clipper(storage);
    Polygon polygon = rectangle_mm(0., 0., 10., 10.);

    // clipper() is a null view; accumulating into it and appending it are no-ops.
    slic3r_api::ClipperOperand accumulated = clipper();
    accumulated += clipper();
    CHECK(accumulated.empty());
    accumulated += clipper(view(polygon));
    CHECK(accumulated.path_count() == 1);
    accumulated += clipper();
    CHECK(accumulated.path_count() == 1);
    accumulated += clipper.empty();
    CHECK(accumulated.path_count() == 1);

    slic3r_api::ClipperOperand null_operand = clipper();
    CHECK((null_operand + accumulated).path_count() == 1);
    CHECK((accumulated + null_operand).path_count() == 1);
    CHECK((null_operand + clipper()).empty());

    // Null views can be moved like owned ones.
    slic3r_api::ClipperOperand moved(std::move(null_operand));
    CHECK(moved.empty());
    moved = clipper();
    CHECK(moved.empty());

    // The C call keeps the first handle when there is nothing to append.
    clipper_shapes_handle *raw = ::clipper_shapes_from_polygon(storage, view(polygon).handle());
    raw = ::clipper_concat_replace(storage, raw, nullptr);
    REQUIRE(raw != nullptr);
    CHECK(::clipper_shapes_path_count(raw) == 1);
    CHECK(storage_free(storage, raw));
}

TEST_CASE("ClipperOperand bounding_box reads raw Clipper points", "[plugins][clipper]")
{
    PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    slic3r_api::ClipperContext clipper(storage);

    CHECK(clipper.empty().bounding_box().min.x == 0);
    CHECK(clipper.empty().bounding_box().min.y == 0);
    CHECK(clipper.empty().bounding_box().max.x == 0);
    CHECK(clipper.empty().bounding_box().max.y == 0);

    Polygon polygon = rectangle_mm(-2., 3., 10., 12.);
    check_bbox(clipper(view(polygon)).bounding_box(), -2., 3., 10., 12.);

    ExPolygon expolygon = rectangle_with_two_holes();
    check_bbox(clipper(view(expolygon)).bounding_box(), 0., 0., 20., 20.);

    Polygon other_polygon = rectangle_mm(30., -5., 40., 6.);
    slic3r_api::ClipperOperand concatenated = clipper(view(polygon));
    concatenated += clipper(view(other_polygon));
    check_bbox(concatenated.bounding_box(), -2., -5., 40., 12.);

    slic3r_api::ClipperOperand unioned = slic3r_api::clipper_union(concatenated);
    check_bbox(unioned.bounding_box(), -2., -5., 40., 12.);
}

// Clipper rounds intersections to 1 unit; its outputs are cleaned at SCALED_EPSILON so that
// callers never see vertices closer than that, from the host helpers or from the plugin API.
TEST_CASE("Clipper outputs merge vertices closer than SCALED_EPSILON", "[plugins][clipper]")
{
    // A 10 mm square whose bottom edge has a vertex SCALED_EPSILON / 2 outside its line, and
    // whose bottom-right corner is doubled by a vertex SCALED_EPSILON / 4 away from it.
    const coord_t side = scale_i(10.);
    Polygon noisy_square({
        Point(0, 0),
        Point(side / 2, -SCALED_EPSILON / 2),
        Point(side - SCALED_EPSILON / 4, 0),
        Point(side, SCALED_EPSILON / 4),
        Point(side, side),
        Point(0, side)
    });
    REQUIRE(noisy_square.is_counter_clockwise());

    ExPolygons unioned = union_ex(Polygons{noisy_square});
    REQUIRE(unioned.size() == 1);
    CHECK(unioned.front().contour.size() == 4);
    CHECK(unioned.front().holes.empty());

    PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    slic3r_api::ClipperContext clipper(storage);
    slic3r_api::ClipperOperand api_unioned = slic3r_api::clipper_union(clipper(view(noisy_square)));
    REQUIRE(api_unioned.path_count() == 1);
    slic3r_api::StoredPolygonCollection api_polygons = api_unioned.to_polygon_collection();
    REQUIRE(api_polygons.size() == 1);
    CHECK(api_polygons[0].size() == 4);
}

TEST_CASE("Clipper outputs drop slivers thinner than SCALED_EPSILON", "[plugins][clipper]")
{
    // Subtracting a copy shifted by `shift` along X leaves a vertical strip `shift` wide.
    const coord_t side = scale_i(10.);
    const Polygon square = rectangle_mm(0., 0., 10., 10.);
    auto shifted_square = [&square](const coord_t shift) {
        Polygon shifted = square;
        shifted.translate(shift, 0);
        return shifted;
    };

    CHECK(diff_ex(Polygons{square}, Polygons{shifted_square(SCALED_EPSILON / 2)}).empty());
    const ExPolygons wide_strip = diff_ex(Polygons{square}, Polygons{shifted_square(4 * SCALED_EPSILON)});
    REQUIRE(wide_strip.size() == 1);
    CHECK(wide_strip.front().contour.size() == 4);
    CHECK(wide_strip.front().contour.bounding_box().size().x() == 4 * SCALED_EPSILON);
    CHECK(wide_strip.front().contour.bounding_box().size().y() == side);

    PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    slic3r_api::ClipperContext clipper(storage);
    const Polygon thin_shift = shifted_square(SCALED_EPSILON / 2);
    const Polygon wide_shift = shifted_square(4 * SCALED_EPSILON);
    CHECK(slic3r_api::clipper_diff(clipper(view(square)), clipper(view(thin_shift))).empty());
    CHECK(slic3r_api::clipper_diff(clipper(view(square)), clipper(view(wide_shift))).path_count() == 1);
}

TEST_CASE("Clipper polyline outputs keep their ends and drop close interior points", "[plugins][clipper]")
{
    // The clip covers x >= 5 mm. The polyline has an interior vertex SCALED_EPSILON / 4
    // before that boundary and slightly off its line, so the cut point lands next to it.
    const coord_t half = scale_i(5.);
    const Polygon clip = rectangle_mm(5., -5., 15., 5.);
    const Polyline polyline({Point(0, 0), Point(half - SCALED_EPSILON / 4, 10), Point(2 * half, 20)});

    const Polylines outside = diff_pl(polyline, clip);
    REQUIRE(outside.size() == 1);
    REQUIRE(outside.front().size() == 2);
    CHECK((outside.front().front() == Point(0, 0) || outside.front().back() == Point(0, 0)));
    const Polylines inside = intersection_pl(polyline, clip);
    REQUIRE(inside.size() == 1);
    CHECK(inside.front().size() == 2);

    // A piece shorter than SCALED_EPSILON keeps its two ends.
    const Polygon thin_clip({Point(half, -half), Point(half + SCALED_EPSILON / 2, -half),
                             Point(half + SCALED_EPSILON / 2, half), Point(half, half)});
    const Polylines tiny = intersection_pl(Polyline({Point(0, 0), Point(2 * half, 0)}), thin_clip);
    REQUIRE(tiny.size() == 1);
    CHECK(tiny.front().size() == 2);
    CHECK(tiny.front().length() == Approx(double(SCALED_EPSILON / 2)));
}

TEST_CASE("Clipper polyline split matches diff and intersection and shares cut points", "[plugins][clipper]")
{
    // A slanted line crossing a square with a hole: inside pieces on both sides of
    // the hole, outside pieces before, in the hole and after.
    ExPolygon area(rectangle_mm(5., -5., 15., 5.));
    area.holes.push_back(rectangle_mm(8., -5. + 3., 12., 5. - 3.));
    area.holes.back().make_clockwise();
    const ExPolygons clip{area};
    const Polyline subject({point_mm(0., -1.), point_mm(20., 1.)});

    PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    const slic3r_api::Polyline subject_view(reinterpret_cast<const polyline_handle *>(&subject));
    const slic3r_api::ExPolygonCollection clip_view(reinterpret_cast<const expolygon_collection_handle *>(&clip));
    auto [inside, outside] = slic3r_api::clipper_split_polyline_expolygons(storage, subject_view, clip_view);

    // Collect each piece as its list of points, sorted so that piece order does not matter.
    auto pieces = [](const slic3r_api::StoredPolylineCollection &collection) {
        std::vector<std::vector<std::pair<coord_t, coord_t>>> out;
        for (uint32_t idx = 0; idx < collection.size(); ++idx) {
            std::vector<std::pair<coord_t, coord_t>> points;
            for (uint32_t pt_idx = 0; pt_idx < collection[idx].size(); ++pt_idx)
                points.emplace_back(collection[idx][pt_idx].x, collection[idx][pt_idx].y);
            out.push_back(std::move(points));
        }
        std::sort(out.begin(), out.end());
        return out;
    };
    REQUIRE(inside.size() == 2);
    REQUIRE(outside.size() == 3);
    CHECK(pieces(inside) == pieces(slic3r_api::clipper_intersection_polyline_expolygons(storage, subject_view, clip_view)));
    CHECK(pieces(outside) == pieces(slic3r_api::clipper_diff_polyline_expolygons(storage, subject_view, clip_view)));

    // Every cut point ends one piece on each side; only the subject ends belong to one side.
    std::multiset<std::pair<coord_t, coord_t>> inside_ends;
    std::multiset<std::pair<coord_t, coord_t>> outside_ends;
    for (const std::vector<std::pair<coord_t, coord_t>> &piece : pieces(inside)) {
        inside_ends.insert(piece.front());
        inside_ends.insert(piece.back());
    }
    for (const std::vector<std::pair<coord_t, coord_t>> &piece : pieces(outside)) {
        outside_ends.insert(piece.front());
        outside_ends.insert(piece.back());
    }
    outside_ends.erase(outside_ends.find({subject.front().x(), subject.front().y()}));
    outside_ends.erase(outside_ends.find({subject.back().x(), subject.back().y()}));
    CHECK(inside_ends == outside_ends);

    // Without clip area, everything is outside.
    const ExPolygons no_clip;
    const slic3r_api::ExPolygonCollection no_clip_view(reinterpret_cast<const expolygon_collection_handle *>(&no_clip));
    auto [nothing_inside, all_outside] = slic3r_api::clipper_split_polyline_expolygons(storage, subject_view, no_clip_view);
    CHECK(nothing_inside.empty());
    REQUIRE(all_outside.size() == 1);
    CHECK(all_outside[0].size() == 2);
}

TEST_CASE("Polyline set split matches intersection and diff", "[plugins][clipper]")
{
    // Two parallel lines crossing a square: each is cut into one inside and two outside pieces.
    const Polylines lines{Polyline({point_mm(0., 2.), point_mm(20., 3.)}), Polyline({point_mm(0., 6.), point_mm(20., 7.)})};
    const Polygons clip{rectangle_mm(5., 0., 15., 10.)};
    auto sorted = [](Polylines polylines) {
        std::vector<Points> out;
        for (Polyline &polyline : polylines)
            out.push_back(std::move(polyline.points));
        std::sort(out.begin(), out.end(), [](const Points &a, const Points &b) {
            return std::lexicographical_compare(a.begin(), a.end(), b.begin(), b.end(),
                                                [](const Point &l, const Point &r) { return l.x() < r.x() || (l.x() == r.x() && l.y() < r.y()); });
        });
        return out;
    };
    auto [inside, outside] = split_pl(lines, clip);
    CHECK(inside.size() == 2);
    CHECK(outside.size() == 4);
    CHECK(sorted(inside) == sorted(intersection_pl(lines, clip)));
    CHECK(sorted(outside) == sorted(diff_pl(lines, clip)));
}

TEST_CASE("ClipperOperand split matches intersection and diff", "[plugins][clipper]")
{
    PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    slic3r_api::ClipperContext clipper(storage);
    const ExPolygon subject = rectangle_with_two_holes();
    const Polygon clip = rectangle_mm(6., -5., 30., 9.);

    auto area = [](const slic3r_api::ClipperOperand &operand) {
        double out = 0.;
        for (const slic3r_api::ExPolygon expolygon : operand.to_expolygon_collection())
            out += expolygon.area();
        return out;
    };
    auto [inside, outside] = slic3r_api::clipper_split(clipper(view(subject)), clipper(view(clip)));
    CHECK(area(inside) == Approx(area(slic3r_api::clipper_intersection(clipper(view(subject)), clipper(view(clip))))));
    CHECK(area(outside) == Approx(area(slic3r_api::clipper_diff(clipper(view(subject)), clipper(view(clip))))));
    CHECK(area(inside) + area(outside) == Approx(subject.area()));
    // The clip cuts through the second hole: a notch on the inside, a notch and the first hole outside.
    CHECK(inside.path_count() == 1);
    CHECK(outside.path_count() == 2);

    // An empty clip leaves the whole subject outside.
    auto [nothing_inside, all_outside] = slic3r_api::clipper_split(clipper(view(subject)), clipper.empty());
    CHECK(nothing_inside.empty());
    CHECK(area(all_outside) == Approx(subject.area()));

    // A polyline operand stays an open path, as with the separate operations: a two-point line
    // crossing the square is not taken as a degenerate polygon.
    const Polyline line({point_mm(-5., 10.), point_mm(25., 10.)});
    const slic3r_api::Polyline line_view(reinterpret_cast<const polyline_handle *>(&line));
    const Polygon square = rectangle_mm(5., 5., 15., 15.);
    auto [line_inside, line_outside] = slic3r_api::clipper_split(clipper(line_view), clipper(view(square)));
    CHECK(line_inside.path_count() == 1);
    CHECK(line_outside.path_count() == 2);
    CHECK(line_inside.path_count() == slic3r_api::clipper_intersection(clipper(line_view), clipper(view(square))).path_count());
    CHECK(line_outside.path_count() == slic3r_api::clipper_diff(clipper(line_view), clipper(view(square))).path_count());
}

TEST_CASE("Clipper polyline pieces cut less than SCALED_EPSILON apart are glued back", "[plugins][clipper]")
{
    const coord_t half = scale_i(5.);
    const Polyline line({Point(0, 0), Point(2 * half, 0)});
    // A vertical strip SCALED_EPSILON / 2 wide crossing the middle of the line.
    const Polygon thin_strip({Point(half, -half), Point(half + SCALED_EPSILON / 2, -half),
                              Point(half + SCALED_EPSILON / 2, half), Point(half, half)});

    // diff: the two outside pieces become one again, from end to end.
    const Polylines outside = diff_pl(line, thin_strip);
    REQUIRE(outside.size() == 1);
    CHECK(outside.front().length() == Approx(double(2 * half)).epsilon(1e-6));

    // intersection: two areas separated by a gap SCALED_EPSILON / 2 wide give one piece.
    const ExPolygons two_areas{ExPolygon(rectangle_mm(-1., -1., 5., 1.)),
                               ExPolygon(Polygon({Point(half + SCALED_EPSILON / 2, scale_i(-1.)), Point(scale_i(11.), scale_i(-1.)),
                                                  Point(scale_i(11.), scale_i(1.)), Point(half + SCALED_EPSILON / 2, scale_i(1.))}))};
    const Polylines inside = intersection_pl(line, two_areas);
    REQUIRE(inside.size() == 1);
    CHECK(inside.front().length() == Approx(double(2 * half)).epsilon(1e-6));

    // split: the outside is glued and the tiny inside piece between the two cuts is dropped.
    auto [split_inside, split_outside] = split_pl(line, ExPolygons{ExPolygon(thin_strip)});
    CHECK(split_inside.empty());
    REQUIRE(split_outside.size() == 1);
    CHECK(split_outside.front().length() == Approx(double(2 * half)).epsilon(1e-6));

    // Ends close in the plane but far apart along the subject are not glued: this path comes back
    // SCALED_EPSILON / 2 above its start after a 34 mm detour inside the clip.
    const Polyline detour({point_mm(-5., 0.), point_mm(10., 0.), point_mm(10., 10.), Point(0, SCALED_EPSILON / 2),
                           Point(scale_i(-5.), SCALED_EPSILON / 2)});
    const ExPolygons detour_clip{ExPolygon(rectangle_mm(0., -1., 20., 20.))};
    CHECK(diff_pl(detour, detour_clip).size() == 2);
    auto [detour_inside, detour_outside] = split_pl(detour, detour_clip);
    REQUIRE(detour_inside.size() == 1);
    CHECK(detour_inside.front().length() == Approx(scale_d(20. + std::sqrt(200.))).epsilon(1e-4));
    CHECK(detour_outside.size() == 2);

    // A loop given as a polyline keeps its start: its two ends are never glued together.
    const Polyline loop({Point(0, 0), Point(2 * half, 0), Point(2 * half, 2 * half), Point(0, 2 * half), Point(0, 0)});
    const Polygon right_side({Point(half, -half), Point(3 * half, -half), Point(3 * half, 3 * half), Point(half, 3 * half)});
    CHECK(diff_pl(loop, right_side).size() == 2);
}

// A fully covered V retains all vertices and the subject direction in each single-path operation.
TEST_CASE("Clipper single subject keeps V direction", "[plugins][clipper][rejoin-direction]")
{
    const Polyline subject({point_mm(0., 0.), point_mm(5., 10.), point_mm(10., 0.)});
    const ExPolygons cover{ExPolygon(rectangle_mm(-1., -1., 11., 11.))};
    const ExPolygons away{ExPolygon(rectangle_mm(20., 20., 30., 30.))};
    const Polylines inside = intersection_pl(subject, cover);
    const Polylines outside = diff_pl(subject, away);
    const std::pair<Polylines, Polylines> split = split_pl(subject, cover);
    REQUIRE(inside.size() == 1);
    REQUIRE(outside.size() == 1);
    REQUIRE(split.first.size() == 1);
    CHECK(split.second.empty());
    CHECK(inside[0].points == subject.points);
    CHECK(outside[0].points == subject.points);
    CHECK(split.first[0].points == subject.points);
}

// Clipper emits both raw V fragments from larger to smaller X; the tiny ascending-branch cut rejoins
// them in subject direction without reversing the stored first fragment again.
TEST_CASE("Clipper consecutive reversed pieces join in subject direction", "[plugins][clipper][rejoin-direction]")
{
    const coord_t half = scale_i(5.);
    const Polyline subject({Point(0, 0), Point(half, 2 * half), Point(2 * half, 0)});
    const coord_t cut = half / 2;
    const Polygon strip({Point(cut, -half), Point(cut + SCALED_EPSILON / 4, -half),
                         Point(cut + SCALED_EPSILON / 4, 3 * half), Point(cut, 3 * half)});
    const Polylines outside = diff_pl(subject, strip);
    REQUIRE(outside.size() == 1);
    CHECK(outside[0].front() == subject.front());
    CHECK(outside[0].back() == subject.back());
    CHECK(outside[0].length() == Approx(subject.length()).epsilon(1e-6));
    for (size_t idx = 1; idx < outside[0].size(); ++idx) {
        CHECK(outside[0].points[idx].x() > outside[0].points[idx - 1].x());
        if (outside[0].points[idx].x() <= half)
            CHECK(outside[0].points[idx].y() > outside[0].points[idx - 1].y());
        else
            CHECK(outside[0].points[idx].y() < outside[0].points[idx - 1].y());
    }
}

// A closed polyline keeps its seam (the repeated start point) and its actual piece order.
TEST_CASE("Clipper closed subject places three pieces along the loop", "[plugins][clipper][rejoin-direction]")
{
    const Polyline subject({point_mm(0., 0.), point_mm(10., 0.), point_mm(10., 10.),
                            point_mm(0., 10.), point_mm(0., 0.)});
    const ExPolygons clip{ExPolygon(rectangle_mm(5., -1., 11., 11.))};
    const std::pair<Polylines, Polylines> split = split_pl(subject, clip);
    REQUIRE(split.first.size() == 1);
    REQUIRE(split.second.size() == 2);
    const Points first{point_mm(0., 0.), point_mm(5., 0.)};
    const Points middle{point_mm(5., 0.), point_mm(10., 0.), point_mm(10., 10.), point_mm(5., 10.)};
    const Points last{point_mm(5., 10.), point_mm(0., 10.), point_mm(0., 0.)};
    CHECK(split.second[0].points == first);
    CHECK(split.first[0].points == middle);
    CHECK(split.second[1].points == last);
    CHECK(split.second[0].back() == split.first[0].front());
    CHECK(split.first[0].back() == split.second[1].front());
    CHECK(split.second[1].back() == split.second[0].front());
    CHECK(split.second[0].length() + split.first[0].length() + split.second[1].length() == Approx(subject.length()));

    // Full coverage keeps the repeated start as the arrival at the total loop length.
    const Polylines covered = intersection_pl(subject, rectangle_mm(-1., -1., 11., 11.));
    REQUIRE(covered.size() == 1);
    CHECK(covered[0].points == subject.points);

    // Polygon overloads recombine the two seam fragments without reversing their direction.
    const Polygon polygon({point_mm(0., 0.), point_mm(10., 0.), point_mm(10., 10.), point_mm(0., 10.)});
    const Polylines recombined = diff_pl(Polygons{polygon}, Polygons{clip[0].contour});
    REQUIRE(recombined.size() == 1);
    const Points around_seam{point_mm(5., 10.), point_mm(0., 10.), point_mm(0., 0.), point_mm(5., 0.)};
    CHECK(recombined[0].points == around_seam);
}

// A tiny strip across the last side is glued outside and removed inside, away from the seam.
TEST_CASE("Clipper closed subject drops a tiny last-side piece", "[plugins][clipper][rejoin-direction]")
{
    const coord_t half = scale_i(5.);
    const Polyline subject({Point(0, 0), Point(2 * half, 0), Point(2 * half, 2 * half),
                            Point(0, 2 * half), Point(0, 0)});
    const Polygon strip({Point(-half, half), Point(half, half),
                         Point(half, half + SCALED_EPSILON / 2), Point(-half, half + SCALED_EPSILON / 2)});
    const std::pair<Polylines, Polylines> split = split_pl(subject, ExPolygons{ExPolygon(strip)});
    CHECK(split.first.empty());
    REQUIRE(split.second.size() == 1);
    CHECK(split.second[0].front() == subject.front());
    CHECK(split.second[0].back() == subject.back());
    REQUIRE(split.second[0].size() >= subject.size());
    CHECK(split.second[0].points[1] == subject.points[1]);
    CHECK(split.second[0].points[2] == subject.points[2]);
    CHECK(split.second[0].points[3] == subject.points[3]);
    CHECK(split.second[0].length() == Approx(subject.length()).epsilon(1e-6));
}

TEST_CASE("Clipper offset grows a polygon list of clockwise rings like ClipperOffset", "[plugins][clipper]")
{
    // Polygon lists keep the grouped ClipperOffset: painted facet slivers may
    // have any orientation, and a clockwise lowest ring reverses every ring,
    // so the set grows instead of being read as holes without a contour.
    PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    slic3r_api::ClipperContext clipper(storage);
    Polygon first = rectangle_mm(0., 0., 10., 10.);
    Polygon second = rectangle_mm(20., 0., 30., 10.);
    first.make_clockwise();
    second.make_clockwise();
    slic3r_api::ClipperOperand clockwise = clipper(view(first));
    clockwise += clipper(view(second));

    const slic3r_api::ClipperOperand grown =
        slic3r_api::clipper_offset(clockwise, scale_d(1.), CLIPPER_JOIN_MITER, 3.);
    CHECK(grown.to_expolygon_collection().size() == 2);
    check_bbox(grown.bounding_box(), -1., -1., 31., 11.);
}


// ---- Native ExPolygon functions of slic3r_clipper.h 2.0 -------------------------------------------
// clipper_union_ex, clipper_offset_ex and clipper_offset2_ex must return exactly what the native
// ClipperUtils returns: same sizes, same order of expolygons, same order of contour and holes, same
// points (no rotation, no sorting, no canonicalisation).
namespace {

const polygon_collection_handle *polygons_handle(const Polygons &polygons)
{
    return reinterpret_cast<const polygon_collection_handle *>(&polygons);
}

const expolygon_collection_handle *expolygons_handle(const ExPolygons &expolygons)
{
    return reinterpret_cast<const expolygon_collection_handle *>(&expolygons);
}

const ExPolygons &native_expolygons(const expolygon_collection_handle *handle)
{
    REQUIRE(handle != nullptr);
    return *reinterpret_cast<const ExPolygons *>(handle);
}

// Strict equality: every size, every order and every point.
void check_identical(const ExPolygons &actual, const ExPolygons &expected)
{
    REQUIRE(actual.size() == expected.size());
    for (size_t i = 0; i < expected.size(); ++i) {
        INFO("expolygon " << i);
        CHECK(actual[i].contour.points == expected[i].contour.points);
        REQUIRE(actual[i].holes.size() == expected[i].holes.size());
        for (size_t h = 0; h < expected[i].holes.size(); ++h)
            CHECK(actual[i].holes[h].points == expected[i].holes[h].points);
    }
}

ClipperLib::PolyFillType native_fill(const clipper_fill_rule_t rule)
{
    switch (rule) {
    case CLIPPER_FILL_EVENODD:  return ClipperLib::pftEvenOdd;
    case CLIPPER_FILL_POSITIVE: return ClipperLib::pftPositive;
    default:                    return ClipperLib::pftNonZero;
    }
}

ClipperLib::JoinType native_join_type(const clipper_join_type_t join)
{
    switch (join) {
    case CLIPPER_JOIN_SQUARE: return ClipperLib::jtSquare;
    case CLIPPER_JOIN_ROUND:  return ClipperLib::jtRound;
    default:                  return ClipperLib::jtMiter;
    }
}

// A single Clipper pass over the polygons, fill rule applied to subject and clip, converted by the same
// ClipperShapes code the host uses for clipper_union(): the reference of the shapes pipeline.
ExPolygons single_pass_union(const Polygons &polygons, const ClipperLib::PolyFillType fill)
{
    ClipperLib::Paths paths;
    for (const Polygon &polygon : polygons)
        paths.emplace_back(polygon.points);
    ClipperLib::Clipper clipper;
    clipper.AddPaths(paths, ClipperLib::ptSubject, true);
    ClipperLib::PolyTree tree;
    clipper.Execute(ClipperLib::ctUnion, tree, fill, fill);
    return ApiClipper::make_polytree_shapes(std::move(tree))->to_expolygons();
}

Polygon clockwise(Polygon polygon)
{
    polygon.make_clockwise();
    return polygon;
}

const clipper_fill_rule_t all_fill_rules[] = {CLIPPER_FILL_NONZERO, CLIPPER_FILL_EVENODD, CLIPPER_FILL_POSITIVE};

struct NamedPolygons {
    const char *name;
    Polygons    polygons;
};

std::vector<NamedPolygons> union_inputs()
{
    // A thin sliver and a tiny square, both touching the first rectangle: the union itself may absorb them,
    // and the native conversion drops what remains degenerate. The isolated small ring that only
    // remove_small_areas can drop has its own test.
    const Polygon sliver({Point(0, 0), Point(scale_i(30.), 0), Point(scale_i(30.), 2), Point(0, 2)});
    const Polygon tiny({Point(0, 0), Point(20, 0), Point(20, 20), Point(0, 20)});
    return {
        {"overlapping rings", {rectangle_mm(0., 0., 10., 10.), rectangle_mm(5., 5., 15., 15.), rectangle_mm(12., 0., 20., 8.)}},
        {"nested rings of the same orientation", {rectangle_mm(0., 0., 20., 20.), rectangle_mm(5., 5., 15., 15.)}},
        {"clockwise rings", {clockwise(rectangle_mm(0., 0., 10., 10.)), clockwise(rectangle_mm(5., 5., 15., 15.)),
                             clockwise(rectangle_mm(30., 0., 40., 10.))}},
        {"nested clockwise rings", {clockwise(rectangle_mm(0., 0., 20., 20.)), clockwise(rectangle_mm(5., 5., 15., 15.))}},
        {"tiny and thin rings", {rectangle_mm(0., 0., 10., 10.), sliver, tiny, rectangle_mm(50., 50., 60., 60.)}},
        {"contour, hole and island", {rectangle_mm(0., 0., 30., 30.), clockwise(rectangle_mm(5., 5., 25., 25.)),
                                      rectangle_mm(10., 10., 20., 20.)}},
        {"mixed orientations", {rectangle_mm(0., 0., 10., 10.), clockwise(rectangle_mm(5., 5., 15., 15.)),
                                rectangle_mm(20., 0., 30., 10.), rectangle_mm(25., 5., 35., 15.)}},
        {"empty", {}},
    };
}

ExPolygon l_shape()
{
    return ExPolygon(Polygon({point_mm(0., 0.), point_mm(20., 0.), point_mm(20., 8.), point_mm(8., 8.),
                              point_mm(8., 20.), point_mm(0., 20.)}));
}

struct NamedExPolygons {
    const char *name;
    ExPolygons  expolygons;
};

std::vector<NamedExPolygons> offset_inputs()
{
    ExPolygon nested(rectangle_mm(0., 0., 40., 40.));
    nested.holes.push_back(clockwise(rectangle_mm(8., 8., 32., 32.)));
    ExPolygon island(rectangle_mm(14., 14., 26., 26.));
    return {
        {"square", {ExPolygon(rectangle_mm(0., 0., 20., 20.))}},
        {"rectangle with two holes", {rectangle_with_two_holes()}},
        {"l shape", {l_shape()}},
        {"touching expolygons", {ExPolygon(rectangle_mm(0., 0., 10., 10.)), ExPolygon(rectangle_mm(10., 0., 20., 10.))}},
        {"expolygons 0.5 mm apart", {ExPolygon(rectangle_mm(0., 0., 10., 10.)), ExPolygon(rectangle_mm(10.5, 0., 20., 10.))}},
        {"expolygon and its island", {nested, ExPolygon(island)}},
        {"empty", {}},
    };
}

} // namespace

TEST_CASE("clipper_union_ex returns the native union_ex for every fill rule", "[plugins][clipper][clipper-native]")
{
    PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    for (const NamedPolygons &input : union_inputs()) {
        for (const clipper_fill_rule_t rule : all_fill_rules) {
            INFO(input.name << ", fill rule " << int(rule));
            const ExPolygons expected = union_ex(input.polygons, native_fill(rule));
            check_identical(native_expolygons(clipper_union_ex(storage, polygons_handle(input.polygons), rule)), expected);
        }
    }
}

TEST_CASE("clipper_offset_ex returns the native offset_ex", "[plugins][clipper][clipper-native]")
{
    PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    const struct { clipper_join_type_t join; double miter; } joins[] = {
        {CLIPPER_JOIN_MITER, 3.}, {CLIPPER_JOIN_SQUARE, 3.}, {CLIPPER_JOIN_ROUND, scale_d(0.01)}};
    const double deltas[] = {scale_d(1.), scale_d(-1.), scale_d(2.5), scale_d(-3.), scale_d(0.0013), scale_d(-0.0007)};
    for (const NamedExPolygons &input : offset_inputs()) {
        for (const auto &join : joins) {
            for (const double delta : deltas) {
                INFO(input.name << ", join " << int(join.join) << ", delta " << delta);
                check_identical(native_expolygons(clipper_offset_ex(storage, expolygons_handle(input.expolygons), delta,
                                                                    join.join, join.miter)),
                                offset_ex(input.expolygons, delta, native_join_type(join.join), join.miter));
            }
        }
        // The native defaults and the safety offset.
        INFO(input.name << ", safety offset");
        check_identical(native_expolygons(clipper_offset_ex(storage, expolygons_handle(input.expolygons), ClipperSafetyOffset,
                                                            CLIPPER_JOIN_MITER, 3.0)),
                        union_safety_offset_ex(input.expolygons));
        check_identical(native_expolygons(clipper_offset_ex(storage, expolygons_handle(input.expolygons), scale_d(1.),
                                                            CLIPPER_JOIN_MITER, 3.0)),
                        offset_ex(input.expolygons, scale_d(1.)));
    }
}

TEST_CASE("clipper_offset2_ex returns the native offset2_ex for closing and opening", "[plugins][clipper][clipper-native]")
{
    PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    const struct { double delta1; double delta2; } passes[] = {
        {scale_d(1.), scale_d(-1.)},      // closing
        {scale_d(-1.), scale_d(1.)},      // opening
        {scale_d(3.), scale_d(-3.)},      // closing that merges the 0.5 mm gap
        {scale_d(-4.), scale_d(4.)},      // opening that removes the thin parts
        {scale_d(2.), scale_d(-1.)},      // unbalanced
        {scale_d(0.0013), scale_d(-0.0007)}};
    const clipper_join_type_t joins[] = {CLIPPER_JOIN_MITER, CLIPPER_JOIN_SQUARE, CLIPPER_JOIN_ROUND};
    for (const NamedExPolygons &input : offset_inputs()) {
        for (const clipper_join_type_t join : joins) {
            const double miter = join == CLIPPER_JOIN_ROUND ? scale_d(0.01) : 3.;
            for (const auto &pass : passes) {
                INFO(input.name << ", join " << int(join) << ", deltas " << pass.delta1 << " " << pass.delta2);
                check_identical(native_expolygons(clipper_offset2_ex(storage, expolygons_handle(input.expolygons),
                                                                     pass.delta1, pass.delta2, join, miter)),
                                offset2_ex(input.expolygons, pass.delta1, pass.delta2, native_join_type(join), miter));
            }
        }
        INFO(input.name << ", default join");
        check_identical(native_expolygons(clipper_offset2_ex(storage, expolygons_handle(input.expolygons), scale_d(1.),
                                                             scale_d(-1.), CLIPPER_JOIN_MITER, 3.0)),
                        offset2_ex(input.expolygons, scale_d(1.), scale_d(-1.)));
    }
}

TEST_CASE("clipper_union applies the fill rule and NONZERO keeps the previous behaviour", "[plugins][clipper][clipper-native]")
{
    PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    for (const NamedPolygons &input : union_inputs()) {
        const clipper_shapes_handle *subject = clipper_shapes_from_polygons(storage, polygons_handle(input.polygons));
        REQUIRE(subject != nullptr);
        for (const clipper_fill_rule_t rule : all_fill_rules) {
            INFO(input.name << ", fill rule " << int(rule));
            clipper_shapes_handle *united = clipper_union(storage, subject, rule);
            REQUIRE(united != nullptr);
            check_identical(native_expolygons(clipper_shapes_to_expolygons(storage, united)),
                            single_pass_union(input.polygons, native_fill(rule)));
        }
        // NONZERO is the rule of the C++ clipper_union(), exactly as before the rule argument existed.
        clipper_shapes_handle *explicit_rule = clipper_union(storage, subject, CLIPPER_FILL_NONZERO);
        clipper_shapes_handle *old_behaviour = clipper_execute(storage, CLIPPER_OPERATION_UNION, subject, nullptr);
        REQUIRE(explicit_rule != nullptr);
        REQUIRE(old_behaviour != nullptr);
        INFO(input.name << ", NONZERO against clipper_execute");
        check_identical(native_expolygons(clipper_shapes_to_expolygons(storage, explicit_rule)),
                        native_expolygons(clipper_shapes_to_expolygons(storage, old_behaviour)));
    }
}

TEST_CASE("clipper_union fill rules give different results where the rules differ", "[plugins][clipper][clipper-native]")
{
    PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    slic3r_api::ClipperContext clipper(storage);
    // Two nested counter-clockwise squares: NONZERO and POSITIVE fill both, EVENODD leaves a hole.
    const Polygons nested{rectangle_mm(0., 0., 20., 20.), rectangle_mm(5., 5., 15., 15.)};
    const slic3r_api::PolygonCollection view_nested(polygons_handle(nested));
    const slic3r_api::ClipperOperand operand = clipper(view_nested);
    CHECK(slic3r_api::clipper_union(operand).to_expolygon_collection().size() == 1);
    CHECK(slic3r_api::clipper_union_with_fill_rule(operand, CLIPPER_FILL_NONZERO).path_count() == 1);
    CHECK(slic3r_api::clipper_union_with_fill_rule(operand, CLIPPER_FILL_POSITIVE).path_count() == 1);
    CHECK(slic3r_api::clipper_union_with_fill_rule(operand, CLIPPER_FILL_EVENODD).path_count() == 2);

    // The C++ twins give the native result too.
    const slic3r_api::StoredExPolygonCollection evenodd = slic3r_api::clipper_union_ex(storage, view_nested, CLIPPER_FILL_EVENODD);
    check_identical(native_expolygons(evenodd.handle()), union_ex(nested, ClipperLib::pftEvenOdd));
}

TEST_CASE("clipper_union_ex drops an isolated small ring that a single pass keeps", "[plugins][clipper][clipper-native]")
{
    PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    // An equilateral triangle of side 150 units, far from the other ring. Every vertex is farther than
    // SCALED_EPSILON from the line through its neighbours, so the clean-up of the Clipper pass keeps the ring,
    // but its area (~9700) is below 2 * SCALED_EPSILON * its radius (~17000): the native remove_small_areas
    // drops it.
    const coord_t x0 = scale_i(50.), y0 = scale_i(50.);
    const Polygon small_ring({Point(x0, y0), Point(x0 + 150, y0), Point(x0 + 75, y0 + 130)});
    REQUIRE(small_ring.is_counter_clockwise());
    const Polygons polygons{rectangle_mm(0., 0., 10., 10.), small_ring};
    for (const clipper_fill_rule_t rule : all_fill_rules) {
        INFO("fill rule " << int(rule));
        // A raw single Clipper pass (no ClipperShapes conversion, which cleans its output) keeps both rings.
        ClipperLib::Paths raw_paths;
        for (const Polygon &polygon : polygons)
            raw_paths.emplace_back(polygon.points);
        ClipperLib::Clipper raw_clipper;
        raw_clipper.AddPaths(raw_paths, ClipperLib::ptSubject, true);
        ClipperLib::Paths single_pass;
        raw_clipper.Execute(ClipperLib::ctUnion, single_pass, native_fill(rule), native_fill(rule));
        CHECK(single_pass.size() == 2);
        const ExPolygons &united = native_expolygons(clipper_union_ex(storage, polygons_handle(polygons), rule));
        REQUIRE(united.size() == 1);
        CHECK(united.front().contour.bounding_box().size().x() == scale_i(10.));
        check_identical(united, union_ex(polygons, native_fill(rule)));
    }
}

TEST_CASE("clipper_offset_ex handles zero delta, acute corners and every join like the native code", "[plugins][clipper][clipper-native]")
{
    PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    // A triangle with a 4 degree corner: the join type and the miter limit change the result.
    const ExPolygons acute{ExPolygon(Polygon({Point(0, 0), Point(scale_i(20.), 0), Point(scale_i(20.), scale_i(1.5))}))};
    const struct { clipper_join_type_t join; double miter; } joins[] = {
        {CLIPPER_JOIN_MITER, 3.}, {CLIPPER_JOIN_MITER, 100.}, {CLIPPER_JOIN_SQUARE, 3.},
        {CLIPPER_JOIN_ROUND, scale_d(0.01)}, {CLIPPER_JOIN_ROUND, scale_d(0.2)}};
    std::vector<BoundingBox> boxes;
    for (const auto &join : joins) {
        INFO("join " << int(join.join) << ", limit " << join.miter);
        const ExPolygons &actual = native_expolygons(clipper_offset_ex(storage, expolygons_handle(acute), scale_d(1.),
                                                                       join.join, join.miter));
        check_identical(actual, offset_ex(acute, scale_d(1.), native_join_type(join.join), join.miter));
        check_identical(native_expolygons(clipper_offset2_ex(storage, expolygons_handle(acute), scale_d(1.), -scale_d(0.5),
                                                             join.join, join.miter)),
                        offset2_ex(acute, scale_d(1.), -scale_d(0.5), native_join_type(join.join), join.miter));
        REQUIRE(actual.size() == 1);
        boxes.push_back(get_extents(actual.front()));
    }
    // The miter limit squares the sharp corner at 3 but not at 100, and a round join differs too.
    CHECK(boxes[0].min.x() > boxes[1].min.x());
    CHECK(boxes[0].min != boxes[3].min);

    // A zero delta is accepted, as by the native offset_ex.
    for (const NamedExPolygons &input : offset_inputs()) {
        INFO(input.name << ", delta 0");
        check_identical(native_expolygons(clipper_offset_ex(storage, expolygons_handle(input.expolygons), 0.,
                                                            CLIPPER_JOIN_MITER, 3.)),
                        offset_ex(input.expolygons, 0., ClipperLib::jtMiter, 3.));
    }
}

TEST_CASE("clipper_union_ex, clipper_offset2_ex and clipper_offset_ex chain like the native slicing post-processing",
          "[plugins][clipper][clipper-native]")
{
    PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    const struct { double closing; double extra; } settings[] = {
        {scale_d(1.), scale_d(0.3)}, {scale_d(1.), scale_d(-0.3)}, {-scale_d(1.), scale_d(0.3)}, {-scale_d(1.), scale_d(-0.3)}};
    for (const NamedPolygons &input : union_inputs()) {
        for (const clipper_fill_rule_t rule : all_fill_rules) {
            for (const auto &setting : settings) {
                INFO(input.name << ", fill rule " << int(rule) << ", closing " << setting.closing << ", extra " << setting.extra);
                // As make_expolygons of the slicer: union, then offset2 (closing or opening), then offset.
                const ExPolygons expected = offset_ex(
                    offset2_ex(union_ex(input.polygons, native_fill(rule)), setting.closing, -setting.closing),
                    setting.extra);
                const expolygon_collection_handle *united = clipper_union_ex(storage, polygons_handle(input.polygons), rule);
                REQUIRE(united != nullptr);
                const expolygon_collection_handle *closed =
                    clipper_offset2_ex(storage, united, setting.closing, -setting.closing, CLIPPER_JOIN_MITER, 3.);
                REQUIRE(closed != nullptr);
                const expolygon_collection_handle *grown = clipper_offset_ex(storage, closed, setting.extra, CLIPPER_JOIN_MITER, 3.);
                check_identical(native_expolygons(grown), expected);
            }
        }
    }
}

TEST_CASE("Native ExPolygon results are storage-owned and can be released", "[plugins][clipper][clipper-native]")
{
    PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    const Polygons polygons{rectangle_mm(0., 0., 10., 10.)};
    const ExPolygons expolygons{ExPolygon(rectangle_mm(0., 0., 10., 10.))};
    expolygon_collection_handle *results[] = {
        clipper_union_ex(storage, polygons_handle(polygons), CLIPPER_FILL_NONZERO),
        clipper_offset_ex(storage, expolygons_handle(expolygons), scale_d(1.), CLIPPER_JOIN_MITER, 3.),
        clipper_offset2_ex(storage, expolygons_handle(expolygons), scale_d(1.), -scale_d(1.), CLIPPER_JOIN_MITER, 3.)};
    for (expolygon_collection_handle *result : results) {
        REQUIRE(result != nullptr);
        CHECK(storage_free(storage, result) != 0);
    }
}

TEST_CASE("Native ExPolygon clipper functions reject NULL inputs and unknown enums", "[plugins][clipper][clipper-native]")
{
    PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    const Polygons polygons{rectangle_mm(0., 0., 10., 10.)};
    const ExPolygons expolygons{ExPolygon(rectangle_mm(0., 0., 10., 10.))};
    const clipper_shapes_handle *subject = clipper_shapes_from_polygons(storage, polygons_handle(polygons));
    REQUIRE(subject != nullptr);
    // 3 is inside the value range of the enums but is not an enumerator.
    const clipper_fill_rule_t unknown_rule = static_cast<clipper_fill_rule_t>(3);
    const clipper_join_type_t unknown_join = static_cast<clipper_join_type_t>(3);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();

    // C: NULL.
    CHECK(clipper_union(nullptr, subject, CLIPPER_FILL_NONZERO) == nullptr);
    CHECK(clipper_union(storage, nullptr, CLIPPER_FILL_NONZERO) == nullptr);
    CHECK(clipper_union(storage, subject, unknown_rule) == nullptr);
    CHECK(clipper_union_ex(nullptr, polygons_handle(polygons), CLIPPER_FILL_NONZERO) == nullptr);
    CHECK(clipper_union_ex(storage, nullptr, CLIPPER_FILL_NONZERO) == nullptr);
    CHECK(clipper_union_ex(storage, polygons_handle(polygons), unknown_rule) == nullptr);
    CHECK(clipper_offset_ex(nullptr, expolygons_handle(expolygons), 1., CLIPPER_JOIN_MITER, 3.) == nullptr);
    CHECK(clipper_offset_ex(storage, nullptr, 1., CLIPPER_JOIN_MITER, 3.) == nullptr);
    CHECK(clipper_offset_ex(storage, expolygons_handle(expolygons), 1., unknown_join, 3.) == nullptr);
    CHECK(clipper_offset2_ex(nullptr, expolygons_handle(expolygons), 1., -1., CLIPPER_JOIN_MITER, 3.) == nullptr);
    CHECK(clipper_offset2_ex(storage, nullptr, 1., -1., CLIPPER_JOIN_MITER, 3.) == nullptr);
    CHECK(clipper_offset2_ex(storage, expolygons_handle(expolygons), 1., -1., unknown_join, 3.) == nullptr);

    // C: non-finite numbers, and delta2 == 0 for offset2 only.
    for (const double bad : {nan, inf, -inf}) {
        CHECK(clipper_offset_ex(storage, expolygons_handle(expolygons), bad, CLIPPER_JOIN_MITER, 3.) == nullptr);
        CHECK(clipper_offset_ex(storage, expolygons_handle(expolygons), 1., CLIPPER_JOIN_MITER, bad) == nullptr);
        CHECK(clipper_offset2_ex(storage, expolygons_handle(expolygons), bad, -1., CLIPPER_JOIN_MITER, 3.) == nullptr);
        CHECK(clipper_offset2_ex(storage, expolygons_handle(expolygons), 1., bad, CLIPPER_JOIN_MITER, 3.) == nullptr);
        CHECK(clipper_offset2_ex(storage, expolygons_handle(expolygons), 1., -1., CLIPPER_JOIN_MITER, bad) == nullptr);
    }
    CHECK(clipper_offset2_ex(storage, expolygons_handle(expolygons), 1., 0., CLIPPER_JOIN_MITER, 3.) == nullptr);
    CHECK(clipper_offset2_ex(storage, expolygons_handle(expolygons), 1., -0., CLIPPER_JOIN_MITER, 3.) == nullptr);
    CHECK(clipper_offset_ex(storage, expolygons_handle(expolygons), 0., CLIPPER_JOIN_MITER, 3.) != nullptr);

    // C++: std::invalid_argument, never an adopted NULL.
    const slic3r_api::PolygonCollection polygon_view(polygons_handle(polygons));
    const slic3r_api::ExPolygonCollection expolygon_view(expolygons_handle(expolygons));
    slic3r_api::ClipperContext clipper(storage);
    const slic3r_api::ClipperOperand operand = clipper(polygon_view);
    CHECK_THROWS_AS(slic3r_api::clipper_union_with_fill_rule(operand, unknown_rule), std::invalid_argument);
    // A NULL storage throws even for an empty subject (no create_empty(nullptr)).
    const slic3r_api::ClipperOperand empty_operand = slic3r_api::ClipperOperand::create_empty(storage);
    CHECK_THROWS_AS(slic3r_api::clipper_union_with_fill_rule(nullptr, empty_operand, CLIPPER_FILL_NONZERO), std::invalid_argument);
    CHECK_THROWS_AS(slic3r_api::clipper_union_with_fill_rule(nullptr, operand, CLIPPER_FILL_NONZERO), std::invalid_argument);
    CHECK_THROWS_AS(slic3r_api::clipper_union_with_fill_rule(storage, empty_operand, unknown_rule), std::invalid_argument);
    CHECK_THROWS_AS(slic3r_api::clipper_union_ex(storage, polygon_view, unknown_rule), std::invalid_argument);
    CHECK_THROWS_AS(slic3r_api::clipper_union_ex(nullptr, polygon_view, CLIPPER_FILL_NONZERO), std::invalid_argument);
    CHECK_THROWS_AS(slic3r_api::clipper_offset_ex(storage, expolygon_view, 1., unknown_join), std::invalid_argument);
    CHECK_THROWS_AS(slic3r_api::clipper_offset_ex(nullptr, expolygon_view, 1.), std::invalid_argument);
    CHECK_THROWS_AS(slic3r_api::clipper_offset_ex(storage, expolygon_view, nan), std::invalid_argument);
    CHECK_THROWS_AS(slic3r_api::clipper_offset_ex(storage, expolygon_view, inf), std::invalid_argument);
    CHECK_THROWS_AS(slic3r_api::clipper_offset2_ex(storage, expolygon_view, 1., -1., unknown_join), std::invalid_argument);
    CHECK_THROWS_AS(slic3r_api::clipper_offset2_ex(nullptr, expolygon_view, 1., -1.), std::invalid_argument);
    CHECK_THROWS_AS(slic3r_api::clipper_offset2_ex(storage, expolygon_view, 1., 0.), std::invalid_argument);
    CHECK_THROWS_AS(slic3r_api::clipper_offset2_ex(storage, expolygon_view, nan, -1.), std::invalid_argument);
    CHECK_THROWS_AS(slic3r_api::clipper_offset2_ex(storage, expolygon_view, 1., inf), std::invalid_argument);
}
// A coordinate beyond ClipperLib::hiRange makes ClipperLib throw "Coordinate outside allowed range". The native _ex
// functions must turn it into a NULL result, without leaving anything in the storage.
TEST_CASE("Native ExPolygon clipper functions return NULL on a native exception and leave the storage unchanged",
          "[plugins][clipper][clipper-native]")
{
    PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    const coord_t out_of_range = coord_t(0x4000000000000000LL);
    static_assert(sizeof(coord_t) == 8, "the test needs 64-bit coordinates");
    REQUIRE(out_of_range > ClipperLib::hiRange);
    const Polygon huge({Point(0, 0), Point(out_of_range, 0), Point(0, out_of_range)});
    const Polygons polygons{huge};
    const ExPolygons expolygons{ExPolygon(huge)};
    const size_t count_before = plugin_storage.size();

    // The native functions themselves throw the range exception: it is the origin of the NULL results below.
    CHECK_THROWS_WITH(union_ex(polygons, ClipperLib::pftNonZero), "Coordinate outside allowed range");
    CHECK_THROWS_WITH(offset_ex(expolygons, scale_d(1.), ClipperLib::jtMiter, 3.), "Coordinate outside allowed range");
    CHECK_THROWS_WITH(offset2_ex(expolygons, scale_d(1.), -scale_d(1.), ClipperLib::jtMiter, 3.), "Coordinate outside allowed range");
    CHECK(clipper_union_ex(storage, polygons_handle(polygons), CLIPPER_FILL_NONZERO) == nullptr);
    CHECK(clipper_offset_ex(storage, expolygons_handle(expolygons), scale_d(1.), CLIPPER_JOIN_MITER, 3.) == nullptr);
    CHECK(clipper_offset2_ex(storage, expolygons_handle(expolygons), scale_d(1.), -scale_d(1.), CLIPPER_JOIN_MITER, 3.) == nullptr);
    CHECK(plugin_storage.size() == count_before);

    // C++ wrappers: an internal error is a runtime_error, an argument error stays an invalid_argument.
    const slic3r_api::PolygonCollection polygon_view(polygons_handle(polygons));
    const slic3r_api::ExPolygonCollection expolygon_view(expolygons_handle(expolygons));
    CHECK_THROWS_AS(slic3r_api::clipper_union_ex(storage, polygon_view, CLIPPER_FILL_NONZERO), std::runtime_error);
    CHECK_THROWS_AS(slic3r_api::clipper_offset_ex(storage, expolygon_view, scale_d(1.)), std::runtime_error);
    CHECK_THROWS_AS(slic3r_api::clipper_offset2_ex(storage, expolygon_view, scale_d(1.), -scale_d(1.)), std::runtime_error);
    CHECK_THROWS_AS(slic3r_api::clipper_offset2_ex(storage, expolygon_view, scale_d(1.), 0.), std::invalid_argument);
    CHECK_THROWS_AS(slic3r_api::clipper_union_ex(nullptr, polygon_view, CLIPPER_FILL_NONZERO), std::invalid_argument);
    CHECK(plugin_storage.size() == count_before);
}
TEST_CASE("Public LineDistancer index matches exhaustive queries and owns its data", "[plugins][line-distancer]")
{
    slic3r_api::LineDistancer reference;
    std::mt19937 random(42);
    std::uniform_int_distribution<int> coordinate(-10000,10000);
    for (size_t i=0;i<1200;++i) {
        const c_point a{coordinate(random),coordinate(random)};
        const c_point b=i%7==0?a:c_point{a.x+coordinate(random)/30,a.y+coordinate(random)/30};
        reference.append_segment(a,b);
    }
    slic3r_api::LineDistancer indexed=reference;
    indexed.build_index();
    for (int i=0;i<120;++i) {
        const c_point p{coordinate(random),coordinate(random)};
        CHECK(indexed.distance_from_lines(p)==Approx(reference.distance_from_lines(p)));
        for (double radius:{-1.,0.,1.,500.,50000.})
            CHECK(indexed.all_lines_in_radius(p,radius)==reference.all_lines_in_radius(p,radius));
        const c_point min{p.x-200,p.y-300},max{p.x+200,p.y+300};
        CHECK(indexed.all_lines_in_box(min,max)==reference.all_lines_in_box(min,max));
        CHECK(indexed.all_lines_in_box(max,min).empty());
    }
    // Invalidation must not expose the stale tree or change the original copy.
    const double original=indexed.distance_from_lines({50000,50000});
    slic3r_api::LineDistancer copy=indexed;
    copy.append_segment({50000,50000},{50000,50000});
    CHECK(copy.distance_from_lines({50000,50000})==0.);
    CHECK(indexed.distance_from_lines({50000,50000})==original);
    copy.build_index();
    slic3r_api::LineDistancer moved=std::move(copy);
    copy=slic3r_api::LineDistancer();
    CHECK(moved.distance_from_lines({50000,50000})==0.);
    CHECK(moved.all_lines_in_radius({50000,50000},0.)==std::vector<size_t>{1200});
    std::vector<std::future<double>> workers;
    for (int i=0;i<8;++i)
        workers.push_back(std::async(std::launch::async,[&indexed] {
            double sum=0;
            for (int j=0;j<100;++j) sum+=indexed.distance_from_lines({j*47,j*31});
            return sum;
        }));
    const double expected=workers.front().get();
    for (size_t i=1;i<workers.size();++i) CHECK(workers[i].get()==expected);

    slic3r_api::LineDistancer empty;
    empty.build_index();
    CHECK(std::isinf(empty.distance_from_lines({0,0},true)));
    CHECK(empty.all_lines_in_box({-1,-1},{1,1}).empty());
    empty.append_segment({0,0},{10,0});
    empty.build_index();
    CHECK(empty.all_lines_in_box({10,0},{10,0})==std::vector<size_t>{0});
    CHECK(empty.distance_from_lines({20,0})==10.); // finite segment endpoint
    CHECK(empty.distance_from_lines({5,0},true)==0.);
}

TEST_CASE("Public LineDistancer signed queries preserve area union and holes", "[plugins][line-distancer]")
{
    PluginStorage storage;
    slic3r_api::StoredExPolygonCollection areas(reinterpret_cast<storage_handle *>(&storage));
    const ExPolygon first=rectangle_with_two_holes();
    const ExPolygon second(rectangle_mm(9.,9.,15.,15.));
    areas.push_back(view(first)); areas.push_back(view(second));
    slic3r_api::LineDistancer indexed(areas.readonly());
    // An added segment does not add an interior; it also forces exhaustive queries.
    slic3r_api::LineDistancer exhaustive=indexed;
    exhaustive.append_segment({scale_i(100),scale_i(100)},{scale_i(100),scale_i(100)});
    for (double x:{-1.,0.,1.,2.,3.,5.,8.,9.,10.,11.,15.,20.,21.}) {
        for (double y:{-1.,0.,1.,3.,5.,9.,10.,11.,15.,20.,21.}) {
            const c_point p{scale_i(x),scale_i(y)};
            CHECK(indexed.distance_from_lines(p,true)==Approx(exhaustive.distance_from_lines(p,true)));
            const double distance=indexed.distance_from_lines(p,true);
            if (distance!=0.)
                CHECK((distance<0.)==(first.contains(Point(p.x,p.y)) || second.contains(Point(p.x,p.y))));
        }
    }
    CHECK(indexed.distance_from_lines({scale_i(3),scale_i(3)},true)>0.);
    CHECK(indexed.distance_from_lines({scale_i(10),scale_i(10)},true)<0.); // second area fills a hole
    CHECK(indexed.distance_from_lines({scale_i(1),scale_i(1)},true)<0.);
    // Closing the stored owners cannot invalidate the copied boundaries.
    areas.clear();
    CHECK(indexed.distance_from_lines({scale_i(1),scale_i(1)},true)<0.);
}

// Opt-in measurement, never a timing assertion. Include index preparation and
// identical query workloads; the checksum prevents dead-code elimination.
TEST_CASE("Public LineDistancer dense boundary benchmark", "[.][plugins][line-distancer-benchmark]")
{
    for (int count:{1000,10000,40000}) {
        slic3r_api::LineDistancer reference;
        for (int i=0;i<count;++i) {
            const coord_t x=(i%200)*1000,y=(i/200)*1000;
            reference.append_segment({x,y},{x+100,y+100});
        }
        slic3r_api::LineDistancer indexed=reference;
        const std::chrono::steady_clock::time_point start=std::chrono::steady_clock::now();
        indexed.build_index();
        const std::chrono::steady_clock::time_point built=std::chrono::steady_clock::now();
        double fast=0,slow=0;
        for (int i=0;i<1000;++i) fast+=indexed.distance_from_lines({(i%200)*1000+50,(i/200)*1000+200});
        const std::chrono::steady_clock::time_point queried=std::chrono::steady_clock::now();
        for (int i=0;i<1000;++i) slow+=reference.distance_from_lines({(i%200)*1000+50,(i/200)*1000+200});
        const std::chrono::steady_clock::time_point done=std::chrono::steady_clock::now();
        CHECK(fast==Approx(slow));
        std::cout << "LineDistancer edges=" << count
                  << " build_ms=" << std::chrono::duration<double,std::milli>(built-start).count()
                  << " indexed_ms=" << std::chrono::duration<double,std::milli>(queried-built).count()
                  << " exhaustive_ms=" << std::chrono::duration<double,std::milli>(done-queried).count() << '\n';
    }
}
