#include <catch2/catch.hpp>

#include <algorithm>
#include <sstream>
#include <utility>
#include <vector>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Plugins/Common/ClipperHelpers.hpp"

#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ClipperUtils.hpp"

/*
Common polygon operation checks
===============================

These tests compare plugin geometry operations in two ways. The core is the
existing Slic3r implementation. Direct API-view offsets and shrinks are compared
to operations on copied work rings using sorted vertex signatures: vertices
within each ring and then ring signatures are sorted, ignoring vertex and ring
order without checking edge connectivity or orientation.

The adapters returning geometry in core order are instead compared directly to
core results, ring by ring, including orientation, starting vertices, holes and
collection order. Variable-offset checks supply contour deltas before hole
deltas; expansion includes clockwise rings. Open-path subtraction compares every
path and vertex in batch order with a nonempty mask. Empty masks are excluded
because the host copies the subject paths without the core Clipper operation,
which may reorder those paths.
API collections borrow stored geometry; work polygons own their point arrays.
Conversion checks compare every coordinate in stored ring and point order, then
verify that output survives explicit freeing and destruction of the storage.

Tests
|-- API view versus work rings: offset and shrink, compared by sorted signatures
|   |-- make_boundary / view: provide the same geometry to both operation paths
|   `-- signatures: sort vertices within each ring, then sort the ring signatures
|-- boolean operations, offsets and simplify: compare ordered rings to core
|-- expand, contains, to_polygons and get_extents: compare expansion rings, containment,
|   flattened rings and bounds to core results
|-- ordered conversion and ownership
|   |-- to_polygons: copy contours and holes from live API views
|   |-- check_exact_view: ensure the source has not changed
|   `-- check_exact_polygons: compare owned output before and after freeing
`-- diff_pl: compare ordered paths for crossing, contained and outside inputs
*/

namespace rect_work = slic3r_api::PluginCommon::polygon_work;
namespace rect_clipper = slic3r_api::PluginCommon::clipper;

TEST_CASE("Plugin common open path subtraction preserves exact native paths", "[plugin-common]")
{
    Slic3r::PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    Slic3r::ExPolygon mask(Slic3r::Polygon({{0, 0}, {1000000, 0}, {1000000, 1000000}, {0, 1000000}}));
    mask.holes.emplace_back(Slic3r::Points{{200000, 200000}, {200000, 800000}, {800000, 800000}, {800000, 200000}});
    const rect_work::ExPolygon work_mask = rect_clipper::from_expolygon_view(
        slic3r_api::ExPolygon(reinterpret_cast<const expolygon_handle *>(&mask)));
    const Slic3r::Polylines paths{
        Slic3r::Polyline{{-100000, 500000}, {1100000, 500000}},
        Slic3r::Polyline{{500000, 1100000}, {500000, -100000}},
        Slic3r::Polyline{{-100000, 0}, {1000000, 0}, {1100000, 100000}},
        Slic3r::Polyline{{-100000, -100000}, {1100000, 1100000}, {-100000, 1100000}},
        Slic3r::Polyline{{10000, 10000}, {100000, 10000}},
        Slic3r::Polyline{{-100000, -100000}, {-200000, -300000}}};
    // Crossings, contained paths and outside paths share the same nonempty clipping mask.
    for (bool empty_subject : {false, true}) {
        {
            INFO("empty subject " << empty_subject);
            const Slic3r::Polylines native_subject = empty_subject ? Slic3r::Polylines{} : paths;
            const Slic3r::ExPolygons native_clip{mask};
            rect_work::Polylines work_subject;
            for (const Slic3r::Polyline &path : native_subject) {
                rect_work::Polyline work_path;
                for (const Slic3r::Point &point : path.points)
                    work_path.points.emplace_back(point.x(), point.y());
                work_subject.push_back(std::move(work_path));
            }
            const rect_work::Polylines actual = rect_clipper::diff_pl(storage, work_subject,
                rect_work::ExPolygons{work_mask});
            const Slic3r::Polylines expected = Slic3r::diff_pl(native_subject, native_clip);
            REQUIRE(actual.size() == expected.size());
            for (size_t i = 0; i < actual.size(); ++i) {
                INFO("path " << i);
                std::ostringstream actual_points, expected_points;
                for (const rect_work::Point &point : actual[i].points)
                    actual_points << '(' << point.x() << ',' << point.y() << ") ";
                for (const Slic3r::Point &point : expected[i].points)
                    expected_points << '(' << point.x() << ',' << point.y() << ") ";
                INFO("actual " << actual_points.str());
                INFO("expected " << expected_points.str());
                REQUIRE(actual[i].size() == expected[i].size());
                for (size_t j = 0; j < actual[i].size(); ++j) {
                    CHECK(actual[i].points[j].x() == expected[i].points[j].x());
                    CHECK(actual[i].points[j].y() == expected[i].points[j].y());
                }
            }
        }
    }
}

TEST_CASE("Plugin common native geometry adapters preserve exact rings", "[plugin-common]")
{
    Slic3r::PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    Slic3r::ExPolygon boundary(Slic3r::Polygon({{-1000000, -1000000}, {2000000, -1000000},
        {2000000, 2000000}, {500000, 2100000}, {-1000000, 2000000}}));
    boundary.holes.emplace_back(Slic3r::Points{{-500000, -500000}, {-500000, 500000}, {500000, 500000}, {500000, -500000}});
    Slic3r::ExPolygon mask(Slic3r::Polygon({{0, -2000000}, {3000000, -2000000}, {3000000, 3000000}, {0, 3000000}}));
    const rect_work::ExPolygon work_boundary = rect_clipper::from_expolygon_view(
        slic3r_api::ExPolygon(reinterpret_cast<const expolygon_handle *>(&boundary)));
    const rect_work::ExPolygon work_mask = rect_clipper::from_expolygon_view(
        slic3r_api::ExPolygon(reinterpret_cast<const expolygon_handle *>(&mask)));
    // Exact comparisons retain contour starts and every hole in native order.
    const auto compare = [](const rect_work::ExPolygons &actual, const Slic3r::ExPolygons &expected, const char *operation = "base") {
        INFO(operation);
        REQUIRE(actual.size() == expected.size());
        for (size_t i = 0; i < actual.size(); ++i) {
            REQUIRE(actual[i].num_contours() == expected[i].num_contours());
            for (size_t j = 0; j < actual[i].num_contours(); ++j) {
                const rect_work::Polygon &a = actual[i].contour_or_hole(j);
                const Slic3r::Polygon &b = expected[i].contour_or_hole(j);
                REQUIRE(a.size() == b.size());
                for (size_t p = 0; p < a.size(); ++p) {
                    CHECK(a[p].x() == b[p].x()); CHECK(a[p].y() == b[p].y());
                }
            }
        }
    };
    const rect_work::ExPolygons first{work_boundary}, second{work_mask};
    const Slic3r::ExPolygons native_first{boundary}, native_second{mask};
    compare(rect_clipper::diff_ex(storage, first, second), Slic3r::diff_ex(native_first, native_second), "difference collections");
    compare(rect_clipper::diff_ex(storage, work_boundary, work_mask), Slic3r::diff_ex(boundary, mask), "difference single polygons");
    compare(rect_clipper::intersection_ex(storage, first, second), Slic3r::intersection_ex(native_first, native_second), "intersection");
    // The contour and hole have different vertex counts and different inward distances.
    const std::vector<std::vector<float>> deltas{{-10000.f, -20000.f, -30000.f, -40000.f, -50000.f},
        {-15000.f, -25000.f, -35000.f, -45000.f}};
    compare(rect_clipper::variable_offset_inner_ex(storage, work_boundary, deltas),
        Slic3r::variable_offset_inner_ex(boundary, deltas), "variable inward offset");
    const rect_work::ExPolygons empty;
    const Slic3r::ExPolygons native_empty;
    compare(rect_clipper::diff_ex(storage, empty, second), Slic3r::diff_ex(native_empty, native_second), "empty subject");
    compare(rect_clipper::diff_ex(storage, first, empty), Slic3r::diff_ex(native_first, native_empty), "empty clip");
    compare(rect_clipper::intersection_ex(storage, empty, second), Slic3r::intersection_ex(native_empty, native_second), "empty intersection subject");
    compare(rect_clipper::intersection_ex(storage, first, empty), Slic3r::intersection_ex(native_first, native_empty), "empty intersection clip");
    compare(rect_clipper::union_ex(storage, first), Slic3r::union_ex(native_first));
    compare(rect_clipper::union_ex(storage, first, second), Slic3r::union_ex(native_first, native_second));
    // Translated masks exercise disjoint, touching, overlapping and fully covering rings.
    for (coord_t shift : {-4000000, -2000000, -1000000, -500000, 0, 500000, 2000000, 4000000}) {
        INFO("mask shift " << shift);
        Slic3r::ExPolygon moved = mask;
        moved.translate(Slic3r::Point(shift, shift / 3));
        const rect_work::ExPolygon work_moved = rect_clipper::from_expolygon_view(
            slic3r_api::ExPolygon(reinterpret_cast<const expolygon_handle *>(&moved)));
        compare(rect_clipper::union_ex(storage, first, {work_moved}), Slic3r::union_ex(native_first, Slic3r::ExPolygons{moved}), "two-input union");
    }
    // An island inside a hole must follow its enclosing contour in collection order.
    Slic3r::ExPolygon island(Slic3r::Polygon({{-100000, -100000}, {100000, -100000}, {100000, 100000}, {-100000, 100000}}));
    const rect_work::ExPolygon work_island = rect_clipper::from_expolygon_view(
        slic3r_api::ExPolygon(reinterpret_cast<const expolygon_handle *>(&island)));
    compare(rect_clipper::union_ex(storage, {work_boundary, work_island}), Slic3r::union_ex(Slic3r::ExPolygons{boundary, island}));
    const rect_work::ExPolygons islands{work_boundary, work_island};
    const Slic3r::ExPolygons native_islands{boundary, island};
    compare(rect_clipper::diff_ex(storage, islands, second), Slic3r::diff_ex(native_islands, native_second), "multiple islands difference");
    compare(rect_clipper::intersection_ex(storage, islands, second), Slic3r::intersection_ex(native_islands, native_second), "multiple islands intersection");
    const std::vector<std::vector<float>> disappearing_deltas(1, std::vector<float>(4, -200000.f));
    const Slic3r::ExPolygons removed = Slic3r::variable_offset_inner_ex(island, disappearing_deltas);
    REQUIRE(removed.empty());
    compare(rect_clipper::variable_offset_inner_ex(storage, work_island, disappearing_deltas), removed, "small ring removed");
    // Compare expansion point by point without sorting or rotating clockwise rings.
    const auto compare_expansion = [storage](const rect_work::Polygons &source, const Slic3r::Polygons &native_source) {
        const rect_work::Polygons actual = rect_clipper::expand(storage, source, 10000.);
        const Slic3r::Polygons expected = Slic3r::expand(native_source, 10000.);
        REQUIRE(actual.size() == expected.size());
        for (size_t i = 0; i < actual.size(); ++i) {
            REQUIRE(actual[i].size() == expected[i].size());
            for (size_t j = 0; j < actual[i].size(); ++j) {
                CHECK(actual[i][j].x() == expected[i][j].x());
                CHECK(actual[i][j].y() == expected[i][j].y());
            }
        }
    };
    compare_expansion(rect_work::to_polygons(islands), Slic3r::to_polygons(native_islands));
    REQUIRE(boundary.holes.front().is_clockwise());
    compare_expansion({work_boundary.holes.front()}, {boundary.holes.front()});
    compare_expansion({}, {});
    for (double delta : {-10000., 0., 10000.}) {
        compare(rect_clipper::offset_ex(storage, first, delta), Slic3r::offset_ex(native_first, delta));
        compare(rect_clipper::offset_ex(storage, work_boundary, delta), Slic3r::offset_ex(boundary, delta));
    }
    for (coord_t tolerance : {100, 10000, 200000, 1000000, 10000000})
        compare(rect_clipper::simplify(storage, work_boundary, tolerance), boundary.simplify(tolerance));
    // Small repeated CCW loops dominate the input area but disappear, leaving a CW lobe.
    Slic3r::ExPolygon hourglass;
    hourglass.contour.points.emplace_back(-100, 0);
    for (size_t i = 0; i < 120; ++i) {
        hourglass.contour.points.emplace_back(0, -100);
        hourglass.contour.points.emplace_back(100, 0);
        hourglass.contour.points.emplace_back(0, 100);
        hourglass.contour.points.emplace_back(-100, 0);
    }
    hourglass.contour.points.emplace_back(0, 0);
    hourglass.contour.points.emplace_back(1000, 1000);
    hourglass.contour.points.emplace_back(2000, 0);
    hourglass.contour.points.emplace_back(1000, -1000);
    hourglass.contour.points.emplace_back(0, 0);
    REQUIRE(hourglass.contour.is_counter_clockwise());
    Slic3r::Polygon simplified_contour = hourglass.contour;
    simplified_contour.douglas_peucker(500);
    REQUIRE(simplified_contour.size() >= 3);
    REQUIRE(simplified_contour.is_clockwise());
    const rect_work::ExPolygon work_hourglass = rect_clipper::from_expolygon_view(
        slic3r_api::ExPolygon(reinterpret_cast<const expolygon_handle *>(&hourglass)));
    REQUIRE(work_hourglass.contour.is_counter_clockwise());
    rect_work::Polygon simplified_work_contour = work_hourglass.contour;
    simplified_work_contour.douglas_peucker(500);
    REQUIRE(simplified_work_contour.size() >= 3);
    REQUIRE(simplified_work_contour.is_clockwise());
    compare(rect_clipper::simplify(storage, work_hourglass, 500), hourglass.simplify(500), "orientation repair");
    // A zero-area bow tie remains self-intersecting after its collinear vertex is removed.
    Slic3r::ExPolygon bow_tie(Slic3r::Polygon({{0, 0}, {1000, 1000}, {2000, 2000}, {0, 2000}, {2000, 0}}));
    const rect_work::ExPolygon work_bow_tie = rect_clipper::from_expolygon_view(
        slic3r_api::ExPolygon(reinterpret_cast<const expolygon_handle *>(&bow_tie)));
    Slic3r::Polygon native_bow_tie = bow_tie.contour;
    native_bow_tie.douglas_peucker(100);
    rect_work::Polygon simplified_bow_tie = work_bow_tie.contour;
    simplified_bow_tie.douglas_peucker(100);
    REQUIRE(native_bow_tie.size() == 4);
    REQUIRE(simplified_bow_tie.size() == 4);
    CHECK(rect_clipper::orientation_area(simplified_bow_tie) == Slic3r::ClipperLib::Area(native_bow_tie.points));
    REQUIRE(rect_clipper::orientation_area(simplified_bow_tie) == 0.);
    REQUIRE(native_bow_tie.is_counter_clockwise());
    REQUIRE_FALSE(simplified_bow_tie.is_counter_clockwise());
    compare(rect_clipper::simplify(storage, work_bow_tie, 100), bow_tie.simplify(100), "zero-area bow tie");
    for (coord_t x : {-1000001, -1000000, -500000, 0, 500000, 2000000, 2000001}) {
        for (bool border : {false, true})
            CHECK(work_boundary.contains({x, 0}, border) == boundary.contains(Slic3r::Point(x, 0), border));
        const rect_work::Polyline line({x, 0}, {1000000, 0});
        const Slic3r::Polyline native_line(Slic3r::Point(x, 0), Slic3r::Point(1000000, 0));
        CHECK(rect_clipper::contains(storage, work_boundary, line) == boundary.contains(native_line));
    }
    const rect_work::Polygons flat = rect_work::to_polygons(first);
    const Slic3r::Polygons native_flat = Slic3r::to_polygons(native_first);
    REQUIRE(flat.size() == native_flat.size());
    for (size_t i = 0; i < flat.size(); ++i) {
        REQUIRE(flat[i].size() == native_flat[i].size());
        for (size_t j = 0; j < flat[i].size(); ++j) {
            CHECK(flat[i][j].x() == native_flat[i][j].x()); CHECK(flat[i][j].y() == native_flat[i][j].y());
        }
    }
    const rect_work::BoundingBox extent = rect_clipper::get_extents(first);
    const Slic3r::BoundingBox native_extent = Slic3r::get_extents(native_first);
    CHECK(extent.min.x() == native_extent.min.x()); CHECK(extent.min.y() == native_extent.min.y());
    CHECK(extent.max.x() == native_extent.max.x()); CHECK(extent.max.y() == native_extent.max.y());

}

namespace {

// Compare owned work points with core points in stored order; neither input changes.
void check_exact_polygons(const rect_work::Polygons &actual, const Slic3r::Polygons &expected);
// Compare a live borrowed ring with every core coordinate without conversion.
void check_exact_view(const slic3r_api::Polygon &actual, const Slic3r::Polygon &expected);

slic3r_api::ExPolygon view(const Slic3r::ExPolygon &expolygon)
{
    return slic3r_api::ExPolygon(reinterpret_cast<const expolygon_handle *>(&expolygon));
}

Slic3r::ExPolygon make_boundary()
{
    Slic3r::ExPolygon out(Slic3r::Polygon({ Slic3r::Point(-10000000, -8000000), Slic3r::Point(12000000, -8000000),
                                             Slic3r::Point(12000000, 9000000), Slic3r::Point(-10000000, 9000000) }));
    out.holes.push_back(Slic3r::Polygon({ Slic3r::Point(-3000000, -2000000), Slic3r::Point(-3000000, 3000000),
                                          Slic3r::Point(4000000, 3000000), Slic3r::Point(4000000, -2000000) }));
    return out;
}

rect_work::Polygons as_polygons(const rect_work::ExPolygon &expolygon)
{
    rect_work::Polygons out;
    out.push_back(expolygon.contour);
    for (const rect_work::Polygon &hole : expolygon.holes)
        out.push_back(hole);
    return out;
}

using PolygonSignature = std::vector<std::pair<coord_t, coord_t>>;

std::vector<PolygonSignature> signatures(const rect_work::Polygons &polygons)
{
    std::vector<PolygonSignature> out;
    out.reserve(polygons.size());
    for (const rect_work::Polygon &polygon : polygons) {
        PolygonSignature signature;
        signature.reserve(polygon.points.size());
        for (const rect_work::Point &point : polygon.points)
            signature.emplace_back(point.x(), point.y());
        std::sort(signature.begin(), signature.end());
        out.push_back(std::move(signature));
    }
    std::sort(out.begin(), out.end());
    return out;
}

void check_exact_polygons(const rect_work::Polygons &actual, const Slic3r::Polygons &expected)
{
    REQUIRE(actual.size() == expected.size());
    for (size_t polygon_idx = 0; polygon_idx < expected.size(); ++polygon_idx) {
        REQUIRE(actual[polygon_idx].points.size() == expected[polygon_idx].points.size());
        for (size_t point_idx = 0; point_idx < expected[polygon_idx].points.size(); ++point_idx) {
            CHECK(actual[polygon_idx].points[point_idx].x() == expected[polygon_idx].points[point_idx].x());
            CHECK(actual[polygon_idx].points[point_idx].y() == expected[polygon_idx].points[point_idx].y());
        }
    }
}

void check_exact_view(const slic3r_api::Polygon &actual, const Slic3r::Polygon &expected)
{
    REQUIRE(actual.size() == expected.points.size());
    for (uint32_t idx = 0; idx < actual.size(); ++idx) {
        const c_point point = actual.at(idx);
        CHECK(point.x == expected.points[idx].x());
        CHECK(point.y == expected.points[idx].y());
    }
}

} // namespace

TEST_CASE("Rectilinear clipper offsets an API ExPolygon without source materialization",
          "[plugins][infill][rectilinear][plugin-common]")
{
    Slic3r::ExPolygon core_boundary = make_boundary();
    const slic3r_api::ExPolygon api_boundary = view(core_boundary);

    Slic3r::PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    const rect_work::ExPolygon value_boundary = rect_clipper::from_expolygon_view(api_boundary);
    const rect_work::Polygons value_source = as_polygons(value_boundary);

    const rect_work::Polygons direct_offset = rect_clipper::offset(storage, api_boundary, 250000.);
    const rect_work::Polygons value_offset = rect_clipper::offset(storage, value_source, 250000.);
    REQUIRE(signatures(direct_offset) == signatures(value_offset));

    const rect_work::Polygons direct_shrink = rect_clipper::shrink(storage, api_boundary, 250000.);
    const rect_work::Polygons value_shrink = rect_clipper::shrink(storage, value_source, 250000.);
    CHECK(signatures(direct_shrink) == signatures(value_shrink));
}

// Convert a live empty stored collection, then free it; owned empty output must survive storage destruction or Catch fails.
TEST_CASE("Rectilinear polygon conversion owns the result of an empty stored collection",
          "[plugins][infill][rectilinear][rectilinear-hole-view]")
{
    rect_work::Polygons output;
    {
        Slic3r::PluginStorage plugin_storage;
        storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
        slic3r_api::StoredExPolygonCollection stored(storage);
        REQUIRE(stored.size() == 0);
        output = rect_clipper::to_polygons(stored.readonly());
        REQUIRE(stored.size() == 0);
        CHECK(output.empty());
        REQUIRE(stored.free_from_storage());
    }
    CHECK(output.empty());
}

// Copy explicit core rings into live storage and convert them; exact order checks detect changes and output owns points after freeing storage.
TEST_CASE("Rectilinear polygon conversion preserves ordered contours and holes after freeing the source",
          "[plugins][infill][rectilinear][rectilinear-hole-view]")
{
    const Slic3r::ExPolygons originals = {
        Slic3r::ExPolygon(Slic3r::Polygon({{90, 80}, {10, 80}, {10, 10}, {90, 10}}),
                         Slic3r::Polygons{Slic3r::Polygon({{35, 25}, {25, 20}, {20, 35}}),
                                           Slic3r::Polygon({{75, 60}, {75, 45}, {60, 45}, {55, 55}, {60, 65}})}),
        Slic3r::ExPolygon(Slic3r::Polygon({{190, 70}, {150, 95}, {110, 70}, {120, 15}, {180, 15}})),
        Slic3r::ExPolygon(Slic3r::Polygon({{290, 75}, {260, 95}, {220, 85}, {210, 45}, {230, 10}, {285, 20}}),
                         Slic3r::Polygons{Slic3r::Polygon({{270, 65}, {275, 40}, {240, 35}, {235, 60}})})
    };
    Slic3r::Polygons expected;
    for (const Slic3r::ExPolygon &original : originals) {
        expected.push_back(original.contour);
        for (const Slic3r::Polygon &hole : original.holes)
            expected.push_back(hole);
    }
    rect_work::Polygons output;
    {
        Slic3r::PluginStorage plugin_storage;
        storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
        slic3r_api::StoredExPolygonCollection stored(storage);
        for (const Slic3r::ExPolygon &original : originals) {
            const slic3r_api::ExPolygon source = view(original);
            stored.push_back(source);
        }
        output = rect_clipper::to_polygons(stored.readonly());
        REQUIRE(stored.size() == originals.size());
        for (uint32_t idx = 0; idx < stored.size(); ++idx) {
            const slic3r_api::ExPolygon source = stored.at(idx);
            check_exact_view(source.contour(), originals[idx].contour);
            REQUIRE(source.hole_size() == originals[idx].holes.size());
            size_t hole_idx = 0;
            for (const slic3r_api::Polygon hole : source.holes())
                check_exact_view(hole, originals[idx].holes[hole_idx++]);
            REQUIRE(hole_idx == originals[idx].holes.size());
        }
        check_exact_polygons(output, expected);
        REQUIRE(stored.free_from_storage());
        check_exact_polygons(output, expected);
    }
    check_exact_polygons(output, expected);
}
