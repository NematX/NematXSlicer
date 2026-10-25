/* Collection clean-ups and the polyline offset of the plugin API must give the
   exact native results, because plugins replace the native
   calls with these functions and their output is compared with the legacy output.

   Each test builds native Polygons or Polylines, passes them to the C API
   through reinterpret_cast handles (in the host process a handle is the native
   object), and compares point for point with the native function applied to a
   copy. The C++ views are checked on one case each, through storage-owned
   collections. */
#include <catch2/catch.hpp>

#include <vector>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/c/slic3r_clipper.h"
#include "libslic3r/Api/plugin/c/slic3r_geometry.h"
#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/MutablePolygon.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Polyline.hpp"

namespace {
using namespace Slic3r;

// Scaled point from millimeters.
Point point_mm(double x, double y);
// Mutable C handle over a native polygon collection.
polygon_collection_handle *handle(Polygons &polygons);
// Mutable C handle over a native polyline collection.
polyline_collection_handle *handle(Polylines &polylines);
// Square with a narrow V notch from the top edge plus a small clockwise triangle, inputs that smooth_outward changes.
Polygons notched_square_and_tiny_hole();

Point point_mm(double x, double y)
{
    return Point(scale_i(x), scale_i(y));
}

polygon_collection_handle *handle(Polygons &polygons)
{
    return reinterpret_cast<polygon_collection_handle *>(&polygons);
}

polyline_collection_handle *handle(Polylines &polylines)
{
    return reinterpret_cast<polyline_collection_handle *>(&polylines);
}

Polygons notched_square_and_tiny_hole()
{
    Polygon notched({point_mm(0., 0.), point_mm(10., 0.), point_mm(10., 10.), point_mm(5.2, 10.),
                     point_mm(5., 5.), point_mm(4.8, 10.), point_mm(0., 10.)});
    Polygon tiny_hole({point_mm(2., 2.), point_mm(2.1, 2.3), point_mm(2.2, 2.)});
    REQUIRE(notched.is_counter_clockwise());
    REQUIRE(tiny_hole.is_clockwise());
    return {notched, tiny_hole};
}
} // namespace

TEST_CASE("Polygon collection ensure_valid matches the native one, holes of a removed contour included",
          "[plugins][geometry-cleanup]")
{
    // A contour reduced below 3 points by the simplification, followed by its hole.
    Polygon flat_contour({point_mm(0., 0.), point_mm(5., 0.001), point_mm(10., 0.)});
    flat_contour.make_counter_clockwise();
    Polygon hole({point_mm(2., -1.), point_mm(2., -2.), point_mm(3., -2.), point_mm(3., -1.)});
    hole.make_clockwise();
    Polygon square({point_mm(20., 0.), point_mm(30., 0.), point_mm(30.002, 5.), point_mm(30., 10.), point_mm(20., 10.)});
    const Polygons input{square, flat_contour, hole, Polygon()};
    const coord_t resolution = scale_i(0.0125);

    Polygons expected = input;
    ensure_valid(expected, resolution);
    Polygons actual = input;
    polygons_ensure_valid(handle(actual), resolution);
    CHECK(actual == expected);
    // The native quirk keeps the hole of the removed contour.
    CHECK(actual.size() == 2);

    // The stored C++ view gives the same result.
    PluginStorage storage;
    slic3r_api::StoredPolygonCollection stored(reinterpret_cast<storage_handle *>(&storage));
    for (const Polygon &polygon : input)
        stored.push_back(slic3r_api::Polygon(reinterpret_cast<const polygon_handle *>(&polygon)));
    stored.ensure_valid(resolution);
    CHECK(*reinterpret_cast<const Polygons *>(stored.handle()) == expected);
    polygons_ensure_valid(nullptr, resolution);
}

TEST_CASE("Polygon collection remove_sticks and remove_degenerate match the native ones", "[plugins][geometry-cleanup]")
{
    // A square with a spike going out and straight back, a two-point polygon and a clean triangle.
    Polygon spiked({point_mm(0., 0.), point_mm(10., 0.), point_mm(10., 5.), point_mm(15., 5.), point_mm(10., 5.),
                    point_mm(10., 10.), point_mm(0., 10.)});
    Polygon two_points({point_mm(0., 0.), point_mm(1., 1.)});
    Polygon triangle({point_mm(20., 0.), point_mm(25., 0.), point_mm(20., 5.)});
    const Polygons input{spiked, two_points, triangle};

    Polygons expected_sticks = input;
    const bool native_sticks = remove_sticks(expected_sticks);
    Polygons actual_sticks = input;
    CHECK(polygons_remove_sticks(handle(actual_sticks)) == (native_sticks ? 1 : 0));
    CHECK(actual_sticks == expected_sticks);
    CHECK(actual_sticks.size() == 2);

    Polygons expected_degenerate = input;
    const bool native_degenerate = remove_degenerate(expected_degenerate);
    Polygons actual_degenerate = input;
    CHECK(polygons_remove_degenerate(handle(actual_degenerate)) == (native_degenerate ? 1 : 0));
    CHECK(actual_degenerate == expected_degenerate);
    CHECK(polygons_remove_degenerate(handle(actual_degenerate)) == 0);

    CHECK(polygons_remove_sticks(nullptr) == 0);
    CHECK(polygons_remove_degenerate(nullptr) == 0);
}

TEST_CASE("Polygon collection smooth_outward matches the native one", "[plugins][geometry-cleanup]")
{
    const Polygons input = notched_square_and_tiny_hole();
    for (const double clip_mm : {0.5, 1.5}) {
        INFO("clip distance " << clip_mm);
        const coord_t clip = scale_i(clip_mm);
        const Polygons expected = smooth_outward(input, clip);
        Polygons actual = input;
        polygons_smooth_outward(handle(actual), clip);
        CHECK(actual == expected);
        // The notch is changed and the tiny clockwise hole is filled.
        CHECK(actual.size() == 1);
        CHECK(actual.front() != input.front());
    }

    PluginStorage storage;
    slic3r_api::StoredPolygonCollection stored(reinterpret_cast<storage_handle *>(&storage));
    for (const Polygon &polygon : input)
        stored.push_back(slic3r_api::Polygon(reinterpret_cast<const polygon_handle *>(&polygon)));
    stored.smooth_outward(scale_i(1.5));
    CHECK(*reinterpret_cast<const Polygons *>(stored.handle()) == smooth_outward(input, scale_i(1.5)));
    polygons_smooth_outward(nullptr, scale_i(1.));
}

TEST_CASE("Polyline collection ensure_valid and remove_degenerate match the native ones", "[plugins][geometry-cleanup]")
{
    // A line with a tiny bump removed by the simplification, a line collapsing to nearly one point, and a single point.
    const Polyline bumpy({point_mm(0., 0.), point_mm(5., 0.001), point_mm(10., 0.)});
    const Polyline collapsing({point_mm(20., 0.), point_mm(20.000001, 0.), point_mm(20.000002, 0.)});
    const Polyline single({point_mm(30., 0.)});
    const Polyline straight({point_mm(0., 5.), point_mm(10., 5.)});
    const coord_t resolution = scale_i(0.0125);

    // The native ensure_valid() asserts on a single point, so it gets the input without it.
    Polylines expected{bumpy, collapsing, straight};
    ensure_valid(expected, resolution);
    Polylines actual{bumpy, collapsing, single, straight};
    polylines_ensure_valid(handle(actual), resolution);
    CHECK(actual == expected);
    CHECK(actual.size() == 2);

    Polylines expected_degenerate{bumpy, single, straight};
    const bool native_degenerate = remove_degenerate(expected_degenerate);
    Polylines actual_degenerate{bumpy, single, straight};
    CHECK(polylines_remove_degenerate(handle(actual_degenerate)) == (native_degenerate ? 1 : 0));
    CHECK(actual_degenerate == expected_degenerate);

    polylines_ensure_valid(nullptr, resolution);
    CHECK(polylines_remove_degenerate(nullptr) == 0);
}

TEST_CASE("Polyline collection offset matches the native offset of Polylines", "[plugins][geometry-cleanup]")
{
    // Two crossing lines and a separate bent line, so the union and the joins both matter.
    const Polylines polylines{
        Polyline({point_mm(0., 0.), point_mm(10., 10.)}),
        Polyline({point_mm(0., 10.), point_mm(10., 0.)}),
        Polyline({point_mm(20., 0.), point_mm(25., 5.), point_mm(30., 0.), point_mm(35., 4.)}),
    };
    const polyline_collection_handle *input = reinterpret_cast<const polyline_collection_handle *>(&polylines);
    PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    const double delta = scale_d(0.4);

    struct Case { clipper_join_type_t join; double miter; clipper_end_type_t end; ClipperLib::JoinType native_join; ClipperLib::EndType native_end; };
    for (const Case &test : {Case{CLIPPER_JOIN_SQUARE, 0., CLIPPER_END_OPEN_BUTT, ClipperLib::jtSquare, ClipperLib::etOpenButt},
                             Case{CLIPPER_JOIN_ROUND, scale_d(0.01), CLIPPER_END_OPEN_ROUND, ClipperLib::jtRound, ClipperLib::etOpenRound},
                             Case{CLIPPER_JOIN_MITER, 3., CLIPPER_END_OPEN_SQUARE, ClipperLib::jtMiter, ClipperLib::etOpenSquare}}) {
        INFO("join " << int(test.join) << " end " << int(test.end));
        const Polygons expected = offset(polylines, delta, test.native_join, test.miter, test.native_end);
        clipper_shapes_handle *shapes = clipper_offset_polylines(storage, input, delta, test.join, test.miter, test.end);
        REQUIRE(shapes != nullptr);
        const polygon_collection_handle *actual = clipper_shapes_to_polygons(storage, shapes);
        REQUIRE(actual != nullptr);
        CHECK(*reinterpret_cast<const Polygons *>(actual) == expected);
        CHECK_FALSE(expected.empty());
    }

    // The C++ view defaults are the native line defaults.
    const slic3r_api::PolylineCollection view(input);
    const slic3r_api::StoredPolygonCollection from_view =
        slic3r_api::clipper_offset_polylines(storage, view, delta).to_polygon_collection();
    CHECK(*reinterpret_cast<const Polygons *>(from_view.handle()) == offset(polylines, delta));

    CHECK(clipper_offset_polylines(nullptr, input, delta, CLIPPER_JOIN_SQUARE, 0., CLIPPER_END_OPEN_BUTT) == nullptr);
    CHECK(clipper_offset_polylines(storage, nullptr, delta, CLIPPER_JOIN_SQUARE, 0., CLIPPER_END_OPEN_BUTT) == nullptr);
    CHECK(clipper_offset_polylines(storage, input, 0., CLIPPER_JOIN_SQUARE, 0., CLIPPER_END_OPEN_BUTT) == nullptr);
}
