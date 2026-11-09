#include <catch2/catch.hpp>

#include <random>
#include <utility>
#include <vector>

#include "libslic3r/Plugins/Common/PolygonWorkBuffers.hpp"

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Geometry/Circle.hpp"
#include "libslic3r/MultiPoint.hpp"
#include "libslic3r/PointUtils.hpp"
#include "libslic3r/Polyline.hpp"

/*
Common mutable geometry checks
==============================

The core is the existing Slic3r implementation.
These tests construct matching core and plugin buffers and compare coordinates,
rounding, tolerances, clipping results and validation. Borrowed contour checks
also edit a source point to verify that the view reads the original array.

Tests
|-- ContourView: read API and work arrays without changing their representation
|-- path helpers: compare simplification, validation and grid alignment to core
|-- geometry helpers: compare clipping, circle hits, orientation and box bounds
`-- integer and ring helpers: compare intersections, projections, areas and box queries

Integer and ring checks compare exact coordinates without ring rotation or
floating-point tolerance. Area checks compare core accumulation on translated
rings and signed holes, including an exact tie between two translated contours.
*/

namespace rect_work = slic3r_api::PluginCommon::polygon_work;

// Translated rings retain the core's exact area tie, including signed hole accumulation.
TEST_CASE("Plugin common core area matches translated rings and holes", "[plugin-common]")
{
    using namespace Slic3r;
    const Points points{{206617184, 18488186}, {181663101, 47362403}, {144756491, 79301278},
        {122374428, 35951167}, {110638435, -4626551}, {146453966, -34887781}, {192838478, -24399510}};
    std::vector<double> areas;
    for (coord_t shift : {coord_t(0), coord_t(110000000)}) {
        Polygon native;
        rect_work::Polygon owned;
        for (const Point &point : points) {
            native.points.emplace_back(point.x() + shift, point.y());
            owned.points.emplace_back(point.x() + shift, point.y());
        }
        CHECK(rect_work::core_area(owned) == native.area());
        areas.push_back(rect_work::core_area(owned));
        ExPolygon native_ex;
        native_ex.contour = native;
        native_ex.holes.push_back(Polygon{{Point(140000000 + shift, 10000000),
            Point(140000000 + shift, 11000000), Point(141000000 + shift, 10000000)}});
        rect_work::ExPolygon owned_ex;
        owned_ex.contour = owned;
        owned_ex.holes.resize(1);
        for (const Point &point : native_ex.holes.front().points)
            owned_ex.holes.front().points.emplace_back(point.x(), point.y());
        CHECK(rect_work::core_area(owned_ex) == native_ex.area());
    }
    CHECK(areas[0] == areas[1]);
}

TEST_CASE("Plugin common integer segments match core rounding and intersections", "[plugin-common]")
{
    std::mt19937 rng(0xace123);
    for (size_t trial = 0; trial < 500; ++trial) {
        INFO(trial);
        rect_work::Point points[4];
        Slic3r::Point native[4];
        for (size_t i = 0; i < 4; ++i) {
            const coord_t x = coord_t(rng() % 10001) - 5000;
            const coord_t y = coord_t(rng() % 10001) - 5000;
            points[i] = rect_work::Point(x, y);
            native[i] = Slic3r::Point(x, y);
        }
        rect_work::Line line(points[0], points[1]);
        Slic3r::Line core(native[0], native[1]);
        rect_work::Point result;
        Slic3r::Point expected;
        CHECK(line.length() == core.length());
        CHECK(line.midpoint().x() == core.midpoint().x());
        CHECK(line.midpoint().y() == core.midpoint().y());
        CHECK(line.distance_to_squared(points[2], &result) == core.distance_to_squared(native[2], &expected));
        CHECK(result.x() == expected.x());
        CHECK(result.y() == expected.y());
        CHECK(rect_work::line_alg::distance_to_squared(line, points[2], &result) ==
              Slic3r::line_alg::distance_to_squared(core, native[2], &expected));
        CHECK(result.x() == expected.x());
        CHECK(result.y() == expected.y());
        const bool hit = core.intersection(Slic3r::Line(native[2], native[3]), &expected);
        REQUIRE(line.intersection(rect_work::Line(points[2], points[3]), &result) == hit);
        if (hit) { CHECK(result.x() == expected.x()); CHECK(result.y() == expected.y()); }
        if (points[0] != points[1] && points[2] != points[3])
            CHECK(rect_work::segments_intersect(points[0], points[1], points[2], points[3]) ==
                  Slic3r::Geometry::segments_intersect(native[0], native[1], native[2], native[3]));
        core.extend_end(113.75);
        line.extend_end(113.75);
        CHECK(line.b.x() == core.b.x());
        CHECK(line.b.y() == core.b.y());
    }
    for (coord_t x : {-20, -10, 0, 5, 10, 20}) {
        CHECK(rect_work::segments_intersect({-10, 0}, {10, 0}, {x, 0}, {x + 5, 0}) ==
              Slic3r::Geometry::segments_intersect({-10, 0}, {10, 0}, {x, 0}, {x + 5, 0}));
    }
}

TEST_CASE("Plugin common ring and box queries match core exactly", "[plugin-common]")
{
    for (bool reverse : {false, true}) {
        Slic3r::Polygon core({{-1000, -1000}, {3000, -500}, {2000, 3000}, {-2000, 1000}});
        if (reverse) core.reverse();
        rect_work::Polygon ring;
        for (const Slic3r::Point &point : core.points) ring.points.emplace_back(point.x(), point.y());
        const Slic3r::BoundingBox box(core.points);
        const rect_work::BoundingBox work_box(ring.points);
        for (coord_t x : {-3000, -2000, -1000, 0, 1000, 3000, 5000}) {
            for (coord_t y : {-3000, -1000, 0, 1000, 3000, 5000}) {
                const Slic3r::Point point(x, y);
                const rect_work::Point query(x, y);
                const Slic3r::Point *nearest = core.closest_point(point);
                const rect_work::Point *actual = ring.closest_point(query);
                CHECK(actual->x() == nearest->x()); CHECK(actual->y() == nearest->y());
                const std::pair<Slic3r::Point, size_t> projection = core.point_projection(point);
                const std::pair<rect_work::Point, size_t> projected = ring.point_projection(query);
                CHECK(projected.first.x() == projection.first.x()); CHECK(projected.first.y() == projection.first.y());
                CHECK(projected.second == projection.second);
                for (bool border : {false, true})
                    CHECK(rect_work::contains(ring, query, border) == Slic3r::contains(core, point, border));
                const Slic3r::Line travel(point, {4000, 50});
                const rect_work::Line work_travel(query, {4000, 50});
                Slic3r::Point intersection;
                rect_work::Point result;
                const bool hit = core.first_intersection(travel, &intersection);
                REQUIRE(ring.first_intersection(work_travel, &result) == hit);
                if (hit) { CHECK(result.x() == intersection.x()); CHECK(result.y() == intersection.y()); }
                CHECK(rect_work::cross(work_box, work_travel) == box.cross(travel));
                const Slic3r::Polyline path(point, Slic3r::Point(4000, 50));
                const rect_work::Polyline work_path(query, rect_work::Point(4000, 50));
                CHECK(rect_work::cross(work_box, work_path) == box.cross(path));
                const Slic3r::Point edge = box.nearest_point(point);
                const rect_work::Point work_edge = rect_work::nearest_point(work_box, query);
                CHECK(work_edge.x() == edge.x()); CHECK(work_edge.y() == edge.y());
            }
        }
        CHECK(rect_work::contains(work_box, work_box) == box.contains(box));
        core.douglas_peucker(300);
        ring.douglas_peucker(300);
        REQUIRE(ring.size() == core.size());
        for (size_t i = 0; i < ring.size(); ++i) {
            CHECK(ring[i].x() == core[i].x()); CHECK(ring[i].y() == core[i].y());
        }
    }
}

TEST_CASE("Plugin common projection distinguishes core rounding from truncation", "[plugin-common]")
{
    for (coord_t sign : {-1, 1}) {
        const rect_work::Line line({0, 0}, {sign * 3, sign * 2});
        const Slic3r::Line native({0, 0}, {sign * 3, sign * 2});
        const rect_work::Point point(0, sign * 2);
        const Slic3r::Point native_point(0, sign * 2);
        rect_work::Point rounded, truncated;
        Slic3r::Point native_rounded, native_truncated;
        CHECK(line.distance_to_squared(point, &rounded) == native.distance_to_squared(native_point, &native_rounded));
        CHECK(rect_work::line_alg::distance_to_squared(line, point, &truncated) ==
              Slic3r::line_alg::distance_to_squared(native, native_point, &native_truncated));
        CHECK(rounded.x() == native_rounded.x()); CHECK(rounded.y() == native_rounded.y());
        CHECK(truncated.x() == native_truncated.x()); CHECK(truncated.y() == native_truncated.y());
        CHECK(rounded != truncated);
    }
}

namespace {

Slic3r::Polyline make_core_polyline(std::mt19937 &rng, size_t count, rect_work::Polyline &ported)
{
    Slic3r::Polyline core;
    for (size_t idx = 0; idx < count; ++idx) {
        const coord_t x = coord_t(int(rng() % 40000) - 20000);
        const coord_t y = coord_t(int(rng() % 40000) - 20000);
        core.points.emplace_back(x, y);
        ported.points.push_back(rect_work::Point(x, y));
    }
    return core;
}

} // namespace

TEST_CASE("Rectilinear contour views borrow API and work point arrays",
          "[plugins][infill][rectilinear][plugin-common]")
{
    const c_point api_points[] = { { 1, 2 }, { 3, 4 }, { 5, 6 } };
    const multipoint_const_view api_view{ api_points, 3, 0 };
    rect_work::ContourView api_contour(api_view);
    CHECK(api_contour.size() == 3);
    CHECK(api_contour.front().x == 1);
    CHECK(api_contour.back().y == 6);

    rect_work::Points work_points{ rect_work::Point(10, 20), rect_work::Point(30, 40), rect_work::Point(50, 60) };
    const rect_work::ContourView work_contour(work_points);
    work_points[1].x() = 31;
    CHECK(work_contour[1].x == 31);
    CHECK(work_contour.with_size(2).back().y == 40);
}

TEST_CASE("Rectilinear work buffer douglas_peucker matches the core implementation",
          "[plugins][infill][rectilinear][plugin-common]")
{
    std::mt19937 rng(0x5eed1234u);
    for (int trial = 0; trial < 200; ++trial) {
        INFO("trial " << trial);
        rect_work::Polyline ported;
        Slic3r::Polyline core = make_core_polyline(rng, 3 + rng() % 40, ported);
        const coord_t tolerance = coord_t(1 + rng() % 200);

        core.douglas_peucker(tolerance);
        ported.douglas_peucker(tolerance);

        REQUIRE(core.points.size() == ported.points.size());
        for (size_t idx = 0; idx < core.points.size(); ++idx) {
            CHECK(core.points[idx].x() == ported.points[idx].x());
            CHECK(core.points[idx].y() == ported.points[idx].y());
        }
    }
}

TEST_CASE("Rectilinear work buffer ensure_valid matches the core implementation",
          "[plugins][infill][rectilinear][plugin-common]")
{
    std::mt19937 rng(0xc0ffee);
    for (int trial = 0; trial < 100; ++trial) {
        INFO("trial " << trial);
        Slic3r::Polylines core;
        rect_work::Polylines ported;
        const size_t polyline_count = 1 + rng() % 5;
        for (size_t poly = 0; poly < polyline_count; ++poly) {
            rect_work::Polyline ported_line;
            Slic3r::Polyline core_line = make_core_polyline(rng, 2 + rng() % 20, ported_line);
            if (rng() % 2 == 0 && core_line.points.size() > 1) {
                core_line.points.back() = core_line.points.front();
                ported_line.points.back() = ported_line.points.front();
            }
            core.push_back(std::move(core_line));
            ported.push_back(std::move(ported_line));
        }
        const coord_t resolution = coord_t(1 + rng() % 100);

        Slic3r::ensure_valid(core, resolution);
        rect_work::ensure_valid(ported, resolution);

        REQUIRE(core.size() == ported.size());
        for (size_t poly = 0; poly < core.size(); ++poly) {
            REQUIRE(core[poly].points.size() == ported[poly].points.size());
            for (size_t idx = 0; idx < core[poly].points.size(); ++idx) {
                CHECK(core[poly].points[idx].x() == ported[poly].points[idx].x());
                CHECK(core[poly].points[idx].y() == ported[poly].points[idx].y());
            }
        }
    }
}

TEST_CASE("Rectilinear work buffer align_to_grid matches the core implementation",
          "[plugins][infill][rectilinear][plugin-common]")
{
    std::mt19937 rng(0xabcdefu);
    for (int trial = 0; trial < 500; ++trial) {
        INFO("trial " << trial);
        const coord_t coord = coord_t(int(rng() % 20000) - 10000);
        const coord_t spacing = coord_t(1 + rng() % 1000);
        const coord_t base = coord_t(int(rng() % 2000) - 1000);

        CHECK(Slic3r::align_to_grid(coord, spacing) == rect_work::align_to_grid(coord, spacing));
        CHECK(Slic3r::align_to_grid(coord, spacing, base) == rect_work::align_to_grid(coord, spacing, base));

        const Slic3r::Point core_point = Slic3r::align_to_grid(Slic3r::Point(coord, base), Slic3r::Point(spacing, spacing));
        const rect_work::Point work_point = rect_work::align_to_grid(rect_work::Point(coord, base), rect_work::Point(spacing, spacing));
        CHECK(core_point.x() == work_point.x());
        CHECK(core_point.y() == work_point.y());

        const Slic3r::Point core_point_base =
            Slic3r::align_to_grid(Slic3r::Point(coord, base), Slic3r::Point(spacing, spacing), Slic3r::Point(base, coord));
        const rect_work::Point work_point_base =
            rect_work::align_to_grid(rect_work::Point(coord, base), rect_work::Point(spacing, spacing), rect_work::Point(base, coord));
        CHECK(core_point_base.x() == work_point_base.x());
        CHECK(core_point_base.y() == work_point_base.y());
    }
}

TEST_CASE("Rectilinear work buffer liang_barsky clipping matches the core implementation",
          "[plugins][infill][rectilinear][plugin-common]")
{
    std::mt19937 rng(0x1234567u);
    for (int trial = 0; trial < 500; ++trial) {
        INFO("trial " << trial);
        const coord_t x0 = coord_t(int(rng() % 20000) - 10000);
        const coord_t y0 = coord_t(int(rng() % 20000) - 10000);
        const coord_t x1 = coord_t(int(rng() % 20000) - 10000);
        const coord_t y1 = coord_t(int(rng() % 20000) - 10000);

        Slic3r::BoundingBoxf core_box(Slic3r::Vec2d(-5000., -3000.), Slic3r::Vec2d(6000., 4000.));
        rect_work::BoundingBoxf work_box(rect_work::Vec2d(-5000., -3000.), rect_work::Vec2d(6000., 4000.));

        Slic3r::Vec2d core_a{double(x0), double(y0)};
        Slic3r::Vec2d core_b{double(x1), double(y1)};
        const bool core_clipped = Slic3r::Geometry::liang_barsky_line_clipping(core_a, core_b, core_box);

        rect_work::Vec2d work_a{double(x0), double(y0)};
        rect_work::Vec2d work_b{double(x1), double(y1)};
        const bool work_clipped = rect_work::liang_barsky_line_clipping(work_a, work_b, work_box);

        REQUIRE(core_clipped == work_clipped);
        if (core_clipped) {
            CHECK(core_a.x() == work_a.x());
            CHECK(core_a.y() == work_a.y());
            CHECK(core_b.x() == work_b.x());
            CHECK(core_b.y() == work_b.y());
        }
    }
}

TEST_CASE("Rectilinear work buffer circle intersections match the core implementation",
          "[plugins][infill][rectilinear][plugin-common]")
{
    std::mt19937 rng(0x77aa33u);
    for (int trial = 0; trial < 500; ++trial) {
        INFO("trial " << trial);
        const double radius = 1. + double(rng() % 1000);
        const double a = double(int(rng() % 200) - 100);
        const double b = double(int(rng() % 200) - 100);
        const double lv2 = a * a + b * b;
        if (lv2 == 0.)
            continue;
        const double c = double(int(rng() % 2000) - 1000);

        std::pair<Slic3r::Vec2d, Slic3r::Vec2d> core_out;
        const int core_count = Slic3r::Geometry::ray_circle_intersections_r2_lv2_c(radius * radius, a, b, lv2, c, core_out);

        std::pair<rect_work::Vec2d, rect_work::Vec2d> work_out;
        const int work_count = rect_work::ray_circle_intersections_r2_lv2_c(radius * radius, a, b, lv2, c, work_out);

        REQUIRE(core_count == work_count);
        if (core_count >= 1) {
            CHECK(core_out.first.x() == work_out.first.x());
            CHECK(core_out.first.y() == work_out.first.y());
        }
        if (core_count == 2) {
            CHECK(core_out.second.x() == work_out.second.x());
            CHECK(core_out.second.y() == work_out.second.y());
        }
    }
}

TEST_CASE("Rectilinear work buffer orientation and extents match the core implementation",
          "[plugins][infill][rectilinear][plugin-common]")
{
    std::mt19937 rng(0x9911u);
    for (int trial = 0; trial < 200; ++trial) {
        INFO("trial " << trial);
        rect_work::Polyline work_line;
        Slic3r::Polyline core_line = make_core_polyline(rng, 3 + rng() % 20, work_line);

        Slic3r::Polygon core_polygon;
        core_polygon.points = core_line.points;
        rect_work::Polygon work_polygon;
        work_polygon.points = work_line.points;

        CHECK(Slic3r::Geometry::is_ccw(core_polygon) == rect_work::is_ccw(work_polygon));

        const Slic3r::BoundingBox core_box = Slic3r::get_extents(core_line.points);
        const rect_work::BoundingBox work_box = rect_work::get_extents(work_line.points);
        CHECK(core_box.min.x() == work_box.min.x());
        CHECK(core_box.min.y() == work_box.min.y());
        CHECK(core_box.max.x() == work_box.max.x());
        CHECK(core_box.max.y() == work_box.max.y());
        CHECK(core_box.center().x() == work_box.center().x());
        CHECK(core_box.center().y() == work_box.center().y());
    }
}

TEST_CASE("Rectilinear work buffer bounding box defined semantics match the core",
          "[plugins][infill][rectilinear][plugin-common]")
{
    const rect_work::BoundingBox work_x_degenerate(rect_work::Point(0, 0), rect_work::Point(0, 10));
    const rect_work::BoundingBox work_y_degenerate(rect_work::Point(0, 0), rect_work::Point(10, 0));
    const Slic3r::BoundingBox core_x_degenerate(Slic3r::Point(0, 0), Slic3r::Point(0, 10));
    const Slic3r::BoundingBox core_y_degenerate(Slic3r::Point(0, 0), Slic3r::Point(10, 0));

    CHECK(work_x_degenerate.defined == core_x_degenerate.defined);
    CHECK(work_y_degenerate.defined == core_y_degenerate.defined);
    CHECK_FALSE(work_x_degenerate.defined);
    CHECK_FALSE(work_y_degenerate.defined);
}
