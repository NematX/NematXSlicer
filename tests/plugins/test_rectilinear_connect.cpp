#include <catch2/catch.hpp>

#include <chrono>
#include <cmath>
#include <iostream>
#include <utility>
#include <vector>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Plugins/Infill/Rectilinear/RectilinearConnect.hpp"

#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Fill/FillBase.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Polyline.hpp"

/*
Rectilinear connect port
========================

The connect helpers (PrusaSimpleConnect and FakePerimeterConnect) are a
verbatim port of src/libslic3r/Fill/FillBase.cpp. These tests run the core and
the ported implementation on the same boundary and infill lines and require
bit-identical output, for both connect strategies.
*/

namespace rect_work = slic3r_api::Infill::RectilinearPatternsPlugin::work;
namespace rect_connect = slic3r_api::Infill::RectilinearPatternsPlugin::connect;

namespace {

Slic3r::ExPolygon make_core_boundary()
{
    Slic3r::ExPolygon out(Slic3r::Polygon({ Slic3r::Point(-10000000, -10000000), Slic3r::Point(10000000, -10000000),
                                            Slic3r::Point(10000000, 10000000), Slic3r::Point(-10000000, 10000000) }));
    out.holes.push_back(Slic3r::Polygon({ Slic3r::Point(-3000000, -3000000), Slic3r::Point(-3000000, 3000000),
                                          Slic3r::Point(3000000, 3000000), Slic3r::Point(3000000, -3000000) }));
    return out;
}

Slic3r::ExPolygon make_dirty_core_boundary()
{
    Slic3r::ExPolygon out(Slic3r::Polygon({ Slic3r::Point(0, 0), Slic3r::Point(10000000, 0),
                                             Slic3r::Point(10000000, 0), Slic3r::Point(10000000, 10000000),
                                             Slic3r::Point(0, 10000000) }));
    out.holes.push_back(Slic3r::Polygon({ Slic3r::Point(2000000, 2000000), Slic3r::Point(2000000, 8000000),
                                          Slic3r::Point(8000000, 8000000), Slic3r::Point(8000000, 2000000) }));
    return out;
}

Slic3r::ExPolygon make_headstick_core_boundary()
{
    // The closing edge retraces the first edge, so the only stick sits around
    // the first vertex. remove_sticks() drops (0, 0) and keeps a 3 point
    // triangle.
    Slic3r::ExPolygon out(Slic3r::Polygon({ Slic3r::Point(0, 0), Slic3r::Point(2000000, 0),
                                            Slic3r::Point(2000000, 2000000), Slic3r::Point(1000000, 0) }));
    out.holes.push_back(Slic3r::Polygon({ Slic3r::Point(1400000, 500000), Slic3r::Point(1400000, 900000),
                                          Slic3r::Point(1900000, 900000), Slic3r::Point(1900000, 500000) }));
    return out;
}

Slic3r::ExPolygon make_headstick_hole_boundary()
{
    // Symmetric case: the stuck contour is a hole, the outer contour is clean.
    Slic3r::ExPolygon out(Slic3r::Polygon({ Slic3r::Point(-10000000, -10000000), Slic3r::Point(10000000, -10000000),
                                            Slic3r::Point(10000000, 10000000), Slic3r::Point(-10000000, 10000000) }));
    out.holes.push_back(Slic3r::Polygon({ Slic3r::Point(0, 0), Slic3r::Point(3000000, 0),
                                          Slic3r::Point(6000000, 6000000), Slic3r::Point(6000000, 0) }));
    return out;
}

Slic3r::ExPolygon make_double_dirty_core_boundary()
{
    // Both contours need cleanup: the outer has an interior spike and the hole
    // has a head-only stick, all points distinct so the source stays assert
    // valid. The hole must be cleaned even though the outer contour is,
    // remove_sticks(ExPolygon&) used to short-circuit on it.
    Slic3r::ExPolygon out(Slic3r::Polygon({ Slic3r::Point(0, 0), Slic3r::Point(20000000, 0),
                                            Slic3r::Point(10000000, 0), Slic3r::Point(10000000, 10000000),
                                            Slic3r::Point(0, 10000000) }));
    out.holes.push_back(Slic3r::Polygon({ Slic3r::Point(2000000, 2000000), Slic3r::Point(5000000, 2000000),
                                          Slic3r::Point(8000000, 8000000), Slic3r::Point(8000000, 2000000) }));
    return out;
}

Slic3r::ExPolygon make_dense_core_boundary(size_t point_count)
{
    Slic3r::Polygon outer;
    outer.points.reserve(point_count);
    for (size_t idx = 0; idx < point_count; ++idx) {
        const double angle = 2. * 3.14159265358979323846 * double(idx) / double(point_count);
        outer.points.emplace_back(coord_t(std::round(20000000. * std::cos(angle))),
                                  coord_t(std::round(20000000. * std::sin(angle))));
    }
    return Slic3r::ExPolygon(std::move(outer));
}

void make_infill_lines(Slic3r::Polylines &core, rect_work::Polylines &ported)
{
    for (coord_t x = -8000000; x <= 8000000; x += 2000000) {
        const coord_t y_min = -9000000;
        const coord_t y_max = 9000000;
        if (x > -3000000 && x < 3000000) {
            core.emplace_back(Slic3r::Polyline({ Slic3r::Point(x, y_min), Slic3r::Point(x, -3000000) }));
            core.emplace_back(Slic3r::Polyline({ Slic3r::Point(x, 3000000), Slic3r::Point(x, y_max) }));
            rect_work::Polyline lower;
            lower.points = { rect_work::Point(x, y_min), rect_work::Point(x, -3000000) };
            rect_work::Polyline upper;
            upper.points = { rect_work::Point(x, 3000000), rect_work::Point(x, y_max) };
            ported.push_back(std::move(lower));
            ported.push_back(std::move(upper));
        } else {
            core.emplace_back(Slic3r::Polyline({ Slic3r::Point(x, y_min), Slic3r::Point(x, y_max) }));
            rect_work::Polyline line;
            line.points = { rect_work::Point(x, y_min), rect_work::Point(x, y_max) };
            ported.push_back(std::move(line));
        }
    }
}

template <typename ExpectedPolylines, typename ActualPolylines>
void require_same_infill(const ExpectedPolylines &expected, const ActualPolylines &actual)
{
    REQUIRE(expected.size() == actual.size());
    for (size_t poly = 0; poly < expected.size(); ++poly) {
        INFO("polyline " << poly);
        REQUIRE(expected[poly].points.size() == actual[poly].points.size());
        for (size_t idx = 0; idx < expected[poly].points.size(); ++idx) {
            CHECK(expected[poly].points[idx].x() == actual[poly].points[idx].x());
            CHECK(expected[poly].points[idx].y() == actual[poly].points[idx].y());
        }
    }
}

void make_support_base_infill_lines(Slic3r::Polylines &core, rect_work::Polylines &ported)
{
    for (coord_t x = -8000000; x <= 8000000; x += 2000000) {
        const coord_t y_min = -10000000;
        const coord_t y_max = 10000000;
        if (x > -3000000 && x < 3000000) {
            core.emplace_back(Slic3r::Polyline({ Slic3r::Point(x, y_min), Slic3r::Point(x, -3000000) }));
            core.emplace_back(Slic3r::Polyline({ Slic3r::Point(x, 3000000), Slic3r::Point(x, y_max) }));
            rect_work::Polyline lower;
            lower.points = { rect_work::Point(x, y_min), rect_work::Point(x, -3000000) };
            rect_work::Polyline upper;
            upper.points = { rect_work::Point(x, 3000000), rect_work::Point(x, y_max) };
            ported.push_back(std::move(lower));
            ported.push_back(std::move(upper));
        } else {
            core.emplace_back(Slic3r::Polyline({ Slic3r::Point(x, y_min), Slic3r::Point(x, y_max) }));
            rect_work::Polyline line;
            line.points = { rect_work::Point(x, y_min), rect_work::Point(x, y_max) };
            ported.push_back(std::move(line));
        }
    }
}

// Outputs of one support-base connect: the native generator and the port, on the same lines and boundary.
struct SupportBaseConnect
{
    Slic3r::Polylines core;
    rect_work::Polylines ported;
};

// Connect the support-base lines once through the native generator and once through the port, with the line spacing of
// this call; both start from fresh copies of the same lines.
SupportBaseConnect connect_support_base(storage_handle *storage, const Slic3r::Polygons &core_boundary,
                                        const Slic3r::BoundingBox &core_bbox, const rect_work::Polygons &ported_boundary,
                                        const rect_work::BoundingBox &ported_bbox, coord_t line_spacing, float density)
{
    Slic3r::Polylines core_infill;
    rect_work::Polylines ported_infill;
    make_support_base_infill_lines(core_infill, ported_infill);

    Slic3r::FillParams core_params;
    core_params.density = density;
    core_params.anchor_length = 1000.f;
    core_params.anchor_length_max = 1000.f;
    raw_infill_pattern_params ported_params = {};
    ported_params.density = density;
    ported_params.anchor_length = 1000.f;
    ported_params.anchor_length_max = 1000.f;

    SupportBaseConnect result;
    Slic3r::Fill::connect_base_support(std::move(core_infill), core_boundary, core_bbox, result.core, line_spacing, core_params);
    rect_connect::connect_base_support(storage, std::move(ported_infill), ported_boundary, ported_bbox, result.ported,
                                       line_spacing, ported_params);
    return result;
}

} // namespace

TEST_CASE("Rectilinear API boundary views selectively materialize cleanup",
          "[plugins][infill][rectilinear]")
{
    Slic3r::ExPolygon clean_boundary = make_core_boundary();
    const slic3r_api::ExPolygon clean_view(
        reinterpret_cast<const expolygon_handle *>(&clean_boundary));
    const rect_connect::BoundaryViewStorage clean_storage(clean_view);
    REQUIRE(clean_storage.materialized_contour_count() == 0);
    REQUIRE(clean_storage.materialized_point_count() == 0);
    const coord_t clean_original_x = clean_boundary.contour.points.front().x();
    clean_boundary.contour.points.front().x() += 123;
    CHECK(clean_storage.contours().front().front().x == clean_original_x + 123);

    Slic3r::ExPolygon dirty_boundary = make_dirty_core_boundary();
    const slic3r_api::ExPolygon dirty_view(
        reinterpret_cast<const expolygon_handle *>(&dirty_boundary));
    const rect_connect::BoundaryViewStorage dirty_storage(dirty_view);
    const rect_work::ExPolygon normalized_dirty = rect_connect::normalized_boundary(dirty_view);
    REQUIRE(dirty_storage.materialized_contour_count() == 1);
    REQUIRE(dirty_storage.contours().size() == dirty_boundary.holes.size() + 1);
    REQUIRE(dirty_storage.contours().front().size() == dirty_boundary.contour.points.size() - 1);
    REQUIRE(normalized_dirty.contour.points.size() == dirty_storage.contours().front().size());
    for (size_t idx = 0; idx < normalized_dirty.contour.points.size(); ++idx) {
        CHECK(dirty_storage.contours().front()[idx].x == normalized_dirty.contour.points[idx].x());
        CHECK(dirty_storage.contours().front()[idx].y == normalized_dirty.contour.points[idx].y());
    }
    const coord_t dirty_original_x = dirty_boundary.contour.points.front().x();
    dirty_boundary.contour.points.front().x() += 123;
    CHECK(dirty_storage.contours().front().front().x == dirty_original_x);
    CHECK(dirty_boundary.contour.points.size() == 5);
    dirty_boundary.holes.front().points.front().x() += 321;
    CHECK(dirty_storage.contours().back().front().x == dirty_boundary.holes.front().points.front().x());
}

TEST_CASE("Rectilinear API boundary views keep head-stick removals",
          "[plugins][infill][rectilinear]")
{
    // remove_sticks() drops head-only sticks and reports them, so the cleaned
    // result is kept. Both boundaries below only have such a stick, and
    // normalized_boundary() keeps the cleaned contour unconditionally: the two
    // must agree.
    {
        Slic3r::ExPolygon headstick_boundary = make_headstick_core_boundary();
        const slic3r_api::ExPolygon headstick_view(
            reinterpret_cast<const expolygon_handle *>(&headstick_boundary));
        const rect_connect::BoundaryViewStorage storage(headstick_view);
        const rect_work::ExPolygon normalized = rect_connect::normalized_boundary(headstick_view);

        REQUIRE(storage.materialized_contour_count() == 1);
        REQUIRE(storage.contours().front().size() == 3);
        CHECK(storage.contours().front()[0].x == 2000000);
        CHECK(storage.contours().front()[0].y == 0);
        REQUIRE(normalized.contour.points.size() == storage.contours().front().size());
        for (size_t idx = 0; idx < normalized.contour.points.size(); ++idx) {
            CHECK(storage.contours().front()[idx].x == normalized.contour.points[idx].x());
            CHECK(storage.contours().front()[idx].y == normalized.contour.points[idx].y());
        }
    }
    {
        Slic3r::ExPolygon headstick_boundary = make_headstick_hole_boundary();
        const slic3r_api::ExPolygon headstick_view(
            reinterpret_cast<const expolygon_handle *>(&headstick_boundary));
        const rect_connect::BoundaryViewStorage storage(headstick_view);
        const rect_work::ExPolygon normalized = rect_connect::normalized_boundary(headstick_view);

        REQUIRE(storage.materialized_contour_count() == 1);
        REQUIRE(storage.contours().size() == 2);
        REQUIRE(storage.contours().back().size() == 3);
        CHECK(storage.contours().back()[0].x == 3000000);
        CHECK(storage.contours().back()[0].y == 0);
        REQUIRE(normalized.holes.size() == 1);
        REQUIRE(normalized.holes.front().points.size() == storage.contours().back().size());
        for (size_t idx = 0; idx < normalized.holes.front().points.size(); ++idx) {
            CHECK(storage.contours().back()[idx].x == normalized.holes.front().points[idx].x());
            CHECK(storage.contours().back()[idx].y == normalized.holes.front().points[idx].y());
        }
    }
}

TEST_CASE("Rectilinear boundary cleanup drops sticks on the contour and the holes",
          "[plugins][infill][rectilinear]")
{
    // A head-only stick removal is part of the reported "modified" state.
    Slic3r::Polygon headstick(Slic3r::Polygon({ Slic3r::Point(0, 0), Slic3r::Point(2000000, 0),
                                                Slic3r::Point(2000000, 2000000), Slic3r::Point(1000000, 0) }));
    CHECK(Slic3r::remove_sticks(headstick));
    CHECK(headstick.points.size() == 3);

    Slic3r::ExPolygon dirty_boundary = make_double_dirty_core_boundary();
    const slic3r_api::ExPolygon dirty_view(
        reinterpret_cast<const expolygon_handle *>(&dirty_boundary));

    // Core: the two cleanups are independent, so the hole is cleaned even
    // though the outer contour was.
    Slic3r::ExPolygon core_normalized = dirty_boundary;
    REQUIRE(Slic3r::remove_sticks(core_normalized));
    REQUIRE(core_normalized.contour.points.size() == 4);
    REQUIRE(core_normalized.holes.size() == 1);
    REQUIRE(core_normalized.holes.front().points.size() == 3);

    // Port: the view storage and the reference normalization agree with it.
    const rect_connect::BoundaryViewStorage storage(dirty_view);
    const rect_work::ExPolygon normalized = rect_connect::normalized_boundary(dirty_view);
    REQUIRE(storage.materialized_contour_count() == 2);
    REQUIRE(storage.contours().front().size() == core_normalized.contour.points.size());
    REQUIRE(storage.contours().back().size() == 3);    REQUIRE(normalized.contour.points.size() == storage.contours().front().size());
    REQUIRE(normalized.holes.size() == 1);
    REQUIRE(normalized.holes.front().points.size() == storage.contours().back().size());
    for (size_t idx = 0; idx < normalized.contour.points.size(); ++idx) {
        CHECK(storage.contours().front()[idx].x == normalized.contour.points[idx].x());
        CHECK(storage.contours().front()[idx].y == normalized.contour.points[idx].y());
        CHECK(storage.contours().front()[idx].x == core_normalized.contour.points[idx].x());
        CHECK(storage.contours().front()[idx].y == core_normalized.contour.points[idx].y());
    }
    for (size_t idx = 0; idx < normalized.holes.front().points.size(); ++idx) {
        CHECK(storage.contours().back()[idx].x == normalized.holes.front().points[idx].x());
        CHECK(storage.contours().back()[idx].y == normalized.holes.front().points[idx].y());
        CHECK(storage.contours().back()[idx].x == core_normalized.holes.front().points[idx].x());
        CHECK(storage.contours().back()[idx].y == core_normalized.holes.front().points[idx].y());
    }
}

TEST_CASE("Rectilinear API boundary view cleanup measurement",
          "[.][plugins][infill][rectilinear][benchmark]")
{
    const size_t point_count = 4096;
    const size_t iterations = 20;
    const Slic3r::ExPolygon core_boundary = make_dense_core_boundary(point_count);
    const slic3r_api::ExPolygon api_boundary(
        reinterpret_cast<const expolygon_handle *>(&core_boundary));
    volatile size_t sink = 0;
    size_t legacy_materialized_points = 0;
    size_t view_materialized_points = 0;

    const std::chrono::steady_clock::time_point legacy_start = std::chrono::steady_clock::now();
    for (size_t iteration = 0; iteration < iterations; ++iteration) {
        const rect_work::ExPolygon normalized = rect_connect::normalized_boundary(api_boundary);
        legacy_materialized_points += normalized.contour.points.size();
        sink += normalized.contour.points.size();
    }
    const std::chrono::steady_clock::time_point legacy_done = std::chrono::steady_clock::now();

    const std::chrono::steady_clock::time_point view_start = std::chrono::steady_clock::now();
    for (size_t iteration = 0; iteration < iterations; ++iteration) {
        const rect_connect::BoundaryViewStorage views(api_boundary);
        view_materialized_points += views.materialized_point_count();
        sink += views.contours().size();
    }
    const std::chrono::steady_clock::time_point view_done = std::chrono::steady_clock::now();

    const double legacy_ms = std::chrono::duration<double, std::milli>(legacy_done - legacy_start).count();
    const double view_ms = std::chrono::duration<double, std::milli>(view_done - view_start).count();
    std::cout << "Rectilinear boundary cleanup benchmark: points=" << point_count
              << " iterations=" << iterations << " legacy_ms=" << legacy_ms
              << " view_ms=" << view_ms
              << " legacy_materialized_points=" << legacy_materialized_points
              << " view_materialized_points=" << view_materialized_points << '\n';

    CHECK(sink > 0);
    CHECK(legacy_materialized_points == point_count * iterations);
    CHECK(view_materialized_points == 0);
}

TEST_CASE("Rectilinear connect clips a two-point polyline", "[plugins][infill][rectilinear]")
{
    const rect_work::Points polyline = {
        rect_work::Point(0, 0),
        rect_work::Point(100, 0)
    };

    const rect_connect::SegmentPoint start =
        rect_connect::clip_start_segment_and_point(polyline, 10.);
    const rect_connect::SegmentPoint end =
        rect_connect::clip_end_segment_and_point(polyline, 10.);

    REQUIRE(start.valid());
    REQUIRE(end.valid());
    CHECK(start.idx_segment == 0);
    CHECK(end.idx_segment == 0);
    CHECK(start.t < end.t);
}

TEST_CASE("Rectilinear connect uses the nearest segment endpoint", "[plugins][infill][rectilinear]")
{
    const rect_work::Vec2d segment_start(0., 0.);
    const rect_work::Vec2d segment_end(10., 0.);
    const rect_work::Vec2d point_after_end(12., 0.);

    CHECK(rect_connect::PrusaSimpleConnect::segment_point_distance_squared(
              segment_start, segment_end, point_after_end) == Approx(4.));
}

TEST_CASE("Rectilinear connect port matches the core PrusaSimpleConnect strategy",
          "[plugins][infill][rectilinear]")
{
    const Slic3r::ExPolygon core_boundary = make_core_boundary();
    const slic3r_api::ExPolygon ported_boundary(
        reinterpret_cast<const expolygon_handle *>(&core_boundary));
    const rect_work::ExPolygon value_boundary = rect_connect::normalized_boundary(ported_boundary);
    Slic3r::PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);

    for (const c_infill_connection connection :
         { C_INFILL_CONNECTION_CONNECTED, C_INFILL_CONNECTION_NOT_CONNECTED,
           C_INFILL_CONNECTION_HOLES, C_INFILL_CONNECTION_OUTER_SHELL }) {
        INFO("connection " << int(connection));
        for (const coord_t spacing : { 400000, 700000, 1200000 }) {
            INFO("spacing " << spacing);

            Slic3r::Polylines core_infill;
            rect_work::Polylines ported_infill;
            make_infill_lines(core_infill, ported_infill);
            rect_work::Polylines value_infill = ported_infill;
            REQUIRE(core_infill.size() == ported_infill.size());

            // anchor_length_max == 0 selects the simple connect strategy.
            Slic3r::FillParams core_params;
            core_params.density = 0.15f;
            core_params.connection = static_cast<Slic3r::InfillConnection>(connection);
            core_params.anchor_length_max = 0.f;
            raw_infill_pattern_params ported_params = {};
            ported_params.density = 0.15f;
            ported_params.connection = connection;
            ported_params.anchor_length_max = 0.f;

            Slic3r::Polylines core_out;
            rect_work::Polylines ported_out;
            rect_work::Polylines value_out;
            Slic3r::Fill::connect_infill(std::move(core_infill), core_boundary, core_out, spacing, core_params);
            slic3r_api::Infill::RectilinearPatternsPlugin::connect::connect_infill(
                storage, std::move(ported_infill), ported_boundary, ported_out, spacing, ported_params);
            rect_connect::connect_infill(
                storage, std::move(value_infill), value_boundary, value_out, spacing, ported_params);
            require_same_infill(core_out, ported_out);
            require_same_infill(ported_out, value_out);
        }
    }
}

TEST_CASE("Rectilinear connect port matches the core FakePerimeterConnect strategy",
          "[plugins][infill][rectilinear]")
{
    const Slic3r::ExPolygon core_boundary = make_core_boundary();
    const slic3r_api::ExPolygon ported_boundary(
        reinterpret_cast<const expolygon_handle *>(&core_boundary));
    const rect_work::ExPolygon value_boundary = rect_connect::normalized_boundary(ported_boundary);
    Slic3r::PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);

    for (const c_infill_connection connection :
         { C_INFILL_CONNECTION_CONNECTED, C_INFILL_CONNECTION_NOT_CONNECTED,
           C_INFILL_CONNECTION_HOLES, C_INFILL_CONNECTION_OUTER_SHELL }) {
        INFO("connection " << int(connection));
        for (const coord_t spacing : { 400000, 700000 }) {
            for (const double anchor_length : { 500000., 2000000. }) {
                INFO("spacing " << spacing << " anchor_length " << anchor_length);

                Slic3r::Polylines core_infill;
                rect_work::Polylines ported_infill;
                make_infill_lines(core_infill, ported_infill);
                rect_work::Polylines value_infill = ported_infill;

                Slic3r::FillParams core_params;
                core_params.density = 0.15f;
                core_params.connection = static_cast<Slic3r::InfillConnection>(connection);
                core_params.anchor_length = float(anchor_length);
                core_params.anchor_length_max = float(anchor_length * 3.);
                raw_infill_pattern_params ported_params = {};
                ported_params.density = 0.15f;
                ported_params.connection = connection;
                ported_params.anchor_length = float(anchor_length);
                ported_params.anchor_length_max = float(anchor_length * 3.);

                Slic3r::Polylines core_out;
                rect_work::Polylines ported_out;
                rect_work::Polylines value_out;
                Slic3r::Fill::connect_infill(std::move(core_infill), core_boundary, core_out, spacing, core_params);
                slic3r_api::Infill::RectilinearPatternsPlugin::connect::connect_infill(
                    storage, std::move(ported_infill), ported_boundary, ported_out, spacing, ported_params);
                rect_connect::connect_infill(
                    storage, std::move(value_infill), value_boundary, value_out, spacing, ported_params);
                require_same_infill(core_out, ported_out);
                require_same_infill(ported_out, value_out);
            }
        }
    }
}

TEST_CASE("Rectilinear support-base connect port matches the core strategy",
          "[plugins][infill][rectilinear][supportbase]")
{
    const Slic3r::ExPolygon core_expolygon = make_core_boundary();
    const Slic3r::Polygons core_boundary = [&core_expolygon]() {
        Slic3r::Polygons boundary;
        boundary.emplace_back(core_expolygon.contour);
        for (const Slic3r::Polygon &hole : core_expolygon.holes)
            boundary.emplace_back(hole);
        return boundary;
    }();

    rect_work::Polygons ported_boundary;
    for (const Slic3r::Polygon &polygon : core_boundary) {
        rect_work::Polygon converted;
        converted.points.reserve(polygon.points.size());
        for (const Slic3r::Point &point : polygon.points)
            converted.points.emplace_back(point.x(), point.y());
        ported_boundary.emplace_back(std::move(converted));
    }

    Slic3r::PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    const Slic3r::BoundingBox core_bbox = Slic3r::get_extents(core_boundary);
    const rect_work::BoundingBox ported_bbox = rect_work::get_extents(ported_boundary.front().points);

    for (const float density : { 0.15f, 0.5f })
        for (const std::pair<float, float> anchors : { std::pair<float, float>(1000.f, 1000.f),
                                                        std::pair<float, float>(2.f, 5.f) }) {
            INFO("density " << density << " anchor_length " << anchors.first
                 << " anchor_length_max " << anchors.second);

            Slic3r::Polylines core_infill;
            rect_work::Polylines ported_infill;
            make_support_base_infill_lines(core_infill, ported_infill);

            Slic3r::FillParams core_params;
            core_params.density = density;
            core_params.anchor_length = anchors.first;
            core_params.anchor_length_max = anchors.second;

            raw_infill_pattern_params ported_params = {};
            ported_params.density = density;
            ported_params.anchor_length = anchors.first;
            ported_params.anchor_length_max = anchors.second;

            Slic3r::Polylines core_out;
            rect_work::Polylines ported_out;
            Slic3r::Fill::connect_base_support(std::move(core_infill), core_boundary, core_bbox,
                                               core_out, scale_i(0.8), core_params);
            rect_connect::connect_base_support(storage, std::move(ported_infill), ported_boundary,
                                               ported_bbox, ported_out, scale_i(0.8), ported_params);
            REQUIRE(!core_out.empty());
            REQUIRE(!ported_out.empty());
            require_same_infill(core_out, ported_out);
        }
}

// The native connect derives its arc-cost thresholds from the spacing of each call, not from an earlier call of the
// process: after a call with another spacing, the first spacing gives its first result again, and every call agrees
// with the port.
TEST_CASE("Native support-base connect takes its thresholds from each call's spacing",
          "[plugins][infill][rectilinear][supportbase]")
{
    const Slic3r::ExPolygon core_expolygon = make_core_boundary();
    const Slic3r::Polygons core_boundary = [&core_expolygon]() {
        Slic3r::Polygons boundary;
        boundary.emplace_back(core_expolygon.contour);
        for (const Slic3r::Polygon &hole : core_expolygon.holes)
            boundary.emplace_back(hole);
        return boundary;
    }();

    rect_work::Polygons ported_boundary;
    for (const Slic3r::Polygon &polygon : core_boundary) {
        rect_work::Polygon converted;
        converted.points.reserve(polygon.points.size());
        for (const Slic3r::Point &point : polygon.points)
            converted.points.emplace_back(point.x(), point.y());
        ported_boundary.emplace_back(std::move(converted));
    }

    Slic3r::PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    const Slic3r::BoundingBox core_bbox = Slic3r::get_extents(core_boundary);
    const rect_work::BoundingBox ported_bbox = rect_work::get_extents(ported_boundary.front().points);

    // Two line spacings of one 0.4 mm fill at densities 0.5 and 0.15, as the fill derives them.
    const coord_t dense_line_spacing = scale_i(0.8);
    const coord_t sparse_line_spacing = scale_i(2.6);
    const SupportBaseConnect first = connect_support_base(storage, core_boundary, core_bbox, ported_boundary, ported_bbox,
                                                          dense_line_spacing, 0.5f);
    const SupportBaseConnect other = connect_support_base(storage, core_boundary, core_bbox, ported_boundary, ported_bbox,
                                                          sparse_line_spacing, 0.15f);
    const SupportBaseConnect again = connect_support_base(storage, core_boundary, core_bbox, ported_boundary, ported_bbox,
                                                          dense_line_spacing, 0.5f);
    REQUIRE(!first.core.empty());
    REQUIRE(!other.core.empty());
    require_same_infill(first.core, first.ported);
    require_same_infill(other.core, other.ported);
    require_same_infill(again.core, first.core);
}
