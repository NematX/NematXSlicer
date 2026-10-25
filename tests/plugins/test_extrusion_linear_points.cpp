///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
/*
Tests for extrusion_polyline_copy_linear_points().

A plugin may store arcs in an extrusion path (a segment with a radius). Code
that offsets or measures the printed path needs a plain polyline, so the host
replaces each arc by points on it, like ArcPolyline::to_polyline(). These tests
build a path with a quarter-circle arc through the C API and compare the copy
with the native conversion point for point; a straight path must come back
unchanged, and the two-call size query must not write anything.
*/

#include <catch2/catch.hpp>

#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_extrusion_polyline.h"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/ExtrusionEntity.hpp"

namespace {
// Borrow a native leaf as a C handle.
extrusion_entity_handle *entity_handle(Slic3r::ExtrusionEntity &entity)
{
    return reinterpret_cast<extrusion_entity_handle *>(&entity);
}
} // namespace

TEST_CASE("Linear points replace arcs like the native polyline conversion", "[plugins][extrusion][linear-points]")
{
    Slic3r::ExtrusionEntity leaf(false);
    extrusion_entity_handle *handle = entity_handle(leaf);
    // A straight segment, then a counter-clockwise quarter circle of radius 10 mm around (10 mm, 10 mm).
    const coord_t r = scale_i(10.);
    const std::vector<c_extrusion_segment> segments = {
        {{0, 0}, {r, 0}, 0.f, RAW_EXTRUSION_ARC_ORIENTATION_UNKNOWN, 0, 0},
        {{r, 0}, {2 * r, r}, float(r), RAW_EXTRUSION_ARC_ORIENTATION_CCW, 0, 0},
    };
    REQUIRE(extrusion_polyline_set_segments(handle, segments.data(), uint32_t(segments.size())) == 1);
    REQUIRE(leaf.polyline_ref().has_arc());

    const Slic3r::Polyline native = leaf.polyline_ref().to_polyline();
    REQUIRE(native.size() > 3);
    // The size query writes nothing; the copy then gives the native points in order.
    REQUIRE(extrusion_polyline_copy_linear_points(handle, nullptr, 0) == native.size());
    const std::vector<c_point> linear = slic3r_api::ExtrusionEntity(handle).linear_points();
    REQUIRE(linear.size() == native.size());
    for (size_t idx = 0; idx < linear.size(); ++idx) {
        INFO("point " << idx);
        CHECK(linear[idx].x == native.points[idx].x());
        CHECK(linear[idx].y == native.points[idx].y());
    }
    // copy_points still returns the stored points only: the arc ends, without intermediate points.
    CHECK(slic3r_api::ExtrusionEntity(handle).points().size() == 3);

    // Without arcs, the linear points are the stored points.
    const std::vector<c_point> straight = {{0, 0}, {r, 0}, {r, r}};
    REQUIRE(extrusion_polyline_set_points(handle, straight.data(), uint32_t(straight.size())) == 1);
    const std::vector<c_point> unchanged = slic3r_api::ExtrusionEntity(handle).linear_points();
    REQUIRE(unchanged.size() == straight.size());
    for (size_t idx = 0; idx < straight.size(); ++idx)
        CHECK((unchanged[idx].x == straight[idx].x && unchanged[idx].y == straight[idx].y));
    CHECK(extrusion_polyline_copy_linear_points(nullptr, nullptr, 0) == 0);
}
