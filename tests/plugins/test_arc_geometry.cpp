#include <catch2/catch.hpp>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "libslic3r/Api/plugin/cpp/gcode/ArcGeometry.hpp"
#include "libslic3r/Geometry/ArcWelder.hpp"

/*
The SDK arc_center() replaces the host's ArcWelder::arc_center() in firmware
encoders. These tests check that both give the same center for every radius
sign and direction, and that the SDK version rejects degenerate input instead
of relying on assertions.

    TEST_CASE
    |-- check_same_center(): compare SDK coordinates with the native helper
    `-- arc_center(): check known circles, clamping and invalid arguments
*/

namespace {

using slic3r_api::GCodeGeneration::arc_center;

// Compare the SDK helper with the host implementation on one arc.
void check_same_center(c_vec2d start, c_vec2d end, double radius, bool ccw);
// Return the Euclidean distance used to check the circle's radius at either end.
double distance(c_vec2d a, c_vec2d b);

void check_same_center(c_vec2d start, c_vec2d end, double radius, bool ccw)
{
    const c_vec2d sdk = arc_center(start, end, radius, ccw);
    const Slic3r::Vec2d host = Slic3r::Geometry::ArcWelder::arc_center(
        Slic3r::Vec2d(start.x, start.y), Slic3r::Vec2d(end.x, end.y), radius, ccw);
    INFO("radius=" << radius << " ccw=" << ccw);
    CHECK(sdk.x == Approx(host.x()).margin(1e-12));
    CHECK(sdk.y == Approx(host.y()).margin(1e-12));
}

double distance(c_vec2d a, c_vec2d b) { return std::hypot(a.x - b.x, a.y - b.y); }

} // namespace

TEST_CASE("SDK arc center matches the host ArcWelder center", "[plugins][gcode][arc]")
{
    const c_vec2d start{ 1.0, 2.0 };
    const c_vec2d end{ 7.0, 5.0 };
    for (double radius : { 4.0, -4.0, 10.0, -10.0 })
        for (bool ccw : { true, false })
            check_same_center(start, end, radius, ccw);
}

TEST_CASE("SDK arc center lies at the radius from both ends", "[plugins][gcode][arc]")
{
    // Quarter circle of radius 5 around the origin, from (5,0) to (0,5).
    const c_vec2d start{ 5.0, 0.0 };
    const c_vec2d end{ 0.0, 5.0 };

    // The short counter-clockwise arc turns around the origin.
    const c_vec2d short_ccw = arc_center(start, end, 5.0, true);
    CHECK(short_ccw.x == Approx(0.0).margin(1e-12));
    CHECK(short_ccw.y == Approx(0.0).margin(1e-12));

    // The long counter-clockwise arc uses the center on the other side.
    const c_vec2d long_ccw = arc_center(start, end, -5.0, true);
    CHECK(long_ccw.x == Approx(5.0));
    CHECK(long_ccw.y == Approx(5.0));
    CHECK(distance(long_ccw, start) == Approx(5.0));
    CHECK(distance(long_ccw, end) == Approx(5.0));

    // A short clockwise arc shares its center with the long counter-clockwise one.
    const c_vec2d short_cw = arc_center(start, end, 5.0, false);
    CHECK(short_cw.x == Approx(long_ccw.x));
    CHECK(short_cw.y == Approx(long_ccw.y));
}

TEST_CASE("SDK arc center uses the chord midpoint for a half circle", "[plugins][gcode][arc]")
{
    // A radius exactly half the chord, or slightly smaller after rounding,
    // has its center at the chord's midpoint, as in ArcWelder.
    const c_vec2d start{ 0.0, 0.0 };
    const c_vec2d end{ 10.0, 0.0 };
    for (double radius : { 5.0, 4.9999999 }) {
        const c_vec2d center = arc_center(start, end, radius, true);
        CHECK(center.x == Approx(5.0));
        CHECK(center.y == Approx(0.0).margin(1e-12));
        check_same_center(start, end, radius, true);
    }
}

TEST_CASE("SDK arc center rejects degenerate arcs", "[plugins][gcode][arc]")
{
    const c_vec2d start{ 0.0, 0.0 };
    const c_vec2d end{ 10.0, 0.0 };
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_THROWS_AS(arc_center(start, end, 0.0, true), std::invalid_argument);
    CHECK_THROWS_AS(arc_center(start, end, nan, true), std::invalid_argument);
    CHECK_THROWS_AS(arc_center(start, start, 5.0, true), std::invalid_argument);
    CHECK_THROWS_AS(arc_center(c_vec2d{ nan, 0.0 }, end, 5.0, true), std::invalid_argument);
}
