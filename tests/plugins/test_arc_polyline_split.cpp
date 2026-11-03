/*
Distance splitting uses XY lengths in internal coordinate units. Z offsets belong
to vertices; vertical moves consume no XY distance. The split boundary is called
the seam. Host segment publication installs Z before validating lifted arcs.
*/
#include <catch2/catch.hpp>

#include <cmath>
#include <initializer_list>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_extrusion_polyline.h"
#include "libslic3r/ExtrusionEntity.hpp"

namespace {
using namespace Slic3r;
using XYZ = std::pair<Point, coord_t>;

// Borrow an entity as an opaque host handle without transferring ownership.
extrusion_entity_handle *handle(ExtrusionEntity &entity)
{
    return reinterpret_cast<extrusion_entity_handle *>(&entity);
}

// Publish contiguous vertices through the host, requiring success; return an owned copy, optionally with an arc on segment 2.
ArcPolyline published_path(const std::vector<XYZ> &vertices, float radius = 0.f)
{
    REQUIRE(vertices.size() > 1);
    std::vector<c_extrusion_segment> segments;
    for (size_t i = 1; i < vertices.size(); ++i) {
        c_extrusion_segment segment{};
        segment.point_a = {vertices[i - 1].first.x(), vertices[i - 1].first.y()};
        segment.point_b = {vertices[i].first.x(), vertices[i].first.y()};
        segment.z_offset_a = vertices[i - 1].second;
        segment.z_offset_b = vertices[i].second;
        segment.radius = i == 2 ? radius : 0.f;
        segment.orientation = segment.radius == 0.f ? RAW_EXTRUSION_ARC_ORIENTATION_UNKNOWN : RAW_EXTRUSION_ARC_ORIENTATION_CCW;
        segments.push_back(segment);
    }
    ExtrusionPath entity(ExtrusionAttributes(ExtrusionRole::Travel, ExtrusionFlow(0., 0.4f, 0.2f)), true);
    REQUIRE(extrusion_polyline_set_segments(handle(entity), segments.data(), uint32_t(segments.size())) == 1);
    return entity.polyline_ref();
}

// Check every expected XYZ and Z-array bound, requiring matching sizes before indexing; leave the path untouched.
void check_xyz(const ArcPolyline &path, std::initializer_list<XYZ> expected)
{
    REQUIRE(path.size() == expected.size());
    REQUIRE(path.has_z_offset());
    size_t i = 0;
    for (const XYZ &vertex : expected) {
        CHECK(path.get_point(i) == vertex.first);
        CHECK(path.z_offset(i) == vertex.second);
        ++i;
    }
    CHECK(path.z_offset(path.size()) == ArcPolyline::INVALID_COORD);
    CHECK(path.is_valid());
}

// Compare all stored vertices, arc metadata and Z presence against an owned snapshot; no geometry is modified.
void check_unchanged(const ArcPolyline &path, const ArcPolyline &snapshot)
{
    REQUIRE(path.size() == snapshot.size());
    CHECK(path.has_z_offset() == snapshot.has_z_offset());
    for (size_t i = 0; i < path.size(); ++i) {
        CHECK(path.get_point(i) == snapshot.get_point(i));
        CHECK(path.z_offset(i) == snapshot.z_offset(i));
        CHECK(path.get_arc(i).radius == snapshot.get_arc(i).radius);
        CHECK(path.get_arc(i).orientation == snapshot.get_arc(i).orientation);
#ifdef _DEBUG
        CHECK(path.get_arc(i).length == snapshot.get_arc(i).length);
        CHECK(path.get_arc(i).center == snapshot.get_arc(i).center);
#endif
    }
#ifdef _DEBUG
    CHECK(path.is_3D == snapshot.is_3D);
#endif
}

// Check planar length conservation and seam XYZ with existing epsilon, without changing either output or source.
void check_split(const ArcPolyline &source, const ArcPolyline &first, const ArcPolyline &second)
{
    CHECK(first.is_valid());
    CHECK(second.is_valid());
    CHECK(first.back() == second.front());
    CHECK(first.z_offset(first.size() - 1) == second.z_offset(0));
    CHECK(std::abs(source.length() - first.length() - second.length()) <= double(SCALED_EPSILON));
#ifdef _DEBUG
    CHECK(first.is_3D == (source.is_3D && source.has_z_offset()));
    CHECK(second.is_3D == (source.is_3D && source.has_z_offset()));
#endif
}
} // namespace

// Split a host-published lift at its plateau; assert every XYZ and source immutability.
TEST_CASE("Lifted straight distance split preserves vertical moves", "[plugins][arc-polyline-split][arc-polyline-lift-red]")
{
    const coord_t u = scale_i(1.), z = scale_i(.4);
    const ArcPolyline source = published_path({{{0, 0}, 0}, {{0, 0}, z}, {{10*u, 0}, z}, {{10*u, 0}, 0}});
    const ArcPolyline snapshot(source);
    ArcPolyline first, second;
    source.split_at(scale_d(5.), first, second);
    check_xyz(first, {{{0, 0}, 0}, {{0, 0}, z}, {{5*u, 0}, z}});
    check_xyz(second, {{{5*u, 0}, z}, {{10*u, 0}, z}, {{10*u, 0}, 0}});
    check_split(source, first, second);
    check_unchanged(source, snapshot);
}

// Cut inside a lifted CCW arc, checking its circle, orientation, seam and preserved endpoint Z values.
TEST_CASE("Lifted arc distance split follows the arc", "[plugins][arc-polyline-split]")
{
    const coord_t u = scale_i(1.), z = scale_i(.4);
    const ArcPolyline source = published_path({{{0, 0}, 0}, {{0, 0}, z}, {{10*u, 0}, z}, {{10*u, 0}, 0}}, float(10*u));
    const ArcPolyline snapshot(source);
    ArcPolyline first, second;
    source.split_at(source.length() / 2., first, second);
    REQUIRE(first.size() == 3);
    REQUIRE(second.size() == 3);
    CHECK(first.front() == Point(0, 0));
    CHECK(first.get_point(1) == Point(0, 0));
    CHECK(first.z_offset(0) == 0);
    CHECK(first.z_offset(1) == z);
    CHECK(first.z_offset(2) == z);
    CHECK(second.get_point(1) == Point(10*u, 0));
    CHECK(second.back() == Point(10*u, 0));
    CHECK(second.z_offset(0) == z);
    CHECK(second.z_offset(1) == z);
    CHECK(second.z_offset(2) == 0);
    CHECK(first.z_offset(first.size()) == ArcPolyline::INVALID_COORD);
    CHECK(second.z_offset(second.size()) == ArcPolyline::INVALID_COORD);
    const Vec2d center = Geometry::ArcWelder::arc_center(Point(0, 0).cast<double>(), Point(10*u, 0).cast<double>(), double(10*u), true);
    CHECK(std::abs((first.back().cast<double>() - center).norm() - double(10*u)) <= double(SCALED_EPSILON));
    CHECK(std::abs(first.back().y()) > SCALED_EPSILON);
    CHECK(std::abs(first.length() - source.length()/2.) <= double(SCALED_EPSILON));
    CHECK(std::abs(second.length() - source.length()/2.) <= double(SCALED_EPSILON));
    CHECK(std::abs(first.get_arc(2).radius) == float(10*u));
    CHECK(std::abs(second.get_arc(1).radius) == float(10*u));
    CHECK(first.get_arc(2).orientation == Geometry::ArcWelder::Orientation::CCW);
    CHECK(second.get_arc(1).orientation == Geometry::ArcWelder::Orientation::CCW);
    check_split(source, first, second);
    check_unchanged(source, snapshot);
}

// Cut at a linear vertex immediately before a vertical move, which must remain in the suffix.
TEST_CASE("Vertical move at a linear split vertex belongs to suffix", "[plugins][arc-polyline-split]")
{
    const coord_t u = scale_i(1.), z = scale_i(.4);
    const ArcPolyline source = published_path({{{0, 0}, 0}, {{5*u, 0}, 0}, {{5*u, 0}, z}, {{10*u, 0}, z}});
    const ArcPolyline snapshot(source);
    ArcPolyline first, second;
    source.split_at(scale_d(5.), first, second);
    check_xyz(first, {{{0, 0}, 0}, {{5*u, 0}, 0}});
    check_xyz(second, {{{5*u, 0}, 0}, {{5*u, 0}, z}, {{10*u, 0}, z}});
    check_split(source, first, second);
    check_unchanged(source, snapshot);
}

// Interpolate changing plateau Z while preserving the full original raise and lower.
TEST_CASE("Distance split interpolates plateau offsets", "[plugins][arc-polyline-split]")
{
    const coord_t u = scale_i(1.);
    const ArcPolyline source = published_path({{{0, 0}, 0}, {{0, 0}, scale_i(.2)}, {{10*u, 0}, scale_i(.6)}, {{10*u, 0}, 0}});
    const ArcPolyline snapshot(source);
    ArcPolyline first, second;
    source.split_at(scale_d(5.), first, second);
    check_xyz(first, {{{0, 0}, 0}, {{0, 0}, scale_i(.2)}, {{5*u, 0}, scale_i(.4)}});
    check_xyz(second, {{{5*u, 0}, scale_i(.4)}, {{10*u, 0}, scale_i(.6)}, {{10*u, 0}, 0}});
    check_split(source, first, second);
    check_unchanged(source, snapshot);
}

// Reuse cleared 3D outputs for 2D and zero-offset sources, checking inherited rather than inferred validation state.
TEST_CASE("Distance split resets reused output dimension state", "[plugins][arc-polyline-split]")
{
    const coord_t u = scale_i(1.);
    ArcPolyline source(Points{{0, 0}, {10*u, 0}, {10*u, 10*u}});
    SECTION("ordinary 2D") {}
    SECTION("explicit zero offsets") {
        for (size_t i = 0; i < source.size(); ++i)
            source.set_z_offset(i, 0);
    }
    const ArcPolyline snapshot(source);
    ArcPolyline first(Points{{0, 0}, {u, 0}}), second(first);
    first.set_z_offset(0, u);
    second.set_z_offset(0, u);
    first.clear();
    second.clear();
    source.split_at(scale_d(15.), first, second);
    REQUIRE(first.size() == 3);
    REQUIRE(second.size() == 2);
    CHECK(first.get_point(0) == Point(0, 0));
    CHECK(first.get_point(1) == Point(10*u, 0));
    CHECK(first.back() == Point(10*u, 5*u));
    CHECK(second.front() == Point(10*u, 5*u));
    CHECK(second.back() == Point(10*u, 10*u));
    CHECK(first.has_z_offset() == source.has_z_offset());
    CHECK(second.has_z_offset() == source.has_z_offset());
    if (source.has_z_offset()) {
        check_xyz(first, {{{0, 0}, 0}, {{10*u, 0}, 0}, {{10*u, 5*u}, 0}});
        check_xyz(second, {{{10*u, 5*u}, 0}, {{10*u, 10*u}, 0}});
    }
    check_split(source, first, second);
    check_unchanged(source, snapshot);
}

// Retain an identical initial XYZ from a lifted producer; the positive planar cut consumes it into the prefix.
TEST_CASE("Distance split accepts inherited lift with identical initial XYZ", "[plugins][arc-polyline-split]")
{
    const coord_t u = scale_i(1.), z = scale_i(.4);
    const ArcPolyline source = published_path({{{0, 0}, z}, {{0, 0}, z}, {{10*u, 0}, z}, {{10*u, 0}, 0}});
    const ArcPolyline snapshot(source);
    ArcPolyline first, second;
    source.split_at(scale_d(5.), first, second);
    check_xyz(first, {{{0, 0}, z}, {{0, 0}, z}, {{5*u, 0}, z}});
    check_xyz(second, {{{5*u, 0}, z}, {{10*u, 0}, z}, {{10*u, 0}, 0}});
    check_split(source, first, second);
    check_unchanged(source, snapshot);
}

// Exercise the host's temporary outputs with source aliasing its second destination; preserve properties but not setter can_sort.
TEST_CASE("Host distance split supports lifted source alias", "[plugins][arc-polyline-split]")
{
    const coord_t u = scale_i(1.), z = scale_i(.4);
    const ExtrusionAttributes attributes(ExtrusionRole::Travel, ExtrusionFlow(0., .4f, .2f));
    ExtrusionPath source(published_path({{{0, 0}, 0}, {{0, 0}, z}, {{10*u, 0}, z}, {{10*u, 0}, 0}}), attributes, nullptr, true);
    ExtrusionPath first(attributes, true);
    source.add_property(ExtrusionPropertySpeed(42.f, 900.f));
    first.add_property(ExtrusionPropertySpeed(17.f, 500.f));
    const bool source_reverse = source.can_reverse(), first_reverse = first.can_reverse();
    REQUIRE(extrusion_polyline_split_at_distance(handle(source), scale_d(5.), handle(first), handle(source)) == 1);
    check_xyz(first.polyline_ref(), {{{0, 0}, 0}, {{0, 0}, z}, {{5*u, 0}, z}});
    check_xyz(source.polyline_ref(), {{{5*u, 0}, z}, {{10*u, 0}, z}, {{10*u, 0}, 0}});
    CHECK(source.can_reverse() == source_reverse);
    CHECK(first.can_reverse() == first_reverse);
    REQUIRE(source.get_property<ExtrusionPropertySpeed>() != nullptr);
    REQUIRE(first.get_property<ExtrusionPropertySpeed>() != nullptr);
    CHECK(source.get_property<ExtrusionPropertySpeed>()->speed_mm_per_s == 42.f);
    CHECK(source.get_property<ExtrusionPropertySpeed>()->accel_mm_per_s2 == 900.f);
    CHECK(first.get_property<ExtrusionPropertySpeed>()->speed_mm_per_s == 17.f);
    CHECK(first.get_property<ExtrusionPropertySpeed>()->accel_mm_per_s2 == 500.f);
    CHECK(source.polyline_ref().length() + first.polyline_ref().length() == scale_d(10.));
}

// Cover the integer short-segment cleanup where representable sub-epsilon segments exist; check all XYZ in both branches.
TEST_CASE("Short split cleanup preserves vertex offsets", "[plugins][arc-polyline-split][arc-polyline-cleanup-red]")
{
    if (SCALED_EPSILON < 4) {
        SUCCEED("No positive integer segment represents three quarters of epsilon in this coordinate mode");
        return;
    }
    const coord_t u = scale_i(1.), z4 = scale_i(.4), z8 = scale_i(.8);
    const coord_t delta = 3*SCALED_EPSILON/4;
    coord_t middle_z = z4;
    bool merge = false;
    SECTION("distinct final Z is retained") {}
    SECTION("equal final Z merges without altering previous Z") { middle_z = z8; merge = true; }
    const ArcPolyline source = published_path({{{0, 0}, 0}, {{0, 0}, z4}, {{5*u, 0}, middle_z}, {{5*u+delta, 0}, z8}, {{10*u, 0}, z8}, {{10*u, 0}, 0}});
    const ArcPolyline snapshot(source);
    ArcPolyline first, second;
    source.split_at(scale_d(5.) + 3*SCALED_EPSILON/2, first, second);
    if (merge)
        check_xyz(first, {{{0, 0}, 0}, {{0, 0}, z4}, {{5*u+delta, 0}, z8}});
    else
        check_xyz(first, {{{0, 0}, 0}, {{0, 0}, z4}, {{5*u, 0}, z4}, {{5*u+delta, 0}, z8}});
    check_xyz(second, {{{5*u+delta, 0}, z8}, {{10*u, 0}, z8}, {{10*u, 0}, 0}});
    check_split(source, first, second);
    check_unchanged(source, snapshot);
}

// Deliberately abort on a suffix XYZ duplicate despite 3D provenance; only explicit hidden-tag selection may run this probe.
TEST_CASE("Identical suffix XYZ is rejected by distance split", "[.][arc-polyline-invalid-suffix-probe]")
{
    const coord_t u = scale_i(1.), z = scale_i(.4);
    const ArcPolyline source = published_path({{{0, 0}, 0}, {{0, 0}, z}, {{10*u, 0}, z}, {{10*u, 0}, z}});
    ArcPolyline first, second;
    source.split_at(scale_d(5.), first, second);
    FAIL("The suffix XYZ duplicate must be rejected by an active assertion");
}
