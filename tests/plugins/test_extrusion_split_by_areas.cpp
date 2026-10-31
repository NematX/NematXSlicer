///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
Tests for the public extrusion area and length splitters.

The fixtures deliberately provide complete, non-overlapping partitions. The
API contract leaves overlap and missing-coverage behavior undefined, so these
tests focus on stable handles, source order, preserved extrusion state and the
exact ArcPolyline geometry reconstructed after temporary clipping.
Direct distance-split regressions also cover short XY segments carrying a Z
change, before this primitive is used by any regional or length-based splitter.

Length requests also verify strictly increasing cuts, minimum fragment lengths
and rejection of every one-shot event before publication. Native and C++ SDK
entry points must preserve the same properties, source intervals and volume.

    TEST_CASE
    |-- straight_path(): own a small native extrusion with explicit process data
    |-- extrusion_split_leaf_by_areas(): classify and then recover native spans
    `-- extrusion_split_leaf_at_lengths(): publish caller-given native spans
        `-- collect_fragment(): copy borrowed child handles and interval indices
*/

#include <catch2/catch.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_extrusion_entity.h"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/Geometry/ArcWelder.hpp"

namespace {

using namespace Slic3r;

struct CallbackCapture
{
    std::vector<extrusion_entity_handle *> handles;
    std::vector<uint32_t> area_indices;
};

ExPolygon rectangle(coord_t min_x, coord_t min_y, coord_t max_x, coord_t max_y);
ExtrusionPath straight_path(const Points &points);
extrusion_entity_handle *entity_handle(ExtrusionEntity &entity);
const expolygon_collection_handle *area_handle(const ExPolygons &areas);
std::vector<const expolygon_collection_handle *> area_handles(const std::vector<ExPolygons> &areas);
void collect_fragment(extrusion_entity_handle *fragment, uint32_t area_index, void *user_data);

ExPolygon rectangle(coord_t min_x, coord_t min_y, coord_t max_x, coord_t max_y)
{
    return ExPolygon({ Point(min_x, min_y), Point(max_x, min_y),
                       Point(max_x, max_y), Point(min_x, max_y) });
}

ExtrusionPath straight_path(const Points &points)
{
    const ExtrusionAttributes attributes(
        ExtrusionRole::Perimeter,
        ExtrusionFlow(0.08, 0.4f, 0.2f));
    return ExtrusionPath(ArcPolyline(points), attributes, nullptr, true);
}

extrusion_entity_handle *entity_handle(ExtrusionEntity &entity)
{
    return reinterpret_cast<extrusion_entity_handle *>(&entity);
}

const expolygon_collection_handle *area_handle(const ExPolygons &areas)
{
    return reinterpret_cast<const expolygon_collection_handle *>(&areas);
}

std::vector<const expolygon_collection_handle *> area_handles(const std::vector<ExPolygons> &areas)
{
    std::vector<const expolygon_collection_handle *> out;
    out.reserve(areas.size());
    for (const ExPolygons &area : areas)
        out.push_back(area_handle(area));
    return out;
}

void collect_fragment(extrusion_entity_handle *fragment, uint32_t area_index, void *user_data)
{
    CallbackCapture &capture = *static_cast<CallbackCapture *>(user_data);
    capture.handles.push_back(fragment);
    capture.area_indices.push_back(area_index);
}

} // namespace

TEST_CASE("ArcPolyline distance splitting preserves Z changes at a nearby vertex",
          "[plugins][extrusion][arc-polyline-split-z]")
{
    const coord_t unit = scale_i(10.);
    const coord_t offset = SCALED_EPSILON / 2;
    const Points points{Point(0, 0), Point(unit, 0), Point(2 * unit, 0),
                        Point(2 * unit + offset, 0), Point(3 * unit, 0)};
    for (bool changing_z : {false, true}) {
        CAPTURE(changing_z);
        ArcPolyline source(points);
        for (size_t idx = 0; idx < source.size(); ++idx)
            source.set_z_offset(idx, changing_z ? coord_t(idx) * unit : 100);
        ArcPolyline prefix, suffix;
        // Reach the short source segment before the usual epsilon snap chooses
        // its endpoint. Cleanup may remove it only when its Z is unchanged.
        source.split_at(distf_t(2 * unit + SCALED_EPSILON + SCALED_EPSILON / 4), prefix, suffix);

        REQUIRE(prefix.size() == (changing_z ? 4 : 3));
        REQUIRE(suffix.size() == 2);
        CHECK(prefix.back() == Point(2 * unit + offset, 0));
        CHECK(suffix.front() == prefix.back());
        CHECK(suffix.back() == source.back());
        CHECK(prefix.z_offset(prefix.size() - 1) == (changing_z ? 3 * unit : 100));
        CHECK(suffix.z_offset(0) == prefix.z_offset(prefix.size() - 1));
        if (changing_z) {
            CHECK(prefix.get_point(2) == Point(2 * unit, 0));
            CHECK(prefix.z_offset(2) == 2 * unit);
        }
        CHECK(prefix.length() + suffix.length() == Approx(source.length()).margin(1.));
        REQUIRE(source.size() == points.size());
        for (size_t idx = 0; idx < source.size(); ++idx) {
            CHECK(source.get_point(idx) == points[idx]);
            CHECK(source.z_offset(idx) == (changing_z ? coord_t(idx) * unit : 100));
        }
    }
}

TEST_CASE("ArcPolyline distance splitting keeps a short XY segment with distinct Z",
          "[plugins][extrusion][arc-polyline-split-z]")
{
    const coord_t unit = scale_i(10.);
    const coord_t offset = SCALED_EPSILON / 4;
    ArcPolyline source(Points{Point(0, 0), Point(unit, 0), Point(unit + offset, 0), Point(2 * unit, 0)});
    for (size_t idx = 0; idx < source.size(); ++idx)
        source.set_z_offset(idx, coord_t(idx) * unit);
    ArcPolyline prefix, suffix;
    source.split_at(distf_t(unit / 2), prefix, suffix);

    REQUIRE(prefix.size() == 2);
    REQUIRE(suffix.size() == 4);
    CHECK(prefix.z_offset(1) == unit / 2);
    CHECK(suffix.z_offset(0) == prefix.z_offset(1));
    CHECK(suffix.get_point(1) == Point(unit, 0));
    CHECK(suffix.get_point(2) == Point(unit + offset, 0));
    CHECK(suffix.z_offset(1) == unit);
    CHECK(suffix.z_offset(2) == 2 * unit);
    CHECK(suffix.back() == source.back());
    CHECK(prefix.length() + suffix.length() == Approx(source.length()).margin(1.));
    CHECK(source.is_valid());
}

TEST_CASE("Extrusion leaves split by area in source order",
          "[plugins][extrusion][split-by-areas]")
{
    const coord_t unit = scale_i(1.);
    ExtrusionPath leaf = straight_path({ Point(0, 0), Point(30 * unit, 0) });
    leaf.add_property(ExtrusionPropertySpeed(42.f, 900.f));

    const std::vector<ExPolygons> areas = {
        { rectangle(-unit, -unit, 10 * unit, unit) },
        { rectangle(10 * unit, -unit, 20 * unit, unit) },
        { rectangle(20 * unit, -unit, 31 * unit, unit) }
    };
    const std::vector<const expolygon_collection_handle *> handles = area_handles(areas);
    CallbackCapture capture;
    capture.handles.reserve(3);
    capture.area_indices.reserve(3);

    const raw_extrusion_split_status status = extrusion_split_leaf_by_areas(
        entity_handle(leaf), handles.data(), uint32_t(handles.size()), SCALED_EPSILON,
        collect_fragment, &capture);

    REQUIRE(status == RAW_EXTRUSION_SPLIT_STATUS_SUCCESS);
    REQUIRE(leaf.child_count() == 3);
    CAPTURE(leaf.can_sort(), leaf.is_continuous(),
            leaf.child(0).last_point().x(), leaf.child(0).last_point().y(),
            leaf.child(1).first_point().x(), leaf.child(1).first_point().y(),
            leaf.child(1).last_point().x(), leaf.child(1).last_point().y(),
            leaf.child(2).first_point().x(), leaf.child(2).first_point().y());
    CHECK_FALSE(leaf.can_sort());
    CHECK(leaf.is_continuous());
    CHECK(capture.area_indices == std::vector<uint32_t>{ 0, 1, 2 });

    for (size_t child_idx = 0; child_idx < leaf.child_count(); ++child_idx) {
        const ExtrusionEntity &child = leaf.child(child_idx);
        REQUIRE(capture.handles[child_idx] == entity_handle(const_cast<ExtrusionEntity &>(child)));
        CHECK(child.can_reverse());
        REQUIRE(child.get_property<ExtrusionPropertySpeed>() != nullptr);
        CHECK(child.get_property<ExtrusionPropertySpeed>()->speed_mm_per_s == 42.f);
        CHECK(child.get_property<ExtrusionPropertySpeed>()->accel_mm_per_s2 == 900.f);
    }

    CHECK(leaf.child(0).first_point() == Point(0, 0));
    CHECK(leaf.child(0).last_point() == leaf.child(1).first_point());
    CHECK(leaf.child(1).last_point() == leaf.child(2).first_point());
    CHECK(leaf.child(2).last_point() == Point(30 * unit, 0));
}

TEST_CASE("A single extrusion area preserves the leaf identity",
          "[plugins][extrusion][split-by-areas]")
{
    const coord_t unit = scale_i(1.);
    ExtrusionPath leaf = straight_path({ Point(0, 0), Point(10 * unit, 0) });
    const ExPolygons area = { rectangle(-unit, -unit, 11 * unit, unit) };
    const expolygon_collection_handle *handle = area_handle(area);
    extrusion_entity_handle *const original_handle = entity_handle(leaf);
    CallbackCapture capture;
    capture.handles.reserve(1);
    capture.area_indices.reserve(1);

    REQUIRE(extrusion_split_leaf_by_areas(
                original_handle, &handle, 1, 0, collect_fragment, &capture) ==
            RAW_EXTRUSION_SPLIT_STATUS_SUCCESS);

    CHECK(leaf.has_polyline());
    CHECK(leaf.child_count() == 0);
    REQUIRE(capture.handles.size() == 1);
    CHECK(capture.handles.front() == original_handle);
    CHECK(capture.area_indices == std::vector<uint32_t>{ 0 });

    // The callback is optional and does not change the successful no-op path.
    REQUIRE(extrusion_split_leaf_by_areas(
                original_handle, &handle, 1, SCALED_EPSILON, nullptr, nullptr) ==
            RAW_EXTRUSION_SPLIT_STATUS_SUCCESS);
    CHECK(leaf.has_polyline());

    const std::vector<ExPolygons> areas_with_empty = { {}, area };
    const std::vector<const expolygon_collection_handle *> handles = area_handles(areas_with_empty);
    CallbackCapture empty_capture;
    REQUIRE(extrusion_split_leaf_by_areas(
                original_handle, handles.data(), uint32_t(handles.size()), SCALED_EPSILON,
                collect_fragment, &empty_capture) == RAW_EXTRUSION_SPLIT_STATUS_SUCCESS);
    CHECK(empty_capture.handles == std::vector<extrusion_entity_handle *>{ original_handle });
    CHECK(empty_capture.area_indices == std::vector<uint32_t>{ 1 });
    CHECK(leaf.has_polyline());
}

TEST_CASE("Adjacent pieces owned by one area become one extrusion fragment",
          "[plugins][extrusion][split-by-areas]")
{
    const coord_t unit = scale_i(1.);
    ExtrusionPath leaf = straight_path({ Point(0, 0), Point(20 * unit, 0) });

    // Area zero intentionally contains two touching polygons. Clipper may
    // return them as separate intervals, but they have identical ownership and
    // are adjacent along the source path.
    const std::vector<ExPolygons> areas = {
        { rectangle(-unit, -unit, 5 * unit, unit),
          rectangle(5 * unit, -unit, 10 * unit, unit) },
        { rectangle(10 * unit, -unit, 21 * unit, unit) }
    };
    const std::vector<const expolygon_collection_handle *> handles = area_handles(areas);
    CallbackCapture capture;
    capture.handles.reserve(2);
    capture.area_indices.reserve(2);

    REQUIRE(extrusion_split_leaf_by_areas(
                entity_handle(leaf), handles.data(), uint32_t(handles.size()), SCALED_EPSILON,
                collect_fragment, &capture) == RAW_EXTRUSION_SPLIT_STATUS_SUCCESS);

    REQUIRE(leaf.child_count() == 2);
    CHECK(capture.area_indices == std::vector<uint32_t>{ 0, 1 });
    CHECK(leaf.child(0).first_point() == Point(0, 0));
    CHECK(leaf.child(0).last_point() == Point(10 * unit, 0));
}

TEST_CASE("Extrusion area splitting handles holes near and far from the path",
          "[plugins][extrusion][split-by-areas]")
{
    const coord_t unit = scale_i(1.);
    // The diagonal path enters the first hole at x = 10 and leaves it through
    // its top at x = 15. The second hole is near the path (their boxes touch)
    // but never crossed; the third hole and its filling are far away.
    ExtrusionPath leaf = straight_path({ Point(0, 0), Point(30 * unit, 4 * unit) });
    const ExPolygon crossed_hole = rectangle(10 * unit, -2 * unit, 20 * unit, 2 * unit);
    const ExPolygon near_hole = rectangle(2 * unit, 3 * unit, 4 * unit, 4 * unit + unit / 2);
    const ExPolygon far_hole = rectangle(26 * unit, -4 * unit, 28 * unit, -3 * unit);
    ExPolygon perforated = rectangle(-unit, -5 * unit, 31 * unit, 5 * unit);
    for (const ExPolygon &hole : { crossed_hole, near_hole, far_hole }) {
        perforated.holes.push_back(hole.contour);
        perforated.holes.back().reverse();
    }
    // The second area fills the three holes, so the two areas cover the path.
    const std::vector<ExPolygons> areas = { { perforated }, { crossed_hole, near_hole, far_hole } };
    const std::vector<const expolygon_collection_handle *> handles = area_handles(areas);
    CallbackCapture capture;

    REQUIRE(extrusion_split_leaf_by_areas(
                entity_handle(leaf), handles.data(), uint32_t(handles.size()), SCALED_EPSILON,
                collect_fragment, &capture) == RAW_EXTRUSION_SPLIT_STATUS_SUCCESS);

    REQUIRE(leaf.child_count() == 3);
    CHECK(capture.area_indices == std::vector<uint32_t>{ 0, 1, 0 });
    const double tolerance = 2. * double(SCALED_EPSILON);
    CHECK((leaf.child(0).last_point() - Point(10 * unit, 4 * unit / 3)).cast<double>().norm() < tolerance);
    CHECK((leaf.child(1).last_point() - Point(15 * unit, 2 * unit)).cast<double>().norm() < tolerance);
    CHECK(leaf.child(2).last_point() == Point(30 * unit, 4 * unit));
}

TEST_CASE("Indexed area splitting matches whole-area clipping across a perforated region",
          "[plugins][extrusion][split-by-areas]")
{
    const coord_t unit = scale_i(1.);
    ExPolygon perforated = rectangle(-unit, -unit, 81 * unit, 81 * unit);
    ExPolygons fillings;
    for (coord_t row = 0; row < 8; ++row)
        for (coord_t column = 0; column < 8; ++column) {
            const coord_t x = (5 + 10 * column) * unit;
            const coord_t y = (5 + 10 * row) * unit;
            fillings.push_back(rectangle(x - unit, y - unit, x + unit, y + unit));
            perforated.holes.push_back(fillings.back().contour);
            perforated.holes.back().reverse();
        }
    const std::vector<ExPolygons> areas = {{perforated}, fillings};
    const std::vector<const expolygon_collection_handle *> handles = area_handles(areas);
    for (coord_t row = 0; row < 8; ++row) {
        const coord_t y = (5 + 10 * row) * unit;
        const Polyline source(Point(0, y), Point(80 * unit, y));
        std::vector<std::pair<uint32_t, Polyline>> expected;
        for (uint32_t area_idx = 0; area_idx < areas.size(); ++area_idx)
            for (Polyline fragment : intersection_pl(source, areas[area_idx])) {
                if (fragment.front().x() > fragment.back().x())
                    fragment.reverse();
                expected.emplace_back(area_idx, std::move(fragment));
            }
        std::sort(expected.begin(), expected.end(),
            [](const std::pair<uint32_t, Polyline> &lhs, const std::pair<uint32_t, Polyline> &rhs) {
                return lhs.second.front().x() < rhs.second.front().x();
            });
        REQUIRE(expected.size() == 17);
        ExtrusionPath leaf = straight_path(source.points);
        CallbackCapture capture;
        REQUIRE(extrusion_split_leaf_by_areas(entity_handle(leaf), handles.data(), uint32_t(handles.size()),
                    SCALED_EPSILON, collect_fragment, &capture) == RAW_EXTRUSION_SPLIT_STATUS_SUCCESS);
        REQUIRE(leaf.child_count() == expected.size());
        REQUIRE(capture.area_indices.size() == expected.size());
        for (size_t idx = 0; idx < expected.size(); ++idx) {
            CAPTURE(row, idx);
            CHECK(capture.area_indices[idx] == expected[idx].first);
            CHECK(leaf.child(idx).first_point() == expected[idx].second.front());
            CHECK(leaf.child(idx).last_point() == expected[idx].second.back());
            CHECK(leaf.child(idx).length() == expected[idx].second.length());
        }
    }
}

TEST_CASE("Arc and Z geometry survive extrusion area splitting",
          "[plugins][extrusion][split-by-areas]")
{
    const coord_t unit = scale_i(1.);
    ArcPolyline source;
    source.append(Point(unit, 0));
    source.append(Geometry::ArcWelder::Segment(
        Point(0, unit), float(unit), Geometry::ArcWelder::Orientation::CCW));
    source.set_z_offset(0, 100);
    source.set_z_offset(1, 900);
    const ExtrusionAttributes attributes(
        ExtrusionRole::Perimeter,
        ExtrusionFlow(0.08, 0.4f, 0.2f));
    ExtrusionPath leaf(std::move(source), attributes, nullptr, true);

    const coord_t split_x = scale_i(0.7);
    const std::vector<ExPolygons> areas = {
        { rectangle(split_x, -unit, 2 * unit, 2 * unit) },
        { rectangle(-unit, -unit, split_x, 2 * unit) }
    };
    std::vector<slic3r_api::ExPolygonCollection> area_views;
    for (const ExPolygons &area : areas)
        area_views.emplace_back(area_handle(area));

    const std::vector<slic3r_api::ExtrusionAreaFragment> fragments =
        slic3r_api::MutableExtrusionEntity(entity_handle(leaf)).split_leaf_by_areas(area_views);

    REQUIRE(fragments.size() == 2);
    CHECK(fragments[0].area_index == 0);
    CHECK(fragments[1].area_index == 1);
    REQUIRE(leaf.child_count() == 2);

    const ArcPolyline &first = leaf.child(0).polyline_ref();
    const ArcPolyline &second = leaf.child(1).polyline_ref();
    CHECK(first.has_arc());
    CHECK(second.has_arc());
    CHECK(first.has_z_offset());
    CHECK(second.has_z_offset());
    CHECK(first.back() == second.front());
    CHECK(first.z_offset(first.size() - 1) == second.z_offset(0));
    CHECK(first.z_offset(first.size() - 1) > 100);
    CHECK(first.z_offset(first.size() - 1) < 900);
    CHECK(first.get_arc().back().orientation == Geometry::ArcWelder::Orientation::CCW);
    CHECK(second.get_arc().back().orientation == Geometry::ArcWelder::Orientation::CCW);
    CHECK(std::abs(first.get_arc().back().radius) == Approx(double(unit)));
    CHECK(std::abs(second.get_arc().back().radius) == Approx(double(unit)));
}

TEST_CASE("Closed extrusion splitting keeps both sides of the source seam",
          "[plugins][extrusion][split-by-areas]")
{
    const coord_t unit = scale_i(1.);
    ExtrusionPath leaf = straight_path({ Point(0, 0), Point(10 * unit, 0),
                                         Point(10 * unit, 10 * unit), Point(0, 10 * unit),
                                         Point(0, 0) });
    const coord_t split_x = 5 * unit;
    const std::vector<ExPolygons> areas = {
        { rectangle(-unit, -unit, split_x, 11 * unit) },
        { rectangle(split_x, -unit, 11 * unit, 11 * unit) }
    };
    const std::vector<const expolygon_collection_handle *> handles = area_handles(areas);
    CallbackCapture capture;
    capture.handles.reserve(3);
    capture.area_indices.reserve(3);

    REQUIRE(extrusion_split_leaf_by_areas(
                entity_handle(leaf), handles.data(), uint32_t(handles.size()), SCALED_EPSILON,
                collect_fragment, &capture) == RAW_EXTRUSION_SPLIT_STATUS_SUCCESS);

    // The first and last left-side fragments meet at the original seam, but
    // they are intentionally not merged across that seam.
    REQUIRE(leaf.child_count() == 3);
    CAPTURE(leaf.can_sort(), leaf.is_continuous(),
            leaf.child(0).last_point().x(), leaf.child(0).last_point().y(),
            leaf.child(1).first_point().x(), leaf.child(1).first_point().y(),
            leaf.child(1).last_point().x(), leaf.child(1).last_point().y(),
            leaf.child(2).first_point().x(), leaf.child(2).first_point().y());
    CHECK(capture.area_indices == std::vector<uint32_t>{ 0, 1, 0 });
    CHECK(leaf.child(0).first_point() == Point(0, 0));
    CHECK(leaf.child(2).last_point() == Point(0, 0));
    CHECK(leaf.is_continuous());
    CHECK(leaf.is_loop());
}

TEST_CASE("Invalid extrusion split inputs leave the leaf unchanged",
          "[plugins][extrusion][split-by-areas]")
{
    const coord_t unit = scale_i(1.);
    ExtrusionPath leaf = straight_path({ Point(0, 0), Point(10 * unit, 0) });
    const Points original_points = leaf.polyline_ref().to_polyline().points;

    const expolygon_collection_handle *null_area = nullptr;
    CHECK(extrusion_split_leaf_by_areas(
              entity_handle(leaf), &null_area, 1, SCALED_EPSILON, nullptr, nullptr) ==
          RAW_EXTRUSION_SPLIT_STATUS_INVALID_ARGUMENT);
    CHECK(leaf.has_polyline());
    CHECK(leaf.polyline_ref().to_polyline().points == original_points);

    const ExtrusionAttributes attributes(
        ExtrusionRole::Perimeter,
        ExtrusionFlow(0.08, 0.4f, 0.2f));
    ExtrusionPath empty_leaf(attributes, true);
    const ExPolygons valid_areas = { rectangle(-unit, -unit, 11 * unit, unit) };
    const expolygon_collection_handle *valid_handle = area_handle(valid_areas);
    CHECK(extrusion_split_leaf_by_areas(
              entity_handle(empty_leaf), &valid_handle, 1, SCALED_EPSILON, nullptr, nullptr) ==
          RAW_EXTRUSION_SPLIT_STATUS_INVALID_GEOMETRY);
    CHECK(empty_leaf.has_polyline());
    CHECK(empty_leaf.polyline_ref().empty());

    ExtrusionEntity parent(true);
    parent.append_child(std::make_unique<ExtrusionPath>(
        straight_path({ Point(0, 0), Point(10 * unit, 0) })));
    CHECK(extrusion_split_leaf_by_areas(
              entity_handle(parent), &valid_handle, 1, SCALED_EPSILON, nullptr, nullptr) ==
          RAW_EXTRUSION_SPLIT_STATUS_NOT_A_LEAF);
    CHECK(parent.child_count() == 1);

    // Clockwise contours are individually invalid for an ExPolygon area.
    ExPolygon invalid = rectangle(-unit, -unit, 11 * unit, unit);
    invalid.contour.reverse();
    const ExPolygons invalid_areas = { invalid };
    const expolygon_collection_handle *invalid_handle = area_handle(invalid_areas);
    CHECK(extrusion_split_leaf_by_areas(
              entity_handle(leaf), &invalid_handle, 1, SCALED_EPSILON, nullptr, nullptr) ==
          RAW_EXTRUSION_SPLIT_STATUS_INVALID_GEOMETRY);
    CHECK(leaf.has_polyline());
    CHECK(leaf.polyline_ref().to_polyline().points == original_points);

    const slic3r_api::ExPolygonCollection invalid_view(invalid_handle);
    CHECK_THROWS_AS(
        slic3r_api::MutableExtrusionEntity(entity_handle(leaf)).split_leaf_by_areas({ invalid_view }),
        std::invalid_argument);
    CHECK(leaf.has_polyline());
    CHECK(leaf.polyline_ref().to_polyline().points == original_points);
}

TEST_CASE("Extrusion leaves split at lengths keep order, properties and volume",
          "[plugins][extrusion][split-at-lengths]")
{
    const coord_t unit = scale_i(1.);
    ExtrusionPath leaf = straight_path({ Point(0, 0), Point(30 * unit, 0) });
    leaf.add_property(ExtrusionPropertySpeed(42.f, 900.f));
    leaf.add_property(ExtrusionPropertyFan(70.f, FAN_REQUEST_MODE_OVERRIDE));
    const double source_volume = leaf.length() * leaf.mm3_per_mm();
    const distf_t lengths[] = { distf_t(10 * unit), distf_t(25 * unit) };
    CallbackCapture capture;
    capture.handles.reserve(3);
    capture.area_indices.reserve(3);

    REQUIRE(extrusion_split_leaf_at_lengths(entity_handle(leaf), lengths, 2, collect_fragment, &capture) ==
            RAW_EXTRUSION_SPLIT_STATUS_SUCCESS);
    REQUIRE(leaf.child_count() == 3);
    CHECK_FALSE(leaf.can_sort());
    CHECK(leaf.is_continuous());
    CHECK(capture.area_indices == std::vector<uint32_t>{ 0, 1, 2 });
    const double expected_lengths[] = { 10. * unit, 15. * unit, 5. * unit };
    double volume = 0.;
    for (size_t child_idx = 0; child_idx < 3; ++child_idx) {
        const ExtrusionEntity &child = leaf.child(child_idx);
        CHECK(capture.handles[child_idx] == entity_handle(const_cast<ExtrusionEntity &>(child)));
        CHECK(child.length() == Approx(expected_lengths[child_idx]));
        CHECK(child.can_reverse());
        REQUIRE(child.get_property<ExtrusionPropertySpeed>() != nullptr);
        CHECK(child.get_property<ExtrusionPropertySpeed>()->speed_mm_per_s == 42.f);
        REQUIRE(child.get_property<ExtrusionPropertyFan>() != nullptr);
        CHECK(child.get_property<ExtrusionPropertyFan>()->percent == 70.f);
        CHECK(child.get_property<ExtrusionPropertyFan>()->mode == FAN_REQUEST_MODE_OVERRIDE);
        REQUIRE(child.get_property<ExtrusionAttributes>() != nullptr);
        volume += child.length() * child.get_property<ExtrusionAttributes>()->mm3_per_mm;
    }
    CHECK(volume == Approx(source_volume));
    CHECK(leaf.child(0).first_point() == Point(0, 0));
    CHECK(leaf.child(0).last_point() == Point(10 * unit, 0));
    CHECK(leaf.child(1).last_point() == Point(25 * unit, 0));
    CHECK(leaf.child(2).last_point() == Point(30 * unit, 0));
}

TEST_CASE("Arc and Z geometry survive splitting at lengths",
          "[plugins][extrusion][split-at-lengths]")
{
    const coord_t unit = scale_i(1.);
    ArcPolyline source;
    source.append(Point(unit, 0));
    source.append(Geometry::ArcWelder::Segment(
        Point(0, unit), float(unit), Geometry::ArcWelder::Orientation::CCW));
    source.set_z_offset(0, 100);
    source.set_z_offset(1, 900);
    const ExtrusionAttributes attributes(ExtrusionRole::Perimeter, ExtrusionFlow(0.08, 0.4f, 0.2f));
    ExtrusionPath leaf(std::move(source), attributes, nullptr, true);
    const distf_t half = leaf.length() / 2.;

    const std::vector<slic3r_api::MutableExtrusionEntity> pieces =
        slic3r_api::MutableExtrusionEntity(entity_handle(leaf)).split_leaf_at_lengths({ half });

    REQUIRE(pieces.size() == 2);
    REQUIRE(leaf.child_count() == 2);
    const ArcPolyline &first = leaf.child(0).polyline_ref();
    const ArcPolyline &second = leaf.child(1).polyline_ref();
    CHECK(first.has_arc());
    CHECK(second.has_arc());
    CHECK(first.length() == Approx(half));
    CHECK(second.length() == Approx(half));
    CHECK(first.back() == second.front());
    // The cut lies on the arc, not on its chord.
    CHECK((first.back() - Point(0, 0)).cast<double>().norm() == Approx(double(unit)).epsilon(1e-4));
    CHECK(first.z_offset(first.size() - 1) == second.z_offset(0));
    CHECK(first.z_offset(first.size() - 1) == Approx(500.).margin(2.));
}

TEST_CASE("Invalid splits at lengths leave the leaf unchanged",
          "[plugins][extrusion][split-at-lengths]")
{
    const coord_t unit = scale_i(1.);
    ExtrusionPath leaf = straight_path({ Point(0, 0), Point(10 * unit, 0) });
    const Points original_points = leaf.polyline_ref().to_polyline().points;
    const auto status = [&](std::vector<distf_t> lengths) {
        return extrusion_split_leaf_at_lengths(entity_handle(leaf), lengths.empty() ? nullptr : lengths.data(),
                                               uint32_t(lengths.size()), nullptr, nullptr);
    };
    CHECK(status({}) == RAW_EXTRUSION_SPLIT_STATUS_INVALID_ARGUMENT);
    CHECK(status({ distf_t(6 * unit), distf_t(4 * unit) }) == RAW_EXTRUSION_SPLIT_STATUS_INVALID_ARGUMENT);
    CHECK(status({ distf_t(SCALED_EPSILON) }) == RAW_EXTRUSION_SPLIT_STATUS_INVALID_ARGUMENT);
    CHECK(status({ distf_t(10 * unit) }) == RAW_EXTRUSION_SPLIT_STATUS_INVALID_ARGUMENT);
    CHECK(status({ std::nan("") }) == RAW_EXTRUSION_SPLIT_STATUS_INVALID_ARGUMENT);
    CHECK(status({ std::numeric_limits<distf_t>::infinity() }) == RAW_EXTRUSION_SPLIT_STATUS_INVALID_ARGUMENT);
    CHECK(status({ distf_t(5 * unit), distf_t(5 * unit) }) == RAW_EXTRUSION_SPLIT_STATUS_INVALID_ARGUMENT);
    CHECK(status({ -distf_t(unit) }) == RAW_EXTRUSION_SPLIT_STATUS_INVALID_ARGUMENT);
    CHECK(status({ distf_t(10 * unit - SCALED_EPSILON) }) == RAW_EXTRUSION_SPLIT_STATUS_INVALID_ARGUMENT);
    CHECK(leaf.has_polyline());
    CHECK(leaf.polyline_ref().to_polyline().points == original_points);

    // A raw E command or an event would be repeated by every piece.
    ExtrusionPath raw_e = straight_path({ Point(0, 0), Point(10 * unit, 0) });
    raw_e.add_property(ExtrusionPropertySpecialCommand(ExtrusionPropertySpecialCommand::Code::EXTRUSION, 1.));
    const distf_t middle = distf_t(5 * unit);
    CHECK(extrusion_split_leaf_at_lengths(entity_handle(raw_e), &middle, 1, nullptr, nullptr) ==
          RAW_EXTRUSION_SPLIT_STATUS_ONE_SHOT_EVENT);
    CHECK(raw_e.has_polyline());
    ExtrusionPath heated = straight_path({ Point(0, 0), Point(10 * unit, 0) });
    heated.add_property(ExtrusionPropertyHeater(C_EXTRUSION_HEATER_TOOL, EXTRUDER_ID_UNKNOWN, 200, false));
    CHECK_THROWS_AS(slic3r_api::MutableExtrusionEntity(entity_handle(heated)).split_leaf_at_lengths({ middle }),
                    std::invalid_argument);
    CHECK(heated.has_polyline());

    ExtrusionEntity parent(true);
    parent.append_child(std::make_unique<ExtrusionPath>(straight_path({ Point(0, 0), Point(10 * unit, 0) })));
    CHECK(extrusion_split_leaf_at_lengths(entity_handle(parent), &middle, 1, nullptr, nullptr) ==
          RAW_EXTRUSION_SPLIT_STATUS_NOT_A_LEAF);
    CHECK(parent.child_count() == 1);
}

TEST_CASE("Length splitting rejects every one-shot property before publishing children",
          "[plugins][extrusion][split-at-lengths]")
{
    const extrusion_property_type type = GENERATE(EXTRUSION_PROPERTY_TYPE_CUSTOM_GCODE,
        EXTRUSION_PROPERTY_TYPE_SPECIAL_COMMAND, EXTRUSION_PROPERTY_TYPE_HEATER, EXTRUSION_PROPERTY_TYPE_EXTRUSION_AXIS);
    ExtrusionPath leaf = straight_path({Point(0, 0), Point(scale_i(10.), 0)});
    switch (type) {
    case EXTRUSION_PROPERTY_TYPE_CUSTOM_GCODE:
        leaf.get_or_add_property<ExtrusionPropertyCustomGcode>();
        break;
    case EXTRUSION_PROPERTY_TYPE_SPECIAL_COMMAND:
        leaf.add_property(ExtrusionPropertySpecialCommand(ExtrusionPropertySpecialCommand::Code::DEPOSIT, 0.5));
        break;
    case EXTRUSION_PROPERTY_TYPE_HEATER:
        leaf.get_or_add_property<ExtrusionPropertyHeater>();
        break;
    case EXTRUSION_PROPERTY_TYPE_EXTRUSION_AXIS:
        leaf.get_or_add_property<ExtrusionPropertyExtrusionAxis>();
        break;
    default:
        FAIL("Unlisted one-shot property");
    }
    const distf_t middle = scale_i(5.);
    CallbackCapture capture;
    CHECK(extrusion_split_leaf_at_lengths(entity_handle(leaf), &middle, 1, collect_fragment, &capture) ==
          RAW_EXTRUSION_SPLIT_STATUS_ONE_SHOT_EVENT);
    CHECK(leaf.child_count() == 0);
    CHECK(leaf.has_polyline());
    CHECK(leaf.first_point() == Point(0, 0));
    CHECK(leaf.last_point() == Point(scale_i(10.), 0));
    CHECK(extrusion_property_has(entity_handle(leaf), type) != 0);
    CHECK(capture.handles.empty());
}

TEST_CASE("Length splitting diagnoses empty and degenerate geometry without mutation",
          "[plugins][extrusion][split-at-lengths]")
{
    const distf_t middle = scale_i(5.);
    ExtrusionPath leaf = straight_path({Point(0, 0), Point(scale_i(10.), 0)});
    leaf.polyline() = ArcPolyline();
    CHECK(extrusion_split_leaf_at_lengths(entity_handle(leaf), &middle, 1, nullptr, nullptr) ==
          RAW_EXTRUSION_SPLIT_STATUS_INVALID_GEOMETRY);
    CHECK(leaf.child_count() == 0);
    leaf.polyline().append(Point(0, 0));
    CHECK(extrusion_split_leaf_at_lengths(entity_handle(leaf), &middle, 1, nullptr, nullptr) ==
          RAW_EXTRUSION_SPLIT_STATUS_INVALID_GEOMETRY);
    CHECK(leaf.polyline().size() == 1);
    leaf.polyline().append(Point(1, 0));
    CHECK(extrusion_split_leaf_at_lengths(entity_handle(leaf), &middle, 1, nullptr, nullptr) ==
          RAW_EXTRUSION_SPLIT_STATUS_INVALID_GEOMETRY);
    CHECK(leaf.polyline().size() == 2);
    CHECK(extrusion_split_leaf_at_lengths(nullptr, &middle, 1, nullptr, nullptr) ==
          RAW_EXTRUSION_SPLIT_STATUS_INVALID_ARGUMENT);
}
