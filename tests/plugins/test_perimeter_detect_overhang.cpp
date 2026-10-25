///|/ Copyright (c) SuperSlicer 2026 Remi Durand.
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
///|/

#include <catch2/catch.hpp>

#include "perimeter_test_helpers.hpp"

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExtrusionProperty.hpp"

#include <algorithm>
#include <limits>

// These tests generate perimeter trees from explicit layer polygons, run overhang
// classification against a lower slice, and inspect path geometry and properties.

namespace {
using namespace Slic3r;
using namespace Slic3r::Test::PerimeterPluginTests;

struct DetectedOverhangStats
{
    size_t perimeter_leaves = 0;
    size_t overhang_properties = 0;
    size_t full_flow = 0;
    size_t full_speed = 0;
    size_t dynamic_flow = 0;
    size_t dynamic_speed = 0;
    size_t bridge_role = 0;
    size_t dynamic_without_bridge = 0;
    double max_distance = 0.;
};

DynamicPrintConfig detect_overhang_config(std::initializer_list<std::pair<std::string, std::string>> overrides)
{
    DynamicPrintConfig config = perimeter_config({
        {"overhangs", "1"},
        {"overhangs_flow_ratio", "100"},
        {"overhangs_type", "nozzle"},
        {"perimeters", "1"}
    });
    for (const std::pair<std::string, std::string> &entry : overrides)
        config.set_deserialize_strict(entry.first, entry.second);
    return config;
}

ExPolygon detected_overhang_target()
{
    return rectangle_expolygon(-10., -5., 10., 5.);
}

ExPolygon half_width_lower_support()
{
    return rectangle_expolygon(-10., -5., 5., 5.);
}

ExPolygon near_full_lower_support()
{
    return rectangle_expolygon(-10., -5., 9.5, 5.);
}

ExPolygon triangle_lower_support()
{
    return ExPolygon(Polygon({
        Point(scale_i(-10.), scale_i(-5.)),
        Point(scale_i(11.), scale_i(0.)),
        Point(scale_i(-10.), scale_i(5.))
    }));
}

ExPolygon wide_detect_overhang_target()
{
    return rectangle_expolygon(-12., -8., 12., 8.);
}

ExPolygon wide_triangular_lower_support()
{
    return ExPolygon(Polygon({
        Point(scale_i(-12.), scale_i(-8.)),
        Point(scale_i(12.), scale_i(-2.)),
        Point(scale_i(-12.), scale_i(8.))
    }));
}

double shortest_leaf_segment_length(const ExtrusionEntity &entity)
{
    if (entity.is_nop())
        return std::numeric_limits<double>::max();
    if (entity.is_leaf() && entity.has_polyline()) {
        Points points;
        entity.collect_points(points);
        double shortest = std::numeric_limits<double>::max();
        for (size_t point_idx = 1; point_idx < points.size(); ++point_idx)
            shortest = std::min(
                shortest,
                (points[point_idx] - points[point_idx - 1]).cast<double>().norm());
        return shortest;
    }

    double shortest = std::numeric_limits<double>::max();
    for (size_t child_idx = 0; child_idx < entity.child_count(); ++child_idx)
        shortest = std::min(shortest, shortest_leaf_segment_length(entity.child(child_idx)));
    return shortest;
}

void collect_detected_overhang_stats(const ExtrusionEntity &entity, DetectedOverhangStats &out)
{
    if (entity.is_nop())
        return;

    if (entity.is_leaf()) {
        const ExtrusionAttributes *attributes = entity.get_property<ExtrusionAttributes>();
        if (attributes == nullptr || !attributes->extrusion_role().is_perimeter())
            return;

        ++out.perimeter_leaves;
        if (attributes->extrusion_role().is_bridge())
            ++out.bridge_role;

        const ExtrusionPropertyOverhang *overhang = entity.get_property<ExtrusionPropertyOverhang>();
        if (overhang == nullptr)
            return;

        ++out.overhang_properties;
        if (overhang->has_full_overhangs_flow)
            ++out.full_flow;
        if (overhang->has_full_overhangs_speed)
            ++out.full_speed;
        if (overhang->has_dynamic_overhangs_flow)
            ++out.dynamic_flow;
        if (overhang->has_dynamic_overhangs_speed)
            ++out.dynamic_speed;
        if ((overhang->has_dynamic_overhangs_flow || overhang->has_dynamic_overhangs_speed) &&
            !attributes->extrusion_role().is_bridge())
            ++out.dynamic_without_bridge;

        out.max_distance = std::max(out.max_distance, double(overhang->end_distance_from_prev_layer));
        return;
    }

    for (size_t child_idx = 0; child_idx < entity.child_count(); ++child_idx)
        collect_detected_overhang_stats(entity.child(child_idx), out);
}

// Count the entities flagged as perimeter loops, and the ones among them that
// no longer form a closed continuous sequence.
void count_flagged_loops(const ExtrusionEntity &entity, size_t &flagged, size_t &broken)
{
    if (const ExtrusionPropertyLoopRole *loop_role = entity.get_property<ExtrusionPropertyLoopRole>();
        loop_role != nullptr && (loop_role->perimeter_flags & C_EXTRUSION_PERIMETER_FLAG_LOOP) != 0) {
        ++flagged;
        if (!entity.is_loop())
            ++broken;
    }
    for (size_t child_idx = 0; child_idx < entity.child_count(); ++child_idx)
        count_flagged_loops(entity.child(child_idx), flagged, broken);
}

DetectedOverhangStats detected_overhang_stats(const PerimeterRunCapture &capture)
{
    DetectedOverhangStats out;
    collect_detected_overhang_stats(capture.external_perimeters, out);
    return out;
}

PerimeterRunCapture run_detect_overhang_case(const DynamicPrintConfig &config,
                                             const ExPolygon &lower_support)
{
    return run_perimeter_and_post_case_with_lower_area(
        config, {SIMPLE_PERIMETER_GENERATOR}, {DETECT_OVERHANG},
        detected_overhang_target(), lower_support, 1);
}

} // namespace

TEST_CASE("Detect overhang marks near unsupported spans as dynamic only", "[plugins][perimeter][detect-overhang]")
{
    // The current layer is a rectangle, and the layer below is only slightly
    // shorter on the right. That creates a real unsupported perimeter span, but
    // the configured thresholds are deliberately much larger than the gap. The
    // plugin should keep the span as a normal perimeter role and attach dynamic
    // overhang metadata for later speed/flow interpolation.
    const DynamicPrintConfig config = detect_overhang_config({
        {"overhangs_width", "5"},
        {"overhangs_width_speed", "5"}
    });
    const PerimeterRunCapture capture = run_detect_overhang_case(config, near_full_lower_support());
    const DetectedOverhangStats stats = detected_overhang_stats(capture);

    REQUIRE(stats.perimeter_leaves > 0);
    REQUIRE(stats.overhang_properties > 0);
    CHECK(stats.full_flow == 0);
    CHECK(stats.full_speed == 0);
    CHECK(stats.dynamic_flow > 0);
    CHECK(stats.dynamic_speed > 0);
    CHECK(stats.bridge_role == 0);
    CHECK(stats.dynamic_without_bridge == stats.overhang_properties);
}

TEST_CASE("Detect overhang marks large unsupported spans as full flow and speed", "[plugins][perimeter][detect-overhang]")
{
    // The lower layer supports only the left half of the rectangle. The right
    // perimeter is far beyond both thresholds, so it should become a full
    // overhang for flow and speed. Full overhang paths are tagged as bridge
    // perimeter so downstream code can use bridge-style handling.
    const DynamicPrintConfig config = detect_overhang_config({
        {"overhangs_width", "0.2"},
        {"overhangs_width_speed", "0.2"}
    });
    const PerimeterRunCapture capture = run_detect_overhang_case(config, half_width_lower_support());
    const DetectedOverhangStats stats = detected_overhang_stats(capture);

    REQUIRE(stats.overhang_properties > 0);
    CHECK(stats.full_flow > 0);
    CHECK(stats.full_speed > 0);
    CHECK(stats.full_flow > stats.dynamic_flow);
    CHECK(stats.full_speed > stats.dynamic_speed);
    CHECK(stats.bridge_role >= std::max(stats.full_flow, stats.full_speed));
    CHECK(stats.max_distance > 0.2);
}

TEST_CASE("Detect overhang keeps split perimeter loops closed", "[plugins][perimeter][detect-overhang]")
{
    // Splitting a loop leaf into supported and overhang fragments must keep the
    // fragments an ordered, non-sortable sequence. Otherwise the leaf still
    // carries the perimeter LOOP flag while is_loop() is false, and the
    // STEP_ORDERING pre-validation rejects the print.
    const DynamicPrintConfig config = detect_overhang_config({
        {"overhangs_width", "0.2"},
        {"overhangs_width_speed", "0.2"}
    });
    const PerimeterRunCapture capture = run_detect_overhang_case(config, half_width_lower_support());
    REQUIRE(detected_overhang_stats(capture).overhang_properties > 0);

    size_t flagged = 0;
    size_t broken = 0;
    count_flagged_loops(capture.external_perimeters, flagged, broken);
    REQUIRE(flagged > 0);
    CHECK(broken == 0);
}

TEST_CASE("Detect overhang merges tiny supported crossings into the loop", "[plugins][perimeter][detect-overhang]")
{
    // The lower layer supports the left half, plus a bump that crosses the top
    // perimeter of the unsupported half over about 0.05 mm. That supported
    // fragment is shorter than the minimum split length, so it must be merged
    // into a neighbor rather than dropped, or the split loop is left open.
    const DynamicPrintConfig config = detect_overhang_config({
        {"overhangs_width", "0.2"},
        {"overhangs_width_speed", "0.2"}
    });
    const double width = unscaled(run_detect_overhang_case(config, half_width_lower_support()).external_perimeter_width);
    REQUIRE(width > 0.);
    // The support area is the lower slice shrunk by half the perimeter width,
    // so the bump keeps 0.05 mm of its width. A strip below the top perimeter
    // connects the bump to the supported half.
    const double bump_min_x = 7.;
    const double bump_max_x = bump_min_x + width + 0.05;
    const ExPolygon lower_support(Polygon({
        Point(scale_i(-10.), scale_i(-5.)),
        Point(scale_i(5.), scale_i(-5.)),
        Point(scale_i(5.), scale_i(3.)),
        Point(scale_i(bump_max_x), scale_i(3.)),
        Point(scale_i(bump_max_x), scale_i(6.)),
        Point(scale_i(bump_min_x), scale_i(6.)),
        Point(scale_i(bump_min_x), scale_i(4.)),
        Point(scale_i(5.), scale_i(4.)),
        Point(scale_i(5.), scale_i(5.)),
        Point(scale_i(-10.), scale_i(5.))
    }));
    const PerimeterRunCapture capture = run_detect_overhang_case(config, lower_support);
    REQUIRE(detected_overhang_stats(capture).overhang_properties > 0);

    size_t flagged = 0;
    size_t broken = 0;
    count_flagged_loops(capture.external_perimeters, flagged, broken);
    REQUIRE(flagged > 0);
    CHECK(broken == 0);
    // Nothing of the 20 x 10 mm contour is lost.
    CHECK(extrusion_length(capture.external_perimeters) ==
          Approx(extrusion_length(run_detect_overhang_case(config, half_width_lower_support()).external_perimeters))
              .epsilon(0.001));
}

// Generate a triangular perimeter partly supported by a lower rectangle, then
// inspect owned trees without mutation; every LOOP-marked entity must be closed.
TEST_CASE("Detect overhang keeps diagonal split loops closed", "[plugins][perimeter][detect-overhang]")
{
    const DynamicPrintConfig config = detect_overhang_config({
        {"overhangs_width", "0.2"},
        {"overhangs_width_speed", "0.2"}
    });
    const ExPolygon target(Polygon({
        Point(scale_i(-2.), scale_i(-1.)),
        Point(scale_i(2.), scale_i(-0.64)),
        Point(scale_i(-2.), scale_i(1.))
    }));
    const ExPolygon lower_support = rectangle_expolygon(-3., -2., 0., 2.);
    const PerimeterRunCapture baseline = run_perimeter_and_post_case_with_lower_area(
        config, {SIMPLE_PERIMETER_GENERATOR}, {}, target, lower_support, 1);
    size_t baseline_flagged = 0;
    size_t baseline_broken = 0;
    count_flagged_loops(baseline.external_perimeters, baseline_flagged, baseline_broken);
    REQUIRE(baseline_flagged > 0);
    REQUIRE(baseline_broken == 0);

    // The unsupported diagonal ends at the loop's starting vertex. Sampling
    // must retain that exact endpoint, not truncate an interpolated coordinate.
    const PerimeterRunCapture capture = run_perimeter_and_post_case_with_lower_area(
        config, {SIMPLE_PERIMETER_GENERATOR}, {DETECT_OVERHANG}, target, lower_support, 1);
    REQUIRE(detected_overhang_stats(capture).overhang_properties > 0);
    size_t flagged = 0;
    size_t broken = 0;
    count_flagged_loops(capture.external_perimeters, flagged, broken);
    REQUIRE(flagged > 0);
    CHECK(broken == 0);

    // Match the source vertices as an exact subsequence, including the repeated
    // closing vertex; split fragments may add samples and duplicate junctions.
    Points source_points;
    Points result_points;
    baseline.external_perimeters.collect_points(source_points);
    capture.external_perimeters.collect_points(result_points);
    size_t preserved = 0;
    for (const Point &point : result_points)
        if (preserved < source_points.size() && point == source_points[preserved])
            ++preserved;
    CHECK(preserved == source_points.size());
    CHECK(total_polyline_points(capture.external_perimeters) > total_polyline_points(baseline.external_perimeters));
}

// Generate a stepped loop whose starting point lies over lower-layer support, then
// inspect the owned result without mutation for LOOP entities that lost closure.
TEST_CASE("Detect overhang keeps supported seam fragments oriented", "[plugins][perimeter][detect-overhang]")
{
    const DynamicPrintConfig config = detect_overhang_config({
        {"overhangs_width", "0.2"},
        {"overhangs_width_speed", "0.2"}
    });
    const ExPolygon source_area(Polygon({
        Point(-9517193, -2282851), Point(-5774973, -2282851),
        Point(-5774973, -2482851), Point(-5380883, -2482851),
        Point(-5380883, -2282851), Point(-1638663, -2282851),
        Point(-1638663, -2482851), Point(-379649, -2482851),
        Point(-379649, -2282851), Point(1661672, -2282851),
        Point(1661672, -2482851), Point(3532853, -2482851),
        Point(3532853, -2082851), Point(13345765, -2082851),
        Point(13345765, 2482853), Point(-14480354, 2482853),
        Point(-14480354, -2082851), Point(-11032852, -2082851),
        Point(-11032852, -2482851), Point(-9517193, -2482851)
    }));
    const double width = unscaled(run_detect_overhang_case(config, half_width_lower_support()).external_perimeter_width);
    // Expand the centerline by half the extrusion width so the generator's
    // inward offset preserves its short bottom steps. Only the right edge
    // extends beyond the lower layer's supported centerline area.
    const ExPolygon target = offset_ex(source_area, scale_i(width * 0.5)).front();
    const ExPolygon lower_support = rectangle_expolygon(-20., -5., 13.23042 + width * 0.5, 5.);
    const PerimeterRunCapture baseline = run_perimeter_and_post_case_with_lower_area(
        config, {SIMPLE_PERIMETER_GENERATOR}, {}, target, lower_support, 1);
    size_t flagged = 0;
    size_t broken = 0;
    count_flagged_loops(baseline.external_perimeters, flagged, broken);
    REQUIRE(flagged > 0);
    REQUIRE(broken == 0);

    const PerimeterRunCapture capture = run_perimeter_and_post_case_with_lower_area(
        config, {SIMPLE_PERIMETER_GENERATOR}, {DETECT_OVERHANG}, target, lower_support, 1);
    REQUIRE(detected_overhang_stats(capture).overhang_properties > 0);
    flagged = 0;
    broken = 0;
    count_flagged_loops(capture.external_perimeters, flagged, broken);
    REQUIRE(flagged > 0);
    CHECK(broken == 0);
}

TEST_CASE("Detect overhang separates flow and speed thresholds", "[plugins][perimeter][detect-overhang]")
{
    // Flow and speed have independent thresholds. With a half-supported
    // rectangle, the unsupported distance is large enough to cross one
    // threshold while remaining below the other when we invert the two values.
    SECTION("flow threshold is crossed before speed threshold") {
        const DynamicPrintConfig config = detect_overhang_config({
            {"overhangs_width", "0.2"},
            {"overhangs_width_speed", "5"}
        });
        const PerimeterRunCapture capture = run_detect_overhang_case(config, half_width_lower_support());
        const DetectedOverhangStats stats = detected_overhang_stats(capture);

        REQUIRE(stats.overhang_properties > 0);
        CHECK(stats.full_flow > 0);
        CHECK(stats.full_speed == 0);
        CHECK(stats.dynamic_speed > 0);
        CHECK(stats.bridge_role > 0);
    }

    SECTION("speed threshold is crossed before flow threshold") {
        const DynamicPrintConfig config = detect_overhang_config({
            {"overhangs_width", "5"},
            {"overhangs_width_speed", "0.2"}
        });
        const PerimeterRunCapture capture = run_detect_overhang_case(config, half_width_lower_support());
        const DetectedOverhangStats stats = detected_overhang_stats(capture);

        REQUIRE(stats.overhang_properties > 0);
        CHECK(stats.full_flow == 0);
        CHECK(stats.full_speed > 0);
        CHECK(stats.dynamic_flow > 0);
        CHECK(stats.bridge_role > 0);
    }
}

TEST_CASE("Detect overhang samples varying support distance along one contour", "[plugins][perimeter][detect-overhang]")
{
    // A triangular lower island creates a support boundary that crosses the
    // rectangular current layer. Different points on the same right-side
    // perimeter are therefore at different distances from support. This checks
    // that the plugin splits and annotates more than one overhang fragment
    // instead of applying one coarse value to the whole contour.
    const DynamicPrintConfig config = detect_overhang_config({
        {"overhangs_width", "5"},
        {"overhangs_width_speed", "5"}
    });
    const PerimeterRunCapture capture = run_detect_overhang_case(config, triangle_lower_support());
    const DetectedOverhangStats stats = detected_overhang_stats(capture);

    REQUIRE(stats.overhang_properties > 1);
    CHECK(stats.dynamic_flow > 1);
    CHECK(stats.dynamic_speed > 1);
    CHECK(stats.full_flow == 0);
    CHECK(stats.full_speed == 0);
    CHECK(stats.max_distance > 0.);
}

TEST_CASE("Detect overhang preserves disabled overhang regions", "[plugins][perimeter][detect-overhang]")
{
    // The unsupported side is covered by a region where overhang detection is
    // disabled. RegionSettings still clips the perimeter by region, so the
    // important contract is that disabled clips are copied back unchanged
    // rather than silently dropping that part of the path.
    const DynamicPrintConfig config = detect_overhang_config({
        {"overhangs_width", "0.2"},
        {"overhangs_width_speed", "0.2"}
    });
    const ExPolygon disabled_right_half = rectangle_expolygon(0., -5., 10., 5.);
    const PerimeterRunCapture capture = run_perimeter_and_post_case_with_lower_area(
        config, {SIMPLE_PERIMETER_GENERATOR}, {DETECT_OVERHANG},
        detected_overhang_target(), half_width_lower_support(), 1,
        {{"overhangs", "0"}}, &disabled_right_half);
    const DetectedOverhangStats stats = detected_overhang_stats(capture);

    REQUIRE(stats.perimeter_leaves > 0);
    CHECK(stats.overhang_properties == 0);
    CHECK(stats.bridge_role == 0);
    CHECK(extrusion_length(capture.external_perimeters) > 0.);
}

TEST_CASE("Detect overhang skips work when flow and speed outputs are disabled", "[plugins][perimeter][detect-overhang]")
{
    // This is the nullable-setting counterpart of the global overhang switch.
    // The current layer has a large unsupported span, but both consumers of the
    // detection result are disabled. The plugin should therefore behave like a
    // no-op: no overhang metadata, no bridge role, and no extra path splitting.
    const DynamicPrintConfig config = detect_overhang_config({
        {"overhangs_flow_ratio", "!100"},
        {"overhangs_width_speed", "!0.2"}
    });
    const PerimeterRunCapture baseline = run_perimeter_and_post_case(
        config, {SIMPLE_PERIMETER_GENERATOR}, {},
        detected_overhang_target(), 1);
    const PerimeterRunCapture capture = run_detect_overhang_case(config, half_width_lower_support());
    const DetectedOverhangStats stats = detected_overhang_stats(capture);

    CHECK(stats.overhang_properties == 0);
    CHECK(stats.bridge_role == 0);
    CHECK(external_perimeter_count(capture) == external_perimeter_count(baseline));
    CHECK(total_polyline_points(capture.external_perimeters) == total_polyline_points(baseline.external_perimeters));
    CHECK(extrusion_length(capture.external_perimeters) == Approx(extrusion_length(baseline.external_perimeters)));
}

TEST_CASE("Detect overhang does not create tiny split crumbs", "[plugins][perimeter][detect-overhang]")
{
    // Extra-overhang perimeter generation already filters the tiny segments
    // produced by geometric clipping. This regression runs DetectOverhang after
    // that clean output, because DetectOverhang performs its own support split
    // and must not reintroduce near-zero fragments that later fail G-code checks.
    DynamicPrintConfig config = detect_overhang_config({
        {"bridged_infill_margin", "200%"},
        {"bridge_precision", "10%"},
        {"extra_perimeters_on_overhangs", "1"},
        {"infill_overlap", "0"},
        {"overhangs_extrusion_spacing", "5"},
        {"overhangs_width", "5"},
        {"overhangs_width_speed", "5"},
        {"perimeters", "2"}
    });

    const ExPolygon target = wide_detect_overhang_target();
    const ExPolygon lower_support = wide_triangular_lower_support();
    const PerimeterRunCapture before_detection =
        run_perimeter_and_post_case_with_lower_area(
            config, {SIMPLE_PERIMETER_GENERATOR}, {EXTRA_PERIMETERS_ON_OVERHANGS},
            target, lower_support, 1);
    const PerimeterRunCapture after_detection =
        run_perimeter_and_post_case_with_lower_area(
            config, {SIMPLE_PERIMETER_GENERATOR}, {EXTRA_PERIMETERS_ON_OVERHANGS, DETECT_OVERHANG},
            target, lower_support, 1);

    // Any segment much shorter than the path width is only a classifier crumb:
    // it does not make a useful extrusion and can later show up as an almost
    // zero-length G-code move. The exact geometry here used to create a
    // roughly 0.06 mm piece after DetectOverhang even though the input from the
    // extra-overhang pass was clean.
    const double min_printable_segment = double(before_detection.external_perimeter_width) / 5.0;
    REQUIRE(shortest_leaf_segment_length(before_detection.external_perimeters) >= min_printable_segment);
    CHECK(shortest_leaf_segment_length(after_detection.external_perimeters) >= min_printable_segment);
}
