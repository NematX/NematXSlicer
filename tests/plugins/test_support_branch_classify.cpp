#include <cmath>
#include <vector>

#include "support_grid_test_helpers.hpp"
#include "libslic3r/Plugins/Support/Toolpath/BranchIslandClass.hpp"

/* Classification of the fused branch islands: the thin fraction and the compactness that select the sheath, the centreline
   or the blob (BranchIslandClass.hpp), and the width settings of the centreline. The shapes are small host polygons, measured
   with an extrusion width of 0.4 mm, so the opening radius is 0.3 mm and the thin parts are narrower than 0.6 mm. The cases
   that print islands through the support toolpath step are in test_support_toolpath_branch.cpp. */

using namespace Slic3r;
using slic3r_api::Support::Toolpath::BranchIslandClass;
using slic3r_api::Support::Toolpath::BranchIslandMeasure;
using slic3r_api::Support::Toolpath::CentrelineWidths;
using slic3r_api::Support::Toolpath::centreline_max_widening;
using slic3r_api::Support::Toolpath::centreline_widths;
using slic3r_api::Support::Toolpath::classify_island;
using slic3r_api::Support::Toolpath::compactness_limit;
using slic3r_api::Support::Toolpath::thin_fraction_limit;

namespace {
// The extrusion width of the shapes, in scaled units.
const coord_t k_width = scale_i(0.4);

// A rectangle with corners (x0, y0) and (x1, y1) in millimeters: four vertices, counter-clockwise, no repeated first point.
ExPolygon rectangle(double x0, double y0, double x1, double y1)
{
    return ExPolygon(Polygon({Point::new_scale(x0, y0), Point::new_scale(x1, y0), Point::new_scale(x1, y1), Point::new_scale(x0, y1)}));
}

// A 64-sided polygon around (x, y) in millimeters, counter-clockwise.
Polygon round_polygon(double radius, double x, double y)
{
    Polygon polygon;
    for (size_t index = 0; index < 64; ++index) {
        const double angle = 2. * PI * double(index) / 64.;
        polygon.points.push_back(Point::new_scale(x + radius * std::cos(angle), y + radius * std::sin(angle)));
    }
    return polygon;
}

ExPolygon disc(double radius) { return ExPolygon(round_polygon(radius, 0., 0.)); }

// A ring between two radii: the disc of the outer radius with a clockwise hole.
ExPolygon ring(double outer, double inner)
{
    ExPolygon result = disc(outer);
    Polygon hole = round_polygon(inner, 0., 0.);
    hole.make_clockwise();
    result.holes.push_back(hole);
    return result;
}

// A 2 x 2 mm block with a 0.4 mm wide tail on its right side, long enough for the tail to hold the given share of the total area.
// The tail is narrower than the opening, so the opening removes all of it and keeps the block: the thin fraction is the share.
ExPolygon block_with_tail(double tail_share)
{
    const double tail_length = 4. * tail_share / (1. - tail_share) / 0.4;
    return ExPolygon(Polygon({Point::new_scale(0., 0.), Point::new_scale(2., 0.), Point::new_scale(2., 0.8), Point::new_scale(2. + tail_length, 0.8),
                              Point::new_scale(2. + tail_length, 1.2), Point::new_scale(2., 1.2), Point::new_scale(2., 2.), Point::new_scale(0., 2.)}));
}

// Classify a host polygon with the extrusion width of the shapes.
BranchIslandMeasure measure(const ExPolygon &shape)
{
    PluginStorage storage;
    return classify_island(reinterpret_cast<storage_handle *>(&storage), slic3r_api::ExPolygon(reinterpret_cast<const expolygon_handle *>(&shape)), k_width);
}
} // namespace

// An island wider than 1.5 widths in every direction has no thin part: the opening keeps the convex corners, so a rectangle
// is its own opening, and the sheath prints.
TEST_CASE("Wide islands have no thin part and keep the sheath", "[plugins][support-toolpath][support-branch-classify]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    for (const ExPolygon &shape : {disc(1.), rectangle(0., 0., 2., 1.)}) {
        const BranchIslandMeasure result = measure(shape);
        CHECK(result.thin_fraction == Approx(0.).margin(0.01));
        CHECK(result.kind == BranchIslandClass::Sheath);
    }
}

// A disc narrower than the opening is all thin and compact (Q is close to 1): it is a blob, even though its sheath could exist.
TEST_CASE("A disc narrower than the opening is a blob", "[plugins][support-toolpath][support-branch-classify]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const BranchIslandMeasure result = measure(disc(0.25));
    CHECK(result.thin_fraction == Approx(1.).margin(0.01));
    CHECK(result.compactness > 0.99);
    CHECK(result.kind == BranchIslandClass::Blob);
}

// A 2 mm block with a thin tail: the island turns thin when the tail holds half of its area. The tail is 0.4 mm wide, so the
// opening removes all of it and, with miter joins, keeps the block untouched: the thin fraction is the tail share, one point
// of the limit on each side.
TEST_CASE("The thin fraction selects the sheath below one half and the thin methods from one half", "[plugins][support-toolpath][support-branch-classify]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const BranchIslandMeasure below = measure(block_with_tail(thin_fraction_limit - 0.01));
    CHECK(below.thin_fraction == Approx(thin_fraction_limit - 0.01).margin(0.002));
    CHECK(below.kind == BranchIslandClass::Sheath);
    const BranchIslandMeasure above = measure(block_with_tail(thin_fraction_limit + 0.01));
    CHECK(above.thin_fraction == Approx(thin_fraction_limit + 0.01).margin(0.002));
    CHECK(above.kind == BranchIslandClass::Centreline);
}

// Two thin rectangles 0.5 mm wide on both sides of the 2:1 shape, whose compactness is the limit: the more compact one is a
// blob and the more elongated one a centreline.
TEST_CASE("The compactness selects the centreline below 2:1 and the blob above", "[plugins][support-toolpath][support-branch-classify]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const BranchIslandMeasure compact = measure(rectangle(0., 0., 0.9, 0.5));
    CHECK(compact.thin_fraction == Approx(1.).margin(0.01));
    CHECK(compact.compactness == Approx(PI * 0.9 * 0.5 / (1.4 * 1.4)).epsilon(0.01));
    CHECK(compact.compactness > compactness_limit);
    CHECK(compact.kind == BranchIslandClass::Blob);
    const BranchIslandMeasure elongated = measure(rectangle(0., 0., 1.1, 0.5));
    CHECK(elongated.compactness < compactness_limit);
    CHECK(elongated.kind == BranchIslandClass::Centreline);
}

// The perimeter counts the closing edge of the ring, which the length of the polygon points leaves out: a 0.5 x 1.5 mm
// rectangle has four vertices and the compactness of the closed outline, 0.589. Without the closing edge (the short side, so
// P = 3.5 instead of 4) it would be 0.77 and the island a blob.
TEST_CASE("The compactness counts the closing edge of every ring", "[plugins][support-toolpath][support-branch-classify]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const BranchIslandMeasure result = measure(rectangle(0., 0., 1.5, 0.5));
    CHECK(result.compactness == Approx(PI * 1.5 * 0.5 / 4.).epsilon(0.01));
    CHECK(result.kind == BranchIslandClass::Centreline);
}

// A thin ring and a 4:1 rectangle are elongated: the holes add their boundary to the perimeter.
TEST_CASE("A thin ring and a 4:1 rectangle are centrelines", "[plugins][support-toolpath][support-branch-classify]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const BranchIslandMeasure round = measure(ring(1., 0.75));
    CHECK(round.thin_fraction == Approx(1.).margin(0.02));
    CHECK(round.compactness < 0.2);
    CHECK(round.kind == BranchIslandClass::Centreline);
    const BranchIslandMeasure strip = measure(rectangle(0., 0., 1.6, 0.4));
    CHECK(strip.compactness == Approx(PI * 1.6 * 0.4 / 4.).epsilon(0.01));
    CHECK(strip.kind == BranchIslandClass::Centreline);
}

// The widths for a 0.4 mm nozzle, a 0.2 mm layer and a spacing ratio of 1 (h (1 - pi / 4) = 0.0429 mm), with an XY distance far
// above the widening: the narrowest extrusion width kept is the nozzle over 1.1, the spacing of the widened parts is the nozzle
// less two rounded ends, and the widest is 1.5 widths. A smaller XY distance raises the narrowest width to the nozzle less twice
// the distance.
TEST_CASE("The centreline widths follow the nozzle, the widening limit and the XY distance", "[plugins][support-toolpath][support-branch-classify]")
{
    c_flow flow = {};
    flow.width = scale_i(0.42);
    flow.height = scale_i(0.2);
    flow.nozzle_diameter = scale_i(0.4);
    flow.spacing_ratio = 1.f;
    const double rounded_ends = 0.2 * (1. - 0.25 * PI);
    const double narrowest_kept = 0.4 / (1. + centreline_max_widening);

    CentrelineWidths widths = centreline_widths(flow, scale_i(0.2));
    CHECK(unscaled(widths.min_medial_width) == Approx(narrowest_kept - rounded_ends).margin(1e-5));
    CHECK(unscaled(widths.min_extrusion_width) == Approx(0.4 - 2. * rounded_ends).margin(1e-5));
    CHECK(unscaled(widths.max_medial_width) == Approx(0.63).margin(1e-5));

    // An XY distance of 0.01 mm allows a widening of 0.02 mm: the narrowest extrusion width kept is 0.38 mm.
    widths = centreline_widths(flow, scale_i(0.01));
    CHECK(unscaled(widths.min_medial_width) == Approx(0.38 - rounded_ends).margin(1e-5));
    CHECK(unscaled(widths.min_extrusion_width) == Approx(0.4 - 2. * rounded_ends).margin(1e-5));

    // Without XY distance nothing is widened.
    widths = centreline_widths(flow, 0);
    CHECK(unscaled(widths.min_medial_width) == Approx(0.4 - rounded_ends).margin(1e-5));
}

// Sweep the rounding collisions with local flows and scaled XY distances; centreline_widths returns settings by value without
// changing or owning its inputs. The helper trims points below min_medial_width, so these checks fail if equal settings disable
// growth or if a width below the XY limit survives the returned trim threshold.
TEST_CASE("The centreline trim threshold stays above equal growth settings", "[plugins][support-toolpath][support-branch-classify]")
{
    c_flow flow = {};
    flow.width = scale_i(0.42);
    flow.height = scale_i(0.15);
    flow.nozzle_diameter = scale_i(0.4);
    flow.spacing_ratio = 0.f;
    size_t shifted = 0;
    for (coord_t xy_distance = scale_i(0.0160); xy_distance <= scale_i(0.0162); ++xy_distance) {
        const CentrelineWidths widths = centreline_widths(flow, xy_distance);
        CHECK(widths.min_medial_width != widths.min_extrusion_width);
        if (widths.min_medial_width == widths.min_extrusion_width + 1)
            ++shifted;
    }
    CHECK(shifted == 1);

    shifted = 0;
    for (coord_t height = scale_i(0.1694); height <= scale_i(0.1696); ++height) {
        flow.height = height;
        const CentrelineWidths widths = centreline_widths(flow, scale_i(0.2));
        CHECK(widths.min_medial_width != widths.min_extrusion_width);
        if (widths.min_medial_width == widths.min_extrusion_width + 1)
            ++shifted;
    }
    CHECK(shifted >= 1);

    // At 0.4 mm nozzle and 0.15 mm height, a 0.016095 mm XY budget puts the trim limit at 0.367810 mm. One scaled unit
    // below it needs 0.0160955 mm per side, so the helper must trim that width rather than retain it at the limit.
    flow.height = scale_i(0.15);
    flow.spacing_ratio = 0.f;
    const coord_t xy_distance = scale_i(0.016095);
    const CentrelineWidths boundary = centreline_widths(flow, xy_distance);
    const coord_t below_xy_limit = flow.nozzle_diameter - 2 * xy_distance - 1;
    CHECK(boundary.min_medial_width == boundary.min_extrusion_width + 1);
    CHECK(below_xy_limit == boundary.min_extrusion_width - 1);
    CHECK(flow.nozzle_diameter - below_xy_limit > 2 * xy_distance);
    CHECK(below_xy_limit < boundary.min_medial_width);
}
