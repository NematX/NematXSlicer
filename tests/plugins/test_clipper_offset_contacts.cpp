/*
 * Clipper offset contacts
 * =======================
 *
 * A single `ClipperOffset` over every ring of an ExPolygon and the legacy
 * `offset_ex(ExPolygon)`, which offsets the contour and each hole separately and
 * subtracts the grown holes, cover the same area.  When a shrink makes rings
 * meet exactly along an edge they report a different topology: the grouped
 * offset keeps a zero-width bridge (an edge walked twice in opposite
 * directions, or a hole sharing an edge with its contour), the legacy path
 * splits the island or opens the hole there.
 *
 * The bridges come from the single clean-up union of all raw offset rings
 * inside `ClipperOffset::Execute`, whatever its fill rule.  A strictly simple
 * clean-up union was measured and changes nothing, because the contacts are
 * shared edges, not shared vertices.  Any later union of the cleaned rings
 * removes them.
 *
 * `slic3r_api::clipper_offset` offsets several rings one by one, keeps
 * `positive_fill_checked_union` for a lone ring, and checks every result for
 * bridges (see test_clipper_offset_strategies.cpp).  This file rebuilds the grouped
 * variants below directly with ClipperLib to document where the bridges come
 * from; the contacts report counts vertices lying on another ring, or on a
 * non-adjacent edge of their own ring.  A bridge always shows as two collinear
 * edges overlapping along a positive length, which is cheap to detect by
 * sorting the edges along their support lines:
 *
 * | variant | passes | description |
 * |---------|--------|-------------|
 * | grouped_offset | offset | one ClipperOffset over all rings, PolyTree output |
 * | positive_fill_offset | offset | same, cleaned with a positive-fill union instead of the bounding rectangle |
 * | union_after | offset + union | grouped offset to Paths, then a non-zero union |
 * | checked_union_after | offset (+ union) | grouped offset, non-zero union only when two edges overlap |
 * | positive_fill_checked_union | offset (+ union) | positive-fill offset, then the same checked union: the API for a lone ring |
 * | ring_difference | ring offsets + difference | legacy ring offsets, difference straight to a PolyTree |
 * | legacy_offset_ex | ring offsets + difference + union | `Slic3r::offset_ex(ExPolygon)` |
 *
 * The `[plugins]` tests check that the API gives exactly the legacy rings on
 * the divergent cases, that every variant covers the legacy area there, and
 * that the positive-fill clean-up and the API cover the same area as the
 * grouped offset on concave shapes for every join type.  The hidden
 * `[clipper-offset-contacts-report]` test prints topology and timing per variant and writes one SVG per case,
 * `clipper_offset_contacts_<case>.svg`, in the working directory: the input
 * followed by one panel per variant, with contact vertices in red.
 */

#include <catch2/catch.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Int128.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/SVG.hpp"
#include "clipper_offset_test_helpers.hpp"
// Last: it includes clipper.hpp, which must follow clipper_z.hpp from the headers above.
#include "libslic3r/Api/host/ClipperShapes.hpp"

namespace {

using namespace Slic3r;
using namespace clipper_offset_test;

struct ContactCase {
    std::string name;
    Slic3r::ExPolygon input;
    double delta_mm;
    clipper_join_type_t join;
    double limit;
    size_t calls_per_batch;
};

enum class Variant {
    GroupedOffset,
    PositiveFillOffset,
    UnionAfter,
    CheckedUnionAfter,
    PositiveFillCheckedUnion,
    RingDifference,
    Legacy
};

const std::array<Variant, 7> all_variants = {
    Variant::GroupedOffset, Variant::PositiveFillOffset, Variant::UnionAfter, Variant::CheckedUnionAfter,
    Variant::PositiveFillCheckedUnion, Variant::RingDifference, Variant::Legacy};

struct Topology {
    size_t islands = 0;
    size_t holes = 0;
    size_t contacts = 0;
    size_t self_touching_rings = 0;
};

struct VariantTiming {
    double median_us = 0.;
    double min_us = 0.;
};

// The four measured divergences between the API and the legacy offset.
std::vector<ContactCase> divergence_cases();

// Checkerboard holes growing until diagonal neighbours touch at their corners.
ContactCase checkerboard_case(const std::string &name, double delta_mm, size_t calls_per_batch);

// Concave shapes shrunk with every join type, for clean-up area equivalence.
std::vector<ContactCase> concave_cases();

// Offset one case through the public API and materialize the result.
Slic3r::ExPolygons api_offset(const ContactCase &contact_case);

// Run one variant on one case.
Slic3r::ExPolygons run_variant(Variant variant, const ContactCase &contact_case);

// One ClipperOffset over every ring as a PolyTree, optionally cleaned with a
// positive-fill union of the offset paths.
Slic3r::ExPolygons grouped_offset(const ContactCase &contact_case, bool positive_fill);

// One ClipperOffset over every ring to Paths, then a non-zero union.
Slic3r::ExPolygons grouped_offset_then_union(const ContactCase &contact_case);

// One ClipperOffset over every ring as a PolyTree, optionally cleaned with a
// positive fill, then a non-zero union of its rings only when two of their
// edges overlap.
Slic3r::ExPolygons grouped_offset_then_checked_union(const ContactCase &contact_case, bool positive_fill);

// Offset the contour and every hole separately, subtract into a PolyTree.
Slic3r::ExPolygons ring_difference(const ContactCase &contact_case);

// Apply the join limit and the shortest edge rule shared with ClipperUtils.
void configure_offsetter(ClipperLib::ClipperOffset &offsetter, const ContactCase &contact_case);

// Contour and holes of the input as Clipper paths, in API order.
ClipperLib::Paths ring_paths(const Slic3r::ExPolygon &expolygon);

// Collect a PolyTree into ExPolygons exactly like the API conversion.
Slic3r::ExPolygons expolygons_from_tree(const ClipperLib::PolyTree &tree);

ClipperLib::JoinType native_join(clipper_join_type_t join);

// Island, hole and contact counts of a result.
Topology topology(const Slic3r::ExPolygons &expolygons);

// Median and minimum time per call, variants interleaved batch by batch.
std::vector<VariantTiming> time_variants(const ContactCase &contact_case, size_t batches);

// Print one table row per variant for a case, and the rings of small results.
void print_report(const ContactCase &contact_case,
                  const std::vector<Slic3r::ExPolygons> &results,
                  const std::vector<VariantTiming> &timings);

// Print every ring of a result in millimetres.
void print_rings(const Slic3r::ExPolygons &expolygons);

// Draw the input and every variant result side by side.
void write_svg(const ContactCase &contact_case, const std::vector<Slic3r::ExPolygons> &results);

const char *variant_name(Variant variant);

// Random triangles with edges on a few shared lines, some of them overlapping
// or touching, within coordinates of +/- extent / 2.
ClipperLib::Paths random_line_triangles(std::mt19937_64 &rng, int64_t extent);

// Exhaustive exact search: two edges on one line overlapping along a positive length.
bool brute_force_overlap(const ClipperLib::Paths &rings);

std::vector<ContactCase> divergence_cases()
{
    return {
        {"88_hole_touching_contour_square",
         with_holes(rectangle_mm(105., 19., 118., 34.),
                    {rectangle_mm(110., 26., 112., 31.), rectangle_mm(110., 21., 115., 24.)}),
         -1., CLIPPER_JOIN_SQUARE, 3., 50},
        {"14_hole_splitting_island_miter3",
         with_holes(rectangle_mm(46., 2., 73., 12.),
                    {rectangle_mm(61., 4., 68., 10.), rectangle_mm(49., 5., 53., 9.)}),
         -1., CLIPPER_JOIN_MITER, 3., 50},
        {"244_holes_merging_contour_miter1.2",
         with_holes(rectangle_mm(85., 21., 104., 37.),
                    {rectangle_mm(93., 27., 97., 34.), rectangle_mm(91., 22., 98., 26.)}),
         -0.5, CLIPPER_JOIN_MITER, 1.2, 50},
        {"two_close_holes_square",
         with_holes(rectangle_mm(0., 0., 30., 30.),
                    {rectangle_mm(10., 10., 14., 14.), rectangle_mm(15., 15., 19., 19.)}),
         -5., CLIPPER_JOIN_SQUARE, 3., 50},
    };
}

ContactCase checkerboard_case(const std::string &name, double delta_mm, size_t calls_per_batch)
{
    // 6 mm holes on a 9 mm pitch, one cell out of two: growing them by 1.5 mm
    // with sharp miter corners makes diagonal neighbours meet exactly at their
    // corners.
    const size_t cells = 20;
    const double pitch = 9.;
    const double size = 6.;
    const double margin = 5.;
    Slic3r::Polygons holes;
    for (size_t row = 0; row < cells; ++row)
        for (size_t column = 0; column < cells; ++column)
            if ((row + column) % 2 == 0) {
                const double x = margin + pitch * column;
                const double y = margin + pitch * row;
                holes.push_back(rectangle_mm(x, y, x + size, y + size));
            }
    const double extent = 2. * margin + pitch * (cells - 1) + size;
    return {name, with_holes(rectangle_mm(0., 0., extent, extent), holes), delta_mm, CLIPPER_JOIN_MITER, 3.,
            calls_per_batch};
}

std::vector<ContactCase> concave_cases()
{
    std::vector<std::pair<std::string, Slic3r::ExPolygon>> shapes = {
        {"star", Slic3r::ExPolygon(star_mm(0., 0., 20., 8., 7))},
        {"holed_star", with_holes(star_mm(0., 0., 20., 8., 7), {rectangle_mm(-3., -2., 3., 2.)})},
        {"comb", Slic3r::ExPolygon(comb_mm(1., 8, 6.))},
        {"holed_comb", with_holes(comb_mm(1.2, 6, 5.), {rectangle_mm(2., 0.8, 9., 1.6)})},
    };
    for (const ContactCase &divergence : divergence_cases())
        shapes.emplace_back(divergence.name, divergence.input);

    const std::vector<std::pair<clipper_join_type_t, double>> joins = {
        {CLIPPER_JOIN_SQUARE, 3.}, {CLIPPER_JOIN_MITER, 3.}, {CLIPPER_JOIN_MITER, 1.2},
        {CLIPPER_JOIN_ROUND, scale_d(0.02)}};
    std::vector<ContactCase> cases;
    for (const std::pair<std::string, Slic3r::ExPolygon> &shape : shapes)
        for (const std::pair<clipper_join_type_t, double> &join : joins)
            for (double delta_mm : {0., -0.3, -0.6, -1., -2.5, -5.})
                cases.push_back({shape.first, shape.second, delta_mm, join.first, join.second, 1});
    return cases;
}

Slic3r::ExPolygons api_offset(const ContactCase &contact_case)
{
    Slic3r::PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    slic3r_api::StoredExPolygonCollection collection(storage);
    slic3r_api::StoredExPolygon stored(storage);
    stored.copy_from(slic3r_api::ExPolygon(reinterpret_cast<const expolygon_handle *>(&contact_case.input)));
    collection.push_back_move(std::move(stored));

    slic3r_api::ClipperOperand operand(storage, collection.readonly());
    slic3r_api::ClipperOperand offset = slic3r_api::clipper_offset(
        operand, scale_d(contact_case.delta_mm), contact_case.join, contact_case.limit);
    slic3r_api::StoredExPolygonCollection result = offset.to_expolygon_collection();
    return to_native(result.readonly());
}

Slic3r::ExPolygons run_variant(Variant variant, const ContactCase &contact_case)
{
    switch (variant) {
    case Variant::GroupedOffset:
        return grouped_offset(contact_case, false);
    case Variant::PositiveFillOffset:
        return grouped_offset(contact_case, true);
    case Variant::UnionAfter:
        return grouped_offset_then_union(contact_case);
    case Variant::CheckedUnionAfter:
        return grouped_offset_then_checked_union(contact_case, false);
    case Variant::PositiveFillCheckedUnion:
        return grouped_offset_then_checked_union(contact_case, true);
    case Variant::RingDifference:
        return ring_difference(contact_case);
    case Variant::Legacy:
        return Slic3r::offset_ex(contact_case.input, scale_d(contact_case.delta_mm), native_join(contact_case.join),
                                 contact_case.limit);
    }
    return {};
}

Slic3r::ExPolygons grouped_offset(const ContactCase &contact_case, bool positive_fill)
{
    ClipperLib::ClipperOffset offsetter;
    configure_offsetter(offsetter, contact_case);
    offsetter.ShrinkWithPositiveFill = positive_fill;
    offsetter.AddPaths(ring_paths(contact_case.input), native_join(contact_case.join), ClipperLib::etClosedPolygon);
    ClipperLib::PolyTree tree;
    offsetter.Execute(tree, scale_d(contact_case.delta_mm));
    return expolygons_from_tree(tree);
}

Slic3r::ExPolygons grouped_offset_then_union(const ContactCase &contact_case)
{
    ClipperLib::ClipperOffset offsetter;
    configure_offsetter(offsetter, contact_case);
    offsetter.AddPaths(ring_paths(contact_case.input), native_join(contact_case.join), ClipperLib::etClosedPolygon);
    ClipperLib::Paths offset;
    offsetter.Execute(offset, scale_d(contact_case.delta_mm));

    ClipperLib::Clipper clipper;
    clipper.AddPaths(offset, ClipperLib::ptSubject, true);
    ClipperLib::PolyTree tree;
    clipper.Execute(ClipperLib::ctUnion, tree, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
    return expolygons_from_tree(tree);
}

Slic3r::ExPolygons grouped_offset_then_checked_union(const ContactCase &contact_case, bool positive_fill)
{
    ClipperLib::ClipperOffset offsetter;
    configure_offsetter(offsetter, contact_case);
    offsetter.ShrinkWithPositiveFill = positive_fill;
    offsetter.AddPaths(ring_paths(contact_case.input), native_join(contact_case.join), ClipperLib::etClosedPolygon);
    ClipperLib::PolyTree tree;
    offsetter.Execute(tree, scale_d(contact_case.delta_mm));
    if (!Slic3r::ApiClipper::has_overlapping_edges(tree))
        return expolygons_from_tree(tree);

    ClipperLib::Paths rings;
    ClipperLib::PolyTreeToPaths(std::move(tree), rings);
    ClipperLib::Clipper clipper;
    clipper.AddPaths(rings, ClipperLib::ptSubject, true);
    ClipperLib::PolyTree cleaned;
    clipper.Execute(ClipperLib::ctUnion, cleaned, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
    return expolygons_from_tree(cleaned);
}

Slic3r::ExPolygons ring_difference(const ContactCase &contact_case)
{
    const ClipperLib::JoinType join = native_join(contact_case.join);
    const double delta = scale_d(contact_case.delta_mm);
    ClipperLib::ClipperOffset offsetter;
    configure_offsetter(offsetter, contact_case);
    ClipperLib::Clipper clipper;

    ClipperLib::Paths offset;
    offsetter.AddPath(contact_case.input.contour.points, join, ClipperLib::etClosedPolygon);
    offsetter.Execute(offset, delta);
    clipper.AddPaths(offset, ClipperLib::ptSubject, true);
    for (const Slic3r::Polygon &hole : contact_case.input.holes) {
        // Execute reorients the clockwise hole, so it grows with the opposite sign.
        offsetter.Clear();
        offsetter.AddPath(hole.points, join, ClipperLib::etClosedPolygon);
        offsetter.Execute(offset, -delta);
        clipper.AddPaths(offset, ClipperLib::ptClip, true);
    }

    ClipperLib::PolyTree tree;
    clipper.Execute(ClipperLib::ctDifference, tree, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
    return expolygons_from_tree(tree);
}

void configure_offsetter(ClipperLib::ClipperOffset &offsetter, const ContactCase &contact_case)
{
    if (contact_case.join == CLIPPER_JOIN_ROUND)
        offsetter.ArcTolerance = contact_case.limit;
    else
        offsetter.MiterLimit = contact_case.limit;
    offsetter.ShortestEdgeLength = std::abs(scale_d(contact_case.delta_mm) * Slic3r::ClipperOffsetShortestEdgeFactor);
}

ClipperLib::Paths ring_paths(const Slic3r::ExPolygon &expolygon)
{
    ClipperLib::Paths paths;
    paths.reserve(expolygon.holes.size() + 1);
    paths.emplace_back(expolygon.contour.points);
    for (const Slic3r::Polygon &hole : expolygon.holes)
        paths.emplace_back(hole.points);
    return paths;
}

Slic3r::ExPolygons expolygons_from_tree(const ClipperLib::PolyTree &tree)
{
    Slic3r::ExPolygons result;
    Slic3r::traverse_pt(&tree, &result);
    return result;
}

ClipperLib::JoinType native_join(clipper_join_type_t join)
{
    switch (join) {
    case CLIPPER_JOIN_SQUARE:
        return ClipperLib::jtSquare;
    case CLIPPER_JOIN_ROUND:
        return ClipperLib::jtRound;
    case CLIPPER_JOIN_MITER:
        return ClipperLib::jtMiter;
    }
    return ClipperLib::jtMiter;
}

Topology topology(const Slic3r::ExPolygons &expolygons)
{
    Topology result;
    result.islands = expolygons.size();
    for (const Slic3r::ExPolygon &expolygon : expolygons)
        result.holes += expolygon.holes.size();
    const Contacts contacts = find_contacts(expolygons);
    result.contacts = contacts.points.size();
    result.self_touching_rings = contacts.self_touching_rings;
    return result;
}

std::vector<VariantTiming> time_variants(const ContactCase &contact_case, size_t batches)
{
    std::vector<std::vector<double>> samples(all_variants.size());
    for (size_t batch = 0; batch < batches; ++batch)
        for (size_t variant = 0; variant < all_variants.size(); ++variant) {
            const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
            for (size_t call = 0; call < contact_case.calls_per_batch; ++call)
                run_variant(all_variants[variant], contact_case);
            const std::chrono::duration<double, std::micro> elapsed = std::chrono::steady_clock::now() - start;
            samples[variant].push_back(elapsed.count() / double(contact_case.calls_per_batch));
        }

    std::vector<VariantTiming> timings(all_variants.size());
    for (size_t variant = 0; variant < all_variants.size(); ++variant) {
        std::vector<double> &variant_samples = samples[variant];
        std::sort(variant_samples.begin(), variant_samples.end());
        timings[variant].median_us = variant_samples[variant_samples.size() / 2];
        timings[variant].min_us = variant_samples.front();
    }
    return timings;
}

void print_report(const ContactCase &contact_case,
                  const std::vector<Slic3r::ExPolygons> &results,
                  const std::vector<VariantTiming> &timings)
{
    const Slic3r::ExPolygons &legacy = results.back();
    const CanonicalShape legacy_canonical = canonical(legacy);
    const double legacy_median = timings.back().median_us;
    std::cout << "\n== " << contact_case.name << " (delta " << contact_case.delta_mm << " mm, "
              << contact_case.input.holes.size() << " holes)\n"
              << std::left << std::setw(30) << "variant" << std::right << std::setw(8) << "islands"
              << std::setw(7) << "holes" << std::setw(10) << "contacts" << std::setw(8) << "bridges"
              << std::setw(8) << "legacy" << std::setw(12) << "area_mm2" << std::setw(12) << "median_us"
              << std::setw(10) << "min_us" << std::setw(9) << "ratio" << '\n';
    for (size_t variant = 0; variant < all_variants.size(); ++variant) {
        const Topology shape = topology(results[variant]);
        const bool exact = canonical(results[variant]) == legacy_canonical;
        std::cout << std::left << std::setw(30) << variant_name(all_variants[variant]) << std::right
                  << std::setw(8) << shape.islands << std::setw(7) << shape.holes << std::setw(10) << shape.contacts
                  << std::setw(8) << shape.self_touching_rings << std::setw(8) << (exact ? "same" : "diff")
                  << std::setw(12) << std::setprecision(3) << symmetric_area_mm2(results[variant], legacy)
                  << std::setw(12) << std::fixed << std::setprecision(1) << timings[variant].median_us
                  << std::setw(10) << timings[variant].min_us << std::setw(9) << std::setprecision(2)
                  << timings[variant].median_us / legacy_median << std::defaultfloat << '\n';
    }
    if (contact_case.input.holes.size() > 2)
        return;
    for (size_t variant = 0; variant < all_variants.size(); ++variant) {
        std::cout << "  " << variant_name(all_variants[variant]) << ":\n";
        print_rings(results[variant]);
    }
}

void print_rings(const Slic3r::ExPolygons &expolygons)
{
    auto print_ring = [](const char *kind, const Slic3r::Polygon &ring) {
        std::cout << "    " << kind << std::setprecision(10);
        for (const Slic3r::Point &point : ring.points)
            std::cout << " (" << unscaled(point.x()) << ',' << unscaled(point.y()) << ')';
        std::cout << '\n';
    };
    for (const Slic3r::ExPolygon &expolygon : expolygons) {
        print_ring("contour", expolygon.contour);
        for (const Slic3r::Polygon &hole : expolygon.holes)
            print_ring("  hole", hole);
    }
}

void write_svg(const ContactCase &contact_case, const std::vector<Slic3r::ExPolygons> &results)
{
    const Slic3r::BoundingBox input_box = Slic3r::get_extents(contact_case.input);
    const coord_t step = input_box.size().x() + scale_i(4.);
    const coord_t label_y = input_box.max.y() + scale_i(1.5);
    Slic3r::BoundingBox canvas = input_box;
    canvas.merge(Slic3r::Point(input_box.max.x() + step * coord_t(results.size()), label_y + scale_i(1.)));

    Slic3r::SVG svg("clipper_offset_contacts_" + contact_case.name + ".svg", canvas, scale_i(3.));
    svg.draw(contact_case.input, "lightgrey");
    svg.draw_outline(contact_case.input, "black", "blue", scale_i(0.05));
    svg.draw_text(Slic3r::Point(input_box.min.x(), label_y), "input", "black");
    for (size_t variant = 0; variant < results.size(); ++variant) {
        const Slic3r::Point shift(step * coord_t(variant + 1), 0);
        Slic3r::ExPolygons panel = results[variant];
        for (Slic3r::ExPolygon &expolygon : panel)
            expolygon.translate(shift);
        svg.draw(panel, "lightskyblue");
        svg.draw_outline(panel, "black", "blue", scale_i(0.05));
        svg.draw(find_contacts(panel).points, "red", scale_i(0.3));
        svg.draw_text(Slic3r::Point(input_box.min.x() + shift.x(), label_y), variant_name(all_variants[variant]),
                      "black");
    }
    svg.Close();
}

const char *variant_name(Variant variant)
{
    switch (variant) {
    case Variant::GroupedOffset:
        return "grouped_offset";
    case Variant::PositiveFillOffset:
        return "positive_fill_offset";
    case Variant::UnionAfter:
        return "union_after";
    case Variant::CheckedUnionAfter:
        return "checked_union_after";
    case Variant::PositiveFillCheckedUnion:
        return "positive_fill_checked_union";
    case Variant::RingDifference:
        return "ring_difference";
    case Variant::Legacy:
        return "legacy_offset_ex";
    }
    return "";
}

ClipperLib::Paths random_line_triangles(std::mt19937_64 &rng, int64_t extent)
{
    // Two base lines, the second one parallel to the first and one unit away
    // along x, so near-coincident lines must not be confused with one line.
    std::uniform_int_distribution<int64_t> coordinate(-extent / 2, extent / 2);
    std::uniform_int_distribution<int64_t> component(-1000, 1000);
    int64_t dx = 0;
    int64_t dy = 0;
    while (dx == 0 && dy == 0) {
        dx = component(rng);
        dy = component(rng);
    }
    // Short segments at 31 positions per line: some overlap, some only touch.
    std::uniform_int_distribution<int64_t> step(-15, 15);
    std::uniform_int_distribution<int64_t> length(1, 3);
    const int64_t base_x = coordinate(rng) / 4;
    const int64_t base_y = coordinate(rng) / 4;
    std::uniform_int_distribution<int> line_choice(0, 1);

    ClipperLib::Paths rings;
    for (int triangle = 0; triangle < 6; ++triangle) {
        const int64_t shift = line_choice(rng);
        const int64_t first = step(rng);
        const int64_t second = first + (line_choice(rng) == 0 ? length(rng) : -length(rng));
        ClipperLib::Path ring{
            ClipperLib::IntPoint(base_x + shift + first * dx, base_y + first * dy),
            ClipperLib::IntPoint(base_x + shift + second * dx, base_y + second * dy),
            ClipperLib::IntPoint(coordinate(rng), coordinate(rng))};
        if (line_choice(rng) == 0)
            std::reverse(ring.begin(), ring.end());
        rings.push_back(std::move(ring));
    }
    return rings;
}

bool brute_force_overlap(const ClipperLib::Paths &rings)
{
    std::vector<std::pair<ClipperLib::IntPoint, ClipperLib::IntPoint>> edges;
    for (const ClipperLib::Path &ring : rings)
        for (size_t index = 0; index < ring.size(); ++index)
            if (ring[index] != ring[(index + 1) % ring.size()])
                edges.emplace_back(ring[index], ring[(index + 1) % ring.size()]);

    for (size_t first = 0; first < edges.size(); ++first)
        for (size_t second = first + 1; second < edges.size(); ++second) {
            const ClipperLib::IntPoint &a = edges[first].first;
            const ClipperLib::IntPoint &b = edges[first].second;
            const int64_t dx = b.x() - a.x();
            const int64_t dy = b.y() - a.y();
            const ClipperLib::IntPoint &p = edges[second].first;
            const ClipperLib::IntPoint &q = edges[second].second;
            if (Int128::sign_determinant_2x2_filtered(dx, dy, p.x() - a.x(), p.y() - a.y()) != 0 ||
                Int128::sign_determinant_2x2_filtered(dx, dy, q.x() - a.x(), q.y() - a.y()) != 0)
                continue;
            // Along the first edge, [a, b] and [p, q] overlap on a positive
            // length when the second edge reaches past a and starts before b.
            const bool past_a = Int128::sign_determinant_2x2_filtered(dx, -dy, p.y() - a.y(), p.x() - a.x()) > 0 ||
                                Int128::sign_determinant_2x2_filtered(dx, -dy, q.y() - a.y(), q.x() - a.x()) > 0;
            const bool before_b = Int128::sign_determinant_2x2_filtered(dx, -dy, p.y() - b.y(), p.x() - b.x()) < 0 ||
                                  Int128::sign_determinant_2x2_filtered(dx, -dy, q.y() - b.y(), q.x() - b.x()) < 0;
            if (past_a && before_b)
                return true;
        }
    return false;
}

} // namespace

TEST_CASE("Clipper bridge detection matches an exhaustive exact search", "[plugins][clipper][clipper-offset-contacts]")
{
    // Extents up to 2^45 units (35 km) keep the products of coordinates far
    // beyond 64 bits, as on plates several metres wide and more.
    std::mt19937_64 rng(0xb51d6eu);
    size_t overlapping = 0;
    size_t separate = 0;
    for (int64_t extent : {int64_t(1) << 24, int64_t(5000) * 1000000, int64_t(1) << 45})
        for (int trial = 0; trial < 400; ++trial) {
            const ClipperLib::Paths rings = random_line_triangles(rng, extent);
            const bool expected = brute_force_overlap(rings);
            INFO("extent " << extent << " trial " << trial);
            CHECK(Slic3r::ApiClipper::has_overlapping_edges(rings) == expected);
            ++(expected ? overlapping : separate);
        }
    CHECK(overlapping > 100);
    CHECK(separate > 100);
}

TEST_CASE("Clipper offset contacts: variants cover the legacy area", "[plugins][clipper][clipper-offset-contacts]")
{
    for (const ContactCase &contact_case : divergence_cases()) {
        INFO(contact_case.name);
        const Slic3r::ExPolygons legacy = run_variant(Variant::Legacy, contact_case);
        CHECK(canonical(api_offset(contact_case)) == canonical(legacy));
        for (Variant variant : all_variants) {
            INFO(variant_name(variant));
            CHECK(symmetric_area_mm2(run_variant(variant, contact_case), legacy) == Approx(0.).margin(1e-9));
        }
        // A union in the normal orientation restores the legacy rings exactly.
        for (Variant variant : {Variant::UnionAfter, Variant::CheckedUnionAfter, Variant::PositiveFillCheckedUnion,
                                Variant::RingDifference}) {
            INFO(variant_name(variant));
            CHECK(canonical(run_variant(variant, contact_case)) == canonical(legacy));
        }
    }
}

TEST_CASE("Clipper offset contacts: positive-fill clean-up and API cover the same area",
          "[plugins][clipper][clipper-offset-contacts]")
{
    for (const ContactCase &contact_case : concave_cases()) {
        INFO(contact_case.name << " delta " << contact_case.delta_mm << " join " << int(contact_case.join)
                               << " limit " << contact_case.limit);
        CHECK(symmetric_area_mm2(run_variant(Variant::PositiveFillOffset, contact_case),
                                 run_variant(Variant::GroupedOffset, contact_case)) == Approx(0.).margin(1e-9));
        CHECK(canonical(api_offset(contact_case)) == canonical(run_variant(Variant::Legacy, contact_case)));
    }
}

TEST_CASE("Clipper offset contacts: topology and timing report", "[.][clipper-offset-contacts-report]")
{
    std::vector<ContactCase> cases = divergence_cases();
    cases.push_back(checkerboard_case("checkerboard_touching_corners", -1.5, 2));
    cases.push_back(checkerboard_case("checkerboard_no_contact", -1., 2));
    for (const ContactCase &contact_case : cases) {
        std::vector<Slic3r::ExPolygons> results;
        for (Variant variant : all_variants)
            results.push_back(run_variant(variant, contact_case));
        print_report(contact_case, results, time_variants(contact_case, 41));
        write_svg(contact_case, results);
    }
}

// traverse_pt() lists an island before the islands nested in its holes, as union_ex() does: a frame with two holes comes first, then
// the two boxes lying in those holes.
TEST_CASE("Traversing a Clipper PolyTree lists the islands from outside to inside", "[plugins][clipper][clipper-island-order]")
{
    using Slic3r::Point;
    using Slic3r::Polygon;
    using Slic3r::Polygons;
    const coord_t unit = coord_t(1. / SCALING_FACTOR);
    Polygon unit_box;
    unit_box.points = { Point(0, 0), Point(unit, 0), Point(unit, unit), Point(0, unit) };
    Polygon frame = unit_box;
    frame.scale(20, 10);
    Polygon hole_left = unit_box;
    hole_left.scale(8);
    hole_left.translate(unit, unit);
    hole_left.reverse();
    Polygon hole_right = hole_left;
    hole_right.translate(unit * 10, 0);
    Polygon inner_left = unit_box;
    inner_left.scale(4);
    inner_left.translate(unit * 3, unit * 3);
    Polygon inner_right = inner_left;
    inner_right.translate(unit * 10, 0);
    const Polygons nested = Slic3r::union_({ frame, hole_left, hole_right, inner_left, inner_right });
    ClipperLib::PolyTree tree = Slic3r::union_pt(nested);
    Slic3r::ExPolygons islands;
    Slic3r::traverse_pt(&tree, &islands);
    REQUIRE(islands.size() == 3);
    CHECK(islands[0].holes.size() == 2);
    CHECK(islands[1].holes.empty());
    CHECK(islands[2].holes.empty());
    for (size_t inner = 1; inner < islands.size(); ++inner)
        CHECK(std::any_of(islands[0].holes.begin(), islands[0].holes.end(),
                          [&](const Polygon &hole) { return hole.contains(islands[inner].contour.first_point()); }));
    const Slic3r::ExPolygons reference = Slic3r::union_ex(nested);
    REQUIRE(reference.size() == islands.size());
    for (size_t idx = 0; idx < islands.size(); ++idx) {
        CHECK(islands[idx].contour.area() == Approx(reference[idx].contour.area()));
        CHECK(islands[idx].holes.size() == reference[idx].holes.size());
    }
}