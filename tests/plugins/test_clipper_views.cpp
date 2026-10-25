// Exercises core clipping and plugin views with local geometry; path coordinates retain direction
// and tests that inspect collection indices also retain the order returned by clipping.
#include <catch2/catch.hpp>

#include <chrono>
#include <future>
#include <iostream>
#include <random>
#include <set>
#include <algorithm>
#include "libslic3r/Api/plugin/cpp/LineDistancer.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Polygon.hpp"

namespace {
using namespace Slic3r;

Point point_mm(const double x, const double y)
{
    return Point(scale_i(x), scale_i(y));
}

Polygon rectangle_mm(const double min_x, const double min_y, const double max_x, const double max_y)
{
    Polygon polygon({
        point_mm(min_x, min_y),
        point_mm(max_x, min_y),
        point_mm(max_x, max_y),
        point_mm(min_x, max_y)
    });
    polygon.make_counter_clockwise();
    return polygon;
}

ExPolygon rectangle_with_two_holes()
{
    ExPolygon expolygon(rectangle_mm(0., 0., 20., 20.));
    expolygon.holes.push_back(rectangle_mm(2., 2., 5., 5.));
    expolygon.holes.push_back(rectangle_mm(8., 8., 11., 11.));
    for (Polygon &hole : expolygon.holes)
        hole.make_clockwise();
    return expolygon;
}

slic3r_api::Polygon view(const Polygon &polygon)
{
    return slic3r_api::Polygon(reinterpret_cast<const polygon_handle *>(&polygon));
}

slic3r_api::ExPolygon view(const ExPolygon &expolygon)
{
    return slic3r_api::ExPolygon(reinterpret_cast<const expolygon_handle *>(&expolygon));
}

void check_bbox(const c_bounding_box &bbox,
                const double min_x,
                const double min_y,
                const double max_x,
                const double max_y)
{
    CHECK(bbox.min.x == scale_i(min_x));
    CHECK(bbox.min.y == scale_i(min_y));
    CHECK(bbox.max.x == scale_i(max_x));
    CHECK(bbox.max.y == scale_i(max_y));
}

} // namespace

namespace {
using DirectedPath = std::vector<std::pair<coord_t, coord_t>>;
using DirectedPaths = std::vector<DirectedPath>;

// Only collection order is ignored: every bend and each path's direction remains significant.
DirectedPaths directed_paths(const Slic3r::Polylines &paths);
// Read coordinates through the public API without interpreting host geometry handles.
DirectedPaths directed_paths(const slic3r_api::PolylineCollection &paths);

DirectedPaths directed_paths(const Slic3r::Polylines &paths)
{
    DirectedPaths result;
    for (const Slic3r::Polyline &path : paths) {
        DirectedPath &coordinates = result.emplace_back();
        for (const Slic3r::Point &point : path.points)
            coordinates.emplace_back(point.x(), point.y());
    }
    std::sort(result.begin(), result.end());
    return result;
}

DirectedPaths directed_paths(const slic3r_api::PolylineCollection &paths)
{
    DirectedPaths result;
    for (const slic3r_api::Polyline path : paths) {
        DirectedPath &coordinates = result.emplace_back();
        for (uint32_t index = 0; index < path.size(); ++index) {
            const c_point point = path.at(index);
            coordinates.emplace_back(point.x, point.y);
        }
    }
    std::sort(result.begin(), result.end());
    return result;
}
}

TEST_CASE("Batch polyline clipping matches core directed paths and owns results", "[plugins][clipper][clipper-batch]")
{
    const bool difference = GENERATE(false, true);
    Slic3r::Polylines subjects{
        Slic3r::Polyline{point_mm(-5., 3.), point_mm(4., 8.), point_mm(15., 8.), point_mm(45., 3.)},
        Slic3r::Polyline{point_mm(-5., 10.), point_mm(45., 10.)},
        Slic3r::Polyline{point_mm(12., 12.), point_mm(17., 17.)},
        Slic3r::Polyline{point_mm(-5., -5.), point_mm(-1., -1.)},
        Slic3r::Polyline{point_mm(0., 0.), point_mm(20., 0.), point_mm(20., 20.)}
    };
    Slic3r::ExPolygons clips{rectangle_with_two_holes(), Slic3r::ExPolygon(rectangle_mm(30., 0., 40., 20.))};
    const DirectedPaths original = directed_paths(subjects);
    const Slic3r::ExPolygons original_clips = clips;
    const DirectedPaths expected = directed_paths(difference ? Slic3r::diff_pl(subjects, clips) : Slic3r::intersection_pl(subjects, clips));
    PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    const slic3r_api::PolylineCollection subject(reinterpret_cast<const polyline_collection_handle *>(&subjects));
    const slic3r_api::ExPolygonCollection clip(reinterpret_cast<const expolygon_collection_handle *>(&clips));
    slic3r_api::StoredPolylineCollection result = difference ?
        slic3r_api::clipper_diff_polylines_expolygons(storage, subject, clip) :
        slic3r_api::clipper_intersection_polylines_expolygons(storage, subject, clip);
    REQUIRE_FALSE(expected.empty());
    if (!difference)
        CHECK(std::any_of(expected.begin(), expected.end(), [](const DirectedPath &path) { return path.size() > 2; }));
    CHECK(directed_paths(result.readonly()) == expected);
    CHECK(directed_paths(subjects) == original);
    REQUIRE(clips.size() == original_clips.size());
    for (size_t index = 0; index < clips.size(); ++index) {
        CHECK(clips[index].contour.points == original_clips[index].contour.points);
        REQUIRE(clips[index].holes.size() == original_clips[index].holes.size());
        for (size_t hole = 0; hole < clips[index].holes.size(); ++hole)
            CHECK(clips[index].holes[hole].points == original_clips[index].holes[hole].points);
    }
    subjects.clear();
    clips.clear();
    CHECK(directed_paths(result.readonly()) == expected);
}

TEST_CASE("Batch polyline C ABI handles null and empty inputs", "[plugins][clipper][clipper-batch]")
{
    const bool difference = GENERATE(false, true);
    using Operation = polyline_collection_handle *(*)(storage_handle *, const polyline_collection_handle *, const expolygon_collection_handle *);
    const Operation operation = difference ? ::clipper_diff_polylines_expolygons : ::clipper_intersection_polylines_expolygons;
    PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    Slic3r::Polylines subjects{Slic3r::Polyline{point_mm(-5., 10.), point_mm(25., 10.)},
                              Slic3r::Polyline{}, Slic3r::Polyline{point_mm(1., 1.)}};
    Slic3r::ExPolygons clips{rectangle_with_two_holes()};
    const polyline_collection_handle *subject = reinterpret_cast<const polyline_collection_handle *>(&subjects);
    const expolygon_collection_handle *clip = reinterpret_cast<const expolygon_collection_handle *>(&clips);
    CHECK(operation(nullptr, subject, clip) == nullptr);
    CHECK(operation(storage, nullptr, clip) == nullptr);
    const DirectedPaths original = directed_paths(subjects);
    Slic3r::ExPolygons empty;
    const expolygon_collection_handle *empty_clips[] = {nullptr, reinterpret_cast<const expolygon_collection_handle *>(&empty)};
    for (const expolygon_collection_handle *mask : empty_clips) {
        polyline_collection_handle *handle = operation(storage, subject, mask);
        REQUIRE(handle != nullptr);
        CHECK(handle != subject);
        slic3r_api::StoredPolylineCollection result = slic3r_api::StoredPolylineCollection::adopt(storage, handle);
        CHECK(directed_paths(result.readonly()) == (difference ? original : DirectedPaths{}));
        subjects.front().points.front().x() += 1;
        CHECK(directed_paths(result.readonly()) == (difference ? original : DirectedPaths{}));
        subjects.front().points.front().x() -= 1;
        if (mask != nullptr) {
            const slic3r_api::PolylineCollection source(subject);
            const slic3r_api::ExPolygonCollection areas(mask);
            slic3r_api::StoredPolylineCollection cpp_result = difference ?
                slic3r_api::clipper_diff_polylines_expolygons(storage, source, areas) :
                slic3r_api::clipper_intersection_polylines_expolygons(storage, source, areas);
            CHECK(directed_paths(cpp_result.readonly()) == (difference ? original : DirectedPaths{}));
        }
    }
    {
        const DirectedPaths expected = directed_paths(difference ? Slic3r::diff_pl(subjects, clips) : Slic3r::intersection_pl(subjects, clips));
        slic3r_api::StoredPolylineCollection result = slic3r_api::StoredPolylineCollection::adopt(storage, operation(storage, subject, clip));
        CHECK(directed_paths(result.readonly()) == expected);
    }
    subjects.clear();
    for (const expolygon_collection_handle *mask : {clip, empty_clips[0], empty_clips[1]}) {
        polyline_collection_handle *handle = operation(storage, subject, mask);
        REQUIRE(handle != nullptr);
        slic3r_api::StoredPolylineCollection result = slic3r_api::StoredPolylineCollection::adopt(storage, handle);
        CHECK(result.empty());
    }
    const slic3r_api::PolylineCollection subject_view(subject);
    const slic3r_api::ExPolygonCollection clip_view(clip);
    CHECK(slic3r_api::clipper_diff_polylines_expolygons(storage, subject_view, clip_view).empty());
    CHECK(slic3r_api::clipper_intersection_polylines_expolygons(storage, subject_view, clip_view).empty());
}

TEST_CASE("ClipperOperand path_count counts raw non-empty paths", "[plugins][clipper]")
{
    PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    slic3r_api::ClipperContext clipper(storage);

    slic3r_api::ClipperOperand empty = clipper.empty();
    CHECK(empty.path_count() == 0);
    CHECK(empty.empty());

    Polygon polygon = rectangle_mm(0., 0., 10., 10.);
    slic3r_api::ClipperOperand polygon_operand = clipper(view(polygon));
    CHECK(polygon_operand.path_count() == 1);
    CHECK(!polygon_operand.empty());

    ExPolygon expolygon = rectangle_with_two_holes();
    slic3r_api::ClipperOperand expolygon_operand = clipper(view(expolygon));
    CHECK(expolygon_operand.path_count() == 3);

    Polygon other_polygon = rectangle_mm(30., 0., 40., 10.);
    slic3r_api::ClipperOperand concatenated = clipper.empty();
    concatenated += polygon_operand;
    CHECK(concatenated.path_count() == 1);
    concatenated += clipper(view(other_polygon));
    CHECK(concatenated.path_count() == 2);

    slic3r_api::ClipperOperand unioned = slic3r_api::clipper_union(concatenated);
    CHECK(unioned.path_count() == 2);
    CHECK(!unioned.empty());
    // A Clipper result is a PolyTree whose root has an empty contour; only its children are paths.
    CHECK(unioned.to_polygon_collection().size() == 2);
}

TEST_CASE("ClipperOperand concatenation treats null operands as empty", "[plugins][clipper]")
{
    PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    slic3r_api::ClipperContext clipper(storage);
    Polygon polygon = rectangle_mm(0., 0., 10., 10.);

    // clipper() is a null view; accumulating into it and appending it are no-ops.
    slic3r_api::ClipperOperand accumulated = clipper();
    accumulated += clipper();
    CHECK(accumulated.empty());
    accumulated += clipper(view(polygon));
    CHECK(accumulated.path_count() == 1);
    accumulated += clipper();
    CHECK(accumulated.path_count() == 1);
    accumulated += clipper.empty();
    CHECK(accumulated.path_count() == 1);

    slic3r_api::ClipperOperand null_operand = clipper();
    CHECK((null_operand + accumulated).path_count() == 1);
    CHECK((accumulated + null_operand).path_count() == 1);
    CHECK((null_operand + clipper()).empty());

    // Null views can be moved like owned ones.
    slic3r_api::ClipperOperand moved(std::move(null_operand));
    CHECK(moved.empty());
    moved = clipper();
    CHECK(moved.empty());

    // The C call keeps the first handle when there is nothing to append.
    clipper_shapes_handle *raw = ::clipper_shapes_from_polygon(storage, view(polygon).handle());
    raw = ::clipper_concat_replace(storage, raw, nullptr);
    REQUIRE(raw != nullptr);
    CHECK(::clipper_shapes_path_count(raw) == 1);
    CHECK(storage_free(storage, raw));
}

TEST_CASE("ClipperOperand bounding_box reads raw Clipper points", "[plugins][clipper]")
{
    PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    slic3r_api::ClipperContext clipper(storage);

    CHECK(clipper.empty().bounding_box().min.x == 0);
    CHECK(clipper.empty().bounding_box().min.y == 0);
    CHECK(clipper.empty().bounding_box().max.x == 0);
    CHECK(clipper.empty().bounding_box().max.y == 0);

    Polygon polygon = rectangle_mm(-2., 3., 10., 12.);
    check_bbox(clipper(view(polygon)).bounding_box(), -2., 3., 10., 12.);

    ExPolygon expolygon = rectangle_with_two_holes();
    check_bbox(clipper(view(expolygon)).bounding_box(), 0., 0., 20., 20.);

    Polygon other_polygon = rectangle_mm(30., -5., 40., 6.);
    slic3r_api::ClipperOperand concatenated = clipper(view(polygon));
    concatenated += clipper(view(other_polygon));
    check_bbox(concatenated.bounding_box(), -2., -5., 40., 12.);

    slic3r_api::ClipperOperand unioned = slic3r_api::clipper_union(concatenated);
    check_bbox(unioned.bounding_box(), -2., -5., 40., 12.);
}

// Clipper rounds intersections to 1 unit; its outputs are cleaned at SCALED_EPSILON so that
// callers never see vertices closer than that, from the host helpers or from the plugin API.
TEST_CASE("Clipper outputs merge vertices closer than SCALED_EPSILON", "[plugins][clipper]")
{
    // A 10 mm square whose bottom edge has a vertex SCALED_EPSILON / 2 outside its line, and
    // whose bottom-right corner is doubled by a vertex SCALED_EPSILON / 4 away from it.
    const coord_t side = scale_i(10.);
    Polygon noisy_square({
        Point(0, 0),
        Point(side / 2, -SCALED_EPSILON / 2),
        Point(side - SCALED_EPSILON / 4, 0),
        Point(side, SCALED_EPSILON / 4),
        Point(side, side),
        Point(0, side)
    });
    REQUIRE(noisy_square.is_counter_clockwise());

    ExPolygons unioned = union_ex(Polygons{noisy_square});
    REQUIRE(unioned.size() == 1);
    CHECK(unioned.front().contour.size() == 4);
    CHECK(unioned.front().holes.empty());

    PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    slic3r_api::ClipperContext clipper(storage);
    slic3r_api::ClipperOperand api_unioned = slic3r_api::clipper_union(clipper(view(noisy_square)));
    REQUIRE(api_unioned.path_count() == 1);
    slic3r_api::StoredPolygonCollection api_polygons = api_unioned.to_polygon_collection();
    REQUIRE(api_polygons.size() == 1);
    CHECK(api_polygons[0].size() == 4);
}

TEST_CASE("Clipper outputs drop slivers thinner than SCALED_EPSILON", "[plugins][clipper]")
{
    // Subtracting a copy shifted by `shift` along X leaves a vertical strip `shift` wide.
    const coord_t side = scale_i(10.);
    const Polygon square = rectangle_mm(0., 0., 10., 10.);
    auto shifted_square = [&square](const coord_t shift) {
        Polygon shifted = square;
        shifted.translate(shift, 0);
        return shifted;
    };

    CHECK(diff_ex(Polygons{square}, Polygons{shifted_square(SCALED_EPSILON / 2)}).empty());
    const ExPolygons wide_strip = diff_ex(Polygons{square}, Polygons{shifted_square(4 * SCALED_EPSILON)});
    REQUIRE(wide_strip.size() == 1);
    CHECK(wide_strip.front().contour.size() == 4);
    CHECK(wide_strip.front().contour.bounding_box().size().x() == 4 * SCALED_EPSILON);
    CHECK(wide_strip.front().contour.bounding_box().size().y() == side);

    PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    slic3r_api::ClipperContext clipper(storage);
    const Polygon thin_shift = shifted_square(SCALED_EPSILON / 2);
    const Polygon wide_shift = shifted_square(4 * SCALED_EPSILON);
    CHECK(slic3r_api::clipper_diff(clipper(view(square)), clipper(view(thin_shift))).empty());
    CHECK(slic3r_api::clipper_diff(clipper(view(square)), clipper(view(wide_shift))).path_count() == 1);
}

TEST_CASE("Clipper polyline outputs keep their ends and drop close interior points", "[plugins][clipper]")
{
    // The clip covers x >= 5 mm. The polyline has an interior vertex SCALED_EPSILON / 4
    // before that boundary and slightly off its line, so the cut point lands next to it.
    const coord_t half = scale_i(5.);
    const Polygon clip = rectangle_mm(5., -5., 15., 5.);
    const Polyline polyline({Point(0, 0), Point(half - SCALED_EPSILON / 4, 10), Point(2 * half, 20)});

    const Polylines outside = diff_pl(polyline, clip);
    REQUIRE(outside.size() == 1);
    REQUIRE(outside.front().size() == 2);
    CHECK((outside.front().front() == Point(0, 0) || outside.front().back() == Point(0, 0)));
    const Polylines inside = intersection_pl(polyline, clip);
    REQUIRE(inside.size() == 1);
    CHECK(inside.front().size() == 2);

    // A piece shorter than SCALED_EPSILON keeps its two ends.
    const Polygon thin_clip({Point(half, -half), Point(half + SCALED_EPSILON / 2, -half),
                             Point(half + SCALED_EPSILON / 2, half), Point(half, half)});
    const Polylines tiny = intersection_pl(Polyline({Point(0, 0), Point(2 * half, 0)}), thin_clip);
    REQUIRE(tiny.size() == 1);
    CHECK(tiny.front().size() == 2);
    CHECK(tiny.front().length() == Approx(double(SCALED_EPSILON / 2)));
}

TEST_CASE("Clipper polyline split matches diff and intersection and shares cut points", "[plugins][clipper]")
{
    // A slanted line crossing a square with a hole: inside pieces on both sides of
    // the hole, outside pieces before, in the hole and after.
    ExPolygon area(rectangle_mm(5., -5., 15., 5.));
    area.holes.push_back(rectangle_mm(8., -5. + 3., 12., 5. - 3.));
    area.holes.back().make_clockwise();
    const ExPolygons clip{area};
    const Polyline subject({point_mm(0., -1.), point_mm(20., 1.)});

    PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    const slic3r_api::Polyline subject_view(reinterpret_cast<const polyline_handle *>(&subject));
    const slic3r_api::ExPolygonCollection clip_view(reinterpret_cast<const expolygon_collection_handle *>(&clip));
    auto [inside, outside] = slic3r_api::clipper_split_polyline_expolygons(storage, subject_view, clip_view);

    // Collect each piece as its list of points, sorted so that piece order does not matter.
    auto pieces = [](const slic3r_api::StoredPolylineCollection &collection) {
        std::vector<std::vector<std::pair<coord_t, coord_t>>> out;
        for (uint32_t idx = 0; idx < collection.size(); ++idx) {
            std::vector<std::pair<coord_t, coord_t>> points;
            for (uint32_t pt_idx = 0; pt_idx < collection[idx].size(); ++pt_idx)
                points.emplace_back(collection[idx][pt_idx].x, collection[idx][pt_idx].y);
            out.push_back(std::move(points));
        }
        std::sort(out.begin(), out.end());
        return out;
    };
    REQUIRE(inside.size() == 2);
    REQUIRE(outside.size() == 3);
    CHECK(pieces(inside) == pieces(slic3r_api::clipper_intersection_polyline_expolygons(storage, subject_view, clip_view)));
    CHECK(pieces(outside) == pieces(slic3r_api::clipper_diff_polyline_expolygons(storage, subject_view, clip_view)));

    // Every cut point ends one piece on each side; only the subject ends belong to one side.
    std::multiset<std::pair<coord_t, coord_t>> inside_ends;
    std::multiset<std::pair<coord_t, coord_t>> outside_ends;
    for (const std::vector<std::pair<coord_t, coord_t>> &piece : pieces(inside)) {
        inside_ends.insert(piece.front());
        inside_ends.insert(piece.back());
    }
    for (const std::vector<std::pair<coord_t, coord_t>> &piece : pieces(outside)) {
        outside_ends.insert(piece.front());
        outside_ends.insert(piece.back());
    }
    outside_ends.erase(outside_ends.find({subject.front().x(), subject.front().y()}));
    outside_ends.erase(outside_ends.find({subject.back().x(), subject.back().y()}));
    CHECK(inside_ends == outside_ends);

    // Without clip area, everything is outside.
    const ExPolygons no_clip;
    const slic3r_api::ExPolygonCollection no_clip_view(reinterpret_cast<const expolygon_collection_handle *>(&no_clip));
    auto [nothing_inside, all_outside] = slic3r_api::clipper_split_polyline_expolygons(storage, subject_view, no_clip_view);
    CHECK(nothing_inside.empty());
    REQUIRE(all_outside.size() == 1);
    CHECK(all_outside[0].size() == 2);
}

TEST_CASE("Polyline set split matches intersection and diff", "[plugins][clipper]")
{
    // Two parallel lines crossing a square: each is cut into one inside and two outside pieces.
    const Polylines lines{Polyline({point_mm(0., 2.), point_mm(20., 3.)}), Polyline({point_mm(0., 6.), point_mm(20., 7.)})};
    const Polygons clip{rectangle_mm(5., 0., 15., 10.)};
    auto sorted = [](Polylines polylines) {
        std::vector<Points> out;
        for (Polyline &polyline : polylines)
            out.push_back(std::move(polyline.points));
        std::sort(out.begin(), out.end(), [](const Points &a, const Points &b) {
            return std::lexicographical_compare(a.begin(), a.end(), b.begin(), b.end(),
                                                [](const Point &l, const Point &r) { return l.x() < r.x() || (l.x() == r.x() && l.y() < r.y()); });
        });
        return out;
    };
    auto [inside, outside] = split_pl(lines, clip);
    CHECK(inside.size() == 2);
    CHECK(outside.size() == 4);
    CHECK(sorted(inside) == sorted(intersection_pl(lines, clip)));
    CHECK(sorted(outside) == sorted(diff_pl(lines, clip)));
}

TEST_CASE("ClipperOperand split matches intersection and diff", "[plugins][clipper]")
{
    PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    slic3r_api::ClipperContext clipper(storage);
    const ExPolygon subject = rectangle_with_two_holes();
    const Polygon clip = rectangle_mm(6., -5., 30., 9.);

    auto area = [](const slic3r_api::ClipperOperand &operand) {
        double out = 0.;
        for (const slic3r_api::ExPolygon expolygon : operand.to_expolygon_collection())
            out += expolygon.area();
        return out;
    };
    auto [inside, outside] = slic3r_api::clipper_split(clipper(view(subject)), clipper(view(clip)));
    CHECK(area(inside) == Approx(area(slic3r_api::clipper_intersection(clipper(view(subject)), clipper(view(clip))))));
    CHECK(area(outside) == Approx(area(slic3r_api::clipper_diff(clipper(view(subject)), clipper(view(clip))))));
    CHECK(area(inside) + area(outside) == Approx(subject.area()));
    // The clip cuts through the second hole: a notch on the inside, a notch and the first hole outside.
    CHECK(inside.path_count() == 1);
    CHECK(outside.path_count() == 2);

    // An empty clip leaves the whole subject outside.
    auto [nothing_inside, all_outside] = slic3r_api::clipper_split(clipper(view(subject)), clipper.empty());
    CHECK(nothing_inside.empty());
    CHECK(area(all_outside) == Approx(subject.area()));

    // A polyline operand stays an open path, as with the separate operations: a two-point line
    // crossing the square is not taken as a degenerate polygon.
    const Polyline line({point_mm(-5., 10.), point_mm(25., 10.)});
    const slic3r_api::Polyline line_view(reinterpret_cast<const polyline_handle *>(&line));
    const Polygon square = rectangle_mm(5., 5., 15., 15.);
    auto [line_inside, line_outside] = slic3r_api::clipper_split(clipper(line_view), clipper(view(square)));
    CHECK(line_inside.path_count() == 1);
    CHECK(line_outside.path_count() == 2);
    CHECK(line_inside.path_count() == slic3r_api::clipper_intersection(clipper(line_view), clipper(view(square))).path_count());
    CHECK(line_outside.path_count() == slic3r_api::clipper_diff(clipper(line_view), clipper(view(square))).path_count());
}

TEST_CASE("Clipper polyline pieces cut less than SCALED_EPSILON apart are glued back", "[plugins][clipper]")
{
    const coord_t half = scale_i(5.);
    const Polyline line({Point(0, 0), Point(2 * half, 0)});
    // A vertical strip SCALED_EPSILON / 2 wide crossing the middle of the line.
    const Polygon thin_strip({Point(half, -half), Point(half + SCALED_EPSILON / 2, -half),
                              Point(half + SCALED_EPSILON / 2, half), Point(half, half)});

    // diff: the two outside pieces become one again, from end to end.
    const Polylines outside = diff_pl(line, thin_strip);
    REQUIRE(outside.size() == 1);
    CHECK(outside.front().length() == Approx(double(2 * half)).epsilon(1e-6));

    // intersection: two areas separated by a gap SCALED_EPSILON / 2 wide give one piece.
    const ExPolygons two_areas{ExPolygon(rectangle_mm(-1., -1., 5., 1.)),
                               ExPolygon(Polygon({Point(half + SCALED_EPSILON / 2, scale_i(-1.)), Point(scale_i(11.), scale_i(-1.)),
                                                  Point(scale_i(11.), scale_i(1.)), Point(half + SCALED_EPSILON / 2, scale_i(1.))}))};
    const Polylines inside = intersection_pl(line, two_areas);
    REQUIRE(inside.size() == 1);
    CHECK(inside.front().length() == Approx(double(2 * half)).epsilon(1e-6));

    // split: the outside is glued and the tiny inside piece between the two cuts is dropped.
    auto [split_inside, split_outside] = split_pl(line, ExPolygons{ExPolygon(thin_strip)});
    CHECK(split_inside.empty());
    REQUIRE(split_outside.size() == 1);
    CHECK(split_outside.front().length() == Approx(double(2 * half)).epsilon(1e-6));

    // Ends close in the plane but far apart along the subject are not glued: this path comes back
    // SCALED_EPSILON / 2 above its start after a 34 mm detour inside the clip.
    const Polyline detour({point_mm(-5., 0.), point_mm(10., 0.), point_mm(10., 10.), Point(0, SCALED_EPSILON / 2),
                           Point(scale_i(-5.), SCALED_EPSILON / 2)});
    const ExPolygons detour_clip{ExPolygon(rectangle_mm(0., -1., 20., 20.))};
    CHECK(diff_pl(detour, detour_clip).size() == 2);
    auto [detour_inside, detour_outside] = split_pl(detour, detour_clip);
    REQUIRE(detour_inside.size() == 1);
    CHECK(detour_inside.front().length() == Approx(scale_d(20. + std::sqrt(200.))).epsilon(1e-4));
    CHECK(detour_outside.size() == 2);

    // A loop given as a polyline keeps its start: its two ends are never glued together.
    const Polyline loop({Point(0, 0), Point(2 * half, 0), Point(2 * half, 2 * half), Point(0, 2 * half), Point(0, 0)});
    const Polygon right_side({Point(half, -half), Point(3 * half, -half), Point(3 * half, 3 * half), Point(half, 3 * half)});
    CHECK(diff_pl(loop, right_side).size() == 2);
}

// A fully covered V retains all vertices and the subject direction in each single-path operation.
TEST_CASE("Clipper single subject keeps V direction", "[plugins][clipper][rejoin-direction]")
{
    const Polyline subject({point_mm(0., 0.), point_mm(5., 10.), point_mm(10., 0.)});
    const ExPolygons cover{ExPolygon(rectangle_mm(-1., -1., 11., 11.))};
    const ExPolygons away{ExPolygon(rectangle_mm(20., 20., 30., 30.))};
    const Polylines inside = intersection_pl(subject, cover);
    const Polylines outside = diff_pl(subject, away);
    const std::pair<Polylines, Polylines> split = split_pl(subject, cover);
    REQUIRE(inside.size() == 1);
    REQUIRE(outside.size() == 1);
    REQUIRE(split.first.size() == 1);
    CHECK(split.second.empty());
    CHECK(inside[0].points == subject.points);
    CHECK(outside[0].points == subject.points);
    CHECK(split.first[0].points == subject.points);
}

// Clipper emits both raw V fragments from larger to smaller X; the tiny ascending-branch cut rejoins
// them in subject direction without reversing the stored first fragment again.
TEST_CASE("Clipper consecutive reversed pieces join in subject direction", "[plugins][clipper][rejoin-direction]")
{
    const coord_t half = scale_i(5.);
    const Polyline subject({Point(0, 0), Point(half, 2 * half), Point(2 * half, 0)});
    const coord_t cut = half / 2;
    const Polygon strip({Point(cut, -half), Point(cut + SCALED_EPSILON / 4, -half),
                         Point(cut + SCALED_EPSILON / 4, 3 * half), Point(cut, 3 * half)});
    const Polylines outside = diff_pl(subject, strip);
    REQUIRE(outside.size() == 1);
    CHECK(outside[0].front() == subject.front());
    CHECK(outside[0].back() == subject.back());
    CHECK(outside[0].length() == Approx(subject.length()).epsilon(1e-6));
    for (size_t idx = 1; idx < outside[0].size(); ++idx) {
        CHECK(outside[0].points[idx].x() > outside[0].points[idx - 1].x());
        if (outside[0].points[idx].x() <= half)
            CHECK(outside[0].points[idx].y() > outside[0].points[idx - 1].y());
        else
            CHECK(outside[0].points[idx].y() < outside[0].points[idx - 1].y());
    }
}

// A closed polyline keeps its seam (the repeated start point) and its actual piece order.
TEST_CASE("Clipper closed subject places three pieces along the loop", "[plugins][clipper][rejoin-direction]")
{
    const Polyline subject({point_mm(0., 0.), point_mm(10., 0.), point_mm(10., 10.),
                            point_mm(0., 10.), point_mm(0., 0.)});
    const ExPolygons clip{ExPolygon(rectangle_mm(5., -1., 11., 11.))};
    const std::pair<Polylines, Polylines> split = split_pl(subject, clip);
    REQUIRE(split.first.size() == 1);
    REQUIRE(split.second.size() == 2);
    const Points first{point_mm(0., 0.), point_mm(5., 0.)};
    const Points middle{point_mm(5., 0.), point_mm(10., 0.), point_mm(10., 10.), point_mm(5., 10.)};
    const Points last{point_mm(5., 10.), point_mm(0., 10.), point_mm(0., 0.)};
    CHECK(split.second[0].points == first);
    CHECK(split.first[0].points == middle);
    CHECK(split.second[1].points == last);
    CHECK(split.second[0].back() == split.first[0].front());
    CHECK(split.first[0].back() == split.second[1].front());
    CHECK(split.second[1].back() == split.second[0].front());
    CHECK(split.second[0].length() + split.first[0].length() + split.second[1].length() == Approx(subject.length()));

    // Full coverage keeps the repeated start as the arrival at the total loop length.
    const Polylines covered = intersection_pl(subject, rectangle_mm(-1., -1., 11., 11.));
    REQUIRE(covered.size() == 1);
    CHECK(covered[0].points == subject.points);

    // Polygon overloads recombine the two seam fragments without reversing their direction.
    const Polygon polygon({point_mm(0., 0.), point_mm(10., 0.), point_mm(10., 10.), point_mm(0., 10.)});
    const Polylines recombined = diff_pl(Polygons{polygon}, Polygons{clip[0].contour});
    REQUIRE(recombined.size() == 1);
    const Points around_seam{point_mm(5., 10.), point_mm(0., 10.), point_mm(0., 0.), point_mm(5., 0.)};
    CHECK(recombined[0].points == around_seam);
}

// A tiny strip across the last side is glued outside and removed inside, away from the seam.
TEST_CASE("Clipper closed subject drops a tiny last-side piece", "[plugins][clipper][rejoin-direction]")
{
    const coord_t half = scale_i(5.);
    const Polyline subject({Point(0, 0), Point(2 * half, 0), Point(2 * half, 2 * half),
                            Point(0, 2 * half), Point(0, 0)});
    const Polygon strip({Point(-half, half), Point(half, half),
                         Point(half, half + SCALED_EPSILON / 2), Point(-half, half + SCALED_EPSILON / 2)});
    const std::pair<Polylines, Polylines> split = split_pl(subject, ExPolygons{ExPolygon(strip)});
    CHECK(split.first.empty());
    REQUIRE(split.second.size() == 1);
    CHECK(split.second[0].front() == subject.front());
    CHECK(split.second[0].back() == subject.back());
    REQUIRE(split.second[0].size() >= subject.size());
    CHECK(split.second[0].points[1] == subject.points[1]);
    CHECK(split.second[0].points[2] == subject.points[2]);
    CHECK(split.second[0].points[3] == subject.points[3]);
    CHECK(split.second[0].length() == Approx(subject.length()).epsilon(1e-6));
}

TEST_CASE("Clipper offset grows a polygon list of clockwise rings like ClipperOffset", "[plugins][clipper]")
{
    // Polygon lists keep the grouped ClipperOffset: painted facet slivers may
    // have any orientation, and a clockwise lowest ring reverses every ring,
    // so the set grows instead of being read as holes without a contour.
    PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    slic3r_api::ClipperContext clipper(storage);
    Polygon first = rectangle_mm(0., 0., 10., 10.);
    Polygon second = rectangle_mm(20., 0., 30., 10.);
    first.make_clockwise();
    second.make_clockwise();
    slic3r_api::ClipperOperand clockwise = clipper(view(first));
    clockwise += clipper(view(second));

    const slic3r_api::ClipperOperand grown =
        slic3r_api::clipper_offset(clockwise, scale_d(1.), CLIPPER_JOIN_MITER, 3.);
    CHECK(grown.to_expolygon_collection().size() == 2);
    check_bbox(grown.bounding_box(), -1., -1., 31., 11.);
}


TEST_CASE("Public LineDistancer index matches exhaustive queries and owns its data", "[plugins][line-distancer]")
{
    slic3r_api::LineDistancer reference;
    std::mt19937 random(42);
    std::uniform_int_distribution<int> coordinate(-10000,10000);
    for (size_t i=0;i<1200;++i) {
        const c_point a{coordinate(random),coordinate(random)};
        const c_point b=i%7==0?a:c_point{a.x+coordinate(random)/30,a.y+coordinate(random)/30};
        reference.append_segment(a,b);
    }
    slic3r_api::LineDistancer indexed=reference;
    indexed.build_index();
    for (int i=0;i<120;++i) {
        const c_point p{coordinate(random),coordinate(random)};
        CHECK(indexed.distance_from_lines(p)==Approx(reference.distance_from_lines(p)));
        for (double radius:{-1.,0.,1.,500.,50000.})
            CHECK(indexed.all_lines_in_radius(p,radius)==reference.all_lines_in_radius(p,radius));
        const c_point min{p.x-200,p.y-300},max{p.x+200,p.y+300};
        CHECK(indexed.all_lines_in_box(min,max)==reference.all_lines_in_box(min,max));
        CHECK(indexed.all_lines_in_box(max,min).empty());
    }
    // Invalidation must not expose the stale tree or change the original copy.
    const double original=indexed.distance_from_lines({50000,50000});
    slic3r_api::LineDistancer copy=indexed;
    copy.append_segment({50000,50000},{50000,50000});
    CHECK(copy.distance_from_lines({50000,50000})==0.);
    CHECK(indexed.distance_from_lines({50000,50000})==original);
    copy.build_index();
    slic3r_api::LineDistancer moved=std::move(copy);
    copy=slic3r_api::LineDistancer();
    CHECK(moved.distance_from_lines({50000,50000})==0.);
    CHECK(moved.all_lines_in_radius({50000,50000},0.)==std::vector<size_t>{1200});
    std::vector<std::future<double>> workers;
    for (int i=0;i<8;++i)
        workers.push_back(std::async(std::launch::async,[&indexed] {
            double sum=0;
            for (int j=0;j<100;++j) sum+=indexed.distance_from_lines({j*47,j*31});
            return sum;
        }));
    const double expected=workers.front().get();
    for (size_t i=1;i<workers.size();++i) CHECK(workers[i].get()==expected);

    slic3r_api::LineDistancer empty;
    empty.build_index();
    CHECK(std::isinf(empty.distance_from_lines({0,0},true)));
    CHECK(empty.all_lines_in_box({-1,-1},{1,1}).empty());
    empty.append_segment({0,0},{10,0});
    empty.build_index();
    CHECK(empty.all_lines_in_box({10,0},{10,0})==std::vector<size_t>{0});
    CHECK(empty.distance_from_lines({20,0})==10.); // finite segment endpoint
    CHECK(empty.distance_from_lines({5,0},true)==0.);
}

TEST_CASE("Public LineDistancer signed queries preserve area union and holes", "[plugins][line-distancer]")
{
    PluginStorage storage;
    slic3r_api::StoredExPolygonCollection areas(reinterpret_cast<storage_handle *>(&storage));
    const ExPolygon first=rectangle_with_two_holes();
    const ExPolygon second(rectangle_mm(9.,9.,15.,15.));
    areas.push_back(view(first)); areas.push_back(view(second));
    slic3r_api::LineDistancer indexed(areas.readonly());
    // An added segment does not add an interior; it also forces exhaustive queries.
    slic3r_api::LineDistancer exhaustive=indexed;
    exhaustive.append_segment({scale_i(100),scale_i(100)},{scale_i(100),scale_i(100)});
    for (double x:{-1.,0.,1.,2.,3.,5.,8.,9.,10.,11.,15.,20.,21.}) {
        for (double y:{-1.,0.,1.,3.,5.,9.,10.,11.,15.,20.,21.}) {
            const c_point p{scale_i(x),scale_i(y)};
            CHECK(indexed.distance_from_lines(p,true)==Approx(exhaustive.distance_from_lines(p,true)));
            const double distance=indexed.distance_from_lines(p,true);
            if (distance!=0.)
                CHECK((distance<0.)==(first.contains(Point(p.x,p.y)) || second.contains(Point(p.x,p.y))));
        }
    }
    CHECK(indexed.distance_from_lines({scale_i(3),scale_i(3)},true)>0.);
    CHECK(indexed.distance_from_lines({scale_i(10),scale_i(10)},true)<0.); // second area fills a hole
    CHECK(indexed.distance_from_lines({scale_i(1),scale_i(1)},true)<0.);
    // Closing the stored owners cannot invalidate the copied boundaries.
    areas.clear();
    CHECK(indexed.distance_from_lines({scale_i(1),scale_i(1)},true)<0.);
}

// Opt-in measurement, never a timing assertion. Include index preparation and
// identical query workloads; the checksum prevents dead-code elimination.
TEST_CASE("Public LineDistancer dense boundary benchmark", "[.][plugins][line-distancer-benchmark]")
{
    for (int count:{1000,10000,40000}) {
        slic3r_api::LineDistancer reference;
        for (int i=0;i<count;++i) {
            const coord_t x=(i%200)*1000,y=(i/200)*1000;
            reference.append_segment({x,y},{x+100,y+100});
        }
        slic3r_api::LineDistancer indexed=reference;
        const std::chrono::steady_clock::time_point start=std::chrono::steady_clock::now();
        indexed.build_index();
        const std::chrono::steady_clock::time_point built=std::chrono::steady_clock::now();
        double fast=0,slow=0;
        for (int i=0;i<1000;++i) fast+=indexed.distance_from_lines({(i%200)*1000+50,(i/200)*1000+200});
        const std::chrono::steady_clock::time_point queried=std::chrono::steady_clock::now();
        for (int i=0;i<1000;++i) slow+=reference.distance_from_lines({(i%200)*1000+50,(i/200)*1000+200});
        const std::chrono::steady_clock::time_point done=std::chrono::steady_clock::now();
        CHECK(fast==Approx(slow));
        std::cout << "LineDistancer edges=" << count
                  << " build_ms=" << std::chrono::duration<double,std::milli>(built-start).count()
                  << " indexed_ms=" << std::chrono::duration<double,std::milli>(queried-built).count()
                  << " exhaustive_ms=" << std::chrono::duration<double,std::milli>(done-queried).count() << '\n';
    }
}
