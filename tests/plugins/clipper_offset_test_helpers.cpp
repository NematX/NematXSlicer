/*
 * Shared helpers for Clipper offset parity tests.
 *
 * These definitions deliberately contain no test-specific policy.  Strict
 * comparison and shape construction are shared so related tests cannot drift
 * in their interpretation of equal geometry.
 */

#include "clipper_offset_test_helpers.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <tuple>

#include <catch2/catch.hpp>

namespace clipper_offset_test {

CanonicalShape canonical(const Slic3r::ExPolygons &expolygons)
{
    CanonicalShape result;
    result.reserve(expolygons.size());
    for (const Slic3r::ExPolygon &expolygon : expolygons) {
        CanonicalExPolygon canonical_expolygon;
        canonical_expolygon.contour = canonical_ring(expolygon.contour);
        canonical_expolygon.holes.reserve(expolygon.holes.size());
        for (const Slic3r::Polygon &hole : expolygon.holes)
            canonical_expolygon.holes.push_back(canonical_ring(hole));
        std::sort(canonical_expolygon.holes.begin(), canonical_expolygon.holes.end());
        result.push_back(std::move(canonical_expolygon));
    }
    std::sort(result.begin(), result.end());
    return result;
}

CanonicalRing canonical_ring(const Slic3r::Polygon &polygon)
{
    CanonicalRing result;
    if (polygon.points.empty())
        return result;

    size_t first = 0;
    for (size_t index = 1; index < polygon.points.size(); ++index) {
        const std::pair<Slic3r::coord_t, Slic3r::coord_t> candidate(
            polygon.points[index].x(), polygon.points[index].y());
        const std::pair<Slic3r::coord_t, Slic3r::coord_t> current(
            polygon.points[first].x(), polygon.points[first].y());
        if (candidate < current)
            first = index;
    }

    result.reserve(polygon.points.size());
    for (size_t offset = 0; offset < polygon.points.size(); ++offset) {
        const Slic3r::Point &point = polygon.points[(first + offset) % polygon.points.size()];
        result.emplace_back(point.x(), point.y());
    }
    return result;
}

bool operator==(const CanonicalExPolygon &first, const CanonicalExPolygon &second)
{
    return first.contour == second.contour && first.holes == second.holes;
}

bool operator<(const CanonicalExPolygon &first, const CanonicalExPolygon &second)
{
    if (first.contour != second.contour)
        return first.contour < second.contour;
    return first.holes < second.holes;
}

// Check borrowed nonempty rings for CCW contours and CW holes; emit a check per ring and format context only on failure, without modifying inputs.
void check_orientation(const std::string &info, const Slic3r::ExPolygons &expolygons)
{
    for (size_t island = 0; island < expolygons.size(); ++island) {
        const Slic3r::ExPolygon &expolygon = expolygons[island];
        if (!expolygon.contour.points.empty()) {
            const bool correct = expolygon.contour.is_counter_clockwise();
            if (correct) {
                CHECK(correct);
            } else {
                INFO(info << " island=" << island << " contour");
                CHECK(correct);
            }
        }
        for (size_t hole = 0; hole < expolygon.holes.size(); ++hole) {
            if (!expolygon.holes[hole].points.empty()) {
                const bool correct = expolygon.holes[hole].is_clockwise();
                if (correct) {
                    CHECK(correct);
                } else {
                    INFO(info << " island=" << island << " hole=" << hole);
                    CHECK(correct);
                }
            }
        }
    }
}

void compare(const std::string &info,
             const Slic3r::ExPolygons &api,
             const Slic3r::ExPolygons &legacy,
             bool documentary)
{
    check_orientation(info + " api", api);
    check_orientation(info + " legacy", legacy);
    const CanonicalShape api_shape = canonical(api);
    const CanonicalShape legacy_shape = canonical(legacy);
    if (api_shape == legacy_shape) {
        if (!documentary)
            CHECK(api_shape == legacy_shape);
        return;
    }

    INFO(info);
    INFO("api " << describe(api));
    INFO("legacy " << describe(legacy));
    INFO("symmetric difference mm2 " << symmetric_area_mm2(api, legacy));
    if (documentary)
        WARN("documented divergence");
    else
        CHECK(api_shape == legacy_shape);
}

std::string describe(const Slic3r::ExPolygons &expolygons)
{
    std::ostringstream output;
    output << "islands=" << expolygons.size();
    for (size_t island = 0; island < expolygons.size(); ++island) {
        const Slic3r::ExPolygon &expolygon = expolygons[island];
        const double scale = Slic3r::scale_d(1.);
        output << " island[" << island << "] vertices=" << expolygon.contour.points.size()
               << " area_mm2=" << std::abs(expolygon.area()) / (scale * scale)
               << " holes=" << expolygon.holes.size() << " contour_um=";
        const size_t point_count = std::min<size_t>(12, expolygon.contour.points.size());
        output << '[';
        for (size_t point = 0; point < point_count; ++point) {
            if (point != 0)
                output << ',';
            output << '(' << expolygon.contour.points[point].x() << ','
                   << expolygon.contour.points[point].y() << ')';
        }
        if (expolygon.contour.points.size() > point_count)
            output << ",...";
        output << "] holes_um=";
        output << '[';
        for (size_t hole = 0; hole < expolygon.holes.size(); ++hole) {
            if (hole != 0)
                output << ',';
            output << '[';
            const size_t hole_point_count = std::min<size_t>(12, expolygon.holes[hole].points.size());
            for (size_t point = 0; point < hole_point_count; ++point) {
                if (point != 0)
                    output << ',';
                output << '(' << expolygon.holes[hole].points[point].x() << ','
                       << expolygon.holes[hole].points[point].y() << ')';
            }
            if (expolygon.holes[hole].points.size() > hole_point_count)
                output << ",...";
            output << ']';
        }
        output << ']';
    }
    return output.str();
}

Slic3r::ExPolygons boolean_ordered(const Slic3r::ExPolygons &expolygons)
{
    Slic3r::ExPolygons result(expolygons.begin(), expolygons.end());
    std::sort(result.begin(), result.end(), [](const Slic3r::ExPolygon &first,
                                               const Slic3r::ExPolygon &second) {
        return std::abs(first.contour.area()) > std::abs(second.contour.area());
    });
    return result;
}

double symmetric_area_mm2(const Slic3r::ExPolygons &first,
                          const Slic3r::ExPolygons &second)
{
    const Slic3r::ExPolygons ordered_first = boolean_ordered(first);
    const Slic3r::ExPolygons ordered_second = boolean_ordered(second);
    const Slic3r::ExPolygons first_only = Slic3r::diff_ex(ordered_first, ordered_second);
    const Slic3r::ExPolygons second_only = Slic3r::diff_ex(ordered_second, ordered_first);
    double area = 0.;
    for (const Slic3r::ExPolygon &expolygon : first_only)
        area += std::abs(expolygon.area());
    for (const Slic3r::ExPolygon &expolygon : second_only)
        area += std::abs(expolygon.area());
    const double scale = Slic3r::scale_d(1.);
    return area / (scale * scale);
}

bool needs_manual_polygon_conversion(const std::vector<Slic3r::ExPolygon> &expolygons)
{
    for (size_t first = 0; first < expolygons.size(); ++first) {
        if (expolygons[first].contour.points.empty())
            continue;
        for (size_t second = first + 1; second < expolygons.size(); ++second)
            for (const Slic3r::Polygon &hole : expolygons[second].holes)
                if (hole.contains(expolygons[first].contour.front()))
                    return true;
    }
    return false;
}

Slic3r::Polygons flatten_polygons(const std::vector<Slic3r::ExPolygon> &expolygons)
{
    Slic3r::Polygons result;
    for (const Slic3r::ExPolygon &expolygon : expolygons) {
        result.push_back(expolygon.contour);
        for (const Slic3r::Polygon &hole : expolygon.holes)
            result.push_back(hole);
    }
    return result;
}

Slic3r::ExPolygons to_native(const slic3r_api::ExPolygonCollection &collection)
{
    Slic3r::ExPolygons result;
    result.reserve(collection.size());
    for (const slic3r_api::ExPolygon source : collection) {
        Slic3r::ExPolygon native;
        const slic3r_api::Polygon contour = source.contour();
        for (const c_point point : contour.points())
            native.contour.points.emplace_back(point.x, point.y);
        for (const slic3r_api::Polygon source_hole : source.holes()) {
            Slic3r::Polygon hole;
            for (const c_point point : source_hole.points())
                hole.points.emplace_back(point.x, point.y);
            native.holes.push_back(std::move(hole));
        }
        result.push_back(std::move(native));
    }
    return result;
}

slic3r_api::ExPolygon view(const Slic3r::ExPolygon &expolygon)
{
    return slic3r_api::ExPolygon(reinterpret_cast<const expolygon_handle *>(&expolygon));
}

slic3r_api::Polygon view(const Slic3r::Polygon &polygon)
{
    return slic3r_api::Polygon(reinterpret_cast<const polygon_handle *>(&polygon));
}

Slic3r::Polygon rectangle_mm(double min_x, double min_y, double max_x, double max_y)
{
    Slic3r::Polygon polygon({
        Slic3r::Point(Slic3r::scale_i(min_x), Slic3r::scale_i(min_y)),
        Slic3r::Point(Slic3r::scale_i(max_x), Slic3r::scale_i(min_y)),
        Slic3r::Point(Slic3r::scale_i(max_x), Slic3r::scale_i(max_y)),
        Slic3r::Point(Slic3r::scale_i(min_x), Slic3r::scale_i(max_y))
    });
    polygon.make_counter_clockwise();
    return polygon;
}

Slic3r::ExPolygon square_with_holes(const Slic3r::Polygon &outer,
                                    std::initializer_list<Slic3r::Polygon> holes)
{
    Slic3r::ExPolygon result(outer);
    for (const Slic3r::Polygon &hole : holes) {
        Slic3r::Polygon oriented_hole(hole);
        oriented_hole.make_clockwise();
        result.holes.push_back(std::move(oriented_hole));
    }
    return result;
}

Slic3r::ExPolygon rotated(Slic3r::ExPolygon expolygon, double angle_rad)
{
    expolygon.rotate(angle_rad);
    return expolygon;
}

Slic3r::ExPolygon start_shifted(Slic3r::ExPolygon expolygon, size_t shift)
{
    if (!expolygon.contour.points.empty()) {
        shift %= expolygon.contour.points.size();
        std::rotate(expolygon.contour.points.begin(), expolygon.contour.points.begin() + shift,
                    expolygon.contour.points.end());
    }
    return expolygon;
}

Slic3r::Polygon star_mm(double x,
                        double y,
                        double outer_radius,
                        double inner_radius,
                        size_t tips)
{
    Slic3r::Polygon polygon;
    for (size_t index = 0; index < 2 * tips; ++index) {
        const double angle = std::acos(-1.) * double(index) / double(tips);
        const double radius = index % 2 == 0 ? outer_radius : inner_radius;
        polygon.points.emplace_back(Slic3r::scale_i(x + radius * std::cos(angle)),
                                    Slic3r::scale_i(y + radius * std::sin(angle)));
    }
    return polygon;
}

Slic3r::Polygon comb_mm(double tooth_width, size_t teeth, double tooth_height)
{
    const double width = tooth_width * double(2 * teeth - 1);
    Slic3r::Polygon polygon;
    polygon.points.emplace_back(Slic3r::scale_i(0.), Slic3r::scale_i(0.));
    polygon.points.emplace_back(Slic3r::scale_i(width), Slic3r::scale_i(0.));
    for (size_t tooth = teeth; tooth > 0; --tooth) {
        const double right = tooth_width * double(2 * tooth - 1);
        const double left = right - tooth_width;
        if (tooth != teeth)
            polygon.points.emplace_back(Slic3r::scale_i(right), Slic3r::scale_i(2.));
        polygon.points.emplace_back(Slic3r::scale_i(right), Slic3r::scale_i(2. + tooth_height));
        polygon.points.emplace_back(Slic3r::scale_i(left), Slic3r::scale_i(2. + tooth_height));
        if (tooth != 1)
            polygon.points.emplace_back(Slic3r::scale_i(left), Slic3r::scale_i(2.));
    }
    return polygon;
}

Slic3r::ExPolygon with_holes(const Slic3r::Polygon &contour,
                             const Slic3r::Polygons &holes)
{
    Slic3r::ExPolygon result(contour);
    for (const Slic3r::Polygon &hole : holes) {
        Slic3r::Polygon oriented_hole(hole);
        oriented_hole.make_clockwise();
        result.holes.push_back(std::move(oriented_hole));
    }
    return result;
}

std::vector<Slic3r::ExPolygon> random_disjoint_holed_rectangles(std::mt19937 &rng)
{
    std::uniform_int_distribution<int> rectangle_count_distribution(1, 3);
    std::uniform_int_distribution<int> size_distribution(10, 30);
    std::uniform_int_distribution<int> hole_count_distribution(0, 2);
    const int rectangle_count = rectangle_count_distribution(rng);
    std::vector<int> cells{0, 1, 2};
    std::shuffle(cells.begin(), cells.end(), rng);
    std::vector<Slic3r::ExPolygon> inputs;

    for (int rectangle_index = 0; rectangle_index < rectangle_count; ++rectangle_index) {
        const int width = size_distribution(rng);
        const int height = size_distribution(rng);
        std::uniform_int_distribution<int> x_offset_distribution(1, 39 - width);
        std::uniform_int_distribution<int> y_offset_distribution(1, 39 - height);
        const int min_x = cells[rectangle_index] * 40 + x_offset_distribution(rng);
        const int min_y = y_offset_distribution(rng);
        const int max_x = min_x + width;
        const int max_y = min_y + height;
        Slic3r::ExPolygon expolygon(rectangle_mm(min_x, min_y, max_x, max_y));
        const int requested_holes = hole_count_distribution(rng);
        const int max_hole_width = std::min(8, width - 2);
        const int max_hole_height = std::min(8, height - 2);
        std::vector<IntegerRect> hole_rectangles;

        for (int hole_index = 0; hole_index < requested_holes && max_hole_width >= 2 && max_hole_height >= 2;
             ++hole_index) {
            bool placed = false;
            for (int attempt = 0; attempt < 100 && !placed; ++attempt) {
                std::uniform_int_distribution<int> hole_width_distribution(2, max_hole_width);
                std::uniform_int_distribution<int> hole_height_distribution(2, max_hole_height);
                const int hole_width = hole_width_distribution(rng);
                const int hole_height = hole_height_distribution(rng);
                std::uniform_int_distribution<int> hole_x_distribution(min_x + 1, max_x - hole_width - 1);
                std::uniform_int_distribution<int> hole_y_distribution(min_y + 1, max_y - hole_height - 1);
                const IntegerRect candidate{hole_x_distribution(rng), hole_y_distribution(rng), 0, 0};
                IntegerRect actual = candidate;
                actual.max_x = actual.min_x + hole_width;
                actual.max_y = actual.min_y + hole_height;
                bool separated = true;
                for (const IntegerRect &existing : hole_rectangles)
                    separated = separated && separated_by_one_mm(actual, existing);
                if (separated) {
                    hole_rectangles.push_back(actual);
                    expolygon.holes.push_back(rectangle_mm(actual.min_x, actual.min_y,
                                                           actual.max_x, actual.max_y));
                    expolygon.holes.back().make_clockwise();
                    placed = true;
                }
            }
        }
        inputs.push_back(std::move(expolygon));
    }
    std::shuffle(inputs.begin(), inputs.end(), rng);
    return inputs;
}

Slic3r::ClipperLib::JoinType legacy_join(clipper_join_type_t join)
{
    switch (join) {
    case CLIPPER_JOIN_SQUARE:
        return Slic3r::ClipperLib::jtSquare;
    case CLIPPER_JOIN_ROUND:
        return Slic3r::ClipperLib::jtRound;
    case CLIPPER_JOIN_MITER:
        return Slic3r::ClipperLib::jtMiter;
    }
    return Slic3r::ClipperLib::jtMiter;
}

const char *join_name(clipper_join_type_t join)
{
    switch (join) {
    case CLIPPER_JOIN_SQUARE:
        return "square";
    case CLIPPER_JOIN_ROUND:
        return "round";
    case CLIPPER_JOIN_MITER:
        return "miter";
    }
    return "unknown";
}

bool separated_by_one_mm(const IntegerRect &first, const IntegerRect &second)
{
    return first.max_x + 1 <= second.min_x || second.max_x + 1 <= first.min_x ||
           first.max_y + 1 <= second.min_y || second.max_y + 1 <= first.min_y;
}

bool on_segment(const Slic3r::Point &point,
                const Slic3r::Point &a,
                const Slic3r::Point &b)
{
    const int64_t cross = int64_t(b.x() - a.x()) * int64_t(point.y() - a.y()) -
                          int64_t(b.y() - a.y()) * int64_t(point.x() - a.x());
    return cross == 0 && std::min(a.x(), b.x()) <= point.x() && point.x() <= std::max(a.x(), b.x()) &&
           std::min(a.y(), b.y()) <= point.y() && point.y() <= std::max(a.y(), b.y());
}

Contacts find_contacts(const Slic3r::ExPolygons &expolygons)
{
    std::vector<const Slic3r::Polygon *> rings;
    for (const Slic3r::ExPolygon &expolygon : expolygons) {
        rings.push_back(&expolygon.contour);
        for (const Slic3r::Polygon &hole : expolygon.holes)
            rings.push_back(&hole);
    }

    Contacts result;
    for (const Slic3r::Polygon *ring : rings) {
        bool self_touching = false;
        const size_t count = ring->points.size();
        for (size_t vertex = 0; vertex < count; ++vertex)
            for (const Slic3r::Polygon *other : rings) {
                const size_t other_count = other->points.size();
                for (size_t edge = 0; edge < other_count; ++edge) {
                    const size_t next = (edge + 1) % other_count;
                    if (other == ring && (edge == vertex || next == vertex))
                        continue;
                    if (on_segment(ring->points[vertex], other->points[edge], other->points[next])) {
                        result.points.push_back(ring->points[vertex]);
                        self_touching |= other == ring;
                    }
                }
            }
        if (self_touching)
            ++result.self_touching_rings;
    }
    std::sort(result.points.begin(), result.points.end());
    result.points.erase(std::unique(result.points.begin(), result.points.end()), result.points.end());
    return result;
}

} // namespace clipper_offset_test
