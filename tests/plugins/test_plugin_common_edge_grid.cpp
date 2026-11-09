#include <catch2/catch.hpp>

#include <random>
#include <utility>
#include <vector>

#include "libslic3r/Plugins/Common/PluginEdgeGrid.hpp"

#include "libslic3r/EdgeGrid.hpp"
#include "libslic3r/Point.hpp"

/*
Common contour grid checks
==========================

The core is the existing Slic3r implementation.
Build core and plugin grids from matching contours and compare cell segment
references, visited-cell sequences and signed closest-point results. An API
ExPolygon supplies the contour and holes for a separate borrowed-view check.

Tests
|-- matching work contours: compare rasterization and line traversal
|-- API ExPolygon: compare indexed contour segments
`-- signed closest points: compare selected edges, distances and parameters
Box traversal additionally checks clipped indices, empty cells and early stopping.
*/

namespace {

namespace rect_work = slic3r_api::PluginCommon::polygon_work;
namespace rect_edge_grid = slic3r_api::PluginCommon::EdgeGrid;

std::vector<Slic3r::Points> make_core_contours()
{
    std::vector<Slic3r::Points> contours;
    contours.push_back({ Slic3r::Point(-8000000, -6000000), Slic3r::Point(9000000, -6000000),
                         Slic3r::Point(9000000, 7000000), Slic3r::Point(-8000000, 7000000) });
    contours.push_back({ Slic3r::Point(-2000000, -1000000), Slic3r::Point(-2000000, 3000000),
                         Slic3r::Point(3000000, 3000000), Slic3r::Point(3000000, -1000000) });
    contours.push_back({ Slic3r::Point(-6000000, 2000000), Slic3r::Point(1000000, 5000000),
                         Slic3r::Point(6000000, 1000000), Slic3r::Point(2000000, -3000000) });
    return contours;
}

std::vector<rect_work::Points> make_ported_contours()
{
    std::vector<rect_work::Points> contours;
    contours.push_back({ rect_work::Point(-8000000, -6000000), rect_work::Point(9000000, -6000000),
                         rect_work::Point(9000000, 7000000), rect_work::Point(-8000000, 7000000) });
    contours.push_back({ rect_work::Point(-2000000, -1000000), rect_work::Point(-2000000, 3000000),
                         rect_work::Point(3000000, 3000000), rect_work::Point(3000000, -1000000) });
    contours.push_back({ rect_work::Point(-6000000, 2000000), rect_work::Point(1000000, 5000000),
                         rect_work::Point(6000000, 1000000), rect_work::Point(2000000, -3000000) });
    return contours;
}

struct TraversalVisitor
{
    std::vector<std::pair<coord_t, coord_t>> cells;
    rect_work::Vec2d pt1;
    rect_work::Vec2d pt2;

    void init(const rect_work::Vec2d &first, const rect_work::Vec2d &second) {
        pt1 = first;
        pt2 = second;
    }
    bool operator()(coord_t iy, coord_t ix) {
        cells.emplace_back(iy, ix);
        return true;
    }
};

struct CoreTraversalVisitor
{
    std::vector<std::pair<coord_t, coord_t>> cells;
    Slic3r::Vec2d pt1;
    Slic3r::Vec2d pt2;

    void init(const Slic3r::Vec2d &first, const Slic3r::Vec2d &second) {
        pt1 = first;
        pt2 = second;
    }
    bool operator()(coord_t iy, coord_t ix) {
        cells.emplace_back(iy, ix);
        return true;
    }
};

} // namespace

TEST_CASE("Rectilinear edge grid port matches the core rasterization", "[plugins][infill][rectilinear][plugin-common]")
{
    const coord_t resolution = 500000;
    std::vector<Slic3r::Points> core_contours = make_core_contours();
    std::vector<rect_work::Points> ported_contours = make_ported_contours();

    Slic3r::EdgeGrid::Grid core_grid;
    rect_edge_grid::Grid ported_grid;
    core_grid.create(core_contours, resolution);
    ported_grid.create(ported_contours, resolution);

    CHECK(core_grid.bbox().min.x() == ported_grid.bbox().min.x());
    CHECK(core_grid.bbox().min.y() == ported_grid.bbox().min.y());
    CHECK(core_grid.bbox().max.x() == ported_grid.bbox().max.x());
    CHECK(core_grid.bbox().max.y() == ported_grid.bbox().max.y());
    REQUIRE(core_grid.resolution() == ported_grid.resolution());
    REQUIRE(core_grid.rows() == ported_grid.rows());
    REQUIRE(core_grid.cols() == ported_grid.cols());
    REQUIRE(core_grid.contours().size() == ported_grid.contours().size());

    // Every cell must reference the same (contour, segment) entries in the
    // same order.
    for (size_t row = 0; row < core_grid.rows(); ++row) {
        for (size_t col = 0; col < core_grid.cols(); ++col) {
            const auto core_range = core_grid.cell_data_range(coord_t(row), coord_t(col));
            const auto ported_range = ported_grid.cell_data_range(coord_t(row), coord_t(col));
            CHECK(core_grid.cell_has_data(coord_t(row), coord_t(col)) == ported_grid.cell_has_data(coord_t(row), coord_t(col)));
            REQUIRE(std::distance(core_range.first, core_range.second) ==
                    std::distance(ported_range.first, ported_range.second));
            auto core_it = core_range.first;
            auto ported_it = ported_range.first;
            for (; core_it != core_range.second; ++core_it, ++ported_it) {
                CHECK(core_it->first == ported_it->first);
                CHECK(core_it->second == ported_it->second);

                const auto core_segment = core_grid.segment(*core_it);
                const auto ported_segment = ported_grid.segment(*ported_it);
                CHECK(core_segment.first.x() == ported_segment.first.x);
                CHECK(core_segment.first.y() == ported_segment.first.y);
                CHECK(core_segment.second.x() == ported_segment.second.x);
                CHECK(core_segment.second.y() == ported_segment.second.y);
                const Slic3r::Line core_line = core_grid.line(*core_it);
                const rect_work::Line work_line = ported_grid.line(*ported_it);
                CHECK(core_line.a.x() == work_line.a.x());
                CHECK(core_line.a.y() == work_line.a.y());
                CHECK(core_line.b.x() == work_line.b.x());
                CHECK(core_line.b.y() == work_line.b.y());
            }
        }
    }

    // Half-open boxes at grid edges retain the core's clipped indices and visitation order.
    for (coord_t x : {-12000000, -8000016, -8000000, 0, 9000000, 14000000}) {
        const Slic3r::BoundingBox box({x, -6000000}, {x + 1000000, 1000000});
        const rect_work::BoundingBox work_box({x, -6000000}, {x + 1000000, 1000000});
        const Slic3r::BoundingBox expected = core_grid.get_cells_intersecting_box(box);
        const rect_work::BoundingBox actual = ported_grid.get_cells_intersecting_box(work_box);
        CHECK(actual.min.x() == expected.min.x()); CHECK(actual.min.y() == expected.min.y());
        CHECK(actual.max.x() == expected.max.x()); CHECK(actual.max.y() == expected.max.y());
        CoreTraversalVisitor core_visitor;
        TraversalVisitor work_visitor;
        core_grid.visit_cells_intersecting_box(box, core_visitor);
        ported_grid.visit_cells_intersecting_box(work_box, work_visitor);
        CHECK(core_visitor.cells == work_visitor.cells);
        std::vector<std::pair<coord_t, coord_t>> first_cells, second_cells;
        // Stop after two cells to exercise the visitor's early return.
        const auto first = [&first_cells](coord_t row, coord_t col) {
            first_cells.emplace_back(row, col); return first_cells.size() < 2;
        };
        const auto second = [&second_cells](coord_t row, coord_t col) {
            second_cells.emplace_back(row, col); return second_cells.size() < 2;
        };
        core_grid.visit_cells_intersecting_box(box, first);
        ported_grid.visit_cells_intersecting_box(work_box, second);
        CHECK(first_cells == second_cells);
    }

    // The traversal of a set of deterministic lines must visit the same cells.
    std::mt19937 rng(0x5eed9876u);
    for (int trial = 0; trial < 200; ++trial) {
        INFO("trial " << trial);
        const coord_t x0 = coord_t(int(rng() % 16000000) - 8000000);
        const coord_t y0 = coord_t(int(rng() % 13000000) - 6000000);
        const coord_t x1 = coord_t(int(rng() % 16000000) - 8000000);
        const coord_t y1 = coord_t(int(rng() % 13000000) - 6000000);

        CoreTraversalVisitor core_visitor;
        core_grid.visit_cells_intersecting_line(Slic3r::Point(x0, y0), Slic3r::Point(x1, y1), core_visitor);
        TraversalVisitor ported_visitor;
        ported_grid.visit_cells_intersecting_line(rect_work::Point(x0, y0), rect_work::Point(x1, y1), ported_visitor);

        REQUIRE(core_visitor.cells.size() == ported_visitor.cells.size());
        for (size_t idx = 0; idx < core_visitor.cells.size(); ++idx) {
            CHECK(core_visitor.cells[idx].first == ported_visitor.cells[idx].first);
            CHECK(core_visitor.cells[idx].second == ported_visitor.cells[idx].second);
        }
    }
}

TEST_CASE("Rectilinear edge grid port matches the core on an ExPolygon boundary", "[plugins][infill][rectilinear][plugin-common]")
{
    Slic3r::ExPolygon core_expolygon(Slic3r::Polygon({ Slic3r::Point(-10000000, -10000000), Slic3r::Point(10000000, -10000000),
                                                       Slic3r::Point(10000000, 10000000), Slic3r::Point(-10000000, 10000000) }));
    core_expolygon.holes.push_back(Slic3r::Polygon({ Slic3r::Point(-3000000, -3000000), Slic3r::Point(-3000000, 3000000),
                                                     Slic3r::Point(3000000, 3000000), Slic3r::Point(3000000, -3000000) }));
    const slic3r_api::ExPolygon ported_expolygon(
        reinterpret_cast<const expolygon_handle *>(&core_expolygon));

    const coord_t resolution = 10000000;
    Slic3r::EdgeGrid::Grid core_grid;
    rect_edge_grid::Grid ported_grid;
    // Reproduce the bounding box pre-set by the connect code.
    Slic3r::BoundingBox core_bbox = Slic3r::get_extents(core_expolygon.contour);
    core_bbox.offset(coordf_t(SCALED_EPSILON));
    rect_work::BoundingBox ported_bbox(
        rect_work::Point(core_bbox.min.x(), core_bbox.min.y()),
        rect_work::Point(core_bbox.max.x(), core_bbox.max.y()));
    ported_bbox.offset(coordf_t(SCALED_EPSILON));
    core_grid.set_bbox(core_bbox);
    ported_grid.set_bbox(ported_bbox);
    core_grid.create(core_expolygon, resolution);
    ported_grid.create(ported_expolygon, resolution);

    rect_work::ExPolygon copied;
    for (const Slic3r::Point &point : core_expolygon.contour.points)
        copied.contour.points.emplace_back(point.x(), point.y());
    for (const Slic3r::Polygon &hole : core_expolygon.holes) {
        rect_work::Polygon ring;
        for (const Slic3r::Point &point : hole.points) ring.points.emplace_back(point.x(), point.y());
        copied.holes.push_back(std::move(ring));
    }
    rect_edge_grid::Grid copied_grid;
    copied_grid.set_bbox(ported_bbox);
    copied_grid.create(copied, resolution);
    REQUIRE(copied_grid.rows() == core_grid.rows());
    REQUIRE(copied_grid.cols() == core_grid.cols());

    REQUIRE(core_grid.contours().size() == ported_grid.contours().size());
    REQUIRE(core_grid.rows() == ported_grid.rows());
    REQUIRE(core_grid.cols() == ported_grid.cols());

    for (size_t row = 0; row < core_grid.rows(); ++row) {
        for (size_t col = 0; col < core_grid.cols(); ++col) {
            const auto core_range = core_grid.cell_data_range(coord_t(row), coord_t(col));
            const auto ported_range = ported_grid.cell_data_range(coord_t(row), coord_t(col));
            const auto copied_range = copied_grid.cell_data_range(coord_t(row), coord_t(col));
            const std::vector<std::pair<size_t, size_t>> core_entries(core_range.first, core_range.second);
            const std::vector<std::pair<size_t, size_t>> copied_entries(copied_range.first, copied_range.second);
            CHECK(core_entries == copied_entries);
            REQUIRE(std::distance(core_range.first, core_range.second) ==
                    std::distance(ported_range.first, ported_range.second));
            auto core_it = core_range.first;
            auto ported_it = ported_range.first;
            for (; core_it != core_range.second; ++core_it, ++ported_it) {
                CHECK(core_it->first == ported_it->first);
                CHECK(core_it->second == ported_it->second);
            }
        }
    }

    // Query every infill end point used by the connect parity test, plus the
    // boundary vertices, with the same search radius as the connect code.
    std::vector<std::pair<coord_t, coord_t>> queries = {
        { -8000000, -9000000 }, { -8000000, 9000000 }, { -6000000, -9000000 }, { -6000000, 9000000 },
        { -4000000, -9000000 }, { -4000000, 9000000 }, { -2000000, -9000000 }, { -2000000, -3000000 },
        { -2000000, 3000000 }, { -2000000, 9000000 }, { 0, -9000000 }, { 0, -3000000 },
        { 0, 3000000 }, { 0, 9000000 }, { 2000000, -9000000 }, { 2000000, -3000000 },
        { 2000000, 3000000 }, { 2000000, 9000000 }, { 4000000, -9000000 }, { 4000000, 9000000 },
        { 6000000, -9000000 }, { 6000000, 9000000 }, { 8000000, -9000000 }, { 8000000, 9000000 },
        { -3000000, -3000000 }, { -3000000, 3000000 }, { 3000000, 3000000 }, { 3000000, -3000000 },
    };
    for (const std::pair<coord_t, coord_t> &query : queries) {
        INFO("query " << query.first << ", " << query.second);
        const Slic3r::EdgeGrid::Grid::ClosestPointResult core_result =
            core_grid.closest_point_signed_distance(Slic3r::Point(query.first, query.second), coord_t(SCALED_EPSILON));
        const rect_edge_grid::Grid::ClosestPointResult ported_result =
            ported_grid.closest_point_signed_distance(rect_work::Point(query.first, query.second), coord_t(SCALED_EPSILON));
        REQUIRE(core_result.valid() == ported_result.valid());
        if (core_result.valid()) {
            CHECK(core_result.contour_idx == ported_result.contour_idx);
            CHECK(core_result.start_point_idx == ported_result.start_point_idx);
            CHECK(core_result.t == ported_result.t);
            CHECK(core_result.distance == ported_result.distance);
        }
    }
}

TEST_CASE("Rectilinear edge grid port matches the core closest point query", "[plugins][infill][rectilinear][plugin-common]")
{
    const coord_t resolution = 500000;
    std::vector<Slic3r::Points> core_contours = make_core_contours();
    std::vector<rect_work::Points> ported_contours = make_ported_contours();

    Slic3r::EdgeGrid::Grid core_grid;
    rect_edge_grid::Grid ported_grid;
    core_grid.create(core_contours, resolution);
    ported_grid.create(ported_contours, resolution);

    std::vector<std::pair<coord_t, coord_t>> queries;
    // Contour vertices and segment midpoints.
    for (const Slic3r::Points &points : core_contours) {
        for (size_t idx = 0; idx < points.size(); ++idx) {
            const Slic3r::Point &p1 = points[idx];
            const Slic3r::Point &p2 = points[(idx + 1) % points.size()];
            queries.emplace_back(p1.x(), p1.y());
            queries.emplace_back(coord_t((p1.x() + p2.x()) / 2), coord_t((p1.y() + p2.y()) / 2));
        }
    }
    // Deterministic random queries.
    std::mt19937 rng(0x2468aceu);
    for (int trial = 0; trial < 500; ++trial)
        queries.emplace_back(coord_t(int(rng() % 18000000) - 9000000), coord_t(int(rng() % 15000000) - 7000000));

    for (const std::pair<coord_t, coord_t> &query : queries) {
        INFO("query " << query.first << ", " << query.second);
        const Slic3r::EdgeGrid::Grid::ClosestPointResult core_result =
            core_grid.closest_point_signed_distance(Slic3r::Point(query.first, query.second), SCALED_EPSILON);
        const rect_edge_grid::Grid::ClosestPointResult ported_result =
            ported_grid.closest_point_signed_distance(rect_work::Point(query.first, query.second), coord_t(SCALED_EPSILON));

        REQUIRE(core_result.valid() == ported_result.valid());
        if (core_result.valid()) {
            CHECK(core_result.contour_idx == ported_result.contour_idx);
            CHECK(core_result.start_point_idx == ported_result.start_point_idx);
            CHECK(core_result.t == ported_result.t);
            CHECK(core_result.distance == ported_result.distance);
        }
    }
}
