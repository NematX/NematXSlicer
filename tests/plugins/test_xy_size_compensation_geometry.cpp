#include <catch2/catch.hpp>

#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ElephantFootCompensation.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Plugins/ElephantFootCompensation.hpp"
#include "libslic3r/Plugins/XYSizeCompensationHelpers.hpp"
#include "libslic3r/libslic3r.h"

/*
These geometry tests call the SDK-only XY compensation core on small regional
masks. They check signed contour and hole offsets, ownership of new material,
no-collapse behavior, invalid inputs and parity with native elephant-foot output.
TestStorage owns the public result handles until each test finishes.

    TEST_CASE
    |-- compensate_layer(): compose islands and assign every piece to a region
    `-- elephant_foot_compensation(): compare SDK and native narrow-wall behavior
*/

namespace {

using namespace Slic3r;
using PolygonType = Slic3r::Polygon;
using slic3r_api::StoredExPolygonCollection;
using slic3r_api::XYSizeCompensationPlugin::detail::IslandInput;
using slic3r_api::XYSizeCompensationPlugin::detail::LayerOutput;
using slic3r_api::XYSizeCompensationPlugin::detail::RegionSettings;
using slic3r_api::XYSizeCompensationPlugin::detail::checked_scaled_delta;
using slic3r_api::XYSizeCompensationPlugin::detail::compensate_layer;

struct TestStorage
{
    PluginStorage native;
    storage_handle *get() { return reinterpret_cast<storage_handle *>(&native); }
};

coord_t mm(double value) { return scale_i(value); }

PolygonType rectangle(double min_x, double min_y, double max_x, double max_y)
{
    return PolygonType{ Point(mm(min_x), mm(min_y)), Point(mm(max_x), mm(min_y)),
                        Point(mm(max_x), mm(max_y)), Point(mm(min_x), mm(max_y)) };
}

ExPolygon rectangle_with_hole(double min_x = 0., double min_y = 0.,
                              double max_x = 20., double max_y = 20.,
                              double hole_min_x = 8., double hole_min_y = 8.,
                              double hole_max_x = 12., double hole_max_y = 12.)
{
    PolygonType hole{ Point(mm(hole_min_x), mm(hole_max_y)), Point(mm(hole_max_x), mm(hole_max_y)),
                      Point(mm(hole_max_x), mm(hole_min_y)), Point(mm(hole_min_x), mm(hole_min_y)) };
    return ExPolygon(rectangle(min_x, min_y, max_x, max_y), std::move(hole));
}

slic3r_api::ExPolygon sdk_view(const ExPolygon &expolygon)
{
    return slic3r_api::ExPolygon(reinterpret_cast<const expolygon_handle *>(&expolygon));
}

slic3r_api::ExPolygonCollection sdk_view(const ExPolygons &expolygons)
{
    return slic3r_api::ExPolygonCollection(reinterpret_cast<const expolygon_collection_handle *>(&expolygons));
}

const ExPolygons &native_result(const StoredExPolygonCollection &geometry)
{
    return *reinterpret_cast<const ExPolygons *>(geometry.handle());
}

double total_area(const ExPolygons &areas)
{
    double result = 0.;
    for (const ExPolygon &area : areas)
        result += std::abs(area.area());
    return result;
}

double total_area(const std::vector<StoredExPolygonCollection> &collections)
{
    double result = 0.;
    for (const StoredExPolygonCollection &collection : collections)
        result += total_area(native_result(collection));
    return result;
}

// A region as the tests describe it: its settings plus the part of one island it owns.
struct TestRegion
{
    RegionSettings settings;
    ExPolygons mask;
};

// One island and its regions, in layer region order.
struct TestIsland
{
    ExPolygon source;
    std::vector<TestRegion> regions;
};

TestRegion region(const ExPolygons &mask, double outer_mm = 0., double inner_mm = 0.,
                  bool no_collapse = false, double minimum_width_mm = 0.4)
{
    TestRegion result;
    result.mask = mask;
    result.settings.outer_delta = checked_scaled_delta(outer_mm);
    result.settings.inner_delta = checked_scaled_delta(inner_mm);
    result.settings.no_collapse = no_collapse;
    result.settings.minimum_contour_width_mm = minimum_width_mm;
    return result;
}

TestIsland one_region_island(const ExPolygon &source, TestRegion settings)
{
    return TestIsland{ source, { std::move(settings) } };
}

std::vector<TestIsland> island_list(TestIsland first)
{
    return { std::move(first) };
}

std::vector<TestIsland> island_list(TestIsland first, TestIsland second)
{
    return { std::move(first), std::move(second) };
}

// Convert the test description into compensate_layer() inputs. Settings are
// per layer region, so every island must use the same settings for a region.
LayerOutput run_layer(TestStorage &storage, const std::vector<TestIsland> &islands, std::size_t region_count)
{
    std::vector<RegionSettings> settings;
    std::vector<IslandInput> inputs;
    for (const TestIsland &island : islands) {
        REQUIRE(island.regions.size() == region_count);
        IslandInput input(storage.get());
        input.original.copy_from(sdk_view(island.source));
        for (std::size_t idx = 0; idx < region_count; ++idx) {
            const RegionSettings &region_settings = island.regions[idx].settings;
            if (settings.size() == idx) {
                settings.push_back(region_settings);
            } else {
                REQUIRE(settings[idx].outer_delta == region_settings.outer_delta);
                REQUIRE(settings[idx].inner_delta == region_settings.inner_delta);
                REQUIRE(settings[idx].no_collapse == region_settings.no_collapse);
            }
            input.region_masks.emplace_back(storage.get());
            input.region_masks.back().copy_from(sdk_view(island.regions[idx].mask));
        }
        inputs.push_back(std::move(input));
    }
    return compensate_layer(storage.get(), settings, inputs);
}

bool contains(const StoredExPolygonCollection &areas, double x, double y)
{
    return expolygons_contain(native_result(areas), Point(mm(x), mm(y)));
}

bool has_nearby_vertices(const PolygonType &ring)
{
    for (std::size_t idx = 0; idx < ring.size(); ++idx)
        if (ring.points[idx].coincides_with_epsilon(ring.points[(idx + 1) % ring.size()]))
            return true;
    return false;
}

// The host refuses region slices with consecutive points closer than its
// epsilon. The plugin relies on Clipper's output cleaning for this.
void check_publishable_regions(const LayerOutput &output)
{
    for (const StoredExPolygonCollection &region : output.regions)
        for (const ExPolygon &area : native_result(region)) {
            REQUIRE(area.contour.size() >= 3);
            REQUIRE_FALSE(has_nearby_vertices(area.contour));
            for (const PolygonType &hole : area.holes) {
                REQUIRE(hole.size() >= 3);
                REQUIRE_FALSE(has_nearby_vertices(hole));
            }
            area.assert_valid();
        }
}

void check_elephant_foot_parity(TestStorage &storage, const ExPolygon &source,
                              double minimum_width_mm, double compensation_mm)
{
    const ExPolygon expected = Slic3r::elephant_foot_compensation(source, minimum_width_mm, compensation_mm);
    const StoredExPolygonCollection actual = slic3r_api::XYSizeCompensationPlugin::detail::elephant_foot_compensation(
        storage.get(), sdk_view(source), minimum_width_mm, checked_scaled_delta(compensation_mm));
    REQUIRE(native_result(actual).size() == 1);
    CHECK(diff_ex(native_result(actual), ExPolygons{ expected }).empty());
    CHECK(diff_ex(ExPolygons{ expected }, native_result(actual)).empty());
}

} // namespace

TEST_CASE("XY compensation preserves neutral geometry and separates outside from hole movement", "[plugins][xy-size-compensation]")
{
    TestStorage storage;
    const ExPolygon source = rectangle_with_hole();
    const ExPolygons source_collection{ source };

    LayerOutput neutral = run_layer(storage,
        island_list(one_region_island(source, region(source_collection))), 1);
    REQUIRE(total_area(native_result(neutral.slices)) == Approx(std::abs(source.area())));
    REQUIRE(native_result(neutral.slices).size() == 1);
    REQUIRE(native_result(neutral.slices).front().holes.size() == 1);

    LayerOutput outside = run_layer(storage,
        island_list(one_region_island(source, region(source_collection, 0.5))), 1);
    BoundingBox outside_bounds = get_extents(native_result(outside.slices));
    REQUIRE(outside_bounds.min.x() == mm(-0.5));
    REQUIRE(outside_bounds.max.x() == mm(20.5));
    REQUIRE(native_result(outside.slices).front().holes.size() == 1);
    BoundingBox outside_hole = get_extents(native_result(outside.slices).front().holes.front());
    REQUIRE(outside_hole.min.x() == mm(8.));
    REQUIRE(outside_hole.max.x() == mm(12.));

    LayerOutput hole_smaller = run_layer(storage,
        island_list(one_region_island(source, region(source_collection, 0., 0.5))), 1);
    BoundingBox hole_smaller_bounds = get_extents(native_result(hole_smaller.slices));
    REQUIRE(hole_smaller_bounds.min.x() == 0);
    REQUIRE(hole_smaller_bounds.max.x() == mm(20.));
    REQUIRE(native_result(hole_smaller.slices).front().holes.size() == 1);
    BoundingBox smaller_hole = get_extents(native_result(hole_smaller.slices).front().holes.front());
    REQUIRE(smaller_hole.min.x() == mm(8.5));
    REQUIRE(smaller_hole.max.x() == mm(11.5));

    LayerOutput hole_larger = run_layer(storage,
        island_list(one_region_island(source, region(source_collection, 0., -0.5))), 1);
    BoundingBox larger_hole = get_extents(native_result(hole_larger.slices).front().holes.front());
    REQUIRE(larger_hole.min.x() == mm(7.5));
    REQUIRE(larger_hole.max.x() == mm(12.5));
}

TEST_CASE("XY compensation applies ordered negative levels across three regions", "[plugins][xy-size-compensation]")
{
    TestStorage storage;
    const ExPolygon source = rectangle_with_hole(0., 0., 30., 10., 14., 3., 16., 7.);
    const ExPolygons source_geometry{ source };
    const ExPolygon left = intersection_ex(source_geometry, ExPolygons{ ExPolygon(rectangle(0., 0., 10., 10.)) }).front();
    const ExPolygon middle = intersection_ex(source_geometry, ExPolygons{ ExPolygon(rectangle(10., 0., 20., 10.)) }).front();
    const ExPolygon right = intersection_ex(source_geometry, ExPolygons{ ExPolygon(rectangle(20., 0., 30., 10.)) }).front();
    TestIsland island{ source, {} };
    island.regions.push_back(region(ExPolygons{ left }, -1.));
    island.regions.push_back(region(ExPolygons{ middle }, -2.));
    island.regions.push_back(region(ExPolygons{ right }, -3.));

    LayerOutput result = run_layer(storage, island_list(std::move(island)), 3);
    const BoundingBox bounds = get_extents(native_result(result.slices));
    REQUIRE(bounds.min.x() == mm(1.));
    REQUIRE(bounds.max.x() == mm(27.));
    REQUIRE(result.regions.size() == 3);
    REQUIRE(total_area(result.regions) == Approx(total_area(native_result(result.slices))));
    REQUIRE(total_area(native_result(result.slices)) < std::abs(source.area()));
}

TEST_CASE("XY positive inner growth is assigned when a hole crosses a region boundary", "[plugins][xy-size-compensation]")
{
    TestStorage storage;
    const ExPolygon source = rectangle_with_hole(0., 0., 20., 20., 9., 6., 11., 14.);
    const ExPolygons source_geometry{ source };
    const ExPolygons left = intersection_ex(source_geometry, ExPolygons{ ExPolygon(rectangle(0., 0., 10., 20.)) });
    const ExPolygons right = intersection_ex(source_geometry, ExPolygons{ ExPolygon(rectangle(10., 0., 20., 20.)) });
    TestIsland island{ source, {} };
    island.regions.push_back(region(left, 0., 1.));
    island.regions.push_back(region(right, 0., 1.));

    LayerOutput result = run_layer(storage, island_list(std::move(island)), 2);
    REQUIRE(native_result(result.slices).size() == 1);
    REQUIRE(native_result(result.slices).front().holes.empty());
    REQUIRE(total_area(native_result(result.slices)) == Approx(mm(20.) * mm(20.)));
    REQUIRE((contains(result.regions[0], 10., 10.) || contains(result.regions[1], 10., 10.)));
}

TEST_CASE("XY positive growth ownership uses the applicable outer or inner delta", "[plugins][xy-size-compensation]")
{
    TestStorage storage;
    const ExPolygon source = rectangle_with_hole(0., 0., 20., 20., 9., 6., 11., 14.);
    const ExPolygons source_geometry{ source };
    const ExPolygons left = intersection_ex(source_geometry, ExPolygons{ ExPolygon(rectangle(0., 0., 10., 20.)) });
    const ExPolygons right = intersection_ex(source_geometry, ExPolygons{ ExPolygon(rectangle(10., 0., 20., 20.)) });
    TestIsland island{ source, {} };
    // Region 0 has much stronger outside growth but weaker hole filling. Its C must not win I growth.
    island.regions.push_back(region(left, 10., 0.5));
    island.regions.push_back(region(right, 0., 1.));

    LayerOutput result = run_layer(storage, island_list(std::move(island)), 2);
    REQUIRE(contains(result.regions[1], 10., 10.));
    REQUIRE_FALSE(contains(result.regions[0], 10., 10.));
}

TEST_CASE("XY inner frame ignores a huge inactive region and remains local to distant geometry", "[plugins][xy-size-compensation]")
{
    TestStorage storage;
    const ExPolygon source = rectangle_with_hole(1000000., 1000000., 1000020., 1000020.,
                                                 1000008., 1000008., 1000012., 1000012.);
    TestIsland island{ source, {} };
    island.regions.push_back(region(ExPolygons{ source }, 1., 0., true));
    TestRegion inactive = region({}, 0., 0., true, 0.);
    inactive.settings.inner_delta = std::numeric_limits<coord_t>::max() / 2;
    island.regions.push_back(std::move(inactive));

    LayerOutput with_inactive = run_layer(storage, island_list(std::move(island)), 2);
    LayerOutput without_inactive = run_layer(storage,
        island_list(one_region_island(source, region(ExPolygons{ source }, 1., 0., true))), 1);
    // Compare material directly: native Polygon::area() loses precision at
    // these distant coordinates and triggers its Debug consistency assertion.
    REQUIRE(diff_ex(native_result(with_inactive.slices), native_result(without_inactive.slices)).empty());
    REQUIRE(diff_ex(native_result(without_inactive.slices), native_result(with_inactive.slices)).empty());
    REQUIRE(get_extents(native_result(with_inactive.slices)) == get_extents(native_result(without_inactive.slices)));
    REQUIRE_THROWS_AS(checked_scaled_delta(std::numeric_limits<double>::infinity()), std::invalid_argument);
    REQUIRE_THROWS_AS(checked_scaled_delta(1e20), std::out_of_range);
}

TEST_CASE("XY offsets reject miter growth beyond Clipper's coordinate range", "[plugins][xy-size-compensation]")
{
    TestStorage storage;
    const coord_t delta = mm(1.);
    const coord_t limit = static_cast<coord_t>(ClipperLib::hiRange);
    const coord_t max_x = limit - 2 * delta;
    const PolygonType contour{ Point(max_x - 10 * delta, 0), Point(max_x, 0),
                               Point(max_x, 10 * delta), Point(max_x - 10 * delta, 10 * delta) };
    const ExPolygon source(contour);
    REQUIRE_THROWS_AS(run_layer(storage,
        island_list(one_region_island(source, region(ExPolygons{ source }, 1.))), 1), std::out_of_range);
}

TEST_CASE("XY coordinate limits are checked after rounding and before miter offsets", "[plugins][xy-size-compensation]")
{
    TestStorage storage;
    REQUIRE_THROWS_AS(checked_scaled_delta(unscaled(double(ClipperLib::hiRange))), std::out_of_range);

    const coord_t limit = static_cast<coord_t>(ClipperLib::hiRange);
    const coord_t max_x = limit - 2;
    const coord_t width = 4096;
    const PolygonType contour{ Point(max_x - width, 0), Point(max_x, 0),
                               Point(max_x, width), Point(max_x - width, width) };
    const ExPolygon source(contour);
    TestRegion edge_region;
    edge_region.mask = ExPolygons{ source };
    edge_region.settings.outer_delta = 1;
    REQUIRE_THROWS_WITH(run_layer(storage,
        island_list(one_region_island(source, std::move(edge_region))), 1),
        "XY offset exceeds Clipper coordinate limits.");
}

TEST_CASE("XY region overlap accepts only the host's insignificant overlap tolerance", "[plugins][xy-size-compensation]")
{
    TestStorage storage;
    const coord_t ten_mm = mm(10.);
    const auto scaled_rectangle = [](coord_t min_x, coord_t min_y, coord_t max_x, coord_t max_y) {
        return PolygonType{ Point(min_x, min_y), Point(max_x, min_y),
                            Point(max_x, max_y), Point(min_x, max_y) };
    };
    const auto island_with_region_overlap = [&](coord_t overlap_side) {
        const ExPolygon first(scaled_rectangle(0, 0, ten_mm, ten_mm));
        const coord_t second_min = mm(5.);
        const ExPolygon second(scaled_rectangle(second_min, second_min,
                                                second_min + overlap_side, second_min + overlap_side));
        const ExPolygons original_islands = union_ex(ExPolygons{ first, second });
        REQUIRE(original_islands.size() == 1);
        TestIsland island{ original_islands.front(), {} };
        island.regions.push_back(region(ExPolygons{ first }, 0.1));
        island.regions.push_back(region(ExPolygons{ second }, 0.1));
        return island;
    };

    const coord_t overlap_side = 2 * SCALED_EPSILON;
    TestIsland within_host_tolerance = island_with_region_overlap(overlap_side);
    const double overlap_area = total_area(intersection_ex(
        within_host_tolerance.regions[0].mask,
        within_host_tolerance.regions[1].mask));
    REQUIRE(overlap_area == double(overlap_side) * double(overlap_side));
    REQUIRE(overlap_area <= 10. * double(SCALED_EPSILON) * double(SCALED_EPSILON));
    const LayerOutput tolerated = run_layer(storage, island_list(std::move(within_host_tolerance)), 2);
    REQUIRE(intersection_ex(native_result(tolerated.regions[0]), native_result(tolerated.regions[1])).empty());

    TestIsland one_square_mm_overlap = island_with_region_overlap(mm(1.));
    REQUIRE(total_area(intersection_ex(one_square_mm_overlap.regions[0].mask,
                                       one_square_mm_overlap.regions[1].mask)) == Approx(mm(1.) * mm(1.)));
#ifndef NDEBUG
    // Input mask validation is debug-only; release relies on the host's validation.
    REQUIRE_THROWS_AS(run_layer(storage, island_list(std::move(one_square_mm_overlap)), 2), std::runtime_error);
#else
    // Release still assigns the overlap to a single region.
    const LayerOutput overlapping = run_layer(storage, island_list(std::move(one_square_mm_overlap)), 2);
    REQUIRE(intersection_ex(native_result(overlapping.regions[0]), native_result(overlapping.regions[1])).empty());
#endif
}

TEST_CASE("XY layer composition preserves islands inside holes and merges growing neighbors", "[plugins][xy-size-compensation]")
{
    TestStorage storage;
    const ExPolygon ring = rectangle_with_hole(0., 0., 20., 20., 5., 5., 15., 15.);
    const ExPolygon nested(rectangle(8., 8., 12., 12.));
    std::vector<TestIsland> nested_islands;
    nested_islands.push_back(one_region_island(ring, region(ExPolygons{ ring }, -0.2, 0.)));
    nested_islands.push_back(one_region_island(nested, region(ExPolygons{ nested }, -0.2, 0.)));
    LayerOutput nested_result = run_layer(storage, std::move(nested_islands), 1);
    REQUIRE(native_result(nested_result.slices).size() == 2);
    REQUIRE(contains(nested_result.slices, 10., 10.));

    const ExPolygon first(rectangle(0., 0., 5., 5.));
    const ExPolygon second(rectangle(5.5, 0., 10.5, 5.));
    LayerOutput merged = run_layer(storage, island_list(
        one_region_island(first, region(ExPolygons{ first }, 0.3)),
        one_region_island(second, region(ExPolygons{ second }, 0.3))), 1);
    REQUIRE(native_result(merged.slices).size() == 1);
}

TEST_CASE("XY no-collapse protects a narrow first-layer tongue only in enabled regions", "[plugins][xy-size-compensation]")
{
    TestStorage storage;
    PolygonType tab_contour{
        Point(mm(0.), mm(0.)), Point(mm(10.), mm(0.)), Point(mm(10.), mm(10.)),
        Point(mm(5.2), mm(10.)), Point(mm(5.2), mm(12.)), Point(mm(4.8), mm(12.)),
        Point(mm(4.8), mm(10.)), Point(mm(0.), mm(10.))
    };
    const ExPolygon tab(tab_contour);
    LayerOutput ordinary = run_layer(storage, island_list(one_region_island(tab,
        region(ExPolygons{ tab }, -0.3, 0., false, 0.4))), 1);
    LayerOutput protected_result = run_layer(storage, island_list(one_region_island(tab,
        region(ExPolygons{ tab }, -0.3, 0., true, 0.4))), 1);
    REQUIRE_FALSE(contains(ordinary.slices, 5., 11.5));
    REQUIRE(contains(protected_result.slices, 5., 11.5));

    PolygonType side_tab_contour{
        Point(mm(0.), mm(0.)), Point(mm(10.), mm(0.)), Point(mm(10.), mm(4.8)),
        Point(mm(12.), mm(4.8)), Point(mm(12.), mm(5.2)), Point(mm(10.), mm(5.2)),
        Point(mm(10.), mm(10.)), Point(mm(0.), mm(10.))
    };
    const ExPolygon side_tab(side_tab_contour);
    const ExPolygons enabled_mask = intersection_ex(ExPolygons{ side_tab },
        ExPolygons{ ExPolygon(rectangle(0., 0., 10., 10.)) });
    const ExPolygons disabled_mask = intersection_ex(ExPolygons{ side_tab },
        ExPolygons{ ExPolygon(rectangle(10., 0., 12., 10.)) });
    TestIsland regional{ side_tab, {} };
    regional.regions.push_back(region(enabled_mask, -0.3, 0., true, 0.4));
    regional.regions.push_back(region(disabled_mask, -0.3, 0., false, 0.4));
    LayerOutput mixed = run_layer(storage, island_list(std::move(regional)), 2);
    REQUIRE_FALSE(contains(mixed.slices, 11., 5.));
    REQUIRE(contains(mixed.slices, 5., 5.));
}

TEST_CASE("XY no-collapse preserves a narrow wall between the outer contour and a hole", "[plugins][xy-size-compensation]")
{
    TestStorage storage;
    const ExPolygon ring = rectangle_with_hole(0., 0., 10., 10., 0.4, 0.4, 9.6, 9.6);
    const ExPolygons source{ ring };
    const LayerOutput ordinary = run_layer(storage, island_list(one_region_island(ring,
        region(source, -0.5, 0., false, 0.8))), 1);
    REQUIRE(native_result(ordinary.slices).empty());

    const LayerOutput protected_result = run_layer(storage, island_list(one_region_island(ring,
        region(source, -0.5, 0., true, 0.8))), 1);
    REQUIRE_FALSE(native_result(protected_result.slices).empty());
    REQUIRE(total_area(native_result(protected_result.slices)) > 0.);
}

TEST_CASE("SDK-only no-collapse helper matches the active native Elephant Foot path", "[plugins][xy-size-compensation][no-collapse-parity]")
{
    TestStorage storage;
    const ExPolygon square = ExPolygon(rectangle(0., 0., 20., 20.));
    check_elephant_foot_parity(storage, square, 0.8, 0.3);

    const ExPolygon thin_ring = rectangle_with_hole(0., 0., 10., 10., 0.4, 0.4, 9.6, 9.6);
    check_elephant_foot_parity(storage, thin_ring, 0.8, 0.5);

    PolygonType concave_contour{ Point(mm(0.), mm(0.)), Point(mm(20.), mm(0.)), Point(mm(20.), mm(6.)),
                                 Point(mm(8.), mm(6.)), Point(mm(8.), mm(20.)), Point(mm(0.), mm(20.)) };
    check_elephant_foot_parity(storage, ExPolygon(concave_contour), 0.8, 0.3);

    ExPolygon nearby_holes(rectangle(0., 0., 30., 16.));
    nearby_holes.holes.push_back(rectangle(5., 5., 10., 11.));
    nearby_holes.holes.push_back(rectangle(12., 5., 17., 11.));
    for (PolygonType &hole : nearby_holes.holes)
        hole.make_clockwise();
    check_elephant_foot_parity(storage, nearby_holes, 0.6, 0.25);

    PolygonType tight_vertices{ Point(mm(0.), mm(0.)), Point(mm(20.), mm(0.)), Point(mm(20.), mm(20.)),
                                Point(mm(10.1), mm(20.)), Point(mm(10.), mm(19.9)),
                                Point(mm(9.9), mm(20.)), Point(mm(0.), mm(20.)) };
    check_elephant_foot_parity(storage, ExPolygon(tight_vertices), 0.8, 0.2);
}


TEST_CASE("No-collapse still compensates a narrow triangle prone to close offset points",
          "[plugins][xy-size-compensation][nearby-vertices]")
{
    TestStorage storage;
    const ExPolygon source(PolygonType{ Point(mm(18.), mm(7.)), Point(mm(22.), mm(3.)),
                                         Point(mm(27.), mm(12.)) });
    const ExPolygon before = source;
    const auto actual = slic3r_api::XYSizeCompensationPlugin::detail::elephant_foot_compensation(
        storage.get(), sdk_view(source), 0.673, checked_scaled_delta(0.712));
    REQUIRE(native_result(actual).size() == 1);
    const ExPolygon &area = native_result(actual).front();
    // With these settings the offset kernel can produce two very close points.
    // They must not make the island fall back to no compensation at all.
    CHECK(area.contour.is_counter_clockwise());
    CHECK(area.holes.empty());
    CHECK(std::abs(area.area()) < std::abs(source.area()));
    const BoundingBox source_bounds = get_extents(source);
    const BoundingBox result_bounds = get_extents(area);
    CHECK(result_bounds.min.x() > source_bounds.min.x());
    CHECK(result_bounds.min.y() > source_bounds.min.y());
    CHECK(result_bounds.max.x() < source_bounds.max.x());
    CHECK(result_bounds.max.y() < source_bounds.max.y());
    CHECK(source.contour.points == before.contour.points);

    const LayerOutput output = run_layer(storage, island_list(one_region_island(source,
        region(ExPolygons{ source }, -0.712, 0., true, 0.673))), 1);
    REQUIRE_FALSE(native_result(output.regions.front()).empty());
    CHECK(total_area(output.regions) < std::abs(source.area()));
    check_publishable_regions(output);
}

TEST_CASE("No-collapse accepts nearby input vertices and handles empty or degenerate input",
          "[plugins][xy-size-compensation][nearby-vertices]")
{
    TestStorage storage;
    const ExPolygon source(PolygonType{ Point(0, 0), Point(1, 1), Point(mm(20.), 0),
                                         Point(mm(20.), mm(20.)), Point(0, mm(20.)) });
    REQUIRE(has_nearby_vertices(source.contour));
    const ExPolygon before = source;
    const auto protected_area = slic3r_api::XYSizeCompensationPlugin::detail::elephant_foot_compensation(
        storage.get(), sdk_view(source), 0.8, checked_scaled_delta(0.3));
    REQUIRE(protected_area.size() == 1);
    CHECK(total_area(native_result(protected_area)) < std::abs(source.area()));
    const LayerOutput output = run_layer(storage, island_list(one_region_island(source,
        region(ExPolygons{ source }, -0.3, 0., true, 0.8))), 1);
    REQUIRE_FALSE(native_result(output.regions.front()).empty());
    CHECK(total_area(output.regions) < std::abs(source.area()));
    check_publishable_regions(output);
    CHECK(source.contour.points == before.contour.points);

    const ExPolygon empty;
    CHECK(slic3r_api::XYSizeCompensationPlugin::detail::elephant_foot_compensation(
        storage.get(), sdk_view(empty), 0.8, checked_scaled_delta(0.3)).empty());
    CHECK_THROWS_AS(slic3r_api::XYSizeCompensationPlugin::detail::elephant_foot_compensation(
        nullptr, sdk_view(empty), 0.8, checked_scaled_delta(0.3)), std::invalid_argument);

    // Degenerate rings used to be rejected as invalid arguments; they are now
    // returned unchanged. Fill the rings directly: Polygon constructors assert
    // on some degenerate point lists.
    std::vector<ExPolygon> degenerate(3);
    degenerate[0].contour.points = { Point(0, 0), Point(mm(0.1), 0) };
    degenerate[1].contour.points = { Point(0, 0), Point(mm(20.), mm(20.)) };
    degenerate[2].contour.points = rectangle(0., 0., 20., 20.).points;
    degenerate[2].holes.resize(1);
    for (const ExPolygon &input : degenerate) {
        const auto result = slic3r_api::XYSizeCompensationPlugin::detail::elephant_foot_compensation(
            storage.get(), sdk_view(input), 0.8, checked_scaled_delta(0.3));
        REQUIRE(native_result(result).size() == 1);
        CHECK(native_result(result).front().contour.points == input.contour.points);
        CHECK(native_result(result).front().holes.size() == input.holes.size());
    }
}

TEST_CASE("XY neighboring regions along a long almost straight border stay within the host overlap tolerance",
          "[plugins][xy-size-compensation][nearby-vertices]")
{
    TestStorage storage;
    const ExPolygon source(rectangle(0., 0., 20., 40.));
    // Both masks describe exactly the same shared border, in opposite order.
    // Each region is computed on its own; over a long border, even a
    // microscopic disagreement could exceed the host's overlap tolerance.
    const Points border{ Point(mm(10.), 0),
                         Point(mm(10.) + 49, mm(10.)), Point(mm(10.) - 49, mm(20.)),
                         Point(mm(10.) + 49, mm(30.)), Point(mm(10.), mm(40.)) };
    Points left_points{ Point(0, 0) };
    left_points.insert(left_points.end(), border.begin(), border.end());
    left_points.emplace_back(0, mm(40.));
    Points right_points{ border.front(), Point(mm(20.), 0), Point(mm(20.), mm(40.)) };
    right_points.insert(right_points.end(), border.rbegin(), border.rend() - 1);
    TestIsland island{ source, {} };
    island.regions.push_back(region(ExPolygons{ ExPolygon(PolygonType(left_points)) }, -0.2));
    island.regions.push_back(region(ExPolygons{ ExPolygon(PolygonType(right_points)) }, -0.3));
    // Check the fixture in release too: Polygon moves may discard a near-closing
    // vertex, which would make two supposedly shared borders inconsistent.
    REQUIRE(intersection_ex(island.regions[0].mask, island.regions[1].mask).empty());
    ExPolygons masks = island.regions[0].mask;
    append(masks, island.regions[1].mask);
    REQUIRE(diff_ex(ExPolygons{ source }, union_ex(masks)).empty());
    const LayerOutput output = run_layer(storage, island_list(std::move(island)), 2);
    REQUIRE(output.regions.size() == 2);
    REQUIRE_FALSE(native_result(output.regions[0]).empty());
    REQUIRE_FALSE(native_result(output.regions[1]).empty());
    check_publishable_regions(output);
    CHECK(total_area(intersection_ex(native_result(output.regions[0]),
                                     native_result(output.regions[1]))) <=
        slic3r_api::XYSizeCompensationPlugin::detail::k_insignificant_area);
}

TEST_CASE("XY assigns a significant regional miter remainder without losing material",
          "[plugins][xy-size-compensation][regional-fallback]")
{
    TestStorage storage;
    const ExPolygon source(PolygonType{ Point(mm(6.), mm(6.)), Point(mm(15.), mm(1.)),
                                         Point(mm(24.), mm(6.)) });
    const ExPolygons left = intersection_ex(ExPolygons{ source },
        ExPolygons{ ExPolygon(rectangle(-30., -30., 15., 60.)) });
    const ExPolygons right = diff_ex(ExPolygons{ source }, left);
    const std::vector<TestIsland> input = island_list(TestIsland{ source,
        { region(left, 1.), region(right, -0.5) } });

    // The new acute corner at the regional boundary grows beyond the whole
    // island's miter envelope. Its unclaimed triangle has area 5/9 mm2;
    // this is an accepted boundary artifact, not a rounding sliver.
    const LayerOutput output = run_layer(storage, input, 2);
    CHECK(output.fallback_area > slic3r_api::XYSizeCompensationPlugin::detail::k_insignificant_area);
    CHECK(unscaled(unscaled(output.fallback_area)) == Approx(5. / 9.).margin(0.00001));

    const ExPolygons expected = union_ex(
        offset_ex(ExPolygons{ source }, double(mm(-0.5)), ClipperLib::jtMiter, 3.),
        offset_ex(left, double(mm(1.)), ClipperLib::jtMiter, 3.));
    CHECK(diff_ex(native_result(output.slices), expected).empty());
    CHECK(diff_ex(expected, native_result(output.slices)).empty());
    REQUIRE(output.regions.size() == 2);
    const ExPolygons assigned = union_ex(native_result(output.regions[0]), native_result(output.regions[1]));
    CHECK(diff_ex(expected, assigned).empty());
    CHECK(diff_ex(assigned, expected).empty());
    CHECK(intersection_ex(native_result(output.regions[0]), native_result(output.regions[1])).empty());
    CHECK(contains(output.slices, 15.8, -0.3));
    CHECK(contains(output.regions[0], 15.8, -0.3));
    CHECK_FALSE(contains(output.regions[1], 15.8, -0.3));
    check_publishable_regions(output);

    // Repeating the same input must keep both the fallback amount and ownership.
    const LayerOutput repeated = run_layer(storage, input, 2);
    CHECK(repeated.fallback_area == output.fallback_area);
    for (std::size_t idx = 0; idx < output.regions.size(); ++idx) {
        CHECK(diff_ex(native_result(output.regions[idx]), native_result(repeated.regions[idx])).empty());
        CHECK(diff_ex(native_result(repeated.regions[idx]), native_result(output.regions[idx])).empty());
    }
}
