#include <catch2/catch.hpp>

#include <algorithm>
#include <utility>
#include <vector>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Plugins/Common/ClipperHelpers.hpp"

#include "libslic3r/ExPolygon.hpp"

/*
Common polygon operation checks
===============================

The core is the existing Slic3r implementation.
Expose a core polygon with a hole through an API view, then compare direct-view
offset and shrink results to operations on copied work rings. Sorted vertex
signatures compare point sets per ring and ignore both vertex and ring order;
they do not check edge connectivity or orientation.
API collections borrow stored geometry; work polygons own their point arrays.
Conversion checks compare every coordinate in stored ring and point order, then
verify that output survives explicit freeing and destruction of the storage.

Tests
`-- API view versus work rings: offset and shrink, compared by sorted signatures
    |-- make_boundary / view: provide the same geometry to both operation paths
    `-- signatures: sort vertices within each ring, then sort the ring signatures
`-- ordered conversion and ownership
    |-- to_polygons: copy contours and holes from live API views
    |-- check_exact_view: ensure the source has not changed
    `-- check_exact_polygons: compare owned output before and after freeing
*/

namespace rect_work = slic3r_api::PluginCommon::polygon_work;
namespace rect_clipper = slic3r_api::PluginCommon::clipper;

namespace {

// Compare owned work points with core points in stored order; neither input changes.
void check_exact_polygons(const rect_work::Polygons &actual, const Slic3r::Polygons &expected);
// Compare a live borrowed ring with every core coordinate without conversion.
void check_exact_view(const slic3r_api::Polygon &actual, const Slic3r::Polygon &expected);

slic3r_api::ExPolygon view(const Slic3r::ExPolygon &expolygon)
{
    return slic3r_api::ExPolygon(reinterpret_cast<const expolygon_handle *>(&expolygon));
}

Slic3r::ExPolygon make_boundary()
{
    Slic3r::ExPolygon out(Slic3r::Polygon({ Slic3r::Point(-10000000, -8000000), Slic3r::Point(12000000, -8000000),
                                             Slic3r::Point(12000000, 9000000), Slic3r::Point(-10000000, 9000000) }));
    out.holes.push_back(Slic3r::Polygon({ Slic3r::Point(-3000000, -2000000), Slic3r::Point(-3000000, 3000000),
                                          Slic3r::Point(4000000, 3000000), Slic3r::Point(4000000, -2000000) }));
    return out;
}

rect_work::Polygons as_polygons(const rect_work::ExPolygon &expolygon)
{
    rect_work::Polygons out;
    out.push_back(expolygon.contour);
    for (const rect_work::Polygon &hole : expolygon.holes)
        out.push_back(hole);
    return out;
}

using PolygonSignature = std::vector<std::pair<coord_t, coord_t>>;

std::vector<PolygonSignature> signatures(const rect_work::Polygons &polygons)
{
    std::vector<PolygonSignature> out;
    out.reserve(polygons.size());
    for (const rect_work::Polygon &polygon : polygons) {
        PolygonSignature signature;
        signature.reserve(polygon.points.size());
        for (const rect_work::Point &point : polygon.points)
            signature.emplace_back(point.x(), point.y());
        std::sort(signature.begin(), signature.end());
        out.push_back(std::move(signature));
    }
    std::sort(out.begin(), out.end());
    return out;
}

void check_exact_polygons(const rect_work::Polygons &actual, const Slic3r::Polygons &expected)
{
    REQUIRE(actual.size() == expected.size());
    for (size_t polygon_idx = 0; polygon_idx < expected.size(); ++polygon_idx) {
        REQUIRE(actual[polygon_idx].points.size() == expected[polygon_idx].points.size());
        for (size_t point_idx = 0; point_idx < expected[polygon_idx].points.size(); ++point_idx) {
            CHECK(actual[polygon_idx].points[point_idx].x() == expected[polygon_idx].points[point_idx].x());
            CHECK(actual[polygon_idx].points[point_idx].y() == expected[polygon_idx].points[point_idx].y());
        }
    }
}

void check_exact_view(const slic3r_api::Polygon &actual, const Slic3r::Polygon &expected)
{
    REQUIRE(actual.size() == expected.points.size());
    for (uint32_t idx = 0; idx < actual.size(); ++idx) {
        const c_point point = actual.at(idx);
        CHECK(point.x == expected.points[idx].x());
        CHECK(point.y == expected.points[idx].y());
    }
}

} // namespace

TEST_CASE("Rectilinear clipper offsets an API ExPolygon without source materialization",
          "[plugins][infill][rectilinear][plugin-common]")
{
    Slic3r::ExPolygon core_boundary = make_boundary();
    const slic3r_api::ExPolygon api_boundary = view(core_boundary);

    Slic3r::PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    const rect_work::ExPolygon value_boundary = rect_clipper::from_expolygon_view(api_boundary);
    const rect_work::Polygons value_source = as_polygons(value_boundary);

    const rect_work::Polygons direct_offset = rect_clipper::offset(storage, api_boundary, 250000.);
    const rect_work::Polygons value_offset = rect_clipper::offset(storage, value_source, 250000.);
    REQUIRE(signatures(direct_offset) == signatures(value_offset));

    const rect_work::Polygons direct_shrink = rect_clipper::shrink(storage, api_boundary, 250000.);
    const rect_work::Polygons value_shrink = rect_clipper::shrink(storage, value_source, 250000.);
    CHECK(signatures(direct_shrink) == signatures(value_shrink));
}

// Convert a live empty stored collection, then free it; owned empty output must survive storage destruction or Catch fails.
TEST_CASE("Rectilinear polygon conversion owns the result of an empty stored collection",
          "[plugins][infill][rectilinear][rectilinear-hole-view]")
{
    rect_work::Polygons output;
    {
        Slic3r::PluginStorage plugin_storage;
        storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
        slic3r_api::StoredExPolygonCollection stored(storage);
        REQUIRE(stored.size() == 0);
        output = rect_clipper::to_polygons(stored.readonly());
        REQUIRE(stored.size() == 0);
        CHECK(output.empty());
        REQUIRE(stored.free_from_storage());
    }
    CHECK(output.empty());
}

// Copy explicit core rings into live storage and convert them; exact order checks detect changes and output owns points after freeing storage.
TEST_CASE("Rectilinear polygon conversion preserves ordered contours and holes after freeing the source",
          "[plugins][infill][rectilinear][rectilinear-hole-view]")
{
    const Slic3r::ExPolygons originals = {
        Slic3r::ExPolygon(Slic3r::Polygon({{90, 80}, {10, 80}, {10, 10}, {90, 10}}),
                         Slic3r::Polygons{Slic3r::Polygon({{35, 25}, {25, 20}, {20, 35}}),
                                           Slic3r::Polygon({{75, 60}, {75, 45}, {60, 45}, {55, 55}, {60, 65}})}),
        Slic3r::ExPolygon(Slic3r::Polygon({{190, 70}, {150, 95}, {110, 70}, {120, 15}, {180, 15}})),
        Slic3r::ExPolygon(Slic3r::Polygon({{290, 75}, {260, 95}, {220, 85}, {210, 45}, {230, 10}, {285, 20}}),
                         Slic3r::Polygons{Slic3r::Polygon({{270, 65}, {275, 40}, {240, 35}, {235, 60}})})
    };
    Slic3r::Polygons expected;
    for (const Slic3r::ExPolygon &original : originals) {
        expected.push_back(original.contour);
        for (const Slic3r::Polygon &hole : original.holes)
            expected.push_back(hole);
    }
    rect_work::Polygons output;
    {
        Slic3r::PluginStorage plugin_storage;
        storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
        slic3r_api::StoredExPolygonCollection stored(storage);
        for (const Slic3r::ExPolygon &original : originals) {
            const slic3r_api::ExPolygon source = view(original);
            stored.push_back(source);
        }
        output = rect_clipper::to_polygons(stored.readonly());
        REQUIRE(stored.size() == originals.size());
        for (uint32_t idx = 0; idx < stored.size(); ++idx) {
            const slic3r_api::ExPolygon source = stored.at(idx);
            check_exact_view(source.contour(), originals[idx].contour);
            REQUIRE(source.hole_size() == originals[idx].holes.size());
            size_t hole_idx = 0;
            for (const slic3r_api::Polygon hole : source.holes())
                check_exact_view(hole, originals[idx].holes[hole_idx++]);
            REQUIRE(hole_idx == originals[idx].holes.size());
        }
        check_exact_polygons(output, expected);
        REQUIRE(stored.free_from_storage());
        check_exact_polygons(output, expected);
    }
    check_exact_polygons(output, expected);
}
