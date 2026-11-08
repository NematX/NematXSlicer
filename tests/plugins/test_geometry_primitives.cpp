///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

#include <catch2/catch.hpp>
#include <limits>
#include <stdexcept>
#include <vector>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExPolygon.hpp"

namespace {
Slic3r::ExPolygon sample_polygon()
{
    using Slic3r::Point;
    using Slic3r::Polygon;
    const auto pt = [](double x, double y) { return Point(Slic3r::scale_i(x), Slic3r::scale_i(y)); };
    Polygon contour{pt(0., 0.), pt(10., 0.), pt(20., 0.), pt(20., 20.), pt(0., 20.)};
    Polygon hole{pt(6., 6.), pt(6., 14.), pt(14., 14.), pt(14., 6.)};
    return Slic3r::ExPolygon(std::move(contour), std::move(hole));
}
slic3r_api::ExPolygon sdk_view(const Slic3r::ExPolygon &source)
{
    // Tests alone bridge to native objects: production plugins cannot see them.
    return slic3r_api::ExPolygon(reinterpret_cast<const expolygon_handle *>(&source));
}
const Slic3r::ExPolygons &native_result(const slic3r_api::StoredExPolygonCollection &source)
{
    return *reinterpret_cast<const Slic3r::ExPolygons *>(source.handle());
}
void check_same_geometry(const Slic3r::ExPolygons &actual, const Slic3r::ExPolygons &expected)
{
    CHECK(actual.size() == expected.size());
    CHECK(Slic3r::diff_ex(actual, expected).empty());
    CHECK(Slic3r::diff_ex(expected, actual).empty());
}
}

TEST_CASE("SDK ensure_valid on a copy matches native ExPolygon simplification", "[plugins][geometry-primitives]")
{
    Slic3r::PluginStorage arena;
    auto *storage = reinterpret_cast<storage_handle *>(&arena);
    const Slic3r::ExPolygon source = sample_polygon();
    const Slic3r::ExPolygons before{source};
    slic3r_api::StoredExPolygonCollection result(storage, sdk_view(source));
    result.ensure_valid(Slic3r::SCALED_EPSILON);
    check_same_geometry(native_result(result), source.simplify(Slic3r::SCALED_EPSILON));
    REQUIRE(native_result(result).size() == 1);
    CHECK(native_result(result).front().contour.size() < source.contour.size());
    check_same_geometry({source}, before);
}

TEST_CASE("SDK variable inward offset matches per-vertex native offsets", "[plugins][geometry-primitives]")
{
    Slic3r::PluginStorage arena;
    auto *storage = reinterpret_cast<storage_handle *>(&arena);
    const Slic3r::ExPolygon source = sample_polygon();
    const Slic3r::ExPolygons before{source};
    std::vector<std::vector<float>> deltas{
        {0.f, float(-Slic3r::scale_d(.1)), float(-Slic3r::scale_d(.2)),
         float(-Slic3r::scale_d(.3)), float(-Slic3r::scale_d(.4))},
        std::vector<float>(4, float(-Slic3r::scale_d(.15)))
    };
    SECTION("Nonuniform contour and independent hole distances") {}
    SECTION("Neutral distances preserve material") {
        for (auto &ring : deltas)
            for (float &delta : ring) delta = 0.f;
    }
    SECTION("Large withdrawal can remove all material") {
        for (auto &ring : deltas)
            for (float &delta : ring) delta = float(-Slic3r::scale_d(100.));
    }
    const auto result = slic3r_api::clipper_variable_offset_inner(storage, sdk_view(source), deltas);
    check_same_geometry(native_result(result), Slic3r::variable_offset_inner_ex(source, deltas, 2.));
    check_same_geometry({source}, before);
}

TEST_CASE("SDK variable offset rejects malformed per-vertex data before the kernel", "[plugins][geometry-primitives]")
{
    Slic3r::PluginStorage arena;
    auto *storage = reinterpret_cast<storage_handle *>(&arena);
    const Slic3r::ExPolygon source = sample_polygon();
    const auto view = sdk_view(source);
    std::vector<float> flat(9, -1.f);
    CHECK(clipper_variable_offset_inner(nullptr, view.handle(), flat.data(), 9, 2.) == nullptr);
    CHECK(clipper_variable_offset_inner(storage, nullptr, flat.data(), 9, 2.) == nullptr);
    CHECK(clipper_variable_offset_inner(storage, view.handle(), nullptr, 9, 2.) == nullptr);
    CHECK(clipper_variable_offset_inner(storage, view.handle(), flat.data(), 8, 2.) == nullptr);
    CHECK(clipper_variable_offset_inner(storage, view.handle(), flat.data(), 9, .5) == nullptr);
    CHECK(clipper_variable_offset_inner(storage, view.handle(), flat.data(), 9,
                                       std::numeric_limits<double>::infinity()) == nullptr);
    flat[0] = 1.f;
    CHECK(clipper_variable_offset_inner(storage, view.handle(), flat.data(), 9, 2.) == nullptr);
    flat[0] = std::numeric_limits<float>::quiet_NaN();
    CHECK(clipper_variable_offset_inner(storage, view.handle(), flat.data(), 9, 2.) == nullptr);
    flat[0] = -std::numeric_limits<float>::max();
    CHECK(clipper_variable_offset_inner(storage, view.handle(), flat.data(), 9, 2.) == nullptr);
    CHECK_THROWS_AS(slic3r_api::clipper_variable_offset_inner(storage, view, {}), std::invalid_argument);
    CHECK_THROWS_AS(slic3r_api::clipper_variable_offset_inner(storage, view,
        std::vector<std::vector<float>>{{-1.f}, {-1.f}}), std::invalid_argument);
}


TEST_CASE("SDK ExPolygon validation reports malformed rings without asserting or mutating",
          "[plugins][geometry-primitives][validation]")
{
    Slic3r::ExPolygon source = sample_polygon();
    expolygon_status expected = EXPOLYGON_STATUS_OK;
    SECTION("Ordinary contour and hole") {}
    SECTION("Ordinary contour without holes") { source.holes.clear(); }
    SECTION("Nearby interior vertices") {
        source.contour.points.insert(source.contour.points.begin() + 1, Slic3r::Point(1, 1));
        expected = EXPOLYGON_STATUS_INVALID_GEOMETRY;
    }
    SECTION("Nearby closing vertices") {
        source.contour.points.emplace_back(1, 1);
        expected = EXPOLYGON_STATUS_INVALID_GEOMETRY;
    }
    SECTION("Duplicate interior vertices") {
        source.contour.points.insert(source.contour.points.begin() + 1, source.contour.points.front());
        expected = EXPOLYGON_STATUS_INVALID_GEOMETRY;
    }
    SECTION("Nearby hole vertices") {
        const Slic3r::Point point = source.holes.front().points.front();
        source.holes.front().points.insert(source.holes.front().points.begin() + 1,
                                          point + Slic3r::Point(1, 1));
        expected = EXPOLYGON_STATUS_INVALID_GEOMETRY;
    }
    SECTION("Contour with one point") {
        source.contour.points.resize(1);
        expected = EXPOLYGON_STATUS_INVALID_GEOMETRY;
    }
    SECTION("Contour with two points") {
        source.contour.points.resize(2);
        expected = EXPOLYGON_STATUS_INVALID_GEOMETRY;
    }
    SECTION("Empty hole") {
        source.holes.front().points.clear();
        expected = EXPOLYGON_STATUS_INVALID_GEOMETRY;
    }
    SECTION("Hole with one point") {
        source.holes.front().points.resize(1);
        expected = EXPOLYGON_STATUS_INVALID_GEOMETRY;
    }
    SECTION("Hole with two points") {
        source.holes.front().points.resize(2);
        expected = EXPOLYGON_STATUS_INVALID_GEOMETRY;
    }
    SECTION("Clockwise contour") {
        source.contour.reverse();
        expected = EXPOLYGON_STATUS_INVALID_ORIENTATION;
    }
    SECTION("Counterclockwise hole") {
        source.holes.front().reverse();
        expected = EXPOLYGON_STATUS_INVALID_ORIENTATION;
    }
    SECTION("Empty contour") {
        source.contour.points.clear();
        expected = EXPOLYGON_STATUS_EMPTY;
    }

    // Do not copy/move malformed Polygon objects: even their constructors
    // assert for one-point rings. Save raw points and populate default rings.
    const Slic3r::Points before_contour = source.contour.points;
    std::vector<Slic3r::Points> before_holes;
    for (const Slic3r::Polygon &hole : source.holes)
        before_holes.push_back(hole.points);
    Slic3r::ExPolygons collection(2);
    collection.front() = sample_polygon();
    collection.back().contour.points = before_contour;
    collection.back().holes.resize(before_holes.size());
    for (std::size_t idx = 0; idx < before_holes.size(); ++idx)
        collection.back().holes[idx].points = before_holes[idx];
    const auto *collection_handle =
        reinterpret_cast<const expolygon_collection_handle *>(&collection);
    CHECK(expolygon_valid(sdk_view(source).handle()) == expected);
    CHECK(sdk_view(source).is_valid() == (expected == EXPOLYGON_STATUS_OK));
    CHECK(expolygons_valid(collection_handle) == expected);
    CHECK(slic3r_api::ExPolygonCollection(collection_handle).valid_collection() ==
          (expected == EXPOLYGON_STATUS_OK));
    CHECK(source.contour.points == before_contour);
    REQUIRE(source.holes.size() == before_holes.size());
    for (std::size_t idx = 0; idx < source.holes.size(); ++idx) {
        CHECK(source.holes[idx].points == before_holes[idx]);
        CHECK(collection.back().holes[idx].points == before_holes[idx]);
    }
    CHECK(collection.back().contour.points == before_contour);
}

TEST_CASE("SDK ExPolygon validation uses the exact per-axis proximity threshold",
          "[plugins][geometry-primitives][validation]")
{
    const Slic3r::coord_t half_epsilon = Slic3r::SCALED_EPSILON / 2;
    for (const Slic3r::Point delta : { Slic3r::Point(half_epsilon - 1, half_epsilon - 1),
                                      Slic3r::Point(half_epsilon, half_epsilon - 1),
                                      Slic3r::Point(half_epsilon - 1, half_epsilon) }) {
        INFO("Point separation: " << delta.x() << ", " << delta.y());
        Slic3r::ExPolygon source = sample_polygon();
        source.contour.points.insert(source.contour.points.begin() + 1, delta);
        const expolygon_status expected = delta.x() < half_epsilon && delta.y() < half_epsilon ?
            EXPOLYGON_STATUS_INVALID_GEOMETRY : EXPOLYGON_STATUS_OK;
        CHECK(expolygon_valid(sdk_view(source).handle()) == expected);
        const Slic3r::ExPolygons collection{ source };
        CHECK(expolygons_valid(reinterpret_cast<const expolygon_collection_handle *>(&collection)) == expected);
    }
}

TEST_CASE("SDK ExPolygon validation preserves null and empty status contracts",
          "[plugins][geometry-primitives][validation]")
{
    CHECK(expolygon_valid(nullptr) == EXPOLYGON_STATUS_EMPTY);
    CHECK(expolygons_valid(nullptr) == EXPOLYGON_STATUS_EMPTY);
    const Slic3r::ExPolygon empty;
    CHECK(expolygon_valid(sdk_view(empty).handle()) == EXPOLYGON_STATUS_EMPTY);
    const Slic3r::ExPolygons empty_collection;
    CHECK(expolygons_valid(reinterpret_cast<const expolygon_collection_handle *>(&empty_collection)) ==
          EXPOLYGON_STATUS_OK);
    const Slic3r::ExPolygons with_empty{ empty };
    CHECK(expolygons_valid(reinterpret_cast<const expolygon_collection_handle *>(&with_empty)) ==
          EXPOLYGON_STATUS_EMPTY);
}
