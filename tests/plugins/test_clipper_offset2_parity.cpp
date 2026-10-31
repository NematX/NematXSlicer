/*
 * Clipper offset2 parity
 * =======================
 *
 * The legacy offset2_ex(ExPolygons) path offsets each ExPolygon ring family,
 * keeps the raw first-stage paths, and applies a second path-by-path offset.
 * The public clipper_offset2 operation chains two grouped PolyTree offsets.
 *
 * | variant | strict reference | intermediate |
 * |---------|------------------|--------------|
 * | expolygons | offset2_ex(ExPolygons) | legacy raw paths |
 * | expolygon | offset2_ex({expolygon}) | API collection cross-check |
 * | polygons | offset_ex(offset(Polygons, d1), d2) | opening/closing guard |
 *
 * The legacy path asserts for d2 == 0 and for polygon d1 == 0, so those cases
 * are covered only by API invariants or omitted from the polygon comparison.
 *
 * Documented divergences include invalid overlapping or touching input
 * ExPolygons, measured round-join differences on holed inputs, final square
 * cleanup that removes collinear vertices, and the contact/intermediate cases
 * listed by is_documented2().
 */

#include <catch2/catch.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <initializer_list>
#include <iostream>
#include <map>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Polygon.hpp"
#include "clipper_offset_test_helpers.hpp"

namespace {

using namespace Slic3r;
using namespace clipper_offset_test;

struct Offset2Params {
    double delta1_mm;
    double delta2_mm;
    clipper_join_type_t join;
    double limit;
};

struct Offset2Case {
    std::string name;
    std::vector<Slic3r::ExPolygon> inputs;
    bool documentary = false;
    std::vector<std::pair<double, double>> extra_pairs;
};

struct Offset2Topology {
    size_t islands = 0;
    size_t holes = 0;
    size_t contacts = 0;
    size_t self_touching_rings = 0;
};

struct ReportCounts {
    std::map<std::string, size_t> categories;
    std::map<std::string, std::string> first_examples;
};

// Build the two-stage matrix and append case-specific contact thresholds.
std::vector<Offset2Params> offset2_matrix(
    const std::vector<std::pair<double, double>> &extra_pairs);

// Return whether a measured mismatch is already documented for offset2.
bool is_documented2(const std::string &name,
                    const std::string &variant,
                    double delta1_mm,
                    double delta2_mm,
                    clipper_join_type_t join,
                    double limit);

// Offset copied ExPolygon collections through the two-stage public API.
Slic3r::ExPolygons api_offset2_expolygons(storage_handle *storage,
                                          const std::vector<Slic3r::ExPolygon> &inputs,
                                          const Offset2Params &params);

// Offset one copied ExPolygon through the single-input two-stage API path.
Slic3r::ExPolygons api_offset2_expolygon(storage_handle *storage,
                                         const Slic3r::ExPolygon &input,
                                         const Offset2Params &params);

// Offset copied flat Polygon inputs through the two-stage public API.
Slic3r::ExPolygons api_offset2_polygons(storage_handle *storage,
                                        const Slic3r::Polygons &inputs,
                                        const Offset2Params &params);

// Offset copied ExPolygon collections through one public API stage.
Slic3r::ExPolygons api_offset_expolygons_once(storage_handle *storage,
                                              const Slic3r::ExPolygons &inputs,
                                              double delta_mm,
                                              clipper_join_type_t join,
                                              double limit);

// Build the strict legacy reference for flat Polygon input.
Slic3r::ExPolygons legacy_offset2_polygons(const Slic3r::Polygons &inputs,
                                           const Offset2Params &params);

// Compose the native ExPolygon offset twice for mismatch classification only.
Slic3r::ExPolygons composed_native(const Slic3r::ExPolygons &inputs,
                                   const Offset2Params &params);

// Return all deterministic offset2 cases from the one-stage and contact suites.
std::vector<Offset2Case> offset2_cases();

// Compare all input variants for one case and one input ordering.
void run_case2(const Offset2Case &test_case);

// Check orientation and canonical equality of two API results.
void compare_api_results(const std::string &info, const Slic3r::ExPolygons &actual, const Slic3r::ExPolygons &expected);

// Verify the public two-stage operation against two public one-stage calls.
void run_api_invariants(const Offset2Case &test_case);

// Return topology and contact counts without asserting equality.
Offset2Topology topology2(const Slic3r::ExPolygons &expolygons);

// Classify one report mismatch without changing test outcomes.
std::string classify_mismatch(const Offset2Case &test_case,
                              const std::string &variant,
                              const Offset2Params &params,
                              const Slic3r::ExPolygons &api,
                              const Slic3r::ExPolygons &legacy,
                              const Slic3r::ExPolygons *composed);

// Add one mismatch to a report and retain its first diagnostic example.
void record_report(ReportCounts &counts,
                   const Offset2Case &test_case,
                   const std::string &variant,
                   const Offset2Params &params,
                   const Slic3r::ExPolygons &api,
                   const Slic3r::ExPolygons &legacy,
                   const Slic3r::ExPolygons *composed);

// Print report counters and first examples for one case.
void print_report(const Offset2Case &test_case, const ReportCounts &counts);

std::string params_info(const Offset2Params &params)
{
    return std::string("d1=") + std::to_string(params.delta1_mm) +
           " d2=" + std::to_string(params.delta2_mm) +
           " join=" + join_name(params.join) +
           " limit=" + std::to_string(params.limit);
}

std::vector<Offset2Params> offset2_matrix(
    const std::vector<std::pair<double, double>> &extra_pairs)
{
    const std::vector<std::pair<double, double>> base_pairs{
        {-0.1, 0.1}, {-0.225, 0.225}, {-1., 1.}, {-5., 5.},
        {0.1, -0.1}, {0.225, -0.225}, {1., -1.}, {5., -5.},
        {-1., 0.5}, {-0.5, 1.}, {1., -0.5}, {0.5, -1.},
        {-0.5, -0.5}, {0.5, 0.5}, {0., 1.}, {0., -1.}};
    std::vector<std::pair<double, double>> pairs(base_pairs.begin(), base_pairs.end());
    for (const std::pair<double, double> &extra : extra_pairs)
        if (std::find(pairs.begin(), pairs.end(), extra) == pairs.end())
            pairs.push_back(extra);

    const double round_limit = scale_d(0.005);
    std::vector<Offset2Params> result;
    result.reserve(pairs.size() * 5);
    for (const std::pair<double, double> &pair : pairs) {
        result.push_back({pair.first, pair.second, CLIPPER_JOIN_MITER, 1.2});
        result.push_back({pair.first, pair.second, CLIPPER_JOIN_MITER, 3.});
        result.push_back({pair.first, pair.second, CLIPPER_JOIN_MITER, 10.});
        result.push_back({pair.first, pair.second, CLIPPER_JOIN_SQUARE, 3.});
        result.push_back({pair.first, pair.second, CLIPPER_JOIN_ROUND, round_limit});
    }
    return result;
}

bool is_documented2(const std::string &name,
                    const std::string &variant,
                    double delta1_mm,
                    double delta2_mm,
                    clipper_join_type_t join,
                    double limit)
{
    (void)variant;
    const bool invalid_input = name == "Overlapping holed squares" ||
                               name == "Squares sharing an edge" ||
                               name == "Squares sharing a vertex";
    if (invalid_input)
        return true;

    const bool holed_input = name == "Square with hole" ||
                             name == "Two close holes" ||
                             name == "Holed square inside a hole" ||
                             name == "Hole near the contour" ||
                             name == "Vanishing hole" ||
                             name == "Rotated holed square" ||
                             name == "Three nesting levels" ||
                             name == "Island in hole after grow" ||
                             name == "Negative coordinates" ||
                             name == "Large positive coordinates" ||
                             name.find("Shifted start vertex ") == 0 ||
                             name == "Random disjoint holed rectangles";
    if (holed_input && join == CLIPPER_JOIN_ROUND)
        return true;

    const bool final_square_cleanup = join == CLIPPER_JOIN_SQUARE &&
                                      limit == 3. &&
                                      delta1_mm == 0.1 &&
                                      delta2_mm == -0.1;
    if (final_square_cleanup)
        return true;

    if (name == "Gap closing" && join == CLIPPER_JOIN_SQUARE &&
        delta1_mm > 0. && delta2_mm < 0.)
        return true;

    const bool contact_case = name == "88 hole touching contour" ||
                              name == "14 hole splitting island" ||
                              name == "244 holes merging contour" ||
                              name == "Two close holes contact";
    if (contact_case)
        return true;

    if (name == "Comb")
        return true;

    if (name == "Random disjoint holed rectangles" && join == CLIPPER_JOIN_SQUARE &&
        limit == 3. && delta1_mm == -1. && delta2_mm == 0.5)
        return true;

    return (name == "Rotated holed square" && join == CLIPPER_JOIN_SQUARE && limit == 3. &&
            delta1_mm == -1. && delta2_mm == 1.);
}

Slic3r::ExPolygons api_offset2_expolygons(storage_handle *storage,
                                          const std::vector<Slic3r::ExPolygon> &inputs,
                                          const Offset2Params &params)
{
    slic3r_api::StoredExPolygonCollection collection(storage);
    for (const Slic3r::ExPolygon &input : inputs) {
        slic3r_api::StoredExPolygon stored(storage);
        stored.copy_from(view(input));
        collection.push_back_move(std::move(stored));
    }

    slic3r_api::ClipperOperand operand(storage, collection.readonly());
    slic3r_api::ClipperOperand offset = slic3r_api::clipper_offset2(
        operand, scale_d(params.delta1_mm), scale_d(params.delta2_mm), params.join, params.limit);
    slic3r_api::StoredExPolygonCollection result = offset.to_expolygon_collection();
    return to_native(result.readonly());
}

Slic3r::ExPolygons api_offset2_expolygon(storage_handle *storage,
                                         const Slic3r::ExPolygon &input,
                                         const Offset2Params &params)
{
    const slic3r_api::ExPolygon source = view(input);
    slic3r_api::ClipperOperand operand(storage, source);
    slic3r_api::ClipperOperand offset = slic3r_api::clipper_offset2(
        operand, scale_d(params.delta1_mm), scale_d(params.delta2_mm), params.join, params.limit);
    slic3r_api::StoredExPolygonCollection result = offset.to_expolygon_collection();
    return to_native(result.readonly());
}

Slic3r::ExPolygons api_offset2_polygons(storage_handle *storage,
                                        const Slic3r::Polygons &inputs,
                                        const Offset2Params &params)
{
    slic3r_api::StoredPolygonCollection collection(storage);
    for (const Slic3r::Polygon &input : inputs) {
        slic3r_api::StoredPolygon stored(storage);
        stored.copy_from(view(input));
        collection.push_back_move(std::move(stored));
    }

    slic3r_api::ClipperOperand operand(storage, collection.readonly());
    slic3r_api::ClipperOperand offset = slic3r_api::clipper_offset2(
        operand, scale_d(params.delta1_mm), scale_d(params.delta2_mm), params.join, params.limit);
    slic3r_api::StoredExPolygonCollection result = offset.to_expolygon_collection();
    return to_native(result.readonly());
}

Slic3r::ExPolygons api_offset_expolygons_once(storage_handle *storage,
                                              const Slic3r::ExPolygons &inputs,
                                              double delta_mm,
                                              clipper_join_type_t join,
                                              double limit)
{
    slic3r_api::StoredExPolygonCollection collection(storage);
    for (const Slic3r::ExPolygon &input : inputs) {
        slic3r_api::StoredExPolygon stored(storage);
        stored.copy_from(view(input));
        collection.push_back_move(std::move(stored));
    }

    slic3r_api::ClipperOperand operand(storage, collection.readonly());
    slic3r_api::ClipperOperand offset = slic3r_api::clipper_offset(
        operand, scale_d(delta_mm), join, limit);
    slic3r_api::StoredExPolygonCollection result = offset.to_expolygon_collection();
    return to_native(result.readonly());
}

Slic3r::ExPolygons legacy_offset2_polygons(const Slic3r::Polygons &inputs,
                                           const Offset2Params &params)
{
    const Slic3r::Polygons first = Slic3r::offset(
        inputs, scale_d(params.delta1_mm), legacy_join(params.join), params.limit);
    return Slic3r::offset_ex(
        first, scale_d(params.delta2_mm), legacy_join(params.join), params.limit);
}

Slic3r::ExPolygons composed_native(const Slic3r::ExPolygons &inputs,
                                   const Offset2Params &params)
{
    const Slic3r::ExPolygons first = Slic3r::offset_ex(
        inputs, scale_d(params.delta1_mm), legacy_join(params.join), params.limit);
    return Slic3r::offset_ex(
        first, scale_d(params.delta2_mm), legacy_join(params.join), params.limit);
}

Offset2Topology topology2(const Slic3r::ExPolygons &expolygons)
{
    Offset2Topology result;
    result.islands = expolygons.size();
    for (const Slic3r::ExPolygon &expolygon : expolygons)
        result.holes += expolygon.holes.size();
    const Contacts contacts = find_contacts(expolygons);
    result.contacts = contacts.points.size();
    result.self_touching_rings = contacts.self_touching_rings;
    return result;
}

bool same_topology(const Offset2Topology &first, const Offset2Topology &second)
{
    return first.islands == second.islands && first.holes == second.holes;
}

// Compare borrowed results by canonical geometry; report orientation/equality failures, formatting full geometry only for mismatches.
void compare_api_results(const std::string &info,
                         const Slic3r::ExPolygons &actual,
                         const Slic3r::ExPolygons &expected)
{
    check_orientation(info + " actual", actual);
    check_orientation(info + " expected", expected);
    const CanonicalShape actual_shape = canonical(actual);
    const CanonicalShape expected_shape = canonical(expected);
    if (actual_shape == expected_shape) {
        CHECK(actual_shape == expected_shape);
        return;
    }
    INFO(info);
    INFO("actual " << describe(actual));
    INFO("expected " << describe(expected));
    CHECK(actual_shape == expected_shape);
}

std::vector<Offset2Case> offset2_cases()
{
    std::vector<Offset2Case> cases;
    const Slic3r::ExPolygon square = square_with_holes(
        rectangle_mm(0., 0., 20., 20.), {rectangle_mm(8., 8., 12., 12.)});
    cases.push_back({"Square", {Slic3r::ExPolygon(rectangle_mm(0., 0., 20., 20.))}, false, {}});
    cases.push_back({"Square with hole", {square}, false, {{-1.9, 1.9}}});
    cases.push_back({"Two close holes", {
        square_with_holes(rectangle_mm(0., 0., 30., 30.),
                          {rectangle_mm(10., 10., 14., 14.), rectangle_mm(15., 15., 19., 19.)})},
        false, {{1., -6.}, {-2.5, -2.5}}});
    cases.push_back({"Holed square inside a hole", {
        square_with_holes(rectangle_mm(0., 0., 40., 40.), {rectangle_mm(5., 5., 35., 35.)}),
        square_with_holes(rectangle_mm(10., 10., 30., 30.), {rectangle_mm(17., 17., 23., 23.)})},
        false, {{-2.4, 2.4}, {4.9, -4.9}}});
    cases.push_back({"Hole near the contour", {
        square_with_holes(rectangle_mm(0., 0., 20., 20.), {rectangle_mm(0.3, 5., 5., 15.)})},
        false, {{-0.2, 0.2}, {-0.4, 0.4}}});
    cases.push_back({"Vanishing hole", {
        square_with_holes(rectangle_mm(0., 0., 20., 20.), {rectangle_mm(9., 9., 11., 11.)})},
        false, {{0.9, -0.9}, {1.1, -1.1}}});

    const Slic3r::ExPolygons dumbbell = union_ex(Slic3r::ExPolygons{
        Slic3r::ExPolygon(rectangle_mm(0., 0., 8., 8.)),
        Slic3r::ExPolygon(rectangle_mm(8., 3.9, 12., 4.1)),
        Slic3r::ExPolygon(rectangle_mm(12., 0., 20., 8.))});
    REQUIRE(dumbbell.size() == 1);
    cases.push_back({"Dumbbell", {dumbbell.front()}, false, {{-0.15, 0.15}, {-0.05, 0.05}}});
    cases.push_back({"Collapsing square", {Slic3r::ExPolygon(rectangle_mm(0., 0., 2., 2.))},
                       false, {{-1.5, 1.5}, {-5., 5.}}});
    cases.push_back({"Empty collection", {}, false, {{-1.5, 1.5}, {-5., 5.}}});
    cases.push_back({"Empty ExPolygon", {Slic3r::ExPolygon()}, false, {{-1.5, 1.5}, {-5., 5.}}});

    Slic3r::Polygon triangle({Slic3r::Point(scale_i(0.), scale_i(0.)),
                              Slic3r::Point(scale_i(20.), scale_i(0.)),
                              Slic3r::Point(scale_i(20.), scale_i(5.36))});
    triangle.make_counter_clockwise();
    cases.push_back({"Sharp triangle", {Slic3r::ExPolygon(triangle)}, false, {}});
    const Slic3r::ExPolygons l_shape = diff_ex(
        Slic3r::ExPolygon(rectangle_mm(0., 0., 20., 20.)), rectangle_mm(10., 10., 20., 20.));
    REQUIRE(l_shape.size() == 1);
    cases.push_back({"Concave L", {l_shape.front()}, false, {}});
    cases.push_back({"Rotated holed square", {rotated(square, 30. * M_PI / 180.)}, false, {}});
    cases.push_back({"Three nesting levels", {
        square_with_holes(rectangle_mm(0., 0., 40., 40.), {rectangle_mm(5., 5., 35., 35.)}),
        square_with_holes(rectangle_mm(10., 10., 30., 30.), {rectangle_mm(15., 15., 25., 25.)}),
        Slic3r::ExPolygon(rectangle_mm(18., 18., 22., 22.))}, false, {}});

    Slic3r::ExPolygon negative = square;
    negative.translate(Point(scale_i(-500.), scale_i(-500.)));
    Slic3r::ExPolygon positive = square;
    positive.translate(Point(scale_i(480.), scale_i(480.)));
    cases.push_back({"Negative coordinates", {negative}, false, {}});
    cases.push_back({"Large positive coordinates", {positive}, false, {}});
    for (size_t shift = 1; shift <= 3; ++shift)
        cases.push_back({"Shifted start vertex " + std::to_string(shift),
                         {start_shifted(square, shift)}, false, {}});

    const Slic3r::ExPolygon overlapping_first = square_with_holes(
        rectangle_mm(0., 0., 20., 20.), {rectangle_mm(5., 5., 9., 9.)});
    const Slic3r::ExPolygon overlapping_second = square_with_holes(
        rectangle_mm(12., 12., 32., 32.), {rectangle_mm(23., 23., 27., 27.)});
    cases.push_back({"Overlapping holed squares", {overlapping_first, overlapping_second}, true, {}});
    cases.push_back({"Squares sharing an edge", {
        Slic3r::ExPolygon(rectangle_mm(0., 0., 10., 10.)),
        Slic3r::ExPolygon(rectangle_mm(10., 0., 20., 10.))}, true, {}});
    cases.push_back({"Squares sharing a vertex", {
        Slic3r::ExPolygon(rectangle_mm(0., 0., 10., 10.)),
        Slic3r::ExPolygon(rectangle_mm(10., 10., 20., 20.))}, true, {}});

    cases.push_back({"Gap closing", {
        Slic3r::ExPolygon(rectangle_mm(0., 0., 10., 10.)),
        Slic3r::ExPolygon(rectangle_mm(10.5, 0., 20.5, 10.))}, false,
        {{0.25, -0.25}, {0.3, -0.3}, {0.2, -0.2}}});
    cases.push_back({"Comb", {Slic3r::ExPolygon(comb_mm(1., 8, 6.))}, false,
        {{-0.5, 0.5}, {0.5, -0.5}}});

    cases.push_back({"88 hole touching contour", {
        with_holes(rectangle_mm(105., 19., 118., 34.),
                   {rectangle_mm(110., 26., 112., 31.), rectangle_mm(110., 21., 115., 24.)})},
        false, {{0.5, -1.5}}});
    cases.push_back({"14 hole splitting island", {
        with_holes(rectangle_mm(46., 2., 73., 12.),
                   {rectangle_mm(61., 4., 68., 10.), rectangle_mm(49., 5., 53., 9.)})},
        false, {{0.5, -1.5}}});
    cases.push_back({"244 holes merging contour", {
        with_holes(rectangle_mm(85., 21., 104., 37.),
                   {rectangle_mm(93., 27., 97., 34.), rectangle_mm(91., 22., 98., 26.)})},
        false, {{0.25, -0.75}}});
    cases.push_back({"Two close holes contact", {
        square_with_holes(rectangle_mm(0., 0., 30., 30.),
                          {rectangle_mm(10., 10., 14., 14.), rectangle_mm(15., 15., 19., 19.)})},
        false, {{1., -6.}, {-2.5, -2.5}}});

    cases.push_back({"Island in hole after grow", {
        square_with_holes(rectangle_mm(0., 0., 40., 40.), {rectangle_mm(5., 5., 35., 35.)}),
        Slic3r::ExPolygon(rectangle_mm(15., 15., 25., 25.))}, false, {}});
    return cases;
}

void compare_polygons_guard(const std::string &info,
                            const Slic3r::Polygons &polygons,
                            const Offset2Params &params,
                            const Slic3r::ExPolygons &api_result,
                            bool documentary)
{
    if (params.delta1_mm < 0. && params.delta2_mm > 0.) {
        const Slic3r::ExPolygons opening = Slic3r::opening_ex(
            polygons, scale_d(-params.delta1_mm), scale_d(params.delta2_mm),
            legacy_join(params.join), params.limit);
        if (documentary)
            compare(info + " opening guard", api_result, opening, true);
        else
            compare_api_results(info + " opening guard", api_result, opening);
    } else if (params.delta1_mm > 0. && params.delta2_mm < 0.) {
        const Slic3r::ExPolygons closing = Slic3r::closing_ex(
            polygons, scale_d(params.delta1_mm), scale_d(-params.delta2_mm),
            legacy_join(params.join), params.limit);
        if (documentary)
            compare(info + " closing guard", api_result, closing, true);
        else
            compare_api_results(info + " closing guard", api_result, closing);
    }
}

// Copy and rotate the borrowed case inputs, reuse order-specific flat rings, and report parity failures for every parameter/variant combination.
void run_case2(const Offset2Case &test_case)
{
    const std::vector<Offset2Params> matrix = offset2_matrix(test_case.extra_pairs);
    std::vector<Slic3r::ExPolygon> ordered_inputs(test_case.inputs);
    std::vector<size_t> ordered_indices;
    for (size_t index = 0; index < ordered_inputs.size(); ++index)
        ordered_indices.push_back(index);

    const size_t order_count = ordered_inputs.size() < 2 ? 1 : ordered_inputs.size();
    for (size_t order = 0; order < order_count; ++order) {
        if (order > 0) {
            std::rotate(ordered_inputs.begin(), ordered_inputs.begin() + 1, ordered_inputs.end());
            std::rotate(ordered_indices.begin(), ordered_indices.begin() + 1, ordered_indices.end());
        }
        const Slic3r::Polygons polygons = needs_manual_polygon_conversion(ordered_inputs) ?
            flatten_polygons(ordered_inputs) : Slic3r::to_polygons(ordered_inputs);
        for (const Offset2Params &params : matrix) {
            Slic3r::PluginStorage plugin_storage;
            storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
            const Slic3r::ExPolygons collection_api = api_offset2_expolygons(storage, ordered_inputs, params);
            const Slic3r::ExPolygons collection_legacy = Slic3r::offset2_ex(
                ordered_inputs, scale_d(params.delta1_mm), scale_d(params.delta2_mm),
                legacy_join(params.join), params.limit);
            const std::string prefix = test_case.name + " order=" + std::to_string(order) +
                                       " " + params_info(params);
            const bool documentary = test_case.documentary || is_documented2(
                test_case.name, "expolygons", params.delta1_mm, params.delta2_mm, params.join, params.limit);
            compare(prefix + " expolygons", collection_api, collection_legacy, documentary);

            if (ordered_inputs.size() == 1) {
                const Slic3r::ExPolygons single_api = api_offset2_expolygon(
                    storage, ordered_inputs.front(), params);
                const Slic3r::ExPolygons &single_legacy = collection_legacy;
                compare_api_results(prefix + " single API collection", single_api, collection_api);
                compare(prefix + " single API legacy", single_api, single_legacy, documentary);
            }

            if (params.delta1_mm != 0.) {
                const Slic3r::ExPolygons polygons_api = api_offset2_polygons(storage, polygons, params);
                const Slic3r::ExPolygons polygons_legacy = legacy_offset2_polygons(polygons, params);
                const bool polygon_documentary = test_case.documentary || is_documented2(
                    test_case.name, "polygons", params.delta1_mm, params.delta2_mm, params.join, params.limit);
                compare(prefix + " polygons", polygons_api, polygons_legacy, polygon_documentary);
                compare_polygons_guard(prefix + " polygons", polygons, params, polygons_api, polygon_documentary);
            }
        }
    }
}

void run_api_invariants(const Offset2Case &test_case)
{
    const std::vector<Offset2Params> matrix = offset2_matrix(test_case.extra_pairs);
    for (const Offset2Params &params : matrix) {
        std::vector<Slic3r::ExPolygon> ordered_inputs(test_case.inputs);
        Slic3r::ExPolygons order_reference;
        bool have_order_reference = false;
        const size_t order_count = ordered_inputs.size() < 2 ? 1 : ordered_inputs.size();
        for (size_t order = 0; order < order_count; ++order) {
            if (order > 0)
                std::rotate(ordered_inputs.begin(), ordered_inputs.begin() + 1, ordered_inputs.end());
            Slic3r::PluginStorage plugin_storage;
            storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
            const Slic3r::ExPolygons actual = api_offset2_expolygons(storage, ordered_inputs, params);
            const Slic3r::ExPolygons first = api_offset_expolygons_once(
                storage, Slic3r::ExPolygons(ordered_inputs.begin(), ordered_inputs.end()),
                params.delta1_mm, params.join, params.limit);
            const Slic3r::ExPolygons sequential = api_offset_expolygons_once(
                storage, first, params.delta2_mm, params.join, params.limit);
            const std::string prefix = test_case.name + " order=" + std::to_string(order) +
                                       " " + params_info(params);
            INFO(prefix);
            check_orientation(prefix + " offset2", actual);
            CHECK(canonical(actual) == canonical(sequential));

            const Offset2Params zero_second{params.delta1_mm, 0., params.join, params.limit};
            const Slic3r::ExPolygons offset2_zero = api_offset2_expolygons(storage, ordered_inputs, zero_second);
            CHECK(canonical(offset2_zero) == canonical(first));
            if (!have_order_reference) {
                order_reference = actual;
                have_order_reference = true;
            } else {
                CHECK(canonical(actual) == canonical(order_reference));
            }
        }
    }
}

void run_random_case2()
{
    const unsigned int seed = 0x0ff5e72u;
    std::mt19937 rng(seed);
    const std::vector<Offset2Params> matrix = offset2_matrix({});
    std::uniform_int_distribution<size_t> matrix_distribution(0, matrix.size() - 1);

    for (size_t iteration = 0; iteration < 300; ++iteration) {
        const std::vector<Slic3r::ExPolygon> inputs = random_disjoint_holed_rectangles(rng);
        const Offset2Params params = matrix[matrix_distribution(rng)];
        Slic3r::PluginStorage plugin_storage;
        storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
        const Slic3r::ExPolygons api = api_offset2_expolygons(storage, inputs, params);
        const Slic3r::ExPolygons legacy = Slic3r::offset2_ex(
            inputs, scale_d(params.delta1_mm), scale_d(params.delta2_mm),
            legacy_join(params.join), params.limit);
        const std::string info = "Random disjoint holed rectangles seed=" + std::to_string(seed) +
                                 " iteration=" + std::to_string(iteration) + " " + params_info(params);
        INFO(info << " inputs=" << describe(Slic3r::ExPolygons(inputs.begin(), inputs.end())));
        compare(info + " variant=expolygons", api, legacy,
                is_documented2("Random disjoint holed rectangles", "expolygons",
                               params.delta1_mm, params.delta2_mm, params.join, params.limit));

        if (params.delta1_mm != 0.) {
            const Slic3r::Polygons polygons = needs_manual_polygon_conversion(inputs) ?
                flatten_polygons(inputs) : Slic3r::to_polygons(inputs);
            const Slic3r::ExPolygons api_polygons = api_offset2_polygons(storage, polygons, params);
            const Slic3r::ExPolygons legacy_polygons = legacy_offset2_polygons(polygons, params);
            compare(info + " variant=polygons", api_polygons, legacy_polygons,
                    is_documented2("Random disjoint holed rectangles", "polygons",
                                   params.delta1_mm, params.delta2_mm, params.join, params.limit));
        }
    }
}

std::string classify_mismatch(const Offset2Case &test_case,
                              const std::string &variant,
                              const Offset2Params &params,
                              const Slic3r::ExPolygons &api,
                              const Slic3r::ExPolygons &legacy,
                              const Slic3r::ExPolygons *composed)
{
    if (test_case.documentary || is_documented2(test_case.name, variant,
                                                 params.delta1_mm, params.delta2_mm,
                                                 params.join, params.limit))
        return "documentary";
    if (canonical(api) == canonical(legacy))
        return "exact";

    const Offset2Topology api_topology = topology2(api);
    const Offset2Topology legacy_topology = topology2(legacy);
    const double area = symmetric_area_mm2(api, legacy);
    if (params.join == CLIPPER_JOIN_ROUND && area < 1e-3 && same_topology(api_topology, legacy_topology))
        return "round_rounding";
    if (composed != nullptr && topology2(legacy).contacts > topology2(api).contacts &&
        canonical(api) == canonical(*composed))
        return "legacy_bridge";
    if (area < 1e-9 && !same_topology(api_topology, legacy_topology) && legacy_topology.contacts == 0)
        return "intermediate";
    if (area >= 1e-3)
        return "area";
    return "other";
}

void record_report(ReportCounts &counts,
                   const Offset2Case &test_case,
                   const std::string &variant,
                   const Offset2Params &params,
                   const Slic3r::ExPolygons &api,
                   const Slic3r::ExPolygons &legacy,
                   const Slic3r::ExPolygons *composed)
{
    const std::string category = classify_mismatch(test_case, variant, params, api, legacy, composed);
    ++counts.categories[category];
    if (category != "exact" && counts.first_examples.find(category) == counts.first_examples.end()) {
        counts.first_examples[category] = variant + " " + params_info(params) +
            " api=" + describe(api) + " legacy=" + describe(legacy);
    }
}

void print_report(const Offset2Case &test_case, const ReportCounts &counts)
{
    std::cout << "offset2_report case=" << test_case.name;
    for (const std::pair<const std::string, size_t> &entry : counts.categories)
        std::cout << " " << entry.first << "=" << entry.second;
    std::cout << '\n';
    for (const std::pair<const std::string, std::string> &entry : counts.first_examples)
        std::cout << "offset2_report_example case=" << test_case.name
                  << " category=" << entry.first << " " << entry.second << '\n';
}

void report_case2(const Offset2Case &test_case)
{
    const std::vector<Offset2Params> matrix = offset2_matrix(test_case.extra_pairs);
    std::vector<Slic3r::ExPolygon> ordered_inputs(test_case.inputs);
    const size_t order_count = ordered_inputs.size() < 2 ? 1 : ordered_inputs.size();
    for (size_t order = 0; order < order_count; ++order) {
        if (order > 0)
            std::rotate(ordered_inputs.begin(), ordered_inputs.begin() + 1, ordered_inputs.end());
        ReportCounts counts;
        for (const Offset2Params &params : matrix) {
            Slic3r::PluginStorage plugin_storage;
            storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
            const Slic3r::ExPolygons api = api_offset2_expolygons(storage, ordered_inputs, params);
            const Slic3r::ExPolygons legacy = Slic3r::offset2_ex(
                ordered_inputs, scale_d(params.delta1_mm), scale_d(params.delta2_mm),
                legacy_join(params.join), params.limit);
            const Slic3r::ExPolygons composed = composed_native(
                Slic3r::ExPolygons(ordered_inputs.begin(), ordered_inputs.end()), params);
            record_report(counts, test_case, "expolygons order=" + std::to_string(order),
                          params, api, legacy, &composed);
            if (params.delta1_mm != 0.) {
                const Slic3r::Polygons polygons = needs_manual_polygon_conversion(ordered_inputs) ?
                    flatten_polygons(ordered_inputs) : Slic3r::to_polygons(ordered_inputs);
                const Slic3r::ExPolygons api_polygons = api_offset2_polygons(storage, polygons, params);
                const Slic3r::ExPolygons legacy_polygons = legacy_offset2_polygons(polygons, params);
                record_report(counts, test_case, "polygons order=" + std::to_string(order),
                              params, api_polygons, legacy_polygons, nullptr);
            }
        }
        print_report(test_case, counts);
    }
}

} // namespace

TEST_CASE("Clipper offset2 parity", "[plugins][clipper][clipper-offset2]")
{
    for (const Offset2Case &test_case : offset2_cases())
        run_case2(test_case);
}

TEST_CASE("Clipper offset2 API invariants", "[plugins][clipper][clipper-offset2]")
{
    for (const Offset2Case &test_case : offset2_cases())
        run_api_invariants(test_case);
}

TEST_CASE("Clipper offset2 random parity", "[plugins][clipper][clipper-offset2-random]")
{
    run_random_case2();
}

TEST_CASE("Clipper offset2 classification report", "[.][clipper-offset2-report]")
{
    for (const Offset2Case &test_case : offset2_cases())
        report_case2(test_case);
}
