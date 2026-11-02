#include <catch2/catch.hpp>

#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include "libslic3r/Plugins/LayerExtrusionEdit/SlowdownSolver.hpp"
#include "libslic3r/GCode.hpp"
#include "libslic3r/GCode/CoolingBuffer.hpp"
#include "libslic3r/Model.hpp"
#include "plugin_test_helpers.hpp"

/*
Pure numerical tests of the short-layer slowdown solver
=======================================================

The numeric cases build a SlowdownSolver::Layer from
plain numbers whose expected result can be computed by hand. A line's length is
`seconds * speed`, so slowing it to speed f gives it `length / f` seconds. The
comment of each test gives the closed-form derivation of its expectation.

    TEST_CASE
    |-- make_extruder() / make_layer(): build owned numeric input
    `-- solve(): compare speeds, bounds and total duration with hand calculations

Both intentional legacy differences have separate numeric oracles. Repeated
calls must preserve the input and produce identical output; invalid inputs throw.
The legacy comparison additionally runs CoolingBuffer on the same straight paths
with one tool, where neither of the two intentional model differences applies.
*/

namespace {
using namespace slic3r_api::LayerExtrusionEdit::SlowdownSolver;

// One extruder with the given bounds and lines (seconds, speed, adjustable).
Extruder make_extruder(double threshold, double min_speed, double reduction, std::vector<Line> lines,
                       double fixed = 0.0)
{
    Extruder extruder;
    extruder.threshold_seconds = threshold;
    extruder.min_print_speed = min_speed;
    extruder.max_speed_reduction = reduction;
    extruder.fixed_seconds = fixed;
    extruder.lines = std::move(lines);
    return extruder;
}

// Combine owned tool inputs with fixed time that belongs to no geometric line.
Layer make_layer(std::vector<Extruder> extruders, double unassigned = 0.0)
{
    Layer layer;
    layer.unassigned_seconds = unassigned;
    layer.extruders = std::move(extruders);
    return layer;
}
} // namespace

TEST_CASE("Solver equalizes the fastest lines first and reaches the threshold",
          "[plugins][layer-extrusion-edit][slowdown][slowdown-solver]")
{
    // A: 1 s at 100 (length 100), B: 1 s at 50 (length 50), threshold 10 s, min speed 10.
    // 8 s are missing. Step 1: A alone goes down to B's speed (gain 1 s). Step 2: both
    // lines (now 2 s and 1 s at 50) go to a common speed f with 3 * (50 / f - 1) = 7,
    // hence 50 / f = 10 / 3 and f = 15 for both: 100/15 + 50/15 = 10 s.
    const Result result = solve(make_layer({make_extruder(10, 10, 0, {{1, 100, true}, {1, 50, true}})}));
    REQUIRE(result.lines[0].size() == 2);
    CHECK(result.lines[0][0].slowed);
    CHECK(result.lines[0][1].slowed);
    CHECK(result.lines[0][0].speed_mm_per_s == Approx(15.f).margin(1e-3));
    CHECK(result.lines[0][1].speed_mm_per_s == Approx(15.f).margin(1e-3));
    CHECK(result.total_seconds == Approx(10.0).margin(1e-3));
}

TEST_CASE("Solver slows only the fastest line when that is enough",
          "[plugins][layer-extrusion-edit][slowdown][slowdown-solver]")
{
    // A: 1 s at 100, B: 1 s at 50, threshold 2.5 s: only 0.5 s is missing.
    // A alone absorbs it: 100 / f = 1.5 -> f = 66.67 (above B's speed), B is untouched.
    const Result result = solve(make_layer({make_extruder(2.5, 10, 0, {{1, 100, true}, {1, 50, true}})}));
    CHECK(result.lines[0][0].slowed);
    CHECK(result.lines[0][0].speed_mm_per_s == Approx(100.f / 1.5f).margin(1e-3));
    CHECK_FALSE(result.lines[0][1].slowed);
    CHECK(result.lines[0][1].speed_mm_per_s == 50.f);
    CHECK(result.total_seconds == Approx(2.5).margin(1e-3));
}

TEST_CASE("Solver never goes under min_print_speed and returns the longest reachable layer",
          "[plugins][layer-extrusion-edit][slowdown][slowdown-solver]")
{
    // Same lines, unreachable threshold 100 s: both go to the 10 mm/s floor,
    // taking 100/10 + 50/10 = 15 s.
    const Result result = solve(make_layer({make_extruder(100, 10, 0, {{1, 100, true}, {1, 50, true}})}));
    CHECK(result.lines[0][0].speed_mm_per_s == Approx(10.f).margin(1e-3));
    CHECK(result.lines[0][1].speed_mm_per_s == Approx(10.f).margin(1e-3));
    CHECK(result.total_seconds == Approx(15.0).margin(1e-3));
}

TEST_CASE("Solver bounds the slowdown by max_speed_reduction",
          "[plugins][layer-extrusion-edit][slowdown][slowdown-solver]")
{
    SECTION("a 50 % reduction halves the speed at most")
    {
        // 1 s at 100, no minimum speed, reduction 50 %: time_max = 1 / (1 - 0.5) = 2 s.
        const Result result = solve(make_layer({make_extruder(100, 0, 0.5, {{1, 100, true}})}));
        CHECK(result.lines[0][0].speed_mm_per_s == Approx(50.f).margin(1e-3));
        CHECK(result.total_seconds == Approx(2.0).margin(1e-3));
    }
    SECTION("a 90 % reduction against a 10 mm/s floor: the tighter bound wins")
    {
        // 1 s at 100: floor gives 10 s, 90 % gives 10 s too; 1 s at 40: floor gives 4 s,
        // 90 % gives 10 s, so the floor (4 s) wins.
        const Result result = solve(make_layer({make_extruder(1000, 10, 0.9, {{1, 100, true}, {1, 40, true}})}));
        CHECK(result.lines[0][0].speed_mm_per_s == Approx(10.f).margin(1e-3));
        CHECK(result.lines[0][1].speed_mm_per_s == Approx(10.f).margin(1e-3));
        CHECK(result.total_seconds == Approx(14.0).margin(1e-3));
    }
    SECTION("0 % and 100 % do not bound anything")
    {
        for (const double reduction : {0.0, 1.0}) {
            const Result result = solve(make_layer({make_extruder(1000, 10, reduction, {{1, 100, true}})}));
            CHECK(result.lines[0][0].speed_mm_per_s == Approx(10.f).margin(1e-3));
        }
    }
}

TEST_CASE("Solver without a minimum speed stretches every slowable line by one factor",
          "[plugins][layer-extrusion-edit][slowdown][slowdown-solver]")
{
    // 1 s at 100 plus 1 s at 50, threshold 6 s, min speed 0, no reduction bound:
    // the 4 missing seconds are spread proportionally, factor (2 + 4) / 2 = 3.
    const Result result = solve(make_layer({make_extruder(6, 0, 0, {{1, 100, true}, {1, 50, true}})}));
    CHECK(result.lines[0][0].speed_mm_per_s == Approx(100.f / 3.f).margin(1e-3));
    CHECK(result.lines[0][1].speed_mm_per_s == Approx(50.f / 3.f).margin(1e-3));
    CHECK(result.total_seconds == Approx(6.0).margin(1e-3));
}

TEST_CASE("Solver leaves a layer that is already long enough untouched (x 1.001 tolerance)",
          "[plugins][layer-extrusion-edit][slowdown][slowdown-solver]")
{
    // 5 s layer: 5 * 1.001 = 5.005. A 5.004 s goal is considered reached, 5.006 s is not.
    const Result reached = solve(make_layer({make_extruder(5.004, 10, 0, {{5, 100, true}})}));
    CHECK_FALSE(reached.lines[0][0].slowed);
    CHECK(reached.total_seconds == Approx(5.0));
    const Result missed = solve(make_layer({make_extruder(5.006, 10, 0, {{5, 100, true}})}));
    CHECK(missed.lines[0][0].slowed);
    CHECK(missed.total_seconds == Approx(5.006).margin(1e-3));
}

TEST_CASE("Solver counts fixed, unassigned and non-adjustable time without changing it",
          "[plugins][layer-extrusion-edit][slowdown][slowdown-solver]")
{
    SECTION("non-adjustable line and fixed seconds")
    {
        // Fixed 1 s + non-adjustable 1 s + unassigned 1 s + adjustable 1 s at 100, goal 8 s:
        // the adjustable line must take 5 s (4 s are missing), i.e. 20 mm/s.
        const Result result = solve(make_layer({make_extruder(8, 10, 0, {{1, 100, false}, {1, 100, true}}, 1.0)}, 1.0));
        CHECK_FALSE(result.lines[0][0].slowed);
        CHECK(result.lines[0][1].speed_mm_per_s == Approx(20.f).margin(1e-3));
        CHECK(result.total_seconds == Approx(8.0).margin(1e-3));
    }
    SECTION("a line already at or under the minimum speed is not touched")
    {
        const Result result = solve(make_layer({make_extruder(100, 10, 0, {{2, 10, true}, {1, 5, true}})}));
        CHECK_FALSE(result.lines[0][0].slowed);
        CHECK_FALSE(result.lines[0][1].slowed);
        CHECK(result.total_seconds == Approx(3.0));
    }
    SECTION("a zero-length line is fixed")
    {
        const Result result = solve(make_layer({make_extruder(100, 10, 0, {{0, 100, true}})}));
        CHECK_FALSE(result.lines[0][0].slowed);
    }
}

TEST_CASE("Solver stretch is capped by each line's maximum duration",
          "[plugins][layer-extrusion-edit][slowdown][slowdown-solver]")
{
    // Deliberate difference 2. A: 1 s at 100, B: 1 s at 20, min speed 1, reduction 50 %:
    // both have time_max = 2 s. Goal 3.8 s -> 1.8 s missing.
    // A can only gain 1 s (it stops at 50 mm/s); B must provide the remaining 0.8 s:
    // 1 * (20 / f - 1) = 0.8 -> f = 11.11. An uncapped formula would slow both to ~31.6.
    const Result result = solve(make_layer({make_extruder(3.8, 1, 0.5, {{1, 100, true}, {1, 20, true}})}));
    CHECK(result.lines[0][0].speed_mm_per_s == Approx(50.f).margin(1e-3));
    CHECK(result.lines[0][1].speed_mm_per_s == Approx(20.f / 1.8f).margin(1e-3));
    CHECK(result.total_seconds == Approx(3.8).margin(1e-3));
}

TEST_CASE("Solver uses current durations when several thresholds apply",
          "[plugins][layer-extrusion-edit][slowdown][slowdown-solver]")
{
    // Deliberate difference 1. E0 (goal 5 s) and E1 (goal 10 s), 1 s at 100 each,
    // no minimum speed. Stage E0: both stretched by (2 + 3) / 2 = 2.5 -> 5 s.
    // Stage E1: the layer lasts 5 s (current value), 5 s missing, only E1 is slowed:
    // 2.5 s -> 7.5 s. Result: 2.5 + 7.5 = 10 s (frozen durations would give 11.5 s).
    const Result result = solve(make_layer({
        make_extruder(5, 0, 0, {{1, 100, true}}), make_extruder(10, 0, 0, {{1, 100, true}})}));
    CHECK(result.lines[0][0].speed_mm_per_s == Approx(40.f).margin(1e-3));
    CHECK(result.lines[1][0].speed_mm_per_s == Approx(100.f / 7.5f).margin(1e-3));
    CHECK(result.total_seconds == Approx(10.0).margin(1e-3));
}

TEST_CASE("Solver slows extruders with the same threshold together",
          "[plugins][layer-extrusion-edit][slowdown][slowdown-solver]")
{
    // Two extruders, goal 6 s, 1 s at 100 each, floor 10: common speed f with
    // 2 * (100 / f - 1) = 4 -> f = 33.33 for both lines; total 6 s.
    const Result result = solve(make_layer({
        make_extruder(6, 10, 0, {{1, 100, true}}), make_extruder(6, 10, 0, {{1, 100, true}})}));
    CHECK(result.lines[0][0].speed_mm_per_s == Approx(100.f / 3.f).margin(1e-3));
    CHECK(result.lines[1][0].speed_mm_per_s == Approx(100.f / 3.f).margin(1e-3));
    CHECK(result.total_seconds == Approx(6.0).margin(1e-3));
}

TEST_CASE("Solver treats a disabled extruder and an extruder without slowable line as fixed time",
          "[plugins][layer-extrusion-edit][slowdown][slowdown-solver]")
{
    SECTION("threshold 0")
    {
        // E0 is disabled: its adjustable line keeps its speed and adds 1 s.
        // E1 (goal 8 s) takes 1 s at 100 -> 7 s -> f = 100 / 7 = 14.29 (above the floor 10).
        const Result result = solve(make_layer({
            make_extruder(0, 10, 0, {{1, 100, true}}), make_extruder(8, 10, 0, {{1, 100, true}})}));
        CHECK_FALSE(result.lines[0][0].slowed);
        CHECK(result.lines[1][0].speed_mm_per_s == Approx(100.f / 7.f).margin(1e-3));
        CHECK(result.total_seconds == Approx(8.0).margin(1e-3));
    }
    SECTION("positive threshold but nothing to slow")
    {
        // E0 (goal 5 s) has only a non-adjustable line and is not part of the equalization;
        // E1 (goal 8 s): 2 + 1 = 3 s, 5 s missing -> 1 * (100 / f - 1) = 5 -> f = 16.67.
        const Result result = solve(make_layer({
            make_extruder(5, 10, 0, {{2, 100, false}}), make_extruder(8, 10, 0, {{1, 100, true}})}));
        CHECK_FALSE(result.lines[0][0].slowed);
        CHECK(result.lines[1][0].speed_mm_per_s == Approx(100.f / 6.f).margin(1e-3));
        CHECK(result.total_seconds == Approx(8.0).margin(1e-3));
    }
}

TEST_CASE("Solver is deterministic and does not modify its input",
          "[plugins][layer-extrusion-edit][slowdown][slowdown-solver]")
{
    const Layer layer = make_layer({make_extruder(10, 10, 0, {{1, 80, true}, {1, 80, true}, {1, 60, true}})});
    const Result first = solve(layer);
    const Result second = solve(layer);
    REQUIRE(first.lines[0].size() == 3);
    for (size_t idx = 0; idx < 3; ++idx)
        CHECK(first.lines[0][idx].speed_mm_per_s == second.lines[0][idx].speed_mm_per_s);
    CHECK(first.total_seconds == second.total_seconds);
    CHECK(layer.extruders[0].lines[0].speed_mm_per_s == 80.f);
}

TEST_CASE("Solver refuses invalid numbers with a diagnostic",
          "[plugins][layer-extrusion-edit][slowdown][slowdown-solver]")
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_THROWS_AS(solve(make_layer({make_extruder(5, -1, 0, {{1, 100, true}})})), std::invalid_argument);
    CHECK_THROWS_AS(solve(make_layer({make_extruder(5, 10, 1.5, {{1, 100, true}})})), std::invalid_argument);
    CHECK_THROWS_AS(solve(make_layer({make_extruder(5, 10, -0.1, {{1, 100, true}})})), std::invalid_argument);
    CHECK_THROWS_AS(solve(make_layer({make_extruder(nan, 10, 0, {{1, 100, true}})})), std::invalid_argument);
    CHECK_THROWS_AS(solve(make_layer({make_extruder(-1, 10, 0, {{1, 100, true}})})), std::invalid_argument);
    CHECK_THROWS_AS(solve(make_layer({make_extruder(5, 10, 0, {{-1, 100, true}})})), std::invalid_argument);
    CHECK_THROWS_AS(solve(make_layer({make_extruder(5, 10, 0, {{1, 100, true}})}, -2.0)), std::invalid_argument);
    CHECK_THROWS_WITH(solve(make_layer({make_extruder(5, -1, 0, {})})), Catch::Contains("min_print_speed"));
}

TEST_CASE("Solver reaches a reachable goal when a reduction cap saturates before the others",
          "[plugins][layer-extrusion-edit][slowdown][slowdown-solver]")
{
    // E0 (goal 5 s, no bound) and E1 (goal 8 s, reduction 50 %: at most 2 s), 1 s at 100 each,
    // no minimum speed. Stage 1 needs 3 more seconds: with a common factor r,
    // (r - 1) + (min(2, r) - 1) = 3 gives r = 3, so E0 lasts 3 s and E1 stops at its cap
    // of 2 s: 5 s in all (a single uncapped factor of 2.5 would give 4.5 s).
    // Stage 2 cannot reach 8 s (E1 is already at its cap), so nothing changes.
    const Result result = solve(make_layer({
        make_extruder(5, 0, 0, {{1, 100, true}}), make_extruder(8, 0, 0.5, {{1, 100, true}})}));
    CHECK(result.lines[0][0].speed_mm_per_s == Approx(100.f / 3.f).margin(1e-3));
    CHECK(result.lines[1][0].speed_mm_per_s == Approx(50.f).margin(1e-3));
    CHECK(result.total_seconds == Approx(5.0).margin(1e-3));
}

TEST_CASE("Solver reaches the goal with extruders that have different minimum speeds",
          "[plugins][layer-extrusion-edit][slowdown][slowdown-solver]")
{
    // E0 (floor 40) and E1 (floor 5), goal 6 s, 1 s at 100 each: 4 s missing.
    // Both lines go to 40 mm/s (3 s gained), then E1 alone provides the last second:
    // 2.5 * (40 / f - 1) = 1 gives f = 28.57. Total 2.5 + 3.5 = 6 s.
    const Result result = solve(make_layer({
        make_extruder(6, 40, 0, {{1, 100, true}}), make_extruder(6, 5, 0, {{1, 100, true}})}));
    CHECK(result.lines[0][0].speed_mm_per_s == Approx(40.f).margin(1e-3));
    CHECK(result.lines[1][0].speed_mm_per_s == Approx(40.f / 1.4f).margin(1e-3));
    CHECK(result.total_seconds == Approx(6.0).margin(1e-3));
}

TEST_CASE("Solver goes to the caps when a proportional range cannot reach the goal",
          "[plugins][layer-extrusion-edit][slowdown][slowdown-solver]")
{
    // No minimum speed, reduction 50 %: both lines are capped at 2 s, so 4 s at most.
    const Result result = solve(make_layer({make_extruder(100, 0, 0.5, {{1, 100, true}, {1, 80, true}})}));
    CHECK(result.lines[0][0].speed_mm_per_s == Approx(50.f).margin(1e-3));
    CHECK(result.lines[0][1].speed_mm_per_s == Approx(40.f).margin(1e-3));
    CHECK(result.total_seconds == Approx(4.0).margin(1e-3));
}

TEST_CASE("Solver finds the factor when all lines are bounded and the first guess falls short",
          "[plugins][layer-extrusion-edit][slowdown][slowdown-solver]")
{
    // E0 (goal 6 s, reduction 90 %: up to 10 s) and E1 (goal 7 s, reduction 50 %: up to 2 s),
    // no minimum speed, 1 s at 100 each. Stage 1 needs 4 more seconds. The uncapped factor 3
    // only gives (3 - 1) + (2 - 1) = 3 s, but the bounds allow 9 + 1 s in all, so the
    // answer is the factor 4: E0 lasts 4 s (25 mm/s), E1 stops at its cap of 2 s (50 mm/s).
    // Stage 2 (goal 7 s) cannot gain more: E1 is capped and E0 is no longer in its range.
    const Result result = solve(make_layer({
        make_extruder(6, 0, 0.9, {{1, 100, true}}), make_extruder(7, 0, 0.5, {{1, 100, true}})}));
    CHECK(result.lines[0][0].speed_mm_per_s == Approx(25.f).margin(1e-3));
    CHECK(result.lines[1][0].speed_mm_per_s == Approx(50.f).margin(1e-3));
    CHECK(result.total_seconds == Approx(6.0).margin(1e-3));
}

TEST_CASE("Solver matches legacy CoolingBuffer when neither model difference applies",
          "[plugins][layer-extrusion-edit][slowdown][slowdown-legacy]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    for (double goal : {1.0, 2.5, 10.0, 1000.0}) {
        INFO(goal);
        const Result result = solve(make_layer({make_extruder(goal, 10., 0.9,
            {{1., 100.f, true}, {1., 50.f, true}})}));
        Slic3r::DynamicPrintConfig dynamic = Slic3r::DynamicPrintConfig::full_print_config();
        dynamic.set_deserialize_strict({{"slowdown_below_layer_time", std::to_string(goal)},
            {"min_print_speed", "10"}, {"max_speed_reduction", "90%"},
            {"fan_below_layer_time", "0"}, {"disable_fan_first_layers", "0"}});
        Slic3r::Print print;
        Slic3r::Model model;
        print.apply(model, dynamic);
        Slic3r::GCodeGenerator generator;
        generator.apply_print_configs(print);
        generator.writer().set_extruders({0});
        generator.writer().set_tool(0);
        Slic3r::CoolingBuffer cooling(generator);
        cooling.reset(Slic3r::Vec3d::Zero());
        // A role marker opens each scope that _EXTRUDE_END closes. Feedrate
        // markers alone do not open scopes in the legacy writer's protocol.
        const std::string role_start = std::string(";_EXTRUDETYPE_") +
            char('A' + uint8_t(Slic3r::GCodeExtrusionRole::Perimeter)) + "\n";
        const std::string output = cooling.process_layer(
            role_start + "G1 F6000;_EXTRUDE_SET_SPEED\nG1 X100 E1\n;_EXTRUDE_END\n" +
            role_start + "G1 F3000;_EXTRUDE_SET_SPEED\nG1 X150 E2\n;_EXTRUDE_END\n", 0, true);
        std::vector<double> speeds;
        double current_speed = 0.0;
        std::istringstream lines(output);
        std::string line;
        while (std::getline(lines, line)) {
            std::istringstream words(line.substr(0, line.find(';')));
            std::string command;
            words >> command;
            if (command != "G1") continue;
            std::string word;
            bool moves = false;
            while (words >> word) {
                if (word.front() == 'F') current_speed = std::stod(word.substr(1)) / 60.;
                if (word.front() == 'X') moves = true;
            }
            if (moves) speeds.push_back(current_speed);
        }
        REQUIRE(speeds.size() == 2);
        // CoolingBuffer emits integer mm/min feedrates, losing at most one unit.
        CHECK(speeds[0] == Approx(result.lines[0][0].speed_mm_per_s).margin(1. / 60.));
        CHECK(speeds[1] == Approx(result.lines[0][1].speed_mm_per_s).margin(1. / 60.));
        const double timing_quantization = 150. * (1. / 60.) / (10. * (10. - 1. / 60.));
        CHECK(100. / speeds[0] + 50. / speeds[1] == Approx(result.total_seconds).margin(timing_quantization));
    }
}
