///|/ Released under the GNU Affero General Public License, version 3 or later.

#include "PressureExtrusion.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <stdexcept>

#include "libslic3r/Api/plugin/cpp/gcode/GCodeFormatter.hpp"

// Standalone checks of corrected filament positions, not a text writer or parser.
// Independent integer tick expectations describe encodable E at fixed precision.
// Each delta is also requantized by the real SDK formatter to check that its
// floating-point subtraction survives output rounding. Release keeps all checks.
// Nominal endpoints stand for source E words or unrounded phase interpolation;
// no lost private volume, nominal accounting or geometric merging is simulated.
//
// Call tree:
//   main: run numeric groups and report failures with nonzero exit status.
//     test_law: distinguish model speed from filament speed in both output forms.
//     test_factors: release old charge through changes of K, including zero.
//     test_quantization: retain cumulative nominal fractions and pressure balance.
//     test_origins: separate relative sums from external absolute reset values.
//     test_cycles: persist across layers and restart absolute state correctly.
//     test_repetition: check integer balances over 1000 charged cycles.
//     test_rounding: check signed binary ties, adjacent doubles and precisions.
//     test_rejection: reject invalid inputs/rebase without state mutation.
//       expect: check literal ticks, SDK delta encoding and analytical differences.
//       same: compare continuations with a control that saw no rejected operation.
//       ticks: encode a value by independent round(value*scale).
//       check / near: count conditions and allow local analytical roundoff only.

// Runs all numeric groups and makes unexpected exceptions visible failures.
int main();

namespace {
using namespace nematx_pressure;
using slic3r_api::GCodeGeneration::GCodeFormatter;
int failures = 0;
int checks = 0;
int cases = 0;

// Checks the 2 mm charge at model 100 mm/s, despite filament speed 4 mm/s.
void test_law();
// Checks per-movement K changes and zero nominal increments with and without charge.
void test_factors();
// Checks cumulative coarse rounding, including the negative pressure delta.
void test_quantization();
// Checks relative zero-based sums and positive/negative off-grid absolute origins.
void test_origins();
// Checks two closed cycles without resetting nominal or emitted positions.
void test_cycles();
// Checks long integer-tick balances and independent states with owned results.
void test_repetition();
// Checks half-away-from-zero ties and the supported precision/magnitude endpoints.
void test_rounding();
// Checks constructor/input/rebase errors and unchanged subsequent results.
void test_rejection();
// Checks both encodable output forms and analytical nominal/charge differences.
void expect(const ExtrusionResult &result, int digits, double scale,
            std::int64_t position_ticks, std::int64_t delta_ticks,
            double nominal_delta, double pressure, double pressure_delta);
// Checks every owned result field for exact equality to an unaffected control.
void same(const ExtrusionResult &actual, const ExtrusionResult &control);
// Converts an encodable value into integer ticks independently of the production quantizer.
std::int64_t ticks(double value, double scale);
// Counts a condition and prints a diagnostic on failure, in every build mode.
void check(bool condition, const char *message);
// Compares analytical differences with a local 16-epsilon allowance.
bool near(double actual, double expected);
} // namespace

int main()
{
    try {
        test_law();
        test_factors();
        test_quantization();
        test_origins();
        test_cycles();
        test_repetition();
        test_rounding();
        test_rejection();
    } catch (const std::exception &error) {
        std::printf("FAILED: unexpected exception: %s\n", error.what());
        ++failures;
    } catch (...) {
        check(false, "unexpected nonstandard exception");
    }
    std::printf("pressure extrusion: %d cases, %d checks, %d failures\n", cases, checks, failures);
    return failures == 0 ? 0 : 1;
}

namespace {

void test_law()
{
    ++cases;
    Extrusion absolute(2, 0.);
    Extrusion relative(2, 0.);
    const std::array<double, 3> nominal = {1., 1.5, 2.};
    const std::array<double, 3> speed = {100., 100., 0.};
    const std::array<std::int64_t, 3> positions = {300, 350, 200};
    const std::array<std::int64_t, 3> deltas = {300, 50, -150};
    const std::array<double, 3> nominal_deltas = {1., .5, .5};
    const std::array<double, 3> pressures = {2., 2., 0.};
    const std::array<double, 3> changes = {2., 0., -2.};
    std::int64_t relative_sum = 0;
    for (std::size_t i = 0; i < nominal.size(); ++i) {
        const ExtrusionResult result = absolute.advance(nominal[i], speed[i], .02);
        const ExtrusionResult rel = relative.advance(nominal[i], speed[i], .02);
        same(result, rel);
        expect(result, 2, 100., positions[i], deltas[i], nominal_deltas[i], pressures[i], changes[i]);
        relative_sum += ticks(rel.delta, 100.);
        check(relative_sum == positions[i], "absolute position equals encoded relative sum in the same frame");
    }
    check(relative_sum == 200, "closed cycle emits exactly 2 mm, not a 0.08 mm filament-speed charge");
}

void test_factors()
{
    ++cases;
    Extrusion state(2, 0.);
    expect(state.advance(1., 100., .02), 2, 100., 300, 300, 1., 2., 2.);
    expect(state.advance(2., 100., .01), 2, 100., 300, 0, 1., 1., -1.);
    expect(state.advance(3., 100., 0.), 2, 100., 300, 0, 1., 0., -1.);
    expect(state.advance(4., 0., .02), 2, 100., 400, 100, 1., 0., 0.);
    ++cases;
    Extrusion uncharged(2, 0.);
    expect(uncharged.advance(0., 100., 0.), 2, 100., 0, 0, 0., 0., 0.);
    expect(uncharged.advance(0., 0., 0.), 2, 100., 0, 0, 0., 0., 0.);
    expect(uncharged.advance(.5, 100., 0.), 2, 100., 50, 50, .5, 0., 0.);
    ++cases;
    Extrusion zero_nominal(2, 0.);
    const ExtrusionResult load = zero_nominal.advance(0., 100., .02);
    const ExtrusionResult close = zero_nominal.advance(0., 0., .02);
    expect(load, 2, 100., 200, 200, 0., 2., 2.);
    expect(close, 2, 100., 0, -200, 0., 0., -2.);
    check(ticks(load.delta, 100.) + ticks(close.delta, 100.) == 0,
          "zero nominal charged cycle balances to zero encoded ticks");
    ++cases;
    Extrusion constant_nominal(2, 0.);
    expect(constant_nominal.advance(1., 100., .02), 2, 100., 300, 300, 1., 2., 2.);
    expect(constant_nominal.advance(1., 100., 0.), 2, 100., 100, -200, 0., 0., -2.);
}

void test_quantization()
{
    ++cases;
    Extrusion nominal(2, 0.);
    const ExtrusionResult a = nominal.advance(.004, 100., 0.);
    const ExtrusionResult b = nominal.advance(.008, 100., 0.);
    const ExtrusionResult c = nominal.advance(.012, 0., 0.);
    expect(a, 2, 100., 0, 0, .004, 0., 0.);
    expect(b, 2, 100., 1, 1, .004, 0., 0.);
    expect(c, 2, 100., 1, 0, .004, 0., 0.);
    check(ticks(a.delta, 100.) + ticks(b.delta, 100.) + ticks(c.delta, 100.) == 1,
          "three nominal fractions retain one encoded tick");
    ++cases;
    Extrusion charged(2, 0.);
    const ExtrusionResult load = charged.advance(.004, 100., .02);
    const ExtrusionResult close = charged.advance(.008, 0., .02);
    expect(load, 2, 100., 200, 200, .004, 2., 2.);
    expect(close, 2, 100., 1, -199, .004, 0., -2.);
    check(ticks(load.delta, 100.) + ticks(close.delta, 100.) == 1,
          "pressure closure returns -199 ticks rather than rounding an increment to -200");
}

void test_origins()
{
    ++cases;
    // An external machine SET_POSITION -1.25 does not define the relative sum.
    Extrusion relative(1, 0.);
    relative.rebase(0.);
    expect(relative.advance(.5, 100., 0.), 1, 10., 5, 5, .5, 0., 0.);
    expect(relative.advance(1., 100., 0.), 1, 10., 10, 5, .5, 0., 0.);
    expect(relative.advance(1.5, 0., 0.), 1, 10., 15, 5, .5, 0., 0.);
    ++cases;
    Extrusion positive(2, 5.125);
    expect(positive.advance(5.125, 0., 0.), 2, 100., 513, 0, 0., 0., 0.);
    expect(positive.advance(5.625, 100., .02), 2, 100., 763, 250, .5, 2., 2.);
    expect(positive.advance(6.125, 0., .02), 2, 100., 613, -150, .5, 0., -2.);
    ++cases;
    Extrusion negative(1, -1.25);
    expect(negative.advance(-1.25, 0., 0.), 1, 10., -13, 0, 0., 0., 0.);
    expect(negative.advance(-.75, 0., 0.), 1, 10., -8, 5, .5, 0., 0.);
    expect(negative.advance(-.25, 0., 0.), 1, 10., -3, 5, .5, 0., 0.);
    expect(negative.advance(.25, 0., 0.), 1, 10., 3, 6, .5, 0., 0.);
    negative.rebase(5.125);
    expect(negative.advance(5.125, 0., 0.), 1, 10., 51, 0, 0., 0., 0.);
    negative.rebase(-1.25);
    expect(negative.advance(-1.25, 0., 0.), 1, 10., -13, 0, 0., 0., 0.);
}

void test_cycles()
{
    // Layer closure changes C, not the cumulative nominal or output frame.
    for (double origin : {0., 5.}) {
        ++cases;
        Extrusion state(2, origin);
        const std::int64_t baseline = origin == 0. ? 0 : 500;
        std::int64_t sum = 0;
        for (int cycle = 0; cycle < 2; ++cycle) {
            const double start = origin + static_cast<double>(cycle);
            const ExtrusionResult load = state.advance(start + .5, 100., .02);
            const ExtrusionResult close = state.advance(start + 1., 0., .02);
            expect(load, 2, 100., baseline + cycle * 100 + 250, 250, .5, 2., 2.);
            expect(close, 2, 100., baseline + (cycle + 1) * 100, -150, .5, 0., -2.);
            sum += ticks(load.delta, 100.) + ticks(close.delta, 100.);
            check(sum == (cycle + 1) * 100, "closed layer retains cumulative encoded balance");
        }
        if (origin != 0.) {
            Extrusion recreated(2, 7.);
            const ExtrusionResult recreated_result = recreated.advance(7.2, 0., .02);
            expect(recreated_result, 2, 100., 720, 20, .2, 0., 0.);
            same(state.advance(7.2, 0., .02), recreated_result);
        }
    }
}

void test_repetition()
{
    Extrusion state(2, 0.);
    std::int64_t sum = 0;
    std::int64_t previous = 0;
    for (int cycle = 0; cycle < 1000; ++cycle) {
        ++cases;
        for (int phase = 1; phase <= 3; ++phase) {
            const int step = cycle * 3 + phase;
            const double nominal = static_cast<double>(step) * .004;
            const double pressure = phase == 1 ? 2. : (phase == 2 ? 1. : 0.);
            const double previous_pressure = phase == 1 ? 0. : (phase == 2 ? 2. : 1.);
            const ExtrusionResult result = state.advance(nominal, phase == 3 ? 0. : 100.,
                                                          phase == 2 ? .01 : .02);
            // Integer rounding of 4*step thousandths to hundredths is independent of the output quantizer.
            const std::int64_t expected = (4 * step + 5) / 10 + (phase == 1 ? 200 : (phase == 2 ? 100 : 0));
            expect(result, 2, 100., expected, expected - previous, .004, pressure, pressure - previous_pressure);
            sum += ticks(result.delta, 100.);
            previous = expected;
        }
        check(sum == (12 * (cycle + 1) + 5) / 10, "each closed cycle balances to integer nominal ticks");
    }
    check(sum == 1200, "1000 cycles and 3000 phases emit exactly 1200 ticks");
    ++cases;
    Extrusion first(2, 0.);
    Extrusion second(2, 10.);
    const ExtrusionResult owned = first.advance(1., 100., .02);
    const ExtrusionResult other = second.advance(10.5, 50., .01);
    expect(first.advance(2., 0., .02), 2, 100., 200, -100, 1., 0., -2.);
    expect(second.advance(11., 0., .01), 2, 100., 1100, 0, .5, 0., -.5);
    expect(owned, 2, 100., 300, 300, 1., 2., 2.);
    expect(other, 2, 100., 1100, 100, .5, .5, .5);
}

void test_rounding()
{
    ++cases;
    Extrusion positive(2, 0.);
    expect(positive.advance(.125, 0., 0.), 2, 100., 13, 13, .125, 0., 0.);
    ++cases;
    Extrusion negative(2, -.25);
    expect(negative.advance(-.125, 0., 0.), 2, 100., -13, 12, .125, 0., 0.);
    ++cases;
    Extrusion below(2, 0.);
    const double adjacent = std::nextafter(.125, 0.);
    expect(below.advance(adjacent, 0., 0.), 2, 100., 12, 12, adjacent, 0., 0.);
    ++cases;
    Extrusion whole(0, 0.);
    expect(whole.advance(.5, 0., 0.), 0, 1., 1, 1, .5, 0., 0.);
    ++cases;
    Extrusion fine(9, 0.);
    expect(fine.advance(.000000001, 0., 0.), 9, 1e9, 1, 1, .000000001, 0., 0.);
    ++cases;
    Extrusion magnitude(9, 99999.);
    expect(magnitude.advance(99999.000000001, 0., 0.), 9, 1e9, 99999000000001LL, 1,
           99999.000000001 - 99999., 0., 0.);
    expect(magnitude.advance(99999.5, 10., .02), 9, 1e9, 99999700000000LL, 699999999,
           99999.5 - 99999.000000001, .2, .2);
    expect(magnitude.advance(100000., 0., .02), 9, 1e9, 100000000000000LL, 300000000,
           .5, 0., -.2);
}

void test_rejection()
{
    for (int digits : {-1, 10}) {
        ++cases;
        try {
            const Extrusion invalid(digits, 0.);
            check(false, "invalid precision accepted");
        } catch (const std::invalid_argument &) {
            check(true, "invalid precision rejected");
        }
    }
    const std::array<double, 3> nonfinite = {std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};
    for (double bad : nonfinite) {
        ++cases;
        try {
            const Extrusion invalid(2, bad);
            check(false, "nonfinite origin accepted");
        } catch (const std::invalid_argument &) {
            check(true, "nonfinite origin rejected");
        }
        Extrusion state(2, 0.);
        Extrusion control(2, 0.);
        try {
            state.rebase(bad);
            check(false, "nonfinite rebase accepted");
        } catch (const std::invalid_argument &) {
            check(true, "nonfinite rebase rejected");
        }
        same(state.advance(.5, 0., 0.), control.advance(.5, 0., 0.));
    }
    for (int slot = 0; slot < 3; ++slot) {
        for (double bad : nonfinite) {
            ++cases;
            Extrusion state(2, 0.);
            Extrusion control(2, 0.);
            same(state.advance(1., 100., .02), control.advance(1., 100., .02));
            std::array<double, 3> args = {1.25, 50., .01};
            args[slot] = bad;
            try {
                state.advance(args[0], args[1], args[2]);
                check(false, "nonfinite advance accepted");
            } catch (const std::invalid_argument &) {
                check(true, "nonfinite advance rejected");
            }
            same(state.advance(1.5, 75., .01), control.advance(1.5, 75., .01));
            same(state.advance(2., 0., .02), control.advance(2., 0., .02));
        }
    }
    for (int slot : {1, 2}) {
        ++cases;
        Extrusion state(2, 0.);
        Extrusion control(2, 0.);
        same(state.advance(1., 100., .02), control.advance(1., 100., .02));
        std::array<double, 3> args = {1.25, 50., .01};
        args[slot] = -1.;
        try {
            state.advance(args[0], args[1], args[2]);
            check(false, "negative speed or K accepted");
        } catch (const std::invalid_argument &) {
            check(true, "negative speed or K rejected");
        }
        same(state.advance(1.5, 0., 0.), control.advance(1.5, 0., 0.));
    }
    ++cases;
    Extrusion state(2, 0.);
    Extrusion control(2, 0.);
    same(state.advance(1., 100., .02), control.advance(1., 100., .02));
    try {
        state.rebase(5.125);
        check(false, "open rebase accepted");
    } catch (const std::logic_error &) {
        check(true, "open rebase rejected");
    }
    same(state.advance(1.5, 50., .02), control.advance(1.5, 50., .02));
    same(state.advance(2., 0., .02), control.advance(2., 0., .02));
    state.rebase(-1.25);
    expect(state.advance(-1., 0., .02), 2, 100., -100, 25, .25, 0., 0.);
    // K=0 with positive speed is closed and permits an explicit new frame.
    state.advance(0., 100., 0.);
    state.rebase(5.125);
    expect(state.advance(5.625, 0., 0.), 2, 100., 563, 50, .5, 0., 0.);
}

void expect(const ExtrusionResult &result, int digits, double scale,
            std::int64_t position_ticks, std::int64_t delta_ticks,
            double nominal_delta, double pressure, double pressure_delta)
{
    const GCodeFormatter formatter(3, static_cast<std::uint16_t>(digits));
    check(ticks(result.position, scale) == position_ticks, "position literal ticks");
    check(ticks(result.delta, scale) == delta_ticks, "delta literal ticks");
    check(ticks(formatter.quantize_e(result.position), scale) == position_ticks, "SDK encodable position ticks");
    check(ticks(formatter.quantize_e(result.delta), scale) == delta_ticks, "SDK encodable delta ticks");
    check(near(result.nominal_delta, nominal_delta), "analytical nominal difference");
    check(near(result.pressure, pressure), "analytical charge");
    check(near(result.pressure_delta, pressure_delta), "analytical charge difference");
}

void same(const ExtrusionResult &actual, const ExtrusionResult &control)
{
    check(actual.position == control.position, "control position unchanged");
    check(actual.delta == control.delta, "control delta unchanged");
    check(actual.nominal_delta == control.nominal_delta, "control nominal difference unchanged");
    check(actual.pressure == control.pressure, "control charge unchanged");
    check(actual.pressure_delta == control.pressure_delta, "control charge difference unchanged");
}

std::int64_t ticks(double value, double scale)
{
    return static_cast<std::int64_t>(std::round(value * scale));
}

void check(bool condition, const char *message)
{
    ++checks;
    if (!condition) {
        ++failures;
        std::printf("FAILED: case %d, check %d: %s\n", cases, checks, message);
    }
}

bool near(double actual, double expected)
{
    return std::isfinite(actual) && std::abs(actual - expected) <=
        16. * std::numeric_limits<double>::epsilon() * std::fmax(1., std::abs(expected));
}

} // namespace
