///|/ Released under the GNU Affero General Public License, version 3 or later.

#include "PressureKinematics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <limits>
#include <stdexcept>

// Standalone analytical checks for the normalized dominant-axis motion solver.
// Distances are mm, speeds mm/s, accelerations mm/s^2, and durations seconds.
// Literal analytical expectations are independent of the production solver.
// Explicit checks remain active in Release; unexpected exceptions fail the case.
//
// Call tree:
//   main: run analytical and rejection cases, return nonzero on any failure.
//     test_profiles: exercise triangles, trapezoids, and rounded boundaries.
//       expect: compare phases and check all motion invariants.
//         check: count and print failures.
//         near: compare finite numbers within a relative roundoff allowance.
//     test_rejections: exercise every input guard, including nonfinite values.
//       reject: require std::invalid_argument for the supplied movement.
//         check: count and print failures.

// Runs all checks and returns failure status even when assertions are compiled out.
int main();

namespace {
using namespace nematx_pressure;

int failures = 0;
int checks = 0;
int cases = 0;

// Runs independent closed-form profiles and selected floating-point boundaries.
void test_profiles();
// Changes one input at a time and verifies every guard rejects with the documented exception.
void test_rejections();
// Checks exact phase counts, analytical values, and invariants; exceptions count as failures.
void expect(const char *name, const Motion &motion, std::initializer_list<Phase> expected);
// Requires invalid_argument, diagnosing an accepted input or an unexpected exception.
void reject(const char *name, const Motion &motion);
// Records a checked condition and prints the named case and invariant on failure.
void check(bool condition, const char *name, const char *message);
// Compares finite values within the given number of relative double epsilons, with no absolute floor.
bool near(double actual, double expected, double epsilons = 16.);
} // namespace

int main()
{
    test_profiles();
    test_rejections();
    std::printf("pressure kinematics: %d cases, %d checks, %d failures\n", cases, checks, failures);
    return failures == 0 ? 0 : 1;
}

namespace {

void test_profiles()
{
    const double root2 = std::sqrt(2.);
    const double root30 = std::sqrt(30.);
    const double root116over3 = std::sqrt(116. / 3.);
    const double root155over3 = std::sqrt(155. / 3.);
    expect("long trapezoid", {30., 10., 0., 0., 5., 5.}, {
        {PhaseKind::Accel, 10., 2., 0., 10.},
        {PhaseKind::Steady, 10., 1., 10., 10.},
        {PhaseKind::Decel, 10., 2., 10., 0.}});
    expect("short triangle", {1., 100., 0., 0., 2., 2.}, {
        {PhaseKind::Accel, .5, 1. / root2, 0., root2},
        {PhaseKind::Decel, .5, 1. / root2, root2, 0.}});
    expect("unequal endpoints", {10., 100., 4., 2., 2., 2.}, {
        {PhaseKind::Accel, 3.5, (root30 - 4.) / 2., 4., root30},
        {PhaseKind::Decel, 6.5, (root30 - 2.) / 2., root30, 2.}});
    expect("unequal acceleration magnitudes", {10., 100., 4., 2., 2., 4.}, {
        {PhaseKind::Accel, 17. / 3., (root116over3 - 4.) / 2., 4., root116over3},
        {PhaseKind::Decel, 13. / 3., (root116over3 - 2.) / 4., root116over3, 2.}});
    expect("asymmetric trapezoid remainder", {100., 10., 0., 5., 2., 4.}, {
        {PhaseKind::Accel, 25., 5., 0., 10.},
        {PhaseKind::Steady, 65.625, 6.5625, 10., 10.},
        {PhaseKind::Decel, 9.375, 1.25, 10., 5.}});
    expect("equal nonzero endpoints", {10., 10., 5., 5., 2., 4.}, {
        {PhaseKind::Accel, 20. / 3., (root155over3 - 5.) / 2., 5., root155over3},
        {PhaseKind::Decel, 10. / 3., (root155over3 - 5.) / 4., root155over3, 5.}});
    for (double limit : {10., 20.}) {
        expect("acceleration only", {10., limit, 0., 10., 5., 5.}, {
            {PhaseKind::Accel, 10., 2., 0., 10.}});
        expect("deceleration only", {10., limit, 10., 0., 5., 5.}, {
            {PhaseKind::Decel, 10., 2., 10., 0.}});
    }
    expect("steady only", {10., 5., 5., 5., 2., 4.}, {
        {PhaseKind::Steady, 10., 2., 5., 5.}});
    expect("acceleration then steady", {15., 10., 0., 10., 5., 5.}, {
        {PhaseKind::Accel, 10., 2., 0., 10.}, {PhaseKind::Steady, 5., .5, 10., 10.}});
    expect("steady then deceleration", {15., 10., 10., 0., 5., 5.}, {
        {PhaseKind::Steady, 5., .5, 10., 10.}, {PhaseKind::Decel, 10., 2., 10., 0.}});
    expect("exact triangle plateau boundary", {20., 10., 0., 0., 5., 5.}, {
        {PhaseKind::Accel, 10., 2., 0., 10.}, {PhaseKind::Decel, 10., 2., 10., 0.}});
    const double below20 = std::nextafter(20., 0.);
    const double above20 = std::nextafter(20., 30.);
    const double below_peak = std::sqrt(5. * below20);
    expect("one step below plateau boundary", {below20, 10., 0., 0., 5., 5.}, {
        {PhaseKind::Accel, below20 / 2., below_peak / 5., 0., below_peak},
        {PhaseKind::Decel, below20 / 2., below_peak / 5., below_peak, 0.}});
    expect("one step above plateau boundary", {above20, 10., 0., 0., 5., 5.}, {
        {PhaseKind::Accel, 10., 2., 0., 10.},
        {PhaseKind::Steady, above20 - 20., (above20 - 20.) / 10., 10., 10.},
        {PhaseKind::Decel, 10., 2., 10., 0.}});
    expect("rounded plateau boundary", {2., root2, 0., 0., 1., 1.}, {
        {PhaseKind::Accel, 1., root2, 0., root2}, {PhaseKind::Decel, 1., root2, root2, 0.}});
    expect("rounded acceleration boundary", {1., 20., 0., root2, 1., 1.}, {
        {PhaseKind::Accel, 1., root2, 0., root2}});
    expect("rounded deceleration boundary", {1., 20., root2, 0., 1., 1.}, {
        {PhaseKind::Decel, 1., root2, root2, 0.}});
    const double above10 = 10. * (1. + 1e-9);
    const double small_decel = (above10 - 10.) / 2.;
    const double near_peak = std::sqrt(100. + 10. * small_decel);
    expect("just above endpoint reachability", {above10, 20., 0., 10., 5., 5.}, {
        {PhaseKind::Accel, above10 - small_decel, near_peak / 5., 0., near_peak},
        {PhaseKind::Decel, small_decel, 2. * small_decel / (near_peak + 10.), near_peak, 10.}});
    const double tiny_peak = std::sqrt(5e-6);
    expect("small stop to stop with unused high limit", {1e-6, 100000., 0., 0., 5., 5.}, {
        {PhaseKind::Accel, 5e-7, tiny_peak / 5., 0., tiny_peak},
        {PhaseKind::Decel, 5e-7, tiny_peak / 5., tiny_peak, 0.}});
    expect("small realistic movement", {.002, .2, 0., 0., 20., 20.}, {
        {PhaseKind::Accel, .001, .01, 0., .2}, {PhaseKind::Decel, .001, .01, .2, 0.}});
    expect("small positive steady remainder", {10.05, 10., 10., 0., 5., 5.}, {
        {PhaseKind::Steady, 10.05 - 10., (10.05 - 10.) / 10., 10., 10.},
        {PhaseKind::Decel, 10., 2., 10., 0.}});
    expect("unrepresentable peak change", {1e-16, 10., 5., 5., 2., 4.}, {
        {PhaseKind::Steady, 1e-16, 2e-17, 5., 5.}});
    const double one_step_above10 = std::nextafter(10., 20.);
    expect("peak rounds to final endpoint", {one_step_above10, 20., 0., 10., 5., 5.}, {
        {PhaseKind::Accel, one_step_above10, one_step_above10 / 5., 0., 10.}});
    expect("peak rounds to initial endpoint", {one_step_above10, 20., 10., 0., 5., 5.}, {
        {PhaseKind::Decel, one_step_above10, one_step_above10 / 5., 10., 0.}});
}

void test_rejections()
{
    // Array order matches Motion so each nonfinite value reaches a different input guard.
    const std::array<double, 6> valid = {30., 10., 0., 0., 5., 5.};
    for (std::size_t index = 0; index < valid.size(); ++index) {
        for (double value : {std::numeric_limits<double>::quiet_NaN(),
                             std::numeric_limits<double>::infinity(),
                             -std::numeric_limits<double>::infinity()}) {
            std::array<double, 6> input = valid;
            input[index] = value;
            char name[100];
            std::snprintf(name, sizeof(name), "nonfinite parameter %zu value %g", index, value);
            reject(name, {input[0], input[1], input[2], input[3], input[4], input[5]});
        }
    }
    reject("zero distance", {0., 10., 0., 0., 5., 5.});
    reject("negative distance", {-1., 10., 0., 0., 5., 5.});
    reject("zero limit", {30., 0., 0., 0., 5., 5.});
    reject("negative limit", {30., -1., 0., 0., 5., 5.});
    reject("zero acceleration", {30., 10., 0., 0., 0., 5.});
    reject("negative acceleration", {30., 10., 0., 0., -1., 5.});
    reject("zero deceleration", {30., 10., 0., 0., 5., 0.});
    reject("negative deceleration", {30., 10., 0., 0., 5., -1.});
    reject("negative initial speed", {30., 10., -1., 0., 5., 5.});
    reject("negative final speed", {30., 10., 0., -1., 5., 5.});
    reject("initial speed one step above limit", {30., 10., std::nextafter(10., 20.), 0., 5., 5.});
    reject("final speed one step above limit", {30., 10., 0., std::nextafter(10., 20.), 5., 5.});
    reject("unreachable acceleration", {9., 20., 0., 10., 5., 5.});
    reject("unreachable deceleration", {9., 20., 10., 0., 5., 5.});
    reject("high unused limit cannot hide distance deficit", {10. * (1. - 1e-9), 100000., 0., 10., 5., 5.});
    reject("small deceleration distance deficit", {10. * (1. - 1e-9), 100000., 10., 0., 5., 5.});
}

void expect(const char *name, const Motion &motion, std::initializer_list<Phase> expected)
{
    ++cases;
    try {
        const Profile actual = solve_phases(motion);
        check(actual.count == expected.size(), name, "exact phase count");
        check(actual.count > 0 && actual.count <= 3, name, "one to three phases");
        if (actual.count == 0 || actual.count > 3)
            return;
        check(actual.phases[0].speed_start == motion.speed_start, name, "exact initial endpoint");
        check(actual.phases[actual.count - 1].speed_end == motion.speed_end, name, "exact final endpoint");
        double sum_distance = 0.;
        const Phase *wanted = expected.begin();
        for (std::size_t i = 0; i < actual.count; ++i) {
            const Phase &phase = actual.phases[i];
            sum_distance += phase.distance;
            check(std::isfinite(phase.distance) && phase.distance > 0., name, "positive finite distance");
            check(std::isfinite(phase.duration) && phase.duration > 0., name, "positive finite duration");
            check(phase.speed_start >= 0. && phase.speed_start <= motion.speed_limit &&
                  phase.speed_end >= 0. && phase.speed_end <= motion.speed_limit, name, "bounded speeds");
            if (i > 0)
                check(phase.speed_start == actual.phases[i - 1].speed_end, name, "exact speed continuity");
            check(near(phase.duration, 2. * phase.distance / (phase.speed_start + phase.speed_end)), name, "duration law");
            const double start_squared = phase.speed_start * phase.speed_start;
            const double end_squared = phase.speed_end * phase.speed_end;
            if (phase.kind == PhaseKind::Accel) {
                check(phase.speed_end > phase.speed_start, name, "strictly increasing speed");
                check(near(end_squared, start_squared + 2. * motion.acceleration * phase.distance), name, "acceleration law");
            } else if (phase.kind == PhaseKind::Decel) {
                check(phase.speed_start > phase.speed_end, name, "strictly decreasing speed");
                check(near(start_squared, end_squared + 2. * motion.deceleration * phase.distance), name, "deceleration law");
            } else {
                check(phase.speed_start == phase.speed_end && phase.speed_start > 0., name, "positive steady speed");
            }
            if (i < expected.size()) {
                check(phase.kind == wanted[i].kind, name, "analytical phase kind");
                check(near(phase.distance, wanted[i].distance), name, "analytical distance");
                check(near(phase.duration, wanted[i].duration), name, "analytical duration");
                check(near(phase.speed_start, wanted[i].speed_start) && near(phase.speed_end, wanted[i].speed_end), name, "analytical speeds");
            }
        }
        check(near(sum_distance, motion.distance, 8.), name, "distance conserved within eight epsilons");
    } catch (const std::exception &error) {
        std::printf("FAILED: %s: unexpected exception: %s\n", name, error.what());
        ++failures;
    } catch (...) {
        check(false, name, "unexpected nonstandard exception");
    }
}

void reject(const char *name, const Motion &motion)
{
    ++cases;
    try {
        solve_phases(motion);
        check(false, name, "invalid input was accepted");
    } catch (const std::invalid_argument &) {
        check(true, name, "invalid_argument");
    } catch (...) {
        check(false, name, "unexpected exception type");
    }
}

void check(bool condition, const char *name, const char *message)
{
    ++checks;
    if (!condition) {
        std::printf("FAILED: %s: %s\n", name, message);
        ++failures;
    }
}

bool near(double actual, double expected, double epsilons)
{
    return std::isfinite(actual) && std::isfinite(expected) &&
        std::abs(actual - expected) <= epsilons * std::numeric_limits<double>::epsilon() *
                                      std::max(std::abs(actual), std::abs(expected));
}
} // namespace
