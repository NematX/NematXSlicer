///|/ Released under the GNU Affero General Public License, version 3 or later.

#include "PressurePlanner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <vector>

// Standalone numeric checks for normalized printing cycles and stable publication.
// Literal expectations use dominant-axis mm, mm/s, mm/s^2 and seconds. Checks stay
// active in Release. No G-code or controller behavior is simulated.
//
// Call tree:
//   main: run cases and report a nonzero status on failures or exceptions.
//     test_normalization: check projection and invalid input guards.
//     test_angles: check rounded rays and attained-speed junction caps.
//     test_profiles: check independent analytical multi-move profiles.
//     test_horizons: check strict source-count and dominant-XYZ thresholds.
//     test_stability: check stopped-envelope retention and late continuation.
//     test_lifecycle: check empty closure, reuse and invalid insertion.
//       straight: create a normalized axis-aligned source movement.
//       append: collect owned publications in source order.
//       verify: check exact continuity and the physical phase laws.
//       start / end: read endpoint speeds from a nonempty profile.
//       check / near: diagnose conditions with explicit roundoff allowances.

// Runs every case and reports unexpected exceptions as visible failures.
int main();

namespace {
using namespace nematx_pressure;
int failures = 0;
int checks = 0;
int cases = 0;
const AxisLimits unlimited = {{0., 0., 0., 0.}, {0., 0., 0., 0.}, 0.};

// Checks each axis projection, fallback policy and invalid normalizer argument.
void test_normalization();
// Checks angular symmetry, exact rounded diagonals and caps applied only once.
void test_angles();
// Checks analytical endpoints and phases with uniform and changing acceleration.
void test_profiles();
// Checks count and distance independently on both sides of the strict horizons.
void test_horizons();
// Checks genuine braking insufficiency, recovery and immutable published values.
void test_stability();
// Checks reset and rejection without mutation, including after publication.
void test_lifecycle();
// Creates one normalized X movement with the supplied source ID.
Move straight(double distance, double speed, double acceleration, std::size_t id);
// Appends owned stable movements without changing their values.
void append(std::vector<PlannedMove> &moves, const Publication &publication);
// Checks source order, shared endpoints, distances, speed caps and phase laws.
void verify(const char *name, const std::vector<PlannedMove> &moves, double initial = 0., double final = 0.);
// Reads the first speed of a solved nonempty movement.
double start(const PlannedMove &move);
// Reads the last speed of a solved nonempty movement.
double end(const PlannedMove &move);
// Counts an explicit condition and prints its context when false.
void check(bool condition, const char *name, const char *message);
// Compares finite values with a relative epsilon allowance and no absolute floor.
bool near(double actual, double expected, double epsilons = 64.);
} // namespace

int main()
{
    const char *group = "normalization";
    try {
        test_normalization();
        group = "angles";
        test_angles();
        group = "profiles";
        test_profiles();
        group = "horizons";
        test_horizons();
        group = "stability";
        test_stability();
        group = "lifecycle";
        test_lifecycle();
    } catch (const std::exception &error) {
        std::printf("FAILED: %s: unexpected exception: %s\n", group, error.what());
        ++failures;
    } catch (...) {
        check(false, "main", "unexpected nonstandard exception");
    }
    std::printf("pressure planner: %d cases, %d checks, %d failures\n", cases, checks, failures);
    return failures == 0 ? 0 : 1;
}

namespace {

void test_normalization()
{
    for (std::size_t axis = 0; axis < 4; ++axis) {
        ++cases;
        AxisLimits limits = unlimited;
        limits.speed[axis] = 2.;
        limits.acceleration[axis] = 3.;
        std::array<double, 4> delta = {1., 0., 0., 0.};
        delta[axis] = 10.;
        const Move move = normalize_move(delta[0], delta[1], delta[2], delta[3], 20., 30., limits, axis);
        check(move.distance == 10. && move.speed_limit == 2. && move.acceleration == 3.,
              "axis projection", "dominant axis speed and acceleration");
        check(move.source_index == axis && move.xyz_distance == (axis == 3 ? 1. : 10.),
              "axis projection", "opaque ID and distinct spatial reach");
        for (std::size_t projected = 0; projected < 4; ++projected) {
            check(move.speed_limit * std::abs(delta[projected]) / move.distance <=
                  (limits.speed[projected] > 0. ? limits.speed[projected] : 100000.),
                  "axis projection", "every projected speed respects limit");
            check(move.acceleration * std::abs(delta[projected]) / move.distance <=
                  (limits.acceleration[projected] > 0. ? limits.acceleration[projected] : 100000.),
                  "axis projection", "every projected acceleration respects limit");
        }
    }
    ++cases;
    AxisLimits crossed = {{2., 5., 10., 10.}, {4., 7., 10., 10.}, 0.};
    const Move cross = normalize_move(-10., 10., 0., 1., 10., 10., crossed, 8);
    check(cross.distance == 10. && cross.speed_limit == 2. && cross.acceleration == 4.,
          "crossed axes", "global maximum time is five seconds, not last-axis time");
    ++cases;
    AxisLimits capped = unlimited;
    capped.max_extruding_acceleration = 4.;
    const Move xyz = normalize_move(10., 0., 0., 1., 10., 100., capped, 0);
    const Move edominant = normalize_move(1., 0., 0., 10., 10., 100., capped, 0);
    check(xyz.acceleration == 4. && edominant.acceleration == 40., "extrusion cap", "XYZ only after fallback");
    capped.acceleration[3] = 2.;
    check(normalize_move(1., 0., 0., 10., 10., 100., capped, 0).acceleration == 2.,
          "extrusion cap", "E retains its own lower acceleration");
    ++cases;
    AxisLimits negative = {{-1., -2., -3., -4.}, {-1., -2., -3., -4.}, -1.};
    const Move fallback = normalize_move(10., 0., 0., 0., 200000., 200000., negative, 0);
    check(fallback.speed_limit == 100000. && fallback.acceleration == 100000., "fallback", "negative limits use 100000");
    const Move ignore = normalize_move(10., 0., 0., 0., 10., 100., unlimited, 0);
    check(ignore.acceleration == 100. && xyz.acceleration == 4., "ignore mapping", "all zeros differ from positive extrusion cap");
    ++cases;
    const Move vertical = normalize_move(0., 0., -5., 0., 7., 8., unlimited, 9);
    check(vertical.distance == 5. && vertical.direction_x == 0 && vertical.direction_y == 0,
          "vertical", "zero E printing and zero XY direction are valid");
    const Move old = straight(1., 10., 2., 0);
    const Move changed = straight(1., 10., 8., 1);
    check(old.acceleration == 2. && changed.acceleration == 8., "acceleration values", "later normalization has no retroactive effect");

    // Each slot corresponds to one finite input checked by the normalizer.
    for (std::size_t slot = 0; slot < 15; ++slot) {
        for (double bad : {std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()}) {
            ++cases;
            std::array<double, 15> values = {1., 2., 3., 1., 10., 20., 2., 3., 4., 5., 6., 7., 8., 9., 10.};
            values[slot] = bad;
            const AxisLimits limits = {{values[6], values[7], values[8], values[9]},
                                       {values[10], values[11], values[12], values[13]}, values[14]};
            try {
                normalize_move(values[0], values[1], values[2], values[3], values[4], values[5], limits, 0);
                check(false, "nonfinite", "normalizer accepted nonfinite input");
            } catch (const std::invalid_argument &) {
                check(true, "nonfinite", "invalid_argument");
            }
        }
    }
    for (const std::array<double, 6> &bad : {
            std::array<double, 6>{0., 0., 0., 1., 10., 2.},
            std::array<double, 6>{0., 0., 0., 0., 10., 2.},
            std::array<double, 6>{1., 0., 0., -1., 10., 2.},
            std::array<double, 6>{1., 0., 0., 1., 0., 2.},
            std::array<double, 6>{1., 0., 0., 1., -1., 2.},
            std::array<double, 6>{1., 0., 0., 1., 10., 0.},
            std::array<double, 6>{1., 0., 0., 1., 10., -1.}}) {
        ++cases;
        try {
            normalize_move(bad[0], bad[1], bad[2], bad[3], bad[4], bad[5], unlimited, 0);
            check(false, "invalid normalizer", "invalid input accepted");
        } catch (const std::invalid_argument &) {
            check(true, "invalid normalizer", "invalid_argument");
        }
    }
}

void test_angles()
{
    for (double mirror : {-1., 1.}) {
        for (const std::array<double, 3> &ray : {std::array<double, 3>{1., 0., 1.},
                 std::array<double, 3>{1., 1., .5}, std::array<double, 3>{0., 1., 0.},
                 std::array<double, 3>{-1., 0., 0.}}) {
            ++cases;
            const Move first = normalize_move(mirror, 0., 0., .1, 100., 2., unlimited, 0);
            const Move next = normalize_move(mirror * ray[0], ray[1], 0., .1, 100., 2., unlimited, 1);
            check(near(junction_factor(first, next), ray[2]), "angles", "analytical factor and mirrored ray");
        }
    }
    ++cases;
    const Move diagonal0 = normalize_move(100.2 - 100.1, 200.2 - 200.0, 0., .01, 1., 100., unlimited, 0);
    Move diagonal1 = normalize_move(100.3 - 100.2, 200.4 - 200.2, 0., .01, 1., 100., unlimited, 1);
    check(diagonal0.direction_x == 100000 && diagonal0.direction_y == 200000 &&
          junction_factor(diagonal0, diagonal1) == 1., "rounded diagonal", "one-nanometre rays are exactly straight");
    const Move negative = normalize_move(-.0000005, -.0000015, 1., 0., 1., 2., unlimited, 0);
    const Move positive = normalize_move(.0000005, .0000015, 1., 0., 1., 2., unlimited, 0);
    check(negative.direction_x == -positive.direction_x && negative.direction_y == -positive.direction_y &&
          positive.direction_x == 1 && positive.direction_y == 2, "rounding", "symmetric half-away rounding");
    const Move vertical = normalize_move(0., 0., 1., 0., 1., 1., unlimited, 0);
    check(junction_factor(vertical, diagonal1) == 0. && junction_factor(diagonal0, vertical) == 0.,
          "vertical angle", "zero XY direction forces zero factor");
    Planner diagonal;
    diagonal.push(diagonal0);
    diagonal.push(diagonal1);
    diagonal1.source_index = 2;
    diagonal.push(diagonal1);
    const Publication exact = diagonal.finish();
    check(exact.moves.size() == 3 && exact.moves[1].profile.count == 1 &&
          exact.moves[1].profile.phases[0].kind == PhaseKind::Steady,
          "rounded diagonal", "no angular microphases in middle movement");
    verify("diagonal profiles", exact.moves);

    ++cases;
    Planner attained;
    attained.push(straight(2., 100., 100., 0)); // Open end is exactly 20.
    attained.push(normalize_move(100., 100., 0., 1., 100., 100., unlimited, 1));
    attained.push(normalize_move(100., 100., 0., 1., 100., 100., unlimited, 2));
    const Publication capped = attained.finish();
    check(near(end(capped.moves[0]), 10.) && near(start(capped.moves[1]), 10.),
          "attained cap", "20 times one half, not min(50,20), with no repeated factor");
    verify("attained profiles", capped.moves);
}

void test_profiles()
{
    ++cases;
    Planner one;
    one.push(straight(30., 10., 5., 0));
    const Publication single = one.finish();
    check(single.moves.size() == 1 && single.moves[0].profile.count == 3, "single move", "three analytical phases");
    for (const Phase &phase : single.moves[0].profile.phases)
        check(phase.distance == 10., "single move", "acceleration, plateau and braking each ten mm");
    verify("single move", single.moves);
    ++cases;
    Planner three;
    for (std::size_t id = 0; id < 3; ++id)
        three.push(straight(1., 10., 2., id));
    const Publication triangle = three.finish();
    check(triangle.moves.size() == 3, "three moves", "all sources returned");
    check(end(triangle.moves[0]) == 2. && start(triangle.moves[1]) == 2. && end(triangle.moves[1]) == 2.,
          "three moves", "analytical endpoints 0,2,2,0");
    const Profile &middle = triangle.moves[1].profile;
    check(middle.count == 2 && near(middle.phases[0].speed_end, std::sqrt(6.)) &&
          middle.phases[0].distance == .5 && middle.phases[1].distance == .5,
          "three moves", "central triangular peak sqrt(6), equal half distances");
    verify("three moves", triangle.moves);
    ++cases;
    Planner projected;
    AxisLimits limits = unlimited;
    limits.acceleration[0] = 4.;
    projected.push(normalize_move(100., 0., 0., 1., 10., 10., limits, 0));
    projected.push(normalize_move(20., 0., 0., 1., 10., 10., limits, 1));
    const Publication symmetric = projected.finish();
    const Profile &last = symmetric.moves[1].profile;
    check(start(symmetric.moves[1]) == 10. && last.count == 2 &&
          last.phases[0].kind == PhaseKind::Steady && last.phases[0].distance == 7.5 &&
          last.phases[1].kind == PhaseKind::Decel && last.phases[1].distance == 12.5,
          "projected braking", "projected a=b=4 gives 7.5 plateau and 12.5 braking");
    verify("projected braking", symmetric.moves);
    ++cases;
    Planner squared;
    squared.push(straight(1.25, 10., 2., 0)); // Open arrival to next is sqrt(5).
    squared.push(straight(1., 10., 2., 1));
    const Publication slowed = squared.finish();
    check(start(slowed.moves[1]) == 2. && end(slowed.moves[1]) == 0.,
          "squared braking", "one mm at a=2 stops from 2");
    verify("squared braking", slowed.moves);
    ++cases;
    Planner varied;
    varied.push(straight(2., 10., 4., 0));
    varied.push(straight(1., 10., 1., 1));
    varied.push(straight(.5, 10., 2., 2));
    const Publication variable = varied.finish();
    check(near(end(variable.moves[0]), 2.) && near(end(variable.moves[1]), std::sqrt(2.)),
          "variable acceleration", "backward reaches sqrt(4) then sqrt(2)");
    verify("variable acceleration", variable.moves);
    ++cases;
    Planner rounds;
    for (std::size_t id = 0; id < 8; ++id)
        rounds.push(straight(1., 10., 2., id));
    const Publication multi = rounds.finish();
    for (std::size_t id = 0; id < 8; ++id)
        check(near(end(multi.moves[id]), 2. * std::sqrt(static_cast<double>(std::min(id + 1, 7 - id)))),
              "multiple braking rounds", "independent stop-to-stop endpoint envelope");
    verify("multiple braking rounds", multi.moves);
    ++cases;
    Planner slow_arrival;
    for (std::size_t id = 0; id < 8; ++id)
        slow_arrival.push(straight(1., 10., 2., id));
    slow_arrival.push(straight(100., 1., 2., 8));
    const Publication limited = slow_arrival.finish();
    for (std::size_t id = 0; id < 8; ++id) {
        const double expected = std::min(2. * std::sqrt(static_cast<double>(id + 1)),
                                         std::sqrt(1. + 4. * static_cast<double>(7 - id)));
        check(near(end(limited.moves[id]), expected), "slow arrival", "new speed cap propagates by squared speeds");
        check(end(limited.moves[id]) <= 2. * std::sqrt(static_cast<double>(id + 1)),
              "slow arrival", "existing open endpoint cannot increase");
    }
    verify("slow arrival", limited.moves);
    ++cases;
    Planner spatial;
    spatial.push(normalize_move(0., 0., 2., 4., 10., 2., unlimited, 0));
    const Publication dominant = spatial.finish();
    check(dominant.moves[0].profile.count == 2 && dominant.moves[0].profile.phases[0].distance == 2. &&
          near(dominant.moves[0].profile.phases[0].speed_end, std::sqrt(8.)),
          "E dominant profile", "D=4, not spatial Dxyz=2, determines the triangle");
    verify("E dominant profile", dominant.moves);
}

void test_horizons()
{
    for (std::size_t count : {99U, 100U, 101U}) {
        ++cases;
        Planner planner;
        std::vector<PlannedMove> published;
        for (std::size_t id = 0; id < count; ++id)
            append(published, planner.push(straight(2., 1., 100., id)));
        check(published.size() == (count > 100 ? 1 : 0), "count horizon", "strictly more than 100 sources");
        append(published, planner.finish());
        verify("count horizon", published);
    }
    for (double total : {99., 100., 101.}) {
        ++cases;
        Planner planner;
        for (std::size_t id = 0; id < 100; ++id)
            check(planner.push(straight(.5, 1., 100., id)).moves.empty(), "distance horizon", "count not yet exceeded");
        const Publication result = planner.push(straight(total - 50., 1., 100., 100));
        check(result.moves.size() == (total > 100. ? 1 : 0) && !result.horizon_insufficient,
              "distance horizon", "strictly more than 100 dominant XYZ mm");
    }
    ++cases;
    Planner diagonals;
    for (std::size_t id = 0; id < 101; ++id) {
        const Publication result = diagonals.push(normalize_move(.8, .8, 0., .01, 1., 100., unlimited, id));
        check(result.moves.empty() && !result.horizon_insufficient, "diagonal horizon", "Dxyz is .8, not Euclidean length");
    }
    ++cases;
    Planner extrusion;
    for (std::size_t id = 0; id < 101; ++id)
        check(extrusion.push(normalize_move(.5, 0., 0., 2., 1., 100., unlimited, id)).moves.empty(),
              "E dominant horizon", "D=2 does not replace Dxyz=.5");
    ++cases;
    Planner phases;
    for (std::size_t id = 0; id < 100; ++id)
        check(phases.push(straight(2., 10., .1, id)).moves.empty(), "source counter", "phases do not count as sources");
    const Publication many_phases = phases.finish();
    std::size_t phase_count = 0;
    for (const PlannedMove &move : many_phases.moves)
        phase_count += move.profile.count;
    check(phase_count > 100, "source counter", "multiple phases really occurred");
    check(many_phases.moves.size() == 100, "rounded reachability", "all 100 source movements returned");
    verify("rounded reachability", many_phases.moves);
    double total_distance = 0.;
    for (const PlannedMove &move : many_phases.moves)
        total_distance += move.move.distance;
    check(total_distance == 200., "rounded reachability", "all 200 mm preserved");
    ++cases;
    Planner low;
    Publication low_result;
    for (std::size_t id = 0; id < 101; ++id)
        low_result = low.push(straight(1., 100., .1, id));
    check(low_result.moves.size() == 1 && !low_result.horizon_insufficient &&
          near(end(low_result.moves[0]), std::sqrt(.2)), "uniform low acceleration", "prefix can stop in known suffix");
}

void test_stability()
{
    ++cases;
    Planner planner;
    planner.push(straight(100., 100., 100., 0));
    Publication blocked;
    for (std::size_t id = 1; id <= 100; ++id) {
        blocked = planner.push(straight(.01, 100., .1, id));
        check(blocked.moves.empty(), "insufficient horizon", "no unsafe prefix is published");
    }
    check(blocked.horizon_insufficient, "insufficient horizon", "101 sources and 101 mm cannot stop from 100");
    const Publication recovered = planner.push(straight(100., 100., 100., 101));
    check(recovered.moves.size() == 2 && !recovered.horizon_insufficient,
          "recovered horizon", "exactly two whole sources published, 100 remain");
    check(recovered.moves[0].move.source_index == 0 && recovered.moves[1].move.source_index == 1 &&
          end(recovered.moves[0]) == 100. && start(recovered.moves[1]) == 100. && end(recovered.moves[1]) == 100.,
          "recovered horizon", "fixed 100 mm/s entry and source order");
    const Publication stopped = planner.finish();
    check(stopped.moves.size() == 100 && start(stopped.moves[0]) == 100. && end(stopped.moves.back()) == 0.,
          "recovered closure", "suffix accepts fixed entry and closes at zero");
    std::vector<PlannedMove> all = recovered.moves;
    append(all, stopped);
    verify("recovered closure", all);
    double distance = 0.;
    for (const PlannedMove &move : all)
        distance += move.move.distance;
    check(near(distance, 201.), "recovered closure", "all 201 mm retained");
    check(end(recovered.moves[0]) == 100. && end(recovered.moves[1]) == 100.,
          "recovered closure", "published copies remain unchanged");

    ++cases;
    Planner long_cycle;
    std::vector<PlannedMove> output;
    for (std::size_t id = 0; id < 240; ++id)
        append(output, long_cycle.push(straight(1., 10., 2., id)));
    const std::vector<PlannedMove> prefix = output;
    check(!prefix.empty(), "late turn", "publication occurs before late turn");
    const Move corner = normalize_move(0., 1., 0., .1, 10., 2., unlimited, 240);
    check(junction_factor(straight(1., 10., 2., 239), corner) == 0., "late turn", "corner factor is exactly zero");
    append(output, long_cycle.push(corner));
    append(output, long_cycle.finish());
    verify("late turn", output);
    for (std::size_t id = 0; id < 240; ++id) {
        const double expected = std::min(10., 2. * std::sqrt(static_cast<double>(std::min(id + 1, 239 - id))));
        check(near(end(output[id]), expected), "late turn", "independent uniform-chain envelope");
        if (id < prefix.size()) {
            check(start(prefix[id]) == start(output[id]) && end(prefix[id]) == end(output[id]),
                  "late turn", "published endpoints unchanged");
            check(near(end(prefix[id]), std::min(10., 2. * std::sqrt(static_cast<double>(id + 1)))),
                  "late turn", "published prefix agrees with analytical open acceleration");
        }
    }
    check(end(output[239]) == 0. && start(output[240]) == 0. && end(output[240]) == 0.,
          "late turn", "right angle creates stopped junction and final triangle");
    ++cases;
    // Compare analytical envelopes across independent cycles whose final long
    // movement leaves the preceding acceleration envelope unconstrained by stopping.
    std::vector<double> previous;
    for (std::size_t length = 2; length <= 12; ++length) {
        Planner sequence;
        for (std::size_t id = 0; id < length; ++id)
            sequence.push(straight(1., 10., 2., id));
        sequence.push(straight(100., 10., 2., length));
        const Publication result = sequence.finish();
        for (std::size_t id = 0; id < length; ++id) {
            check(near(end(result.moves[id]), std::min(10., 2. * std::sqrt(static_cast<double>(id + 1)))),
                  "independent cycle envelopes", "endpoints follow the analytical forward envelope");
            if (id < previous.size())
                check(end(result.moves[id]) <= previous[id], "independent cycle envelopes", "shared endpoints do not rise between these cycles");
        }
        previous.clear();
        for (std::size_t id = 0; id < length; ++id)
            previous.push_back(end(result.moves[id]));
    }
}

void test_lifecycle()
{
    ++cases;
    Planner planner;
    check(planner.finish().moves.empty() && !planner.finish().horizon_insufficient, "empty finish", "empty and repeated closure");
    for (std::size_t count : {1U, 3U}) {
        ++cases;
        for (std::size_t id = 0; id < count; ++id)
            planner.push(straight(1., 10., 2., id));
        const Publication result = planner.finish();
        check(result.moves.size() == count && start(result.moves.front()) == 0. && end(result.moves.back()) == 0.,
              "reuse", "new cycle always starts and ends at zero");
        check(planner.finish().moves.empty(), "reuse", "second finish is empty");
    }
    for (std::size_t field = 0; field < 6; ++field) {
        ++cases;
        Planner intact;
        intact.push(straight(1., 10., 2., 0));
        Move invalid = straight(1., 10., 2., 999);
        if (field == 0) invalid.distance = 0.;
        if (field == 1) invalid.xyz_distance = 2.;
        if (field == 2) invalid.speed_limit = -1.;
        if (field == 3) invalid.acceleration = std::numeric_limits<double>::quiet_NaN();
        if (field == 4) invalid.direction_x = std::numeric_limits<std::int64_t>::min();
        if (field == 5) invalid.direction_y = std::numeric_limits<std::int64_t>::min();
        try {
            intact.push(invalid);
            check(false, "invalid push", "invalid normalized move accepted");
        } catch (const std::invalid_argument &) {
            check(true, "invalid push", "invalid_argument before mutation");
        }
        intact.push(straight(1., 10., 2., 1));
        const Publication result = intact.finish();
        check(result.moves.size() == 2 && result.moves[0].move.source_index == 0 && result.moves[1].move.source_index == 1 &&
              end(result.moves[0]) == 2., "invalid push", "valid continuation and analytical closure unchanged");
        verify("invalid push", result.moves);
    }
    ++cases;
    Planner published;
    std::vector<PlannedMove> output;
    for (std::size_t id = 0; id < 101; ++id)
        append(output, published.push(straight(2., 1., 100., id)));
    check(output.size() == 1 && end(output[0]) == 1., "invalid after publication", "fixed prefix exists");
    Move invalid = straight(1., 1., 100., 999);
    invalid.xyz_distance = -1.;
    try {
        published.push(invalid);
        check(false, "invalid after publication", "invalid move accepted");
    } catch (const std::invalid_argument &) {
        check(true, "invalid after publication", "invalid_argument");
    }
    append(output, published.push(straight(2., 1., 100., 101)));
    append(output, published.finish());
    check(output.size() == 102 && end(output[0]) == 1., "invalid after publication", "order and fixed entry preserved");
    verify("invalid after publication", output);
    ++cases;
    published.push(straight(1., 10., 2., 0));
    const Publication reused = published.finish();
    check(reused.moves.size() == 1, "reuse after publication", "one source in the new cycle");
    if (reused.moves.size() == 1)
        check(start(reused.moves[0]) == 0. && end(reused.moves[0]) == 0. && reused.moves[0].move.source_index == 0,
              "reuse after publication", "published entry and source order reset for the new cycle");
    verify("reuse after publication", reused.moves);
}

Move straight(double distance, double speed, double acceleration, std::size_t id)
{
    return normalize_move(distance, 0., 0., distance * .01, speed, acceleration, unlimited, id);
}

void append(std::vector<PlannedMove> &moves, const Publication &publication)
{
    moves.insert(moves.end(), publication.moves.begin(), publication.moves.end());
}

void verify(const char *name, const std::vector<PlannedMove> &moves, double initial, double final)
{
    check(!moves.empty(), name, "nonempty analytical cycle");
    if (moves.empty())
        return;
    check(start(moves.front()) == initial && end(moves.back()) == final, name, "exact cycle endpoints");
    for (std::size_t index = 0; index < moves.size(); ++index) {
        const PlannedMove &move = moves[index];
        check(move.move.source_index == index, name, "source IDs in order without loss");
        check(move.profile.count > 0 && move.profile.count <= 3, name, "one to three phases");
        if (index > 0)
            check(start(move) == end(moves[index - 1]), name, "exact source endpoint continuity");
        double distance = 0.;
        for (std::size_t phase_index = 0; phase_index < move.profile.count; ++phase_index) {
            const Phase &phase = move.profile.phases[phase_index];
            distance += phase.distance;
            check(phase.distance > 0. && std::isfinite(phase.distance) &&
                  phase.duration > 0. && std::isfinite(phase.duration), name, "positive finite phases");
            check(phase.speed_start >= 0. && phase.speed_end >= 0. &&
                  phase.speed_start <= move.move.speed_limit && phase.speed_end <= move.move.speed_limit,
                  name, "nominal limit respected");
            check(near(phase.duration, 2. * phase.distance / (phase.speed_start + phase.speed_end)), name, "duration law");
            if (phase_index > 0)
                check(phase.speed_start == move.profile.phases[phase_index - 1].speed_end, name, "exact phase continuity");
            if (phase.kind == PhaseKind::Steady)
                check(phase.speed_start == phase.speed_end, name, "steady speed unchanged");
            else {
                // Compare squared speeds without subtracting nearly equal values.
                const double start_squared = phase.speed_start * phase.speed_start;
                const double end_squared = phase.speed_end * phase.speed_end;
                const double change = 2. * move.move.acceleration * phase.distance;
                const bool increasing = phase.kind == PhaseKind::Accel;
                const bool direction_valid = increasing ? phase.speed_end > phase.speed_start :
                                                          phase.speed_start > phase.speed_end;
                const bool relation_valid = increasing ? near(end_squared, start_squared + change, 16.) :
                                                         near(start_squared, end_squared + change, 16.);
                check(direction_valid, name, "strict speed direction for phase kind");
                check(relation_valid, name, "squared-speed kinematic relation");
                if (!direction_valid || !relation_valid)
                    std::printf("  source=%zu phase=%zu kind=%d d=%.17g a=%.17g v0=%.17g v1=%.17g\n",
                                index, phase_index, static_cast<int>(phase.kind), phase.distance,
                                move.move.acceleration, phase.speed_start, phase.speed_end);
            }
        }
        check(near(distance, move.move.distance), name, "source distance conserved");
    }
}

double start(const PlannedMove &move)
{
    return move.profile.phases[0].speed_start;
}

double end(const PlannedMove &move)
{
    return move.profile.phases[move.profile.count - 1].speed_end;
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
