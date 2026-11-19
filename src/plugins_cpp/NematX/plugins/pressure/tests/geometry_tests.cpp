///|/ Released under the GNU Affero General Public License, version 3 or later.

#include "PressureExtrusion.hpp"
#include "PressureGeometry.hpp"
#include "PressurePlanner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <stdexcept>

#include "libslic3r/Api/plugin/cpp/gcode/GCodeFormatter.hpp"

// Standalone checks of per-source spatial grouping before any E state advances.
// Literal cuts and integer ticks are independent oracles; the SDK additionally
// checks output-grid round trips. The real phase solver and planner supply valid
// profiles, including a rounding regression with a positive microscopic tail.
// Checks remain active in Release. No file export or stream barrier is simulated.
//
// Call tree:
//   main: run cases, report all failed checks and unexpected exceptions.
//     test_regular: literal three-phase geometry and coarse E balance.
//     test_merging: head, middle, tail and full three-phase fusion.
//     test_axes: mirrored/diagonal/vertical sources and signed binary ties.
//     test_dominant: dominant E distance, not spatial distance, sets fractions.
//     test_planner: feed true Planner publications into geometry and extrusion.
//     test_rounded_fraction: clamp an interpolated endpoint before XYZ rounding.
//     test_nominal: zero nominal and signed origins retain exact final values.
//     test_collapsed: retain an entire source without advancing extrusion state.
//     test_rejection: invalid inputs throw and preserve borrowed source data.
//       make_move: construct a coherent normalized source with the real solver.
//       verify: check partition, continuity, sums, monotonicity and SDK round trip.
//       extrusion: check literal cumulative and relative ticks for final groups.
//       same_source: check borrowed endpoints without treating NaN as equal by ==.
//       ticks / near / check: independent rounding and counted numeric assertions.

// Runs all numeric groups and reports unexpected exceptions as failures.
int main();

namespace {
using namespace nematx_pressure;
using slic3r_api::GCodeGeneration::GCodeFormatter;
int failures = 0;
int checks = 0;
int cases = 0;

// Checks literal 2/6/2 cuts, unrounded nominal interpolation and closed E ticks.
void test_regular();
// Checks precision-driven fusion without losing any original phase or final charge.
void test_merging();
// Checks signed directions, XYZ axes, binary ties and both precision endpoints.
void test_axes();
// Distinguishes dominant E interpolation from interpolation by spatial distance.
void test_dominant();
// Bridges two actual planned moves through geometry and a change of K.
void test_planner();
// Checks real solver microphases when a prefix fraction rounds to one.
void test_rounded_fraction();
// Checks constant nominal and nonzero signed origins without final nominal drift.
void test_nominal();
// Checks owned non-emittable aggregates and status-gated unchanged E state.
void test_collapsed();
// Checks finite invalid-input matrices and precision validation without mutations.
void test_rejection();
// Normalizes coherent deltas and solves a profile with specified endpoint speeds.
PlannedMove make_move(const SourceGeometry &source, double speed, double acceleration,
                      double start_speed = 0., double end_speed = 0.);
// Checks the complete original phase partition and monotone, continuous output grid.
// Throws runtime_error after a failed structural check to prevent unsafe caller reads.
void verify(const PlannedMove &move, const SourceGeometry &source,
            int digits, const GeometryResult &result);
// Checks literal integer positions/deltas and SDK re-encoding for final spatial groups.
void extrusion(const GeometryResult &result, double origin, double k,
               const std::array<std::int64_t, 3> &positions,
               const std::array<std::int64_t, 3> &deltas);
// Compares all source values, accepting paired NaNs only for immutability checks.
bool same_source(const SourceGeometry &left, const SourceGeometry &right);
// Independently rounds a grid value to ticks for literal comparisons.
std::int64_t ticks(double value, double scale);
// Allows only proportional analytical roundoff at sixteen double epsilons.
bool near(double actual, double expected);
// Counts every condition in all configurations and prints failures.
void check(bool condition, const char *message);
} // namespace

int main()
{
    try {
        test_regular();
        test_merging();
        test_axes();
        test_dominant();
        test_planner();
        test_rounded_fraction();
        test_nominal();
        test_collapsed();
        test_rejection();
    } catch (const std::exception &error) {
        std::printf("FAILED: unexpected exception: %s\n", error.what());
        ++failures;
    } catch (...) {
        check(false, "unexpected nonstandard exception");
    }
    std::printf("pressure geometry: %d cases, %d checks, %d failures\n", cases, checks, failures);
    return failures == 0 ? 0 : 1;
}

namespace {
void test_regular()
{
    ++cases;
    const SourceGeometry source = {{0., 0., 0.}, {10., 0., 0.}, 0., .012};
    const PlannedMove move = make_move(source, 2., 1.);
    const GeometryResult result = project_geometry(move, source, 3);
    verify(move, source, 3, result);
    check(result.count == 3, "three representable phases");
    const std::array<double, 3> cuts = {2., 8., 10.};
    const std::array<double, 3> nominal = {.0024, .0096, .012};
    const std::array<double, 3> distance = {2., 6., 2.};
    const std::array<double, 3> duration = {2., 3., 2.};
    const std::array<SegmentKind, 3> kinds = {SegmentKind::Accel, SegmentKind::Steady, SegmentKind::Decel};
    for (std::size_t i = 0; i < 3; ++i) {
        check(result.groups[i].end[0] == cuts[i], "literal X cut");
        check(near(result.groups[i].nominal_end, nominal[i]), "nominal cut retains double fraction");
        check(result.groups[i].nominal_end != std::round(nominal[i] * 100.) / 100., "nominal not E-prequantized");
        check(result.groups[i].distance == distance[i], "literal distance");
        check(result.groups[i].duration == duration[i], "literal duration");
        check(result.groups[i].kind == kinds[i], "original single-phase kind");
    }
    extrusion(result, 0., 1., {200, 201, 1}, {200, 1, -200});
}

void test_merging()
{
    const SourceGeometry source = {{0., 0., 0.}, {1., 0., 0.}, 0., .012};
    const SourceGeometry saved = source;
    const PlannedMove move = make_move(source, 1., 100.);
    for (int digits : {1, 3}) {
        ++cases;
        const GeometryResult result = project_geometry(move, source, digits);
        verify(move, source, digits, result);
        check(result.count == (digits == 1 ? 1 : 3), "precision changes grouping only");
        check(near(move.profile.phases[0].distance, .005) &&
              near(move.profile.phases[1].distance, .99) &&
              near(move.profile.phases[2].distance, .005), "literal short edge phase distances");
        check(same_source(source, saved), "precision projection leaves source unchanged");
        if (digits == 3) {
            check(result.groups[0].end[0] == .005 && result.groups[1].end[0] == .995 &&
                  result.groups[2].end[0] == 1., "fine precision literal cuts");
        }
        if (digits == 1) {
            const GeometryGroup &group = result.groups[0];
            check(group.kind == SegmentKind::Merged && group.phase_count == 3, "three phases in one group");
            check(group.speed_start == 0. && group.speed_end == 0., "merged stop endpoints");
            check(group.distance == 1. && near(group.duration, 1.01), "summed merged duration, not 2d/(v0+v1)");
            extrusion(result, 0., .2, {1, 0, 0}, {1, 0, 0});
            Extrusion charged(2, 0.);
            charged.advance(0., 1., .2);
            const ExtrusionResult closed = charged.advance(group.nominal_end, group.speed_end, .2);
            check(closed.pressure == 0. && closed.pressure_delta == -.2, "merged tail releases preceding charge");
            check(ticks(closed.delta, 100.) == -19, "charge release has no hidden E-only advance");
        }
    }
    ++cases;
    const PlannedMove head = make_move(source, 1., 1., .99, 0.);
    const GeometryResult head_result = project_geometry(head, source, 1);
    verify(head, source, 1, head_result);
    check(near(head.profile.phases[0].distance, .00995) && near(head.profile.phases[1].distance, .49005), "literal head distances");
    check(head_result.count == 2 && head_result.groups[0].phase_count == 2 &&
          head_result.groups[0].kind == SegmentKind::Merged && head_result.groups[1].kind == SegmentKind::Decel, "head joins next phase");
    ++cases;
    const PlannedMove tail = make_move(source, 1., 100., 1., 0.);
    const GeometryResult tail_result = project_geometry(tail, source, 1);
    verify(tail, source, 1, tail_result);
    check(near(tail.profile.phases[0].distance, .995) && near(tail.profile.phases[1].distance, .005), "literal tail distances");
    check(tail_result.count == 1 && tail_result.groups[0].phase_count == 2 &&
          tail_result.groups[0].speed_end == 0., "collapsed tail joins previous unexposed group");
    ++cases;
    const PlannedMove middle = make_move(source, std::sqrt(.98), 1.);
    const GeometryResult middle_result = project_geometry(middle, source, 1);
    verify(middle, source, 1, middle_result);
    check(middle_result.count == 2 && middle_result.groups[0].kind == SegmentKind::Accel &&
          middle_result.groups[1].kind == SegmentKind::Merged && middle_result.groups[1].phase_count == 2, "middle joins following deceleration");
    check(middle_result.groups[0].end[0] == .5 && middle_result.groups[1].start[0] == .5, "literal middle collapsed cut");
}

void test_axes()
{
    const std::array<Point, 5> ends = {{{10., 0., 0.}, {-10., 0., 0.},
                                      {10., -10., 10.}, {-10., 10., -10.}, {0., 0., 10.}}};
    for (const Point &end : ends) {
        for (int digits : {0, 3, 9}) {
            ++cases;
            const SourceGeometry source = {{0., 0., 0.}, end, 0., .012};
            const PlannedMove move = make_move(source, 2., 1.);
            const GeometryResult result = project_geometry(move, source, digits);
            verify(move, source, digits, result);
            check(result.count == 3, "axis direction retains three groups");
            for (std::size_t axis = 0; axis < 3; ++axis)
                check(result.groups[0].end[axis] == end[axis] / 5., "literal signed/diagonal first cut");
        }
    }
    for (double sign : {-1., 1.}) {
        ++cases;
        const SourceGeometry source = {{sign * .125, 0., 0.}, {sign * .375, 0., 0.}, 0., 0.};
        const PlannedMove move = make_move(source, 1., 100., 1., 1.);
        const GeometryResult result = project_geometry(move, source, 2);
        verify(move, source, 2, result);
        check(ticks(result.groups[0].start[0], 100.) == static_cast<std::int64_t>(sign * 13.), "binary .125 tie away from zero");
        check(ticks(result.groups[0].end[0], 100.) == static_cast<std::int64_t>(sign * 38.), "binary .375 tie away from zero");
    }
}

void test_dominant()
{
    ++cases;
    const SourceGeometry source = {{0., 0., 0.}, {1., 0., 0.}, 0., 10.};
    const PlannedMove move = make_move(source, 10., 50.);
    const GeometryResult result = project_geometry(move, source, 2);
    verify(move, source, 2, result);
    check(move.move.distance == 10. && move.move.xyz_distance == 1., "E dominates model distance");
    check(result.count == 3, "dominant distance preserves distinct XYZ cuts");
    const std::array<double, 3> x = {.1, .9, 1.};
    const std::array<double, 3> nominal = {1., 9., 10.};
    const std::array<double, 3> distances = {1., 8., 1.};
    for (std::size_t i = 0; i < 3; ++i) {
        check(result.groups[i].end[0] == x[i], "literal E-dominant XYZ cut");
        check(result.groups[i].nominal_end == nominal[i], "literal E-dominant nominal cut");
        check(result.groups[i].distance == distances[i], "literal E-dominant phase distance");
    }
    extrusion(result, 0., .02, {120, 920, 1000}, {120, 800, 80});
}

void test_planner()
{
    ++cases;
    const SourceGeometry first = {{0., 0., 0.}, {5., 0., 0.}, 0., .1};
    const SourceGeometry second = {{5., 0., 0.}, {10., 0., 0.}, .1, .2};
    const AxisLimits limits = {};
    Planner planner;
    check(planner.push(normalize_move(5., 0., 0., .1, 2., 1., limits, 10)).moves.empty(), "first source retained");
    check(planner.push(normalize_move(5., 0., 0., .1, 2., 1., limits, 11)).moves.empty(), "second source retained");
    const Publication publication = planner.finish();
    check(publication.moves.size() == 2, "real planner returns two sources");
    if (publication.moves.size() != 2) return;
    const GeometryResult a = project_geometry(publication.moves[0], first, 3);
    const GeometryResult b = project_geometry(publication.moves[1], second, 3);
    verify(publication.moves[0], first, 3, a);
    verify(publication.moves[1], second, 3, b);
    check(a.count == 2 && b.count == 2, "two phases per source at shared speed two");
    check(a.groups[0].end[0] == 2. && a.groups[1].end[0] == 5. &&
          b.groups[0].end[0] == 8. && b.groups[1].end[0] == 10., "literal planner bridge cuts");
    check(a.groups[0].speed_start == 0. && a.groups[1].speed_end == 2. &&
          b.groups[0].speed_start == 2. && b.groups[1].speed_end == 0., "literal planner boundary speeds");
    Extrusion state(2, 0.);
    const std::array<GeometryGroup, 4> groups = {a.groups[0], a.groups[1], b.groups[0], b.groups[1]};
    const std::array<std::int64_t, 4> positions = {8, 14, 16, 20};
    const std::array<std::int64_t, 4> deltas = {8, 6, 2, 4};
    const GCodeFormatter formatter(3, 2);
    std::int64_t total = 0;
    for (std::size_t i = 0; i < 4; ++i) {
        const ExtrusionResult output = state.advance(groups[i].nominal_end, groups[i].speed_end, i < 2 ? .02 : 0.);
        check(ticks(output.position, 100.) == positions[i] && ticks(output.delta, 100.) == deltas[i], "literal planner bridge E ticks");
        check(ticks(formatter.quantize_e(output.delta), 100.) == deltas[i], "bridge delta SDK re-encoding");
        total += ticks(output.delta, 100.);
        if (i == 2) check(output.pressure_delta == -.04 && groups[i].speed_start == groups[i].speed_end, "K zero releases charge at constant speed");
        if (i == 3) check(output.pressure == 0., "bridge closes charge");
    }
    check(total == 20, "bridge emits nominal total twenty ticks");
}

void test_rounded_fraction()
{
    for (double sign : {-1., 1.}) {
        ++cases;
        const SourceGeometry source = {{-sign, 0., 0.}, {sign * std::nextafter(.125, 0.), 0., 0.}, 0., 2.};
        const SourceGeometry saved = source;
        const PlannedMove move = make_move(source, 1., 1., 0., std::nextafter(1., 0.));
        check(move.profile.count == 3 && move.profile.phases[2].distance > 0. &&
              move.profile.phases[2].distance < 2e-16, "real solver retains micro deceleration");
        const double prefix = move.profile.phases[0].distance + move.profile.phases[1].distance;
        const double raw = source.start[0] + (prefix / move.move.distance) * (source.end[0] - source.start[0]);
        check(prefix / move.move.distance == 1. && raw == sign * .125, "real prefix rounds fraction to one and overshoots source");
        const GCodeFormatter formatter(2, 2);
        check(ticks(formatter.quantize_xyzf(raw), 100.) == static_cast<std::int64_t>(sign * 13.), "unbounded interpolation would round to thirteen ticks");
        const GeometryResult result = project_geometry(move, source, 2);
        verify(move, source, 2, result);
        check(result.count == 2 && result.groups[1].kind == SegmentKind::Merged &&
              result.groups[1].first_phase == 1 && result.groups[1].phase_count == 2, "micro tail belongs to nonempty steady/decel group");
        check(ticks(result.groups[result.count - 1].end[0], 100.) == static_cast<std::int64_t>(sign * 12.), "bounded endpoint stays at twelve ticks");
        check(same_source(source, saved), "rounding regression source unchanged");
    }
}

void test_nominal()
{
    for (double origin : {0., -1.253, 5.127}) {
        for (double increment : {0., .012}) {
            ++cases;
            const SourceGeometry source = {{0., 0., 0.}, {10., 0., 0.}, origin, origin + increment};
            const PlannedMove move = make_move(source, 2., 1.);
            const GeometryResult result = project_geometry(move, source, 3);
            verify(move, source, 3, result);
            check(result.groups[result.count - 1].nominal_end == source.nominal_end, "final signed nominal has no drift");
            for (double k : {0., .02}) {
                Extrusion state(2, origin);
                std::int64_t total = 0;
                double last_delta = 0.;
                for (std::size_t i = 0; i < result.count; ++i) {
                    const GeometryGroup &group = result.groups[i];
                    const ExtrusionResult output = state.advance(group.nominal_end, group.speed_end, k);
                    total += ticks(output.delta, 100.);
                    last_delta = output.delta;
                    if (increment == 0. && k == 0.) check(output.delta == 0., "zero nominal and K preserve XYZ with zero E");
                    if (i + 1 == result.count) check(output.pressure == 0., "signed-origin cycle closes charge");
                }
                check(total == ticks(source.nominal_end, 100.) - ticks(origin, 100.), "signed-origin nominal tick balance");
                if (increment == 0. && k > 0.) check(last_delta < 0., "constant nominal releases charge through negative spatial E");
            }
        }
    }
}

void test_collapsed()
{
    for (bool steady : {false, true}) {
        ++cases;
        const SourceGeometry source = {{0., 0., 0.}, {.0005, 0., 0.}, -.123, -.122};
        const PlannedMove move = make_move(source, 1., 100., steady ? 1. : 0., steady ? 1. : 0.);
        const GeometryResult result = project_geometry(move, source, 2);
        verify(move, source, 2, result);
        check(result.status == GeometryStatus::EntireSourceCollapsed && result.count == 1, "collapsed source has explicit one-group aggregate");
        check(result.groups[0].kind == (steady ? SegmentKind::Steady : SegmentKind::Merged), "single versus multiple aggregate kind");
        check(result.groups[0].nominal_start == source.nominal_start &&
              result.groups[0].nominal_end == source.nominal_end, "collapsed nominal retained exactly");
        Extrusion state(2, source.nominal_start);
        Extrusion control(2, source.nominal_start);
        state.advance(source.nominal_start, 1., .02);
        control.advance(source.nominal_start, 1., .02);
        if (result.status == GeometryStatus::Representable) {
            for (std::size_t i = 0; i < result.count; ++i)
                state.advance(result.groups[i].nominal_end, result.groups[i].speed_end, .02);
        }
        const ExtrusionResult after = state.advance(0., 0., .02);
        const ExtrusionResult reference = control.advance(0., 0., .02);
        check(after.position == reference.position && after.delta == reference.delta &&
              after.nominal_delta == reference.nominal_delta && after.pressure == reference.pressure &&
              after.pressure_delta == reference.pressure_delta, "caller example status gate leaves charge and nominal state unchanged");
    }
}

void test_rejection()
{
    const SourceGeometry source = {{0., 0., 0.}, {10., 0., 0.}, 0., .012};
    const PlannedMove move = make_move(source, 2., 1.);
    for (int precision : {-1, 10}) {
        ++cases;
        bool rejected = false;
        try { project_geometry(move, source, precision); }
        catch (const std::invalid_argument &) { rejected = true; }
        check(rejected, "invalid precision rejected");
    }
    const std::array<double, 3> invalid = {std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};
    for (double value : invalid) {
        for (std::size_t field = 0; field < 8; ++field) {
            ++cases;
            SourceGeometry bad = source;
            if (field < 3) bad.start[field] = value;
            else if (field < 6) bad.end[field - 3] = value;
            else if (field == 6) bad.nominal_start = value;
            else bad.nominal_end = value;
            const SourceGeometry saved = bad;
            bool rejected = false;
            try { project_geometry(move, bad, 3); }
            catch (const std::invalid_argument &) { rejected = true; }
            check(rejected, "nonfinite source field rejected");
            check(same_source(bad, saved), "rejected borrowed source unchanged");
            const GeometryResult good = project_geometry(move, source, 3);
            check(good.count == 3 && good.groups[2].nominal_end == .012, "independent valid source unaffected");
        }
    }
}

PlannedMove make_move(const SourceGeometry &source, double speed, double acceleration,
                      double start_speed, double end_speed)
{
    const AxisLimits limits = {};
    const Move move = normalize_move(source.end[0] - source.start[0], source.end[1] - source.start[1],
                                    source.end[2] - source.start[2], source.nominal_end - source.nominal_start,
                                    speed, acceleration, limits, 0);
    return {move, solve_phases({move.distance, move.speed_limit, start_speed, end_speed,
                               move.acceleration, move.acceleration})};
}

void verify(const PlannedMove &move, const SourceGeometry &source,
            int digits, const GeometryResult &result)
{
    check(result.count >= 1 && result.count <= 3, "owned bounded group count");
    if (result.count < 1 || result.count > 3)
        throw std::runtime_error("Geometry group count prevents safe result reads");
    const GCodeFormatter formatter(static_cast<std::uint16_t>(digits), 2);
    const double scale = GCodeFormatter::pow_10[digits];
    std::size_t phase_index = 0;
    double distance = 0.;
    double duration = 0.;
    double original_duration = 0.;
    for (std::size_t i = 0; i < move.profile.count; ++i) original_duration += move.profile.phases[i].duration;
    for (std::size_t i = 0; i < result.count; ++i) {
        const GeometryGroup &group = result.groups[i];
        check(group.first_phase == phase_index && group.phase_count > 0, "ordered exact phase partition");
        check(group.first_phase <= move.profile.count &&
              group.phase_count <= move.profile.count - group.first_phase, "group phase range valid");
        if (group.phase_count == 0 || phase_index >= move.profile.count ||
            group.phase_count > move.profile.count - phase_index)
            throw std::runtime_error("Geometry phase partition prevents safe profile reads");
        check(group.kind == SegmentKind::Merged || group.phase_count == 1, "single kind has one phase");
        check((group.kind == SegmentKind::Merged) == (group.phase_count >= 2), "merged iff multiple original phases");
        check(group.distance > 0. && group.duration > 0., "positive group sums");
        check(group.speed_start == move.profile.phases[phase_index].speed_start &&
              group.speed_end == move.profile.phases[phase_index + group.phase_count - 1].speed_end, "original boundary speeds exact");
        double group_distance = 0.;
        double group_duration = 0.;
        for (std::size_t j = 0; j < group.phase_count; ++j) {
            group_distance += move.profile.phases[phase_index + j].distance;
            group_duration += move.profile.phases[phase_index + j].duration;
        }
        check(near(group.distance, group_distance) && near(group.duration, group_duration), "each group retains its original phase sums");
        check(group.nominal_start <= group.nominal_end && group.nominal_start >= source.nominal_start &&
              group.nominal_end <= source.nominal_end, "nominal monotone and bounded");
        if (result.status == GeometryStatus::Representable) check(group.start != group.end, "every emitted group has nonempty XYZ");
        else check(result.count == 1 && group.start == group.end, "collapsed aggregate is non-emittable");
        for (std::size_t axis = 0; axis < 3; ++axis) {
            check(ticks(formatter.quantize_xyzf(group.start[axis]), scale) == ticks(group.start[axis], scale) &&
                  ticks(formatter.quantize_xyzf(group.end[axis]), scale) == ticks(group.end[axis], scale), "XYZ group round trips SDK grid");
            check(source.end[axis] >= source.start[axis] ? group.end[axis] >= group.start[axis] :
                                                        group.end[axis] <= group.start[axis], "XYZ monotone in source direction");
            if (i == 0) check(group.start[axis] == formatter.quantize_xyzf(source.start[axis]), "origin is SDK Q(source start)");
            if (i + 1 == result.count) check(group.end[axis] == formatter.quantize_xyzf(source.end[axis]), "final is exact SDK Q(source end)");
        }
        if (i == 0) check(group.nominal_start == source.nominal_start, "exact nominal origin");
        if (i > 0) check(group.start == result.groups[i - 1].end &&
                         group.nominal_start == result.groups[i - 1].nominal_end, "XYZ and nominal continuity");
        phase_index += group.phase_count;
        distance += group.distance;
        duration += group.duration;
    }
    check(phase_index == move.profile.count, "all original phases assigned exactly once");
    check(near(distance, move.move.distance) && near(duration, original_duration), "total distance and duration conserved");
    check(result.groups[result.count - 1].nominal_end == source.nominal_end, "final nominal exact, not cumulative approximation");
}

void extrusion(const GeometryResult &result, double origin, double k,
               const std::array<std::int64_t, 3> &positions,
               const std::array<std::int64_t, 3> &deltas)
{
    check(result.status == GeometryStatus::Representable, "extrusion consumes only representable groups");
    if (result.status != GeometryStatus::Representable) return;
    Extrusion state(2, origin);
    const GCodeFormatter formatter(3, 2);
    std::int64_t sum = ticks(origin, 100.);
    for (std::size_t i = 0; i < result.count; ++i) {
        const GeometryGroup &group = result.groups[i];
        const ExtrusionResult output = state.advance(group.nominal_end, group.speed_end, k);
        check(ticks(output.position, 100.) == positions[i], "literal absolute E ticks");
        check(ticks(output.delta, 100.) == deltas[i], "literal relative E ticks");
        check(ticks(formatter.quantize_e(output.delta), 100.) == deltas[i], "literal delta re-encodes with SDK");
        sum += ticks(output.delta, 100.);
        check(sum == positions[i], "relative ticks equal cumulative absolute ticks");
        if (i + 1 == result.count) check(output.pressure == 0., "last final group closes charge");
    }
}

bool same_source(const SourceGeometry &left, const SourceGeometry &right)
{
    const std::array<double, 8> a = {left.start[0], left.start[1], left.start[2], left.end[0], left.end[1], left.end[2], left.nominal_start, left.nominal_end};
    const std::array<double, 8> b = {right.start[0], right.start[1], right.start[2], right.end[0], right.end[1], right.end[2], right.nominal_start, right.nominal_end};
    for (std::size_t i = 0; i < a.size(); ++i)
        if (a[i] != b[i] && !(std::isnan(a[i]) && std::isnan(b[i]))) return false;
    return true;
}

std::int64_t ticks(double value, double scale)
{
    return static_cast<std::int64_t>(std::round(value * scale));
}

bool near(double actual, double expected)
{
    return std::isfinite(actual) && std::isfinite(expected) &&
           std::abs(actual - expected) <= 16. * std::numeric_limits<double>::epsilon() *
           std::max(std::abs(actual), std::abs(expected));
}

void check(bool condition, const char *message)
{
    ++checks;
    if (!condition) {
        ++failures;
        std::printf("FAILED: case %d, check %d: %s\n", cases, checks, message);
    }
}
} // namespace
