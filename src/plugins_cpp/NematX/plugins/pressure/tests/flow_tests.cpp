///|/ Released under the GNU Affero General Public License, version 3 or later.

#include "PressureFlow.hpp"
#include "PressureRewrite.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <utility>

#include "../../nematx_firmware/src/NematXSyntax.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeLineParser.hpp"

// Exercises the ordered assembly with local, explicit SDK-validated Reader
// deltas. Source/Nominal/Syntax and all mathematical/text components are real;
// no substitute Reader parses commands. Literal byte, phase and material oracles
// test the assembly, not the real provider or firmware/export integration.
// Throwing assertions remain active in Release and guard dependent accesses.
//
// Call tree:
//   main: run analytical, controls, tool/frame, transfers, retention and errors.
//     Harness::send: index explicit validated deltas and consume owned fragments.
//       source / event: reuse the local Source/Nominal test construction pattern.
//     numbers / fields: inspect actual output with SDK tokens and annotation keys.
//     check / near: count executed assertions and throw on any failure.

// Runs all cases, returning nonzero on the first failure in every configuration.
int main();

namespace {
using namespace nematx_pressure;
using Tracker = slic3r_api::GCodeStateTracker;
using Parser = slic3r_api::GCodeGeneration::GCodeLineParser;
using Item = raw_gcode_reader_event_item;
constexpr std::uint32_t X = RAW_GCODE_READER_ITEM_X, Y = RAW_GCODE_READER_ITEM_Y,
    Z = RAW_GCODE_READER_ITEM_Z, E = RAW_GCODE_READER_ITEM_E,
    F = RAW_GCODE_READER_ITEM_FEEDRATE, XYZ = RAW_GCODE_READER_ITEM_XYZ_ABSOLUTE,
    EA = RAW_GCODE_READER_ITEM_E_ABSOLUTE, MM = RAW_GCODE_READER_ITEM_UNITS_MM,
    TOOL = RAW_GCODE_READER_ITEM_ACTIVE_TOOL;
constexpr std::uint32_t MOVE = RAW_GCODE_READER_EVENT_LINEAR_MOVE,
    UPDATE = RAW_GCODE_READER_EVENT_STATE_UPDATE, RESET = RAW_GCODE_READER_EVENT_SET_POSITION;
int cases = 0, checks = 0;

// Creates fully resolved local settings with two tools and no runtime lookup.
FlowConfig config(double k = .1, bool annotate = true, int xyz_digits = 3, int e_digits = 3);
// Attaches the explicit local row to a manual event for public SDK validation.
ReaderEvent event(std::uint32_t kind, std::size_t row, std::vector<Item> items);
// Validates events, owns their original text, and carries the final Reader state.
SourceFragment source(std::string text, Tracker &prior, std::vector<ReaderEvent> events);
struct Harness
{
    Flow flow;
    Tracker prior;
    // Starts the real Flow from empty state, following a manual outside-layer preamble.
    explicit Harness(FlowConfig settings = config(), bool absolute = false);
    // Sends one complete original fragment; no command is divided between calls.
    FlowOutput send(std::string text, std::vector<ReaderEvent> events = {},
                    FlowBoundary boundary = {}, std::vector<MoveHint> hints = {});
};
// Checks three analytical phases, four settings combinations and both E modes.
void test_analytical();
// Checks persistent controls, frozen K, neutral tools and owned diagnostics.
void test_controls();
// Checks tool-local rounding, absolute discontinuities and relative external resets.
void test_frames();
// Checks forward/backward transfers, endpoint speeds, annotations and event barriers.
void test_transfers();
// Checks complete-publication scanning and retention of the last spatial source.
void test_retention();
// Checks Begin+End, pre-script closure, LF continuity and per-call failure isolation.
void test_boundaries_errors();
// Extracts real SDK numeric words from output; assertions use literal expectations.
std::vector<double> numbers(const std::string &text, char letter);
// Reads round-trip annotation values, requiring the requested field on each row.
std::vector<double> fields(const std::string &text, const std::string &key);
// Counts exact substrings, including multiple annotations in the output.
std::size_t occurrences(const std::string &text, const std::string &needle);
// Counts a real assertion and throws before dependent accesses on failure.
void check(bool condition, const char *message);
// Compares analytical doubles at a declared absolute tolerance.
void near(double actual, double expected, double tolerance, const char *message);
} // namespace

int main()
{
    try {
        test_analytical();
        test_controls();
        test_frames();
        test_transfers();
        test_retention();
        test_boundaries_errors();
        std::printf("pressure flow: %d cases, %d checks, 0 failures\n", cases, checks);
        return 0;
    } catch (const std::exception &error) {
        std::fprintf(stderr, "FAILED: %s (%d cases, %d checks)\n", error.what(), cases, checks);
        return 1;
    }
}

namespace {
void test_analytical()
{
    for (bool absolute : {false, true}) {
        for (double k : {0., .1}) {
            for (bool annotate : {false, true}) {
                ++cases;
                Harness h(config(k, annotate), absolute);
                const FlowOutput result = h.send("G1 X10 E1 F120 ;one\n", {
                    event(MOVE, 0, {{X, 0, 10}, {E, 0, 1}, {F, 0, 120}})}, {true, true, false});
                if (k == 0 && !annotate) {
                    check(result.bytes == "G1 X10 E1 F120 ;one\n", "neutral tool remains byte-identical");
                    continue;
                }
                const std::vector<double> xs = numbers(result.bytes, 'X'), es = numbers(result.bytes, 'E');
                check(xs == std::vector<double>({2, 8, 10}) && es.size() == 3, "analytical three-phase geometry");
                near(es[0], k > 0 ? .4 : .2, 1e-7, "first phase charge");
                near(es[1], absolute ? (k > 0 ? 1 : .8) : .6, 1e-7, "steady phase E");
                near(es[2], absolute ? 1 : (k > 0 ? 0 : .2), 1e-7, "closed final E");
                check(occurrences(result.bytes, "F120") == 1 && occurrences(result.bytes, ";one") == 1,
                    "no phase feed or duplicate source comment");
                check(occurrences(result.bytes, ";NX_PRESSURE") == (annotate ? 3 : 0), "F09 selection");
                if (annotate) {
                    check(fields(result.bytes, "v0_mm_s") == std::vector<double>({0, 2, 2}), "analytical initial speeds");
                    check(fields(result.bytes, "v1_mm_s") == std::vector<double>({2, 2, 0}), "analytical final speeds");
                    check(fields(result.bytes, "dt_s") == std::vector<double>({2, 3, 2}), "feed is converted from mm/min");
                }
            }
        }
    }
}

void test_controls()
{
    ++cases;
    Harness h;
    check(h.send("; PRESSURE_FACTOR=.2\n").bytes == "; PRESSURE_FACTOR=.2\n", "outside-layer control passes unchanged");
    const FlowOutput first = h.send("G1 X10 E1\n; PRESSURE_FACTOR=0\nG1 X20 E1\n", {
        event(MOVE, 0, {{X, 0, 10}, {E, 0, 1}}), event(MOVE, 2, {{X, 0, 20}, {E, 0, 2}})}, {true, true, false});
    const std::vector<double> ks = fields(first.bytes, "k_s");
    check(ks.size() >= 4 && ks.front() == .2 && ks.back() == 0, "captured K is not retroactive");
    const std::vector<double> charges = fields(first.bytes, "de_pressure_mm");
    double charge = 0.;
    for (double delta : charges) charge += delta;
    near(charge, 0, 1e-14, "override zero releases actual prior charge");
    const FlowOutput reset = h.send("; PRESSURE_FACTOR=\nG1 X30 E1\n", {event(MOVE, 1, {{X, 0, 30}, {E, 0, 3}})}, {true, true, false});
    check(fields(reset.bytes, "k_s").front() == .1, "empty override returns configuration");

    ++cases;
    FlowConfig settings = config(0, true);
    settings.tools[1] = {0, false};
    Harness f09(settings);
    f09.send("; PRESSURE_FACTOR=.2\n");
    const FlowOutput active = f09.send("G1 X10 E1\n", {event(MOVE, 0, {{X, 0, 10}, {E, 0, 1}})}, {true, true, false});
    check(fields(active.bytes, "k_s").front() == .2, "positive override corrects F09-only tool");
    const FlowOutput neutral = f09.send("T1\nG1 X20 E.5\n", {
        event(UPDATE, 0, {{TOOL, 0, 1}}), event(MOVE, 1, {{X, 0, 20}, {E, 1, .5}})}, {true, true, false});
    check(neutral.bytes == "T1\nG1 X20 E.5\n", "override does not awaken neutral tool");

    ++cases;
    Harness diagnostic;
    const FlowOutput issue = diagnostic.send(";M204 Snan\n");
    check(issue.issues.size() == 1 && issue.issues[0].kind == FlowIssueKind::InvalidAcceleration &&
        issue.issues[0].line_index == 0, "invalid acceleration is an owned issue");
    const FlowOutput valid = diagnostic.send("G1 X10 E1\n", {event(MOVE, 0, {{X, 0, 10}, {E, 0, 1}})}, {true, true, false});
    check(numbers(valid.bytes, 'X') == std::vector<double>({2, 8, 10}), "invalid acceleration leaves model unchanged");
    const FlowOutput reader = diagnostic.send("opaque\n", {event(RAW_GCODE_READER_EVENT_INVALIDATE, 0, {{F, 0, 0}}),
        event(RAW_GCODE_READER_EVENT_DIAGNOSTIC, 0, {{RAW_GCODE_READER_ITEM_DIAGNOSTIC_TYPE, 0,
            RAW_GCODE_READER_DIAGNOSTIC_UNKNOWN_COMMAND}})});
    check(reader.issues.size() == 2 && !reader.issues[0].reader_items.empty(), "Reader issues own copied event items");

    ++cases;
    Harness acceleration;
    acceleration.send(";M204 S2\n");
    const FlowOutput accelerated = acceleration.send("G1 X10 E1\n", {event(MOVE, 0, {{X, 0, 10}, {E, 0, 1}})}, {true, true, false});
    check(numbers(accelerated.bytes, 'X') == std::vector<double>({1, 9, 10}), "outside-layer acceleration persists into layer");
    const FlowOutput next_layer = acceleration.send("G1 X20 E1\n", {event(MOVE, 0, {{X, 0, 20}, {E, 0, 2}})}, {true, true, false});
    check(numbers(next_layer.bytes, 'X') == std::vector<double>({11, 19, 20}), "layer closure does not reset acceleration");
}

void test_frames()
{
    ++cases;
    Harness h(config(0, true));
    std::vector<double> totals;
    for (int step = 0; step < 3; ++step) {
        const std::uint32_t tool = step == 1 ? 1 : 0;
        const std::string text = std::string("T") + std::to_string(tool) + "\nG1 X" + std::to_string(step + 1) + " E.0006\n";
        const FlowOutput out = h.send(text, {event(UPDATE, 0, {{TOOL, 0, double(tool)}}),
            event(MOVE, 1, {{X, 0, double(step + 1)}, {E, tool, step == 2 ? .0012 : .0006}})}, {true, true, false});
        double sum = 0.;
        for (double e : numbers(out.bytes, 'E')) sum += e;
        totals.push_back(sum);
    }
    near(totals[0], .001, 1e-10, "T0 rounds first .0006");
    near(totals[1], .001, 1e-10, "T1 has independent rounding");
    near(totals[2], 0, 1e-10, "return T0 preserves residual rounding across layers");

    ++cases;
    Harness relative(config(0, true));
    relative.send("G1 X1 E.0006\n", {event(MOVE, 0, {{X, 0, 1}, {E, 0, .0006}})}, {true, true, false});
    const FlowOutput after_reset = relative.send("G92 E10\nG1 X2 E.0006\n", {
        event(RESET, 0, {{E, 0, 10}}), event(MOVE, 1, {{X, 0, 2}, {E, 0, 10.0006}})}, {true, true, false});
    const std::vector<double> reset_es = numbers(after_reset.bytes, 'E');
    check(reset_es.size() == 3 && reset_es[0] == 10, "external relative reset is preserved");
    near(reset_es[1] + reset_es[2], 0, 1e-10, "relative reset does not reset working sum");

    for (const std::string &barrier : {std::string("G1 E.25\n"), std::string("G1 E-.25\n"), std::string("G92 E-1.25\n")}) {
        ++cases;
        Harness absolute(config(.1, true), true);
        absolute.send("G1 X10 E1\n", {event(MOVE, 0, {{X, 0, 10}, {E, 0, 1}})}, {true, true, false});
        const double origin = barrier.find("G92") == 0 ? -1.25 : (barrier.find("-") != std::string::npos ? -.25 : .25);
        const FlowOutput out = absolute.send(barrier + "G1 X20 E2\n", {
            event(barrier.find("G92") == 0 ? RESET : MOVE, 0, {{E, 0, origin}}),
            event(MOVE, 1, {{X, 0, 20}, {E, 0, 2}})}, {true, true, false});
        const std::vector<double> es = numbers(out.bytes, 'E');
        check(es.size() == 4, "absolute barrier plus three phases");
        near(es.back(), 2, 1e-10, "absolute discontinuity reanchors and closes to exact nominal");
        const std::vector<double> nominals = fields(out.bytes, "de_nom_mm");
        double sum = 0.;
        for (double value : nominals) sum += value;
        near(sum, 2 - origin, 1e-14, "absolute reanchor preserves analytical nominal deltas");
    }

    ++cases;
    Harness mode(config(0, true));
    mode.send("G1 X1 E.0006\n", {event(MOVE, 0, {{X, 0, 1}, {E, 0, .0006}})}, {true, true, false});
    const FlowOutput switched = mode.send("M82\nG92 E0\nM83\nG1 X2 E.0006\n", {
        event(UPDATE, 0, {{EA, 0, 1}}), event(RESET, 1, {{E, 0, 0}}),
        event(UPDATE, 2, {{EA, 0, 0}}), event(MOVE, 3, {{X, 0, 2}, {E, 0, .0006}})}, {true, true, false});
    const std::vector<double> mode_es = numbers(switched.bytes, 'E');
    check(mode_es.size() == 3, "mode-change source shape");
    near(mode_es[1] + mode_es[2], .001, 1e-10, "real mode changes rebase relative frame");

    ++cases;
    Harness inline_modes(config(0, true));
    inline_modes.send("G1 G90 X1 G91 A.0006\n;plain\n", {
        event(MOVE, 0, {{X, 0, 1}, {E, 0, .0006}}), event(UPDATE, 0, {{XYZ, 0, 0}})}, {true, true, false});
    const FlowOutput reaffirmed = inline_modes.send("G1 G90 X2 G91 A.0006\n", {
        event(UPDATE, 0, {{XYZ, 0, 1}}), event(MOVE, 0, {{X, 0, 2}, {E, 0, .0012}}),
        event(UPDATE, 0, {{XYZ, 0, 0}})}, {true, true, false});
    const std::vector<double> reaffirmed_e = numbers(reaffirmed.bytes, 'A');
    check(reaffirmed_e.size() == 2, "inline reaffirmation retains two spatial groups");
    check(reaffirmed_e == std::vector<double>({0, 0}),
        "inline mode reaffirmation and intervening comment preserve exact relative residual");

    ++cases;
    Harness tool_row;
    const FlowOutput old_tool = tool_row.send("G1 X10 E1\n", {event(MOVE, 0, {{X, 0, 10}, {E, 0, 1}})}, {true, false, false});
    check(old_tool.bytes.empty(), "first tool waits for its terminal boundary");
    const FlowOutput new_tool = tool_row.send("G1 G90 X20 G91 B1\n", {
        event(UPDATE, 0, {{TOOL, 0, 1}}), event(MOVE, 0, {{X, 0, 20}, {E, 1, 1}}),
        event(UPDATE, 0, {{XYZ, 0, 0}})}, {false, true, false});
    const std::vector<double> tool_end = fields(new_tool.bytes, "v1_mm_s");
    check(tool_end.size() == 6 && tool_end[2] == 0 && tool_end.back() == 0,
        "tool-change row closes old cycle then corrects its own new-tool motion");
    check(occurrences(new_tool.bytes, "tool=0") == 3 && occurrences(new_tool.bytes, "tool=1") == 3,
        "tool-local annotations stay in source order");
}

void test_transfers()
{
    ++cases;
    Harness forward(config(.1, true, 3, 6));
    const FlowOutput f = forward.send("G1 X.0001 E.00001 ;tiny\n;plain\nG1 X.0021 E.0002\n", {
        event(MOVE, 0, {{X, 0, .0001}, {E, 0, .00001}}),
        event(MOVE, 2, {{X, 0, .0021}, {E, 0, .00021}})}, {true, true, false});
    check(f.bytes.find(";tiny\n;plain\n") == 0 && occurrences(f.bytes, "phase=merged") == 1,
        "forward transfer retains source residue and effect-free comment order");
    const std::vector<double> fv0 = fields(f.bytes, "v0_mm_s"), fv1 = fields(f.bytes, "v1_mm_s");
    check(fv0.size() == 2 && fv1.size() == 2, "forward receiver retains two emission groups");
    near(fv0[0], 0, 1e-14, "forward merge starts at first collapsed entry");
    near(fv1[0], std::sqrt(.0021), 1e-12, "forward merge ends at receiver peak, not collapsed exit");
    const std::vector<double> fn = fields(f.bytes, "de_nom_mm");
    near(fn[0] + fn[1], .00021, 1e-15, "forward nominal sum matches all original material");
    const std::vector<double> fe = numbers(f.bytes, 'E');
    near(fe[0] + fe[1], .00021, 1e-8, "forward emitted material closes without E-only compensator");

    ++cases;
    Harness backward(config(.1, true, 3, 6));
    const FlowOutput b = backward.send("G1 X.002 E.0002\nG1 X.0021 E.00001 ;tiny\n", {
        event(MOVE, 0, {{X, 0, .002}, {E, 0, .0002}}),
        event(MOVE, 1, {{X, 0, .0021}, {E, 0, .00021}})}, {true, true, false});
    const std::vector<double> bv0 = fields(b.bytes, "v0_mm_s"), bv1 = fields(b.bytes, "v1_mm_s");
    check(bv0.size() == 2 && bv1.size() == 2 && b.bytes.find(";tiny\n") != std::string::npos,
        "backward anchor survives until collapsed finish suffix is inspected");
    near(bv0[1], std::sqrt(.0021), 1e-12, "backward merge starts at anchor peak");
    near(bv1[1], 0, 1e-14, "backward merged tail closes at zero");
    const std::vector<double> bn = fields(b.bytes, "de_nom_mm");
    near(bn[0] + bn[1], .00021, 1e-15, "backward nominal sum");

    for (const std::string &effect : {std::string("M106 S1\n"), std::string(";TYPE:Perimeter\n"),
             std::string(";M204 S1\n"), std::string("; PRESSURE_FACTOR=.1\n")}) {
        ++cases;
        Harness blocked(config(.1, true, 3, 6));
        bool refused = false;
        try {
            blocked.send("G1 X.0001 E.00001\n" + effect + "G1 X.0021 E.0002\n", {
                event(MOVE, 0, {{X, 0, .0001}, {E, 0, .00001}}),
                event(MOVE, 2, {{X, 0, .0021}, {E, 0, .00021}})}, {true, true, false});
        } catch (const std::invalid_argument &) { refused = true; }
        check(refused, "material cannot cross a fan, TYPE or same-value model control");

        ++cases;
        Harness continuous;
        const FlowOutput c = continuous.send("G1 X10 E1\n" + effect + "G1 X20 E1\n", {
            event(MOVE, 0, {{X, 0, 10}, {E, 0, 1}}), event(MOVE, 2, {{X, 0, 20}, {E, 0, 2}})}, {true, true, false});
        const std::vector<double> v1 = fields(c.bytes, "v1_mm_s");
        check(v1.size() >= 4 && v1[1] > 0, "transfer barrier does not stop pressure planning");
        check(occurrences(c.bytes, effect) == 1, "side-effect row remains once and in order");
    }

    ++cases;
    Harness none;
    bool refused = false;
    try { none.send("G1 X.0001 E.00001\n", {event(MOVE, 0, {{X, 0, .0001}, {E, 0, .00001}})}, {true, true, false}); }
    catch (const std::invalid_argument &) { refused = true; }
    check(refused, "entirely collapsed cycle fails explicitly without a neighbour");

    ++cases;
    Harness lost(config(.1, true, 3, 6));
    bool lost_mode = false;
    try {
        lost.send("G1 G90 X.0001 G91 B.00001\nG1 G90 X.0021 G91 B.0002\n", {
            event(UPDATE, 0, {{TOOL, 0, 1}}), event(MOVE, 0, {{X, 0, .0001}, {E, 1, .00001}}),
            event(UPDATE, 0, {{XYZ, 0, 0}}), event(UPDATE, 1, {{XYZ, 0, 1}}),
            event(MOVE, 1, {{X, 0, .0021}, {E, 1, .00021}}), event(UPDATE, 1, {{XYZ, 0, 0}})}, {true, true, false});
    } catch (const RewriteError &error) { lost_mode = error.cause == RewriteCause::LostModalEffect; }
    check(lost_mode, "residue loss of axis-implied tool is propagated unchanged");

    ++cases;
    Harness fallback(config(.1, true, 3, 6));
    const FlowOutput fallback_out = fallback.send("G1 X.002 E.0002 ;anchor\nG1 X.0021 E.00001 ;tiny\nM106 S1\nG1 X.0041 E.0002 ;receiver\n", {
        event(MOVE, 0, {{X, 0, .002}, {E, 0, .0002}}), event(MOVE, 1, {{X, 0, .0021}, {E, 0, .00021}}),
        event(MOVE, 3, {{X, 0, .0041}, {E, 0, .00041}})}, {true, true, false});
    check(fallback_out.bytes.find("phase=merged") < fallback_out.bytes.find("M106 S1") &&
        fallback_out.bytes.find(";tiny\nM106 S1\n") != std::string::npos,
        "effect forces backward fallback without moving the effect or residue");
    const std::vector<double> fallback_v1 = fields(fallback_out.bytes.substr(0, fallback_out.bytes.find("M106 S1")), "v1_mm_s");
    check(fallback_v1.size() == 1, "backward fallback retains the original one-group anchor");
    near(fallback_v1[0], std::sqrt(.004), 1e-12, "backward fallback keeps the live pressure junction before the fan");

    ++cases;
    Harness rebased(config(.1, true, 3, 6), true);
    rebased.send("G1 X10 E1\n", {event(MOVE, 0, {{X, 0, 10}, {E, 0, 1}})}, {true, true, false});
    const FlowOutput rebased_out = rebased.send("G92 E5\nG1 X10.0001 E5.00001 ;tiny\nG1 X10.0021 E5.00021 ;receiver\n", {
        event(RESET, 0, {{E, 0, 5}}), event(MOVE, 1, {{X, 0, 10.0001}, {E, 0, 5.00001}}),
        event(MOVE, 2, {{X, 0, 10.0021}, {E, 0, 5.00021}})}, {true, true, false});
    check(rebased_out.bytes.find("G92 E5\n;tiny\n") == 0 && occurrences(rebased_out.bytes, "phase=merged") == 1,
        "absolute collapsed source forwards material and rebase after unchanged G92");
    const std::vector<double> rebased_e = numbers(rebased_out.bytes, 'E');
    check(rebased_e.size() == 3, "reset plus two absolute receiver groups");
    near(rebased_e[1], 5.004688, 5e-7, "forward rebase applies charge from the new baseline");
    near(rebased_e[2], 5.00021, 5e-7, "forward absolute transfer closes at new nominal end");
    const std::vector<double> rebased_nominal = fields(rebased_out.bytes, "de_nom_mm");
    check(rebased_nominal.size() == 2, "absolute receiver has two nominal annotations");
    near(rebased_nominal[0], .000105, 2e-15, "first merged group includes collapsed material at the new baseline");
    near(rebased_nominal[0] + rebased_nominal[1], .00021, 2e-15, "new baseline does not become added nominal material");

    ++cases;
    Harness seam;
    const FlowOutput seam_out = seam.send("G1 X10 E1\n;SEAM\nG1 X20 E1\n", {
        event(MOVE, 0, {{X, 0, 10}, {E, 0, 1}}),
        event(RAW_GCODE_READER_EVENT_MARKER, 1, {{RAW_GCODE_READER_ITEM_MARKER, 0, RAW_GCODE_READER_MARKER_SEAM}}),
        event(MOVE, 2, {{X, 0, 20}, {E, 0, 2}})}, {true, true, false});
    check(occurrences(seam_out.bytes, ";SEAM\n") == 1, "neutral Reader marker text is emitted once");
    const std::size_t seam_position = seam_out.bytes.find(";SEAM\n");
    const std::vector<double> seam_before = fields(seam_out.bytes.substr(0, seam_position), "v1_mm_s");
    const std::vector<double> seam_after = fields(seam_out.bytes.substr(seam_position), "v0_mm_s");
    check(!seam_before.empty() && !seam_after.empty(), "printing remains on both sides of SEAM");
    near(seam_before.back(), 2, 1e-14, "SEAM does not close previous printing junction");
    near(seam_after.front(), 2, 1e-14, "SEAM does not restart next printing movement");

    ++cases;
    Harness seam_blocked(config(.1, true, 3, 6));
    bool seam_refused = false;
    try {
        seam_blocked.send("G1 X.0001 E.00001\n;SEAM\nG1 X.0021 E.0002\n", {
            event(MOVE, 0, {{X, 0, .0001}, {E, 0, .00001}}),
            event(RAW_GCODE_READER_EVENT_MARKER, 1, {{RAW_GCODE_READER_ITEM_MARKER, 0, RAW_GCODE_READER_MARKER_SEAM}}),
            event(MOVE, 2, {{X, 0, .0021}, {E, 0, .00021}})}, {true, true, false});
    } catch (const std::invalid_argument &error) {
        seam_refused = std::string(error.what()).find("no compatible un-emitted spatial neighbour") != std::string::npos;
    }
    check(seam_refused, "Reader SEAM event forbids forward material transfer without a backward anchor");
}

void test_retention()
{
    for (bool middle : {false, true}) {
        ++cases;
        Harness h(config(.1, true, 3, 6));
        const std::string text = middle ? "G1 X.002 E.0002 ;R1\nG1 X.0021 E.00001 ;C\nG1 X.0041 E.0002 ;R2\n" :
            "G1 X.002 E.0002 ;R1\nG1 X.004 E.0002 ;R2\nG1 X.0041 E.00001 ;C\n";
        const FlowOutput out = h.send(text, {event(MOVE, 0, {{X, 0, .002}, {E, 0, .0002}}),
            event(MOVE, 1, {{X, 0, middle ? .0021 : .004}, {E, 0, middle ? .00021 : .0004}}),
            event(MOVE, 2, {{X, 0, .0041}, {E, 0, .00041}})}, {true, true, false});
        check(occurrences(out.bytes, ";R1") == 1 && occurrences(out.bytes, ";R2") == 1 && occurrences(out.bytes, ";C") == 1,
            "three-source publication preserves each original comment");
        check(occurrences(out.bytes, "phase=merged") == 1, "three-source result has exactly one merged emission group");
        const std::size_t receiver = out.bytes.find(";R2");
        const std::size_t receiver_end = out.bytes.find('\n', receiver);
        const std::string receiver_line = out.bytes.substr(receiver, receiver_end - receiver);
        check(receiver_line.find("phase=merged") != std::string::npos, "R2 receives C in both source orders");
        const std::vector<double> receiver_nominal = fields(receiver_line, "de_nom_mm");
        check(receiver_nominal.size() == 1, "R2 has one merged nominal annotation");
        near(receiver_nominal[0], .00021, 1e-15, "R2 receives its own material plus C, not merely an internal phase merge");
        double nominal = 0.;
        for (double value : fields(out.bytes, "de_nom_mm")) nominal += value;
        near(nominal, .00041, 1e-15, "three-source nominal sum");
    }

    ++cases;
    Harness stream(config(.1, true, 3, 6));
    std::string text;
    std::vector<ReaderEvent> events;
    // More than both planner horizons makes open publication observable. The
    // collapsed third source can transfer forward to the fourth source.
    for (std::size_t row = 0; row < 105; ++row) {
        const double x = row == 2 ? 4.0001 : (row < 2 ? 2. * (row + 1) : 2. * row);
        const double delta = row == 2 ? .00001 : .2;
        text += "G1 X" + std::to_string(x) + " E" + std::to_string(delta) + " ;s" + std::to_string(row) + "\n";
        const double cumulative = row < 2 ? .2 * (row + 1) : .2 * row + .00001;
        events.push_back(event(MOVE, row, {{X, 0, x}, {E, 0, cumulative}}));
    }
    const FlowOutput prefix = stream.send(text, events, {true, false, false});
    check(!prefix.bytes.empty() && prefix.bytes.find(";s104") == std::string::npos, "stable publication emits a prefix, not the open tail");
    const FlowOutput tail = stream.send("", {}, {false, true, false});
    const std::string all = prefix.bytes + tail.bytes;
    check(occurrences(all, ";s2\n") == 1 && occurrences(all, ";s104") == 1, "cross-publication collapsed source and tail appear once");
    double sum = 0.;
    for (double value : numbers(all, 'E')) sum += value;
    near(sum, 20.80001, 2e-6, "streaming cycle material closes");

    ++cases;
    Harness retained(config(.1, true, 3, 6));
    std::string first_text = "G1 X2 E.2 ;retained-anchor\nG1 X2.0001 E.00001 ;later-collapsed\nM106 S1\n";
    std::vector<ReaderEvent> first_events{
        event(MOVE, 0, {{X, 0, 2}, {E, 0, .2}}),
        event(MOVE, 1, {{X, 0, 2.0001}, {E, 0, .20001}})};
    for (std::size_t index = 0; index < 99; ++index) {
        const double x = 4.0001 + 2. * index;
        first_text += "G1 X" + std::to_string(x) + " E.2\n";
        first_events.push_back(event(MOVE, index + 3, {{X, 0, x}, {E, 0, .40001 + .2 * index}}));
    }
    // The 101st insertion publishes only R; C is still in the planner. The
    // following insertion publishes C, whose fan separator requires R behind it.
    const FlowOutput first_prefix = retained.send(first_text, first_events, {true, false, false});
    check(first_prefix.bytes.empty(), "last representable published source is withheld before later collapsed publication");
    check(first_prefix.issues.empty(), "collapsed source has not yet been projected in first publication");
    const FlowOutput later_prefix = retained.send("G1 X202.0001 E.2\n", {
        event(MOVE, 0, {{X, 0, 202.0001}, {E, 0, 20.20001}})});
    check(later_prefix.bytes.empty(), "backward receiver stays un-emitted until its suffix is resolved");
    check(later_prefix.issues.size() == 1 && later_prefix.issues[0].kind == FlowIssueKind::GeometryMerged &&
        later_prefix.issues[0].line_index == 1 && later_prefix.issues[0].source_ordinal == 7,
        "collapsed source is projected only in the later publication");
    const FlowOutput retained_tail = retained.send("", {}, {false, true, false});
    const std::size_t anchor_end = retained_tail.bytes.find('\n');
    const std::string anchor_line = retained_tail.bytes.substr(0, anchor_end);
    check(anchor_line.find(";retained-anchor") != std::string::npos && anchor_line.find("phase=merged") != std::string::npos,
        "retained R itself receives the later collapsed source");
    const std::vector<double> anchor_nominal = fields(anchor_line, "de_nom_mm");
    check(anchor_nominal.size() == 1, "retained R has exactly one merged group");
    near(anchor_nominal[0], .20001, 1e-14, "retained anchor nominal includes C");
    check(retained_tail.bytes.find(";later-collapsed\nM106 S1\n") != std::string::npos,
        "backward transfer keeps collapsed residue and fan in order");
    double retained_sum = 0.;
    for (double value : numbers(retained_tail.bytes, 'E')) retained_sum += value;
    near(retained_sum, 20.20001, 2e-6, "successive-publication backward transfer closes total material");
}

void test_boundaries_errors()
{
    ++cases;
    Harness h;
    check(h.send("G1 X10 E1\n", {event(MOVE, 0, {{X, 0, 10}, {E, 0, 1}})}, {true, false, false}).bytes.empty(),
        "open cycle buffers its source");
    const FlowOutput end = h.send("G1 X20 E1 ;script\n", {event(MOVE, 0, {{X, 0, 20}, {E, 0, 2}})}, {false, false, true});
    check(occurrences(end.bytes, ";NX_PRESSURE") == 3 && end.bytes.find("G1 X20 E1 ;script\n") > 0,
        "EndPrint closes before unchanged outside-layer script");

    ++cases;
    Harness lf;
    const FlowOutput a = lf.send(";first");
    check(a.bytes == ";first", "no LF at actual current EOF");
    check(lf.send("").bytes.empty(), "empty fragment creates no separator");
    const FlowOutput b = lf.send(";second");
    check(b.bytes == "\n;second", "next real fragment separates previous unterminated comment");
    check(lf.send("", {}, {false, false, true}).bytes.empty(), "true EOF stays unterminated");

    ++cases;
    Harness cr;
    check(cr.send(";first\r").bytes == ";first\r", "isolated source CR is preserved at current EOF");
    check(cr.send("").bytes.empty(), "empty fragment does not append LF after CR");
    check(cr.send(";second").bytes == "\n;second", "following real text adds LF only, without moving the source CR");
    check(cr.send("", {}, {false, false, true}).bytes.empty(), "final unterminated text remains unchanged at true EOF");

    ++cases;
    Harness rewritten;
    const FlowOutput r1 = rewritten.send("G1 X10 E1", {event(MOVE, 0, {{X, 0, 10}, {E, 0, 1}})}, {true, true, false});
    check(!r1.bytes.empty() && r1.bytes.back() != '\n', "rewritten final source retains absent LF");
    check(rewritten.send(";later\n").bytes == "\n;later\n", "LF continuity applies after rewritten text too");

    ++cases;
    Harness failure;
    FlowOutput escaped{"sentinel", {}};
    bool invalid = false;
    try { escaped = failure.send(";would-have-been-prefix\n; PRESSURE_FACTOR=-1\n"); }
    catch (const ControlError &) { invalid = true; }
    check(invalid && escaped.bytes == "sentinel", "fatal call returns no partial block");
    bool abandoned = false;
    try { failure.send(""); } catch (const std::logic_error &) { abandoned = true; }
    check(abandoned, "failed instance cannot be reused");
}

FlowConfig config(double k, bool annotate, int xyz_digits, int e_digits)
{
    return {{{k, annotate}, {k, annotate}}, {'A', 'B'}, xyz_digits, e_digits,
        {{0, 0, 0, 0}, {0, 0, 0, 0}, 0}, false, 1};
}

ReaderEvent event(std::uint32_t kind, std::size_t row, std::vector<Item> items)
{
    items.push_back({RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, double(row)});
    return {kind, std::move(items)};
}

SourceFragment source(std::string text, Tracker &prior, std::vector<ReaderEvent> events)
{
    prior.set_line_number(0);
    ReaderFragment reader{prior, std::move(events), prior};
    for (const ReaderEvent &delta : reader.events)
        reader.final.apply({delta.kind, delta.items.data(), std::uint32_t(delta.items.size())});
    reader.final.set_line_number(std::uint32_t(std::count(text.begin(), text.end(), '\n')));
    prior = reader.final;
    return index_fragment(std::move(text), std::move(reader));
}

Harness::Harness(FlowConfig settings, bool absolute) : flow(settings)
{
    const FlowOutput preamble = send(std::string("G71\nG90\n") + (absolute ? "M82\n" : "M83\n") +
        "T0\nG92 X0 Y0 Z0 E0\nG1 F120\n", {
        event(UPDATE, 0, {{MM, 0, 1}}), event(UPDATE, 1, {{XYZ, 0, 1}}),
        event(UPDATE, 2, {{EA, 0, absolute ? 1. : 0.}}), event(UPDATE, 3, {{TOOL, 0, 0}}),
        event(RESET, 4, {{X, 0, 0}, {Y, 0, 0}, {Z, 0, 0}, {E, 0, 0}, {E, 1, 0}}),
        event(MOVE, 5, {{F, 0, 120}})});
    check(preamble.bytes.find("G71\n") == 0 && preamble.issues.empty(), "preamble followed outside layers");
}

FlowOutput Harness::send(std::string text, std::vector<ReaderEvent> events,
                         FlowBoundary boundary, std::vector<MoveHint> hints)
{
    return flow.consume(source(std::move(text), prior, std::move(events)), hints, boundary);
}

std::vector<double> numbers(const std::string &text, char letter)
{
    std::vector<double> result;
    Parser parser;
    parser.parse_buffer(text, [&](Parser &, const Parser::GCodeLine &line) {
        const std::optional<float> value = line.parameter_value(letter);
        if (value) result.push_back(*value);
    });
    return result;
}

std::vector<double> fields(const std::string &text, const std::string &key)
{
    std::vector<double> result;
    std::size_t cursor = 0;
    while ((cursor = text.find(";NX_PRESSURE ", cursor)) != std::string::npos) {
        const std::size_t end = text.find('\n', cursor);
        const std::size_t field = text.find(" " + key + "=", cursor);
        check(field != std::string::npos && (end == std::string::npos || field < end), "annotation field exists on this row");
        result.push_back(std::stod(text.substr(field + key.size() + 2)));
        cursor += 13;
    }
    return result;
}

std::size_t occurrences(const std::string &text, const std::string &needle)
{
    std::size_t count = 0, cursor = 0;
    while ((cursor = text.find(needle, cursor)) != std::string::npos) {
        ++count;
        cursor += needle.size();
    }
    return count;
}

void check(bool condition, const char *message)
{
    ++checks;
    if (!condition) throw std::runtime_error(message);
}

void near(double actual, double expected, double tolerance, const char *message)
{
    check(std::isfinite(actual) && std::abs(actual - expected) <= tolerance, message);
}
} // namespace
