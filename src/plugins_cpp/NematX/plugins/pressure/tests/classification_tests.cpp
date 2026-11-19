///|/ Released under the GNU Affero General Public License, version 3 or later.

#include "PressureClassification.hpp"
#include "PressureNominal.hpp"
#include "PressureSource.hpp"

#include <algorithm>
#include <cstdio>
#include <limits>
#include <string>
#include <utility>

#include "../../nematx_firmware/src/NematXSyntax.hpp"
#include "libslic3r/Api/plugin/c/slic3r_extrusions.h"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeLineParser.hpp"

// Tests line decisions with literal oracles, independent of their implementation.
// Explicit manual Reader events pass public SDK apply before the real nominal
// component computes doubles. No provider or substitute interpreter is present.
// SDK-valid structural counterexamples are labeled separately from representative
// NematX event sequences. Parser/Syntax checks verify lexical assumptions only;
// apply alone cannot prove that a provider would produce a sequence.
// Every dependent index and optional access follows a throwing structural guard,
// including Release builds. All counters count actual executed cases/checks.
//
// Call tree:
//   main: run relative continuity, modes, physical moves, controls and failures.
//     test_relative / test_modes / test_noops / test_moves / test_controls
//     test_volume_reader / test_failures
//       source: validate explicit events, then index original owned text.
//         event: attach a literal physical line number.
//         seed: validate manual initial state.
//       classify: compute real nominal doubles before invoking classification.
//       expect / reject: compare literal decisions or a specific local error.
//       check: count and throw on the first failed assertion.

// Runs every case and returns nonzero on any exception, in every configuration.
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
    TOOL = RAW_GCODE_READER_ITEM_ACTIVE_TOOL, VOL = RAW_GCODE_READER_ITEM_VOLUMETRIC,
    ROLE = RAW_GCODE_READER_ITEM_EXTRUSION_ROLE, OBJECT = RAW_GCODE_READER_ITEM_OBJECT,
    MARK = RAW_GCODE_READER_ITEM_MARKER, DIAG = RAW_GCODE_READER_ITEM_DIAGNOSTIC_TYPE;
constexpr std::uint32_t MOVE = RAW_GCODE_READER_EVENT_LINEAR_MOVE,
    ARC = RAW_GCODE_READER_EVENT_ARC_MOVE, UPDATE = RAW_GCODE_READER_EVENT_STATE_UPDATE,
    INVALID = RAW_GCODE_READER_EVENT_INVALIDATE, RESET = RAW_GCODE_READER_EVENT_SET_POSITION,
    MARKER = RAW_GCODE_READER_EVENT_MARKER, DIAGNOSTIC = RAW_GCODE_READER_EVENT_DIAGNOSTIC;
const ClassificationContext active{true, false, {true, false}};
int cases = 0, checks = 0;

// Proves relative NematX continuity without an external E origin or reset.
void test_relative();
// Compares print snapshots with pre-motion and net standalone mode/tool updates.
void test_modes();
// Keeps residual post-print modes transparent on NOOPs while detecting this row's
// net changes, including Syntax updates emitted after the motion event.
void test_noops();
// Exercises physical nonprints, missing proofs, source intent and SDK-invalid words.
void test_moves();
// Checks wipe scope, transparent metadata, pauses and persistent volume distrust.
void test_controls();
// Checks exact unknown-command barriers for M200, repaired volume refusals and
// the distinct immediate failure before tool proof is restored.
void test_volume_reader();
// Exercises transactional failure, structural guards and scalar result ownership.
void test_failures();
// Applies public SDK validation to explicit events and owns their original text.
SourceFragment source(std::string text, Tracker initial, std::vector<ReaderEvent> events);
// Attaches the explicit zero-based physical row required by SDK validation.
ReaderEvent event(std::uint32_t kind, std::uint32_t line, std::vector<Item> items);
// Constructs a manual known state through the SDK's public update operation.
Tracker seed(std::vector<Item> items);
// Provides a known relative script context, with no external E coordinate or role.
Tracker relative_start();
// Matches the manual relative context's known coordinate keys, not Reader floats.
NominalState relative_prior();
// Supplies a cycle at tool zero with proven absolute XYZ, relative E and mm.
ClassificationState cycle();
// Reads doubles using the real nominal component; no classification oracle is computed.
ClassificationFragment classify(const SourceFragment &input,
    const std::vector<MoveHint> &hints = {}, const ClassificationContext &context = active,
    ClassificationState prior = cycle());
// Compares every decision against literal close flags and nominal motion indices.
void expect(const ClassificationFragment &result, const std::vector<bool> &close,
            const std::vector<int> &indices);
// Requires an exact owned cause and local row, never merely any thrown exception.
void reject(const SourceFragment &input, ClassificationCause cause,
    const std::vector<MoveHint> &hints = {}, const ClassificationContext &context = active);
// Counts a real assertion and prevents unsafe subsequent accesses on failure.
void check(bool condition, const char *message);
} // namespace

int main()
{
    try {
        test_relative();
        test_modes();
        test_noops();
        test_moves();
        test_controls();
        test_volume_reader();
        test_failures();
    } catch (const std::exception &error) {
        std::printf("FAILED: %s (%d cases, %d checks)\n", error.what(), cases, checks);
        return 1;
    }
    std::printf("pressure classification: %d cases, %d checks, 0 failures\n", cases, checks);
    return 0;
}

namespace {
void test_relative()
{
    ++cases;
    // NematX generates G71 and inline modes, not G21/M83. The first A word
    // proves tool zero and relative E before its adjacent unknown-origin event.
    const SourceFragment preamble = source("G71\nG90\nG1 X0 Y0 Z0 F600\n", Tracker(), {
        event(UPDATE, 0, {{MM, 0, 1}}), event(UPDATE, 1, {{XYZ, 0, 1}}),
        event(MOVE, 2, {{X, 0, 0}, {Y, 0, 0}, {Z, 0, 0}, {F, 0, 600}})});
    const NominalFragment p = read_nominal_fragment(preamble, {}, {'A', 'B'});
    const ClassificationFragment pc = classify_fragment(preamble, p, {}, {false, false, {true, false}}, {});
    expect(pc, {false, false, true}, {-1, -1, -1});
    check(!preamble.reader.initial.value(VOL), "Reader starts without volume configuration");
    const SourceFragment first = source("G1 G90 X1 G91 A.1\n", preamble.reader.final, {
        event(UPDATE, 0, {{TOOL, 0, 0}}), event(UPDATE, 0, {{EA, 0, 0}}),
        event(INVALID, 0, {{E, 0, 0}}), event(MOVE, 0, {{X, 0, 1}}),
        event(UPDATE, 0, {{XYZ, 0, 0}})});
    const NominalFragment a = read_nominal_fragment(first, p.final, {'A', 'B'});
    check(a.motions.size() == 1, "first relative motion shape");
    check(a.motions[0].e_delta == .1 && !a.motions[0].e_start && !a.motions[0].e_end &&
          a.final.e.empty(), "exact .1 delta needs no forged external origin");
    const ClassificationFragment ac = classify_fragment(first, a, {}, active, pc.final);
    expect(ac, {false}, {0});
    const SourceFragment second = source("G1 G90 X2 G91 A.2\n", first.reader.final, {
        event(UPDATE, 0, {{XYZ, 0, 1}}), event(INVALID, 0, {{E, 0, 0}}),
        event(MOVE, 0, {{X, 0, 2}}), event(UPDATE, 0, {{XYZ, 0, 0}})});
    const NominalFragment b = read_nominal_fragment(second, a.final, {'A', 'B'});
    check(b.motions.size() == 1, "second relative motion shape");
    check(b.motions[0].e_delta == .2 && b.final.e.empty(), "exact .2 delta without absolute E");
    const ClassificationFragment bc = classify_fragment(second, b, {}, active, ac.final);
    expect(bc, {false}, {0});
    check(bc.final.cycle && bc.final.cycle->xyz_absolute && !bc.final.cycle->e_absolute,
          "snapshot is pre-motion, not final G91");
    check(second.reader.final.xyz_is_absolute() == false, "Reader post-motion XYZ is relative");
    nematx_firmware::ReaderModes modes;
    modes.xyz_absolute = true;
    const std::optional<std::vector<nematx_firmware::ReaderAction>> actions =
        nematx_firmware::translate_line("G1 G90 X1 G91 A.1", {'A', 'B'}, modes);
    check(actions && !actions->empty(), "existing Syntax accepts representative generated move");
    bool relative = false, standard = false;
    for (const nematx_firmware::ReaderAction &action : *actions) {
        relative = relative || action.kind == nematx_firmware::ReaderAction::Kind::ExtrusionMode;
        standard = standard || action.kind == nematx_firmware::ReaderAction::Kind::Standard;
    }
    check(relative && standard, "Syntax emits mode and standard actions");

    ++cases;
    const SourceFragment unknown = source("G1 X1\n", seed({{X, 0, 0}, {Y, 0, 0}, {Z, 0, 0},
        {F, 0, 600}, {XYZ, 0, 1}, {MM, 0, 1}}), {event(MOVE, 0, {{X, 0, 1}})});
    reject(unknown, ClassificationCause::UnknownTool, {MoveHint::Extrusion});
    ++cases;
    const SourceFragment zero = source("G1 X1\n", relative_start(), {event(MOVE, 0, {{X, 0, 1}})});
    expect(classify(zero, {MoveHint::Extrusion}), {false}, {0});
    const NominalFragment zn = read_nominal_fragment(zero, relative_prior(), {'A', 'B'});
    check(zn.motions.size() == 1, "absent E shape");
    check(zn.motions[0].e_delta == 0 && !zn.motions[0].tokens[3] && zn.final.e.empty(),
          "generated zero absent E retains no external key");

    ++cases;
    const SourceFragment absolute = source("M82\nG1 X1 E.1\n", relative_start(), {
        event(UPDATE, 0, {{EA, 0, 1}}), event(MOVE, 1, {{X, 0, 1}, {E, 0, double(float(.1))}})});
    reject(absolute, ClassificationCause::UnknownExtrusion);

    ++cases;
    const SourceFragment absolute_known = source("G92 E2\nM82\nG1 X1 E2.25\n", relative_start(), {
        event(RESET, 0, {{E, 0, 2}}), event(UPDATE, 1, {{EA, 0, 1}}),
        event(MOVE, 2, {{X, 0, 1}, {E, 0, 2.25}})});
    const ClassificationFragment abs = classify(absolute_known);
    expect(abs, {true, false, false}, {-1, -1, 0});
    check(abs.final.cycle && abs.final.cycle->e_absolute, "absolute print with real nominal origin accepted");
    ++cases;
    const SourceFragment absolute_absent = source("G92 E2\nM82\nG1 X1\n", relative_start(), {
        event(RESET, 0, {{E, 0, 2}}), event(UPDATE, 1, {{EA, 0, 1}}), event(MOVE, 2, {{X, 0, 1}})});
    expect(classify(absolute_absent, {MoveHint::None, MoveHint::None, MoveHint::Extrusion}),
        {true, false, false}, {-1, -1, 0});
    const NominalFragment absent = read_nominal_fragment(absolute_absent, relative_prior(), {'A', 'B'});
    check(absent.motions.size() == 1, "absolute absent-E motion shape");
    check(absent.motions[0].e_absolute == true && !absent.motions[0].tokens[3] &&
          absent.motions[0].e_start == 2 && absent.motions[0].e_end == 2 &&
          absent.motions[0].e_delta == 0, "absolute absent-E has proven origin, endpoint and zero delta");
}

void test_modes()
{
    struct Sample { const char *text; std::vector<ReaderEvent> events; std::vector<bool> close; };
    const std::vector<Sample> samples{
        {"G90\nG21\nT0\n", {event(UPDATE, 0, {{XYZ, 0, 1}}),
            event(UPDATE, 1, {{MM, 0, 1}}), event(UPDATE, 2, {{TOOL, 0, 0}})}, {false, false, false}},
        {"G91\n", {event(UPDATE, 0, {{XYZ, 0, 0}})}, {true}},
        {"M82\n", {event(UPDATE, 0, {{EA, 0, 1}})}, {true}},
        {"G20\n", {event(UPDATE, 0, {{MM, 0, 0}})}, {true}},
        {"T1\n", {event(UPDATE, 0, {{TOOL, 0, 1}})}, {true}},
        // SDK-valid structural net-slot sequence, not the Reader output for this text.
        {"G90 G91\n", {event(UPDATE, 0, {{XYZ, 0, 0}}),
            event(UPDATE, 0, {{XYZ, 0, 1}})}, {false}}};
    for (const Sample &sample : samples) {
        ++cases;
        const SourceFragment input = source(sample.text, relative_start(), sample.events);
        expect(classify(input), sample.close, std::vector<int>(sample.close.size(), -1));
    }
    ++cases;
    // Raw Reader already has G91, but the printing snapshot still proves G90.
    Tracker post = relative_start();
    const ReaderEvent change = event(UPDATE, 0, {{XYZ, 0, 0}});
    post.apply({change.kind, change.items.data(), std::uint32_t(change.items.size())});
    expect(classify(source("G91\n", post, {change})), {true}, {-1});
    ++cases;
    const SourceFragment mixed = source("G1 G90 X1 G91 A.1\nG1 G90 X2 G91 A.2\n", relative_start(), {
        event(INVALID, 0, {{E, 0, 0}}), event(MOVE, 0, {{X, 0, 1}}),
        event(UPDATE, 0, {{XYZ, 0, 0}}), event(UPDATE, 1, {{XYZ, 0, 1}}),
        event(INVALID, 1, {{E, 0, 0}}), event(MOVE, 1, {{X, 0, 2}}),
        event(UPDATE, 1, {{XYZ, 0, 0}})});
    expect(classify(mixed), {false, false}, {0, 1});
    ++cases;
    const SourceFragment tool = source("G1 G90 X1 G91 B.1\nG1 G90 X2 G91 B.2\n", relative_start(), {
        event(UPDATE, 0, {{TOOL, 0, 1}}), event(INVALID, 0, {{E, 1, 0}}),
        event(MOVE, 0, {{X, 0, 1}}), event(UPDATE, 0, {{XYZ, 0, 0}}),
        event(UPDATE, 1, {{XYZ, 0, 1}}), event(INVALID, 1, {{E, 1, 0}}),
        event(MOVE, 1, {{X, 0, 2}}), event(UPDATE, 1, {{XYZ, 0, 0}})});
    const ClassificationFragment t = classify(tool, {}, {true, false, {true, true}});
    expect(t, {true, false}, {0, 1});
    check(t.final.cycle && t.final.cycle->tool == 1, "tool changes and same-tool reassertion");
    ++cases;
    // Without inline G91, B selects absolute E even from a relative E state.
    const SourceFragment tool_absolute = source("G1 X1 B.1\n", relative_start(), {
        event(UPDATE, 0, {{TOOL, 0, 1}}), event(UPDATE, 0, {{EA, 0, 1}}),
        event(MOVE, 0, {{X, 0, 1}, {E, 1, double(float(.1))}})});
    reject(tool_absolute, ClassificationCause::UnknownExtrusion, {}, {true, false, {true, true}});
    ++cases;
    const SourceFragment reset = source("G92 X0\n", relative_start(), {event(RESET, 0, {{X, 0, 0}})});
    expect(classify(reset), {true}, {-1});
    ++cases;
    expect(classify(source("G92\n", relative_start(), {event(RESET, 0, {})})), {true}, {-1});
    ++cases;
    // SDK-valid structural event category; the current Reader emits SET_POSITION,
    // not SET_ORIGIN, for G92. No coordinate provenance is claimed by this case.
    const SourceFragment origin = source("G92\n", relative_start(), {
        event(RAW_GCODE_READER_EVENT_SET_ORIGIN, 0, {})});
    expect(classify(origin), {true}, {-1});

    ++cases;
    // SDK-valid structural invalidation on a motion row, not provider output.
    const SourceFragment mode_barrier = source("G1 G90 X1 G91 A.1\n", relative_start(), {
        event(INVALID, 0, {{MM, 0, 0}}), event(UPDATE, 0, {{MM, 0, 1}}),
        event(INVALID, 0, {{E, 0, 0}}), event(MOVE, 0, {{X, 0, 1}}),
        event(UPDATE, 0, {{XYZ, 0, 0}})});
    expect(classify(mode_barrier), {true}, {0});
}

void test_noops()
{
    ++cases;
    const SourceFragment input = source("G1 G90 X1 G91 A.1\nG1 F1200\nG1 X0\nG1 G90 X2 G91 A.2\n",
        relative_start(), {event(INVALID, 0, {{E, 0, 0}}), event(MOVE, 0, {{X, 0, 1}}),
        event(UPDATE, 0, {{XYZ, 0, 0}}), event(MOVE, 1, {{F, 0, 1200}}),
        event(MOVE, 2, {{X, 0, 1}}), event(UPDATE, 3, {{XYZ, 0, 1}}),
        event(INVALID, 3, {{E, 0, 0}}), event(MOVE, 3, {{X, 0, 2}}),
        event(UPDATE, 3, {{XYZ, 0, 0}})});
    const NominalFragment nominal = read_nominal_fragment(input, relative_prior(), {'A', 'B'});
    check(nominal.motions.size() == 4, "NOOPs also occupy nominal motion indices");
    check(nominal.motions[0].xyz_absolute == true && nominal.motions[1].xyz_absolute == false &&
          nominal.motions[2].xyz_absolute == false && nominal.motions[3].xyz_absolute == true,
          "real mixed lines leave residual G91 only on intervening NOOPs");
    check(nominal.motions[2].start[0] == 1 && nominal.motions[2].end[0] == 1 &&
          nominal.motions[3].feedrate == 1200 && input.reader.final.xyz_is_absolute() == false,
          "relative X0 is stationary and F-only still updates feed");
    const ClassificationFragment result = classify_fragment(input, nominal, {}, active, cycle());
    expect(result, {false, false, false, false}, {0, -1, -1, 3});
    check(result.final.cycle && result.final.cycle->xyz_absolute,
          "NOOP transparency never rewrites the accepted-print snapshot");

    for (bool relative : {true, false}) {
        ++cases;
        const char *text = relative ? "G1 G91 F1200" : "G1 G90 F1200";
        nematx_firmware::ReaderModes modes;
        modes.xyz_absolute = true;
        modes.e_absolute = false;
        modes.active_tool = 0;
        const std::optional<std::vector<nematx_firmware::ReaderAction>> actions =
            nematx_firmware::translate_line(text, {'A', 'B'}, modes);
        check(actions && actions->size() == (relative ? 2 : 1), "Syntax NOOP action count");
        check(actions->front().kind == nematx_firmware::ReaderAction::Kind::Standard &&
              actions->front().text == "G1 F1200", "Syntax emits NOOP Standard before mode change");
        Parser parser;
        Parser::GCodeLine line;
        parser.parse_line(actions->front().text, [&](Parser &, const Parser::GCodeLine &parsed) { line = parsed; });
        check(line.command() == "G1" && line.parameter_value('F') == 1200 &&
              !line.has_parameter('X') && !line.has_parameter('E'), "SDK parses F-only Standard");
        std::vector<ReaderEvent> events{event(MOVE, 0, {{F, 0, 1200}})};
        if (relative) {
            check(actions->back().kind == nematx_firmware::ReaderAction::Kind::XyzMode &&
                  !actions->back().absolute, "real XYZ-relative update follows MOVE");
            events.push_back(event(UPDATE, 0, {{XYZ, 0, 0}}));
        }
        expect(classify(source(std::string(text) + "\n", relative_start(), events)), {relative}, {-1});
    }
    ++cases;
    // SDK-valid structural updates on both sides of a NOOP prove that only the
    // row's net updated slots matter, not their intermediate values or ordering.
    expect(classify(source("G1 F1200\n", relative_start(), {event(UPDATE, 0, {{XYZ, 0, 0}}),
        event(MOVE, 0, {{F, 0, 1200}}), event(UPDATE, 0, {{XYZ, 0, 1}})})), {false}, {-1});
}

void test_moves()
{
    // These reduced manual movement/hint cases isolate classification. Missing
    // diagnostics and producer hints are not claims of exact provider output.
    struct Sample { const char *text; std::vector<ReaderEvent> events; MoveHint hint;
                    bool close; int index; };
    const std::vector<Sample> samples{
        {"G1 X1 E.1\n", {event(INVALID, 0, {{E, 0, 0}}), event(MOVE, 0, {{X, 0, 1}})},
            MoveHint::None, false, 0},
        {"G1 X1 E.1\n", {event(INVALID, 0, {{E, 0, 0}}), event(MOVE, 0, {{X, 0, 1}})},
            MoveHint::Travel, true, -1},
        {"G1 X1 E.1\n", {event(INVALID, 0, {{E, 0, 0}}), event(MOVE, 0, {{X, 0, 1}})},
            MoveHint::Wipe, true, -1},
        {"G1 X1 E-.1\n", {event(INVALID, 0, {{E, 0, 0}}), event(MOVE, 0, {{X, 0, 1}})},
            MoveHint::Extrusion, true, -1},
        {"G1 E.1\n", {event(INVALID, 0, {{E, 0, 0}}), event(MOVE, 0, {})},
            MoveHint::Extrusion, true, -1},
        {"G1 X0 E.1\n", {event(INVALID, 0, {{E, 0, 0}}), event(MOVE, 0, {{X, 0, 0}})},
            MoveHint::None, true, -1},
        {"G1 X.00000001 E.1\n", {event(INVALID, 0, {{E, 0, 0}}),
            event(MOVE, 0, {{X, 0, double(float(.00000001))}})}, MoveHint::None, false, 0},
        {"G1 X1\n", {event(MOVE, 0, {{X, 0, 1}})}, MoveHint::None, true, -1},
        {"G1 X1 E0\n", {event(INVALID, 0, {{E, 0, 0}}), event(MOVE, 0, {{X, 0, 1}})},
            MoveHint::Extrusion, false, 0},
        {"G1 F1200\n", {event(MOVE, 0, {{F, 0, 1200}})}, MoveHint::None, false, -1},
        {"G1 X0\n", {event(MOVE, 0, {{X, 0, 0}})}, MoveHint::None, false, -1},
        {"G1 Xbad\n", {event(INVALID, 0, {{X, 0, 0}}), event(MOVE, 0, {})},
            MoveHint::None, true, -1}};
    for (const Sample &sample : samples) {
        ++cases;
        const SourceFragment input = source(sample.text, relative_start(), sample.events);
        expect(classify(input, {sample.hint}), {sample.close}, {sample.index});
    }
    ++cases;
    const SourceFragment invalid_xyz = source("G1 Xbad E.1\n", relative_start(), {
        event(INVALID, 0, {{X, 0, 0}, {E, 0, 0}}), event(MOVE, 0, {})});
    reject(invalid_xyz, ClassificationCause::UnknownCoordinates);
    ++cases;
    Parser parser;
    Parser::GCodeLine lexical;
    parser.parse_line("G1 X1 E+0.1", [&](Parser &, const Parser::GCodeLine &line) { lexical = line; });
    float accepted = 0;
    check(!lexical.parameter_value('E', accepted), "SDK rejects leading plus E before oracle");
    const SourceFragment invalid_e = source("G1 X1 E+0.1\n", relative_start(), {
        event(INVALID, 0, {{E, 0, 0}}), event(MOVE, 0, {{X, 0, 1}})});
    reject(invalid_e, ClassificationCause::UnknownExtrusion);
    ++cases;
    const SourceFragment invalid_only = source("G1 E+0.1\n", relative_start(), {
        event(INVALID, 0, {{E, 0, 0}}), event(MOVE, 0, {})});
    expect(classify(invalid_only), {true}, {-1});
    ++cases;
    const SourceFragment no_tool = source("G1 X1 E.1\n", seed({{X, 0, 0}, {Y, 0, 0}, {Z, 0, 0},
        {F, 0, 600}, {XYZ, 0, 1}, {EA, 0, 0}, {MM, 0, 1}}), {
        event(INVALID, 0, {{E, 0, 0}}), event(MOVE, 0, {{X, 0, 1}})});
    reject(no_tool, ClassificationCause::UnknownTool);
    ++cases;
    expect(classify(no_tool, {MoveHint::Travel}), {true}, {-1});
    ++cases;
    // SDK-valid structural tool-bound check, not a tool selection parsed from G1.
    reject(source("G1 X1 E.1\n", relative_start(), {event(UPDATE, 0, {{TOOL, 0, 9}}),
        event(INVALID, 0, {{E, 9, 0}}), event(MOVE, 0, {{X, 0, 1}})}), ClassificationCause::UnknownTool);
    ++cases;
    reject(source("G1 X1 E.1\n", relative_start(), {event(INVALID, 0, {{E, 0, 0}}),
        event(MOVE, 0, {{X, 0, 1}})}), ClassificationCause::UnknownTool, {}, {true, false, {}});

    const SourceFragment arc = source("G2 X1 E.1\n", relative_start(), {event(INVALID, 0, {{E, 0, 0}}),
        event(ARC, 0, {{X, 0, 1}, {RAW_GCODE_READER_ITEM_ARC_CLOCKWISE, 0, 1}})});
    for (MoveHint hint : {MoveHint::None, MoveHint::Travel, MoveHint::Wipe}) {
        ++cases;
        reject(arc, ClassificationCause::UnsupportedArc, {hint});
    }
    ++cases;
    expect(classify(arc, {}, {true, true, {false}}), {true}, {-1});
    ++cases;
    const ClassificationFragment outside = classify(arc, {}, {false, true, {}});
    expect(outside, {true}, {-1});
    check(!outside.final.cycle, "outside layer clears only logical cycle");

    // These pairs deliberately probe SDK-valid structure, not provider output.
    const std::vector<std::vector<ReaderEvent>> barriers{
        {event(INVALID, 0, {{E, 1, 0}}), event(INVALID, 0, {{E, 0, 0}}), event(MOVE, 0, {{X, 0, 1}})},
        {event(INVALID, 0, {{E, 0, 0}}), event(UPDATE, 0, {}), event(MOVE, 0, {{X, 0, 1}})},
        {event(INVALID, 0, {{E, 0, 0}, {MM, 0, 0}}), event(UPDATE, 0, {{MM, 0, 1}}),
         event(INVALID, 0, {{E, 0, 0}}), event(MOVE, 0, {{X, 0, 1}})}};
    for (const std::vector<ReaderEvent> &events : barriers) {
        ++cases;
        // A nonadjacent invalidation still needs a separate adjacent target
        // proof for Nominal; the middle case has that target in the MOVE itself.
        if (events.size() == 3 && events[1].kind == UPDATE) {
            const SourceFragment input = source("G1 X1 E.1\n", seed({{X, 0, 0}, {Y, 0, 0},
                {Z, 0, 0}, {F, 0, 600}, {XYZ, 0, 1}, {EA, 0, 1}, {MM, 0, 1}, {TOOL, 0, 0}}),
                {event(INVALID, 0, {{E, 0, 0}}), event(UPDATE, 0, {}),
                 event(MOVE, 0, {{X, 0, 1}, {E, 0, double(float(.1))}})});
            reject(input, ClassificationCause::UnknownExtrusion);
        } else {
            expect(classify(source("G1 X1 E.1\n", relative_start(), events)), {true}, {0});
        }
    }

    ++cases;
    // An E-only invalidation on an earlier physical row cannot be excused by
    // the accepted relative print on the next row.
    const SourceFragment earlier = source(";invalidated\nG1 X1 E.1\n", relative_start(), {
        event(INVALID, 0, {{E, 0, 0}}), event(INVALID, 1, {{E, 0, 0}}), event(MOVE, 1, {{X, 0, 1}})});
    expect(classify(earlier), {true, false}, {-1, 0});
    ++cases;
    const SourceFragment negative_unknown = source("G1 X1 E-.1\n", seed({{X, 0, 0}, {Y, 0, 0},
        {Z, 0, 0}, {F, 0, 600}, {XYZ, 0, 1}, {EA, 0, 0}, {MM, 0, 1}}), {
        event(INVALID, 0, {{E, 0, 0}}), event(MOVE, 0, {{X, 0, 1}})});
    expect(classify(negative_unknown, {MoveHint::Extrusion}), {true}, {-1});
}

void test_controls()
{
    // Source-only delimiter cases deliberately omit Reader role updates. They
    // isolate lexical scope; representative TYPE/role sequences appear below.
    const std::vector<std::string> wipe_starts{";WIPE_START\n", " N10 ; \tWIPE_START \t\n",
        ";WIPE_START\rignored\n", std::string(";WIPE_START\0ignored\n", 20)};
    for (const std::string &text : wipe_starts) {
        ++cases;
        const ClassificationFragment begin = classify(source(text, relative_start(), {}));
        check(begin.final.wipe_scope, "exact wipe delimiter opens scope");
        const SourceFragment typed = source(";TYPE:Perimeter\n", relative_start(), {
            event(UPDATE, 0, {{ROLE, 0, RAW_EXTRUSION_ROLE_PERIMETER}})});
        const ClassificationFragment outside = classify(typed, {}, {false, false, {true}}, begin.final);
        check(outside.final.wipe_scope && !outside.final.cycle, "TYPE and outside layer preserve wipe");
        const SourceFragment printing = source("G1 X1 E.1\n", relative_start(), {
            event(INVALID, 0, {{E, 0, 0}}), event(MOVE, 0, {{X, 0, 1}})});
        expect(classify(printing, {}, active, outside.final), {true}, {-1});
        const ClassificationFragment end = classify(source("; WIPE_END \t\n", relative_start(), {}),
            {}, active, outside.final);
        check(!end.final.wipe_scope, "only explicit end clears wipe scope");
        expect(classify(printing, {}, active, end.final), {false}, {0});
    }
    for (const std::string &text : {std::string("G1 F600 ;WIPE_START\n"),
         std::string(";WIPE_START extra\n"), std::string(";\vWIPE_START\n"),
         std::string(";WIPE_END\n"), std::string("\r;WIPE_START\n")}) {
        ++cases;
        const ClassificationFragment result = classify(source(text, relative_start(), {}));
        check(!result.final.wipe_scope, "non-delimiter and unmatched end do not open scope");
    }
    ++cases;
    // SDK-valid structural ROLE invalidation under lexical wipe scope; a valid
    // TYPE:Perimeter actually publishes the separate update tested below.
    const SourceFragment role = source(";WIPE_START\n;TYPE:Perimeter\nG1 X1 E.1\n", relative_start(), {
        event(UPDATE, 0, {{ROLE, 0, RAW_EXTRUSION_ROLE_WIPE}}),
        event(INVALID, 1, {{ROLE, 0, 0}}), event(INVALID, 2, {{E, 0, 0}}), event(MOVE, 2, {{X, 0, 1}})});
    expect(classify(role), {false, false, true}, {-1, -1, -1});
    ++cases;
    const SourceFragment typed_scope = source(";WIPE_START\n;TYPE:Perimeter\nG1 X1 E.1\n;WIPE_END\nG1 X2 E.2\n",
        relative_start(), {event(UPDATE, 0, {{ROLE, 0, RAW_EXTRUSION_ROLE_WIPE}}),
        event(UPDATE, 1, {{ROLE, 0, RAW_EXTRUSION_ROLE_PERIMETER}}),
        event(INVALID, 2, {{E, 0, 0}}), event(MOVE, 2, {{X, 0, 1}}),
        event(UPDATE, 3, {{ROLE, 0, RAW_EXTRUSION_ROLE_PERIMETER}}),
        event(INVALID, 4, {{E, 0, 0}}), event(MOVE, 4, {{X, 0, 2}})});
    expect(classify(typed_scope), {false, false, true, false, false}, {-1, -1, -1, -1, 1});
    ++cases;
    // SDK-valid structural role assignment, not Reader output: Wipe is not a
    // recognized TYPE name. The representative invalid-TYPE case is separate.
    const SourceFragment role_only = source(";TYPE:Wipe\nG1 X1 E.1\n", relative_start(), {
        event(UPDATE, 0, {{ROLE, 0, RAW_EXTRUSION_ROLE_WIPE}}),
        event(INVALID, 1, {{E, 0, 0}}), event(MOVE, 1, {{X, 0, 1}})});
    expect(classify(role_only), {false, true}, {-1, -1});
    ++cases;
    // Manually seed the prior role to isolate role classification; a preceding
    // Reader fragment can establish WIPE without the current G1 establishing it.
    Tracker initial_role = relative_start();
    const ReaderEvent wipe_role = event(UPDATE, 0, {{ROLE, 0, RAW_EXTRUSION_ROLE_WIPE}});
    initial_role.apply({wipe_role.kind, wipe_role.items.data(), std::uint32_t(wipe_role.items.size())});
    expect(classify(source("G1 X1 E.1\n", initial_role, {event(INVALID, 0, {{E, 0, 0}}),
        event(MOVE, 0, {{X, 0, 1}})})), {true}, {-1});
    ++cases;
    // SDK-valid structural loss of role proof, not the response to TYPE:bad.
    expect(classify(source(";TYPE:bad\nG1 X1 E.1\n", initial_role, {
        event(INVALID, 0, {{ROLE, 0, 0}}), event(INVALID, 1, {{E, 0, 0}}),
        event(MOVE, 1, {{X, 0, 1}})})), {false, false}, {-1, 0});
    for (const char *text : {";TYPE:bad\nG1 X1 E.1\n", ";TYPE:Wipe\nG1 X1 E.1\n"}) {
        ++cases;
        const SourceFragment bad_type = source(text, initial_role, {
            event(DIAGNOSTIC, 0, {{DIAG, 0, RAW_GCODE_READER_DIAGNOSTIC_INVALID_COMMAND_PARAMETERS}}),
            event(INVALID, 1, {{E, 0, 0}}), event(MOVE, 1, {{X, 0, 1}})});
        expect(classify(bad_type), {false, true}, {-1, -1});
        check(bad_type.reader.final.value(ROLE) == RAW_EXTRUSION_ROLE_WIPE,
              "invalid TYPE preserves the previous Reader wipe role");
    }

    for (const char *command : {"G4", "G04 P1", "M109", "M129", "M190", "\vG04", "\fM109"}) {
        ++cases;
        const ClassificationFragment result = classify(source(std::string(command) + "\n", relative_start(), {}));
        expect(result, {true}, {-1});
    }
    ++cases;
    const SourceFragment wait = source("M109\n", relative_start(), {
        event(RAW_GCODE_READER_EVENT_WAIT_TEMPERATURE, 0,
            {{RAW_GCODE_READER_ITEM_TOOL_TEMPERATURE_REACHED, 0, 210}})});
    expect(classify(wait), {true}, {-1});
    for (const char *command : {"M109", "M129", "M190", "G4"}) {
        ++cases;
        const std::optional<std::vector<nematx_firmware::ReaderAction>> actions =
            nematx_firmware::translate_line(command, {'A', 'B'}, {});
        check(actions && actions->size() == 1, "existing Syntax owns wait and dwell recognition");
        const nematx_firmware::ReaderAction::Kind kind = actions->front().kind;
        check(kind == nematx_firmware::ReaderAction::Kind::WaitTool ||
              kind == nematx_firmware::ReaderAction::Kind::WaitBed ||
              kind == nematx_firmware::ReaderAction::Kind::Neutral, "representative wait/dwell action");
    }
    for (std::uint32_t marker : {RAW_GCODE_READER_MARKER_PAUSE_PRINT,
         RAW_GCODE_READER_MARKER_COLOR_CHANGE, RAW_GCODE_READER_MARKER_SEAM,
         RAW_GCODE_READER_MARKER_CUSTOM_GCODE}) {
        ++cases;
        // SDK-valid structural combinations: M109 itself closes regardless of
        // the marker. Independent marker-only decisions are asserted next.
        const SourceFragment input = source("M109 ;PAUSE_PRINT\n", relative_start(),
            {event(MARKER, 0, {{MARK, 0, double(marker)}})});
        expect(classify(input), {true}, {-1});
        ++cases;
        // SDK-valid structural marker injection on a comment-only row; no lexical
        // pause guard can mask PAUSE/COLOR closing or SEAM/CUSTOM transparency.
        expect(classify(source(";marker\n", relative_start(), {event(MARKER, 0, {{MARK, 0, double(marker)}})})),
            {marker == RAW_GCODE_READER_MARKER_PAUSE_PRINT || marker == RAW_GCODE_READER_MARKER_COLOR_CHANGE}, {-1});
    }
    ++cases;
    // Metadata items on ;object/;progress are SDK-valid structural examples,
    // not Reader responses to those two ordinary comments.
    const SourceFragment transparent = source(";M204 S100\nM104=210\nM106=255\n;TYPE:bad\n"
        ";object\n;progress\n", relative_start(), {
        event(RAW_GCODE_READER_EVENT_SET_TEMPERATURE, 1, {{RAW_GCODE_READER_ITEM_TOOL_TEMPERATURE, 0, 210}}),
        event(UPDATE, 2, {{RAW_GCODE_READER_ITEM_FAN_SPEED, 0, 255}}),
        event(DIAGNOSTIC, 3, {{DIAG, 0, RAW_GCODE_READER_DIAGNOSTIC_INVALID_COMMAND_PARAMETERS}}),
        event(INVALID, 4, {{OBJECT, 0, 0}}), event(UPDATE, 5, {})});
    expect(classify(transparent), {false, false, false, false, false, false}, {-1, -1, -1, -1, -1, -1});
    for (std::uint32_t diagnostic : {RAW_GCODE_READER_DIAGNOSTIC_UNKNOWN_COMMAND,
         RAW_GCODE_READER_DIAGNOSTIC_UNSUPPORTED_STATE_EFFECT}) {
        ++cases;
        expect(classify(source("M204 S100\n", relative_start(), {
            event(DIAGNOSTIC, 0, {{DIAG, 0, double(diagnostic)}}), event(INVALID, 0, {})})), {true}, {-1});
    }

    const SourceFragment print = source("G1 X1 E.1\n", relative_start(), {
        event(INVALID, 0, {{E, 0, 0}}), event(MOVE, 0, {{X, 0, 1}})});
    ++cases;
    reject(print, ClassificationCause::UnsupportedVolume, {}, {true, true, {true}});
    ++cases;
    expect(classify(print, {}, {true, true, {false}}), {true}, {-1});
    ++cases;
    expect(classify(print, {MoveHint::Travel}, {true, true, {true}}), {true}, {-1});
    ++cases;
    expect(classify(print, {}, {false, true, {true}}), {true}, {-1});
    // Source-only lexical cases omit unknown-command events on purpose. They
    // prove the guard/latch, not Reader barriers or global failure priority.
    for (const char *command : {"M200 D0", "M0200 D1", "M200=0", "\vM200 D0", "\fM00200=bad"}) {
        ++cases;
        const ClassificationFragment head = classify(source(std::string(command) + "\n", relative_start(), {}),
            {}, {false, false, {true}});
        check(head.final.volume_untrusted, "M200 lexical guard latches even outside layer");
        const ClassificationFragment restored = classify(source("G21\nG92 X0\nT0\n", relative_start(), {
            event(UPDATE, 0, {{MM, 0, 1}}), event(RESET, 1, {{X, 0, 0}}), event(UPDATE, 2, {{TOOL, 0, 0}})}),
            {}, {false, false, {true}}, head.final);
        check(restored.final.volume_untrusted, "units reset tool and layer cannot clear volume latch");
        const NominalFragment nominal = read_nominal_fragment(print, relative_prior(), {'A', 'B'});
        bool failed = false;
        try { (void)classify_fragment(print, nominal, {}, active, restored.final); }
        catch (const ClassificationError &error) { failed = error.cause == ClassificationCause::UnsupportedVolume; }
        check(failed, "later active print refuses latched M200");
    }
    for (const char *command : {"m200 D0", "M200x", "M2000", "M+200", ";M200 D0", "G1 ;M200"}) {
        ++cases;
        check(!classify(source(std::string(command) + "\n", relative_start(), {})).final.volume_untrusted,
              "literal complete uppercase M200 only");
    }
    ++cases;
    const SourceFragment known_volume = source(";state\n", seed({{VOL, 0, 1}}), {});
    const ClassificationFragment volume = classify_fragment(known_volume,
        read_nominal_fragment(known_volume, {}, {'A', 'B'}), {}, {false, false, {}}, {});
    check(volume.final.volume_untrusted, "explicit known Reader volume true is latched without default assumption");
    ++cases;
    bool known_refused = false;
    const NominalFragment pn = read_nominal_fragment(print, relative_prior(), {'A', 'B'});
    try { (void)classify_fragment(print, pn, {}, active, volume.final); }
    catch (const ClassificationError &error) { known_refused = error.cause == ClassificationCause::UnsupportedVolume; }
    check(known_refused, "known-true Reader state refuses a later active print");
    ++cases;
    // Source-only lexical postamble: no Reader barrier is represented here.
    // The representative UNKNOWN_COMMAND postamble is tested separately.
    const SourceFragment postamble = source("G1 X1 E.1\nM200 D0\n", relative_start(), {
        event(INVALID, 0, {{E, 0, 0}}), event(MOVE, 0, {{X, 0, 1}})});
    const ClassificationFragment post = classify(postamble);
    expect(post, {false, false}, {0, -1});
    check(post.final.volume_untrusted, "postamble M200 without future active print does not refuse");
    ++cases;
    const SourceFragment updates = source(";true\n;false\n;unknown\n", relative_start(), {
        event(UPDATE, 0, {{VOL, 0, 1}}), event(UPDATE, 1, {{VOL, 0, 0}}), event(INVALID, 2, {{VOL, 0, 0}})});
    const ClassificationFragment persisted = classify(updates);
    check(persisted.final.volume_untrusted, "false and unknown cannot clear known-true latch");
    expect(persisted, {false, false, true}, {-1, -1, -1});

    ++cases;
    // Explicit restoration after an unknown: a real nominal read must regain all
    // coordinate keys and tool/modes. Generic unknown does not poison volume.
    const SourceFragment repair = source("OPAQUE\nG71\nG90\nM83\nT0\nG1 X0 Y0 Z0 F600\nG1 X1 E.1\n",
        relative_start(), {event(DIAGNOSTIC, 0, {{DIAG, 0, RAW_GCODE_READER_DIAGNOSTIC_UNKNOWN_COMMAND}}),
        event(INVALID, 0, {{X, 0, 0}, {Y, 0, 0}, {Z, 0, 0}, {F, 0, 0},
            {XYZ, 0, 0}, {EA, 0, 0}, {MM, 0, 0}, {TOOL, 0, 0}}),
        event(UPDATE, 1, {{MM, 0, 1}}), event(UPDATE, 2, {{XYZ, 0, 1}}),
        event(UPDATE, 3, {{EA, 0, 0}}), event(UPDATE, 4, {{TOOL, 0, 0}}),
        event(MOVE, 5, {{X, 0, 0}, {Y, 0, 0}, {Z, 0, 0}, {F, 0, 600}}),
        event(INVALID, 6, {{E, 0, 0}}), event(MOVE, 6, {{X, 0, 1}})});
    const ClassificationFragment repaired = classify(repair);
    expect(repaired, {true, false, false, false, false, true, false}, {-1, -1, -1, -1, -1, -1, 1});
    check(!repaired.final.volume_untrusted, "generic unknown permits restored printing");
}

void test_volume_reader()
{
    ++cases;
    // publish_unknown invalidates exactly the known initial portable items,
    // excluding LINE_NUMBER and READER_ID; event() supplies the normal cursor.
    const SourceFragment head = source("M200 D0\n", relative_start(), {
        event(DIAGNOSTIC, 0, {{DIAG, 0, RAW_GCODE_READER_DIAGNOSTIC_UNKNOWN_COMMAND}}),
        event(INVALID, 0, {{X, 0, 0}, {Y, 0, 0}, {Z, 0, 0}, {F, 0, 600},
            {MM, 0, 1}, {XYZ, 0, 1}, {EA, 0, 0}, {TOOL, 0, 0}})});
    const NominalFragment hn = read_nominal_fragment(head, relative_prior(), {'A', 'B'});
    const ClassificationFragment hc = classify_fragment(head, hn, {}, {false, false, {true}}, cycle());
    expect(hc, {true}, {-1});
    check(hc.final.volume_untrusted && !hc.final.cycle && hn.final.e.empty(),
          "real M200 D0 preamble closes and latches outside layers without inventing E");
    check(!head.reader.final.active_tool() && !hn.final.xyz[0] && !hn.final.feedrate,
          "unknown-command event invalidates actual coordinate, feed and tool proofs");

    ++cases;
    const SourceFragment postamble = source("G1 X1 E.1\nM200 D0\n", relative_start(), {
        event(INVALID, 0, {{E, 0, 0}}), event(MOVE, 0, {{X, 0, 1}}),
        event(DIAGNOSTIC, 1, {{DIAG, 0, RAW_GCODE_READER_DIAGNOSTIC_UNKNOWN_COMMAND}}),
        event(INVALID, 1, {{X, 0, 1}, {Y, 0, 0}, {Z, 0, 0}, {F, 0, 600},
            {MM, 0, 1}, {XYZ, 0, 1}, {EA, 0, 0}, {TOOL, 0, 0}})});
    const ClassificationFragment post = classify(postamble);
    expect(post, {false, true}, {0, -1});
    check(post.final.volume_untrusted && !post.final.cycle,
          "Reader M200 postamble is a barrier but does not reject without future printing");

    ++cases;
    const SourceFragment immediate = source("G1 X1 E.1\n", head.reader.final, {
        event(MOVE, 0, {{X, 0, 1}, {E, 0, double(float(.1))}})});
    const NominalFragment in = read_nominal_fragment(immediate, hn.final, {'A', 'B'});
    bool failed = false;
    try { (void)classify_fragment(immediate, in, {}, active, hc.final); }
    catch (const ClassificationError &error) {
        failed = error.line_index == 0 && !error.tool && error.cause == ClassificationCause::UnknownTool;
    }
    check(failed, "immediate standard print after M200 fails exact UnknownTool before volume");

    ++cases;
    const SourceFragment repair = source("G71\nG90\nM83\nT0\nG1 X0 Y0 Z0 F600\nG92 X0\nG1 X1 E.1\n",
        head.reader.final, {event(UPDATE, 0, {{MM, 0, 1}}), event(UPDATE, 1, {{XYZ, 0, 1}}),
        event(UPDATE, 2, {{EA, 0, 0}}), event(UPDATE, 3, {{TOOL, 0, 0}}),
        event(MOVE, 4, {{X, 0, 0}, {Y, 0, 0}, {Z, 0, 0}, {F, 0, 600}}),
        event(RESET, 5, {{X, 0, 0}}), event(INVALID, 6, {{E, 0, 0}}), event(MOVE, 6, {{X, 0, 1}})});
    const NominalFragment rn = read_nominal_fragment(repair, hn.final, {'A', 'B'});
    check(rn.motions.size() == 2, "repaired candidate motion count before indexing");
    check(rn.motions[1].active_tool == 0 && rn.motions[1].xyz_absolute == true &&
          rn.motions[1].e_absolute == false && rn.motions[1].units_mm == true &&
          rn.motions[1].e_delta == .1 && !rn.motions[1].e_start && !rn.motions[1].e_end,
          "repair proves exact relative candidate without an external E origin");
    failed = false;
    try { (void)classify_fragment(repair, rn, {}, active, hc.final); }
    catch (const ClassificationError &error) {
        failed = error.line_index == 6 && error.tool == 0 &&
                 error.cause == ClassificationCause::UnsupportedVolume;
    }
    check(failed, "repaired active print fails exact UnsupportedVolume at local row six");
    check(hc.final.volume_untrusted && !hc.final.cycle && !hc.final.wipe_scope &&
          !head.reader.final.active_tool() && !hn.final.xyz[0] &&
          repair.reader.final.active_tool() == 0 && rn.final.xyz[0] == 1 && rn.final.e.empty() &&
          active.enabled_tools[0] && !active.enabled_tools[1],
          "volume refusal leaves prior, source, nominal and context proofs unchanged");
}

void test_failures()
{
    const SourceFragment input = source("G1 X1 E.1\n", relative_start(), {
        event(INVALID, 0, {{E, 0, 0}}), event(MOVE, 0, {{X, 0, 1}})});
    const NominalFragment nominal = read_nominal_fragment(input, relative_prior(), {'A', 'B'});
    ++cases;
    bool failed = false;
    try { (void)classify_fragment(input, nominal, {MoveHint::None, MoveHint::None}, active, cycle()); }
    catch (const std::invalid_argument &) { failed = true; }
    check(failed, "hint vector size guard");
    ++cases;
    failed = false;
    const SourceFragment comment = source(";no move\n", relative_start(), {});
    try { (void)classify(comment, {MoveHint::Extrusion}); }
    catch (const std::invalid_argument &) { failed = true; }
    check(failed, "hint requires Reader motion");
    ++cases;
    NominalFragment mismatched = nominal;
    check(mismatched.motions.size() == 1, "manual join perturbation shape");
    mismatched.motions[0].event_index = 0;
    failed = false;
    try { (void)classify_fragment(input, mismatched, {}, active, cycle()); }
    catch (const std::logic_error &) { failed = true; }
    check(failed, "motion cursor guard");
    ++cases;
    mismatched = nominal;
    mismatched.motions.push_back(mismatched.motions.front());
    failed = false;
    try { (void)classify_fragment(input, mismatched, {}, active, cycle()); }
    catch (const std::logic_error &) { failed = true; }
    check(failed, "unconsumed nominal guard");

    // Manual nominal perturbations test boundary proofs, not provider behavior.
    for (int mode = 0; mode < 10; ++mode) {
        ++cases;
        mismatched = nominal;
        NominalMotion &motion = mismatched.motions[0];
        ClassificationCause reason = ClassificationCause::UnknownExtrusion;
        if (mode == 0) { motion.e_absolute.reset(); reason = ClassificationCause::UnknownModes; }
        if (mode == 1) { motion.xyz_absolute = false; reason = ClassificationCause::UnsupportedModes; }
        if (mode == 2) { motion.units_mm = false; reason = ClassificationCause::UnsupportedModes; }
        if (mode == 3) { motion.start[1].reset(); reason = ClassificationCause::UnknownCoordinates; }
        if (mode == 4) { motion.end[2] = std::numeric_limits<double>::infinity(); reason = ClassificationCause::UnknownCoordinates; }
        if (mode == 5) { motion.e_delta = std::numeric_limits<double>::quiet_NaN(); }
        if (mode == 6) { motion.feedrate = 0; reason = ClassificationCause::UnknownFeedrate; }
        if (mode == 7) { motion.feedrate.reset(); reason = ClassificationCause::UnknownFeedrate; }
        if (mode == 8) { motion.e_absolute = true; }
        if (mode == 9) { motion.e_target.reset(); }
        failed = false;
        try { (void)classify_fragment(input, mismatched, {MoveHint::Extrusion}, active, cycle()); }
        catch (const ClassificationError &error) { failed = error.cause == reason && error.line_index == 0; }
        check(failed, "exact proof failure cause");
    }
    ++cases;
    mismatched = nominal;
    mismatched.motions[0].e_target = 1;
    failed = false;
    try { (void)classify_fragment(input, mismatched, {}, active, cycle()); }
    catch (const std::logic_error &) { failed = true; }
    check(failed, "proven E target conflict is input inconsistency");

    ++cases;
    // Source-only lexical M200 between two known motions isolates transactionality.
    // It does not represent the Reader's intervening unknown-command invalidation.
    const SourceFragment later = source("G1 X1 E.1\nM200 D0\nG1 X2 E.2\n", relative_start(), {
        event(INVALID, 0, {{E, 0, 0}}), event(MOVE, 0, {{X, 0, 1}}),
        event(INVALID, 2, {{E, 0, 0}}), event(MOVE, 2, {{X, 0, 2}})});
    const NominalFragment ln = read_nominal_fragment(later, relative_prior(), {'A', 'B'});
    const ClassificationState prior = cycle();
    const std::string original = later.text;
    failed = false;
    try { (void)classify_fragment(later, ln, {}, active, prior); }
    catch (const ClassificationError &error) {
        failed = error.line_index == 2 && error.tool == 0 && error.cause == ClassificationCause::UnsupportedVolume;
        check(std::string(error.what()).find("local row 2") != std::string::npos, "owned error local position");
    }
    check(failed, "failure after earlier accepted print never returns a partial result");
    check(prior.cycle && prior.cycle->tool == 0 && !prior.volume_untrusted && !prior.wipe_scope,
          "caller classification state unchanged after failure");
    check(later.text == original && later.reader.initial.x() == 0 && later.reader.final.x() == 2 &&
          ln.final.xyz[0] == 2 && ln.final.e.empty() && active.enabled_tools.size() == 2 &&
          active.enabled_tools[0] && !active.enabled_tools[1], "source nominal and context unchanged after failure");
    ++cases;
    ClassificationFragment owned = classify(input);
    ClassificationFragment copied = owned;
    owned.lines.clear();
    owned.final.cycle.reset();
    expect(copied, {false}, {0});
    check(copied.final.cycle && copied.final.cycle->tool == 0, "owned indices and snapshot survive independent copy");
}

SourceFragment source(std::string text, Tracker initial, std::vector<ReaderEvent> events)
{
    initial.set_line_number(0);
    Tracker final = initial;
    for (const ReaderEvent &entry : events)
        final.apply({entry.kind, entry.items.data(), std::uint32_t(entry.items.size())});
    final.set_line_number(std::uint32_t(std::count(text.begin(), text.end(), '\n')));
    return index_fragment(std::move(text), {std::move(initial), std::move(events), std::move(final)});
}

ReaderEvent event(std::uint32_t kind, std::uint32_t line, std::vector<Item> items)
{
    items.push_back({RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, double(line)});
    return {kind, std::move(items)};
}

Tracker seed(std::vector<Item> items)
{
    Tracker result;
    const ReaderEvent update = event(UPDATE, 0, std::move(items));
    result.apply({update.kind, update.items.data(), std::uint32_t(update.items.size())});
    return result;
}

Tracker relative_start()
{
    return seed({{X, 0, 0}, {Y, 0, 0}, {Z, 0, 0}, {F, 0, 600},
                 {XYZ, 0, 1}, {EA, 0, 0}, {MM, 0, 1}, {TOOL, 0, 0}});
}

NominalState relative_prior()
{
    NominalState prior;
    prior.xyz = {{0, 0, 0}};
    prior.feedrate = 600;
    return prior;
}

ClassificationState cycle()
{
    return {CycleSnapshot{0, true, false, true}, false, false};
}

ClassificationFragment classify(const SourceFragment &input, const std::vector<MoveHint> &hints,
    const ClassificationContext &context, ClassificationState prior)
{
    const NominalFragment nominal = read_nominal_fragment(input, relative_prior(), {'A', 'B'});
    return classify_fragment(input, nominal, hints, context, std::move(prior));
}

void expect(const ClassificationFragment &result, const std::vector<bool> &close,
            const std::vector<int> &indices)
{
    check(result.lines.size() == close.size() && close.size() == indices.size(), "decision count");
    for (std::size_t row = 0; row < close.size(); ++row) {
        check(result.lines[row].close_before == close[row], "literal close-before oracle");
        check(bool(result.lines[row].motion_index) == (indices[row] >= 0), "literal print presence");
        if (indices[row] >= 0)
            check(*result.lines[row].motion_index == std::size_t(indices[row]), "literal nominal motion index");
    }
}

void reject(const SourceFragment &input, ClassificationCause cause,
    const std::vector<MoveHint> &hints, const ClassificationContext &context)
{
    bool failed = false;
    try { (void)classify(input, hints, context); }
    catch (const ClassificationError &error) {
        failed = error.cause == cause;
        check(error.line_index < input.lines.size(), "owned local error row is in bounds");
    }
    check(failed, "specific classification error required");
}

void check(bool condition, const char *message)
{
    ++checks;
    if (!condition) throw std::runtime_error(message);
}
} // namespace
