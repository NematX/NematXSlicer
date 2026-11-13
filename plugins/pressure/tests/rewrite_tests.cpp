///|/ Released under the GNU Affero General Public License, version 3 or later.

#include "PressureClassification.hpp"
#include "PressureExtrusion.hpp"
#include "PressureGeometry.hpp"
#include "PressureNominal.hpp"
#include "PressurePlanner.hpp"
#include "PressureRewrite.hpp"
#include "PressureSource.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <utility>

#include "../../nematx_firmware/src/NematXSyntax.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeLineParser.hpp"

// Exercises the pure rewriter with literal byte/number oracles. Manual Reader
// deltas pass public SDK apply, then real Source, Nominal and Classification
// components prepare eligible inputs. These representative sequences are not a
// provider invocation. Structural counterexamples deliberately test the joins
// and lexical refusal boundary without claiming producer provenance. Real
// normalization, phase solving, spatial projection and extrusion prepare groups.
// Syntax reparses output and the following modeless line; public SDK updates
// retain the mode/tool effects, without a substitute coordinate interpreter.
// Release checks throw and dependent accesses always follow structural guards.
// The optional stdout sample is actual rewriter output, not a saved fixture.
//
// Call tree:
//   main: emit the annotation sample or run all counted cases.
//     test_text / test_modes / test_lexical_paths / test_missing / test_geometry / test_refusals
//     test_output_nominal / test_guards / test_annotations
//     test_residue: remove words and compare modal effects from pre-row state.
//       input: validate manual deltas, index bytes, read nominal and classify.
//       prepare: normalize, solve, project and advance real extrusion results.
//       rewrite: invoke the production entry point with prepared values.
//       output_modes: reparse real Syntax actions and apply only mode/tool items.
//       expect_rejection / check: require specific failures with throwing checks.

// Runs normal checks or prints only actual annotation output for the Python test.
int main(int argc, char **argv);

namespace {
using namespace nematx_pressure;
using Tracker = slic3r_api::GCodeStateTracker;
using Parser = slic3r_api::GCodeGeneration::GCodeLineParser;
using Item = raw_gcode_reader_event_item;
using Action = nematx_firmware::ReaderAction;
constexpr std::uint32_t X = RAW_GCODE_READER_ITEM_X, Y = RAW_GCODE_READER_ITEM_Y,
    Z = RAW_GCODE_READER_ITEM_Z, E = RAW_GCODE_READER_ITEM_E,
    F = RAW_GCODE_READER_ITEM_FEEDRATE, XYZ = RAW_GCODE_READER_ITEM_XYZ_ABSOLUTE,
    EA = RAW_GCODE_READER_ITEM_E_ABSOLUTE, MM = RAW_GCODE_READER_ITEM_UNITS_MM,
    TOOL = RAW_GCODE_READER_ITEM_ACTIVE_TOOL;
constexpr std::uint32_t MOVE = RAW_GCODE_READER_EVENT_LINEAR_MOVE,
    UPDATE = RAW_GCODE_READER_EVENT_STATE_UPDATE, INVALID = RAW_GCODE_READER_EVENT_INVALIDATE;
int cases = 0, checks = 0;

// Owns all prepared input values so copy/const behavior can be checked directly.
struct Prepared
{
    SourceFragment source;
    NominalMotion motion;
    GeometryResult geometry;
    std::array<ExtrusionResult, 3> extrusion{};
    std::vector<char> axes;
    double k = .1;
    RewriteFormat format{3, 3, false};
};

// Checks exact preservation of first-row text, endings and minimal later rows.
void test_text();
// Checks axes and generic E paths, then reparses the output and a modeless row.
void test_modes();
// Checks translated VT/FF command boundaries and lowercase generic E without
// changing the direct SDK grammar; the next modeless row preserves modes/tool.
void test_lexical_paths();
// Checks producer-proven missing E, zero E and release of a preceding charge.
void test_missing();
// Reads produced rows plus a modeless successor with explicit SDK-valid deltas,
// proving nominal XYZ/E interpretation without implementing a replacement Reader.
void test_output_nominal();
// Checks real triangle, merged, one-group and fully collapsed projections.
void test_geometry();
// Requires contextual lexical refusal for every newly disallowed source shape.
void test_refusals();
// Exercises prepared-input failures without weakening Release checks.
void test_guards();
// Checks analytical annotations on every group and normal comment filtering.
void test_annotations();
// Checks literal residues, shared lexical refusals and lost mode/tool effects,
// including pre-row states that differ from the flags attached to the motion.
void test_residue();
// Applies literal manual initial/motion deltas, then real nominal/classification.
// e_word describes the explicit E event, not a reimplementation of text parsing.
// start seeds both the Reader and nominal XYZ; endpoint supplies the motion end.
Prepared input(std::string text, bool absolute = false, std::vector<char> axes = {'A', 'B'},
    std::uint32_t tool = 0, bool e_word = true, bool final_xyz = true,
    Point endpoint = {10, 0, 0}, double nominal = 1, double origin = 0, Point start = {0, 0, 0});
// Computes groups and extrusion with real mathematical components only in tests.
void prepare(Prepared &p, double speed = 2, double acceleration = 1,
             double entry = 0, double exit = 0, double old_charge = 0);
// Calls the production API with immutable borrowed values.
std::string rewrite(const Prepared &p);
// Applies public SDK validation to one explicit event with a physical row.
ReaderEvent event(std::uint32_t kind, std::vector<Item> items, std::uint32_t row = 0);
// Reuses Syntax and SDK apply for modes/tools, while asserting parsed coordinates
// and E values against literal vectors; it does not decode coordinate semantics.
Tracker output_modes(const std::string &text, const Prepared &p,
    const std::vector<double> &xs, const std::vector<double> &es);
// Requires an exact lexical cause, local row and proven tool with no result.
void expect_rejection(const Prepared &p, RewriteCause cause);
// Emits a small multi-phase sample from the actual production rewriter.
std::string annotation_fixture();
// Counts an assertion and throws before dependent indexing can proceed.
void check(bool condition, const char *message);
} // namespace

int main(int argc, char **argv)
{
    try {
        if (argc == 2 && std::string(argv[1]) == "--annotation-fixture") {
            const std::string bytes = annotation_fixture();
            std::fwrite(bytes.data(), 1, bytes.size(), stdout);
            return 0;
        }
        if (argc != 1)
            throw std::invalid_argument("Unknown pressure rewrite test argument");
        test_text();
        test_modes();
        test_lexical_paths();
        test_missing();
        test_output_nominal();
        test_geometry();
        test_refusals();
        test_guards();
        test_annotations();
        test_residue();
        std::printf("pressure rewrite: %d cases, %d checks, 0 failures\n", cases, checks);
        return 0;
    } catch (const std::exception &error) {
        std::fprintf(stderr, "FAILED: %s (%d cases, %d checks)\n", error.what(), cases, checks);
        return 1;
    }
}

namespace {
void test_text()
{
    const std::vector<std::pair<std::string, std::string>> examples{
        {"N120 G1  X10\tE1 F600 F 900 ;source\n", "N120 G1  X2\tE.4 F600 F 900 ;source\nG1 X8 E.6\nG1 X10 E0\n"},
        {"G01 X 10 E\t1 F 600\r\n", "G01 X 2 E\t.4 F 600\r\nG01 X8 E.6\r\nG01 X10 E0\r\n"},
        {"G0 X1e1 E1e0 F6e2", "G0 X2 E.4 F6e2\nG0 X8 E.6\nG0 X10 E0"},
        {"G1 G90 x10 G91 a1 f600\vF900\f;source\n", "G1 G90 x2 G91 a.4 f600\vF900\f;source\nG1 G90 X8 G91 a.6\nG1 G90 X10 G91 a0\n"}
    };
    for (const auto &sample : examples) {
        ++cases;
        const bool relative_end = sample.first.find("G91") != std::string::npos;
        const Prepared p = input(sample.first, false, {'A', 'B'}, 0, true, !relative_end);
        check(rewrite(p) == sample.second, "literal first-byte/minimal-phase oracle");
    }
    ++cases;
    Prepared long_comment = input("G1 X10 E1 ;" + std::string(2000, 'q') + "\n");
    const std::string result = rewrite(long_comment);
    check(result == "G1 X2 E.4 ;" + std::string(2000, 'q') + "\nG1 X8 E.6\nG1 X10 E0\n", "long comments avoid formatter buffer");
    ++cases;
    Parser parser;
    parser.parse_line("G01 X 1 E\t.1 F 600", [&](Parser &, const Parser::GCodeLine &line) {
        check(line.parameter_value('X') == 1 && line.parameter_value('E') == float(.1) &&
              line.parameter_value('F') == 600, "SDK accepts direct spaced numeric parameters");
    });
    parser.parse_line("G01 X1E.1", [&](Parser &, const Parser::GCodeLine &line) {
        check(!line.parameter_value('X') && !line.parameter_value('E'), "SDK rejects concatenated words, no unsupported claim");
    });
}

void test_modes()
{
    for (char axis : {'A', 'B', 'C', 'D', 'E'}) {
        for (bool absolute : {false, true}) {
            ++cases;
            const std::vector<char> axes = axis == 'B' ? std::vector<char>{'A', 'B'} : std::vector<char>{axis, 'Q'};
            const std::uint32_t tool = axis == 'B' ? 1 : 0;
            const std::string text = std::string("G1 G90 X10 ") + (absolute ? "" : "G91 ") + axis + "1\n";
            const Prepared p = input(text, absolute, axes, tool, true, absolute);
            const std::string result = rewrite(p);
            const Tracker final = output_modes(result, p, {2, 8, 10}, absolute ?
                std::vector<double>{.4, 1, 1} : std::vector<double>{.4, .6, 0});
            check(final.xyz_is_absolute() == absolute && final.e_is_absolute() == absolute &&
                  final.active_tool() == tool, "axis modes/tool persist across all output rows");
            const std::string next = std::string("G1 X11 ") + axis + ".2";
            nematx_firmware::ReaderModes modes{final.xyz_is_absolute(), final.e_is_absolute(), tool};
            const auto actions = nematx_firmware::translate_line(next, axes, modes);
            check(actions && !actions->empty(), "next modeless axis row reparses");
            check(std::none_of(actions->begin(), actions->end(), [](const Action &a) {
                return a.kind == Action::Kind::XyzMode || a.kind == Action::Kind::ExtrusionMode || a.kind == Action::Kind::SelectTool;
            }), "next modeless row has no unintended mode/tool action");
        }
    }
    for (const std::string &command : {std::string("G1"), std::string("G01")}) {
        for (bool absolute : {false, true}) {
            ++cases;
            const Prepared p = input(command + " X10 E1\n", absolute);
            const std::string result = rewrite(p);
            check(result.find("G90") == std::string::npos && result.find("G91") == std::string::npos,
                  "existing generic E direct path never gains inline modes");
            const Tracker final = output_modes(result, p, {2, 8, 10}, absolute ?
                std::vector<double>{.4, 1, 1} : std::vector<double>{.4, .6, 0});
            check(final.xyz_is_absolute() == true && final.e_is_absolute() == absolute, "direct generic E modes retained");
        }
    }
    for (bool absolute : {false, true}) {
        for (bool final_xyz : {false, true}) {
            ++cases;
            const Prepared p = input(std::string("G1 G90 X10 E1") + (final_xyz ? "\n" : " G91\n"), absolute,
                {'A', 'B'}, 0, true, final_xyz);
            const std::string result = rewrite(p);
            check(result == std::string("G1 G90 X2 E.4") + (final_xyz ? "\n" : " G91\n") +
                (final_xyz ? "G1 X8 E" : "G1 G90 X8 E") + (absolute ? "1" : ".6") + (final_xyz ? "\n" : " G91\n") +
                (final_xyz ? "G1 X10 E" : "G1 G90 X10 E") + (absolute ? "1" : "0") + (final_xyz ? "\n" : " G91\n"),
                "translated generic E is not converted to an axis letter");
            const Tracker final = output_modes(result, p, {2, 8, 10}, absolute ?
                std::vector<double>{.4, 1, 1} : std::vector<double>{.4, .6, 0});
            check(final.xyz_is_absolute() == final_xyz && final.e_is_absolute() == absolute, "translated generic E final modes retained");
            nematx_firmware::ReaderModes modes{final.xyz_is_absolute(), final.e_is_absolute(), 0};
            check(!nematx_firmware::translate_line("G1 X11 E.2", p.axes, modes), "next generic E stays on direct SDK path");
        }
    }
}

void test_lexical_paths()
{
    for (bool absolute : {false, true}) {
        ++cases;
        const Prepared p = input(absolute ? "G1\vX10\fA1\n" : "G1\vG90\fX10\vG91\fA1\n",
            absolute, {'A', 'B'}, 0, true, absolute);
        const std::string result = rewrite(p);
        check(result == (absolute ? "G1\vX2\fA.4\nG1 X8 A1\nG1 X10 A1\n" :
            "G1\vG90\fX2\vG91\fA.4\nG1 G90 X8 G91 A.6\nG1 G90 X10 G91 A0\n"),
            "translated command VT/FF survives first row with literal minimal later rows");
        const Tracker final = output_modes(result + "G1 X11 A.2\n", p, {2, 8, 10, 11}, absolute ?
            std::vector<double>{.4, 1, 1, .2} : std::vector<double>{.4, .6, 0, .2});
        check(final.xyz_is_absolute() == absolute && final.e_is_absolute() == absolute && final.active_tool() == 0,
              "VT/FF output and modeless axis successor retain proven modes/tool");
    }
    for (bool absolute : {false, true}) {
        for (bool final_xyz : {false, true}) {
            ++cases;
            const Prepared p = input(std::string("G1 G90 X10 e1") + (final_xyz ? "\n" : " G91\n"),
                absolute, {'A', 'B'}, 0, true, final_xyz);
            const std::string result = rewrite(p);
            check(result == std::string("G1 G90 X2 e.4") + (final_xyz ? "\n" : " G91\n") +
                (final_xyz ? "G1 X8 E" : "G1 G90 X8 E") + (absolute ? "1" : ".6") + (final_xyz ? "\n" : " G91\n") +
                (final_xyz ? "G1 X10 E" : "G1 G90 X10 E") + (absolute ? "1" : "0") + (final_xyz ? "\n" : " G91\n"),
                "lowercase generic E retained only on original translated row");
            const Tracker final = output_modes(result + "G1 X11 E.2\n", p, {2, 8, 10, 11}, absolute ?
                std::vector<double>{.4, 1, 1, .2} : std::vector<double>{.4, .6, 0, .2});
            check(final.xyz_is_absolute() == final_xyz && final.e_is_absolute() == absolute && final.active_tool() == 0,
                  "generic E output and modeless successor retain XYZ/E modes and active tool");
        }
    }
}

void test_missing()
{
    for (bool absolute : {false, true}) {
        for (char axis : {'A', 'C', 'D', 'E'}) {
            ++cases;
            Prepared p = input("G1 X10 F600 ;generated shape\n", absolute, {axis, 'B'}, 0, false, true, {10, 0, 0}, 0);
            check(rewrite(p) == std::string("G1 X2") + (absolute ? " " : " G91 ") + axis + ".2" +
                (absolute ? "" : " G90") + " F600 ;generated shape\nG1 X8\nG1 X10" +
                (absolute ? " " : " G91 ") + axis + (absolute ? "0" : "-.2 G90") + "\n",
                "missing E inserts active resolved axis only for nonzero delta");
        }
    }
    ++cases;
    Prepared p = input("G1 X10\n", false, {'A', 'B'}, 0, false, true, {10, 0, 0}, 0);
    p.k = 0;
    prepare(p);
    check(rewrite(p) == "G1 X2\nG1 X8\nG1 X10\n", "F09-only zero delta adds no E word and no local bypass");
    ++cases;
    p.k = 0;
    prepare(p, 2, 1, 0, 0, .3);
    check(p.extrusion[0].pressure_delta == -.3, "old charge is analytical, not current K times speed change");
    check(rewrite(p) == "G1 X2 G91 A-.3 G90\nG1 X8\nG1 X10\n", "K zero releases preceding real extrusion charge");
    ++cases;
    Prepared rel = input("G1 G90 X10 F600\n", false, {'A', 'B'}, 0, false, true, {10, 0, 0}, 0);
    check(rewrite(rel).find("G1 G90 X2 G91 A.2 G90 F600\n") == 0, "relative insertion precedes original F");
    ++cases;
    Prepared unsupported = input("G01 X10\n", false, {'A', 'B'}, 0, false, true, {10, 0, 0}, 0);
    bool failed = false;
    try { (void)rewrite(unsupported); } catch (const std::invalid_argument &) { failed = true; }
    check(failed, "missing E on G01 is not claimed as a generated shape");
    ++cases;
    const Prepared tool_one = input("G1 X10\n", false, {'A', 'B'}, 1, false, true, {10, 0, 0}, 0);
    check(rewrite(tool_one) == "G1 X2 G91 B.2 G90\nG1 X8\nG1 X10 G91 B-.2 G90\n",
          "missing E selects proven active tool one, never a guessed tool zero");
}

void test_output_nominal()
{
    ++cases;
    const Prepared p = input("G1 G90 X10 E1 G91\n", false, {'A', 'B'}, 0, true, false);
    const std::string bytes = rewrite(p) + "G1 X11 E.2\n";
    // Representative manual NematX deltas: relative generic E has no external
    // origin, and the last modeless XYZ row inherits G91 rather than G90.
    const std::vector<ReaderEvent> events{
        event(INVALID, {{E, 0, 0}}, 0), event(MOVE, {{X, 0, 2}}, 0), event(UPDATE, {{XYZ, 0, 0}}, 0),
        event(UPDATE, {{XYZ, 0, 1}}, 1), event(INVALID, {{E, 0, 0}}, 1),
        event(MOVE, {{X, 0, 8}}, 1), event(UPDATE, {{XYZ, 0, 0}}, 1),
        event(UPDATE, {{XYZ, 0, 1}}, 2), event(INVALID, {{E, 0, 0}}, 2),
        event(MOVE, {{X, 0, 10}}, 2), event(UPDATE, {{XYZ, 0, 0}}, 2),
        event(INVALID, {{E, 0, 0}}, 3), event(MOVE, {{X, 0, 21}}, 3)};
    Tracker final = p.source.reader.initial;
    for (const ReaderEvent &entry : events)
        final.apply({entry.kind, entry.items.data(), std::uint32_t(entry.items.size())});
    final.set_line_number(4);
    const SourceFragment source = index_fragment(bytes, {p.source.reader.initial, events, final});
    NominalState prior;
    prior.xyz = {{0, 0, 0}};
    prior.feedrate = 600;
    const NominalFragment nominal = read_nominal_fragment(source, prior, p.axes);
    check(nominal.motions.size() == 4, "actual produced bytes join all four explicit Reader motions");
    check(nominal.motions[0].e_delta == .4 && nominal.motions[1].e_delta == .6 &&
          nominal.motions[2].e_delta == 0 && nominal.motions[3].e_delta == .2,
          "reparsed real nominal channel retains literal relative E deltas");
    check(nominal.final.xyz[0] == 21 && nominal.final.e.empty() && final.x() == 21 &&
          final.xyz_is_absolute() == false && final.e_is_absolute() == false && final.active_tool() == 0,
          "modeless successor proves XYZ/E modes, active tool and no invented E origin");

    ++cases;
    const Prepared lower = input("G1 G90 X10 e1\n");
    const std::string lower_bytes = rewrite(lower) + "G1 X11 E.2\n";
    // With absolute XYZ after the source, constructed generic E rows are direct
    // SDK lines. Explicit relative-E invalidations retain no external E origin.
    const std::vector<ReaderEvent> lower_events{
        event(INVALID, {{E, 0, 0}}, 0), event(MOVE, {{X, 0, 2}}, 0),
        event(INVALID, {{E, 0, 0}}, 1), event(MOVE, {{X, 0, 8}}, 1),
        event(INVALID, {{E, 0, 0}}, 2), event(MOVE, {{X, 0, 10}}, 2),
        event(INVALID, {{E, 0, 0}}, 3), event(MOVE, {{X, 0, 11}}, 3)};
    Tracker lower_final = lower.source.reader.initial;
    for (const ReaderEvent &entry : lower_events)
        lower_final.apply({entry.kind, entry.items.data(), std::uint32_t(entry.items.size())});
    lower_final.set_line_number(4);
    const SourceFragment lower_source = index_fragment(lower_bytes,
        {lower.source.reader.initial, lower_events, lower_final});
    const NominalFragment lower_nominal = read_nominal_fragment(lower_source, prior, lower.axes);
    check(lower_nominal.motions.size() == 4, "lowercase generic source output joins all explicit Reader motions");
    check(lower_nominal.motions[0].e_delta == .4 && lower_nominal.motions[1].e_delta == .6 &&
          lower_nominal.motions[2].e_delta == 0 && lower_nominal.motions[3].e_delta == .2,
          "all constructed direct rows retain generic E through real nominal decoding");
    check(lower_nominal.final.xyz[0] == 11 && lower_nominal.final.e.empty() &&
          lower_final.xyz_is_absolute() == true && lower_final.e_is_absolute() == false && lower_final.active_tool() == 0,
          "absolute XYZ successor retains relative E without a fabricated origin");
}

void test_geometry()
{
    ++cases;
    const Prepared absolute_origin = input("G1 X10 E3\n", true, {'A', 'B'}, 0, true, true, {10, 0, 0}, 3, 2);
    check(rewrite(absolute_origin) == "G1 X2 E2.4\nG1 X8 E3\nG1 X10 E3\n", "absolute E preserves a proven nonzero external origin");
    ++cases;
    Prepared p = input("G1 X10 E1\n");
    prepare(p, 2, 1, 2, 2);
    check(p.geometry.count == 1, "real steady one-group profile");
    check(rewrite(p) == "G1 X10 E1.2\n", "literal one-group output");
    ++cases;
    prepare(p, 100, 1);
    check(p.geometry.count == 2, "real triangular profile");
    check(rewrite(p) == "G1 X5 E.816\nG1 X10 E.184\n", "triangle literal cuts and E");
    ++cases;
    p = input("G1 X1 E.1\n", false, {'A', 'B'}, 0, true, true, {1, 0, 0}, .1);
    p.format.xyz_digits = 1;
    prepare(p, 1, 100);
    check(p.geometry.count == 1 && p.geometry.groups[0].kind == SegmentKind::Merged, "real coarse spatial fusion");
    check(rewrite(p) == "G1 X1 E.1\n", "merged group has no stationary compensation");
    ++cases;
    p = input("G1 Y10 E1\n", false, {'A', 'B'}, 0, true, true, {0, 10, 0});
    check(rewrite(p) == "G1 Y2 E.4\nG1 Y8 E.6\nG1 Y10 E0\n", "single Y source never gains X/Z");
    ++cases;
    p = input("G1 Z10 E1\n", false, {'A', 'B'}, 0, true, true, {0, 0, 10});
    check(rewrite(p) == "G1 Z2 E.4\nG1 Z8 E.6\nG1 Z10 E0\n", "single Z remains spatial");
    ++cases;
    p = input("G1 X10 E1\n");
    p.motion.start[1] = .0004;
    p.motion.end[1] = .0004;
    prepare(p);
    check(p.geometry.groups[0].start[1] == 0 && p.geometry.groups[0].end[1] == 0,
          "real projection quantizes the unchanged off-grid Y coordinate");
    check(rewrite(p) == "G1 X2 E.4\nG1 X8 E.6\nG1 X10 E0\n",
          "structural off-grid absent Y is never emitted or changed");
    ++cases;
    p = input("G1 X.001 E.001\n", false, {'A', 'B'}, 0, true, true, {.001, 0, 0}, .001);
    p.format.xyz_digits = 0;
    prepare(p);
    check(p.geometry.status == GeometryStatus::EntireSourceCollapsed, "real entire-source collapsed aggregate");
    bool failed = false;
    try { (void)rewrite(p); } catch (const std::invalid_argument &) { failed = true; }
    check(failed, "collapsed input must be resolved outside rewriter, not emitted E-only");
    ++cases;
    const Prepared shifted = input("G1 X14 E1\n", false, {'A', 'B'}, 0, true, true,
                                   {14, 0, 0}, 1, 0, {4, 0, 0});
    check(shifted.source.reader.initial.x() == 4 && shifted.motion.start[0] == 4 && shifted.motion.end[0] == 14,
          "nonzero moving-axis start is shared by manual Reader and real nominal inputs");
    check(shifted.geometry.count == 3, "shifted source has three spatial groups");
    check(shifted.geometry.groups[0].distance == 2 && shifted.geometry.groups[1].distance == 6 &&
          shifted.geometry.groups[2].distance == 2 && shifted.geometry.groups[0].end[0] == 6 &&
          shifted.geometry.groups[1].end[0] == 12 && shifted.geometry.groups[2].end[0] == 14,
          "signed deltas yield literal ten-mm profile and shifted 6/12/14 cuts");
    check(rewrite(shifted) == "G1 X6 E.4\nG1 X12 E.6\nG1 X14 E0\n",
          "nonzero starting X keeps literal pressure output and source endpoint");
}

void test_refusals()
{
    const std::vector<std::pair<std::string, RewriteCause>> samples{
        {"G1 X10 X20 E1\n", RewriteCause::DuplicateParameter},
        {"G1 X10 X10 E1\n", RewriteCause::DuplicateParameter},
        {"G1 G90 x10 X20 G91 A1\n", RewriteCause::DuplicateParameter},
        {"G1 G90 X10 G91 A1 E1\n", RewriteCause::DuplicateParameter},
        {"G1 X10 E1 E1\n", RewriteCause::DuplicateParameter},
        {"G1 G90 X10 G91 Y2 E1\n", RewriteCause::ConflictingXyzModes},
        {"G1 X10 E1 S1\n", RewriteCause::UnsupportedWord},
        {"G1 X10 E1 G4\n", RewriteCause::UnsupportedWord},
        {"G01 X10 E1 G90\n", RewriteCause::UnsupportedWord},
        {"G01 X10 A1 E1\n", RewriteCause::UnsupportedWord},
        {"G1 X10 E1 g90\n", RewriteCause::UnsupportedWord},
        {"G1 G90 X10 E1 g90\n", RewriteCause::UnsupportedWord},
        {"G1 X10 E1\r", RewriteCause::InvalidBytes},
        {"G1 X10 E1\rG4\n", RewriteCause::InvalidBytes},
        {std::string("G1 X10 E1") + char(0) + " S1\n", RewriteCause::InvalidBytes},
        {std::string("G1 X10 E1 ;comment") + char(0) + "hidden\n", RewriteCause::InvalidBytes}
    };
    for (const auto &sample : samples) {
        ++cases;
        // Structural manual events isolate lexical checks; Syntax itself ignores
        // some words, which is precisely why eligible output must reject them.
        const bool y = sample.first.find("Y2") != std::string::npos;
        const bool rel = sample.first.find("G91") != std::string::npos;
        Prepared p = input(sample.first, false, {'A', 'B'}, 0, true, !rel, y ? Point{10, 2, 0} : Point{10, 0, 0});
        expect_rejection(p, sample.second);
    }
}

void test_guards()
{
    const Prepared good = input("G1 X10 E1\n");
    for (int which = 0; which < 22; ++which) {
        ++cases;
        Prepared p = good;
        switch (which) {
        case 0: p.geometry.count = 0; break;
        case 1: p.geometry.count = 4; break;
        case 2: p.format.xyz_digits = -1; break;
        case 3: p.format.e_digits = 10; break;
        case 4: p.motion.tokens[0]->offset = p.source.text.size(); break;
        case 5: p.motion.tokens[3]->length = 9999; break;
        case 6: p.motion.event_index = 999; break;
        case 7: p.source.reader.events[p.motion.event_index].kind = RAW_GCODE_READER_EVENT_ARC_MOVE; break;
        case 8: p.motion.active_tool.reset(); break;
        case 9: p.motion.e_target = 1; break;
        case 10: p.motion.units_mm = false; break;
        case 11: p.motion.e_absolute.reset(); break;
        case 12: p.geometry.groups[1].end[0] = std::numeric_limits<double>::infinity(); break;
        case 13: p.extrusion[2].delta = 0x1p63; break;
        case 14: p.geometry.groups[2].end[0] = 1e300; break;
        case 15: p.source.lines[0].length = 999; break;
        case 16: p.geometry.groups[2].duration = 0; break;
        case 17: p.format.annotate = true; p.geometry.groups[1].speed_start = std::numeric_limits<double>::quiet_NaN(); break;
        case 18: p.format.annotate = true; p.extrusion[2].pressure_delta = std::numeric_limits<double>::infinity(); break;
        case 19: p.k = -1; break;
        case 20: p.source.reader.events[p.motion.event_index].items.back().value = 1; break;
        case 21: p.source.lines[0].first_event = 999; break;
        }
        const std::string original = p.source.text;
        bool failed = false;
        std::string result = "unchanged";
        try { result = rewrite(p); } catch (const std::logic_error &) { failed = true; }
        check(failed && result == "unchanged" && p.source.text == original, "shape/number failure returns no partial result");
    }
    ++cases;
    const Prepared copied = good;
    check(rewrite(copied) == rewrite(good) && good.source.text == "G1 X10 E1\n" &&
          good.motion.end[0] == 10 && good.geometry.groups[0].end[0] == 2 &&
          good.extrusion[0].delta == .4, "copies and immutable prepared inputs survive output");
    for (int digits : {0, 9}) {
        ++cases;
        Prepared p = good;
        p.format = {digits, digits, false};
        check(!rewrite(p).empty(), "both allowed precision endpoints format safely");
    }
    ++cases;
    Prepared large = good;
    large.format.e_digits = 0;
    large.extrusion[0].delta = 1e14;
    check(rewrite(large).find("E100000000000000") != std::string::npos, "existing large E domain is not restricted to arbitrary small coordinates");
    ++cases;
    large.extrusion[0].delta = 0x1p54;
    check(rewrite(large).find("E18014398509481984") != std::string::npos, "scaled int64 domain is not capped at 2^53");
}

void test_annotations()
{
    ++cases;
    const std::string actual = annotation_fixture();
    check(actual ==
        "G1 X2 E.4 ;NX_PRESSURE v=1 tool=0 phase=accel v0_mm_s=0 v1_mm_s=2 e0_mm_s=0 e1_mm_s=0.2 dt_s=2 k_s=0.1 de_nom_mm=0.2 de_pressure_mm=0.2\n"
        "G1 X8 E.6 ;NX_PRESSURE v=1 tool=0 phase=steady v0_mm_s=2 v1_mm_s=2 e0_mm_s=0.20000000000000004 e1_mm_s=0.20000000000000004 dt_s=3 k_s=0.1 de_nom_mm=0.6000000000000001 de_pressure_mm=0\n"
        "G1 X10 E0 ;NX_PRESSURE v=1 tool=0 phase=decel v0_mm_s=2 v1_mm_s=0 e0_mm_s=0.19999999999999996 e1_mm_s=0 dt_s=2 k_s=0.1 de_nom_mm=0.19999999999999996 de_pressure_mm=-0.2\n"
        "G1 X1 E.1 ;NX_PRESSURE v=1 tool=0 phase=merged v0_mm_s=0 v1_mm_s=0 e0_mm_s=0 e1_mm_s=0 dt_s=1.01 k_s=0.1 de_nom_mm=0.1 de_pressure_mm=0\n",
        "literal round-trip annotation schema and analytical values on all groups");
    nematx_firmware::OutputFilter filter(true, false);
    check(filter.process(actual) + filter.flush() == "G1 X2 E.4\nG1 X8 E.6\nG1 X10 E0\nG1 X1 E.1\n", "normal Syntax filter removes annotations without altering correction");
    ++cases;
    Prepared p = input("G1 X1 E.1 ;original\r\n", false, {'A', 'B'}, 0, true, true, {1, 0, 0}, .1);
    p.format = {1, 3, true};
    prepare(p, 1, 100);
    check(rewrite(p).find(";original ;NX_PRESSURE v=1 tool=0 phase=merged") != std::string::npos,
          "original comment once before merged annotation");
    check(rewrite(p).substr(rewrite(p).size() - 2) == "\r\n", "annotation precedes CRLF");
    ++cases;
    p.k = 0;
    prepare(p, 1, 100, 0, 0, .3);
    check(rewrite(p).find("k_s=0 de_nom_mm=0.1 de_pressure_mm=-0.3") != std::string::npos,
          "annotation uses analytical preceding charge release at K zero");
    ++cases;
    p.geometry.groups[0].speed_start = -0.;
    p.extrusion[0].pressure_delta = -0.;
    check(rewrite(p).find("=-0") == std::string::npos, "annotation negative zero is normalized");
}

void test_residue()
{
    const std::vector<std::pair<std::string, std::string>> samples{
        {"G1 X.0004 E.0001\n", ""},
        {"N5 G1 X.0004 E.0001\r\n", ""},
        {"N5 G1 X.0004 E.0001 ;source\r\n", ";source\r\n"},
        {"G01 X .0004 E\t.0001 ;source", ";source"},
        {"G1 X.0004 E.0001 ;source\n", ";source\n"},
        {"N5 G01 X .0004 E\t.0001 F600 F 900 ;source\r\n", "N5 G01   F600 F 900 ;source\r\n"},
        {"G1 X.0004 E.0001 F600", "G1   F600"},
        {"G1 G90 X.0004 G91 A.0001\n", "G1 G90  G91 \n"},
        {"N5 G1 G90 X.0004 E.0001 G91 ;source\n", "N5 G1 G90   G91 ;source\n"},
        {"G1 G90 X.0004 A.0001\n", ""}
    };
    for (const auto &sample : samples) {
        ++cases;
        const bool axis_absolute = sample.first.find(" A") != std::string::npos &&
            sample.first.find("G91") == std::string::npos;
        const bool final_xyz = sample.first.find("G91") == std::string::npos;
        const Prepared p = input(sample.first, axis_absolute, {'A', 'B'}, 0, true,
            final_xyz, {.0004, 0, 0}, .0001);
        check(p.geometry.status == GeometryStatus::EntireSourceCollapsed, "residue source genuinely collapses at XYZ precision");
        const std::string original = p.source.text;
        check(rewrite_residue(p.source, 0, p.motion, p.axes) == sample.second, "literal residue bytes and original ending");
        check(p.source.text == original && p.motion.e_delta == .0001, "successful residue keeps borrowed inputs unchanged");
    }
    for (bool absolute : {false, true}) {
        for (std::uint32_t tool : {0u, 1u}) {
            ++cases;
            const char axis = tool ? 'B' : 'A';
            const Prepared p = input(std::string("G1 G90 X.0004 ") +
                (absolute ? "" : "G91 ") + axis + ".0001 F600\n", absolute,
                {'A', 'B'}, tool, true, absolute, {.0004, 0, 0}, .0001);
            check(rewrite_residue(p.source, 0, p.motion, p.axes) ==
                std::string("G1 G90  ") + (absolute ? "" : "G91 ") + " F600\n",
                "resolved A/B words removed in both E modes without inventing commands");
        }
    }
    for (const std::string &command : {std::string("G1"), std::string("G01")}) {
        for (bool absolute : {false, true}) {
            ++cases;
            const Prepared p = input(command + " X.0004 E.0001\n", absolute,
                {'A', 'B'}, 0, true, true, {.0004, 0, 0}, .0001);
            check(rewrite_residue(p.source, 0, p.motion, p.axes).empty(), "direct generic E supports both extrusion modes");
        }
    }
    ++cases;
    Prepared mode = input("N5 G1 G90 X.0004 E.0001 ;source\n", false,
        {'A', 'B'}, 0, true, true, {.0004, 0, 0}, .0001);
    const ReaderEvent relative_xyz = event(UPDATE, {{XYZ, 0, 0}});
    mode.source.reader.initial.apply({relative_xyz.kind, relative_xyz.items.data(), std::uint32_t(relative_xyz.items.size())});
    const ReaderEvent absolute_xyz = event(UPDATE, {{XYZ, 0, 1}});
    mode.source.reader.events.insert(mode.source.reader.events.begin(), absolute_xyz);
    ++mode.source.lines[0].event_count;
    ++mode.motion.event_index;
    check(rewrite_residue(mode.source, 0, mode.motion, mode.axes) == "N5 G1 G90   ;source\n",
        "useful XYZ change survives even though the motion already has absolute flags");
    for (int which = 0; which < 4; ++which) {
        ++cases;
        Prepared p = input("G1 G90 X.0004 G91 A.0001\n", false,
            {'A', 'B'}, 0, true, false, {.0004, 0, 0}, .0001);
        // Motion flags already reflect inline actions. Change the actual entry
        // state, then supply SDK-valid preceding events where appropriate.
        const ReaderEvent changed = event(UPDATE, {{EA, 0, which == 1 ? 0. : 1.}, {TOOL, 0, which == 1 ? 1. : 0.}});
        if (which < 2) {
            p.source.reader.initial.apply({changed.kind, changed.items.data(), std::uint32_t(changed.items.size())});
            const ReaderEvent inline_modes = event(UPDATE, {{EA, 0, 0}, {TOOL, 0, 0}});
            p.source.reader.events.insert(p.source.reader.events.begin(), inline_modes);
            ++p.source.lines[0].event_count;
            ++p.motion.event_index;
        } else {
            p.source.text.insert(0, ";prior\n");
            for (SourceLine &line : p.source.lines)
                line.offset += 7;
            for (std::optional<SourceToken> &token : p.motion.tokens)
                if (token)
                    token->offset += 7;
            for (ReaderEvent &entry : p.source.reader.events)
                entry.items.back().value = 1;
            p.source.reader.events.insert(p.source.reader.events.begin(), changed);
            ++p.source.lines[0].first_event;
            ++p.motion.event_index;
            p.source.lines.insert(p.source.lines.begin(), {0, 7, 0, 1});
            if (which == 3) {
                const ReaderEvent restore = event(UPDATE, {{EA, 0, 0}});
                p.source.reader.events.insert(p.source.reader.events.begin() + 1, restore);
                ++p.source.lines[0].event_count;
                ++p.source.lines[1].first_event;
                ++p.motion.event_index;
            }
        }
        const std::size_t row = which < 2 ? 0 : 1;
        const std::string original = p.source.text;
        std::string result = "not published";
        bool failed = false;
        try { result = rewrite_residue(p.source, row, p.motion, p.axes); }
        catch (const RewriteError &error) {
            failed = error.cause == RewriteCause::LostModalEffect && error.line_index == row && error.tool == 0;
            check(std::string(error.what()).find("mode or tool effect") != std::string::npos, "lost effect diagnostic is readable");
        }
        if (which == 3)
            check(!failed && result == "G1 G90  G91 \n", "prior events restore relative E before row, not fragment initial");
        else
            check(failed && result == "not published", "true pre-row E/tool state prevents partial residue");
        check(p.source.text == original, "failed modal replay leaves source unchanged");
    }
    const std::vector<std::pair<std::string, RewriteCause>> refusals{
        {"G1 X10 X20 E1\n", RewriteCause::DuplicateParameter},
        {"G1 G90 x10 X20 G91 A1\n", RewriteCause::DuplicateParameter},
        {"G1 G90 X10 G91 A1 E1\n", RewriteCause::DuplicateParameter},
        {"G1 X10 E1 E1\n", RewriteCause::DuplicateParameter},
        {"G1 G90 X10 G91 Y2 E1\n", RewriteCause::ConflictingXyzModes},
        {"G1 X10 E1 S1\n", RewriteCause::UnsupportedWord},
        {"G01 X10 E1 G90\n", RewriteCause::UnsupportedWord},
        {"G1 X10 E1\r", RewriteCause::InvalidBytes},
        {"G1 X10 E1\rG4\n", RewriteCause::InvalidBytes},
        {std::string("G1 X10 E1 ;comment") + char(0) + "hidden\n", RewriteCause::InvalidBytes}
    };
    for (const auto &sample : refusals) {
        ++cases;
        const bool y = sample.first.find("Y2") != std::string::npos;
        const bool rel = sample.first.find("G91") != std::string::npos;
        const Prepared p = input(sample.first, false, {'A', 'B'}, 0, true, !rel, y ? Point{10, 2, 0} : Point{10, 0, 0});
        std::string result = "not published";
        bool failed = false;
        try { result = rewrite_residue(p.source, 0, p.motion, p.axes); }
        catch (const RewriteError &error) {
            failed = error.cause == sample.second && error.line_index == 0 && error.tool == 0;
        }
        check(failed && result == "not published" && p.source.text == sample.first, "residue shares E23 refusals without partial bytes");
    }
    for (int which = 0; which < 8; ++which) {
        ++cases;
        Prepared p = input("G1 X.0004 E.0001\n", false,
            {'A', 'B'}, 0, true, true, {.0004, 0, 0}, .0001);
        switch (which) {
        case 0: p.motion.tokens[0]->offset = p.source.text.size(); break;
        case 1: p.motion.event_index = 999; break;
        case 2: p.motion.units_mm = false; break;
        case 3: p.motion.e_absolute.reset(); break;
        case 4: p.motion.e_delta = -1; break;
        case 5: p.motion.e_delta = std::numeric_limits<double>::infinity(); break;
        case 6: p.source.reader.events[p.motion.event_index].kind = RAW_GCODE_READER_EVENT_ARC_MOVE; break;
        case 7: p.source.lines[0].first_event = 999; break;
        }
        std::string result = "not published";
        bool failed = false;
        try { result = rewrite_residue(p.source, 0, p.motion, p.axes); }
        catch (const std::logic_error &) { failed = true; }
        check(failed && result == "not published", "residue rejects invalid joined inputs transactionally");
    }
}

Prepared input(std::string text, bool absolute, std::vector<char> axes,
    std::uint32_t tool, bool e_word, bool final_xyz, Point endpoint, double nominal, double origin, Point start)
{
    Prepared p;
    p.axes = std::move(axes);
    Tracker initial;
    std::vector<Item> initial_items{{X, 0, start[0]}, {Y, 0, start[1]}, {Z, 0, start[2]}, {F, 0, 600},
        {XYZ, 0, 1}, {EA, 0, absolute ? 1. : 0.}, {MM, 0, 1}, {TOOL, 0, double(tool)}};
    NominalState prior;
    for (std::size_t a = 0; a < 3; ++a)
        prior.xyz[a] = start[a];
    prior.feedrate = 600;
    if (absolute) {
        initial_items.push_back({E, tool, origin});
        prior.e[tool] = origin;
    }
    const ReaderEvent seed = event(UPDATE, std::move(initial_items));
    initial.apply({seed.kind, seed.items.data(), std::uint32_t(seed.items.size())});
    std::vector<Item> coordinates;
    for (std::size_t a = 0; a < 3; ++a)
        if (endpoint[a] != start[a])
            coordinates.push_back({a == 0 ? X : a == 1 ? Y : Z, 0, double(float(endpoint[a]))});
    std::vector<ReaderEvent> events;
    if (e_word) {
        if (absolute)
            coordinates.push_back({E, tool, double(float(nominal))});
        else
            events.push_back(event(INVALID, {{E, tool, 0}}));
    }
    events.push_back(event(MOVE, std::move(coordinates)));
    if (!final_xyz)
        events.push_back(event(UPDATE, {{XYZ, 0, 0}}));
    Tracker final = initial;
    for (const ReaderEvent &entry : events)
        final.apply({entry.kind, entry.items.data(), std::uint32_t(entry.items.size())});
    final.set_line_number(std::uint32_t(std::count(text.begin(), text.end(), '\n')));
    p.source = index_fragment(std::move(text), {std::move(initial), std::move(events), std::move(final)});
    const NominalFragment n = read_nominal_fragment(p.source, prior, p.axes);
    check(n.motions.size() == 1, "manual source joins one real nominal motion");
    const ClassificationFragment c = classify_fragment(p.source, n,
        {e_word ? MoveHint::None : MoveHint::Extrusion}, {true, false, std::vector<bool>(p.axes.size(), true)}, {});
    check(c.lines.size() == 1 && c.lines[0].motion_index == 0, "real classifier proves eligible input");
    p.motion = n.motions[0];
    prepare(p);
    return p;
}

void prepare(Prepared &p, double speed, double acceleration, double entry, double exit, double old_charge)
{
    const double origin = *p.motion.e_absolute ? *p.motion.e_start : 0;
    SourceGeometry source{{*p.motion.start[0], *p.motion.start[1], *p.motion.start[2]},
        {*p.motion.end[0], *p.motion.end[1], *p.motion.end[2]}, origin, origin + *p.motion.e_delta};
    const AxisLimits limits{{0, 0, 0, 0}, {0, 0, 0, 0}, 0};
    const Move move = normalize_move(source.end[0] - source.start[0], source.end[1] - source.start[1],
                                    source.end[2] - source.start[2],
                                    *p.motion.e_delta, speed, acceleration, limits, 0);
    const PlannedMove planned{move, solve_phases({move.distance, move.speed_limit, entry, exit,
                                                move.acceleration, move.acceleration})};
    p.geometry = project_geometry(planned, source, p.format.xyz_digits);
    if (p.geometry.status != GeometryStatus::Representable)
        return;
    Extrusion extrusion(p.format.e_digits, origin);
    if (old_charge != 0)
        (void)extrusion.advance(origin, 1, old_charge);
    for (std::size_t i = 0; i < p.geometry.count; ++i)
        p.extrusion[i] = extrusion.advance(p.geometry.groups[i].nominal_end, p.geometry.groups[i].speed_end, p.k);
}

std::string rewrite(const Prepared &p)
{
    return rewrite_print(p.source, 0, p.motion, p.geometry, p.extrusion, p.k, p.axes, p.format);
}

ReaderEvent event(std::uint32_t kind, std::vector<Item> items, std::uint32_t row)
{
    items.push_back({RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, double(row)});
    return {kind, std::move(items)};
}

Tracker output_modes(const std::string &text, const Prepared &p,
    const std::vector<double> &xs, const std::vector<double> &es)
{
    Tracker state = p.source.reader.initial;
    Parser parser;
    std::size_t row = 0;
    parser.parse_buffer(text, [&](Parser &, const Parser::GCodeLine &line) {
        check(row < xs.size() && row < es.size(), "reparsed output row is in literal oracle bounds");
        nematx_firmware::ReaderModes modes{state.xyz_is_absolute(), state.e_is_absolute(), state.active_tool()};
        const auto actions = nematx_firmware::translate_line(line.raw(), p.axes, modes);
        std::string standard = line.raw();
        if (actions) {
            for (const Action &action : *actions) {
                if (action.kind == Action::Kind::Standard) {
                    standard = action.text;
                    continue;
                }
                std::vector<Item> items;
                if (action.kind == Action::Kind::XyzMode)
                    items.push_back({XYZ, 0, action.absolute ? 1. : 0.});
                else if (action.kind == Action::Kind::ExtrusionMode)
                    items.push_back({EA, 0, action.absolute ? 1. : 0.});
                else if (action.kind == Action::Kind::SelectTool)
                    items.push_back({TOOL, 0, double(action.tool)});
                else
                    check(false, "output Syntax contains only linear and mode/tool actions");
                const ReaderEvent update = event(UPDATE, std::move(items));
                state.apply({update.kind, update.items.data(), std::uint32_t(update.items.size())});
            }
        }
        parser.parse_line(standard, [&](Parser &, const Parser::GCodeLine &parsed) {
            check(parsed.parameter_value('X') == float(xs[row]), "reparsed X equals literal group endpoint");
            check(parsed.parameter_value('E') == float(es[row]), "reparsed E equals literal position/delta");
        });
        ++row;
    });
    check(row == xs.size() && row == es.size(), "all literal groups reparsed exactly once");
    return state;
}

void expect_rejection(const Prepared &p, RewriteCause cause)
{
    bool failed = false;
    std::string result = "not published";
    try { result = rewrite(p); }
    catch (const RewriteError &error) {
        failed = error.cause == cause && error.line_index == 0 && error.tool == *p.motion.active_tool;
        check(std::string(error.what()).find("local row 0, tool 0") != std::string::npos, "contextual local error text");
    }
    check(failed && result == "not published", "specific lexical refusal without partial output");
}

std::string annotation_fixture()
{
    Prepared p = input("G1 X10 E1\n");
    p.format.annotate = true;
    std::string bytes = rewrite(p);
    p = input("G1 X1 E.1\n", false, {'A', 'B'}, 0, true, true, {1, 0, 0}, .1);
    p.format = {1, 3, true};
    prepare(p, 1, 100);
    bytes += rewrite(p);
    return bytes;
}

void check(bool condition, const char *message)
{
    ++checks;
    if (!condition)
        throw std::runtime_error(message);
}
} // namespace
