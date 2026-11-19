///|/ Released under the GNU Affero General Public License, version 3 or later.

#include "PressureNominal.hpp"
#include "PressureSource.hpp"

#include <algorithm>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <utility>

#include "libslic3r/Api/plugin/cpp/gcode/GCodeLineParser.hpp"
#include "../../nematx_firmware/src/NematXSyntax.hpp"

// Checks nominal doubles against literal endpoints, deltas and original bytes.
// Reader deltas are manually specified, then validated by public SDK apply;
// there is no Reader provider or replacement interpreter. Real SDK parsing and
// NematX translation are used to verify the invalid-number assumptions. Checks
// throw in every configuration before any dependent indexing or dereference.
//
// Call tree:
//   main: run continuity, modes, lexical mapping, resets, failures and lifetimes.
//     test_continuity / test_modes / test_mapping / test_resets / test_failures
//       source: validate explicit deltas and index the same owned text.
//       event: attach an explicit physical line to manual items.
//       seed: validate initial state items via a public state-update event.
//       one: guard the single motion shape before returning it.
//       token: check scalar provenance against exact original byte slices.
//       check: count assertions, stop immediately on a failed oracle.

// Reports all failures as exit 1, including SDK validation exceptions.
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
    UPDATE = RAW_GCODE_READER_EVENT_STATE_UPDATE, RESET = RAW_GCODE_READER_EVENT_SET_POSITION,
    INVALID = RAW_GCODE_READER_EVENT_INVALIDATE;
int checks = 0, cases = 0;

// Checks persistence across fragments, per-target E, rounding and tiny values.
void test_continuity();
// Checks pre-motion modes, post-line modes and the four E delta cases.
void test_modes();
// Checks direct/translated selection, ordinal mapping and lexical stop bytes.
void test_mapping();
// Checks item-selected G28 zeros, G92 doubles and unscaled bracket resets.
void test_resets();
// Checks SDK-invalid inputs, strict key rejection and immutable owned arguments.
void test_failures();
// Builds one manual fragment by applying public SDK validation, never the DUT.
SourceFragment source(std::string text, Tracker initial, std::vector<ReaderEvent> events);
// Adds the explicit line number required by public SDK event validation.
ReaderEvent event(std::uint32_t kind, std::uint32_t line, std::vector<Item> items);
// Establishes manual initial items using the public validated update operation.
Tracker seed(std::vector<Item> items);
// Guards vector size before exposing the only expected motion.
const NominalMotion &one(const NominalFragment &result);
// Checks the original letter and numeric byte range after structural guards.
void token(const SourceFragment &input, const NominalMotion &motion, std::size_t axis,
           char letter, const std::string &bytes, std::size_t offset);
// Counts assertions and throws rather than allowing later unsafe indexing.
void check(bool condition, const char *message);
} // namespace

int main()
{
    try {
        test_continuity();
        test_modes();
        test_mapping();
        test_resets();
        test_failures();
    } catch (const std::exception &error) {
        std::printf("FAILED: %s (%d cases, %d checks)\n", error.what(), cases, checks);
        return 1;
    }
    std::printf("pressure nominal: %d cases, %d checks, 0 failures\n", cases, checks);
    return 0;
}

namespace {
void test_continuity()
{
    ++cases;
    const SourceFragment first = source("G1 X12.3456789\nG92 E.123456789\n", Tracker(), {
        event(MOVE, 0, {{X, 0, double(float(12.3456789))}}),
        event(RESET, 1, {{E, 0, double(float(.123456789))}})});
    const NominalFragment a = read_nominal_fragment(first, {}, {'A', 'B'});
    check(a.final.xyz[0] == 12.3456789, "source X precision");
    check(first.reader.final.x() != a.final.xyz[0], "float differs with same keys");
    check(a.final.e.at(0) == .123456789, "source E reset precision");
    const SourceFragment second = source("M83\nG1 Y2 E.031250001\nG1 E.031250001\nG1 E-.01\nG1 Z1e-50\n",
        first.reader.final, {event(UPDATE, 0, {{EA, 0, 0}}),
        event(MOVE, 1, {{Y, 0, 2}, {E, 0, double(float(.154706790))}}),
        event(MOVE, 2, {{E, 0, double(float(.185956791))}}),
        event(MOVE, 3, {{E, 0, double(float(.175956791))}}),
        event(MOVE, 4, {{Z, 0, 0}})});
    const NominalFragment b = read_nominal_fragment(second, a.final, {'A', 'B'});
    check(b.motions.size() == 4, "second fragment motion count");
    for (const NominalMotion &motion : b.motions) {
        check(motion.start[0] == 12.3456789, "omitted X retains exact starting double");
        check(motion.end[0] == 12.3456789, "omitted X retains exact double");
        check(motion.e_absolute == false, "M83 remains proven on every motion");
    }
    check(b.motions[0].e_delta == .031250001, "relative source double");
    check(b.motions[2].e_delta == -.01, "negative E delta");
    check(b.final.e.at(0) == .123456789 + .031250001 + .031250001 - .01, "relative continuity");
    check(b.final.xyz[2] == 1e-50, "float zero retains tiny source double");
    check(a.final.e.at(0) == .123456789, "prior remains unchanged");

    ++cases;
    const SourceFragment tools = source("G1 A.1\nG1 B.2\nT0\nG1 E.3\n", Tracker(), {
        event(UPDATE, 0, {{TOOL, 0, 0}}), event(UPDATE, 0, {{EA, 0, 1}}),
        event(MOVE, 0, {{E, 0, double(float(.1))}}),
        event(UPDATE, 0, {{XYZ, 0, 1}}),
        event(UPDATE, 1, {{TOOL, 0, 1}}), event(MOVE, 1, {{E, 1, double(float(.2))}}),
        event(UPDATE, 2, {{TOOL, 0, 0}}), event(MOVE, 3, {{E, 0, double(float(.3))}})});
    const NominalFragment t = read_nominal_fragment(tools, {}, {'A', 'B'});
    check(t.motions.size() == 3, "tool motion count");
    check(t.motions[0].e_target == 0 && t.motions[1].e_target == 1 && t.motions[2].e_target == 0,
          "A B T targets");
    check(t.final.e.at(0) == .3 && t.final.e.at(1) == .2, "separate tool doubles");
    check(t.motions[0].active_tool == 0 && t.motions[1].active_tool == 1 &&
          t.motions[2].active_tool == 0, "proven active tools follow A B T");
    check(t.motions[0].e_absolute == true && t.motions[1].e_absolute == true &&
          t.motions[2].e_absolute == true, "proven absolute E persists across tools");
    check(!t.motions[0].xyz_absolute && t.motions[1].xyz_absolute == true &&
          t.motions[2].xyz_absolute == true, "XYZ becomes proven only after first E-only move");
    check(t.motions[0].event_index == 2 && t.motions[1].event_index == 5 &&
          t.motions[2].event_index == 7, "tool motions retain literal Reader event indices");
}

void test_modes()
{
    ++cases;
    const Tracker initial = seed({{X, 0, 10}, {Y, 0, 20}, {XYZ, 0, 1}});
    NominalState prior;
    prior.xyz[0] = 10;
    prior.xyz[1] = 20;
    const SourceFragment mixed = source("G1 G91 X1 G90 Y2\nG1 X3\n", initial, {
        event(UPDATE, 0, {{XYZ, 0, 0}}), event(MOVE, 0, {{X, 0, 11}, {Y, 0, 22}}),
        event(UPDATE, 0, {{XYZ, 0, 1}}), event(MOVE, 1, {{X, 0, 3}})});
    const NominalFragment m = read_nominal_fragment(mixed, prior, {'A', 'B'});
    check(m.motions.size() == 2, "mixed motion count");
    check(m.motions[0].start[0] == 10 && m.motions[0].start[1] == 20 &&
          m.motions[1].start[0] == 11 && m.motions[1].start[1] == 22, "mixed motion starting points");
    check(m.motions[0].event_index == 1, "first mixed motion follows its mode event");
    check(m.motions[0].xyz_absolute == false && m.motions[0].end[0] == 11 &&
          m.motions[0].end[1] == 22, "mode at first XYZ applies to all XYZ");
    check(m.motions[1].xyz_absolute == true && m.final.xyz[0] == 3, "post-line mode replay");

    ++cases;
    const SourceFragment rel = source("G1 G90 X2 G91 A.031250001\nG1 X1\n", Tracker(), {
        event(UPDATE, 0, {{TOOL, 0, 0}}), event(UPDATE, 0, {{XYZ, 0, 1}}),
        event(UPDATE, 0, {{EA, 0, 0}}),
        event(INVALID, 0, {{E, 0, 0}}), event(MOVE, 0, {{X, 0, 2}}),
        event(UPDATE, 0, {{XYZ, 0, 0}}), event(MOVE, 1, {{X, 0, 3}})});
    const NominalFragment r = read_nominal_fragment(rel, {}, {'A', 'B'});
    check(r.motions.size() == 2, "relative unknown motion count");
    check(r.motions[0].e_delta == .031250001 && !r.motions[0].e_end &&
          !r.motions[0].e_start && r.final.e.empty(), "relative unknown origin retains delta only");
    check(r.motions[1].xyz_absolute == false && r.final.xyz[0] == 3, "mixed final mode persists");
    check(r.motions[0].e_absolute == false && r.motions[1].e_absolute == false,
          "relative E mode is proven across the line boundary");
    check(r.motions[0].event_index == 4 && r.motions[1].event_index == 6,
          "relative motions retain literal Reader event indices");
    check(r.motions[1].active_tool == 0 && r.motions[1].e_target == 0 &&
          r.motions[1].e_delta == 0 && !r.motions[1].e_start && !r.motions[1].e_end,
          "absent E proves zero for the known tool without inventing its origin");
    token(rel, r.motions[0], 3, 'A', ".031250001", rel.text.find(".031250001"));

    ++cases;
    const SourceFragment unknown = source("G1 X2 E.3\nG1 Y4\n", Tracker(), {
        event(MOVE, 0, {{X, 0, 2}, {E, 0, double(float(.3))}}), event(MOVE, 1, {{Y, 0, 4}})});
    const NominalFragment u = read_nominal_fragment(unknown, {}, {'A', 'B'});
    check(u.motions.size() == 2, "unknown flags count");
    check(!u.motions[0].xyz_absolute && !u.motions[0].e_absolute && !u.motions[0].units_mm &&
          !u.motions[0].active_tool && u.motions[0].e_target == 0, "no defaults presented as proof");
    check(u.motions[0].e_end == .3 && !u.motions[0].e_delta, "absolute unknown E origin");
    check(u.motions[1].e_delta == 0 && !u.motions[1].e_target && !u.motions[1].e_end,
          "absent E proves zero without invented tool");

    ++cases;
    const SourceFragment inch = source("G20\nG1 X2 E.1 F3\nG21\nG2 Y4\nG70\nG1 X2\nG71\nG1 Z2\n", Tracker(), {
        event(UPDATE, 0, {{MM, 0, 0}}),
        event(MOVE, 1, {{X, 0, 50.8}, {E, 0, double(float(.1) * 25.4)}, {F, 0, 76.2}}),
        event(UPDATE, 2, {{MM, 0, 1}}),
        event(RAW_GCODE_READER_EVENT_ARC_MOVE, 3, {{Y, 0, 4}, {RAW_GCODE_READER_ITEM_ARC_CLOCKWISE, 0, 1}}),
        event(UPDATE, 4, {{MM, 0, 0}}), event(MOVE, 5, {{X, 0, 50.8}}),
        event(UPDATE, 6, {{MM, 0, 1}}), event(MOVE, 7, {{Z, 0, 2}})});
    const NominalFragment i = read_nominal_fragment(inch, {}, {'A', 'B'});
    check(i.motions.size() == 4, "arc and unit motion count");
    check(i.motions[0].units_mm == false && !i.motions[0].xyz_absolute && i.motions[0].end[0] == 50.8,
          "G20 proves inches without proving XYZ mode, and X uses the absolute fallback");
    check(i.motions[0].e_end == .1 * 25.4 && i.motions[0].feedrate == 3 * 25.4,
          "inch E and feed scaling");
    check(i.motions[1].end[1] == 4 && i.final.xyz[0] == 50.8 && i.final.xyz[2] == 2,
          "arc endpoints and G70 G71 scale");
    for (const NominalMotion &motion : i.motions)
        check(motion.feedrate == 3 * 25.4, "modal feed persists on motions without F");
    check(i.final.feedrate == 3 * 25.4, "modal feed survives unit changes without rescaling");
}

void test_mapping()
{
    struct Sample { std::string text; double x, e; char xletter, eletter; std::string xb, eb;
                    std::vector<char> axes; };
    const std::vector<Sample> samples = {
        {"G1 x1 X2", 2, 0, 'X', 'E', "2", "", {'A', 'B'}},
        {"G1 G90 x1 X2", 1, 0, 'x', 'E', "1", "", {'A', 'B'}},
        {"G1 x1 X2 A.1", 1, .1, 'x', 'A', "1", ".1", {'A', 'B'}},
        {"G1 X2 x1 A.1", 2, .1, 'X', 'A', "2", ".1", {'A', 'B'}},
        {"G1 E.3 A.1", 0, .3, 'X', 'E', "", ".3", {'A', 'B'}},
        {"G1 A.1 E.3", 0, .1, 'X', 'A', "", ".1", {'A', 'B'}},
        {"G1 x1 A.1", 1, .1, 'x', 'A', "1", ".1", {'A', 'B'}},
        {"G1 x1 C.1", 1, .1, 'x', 'C', "1", ".1", {'C', 'D', 'E'}},
        {"G1 x1 D.1", 1, .1, 'x', 'D', "1", ".1", {'C', 'D', 'E'}},
        {"G1 x1 E.1", 1, .1, 'x', 'E', "1", ".1", {'C', 'D', 'E'}},
        {"G01 X2 A.1", 2, 0, 'X', 'E', "2", "", {'A', 'B'}},
        {" \tN10  G1 x1 A.1 ; hi\r\n", 1, .1, 'x', 'A', "1", ".1", {'A', 'B'}},
        {"G1 X2\r A.1 G90", 2, 0, 'X', 'E', "2", "", {'A', 'B'}},
        {std::string("G1 X2\0 A.1 G90", 15), 2, 0, 'X', 'E', "2", "", {'A', 'B'}}};
    for (const Sample &sample : samples) {
        ++cases;
        // These reduced deltas isolate numeric selection and source ranges;
        // they do not claim complete Reader mode/tool events or prove flags.
        std::vector<Item> items;
        if (!sample.xb.empty()) items.push_back({X, 0, double(float(sample.x))});
        const std::uint32_t target = sample.eletter == 'D' ? 1 : sample.eletter == 'E' &&
            sample.axes.size() == 3 ? 2 : 0;
        if (!sample.eb.empty()) items.push_back({E, target, double(float(sample.e))});
        const SourceFragment s = source(sample.text, Tracker(), {event(MOVE, 0, items)});
        const NominalFragment result = read_nominal_fragment(s, {}, sample.axes);
        const NominalMotion &m = one(result);
        if (!sample.xb.empty()) {
            check(m.end[0] == sample.x, "selected X endpoint");
            const std::size_t p = sample.text.find(std::string(1, sample.xletter) + sample.xb);
            check(p != std::string::npos, "literal X source exists");
            token(s, m, 0, sample.xletter, sample.xb, p + 1);
        }
        if (!sample.eb.empty()) {
            check(m.e_end == sample.e && m.e_target == target, "selected E endpoint target");
            const std::size_t p = sample.text.find(std::string(1, sample.eletter) + sample.eb);
            check(p != std::string::npos, "literal E source exists");
            token(s, m, 3, sample.eletter, sample.eb, p + 1);
        } else check(m.e_delta == 0, "direct A not forcibly aliased");
    }
    // The SDK command word retains VT/FF, but NematX translation runs before
    // numeric-command rejection and separates these bytes with isspace.
    ++cases;
    const SourceFragment vertical = source("G1\vX1\fA.1", Tracker(), {
        event(UPDATE, 0, {{TOOL, 0, 0}}), event(UPDATE, 0, {{XYZ, 0, 1}}),
        event(UPDATE, 0, {{EA, 0, 1}}), event(MOVE, 0, {{X, 0, 1}, {E, 0, double(float(.1))}})});
    const NominalFragment v = read_nominal_fragment(vertical, {}, {'A', 'B'});
    const NominalMotion &vm = one(v);
    check(vm.end[0] == 1 && vm.e_end == .1 && vm.e_target == 0, "VT after command reaches translated geometry");
    check(vm.xyz_absolute == true && vm.e_absolute == true && vm.active_tool == 0 && vm.event_index == 3,
          "VT geometry sees ordered pre-motion flags and event index");
    token(vertical, vm, 0, 'X', "1", 4);
    token(vertical, vm, 3, 'A', ".1", 7);

    // A G4 word inside geometry is copied, not dropped as a separate command;
    // the selected Y and E must retain their ordinals beyond that word.
    ++cases;
    const SourceFragment embedded = source("G1 X1\vG4\fY2 A.1", Tracker(), {
        event(UPDATE, 0, {{TOOL, 0, 0}}), event(UPDATE, 0, {{XYZ, 0, 1}}),
        event(UPDATE, 0, {{EA, 0, 1}}),
        event(MOVE, 0, {{X, 0, 1}, {Y, 0, 2}, {E, 0, double(float(.1))}})});
    const NominalFragment g = read_nominal_fragment(embedded, {}, {'A', 'B'});
    const NominalMotion &gm = one(g);
    check(gm.end[0] == 1 && gm.end[1] == 2 && gm.e_end == .1 && gm.e_target == 0,
          "embedded G4 retains all translated endpoints");
    check(gm.xyz_absolute == true && gm.e_absolute == true && gm.active_tool == 0 && gm.event_index == 3,
          "embedded G4 geometry sees ordered pre-motion flags and event index");
    token(embedded, gm, 0, 'X', "1", 4);
    token(embedded, gm, 1, 'Y', "2", 10);
    token(embedded, gm, 3, 'A', ".1", 13);

    ++cases;
    const SourceFragment bare = source("N10\r\n", Tracker(), {});
    check(read_nominal_fragment(bare, {}, {'A', 'B'}).motions.empty(), "N10 CR does not invent geometry");
}

void test_resets()
{
    for (const std::string &text : {"G28", "G028", "G28 X999", "G28 Xabc"}) {
        ++cases;
        const bool all = text == "G28" || text == "G028";
        const SourceFragment s = source(text, Tracker(), {event(RESET, 0, all ?
            std::vector<Item>{{X, 0, 0}, {Y, 0, 0}, {Z, 0, 0}} : std::vector<Item>{{X, 0, 0}})});
        const NominalFragment r = read_nominal_fragment(s, {}, {'A', 'B'});
        check(r.motions.empty() && r.final.xyz[0] == 0, "G28 item-selected exact zero");
        check(r.final.xyz[1].has_value() == all && r.final.xyz[2].has_value() == all, "G28 selected axes");
    }
    for (const std::string &command : {"G92", "G092"}) {
        ++cases;
        const SourceFragment s = source(command + " X.123456789 E.031250001", seed({{MM, 0, 0}}), {
            event(RESET, 0, {{X, 0, double(float(.123456789) * 25.4)},
                              {E, 0, double(float(.031250001) * 25.4)}})});
        const NominalFragment r = read_nominal_fragment(s, {}, {'A', 'B'});
        check(r.final.xyz[0] == .123456789 * 25.4 && r.final.e.at(0) == .031250001 * 25.4,
              "G92 scale independent of motion modes");
    }
    ++cases;
    const SourceFragment bracket = source("A[SET_POSITION POS=.123456789]\nG1 X2 E.2 F3\n",
        seed({{MM, 0, 0}}), {event(RESET, 0, {{E, 0, .123456789}}),
        event(MOVE, 1, {{X, 0, 50.8}, {E, 0, double(float(.2) * 25.4)}, {F, 0, 76.2}})});
    const NominalFragment b = read_nominal_fragment(bracket, {}, {'A', 'B'});
    const NominalMotion &m = one(b);
    check(m.e_start == .123456789 && m.e_end == .2 * 25.4, "bracket double is unscaled under G20");
    check(b.final.xyz[0] == 2 * 25.4 && b.final.feedrate == 3 * 25.4, "normal XYZ F scale under G20");
}

void test_failures()
{
    for (const std::string &word : {"X+1", "X1e39", "A+0.1"}) {
        ++cases;
        const std::string text = "G1 " + word;
        std::string effective = text;
        const std::optional<std::vector<nematx_firmware::ReaderAction>> actions =
            nematx_firmware::translate_line(text, {'A', 'B'}, {});
        if (actions)
            for (const nematx_firmware::ReaderAction &action : *actions)
                if (action.kind == nematx_firmware::ReaderAction::Kind::Standard)
                    effective = action.text;
        const char letter = word.front() == 'A' ? 'E' : 'X';
        Parser parser;
        parser.parse_line(effective, [&](Parser &, const Parser::GCodeLine &line) {
            float value = 123;
            check(line.has_parameter(letter) && !line.parameter_value(letter, value),
                  "public SDK must reject the effective numeric word");
        });
        const std::uint32_t type = letter == 'E' ? E : X;
        NominalState prior;
        prior.xyz[0] = 4;
        prior.e[0] = .25;
        std::vector<ReaderEvent> events;
        if (word.front() == 'A') {
            events.push_back(event(UPDATE, 0, {{TOOL, 0, 0}}));
            events.push_back(event(UPDATE, 0, {{EA, 0, 1}}));
        }
        events.push_back(event(INVALID, 0, {{type, 0, 0}}));
        events.push_back(event(MOVE, 0, {}));
        if (word.front() == 'A')
            events.push_back(event(UPDATE, 0, {{XYZ, 0, 1}}));
        const SourceFragment s = source(text, seed({{X, 0, 4}, {E, 0, .25}}), std::move(events));
        const NominalFragment r = read_nominal_fragment(s, prior, {'A', 'B'});
        const NominalMotion &m = one(r);
        check(!m.tokens[letter == 'E' ? 3 : 0], "invalid numeric token not exposed");
        if (letter == 'E') {
            check(!m.e_delta && !m.e_end && r.final.e.empty(), "invalid E is not zero");
            check(m.active_tool == 0 && m.e_absolute == true && !m.xyz_absolute && m.event_index == 3,
                  "invalid alias E sees tool and E mode before the post-motion XYZ update");
        }
        else check(!r.final.xyz[0] && m.e_delta == 0, "invalid X only erases listed key");
        check(prior.xyz[0] == 4 && prior.e.at(0) == .25, "copied prior immutable");
    }
    ++cases;
    NominalState prior;
    prior.xyz[0] = 4.123456789;
    prior.feedrate = 123.456789;
    const Tracker initial = seed({{X, 0, 4}, {F, 0, 123}});
    const SourceFragment omissions = source("G92 X+1\nG92 X1e39\nG1 Fbad\n", initial, {
        event(RESET, 0, {}), event(RESET, 1, {}), event(MOVE, 2, {})});
    const NominalFragment keep = read_nominal_fragment(omissions, prior, {'A', 'B'});
    check(keep.final.xyz[0] == prior.xyz[0] && keep.final.feedrate == prior.feedrate,
          "invalid reset X and feed omit rather than invalidate");

    // Each boundary mismatch has a valid public Reader state but disagrees only
    // in coordinate keys. Values are intentionally not used as a key oracle.
    for (std::uint32_t type : {X, Y, Z, E, F}) {
        for (bool entry : {true, false}) {
            ++cases;
            SourceFragment s = source("", Tracker(), {});
            const Tracker changed = seed({{type, type == E ? 7u : 0u, 1}});
            if (entry) s.reader.initial = changed;
            else s.reader.final = changed;
            bool rejected = false;
            try { (void)read_nominal_fragment(s, {}, {'A', 'B'}); }
            catch (const std::invalid_argument &) { rejected = true; }
            check(rejected, "strict boundary key mismatch");
        }
    }
    ++cases;
    SourceFragment targets = source("", seed({{E, 3, .1}}), {});
    NominalState wrong;
    wrong.e[4] = .1;
    bool rejected = false;
    try { (void)read_nominal_fragment(targets, wrong, {'A', 'B'}); }
    catch (const std::invalid_argument &) { rejected = true; }
    check(rejected && wrong.e.count(4) == 1, "E target keys not just counts");

    ++cases;
    SourceFragment independent = source("G1 X2", Tracker(), {event(MOVE, 0, {{X, 0, 2}})});
    independent.reader.final.set_reader_id(7);
    independent.reader.final.set_custom_value(RAW_GCODE_READER_ITEM_CUSTOM_BEGIN + 3, 4, 99);
    const std::string original = independent.text;
    const NominalFragment owned = read_nominal_fragment(independent, {}, {'A', 'B'});
    check(independent.text == original && independent.reader.initial.x() == std::nullopt &&
          independent.reader.final.x() == 2, "source and Reader remain immutable");
    independent.text.assign("destroyed source");
    const NominalMotion &m = one(owned);
    check(m.end[0] == 2 && m.tokens[0] && m.tokens[0]->offset == 4 &&
          m.tokens[0]->length == 1 && m.tokens[0]->letter == 'X', "owned values and three scalar range survive");

    ++cases;
    SourceFragment inconsistent = source("G1 X2", initial, {event(MOVE, 0, {{Y, 0, 2}})});
    rejected = false;
    try { (void)read_nominal_fragment(inconsistent, prior, {'A', 'B'}); }
    catch (const std::logic_error &) { rejected = true; }
    check(rejected && prior.xyz[0] == 4.123456789, "unproven numeric source fails without mutating prior");

    ++cases;
    const SourceFragment unknown = source("OPAQUE\n", initial, {
        event(RAW_GCODE_READER_EVENT_DIAGNOSTIC, 0,
              {{RAW_GCODE_READER_ITEM_DIAGNOSTIC_TYPE, 0, RAW_GCODE_READER_DIAGNOSTIC_UNKNOWN_COMMAND}}),
        event(INVALID, 0, {{X, 0, 0}, {F, 0, 0}})});
    const NominalFragment cleared = read_nominal_fragment(unknown, prior, {'A', 'B'});
    check(cleared.motions.empty() && !cleared.final.xyz[0] && !cleared.final.feedrate,
          "unknown invalidation has no fake motion");
}

SourceFragment source(std::string text, Tracker initial, std::vector<ReaderEvent> events)
{
    initial.set_line_number(0);
    Tracker final = initial;
    for (const ReaderEvent &entry : events) {
        const raw_gcode_reader_event raw{entry.kind, entry.items.data(), std::uint32_t(entry.items.size())};
        final.apply(raw);
    }
    final.set_line_number(std::uint32_t(std::count(text.begin(), text.end(), '\n')));
    ReaderFragment reader{std::move(initial), std::move(events), std::move(final)};
    return index_fragment(std::move(text), std::move(reader));
}

ReaderEvent event(std::uint32_t kind, std::uint32_t line, std::vector<Item> items)
{
    items.push_back({RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, double(line)});
    return {kind, std::move(items)};
}

Tracker seed(std::vector<Item> items)
{
    Tracker tracker;
    const ReaderEvent entry = event(UPDATE, 0, std::move(items));
    const raw_gcode_reader_event raw{entry.kind, entry.items.data(), std::uint32_t(entry.items.size())};
    tracker.apply(raw);
    return tracker;
}

const NominalMotion &one(const NominalFragment &result)
{
    check(result.motions.size() == 1, "expected one motion");
    return result.motions.front();
}

void token(const SourceFragment &input, const NominalMotion &motion, std::size_t axis,
           char letter, const std::string &bytes, std::size_t offset)
{
    check(axis < motion.tokens.size(), "token axis bound");
    check(motion.tokens[axis].has_value(), "selected token exists");
    const SourceToken &span = *motion.tokens[axis];
    check(span.offset <= input.text.size() && span.length <= input.text.size() - span.offset, "token byte bounds");
    check(span.offset == offset && span.length == bytes.size() && span.letter == letter, "exact scalar provenance");
    check(input.text.substr(span.offset, span.length) == bytes, "exact numeric bytes");
}

void check(bool condition, const char *message)
{
    ++checks;
    if (!condition) throw std::runtime_error(message);
}

} // namespace
