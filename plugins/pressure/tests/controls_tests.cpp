///|/ Released under the GNU Affero General Public License, version 3 or later.

#include "PressureControls.hpp"

#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

// Checks pure control recognition, full-field doubles and independent state
// snapshots against literal values. Records include physical terminators and
// malformed bytes, without an SDK lexer that could truncate them. Assertions
// throw in Debug and Release, and main converts failures to a nonzero exit.
// No Reader provider, configuration resolver or pressure pipeline is simulated.
//
// Call tree:
//   main: run state, pressure, acceleration and recognition groups.
//     test_state: capture independent values across later reads and new exports.
//     test_pressure: check literal accepted values and typed row-specific errors.
//       expect_pressure_error: require ControlError and immutable inputs.
//     test_acceleration: check accepted updates and owned ignored-field issues.
//     test_recognition: check exact prefixes, framing and opaque ordinary records.
// Shared assertion helper called by all four test groups and expect_pressure_error:
//   check: count Release-active assertions and throw on any mismatch.

// Runs all groups, reporting case/check counts and returning 1 on any failure.
int main();

namespace {
using nematx_pressure::ControlCause;
using nematx_pressure::ControlError;
using nematx_pressure::ControlIssue;
using nematx_pressure::ControlResult;
using nematx_pressure::ControlState;
using nematx_pressure::read_control_line;

int cases = 0;
int checks = 0;

// Checks persistent values, copied snapshots, explicit zero and separate exports.
void test_state();
// Checks double precision, complete numeric syntax and negative underflow errors.
void test_pressure();
// Checks finite positive acceleration and whole-prior preservation on warnings.
void test_acceleration();
// Checks byte-zero recognition and physical record terminators without mutation.
void test_recognition();
// Requires a typed pressure failure at the supplied row with readable reason,
// preserving source and prior; unexpected exceptions propagate as test failures.
void expect_pressure_error(const std::string &record, std::size_t row);
// Counts a Release-active assertion, throwing its explanation on failure.
void check(bool condition, const char *message);
} // namespace

int main()
{
    try {
        test_state();
        test_pressure();
        test_acceleration();
        test_recognition();
        std::printf("pressure controls: %d cases, %d checks, 0 failures\n", cases, checks);
        return 0;
    } catch (const std::exception &error) {
        std::printf("pressure controls: %d cases, %d checks, FAILED: %s\n",
                    cases, checks, error.what());
        return 1;
    }
}

namespace {
void test_state()
{
    ++cases;
    const ControlState initial{std::nullopt, 100.};
    const ControlResult first = read_control_line("; PRESSURE_FACTOR=.02\n", 0, initial);
    check(first.next.pressure_override == .02 && first.next.acceleration == 100., "first K snapshot");
    check(!first.issue, "valid K has no issue");
    const ControlState captured = first.next;
    ControlState current = read_control_line("; PRESSURE_FACTOR=.03", 1, first.next).next;
    check(current.pressure_override == .03, "later K update");
    current = read_control_line(";M204 S500", 2, current).next;
    check(current.acceleration == 500. && current.pressure_override == .03, "acceleration keeps K");
    check(captured.pressure_override == .02 && captured.acceleration == 100., "capture is nonretroactive");
    const char *ordinary[] = {"; fragment", ";LAYER_CHANGE\n", "T1\r\n", "G1 X2 E.1"};
    for (const char *record : ordinary) {
        ++cases;
        const ControlResult next = read_control_line(record, 3, current);
        check(next.next.pressure_override == .03 && next.next.acceleration == 500., "ordinary row persists state");
        check(!next.issue, "ordinary row has no issue");
        current = next.next;
    }
    ++cases;
    current = read_control_line("; PRESSURE_FACTOR=0", 7, current).next;
    check(current.pressure_override.has_value() && *current.pressure_override == 0., "zero is not absence");
    current = read_control_line("; PRESSURE_FACTOR=", 8, current).next;
    check(!current.pressure_override && current.acceleration == 500., "empty resets override only");
    current = read_control_line("; PRESSURE_FACTOR=.03", 9, current).next;
    current = read_control_line("; PRESSURE_FACTOR= \t \r\n", 10, current).next;
    check(!current.pressure_override && current.acceleration == 500., "blank resets override only");
    const ControlState second_export{std::nullopt, 200.};
    const ControlResult second = read_control_line("; ordinary", 0, second_export);
    check(!second.next.pressure_override && second.next.acceleration == 200., "new export independent");
    check(!initial.pressure_override && initial.acceleration == 100., "initial state unchanged");
}

void test_pressure()
{
    struct Accepted { const char *field; double value; };
    const Accepted accepted[] = {
        {".02", .02}, {"2e-2", .02}, {"+0.02", .02}, {"+.0", 0.},
        {"1.", 1.}, {"-0", 0.}, {"-0.0", 0.}, {"-00.000", 0.},
        {"-.0", 0.}, {"-0e-400", 0.}, {"-0.000e9", 0.}, {" -0 ", 0.},
        {"1e39", 1e39}, {"1e-400", 0.}, {"+1e-400", 0.},
        {"0.123456789012345", 0.123456789012345}, {" \t.02\t ", .02}
    };
    const ControlState prior{.03, 100.};
    for (const Accepted &entry : accepted) {
        ++cases;
        const std::string record = std::string("; PRESSURE_FACTOR=") + entry.field;
        const std::string saved = record;
        const ControlResult result = read_control_line(record, 11, prior);
        check(result.next.pressure_override == entry.value, "literal double K");
        check(result.next.acceleration == 100. && !result.issue, "valid K changes only override");
        if (entry.value == 0.)
            check(!std::signbit(*result.next.pressure_override), "zero K is normalized positive");
        check(record == saved && prior.pressure_override == .03 && prior.acceleration == 100., "valid K inputs unchanged");
    }
    const char *invalid[] = {
        "-.02", "-1", "nan", "nan(payload)", "inf", "-inf", "+inf", "+nan", "1e400",
        "0.02xyz", "0,02", "0x1", "abc", "++1", "+-0", "+-0.0", "+-0.02", "+",
        "-+1", "--1", "0.02; comment", "1 2", "1e", ".", "\v.02", ".02\f",
        "-1e-400", "-1e-324", "-0.1e-400", "-0.00001e-400"
    };
    std::size_t row = 20;
    for (const char *field : invalid)
        expect_pressure_error(std::string("; PRESSURE_FACTOR=") + field, row++);
}

void test_acceleration()
{
    const ControlState prior{.03, 100.};
    struct Accepted { const char *record; double value; };
    const Accepted accepted[] = {
        {";M204 S500", 500.}, {";M204 S+100", 100.},
        {";M204 S 2e2\t\r\n", 200.}, {";M204 S.5\n", .5}
    };
    for (const Accepted &entry : accepted) {
        ++cases;
        const ControlResult result = read_control_line(entry.record, 60, prior);
        check(result.next.acceleration == entry.value, "literal acceleration");
        check(result.next.pressure_override == .03 && !result.issue, "valid acceleration keeps K");
    }
    std::vector<std::string> invalid = {
        "0", "-0", "-1", "", " \t", "nan", "inf", "-inf", "1e400", "100xyz",
        "100; suffix", "0,2", "0x10", "++1", "+-0.0", "+", "+inf", "+1e-400", "1e-400",
        "100\r", "100\r \n", "10\r0", std::string("100\0tail", 8)
    };
    std::optional<ControlIssue> owned;
    std::size_t row = 61;
    for (const std::string &field : invalid) {
        ++cases;
        const std::string record = ";M204 S" + field;
        const std::string saved = record;
        const ControlResult result = read_control_line(record, row, prior);
        check(result.next.pressure_override == .03 && result.next.acceleration == 100., "warning keeps entire prior");
        check(result.issue.has_value(), "invalid acceleration has issue");
        check(result.issue->line_index == row && result.issue->cause == ControlCause::InvalidAcceleration, "warning row and cause");
        check(record == saved && prior.pressure_override == .03 && prior.acceleration == 100., "warning inputs unchanged");
        owned = result.issue;
        ++row;
    }
    check(owned && owned->line_index == 83 && owned->cause == ControlCause::InvalidAcceleration, "owned issue outlives source and result");
}

void test_recognition()
{
    const ControlState prior{.03, 100.};
    const std::vector<std::string> ordinary = {
        "N120 ; PRESSURE_FACTOR=.02", " ; PRESSURE_FACTOR=.02", "\t;M204 S500",
        "; pressure_factor=.02", ";PRESSURE_FACTOR=.02", "; PRESSURE_FACTOR =.02",
        ";m204 S500", ";M2040 S500", ";M204 P200", ";M204", "G1 X1 ; PRESSURE_FACTOR=.02",
        "G1 X1 ;M204 S500", "M204 S500", "M204 P500 R200 T100",
        std::string(1, '\0') + "; PRESSURE_FACTOR=.02", std::string("; PRESSURE_") + '\0' + "FACTOR=.02",
        "ordinary\r", std::string("ordinary\0tail", 13)
    };
    for (const std::string &record : ordinary) {
        ++cases;
        const std::string saved = record;
        const ControlResult result = read_control_line(record, 90, prior);
        check(result.next.pressure_override == .03 && result.next.acceleration == 100. && !result.issue, "nonmatching record is opaque");
        check(record == saved, "ordinary record unchanged");
    }
    const char *framed[] = {"; PRESSURE_FACTOR=.02\n", "; PRESSURE_FACTOR=.02\r\n",
                           "; PRESSURE_FACTOR=.02", "; PRESSURE_FACTOR=.02 \r\n"};
    for (const char *record : framed) {
        ++cases;
        const ControlResult result = read_control_line(record, 91, prior);
        check(result.next.pressure_override == .02 && !result.issue, "normal framing accepted");
    }
    expect_pressure_error("; PRESSURE_FACTOR=.02\r \n", 92);
    expect_pressure_error("; PRESSURE_FACTOR=.02\r", 93);
    expect_pressure_error("; PRESSURE_FACTOR=.0\r2", 94);
    expect_pressure_error(std::string("; PRESSURE_FACTOR=.02\0tail", 26), 95);
}

void expect_pressure_error(const std::string &record, std::size_t row)
{
    ++cases;
    const std::string saved = record;
    const ControlState prior{.03, 100.};
    bool caught = false;
    try {
        read_control_line(record, row, prior);
    } catch (const ControlError &error) {
        caught = true;
        check(error.line_index == row, "pressure error exact row");
        check(error.cause == ControlCause::InvalidPressureFactor, "pressure error exact cause");
        const std::string message = error.what();
        check(message.find("row " + std::to_string(row) + ":") != std::string::npos,
              "pressure what identifies exact local row");
        check(message.find("pressure factor") != std::string::npos && message.find("nonnegative") != std::string::npos,
              "pressure what explains reason");
    }
    check(caught, "invalid pressure throws ControlError");
    check(record == saved && prior.pressure_override == .03 && prior.acceleration == 100., "error inputs unchanged");
}

void check(bool condition, const char *message)
{
    ++checks;
    if (!condition)
        throw std::runtime_error(message);
}
} // namespace
