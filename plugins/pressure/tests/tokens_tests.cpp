///|/ Released under the GNU Affero General Public License, version 3 or later.

#include "PressureTokens.hpp"

#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

// Checks original double values and byte ranges through the unchanged SDK lexical
// parser. Literal numeric oracles are independent of the helper and SDK float.
// Checks remain active in Release; structural failures throw before dereferencing
// a missing optional or reading outside a source. No Reader or mode interpreter
// runs. Local source copies demonstrate replacement and ownership, not a writer.
//
// Call tree:
//   main: run precision, syntax, grammar, case and lifetime groups; report failure.
//     test_precision / test_syntax / test_grammar / test_case / test_lifetime:
//       parse: obtain an owned GCodeLine through the real SDK callback.
//       expect: check exact double, literal offset, length and numeric substring.
//         read_parameter: use production lexical access.
//         check: count assertions and throw on failure in every configuration.

// Runs all groups and converts any failed assertion or unexpected error to exit 1.
int main();

namespace {
using Parser = slic3r_api::GCodeGeneration::GCodeLineParser;
using Line = Parser::GCodeLine;
using nematx_pressure::NumericToken;
using nematx_pressure::read_parameter;

int checks = 0;
int cases = 0;

// Checks source double precision, float contrast and one local numeric replacement.
void test_precision();
// Checks mixed mode text, standard reset words, literal selection and absence.
void test_syntax();
// Checks SDK acceptance parity, underflow, exponent spans and invalid isolation.
void test_grammar();
// Checks exact-case queries without making any claim about Reader translation.
void test_case();
// Checks CRLF framing, copied SDK lines and scalar results after source destruction.
void test_lifetime();
// Copies one SDK callback line into an owned return value; no borrowed view escapes.
Line parse(std::string_view source);
// Reads a present token and checks independent literal oracles, source bounds and
// immutability. Throws before optional/range access if a structural check fails.
NumericToken expect(const Line &line, char letter, double value,
                    std::size_t offset, const char *digits);
// Counts a Release-active assertion and throws with its explanation on failure.
void check(bool condition, const char *message);
} // namespace

int main()
{
    try {
        test_precision();
        test_syntax();
        test_grammar();
        test_case();
        test_lifetime();
        std::printf("pressure tokens: %d cases, %d checks, 0 failures\n", cases, checks);
        return 0;
    } catch (const std::exception &error) {
        std::printf("pressure tokens: %d cases, %d checks, FAILED: %s\n",
                    cases, checks, error.what());
        return 1;
    }
}

namespace {
void test_precision()
{
    ++cases;
    const Line line = parse("G1 X16777217 A0.123456789 F1200 ; keep X99");
    float sdk = 0.f;
    check(line.parameter_value('X', sdk) && sdk == 16777216.f, "SDK float rounds integer");
    const NumericToken x = expect(line, 'X', 16777217., 4, "16777217");
    expect(line, 'A', 0.123456789, 14, "0.123456789");
    expect(line, 'F', 1200., 27, "1200");
    std::string replaced = line.raw();
    replaced.replace(x.offset, x.length, "42.25");
    check(replaced == "G1 X42.25 A0.123456789 F1200 ; keep X99",
          "numeric replacement preserves command, other words and comment");

    ++cases;
    const Line ordinary = parse("G1 X12.3456789");
    const NumericToken coordinate = expect(ordinary, 'X', 12.3456789, 4, "12.3456789");
    check(ordinary.parameter_value('X', sdk), "ordinary SDK coordinate accepted");
    check(coordinate.value != static_cast<double>(sdk), "ordinary double differs from float");
}

void test_syntax()
{
    ++cases;
    const Line mixed = parse("G1 G90 X12.3456789 Y-0.125 G91 A0.031250001 ; source");
    expect(mixed, 'X', 12.3456789, 8, "12.3456789");
    expect(mixed, 'Y', -0.125, 20, "-0.125");
    expect(mixed, 'A', 0.031250001, 32, "0.031250001");
    ++cases;
    const Line reset = parse("G92 X-1.23456789 E0.123456789");
    expect(reset, 'X', -1.23456789, 5, "-1.23456789");
    expect(reset, 'E', 0.123456789, 18, "0.123456789");

    ++cases;
    const Line whitespace = parse("G1 X \t12 Y-0 Z.125 E0 A-2");
    expect(whitespace, 'X', 12., 6, "12");
    const NumericToken negative_zero = expect(whitespace, 'Y', -0., 10, "-0");
    check(std::signbit(negative_zero.value), "negative zero sign retained");
    expect(whitespace, 'Z', .125, 14, ".125");
    const NumericToken zero = expect(whitespace, 'E', 0., 20, "0");
    check(!std::signbit(zero.value), "positive zero sign retained");
    expect(whitespace, 'A', -2., 23, "-2");
    check(!read_parameter(whitespace, 'B').has_value(), "absent distinct from zero");

    ++cases;
    const Line duplicate = parse("G1 X1 X2 ; X99");
    expect(duplicate, 'X', 1., 4, "1");
    const char *absent_sources[] = {"G1 MIX1 ; X99", "G1 ; X99", "", "; X99", "X1"};
    for (const char *source : absent_sources) {
        ++cases;
        const Line line = parse(source);
        check(!read_parameter(line, 'X').has_value(), "no embedded, comment or command parameter");
        check(line.raw() == source, "absent query leaves source unchanged");
    }
}

void test_grammar()
{
    struct Accepted {
        const char *source;
        const char *digits;
        double value;
        float sdk_value;
    };
    const Accepted accepted[] = {
        {"G1 X1e-50", "1e-50", 1e-50, 0.f},
        {"G1 X1e-400", "1e-400", 0., 0.f},
        {"G1 X1e+2", "1e+2", 100., 100.f},
        {"G1 X1E2", "1E2", 100., 100.f}
    };
    for (const Accepted &entry : accepted) {
        ++cases;
        const Line line = parse(entry.source);
        float sdk = -1.f;
        check(line.parameter_value('X', sdk) && sdk == entry.sdk_value,
              "SDK accepts exponent with literal float oracle");
        expect(line, 'X', entry.value, 4, entry.digits);
    }
    const char *invalid[] = {"G1 X1e39", "G1 X1e309", "G1 X1e400", "G1 Xinf",
                             "G1 Xnan", "G1 Xabc", "G1 X1oops", "G1 X",
                             "G1 X Y1", "G1 X+1"};
    for (const char *source : invalid) {
        ++cases;
        const Line line = parse(source);
        float sdk = 0.f;
        check(!line.parameter_position('X').empty(), "invalid parameter is present");
        check(!line.parameter_value('X', sdk), "SDK rejects invalid parameter");
        bool rejected = false;
        try {
            read_parameter(line, 'X');
        } catch (const std::invalid_argument &) {
            rejected = true;
        }
        check(rejected, "helper rejects SDK-invalid value with invalid_argument");
        check(line.raw() == source, "invalid query leaves source unchanged");
        check(!read_parameter(line, 'B').has_value(), "absent query does not validate other words");
    }
}

void test_case()
{
    const char *sources[] = {"G1 x1 X2", "G1 G90 x1 X2", "G1 X1 x2", "G1 G90 X1 x2"};
    const std::size_t upper_offsets[] = {7, 11, 4, 8};
    const std::size_t lower_offsets[] = {4, 8, 7, 11};
    for (std::size_t index = 0; index < 4; ++index) {
        ++cases;
        const Line line = parse(sources[index]);
        expect(line, 'X', index < 2 ? 2. : 1., upper_offsets[index], index < 2 ? "2" : "1");
        expect(line, 'x', index < 2 ? 1. : 2., lower_offsets[index], index < 2 ? "1" : "2");
    }
}

void test_lifetime()
{
    ++cases;
    Parser parser;
    std::vector<Line> lines;
    {
        const std::string source = "G1 X12.3456789 ; first\r\nG92 E-0.125\r\n";
        // Copy the reusable SDK line while its callback owns the current text.
        parser.parse_buffer(source, [&lines](Parser &, const Line &line) {
            lines.push_back(line);
        });
        check(source == "G1 X12.3456789 ; first\r\nG92 E-0.125\r\n", "buffer unchanged");
    }
    check(lines.size() == 2, "two copied callback lines survive source");
    check(lines[0].raw() == "G1 X12.3456789 ; first", "first raw excludes CRLF");
    check(lines[1].raw() == "G92 E-0.125", "second raw excludes CRLF");
    expect(lines[0], 'X', 12.3456789, 4, "12.3456789");
    expect(lines[1], 'E', -0.125, 5, "-0.125");

    ++cases;
    std::optional<NumericToken> retained;
    {
        const Line original = parse("G1 A0.123456789");
        retained = read_parameter(original, 'A');
    }
    check(retained.has_value(), "scalar token survives original destruction");
    check(retained->value == .123456789 && retained->offset == 4 && retained->length == 11,
          "all three owned scalars remain exact");
}

Line parse(std::string_view source)
{
    Parser parser;
    Line owned;
    // Keep the line's owned text, never a view into the reusable callback object.
    parser.parse_line(source, [&owned](Parser &, const Line &line) { owned = line; });
    return owned;
}

NumericToken expect(const Line &line, char letter, double value,
                    std::size_t offset, const char *digits)
{
    const std::string before = line.raw();
    const std::optional<NumericToken> token = read_parameter(line, letter);
    check(token.has_value(), "expected numeric token exists");
    check(token->value == value, "exact independent double oracle");
    check(token->offset == offset, "independent numeric byte offset");
    check(token->length == std::string_view(digits).size(), "independent numeric length");
    check(token->offset <= before.size() && token->length <= before.size() - token->offset,
          "span stays inside raw without CRLF");
    check(before.substr(token->offset, token->length) == digits, "literal numeric substring");
    check(line.raw() == before, "successful query leaves source unchanged");
    return *token;
}

void check(bool condition, const char *message)
{
    ++checks;
    if (!condition)
        throw std::runtime_error(message);
}
} // namespace
