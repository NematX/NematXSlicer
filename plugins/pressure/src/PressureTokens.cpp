///|/ Released under the GNU Affero General Public License, version 3 or later.

#include "PressureTokens.hpp"

#include <fast_float/fast_float.h>

#include <stdexcept>
#include <string_view>

// Supplies lexical values without the SDK Reader's float rounding. The SDK first
// chooses and validates the original parameter, keeping grammar and delimiters
// authoritative. Decoding that same text into double then preserves its precision;
// the float is only a validation destination. The identical fast_float grammar
// gives the same numeric end pointer for float and double. A finite validated
// float also guarantees a finite double, including accepted underflow to zero.
// Only scalar value/range data leaves the call; no source storage is retained.
//
// Call tree:
//   read_parameter: locate literal word, reject SDK-invalid values, decode double.
//     GCodeLine::parameter_position: select the original source suffix.
//     GCodeLine::parameter_value: validate float grammar, finiteness and delimiter.
//     fast_float::from_chars: decode original digits and locate the numeric end.

namespace nematx_pressure {

// Borrows one parsed line for this call, returning owned numeric scalars or
// nullopt for absence. SDK validation failure throws invalid_argument before any
// result is published; neither the source nor Reader state is modified.
std::optional<NumericToken> read_parameter(
    const slic3r_api::GCodeGeneration::GCodeLineParser::GCodeLine &line,
    char literal_letter);

std::optional<NumericToken> read_parameter(
    const slic3r_api::GCodeGeneration::GCodeLineParser::GCodeLine &line,
    char literal_letter)
{
    const std::string_view position = line.parameter_position(literal_letter);
    if (position.empty())
        return std::nullopt;

    float validation_value = 0.f;
    if (!slic3r_api::GCodeGeneration::GCodeLineParser::GCodeLine::parameter_value(
            position, validation_value))
        throw std::invalid_argument("Invalid SDK numeric parameter");

    // Reuse the SDK's input range, not its rounded float, to retain source digits.
    const char *begin = position.data() + 1;
    const char *end = position.data() + position.size();
    while (begin < end && (*begin == ' ' || *begin == '\t'))
        ++begin;
    double value = 0.;
    const fast_float::from_chars_result parsed = fast_float::from_chars(begin, end, value);
    return NumericToken{value, static_cast<std::size_t>(begin - line.raw().data()),
                        static_cast<std::size_t>(parsed.ptr - begin)};
}

} // namespace nematx_pressure
