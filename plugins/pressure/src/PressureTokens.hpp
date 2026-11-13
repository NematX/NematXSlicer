///|/ Released under the GNU Affero General Public License, version 3 or later.

#pragma once

#include <cstddef>
#include <optional>

#include "libslic3r/Api/plugin/cpp/gcode/GCodeLineParser.hpp"

// Provides lexical double values and numeric byte ranges to pressure callers.
// Use a parsed SDK line, then query the exact source letter:
//   std::optional<nematx_pressure::NumericToken> x =
//       nematx_pressure::read_parameter(line, 'X');
// The SDK skips the first word (normally the command), selects the first literal
// duplicate and ignores text after ';'. Letter case is significant; spaces/tabs
// after it are allowed.
// This helper assigns no modes, axes or tools. The caller must establish Reader
// state and map the original letter according to the actual Reader path, not a
// global case-folding rule. Lexical acceptance alone proves no machine meaning.

namespace nematx_pressure {

// Owns only scalars. The range excludes the letter and whitespace, and addresses
// numeric bytes in line.raw(), without CR/LF. Applying it requires the same
// unchanged original text; the caller handles fragment offsets and edit order.
struct NumericToken
{
    double value;
    std::size_t offset;
    std::size_t length;
};

// Returns absence before validation; a present SDK-invalid value throws
// std::invalid_argument. Reads the original number as double after SDK float
// validation, without retaining text or changing the borrowed line.
std::optional<NumericToken> read_parameter(
    const slic3r_api::GCodeGeneration::GCodeLineParser::GCodeLine &line,
    char literal_letter);

} // namespace nematx_pressure
