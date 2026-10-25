///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_Api_plugin_cpp_StringHelpers_hpp_
#define slic3r_Api_plugin_cpp_StringHelpers_hpp_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <locale>
#include <sstream>
#include <string>

/*
Header-only text helpers for C++ plugins.

They do not depend on the host library, so external plugins can use them as
they are.

    const std::string text = slic3r_api::to_string_nozero(1.25, 6); // "1.25"
*/

namespace slic3r_api {

/*
Write a number in plain decimal notation, without trailing zeros: 2 -> "2",
1.250 -> "1.25", 0.1 -> "0.1". It always uses a '.' decimal separator, whatever
the locale of the plugin or host.

max_precision is the maximal number of decimals. A double only holds about 15
significant digits, so large values get fewer decimals: the output never shows
rounding noise beyond what the double can represent. Values smaller than the
last kept decimal become "0", so pick a large max_precision (for example 15)
when tiny values matter, as in error messages.

NaN and infinities are written "nan", "inf" and "-inf".

For finite values this matches the host's to_string_nozero(), except that a
negative value rounded to zero is written "0" instead of "-0".
*/
inline std::string to_string_nozero(double value, int32_t max_precision)
{
    if (std::isnan(value))
        return "nan";
    if (std::isinf(value))
        return value < 0. ? "-inf" : "inf";

    double int_part = 0.;
    if (std::modf(value, &int_part) == 0. && std::abs(int_part) < 9.2e18) {
        // Integer value that fits in int64_t: no decimal part to write.
        return std::to_string(int64_t(int_part));
    }

    // Count the digits of the integer part to keep at most 15 significant digits.
    int int_digits = 0;
    if (std::abs(int_part) > 9.)
        int_digits = int(std::floor(std::log10(std::abs(int_part))));
    const int decimals = std::max(0, std::min(15 - int_digits, int(max_precision)));

    std::ostringstream stream;
    stream.imbue(std::locale::classic());
    stream << std::fixed << std::setprecision(decimals) << value;
    std::string text = stream.str();

    // Remove the trailing zeros of the decimal part, then a trailing '.'.
    if (text.find('.') != std::string::npos) {
        const size_t last = text.find_last_not_of('0');
        text.erase(text[last] == '.' ? last : last + 1);
    }
    return text == "-0" ? "0" : text;
}

} // namespace slic3r_api

#endif // slic3r_Api_plugin_cpp_StringHelpers_hpp_
