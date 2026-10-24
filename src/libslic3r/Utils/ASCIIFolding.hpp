///|/ Copyright (c) SuperSlicer 2026 Durand R?mi @supermerill
///|/ Copyright (c) Prusa Research 2018 - 2020 Lukáš Matěna @lukasmatena, Vojtěch Bubník @bubnikv
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
ASCII approximations for selected Unicode characters
------------------------------------------------------
fold_utf8_to_ascii converts UTF-8 to wide characters, maps supported characters
to ASCII approximations, preserves characters without a mapping, and converts
the result back to UTF-8. fold_to_ascii applies the same mapping to one wide
character and writes at most four wide characters to the caller's buffer.

For a filename proposal, use the whole-string helper:

    std::string proposal = fold_utf8_to_ascii(source_name);

For one wide character, provide wchar_t output[4] to fold_to_ascii() and consume
the range from output to its returned end pointer. That range is not promised to
have a trailing zero. Both forms return or write caller-owned text.
*/

#ifndef slic3r_ASCIIFolding_hpp_
#define slic3r_ASCIIFolding_hpp_

#include <string>
namespace Slic3r {

// If possible, remove accents from accented latin characters.
// This function is useful for generating file names to be processed by legacy firmwares.
extern std::string 	fold_utf8_to_ascii(const std::string &src);

// Convert the input UNICODE character to a string of maximum 4 output ASCII characters.
// Return the end of the string written to the output.
// The output buffer must be at least 4 characters long.
extern wchar_t* fold_to_ascii(wchar_t c, wchar_t *out);

} // namespace Slic3r

#endif /* slic3r_ASCIIFolding_hpp_ */
