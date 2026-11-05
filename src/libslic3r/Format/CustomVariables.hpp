///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_Format_CustomVariables_hpp_
#define slic3r_Format_CustomVariables_hpp_

#include <map>
#include <string>

/*
Custom variables text
=====================

`print_custom_variables`, `printer_custom_variables` and the elements of `filament_custom_variables` hold
one `name = value` definition per line. This file reads that text into a map, with the rules of the
placeholder parser (PlaceholderParser::parse_custom_variables) so that every reader of the text agrees
on what a variable is. The readers are the Bambu G-code firmware (scan_first_layer,
enable_power_loss_recovery), the Bambu configuration exporter and the Bambu configuration import.

For example:

    const auto variables = parse_custom_variable_lines("scan_first_layer = true");
    const auto found = variables.find("scan_first_layer");

The returned map owns its names and values. Callers decide whether a particular
name/value is meaningful; parsing a free variable does not register a setting
or validate a firmware-specific option.
*/

namespace Slic3r {

// Read the `name = value` lines of a custom variables text: one variable per line, cut at the first `=`,
// names and values trimmed of all white space, `\r` dropped, a quoted value loses its quotes, and a later
// line replaces an earlier one of the same name. A line without `=` is ignored. A name with inner spaces
// or invalid characters never matches the names looked up, so it needs no check.
std::map<std::string, std::string> parse_custom_variable_lines(const std::string &text);

} // namespace Slic3r

#endif /* slic3r_Format_CustomVariables_hpp_ */
