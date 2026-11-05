///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "CustomVariables.hpp"

#include <algorithm>
#include <cctype>
#include <sstream>

/*
Custom variables text
=====================

The text is read line by line. Each line is cut at its first `=`: the left part is the name, the right
part the value, both trimmed of every white space character (like boost::algorithm::trim). A value
surrounded by double quotes is stored without them. When a name appears twice, the last line wins, as
for the placeholder parser. Nothing is validated: the readers only look up the few names they know.

Representative direct calls (branches are not a timeline):

    parse_custom_variable_lines()
        |-- std::getline()
        |-- std::remove()
        |-- trim()
        |-- line.substr()
        `-- value.substr()
*/

namespace Slic3r {

std::map<std::string, std::string> parse_custom_variable_lines(const std::string &text)
{
    // Every white space character is trimmed, like boost::algorithm::trim.
    const auto trim = [](std::string value) {
        const auto is_blank = [](const unsigned char c) { return std::isspace(c) != 0; };
        const size_t first = size_t(std::find_if_not(value.begin(), value.end(), is_blank) - value.begin());
        if (first == value.size())
            return std::string();
        const size_t last = size_t(std::find_if_not(value.rbegin(), value.rend(), is_blank).base() - value.begin());
        return value.substr(first, last - first);
    };
    std::map<std::string, std::string> variables;
    std::istringstream lines(text);
    std::string line;
    while (std::getline(lines, line)) {
        line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
        const size_t equal = line.find('=');
        if (equal == std::string::npos)
            continue;
        const std::string name = trim(line.substr(0, equal));
        std::string value = trim(line.substr(equal + 1));
        if (value.size() >= 2 && value.front() == '"' && value.back() == '"')
            value = value.substr(1, value.size() - 2);
        variables[name] = value;
    }
    return variables;
}

} // namespace Slic3r
