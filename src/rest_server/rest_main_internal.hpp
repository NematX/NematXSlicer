// Test seams for the real launch implementations, not alternate server modes.
// Callbacks supply an ephemeral port or password; they add no public CLI options.
#pragma once

#include "rest_main.hpp"

#include <cstdint>
#include <functional>
#include <iosfwd>
#include <string>

namespace Slic3r::Rest {

// Windows console input is ASCII-only; unsupported keys fail without producing
// a truncated secret. Injected key codes exercise the same no-echo reader loop.
std::string read_ascii_console_password(const std::function<int()> &read_character);

// Launches are sequential within a process because signal handlers are global.
// on_started runs after binding; exceptions still stop the server via RAII.
int run_rest_server(int argc, char **argv, const RestLaunchDefaults &defaults,
    const std::function<void(std::uint16_t)> &on_started);

// The production entry point uses the same parser, hashing and document writer.
int run_password_tool(int argc, char **argv, const std::function<std::string()> &read_password,
    std::ostream &out, std::ostream &err, const std::string &invocation = {},
    std::istream *password_input = nullptr, bool input_is_console = false);

} // namespace Slic3r::Rest
