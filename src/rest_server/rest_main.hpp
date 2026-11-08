// Library entry points shared by the application modes and standalone wrappers.
// All argv strings are UTF-8; these functions return process exit codes.
#pragma once

#include <filesystem>
#include <optional>
#include <string>

namespace Slic3r::Rest {

// Hosts may provide resources without borrowing GUI profiles or account data.
// An empty value keeps the standalone current-directory defaults.
struct RestLaunchDefaults {
    std::optional<std::filesystem::path> resource_directory;
    // Help/error invocation supplied by the host; empty uses UTF-8 argv[0].
    std::string invocation;
};

// Run until SIGINT/SIGTERM, then join workers and restore prior signal handlers.
int rest_server_main(int argc, char **argv, const RestLaunchDefaults &defaults = {});

// Emit accounts JSON from no-echo interactive input or one redirected line
// selected by --password-stdin; the latter is forbidden for console stdin.
int rest_password_main(int argc, char **argv, const std::string &invocation = {});

} // namespace Slic3r::Rest
