// Uploads and exports share exclusive creation while keeping distinct cleanup prefixes.
#pragma once

#include <cstdio>
#include <filesystem>
#include <memory>
#include <string>

namespace Slic3r::Rest {

// Retry collisions only; persistent storage failures must remain visible.
constexpr int max_temporary_file_attempts = 8;

// Unpredictable 128-bit names prevent clients from anticipating server-owned paths.
std::string random_file_suffix();
// Exclusive creation avoids overwriting a concurrent owner's file; the caller closes the stream.
std::FILE *open_exclusive(const std::filesystem::path &path);

// The last shared owner removes the file after transport readers release it.
struct TemporaryFile {
    explicit TemporaryFile(std::filesystem::path file_path);
    ~TemporaryFile();
    TemporaryFile(const TemporaryFile &) = delete;
    TemporaryFile &operator=(const TemporaryFile &) = delete;
    const std::filesystem::path path;
};

// Per-run directories also own any temporary files left by engine plugins.
struct TemporaryDirectory {
    explicit TemporaryDirectory(std::filesystem::path directory_path);
    ~TemporaryDirectory();
    TemporaryDirectory(const TemporaryDirectory &) = delete;
    TemporaryDirectory &operator=(const TemporaryDirectory &) = delete;
    const std::filesystem::path path;
};

// Exclusive run directories isolate engine leftovers until the last shared owner releases them.
std::shared_ptr<TemporaryDirectory> create_temporary_directory(
    const std::filesystem::path &directory, const std::string &prefix);

// Return shared cleanup ownership of a closed file; prefix and extension define its server-only domain.
std::shared_ptr<TemporaryFile> create_temporary_file(
    const std::filesystem::path &directory, const std::string &prefix, const std::string &extension);
// Startup cleanup is prefix-scoped to preserve unrelated files in the same directory.
void remove_stale_temporary_files(const std::filesystem::path &directory, const std::string &prefix);

} // namespace Slic3r::Rest
