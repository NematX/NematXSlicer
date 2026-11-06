// Server-owned temporary paths must never overwrite another request's file.
#include "temporary_file.hpp"

#include <cerrno>
#include <cstdint>
#include <iomanip>
#include <mutex>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace Slic3r::Rest {

namespace {

// A server restart must not sweep directories still retained by old snapshots.
std::mutex directory_owners_mutex;
// Absolute ownership keys assume a stable process working directory.
std::set<std::filesystem::path> owned_directories;

} // namespace

std::string random_file_suffix()
{
    std::random_device random;
    std::ostringstream suffix;
    suffix << std::hex << std::setfill('0');
    for (int index = 0; index < 4; ++index)
        suffix << std::setw(8) << static_cast<std::uint32_t>(random());
    return suffix.str();
}

std::FILE *open_exclusive(const std::filesystem::path &path)
{
#ifdef _WIN32
    return _wfopen(path.c_str(), L"wbx");
#else
    return std::fopen(path.c_str(), "wbx");
#endif
}

TemporaryFile::TemporaryFile(std::filesystem::path file_path) : path(std::move(file_path)) {}

TemporaryFile::~TemporaryFile()
{
    std::error_code ignored;
    std::filesystem::remove(path, ignored);
}

std::shared_ptr<TemporaryFile> create_temporary_file(
    const std::filesystem::path &directory, const std::string &prefix, const std::string &extension)
{
    std::filesystem::create_directories(directory);
    for (int attempt = 0; attempt < max_temporary_file_attempts; ++attempt) {
        const std::filesystem::path candidate = directory / (prefix + random_file_suffix() + extension);
        errno = 0;
        std::FILE *file = open_exclusive(candidate);
        if (file != nullptr) {
            std::shared_ptr<TemporaryFile> result = std::make_shared<TemporaryFile>(candidate);
            if (std::fclose(file) != 0)
                throw std::runtime_error("The temporary file could not be closed.");
            return result;
        }
        if (errno != EEXIST)
            break;
    }
    throw std::runtime_error("The temporary file could not be stored.");
}

TemporaryDirectory::TemporaryDirectory(std::filesystem::path directory_path) : path(std::move(directory_path))
{
    std::lock_guard<std::mutex> lock(directory_owners_mutex);
    owned_directories.insert(std::filesystem::absolute(path).lexically_normal());
}

TemporaryDirectory::~TemporaryDirectory()
{
    std::error_code ignored;
    std::filesystem::remove_all(path, ignored);
    std::lock_guard<std::mutex> lock(directory_owners_mutex);
    owned_directories.erase(std::filesystem::absolute(path).lexically_normal());
}

std::shared_ptr<TemporaryDirectory> create_temporary_directory(
    const std::filesystem::path &directory, const std::string &prefix)
{
    for (int attempt = 0; attempt < max_temporary_file_attempts; ++attempt) {
        const std::filesystem::path candidate = directory / (prefix + random_file_suffix());
        if (std::filesystem::create_directory(candidate))
            return std::make_shared<TemporaryDirectory>(candidate);
    }
    throw std::runtime_error("The temporary directory could not be stored.");
}

void remove_stale_temporary_files(const std::filesystem::path &directory, const std::string &prefix)
{
    for (const std::filesystem::directory_entry &entry : std::filesystem::directory_iterator(directory)) {
        if (entry.path().filename().string().rfind(prefix, 0) != 0)
            continue;
        {
            std::lock_guard<std::mutex> lock(directory_owners_mutex);
            if (owned_directories.count(std::filesystem::absolute(entry.path()).lexically_normal()) != 0)
                continue;
        }
        std::error_code ignored;
        std::filesystem::remove_all(entry.path(), ignored);
    }
}

} // namespace Slic3r::Rest
