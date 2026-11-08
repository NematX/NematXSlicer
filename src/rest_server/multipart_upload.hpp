/*
 * Incremental multipart/form-data parser for model uploads.
 *
 * Only the single `file` part required by the REST contract is accepted. File
 * bytes are written directly to a server-generated temporary path while the
 * parser retains only boundary and part-header fragments in memory.
 *
 * Temporary files live in the `uploads` subdirectory of the server work
 * directory. Their names are random and created exclusively, so a client can
 * neither choose nor predict a server path. Files left behind by a crashed
 * process are removed when the server starts.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rest_server.hpp"

namespace Slic3r::Rest {

class MultipartUploadError final : public std::runtime_error {
public:
    MultipartUploadError(unsigned int status, std::string code, std::string message);

    unsigned int status() const noexcept;
    const std::string &code() const noexcept;

private:
    unsigned int status_;
    std::string code_;
};

// Directory holding the temporary upload files of one server work directory.
std::filesystem::path upload_directory(const std::filesystem::path &work_directory);
// Create the upload directory and delete the temporary files a previous
// process left there. It assumes one server instance per work directory.
void remove_stale_uploads(const std::filesystem::path &upload_directory);

class MultipartUploadParser final {
public:
    // `allowed_extensions` are lower-case with their dot (".3mf"); a file with
    // another extension is refused before any byte of it is stored.
    MultipartUploadParser(
        const std::string &content_type,
        const std::filesystem::path &upload_directory,
        std::uint64_t max_bytes,
        std::uint64_t max_part_header_bytes,
        std::vector<std::string> allowed_extensions);
    ~MultipartUploadParser();

    MultipartUploadParser(const MultipartUploadParser &) = delete;
    MultipartUploadParser &operator=(const MultipartUploadParser &) = delete;

    void consume(const char *data, std::size_t size);
    std::shared_ptr<const HttpUpload> finish();

private:
    enum class State {
        preamble,
        headers,
        data,
        boundary,
        done
    };

    struct FileCloser {
        void operator()(std::FILE *file) const noexcept;
    };

    static std::string parse_boundary(const std::string &content_type);
    static std::string trim(std::string value);
    static std::string lower_ascii(std::string value);
    static std::string header_parameter(const std::string &value, const std::string &name);
    static std::string safe_filename(const std::string &filename);
    std::string supported_extension(const std::string &filename) const;

    void process();
    void process_preamble();
    void process_headers();
    void process_data();
    void process_boundary();
    void open_output();
    void write_payload(const char *data, std::size_t size);
    void write_payload_prefix(std::size_t size);
    [[noreturn]] void invalid(std::string message) const;
    [[noreturn]] void unsupported(std::string message) const;

    std::filesystem::path upload_directory_;
    std::string boundary_;
    std::string boundary_marker_;
    std::uint64_t max_bytes_;
    std::uint64_t max_part_header_bytes_;
    std::vector<std::string> allowed_extensions_;
    std::string buffer_;
    State state_ = State::preamble;
    std::unique_ptr<std::FILE, FileCloser> output_;
    std::filesystem::path temporary_path_;
    std::string filename_;
    std::string extension_;
    std::uint64_t byte_count_ = 0;
    bool file_part_seen_ = false;
    bool opening_boundary_ = false;
};

} // namespace Slic3r::Rest
