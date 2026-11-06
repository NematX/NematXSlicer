/*
 * Incremental multipart/form-data parsing and temporary-file ownership.
 *
 * The parser accepts the REST upload contract only: one file part named
 * `file`, a validated model extension, and a bounded payload written outside
 * the process heap. Everything after the closing boundary is an epilogue that
 * RFC 2046 tells receivers to ignore, whatever transport read it arrives in.
 */

#include "multipart_upload.hpp"
#include "temporary_file.hpp"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <limits>
#include <utility>

namespace Slic3r::Rest {

namespace {

constexpr const char *upload_prefix = "rest-upload-";

std::string trim_local(std::string value);
std::string remove_optional_quotes(std::string value);

std::string trim_local(std::string value)
{
    const std::string whitespace = " \t\r\n";
    const std::size_t first = value.find_first_not_of(whitespace);
    if (first == std::string::npos)
        return {};
    const std::size_t last = value.find_last_not_of(whitespace);
    return value.substr(first, last - first + 1);
}

std::string remove_optional_quotes(std::string value)
{
    value = trim_local(std::move(value));
    if (value.size() >= 2 && value.front() == '"' && value.back() == '"') {
        value = value.substr(1, value.size() - 2);
        std::string unescaped;
        unescaped.reserve(value.size());
        bool escaped = false;
        for (char character : value) {
            if (escaped) {
                unescaped.push_back(character);
                escaped = false;
            } else if (character == '\\') {
                escaped = true;
            } else {
                unescaped.push_back(character);
            }
        }
        if (escaped)
            unescaped.push_back('\\');
        return unescaped;
    }
    return value;
}

} // namespace

std::filesystem::path upload_directory(const std::filesystem::path &work_directory)
{
    return work_directory / "uploads";
}

void remove_stale_uploads(const std::filesystem::path &directory)
{
    std::error_code error;
    std::filesystem::create_directories(directory, error);
    if (error)
        throw std::invalid_argument("paths.work: cannot create the upload directory: " + error.message());
    try {
        remove_stale_temporary_files(directory, upload_prefix);
    } catch (const std::filesystem::filesystem_error &failure) {
        throw std::invalid_argument("paths.work: cannot list the upload directory: " + failure.code().message());
    }
}

void MultipartUploadParser::FileCloser::operator()(std::FILE *file) const noexcept
{
    if (file != nullptr)
        std::fclose(file);
}

MultipartUploadError::MultipartUploadError(
    unsigned int status,
    std::string code,
    std::string message)
    : std::runtime_error(std::move(message)),
      status_(status),
      code_(std::move(code))
{
}

unsigned int MultipartUploadError::status() const noexcept
{
    return status_;
}

const std::string &MultipartUploadError::code() const noexcept
{
    return code_;
}

HttpUpload::~HttpUpload()
{
    if (temporary_path.empty())
        return;
    std::error_code ignored;
    std::filesystem::remove(temporary_path, ignored);
}

MultipartUploadParser::MultipartUploadParser(
    const std::string &content_type,
    const std::filesystem::path &upload_directory,
    std::uint64_t max_bytes,
    std::uint64_t max_part_header_bytes,
    std::vector<std::string> allowed_extensions)
    : upload_directory_(upload_directory),
      boundary_(parse_boundary(content_type)),
      boundary_marker_("\r\n--" + boundary_),
      max_bytes_(max_bytes),
      max_part_header_bytes_(max_part_header_bytes),
      allowed_extensions_(std::move(allowed_extensions))
{
    if (upload_directory_.empty())
        throw std::invalid_argument("Upload directory must not be empty.");
    if (allowed_extensions_.empty())
        throw std::invalid_argument("An upload must allow at least one file extension.");
    if (max_bytes_ == 0 || max_part_header_bytes_ == 0)
        throw std::invalid_argument("Upload limits must be positive.");
}

// An unfinished parser still owns its temporary file: close and delete it.
MultipartUploadParser::~MultipartUploadParser()
{
    output_.reset();
    if (!temporary_path_.empty()) {
        std::error_code ignored;
        std::filesystem::remove(temporary_path_, ignored);
    }
}

// Bytes after the closing boundary form the epilogue and are ignored.
void MultipartUploadParser::consume(const char *data, std::size_t size)
{
    if (state_ == State::done || size == 0)
        return;
    buffer_.append(data, size);
    process();
}

std::shared_ptr<const HttpUpload> MultipartUploadParser::finish()
{
    process();
    if (state_ != State::done || !file_part_seen_)
        invalid("The multipart file is truncated or has no closing boundary.");
    // fclose flushes the stdio buffer, so its result is the last write check.
    if (std::fclose(output_.release()) != 0) {
        std::error_code ignored;
        std::filesystem::remove(temporary_path_, ignored);
        temporary_path_.clear();
        throw MultipartUploadError(500, "upload_storage_error", "The uploaded file could not be closed.");
    }

    std::shared_ptr<HttpUpload> upload = std::make_shared<HttpUpload>();
    upload->temporary_path = std::move(temporary_path_);
    upload->filename = filename_;
    upload->extension = extension_;
    upload->byte_count = byte_count_;
    return upload;
}

std::string MultipartUploadParser::parse_boundary(const std::string &content_type)
{
    const std::size_t separator = content_type.find(';');
    const std::string media_type = lower_ascii(trim(content_type.substr(0, separator)));
    if (media_type != "multipart/form-data")
        throw MultipartUploadError(400, "invalid_multipart", "The upload must use multipart/form-data.");

    std::size_t cursor = separator == std::string::npos ? content_type.size() : separator + 1;
    while (cursor < content_type.size()) {
        const std::size_t next = content_type.find(';', cursor);
        const std::string parameter = trim(content_type.substr(cursor, next - cursor));
        const std::size_t equals = parameter.find('=');
        if (equals != std::string::npos &&
            lower_ascii(trim(parameter.substr(0, equals))) == "boundary") {
            const std::string value = remove_optional_quotes(parameter.substr(equals + 1));
            if (value.empty() || value.size() > 70)
                throw MultipartUploadError(400, "invalid_multipart", "The multipart boundary is invalid.");
            for (char character : value) {
                if (std::iscntrl(static_cast<unsigned char>(character)))
                    throw MultipartUploadError(400, "invalid_multipart", "The multipart boundary is invalid.");
            }
            return value;
        }
        if (next == std::string::npos)
            break;
        cursor = next + 1;
    }
    throw MultipartUploadError(400, "invalid_multipart", "The multipart boundary is missing.");
}

std::string MultipartUploadParser::trim(std::string value)
{
    const std::string whitespace = " \t\r\n";
    const std::size_t first = value.find_first_not_of(whitespace);
    if (first == std::string::npos)
        return {};
    const std::size_t last = value.find_last_not_of(whitespace);
    return value.substr(first, last - first + 1);
}

std::string MultipartUploadParser::lower_ascii(std::string value)
{
    for (char &character : value)
        character = static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
    return value;
}

std::string MultipartUploadParser::header_parameter(const std::string &value, const std::string &name)
{
    std::size_t cursor = value.find(';');
    if (cursor == std::string::npos)
        return {};
    ++cursor;
    while (cursor <= value.size()) {
        const std::size_t next = value.find(';', cursor);
        const std::string parameter = trim(value.substr(cursor, next - cursor));
        const std::size_t equals = parameter.find('=');
        if (equals != std::string::npos &&
            lower_ascii(trim(parameter.substr(0, equals))) == lower_ascii(name))
            return remove_optional_quotes(parameter.substr(equals + 1));
        if (next == std::string::npos)
            break;
        cursor = next + 1;
    }
    return {};
}

std::string MultipartUploadParser::safe_filename(const std::string &filename)
{
    if (filename.empty())
        throw MultipartUploadError(400, "invalid_multipart", "The uploaded file name is missing.");
    for (char character : filename) {
        if (character == '\0' || std::iscntrl(static_cast<unsigned char>(character)))
            throw MultipartUploadError(400, "invalid_multipart", "The uploaded file name is invalid.");
    }
    const std::size_t separator = filename.find_last_of("/\\");
    const std::string basename = separator == std::string::npos ? filename : filename.substr(separator + 1);
    if (basename.empty() || basename == "." || basename == "..")
        throw MultipartUploadError(400, "invalid_multipart", "The uploaded file name is invalid.");
    return basename;
}

std::string MultipartUploadParser::supported_extension(const std::string &filename) const
{
    const std::size_t dot = filename.find_last_of('.');
    if (dot == std::string::npos)
        unsupported("The uploaded file extension is not supported.");
    const std::string extension = lower_ascii(filename.substr(dot));
    if (std::find(allowed_extensions_.begin(), allowed_extensions_.end(), extension) == allowed_extensions_.end()) {
        std::string formats;
        for (const std::string &allowed : allowed_extensions_)
            formats += (formats.empty() ? "" : ", ") + allowed;
        unsupported("The uploaded file must use one of these formats: " + formats + ".");
    }
    return extension;
}

void MultipartUploadParser::process()
{
    for (;;) {
        const State previous = state_;
        switch (state_) {
        case State::preamble:
            process_preamble();
            break;
        case State::headers:
            process_headers();
            break;
        case State::data:
            process_data();
            break;
        case State::boundary:
            process_boundary();
            break;
        case State::done:
            buffer_.clear();
            return;
        }
        if (state_ == previous && (state_ == State::preamble || state_ == State::headers ||
                                   state_ == State::data || state_ == State::boundary))
            return;
    }
}

void MultipartUploadParser::process_preamble()
{
    const std::string marker = "--" + boundary_;
    const std::size_t position = buffer_.find(marker);
    if (position == std::string::npos) {
        if (buffer_.size() > marker.size())
            buffer_.erase(0, buffer_.size() - marker.size() + 1);
        if (buffer_.size() > max_part_header_bytes_)
            invalid("The multipart opening boundary is missing.");
        return;
    }
    buffer_.erase(0, position + marker.size());
    state_ = State::boundary;
    opening_boundary_ = true;
}

void MultipartUploadParser::process_headers()
{
    const std::size_t separator = buffer_.find("\r\n\r\n");
    if (separator == std::string::npos) {
        if (buffer_.size() > max_part_header_bytes_)
            invalid("The multipart part headers are too large.");
        return;
    }

    const std::string headers = buffer_.substr(0, separator);
    buffer_.erase(0, separator + 4);
    std::string disposition;
    std::size_t line_start = 0;
    while (line_start <= headers.size()) {
        const std::size_t line_end = headers.find("\r\n", line_start);
        const std::string line = trim(headers.substr(line_start, line_end - line_start));
        const std::size_t colon = line.find(':');
        if (colon != std::string::npos &&
            lower_ascii(trim(line.substr(0, colon))) == "content-disposition")
            disposition = trim(line.substr(colon + 1));
        if (line_end == std::string::npos)
            break;
        line_start = line_end + 2;
    }
    if (disposition.empty() || lower_ascii(trim(disposition.substr(0, disposition.find(';')))) != "form-data")
        invalid("The multipart file part has no valid Content-Disposition header.");
    if (file_part_seen_)
        invalid("Only one multipart file part is accepted.");
    if (header_parameter(disposition, "name") != "file")
        invalid("The multipart field must be named file.");
    const std::string filename = safe_filename(header_parameter(disposition, "filename"));
    extension_ = supported_extension(filename);
    filename_ = filename;
    open_output();
    file_part_seen_ = true;
    state_ = State::data;
}

void MultipartUploadParser::process_data()
{
    const std::size_t position = buffer_.find(boundary_marker_);
    if (position == std::string::npos) {
        const std::size_t retained = boundary_marker_.size() - 1;
        if (buffer_.size() > retained)
            write_payload_prefix(buffer_.size() - retained);
        return;
    }
    write_payload_prefix(position);
    buffer_.erase(0, boundary_marker_.size());
    state_ = State::boundary;
}

void MultipartUploadParser::process_boundary()
{
    if (buffer_.size() < 2)
        return;
    if (opening_boundary_) {
        if (buffer_.compare(0, 2, "\r\n") != 0)
            invalid("The multipart opening boundary terminator is invalid.");
        buffer_.erase(0, 2);
        opening_boundary_ = false;
        state_ = State::headers;
        return;
    }
    // "--" closes the body; the CRLF and any epilogue after it are ignored.
    if (buffer_.compare(0, 2, "--") == 0) {
        buffer_.clear();
        state_ = State::done;
        return;
    }
    if (buffer_.compare(0, 2, "\r\n") == 0)
        invalid("Only one multipart file part is accepted.");
    invalid("The multipart boundary terminator is invalid.");
}

// Create the temporary file under a fresh random name. The client file name
// only contributes the validated extension the engine loader dispatches on.
void MultipartUploadParser::open_output()
{
    std::error_code error;
    std::filesystem::create_directories(upload_directory_, error);
    if (error)
        throw MultipartUploadError(500, "upload_storage_error", "The upload directory could not be created.");
    for (int attempt = 0; attempt < max_temporary_file_attempts; ++attempt) {
        const std::filesystem::path candidate = upload_directory_ / (upload_prefix + random_file_suffix() + extension_);
        errno = 0;
        output_.reset(open_exclusive(candidate));
        if (output_ != nullptr) {
            temporary_path_ = candidate;
            return;
        }
        if (errno != EEXIST)
            break;
    }
    throw MultipartUploadError(500, "upload_storage_error", "The uploaded file could not be stored.");
}

void MultipartUploadParser::write_payload(const char *data, std::size_t size)
{
    if (size == 0)
        return;
    if (size > max_bytes_ - std::min(max_bytes_, byte_count_))
        throw MultipartUploadError(413, "request_too_large", "The uploaded file exceeds the configured size limit.");
    if (std::fwrite(data, 1, size, output_.get()) != size)
        throw MultipartUploadError(500, "upload_storage_error", "The uploaded file could not be written.");
    byte_count_ += size;
}

void MultipartUploadParser::write_payload_prefix(std::size_t size)
{
    if (size == 0)
        return;
    write_payload(buffer_.data(), size);
    buffer_.erase(0, size);
}

void MultipartUploadParser::invalid(std::string message) const
{
    throw MultipartUploadError(400, "invalid_multipart", std::move(message));
}

void MultipartUploadParser::unsupported(std::string message) const
{
    throw MultipartUploadError(422, "unsupported_model_format", std::move(message));
}

} // namespace Slic3r::Rest
