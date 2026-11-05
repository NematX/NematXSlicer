///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "GCodeFilePublisher.hpp"

#include <array>
#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <streambuf>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/crc.hpp>
#include <boost/filesystem.hpp>
#include <boost/log/trivial.hpp>
#include <boost/nowide/fstream.hpp>

#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/FilesystemTransaction.hpp"

/*
Transactional publication of an exported file
=============================================

See GCodeFilePublisher.hpp for the sequence. The destination is validated first,
so that a refusal costs no work. The writer then runs on a unique sibling stream.
The helper counts and checksums each byte accepted by the stream, closes the file,
and rereads it in bounded blocks to compare the same fingerprint. A mismatch or a
Print cancellation before replacement publishes nothing. The replacement itself
is delegated to FilesystemTransaction: it renames the existing destination to a
backup, renames the sibling to the destination, then discards the backup, and
tries to restore the backup when the second rename fails. Cancellation after the
replacement does not undo it.

Representative direct calls (branches are not a timeline):

    publish_gcode_file()
        |-- destination_with_suffix()
        |-- check_destination()
        |-- reserve_sibling()
        `-- replace_destination()
*/

namespace slic3r_api {

// Compare the borrowed name's ending with dotted suffixes without case, including the last component of a composed main suffix; no I/O or ownership transfer occurs, suffixless names remain the caller's decision and substring allocation failures propagate.
bool gcode_destination_matches_format(std::string_view destination, const GCodeFileFormat &format)
{
    if (format.extension.empty() || boost::iends_with(destination, format.extension))
        return true;
    const std::size_t last_dot = format.extension.rfind('.');
    if (last_dot != std::string::npos && last_dot > 0 &&
        boost::iends_with(destination, format.extension.substr(last_dot)))
        return true;
    for (const std::string &alias : format.aliases) {
        if (boost::iends_with(destination, alias))
            return true;
    }
    return false;
}

namespace {

// Byte count and CRC-32 calculated for one complete file stream, kept internal to the publisher.
struct GCodeFileFingerprint
{
    std::uint64_t byte_count = 0;
    std::uint32_t crc32 = 0;
};

// Compare the byte count and CRC-32 values without modifying either fingerprint.
bool same_gcode_file_fingerprint(const GCodeFileFingerprint &left, const GCodeFileFingerprint &right)
{
    return left.byte_count == right.byte_count && left.crc32 == right.crc32;
}

// Return the destination with the main suffix added when it has none, or throw when its suffix is not accepted.
boost::filesystem::path destination_with_suffix(const GCodeFileFormat &format, std::string_view destination_utf8);

// Throw unless the destination directory exists, the destination is not a directory and is not the reference.
void check_destination(const boost::filesystem::path &destination, const boost::filesystem::path &reference);

// Reserve a sibling name of the destination that carries the main suffix and does not exist yet.
boost::filesystem::path reserve_sibling(const GCodeFileFormat &format, const boost::filesystem::path &destination);

// Replace the destination by the staging file through a transaction; log cleanup warnings and throw when it did not commit.
void replace_destination(const boost::filesystem::path &staging, const boost::filesystem::path &destination);

// Add accepted bytes to the CRC-32 and byte count used to fingerprint a stream.
class GCodeFileFingerprintBuilder
{
public:
    // Add the supplied byte range; the caller provides a readable range of `size` bytes.
    void add(const char *data, std::size_t size);
    // Return the current byte count and finalized CRC-32 value.
    GCodeFileFingerprint result() const;

private:
    boost::crc_32_type m_crc32;
    std::uint64_t m_byte_count = 0;
};

// Forward stream writes to the file and fingerprint exactly the bytes accepted by its file buffer.
class GCodeFingerprintStreambuf : public std::streambuf
{
public:
    // Borrow the opened file buffer and the fingerprint builder for one writer call.
    GCodeFingerprintStreambuf(std::streambuf &file_buffer, GCodeFileFingerprintBuilder &fingerprint);

protected:
    // Forward one character and fingerprint it only when the file buffer accepts it.
    int_type overflow(int_type value) override;
    // Forward a byte block and fingerprint only the number of bytes accepted by the file buffer.
    std::streamsize xsputn(const char *data, std::streamsize size) override;
    // Flush the borrowed file buffer.
    int sync() override;

private:
    std::streambuf &m_file_buffer;
    GCodeFileFingerprintBuilder &m_fingerprint;
};

// Read a closed sibling in bounded blocks and return its byte count and CRC-32; I/O errors or cancellation throw.
GCodeFileFingerprint fingerprint_gcode_file(const plugin_run_context &run_context,
                                           const boost::filesystem::path &path);

// Open the reserved sibling, invoke the writer, close it, and compare the written and reread fingerprints; false is propagated and failures throw.
bool write_and_verify_sibling(const plugin_run_context &run_context,
                              const boost::filesystem::path &sibling,
                              const GCodeFileWriter &write);

// Build an owned path from the borrowed destination, append the main suffix only to a suffixless name, and otherwise use the shared name matcher; empty or incompatible destinations throw without I/O.
boost::filesystem::path destination_with_suffix(const GCodeFileFormat &format, std::string_view destination_utf8)
{
    if (destination_utf8.empty())
        throw std::runtime_error("G-code export requires a non-empty destination (output_path).");

    boost::filesystem::path destination{std::string(destination_utf8)};
    if (destination.extension().empty())
        return destination += format.extension;

    const std::string suffix = destination.extension().string();
    if (!gcode_destination_matches_format(destination_utf8, format))
        throw std::runtime_error("The G-code exporter '" + format.exporter_id +
                                 "' does not support the destination extension '" + suffix + "'.");
    return destination;
}

// Add a byte range to the running CRC-32 and its total byte count without taking ownership of the data.
void GCodeFileFingerprintBuilder::add(const char *data, std::size_t size)
{
    m_crc32.process_bytes(data, size);
    m_byte_count += static_cast<std::uint64_t>(size);
}

// Return the accumulated byte count and finalized CRC-32 without changing the builder.
GCodeFileFingerprint GCodeFileFingerprintBuilder::result() const
{
    GCodeFileFingerprint fingerprint;
    fingerprint.byte_count = m_byte_count;
    fingerprint.crc32 = static_cast<std::uint32_t>(m_crc32.checksum());
    return fingerprint;
}

// Retain references to the file buffer and fingerprint builder for the lifetime of one output stream.
GCodeFingerprintStreambuf::GCodeFingerprintStreambuf(std::streambuf &file_buffer,
                                                     GCodeFileFingerprintBuilder &fingerprint)
    : m_file_buffer(file_buffer)
    , m_fingerprint(fingerprint)
{}

// Forward a single character and count it only after the borrowed file buffer accepts it; report EOF when the write fails.
GCodeFingerprintStreambuf::int_type GCodeFingerprintStreambuf::overflow(int_type value)
{
    if (traits_type::eq_int_type(value, traits_type::eof()))
        return traits_type::not_eof(value);
    const char character = traits_type::to_char_type(value);
    if (traits_type::eq_int_type(m_file_buffer.sputc(character), traits_type::eof()))
        return traits_type::eof();
    m_fingerprint.add(&character, 1);
    return value;
}

// Forward a block and fingerprint only its accepted prefix; the caller retains ownership of the bytes.
std::streamsize GCodeFingerprintStreambuf::xsputn(const char *data, std::streamsize size)
{
    if (size <= 0)
        return 0;
    const std::streamsize written = m_file_buffer.sputn(data, size);
    if (written > 0)
        m_fingerprint.add(data, static_cast<std::size_t>(written));
    return written;
}

// Flush the underlying file buffer and return its synchronization status.
int GCodeFingerprintStreambuf::sync()
{
    return m_file_buffer.pubsync();
}

// Read the closed sibling in fixed-size chunks, checking cancellation between reads and rejecting open/read failures.
GCodeFileFingerprint fingerprint_gcode_file(const plugin_run_context &run_context,
                                            const boost::filesystem::path &path)
{
    const std::string path_utf8 = path.string();
    boost::nowide::ifstream input(path_utf8.c_str(), std::ios::in | std::ios::binary);
    if (!input)
        throw std::runtime_error("Cannot reopen temporary G-code output file '" + path_utf8 + "'.");

    GCodeFileFingerprintBuilder fingerprint;
    std::array<char, 64 * 1024> buffer;
    while (true) {
        throw_if_cancelled(&run_context);
        input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
        const std::streamsize count = input.gcount();
        if (count > 0)
            fingerprint.add(buffer.data(), static_cast<std::size_t>(count));
        if (input.bad() || (input.fail() && !input.eof()))
            throw std::runtime_error("Failed while rereading the temporary G-code output file.");
        if (input.eof())
            break;
    }
    return fingerprint.result();
}

// Open the helper-owned sibling and lend its stream to the writer; return false on writer refusal, otherwise close and reread it, throwing on writer, I/O, cancellation, or fingerprint failures.
bool write_and_verify_sibling(const plugin_run_context &run_context,
                              const boost::filesystem::path &sibling,
                              const GCodeFileWriter &write)
{
    const std::string sibling_utf8 = sibling.string();
    boost::nowide::ofstream file(sibling_utf8.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
    if (!file)
        throw std::runtime_error("Cannot open temporary G-code output file '" + sibling_utf8 + "'.");

    GCodeFileFingerprintBuilder fingerprint;
    GCodeFingerprintStreambuf fingerprint_buffer(*file.rdbuf(), fingerprint);
    std::ostream output(&fingerprint_buffer);
    const bool writer_succeeded = write(output);
    if (writer_succeeded) {
        output.flush();
        if (!output)
            throw std::runtime_error("Failed while flushing the temporary G-code output stream.");
    }
    file.flush();
    if (writer_succeeded && !file)
        throw std::runtime_error("Failed while flushing the temporary G-code output file.");
    file.close();
    if (writer_succeeded && !file)
        throw std::runtime_error("Failed while closing the temporary G-code output file.");
    if (!writer_succeeded)
        return false;

    const GCodeFileFingerprint written = fingerprint.result();
    const GCodeFileFingerprint reread = fingerprint_gcode_file(run_context, sibling);
    if (!same_gcode_file_fingerprint(written, reread))
        throw std::runtime_error("G-code export temporary file changed while writing.");
    return true;
}

// Check the directory first, then the destination itself, so that the error names the first thing that is wrong.
void check_destination(const boost::filesystem::path &destination, const boost::filesystem::path &reference)
{
    const boost::filesystem::path parent = destination.parent_path().empty() ?
        boost::filesystem::current_path() : destination.parent_path();
    boost::system::error_code filesystem_error;
    if (!boost::filesystem::is_directory(parent, filesystem_error) || filesystem_error)
        throw std::runtime_error("G-code export destination directory does not exist: '" + parent.string() + "'.");

    if (!boost::filesystem::exists(destination))
        return;
    if (boost::filesystem::is_directory(destination))
        throw std::runtime_error("G-code export destination is not a file path: '" + destination.string() + "'.");
    filesystem_error.clear();
    if (boost::filesystem::equivalent(reference, destination, filesystem_error))
        throw std::runtime_error("The G-code export destination must differ from its reference.");
    if (filesystem_error)
        throw std::runtime_error("Could not inspect G-code export paths: " + filesystem_error.message());
}

// The sibling lives in the destination's directory so that the replacement is a rename on one volume.
boost::filesystem::path reserve_sibling(const GCodeFileFormat &format, const boost::filesystem::path &destination)
{
    const std::string unique_suffix = boost::filesystem::unique_path("%%%%-%%%%-%%%%-%%%%").string();
    const boost::filesystem::path sibling = destination.parent_path() /
        boost::filesystem::path(destination.stem().string() + ".export-" + unique_suffix + format.extension);
    if (boost::filesystem::exists(sibling))
        throw std::runtime_error("Could not reserve a unique G-code export temporary path.");
    return sibling;
}

// A cleanup warning concerns hidden transaction artifacts after the visible file was committed: it is logged, not thrown.
void replace_destination(const boost::filesystem::path &staging, const boost::filesystem::path &destination)
{
    Slic3r::FilesystemTransaction transaction;
    transaction.add_replacement(staging, destination);
    const Slic3r::FilesystemTransactionResult result = transaction.commit();
    for (const Slic3r::FilesystemTransactionFailure &warning : result.cleanup_warnings)
        BOOST_LOG_TRIVIAL(warning) << "G-code export cleanup warning: "
                                   << Slic3r::format_filesystem_transaction_failure(warning);
    if (result.status != Slic3r::FilesystemTransactionStatus::Committed)
        throw std::runtime_error("Could not publish the G-code export: " +
                                 Slic3r::format_filesystem_transaction_error(result));
}

} // namespace

// Validate borrowed paths, own sibling cleanup, and replace the destination only after the writer's stream matches its reread; return the published path or empty on writer refusal, and throw on validation, I/O, fingerprint, cancellation, or replacement failure.
std::string publish_gcode_file(const plugin_run_context &run_context,
                               const GCodeFileFormat &format,
                               std::string_view destination_utf8,
                               std::string_view reference_path_utf8,
                               const GCodeFileWriter &write)
{
    const boost::filesystem::path destination = destination_with_suffix(format, destination_utf8);
    check_destination(destination, boost::filesystem::path(std::string(reference_path_utf8)));
    const boost::filesystem::path sibling = reserve_sibling(format, destination);

    try {
        if (!write_and_verify_sibling(run_context, sibling, write)) {
            boost::system::error_code cleanup_error;
            boost::filesystem::remove(sibling, cleanup_error);
            return std::string();
        }
        throw_if_cancelled(&run_context);
        boost::system::error_code filesystem_error;
        if (!boost::filesystem::is_regular_file(sibling, filesystem_error) || filesystem_error)
            throw std::runtime_error("G-code exporter did not create a regular output file.");
        replace_destination(sibling, destination);
    } catch (...) {
        boost::system::error_code cleanup_error;
        boost::filesystem::remove(sibling, cleanup_error);
        throw;
    }
    return destination.string();
}

} // namespace slic3r_api
