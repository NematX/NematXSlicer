#include "GCodePrivateFile.hpp"
#include <array>
#include <stdexcept>
#include <boost/filesystem.hpp>
#include <boost/log/trivial.hpp>
#include <boost/nowide/cstdio.hpp>
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"

/*
Some file-format libraries need a seekable FILE so they can fill in earlier
parts of their output. The publisher's ostream instead records bytes in write
order for later verification. GCodePrivateFile bridges those two requirements
without letting a library seek inside the publisher's verified stream.

Construct the guard beside the read-only reference, lend handle() to the library,
then call the guard's close() before open_read(). Copy the completed file with
copy_private_gcode_file() into the publisher's stream. Use the guard's close(),
not fclose() on its borrowed handle, so its ownership state remains accurate.
Destruction removes the private file; it never removes the reference.

See GCodePrivateFile.hpp: seekable libraries write to this guard-owned FILE,
never to the publisher's sequential fingerprint stream.

Representative direct calls (branches are not a timeline):

    GCodePrivateFile::GCodePrivateFile()
        |-- boost::filesystem::unique_path()
        `-- boost::nowide::fopen()

    GCodePrivateFile::open_read()
        `-- boost::nowide::fopen()

    copy_private_gcode_file()
        |-- std::fread()
        |-- output.write()
        |-- std::ferror()
        `-- std::feof()
*/
namespace slic3r_api {
// Reserve a random reference-side path and open it in binary mode; own both path and FILE, throwing on allocation or open failure.
GCodePrivateFile::GCodePrivateFile(std::string_view reference, const std::string &prefix)
{
    m_path = boost::filesystem::path(std::string(reference)).parent_path() /
        boost::filesystem::unique_path(prefix + "-%%%%-%%%%-%%%%.tmp");
    m_path_utf8 = m_path.string();
    m_file = boost::nowide::fopen(m_path_utf8.c_str(), "wb");
    if (m_file == nullptr)
        throw std::runtime_error("Cannot create private binary G-code file '" + m_path_utf8 + "'.");
}
// Close the owned FILE and attempt removal on every exit; cleanup and logging failures never escape destruction.
GCodePrivateFile::~GCodePrivateFile() noexcept
{
    if (m_file != nullptr)
        std::fclose(m_file);
    boost::system::error_code error;
    const bool removed = boost::filesystem::remove(m_path, error);
    if (!removed || error) {
        try {
            BOOST_LOG_TRIVIAL(warning) << "Failed to remove private binary G-code file '" << m_path_utf8
                << "': " << (error ? error.message() : "file was not removed");
        } catch (...) {
        }
    }
}
// Return a borrowed FILE without changing ownership or state; a closed guard throws logic_error.
FILE *GCodePrivateFile::handle() const
{
    if (m_file == nullptr)
        throw std::logic_error("The private binary G-code file is closed.");
    return m_file;
}
// Open the guard-owned path for binary input after close; reject an open guard and propagate fopen failure without losing cleanup ownership.
void GCodePrivateFile::open_read()
{
    if (m_file != nullptr)
        throw std::logic_error("The private binary G-code file must be closed before reading.");
    m_file = boost::nowide::fopen(m_path_utf8.c_str(), "rb");
    if (m_file == nullptr)
        throw std::runtime_error("Cannot reopen private binary G-code file '" + m_path_utf8 + "'.");
}
// Clear the owned handle before fclose so destruction cannot close it twice; retain path ownership and throw on a close error.
void GCodePrivateFile::close()
{
    if (m_file == nullptr)
        return;
    FILE *file = m_file;
    m_file = nullptr;
    if (std::fclose(file) != 0)
        throw std::runtime_error("Cannot close private binary G-code file '" + m_path_utf8 + "'.");
}
// Copy borrowed input through bounded chunks to borrowed output, checking cancellation and read/write progress; neither stream is closed and any I/O failure throws.
void copy_private_gcode_file(const plugin_run_context &run_context, FILE *input, std::ostream &output)
{
    std::array<char, 64 * 1024> buffer;
    while (true) {
        throw_if_cancelled(&run_context);
        const std::size_t count = std::fread(buffer.data(), 1, buffer.size(), input);
        if (count != 0) {
            output.write(buffer.data(), static_cast<std::streamsize>(count));
            if (!output)
                throw std::runtime_error("Failed while copying the private binary G-code file.");
        }
        if (count < buffer.size()) {
            if (std::ferror(input) != 0)
                throw std::runtime_error("Failed while reading the private binary G-code file.");
            if (std::feof(input) != 0)
                break;
            if (count == 0)
                throw std::runtime_error("The private binary G-code file read made no progress.");
        }
    }
}
}
