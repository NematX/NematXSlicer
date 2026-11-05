#ifndef slic3r_GCodePrivateFile_hpp_
#define slic3r_GCodePrivateFile_hpp_

#include <cstdio>
#include <ostream>
#include <string>
#include <string_view>
#include <boost/filesystem/path.hpp>
#include "libslic3r/Api/plugin/c/slic3r_plugin_run_context.h"

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

Private file used by seekable exporters before transactional publication.
The guard owns a unique reference-side path and FILE. Writers borrow the FILE,
close it before rereading, and copy bounded chunks to the publisher's stream.
Destruction closes and removes the file on success, failure and cancellation.
*/
namespace slic3r_api {
class GCodePrivateFile
{
public:
    // Create a unique binary file beside the reference, using the supplied name prefix.
    explicit GCodePrivateFile(std::string_view reference, const std::string &prefix = "bgcode");
    // Close and remove the owned file, logging cleanup failures without throwing.
    ~GCodePrivateFile() noexcept;
    // Forbid copying the path and FILE ownership into another guard.
    GCodePrivateFile(const GCodePrivateFile &) = delete;
    // Forbid replacing this guard with another guard's owned resources.
    GCodePrivateFile &operator=(const GCodePrivateFile &) = delete;
    // Borrow the open FILE, rejecting a closed guard.
    FILE *handle() const;
    // Reopen the closed owned file for binary reading.
    void open_read();
    // Close the owned FILE and report close errors.
    void close();
private:
    boost::filesystem::path m_path;
    std::string m_path_utf8;
    FILE *m_file = nullptr;
};
// Copy a borrowed private FILE to a borrowed output stream without closing either.
void copy_private_gcode_file(const plugin_run_context &run_context, FILE *input, std::ostream &output);
}
#endif
