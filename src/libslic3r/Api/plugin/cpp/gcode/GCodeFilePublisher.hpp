///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_plugin_cpp_gcode_GCodeFilePublisher_hpp_
#define slic3r_Api_plugin_cpp_gcode_GCodeFilePublisher_hpp_

#include <functional>
#include <string>
#include <string_view>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_plugin_run_context.h"

/*
Transactional publication of an exported file
=============================================

A file exporter should not leave a half-written file at its destination, and
should keep the file that already sits there when its work fails or is
cancelled. publish_gcode_file() gives every such exporter the same behavior:

    destination checks  ->  write into a unique sibling  ->  replace the destination
    (not empty, suffix,      (the exporter's own work)       (FilesystemTransaction)
     directory, not the
     reference)

The sibling shares the destination's directory so that the replacement stays on
one volume. The replacement renames an existing destination to a backup, then
renames the sibling to the destination, and discards the backup; if the second
rename fails it tries to restore the backup (and reports when it cannot). The
sibling is removed on every path that does not publish it: a failing or cancelled
write, a refusal to publish, or a failed replacement. Before the replacement the
destination is untouched. A cancellation that arrives after the replacement does
not undo it: the file stays published. The reference is never touched.

The exporter supplies the work as a function that receives the sibling's path and
returns whether it succeeded. It reports a failure itself (an exception, or
report_error followed by `false`), and nothing is published in that case.

Call publish_gcode_file() with the export context, destination description and a
writer that fills the supplied private output. The writer must finish its work
before returning success; the publisher then verifies and replaces the destination.
Keep the referenced context and writer captures alive until that synchronous call
returns. The return value describes publication, not ownership of the Print.
*/

namespace slic3r_api {

// The suffixes that one file exporter accepts for its destination.
struct GCodeFileFormat
{
    // Exporter identity, named in the refusal of a suffix.
    std::string exporter_id;
    // Main suffix with its dot (".gcode"): appended when the destination has none, and given to the sibling.
    std::string extension;
    // Other accepted suffixes (".g"), compared without case.
    std::vector<std::string> aliases;
};

// Writes the complete file at the path it receives (a UTF-8 sibling of the destination). Returns true on success; false
// when it already reported the failure; throws on I/O failure or cancellation.
using GCodeFileWriter = std::function<bool(const std::string &temporary_path_utf8)>;

// Check the destination, run the writer on a unique sibling and replace the destination by it. Return the path that
// was published (the destination, with the main suffix added when it had none), or an empty string when the writer
// returned false. Throws for an empty destination, a suffix that the format does not accept, a missing directory, a
// destination that is a directory or the reference, a writer that created no regular file, a cancellation before the
// replacement, and a failed replacement. In all these cases no sibling remains, and the destination is unchanged except when
// the replacement itself fails: it renames the old destination to a backup and then the sibling to the destination, and only
// tries to restore the backup if the second rename fails (it reports RollbackFailed when that does not work). A
// cancellation that arrives after the replacement does not undo it: the file stays published.
std::string publish_gcode_file(const plugin_run_context &run_context,
                               const GCodeFileFormat &format,
                               std::string_view destination_utf8,
                               std::string_view reference_path_utf8,
                               const GCodeFileWriter &write);

} // namespace slic3r_api

#endif // slic3r_Api_plugin_cpp_gcode_GCodeFilePublisher_hpp_
