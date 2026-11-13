///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_plugin_cpp_gcode_GCodeFilePublisher_hpp_
#define slic3r_Api_plugin_cpp_gcode_GCodeFilePublisher_hpp_

#include <functional>
#include <ostream>
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

    destination checks  ->  write into a unique sibling  ->  verify its fingerprint  ->  replace the destination
    (not empty, main suffix  (the exporter's own work)       (byte count and CRC-32)      (FilesystemTransaction)
     added when missing,
     directory, not the
     reference)

Any destination suffix is accepted: the user chooses the file name. Only a name
without a suffix receives the exporter's main suffix.

The sibling shares the destination's directory so that the replacement stays on
one volume. The replacement renames an existing destination to a backup, then
renames the sibling to the destination, and discards the backup; if the second
rename fails it tries to restore the backup (and reports when it cannot). The
sibling is removed on every path that does not publish it: a failing or cancelled
write, a refusal to publish, or a failed replacement. Before the replacement the
destination is untouched. A cancellation that arrives after the replacement does
not undo it: the file stays published. The reference is never touched.

The exporter supplies the work as a function that writes to a stream opened on the
sibling and returns whether it succeeded. While bytes pass through that stream,
the helper records their byte count and CRC-32. It closes and rereads the sibling,
then compares that fingerprint before replacing the destination. A failure or a
different fingerprint leaves the destination untouched. CRC-32 detects changed
bytes but is not a cryptographic authenticity check. The exporter reports a
failure itself (an exception, or report_error followed by `false`).

Call publish_gcode_file() with the export context, destination description and a
writer that fills the supplied private output. The writer must finish its work
before returning success; the publisher then verifies and replaces the destination.
Keep the referenced context and writer captures alive until that synchronous call
returns. The return value describes publication, not ownership of the Print.
*/

namespace slic3r_api {

// The usual suffixes of one file exporter. They name suffixless destinations and staging files and fill
// file filters; a destination with any other suffix is still accepted.
struct GCodeFileFormat
{
    // Exporter identity.
    std::string exporter_id;
    // Main suffix with its dot (".gcode"): appended when the destination has none, and given to the sibling.
    std::string extension;
    // Other usual suffixes (".g"), compared without case.
    std::vector<std::string> aliases;
};

// Tell whether a complete destination name ends with the main suffix, its final component or an explicit alias, without case;
// an empty main suffix matches every name. Publication does not require a match.
bool gcode_destination_matches_format(std::string_view destination, const GCodeFileFormat &format);

// Write the complete file into the helper-owned sibling stream; return false after reporting a failure, or throw on I/O failure or cancellation.
using GCodeFileWriter = std::function<bool(std::ostream &output)>;

// Check the borrowed destination/reference, write and reread a unique sibling stream, then transactionally replace the destination; return its published path or empty on writer refusal, and throw on an empty destination, I/O, fingerprint, cancellation, or replacement failure.
std::string publish_gcode_file(const plugin_run_context &run_context,
                               const GCodeFileFormat &format,
                               std::string_view destination_utf8,
                               std::string_view reference_path_utf8,
                               const GCodeFileWriter &write);

} // namespace slic3r_api

#endif // slic3r_Api_plugin_cpp_gcode_GCodeFilePublisher_hpp_
