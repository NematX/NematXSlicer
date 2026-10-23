///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "GCodeFilePublisher.hpp"

#include <stdexcept>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/filesystem.hpp>
#include <boost/log/trivial.hpp>

#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/FilesystemTransaction.hpp"

/*
Transactional publication of an exported file
=============================================

See GCodeFilePublisher.hpp for the sequence. The destination is validated first,
so that a refusal costs no work. The writer then runs on a unique sibling of the
destination (name reserved by randomness and checked against collisions, no file
created yet). After a successful write the cancellation flag is read once more, so
that a Print cancelled during the write publishes nothing (a cancellation after the
replacement does not undo it). The replacement itself is delegated to
FilesystemTransaction: it renames the existing destination to a backup, renames the
sibling to the destination, then discards the backup, and tries to restore the backup
when the second rename fails.

Representative direct calls (branches are not a timeline):

    publish_gcode_file()
        |-- destination_with_suffix()
        |-- check_destination()
        |-- reserve_sibling()
        |-- write()
        `-- replace_destination()
*/

namespace slic3r_api {
namespace {

// Return the destination with the main suffix added when it has none, or throw when its suffix is not accepted.
boost::filesystem::path destination_with_suffix(const GCodeFileFormat &format, std::string_view destination_utf8);

// Throw unless the destination directory exists, the destination is not a directory and is not the reference.
void check_destination(const boost::filesystem::path &destination, const boost::filesystem::path &reference);

// Reserve a sibling name of the destination that carries the main suffix and does not exist yet.
boost::filesystem::path reserve_sibling(const GCodeFileFormat &format, const boost::filesystem::path &destination);

// Replace the destination by the staging file through a transaction; log cleanup warnings and throw when it did not commit.
void replace_destination(const boost::filesystem::path &staging, const boost::filesystem::path &destination);

// Build the destination path. An empty path, or a suffix that is neither the main one nor an alias, is refused.
boost::filesystem::path destination_with_suffix(const GCodeFileFormat &format, std::string_view destination_utf8)
{
    if (destination_utf8.empty())
        throw std::runtime_error("G-code export requires a non-empty destination (output_path).");

    boost::filesystem::path destination{std::string(destination_utf8)};
    if (destination.extension().empty())
        return destination += format.extension;

    const std::string suffix = destination.extension().string();
    bool accepted = boost::iequals(suffix, format.extension);
    for (const std::string &alias : format.aliases)
        accepted = accepted || boost::iequals(suffix, alias);
    if (!accepted)
        throw std::runtime_error("The G-code exporter '" + format.exporter_id +
                                 "' does not support the destination extension '" + suffix + "'.");
    return destination;
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

// Validate, write into a sibling, then replace the destination; the sibling is removed unless it was published.
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
        if (!write(sibling.string())) {
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
