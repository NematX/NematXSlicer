// Keep status conversion independent of producer threads and session lifetime.
#include "progress.hpp"

#include <algorithm>

#include "engine.hpp"

namespace Slic3r::Rest {

std::optional<ProgressUpdate> progress_update(const PrintStatus::SlicingStatus &status,
    const std::filesystem::path &directory, const std::filesystem::path &work)
{
    // TODO(progress rework): filter legacy control flags and format untranslated text until libslic3r replaces it.
    constexpr unsigned int control_flags = PrintStatus::SlicingStatus::RELOAD_SCENE |
        PrintStatus::SlicingStatus::RELOAD_SLA_SUPPORT_POINTS | PrintStatus::SlicingStatus::RELOAD_SLA_PREVIEW |
        PrintStatus::SlicingStatus::UPDATE_PRINT_STEP_WARNINGS | PrintStatus::SlicingStatus::UPDATE_PRINT_OBJECT_STEP_WARNINGS |
        PrintStatus::SlicingStatus::SLICING_ENDED | PrintStatus::SlicingStatus::GCODE_ENDED |
        PrintStatus::SlicingStatus::FORCE_SHOW;
    if ((status.flags & control_flags) != 0)
        return std::nullopt;
    return ProgressUpdate{
        (status.flags & PrintStatus::SlicingStatus::SECONDARY_STATE) != 0,
        sanitized_slice_error(format_status_text(status.main_text, status.args), directory, work),
        std::clamp(status.percent, -1, 100)};
}

std::string sanitized_slice_error(std::string message, const std::filesystem::path &directory,
    const std::filesystem::path &work)
{
    const std::string placeholder = "<output>";
    for (const std::filesystem::path &path : {directory, work}) {
        for (const std::string &prefix : {path.u8string(), path.generic_u8string(), path.string(), path.generic_string()}) {
            for (std::size_t pos = message.find(prefix); pos != std::string::npos;
                 pos = message.find(prefix, pos + placeholder.size()))
                message.replace(pos, prefix.size(), placeholder);
        }
    }
    return message;
}

} // namespace Slic3r::Rest
