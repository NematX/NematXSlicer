// Translate engine statuses into sanitized REST progress without session state.
#pragma once

#include <filesystem>
#include <optional>
#include <string>

#include "libslic3r/PrintStatus.hpp"

namespace Slic3r::Rest {

struct ProgressUpdate {
    bool secondary;
    std::string message;
    int percentage;
};

// Control events have side effects, not progress; never expose their payloads as messages.
std::optional<ProgressUpdate> progress_update(const PrintStatus::SlicingStatus &status,
    const std::filesystem::path &directory, const std::filesystem::path &work);

// Mask engine-native and UTF-8 work paths in both progress messages and failures.
std::string sanitized_slice_error(std::string message, const std::filesystem::path &directory,
    const std::filesystem::path &work);

} // namespace Slic3r::Rest
