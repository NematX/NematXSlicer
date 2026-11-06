// Session names and the portable file names derived from them for downloads.
#pragma once

#include <cstddef>
#include <optional>
#include <string>

namespace Slic3r::Rest {

// Longest session name and longest download file name, in UTF-8 bytes.
constexpr std::size_t max_session_name_bytes = 255;
constexpr std::size_t max_file_name_bytes = 255;

// Throws std::invalid_argument unless the name is valid UTF-8, has 1 to 255
// bytes and contains no control character.
void validate_session_name(const std::string &name);

// The session name given by a loaded project file: the file name without its
// last extension, as the GUI project name. Nothing when no valid name remains.
std::optional<std::string> session_name_from_file(const std::string &file_name);

// A file name accepted by Windows, Linux and macOS, ending with `extension`
// (for example ".3mf"). Unsupported characters and invalid UTF-8 bytes become
// '_', trailing dots and spaces are removed, Windows device names get a '_'
// prefix, and the name is truncated on a character boundary. Nothing when no
// usable name remains, so the caller can fall back to another name.
std::optional<std::string> portable_file_name(const std::string &name, const std::string &extension);

} // namespace Slic3r::Rest
