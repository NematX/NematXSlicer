// Validate session names and derive portable download file names from them.
#include "file_names.hpp"

#include <array>
#include <stdexcept>

namespace Slic3r::Rest {

namespace {

// Length of the well-formed UTF-8 sequence starting at `index`, or 0 for an
// invalid byte (overlong forms, surrogates and code points above U+10FFFF
// included).
std::size_t utf8_sequence_length(const std::string &text, std::size_t index)
{
    const unsigned char lead = static_cast<unsigned char>(text[index]);
    std::size_t length = 0;
    unsigned char low = 0x80;
    unsigned char high = 0xBF;
    if (lead < 0x80)
        return 1;
    if (lead >= 0xC2 && lead <= 0xDF)
        length = 2;
    else if (lead >= 0xE0 && lead <= 0xEF) {
        length = 3;
        if (lead == 0xE0)
            low = 0xA0;
        else if (lead == 0xED)
            high = 0x9F;
    } else if (lead >= 0xF0 && lead <= 0xF4) {
        length = 4;
        if (lead == 0xF0)
            low = 0x90;
        else if (lead == 0xF4)
            high = 0x8F;
    } else
        return 0;
    if (index + length > text.size())
        return 0;
    for (std::size_t offset = 1; offset < length; ++offset) {
        const unsigned char continuation = static_cast<unsigned char>(text[index + offset]);
        if (continuation < (offset == 1 ? low : 0x80) || continuation > (offset == 1 ? high : 0xBF))
            return 0;
    }
    return length;
}

bool is_control(unsigned char character)
{
    return character < 0x20 || character == 0x7F;
}

char ascii_upper(char character)
{
    return character >= 'a' && character <= 'z' ? static_cast<char>(character - 'a' + 'A') : character;
}

bool ends_with_ignoring_case(const std::string &text, const std::string &suffix)
{
    if (text.size() < suffix.size())
        return false;
    for (std::size_t index = 0; index < suffix.size(); ++index)
        if (ascii_upper(text[text.size() - suffix.size() + index]) != ascii_upper(suffix[index]))
            return false;
    return true;
}

void trim_trailing_dots_and_spaces(std::string &text)
{
    while (!text.empty() && (text.back() == '.' || text.back() == ' '))
        text.pop_back();
}

// Windows refuses a device name as the part before the first dot, whatever
// the case and the extension.
bool is_windows_device_name(const std::string &stem)
{
    std::string device = stem.substr(0, stem.find('.'));
    while (!device.empty() && device.back() == ' ')
        device.pop_back();
    for (char &character : device)
        character = ascii_upper(character);
    static const std::array<const char *, 4> fixed{"CON", "PRN", "AUX", "NUL"};
    for (const char *name : fixed)
        if (device == name)
            return true;
    return device.size() == 4 && (device.compare(0, 3, "COM") == 0 || device.compare(0, 3, "LPT") == 0) &&
        device[3] >= '1' && device[3] <= '9';
}

// Cut `text` to at most `bytes` bytes without splitting a UTF-8 sequence.
// The text is already well-formed, so continuation bytes are recognizable.
void truncate_utf8(std::string &text, std::size_t bytes)
{
    if (text.size() <= bytes)
        return;
    std::size_t end = bytes;
    while (end > 0 && (static_cast<unsigned char>(text[end]) & 0xC0) == 0x80)
        --end;
    text.resize(end);
}

} // namespace

void validate_session_name(const std::string &name)
{
    if (name.empty() || name.size() > max_session_name_bytes)
        throw std::invalid_argument("The session name must contain 1 to 255 UTF-8 bytes.");
    for (std::size_t index = 0; index < name.size();) {
        const std::size_t length = utf8_sequence_length(name, index);
        if (length == 0)
            throw std::invalid_argument("The session name must be valid UTF-8.");
        if (length == 1 && is_control(static_cast<unsigned char>(name[index])))
            throw std::invalid_argument("The session name must not contain control characters.");
        index += length;
    }
}

std::optional<std::string> session_name_from_file(const std::string &file_name)
{
    std::string name = file_name;
    // A leading dot starts a hidden name, not an extension.
    const std::size_t dot = name.find_last_of('.');
    if (dot != std::string::npos && dot > 0)
        name.resize(dot);
    try {
        validate_session_name(name);
    } catch (const std::invalid_argument &) {
        return std::nullopt;
    }
    return name;
}

std::optional<std::string> portable_file_name(const std::string &name, const std::string &extension)
{
    std::string stem;
    for (std::size_t index = 0; index < name.size();) {
        const std::size_t length = utf8_sequence_length(name, index);
        if (length == 0) {
            stem.push_back('_');
            ++index;
            continue;
        }
        const char character = name[index];
        if (length == 1 && (is_control(static_cast<unsigned char>(character)) ||
                std::string("<>:\"/\\|?*").find(character) != std::string::npos))
            stem.push_back('_');
        else
            stem.append(name, index, length);
        index += length;
    }
    // An extension already present keeps the case the caller chose; otherwise
    // the expected one is appended.
    std::string suffix = extension;
    if (ends_with_ignoring_case(stem, extension)) {
        suffix = stem.substr(stem.size() - extension.size());
        stem.resize(stem.size() - extension.size());
    }
    trim_trailing_dots_and_spaces(stem);
    if (stem.empty())
        return std::nullopt;
    if (is_windows_device_name(stem))
        stem.insert(stem.begin(), '_');
    truncate_utf8(stem, max_file_name_bytes - suffix.size());
    trim_trailing_dots_and_spaces(stem);
    if (stem.empty())
        return std::nullopt;
    return stem + suffix;
}

} // namespace Slic3r::Rest
