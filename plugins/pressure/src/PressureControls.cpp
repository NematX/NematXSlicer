///|/ Released under the GNU Affero General Public License, version 3 or later.

#include "PressureControls.hpp"

#include <fast_float/fast_float.h>

#include <cmath>
#include <string>
#include <system_error>

// Supplies persistent model settings to callers that already frame physical
// source records. Recognition precedes field trimming so ordinary comments and
// actual controller commands remain opaque. Numeric decoding uses the original
// bytes and locale-independent double precision, then validates the full field.
// Results contain values only; the caller retains text and decides tool settings,
// movement snapshots and how to display returned acceleration diagnostics.
//
// Call tree:
//   read_control_line: strip one normal terminator, recognize and trim a field.
//     parse_field: decode a whole signed decimal/scientific field into double.
//     ControlError: own the row and explanation when pressure is invalid.

namespace nematx_pressure {

// Returns an independent snapshot from valid prior, borrowing text only during
// this call. Invalid pressure throws before publication; invalid acceleration
// returns prior intact with its row/cause. Other records have no effect.
ControlResult read_control_line(std::string_view physical_record,
    std::size_t local_row, const ControlState &prior);

namespace {
// Decodes a nonempty complete field without locale or float intermediates.
// Returns false for bad syntax or nonfinite values; accepted underflow is zero.
// The output is used only on success and no source storage survives the call.
bool parse_field(std::string_view field, double &value);
} // namespace

ControlResult read_control_line(std::string_view physical_record,
    std::size_t local_row, const ControlState &prior)
{
    if (!physical_record.empty() && physical_record.back() == '\n') {
        physical_record.remove_suffix(1);
        if (!physical_record.empty() && physical_record.back() == '\r')
            physical_record.remove_suffix(1);
    }
    constexpr std::string_view pressure_prefix = "; PRESSURE_FACTOR=";
    constexpr std::string_view acceleration_prefix = ";M204 S";
    const bool pressure = physical_record.substr(0, pressure_prefix.size()) == pressure_prefix;
    const bool acceleration = physical_record.substr(0, acceleration_prefix.size()) == acceleration_prefix;
    ControlResult result{prior, std::nullopt};
    if (!pressure && !acceleration)
        return result;

    std::string_view field = physical_record.substr(
        pressure ? pressure_prefix.size() : acceleration_prefix.size());
    const std::size_t first = field.find_first_not_of(" \t");
    if (first == std::string_view::npos)
        field = {};
    else
        field = field.substr(first, field.find_last_not_of(" \t") - first + 1);
    if (pressure && field.empty()) {
        result.next.pressure_override.reset();
        return result;
    }

    double value = 0.;
    bool valid = parse_field(field, value);
    if (pressure) {
        valid = valid && value >= 0.;
        // Underflow can hide a negative mantissa in -0; exponent digits do not
        // change whether a syntactically valid decimal is exactly zero.
        if (valid && field.front() == '-') {
            const std::string_view mantissa = field.substr(1, field.find_first_of("eE") - 1);
            if (mantissa.find_first_of("123456789") != std::string_view::npos)
                valid = false;
        }
        if (!valid)
            throw ControlError(local_row);
        result.next.pressure_override = value == 0. ? 0. : value;
    } else if (valid && value > 0.) {
        result.next.acceleration = value;
    } else {
        result.issue = ControlIssue{local_row, ControlCause::InvalidAcceleration};
    }
    return result;
}

ControlError::ControlError(std::size_t line)
    : std::invalid_argument("Invalid pressure factor at local physical row " + std::to_string(line)
          + ": expected a complete finite nonnegative decimal field"),
      line_index(line), cause(ControlCause::InvalidPressureFactor)
{
}

namespace {
bool parse_field(std::string_view field, double &value)
{
    if (field.empty())
        return false;
    // fast_float rejects a leading plus; require a digit or dot after removing it to admit only one sign.
    if (field.front() == '+') {
        field.remove_prefix(1);
        if (field.empty() || !((field.front() >= '0' && field.front() <= '9') || field.front() == '.'))
            return false;
    }
    const char *end = field.data() + field.size();
    const fast_float::from_chars_result parsed = fast_float::from_chars(field.data(), end, value);
    return parsed.ec == std::errc{} && parsed.ptr == end && std::isfinite(value);
}
} // namespace
} // namespace nematx_pressure
