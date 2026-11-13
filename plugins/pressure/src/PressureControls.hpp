///|/ Released under the GNU Affero General Public License, version 3 or later.

// Provides value snapshots of the pressure override and model acceleration.
// Initialize ControlState for each export, then pass each complete physical
// record to read_control_line and carry next forward, including outside layers.
// Copy a state when capturing movement settings; later reads cannot change it.
// Configuration, tool activation, cycle boundaries and diagnostics routing are
// caller responsibilities. This component does not interpret Reader commands.
// With caller-resolved finite positive acceleration, a typical update is:
//   nematx_pressure::ControlState state{std::nullopt, resolved_acceleration};
//   nematx_pressure::ControlResult result = nematx_pressure::read_control_line(record, row, state);
//   state = result.next; // result.issue remains available for caller diagnostics.
#pragma once

#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string_view>

namespace nematx_pressure {

// The caller supplies finite positive acceleration in mm/s², resolved from the
// legacy X limit (nonpositive falls back to 100000), then capped by a positive
// maximum extruding acceleration. Override is absent or finite nonnegative K
// in seconds; absent means configuration, while zero is an explicit override.
struct ControlState
{
    std::optional<double> pressure_override;
    double acceleration;
};

// Identifies the invalid recognized numeric field without retaining source text.
enum class ControlCause { InvalidPressureFactor, InvalidAcceleration };

// Owns the local zero-based physical row and reason for an ignored acceleration.
struct ControlIssue
{
    std::size_t line_index;
    ControlCause cause;
};

// Owns the next snapshot and, for invalid acceleration only, one diagnostic.
struct ControlResult
{
    ControlState next;
    std::optional<ControlIssue> issue;
};

// Reports an invalid pressure field with an owned local row and readable reason.
struct ControlError : std::invalid_argument
{
    std::size_t line_index;
    ControlCause cause;

    // Constructs a pressure-field failure with a zero-based physical row.
    explicit ControlError(std::size_t line);
};

// Borrows one complete physical record, optionally ending in LF or CRLF, and a
// valid prior as described above. Only exact byte-zero prefixes are recognized:
// "; PRESSURE_FACTOR=" and ";M204 S". Fields use complete C-locale decimal double
// syntax with surrounding space/tab only. Empty K clears the override; invalid K
// throws ControlError without a partial return. Invalid acceleration preserves
// both prior fields and returns an owned issue. Neither input is changed or kept.
ControlResult read_control_line(std::string_view physical_record,
    std::size_t local_row, const ControlState &prior);

} // namespace nematx_pressure
