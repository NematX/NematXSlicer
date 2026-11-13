///|/ Released under the GNU Affero General Public License, version 3 or later.

// Provides owned double coordinates alongside the NematX Reader event stream.
// Start with an empty NominalState, then pass each returned final to the next
// fragment from the same export. Axis letters must be resolved exactly as in
// that Reader's print context. For example:
//   NominalFragment next = read_nominal_fragment(source, prior, {'A', 'B'});
// SourceFragment must contain the same original text and validated Reader read.
// This helper follows endpoints, including arcs and text outside layers; it
// neither classifies printing nor qualifies any text for rewriting.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <vector>

namespace nematx_pressure {
struct SourceFragment;

// Known XYZ and per-target E positions in mm, and modal feedrate in mm/min.
struct NominalState
{
    std::array<std::optional<double>, 3> xyz;
    std::map<std::uint32_t, double> e;
    std::optional<double> feedrate;
};

// Numeric bytes in SourceFragment.text, excluding the original source letter.
// Only scalars survive; applying a range requires the same unchanged text.
struct SourceToken
{
    std::size_t offset;
    std::size_t length;
    char letter;
};

// One Reader motion's endpoints and selected X/Y/Z/E/F tokens. Optional flags
// report only proven pre-motion state; an E target need not prove active_tool.
// XYZ and E positions are in mm; e_delta is signed and may be negative.
// Feedrate is in mm/min. Token indices are exactly X, Y, Z, E, F, with numeric
// byte ranges in SourceFragment.text.
struct NominalMotion
{
    std::size_t event_index;
    std::array<std::optional<double>, 3> start, end;
    std::optional<std::uint32_t> e_target;
    std::optional<double> e_start, e_end, e_delta, feedrate;
    std::array<std::optional<SourceToken>, 5> tokens;
    std::optional<bool> xyz_absolute, e_absolute, units_mm;
    std::optional<std::uint32_t> active_tool;
};

// Owns the ordered motions and the final nominal coordinate state of a fragment.
struct NominalFragment
{
    std::vector<NominalMotion> motions;
    NominalState final;
};

// Follows validated events with original numeric doubles, never Reader floats.
// Requires prior from the same export and exactly matching known XYZ/E/F keys
// at entry and exit; mismatch throws invalid_argument without a partial result.
// Impossible numeric provenance throws logic_error. Copying prior leaves the
// caller unchanged; moving intentionally transfers its owned values. Equality
// of keys does not establish text identity or correct prior numeric values.
NominalFragment read_nominal_fragment(const SourceFragment &source, NominalState prior,
                                     const std::vector<char> &resolved_axis_letters);

} // namespace nematx_pressure
