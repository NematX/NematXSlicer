///|/ Released under the GNU Affero General Public License, version 3 or later.

// Provides owned Reader inputs, ordered event deltas and final states to clients.
// Call read_fragment with a complete input and the preceding final state, then
// retain its result by value. GCodeStateTracker owns and rebuilds its raw views.
// To inspect intermediate states, apply temporary raw events formed from each
// owned event to a copy of initial. Final remains authoritative: private values
// and the final line cursor need not be reproduced by replaying event deltas.
// No parsing, interpretation, rewriting or pressure runtime is performed here.
#pragma once

#include <cstdint>
#include <string_view>
#include <vector>

#include "libslic3r/Api/plugin/cpp/gcode/GCodeStateTracker.hpp"

namespace slic3r_api { class GCodeReaderService; }

namespace nematx_pressure {

// All items, including event-local markers and diagnostics, in provider order.
struct ReaderEvent
{
    std::uint32_t kind;
    std::vector<raw_gcode_reader_event_item> items;
};

// Two complete states per input, with owned deltas rather than per-event states.
// Implicit copy/move keeps destination views valid; moved-from trackers are only
// safe to destroy or assign, not to use as complete Reader states.
struct ReaderFragment
{
    slic3r_api::GCodeStateTracker initial;
    std::vector<ReaderEvent> events;
    slic3r_api::GCodeStateTracker final;
};

// Executes exactly gcode.reader.nematx once, returning only after SDK success.
// Copies prior and changes only LINE_NUMBER to zero for initial; prior is never
// mutated. Event line numbers and the final cursor are relative to this input.
// The caller supplies one complete Reader input, not half of a command; empty
// text and a complete last command without LF are valid. No pending text is kept.
// Service, its borrowed orchestrator/Print context, prior and text must live for
// the synchronous call. Only owned values survive it. SDK failures propagate
// without a partial result. The observer only copies under the SDK's nonrecursive
// mutex; callers must not arrange a nested read on that same service.
ReaderFragment read_fragment(const slic3r_api::GCodeReaderService &service,
                             const slic3r_api::GCodeStateTracker &prior,
                             std::string_view text);

} // namespace nematx_pressure
