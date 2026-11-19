///|/ Released under the GNU Affero General Public License, version 3 or later.

// Provides owned original bytes and LF-delimited ranges for Reader clients.
// Read one complete fragment first, then join that exact text with its result.
// With <utility> included by the caller, use two separate instructions:
//   ReaderFragment reader = read_fragment(service, prior, text);
//   SourceFragment source = index_fragment(std::move(text), std::move(reader));
// Reader events must already have passed SDK validation. The Reader supplies
// semantics; original bytes precede its normalization and may retain N prefixes.
// CR and NUL are preserved here, even when a lexical line view ends before them.
#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "PressureReader.hpp"

namespace nematx_pressure {

// offset/length describe bytes in SourceFragment.text, including the terminator.
// first_event/event_count select the contiguous half-open range
// [first_event, first_event + event_count) in SourceFragment.reader.events.
// Its vector index is the zero-based physical line number within this fragment.
struct SourceLine
{
    std::size_t offset;
    std::size_t length;
    std::size_t first_event;
    std::size_t event_count;
};

// Owns bytes, Reader states and deltas; scalar ranges survive implicit copy/move.
struct SourceFragment
{
    std::string text;
    ReaderFragment reader;
    std::vector<SourceLine> lines;
};

// Joins text and SDK-validated events from exactly the same read_fragment call.
// Splits only at LF, with no empty record after a terminal LF. Requires the
// NematX final cursor to equal the LF count and event lines to be in bounds and
// nondecreasing; otherwise throws std::invalid_argument without a partial result.
// This cursor check cannot establish content identity. Reader values and item
// order are unchanged, and final need not equal event replay. Arguments own local
// values: copying preserves caller inputs, moving intentionally transfers them.
SourceFragment index_fragment(std::string text, ReaderFragment reader);

} // namespace nematx_pressure
