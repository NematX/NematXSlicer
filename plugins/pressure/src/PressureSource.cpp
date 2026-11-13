///|/ Released under the GNU Affero General Public License, version 3 or later.

#include "PressureSource.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

// Joins original text to validated Reader deltas without interpreting commands.
// Three passes separate byte framing, event counts and event-range prefixes.
// Framing counts LF for the NematX final cursor, which advances only on LF,
// not on a complete pending command at EOF. Event items supply local line numbers;
// the caller guarantees their SDK validity and provenance from the same read.
// All storage is local and owned; only a complete result escapes. No Reader is
// invoked, no state is replayed, and no shared mutable data is used.
//
// Call tree:
//   index_fragment: frame bytes, check cursor, count events, prefix their ranges.
//     std::find_if: locate the validated LINE_NUMBER item without changing order.

namespace nematx_pressure {

// Frames owned bytes, counts validated deltas and assigns contiguous prefixes.
// Requires same-read inputs and SDK-valid events; invalid cursor, bounds or order
// throws invalid_argument. Allocation failures also discard the local result.
// Reader states and item order are transferred unchanged, with no replay.
SourceFragment index_fragment(std::string text, ReaderFragment reader);

SourceFragment index_fragment(std::string text, ReaderFragment reader)
{
    SourceFragment source{std::move(text), std::move(reader), {}};
    std::size_t begin = 0;
    std::size_t lf_count = 0;
    for (std::size_t index = 0; index < source.text.size(); ++index) {
        if (source.text[index] == '\n') {
            source.lines.push_back({begin, index + 1 - begin, 0, 0});
            begin = index + 1;
            ++lf_count;
        }
    }
    if (begin < source.text.size())
        source.lines.push_back({begin, source.text.size() - begin, 0, 0});
    if (source.reader.final.line_number() != lf_count)
        throw std::invalid_argument("NematX Reader cursor differs from source LF count.");

    // SDK validation guarantees a readable integer LINE_NUMBER, wherever it occurs.
    std::size_t previous_line = 0;
    for (const ReaderEvent &event : source.reader.events) {
        const std::vector<raw_gcode_reader_event_item>::const_iterator item =
            std::find_if(event.items.begin(), event.items.end(),
                [](const raw_gcode_reader_event_item &value) {
                    return value.type == RAW_GCODE_READER_ITEM_LINE_NUMBER;
                });
        const std::size_t line = static_cast<std::size_t>(item->value);
        if (line >= source.lines.size() || line < previous_line)
            throw std::invalid_argument("Reader event cannot join source lines in order.");
        ++source.lines[line].event_count;
        previous_line = line;
    }

    // Lines without events retain the current event cursor, including at the end.
    std::size_t cursor = 0;
    for (SourceLine &line : source.lines) {
        line.first_event = cursor;
        cursor += line.event_count;
    }
    return source;
}

} // namespace nematx_pressure
