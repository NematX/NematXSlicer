///|/ Released under the GNU Affero General Public License, version 3 or later.

#include "PressureReader.hpp"

#include <utility>

#include "libslic3r/Api/plugin/cpp/gcode/GCodeReaderService.hpp"

// Copies one complete input's Reader output for clients needing ordered deltas.
// A local copy of prior establishes the exact initial state, with a local line
// cursor. The SDK validates events before lending them to the copying observer
// and returns its owned authoritative final state. All accumulation is local;
// an exception discards it without changing the caller's preceding state.
// The service owns serialization. Its orchestrator and Print remain borrowed;
// no callback invokes the service again or retains provider storage.
//
// Call tree:
//   read_fragment: prepare initial, collect deltas, return completed owned value.
//     GCodeReaderService::read: execute the exact provider and validate callbacks.
//       observer: copy kind and every item before provider buffers can change.

namespace nematx_pressure {
namespace {
// Keep this literal in agreement with k_reader_plugin_id in
// nematx_firmware/src/NematXFirmwareSession.hpp; no default Reader is selected.
constexpr std::string_view reader_id = "gcode.reader.nematx";
}

// Copies prior with only its cursor reset, then executes the borrowed service on
// complete text. Validated deltas are copied synchronously; SDK or allocation
// failures escape without publishing local data. Neither prior nor text changes.
ReaderFragment read_fragment(const slic3r_api::GCodeReaderService &service,
                             const slic3r_api::GCodeStateTracker &prior,
                             std::string_view text);

ReaderFragment read_fragment(const slic3r_api::GCodeReaderService &service,
                             const slic3r_api::GCodeStateTracker &prior,
                             std::string_view text)
{
    slic3r_api::GCodeStateTracker initial = prior;
    initial.set_line_number(0);
    std::vector<ReaderEvent> events;
    // Copy inside the borrowed callback lifetime, after the SDK's validation.
    slic3r_api::GCodeStateTracker final = service.read(reader_id, initial.state(), text,
        [&events](const raw_gcode_reader_event &event) {
            ReaderEvent owned{event.kind, {}};
            for (std::uint32_t index = 0; index < event.item_count; ++index)
                owned.items.push_back(event.items[index]);
            events.push_back(std::move(owned));
        });
    return {std::move(initial), std::move(events), std::move(final)};
}

} // namespace nematx_pressure
