///|/ Released under the GNU Affero General Public License, version 3 or later.

#include "PressureReader.hpp"

#include <cstdio>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "libslic3r/Api/plugin/cpp/gcode/GCodeReaderService.hpp"

// Exercises the owned Reader client through the real inline SDK and three C
// orchestrator stubs. Script means an explicit list of events and a final state,
// not a parser: source bytes are checked unchanged but never interpreted.
// One local Script lends opaque handles and callback buffers for a synchronous
// call. Buffers are overwritten after every callback to expose borrowed storage.
// All checks run in Release too; no host library, registry or real Reader runs.
//
// Call tree:
//   main: run ownership, complete-input and failure groups; report exceptions.
//     test_owned: check exact initial, event order, replay and authoritative final.
//     test_lifetimes: propagate final and copy/move after source destruction.
//     test_inputs: check empty input and a complete command without LF.
//     test_failures: reject partial results and compare a clean subsequent call.
//       read_fragment -> SDK -> three orchestrator C stubs:
//         find_plugin: record exact IDs and lend a local opaque handle.
//         plugin_step: select the scripted valid or wrong step.
//         execute_plugin: pull bytes, lend/reuse event arrays, publish final.
//       make_prior / make_script: create explicit owned scalar data.
//       verify_result / compare_state / compare_items: check owned raw snapshots.
//       check: count and print failures without throwing through C callbacks.

// Runs all groups and converts unexpected exceptions to a failing exit code.
int main();

namespace {
using nematx_pressure::ReaderEvent;
using nematx_pressure::ReaderFragment;
using nematx_pressure::read_fragment;
using slic3r_api::GCodeReaderService;
using slic3r_api::GCodeStateTracker;
using Item = raw_gcode_reader_event_item;

constexpr std::uint32_t private_item = RAW_GCODE_READER_ITEM_CUSTOM_BEGIN + 17;
int failures = 0;
int checks = 0;
int cases = 0;

// Owns one provider's explicit output and scratch buffers; handles borrow tokens.
struct Script
{
    int orchestrator_token = 0;
    int plugin_token = 0;
    bool unavailable = false;
    bool wrong_step = false;
    bool missing_final = false;
    raw_plugin_execution_status status = RAW_PLUGIN_EXECUTION_SUCCESS;
    std::string text;
    std::vector<ReaderEvent> events;
    GCodeStateTracker final;
    std::optional<GCodeStateTracker> initial_seen;
    std::vector<std::string> ids;
    std::vector<Item> scratch;
    int executions = 0;
    int rejected = 0;
    int accepted = 0;
};

// Active only while a test's local provider and service live; never concurrent.
Script *active = nullptr;

// Checks complete snapshots, event-local items and replay without equating final.
void test_owned();
// Checks final-to-initial propagation and independent copy/move destinations.
void test_lifetimes();
// Checks unchanged empty and no-LF source delivery through the SDK pull protocol.
void test_inputs();
// Checks six error paths, rejected callback precedence and clean subsequent calls.
void test_failures();
// Builds a prior with known zero, unknown Z, modes, tool and opaque private data.
GCodeStateTracker make_prior();
// Builds explicit ordered deltas and a deliberately different authoritative final.
Script make_script();
// Checks each copied event against explicit data, plus round-trip raw state views.
void verify_result(const ReaderFragment &actual, const Script &expected,
                   const GCodeStateTracker &prior);
// Checks complete raw snapshots by constructing another owned tracker from them.
void compare_state(const GCodeStateTracker &actual, const GCodeStateTracker &expected);
// Checks exact ordered type/target/double identity; throws on an unreadable range.
void compare_items(const Item *actual, std::size_t actual_count,
                   const Item *expected, std::size_t expected_count);
// Counts every assertion and prints a descriptive failure in all configurations.
void check(bool condition, const char *message);
} // namespace

extern "C" {
// Records lookup IDs and lends a local token only for the selected available ID.
// Pointers are borrowed for the call; check() never throws here.
const orchestrator_plugin_handle *orchestrator_find_plugin(
    orchestrator_handle *orchestrator, const char *plugin_id);
// Returns the scripted step for a borrowed local token, without a real registry.
slicing_step_t orchestrator_plugin_step(const orchestrator_plugin_handle *plugin);
// Drives one Reader payload synchronously, mutating all borrowed arrays after
// callbacks. Records callback rejection but deliberately continues to expose SDK
// error precedence. Owns no host objects; returns the scripted execution status.
raw_plugin_execution_status orchestrator_execute_plugin(
    orchestrator_handle *orchestrator, const orchestrator_plugin_handle *plugin,
    const print_handle *print, const raw_plugin_run_payload *payloads, uint32_t run_count);
}

int main()
{
    try {
        test_owned();
        test_lifetimes();
        test_inputs();
        test_failures();
    } catch (const std::exception &error) {
        std::printf("FAILED: unexpected exception: %s\n", error.what());
        ++failures;
    } catch (...) {
        check(false, "unexpected nonstandard exception");
    }
    active = nullptr;
    std::printf("pressure reader: %d cases, %d checks, %d failures\n", cases, checks, failures);
    return failures == 0 ? 0 : 1;
}

extern "C" const orchestrator_plugin_handle *orchestrator_find_plugin(
    orchestrator_handle *orchestrator, const char *plugin_id)
{
    check(active != nullptr, "provider is alive for lookup");
    if (active == nullptr)
        return nullptr;
    check(orchestrator == reinterpret_cast<orchestrator_handle *>(&active->orchestrator_token),
          "lookup uses borrowed orchestrator");
    const std::string id = plugin_id == nullptr ? "" : plugin_id;
    active->ids.push_back(id);
    check(id == "gcode.reader.nematx", "every lookup exclusively requests literal NematX ID");
    if ((id == "gcode.reader.nematx" && !active->unavailable) ||
        id == "gcode.reader.default")
        return reinterpret_cast<const orchestrator_plugin_handle *>(&active->plugin_token);
    return nullptr;
}

extern "C" slicing_step_t orchestrator_plugin_step(const orchestrator_plugin_handle *plugin)
{
    check(active != nullptr, "provider is alive for step");
    if (active == nullptr)
        return GCODE_FIRMWARE;
    check(plugin == reinterpret_cast<const orchestrator_plugin_handle *>(&active->plugin_token),
          "step receives local opaque handle");
    return active->wrong_step ? GCODE_FIRMWARE : GCODE_READER;
}

extern "C" raw_plugin_execution_status orchestrator_execute_plugin(
    orchestrator_handle *orchestrator, const orchestrator_plugin_handle *plugin,
    const print_handle *print, const raw_plugin_run_payload *payloads, uint32_t run_count)
{
    check(active != nullptr, "provider is alive for execution");
    if (active == nullptr)
        return RAW_PLUGIN_EXECUTION_PLUGIN_ERROR;
    Script &script = *active;
    ++script.executions;
    check(orchestrator == reinterpret_cast<orchestrator_handle *>(&script.orchestrator_token),
          "execute uses same orchestrator");
    check(plugin == reinterpret_cast<const orchestrator_plugin_handle *>(&script.plugin_token),
          "execute uses same plugin");
    check(print == nullptr, "standalone context has no Print");
    check(run_count == 1 && payloads != nullptr, "exactly one Reader payload");
    if (run_count != 1 || payloads == nullptr || payloads[0].data == nullptr)
        return RAW_PLUGIN_EXECUTION_INVALID_ARGUMENT;
    run_ctx_gcode_reader &context = *static_cast<run_ctx_gcode_reader *>(payloads[0].data);
    check(context.operation == RAW_GCODE_READER_OPERATION_READ, "Reader operation is READ");
    check(context.print == print, "payload carries borrowed Print");
    check(context.initial_state != nullptr && context.get_next_chunk != nullptr &&
          context.emit_event != nullptr && context.emit_final_state != nullptr,
          "SDK supplies complete callbacks and initial state");
    if (context.initial_state == nullptr || context.get_next_chunk == nullptr ||
        context.emit_event == nullptr || context.emit_final_state == nullptr)
        return RAW_PLUGIN_EXECUTION_INVALID_ARGUMENT;
    script.initial_seen.emplace(*context.initial_state);

    // Pull to EOS before final publication; text overload supplies at most one chunk.
    raw_gcode_reader_chunk chunk{};
    raw_gcode_reader_result result{};
    context.get_next_chunk(context.callback_context, &chunk, &result);
    check(result.status == RAW_GCODE_READER_STATUS_SUCCESS, "first source pull succeeds");
    check(chunk.byte_count == script.text.size(), "source byte count is unchanged");
    if (chunk.byte_count != 0 && chunk.bytes != nullptr)
        check(std::string_view(chunk.bytes, static_cast<std::size_t>(chunk.byte_count)) == script.text,
              "source bytes arrive intact");
    else
        check(script.text.empty(), "only empty input has an empty first chunk");
    context.get_next_chunk(context.callback_context, &chunk, &result);
    check(result.status == RAW_GCODE_READER_STATUS_SUCCESS && chunk.byte_count == 0,
          "source reaches EOS successfully");

    // Reuse one provider array, then overwrite every scalar after each callback.
    for (const ReaderEvent &event : script.events) {
        script.scratch = event.items;
        const raw_gcode_reader_event raw{event.kind, script.scratch.data(),
                                         static_cast<uint32_t>(script.scratch.size())};
        context.emit_event(context.callback_context, &raw, &result);
        if (result.status == RAW_GCODE_READER_STATUS_SUCCESS)
            ++script.accepted;
        else
            ++script.rejected;
        for (Item &item : script.scratch)
            item = {RAW_GCODE_READER_ITEM_X, 0, -999.};
    }
    if (!script.missing_final) {
        const raw_gcode_reader_state &state = script.final.state();
        script.scratch.assign(state.items, state.items + state.item_count);
        const raw_gcode_reader_state raw{script.scratch.data(),
                                         static_cast<uint32_t>(script.scratch.size())};
        context.emit_final_state(context.callback_context, &raw, &result);
        check(result.status == RAW_GCODE_READER_STATUS_SUCCESS, "scripted final is accepted");
        for (Item &item : script.scratch)
            item = {RAW_GCODE_READER_ITEM_Y, 0, -888.};
    }
    return script.status;
}

namespace {
void test_owned()
{
    ++cases;
    Script script = make_script();
    active = &script;
    GCodeReaderService service(reinterpret_cast<orchestrator_handle *>(&script.orchestrator_token));
    const GCodeStateTracker prior = make_prior();
    const GCodeStateTracker saved = prior;
    const ReaderFragment result = read_fragment(service, prior, script.text);
    verify_result(result, script, prior);
    compare_state(prior, saved);
    check(script.executions == 1 && script.ids.size() == 2, "one execute and two SDK lookups");
    check(script.accepted == 6 && script.rejected == 0, "all six explicit deltas accepted");

    // Replay ordered deltas, not final-only private changes or the final cursor.
    GCodeStateTracker replay = result.initial;
    for (const ReaderEvent &event : result.events) {
        const raw_gcode_reader_event raw{event.kind, event.items.data(),
                                         static_cast<uint32_t>(event.items.size())};
        replay.apply(raw);
    }
    check(replay.active_tool() == 2 && replay.e_is_absolute() == false, "same-line tool then mode");
    check(replay.x() == 2.125 && replay.e(2) == .03125, "linear values retain exact doubles");
    check(!replay.known(RAW_GCODE_READER_ITEM_Y), "invalidation removes known zero");
    check(!replay.known(RAW_GCODE_READER_ITEM_Z), "omitted coordinate remains unknown");
    check(!replay.known(RAW_GCODE_READER_ITEM_MARKER) &&
          !replay.known(RAW_GCODE_READER_ITEM_DIAGNOSTIC_TYPE), "local items do not become state");
    check(replay.line_number() == 8 && result.final.line_number() == 11, "final cursor may differ");
    check(replay.value(private_item, 4) == 7.25 && result.final.value(private_item, 4) == -3.5,
          "final private state is authoritative rather than event replay");
}

void test_lifetimes()
{
    ++cases;
    std::optional<ReaderFragment> copied;
    std::optional<ReaderFragment> moved;
    std::optional<ReaderFragment> copy_assigned;
    std::optional<ReaderFragment> move_assigned;
    Script expected = make_script();
    const GCodeStateTracker expected_prior = make_prior();
    expected.initial_seen = expected_prior;
    expected.initial_seen->set_line_number(0);
    {
        Script script = make_script();
        active = &script;
        GCodeReaderService service(reinterpret_cast<orchestrator_handle *>(&script.orchestrator_token));
        GCodeStateTracker prior = make_prior();
        std::string text = script.text;
        ReaderFragment first = read_fragment(service, prior, text);
        copied.emplace(first);
        moved.emplace(std::move(first));
        ReaderFragment copy_source = *copied;
        copy_assigned.emplace();
        *copy_assigned = copy_source;
        ReaderFragment move_source = *copied;
        move_assigned.emplace();
        *move_assigned = std::move(move_source);
        // Mutate all original borrowed values and reassign sources before destruction.
        prior = GCodeStateTracker();
        text.assign(200, '!');
        first = ReaderFragment{};
        copy_source = ReaderFragment{};
        script = Script{};
    }
    active = nullptr;
    verify_result(*copied, expected, expected_prior);
    verify_result(*moved, expected, expected_prior);
    verify_result(*copy_assigned, expected, expected_prior);
    verify_result(*move_assigned, expected, expected_prior);

    // A second complete input inherits all authoritative private and portable keys.
    Script second;
    second.text = "G1 X2";
    second.final = copied->final;
    second.final.set_line_number(0);
    active = &second;
    GCodeReaderService service(reinterpret_cast<orchestrator_handle *>(&second.orchestrator_token));
    const ReaderFragment result = read_fragment(service, copied->final, second.text);
    GCodeStateTracker expected_initial = copied->final;
    expected_initial.set_line_number(0);
    compare_state(result.initial, expected_initial);
    compare_state(result.final, second.final);
    check(second.initial_seen.has_value(), "second input received a complete initial");
    if (!second.initial_seen)
        throw std::runtime_error("second initial snapshot is missing");
    compare_state(*second.initial_seen, expected_initial);
    check(result.initial.value(private_item + 1, 9) == 123.125,
          "final-only opaque item survives into next input");
    check(result.initial.reader_id() == 54321, "authoritative Reader identity persists");
    check(result.events.empty(), "no invented events in second input");
}

void test_inputs()
{
    for (const std::string &text : {std::string(), std::string("G1 X2")}) {
        ++cases;
        Script script;
        script.text = text;
        script.final = make_prior();
        script.final.set_line_number(text.empty() ? 0 : 7);
        active = &script;
        GCodeReaderService service(reinterpret_cast<orchestrator_handle *>(&script.orchestrator_token));
        const GCodeStateTracker prior = make_prior();
        const ReaderFragment result = read_fragment(service, prior, text);
        verify_result(result, script, prior);
        check(result.events.empty(), "empty script emits no artificial events");
        check(script.executions == 1 && script.ids.size() == 2, "complete input executes once");
    }
}

void test_failures()
{
    // Each failure gets fresh provider data, while one service tests collector isolation.
    for (int failure = 0; failure < 6; ++failure) {
        ++cases;
        Script script = make_script();
        script.events.resize(1);
        const char *message = "";
        switch (failure) {
        case 0: script.unavailable = true; message = "unavailable"; break;
        case 1: script.wrong_step = true; message = "not a G-code reader"; break;
        case 2: script.status = RAW_PLUGIN_EXECUTION_PLUGIN_ERROR; message = "failed"; break;
        case 3: script.status = RAW_PLUGIN_EXECUTION_CANCELLED; message = "cancelled"; break;
        case 4: script.missing_final = true; message = "without a final state"; break;
        case 5:
            // Reject the second event after one accepted event; provider still says SUCCESS.
            script.events.push_back({RAW_GCODE_READER_EVENT_LINEAR_MOVE,
                {{RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 1.},
                 {RAW_GCODE_READER_ITEM_X, 0, 3.}, {RAW_GCODE_READER_ITEM_X, 0, 4.}}});
            message = "duplicates an item target";
            break;
        }
        active = &script;
        GCodeReaderService service(reinterpret_cast<orchestrator_handle *>(&script.orchestrator_token));
        const GCodeStateTracker prior = make_prior();
        const GCodeStateTracker saved = prior;
        bool returned = false;
        bool threw = false;
        try {
            const ReaderFragment rejected = read_fragment(service, prior, script.text);
            returned = true;
        } catch (const std::runtime_error &error) {
            threw = true;
            check(std::string(error.what()).find(message) != std::string::npos,
                  "SDK preserves the expected failure reason");
        }
        check(threw && !returned, "failure returns no partial owned result");
        compare_state(prior, saved);
        check(script.executions == (failure < 2 ? 0 : 1), "failure execution count");
        if (failure >= 2)
            check(script.accepted == 1, "failure follows one accepted event");
        if (failure == 5)
            check(script.rejected == 1 && script.status == RAW_PLUGIN_EXECUTION_SUCCESS,
                  "callback rejection wins over provider success");

        // Reset provider, not service: subsequent success equals a clean control.
        const int token = script.orchestrator_token;
        script = make_script();
        script.orchestrator_token = token;
        const ReaderFragment clean = read_fragment(service, prior, script.text);
        verify_result(clean, script, prior);
        Script control = make_script();
        active = &control;
        GCodeReaderService control_service(
            reinterpret_cast<orchestrator_handle *>(&control.orchestrator_token));
        const ReaderFragment reference = read_fragment(control_service, prior, control.text);
        verify_result(reference, control, prior);
        compare_state(clean.initial, reference.initial);
        compare_state(clean.final, reference.final);
        check(clean.events.size() == reference.events.size(), "clean collector matches fresh control");
    }
}

GCodeStateTracker make_prior()
{
    const Item items[] = {
        {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 137.},
        {RAW_GCODE_READER_ITEM_READER_ID, 0, 54321.},
        {RAW_GCODE_READER_ITEM_X, 0, -1.25},
        {RAW_GCODE_READER_ITEM_Y, 0, 0.},
        {RAW_GCODE_READER_ITEM_E, 1, 5.125},
        {RAW_GCODE_READER_ITEM_E, 2, 0.},
        {RAW_GCODE_READER_ITEM_ACTIVE_TOOL, 0, 1.},
        {RAW_GCODE_READER_ITEM_UNITS_MM, 0, 1.},
        {RAW_GCODE_READER_ITEM_XYZ_ABSOLUTE, 0, 1.},
        {RAW_GCODE_READER_ITEM_E_ABSOLUTE, 0, 1.},
        {private_item, 4, 7.25}
    };
    return GCodeStateTracker(raw_gcode_reader_state{items, 11});
}

Script make_script()
{
    Script script;
    script.text = "\n; comment\nT2 G91 A.03125\n\n; retained\n; marker\n; diagnostic\n; gap\nM999\n\n\n";
    script.events = {
        {RAW_GCODE_READER_EVENT_STATE_UPDATE,
            {{RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 2.}, {RAW_GCODE_READER_ITEM_ACTIVE_TOOL, 0, 2.}}},
        {RAW_GCODE_READER_EVENT_STATE_UPDATE,
            {{RAW_GCODE_READER_ITEM_E_ABSOLUTE, 0, 0.}, {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 2.}}},
        {RAW_GCODE_READER_EVENT_LINEAR_MOVE,
            {{RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 2.}, {RAW_GCODE_READER_ITEM_X, 0, 2.125},
             {RAW_GCODE_READER_ITEM_E, 2, .03125}, {RAW_GCODE_READER_ITEM_FEEDRATE, 0, 12.5}}},
        {RAW_GCODE_READER_EVENT_MARKER,
            {{RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 5.},
             {RAW_GCODE_READER_ITEM_MARKER, 0, RAW_GCODE_READER_MARKER_SEAM}}},
        {RAW_GCODE_READER_EVENT_DIAGNOSTIC,
            {{RAW_GCODE_READER_ITEM_DIAGNOSTIC_TYPE, 0, RAW_GCODE_READER_DIAGNOSTIC_UNKNOWN_COMMAND},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 6.}}},
        {RAW_GCODE_READER_EVENT_INVALIDATE,
            {{RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 8.}, {RAW_GCODE_READER_ITEM_Y, 0, 0.}}}
    };
    script.final = make_prior();
    script.final.set_line_number(11);
    script.final.set_custom_value(private_item, 4, -3.5);
    script.final.set_custom_value(private_item + 1, 9, 123.125);
    return script;
}

void verify_result(const ReaderFragment &actual, const Script &expected,
                   const GCodeStateTracker &prior)
{
    GCodeStateTracker initial = prior;
    initial.set_line_number(0);
    compare_state(actual.initial, initial);
    check(expected.initial_seen.has_value(), "provider receives initial state");
    if (!expected.initial_seen)
        throw std::runtime_error("provider initial snapshot is missing");
    compare_state(*expected.initial_seen, initial);
    compare_state(actual.final, expected.final);
    check(actual.events.size() == expected.events.size(), "exact event count without empty-line events");
    if (actual.events.size() != expected.events.size())
        throw std::runtime_error("event count does not permit indexed comparisons");
    for (std::size_t index = 0; index < expected.events.size(); ++index) {
        check(actual.events[index].kind == expected.events[index].kind, "exact event kind order");
        compare_items(actual.events[index].items.data(), actual.events[index].items.size(),
                      expected.events[index].items.data(), expected.events[index].items.size());
    }
}

void compare_state(const GCodeStateTracker &actual, const GCodeStateTracker &expected)
{
    const raw_gcode_reader_state &left = actual.state();
    const raw_gcode_reader_state &right = expected.state();
    compare_items(left.items, left.item_count, right.items, right.item_count);
    const GCodeStateTracker snapshot(left);
    compare_items(snapshot.state().items, snapshot.state().item_count, right.items, right.item_count);
}

void compare_items(const Item *actual, std::size_t actual_count,
                   const Item *expected, std::size_t expected_count)
{
    check(actual_count == expected_count, "exact owned item count");
    if (actual_count != expected_count || (actual_count != 0 && (actual == nullptr || expected == nullptr)))
        throw std::runtime_error("item range does not permit safe comparisons");
    for (std::size_t index = 0; index < actual_count; ++index) {
        check(actual[index].type == expected[index].type, "exact item type order");
        check(actual[index].target == expected[index].target, "exact item target");
        check(actual[index].value == expected[index].value, "exact item double");
    }
}

void check(bool condition, const char *message)
{
    ++checks;
    if (!condition) {
        ++failures;
        std::printf("FAILED: %s\n", message);
    }
}
} // namespace
