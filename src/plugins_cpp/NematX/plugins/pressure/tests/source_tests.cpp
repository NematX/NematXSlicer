///|/ Released under the GNU Affero General Public License, version 3 or later.

#include "PressureSource.hpp"

#include <cstdio>
#include <stdexcept>
#include <utility>

// Exercises LF framing and event association with explicit original bytes and
// manual Reader values. Public SDK apply validates every event on a temporary
// initial state; the final state is deliberately independent of event replay.
// No Reader service or parser runs. Literal ranges provide independent oracles,
// and checks remain active in Release. Copy/move destinations are verified only
// after their source storage has been reassigned or destroyed.
//
// Call tree:
//   main: run literal joins, framing, rejection and lifetime groups.
//     test_join / test_framing / test_rejections / test_lifetimes: local cases.
//       make_reader: validate manual deltas and create distinct private states.
//       verify: check byte/event partitions, exact deltas and raw state views.
//         compare_state / compare_items: inspect owned SDK scalar arrays.
//         check: count assertions, throwing on failure to prevent unsafe indexing.

// Runs all groups and reports exceptions as a nonzero exit in every build mode.
int main();

namespace {
using nematx_pressure::ReaderEvent;
using nematx_pressure::ReaderFragment;
using nematx_pressure::SourceFragment;
using nematx_pressure::SourceLine;
using nematx_pressure::index_fragment;
using slic3r_api::GCodeStateTracker;
using Item = raw_gcode_reader_event_item;

int checks = 0;
int cases = 0;
constexpr std::uint32_t private_type = RAW_GCODE_READER_ITEM_CUSTOM_BEGIN + 17;

// Checks multi-event lines, event-bearing comments and an untouched N prefix.
void test_join();
// Checks literal LF, CR and NUL byte ranges without interpreting their contents.
void test_framing();
// Checks SDK-valid but unjoinable deltas and incorrect final LF cursors.
void test_rejections();
// Checks destination copies and moves after original storage is gone.
void test_lifetimes();
// Validates event shapes through public apply, retaining an independent final.
ReaderFragment make_reader(std::uint32_t final_line, std::vector<ReaderEvent> events = {});
// Returns the five explicit deltas for the main six-line byte sample.
std::vector<ReaderEvent> main_events();
// Checks literal ranges, exact bytes/deltas/states and complete partitions.
void verify(const SourceFragment &source, const std::string &text,
            const ReaderFragment &reader, const std::vector<SourceLine> &lines);
// Checks exact state values and reconstructs a tracker from its borrowed raw view.
void compare_state(const GCodeStateTracker &actual, const GCodeStateTracker &expected);
// Checks ordered item type, target and exact double values after structural guards.
void compare_items(const Item *actual, std::size_t count, const Item *expected, std::size_t size);
// Counts each assertion and throws on failure so subsequent indexing stays safe.
void check(bool condition, const char *message);

const std::string main_text = "; intro\n\nG1 G90 X2 G91 A.1 ; print\r\n; gap\nM104=200\nG1 X3 A.2";
const std::vector<SourceLine> main_lines = {
    {0, 8, 0, 0}, {8, 1, 0, 0}, {9, 27, 0, 3},
    {36, 6, 3, 0}, {42, 9, 3, 1}, {51, 9, 4, 1}};
} // namespace

int main()
{
    try {
        test_join();
        test_framing();
        test_rejections();
        test_lifetimes();
    } catch (const std::exception &error) {
        std::printf("FAILED: %s\n", error.what());
        std::printf("pressure source: %d cases, %d checks, 1 failure\n", cases, checks);
        return 1;
    }
    std::printf("pressure source: %d cases, %d checks, 0 failures\n", cases, checks);
    return 0;
}

namespace {
void test_join()
{
    ++cases;
    const ReaderFragment reader = make_reader(5, main_events());
    const SourceFragment source = index_fragment(main_text, reader);
    verify(source, main_text, reader, main_lines);
    check(reader.initial.value(private_type, 3) == 12.25, "copied initial unchanged");
    check(reader.final.value(private_type, 3) == 99.75, "copied final unchanged");

    ++cases;
    const std::string layer = ";LAYER_CHANGE\n\n";
    const ReaderFragment layer_reader = make_reader(2, {
        {RAW_GCODE_READER_EVENT_STATE_UPDATE, {
            {RAW_GCODE_READER_ITEM_LAYER, 0, 0.0},
            {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 0.0}}}});
    verify(index_fragment(layer, layer_reader), layer, layer_reader,
           {{0, 14, 0, 1}, {14, 1, 1, 0}});

    ++cases;
    const std::string numbered = "N10 G1 X2 A.1";
    const ReaderFragment numbered_reader = make_reader(0, {
        {RAW_GCODE_READER_EVENT_LINEAR_MOVE, {
            {RAW_GCODE_READER_ITEM_X, 0, 2.0},
            {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 0.0},
            {RAW_GCODE_READER_ITEM_E, 0, 0.1}}}});
    verify(index_fragment(numbered, numbered_reader), numbered, numbered_reader, {{0, 13, 0, 1}});
}

void test_framing()
{
    struct Sample {
        std::string text;
        std::uint32_t final_line;
        std::vector<SourceLine> lines;
    };
    const std::vector<Sample> samples = {
        {"", 0, {}}, {"\n", 1, {{0, 1, 0, 0}}},
        {"\r\n", 1, {{0, 2, 0, 0}}},
        {"a\n", 1, {{0, 2, 0, 0}}},
        {"a\nb", 1, {{0, 2, 0, 0}, {2, 1, 0, 0}}},
        {"G1 X2", 0, {{0, 5, 0, 0}}},
        {"; unknown\n\n  ", 2, {{0, 10, 0, 0}, {10, 1, 0, 0}, {11, 2, 0, 0}}},
        {"a\rb\n", 1, {{0, 4, 0, 0}}},
        {std::string("a\0b\n", 4), 1, {{0, 4, 0, 0}}}};
    for (const Sample &sample : samples) {
        ++cases;
        const ReaderFragment reader = make_reader(sample.final_line);
        verify(index_fragment(sample.text, reader), sample.text, reader, sample.lines);
    }
}

void test_rejections()
{
    struct Rejection {
        std::string text;
        std::uint32_t final_line;
        std::vector<std::uint32_t> event_lines;
    };
    const std::vector<Rejection> inputs = {
        {"a\nb", 2, {}}, {"a\n", 0, {}},
        {"a", 0, {1}}, {"a", 0, {7}},
        {"a\nb\nc", 2, {2, 1}}, {"", 0, {0}}};
    for (const Rejection &input : inputs) {
        ++cases;
        std::vector<ReaderEvent> events;
        for (const std::uint32_t line : input.event_lines)
            events.push_back({RAW_GCODE_READER_EVENT_STATE_UPDATE, {
                {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, static_cast<double>(line)}}});
        const ReaderFragment reader = make_reader(input.final_line, std::move(events));
        const ReaderFragment before = reader;
        const std::string text_before = input.text;
        bool rejected = false;
        try {
            const SourceFragment unexpected = index_fragment(input.text, reader);
            (void)unexpected;
        } catch (const std::invalid_argument &) {
            rejected = true;
        }
        check(rejected, "invalid join throws invalid_argument");
        check(input.text == text_before, "failed copy call preserves text");
        compare_state(reader.initial, before.initial);
        compare_state(reader.final, before.final);
        check(reader.events.size() == before.events.size(), "failed copy preserves events");
        for (std::size_t i = 0; i < reader.events.size(); ++i) {
            check(reader.events[i].kind == before.events[i].kind, "failed copy preserves kind");
            compare_items(reader.events[i].items.data(), reader.events[i].items.size(),
                          before.events[i].items.data(), before.events[i].items.size());
        }
        const ReaderFragment good = make_reader(1);
        verify(index_fragment("a\nb", good), "a\nb", good, {{0, 2, 0, 0}, {2, 1, 0, 0}});
    }
}

void test_lifetimes()
{
    const ReaderFragment expected = make_reader(5, main_events());
    ++cases;
    SourceFragment copied;
    {
        SourceFragment original = index_fragment(main_text, expected);
        copied = original;
        original = {};
    }
    verify(copied, main_text, expected, main_lines);

    ++cases;
    SourceFragment moved;
    {
        std::string text = main_text;
        ReaderFragment reader = make_reader(5, main_events());
        SourceFragment original = index_fragment(std::move(text), std::move(reader));
        text.assign(400, 'x');
        reader = make_reader(0);
        moved = std::move(original);
        original = {};
    }
    verify(moved, main_text, expected, main_lines);
}

ReaderFragment make_reader(std::uint32_t final_line, std::vector<ReaderEvent> events)
{
    GCodeStateTracker initial;
    initial.set_reader_id(23);
    initial.set_custom_value(private_type, 3, 12.25);
    GCodeStateTracker validation = initial;
    for (const ReaderEvent &event : events) {
        const raw_gcode_reader_event raw{event.kind, event.items.data(),
                                        static_cast<std::uint32_t>(event.items.size())};
        validation.apply(raw);
    }
    GCodeStateTracker final = initial;
    final.set_line_number(final_line);
    final.set_custom_value(private_type, 3, 99.75);
    return {std::move(initial), std::move(events), std::move(final)};
}

std::vector<ReaderEvent> main_events()
{
    return {
        {RAW_GCODE_READER_EVENT_STATE_UPDATE, {
            {RAW_GCODE_READER_ITEM_ACTIVE_TOOL, 0, 0.0},
            {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 2.0}}},
        {RAW_GCODE_READER_EVENT_STATE_UPDATE, {
            {RAW_GCODE_READER_ITEM_XYZ_ABSOLUTE, 0, 1.0},
            {RAW_GCODE_READER_ITEM_E_ABSOLUTE, 0, 0.0},
            {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 2.0}}},
        {RAW_GCODE_READER_EVENT_LINEAR_MOVE, {
            {RAW_GCODE_READER_ITEM_X, 0, 2.0},
            {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 2.0},
            {RAW_GCODE_READER_ITEM_E, 0, 0.1}}},
        {RAW_GCODE_READER_EVENT_SET_TEMPERATURE, {
            {RAW_GCODE_READER_ITEM_TOOL_TEMPERATURE, 0, 200.0},
            {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 4.0}}},
        {RAW_GCODE_READER_EVENT_LINEAR_MOVE, {
            {RAW_GCODE_READER_ITEM_E, 0, 0.2},
            {RAW_GCODE_READER_ITEM_X, 0, 3.0},
            {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 5.0}}}};
}

void verify(const SourceFragment &source, const std::string &text,
            const ReaderFragment &reader, const std::vector<SourceLine> &lines)
{
    check(source.text == text, "all original bytes preserved");
    check(source.lines.size() == lines.size(), "literal record count");
    check(source.reader.events.size() == reader.events.size(), "event count unchanged");
    std::string joined;
    std::size_t byte_cursor = 0;
    std::size_t event_cursor = 0;
    for (std::size_t i = 0; i < lines.size(); ++i) {
        const SourceLine &line = source.lines[i];
        check(line.offset == lines[i].offset && line.length == lines[i].length &&
              line.first_event == lines[i].first_event && line.event_count == lines[i].event_count,
              "literal four-scalar range");
        check(line.offset == byte_cursor && line.offset <= source.text.size() &&
              line.length <= source.text.size() - line.offset, "contiguous bounded byte partition");
        check(line.first_event == event_cursor && event_cursor <= source.reader.events.size() &&
              line.event_count <= source.reader.events.size() - event_cursor,
              "contiguous bounded event partition");
        joined.append(source.text, line.offset, line.length);
        byte_cursor += line.length;
        event_cursor += line.event_count;
    }
    check(joined == text && byte_cursor == text.size(), "records cover every byte once");
    check(event_cursor == reader.events.size(), "ranges cover every event once");
    for (std::size_t i = 0; i < reader.events.size(); ++i) {
        check(source.reader.events[i].kind == reader.events[i].kind, "exact ordered event kinds");
        compare_items(source.reader.events[i].items.data(), source.reader.events[i].items.size(),
                      reader.events[i].items.data(), reader.events[i].items.size());
    }
    compare_state(source.reader.initial, reader.initial);
    compare_state(source.reader.final, reader.final);
}

void compare_state(const GCodeStateTracker &actual, const GCodeStateTracker &expected)
{
    const raw_gcode_reader_state &raw = actual.state();
    const raw_gcode_reader_state &reference = expected.state();
    compare_items(raw.items, raw.item_count, reference.items, reference.item_count);
    const GCodeStateTracker rebuilt(raw);
    compare_items(rebuilt.state().items, rebuilt.state().item_count, reference.items, reference.item_count);
}

void compare_items(const Item *actual, std::size_t count, const Item *expected, std::size_t size)
{
    check(count == size, "exact item count");
    check(size == 0 || (actual != nullptr && expected != nullptr), "readable item arrays");
    for (std::size_t i = 0; i < size; ++i)
        check(actual[i].type == expected[i].type && actual[i].target == expected[i].target &&
              actual[i].value == expected[i].value, "exact ordered type target and double");
}

void check(bool condition, const char *message)
{
    ++checks;
    if (!condition)
        throw std::runtime_error(message);
}
} // namespace
