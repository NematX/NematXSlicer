///|/ Copyright (c) NematXSlicer 2026 Durand Remi @supermerill
///|/
///|/ NematXSlicer is released under the terms of the AGPLv3 or higher

#include "NematXGCodeReader.hpp"

#include <algorithm>
#include <charconv>
#include <exception>
#include <string>
#include <vector>

#include "libslic3r/Api/plugin/cpp/ConfigViews.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"

#include "NematXFirmwareSession.hpp"
#include "NematXSyntax.hpp"

namespace nematx_firmware {

using namespace slic3r_api;
using namespace slic3r_api::GCodeGeneration;
using slic3r_api::GCodeGeneration::GCodeLineParser;

namespace {

// Dialect settings for one read: the axis letter of each tool and the fan
// scale. They come from the print when the reader runs during an export, or
// use the defaults (A, B; fans from 0 to 255) for a standalone file.
struct Dialect
{
    std::vector<char> axis_letters{ default_axis_letter(0), default_axis_letter(1) };
    bool fan_percentage = false;
};

// The reader object is shared and may run several reads at once, so the
// settings of the current read live on the reading thread. read_impl() sets
// them; consume_text_command(), called back on the same thread, reads them.
thread_local const Dialect *t_dialect = nullptr;

Dialect dialect_from(const run_ctx_gcode_reader &context)
{
    Dialect dialect;
    if (context.print == nullptr)
        return dialect;
    const Config config = Print(context.print).config();
    const size_t tools = std::max<size_t>(2, config.size("nozzle_diameter", 1));
    std::vector<std::string> axes;
    for (uint32_t tool = 0; tool < tools; ++tool)
        axes.push_back(config.vector_string_or_default(k_extruder_axis_key, tool, ""));
    dialect.axis_letters = resolve_axis_letters(axes, tools);
    dialect.fan_percentage = config.bool_or_default("fan_percentage", false);
    return dialect;
}

// Wraps the host's chunk source to remove the "N<k> " line numbers. Lines are
// kept (only their prefix is removed), so the line numbers reported to the host
// still match the file. A line split between two host chunks is kept until it
// is complete.
struct LineNumberFilter
{
    const run_ctx_gcode_reader *original = nullptr;
    std::string chunk;   // text lent to the reader base, valid until the next call
    std::string pending; // incomplete last line
    bool ended = false;

    void append_lines(std::string_view text)
    {
        size_t begin = 0;
        while (begin < text.size()) {
            const size_t end = text.find('\n', begin);
            if (end == std::string_view::npos) {
                pending.append(text.substr(begin));
                return;
            }
            pending.append(text.substr(begin, end - begin));
            chunk.append(strip_line_number(pending));
            chunk += '\n';
            pending.clear();
            begin = end + 1;
        }
    }

    static void next_chunk(void *opaque, raw_gcode_reader_chunk *out, raw_gcode_reader_result *result)
    {
        LineNumberFilter &filter = *static_cast<LineNumberFilter *>(opaque);
        try {
            filter.chunk.clear();
            while (filter.chunk.empty() && !filter.ended) {
                raw_gcode_reader_chunk input = {};
                raw_gcode_reader_result input_result = { RAW_GCODE_READER_STATUS_UNSET, nullptr };
                filter.original->get_next_chunk(filter.original->callback_context, &input, &input_result);
                if (input_result.status != RAW_GCODE_READER_STATUS_SUCCESS) {
                    *result = input_result;
                    return;
                }
                if (input.bytes == nullptr) {
                    filter.ended = true;
                    filter.chunk.append(strip_line_number(filter.pending));
                    filter.pending.clear();
                } else {
                    filter.append_lines(std::string_view(input.bytes, size_t(input.byte_count)));
                }
            }
            out->bytes = filter.chunk.empty() ? nullptr : filter.chunk.data();
            out->byte_count = filter.chunk.size();
            result->status = RAW_GCODE_READER_STATUS_SUCCESS;
        } catch (const std::exception &) {
            result->status = RAW_GCODE_READER_STATUS_ERROR;
            result->error_message = "The NematX reader could not prepare its input.";
        }
    }

    // Events and states go unchanged to the host.
    static void emit_event(void *opaque, const raw_gcode_reader_event *event, raw_gcode_reader_result *result)
    {
        const run_ctx_gcode_reader &host = *static_cast<LineNumberFilter *>(opaque)->original;
        host.emit_event(host.callback_context, event, result);
    }
    static void emit_final_state(void *opaque, const raw_gcode_reader_state *state, raw_gcode_reader_result *result)
    {
        const run_ctx_gcode_reader &host = *static_cast<LineNumberFilter *>(opaque)->original;
        host.emit_final_state(host.callback_context, state, result);
    }
    static void emit_probe(void *opaque, uint32_t confidence, raw_gcode_reader_result *result)
    {
        const run_ctx_gcode_reader &host = *static_cast<LineNumberFilter *>(opaque)->original;
        host.emit_probe(host.callback_context, confidence, result);
    }
};

std::optional<bool> known_flag(const GCodeStateTracker &state, uint32_t item)
{
    const std::optional<double> value = state.value(item);
    return value ? std::optional<bool>(*value != 0.0) : std::nullopt;
}

} // namespace

NematXGCodeReader::NematXGCodeReader(orchestrator_handle *orchestrator) : BuiltinGCodeReaderBase(orchestrator) {}

const char *NematXGCodeReader::id_impl() const noexcept { return k_reader_plugin_id; }
const char *NematXGCodeReader::name_impl() const noexcept { return "NematX G-code reader"; }
const char *NematXGCodeReader::description_impl() const noexcept
{
    return "Reads NematX G-code (per-extruder axes, '=' values, SET_POSITION) into machine-state events.";
}

// Recognize a NematX file from its first lines, for files opened in the viewer.
uint32_t NematXGCodeReader::probe_impl(std::string_view prefix) const
{
    if (prefix.find("[SET_POSITION POS=") != std::string_view::npos)
        return 90;
    if (prefix.find("M104=") != std::string_view::npos || prefix.find("M140=") != std::string_view::npos ||
        prefix.find("M106=") != std::string_view::npos)
        return 80;
    return 0;
}

void NematXGCodeReader::read_impl(const plugin_run_context &run_context, const run_ctx_gcode_reader &context,
                                  GCodeStateTracker &state) const
{
    const Dialect dialect = dialect_from(context);
    LineNumberFilter filter;
    filter.original = &context;
    run_ctx_gcode_reader filtered = context;
    filtered.callback_context = &filter;
    filtered.get_next_chunk = &LineNumberFilter::next_chunk;
    filtered.emit_event = &LineNumberFilter::emit_event;
    filtered.emit_final_state = &LineNumberFilter::emit_final_state;
    filtered.emit_probe = &LineNumberFilter::emit_probe;

    // Restore the previous settings even if reading fails.
    struct DialectScope
    {
        const Dialect *previous;
        explicit DialectScope(const Dialect &current) : previous(t_dialect) { t_dialect = &current; }
        ~DialectScope() { t_dialect = previous; }
    } scope(dialect);
    BuiltinGCodeReaderBase::read_impl(run_context, filtered, state);
}

bool NematXGCodeReader::consume_text_command(const plugin_run_context &run_context,
                                             const run_ctx_gcode_reader &context, GCodeStateTracker &state,
                                             const GCodeLineParser::GCodeLine &line) const
{
    static const Dialect default_dialect;
    const Dialect &dialect = t_dialect != nullptr ? *t_dialect : default_dialect;

    ReaderModes modes;
    modes.xyz_absolute = known_flag(state, RAW_GCODE_READER_ITEM_XYZ_ABSOLUTE);
    modes.e_absolute = known_flag(state, RAW_GCODE_READER_ITEM_E_ABSOLUTE);
    if (const std::optional<double> tool = state.value(RAW_GCODE_READER_ITEM_ACTIVE_TOOL))
        modes.active_tool = size_t(*tool);
    modes.fan_percentage = dialect.fan_percentage;

    const std::optional<std::vector<ReaderAction>> actions = translate_line(line.raw(), dialect.axis_letters, modes);
    if (!actions)
        return false; // standard G-code: the base reads it

    for (const ReaderAction &action : *actions) {
        switch (action.kind) {
        case ReaderAction::Kind::XyzMode:
            process_G90_G91(run_context, context, state, action.absolute ? 90 : 91);
            break;
        case ReaderAction::Kind::ExtrusionMode:
            process_M82_M83(run_context, context, state, action.absolute ? 82 : 83);
            break;
        case ReaderAction::Kind::SelectTool:
            process_T(run_context, context, state, uint32_t(action.tool));
            break;
        case ReaderAction::Kind::Standard:
            dispatch_standard(run_context, context, state, action.text);
            break;
        case ReaderAction::Kind::SetPosition:
            publish_items(run_context, context, state, RAW_GCODE_READER_EVENT_SET_POSITION,
                          { { RAW_GCODE_READER_ITEM_E, uint32_t(action.tool), action.value } });
            break;
        case ReaderAction::Kind::WaitTool:
        case ReaderAction::Kind::WaitBed: {
            // A bare wait reaches the target set by an earlier command.
            const bool bed = action.kind == ReaderAction::Kind::WaitBed;
            const uint32_t item = bed ? RAW_GCODE_READER_ITEM_BED_TEMPERATURE : RAW_GCODE_READER_ITEM_TOOL_TEMPERATURE;
            const uint32_t target = bed ? 0 : uint32_t(action.tool);
            if (const std::optional<double> temperature = state.value(item, target))
                publish_items(run_context, context, state, RAW_GCODE_READER_EVENT_WAIT_TEMPERATURE,
                              { { item, target, *temperature } });
            else
                publish_state_update(run_context, context, state);
            break;
        }
        case ReaderAction::Kind::Neutral:
            publish_state_update(run_context, context, state);
            break;
        case ReaderAction::Kind::Unknown:
            publish_unknown(run_context, context, state, RAW_GCODE_READER_DIAGNOSTIC_UNKNOWN_COMMAND);
            break;
        }
    }
    return true;
}

void NematXGCodeReader::dispatch_standard(const plugin_run_context &run_context,
                                          const run_ctx_gcode_reader &context, GCodeStateTracker &state,
                                          const std::string &text) const
{
    GCodeLineParser parser;
    parser.parse_line(text, [&](GCodeLineParser &, const GCodeLineParser::GCodeLine &parsed) {
        const std::string_view command = parsed.command();
        uint32_t number = 0;
        const char *digits = command.data() + 1;
        const std::from_chars_result result = std::from_chars(digits, command.data() + command.size(), number);
        if (command.size() < 2 || result.ec != std::errc() || result.ptr != command.data() + command.size()) {
            publish_unknown(run_context, context, state, RAW_GCODE_READER_DIAGNOSTIC_UNKNOWN_COMMAND);
            return;
        }
        const char letter = command.front();
        if (consume_numeric_command(run_context, context, state, parsed, letter, number))
            return;
        if (is_neutral_command(letter, number))
            publish_state_update(run_context, context, state);
        else if (const std::optional<raw_gcode_reader_marker_type> marker = marker_type(letter, number))
            publish_items(run_context, context, state, RAW_GCODE_READER_EVENT_MARKER,
                          { { RAW_GCODE_READER_ITEM_MARKER, 0, double(*marker) } });
        else
            publish_unknown(run_context, context, state, RAW_GCODE_READER_DIAGNOSTIC_UNKNOWN_COMMAND);
    });
}

// G4 (dwell) changes no tracked state.
bool NematXGCodeReader::is_neutral_command(char letter, uint32_t number) const
{
    return letter == 'G' && number == 4;
}

// Pause and color-change commands, as used by the NematX profiles (M601, M600).
std::optional<raw_gcode_reader_marker_type> NematXGCodeReader::marker_type(char letter, uint32_t number) const
{
    if (letter != 'M')
        return std::nullopt;
    if (number == 600)
        return RAW_GCODE_READER_MARKER_COLOR_CHANGE;
    if (number == 0 || number == 601 || number == 226)
        return RAW_GCODE_READER_MARKER_PAUSE_PRINT;
    return std::nullopt;
}

} // namespace nematx_firmware
