
#include <catch2/catch.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <initializer_list>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "plugin_test_helpers.hpp"
#include "perimeter_test_helpers.hpp"
#include "test_data.hpp"

#include "libslic3r/Api/host/GCodeReaderService.hpp"
#include "libslic3r/Api/host/GCodeReaderResultBuilder.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/c/slic3r_extrusions.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_gcode_reader.h"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeReaderPlugin.hpp"
#include "libslic3r/Plugins/Ordering/ObjectInstanceCatalog.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeStateTracker.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/I18N.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"
#include "libslic3r/Plugins/GCode/Reader/BuiltinGCodeReaderBase.hpp"
#include "libslic3r/Plugins/GCode/Reader/KlipperGCodeReader.hpp"
#include "libslic3r/Plugins/GCode/Reader/Marlin2GCodeReader.hpp"
#include "libslic3r/Plugins/GCode/Reader/RepRapGCodeReader.hpp"
#include "libslic3r/Plugins/GCode/Reader/RepRapGCodeReaderState.hpp"
#include "libslic3r/TriangleMesh.hpp"

/* Tests for normalized G-code Reader events, built-in command handling,
   host viewer conversion, and legacy GCodeProcessor results. */

/*
G-code reader event tests
=========================

These tests exercise the plugin service rather than the legacy GCodeReader
class. Service tests use read_gcode() to verify the host-owned final state.
Tests which need individual actions invoke the built-in provider through its
real run payload and capture the same callbacks that the host implements.
Viewer material tests use Print settings to distinguish single-extruder
multi-material (SEMM) shared retraction debt from independent tool debts,
and compare role/tool volumes with the existing length and mass totals.
*/

namespace {

using namespace Slic3r;
using Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized;
using slic3r_api::GCodeGeneration::Reader::RepRapGCodeReaderState;
using slic3r_api::GCodeGeneration::Reader::RepRapToolState;

const char *const k_no_dependencies[] = {nullptr};

class ProbeTestReader final : public slic3r_api::GCodeReaderPluginBase
{
public:
    ProbeTestReader(orchestrator_handle *orchestrator, const char *id,
                    int32_t priority, bool fail_probe)
        : GCodeReaderPluginBase(orchestrator), m_id(id), m_priority(priority),
          m_fail_probe(fail_probe)
    {}

private:
    const char *id_impl() const noexcept override { return m_id; }
    const char *name_impl() const noexcept override { return m_id; }
    const char *description_impl() const noexcept override
    { return "Exercises deterministic G-code Reader probe routing."; }
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }
    int32_t priority_impl() const noexcept override { return m_priority; }
    uint32_t probe_impl(std::string_view prefix) const override
    {
        if (m_fail_probe && prefix.find(";probe-failure") != std::string_view::npos)
            throw std::runtime_error("Deliberate Reader probe failure.");
        if (m_fail_probe)
            return 0u;
        return prefix.find(";route-test") != std::string_view::npos ? 80u : 0u;
    }
    void read_impl(const plugin_run_context &run_context,
                   const run_ctx_gcode_reader &context,
                   slic3r_api::GCodeStateTracker &) const override
    {
        std::string_view chunk;
        while (next_chunk(run_context, context, chunk)) {}
    }

    const char *m_id;
    int32_t m_priority;
    bool m_fail_probe;
};

// Exercise both supported extension styles of the common dispatcher. G91 is
// replaced completely, while M220 delegates to the portable implementation
// before publishing an additional diagnostic event.
class CommonHandlerOverrideReader final :
    public slic3r_api::GCodeGeneration::Reader::BuiltinGCodeReaderBase
{
public:
    explicit CommonHandlerOverrideReader(orchestrator_handle *orchestrator) :
        BuiltinGCodeReaderBase(orchestrator)
    {}

protected:
    void process_G90_G91(const plugin_run_context &run_context,
                         const run_ctx_gcode_reader &context,
                         slic3r_api::GCodeStateTracker &state,
                         uint32_t number) const override
    {
        publish_items(run_context, context, state,
                      RAW_GCODE_READER_EVENT_STATE_UPDATE,
                      {{RAW_GCODE_READER_ITEM_SPEED_FACTOR, 0, double(number)}});
    }

    void process_M220(const plugin_run_context &run_context,
                      const run_ctx_gcode_reader &context,
                      slic3r_api::GCodeStateTracker &state,
                      const slic3r_api::GCodeGeneration::GCodeLineParser::GCodeLine &line) const override
    {
        BuiltinGCodeReaderBase::process_M220(
            run_context, context, state, line);
        publish_diagnostic(
            run_context, context, state,
            RAW_GCODE_READER_DIAGNOSTIC_UNSUPPORTED_STATE_EFFECT);
    }

private:
    const char *id_impl() const noexcept override
    { return "gcode.reader.test.common_handlers"; }
    const char *name_impl() const noexcept override
    { return "Common handler override test Reader"; }
    const char *description_impl() const noexcept override
    { return "Exercises virtual common G-code command handlers."; }
    uint32_t probe_impl(std::string_view) const override { return 0u; }
};

// Replace one RepRap-only command while inheriting the remaining RRF
// dispatcher. The test verifies that extending one handler does not require a
// copy of consume_numeric_command().
class RepRapHandlerOverrideReader final :
    public slic3r_api::GCodeGeneration::Reader::RepRapGCodeReader
{
public:
    explicit RepRapHandlerOverrideReader(orchestrator_handle *orchestrator) :
        RepRapGCodeReader(orchestrator)
    {}

protected:
    void process_G10(const plugin_run_context &run_context,
                     const run_ctx_gcode_reader &context,
                     slic3r_api::GCodeStateTracker &state,
                     RepRapGCodeReaderState &,
                     const slic3r_api::GCodeGeneration::GCodeLineParser::GCodeLine &) const override
    {
        publish_diagnostic(
            run_context, context, state,
            RAW_GCODE_READER_DIAGNOSTIC_UNSUPPORTED_STATE_EFFECT);
    }

private:
    const char *id_impl() const noexcept override
    { return "gcode.reader.test.reprap_handlers"; }
    const char *name_impl() const noexcept override
    { return "RepRap handler override test Reader"; }
    const char *description_impl() const noexcept override
    { return "Exercises virtual RepRapFirmware command handlers."; }
    uint32_t probe_impl(std::string_view) const override { return 0u; }
};

// Verify that a Marlin subclass inherits the common dispatcher and replaces
// one Marlin-only handler without overriding consume_numeric_command().
class MarlinHandlerOverrideReader final :
    public slic3r_api::GCodeGeneration::Reader::Marlin2GCodeReader
{
public:
    explicit MarlinHandlerOverrideReader(orchestrator_handle *orchestrator) :
        Marlin2GCodeReader(orchestrator)
    {}

protected:
    bool consume_numeric_command(
        const plugin_run_context &run_context,
        const run_ctx_gcode_reader &context,
        slic3r_api::GCodeStateTracker &state,
        const slic3r_api::GCodeGeneration::GCodeLineParser::GCodeLine &line,
        char letter,
        uint32_t number) const override
    {
        if (Marlin2GCodeReader::consume_numeric_command(
                run_context, context, state, line, letter, number))
            return true;
        if (letter != 'M' || number != 700)
            return false;
        process_M700(run_context, context, state);
        return true;
    }

    void process_M204(const plugin_run_context &run_context,
                      const run_ctx_gcode_reader &context,
                      slic3r_api::GCodeStateTracker &state,
                      const slic3r_api::GCodeGeneration::GCodeLineParser::GCodeLine &) const override
    {
        publish_items(run_context, context, state,
                      RAW_GCODE_READER_EVENT_STATE_UPDATE,
                      {{RAW_GCODE_READER_ITEM_ACCELERATION, 0, 204.0}});
    }

    void process_M700(const plugin_run_context &run_context,
                      const run_ctx_gcode_reader &context,
                      slic3r_api::GCodeStateTracker &state) const
    {
        publish_items(run_context, context, state,
                      RAW_GCODE_READER_EVENT_STATE_UPDATE,
                      {{RAW_GCODE_READER_ITEM_SPEED_FACTOR, 0, 700.0}});
    }

private:
    const char *id_impl() const noexcept override
    { return "gcode.reader.test.marlin_handlers"; }
    const char *name_impl() const noexcept override
    { return "Marlin handler override test Reader"; }
    const char *description_impl() const noexcept override
    { return "Exercises virtual Marlin command handlers."; }
    uint32_t probe_impl(std::string_view) const override { return 0u; }
};

// Verify both textual and numeric handler replacement through Klipper's
// chained dispatchers while ordinary common commands still reach the base.
class KlipperHandlerOverrideReader final :
    public slic3r_api::GCodeGeneration::Reader::KlipperGCodeReader
{
public:
    explicit KlipperHandlerOverrideReader(orchestrator_handle *orchestrator) :
        KlipperGCodeReader(orchestrator)
    {}

protected:
    bool consume_text_command(
        const plugin_run_context &run_context,
        const run_ctx_gcode_reader &context,
        slic3r_api::GCodeStateTracker &state,
        const slic3r_api::GCodeGeneration::GCodeLineParser::GCodeLine &line) const override
    {
        if (KlipperGCodeReader::consume_text_command(
                run_context, context, state, line))
            return true;
        if (line.command() != "TEST_EXTENSION")
            return false;
        process_TEST_EXTENSION(run_context, context, state);
        return true;
    }

    void process_PAUSE(const plugin_run_context &run_context,
                       const run_ctx_gcode_reader &context,
                       slic3r_api::GCodeStateTracker &state) const override
    {
        publish_diagnostic(
            run_context, context, state,
            RAW_GCODE_READER_DIAGNOSTIC_UNSUPPORTED_STATE_EFFECT);
    }

    void process_M204(const plugin_run_context &run_context,
                      const run_ctx_gcode_reader &context,
                      slic3r_api::GCodeStateTracker &state,
                      const slic3r_api::GCodeGeneration::GCodeLineParser::GCodeLine &) const override
    {
        publish_items(run_context, context, state,
                      RAW_GCODE_READER_EVENT_STATE_UPDATE,
                      {{RAW_GCODE_READER_ITEM_EXTRUSION_FACTOR, 0, 204.0}});
    }

    void process_TEST_EXTENSION(const plugin_run_context &run_context,
                                const run_ctx_gcode_reader &context,
                                slic3r_api::GCodeStateTracker &state) const
    {
        publish_items(run_context, context, state,
                      RAW_GCODE_READER_EVENT_STATE_UPDATE,
                      {{RAW_GCODE_READER_ITEM_SPEED_FACTOR, 0, 701.0}});
    }

private:
    const char *id_impl() const noexcept override
    { return "gcode.reader.test.klipper_handlers"; }
    const char *name_impl() const noexcept override
    { return "Klipper handler override test Reader"; }
    const char *description_impl() const noexcept override
    { return "Exercises virtual Klipper command handlers."; }
    uint32_t probe_impl(std::string_view) const override { return 0u; }
};

struct ObservedEvent
{
    uint32_t kind = 0;
    uint32_t line_number = UINT32_MAX;
    std::vector<raw_gcode_reader_event_item> items;
    std::optional<uint32_t> marker;
    std::optional<uint32_t> diagnostic;
};

struct ReaderProbe
{
    std::vector<std::string_view> chunks;
    size_t next_chunk = 0;
    size_t cancel_after_chunks = 0;
    size_t cancel_after_events = 0;
    std::vector<ObservedEvent> observed;
    std::optional<slic3r_api::GCodeStateTracker> final_state;
};

// Supply stable borrowed views in sequence. An empty output marks the end of
// input exactly as the production service does.
void supply_chunk(void *context,
                  raw_gcode_reader_chunk *chunk,
                  raw_gcode_reader_result *result);
// Copy the small pieces of each borrowed event needed by assertions before
// returning control to the provider.
void collect_event(void *context,
                   const raw_gcode_reader_event *event,
                   raw_gcode_reader_result *result);
// Copy the borrowed final state into a tracker owned by the test probe.
void collect_final_state(void *context,
                         const raw_gcode_reader_state *state,
                         raw_gcode_reader_result *result);
// Execute the registered provider through Orchestrator so the common plugin
// cancellation and exception boundaries remain part of each test.
raw_plugin_execution_status execute_default_reader(
    ReaderProbe &probe, const raw_gcode_reader_state &initial);
raw_plugin_execution_status execute_reader(
    const char *reader_id, ReaderProbe &probe,
    const raw_gcode_reader_state &initial, Print *print = nullptr);
uint32_t reprap_reader_state_id();
raw_gcode_reader_state initial_state(
    std::vector<raw_gcode_reader_event_item> &items,
    uint32_t reader_state_id = GCODE_READER_STATE_ID_PORTABLE);

void supply_chunk(void *context,
                  raw_gcode_reader_chunk *chunk,
                  raw_gcode_reader_result *result)
{
    ReaderProbe *probe = static_cast<ReaderProbe *>(context);
    *chunk = {};
    if (probe->next_chunk < probe->chunks.size()) {
        const std::string_view bytes = probe->chunks[probe->next_chunk++];
        chunk->bytes = bytes.data();
        chunk->byte_count = static_cast<uint64_t>(bytes.size());
        if (probe->cancel_after_chunks != 0 &&
            probe->next_chunk == probe->cancel_after_chunks)
            Orchestrator::instance().request_plugin_cancel();
    }
    result->status = RAW_GCODE_READER_STATUS_SUCCESS;
}

void collect_event(void *context,
                   const raw_gcode_reader_event *event,
                   raw_gcode_reader_result *result)
{
    ReaderProbe *probe = static_cast<ReaderProbe *>(context);
    uint32_t line_number = UINT32_MAX;
    std::optional<uint32_t> marker;
    std::optional<uint32_t> diagnostic;
    std::vector<raw_gcode_reader_event_item> items;
    items.reserve(event->item_count);
    for (uint32_t index = 0; index < event->item_count; ++index) {
        const raw_gcode_reader_event_item &item = event->items[index];
        items.push_back(item);
        if (item.type == RAW_GCODE_READER_ITEM_LINE_NUMBER)
            line_number = static_cast<uint32_t>(item.value);
        else if (item.type == RAW_GCODE_READER_ITEM_MARKER)
            marker = static_cast<uint32_t>(item.value);
        else if (item.type == RAW_GCODE_READER_ITEM_DIAGNOSTIC_TYPE)
            diagnostic = static_cast<uint32_t>(item.value);
    }
    probe->observed.push_back({event->kind, line_number, std::move(items), marker, diagnostic});
    if (probe->cancel_after_events != 0 &&
        probe->observed.size() == probe->cancel_after_events)
        Orchestrator::instance().request_plugin_cancel();
    result->status = RAW_GCODE_READER_STATUS_SUCCESS;
}

void collect_final_state(void *context,
                         const raw_gcode_reader_state *state,
                         raw_gcode_reader_result *result)
{
    ReaderProbe *probe = static_cast<ReaderProbe *>(context);
    probe->final_state.emplace(*state);
    result->status = RAW_GCODE_READER_STATUS_SUCCESS;
}

raw_plugin_execution_status execute_default_reader(
    ReaderProbe &probe, const raw_gcode_reader_state &initial)
{
    return execute_reader("gcode.reader.default", probe, initial);
}

raw_plugin_execution_status execute_reader(
    const char *reader_id, ReaderProbe &probe,
    const raw_gcode_reader_state &initial, Print *print)
{
    Orchestrator &orchestrator = Orchestrator::instance();
    Plugin *plugin = orchestrator.get_plugin(reader_id);
    if (plugin == nullptr)
        throw std::runtime_error(std::string("The G-code reader test provider is not registered: ") + reader_id);

    run_ctx_gcode_reader context = {};
    context.operation = RAW_GCODE_READER_OPERATION_READ;
    context.print = reinterpret_cast<const print_handle *>(print);
    context.initial_state = &initial;
    context.callback_context = &probe;
    context.get_next_chunk = supply_chunk;
    context.emit_event = collect_event;
    context.emit_final_state = collect_final_state;
    raw_plugin_run_payload payload = {&context};
    return orchestrator.execute_plugin(*plugin, print, &payload, 1);
}

uint32_t reprap_reader_state_id()
{
    const orchestrator_runtime_id id = orchestrator_register_runtime_id(
        reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()),
        GCODE_READER_STATE_ID_TYPE,
        slic3r_api::GCodeGeneration::Reader::REPRAP_READER_STATE_NAME);
    if (id == ORCHESTRATOR_RUNTIME_ID_INVALID)
        throw std::runtime_error("The RepRap Reader state identity could not be registered for the test.");
    return id;
}

raw_gcode_reader_state initial_state(
    std::vector<raw_gcode_reader_event_item> &items,
    uint32_t reader_state_id)
{
    items = {
        {RAW_GCODE_READER_ITEM_X, 0, 1.0},
        {RAW_GCODE_READER_ITEM_Y, 0, 2.0},
        {RAW_GCODE_READER_ITEM_Z, 0, 3.0},
        {RAW_GCODE_READER_ITEM_E, 0, 0.0},
        {RAW_GCODE_READER_ITEM_UNITS_MM, 0, 1.0},
        {RAW_GCODE_READER_ITEM_XYZ_ABSOLUTE, 0, 1.0},
        {RAW_GCODE_READER_ITEM_E_ABSOLUTE, 0, 1.0},
        {RAW_GCODE_READER_ITEM_ACTIVE_TOOL, 0, 0.0},
        {RAW_GCODE_READER_ITEM_READER_ID, 0, double(reader_state_id)},
        {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 0.0}
    };
    raw_gcode_reader_state state = {};
    state.items = items.data();
    state.item_count = static_cast<uint32_t>(items.size());
    return state;
}

// Apply the same event to the authoritative state and to the viewer builder,
// matching ViewerReaderInvocation's production callback ordering.
void consume_builder_event(
    GCodeReaderResultBuilder &builder,
    slic3r_api::GCodeStateTracker &state,
    uint32_t kind,
    std::initializer_list<raw_gcode_reader_event_item> items)
{
    const std::vector<raw_gcode_reader_event_item> owned_items(items);
    const raw_gcode_reader_event event = {
        kind, owned_items.data(), static_cast<uint32_t>(owned_items.size())
    };
    state.apply(event);
    builder.consume_event(event, state);
}

// Return one copied event value without exposing the borrowed ABI event after
// the callback has returned.
std::optional<double> observed_item_value(const ObservedEvent &event, uint32_t type)
{
    for (const raw_gcode_reader_event_item &item : event.items)
        if (item.type == type)
            return item.value;
    return std::nullopt;
}

size_t observed_event_count(const ReaderProbe &probe, uint32_t kind)
{
    size_t count = 0;
    for (const ObservedEvent &event : probe.observed)
        if (event.kind == kind)
            ++count;
    return count;
}

TEST_CASE("G-code reader publishes semantic actions and an owned final state",
          "[plugins][gcode][reader]")
{
    ensure_plugin_test_runtime_initialized();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    ReaderProbe probe;
    probe.chunks = {
        "G91\nG1 X2 E1\nG90\nG92 X10 E0\nT1\nM104 S215 T1\nM109 S215 T1\nM106 P2 S128\n"
    };
    REQUIRE(execute_default_reader(probe, initial) == RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(probe.final_state);
    const slic3r_api::GCodeStateTracker &state = *probe.final_state;

    REQUIRE(probe.observed.size() == 8);
    CHECK(probe.observed.front().kind == RAW_GCODE_READER_EVENT_STATE_UPDATE);
    CHECK(probe.observed[1].kind == RAW_GCODE_READER_EVENT_LINEAR_MOVE);
    CHECK(probe.observed[3].kind == RAW_GCODE_READER_EVENT_SET_POSITION);
    CHECK(probe.observed[5].kind == RAW_GCODE_READER_EVENT_SET_TEMPERATURE);
    CHECK(probe.observed[6].kind == RAW_GCODE_READER_EVENT_WAIT_TEMPERATURE);
    CHECK(probe.observed[1].line_number == 1u);
    REQUIRE(state.x());
    REQUIRE(state.y());
    REQUIRE(state.z());
    REQUIRE(state.e(0));
    CHECK(*state.x() == Approx(10.0));
    CHECK(*state.y() == Approx(2.0));
    CHECK(*state.z() == Approx(3.0));
    CHECK(*state.e(0) == Approx(0.0));
    CHECK(state.active_tool() == 1u);
    REQUIRE(state.tool_temperature(1));
    CHECK(*state.tool_temperature(1) == Approx(215.0));
    CHECK(state.known(RAW_GCODE_READER_ITEM_TOOL_TEMPERATURE_REACHED, 1));
    REQUIRE(state.fan_speed(2));
    CHECK(*state.fan_speed(2) == Approx(128.0 * 100.0 / 255.0));
    CHECK(state.line_number() == 8u);
}

TEST_CASE("Common G-code reader parses historical state comments",
          "[plugins][gcode][reader][markers]")
{
    ensure_plugin_test_runtime_initialized();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    ReaderProbe probe;
    probe.chunks = {
        ";LAYER_CHANGE\n"
        "; Z:0.400\n"
        ";HEIGHT:0.200\n"
        ";TYPE:External perimeter\n"
        ";WIDTH:0.450\n"
        "G1 X10 Y10 E1\n"
        ";LAYER_CHANGE\n"
        ";Z:0.600\n"
        ";HEIGHT:0.250\n"
        ";TYPE:Internal infill\n"
        ";WIDTH:0.500\n"
        "G1 X20 Y10 E2\n"
    };

    REQUIRE(execute_default_reader(probe, initial) == RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(probe.final_state);
    REQUIRE(probe.observed.size() == 12);

    const uint32_t expected_kinds[] = {
        RAW_GCODE_READER_EVENT_STATE_UPDATE,
        RAW_GCODE_READER_EVENT_STATE_UPDATE,
        RAW_GCODE_READER_EVENT_STATE_UPDATE,
        RAW_GCODE_READER_EVENT_STATE_UPDATE,
        RAW_GCODE_READER_EVENT_STATE_UPDATE,
        RAW_GCODE_READER_EVENT_LINEAR_MOVE,
        RAW_GCODE_READER_EVENT_STATE_UPDATE,
        RAW_GCODE_READER_EVENT_STATE_UPDATE,
        RAW_GCODE_READER_EVENT_STATE_UPDATE,
        RAW_GCODE_READER_EVENT_STATE_UPDATE,
        RAW_GCODE_READER_EVENT_STATE_UPDATE,
        RAW_GCODE_READER_EVENT_LINEAR_MOVE,
    };
    for (size_t index = 0; index < probe.observed.size(); ++index)
        CHECK(probe.observed[index].kind == expected_kinds[index]);

    REQUIRE(observed_item_value(
        probe.observed[0], RAW_GCODE_READER_ITEM_LAYER));
    CHECK(*observed_item_value(
        probe.observed[0], RAW_GCODE_READER_ITEM_LAYER) == Approx(0.0));
    CHECK_FALSE(observed_item_value(probe.observed[1], RAW_GCODE_READER_ITEM_Z));
    REQUIRE(observed_item_value(
        probe.observed[2], RAW_GCODE_READER_ITEM_HEIGHT));
    CHECK(*observed_item_value(
        probe.observed[2], RAW_GCODE_READER_ITEM_HEIGHT) == Approx(0.2));
    REQUIRE(observed_item_value(
        probe.observed[3], RAW_GCODE_READER_ITEM_EXTRUSION_ROLE));
    CHECK(*observed_item_value(
        probe.observed[3], RAW_GCODE_READER_ITEM_EXTRUSION_ROLE) ==
          Approx(double(RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER)));
    REQUIRE(observed_item_value(
        probe.observed[4], RAW_GCODE_READER_ITEM_WIDTH));
    CHECK(*observed_item_value(
        probe.observed[4], RAW_GCODE_READER_ITEM_WIDTH) == Approx(0.45));

    REQUIRE(observed_item_value(
        probe.observed[6], RAW_GCODE_READER_ITEM_LAYER));
    CHECK(*observed_item_value(
        probe.observed[6], RAW_GCODE_READER_ITEM_LAYER) == Approx(1.0));
    CHECK_FALSE(observed_item_value(probe.observed[7], RAW_GCODE_READER_ITEM_Z));
    REQUIRE(observed_item_value(
        probe.observed[8], RAW_GCODE_READER_ITEM_HEIGHT));
    CHECK(*observed_item_value(
        probe.observed[8], RAW_GCODE_READER_ITEM_HEIGHT) == Approx(0.25));
    REQUIRE(observed_item_value(
        probe.observed[9], RAW_GCODE_READER_ITEM_EXTRUSION_ROLE));
    CHECK(*observed_item_value(
        probe.observed[9], RAW_GCODE_READER_ITEM_EXTRUSION_ROLE) ==
          Approx(double(RAW_EXTRUSION_ROLE_INTERNAL_INFILL)));
    REQUIRE(observed_item_value(
        probe.observed[10], RAW_GCODE_READER_ITEM_WIDTH));
    CHECK(*observed_item_value(
        probe.observed[10], RAW_GCODE_READER_ITEM_WIDTH) == Approx(0.5));

    const slic3r_api::GCodeStateTracker &state = *probe.final_state;
    REQUIRE(state.value(RAW_GCODE_READER_ITEM_LAYER));
    CHECK(*state.value(RAW_GCODE_READER_ITEM_LAYER) == Approx(1.0));
    REQUIRE(state.value(RAW_GCODE_READER_ITEM_EXTRUSION_ROLE));
    CHECK(*state.value(RAW_GCODE_READER_ITEM_EXTRUSION_ROLE) ==
          Approx(double(RAW_EXTRUSION_ROLE_INTERNAL_INFILL)));
    REQUIRE(state.value(RAW_GCODE_READER_ITEM_WIDTH));
    CHECK(*state.value(RAW_GCODE_READER_ITEM_WIDTH) == Approx(0.5));
    REQUIRE(state.value(RAW_GCODE_READER_ITEM_HEIGHT));
    CHECK(*state.value(RAW_GCODE_READER_ITEM_HEIGHT) == Approx(0.25));
    REQUIRE(state.z());
    CHECK(*state.z() == Approx(3.0));
    REQUIRE(state.x());
    CHECK(*state.x() == Approx(20.0));
    CHECK(state.line_number() == 12u);
}

// Verify that the integrated Reader publishes the tag separately without changing layer transitions.
TEST_CASE("Common G-code reader publishes MAX_LAYER_HEIGHT without touching HEIGHT or LAYER_CHANGE",
          "[plugins][gcode][reader][markers][layer_height]")
{
    ensure_plugin_test_runtime_initialized();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    ReaderProbe probe;
    probe.chunks = {
        ";LAYER_CHANGE\n"
        ";Z:0.400\n"
        ";MAX_LAYER_HEIGHT:0.400\n"
        ";HEIGHT:0.200\n"
        "G1 X10 Y10 E1\n"
        ";LAYER_CHANGE\n"
        ";Z:0.600\n"
        ";HEIGHT:0.300\n"
    };

    REQUIRE(execute_default_reader(probe, initial) == RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(probe.final_state);
    REQUIRE(probe.observed.size() == 8);

    // The tag has its own state update and does not set HEIGHT.
    CHECK(probe.observed[2].kind == RAW_GCODE_READER_EVENT_STATE_UPDATE);
    REQUIRE(observed_item_value(probe.observed[2], RAW_GCODE_READER_ITEM_MAX_LAYER_HEIGHT));
    CHECK(*observed_item_value(probe.observed[2], RAW_GCODE_READER_ITEM_MAX_LAYER_HEIGHT) == Approx(0.4));
    CHECK_FALSE(observed_item_value(probe.observed[2], RAW_GCODE_READER_ITEM_HEIGHT));

    // A real HEIGHT tag still sets HEIGHT only.
    REQUIRE(observed_item_value(probe.observed[3], RAW_GCODE_READER_ITEM_HEIGHT));
    CHECK(*observed_item_value(probe.observed[3], RAW_GCODE_READER_ITEM_HEIGHT) == Approx(0.2));
    CHECK_FALSE(observed_item_value(probe.observed[3], RAW_GCODE_READER_ITEM_MAX_LAYER_HEIGHT));

    // The next LAYER_CHANGE only advances LAYER; clearing the value is left to
    // the consumers, which store it under the layer it was published for.
    REQUIRE(observed_item_value(probe.observed[5], RAW_GCODE_READER_ITEM_LAYER));
    CHECK_FALSE(observed_item_value(probe.observed[5], RAW_GCODE_READER_ITEM_MAX_LAYER_HEIGHT));

    REQUIRE(probe.final_state->value(RAW_GCODE_READER_ITEM_MAX_LAYER_HEIGHT));
    CHECK(*probe.final_state->value(RAW_GCODE_READER_ITEM_MAX_LAYER_HEIGHT) == Approx(0.4));
}

// Verify that malformed or nonpositive layer-height values become Reader diagnostics.
TEST_CASE("Common G-code reader reports an invalid MAX_LAYER_HEIGHT",
          "[plugins][gcode][reader][markers][layer_height][diagnostic]")
{
    ensure_plugin_test_runtime_initialized();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    ReaderProbe probe;
    probe.chunks = {
        ";MAX_LAYER_HEIGHT:abc\n"
        ";MAX_LAYER_HEIGHT:0\n"
        ";MAX_LAYER_HEIGHT:-0.2\n"
        ";MAX_LAYER_HEIGHT:nan\n"
        ";MAX_LAYER_HEIGHT:\n"
        ";MAX_LAYER_HEIGHT\n"
    };

    REQUIRE(execute_default_reader(probe, initial) == RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(probe.final_state);
    REQUIRE(probe.observed.size() == 6);
    for (const ObservedEvent &event : probe.observed) {
        CHECK(event.kind == RAW_GCODE_READER_EVENT_DIAGNOSTIC);
        REQUIRE(event.diagnostic);
        CHECK(*event.diagnostic == RAW_GCODE_READER_DIAGNOSTIC_INVALID_COMMAND_PARAMETERS);
        CHECK_FALSE(observed_item_value(event, RAW_GCODE_READER_ITEM_MAX_LAYER_HEIGHT));
    }
    CHECK_FALSE(probe.final_state->known(RAW_GCODE_READER_ITEM_MAX_LAYER_HEIGHT));
}

TEST_CASE("Common G-code reader matches LAYER_CHANGE exactly",
          "[plugins][gcode][reader][markers]")
{
    ensure_plugin_test_runtime_initialized();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    ReaderProbe probe;
    probe.chunks = {
        ";LAYER_CHANGE\n"
        ";LAYER_CHANGE_TRAVEL\n"
        ";LAYER_CHANGE_RETRACTION_START\n"
        ";LAYER_CHANGE_RETRACTION_END\n"
    };

    REQUIRE(execute_default_reader(probe, initial) == RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(probe.final_state);
    REQUIRE(probe.observed.size() == 1);
    CHECK(probe.observed.front().kind == RAW_GCODE_READER_EVENT_STATE_UPDATE);
    REQUIRE(observed_item_value(
        probe.observed.front(), RAW_GCODE_READER_ITEM_LAYER));
    CHECK(*observed_item_value(
        probe.observed.front(), RAW_GCODE_READER_ITEM_LAYER) == Approx(0.0));

    REQUIRE(probe.final_state->value(RAW_GCODE_READER_ITEM_LAYER));
    CHECK(*probe.final_state->value(RAW_GCODE_READER_ITEM_LAYER) == Approx(0.0));
}

TEST_CASE("Common G-code reader maps legacy TYPE names and matches them exactly",
          "[plugins][gcode][reader][markers]")
{
    ensure_plugin_test_runtime_initialized();
    struct TypeMarker
    {
        const char *name;
        raw_extrusion_role role;
    };
    const TypeMarker markers[] = {
        {"Unknown", RAW_EXTRUSION_ROLE_NONE},
        {"Perimeter", RAW_EXTRUSION_ROLE_INTERNAL_PERIMETER},
        {"External perimeter", RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER},
        {"Overhang perimeter", RAW_EXTRUSION_ROLE_OVERHANG_PERIMETER},
        {"Internal infill", RAW_EXTRUSION_ROLE_INTERNAL_INFILL},
        {"Solid infill", RAW_EXTRUSION_ROLE_SOLID_INFILL},
        {"Top solid infill", RAW_EXTRUSION_ROLE_TOP_SOLID_INFILL},
        {"Ironing", RAW_EXTRUSION_ROLE_IRONING_INFILL},
        {"Bridge infill", RAW_EXTRUSION_ROLE_BRIDGE_INFILL},
        {"Internal bridge infill", RAW_EXTRUSION_ROLE_INTERNAL_BRIDGE_INFILL},
        {"Thin wall", RAW_EXTRUSION_ROLE_THIN_WALL},
        {"Gap fill", RAW_EXTRUSION_ROLE_GAP_FILL},
        {"Skirt/Brim", RAW_EXTRUSION_ROLE_SKIRT},
        {"Support material", RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL},
        {"Support material interface", RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL_INTERFACE},
        {"Wipe tower", RAW_EXTRUSION_ROLE_WIPE_TOWER_DEFAULT},
        {"Milling", RAW_EXTRUSION_ROLE_MILLING},
        {"Custom", RAW_EXTRUSION_ROLE_NONE},
        {"Travel", RAW_EXTRUSION_ROLE_TRAVEL},
    };

    for (const TypeMarker &marker : markers) {
        DYNAMIC_SECTION("legacy TYPE marker: " << marker.name)
        {
            std::vector<raw_gcode_reader_event_item> initial_items;
            const raw_gcode_reader_state initial = initial_state(initial_items);
            ReaderProbe probe;
            const std::string source = std::string(";TYPE:") + marker.name + "\n";
            probe.chunks = {source};

            REQUIRE(execute_default_reader(probe, initial) == RAW_PLUGIN_EXECUTION_SUCCESS);
            REQUIRE(probe.final_state);
            REQUIRE(probe.observed.size() == 1);
            CHECK(probe.observed.front().kind == RAW_GCODE_READER_EVENT_STATE_UPDATE);
            REQUIRE(probe.final_state->value(RAW_GCODE_READER_ITEM_EXTRUSION_ROLE));
            CHECK(*probe.final_state->value(RAW_GCODE_READER_ITEM_EXTRUSION_ROLE) ==
                  Approx(double(marker.role)));
        }
    }

    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    ReaderProbe suffix_probe;
    suffix_probe.chunks = {
        ";TYPE:External perimeter\n"
        ";TYPE_SUFFIX:Internal infill\n"
    };
    REQUIRE(execute_default_reader(suffix_probe, initial) == RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(suffix_probe.final_state);
    REQUIRE(suffix_probe.observed.size() == 1);
    REQUIRE(suffix_probe.final_state->value(RAW_GCODE_READER_ITEM_EXTRUSION_ROLE));
    CHECK(*suffix_probe.final_state->value(RAW_GCODE_READER_ITEM_EXTRUSION_ROLE) ==
          Approx(double(RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER)));
}

TEST_CASE("Common G-code reader publishes historical comment markers",
          "[plugins][gcode][reader][markers]")
{
    ensure_plugin_test_runtime_initialized();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    ReaderProbe probe;
    probe.chunks = {
        ";TYPE:External perimeter\n"
        ";WIPE_START\n"
        "G1 X4 Y5 E1\n"
        ";WIPE_END\n"
        ";SEAM:4.000:5.000\n"
        ";COLOR_CHANGE,T0,#112233\n"
        ";PAUSE_PRINT\n"
        ";CUSTOM_GCODE\n"
        ";Z:0.400\n"
    };

    REQUIRE(execute_default_reader(probe, initial) == RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(probe.final_state);
    REQUIRE(probe.observed.size() == 9);

    CHECK(probe.observed[0].kind == RAW_GCODE_READER_EVENT_STATE_UPDATE);
    REQUIRE(observed_item_value(
        probe.observed[0], RAW_GCODE_READER_ITEM_EXTRUSION_ROLE));
    CHECK(*observed_item_value(
        probe.observed[0], RAW_GCODE_READER_ITEM_EXTRUSION_ROLE) ==
          Approx(double(RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER)));

    CHECK(probe.observed[1].kind == RAW_GCODE_READER_EVENT_STATE_UPDATE);
    REQUIRE(observed_item_value(
        probe.observed[1], RAW_GCODE_READER_ITEM_EXTRUSION_ROLE));
    CHECK(*observed_item_value(
        probe.observed[1], RAW_GCODE_READER_ITEM_EXTRUSION_ROLE) ==
          Approx(double(RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER |
                       RAW_EXTRUSION_ROLE_WIPE)));
    CHECK(probe.observed[2].kind == RAW_GCODE_READER_EVENT_LINEAR_MOVE);

    CHECK(probe.observed[3].kind == RAW_GCODE_READER_EVENT_STATE_UPDATE);
    REQUIRE(observed_item_value(
        probe.observed[3], RAW_GCODE_READER_ITEM_EXTRUSION_ROLE));
    CHECK(*observed_item_value(
        probe.observed[3], RAW_GCODE_READER_ITEM_EXTRUSION_ROLE) ==
          Approx(double(RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER)));

    const uint32_t expected_markers[] = {
        RAW_GCODE_READER_MARKER_SEAM,
        RAW_GCODE_READER_MARKER_COLOR_CHANGE,
        RAW_GCODE_READER_MARKER_PAUSE_PRINT,
        RAW_GCODE_READER_MARKER_CUSTOM_GCODE,
    };
    for (size_t index = 0; index < 4; ++index) {
        const ObservedEvent &event = probe.observed[index + 4];
        CHECK(event.kind == RAW_GCODE_READER_EVENT_MARKER);
        REQUIRE(event.marker);
        CHECK(*event.marker == expected_markers[index]);
        REQUIRE(observed_item_value(event, RAW_GCODE_READER_ITEM_MARKER));
        CHECK(*observed_item_value(event, RAW_GCODE_READER_ITEM_MARKER) ==
              Approx(double(expected_markers[index])));
    }
    CHECK(probe.observed[8].kind == RAW_GCODE_READER_EVENT_STATE_UPDATE);
    CHECK_FALSE(observed_item_value(probe.observed[8], RAW_GCODE_READER_ITEM_Z));

    const slic3r_api::GCodeStateTracker &state = *probe.final_state;
    REQUIRE(state.value(RAW_GCODE_READER_ITEM_EXTRUSION_ROLE));
    CHECK(*state.value(RAW_GCODE_READER_ITEM_EXTRUSION_ROLE) ==
          Approx(double(RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER)));
    CHECK_FALSE(state.known(RAW_GCODE_READER_ITEM_MARKER));
    REQUIRE(state.x());
    REQUIRE(state.y());
    REQUIRE(state.z());
    CHECK(*state.x() == Approx(4.0));
    CHECK(*state.y() == Approx(5.0));
    CHECK(*state.z() == Approx(3.0));
}

TEST_CASE("Common G-code reader accepts bare COLOR_CHANGE markers",
          "[plugins][gcode][reader][markers][color-change]")
{
    ensure_plugin_test_runtime_initialized();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    ReaderProbe probe;
    probe.chunks = {";COLOR_CHANGE\n"};

    REQUIRE(execute_default_reader(probe, initial) ==
            RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(probe.final_state);
    REQUIRE(probe.observed.size() == 1);
    const ObservedEvent &event = probe.observed.front();
    CHECK(event.kind == RAW_GCODE_READER_EVENT_MARKER);
    REQUIRE(event.marker);
    CHECK(*event.marker == RAW_GCODE_READER_MARKER_COLOR_CHANGE);
    CHECK_FALSE(event.diagnostic);
    CHECK_FALSE(probe.final_state->known(RAW_GCODE_READER_ITEM_MARKER));
}

TEST_CASE("Common G-code reader accepts canonical COLOR_CHANGE markers",
          "[plugins][gcode][reader][markers][color-change]")
{
    ensure_plugin_test_runtime_initialized();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    ReaderProbe probe;
    probe.chunks = {";COLOR_CHANGE,T0,#112233\n"};

    REQUIRE(execute_default_reader(probe, initial) ==
            RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(probe.final_state);
    REQUIRE(probe.observed.size() == 1);
    const ObservedEvent &event = probe.observed.front();
    CHECK(event.kind == RAW_GCODE_READER_EVENT_MARKER);
    REQUIRE(event.marker);
    CHECK(*event.marker == RAW_GCODE_READER_MARKER_COLOR_CHANGE);
    CHECK_FALSE(event.diagnostic);
    CHECK_FALSE(probe.final_state->known(RAW_GCODE_READER_ITEM_MARKER));
}

TEST_CASE("Common G-code reader rejects COLOR_CHANGE with malformed tool",
          "[plugins][gcode][reader][markers][color-change]")
{
    ensure_plugin_test_runtime_initialized();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    ReaderProbe probe;
    probe.chunks = {";COLOR_CHANGE,Tbad,#112233\n"};

    REQUIRE(execute_default_reader(probe, initial) ==
            RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(probe.final_state);
    REQUIRE(probe.observed.size() == 1);
    const ObservedEvent &event = probe.observed.front();
    CHECK(event.kind == RAW_GCODE_READER_EVENT_DIAGNOSTIC);
    REQUIRE(event.diagnostic);
    CHECK(*event.diagnostic ==
          RAW_GCODE_READER_DIAGNOSTIC_INVALID_COMMAND_PARAMETERS);
    CHECK_FALSE(event.marker);
    CHECK(observed_event_count(probe, RAW_GCODE_READER_EVENT_MARKER) == 0);
    CHECK(observed_event_count(probe, RAW_GCODE_READER_EVENT_INVALIDATE) == 0);
    CHECK_FALSE(probe.final_state->known(RAW_GCODE_READER_ITEM_MARKER));
    REQUIRE(probe.final_state->x());
    REQUIRE(probe.final_state->y());
    REQUIRE(probe.final_state->z());
    CHECK(*probe.final_state->x() == Approx(1.0));
    CHECK(*probe.final_state->y() == Approx(2.0));
    CHECK(*probe.final_state->z() == Approx(3.0));
}

TEST_CASE("Common G-code reader rejects COLOR_CHANGE with malformed RGB",
          "[plugins][gcode][reader][markers][color-change]")
{
    ensure_plugin_test_runtime_initialized();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    ReaderProbe probe;
    probe.chunks = {";COLOR_CHANGE,T0,#12zz33\n"};

    REQUIRE(execute_default_reader(probe, initial) ==
            RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(probe.final_state);
    REQUIRE(probe.observed.size() == 1);
    const ObservedEvent &event = probe.observed.front();
    CHECK(event.kind == RAW_GCODE_READER_EVENT_DIAGNOSTIC);
    REQUIRE(event.diagnostic);
    CHECK(*event.diagnostic ==
          RAW_GCODE_READER_DIAGNOSTIC_INVALID_COMMAND_PARAMETERS);
    CHECK_FALSE(event.marker);
    CHECK(observed_event_count(probe, RAW_GCODE_READER_EVENT_MARKER) == 0);
    CHECK(observed_event_count(probe, RAW_GCODE_READER_EVENT_INVALIDATE) == 0);
    CHECK_FALSE(probe.final_state->known(RAW_GCODE_READER_ITEM_MARKER));
    REQUIRE(probe.final_state->x());
    REQUIRE(probe.final_state->y());
    REQUIRE(probe.final_state->z());
    CHECK(*probe.final_state->x() == Approx(1.0));
    CHECK(*probe.final_state->y() == Approx(2.0));
    CHECK(*probe.final_state->z() == Approx(3.0));
}

TEST_CASE("Common G-code reader rejects COLOR_CHANGE with a trailing token",
          "[plugins][gcode][reader][markers][color-change]")
{
    ensure_plugin_test_runtime_initialized();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    ReaderProbe probe;
    probe.chunks = {";COLOR_CHANGE,T0,#112233,extra\n"};

    REQUIRE(execute_default_reader(probe, initial) ==
            RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(probe.final_state);
    REQUIRE(probe.observed.size() == 1);
    const ObservedEvent &event = probe.observed.front();
    CHECK(event.kind == RAW_GCODE_READER_EVENT_DIAGNOSTIC);
    REQUIRE(event.diagnostic);
    CHECK(*event.diagnostic ==
          RAW_GCODE_READER_DIAGNOSTIC_INVALID_COMMAND_PARAMETERS);
    CHECK_FALSE(event.marker);
    CHECK(observed_event_count(probe, RAW_GCODE_READER_EVENT_MARKER) == 0);
    CHECK(observed_event_count(probe, RAW_GCODE_READER_EVENT_INVALIDATE) == 0);
    CHECK_FALSE(probe.final_state->known(RAW_GCODE_READER_ITEM_MARKER));
    REQUIRE(probe.final_state->x());
    REQUIRE(probe.final_state->y());
    REQUIRE(probe.final_state->z());
    CHECK(*probe.final_state->x() == Approx(1.0));
    CHECK(*probe.final_state->y() == Approx(2.0));
    CHECK(*probe.final_state->z() == Approx(3.0));
}

TEST_CASE("Common G-code reader keeps valid state after malformed comments",
          "[plugins][gcode][reader][markers]")
{
    ensure_plugin_test_runtime_initialized();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    ReaderProbe probe;
    probe.chunks = {
        ";WIDTH:0.400\n"
        ";HEIGHT:0.200\n"
        ";TYPE:External perimeter\n"
        ";WIDTH:nan\n"
        ";WIDTH:inf\n"
        ";WIDTH:0\n"
        ";HEIGHT:-1\n"
        ";HEIGHT:0\n"
        ";HEIGHT:0.2mm\n"
        ";TYPE:Unknown role\n"
        ";Z:nan\n"
        ";SEAM:1.0:inf\n"
        "G1 X4 Y5 E1\n"
    };

    REQUIRE(execute_default_reader(probe, initial) == RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(probe.final_state);
    REQUIRE(probe.observed.size() == 13);
    CHECK(observed_event_count(probe, RAW_GCODE_READER_EVENT_DIAGNOSTIC) == 9);
    CHECK(observed_event_count(probe, RAW_GCODE_READER_EVENT_INVALIDATE) == 0);

    for (size_t index = 3; index < 12; ++index) {
        CHECK(probe.observed[index].kind == RAW_GCODE_READER_EVENT_DIAGNOSTIC);
        REQUIRE(probe.observed[index].diagnostic);
        CHECK(*probe.observed[index].diagnostic ==
              RAW_GCODE_READER_DIAGNOSTIC_INVALID_COMMAND_PARAMETERS);
    }
    CHECK(probe.observed.back().kind == RAW_GCODE_READER_EVENT_LINEAR_MOVE);

    const slic3r_api::GCodeStateTracker &state = *probe.final_state;
    REQUIRE(state.value(RAW_GCODE_READER_ITEM_WIDTH));
    CHECK(*state.value(RAW_GCODE_READER_ITEM_WIDTH) == Approx(0.4));
    REQUIRE(state.value(RAW_GCODE_READER_ITEM_HEIGHT));
    CHECK(*state.value(RAW_GCODE_READER_ITEM_HEIGHT) == Approx(0.2));
    REQUIRE(state.value(RAW_GCODE_READER_ITEM_EXTRUSION_ROLE));
    CHECK(*state.value(RAW_GCODE_READER_ITEM_EXTRUSION_ROLE) ==
          Approx(double(RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER)));
    REQUIRE(state.z());
    CHECK(*state.z() == Approx(3.0));
    REQUIRE(state.x());
    REQUIRE(state.y());
    CHECK(*state.x() == Approx(4.0));
    CHECK(*state.y() == Approx(5.0));
}

TEST_CASE("All built-in G-code readers publish identical common marker items",
          "[plugins][gcode][reader][markers][parity]")
{
    ensure_plugin_test_runtime_initialized();
    const char *const reader_ids[] = {
        "gcode.reader.default", "gcode.reader.marlin1",
        "gcode.reader.marlin2", "gcode.reader.prusa",
        "gcode.reader.sprinter", "gcode.reader.reprap",
        "gcode.reader.klipper"
    };
    const std::string source =
        ";LAYER_CHANGE\n"
        ";Z:0.400\n"
        ";HEIGHT:0.200\n"
        ";TYPE:External perimeter\n"
        ";WIDTH:0.450\n"
        ";WIPE_START\n"
        "G1 X4 Y5 E1\n"
        ";WIPE_END\n"
        ";SEAM:4.000:5.000\n"
        ";COLOR_CHANGE,T0,#112233\n"
        ";PAUSE_PRINT\n"
        ";CUSTOM_GCODE\n";
    const uint32_t reprap_state_id = reprap_reader_state_id();
    std::vector<raw_gcode_reader_event_item> default_initial_items;
    const raw_gcode_reader_state default_initial = initial_state(
        default_initial_items);
    ReaderProbe default_probe;
    default_probe.chunks = {source};
    REQUIRE(execute_default_reader(default_probe, default_initial) ==
            RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(default_probe.final_state);
    const std::vector<ObservedEvent> expected_events = default_probe.observed;

    // Catch2 re-enters the test case for each dynamic section, so keep this
    // count across those invocations and verify that all six comparisons ran.
    static size_t comparison_count = 0;
    constexpr size_t reader_count = sizeof(reader_ids) / sizeof(reader_ids[0]);
    for (size_t reader_index = 1; reader_index < reader_count; ++reader_index) {
        const char *reader_id = reader_ids[reader_index];
        DYNAMIC_SECTION("common marker parity: " << reader_id)
        {
            if (reader_index == 1)
                comparison_count = 0;

            std::vector<raw_gcode_reader_event_item> initial_items;
            const uint32_t state_id = std::string_view(reader_id) ==
                "gcode.reader.reprap" ? reprap_state_id :
                GCODE_READER_STATE_ID_PORTABLE;
            const raw_gcode_reader_state initial = initial_state(
                initial_items, state_id);
            ReaderProbe probe;
            probe.chunks = {source};

            REQUIRE(execute_reader(reader_id, probe, initial) ==
                    RAW_PLUGIN_EXECUTION_SUCCESS);
            REQUIRE(probe.final_state);
            ++comparison_count;

            REQUIRE(probe.observed.size() == expected_events.size());
            for (size_t event_index = 0;
                 event_index < expected_events.size(); ++event_index) {
                const ObservedEvent &expected = expected_events[event_index];
                const ObservedEvent &actual = probe.observed[event_index];
                CHECK(actual.kind == expected.kind);
                CHECK(actual.line_number == expected.line_number);
                REQUIRE(actual.items.size() == expected.items.size());
                for (size_t item_index = 0;
                     item_index < expected.items.size(); ++item_index) {
                    CHECK(actual.items[item_index].type ==
                          expected.items[item_index].type);
                    CHECK(actual.items[item_index].target ==
                          expected.items[item_index].target);
                    CHECK(actual.items[item_index].value ==
                          Approx(expected.items[item_index].value));
                }
            }

            if (reader_index + 1 == reader_count) {
                CHECK(comparison_count == 6);
            }
        }
    }
}

TEST_CASE("Legacy custom markers and matching commands produce one semantic action",
          "[plugins][gcode][reader][markers][custom-gcode]")
{
    ensure_plugin_test_runtime_initialized();
    const std::string source =
        ";COLOR_CHANGE,T0,#112233\n"
        "M600\n"
        ";PAUSE_PRINT\n"
        "M601\n"
        ";CUSTOM_GCODE\n"
        "M117 custom action\n";
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    ReaderProbe probe;
    probe.chunks = {source};

    REQUIRE(execute_reader("gcode.reader.prusa", probe, initial) ==
            RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(observed_event_count(probe, RAW_GCODE_READER_EVENT_MARKER) == 3);
    std::vector<uint32_t> markers;
    std::vector<uint32_t> marker_lines;
    for (const ObservedEvent &event : probe.observed) {
        if (event.kind != RAW_GCODE_READER_EVENT_MARKER)
            continue;
        REQUIRE(event.marker);
        markers.push_back(*event.marker);
        marker_lines.push_back(event.line_number);
    }
    CHECK(markers == std::vector<uint32_t>{
        RAW_GCODE_READER_MARKER_COLOR_CHANGE,
        RAW_GCODE_READER_MARKER_PAUSE_PRINT,
        RAW_GCODE_READER_MARKER_CUSTOM_GCODE
    });
    CHECK(marker_lines == std::vector<uint32_t>{0, 2, 4});

    GCodeReaderService service(Orchestrator::instance(), nullptr);
    const GCodeProcessorResult result = service.read_for_viewer(
        "gcode.reader.prusa", source);
    size_t color_changes = 0;
    size_t pauses = 0;
    size_t custom_codes = 0;
    for (const GCodeProcessorResult::MoveVertex &move : result.moves) {
        color_changes += move.type == EMoveType::Color_change ? 1 : 0;
        pauses += move.type == EMoveType::Pause_Print ? 1 : 0;
        custom_codes += move.type == EMoveType::Custom_GCode ? 1 : 0;
    }
    CHECK(color_changes == 1);
    CHECK(pauses == 1);
    CHECK(custom_codes == 1);

    ReaderProbe klipper_probe;
    klipper_probe.chunks = {";PAUSE_PRINT\n", "PAUSE\n"};
    REQUIRE(execute_reader("gcode.reader.klipper", klipper_probe, initial) ==
            RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(observed_event_count(
                klipper_probe, RAW_GCODE_READER_EVENT_MARKER) == 1);
    CHECK(klipper_probe.observed.front().marker ==
          RAW_GCODE_READER_MARKER_PAUSE_PRINT);
    CHECK(klipper_probe.observed.front().line_number == 0);
}

TEST_CASE("G-code reader invalidates stale state after an unknown command",
          "[plugins][gcode][reader]")
{
    ensure_plugin_test_runtime_initialized();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    ReaderProbe probe;
    probe.chunks = {"M900 K0.05\n"};
    REQUIRE(execute_default_reader(probe, initial) == RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(probe.final_state);
    const slic3r_api::GCodeStateTracker &state = *probe.final_state;

    CHECK_FALSE(state.x().has_value());
    CHECK_FALSE(state.active_tool().has_value());
    CHECK(state.line_number() == 1u);
    REQUIRE(probe.observed.size() == 2);
    CHECK(probe.observed[0].kind == RAW_GCODE_READER_EVENT_DIAGNOSTIC);
    CHECK(probe.observed[0].diagnostic ==
          RAW_GCODE_READER_DIAGNOSTIC_UNKNOWN_COMMAND);
    CHECK(probe.observed[1].kind == RAW_GCODE_READER_EVENT_INVALIDATE);
}

TEST_CASE("Built-in readers recognize homing and motor commands",
          "[plugins][gcode][reader][common-commands]")
{
    ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);

    SECTION("G28 without an axis resets every Cartesian coordinate") {
        const slic3r_api::GCodeStateTracker state = read_gcode(
            orchestrator, nullptr, "gcode.reader.default", initial, "G28\n");
        CHECK(state.x() == Approx(0.0));
        CHECK(state.y() == Approx(0.0));
        CHECK(state.z() == Approx(0.0));
        CHECK(state.e(0) == Approx(0.0));
        CHECK(state.active_tool() == 0u);
        CHECK(state.value(RAW_GCODE_READER_ITEM_XYZ_ABSOLUTE) == Approx(1.0));
    }

    SECTION("G28 only resets explicitly selected axes") {
        const slic3r_api::GCodeStateTracker state = read_gcode(
            orchestrator, nullptr, "gcode.reader.marlin2", initial,
            "G28 X0 Y0\n");
        CHECK(state.x() == Approx(0.0));
        CHECK(state.y() == Approx(0.0));
        CHECK(state.z() == Approx(3.0));

        const slic3r_api::GCodeStateTracker x_only = read_gcode(
            orchestrator, nullptr, "gcode.reader.marlin2", initial,
            "G28 X0\n");
        CHECK(x_only.x() == Approx(0.0));
        CHECK(x_only.y() == Approx(2.0));
        CHECK(x_only.z() == Approx(3.0));
    }

    SECTION("motor commands are neutral") {
        ReaderProbe probe;
        probe.chunks = {"M17\nM18\nM84\n"};
        REQUIRE(execute_default_reader(probe, initial) ==
                RAW_PLUGIN_EXECUTION_SUCCESS);
        REQUIRE(probe.final_state);
        CHECK(probe.final_state->x() == Approx(1.0));
        CHECK(probe.final_state->y() == Approx(2.0));
        CHECK(probe.final_state->z() == Approx(3.0));
        CHECK(probe.final_state->line_number() == 3u);
        REQUIRE(probe.observed.size() == 3);
        CHECK(probe.observed[0].line_number == 0u);
        CHECK(probe.observed[1].line_number == 1u);
        CHECK(probe.observed[2].line_number == 2u);
        for (const ObservedEvent &event : probe.observed)
            CHECK_FALSE(event.diagnostic);
    }

    SECTION("the viewer preserves the homing discontinuity") {
        GCodeReaderService service(orchestrator, nullptr);
        const GCodeProcessorResult result = service.read_for_viewer(
            "gcode.reader.marlin2",
            "G1 X10 Y20 Z30 F60\n"
            "G28 X0\n"
            "G1 X20 Y30 Z40 F60\n");
        REQUIRE(result.moves.size() == 2);
        CHECK(result.moves.front().position.x() == Approx(10.0));
        CHECK(result.moves.front().position.y() == Approx(20.0));
        CHECK(result.moves.front().position.z() == Approx(30.0));
        CHECK(result.moves.back().position.x() == Approx(20.0));
        CHECK(result.moves.back().position.y() == Approx(30.0));
        CHECK(result.moves.back().position.z() == Approx(40.0));
        CHECK(result.reader_diagnostics.empty());
    }
}

TEST_CASE("G-code reader detection recognizes firmware tags and falls back deterministically",
          "[plugins][gcode][reader][probe]")
{
    ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    CHECK(detect_gcode_reader(orchestrator, nullptr,
          "; generated for MaRlIn\nG1 X1\n") == "gcode.reader.marlin2");
    CHECK(detect_gcode_reader(orchestrator, nullptr,
          "; firmware: MARLIN 1\nG1 X1\n") == "gcode.reader.marlin1");
    CHECK(detect_gcode_reader(orchestrator, nullptr,
          "; firmware: marlin1\nG1 X1\n") == "gcode.reader.marlin1");
    CHECK(detect_gcode_reader(orchestrator, nullptr,
          "; firmware: MARLIN 2\nG1 X1\n") == "gcode.reader.marlin2");
    CHECK(detect_gcode_reader(orchestrator, nullptr,
          "; firmware: marlin2\nG1 X1\n") == "gcode.reader.marlin2");
    CHECK(detect_gcode_reader(orchestrator, nullptr,
          "; PrUsA firmware based on Marlin 2\nG1 X1\n") == "gcode.reader.prusa");
    CHECK(detect_gcode_reader(orchestrator, nullptr,
          "; SPRINTER based on Marlin\nG1 X1\n") == "gcode.reader.sprinter");
    CHECK(detect_gcode_reader(orchestrator, nullptr,
          "; RepRapFirmware\nG1 X1\n") == "gcode.reader.reprap");
    CHECK(detect_gcode_reader(orchestrator, nullptr,
          "; KLIPPER\nG1 X1\n") == "gcode.reader.klipper");
    // Equal confidence and priority use the service's lexical-id tie break.
    CHECK(detect_gcode_reader(orchestrator, nullptr,
          "; reprap and klipper\nG1 X1\n") == "gcode.reader.klipper");
    CHECK(detect_gcode_reader(orchestrator, nullptr,
          "; generated by SuperSlicer\nG1 X1\n") == "gcode.reader.default");
    CHECK(detect_gcode_reader(orchestrator, nullptr,
          "; unrelated generator\nG1 X1\n") == "gcode.reader.default");
}

TEST_CASE("Built-in G-code readers are active addressable services",
          "[plugins][gcode][reader][registration]")
{
    ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    const char *const ids[] = {
        "gcode.reader.default", "gcode.reader.marlin1",
        "gcode.reader.marlin2", "gcode.reader.prusa",
        "gcode.reader.sprinter", "gcode.reader.reprap",
        "gcode.reader.klipper"
    };
    for (const char *id : ids) {
        Plugin *plugin = orchestrator.get_plugin(id);
        REQUIRE(plugin != nullptr);
        CHECK(plugin->get_step() == GCODE_READER);
        CHECK(plugin->get_exclusive_group().empty());
        CHECK(orchestrator.is_plugin_active(plugin));
    }
    CHECK(orchestrator.get_plugin("gcode.reader.marlin") == nullptr);
}

TEST_CASE("Dialect G-code readers preserve and update representable state",
          "[plugins][gcode][reader][dialects]")
{
    ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    std::vector<raw_gcode_reader_event_item> reprap_initial_items;
    const raw_gcode_reader_state reprap_initial = initial_state(
        reprap_initial_items, reprap_reader_state_id());

    const slic3r_api::GCodeStateTracker marlin1 = read_gcode(
        orchestrator, nullptr, "gcode.reader.marlin1", initial,
        "M201 X1000 Y1000\nM204 S750\nM109 R180 T1\nM900 K0.04\nM600\n");
    REQUIRE(marlin1.x());
    CHECK(*marlin1.x() == Approx(1.0));
    REQUIRE(marlin1.value(RAW_GCODE_READER_ITEM_ACCELERATION));
    CHECK(*marlin1.value(RAW_GCODE_READER_ITEM_ACCELERATION) == Approx(750.0));
    REQUIRE(marlin1.tool_temperature(1));
    CHECK(*marlin1.tool_temperature(1) == Approx(180.0));
    CHECK(marlin1.known(RAW_GCODE_READER_ITEM_TOOL_TEMPERATURE_REACHED, 1));

    const slic3r_api::GCodeStateTracker marlin2_print = read_gcode(
        orchestrator, nullptr, "gcode.reader.marlin2", initial, "M204 P800\n");
    REQUIRE(marlin2_print.value(RAW_GCODE_READER_ITEM_ACCELERATION));
    CHECK(*marlin2_print.value(RAW_GCODE_READER_ITEM_ACCELERATION) == Approx(800.0));

    const slic3r_api::GCodeStateTracker marlin2_travel = read_gcode(
        orchestrator, nullptr, "gcode.reader.marlin2", initial, "M204 T1200\n");
    REQUIRE(marlin2_travel.value(RAW_GCODE_READER_ITEM_ACCELERATION));
    CHECK(*marlin2_travel.value(RAW_GCODE_READER_ITEM_ACCELERATION) == Approx(1200.0));

    const slic3r_api::GCodeStateTracker marlin2_registers = read_gcode(
        orchestrator, nullptr, "gcode.reader.marlin2", initial,
        "M204 P800 R500 T1200\n");
    CHECK_FALSE(marlin2_registers.value(RAW_GCODE_READER_ITEM_ACCELERATION));

    const slic3r_api::GCodeStateTracker marlin2_retract = read_gcode(
        orchestrator, nullptr, "gcode.reader.marlin2", initial, "M204 R500\n");
    CHECK(marlin2_retract.x() == Approx(1.0));
    CHECK_FALSE(marlin2_retract.value(RAW_GCODE_READER_ITEM_ACCELERATION));

    const slic3r_api::GCodeStateTracker prusa = read_gcode(
        orchestrator, nullptr, "gcode.reader.prusa", initial,
        "M204 P850\nM601\n");
    REQUIRE(prusa.value(RAW_GCODE_READER_ITEM_ACCELERATION));
    CHECK(*prusa.value(RAW_GCODE_READER_ITEM_ACCELERATION) == Approx(850.0));
    CHECK(prusa.x() == Approx(1.0));

    ReaderProbe prusa_marker_probe;
    prusa_marker_probe.chunks = {"M600\nM601\n"};
    REQUIRE(execute_reader("gcode.reader.prusa", prusa_marker_probe, initial) ==
            RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(prusa_marker_probe.observed.size() == 2);
    CHECK(prusa_marker_probe.observed[0].kind == RAW_GCODE_READER_EVENT_MARKER);
    CHECK(prusa_marker_probe.observed[0].marker ==
          RAW_GCODE_READER_MARKER_COLOR_CHANGE);
    CHECK(prusa_marker_probe.observed[1].kind == RAW_GCODE_READER_EVENT_MARKER);
    CHECK(prusa_marker_probe.observed[1].marker ==
          RAW_GCODE_READER_MARKER_PAUSE_PRINT);

    const slic3r_api::GCodeStateTracker sprinter = read_gcode(
        orchestrator, nullptr, "gcode.reader.sprinter", initial,
        "M204 P900\nM572 D0 S0.03\n");
    REQUIRE(sprinter.value(RAW_GCODE_READER_ITEM_ACCELERATION));
    CHECK(*sprinter.value(RAW_GCODE_READER_ITEM_ACCELERATION) == Approx(900.0));
    CHECK(sprinter.x() == Approx(1.0));

    const slic3r_api::GCodeStateTracker reprap = read_gcode(
        orchestrator, nullptr, "gcode.reader.reprap", reprap_initial,
        "G10 P1 S215\nM116 P1\nM106 P2 S0.5\nM566 X600\nM572 D1 S0.03\n");
    REQUIRE(reprap.tool_temperature(1));
    CHECK(*reprap.tool_temperature(1) == Approx(215.0));
    CHECK(reprap.known(RAW_GCODE_READER_ITEM_TOOL_TEMPERATURE_REACHED, 1));
    REQUIRE(reprap.fan_speed(2));
    CHECK(*reprap.fan_speed(2) == Approx(50.0));

    const slic3r_api::GCodeStateTracker klipper = read_gcode(
        orchestrator, nullptr, "gcode.reader.klipper", initial,
        "ACTIVATE_EXTRUDER EXTRUDER=extruder2\n"
        "SET_PRESSURE_ADVANCE ADVANCE=0.04 EXTRUDER=extruder2\n"
        "M204 P1200 T900\nPAUSE\n");
    CHECK(klipper.active_tool() == 2u);
    REQUIRE(klipper.value(RAW_GCODE_READER_ITEM_ACCELERATION));
    CHECK(*klipper.value(RAW_GCODE_READER_ITEM_ACCELERATION) == Approx(900.0));

    ReaderProbe klipper_pause_probe;
    klipper_pause_probe.chunks = {"PAUSE\n"};
    REQUIRE(execute_reader("gcode.reader.klipper", klipper_pause_probe, initial) ==
            RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(klipper_pause_probe.observed.size() == 1);
    CHECK(klipper_pause_probe.observed.front().marker ==
          RAW_GCODE_READER_MARKER_PAUSE_PRINT);
}

TEST_CASE("Klipper G-code reader resolves configured tool names conservatively",
          "[plugins][gcode][reader][dialects][klipper]")
{
    ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    Print print;
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.nozzle_diameter.set(std::vector<double>{0.4, 0.4});
    config.filament_diameter.set(std::vector<double>{1.75, 1.75});
    config.tool_name.set(std::vector<std::string>{"left_hotend", "right_hotend"});
    config.tool_name.set_enabled(true);

    const slic3r_api::GCodeStateTracker configured = read_gcode(
        orchestrator, &print, "gcode.reader.klipper", initial,
        "ACTIVATE_EXTRUDER EXTRUDER=right_hotend\n");
    CHECK(configured.active_tool() == 1u);

    const slic3r_api::GCodeStateTracker unavailable = read_gcode(
        orchestrator, nullptr, "gcode.reader.klipper", initial,
        "ACTIVATE_EXTRUDER EXTRUDER=right_hotend\n");
    CHECK_FALSE(unavailable.active_tool());

    ReaderProbe overflowing_probe;
    overflowing_probe.chunks = {
        "ACTIVATE_EXTRUDER EXTRUDER=extruder32768\n"};
    REQUIRE(execute_reader(
        "gcode.reader.klipper", overflowing_probe, initial) ==
        RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(overflowing_probe.final_state);
    CHECK_FALSE(overflowing_probe.final_state->active_tool());
    REQUIRE(overflowing_probe.observed.size() == 2);
    CHECK(overflowing_probe.observed[0].diagnostic ==
          RAW_GCODE_READER_DIAGNOSTIC_UNKNOWN_TOOL);

    ReaderProbe unknown_probe;
    unknown_probe.chunks = {"ACTIVATE_EXTRUDER EXTRUDER=extruder\n"};
    REQUIRE(execute_reader(
        "gcode.reader.klipper", unknown_probe, initial, &print) ==
        RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(unknown_probe.final_state);
    CHECK_FALSE(unknown_probe.final_state->active_tool());
    REQUIRE(unknown_probe.observed.size() == 2);
    CHECK(unknown_probe.observed[0].diagnostic ==
          RAW_GCODE_READER_DIAGNOSTIC_UNKNOWN_TOOL);

    Print default_names_print;
    PrintConfig &default_names_config = const_cast<PrintConfig &>(
        default_names_print.config());
    default_names_config.nozzle_diameter.set(std::vector<double>{0.4, 0.4});
    default_names_config.filament_diameter.set(std::vector<double>{1.75, 1.75});
    default_names_config.extrusion_multiplier.set(std::vector<double>{1.0, 1.0});
    default_names_config.tool_name.set(std::vector<std::string>{"", ""});
    default_names_config.tool_name.set_enabled(true);

    ReaderProbe default_heater_probe;
    default_heater_probe.chunks = {
        "SET_HEATER_TEMPERATURE HEATER=extruder1 TARGET=210\n"
        "TEMPERATURE_WAIT SENSOR=extruder1 MINIMUM=210\n"
    };
    REQUIRE(execute_reader(
        "gcode.reader.klipper", default_heater_probe, initial,
        &default_names_print) == RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(default_heater_probe.final_state);
    CHECK(default_heater_probe.final_state->tool_temperature(1) == Approx(210.0));
    CHECK(default_heater_probe.final_state->known(
        RAW_GCODE_READER_ITEM_TOOL_TEMPERATURE_REACHED, 1));

    ReaderProbe default_activate_probe;
    default_activate_probe.chunks = {
        "ACTIVATE_EXTRUDER EXTRUDER=extruder1\n"
    };
    REQUIRE(execute_reader(
        "gcode.reader.klipper", default_activate_probe, initial,
        &default_names_print) == RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(default_activate_probe.final_state);
    CHECK(default_activate_probe.final_state->active_tool() == 1u);

    config.tool_name.set(std::vector<std::string>{"duplicate", "duplicate"});
    ReaderProbe ambiguous_probe;
    ambiguous_probe.chunks = {"ACTIVATE_EXTRUDER EXTRUDER=duplicate\n"};
    REQUIRE(execute_reader(
        "gcode.reader.klipper", ambiguous_probe, initial, &print) ==
        RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(ambiguous_probe.final_state);
    CHECK_FALSE(ambiguous_probe.final_state->active_tool());
    REQUIRE(ambiguous_probe.observed.size() == 2);
    CHECK(ambiguous_probe.observed[0].diagnostic ==
          RAW_GCODE_READER_DIAGNOSTIC_AMBIGUOUS_TOOL);

    ReaderProbe invalid_parameters_probe;
    invalid_parameters_probe.chunks = {"ACTIVATE_EXTRUDER\n"};
    REQUIRE(execute_reader(
        "gcode.reader.klipper", invalid_parameters_probe, initial, &print) ==
        RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(invalid_parameters_probe.observed.size() == 2);
    CHECK(invalid_parameters_probe.observed[0].diagnostic ==
          RAW_GCODE_READER_DIAGNOSTIC_INVALID_COMMAND_PARAMETERS);

    ReaderProbe unsupported_effect_probe;
    unsupported_effect_probe.chunks = {"M201 X1200\n"};
    REQUIRE(execute_reader(
        "gcode.reader.klipper", unsupported_effect_probe, initial, &print) ==
        RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(unsupported_effect_probe.observed.size() == 1);
    CHECK(unsupported_effect_probe.observed[0].diagnostic ==
          RAW_GCODE_READER_DIAGNOSTIC_UNSUPPORTED_STATE_EFFECT);
}

TEST_CASE("Klipper heater commands round-trip through named Reader events",
          "[plugins][gcode][reader][dialects][klipper][heater]")
{
    ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    Print print;
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.nozzle_diameter.set(std::vector<double>{0.4, 0.4});
    config.filament_diameter.set(std::vector<double>{1.75, 1.75});
    config.tool_name.set(std::vector<std::string>{"left_hotend", "right_hotend"});
    config.tool_name.set_enabled(true);

    ReaderProbe round_trip;
    round_trip.chunks = {
        "SET_HEATER_TEMPERATURE HEATER=right_hotend TARGET=210\n"
        "TEMPERATURE_WAIT SENSOR=right_hotend MINIMUM=210\n"
        "SET_HEATER_TEMPERATURE HEATER=heater_bed TARGET=60\n"
        "TEMPERATURE_WAIT SENSOR=heater_bed MINIMUM=60\n"
    };
    REQUIRE(execute_reader("gcode.reader.klipper", round_trip, initial, &print) ==
            RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(round_trip.final_state);
    CHECK(round_trip.final_state->tool_temperature(1) == Approx(210.0));
    CHECK_FALSE(round_trip.final_state->tool_temperature(0));
    CHECK(round_trip.final_state->known(
        RAW_GCODE_READER_ITEM_TOOL_TEMPERATURE_REACHED, 1));
    CHECK(round_trip.final_state->value(RAW_GCODE_READER_ITEM_BED_TEMPERATURE) ==
          Approx(60.0));
    CHECK(round_trip.final_state->known(
        RAW_GCODE_READER_ITEM_BED_TEMPERATURE_REACHED));
    CHECK(observed_event_count(round_trip, RAW_GCODE_READER_EVENT_DIAGNOSTIC) == 0);
    CHECK(observed_event_count(round_trip, RAW_GCODE_READER_EVENT_INVALIDATE) == 0);

    ReaderProbe unknown;
    unknown.chunks = {
        "SET_HEATER_TEMPERATURE HEATER=unknown_hotend TARGET=210\n"
    };
    REQUIRE(execute_reader("gcode.reader.klipper", unknown, initial, &print) ==
            RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(unknown.observed.size() == 2);
    CHECK(unknown.observed.front().diagnostic ==
          RAW_GCODE_READER_DIAGNOSTIC_UNKNOWN_TOOL);
    CHECK(observed_event_count(unknown, RAW_GCODE_READER_EVENT_INVALIDATE) == 1);
}

TEST_CASE("Klipper temperature waits preserve targets and classify parameters",
          "[plugins][gcode][reader][dialects][klipper][heater]")
{
    ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    Print print;
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.nozzle_diameter.set(std::vector<double>{0.4, 0.4});
    config.filament_diameter.set(std::vector<double>{1.75, 1.75});
    config.tool_name.set(std::vector<std::string>{"left_hotend", "right_hotend"});
    config.tool_name.set_enabled(true);

    ReaderProbe different_threshold;
    different_threshold.chunks = {
        "SET_HEATER_TEMPERATURE HEATER=left_hotend TARGET=215\n"
        "TEMPERATURE_WAIT SENSOR=left_hotend MINIMUM=210\n"
    };
    REQUIRE(execute_reader(
        "gcode.reader.klipper", different_threshold, initial, &print) ==
            RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(different_threshold.final_state);
    CHECK(different_threshold.final_state->tool_temperature(0) == Approx(215.0));
    CHECK_FALSE(different_threshold.final_state->known(
        RAW_GCODE_READER_ITEM_TOOL_TEMPERATURE_REACHED, 0));
    CHECK(observed_event_count(
        different_threshold, RAW_GCODE_READER_EVENT_WAIT_TEMPERATURE) == 0);

    ReaderProbe proven_threshold;
    proven_threshold.chunks = {
        "SET_HEATER_TEMPERATURE HEATER=left_hotend TARGET=215\n"
        "TEMPERATURE_WAIT SENSOR=left_hotend MINIMUM=220\n"
    };
    REQUIRE(execute_reader(
        "gcode.reader.klipper", proven_threshold, initial, &print) ==
            RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(proven_threshold.final_state);
    CHECK(proven_threshold.final_state->tool_temperature(0) == Approx(215.0));
    CHECK(proven_threshold.final_state->known(
        RAW_GCODE_READER_ITEM_TOOL_TEMPERATURE_REACHED, 0));
    REQUIRE(proven_threshold.observed.size() == 2);
    CHECK(proven_threshold.observed[1].kind ==
          RAW_GCODE_READER_EVENT_WAIT_TEMPERATURE);
    CHECK(observed_item_value(
        proven_threshold.observed[1], RAW_GCODE_READER_ITEM_TOOL_TEMPERATURE) ==
          Approx(215.0));

    ReaderProbe proven_maximum;
    proven_maximum.chunks = {
        "SET_HEATER_TEMPERATURE HEATER=left_hotend TARGET=170\n"
        "TEMPERATURE_WAIT SENSOR=left_hotend MAXIMUM=170\n"
    };
    REQUIRE(execute_reader(
        "gcode.reader.klipper", proven_maximum, initial, &print) ==
            RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(proven_maximum.final_state);
    CHECK(proven_maximum.final_state->tool_temperature(0) == Approx(170.0));
    CHECK(proven_maximum.final_state->known(
        RAW_GCODE_READER_ITEM_TOOL_TEMPERATURE_REACHED, 0));
    REQUIRE(proven_maximum.observed.size() == 2);
    CHECK(proven_maximum.observed[1].kind ==
          RAW_GCODE_READER_EVENT_WAIT_TEMPERATURE);
    CHECK(observed_item_value(
        proven_maximum.observed[1], RAW_GCODE_READER_ITEM_TOOL_TEMPERATURE) ==
          Approx(170.0));

    ReaderProbe unproven_maximum;
    unproven_maximum.chunks = {
        "SET_HEATER_TEMPERATURE HEATER=left_hotend TARGET=170\n"
        "TEMPERATURE_WAIT SENSOR=left_hotend MAXIMUM=175\n"
    };
    REQUIRE(execute_reader(
        "gcode.reader.klipper", unproven_maximum, initial, &print) ==
            RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(unproven_maximum.final_state);
    CHECK(unproven_maximum.final_state->tool_temperature(0) == Approx(170.0));
    CHECK_FALSE(unproven_maximum.final_state->known(
        RAW_GCODE_READER_ITEM_TOOL_TEMPERATURE_REACHED, 0));
    CHECK(observed_event_count(
        unproven_maximum, RAW_GCODE_READER_EVENT_WAIT_TEMPERATURE) == 0);
    REQUIRE(unproven_maximum.observed.size() == 2);
    CHECK(unproven_maximum.observed[1].kind ==
          RAW_GCODE_READER_EVENT_STATE_UPDATE);

    ReaderProbe unknown_target;
    unknown_target.chunks = {
        "TEMPERATURE_WAIT SENSOR=right_hotend MINIMUM=200\n"
    };
    REQUIRE(execute_reader(
        "gcode.reader.klipper", unknown_target, initial, &print) ==
            RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(unknown_target.final_state);
    CHECK_FALSE(unknown_target.final_state->tool_temperature(1));
    CHECK_FALSE(unknown_target.final_state->known(
        RAW_GCODE_READER_ITEM_TOOL_TEMPERATURE_REACHED, 1));
    CHECK(observed_event_count(
        unknown_target, RAW_GCODE_READER_EVENT_DIAGNOSTIC) == 0);
    REQUIRE(unknown_target.observed.size() == 1);
    CHECK(unknown_target.observed.front().kind ==
          RAW_GCODE_READER_EVENT_STATE_UPDATE);

    ReaderProbe missing_sensor;
    missing_sensor.chunks = {"TEMPERATURE_WAIT MINIMUM=210\n"};
    REQUIRE(execute_reader(
        "gcode.reader.klipper", missing_sensor, initial, &print) ==
            RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(missing_sensor.observed.size() == 2);
    CHECK(missing_sensor.observed.front().diagnostic ==
          RAW_GCODE_READER_DIAGNOSTIC_INVALID_COMMAND_PARAMETERS);

    Print ambiguous_print;
    PrintConfig &ambiguous_config = const_cast<PrintConfig &>(
        ambiguous_print.config());
    ambiguous_config.nozzle_diameter.set(std::vector<double>{0.4, 0.4});
    ambiguous_config.filament_diameter.set(std::vector<double>{1.75, 1.75});
    ambiguous_config.tool_name.set(std::vector<std::string>{"duplicate", "duplicate"});
    ambiguous_config.tool_name.set_enabled(true);
    ReaderProbe ambiguous_sensor;
    ambiguous_sensor.chunks = {
        "TEMPERATURE_WAIT SENSOR=duplicate MINIMUM=210\n"
    };
    REQUIRE(execute_reader(
        "gcode.reader.klipper", ambiguous_sensor, initial,
        &ambiguous_print) == RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(ambiguous_sensor.observed.size() == 2);
    CHECK(ambiguous_sensor.observed.front().diagnostic ==
          RAW_GCODE_READER_DIAGNOSTIC_AMBIGUOUS_TOOL);

    ReaderProbe missing_threshold;
    missing_threshold.chunks = {
        "TEMPERATURE_WAIT SENSOR=left_hotend\n"
    };
    REQUIRE(execute_reader(
        "gcode.reader.klipper", missing_threshold, initial, &print) ==
            RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(missing_threshold.observed.size() == 2);
    CHECK(missing_threshold.observed.front().diagnostic ==
          RAW_GCODE_READER_DIAGNOSTIC_INVALID_COMMAND_PARAMETERS);

    ReaderProbe invalid_target;
    invalid_target.chunks = {
        "SET_HEATER_TEMPERATURE HEATER=left_hotend TARGET=not-a-number\n"
    };
    REQUIRE(execute_reader(
        "gcode.reader.klipper", invalid_target, initial, &print) ==
            RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(invalid_target.observed.size() == 2);
    CHECK(invalid_target.observed.front().diagnostic ==
          RAW_GCODE_READER_DIAGNOSTIC_INVALID_COMMAND_PARAMETERS);
}

TEST_CASE("Dialect G-code readers preserve state across recognized control commands",
          "[plugins][gcode][reader][dialects]")
{
    ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    std::vector<raw_gcode_reader_event_item> reprap_initial_items;
    const raw_gcode_reader_state reprap_initial = initial_state(
        reprap_initial_items, reprap_reader_state_id());

    const char *const marlin_family_ids[] = {
        "gcode.reader.marlin1", "gcode.reader.marlin2",
        "gcode.reader.prusa", "gcode.reader.sprinter"
    };
    for (const char *reader_id : marlin_family_ids) {
        const slic3r_api::GCodeStateTracker marlin = read_gcode(
            orchestrator, nullptr, reader_id, initial,
            "G4 P1\nG10\nG11\nM201 X1000\nM203 X300\nM205 X8\n"
            "M572 D0 S0.02\nM900 K0.02\nM906 T0 E800\nM0\nM600\nM601\nM226\n");
        CHECK(marlin.x() == Approx(1.0));
    }

    const slic3r_api::GCodeStateTracker reprap = read_gcode(
        orchestrator, nullptr, "gcode.reader.reprap", reprap_initial,
        "G4 P1\nM201 X1000\nM203 X18000\nM204 P800 T1000\n"
        "M566 X600\nM572 D0 S0.02\nM906 E800\nM600\nM226\n");
    CHECK(reprap.x() == Approx(1.0));

    const slic3r_api::GCodeStateTracker klipper = read_gcode(
        orchestrator, nullptr, "gcode.reader.klipper", initial,
        "G4 P1\nG10\nG11\nM201 X1000\nM203 X300\nM205 X8\nM906 E800\n");
    CHECK(klipper.x() == Approx(1.0));

    const slic3r_api::GCodeStateTracker malformed_reprap = read_gcode(
        orchestrator, nullptr, "gcode.reader.reprap", reprap_initial, "G10 P0\n");
    CHECK_FALSE(malformed_reprap.x());

    const slic3r_api::GCodeStateTracker unknown_marlin = read_gcode(
        orchestrator, nullptr, "gcode.reader.marlin2", initial, "M9999\n");
    CHECK_FALSE(unknown_marlin.x());
}

TEST_CASE("G-code reader detection uses confidence priority and stable identifiers",
          "[plugins][gcode][reader][probe]")
{
    ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();

    // All three fixtures return 80. Numeric priority first removes the
    // lower-priority fixture, then the lexical id selects .a before .b.
    GCodeReaderService service(orchestrator, nullptr);
    CHECK(service.detect(";route-test\nG1 X1\n") == "gcode.reader.test.a");
    // One broken probe must not prevent a healthy provider from winning.
    CHECK(service.detect(";probe-failure\n;route-test\n") == "gcode.reader.test.a");
}

TEST_CASE("G-code reader detection never exposes more than 256 KiB to probes",
          "[plugins][gcode][reader][probe]")
{
    ensure_plugin_test_runtime_initialized();
    std::string text(GCODE_READER_PROBE_PREFIX_SIZE, ' ');
    text += ";klipper\n";
    CHECK(detect_gcode_reader(Orchestrator::instance(), nullptr, text) ==
          DEFAULT_GCODE_READER_PLUGIN_ID);
}

TEST_CASE("G-code reader detection rejects an unavailable default fallback",
          "[plugins][gcode][reader][probe]")
{
    ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    REQUIRE(orchestrator.set_plugin_active(DEFAULT_GCODE_READER_PLUGIN_ID, false));
    CHECK_THROWS_WITH(detect_gcode_reader(orchestrator, nullptr, ";unknown\n"),
                      Catch::Matchers::Contains("default Reader is unavailable"));
    REQUIRE(orchestrator.set_plugin_active(DEFAULT_GCODE_READER_PLUGIN_ID, true));
}

TEST_CASE("G-code reader C++ plugin helper executes the requested Reader synchronously",
          "[plugins][gcode][reader][service]")
{
    ensure_plugin_test_runtime_initialized();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    slic3r_api::GCodeReaderService service(
        reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
    const slic3r_api::GCodeStateTracker state =
        service.read("", initial, "G1 X5\n");

    REQUIRE(state.x());
    CHECK(*state.x() == Approx(5.0));
}

TEST_CASE("Host G-code reader builds viewer movements and derived statistics",
          "[plugins][gcode][reader][viewer]")
{
    ensure_plugin_test_runtime_initialized();
    GCodeReaderService service(Orchestrator::instance(), nullptr);
    const std::string gcode =
        "M104 S210\n"
        "M106 S255\n"
        "G1 X60 F600\n"
        "G1 X120 E10\n";
    const GCodeProcessorResult result = service.read_for_viewer(
        "gcode.reader.default", gcode);

    REQUIRE(result.object_names.empty());
    REQUIRE(result.lines_ends.size() == 1);
    REQUIRE(result.lines_ends.front().size() == 4);
    CHECK(result.lines_ends.front().back() == gcode.size());
    REQUIRE(result.moves.size() == 2);
    CHECK(result.moves[0].type == EMoveType::Travel);
    CHECK(result.moves[0].gcode_id == 3u);
    CHECK(result.moves[0].position.x() == Approx(60.0));
    CHECK(result.moves[0].feedrate == Approx(10.0));
    CHECK(result.moves[0].fan_speed == Approx(100.0));
    CHECK(result.moves[0].temperature == Approx(210.0));
    CHECK(result.moves[0].move_time == Approx(6.0));
    CHECK(result.moves[1].type == EMoveType::Extrude);
    CHECK(result.moves[1].gcode_id == 4u);
    CHECK(result.moves[1].position.x() == Approx(120.0));
    CHECK(result.moves[1].delta_extruder == Approx(10.0));
    CHECK(result.moves[1].move_time == Approx(12.0));

    const double filament_area = PI * std::pow(1.75 * 0.5, 2.0);
    REQUIRE(result.print_statistics.volumes_per_extruder.count(0) == 1);
    CHECK(result.print_statistics.volumes_per_extruder.at(0) ==
          Approx(filament_area * 10.0));
    CHECK(result.print_statistics.modes[0].time == Approx(12.0));
    CHECK(result.print_statistics.modes[0].travel_time == Approx(6.0));
}

TEST_CASE("G92 placement does not remove viewer movements",
          "[plugins][gcode][reader][viewer][g92]")
{
    ensure_plugin_test_runtime_initialized();
    GCodeReaderService service(Orchestrator::instance(), nullptr);
    const std::string reset_before_travel =
        "G92 E0\n"
        "G1 X10 Y10 F600\n"
        "G1 X20 Y10 E1 F600\n"
        "G1 X30 Y10 E2 F600\n";
    const std::string reset_after_travel =
        "G1 X10 Y10 F600\n"
        "G92 E0\n"
        "G1 X20 Y10 E1 F600\n"
        "G1 X30 Y10 E2 F600\n";

    const std::vector<std::string> reader_ids = {
        "gcode.reader.default", "gcode.reader.marlin1"
    };
    for (const std::string &reader_id : reader_ids) {
        CAPTURE(reader_id);
        const GCodeProcessorResult before = service.read_for_viewer(
            reader_id, reset_before_travel);
        const GCodeProcessorResult after = service.read_for_viewer(
            reader_id, reset_after_travel);

        REQUIRE(before.moves.size() == 3);
        REQUIRE(after.moves.size() == before.moves.size());
        for (size_t move_idx = 0; move_idx < before.moves.size(); ++move_idx) {
            CAPTURE(move_idx);
            const GCodeProcessorResult::MoveVertex &lhs = before.moves[move_idx];
            const GCodeProcessorResult::MoveVertex &rhs = after.moves[move_idx];
            CHECK(lhs.type == rhs.type);
            CHECK(lhs.position.x() == Approx(rhs.position.x()));
            CHECK(lhs.position.y() == Approx(rhs.position.y()));
            CHECK(lhs.position.z() == Approx(rhs.position.z()));
            CHECK(lhs.delta_extruder == Approx(rhs.delta_extruder));
            CHECK(lhs.feedrate == Approx(rhs.feedrate));
            CHECK(lhs.mm3_per_mm == Approx(rhs.mm3_per_mm));
        }
        CHECK(before.moves[0].type == EMoveType::Travel);
        CHECK(before.moves[1].type == EMoveType::Extrude);
        CHECK(before.moves[2].type == EMoveType::Extrude);
    }
}

TEST_CASE("G92 preserves the first extrusion in absolute and relative E modes",
          "[plugins][gcode][reader][viewer][g92]")
{
    ensure_plugin_test_runtime_initialized();
    GCodeReaderService service(Orchestrator::instance(), nullptr);
    const std::vector<std::string> reader_ids = {
        "gcode.reader.default", "gcode.reader.marlin1"
    };
    for (const std::string &reader_id : reader_ids) {
        CAPTURE(reader_id);
        const GCodeProcessorResult absolute = service.read_for_viewer(
            reader_id,
            "M82\n"
            "G0 X33.878 Y40.185 Z0.3 F7800\n"
            "G92 E0\n"
            "G1 X29.878 E1.874 F1800\n"
            "G1 X28.438 Y40.01 E0.06796\n");
        const GCodeProcessorResult relative = service.read_for_viewer(
            reader_id,
            "M83\n"
            "G0 X33.878 Y40.185 Z0.3 F7800\n"
            "G92 E0\n"
            "G1 X29.878 E1.874 F1800\n"
            "G1 X28.438 Y40.01 E0.06796\n");

        REQUIRE(absolute.moves.size() == 3);
        REQUIRE(relative.moves.size() == 3);
        CHECK(absolute.moves[0].type == EMoveType::Travel);
        CHECK(absolute.moves[1].type == EMoveType::Extrude);
        CHECK(absolute.moves[1].delta_extruder == Approx(1.874));
        CHECK(absolute.moves[1].mm3_per_mm > 0.0f);
        CHECK(absolute.moves[2].type == EMoveType::Travel);
        CHECK(absolute.moves[2].delta_extruder == Approx(0.06796 - 1.874));
        CHECK(relative.moves[0].type == EMoveType::Travel);
        CHECK(relative.moves[1].type == EMoveType::Extrude);
        CHECK(relative.moves[1].delta_extruder == Approx(1.874));
        CHECK(relative.moves[1].mm3_per_mm > 0.0f);
        CHECK(relative.moves[2].type == EMoveType::Extrude);
        CHECK(relative.moves[2].delta_extruder == Approx(0.06796));
        CHECK(relative.moves[2].mm3_per_mm > 0.0f);
    }
}

// Tool-selection resets establish a fresh E origin without adding viewer movement or losing prime.
TEST_CASE("Built-in G-code readers preserve extrusion after tool selection and G92",
          "[plugins][gcode][reader][viewer][toolchange-reset]")
{
    ensure_plugin_test_runtime_initialized();
    const bool semm = GENERATE(false, true);
    const char *reader_id = GENERATE("gcode.reader.default", "gcode.reader.marlin1",
        "gcode.reader.marlin2", "gcode.reader.prusa", "gcode.reader.sprinter", "gcode.reader.reprap", "gcode.reader.klipper");
    CAPTURE(semm, reader_id);
    Print print;
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.single_extruder_multi_material.value = semm;
    config.nozzle_diameter.set(std::vector<double>{0.4, 0.4});
    config.filament_diameter.set(std::vector<double>{1.75, 1.75});
    config.tool_name.set(std::vector<std::string>{"extruder", "extruder1"});
    config.tool_name.set_enabled(true);
    Printing::PrintingPlan &plan = print.mutable_printing_plan();
    PluginStorage storage;
    slic3r_api::Ordering::ObjectInstanceCatalogPlugin::populate_object_instance_catalog(
        reinterpret_cast<const print_handle *>(&print),
        reinterpret_cast<printing_plan_handle *>(&plan),
        reinterpret_cast<storage_handle *>(&storage));
    const bool named_tools = std::string(reader_id) == "gcode.reader.klipper" && !semm;
    const std::string select_a = named_tools ? "ACTIVATE_EXTRUDER EXTRUDER=extruder\n" : "T0\n";
    const std::string select_b = named_tools ? "ACTIVATE_EXTRUDER EXTRUDER=extruder1\n" : "T1\n";
    GCodeReaderService service(Orchestrator::instance(), &print);
    const GCodeProcessorResult result = service.read_for_viewer(reader_id,
        "M82\nG90\n" + select_a + "G92 E0\nG1 X1 Y0 Z.2 E1.25 F1200\n" +
        select_b + "G92 E0\nG1 X2 E2.5\n" + select_a + "G92 E0\nG1 X3 E.75\n");
    std::vector<double> extrusion_deltas;
    std::vector<uint32_t> extrusion_tools;
    for (const GCodeProcessorResult::MoveVertex &move : result.moves) {
        if (move.internal_only || move.type != EMoveType::Extrude)
            continue;
        extrusion_deltas.push_back(move.delta_extruder);
        extrusion_tools.push_back(move.extruder_id);
    }
    REQUIRE(extrusion_deltas.size() == 3);
    CHECK(extrusion_deltas[0] == Approx(1.25));
    CHECK(extrusion_deltas[1] == Approx(2.5));
    CHECK(extrusion_deltas[2] == Approx(0.75));
    CHECK(extrusion_tools == std::vector<uint32_t>{0, 1, 0});
}

TEST_CASE("Host G-code reader seeds tools and viewer metadata from Print settings",
          "[plugins][gcode][reader][viewer]")
{
    ensure_plugin_test_runtime_initialized();
    Print print;
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.nozzle_diameter.set(std::vector<double>{0.4, 0.6});
    config.filament_diameter.set(std::vector<double>{1.75, 2.85});
    config.filament_density.set(std::vector<double>{1.1, 1.3});
    config.filament_cost.set(std::vector<double>{20.0, 30.0});
    config.extruder_offset.deserialize("0x0,2x3", false);
    config.extruder_colour.set(std::vector<std::string>{"#112233", "#445566"});
    config.filament_colour.set(std::vector<std::string>{"#778899", "#aabbcc"});

    Printing::PrintingPlan &plan = print.mutable_printing_plan();
    PluginStorage storage;
    slic3r_api::Ordering::ObjectInstanceCatalogPlugin::populate_object_instance_catalog(
        reinterpret_cast<const print_handle *>(&print),
        reinterpret_cast<printing_plan_handle *>(&plan),
        reinterpret_cast<storage_handle *>(&storage));

    GCodeReaderService service(Orchestrator::instance(), &print);
    const slic3r_api::GCodeStateTracker initial =
        service.initial_state("gcode.reader.marlin2");
    CHECK(initial.e(0) == Approx(0.0));
    CHECK(initial.e(1) == Approx(0.0));
    CHECK(initial.value(RAW_GCODE_READER_ITEM_TOOL_FILAMENT_DIAMETER, 1) ==
          Approx(2.85));
    CHECK(initial.value(RAW_GCODE_READER_ITEM_TOOL_OFFSET_X, 1) == Approx(2.0));
    CHECK(initial.value(RAW_GCODE_READER_ITEM_TOOL_OFFSET_Y, 1) == Approx(3.0));
    CHECK(initial.value(RAW_GCODE_READER_ITEM_FAN_SPEED, 1) == Approx(0.0));

    const GCodeProcessorResult result = service.read_for_viewer(
        "gcode.reader.marlin2", "T1\nM106 P1 S128\nG1 X10 F600\n");
    CHECK(result.extruders_count == 2);
    CHECK(result.extruder_colors ==
          std::vector<std::string>{"#112233", "#445566"});
    CHECK(result.filament_colors ==
          std::vector<std::string>{"#778899", "#aabbcc"});
    REQUIRE(result.moves.size() == 2);
    CHECK(result.moves[0].type == EMoveType::Tool_change);
    CHECK(result.moves[1].extruder_id == 1u);
    CHECK(result.moves[1].position.x() == Approx(12.0));
    CHECK(result.moves[1].position.y() == Approx(3.0));
    CHECK(result.moves[1].fan_speed == Approx(128.0 / 255.0 * 100.0));
}

TEST_CASE("Host G-code reader initializes layer areas from final plan islands",
          "[plugins][gcode][reader][viewer][layer-area]")
{
    ensure_plugin_test_runtime_initialized();
    using namespace Slic3r::Test::PerimeterPluginTests;

    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({}));
    PrintObject &object = prepared.print.object(0);
    REQUIRE(object.layer_count() >= 2);

    Printing::PrintingPlan &plan = prepared.print.mutable_printing_plan();
    plan.groups.emplace_back();
    Printing::PrintingGroup &group = plan.groups.back();
    for (size_t layer_index = 0; layer_index < 2; ++layer_index) {
        Layer &source_layer = object.layer(layer_index);
        REQUIRE(source_layer.islands().size() == 1);
        LayerSliceIsland &source_island = source_layer.island(0);
        LayerRegionIsland &first_region_island =
            source_island.get_or_add_region_island(source_island.regions(), 0);
        LayerRegionIsland &second_region_island =
            source_island.get_or_add_region_island(source_island.regions(), 1);

        group.layers.emplace_back();
        Printing::PrintingLayerGroup &layer_group = group.layers.back();
        layer_group.print_z = source_layer.scaled_print_z();
        layer_group.layers.push_back(&source_layer);
        layer_group.tool_groups.emplace_back();
        Printing::PrintingToolGroup &tool_group = layer_group.tool_groups.back();

        // Two region buckets from the same geometric island must not double
        // the cross-section, and neither should repeated extrusion roles.
        tool_group.extrusions.emplace_back();
        tool_group.extrusions.back().region_island = &first_region_island;
        tool_group.extrusions.emplace_back();
        tool_group.extrusions.back().region_island = &first_region_island;
        tool_group.extrusions.emplace_back();
        tool_group.extrusions.back().region_island = &second_region_island;
    }

    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state raw_initial = initial_state(initial_items);
    const slic3r_api::GCodeStateTracker initial(raw_initial);
    prepared.print.print_statistics()._layer_area_stats.emplace_back(
        scale_i(99.0), 99.0f);
    GCodeReaderResultBuilder builder(
        &prepared.print, &plan, "gcode.reader.default", initial);

    const std::vector<std::pair<coord_t, float>> &areas =
        prepared.print.print_statistics()._layer_area_stats;
    REQUIRE(areas.size() == 2);
    CHECK(areas[0].first == object.layer(0).scaled_print_z());
    CHECK(areas[1].first == object.layer(1).scaled_print_z());
    CHECK(areas[0].second == Approx(400.0));
    CHECK(areas[1].second == Approx(400.0));
}

TEST_CASE("Host G-code reader keeps object ids aligned with viewer names",
          "[plugins][gcode][reader][viewer]")
{
    ensure_plugin_test_runtime_initialized();
    Model model;
    Print print;
    Test::init_print({Test::TestMesh::cube_20x20x20,
                      Test::TestMesh::cube_20x20x20},
                     print, model);
    REQUIRE(print.objects().size() == 2);
    print.object(0).model_object()->name = "First object";
    print.object(1).model_object()->name = "Second object";

    Printing::PrintingPlan &plan = print.mutable_printing_plan();
    REQUIRE(plan.find_or_add_object_instance(print.objects()[0], 0) == 0);
    REQUIRE(plan.find_or_add_object_instance(print.objects()[1], 0) == 1);
    PluginStorage storage;
    slic3r_api::Ordering::ObjectInstanceCatalogPlugin::populate_object_instance_catalog(
        reinterpret_cast<const print_handle *>(&print),
        reinterpret_cast<printing_plan_handle *>(&plan),
        reinterpret_cast<storage_handle *>(&storage));
    REQUIRE(plan.object_instances.size() == 2);
    plan.object_instances[1].name = "Name copied from PrintingPlan";

    GCodeReaderService service(Orchestrator::instance(), &print);
    slic3r_api::GCodeStateTracker state =
        service.initial_state("gcode.reader.default");
    GCodeReaderResultBuilder builder(
        &print, &plan, "gcode.reader.default", state);
    consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_STATE_UPDATE,
        {{RAW_GCODE_READER_ITEM_OBJECT, 0, 1.0},
         {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 0.0}});
    consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_LINEAR_MOVE,
        {{RAW_GCODE_READER_ITEM_X, 0, 1.0},
         {RAW_GCODE_READER_ITEM_FEEDRATE, 0, 60.0},
         {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 1.0}});

    const GCodeProcessorResult result = builder.finish(state);
    REQUIRE(result.object_names.size() == 2);
    CHECK(result.object_names[0] == "First_object id:0 copy 0");
    CHECK(result.object_names[1] == "Name copied from PrintingPlan");
    REQUIRE(result.moves.size() == 1);
    CHECK(result.moves[0].object_id == 1u);
}

TEST_CASE("Built-in Readers recover plan object instance transitions",
          "[plugins][gcode][reader][viewer][object-instance]")
{
    ensure_plugin_test_runtime_initialized();
    GCodeReaderService service(Orchestrator::instance(), nullptr);

    SECTION("M486 uses zero-based firmware ids and minus one closes the object") {
        const GCodeProcessorResult result = service.read_for_viewer(
            "gcode.reader.marlin2",
            "M486 S0\nG1 X1 F60\nM486 S-1\nG1 X2\n");
        REQUIRE(result.moves.size() == 2);
        CHECK(result.moves[0].object_id == 0);
        CHECK(result.moves[1].object_id == GCodeProcessorResult::OBJECT_INSTANCE_ID_NONE);
        REQUIRE(result.object_names.size() == 1);
        CHECK(result.object_names[0] == "Object 0");
    }

    SECTION("the generic object comment also identifies plan instances") {
        const GCodeProcessorResult result = service.read_for_viewer(
            "gcode.reader.default",
            "; SLIC3R_OBJECT_INSTANCE_ID=3\nG1 X1 F60\n"
            "; SLIC3R_OBJECT_INSTANCE_ID=-1\nG1 X2\n");
        REQUIRE(result.moves.size() == 2);
        CHECK(result.moves[0].object_id == 3);
        CHECK(result.moves[1].object_id == GCodeProcessorResult::OBJECT_INSTANCE_ID_NONE);
        REQUIRE(result.object_names.size() == 4);
        CHECK(result.object_names[3] == "Object 3");
    }

    SECTION("an invalid M486 marker discards the previous object identity") {
        const GCodeProcessorResult result = service.read_for_viewer(
            "gcode.reader.marlin2",
            "M486 S0\nG1 X1 F60\nM486 Sinvalid\nG1 X2\n");
        REQUIRE(result.moves.size() == 2);
        CHECK(result.moves[0].object_id == 0);
        CHECK(result.moves[1].object_id == GCodeProcessorResult::OBJECT_INSTANCE_ID_NONE);
        REQUIRE(result.reader_diagnostics.size() == 1);
        CHECK(result.reader_diagnostics.front().type ==
              RAW_GCODE_READER_DIAGNOSTIC_INVALID_COMMAND_PARAMETERS);
    }

    SECTION("an invalid generic marker discards the previous object identity") {
        const GCodeProcessorResult result = service.read_for_viewer(
            "gcode.reader.default",
            "; SLIC3R_OBJECT_INSTANCE_ID=0\nG1 X1 F60\n"
            "; SLIC3R_OBJECT_INSTANCE_ID=invalid\nG1 X2\n");
        REQUIRE(result.moves.size() == 2);
        CHECK(result.moves[0].object_id == 0);
        CHECK(result.moves[1].object_id == GCodeProcessorResult::OBJECT_INSTANCE_ID_NONE);
        REQUIRE(result.reader_diagnostics.size() == 1);
        CHECK(result.reader_diagnostics.front().type ==
              RAW_GCODE_READER_DIAGNOSTIC_INVALID_COMMAND_PARAMETERS);
    }

    SECTION("Klipper definitions fix ids before named starts") {
        const GCodeProcessorResult result = service.read_for_viewer(
            "gcode.reader.klipper",
            "EXCLUDE_OBJECT_DEFINE NAME='second'\n"
            "EXCLUDE_OBJECT_DEFINE NAME='first'\n"
            "EXCLUDE_OBJECT_START NAME='first'\n"
            "G1 X1 F60\n"
            "EXCLUDE_OBJECT_END NAME='first'\n"
            "G1 X2\n");
        REQUIRE(result.moves.size() == 2);
        CHECK(result.moves[0].object_id == 1);
        CHECK(result.moves[1].object_id == GCodeProcessorResult::OBJECT_INSTANCE_ID_NONE);
        REQUIRE(result.object_names.size() == 2);
        CHECK(result.object_names[1] == "Object 1");
    }

    SECTION("Klipper starts allocate ids by first appearance without definitions") {
        const GCodeProcessorResult result = service.read_for_viewer(
            "gcode.reader.klipper",
            "EXCLUDE_OBJECT_START NAME='alpha'\nG1 X1 F60\n"
            "EXCLUDE_OBJECT_END NAME='alpha'\n"
            "EXCLUDE_OBJECT_START NAME='beta'\nG1 X2\n");
        REQUIRE(result.moves.size() == 2);
        CHECK(result.moves[0].object_id == 0);
        CHECK(result.moves[1].object_id == 1);
    }
}

TEST_CASE("Firmware and legacy object labels do not duplicate transitions",
          "[plugins][gcode][reader][viewer][object-instance][markers]")
{
    ensure_plugin_test_runtime_initialized();
    const std::string source =
        "M486 S0\n"
        "; SLIC3R_OBJECT_INSTANCE_ID=0\n"
        "G1 X1 F60\n"
        "M486 S-1\n"
        "; SLIC3R_OBJECT_INSTANCE_ID=-1\n"
        "M486 S1\n"
        "; SLIC3R_OBJECT_INSTANCE_ID=1\n"
        "G1 X2\n"
        "M486 S-1\n"
        "; SLIC3R_OBJECT_INSTANCE_ID=-1\n";
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    ReaderProbe probe;
    probe.chunks = {source};

    REQUIRE(execute_reader("gcode.reader.marlin2", probe, initial) ==
            RAW_PLUGIN_EXECUTION_SUCCESS);
    std::vector<uint32_t> transitions;
    for (const ObservedEvent &event : probe.observed) {
        const std::optional<double> object = observed_item_value(
            event, RAW_GCODE_READER_ITEM_OBJECT);
        if (object)
            transitions.push_back(static_cast<uint32_t>(*object));
    }
    CHECK(transitions == std::vector<uint32_t>{
        0, UINT32_MAX, 1, UINT32_MAX
    });

    GCodeReaderService service(Orchestrator::instance(), nullptr);
    const GCodeProcessorResult result = service.read_for_viewer(
        "gcode.reader.marlin2", source);
    REQUIRE(result.moves.size() == 2);
    CHECK(result.moves[0].object_id == 0);
    CHECK(result.moves[1].object_id == 1);
    REQUIRE(result.object_names.size() == 2);
    CHECK(result.object_names[0] == "Object 0");
    CHECK(result.object_names[1] == "Object 1");
}

TEST_CASE("Host G-code reader excludes retraction recovery from material usage",
          "[plugins][gcode][reader][viewer]")
{
    ensure_plugin_test_runtime_initialized();
    GCodeReaderService service(Orchestrator::instance(), nullptr);
    const GCodeProcessorResult result = service.read_for_viewer(
        "gcode.reader.default",
        "G1 E-2 F120\n"
        "G1 E0\n"
        "G1 X10 E1 F600\n");

    REQUIRE(result.moves.size() == 3);
    CHECK(result.moves[0].type == EMoveType::Retract);
    CHECK(result.moves[1].type == EMoveType::Unretract);
    CHECK(result.moves[2].type == EMoveType::Extrude);
    const double filament_area = PI * std::pow(1.75 * 0.5, 2.0);
    REQUIRE(result.print_statistics.volumes_per_extruder.count(0) == 1);
    CHECK(result.print_statistics.volumes_per_extruder.at(0) ==
          Approx(filament_area));
}

TEST_CASE("Host G-code reader does not display retraction recovery as extrusion",
          "[plugins][gcode][reader][viewer]")
{
    ensure_plugin_test_runtime_initialized();
    GCodeReaderService service(Orchestrator::instance(), nullptr);
    const GCodeProcessorResult result = service.read_for_viewer(
        "gcode.reader.default",
        "G1 E-2 F120\n"
        "G1 X10 E-1 F600\n"
        "G1 X20 E1\n");

    REQUIRE(result.moves.size() == 3);
    CHECK(result.moves[0].type == EMoveType::Retract);
    CHECK(result.moves[1].type == EMoveType::Travel);
    CHECK(result.moves[1].mm3_per_mm == 0.0f);
    CHECK(result.moves[2].type == EMoveType::Extrude);
    CHECK(result.moves[2].mm3_per_mm > 0.0f);

    const double filament_area = PI * std::pow(1.75 * 0.5, 2.0);
    REQUIRE(result.print_statistics.volumes_per_extruder.count(0) == 1);
    CHECK(result.print_statistics.volumes_per_extruder.at(0) ==
          Approx(filament_area));
}

// Read fresh streams at each loading boundary from borrowed local Print/plan
// settings. Debt repayment produces no consumption; only excess E contributes
// owned viewer statistics. The standalone path keeps independent tool debt.
TEST_CASE("Host G-code reader shares loading debt only for SEMM prints",
          "[plugins][gcode][reader][viewer][semm-material]")
{
    ensure_plugin_test_runtime_initialized();
    const bool semm = GENERATE(false, true);
    const char *reader_id = GENERATE("gcode.reader.default", "gcode.reader.marlin2");
    CAPTURE(semm, reader_id);
    Print print;
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.single_extruder_multi_material.value = semm;
    config.nozzle_diameter.set(std::vector<double>{0.4, 0.4});
    config.filament_diameter.set(std::vector<double>{1.75, 1.75});
    print.mutable_printing_plan();
    GCodeReaderService service(Orchestrator::instance(), &print);
    const std::string load = "M82\nT0\nG92 E0\nG1 E-92 F120\nT1\nG92 E0\n"
                             ";TYPE:Wipe tower\nG1 X10 E90 F600\n";
    const double filament_area = PI * std::pow(1.75 * 0.5, 2.0);
    const std::vector<std::pair<std::string, double>> boundaries = {
        {"", 90.0}, {"G1 X20 E92\n", 92.0}, {"G1 X20 E92\nG1 X30 E93\n", 93.0}
    };
    for (const std::pair<std::string, double> &boundary : boundaries) {
        CAPTURE(boundary.second);
        const GCodeProcessorResult result = service.read_for_viewer(reader_id, load + boundary.first);
        const double consumed = semm ? std::max(boundary.second - 92.0, 0.0) : boundary.second;
        CHECK(result.print_statistics.volumes_per_extruder.count(0) == 0);
        if (consumed == 0.0) {
            CHECK(result.print_statistics.volumes_per_extruder.empty());
            CHECK(result.print_statistics.volumes_per_role_per_extruder.empty());
            CHECK(result.print_statistics.used_filaments_per_role.empty());
            CHECK(result.moves.back().type == EMoveType::Travel);
        } else {
            REQUIRE(result.print_statistics.volumes_per_extruder.count(1) == 1);
            CHECK(result.print_statistics.volumes_per_extruder.at(1) == Approx(consumed * filament_area));
            REQUIRE(result.print_statistics.volumes_per_role_per_extruder.count(GCodeExtrusionRole::WipeTower) == 1);
            CHECK(result.print_statistics.volumes_per_role_per_extruder.at(GCodeExtrusionRole::WipeTower).at(1) ==
                  Approx(consumed * filament_area));
            CHECK(result.moves.back().type == EMoveType::Extrude);
        }
    }
    GCodeReaderService standalone(Orchestrator::instance(), nullptr);
    const GCodeProcessorResult external = standalone.read_for_viewer(reader_id, load);
    REQUIRE(external.print_statistics.volumes_per_extruder.count(1) == 1);
    CHECK(external.print_statistics.volumes_per_extruder.at(1) == Approx(90.0 * filament_area));
}

// Supply normalized events through the existing helper and infer retained debt
// from a subsequent 2 mm load. Invalidation uses active-tool E, not slot 0's E;
// losing only the tool identity preserves debt, but unattributable E changes clear it.
TEST_CASE("Host G-code reader SEMM debt follows active tool E validity",
          "[plugins][gcode][reader][viewer][semm-material]")
{
    ensure_plugin_test_runtime_initialized();
    Print print;
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.single_extruder_multi_material.value = true;
    config.nozzle_diameter.set(std::vector<double>{0.4, 0.4});
    config.filament_diameter.set(std::vector<double>{1.75, 1.75});
    Printing::PrintingPlan &plan = print.mutable_printing_plan();
    std::vector<raw_gcode_reader_event_item> initial_items;
    raw_gcode_reader_state initial = initial_state(initial_items);
    initial_items.push_back({RAW_GCODE_READER_ITEM_E, 1, 0.0});
    initial.items = initial_items.data();
    initial.item_count = static_cast<uint32_t>(initial_items.size());
    slic3r_api::GCodeStateTracker state(initial);
    GCodeReaderResultBuilder builder(&print, &plan, "test.reader", state);
    consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_LINEAR_MOVE,
        {{RAW_GCODE_READER_ITEM_E, 0, -2.0},
         {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 0.0}});
    consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_STATE_UPDATE,
        {{RAW_GCODE_READER_ITEM_ACTIVE_TOOL, 0, 1.0},
         {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 1.0}});
    bool debt_cleared = false;
    SECTION("inactive tool E invalidation preserves shared debt") {
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_INVALIDATE,
            {{RAW_GCODE_READER_ITEM_E, 0, 0.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 2.0}});
    }
    SECTION("active tool E invalidation clears shared debt") {
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_INVALIDATE,
            {{RAW_GCODE_READER_ITEM_E, 1, 0.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 2.0}});
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_SET_POSITION,
            {{RAW_GCODE_READER_ITEM_E, 1, 0.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 3.0}});
        debt_cleared = true;
    }
    SECTION("unknown active tool alone preserves shared debt") {
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_INVALIDATE,
            {{RAW_GCODE_READER_ITEM_ACTIVE_TOOL, 0, 0.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 2.0}});
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_STATE_UPDATE,
            {{RAW_GCODE_READER_ITEM_ACTIVE_TOOL, 0, 1.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 3.0}});
    }
    SECTION("E change with unknown active tool clears shared debt") {
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_INVALIDATE,
            {{RAW_GCODE_READER_ITEM_ACTIVE_TOOL, 0, 0.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 2.0}});
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_STATE_UPDATE,
            {{RAW_GCODE_READER_ITEM_E, 1, 1.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 3.0}});
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_STATE_UPDATE,
            {{RAW_GCODE_READER_ITEM_ACTIVE_TOOL, 0, 1.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 4.0}});
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_SET_POSITION,
            {{RAW_GCODE_READER_ITEM_E, 1, 0.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 5.0}});
        debt_cleared = true;
    }
    consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_LINEAR_MOVE,
        {{RAW_GCODE_READER_ITEM_X, 0, 2.0}, {RAW_GCODE_READER_ITEM_E, 1, 2.0},
         {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 6.0}});
    const GCodeProcessorResult result = builder.finish(state);
    if (debt_cleared) {
        REQUIRE(result.print_statistics.volumes_per_extruder.count(1) == 1);
        CHECK(result.print_statistics.volumes_per_extruder.at(1) ==
              Approx(2.0 * PI * std::pow(1.75 * 0.5, 2.0)));
    } else {
        CHECK(result.print_statistics.volumes_per_extruder.empty());
    }
}

// Read two tools with different filament properties and unlabelled tower moves;
// role/tool volumes must reconcile with per-tool volumes and per-role m/g totals.
// Reset a copied statistics value to verify that its owned role/tool map is cleared.
TEST_CASE("Host G-code reader accounts for filament by role and logical tool",
          "[plugins][gcode][reader][viewer][role-tool-material]")
{
    ensure_plugin_test_runtime_initialized();
    Print print;
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.single_extruder_multi_material.value = false;
    config.nozzle_diameter.set(std::vector<double>{0.4, 0.6});
    config.filament_diameter.set(std::vector<double>{1.75, 2.85});
    config.filament_density.set(std::vector<double>{1.1, 1.3});
    print.mutable_printing_plan();
    GCodeReaderService service(Orchestrator::instance(), &print);
    const GCodeProcessorResult result = service.read_for_viewer("gcode.reader.marlin2",
        "M82\nT0\nG92 E0\n;TYPE:Wipe tower\nG1 X10 E2 F600\n"
        ";TYPE:Perimeter\nG1 X20 E3\nT1\nG92 E0\n"
        ";TYPE:Wipe tower\nG1 X30 E3\n;TYPE:Perimeter\nG1 X40 E7\n");
    const PrintEstimatedStatistics &stats = result.print_statistics;
    const double area0 = PI * std::pow(result.filament_diameters[0] * 0.5, 2.0);
    const double area1 = PI * std::pow(result.filament_diameters[1] * 0.5, 2.0);
    REQUIRE(stats.volumes_per_role_per_extruder.size() == 2);
    REQUIRE(stats.volumes_per_role_per_extruder.count(GCodeExtrusionRole::WipeTower) == 1);
    REQUIRE(stats.volumes_per_role_per_extruder.count(GCodeExtrusionRole::Perimeter) == 1);
    const std::map<extruder_index_t, double> &tower = stats.volumes_per_role_per_extruder.at(GCodeExtrusionRole::WipeTower);
    const std::map<extruder_index_t, double> &perimeter = stats.volumes_per_role_per_extruder.at(GCodeExtrusionRole::Perimeter);
    REQUIRE(tower.size() == 2);
    REQUIRE(perimeter.size() == 2);
    CHECK(tower.at(0) == Approx(2.0 * area0));
    CHECK(tower.at(1) == Approx(3.0 * area1));
    CHECK(perimeter.at(0) == Approx(area0));
    CHECK(perimeter.at(1) == Approx(4.0 * area1));
    CHECK(stats.volumes_per_extruder.at(0) == Approx(tower.at(0) + perimeter.at(0)));
    CHECK(stats.volumes_per_extruder.at(1) == Approx(tower.at(1) + perimeter.at(1)));
    for (const auto &[role, volumes] : stats.volumes_per_role_per_extruder) {
        const double expected_length_m = (volumes.at(0) / area0 + volumes.at(1) / area1) * 0.001;
        const double expected_mass_g = (volumes.at(0) * result.filament_densities[0] +
                                        volumes.at(1) * result.filament_densities[1]) * 0.001;
        REQUIRE(stats.used_filaments_per_role.count(role) == 1);
        CHECK(stats.used_filaments_per_role.at(role).first == Approx(expected_length_m));
        CHECK(stats.used_filaments_per_role.at(role).second == Approx(expected_mass_g));
    }
    CHECK(stats.volumes_per_role_per_extruder_per_object.empty());
    PrintEstimatedStatistics cleared = stats;
    cleared.reset();
    CHECK(cleared.volumes_per_role_per_extruder.empty());
}

TEST_CASE("Host G-code reader invalidates only unreliable retraction debt",
          "[plugins][gcode][reader][viewer]")
{
    std::vector<raw_gcode_reader_event_item> initial_items;
    raw_gcode_reader_state initial = initial_state(initial_items);
    initial_items.push_back({RAW_GCODE_READER_ITEM_E, 1, 0.0});
    initial.items = initial_items.data();
    initial.item_count = static_cast<uint32_t>(initial_items.size());
    const double filament_area = PI * std::pow(1.75 * 0.5, 2.0);

    SECTION("unknown E clears the corresponding tool debt") {
        slic3r_api::GCodeStateTracker state(initial);
        GCodeReaderResultBuilder builder(nullptr, nullptr, "test.reader", state);
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_LINEAR_MOVE,
            {{RAW_GCODE_READER_ITEM_E, 0, -2.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 0.0}});
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_INVALIDATE,
            {{RAW_GCODE_READER_ITEM_E, 0, 0.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 1.0}});
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_STATE_UPDATE,
            {{RAW_GCODE_READER_ITEM_E, 0, 0.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 2.0}});
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_LINEAR_MOVE,
            {{RAW_GCODE_READER_ITEM_X, 0, 2.0},
             {RAW_GCODE_READER_ITEM_E, 0, 1.0},
             {RAW_GCODE_READER_ITEM_FEEDRATE, 0, 60.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 3.0}});

        const GCodeProcessorResult result = builder.finish(state);
        REQUIRE(result.print_statistics.volumes_per_extruder.count(0) == 1);
        CHECK(result.print_statistics.volumes_per_extruder.at(0) ==
              Approx(filament_area));
    }

    SECTION("unknown active tool alone preserves every tool debt") {
        slic3r_api::GCodeStateTracker state(initial);
        GCodeReaderResultBuilder builder(nullptr, nullptr, "test.reader", state);
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_LINEAR_MOVE,
            {{RAW_GCODE_READER_ITEM_E, 0, -2.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 0.0}});
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_INVALIDATE,
            {{RAW_GCODE_READER_ITEM_ACTIVE_TOOL, 0, 0.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 1.0}});
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_STATE_UPDATE,
            {{RAW_GCODE_READER_ITEM_ACTIVE_TOOL, 0, 0.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 2.0}});
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_LINEAR_MOVE,
            {{RAW_GCODE_READER_ITEM_X, 0, 2.0},
             {RAW_GCODE_READER_ITEM_E, 0, 0.0},
             {RAW_GCODE_READER_ITEM_FEEDRATE, 0, 60.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 3.0}});

        const GCodeProcessorResult result = builder.finish(state);
        CHECK(result.print_statistics.volumes_per_extruder.count(0) == 0);
    }

    SECTION("state update changing E with unknown active tool clears every debt") {
        slic3r_api::GCodeStateTracker state(initial);
        GCodeReaderResultBuilder builder(nullptr, nullptr, "test.reader", state);
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_LINEAR_MOVE,
            {{RAW_GCODE_READER_ITEM_E, 0, -2.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 0.0}});
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_INVALIDATE,
            {{RAW_GCODE_READER_ITEM_ACTIVE_TOOL, 0, 0.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 1.0}});
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_STATE_UPDATE,
            {{RAW_GCODE_READER_ITEM_E, 0, 0.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 2.0}});
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_STATE_UPDATE,
            {{RAW_GCODE_READER_ITEM_ACTIVE_TOOL, 0, 0.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 3.0}});
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_LINEAR_MOVE,
            {{RAW_GCODE_READER_ITEM_X, 0, 2.0},
             {RAW_GCODE_READER_ITEM_E, 0, 1.0},
             {RAW_GCODE_READER_ITEM_FEEDRATE, 0, 60.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 4.0}});

        const GCodeProcessorResult result = builder.finish(state);
        REQUIRE(result.print_statistics.volumes_per_extruder.count(0) == 1);
        CHECK(result.print_statistics.volumes_per_extruder.at(0) ==
              Approx(filament_area));
    }

    SECTION("set position changing E with unknown active tool clears every debt") {
        slic3r_api::GCodeStateTracker state(initial);
        GCodeReaderResultBuilder builder(nullptr, nullptr, "test.reader", state);
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_LINEAR_MOVE,
            {{RAW_GCODE_READER_ITEM_E, 0, -2.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 0.0}});
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_INVALIDATE,
            {{RAW_GCODE_READER_ITEM_ACTIVE_TOOL, 0, 0.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 1.0}});
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_SET_POSITION,
            {{RAW_GCODE_READER_ITEM_E, 0, 0.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 2.0}});
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_STATE_UPDATE,
            {{RAW_GCODE_READER_ITEM_ACTIVE_TOOL, 0, 0.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 3.0}});
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_LINEAR_MOVE,
            {{RAW_GCODE_READER_ITEM_X, 0, 2.0},
             {RAW_GCODE_READER_ITEM_E, 0, 1.0},
             {RAW_GCODE_READER_ITEM_FEEDRATE, 0, 60.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 4.0}});

        const GCodeProcessorResult result = builder.finish(state);
        REQUIRE(result.print_statistics.volumes_per_extruder.count(0) == 1);
        CHECK(result.print_statistics.volumes_per_extruder.at(0) ==
              Approx(filament_area));
    }

    SECTION("E movement with unknown active tool clears debt without assigning new debt") {
        slic3r_api::GCodeStateTracker state(initial);
        GCodeReaderResultBuilder builder(nullptr, nullptr, "test.reader", state);
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_LINEAR_MOVE,
            {{RAW_GCODE_READER_ITEM_E, 0, -2.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 0.0}});
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_INVALIDATE,
            {{RAW_GCODE_READER_ITEM_ACTIVE_TOOL, 0, 0.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 1.0}});
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_LINEAR_MOVE,
            {{RAW_GCODE_READER_ITEM_E, 0, -3.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 2.0}});
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_STATE_UPDATE,
            {{RAW_GCODE_READER_ITEM_ACTIVE_TOOL, 0, 0.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 3.0}});
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_LINEAR_MOVE,
            {{RAW_GCODE_READER_ITEM_X, 0, 2.0},
             {RAW_GCODE_READER_ITEM_E, 0, 0.0},
             {RAW_GCODE_READER_ITEM_FEEDRATE, 0, 60.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 4.0}});

        const GCodeProcessorResult result = builder.finish(state);
        REQUIRE(result.print_statistics.volumes_per_extruder.count(0) == 1);
        CHECK(result.print_statistics.volumes_per_extruder.at(0) ==
              Approx(3.0 * filament_area));
    }

    SECTION("unchanged E with unknown active tool preserves every debt") {
        slic3r_api::GCodeStateTracker state(initial);
        GCodeReaderResultBuilder builder(nullptr, nullptr, "test.reader", state);
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_LINEAR_MOVE,
            {{RAW_GCODE_READER_ITEM_E, 0, -2.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 0.0}});
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_INVALIDATE,
            {{RAW_GCODE_READER_ITEM_ACTIVE_TOOL, 0, 0.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 1.0}});
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_STATE_UPDATE,
            {{RAW_GCODE_READER_ITEM_E, 0, -2.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 2.0}});
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_STATE_UPDATE,
            {{RAW_GCODE_READER_ITEM_ACTIVE_TOOL, 0, 0.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 3.0}});
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_LINEAR_MOVE,
            {{RAW_GCODE_READER_ITEM_E, 0, 0.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 4.0}});

        const GCodeProcessorResult result = builder.finish(state);
        CHECK(result.print_statistics.volumes_per_extruder.count(0) == 0);
    }

    SECTION("targeted E invalidation preserves another tool debt") {
        slic3r_api::GCodeStateTracker state(initial);
        GCodeReaderResultBuilder builder(nullptr, nullptr, "test.reader", state);
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_LINEAR_MOVE,
            {{RAW_GCODE_READER_ITEM_E, 0, -2.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 0.0}});
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_STATE_UPDATE,
            {{RAW_GCODE_READER_ITEM_ACTIVE_TOOL, 0, 1.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 1.0}});
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_LINEAR_MOVE,
            {{RAW_GCODE_READER_ITEM_E, 1, -2.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 2.0}});
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_INVALIDATE,
            {{RAW_GCODE_READER_ITEM_E, 0, 0.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 3.0}});
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_LINEAR_MOVE,
            {{RAW_GCODE_READER_ITEM_E, 1, 0.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 4.0}});

        const GCodeProcessorResult result = builder.finish(state);
        CHECK(result.print_statistics.volumes_per_extruder.count(1) == 0);
    }

    SECTION("known tool changes preserve per-tool debt") {
        slic3r_api::GCodeStateTracker state(initial);
        GCodeReaderResultBuilder builder(nullptr, nullptr, "test.reader", state);
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_LINEAR_MOVE,
            {{RAW_GCODE_READER_ITEM_E, 0, -2.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 0.0}});
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_STATE_UPDATE,
            {{RAW_GCODE_READER_ITEM_ACTIVE_TOOL, 0, 1.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 1.0}});
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_STATE_UPDATE,
            {{RAW_GCODE_READER_ITEM_ACTIVE_TOOL, 0, 0.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 2.0}});
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_LINEAR_MOVE,
            {{RAW_GCODE_READER_ITEM_E, 0, 0.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 3.0}});

        const GCodeProcessorResult result = builder.finish(state);
        CHECK(result.print_statistics.volumes_per_extruder.count(0) == 0);
    }
}

TEST_CASE("Host G-code reader derives tools and arc vertices from events",
          "[plugins][gcode][reader][viewer]")
{
    ensure_plugin_test_runtime_initialized();
    GCodeReaderService service(Orchestrator::instance(), nullptr);
    const GCodeProcessorResult result = service.read_for_viewer(
        "gcode.reader.default",
        "T1\n"
        "G1 X1 Y0 F60\n"
        "G3 X0 Y1 I-1 J0\n");

    CHECK(result.extruders_count == 2);
    REQUIRE(result.moves.size() >= 3);
    CHECK(result.moves[result.moves.size() - 2].internal_only);
    CHECK_FALSE(result.moves.back().internal_only);
    CHECK(result.moves.back().extruder_id == 1u);
    CHECK(result.moves.back().position.x() == Approx(0.0));
    CHECK(result.moves.back().position.y() == Approx(1.0));
}

TEST_CASE("Host G-code reader creates typed marker vertices",
          "[plugins][gcode][reader][viewer]")
{
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    slic3r_api::GCodeStateTracker state(initial);
    GCodeReaderResultBuilder builder(nullptr, nullptr, "test.reader", state);

    const uint32_t markers[] = {
        RAW_GCODE_READER_MARKER_SEAM,
        RAW_GCODE_READER_MARKER_COLOR_CHANGE,
        RAW_GCODE_READER_MARKER_PAUSE_PRINT,
        RAW_GCODE_READER_MARKER_CUSTOM_GCODE
    };
    for (uint32_t line = 0; line < 4; ++line) {
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_MARKER,
            {{RAW_GCODE_READER_ITEM_MARKER, 0, double(markers[line])},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, double(line)}});
        CHECK_FALSE(state.known(RAW_GCODE_READER_ITEM_MARKER));
    }
    consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_LINEAR_MOVE,
        {{RAW_GCODE_READER_ITEM_X, 0, 2.0},
         {RAW_GCODE_READER_ITEM_E, 0, 1.0},
         {RAW_GCODE_READER_ITEM_FEEDRATE, 0, 60.0},
         {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 4.0}});

    const GCodeProcessorResult result = builder.finish(state);
    REQUIRE(result.moves.size() == 5);
    CHECK(result.moves[0].type == EMoveType::Seam);
    CHECK(result.moves[1].type == EMoveType::Color_change);
    CHECK(result.moves[2].type == EMoveType::Pause_Print);
    CHECK(result.moves[3].type == EMoveType::Custom_GCode);
    CHECK(result.moves[0].cp_color_id == 0u);
    CHECK(result.moves[1].cp_color_id == 1u);
    CHECK(result.moves.back().cp_color_id == 1u);
}

// Search a borrowed result in order; return the first matching index or size() if absent, without mutation or allocation.
size_t first_move_of_type(const GCodeProcessorResult &result, EMoveType type)
{
    for (size_t index = 0; index < result.moves.size(); ++index)
        if (result.moves[index].type == type)
            return index;
    return result.moves.size();
}

// Read a borrowed layer-height slot, returning 0 when absent, without mutation or allocation.
float recorded_layer_height(const GCodeProcessorResult &result, size_t layer)
{
    return layer < result.layers_max_height.size() ? result.layers_max_height[layer] : 0.0f;
}

// Verify tagged heights, extrusion-derived maxima, absent layers, and unchanged move heights.
TEST_CASE("Host G-code reader stores the maximal height of each layer",
          "[plugins][gcode][reader][viewer][layer_height]")
{
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    slic3r_api::GCodeStateTracker state(initial);
    GCodeReaderResultBuilder builder(nullptr, nullptr, "test.reader", state);

    double line = 0.0;
    // Copy the borrowed delta into local storage for synchronous tracker and builder calls,
    // append a line number, and propagate allocation or validation failures.
    const std::function<void(std::initializer_list<raw_gcode_reader_event_item>)> consume_update =
        [&](std::initializer_list<raw_gcode_reader_event_item> items) {
            std::vector<raw_gcode_reader_event_item> owned(items);
            owned.push_back({RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, line++});
            const raw_gcode_reader_event event = {
                RAW_GCODE_READER_EVENT_STATE_UPDATE, owned.data(), static_cast<uint32_t>(owned.size())};
            state.apply(event);
            builder.consume_event(event, state);
        };
    double x = 1.0;
    double e = 0.0;
    // Advance local X/E and consume one linear move at the current layer and HEIGHT;
    // event data is not retained, and allocation or validation failures propagate.
    const std::function<void()> consume_extrusion = [&]() {
        x += 1.0;
        e += 1.0;
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_LINEAR_MOVE,
            {{RAW_GCODE_READER_ITEM_X, 0, x},
             {RAW_GCODE_READER_ITEM_E, 0, e},
             {RAW_GCODE_READER_ITEM_FEEDRATE, 0, 60.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, line++}});
    };

    // Layer 0: the tag (0.4) wins over the extrusions (0.2).
    consume_update({{RAW_GCODE_READER_ITEM_LAYER, 0, 0.0}});
    consume_update({{RAW_GCODE_READER_ITEM_MAX_LAYER_HEIGHT, 0, 0.4}});
    consume_update({{RAW_GCODE_READER_ITEM_HEIGHT, 0, 0.2}});
    consume_extrusion();
    // Layer 1: no tag, the greatest extrusion height (0.3 then 0.2) is kept.
    consume_update({{RAW_GCODE_READER_ITEM_LAYER, 0, 1.0}});
    consume_update({{RAW_GCODE_READER_ITEM_HEIGHT, 0, 0.3}});
    consume_extrusion();
    consume_update({{RAW_GCODE_READER_ITEM_HEIGHT, 0, 0.2}});
    consume_extrusion();
    // Layer 2: neither tag nor extrusion; nothing is inherited from layer 0.
    consume_update({{RAW_GCODE_READER_ITEM_LAYER, 0, 2.0}});

    const GCodeProcessorResult result = builder.finish(state);
    CHECK(recorded_layer_height(result, 0) == Approx(0.4));
    CHECK(recorded_layer_height(result, 1) == Approx(0.3));
    CHECK(recorded_layer_height(result, 2) == 0.0f);
    // The extrusions keep their own height.
    REQUIRE(result.moves.size() == 3);
    CHECK(result.moves[0].height == Approx(0.2));
    CHECK(result.moves[1].height == Approx(0.3));
    CHECK(result.moves[2].height == Approx(0.2));
}

// Verify that tagged marker geometry uses its layer height while extrusion geometry remains unchanged.
TEST_CASE("Host G-code reader sizes markers with the maximal height of their layer",
          "[plugins][gcode][reader][viewer][layer_height]")
{
    // Layer 0 ends with HEIGHT 0.2; layer 1 starts with the marker, optionally
    // tagged 0.4, and then extrudes at 0.3.
    // Build fresh local Reader state for one supported marker and optional tag, then
    // return its owned result; event data is synchronous and processing failures propagate.
    const std::function<GCodeProcessorResult(bool, uint32_t)> build = [](bool tagged, uint32_t marker) {
        std::vector<raw_gcode_reader_event_item> initial_items;
        const raw_gcode_reader_state initial = initial_state(initial_items);
        slic3r_api::GCodeStateTracker state(initial);
        GCodeReaderResultBuilder builder(nullptr, nullptr, "test.reader", state);
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_STATE_UPDATE,
            {{RAW_GCODE_READER_ITEM_LAYER, 0, 0.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 0.0}});
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_STATE_UPDATE,
            {{RAW_GCODE_READER_ITEM_HEIGHT, 0, 0.2},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 1.0}});
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_STATE_UPDATE,
            {{RAW_GCODE_READER_ITEM_LAYER, 0, 1.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 2.0}});
        if (tagged)
            consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_STATE_UPDATE,
                {{RAW_GCODE_READER_ITEM_MAX_LAYER_HEIGHT, 0, 0.4},
                 {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 3.0}});
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_MARKER,
            {{RAW_GCODE_READER_ITEM_MARKER, 0, double(marker)},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 4.0}});
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_STATE_UPDATE,
            {{RAW_GCODE_READER_ITEM_HEIGHT, 0, 0.3},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 5.0}});
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_LINEAR_MOVE,
            {{RAW_GCODE_READER_ITEM_X, 0, 2.0},
             {RAW_GCODE_READER_ITEM_E, 0, 1.0},
             {RAW_GCODE_READER_ITEM_FEEDRATE, 0, 60.0},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 6.0}});
        return builder.finish(state);
    };

    SECTION("a color change takes the tag of its layer, the extrusion keeps its height") {
        const GCodeProcessorResult result = build(true, RAW_GCODE_READER_MARKER_COLOR_CHANGE);
        const size_t marker = first_move_of_type(result, EMoveType::Color_change);
        const size_t extrusion = first_move_of_type(result, EMoveType::Extrude);
        REQUIRE(marker < result.moves.size());
        REQUIRE(extrusion < result.moves.size());
        CHECK(result.moves[marker].height == Approx(0.4));
        CHECK(result.moves[extrusion].height == Approx(0.3));
    }
    SECTION("a seam takes the tag of its layer") {
        const GCodeProcessorResult result = build(true, RAW_GCODE_READER_MARKER_SEAM);
        const size_t marker = first_move_of_type(result, EMoveType::Seam);
        REQUIRE(marker < result.moves.size());
        CHECK(result.moves[marker].height == Approx(0.4));
    }
    SECTION("without a tag the marker keeps the last HEIGHT read") {
        const GCodeProcessorResult result = build(false, RAW_GCODE_READER_MARKER_COLOR_CHANGE);
        const size_t marker = first_move_of_type(result, EMoveType::Color_change);
        REQUIRE(marker < result.moves.size());
        CHECK(result.moves[marker].height == Approx(0.2));
    }
}

// Parse borrowed G-code text with a fresh processor and return its owned result after finalization;
// the input is not retained, and parse or allocation failures propagate.
GCodeProcessorResult process_with_legacy_processor(const std::string &gcode)
{
    GCodeProcessor processor;
    processor.reset();
    processor.process_string(gcode);
    processor.finalize(false);
    return processor.extract_result();
}

// Verify tagged heights, extrusion-derived maxima, absent layers, reset behavior, and tag reservation.
TEST_CASE("GCodeProcessor stores the maximal height of each layer",
          "[plugins][gcode][processor][layer_height]")
{
    ensure_plugin_test_runtime_initialized();
    SECTION("tag, extrusion fallback and absent layer") {
        const GCodeProcessorResult result = process_with_legacy_processor(
            "G90\nM82\n"
            ";LAYER_CHANGE\n;Z:0.4\n;MAX_LAYER_HEIGHT:0.4\n;HEIGHT:0.2\nG1 X10 Y0 E1 F6000\n"
            ";LAYER_CHANGE\n;Z:0.6\n;HEIGHT:0.3\nG1 X20 Y0 E2\n;HEIGHT:0.2\nG1 X30 Y0 E3\n"
            ";LAYER_CHANGE\n;Z:0.8\n");
        // GCodeProcessor numbers the layers from 1: LAYER_CHANGE increments the id.
        CHECK(recorded_layer_height(result, 1) == Approx(0.4));
        CHECK(recorded_layer_height(result, 2) == Approx(0.3));
        CHECK(recorded_layer_height(result, 3) == 0.0f);
    }
    SECTION("an invalid tag is ignored and the extrusions decide") {
        const GCodeProcessorResult result = process_with_legacy_processor(
            "G90\nM82\n"
            ";LAYER_CHANGE\n;Z:0.4\n;MAX_LAYER_HEIGHT:abc\n;HEIGHT:0.2\nG1 X10 Y0 E1 F6000\n"
            ";LAYER_CHANGE\n;Z:0.6\n;MAX_LAYER_HEIGHT:0\n;HEIGHT:0.3\nG1 X20 Y0 E2\n");
        CHECK(recorded_layer_height(result, 1) == Approx(0.2));
        CHECK(recorded_layer_height(result, 2) == Approx(0.3));
    }
    SECTION("a processor reset forgets the previous file") {
        GCodeProcessor processor;
        processor.process_string(
            ";LAYER_CHANGE\n;Z:0.4\n;MAX_LAYER_HEIGHT:0.4\n;HEIGHT:0.2\nG1 X10 Y0 E1 F6000\n");
        REQUIRE(processor.get_result().layers_max_height.size() == 2);
        processor.reset();
        CHECK(processor.get_result().layers_max_height.empty());
        // A fresh layer 1 without tag must not see the tag of the earlier file.
        processor.process_string(
            ";LAYER_CHANGE\n;Z:0.4\n;HEIGHT:0.2\nG1 X10 Y0 E1 F6000\n");
        CHECK(recorded_layer_height(processor.get_result(), 1) == Approx(0.2));
    }
    SECTION("the tag is a reserved tag") {
        std::string found;
        CHECK(GCodeProcessor::contains_reserved_tag(";MAX_LAYER_HEIGHT:0.3\n", found));
    }
}

// Verify marker tag selection in immediate and deferred paths without changing extrusion heights.
TEST_CASE("GCodeProcessor sizes markers with the maximal height of their layer",
          "[plugins][gcode][processor][layer_height]")
{
    ensure_plugin_test_runtime_initialized();
    // Layer 1 extrudes at 0.2. Layer 2 starts with a color change, optionally
    // in a layer tagged 0.4, and then extrudes at 0.3.
    // Return an owned G-code string differing only by layer-tag presence; no processor
    // state is mutated, and string-allocation failures propagate.
    const std::function<std::string(bool)> color_change_gcode = [](bool tagged) {
        return std::string("G90\nM82\n"
            ";LAYER_CHANGE\n;Z:0.4\n;HEIGHT:0.2\nG1 X10 Y0 E1 F6000\n"
            ";LAYER_CHANGE\n;Z:0.6\n") +
            (tagged ? ";MAX_LAYER_HEIGHT:0.4\n" : "") +
            ";COLOR_CHANGE,T0,#FF0000\n;HEIGHT:0.3\nG1 X20 Y0 E2\n";
    };

    SECTION("a waiting color change takes the tag of its layer, the extrusion keeps its height") {
        const GCodeProcessorResult result = process_with_legacy_processor(color_change_gcode(true));
        const size_t marker = first_move_of_type(result, EMoveType::Color_change);
        REQUIRE(marker < result.moves.size());
        CHECK(result.moves[marker].height == Approx(0.4));
        // Both extrusions stay at their own height.
        REQUIRE(first_move_of_type(result, EMoveType::Extrude) < result.moves.size());
        CHECK(result.moves[first_move_of_type(result, EMoveType::Extrude)].height == Approx(0.2));
        CHECK(result.moves.back().type == EMoveType::Extrude);
        CHECK(result.moves.back().height == Approx(0.3));
    }
    SECTION("without a tag the color change takes the height of the next extrusion") {
        const GCodeProcessorResult result = process_with_legacy_processor(color_change_gcode(false));
        const size_t marker = first_move_of_type(result, EMoveType::Color_change);
        REQUIRE(marker < result.moves.size());
        CHECK(result.moves[marker].height == Approx(0.3));
        CHECK(result.moves.back().height == Approx(0.3));
    }
    SECTION("a pause without following extrusion takes the tag of its layer") {
        const GCodeProcessorResult result = process_with_legacy_processor(
            "G90\nM82\n"
            ";LAYER_CHANGE\n;Z:0.4\n;MAX_LAYER_HEIGHT:0.4\n;HEIGHT:0.2\nG1 X10 Y0 E1 F6000\n"
            ";PAUSE_PRINT\n");
        const size_t marker = first_move_of_type(result, EMoveType::Pause_Print);
        REQUIRE(marker < result.moves.size());
        CHECK(result.moves[marker].height == Approx(0.4));
    }
    SECTION("a seam takes the tag of its layer and the extrusion keeps its height") {
        const GCodeProcessorResult result = process_with_legacy_processor(
            "G90\nM82\n"
            ";LAYER_CHANGE\n;Z:0.4\n;MAX_LAYER_HEIGHT:0.4\n;HEIGHT:0.3\n;SEAM:5:0\nG1 X10 Y0 E1 F6000\n");
        const size_t marker = first_move_of_type(result, EMoveType::Seam);
        const size_t extrusion = first_move_of_type(result, EMoveType::Extrude);
        REQUIRE(marker < result.moves.size());
        REQUIRE(extrusion < result.moves.size());
        CHECK(result.moves[marker].height == Approx(0.4));
        CHECK(result.moves[extrusion].height == Approx(0.3));
    }
}

// Process local G-code with a tag on layer 65535 and a deferred marker on 65536; verify the shared
// capped slot, 0.4 marker height, and explicit 0.2 extrusion HEIGHT. Result data is locally owned;
// parsing or allocation failures fail the test.
TEST_CASE("GCodeProcessor caps layer heights at the MoveVertex boundary",
          "[plugins][gcode][processor][layer_height]")
{
    ensure_plugin_test_runtime_initialized();

    std::string gcode = "G90\nM82\n";
    gcode.reserve(gcode.size() + (static_cast<size_t>(UINT16_MAX) + 1u) * 14u + 128u);
    for (size_t layer = 0; layer < static_cast<size_t>(UINT16_MAX); ++layer)
        gcode += ";LAYER_CHANGE\n";
    gcode += ";MAX_LAYER_HEIGHT:0.4\n"
             ";LAYER_CHANGE\n"
             ";COLOR_CHANGE,T0,#FF0000\n"
             ";HEIGHT:0.2\n"
             "G1 X10 Y0 E1 F6000\n";

    const GCodeProcessorResult result = process_with_legacy_processor(gcode);
    const size_t marker = first_move_of_type(result, EMoveType::Color_change);
    const size_t extrusion = first_move_of_type(result, EMoveType::Extrude);
    const uint16_t capped_layer = static_cast<uint16_t>(UINT16_MAX);

    REQUIRE(marker < result.moves.size());
    REQUIRE(extrusion < result.moves.size());
    REQUIRE(result.layers_max_height.size() == static_cast<size_t>(UINT16_MAX) + 1u);
    CHECK(result.layers_max_height[capped_layer] == Approx(0.4));
    CHECK(result.moves[marker].layer_id == capped_layer);
    CHECK(result.moves[marker].height == Approx(0.4));
    CHECK(result.moves[marker].height == Approx(
        result.layers_max_height[result.moves[marker].layer_id]));
    CHECK(result.moves[extrusion].layer_id == capped_layer);
    CHECK(result.moves[extrusion].height == Approx(0.2));
}

TEST_CASE("Host G-code reader owns typed diagnostics in stream order",
          "[plugins][gcode][reader][viewer][diagnostic]")
{
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    slic3r_api::GCodeStateTracker state(initial);
    GCodeReaderResultBuilder builder(nullptr, nullptr, "test.reader", state);

    const std::vector<raw_gcode_reader_diagnostic_type> types = {
        RAW_GCODE_READER_DIAGNOSTIC_UNKNOWN_COMMAND,
        RAW_GCODE_READER_DIAGNOSTIC_INVALID_COMMAND_PARAMETERS,
        RAW_GCODE_READER_DIAGNOSTIC_UNKNOWN_TOOL,
        RAW_GCODE_READER_DIAGNOSTIC_AMBIGUOUS_TOOL,
        RAW_GCODE_READER_DIAGNOSTIC_UNSUPPORTED_STATE_EFFECT
    };
    const std::vector<std::string> messages = {
        "The G-code Reader encountered an unknown command.",
        "The G-code Reader encountered invalid command parameters.",
        "The G-code Reader could not identify the requested tool.",
        "The G-code Reader found several tools matching the requested tool.",
        "The G-code Reader recognized a command whose machine-state effect is not supported."
    };
    for (uint32_t index = 0; index < types.size(); ++index) {
        consume_builder_event(builder, state, RAW_GCODE_READER_EVENT_DIAGNOSTIC,
            {{RAW_GCODE_READER_ITEM_DIAGNOSTIC_TYPE, 0, double(types[index])},
             {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, double(index)}});
    }

    const GCodeProcessorResult result = builder.finish(state);
    REQUIRE(result.reader_diagnostics.size() == types.size());
    for (uint32_t index = 0; index < types.size(); ++index) {
        const GCodeProcessorResult::ReaderDiagnostic &diagnostic =
            result.reader_diagnostics[index];
        CHECK(diagnostic.type == types[index]);
        CHECK(diagnostic.line == index);
        CHECK(diagnostic.level == PrintStateBase::WarningLevel::NON_CRITICAL);
        CHECK(diagnostic.message == messages[index]);
    }
}

TEST_CASE("Host G-code reader forwards diagnostics to the active Print step",
          "[plugins][gcode][reader][viewer][diagnostic]")
{
    ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    Slic3r::Test::init_print(
        {make_cube(10.0, 10.0, 10.0)}, print, model,
        DynamicPrintConfig::full_print_config());
    print.set_running_step(STEP_GCODE);
    const PrintStateBase::TimeStamp started = print.step_state_with_timestamp(STEP_GCODE).timestamp;
    size_t warning_events = 0;
    print.set_status_callback([&](const PrintBase::SlicingStatus &event) {
        if (event.flags & PrintBase::SlicingStatus::UPDATE_PRINT_STEP_WARNINGS) {
            CHECK(event.warning_step == STEP_GCODE);
            CHECK(print.step_state_with_warnings(STEP_GCODE).warnings.size() == 1);
            ++warning_events;
        }
    });

    Printing::PrintingPlan &plan = print.mutable_printing_plan();
    PluginStorage storage;
    slic3r_api::Ordering::ObjectInstanceCatalogPlugin::populate_object_instance_catalog(
        reinterpret_cast<const print_handle *>(&print),
        reinterpret_cast<printing_plan_handle *>(&plan),
        reinterpret_cast<storage_handle *>(&storage));

    GCodeReaderService service(Orchestrator::instance(), &print);
    const GCodeProcessorResult result = service.read_for_viewer(
        "gcode.reader.default", "M9999\n");
    REQUIRE(result.reader_diagnostics.size() == 1);

    const PrintStateBase::StateWithWarnings step =
        print.step_state_with_warnings(STEP_GCODE);
    REQUIRE(step.warnings.size() == 1);
    CHECK(step.warnings.front().level ==
          PrintStateBase::WarningLevel::NON_CRITICAL);
    CHECK(step.warnings.front().message.find("gcode.reader.default") !=
          std::string::npos);
    CHECK(step.warnings.front().message.find("line 1") != std::string::npos);
    CHECK(step.timestamp > started);
    CHECK(warning_events == 1);
    CHECK(step.state == PrintStateBase::State::Started);
    print.set_running_step(STEP_NONE);
    print.mark_step_executed(STEP_GCODE);
    CHECK(print.is_step_done(STEP_GCODE));
}

TEST_CASE("Standalone host G-code reader diagnostics are transient host warnings",
          "[plugins][gcode][reader][viewer][diagnostic][host-attribution]")
{
    ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    orchestrator.consume_plugin_messages();
    orchestrator.reset_plugin_cancel();
    Print print;
    Model model;
    Slic3r::Test::init_print({make_cube(10.0, 10.0, 10.0)}, print, model,
                           DynamicPrintConfig::full_print_config());
    Printing::PrintingPlan &plan = print.mutable_printing_plan();
    PluginStorage storage;
    slic3r_api::Ordering::ObjectInstanceCatalogPlugin::populate_object_instance_catalog(
        reinterpret_cast<const print_handle *>(&print),
        reinterpret_cast<printing_plan_handle *>(&plan),
        reinterpret_cast<storage_handle *>(&storage));
    GCodeReaderService service(orchestrator, &print);
    const GCodeProcessorResult result = service.read_for_viewer("gcode.reader.default", "M9999\n");
    REQUIRE(result.reader_diagnostics.size() == 1);
    const std::vector<Orchestrator::PluginMessage> messages = orchestrator.consume_plugin_messages();
    REQUIRE(messages.size() == 1);
    CHECK(messages[0].plugin_id == "<host>");
    CHECK(messages[0].step == STEP_NONE);
    CHECK(messages[0].level == Orchestrator::PluginMessageLevel::Warning);
    CHECK(messages[0].message.find("gcode.reader.default") != std::string::npos);
    CHECK(messages[0].message.find("line 1") != std::string::npos);
    CHECK(print.step_state_with_warnings(STEP_GCODE).warnings.empty());
    CHECK(print.step_state_with_timestamp(STEP_GCODE).timestamp == 0);
    CHECK_FALSE(print.canceled());
    CHECK_FALSE(orchestrator.is_plugin_cancelled());
}

TEST_CASE("Host G-code reader initializes the built-in RepRap private schema",
          "[plugins][gcode][reader][viewer]")
{
    ensure_plugin_test_runtime_initialized();
    GCodeReaderService service(Orchestrator::instance(), nullptr);
    const slic3r_api::GCodeStateTracker initial =
        service.initial_state("gcode.reader.reprap");
    CHECK(initial.reader_id() == reprap_reader_state_id());
}

TEST_CASE("G-code reader C++ plugin helper rejects a null orchestrator",
          "[plugins][gcode][reader][service]")
{
    CHECK_THROWS_WITH(slic3r_api::GCodeReaderService(nullptr),
                      Catch::Matchers::Contains("requires an orchestrator handle"));
}

TEST_CASE("G-code reader execution rejects an unknown explicit provider",
          "[plugins][gcode][reader][selection]")
{
    ensure_plugin_test_runtime_initialized();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    CHECK_THROWS_WITH(read_gcode(Orchestrator::instance(), nullptr,
        "gcode.reader.missing", initial, "G1 X1\n"),
        Catch::Matchers::Contains("unavailable in the host activation set"));
}

TEST_CASE("G-code reader execution rejects inactive and wrong-step providers",
          "[plugins][gcode][reader][selection]")
{
    ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);

    REQUIRE(orchestrator.set_plugin_active("gcode.reader.test.a", false));
    CHECK_THROWS_WITH(read_gcode(orchestrator, nullptr,
        "gcode.reader.test.a", initial, "G1 X1\n"),
        Catch::Matchers::Contains("unavailable in the host activation set"));
    REQUIRE(orchestrator.set_plugin_active("gcode.reader.test.a", true));

    CHECK_THROWS_WITH(read_gcode(orchestrator, nullptr,
        "gcode.firmware.marlin1", initial, "G1 X1\n"),
        Catch::Matchers::Contains("is not a G-code reader"));
}

TEST_CASE("G-code reader keeps modal state across an M117 display message",
          "[plugins][gcode][reader]")
{
    ensure_plugin_test_runtime_initialized();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    ReaderProbe probe;
    probe.chunks = {"M117 Printing perimeter\n"};
    REQUIRE(execute_default_reader(probe, initial) == RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(probe.final_state);
    const slic3r_api::GCodeStateTracker &state = *probe.final_state;

    REQUIRE(probe.observed.size() == 1);
    CHECK(probe.observed.front().kind == RAW_GCODE_READER_EVENT_STATE_UPDATE);
    CHECK(state.x() == 1.0);
    CHECK(state.active_tool() == 0u);
    CHECK(state.line_number() == 1u);
}

TEST_CASE("G-code reader line number follows physical newlines", "[plugins][gcode][reader]")
{
    ensure_plugin_test_runtime_initialized();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    ReaderProbe probe;
    probe.chunks = {"\n; comment\r\nG1 X1\r\nG1 X2"};
    REQUIRE(execute_default_reader(probe, initial) == RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(probe.final_state);
    const slic3r_api::GCodeStateTracker &state = *probe.final_state;

    REQUIRE(probe.observed.size() == 2);
    CHECK(probe.observed[0].line_number == 2u);
    CHECK(probe.observed[1].line_number == 3u);
    CHECK(state.line_number() == 3u);
}

TEST_CASE("G-code reader requires a valid line number item", "[plugins][gcode][reader]")
{
    raw_gcode_reader_event_item missing_line_items[] = {
        {RAW_GCODE_READER_ITEM_X, 0, 1.0}
    };
    raw_gcode_reader_state missing_line = {};
    missing_line.items = missing_line_items;
    missing_line.item_count = 1;
    CHECK_THROWS_AS(slic3r_api::GCodeStateTracker(missing_line), std::invalid_argument);

    raw_gcode_reader_event missing_line_event = {};
    missing_line_event.kind = RAW_GCODE_READER_EVENT_STATE_UPDATE;
    missing_line_event.items = missing_line_items;
    missing_line_event.item_count = 1;
    CHECK_THROWS_AS(slic3r_api::GCodeStateTracker().apply(missing_line_event), std::invalid_argument);

    raw_gcode_reader_event_item invalid_line_items[] = {
        {RAW_GCODE_READER_ITEM_LINE_NUMBER, 1, 1.0}
    };
    raw_gcode_reader_state invalid_line = {};
    invalid_line.items = invalid_line_items;
    invalid_line.item_count = 1;
    CHECK_THROWS_AS(slic3r_api::GCodeStateTracker(invalid_line), std::invalid_argument);

    invalid_line_items[0] = {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 0.5};
    CHECK_THROWS_AS(slic3r_api::GCodeStateTracker(invalid_line), std::invalid_argument);
}

TEST_CASE("G-code reader markers are typed event-only attributes",
          "[plugins][gcode][reader]")
{
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    slic3r_api::GCodeStateTracker state(initial);

    const raw_gcode_reader_event_item valid_items[] = {
        {RAW_GCODE_READER_ITEM_MARKER, 0, RAW_GCODE_READER_MARKER_PAUSE_PRINT},
        {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 1.0}
    };
    const raw_gcode_reader_event valid = {
        RAW_GCODE_READER_EVENT_MARKER, valid_items, 2
    };
    state.apply(valid);
    CHECK_FALSE(state.known(RAW_GCODE_READER_ITEM_MARKER));

    const raw_gcode_reader_event_item line_only[] = {
        {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 2.0}
    };
    const raw_gcode_reader_event missing_type = {
        RAW_GCODE_READER_EVENT_MARKER, line_only, 1
    };
    CHECK_THROWS_AS(state.apply(missing_type), std::invalid_argument);

    const raw_gcode_reader_event wrong_kind = {
        RAW_GCODE_READER_EVENT_STATE_UPDATE, valid_items, 2
    };
    CHECK_THROWS_AS(state.apply(wrong_kind), std::invalid_argument);

    raw_gcode_reader_event_item invalid_items[] = {
        {RAW_GCODE_READER_ITEM_MARKER, 0,
         double(RAW_GCODE_READER_MARKER_CUSTOM_GCODE) + 1.0},
        {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 2.0}
    };
    const raw_gcode_reader_event invalid_type = {
        RAW_GCODE_READER_EVENT_MARKER, invalid_items, 2
    };
    CHECK_THROWS_AS(state.apply(invalid_type), std::invalid_argument);

    raw_gcode_reader_state invalid_state = initial;
    invalid_state.items = valid_items;
    invalid_state.item_count = 2;
    CHECK_THROWS_AS(slic3r_api::GCodeStateTracker(invalid_state),
                    std::invalid_argument);
}

TEST_CASE("G-code reader diagnostics are typed event-only attributes",
          "[plugins][gcode][reader][diagnostic]")
{
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    slic3r_api::GCodeStateTracker state(initial);

    for (uint32_t type = RAW_GCODE_READER_DIAGNOSTIC_UNKNOWN_COMMAND;
         type <= RAW_GCODE_READER_DIAGNOSTIC_UNSUPPORTED_STATE_EFFECT;
         ++type) {
        const raw_gcode_reader_event_item valid_items[] = {
            {RAW_GCODE_READER_ITEM_DIAGNOSTIC_TYPE, 0, double(type)},
            {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, double(type)}
        };
        const raw_gcode_reader_event valid = {
            RAW_GCODE_READER_EVENT_DIAGNOSTIC, valid_items, 2
        };
        state.apply(valid);
        CHECK_FALSE(state.known(RAW_GCODE_READER_ITEM_DIAGNOSTIC_TYPE));
    }

    const raw_gcode_reader_event_item line_only[] = {
        {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 6.0}
    };
    const raw_gcode_reader_event missing_type = {
        RAW_GCODE_READER_EVENT_DIAGNOSTIC, line_only, 1
    };
    CHECK_THROWS_AS(state.apply(missing_type), std::invalid_argument);

    raw_gcode_reader_event_item diagnostic_items[] = {
        {RAW_GCODE_READER_ITEM_DIAGNOSTIC_TYPE, 0,
         RAW_GCODE_READER_DIAGNOSTIC_UNKNOWN_COMMAND},
        {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 6.0}
    };
    const raw_gcode_reader_event wrong_kind = {
        RAW_GCODE_READER_EVENT_STATE_UPDATE, diagnostic_items, 2
    };
    CHECK_THROWS_AS(state.apply(wrong_kind), std::invalid_argument);

    diagnostic_items[0].target = 1;
    const raw_gcode_reader_event invalid_target = {
        RAW_GCODE_READER_EVENT_DIAGNOSTIC, diagnostic_items, 2
    };
    CHECK_THROWS_AS(state.apply(invalid_target), std::invalid_argument);
    diagnostic_items[0].target = 0;

    diagnostic_items[0].value = 1.5;
    const raw_gcode_reader_event non_integral = {
        RAW_GCODE_READER_EVENT_DIAGNOSTIC, diagnostic_items, 2
    };
    CHECK_THROWS_AS(state.apply(non_integral), std::invalid_argument);
    diagnostic_items[0].value =
        double(RAW_GCODE_READER_DIAGNOSTIC_UNSUPPORTED_STATE_EFFECT) + 1.0;
    const raw_gcode_reader_event unknown_type = {
        RAW_GCODE_READER_EVENT_DIAGNOSTIC, diagnostic_items, 2
    };
    CHECK_THROWS_AS(state.apply(unknown_type), std::invalid_argument);

    diagnostic_items[0].value = RAW_GCODE_READER_DIAGNOSTIC_UNKNOWN_COMMAND;
    const raw_gcode_reader_event_item duplicate_items[] = {
        diagnostic_items[0], diagnostic_items[0], diagnostic_items[1]
    };
    const raw_gcode_reader_event duplicate_type = {
        RAW_GCODE_READER_EVENT_DIAGNOSTIC, duplicate_items, 3
    };
    CHECK_THROWS_AS(state.apply(duplicate_type), std::invalid_argument);

    raw_gcode_reader_state invalid_state = initial;
    invalid_state.items = diagnostic_items;
    invalid_state.item_count = 2;
    CHECK_THROWS_AS(slic3r_api::GCodeStateTracker(invalid_state),
                    std::invalid_argument);
}

TEST_CASE("G-code reader normalizes and validates its provider identity",
          "[plugins][gcode][reader][state]")
{
    std::vector<raw_gcode_reader_event_item> items;
    const raw_gcode_reader_state initial = initial_state(items);
    slic3r_api::GCodeStateTracker normalized(initial);
    CHECK(normalized.reader_id() == 0u);

    normalized.set_reader_id(42u);
    CHECK(normalized.reader_id() == 42u);

    raw_gcode_reader_event_item invalid[] = {
        {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 0.0},
        {RAW_GCODE_READER_ITEM_READER_ID, 1, 42.0}
    };
    raw_gcode_reader_state state = {invalid, 2};
    CHECK_THROWS_AS(slic3r_api::GCodeStateTracker(state), std::invalid_argument);
    invalid[1] = {RAW_GCODE_READER_ITEM_READER_ID, 0, 0.5};
    CHECK_THROWS_AS(slic3r_api::GCodeStateTracker(state), std::invalid_argument);
}

TEST_CASE("G-code reader transports provider-private scalar state opaquely",
          "[plugins][gcode][reader][state]")
{
    std::vector<raw_gcode_reader_event_item> items;
    raw_gcode_reader_state initial = initial_state(items);
    const uint32_t private_type = RAW_GCODE_READER_ITEM_CUSTOM_BEGIN + 17u;
    items.push_back({private_type, 93u, 12.5});
    initial.items = items.data();
    initial.item_count = static_cast<uint32_t>(items.size());

    slic3r_api::GCodeStateTracker state(initial);
    CHECK(state.value(private_type, 93u) == 12.5);
    state.set_custom_value(private_type, 94u, -8.25);
    CHECK(state.value(private_type, 94u) == -8.25);
    state.erase_custom_value(private_type, 93u);
    CHECK_FALSE(state.known(private_type, 93u));

    slic3r_api::GCodeStateTracker copied(state);
    slic3r_api::GCodeStateTracker moved(std::move(copied));
    CHECK(moved.value(private_type, 94u) == -8.25);
    const raw_gcode_reader_event_item update_items[] = {
        {private_type, 94u, 7.0},
        {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 1.0}
    };
    raw_gcode_reader_event update = {
        RAW_GCODE_READER_EVENT_STATE_UPDATE, update_items, 2
    };
    moved.apply(update);
    CHECK(moved.value(private_type, 94u) == 7.0);
    raw_gcode_reader_event invalidate = {
        RAW_GCODE_READER_EVENT_INVALIDATE, update_items, 2
    };
    moved.apply(invalidate);
    CHECK_FALSE(moved.known(private_type, 94u));

    items.back().value = std::numeric_limits<double>::infinity();
    CHECK_THROWS_AS(slic3r_api::GCodeStateTracker(initial), std::invalid_argument);
}

TEST_CASE("G-code reader rejects a state owned by another Reader identity",
          "[plugins][gcode][reader][state]")
{
    const uint32_t expected_id = reprap_reader_state_id();
    REQUIRE(expected_id >= ORCHESTRATOR_RUNTIME_ID_DYNAMIC_BEGIN);

    SECTION("missing identity normalizes to the portable schema") {
        slic3r_api::GCodeStateTracker tracker;
        CHECK(tracker.reader_id() == GCODE_READER_STATE_ID_PORTABLE);
        CHECK_THROWS_WITH(RepRapGCodeReaderState::from_tracker(tracker, expected_id),
                          Catch::Matchers::Contains("does not belong to RepRapFirmware"));
    }
    SECTION("foreign dynamic identity") {
        slic3r_api::GCodeStateTracker tracker;
        tracker.set_reader_id(expected_id + 1u);
        CHECK_THROWS_WITH(RepRapGCodeReaderState::from_tracker(tracker, expected_id),
                          Catch::Matchers::Contains("does not belong to RepRapFirmware"));

        RepRapGCodeReaderState state;
        CHECK_THROWS_WITH(state.write_to(tracker, expected_id),
                          Catch::Matchers::Contains("another Reader identity"));
    }
}

TEST_CASE("RepRap reader private state round-trips mappings and preserves unknown keys",
          "[plugins][gcode][reader][reprap][state]")
{
    using namespace slic3r_api::GCodeGeneration::Reader;
    const uint32_t reader_state_id = reprap_reader_state_id();
    slic3r_api::GCodeStateTracker tracker;
    tracker.set_reader_id(reader_state_id);
    const uint32_t future_key = RAW_GCODE_READER_ITEM_CUSTOM_BEGIN + 0x700000u;
    tracker.set_custom_value(future_key, 4u, 19.0);

    RepRapGCodeReaderState written;
    written.print_acceleration = 800.0;
    written.travel_acceleration = 1500.0;
    written.max_feedrate[0] = 300.0;
    written.pressure_advance[7] = 0.035;
    RepRapToolState &tool = written.tools[2];
    tool.drives = {7, 8};
    tool.heaters = {3};
    tool.fans = {1, 4};
    tool.standby_temperature = 145.0;
    tool.offset = {1.5, -2.0, 0.25};
    tool.retracted = true;
    written.tools[9];
    written.write_to(tracker, reader_state_id);

    const RepRapGCodeReaderState decoded =
        RepRapGCodeReaderState::from_tracker(tracker, reader_state_id);
    CHECK(decoded.print_acceleration == 800.0);
    CHECK(decoded.travel_acceleration == 1500.0);
    CHECK(decoded.max_feedrate[0] == 300.0);
    CHECK(decoded.pressure_advance.at(7) == Approx(0.035));
    REQUIRE(decoded.tools.count(2) == 1);
    CHECK(decoded.tools.at(2).drives == std::vector<uint32_t>{7, 8});
    CHECK(decoded.tools.at(2).heaters == std::vector<uint32_t>{3});
    CHECK(decoded.tools.at(2).fans == std::vector<uint32_t>{1, 4});
    CHECK(decoded.tools.at(2).standby_temperature == 145.0);
    CHECK(decoded.tools.at(2).offset[1] == -2.0);
    CHECK(decoded.tools.at(2).retracted == true);
    REQUIRE(decoded.tools.count(9) == 1);
    CHECK(decoded.tools.at(9).drives.empty());
    CHECK(tracker.value(future_key, 4u) == 19.0);
}

TEST_CASE("RepRap reader private mappings reject malformed slot lists",
          "[plugins][gcode][reader][reprap][state]")
{
    using namespace slic3r_api::GCodeGeneration::Reader;
    const uint32_t reader_state_id = reprap_reader_state_id();
    SECTION("missing slot") {
        slic3r_api::GCodeStateTracker tracker;
        tracker.set_reader_id(reader_state_id);
        tracker.set_custom_value(RRF_ITEM_TOOL_DRIVE_COUNT, 0, 2.0);
        tracker.set_custom_value(RRF_ITEM_TOOL_DRIVE_SLOT_BEGIN, 0, 3.0);
        CHECK_THROWS_WITH(RepRapGCodeReaderState::from_tracker(tracker, reader_state_id),
                          Catch::Matchers::Contains("missing indexed slot"));
    }
    SECTION("duplicate device") {
        slic3r_api::GCodeStateTracker tracker;
        tracker.set_reader_id(reader_state_id);
        tracker.set_custom_value(RRF_ITEM_TOOL_DRIVE_COUNT, 0, 2.0);
        tracker.set_custom_value(RRF_ITEM_TOOL_DRIVE_SLOT_BEGIN, 0, 3.0);
        tracker.set_custom_value(RRF_ITEM_TOOL_DRIVE_SLOT_BEGIN + 1u, 0, 3.0);
        CHECK_THROWS_WITH(RepRapGCodeReaderState::from_tracker(tracker, reader_state_id),
                          Catch::Matchers::Contains("duplicate device"));
    }
    SECTION("slot beyond count") {
        slic3r_api::GCodeStateTracker tracker;
        tracker.set_reader_id(reader_state_id);
        tracker.set_custom_value(RRF_ITEM_TOOL_DRIVE_COUNT, 0, 1.0);
        tracker.set_custom_value(RRF_ITEM_TOOL_DRIVE_SLOT_BEGIN, 0, 3.0);
        tracker.set_custom_value(RRF_ITEM_TOOL_DRIVE_SLOT_BEGIN + 1u, 0, 4.0);
        CHECK_THROWS_WITH(RepRapGCodeReaderState::from_tracker(tracker, reader_state_id),
                          Catch::Matchers::Contains("beyond its count"));
    }
    SECTION("invalid count") {
        slic3r_api::GCodeStateTracker tracker;
        tracker.set_reader_id(reader_state_id);
        tracker.set_custom_value(RRF_ITEM_TOOL_DRIVE_COUNT, 0, -1.0);
        CHECK_THROWS_WITH(RepRapGCodeReaderState::from_tracker(tracker, reader_state_id),
                          Catch::Matchers::Contains("exact uint32"));
        tracker.set_custom_value(RRF_ITEM_TOOL_DRIVE_COUNT, 0, 1.5);
        CHECK_THROWS_WITH(RepRapGCodeReaderState::from_tracker(tracker, reader_state_id),
                          Catch::Matchers::Contains("exact uint32"));
    }
    SECTION("invalid device id") {
        slic3r_api::GCodeStateTracker tracker;
        tracker.set_reader_id(reader_state_id);
        tracker.set_custom_value(RRF_ITEM_TOOL_DRIVE_COUNT, 0, 1.0);
        tracker.set_custom_value(RRF_ITEM_TOOL_DRIVE_SLOT_BEGIN, 0, 2.5);
        CHECK_THROWS_WITH(RepRapGCodeReaderState::from_tracker(tracker, reader_state_id),
                          Catch::Matchers::Contains("exact uint32"));
    }
}

TEST_CASE("RepRap reader parses its operational private state",
          "[plugins][gcode][reader][reprap][state]")
{
    ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const uint32_t reader_state_id = reprap_reader_state_id();
    const raw_gcode_reader_state initial = initial_state(initial_items, reader_state_id);

    const slic3r_api::GCodeStateTracker tracked = read_gcode(
        orchestrator, nullptr, "gcode.reader.reprap", initial,
        "M563 P2 D3:4 H5 F6:7\n"
        "T2\n"
        "G10 P2 S210 R145 X1.5 Y-2 Z0.25\n"
        "M201 X1000 Y1100 Z100 E1200\n"
        "M203 X18000 Y12000 Z600 E900 I60\n"
        "M205 X8 Y9 Z0.4 E2\n"
        "M566 X600 Y660 Z30 E120\n"
        "M204 P800 T1500\n"
        "M568 P2 S220 R150\n"
        "M572 D3 S0.035\n"
        "M106 S0.25\n"
        "G10\nG11\nM116\n");

    const RepRapGCodeReaderState state =
        RepRapGCodeReaderState::from_tracker(tracked, reader_state_id);
    REQUIRE(state.tools.count(2) == 1);
    const RepRapToolState &tool = state.tools.at(2);
    CHECK(tool.drives == std::vector<uint32_t>{3, 4});
    CHECK(tool.heaters == std::vector<uint32_t>{5});
    CHECK(tool.fans == std::vector<uint32_t>{6, 7});
    CHECK(tool.standby_temperature == 150.0);
    CHECK(tool.offset[0] == 1.5);
    CHECK(tool.offset[1] == -2.0);
    CHECK(tool.offset[2] == 0.25);
    CHECK(tool.retracted == false);
    CHECK(state.max_acceleration[1] == 1100.0);
    CHECK(state.max_feedrate[0] == 300.0);
    CHECK(state.min_extruding_feedrate == 1.0);
    CHECK(state.job_jerk[0] == 8.0);
    CHECK(state.machine_jerk[1] == 11.0);
    CHECK(state.print_acceleration == 800.0);
    CHECK(state.travel_acceleration == 1500.0);
    CHECK(state.pressure_advance.at(3) == Approx(0.035));
    CHECK(tracked.tool_temperature(2) == 220.0);
    CHECK(tracked.fan_speed(6) == Approx(25.0));
    CHECK(tracked.fan_speed(7) == Approx(25.0));
}

TEST_CASE("Built-in Reader dispatches common commands through virtual handlers",
          "[plugins][gcode][reader][dispatch]")
{
    ensure_plugin_test_runtime_initialized();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    ReaderProbe probe;
    probe.chunks = {"G91\nM220 S75\n"};

    REQUIRE(execute_reader("gcode.reader.test.common_handlers", probe, initial) ==
            RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(probe.final_state);
    REQUIRE(probe.observed.size() == 3);
    CHECK(probe.final_state->value(RAW_GCODE_READER_ITEM_XYZ_ABSOLUTE) == 1.0);
    CHECK(probe.final_state->value(RAW_GCODE_READER_ITEM_SPEED_FACTOR) == 75.0);
    CHECK(probe.observed[0].kind == RAW_GCODE_READER_EVENT_STATE_UPDATE);
    CHECK(probe.observed[1].kind == RAW_GCODE_READER_EVENT_STATE_UPDATE);
    CHECK(probe.observed[2].kind == RAW_GCODE_READER_EVENT_DIAGNOSTIC);
}

TEST_CASE("RepRap Reader dispatches one overridden dialect handler",
          "[plugins][gcode][reader][dispatch][reprap]")
{
    ensure_plugin_test_runtime_initialized();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const uint32_t reader_state_id = reprap_reader_state_id();
    const raw_gcode_reader_state initial = initial_state(initial_items, reader_state_id);
    ReaderProbe probe;
    probe.chunks = {"G10\nM201 X1200\n"};

    REQUIRE(execute_reader("gcode.reader.test.reprap_handlers", probe, initial) ==
            RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(probe.final_state);
    REQUIRE(probe.observed.size() == 2);
    CHECK(probe.observed[0].kind == RAW_GCODE_READER_EVENT_DIAGNOSTIC);
    CHECK(probe.observed[1].kind == RAW_GCODE_READER_EVENT_STATE_UPDATE);
    const RepRapGCodeReaderState state =
        RepRapGCodeReaderState::from_tracker(*probe.final_state, reader_state_id);
    CHECK(state.max_acceleration[0] == 1200.0);
}

TEST_CASE("Marlin Reader chains common and overridden dialect handlers",
          "[plugins][gcode][reader][dispatch][marlin]")
{
    ensure_plugin_test_runtime_initialized();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    ReaderProbe probe;
    probe.chunks = {"G91\nM204 S500\nM700\n"};

    REQUIRE(execute_reader("gcode.reader.test.marlin_handlers", probe, initial) ==
            RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(probe.final_state);
    REQUIRE(probe.observed.size() == 3);
    CHECK(probe.final_state->value(RAW_GCODE_READER_ITEM_XYZ_ABSOLUTE) == 0.0);
    CHECK(probe.final_state->value(RAW_GCODE_READER_ITEM_ACCELERATION) == 204.0);
    CHECK(probe.final_state->value(RAW_GCODE_READER_ITEM_SPEED_FACTOR) == 700.0);
}

TEST_CASE("Klipper Reader chains textual and numeric virtual handlers",
          "[plugins][gcode][reader][dispatch][klipper]")
{
    ensure_plugin_test_runtime_initialized();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    ReaderProbe probe;
    probe.chunks = {"G91\nPAUSE\nTEST_EXTENSION\nM204 S500\n"};

    REQUIRE(execute_reader("gcode.reader.test.klipper_handlers", probe, initial) ==
            RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(probe.final_state);
    REQUIRE(probe.observed.size() == 4);
    CHECK(probe.observed[1].kind == RAW_GCODE_READER_EVENT_DIAGNOSTIC);
    CHECK(probe.final_state->value(RAW_GCODE_READER_ITEM_XYZ_ABSOLUTE) == 0.0);
    CHECK(probe.final_state->value(RAW_GCODE_READER_ITEM_EXTRUSION_FACTOR) == 204.0);
    CHECK(probe.final_state->value(RAW_GCODE_READER_ITEM_SPEED_FACTOR) == 701.0);
}

TEST_CASE("G-code readers preserve opaque state only while commands stay understood",
          "[plugins][gcode][reader][state]")
{
    ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    std::vector<raw_gcode_reader_event_item> items;
    raw_gcode_reader_state initial = initial_state(items);
    const uint32_t private_type = RAW_GCODE_READER_ITEM_CUSTOM_BEGIN + 0x700001u;
    items.push_back({private_type, 12u, 3.5});
    initial.items = items.data();
    initial.item_count = static_cast<uint32_t>(items.size());

    const slic3r_api::GCodeStateTracker transported = read_gcode(
        orchestrator, nullptr, "gcode.reader.default", initial, "G1 X2\n");
    CHECK(transported.value(private_type, 12u) == 3.5);

    const slic3r_api::GCodeStateTracker invalidated = read_gcode(
        orchestrator, nullptr, "gcode.reader.default", initial, "M9999\n");
    CHECK_FALSE(invalidated.known(private_type, 12u));
    CHECK(invalidated.reader_id() == 0u);

    // Klipper keeps its textual object registry outside the tracker. Object
    // identities use the same zero-based convention as the plan; UINT32_MAX
    // remains reserved for outside work.
    std::vector<raw_gcode_reader_event_item> klipper_items;
    raw_gcode_reader_state klipper_initial = initial_state(klipper_items);
    klipper_items.push_back(
        {RAW_GCODE_READER_ITEM_CUSTOM_BEGIN, 1u, 42.0});
    klipper_initial.items = klipper_items.data();
    klipper_initial.item_count =
        static_cast<uint32_t>(klipper_items.size());
    const slic3r_api::GCodeStateTracker klipper = read_gcode(
        orchestrator, nullptr, "gcode.reader.klipper", klipper_initial,
        "EXCLUDE_OBJECT_DEFINE NAME='alpha'\n"
        "EXCLUDE_OBJECT_START NAME='alpha'\n");
    CHECK(klipper.value(RAW_GCODE_READER_ITEM_CUSTOM_BEGIN, 1u) == 42.0);
    CHECK(klipper.value(RAW_GCODE_READER_ITEM_OBJECT) == 0.0);
}

TEST_CASE("G-code reader tracker copies own their raw state view",
          "[plugins][gcode][reader]")
{
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    slic3r_api::GCodeStateTracker original(initial);
    slic3r_api::GCodeStateTracker copied(original);

    REQUIRE(original.state().items != nullptr);
    REQUIRE(copied.state().items != nullptr);
    CHECK(copied.state().items != original.state().items);
    original.set_line_number(12);
    CHECK(copied.line_number() == 0u);

    const raw_gcode_reader_event_item *copied_items = copied.state().items;
    slic3r_api::GCodeStateTracker moved(std::move(copied));
    REQUIRE(moved.state().items != nullptr);
    CHECK(moved.state().items != copied_items);
    CHECK(moved.line_number() == 0u);

    slic3r_api::GCodeStateTracker assigned;
    assigned = moved;
    REQUIRE(assigned.state().items != nullptr);
    CHECK(assigned.state().items != moved.state().items);
    CHECK(assigned.line_number() == 0u);
}

TEST_CASE("G-code reader reconstructs lines split across borrowed chunks",
          "[plugins][gcode][reader]")
{
    ensure_plugin_test_runtime_initialized();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    const std::vector<std::string_view> chunks = {
        "G91\r", "\nG1 X", "2 E1\nG", "90\nG1 X5"
    };
    size_t next = 0;
    GCodeReaderChunkSource source = [&chunks, &next]() -> std::string_view {
        return next < chunks.size() ? chunks[next++] : std::string_view();
    };

    const slic3r_api::GCodeStateTracker state = read_gcode(
        Orchestrator::instance(), nullptr, "gcode.reader.default", initial, std::move(source));

    REQUIRE(state.x());
    REQUIRE(state.e(0));
    CHECK(*state.x() == Approx(5.0));
    CHECK(*state.e(0) == Approx(1.0));
    CHECK(state.line_number() == 3u);
    CHECK(next == chunks.size());
}

// Verify that the state tracker rejects a standard item identifier beyond its declared range.
TEST_CASE("G-code reader rejects item types outside its declared vocabulary",
          "[plugins][gcode][reader]")
{
    std::vector<raw_gcode_reader_event_item> items;
    raw_gcode_reader_state state = initial_state(items);
    items.push_back({RAW_GCODE_READER_ITEM_MAX_LAYER_HEIGHT + 1u, 0, 1.0});
    state.items = items.data();
    state.item_count = static_cast<uint32_t>(items.size());
    CHECK_THROWS_AS(slic3r_api::GCodeStateTracker(state), std::invalid_argument);
}

TEST_CASE("G-code reader uses common plugin cancellation before reading",
          "[plugins][gcode][reader]")
{
    ensure_plugin_test_runtime_initialized();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    ReaderProbe probe;
    probe.chunks = {"G1 X1\n"};

    Orchestrator &orchestrator = Orchestrator::instance();
    orchestrator.request_plugin_cancel();
    const raw_plugin_execution_status status = execute_default_reader(probe, initial);
    orchestrator.reset_plugin_cancel();

    CHECK(status == RAW_PLUGIN_EXECUTION_CANCELLED);
    CHECK(probe.next_chunk == 0);
    CHECK_FALSE(probe.final_state);
}

TEST_CASE("G-code reader uses common plugin cancellation between chunks",
          "[plugins][gcode][reader]")
{
    ensure_plugin_test_runtime_initialized();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    ReaderProbe probe;
    probe.chunks = {"G91\n", "G1 X2\n"};
    probe.cancel_after_chunks = 1;

    Orchestrator &orchestrator = Orchestrator::instance();
    orchestrator.reset_plugin_cancel();
    const raw_plugin_execution_status status = execute_default_reader(probe, initial);
    orchestrator.reset_plugin_cancel();

    CHECK(status == RAW_PLUGIN_EXECUTION_CANCELLED);
    CHECK(probe.next_chunk == 1);
    CHECK_FALSE(probe.final_state);
}

TEST_CASE("G-code reader checks cancellation within one borrowed chunk",
          "[plugins][gcode][reader][cancellation]")
{
    ensure_plugin_test_runtime_initialized();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    std::string source = "G1 X1\n";
    for (size_t line = 0; line < 128; ++line)
        source += "; long comment\n";
    source += "G1 X9\n";

    ReaderProbe probe;
    probe.chunks = {source};
    probe.cancel_after_events = 1;

    Orchestrator &orchestrator = Orchestrator::instance();
    orchestrator.reset_plugin_cancel();
    const raw_plugin_execution_status status = execute_default_reader(probe, initial);
    orchestrator.reset_plugin_cancel();

    CHECK(status == RAW_PLUGIN_EXECUTION_CANCELLED);
    CHECK(probe.next_chunk == 1);
    CHECK(probe.observed.size() == 1);
    CHECK_FALSE(probe.final_state);
}

} // namespace

namespace Slic3r::Test::Plugins {

void register_gcode_reader_test_providers(orchestrator_handle *orchestrator)
{
    static ProbeTestReader lexical_first(orchestrator, "gcode.reader.test.a", 10, false);
    static ProbeTestReader lexical_second(orchestrator, "gcode.reader.test.b", 10, false);
    static ProbeTestReader lower_priority(orchestrator, "gcode.reader.test.lower_priority", 20, false);
    static ProbeTestReader failing_probe(orchestrator, "gcode.reader.test.failing_probe", 0, true);
    static CommonHandlerOverrideReader common_handlers(orchestrator);
    static RepRapHandlerOverrideReader reprap_handlers(orchestrator);
    static MarlinHandlerOverrideReader marlin_handlers(orchestrator);
    static KlipperHandlerOverrideReader klipper_handlers(orchestrator);
    // Register lexical_second first so the tie-break test also proves that
    // discovery order cannot change the selected id.
    orchestrator_register_plugin(orchestrator, lexical_second.c_instance());
    orchestrator_register_plugin(orchestrator, lexical_first.c_instance());
    orchestrator_register_plugin(orchestrator, lower_priority.c_instance());
    orchestrator_register_plugin(orchestrator, failing_probe.c_instance());
    orchestrator_register_plugin(orchestrator, common_handlers.c_instance());
    orchestrator_register_plugin(orchestrator, reprap_handlers.c_instance());
    orchestrator_register_plugin(orchestrator, marlin_handlers.c_instance());
    orchestrator_register_plugin(orchestrator, klipper_handlers.c_instance());
}

} // namespace Slic3r::Test::Plugins
