///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "GCodeReaderService.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <boost/log/trivial.hpp>

#include "libslic3r/Api/host/GCodeReaderResultBuilder.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_gcode_reader.h"
#include "libslic3r/Print.hpp"
#include "libslic3r/Plugins/GCode/Reader/BambuGCodeReaderItems.hpp"
#include "libslic3r/Plugins/GCode/Reader/RepRapGCodeReaderState.hpp"

/*
Host G-code reader service implementation
==========================================

The plugin-side GCodeReaderService owns the synchronous path which only needs a
final state: it supplies chunks, validates callbacks and copies that state. The
host facade also probes files whose dialect is unknown and owns a second
invocation path which forwards each validated event to the viewer builder.

    final state only --> plugin helper --> exact Reader plugin
    unknown dialect --> host probes ----> selected Reader id

Viewer reads additionally wrap that same execution with a
GCodeReaderResultBuilder. The wrapper indexes source lines before lending each
chunk and observes validated events after they have updated the tracker.

Operation notes:
- viewer result --:  host invocation :  event builder

Representative direct calls (branches are not a timeline):

    detect_gcode_reader()
        |-- orchestrator.get_active_plugins_for_step()
        |-- orchestrator.execute_plugin()
        |-- invocation.confidence()
        |-- plugin->get_priority()
        `-- winner->get_priority()

    GCodeReaderService::read()
        `-- m_reader.read()

    GCodeReaderService::read_for_viewer()
        |-- validated_reader_id()
        |-- initial_state()
        |-- m_orchestrator.execute_plugin()
        `-- invocation.take_result()
*/

namespace Slic3r {

slic3r_api::GCodeStateTracker read_gcode(
    Orchestrator &orchestrator,
    Print *print,
    std::string_view reader_plugin_id,
    const raw_gcode_reader_state &initial_state,
    GCodeReaderChunkSource source)
{
    return slic3r_api::GCodeReaderService(
        reinterpret_cast<orchestrator_handle *>(&orchestrator),
        reinterpret_cast<const print_handle *>(print)).read(
            reader_plugin_id, initial_state, std::move(source));
}

slic3r_api::GCodeStateTracker read_gcode(
    Orchestrator &orchestrator,
    Print *print,
    std::string_view reader_plugin_id,
    const raw_gcode_reader_state &initial_state,
    std::string_view text)
{
    return slic3r_api::GCodeReaderService(
        reinterpret_cast<orchestrator_handle *>(&orchestrator),
        reinterpret_cast<const print_handle *>(print)).read(
            reader_plugin_id, initial_state, text);
}

namespace {

class ProbeInvocation
{
public:
    ProbeInvocation(std::string_view prefix, const Print *print);
    run_ctx_gcode_reader context();
    uint32_t confidence() const;

private:
    static void next_callback(void *opaque, raw_gcode_reader_chunk *chunk,
                              raw_gcode_reader_result *result) noexcept;

    static void probe_callback(void *opaque, uint32_t confidence,
                               raw_gcode_reader_result *result) noexcept;

    std::string_view m_prefix;
    const Print *m_print;
    uint32_t m_confidence = 0;
    bool m_supplied = false;
    bool m_reported = false;
};

// Own the host callbacks for one viewer read. This host-only invocation keeps
// the public plugin helper unchanged while making every validated event
// available to GCodeReaderResultBuilder before borrowed callback data expires.
class ViewerReaderInvocation
{
public:
    ViewerReaderInvocation(GCodeReaderChunkSource source,
                           const Print *print,
                           const raw_gcode_reader_state &initial_state,
                           GCodeReaderResultBuilder &builder);
    run_ctx_gcode_reader context();
    GCodeProcessorResult take_result();
    const std::string &error() const;

private:
    static void next_callback(void *opaque, raw_gcode_reader_chunk *chunk,
                              raw_gcode_reader_result *result) noexcept;
    static void event_callback(void *opaque, const raw_gcode_reader_event *event,
                               raw_gcode_reader_result *result) noexcept;
    static void final_callback(void *opaque, const raw_gcode_reader_state *state,
                               raw_gcode_reader_result *result) noexcept;
    static void initialize_success(raw_gcode_reader_result *result) noexcept;

    void next(raw_gcode_reader_chunk &chunk);
    void event(const raw_gcode_reader_event &event);
    void final_state(const raw_gcode_reader_state &state);
    void fail(raw_gcode_reader_result *result, raw_gcode_reader_status status,
              const std::string &message) noexcept;

    GCodeReaderChunkSource m_source;
    const Print *m_print;
    GCodeReaderResultBuilder &m_builder;
    slic3r_api::GCodeStateTracker m_initial_state;
    slic3r_api::GCodeStateTracker m_event_state;
    std::optional<slic3r_api::GCodeStateTracker> m_final_state;
    std::string_view m_active_chunk;
    std::string m_error;
    bool m_end_reached = false;
};

// Translate the generic plugin execution status at the host boundary.
std::string viewer_execution_error(raw_plugin_execution_status status,
                                   const std::string &id);

ProbeInvocation::ProbeInvocation(std::string_view prefix, const Print *print)
    : m_prefix(prefix), m_print(print)
{}

run_ctx_gcode_reader ProbeInvocation::context()
{
    run_ctx_gcode_reader value = {};
    value.operation = RAW_GCODE_READER_OPERATION_PROBE;
    value.print = reinterpret_cast<const print_handle *>(m_print);
    value.callback_context = this;
    value.get_next_chunk = &ProbeInvocation::next_callback;
    value.emit_probe = &ProbeInvocation::probe_callback;
    return value;
}

uint32_t ProbeInvocation::confidence() const
{
    if (!m_reported)
        throw std::runtime_error("The G-code reader probe returned without a confidence result.");
    return m_confidence;
}

void ProbeInvocation::next_callback(void *opaque, raw_gcode_reader_chunk *chunk,
                                    raw_gcode_reader_result *result) noexcept
{
    ProbeInvocation *invocation = static_cast<ProbeInvocation *>(opaque);
    if (invocation == nullptr || chunk == nullptr || result == nullptr)
        return;
    *chunk = {};
    if (!invocation->m_supplied) {
        invocation->m_supplied = true;
        if (!invocation->m_prefix.empty()) {
            chunk->bytes = invocation->m_prefix.data();
            chunk->byte_count = static_cast<uint64_t>(invocation->m_prefix.size());
        }
    }
    result->status = RAW_GCODE_READER_STATUS_SUCCESS;
    result->error_message = nullptr;
}

void ProbeInvocation::probe_callback(void *opaque, uint32_t confidence,
                                     raw_gcode_reader_result *result) noexcept
{
    ProbeInvocation *invocation = static_cast<ProbeInvocation *>(opaque);
    if (invocation == nullptr || result == nullptr)
        return;
    if (invocation->m_reported || confidence > 100) {
        result->status = RAW_GCODE_READER_STATUS_INVALID_ARGUMENT;
        result->error_message = "A G-code reader probe published an invalid confidence result.";
        return;
    }
    invocation->m_reported = true;
    invocation->m_confidence = confidence;
    result->status = RAW_GCODE_READER_STATUS_SUCCESS;
    result->error_message = nullptr;
}

ViewerReaderInvocation::ViewerReaderInvocation(
    GCodeReaderChunkSource source,
    const Print *print,
    const raw_gcode_reader_state &initial_state,
    GCodeReaderResultBuilder &builder)
    : m_source(std::move(source)), m_print(print), m_builder(builder),
      m_initial_state(initial_state), m_event_state(initial_state)
{
    if (!m_source)
        throw std::invalid_argument("G-code viewer parsing requires an input source.");
}

run_ctx_gcode_reader ViewerReaderInvocation::context()
{
    run_ctx_gcode_reader value = {};
    value.operation = RAW_GCODE_READER_OPERATION_READ;
    value.print = reinterpret_cast<const print_handle *>(m_print);
    value.initial_state = &m_initial_state.state();
    value.callback_context = this;
    value.get_next_chunk = &ViewerReaderInvocation::next_callback;
    value.emit_event = &ViewerReaderInvocation::event_callback;
    value.emit_final_state = &ViewerReaderInvocation::final_callback;
    return value;
}

GCodeProcessorResult ViewerReaderInvocation::take_result()
{
    if (!m_final_state)
        throw std::runtime_error(
            "The G-code reader returned successfully without a final state.");
    return m_builder.finish(*m_final_state);
}

const std::string &ViewerReaderInvocation::error() const
{
    return m_error;
}

void ViewerReaderInvocation::next_callback(
    void *opaque, raw_gcode_reader_chunk *chunk,
    raw_gcode_reader_result *result) noexcept
{
    ViewerReaderInvocation *invocation =
        static_cast<ViewerReaderInvocation *>(opaque);
    if (invocation == nullptr || chunk == nullptr) {
        if (invocation != nullptr)
            invocation->fail(result, RAW_GCODE_READER_STATUS_INVALID_ARGUMENT,
                             "G-code reader requested an invalid viewer input chunk.");
        return;
    }
    try {
        invocation->next(*chunk);
        initialize_success(result);
    } catch (const std::invalid_argument &error) {
        invocation->fail(result, RAW_GCODE_READER_STATUS_INVALID_ARGUMENT, error.what());
    } catch (const std::exception &error) {
        invocation->fail(result, RAW_GCODE_READER_STATUS_ERROR, error.what());
    } catch (...) {
        invocation->fail(result, RAW_GCODE_READER_STATUS_ERROR,
                         "Unknown exception while supplying G-code viewer input.");
    }
}

void ViewerReaderInvocation::event_callback(
    void *opaque, const raw_gcode_reader_event *event_value,
    raw_gcode_reader_result *result) noexcept
{
    ViewerReaderInvocation *invocation =
        static_cast<ViewerReaderInvocation *>(opaque);
    if (invocation == nullptr || event_value == nullptr) {
        if (invocation != nullptr)
            invocation->fail(result, RAW_GCODE_READER_STATUS_INVALID_ARGUMENT,
                             "G-code reader emitted a null viewer event.");
        return;
    }
    try {
        invocation->event(*event_value);
        initialize_success(result);
    } catch (const std::invalid_argument &error) {
        invocation->fail(result, RAW_GCODE_READER_STATUS_INVALID_ARGUMENT, error.what());
    } catch (const std::exception &error) {
        invocation->fail(result, RAW_GCODE_READER_STATUS_ERROR, error.what());
    } catch (...) {
        invocation->fail(result, RAW_GCODE_READER_STATUS_ERROR,
                         "Unknown exception while building G-code viewer data.");
    }
}

void ViewerReaderInvocation::final_callback(
    void *opaque, const raw_gcode_reader_state *state,
    raw_gcode_reader_result *result) noexcept
{
    ViewerReaderInvocation *invocation =
        static_cast<ViewerReaderInvocation *>(opaque);
    if (invocation == nullptr || state == nullptr) {
        if (invocation != nullptr)
            invocation->fail(result, RAW_GCODE_READER_STATUS_INVALID_ARGUMENT,
                             "G-code reader emitted a null final viewer state.");
        return;
    }
    try {
        invocation->final_state(*state);
        initialize_success(result);
    } catch (const std::invalid_argument &error) {
        invocation->fail(result, RAW_GCODE_READER_STATUS_INVALID_ARGUMENT, error.what());
    } catch (const std::exception &error) {
        invocation->fail(result, RAW_GCODE_READER_STATUS_ERROR, error.what());
    } catch (...) {
        invocation->fail(result, RAW_GCODE_READER_STATUS_ERROR,
                         "Unknown exception while receiving the final viewer state.");
    }
}

void ViewerReaderInvocation::initialize_success(
    raw_gcode_reader_result *result) noexcept
{
    if (result == nullptr)
        return;
    result->status = RAW_GCODE_READER_STATUS_SUCCESS;
    result->error_message = nullptr;
}

void ViewerReaderInvocation::next(raw_gcode_reader_chunk &chunk)
{
    if (m_final_state)
        throw std::logic_error(
            "The G-code reader requested viewer input after its final state.");
    chunk = {};
    if (m_end_reached)
        return;
    m_active_chunk = m_source();
    if (m_active_chunk.empty()) {
        m_end_reached = true;
        return;
    }
    m_builder.consume_chunk(m_active_chunk);
    chunk.bytes = m_active_chunk.data();
    chunk.byte_count = static_cast<uint64_t>(m_active_chunk.size());
}

void ViewerReaderInvocation::event(const raw_gcode_reader_event &event_value)
{
    if (m_final_state)
        throw std::logic_error(
            "The G-code reader emitted a viewer event after its final state.");
    m_event_state.apply(event_value);
    m_builder.consume_event(event_value, m_event_state);
}

void ViewerReaderInvocation::final_state(const raw_gcode_reader_state &state)
{
    if (!m_end_reached)
        throw std::logic_error(
            "The G-code reader published viewer state before end-of-stream.");
    if (m_final_state)
        throw std::logic_error(
            "The G-code reader published more than one final viewer state.");
    m_final_state.emplace(state);
}

void ViewerReaderInvocation::fail(
    raw_gcode_reader_result *result,
    raw_gcode_reader_status status,
    const std::string &message) noexcept
{
    m_error = message;
    if (result == nullptr)
        return;
    result->status = status;
    result->error_message = m_error.c_str();
}

std::string viewer_execution_error(raw_plugin_execution_status status,
                                   const std::string &id)
{
    switch (status) {
    case RAW_PLUGIN_EXECUTION_CANCELLED:
        return "G-code reader plugin '" + id + "' was cancelled.";
    case RAW_PLUGIN_EXECUTION_INVALID_ARGUMENT:
        return "G-code reader plugin '" + id + "' rejected the viewer request.";
    case RAW_PLUGIN_EXECUTION_INACTIVE:
        return "G-code reader plugin '" + id + "' is unavailable for viewer parsing.";
    case RAW_PLUGIN_EXECUTION_PLUGIN_ERROR:
        return "G-code reader plugin '" + id + "' failed while building viewer data.";
    default:
        return "G-code reader plugin '" + id + "' returned an unknown viewer status.";
    }
}

} // namespace

std::string detect_gcode_reader(Orchestrator &orchestrator, Print *print,
                                std::string_view prefix)
{
    const std::string_view bounded = prefix.substr(0, GCODE_READER_PROBE_PREFIX_SIZE);
    Plugin *winner = nullptr;
    uint32_t winner_confidence = 0;
    const std::vector<Plugin *> readers = orchestrator.get_active_plugins_for_step(GCODE_READER);
    for (Plugin *plugin : readers) {
        ProbeInvocation invocation(bounded, print);
        run_ctx_gcode_reader context = invocation.context();
        raw_plugin_run_payload payload = {};
        payload.data = &context;
        // Probes are independent candidates. Their diagnostics are retained,
        // but a provider failure must not cancel or suppress later probes.
        if (orchestrator.execute_plugin(*plugin, print, &payload, 1, false) !=
            RAW_PLUGIN_EXECUTION_SUCCESS)
            continue;
        uint32_t confidence = 0;
        try {
            confidence = invocation.confidence();
        } catch (const std::exception &error) {
            // A malformed probe is local to its provider. Keep its diagnostic
            // visible while allowing the remaining Readers to compete.
            BOOST_LOG_TRIVIAL(warning) << "G-code Reader probe '" << plugin->get_id()
                                       << "' failed: " << error.what();
            continue;
        }
        if (confidence == 0)
            continue;
        if (winner == nullptr || confidence > winner_confidence ||
            (confidence == winner_confidence &&
             (plugin->get_priority() < winner->get_priority() ||
              (plugin->get_priority() == winner->get_priority() && plugin->get_id() < winner->get_id())))) {
            winner = plugin;
            winner_confidence = confidence;
        }
    }
    if (winner != nullptr)
        return winner->get_id();

    Plugin *fallback = orchestrator.get_plugin(DEFAULT_GCODE_READER_PLUGIN_ID);
    if (fallback == nullptr || fallback->get_step() != GCODE_READER ||
        !orchestrator.is_plugin_active(fallback))
        throw std::runtime_error("No probe recognized the G-code and the default Reader is unavailable.");
    return DEFAULT_GCODE_READER_PLUGIN_ID;
}

GCodeReaderService::GCodeReaderService(Orchestrator &orchestrator, Print *print)
    : m_orchestrator(orchestrator), m_print(print),
      m_reader(reinterpret_cast<orchestrator_handle *>(&orchestrator),
               reinterpret_cast<const print_handle *>(print))
{}

slic3r_api::GCodeStateTracker GCodeReaderService::initial_state(
    std::string_view reader_plugin_id) const
{
    const std::string id = validated_reader_id(reader_plugin_id);
    uint32_t reader_state_id = GCODE_READER_STATE_ID_PORTABLE;
    const char *state_name = nullptr;
    if (id == "gcode.reader.reprap")
        state_name = slic3r_api::GCodeGeneration::Reader::REPRAP_READER_STATE_NAME;
    else if (id == "gcode.reader.bambu")
        state_name = slic3r_api::GCodeGeneration::Reader::BAMBU_READER_STATE_NAME;
    if (state_name != nullptr) {
        reader_state_id = m_orchestrator.register_runtime_id(
            GCODE_READER_STATE_ID_TYPE, state_name);
        if (reader_state_id == ORCHESTRATOR_RUNTIME_ID_INVALID)
            throw std::runtime_error(
                std::string("The G-code Reader state identity could not be registered: ") + state_name);
    }
    // TODO: a future Reader registration descriptor must expose a private
    // state schema name. This phase cannot initialize an external Reader with
    // private keys without changing the plugin API, so only the built-in RRF
    // and Bambu schemas are known here; every portable Reader uses the
    // permanent id zero.

    const PrintConfig *config = m_print != nullptr ? &m_print->config() : nullptr;
    const size_t tool_count = config != nullptr ?
        std::max<size_t>(config->nozzle_diameter.size(), 1) : 1;
    std::vector<raw_gcode_reader_event_item> items = {
        {RAW_GCODE_READER_ITEM_X, 0, 0.0},
        {RAW_GCODE_READER_ITEM_Y, 0, 0.0},
        {RAW_GCODE_READER_ITEM_Z, 0, 0.0},
        {RAW_GCODE_READER_ITEM_UNITS_MM, 0, 1.0},
        {RAW_GCODE_READER_ITEM_XYZ_ABSOLUTE, 0, 1.0},
        {RAW_GCODE_READER_ITEM_E_ABSOLUTE, 0, 1.0},
        {RAW_GCODE_READER_ITEM_VOLUMETRIC, 0,
         config != nullptr && config->use_volumetric_e.value ? 1.0 : 0.0},
        {RAW_GCODE_READER_ITEM_ACTIVE_TOOL, 0, 0.0},
        {RAW_GCODE_READER_ITEM_BED_TEMPERATURE, 0, 0.0},
        {RAW_GCODE_READER_ITEM_CHAMBER_TEMPERATURE, 0, 0.0},
        {RAW_GCODE_READER_ITEM_SPEED_FACTOR, 0, 100.0},
        {RAW_GCODE_READER_ITEM_EXTRUSION_FACTOR, 0, 100.0},
        {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 0.0},
        {RAW_GCODE_READER_ITEM_READER_ID, 0, double(reader_state_id)}
    };
    for (size_t tool = 0; tool < tool_count; ++tool) {
        const double filament_diameter = config != nullptr &&
            config->filament_diameter.size() != 0 ?
            config->filament_diameter.get_at(
                std::min(tool, config->filament_diameter.size() - 1)) : 1.75;
        Vec2d offset = Vec2d::Zero();
        if (config != nullptr && config->extruder_offset.size() != 0)
            offset = config->extruder_offset.get_at(
                std::min(tool, config->extruder_offset.size() - 1));
        items.push_back({RAW_GCODE_READER_ITEM_E, static_cast<uint32_t>(tool), 0.0});
        items.push_back({RAW_GCODE_READER_ITEM_TOOL_NOZZLE, static_cast<uint32_t>(tool), double(tool)});
        items.push_back({RAW_GCODE_READER_ITEM_TOOL_FAN, static_cast<uint32_t>(tool), double(tool)});
        items.push_back({RAW_GCODE_READER_ITEM_TOOL_E_AXIS, static_cast<uint32_t>(tool), double(tool)});
        items.push_back({RAW_GCODE_READER_ITEM_TOOL_FILAMENT_DIAMETER,
                         static_cast<uint32_t>(tool), filament_diameter});
        items.push_back({RAW_GCODE_READER_ITEM_TOOL_OFFSET_X,
                         static_cast<uint32_t>(tool), offset.x()});
        items.push_back({RAW_GCODE_READER_ITEM_TOOL_OFFSET_Y,
                         static_cast<uint32_t>(tool), offset.y()});
        items.push_back({RAW_GCODE_READER_ITEM_TOOL_OFFSET_Z,
                         static_cast<uint32_t>(tool), 0.0});
        items.push_back({RAW_GCODE_READER_ITEM_TOOL_TEMPERATURE,
                         static_cast<uint32_t>(tool), 0.0});
        items.push_back({RAW_GCODE_READER_ITEM_FAN_SPEED,
                         static_cast<uint32_t>(tool), 0.0});
    }
    const raw_gcode_reader_state raw = {
        items.data(), static_cast<uint32_t>(items.size())
    };
    return slic3r_api::GCodeStateTracker(raw);
}

slic3r_api::GCodeStateTracker GCodeReaderService::read(
    std::string_view reader_plugin_id,
    const raw_gcode_reader_state &initial_state,
    GCodeReaderChunkSource source) const
{
    return m_reader.read(reader_plugin_id, initial_state, std::move(source));
}

GCodeProcessorResult GCodeReaderService::read_for_viewer(
    std::string_view reader_plugin_id,
    GCodeReaderChunkSource source) const
{
    if (!source)
        throw std::invalid_argument("G-code viewer parsing requires an input source.");
    const std::lock_guard<std::mutex> lock(m_viewer_mutex);
    const std::string id = validated_reader_id(reader_plugin_id);
    const slic3r_api::GCodeStateTracker initial = initial_state(id);
    const Printing::PrintingPlan *plan =
        m_print == nullptr ? nullptr : m_print->printing_plan();
    if (m_print != nullptr && plan == nullptr)
        throw std::runtime_error(
            "G-code viewer parsing for a Print requires its completed PrintingPlan.");
    GCodeReaderResultBuilder builder(m_print, plan, id, initial);
    ViewerReaderInvocation invocation(
        std::move(source), m_print, initial.state(), builder);
    run_ctx_gcode_reader context = invocation.context();
    raw_plugin_run_payload payload = {&context};
    Plugin *plugin = m_orchestrator.get_plugin(id);
    if (plugin == nullptr || plugin->get_step() != GCODE_READER ||
        !m_orchestrator.is_plugin_active(plugin))
        throw std::runtime_error(
            "G-code reader plugin '" + id + "' is unavailable for viewer parsing.");
    const raw_plugin_execution_status status =
        m_orchestrator.execute_plugin(*plugin, m_print, &payload, 1, false);
    if (!invocation.error().empty())
        throw std::runtime_error(invocation.error());
    if (status != RAW_PLUGIN_EXECUTION_SUCCESS)
        throw std::runtime_error(viewer_execution_error(status, id));
    GCodeProcessorResult result = invocation.take_result();
    if (m_print != nullptr) {
        Print::StatusMonitor status(*m_print);
        for (const GCodeProcessorResult::ReaderDiagnostic &diagnostic :
             result.reader_diagnostics) {
            const std::string message = "G-code Reader '" + id + "', line " +
                std::to_string(uint64_t(diagnostic.line) + 1u) + ": " +
                diagnostic.message;
            status.active_step_add_warning(diagnostic.level, message);
        }
    }
    return result;
}

GCodeProcessorResult GCodeReaderService::read_for_viewer(
    std::string_view reader_plugin_id,
    std::string_view text) const
{
    bool supplied = false;
    GCodeReaderChunkSource source = [text, &supplied]() mutable -> std::string_view {
        if (supplied)
            return {};
        supplied = true;
        return text;
    };
    return read_for_viewer(reader_plugin_id, std::move(source));
}

slic3r_api::GCodeStateTracker GCodeReaderService::read(
    std::string_view reader_plugin_id,
    const raw_gcode_reader_state &initial_state,
    std::string_view text) const
{
    return m_reader.read(reader_plugin_id, initial_state, text);
}

std::string GCodeReaderService::detect(std::string_view prefix) const
{
    return detect_gcode_reader(m_orchestrator, m_print, prefix);
}

std::string GCodeReaderService::validated_reader_id(std::string_view reader_plugin_id) const
{
    return m_reader.validated_reader_id(reader_plugin_id);
}

} // namespace Slic3r
