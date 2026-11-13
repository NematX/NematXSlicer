///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_plugin_cpp_gcode_GCodeReaderService_hpp_
#define slic3r_Api_plugin_cpp_gcode_GCodeReaderService_hpp_

#include <cstdint>
#include <functional>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include "libslic3r/Api/plugin/c/steps/slic3r_step_gcode_reader.h"
#include "libslic3r/Api/plugin/cpp/OrchestratorViews.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeStateTracker.hpp"

/*
Plugin-side G-code Reader client
================================

GCodeReaderService lets a C++ plugin synchronously execute another active
plugin implementing GCODE_READER. It uses only the generic orchestrator API:
there is no special Reader callback supplied by the host.

    firmware session
        `-- GCodeReaderService::read(reader id, initial state, source[, observer])
              |-- find the active Reader through OrchestratorView
              |-- execute it with a temporary run_ctx_gcode_reader
              |-- lend chunks and validate every emitted event
              |-- synchronously notify the optional observer after validation
              `-- copy and return the final state

The orchestrator and optional Print are borrowed and must outlive this helper.
Every callback context and raw state is local to read(); the returned tracker
owns its data. Calls through one helper are serialized because the generic
plugin execution contract does not permit concurrent execution of one provider
instance. Detection by file contents remains a host responsibility: this
helper always executes one exact identifier. Each read owns its observer callable
until execution ends; resources captured by reference remain the caller's
responsibility. Events and their item arrays are borrowed only during the
observer call and must be copied if retained. The service does not copy events.
The observer runs under this helper's non-recursive mutex and must not reenter
read() on the same helper. Its exceptions become Reader errors through the
existing C callback boundary and may cancel the current plugin execution.
Observation may already be partial on failure; discard accumulated data rather
than treating it as a completed read. There is no rollback or automatic resume.
*/

namespace slic3r_api {

inline constexpr const char *DEFAULT_GCODE_READER_PLUGIN_ID = "gcode.reader.default";

// Return the next non-empty borrowed chunk, or an empty view for end-of-stream.
// The bytes must remain valid until the source is called again.
using GCodeReaderChunkSource = std::function<std::string_view()>;

// Observe a validated event synchronously; copy borrowed items before returning
// if they are needed later. The read owns the callable, not referenced captures.
using GCodeReaderEventObserver = std::function<void(const raw_gcode_reader_event &)>;

class GCodeReaderService
{
public:
    GCodeReaderService(orchestrator_handle *orchestrator,
                       const print_handle *print = nullptr)
        : m_orchestrator(require_orchestrator(orchestrator)), m_print(print)
    {}

    GCodeReaderService(const GCodeReaderService &) = delete;
    GCodeReaderService &operator=(const GCodeReaderService &) = delete;

    // Return the exact usable Reader id. Empty explicitly selects the default.
    // Lookup exposes only the host's established activation set.
    std::string validated_reader_id(std::string_view reader_plugin_id) const
    {
        const std::string id = reader_plugin_id.empty() ?
            DEFAULT_GCODE_READER_PLUGIN_ID : std::string(reader_plugin_id);
        const PluginView plugin = m_orchestrator.find_plugin(id);
        if (!plugin.valid())
            throw std::runtime_error("G-code reader plugin '" + id +
                                     "' is unavailable in the host activation set.");
        if (plugin.step() != GCODE_READER)
            throw std::runtime_error("Plugin '" + id + "' is not a G-code reader.");
        return id;
    }

    // Consume a streaming source and return an owned copy of the final state.
    // A single helper serializes reads so its selected provider is never
    // entered concurrently through this client. The optional observer receives
    // borrowed events after validation, under the same non-recursive mutex;
    // it must not reenter this helper. Exceptions become Reader errors through
    // the C callback boundary and may cancel the current plugin execution.
    // On failure no successful final state is returned; discard partial observations.
    GCodeStateTracker read(std::string_view reader_plugin_id,
                           const raw_gcode_reader_state &initial_state,
                           GCodeReaderChunkSource source,
                           GCodeReaderEventObserver observer = {}) const
    {
        const std::lock_guard<std::mutex> lock(m_mutex);
        const std::string id = validated_reader_id(reader_plugin_id);
        const PluginView plugin = m_orchestrator.find_plugin(id);
        if (!plugin.valid())
            throw std::runtime_error("G-code reader plugin '" + id +
                                     "' left the host activation set before execution.");

        Invocation invocation(std::move(source), m_print, initial_state, std::move(observer));
        run_ctx_gcode_reader context = invocation.context();
        const raw_plugin_execution_status status =
            m_orchestrator.execute(plugin, m_print, context);
        // A provider must stop when a host callback rejects its data. Retain
        // that failure even if a malformed provider ignores the callback
        // result and reports overall success afterward.
        if (!invocation.error().empty())
            throw std::runtime_error(invocation.error());
        if (status != RAW_PLUGIN_EXECUTION_SUCCESS)
            throw std::runtime_error(execution_error(status, id));
        return invocation.take_final_state();
    }

    // Present a small in-memory fragment through the same pull protocol. Large
    // inputs should use the chunk-source overload to avoid retaining them.
    // The forwarded observer has the same borrowed lifetime, non-reentrance and
    // error rules as the streaming overload; copy retained items. Observer
    // exceptions become Reader errors and may cancel plugin execution;
    // discard partial observations on failure.
    GCodeStateTracker read(std::string_view reader_plugin_id,
                           const raw_gcode_reader_state &initial_state,
                           std::string_view text,
                           GCodeReaderEventObserver observer = {}) const
    {
        bool supplied = false;
        GCodeReaderChunkSource source = [text, &supplied]() mutable -> std::string_view {
            if (supplied)
                return {};
            supplied = true;
            return text;
        };
        return read(reader_plugin_id, initial_state, std::move(source), std::move(observer));
    }

private:
    static orchestrator_handle *require_orchestrator(orchestrator_handle *orchestrator)
    {
        if (orchestrator == nullptr)
            throw std::invalid_argument(
                "G-code Reader service requires an orchestrator handle.");
        return orchestrator;
    }

    // Own every adapter used during one execution. The initial and event
    // trackers validate input and emitted deltas independently; only the copied
    // final tracker is returned to the caller. The observer callable is owned
    // here until read ends and receives borrowed events after validation.
    class Invocation
    {
    public:
        // Own the source and observer and copy the initial state for validation.
        // Referenced captures remain borrowed; an empty source is rejected.
        Invocation(GCodeReaderChunkSource source,
                   const print_handle *print,
                   const raw_gcode_reader_state &initial_state,
                   GCodeReaderEventObserver observer)
            : m_source(std::move(source)), m_observer(std::move(observer)), m_print(print),
              m_initial_state(initial_state), m_event_state(initial_state)
        {
            if (!m_source)
                throw std::invalid_argument("G-code reader requires an input chunk source.");
        }

        run_ctx_gcode_reader context()
        {
            run_ctx_gcode_reader value = {};
            value.operation = RAW_GCODE_READER_OPERATION_READ;
            value.print = m_print;
            value.initial_state = &m_initial_state.state();
            value.callback_context = this;
            value.get_next_chunk = &Invocation::get_next_chunk_callback;
            value.emit_event = &Invocation::emit_event_callback;
            value.emit_final_state = &Invocation::emit_final_state_callback;
            return value;
        }

        GCodeStateTracker take_final_state()
        {
            if (!m_final_state)
                throw std::runtime_error(
                    "The G-code reader returned successfully without a final state.");
            return std::move(*m_final_state);
        }

        const std::string &error() const { return m_error; }

    private:
        static void get_next_chunk_callback(void *opaque,
                                            raw_gcode_reader_chunk *chunk,
                                            raw_gcode_reader_result *result) noexcept
        {
            Invocation *invocation = static_cast<Invocation *>(opaque);
            if (invocation == nullptr || chunk == nullptr) {
                if (invocation != nullptr)
                    invocation->fail(result, RAW_GCODE_READER_STATUS_INVALID_ARGUMENT,
                        "G-code reader requested a chunk with an invalid output structure.");
                return;
            }
            try {
                invocation->get_next_chunk(*chunk);
                initialize_success(result);
            } catch (const std::invalid_argument &error) {
                invocation->fail(result, RAW_GCODE_READER_STATUS_INVALID_ARGUMENT, error.what());
            } catch (const std::exception &error) {
                invocation->fail(result, RAW_GCODE_READER_STATUS_ERROR, error.what());
            } catch (...) {
                invocation->fail(result, RAW_GCODE_READER_STATUS_ERROR,
                                 "Unknown exception while supplying G-code input.");
            }
        }

        static void emit_event_callback(void *opaque,
                                        const raw_gcode_reader_event *event,
                                        raw_gcode_reader_result *result) noexcept
        {
            Invocation *invocation = static_cast<Invocation *>(opaque);
            if (invocation == nullptr || event == nullptr) {
                if (invocation != nullptr)
                    invocation->fail(result, RAW_GCODE_READER_STATUS_INVALID_ARGUMENT,
                                     "G-code reader emitted a null event.");
                return;
            }
            try {
                invocation->emit_event(*event);
                initialize_success(result);
            } catch (const std::invalid_argument &error) {
                invocation->fail(result, RAW_GCODE_READER_STATUS_INVALID_ARGUMENT, error.what());
            } catch (const std::exception &error) {
                invocation->fail(result, RAW_GCODE_READER_STATUS_ERROR, error.what());
            } catch (...) {
                invocation->fail(result, RAW_GCODE_READER_STATUS_ERROR,
                                 "Unknown exception while receiving a G-code reader event.");
            }
        }

        static void emit_final_state_callback(void *opaque,
                                              const raw_gcode_reader_state *state,
                                              raw_gcode_reader_result *result) noexcept
        {
            Invocation *invocation = static_cast<Invocation *>(opaque);
            if (invocation == nullptr || state == nullptr) {
                if (invocation != nullptr)
                    invocation->fail(result, RAW_GCODE_READER_STATUS_INVALID_ARGUMENT,
                                     "G-code reader emitted a null final state.");
                return;
            }
            try {
                invocation->emit_final_state(*state);
                initialize_success(result);
            } catch (const std::invalid_argument &error) {
                invocation->fail(result, RAW_GCODE_READER_STATUS_INVALID_ARGUMENT, error.what());
            } catch (const std::exception &error) {
                invocation->fail(result, RAW_GCODE_READER_STATUS_ERROR, error.what());
            } catch (...) {
                invocation->fail(result, RAW_GCODE_READER_STATUS_ERROR,
                                 "Unknown exception while receiving the G-code reader final state.");
            }
        }

        void get_next_chunk(raw_gcode_reader_chunk &chunk)
        {
            if (m_final_state)
                throw std::logic_error(
                    "The G-code reader requested input after publishing its final state.");
            chunk = {};
            if (m_end_reached)
                return;
            m_active_chunk = m_source();
            if (m_active_chunk.empty()) {
                m_end_reached = true;
                return;
            }
            chunk.bytes = m_active_chunk.data();
            chunk.byte_count = static_cast<uint64_t>(m_active_chunk.size());
        }

        // Validate and apply the borrowed delta before notifying the observer.
        // Exceptions reach the noexcept C callback's error channel; no rollback
        // undoes earlier observations and no event storage is retained here.
        void emit_event(const raw_gcode_reader_event &event)
        {
            if (m_final_state)
                throw std::logic_error(
                    "The G-code reader emitted an event after its final state.");
            m_event_state.apply(event);
            if (m_observer)
                m_observer(event);
        }

        void emit_final_state(const raw_gcode_reader_state &state)
        {
            if (!m_end_reached)
                throw std::logic_error(
                    "The G-code reader published its final state before reaching end-of-stream.");
            if (m_final_state)
                throw std::logic_error(
                    "The G-code reader published more than one final state.");
            m_final_state.emplace(state);
        }

        static void initialize_success(raw_gcode_reader_result *result) noexcept
        {
            if (result == nullptr)
                return;
            result->status = RAW_GCODE_READER_STATUS_SUCCESS;
            result->error_message = nullptr;
        }

        void fail(raw_gcode_reader_result *result,
                  raw_gcode_reader_status status,
                  const std::string &message) noexcept
        {
            m_error = message;
            if (result == nullptr)
                return;
            result->status = status;
            result->error_message = m_error.c_str();
        }

        GCodeReaderChunkSource m_source;
        GCodeReaderEventObserver m_observer;
        const print_handle *m_print;
        GCodeStateTracker m_initial_state;
        GCodeStateTracker m_event_state;
        std::optional<GCodeStateTracker> m_final_state;
        std::string_view m_active_chunk;
        std::string m_error;
        bool m_end_reached = false;
    };

    static std::string execution_error(raw_plugin_execution_status status,
                                       const std::string &id)
    {
        switch (status) {
        case RAW_PLUGIN_EXECUTION_CANCELLED:
            return "G-code reader plugin '" + id + "' was cancelled.";
        case RAW_PLUGIN_EXECUTION_INVALID_ARGUMENT:
            return "G-code reader plugin '" + id + "' received an invalid execution request.";
        case RAW_PLUGIN_EXECUTION_INACTIVE:
            return "G-code reader plugin '" + id + "' is no longer active.";
        case RAW_PLUGIN_EXECUTION_PLUGIN_ERROR:
            return "G-code reader plugin '" + id + "' failed.";
        default:
            return "G-code reader plugin '" + id + "' returned an unknown execution status.";
        }
    }

    OrchestratorView m_orchestrator;
    const print_handle *m_print;
    mutable std::mutex m_mutex;
};

} // namespace slic3r_api

#endif // slic3r_Api_plugin_cpp_gcode_GCodeReaderService_hpp_
