///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_plugin_cpp_PluginBase_hpp_
#define slic3r_Api_plugin_cpp_PluginBase_hpp_

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <stdexcept>
#include <string>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_plugin.h"
#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"

/*
PluginBase and PluginProgress: writing a C++ plugin through the C interface
=======================================================================

The host is the slicer application. A plugin is an object that performs work at
a processing stage, called a step. PluginBase adapts a derived C++ class to the
C ABI, the binary calling interface used by the host. It builds the function
table and contains exceptions at its callbacks, so a plugin need not write that
table or the C-to-C++ wrappers itself.

PluginProgress is a thread-safe work counter selected by PluginBase for each callback. A plugin
estimates its work during preparation, then reports completed units during
execution. The host displays the progress it receives; it does not infer how
much work a plugin will perform.

A minimal plugin that counts one work unit per call can be written as follows:

    class MyPlugin final : public slic3r_api::PluginBase
    {
    public:
        explicit MyPlugin(orchestrator_handle *host) : PluginBase(host) {}

    private:
        const char *id_impl() const noexcept override { return "my_plugin"; }
        slicing_step_t step_impl() const noexcept override { return STEP_POST_SLICING; }
        const char *const *dependencies_impl() const noexcept override { return nullptr; }
        int32_t priority_impl() const noexcept override { return 0; }

        const char *progress_message_format_impl() const noexcept override
        {
            return "My plugin: %u / %u";
        }

        void setup_run_impl(const plugin_run_context *) const override
        {
            progress().add_max(1);
        }

        void run_impl(const plugin_run_context *context) const override
        {
            slic3r_api::throw_if_cancelled(context);
            // Perform the work for this call before counting it as complete.
            progress().increment();
        }
    };

Register a long-lived instance with the same host passed to its constructor:

    static MyPlugin plugin(host);
    orchestrator_register_plugin(host, plugin.c_instance());

The returned plugin_instance contains a borrowed pointer to the C++ object and
a function table with its size in bytes. Registration does not transfer ownership
of the object or its code: both must outlive every host callback. The context of
an individual callback, plugin_run_context, is borrowed only for that call and
must not be saved for a later run.

Some plugins are alternatives for one job, an exclusive group. Its selector is
the setting that chooses a member. After registering the plugin, initialize a
raw_exclusive_group_def with raw_exclusive_group_def_init(), fill its description,
then call register_exclusive_group(step, definition). The helper supplies this
plugin's id and throws if the host rejects the description. Execution priority
controls call order within a step; exclusive_group_priority_impl() separately
ranks alternatives for the default choice.

For one step/plugin pair, register the instance and initialize it before
preparing the execution. Prepare every job before running any of them;
all preparation workers finish before execution workers start.

setup_run_impl() prepares work and estimates its size. Only run_impl() should
change the slicer data tree, within the capabilities of its step payload. Keep
shared plugin state protected or atomic when calls run concurrently. Each
preparation can add its expected units with progress().add_max(); execution uses
increment() or increment_by(). progress_message_format_impl() supplies the
displayed format, and setup_safe() resets the counters before setup_impl().

Initialization returns PLUGIN_ERROR_OK on success or a message id obtained from
orchestrator_set_error_message() on failure; it does not let exceptions cross C.
The setup and run wrappers treat PluginCancelled as cooperative cancellation and
add no error message. Standard and unknown exceptions instead become plugin
errors through the current callback context. The host observes the stop/error
state after the callback returns.

The host may supply separate counters for each execution.
ProgressScope selects them for the current callback, then restores the previous selection.
Without host counters, PluginBase uses the plugin's own counters.
On the callback thread, obtain PluginProgress &counters = progress().
Pass that reference to any threads started by the callback. Finish those threads
before returning. Do not call progress() from an unassociated thread.

Operation notes:
- construct the plugin with its host
- c_instance() :  orchestrator_register_plugin()
- initialize_bridge() :  initialize_safe() :  inilialize_impl()
- [the interface spelling is inilialize_impl]
- setup_bridge() :  setup_safe()
- setup_impl(context, run_count) once
- setup_run_bridge() :  setup_run_safe() :  setup_run_impl(context) for each call
- [these preparations may run in parallel; all finish before execution]
- run_bridge() :  run_safe() :  run_impl(context) for each call
- [these executions may also run in parallel]

Representative direct calls (branches are not a timeline):

    setup_bridge()
        `-- setup_safe()
            |-- progress()
            `-- setup_impl()

    setup_run_bridge()
        `-- setup_run_safe()
            `-- setup_run_impl()

    run_bridge()
        `-- run_safe()
            `-- run_impl()
*/

namespace slic3r_api {

// Thread-safe progress helper for C++ plugins.
//
// The helper counts "work units". During setup_run_impl(), call
// add_max() with the amount of work each run expects to do. During run_impl(),
// call increment()/increment_by() whenever work is completed. A progress update
// is sent to the host only when the visible percentage changes.
class PluginProgress
{
public:
    static constexpr uint32_t DefaultPhase = 0;
    static constexpr uint32_t MaxPhases = 16;

    // Reset all counters. PluginBase calls this automatically in setup()
    // (before setup_impl()), so normal plugins do not need to call it.
    // Only the host progress callback is stored; plugin_run_context itself is
    // not kept because run contexts are copied per worker thread.
    void reset(const plugin_run_context *ctx, uint32_t run_count, const char *message_format)
    {
        m_host_context = ctx != nullptr ? ctx->host_context : nullptr;
        m_report_progress = ctx != nullptr ? ctx->report_progress : nullptr;
        m_run_count.store(run_count, std::memory_order_relaxed);
        for (PhaseProgress &phase : m_phases) {
            phase.done.store(0, std::memory_order_relaxed);
            phase.max.store(0, std::memory_order_relaxed);
            phase.finished_runs.store(0, std::memory_order_relaxed);
            phase.active.store(0, std::memory_order_relaxed);
            phase.message_format = message_format != nullptr ? message_format : "Plugin progress: %u / %u";
        }
        m_phases[DefaultPhase].active.store(1, std::memory_order_relaxed);
        m_last_percent.store(-1, std::memory_order_relaxed);
        m_last_phase.store(-1, std::memory_order_relaxed);
    }

    // Configure the message used when a phase is the visible phase. The format
    // receives two integers: completed work and total work for that phase.
    void set_phase_format(uint32_t phase, const char *message_format)
    {
        if (phase >= MaxPhases)
            return;
        m_phases[phase].message_format = message_format != nullptr ? message_format : "Plugin progress: %u / %u";
        m_phases[phase].active.store(1, std::memory_order_relaxed);
    }

    // Add expected work units. This is atomic so setup_run_impl() may be called
    // in parallel for multiple objects.
    void add_max(uint32_t count)
    {
        add_max(DefaultPhase, count);
    }

    void add_max(uint32_t phase, uint32_t count)
    {
        if (phase >= MaxPhases)
            return;
        m_phases[phase].active.store(1, std::memory_order_relaxed);
        m_phases[phase].max.fetch_add(count, std::memory_order_relaxed);
    }

    // Mark one work unit as completed and maybe report progress.
    void increment()
    {
        increment(DefaultPhase);
    }

    void increment(uint32_t phase)
    {
        increment_by(phase, 1);
    }

    // Move progress by `count` work units and maybe report progress. `count`
    // may be negative if a plugin intentionally wants to move the bar back.
    void increment_by(int32_t count)
    {
        increment_by(DefaultPhase, count);
    }

    void increment_by(uint32_t phase, int32_t count)
    {
        if (phase >= MaxPhases)
            return;
        m_phases[phase].active.store(1, std::memory_order_relaxed);
        const int32_t done = add_done(phase, count);
        report_if_needed(phase, phase_message(phase, done).c_str());
    }

    // Mark this run as done with a phase. A phase is considered globally
    // finished only once every planned run called finish_run() for it. The
    // final run completes any missing work and publishes 100% when there is no
    // following phase to display.
    void finish_run(uint32_t phase = DefaultPhase)
    {
        if (phase >= MaxPhases)
            return;
        m_phases[phase].active.store(1, std::memory_order_relaxed);
        const uint32_t finished = m_phases[phase].finished_runs.fetch_add(1, std::memory_order_relaxed) + 1;
        const int32_t done = m_phases[phase].done.load(std::memory_order_relaxed);
        const uint32_t max = m_phases[phase].max.load(std::memory_order_relaxed);
        const bool completed_by_this_call = finished == m_run_count.load(std::memory_order_relaxed);
        if (completed_by_this_call && done < int32_t(max))
            m_phases[phase].done.store(int32_t(max), std::memory_order_relaxed);
        report_if_needed(phase, phase_message(phase).c_str(), completed_by_this_call);
    }

    // Convenience wrapper around increment() with one-shot printf-style
    // formatting. Most plugins should prefer progress_message_format_impl()
    // and plain increment().
    template<class... Args>
    void incrementf(const char *format, Args... args)
    {
        increment_byf(DefaultPhase, 1, format, args...);
    }

    template<class... Args>
    void incrementf(uint32_t phase, const char *format, Args... args)
    {
        increment_byf(phase, 1, format, args...);
    }

    // Same as incrementf(), but moves multiple work units at once.
    template<class... Args>
    void increment_byf(int32_t count, const char *format, Args... args)
    {
        increment_byf(DefaultPhase, count, format, args...);
    }

    template<class... Args>
    void increment_byf(uint32_t phase, int32_t count, const char *format, Args... args)
    {
        if (phase >= MaxPhases)
            return;
        m_phases[phase].active.store(1, std::memory_order_relaxed);
        add_done(phase, count);
        report_if_needed(phase, format_message(format, args...).c_str());
    }

    int32_t done() const { return done(DefaultPhase); }
    int32_t done(uint32_t phase) const { return phase < MaxPhases ? m_phases[phase].done.load(std::memory_order_relaxed) : 0; }
    uint32_t max() const { return max(DefaultPhase); }
    uint32_t max(uint32_t phase) const { return phase < MaxPhases ? m_phases[phase].max.load(std::memory_order_relaxed) : 0; }
    uint32_t run_count() const { return m_run_count.load(std::memory_order_relaxed); }

private:
    struct PhaseProgress
    {
        std::atomic<int32_t> done { 0 };
        std::atomic<uint32_t> max { 0 };
        std::atomic<uint32_t> finished_runs { 0 };
        std::atomic<uint32_t> active { 0 };
        const char *message_format = "Plugin progress: %u / %u";
    };

    // Convert completed work units to a [0.0, 1.0] progress value and pass it
    // through the C ABI callback when the visible percentage changes.
    int32_t add_done(uint32_t phase, int32_t delta)
    {
        int32_t current = m_phases[phase].done.load(std::memory_order_relaxed);
        for (;;) {
            const int32_t next = current + delta;
            if (m_phases[phase].done.compare_exchange_weak(current,
                                                           next,
                                                           std::memory_order_relaxed,
                                                           std::memory_order_relaxed))
                return next;
        }
    }

    bool phase_finished(uint32_t phase) const
    {
        const PhaseProgress &progress = m_phases[phase];
        if (progress.active.load(std::memory_order_relaxed) == 0)
            return true;
        if (progress.finished_runs.load(std::memory_order_relaxed) < m_run_count.load(std::memory_order_relaxed))
            return false;
        return progress.done.load(std::memory_order_relaxed) >= int32_t(progress.max.load(std::memory_order_relaxed));
    }

    int32_t first_visible_phase() const
    {
        for (uint32_t phase = 0; phase < MaxPhases; ++phase) {
            if (!phase_finished(phase))
                return int32_t(phase);
        }
        return -1;
    }

    void report_if_needed(uint32_t updated_phase,
                          const char *message,
                          bool allow_completed_updated_phase = false)
    {
        if (m_report_progress == nullptr)
            return;

        int32_t visible_phase = first_visible_phase();
        if (visible_phase < 0) {
            // Once the final active phase is complete there is normally no
            // visible phase. finish_run() may still publish its terminal 100%,
            // but ordinary increments and duplicate finish calls may not.
            if (!allow_completed_updated_phase || updated_phase >= MaxPhases || !phase_finished(updated_phase))
                return;
            visible_phase = int32_t(updated_phase);
        }

        const PhaseProgress &progress = m_phases[uint32_t(visible_phase)];
        const uint32_t max = progress.max.load(std::memory_order_relaxed);
        if (max == 0)
            return;

        const int32_t done = progress.done.load(std::memory_order_relaxed);
        const uint32_t clamped_done = uint32_t(std::clamp<int64_t>(done, 0, int64_t(max)));
        const int percent = int((uint64_t(clamped_done) * 100u) / max);
        const bool phase_changed = m_last_phase.exchange(visible_phase, std::memory_order_relaxed) != visible_phase;
        if (phase_changed)
            m_last_percent.store(-1, std::memory_order_relaxed);

        const int previous_percent = m_last_percent.exchange(percent, std::memory_order_relaxed);
        if (phase_changed || percent != previous_percent) {
            const std::string visible_message =
                updated_phase == uint32_t(visible_phase) ? std::string(message != nullptr ? message : "") :
                phase_message(uint32_t(visible_phase), done);
            m_report_progress(m_host_context, double(percent) / 100.0, visible_message.c_str());
        }
    }

    std::string phase_message(uint32_t phase) const
    {
        return phase_message(phase, m_phases[phase].done.load(std::memory_order_relaxed));
    }

    std::string phase_message(uint32_t phase, int32_t done) const
    {
        const PhaseProgress &progress = m_phases[phase];
        const uint32_t max = progress.max.load(std::memory_order_relaxed);
        const uint32_t clamped_done = uint32_t(std::clamp<int64_t>(done, 0, int64_t(max)));
        return format_message(progress.message_format, clamped_done, max);
    }

    template<class... Args>
    static std::string format_message(const char *format, Args... args)
    {
        if (format == nullptr)
            return {};

        const int needed = std::snprintf(nullptr, 0, format, args...);
        if (needed <= 0)
            return {};

        std::vector<char> buffer(size_t(needed) + 1);
        std::snprintf(buffer.data(), buffer.size(), format, args...);
        return std::string(buffer.data(), size_t(needed));
    }

    std::array<PhaseProgress, MaxPhases> m_phases;
    std::atomic<uint32_t> m_run_count { 0 };
    std::atomic<int> m_last_percent { -1 };
    std::atomic<int32_t> m_last_phase { -1 };
    plugin_host_context *m_host_context = nullptr;
    plugin_report_progress_fn m_report_progress = nullptr;
};

// Base class for C++ plugins exposed through the C ABI.
//
// A plugin author normally only interacts with the protected section:
// implement the required *_impl() methods, optionally implement setup_impl()
// and setup_run_impl(), and use progress() from run_impl().
class PluginBase
{
public:
    virtual ~PluginBase() = default;

    // Start definition with raw_exclusive_group_def_init(); this member supplies its own id. Return true when added or false when already present.
    // Throw std::invalid_argument when the application rejects the description with a negative C result.
    bool register_exclusive_group(slicing_step_t step, raw_exclusive_group_def definition) const
    {
        definition.declaring_plugin_id = id_impl();
        const int32_t result = orchestrator_register_exclusive_group(m_orchestrator, step, &definition);
        if (result < 0)
            throw std::invalid_argument("Cannot register exclusive group definition");
        return result == 1;
    }

    // Return the C ABI instance registered with orchestrator_register_plugin().
    // Built-in plugins usually expose this through a register_*_plugin()
    // function; external plugin DLLs usually call it from register_plugin().
    plugin_instance c_instance() const
    {
        plugin_instance instance = {};
        instance.ctx = const_cast<PluginBase *>(this);
        // Cleanup participation varies by instance; never modify the shared vtable for one plugin.
        m_instance_vtable = vtable();
        m_instance_vtable.clear = clear_enabled_impl() ? &PluginBase::clear_bridge : nullptr;
        instance.vt = &m_instance_vtable;
        return instance;
    }

protected:
    // Constructor: please give the orchestrator that register you so you can reuse it afterwards.
    PluginBase(orchestrator_handle *orch) : m_orchestrator(orch) {
        assert(m_orchestrator);
    }

    // Borrow this callback's counters. Obtain the reference here before starting other threads.
    // Those threads must finish using it before the callback returns.
    PluginProgress &progress() const
    {
        assert(s_active_progress != nullptr);
        return s_active_progress != nullptr ? *s_active_progress : m_progress;
    }

    // Stable plugin identifier. It must be unique among registered plugins.
    virtual const char *id_impl() const noexcept = 0;

    // Short user-facing plugin name. It is used in plugin selectors while
    // id_impl() remains the stable serialized value. Override when the id is
    // too technical for end users.
    virtual const char *name_impl() const noexcept { return id_impl(); }

    // Longer user-facing description. Dialogs may show this as helper text.
    virtual const char *description_impl() const noexcept { return ""; }

    // Exclusive group id. Plugins sharing the same group are mutually
    // exclusive: the host creates one selector and runs only the selected
    // active plugin. A plugin that has no alternatives should use its own id as
    // a singleton group; this makes it possible for a future alternative to
    // target the old plugin without modifying the old implementation.
    virtual const char *exclusive_group_impl() const noexcept { return id_impl(); }

    // Pipeline step where the plugin runs, for example STEP_POST_SLICING.
    virtual slicing_step_t step_impl() const noexcept = 0;

    // Null-terminated dependency list. Return `{ nullptr }` when there are no
    // dependencies. PluginBase converts it to const_strings_t for the C ABI.
    virtual const char *const *dependencies_impl() const noexcept = 0;

    // Lower priority runs first inside a step.
    virtual int32_t priority_impl() const noexcept = 0;
    // Return this member's preference rank among alternatives.
    virtual int32_t exclusive_group_priority_impl() const noexcept { return 100; }

    // Configuration options read or defined by this plugin. The default is
    // empty; every key returned by defined_config_keys() must also be present.
    // Return the required entry count when keys is nullptr, otherwise fill the
    // caller-provided array with borrowed key pointers and type expectations.
    virtual int32_t used_config_keys(raw_used_config_key *keys) const noexcept
    {
        (void)keys;
        return 0;
    }

    // Configuration option keys created by this plugin. The host uses this
    // declaration for early activation checks; orchestrator_create_option_def()
    // still performs the full definition compatibility validation. The host
    // obtains each defined key's type from used_config_keys().
    virtual int32_t defined_config_keys(const char **keys) const noexcept
    {
        (void)keys;
        return 0;
    }

    // Optional: message format used by PluginProgress when increment() reports
    // progress. It receives two unsigned integers: completed work and total
    // expected work. Override for clearer plugin-specific messages.
    virtual const char *progress_message_format_impl() const noexcept
    {
        return "Plugin progress: %u / %u";
    }
    
    // Optional: called once at startup.
    // You can ask the orchestrator to add settings definitions here.
    virtual void inilialize_impl(storage_handle *storage) const {};

    // Optional: called once before setup_run_impl()/run_impl().
    // PluginBase resets progress before calling this method.
    virtual void setup_impl(const plugin_run_context *, uint32_t) const {}

    // Optional: called once for each future run context, before any run_impl()
    // starts. Use it to call progress().add_max(...).
    virtual void setup_run_impl(const plugin_run_context *) const {}

    // Optional: release state associated with the Print that was prepared by
    // setup(). Return PLUGIN_ERROR_OK on success, or a diagnostic id obtained
    // from orchestrator_set_error_message() to report why the state survived.
    // The default keeps the ABI callback absent for services that do not
    // participate in pipeline cleanup.
    virtual bool clear_enabled_impl() const noexcept { return false; }
    virtual int32_t clear_impl(const print_handle *, storage_handle *) const
    {
        return PLUGIN_ERROR_OK;
    }

    // Required: perform the plugin work for one run context. This method may be
    // called in parallel for different objects, so shared plugin state must be
    // protected or atomic.
    virtual void run_impl(const plugin_run_context *run_ctx) const = 0;

private:
    // Select one callback's counters on this thread. Restore the previous selection when the scope ends.
    class ProgressScope
    {
    public:
        // Use the host's execution counters when supplied. Otherwise use this plugin's own counters.
        ProgressScope(const PluginBase &plugin, const plugin_run_context *run_ctx)
            : previous_(s_active_progress)
        {
            s_active_progress = run_ctx != nullptr && run_ctx->progress_context != nullptr ?
                static_cast<PluginProgress *>(run_ctx->progress_context) : &plugin.m_progress;
        }
        // Restore progress selection when nested callbacks return, including exceptional exits.
        ~ProgressScope() { s_active_progress = previous_; }
    private:
        PluginProgress *previous_;
    };

    // The *_safe() wrappers are the C++/C ABI safety belt: exceptions are
    // converted to plugin errors instead of crossing function pointers.

    // Initialize with application-owned storage and return success or a stored error id. Exceptions cannot cross C.
    int32_t initialize_safe(storage_handle *storage) const noexcept {
        try {
            inilialize_impl(storage);
            return PLUGIN_ERROR_OK;
        } catch (const std::exception &error) {
            return orchestrator_set_error_message(m_orchestrator, error.what());
        } catch (...) {
            return orchestrator_set_error_message(m_orchestrator, "Unknown C++ exception in plugin initialization");
        }
    }


    void setup_safe(const plugin_run_context *run_ctx, uint32_t run_count) const noexcept
    {
        ProgressScope progress_scope(*this, run_ctx);
        try {
            progress().reset(run_ctx, run_count, progress_message_format_impl());
            setup_impl(run_ctx, run_count);
        } catch (const PluginCancelled &) {
            return;
        } catch (const std::exception &e) {
            report_error(run_ctx, e.what());
            return;
        } catch (...) {
            report_error(run_ctx, "Unknown C++ exception in plugin setup");
            return;
        }
    }

    void setup_run_safe(const plugin_run_context *run_ctx) const noexcept
    {
        ProgressScope progress_scope(*this, run_ctx);
        try {
            setup_run_impl(run_ctx);
        } catch (const PluginCancelled &) {
            return;
        } catch (const std::exception &e) {
            report_error(run_ctx, e.what());
            return;
        } catch (...) {
            report_error(run_ctx, "Unknown C++ exception in plugin setup_run");
            return;
        }
    }

    void run_safe(const plugin_run_context *run_ctx) const noexcept
    {
        ProgressScope progress_scope(*this, run_ctx);
        try {
            run_impl(run_ctx);
        } catch (const PluginCancelled &) {
            return;
        } catch (const std::exception &e) {
            report_error(run_ctx, e.what());
            return;
        } catch (...) {
            report_error(run_ctx, "Unknown C++ exception in plugin");
            return;
        }
    }

    int32_t clear_safe(const print_handle *print, storage_handle *storage) const noexcept
    {
        // Copy the diagnostic into the host registry before the exception dies;
        // no exception may cross the plugin ABI.
        try {
            return clear_impl(print, storage);
        } catch (const std::exception &error) {
            return store_error_message(error.what());
        } catch (...) {
            return store_error_message("Unknown C++ exception in plugin clear");
        }
    }

    // Register the failure text and return its id. The host returns the generic
    // failure code when it cannot store the message.
    int32_t store_error_message(const char *message) const noexcept
    {
        return orchestrator_set_error_message(m_orchestrator, message);
    }

    static const char *get_id_bridge(void *plugin_ctx)
    {
        return static_cast<PluginBase *>(plugin_ctx)->id_impl();
    }

    static const char *get_name_bridge(void *plugin_ctx)
    {
        return static_cast<PluginBase *>(plugin_ctx)->name_impl();
    }

    static const char *get_description_bridge(void *plugin_ctx)
    {
        return static_cast<PluginBase *>(plugin_ctx)->description_impl();
    }

    static const char *get_exclusive_group_bridge(void *plugin_ctx)
    {
        return static_cast<PluginBase *>(plugin_ctx)->exclusive_group_impl();
    }

    static slicing_step_t get_step_bridge(void *plugin_ctx)
    {
        return static_cast<PluginBase *>(plugin_ctx)->step_impl();
    }

    static const_strings_t get_dependencies_bridge(void *plugin_ctx)
    {
        const char *const *deps = static_cast<PluginBase *>(plugin_ctx)->dependencies_impl();
        uint32_t count = 0;
        while (deps != nullptr && deps[count] != nullptr)
            ++count;

        const_strings_t out = {};
        out.items = deps;
        out.size = count;
        return out;
    }

    static int32_t get_priority_bridge(void *plugin_ctx)
    {
        return static_cast<PluginBase *>(plugin_ctx)->priority_impl();
    }

    static int32_t used_config_keys_bridge(void *plugin_ctx, raw_used_config_key *keys)
    {
        return static_cast<PluginBase *>(plugin_ctx)->used_config_keys(keys);
    }

    static int32_t defined_config_keys_bridge(void *plugin_ctx, const char **keys)
    {
        return static_cast<PluginBase *>(plugin_ctx)->defined_config_keys(keys);
    }

    // Borrow the plugin and storage to initialize it, returning a diagnostic id without allowing exceptions through C.
    static int32_t initialize_bridge(void *plugin_ctx, storage_handle *storage)
    {
        return static_cast<PluginBase *>(plugin_ctx)->initialize_safe(storage);
    }

    static void setup_bridge(void *plugin_ctx, const plugin_run_context *run_ctx, uint32_t run_count)
    {
        static_cast<PluginBase *>(plugin_ctx)->setup_safe(run_ctx, run_count);
    }

    static void setup_run_bridge(void *plugin_ctx, const plugin_run_context *run_ctx)
    {
        static_cast<PluginBase *>(plugin_ctx)->setup_run_safe(run_ctx);
    }

    static void run_bridge(void *plugin_ctx, const plugin_run_context *run_ctx)
    {
        static_cast<PluginBase *>(plugin_ctx)->run_safe(run_ctx);
    }

    static int32_t clear_bridge(void *plugin_ctx,
                                const print_handle *print,
                                storage_handle *storage) noexcept
    {
        return static_cast<PluginBase *>(plugin_ctx)->clear_safe(print, storage);
    }

    // Read the borrowed plugin's preference rank without changing its state.
    static int32_t get_exclusive_group_priority_bridge(void *plugin_ctx)
    {
        return static_cast<PluginBase *>(plugin_ctx)->exclusive_group_priority_impl();
    }

    // Borrow the complete function table, including cleanup and alternative-preference functions.
    static const plugin_vtable &vtable()
    {
        static const plugin_vtable vt = {
            sizeof(plugin_vtable),
            &PluginBase::get_id_bridge,
            &PluginBase::get_name_bridge,
            &PluginBase::get_description_bridge,
            &PluginBase::get_exclusive_group_bridge,
            &PluginBase::get_step_bridge,
            &PluginBase::get_dependencies_bridge,
            &PluginBase::get_priority_bridge,
            &PluginBase::used_config_keys_bridge,
            &PluginBase::defined_config_keys_bridge,
            &PluginBase::initialize_bridge,
            &PluginBase::setup_bridge,
            &PluginBase::setup_run_bridge,
            &PluginBase::run_bridge,
            &PluginBase::clear_bridge,
            &PluginBase::get_exclusive_group_priority_bridge
        };
        return vt;
    }

    // Per-instance copy of the shared vtable, so enabling clear for one derived
    // plugin does not modify the table every other instance points to.
    mutable plugin_vtable m_instance_vtable = {};
    mutable PluginProgress m_progress;
    inline static thread_local PluginProgress *s_active_progress = nullptr;

protected:
    orchestrator_handle *m_orchestrator;
};

} // namespace slic3r_api


#endif // slic3r_Api_plugin_cpp_PluginBase_hpp_
