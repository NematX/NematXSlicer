///|/ Copyright (c) SuperSlicer 2026 Durand R?mi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef steps_steprunner_hpp_
#define steps_steprunner_hpp_

#include <atomic>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <numeric>
#include <string>
#include <vector>

#include <boost/log/trivial.hpp>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/DataTreeFwd.hpp"
#include "libslic3r/Exception.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Thread.hpp"

/*
Run plugin work for the objects selected by a pipeline step. One preparation
call sets up the plugin execution; per-object setup_run calls prepare the jobs,
then run calls perform them. All preparation workers finish before execution
workers begin, and all workers finish before their borrowed callback data dies.

The helpers test cancellation and plugin failure between these phases. A host
callback failure is contained before returning through C and is attributed to
the owning Print rather than allowing an exception across the plugin interface.

These helpers run pipeline plugins and handle failures from host callbacks.
setup() prepares the plugin. setup_run() prepares each job. All preparation
calls finish before run() starts those jobs. Each phase checks the owning
Print's stop state before continuing.

Operation notes:
- run_object_step_plugin
- Plugin::setup: prepare one execution
- Plugin::setup_run: prepare all object jobs, then wait for them
- Plugin::run: execute those jobs, then wait for them
- throw_if_plugin_stopped: check cancellation after each phase
- contain_callback_failure: handle an active exception without letting it cross C

Representative direct calls (branches are not a timeline):

    run_object_step_plugin()
        |-- prepare_guarded_plugin_host_context()
        |-- orchestrator.prepare_plugin_run_context()
        |-- plugin.setup()
        |-- throw_if_plugin_stopped()
        `-- parallel_for()

    parallel_for worker callback [not a direct call of the enclosing function]
        `-- plugin.setup_run()

    parallel_for worker callback [not a direct call of the enclosing function]
        `-- plugin.run()
*/

namespace Slic3r {
class ConfigBase;
}

namespace Slic3r::Steps {
std::vector<Plugin *> selected_or_active_plugins_for_step(Orchestrator &orchestrator,
                                                          slicing_step_t step,
                                                          const ConfigBase *config);
}

namespace Slic3r::Steps::Detail {

// Name a built-in pipeline or service step for diagnostics.
inline const char *builtin_step_name(slicing_step_t step)
{
    switch (step) {
    case STEP_NONE:                  return "STEP_NONE";
    case STEP_LAYER_HEIGHT:          return "STEP_LAYER_HEIGHT";
    case STEP_SLICING:               return "STEP_SLICING";
    case STEP_POST_SLICING:          return "STEP_POST_SLICING";
    case STEP_PRE_PERIMETER:         return "STEP_PRE_PERIMETER";
    case STEP_PERIMETER:             return "STEP_PERIMETER";
    case STEP_POST_PERIMETER:        return "STEP_POST_PERIMETER";
    case STEP_SURFACE_GENERATION:    return "STEP_SURFACE_GENERATION";
    case STEP_SKIRT_BRIM:            return "STEP_SKIRT_BRIM";
    case STEP_PRE_INFILL:            return "STEP_PRE_INFILL";
    case STEP_INFILL_GROUP:          return "STEP_INFILL_GROUP";
    case STEP_INFILL:                return "STEP_INFILL";
    case STEP_POST_INFILL:           return "STEP_POST_INFILL";
    case STEP_SUPPORT_DEMAND:        return "STEP_SUPPORT_DEMAND";
    case STEP_SUPPORT:               return "STEP_SUPPORT";
    case STEP_SUPPORT_TOOLPATH:      return "STEP_SUPPORT_TOOLPATH";
    case STEP_FINALIZE_SLICING:      return "STEP_FINALIZE_SLICING";
    case STEP_ORDERING:              return "STEP_ORDERING";
    case STEP_WIPETOWER:             return "STEP_WIPETOWER";
    case STEP_SUPPORT_SPOT:          return "STEP_SUPPORT_SPOT";
    case STEP_LAYER_EXTRUSION_EDIT:  return "STEP_LAYER_EXTRUSION_EDIT";
    case STEP_LAYER_STICHING:        return "STEP_LAYER_STICHING";
    case STEP_EXTRUSION_EDIT:        return "STEP_EXTRUSION_EDIT";
    case STEP_EXTRUSION_SIMPLIFICATION: return "STEP_EXTRUSION_SIMPLIFICATION";
    case STEP_GCODE:                 return "STEP_GCODE";
    case STEP_EXPORT:                return "STEP_EXPORT";
    case INFILL_PATTERN:             return "INFILL_PATTERN";
    case INFILL_SURFACE_RECIPE_MODIFIER: return "INFILL_SURFACE_RECIPE_MODIFIER";
    case BRIDGE_DETECTOR:            return "BRIDGE_DETECTOR";
    case PERIMETER_GENERATION_MODULE: return "PERIMETER_GENERATION_MODULE";
    case GCODE_FIRMWARE:             return "GCODE_FIRMWARE";
    case GCODE_THUMBNAIL:            return "GCODE_THUMBNAIL";
    case GCODE_READER:               return "GCODE_READER";
    case WIPE_TOWER_WIPE:            return "WIPE_TOWER_WIPE";
    default:                         return nullptr;
    }
}

inline std::string step_name_for_diagnostic(Orchestrator &orchestrator, slicing_step_t step)
{
    if (const char *name = builtin_step_name(step))
        return name;
    if (const Orchestrator::DynamicStepInfo *step_info = orchestrator.step_info(step))
        return step_info->name;
    return "step " + std::to_string(int(step));
}

// Stop a plugin phase after an error or stop request. The text names the cause when known.
// Callback handlers recognize this type and add no new diagnostic: a known cause
// already has its message; a plain stop request does not need a new error message.
class PluginStopped : public RuntimeError
{
public:
    using RuntimeError::RuntimeError;
};

// Handle the current exception inside a host callback. Call only from an active catch block.
// No C++ exception escapes to the C caller. A supplied Print selects its execution on any thread.
// PluginStopped keeps the stop requested without adding a message.
// CanceledException is left for the runner to rethrow at the next phase boundary.
// Other exceptions request a host-failure stop, then try to log and queue an error message.
// Failure to build or record that message must not undo the stop request.
inline void contain_callback_failure(Orchestrator &orchestrator, slicing_step_t step, const Print *print = nullptr) noexcept
{
    // Request the cancellation first (the request functions are noexcept); logging and message creation
    // allocate, so their failure is swallowed: it must neither escape nor undo the cancellation request.
    const auto record = [&orchestrator, step, print](const char *message) noexcept {
        orchestrator.request_plugin_cancel_after_host_failure(print);
        try {
            const char *step_name = builtin_step_name(step);
            BOOST_LOG_TRIVIAL(error) << (step_name != nullptr ? step_name : "step") << " host callback error: " << message;
            orchestrator.add_plugin_message(Orchestrator::PluginMessageLevel::Error, nullptr, step, message);
        } catch (...) {
        }
    };
    try {
        throw;
    } catch (const PluginStopped &) {
        orchestrator.request_plugin_cancel(print);
    } catch (const CanceledException &) {
    } catch (const std::exception &error) {
        record(error.what());
    } catch (...) {
        try {
            const char *step_name = builtin_step_name(step);
            const std::string message = std::string("Unknown C++ exception in a host callback called from ") +
                                        (step_name != nullptr ? step_name : "a step");
            record(message.c_str());
        } catch (...) {
            // Building the text failed: still stop the step.
            orchestrator.request_plugin_cancel_after_host_failure(print);
        }
    }
}

inline plugin_host_context prepare_guarded_plugin_host_context(Orchestrator &orchestrator,
                                                               slicing_step_t step,
                                                               Plugin &plugin,
                                                               Print &print,
                                                               std::atomic_bool &execution_error)
{
    plugin_host_context host_context = orchestrator.prepare_plugin_host_context(step, &plugin, &print);
    host_context.execution_error = &execution_error;
    return host_context;
}

// Check after setup(), setup_run() or run(). First rethrow user cancellation from Print.
// The C++ plugin wrapper absorbs PluginCancelled, so the host must check Print again.
// A plugin error or execution stop then throws PluginStopped. Its text identifies
// this plugin's error, another failing plugin, a host failure or an unknown stop cause.
// A known cause already has a queued diagnostic; this error points to it instead of repeating it.
// Diagnostic forms:
//  - own error:    <STEP> stopped after plugin 'Y' reported an error during <phase>.
//  - else, by the recorded stop cause of the orchestrator:
//    plugin Z:  <STEP> stopped during <phase> of plugin 'Y' after plugin 'Z' reported an error.
//    host:      <STEP> stopped during <phase> of plugin 'Y' after a host failure.
//    unknown:   <STEP> stopped during <phase> of plugin 'Y' after a stop was requested.
inline void throw_if_plugin_stopped(Orchestrator &orchestrator,
                                    const Print &print,
                                    slicing_step_t step,
                                    const Plugin &plugin,
                                    const std::atomic_bool &execution_error,
                                    const char *phase)
{
    // PluginCancelled is intentionally absorbed by the C++ ABI bridge. The
    // host therefore restores the public cancellation contract at every phase
    // boundary before considering plugin-specific failure state.
    print.throw_if_canceled();

    const bool has_error = execution_error.load(std::memory_order_relaxed);
    if (!has_error && !orchestrator.is_plugin_cancelled(&print))
        return;

    const std::string step_name = step_name_for_diagnostic(orchestrator, step);
    if (has_error)
        throw PluginStopped(step_name + " stopped after plugin '" + plugin.get_id() +
                            "' reported an error during " + phase + ".");

    // This plugin is healthy: name what stopped it, so the step error points to the real cause.
    const std::string prefix = step_name + " stopped during " + phase + " of plugin '" + plugin.get_id() + "' after ";
    const Orchestrator::PluginStopCause cause = orchestrator.plugin_stop_cause(&print);
    switch (cause.kind) {
    case Orchestrator::PluginStopCause::Kind::Plugin:
        throw PluginStopped(prefix + "plugin '" + cause.plugin->get_id() + "' reported an error.");
    case Orchestrator::PluginStopCause::Kind::Host:
        throw PluginStopped(prefix + "a host failure.");
    default:
        throw PluginStopped(prefix + "a stop was requested.");
    }
}

inline void validate_or_report(bool (*validator)(const Print &, std::string &),
                               const Print &print,
                               const char *validation_name)
{
    std::string error;
    if (validator(print, error))
        return;

    BOOST_LOG_TRIVIAL(error) << validation_name << " failed:\n" << error;
    assert(false && "Step data tree validation failed");
}

inline void validate_or_report(bool (*validator)(const Print &, std::string *),
                               const Print &print,
                               const char *validation_name)
{
    std::string error;
    if (validator(print, &error))
        return;

    BOOST_LOG_TRIVIAL(error) << validation_name << " failed:\n" << error;
    assert(false && "Step data tree validation failed");
}

// Run one plugin on the listed objects. setup() prepares one run per listed
// object, while each run sees its real object index among object_count objects.
struct ObjectStepReady { void operator()() const noexcept {} };

template<class PayloadFactory, class BeforeRun = ObjectStepReady>
void run_object_step_plugin(Orchestrator &orchestrator,
                            Print &print,
                            slicing_step_t step,
                            Plugin &plugin,
                            const std::vector<size_t> &object_indices,
                            size_t object_count,
                            PayloadFactory payload_factory,
                            BeforeRun before_run = {})
{
    assert(object_indices.size() <= UINT32_MAX);
    std::atomic_bool execution_error { false };
    plugin_host_context host_context =
        prepare_guarded_plugin_host_context(orchestrator, step, plugin, print, execution_error);
    host_context.object_count = object_count;
    plugin_run_context run_context = orchestrator.prepare_plugin_run_context(step, &plugin, &host_context);
    plugin.setup(run_context, uint32_t(object_indices.size()));
    throw_if_plugin_stopped(orchestrator, print, step, plugin, execution_error, "setup");

    // Prepare every object payload before any object run starts. This gives
    // plugins a deterministic discovery pass even if the host executes the
    // per-object setup in parallel.
    parallel_for(size_t(0), object_indices.size(), [&host_context, &object_indices, &payload_factory, &run_context, &plugin](const size_t run_idx) {
        const size_t idx = object_indices[run_idx];
        plugin_run_context context_copy = run_context;
        plugin_host_context host_context_copy = host_context;
        host_context_copy.object_idx = idx;
        context_copy.host_context = &host_context_copy;
        if (context_copy.is_cancelled != nullptr && context_copy.is_cancelled(context_copy.host_context))
            return;

        decltype(payload_factory(idx)) payload = payload_factory(idx);
        context_copy.data = &payload;
        plugin.setup_run(context_copy);
    });
    throw_if_plugin_stopped(orchestrator, print, step, plugin, execution_error, "setup_run");

    // Optional phase-local services may seal their discovery data here. No
    // object worker is running, and execution has not yet borrowed that data.
    before_run();

    parallel_for(size_t(0), object_indices.size(), [&host_context, &object_indices, &payload_factory, &run_context, &plugin](const size_t run_idx) {
        const size_t idx = object_indices[run_idx];
        plugin_run_context context_copy = run_context;
        plugin_host_context host_context_copy = host_context;
        host_context_copy.object_idx = idx;
        context_copy.host_context = &host_context_copy;
        if (context_copy.is_cancelled != nullptr && context_copy.is_cancelled(context_copy.host_context))
            return;

        decltype(payload_factory(idx)) payload = payload_factory(idx);
        context_copy.data = &payload;
        plugin.run(context_copy);
    });
    throw_if_plugin_stopped(orchestrator, print, step, plugin, execution_error, "run");
}

// Run one plugin on objects 0 .. run_count-1 of a print with run_count objects.
template<class PayloadFactory, class BeforeRun = ObjectStepReady>
void run_object_step_plugin(Orchestrator &orchestrator,
                            Print &print,
                            slicing_step_t step,
                            Plugin &plugin,
                            size_t run_count,
                            PayloadFactory payload_factory,
                            BeforeRun before_run = {})
{
    std::vector<size_t> object_indices(run_count);
    std::iota(object_indices.begin(), object_indices.end(), size_t(0));
    run_object_step_plugin(orchestrator, print, step, plugin, object_indices, run_count, payload_factory, before_run);
}

template<class PayloadFactory>
void run_object_step_plugins(Orchestrator &orchestrator,
                             Print &print,
                             slicing_step_t step,
                             const ConfigBase *config,
                             size_t run_count,
                             PayloadFactory payload_factory)
{
    std::vector<Plugin *> plugins = selected_or_active_plugins_for_step(orchestrator, step, config);

    for (Plugin *plugin : plugins) {
        if (plugin != nullptr) {
            run_object_step_plugin(orchestrator, print, step, *plugin, run_count, payload_factory);
            print.pipeline_progress().complete_plugin(step, plugin->get_id());
        }
    }
}

} // namespace Slic3r::Steps::Detail

#endif // steps_steprunner_hpp_
