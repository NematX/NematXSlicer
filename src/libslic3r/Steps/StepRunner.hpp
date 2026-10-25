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
#include <string>
#include <vector>

#include <boost/log/trivial.hpp>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/DataTreeFwd.hpp"
#include "libslic3r/Exception.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Thread.hpp"

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
    case STEP_FINALIZE_SLICING:      return "STEP_FINALIZE_SLICING";
    case STEP_ORDERING:              return "STEP_ORDERING";
    case STEP_WIPETOWER:             return "STEP_WIPETOWER";
    case STEP_SUPPORT_SPOT:          return "STEP_SUPPORT_SPOT";
    case STEP_LAYER_EXTRUSION_EDIT:  return "STEP_LAYER_EXTRUSION_EDIT";
    case STEP_LAYER_STICHING:        return "STEP_LAYER_STICHING";
    case STEP_EXTRUSION_EDIT:        return "STEP_EXTRUSION_EDIT";
    case STEP_EXTRUSION_SIMPLIFICATION: return "STEP_EXTRUSION_SIMPLIFICATION";
    case STEP_GCODE:                 return "STEP_GCODE";
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

// Contain the exception being handled by the calling catch block at a host callback boundary.
// Host callbacks are called through the C ABI, so no C++ exception may escape them. The callback
// catches everything and calls this function from its catch block: `throw;` rethrows the active
// exception into the local handlers, which sort it. A CanceledException is dropped (the runner rethrows
// the cancellation at the next phase boundary, see throw_if_plugin_stopped); every other exception is
// logged, added as an error message of the step with its what() text (or an unknown-exception text), and
// the plugin cancellation is requested so that the runner stops the step. Nothing escapes: building
// the text is itself guarded, and the plugin cancellation is requested in the worst case.
// Must only be called from an active catch block.
inline void contain_callback_failure(Orchestrator &orchestrator, slicing_step_t step) noexcept
{
    // Request the cancellation first, it cannot fail; logging and message creation allocate, so their
    // failure is swallowed: it must neither escape nor undo the cancellation request.
    const auto record = [&orchestrator, step](const char *message) noexcept {
        orchestrator.request_plugin_cancel();
        try {
            const char *step_name = builtin_step_name(step);
            BOOST_LOG_TRIVIAL(error) << (step_name != nullptr ? step_name : "step") << " host callback error: " << message;
            orchestrator.add_plugin_message(Orchestrator::PluginMessageLevel::Error, nullptr, step, message);
        } catch (...) {
        }
    };
    try {
        throw;
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
            orchestrator.request_plugin_cancel();
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
    if (!has_error && !orchestrator.is_plugin_cancelled())
        return;

    const std::string reason = has_error ? "reported an error" : "was cancelled";
    throw RuntimeError(
        step_name_for_diagnostic(orchestrator, step) + " stopped after plugin '" +
        plugin.get_id() + "' " + reason + " during " + phase + ".");
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

template<class PayloadFactory>
void run_object_step_plugin(Orchestrator &orchestrator,
                            Print &print,
                            slicing_step_t step,
                            Plugin &plugin,
                            size_t run_count,
                            PayloadFactory payload_factory)
{
    assert(run_count <= UINT32_MAX);
    std::atomic_bool execution_error { false };
    plugin_host_context host_context =
        prepare_guarded_plugin_host_context(orchestrator, step, plugin, print, execution_error);
    host_context.object_count = run_count;
    plugin_run_context run_context = orchestrator.prepare_plugin_run_context(step, &plugin, &host_context);
    plugin.setup(run_context, uint32_t(run_count));
    throw_if_plugin_stopped(orchestrator, print, step, plugin, execution_error, "setup");

    // Prepare every object payload before any object run starts. This gives
    // plugins a deterministic discovery pass even if the host executes the
    // per-object setup in parallel.
    parallel_for(size_t(0), run_count, [&host_context, &payload_factory, &run_context, &plugin](const size_t idx) {
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

    parallel_for(size_t(0), run_count, [&host_context, &payload_factory, &run_context, &plugin](const size_t idx) {
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
