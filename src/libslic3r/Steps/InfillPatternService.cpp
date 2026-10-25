///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/* Implementation of the shared INFILL_PATTERN service (see the header for the
   usage sequence and the concurrency contract).

   The constructor is the only writer: it walks the active pattern plugins once
   and stores, per plugin, the runtime id, the guarded host context, the error
   flag and a ready-made plugin_run_context. Every later call only reads those
   entries and copies the run context, so no map insertion can happen while
   step plugins call patterns from several threads. */
#include "InfillPatternService.hpp"

#include <cassert>
#include <set>
#include <string>

#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Steps/StepRunner.hpp"

namespace Slic3r::Steps {
namespace {

// True for pattern plugins that build their state in setup() from the
// discovery payload of STEP_INFILL and fail without it.
bool pattern_requires_setup(const Plugin &plugin);

// The C ABI has no flag telling that a pattern needs setup-time
// initialization, so the known ones are listed by stable id. A pattern added to
// this family must be added here to get a clear refusal outside STEP_INFILL.
bool pattern_requires_setup(const Plugin &plugin)
{
    static const std::set<std::string> ids { "lightning", "adaptivecubic", "supportcubic" };
    return ids.count(plugin.get_id()) != 0;
}

} // namespace

InfillPatternService::InfillPatternService(Orchestrator &orchestrator,
                                           Print &print,
                                           size_t run_count,
                                           SetupPatterns setup_patterns)
    : m_orchestrator(orchestrator), m_print(print), m_setup_patterns(setup_patterns)
{
    // Runtime ids are one-based so 0 can remain the invalid value exposed by
    // the C ABI. They are rebuilt for each run; plugin config and UI keep using
    // plugin->get_id().
    infill_pattern_runtime_id next_runtime_id = 1;
    for (Plugin *plugin : orchestrator.get_active_plugins_for_step(INFILL_PATTERN)) {
        if (plugin == nullptr)
            continue;

        const infill_pattern_runtime_id runtime_id = next_runtime_id++;
        m_runtime_ids_by_plugin_id.emplace(plugin->get_id(), runtime_id);
        Pattern &pattern = m_patterns[runtime_id];
        pattern.plugin = plugin;
        pattern.host_context = Detail::prepare_guarded_plugin_host_context(
            orchestrator, INFILL_PATTERN, *plugin, print, pattern.execution_error);
        pattern.host_context.object_count = run_count;
        // Preparing the run context here also creates the plugin storage entry,
        // so concurrent calls never insert into the orchestrator storage map.
        pattern.run_context = orchestrator.prepare_plugin_run_context(INFILL_PATTERN, plugin, &pattern.host_context);
    }

    // Without an initialization pass, setup() must still run once before the
    // first call: it binds per-run plugin state such as the progress reporter
    // to this run's host context, which would otherwise point to a context of
    // an earlier, destroyed run. The empty payload tells the pattern that no
    // work group was discovered.
    if (setup_patterns == SetupPatterns::Refuse) {
        run_ctx_infill_pattern_init payload = {};
        payload.print = reinterpret_cast<const print_handle *>(&print);
        for (const std::pair<const infill_pattern_runtime_id, Pattern> &entry : m_patterns)
            if (!pattern_requires_setup(*entry.second.plugin))
                this->initialize_pattern(entry.first, payload, uint32_t(run_count));
    }
}

infill_pattern_runtime_id InfillPatternService::resolve_pattern_id(const char *pattern_plugin_id) const
{
    const std::string requested_id = pattern_plugin_id != nullptr ? pattern_plugin_id : "";
    const std::map<std::string, infill_pattern_runtime_id>::const_iterator selected =
        m_runtime_ids_by_plugin_id.find(requested_id);
    if (selected != m_runtime_ids_by_plugin_id.end())
        return selected->second;

    // The monotoniclines pattern was removed because it was identical to
    // monotonic. Presets may still store its id; resolve it to monotonic so
    // they keep slicing with the same result instead of the generic fallback.
    if (requested_id == "monotoniclines")
        return resolve_pattern_id("monotonic");

    // Presets can outlive plugin activation changes. Falling back keeps the
    // slice usable, while the warning tells the user that the selected pattern
    // id is no longer available in the active plugin set.
    if (!m_patterns.empty()) {
        const std::map<infill_pattern_runtime_id, Pattern>::const_iterator fallback = m_patterns.begin();
        const std::string message = "Infill pattern '" + requested_id + "' is not active; using '" +
                                    fallback->second.plugin->get_id() + "' instead.";
        m_orchestrator.add_plugin_message(Orchestrator::PluginMessageLevel::Warning,
                                          fallback->second.plugin,
                                          INFILL_PATTERN,
                                          message.c_str());
        return fallback->first;
    }

    return INFILL_PATTERN_RUNTIME_ID_INVALID;
}

const char *InfillPatternService::pattern_plugin_id(infill_pattern_runtime_id pattern_id) const
{
    // The returned string is owned by Plugin and stays valid while the plugin
    // registry exists; callers treat it as borrowed.
    const std::map<infill_pattern_runtime_id, Pattern>::const_iterator found = m_patterns.find(pattern_id);
    if (found == m_patterns.end())
        return "";
    return found->second.plugin->get_id().c_str();
}

void InfillPatternService::initialize_pattern(infill_pattern_runtime_id pattern_id,
                                              run_ctx_infill_pattern_init payload,
                                              uint32_t surface_count)
{
    const std::map<infill_pattern_runtime_id, Pattern>::iterator found = m_patterns.find(pattern_id);
    if (found == m_patterns.end())
        return;

    Pattern &pattern = found->second;
    plugin_run_context setup_context = pattern.run_context;
    setup_context.data = &payload;
    pattern.plugin->setup(setup_context, surface_count);
    Detail::throw_if_plugin_stopped(
        m_orchestrator, m_print, INFILL_PATTERN, *pattern.plugin, pattern.execution_error, "initialize_object");
}

int32_t InfillPatternService::generate_pattern(infill_pattern_runtime_id pattern_id, run_ctx_infill_pattern payload)
{
    Pattern *pattern = this->pattern_for_generation(pattern_id);
    if (pattern == nullptr)
        return 0;

    // Without the STEP_INFILL initialization pass these patterns have no
    // per-object cache and would fail inside run(). Refuse them up front with
    // an error the user can act on instead.
    if (m_setup_patterns == SetupPatterns::Refuse && pattern_requires_setup(*pattern->plugin)) {
        const std::string message = "Infill pattern '" + pattern->plugin->get_id() +
                                    "' needs the STEP_INFILL initialization pass and cannot be called "
                                    "from this step; no path was generated.";
        m_orchestrator.add_plugin_message(Orchestrator::PluginMessageLevel::Error,
                                          pattern->plugin,
                                          INFILL_PATTERN,
                                          message.c_str());
        return 0;
    }

    // The pattern plugin receives the exact surface recipe and an empty output
    // extrusion root. It does not know how it was selected; that remains the
    // responsibility of the calling step and the runtime registry above.
    plugin_run_context run_context = pattern->run_context;
    run_context.data = &payload;
    pattern->plugin->setup_run(run_context);
    Detail::throw_if_plugin_stopped(
        m_orchestrator, m_print, INFILL_PATTERN, *pattern->plugin, pattern->execution_error, "setup_run");
    pattern->plugin->run(run_context);
    Detail::throw_if_plugin_stopped(
        m_orchestrator, m_print, INFILL_PATTERN, *pattern->plugin, pattern->execution_error, "run");
    return 1;
}

InfillPatternService::Pattern *InfillPatternService::pattern_for_generation(infill_pattern_runtime_id pattern_id)
{
    if (pattern_id == INFILL_PATTERN_RUNTIME_ID_INVALID)
        return nullptr;

    const std::map<infill_pattern_runtime_id, Pattern>::iterator selected = m_patterns.find(pattern_id);
    if (selected != m_patterns.end())
        return &selected->second;

    // A runtime id is created only by this service for the current run. If it
    // cannot be found here, a caller kept an id past its valid lifetime or wrote
    // an arbitrary value. Do not guess a pattern: emitting no extrusion is
    // safer than silently using the wrong algorithm.
    const std::string message = "Unknown runtime infill pattern id '" + std::to_string(pattern_id) + "'.";
    m_orchestrator.add_plugin_message(Orchestrator::PluginMessageLevel::Warning,
                                      nullptr,
                                      INFILL_PATTERN,
                                      message.c_str());
    return nullptr;
}

} // namespace Slic3r::Steps
