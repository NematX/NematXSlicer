///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "StepSupportDemand.hpp"

#include <atomic>
#include <memory>
#include <vector>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/host/steps/SupportDemandStep.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Thread.hpp"

#include "StepPipeline.hpp"
#include "StepRunner.hpp"

/* This module collects the areas that plugins identify as needing support for each print object.
   Orchestrator is the plugin manager; StepSelectorTraits.hpp defines the selector metadata registered here.
   Call graph:
     register_selector_traits
       Orchestrator::register_builtin_step_selector
     run_step
       State::reset
       selected_or_active_plugins_for_step
       State::demand_for
       Plugin::setup
       Plugin::setup_run
       Plugin::run
   Plugins read print objects and settings and write support requests into State.
   Plugins run in sequence; each plugin's per-object work runs in parallel. */

namespace Slic3r::Steps::StepSupportDemand {

// Install the support-demand selector declaration on the supplied host.
void register_selector_traits(Orchestrator &orchestrator);

void register_selector_traits(Orchestrator &orchestrator)
{
    StepSelectorTraits traits;
    traits.category = RAW_OPTION_CATEGORY_SUPPORT;
    traits.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    const int32_t registered = orchestrator.register_builtin_step_selector(STEP_SUPPORT_DEMAND, std::move(traits));
    assert(registered == 1);
    (void)registered;
}

State::State() = default;
State::~State() = default;
State::State(State &&) noexcept = default;
State &State::operator=(State &&) noexcept = default;

void State::reset()
{
    m_demands.clear();
}

size_t State::object_count() const
{
    return m_demands.size();
}

ApiHost::Steps::SupportDemandSet &State::demand_for(PrintObject &object)
{
    std::unique_ptr<ApiHost::Steps::SupportDemandSet> &demand = m_demands[&object];
    if (demand == nullptr)
        demand = std::make_unique<ApiHost::Steps::SupportDemandSet>();
    return *demand;
}

const ApiHost::Steps::SupportDemandSet *State::demand_for(const PrintObject &object) const
{
    std::map<const PrintObject *, std::unique_ptr<ApiHost::Steps::SupportDemandSet>>::const_iterator it =
        m_demands.find(&object);
    return it == m_demands.end() ? nullptr : it->second.get();
}

void clear(Print &) {}
void prepare(Print &) {}

bool validate_pre(const Print &, std::string &)
{
    return true;
}

bool validate_post(const Print &, std::string &)
{
    return true;
}

State run_step(Orchestrator &orchestrator, Print &print)
{
    State state;
    run_step(orchestrator, print, state);
    return state;
}

void run_step(Orchestrator &orchestrator, Print &print, State &state)
{
    Detail::validate_or_report(validate_pre, print, "Support-demand pre-step validation");

    const size_t run_count = print.objects().size();
    state.reset();

    std::vector<Plugin *> plugins = selected_or_active_plugins_for_step(orchestrator,
                                                                        STEP_SUPPORT_DEMAND,
                                                                        &print.full_print_config());
    if (plugins.empty())
        return;

    for (Plugin *plugin : plugins) {
        std::atomic_bool execution_error { false };
        plugin_host_context host_context =
            Detail::prepare_guarded_plugin_host_context(
                orchestrator, STEP_SUPPORT_DEMAND, *plugin, print, execution_error);
        plugin_run_context run_context =
            orchestrator.prepare_plugin_run_context(STEP_SUPPORT_DEMAND, plugin, &host_context);
        plugin->setup(run_context, uint32_t(run_count));
        Detail::throw_if_plugin_stopped(orchestrator, print, STEP_SUPPORT_DEMAND, *plugin, execution_error, "setup");

        std::vector<std::unique_ptr<ApiHost::Steps::SupportDemandRunContext>> run_contexts;
        run_contexts.reserve(run_count);
        for (size_t object_idx = 0; object_idx < run_count; ++object_idx) {
            run_contexts.push_back(
                ApiHost::Steps::make_support_demand_run_context(print,
                                                                object_idx,
                                                                state.demand_for(print.object(object_idx))));
        }

        parallel_for(size_t(0), run_count, [plugin, &run_context, &run_contexts](const size_t object_idx) {
            plugin_run_context context_copy = run_context;
            if (context_copy.is_cancelled != nullptr && context_copy.is_cancelled(context_copy.host_context))
                return;

            context_copy.data = &run_contexts[object_idx]->context_step;
            plugin->setup_run(context_copy);
        });
        Detail::throw_if_plugin_stopped(orchestrator, print, STEP_SUPPORT_DEMAND, *plugin, execution_error, "setup_run");

        parallel_for(size_t(0), run_count, [plugin, &run_context, &run_contexts](const size_t object_idx) {
            plugin_run_context context_copy = run_context;
            if (context_copy.is_cancelled != nullptr && context_copy.is_cancelled(context_copy.host_context))
                return;

            context_copy.data = &run_contexts[object_idx]->context_step;
            plugin->run(context_copy);
        });
        Detail::throw_if_plugin_stopped(orchestrator, print, STEP_SUPPORT_DEMAND, *plugin, execution_error, "run");

        Detail::validate_or_report(validate_post, print, "Support-demand post-plugin validation");
        print.pipeline_progress().complete_plugin(STEP_SUPPORT_DEMAND, plugin->get_id());
    }
}

} // namespace Slic3r::Steps::StepSupportDemand
