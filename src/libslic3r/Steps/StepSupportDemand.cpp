///|/ Copyright (c) SuperSlicer 2026 Durand R?mi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "StepSupportDemand.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/steps/SupportDemandStep.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_support_demand.h"
#include "libslic3r/Print.hpp"

#include "StepPipeline.hpp"
#include "StepRunner.hpp"

/* This module collects the areas that plugins identify as needing support for each print object.
   Orchestrator is the plugin manager; StepSelectorTraits.hpp defines the selector metadata registered here.
   Call graph:
     register_selector_traits
       Orchestrator::register_builtin_step_selector
     run_step
       Detail::run_object_step_plugins
       Plugin::setup
       Plugin::setup_run
       Plugin::run
   Plugins read print objects and settings and write support requests into their layer islands.
   Plugins run in sequence; each plugin's per-object work runs in parallel. */

namespace Slic3r::Steps::StepSupportDemand {

// Install the support-demand selector declaration on the supplied host.
void register_selector_traits(Orchestrator &orchestrator);
// Empty support requests in the borrowed print's layer islands; no ownership transfers and host failures propagate.
void clear(Print &print);

void register_selector_traits(Orchestrator &orchestrator)
{
    StepSelectorTraits traits;
    traits.category = RAW_OPTION_CATEGORY_SUPPORT;
    traits.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    const int32_t registered = orchestrator.register_builtin_step_selector(STEP_SUPPORT_DEMAND, std::move(traits));
    assert(registered == 1);
    (void)registered;
}

void clear(Print &print)
{
    ApiHost::Steps::clear_support_demand(print);
}

void prepare(Print &) {}

// Accepts borrowed input without modifying the optional error string; always returns true without failure.
bool validate_pre(const Print &, std::string *)
{
    return true;
}

// Accepts borrowed output without modifying the optional error string; always returns true without failure.
bool validate_post(const Print &, std::string *)
{
    return true;
}

void run_step(Orchestrator &orchestrator, Print &print)
{
    Detail::validate_or_report(validate_pre, print, "Support-demand pre-step validation");
    // Each plugin starts from the demand left by the previous ones; the first one
    // starts from empty islands (clear() or freshly sliced layers).
    Detail::run_object_step_plugins(orchestrator,
                                    print,
                                    STEP_SUPPORT_DEMAND,
                                    &print.full_print_config(),
                                    print.objects().size(),
                                    [&print](const size_t object_idx) {
        run_ctx_support_demand context = {};
        context.print = reinterpret_cast<const print_handle *>(&print);
        context.object = reinterpret_cast<const object_handle *>(&print.object(object_idx));
        return context;
    });
    Detail::validate_or_report(validate_post, print, "Support-demand post-step validation");
}

} // namespace Slic3r::Steps::StepSupportDemand
