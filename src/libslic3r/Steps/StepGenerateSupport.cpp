
#include "StepGenerateSupport.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"

#include "StepSupportDemand.hpp"

/* This module registers support-generation selector metadata; its execution hooks do no work.
   StepSelectorTraits.hpp defines the metadata copied into Orchestrator, the plugin manager.
   Call graph:
     register_selector_traits
       Orchestrator::register_builtin_step_selector
     run_step(with support state)
       run_step
   Registration writes declarations, not print data or settings values. */

namespace Slic3r::Steps::StepGenerateSupport {

// Install the support-generation selector declaration on the supplied host.
void register_selector_traits(Orchestrator &orchestrator);

void register_selector_traits(Orchestrator &orchestrator)
{
    StepSelectorTraits traits;
    traits.category = RAW_OPTION_CATEGORY_SUPPORT;
    traits.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    traits.invalidates_step = STEP_SUPPORT;
    traits.show_with_one_provider = false;
    const int32_t registered = orchestrator.register_builtin_step_selector(STEP_SUPPORT, std::move(traits));
    assert(registered == 1);
    (void)registered;
}

void clear(Print &) {}
void prepare(Print &) {}

bool validate_pre(const Print &, std::string *)
{
    return true;
}

bool validate_post(const Print &, std::string *)
{
    return true;
}

void run_step(Orchestrator &, Print &) {}

void run_step(Orchestrator &orchestrator, Print &print, const StepSupportDemand::State &)
{
    run_step(orchestrator, print);
}

} // namespace Slic3r::Steps::StepGenerateSupport
