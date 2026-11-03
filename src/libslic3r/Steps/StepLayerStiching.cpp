
#include "StepLayerStiching.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"

/* This module registers selector metadata for layer stitching; its execution hooks do no work.
   StepSelectorTraits.hpp defines the metadata copied into Orchestrator, the plugin manager.
   Call graph:
     register_selector_traits
       Orchestrator::register_builtin_step_selector
   Registration writes declarations, not print data or settings values. */

namespace Slic3r::Steps::StepLayerStiching {

// Install the layer-stitching selector declaration on the supplied host.
void register_selector_traits(Orchestrator &orchestrator);

void register_selector_traits(Orchestrator &orchestrator)
{
    StepSelectorTraits traits;
    traits.category = RAW_OPTION_CATEGORY_OUTPUT;
    traits.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    traits.invalidates_step = STEP_LAYER_STICHING;
    traits.show_with_one_provider = false;
    const int32_t registered = orchestrator.register_builtin_step_selector(STEP_LAYER_STICHING, std::move(traits));
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

} // namespace Slic3r::Steps::StepLayerStiching
