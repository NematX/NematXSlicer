///|/ Copyright (c) SuperSlicer 2026 Durand R?mi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "StepSupportDemand.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/steps/SupportDemandStep.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_support_demand.h"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Api/plugin/cpp/properties/BridgeProperties.hpp"

#include "StepPipeline.hpp"
#include "StepRunner.hpp"

/* Collect the areas that plugins identify as needing support for each print object.
   Requests belong to layer islands and describe demand, not deposited material.
   Orchestrator manages providers; StepSelectorTraits.hpp defines their selector
   metadata. A provider reads object geometry/settings and adds requests to the
   demand left by preceding providers.

   clear() removes island requests and all SupportPipelineInfo completion flags.
   After successful joined dispatch and post-validation, run_step() marks demand
   computed even when every request is empty. An uncomputed mask must not be
   interpreted as a completed empty result by later support/finalization steps.

       register_selector_traits()
           `-- Orchestrator::register_builtin_step_selector()

       clear()
           `-- ApiHost::Steps::clear_support_demand()

       run_step()
           |-- Detail::validate_or_report() [before and after dispatch]
           `-- Detail::run_object_step_plugins() [provider lifecycle and joined object runs]

   Lifecycle calls to Plugin::setup/setup_run/run belong to the dispatcher, not
   directly to run_step(). Providers run in sequence; one provider's object work
   may run in parallel. Print/object handles in its payload are borrowed.
*/

namespace Slic3r::Steps::StepSupportDemand {

// Install the support-demand selector declaration on the supplied host.
void register_selector_traits(Orchestrator &orchestrator);
// Empty layer-island requests and remove all support completion flags from the
// borrowed print's objects; host failures propagate without ownership transfer.
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
    for (PrintObject &object : print.objects())
        object.remove_property<slic3r_api::SupportPipelineInfo>();
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
    // A completed empty demand differs from a pass that has not run.
    for (PrintObject &object : print.objects())
        object.get_or_add_property<slic3r_api::SupportPipelineInfo>().demand_computed = 1;
}

} // namespace Slic3r::Steps::StepSupportDemand
