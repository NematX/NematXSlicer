
#include "StepPrepareForPeriemters.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"

#include "libslic3r/Api/plugin/c/steps/slic3r_step_pre_perimeter.h"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"

#include "StepRunner.hpp"

/* This module runs perimeter-preparation plugins and clears auxiliary layers used for skirts and brims.
   It also registers selector metadata defined by StepSelectorTraits.hpp with Orchestrator, the plugin manager.
   Call graph:
     register_selector_traits
       Orchestrator::register_builtin_step_selector
     clear
       PrintObject::clear_auxiliary_layers
       Print::reset_auxiliary_object
     run_step
       Detail::run_object_step_plugins
   Plugins receive the print and each object's data through the step context. */

namespace Slic3r::Steps::StepPrepareForPeriemters {

// Install the perimeter-preparation selector declaration on the supplied host.
void register_selector_traits(Orchestrator &orchestrator);

void register_selector_traits(Orchestrator &orchestrator)
{
    StepSelectorTraits traits;
    traits.category = RAW_OPTION_CATEGORY_PERIMETER;
    traits.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    const int32_t registered = orchestrator.register_builtin_step_selector(STEP_PRE_PERIMETER, std::move(traits));
    assert(registered == 1);
    (void)registered;
}

void clear(Print &print)
{
    for (PrintObject &object : print.objects())
        object.clear_auxiliary_layers();
    // Print-level auxiliaries have a hidden owner and are not included in print.objects().
    print.reset_auxiliary_object();
}
void prepare(Print &) {}

bool validate_pre(const Print &, std::string &)
{
    return true;
}

bool validate_post(const Print &, std::string &)
{
    return true;
}

void run_step(Orchestrator &orchestrator, Print &print)
{
    Detail::run_object_step_plugins(orchestrator,
                                    print,
                                    STEP_PRE_PERIMETER,
                                    &print.full_print_config(),
                                    print.objects().size(),
                                    [&print](const size_t object_idx) {
        run_ctx_prepare_for_perimeters context = {};
        context.print = reinterpret_cast<const print_handle *>(&print);
        context.object = reinterpret_cast<const object_handle *>(&print.object(object_idx));
        return context;
    });
}

} // namespace Slic3r::Steps::StepPrepareForPeriemters
