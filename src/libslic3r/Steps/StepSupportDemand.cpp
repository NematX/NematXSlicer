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

namespace Slic3r::Steps::StepSupportDemand {

void clear(Print &print)
{
    ApiHost::Steps::clear_support_demand(print);
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
