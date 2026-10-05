///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "StepSupportToolpath.hpp"

#include "libslic3r/Api/plugin/c/steps/slic3r_step_support_toolpath.h"
#include "StepRunner.hpp"

/* The object-step runner selects one algorithm per exclusive role group and
   orders the selected algorithms by priority. Each callback processes one
   object's support surfaces and adds extrusions through the data tree.
   The dispatcher does not store geometry or plugin output. */

namespace Slic3r::Steps::StepSupportToolpath {

// Nothing to clear: the extrusions live in the support auxiliary layers, which a Slicing replay destroys.
void clear(Print &) {}

// Nothing to prepare: the algorithms read the surfaces left by the support families.
void prepare(Print &) {}

// No common input check: input validation belongs to each algorithm.
bool validate_pre(const Print &, std::string &)
{
    return true;
}

// No common output check: output validation belongs to each algorithm.
bool validate_post(const Print &, std::string &)
{
    return true;
}

// Run the selected toolpath algorithms on every object.
void run_step(Orchestrator &orchestrator, Print &print)
{
    Detail::validate_or_report(validate_pre, print, "Support-toolpath pre-step validation");
    Detail::run_object_step_plugins(orchestrator,
                                    print,
                                    STEP_SUPPORT_TOOLPATH,
                                    &print.full_print_config(),
                                    print.objects().size(),
                                    [&print](const size_t object_idx) {
        run_ctx_support_toolpath context = {};
        context.print = reinterpret_cast<const print_handle *>(&print);
        context.object = reinterpret_cast<const object_handle *>(&print.object(object_idx));
        return context;
    });
    Detail::validate_or_report(validate_post, print, "Support-toolpath post-step validation");
}

} // namespace Slic3r::Steps::StepSupportToolpath
