
#include "StepGenerateSupport.hpp"

#include "libslic3r/Api/plugin/c/steps/slic3r_step_support.h"
#include "StepRunner.hpp"

namespace Slic3r::Steps::StepGenerateSupport {

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

// Call the active support families in priority order for each object; they publish auxiliary surfaces and runner errors stop the step.
void run_step(Orchestrator &orchestrator, Print &print)
{
    Detail::run_object_step_plugins(orchestrator,
                                   print,
                                   STEP_SUPPORT,
                                   &print.full_print_config(),
                                   print.objects().size(),
                                   [&print](const size_t object_idx) {
        run_ctx_generate_support context = {};
        context.print = reinterpret_cast<const print_handle *>(&print);
        context.object = reinterpret_cast<const object_handle *>(&print.object(object_idx));
        return context;
    });
}

} // namespace Slic3r::Steps::StepGenerateSupport
