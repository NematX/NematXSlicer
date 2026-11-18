///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#ifndef slic3r_Api_host_steps_InfillStep_hpp_
#define slic3r_Api_host_steps_InfillStep_hpp_
#include <memory>
#include "libslic3r/Api/plugin/c/steps/slic3r_step_infill.h"

namespace Slic3r { class Orchestrator; class Print; class PrintObject; }
namespace Slic3r::ApiHost::Steps {
struct InfillGenerationHostContext;
// Host side of the STEP_INFILL payload: own the C callbacks' guarded services for one plugin phase. The object
// runner joins discovery before finish_discovery(); the first pattern generation
// after it initializes the used patterns once, with the work of all objects.
class InfillGenerationContext {
public:
    InfillGenerationContext(Orchestrator &orchestrator, Print &print);
    ~InfillGenerationContext();
    InfillGenerationContext(const InfillGenerationContext &) = delete;
    InfillGenerationContext &operator=(const InfillGenerationContext &) = delete;
    run_ctx_generate_infill payload(PrintObject &object) const;
    void finish_discovery() noexcept;
private:
    std::unique_ptr<InfillGenerationHostContext> m_host;
};
}
#endif
