///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/* Support families build auxiliary layers and typed surfaces at STEP_SUPPORT.
   This step dispatches selected toolpath algorithms once per object, in priority
   order. Outputs belong to region islands, not to the dispatcher, and disappear
   with the auxiliary layers when the Slicing checkpoint is replayed. */
#ifndef steps_stepsupporttoolpath_hpp_
#define steps_stepsupporttoolpath_hpp_

#include <string>

namespace Slic3r {
class Orchestrator;
class Print;

namespace Steps::StepSupportToolpath {

// Nothing to clear (see the .cpp).
void clear(Print &print);
// Nothing to prepare (see the .cpp).
void prepare(Print &print);
// Accept the input tree without imposing algorithm-specific requirements.
bool validate_pre(const Print &print, std::string &error);
// Accept the output tree without imposing algorithm-specific requirements.
bool validate_post(const Print &print, std::string &error);
// Run selected support toolpath algorithms sequentially for every object.
void run_step(Orchestrator &orchestrator, Print &print);

} // namespace Steps::StepSupportToolpath
} // namespace Slic3r

#endif // steps_stepsupporttoolpath_hpp_
