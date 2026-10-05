///|/ Copyright (c) SuperSlicer 2026 Durand R?mi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef steps_stepsupportdemand_hpp_
#define steps_stepsupportdemand_hpp_

/*
STEP_SUPPORT_DEMAND runs right after STEP_POST_SLICING. Its plugins write, in
each LayerSliceIsland, the areas that ask for support. The demand stays in the
islands: later steps read it through the data tree and support generation
consumes it. clear() empties it so a replay from Checkpoint::Slicing rebuilds it.
*/

#include <string>

#include "libslic3r/Api/plugin/c/slic3r_plugin_types.h"

namespace Slic3r {
class Orchestrator;
class Print;

namespace Steps::StepSupportDemand {

void clear(Print &print);
void prepare(Print &print);
bool validate_pre(const Print &print, std::string &error);
bool validate_post(const Print &print, std::string &error);
void run_step(Orchestrator &orchestrator, Print &print);

} // namespace Steps::StepSupportDemand
} // namespace Slic3r

#endif // steps_stepsupportdemand_hpp_
