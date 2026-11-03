///|/ Copyright (c) SuperSlicer 2026 Durand R?mi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef steps_stepgenerateinfill_hpp_
#define steps_stepgenerateinfill_hpp_

#include <string>

#include "libslic3r/Api/plugin/c/slic3r_plugin_types.h"

/* This interface lets StepPipeline generate infill through the selected plugin.
   Register the settings metadata defined by StepSelectorTraits.hpp with register_selector_traits()
   at host startup. For a print, use prepare(), validate_pre(), run_step() and validate_post();
   clear() leaves infill in place until a slicing restart removes its region islands. */
namespace Slic3r {
class Orchestrator;
class Print;

namespace Steps::StepGenerateInfill {

// Declare host selector traits for infill generation on the supplied host.
void register_selector_traits(Orchestrator &orchestrator);

void clear(Print &print);
void prepare(Print &print);
bool validate_pre(const Print &print, std::string *error = nullptr);
bool validate_post(const Print &print, std::string *error = nullptr);
void run_step(Orchestrator &orchestrator, Print &print);

} // namespace Steps::StepGenerateInfill
} // namespace Slic3r

#endif // steps_stepgenerateinfill_hpp_
