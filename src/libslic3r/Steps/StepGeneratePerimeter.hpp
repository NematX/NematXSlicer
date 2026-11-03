///|/ Copyright (c) SuperSlicer 2026 Durand R?mi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef steps_stepgenerateperimeter_hpp_
#define steps_stepgenerateperimeter_hpp_

#include <string>

#include "libslic3r/Api/plugin/c/slic3r_plugin_types.h"

/* This interface lets StepPipeline generate perimeter paths through the selected plugin.
   Register the settings metadata defined by StepSelectorTraits.hpp with register_selector_traits()
   at host startup. For a print, use prepare(), validate_pre(), run_step() and validate_post();
   clear() removes step output when needed. */
namespace Slic3r {
class Orchestrator;
class Print;

namespace Steps::StepGeneratePerimeter {

// Declare host selector traits for perimeter generation on the supplied host.
void register_selector_traits(Orchestrator &orchestrator);

void clear(Print &print);
void prepare(Print &print);
bool validate_pre(const Print &print, std::string *error = nullptr);
bool validate_post(const Print &print, std::string *error = nullptr);
void run_step(Orchestrator &orchestrator, Print &print);

} // namespace Steps::StepGeneratePerimeter
} // namespace Slic3r

#endif // steps_stepgenerateperimeter_hpp_
