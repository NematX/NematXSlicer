///|/ Copyright (c) SuperSlicer 2026 Durand R?mi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef steps_stepslicing_hpp_
#define steps_stepslicing_hpp_

#include <string>

#include "libslic3r/Api/plugin/c/slic3r_plugin_types.h"

/* This interface lets StepPipeline create sliced layers through the selected plugin.
   Register the settings metadata defined by StepSelectorTraits.hpp with register_selector_traits()
   at host startup. run_step() validates the profile, prepares layers, runs the plugin and checks
   its output; clear() removes generated layers. */
namespace Slic3r {
class Orchestrator;
class Print;

namespace Steps::StepSlicing {

// Declare host selector traits for slicing on the supplied host.
void register_selector_traits(Orchestrator &orchestrator);

void clear(Print &print);
void prepare(Print &print);
bool validate_pre(const Print &print, std::string &error);
bool validate_post(const Print &print, std::string &error);
void run_step(Orchestrator &orchestrator, Print &print);

} // namespace Steps::StepSlicing
} // namespace Slic3r

#endif // steps_stepslicing_hpp_
