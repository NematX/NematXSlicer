///|/ Copyright (c) SuperSlicer 2026 Durand R?mi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef steps_stepprepareforperiemters_hpp_
#define steps_stepprepareforperiemters_hpp_

#include <string>

#include "libslic3r/Api/plugin/c/slic3r_plugin_types.h"

/* This interface lets StepPipeline run plugins before perimeter generation.
   Register the settings metadata defined by StepSelectorTraits.hpp with register_selector_traits()
   at host startup. For a print, use prepare(), validate_pre(), run_step() and validate_post();
   clear() removes object and print auxiliary layers used for skirts and brims. */
namespace Slic3r {
class Orchestrator;
class Print;

namespace Steps::StepPrepareForPeriemters {

// Declare host selector traits for perimeter preparation on the supplied host.
void register_selector_traits(Orchestrator &orchestrator);

void clear(Print &print);
void prepare(Print &print);
bool validate_pre(const Print &print, std::string &error);
bool validate_post(const Print &print, std::string &error);
void run_step(Orchestrator &orchestrator, Print &print);

} // namespace Steps::StepPrepareForPeriemters
} // namespace Slic3r

#endif // steps_stepprepareforperiemters_hpp_
