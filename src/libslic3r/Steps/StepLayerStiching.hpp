///|/ Copyright (c) SuperSlicer 2026 Durand R?mi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef steps_steplayerstiching_hpp_
#define steps_steplayerstiching_hpp_

#include <string>

#include "libslic3r/Api/plugin/c/slic3r_plugin_types.h"

/* This interface supplies layer-stitching hooks to StepPipeline; the execution hooks do no work.
   At host startup, call register_selector_traits() to register the settings metadata defined by
   StepSelectorTraits.hpp. The pipeline can then call prepare(), validate_pre(), run_step() and
   validate_post(); clear() is also available when invalidating a print. */
namespace Slic3r {
class Orchestrator;
class Print;

namespace Steps::StepLayerStiching {

// Declare host selector traits for layer stitching on the supplied host.
void register_selector_traits(Orchestrator &orchestrator);

void clear(Print &print);
void prepare(Print &print);
bool validate_pre(const Print &print, std::string *error = nullptr);
bool validate_post(const Print &print, std::string *error = nullptr);
void run_step(Orchestrator &orchestrator, Print &print);

} // namespace Steps::StepLayerStiching
} // namespace Slic3r

#endif // steps_steplayerstiching_hpp_
