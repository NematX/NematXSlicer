///|/ Copyright (c) SuperSlicer 2026 Durand R?mi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef steps_stepextrusionordering_hpp_
#define steps_stepextrusionordering_hpp_

#include <string>

#include "libslic3r/Api/plugin/c/slic3r_plugin_types.h"

/* This interface lets StepPipeline order extrusion paths and choose where closed loops start.
   Register the settings metadata defined by StepSelectorTraits.hpp with register_selector_traits()
   at host startup. For a print, use prepare(), validate_pre(), run_step() and validate_post().
   Seam settings apply per region; changing them requires ordering to run again. */
namespace Slic3r {
class Orchestrator;
class Print;

namespace Steps::StepExtrusionOrdering {

// Register settings metadata for extrusion ordering and per-region seam placement with the supplied plugin manager.
void register_selector_traits(Orchestrator &orchestrator);

void clear(Print &print);
void prepare(Print &print);
bool validate_pre(const Print &print, std::string *error = nullptr);
bool validate_post(const Print &print, std::string *error = nullptr);
void run_step(Orchestrator &orchestrator, Print &print);

} // namespace Steps::StepExtrusionOrdering
} // namespace Slic3r

#endif // steps_stepextrusionordering_hpp_
