///|/ Copyright (c) SuperSlicer 2026 Durand R?mi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef steps_stepextrusionordering_hpp_
#define steps_stepextrusionordering_hpp_

#include <string>

#include "libslic3r/Api/plugin/c/slic3r_plugin_types.h"

/* Extrusion ordering exposes its lifecycle and the category of selectors belonging to this step. */
namespace Slic3r {
class Orchestrator;
class Print;

namespace Steps::StepExtrusionOrdering {

// Declare host selector traits for extrusion ordering on the supplied host.
void register_selector_traits(Orchestrator &orchestrator);

void clear(Print &print);
void prepare(Print &print);
bool validate_pre(const Print &print, std::string *error = nullptr);
bool validate_post(const Print &print, std::string *error = nullptr);
void run_step(Orchestrator &orchestrator, Print &print);

} // namespace Steps::StepExtrusionOrdering
} // namespace Slic3r

#endif // steps_stepextrusionordering_hpp_
