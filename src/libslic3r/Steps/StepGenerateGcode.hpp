///|/ Copyright (c) SuperSlicer 2026 Durand R?mi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef steps_stepgenerategcode_hpp_
#define steps_stepgenerategcode_hpp_

#include <string>

#include "libslic3r/Api/plugin/c/slic3r_plugin_types.h"

/* This interface lets StepPipeline produce a G-code file and optional thumbnails.
   Register the settings metadata defined by StepSelectorTraits.hpp with register_selector_traits()
   at host startup. For a print, use prepare(), validate_pre(), run_step() and validate_post();
   run_step() returns the output path. */
namespace Slic3r {
class Orchestrator;
class Print;

namespace Steps::StepGenerateGcode {

// Declare host selector traits for G-code generation, firmware and thumbnail services on the supplied host.
void register_selector_traits(Orchestrator &orchestrator);

void clear(Print &print);
void prepare(Print &print);
bool validate_pre(const Print &print, std::string *error = nullptr);
bool validate_post(const Print &print, std::string *error = nullptr);
std::string run_step(Orchestrator &orchestrator, Print &print, const std::string &path);

} // namespace Steps::StepGenerateGcode
} // namespace Slic3r

#endif // steps_stepgenerategcode_hpp_
