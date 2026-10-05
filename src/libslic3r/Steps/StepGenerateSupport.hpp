///|/ Copyright (c) SuperSlicer 2026 Durand R?mi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef steps_stepgeneratesupport_hpp_
#define steps_stepgeneratesupport_hpp_

#include <string>

#include "libslic3r/Api/plugin/c/slic3r_plugin_types.h"

namespace Slic3r {
class Orchestrator;
class Print;

namespace Steps::StepGenerateSupport {

void clear(Print &print);
void prepare(Print &print);
bool validate_pre(const Print &print, std::string *error = nullptr);
bool validate_post(const Print &print, std::string *error = nullptr);
// Support generation reads the demand stored in the layer islands.
void run_step(Orchestrator &orchestrator, Print &print);

} // namespace Steps::StepGenerateSupport
} // namespace Slic3r

#endif // steps_stepgeneratesupport_hpp_
