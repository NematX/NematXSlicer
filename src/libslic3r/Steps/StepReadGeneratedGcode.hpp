///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef steps_stepreadgeneratedgcode_hpp_
#define steps_stepreadgeneratedgcode_hpp_

#include <string_view>

namespace Slic3r {

struct GCodeProcessorResult;
class Orchestrator;
class Print;
class PrintConfig;
struct PrintStatistics;

namespace Steps::StepReadGeneratedGcode {

// Declare host selector traits for G-code reader services on the supplied host.
void register_selector_traits(Orchestrator &orchestrator);

/*
Generated G-code reading and print statistics
==============================

This host-only phase follows STEP_GCODE but is not a selectable slicing step.
The writer has already completed its artifact and stored the path of a readable
text representation in Print. This phase streams that text through the Reader
selected by the firmware and builds the legacy GCodeProcessorResult expected
by GCodeViewer.
Measured role/tool volumes are converted to print totals and the wipe-tower
share, the support flag and the availability flag are set too. Every caller that wants
statistics (the GUI, the CLI) asks STEP_GCODE for a viewer result; exports without a
requested viewer result do not trigger reading in the pipeline.

The caller decides whether a Reader failure is fatal. run() reports failures as
exceptions and never removes or modifies the producer-owned source file.

At host startup, register_selector_traits(orchestrator) declares the reader's
settings metadata. StepSelectorTraits.hpp defines that metadata; the reading
entry points below supply the requested preview or print statistics.
*/
// Build a viewer result from the published source using the specified reader; first marks the print statistics unavailable, so a failed reread leaves them so.
void run(Orchestrator &orchestrator,
         Print &print,
         std::string_view reader_plugin_id,
         GCodeProcessorResult &result);

// Set measured time/material totals, tower shares and the support flag, then mark the statistics available; other statistics are preserved.
void fill_print_statistics(const GCodeProcessorResult &result, const PrintConfig &config,
                           PrintStatistics &stats);

} // namespace Steps::StepReadGeneratedGcode
} // namespace Slic3r

#endif // steps_stepreadgeneratedgcode_hpp_
