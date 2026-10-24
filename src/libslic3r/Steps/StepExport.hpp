///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef steps_stepexport_hpp_
#define steps_stepexport_hpp_

namespace Slic3r {

class Orchestrator;
class Print;

namespace Steps::StepExport {

// Declare export-stage selectors as printer settings that do not invalidate slicing.
void register_selector_traits(Orchestrator &orchestrator);

/*
STEP_EXPORT runner
==================

STEP_EXPORT hands the G-code reference that STEP_GCODE prepared to the selected
exporter plugin, which publishes it (a file, a stream or anything else). It runs
only when a caller deposited an export request in the Print
(Print::request_export): the request carries the configuration copy, an
optional destination, optional proposed directory and filename base, optional ASCII
filename fallback, and thumbnail renderer of this export alone.

The step reads the reference and never changes it. It does not replay any
earlier step: when the reference is not ready (never produced, invalidated since
the request, or regenerated meanwhile) the export fails and writes nothing.
*/

// Consume one request, verify the ready G-code reference, form the request's file suggestion from an existing proposed directory and optional base, then run its selected exporter.
void run_step(Orchestrator &orchestrator, Print &print);

} // namespace Steps::StepExport
} // namespace Slic3r

#endif // steps_stepexport_hpp_
