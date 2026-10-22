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

// Declare export-stage selectors as printer settings that replay only the requested export.
void register_selector_traits(Orchestrator &orchestrator);

/*
STEP_EXPORT runner
==================

STEP_EXPORT publishes the G-code reference that STEP_GCODE prepared. It runs
only when a caller deposited an export request in the Print
(Print::request_export): the request carries the configuration copy, the
destination and the thumbnail renderer of this export alone.

The step reads the reference and never changes it. It does not replay any
earlier step: when the reference is not ready (never produced, invalidated since
the request, or regenerated meanwhile) the export fails and writes nothing.
*/

// Move the pending request out of the Print, then verify the reference and export it with the exporter that the
// request configuration selects. The published path is stored in the Print (Print::set_export_result_path) as soon
// as the exporter returns it. Throws RuntimeError when no request was deposited, when the destination is empty
// or when the reference is not a ready regular file; exporter failures and cancellation propagate and leave the
// request consumed. Locking: each Print access takes state_mutex() on its own; the caller must not hold it.
void run_step(Orchestrator &orchestrator, Print &print);

} // namespace Steps::StepExport
} // namespace Slic3r

#endif // steps_stepexport_hpp_
