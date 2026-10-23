///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_host_ExportInputs_hpp_
#define slic3r_Api_host_ExportInputs_hpp_

#include <functional>
#include <iosfwd>
#include <vector>

#include "libslic3r/Config/ConfigDef.hpp"

namespace Slic3r {

/*
Inputs that an exporter asks for
================================

During its setup, an exporter can ask the requester of an export for values (a
destination file, a choice, a switch...). The exporter describes each value with
a setting definition. The host converts the definitions into an ExportInputForm
and hands it to the requester through an ExportInputsCallback; the requester
(the command line, or a dialog of the graphical interface) shows one field per
definition, fills `values` with the answers and returns whether it validated.

The definitions are kept in this form only: they are never registered in the
global catalogue of settings, so the keys they name exist for the duration of
one export. A requester that needs a ConfigDef (a graphical form built on
the settings widgets) builds it from `definitions`.
*/
struct ExportInputForm
{
    // One definition per requested setting, in the order the exporter gave them.
    std::vector<ConfigOptionDef> definitions;
    // One option per definition, keyed by opt_key and of the type of the definition. On entry it holds the proposed
    // value (the one the export already had for the key when it is not empty, else the default of the definition); the
    // requester replaces it with the answer.
    DynamicConfig                values;
};

// Ask the requester for the values of a form. Return true when it validated (the answers are in the form), false when it
// cancelled; a CanceledException thrown by the requester is a cancellation too. Any other exception means that the requester
// could not ask; its message is reported as the cause of the failure.
using ExportInputsCallback = std::function<bool(ExportInputForm &)>;

// Build the requester of the command line: for each definition it writes the label, the tooltip and the proposed value
// to `out`, and reads the answer as one line of `in`. An empty line keeps the proposed value; an answer that the type of
// the setting refuses is reported and asked again; a closed input throws a RuntimeError that names the setting. The
// file filter of the path types is not shown. The streams must outlive the returned callback.
ExportInputsCallback make_stream_export_inputs(std::istream &in, std::ostream &out);

} // namespace Slic3r

#endif // slic3r_Api_host_ExportInputs_hpp_
