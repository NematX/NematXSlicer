///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_step_gcode_export_h_
#define slic3r_step_gcode_export_h_

/*
An exporter turns the generated G-code reference into the requested output.
QUERY checks compatibility and describes the available action; EXPORT performs
the output operation. The host supplies the configuration for that export, which
can differ from the configuration currently attached to the source Print.

Use the operation-specific payload below only during its synchronous callback.
Copy any returned text that must outlive the call. The structure contracts keep
the detailed rules for optional fields, result publication and input requests.

STEP_EXPORT separates a synchronous compatibility/action QUERY from publication
by the provider. Size-prefixed responses allow optional appended information.
*/

#define SLIC3R_PLUGIN_API_STEP_GCODE_EXPORT_MAJOR 4u
#define SLIC3R_PLUGIN_API_STEP_GCODE_EXPORT_MINOR 2u

#include <stddef.h>
#include <stdint.h>

#include "slic3r_step_common.h"
#include "../slic3r_config_def.h"
#include "../slic3r_gcode_thumbnail.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct raw_gcode_export_entry {
    const char *key;
    const char *value;
} raw_gcode_export_entry;

typedef struct raw_gcode_export_list {
    const raw_gcode_export_entry *items;
    size_t count;
} raw_gcode_export_list;

/* Measured material of one tool (logical filament), in the units below. */
typedef struct raw_gcode_export_filament_usage {
    /* Zero-based tool index; the position of this tool in the per-tool lines of the text statistics. */
    uint32_t tool;
    /* Length of filament pushed through the extruder, in millimeters. */
    double length_mm;
    /* Extruded volume, in cubic millimeters. */
    double volume_mm3;
    /* Weight from the filament density of the configuration, in grams. */
    double weight_g;
} raw_gcode_export_filament_usage;

/* Borrowed array of per-tool material; items is null only when count is zero. */
typedef struct raw_gcode_export_filament_usage_list {
    const raw_gcode_export_filament_usage *items;
    size_t count;
} raw_gcode_export_filament_usage_list;

/* Read-only export metadata borrowed until the synchronous EXPORT callback returns. */
typedef struct raw_gcode_export_metadata {
    uint32_t struct_size;
    raw_gcode_export_list configuration;
    raw_gcode_export_list extrusion_widths;
    /* Measured print statistics, empty when they are unavailable. */
    raw_gcode_export_list statistics;
    /*
    Since 4.2, the numeric statistics. They are appended as one group: a structure whose
    struct_size stops before total_weight_g carries none of them. Plan facts (layer_count,
    max_print_z) are valid once STEP_GCODE has finished. Measured values (the two times and all
    material values) are valid if and only if the statistics list above is not empty, else
    they are zero. Plan facts are zero until STEP_GCODE has published them, also for an EXPORT run
    directly on a reference that STEP_GCODE did not produce.
    */
    /* Estimated print time in seconds; nominal path time without acceleration. Zero without measured statistics. */
    double estimated_time_seconds;
    /* Seconds that the Reader attributed to its layer 0, moves before the first layer included. Zero without a measure. */
    double first_layer_time_seconds;
    /* Number of layers of the printing plan, auxiliary layers included; zero before STEP_GCODE has finished. */
    uint32_t layer_count;
    /* Highest layer Z of the printing plan in millimeters, without the Z offset; zero before STEP_GCODE has finished. */
    double max_print_z;
    /* Material of every tool of the configuration copy of this export, in tool order, host-owned; empty without measured statistics. */
    raw_gcode_export_filament_usage_list filaments;
    /* Sums of the per-tool values above: millimeters, cubic millimeters and grams. Zero without measured statistics. */
    double total_length_mm;
    double total_volume_mm3;
    double total_weight_g;
} raw_gcode_export_metadata;

/* Choose whether a run checks exporter compatibility or exports the G-code reference. */
typedef enum raw_gcode_export_operation {
    RAW_GCODE_EXPORT_OPERATION_QUERY = 1,
    RAW_GCODE_EXPORT_OPERATION_EXPORT
} raw_gcode_export_operation;

/* QUERY returns compatibility; operational failures use report_error or an exception. */
typedef enum raw_gcode_export_query_status {
    RAW_GCODE_EXPORT_QUERY_UNSET = 0,
    RAW_GCODE_EXPORT_QUERY_COMPATIBLE,
    RAW_GCODE_EXPORT_QUERY_INCOMPATIBLE
} raw_gcode_export_query_status;

/* Return one provider-owned QUERY result; the host copies its strings during the callback. */
typedef struct raw_gcode_export_query_result {
    uint32_t struct_size;
    raw_gcode_export_query_status status;
    const char *extension;
    const char *message;
    /* Since 4.1: optional UTF-8 button text without ellipsis, translated by the GUI in the provider's domain. */
    const char *action_label;
    /* Since 4.1: 0 or 1, whether the removable-media export action is suitable. */
    int supports_removable_media;
} raw_gcode_export_query_result;

/* Deliver a synchronous borrowed QUERY result to the host. */
typedef void (*gcode_exporter_emit_query_result_fn)(
    void *callback_context,
    const raw_gcode_export_query_result *result);

/*
Deliver the outcome of one EXPORT to the host. Both strings are optional (null or
empty) and are copied during the call. Return 1 when the host accepted the
outcome, 0 when it refused it (a second report).
*/
typedef int32_t (*gcode_exporter_report_result_fn)(
    void *callback_context,
    const char *local_path,
    const char *label);

/*
Ask the requester for the values of the given settings, blocking until the answer.
Return 1 when the answers are in config, 0 when the exporter has to stop.
*/
typedef int32_t (*gcode_exporter_request_inputs_fn)(
    void *callback_context,
    const raw_config_option_def *definitions,
    size_t count);

/*
Synchronous STEP_EXPORT payload.

An exporter is a plugin of STEP_EXPORT. The step runs only when an export is
requested, on the G-code reference that STEP_GCODE finished. The host owns the
payload, handles, strings, callbacks, callback contexts and optional services for
one plugin run. QUERY requires print, config, firmware_plugin_id, query_callback_context
and emit_query_result; EXPORT requires config, print, reference_path,
thumbnail_renderer, metadata, report_export_result and request_export_inputs. Fields of the other
operation are null except print and config, shared by both operations. In QUERY,
print is supplied only in this payload; the exporter's own execution has no generic
Print, so it ignores Print cancellation and does not register cleanup on that Print.
A service acquired during QUERY, such as the thumbnail encoder, receives the Print
and may refuse it when it is cancelled.
The configuration, Print, firmware identity and paths are
borrowed read-only inputs. New fields are appended after the existing ones, in
minor versions.

config is the full print configuration (printer, print and filament settings,
including the exporter selector) that the host copied for this export, in both
operations, plus any request-only settings already supplied for this export. A
QUERY before the request has no output_path or output_path_suggestion; compatibility
must not depend on them. An exporter reads its
settings from config, never from print: print is the object being exported and
its own configuration may already differ.

The destination of the export is the setting output_path of config, a string
that the requester sets for this export only. The exporter decides what it means
and publishes its output itself: a file, a stream or anything else. A file
exporter should write a complete file next to its destination and replace the
destination only then (the SDK helper publish_gcode_file() does: it renames an
existing destination to a backup, renames the new file in, and tries to restore the
backup if that fails), so that a destination that already exists stays as it was when
the work fails or is cancelled before the replacement. A cancellation after the
replacement does not undo it. The exporter that has no destination
(no output_path in config) may ignore it.

QUERY runs on the calling thread, which may be the GUI thread, and the host may
repeat it around one export: it must be quick and free of side effects, with no
rendering or writing. It may inspect only Print data stable during computation:
its applied configuration and selected plugins/services, never geometry or layers.
Its settings still come from config, not from the Print configuration. Acquiring
and releasing another plugin's service session to inspect a capability is allowed,
without rendering or writing. QUERY calls emit_query_result exactly once. The host checks
struct_size before reading any other member. The prefix through message is the
minimum size; larger structures are accepted and fields added after this prefix
are read only when they fit in the supplied size. Missing action_label defaults
to empty (the GUI uses its ordinary export label); missing supports_removable_media
defaults to 1. These fields describe only COMPATIBLE answers. COMPATIBLE may carry an
extension: it is null or empty for an exporter that writes no local file, else it
begins with a dot and contains neither slash nor backslash; message may be null.
INCOMPATIBLE requires a non-null message. The host copies returned strings during the
callback, rejects missing or repeated answers and contains callback exceptions.
An incompatible answer is a normal result; operational failures use report_error
or a contained C++ exception.

Paths and identifiers are null-terminated UTF-8 strings borrowed for the run.
The exporter may read reference_path but must not modify, rename or remove it.

The reference holds G-code only. A text exporter writes the file header, notes,
extrusion-width comments, a copy of the reference, the statistics, the filtered
configuration, and trailing thumbnails in the format's order. In EXPORT,
thumbnail_renderer is never null: it is the host bridge to the graphical renderer,
and a render request returns SKIPPED when no graphical renderer exists (command
line). Rendering is synchronous and sequential; the host holds no print state lock
while it waits for the graphical interface. A render cancelled with the Print, or
that fails, reports an error with its message and fails the export. In QUERY it is
null.

metadata is non-null in EXPORT and null in QUERY. Its size must cover the prefix
through extrusion_widths; larger structures are accepted. Each list has null items
only when count is zero. Entry strings are non-null, null-terminated UTF-8 owned by
the host for the callback; the plugin neither modifies nor frees them. A pair of
empty strings separates extrusion-width regions. Configuration values are filtered
before they are written to a file; plugins still receive the full configuration
through config and the configuration API, so this filter does not limit plugin
access to settings. The statistics list holds a label and a value formatted by the
host (for example "filament used [mm]" and "12.34, 0.00"), in the order in which a
text file lists them. It is empty when the host has no measured statistics for this
export, for example when the G-code was generated without reading it back; it is
never empty otherwise, because several entries are unconditional. An exporter
therefore tests statistics.count == 0 and writes no statistics line in that case. A
structure whose size stops before the statistics field means no statistics.

Since 4.2 the numeric statistics follow the statistics list (see the structure above). They
are the unrounded values from which the text above is formatted, computed once with the
configuration copy of this export: the same tools (those of its nozzle diameters, measured
tools beyond them are ignored) and its filament diameter and density. Units are seconds,
millimeters, cubic millimeters and grams. The time is the nominal path time of the Reader,
without acceleration; no silent-mode time exists. layer_count and max_print_z come from the
printing plan and do not need the Reader, but they are zero until STEP_GCODE has published them,
also for an EXPORT run directly on a reference that STEP_GCODE did not produce, even when the
Print has a plan. The time, material and
first-layer values are measured: they are valid exactly when the numeric statistics exist and the
statistics list is not empty. A structure whose size stops before total_weight_g carries no numeric statistics;
an exporter that needs them treats it as an older host. The filaments array and
everything it points to are owned by the host and borrowed like the lists above.

The exporter reports its outcome with report_export_result, at most once: a local
path when it wrote a file that the requester can read, and a label that the
requester may display. The host records the outcome when it is reported, so a
cancellation that arrives afterwards does not hide a file that was already
published. A second call is refused and returns 0.

The exporter checks is_cancelled during long work and stops when cancellation is
requested. I/O failure or cancellation fails the step; the host preserves the
reference.

Asking for inputs (since 3.1). In EXPORT only, during setup_run and at most once per
export, an exporter that needs values from the requester calls request_export_inputs
with an array of definitions (contiguous raw_config_option_def, count entries). The
call blocks until the requester has answered. The host copies what it keeps during
the call; the exporter keeps every pointed string valid until the call returns. A
definition describes one setting: its key, its type, its label, its default value and
its GUI type; the host never registers it in the global catalogue. The type must be a
single value that a form can edit (not a list, not RAW_CO_POINT3) and a key may appear
only once, or the host refuses the call. The path types
RAW_GUI_TYPE_SAVE_PATH and RAW_GUI_TYPE_OPEN_PATH use sidetext as a file filter in
the wxWidgets format (for example "G-code (*.gcode)|*.gcode"). The requester sees one
field per definition, filled with the value that config already holds for the key
when it has the same type and is not empty, else with the default value of the
definition. When the requester validates, the host copies every answer into config,
as volatile settings of this export only, and returns 1: the exporter reads them
from config, with the same pointer, from that moment on, in setup_run as well as in
run. The metadata of the export are those taken before the answers and do not
contain them. The call returns 0 when the requester cancelled, when a definition was
refused, when the requester gave an invalid answer, when no requester is available or
when it is called a second time. The host already knows the cause and reports it: the
exporter must stop by throwing an ordinary exception, never by cancelling itself, and
what it reports afterwards does not replace the cause. The host then also requests the
cancellation of the plugin, so that run is not executed even if the exporter ignores
the 0.
*/
typedef struct run_ctx_gcode_export {
    raw_gcode_export_operation operation;
    /* QUERY and EXPORT: full read-only configuration copy used for the settings. */
    const config_handle *config;
    const char *firmware_plugin_id;
    void *query_callback_context;
    gcode_exporter_emit_query_result_fn emit_query_result;
    const print_handle *print;
    const char *reference_path;
    const raw_gcode_thumbnail_renderer *thumbnail_renderer;
    const raw_gcode_export_metadata *metadata;
    void *result_callback_context;
    gcode_exporter_report_result_fn report_export_result;
    /* EXPORT, since 3.1: the callback that asks the requester for inputs. Null in QUERY. */
    void *input_callback_context;
    gcode_exporter_request_inputs_fn request_export_inputs;
} run_ctx_gcode_export;

/* Return the borrowed export payload only for a non-null STEP_EXPORT context with data. */
static inline const run_ctx_gcode_export *
plugin_ctx_as_gcode_export(const plugin_run_context *ctx)
{
    if (ctx == NULL || ctx->step != STEP_EXPORT || ctx->data == NULL)
        return NULL;
    return (const run_ctx_gcode_export *)ctx->data;
}

#ifdef __cplusplus
}
#endif

#endif /* slic3r_step_gcode_export_h_ */
