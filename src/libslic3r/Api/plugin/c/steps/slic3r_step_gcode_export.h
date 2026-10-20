///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_step_gcode_export_h_
#define slic3r_step_gcode_export_h_

#define SLIC3R_PLUGIN_API_STEP_GCODE_EXPORT_MAJOR 2u
#define SLIC3R_PLUGIN_API_STEP_GCODE_EXPORT_MINOR 2u

#include <stddef.h>
#include <stdint.h>

#include "slic3r_step_common.h"
#include "../slic3r_gcode_thumbnail.h"

#ifdef __cplusplus

/*
An exporter turns the generated G-code reference into the requested output.
QUERY checks compatibility and describes the available action; EXPORT performs
the output operation. The host supplies the configuration for that export, which
can differ from the configuration currently attached to the source Print.

Use the operation-specific payload below only during its synchronous callback.
Copy any returned text that must outlive the call. The structure contracts keep
the detailed rules for optional fields, result publication and input requests.
*/
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

/* Read-only export metadata borrowed until the synchronous EXPORT callback returns. */
typedef struct raw_gcode_export_metadata {
    uint32_t struct_size;
    raw_gcode_export_list configuration;
    raw_gcode_export_list extrusion_widths;
    /* ABI 2.2: measured print statistics, empty when they are unavailable. */
    raw_gcode_export_list statistics;
} raw_gcode_export_metadata;

/* Choose whether a run checks exporter compatibility or writes an exported file. */
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
} raw_gcode_export_query_result;

/* Deliver a synchronous borrowed QUERY result to the host. */
typedef void (*gcode_exporter_emit_query_result_fn)(
    void *callback_context,
    const raw_gcode_export_query_result *result);

/*
Synchronous GCODE_EXPORT payload.

The host owns the payload, handles, strings, callback, callback context and
optional services for one plugin run. QUERY requires config, firmware_plugin_id,
query_callback_context and emit_query_result; EXPORT requires config, print,
reference_path and output_path. Fields for the other operation are null. The
configuration, Print, firmware identity and paths are borrowed read-only inputs.

config is the full print configuration (printer, print and filament settings,
including the exporter selector) that the host copied for this export, in both
operations. An exporter reads its settings from config, never from print: print
is the object being exported and its own configuration may already differ.

QUERY runs on the calling thread, which may be the GUI thread, and the host may
repeat it before every export: it must be quick and free of side effects.
QUERY calls emit_query_result exactly once. The host checks struct_size before
reading any other member. The version 1.0 prefix through message is the minimum
size; larger structures are accepted and fields added after this prefix are read
only when they fit in the supplied size. COMPATIBLE requires a non-null
extension beginning with a dot and containing neither slash nor backslash;
message may be null. INCOMPATIBLE requires a non-null message. The host copies
both strings during the callback, rejects missing or repeated answers and
contains callback exceptions. An incompatible answer is a normal result;
operational failures use report_error or a contained C++ exception.

Paths and identifiers are null-terminated UTF-8 strings borrowed for the run.
The exporter may read reference_path but must not modify, rename or remove it.
output_path names a unique temporary path supplied by the host; the exporter
creates and closes the file but does not publish or remove it. The host publishes
the final destination only after successful execution and removes partial output
after failure.

The reference holds G-code only. The exporter writes the file header, notes,
extrusion-width comments, a copy of the reference, the statistics, the filtered
configuration, and trailing thumbnails in the format's order. In EXPORT, thumbnail_renderer is
never null: it is the host bridge to the graphical renderer, and a render request
returns SKIPPED when no graphical renderer exists (command line). Rendering is
synchronous and sequential; the host holds no print state lock while it waits for
the graphical interface. A render cancelled with the Print, or that fails,
reports an error with its message and fails the export. In QUERY it is null.

In ABI 2.1, metadata is non-null in EXPORT and null in QUERY. Its size must cover
the prefix through extrusion_widths; larger structures are accepted. Each list
has null items only when count is zero. Entry strings are non-null, null-terminated
UTF-8 owned by the host for the callback; the plugin neither modifies nor frees
them. A pair of empty strings separates extrusion-width regions. Configuration
values are filtered before they are written to the file; plugins still receive
the full configuration through config and the configuration API, so this filter
does not limit plugin access to settings. The ABI 2.0 promise that metadata is
always null is superseded: a 2.0 manifest is accepted and receives the same
non-null metadata in EXPORT.

ABI 2.2 adds the statistics list after extrusion_widths. Each entry is a label and a
value formatted by the host (for example "filament used [mm]" and "12.34, 0.00"), in the
order in which a text file lists them. The list is empty when the host has no measured
statistics for this export, for example when the G-code was generated without reading
it back; it is never empty otherwise, because several entries are unconditional. An
exporter therefore tests statistics.count == 0 and writes no statistics line in that case.
A structure whose size stops before the statistics field (ABI 2.1) means no statistics.

The exporter checks is_cancelled during long work and stops when cancellation is
requested. I/O failure or cancellation fails the export; the host preserves the
reference and any existing destination.
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
    const char *output_path;
    const raw_gcode_thumbnail_renderer *thumbnail_renderer;
    const raw_gcode_export_metadata *metadata;
} run_ctx_gcode_export;

/* Return the borrowed export payload only for a non-null GCODE_EXPORT context with data. */
static inline const run_ctx_gcode_export *
plugin_ctx_as_gcode_export(const plugin_run_context *ctx)
{
    if (ctx == NULL || ctx->step != GCODE_EXPORT || ctx->data == NULL)
        return NULL;
    return (const run_ctx_gcode_export *)ctx->data;
}

#ifdef __cplusplus
}
#endif

#endif /* slic3r_step_gcode_export_h_ */
