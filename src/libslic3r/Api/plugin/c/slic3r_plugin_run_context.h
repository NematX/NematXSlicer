///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_plugin_run_context_h_
#define slic3r_plugin_run_context_h_

#define SLIC3R_PLUGIN_API_PLUGIN_RUN_CONTEXT_MAJOR 2u
#define SLIC3R_PLUGIN_API_PLUGIN_RUN_CONTEXT_MINOR 1u

#include <stddef.h>

#include "slic3r_slicing_step.h"
#include "slic3r_utils.h"

/* This header supplies call data for plugin setup, preparation or execution.
   It also defines host callbacks for cancellation, diagnostics and progress.
   Use is_cancelled(), report_warning(), report_error() or report_progress() with host_context.
   The context and step data are borrowed for the call. Do not retain their addresses.
   plugin_storage is persistent allocation storage shared by the plugin's calls.
   The optional progress_context is separate: PluginBase uses it for C++ progress counters. */

#ifdef __cplusplus
extern "C" {
#endif

/* ========================= RUN CONTEXT ========================= */

typedef struct plugin_host_context plugin_host_context;
typedef struct print_handle print_handle;

/*
Return non-zero if the current Print was cancelled or the host requested this
plugin execution to stop for another reason.
Plugins should check this regularly in long loops and return as soon as
possible when cancellation is requested.
*/
typedef int (*plugin_is_cancelled_fn)(plugin_host_context *host_context);

/*
Report a message to the host.
Warnings are non-blocking. Errors are fatal for the current slicing operation:
the host will request cancellation for the slicing operation.
*/
typedef void (*plugin_report_fn)(plugin_host_context *host_context, const char *message);

/*
Report progress inside the currently running step.
Finite progress values are clamped by the host to the [0.0, 1.0] range;
non-finite values (NaN, +/-infinity) are ignored. message is optional
UTF-8 text and may be NULL to let the host use a default step/plugin message.
*/
typedef void (*plugin_report_progress_fn)(plugin_host_context *host_context, double progress, const char *message);

/*
Context passed to setup(), setup_run() and run().
step identifies the slicing stage. data points to that stage's call data,
documented in Api/plugin/c/steps/slic3r_step_*.h. Use its plugin_ctx_as_* helper
to access data with the correct type.
The context and data are valid for the call only. Progress counters belong
to the execution. Allocations in plugin_storage keep their separate lifetime.
*/
typedef struct plugin_run_context {
    slicing_step_t step;
    /*
    Print that owns this execution, or NULL when the host runs the step
    outside a Print. The pointer is borrowed and valid only during the call.
    */
    const print_handle *print;
    /*
    Persistent plugin storage, shared by parallel calls of the same plugin.
    Allocations, membership/count queries and individual releases are protected
    by a host mutex. This does not protect simultaneous edits of the same object
    or reads of a handle that another callback frees. Do not call storage_clear()
    during setup_run() or run(): it would destroy other callbacks' allocations.
    Use slic3r_parallel_for_storage() for temporary storage isolated per task.
    */
    storage_handle *plugin_storage;
    void *data;

    plugin_host_context *host_context;
    plugin_is_cancelled_fn is_cancelled;
    plugin_report_fn report_warning;
    plugin_report_fn report_error;
    plugin_report_progress_fn report_progress;
    /* Opaque host-owned counters used by C++ PluginBase during execution.
       Borrowed until the execution ends. Do not free or cast it in C plugins. NULL selects the plugin's own counters. */
    void *progress_context;
} plugin_run_context;

#ifdef __cplusplus
}
#endif

#endif // slic3r_plugin_run_context_h_
