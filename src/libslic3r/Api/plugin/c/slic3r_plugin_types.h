///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
/* This header defines how the application calls plugin objects and reads their results.
   The host is the slicer application. A C ABI is the binary calling interface
   used to invoke plugin functions independently of their implementation language.

   A plugin_instance pairs borrowed object data with plugin_vtable, a table of
   function pointers. Fill struct_size with the table's byte size, provide the
   required functions, then register the instance through the manager API.
   The object and its code must outlive all application calls; optional functions
   are available only when their complete fields fit the reported size.

   The application calls initialize, then setup, setup_run and run for jobs.
   Initialization and clear (cleanup) return diagnostic numbers. Use
   orchestrator_set_error_message() to store an explanation and obtain its number.
   The function-type comments describe result codes and argument lifetimes. */
#ifndef slic3r_plugin_types_h_
#define slic3r_plugin_types_h_

#define SLIC3R_PLUGIN_API_PLUGIN_TYPES_MAJOR 4u
#define SLIC3R_PLUGIN_API_PLUGIN_TYPES_MINOR 1u

#include <stddef.h>
#include <stdint.h>

#include "slic3r_config_types.h"
#include "slic3r_plugin_run_context.h"
#include "slic3r_slicing_step.h"
#include "slic3r_utils.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================= HANDLES ========================= */

typedef struct orchestrator_handle orchestrator_handle;

/* ========================= PLUGIN CALLBACKS ========================= */

/* Borrow the plugin context and application storage for initialization. Return 0
   on success, PLUGIN_ERROR_GENERAL or an orchestrator_set_error_message() id on
   failure. No exception may escape; the application disables failed instances
   and preserves a rejection it already recorded instead of duplicating it. */
typedef int32_t (*plugin_initialize_fn)(void *plugin_ctx, storage_handle *storage);

/* This plugin_run_context is destroyed after each of these function return, so don't keep it, copy the data you want instead. */
typedef void (*plugin_setup_fn)(void *plugin_ctx, const plugin_run_context *run_ctx, uint32_t run_count);
typedef void (*plugin_setup_run_fn)(void *plugin_ctx, const plugin_run_context *run_ctx);
typedef void (*plugin_run_fn)(void *plugin_ctx, const plugin_run_context *run_ctx);
/*
Diagnostic identifier returned by plugin callbacks.

0 means success. 1 is the host-defined generic failure without diagnostic.
2..PLUGIN_ERROR_RESERVED_LAST are reserved for future host-defined codes.
orchestrator_set_error_message() allocates ids from PLUGIN_ERROR_FIRST_DYNAMIC
upward: the host keeps the registered UTF-8 text and resolves it when a callback
returns the id. Ids are process-local, must not be serialized and stay valid for
the lifetime of the running application.
*/
typedef enum plugin_error_id {
    PLUGIN_ERROR_OK = 0,
    PLUGIN_ERROR_GENERAL = 1
} plugin_error_id;

#define PLUGIN_ERROR_RESERVED_LAST 127
#define PLUGIN_ERROR_FIRST_DYNAMIC 128

/*
Inputs remain host-owned; only PLUGIN_ERROR_OK means this Print's private state
was released. A positive result is a diagnostic id resolved through the host
error registry; an unknown or negative id is treated as PLUGIN_ERROR_GENERAL.
*/
typedef int32_t (*plugin_clear_fn)(void *plugin_ctx,
                                   const print_handle *print,
                                   storage_handle *plugin_storage);

/*
One configuration option read or defined by a plugin.

type is mandatory and must not be RAW_CO_NONE. It lets the host verify that the
plugin and the option owner agree on the value representation.

container_type and option_preset_type are optional filters. Use *_NONE when the
plugin only cares about the value type and key, for example when a setting may
legitimately live in several preset buckets.
*/
typedef struct raw_used_config_key {
    const char *key;
    raw_config_option_type type;
    raw_container_type container_type;
    raw_option_preset_type option_preset_type;
} raw_used_config_key;

/*
Return the configuration options read or defined by this plugin.

This uses the usual C double-call pattern:
- call with keys == NULL to get the number of entries to allocate;
- call again with an array of that size to receive borrowed key pointers and
  value expectations owned by the plugin.

The host uses this list to validate plugin contracts and to enable/disable GUI
fields when several plugins are available for an exclusive step and a project
selects one of them. Every key returned by defined_config_keys must have a
matching typed entry in this list, even when the plugin only defines that option
for its GUI or for another plugin.
*/
typedef int32_t (*plugin_used_config_keys_fn)(void *plugin_ctx, raw_used_config_key *keys);

/*
Return the configuration option keys defined by this plugin.

This is a lightweight declaration used for early validation before a plugin is
enabled from the GUI. The authoritative check still happens when the plugin
calls orchestrator_create_option_def(), because only that call contains the full
definition to compare.
Every key returned here must also appear in plugin_used_config_keys_fn. The host
uses that typed declaration to describe settings before an inactive plugin has
called initialize().

Use the same double-call convention as plugin_used_config_keys_fn.
*/
typedef int32_t (*plugin_defined_config_keys_fn)(void *plugin_ctx, const char **keys);

/*
Each identifier represents one public C header. A package reports a dense
version table indexed by this enum before it is allowed to register plugins.
Existing values are append-only: adding a header appends an id; it never
renumbers an already published header identity.

Entry zero is always PLUGIN_TYPES, the contract defining plugin_vtable. Every
package must report a nonzero compatible major for this entry. Other entries
may be 0.0 when unused. Known requirements need the same major and a minor no
newer than the host; unknown requirements are rejected unless they are 0.0.
The table length alone does not determine compatibility.
*/
typedef enum slic3r_plugin_api_id {
    SLIC3R_PLUGIN_API_PLUGIN_TYPES = 0,
    SLIC3R_PLUGIN_API_BRIDGE_DETECTOR = 1,
    SLIC3R_PLUGIN_API_CLIPPER = 2,
    SLIC3R_PLUGIN_API_CONFIG = 3,
    SLIC3R_PLUGIN_API_CONFIG_DEF = 4,
    SLIC3R_PLUGIN_API_CONFIG_OPTION = 5,
    SLIC3R_PLUGIN_API_CONFIG_OPTION_TYPE = 6,
    SLIC3R_PLUGIN_API_CONFIG_TYPES = 7,
    SLIC3R_PLUGIN_API_DATA_TREE = 8,
    SLIC3R_PLUGIN_API_DEF = 9,
    SLIC3R_PLUGIN_API_EXTRUSION_ENTITY = 10,
    SLIC3R_PLUGIN_API_EXTRUSION_POLYLINE = 11,
    SLIC3R_PLUGIN_API_EXTRUSION_PROPERTY = 12,
    SLIC3R_PLUGIN_API_EXTRUSIONS = 13,
    SLIC3R_PLUGIN_API_GCODE_FIRMWARE = 14,
    SLIC3R_PLUGIN_API_GCODE_SCRIPT = 15,
    SLIC3R_PLUGIN_API_GEOMETRY = 16,
    SLIC3R_PLUGIN_API_ORCHESTRATOR = 17,
    SLIC3R_PLUGIN_API_PLUGIN_RUN_CONTEXT = 18,
    SLIC3R_PLUGIN_API_PLUGIN = 19,
    SLIC3R_PLUGIN_API_PRINTING_PLAN = 20,
    SLIC3R_PLUGIN_API_SLICING_STEP = 21,
    SLIC3R_PLUGIN_API_UTILS = 22,
    SLIC3R_PLUGIN_API_VOLUME = 23,
    SLIC3R_PLUGIN_API_STEP_BRIDGE_DETECTOR = 24,
    SLIC3R_PLUGIN_API_STEP_COMMON = 25,
    SLIC3R_PLUGIN_API_STEP_EXTRUSION_EDIT = 26,
    SLIC3R_PLUGIN_API_STEP_EXTRUSION_SIMPLIFICATION = 27,
    SLIC3R_PLUGIN_API_STEP_GCODE_FIRMWARE = 28,
    SLIC3R_PLUGIN_API_STEP_GCODE = 29,
    SLIC3R_PLUGIN_API_STEP_INFILL_GROUP = 30,
    SLIC3R_PLUGIN_API_STEP_INFILL = 31,
    SLIC3R_PLUGIN_API_STEP_LAYER_EXTRUSION_EDIT = 32,
    SLIC3R_PLUGIN_API_STEP_LAYER_HEIGHT = 33,
    SLIC3R_PLUGIN_API_STEP_LAYER_STICHING = 34,
    SLIC3R_PLUGIN_API_STEP_ORDERING = 35,
    SLIC3R_PLUGIN_API_STEP_PERIMETER = 36,
    SLIC3R_PLUGIN_API_STEP_POST_INFILL = 37,
    SLIC3R_PLUGIN_API_STEP_POST_PERIMETER = 38,
    SLIC3R_PLUGIN_API_STEP_POST_SLICING = 39,
    SLIC3R_PLUGIN_API_STEP_FINALIZE_SLICING = 40,
    SLIC3R_PLUGIN_API_STEP_PRE_INFILL = 41,
    SLIC3R_PLUGIN_API_STEP_PRE_PERIMETER = 42,
    SLIC3R_PLUGIN_API_STEP_SEAM_PLACER = 43,
    SLIC3R_PLUGIN_API_STEP_SKIRT_BRIM = 44,
    SLIC3R_PLUGIN_API_STEP_SLICING = 45,
    SLIC3R_PLUGIN_API_STEP_SUPPORT = 46,
    SLIC3R_PLUGIN_API_STEP_SUPPORT_DEMAND = 47,
    SLIC3R_PLUGIN_API_STEP_SUPPORT_SPOT = 48,
    SLIC3R_PLUGIN_API_STEP_SURFACE_GENERATION = 49,
    SLIC3R_PLUGIN_API_STEP_WIPETOWER = 50,
    SLIC3R_PLUGIN_API_GCODE_THUMBNAIL = 51,
    SLIC3R_PLUGIN_API_STEP_GCODE_THUMBNAIL = 52,
    SLIC3R_PLUGIN_API_GCODE_READER = 53,
    SLIC3R_PLUGIN_API_STEP_GCODE_READER = 54,
    SLIC3R_PLUGIN_API_MESH_QUERIES = 55,
    SLIC3R_PLUGIN_API_GEOMETRY_TYPES = 56,
    SLIC3R_PLUGIN_API_STEP_WIPE_TOWER_WIPE = 57,
    SLIC3R_PLUGIN_API_STEP_SUPPORT_TOOLPATH = 58,
    SLIC3R_PLUGIN_API_STEP_GCODE_EXPORT = 59,
    /* SLIC3R_PLUGIN_API_COUNT is always last and has its value auto-filled. */
    SLIC3R_PLUGIN_API_COUNT
} slic3r_plugin_api_id;

typedef struct slic3r_major_minor_version {
    uint16_t major;
    uint16_t minor;
} slic3r_major_minor_version;

/* ========================= PLUGIN VTABLE ========================= */

typedef struct plugin_vtable {

    /* Required prefix ends after run; every callback in that prefix must be nonnull.
       The host reads this size before any callback.
       Optional members are read only when offsetof(member) + sizeof(member) fits.
       Unknown trailing bytes are ignored. Packages with an incompatible major are
       rejected before registration; their vtables cannot be read as this layout. */
    uint32_t struct_size;

    /*
    Stable machine-readable id.

    This value is used in config files, dependencies and plugin activation
    lists. It must not be translated and should not change between releases
    unless the plugin is intentionally replaced by a different plugin.
    */
    const char* (*get_id)(void *plugin_ctx);

    /*
    Short user-facing name.

    This is displayed in combo boxes and plugin lists. It may contain spaces and
    should be clear to non-developers. It is only a label: the host still stores
    get_id() as the serialized value.
    */
    const char* (*get_name)(void *plugin_ctx);

    /*
    Optional longer user-facing description.

    Return an empty string if there is no useful description yet. The pointer is
    borrowed from the plugin and only read during registration.
    */
    const char* (*get_description)(void *plugin_ctx);

    /*
    Machine-readable exclusive group id.

    Plugins in the same exclusive group are alternatives: the project stores a
    selector setting and the host runs only the selected active plugin from the
    group. Return the plugin id when the plugin has no known alternatives yet;
    NULL also requests this singleton fallback for older plugins. Return an
    empty string only for an addressable service that deliberately has no
    selector and may remain active alongside every other provider of its step.

    Some host-defined "unique" steps force all active plugins for that step into
    one exclusive group even if this callback returns empty. This lets old-style
    step replacement plugins use the same selection machinery.
    */
    const char* (*get_exclusive_group)(void *plugin_ctx);

    slicing_step_t (*get_step)(void *plugin_ctx);

    const_strings_t (*get_dependencies)(void *plugin_ctx);

    /* Execution priority must be equal among alternatives of one effective group. */
    int32_t (*get_priority)(void *plugin_ctx);

    plugin_used_config_keys_fn used_config_keys;

    plugin_defined_config_keys_fn defined_config_keys;

    /* Initialize settings before slicing. Return success or a diagnostic id;
       the application disables a failed instance without duplicating a rejection
       already recorded while this callback was running. */
    plugin_initialize_fn initialize;

    /*
    Called once before any setup_run()/run() call for this step/plugin pair.
    run_count is the number of run contexts that will be prepared for this
    plugin. Use it to reset plugin-side shared state such as progress helpers.
    */
    plugin_setup_fn setup;

    /*
    Called once for each run context before any run() starts. The host may call
    setup_run() in parallel, but it guarantees that all setup_run() calls finish
    before the first run() starts. Use it to estimate per-run work.
    */
    plugin_setup_run_fn setup_run;

    /*
    * Called once per object to do the step this plugin is made for.
    */
    plugin_run_fn run;

    /*
    Optional notification that the host invalidated this plugin's state for one
    Print. The callback must release only plugin-owned state for that Print.
    Missing bytes or NULL mean no cleanup notification. A partially covered
    pointer is absent and must not be read.
    */
    plugin_clear_fn clear;

    /*
    Exclusive-group preference, independent from execution priority get_priority().
    Among active alternatives, the smallest value is the default; equal values
    use the smallest provider id, independent of registration order. Execution
    ordering does not use this preference. The callback borrows plugin_ctx and
    returns a stable int32_t during registration, with no ownership transfer.
    This optional tail member defaults to 100 when absent or NULL. The host reads
    it only when offsetof(plugin_vtable, get_exclusive_group_priority) plus its
    full member size fits struct_size; an incomplete pointer is never read.
    */
    int32_t (*get_exclusive_group_priority)(void *plugin_ctx);

} plugin_vtable;

/* ========================= PLUGIN INSTANCE ========================= */

typedef struct plugin_instance {
    void *ctx;
    const plugin_vtable *vt;
} plugin_instance;

#ifdef __cplusplus
}
#endif

#endif // slic3r_plugin_types_h_
