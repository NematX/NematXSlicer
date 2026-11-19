///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
/* This header lets plugins register objects and resources, run jobs and report diagnostics.
   The host is the slicer application. An orchestrator_handle is a borrowed
   reference to its plugin manager; callers must not delete it.

   Introduce instances with orchestrator_register_plugin(), or use
   orchestrator_register_plugin_from_package() to identify their installed
   directory explicitly. orchestrator_is_plugin_registered() includes disabled
   instances; orchestrator_find_plugin() finds enabled ones for execution.
   Package-loading functions bracket resource registration and record load errors.

   A service is a directly callable plugin job. Select a registered instance,
   then orchestrator_execute_plugin() prepares and runs its per-call data.
   Step and property registration functions name processing stages and associated
   values; their function comments describe ownership and valid call times.

   Logging writes UTF-8 text at a severity, a category from fatal through trace.
   orchestrator_send_log() uses the application's configured logger rather than
   a plugin library's private logger. It does not create a package error.
   orchestrator_set_error_message() instead stores an explanation and returns
   a diagnostic id for initialization or cleanup callbacks to return on failure.
   Warning, error and progress reporting during execution use plugin_host_context,
   a borrowed reference describing the current application call. */
#ifndef slic3r_orchestrator_h_
#define slic3r_orchestrator_h_

#define SLIC3R_PLUGIN_API_ORCHESTRATOR_MAJOR 3u
#define SLIC3R_PLUGIN_API_ORCHESTRATOR_MINOR 5u

/*
Property registration guide:
[Using Plugin Properties](/doc/plugins/properties.md)
*/

#include <stdint.h>

#include "slic3r_bridge_detector.h"
#include "slic3r_plugin_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
Compact identifier interned by one orchestrator.

The pair (id_type, namespaced_name) is the stable identity. The numeric value
is only a process-local lookup key and must not be serialized. Each id_type has
an independent numeric namespace, so equal numbers from different types are
unrelated.

The lower half is reserved for host-defined static ids. Plugins receive ids
from the upper half and cannot choose their numeric value.
*/
/* orchestrator_runtime_id and its range constants are declared in slic3r_def.h. */

/* ========================= REGISTRATION ========================= */

typedef struct config_handle config_handle;
typedef struct print_handle print_handle;

/*
Borrowed reference to one plugin registered in an orchestrator.

The handle remains valid until the orchestrator is destroyed. It does not keep
the plugin active and must only be passed back to the orchestrator that
returned it.
*/
typedef struct orchestrator_plugin_handle orchestrator_plugin_handle;

/*
One step-specific payload used by setup_run() and run(). The orchestrator does
not inspect data; its concrete type is the private contract shared by the
caller and the selected service plugin.
*/
typedef struct raw_plugin_run_payload {
    void *data;
} raw_plugin_run_payload;

/* Result of one host-managed plugin execution. */
typedef enum raw_plugin_execution_status {
    RAW_PLUGIN_EXECUTION_SUCCESS = 1,
    RAW_PLUGIN_EXECUTION_CANCELLED = 0,
    RAW_PLUGIN_EXECUTION_INVALID_ARGUMENT = -1,
    RAW_PLUGIN_EXECUTION_INACTIVE = -2,
    RAW_PLUGIN_EXECUTION_PLUGIN_ERROR = -3
} raw_plugin_execution_status;

/*
Host application and G-code viewer version information
======================================================

Set struct_size to sizeof the caller's structure. The version 3.4 prefix
through generated_header is the minimum size; future fields are appended after
it. Macro-backed strings are valid for the process lifetime. generated_header
is borrowed until the next call on the same thread or that thread exits, and
contains neither the G-code comment prefix "; " nor a line break.
*/
typedef struct raw_slic3r_version_info {
    uint32_t struct_size;
    const char *application_name;
    const char *application_key;
    const char *application_command;
    const char *version;
    const char *version_full;
    const char *build_id;
    const char *gcodeviewer_name;
    const char *gcodeviewer_key;
    const char *gcodeviewer_command;
    const char *gcodeviewer_build_id;
    const char *based_on;
    const char *github_url;
    const char *download_url;
    const char *documentation_url;
    const char *intro_url;
    const char *generated_header;
} raw_slic3r_version_info;

/*
Fill host version information. Returns 1 on success; a null pointer, a size
smaller than the version 3.4 prefix, or failure to produce the generated header
returns 0 without modifying the structure. On success, only complete fields
that fit within struct_size are written.
*/
SLIC3R_HOST_API int32_t orchestrator_get_version_info(raw_slic3r_version_info *info);

/*
Register a plugin instance.
*/
SLIC3R_HOST_API void orchestrator_register_plugin(
    orchestrator_handle *orch,
    plugin_instance plugin
);

/* Return 1 for a registered id, including disabled instances, or 0 for invalid
   inputs, missing ids or failure. References remain borrowed and activation is
   unchanged. A NULL orchestrator returns 0; exceptions do not escape. */
SLIC3R_HOST_API int32_t orchestrator_is_plugin_registered(orchestrator_handle *orchestrator, const char *plugin_id);

/* Severity numbers match the application's logging verbosity levels. */
typedef enum raw_plugin_log_level {
    RAW_PLUGIN_LOG_FATAL = 0,
    RAW_PLUGIN_LOG_ERROR = 1,
    RAW_PLUGIN_LOG_WARNING = 2,
    RAW_PLUGIN_LOG_INFO = 3,
    RAW_PLUGIN_LOG_DEBUG = 4,
    RAW_PLUGIN_LOG_TRACE = 5
} raw_plugin_log_level;

/* Log borrowed UTF-8 text through the application's configured logger outputs
   and severity filters. NULL orchestrator selects application logging as usual; it does
   not attribute a package diagnostic or change load/activation state. NULL text,
   invalid severity and logging failures have no effect; no exception escapes. */
SLIC3R_HOST_API void orchestrator_send_log(orchestrator_handle *orchestrator,
    raw_plugin_log_level severity, const char *message);

/*
Declare jointly activated plugins. Strings are copied. Repeated declarations
of the same group accumulate their member union, independently of order.
Overlapping groups form a transitive solidarity relation, not execution edges.
Declare during registration; member existence is checked after loading.
Returns 1 on success, 0 with a logged diagnostic for invalid arguments.
IDs must be nonempty; each call requires at least two distinct, nonempty members.
Repeated members within one call are invalid. No call removes existing members.
*/
SLIC3R_HOST_API int32_t orchestrator_register_activation_group(
    orchestrator_handle *orchestrator, const char *group_id, const_strings_t members);

/*
Register a plugin instance and explicitly associate it with its installed
package directory. This variant is intended for loaders which discover a
plugin after the native package-loading scope has ended, notably the Python
loader. Relative resources, translations and diagnostics for the registered
plugin are resolved from package_root.

package_root must name the root containing description.ini and version.ini.
Normal native register_plugin() entry points should continue to call
orchestrator_register_plugin(), because PluginLoader already supplies their
package scope.
*/
SLIC3R_HOST_API void orchestrator_register_plugin_from_package(
    orchestrator_handle *orch,
    plugin_instance plugin,
    const char *package_root
);

/*
Register a service step which is not part of the host's compiled pipeline.

namespaced_name is the stable identity shared by providers and consumers, for
example "com.example.seam_placer". invalidates_step is the normal pipeline
step whose result becomes stale when the selected provider changes.

Registering the same name and invalidation again returns the same runtime id.
Registering the same name with another invalidation, exhausting the custom id
range, or passing invalid arguments returns STEP_NONE.

The returned id belongs to this orchestrator and must be registered again for
another orchestrator. Register it before registering plugin instances whose
get_step() returns that id.
*/
SLIC3R_HOST_API slicing_step_t orchestrator_register_step(
    orchestrator_handle *orchestrator,
    const char *namespaced_name,
    slicing_step_t invalidates_step
);

/*
Intern a stable name in one typed runtime-id registry.

Registering the same type and name repeatedly returns the existing id. A null
or empty type/name, a null orchestrator, or an exhausted dynamic range returns
ORCHESTRATOR_RUNTIME_ID_INVALID. The host may pre-register names in the static
range; requesting such a name returns its reserved value.
*/
SLIC3R_HOST_API orchestrator_runtime_id orchestrator_register_runtime_id(
    orchestrator_handle *orchestrator,
    const char *id_type,
    const char *namespaced_name
);

/*
Resolve a runtime id inside one type.

The returned string is borrowed from the orchestrator and remains valid until
its destruction. Unknown types and ids return NULL.
*/
SLIC3R_HOST_API const char *orchestrator_runtime_id_name(
    const orchestrator_handle *orchestrator,
    const char *id_type,
    orchestrator_runtime_id id
);

/*
Return the stable name of a dynamically registered step.

The returned string is borrowed from the orchestrator and remains valid until
its destruction. Built-in and unregistered numeric steps return NULL.
*/
SLIC3R_HOST_API const char *orchestrator_step_name(
    const orchestrator_handle *orchestrator,
    slicing_step_t step
);

/*
Find an active plugin by its stable id.

plugin_id is invalid only when it is NULL or points to an empty string. A NULL
orchestrator selects the process-wide orchestrator and is therefore valid.
Returns NULL for an invalid plugin_id, an unknown id, or a plugin that is not
part of the activation set established by the host.

The returned handle is borrowed from the orchestrator. As a defensive measure,
orchestrator_execute_plugin() validates the handle and its activation again
before invoking the plugin.
*/
SLIC3R_HOST_API const orchestrator_plugin_handle *orchestrator_find_plugin(
    orchestrator_handle *orchestrator,
    const char *plugin_id
);

/*
Select the active provider from one exclusive group on a step.

Only active members of the requested group are candidates. A sole active member
is returned without reading the stored choice unless the group's selector reads
that choice even with one active member. When reading a choice, config uses the
whole-step selector when present, otherwise the group's own option.
The returned handle stays host-owned. No configuration is changed.
A missing config uses the group-priority default. An unknown or empty stored
choice uses the group-priority default, or NULL for a strict selector.
Returns NULL for an empty group,
invalid host, step or group, or a strict selector without a matching member.
*/
SLIC3R_HOST_API const orchestrator_plugin_handle *orchestrator_select_plugin(
    orchestrator_handle *orchestrator,
    const config_handle *config,
    slicing_step_t step,
    const char *exclusive_group
);

/* Return borrowed identity metadata for a plugin handle. */
SLIC3R_HOST_API const char *orchestrator_plugin_id(
    const orchestrator_plugin_handle *plugin
);
SLIC3R_HOST_API slicing_step_t orchestrator_plugin_step(
    const orchestrator_plugin_handle *plugin
);

/*
Execute one active plugin through the complete host lifecycle.

setup() runs once with data == NULL. The host then calls setup_run() once for
each payload, waits for every setup_run() call, and calls run() once for each
payload. Calls inside either per-payload phase may run in parallel. The same
data pointer is supplied to both calls for an index, and object_idx/object_count
identify that index and the total count.

print and individual data pointers may be NULL. payloads may be NULL only when
run_count is zero. Concurrent or recursive executions of the same plugin
instance are not supported. Plugin code should be invoked from a sequential
orchestration phase and use the parallelism supplied by this function.
*/
SLIC3R_HOST_API raw_plugin_execution_status orchestrator_execute_plugin(
    orchestrator_handle *orchestrator,
    const orchestrator_plugin_handle *plugin,
    const print_handle *print,
    const raw_plugin_run_payload *payloads,
    uint32_t run_count
);

/*
Report a package-loading failure discovered by a secondary loader.

The main native loader records its failures directly. A runtime loader such as
the Python bridge runs in another shared library and uses these functions to
publish the same package-level diagnostics. package_root identifies the live
package directory containing description.ini and version.ini.
*/
typedef enum raw_plugin_package_load_error_code {
    RAW_PLUGIN_PACKAGE_LOAD_ERROR_PACKAGE_MISSING = 0,
    RAW_PLUGIN_PACKAGE_LOAD_ERROR_INVALID_PACKAGE = 1,
    RAW_PLUGIN_PACKAGE_LOAD_ERROR_LIBRARY_OPEN_FAILED = 2,
    RAW_PLUGIN_PACKAGE_LOAD_ERROR_MISSING_ABI_EXPORT = 3,
    /* Reserved to preserve the numeric values of the package-load errors. */
    RAW_PLUGIN_PACKAGE_LOAD_ERROR_RESERVED_ABI_MISMATCH = 4,
    RAW_PLUGIN_PACKAGE_LOAD_ERROR_MISSING_REGISTRATION_EXPORT = 5,
    RAW_PLUGIN_PACKAGE_LOAD_ERROR_REGISTRATION_FAILED = 6,
    RAW_PLUGIN_PACKAGE_LOAD_ERROR_NO_PLUGINS_REGISTERED = 7,
    RAW_PLUGIN_PACKAGE_LOAD_ERROR_PYTHON_RUNTIME_UNAVAILABLE = 8,
    RAW_PLUGIN_PACKAGE_LOAD_ERROR_PYTHON_READ_FAILED = 9,
    RAW_PLUGIN_PACKAGE_LOAD_ERROR_PYTHON_COMPILE_FAILED = 10,
    RAW_PLUGIN_PACKAGE_LOAD_ERROR_PYTHON_IMPORT_FAILED = 11,
    RAW_PLUGIN_PACKAGE_LOAD_ERROR_PYTHON_REGISTRATION_FAILED = 12,
    RAW_PLUGIN_PACKAGE_LOAD_ERROR_CONFIGURED_PLUGIN_MISSING = 13,
    /* The unified ABI table now uses MISSING_ABI_EXPORT instead. */
    RAW_PLUGIN_PACKAGE_LOAD_ERROR_RESERVED_MISSING_API_VERSIONS_EXPORT = 14,
    RAW_PLUGIN_PACKAGE_LOAD_ERROR_API_HEADER_VERSION_MISMATCH = 15
} raw_plugin_package_load_error_code;

/* Start the package's loading record and make its copied directory current on this thread.
   Pair with finish for the same borrowed application/path, including errors.
   Nested instance registrations preserve that directory for later group descriptions; exceptions cannot cross C. */
SLIC3R_HOST_API void orchestrator_begin_plugin_package_load(
    orchestrator_handle *orch,
    const char *package_root
);
SLIC3R_HOST_API void orchestrator_report_plugin_package_load_error(
    orchestrator_handle *orch,
    const char *package_root,
    raw_plugin_package_load_error_code code,
    const char *detail
);
/* Finish the package's loading record and release its matching current-directory record, including after errors.
   References remain borrowed, other loading records are unchanged, and exceptions cannot cross C. */
SLIC3R_HOST_API void orchestrator_finish_plugin_package_load(
    orchestrator_handle *orch,
    const char *package_root
);

/*
Register one gettext catalog domain provided by the plugin package currently
being loaded. locale_directory is relative to that package root and contains
one subdirectory per language, for example:

    locale/fr/com.example.plugin.mo

The host loads the catalog after it has selected the application language.
Call this from register_plugin(), before the loader returns control to the
host. A plugin may register several distinct domains. Returns 1 when added,
0 for an identical duplicate, and a negative value for invalid input.
*/
SLIC3R_HOST_API int32_t orchestrator_register_translation_catalog(
    orchestrator_handle *orch,
    const char *domain,
    const char *locale_directory
);

SLIC3R_HOST_API bridge_detector_instance orchestrator_create_bridge_detector(
    orchestrator_handle *orch,
    const bridge_detector_create_input *input
);

/*
Register a generic property payload and receive its runtime numeric id.

The namespaced_name is the stable identity of the payload layout, for example
"com.example.plugin.surface_priority". The returned id is the compact key
stored by PluginPropertyContainer and ExtrusionPropertyContainer. Custom ids
belong to one orchestrator; register the same name again for each slicing
orchestrator instead of serializing the numeric value.

If the name was already registered with the same size and alignment, the
existing id is returned. If the same name is registered with a different layout,
SLIC3R_PROPERTY_TYPE_INVALID is returned.
*/
SLIC3R_HOST_API slic3r_property_type orchestrator_register_property(
    orchestrator_handle *orch,
    const char *namespaced_name,
    uint32_t byte_count,
    uint32_t alignment
);

/* Return metadata for a built-in or registered property type. */
SLIC3R_HOST_API uint32_t orchestrator_property_byte_count(
    const orchestrator_handle *orch,
    slic3r_property_type type
);
SLIC3R_HOST_API uint32_t orchestrator_property_alignment(
    const orchestrator_handle *orch,
    slic3r_property_type type
);
SLIC3R_HOST_API const char *orchestrator_property_name(
    const orchestrator_handle *orch,
    slic3r_property_type type
);

/*
Register a simple seam-like FacetsAnnotation kind.

This is intentionally a small declaration API, not a custom GUI API. The host
creates the toolbar button, brush controls, left/right mouse behavior and model
storage. The plugin only provides stable identity and labels:

- key is the persistent identity, for example "com.example.plugin:paint_name".
  It is saved with the model; do not use a translated label as a key.
- label is the painter window/tool name.
- enforce_label is the left-click action label.
- block_label is the right-click action label.
- icon_svg is a complete SVG document stored as UTF-8 text. The host registers
  it in the GUI bitmap cache under key, then uses it to build the toolbar icon.

Call this during plugin registration/initialization. The GUI builds its toolbar
from the orchestrator registry, so annotation tools registered after the toolbar
exists may not be visible until the GUI is recreated.
*/
typedef struct raw_generic_facets_annotation_def {
    const char *key;
    const char *label;
    const char *enforce_label;
    const char *block_label;
    /*
    NULL or empty selects the registering plugin's default gettext domain.
    A non-empty domain must be "Slic3r" or registered by the current package.
    */
    const char *translation_domain;
    const char *icon_svg;
} raw_generic_facets_annotation_def;

/*
Returns 1 when the definition is accepted, or a negative value on invalid
arguments or internal failure.
*/
SLIC3R_HOST_API int32_t orchestrator_register_generic_facets_annotation(
    orchestrator_handle *orch,
    const raw_generic_facets_annotation_def *def
);

/*
Register a UI layout fragment for one target layout file.

target_file is the base UI file the fragment applies to, for example
"print.ui", "filament.ui", or "printer_fff.ui".

fragment_id identifies this contribution inside target_file. If the same
target_file + fragment_id pair is registered twice, the second registration is
ignored. Identical duplicate content is expected when alternative plugins in
the same exclusive group expose the same settings. If the duplicate content is
different, the host keeps the first fragment and logs a warning because those
plugins no longer agree on the UI hidden behind that shared id.

ui_fragment is a small .ui document using the normal UI layout syntax. During
GUI construction, the host parses the base UI file and this fragment, then
merges pages, groups and lines by name. Missing pages/groups/lines are created.
Use insert$before$NAME or insert$after$NAME on page/group/line commands to
place a missing node next to an existing node of the same kind. A kind may be
specified explicitly, for example insert$aftergroup$Filtering.

priority orders fragments for the same target_file. Lower priority is applied
first. Fragments with the same priority keep registration order.

Returns 1 when the fragment was added, 0 when it was already present, and a
negative value on invalid arguments or internal failure.
*/
SLIC3R_HOST_API int32_t orchestrator_add_ui_fragment(
    orchestrator_handle *orch,
    const char *target_file,
    const char *fragment_id,
    const char *ui_fragment,
    int32_t priority
);

typedef enum raw_gui_rule_condition {
    RAW_GUI_RULE_CONDITION_NONE = 0,
    RAW_GUI_RULE_CONDITION_BOOL_TRUE,
    RAW_GUI_RULE_CONDITION_BOOL_FALSE,
    RAW_GUI_RULE_CONDITION_OPTION_ENABLED,
    RAW_GUI_RULE_CONDITION_OPTION_DISABLED,
    RAW_GUI_RULE_CONDITION_VALUE_NON_ZERO,
    RAW_GUI_RULE_CONDITION_INT_EQUALS,
    RAW_GUI_RULE_CONDITION_INT_NOT_EQUALS,
    /*
    float condition_float_value comparison; works on any numeric option.
    The comparison reads the normalized value returned by ConfigOption::get_float(),
    so a percent option compares against a fraction (for example 50% is 0.5).
    */
    RAW_GUI_RULE_CONDITION_FLOAT_EQUALS,
    RAW_GUI_RULE_CONDITION_FLOAT_LESS_THAN,
    RAW_GUI_RULE_CONDITION_FLOAT_GREATER_OR_EQUAL,
    /* True when the option at condition_index stores a percentage. */
    RAW_GUI_RULE_CONDITION_IS_PERCENT,
    /* True when the option is a vector and holds no item. */
    RAW_GUI_RULE_CONDITION_VECTOR_EMPTY,
    /*
    Compare the option's serialized value with condition_string_value.

    The comparison is string-based, so it works for every option type: enums
    compare their value name (for example "smooth"), strings their text. For a
    vector option, the item selected by condition_index is compared: an explicit
    index reads that item, RAW_GUI_RULE_INDEX_CURRENT reads the current extruder
    item and RAW_GUI_RULE_INDEX_ALL is true when any item matches. A leading '!'
    (disabled option) is not part of the compared value, matching serialize()
    normalization used elsewhere in the GUI.
    */
    RAW_GUI_RULE_CONDITION_STRING_EQUALS,
    /* Number of declared condition kinds; unknown values are rejected. */
    RAW_GUI_RULE_CONDITION_COUNT
} raw_gui_rule_condition;

/*
Index values shared by target_index and condition_index.

RAW_GUI_RULE_INDEX_ALL means "no specific item":
- for target_index, apply the rule to the whole GUI field;
- for condition_index, automatically choose the safest condition read:
  use the current extruder item when such an item exists, otherwise enable if
  any vector item satisfies the condition.

RAW_GUI_RULE_INDEX_CURRENT is mainly useful for target_index. It means "apply
this rule to the vector item currently processed by the GUI refresh loop".
Use it only for options whose value is sized like the extruder count.
*/
#define RAW_GUI_RULE_INDEX_ALL     (-1)
#define RAW_GUI_RULE_INDEX_CURRENT (-2)

/*
One condition clause of a GUI rule.

A clause is a single predicate on one config option. condition_index follows
the index contract documented above. condition_float_value carries the numeric
operand:

- RAW_GUI_RULE_CONDITION_INT_EQUALS and RAW_GUI_RULE_CONDITION_INT_NOT_EQUALS
  compare the option integer value to llround(condition_float_value), so enum
  comparisons use the same field;
- RAW_GUI_RULE_CONDITION_FLOAT_EQUALS, RAW_GUI_RULE_CONDITION_FLOAT_LESS_THAN
  and RAW_GUI_RULE_CONDITION_FLOAT_GREATER_OR_EQUAL compare the normalized
  option float value directly;
- RAW_GUI_RULE_CONDITION_STRING_EQUALS compares the serialized option value
  (or, for a vector option, the serialized item selected by condition_index)
  with condition_string_value;
- IS_PERCENT and VECTOR_EMPTY ignore both operands.
*/
typedef struct raw_gui_rule_clause {
    raw_gui_rule_condition condition;
    const char *condition_key;
    int32_t condition_index;
    double condition_float_value;
    /*
    Expected serialized value for STRING_EQUALS. Borrowed only for the duration
    of orchestrator_add_gui_rule(); the host copies it.
    */
    const char *condition_string_value;
} raw_gui_rule_clause;

/* Return an empty clause with no specific item and no borrowed keys. */
static inline raw_gui_rule_clause raw_gui_rule_clause_init()
{
    raw_gui_rule_clause clause = {RAW_GUI_RULE_CONDITION_NONE, 0, RAW_GUI_RULE_INDEX_ALL, 0.0, 0};
    return clause;
}

/*
GUI activation rule.

A rule is one reason to keep target_key editable. Several rules may share the
same target:

- the clauses of one rule are combined with a logical AND;
- the target is enabled when at least one rule registered for it evaluates to
  true (OR). There is no separate "must hold" rule kind: every caller that
  registers a rule for a setting declares a condition under which that setting
  is useful to someone.

A setting with no rule at all stays enabled.

- scope restricts a rule to GUI contexts that refresh one preset kind.
  RAW_PRESET_TYPE_NONE applies to every context. The FFF print settings use
  RAW_PRESET_TYPE_FFF_PRINT so their rules are never evaluated while the
  printer or SLA tab refreshes.
- target_index and the clause condition_index are used for vector/extruder
  options. Use RAW_GUI_RULE_INDEX_ALL for scalar options or when the whole
  field should be affected. Use RAW_GUI_RULE_INDEX_CURRENT for a target that
  must be applied item by item in the current extruder loop.
- struct_size must be set to sizeof(raw_gui_rule) and clause_struct_size to
  sizeof(raw_gui_rule_clause). They allow later additions to the structures
  without breaking already compiled callers: the host rejects a descriptor
  whose struct_size or clause_struct_size is smaller than the host layout.

Rules whose target setting is not defined yet are queued and attached when the
definition appears; a non-blocking warning lists the keys that never appeared.

Rejected descriptors are never stored and are reported through the host log.

Returns:
- 1 when the rule was added;
- 0 when an identical rule was already registered;
- -1 when the descriptor is invalid. Validation rejects a null descriptor or
  a truncated struct_size, an unknown condition or scope, a negative
  target_index or condition_index other than RAW_GUI_RULE_INDEX_ALL and
  RAW_GUI_RULE_INDEX_CURRENT, a null or empty target_key or clause
  condition_key, a null clause array or an empty clause list, and a
  non-finite condition_float_value on a FLOAT_* condition;
- -2 on internal failure.
*/
typedef struct raw_gui_rule {
    uint32_t struct_size;
    const char *target_key;
    int32_t target_index;
    raw_option_preset_type scope;
    const raw_gui_rule_clause *clauses;
    uint32_t clause_count;
    /* Size of one element of clauses; must be sizeof(raw_gui_rule_clause). */
    uint32_t clause_struct_size;
} raw_gui_rule;

static inline raw_gui_rule raw_gui_rule_init()
{
    raw_gui_rule rule = {0};
    rule.struct_size = (uint32_t)sizeof(raw_gui_rule);
    rule.clause_struct_size = (uint32_t)sizeof(raw_gui_rule_clause);
    rule.target_index = RAW_GUI_RULE_INDEX_ALL;
    rule.scope = RAW_PRESET_TYPE_NONE;
    return rule;
}

SLIC3R_HOST_API int32_t orchestrator_add_gui_rule(
    orchestrator_handle *orch,
    const raw_gui_rule *rule
);

/*
Return the config keys used by the plugin or plugins currently active for a
step.

This is mainly for plugins that need to mirror another plugin's compatibility
rules. For example, a surface-generation plugin may need to split regions by
the settings used by the selected STEP_PERIMETER plugin when concentric infill
delegates line generation to the perimeter generator.

For exclusive steps, config is the project/full config that contains the
exclusive-step selector option, such as "step_perimeter_plugin"; the function
returns the selected plugin's keys. For non-exclusive steps, no selection exists,
so the function returns the de-duplicated union of every active plugin's keys.
If a selector option does not exist because only one plugin is active, the host
falls back to that active plugin.

The function uses the usual double-call convention:
- call with keys == NULL to get the number of entries;
- call again with an array of that size to receive borrowed key pointers.
*/
SLIC3R_HOST_API int32_t orchestrator_selected_plugin_used_config_keys(
    orchestrator_handle *orch,
    const config_handle *config,
    slicing_step_t step,
    raw_used_config_key *keys
);

/*
Default host callbacks used to populate plugin_run_context.
Plugins normally call these through the function pointers stored in the run
context instead of calling them directly.

report_warning copies the message into the host diagnostic queue. report_error
marks the current plugin execution as failed; normal slicing also queues the
message and requests cancellation, while a host action may capture it in
call-local storage instead. The GUI drains queued messages as Plater
notifications. NULL host context only logs an unattributed error.
*/
SLIC3R_HOST_API int orchestrator_plugin_is_cancelled(plugin_host_context *host_context);
SLIC3R_HOST_API void orchestrator_plugin_report_warning(plugin_host_context *host_context, const char *message);
SLIC3R_HOST_API void orchestrator_plugin_report_error(plugin_host_context *host_context, const char *message);
SLIC3R_HOST_API void orchestrator_plugin_report_progress(plugin_host_context *host_context, double progress, const char *message);

/*
Register one UTF-8 diagnostic message for the lifetime of the running
application and return its id. A plugin callback reports a failure by returning
that id (plugin_initialize_fn or plugin_clear_fn); the host then
resolves the id, logs the text and queues it as a Plater notification.

Returns PLUGIN_ERROR_GENERAL (1) when orchestrator is NULL, when message is NULL
or empty, or when the message cannot be stored. Otherwise the result is an id in
[PLUGIN_ERROR_FIRST_DYNAMIC, INT32_MAX], unique and never reused. Ids are
process-local: do not persist them and do not reuse one across runs.
*/
SLIC3R_HOST_API int32_t orchestrator_set_error_message(
    orchestrator_handle *orchestrator,
    const char *message
);

#ifdef __cplusplus
}
#endif

#endif // slic3r_orchestrator_h_
