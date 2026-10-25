///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
/* Configuration definitions carry metadata and defaults across the C ABI. */
#ifndef slic3r_config_def_h_
#define slic3r_config_def_h_

#define SLIC3R_PLUGIN_API_CONFIG_DEF_MAJOR 1u
#define SLIC3R_PLUGIN_API_CONFIG_DEF_MINOR 4u

#include "slic3r_config_option.h"
#include "slic3r_config_types.h"
#include "slic3r_plugin_types.h"
#include "slic3r_utils.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================== ENUMS ========================== */

/* Nature combines withdrawal flags matching ConfigOptionNature: zero is NORMAL
   and every combination of the four flags is permitted. */
typedef enum raw_config_option_nature {
    RAW_CONFIG_NATURE_NOT_LOCAL = 1,      /* Withdraw writing to the setting owner's file. */
    RAW_CONFIG_NATURE_NOT_PROJECT = 2,    /* Withdraw writing to the project. */
    RAW_CONFIG_NATURE_NOT_GCODE = 4,      /* Withdraw writing to G-code. */
    RAW_CONFIG_NATURE_NOT_COMPARABLE = 8, /* Withdraw participation in value comparisons. */
    RAW_CONFIG_NATURE_NORMAL = 0,        /* Persisted and comparable without restrictions. */
    RAW_CONFIG_NATURE_LOCAL = RAW_CONFIG_NATURE_NOT_PROJECT | RAW_CONFIG_NATURE_NOT_GCODE, /* Comparable, written only to the owner's file. */
    RAW_CONFIG_NATURE_BOOKKEEPING = RAW_CONFIG_NATURE_NOT_COMPARABLE, /* Persisted tracking information, not compared. */
    RAW_CONFIG_NATURE_PRESET_META = RAW_CONFIG_NATURE_NOT_GCODE, /* Comparable profile metadata, not written to G-code. */
    /* Runtime-only: neither persisted nor compared. */
    /* Keep the expression on one line; otherwise the Python API generator omits this constant. */
    RAW_CONFIG_NATURE_VOLATILE = RAW_CONFIG_NATURE_NOT_LOCAL | RAW_CONFIG_NATURE_NOT_PROJECT | RAW_CONFIG_NATURE_NOT_GCODE | RAW_CONFIG_NATURE_NOT_COMPARABLE
} raw_config_option_nature;

/* GUI behavior / rendering type */
typedef enum raw_gui_type {
    RAW_GUI_TYPE_UNDEFINED = 0,

    /* Open enums (value may be outside predefined list) */
    RAW_GUI_TYPE_I_ENUM_OPEN,
    RAW_GUI_TYPE_F_ENUM_OPEN,
    RAW_GUI_TYPE_SELECT_OPEN,

    /* Color picker (string value) */
    RAW_GUI_TYPE_COLOR,

    /* Currently unused */
    RAW_GUI_TYPE_SLIDER,

    /* Static text */
    RAW_GUI_TYPE_LEGEND,

    /* Closed enum (must match provided values) */
    RAW_GUI_TYPE_SELECT_CLOSE
} raw_gui_type;

/* Category of a configuration field (GUI grouping) */
typedef enum raw_option_category {
    RAW_OPTION_CATEGORY_NONE = 0,

    RAW_OPTION_CATEGORY_PERIMETER,
    RAW_OPTION_CATEGORY_SLICING,
    RAW_OPTION_CATEGORY_INFILL,
    RAW_OPTION_CATEGORY_IRONING,
    RAW_OPTION_CATEGORY_FUZZY_SKIN,
    RAW_OPTION_CATEGORY_SKIRT_BRIM,
    RAW_OPTION_CATEGORY_SUPPORT,
    RAW_OPTION_CATEGORY_SPEED,
    RAW_OPTION_CATEGORY_WIDTH,
    RAW_OPTION_CATEGORY_EXTRUDERS,
    RAW_OPTION_CATEGORY_OUTPUT,
    RAW_OPTION_CATEGORY_NOTES,
    RAW_OPTION_CATEGORY_DEPENDENCIES,

    RAW_OPTION_CATEGORY_FILAMENT,
    RAW_OPTION_CATEGORY_COOLING,
    RAW_OPTION_CATEGORY_ADVANCED,
    RAW_OPTION_CATEGORY_FILOVERRIDE,
    RAW_OPTION_CATEGORY_CUSTOMGCODE,

    RAW_OPTION_CATEGORY_GENERAL,
    RAW_OPTION_CATEGORY_LIMITS,
    RAW_OPTION_CATEGORY_MMSETUP,
    RAW_OPTION_CATEGORY_FIRMWARE,

    RAW_OPTION_CATEGORY_PAD,
    RAW_OPTION_CATEGORY_PAD_SUPP,
    RAW_OPTION_CATEGORY_WIPE,

    RAW_OPTION_CATEGORY_HOLLOWING,

    RAW_OPTION_CATEGORY_MILLING_EXTRUDERS,
    RAW_OPTION_CATEGORY_MILLING

} raw_option_category;

/* Logical level of the option (UI / filtering purpose) */
typedef enum raw_option_level {
    RAW_OPTION_LEVEL_BASIC = 0,
    RAW_OPTION_LEVEL_ADVANCED,
    RAW_OPTION_LEVEL_EXPERT
} raw_option_level;

/* Which printer technology this config applies to */
typedef enum raw_printer_technology {
    RAW_PT_NONE = 0,
    RAW_PT_FFF,
    RAW_PT_SLA,
    /* Definition shared by both currently supported printing technologies. */
    RAW_PT_FFF_AND_SLA
} raw_printer_technology;

/* Bitmask flags for modes (mirrors ConfigOptionMode) */
typedef uint64_t raw_config_option_mode;
#define RAW_CONFIG_OPTION_MODE_NONE 0
#define RAW_CONFIG_OPTION_MODE_SIMPLE 1
#define RAW_CONFIG_OPTION_MODE_ADVANCED (1 << 1)
#define RAW_CONFIG_OPTION_MODE_EXPERT (1 << 2)
#define RAW_CONFIG_OPTION_MODE_ADV_EXP RAW_CONFIG_OPTION_MODE_ADVANCED | RAW_CONFIG_OPTION_MODE_EXPERT
#define RAW_CONFIG_OPTION_MODE_SIM_ADV_EXP RAW_CONFIG_OPTION_MODE_SIMPLE | RAW_CONFIG_OPTION_MODE_ADVANCED | RAW_CONFIG_OPTION_MODE_EXPERT
#define RAW_CONFIG_OPTION_MODE_PRUSA (1 << 3)
#define RAW_CONFIG_OPTION_MODE_SUSI (1 << 4)
#define RAW_CONFIG_OPTION_MODE_HIDDEN (1 << 5)

/* Generic extensibility flags */
typedef uint64_t RawConfigOptionFlags;

/* ========================== ARRAY HELPERS ========================== */

/* Pair of (value, label) for enum entries */
typedef struct key_value_string_pair_t {
    const char *value; /* stored value */
    const char *label; /* display label */
} key_value_string_pair_t;

/* Array of enum pairs */
typedef struct key_value_string_pair_array_t {
    const key_value_string_pair_t *items;
    uint32_t count;
} key_value_string_pair_array_t;

/* ========================== ENUM DEFINITION ========================== */

/*
Definition of values / labels for a combo box.
A value is a lowercase string with only alphanumerical and '_' characters. It is used to serialize the value.
A Label is the string that is shown in the interface.
*/
typedef struct option_enum_def_t {
    key_value_string_pair_array_t value_label_pairs; /* value+label pairs */
} option_enum_def_t;

/* A graph value expressed without exposing host-owned C++ containers. */
typedef struct raw_graph_data {
    /* Usable range inside points: begin_index is included, end_index excluded. */
    uint32_t begin_index;
    uint32_t end_index;
    raw_graph_type type;
    const c_vec2d *points;
    uint32_t point_count;
} raw_graph_data;

/*
GUI metadata required to edit a GRAPH or VECTOR_GRAPH option.

All strings and arrays are borrowed only for the duration of
orchestrator_create_option_def(). The host copies them. Introspection snapshots
returned by orchestrator_get_option_def() remain valid until the next query on
the same thread, following the lifetime of the containing definition.
*/
typedef struct raw_graph_settings {
    const char *title;
    const char *description;
    const char *y_label;
    const char *x_label;
    const char *null_label;

    /* Finite editor bounds with min < max, and a finite positive step. */
    double min_x;
    double max_x;
    double step_x;
    double min_y;
    double max_y;
    double step_y;

    const char *label_min_x;
    const char *label_max_x;
    const char *label_min_y;
    const char *label_max_y;

    /* NONE is not an interpolation mode and is rejected in this array. */
    const raw_graph_type *allowed_types;
    uint32_t allowed_type_count;

    /*
    Optional reset graph. NONE means that the editor falls back to the option's
    default value. A real interpolation type supplies an explicit reset target;
    its point list may intentionally be empty.
    */
    raw_graph_data reset_values;

    /* Zero, one or two endpoint values enforced by the graph editor. */
    const c_vec2d *enforced_values;
    uint32_t enforced_value_count;
} raw_graph_settings;


/* ========================== MAIN STRUCT ========================== */

/*
Definition of a configuration value for:
- GUI presentation
- editing
- value mapping
- config file handling

This is the C-compatible version of ConfigOptionDef.
*/
typedef struct raw_config_option_def {

    /* Identifier of this option (unique key) */
    const char *opt_key;

    /* What type? bool, int, string etc. */
    raw_config_option_type type;

    /* Category of the option (GUI grouping) */
    raw_option_category category;

    /* Logical level (basic/advanced/expert) */
    raw_option_level level;

    /* Where this option is accessible */
    raw_container_type container_type;

    /* Type of preset it resides in */
    raw_option_preset_type option_preset_type;

    /* GUI type (affects rendering & interaction) */
    raw_gui_type gui_type;

    /* Which printer technology this applies to */
    raw_printer_technology printer_technology;

    /*
    Earliest slicing step invalidated when this option changes.
    The default STEP_NONE means "not filled" for plugin-created options: while
    a plugin is initialized, the host replaces it with the plugin's own step.
    Use STEP_ANY to force conservative full invalidation. Outside plugin option
    registration, STEP_NONE still means no slicing invalidation.
    */
    slicing_step_t invalidates_step;

    /* If optional and enabled, may not be serialized */
    int32_t is_optional;

    /* GUI flags (e.g. "serialized", "show_value") */
    const char *gui_flags;

    /*
    Label of the GUI input field.
    Short label used in grouped views.
    */
    const char *label;

    /*
    Full label used when shown standalone or in overrides.
    If NULL, label is used instead.
    */
    const char *full_label;

    /* Tooltip text shown in GUI */
    const char *tooltip;

    /*
    Gettext domain used for every user-facing text in this definition,
    including enum labels. NULL or empty selects the registering plugin's
    default domain. The domain must either be "Slic3r" or have been
    registered with orchestrator_register_translation_catalog().
    */
    const char *translation_domain;

    /* Text displayed next to input (e.g. unit) */
    const char *sidetext;

    /* CLI representation (command-line argument format) */
    const char *cli;

    /*
    Reference option key for percentage-based values.
    Example: speed relative to another speed.
    */
    const char *ratio_over;

    /* True for multiline text inputs */
    int32_t multiline;

    /* If true, GUI input spans full width */
    int32_t full_width;

    /* If true, display as code (monospace) */
    int32_t is_code;

    /*
    For array settings:
    if true, size matches number of extruders
    */
    int32_t is_vector_extruder;

    /* Not editable (read-only field) */
    int32_t readonly;

    /*
    Can be "phony":
    if missing at load, GUI adapts accordingly
    */
    int32_t can_phony;
    
    /* If true, option can be disabled (separate flag, not nullable) */
    /* Can be enabled/disabled via checkbox */
    int32_t can_be_disabled;

    /* Height of multiline input */
    int32_t height;

    /* Width of input field */
    int32_t width;

    /* Width of label */
    int32_t label_width;

    /* If true, label aligned left instead of right */
    int32_t aligned_label_left;

    /* Width of sidetext */
    int32_t sidetext_width;

    /*
    Numeric limits:
    If not set, defaults to (-inf, +inf)
    */
    double min_value;
    double max_value;
    int32_t has_min;
    int32_t has_max;

    /*
    Max literal check:
    Used to detect missing % or abnormal values.
    */
    double max_literal_value;
    int32_t max_literal_is_percent;
    int32_t has_max_literal;

    /* Display precision (digits after decimal point) */
    int32_t precision;

    /* Mode flags (bitmask) */
    raw_config_option_mode mode;

    /* Generic flags for future extension */
    RawConfigOptionFlags flags;

    /*
    Legacy names for this option.
    Used when parsing old config files.
    */
    const_strings_t aliases;

    /*
    Shortcut mapping:
    one option maps to multiple internal ones
    */
    const_strings_t shortcut;

    /*
    Dependencies:
    list of "opt_key#idx" that affect this value
    */
    const_strings_t depends_on;

    /* Default value (serialized in a string) */
    const char* default_serialized_value;

    /* Enum definition (for select/enum types) */
    option_enum_def_t enum_def;

    /*
    Set to 1 when every value of this enum is a plugin id.

    Such a list may only contain ids of registered plugins. When plugin option
    definitions are finalized, the host removes values whose plugin is missing
    or inactive and reports a non-fatal error when the remaining list is empty.
    No host-only value may be mixed into a plugin-id list.

    This flag and nature occupy the first reserved 64-bit slot. The remaining
    reserved_u64 entries keep the structure size and following offsets unchanged.
    */
    int32_t contains_plugin_id;
    /* A combination of raw_config_option_nature withdrawal flags, stored at a fixed 32-bit width.
       Callers that zero the reserved area obtain NORMAL. */
    uint32_t nature;

    /* Reserved for future expansion (must be zero) */
    uint64_t reserved_u64[7];

    /*
    Optional graph editor definition. It is accepted only for GRAPH and
    VECTOR_GRAPH options. This field consumes the first pointer slot reserved
    by version 1.1, preserving the size and offsets of the public structure.
    */
    const raw_graph_settings *graph_settings;

    /* Reserved for future expansion (must be zero) */
    const void *reserved_ptr[7];

} raw_config_option_def;

/* ========================= CONFIG CREATION ========================= */
typedef enum option_def_error_code {
    OPTION_DEF_ERROR_OK = 0,
    OPTION_DEF_ERROR_ALREADY_EXISTS,
    OPTION_DEF_ERROR_INVALID_ARGUMENT,
    OPTION_DEF_ERROR_INTERNAL
} option_def_error_code;

/*
Initialize a config option definition with safe defaults.
invalidates_step defaults to STEP_NONE. During plugin initialization the host
interprets that default as "use the plugin's own slicing step". Set STEP_ANY
explicitly if an option must invalidate the whole slicing state.
*/
static inline raw_config_option_def raw_config_option_def_init()
{
    raw_config_option_def def = {0};
    def.height = -1;
    def.width = -1;
    def.label_width = -1;
    def.sidetext_width = -1;
    def.invalidates_step = STEP_NONE;
    return def;
}

/*
Create a new config option definition.
The options will be available in the gui and you will have access to them 
*/
SLIC3R_HOST_API option_def_error_code orchestrator_create_option_def(
    orchestrator_handle *orch,
    const raw_config_option_def *def
);

/* ========================= CONFIG DEFINITION REQUEST ========================= */

/* One borrowed option key and the type of value described by its definition. */
typedef struct raw_config_option_key {
    const char *key;
    raw_config_option_type type;
} raw_config_option_key;

/*
Enumerate the global print configuration definitions in lexicographic key order.

RAW_CO_NONE disables filtering. Any other supported type keeps only matching
definitions. The return value is always the total number of matching entries.
Call first with out == NULL or capacity == 0 to obtain the required capacity;
a smaller output array receives a deterministic prefix.

The key pointers are borrowed from the global definition and remain valid for
the lifetime of the host. They must not be modified or freed.
*/
SLIC3R_HOST_API uint32_t orchestrator_get_option_keys(
    const orchestrator_handle *orchestrator,
    raw_config_option_type type_filter,
    raw_config_option_key *out,
    uint32_t capacity);

/*
Copy one normalized global option definition into caller-owned storage.

The raw_config_option_def structure itself belongs to the caller. Every pointer
stored inside it refers to a thread-local host snapshot and remains valid only
until the next orchestrator_get_option_def() call made by the same thread. The
caller must copy any strings or arrays it needs to retain and must never free or
modify the pointed-to data.

Return 1 on success. A null key, null output, or unknown key returns 0. When an
output structure is provided on failure, every byte is reset to zero.
*/
SLIC3R_HOST_API int32_t orchestrator_get_option_def(
    const orchestrator_handle *orchestrator,
    const char *key,
    raw_config_option_def *out);

/* ========================= CONFIG DEFINITION PATCH ========================= */

/*
One deferred modification of an existing configuration option definition.

Patches are recorded while plugins initialize and applied together after every
plugin has been loaded. Keeping them deferred makes the result independent of
plugin initialization order and lets a patch target a key that another plugin
creates later. A patch whose target key is never defined is ignored.

Patch kinds:

- ADD_ENUM_VALUES appends (prepend != 0: prepends) entries to an enum. Already
  present values keep their first position and label.
- REMOVE_ENUM_VALUES removes the listed values from an enum.
- SET_DEFAULT replaces the default with default_serialized_value.
- REPLICATE_ENUM replaces the enum with a snapshot of source_option_key's
  resolved values. The source is resolved first and may itself be built from
  patches; a dependency cycle is broken softly, so each option still resolves
  deterministically.

Replicating an option whose values change later is impossible because the
snapshot is taken once during finalization.
*/
typedef enum raw_option_def_patch_kind {
    RAW_OPTION_DEF_PATCH_NONE = 0,
    RAW_OPTION_DEF_PATCH_ADD_ENUM_VALUES,
    RAW_OPTION_DEF_PATCH_REMOVE_ENUM_VALUES,
    RAW_OPTION_DEF_PATCH_SET_DEFAULT,
    RAW_OPTION_DEF_PATCH_REPLICATE_ENUM
} raw_option_def_patch_kind;

typedef struct raw_option_def_patch {
    raw_option_def_patch_kind kind;

    /* Target option key. Required for every kind. */
    const char *opt_key;

    /* ADD_ENUM_VALUES entries. The host copies values and labels. */
    key_value_string_pair_array_t enum_values;

    /* REMOVE_ENUM_VALUES entries. Only the value strings are read. */
    const_strings_t remove_values;

    /* SET_DEFAULT serialized value, for example "rectilinear". */
    const char *default_serialized_value;

    /* REPLICATE_ENUM source option key. */
    const char *source_option_key;

    /* ADD_ENUM_VALUES position: 0 appends, 1 prepends. */
    int32_t prepend;

    /* Reserved for future expansion (must be zero). */
    uint64_t reserved[4];
} raw_option_def_patch;

static inline raw_option_def_patch raw_option_def_patch_init()
{
    raw_option_def_patch patch = {0};
    return patch;
}

/*
Record one deferred option definition patch.

The patch is retained until the host finalizes plugin option definitions. An
invalid descriptor is rejected and nothing is recorded. Reusing a key owned by
another plugin or exclusive group is intentionally allowed: a patch adds values
to an existing list, it does not redefine the option.

Returns OPTION_DEF_ERROR_OK when the patch was recorded.
*/
SLIC3R_HOST_API option_def_error_code orchestrator_modify_option_def(
    orchestrator_handle *orch,
    const raw_option_def_patch *patch);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // slic3r_config_def_h_
