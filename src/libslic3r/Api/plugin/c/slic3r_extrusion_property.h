///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_extrusion_property_h_
#define slic3r_extrusion_property_h_

/* Major 4 separates fan requests from speed, keeping heater events and the DEPOSIT command. */
#define SLIC3R_PLUGIN_API_EXTRUSION_PROPERTY_MAJOR 4u
#define SLIC3R_PLUGIN_API_EXTRUSION_PROPERTY_MINOR 0u

#include <stdint.h>

#include "slic3r_def.h"
#include "slic3r_gcode_script.h"
#include "slic3r_utils.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
Extrusion properties describe how an extrusion entity should be interpreted.

Each extrusion entity may contain at most one property of a given type. A
property inherited from a parent entity applies to its descendants until a child
defines another property of the same type, unless the property's payload
documentation describes an entry event with different lifetime rules.

Property payloads are plain C byte records. The built-in property structures
below are stable ABI records. Custom properties registered by plugins follow the
same rule: they must be trivially copyable raw data. If a property needs text or
larger binary data, store that data on the extrusion entity with
extrusion_property_store_data_aligned(), passing the address of the
extrusion_data_id field that will reference the data.

Payloads evolve append-only: a new field is only added at the end of a
structure, which needs a MINOR version bump, while moving, resizing or removing
an existing field needs a MAJOR one.

Developer guide: [Using Plugin Properties](/doc/plugins/properties.md)

Entity guide: [Using Unified Extrusion Entities](/doc/plugins/extrusions.md)
*/

typedef struct extrusion_entity_handle extrusion_entity_handle;
typedef struct orchestrator_handle orchestrator_handle;

typedef slic3r_property_type extrusion_property_type;

#define EXTRUSION_PROPERTY_TYPE_INVALID         ((extrusion_property_type)SLIC3R_PROPERTY_TYPE_INVALID)
#define EXTRUSION_PROPERTY_TYPE_ATTRIBUTES      ((extrusion_property_type)SLIC3R_PROPERTY_TYPE_EXTRUSION_ATTRIBUTES)
#define EXTRUSION_PROPERTY_TYPE_SPEED           ((extrusion_property_type)SLIC3R_PROPERTY_TYPE_EXTRUSION_SPEED)
#define EXTRUSION_PROPERTY_TYPE_MODIFIER        ((extrusion_property_type)SLIC3R_PROPERTY_TYPE_EXTRUSION_MODIFIER)
#define EXTRUSION_PROPERTY_TYPE_CUSTOM_GCODE    ((extrusion_property_type)SLIC3R_PROPERTY_TYPE_EXTRUSION_CUSTOM_GCODE)
#define EXTRUSION_PROPERTY_TYPE_SPECIAL_COMMAND ((extrusion_property_type)SLIC3R_PROPERTY_TYPE_EXTRUSION_SPECIAL_COMMAND)
#define EXTRUSION_PROPERTY_TYPE_OVERHANG        ((extrusion_property_type)SLIC3R_PROPERTY_TYPE_EXTRUSION_OVERHANG)
#define EXTRUSION_PROPERTY_TYPE_Z_OFFSET        ((extrusion_property_type)SLIC3R_PROPERTY_TYPE_EXTRUSION_Z_OFFSET)
#define EXTRUSION_PROPERTY_TYPE_PERIMETER       ((extrusion_property_type)SLIC3R_PROPERTY_TYPE_EXTRUSION_PERIMETER)
#define EXTRUSION_PROPERTY_TYPE_INFILL          ((extrusion_property_type)SLIC3R_PROPERTY_TYPE_EXTRUSION_INFILL)
#define EXTRUSION_PROPERTY_TYPE_EXTRUSION_AXIS  ((extrusion_property_type)SLIC3R_PROPERTY_TYPE_EXTRUSION_AXIS)
#define EXTRUSION_PROPERTY_TYPE_HEATER           ((extrusion_property_type)SLIC3R_PROPERTY_TYPE_EXTRUSION_HEATER)
#define EXTRUSION_PROPERTY_TYPE_FAN              ((extrusion_property_type)SLIC3R_PROPERTY_TYPE_EXTRUSION_FAN)

typedef uint32_t extrusion_data_id;

#define EXTRUSION_DATA_ID_INVALID ((extrusion_data_id)UINT32_MAX)

/*
Register a plugin-defined property type on one orchestrator. orch must not be
NULL for custom properties.

namespaced_name should be stable and unique, for example:
    "com.example.plugin.property_name"

byte_count is the size of one property payload.
alignment is the required C alignment of that payload. Use _Alignof(T) in C11 or
alignof(T) in C++ wrappers.

The returned type id is valid for this orchestrator. It is not a file format
identifier and must not be serialized as a stable value. Keeping the registry on
the orchestrator lets multiple plugin configurations slice in parallel without
sharing custom type ids.
*/
SLIC3R_HOST_API extrusion_property_type extrusion_property_register_type(orchestrator_handle *orch,
                                                                         const char *namespaced_name,
                                                                         uint32_t byte_count,
                                                                         uint32_t alignment);

/* Return the registered byte size for one property payload, or 0 if unknown. */
SLIC3R_HOST_API uint32_t extrusion_property_byte_count(const orchestrator_handle *orch, extrusion_property_type type);

/* Return the registered alignment for one property payload, or 0 if unknown. */
SLIC3R_HOST_API uint32_t extrusion_property_alignment(const orchestrator_handle *orch, extrusion_property_type type);

/*
Return the property type name, or NULL if unknown.

For plugin-defined properties, this is the exact namespaced_name passed to
extrusion_property_register_type(). For built-in properties, this is a stable
host-defined name. The returned string is host-owned.
*/
SLIC3R_HOST_API const char *extrusion_property_name(const orchestrator_handle *orch, extrusion_property_type type);

/* Return the number of properties stored directly on this extrusion entity. */
SLIC3R_HOST_API uint32_t extrusion_property_count(const extrusion_entity_handle *entity);

/*
Return the property type stored at idx.

This is only for enumeration and discovery of the properties stored directly on
this entity. The returned type can be passed to extrusion_property_data(),
extrusion_property_byte_count(), extrusion_property_alignment(), and
extrusion_property_name(). Pass the same orchestrator used to register custom
properties to those introspection functions. This is useful for tools that need
to preserve or inspect custom properties without knowing them at compile time.

The order has no semantic meaning. Returns EXTRUSION_PROPERTY_TYPE_INVALID if
entity is NULL or idx is invalid.
*/
SLIC3R_HOST_API extrusion_property_type extrusion_property_type_at(const extrusion_entity_handle *entity, uint32_t idx);

/* Return non-zero if this extrusion entity directly stores a property of type. */
SLIC3R_HOST_API int32_t extrusion_property_has(const extrusion_entity_handle *entity, extrusion_property_type type);

/*
Return a read-only pointer to one property payload.

The type selects the property slot. Its byte size is
extrusion_property_byte_count(orch, type) when the property type is known to
the orchestrator. The pointer is owned by the extrusion entity. It remains valid
until the property is removed, replaced, or the owning entity is modified by
copy/move or destroyed. Store the data elsewhere if it needs to outlive the
current callback.

This is different from extrusion_data(): property data is the small typed
payload selected by extrusion_property_type; stored data is a larger auxiliary
buffer selected by extrusion_data_id.
*/
SLIC3R_HOST_API const void *extrusion_property_data(const extrusion_entity_handle *entity, extrusion_property_type type);

/*
Return a mutable pointer to an existing property payload.

This does not create the property. Returns NULL if the property is absent or the
type is unknown.
*/
SLIC3R_HOST_API void *extrusion_property_data_mutable(extrusion_entity_handle *entity, extrusion_property_type type);

/*
Return a mutable pointer to a property payload, creating it if needed.

If the property is created, its bytes are zero-initialized. Returns NULL if the
type is unknown in this orchestrator or the entity cannot be modified.
*/
SLIC3R_HOST_API void *extrusion_property_get_or_add_data_mutable(orchestrator_handle *orch,
                                                                 extrusion_entity_handle *entity,
                                                                 extrusion_property_type type);

/*
Remove one property from this entity. Returns non-zero if a property was removed.

Stored data created with extrusion_property_store_data_aligned() for fields in
this property type is released at the same time. Stored data created with
extrusion_store_data_aligned() is independent and is not released here.
*/
SLIC3R_HOST_API int32_t extrusion_property_remove(extrusion_entity_handle *entity, extrusion_property_type type);

/*
Store arbitrary byte data on this extrusion entity and return its id.

The host copies byte_size bytes from data immediately. The caller may reuse,
modify, or release its source buffer after this function returns. data must point
to byte_size readable bytes unless byte_size is 0.

The returned id can be placed inside a property payload, but it is not owned by
that property. The host owns the stored memory, clones it with the extrusion
entity, and releases it when the entity or data id is destroyed.

alignment must be a power of two and at least 1. Passing sizeof(T) bytes with
alignment alignof(T) allows the returned data pointer to be safely cast to T*.
*/
SLIC3R_HOST_API extrusion_data_id extrusion_store_data_aligned(extrusion_entity_handle *entity,
                                                               const void *data,
                                                               uint32_t byte_size,
                                                               uint32_t alignment);

/*
Store arbitrary byte data owned by one extrusion_data_id field inside a property.

Use this for text or binary data whose lifetime should follow a property field.
The host copies the bytes immediately, releases the old data referenced by this
same field, writes the new id into *field, and returns that id.

field must point inside the mutable payload of owner_type, usually obtained from
extrusion_property_get_or_add_data_mutable(). This lets the host verify that the
data id field really belongs to the selected property.

When extrusion_property_remove() removes owner_type, or when owner_type is
replaced, all data stored for fields in owner_type is released automatically.
*/
SLIC3R_HOST_API extrusion_data_id extrusion_property_store_data_aligned(extrusion_entity_handle *entity,
                                                                        extrusion_property_type owner_type,
                                                                        extrusion_data_id *field,
                                                                        const void *data,
                                                                        uint32_t byte_size,
                                                                        uint32_t alignment);

/*
Return a direct read-only pointer to one stored data buffer.

The returned pointer is owned by the extrusion entity. The plugin must not free
it or write through it. The pointer remains valid only while the owning entity is
not destroyed, copied over, moved over, or modified in a way that removes or
reallocates this data id. If the plugin needs the bytes after the current API
call, it must copy them into plugin-owned memory.

byte_size_out may be NULL. If it is not NULL, it receives the exact buffer size
in bytes. Returns NULL if entity is NULL or data_id is invalid. A valid stored
buffer of size 0 may also return NULL; use byte_size_out to distinguish that
case if zero-sized buffers are meaningful for the caller.
*/
SLIC3R_HOST_API const void *extrusion_data(const extrusion_entity_handle *entity,
                                           extrusion_data_id data_id,
                                           uint32_t *byte_size_out);

/*
Free one stored data buffer. Properties referencing this id are not modified.

This is different from extrusion_property_remove(): removing a property removes
the typed payload and the stored data owned by that property type; freeing
stored data releases exactly one auxiliary buffer referenced by id.
*/
SLIC3R_HOST_API int32_t extrusion_free_data(extrusion_entity_handle *entity, extrusion_data_id data_id);

/* Built-in property payloads. */

typedef struct c_extrusion_flow {
    double mm3_per_mm;
    float width;
    float height;
} c_extrusion_flow;

/* Property type: EXTRUSION_PROPERTY_TYPE_ATTRIBUTES. */
typedef struct c_extrusion_property_attributes {
    double mm3_per_mm;
    float width;
    float height;
    uint16_t role;
    uint8_t no_seam;
} c_extrusion_property_attributes;

/*
Property type: EXTRUSION_PROPERTY_TYPE_SPEED.

Process values inherited by descendants. The firmware requests them from the
active tool on node entry and restores them on node exit. A value of -1 means
that the corresponding process value is not requested.
*/
typedef struct c_extrusion_property_speed {
    float speed_mm_per_s;
    float accel_mm_per_s2;
    float pressure_adv;
} c_extrusion_property_speed;

/* Exclusive fan-request modes, not combinable flags. Plugins extend this typed
   registry with orchestrator_register_runtime_id(). The host reserves the
   standard names below; custom numeric ids must never be serialized. */
typedef orchestrator_runtime_id fan_request_mode;
#define FAN_REQUEST_MODE_ID_TYPE "slic3r.fan.request_mode"
#define FAN_REQUEST_MODE_UNSPECIFIED ((fan_request_mode)0u)
#define FAN_REQUEST_MODE_AUTOMATIC   ((fan_request_mode)1u)
#define FAN_REQUEST_MODE_OVERRIDE    ((fan_request_mode)2u)
#define FAN_REQUEST_MODE_NAME_UNSPECIFIED "slic3r.unspecified"
#define FAN_REQUEST_MODE_NAME_AUTOMATIC   "slic3r.automatic"
#define FAN_REQUEST_MODE_NAME_OVERRIDE    "slic3r.override"

/* Property type: EXTRUSION_PROPERTY_TYPE_FAN.
   The pair is inherited atomically. Unspecified inherits, even when a local
   payload is present; zero-initialized bytes are therefore neutral. Every
   other registered mode is a set request, including a percentage of zero.
   Active percentages must be finite and in [0,100]. Modes arbitrate policies:
   Automatic is modifiable by any policy, Override by none, and a plugin mode
   only by its owning policy. Registration provides identity, not behavior.
   STEP_GCODE validates registration in the export's orchestrator before the
   finalized tree reaches firmware. Firmware checks active percentages and
   encodes without mode conversion; direct SDK callers supply validated modes. */
typedef struct c_extrusion_property_fan {
    float percent;
    orchestrator_runtime_id mode;
} c_extrusion_property_fan;

/* Property type: EXTRUSION_PROPERTY_TYPE_MODIFIER. */
typedef struct c_extrusion_property_modifier {
    uint8_t enforce_travel;
    uint8_t enforce_retraction;
    uint8_t enforce_unlift;
    uint8_t disable_retraction;
    uint8_t disable_lift;
    uint8_t toolchange_retraction;
} c_extrusion_property_modifier;

typedef enum c_extrusion_custom_gcode_kind {
    /* the property field contains raw gcode */
    C_EXTRUSION_CUSTOM_GCODE_GCODE = 0,
    /* the property field contains raw comment to place after a ";" if verbose */
    C_EXTRUSION_CUSTOM_GCODE_COMMENT = 1,
    /* the property field contains scripted gcode that needs to be executed to get the real gcode */
    C_EXTRUSION_CUSTOM_GCODE_SCRIPT = 2
} c_extrusion_custom_gcode_kind;

/* Property type: EXTRUSION_PROPERTY_TYPE_CUSTOM_GCODE. */
typedef struct c_extrusion_property_custom_gcode {
    c_extrusion_custom_gcode_kind kind;
    gcode_script_type script_type;
    extrusion_data_id text_id;
    /* Serialized Config snapshot containing immutable producer inputs. */
    extrusion_data_id config_id;
    /* Extruder whose runtime E state is exposed, or EXTRUDER_ID_UNKNOWN for the current one. */
    extruder_index_t processing_extruder_id;
} c_extrusion_property_custom_gcode;

/*
Special commands are entry events carried in the extrusion stream, except for
EXTRUSION on a geometric leaf and DEPOSIT on a one-point leaf as described below.
c_extrusion_property_special_command.extra_data stores the numeric parameter
described below; commands without an explicit parameter ignore it.
*/
typedef enum c_extrusion_special_command {
    /* Change to the tool index stored in extra_data. */
    C_EXTRUSION_SPECIAL_COMMAND_TOOLCHANGE = 0,
    /* Set the speed override to extra_data (1.0 = 100%). Marlin backs up the
       prior override with M220 B; other built-in dialects only emit M220 S. */
    C_EXTRUSION_SPECIAL_COMMAND_SAVE_AND_RESET_SPEED_RATIO = 1,
    /* Marlin restores its backed-up speed override with M220 R. Other built-in
       dialects emit M220 S100, discarding the user's prior speed override. */
    C_EXTRUSION_SPECIAL_COMMAND_RESTORE_SPEED_RATIO = 2,
    /* Flush/synchronize the motion planner with a zero-duration dwell. */
    C_EXTRUSION_SPECIAL_COMMAND_FLUSH_PLANNER_QUEUE = 3,
    /* Emit extra_data as a signed filament length in millimeters, with no flow
       or extrusion multiplier. A leaf with segments distributes it over their
       planar lengths (arcs included), at the leaf's XY speed; zero width/height
       omit dimension markers. Zero planar length or a simultaneous extrusion-axis
       request is invalid. Parents and leaves without segments emit E-only on
       entry. The move is raw: the E position and the usage statistics advance,
       but the retraction state is unchanged in both directions. Use an
       EXTRUSION_PROPERTY_TYPE_EXTRUSION_AXIS RETRACT_TO/UNRETRACT request to
       retract or restore filament. */
    C_EXTRUSION_SPECIAL_COMMAND_EXTRUSION = 4,
    /* Emit an E-only retract/deretract move of extra_data millimeters:
       negative extra_data adds retraction, positive extra_data repays
       outstanding retraction. */
    C_EXTRUSION_SPECIAL_COMMAND_RETRACT = 5,
    /* Pause the print for extra_data milliseconds when no custom pause G-code is configured. */
    C_EXTRUSION_SPECIAL_COMMAND_PAUSE = 6,
    /* Wait until the active tool reaches its current target, set by a heater event or by G-code. */
    C_EXTRUSION_SPECIAL_COMMAND_WAIT_FOR_TEMP = 7,
    /* Stop emitting preview/G-code viewer tags for following generated G-code. */
    C_EXTRUSION_SPECIAL_COMMAND_DISABLE_PREVIEW = 8,
    /* Resume emitting preview/G-code viewer tags for following generated G-code. */
    C_EXTRUSION_SPECIAL_COMMAND_ENABLE_PREVIEW = 9,
    /* Set the extruder motor current/trimpot to extra_data where the firmware supports it. */
    C_EXTRUSION_SPECIAL_COMMAND_EXTRUDER_CURRENT = 10,
    /* Deposit finite positive extra_data cubic millimeters before the extrusion
       multiplier at the leaf's single polyline point. The leaf has flags 0,
       printable attributes with positive flow/width/height, a positive speed,
       and no extrusion-axis request. The writer travels to the point, then emits
       E-only at the filament speed of a line with the same attributes. The active
       tool supplies the multiplier, filament area and E mode. Retraction is
       prepared by the plan; the deposit only repays outstanding retraction.
       This command belongs inside PrintingExtrusion roots, not event scopes. */
    C_EXTRUSION_SPECIAL_COMMAND_DEPOSIT = 11
} c_extrusion_special_command;

/* Property type: EXTRUSION_PROPERTY_TYPE_SPECIAL_COMMAND. */
typedef struct c_extrusion_property_special_command {
    c_extrusion_special_command code;
    double extra_data;
} c_extrusion_property_special_command;

/* Heater addressed by c_extrusion_property_heater. */
typedef enum c_extrusion_heater_kind {
    C_EXTRUSION_HEATER_TOOL = 0,
    C_EXTRUSION_HEATER_BED = 1,
    C_EXTRUSION_HEATER_CHAMBER = 2
} c_extrusion_heater_kind;

/*
Property type: EXTRUSION_PROPERTY_TYPE_HEATER.

This is an entry event, like a special command: the firmware applies it once
when entering the node, before that node's custom G-code and special command.
Descendants do not inherit it and nothing is restored when the node is left.
The target remains in place until another heater event or external G-code
changes it. `tool` is used only for C_EXTRUSION_HEATER_TOOL: it selects the
target tool, or EXTRUDER_ID_UNKNOWN for the active tool. In SEMM only the
active tool may be selected because logical tools share one nozzle. The
temperature is a nonnegative target before correction. Tool heaters add their
extruder_temperature_offset; bed and chamber heaters currently use zero offset.
The effective target is limited to [0,2000] degrees Celsius; a corrected sum
of zero or less turns the heater off. A nonzero `wait` makes
the firmware wait for the target. Host-created defaults select the active
tool, use -1 for the not-yet-set temperature, and use zero for `wait`.
*/
typedef struct c_extrusion_property_heater {
    c_extrusion_heater_kind heater;
    extruder_index_t tool;
    int16_t temperature_C;
    uint8_t wait;
} c_extrusion_property_heater;

/* Property type: EXTRUSION_PROPERTY_TYPE_OVERHANG. */
typedef struct c_extrusion_property_overhang {
    float start_distance_from_prev_layer;
    float end_distance_from_prev_layer;
    float proximity_to_curled_lines;
    uint8_t has_full_overhangs_flow;
    uint8_t has_full_overhangs_speed;
    uint8_t has_dynamic_overhangs_flow;
    uint8_t has_dynamic_overhangs_speed;
} c_extrusion_property_overhang;

/* Property type: EXTRUSION_PROPERTY_TYPE_Z_OFFSET. */
typedef struct c_extrusion_property_z_offset {
    coord_t z_offset;
} c_extrusion_property_z_offset;

/*
Bit flags stored in c_extrusion_property_perimeter::perimeter_flags.
*/
typedef enum c_extrusion_perimeter_flag {
    /* No perimeter flag is set; the entity is not tagged as a perimeter loop. */
    C_EXTRUSION_PERIMETER_FLAG_NONE = 0,
    /* Base flag for a regular perimeter loop. */
    C_EXTRUSION_PERIMETER_FLAG_LOOP = 1u << 0,
    /* The loop has no inner contour and is the most internal perimeter contour. */
    C_EXTRUSION_PERIMETER_FLAG_INTERNAL = 1u << 1,
    /* Skirt or brim loop. */
    C_EXTRUSION_PERIMETER_FLAG_SKIRT = 1u << 2,
    /* The loop surrounds a hole instead of an infill/material island. */
    C_EXTRUSION_PERIMETER_FLAG_HOLE = 1u << 3,
    /* The loop should be emitted as a vase/spiralized loop. */
    C_EXTRUSION_PERIMETER_FLAG_VASE = 1u << 4,
    /* The loop has no inner loop, used by seam placement logic. */
    C_EXTRUSION_PERIMETER_FLAG_FIRST_LOOP = 1u << 5,
    /* The loop shouldn't have any seam from here. */
    C_EXTRUSION_PERIMETER_FLAG_NO_SEAM = 1u << 6
} c_extrusion_perimeter_flag;

/* Property type: EXTRUSION_PROPERTY_TYPE_PERIMETER. */
typedef struct c_extrusion_property_perimeter {
    int16_t perimeter_idx;
    int16_t reserved;
    /* Bitmask of c_extrusion_perimeter_flag values. Stored as uint16_t to keep the ABI layout stable. */
    uint16_t perimeter_flags;
} c_extrusion_property_perimeter;

/*
Property type: EXTRUSION_PROPERTY_TYPE_INFILL.

Stored on the root extrusion tree generated for one Surface. Descendant
extrusions inherit it, so post-infill plugins can still recover the Surface
that produced a path after splitting or regrouping the tree.

The payload carries the raw fill recipe of the source Surface, so a
STEP_POST_INFILL plugin can reproduce the legacy Fill::fill_surface_extrusion()
accounting without re-reading the configuration:

- area_to_fill_mm2 is the free-of-perimeter-overlap area in mm2, or 0 when no
  volume correction was requested for this surface.
- height, spacing and spacing_ratio describe the recipe flow. height and
  spacing are scaled coordinates, spacing_ratio is dimensionless.
- density and flow_mult are the recipe values; flow_mult is already baked into
  the mm3_per_mm of the generated leaves.
- surface_type stores the raw_surface_type bits of the source Surface.
- infill_flags stores c_infill_flag bits (steps/slic3r_step_infill.h).

The host stamps the payload at STEP_INFILL publication time. Patterns must
neither read nor set it; "infill.post_process.full_fill_volume" is the built-in
consumer.
*/
typedef struct c_extrusion_property_infill {
    uint64_t source_surface_id;
    double   area_to_fill_mm2;
    coord_t  height;
    coord_t  spacing;
    float    spacing_ratio;
    float    density;
    float    flow_mult;
    uint16_t surface_type;
    uint16_t infill_flags;
} c_extrusion_property_infill;

/*
How EPropertyExtrusionAxis changes the physical extrusion axis.

The property expresses a process request rather than firmware syntax. A
firmware session may therefore encode retract/unretract as explicit E moves or
as native commands such as G10/G11 while keeping the same PrintingPlan. It is
the explicit way to change the retraction state: raw filament moves such as
C_EXTRUSION_SPECIAL_COMMAND_EXTRUSION leave it unchanged.
*/
typedef enum c_extrusion_axis_operation {
    /* No extrusion-axis operation has been configured on this property. */
    C_EXTRUSION_AXIS_OPERATION_NONE = 0,
    /* Retract until the tool reaches the positive target stored in value. */
    C_EXTRUSION_AXIS_OPERATION_RETRACT_TO = 1,
    /* Restore the current retraction and add restart_extra. */
    C_EXTRUSION_AXIS_OPERATION_UNRETRACT = 2
} c_extrusion_axis_operation;

/* Property type: EXTRUSION_PROPERTY_TYPE_EXTRUSION_AXIS. */
typedef struct c_extrusion_property_extrusion_axis {
    c_extrusion_axis_operation operation;
    /* Target retraction length for RETRACT_TO. For an E-only UNRETRACT event,
       zero restores everything plus restart extras; a positive value restores
       at most this many filament millimeters, clamped to outstanding retraction,
       without scheduling or consuming extras. Native G10/G11 cannot perform
       partial unretraction: the partial event emits nothing, and a following
       full UNRETRACT emits G11. Geometric UNRETRACT always restores everything. */
    double value;
    /* Signed adjustment applied by UNRETRACT after restoring the retraction. */
    double restart_extra;
    /* Select the tool-change retraction/restart state instead of the normal one. */
    uint8_t toolchange;
} c_extrusion_property_extrusion_axis;

#ifdef __cplusplus
}
#endif

#endif /* slic3r_extrusion_property_h_ */
