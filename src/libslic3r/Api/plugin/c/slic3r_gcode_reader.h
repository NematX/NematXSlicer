///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_gcode_reader_h_
#define slic3r_gcode_reader_h_

#define SLIC3R_PLUGIN_API_GCODE_READER_MAJOR 2u
#define SLIC3R_PLUGIN_API_GCODE_READER_MINOR 3u

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
G-code reader API
=================

A G-code reader runs synchronously and owns no host resource after run()
returns. The host lends an initial machine state and a sequential byte source.
The reader pulls bounded chunks, publishes normalized action events, then
publishes exactly one complete final state after reaching the end of the input.

The same provider also supports a side-effect-free probe operation. A probe
reads a host-bounded prefix and publishes one confidence score; it receives no
initial state and emits neither actions nor a final state. A firmware does not
probe: it names the exact Reader for its own dialect through its session API.

The call sequence is:

    host                         reader provider
      |-- run(initial_state) ---------->|
      |<--------- get_next_chunk() -----|  repeated until end-of-stream
      |<------------- emit_event() -----|  zero or more times
      |<------- emit_final_state() ------|  exactly once, after end-of-stream
      |<---------------- return ---------|

Published events describe what parsed commands did and identify their physical
line through the mandatory LINE_NUMBER item. Blank or comment-only lines may
advance the final line cursor without an event. The Reader is responsible for
tracking this cursor across chunks, and the host treats the reported line as
authoritative. The host separately indexes line endings for consumers that need
to recover source text. The final state describes only what is known after all
commands.
Consumers which need a timeline use events; consumers which only need to
continue generating G-code use the final state.

Input chunks remain valid until the next get_next_chunk() call or the end of
run(), whichever comes first. Event and final-state data remain valid only for
the receiving callback. Either side must copy borrowed data that it needs to
retain beyond those boundaries.

All structures in one API major have a fixed layout. An incompatible layout
change requires incrementing SLIC3R_PLUGIN_API_GCODE_READER_MAJOR; callers and
providers must therefore use structures from the negotiated major version.
*/

typedef enum raw_gcode_reader_status {
    RAW_GCODE_READER_STATUS_UNSET = 0,
    RAW_GCODE_READER_STATUS_SUCCESS,
    RAW_GCODE_READER_STATUS_INVALID_ARGUMENT,
    RAW_GCODE_READER_STATUS_ERROR
} raw_gcode_reader_status;

/* Result channel used by every reader callback. The callback caller initializes
   status to UNSET and must inspect it immediately after the callback returns.
   The callee sets status and may provide an error string borrowed until the
   next callback on the same context. SUCCESS is the only successful status;
   UNSET therefore detects callbacks which returned without reporting a result. */
typedef struct raw_gcode_reader_result {
    raw_gcode_reader_status status;
    const char *error_message;
} raw_gcode_reader_result;

/* Semantic category of one parsed command. It is deliberately separate from
   state items: G1 X10 and G92 X10 may produce the same final X value but have
   different viewer semantics. INVALIDATE means that listed values are no
   longer trustworthy; it does not assign their accompanying numeric values. */
typedef enum raw_gcode_reader_event_kind {
    RAW_GCODE_READER_EVENT_STATE_UPDATE = 1,
    RAW_GCODE_READER_EVENT_LINEAR_MOVE,
    RAW_GCODE_READER_EVENT_ARC_MOVE,
    RAW_GCODE_READER_EVENT_SET_POSITION,
    RAW_GCODE_READER_EVENT_SET_ORIGIN,
    RAW_GCODE_READER_EVENT_SET_TEMPERATURE,
    RAW_GCODE_READER_EVENT_WAIT_TEMPERATURE,
    RAW_GCODE_READER_EVENT_MARKER,
    RAW_GCODE_READER_EVENT_DIAGNOSTIC,
    RAW_GCODE_READER_EVENT_INVALIDATE
} raw_gcode_reader_event_kind;

/* Each item changes or describes one scalar value. target is interpreted by
   type: axis and global modal items ignore it; tool items target a logical
   extruder; fan and heater mapping items target their physical device. Every
   integral, boolean, enum and mask value is transported as an exactly
   representable double. Duplicate (type, target) pairs are invalid. */
typedef struct raw_gcode_reader_event_item {
    uint32_t type;
    uint32_t target;
    double value;
} raw_gcode_reader_event_item;

/* Point-in-time annotations consumed by timeline builders. Unlike machine
   state, a marker applies only to the event which carries it and is never
   retained in the Reader's final state. */
typedef enum raw_gcode_reader_marker_type {
    RAW_GCODE_READER_MARKER_SEAM = 1,
    RAW_GCODE_READER_MARKER_COLOR_CHANGE,
    RAW_GCODE_READER_MARKER_PAUSE_PRINT,
    RAW_GCODE_READER_MARKER_CUSTOM_GCODE
} raw_gcode_reader_marker_type;

/* Stable reasons for non-fatal Reader diagnostics. The host owns their
   localized wording and severity; plugins only identify the condition. A
   fatal parsing failure is still returned through raw_gcode_reader_status. */
typedef enum raw_gcode_reader_diagnostic_type {
    RAW_GCODE_READER_DIAGNOSTIC_UNKNOWN_COMMAND = 1,
    RAW_GCODE_READER_DIAGNOSTIC_INVALID_COMMAND_PARAMETERS,
    RAW_GCODE_READER_DIAGNOSTIC_UNKNOWN_TOOL,
    RAW_GCODE_READER_DIAGNOSTIC_AMBIGUOUS_TOOL,
    RAW_GCODE_READER_DIAGNOSTIC_UNSUPPORTED_STATE_EFFECT
} raw_gcode_reader_diagnostic_type;

typedef enum raw_gcode_reader_item_type {
    RAW_GCODE_READER_ITEM_X = 1,
    RAW_GCODE_READER_ITEM_Y,
    RAW_GCODE_READER_ITEM_Z,
    RAW_GCODE_READER_ITEM_E,
    RAW_GCODE_READER_ITEM_FEEDRATE,
    RAW_GCODE_READER_ITEM_UNITS_MM,
    RAW_GCODE_READER_ITEM_XYZ_ABSOLUTE,
    RAW_GCODE_READER_ITEM_E_ABSOLUTE,
    RAW_GCODE_READER_ITEM_VOLUMETRIC,
    RAW_GCODE_READER_ITEM_ACTIVE_TOOL,
    RAW_GCODE_READER_ITEM_TOOL_NOZZLE,
    RAW_GCODE_READER_ITEM_TOOL_FAN,
    RAW_GCODE_READER_ITEM_TOOL_E_AXIS,
    RAW_GCODE_READER_ITEM_TOOL_FILAMENT_DIAMETER,
    RAW_GCODE_READER_ITEM_TOOL_OFFSET_X,
    RAW_GCODE_READER_ITEM_TOOL_OFFSET_Y,
    RAW_GCODE_READER_ITEM_TOOL_OFFSET_Z,
    RAW_GCODE_READER_ITEM_TOOL_TEMPERATURE,
    RAW_GCODE_READER_ITEM_BED_TEMPERATURE,
    RAW_GCODE_READER_ITEM_CHAMBER_TEMPERATURE,
    RAW_GCODE_READER_ITEM_FAN_SPEED,
    RAW_GCODE_READER_ITEM_SPEED_FACTOR,
    RAW_GCODE_READER_ITEM_EXTRUSION_FACTOR,
    RAW_GCODE_READER_ITEM_ACCELERATION,
    RAW_GCODE_READER_ITEM_EXTRUSION_ROLE,
    RAW_GCODE_READER_ITEM_WIDTH,
    RAW_GCODE_READER_ITEM_HEIGHT,
    RAW_GCODE_READER_ITEM_LAYER,
    /* Zero-based logical object associated with subsequent movements.
       UINT32_MAX means that the movement is outside every printable object.
       A Reader created with a Print uses the same plan-local IDs as its
       PrintingPlan. A Reader for an external file may allocate IDs in
       first-seen order, but object display names require separate metadata
       that this numeric contract does not currently transport. */
    RAW_GCODE_READER_ITEM_OBJECT,
    /* Event-local raw_gcode_reader_marker_type. A MARKER event contains
       exactly one such item with target zero; no other event or complete
       machine state may contain it. */
    RAW_GCODE_READER_ITEM_MARKER,
    RAW_GCODE_READER_ITEM_ARC_CENTER_X,
    RAW_GCODE_READER_ITEM_ARC_CENTER_Y,
    RAW_GCODE_READER_ITEM_ARC_CENTER_Z,
    RAW_GCODE_READER_ITEM_ARC_PLANE,
    RAW_GCODE_READER_ITEM_ARC_CLOCKWISE,
    RAW_GCODE_READER_ITEM_ARC_TURNS,
    /* Waiting is scoped to a heater class. Separate item types avoid
       conflating tool zero with the bed, whose target is also conventionally
       zero. The value is the reached setpoint, not a sensor measurement. */
    RAW_GCODE_READER_ITEM_TOOL_TEMPERATURE_REACHED,
    RAW_GCODE_READER_ITEM_BED_TEMPERATURE_REACHED,
    RAW_GCODE_READER_ITEM_CHAMBER_TEMPERATURE_REACHED,
    /* Zero-based number of the line currently being parsed. It is mandatory
       in every event and state, uses target zero and is an exact uint32_t. */
    RAW_GCODE_READER_ITEM_LINE_NUMBER,
    /* Numeric identity of the Reader-specific state carried alongside the
       portable items. Zero permanently identifies the public portable schema,
       which carries no provider-private fields. Missing values are normalized
       to that portable identity by the C++ tracker. The target is always zero. */
    RAW_GCODE_READER_ITEM_READER_ID,
    /* Event-local raw_gcode_reader_diagnostic_type. A DIAGNOSTIC event
       contains exactly one such item with target zero. It is forbidden in
       every other event and in a complete machine state. */
    RAW_GCODE_READER_ITEM_DIAGNOSTIC_TYPE,
    /* Greatest height in millimetres published for the current LAYER. The target
       is always zero. The complete state retains the last published value until
       replaced or invalidated; consumers associate each publication with its
       current LAYER and must not assign it to later layers. Without it, consumers
       may use extrusion HEIGHTs. */
    RAW_GCODE_READER_ITEM_MAX_LAYER_HEIGHT
} raw_gcode_reader_item_type;

/* Registry type used to intern private Reader-state schema names. */
#define GCODE_READER_STATE_ID_TYPE "gcode_reader_state"
/* Static schema containing only the portable raw_gcode_reader_item_type data. */
#define GCODE_READER_STATE_ID_PORTABLE ((uint32_t)0u)

/* Provider-private scalar items use this range. The generic host validates
   only that their transported double is finite, then copies them without
   interpreting target or value. Their meaning belongs to the explicitly
   selected Reader; callers must not transfer them to an unrelated Reader. */
#define RAW_GCODE_READER_ITEM_CUSTOM_BEGIN ((uint32_t)0x80000000u)

/* One normalized action. items contains the state delta associated with the
   action and must include LINE_NUMBER. The Reader supplies the authoritative
   source line; several events may identify the same line. The structure and
   item array are borrowed only during emit_event(); the host validates their
   shape and copies anything it keeps before returning. */
typedef struct raw_gcode_reader_event {
    uint32_t kind;
    const raw_gcode_reader_event_item *items;
    uint32_t item_count;
} raw_gcode_reader_event;

/* Complete set of values currently known by a reader. An absent (type,target)
   pair means unknown; it is not equivalent to a zero value. The same type is
   used for the host-provided initial state and the final callback. */
typedef struct raw_gcode_reader_state {
    const raw_gcode_reader_event_item *items;
    uint32_t item_count;
} raw_gcode_reader_state;

/* One borrowed, non-empty input fragment. On successful end-of-stream the host
   returns bytes == NULL and byte_count == 0. Every other zero-sized or null
   combination is invalid. bytes remains valid until the next chunk request or
   until run() returns. */
typedef struct raw_gcode_reader_chunk {
    const char *bytes;
    uint64_t byte_count;
} raw_gcode_reader_chunk;

/* Host callbacks carried by run_ctx_gcode_reader.

   get_next_chunk() is a pull source. The provider must finish using one chunk
   before requesting another because the host may reuse its input buffer.

   emit_event() validates and consumes one action synchronously. The provider
   may reuse or destroy the event and item array as soon as the call returns.

   emit_final_state() transfers no ownership: the host copies the complete
   state synchronously. It may be called exactly once and only after
   get_next_chunk() reported end-of-stream.

   All three functions report errors through raw_gcode_reader_result. No
   callback may let an exception cross the C ABI boundary. */
typedef void (*gcode_reader_get_next_chunk_fn)(void *context,
                                                raw_gcode_reader_chunk *chunk,
                                                raw_gcode_reader_result *result);
typedef void (*gcode_reader_emit_event_fn)(void *context,
                                            const raw_gcode_reader_event *event,
                                            raw_gcode_reader_result *result);
typedef void (*gcode_reader_emit_final_state_fn)(void *context,
                                                  const raw_gcode_reader_state *state,
                                                  raw_gcode_reader_result *result);

/* Deliver the confidence produced by one detection probe. Zero means that the
   provider found no identifying evidence; 100 means an exact dialect marker.
   The host rejects values above 100. A probe publishes exactly one score. */
typedef void (*gcode_reader_emit_probe_fn)(void *context,
                                           uint32_t confidence,
                                           raw_gcode_reader_result *result);

#ifdef __cplusplus
}
#endif

#endif // slic3r_gcode_reader_h_
