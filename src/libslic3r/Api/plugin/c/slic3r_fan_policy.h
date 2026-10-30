///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#ifndef slic3r_fan_policy_h_
#define slic3r_fan_policy_h_

#define SLIC3R_PLUGIN_API_FAN_POLICY_MAJOR 2u
#define SLIC3R_PLUGIN_API_FAN_POLICY_MINOR 0u

#include <stdint.h>

/* Versioned, pointer-free context on PrintingToolGroup::properties(). This
   header defines only bytes, not registration or tree/Config operations. Every
   execution rebuilds references before downstream policies read them. */
#define FAN_POLICY_CONTEXT_PROPERTY_NAME "slic3r.printing_tool.fan_context"

typedef enum raw_fan_role {
    RAW_FAN_ROLE_DEFAULT = 0,
    RAW_FAN_ROLE_PERIMETER = 1,
    RAW_FAN_ROLE_EXTERNAL_PERIMETER = 2,
    RAW_FAN_ROLE_OVERHANG_PERIMETER = 3,
    RAW_FAN_ROLE_INTERNAL_INFILL = 4,
    RAW_FAN_ROLE_INTERNAL_BRIDGE_INFILL = 5,
    RAW_FAN_ROLE_SOLID_INFILL = 6,
    RAW_FAN_ROLE_TOP_SOLID_INFILL = 7,
    RAW_FAN_ROLE_IRONING = 8,
    RAW_FAN_ROLE_BRIDGE_INFILL = 9,
    RAW_FAN_ROLE_THIN_WALL = 10,
    RAW_FAN_ROLE_GAP_FILL = 11,
    RAW_FAN_ROLE_SKIRT = 12,
    RAW_FAN_ROLE_SUPPORT_MATERIAL = 13,
    RAW_FAN_ROLE_SUPPORT_MATERIAL_INTERFACE = 14,
    RAW_FAN_ROLE_COUNT = 15
} raw_fan_role;

typedef enum raw_fan_policy_phase {
    RAW_FAN_POLICY_PHASE_BASE = 1,
    RAW_FAN_POLICY_PHASE_TIME = 2,
    RAW_FAN_POLICY_PHASE_CLOSED = 3
} raw_fan_policy_phase;

/* Negative percentages mean a disabled/absent reference, not an explicit stop.
   configured_percent preserves normalized filament input through all phases.
   Early family/default fallbacks have already been applied to percent in BASE.
   Late fallback rules belong to the policy implementation, not this record. */
typedef struct c_fan_role_reference {
    int32_t configured_percent;
    int32_t percent;
} c_fan_role_reference;

/* struct_size must equal sizeof(c_fan_policy_context); zero-filled storage is
   not a published context. Phase transitions are BASE -> TIME -> CLOSED.
   References are computed even if every extrusion has a protected request.
   Bounds apply to automatic dynamic requests, never to Override/plugin modes.
   Configuration and the layer ordinal are inputs to kernels, not results. */
typedef struct c_fan_policy_context {
    uint32_t struct_size;
    uint32_t phase;
    c_fan_role_reference roles[15]; /* RAW_FAN_ROLE_COUNT, fixed ABI extent */
    int32_t dynamic_min;
    int32_t dynamic_max;
} c_fan_policy_context;

#endif
