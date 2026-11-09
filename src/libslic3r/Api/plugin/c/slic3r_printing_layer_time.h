///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#ifndef slic3r_printing_layer_time_h_
#define slic3r_printing_layer_time_h_

#define SLIC3R_PLUGIN_API_PRINTING_LAYER_TIME_MAJOR 1u
#define SLIC3R_PLUGIN_API_PRINTING_LAYER_TIME_MINOR 0u

#include <stdint.h>

/* Payload on PrintingLayerGroup::properties(). Register its stable name through
   orchestrator_register_property before processing. Numeric property IDs are
   orchestrator-local. This preserves the former private timing layout/name. */
#define PRINTING_LAYER_TIME_PROPERTY_NAME "slic3r.printing_layer.time"

typedef enum raw_printing_layer_time_origin {
    RAW_PRINTING_LAYER_TIME_ORIGIN_ESTIMATED = 0,
    /* The timing model was run after speed/slowdown changes. Not a measured
       machine duration and not enough to reconstruct intra-movement times. */
    RAW_PRINTING_LAYER_TIME_ORIGIN_RESOLVED = 1
} raw_printing_layer_time_origin;

/* Total duration of the layer, shared by all its tool sections. Producers must
   republish after speed changes; ESTIMATED may be replaced by a fresh estimate.
   duration_seconds must be finite and nonnegative for either origin. */
typedef struct c_printing_layer_time_property {
    double duration_seconds;
    raw_printing_layer_time_origin origin;
} c_printing_layer_time_property;

#endif
