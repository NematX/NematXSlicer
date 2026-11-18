///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

#ifndef slic3r_bridge_h_
#define slic3r_bridge_h_

/* Define the property payloads used to describe bridge surfaces, extrusion flow,
   connected path pieces and the completion of support calculations.

   Attach c_bridge_surface_property to a Surface to identify its bridge group
   and the rule for interpreting its prepared area. The printing direction is
   stored on the Surface itself; read it with surface_get_bridge_angle().

   c_bridge_volume_property describes one extrusion piece: its incoming flow,
   assigned flow and the confidence of the correction. A strand is one continuous
   path that may contain several such pieces. Put c_bridge_strand_property on
   their containing collection when ordering must keep them in sequence and
   reverse them only together.

   c_support_pipeline_property is object-level state for the host support steps.
   Its flags distinguish an unfinished calculation from one that completed with
   an empty result. A support request alone is not evidence of printed material.
   These payload definitions do not themselves run any of those calculations.

   The structs own no pointers or geometry and can be copied as fixed-size values.
   Initialize reserved fields to zero. Field names ending in _mm or _mm2 use
   millimetres or square millimetres, not scaled coordinates. Interpret each
   measurement together with its correction and confidence states below.
*/
#include "slic3r_extrusion_property.h"

/* Correction was calculated for this piece; its flow may already fit unchanged. */
#define BRIDGE_CORRECTION_APPLIED 1u
/* No reliable correction could be calculated; keep the incoming flow and dimensions. */
#define BRIDGE_CORRECTION_KEPT_NOMINAL_UNKNOWN 2u
/* The chosen cross section fits the measured gap within geometry/flow precision. */
#define BRIDGE_CONFIDENCE_CERTIFIED 1u
/* Even the minimum printable thread height exceeds the gap; overlap remains. */
#define BRIDGE_CONFIDENCE_FLOOR_LIMITED 2u
/* A fit cannot be guaranteed; keep the original flow without claiming that it fits. */
#define BRIDGE_CONFIDENCE_UNKNOWN 3u
/* Fits a mean volume budget inferred from surfaces and infill densities, not a
   measured local gap. available_height_mm is the equivalent total free height. */
#define BRIDGE_CONFIDENCE_DENSITY_ESTIMATE 4u
/* The printable height floor exceeds that estimated volume budget. */
#define BRIDGE_CONFIDENCE_DENSITY_FLOOR_LIMITED 5u

/* Describe a surface with a chosen bridge direction and prepared fill area.
   core_rule 1 identifies an external bridge: subtract the linked lower-island
   slices from the prepared area to reconstruct its original unsupported core.
   With core_rule 2, the entire prepared internal bridge is the working core;
   sparse/void support lies inside the lower model slice. Its area is not
   extended: with extension_requested, its threads may leave it at their ends,
   where infill.post_process.internal_bridge_anchors extends them after infill
   generation. Read the printing direction with surface_get_bridge_angle(). */
typedef struct c_bridge_surface_property {
    /* Payload schema; the supported value is 1. */
    uint32_t schema_version;
    /* Geometry reconstruction rule; supported values are 1 and 2, defined above. */
    uint32_t core_rule;
    /* Identify one bridge group within this object, including all its fragments. */
    uint64_t logical_key;
    /* Hash both margin values used to build this bridge; compare it when
       checking whether the inputs to volume correction have changed. */
    uint64_t policy_signature;
    /* 1 for an enabled user angle override, otherwise 0 for detection. */
    uint8_t angle_from_user;
    /* 1 for a positive requested margin, or for internal_bridge_expansion on an
       internal bridge; not proof that anchors were obtained. */
    uint8_t extension_requested;
    uint8_t reserved[6];
} c_bridge_surface_property;

/* Record the result of volume correction on each extrusion piece. nominal holds
   the incoming flow, before correction. A strand is one continuous bridge path;
   pieces may need different flow along that path. Do not put one result on the
   containing strand: an unknown piece must keep its own KEPT_NOMINAL_UNKNOWN state. */
typedef struct c_bridge_volume_property {
    /* Identify the source bridge group that produced this piece. */
    uint64_t logical_key;
    /* Hash of the correction inputs. If it changes, regenerate the uncorrected
       path first rather than correcting the already changed flow again. */
    uint64_t signature;
    /* Hash of this piece's geometry after splitting and endpoint rounding. */
    uint64_t geometry_signature;
    /* Incoming attributes before correction: lengths in mm, area in mm3/mm. */
    c_extrusion_property_attributes nominal;
    /* Smallest vertical gap, or total equivalent free height for DENSITY_* states.
       +infinity means no geometric obstacle. UNKNOWN is not a reliable measurement. */
    double available_height_mm;
    /* Largest section allowed by the gap or mean volume budget, ignoring the
       printable height floor. UNKNOWN means no reliable limit was established. */
    double area_cap_mm2;
    /* Actual area and height assigned to this piece's extrusion attributes. */
    double effective_area_mm2;
    float effective_height_mm;
    /* One of BRIDGE_CORRECTION_*; APPLIED does not imply a volume reduction. */
    uint8_t correction_state;
    /* One of BRIDGE_CONFIDENCE_*; interpret the cap only with this state. */
    uint8_t confidence;
    uint8_t reserved[2];
} c_bridge_volume_property;

/* Keep the pieces of one bridge path in a fixed sequence so they remain connected.
   If reversal is permitted, ordering must reverse the whole path, not each piece. */
typedef struct c_bridge_strand_property {
    /* 1 means all pieces must share one whole-path direction; 0 leaves it unmarked. */
    uint32_t atomic_orientation;
    uint32_t reserved;
} c_bridge_strand_property;

/* Stored on the print object and maintained by the host support steps. Each flag
   is 0 or 1; completion may describe an empty result. Clearing the relevant step
   resets its completion state. A requested area is never proof of printed support. */
typedef struct c_support_pipeline_property {
    /* The areas requiring support were calculated, even if no area needs support. */
    uint32_t demand_computed;
    /* The selected support algorithm and required path generators finished, or
       no support material was expected. Finished calculations may produce no paths. */
    uint32_t material_generated;
    /* The algorithm selected for this object's support style finished its surfaces. */
    uint32_t surfaces_generated;
} c_support_pipeline_property;

#endif
