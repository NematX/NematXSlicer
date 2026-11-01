///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_Printing_PrintingPlanFacts_hpp_
#define slic3r_Printing_PrintingPlanFacts_hpp_

#include <cstdint>
#include <vector>

#include "libslic3r/libslic3r.h"
#include "libslic3r/Point.hpp"

/*
Facts about a finalized PrintingPlan
====================================

The plan is the single source of truth for what is printed: layer count, which
tools are used, the first tool and the planned tool transitions. This module
reads a finalized plan (and the sliced Print for the object layer count) and
returns plain values, so the custom G-code variables and the print statistics
are computed by the same code and cannot disagree. These plan facts are
independent of the generated text: the measured values (volumes, weights, costs,
tower share, support flag) come from the Reader result and are not computed here.
*/

namespace Slic3r {

class Print;

namespace Printing {

struct PrintingPlan;

// Layer and tool facts extracted from a plan; holds no reference to it.
struct PlanFacts
{
    // First concrete tool of the plan; 0 when the plan has no concrete tool visit.
    extruder_index_t initial_tool = 0;
    // Changes between consecutive concrete tool visits, in plan order.
    int32_t toolchanges = 0;
    // Number of PrintingLayerGroup, auxiliary layers included.
    int32_t layer_count = 0;
    // Highest layer print Z of the plan (scaled), 0 without layers. It is the layer's own print Z: the Z offset of the
    // printer is not added, as in the historical max_layer_z.
    coord_t max_print_z = 0;
    // Tools having at least one concrete visit, in ascending order, without duplicates.
    std::vector<extruder_index_t> used_tools;
};

// Read a possibly null finalized plan and return its layer and tool facts; a null plan gives default facts.
PlanFacts summarize_plan(const PrintingPlan *plan);

// Count the distinct object layer Z values of a sliced Print, auxiliary layers excluded.
int32_t object_layer_count(const Print &print);

} // namespace Printing
} // namespace Slic3r

#endif // slic3r_Printing_PrintingPlanFacts_hpp_
