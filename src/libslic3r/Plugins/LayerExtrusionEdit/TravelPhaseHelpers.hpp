///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_Plugins_LayerExtrusionEdit_TravelPhaseHelpers_hpp_
#define slic3r_Plugins_LayerExtrusionEdit_TravelPhaseHelpers_hpp_

/*
Ordered travel-phase leaves
===========================

A travel provider materializes a scope's incoming travel as one segmented leaf,
but later providers may split it into an ordered subtree, for example to give
its final part another acceleration. Consumers which inspect or reshape the
whole travel read it through collect_travel_leaves() instead of assuming a
single leaf.

The collected leaves keep machine order and form one continuous path: each
leaf starts within SCALED_EPSILON of the previous end in XY and absolute Z.
Views are borrowed; a caller restructuring a leaf must refresh the views it
replaces.
*/

#include <vector>

#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"

namespace slic3r_api { namespace LayerExtrusionEdit { namespace TravelPhase {

/* One segmented leaf of a travel phase with its effective role and Z offset. */
struct TravelLeaf
{
    MutableExtrusionEntity entity;
    raw_extrusion_role role = RAW_EXTRUSION_ROLE_NONE;
    // Nearest ZOffset property on the leaf or its ancestors, or the inherited scope offset.
    coord_t z_offset = 0;
};

/* Collect the segmented leaves of a travel phase in machine order, starting from the scope's
   effective Z offset. Segmentless leaves are ignored. Returns false with no leaves when a
   segmented leaf has no role attributes or also has children, when a deposit is present, or
   when consecutive leaves are not continuous; a phase without geometry returns true and no leaves. */
bool collect_travel_leaves(MutableExtrusionEntity phase, coord_t inherited_z_offset,
                           std::vector<TravelLeaf> &leaves);

/* Return the true planar length of one line or circular arc; an impossible arc radius throws. */
distf_t segment_planar_length(const c_extrusion_segment &segment);

}}} // namespace slic3r_api::LayerExtrusionEdit::TravelPhase

#endif // slic3r_Plugins_LayerExtrusionEdit_TravelPhaseHelpers_hpp_
