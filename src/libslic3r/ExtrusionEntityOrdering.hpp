///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/* Shared classification and event checks for choosing a contour's cyclic start.
   is_seam_unit checks permission and exact stored geometry, not legacy types.
   can_choose_child_seam carries ancestor restrictions through nested wrappers;
   has_seam_barrier identifies direct events that prevent child reordering.
   has_one_shot_extrusion_event recognizes commands and fixed extrusion quantities
   that must stay where they are. These queries borrow entities
   without changing geometry, permissions or event order:
       const bool eligible = is_seam_unit(entity);
       const bool child_context = can_choose_child_seam(parent, child, ancestors_allow_seams);
   The child must belong directly to parent; pass child_context on descent.
   Eligibility describes the node, not permission to move surrounding events. */
#ifndef slic3r_ExtrusionEntityOrdering_hpp_
#define slic3r_ExtrusionEntityOrdering_hpp_

#include "ExtrusionEntity.hpp"
#include "Api/internal/ExtrusionPropertyAccess.hpp"

namespace Slic3r {

// Read explicit permission and exact stored XY closure without dispatching to legacy loop classifiers.
inline bool is_seam_unit(const ExtrusionEntity &entity)
{
    return entity.can_rotate() && entity.ExtrusionEntity::is_loop();
}

// A child must be directly owned by parent. Carry a blocked ancestor context through nested wrappers.
inline bool can_choose_child_seam(const ExtrusionEntity &parent, const ExtrusionEntity &child,
                                  const bool ancestors_allow_seams)
{
    if (!ancestors_allow_seams)
        return false;
    if (!parent.is_continuous())
        return true;

    const ExtrusionEntity *only_geometry = nullptr;
    bool has_empty_sibling = false;
    for (const ExtrusionEntityUPtr &candidate : parent.children()) {
        assert(candidate);
        if (!candidate || candidate->ExtrusionEntity::empty()) {
            has_empty_sibling = true;
        } else {
            if (only_geometry != nullptr)
                return false;
            only_geometry = candidate.get();
        }
    }
    // A sole contour shares its parent's entry unless a rotatable parent delegates,
    // or empty siblings identify a structural wrapper whose events stay in place.
    return only_geometry == &child && (is_seam_unit(parent) || has_empty_sibling);
}

// Return whether the entity directly carries a command or fixed extrusion quantity that must run exactly once.
inline bool has_one_shot_extrusion_event(const ExtrusionEntity &entity)
{
    using Access = ApiInternal::ExtrusionPropertyAccess;
    return Access::has_property(entity, EXTRUSION_PROPERTY_TYPE_CUSTOM_GCODE) ||
           Access::has_property(entity, EXTRUSION_PROPERTY_TYPE_SPECIAL_COMMAND) ||
           Access::has_property(entity, EXTRUSION_PROPERTY_TYPE_HEATER) ||
           Access::has_property(entity, EXTRUSION_PROPERTY_TYPE_EXTRUSION_AXIS);
}

// Reordering direct children must not move an empty property event or cross a one-shot child.
inline bool has_seam_barrier(const ExtrusionEntity &entity)
{
    for (const ExtrusionEntityUPtr &child : entity.children()) {
        assert(child);
        if (child && (has_one_shot_extrusion_event(*child) ||
            (child->ExtrusionEntity::empty() && child->has_properties())))
            return true;
    }
    return false;
}

} // namespace Slic3r

#endif // slic3r_ExtrusionEntityOrdering_hpp_
