///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/* Shared classification for choosing a contour's cyclic starting point.
   Query is_seam_unit before planning a seam; a true permission alone does not
   prove that the stored geometry is a nonempty continuous closed contour.
   The query borrows the entity without changing geometry or permissions:
       const bool eligible = is_seam_unit(entity);
   Eligibility describes the node, not permission to move surrounding events. */
#ifndef slic3r_ExtrusionEntityOrdering_hpp_
#define slic3r_ExtrusionEntityOrdering_hpp_

#include "ExtrusionEntity.hpp"

namespace Slic3r {

// Read explicit permission and exact stored XY closure without dispatching to legacy loop classifiers.
inline bool is_seam_unit(const ExtrusionEntity &entity)
{
    return entity.can_rotate() && entity.ExtrusionEntity::is_loop();
}

} // namespace Slic3r

#endif // slic3r_ExtrusionEntityOrdering_hpp_
