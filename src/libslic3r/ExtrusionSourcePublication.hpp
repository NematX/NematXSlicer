///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_ExtrusionSourcePublication_hpp_
#define slic3r_ExtrusionSourcePublication_hpp_

#include "ExtrusionEntity.hpp"

/* Transfer temporary generator output into source extrusion buckets:
       append_source_extrusions(destination, generated);
   Only property-free transport roots donate children. Commands and property
   scopes remain whole, so descendants keep inherited metadata. Source handles
   stay valid but become empty after transfer. This is not canonical plan ingress;
   native producer types and their child handles need not be replaced here.
   The caller exclusively owns both distinct trees and provides a
   destination with Children storage, whether generic or a compatibility shell.
   Destination capacity is reserved before source consumption.
*/

namespace Slic3r {

// Move generated content under exclusive access, retaining semantic root scopes.
// Reserve publication storage before consumption; source handles remain valid
// but become empty. Native producer representation is not normalized here.
inline void append_source_extrusions(ExtrusionEntity &destination, ExtrusionEntity &source)
{
    if (source.is_nop() && !source.has_properties())
        return;
    if (source.is_leaf() || source.has_properties()) {
        destination.children().reserve(destination.child_count() + 1);
        destination.append_child(ExtrusionEntityUPtr(source.clone_move()));
        source.clear_content();
        source.clear_properties();
        return;
    }
    ExtrusionEntity::Children &children = source.children();
    destination.children().reserve(destination.child_count() + children.size());
    for (ExtrusionEntityUPtr &child : children)
        destination.append_child(std::move(child));
    children.clear();
}

} // namespace Slic3r

#endif // slic3r_ExtrusionSourcePublication_hpp_
