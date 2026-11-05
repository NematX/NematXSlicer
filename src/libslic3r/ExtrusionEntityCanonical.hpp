///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_ExtrusionEntityCanonical_hpp_
#define slic3r_ExtrusionEntityCanonical_hpp_

/* Internal helpers for transferring generated trees into newly owned plan storage.
   Reserve publication capacity, then acquire the source:
       ExtrusionEntityUPtr owned = CanonicalExtrusion::acquire(source);
   is_tree checks exact base nodes, including commands without a polyline. Such
   trees move without replacing child handles; legacy trees are copied before
   their source content is consumed. The caller excludes concurrent mutation.
   Allocation failure leaves the source intact. A successful transfer clears its
   content, properties and detached resources, without replacing its outer handle.
   Generic SDK moves do not use this boundary and retain legacy representations.
*/

#include <typeinfo>

#include "ExtrusionEntity.hpp"

namespace Slic3r { namespace CanonicalExtrusion {

// Inspect borrowed storage without allocating or caching type state.
inline bool is_tree(const ExtrusionEntity &entity)
{
    if (typeid(entity) != typeid(ExtrusionEntity))
        return false;
    for (const ExtrusionEntityUPtr &child : entity.children())
        if (child && !is_tree(*child))
            return false;
    return true;
}

// Consume only after a fresh destination is prepared. Clear detached resources
// as well as properties. Permissions and identity follow the existing moved-from
// contract; no handle is replaced. Only properties define active command state.
inline void clear_transferred_source(ExtrusionEntity &source)
{
    source.clear_content();
    static_cast<ExtrusionPropertyContainer &>(source) = ExtrusionPropertyContainer();
}

// Acquire NEW owned plan storage, never normalize an existing live tree in place.
// Callers reserve publication capacity first. Conforming inputs move their child
// addresses unchanged; legacy inputs are independently prepared before any source
// consumption. Allocation failure leaves the source intact. No generic SDK move
// may call this helper, since those retain their legacy representation contract.
inline ExtrusionEntityUPtr acquire(ExtrusionEntity &source)
{
    ExtrusionEntityUPtr prepared;
    if (is_tree(source)) {
        prepared = std::make_unique<ExtrusionEntity>(std::move(source));
    } else {
        prepared = source.clone_canonical();
    }
    clear_transferred_source(source);
    return prepared;
}

}} // namespace Slic3r::CanonicalExtrusion

#endif // slic3r_ExtrusionEntityCanonical_hpp_
