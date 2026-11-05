///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#ifndef plugins_support_common_extrusion_lines_hpp_
#define plugins_support_common_extrusion_lines_hpp_

#include <cstdint>
#include <vector>

#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"

namespace slic3r_api::Support::Common {

// Appends to out, in pre-order, the points of every entity of the tree that carries a polyline: the polyline of an entity, then
// those of its children in order. out is not cleared and the tree is not modified; the points are copies.
inline void collect_leaf_polylines(const ExtrusionEntity &entity, std::vector<std::vector<c_point>> &out)
{
    if (entity.has_polyline())
        out.push_back(entity.points());
    for (uint32_t idx = 0; idx < entity.child_count(); ++idx)
        collect_leaf_polylines(entity.child(idx), out);
}

} // namespace slic3r_api::Support::Common

#endif // plugins_support_common_extrusion_lines_hpp_
