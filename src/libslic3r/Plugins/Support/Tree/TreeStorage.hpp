///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* Native access to the plugin polygon collections of the tree family.

   Temporary bridge: only the slab drawing of TreeSupportGenerator.cpp and two test fixtures still use it, and this file is to be
   deleted once they no longer do. A StoredPolygonCollection keeps native Polygons in its storage, so these helpers read and write
   those Polygons directly, and the plugin API sees the same polygons afterwards:

       native_polygons(slab.polygons) = union_(native_polygons(slab.polygons)); */
#ifndef plugins_support_tree_storage_hpp_
#define plugins_support_tree_storage_hpp_

#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Polygon.hpp"

namespace Slic3r::TreeFamily {
// The native polygons that a plugin polygon collection holds in its storage, for reading and writing.
inline Polygons &native_polygons(slic3r_api::StoredPolygonCollection &collection)
{
    return *reinterpret_cast<Polygons *>(collection.mutable_handle());
}

// The native polygons that a const plugin polygon collection holds in its storage, for reading.
inline const Polygons &native_polygons(const slic3r_api::StoredPolygonCollection &collection)
{
    return *reinterpret_cast<const Polygons *>(collection.handle());
}

// The native expolygons that a const plugin expolygon collection holds in its storage, for reading.
inline const ExPolygons &native_expolygons(const slic3r_api::StoredExPolygonCollection &collection)
{
    return *reinterpret_cast<const ExPolygons *>(collection.handle());
}
} // namespace Slic3r::TreeFamily

#endif // plugins_support_tree_storage_hpp_
