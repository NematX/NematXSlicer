///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* The generator turns per-island demand into temporary contacts and columns.
   It projects contacts down, places slabs along Z, trims them against the
   object and separates dense interfaces from sparse base. The caller publishes
   these slabs through the data-tree API (Common/SupportPublication.hpp); no
   extrusion paths are generated. raster_contours() also extracts corner rings
   directly from a zero-bordered raster in the caller's storage.

   Everything goes through the plugin API: the sliced object is read through
   the data-tree views and every slab is kept in the given plugin storage.

       Common::GeneratedSupport support(storage);
       generate_support(context, print, object, Common::SupportParams(print, object), support); */
#ifndef plugins_support_grid_generator_hpp_
#define plugins_support_grid_generator_hpp_
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Plugins/Support/Common/SupportLayers.hpp"
#include "libslic3r/Plugins/Support/Common/SupportParams.hpp"

namespace slic3r_api::Support::Grid {
// Chain cell rings and filter corners into storage-owned closed polygons without a repeated first vertex.
// The borrowed row-major pixels array has size_x columns and size_y rows, including a zero border.
// pixel_size, origin and offset use scaled coordinates; require |2*offset| < pixel_size - 10.
// fill_holes fills empty cells with filled opposite neighbours (left/right or up/down) in the input array.
StoredPolygonCollection raster_contours(storage_handle *storage, int size_x, int size_y, double pixel_size,
    c_point origin, const std::vector<unsigned char> &pixels, coord_t offset, bool fill_holes);
// Populate temporary slabs for the sliced object's demand, leaving publication to the caller; a null
// context never cancels.
void generate_support(const plugin_run_context *context, const Print &print, const Object &object,
                      const Common::SupportParams &params, Common::GeneratedSupport &result);
}
#endif
