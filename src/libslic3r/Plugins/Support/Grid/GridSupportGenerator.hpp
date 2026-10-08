///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* The generator turns per-island demand into temporary contacts and columns.
   It projects contacts down, places slabs along Z, trims them against the
   object and separates dense interfaces from sparse base. The caller publishes
   these slabs through the data-tree API; no extrusion paths are generated.

   Everything goes through the plugin API: the sliced object is read through
   the data-tree views and every slab is kept in the given plugin storage.

       GeneratedSupport support(storage);
       generate_support(context, print, object, Common::SupportParams(print, object), support); */
#ifndef plugins_support_grid_generator_hpp_
#define plugins_support_grid_generator_hpp_
#include "GridSupportLayers.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Plugins/Support/Common/SupportParams.hpp"

namespace slic3r_api::Support::Grid {
struct GeneratedSupport {
    // Keep the slabs and their temporary geometry in storage.
    explicit GeneratedSupport(storage_handle *storage) : storage(storage) {}
    storage_handle *storage;
    LayerStorage slabs;
    GeneratorLayers top, bottom, base, interfaces;
};
// Populate temporary slabs for the sliced object's demand, leaving publication to the caller; a null
// context never cancels.
void generate_support(const plugin_run_context *context, const Print &print, const Object &object,
                      const Common::SupportParams &params, GeneratedSupport &result);
}
#endif
