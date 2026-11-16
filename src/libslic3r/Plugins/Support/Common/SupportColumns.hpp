///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* Support columns: helper volumes of an object whose sections become support, whether or not a surface asks for support. A support
   column (SUPPORT_COLUMN) becomes sparse support; a support interface column (SUPPORT_INTERFACE_COLUMN) becomes dense interface support.

   run_support_family() (SupportPublication.hpp) calls add_support_columns() once per object, after the support family has
   built its slabs and before publication. The columns are added the same way for every family: each sparse column section joins the
   sparse slabs (base, intermediate or branch) at their middle Z, each interface column section becomes a top interface slab of the
   same print Z and height (publication then removes its area from the sparse slab), and new base slabs carry the column where no sparse slab
   covers its height. The column keeps clear of the object and of the no-support zones like the family's own slabs, and leaves
   the contacts and interfaces their areas, so no area is printed twice. A column is never extended to the bed.

       if (has_support_columns(object))
           add_support_columns(context, print, object, params, support); */
#ifndef plugins_support_common_columns_hpp_
#define plugins_support_common_columns_hpp_

#include "libslic3r/Api/plugin/c/slic3r_plugin_run_context.h"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Plugins/Support/Common/SupportLayers.hpp"
#include "libslic3r/Plugins/Support/Common/SupportParams.hpp"

namespace slic3r_api::Support::Common {
// Whether the support of the object includes columns: support_material is enabled and the object has a support column or support
// interface column volume.
bool has_support_columns(const Object &object);
// Adds the object's support columns to the slabs of out. Each sparse slab of out.base whose printable interval meets a column
// receives the support column sections at its middle Z (widened like the support foot on the bed layer), and a new top interface slab of out.interfaces with the same print Z and
// height receives the support interface column sections; the column heights that no sparse slab covers get new base slabs,
// cut at the tops of the object layers (or every top layer height above the object) and never thinner than params.min_height.
// Before it joins a slab, a section loses the object and the no-support zones of out grown by params.gap_xy within the Z gaps of
// the slab, and the areas of every contact and interface slab that overlaps it in Z. Does nothing when has_support_columns() is
// false. The new slabs and their geometry are owned by out; cancellation, geometry and allocation failures propagate.
void add_support_columns(const plugin_run_context *context, const Print &print, const Object &object, const SupportParams &params,
                         GeneratedSupport &out);
} // namespace slic3r_api::Support::Common

#endif
