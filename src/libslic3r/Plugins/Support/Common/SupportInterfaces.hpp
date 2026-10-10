///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* Dense interface regions of the support slabs, shared by the grid and the tree families.

   A top contact is followed by its top interface slabs, and a bottom contact is preceded by its bottom interface
   slabs. Each base slab takes the contacts whose interface window reaches it, and its interface part is carved out
   of the slab. The carved part must lie inside the base slab, so a family whose base slabs do not already cover the
   contacts first adds their footprints with project_interface_contacts(); the grid's columns already do.

       Common::project_interface_contacts(params, out);  // tree only
       Common::carve_interfaces(params, snug, out); */
#ifndef plugins_support_common_interfaces_hpp_
#define plugins_support_common_interfaces_hpp_

#include "libslic3r/Plugins/Support/Common/SupportLayers.hpp"
#include "libslic3r/Plugins/Support/Common/SupportParams.hpp"

namespace slic3r_api::Support::Common {
// Add the footprint of every contact whose interface window reaches a base slab to that slab, before the carving.
void project_interface_contacts(const SupportParams &p, GeneratedSupport &out);
// Carve the top and bottom interface regions out of the base slabs and list them as interface slabs in out.interfaces;
// snug support closes and smooths the overhangs of its top contacts, other styles only unite the contact footprints.
void carve_interfaces(const SupportParams &p, bool snug, GeneratedSupport &out);
} // namespace slic3r_api::Support::Common

#endif
