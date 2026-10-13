///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* Dense interface regions of the support slabs, shared by the grid and the tree families.

   A top contact is followed by its top interface slabs, and a bottom contact is preceded by its bottom interface
   slabs. Each base slab takes the contacts whose interface window reaches it, and its interface part is carved out
   of the slab. The carved part must lie inside the base slab, so a family whose base slabs do not already cover the
   contacts first adds their footprints with project_interface_contacts(); the grid's columns already do.

       Common::project_interface_contacts(params, out);  // tree only
       Common::carve_interfaces(params, snug, out);

   Once the slabs are final, expand_first_layer_foot() widens the first base slab and then remove_first_layer_brim() cuts the
   reserved brim area of the object out of the slabs of the first layer, for every family and in this order:

       Common::expand_first_layer_foot(object, layers, params, out);
       Common::remove_first_layer_brim(object, layers, params, out); */
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
// Widen the first base slab, when it starts on the bed, by the raft_first_layer_expansion setting without crossing the first object
// layer grown by gap_xy, then remove from it the footprints of the contacts and interfaces on the bed. Other slabs are not changed.
// Details are in the definition.
void expand_first_layer_foot(const Object &object, const std::vector<Layer> &layers, const SupportParams &p, GeneratedSupport &out);
// Remove the reserved brim area of the object from the support slabs of the first layer (top, base and interface slabs; not the
// floors), in place. Nothing is removed unless the object has a brim in the sense of the native PrintObject::has_brim(), so a brim
// with only one of the two widths positive removes nothing, and with both widths positive the holes are shrunk by brim_width
// (not by brim_width_interior). Details are in the definition.
void remove_first_layer_brim(const Object &object, const std::vector<Layer> &layers, const SupportParams &p, GeneratedSupport &out);
} // namespace slic3r_api::Support::Common

#endif
