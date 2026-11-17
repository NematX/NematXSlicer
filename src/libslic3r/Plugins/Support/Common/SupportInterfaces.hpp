///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* Dense interface regions of the support slabs, shared by the grid and the tree families.

   A top contact is followed by its top interface slabs, and a bottom contact is preceded by its bottom interface
   slabs. Each base slab takes the contacts whose interface window reaches it, and its interface part is carved out
   of the slab. The carved part must lie inside the base slab, so a family whose base slabs do not already cover the
   contacts first adds their footprints with project_interface_contacts(); the grid's columns already do.

       Common::project_interface_contacts(params, out);  // tree only
       Common::carve_interfaces(params, snug, out);

   In contact mode (SupportParams::contact_mode), the contacts keep only gap_xy_contact from the object. contact_cones() gives
   the cones that bring the slabs near a contact from that gap back to gap_xy; a family unites them with its slabs before the
   carving, so the part of a cone in the interface window becomes interface.

   Once the slabs are final, expand_first_layer_foot() widens the first base slab, for every family. The support ignores the
   brim and the skirt: it uses all the room it wants on the first layer.

       Common::expand_first_layer_foot(object, layers, params, out); */
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
// The cones of the contacts of out on the slabs (sorted by print_z): one collection per slab, in out.storage. columns[i] is the
// area slabs[i] covers before the object trimming. Below a top contact (above a bottom contact), each slab whose column meets the
// contact receives the contact shrunk by step times its depth: 1 for the first such slab, plus one for each such slab between it
// and the contact. Slabs whose column misses the contact are not counted and receive nothing. The walk stops after max_depth
// slabs or when the cone vanishes; max_depth 0 (outside contact mode, p.contact_cone_layers) gives only empty collections. The
// cones are raw: the caller removes the object grown by p.gap_xy_contact and the contacts beside each slab.
std::vector<StoredPolygonCollection> contact_cones(coord_t step, size_t max_depth, const GeneratedSupport &out,
                                                   const GeneratorLayers &slabs, const std::vector<StoredPolygonCollection> &columns);
// Widen the first base slab, when it starts on the bed, by the raft_first_layer_expansion setting without crossing the first object
// layer grown by gap_xy, then remove from it the footprints of the contacts and interfaces on the bed. Other slabs are not changed.
// Details are in the definition.
void expand_first_layer_foot(const Object &object, const std::vector<Layer> &layers, const SupportParams &p, GeneratedSupport &out);
// The widening of expand_first_layer_foot() applied to any sparse area of the bed layer: the area grown by raft_first_layer_expansion
// without crossing the first object layer and its no-support zones grown by gap_xy, less the contacts and interfaces of out on the
// bed. The result lives in out.storage; area and out are unchanged.
StoredPolygonCollection widen_first_layer_area(const Object &object, const std::vector<Layer> &layers, const SupportParams &p,
                                               const GeneratedSupport &out, const StoredPolygonCollection &area);
} // namespace slic3r_api::Support::Common

#endif
