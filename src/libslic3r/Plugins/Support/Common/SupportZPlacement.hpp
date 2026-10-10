///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* Z placement of the contacts and of the dense interface slabs, shared by the grid and the tree families.

   A support run is a stack of slabs between extremes. A top contact lies one contact distance under the object
   layer above it, and its extreme is its bottom: the support below it ends there. A bottom contact stands on the
   object layer below it, and its extreme is its top: the support above it starts there. Sorted by Z, two
   neighbouring extremes bound an interval, and its slabs fill it from the bottom up:

       dense bottom slabs   just above the lower extreme, at most the interface step thick
       middle               the sparse part between the two dense stacks, at most the base step thick
       dense top slabs      just under the upper extreme, at most the interface step thick

   Two layouts place an interval. The grid layout computes its dense and middle thicknesses together and spaces
   the whole interval uniformly in one pass; without interface layers its top contact takes the sparse step.
   The handed-over layout is the tree's: a MiddleSplitter fills the middle, the dense stacks keep the interface
   step rounded down to the machine Z step, and a top contact is one more dense slab of that step. When the
   stacks leave a middle that is thinner than the minimum height, or the interval is shorter than its stacks, the
   interval is split evenly into dense slabs instead, with no middle; an unknown top contact is the last of them.
   The dense step is the interface step rounded down to the machine Z step, raised to the minimum height when that is
   larger, so user settings never trigger an assertion. An interval split evenly puts every dense slab between the minimum
   height and the dense step when that is feasible: the split needs n = ceil(length / dense step) slabs, and it is feasible
   when the length is at least n times the minimum height. When it is not, two degradations apply. An interval shorter than
   the minimum height becomes one slab thinner than the minimum. When the n slabs would be thinner than the minimum, the
   split uses fewer slabs, which are at least the minimum height and thicker than the dense step.

   The grid and the tree share the contacts and their dense stacks, so both families print contacts and
   interfaces alike. Contacts follow the native rules: a contact lies the contact distance under the object layer
   above it, a contact below the first layer is dropped, a contact less than the minimum height above the first
   layer snaps onto it, a filament bridge contact sits at the bridge height, and contacts closer than the minimum
   height merge into the lower one. bridge_footprint() gives the area under the bridge threads of the object layers
   above a slab, which a family removes from the slab.

       for each object layer with demand:
           TopContactSlabs slabs = add_top_contacts(layers, print_config, params, id, out);
           // the family fills the polygons of slabs.contact, and of slabs.bridge when it is set
       merge_contacts(params, out);
       place_intermediate_slabs(spacing, layers, out, splitter); */
#ifndef plugins_support_common_z_placement_hpp_
#define plugins_support_common_z_placement_hpp_

#include <cstdint>
#include <functional>
#include <vector>

#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Plugins/Support/Common/SupportLayers.hpp"
#include "libslic3r/Plugins/Support/Common/SupportParams.hpp"

namespace slic3r_api::Support::Common {

// Thicknesses and dense counts that split the space between two extremes; lengths are scaled coordinates.
struct SlabSpacing {
    // First layer height, thinnest printable slab, and the machine Z step that slab boundaries snap to.
    coord_t first_z = 0, min_height = 0, machine_step = 0;
    // Thickest sparse slab, and the interface step: the thickest dense slab.
    coord_t base_step = 0, interface_step = 0;
    // Dense slabs at the top and at the bottom, the contact of each side included.
    int32_t interface_layers = 0, bottom_interface_layers = 0;
    // A soluble contact already has its thickness, so no interval gives it one.
    bool soluble = false;
    // Grid only: the slabs share the Z of the object layers.
    bool synchronize = false;
};

// Receives the middle span [from, to] of an interval, with to > from and to - from at least min_height. It must append
// slabs that tile [from, to] in increasing Z to out.base, and nothing else. The appended slabs belong to out.slabs.
using MiddleSplitter = std::function<void(coord_t from, coord_t to)>;

// The slabs made for one object layer: the contact, and the bridge contact when the layer has bridges. A null contact
// means that the layer gives no contact. Both are owned by out.slabs and listed in out.top, and stay valid until out is destroyed.
struct TopContactSlabs {
    GeneratorLayer *contact = nullptr;
    GeneratorLayer *bridge = nullptr;
};

// Create the top contact of object layer id (id >= 1, the layer below is needed) under that layer; the caller fills
// the polygons of the returned slabs. The layers and the print configuration give the bridge heights, and p gives the
// gaps, the first layer and the minimum height.
TopContactSlabs add_top_contacts(const std::vector<Layer> &layers, const Config &print_config, const SupportParams &p, size_t id,
                                 GeneratedSupport &out);
// Union each top contact within the minimum height above a lower one into that lower contact, and remove the merged
// slabs from out.top. Every top contact must already have its contact_polygons and overhang_polygons set, since the union reads them.
void merge_contacts(const SupportParams &p, GeneratedSupport &out);
// Create the bottom contact that stands on the top surfaces of object layer id and list it in out.bottom. Its polygons are the
// touching area, grown by the bottom expansion and kept on the top surfaces. Its Z is the gap above the layer plus the bottom
// flow height (the upper layer in soluble mode), and it snaps onto the first top contact that the minimum height allows.
// top_surfaces and touching are polygon collections in the plugin storage; the layers must hold id + 1.
GeneratorLayer &add_bottom_contact(const std::vector<Layer> &layers, const SupportParams &p, ContactDistanceType distance_type,
                                   size_t id, const StoredPolygonCollection &touching, const StoredPolygonCollection &top_surfaces,
                                   GeneratedSupport &out);
// Fill every interval between neighbouring extremes with slabs appended to out.base, bottom first. Without a splitter the
// grid layout spaces the middle uniformly; with one, the handed-over layout gives the middle of each interval to it.
// A splitter cannot be combined with synchronize.
void place_intermediate_slabs(const SlabSpacing &spacing, const std::vector<Layer> &layers, GeneratedSupport &out,
                              const MiddleSplitter &split_middle = nullptr);
// Thickness of the dense slabs of the handed-over layout: the interface step rounded down to the machine Z step, raised to the
// minimum height (rounded up to the step) when that is larger. The tips of a tree stack their interfaces at this thickness too.
coord_t handed_over_dense_height(coord_t interface_step, coord_t min_height, coord_t machine_step);
// Allocate a slab owned by out.slabs with the given type and Z dimensions, which must lie on the layer grid.
GeneratorLayer &add_layer(GeneratedSupport &out, LayerType type, coord_t z, coord_t bottom, coord_t height);
// Append to out the XY footprint of each overhang perimeter in an extrusion tree, grown by gap.
void bridge_areas(storage_handle *storage, const ExtrusionEntity &entity, coord_t gap, StoredExPolygonCollection &out,
                  const EPropertyAttributes *attributes = nullptr);
// Height of the bridges of a layer region, the native LayerRegion::bridging_height_avg_mm().
double bridging_height_avg_mm(const Config &print_config, const Layer &layer, const LayerRegion &region);
// Footprints, grown by gap_xy, of the object bridges (thick bridge fills and overhang perimeters) whose thread starts below
// the top of the slab plus the gap above, found in the object layers from the first one that begins at or above that top.
// A slab must not stand where those threads hang. The bridge height follows p.distance_type. The areas are raw offsets, neither
// united nor normalized: the caller unites and normalizes them as it needs. Empty for a soluble support. The caller owns the
// returned collection. An exception from bridging_height_avg_mm (unknown bridge type) propagates.
StoredExPolygonCollection bridge_footprint(storage_handle *storage, const std::vector<Layer> &layers, const Config &print_config,
                                           const SupportParams &p, const GeneratorLayer &slab);

} // namespace slic3r_api::Support::Common
#endif
