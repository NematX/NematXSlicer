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
   above a slab, which a family removes from the slab. A caller-owned BridgeFootprintCache retains each used layer's raw
   bridge pieces in caller-provided storage (grid: Run's run storage; tree: generate_tree_slabs scratch storage).
   The cache must be destroyed before that storage. Sequential slab visits evaluate pieces only as their upper Z bound needs them.

       for each object layer with demand:
           TopContactSlabs slabs = add_top_contacts(layers, print_config, params, id, out);
           // the family fills the polygons of slabs.contact, and of slabs.bridge when it is set
       merge_contacts(params, out);
       place_intermediate_slabs(spacing, layers, out, splitter); */
#ifndef plugins_support_common_z_placement_hpp_
#define plugins_support_common_z_placement_hpp_

#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Plugins/Support/Common/SupportLayers.hpp"
#include "libslic3r/Plugins/Support/Common/SupportParams.hpp"

namespace slic3r_api::Support::Common {

// Convert a slab setting in mm or percent of the nozzle to scaled layer coordinates, capped by max_height_mm; zero selects the maximum and negative effective values resolve to 0.
coord_t slab_step(const c_float_or_percent &setting, double nozzle_mm, double max_height_mm);

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

// Fills the contact-inclusive interface counts from shared parameters, leaving family-specific steps unchanged.
void set_interface_counts(SlabSpacing &spacing, const SupportParams &params);

// Receives the middle span [from, to] of an interval, with to > from and to - from at least min_height. It must append
// slabs that tile [from, to] in increasing Z to out.base, and nothing else. The appended slabs belong to out.slabs.
using MiddleSplitter = std::function<void(coord_t from, coord_t to)>;

// The slabs made for one object layer: the contact, and the bridge contact when the layer has bridges. A null contact
// means that the layer gives no contact. Both are owned by out.slabs and listed in out.top, and stay valid until out is destroyed.
struct TopContactSlabs {
    GeneratorLayer *contact = nullptr;
    GeneratorLayer *bridge = nullptr;
};

// The Z of the top contacts under object layer id: the contact's print Z, bottom and height (zero: the slab placement gives it),
// and the lower print Z of the bridge contact when the layer's bridge threads hang below it. The bridge contact's bottom is its
// print Z, or 0 with the first layer height when it lies on the first layer.
struct TopContactZ {
    coord_t z = 0, bottom = 0, height = 0;
    std::optional<coord_t> bridge_z;
};
// The Z of the contacts that add_top_contacts() creates under object layer id (id >= 1), without creating them: none when the
// contact would lie below the first layer. storage holds the temporaries of the bridge detection.
std::optional<TopContactZ> top_contact_z(const std::vector<Layer> &layers, const Config &print_config, const SupportParams &p, size_t id,
                                         storage_handle *storage);
// Create the top contact of object layer id (id >= 1, the layer below is needed) under that layer; the caller fills
// the polygons of the returned slabs. The layers and the print configuration give the bridge heights, and p gives the
// gaps, the first layer and the minimum height.
TopContactSlabs add_top_contacts(const std::vector<Layer> &layers, const Config &print_config, const SupportParams &p, size_t id,
                                 GeneratedSupport &out);
// Union each top contact within the minimum height above a lower one into that lower contact, and remove the merged
// slabs from out.top. Every top contact must already have its contact_polygons and overhang_polygons set, since the union reads them.
void merge_contacts(const SupportParams &p, GeneratedSupport &out);
// Create the bottom contact that stands on the top surfaces of object layer id and list it in out.bottom. Its polygons are the
// touching area, grown by the bottom expansion and kept on the top surfaces. Its Z is the gap below the layer plus its thickness:
// the height of the upper layer (Plane distance type), filled up to the top of the upper layer (soluble), or the height of the
// bottom contact flow, bottom_flow_mm (otherwise). It snaps onto the first top contact that the minimum height allows.
// top_surfaces and touching are polygon collections in the plugin storage; the layers must hold id + 1. The returned reference
// belongs to out.
GeneratorLayer &add_bottom_contact(const std::vector<Layer> &layers, const SupportParams &p, size_t id,
                                   const StoredPolygonCollection &touching, const StoredPolygonCollection &top_surfaces,
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
// Borrowed layer/work views and independently lazy raw pieces in caller-provided storage; views and storage outlive the cache.
struct BridgeFootprintLayer {
    // Regions retain layer order; work islands retain island/work-island order.
    std::vector<LayerRegion> regions;
    std::vector<LayerRegionIsland> works;
    // Empty until reached by composition; a region's fills include every member work island in work order.
    std::vector<std::optional<coord_t>> region_starts;
    std::vector<std::optional<StoredExPolygonCollection>> region_fills;
    // Empty until a work island with perimeters passes its member-region height checks.
    std::vector<std::optional<StoredExPolygonCollection>> perimeters;
};

// Lazy per-layer pieces for one object/settings/storage combination. Grid and tree trimming use this cache sequentially;
// no parallel slab task accesses it. Destroy the cache before its storage; failed layer/piece construction leaves an empty slot.
struct BridgeFootprintCache {
    std::vector<std::optional<BridgeFootprintLayer>> layers;
};

// Footprints, grown by gap, of the object bridges (thick bridge fills and overhang perimeters) whose thread starts below
// the top of the slab plus the gap above, found in the object layers from the first one that begins at or above from_z. The layers
// that begin below from_z are not examined: the caller removes their outlines itself, so from_z is the slab top plus the gap above
// for a caller whose outline trimming reaches that far, and the slab top for one that trims only the layers its Z span crosses.
// A slab must not stand where those threads hang. The bridge height follows p.distance_type. The areas are raw offsets, neither
// united nor normalized: the caller unites and normalizes them as it needs. Empty for a soluble support. The caller owns the
// returned collection. cache, when supplied, retains pieces in storage for this object/settings combination and must be
// accessed sequentially and with a single gap; nullptr uses a cache local to this call. An exception from bridging_height_avg_mm (unknown bridge type) propagates.
StoredExPolygonCollection bridge_footprint(storage_handle *storage, const std::vector<Layer> &layers, const Config &print_config,
                                           const SupportParams &p, coord_t from_z, const GeneratorLayer &slab, coord_t gap,
                                           BridgeFootprintCache *cache = nullptr);

} // namespace slic3r_api::Support::Common
#endif
