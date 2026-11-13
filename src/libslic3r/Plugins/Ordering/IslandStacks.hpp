///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#ifndef slic3r_Plugins_Ordering_IslandStacks_hpp_
#define slic3r_Plugins_Ordering_IslandStacks_hpp_

#include <set>
#include <vector>
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"

/*
Island stacks for the By object builder borrow source layers and island handles.
Collect object and object-wide auxiliary layers by increasing print Z, pairing
equal-Z layers in ObjectLayerToPrint, then call separate_islands. Its result is
an ordered vector of stacks; each stack contains selected islands by source layer.
Source layers and clearance graphs must outlive both the call and its result.
The caller clones each stack separately for every physical object instance.
start_z and max_height use scaled coordinate units, not millimeters.

    ObjectLayerToPrint level;
    level.object_layer = object.layer(1);
    auto stacks = separate_islands({level}, scale_i(0.2), scale_i(1.),
                                   clearance_graphs, EXTRUDER_ID_UNKNOWN, storage);
*/
namespace slic3r_api { namespace Ordering { namespace IslandStacks {

struct ObjectLayerToPrint
{
    Layer object_layer;
    Layer auxiliary_layer;
    std::set<const layer_island_handle *> islands;
    bool allow_wipe_tower = false;
    std::vector<extruder_index_t> extruders_order;

    // Borrow the object layer when present, otherwise its auxiliary companion.
    Layer layer() const { return object_layer.valid() ? object_layer : auxiliary_layer; }
};
using ObjectsLayerToPrint = std::vector<ObjectLayerToPrint>;

// Group printable islands by clearance and extruder into height-limited stacks.
std::vector<ObjectsLayerToPrint> separate_islands(
    const ObjectsLayerToPrint object_layers, coord_t start_z, coord_t max_height,
    const std::vector<GraphData> &extruder_min_dist, extruder_index_t initial_extruder_id,
    storage_handle *storage);

}}}
#endif
