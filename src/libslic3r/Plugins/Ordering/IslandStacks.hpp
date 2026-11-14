///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#ifndef slic3r_Plugins_Ordering_IslandStacks_hpp_
#define slic3r_Plugins_Ordering_IslandStacks_hpp_

#include <set>
#include <vector>
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"

/*
Island stacks for the By object builder borrow source layers and island handles.
Collect object and auxiliary layers for every physical instance. Each selection
holds one layer, or an object and auxiliary layer at the same Z. Call separate_islands
after collecting every selection. Its result is an ordered
vector of stacks; each stack contains selected islands by source layer and instance.
Source layers must outlive the call and its result; graphs are borrowed during the call.
The caller emits one group per stack, cloning each selection at its instance shift.
Instance ids distinguish copies even when their source island handles are equal.
Clearance graphs include the printer's radius and height as their final point.
max_height is positive and uses scaled coordinate units, not millimeters. A blocked stack is split
by emitting the lowest remaining Z across all stacks before trying whole stacks again.

    ObjectLayerToPrint level;
    level.object_layer = object.layer(1);
    auto stacks = separate_islands({level}, scale_i(1.),
                                   clearance_graphs, EXTRUDER_ID_UNKNOWN, storage);
*/
namespace slic3r_api { namespace Ordering { namespace IslandStacks {

struct ObjectLayerToPrint
{
    uint32_t instance_id = 0;
    c_point shift = {};
    Layer object_layer;
    Layer auxiliary_layer;
    std::set<const layer_island_handle *> islands;
    // Shared tool order of all selections at this Z in the returned stack.
    std::vector<extruder_index_t> extruders_order;

    // Borrow the object layer when present, otherwise its auxiliary companion.
    Layer layer() const { return object_layer.valid() ? object_layer : auxiliary_layer; }
};
using ObjectsLayerToPrint = std::vector<ObjectLayerToPrint>;

// Group plate-coordinate islands by clearance into height-limited stacks.
std::vector<ObjectsLayerToPrint> separate_islands(
    const ObjectsLayerToPrint object_layers, coord_t max_height,
    const std::vector<GraphData> &extruder_min_dist, extruder_index_t initial_extruder_id,
    storage_handle *storage);

}}}
#endif
