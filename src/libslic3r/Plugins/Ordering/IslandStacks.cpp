///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#include "IslandStacks.hpp"

#include <algorithm>
#include <cassert>
#include <limits>
#include <optional>
#include <stdexcept>
#include <unordered_map>
#include "libslic3r/Plugins/Common/ClipperHelpers.hpp"

/*
Builds island stacks for the By object builder from borrowed API layer views.
Each Z level first groups nearby islands using nozzle clearance, including support
islands from the auxiliary layer in the same search. Extruder compatibility keeps
unrelated tools apart. Lower-island links attach each group to an open stack;
height differences close stacks before further layers can extend them. Groups
with several extruders are closed immediately. Temporary polygon copies belong
to this call; returned stacks borrow their layer and island handles from input.
Wipe-tower flags and tool sequences are local scheduling state, not emitted work.

    separate_islands() -> group islands at each Z and update open stacks
        fn_group_islands() -> combine islands without enough nozzle clearance
        min_bottom_z() -> measure physical layer thickness for stack closing
        fn_add_group_into() -> retain selected object and support islands
        set_finished() -> remove closed islands from the parent search
*/
namespace slic3r_api { namespace Ordering { namespace IslandStacks {

// Group each Z level by clearance and tool, then use lower-island links to extend
// or close stacks. Input excludes layer zero and layers at the caller's upper Z
// limit. Geometry temporaries use borrowed storage; returned selections borrow
// unchanged source layers and islands. Allocation and API failures propagate.
std::vector<ObjectsLayerToPrint> separate_islands(
    const ObjectsLayerToPrint object_layers, coord_t start_z, coord_t max_height,
    const std::vector<GraphData> &extruder_min_dist, extruder_index_t initial_extruder_id,
    storage_handle *storage);

std::vector<ObjectsLayerToPrint> separate_islands(
                const ObjectsLayerToPrint object_layers,
                const coord_t start_z,
                const coord_t max_height,
                const std::vector<GraphData> &extruder_min_dist,
                const extruder_index_t initial_extruder_id,
                storage_handle *storage) {
    // This private state is not a public extruder identity: it marks a group
    // containing several extruders before the group is split for emission.
    constexpr extruder_index_t MULTIPLE_EXTRUDERS = -2;
    std::vector<ObjectsLayerToPrint> ordered_layers_islands_to_print;
    if (object_layers.empty())
        return ordered_layers_islands_to_print;
    struct GroupedIslands
    {
        c_bounding_box bb;
        PluginCommon::polygon_work::ExPolygons slices;
        std::vector<LayerIsland> islands;
        std::vector<bool> is_support;
        // extruder of the group.(-1: any; -2: multiple)
        extruder_index_t extruder_id = EXTRUDER_ID_UNKNOWN;
        std::set<extruder_index_t> extruders_id;
    };

    std::set<const layer_island_handle *> all_islands;
    std::vector<bool> finished(ordered_layers_islands_to_print.size());
    std::vector<extruder_index_t> extruder(ordered_layers_islands_to_print.size(), EXTRUDER_ID_UNKNOWN);
    extruder_index_t previous_extruder_id = initial_extruder_id;
    auto set_finished = [&ordered_layers_islands_to_print, &finished, &all_islands](size_t idx) {
        finished[idx] = true;
        // now tht this group of layers is finished, all next groups are after these layer-islands, so we
        // can safely remove them from 'all_islands' collection
        for (const ObjectLayerToPrint &objectlayer : ordered_layers_islands_to_print[idx]) {
            for (const layer_island_handle *island : objectlayer.islands) {
                all_islands.erase(island);
            }
        }
    };

    // we need a function to get the min z of an ObjectLayerToPrint (both layer have same printz, though)
    auto min_bottom_z = [](const ObjectLayerToPrint &objlay) -> coord_t {
        coord_t bottom = std::numeric_limits<coord_t>::max();
        if (objlay.object_layer.valid()) {
            bottom = objlay.object_layer.bottom_z();
        }
        if (objlay.auxiliary_layer.valid()) {
            bottom = std::min(bottom, objlay.auxiliary_layer.bottom_z());
        }
        assert(bottom >= 0 && bottom < std::numeric_limits<coord_t>::max());
        return bottom;
    };

    auto fn_group_islands = [&all_islands, max_height, storage,
                             &extruder_min_dist](std::vector<GroupedIslands> &grouped_islands, const Layer &layer,
                                                 bool is_support) {
        for (uint32_t island_idx = 0; island_idx < layer.island_count(); ++island_idx) {
            const LayerIsland island = layer.island(island_idx);
            bool has_extrusions = false;
            for (uint32_t idx = 0; idx < island.region_island_count(); ++idx)
                has_extrusions |= island.region_island(idx).has_extrusions();
            if (!has_extrusions) {
                // shouldn't happen... i guess. but anyway, there is no extrusions inside, ignore it
                continue;
            }
            all_islands.insert(island.handle());
            size_t added = size_t(-1);
            size_t group_idx = 0;
            // check if the island has multiple nozzle, if true don't merge it.
            extruder_index_t my_extruder_id = EXTRUDER_ID_UNKNOWN;
            std::set<extruder_index_t> my_extruders_id;
            for (uint32_t idx = 0; idx < island.region_island_count(); ++idx) {
                const LayerRegionIsland lri = island.region_island(idx);
                const extruder_index_t region_extruder = lri.extruder_id();
                if (region_extruder < EXTRUDER_ID_UNKNOWN)
                    throw std::invalid_argument("LayerRegionIsland contains an invalid negative extruder ID");
                if (region_extruder != EXTRUDER_ID_UNKNOWN)
                    my_extruders_id.insert(region_extruder);
                if (my_extruder_id == EXTRUDER_ID_UNKNOWN) {
                    my_extruder_id = region_extruder;
                } else {
                    if (region_extruder != EXTRUDER_ID_UNKNOWN && my_extruder_id != region_extruder) {
                        // multiple extruder in same island -> can't be printed in parallel
                        // skip the search loop, go to add it in a new group.
                        group_idx = grouped_islands.size();
                        my_extruder_id = MULTIPLE_EXTRUDERS;
                    }
                }
            }
            // try to merge into a group
            for (; group_idx < grouped_islands.size(); group_idx++) {
                GroupedIslands &group = grouped_islands[group_idx];
                c_bounding_box inflated = group.bb;
                // check if extruder is compatible
                if (my_extruder_id != EXTRUDER_ID_UNKNOWN &&
                    group.extruder_id != EXTRUDER_ID_UNKNOWN) {
                    if (my_extruder_id != group.extruder_id) {
                        // extruder is incompatible, don't add it inside.
                        continue;
                    }
                }
                // get compute min dist for the group height and the island extruder(s)
                coord_t min_dist = 0;
                bool no_clearance = false;
                for (uint32_t idx = 0; idx < island.region_island_count(); ++idx) {
                    const LayerRegionIsland region_island = island.region_island(idx);
                    assert(region_island.extruder_id() < extruder_min_dist.size() ||
                           region_island.extruder_id() == EXTRUDER_ID_UNKNOWN);
                    if (region_island.extruder_id() != EXTRUDER_ID_UNKNOWN) {
                        // check if in the usable part of the clearance
                        const GraphData &graph = extruder_min_dist[region_island.extruder_id()];
                        if (graph.point(graph.size() - 1).y < double(max_height) * SCALING_FACTOR) {
                            no_clearance = true;
                        } else {
                            min_dist = std::max(min_dist, scale_i(
                                graph.inverse_interpolate(double(max_height) * SCALING_FACTOR)));
                        }
                    } else {
                        for (const GraphData &extr_data : extruder_min_dist) {
                            if (extr_data.point(extr_data.size() - 1).y < double(max_height) * SCALING_FACTOR) {
                                no_clearance = true;
                            } else {
                                min_dist = std::max(min_dist, scale_i(
                                    extr_data.inverse_interpolate(double(max_height) * SCALING_FACTOR)));
                            }
                        }
                    }
                }
                inflated.min.x -= min_dist;
                inflated.min.y -= min_dist;
                inflated.max.x += min_dist;
                inflated.max.y += min_dist;
                if (no_clearance || bounding_box_overlap(inflated, island.bounding_box())) {
                    if (no_clearance || !PluginCommon::clipper::intersection_ex(storage,
                            PluginCommon::clipper::offset_ex(storage,
                                PluginCommon::clipper::from_expolygon_view(island.slice()), double(min_dist)), group.slices).empty()) {
                        if (added == size_t(-1)) {
                            // collide -> add it
                            bounding_box_merge(group.bb, island.bounding_box());
                            group.slices.push_back(PluginCommon::clipper::from_expolygon_view(
                                island.slice())); // note: as they are islands, they shouldn't touch each other.
                            group.islands.push_back(island);
                            group.is_support.push_back(is_support);
                            if (group.extruder_id == EXTRUDER_ID_UNKNOWN) {
                                group.extruder_id = my_extruder_id;
                            }
                            group.extruders_id.insert(my_extruders_id.begin(), my_extruders_id.end());
                            added = group_idx;
                        } else {
                            // merge the groups, sorry
                            GroupedIslands &main_group = grouped_islands[added];
                            assert(main_group.extruder_id == group.extruder_id ||
                                   main_group.extruder_id == EXTRUDER_ID_UNKNOWN || group.extruder_id == EXTRUDER_ID_UNKNOWN);
                            bounding_box_merge(main_group.bb, group.bb);
                            for (size_t gr_i = 0; gr_i < group.islands.size(); gr_i++) {
                                main_group.slices.push_back(std::move(group.slices[gr_i]));
                                main_group.islands.push_back(std::move(group.islands[gr_i]));
                                main_group.is_support.push_back(std::move(group.is_support[gr_i]));
                            }
                            if (main_group.extruder_id == EXTRUDER_ID_UNKNOWN && group.extruder_id != EXTRUDER_ID_UNKNOWN) {
                                main_group.extruder_id = group.extruder_id;
                                main_group.extruders_id.insert(group.extruders_id.begin(), group.extruders_id.end());
                            }
                            grouped_islands.erase(grouped_islands.begin() + group_idx);
                            group_idx--;
                        }
                    }
                }
            }
            if (added == size_t(-1)) {
                // create your group
                grouped_islands.emplace_back();
                grouped_islands.back().bb = island.bounding_box();
                grouped_islands.back().slices.push_back(PluginCommon::clipper::from_expolygon_view(island.slice()));
                grouped_islands.back().islands.push_back(island);
                grouped_islands.back().is_support.push_back(is_support);
                grouped_islands.back().extruder_id = my_extruder_id;
                grouped_islands.back().extruders_id.insert(my_extruders_id.begin(), my_extruders_id.end());
            }
        }
    };
    // we use the object layer graph to print each island separatly
    //const PrintObject* obj = object_layers.front().object_layer->object();
    // we explore the graph layer per layer.
    for (const ObjectLayerToPrint &layer : object_layers) {
        if (start_z > layer.layer().print_z() /*|| layer.layer()->scaled_print_z() >= start_z + max_height*/) {
            continue;
        }
        // Layer zero and its auxiliaries are selected by the caller's first pass.
        std::vector<GroupedIslands> grouped_islands;
        // first, we group islands that are too close toguether
        if (layer.object_layer.valid()) {
            assert(layer.object_layer.island_count() != 0);
            fn_group_islands(grouped_islands, layer.object_layer, false);
        }
        if (layer.auxiliary_layer.valid()) {
            assert(layer.auxiliary_layer.island_count() != 0);
            fn_group_islands(grouped_islands, layer.auxiliary_layer, true);
        }

        // need to check if the new printz may close some too tall stacks
        for (size_t coll_idx = 0; coll_idx < ordered_layers_islands_to_print.size(); coll_idx++) {
            if (finished[coll_idx]) {
                continue;
            }
            // if a collection after it start may be than max_height before
            for (size_t coll_check_idx = coll_idx + 1; coll_check_idx < ordered_layers_islands_to_print.size();
                 coll_check_idx++) {
                assert(!ordered_layers_islands_to_print[coll_check_idx].empty());
                assert(ordered_layers_islands_to_print[coll_check_idx].front().layer().valid());
                if (ordered_layers_islands_to_print[coll_idx].back().layer().print_z() -
                        ordered_layers_islands_to_print[coll_check_idx].front().layer().print_z() >
                    max_height - (layer.layer().print_z() - min_bottom_z(layer))) {
                    //// remove last layer and put it at the end in a new collection
                    //ordered_layers_islands_to_print.emplace_back();
                    //finished.push_back(false);
                    //extruder.push_back(extruder[coll_idx]);
                    //ordered_layers_islands_to_print.back().push_back(std::move(ordered_layers_islands_to_print[coll_idx].back()));
                    //ordered_layers_islands_to_print[coll_idx].pop_back();
                    set_finished(coll_idx);
                    break;
                } else if (ordered_layers_islands_to_print[coll_idx].back().layer().print_z() <
                           ordered_layers_islands_to_print[coll_check_idx].front().layer().print_z()) {
                    // some new groups start higher than the last one of this group so it can't grow anymore.
                    // we can set it to finished (even if it's of no use).
                    set_finished(coll_idx);
                    break;
                }

            }
        }

        // check if extruder swap
        //bool has_extruder_swap = false;
        //ObjectsLayerToPrint mmu_layers;
        size_t mmu_layers_idx = size_t(-1);
        // extruder available after the end of the current group
        extruder_index_t current_extruder_id = previous_extruder_id; // -1 == any, -2 == multiple (there is toolchange)
        //for (const GroupedIslands &group : grouped_islands) {
        //    if (current_extruder_id == EXTRUDER_ID_UNKNOWN) {
        //        current_extruder_id = group.extruder_id;
        //    } else if (group.extruder_id != EXTRUDER_ID_UNKNOWN && current_extruder_id != group.extruder_id) {
        //        has_extruder_swap = true;
        //        current_extruder_id = MULTIPLE_EXTRUDERS;
        //        break;
        //    }
        //}
        //if (!has_extruder_swap && current_extruder_id >= 0 && previous_extruder_id >= 0 &&
        //    current_extruder_id != previous_extruder_id) {
        //    has_extruder_swap = true;
        //}

        // - when all of our below islands are in the same ObjectsLayerToPrint, we put ourself into it (or adding our
        // island into the current one)
        // - when our below islands are in different ObjectsLayerToPrint, we create a new one and put ourself into when
        auto fn_add_group_into = [&layer](ObjectLayerToPrint& container_to_fill, GroupedIslands& group) {
            bool has_obj = false;
            bool has_supp = false;
            assert(group.islands.size() == group.is_support.size());
            for (size_t i=0;i<group.islands.size(); i++) {
                if (group.is_support[i]) {
                    has_supp = true;
                } else {
                    has_obj = true;
                }
                container_to_fill.islands.insert(group.islands[i].handle());
            }
            if (has_obj) {
                container_to_fill.object_layer = layer.object_layer;
            }
            if (has_supp) {
                container_to_fill.auxiliary_layer = layer.auxiliary_layer;
            }
            container_to_fill.allow_wipe_tower = false;
        };
        std::vector<std::optional<ObjectLayerToPrint>> to_add(ordered_layers_islands_to_print.size());
        extruder_index_t mmu_layers_extruder_id = EXTRUDER_ID_UNKNOWN;
        // find the extruder after mmu if there is a mmu layer.
        std::set<extruder_index_t> mmu_layers_extruders_id;
        for (GroupedIslands &group : grouped_islands) {
            if (group.extruder_id == MULTIPLE_EXTRUDERS) {
                mmu_layers_extruders_id.insert(group.extruders_id.begin(), group.extruders_id.end());
            }
        }
        if (!mmu_layers_extruders_id.empty()) {
            std::unordered_map<extruder_index_t, size_t> extruder_count;
            // Count occurrences
            for (GroupedIslands &group : grouped_islands) {
                if (group.extruder_id != previous_extruder_id &&
                    group.extruder_id != EXTRUDER_ID_UNKNOWN &&
                    group.extruder_id != MULTIPLE_EXTRUDERS) {
                    for (extruder_index_t extr_id : group.extruders_id) {
                        extruder_count[extr_id]++;
                    }
                }
            }
            size_t max_count = 0;
            for (GroupedIslands &group : grouped_islands) {
                if (auto it = extruder_count.find(group.extruder_id); it != extruder_count.end()) {
                    if (it->second > max_count) {
                        mmu_layers_extruder_id = group.extruder_id;
                        max_count = it->second;
                    }
                }
            }
        }
        std::sort(grouped_islands.begin(), grouped_islands.end(), [previous_extruder_id](GroupedIslands &g1, GroupedIslands &g2) {
            // first: groups with same extruder as current
            if (previous_extruder_id == g1.extruder_id && previous_extruder_id != g2.extruder_id) {
                return true;
            }
            if (previous_extruder_id != g1.extruder_id && previous_extruder_id == g2.extruder_id) {
                return false;
            }
            // second: groups than can still use the current extruder
            if (g1.extruder_id == EXTRUDER_ID_UNKNOWN && g2.extruder_id != EXTRUDER_ID_UNKNOWN) {
                return true;
            }
            if (g1.extruder_id != EXTRUDER_ID_UNKNOWN && g2.extruder_id == EXTRUDER_ID_UNKNOWN) {
                return false;
            }
            //thrid: mmu groups, so it can start to prnt the wipetower with as much toolchange as needed
            if (g1.extruder_id == MULTIPLE_EXTRUDERS && g2.extruder_id != MULTIPLE_EXTRUDERS) {
                return true;
            }
            if (g1.extruder_id != MULTIPLE_EXTRUDERS && g2.extruder_id == MULTIPLE_EXTRUDERS) {
                return false;
            }
            // then extruder in simple sequence (don't reuse tool_oerdering, too complex)
            if (g1.extruder_id != g2.extruder_id) {
                return g1.extruder_id < g2.extruder_id;
            }
            if (g1.islands.size() == g2.islands.size()) {
                const c_point size1 = bounding_box_size(g1.bb), size2 = bounding_box_size(g2.bb);
                return double(size1.x) * double(size1.y) > double(size2.x) * double(size2.y);
            }
            return g1.islands.size() > g2.islands.size();
        });
        for (GroupedIslands &group : grouped_islands) {
            assert(group.extruder_id != MULTIPLE_EXTRUDERS || group.extruders_id.size() > 1);
            // need to be in mmu?
            bool need_mmu = group.extruder_id == MULTIPLE_EXTRUDERS;
            if (!need_mmu && mmu_layers_idx != size_t(-1) &&
                group.extruder_id != EXTRUDER_ID_UNKNOWN &&
                group.extruder_id != MULTIPLE_EXTRUDERS) {
                // add this group to muu if use an extruder
                auto mmu_extruders = ordered_layers_islands_to_print[mmu_layers_idx].front().extruders_order;
                if (mmu_extruders.front() != group.extruder_id && mmu_extruders.back() != group.extruder_id) {
                    need_mmu = std::find(mmu_extruders.begin(), mmu_extruders.end(),group.extruder_id) != mmu_extruders.end();
                }
            }
            if (need_mmu) {
                if (mmu_layers_idx != size_t(-1)) {
                    ObjectsLayerToPrint &mmu_layers = ordered_layers_islands_to_print[mmu_layers_idx];
                    // add it to mmu layer
                    mmu_layers.emplace_back();
                    fn_add_group_into(mmu_layers.back(), group);
                    mmu_layers.back().allow_wipe_tower = true;
                    // Added islands belong to a closed stack and cannot be pending parents.
                    for (const LayerIsland &island : group.islands)
                        all_islands.erase(island.handle());
                    // add to extruder order
                    for(extruder_index_t extr_id : group.extruders_id) {
                        if (std::find(mmu_layers.front().extruders_order.begin(),
                                      mmu_layers.front().extruders_order.end(),
                                      extr_id) == mmu_layers.front().extruders_order.end()) {
                            mmu_layers.front().extruders_order.insert(mmu_layers.front().extruders_order.begin()+1,
                                                                      extr_id);
                        }
                    }
                    // Sort the "middle part" only (everything except first and last)
                    if (mmu_layers.front().extruders_order.size() > 2) {
                        std::sort(mmu_layers.front().extruders_order.begin() + 1, mmu_layers.front().extruders_order.end() - 1);
                    }
                } else {
                    mmu_layers_idx = ordered_layers_islands_to_print.size();
                    ordered_layers_islands_to_print.emplace_back();
                    to_add.emplace_back();
                    extruder.push_back(-2);
                    finished.push_back(true);
                    ObjectsLayerToPrint &mmu_layers = ordered_layers_islands_to_print[mmu_layers_idx];
                    // put it into the mmu_layers
                    mmu_layers.emplace_back();
                    fn_add_group_into(mmu_layers.back(), group);
                    mmu_layers.back().allow_wipe_tower = true;
                    assert(group.extruders_id.size() > 1);
                    // get the next value after current_extruder_id
                    if (mmu_layers_extruder_id == EXTRUDER_ID_UNKNOWN) {
                        auto it_extr = group.extruders_id.upper_bound(current_extruder_id);
                        if (it_extr == group.extruders_id.end()) {
                            // all values are lower or equal to current_extruder_id, so get the previous one from
                            // current_extruder_id
                            it_extr = group.extruders_id.lower_bound(current_extruder_id);
                            // A current tool above every group tool has the last tool as its predecessor.
                            if (it_extr == group.extruders_id.end() || *it_extr == current_extruder_id) {
                                assert(it_extr != group.extruders_id.begin());
                                --it_extr;
                            }
                        }
                        mmu_layers_extruder_id = *it_extr;
                    }
                    // set the mmu_layer extruder ordering (natural order, with enforced current_extruder at start)
                    // 1. Copy set → vector (keeps sorted order)
                    mmu_layers.back().extruders_order.assign(group.extruders_id.begin(), group.extruders_id.end());
                    // 2. Rotate so current_extruder_id is first
                    auto it = std::find(mmu_layers.back().extruders_order.begin(),
                                        mmu_layers.back().extruders_order.end(), current_extruder_id);
                    if (it != mmu_layers.back().extruders_order.end()) {
                        // move to first
                        std::rotate(mmu_layers.back().extruders_order.begin(), it,
                                    mmu_layers.back().extruders_order.end());
                    } else {
                        // add as first
                        mmu_layers.back().extruders_order.insert(mmu_layers.back().extruders_order.begin(),
                                                                 current_extruder_id);
                    }
                    //ensure mmu_layers_extruder_id is last
                    it = std::find(mmu_layers.back().extruders_order.begin(),
                                        mmu_layers.back().extruders_order.end(), mmu_layers_extruder_id);
                    if (it != mmu_layers.back().extruders_order.end()) {
                        // move to last
                        std::rotate(it, it + 1, mmu_layers.back().extruders_order.end());
                    } else {
                        // add as last
                        mmu_layers.back().extruders_order.insert(mmu_layers.back().extruders_order.end(),
                                                                 mmu_layers_extruder_id);
                    }
                    assert(mmu_layers.back().extruders_order.size() > 1);
                    assert(mmu_layers.back().extruders_order.back() == mmu_layers_extruder_id);

                    // set this mmu as finished, as it really uses the colors, it can't continue with only one.
                    set_finished(ordered_layers_islands_to_print.size() - 1);
                    assert(mmu_layers_extruder_id != EXTRUDER_ID_UNKNOWN);
                    current_extruder_id = mmu_layers_extruder_id;
                }
                continue;
            }

            std::set<const layer_island_handle *> still_needed;
            // create list of parent peer group
            for (const LayerIsland &island : group.islands) {
                for (const LayerIsland &link : island.lower_islands()) {
                    //don't use layer from other calls.
                    if (all_islands.find(link.handle()) != all_islands.end()) {
                        still_needed.insert(link.handle());
                    }
                }
            }
            bool was_added = false;
            // remove from still_needed until it's empty, then add it.
            if (still_needed.empty()) {
                // put at the end
                ordered_layers_islands_to_print.emplace_back();
                to_add.emplace_back(ObjectLayerToPrint());
                finished.push_back(false);
                extruder.push_back(group.extruder_id);
                fn_add_group_into(to_add.back().value(), group);
                if (group.extruder_id != EXTRUDER_ID_UNKNOWN && group.extruder_id != current_extruder_id) {
                    // create a new group with extruder change.
                    to_add.back().value().allow_wipe_tower = true;
                    to_add.back().value().extruders_order = {current_extruder_id, group.extruder_id};
                    current_extruder_id = group.extruder_id;
                } else {
                    to_add.back().value().extruders_order = {group.extruder_id};
                }
                was_added = true;
            } else {
                // true if at least 2 parent are in two different group, so i need to be in a new one after these two.
                bool already_in_another = false;
                bool parent_not_last = false;
                for (size_t i=0;i<ordered_layers_islands_to_print.size(); i++) {
                    assert(i < finished.size());
                    if (finished[i]) {
                        continue;
                    }
                    assert(!still_needed.empty());
                    // do we ahave remove a parent from this group?
                    bool removed_at_least_one = false;
                    {
                        ObjectsLayerToPrint &previous_layers = ordered_layers_islands_to_print[i];
                        if (previous_layers.empty()) {
                            continue;
                        }
                        for (size_t layer_idx = 0; layer_idx < previous_layers.size(); layer_idx++) {
                            //if (previous_layers[layer_idx].layer()->scaled_print_z() <
                            //    min_bottom_z(layer)) {
                            //    // too far back to be possible to contains a 'still needed' layer-island
                            //    continue;
                            //}
                            for (const layer_island_handle *check_island : previous_layers[layer_idx].islands) {
                                if (auto it = still_needed.find(check_island); it != still_needed.end()) {
                                    still_needed.erase(it);
                                    removed_at_least_one = true;
                                    parent_not_last = layer_idx < previous_layers.size() - 1;
                                }
                            }
                        }
                    }

                    //found! add it after!
                    if (still_needed.empty()) {
                        assert(extruder[i] != MULTIPLE_EXTRUDERS && group.extruder_id != MULTIPLE_EXTRUDERS);
                        // if continuation of the current group or need toolchange, go here
                        if ((/*!already_in_another &&*/ !to_add[i].has_value()) ||
                            (extruder[i] != group.extruder_id && group.extruder_id != EXTRUDER_ID_UNKNOWN)) {
                            assert(!parent_not_last);
                            // if toolchange, then need to create a new one now.
                            if ((extruder[i] != group.extruder_id && group.extruder_id != EXTRUDER_ID_UNKNOWN)) {
                                // change of extruder, create new stack with the new extruder
                                // put at the end
                                ordered_layers_islands_to_print.emplace_back();
                                to_add.emplace_back(ObjectLayerToPrint());
                                finished.push_back(false);
                                extruder.push_back(group.extruder_id);
                                fn_add_group_into(to_add.back().value(), group);
                                if (group.extruder_id != EXTRUDER_ID_UNKNOWN && group.extruder_id != current_extruder_id) {
                                    // create a new group with extruder change.
                                    to_add.back().value().allow_wipe_tower = true;
                                    to_add.back().value().extruders_order = {current_extruder_id, group.extruder_id};
                                    current_extruder_id = group.extruder_id;
                                } else {
                                    to_add.back().value().extruders_order = {group.extruder_id};
                                }
                            } else {
                                // put inside the current one
                                to_add[i] = ObjectLayerToPrint();
                                assert(extruder[i] == EXTRUDER_ID_UNKNOWN || group.extruder_id == EXTRUDER_ID_UNKNOWN ||
                                       extruder[i] == group.extruder_id);
                                if (extruder[i] == EXTRUDER_ID_UNKNOWN) {
                                    extruder[i] = group.extruder_id;
                                }
                                fn_add_group_into(to_add[i].value(), group);
                                assert(ordered_layers_islands_to_print[i].front().extruders_order.back() ==
                                       group.extruder_id);
                                to_add[i].value().extruders_order = {group.extruder_id};
                            }
                            was_added = true;
                        } else {
                            // no toolchange, but the place is already taken, so I need to create a new stack just after
                            ordered_layers_islands_to_print.emplace(ordered_layers_islands_to_print.begin() + i + 1);
                            to_add.emplace(to_add.begin() + i + 1, ObjectLayerToPrint());
                            finished.emplace(finished.begin() + i + 1, false);
                            extruder.insert(extruder.begin() + i + 1, group.extruder_id);
                            fn_add_group_into(to_add[i+1].value(), group);
                            was_added = true;
                            assert(ordered_layers_islands_to_print[i].front().extruders_order.back() == group.extruder_id);
                            assert(extruder[i] == group.extruder_id);
                            if (ordered_layers_islands_to_print[i].front().extruders_order.size() > 1) {
                                assert(ordered_layers_islands_to_print[i].front().extruders_order.size() == 2);
                            }
                            to_add[i + 1].value().extruders_order = {group.extruder_id};
                        }
                        //end research
                        break;
                    } else {
                        already_in_another = already_in_another || removed_at_least_one;
                    }
                }
                assert(still_needed.empty());
            }
            assert(was_added);
        }

        // add new layer groups from to_add
        assert(ordered_layers_islands_to_print.size() == to_add.size());
        assert(finished.size() == to_add.size());
        assert(extruder.size() == to_add.size());
        for (size_t i = 0; i < to_add.size(); i++) {
            if (to_add[i].has_value()) {
                ordered_layers_islands_to_print[i].push_back(std::move(to_add[i].value()));
            } else {
                assert(!ordered_layers_islands_to_print[i].empty());
                // test if this layer can't have new ones, before of air gap
                // nope, a hiher layer may be very thick
                //if (min_bottom_z(layer) > ordered_layers_islands_to_print[i].back().layer()->scaled_print_z()) {
                //    set_finished(i);
                //}
            }
        }

        previous_extruder_id = current_extruder_id;
    }

    return ordered_layers_islands_to_print;
}
}}}
