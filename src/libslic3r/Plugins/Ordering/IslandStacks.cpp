///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#include "IslandStacks.hpp"

#include <algorithm>
#include <cassert>
#include <limits>
#include <map>
#include <stdexcept>
#include "libslic3r/Plugins/Common/ClipperHelpers.hpp"

/*
Builds plate-wide island stacks for the By object builder from borrowed layers.
A unit is one printable source island of one physical instance. All units and Z
levels are collected before lower-island links are resolved within each instance.
Temporary translated polygons belong to this call; returned selections borrow
their source layers and islands and retain the physical instance identity.
Each level groups islands without enough nozzle clearance, regardless of tool.
Lower links extend parent stacks; the current tops of other open stacks participate
in the clearance search at their actual Z. Height differences close stacks before
further layers can extend them. Tool ordering is applied only after grouping.
If no complete remaining stack is safe, one group prints the lowest remaining Z
across all stacks. This breaks history and parent-order deadlocks without adding lead.

    separate_islands() -> collect units, group each Z and update open stacks
        fn_group_islands() -> combine units without enough nozzle clearance
        too_close() -> compare translated slices at the requested height lead
        set_finished() -> remove closed units from the parent search
        fn_add_group_into() -> retain source islands and instance placements
*/
namespace slic3r_api { namespace Ordering { namespace IslandStacks {

namespace {
struct Unit
{
    size_t source = 0;
    Layer layer;
    LayerIsland island;
    uint32_t instance_id = 0;
    c_bounding_box bb;
    PluginCommon::polygon_work::ExPolygons slices;
    std::set<extruder_index_t> extruders;
    std::vector<size_t> parents;
};

struct GroupedIslands
{
    std::vector<size_t> islands;
    c_bounding_box bb;
};

// Compare owned translated slices using the larger clearance requested by either
// unit's tools. Unknown tools use every profile. The positive requested height
// does not exceed max_height, which the augmented graphs cover. Bounding boxes
// reject distances beyond every profile before inverse queries. Geometry allocation
// and API failures propagate without changing source layers.
bool too_close(const Unit &island, const Unit &other, coord_t height,
    const std::vector<GraphData> &extruder_min_dist, storage_handle *storage);

} // namespace

// Collect all plate units before linking parents, then group each Z by clearance
// and extend or close stacks using lower links and other open stacks' current tops.
// Input excludes work already emitted by the caller. Returned selections borrow
// source views; allocation, invalid tool identities and API failures propagate.
std::vector<ObjectsLayerToPrint> separate_islands(
    const ObjectsLayerToPrint object_layers, coord_t max_height,
    const std::vector<GraphData> &extruder_min_dist, extruder_index_t initial_extruder_id,
    storage_handle *storage);

std::vector<ObjectsLayerToPrint> separate_islands(
    const ObjectsLayerToPrint object_layers, coord_t max_height,
    const std::vector<GraphData> &extruder_min_dist, extruder_index_t initial_extruder_id,
    storage_handle *storage)
{
    std::vector<Unit> units;
    std::map<coord_t, std::vector<size_t>> levels;
    std::map<std::pair<uint32_t, const layer_island_handle *>, size_t> unit_by_island;
    // Finish the unit table before resolving links so every identity stays stable.
    for (size_t source = 0; source < object_layers.size(); ++source) {
        const ObjectLayerToPrint &selection = object_layers[source];
        for (const Layer &layer : {selection.object_layer, selection.auxiliary_layer}) {
            if (!layer.valid())
                continue;
            for (uint32_t idx = 0; idx < layer.island_count(); ++idx) {
                const LayerIsland island = layer.island(idx);
                bool has_extrusions = false;
                for (uint32_t region = 0; region < island.region_island_count(); ++region)
                    has_extrusions |= island.region_island(region).has_extrusions();
                if (!has_extrusions)
                    continue;
                Unit unit;
                unit.source = source;
                unit.layer = layer;
                unit.island = island;
                unit.instance_id = selection.instance_id;
                unit.bb = island.bounding_box();
                unit.bb.min.x += selection.shift.x;
                unit.bb.max.x += selection.shift.x;
                unit.bb.min.y += selection.shift.y;
                unit.bb.max.y += selection.shift.y;
                unit.slices.push_back(PluginCommon::clipper::from_expolygon_view(island.slice()));
                for (PluginCommon::polygon_work::ExPolygon &slice : unit.slices) {
                    for (PluginCommon::polygon_work::Point &point : slice.contour.points) {
                        point.x() += selection.shift.x;
                        point.y() += selection.shift.y;
                    }
                    for (PluginCommon::polygon_work::Polygon &hole : slice.holes)
                        for (PluginCommon::polygon_work::Point &point : hole.points) {
                            point.x() += selection.shift.x;
                            point.y() += selection.shift.y;
                        }
                }
                for (uint32_t region = 0; region < island.region_island_count(); ++region) {
                    const extruder_index_t tool = island.region_island(region).extruder_id();
                    if (tool < EXTRUDER_ID_UNKNOWN)
                        throw std::invalid_argument("LayerRegionIsland contains an invalid negative extruder ID");
                    unit.extruders.insert(tool);
                }
                levels[layer.print_z()].push_back(units.size());
                unit_by_island.emplace(std::make_pair(unit.instance_id, island.handle()), units.size());
                units.push_back(std::move(unit));
            }
        }
    }
    for (Unit &unit : units)
        for (const LayerIsland &parent : unit.island.lower_islands()) {
            const auto found = unit_by_island.find(std::make_pair(unit.instance_id, parent.handle()));
            if (found != unit_by_island.end())
                unit.parents.push_back(found->second);
        }

    std::vector<std::vector<size_t>> ordered_layers_islands_to_print;
    std::vector<size_t> top_begin;
    std::vector<bool> finished;
    std::vector<size_t> stack_by_unit(units.size(), size_t(-1));
    std::set<size_t> all_islands;
    // Closed stacks remain in print order but no longer accept lower-link extensions.
    const auto set_finished = [&](size_t idx) {
        finished[idx] = true;
        for (size_t island : ordered_layers_islands_to_print[idx])
            all_islands.erase(island);
    };

    // we explore the graph layer per layer.
    for (const auto &[print_z, level] : levels) {
        std::vector<GroupedIslands> grouped_islands;
        // first, we group islands that are too close together
        const auto fn_group_islands = [&](size_t island) {
            all_islands.insert(island);
            size_t added = size_t(-1);
            for (size_t group_idx = 0; group_idx < grouped_islands.size(); ++group_idx) {
                GroupedIslands &group = grouped_islands[group_idx];
                bool collide = false;
                for (size_t other : group.islands)
                    collide |= too_close(units[island], units[other], max_height, extruder_min_dist, storage);
                if (!collide)
                    continue;
                if (added == size_t(-1)) {
                    // collide -> add it
                    group.islands.push_back(island);
                    bounding_box_merge(group.bb, units[island].bb);
                    added = group_idx;
                } else {
                    // merge the groups
                    GroupedIslands &main_group = grouped_islands[added];
                    main_group.islands.insert(main_group.islands.end(), group.islands.begin(), group.islands.end());
                    bounding_box_merge(main_group.bb, group.bb);
                    grouped_islands.erase(grouped_islands.begin() + group_idx--);
                }
            }
            if (added == size_t(-1))
                grouped_islands.push_back(GroupedIslands{{island}, units[island].bb});
        };
        for (size_t island : level)
            fn_group_islands(island);
        // Keep the island-count and area tie-breaks after tool-independent grouping.
        std::stable_sort(grouped_islands.begin(), grouped_islands.end(), [](const GroupedIslands &g1, const GroupedIslands &g2) {
            if (g1.islands.size() != g2.islands.size())
                return g1.islands.size() > g2.islands.size();
            const c_point size1 = bounding_box_size(g1.bb), size2 = bounding_box_size(g2.bb);
            return double(size1.x) * double(size1.y) > double(size2.x) * double(size2.y);
        });

        // need to check if the new printz may close some too tall stacks
        coord_t thickness = 0;
        for (size_t island : level)
            thickness = std::max(thickness, print_z - units[island].layer.bottom_z());
        for (size_t coll_idx = 0; coll_idx < ordered_layers_islands_to_print.size(); ++coll_idx) {
            if (finished[coll_idx])
                continue;
            for (size_t check = coll_idx + 1; check < ordered_layers_islands_to_print.size(); ++check) {
                if (ordered_layers_islands_to_print[check].empty())
                    continue;
                const coord_t top = units[ordered_layers_islands_to_print[coll_idx].back()].layer.print_z();
                const coord_t bottom = units[ordered_layers_islands_to_print[check].front()].layer.print_z();
                // A later stack starting above this top also closes it. Different
                // layer grids can therefore fragment otherwise continuous stacks.
                if (top - bottom > max_height - thickness || top < bottom) {
                    set_finished(coll_idx);
                    break;
                }
            }
        }

        for (GroupedIslands &group : grouped_islands) {
            std::set<size_t> still_needed, nearby_stacks;
            for (size_t island : group.islands)
                for (size_t parent : units[island].parents)
                    if (all_islands.count(parent))
                        still_needed.insert(stack_by_unit[parent]);
            for (size_t island : group.islands) {
                // Every other open stack participates, including support of the
                // same copy on another Z grid. Parent stacks attach through lower
                // links rather than through these proximity comparisons.
                for (size_t stack = 0; stack < ordered_layers_islands_to_print.size(); ++stack) {
                    if (finished[stack] || still_needed.count(stack) || ordered_layers_islands_to_print[stack].empty())
                        continue;
                    const std::vector<size_t> &previous_layers = ordered_layers_islands_to_print[stack];
                    for (size_t idx = top_begin[stack]; idx < previous_layers.size(); ++idx) {
                        const size_t other = previous_layers[idx];
                        if (too_close(units[island], units[other], max_height, extruder_min_dist, storage))
                            nearby_stacks.insert(stack);
                    }
                }
            }
            size_t destination = size_t(-1);
            if (!nearby_stacks.empty()) {
                // Nearby stacks must share the same layer-by-layer history.
                nearby_stacks.insert(still_needed.begin(), still_needed.end());
                destination = *nearby_stacks.begin();
                for (size_t stack : nearby_stacks) {
                    if (stack == destination)
                        continue;
                    std::vector<size_t> &target = ordered_layers_islands_to_print[destination];
                    std::vector<size_t> &source = ordered_layers_islands_to_print[stack];
                    target.insert(target.end(), source.begin(), source.end());
                    for (size_t island : source)
                        stack_by_unit[island] = destination;
                    source.clear();
                    top_begin[stack] = 0;
                    finished[stack] = true;
                }
            } else if (!still_needed.empty()) {
                // A group following several parents starts after the last parent.
                destination = *still_needed.rbegin();
                if (units[ordered_layers_islands_to_print[destination].back()].layer.print_z() == print_z) {
                    // Separate children of one parent need separate stacks when
                    // their clearance groups at this level do not touch.
                    ++destination;
                    ordered_layers_islands_to_print.emplace(ordered_layers_islands_to_print.begin() + destination);
                    finished.insert(finished.begin() + destination, false);
                    top_begin.insert(top_begin.begin() + destination, 0);
                    for (size_t &stack : stack_by_unit)
                        if (stack != size_t(-1) && stack >= destination)
                            ++stack;
                }
            }
            if (destination == size_t(-1)) {
                destination = ordered_layers_islands_to_print.size();
                ordered_layers_islands_to_print.emplace_back();
                finished.push_back(false);
                top_begin.push_back(0);
            }
            std::vector<size_t> &target = ordered_layers_islands_to_print[destination];
            target.insert(target.end(), group.islands.begin(), group.islands.end());
            for (size_t island : group.islands)
                stack_by_unit[island] = destination;
            std::stable_sort(target.begin(), target.end(), [&](size_t a, size_t b) {
                return units[a].layer.print_z() < units[b].layer.print_z();
            });
            top_begin[destination] = size_t(std::lower_bound(target.begin(), target.end(),
                units[target.back()].layer.print_z(), [&](size_t island, coord_t z) {
                    return units[island].layer.print_z() < z;
                }) - target.begin());
        }
    }

    std::vector<ObjectsLayerToPrint> result;
    extruder_index_t previous_extruder_id = initial_extruder_id;
    // Prefer safe remaining stacks that continue the tool. If histories block
    // every complete stack, the lowest remaining level makes progress safely.
    std::vector<bool> emitted(units.size(), false);
    size_t remaining = units.size();
    auto lowest_level = levels.begin();
    while (remaining != 0) {
        // Excluding one candidate only needs the two lowest remaining stack fronts.
        coord_t lowest_front_z = std::numeric_limits<coord_t>::max();
        coord_t second_front_z = std::numeric_limits<coord_t>::max();
        size_t lowest_front_stack = size_t(-1);
        for (size_t stack = 0; stack < ordered_layers_islands_to_print.size(); ++stack) {
            if (ordered_layers_islands_to_print[stack].empty())
                continue;
            const coord_t z = units[ordered_layers_islands_to_print[stack].front()].layer.print_z();
            if (z < lowest_front_z) {
                second_front_z = lowest_front_z;
                lowest_front_z = z;
                lowest_front_stack = stack;
            } else {
                second_front_z = std::min(second_front_z, z);
            }
        }
        size_t selected = size_t(-1);
        for (size_t stack = 0; stack < ordered_layers_islands_to_print.size(); ++stack) {
            if (ordered_layers_islands_to_print[stack].empty())
                continue;
            bool ready = true;
            for (size_t island : ordered_layers_islands_to_print[stack])
                for (size_t parent : units[island].parents)
                    if (stack_by_unit[parent] != stack && !emitted[parent])
                        ready = false;
            // Tool preferences cannot put a taller stack before lower work that
            // still needs its nozzle clearance. Test the complete candidate lead.
            if (!ready)
                continue;
            const coord_t lowest_other_z = stack == lowest_front_stack ? second_front_z : lowest_front_z;
            if (lowest_other_z != std::numeric_limits<coord_t>::max() &&
                units[ordered_layers_islands_to_print[stack].back()].layer.print_z() - lowest_other_z > max_height)
                ready = false;
            for (size_t island : ordered_layers_islands_to_print[stack]) {
                if (!ready)
                    break;
                const coord_t z = units[island].layer.print_z();
                const auto end = levels.lower_bound(z);
                for (auto level = levels.lower_bound(z - max_height); ready && level != end; ++level)
                    for (size_t other : level->second) {
                        if (emitted[other] || stack_by_unit[other] == stack)
                            continue;
                        if (too_close(units[island], units[other], z - level->first, extruder_min_dist, storage)) {
                            ready = false;
                            break;
                        }
                    }
            }
            if (!ready)
                continue;
            if (selected == size_t(-1))
                selected = stack;
            bool continues_tool = false;
            if (!ordered_layers_islands_to_print[stack].empty()) {
                const coord_t first_z = units[ordered_layers_islands_to_print[stack].front()].layer.print_z();
                for (size_t island : ordered_layers_islands_to_print[stack]) {
                    if (units[island].layer.print_z() != first_z)
                        break;
                    continues_tool |= units[island].extruders.count(previous_extruder_id) != 0;
                }
            }
            if (continues_tool) {
                selected = stack;
                break;
            }
        }
        std::vector<size_t> printable;
        if (selected != size_t(-1)) {
            printable = std::move(ordered_layers_islands_to_print[selected]);
            ordered_layers_islands_to_print[selected].clear();
        } else {
            // Parents are strictly lower. Emitting all work at the lowest Z
            // introduces no positive lead over any remaining unit.
            for (size_t island : lowest_level->second)
                if (!emitted[island])
                    printable.push_back(island);
        }
        ObjectsLayerToPrint selections;
        // Retain a separate source selection per instance, source layer and Z.
        const auto fn_add_group_into = [&](size_t island) {
            const Unit &unit = units[island];
            const ObjectLayerToPrint &source = object_layers[unit.source];
            if (selections.empty() || selections.back().instance_id != source.instance_id ||
                !selections.back().object_layer.same_handle(source.object_layer) ||
                !selections.back().auxiliary_layer.same_handle(source.auxiliary_layer)) {
                selections.push_back(source);
                selections.back().islands.clear();
                selections.back().extruders_order.clear();
            }
            selections.back().islands.insert(unit.island.handle());
        };
        for (size_t island : printable)
            fn_add_group_into(island);
        // A tool bucket is shared by all copies at this Z. A later selection
        // using an existing bucket does not change the layer's last tool.
        for (size_t begin = 0; begin < selections.size();) {
            size_t end = begin + 1;
            while (end < selections.size() && selections[end].layer().print_z() == selections[begin].layer().print_z())
                ++end;
            std::vector<extruder_index_t> tools;
            for (size_t idx = begin; idx < end; ++idx) {
                const ObjectLayerToPrint &selection = selections[idx];
                for (const Layer &layer : {selection.object_layer, selection.auxiliary_layer}) {
                    if (!layer.valid())
                        continue;
                    for (uint32_t island_idx = 0; island_idx < layer.island_count(); ++island_idx) {
                        const LayerIsland island = layer.island(island_idx);
                        if (selection.islands.count(island.handle()) == 0)
                            continue;
                        for (uint32_t region = 0; region < island.region_island_count(); ++region) {
                            const LayerRegionIsland region_island = island.region_island(region);
                            const extruder_index_t tool = region_island.extruder_id();
                            if (region_island.has_extrusions() && std::find(tools.begin(), tools.end(), tool) == tools.end())
                                tools.push_back(tool);
                        }
                    }
                }
            }
            const auto current = std::find(tools.begin(), tools.end(), previous_extruder_id);
            if (current != tools.end() && current != tools.begin()) {
                const extruder_index_t tool = *current;
                tools.erase(current);
                tools.insert(tools.begin(), tool);
            }
            for (size_t idx = begin; idx < end; ++idx)
                selections[idx].extruders_order = tools;
            if (!tools.empty())
                previous_extruder_id = tools.back();
            begin = end;
        }
        result.push_back(std::move(selections));
        for (size_t island : printable) {
            emitted[island] = true;
            --remaining;
        }
        // A fallback level only removes prefixes from the remaining stacks.
        for (std::vector<size_t> &stack : ordered_layers_islands_to_print)
            stack.erase(stack.begin(), std::find_if(stack.begin(), stack.end(), [&](size_t island) {
                return !emitted[island];
            }));
        while (lowest_level != levels.end() && std::all_of(lowest_level->second.begin(), lowest_level->second.end(),
                [&](size_t island) { return emitted[island]; }))
            ++lowest_level;
    }
    return result;
}

namespace {
bool too_close(const Unit &island, const Unit &other, coord_t height,
    const std::vector<GraphData> &extruder_min_dist, storage_handle *storage)
{
    coord_t radius = 0;
    for (const GraphData &graph : extruder_min_dist) {
        const ConfigPoint last = graph.point(graph.size() - 1);
        assert(double(height) * SCALING_FACTOR <= last.y);
        radius = std::max(radius, scale_i(last.x));
    }
    // LINEAR and SQUARE inverses stay within the last X; SPLINE overshoot is clamped to this bounding-box radius.
    c_bounding_box inflated = other.bb;
    inflated.min.x -= radius;
    inflated.min.y -= radius;
    inflated.max.x += radius;
    inflated.max.y += radius;
    if (!bounding_box_overlap(inflated, island.bb))
        return false;
    coord_t min_dist = 0;
    for (const Unit *unit : {&island, &other}) {
        for (extruder_index_t tool : unit->extruders) {
            if (tool == EXTRUDER_ID_UNKNOWN) {
                for (const GraphData &graph : extruder_min_dist) {
                    min_dist = std::max(min_dist, scale_i(graph.inverse_interpolate(
                        double(height) * SCALING_FACTOR)));
                }
            } else {
                assert(size_t(tool) < extruder_min_dist.size());
                const GraphData &graph = extruder_min_dist[tool];
                min_dist = std::max(min_dist, scale_i(graph.inverse_interpolate(
                    double(height) * SCALING_FACTOR)));
            }
        }
    }
    min_dist = std::min(min_dist, radius);
    inflated = other.bb;
    inflated.min.x -= min_dist;
    inflated.min.y -= min_dist;
    inflated.max.x += min_dist;
    inflated.max.y += min_dist;
    return bounding_box_overlap(inflated, island.bb) &&
        !PluginCommon::clipper::intersection_ex(storage,
            PluginCommon::clipper::offset_ex(storage, island.slices, double(min_dist)), other.slices).empty();
}
} // namespace
}}}
