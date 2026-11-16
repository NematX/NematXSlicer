///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#include "IslandStacks.hpp"

#include <algorithm>
#include <cassert>
#include <iterator>
#include <limits>
#include <map>
#include <stdexcept>
#include <tuple>
#include "libslic3r/Plugins/Common/ClipperHelpers.hpp"

/*
Builds plate-wide island stacks for the By object builder from borrowed layers.
A unit contains either one printable island or every selected island of a physical
instance at one Z. Members retain their source selection and island for filtered emission.
All units and Z levels are collected before lower-island links are resolved
within each instance.
Temporary translated polygons belong to this call; returned selections borrow
their source layers and islands and retain the physical instance identity.
Levels are visited in increasing Z. Lower links extend stacks while their units
still have unprocessed children. Exhausted stacks remain visible for clearance.
An extension that would exceed the height lead over a later stack starts at the
end instead; its old stack stops accepting children but stays visible for proximity.
Clearance compares each lower layer of a later stack with all higher layers of
the earlier stack at the actual height difference. Equal-Z geometry is not a
height obstacle. New units use the bounded Z-level index; only merges recheck
their histories in that window. A lead above max_height is a collision without
a graph query only inside the cached radius envelope; far histories stay separate
for emission to order. Intersecting any layer is equivalent to intersecting their union.
Tool ordering is applied only after grouping.
An optional selected tool adds its clearance profile to those of the source
tools for every geometric query; it does not change tool ordering or source data.
If no complete remaining stack is safe, one group prints the lowest remaining Z
across all stacks. This breaks history and parent-order deadlocks without adding lead.

    separate_islands() -> collect units, group each Z and update open stacks
        too_close() -> compare translated slices at the requested height lead
        fn_add_group_into() -> retain source islands and instance placements
*/
namespace slic3r_api { namespace Ordering { namespace IslandStacks {

namespace {
struct Member
{
    size_t source = 0;
    LayerIsland island;
};

struct Unit
{
    std::vector<Member> members;
    Layer layer;
    uint32_t instance_id = 0;
    uint32_t copy_rank = 0;
    c_bounding_box bb;
    PluginCommon::polygon_work::ExPolygons slices;
    std::set<extruder_index_t> extruders;
    std::vector<size_t> parents;
};

// Compare owned translated slices using the larger clearance requested by either
// unit's tools and the optional clearance_tool. Unknown source tools use every profile.
// A specified clearance_tool indexes extruder_min_dist. The positive requested height
// does not exceed max_height or the cached final graph height: nearby larger leads
// are collisions before this geometric query. The cached maximum radius lets bounding boxes
// reject distances beyond every profile before inverse queries. Geometry allocation
// and API failures propagate without changing source layers.
bool too_close(const Unit &island, const Unit &other, coord_t height,
    const std::vector<GraphData> &extruder_min_dist, coord_t radius, double graph_height,
    extruder_index_t clearance_tool, storage_handle *storage);

} // namespace

// Collect plate units before following lower links in Z order.
// Admit continuations by their actual top lead and merge obstructing histories
// using real height differences. Exhausted stacks remain visible for proximity.
// Input excludes work already emitted by the caller. Returned selections borrow
// source views; clearance_tool adds a profile without changing their tool identities.
// Allocation, invalid tool identities and API failures propagate.
std::vector<ObjectsLayerToPrint> separate_islands(
    const ObjectsLayerToPrint object_layers, coord_t max_height,
    const std::vector<GraphData> &extruder_min_dist, extruder_index_t initial_extruder_id,
    storage_handle *storage, bool complete_islands, extruder_index_t clearance_tool);

std::vector<ObjectsLayerToPrint> separate_islands(
    const ObjectsLayerToPrint object_layers, coord_t max_height,
    const std::vector<GraphData> &extruder_min_dist, extruder_index_t initial_extruder_id,
    storage_handle *storage, bool complete_islands, extruder_index_t clearance_tool)
{
    coord_t graph_radius = 0;
    double graph_height = std::numeric_limits<double>::max();
    for (const GraphData &graph : extruder_min_dist) {
        const ConfigPoint last = graph.point(graph.size() - 1);
        graph_radius = std::max(graph_radius, scale_i(last.x));
        graph_height = std::min(graph_height, last.y);
    }
    std::vector<Unit> units;
    std::map<coord_t, std::vector<size_t>> levels;
    std::map<std::pair<uint32_t, const layer_island_handle *>, size_t> unit_by_island;
    std::map<std::pair<uint32_t, coord_t>, size_t> unit_by_level;
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
                unit.members.push_back(Member{source, island});
                unit.layer = layer;
                unit.instance_id = selection.instance_id;
                unit.copy_rank = selection.copy_rank;
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
                const auto key = std::make_pair(unit.instance_id, layer.print_z());
                const auto existing = unit_by_level.find(key);
                if (!complete_islands && existing != unit_by_level.end()) {
                    Unit &target = units[existing->second];
                    target.members.insert(target.members.end(), unit.members.begin(), unit.members.end());
                    bounding_box_merge(target.bb, unit.bb);
                    target.slices.insert(target.slices.end(), std::make_move_iterator(unit.slices.begin()),
                        std::make_move_iterator(unit.slices.end()));
                    target.extruders.insert(unit.extruders.begin(), unit.extruders.end());
                } else {
                    levels[layer.print_z()].push_back(units.size());
                    if (complete_islands)
                        unit_by_island.emplace(std::make_pair(unit.instance_id, island.handle()), units.size());
                    unit_by_level.emplace(key, units.size());
                    units.push_back(std::move(unit));
                }
            }
        }
    }
    if (complete_islands) {
        for (Unit &unit : units)
            for (const Member &member : unit.members)
                for (const LayerIsland &parent : member.island.lower_islands()) {
                    const auto found = unit_by_island.find(std::make_pair(unit.instance_id, parent.handle()));
                    if (found != unit_by_island.end())
                        unit.parents.push_back(found->second);
                }
    } else {
        std::map<uint32_t, size_t> previous;
        for (const auto &[key, idx] : unit_by_level) {
            const auto parent = previous.find(key.first);
            if (parent != previous.end())
                units[idx].parents.push_back(parent->second);
            previous[key.first] = idx;
        }
    }

    std::vector<std::vector<size_t>> ordered_layers_islands_to_print;
    std::vector<bool> closed_for_children;
    std::vector<size_t> stack_by_unit(units.size(), size_t(-1));

    // Exhausted stacks have no more continuations but remain visible for proximity.
    // Refusing an admission closes only lower-link continuation, never visibility.
    for (const auto &[print_z, level] : levels) {
        std::vector<size_t> ordered_level = level;
        // Respect the configured copy order before the island-count and area ties.
        std::stable_sort(ordered_level.begin(), ordered_level.end(), [&](size_t a, size_t b) {
            const Unit &g1 = units[a], &g2 = units[b];
            if (g1.copy_rank != g2.copy_rank)
                return g1.copy_rank < g2.copy_rank;
            if (g1.members.size() != g2.members.size())
                return g1.members.size() > g2.members.size();
            const c_point size1 = bounding_box_size(g1.bb), size2 = bounding_box_size(g2.bb);
            return double(size1.x) * double(size1.y) > double(size2.x) * double(size2.y);
        });

        for (size_t island : ordered_level) {
            std::set<size_t> parents, candidate_stacks, collisions;
            for (size_t parent : units[island].parents)
                if (!closed_for_children[stack_by_unit[parent]])
                    parents.insert(stack_by_unit[parent]);
            size_t destination = ordered_layers_islands_to_print.size();
            bool inserting = false;
            if (!parents.empty()) {
                destination = *parents.rbegin();
                bool sibling = false;
                for (size_t previous : ordered_layers_islands_to_print[destination])
                    if (units[previous].layer.print_z() == print_z)
                        for (size_t parent : units[island].parents)
                            sibling |= std::find(units[previous].parents.begin(), units[previous].parents.end(), parent) != units[previous].parents.end();
                if (sibling) {
                    ++destination;
                    inserting = true;
                } else {
                    candidate_stacks.insert(destination);
                }
            }
            // A new top must stay within max_height above each later stack's first Z
            // in this provisional order, including middle sibling insertions.
            const auto admission_fits = [&](size_t position, coord_t new_top, bool insert) {
                for (size_t later = position + (insert ? 0 : 1); later < ordered_layers_islands_to_print.size(); ++later)
                    if (!candidate_stacks.count(later) && !ordered_layers_islands_to_print[later].empty() &&
                        new_top - units[ordered_layers_islands_to_print[later].front()].layer.print_z() > max_height)
                        return false;
                return true;
            };
            if (!admission_fits(destination, print_z, inserting)) {
                for (size_t stack : candidate_stacks)
                    closed_for_children[stack] = true;
                candidate_stacks.clear();
                destination = ordered_layers_islands_to_print.size();
                inserting = false;
            }
            // New work only adds pairs with earlier Z in the bounded levels index.
            // After a merge, ordering changes may also expose the opposite direction.
            // A negative window result remains valid for this unit and stack until
            // their relative provisional order changes; new merged units still run.
            std::set<std::tuple<size_t, size_t, bool>> compared_windows;
            const auto compare_window = [&](size_t current, bool merged) {
                const coord_t z = units[current].layer.print_z();
                std::set<size_t> tested_stacks;
                for (auto at = levels.lower_bound(z - max_height); at != levels.end() && at->first <= z + (merged ? max_height : 0); ++at) {
                    if (at->first == z)
                        continue;
                    for (size_t other : at->second) {
                        const size_t stack = stack_by_unit[other];
                        if (stack == size_t(-1) || candidate_stacks.count(stack) || collisions.count(stack))
                            continue;
                        const bool other_later = stack >= destination;
                        if (compared_windows.count({current, stack, other_later}))
                            continue;
                        const coord_t dz = other_later ? z - at->first : at->first - z;
                        if (dz > 0) {
                            tested_stacks.insert(stack);
                            if (too_close(units[current], units[other], dz,
                                    extruder_min_dist, graph_radius, graph_height, clearance_tool, storage))
                                collisions.insert(stack);
                        }
                    }
                }
                for (size_t stack : tested_stacks)
                    if (!collisions.count(stack))
                        compared_windows.emplace(current, stack, stack >= destination);
            };
            compare_window(island, false);
            bool merging = !collisions.empty();
            bool reopen_destination = false;
            while (!collisions.empty()) {
                candidate_stacks.insert(collisions.begin(), collisions.end());
                candidate_stacks.insert(parents.begin(), parents.end());
                collisions.clear();
                destination = *candidate_stacks.begin();
                inserting = false;
                coord_t top = print_z, first_z = print_z;
                c_bounding_box merged_box = units[island].bb;
                for (size_t stack : candidate_stacks) {
                    const std::vector<size_t> &history = ordered_layers_islands_to_print[stack];
                    top = std::max(top, units[history.back()].layer.print_z());
                    first_z = std::min(first_z, units[history.front()].layer.print_z());
                    for (size_t current : history)
                        bounding_box_merge(merged_box, units[current].bb);
                }
                // A merged history that cannot fit at its earliest source position
                // moves to the end, rather than bypassing the admission rule.
                reopen_destination = admission_fits(destination, top, false);
                if (!reopen_destination)
                    destination = ordered_layers_islands_to_print.size();
                // Excess lead is not a reason to absorb geometrically remote work.
                merged_box.min.x -= graph_radius;
                merged_box.max.x += graph_radius;
                merged_box.min.y -= graph_radius;
                merged_box.max.y += graph_radius;
                for (size_t stack = 0; stack < ordered_layers_islands_to_print.size(); ++stack) {
                    const std::vector<size_t> &history = ordered_layers_islands_to_print[stack];
                    if (candidate_stacks.count(stack) || history.empty() || stack >= destination)
                        continue;
                    const coord_t dz = units[history.back()].layer.print_z() - first_z;
                    if (dz > max_height) {
                        c_bounding_box other_box = units[history.front()].bb;
                        for (size_t current : history)
                            bounding_box_merge(other_box, units[current].bb);
                        if (bounding_box_overlap(merged_box, other_box))
                            collisions.insert(stack);
                    }
                }
                compare_window(island, true);
                for (size_t stack : candidate_stacks)
                    for (size_t current : ordered_layers_islands_to_print[stack])
                        compare_window(current, true);
            }
            if (inserting) {
                ordered_layers_islands_to_print.emplace(ordered_layers_islands_to_print.begin() + destination);
                closed_for_children.insert(closed_for_children.begin() + destination, false);
                for (size_t &stack : stack_by_unit)
                    if (stack != size_t(-1) && stack >= destination)
                        ++stack;
            } else if (destination == ordered_layers_islands_to_print.size()) {
                ordered_layers_islands_to_print.emplace_back();
                closed_for_children.push_back(false);
            }
            if (merging)
                for (size_t stack : candidate_stacks) {
                    if (stack == destination)
                        continue;
                    std::vector<size_t> &source = ordered_layers_islands_to_print[stack];
                    ordered_layers_islands_to_print[destination].insert(ordered_layers_islands_to_print[destination].end(), source.begin(), source.end());
                    for (size_t current : source)
                        stack_by_unit[current] = destination;
                    source.clear();
                    closed_for_children[stack] = true;
                }
            if (merging && reopen_destination)
                closed_for_children[destination] = false;
            std::vector<size_t> &target = ordered_layers_islands_to_print[destination];
            target.push_back(island);
            stack_by_unit[island] = destination;
            if (merging)
                std::stable_sort(target.begin(), target.end(), [&](size_t a, size_t b) {
                    return units[a].layer.print_z() < units[b].layer.print_z();
                });
        }
    }

    std::vector<ObjectsLayerToPrint> result;
    std::vector<size_t> stack_order;
    std::vector<uint32_t> stack_rank(ordered_layers_islands_to_print.size(), std::numeric_limits<uint32_t>::max());
    for (size_t stack = 0; stack < ordered_layers_islands_to_print.size(); ++stack) {
        stack_order.push_back(stack);
        for (size_t island : ordered_layers_islands_to_print[stack])
            stack_rank[stack] = std::min(stack_rank[stack], units[island].copy_rank);
    }
    std::stable_sort(stack_order.begin(), stack_order.end(), [&](size_t a, size_t b) {
        return stack_rank[a] < stack_rank[b];
    });
    extruder_index_t previous_extruder_id = initial_extruder_id;
    // Prefer ready stacks that continue the tool, then copy rank and the existing order. If histories block
    // every complete stack, the lowest remaining level makes progress safely.
    std::vector<bool> emitted(units.size(), false);
    size_t remaining = units.size();
    auto lowest_level = levels.begin();
    while (remaining != 0) {
        // Excluding one candidate only needs the two lowest remaining stack fronts.
        coord_t lowest_front_z = std::numeric_limits<coord_t>::max();
        coord_t second_front_z = std::numeric_limits<coord_t>::max();
        size_t lowest_front_stack = size_t(-1);
        for (size_t stack : stack_order) {
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
        for (size_t stack : stack_order) {
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
                        if (too_close(units[island], units[other], z - level->first, extruder_min_dist, graph_radius, graph_height, clearance_tool, storage)) {
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
            for (const Member &member : unit.members) {
                const ObjectLayerToPrint &source = object_layers[member.source];
                if (selections.empty() || selections.back().instance_id != source.instance_id ||
                    !selections.back().object_layer.same_handle(source.object_layer) ||
                    !selections.back().auxiliary_layer.same_handle(source.auxiliary_layer)) {
                    selections.push_back(source);
                    selections.back().islands.clear();
                    selections.back().extruders_order.clear();
                }
                selections.back().islands.insert(member.island.handle());
            }
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
    const std::vector<GraphData> &extruder_min_dist, coord_t radius, double graph_height,
    extruder_index_t clearance_tool, storage_handle *storage)
{
    assert(double(height) * SCALING_FACTOR <= graph_height);
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
    if (clearance_tool != EXTRUDER_ID_UNKNOWN) {
        const GraphData &graph = extruder_min_dist.at(size_t(clearance_tool));
        min_dist = std::max(min_dist, scale_i(graph.inverse_interpolate(double(height) * SCALING_FACTOR)));
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
