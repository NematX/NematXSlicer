///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/ Copyright (c) 2021 Ultimaker B.V.
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

// Keep sequential grounding and reconnection tie decisions synchronized with Fill/Lightning/LightningLayer.cpp.
#include "LightningLayer.hpp"
#include "LightningDistanceField.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Plugins/Common/ClipperHelpers.hpp"

namespace slic3r_api { namespace Infill { namespace Lightning {

void LayerTree::populate(NodeGrid &nodes, const work::BoundingBox &bounds)
{
    for (const NodePtr &root : roots)
        root->visit([&](const NodePtr &node) {
            nodes.emplace((node->location - bounds.min) / locator_cell_size, node);
        });
}

LayerTree::Ground LayerTree::best_ground(const work::Point &point, const work::Polygons &outlines,
                                         const grid::Grid &locator, coord_t radius, coord_t wall_radius,
                                         const NodeGrid &nodes, const NodePtr &exclude, const plugin_run_context *run_ctx)
{
    const work::BoundingBox bounds = PluginCommon::clipper::get_extents(outlines);
    work::Point ground;
    double minimum_squared = std::numeric_limits<double>::max();
    for (const work::Polygon &contour : outlines) {
        if (contour.size() <= 2)
            continue;
        work::Point previous = contour.points.back();
        for (const work::Point &current : contour.points) {
            work::Point candidate;
            const double squared = distance_to_segment(previous, current, point, &candidate);
            if (squared < minimum_squared) {
                minimum_squared = squared;
                ground = candidate;
            }
            previous = current;
        }
    }
    const coord_t initial_distance = coord_t(ground.distance_to(point));
    coord_t minimum = initial_distance;
    NodePtr best;
    work::Point best_address(std::numeric_limits<coord_t>::lowest(), std::numeric_limits<coord_t>::lowest());
    if (minimum >= wall_radius) {
        const work::Point diagonal(minimum, minimum);
        const work::Point begin = (point - diagonal - bounds.min) / locator_cell_size;
        const work::Point end = (point + diagonal + work::Point(locator_cell_size, locator_cell_size) - bounds.min) / locator_cell_size;
        for (coord_t y = begin.y(); y < end.y(); ++y) {
            throw_if_cancelled(run_ctx);
            for (coord_t x = begin.x(); x < end.x(); ++x) {
                const work::Point address(x, y);
                NodePtr candidate;
                coord_t local = initial_distance;
                const std::pair<NodeGrid::const_iterator, NodeGrid::const_iterator> range = nodes.equal_range(address);
                for (NodeGrid::const_iterator it = range.first; it != range.second; ++it) {
                    const NodePtr node = it->second.lock();
                    if (!node || node == exclude || (exclude && exclude->has_offspring(node)) ||
                        boundary_collision(point, node->location, locator))
                        continue;
                    const coord_t weighted = node->weighted_distance(point, radius);
                    if (weighted < local) {
                        local = weighted;
                        candidate = node;
                    }
                }
                // Match the row-major tie reduction of the native parallel query.
                if (local < minimum || (local == minimum && (y < best_address.y() ||
                    (y == best_address.y() && x < best_address.x())))) {
                    minimum = local;
                    best_address = address;
                    best = candidate;
                }
            }
        }
    }
    return {best, ground};
}

void LayerTree::generate(const work::Polygons &overhang, const work::Polygons &outlines, const grid::Grid &locator,
                         coord_t radius, coord_t wall_radius, storage_handle *scratch, const plugin_run_context *run_ctx)
{
    const work::BoundingBox bounds = PluginCommon::clipper::get_extents(outlines);
    DistanceField field(radius, bounds, overhang, scratch, run_ctx);
    NodeGrid nodes;
    populate(nodes, bounds);
    size_t index = 0;
    work::Point point;
    while (field.next(point, index, index)) {
        throw_if_cancelled(run_ctx);
        const Ground ground = best_ground(point, outlines, locator, radius, wall_radius, nodes, {}, run_ctx);
        NodePtr child;
        NodePtr new_root;
        if (ground.node) {
            child = ground.node->add_child(point);
        } else {
            new_root = std::make_shared<Node>(ground.boundary, ground.boundary);
            child = new_root->add_child(point);
            roots.push_back(new_root);
        }
        nodes.emplace((child->location - bounds.min) / locator_cell_size, child);
        if (new_root)
            nodes.emplace((new_root->location - bounds.min) / locator_cell_size, new_root);
        field.update(ground.point(), point);
    }
}

void LayerTree::reconnect(const std::vector<NodePtr> &previous, const work::Polygons &outlines, const grid::Grid &locator,
                          coord_t radius, coord_t wall_radius, const plugin_run_context *run_ctx)
{
    NodeGrid nodes;
    const work::BoundingBox bounds = PluginCommon::clipper::get_extents(outlines);
    populate(nodes, bounds);
    for (const NodePtr &root : previous) {
        throw_if_cancelled(run_ctx);
        const std::vector<NodePtr>::iterator old = std::find(roots.begin(), roots.end(), root);
        assert(old != roots.end());
        if (root->grounding && *root->grounding != root->location) {
            work::Point hit;
            if (boundary_intersection(root->location, *root->grounding, locator, hit, locator.resolution() * 2)) {
                const NodePtr new_root = std::make_shared<Node>(hit, hit);
                root->attach(new_root);
                new_root->reroot();
                nodes.emplace((new_root->location - bounds.min) / locator_cell_size, new_root);
                *old = new_root;
                continue;
            }
        }
        const Ground ground = best_ground(root->location, outlines, locator, radius, wall_radius - 100, nodes, root, run_ctx);
        if (!ground.node) {
            if (ground.boundary == root->location)
                continue;
            const NodePtr new_root = std::make_shared<Node>(ground.boundary, ground.boundary);
            const NodePtr attachment = root->closest(new_root->location);
            attachment->reroot();
            new_root->attach(attachment);
            nodes.emplace((new_root->location - bounds.min) / locator_cell_size, new_root);
            *old = new_root;
        } else {
            const NodePtr attachment = root->closest(ground.node->location);
            attachment->reroot();
            ground.node->attach(attachment);
            *old = std::move(roots.back());
            roots.pop_back();
        }
    }
}

work::Polylines LayerTree::paths(coord_t overlap, const plugin_run_context *run_ctx) const
{
    work::Polylines output;
    for (const NodePtr &root : roots) {
        work::Polylines branches = root->paths(overlap, run_ctx);
        output.insert(output.end(), std::make_move_iterator(branches.begin()), std::make_move_iterator(branches.end()));
    }
    return output;
}
}}}
