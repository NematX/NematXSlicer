///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/ Copyright (c) 2021 Ultimaker B.V.
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

// Layer dependencies require top-down construction before surface calls can share immutable trees.
#include "LightningGenerator.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Plugins/Common/ClipperHelpers.hpp"

namespace slic3r_api { namespace Infill { namespace Lightning {

std::unique_ptr<Cache> build_cache(const Inputs &inputs, storage_handle *scratch,
                                  const plugin_run_context *run_ctx)
{
    namespace clipper = PluginCommon::clipper;
    std::unique_ptr<Cache> cache = std::make_unique<Cache>();
    cache->layers.resize(inputs.layers.size());
    std::vector<work::Polygons> overhangs(inputs.layers.size());
    work::Polygons above;
    for (size_t index = inputs.layers.size(); index-- > 0;) {
        throw_if_cancelled(run_ctx);
        const LayerInput &layer = inputs.layers[index];
        const StoredPolygonCollection current = clipper::to_stored_polygons(scratch, layer.outlines);
        const StoredPolygonCollection upper = clipper::to_stored_polygons(scratch, above);
        const ClipperOperand inset = clipper_offset(ClipperOperand(scratch, current.readonly()), -double(layer.wall_radius));
        const ClipperOperand difference = clipper_diff(inset, ClipperOperand(scratch, upper.readonly()));
        const StoredExPolygonCollection opened = clipper_offset2(difference, -double(SCALED_EPSILON), double(SCALED_EPSILON)).to_expolygon_collection();
        overhangs[index] = clipper::to_polygons(opened.readonly());
        above = layer.outlines;
        cache->layer_indices.emplace(layer.identity, index);
    }
    if (inputs.layers.empty())
        return cache;
    grid::Grid locator;
    work::BoundingBox bounds = clipper::get_extents(inputs.layers.back().outlines);
    bounds.offset(SCALED_EPSILON);
    locator.set_bbox(bounds);
    locator.create(inputs.layers.back().outlines, locator_cell_size);
    for (size_t index = inputs.layers.size(); index-- > 0;) {
        throw_if_cancelled(run_ctx);
        const LayerInput &input = inputs.layers[index];
        LayerTree &layer = cache->layers[index];
        const std::vector<NodePtr> previous = layer.roots;
        layer.generate(overhangs[index], input.outlines, locator, inputs.parameters.supporting_radius,
                       input.wall_radius, scratch, run_ctx);
        layer.reconnect(previous, input.outlines, locator, inputs.parameters.supporting_radius, input.wall_radius, run_ctx);
        if (index == 0)
            break;
        work::BoundingBox below = clipper::get_extents(inputs.layers[index - 1].outlines);
        below.offset(SCALED_EPSILON);
        if (locator.bbox().defined) {
            below.merge(locator.bbox().min);
            below.merge(locator.bbox().max);
        }
        for (const NodePtr &root : layer.roots) {
            work::BoundingBox tree_bounds = root->bounds();
            tree_bounds.offset(SCALED_EPSILON);
            below.merge(tree_bounds.min);
            below.merge(tree_bounds.max);
        }
        locator.set_bbox(below);
        locator.create(inputs.layers[index - 1].outlines, locator_cell_size);
        for (const NodePtr &root : layer.roots)
            root->propagate(cache->layers[index - 1].roots, inputs.layers[index - 1].outlines, locator,
                            input.prune_length, input.straightening_distance, run_ctx);
    }
    return cache;
}
}}}
