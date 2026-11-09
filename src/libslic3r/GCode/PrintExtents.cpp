///|/ Copyright (c) SuperSlicer 2026 Durand R?mi @supermerill
///|/ Copyright (c) Prusa Research 2017 - 2022 Vojtěch Bubník @bubnikv, Lukáš Matěna @lukasmatena
///|/ Copyright (c) 2019 Thomas Moore
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "PrintExtents.hpp"

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityVisitors.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/PointUtils.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"

/* Extent queries walk stored paths with inherited attributes and expand their
   XY bounds by half the line width. Object queries apply instance translations
   and an optional layer-Z limit without rebuilding the extrusion hierarchy.
   Property-only commands have no local position; missing attributes and empty
   local bounds contribute nothing. All roots remain borrowed and unchanged.

       get_print_extrusions_extents() / get_print_object_extrusions_extents()
           `-- get_extrusion_extents()
               `-- ExtrusionExtentVisitor::visit_leaf()
                   `-- extrusion_polyline_extents() [expand by effective half-width]
*/
namespace Slic3r {

namespace {

// Accumulate projected paths while the existing tree visitor resolves scopes.
class ExtrusionExtentVisitor final : public ExtrusionTreeConstVisitor<true>
{
public:
    BoundingBox bounds;

    // Include local geometry only; property-only leaves retain no position.
    void visit_leaf(const ExtrusionEntity &entity) override;
};

// Expand every projected point by the supplied half-width in scaled coordinates.
BoundingBox extrusion_polyline_extents(const Polyline &polyline, coord_t radius);

} // namespace

BoundingBoxf get_extrusion_extents(const ExtrusionEntity &entity)
{
    ExtrusionExtentVisitor visitor;
    visitor.traverse(entity);
    BoundingBoxf result;
    if (!empty(visitor.bounds)) {
        result.min = unscale_p(visitor.bounds.min);
        result.max = unscale_p(visitor.bounds.max);
        result.defined = true;
    }
    return result;
}

namespace {

BoundingBox extrusion_polyline_extents(const Polyline &polyline, const coord_t radius)
{
    BoundingBox bbox;
    if (! polyline.points.empty())
        bbox.merge(polyline.points.front());
    for (const Point &pt : polyline.points) {
        bbox.min(0) = std::min(bbox.min(0), pt(0) - radius);
        bbox.min(1) = std::min(bbox.min(1), pt(1) - radius);
        bbox.max(0) = std::max(bbox.max(0), pt(0) + radius);
        bbox.max(1) = std::max(bbox.max(1), pt(1) + radius);
    }
    return bbox;
}

void ExtrusionExtentVisitor::visit_leaf(const ExtrusionEntity &entity)
{
    const ArcPolyline *polyline = entity.polyline_or_null();
    const ExtrusionAttributes *attributes = current_property<ExtrusionAttributes>();
    if (polyline != nullptr && attributes != nullptr) {
        const BoundingBox projected = extrusion_polyline_extents(polyline->to_polyline(), scale_i(0.5 * attributes->width));
        if (!empty(projected))
            bounds.merge(projected);
    }
}

} // namespace

BoundingBoxf get_print_extrusions_extents(const Print &print)
{
    BoundingBoxf bbox(get_extrusion_extents(print.brim()));
    bbox.merge(get_extrusion_extents(print.skirt()));
    return bbox;
}

BoundingBoxf get_print_object_extrusions_extents(const PrintObject &print_object, const coord_t max_print_z)
{
    BoundingBoxf bbox;
    for (const Layer &layer : print_object.layers()) {
        if (layer.scaled_print_z() > max_print_z)
            break;
        BoundingBoxf bbox_this;
        for (const LayerSliceIsland &layer_island_ptr : layer.islands()) {
            for (const LayerRegionIsland &region_island_ptr : layer_island_ptr.regions_islands()) {
                if (region_island_ptr.has_extrusion(LayerRegionIsland::PERIMETERS)) {
                    bbox_this.merge(get_extrusion_extents(region_island_ptr.extrusion(LayerRegionIsland::PERIMETERS)));
                }
                if (region_island_ptr.has_extrusion(LayerRegionIsland::INFILLS)) {
                    bbox_this.merge(get_extrusion_extents(region_island_ptr.extrusion(LayerRegionIsland::INFILLS)));
                }
                // 2 next take care of the case 'layer.get_property<LayerSupportProperty>() != nullptr;'
                if (region_island_ptr.has_extrusion(LayerRegionIsland::SUPPORT)) {
                    bbox_this.merge(get_extrusion_extents(region_island_ptr.extrusion(LayerRegionIsland::SUPPORT)));
                }
                if (region_island_ptr.has_extrusion(LayerRegionIsland::SUPPORT_INTERFACE)) {
                    bbox_this.merge(get_extrusion_extents(region_island_ptr.extrusion(LayerRegionIsland::SUPPORT_INTERFACE)));
                }
            }
        }
        for (const PrintInstance &instance : print_object.instances()) {
            BoundingBoxf bbox_translated(bbox_this);
            bbox_translated.translate(unscale_p(instance.shift));
            bbox.merge(bbox_translated);
        }
    }
    return bbox;
}

}
