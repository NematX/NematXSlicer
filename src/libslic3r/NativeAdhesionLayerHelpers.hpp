///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_NativeAdhesionLayerHelpers_hpp_
#define slic3r_NativeAdhesionLayerHelpers_hpp_

#include <algorithm>
#include <type_traits>
#include <utility>

#include "ExtrusionEntity.hpp"
#include "Layer.hpp"
#include "PluginProperty.hpp"
#include "PrintObject.hpp"
#include "Api/plugin/cpp/DataTreeViews.hpp"

namespace Slic3r {

// Translate every geometric leaf of an owned adhesion tree without changing metadata.
inline void translate_adhesion_tree(ExtrusionEntity &entity, const Point &shift)
{
    if (ArcPolyline *polyline = entity.polyline_or_null()) {
        polyline->translate(shift);
        return;
    }
    for (const ExtrusionEntityUPtr &child : entity.children())
        if (child)
            translate_adhesion_tree(*child, shift);
}

/*
Native adhesion layer helpers
=============================

Brim and skirt are stored as ordinary auxiliary Layer objects. The layer
metadata says whether a layer is brim, normal skirt, or first-layer-only skirt;
the extrusion tree still lives in the perimeter bucket of each
LayerRegionIsland. Real-object layers are local, optionally restricted to one
instance; global auxiliary-object layers are already in plateau coordinates.

The helpers classify, traverse and place adhesion geometry:
- test an auxiliary layer with a small predicate;
- append owned copies of matched extrusion content to a destination tree;
- remove matched auxiliary layers;
- visit matched extrusion roots for post-processing;
- project the physical layer slice points used by first-layer hull rebuilding.

The predicates classify brim, normal skirt and first-layer-only skirt. Placement
is separate: append_matching_auxiliary_layer_points applies LayerInstanceProperty
through the shared projection helper, placing a marked layer only at its owner
and an unmarked common pattern at every instance.

For example, collect_auxiliary_layer_extrusions(object, layer_is_brim_adhesion,
cache) collects local compatibility geometry into cache, while
append_matching_auxiliary_layer_points(object, layer_is_any_adhesion, points)
adds physically placed points to a plate-space hull. Returned copies own their
trees; input layers are borrowed and are not retained by these helpers.
Copy destinations may be generic roots or compatibility collections. Leaf and
property-bearing source roots remain complete scopes; unmarked structural roots
contribute independent child copies in order. Destination identities and existing
prefix children stay in place. Mutations require exclusive destination access,
and allocation errors may leave an already appended prefix.
*/

inline bool layer_is_brim_adhesion(const Layer &layer)
{
    return LayerAdhesionProperty::layer_is_brim(layer);
}

inline bool layer_is_any_skirt_adhesion(const Layer &layer)
{
    return LayerAdhesionProperty::layer_has_kind(layer, RAW_LAYER_ADHESION_KIND_SKIRT);
}

inline bool layer_is_normal_skirt_adhesion(const Layer &layer)
{
    return LayerAdhesionProperty::layer_is_normal_skirt(layer);
}

inline bool layer_is_first_layer_skirt_adhesion(const Layer &layer)
{
    return LayerAdhesionProperty::layer_is_skirt_first_layer_only(layer);
}

inline bool layer_is_any_adhesion(const Layer &layer)
{
    return layer_is_brim_adhesion(layer) || layer_is_any_skirt_adhesion(layer);
}

// Copy perimeter content into the destination's children, preserving leaves and marked root scopes.
inline void append_auxiliary_layer_extrusions(const Layer &layer, ExtrusionEntity &dst);

/*
Collect the perimeter-bucket extrusion roots from every auxiliary layer matched
by matches_layer. The predicate can be a named helper or a lambda with the shape
`bool(const Layer&)`; it should only inspect the layer and must not keep
references to the borrowed layer after the call.
Copies are appended without changing source roots; allocation failures propagate.
*/
template<class MatchesLayer>
void collect_auxiliary_layer_extrusions(const PrintObject &object,
                                        MatchesLayer matches_layer,
                                        ExtrusionEntity &dst);

/*
Remove every auxiliary layer matched by matches_layer. The predicate can be a
named helper or a lambda with the shape `bool(const Layer&)`; keep it side
effect free because this helper mutates the PrintObject layer list during the
walk.
*/
template<class MatchesLayer>
void remove_auxiliary_layers(PrintObject &object, MatchesLayer matches_layer);

template<class MatchesLayer>
bool has_auxiliary_layer(const PrintObject &object, MatchesLayer matches_layer);

template<class Visitor>
void visit_auxiliary_layer_extrusions(Layer &layer, Visitor &visitor);

template<class MatchesLayer, class Visitor>
void visit_matching_auxiliary_layer_extrusions(PrintObject &object,
                                               MatchesLayer matches_layer,
                                               Visitor &visitor);

void append_auxiliary_layer_points(const Layer &layer, Points &dst);

// Append plateau-space slice points for selected auxiliary layers, projecting only their owner or every instance for a common pattern.
// Append owned points without changing borrowed layers; invalid-owner and allocation errors propagate.
template<class MatchesLayer>
void append_matching_auxiliary_layer_points(const PrintObject &object,
                                            MatchesLayer matches_layer,
                                            Points &dst);

namespace detail {

// Append ordinary virtual copies, preserving leaves and marked roots; otherwise copy children in order.
// The source is unchanged; allocation failures propagate after any earlier successful appends.
inline void append_adhesion_extrusion_copy(ExtrusionEntity &dst,
                                           const ExtrusionEntity &src)
{
    if (src.is_nop() && !src.has_properties())
        return;

    // Root properties apply to the complete subtree for both native and SDK output.
    if (src.is_leaf() || src.has_properties()) {
        dst.append_child(src);
        return;
    }

    for (const ExtrusionEntityUPtr &child : src.children())
        dst.append_child(*child);
}

template<class Visitor, class = void> struct HasTraverse : std::false_type {};

template<class Visitor>
struct HasTraverse<Visitor,
                   std::void_t<decltype(std::declval<Visitor &>().traverse(
                       std::declval<ExtrusionEntity &>()))>> : std::true_type {};

template<class Visitor>
void visit_extrusion_tree(ExtrusionEntity &entity, Visitor &visitor)
{
    /*
    The native codebase has two visitor families. New tree visitors own a
    traverse() method, while legacy visitors are driven through
    ExtrusionEntity::visit(). Support both so callers can use the traversal
    style they already have.
    */
    if constexpr (HasTraverse<Visitor>::value)
        visitor.traverse(entity);
    else
        entity.visit(visitor);
}

} // namespace detail

inline void append_auxiliary_layer_extrusions(const Layer &layer, ExtrusionEntity &dst)
{
    for (const LayerSliceIsland &island : layer.islands()) {
        for (const LayerRegionIsland &region_island : island.regions_islands()) {
            if (region_island.has_extrusion(LayerRegionIsland::PERIMETERS))
                detail::append_adhesion_extrusion_copy(dst, region_island.extrusion(LayerRegionIsland::PERIMETERS));
        }
    }
}

template<class MatchesLayer>
void collect_auxiliary_layer_extrusions(const PrintObject &object,
                                        MatchesLayer matches_layer,
                                        ExtrusionEntity &dst)
{
    for (const Layer &layer : object.auxiliary_layers())
        if (matches_layer(layer))
            append_auxiliary_layer_extrusions(layer, dst);
}

template<class MatchesLayer>
void remove_auxiliary_layers(PrintObject &object, MatchesLayer matches_layer)
{
    LayerUPtrs &layers = object.mutable_auxiliary_layers();
    layers.erase(std::remove_if(layers.begin(),
                                layers.end(),
                                [&matches_layer](const LayerUPtr &layer) {
                                    return layer != nullptr && matches_layer(*layer);
                                }),
                 layers.end());
}

template<class MatchesLayer>
bool has_auxiliary_layer(const PrintObject &object, MatchesLayer matches_layer)
{
    for (const Layer &layer : object.auxiliary_layers())
        if (matches_layer(layer))
            return true;
    return false;
}

template<class Visitor>
void visit_auxiliary_layer_extrusions(Layer &layer, Visitor &visitor)
{
    for (LayerSliceIsland &island : layer.islands()) {
        for (LayerRegionIsland &region_island : island.regions_islands()) {
            if (region_island.has_extrusion(LayerRegionIsland::PERIMETERS))
                detail::visit_extrusion_tree(region_island.mutable_extrusion(LayerRegionIsland::PERIMETERS), visitor);
        }
    }
}

template<class MatchesLayer, class Visitor>
void visit_matching_auxiliary_layer_extrusions(PrintObject &object,
                                               MatchesLayer matches_layer,
                                               Visitor &visitor)
{
    for (Layer &layer : object.auxiliary_layers())
        if (matches_layer(layer))
            visit_auxiliary_layer_extrusions(layer, visitor);
}

inline void append_auxiliary_layer_points(const Layer &layer, Points &dst)
{
    /*
    Auxiliary adhesion layer slices are built from extrusion coverage, not only
    centerlines. Their polygon points therefore describe the physical bed area
    that must be included in the first-layer convex hull.
    */
    for (const ExPolygon &slice : layer.lslices()) {
        dst.insert(dst.end(), slice.contour.points.begin(), slice.contour.points.end());
        for (const Polygon &hole : slice.holes)
            dst.insert(dst.end(), hole.points.begin(), hole.points.end());
    }
}

template<class MatchesLayer>
void append_matching_auxiliary_layer_points(const PrintObject &object,
                                            MatchesLayer matches_layer,
                                            Points &dst)
{
    for (const Layer &layer : object.auxiliary_layers()) {
        if (!matches_layer(layer))
            continue;
        Points local;
        append_auxiliary_layer_points(layer, local);
        const slic3r_api::Object view(reinterpret_cast<const object_handle *>(&object));
        const slic3r_api::Layer layer_view(reinterpret_cast<const layer_handle *>(&layer));
        for (const std::pair<uint32_t, c_point> &projection : slic3r_api::layer_instance_projections(view, layer_view))
            for (const Point &point : local)
                dst.push_back(point + Point(projection.second.x, projection.second.y));
    }
}

} // namespace Slic3r

#endif // slic3r_NativeAdhesionLayerHelpers_hpp_
