///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_plugin_cpp_properties_DataTreeProperties_hpp_
#define slic3r_Api_plugin_cpp_properties_DataTreeProperties_hpp_

/*
Built-in data-tree properties
=============================

This header gives the fixed C payloads stored on data-tree objects their typed
C++ keys and small interpretation helpers. The types add no data to their C ABI
payload, so pointers returned by PluginProperties may be viewed directly as the
matching C++ type.

The layer helpers are templates because they only require a view exposing
properties(). Their definitions therefore remain independent of DataTreeViews
and are instantiated only after the caller's concrete layer view is complete.

LayerInstanceProperty marks object-local auxiliary geometry for one physical
instance; without it, the geometry is a common pattern for every instance.
The global auxiliary object already uses plate coordinates and does not use
this marker. Producers initialize reserved to zero before publishing a layer:

    LayerInstanceProperty &owner = layer.mutable_properties().get_or_add(LayerInstanceProperty::key);
    owner.object_instance_idx = instance_index;
    owner.reserved = 0;

Consumers call layer_instance_owner() for the optional index, or
layer_instance_projections(object, layer) for a caller-owned vector of indices
and scaled shifts. The projection helper rejects an out-of-range owner rather
than silently dropping geometry. Neither helper changes a layer or its owner.

Developer guide: [Using Plugin Properties](/doc/plugins/properties.md)
*/

#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_data_tree.h"
#include "libslic3r/Api/plugin/cpp/PluginPropertyKey.hpp"

namespace slic3r_api {

/* Object-local auxiliary geometry owned by one physical instance. */
struct LayerInstanceProperty :
    BuiltInPluginPropertyPayload<LayerInstanceProperty,
                                 c_layer_instance_property,
                                 PLUGIN_PROPERTY_TYPE_LAYER_INSTANCE>
{
};

// Return the stored owner index, or no owner for an auxiliary pattern shared by all instances.
template<class LayerView>
std::optional<uint32_t> layer_instance_owner(const LayerView &layer)
{
    const LayerInstanceProperty *owner = layer.properties().get(LayerInstanceProperty::key);
    return owner == nullptr ? std::nullopt : std::optional<uint32_t>(owner->object_instance_idx);
}

// Return a caller-owned vector of (index, scaled shift): the marked owner or every instance for an unmarked local pattern; reject an invalid owner index.
template<class ObjectView, class LayerView>
std::vector<std::pair<uint32_t, c_point>> layer_instance_projections(const ObjectView &object, const LayerView &layer)
{
    std::vector<std::pair<uint32_t, c_point>> result;
    const std::optional<uint32_t> owner = layer_instance_owner(layer);
    if (owner) {
        if (*owner >= object.instance_count())
            throw std::out_of_range("Auxiliary layer owner instance index is outside the Object.");
        result.emplace_back(*owner, object.instance_shift(*owner));
    } else {
        for (uint32_t index = 0; index < object.instance_count(); ++index)
            result.emplace_back(index, object.instance_shift(index));
    }
    return result;
}

/* Identifies an auxiliary layer containing generated support geometry. */
struct LayerSupportProperty :
    BuiltInPluginPropertyPayload<LayerSupportProperty,
                                 c_layer_support_property,
                                 PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT>
{
};

/* Legacy marker identifying an auxiliary layer containing brim geometry. */
struct LayerBrimProperty :
    BuiltInPluginPropertyPayload<LayerBrimProperty,
                                 c_layer_brim_property,
                                 PLUGIN_PROPERTY_TYPE_LAYER_BRIM>
{
};

/* Describes the kind and lifetime of generated skirt or brim geometry. */
struct LayerAdhesionProperty :
    BuiltInPluginPropertyPayload<LayerAdhesionProperty,
                                 c_layer_adhesion_property,
                                 PLUGIN_PROPERTY_TYPE_LAYER_ADHESION>
{
    bool has_kind(raw_layer_adhesion_kind expected_kind) const { return kind == expected_kind; }
    bool has_flag(raw_layer_adhesion_flag flag) const { return (flags & flag) != 0; }
    bool is_brim() const { return has_kind(RAW_LAYER_ADHESION_KIND_BRIM); }
    bool is_skirt() const { return has_kind(RAW_LAYER_ADHESION_KIND_SKIRT); }
    bool is_first_layer_only() const { return has_flag(RAW_LAYER_ADHESION_FLAG_FIRST_LAYER_ONLY); }
};

/* Return the adhesion payload physically stored on a layer, if present. */
template<class LayerView>
const LayerAdhesionProperty *layer_adhesion_property(const LayerView &layer)
{
    return layer.properties().get(LayerAdhesionProperty::key);
}

/*
Test the effective adhesion kind carried by a layer.

The legacy brim marker remains a valid fallback so older producers and newer
consumers can share the same auxiliary-layer representation.
*/
template<class LayerView>
bool layer_has_adhesion_kind(const LayerView &layer, raw_layer_adhesion_kind kind)
{
    const LayerAdhesionProperty *adhesion = layer_adhesion_property(layer);
    if (adhesion != nullptr)
        return adhesion->has_kind(kind);
    return kind == RAW_LAYER_ADHESION_KIND_BRIM &&
           layer.properties().get(LayerBrimProperty::key) != nullptr;
}

template<class LayerView>
bool layer_is_brim(const LayerView &layer)
{
    return layer_has_adhesion_kind(layer, RAW_LAYER_ADHESION_KIND_BRIM);
}

template<class LayerView>
bool layer_is_normal_skirt(const LayerView &layer)
{
    const LayerAdhesionProperty *adhesion = layer_adhesion_property(layer);
    return adhesion != nullptr && adhesion->is_skirt() && !adhesion->is_first_layer_only();
}

template<class LayerView>
bool layer_is_skirt_first_layer_only(const LayerView &layer)
{
    const LayerAdhesionProperty *adhesion = layer_adhesion_property(layer);
    return adhesion != nullptr && adhesion->is_skirt() && adhesion->is_first_layer_only();
}

} // namespace slic3r_api

#endif // slic3r_Api_plugin_cpp_properties_DataTreeProperties_hpp_
