///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

#ifndef slic3r_plugins_avoid_crossing_input_hpp_
#define slic3r_plugins_avoid_crossing_input_hpp_

/*
Crossing tests and travel routing need layer geometry that remains usable after
temporary API storage is released. This interface copies that geometry and the
settings needed to prepare it; it does not test or route a travel itself.

Select an existing layer with LayerIdentity, then call build_input. Input owns a
stable, immutable LayerInput containing the target slices, the filled areas
published by perimeter processing, and spacing averages in scaled coordinates.
Each RegionInput retains raw slices, reconstructed fill areas, top surfaces and
regional settings, including regions with no slices. For support targets, Input
also copies the owner's object-layer perimeter slices strictly below print Z +
SCALED_EPSILON for routing. platter_blockers separately unions the last normal
object slices below a supplied Z plus epsilon, translated by each real instance's
shift, for the current and earlier printing groups. Synthetic entries such as
wipe towers supply no blockers. These areas use platter coordinates.

An external boundary is a routing ring built from object holes placed at their
instance shifts in print coordinates, rather than from the target's local filled
areas. Call build_external_input only when such routing is needed. ExternalInput
owns holes and shifts separately in object order, plus the mean spacing at the
target Z. Support additionally appends holes from the last object layer strictly
below Z + SCALED_EPSILON; same-Z holes may therefore occur twice.

layer_at_print_z finds an exact object-layer Z; first_layer_below_print_z finds
the last object layer strictly below a supplied Z. Neither searches auxiliary
layers. is_first_layer reports exact equality with normal layer zero's print Z,
including auxiliary targets; it returns false when there are no normal layers.
is_off_object_layer compares the layer handle with the normal layer at its Z;
auxiliary layers, including the global auxiliary owner's layers, are off object.
Geometry and settings are copied; no host handle is retained. Keep Input
alive while consumers borrow its target. Workers own separate inputs and geometry.
Allocation storage may be shared; the host synchronizes its allocation bookkeeping.
LayerIdentity first selects an owner: global_object chooses Print::auxiliary_object,
otherwise object indexes the print objects. Then auxiliary selects that owner's
auxiliary layer collection rather than its normal layer collection, and layer
indexes that collection. The global owner holds platter-space skirt and brim
layers. External geometry still visits only the ordinary print objects.
An owner without normal layers cannot mark a target as first_layer.
*/

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "libslic3r/Plugins/Common/PolygonWorkBuffers.hpp"

struct storage_handle;
namespace slic3r_api {
class Print;
class Object;
class Layer;
class PrintingPlan;
namespace LayerExtrusionEdit { namespace AvoidCrossing {

// Select the owner first (global auxiliary owner or indexed print object), then its layer collection.
// auxiliary selects auxiliary layers, not the owner; object is ignored for the global owner and caches set it to zero.
struct LayerIdentity
{
    uint32_t object = 0;
    uint32_t layer = 0;
    bool auxiliary = false;
    bool global_object = false;
};

// Settings and geometry for every public region, including regions without slices.
struct RegionInput
{
    PluginCommon::polygon_work::ExPolygons raw_slices;
    PluginCommon::polygon_work::ExPolygons fill_expolygons;
    PluginCommon::polygon_work::ExPolygons top_surfaces;
    int perimeter_extruder = 0;
    int infill_extruder = 0;
    int solid_infill_extruder = 0;
    bool avoid_crossing_top = false;
    int perimeters = 0;
    bool ironing = false;
    bool avoid_travel_island = false;
    double avoid_travel_island_weight = 0.;
};

// Own exact ring sequences and mean perimeter spacing and external width, both in scaled units.
struct LayerInput
{
    LayerIdentity identity;
    coord_t print_z = 0;
    bool support = false;
    // True for object layer zero and every layer, including support, at its exact print Z.
    bool first_layer = false;
    PluginCommon::polygon_work::ExPolygons lslices;
    PluginCommon::polygon_work::ExPolygons perimeter_slices;
    std::vector<RegionInput> regions;
    coord_t default_perimeter_spacing = 0;
    coord_t perimeter_spacing = 0;
    float external_perimeter_width = 0.f;
};

// Holes retain source CW orientation; consumers reverse them before expand.
// Support appends the last object layer strictly below Z + SCALED_EPSILON, possibly duplicating same-Z holes.
struct ExternalObjectInput
{
    uint32_t object = 0;
    PluginCommon::polygon_work::Polygons holes;
    PluginCommon::work::Points instance_shifts;
};

// Own the target and local support routing slices.
struct Input
{
    std::unique_ptr<const LayerInput> target;
    PluginCommon::polygon_work::ExPolygons lower_perimeter_slices;
};

// Independently owned external boundary data, constructed only when requested by a consumer.
struct ExternalInput
{
    std::vector<ExternalObjectInput> external_objects;
    coord_t external_perimeter_spacing = 0;
};

// Find an exact scaled Z among object layers only; return no index when absent.
std::optional<uint32_t> layer_at_print_z(const Object &object, coord_t z);
// Find the last object layer strictly below the supplied scaled Z; auxiliary layers are ignored.
std::optional<uint32_t> first_layer_below_print_z(const Object &object, coord_t z);
// Copy geometry, settings and support routing slices.
Input build_input(storage_handle *storage, const Print &print, LayerIdentity target);
// Return whether the layer is absent from its owner's normal object layers.
bool is_off_object_layer(const Layer &layer);
// Union the last normal object slices below Z plus epsilon for real instances in groups through group_idx.
PluginCommon::polygon_work::ExPolygons platter_blockers(storage_handle *storage,
    const PrintingPlan &plan, uint32_t group_idx, coord_t print_z);
// Return whether this layer shares the first object layer's print Z, including auxiliary layers; false without object layers.
bool is_first_layer(const Object &object, const Layer &layer);
// Given an existing public layer identity, own ordered external holes/shifts and the global spacing at its Z.
ExternalInput build_external_input(const Print &print, LayerIdentity target);

}}}

#endif
