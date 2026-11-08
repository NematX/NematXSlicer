///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstring>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "libslic3r/Api/host/ApiHostUtils.hpp"
#include "libslic3r/Api/host/InfillBridgeFlow.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/internal/LayerAccess.hpp"
#include "libslic3r/Api/internal/LayerIslandAccess.hpp"
#include "libslic3r/Api/internal/LayerRegionAccess.hpp"
#include "libslic3r/Api/plugin/c/slic3r_data_tree.h"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Config/ConfigDef.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/ExtrusionRole.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/PrintRegion.hpp"
#include "libslic3r/PluginProperty.hpp"
#include "libslic3r/Surface.hpp"
#include "libslic3r/SurfaceCollection.hpp"
#include "libslic3r/Steps/StepRunner.hpp"

/*
Expose the FFF print's sliced data to plugins through the C data-tree API.
Opaque handles identify native objects; they do not transfer ownership or expose
the C++ layout. Print owns objects, objects own layers, and layers own their
regions, islands, surfaces and generated extrusion trees.

Query functions resolve a borrowed handle and return a value or another borrowed
view. Keep the owning node alive and reacquire descendants after rebuilding or
replacing a collection. Mutable access is limited by the current slicing step's
capabilities; receiving a mutable handle does not grant unrestricted live editing.

Some operations build or edit temporary surface collections owned by plugin
storage. Type and bridge-angle setters act on that scratch data, with the supplied
storage owner checked before mutation. Publication callbacks then copy or move
the prepared result into the live tree at the permitted processing boundary.
Bridge metadata and native-flow queries let plugins describe prepared surfaces
without guessing the host's surface flags or flow calculation.
Default configuration queries borrow Print-owned settings without creating the
auxiliary object. Concurrent readers require stable defaults, as other views do.
The region resolver delegates to PrintObject::default_region_config(), so host
callers and SDK views share one rule for objects without regions.

Scratch setters keep ownership unchanged, validate arguments and contain failures
before returning through C. Null, range and phase rules are documented with their functions
in slic3r_data_tree.h; callers must not infer success from a borrowed pointer's
existence alone. No lock here makes concurrent edits of one collection safe.
Protected edits resolve the owning Print and check the permitted editing step.
With SLIC3R_STRICT_FREEZE, an invalid edit is refused and requests a stop for
that Print's execution. Without that mode, the check is an assertion.

Representative direct calls (branches are not a timeline):

    object_get_layer()
        |-- Slic3r::to_object()
        |-- layer_count()
        `-- layer()

    layer_region_island_set_fill_surfaces()
        |-- Slic3r::to_layer_region_island()
        |-- island->regions()
        |-- region->layer()
        |-- layer->object()
        |-- object->auxiliary_layers()
        `-- Slic3r::data_tree_editable()
*/

namespace Slic3r {

static Surface *to_surface(surface_handle *me) { return reinterpret_cast<Surface*>(me); }
static const Surface *to_surface(const surface_handle *me) { return reinterpret_cast<const Surface*>(me); }
static SurfaceCollection *to_surface_collection(surface_collection_handle *me) { return reinterpret_cast<SurfaceCollection*>(me); }
static const SurfaceCollection *to_surface_collection(const surface_collection_handle *me) { return reinterpret_cast<const SurfaceCollection*>(me); }

static Layer *to_layer(layer_handle *me) { return reinterpret_cast<Layer*>(me); }
static const Layer *to_layer(const layer_handle *me) { return reinterpret_cast<const Layer*>(me); }

static LayerRegion *to_layer_region(layer_region_handle *me) { return reinterpret_cast<LayerRegion*>(me); }
static const LayerRegion *to_layer_region(const layer_region_handle *me) { return reinterpret_cast<const LayerRegion*>(me); }

static LayerSliceIsland *to_layer_island(layer_island_handle *me) { return reinterpret_cast<LayerSliceIsland*>(me); }
static const LayerSliceIsland *to_layer_island(const layer_island_handle *me) { return reinterpret_cast<const LayerSliceIsland*>(me); }

static LayerRegionIsland *to_layer_region_island(layer_region_island_handle *me) { return reinterpret_cast<LayerRegionIsland*>(me); }
static const LayerRegionIsland *to_layer_region_island(const layer_region_island_handle *me) { return reinterpret_cast<const LayerRegionIsland*>(me); }

static PrintRegion *to_print_region(print_region_handle *me) { return reinterpret_cast<PrintRegion*>(me); }
static const PrintRegion *to_print_region(const print_region_handle *me) { return reinterpret_cast<const PrintRegion*>(me); }

static PrintObject *to_object(object_handle *me) { return reinterpret_cast<PrintObject*>(me); }
static const PrintObject *to_object(const object_handle *me) { return reinterpret_cast<const PrintObject*>(me); }

static Print *to_print(print_handle *me) { return reinterpret_cast<Print*>(me); }
static const Print *to_print(const print_handle *me) { return reinterpret_cast<const Print*>(me); }

static PluginPropertyContainer *to_plugin_properties(plugin_property_container_handle *me)
{
    return reinterpret_cast<PluginPropertyContainer *>(me);
}

static const PluginPropertyContainer *to_plugin_properties(const plugin_property_container_handle *me)
{
    return reinterpret_cast<const PluginPropertyContainer *>(me);
}

static plugin_property_container_handle *to_mutable_property_handle(PluginPropertyContainer *container)
{
    return reinterpret_cast<plugin_property_container_handle *>(container);
}

static const plugin_property_container_handle *to_property_handle(const PluginPropertyContainer *container)
{
    return reinterpret_cast<const plugin_property_container_handle *>(container);
}

static ExPolygon *to_expolygon(expolygon_handle *me) { return reinterpret_cast<ExPolygon*>(me); }
static const ExPolygon *to_expolygon(const expolygon_handle *me) { return reinterpret_cast<const ExPolygon*>(me); }

static ExPolygons *to_expolygons(expolygon_collection_handle *me) { return reinterpret_cast<ExPolygons*>(me); }
static const ExPolygons *to_expolygons(const expolygon_collection_handle *me) { return reinterpret_cast<const ExPolygons*>(me); }

static bool check_freeze(const Print *print, slicing_step_t last_editor_step, const char *what)
{
    // STEP_NONE means idle-only access; it is not a boundary in execution_order().
    const bool allowed = print == nullptr ||
        (last_editor_step == STEP_NONE ? print->running_step() == STEP_NONE : print->running_step_may_edit(last_editor_step));
#if defined(SLIC3R_STRICT_FREEZE)
    if (!allowed) {
        Orchestrator &orchestrator = Orchestrator::instance();
        orchestrator.request_plugin_cancel_after_host_failure(print);
        if (!print->claim_freeze_diagnostic())
            return false;
        try {
            const std::string message = std::string("Frozen data-tree edit refused: ") + what +
                " (running step " + Steps::Detail::step_name_for_diagnostic(orchestrator, print->running_step()) +
                ", last editor " + Steps::Detail::step_name_for_diagnostic(orchestrator, last_editor_step) + ").";
            const int32_t id = orchestrator.store_error_message(message.c_str());
            // No plugin context reaches these getters; only the owning Print's step is attributable.
            orchestrator.add_plugin_message(Orchestrator::PluginMessageLevel::Error,
                                            nullptr, print->running_step(), message.c_str(), id);
        } catch (...) {
            // Cancellation survives diagnostic allocation failures at the C boundary.
        }
        return false;
    }
#else
    (void)what;
    assert(allowed);
#endif
    return true;
}

static bool data_tree_editable(const PrintObject &object, const char *what)
{
    return check_freeze(object.print(), STEP_FINALIZE_SLICING, what);
}

static bool island_outputs_editable(const LayerSliceIsland &island, const char *what)
{
    const Layer *layer = island.layer();
    const PrintObject *object = layer == nullptr ? nullptr : layer->object();
    return object == nullptr || data_tree_editable(*object, what);
}

static bool region_island_editable(const LayerRegionIsland &island, const char *what)
{
    // Region-islands borrow their Print ownership through their associated regions.
    for (const LayerRegion *region : island.regions()) {
        const PrintObject *object = region->layer() == nullptr ? nullptr : region->layer()->object();
        if (object != nullptr && !data_tree_editable(*object, what))
            return false;
    }
    return true;
}

// Auxiliary geometry is produced after PostSlicing and follows the later data-tree boundary.
static bool slices_editable(const Layer &layer, const char *what)
{
    const PrintObject *object = layer.object();
    if (object == nullptr || object->print() == nullptr)
        return true;
    for (const Layer &auxiliary_layer : object->auxiliary_layers())
        if (&auxiliary_layer == &layer)
            return data_tree_editable(*object, what);
    return check_freeze(object->print(), STEP_POST_SLICING, what);
}

static c_point to_c_point(const Point &point)
{
    c_point out = {};
    out.x = point.x();
    out.y = point.y();
    return out;
}

static c_bounding_box to_c_bounding_box(const BoundingBox &box)
{
    c_bounding_box out = {};
    out.min.x = box.min.x();
    out.min.y = box.min.y();
    out.max.x = box.max.x();
    out.max.y = box.max.y();
    return out;
}

static c_matrix4d to_c_matrix4d(const Transform3d &matrix)
{
    c_matrix4d out = {};
    const auto &raw = matrix.matrix();
    for (uint32_t row = 0; row < 4; ++row)
        for (uint32_t col = 0; col < 4; ++col)
            out.value[row * 4 + col] = raw(row, col);
    return out;
}

static c_flow to_c_flow(const Flow &flow)
{
    c_flow out = {};
    out.width = flow.scaled_width();
    out.spacing = flow.scaled_spacing();
    out.height = flow.scaled_height();
    out.nozzle_diameter = scale_i(flow.nozzle_diameter());
    out.is_bridge = flow.bridge() ? 1 : 0;
    out.spacing_ratio = flow.spacing_ratio();
    out.mm3_per_mm = flow.mm3_per_mm();
    return out;
}

// Use object support settings at this layer's height; bridge flow uses that height as diameter, and normal bed-contact flow uses first-layer settings.
// The bed test reads the bottom only: an auxiliary layer's id is its creation rank, not its position.
static Flow support_layer_flow(const Layer &layer, FlowRole role, bool bridge)
{
    const PrintObject *object = layer.object();
    const float height = float(layer.unscaled_height());
    if (!bridge && layer.scaled_bottom_z() <= 0)
        return support_material_1st_layer_flow(object, height);
    const Flow flow = role == frSupportMaterialInterface ? support_material_interface_flow(object, height) :
                                                         support_material_flow(object, height);
    return bridge ? Flow::bridging_flow(height, flow.nozzle_diameter()) : flow;
}

// Build an object support flow like the legacy support generator: the native
// base, interface or first-layer flow, then the optional bottom-contact
// conversion, then a bridge thread or a new printed height. See
// object_get_support_flow() in slic3r_data_tree.h for the meaning of each flag.
static Flow object_support_flow(const PrintObject &object, FlowRole role, double flow_height, double print_height,
                                uint32_t flags)
{
    const float height = float(std::max(0., flow_height));
    Flow flow = (flags & OBJECT_SUPPORT_FLOW_FIRST_LAYER) != 0 ? support_material_1st_layer_flow(&object, height) :
                role == frSupportMaterialInterface                ? support_material_interface_flow(&object, height) :
                                                                    support_material_flow(&object, height);
    if ((flags & OBJECT_SUPPORT_FLOW_BOTTOM_CONTACT) != 0) {
        // Same averaged ratio and soluble test as the legacy support parameters.
        double bridge_ratio = 0;
        for (size_t region_idx = 0; region_idx < object.num_printing_regions(); ++region_idx)
            bridge_ratio += object.printing_region(region_idx).config().bridge_flow_ratio.get_effective_value(1.);
        bridge_ratio /= object.num_printing_regions();
        const bool soluble = object.config().support_material_contact_distance_type.value == zdNone;
        flow = soluble ? flow.with_flow_ratio(bridge_ratio) :
                         Flow::bridging_flow(std::sqrt(bridge_ratio) * flow.nozzle_diameter(), flow.nozzle_diameter());
    }
    if ((flags & OBJECT_SUPPORT_FLOW_BRIDGE) != 0)
        return Flow::bridging_flow(float(print_height), flow.nozzle_diameter());
    if (print_height > 0 && !flow.bridge())
        flow = flow.with_height(float(print_height));
    return flow;
}

static FlowRole to_flow_role(raw_extrusion_role role)
{
    if ((role & RAW_EXTRUSION_ROLE_SUPPORT) != 0)
        return (role & RAW_EXTRUSION_ROLE_EXTERNAL) != 0 ? frSupportMaterialInterface : frSupportMaterial;
    if ((role & RAW_EXTRUSION_ROLE_PERIMETER) != 0)
        return (role & RAW_EXTRUSION_ROLE_EXTERNAL) != 0 ? frExternalPerimeter : frPerimeter;
    if ((role & RAW_EXTRUSION_ROLE_INFILL) != 0) {
        if ((role & RAW_EXTRUSION_ROLE_EXTERNAL) != 0 && (role & RAW_EXTRUSION_ROLE_SOLID) != 0)
            return frTopSolidInfill;
        if ((role & RAW_EXTRUSION_ROLE_SOLID) != 0)
            return frSolidInfill;
    }
    return frInfill;
}

// Compute a support flow from the object settings at any height; unusable input or settings give no flow rather than throwing through the C API.
static std::optional<Flow> object_support_flow_or_none(const object_handle *me, raw_extrusion_role flow_role,
                                                               double flow_height_mm, double print_height_mm, uint32_t flags)
{
    if (me == nullptr || (flow_role & RAW_EXTRUSION_ROLE_SUPPORT) == 0 ||
        ((flags & OBJECT_SUPPORT_FLOW_BRIDGE) != 0 && print_height_mm <= 0))
        return std::nullopt;
    try {
        const PrintObject &object = *to_object(me);
        // The bottom-contact ratio is an average over the regions, undefined without any.
        if (object.print() == nullptr ||
            ((flags & OBJECT_SUPPORT_FLOW_BOTTOM_CONTACT) != 0 && object.num_printing_regions() == 0))
            return std::nullopt;
        const Flow flow = object_support_flow(object, to_flow_role(flow_role), flow_height_mm, print_height_mm, flags);
        // mm3_per_mm() throws for a flow without a positive cross section (FlowErrorNegativeFlow): such
        // a flow is refused here, so the c_flow and the millimeter results agree.
        (void)flow.mm3_per_mm();
        return flow;    } catch (...) {
        return std::nullopt;
    }
}

// Compute a regional perimeter or infill flow at any height and layer id; unusable input or settings give no flow rather than throwing through the C API.
static std::optional<Flow> object_region_flow_or_none(const object_handle *me, uint32_t region_idx,
                                                              raw_extrusion_role flow_role, double height_mm, uint32_t layer_id)
{
    if (me == nullptr || height_mm <= 0 || (flow_role & RAW_EXTRUSION_ROLE_SUPPORT) != 0)
        return std::nullopt;
    try {
        const PrintObject &object = *to_object(me);
        if (object.print() == nullptr || region_idx >= object.num_printing_regions())
            return std::nullopt;
        const Flow flow = object.printing_region(region_idx).flow(object, to_flow_role(flow_role), height_mm, layer_id);
        // A flow without a positive cross section is refused, as for the support flows.
        (void)flow.mm3_per_mm();
        return flow;    } catch (...) {
        return std::nullopt;
    }
}

// Copy the float millimeter values of a host flow.
static c_flow_mm to_c_flow_mm(const Flow &flow)
{
    return c_flow_mm{flow.width(), flow.spacing(), flow.height(), flow.nozzle_diameter()};
}

static ExtrusionRole to_extrusion_role(raw_extrusion_role role)
{
    return ExtrusionRole(static_cast<ExtrusionRoleModifier>(role));
}

static const ExPolygon *first_or_null(const ExPolygons &expolygons)
{
    return expolygons.empty() ? nullptr : &expolygons.front();
}

static BoundingBox first_extents_or_empty(const ExPolygons &expolygons)
{
    return expolygons.empty() ? BoundingBox() : get_extents(expolygons.front());
}


static ConfigBase *mutable_config(const PrintObject &object)
{
    return const_cast<PrintObjectConfig*>(&object.config());
}

static ConfigBase *mutable_config(const Print &print)
{
    return const_cast<PrintConfig*>(&print.config());
}

} // namespace Slic3r

extern "C" {

uint32_t plugin_property_count(const plugin_property_container_handle *me)
{
    return me == nullptr ? 0u : static_cast<uint32_t>(Slic3r::to_plugin_properties(me)->property_count());
}

plugin_property_type plugin_property_type_at(const plugin_property_container_handle *me, uint32_t idx)
{
    return me == nullptr ? PLUGIN_PROPERTY_TYPE_INVALID : Slic3r::to_plugin_properties(me)->property_type_at(idx);
}

int32_t plugin_property_has(const plugin_property_container_handle *me, plugin_property_type type)
{
    return me != nullptr && Slic3r::to_plugin_properties(me)->has_property(type);
}

uint32_t plugin_property_data_size(const plugin_property_container_handle *me, plugin_property_type type)
{
    return me == nullptr ? 0u : Slic3r::to_plugin_properties(me)->property_data_size(type);
}

const void *plugin_property_data(const plugin_property_container_handle *me, plugin_property_type type)
{
    return me == nullptr ? nullptr : Slic3r::to_plugin_properties(me)->property_data(type);
}

void *plugin_property_data_mutable(plugin_property_container_handle *me, plugin_property_type type)
{
    return me == nullptr ? nullptr : Slic3r::to_plugin_properties(me)->property_data_mutable(type);
}

void *plugin_property_get_or_add_data_mutable(orchestrator_handle *orch,
                                              plugin_property_container_handle *me,
                                              plugin_property_type type)
{
    const uint32_t byte_count = orchestrator_property_byte_count(orch, type);
    const uint32_t alignment = orchestrator_property_alignment(orch, type);
    if (byte_count == 0 || alignment == 0)
        return nullptr;
    return me == nullptr ?
               nullptr :
               Slic3r::to_plugin_properties(me)->get_or_add_property_data_mutable(type, byte_count, alignment);
}

int32_t plugin_property_remove(plugin_property_container_handle *me, plugin_property_type type)
{
    return me != nullptr && Slic3r::to_plugin_properties(me)->remove_property(type);
}

void plugin_property_clear(plugin_property_container_handle *me)
{
    if (me != nullptr)
        Slic3r::to_plugin_properties(me)->clear_properties();
}

void plugin_property_copy_all(plugin_property_container_handle *dst, const plugin_property_container_handle *src)
{
    if (dst != nullptr && src != nullptr)
        Slic3r::to_plugin_properties(dst)->copy_properties_from(*Slic3r::to_plugin_properties(src));
}

c_surface surface_c_view(const surface_handle *me)
{
    if (me == nullptr)
        return {};
    const Slic3r::Surface &surface = *Slic3r::to_surface(me);
    return c_surface{
        reinterpret_cast<const expolygon_handle *>(&surface.expolygon),
        static_cast<raw_surface_type>(surface.surface_type),
        surface.id()
    };
}

const expolygon_handle *surface_get_expolygon(const surface_handle *me)
{
    return me == nullptr ? nullptr : reinterpret_cast<const expolygon_handle*>(&Slic3r::to_surface(me)->expolygon);
}

raw_surface_type surface_get_type(const surface_handle *me)
{
    return me == nullptr ? RAW_SURFACE_TYPE_NONE : static_cast<raw_surface_type>(Slic3r::to_surface(me)->surface_type);
}

uint64_t surface_get_id(const surface_handle *me)
{
    return me == nullptr ? 0u : Slic3r::to_surface(me)->id();
}

double surface_get_bridge_angle(const surface_handle *me)
{
    return me == nullptr ? -1. : Slic3r::to_surface(me)->bridge_angle;
}

int32_t surface_collection_set_bridge_angle(storage_handle *storage, surface_collection_handle *me,
                                           uint32_t idx, double angle)
{
    if (storage == nullptr || me == nullptr || !std::isfinite(angle) || angle < 0.)
        return 0;
    try {
        if (!reinterpret_cast<Slic3r::PluginStorage *>(storage)->contains(me) || idx >= surface_collection_size(me))
            return 0;
        Slic3r::to_surface_collection(me)->at(idx).bridge_angle = std::fmod(angle, PI);
        return 1;
    } catch (...) { return 0; }
}

int32_t surface_collection_set_type(storage_handle *storage, surface_collection_handle *me,
                                  uint32_t idx, raw_surface_type type)
{
    const raw_surface_type positions = RAW_SURFACE_TYPE_POS_TOP | RAW_SURFACE_TYPE_POS_BOTTOM |
        RAW_SURFACE_TYPE_POS_INTERNAL | RAW_SURFACE_TYPE_POS_PERIMETER;
    const raw_surface_type densities = RAW_SURFACE_TYPE_DENS_SOLID | RAW_SURFACE_TYPE_DENS_SPARSE | RAW_SURFACE_TYPE_DENS_VOID;
    const raw_surface_type modifiers = RAW_SURFACE_TYPE_MOD_BRIDGE | RAW_SURFACE_TYPE_MOD_OVERBRIDGE |
        RAW_SURFACE_TYPE_MOD_SUPPORT | RAW_SURFACE_TYPE_MOD_CONTACT;
    const raw_surface_type position = type & positions, density = type & densities;
    if (storage == nullptr || me == nullptr || position == 0 || (position & (position - 1)) != 0 ||
        density == 0 || (density & (density - 1)) != 0 || (type & ~(positions | densities | modifiers)) != 0)
        return 0;
    try {
        if (!reinterpret_cast<Slic3r::PluginStorage *>(storage)->contains(me) || idx >= surface_collection_size(me))
            return 0;
        Slic3r::to_surface_collection(me)->at(idx).surface_type = static_cast<Slic3r::SurfaceType>(type);
        return 1;
    } catch (...) { return 0; }
}

int32_t surface_metadata_compatible_for_bridge(const surface_handle *a, const surface_handle *b)
{
    if (a == nullptr || b == nullptr)
        return 0;
    const Slic3r::Surface &x = *Slic3r::to_surface(a), &y = *Slic3r::to_surface(b);
    return x.surface_type == y.surface_type && x.scaled_thickness() == y.scaled_thickness() &&
        x.thickness_layers == y.thickness_layers && x.extra_perimeters == y.extra_perimeters &&
        x.maxNbSolidLayersOnTop == y.maxNbSolidLayersOnTop && x.priority == y.priority &&
        (x.bridge_angle < 0. || y.bridge_angle < 0. || std::abs(std::remainder(x.bridge_angle-y.bridge_angle, PI)) <= 1e-9) &&
        x.properties_equal(y);
}

int32_t surface_get_flag(const surface_handle *me, raw_surface_type flag)
{
    return me != nullptr && (surface_get_type(me) & flag) != 0;
}

c_flow_mm layer_region_get_infill_bridge_flow_mm(const layer_region_handle *me, raw_extrusion_role role)
{
    if (me == nullptr)
        return {};
    try {
        const Slic3r::LayerRegion &region = *Slic3r::to_layer_region(me);
        return Slic3r::to_c_flow_mm(Slic3r::infill_bridge_flow(*region.layer(), region, Slic3r::to_flow_role(role)));
    } catch (...) { return {}; }
}

c_flow layer_region_get_infill_bridge_flow(const layer_region_handle *me, raw_extrusion_role role)
{
    if (me == nullptr)
        return {};
    try {
        const Slic3r::LayerRegion &region = *Slic3r::to_layer_region(me);
        return Slic3r::to_c_flow(Slic3r::infill_bridge_flow(*region.layer(), region, Slic3r::to_flow_role(role)));
    } catch (...) { return {}; }
}

extruder_index_t layer_region_get_role_extruder(const layer_region_handle *me, raw_extrusion_role role)
{
    if (me == nullptr)
        return EXTRUDER_ID_UNKNOWN;
    try {
        const Slic3r::LayerRegion &region = *Slic3r::to_layer_region(me);
        return extruder_index_t(region.region().extruder(Slic3r::to_flow_role(role), *region.layer()->object()) - 1);
    } catch (...) { return EXTRUDER_ID_UNKNOWN; }
}

const plugin_property_container_handle *surface_get_properties(const surface_handle *me)
{
    return me == nullptr ? nullptr : Slic3r::to_property_handle(Slic3r::to_surface(me));
}

plugin_property_container_handle *surface_collection_get_properties_mutable(surface_collection_handle *me, uint32_t idx)
{
    if (me == nullptr || idx >= Slic3r::to_surface_collection(me)->size())
        return nullptr;
    return Slic3r::to_mutable_property_handle(&Slic3r::to_surface_collection(me)->at(idx));
}

// Retain an empty surface collection in the locked registry; null storage returns null and insertion errors release the allocation.
surface_collection_handle *storage_new_surface_collection(storage_handle *me)
{
    Slic3r::PluginStorage *storage = reinterpret_cast<Slic3r::PluginStorage *>(me);
    if (storage == nullptr)
        return nullptr;
    surface_collection_handle *out =
        reinterpret_cast<surface_collection_handle *>(storage->store(std::make_unique<Slic3r::SurfaceCollection>()));
    return out;
}

void surface_collection_clear(surface_collection_handle *me)
{
    if (me != nullptr)
        Slic3r::to_surface_collection(me)->clear();
}

void surface_collection_append_expolygon_copy(surface_collection_handle *me,
                                              const expolygon_handle *area,
                                              raw_surface_type surface_type)
{
    if (me != nullptr && area != nullptr)
        Slic3r::to_surface_collection(me)->surfaces.emplace_back(
            static_cast<Slic3r::SurfaceType>(surface_type), *Slic3r::to_expolygon(area));
}

void surface_collection_append_expolygon_move(surface_collection_handle *me,
                                              expolygon_handle *area,
                                              raw_surface_type surface_type)
{
    if (me == nullptr || area == nullptr)
        return;

    Slic3r::to_surface_collection(me)->surfaces.emplace_back(
        static_cast<Slic3r::SurfaceType>(surface_type), std::move(*Slic3r::to_expolygon(area)));
    Slic3r::to_expolygon(area)->clear();
}

void surface_collection_append_expolygons_copy(surface_collection_handle *me,
                                               const expolygon_collection_handle *areas,
                                               raw_surface_type surface_type)
{
    if (me != nullptr && areas != nullptr)
        Slic3r::to_surface_collection(me)->append(*Slic3r::to_expolygons(areas),
                                                  static_cast<Slic3r::SurfaceType>(surface_type));
}

void surface_collection_append_expolygons_move(surface_collection_handle *me,
                                               expolygon_collection_handle *areas,
                                               raw_surface_type surface_type)
{
    if (me != nullptr && areas != nullptr)
        Slic3r::to_surface_collection(me)->append(std::move(*Slic3r::to_expolygons(areas)),
                                                  static_cast<Slic3r::SurfaceType>(surface_type));
}

uint32_t surface_collection_size(const surface_collection_handle *me)
{
    return me == nullptr ? 0u : uint32_t(Slic3r::to_surface_collection(me)->size());
}

const surface_handle *surface_collection_at(const surface_collection_handle *me, uint32_t idx)
{
    if (me == nullptr || idx >= Slic3r::to_surface_collection(me)->size())
        return nullptr;
    return reinterpret_cast<const surface_handle *>(&Slic3r::to_surface_collection(me)->at(idx));
}

surface_handle *surface_collection_at_mutable(surface_collection_handle *me, uint32_t idx)
{
    if (me == nullptr || idx >= Slic3r::to_surface_collection(me)->size())
        return nullptr;
    return reinterpret_cast<surface_handle *>(&Slic3r::to_surface_collection(me)->at(idx));
}

coord_t layer_get_height(const layer_handle *me)
{
    return me == nullptr ? 0 : Slic3r::to_layer(me)->scaled_height();
}

coord_t layer_get_print_z(const layer_handle *me)
{
    return me == nullptr ? 0 : Slic3r::to_layer(me)->scaled_print_z();
}

coord_t layer_get_slice_z(const layer_handle *me)
{
    return me == nullptr ? 0 : Slic3r::to_layer(me)->scaled_print_z() - Slic3r::to_layer(me)->scaled_height() / 2;
}

const object_handle *layer_get_object(const layer_handle *me)
{
    return me == nullptr ? nullptr : reinterpret_cast<const object_handle *>(Slic3r::to_layer(me)->object());
}

const expolygon_collection_handle *layer_get_slices(const layer_handle *me) {
    return me == nullptr ? nullptr : reinterpret_cast<const expolygon_collection_handle*>(&Slic3r::to_layer(me)->lslices());
}

uint32_t layer_count_curled_line(const layer_handle *me)
{
    return me == nullptr ? 0u : uint32_t(Slic3r::to_layer(me)->curled_lines.size());
}

c_curled_line layer_get_curled_line(const layer_handle *me, uint32_t idx)
{
    c_curled_line out = {};
    if (me == nullptr || idx >= Slic3r::to_layer(me)->curled_lines.size())
        return out;

    const Slic3r::CurledLine &line = Slic3r::to_layer(me)->curled_lines[idx];
    out.a = Slic3r::to_c_point(line.a);
    out.b = Slic3r::to_c_point(line.b);
    out.curled_height = line.curled_height;
    return out;
}

layer_handle *layer_get_upper_layer_mutable(layer_handle *me)
{
    return me == nullptr ? nullptr : reinterpret_cast<layer_handle*>(Slic3r::to_layer(me)->upper_layer);
}

const layer_handle *layer_get_upper_layer(const layer_handle *me)
{
    return me == nullptr ? nullptr : reinterpret_cast<const layer_handle*>(Slic3r::to_layer(me)->upper_layer);
}

layer_handle *layer_get_lower_layer_mutable(layer_handle *me)
{
    return me == nullptr ? nullptr : reinterpret_cast<layer_handle*>(Slic3r::to_layer(me)->lower_layer);
}

const layer_handle *layer_get_lower_layer(const layer_handle *me)
{
    return me == nullptr ? nullptr : reinterpret_cast<const layer_handle*>(Slic3r::to_layer(me)->lower_layer);
}

const plugin_property_container_handle *layer_get_properties(const layer_handle *me)
{
    return me == nullptr ? nullptr : Slic3r::to_property_handle(Slic3r::to_layer(me));
}

plugin_property_container_handle *layer_get_properties_mutable(layer_handle *me)
{
    if (me != nullptr && !Slic3r::slices_editable(*Slic3r::to_layer(me), "layer_get_properties_mutable"))
        return nullptr;
    return me == nullptr ? nullptr : Slic3r::to_mutable_property_handle(Slic3r::to_layer(me));
}

uint32_t layer_count_region(const layer_handle *me)
{
    return me == nullptr ? 0 : Slic3r::to_layer(me)->region_count();
}

layer_region_handle *layer_get_region_mutable(layer_handle *me, uint32_t idx)
{
    if (me == nullptr || idx >= Slic3r::to_layer(me)->region_count())
        return nullptr;
    return reinterpret_cast<layer_region_handle*>(&Slic3r::to_layer(me)->region(idx));
}

const layer_region_handle *layer_get_region(const layer_handle *me, uint32_t idx)
{
    if (me == nullptr || idx >= Slic3r::to_layer(me)->region_count())
        return nullptr;
    return reinterpret_cast<const layer_region_handle*>(&Slic3r::to_layer(me)->region(idx));
}

uint32_t layer_count_island(const layer_handle *me)
{
    return me == nullptr ? 0 : Slic3r::to_layer(me)->islands().size();
}

layer_island_handle *layer_get_island_mutable(layer_handle *me, uint32_t idx)
{
    if (me == nullptr || idx >= Slic3r::to_layer(me)->islands().size())
        return nullptr;
    return reinterpret_cast<layer_island_handle*>(&Slic3r::to_layer(me)->island(idx));
}

const layer_island_handle *layer_get_island(const layer_handle *me, uint32_t idx)
{
    if (me == nullptr || idx >= Slic3r::to_layer(me)->islands().size())
        return nullptr;
    return reinterpret_cast<const layer_island_handle*>(&Slic3r::to_layer(me)->island(idx));
}

const plugin_property_container_handle *layer_region_get_properties(const layer_region_handle *me)
{
    return me == nullptr ? nullptr : Slic3r::to_property_handle(Slic3r::to_layer_region(me));
}

plugin_property_container_handle *layer_region_get_properties_mutable(layer_region_handle *me)
{
    const Slic3r::Layer *layer = me == nullptr ? nullptr : Slic3r::to_layer_region(me)->layer();
    if (layer != nullptr && !Slic3r::slices_editable(*layer, "layer_region_get_properties_mutable"))
        return nullptr;
    return me == nullptr ? nullptr : Slic3r::to_mutable_property_handle(Slic3r::to_layer_region(me));
}

// Compute normal region or support flow at the layer height; null or unusable settings return zero rather than throwing through the C API.
c_flow layer_region_get_flow(const layer_region_handle *me, raw_extrusion_role flow_role)
{
    if (me == nullptr)
        return {};
    try {
        const Slic3r::LayerRegion &region = *Slic3r::to_layer_region(me);
        const Slic3r::Layer *layer = region.layer();
        if (layer == nullptr || layer->object() == nullptr || layer->object()->print() == nullptr)
            return {};
        const Slic3r::FlowRole role = Slic3r::to_flow_role(flow_role);
        return Slic3r::to_c_flow(role == Slic3r::frSupportMaterial || role == Slic3r::frSupportMaterialInterface ?
                                   Slic3r::support_layer_flow(*layer, role, false) : region.flow(role));
    } catch (...) {
        return {};
    }
}

// Compute bridge flow, using support-layer height for support contacts; null or unusable settings return zero rather than throwing through the C API.
c_flow layer_region_get_bridging_flow(const layer_region_handle *me, raw_extrusion_role flow_role)
{
    if (me == nullptr)
        return {};
    try {
        const Slic3r::LayerRegion &region = *Slic3r::to_layer_region(me);
        const Slic3r::Layer *layer = region.layer();
        if (layer == nullptr || layer->object() == nullptr || layer->object()->print() == nullptr)
            return {};
        const Slic3r::FlowRole role = Slic3r::to_flow_role(flow_role);
        return Slic3r::to_c_flow(role == Slic3r::frSupportMaterial || role == Slic3r::frSupportMaterialInterface ?
                                   Slic3r::support_layer_flow(*layer, role, true) : region.bridging_flow(role));
    } catch (...) {
        return {};
    }
}

const expolygon_collection_handle *layer_region_get_slices(const layer_region_handle *me)
{
    return me == nullptr ? nullptr : reinterpret_cast<const expolygon_collection_handle*>(&Slic3r::to_layer_region(me)->get_raw_slices());
}

c_bounding_box layer_region_get_bounding_box(const layer_region_handle *me)
{
    return me == nullptr ? c_bounding_box{} : Slic3r::to_c_bounding_box(Slic3r::get_extents(*Slic3r::to_layer_region(me)));
}

const layer_handle *layer_region_get_layer(const layer_region_handle *me)
{
    return me == nullptr ? nullptr : reinterpret_cast<const layer_handle*>(Slic3r::to_layer_region(me)->layer());
}

const print_region_handle *layer_region_get_print_region(const layer_region_handle *me)
{
    return me == nullptr ? nullptr : reinterpret_cast<const print_region_handle*>(&Slic3r::to_layer_region(me)->region());
}

expolygon_handle *layer_island_get_slice_mutable(layer_island_handle *me)
{
    const Slic3r::Layer *layer = me == nullptr ? nullptr : Slic3r::to_layer_island(me)->layer();
    if (layer != nullptr && !Slic3r::slices_editable(*layer, "layer_island_get_slice_mutable"))
        return nullptr;
    return me == nullptr ? nullptr :
                           reinterpret_cast<expolygon_handle *>(
                               &Slic3r::ApiInternal::LayerIslandAccess::slice_mutable(*Slic3r::to_layer_island(me)));
}

const expolygon_handle *layer_island_get_slice(const layer_island_handle *me)
{
    return me == nullptr ? nullptr : reinterpret_cast<const expolygon_handle*>(&Slic3r::to_layer_island(me)->get_slice());
}

c_bounding_box layer_island_get_bounding_box(const layer_island_handle *me)
{
    return me == nullptr ? c_bounding_box{} : Slic3r::to_c_bounding_box(Slic3r::to_layer_island(me)->get_bounding_box());
}

const expolygon_handle *layer_island_get_infill_slice(const layer_island_handle *me)
{
    if (me == nullptr)
        return nullptr;
    const Slic3r::ExPolygon *expoly = Slic3r::first_or_null(Slic3r::to_layer_island(me)->infill_areas());
    return reinterpret_cast<const expolygon_handle*>(expoly);
}

const expolygon_collection_handle *layer_island_get_infill_areas(const layer_island_handle *me)
{
    return me == nullptr ? nullptr :
                           reinterpret_cast<const expolygon_collection_handle *>(
                               &Slic3r::to_layer_island(me)->infill_areas());
}

c_bounding_box layer_island_get_infill_bounding_box(const layer_island_handle *me)
{
    return me == nullptr ? c_bounding_box{} : Slic3r::to_c_bounding_box(Slic3r::first_extents_or_empty(Slic3r::to_layer_island(me)->infill_areas()));
}

const expolygon_handle *layer_island_get_infill_no_overlap_slice(const layer_island_handle *me)
{
    if (me == nullptr)
        return nullptr;
    const Slic3r::ExPolygon *expoly = Slic3r::first_or_null(Slic3r::to_layer_island(me)->infill_free_areas());
    return reinterpret_cast<const expolygon_handle*>(expoly);
}

const expolygon_collection_handle *layer_island_get_infill_no_overlap_areas(const layer_island_handle *me)
{
    return me == nullptr ? nullptr :
                           reinterpret_cast<const expolygon_collection_handle *>(
                               &Slic3r::to_layer_island(me)->infill_free_areas());
}

const expolygon_collection_handle *layer_island_get_support_demand(const layer_island_handle *me)
{
    return me == nullptr ? nullptr :
                           reinterpret_cast<const expolygon_collection_handle *>(
                               &Slic3r::to_layer_island(me)->support_demand());
}

const plugin_property_container_handle *layer_island_get_properties(const layer_island_handle *me)
{
    return me == nullptr ? nullptr : Slic3r::to_property_handle(Slic3r::to_layer_island(me));
}

plugin_property_container_handle *layer_island_get_properties_mutable(layer_island_handle *me)
{
    const Slic3r::Layer *layer = me == nullptr ? nullptr : Slic3r::to_layer_island(me)->layer();
    if (layer != nullptr && !Slic3r::slices_editable(*layer, "layer_island_get_properties_mutable"))
        return nullptr;
    return me == nullptr ? nullptr : Slic3r::to_mutable_property_handle(Slic3r::to_layer_island(me));
}

uint32_t layer_island_count_region(const layer_island_handle *me)
{
    return me == nullptr ? 0 : Slic3r::to_layer_island(me)->regions().size();
}

layer_region_handle *layer_island_get_region_mutable(layer_island_handle *me, uint32_t idx)
{
    if (me == nullptr || idx >= Slic3r::to_layer_island(me)->regions().size())
        return nullptr;
    auto it = Slic3r::to_layer_island(me)->regions().begin();
    std::advance(it, static_cast<std::ptrdiff_t>(idx));
    return reinterpret_cast<layer_region_handle*>(const_cast<Slic3r::LayerRegion*>(*it));
}

const layer_region_handle *layer_island_get_region(const layer_island_handle *me, uint32_t idx)
{
    if (me == nullptr || idx >= Slic3r::to_layer_island(me)->regions().size())
        return nullptr;
    auto it = Slic3r::to_layer_island(me)->regions().begin();
    std::advance(it, static_cast<std::ptrdiff_t>(idx));
    return reinterpret_cast<const layer_region_handle*>(*it);
}

uint32_t layer_island_count_region_island(const layer_island_handle *me)
{
    return me == nullptr ? 0 : Slic3r::to_layer_island(me)->regions_islands().size();
}

layer_region_island_handle *layer_island_get_region_island_mutable(layer_island_handle *me, uint32_t idx)
{
    if (me == nullptr || idx >= Slic3r::to_layer_island(me)->regions_islands().size())
        return nullptr;
    if (!Slic3r::island_outputs_editable(*Slic3r::to_layer_island(me), "layer_island_get_region_island_mutable"))
        return nullptr;
    return reinterpret_cast<layer_region_island_handle*>(&Slic3r::to_layer_island(me)->regions_island(idx));
}

const layer_region_island_handle *layer_island_get_region_island(const layer_island_handle *me, uint32_t idx)
{
    if (me == nullptr || idx >= Slic3r::to_layer_island(me)->regions_islands().size())
        return nullptr;
    return reinterpret_cast<const layer_region_island_handle*>(&Slic3r::to_layer_island(me)->regions_island(idx));
}

layer_region_island_handle *layer_island_get_or_create_region_island(
    layer_island_handle *island_handle,
    const layer_region_handle *const *region_handles,
    uint32_t region_count,
    extruder_index_t extruder_id)
{
    Slic3r::LayerSliceIsland *island = Slic3r::to_layer_island(island_handle);
    if (island == nullptr)
        return nullptr;
    if (!Slic3r::island_outputs_editable(*island, "layer_island_get_or_create_region_island"))
        return nullptr;

    Slic3r::LayerRegionSetCPtrs regions;
    if (region_handles == nullptr || region_count == 0) {
        regions = island->regions();
    } else {
        const Slic3r::LayerRegionSetCPtrs &island_regions = island->regions();
        for (uint32_t idx = 0; idx < region_count; ++idx) {
            const Slic3r::LayerRegion *region = Slic3r::to_layer_region(region_handles[idx]);
            if (region == nullptr || island_regions.find(region) == island_regions.end())
                return nullptr;
            regions.insert(region);
        }
    }

    if (regions.empty())
        return nullptr;

    // Reject malformed negative identities instead of silently converting
    // them into the one public unknown-extruder sentinel.
    if (extruder_id < EXTRUDER_ID_UNKNOWN)
        return nullptr;

    Slic3r::LayerRegionIsland &region_island =
        island->get_or_add_region_island(regions, extruder_id);
    return reinterpret_cast<layer_region_island_handle *>(&region_island);
}

const layer_handle *layer_island_get_layer(const layer_island_handle *me)
{
    return me == nullptr ? nullptr : reinterpret_cast<const layer_handle*>(Slic3r::to_layer_island(me)->layer());
}

uint32_t layer_island_count_lower_island(const layer_island_handle *me)
{
    return me == nullptr ? 0 : static_cast<uint32_t>(Slic3r::to_layer_island(me)->overlaps_below.size());
}

const layer_island_handle *layer_island_get_lower_island(const layer_island_handle *me, uint32_t idx)
{
    if (me == nullptr || idx >= Slic3r::to_layer_island(me)->overlaps_below.size())
        return nullptr;
    return reinterpret_cast<const layer_island_handle *>(Slic3r::to_layer_island(me)->overlaps_below[idx].to);
}

uint32_t layer_island_count_upper_island(const layer_island_handle *me)
{
    return me == nullptr ? 0 : static_cast<uint32_t>(Slic3r::to_layer_island(me)->overlaps_above.size());
}

const layer_island_handle *layer_island_get_upper_island(const layer_island_handle *me, uint32_t idx)
{
    if (me == nullptr || idx >= Slic3r::to_layer_island(me)->overlaps_above.size())
        return nullptr;
    return reinterpret_cast<const layer_island_handle *>(Slic3r::to_layer_island(me)->overlaps_above[idx].to);
}

extruder_index_t layer_region_island_extruder_id(const layer_region_island_handle *me) {
    if (me == nullptr)
        return EXTRUDER_ID_UNKNOWN;
    return Slic3r::to_layer_region_island(me)->extruder_id();
}

int32_t layer_region_island_has_extrusions(const layer_region_island_handle *me)
{
    return me != nullptr && Slic3r::to_layer_region_island(me)->has_extrusions();
}

uint32_t layer_region_island_extrusion_count(const layer_region_island_handle *me)
{
    return me==nullptr ? 0u : uint32_t(Slic3r::to_layer_region_island(me)->extrusion_buckets().size());
}
raw_extrusion_role layer_region_island_extrusion_role_at(const layer_region_island_handle *me,uint32_t idx)
{
    if (me==nullptr) return 0;
    const std::map<Slic3r::ExtrusionRole,Slic3r::ExtrusionEntityCollection> &buckets=Slic3r::to_layer_region_island(me)->extrusion_buckets();
    if (idx>=buckets.size()) return 0;
    std::map<Slic3r::ExtrusionRole,Slic3r::ExtrusionEntityCollection>::const_iterator it=buckets.begin();
    std::advance(it,idx);
    return raw_extrusion_role(it->first());
}

int32_t layer_region_island_has_extrusion(const layer_region_island_handle *me, raw_extrusion_role role)
{
    return me != nullptr && Slic3r::to_layer_region_island(me)->has_extrusion(Slic3r::to_extrusion_role(role));
}

extrusion_entity_handle *layer_region_island_get_mutable_extrusion(layer_region_island_handle *me, raw_extrusion_role role)
{
    if (me == nullptr)
        return nullptr;
    if (!Slic3r::region_island_editable(*Slic3r::to_layer_region_island(me), "layer_region_island_get_mutable_extrusion"))
        return nullptr;
    Slic3r::ExtrusionEntityCollection &collection = Slic3r::to_layer_region_island(me)->mutable_extrusion(Slic3r::to_extrusion_role(role));
    return reinterpret_cast<extrusion_entity_handle*>(&collection);
}

const extrusion_entity_handle *layer_region_island_get_extrusion(const layer_region_island_handle *me, raw_extrusion_role role)
{
    if (me == nullptr || !Slic3r::to_layer_region_island(me)->has_extrusion(Slic3r::to_extrusion_role(role)))
        return nullptr;
    return reinterpret_cast<const extrusion_entity_handle*>(&Slic3r::to_layer_region_island(me)->extrusion(Slic3r::to_extrusion_role(role)));
}

const surface_collection_handle *layer_region_island_get_fill_surfaces(const layer_region_island_handle *me)
{
    return me == nullptr ?
               nullptr :
               reinterpret_cast<const surface_collection_handle *>(&Slic3r::to_layer_region_island(me)->fill_surfaces());
}

// Check that every associated region is auxiliary and editable, then move the temporary surfaces; object layers or null destinations return zero unchanged.
int32_t layer_region_island_set_fill_surfaces(layer_region_island_handle *region_island,
                                             surface_collection_handle *surfaces)
{
    Slic3r::LayerRegionIsland *island = Slic3r::to_layer_region_island(region_island);
    if (island == nullptr || island->regions().empty())
        return 0;
    for (const Slic3r::LayerRegion *region : island->regions()) {
        const Slic3r::Layer *layer = region->layer();
        const Slic3r::PrintObject *object = layer == nullptr ? nullptr : layer->object();
        if (object == nullptr)
            return 0;
        bool auxiliary = false;
        for (const Slic3r::Layer &candidate : object->auxiliary_layers())
            if (&candidate == layer) {
                auxiliary = true;
                break;
            }
        if (!auxiliary)
            return 0;
        if (!Slic3r::data_tree_editable(*object, "layer_region_island_set_fill_surfaces"))
            return 0;
    }
    return Slic3r::ApiInternal::LayerRegionAccess::set_fill_surfaces(island, Slic3r::to_surface_collection(surfaces));
}

uint32_t layer_region_island_count_fill_surface(const layer_region_island_handle *me)
{
    return surface_collection_size(layer_region_island_get_fill_surfaces(me));
}

const surface_handle *layer_region_island_get_fill_surface(const layer_region_island_handle *me, uint32_t idx)
{
    return surface_collection_at(layer_region_island_get_fill_surfaces(me), idx);
}

const plugin_property_container_handle *layer_region_island_get_properties(const layer_region_island_handle *me)
{
    return me == nullptr ? nullptr : Slic3r::to_property_handle(Slic3r::to_layer_region_island(me));
}

plugin_property_container_handle *layer_region_island_get_properties_mutable(layer_region_island_handle *me)
{
    if (me == nullptr)
        return nullptr;
    if (!Slic3r::region_island_editable(*Slic3r::to_layer_region_island(me), "layer_region_island_get_properties_mutable"))
        return nullptr;
    return Slic3r::to_mutable_property_handle(Slic3r::to_layer_region_island(me));
}

uint32_t layer_region_island_count_region(const layer_region_island_handle *me)
{
    return me == nullptr ? 0 : uint32_t(Slic3r::to_layer_region_island(me)->regions().size());
}

const layer_region_handle *layer_region_island_get_region(const layer_region_island_handle *me, uint32_t idx)
{
    if (me == nullptr || idx >= layer_region_island_count_region(me))
        return nullptr;

    Slic3r::LayerRegionSetCPtrs::const_iterator it = Slic3r::to_layer_region_island(me)->regions().begin();
    std::advance(it, idx);
    return reinterpret_cast<const layer_region_handle *>(*it);
}


const config_handle *print_region_get_config(const print_region_handle *me)
{
    return me == nullptr ? nullptr : Slic3r::ApiHost::to_config_handle(&Slic3r::to_print_region(me)->config());
}

config_handle *object_get_config_mutable(object_handle *me)
{
    if (me != nullptr && !Slic3r::check_freeze(Slic3r::to_object(me)->print(), STEP_NONE, "object_get_config_mutable"))
        return nullptr;
    return me == nullptr ? nullptr : Slic3r::ApiHost::to_config_handle(Slic3r::mutable_config(*Slic3r::to_object(me)));
}

const config_handle *object_get_config(const object_handle *me)
{
    return me == nullptr ? nullptr : Slic3r::ApiHost::to_config_handle(&Slic3r::to_object(me)->config());
}

const plugin_property_container_handle *object_get_properties(const object_handle *me)
{
    return me == nullptr ? nullptr : Slic3r::to_property_handle(Slic3r::to_object(me));
}

plugin_property_container_handle *object_get_properties_mutable(object_handle *me)
{
    if (me != nullptr && !Slic3r::data_tree_editable(*Slic3r::to_object(me), "object_get_properties_mutable"))
        return nullptr;
    return me == nullptr ? nullptr : Slic3r::to_mutable_property_handle(Slic3r::to_object(me));
}

uint32_t object_get_name(const object_handle *me, char *out, uint32_t capacity)
{
    if (me == nullptr)
        return 0;

    const Slic3r::ModelObject *model_object = Slic3r::to_object(me)->model_object();
    if (model_object == nullptr)
        return 0;
    const std::string &name = model_object->name;
    const uint32_t required = uint32_t(name.size());
    if (out != nullptr && capacity > 0) {
        const size_t count = std::min<size_t>(required, capacity - 1);
        std::memcpy(out, name.data(), count);
        out[count] = '\0';
    }
    return required;
}

coord_t object_get_max_z(const object_handle *me)
{
    if (me == nullptr)
        return coord_t(0);
    const Slic3r::PrintObject *object = Slic3r::to_object(me);
    return object->model_object() == nullptr ? object->height() : scale_i(object->model_object()->max_z());
}

c_matrix4d object_get_transform(const object_handle *me)
{
    return me == nullptr ? c_matrix4d{} : Slic3r::to_c_matrix4d(Slic3r::to_object(me)->trafo());
}

c_point object_get_center_offset(const object_handle *me)
{
    return me == nullptr ? c_point{} : Slic3r::to_c_point(Slic3r::to_object(me)->center_offset());
}

uint32_t object_count_instance(const object_handle *me)
{
    return me == nullptr ? 0u : uint32_t(Slic3r::to_object(me)->instances().size());
}

c_point object_get_instance_shift(const object_handle *me, uint32_t idx)
{
    if (me == nullptr || idx >= Slic3r::to_object(me)->instances().size())
        return c_point{};
    return Slic3r::to_c_point(Slic3r::to_object(me)->instances()[idx].shift);
}

uint32_t object_count_layer(const object_handle *me)
{
    return me == nullptr ? 0 : Slic3r::to_object(me)->layer_count();
}

layer_handle *object_get_layer_mutable(object_handle *me, uint32_t idx)
{
    if (me == nullptr || idx >= Slic3r::to_object(me)->layer_count())
        return nullptr;
    return reinterpret_cast<layer_handle*>(&Slic3r::to_object(me)->layer(static_cast<size_t>(idx)));
}

const layer_handle *object_get_layer(const object_handle *me, uint32_t idx)
{
    if (me == nullptr || idx >= Slic3r::to_object(me)->layer_count())
        return nullptr;
    return reinterpret_cast<const layer_handle*>(&Slic3r::to_object(me)->layer(static_cast<size_t>(idx)));
}

uint32_t object_count_auxiliary_layer(const object_handle *me)
{
    return me == nullptr ? 0 : static_cast<uint32_t>(Slic3r::to_object(me)->auxiliary_layers().size());
}

layer_handle *object_get_auxiliary_layer(const object_handle *me, uint32_t idx)
{
    if (me == nullptr || idx >= Slic3r::to_object(me)->auxiliary_layers().size())
        return nullptr;
    return reinterpret_cast<layer_handle *>(&const_cast<Slic3r::Layer &>(
        Slic3r::to_object(me)->auxiliary_layers()[static_cast<size_t>(idx)]));
}

layer_handle *object_add_auxiliary_layer(const object_handle *me,
                                         coord_t height,
                                         coord_t print_z,
                                         coord_t slice_z)
{
    if (me == nullptr || height <= 0 || print_z <= 0)
        return nullptr;
    if (!Slic3r::data_tree_editable(*Slic3r::to_object(me), "object_add_auxiliary_layer"))
        return nullptr;

    Slic3r::PrintObject *object = const_cast<Slic3r::PrintObject *>(Slic3r::to_object(me));
    Slic3r::LayerUPtrs &layers = object->mutable_auxiliary_layers();
    const size_t id = layers.size();
    Slic3r::LayerUPtrs::iterator pos = std::lower_bound(
        layers.begin(),
        layers.end(),
        print_z,
        [](const Slic3r::LayerUPtr &layer, coord_t value) {
            return layer->scaled_print_z() < value;
        });

    Slic3r::LayerUPtrs::iterator inserted =
        object->insert_auxiliary_layer(pos, id, height, print_z, unscaled(slice_z));
    Slic3r::ApiInternal::LayerAccess::init_regions_from_object(**inserted);
    return reinterpret_cast<layer_handle *>(inserted->get());
}

int32_t object_remove_auxiliary_layer(const object_handle *me, layer_handle *layer)
{
    if (me == nullptr || layer == nullptr)
        return 0;
    if (!Slic3r::data_tree_editable(*Slic3r::to_object(me), "object_remove_auxiliary_layer"))
        return 0;

    Slic3r::PrintObject *object = const_cast<Slic3r::PrintObject *>(Slic3r::to_object(me));
    Slic3r::LayerUPtrs &layers = object->mutable_auxiliary_layers();
    Slic3r::Layer *target = Slic3r::to_layer(layer);
    Slic3r::LayerUPtrs::iterator it = std::find_if(layers.begin(), layers.end(), [target](const Slic3r::LayerUPtr &candidate) {
        return candidate.get() == target;
    });
    if (it == layers.end())
        return 0;
    layers.erase(it);
    return 1;
}

expolygon_collection_handle *layer_borrow_mutable_slices(layer_handle *me)
{
    if (me != nullptr && !Slic3r::slices_editable(*Slic3r::to_layer(me), "layer_borrow_mutable_slices"))
        return nullptr;
    return me == nullptr ?
        nullptr :
        reinterpret_cast<expolygon_collection_handle *>(&Slic3r::ApiInternal::LayerAccess::slices_mutable(
            *reinterpret_cast<Slic3r::Layer *>(me)));
}

expolygon_collection_handle *layer_region_borrow_mutable_slices(const layer_region_handle *me)
{
    if (me != nullptr && !Slic3r::slices_editable(*Slic3r::to_layer_region(me)->layer(), "layer_region_borrow_mutable_slices"))
        return nullptr;
    return me == nullptr ?
        nullptr :
        reinterpret_cast<expolygon_collection_handle *>(&Slic3r::ApiInternal::LayerRegionAccess::slices_mutable(
            *const_cast<Slic3r::LayerRegion *>(reinterpret_cast<const Slic3r::LayerRegion *>(me))));
}

void layer_recompute_slices_from_islands(layer_handle *me)
{
    if (me != nullptr && !Slic3r::slices_editable(*Slic3r::to_layer(me), "layer_recompute_slices_from_islands"))
        return;
    if (me != nullptr)
        Slic3r::ApiInternal::LayerAccess::recompute_slices_from_islands(*reinterpret_cast<Slic3r::Layer *>(me));
}

void layer_recompute_slices_and_islands_from_layer_regions(layer_handle *me)
{
    if (me != nullptr && !Slic3r::slices_editable(*Slic3r::to_layer(me), "layer_recompute_slices_and_islands_from_layer_regions"))
        return;
    if (me != nullptr)
        Slic3r::ApiInternal::LayerAccess::recompute_slices_from_layer_regions(*reinterpret_cast<Slic3r::Layer *>(me));
}

void layer_add_regions_to_islands(layer_handle *me)
{
    if (me != nullptr && !Slic3r::slices_editable(*Slic3r::to_layer(me), "layer_add_regions_to_islands"))
        return;
    if (me != nullptr && !Slic3r::to_layer(me)->islands().empty())
        Slic3r::to_layer(me)->add_regions_to_islands();
}

uint32_t object_count_region(const object_handle *me)
{
    return me == nullptr ? 0 : Slic3r::to_object(me)->num_printing_regions();
}


const print_region_handle *object_get_print_region(const object_handle *me, uint32_t idx)
{
    if (me == nullptr || idx >= Slic3r::to_object(me)->num_printing_regions())
        return nullptr;
    return reinterpret_cast<const print_region_handle*>(&Slic3r::to_object(me)->printing_region(idx));
}

c_flow object_get_support_flow(const object_handle *me, raw_extrusion_role flow_role, double flow_height_mm,
                               double print_height_mm, uint32_t flags)
{
    // The conversion reads the flow again; no exception may cross the C ABI.
    try {
        const std::optional<Slic3r::Flow> flow = Slic3r::object_support_flow_or_none(me, flow_role, flow_height_mm, print_height_mm, flags);
        return flow ? Slic3r::to_c_flow(*flow) : c_flow{};
    } catch (...) {
        return {};
    }
}

c_flow object_get_region_flow(const object_handle *me, uint32_t region_idx, raw_extrusion_role flow_role,
                              double height_mm, uint32_t layer_id)
{
    // The conversion reads the flow again; no exception may cross the C ABI.
    try {
        const std::optional<Slic3r::Flow> flow = Slic3r::object_region_flow_or_none(me, region_idx, flow_role, height_mm, layer_id);
        return flow ? Slic3r::to_c_flow(*flow) : c_flow{};
    } catch (...) {
        return {};
    }
}

c_flow_mm object_get_support_flow_mm(const object_handle *me, raw_extrusion_role flow_role, double flow_height_mm,
                                     double print_height_mm, uint32_t flags)
{
    // The conversion reads the flow again; no exception may cross the C ABI.
    try {
        const std::optional<Slic3r::Flow> flow = Slic3r::object_support_flow_or_none(me, flow_role, flow_height_mm, print_height_mm, flags);
        return flow ? Slic3r::to_c_flow_mm(*flow) : c_flow_mm{};
    } catch (...) {
        return {};
    }
}

c_flow_mm object_get_region_flow_mm(const object_handle *me, uint32_t region_idx, raw_extrusion_role flow_role,
                                    double height_mm, uint32_t layer_id)
{
    // The conversion reads the flow again; no exception may cross the C ABI.
    try {
        const std::optional<Slic3r::Flow> flow = Slic3r::object_region_flow_or_none(me, region_idx, flow_role, height_mm, layer_id);
        return flow ? Slic3r::to_c_flow_mm(*flow) : c_flow_mm{};
    } catch (...) {
        return {};
    }
}

// Copy and validate before publishing; no C++ exception may cross the C ABI.
const print_region_handle *print_register_print_region_config(print_handle *me, const config_handle *config)
{
    if (me == nullptr || config == nullptr)
        return nullptr;
    try {
        return reinterpret_cast<const print_region_handle *>(
            Slic3r::to_print(me)->register_print_region_config(*Slic3r::ApiHost::to_config(config)));
    } catch (...) {
        return nullptr;
    }
}

config_handle *print_get_config_mutable(print_handle *me)
{
    if (me != nullptr && !Slic3r::check_freeze(Slic3r::to_print(me), STEP_NONE, "print_get_config_mutable"))
        return nullptr;
    return me == nullptr ? nullptr : Slic3r::ApiHost::to_config_handle(Slic3r::mutable_config(*Slic3r::to_print(me)));
}

const config_handle *print_get_config(const print_handle *me)
{
    return me == nullptr ? nullptr : Slic3r::ApiHost::to_config_handle(&Slic3r::to_print(me)->config());
}

const config_handle *print_get_default_region_config(const print_handle *me)
{
    return me == nullptr ? nullptr : Slic3r::ApiHost::to_config_handle(&Slic3r::to_print(me)->default_region_config());
}

const config_handle *print_resolve_default_region_config(const print_handle *me, const object_handle *object)
{
    if (me == nullptr)
        return nullptr;
    const Slic3r::PrintRegionConfig &from_print = Slic3r::to_print(me)->default_region_config();
    return Slic3r::ApiHost::to_config_handle(
        object == nullptr ? &from_print : &Slic3r::to_object(object)->default_region_config(from_print));
}

const config_handle *print_get_default_object_config(const print_handle *me)
{
    return me == nullptr ? nullptr : Slic3r::ApiHost::to_config_handle(&Slic3r::to_print(me)->default_object_config());
}

const_strings_t print_records_channels(const print_handle *me)
{
    const_strings_t out = {};
    if (me == nullptr)
        return out;

    try {
        /*
        The C array borrows thread-local strings so it remains consumable after
        this function returns without transferring allocations to the plugin.
        A later call on the same thread replaces the returned view.
        */
        static thread_local std::vector<std::string> channel_storage;
        static thread_local std::vector<const char *> channel_ptrs;
        channel_storage = Slic3r::to_print(me)->records().channels();
        channel_ptrs.clear();
        channel_ptrs.reserve(channel_storage.size());
        for (const std::string &channel : channel_storage)
            channel_ptrs.push_back(channel.c_str());

        if (channel_ptrs.size() > std::numeric_limits<uint32_t>::max())
            return out;
        out.items = channel_ptrs.empty() ? nullptr : channel_ptrs.data();
        out.size = static_cast<uint32_t>(channel_ptrs.size());
    } catch (...) {
        return {};
    }
    return out;
}

const config_handle *print_records_get(const print_handle *me, const char *channel)
{
    if (me == nullptr || channel == nullptr)
        return nullptr;
    try {
        const Slic3r::DynamicConfig *config = Slic3r::to_print(me)->records().find(channel);
        return Slic3r::ApiHost::to_config_handle(config);
    } catch (...) {
        return nullptr;
    }
}

config_handle *print_records_get_or_add(const print_handle *me, const char *channel)
{
    if (me == nullptr || channel == nullptr)
        return nullptr;
    if (!Slic3r::check_freeze(Slic3r::to_print(me), STEP_FINALIZE_SLICING, "print_records_get_or_add"))
        return nullptr;
    try {
        Slic3r::Print *print = const_cast<Slic3r::Print *>(Slic3r::to_print(me));
        return Slic3r::ApiHost::to_config_handle(&print->records().get_or_add(channel));
    } catch (...) {
        return nullptr;
    }
}

int32_t print_records_remove(const print_handle *me, const char *channel)
{
    if (me == nullptr || channel == nullptr)
        return 0;
    if (!Slic3r::check_freeze(Slic3r::to_print(me), STEP_FINALIZE_SLICING, "print_records_remove"))
        return 0;
    try {
        Slic3r::Print *print = const_cast<Slic3r::Print *>(Slic3r::to_print(me));
        return print->records().remove(channel) ? 1 : 0;
    } catch (...) {
        return 0;
    }
}

print_record_id print_records_allocate_id(const print_handle *me)
{
    if (me == nullptr)
        return PRINT_RECORD_ID_INVALID;
    Slic3r::Print *print = const_cast<Slic3r::Print *>(Slic3r::to_print(me));
    return print->records().allocate_id();
}

uint32_t print_count_object(const print_handle *me)
{
    return me == nullptr ? 0 : Slic3r::to_print(me)->objects().size();
}

object_handle *print_get_object_mutable(print_handle *me, uint32_t idx)
{
    if (me == nullptr || idx >= Slic3r::to_print(me)->objects().size())
        return nullptr;
    return reinterpret_cast<object_handle*>(&Slic3r::to_print(me)->object(idx));
}

const object_handle *print_get_object(const print_handle *me, uint32_t idx)
{
    if (me == nullptr || idx >= Slic3r::to_print(me)->objects().size())
        return nullptr;
    return reinterpret_cast<const object_handle*>(&Slic3r::to_print(me)->object(idx));
}

object_handle *print_get_auxiliary_object(const print_handle *me)
{
    if (me == nullptr)
        return nullptr;
    return reinterpret_cast<object_handle *>(
        &const_cast<Slic3r::Print *>(Slic3r::to_print(me))->mutable_auxiliary_object());
}

} // extern "C"
