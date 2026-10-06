///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#include "SupportFamilyGrid.hpp"
#include "GridSupportGenerator.hpp"

#include <map>
#include <stdexcept>
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_support.h"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/PrintObject.hpp"

/* Support slabs sharing a Z become one auxiliary layer. Its raw slice is their
   union in object region zero, without modifier splitting. Rebuilding slices
   creates islands before region membership is locked. Each island then gets
   one work region per configured extruder, with dense top/bottom or sparse
   internal support surfaces. Extruder zero means the current extruder, stored
   as EXTRUDER_ID_UNKNOWN. Temporary API collections use shared plugin storage
   but are released individually; a run never clears that shared storage. */

namespace slic3r_api::Support::Grid {
namespace {
// Publish sorted slabs as auxiliary layers, with one region-island per extruder and clipped fill surfaces.
void publish(const plugin_run_context *context, const object_handle *object, Slic3r::SupportGrid::GeneratedSupport &support, orchestrator_handle *orchestrator);
}

// Construct the provider once; the orchestrator retains its registration but does not own the static instance.
SupportFamilyGrid &SupportFamilyGrid::instance(orchestrator_handle *orchestrator)
{
    static SupportFamilyGrid provider(orchestrator);
    return provider;
}

// Copy direct geometry and transitive flow/extruder-selection keys into a caller-sized buffer, or return its required count for null keys; the static list remains owned here.
int32_t SupportFamilyGrid::used_config_keys(raw_used_config_key *keys) const noexcept
{
    static const raw_used_config_key used[] = {
        {"support_material_style", RAW_CO_ENUM, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"support_material_enforce_layers", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"support_material_threshold", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"support_material_synchronize_layers", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"support_material_contact_distance_type", RAW_CO_ENUM, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"support_material_contact_distance", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"support_material_bottom_contact_distance", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"support_material_xy_spacing", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"support_material_layer_height", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"support_material_interface_layer_height", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"support_material_interface_layers", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"support_material_bottom_interface_layers", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"support_material_interface_spacing", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"support_material_spacing", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"support_material_angle", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"support_material_closing_radius", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"support_material_bottom_interface_expansion", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"support_material_extruder", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"support_material_interface_extruder", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"support_material_extrusion_width", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"first_layer_height", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"layer_height", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"nozzle_diameter", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"min_layer_height", RAW_CO_VECTOR_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"max_layer_height", RAW_CO_VECTOR_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"z_step", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"resolution_internal", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"extrusion_width", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"extrusion_spacing", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"first_layer_extrusion_width", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"first_layer_extrusion_spacing", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"first_layer_infill_extrusion_width", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"first_layer_infill_extrusion_spacing", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"external_perimeter_extrusion_width", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"external_perimeter_extrusion_spacing", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"external_perimeter_extrusion_change_odd_layers", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"external_perimeter_overlap", RAW_CO_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"perimeter_extrusion_width", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"perimeter_extrusion_spacing", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"perimeter_extruder", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"infill_extruder", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"solid_infill_extruder", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"perimeters", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"fill_density", RAW_CO_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"top_solid_layers", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"bottom_solid_layers", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"solid_infill_every_layers", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"infill_extrusion_width", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"infill_extrusion_spacing", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"infill_extrusion_change_odd_layers", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"bridge_type", RAW_CO_ENUM, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"bridge_flow_ratio", RAW_CO_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"filament_max_overlap", RAW_CO_VECTOR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"brim_width", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"brim_width_interior", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"raft_layers", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"raft_first_layer_expansion", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE}
    };
    if (keys) std::copy(std::begin(used), std::end(used), keys);
    return int32_t(std::size(used));
}

// Skip unsupported styles or empty demand, warn once for a requested raft, and publish geometric support; host cancellation and calculation errors are reported by PluginBase.
void SupportFamilyGrid::run_impl(const plugin_run_context *context) const
{
    throw_if_cancelled(context);
    const run_ctx_generate_support *payload = plugin_ctx_as_generate_support(context);
    if (!payload || !payload->object) return;
    const Slic3r::PrintObject &object = *reinterpret_cast<const Slic3r::PrintObject *>(payload->object);
    const Slic3r::SupportMaterialStyle style = object.config().support_material_style.value;
    if (style != Slic3r::smsGrid && style != Slic3r::smsSnug) {
        progress().finish_run();
        return;
    }
    bool has_demand = false;
    for (const Slic3r::Layer &layer : object.layers())
        for (const Slic3r::LayerSliceIsland &island : layer.islands())
            has_demand = has_demand || !island.support_demand().empty();
    if (!has_demand) {
        progress().finish_run();
        return;
    }
    if (object.config().raft_layers.value > 0 && context->report_warning)
        context->report_warning(context->host_context, "Grid support is generated without a raft; raft layers are not implemented by this family.");
    const Slic3r::SupportGrid::GridSupportParams params(object);
    Slic3r::SupportGrid::GeneratedSupport support;
    Slic3r::SupportGrid::generate_support(object, params, support);
    publish(context, payload->object, support, m_orchestrator);
    progress().finish_run();
}

namespace {
// Group non-empty slabs by integer Z and publish their union before locking islands; failed API creation or surface publication throws so the host retains failure visibility.
void publish(const plugin_run_context *context, const object_handle *object, Slic3r::SupportGrid::GeneratedSupport &support, orchestrator_handle *orchestrator)
{
    using namespace Slic3r;
    using namespace Slic3r::SupportGrid;
    GeneratorLayers sorted = support.top;
    append(sorted, support.bottom);
    append(sorted, support.base);
    append(sorted, support.interfaces);
    std::sort(sorted.begin(), sorted.end(), [](const GeneratorLayer *a, const GeneratorLayer *b) { return *a < *b; });
    const PrintObject &native_object = *reinterpret_cast<const PrintObject *>(object);
    const int base_tool = native_object.config().support_material_extruder.value;
    const int interface_tool = native_object.config().support_material_interface_extruder.value;
    const extruder_index_t base_extruder = base_tool > 0 ? base_tool - 1 : EXTRUDER_ID_UNKNOWN;
    const extruder_index_t interface_extruder = interface_tool > 0 ? interface_tool - 1 : EXTRUDER_ID_UNKNOWN;
    uint32_t interface_id = 0;
    std::map<coord_t, uint32_t> installed_ids;
    for (size_t i = 0; i < sorted.size();) {
        size_t end = i + 1;
        const coord_t z = sorted[i]->print_z;
        while (end < sorted.size() && sorted[end]->print_z == z) ++end;
        coord_t height = sorted[i]->height;
        Polygons all;
        size_t interface_count = 0, top_count = 0;
        coord_t contact_bottom = 0;
        for (size_t j = i; j < end; ++j) {
            const GeneratorLayer &slab = *sorted[j];
            height = std::min(height, slab.height);
            if (slab.polygons.empty()) continue;
            polygons_append(all, slab.polygons);
            if (slab.type != LayerType::Base && slab.type != LayerType::Intermediate) ++interface_count;
            if (slab.type == LayerType::TopContact) { ++top_count; contact_bottom = slab.bottom_z; }
        }
        if (!all.empty()) {
            layer_handle *layer = object_add_auxiliary_layer(object, height, z, z - height / 2);
            if (!layer) throw std::runtime_error("Cannot create grid support layer");
            uint32_t this_id = interface_id;
            const bool contacts_only = top_count > 0 && top_count == interface_count;
            if (contacts_only && installed_ids.count(contact_bottom)) this_id = installed_ids.at(contact_bottom) + 1;
            LayerSupportProperty *property = static_cast<LayerSupportProperty *>(plugin_property_get_or_add_data_mutable(
                orchestrator, layer_get_properties_mutable(layer), PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT));
            if (!property) throw std::runtime_error("Cannot mark grid support layer");
            property->interface_id = this_id;
            property->reserved = 0;
            installed_ids[z] = this_id;
            if (interface_count && !contacts_only) ++interface_id;
            const ExPolygons slices = union_ex(all);
            expolygons_copy(layer_region_borrow_mutable_slices(layer_get_region_mutable(layer, 0)),
                reinterpret_cast<const expolygon_collection_handle *>(&slices));
            layer_recompute_slices_and_islands_from_layer_regions(layer);
            layer_add_regions_to_islands(layer);
            for (uint32_t island_idx = 0; island_idx < layer_count_island(layer); ++island_idx) {
                layer_island_handle *island = layer_get_island_mutable(layer, island_idx);
                const Slic3r::ExPolygon &island_slice = *reinterpret_cast<const Slic3r::ExPolygon *>(layer_island_get_slice(island));
                std::map<extruder_index_t, Slic3r::SurfaceCollection> surfaces;
                for (size_t j = i; j < end; ++j) {
                    const GeneratorLayer &slab = *sorted[j];
                    const bool base = slab.type == LayerType::Base || slab.type == LayerType::Intermediate;
                    const bool top = slab.type == LayerType::TopContact || slab.type == LayerType::TopInterface;
                    SurfaceType type = stModSupport | (base ? stPosInternal | stDensSparse : top ? stPosTop | stDensSolid : stPosBottom | stDensSolid);
                    if (slab.type == LayerType::BottomContact && slab.bridging) type |= stModBridge;
                    surfaces[base ? base_extruder : interface_extruder].append(intersection_ex(slab.polygons, ExPolygons{island_slice}), type);
                }
                for (const auto &[extruder, collection] : surfaces) {
                    if (collection.empty()) continue;
                    layer_region_island_handle *region = layer_island_get_or_create_region_island(island, nullptr, 0, extruder);
                    StoredSurfaceCollection stored(context->plugin_storage);
                    for (const Slic3r::Surface &surface : collection)
                        stored.append(slic3r_api::ExPolygon(reinterpret_cast<const expolygon_handle *>(&surface.expolygon)), raw_surface_type(surface.surface_type));
                    if (!region || !layer_region_island_set_fill_surfaces(region, stored.mutable_handle()))
                        throw std::runtime_error("Cannot publish grid support surfaces");
                }
            }
        }
        i = end;
    }
}
} // namespace

// Register the static provider with the host, which manages activation and shared scratch storage.
void register_support_family_grid_plugin(orchestrator_handle *orchestrator)
{
    orchestrator_register_plugin(orchestrator, SupportFamilyGrid::instance(orchestrator).c_instance());
}
}
