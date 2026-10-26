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

/* Support slabs sharing both Z and printable thickness become one auxiliary
   layer. Its raw slice is their union in object region zero, without modifier
   splitting. Rebuilding slices
   creates islands before region membership is locked. Each island then gets
   one work region per configured extruder, with dense top/bottom or sparse
   internal support surfaces. Extruder zero means the current extruder, stored
   as EXTRUDER_ID_UNKNOWN. Temporary API collections use shared plugin storage
   but are released individually; a run never clears that shared storage.
   Groups of one print Z share the native interface id of that Z, and they are
   published thinnest first so the host's ordered insertion keeps the thickest
   layer first at that Z. */

namespace slic3r_api::Support::Grid {
namespace {
// Publish slabs grouped by top Z and thickness with one region-island per extruder and clipped fill surfaces; one interface id per print Z and the thinnest group published first keep the native numbering and order.
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
// Group non-empty slabs by integer Z and thickness and publish their union before locking islands; equal-Z insertion precedes existing layers with unique creation ids, and rejected API writes throw.
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
    const bool top_enabled = native_object.config().support_material_interface_layers.value != 0;
    const bool bottom_enabled = native_object.config().support_material_bottom_interface_layers.is_enabled() ?
        top_enabled && native_object.config().support_material_bottom_interface_layers.value != 0 : top_enabled;
    const extruder_index_t base_extruder = base_tool > 0 ? base_tool - 1 : EXTRUDER_ID_UNKNOWN;
    const extruder_index_t interface_extruder = interface_tool > 0 ? interface_tool - 1 : EXTRUDER_ID_UNKNOWN;
    // The native generator only moves a contact into the base when both support roles share one
    // extruder, so the family types the area the same way.
    const bool merge_regions = base_tool == interface_tool;
    uint32_t interface_id = 0;
    std::map<coord_t, uint32_t> installed_ids;
    for (size_t i = 0; i < sorted.size();) {
        // One print Z carries several (Z, thickness) groups. The native generator counts the
        // interfaces and advances its interface id once per print Z, over all groups of that Z.
        const coord_t z = sorted[i]->print_z;
        std::vector<std::pair<size_t, size_t>> groups;
        size_t interface_count = 0, top_count = 0, z_end = i;
        coord_t contact_bottom = 0;
        bool z_has_base = false;
        coord_t z_min_height = 0;
        for (size_t end = i; end < sorted.size() && sorted[end]->print_z == z;) {
            const coord_t height = sorted[end]->height;
            size_t group_end = end + 1;
            while (group_end < sorted.size() && sorted[group_end]->print_z == z && sorted[group_end]->height == height) ++group_end;
            for (size_t j = end; j < group_end; ++j) {
                const GeneratorLayer &slab = *sorted[j];
                if (slab.polygons.empty()) continue;
                if (slab.type == LayerType::Base || slab.type == LayerType::Intermediate) z_has_base = true;
                else ++interface_count;
                z_min_height = z_min_height == 0 ? slab.height : std::min(z_min_height, slab.height);
                if (slab.type == LayerType::TopContact) { ++top_count; contact_bottom = slab.bottom_z; }
            }
            groups.emplace_back(end, group_end);
            z_end = group_end;
            end = group_end;
        }
        const bool contacts_only = top_count > 0 && top_count == interface_count;
        const uint32_t this_id = contacts_only && installed_ids.count(contact_bottom) ? installed_ids.at(contact_bottom) + 1 : interface_id;
        if (interface_count && !contacts_only) ++interface_id;
        // The host inserts an equal-Z layer before the existing ones, so publishing the thinnest
        // group first leaves the native order at this print Z: the thickest layer first.
        for (std::vector<std::pair<size_t, size_t>>::const_reverse_iterator group = groups.rbegin(); group != groups.rend(); ++group) {
            const size_t begin = group->first, end = group->second;
            Polygons all;
            for (size_t j = begin; j < end; ++j) polygons_append(all, sorted[j]->polygons);
            if (all.empty()) continue;
            const coord_t height = sorted[begin]->height;
            layer_handle *layer = object_add_auxiliary_layer(object, height, z, z - height / 2);
            if (!layer) throw std::runtime_error("Cannot create grid support layer");
            LayerSupportProperty *property = static_cast<LayerSupportProperty *>(plugin_property_get_or_add_data_mutable(
                orchestrator, layer_get_properties_mutable(layer), PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT));
            if (!property) throw std::runtime_error("Cannot mark grid support layer");
            property->interface_id = this_id;
            property->reserved = 0;
            installed_ids[z] = this_id;
            const ExPolygons slices = union_ex(all);
            expolygons_copy(layer_region_borrow_mutable_slices(layer_get_region_mutable(layer, 0)),
                reinterpret_cast<const expolygon_collection_handle *>(&slices));
            layer_recompute_slices_and_islands_from_layer_regions(layer);
            layer_add_regions_to_islands(layer);
            for (uint32_t island_idx = 0; island_idx < layer_count_island(layer); ++island_idx) {
                layer_island_handle *island = layer_get_island_mutable(layer, island_idx);
                const Slic3r::ExPolygon &island_slice = *reinterpret_cast<const Slic3r::ExPolygon *>(layer_island_get_slice(island));
                // Clip every slab of this layer to the island and collect the three role surfaces:
                // upper (top), lower (bottom, bridged or not) and sparse base.
                Polygons top_surfaces, bridge_surfaces, bottom_surfaces, sparse_surfaces;
                for (size_t j = begin; j < end; ++j) {
                    const GeneratorLayer &slab = *sorted[j];
                    const bool base = slab.type == LayerType::Base || slab.type == LayerType::Intermediate;
                    const bool top = slab.type == LayerType::TopContact || slab.type == LayerType::TopInterface;
                    const bool bridge = slab.type == LayerType::BottomContact && slab.bridging;
                    /* A contact is typed as base when no interface layer is configured on its side
                       and the native generator would have moved that contact area into the base
                       layer: the user asked for no interface there, so the dense strip that would
                       have joined the base is just base, and the sparse algorithm fills base and
                       strip in one pass. The conditions are the native merge ones: the two support
                       roles share one extruder, the contact does not bridge (the base never
                       bridges), and the merged base layer of this print Z either has the contact's
                       thickness, the smallest of that Z, or does not exist. Otherwise the contact
                       keeps its own type and its own algorithm prints
                       it with the base settings. This is decided here because each algorithm prints
                       only the surfaces of its own type. */
                    const bool mergeable_contact = (slab.type == LayerType::TopContact && !top_enabled) ||
                        (slab.type == LayerType::BottomContact && !bottom_enabled);
                    // The merged base layer of a print Z carries the smallest thickness of its slabs,
                    // so it takes this contact only when the contact has exactly that thickness.
                    const bool base_takes_contact = z_has_base && slab.height == z_min_height;
                    const bool typed_as_base = mergeable_contact &&
                        Internal::merged_into_base(merge_regions, base_takes_contact, z_has_base, slab.bridging);
                    Polygons clipped = intersection(slab.polygons, island_slice);
                    if (base || typed_as_base) polygons_append(sparse_surfaces, std::move(clipped));
                    else if (top) polygons_append(top_surfaces, std::move(clipped));
                    else if (bridge) polygons_append(bridge_surfaces, std::move(clipped));
                    else polygons_append(bottom_surfaces, std::move(clipped));
                }
                // One owner per area: the upper surfaces keep every area they share with a lower
                // one, and the base keeps only the areas no interface covers.
                const Polygons top_union = union_(top_surfaces);
                bridge_surfaces = Internal::resolve_overlap(bridge_surfaces, top_union);
                bottom_surfaces = Internal::resolve_overlap(bottom_surfaces, top_union);
                sparse_surfaces = Internal::resolve_overlap(sparse_surfaces,
                    union_(top_union, union_(bridge_surfaces, bottom_surfaces)));
                std::map<extruder_index_t, Slic3r::SurfaceCollection> surfaces;
                if (!top_surfaces.empty())
                    surfaces[interface_extruder].append(union_ex(top_surfaces), stModSupport | stPosTop | stDensSolid);
                if (!bridge_surfaces.empty())
                    surfaces[interface_extruder].append(union_ex(bridge_surfaces), stModSupport | stPosBottom | stDensSolid | stModBridge);
                if (!bottom_surfaces.empty())
                    surfaces[interface_extruder].append(union_ex(bottom_surfaces), stModSupport | stPosBottom | stDensSolid);
                if (!sparse_surfaces.empty())
                    surfaces[base_extruder].append(union_ex(sparse_surfaces), stModSupport | stPosInternal | stDensSparse);
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
        i = z_end;
    }
}
} // namespace

// Decide whether the native generator would have moved a contact area into the base layer: the
// support regions must share one extruder, the contact must not bridge (the base never bridges, and
// the native merge also requires one bridging flag), and the merged base layer of that print Z must
// either have the contact's thickness or not exist at that print Z. That merged base layer is one
// layer per print Z, so its thickness is the smallest thickness among the slabs of the Z: a thicker
// base in another (Z, thickness) group of the same print Z does not prevent the contact joining it.
bool Internal::merged_into_base(bool merge, bool base_takes_contact, bool z_has_base, bool bridging)
{
    if (!merge || bridging) return false;
    return base_takes_contact || !z_has_base;
}

// Subtract the covering areas from the surfaces and leave the covering ones untouched; both sets
// stay owned by the caller, and an empty input short-circuits the Clipper difference.
Slic3r::Polygons Internal::resolve_overlap(const Slic3r::Polygons &surfaces, const Slic3r::Polygons &covering)
{
    if (surfaces.empty() || covering.empty()) return surfaces;
    return diff(surfaces, covering);
}

// Register the static provider with the host, which manages activation and shared scratch storage.
void register_support_family_grid_plugin(orchestrator_handle *orchestrator)
{
    orchestrator_register_plugin(orchestrator, SupportFamilyGrid::instance(orchestrator).c_instance());
}
}
