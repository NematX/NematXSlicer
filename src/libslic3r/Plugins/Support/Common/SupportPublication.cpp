///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#include "SupportPublication.hpp"

#include <algorithm>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>
#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"
#include "libslic3r/Api/plugin/cpp/properties/DataTreeProperties.hpp"
#include "libslic3r/Plugins/Support/Common/ClipperResults.hpp"

/* Publication turns the slabs of a GeneratedSupport into auxiliary layers, for any support family.
   Slabs sharing the same exact print Z and printable thickness become one auxiliary layer, whose
   raw slice is their union in object region zero, without modifier splitting. Rebuilding slices
   creates islands before region membership is locked. Each island then gets one work region per
   extruder that prints on it, holding dense top or bottom surfaces, or sparse internal support
   surfaces (base slabs, and branch slabs typed as branch surfaces), each typed by its role.
   Publication gives each area one owner: the lower surfaces lose the areas the upper ones print,
   the branch and the base lose the areas any interface covers, and the base also loses the areas a
   branch prints. It does not separate bridged lower slabs from non-bridged ones, so the family must
   keep those two disjoint.
   Extruder zero
   means the current extruder, stored as EXTRUDER_ID_UNKNOWN. Temporary API collections use shared
   plugin storage but are released individually; a run never clears that shared storage. Groups of
   one print Z share the native interface id of that Z, and they are published thinnest first so the
   host's ordered insertion keeps the thickest layer first at that Z. */

namespace slic3r_api::Support::Common {
// Group non-empty slabs by integer Z and thickness and publish their union before locking islands; equal-Z insertion precedes existing layers with unique creation ids, and rejected API writes throw.
// Inputs and context are borrowed; the object owns published layers. Rejected writes exit through silent
// PluginCancelled when stopping (the cause has its message, the step names it, or the runner rethrows the print
// cancellation), otherwise std::runtime_error.
void publish(const plugin_run_context *context, const Object &object, GeneratedSupport &support, orchestrator_handle *orchestrator)
{
    storage_handle *storage = support.storage;
    const ClipperContext clipper(storage);
    GeneratorLayers sorted = support.top;
    sorted.insert(sorted.end(), support.bottom.begin(), support.bottom.end());
    sorted.insert(sorted.end(), support.base.begin(), support.base.end());
    sorted.insert(sorted.end(), support.interfaces.begin(), support.interfaces.end());
    std::sort(sorted.begin(), sorted.end(), [](const GeneratorLayer *a, const GeneratorLayer *b) { return *a < *b; });
    const Config config = object.config();
    const int32_t base_tool = config.get("support_material_extruder").get_int();
    const int32_t interface_tool = config.get("support_material_interface_extruder").get_int();
    const bool top_enabled = config.get("support_material_interface_layers").get_int() != 0;
    const ConfigOption bottom_layers = config.get("support_material_bottom_interface_layers");
    // Index -1 asks whether the option as a whole is enabled, like the native is_enabled().
    const bool bottom_enabled = config_option_is_enabled(bottom_layers.handle(), -1) != 0 ?
        top_enabled && bottom_layers.get_int() != 0 : top_enabled;
    const extruder_index_t base_extruder = base_tool > 0 ? base_tool - 1 : EXTRUDER_ID_UNKNOWN;
    const extruder_index_t interface_extruder = interface_tool > 0 ? interface_tool - 1 : EXTRUDER_ID_UNKNOWN;
    // The native generator only moves a contact into the base when both support roles share one
    // extruder, so publication types the area the same way.
    const bool merge_regions = base_tool == interface_tool;
    const object_handle *object_ptr = object.handle();
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
                if (slab.type == LayerType::Base || slab.type == LayerType::Intermediate || slab.type == LayerType::Branch) z_has_base = true;
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
            StoredPolygonCollection all(storage);
            for (size_t j = begin; j < end; ++j) all.append_copy_from(sorted[j]->polygons);
            if (all.empty()) continue;
            const coord_t height = sorted[begin]->height;
            layer_handle *layer = object_add_auxiliary_layer(object_ptr, height, z, z - height / 2);
            if (!layer) {
                throw_if_cancelled(context);
                throw std::runtime_error("Cannot create support layer");
            }
            LayerSupportProperty *property = static_cast<LayerSupportProperty *>(plugin_property_get_or_add_data_mutable(
                orchestrator, layer_get_properties_mutable(layer), PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT));
            if (!property) {
                throw_if_cancelled(context);
                throw std::runtime_error("Cannot mark support layer");
            }
            property->interface_id = this_id;
            property->reserved = 0;
            installed_ids[z] = this_id;
            const StoredExPolygonCollection slices = native_expolygons(clipper_union(clipper(all)));
            expolygons_copy(layer_region_borrow_mutable_slices(layer_get_region_mutable(layer, 0)), slices.handle());
            layer_recompute_slices_and_islands_from_layer_regions(layer);
            layer_add_regions_to_islands(layer);
            for (uint32_t island_idx = 0; island_idx < layer_count_island(layer); ++island_idx) {
                layer_island_handle *island = layer_get_island_mutable(layer, island_idx);
                const ExPolygon island_slice(layer_island_get_slice(island));
                // Clip every slab of this layer to the island and collect the role surfaces: upper
                // (top), lower (bottom, bridged or not), sparse base and branch bodies.
                StoredPolygonCollection top_surfaces(storage), bridge_surfaces(storage), bottom_surfaces(storage),
                    sparse_surfaces(storage), branch_surfaces(storage);
                for (size_t j = begin; j < end; ++j) {
                    const GeneratorLayer &slab = *sorted[j];
                    const bool base = slab.type == LayerType::Base || slab.type == LayerType::Intermediate;
                    const bool branch = slab.type == LayerType::Branch;
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
                    StoredPolygonCollection clipped = clipper_intersection(clipper(slab.polygons), clipper(island_slice)).to_polygon_collection();
                    if (branch) branch_surfaces.append_move_from(clipped);
                    else if (base || typed_as_base) sparse_surfaces.append_move_from(clipped);
                    else if (top) top_surfaces.append_move_from(clipped);
                    else if (bridge) bridge_surfaces.append_move_from(clipped);
                    else bottom_surfaces.append_move_from(clipped);
                }
                // One owner per area: the upper surfaces keep every area they share with a lower
                // one, the branch keeps the areas no interface covers, and the base keeps only the
                // areas that neither an interface nor a branch covers.
                const StoredPolygonCollection top_union = clipper_union(clipper(top_surfaces)).to_polygon_collection();
                bridge_surfaces = Internal::resolve_overlap(bridge_surfaces, top_union);
                bottom_surfaces = Internal::resolve_overlap(bottom_surfaces, top_union);
                const StoredPolygonCollection lower_union =
                    clipper_union2(clipper(bridge_surfaces), clipper(bottom_surfaces)).to_polygon_collection();
                const StoredPolygonCollection covered = clipper_union2(clipper(top_union), clipper(lower_union)).to_polygon_collection();
                branch_surfaces = Internal::resolve_overlap(branch_surfaces, covered);
                sparse_surfaces = Internal::resolve_overlap(sparse_surfaces, covered);
                sparse_surfaces = Internal::resolve_overlap(sparse_surfaces, branch_surfaces);
                // Surfaces of one extruder, in the native order of their types.
                std::map<extruder_index_t, std::vector<std::pair<StoredExPolygonCollection, raw_surface_type>>> surfaces;
                const auto add = [&](extruder_index_t extruder, const StoredPolygonCollection &area, raw_surface_type type) {
                    if (!area.empty())
                        surfaces[extruder].emplace_back(native_expolygons(clipper_union(clipper(area))), type);
                };
                add(interface_extruder, top_surfaces, srf_type().top().solid().support().raw());
                add(interface_extruder, bridge_surfaces, srf_type().bottom().solid().bridge().support().raw());
                add(interface_extruder, bottom_surfaces, srf_type().bottom().solid().support().raw());
                add(base_extruder, sparse_surfaces, srf_type().internal().sparse().support().raw());
                add(base_extruder, branch_surfaces, srf_type().internal().sparse().support().branch().raw());
                for (const auto &[extruder, collection] : surfaces) {
                    StoredSurfaceCollection stored(storage);
                    for (const auto &[expolygons, type] : collection)
                        for (const ExPolygon expolygon : expolygons) stored.append(expolygon, type);
                    if (stored.empty()) continue;
                    layer_region_island_handle *region = layer_island_get_or_create_region_island(island, nullptr, 0, extruder);
                    if (!region || !layer_region_island_set_fill_surfaces(region, stored.mutable_handle())) {
                        throw_if_cancelled(context);
                        throw std::runtime_error("Cannot publish support surfaces");
                    }
                }
            }
        }
        i = z_end;
    }
}

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
StoredPolygonCollection Internal::resolve_overlap(const StoredPolygonCollection &surfaces, const StoredPolygonCollection &covering)
{
    if (surfaces.empty() || covering.empty()) return surfaces.readonly().clone(surfaces.storage());
    return clipper_diff(ClipperOperand(surfaces.storage(), surfaces.readonly()),
                        ClipperOperand(surfaces.storage(), covering.readonly())).to_polygon_collection();
}
} // namespace slic3r_api::Support::Common
