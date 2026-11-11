///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "SupportInterfaceGapFill.hpp"

#include <algorithm>
#include <cstring>
#include <map>
#include <stdexcept>
#include <tuple>

#include "libslic3r/Api/plugin/c/steps/slic3r_step_support_toolpath.h"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/GuiRuleHelpers.hpp"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"
#include "libslic3r/Plugins/Common/ResidualGapFill.hpp"
#include "libslic3r/Plugins/Support/Common/ClipperResults.hpp"
#include "libslic3r/Plugins/Support/Common/SupportParams.hpp"
#include "libslic3r/Plugins/Support/Common/SupportSettings.hpp"
#include "libslic3r/Plugins/Support/Toolpath/LegacySupportPaths.hpp"

/* Complete dense support interfaces after their selected providers publish paths.
   Published surfaces bound additions; same-Z toolpaths describe occupied space.
   All additions are staged before publication so insertion invalidates no reads.
   Object callbacks share no mutable calculation state. Storage is borrowed from
   the host and encloses all returned geometry and snapshots.

       register_support_interface_gap_fill_plugin -> instance: host metadata
       inilialize_impl -> option, print.ui fragment and GUI activation rule
       run_impl -> process: select dense passes and nominal flows
           surfaces / existing_coverage -> residuals -> GapFill::generate
           overlaps -> Internal::modulate: only new paths
           stage -> GapFill::ordered_boundary / append_additions: publish last

   PluginBase contains exceptions and propagates cancellation at the C ABI.
   Unknown providers are left intact because they may reserve unpublished masks. */
namespace slic3r_api::Support::Toolpath {
namespace {
constexpr const char *k_option = "support_material_interface_gap_fill";
constexpr raw_extrusion_role k_bucket = RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL_INTERFACE;
// Each dense pass accepts its ordinary provider or the matching interface-perimeter provider.
struct Role { const char *group; const char *legacy; const char *perimeters; bool top; bool contact; };
const Role roles[] = {
    {"support_toolpath_bottom_contact", "support.toolpath.legacy_bottom_contact", "nematx.support.bottom_contact", false, true},
    {"support_toolpath_bottom_interface", "support.toolpath.legacy_bottom", "nematx.support.bottom_interface", false, false},
    {"support_toolpath_top_contact", "support.toolpath.legacy_top_contact", "nematx.support.top_contact", true, true},
    {"support_toolpath_top_interface", "support.toolpath.legacy_top", "nematx.support.top_interface", true, false}
};
// Owned staged paths grouped by borrowed destination layer, island and tool.
struct Addition { layer_handle *layer; uint32_t island; extruder_index_t tool; StoredExtrusionEntity paths; };
// Resolve dense passes, generate nominal paths, modulate new paths, then stage and publish them; failures propagate.
void process(const plugin_run_context *, orchestrator_handle *);
// Copy pass surfaces into borrowed storage; bridge -1 includes both bridge classifications.
StoredPolygonCollection surfaces(storage_handle *, const layer_handle *, const Role &, int bridge = -1);
// Offset same-Z buckets; only related non-bridged pieces inside the domain use spacing.
StoredPolygonCollection existing_coverage(storage_handle *, const std::vector<layer_handle *> &,
    layer_handle *, extruder_index_t, c_flow, const ExPolygonCollection &);
// Collect strictly lower slabs for the ordinary support modulator, ordered by Z.
std::vector<Internal::Overlap> overlaps(storage_handle *, const std::vector<layer_handle *> &, layer_handle *, const Role &);
// Move new child groups to the first crossing island and coalesce destinations; missing islands throw.
void stage(storage_handle *, layer_handle *, extruder_index_t, StoredExtrusionEntity &, std::vector<Addition> &);
}

// Register immutable metadata on the host; no print data survives a run.
void register_support_interface_gap_fill_plugin(orchestrator_handle *host);

void register_support_interface_gap_fill_plugin(orchestrator_handle *host)
{
    orchestrator_register_plugin(host, SupportInterfaceGapFill::instance(host).c_instance());
}

SupportInterfaceGapFill &SupportInterfaceGapFill::instance(orchestrator_handle *host)
{
    static SupportInterfaceGapFill provider(host);
    return provider;
}

int32_t SupportInterfaceGapFill::used_config_keys(raw_used_config_key *keys) const noexcept
{
    int32_t count = Common::append_used_config_keys(keys, 0, Common::support_geometry_keys());
    count = Common::append_used_config_keys(keys, count, Common::support_path_keys());
    if (keys) {
        keys[count] = {k_option, RAW_CO_BOOL, RAW_CONTAINER_TYPE_OBJECT, RAW_PRESET_TYPE_FFF_PRINT};
        keys[count + 1] = {"support_material", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE};
        keys[count + 2] = {"raft_layers", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE};
    }
    return count + 3;
}

int32_t SupportInterfaceGapFill::defined_config_keys(const char **keys) const noexcept
{
    if (keys) keys[0] = k_option;
    return 1;
}

void SupportInterfaceGapFill::inilialize_impl(storage_handle *) const
{
    raw_config_option_def option = raw_config_option_def_init();
    option.opt_key = k_option;
    option.type = RAW_CO_BOOL;
    option.container_type = RAW_CONTAINER_TYPE_OBJECT;
    option.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    option.printer_technology = RAW_PT_FFF;
    option.category = RAW_OPTION_CATEGORY_SUPPORT;
    option.level = RAW_OPTION_LEVEL_ADVANCED;
    option.mode = RAW_CONFIG_OPTION_MODE_ADV_EXP | RAW_CONFIG_OPTION_MODE_SUSI;
    option.invalidates_step = STEP_SUPPORT_TOOLPATH;
    option.label = option.full_label = "Interface gap fill";
    option.tooltip = "Complete narrow uncovered areas of full-density support interfaces. Top contacts with removable loops are left unchanged.";
    option.default_serialized_value = "0";
    option.translation_domain = "Slic3r";
    if (orchestrator_create_option_def(m_orchestrator, &option) != OPTION_DEF_ERROR_OK)
        throw std::runtime_error("Cannot define support interface gap fill");
    const char *layout = "page:Support material\ngroup:Options for support material and raft\n"
        "setting:insert$aftersetting$support_material_interface_spacing:support_material_interface_gap_fill\n";
    if (orchestrator_add_ui_fragment(m_orchestrator, "print.ui", "support.toolpath.interface_gap_fill", layout, 0) < 0)
        throw std::runtime_error("Cannot place support interface gap fill");
    GuiRule::activate(k_option).when(all_of({
        any_of({GuiCondition::true_("support_material"), GuiCondition::non_zero("raft_layers")}),
        GuiCondition::non_zero("support_material_interface_layers"),
        GuiCondition::float_equals("support_material_interface_spacing", 0.)})).add_to(m_orchestrator);
}

void SupportInterfaceGapFill::run_impl(const plugin_run_context *context) const
{
    process(context, m_orchestrator);
    progress().finish_run();
}

namespace {
void process(const plugin_run_context *context, orchestrator_handle *host)
{
    throw_if_cancelled(context);
    const run_ctx_support_toolpath *payload = plugin_ctx_as_support_toolpath(context);
    if (!payload || !payload->object || !payload->print) throw std::runtime_error("Missing support gap fill object");
    const Object object(payload->object);
    const Config config = object.config();
    // Spaced interfaces deliberately leave room between paths.
    if (!config.bool_or_default(k_option, false) || config.get("support_material_interface_spacing").get_float() != 0.) return;
    const Print print(payload->print);
    const Common::SupportParams geometry(print, object);
    const Config region = object.default_region_config(print);
    storage_handle *storage = context->plugin_storage;
    const ClipperContext clipper(storage);
    std::vector<layer_handle *> layers;
    for (uint32_t i = 0; i < object_count_auxiliary_layer(payload->object); ++i) {
        layer_handle *layer = object_get_auxiliary_layer(payload->object, i);
        if (plugin_property_has(layer_get_properties(layer), PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT)) layers.push_back(layer);
    }
    std::vector<Addition> additions;
    // Later passes at the same Z must account for paths already staged there.
    std::map<coord_t, std::vector<GapFill::Path>> added;
    bool supported[4] = {};
    for (size_t r = 0; r < 4; ++r) {
        const orchestrator_plugin_handle *selected = orchestrator_select_plugin(host, config.handle(), STEP_SUPPORT_TOOLPATH, roles[r].group);
        const char *id = orchestrator_plugin_id(selected);
        supported[r] = id && (std::strcmp(id, roles[r].legacy) == 0 || std::strcmp(id, roles[r].perimeters) == 0);
        if (id && !supported[r]) orchestrator_plugin_report_warning(context->host_context,
            "Support interface gap fill left an unknown toolpath provider unchanged.");
    }
    const extruder_index_t tool = geometry.interface_tool > 0 ? geometry.interface_tool - 1 : EXTRUDER_ID_UNKNOWN;
    for (layer_handle *layer : layers) for (size_t r = 0; r < 4; ++r) {
        throw_if_cancelled(context);
        const Role &role = roles[r];
        // Base-setting passes and removable top-contact loops reserve unprinted corridors.
        if (!supported[r] || !(role.top ? geometry.top_published_interfaces_enabled : geometry.bottom_published_interfaces_enabled) ||
            (role.top && role.contact && geometry.interface_contact_loops)) continue;
        const bool first = layer_get_print_z(layer) - layer_get_height(layer) <= 0;
        const float density = float(first ? geometry.raft_first_layer_density : geometry.interface_density);
        if (density <= float(1. - EPSILON)) continue;
        // Bridged and ordinary surfaces need separate nominal flows.
        for (int bridge = 0; bridge < 2; ++bridge) {
            StoredPolygonCollection rings = surfaces(storage, layer, role, bridge);
            if (rings.empty()) continue;
            StoredExPolygonCollection domain = clipper_union(clipper(rings)).to_expolygon_collection();
            const raw_extrusion_role flow_role = geometry.top_interfaces_enabled ? k_bucket : RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL;
            const c_flow flow = first ? geometry.first_flow : object.support_flow(flow_role, geometry.layer_height,
                unscaled(layer_get_height(layer)), bridge ? OBJECT_SUPPORT_FLOW_BRIDGE : 0);
            const GapFill::Limits limits = GapFill::resolve_limits([&region](const char *key, double base) {
                return region.get(key).get_effective_value(base);
            }, unscaled(flow.width), geometry.resolution);
            StoredPolygonCollection occupied = existing_coverage(storage, layers, layer, tool, flow, domain);
            StoredPolygonCollection staged = GapFill::coverage(storage, added[layer_get_print_z(layer)], flow.spacing_ratio, GapFill::Footprint::Physical);
            occupied.append_move_from(staged);
            StoredExPolygonCollection residual = clipper_diff(clipper(domain), clipper(occupied)).to_expolygon_collection();
            StoredExtrusionEntity generated = GapFill::generate(storage, residual, domain, flow, limits,
                RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL_INTERFACE, context);
            if (generated.readonly().empty()) continue;
            // Only new paths are lowered above overlapping support slabs.
            Internal::modulate(storage, generated.mutable_view(), layer_get_print_z(layer),
                overlaps(storage, layers, layer, role), geometry.resolution);
            if (generated.readonly().empty()) continue;
            GapFill::collect_paths(generated.readonly(), added[layer_get_print_z(layer)]);
            stage(storage, layer, tool, generated, additions);
        }
    }
    // Publish after every read; old properties move with old content, not onto additions.
    for (Addition &addition : additions) {
        throw_if_cancelled(context);
        layer_island_handle *island = layer_get_island_mutable(addition.layer, addition.island);
        layer_region_island_handle *region_island = layer_island_get_or_create_region_island(island, nullptr, 0, addition.tool);
        if (!region_island) throw std::runtime_error("Cannot access interface gap fill island");
        extrusion_entity_handle *root = layer_region_island_get_mutable_extrusion(region_island, k_bucket);
        if (!root) throw std::runtime_error("Cannot access interface gap fill destination");
        MutableExtrusionEntity queue = GapFill::ordered_boundary(MutableExtrusionEntity(root));
        GapFill::append_additions(queue, addition.paths);
    }
}

StoredPolygonCollection existing_coverage(storage_handle *storage, const std::vector<layer_handle *> &layers,
    layer_handle *target, extruder_index_t tool, c_flow flow, const ExPolygonCollection &domain)
{
    StoredPolygonCollection result(storage);
    for (const layer_handle *layer : layers) {
        if (layer_get_print_z(layer) != layer_get_print_z(target)) continue;
        for (uint32_t i = 0; i < layer_count_island(layer); ++i) {
            const layer_island_handle *island = layer_get_island(layer, i);
            for (uint32_t r = 0; r < layer_island_count_region_island(island); ++r) {
                const layer_region_island_handle *region = layer_island_get_region_island(island, r);
                for (const raw_extrusion_role bucket : {RAW_EXTRUSION_ROLE_SUPPORT, k_bucket}) {
                    if (!layer_region_island_has_extrusion(region, bucket)) continue;
                    std::vector<GapFill::Path> paths;
                    GapFill::collect_paths(ExtrusionEntity(layer_region_island_get_extrusion(region, bucket)), paths);
                    const bool related = layer == target && bucket == k_bucket && layer_region_island_extruder_id(region) == tool;
                    std::vector<GapFill::Path> inside, outside;
                    // Mixed buckets use this pass's spacing only for pieces inside its domain.
                    if (related) {
                        for (const GapFill::Path &path : paths) {
                            StoredPolyline line(storage);
                            line.insert_array(0, path.points.data(), uint32_t(path.points.size()));
                            std::tuple<StoredPolylineCollection, StoredPolylineCollection> pieces =
                                clipper_split_polyline_expolygons(storage, line, domain);
                            for (const Polyline piece : std::get<0>(pieces)) inside.push_back({piece.points(), path.attributes});
                            for (const Polyline piece : std::get<1>(pieces)) outside.push_back({piece.points(), path.attributes});
                        }
                    }
                    const GapFill::Footprint kind = !related || flow.is_bridge ? GapFill::Footprint::Physical : GapFill::Footprint::Spacing;
                    StoredPolygonCollection footprint = GapFill::coverage(storage, related ? inside : paths, flow.spacing_ratio, kind);
                    if (related) {
                        StoredPolygonCollection obstacles = GapFill::coverage(storage, outside, flow.spacing_ratio, GapFill::Footprint::Physical);
                        footprint.append_move_from(obstacles);
                    }
                    result.append_move_from(footprint);
                }
            }
        }
    }
    return result;
}

std::vector<Internal::Overlap> overlaps(storage_handle *storage, const std::vector<layer_handle *> &layers,
    layer_handle *target, const Role &role)
{
    const coord_t z = layer_get_print_z(target), bottom = z - layer_get_height(target);
    // Bottom passes also count lower sparse and non-bridged bottom slabs when
    // they are contacts or their layer has no bottom contact.
    const bool lower = !role.top && (role.contact || surfaces(storage, target, roles[0]).empty());
    const ClipperContext clipper(storage);
    std::vector<Internal::Overlap> result;
    for (const layer_handle *layer : layers) {
        const coord_t other_z = layer_get_print_z(layer);
        if (other_z <= bottom || other_z >= z) continue;
        StoredPolygonCollection polygons(storage);
        for (const Role &candidate : roles) {
            if (!candidate.top && !lower) continue;
            StoredPolygonCollection areas = surfaces(storage, layer, candidate, candidate.top ? -1 : 0);
            polygons.append_move_from(areas);
        }
        if (lower) {
            for (uint32_t i = 0; i < layer_count_island(layer); ++i) {
                const layer_island_handle *island = layer_get_island(layer, i);
                for (uint32_t r = 0; r < layer_island_count_region_island(island); ++r) {
                    const layer_region_island_handle *region = layer_island_get_region_island(island, r);
                    for (uint32_t s = 0; s < layer_region_island_count_fill_surface(region); ++s) {
                        const Surface surface(layer_region_island_get_fill_surface(region, s));
                        const raw_surface_type mask = raw_surface_type(RAW_SURFACE_TYPE_MOD_SUPPORT |
                            RAW_SURFACE_TYPE_POS_INTERNAL | RAW_SURFACE_TYPE_DENS_SPARSE);
                        if ((surface.type() & mask) == mask) Common::append_polygons(polygons, surface.expolygon());
                    }
                }
            }
        }
        if (!polygons.empty()) result.push_back({other_z, clipper_union(clipper(polygons)).to_polygon_collection()});
    }
    std::stable_sort(result.begin(), result.end(), [](const Internal::Overlap &a, const Internal::Overlap &b) { return a.z < b.z; });
    return result;
}

StoredPolygonCollection surfaces(storage_handle *storage, const layer_handle *layer, const Role &role, int bridge)
{
    StoredPolygonCollection result(storage);
    for (uint32_t i = 0; i < layer_count_island(layer); ++i) {
        const layer_island_handle *island = layer_get_island(layer, i);
        for (uint32_t r = 0; r < layer_island_count_region_island(island); ++r) {
            const layer_region_island_handle *region = layer_island_get_region_island(island, r);
            for (uint32_t s = 0; s < layer_region_island_count_fill_surface(region); ++s) {
                const Surface surface(layer_region_island_get_fill_surface(region, s));
                const raw_surface_type type = surface.type();
                const raw_surface_type mask = raw_surface_type(RAW_SURFACE_TYPE_MOD_SUPPORT | RAW_SURFACE_TYPE_DENS_SOLID |
                    (role.top ? RAW_SURFACE_TYPE_POS_TOP : RAW_SURFACE_TYPE_POS_BOTTOM));
                if ((type & mask) != mask || bool(surface_type_is_contact(type)) != role.contact ||
                    (bridge >= 0 && bool(type & RAW_SURFACE_TYPE_MOD_BRIDGE) != bool(bridge))) continue;
                Common::append_polygons(result, surface.expolygon());
            }
        }
    }
    return result;
}

void stage(storage_handle *storage, layer_handle *layer, extruder_index_t tool,
    StoredExtrusionEntity &generated, std::vector<Addition> &out)
{
    for (uint32_t c = 0; c < generated.readonly().child_count(); ++c) {
        MutableExtrusionEntity child = generated.child_mutable(c);
        if (child.readonly().empty()) continue;
        std::vector<GapFill::Path> paths;
        GapFill::collect_paths(child.readonly(), paths);
        uint32_t assigned = layer_count_island(layer);
        for (uint32_t i = 0; i < layer_count_island(layer) && assigned == layer_count_island(layer); ++i) {
            StoredExPolygonCollection slice(storage, ExPolygon(layer_island_get_slice(layer_get_island(layer, i))));
            for (const GapFill::Path &path : paths) {
                StoredPolyline line(storage);
                line.insert_array(0, path.points.data(), uint32_t(path.points.size()));
                if (!clipper_intersection_polyline_expolygons(storage, line, slice).empty()) { assigned = i; break; }
            }
        }
        if (assigned == layer_count_island(layer)) throw std::runtime_error("Cannot assign interface gap fill to an island");
        std::vector<Addition>::iterator found = std::find_if(out.begin(), out.end(),
            [layer, assigned, tool](const Addition &a) { return a.layer == layer && a.island == assigned && a.tool == tool; });
        if (found == out.end()) {
            out.push_back({layer, assigned, tool, StoredExtrusionEntity(storage)});
            found = out.end() - 1;
        }
        if (found->paths.append_child_move(child) == EXTRUSION_INDEX_INVALID)
            throw std::runtime_error("Cannot stage interface gap fill");
    }
}
}
}
