///|/ Released under the GNU Affero General Public License, version 3 or later.
#include "InterfaceGapFill.hpp"

#include <algorithm>
#include <cstring>
#include <map>
#include <stdexcept>
#include <string>
#include <tuple>

#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/GuiRuleHelpers.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"
#include "libslic3r/Plugins/Support/Common/ClipperResults.hpp"
#include "libslic3r/Plugins/Support/Common/SupportParams.hpp"
#include "libslic3r/Plugins/Support/Common/SupportSettings.hpp"
#include "libslic3r/Plugins/Support/Toolpath/LegacySupportPaths.hpp"

/* A non-exclusive support toolpath provider completes the paths published by
   the selected legacy or interface-perimeter providers. Surfaces define where
   additions are allowed; existing paths define what is already covered. All
   additions are prepared before publication, so no read view survives insertion.
   Object runs share no mutable calculation state.

       Provider::run_impl -> process: select dense passes and nominal flows
           surfaces / existing_coverage -> residuals -> generate
           overlaps -> Internal::modulate: only the new paths
           stage -> publish: preserve old content before the new paths

   PluginBase catches exceptions and propagates cancellation across the C ABI.
   Unknown providers are left intact because they may reserve unpublished masks. */
namespace nematx::interface_gaps {
namespace {
namespace Common = slic3r_api::Support::Common;
namespace Toolpath = slic3r_api::Support::Toolpath;
constexpr const char *k_option = "support_material_interface_gap_fill";
constexpr raw_extrusion_role k_bucket = RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL_INTERFACE;
// One interface pass: its provider group, the two providers this plugin knows for it
// (legacy and interface perimeters), and which side and surface kind it fills.
// roles[0] is the bottom contact pass.
struct Role { const char *group; const char *legacy; const char *perimeters; bool top; bool contact; };
const Role roles[] = {
    {"support_toolpath_bottom_contact", "support.toolpath.legacy_bottom_contact", "nematx.support.bottom_contact", false, true},
    {"support_toolpath_bottom_interface", "support.toolpath.legacy_bottom", "nematx.support.bottom_interface", false, false},
    {"support_toolpath_top_contact", "support.toolpath.legacy_top_contact", "nematx.support.top_contact", true, true},
    {"support_toolpath_top_interface", "support.toolpath.legacy_top", "nematx.support.top_interface", true, false}
};
// New paths waiting for publication, grouped by destination layer, island and tool.
struct Addition {
    layer_handle *layer;
    uint32_t island;
    extruder_index_t tool;
    StoredExtrusionEntity paths;
};
// Copy the final surfaces of this pass without changing the contact classification.
// bridge = 0 or 1 keeps only non-bridged or bridged surfaces; -1 keeps both.
StoredPolygonCollection surfaces(storage_handle *, const layer_handle *, const Role &, int bridge = -1);
// Read same-Z support buckets; unrelated tools and layers are physical obstacles.
StoredPolygonCollection existing_coverage(storage_handle *, const std::vector<layer_handle *> &,
    layer_handle *, extruder_index_t, c_flow, const ExPolygonCollection &);
// Collect the same strictly lower slabs used by the ordinary support modulator.
std::vector<Toolpath::Internal::Overlap> overlaps(storage_handle *, const std::vector<layer_handle *> &,
    layer_handle *, const Role &);
// Assign new child groups to the first crossing island, then coalesce destinations.
void stage(storage_handle *, layer_handle *, extruder_index_t, StoredExtrusionEntity &, std::vector<Addition> &);
// Resolve passes, build all additions, then publish behind an ordered boundary.
void process(const plugin_run_context *, orchestrator_handle *);

class Provider final : public PluginBase {
public:
    // The host owns this process-lifetime instance; it stores no print data.
    explicit Provider(orchestrator_handle *host) : PluginBase(host) {}
private:
    // Stable id used in activation files; the plugin joins no exclusive group.
    const char *id_impl() const noexcept override { return "nematx.support.interface_gap_fill"; }
    // Label shown in the plugin list.
    const char *name_impl() const noexcept override { return "Support interface gap fill"; }
    // Explain the operation in plugin selection and diagnostics.
    const char *description_impl() const noexcept override { return "Complete uncovered narrow areas of dense support interfaces."; }
    // Consume the surfaces and paths published during the support toolpath step.
    slicing_step_t step_impl() const noexcept override { return STEP_SUPPORT_TOOLPATH; }
    // Run strictly after the current providers at priorities up to 200.
    int32_t priority_impl() const noexcept override { return 300; }
    // Geometry is consumed through the data tree, not a required provider id.
    const char *const *dependencies_impl() const noexcept override { return nullptr; }
    // Declare the settings read by SupportParams, the thresholds and selection.
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override
    {
        int32_t count = Common::append_used_config_keys(keys, 0, Common::support_geometry_keys());
        count = Common::append_used_config_keys(keys, count, Common::support_path_keys());
        if (keys) {
            keys[count] = {k_option, RAW_CO_BOOL, RAW_CONTAINER_TYPE_OBJECT, RAW_PRESET_TYPE_FFF_PRINT};
            keys[count + 1] = {"support_material", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE};
            keys[count + 2] = {"raft_layers", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE};
        }
        // Selection and its invalidation belong to the host's group settings.
        return count + 3;
    }
    // This provider alone owns the object-scoped boolean.
    int32_t defined_config_keys(const char **keys) const noexcept override
    { if (keys) keys[0] = k_option; return 1; }
    // Define the option independently of the perimeter package and its layout.
    void inilialize_impl(storage_handle *) const override
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
        if (orchestrator_add_ui_fragment(m_orchestrator, "print.ui", "nematx.support.interface_gap_fill", layout, 0) < 0)
            throw std::runtime_error("Cannot place support interface gap fill");
        GuiRule::activate(k_option).when(all_of({
            any_of({GuiCondition::true_("support_material"), GuiCondition::non_zero("raft_layers")}),
            GuiCondition::non_zero("support_material_interface_layers"),
            GuiCondition::float_equals("support_material_interface_spacing", 0.)})).add_to(m_orchestrator);
    }
    // Per-object state stays on the stack; exceptions are handled by PluginBase.
    void run_impl(const plugin_run_context *context) const override
    { process(context, m_orchestrator); progress().finish_run(); }
};

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
                    std::vector<Path> paths;
                    collect_paths(ExtrusionEntity(layer_region_island_get_extrusion(region, bucket)), paths);
                    const bool related = layer == target && bucket == k_bucket && layer_region_island_extruder_id(region) == tool;
                    std::vector<Path> inside, outside;
                    // A bucket may mix surface roles and bridge recipes. Only
                    // centerline pieces in this domain use this pass's spacing.
                    if (related) {
                        for (const Path &path : paths) {
                            StoredPolyline line(storage);
                            line.insert_array(0, path.points.data(), uint32_t(path.points.size()));
                            std::tuple<StoredPolylineCollection, StoredPolylineCollection> pieces =
                                clipper_split_polyline_expolygons(storage, line, domain);
                            for (const Polyline piece : std::get<0>(pieces)) inside.push_back({piece.points(), path.attributes});
                            for (const Polyline piece : std::get<1>(pieces)) outside.push_back({piece.points(), path.attributes});
                        }
                    }
                    StoredPolygonCollection footprint = coverage(storage, related ? inside : paths, flow, !related);
                    if (related) {
                        StoredPolygonCollection obstacles = coverage(storage, outside, flow, true);
                        footprint.append_move_from(obstacles);
                    }
                    result.append_move_from(footprint);
                }
            }
        }
    }
    return result;
}

std::vector<Toolpath::Internal::Overlap> overlaps(storage_handle *storage, const std::vector<layer_handle *> &layers,
    layer_handle *target, const Role &role)
{
    const coord_t z = layer_get_print_z(target), bottom = z - layer_get_height(target);
    // Top surfaces below always count. Like the support generator, a bottom contact pass,
    // or a bottom interface pass on a layer without bottom contact, also counts lower
    // sparse and non-bridged bottom surfaces.
    const bool lower = !role.top && (role.contact || surfaces(storage, target, roles[0]).empty());
    const ClipperContext clipper(storage);
    std::vector<Toolpath::Internal::Overlap> result;
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
    std::stable_sort(result.begin(), result.end(), [](const Toolpath::Internal::Overlap &a, const Toolpath::Internal::Overlap &b) { return a.z < b.z; });
    return result;
}

void stage(storage_handle *storage, layer_handle *layer, extruder_index_t tool,
    StoredExtrusionEntity &generated, std::vector<Addition> &out)
{
    for (uint32_t c = 0; c < generated.readonly().child_count(); ++c) {
        MutableExtrusionEntity child = generated.child_mutable(c);
        if (child.readonly().empty()) continue;
        std::vector<Path> paths;
        collect_paths(child.readonly(), paths);
        uint32_t assigned = layer_count_island(layer);
        for (uint32_t i = 0; i < layer_count_island(layer) && assigned == layer_count_island(layer); ++i) {
            StoredExPolygonCollection slice(storage, ExPolygon(layer_island_get_slice(layer_get_island(layer, i))));
            for (const Path &path : paths) {
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

void process(const plugin_run_context *context, orchestrator_handle *host)
{
    throw_if_cancelled(context);
    const run_ctx_support_toolpath *payload = plugin_ctx_as_support_toolpath(context);
    if (!payload || !payload->object || !payload->print) throw std::runtime_error("Missing support gap fill object");
    const Object object(payload->object);
    const Config config = object.config();
    // Only interfaces printed without spacing can have narrow uncovered areas to fill.
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
    // New paths per print Z, so later passes at the same height do not fill them again.
    std::map<coord_t, std::vector<Path>> added;
    // A pass is completed only when its selected provider is a known one: an unknown
    // provider may keep areas empty on purpose.
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
        // Skip passes printed with base settings, and top contacts whose removable
        // loops reserve a corridor that the published surfaces do not show.
        if (!supported[r] || !(role.top ? geometry.top_published_interfaces_enabled : geometry.bottom_published_interfaces_enabled) ||
            (role.top && role.contact && geometry.interface_contact_loops)) continue;
        // Only full-density passes are completed, with the first layer's own density.
        const bool first = layer_get_print_z(layer) - layer_get_height(layer) <= 0;
        const float density = float(first ? geometry.raft_first_layer_density : geometry.interface_density);
        if (density <= float(1. - EPSILON)) continue;
        // Bridged and non-bridged surfaces use different flows: complete them separately.
        for (int bridge = 0; bridge < 2; ++bridge) {
            StoredPolygonCollection rings = surfaces(storage, layer, role, bridge);
            if (rings.empty()) continue;
            StoredExPolygonCollection domain = clipper_union(clipper(rings)).to_expolygon_collection();
            // Rebuild the flow the provider used. Its role follows top_interfaces_enabled,
            // which differs from the published-interface flags tested above.
            const raw_extrusion_role flow_role = geometry.top_interfaces_enabled ? k_bucket : RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL;
            const c_flow flow = first ? geometry.first_flow : object.support_flow(flow_role, geometry.layer_height,
                unscaled(layer_get_height(layer)), bridge ? OBJECT_SUPPORT_FLOW_BRIDGE : 0);
            const double width = unscaled(flow.width);
            Limits limits;
            limits.min_width = region.get("gap_fill_min_width").get_effective_value(width);
            limits.max_width = region.get("gap_fill_max_width").get_effective_value(width);
            limits.min_area = region.get("gap_fill_min_area").get_effective_value(width * width);
            limits.min_length = region.get("gap_fill_min_length").get_effective_value(width);
            limits.extension = region.get("gap_fill_extension").get_effective_value(width);
            limits.resolution = geometry.resolution;
            // Residual = pass surfaces minus existing paths and paths added at this Z.
            StoredPolygonCollection occupied = existing_coverage(storage, layers, layer, tool, flow, domain);
            StoredPolygonCollection staged = coverage(storage, added[layer_get_print_z(layer)], flow, true);
            occupied.append_move_from(staged);
            StoredExPolygonCollection residual = clipper_diff(clipper(domain), clipper(occupied)).to_expolygon_collection();
            StoredExtrusionEntity generated = generate(storage, residual, domain, flow, limits);
            if (generated.readonly().empty()) continue;
            // Lower the height and flow of new paths above lower support layers, as the
            // provider did for its own paths; modulation may drop paths entirely.
            Toolpath::Internal::modulate(storage, generated.mutable_view(), layer_get_print_z(layer),
                overlaps(storage, layers, layer, role), geometry.resolution);
            if (generated.readonly().empty()) continue;
            collect_paths(generated.readonly(), added[layer_get_print_z(layer)]);
            stage(storage, layer, tool, generated, additions);
        }
    }
    // Publish only now, so no read above sees a modified tree. The old content moves
    // into an ordered leaf with its properties, and the new paths follow it.
    for (Addition &addition : additions) {
        throw_if_cancelled(context);
        layer_island_handle *island = layer_get_island_mutable(addition.layer, addition.island);
        layer_region_island_handle *region_island = layer_island_get_or_create_region_island(island, nullptr, 0, addition.tool);
        if (!region_island) throw std::runtime_error("Cannot access interface gap fill island");
        extrusion_entity_handle *root = layer_region_island_get_mutable_extrusion(region_island, k_bucket);
        if (!root) throw std::runtime_error("Cannot access interface gap fill destination");
        MutableExtrusionEntity queue = MutableExtrusionEntity(root).emplace_ordered_leaf(
            OrderedLeafPosition::After, ExistingPropertyPlacement::MoveWithExistingContent);
        if (!queue.valid() || queue.append_child_move(addition.paths.mutable_view()) == EXTRUSION_INDEX_INVALID)
            throw std::runtime_error("Cannot publish interface gap fill");
    }
}
}
}

// Register one ordinary provider; its instance is immutable between object runs.
extern "C" SLIC3R_PLUGIN_API void register_plugin(orchestrator_handle *host)
{
    static nematx::interface_gaps::Provider provider(host);
    orchestrator_register_plugin(host, provider.c_instance());
}
#include "libslic3r/Api/plugin/c/slic3r_plugin_register_version.h"
