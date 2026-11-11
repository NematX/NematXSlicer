#include "support_grid_test_helpers.hpp"
#include "probe_pattern_plugin.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/cpp/AuxiliaryLayerHelpers.hpp"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_support_demand.h"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Plugins/Support/Toolpath/LegacySupportPaths.hpp"
#include "libslic3r/Plugins/Support/Toolpath/SupportToolpathLegacy.hpp"
#include "libslic3r/Plugins/Support/Common/SupportCloseZ.hpp"
#include "libslic3r/Plugins/Support/Common/SupportCounters.hpp"
#include "libslic3r/Plugins/Support/Common/SupportToolpathRoles.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>

/* Small roof-and-platform objects and synthetic layers exercise the five built-in support roles.
   The slicing prefix leaves support editable, then the normal dispatcher fills
   the typed surfaces. Leaf checks resolve inherited flow attributes, so both
   native paths and generic API wrappers are tested in their published form.
   A failing pattern checks that a legacy algorithm preserves its sole error message.
   Synthetic dense bottom surfaces distinguish marked bridging contacts from unmarked
   interfaces even when interface layers are disabled; both retain base-role settings.
    Contact cases check automatic fusion, priority ordering, selector descriptions,
    preserved metadata and earlier paths, per-component coverage and replay.
    Two cases drive the internal helpers directly: synthetic slabs for height
    modulation and plain arguments for the filler pattern choice. A variable-height grid/tree case runs the full support
    toolpath dispatcher, reports modulation counters and checks published heights and height-scaled flow.
    Selector probes check ranked dense alternatives for each style and explicit overrides.

        selector TEST_CASE -> register metadata probes -> resolve automatic/explicit choices
*/

using namespace Slic3r;
namespace SG = Slic3r::Test::SupportGrid;
namespace ToolpathInternal = slic3r_api::Support::Toolpath::Internal;
namespace {
// Scope the active providers like the infill pattern tests, restoring the saved plugin pointers afterward.
class ScopedActivePlugins
{
public:
    // Save the process-owned providers before the caller changes activation; allocation failure leaves activation intact.
    explicit ScopedActivePlugins() : m_host(Orchestrator::instance())
    {
        for (Plugin *plugin : m_host.active_plugins())
            m_previous.push_back(plugin);
    }
    // Restore activation without transferring ownership of the process-lifetime plugin instances.
    ~ScopedActivePlugins()
    {
        m_host.clear_active_plugins();
        for (Plugin *plugin : m_previous)
            m_host.set_plugin_active(plugin, true);
    }
private:
    Orchestrator &m_host;
    std::vector<Plugin *> m_previous;
};

// Metadata-only alternatives exercise contact selector elections without changing any print data.
class ContactGroupProbe final : public slic3r_api::PluginBase {
public:
    // Borrow process-lifetime strings and select the native side's execution priority.
    ContactGroupProbe(orchestrator_handle *host, const char *id, const char *group, int32_t priority)
        : PluginBase(host), m_id(id), m_group(group), m_priority(priority) {}
    // Read the static probe identity for activation and selection checks.
    const char *get_id() const noexcept { return m_id; }
    // Read the role being replaced without querying mutable host state.
    const char *get_exclusive_group() const noexcept { return m_group; }
private:
    // Return the stable registration identifier without effects or failures.
    const char *id_impl() const noexcept override { return m_id; }
    // Select the support toolpath callback step without effects or failures.
    slicing_step_t step_impl() const noexcept override { return STEP_SUPPORT_TOOLPATH; }
    // Return the contact role shared with the native provider without effects or failures.
    const char *exclusive_group_impl() const noexcept override { return m_group; }
    // Keep execution priority identical within the role without effects or failures.
    int32_t priority_impl() const noexcept override { return m_priority; }
    // Prefer the probe's selector text to the native description without changing execution order.
    int32_t exclusive_group_priority_impl() const noexcept override { return 0; }
    // Require no other providers and transfer no ownership.
    const char *const *dependencies_impl() const noexcept override { return nullptr; }
    // This metadata probe publishes no paths or surfaces and cannot fail.
    void run_impl(const plugin_run_context *) const override {}
    const char *m_id;
    const char *m_group;
    int32_t m_priority;
};

struct PathCounts { size_t base = 0, interface = 0, loops = 0, bridges = 0; double first_width = 0; };
// Check leaf role, positive flow and bucket placement and count loops and bridge flows.
void check_paths(const ExtrusionEntity &entity, ExtrusionRole bucket, PathCounts &counts, bool first, const ExtrusionAttributes *attributes = nullptr);

// Collect the polyline-bearing leaves of one extrusion tree in traversal order.
void collect_leaves(const ExtrusionEntity &entity, std::vector<const ExtrusionEntity *> &leaves);

// Sum the length of the leaves whose flow matches the circular bridging section of their height.
double bridge_length(const ExtrusionEntity &tree);
// Sum the bridging-flow interface path length published across one object's auxiliary layers.
double published_bridge_length(const PrintObject &object);
// Publish ordered disjoint surfaces in a marked auxiliary layer above the bed at the supplied scaled Z and height, moving copies into the object.
// Test assertions expose API failures; returned region is borrowed until the object is destroyed.
layer_region_island_handle *publish_test_surfaces(Print &print, PluginStorage &storage, const ExPolygons &areas,
                                                 const std::vector<raw_surface_type> &types,
                                                 coord_t print_z = scale_i(0.6), coord_t height = scale_i(0.2));

// Resolve nearest attributes and inspect every published leaf; missing flow or a role in the wrong bucket fails the test while counts accumulate owned scalar values.
void check_paths(const ExtrusionEntity &entity, ExtrusionRole bucket, PathCounts &counts, bool first, const ExtrusionAttributes *attributes)
{
    if (const ExtrusionAttributes *local = entity.get_property<ExtrusionAttributes>()) attributes = local;
    if (entity.is_loop()) ++counts.loops;
    if (entity.has_polyline()) {
        REQUIRE(attributes != nullptr);
        CHECK(attributes->width > 0);
        CHECK(attributes->height > 0);
        CHECK(attributes->mm3_per_mm > 0);
        if (bucket == LayerRegionIsland::SUPPORT) {
            CHECK(attributes->role == ExtrusionRole::SupportMaterial);
            ++counts.base;
        } else {
            CHECK(attributes->role == ExtrusionRole::SupportMaterialInterface);
            ++counts.interface;
        }
        if (std::abs(attributes->mm3_per_mm - M_PI * attributes->height * attributes->height / 4.) < 1e-6) ++counts.bridges;
        if (first) counts.first_width = attributes->width;
    }
    for (const ExtrusionEntityUPtr &child : entity.children()) check_paths(*child, bucket, counts, first, attributes);
}

// Recurse into collections and append every polyline-bearing leaf; the returned pointers stay owned by the inspected tree.
void collect_leaves(const ExtrusionEntity &entity, std::vector<const ExtrusionEntity *> &leaves)
{
    if (entity.has_polyline()) {
        leaves.push_back(&entity);
        return;
    }
    for (const ExtrusionEntityUPtr &child : entity.children()) collect_leaves(*child, leaves);
}

// Compare each leaf flow against the area of a circle of its own height and add the matching path lengths in millimetres.
double bridge_length(const ExtrusionEntity &tree)
{
    std::vector<const ExtrusionEntity *> leaves;
    collect_leaves(tree, leaves);
    double total = 0;
    for (const ExtrusionEntity *leaf : leaves) {
        const ExtrusionAttributes *attributes = leaf->get_property<ExtrusionAttributes>();
        REQUIRE(attributes != nullptr);
        if (std::abs(attributes->mm3_per_mm - M_PI * attributes->height * attributes->height / 4.) >= 1e-6) continue;
        for (const ArcPolyline &polyline : leaf->as_polylines()) total += unscaled(polyline.length());
    }
    return total;
}

// Walk the owned auxiliary layers and sum the published interface trees without changing them.
double published_bridge_length(const PrintObject &object)
{
    double total = 0;
    for (const Layer &layer : object.auxiliary_layers())
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands())
                if (region.has_extrusion(LayerRegionIsland::SUPPORT_INTERFACE))
                    total += bridge_length(region.extrusion(LayerRegionIsland::SUPPORT_INTERFACE));
    return total;
}

layer_region_island_handle *publish_test_surfaces(Print &print, PluginStorage &storage, const ExPolygons &areas,
                                                 const std::vector<raw_surface_type> &types, coord_t print_z, coord_t height)
{
    REQUIRE(areas.size() == types.size());
    PrintObject &object = print.object(0);
    const slic3r_api::AuxiliaryLayerBuildResult built = slic3r_api::build_auxiliary_layer_regions_from_subject(
        SG::handle(storage), SG::print_view(print), SG::object_view(object),
        slic3r_api::ExPolygonCollection(reinterpret_cast<const expolygon_collection_handle *>(&areas)),
        height, print_z, print_z - height / 2);
    REQUIRE(built.created);
    layer_handle *layer = nullptr;
    for (uint32_t index = 0; index < object_count_auxiliary_layer(SG::object_view(object).handle()); ++index) {
        layer_handle *candidate = object_get_auxiliary_layer(SG::object_view(object).handle(), index);
        if (candidate == built.layer.handle()) layer = candidate;
    }
    REQUIRE(layer != nullptr);
    REQUIRE(plugin_property_get_or_add_data_mutable(reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()),
        layer_get_properties_mutable(layer), PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT) != nullptr);
    layer_region_island_handle *region = layer_island_get_or_create_region_island(layer_get_island_mutable(layer, 0), nullptr, 0, 0);
    REQUIRE(region != nullptr);
    slic3r_api::StoredSurfaceCollection surfaces(SG::handle(storage));
    for (size_t index = 0; index < areas.size(); ++index)
        surfaces.append(slic3r_api::ExPolygon(reinterpret_cast<const expolygon_handle *>(&areas[index])), types[index]);
    REQUIRE(slic3r_api::LayerRegionIsland(region).set_fill_surfaces(&surfaces));
    return region;
}

}

// Ranking replaces dense roles for every family; explicit selections and removal retain their meaning.
TEST_CASE("Dense support auto selection allows ranked alternatives for every family", "[plugins][support-toolpath-legacy][support-toolpath-auto]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    namespace Common = slic3r_api::Support::Common;
    Orchestrator &host = Orchestrator::instance();
    orchestrator_handle *handle = reinterpret_cast<orchestrator_handle *>(&host);
    static ContactGroupProbe alternatives[] = {
        {handle, "test.support.dense.auto.top", Common::k_top_interface_role, 200},
        {handle, "test.support.dense.auto.top_contact", Common::k_top_contact_role, 190},
        {handle, "test.support.dense.auto.bottom", Common::k_bottom_interface_role, 100},
        {handle, "test.support.dense.auto.bottom_contact", Common::k_bottom_contact_role, 90}
    };
    const ScopedActivePlugins restore;
    for (ContactGroupProbe &alternative : alternatives) {
        if (!host.get_plugin(alternative.get_id())) {
            Orchestrator::PluginRegistrationScope scope = host.plugin_registration_scope({}, false);
            orchestrator_register_plugin(handle, alternative.c_instance());
            host.set_plugin_active(alternative.get_id(), false);
        }
        const std::string group = alternative.get_exclusive_group();
        const std::string key = Steps::plugin_exclusive_group_option_key(host, STEP_SUPPORT_TOOLPATH, group);
        for (const char *style : {"grid", "snug", "tree", "organic", "filled"}) {
            CAPTURE(group, style);
            DynamicPrintConfig config = SG::config(style);
            // Runtime probes are registered after catalogue initialization; singleton contact keys are not in that catalogue.
            config.set_key_value(key, new ConfigOptionString(SLIC3R_SELECTOR_AUTO_VALUE));
            REQUIRE(host.set_plugin_active(alternative.get_id(), false));
            const Plugin *legacy = Steps::selected_active_plugin_from_group(host, STEP_SUPPORT_TOOLPATH, group, &config);
            REQUIRE(legacy != nullptr);
            const std::string legacy_id = legacy->get_id();
            REQUIRE(legacy_id.rfind("support.toolpath.legacy_", 0) == 0);
            REQUIRE(host.set_plugin_active(alternative.get_id(), true));
            CHECK(Steps::selected_active_plugin_from_group(host, STEP_SUPPORT_TOOLPATH, group, &config)->get_id() == alternative.get_id());
            config.set_key_value(key, new ConfigOptionString(legacy_id));
            REQUIRE(config.option(key) != nullptr);
            CHECK(config.option(key)->serialize() == legacy_id);
            CHECK(Steps::selected_active_plugin_from_group(host, STEP_SUPPORT_TOOLPATH, group, &config)->get_id() == legacy_id);
            config.set_key_value(key, new ConfigOptionString(SLIC3R_SELECTOR_AUTO_VALUE));
            REQUIRE(host.set_plugin_active(alternative.get_id(), false));
            CHECK(Steps::selected_active_plugin_from_group(host, STEP_SUPPORT_TOOLPATH, group, &config)->get_id() == legacy_id);
        }
    }
}

// Publish one dense lower surface without interfaces and retain its explicitly marked bridge flow after contact hand-back.
TEST_CASE("Dense surfaces without interfaces retain explicitly marked bridge flow", "[plugins][support-toolpath-legacy][support-contact]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool contact = GENERATE(false, true);
    DynamicPrintConfig config = SG::config("grid", false, 0, false);
    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
    SG::prefix(print);
    PrintObject &object = print.object(0);
    const slic3r_api::Support::Common::SupportParams geometry = SG::params(print, object);
    REQUIRE_FALSE(geometry.bottom_published_interfaces_enabled);
    REQUIRE_FALSE(geometry.soluble);
    const ExPolygons subject{ExPolygon(Polygon({Point(0, 0), Point(scale_i(10.), 0),
        Point(scale_i(10.), scale_i(10.)), Point(0, scale_i(10.))}))};
    PluginStorage storage;
    const slic3r_api::AuxiliaryLayerBuildResult built = slic3r_api::build_auxiliary_layer_regions_from_subject(
        reinterpret_cast<storage_handle *>(&storage), slic3r_api::Print(reinterpret_cast<const print_handle *>(&print)),
        slic3r_api::Object(reinterpret_cast<const object_handle *>(&object)),
        slic3r_api::ExPolygonCollection(reinterpret_cast<const expolygon_collection_handle *>(&subject)),
        scale_i(0.2), scale_i(0.6), scale_i(0.5));
    REQUIRE(built.created);
    layer_handle *layer = object_get_auxiliary_layer(reinterpret_cast<const object_handle *>(&object), 0);
    REQUIRE(layer_get_print_z(layer) - layer_get_height(layer) > 0);
    REQUIRE(plugin_property_get_or_add_data_mutable(
        reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()), layer_get_properties_mutable(layer),
        PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT) != nullptr);
    layer_region_island_handle *region = layer_island_get_or_create_region_island(
        layer_get_island_mutable(layer, 0), nullptr, 0, 0);
    REQUIRE(region != nullptr);
    slic3r_api::StoredSurfaceCollection surfaces(reinterpret_cast<storage_handle *>(&storage));
    const raw_surface_type type = RAW_SURFACE_TYPE_MOD_SUPPORT | RAW_SURFACE_TYPE_POS_BOTTOM |
        RAW_SURFACE_TYPE_DENS_SOLID | (contact ? RAW_SURFACE_TYPE_MOD_CONTACT | RAW_SURFACE_TYPE_MOD_BRIDGE : 0);
    surfaces.append(slic3r_api::ExPolygon(reinterpret_cast<const expolygon_handle *>(&subject.front())), type);
    REQUIRE(slic3r_api::LayerRegionIsland(region).set_fill_surfaces(&surfaces));

    SG::toolpaths(print);

    CHECK_FALSE(surface_get_type(layer_region_island_get_fill_surface(region, 0)) & RAW_SURFACE_TYPE_MOD_CONTACT);
    CHECK(bool(surface_get_type(layer_region_island_get_fill_surface(region, 0)) & RAW_SURFACE_TYPE_MOD_BRIDGE) == contact);
    PathCounts counts;
    for (const Layer &support_layer : object.auxiliary_layers())
        for (const LayerSliceIsland &island : support_layer.islands())
            for (const LayerRegionIsland &region_island : island.regions_islands()) {
                REQUIRE(region_island.has_extrusion(LayerRegionIsland::SUPPORT));
                check_paths(region_island.extrusion(LayerRegionIsland::SUPPORT), LayerRegionIsland::SUPPORT, counts, false);
            }
    REQUIRE(counts.base > 0);
    CHECK(counts.interface == 0);
    if (contact)
        CHECK(counts.bridges == counts.base);
    else
        CHECK(counts.bridges == 0);
}

// Synthetic disjoint components exercise fusion without changing their rings or the explicitly supplied bridge state.
TEST_CASE("Legacy contact fusion follows native contact patterns and the three automatic alternatives",
          "[plugins][support-toolpath-legacy][support-contact]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool top = GENERATE(false, true);
    const bool bridge = GENERATE(false, true);
    const bool equal_pattern = GENERATE(false, true);
    const bool loops = GENERATE(false, true);
    const int interfaces = GENERATE(0, 3);
    DynamicPrintConfig config = SG::config("grid", false, interfaces, false);
    config.set_deserialize_strict({{"support_material_top_interface_pattern", equal_pattern ? "rectilinear" : "concentric"},
                                   {"support_material_bottom_interface_pattern", equal_pattern ? "rectilinear" : "concentric"},
                                   {"support_material_interface_contact_loops", loops ? "1" : "0"}});
    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
    SG::prefix(print);
    PrintObject &object = print.object(0);
    const slic3r_api::Support::Common::SupportParams geometry = SG::params(print, object);
    const double width = unscaled(geometry.interface_flow.width);
    const std::vector<double> widths{4., 4., width * 0.99, width * 1.01, width * 0.5};
    ExPolygons areas;
    for (size_t index = 0; index < widths.size(); ++index) {
        const double x = index == 4 ? 10. : double(index) * 6.;
        areas.emplace_back(Polygon({Point(scale_i(x), 0), Point(scale_i(x + widths[index]), 0),
            Point(scale_i(x + widths[index]), scale_i(4.)), Point(scale_i(x), scale_i(4.))}));
    }
    areas.front().holes.emplace_back(Polygon({Point(scale_i(1.), scale_i(1.)), Point(scale_i(1.), scale_i(2.)),
        Point(scale_i(2.), scale_i(2.)), Point(scale_i(2.), scale_i(1.))}));
    const ExPolygons subject{ExPolygon(Polygon({Point(0, 0), Point(scale_i(30.), 0),
        Point(scale_i(30.), scale_i(4.)), Point(0, scale_i(4.))}))};
    PluginStorage storage;
    storage_handle *store = reinterpret_cast<storage_handle *>(&storage);
    const slic3r_api::AuxiliaryLayerBuildResult built = slic3r_api::build_auxiliary_layer_regions_from_subject(
        store, slic3r_api::Print(reinterpret_cast<const print_handle *>(&print)),
        slic3r_api::Object(reinterpret_cast<const object_handle *>(&object)),
        slic3r_api::ExPolygonCollection(reinterpret_cast<const expolygon_collection_handle *>(&subject)),
        scale_i(0.2), scale_i(0.6), scale_i(0.5));
    REQUIRE(built.created);
    layer_handle *layer = object_get_auxiliary_layer(reinterpret_cast<const object_handle *>(&object), 0);
    REQUIRE(plugin_property_get_or_add_data_mutable(reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()),
        layer_get_properties_mutable(layer), PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT) != nullptr);
    layer_region_island_handle *region = layer_island_get_or_create_region_island(layer_get_island_mutable(layer, 0), nullptr, 0, 0);
    REQUIRE(region != nullptr);
    slic3r_api::StoredSurfaceCollection surfaces(store);
    const slic3r_api::PluginPropertyKey<int32_t> marker = slic3r_api::PluginPropertyKey<int32_t>::register_dynamic(
        reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()), "tests.support.contact.marker");
    const raw_surface_type side = RAW_SURFACE_TYPE_MOD_SUPPORT | RAW_SURFACE_TYPE_DENS_SOLID |
        (top ? RAW_SURFACE_TYPE_POS_TOP : RAW_SURFACE_TYPE_POS_BOTTOM) | (bridge ? RAW_SURFACE_TYPE_MOD_BRIDGE : 0);
    for (size_t index = 0; index < areas.size(); ++index) {
        surfaces.append(slic3r_api::ExPolygon(reinterpret_cast<const expolygon_handle *>(&areas[index])),
            side | (index == 1 ? 0 : RAW_SURFACE_TYPE_MOD_CONTACT));
        surfaces.back_mutable().get_or_add(marker) = int32_t(index + 11);
    }
    REQUIRE(slic3r_api::LayerRegionIsland(region).set_fill_surfaces(&surfaces));
    SG::toolpaths(print);
    REQUIRE(layer_region_island_count_fill_surface(region) == areas.size());
    for (size_t index = 0; index < areas.size(); ++index) {
        CAPTURE(top, bridge, equal_pattern, loops, interfaces, index);
        const surface_handle *surface = layer_region_island_get_fill_surface(region, uint32_t(index));
        const bool retained = index != 1 && index != 2 && index != 4 && interfaces != 0 && (!equal_pattern || (top && loops));
        CHECK(surface_get_type(surface) == (side | (retained ? RAW_SURFACE_TYPE_MOD_CONTACT : 0)));
        REQUIRE(slic3r_api::Surface(surface).get(marker) != nullptr);
        CHECK(*slic3r_api::Surface(surface).get(marker) == int32_t(index + 11));
        const ExPolygon &actual = *reinterpret_cast<const ExPolygon *>(surface_get_expolygon(surface));
        CHECK(actual.contour.points == areas[index].contour.points);
        CHECK(actual.holes == areas[index].holes);
    }
    PathCounts counts;
    Polygons covered;
    for (const LayerSliceIsland &island : object.auxiliary_layers().front().islands())
        for (const LayerRegionIsland &ri : island.regions_islands())
            for (const ExtrusionRole bucket : {LayerRegionIsland::SUPPORT, LayerRegionIsland::SUPPORT_INTERFACE})
                if (ri.has_extrusion(bucket)) {
                    check_paths(ri.extrusion(bucket), bucket, counts, false);
                    ri.extrusion(bucket).polygons_covered_by_width(covered, 0.f);
                }
    for (const ExPolygon &component : areas)
        CHECK(area(intersection_ex(covered, ExPolygons{component})) > 0);
    CHECK(counts.base + counts.interface > 0);
    CHECK((interfaces == 0 ? counts.interface : counts.base) == 0);
    CHECK(counts.bridges == (bridge ? counts.base + counts.interface : 0));
}

// Disable one algorithm at a time: retained wide contacts and unmarked interfaces cannot be printed by the other role.
TEST_CASE("Each contact and interface role prints only its own surfaces", "[plugins][support-toolpath-legacy][support-contact]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool top = GENERATE(false, true);
    const bool contact_only = GENERATE(false, true);
    const bool bridge = GENERATE(false, true);
    const ScopedActivePlugins active;
    Orchestrator &host = Orchestrator::instance();
    REQUIRE(host.set_plugin_active(top ? (contact_only ? "support.toolpath.legacy_top" : "support.toolpath.legacy_top_contact") :
        (contact_only ? "support.toolpath.legacy_bottom" : "support.toolpath.legacy_bottom_contact"), false));
    DynamicPrintConfig config = SG::config("grid", !bridge, 3, false);
    config.set_deserialize_strict({{"support_material_top_interface_pattern", "concentric"},
                                   {"support_material_bottom_interface_pattern", "concentric"}});
    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
    SG::prefix(print);
    const ExPolygons areas{ExPolygon(Polygon({Point(0, 0), Point(scale_i(4.), 0), Point(scale_i(4.), scale_i(4.)), Point(0, scale_i(4.))})),
        ExPolygon(Polygon({Point(scale_i(6.), 0), Point(scale_i(10.), 0), Point(scale_i(10.), scale_i(4.)), Point(scale_i(6.), scale_i(4.))}))};
    const raw_surface_type side = RAW_SURFACE_TYPE_MOD_SUPPORT | RAW_SURFACE_TYPE_DENS_SOLID |
        (top ? RAW_SURFACE_TYPE_POS_TOP : RAW_SURFACE_TYPE_POS_BOTTOM) | (bridge ? RAW_SURFACE_TYPE_MOD_BRIDGE : 0);
    PluginStorage storage;
    layer_region_island_handle *region = publish_test_surfaces(print, storage, areas, {raw_surface_type(side | RAW_SURFACE_TYPE_MOD_CONTACT), side});
    SG::toolpaths(print);
    CHECK(surface_get_type(layer_region_island_get_fill_surface(region, 0)) == (side | RAW_SURFACE_TYPE_MOD_CONTACT));
    CHECK(surface_get_type(layer_region_island_get_fill_surface(region, 1)) == side);
    size_t count = 0;
    for (const LayerSliceIsland &island : print.object(0).auxiliary_layers().front().islands())
        for (const LayerRegionIsland &ri : island.regions_islands()) {
            if (!ri.has_extrusion(LayerRegionIsland::SUPPORT_INTERFACE)) continue;
            std::vector<const ExtrusionEntity *> leaves;
            collect_leaves(ri.extrusion(LayerRegionIsland::SUPPORT_INTERFACE), leaves);
            for (const ExtrusionEntity *leaf : leaves) {
                ++count;
                for (const ArcPolyline &line : leaf->as_polylines())
                    for (size_t index = 0; index < line.size(); ++index)
                        CHECK((contact_only ? line.get_point(index).x() < scale_i(4.1) : line.get_point(index).x() > scale_i(5.9)));
            }
        }
    CHECK(count > 0);
}

// The host accepts either priority order; a late contact returns its area only after the interface has passed it.
TEST_CASE("A contact provider running after its interface leaves handed-back areas unprinted",
          "[plugins][support-toolpath][support-contact]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    namespace TP = slic3r_api::Support::Toolpath;
    Orchestrator &host = Orchestrator::instance();
    orchestrator_handle *orchestrator = reinterpret_cast<orchestrator_handle *>(&host);
    static TP::SupportToolpathLegacy late(orchestrator, TP::SurfaceRole::TopContact,
        "test.support.contact.late", "test.support.contact.role.late", 210);
    static TP::SupportToolpathLegacy early(orchestrator, TP::SurfaceRole::TopContact,
        "test.support.contact.early", "test.support.contact.role.early", 190);
    static TP::SupportToolpathLegacy interface_provider(orchestrator, TP::SurfaceRole::TopInterface,
        "test.support.contact.interface", "test.support.contact.role.interface", 200);
    if (!host.get_plugin("test.support.contact.late")) {
        orchestrator_register_plugin(orchestrator, late.c_instance());
        orchestrator_register_plugin(orchestrator, early.c_instance());
        orchestrator_register_plugin(orchestrator, interface_provider.c_instance());
        host.set_plugin_active("test.support.contact.late", false);
        host.set_plugin_active("test.support.contact.early", false);
        host.set_plugin_active("test.support.contact.interface", false);
    }
    const ScopedActivePlugins active;
    const bool correct_order = GENERATE(false, true);
    REQUIRE(host.set_plugin_active("support.toolpath.legacy_top_contact", false));
    REQUIRE(host.set_plugin_active("support.toolpath.legacy_top", false));
    REQUIRE(host.set_plugin_active("test.support.contact.late", !correct_order));
    REQUIRE(host.set_plugin_active("test.support.contact.early", correct_order));
    REQUIRE(host.set_plugin_active("test.support.contact.interface", true));
    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, SG::config());
    SG::prefix(print);
    const ExPolygons areas{ExPolygon(Polygon({Point(0, 0), Point(scale_i(4.), 0), Point(scale_i(4.), scale_i(4.)), Point(0, scale_i(4.))}))};
    PluginStorage storage;
    layer_region_island_handle *region = publish_test_surfaces(print, storage, areas,
        {RAW_SURFACE_TYPE_MOD_SUPPORT | RAW_SURFACE_TYPE_DENS_SOLID | RAW_SURFACE_TYPE_POS_TOP | RAW_SURFACE_TYPE_MOD_CONTACT});
    SG::toolpaths(print);
    CHECK_FALSE(surface_get_type(layer_region_island_get_fill_surface(region, 0)) & RAW_SURFACE_TYPE_MOD_CONTACT);
    size_t count = 0;
    for (const LayerSliceIsland &island : print.object(0).auxiliary_layers().front().islands())
        for (const LayerRegionIsland &ri : island.regions_islands())
            if (ri.has_extrusion(LayerRegionIsland::SUPPORT_INTERFACE)) {
                std::vector<const ExtrusionEntity *> leaves;
                collect_leaves(ri.extrusion(LayerRegionIsland::SUPPORT_INTERFACE), leaves);
                count += leaves.size();
            }
    CHECK((count > 0) == correct_order);
}

// Lower sparse and bottom footprints cover both dense roles, but only the first bottom contribution includes them in its flow modulation.
TEST_CASE("Only the first bottom contribution modulates against lower sparse and bottom footprints",
          "[plugins][support-toolpath-legacy][support-contact]")
{
    const bool handed_back = GENERATE(false, true);
    SG::PathMeasure reference[2];
    for (const bool lower_footprints : {false, true}) {
        Print print;
        Model model;
        DynamicPrintConfig config = SG::config("grid", true, 3, false);
        config.set_deserialize_strict("support_material_bottom_interface_pattern", handed_back ? "rectilinear" : "concentric");
        Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
        SG::prefix(print);
        PluginStorage storage;
        const raw_surface_type bottom = RAW_SURFACE_TYPE_MOD_SUPPORT | RAW_SURFACE_TYPE_POS_BOTTOM | RAW_SURFACE_TYPE_DENS_SOLID;
        const ExPolygons target{ExPolygon(Polygon({Point(0, 0), Point(scale_i(4.), 0),
            Point(scale_i(4.), scale_i(4.)), Point(0, scale_i(4.))})),
            ExPolygon(Polygon({Point(scale_i(6.), 0), Point(scale_i(10.), 0),
            Point(scale_i(10.), scale_i(4.)), Point(scale_i(6.), scale_i(4.))}))};
        if (lower_footprints) {
            // The two disjoint lower types each cover half of both target components; neither type alone can thin every target path.
            const ExPolygons lower{ExPolygon(Polygon({Point(0, 0), Point(scale_i(10.), 0),
                Point(scale_i(10.), scale_i(2.)), Point(0, scale_i(2.))})),
                ExPolygon(Polygon({Point(0, scale_i(2.)), Point(scale_i(10.), scale_i(2.)),
                Point(scale_i(10.), scale_i(4.)), Point(0, scale_i(4.))}))};
            publish_test_surfaces(print, storage, lower,
                {bottom, RAW_SURFACE_TYPE_MOD_SUPPORT | RAW_SURFACE_TYPE_POS_INTERNAL | RAW_SURFACE_TYPE_DENS_SPARSE},
                scale_i(0.5), scale_i(0.1));
            for (const ExPolygon &footprint : lower)
                for (const ExPolygon &component : target)
                    REQUIRE(area(intersection_ex(ExPolygons{footprint}, ExPolygons{component})) > 0);
        }
        layer_region_island_handle *region = publish_test_surfaces(print, storage, target,
            {raw_surface_type(bottom | RAW_SURFACE_TYPE_MOD_CONTACT), bottom});
        SG::toolpaths(print);
        CHECK(surface_get_type(layer_region_island_get_fill_surface(region, 0)) ==
            (bottom | (handed_back ? 0 : RAW_SURFACE_TYPE_MOD_CONTACT)));
        CHECK(surface_get_type(layer_region_island_get_fill_surface(region, 1)) == bottom);
        SG::PathMeasure measured[2];
        const Layer &layer = print.object(0).auxiliary_layers().back();
        REQUIRE(layer.scaled_print_z() == scale_i(0.6));
        REQUIRE(layer.scaled_height() == scale_i(0.2));
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &ri : island.regions_islands()) {
                REQUIRE_FALSE(ri.has_extrusion(LayerRegionIsland::SUPPORT));
                if (!ri.has_extrusion(LayerRegionIsland::SUPPORT_INTERFACE)) continue;
                std::vector<const ExtrusionEntity *> leaves;
                collect_leaves(ri.extrusion(LayerRegionIsland::SUPPORT_INTERFACE), leaves);
                for (const ExtrusionEntity *leaf : leaves) {
                    const ExtrusionAttributes *attributes = leaf->get_property<ExtrusionAttributes>();
                    REQUIRE(attributes != nullptr);
                    REQUIRE(leaf->has_polyline());
                    const size_t side = leaf->polyline_ref().front().x() < scale_i(5.) ? 0 : 1;
                    const bool thinned = lower_footprints && (side == 0 || handed_back);
                    CAPTURE(handed_back, lower_footprints, side);
                    CHECK(attributes->height == Approx(thinned ? 0.1 : 0.2));
                    CHECK(attributes->role == ExtrusionRole::SupportMaterialInterface);
                    for (const ArcPolyline &line : leaf->as_polylines())
                        for (size_t index = 0; index < line.size(); ++index)
                            CHECK((side == 0 ? line.get_point(index).x() < scale_i(4.1) : line.get_point(index).x() > scale_i(5.9)));
                    SG::PathMeasure &measure = measured[side];
                    const double length = unscaled(leaf->polyline_ref().length());
                    REQUIRE(length > 0);
                    ++measure.count;
                    measure.length += length;
                    measure.volume += length * attributes->mm3_per_mm;
                }
            }
        for (size_t side = 0; side < 2; ++side) {
            REQUIRE(measured[side].count > 0);
            if (!lower_footprints) reference[side] = measured[side];
            else {
                const double ratio = side == 0 || handed_back ? 0.5 : 1.;
                CHECK(measured[side].count == reference[side].count);
                CHECK(measured[side].length == Approx(reference[side].length));
                CHECK(measured[side].volume == Approx(reference[side].volume * ratio));
            }
        }
    }
}

// Default contacts use the interface reference paths without loops; active loops keep wide contacts and print removable outlines.
TEST_CASE("Default contacts are handed back to the interface role", "[plugins][support-toolpath-legacy][support-contact]")
{
    const bool loops = GENERATE(false, true);
    SG::PathMeasures reference;
    for (const bool contact : {false, true}) {
        Print print;
        Model model;
        DynamicPrintConfig config = SG::config();
        config.set_deserialize_strict({{"support_material_interface_contact_loops", loops ? "1" : "0"},
                                       {"support_material_top_interface_pattern", "rectilinear"}});
        Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
        SG::prefix(print);
        const ExPolygons areas{ExPolygon(Polygon({Point(0, 0), Point(scale_i(4.), 0), Point(scale_i(4.), scale_i(4.)), Point(0, scale_i(4.))})),
            ExPolygon(Polygon({Point(scale_i(6.), 0), Point(scale_i(10.), 0), Point(scale_i(10.), scale_i(4.)), Point(scale_i(6.), scale_i(4.))}))};
        const raw_surface_type side = RAW_SURFACE_TYPE_MOD_SUPPORT | RAW_SURFACE_TYPE_DENS_SOLID | RAW_SURFACE_TYPE_POS_TOP;
        PluginStorage storage;
        layer_region_island_handle *region = publish_test_surfaces(print, storage, areas,
            {raw_surface_type(side | (contact ? RAW_SURFACE_TYPE_MOD_CONTACT : 0)), side});
        // Supply demand at the matching object layer so retained top contacts actually print loops, not just keep their marking.
        run_ctx_support_demand payload{SG::print_view(print).handle(), SG::object_view(print.object(0)).handle()};
        plugin_run_context context = {};
        context.step = STEP_SUPPORT_DEMAND;
        context.data = &payload;
        context.plugin_storage = SG::handle(storage);
        for (const Layer &object_layer : print.object(0).layers())
            for (const LayerSliceIsland &island : object_layer.islands()) {
                expolygon_collection_handle *demand = support_demand_get_island_mutable(&context,
                    reinterpret_cast<const layer_island_handle *>(&island));
                REQUIRE(demand != nullptr);
                expolygons_copy(demand, reinterpret_cast<const expolygon_collection_handle *>(&areas));
            }
        SG::toolpaths(print);
        CHECK(surface_get_type(layer_region_island_get_fill_surface(region, 0)) ==
            (side | (contact && loops ? RAW_SURFACE_TYPE_MOD_CONTACT : 0)));
        CHECK(surface_get_type(layer_region_island_get_fill_surface(region, 1)) == side);
        const SG::PathMeasures measured = SG::measure_layer(print.object(0).auxiliary_layers().front());
        REQUIRE_FALSE(measured.empty());
        PathCounts counts;
        for (const LayerSliceIsland &island : print.object(0).auxiliary_layers().front().islands())
            for (const LayerRegionIsland &ri : island.regions_islands())
                if (ri.has_extrusion(LayerRegionIsland::SUPPORT_INTERFACE))
                    check_paths(ri.extrusion(LayerRegionIsland::SUPPORT_INTERFACE), LayerRegionIsland::SUPPORT_INTERFACE, counts, false);
        CHECK(counts.interface > 0);
        if (!loops || !contact) CHECK(counts.loops == 0);
        if (!contact) reference = measured;
        else {
            REQUIRE(measured.size() == reference.size());
            for (const auto &[key, measure] : measured) {
                REQUIRE(reference.count(key) == 1);
                if (loops) {
                    CHECK(measure.count != reference.at(key).count);
                    CHECK(std::abs(measure.length - reference.at(key).length) > 1e-6);
                } else {
                    CHECK(measure.count == reference.at(key).count);
                    CHECK(measure.length == Approx(reference.at(key).length));
                    CHECK(measure.volume == Approx(reference.at(key).volume));
                }
            }
        }
    }
}

// Contact marking remains independent of interface counts, and never synthesizes a bridge modifier.
TEST_CASE("Dense contact classification remains independent of interface-layer settings",
          "[plugins][support-toolpath-legacy][support-contact]")
{
    const int interfaces = GENERATE(0, 3);
    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, SG::config("grid", false, interfaces, false));
    SG::prefix(print);
    const ExPolygons areas{ExPolygon(Polygon({Point(0, 0), Point(scale_i(4.), 0), Point(scale_i(4.), scale_i(4.)), Point(0, scale_i(4.))})),
        ExPolygon(Polygon({Point(scale_i(6.), 0), Point(scale_i(10.), 0), Point(scale_i(10.), scale_i(4.)), Point(scale_i(6.), scale_i(4.))}))};
    const raw_surface_type side = RAW_SURFACE_TYPE_MOD_SUPPORT | RAW_SURFACE_TYPE_DENS_SOLID | RAW_SURFACE_TYPE_POS_BOTTOM;
    PluginStorage storage;
    layer_region_island_handle *region = publish_test_surfaces(print, storage, areas, {raw_surface_type(side | RAW_SURFACE_TYPE_MOD_CONTACT), side});
    CHECK(surface_get_type(layer_region_island_get_fill_surface(region, 0)) == (side | RAW_SURFACE_TYPE_MOD_CONTACT));
    CHECK(surface_get_type(layer_region_island_get_fill_surface(region, 1)) == side);
    SG::toolpaths(print);
    CHECK(surface_get_type(layer_region_island_get_fill_surface(region, 0)) == side);
    CHECK(surface_get_type(layer_region_island_get_fill_surface(region, 1)) == side);
    size_t paths_in_area[2] = {};
    PathCounts counts;
    for (const LayerSliceIsland &island : print.object(0).auxiliary_layers().front().islands())
        for (const LayerRegionIsland &ri : island.regions_islands())
            for (const ExtrusionRole bucket : {LayerRegionIsland::SUPPORT, LayerRegionIsland::SUPPORT_INTERFACE}) {
                if (!ri.has_extrusion(bucket)) continue;
                check_paths(ri.extrusion(bucket), bucket, counts, false);
                std::vector<const ExtrusionEntity *> leaves;
                collect_leaves(ri.extrusion(bucket), leaves);
                for (const ExtrusionEntity *leaf : leaves)
                    for (const ArcPolyline &line : leaf->as_polylines())
                        ++paths_in_area[line.front().x() < scale_i(5.) ? 0 : 1];
            }
    CHECK(paths_in_area[0] > 0);
    CHECK(paths_in_area[1] > 0);
    CHECK(counts.bridges == 0);
    CHECK((interfaces == 0 ? counts.interface : counts.base) == 0);
}

// A changed contact collection must not erase paths already stored in that region's support bucket.
TEST_CASE("Contact hand-back preserves surface properties and earlier extrusions",
          "[plugins][support-toolpath-legacy][support-contact]")
{
    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, SG::config());
    SG::prefix(print);
    ExPolygons areas;
    for (size_t index = 0; index < 5; ++index) {
        const double x = double(index) * 6.;
        areas.emplace_back(Polygon({Point(scale_i(x), 0), Point(scale_i(x + 4.), 0),
            Point(scale_i(x + 4.), scale_i(4.)), Point(scale_i(x), scale_i(4.))}));
    }
    const raw_surface_type dense = RAW_SURFACE_TYPE_MOD_SUPPORT | RAW_SURFACE_TYPE_DENS_SOLID;
    const std::vector<raw_surface_type> types{
        RAW_SURFACE_TYPE_MOD_SUPPORT | RAW_SURFACE_TYPE_POS_INTERNAL | RAW_SURFACE_TYPE_DENS_SPARSE,
        raw_surface_type(dense | RAW_SURFACE_TYPE_POS_BOTTOM | RAW_SURFACE_TYPE_MOD_CONTACT | RAW_SURFACE_TYPE_MOD_BRIDGE),
        raw_surface_type(dense | RAW_SURFACE_TYPE_POS_BOTTOM),
        raw_surface_type(dense | RAW_SURFACE_TYPE_POS_TOP | RAW_SURFACE_TYPE_MOD_CONTACT),
        raw_surface_type(dense | RAW_SURFACE_TYPE_POS_TOP)};
    PluginStorage storage;
    layer_region_island_handle *region = publish_test_surfaces(print, storage, areas, types);
    const std::vector<c_point> points{{scale_i(1.), scale_i(1.)}, {scale_i(2.), scale_i(1.)}};
    slic3r_api::StoredExtrusionEntity earlier(SG::handle(storage), points);
    extrusion_entity_handle *root = layer_region_island_get_mutable_extrusion(region, RAW_EXTRUSION_ROLE_SUPPORT);
    REQUIRE(root != nullptr);
    REQUIRE(extrusion_insert_child_move(root, 0, earlier.mutable_handle()) != EXTRUSION_INDEX_INVALID);
    SG::toolpaths(print);
    REQUIRE(extrusion_child_count(root) > 1);
    const slic3r_api::ExtrusionEntity kept(extrusion_child(root, 0));
    const std::vector<c_point> actual = kept.points();
    REQUIRE(actual.size() == points.size());
    for (size_t index = 0; index < points.size(); ++index) {
        CHECK(actual[index].x == points[index].x);
        CHECK(actual[index].y == points[index].y);
    }
    REQUIRE(layer_region_island_count_fill_surface(region) == types.size());
    for (size_t index = 0; index < types.size(); ++index) {
        const surface_handle *surface = layer_region_island_get_fill_surface(region, uint32_t(index));
        CHECK(surface_get_type(surface) == (types[index] & ~RAW_SURFACE_TYPE_MOD_CONTACT));
        const ExPolygon &actual_area = *reinterpret_cast<const ExPolygon *>(surface_get_expolygon(surface));
        CHECK(actual_area.contour.points == areas[index].contour.points);
        CHECK(actual_area.holes == areas[index].holes);
    }
    size_t interface_leaves = 0;
    for (const LayerSliceIsland &island : print.object(0).auxiliary_layers().front().islands())
        for (const LayerRegionIsland &ri : island.regions_islands())
            if (ri.has_extrusion(LayerRegionIsland::SUPPORT_INTERFACE)) {
                std::vector<const ExtrusionEntity *> leaves;
                collect_leaves(ri.extrusion(LayerRegionIsland::SUPPORT_INTERFACE), leaves);
                interface_leaves += leaves.size();
            }
    CHECK(interface_leaves > 0);
}

// Compare every ordered published ring after dispatch, allowing only the contact bit selected by the native fusion rule to change.
TEST_CASE("Published contacts follow the hand-back rules on a real print", "[plugins][support-toolpath-legacy][support-contact]")
{
    const bool soluble = GENERATE(false, true);
    const int interfaces = GENERATE(0, 3);
    const bool equal_pattern = GENERATE(false, true);
    const bool loops = GENERATE(false, true);
    Print print;
    Model model;
    DynamicPrintConfig config = SG::config("grid", soluble, interfaces);
    config.set_deserialize_strict({{"support_material_top_interface_pattern", equal_pattern ? "rectilinear" : "concentric"},
                                   {"support_material_bottom_interface_pattern", equal_pattern ? "rectilinear" : "concentric"},
                                   {"support_material_interface_contact_loops", loops ? "1" : "0"}});
    Test::init_print({SG::slab(true)}, print, model, config);
    SG::prefix(print);
    SG::family(print);
    const slic3r_api::Support::Common::SupportParams geometry = SG::params(print, print.object(0));
    std::vector<Surface> before;
    size_t contacts[2] = {};
    for (const Layer &layer : print.object(0).auxiliary_layers())
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands())
                for (const Surface &surface : region.fill_surfaces()) {
                    before.push_back(surface);
                    if (surface.has_mod_contact()) ++contacts[surface.has_pos_top() ? 1 : 0];
                }
    REQUIRE(contacts[1] > 0);
    // A mergeable contact on a side without interfaces can already be published as sparse base.
    if (interfaces != 0 || !soluble) REQUIRE(contacts[0] > 0);
    SG::toolpaths(print);
    size_t index = 0;
    for (const Layer &layer : print.object(0).auxiliary_layers())
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands())
                for (const Surface &surface : region.fill_surfaces()) {
                    REQUIRE(index < before.size());
                    const Surface &source = before[index++];
                    CHECK(surface.expolygon.contour.points == source.expolygon.contour.points);
                    CHECK(surface.expolygon.holes == source.expolygon.holes);
                    const bool as_base = source.has_pos_top() ? !geometry.top_published_interfaces_enabled : !geometry.bottom_published_interfaces_enabled;
                    const bool thin = offset_ex(source.expolygon, -double(geometry.interface_flow.width) / 2.).empty();
                    const bool active_loops = source.has_pos_top() && geometry.top_published_interfaces_enabled && loops;
                    const bool returned = source.has_mod_contact() && (as_base || (equal_pattern && !active_loops) || thin);
                    CHECK(surface.surface_type == (returned ? SG::without_contact(source.surface_type) : source.surface_type));
                }
    CHECK(index == before.size());
}

// A changed contact setting invalidates support toolpaths; rebuilding must agree with a print that starts with that setting.
TEST_CASE("Contact hand-back replay agrees with a fresh print", "[plugins][support-toolpath-legacy][support-contact]")
{
    DynamicPrintConfig config = SG::config();
    Print replay;
    Model replay_model;
    Test::init_print({SG::slab(true)}, replay, replay_model, config);
    replay.process();
    config.set_deserialize_strict("support_material_top_interface_pattern", "concentric");
    replay.apply(replay_model, config);
    replay.process();
    Print fresh;
    Model fresh_model;
    Test::init_print({SG::slab(true)}, fresh, fresh_model, config);
    fresh.process();
    REQUIRE(replay.object(0).auxiliary_layers().size() == fresh.object(0).auxiliary_layers().size());
    size_t index = 0;
    for (const Layer &layer : replay.object(0).auxiliary_layers()) {
        const Layer &expected = fresh.object(0).auxiliary_layers()[index++];
        CHECK(layer.scaled_print_z() == expected.scaled_print_z());
        const SG::LayerSurfaceAreas actual_areas = SG::layer_surface_areas(layer);
        const SG::LayerSurfaceAreas expected_areas = SG::layer_surface_areas(expected);
        CHECK(actual_areas.top_contact == Approx(expected_areas.top_contact));
        CHECK(actual_areas.top_interface == Approx(expected_areas.top_interface));
        const SG::PathMeasures actual = SG::measure_layer(layer);
        const SG::PathMeasures wanted = SG::measure_layer(expected);
        REQUIRE(actual.size() == wanted.size());
        for (const auto &[key, measure] : actual) {
            REQUIRE(wanted.count(key) == 1);
            CHECK(measure.count == wanted.at(key).count);
            CHECK(measure.length == Approx(wanted.at(key).length));
            CHECK(measure.volume == Approx(wanted.at(key).volume));
        }
    }
}

// Check that the built-in support toolpaths (not the native generator) print the interface lines of a support layer at the angle
// interface_angle + interface_id * increment, for every style. Lines at 0 degrees run along X and lines at 90 degrees along Y: the
// base settings test below prints its contacts at the base angle 0 along X and fails on any vertical path. The first layer is left out:
// its flange uses its own angle.
TEST_CASE("Built-in support toolpaths turn interfaces by the angle increment per interface id", "[plugins][support-toolpath-legacy][support-toolpath]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    // Grid and snug with an increment of 90, then snug with no increment.
    const int case_index = GENERATE(0, 1, 2);
    const char *style = case_index == 0 ? "grid" : "snug";
    const bool no_increment = case_index == 2;
    DynamicPrintConfig config = SG::config(style, false, 3);
    config.set_deserialize_strict({{"support_material_interface_angle", "90"},
                                   {"support_material_interface_angle_increment", no_increment ? "0" : "90"}});
    Print print;
    Model model;
    Test::init_print({SG::slab(true)}, print, model, config);
    SG::prefix(print);
    SG::family(print);
    SG::toolpaths(print);
    // Count of the layers seen for each parity of the interface id.
    size_t layers_of_parity[2] = {0, 0};
    for (const Layer &layer : print.object(0).auxiliary_layers()) {
        if (layer.scaled_print_z() - layer.scaled_height() <= 0) continue;
        const LayerSupportProperty *property = layer.get_property<LayerSupportProperty>();
        REQUIRE(property != nullptr);
        double length_along[2] = {0, 0};
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands()) {
                if (!region.has_extrusion(LayerRegionIsland::SUPPORT_INTERFACE)) continue;
                std::vector<const ExtrusionEntity *> leaves;
                collect_leaves(region.extrusion(LayerRegionIsland::SUPPORT_INTERFACE), leaves);
                for (const ExtrusionEntity *leaf : leaves)
                    for (const ArcPolyline &line : leaf->as_polylines())
                        for (size_t index = 1; index < line.size(); ++index) {
                            const Point delta = line.get_point(index) - line.get_point(index - 1);
                            length_along[0] += unscaled(double(std::abs(delta.x())));
                            length_along[1] += unscaled(double(std::abs(delta.y())));
                        }
            }
        const double total = length_along[0] + length_along[1];
        if (total == 0) continue;
        // With an increment of 90 an even id prints at 90 or 270 degrees (along Y) and an odd id at 180 or 360 degrees (along X);
        // with no increment every id prints at 90 degrees (along Y). The links between neighbouring lines are short:
        // the lines own nearly all the length.
        const size_t parity = property->interface_id & 1;
        const size_t expected_axis = no_increment || parity == 0 ? 1 : 0;
        ++layers_of_parity[parity];
        CHECK(length_along[expected_axis] > 0.9 * total);
    }
    if (no_increment) {
        REQUIRE(layers_of_parity[0] + layers_of_parity[1] > 0);
    } else {
        REQUIRE(layers_of_parity[0] > 0);
        REQUIRE(layers_of_parity[1] > 0);
    }
}

// Check registration, activation and dispatch ordering of sparse, branch, bottom, top and residual gap-fill providers.
TEST_CASE("Contact groups retain their declaring provider and elect complete selector descriptions",
          "[plugins][support-toolpath-legacy][support-contact]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    namespace Common = slic3r_api::Support::Common;
    Orchestrator &host = Orchestrator::instance();
    orchestrator_handle *orchestrator = reinterpret_cast<orchestrator_handle *>(&host);
    static ContactGroupProbe bottom_b(orchestrator, "test.support.contact.group.bottom.b", Common::k_bottom_contact_role, 90);
    static ContactGroupProbe bottom_a(orchestrator, "test.support.contact.group.bottom.a", Common::k_bottom_contact_role, 90);
    static ContactGroupProbe top_b(orchestrator, "test.support.contact.group.top.b", Common::k_top_contact_role, 190);
    static ContactGroupProbe top_a(orchestrator, "test.support.contact.group.top.a", Common::k_top_contact_role, 190);
    const bool top = GENERATE(false, true);
    const char *group = top ? Common::k_top_contact_role : Common::k_bottom_contact_role;
    const char *native_id = top ? Common::k_legacy_top_contact_id : Common::k_legacy_bottom_contact_id;
    const std::string native_label = top ? "Top support contact" : "Bottom support contact";
    const std::string native_tooltip = top ? "Selects the algorithm that prints support contacts above the support." :
        "Selects the algorithm that prints support contacts below the object.";
    const Orchestrator::ExclusiveGroupDefinition *native = host.exclusive_group_definition(STEP_SUPPORT_TOOLPATH, group);
    REQUIRE(native != nullptr);
    CHECK(native->plugin_id == native_id);
    CHECK(native->label == native_label);
    CHECK(native->tooltip == native_tooltip);
    CHECK(native->traits.container_type == RAW_CONTAINER_TYPE_OBJECT);
    CHECK(native->traits.category == RAW_OPTION_CATEGORY_SUPPORT);
    CHECK(native->traits.option_preset_type == RAW_PRESET_TYPE_FFF_PRINT);
    CHECK(native->traits.ui_target_file == "print.ui");
    CHECK(native->traits.ui_page == "Support material");
    CHECK(native->traits.ui_group == "Support toolpaths");
    REQUIRE(native->traits.invalidates_step.has_value());
    CHECK(*native->traits.invalidates_step == STEP_SUPPORT_TOOLPATH);
    CHECK(native->traits.auto_value);
    const std::vector<Common::ToolpathPreference> preferences = Common::legacy_toolpath_preferences(Common::k_legacy_sparse_id);
    REQUIRE(preferences.size() == 1);
    CHECK(std::string(preferences.front().role) == Common::k_sparse_role);
    CHECK(std::string(preferences.front().provider_id) == Common::k_legacy_sparse_id);
    ContactGroupProbe *a = top ? &top_a : &bottom_a;
    ContactGroupProbe *b = top ? &top_b : &bottom_b;
    const std::string a_id = top ? "test.support.contact.group.top.a" : "test.support.contact.group.bottom.a";
    const std::string b_id = top ? "test.support.contact.group.top.b" : "test.support.contact.group.bottom.b";
    if (!host.get_plugin(a_id)) {
        Orchestrator::PluginRegistrationScope scope = host.plugin_registration_scope({}, false);
        for (ContactGroupProbe *probe : {b, a}) {
            orchestrator_register_plugin(orchestrator, probe->c_instance());
            REQUIRE(host.get_plugin(probe == a ? a_id : b_id) != nullptr);
            raw_exclusive_group_def definition = raw_exclusive_group_def_init();
            definition.exclusive_group = group;
            definition.label = probe == a ? "Alternative A contact" : "Alternative B contact";
            definition.tooltip = probe == a ? "Alternative A tooltip." : "Alternative B tooltip.";
            definition.container_type = RAW_CONTAINER_TYPE_OBJECT;
            definition.category = RAW_OPTION_CATEGORY_SUPPORT;
            definition.auto_value = 1;
            definition.has_invalidates_step = 1;
            definition.invalidates_step = STEP_SUPPORT_TOOLPATH;
            definition.ui_page = "Support material";
            definition.ui_group = "Support toolpaths";
            probe->register_exclusive_group(STEP_SUPPORT_TOOLPATH, definition);
            host.set_plugin_active(probe == a ? a_id : b_id, false);
        }
    }
    const ScopedActivePlugins active;
    struct RestoreDefinition {
        Orchestrator &host;
        std::string group, a_id, b_id;
        // Remove probe candidates and reelect native text after assertions, including failures.
        ~RestoreDefinition() {
            host.set_plugin_active(a_id, false);
            host.set_plugin_active(b_id, false);
            host.elect_exclusive_group_definition(STEP_SUPPORT_TOOLPATH, group);
        }
    } restore{host, group, a_id, b_id};
    REQUIRE(host.set_plugin_active(b_id, true));
    host.elect_exclusive_group_definition(STEP_SUPPORT_TOOLPATH, group);
    CHECK(host.exclusive_group_definition(STEP_SUPPORT_TOOLPATH, group)->label == "Alternative B contact");
    REQUIRE(host.set_plugin_active(a_id, true));
    host.elect_exclusive_group_definition(STEP_SUPPORT_TOOLPATH, group);
    const Orchestrator::ExclusiveGroupDefinition elected = *host.exclusive_group_definition(STEP_SUPPORT_TOOLPATH, group);
    CHECK(elected.plugin_id == a_id);
    CHECK(elected.label == "Alternative A contact");
    CHECK(elected.tooltip == "Alternative A tooltip.");
    REQUIRE(host.set_plugin_active(a_id, false));
    CHECK(host.exclusive_group_definition(STEP_SUPPORT_TOOLPATH, group)->plugin_id == a_id);
    CHECK_FALSE(host.is_plugin_active(a_id));
}

// Check registration, activation and dispatch ordering of the production algorithms: sparse, branch, bottom, top.
TEST_CASE("Native support toolpath providers are active in role order", "[plugins][support-toolpath-legacy]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const std::vector<Plugin *> providers = Orchestrator::instance().get_active_plugins_for_step(STEP_SUPPORT_TOOLPATH);
    REQUIRE(providers.size() == 7);
    const char *ids[] = {"support.toolpath.legacy_sparse", "support.toolpath.branch_sheath", "support.toolpath.legacy_bottom_contact",
                         "support.toolpath.legacy_bottom", "support.toolpath.legacy_top_contact", "support.toolpath.legacy_top",
                         "support.toolpath.interface_gap_fill"};
    const int32_t priorities[] = {0, 0, 90, 100, 190, 200, 300};
    CHECK(providers[2]->get_priority() < providers[3]->get_priority());
    CHECK(providers[4]->get_priority() < providers[5]->get_priority());
    for (size_t index = 0; index < 7; ++index) {
        CHECK(providers[index]->get_id() == ids[index]);
        CHECK(providers[index]->get_priority() == priorities[index]);
    }
    std::ifstream stream(std::string(TEST_DATA_DIR) + "/../../src/plugins/default_activated.ini.in");
    REQUIRE(stream.good());
    std::ostringstream contents;
    contents << stream.rdbuf();
    CHECK(contents.str().find("support.toolpath.legacy_bottom_contact = 1") != std::string::npos);
    CHECK(contents.str().find("support.toolpath.legacy_top_contact = 1") != std::string::npos);
    CHECK(contents.str().find("support.toolpath.interface_gap_fill = 1") != std::string::npos);
}

// Run real surfaces through every role and check first-layer and bridge flows on a roof above a platform; a role with no interface layer prints its contact as base, without a bridge flow.
TEST_CASE("Native support paths carry roles buckets and positive flow", "[plugins][support-toolpath-legacy]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool soluble = GENERATE(false, true);
    const int interfaces = GENERATE(0, 3);
    Print print;
    Model model;
    DynamicPrintConfig config = SG::config("grid", soluble, interfaces);
    Test::init_print({SG::slab(true)}, print, model, config);
    SG::prefix(print);
    SG::family(print);
    SG::toolpaths(print);
    PathCounts counts;
    for (const Layer &layer : print.object(0).auxiliary_layers())
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands())
                for (const ExtrusionRole bucket : {LayerRegionIsland::SUPPORT, LayerRegionIsland::SUPPORT_INTERFACE})
                    if (region.has_extrusion(bucket)) check_paths(region.extrusion(bucket), bucket, counts, layer.scaled_print_z() - layer.scaled_height() <= 0);
    CHECK(counts.base > 0);
    if (interfaces > 0) CHECK(counts.interface > 0);
    else CHECK(counts.interface == 0);
    // A non-soluble bottom contact keeps its bridging flow, with or without interface layers.
    if (!soluble) CHECK(counts.bridges > 0);
}

// Compare enabled contact loops with the same roof without loops; path geometry must change without removing either support role.
TEST_CASE("Native top contacts generate removable loops", "[plugins][support-toolpath-legacy]")
{
    double lengths[2] = {};
    for (int enabled = 0; enabled < 2; ++enabled) {
        DynamicPrintConfig config = SG::config();
        config.set_deserialize_strict({{"support_material_interface_contact_loops", enabled ? "1" : "0"},
                                       {"support_material_top_interface_pattern", "concentric"}});
        Print print;
        Model model;
        Test::init_print({SG::slab()}, print, model, config);
        SG::prefix(print);
        SG::family(print);
        SG::toolpaths(print);
        PathCounts counts;
        for (const Layer &layer : print.object(0).auxiliary_layers())
            for (const LayerSliceIsland &island : layer.islands())
                for (const LayerRegionIsland &region : island.regions_islands())
                    if (region.has_extrusion(LayerRegionIsland::SUPPORT_INTERFACE)) {
                        const ExtrusionEntity &paths = region.extrusion(LayerRegionIsland::SUPPORT_INTERFACE);
                        check_paths(paths, LayerRegionIsland::SUPPORT_INTERFACE, counts, false);
                        for (const ArcPolyline &line : paths.as_polylines()) lengths[enabled] += line.length();
                    }
        REQUIRE(counts.interface > 0);
    }
    CHECK(std::abs(lengths[0] - lengths[1]) > scale_i(0.1));
}

// Verify the support flange uses the first-layer flow instead of the ordinary support flow.
TEST_CASE("Native sparse first layer uses the support flange flow", "[plugins][support-toolpath-legacy]")
{
    DynamicPrintConfig config = SG::config();
    config.set_deserialize_strict({{"first_layer_extrusion_width", "0.7"}, {"raft_first_layer_density", "80%"}});
    Print print;
    Model model;
    Test::init_print({SG::slab()}, print, model, config);
    SG::prefix(print);
    SG::family(print);
    SG::toolpaths(print);
    const Layer &first = print.object(0).auxiliary_layers().front();
    PathCounts counts;
    for (const LayerSliceIsland &island : first.islands())
        for (const LayerRegionIsland &region : island.regions_islands())
            if (region.has_extrusion(LayerRegionIsland::SUPPORT)) check_paths(region.extrusion(LayerRegionIsland::SUPPORT), LayerRegionIsland::SUPPORT, counts, true);
    CHECK(counts.base > 0);
    CHECK(counts.loops > 0);
    CHECK(counts.first_width == Approx(0.7));
}

// Run the real legacy sparse flange through pattern_service_fill; the pattern's error must remain the sole diagnostic.
TEST_CASE("Legacy support pattern failure preserves only the pattern error", "[plugins][support-toolpath-legacy]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    namespace Probe = Test::ProbePatternPlugin;
    Orchestrator &host = Orchestrator::instance();
    // Restore probe activation and failure controls even when a required assertion aborts the test.
    struct ResetProbe {
        Orchestrator &host;
        bool active;
        // Clear all shared probe controls before preparation; no probe run is active at test entry or exit.
        explicit ResetProbe(Orchestrator &orchestrator) : host(orchestrator), active(host.is_plugin_active(Probe::probe_pattern_id))
        {
            reset();
        }
        // Reset counters and failure, cancellation and rendezvous controls only while no probe run is active.
        void reset()
        {
            Probe::pattern_setup_count = 0;
            Probe::pattern_active_runs = 0;
            Probe::pattern_max_active_runs = 0;
            Probe::pattern_rendezvous = false;
            Probe::pattern_fail_setup = false;
            Probe::pattern_fail_run = 0;
            Probe::pattern_wrote_before_failure = false;
            Probe::pattern_cancel_print = nullptr;
            Probe::rendezvous_timed_out = false;
            host.reset_plugin_cancel();
            (void) host.consume_plugin_messages();
        }
        // Release the test's failure mode and stop request without retaining any print-owned data.
        ~ResetProbe()
        {
            reset();
            host.set_plugin_active(Probe::probe_pattern_id, active);
        }
    };
    ResetProbe reset{host};
    DynamicPrintConfig config = SG::config();
    Print print;
    Model model;
    Test::init_print({SG::slab()}, print, model, config);
    SG::prefix(print);
    SG::family(print);
    // The configured rectilinear flange resolves to the sole active pattern, with a fallback warning, not an error.
    const ScopedActivePlugins active;
    host.clear_active_plugins();
    for (const char *id : {"support.toolpath.legacy_sparse", "support.toolpath.branch_sheath",
                           "support.toolpath.legacy_bottom", "support.toolpath.legacy_top", "support.toolpath.legacy_bottom_contact",
                           "support.toolpath.legacy_top_contact", Probe::probe_pattern_id})
        REQUIRE(host.set_plugin_active(id, true));
    Probe::pattern_fail_run = 1;
    (void) host.consume_plugin_messages();
    std::string failure;
    try {
        SG::toolpaths(print);
    } catch (const std::exception &error) {
        failure = error.what();
    }
    const std::vector<Orchestrator::PluginMessage> messages = host.consume_plugin_messages();
    CAPTURE(failure);
    CHECK(Probe::pattern_wrote_before_failure);
    CHECK(failure.find("STEP_SUPPORT_TOOLPATH stopped during ") != std::string::npos);
    CHECK(failure.find("of plugin 'support.toolpath.legacy_sparse' after plugin 'test.pattern.probe' reported an error.") != std::string::npos);
    size_t errors = 0;
    for (const Orchestrator::PluginMessage &message : messages) {
        INFO(message.plugin_id << ": " << message.message);
        CHECK(message.message.find("could not fill a perimeter-wrapped area") == std::string::npos);
        if (message.level != Orchestrator::PluginMessageLevel::Error) continue;
        ++errors;
        CHECK(message.plugin_id == Probe::probe_pattern_id);
        CHECK(message.message == "probe pattern run failure");
    }
    CHECK(errors == 1);
}

// Collect the leaves of a plugin-API extrusion tree in traversal order.
void collect_api_leaves(const slic3r_api::ExtrusionEntity &entity, std::vector<slic3r_api::ExtrusionEntity> &leaves)
{
    if (entity.has_polyline()) {
        leaves.push_back(entity);
        return;
    }
    for (uint32_t index = 0; index < entity.child_count(); ++index)
        collect_api_leaves(entity.child(index), leaves);
}

// Length in millimetres of the polyline of one plugin-API leaf.
double api_length(const slic3r_api::ExtrusionEntity &leaf)
{
    const std::vector<c_point> points = leaf.points();
    double length = 0;
    for (size_t index = 1; index < points.size(); ++index)
        length += std::hypot(double(points[index].x - points[index - 1].x), double(points[index].y - points[index - 1].y));
    return unscaled(length);
}

// Drive the modulation helper with synthetic slabs: a path crossing an overlapping footprint is
// split and its flow scaled by the remaining height, and an empty overlap list leaves it untouched.
TEST_CASE("Native support path modulation splits and thins overlapping paths", "[plugins][support-toolpath-legacy]")
{
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    const coord_t layer_z = scale_to_layer_coord(1.);
    const std::vector<c_point> line{c_point{0, 0}, c_point{scale_i(10.), 0}};
    slic3r_api::EPropertyAttributes attributes;
    attributes.extrusion_role(int32_t(RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL)).mm3_per_mm(0.1).width(0.4f).height(0.2f);
    // One slab 0.1 mm below the layer top, covering the middle of the path.
    const std::vector<c_point> square{c_point{scale_i(4.), -scale_i(1.)}, c_point{scale_i(6.), -scale_i(1.)},
                                      c_point{scale_i(6.), scale_i(1.)}, c_point{scale_i(4.), scale_i(1.)}};
    slic3r_api::StoredPolygonCollection footprint(handle);
    slic3r_api::StoredPolygon polygon(handle);
    polygon.insert_array(0, square.data(), uint32_t(square.size()));
    footprint.push_back(polygon);
    std::vector<ToolpathInternal::Overlap> overlapping;
    overlapping.push_back(ToolpathInternal::Overlap{scale_to_layer_coord(0.9), std::move(footprint)});
    slic3r_api::StoredExtrusionEntity split(handle);
    slic3r_api::StoredExtrusionEntity split_path(handle, line);
    split_path.set_flags(uint32_t(RAW_EXTRUSION_FLAG_REVERSIBLE));
    split_path.get_or_add(slic3r_api::EPropertyAttributes::key) = attributes;
    split.append_child_move(split_path.mutable_view());
    ToolpathInternal::modulate(handle, split.mutable_view(), layer_z, overlapping, SCALED_EPSILON);
    // The path is cut at the overlap boundary: one thinned middle path between two unchanged parts.
    std::vector<slic3r_api::ExtrusionEntity> leaves;
    collect_api_leaves(split.readonly(), leaves);
    REQUIRE(leaves.size() == 3);
    double thinned_length = 0, thinned_height = 0, thinned_mm3 = 0;
    std::vector<double> heights;
    for (const slic3r_api::ExtrusionEntity &leaf : leaves) {
        const slic3r_api::EPropertyAttributes *leaf_attributes = leaf.get(slic3r_api::EPropertyAttributes::key);
        REQUIRE(leaf_attributes != nullptr);
        heights.push_back(leaf_attributes->c_extrusion_property_attributes::height);
        CHECK(!leaf.reversible());
        if (leaf_attributes->c_extrusion_property_attributes::height < 0.2f) {
            thinned_length = api_length(leaf);
            thinned_height = leaf_attributes->c_extrusion_property_attributes::height;
            thinned_mm3 = leaf_attributes->mm3_per_mm();
        }
    }
    CHECK(thinned_height == Approx(0.1));
    CHECK(thinned_mm3 == Approx(0.1 * 0.1 / 0.2));
    CHECK(thinned_length == Approx(2.4)); // The 2 mm footprint plus half the flow width on both sides.
    // The fragments form one group in the source order. Like the native multipath, neither the group nor its
    // fragments may be reversed or sorted, so the printing direction is kept; the first child insertion would make
    // the group sortable, which is why the flags are cleared after the children.
    const slic3r_api::ExtrusionEntity modulated = split.readonly().child(0);
    REQUIRE(modulated.child_count() == 3);
    CHECK(!modulated.reversible());
    CHECK(!modulated.sortable());
    CHECK(modulated.continuous());
    REQUIRE(heights.size() == 3);
    CHECK(heights[0] == Approx(0.2));
    CHECK(heights[1] == Approx(0.1));
    CHECK(heights[2] == Approx(0.2));
    // Each fragment starts exactly where the previous one ends; the chain starts at the source start and ends at its end.
    for (size_t index = 0; index + 1 < leaves.size(); ++index) {
        const std::vector<c_point> ending = leaves[index].points();
        const std::vector<c_point> starting = leaves[index + 1].points();
        CHECK(ending.back().x == starting.front().x);
        CHECK(ending.back().y == starting.front().y);
    }
    const std::vector<c_point> first_points = leaves.front().points();
    const std::vector<c_point> last_points = leaves.back().points();
    CHECK(first_points.front().x == line.front().x);
    CHECK(first_points.front().y == line.front().y);
    CHECK(unscaled(std::hypot(double(last_points.back().x - line.back().x), double(last_points.back().y - line.back().y))) < 1e-3);
    // Without overlapping slabs the helper must leave the path untouched.
    slic3r_api::StoredExtrusionEntity untouched(handle);
    slic3r_api::StoredExtrusionEntity untouched_path(handle, line);
    untouched_path.set_flags(uint32_t(RAW_EXTRUSION_FLAG_REVERSIBLE));
    untouched_path.get_or_add(slic3r_api::EPropertyAttributes::key) = attributes;
    untouched.append_child_move(untouched_path.mutable_view());
    ToolpathInternal::modulate(handle, untouched.mutable_view(), layer_z, {}, SCALED_EPSILON);
    leaves.clear();
    collect_api_leaves(untouched.readonly(), leaves);
    REQUIRE(leaves.size() == 1);
    CHECK(leaves.front().reversible());
    const slic3r_api::EPropertyAttributes *leaf_attributes = leaves.front().get(slic3r_api::EPropertyAttributes::key);
    REQUIRE(leaf_attributes != nullptr);
    CHECK(leaf_attributes->c_extrusion_property_attributes::height == 0.2f);
    CHECK(leaf_attributes->mm3_per_mm() == Approx(0.1));
}

// Two paths in one pass, one footprint touching only the first: only that path is rebuilt (split, thinned, non-reversible);
// the other keeps its object, flags, flow and place among its siblings.
TEST_CASE("Native support path modulation keeps the paths no footprint touches", "[plugins][support-toolpath-legacy]")
{
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    const coord_t layer_z = scale_to_layer_coord(1.);
    const std::vector<c_point> cut_line{c_point{0, 0}, c_point{scale_i(10.), 0}};
    // At y = 5 mm, 3.8 mm outside the footprint (y in [-1, 1] mm) grown by half the 0.4 mm width.
    const std::vector<c_point> far_line{c_point{0, scale_i(5.)}, c_point{scale_i(10.), scale_i(5.)}};
    slic3r_api::EPropertyAttributes attributes;
    attributes.extrusion_role(int32_t(RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL)).mm3_per_mm(0.1).width(0.4f).height(0.2f);
    const std::vector<c_point> square{c_point{scale_i(4.), -scale_i(1.)}, c_point{scale_i(6.), -scale_i(1.)},
                                      c_point{scale_i(6.), scale_i(1.)}, c_point{scale_i(4.), scale_i(1.)}};
    slic3r_api::StoredPolygonCollection footprint(handle);
    slic3r_api::StoredPolygon polygon(handle);
    polygon.insert_array(0, square.data(), uint32_t(square.size()));
    footprint.push_back(polygon);
    std::vector<ToolpathInternal::Overlap> overlapping;
    overlapping.push_back(ToolpathInternal::Overlap{scale_to_layer_coord(0.9), std::move(footprint)});
    slic3r_api::StoredExtrusionEntity pass(handle);
    for (const std::vector<c_point> &line : {cut_line, far_line}) {
        slic3r_api::StoredExtrusionEntity path(handle, line);
        path.set_flags(uint32_t(RAW_EXTRUSION_FLAG_REVERSIBLE));
        path.get_or_add(slic3r_api::EPropertyAttributes::key) = attributes;
        pass.append_child_move(path.mutable_view());
    }
    // What the second path is before the modulation: its object, flags and role must be the same afterwards.
    REQUIRE(pass.readonly().child_count() == 2);
    const extrusion_entity_handle *second_handle = pass.readonly().child(1).handle();
    const uint32_t second_flags = pass.readonly().child(1).flags();
    const slic3r_api::EPropertyAttributes *second_before = pass.readonly().child(1).get(slic3r_api::EPropertyAttributes::key);
    REQUIRE(second_before != nullptr);
    const auto second_role = second_before->c_extrusion_property_attributes::role;
    ToolpathInternal::modulate(handle, pass.mutable_view(), layer_z, overlapping, SCALED_EPSILON);
    REQUIRE(pass.readonly().child_count() == 2);
    // The first path is split into three parts, the middle one thinned, and neither it nor its parts can be reversed.
    const slic3r_api::ExtrusionEntity first = pass.readonly().child(0);
    REQUIRE(!first.has_polyline());
    REQUIRE(first.child_count() == 3);
    CHECK(!first.reversible());
    CHECK(!first.sortable());
    std::vector<slic3r_api::ExtrusionEntity> leaves;
    collect_api_leaves(first, leaves);
    REQUIRE(leaves.size() == 3);
    for (const slic3r_api::ExtrusionEntity &leaf : leaves)
        CHECK(!leaf.reversible());
    const slic3r_api::EPropertyAttributes *middle = leaves[1].get(slic3r_api::EPropertyAttributes::key);
    REQUIRE(middle != nullptr);
    CHECK(middle->c_extrusion_property_attributes::height == Approx(0.1));
    CHECK(middle->mm3_per_mm() == Approx(0.1 * 0.1 / 0.2));
    // The second path is the same leaf as before: one polyline, still reversible, same points, height and flow, still second.
    const slic3r_api::ExtrusionEntity second = pass.readonly().child(1);
    REQUIRE(second.has_polyline());
    CHECK(second.handle() == second_handle);
    CHECK(second.flags() == second_flags);
    CHECK(second.reversible());
    const std::vector<c_point> second_points = second.points();
    REQUIRE(second_points.size() == far_line.size());
    for (size_t index = 0; index < far_line.size(); ++index) {
        CHECK(second_points[index].x == far_line[index].x);
        CHECK(second_points[index].y == far_line[index].y);
    }
    const slic3r_api::EPropertyAttributes *second_attributes = second.get(slic3r_api::EPropertyAttributes::key);
    REQUIRE(second_attributes != nullptr);
    CHECK(second_attributes->c_extrusion_property_attributes::role == second_role);
    CHECK(second_attributes->c_extrusion_property_attributes::height == 0.2f);
    CHECK(second_attributes->mm3_per_mm() == Approx(0.1));
}

// With a Z step of 0.04 mm and layers of 0.04 mm, the contact match window (SCALED_EPSILON + 2 * the close-Z limit, about one step) holds the
// neighbouring object layers too: the match keeps the layer nearest to the expected Z, the lower one on a tie, and none when the window is empty.
TEST_CASE("Contact match keeps the nearest object layer", "[plugins][support-toolpath-legacy][support-close-z-step]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = SG::config("grid", false, 0);
    config.set_deserialize_strict({{"z_step", "0.04"}, {"layer_height", "0.04"}, {"first_layer_height", "0.04"},
                                   {"min_layer_height", "0.04,0.04"}, {"max_layer_height", "0.04,0.04"}});
    Print print;
    Model model;
    Test::init_print({make_cube(10., 10., 1.)}, print, model, config);
    SG::prefix(print);
    const PrintObject &host_object = print.object(0);
    const slic3r_api::Object object = SG::object_view(host_object);
    const uint32_t count = object.layer_count();
    REQUIRE(count > 12);
    const uint32_t k = 10;
    const coord_t step = scale_to_layer_coord(0.04);
    REQUIRE(object.layer(k + 1).bottom_z() - object.layer(k).bottom_z() == step);
    REQUIRE(object.layer(k).print_z() - object.layer(k).bottom_z() == step);
    const coord_t tolerance = SCALED_EPSILON + 2 * slic3r_api::Support::Common::close_z_tolerance_limit(0.04);
    REQUIRE(tolerance > step);
    // Both neighbours lie in the window, yet the layer on the expected Z wins, whichever key is compared.
    CHECK(ToolpathInternal::matched_layer(object, object.layer(k).bottom_z(), tolerance, true) == k);
    CHECK(ToolpathInternal::matched_layer(object, object.layer(k).bottom_z() + 3000, tolerance, true) == k);
    CHECK(ToolpathInternal::matched_layer(object, object.layer(k).bottom_z() - 3000, tolerance, true) == k);
    CHECK(ToolpathInternal::matched_layer(object, object.layer(k).print_z(), tolerance, false) == k);
    CHECK(ToolpathInternal::matched_layer(object, object.layer(k).print_z() - 3000, tolerance, false) == k);
    // Exactly between two layers: the lower one.
    CHECK(ToolpathInternal::matched_layer(object, object.layer(k).bottom_z() + step / 2, tolerance, true) == k);
    CHECK(ToolpathInternal::matched_layer(object, object.layer(k).bottom_z() + step / 2 + 100, tolerance, true) == k + 1);
    // Nothing in the window.
    CHECK(ToolpathInternal::matched_layer(object, object.layer(count - 1).print_z() + 10 * step, tolerance, false) == count);
}

// Generates grid/tree supports on variable object heights and runs the registered toolpath providers. Print/model own
// inputs and published paths; scoped counter enablement is restored on failure. Counts are diagnostic, while every leaf must
// have positive geometry/flow, fit its slab height and scale nominal mm3_per_mm consistently with its height.
// variant: "soluble" (no Z gap), "plane" (non-soluble, contact distance type plane) or "filament" (non-soluble, filament distance).
// Non-soluble contacts keep a Z gap, so support layers of different heights can overlap in Z, which is what triggers the
// legacy modulation; soluble ones never overlap.
void check_variable_height_toolpaths(const char *style, const char *variant)
{
    namespace Counters = slic3r_api::Support::Common::Counters;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool soluble = strcmp(variant, "soluble") == 0;
    CAPTURE(style, variant);
    // Soluble contacts exclude bridging flow, leaving the ordinary role flow as the nominal reference for non-flange paths.
    DynamicPrintConfig config = SG::config(style, soluble, 3);
    config.set_deserialize_strict({{"support_material_synchronize_layers", "0"}});
    if (strcmp(variant, "filament") == 0)
        config.set_deserialize_strict({{"support_material_contact_distance_type", "filament"}});
    TriangleMesh mesh = make_cube(4., 8., 6.);
    TriangleMesh roof = make_cube(12., 8., 1.);
    roof.translate(-4., 0., 6.);
    mesh.merge(roof);
    const std::vector<coordf_t> profile{0., 0.1, 2., 0.1, 2.01, 0.3, 4., 0.3, 4.01, 0.15, 7., 0.15};
    Print print;
    Model model;
    SG::init_print_with_profile(print, model, std::move(mesh), profile, config);
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    REQUIRE(object.layer_count() > 2);
    bool variable = false;
    for (size_t id = 1; id < object.layer_count(); ++id)
        variable = variable || object.layer(id).scaled_height() != object.layer(0).scaled_height();
    REQUIRE(variable);
    SG::family(print);
    REQUIRE_FALSE(object.auxiliary_layers().empty());
    const slic3r_api::Support::Common::SupportParams params = SG::params(print, object);
    REQUIRE(params.soluble == soluble);
    const slic3r_api::Object object_view = SG::object_view(object);
    struct RestoreCounters {
        bool previous;
        // Restores process-owned counter enablement after synchronous toolpath generation, including assertion/exception exits.
        ~RestoreCounters() { Counters::set_enabled_for_tests(previous); }
    } restore{Counters::set_enabled_for_tests(true)};
    Counters::reset();
    SG::toolpaths(print);
    const unsigned long long modulation_layers = Counters::totals.modulation_layers.load(std::memory_order_relaxed);
    const unsigned long long height_changes = Counters::totals.height_changes.load(std::memory_order_relaxed);
    const unsigned long long overlap_pairs = Counters::totals.overlap_pairs.load(std::memory_order_relaxed);
    INFO("style=" << style << " variant=" << variant << " modulation_layers=" << modulation_layers << " height_changes=" << height_changes << " overlap_pairs=" << overlap_pairs);
    std::printf("support-toolpath-variable-height style=%s variant=%s modulation_layers=%llu height_changes=%llu overlap_pairs=%llu\n",
        style, variant, modulation_layers, height_changes, overlap_pairs);
    size_t leaves = 0, thinner = 0, layers_with_paths = 0;
    // Resolves inherited attributes in borrowed trees, checks each leaf against its original role/slab flow and accumulates
    // scalar counts without changing paths; nominal is a non-bridging flow and geometry/generation failures reach Catch.
    const auto inspect = [&](const auto &self, const ExtrusionEntity &entity, const ExtrusionAttributes *inherited,
                             const c_flow &nominal, double layer_height, bool bridging_flow) -> void {
        const ExtrusionAttributes *attributes = entity.get_property<ExtrusionAttributes>();
        if (!attributes) attributes = inherited;
        if (entity.has_polyline()) {
            REQUIRE(attributes != nullptr);
            REQUIRE(std::isfinite(attributes->height));
            REQUIRE(attributes->height > 0.);
            CHECK(double(attributes->height) <= layer_height + 1e-6);
            CHECK(std::isfinite(attributes->width));
            CHECK(attributes->width > 0.);
            REQUIRE(std::isfinite(attributes->mm3_per_mm));
            REQUIRE(attributes->mm3_per_mm > 0.);
            const float nominal_height = float(unscaled(nominal.height));
            REQUIRE(nominal_height > 0.f);
            // A non-soluble bottom contact prints with the bridging flow, whose width and height are not those of the
            // ordinary interface role: the comparison to the role flow only holds without it.
            if (!bridging_flow) {
                CHECK(attributes->width == Approx(unscaled(nominal.width)).margin(1e-6));
                CHECK(attributes->mm3_per_mm == Approx(nominal.mm3_per_mm * (attributes->height / nominal_height)).epsilon(1e-5));
            }
            const std::vector<ArcPolyline> lines = entity.as_polylines();
            REQUIRE_FALSE(lines.empty());
            for (const ArcPolyline &line : lines) {
                CHECK(line.size() >= 2);
                CHECK(line.length() > 0);
            }
            ++leaves;
            thinner += double(attributes->height) < layer_height - 1e-6;
        }
        for (const ExtrusionEntityUPtr &child : entity.children())
            self(self, *child, attributes, nominal, layer_height, bridging_flow);
    };
    for (const Layer &layer : object.auxiliary_layers()) {
        const size_t before = leaves;
        const double height = layer.unscaled_height();
        // Only the interface bucket of a layer holding a bottom contact surface can carry the bridging flow (non-soluble only).
        bool bottom_contact = false;
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands())
                for (const Surface &surface : region.fill_surfaces())
                    bottom_contact = bottom_contact || (surface.surface_type & stPosBottom) != 0;
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands())
                for (const ExtrusionRole bucket : {LayerRegionIsland::SUPPORT, LayerRegionIsland::SUPPORT_INTERFACE}) {
                    if (!region.has_extrusion(bucket)) continue;
                    const raw_extrusion_role role = bucket == LayerRegionIsland::SUPPORT ?
                        RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL : RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL_INTERFACE;
                    const c_flow nominal = layer.scaled_bottom_z() <= 0 ? params.first_flow :
                        object_view.support_flow(role, params.layer_height, height);
                    inspect(inspect, region.extrusion(bucket), nullptr, nominal, height,
                            !soluble && bucket == LayerRegionIsland::SUPPORT_INTERFACE && bottom_contact);
                }
        layers_with_paths += leaves > before;
    }
    REQUIRE(leaves > 0);
    REQUIRE(layers_with_paths > 0);
    INFO("thinner_leaves=" << thinner << " leaves=" << leaves << " layers_with_paths=" << layers_with_paths);
    std::printf("support-toolpath-variable-height style=%s variant=%s thinner_leaves=%zu leaves=%zu layers_with_paths=%zu\n",
        style, variant, thinner, leaves, layers_with_paths);
}

// Soluble contacts: no Z gap, so no support layer overlaps another (the legacy modulation does not fire).
TEST_CASE("Variable layer height support toolpaths report legacy modulation", "[plugins][support-toolpath-legacy][support-toolpath-variable-height]")
{
    check_variable_height_toolpaths(GENERATE("grid", "tree"), "soluble");
}

// Non-soluble contacts with a Z gap (plane and filament distance types): support layers of different heights can overlap in Z.
TEST_CASE("Variable layer height support toolpaths with a Z gap report legacy modulation", "[plugins][support-toolpath-legacy][support-toolpath-variable-height]")
{
    const char *style = GENERATE("grid", "tree");
    const char *variant = GENERATE("plane", "filament");
    check_variable_height_toolpaths(style, variant);
}

// With no interface layers and separate support extruders the contact cannot merge into the base, so
// it stays on its role's surface and that role prints it with the base settings and its own filler.
TEST_CASE("Native contacts print with base settings without interface layers", "[plugins][support-toolpath-legacy]")
{
    DynamicPrintConfig config = SG::config("grid", false, 0, false);
    config.set_deserialize_strict({{"support_material_angle", "0"}, {"support_material_interface_angle", "90"}});
    Print print;
    Model model;
    Test::init_print({SG::slab(true)}, print, model, config);
    SG::prefix(print);
    SG::family(print);
    SG::toolpaths(print);
    PathCounts counts;
    size_t contact_layers = 0, wrong_tool = 0, sparse_contact_layers = 0;
    bool vertical = false;
    for (const Layer &layer : print.object(0).auxiliary_layers()) {
        // Collect the dense surfaces of this layer; the sparse base must not share them.
        ExPolygons dense_area;
        bool sparse_surface = false;
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands())
                for (const Surface &surface : region.fill_surfaces()) {
                    const SurfaceType type = SurfaceType(surface.surface_type & ~stModContact);
                    if (type == (stModSupport | stPosTop | stDensSolid) ||
                        type == (stModSupport | stPosBottom | stDensSolid) ||
                        type == (stModSupport | stPosBottom | stDensSolid | stModBridge))
                        dense_area.push_back(surface.expolygon);
                    else if (type == (stModSupport | stPosInternal | stDensSparse)) sparse_surface = true;
                }
        if (dense_area.empty()) continue;
        ++contact_layers;
        if (sparse_surface) ++sparse_contact_layers;
        // The role writes its own contact into the support bucket with the support tool.
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands()) {
                if (!region.has_extrusion(LayerRegionIsland::SUPPORT)) continue;
                const ExtrusionEntity &paths = region.extrusion(LayerRegionIsland::SUPPORT);
                check_paths(paths, LayerRegionIsland::SUPPORT, counts, false);
                if (region.extruder_id() != 0) ++wrong_tool;
                for (const ArcPolyline &polyline : paths.as_polylines())
                    vertical = vertical || std::abs(polyline.back().y() - polyline.front().y()) >
                        std::abs(polyline.back().x() - polyline.front().x());
                // The paths stay inside the surface that carries them.
                const Polylines lines = to_polylines(paths.as_polylines());
                const Polygons allowed = offset(union_ex(dense_area), scale_d(0.01));
                double kept = 0, total = 0;
                for (const Polyline &line : intersection_pl(lines, allowed)) kept += unscaled(line.length());
                for (const Polyline &line : lines) total += unscaled(line.length());
                CHECK(kept == Approx(total));
            }
    }
    CHECK(contact_layers > 0);
    CHECK(counts.base > 0);
    CHECK(counts.interface == 0);
    CHECK(wrong_tool == 0);
    CHECK(sparse_contact_layers == 0);
    CHECK_FALSE(vertical);
}

// The filler choice of a contact printed with the base settings is decided by surface_pattern and by
// the family's typing rule (see "Grid family types a mergeable contact as base"); the cases here
// cover the filler patterns and the base-settings path through real surfaces.
// FillSupportBase rejects full-infill and bridging parameters, so a bridging contact that is not
// printed as base falls back to rectilinear; a base-printed part uses the base pattern where the
// support regions could merge and the intermediate rectilinear filler otherwise.
TEST_CASE("Native support pattern keeps bridging contacts away from the sparse pattern", "[plugins][support-toolpath-legacy]")
{
    CHECK(ToolpathInternal::surface_pattern(false, true, true, false, "supportbase", "honeycomb") == "rectilinear");
    CHECK(ToolpathInternal::surface_pattern(false, true, false, false, "supportbase", "honeycomb") == "supportbase");
    CHECK(ToolpathInternal::surface_pattern(false, false, false, false, "concentric", "honeycomb") == "rectilinear");
    CHECK(ToolpathInternal::surface_pattern(true, true, false, true, "concentric", "honeycomb") == "honeycomb");
    CHECK(ToolpathInternal::surface_pattern(true, true, false, false, "concentric", "honeycomb") == "rectilinear");
}

// An automatic bottom pattern resolving to the sparse pattern must not reach the bridging bottom
// contact: the run completes and the contact keeps the paths of an explicit rectilinear pattern.
TEST_CASE("Native bridging bottom contacts avoid the sparse pattern", "[plugins][support-toolpath-legacy]")
{
    // Slice the platform slab with the automatic or the explicit rectilinear bottom pattern and
    // measure the bridging-flow interface paths the family publishes.
    const auto slice_and_measure = [](const char *bottom_pattern) {
        DynamicPrintConfig config = SG::config("grid", false, 3);
        config.set_deserialize_strict({{"support_material_top_interface_pattern", "auto"},
                                       {"support_material_bottom_interface_pattern", bottom_pattern},
                                       {"support_material_interface_spacing", "2"}});
        Print print;
        Model model;
        Test::init_print({SG::slab(true)}, print, model, config);
        SG::prefix(print);
        SG::family(print);
        SG::toolpaths(print);
        return published_bridge_length(print.object(0));
    };
    const double automatic = slice_and_measure("auto");
    const double rectilinear = slice_and_measure("rectilinear");
    CHECK(automatic > 0);
    CHECK(automatic == Approx(rectilinear));
}
