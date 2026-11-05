#include "polygon_metrics_test_helpers.hpp"
#include "support_grid_test_helpers.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_support_demand.h"
#include "libslic3r/Plugins/Support/Filled/FilledSupportGenerator.hpp"
#include "libslic3r/Plugins/Support/Common/SupportSettings.hpp"
#include "libslic3r/Plugins/Support/Common/SupportToolpathRoles.hpp"

/* Filled-family tests inspect temporary slabs and the published auxiliary layers before toolpaths.
   The production slicing prefix supplies final island demand; local helpers copy polygons for area comparisons.
   Tests cover downward projection, contact splitting, object-layer Z, foot expansion and visible refusal. */

using namespace Slic3r;
namespace SG = Slic3r::Test::SupportGrid;
namespace Common = slic3r_api::Support::Common;
namespace {
// Return owned soluble settings without a raft; the serialized expansion controls the common first-layer ring.
DynamicPrintConfig filled_config(const char *expansion = "0");
// Copy slab polygons at exact scaled Z; the slabs and their storage remain unchanged.
Polygons slabs_at(const Common::GeneratorLayers &list, coord_t z);
// Return union area in mm2 without double-counting overlaps; geometry failures propagate.
double area_mm2(const Polygons &polygons);
// Return an owned mesh of separated cantilevers with roofs at 3 and 6 mm for mixed contact/interface layers.
TriangleMesh two_roofs();
// Copy the first nonempty demand union and return its layer index, or size_t(-1) with empty demand.
size_t demand_layer(const PrintObject &object, ExPolygons &demand);

struct Generated {
    PluginStorage storage;
    Common::GeneratedSupport support{SG::handle(storage)};
    // Generate owned slabs from borrowed sliced inputs; storage outlives the slabs and generation failures propagate.
    Generated(const Print &print, const PrintObject &object);
};

DynamicPrintConfig filled_config(const char *expansion)
{
    DynamicPrintConfig config = SG::config("filled", true);
    config.set_deserialize_strict({{"raft_layers", "0"}, {"raft_first_layer_expansion", expansion}});
    return config;
}

Generated::Generated(const Print &print, const PrintObject &object)
{
    const Common::SupportParams params = SG::params(print, object);
    slic3r_api::Support::Filled::generate_support(nullptr, SG::print_view(print), SG::object_view(object), params, support);
}

Polygons slabs_at(const Common::GeneratorLayers &list, coord_t z)
{
    Polygons result;
    for (const Common::GeneratorLayer *slab : list)
        if (slab->print_z == z) append(result, SG::native(slab->polygons));
    return result;
}

double area_mm2(const Polygons &polygons)
{
    return unscaled(unscaled(area(union_ex(polygons))));
}

TriangleMesh two_roofs()
{
    TriangleMesh result;
    for (size_t bay = 0; bay < 2; ++bay) {
        const double z = bay == 0 ? 3. : 6.;
        TriangleMesh post = make_cube(4., 8., z);
        TriangleMesh roof = make_cube(12., 8., 1.);
        post.translate(float(20 * bay), 0.f, 0.f);
        roof.translate(float(20 * bay), 0.f, float(z));
        result.merge(post);
        result.merge(roof);
    }
    return result;
}

size_t demand_layer(const PrintObject &object, ExPolygons &demand)
{
    demand.clear();
    for (size_t id = 0; id < object.layer_count(); ++id) {
        for (const LayerSliceIsland &island : object.layer(id).islands()) append(demand, island.support_demand());
        if (!demand.empty()) {
            demand = union_ex(demand);
            return id;
        }
    }
    return size_t(-1);
}
}

// Compare every generated layer with a nonempty independent projection of the roof demand.
TEST_CASE("Filled support projects the overhang demand straight down", "[plugins][support-filled]")
{
    Print print;
    Model model;
    Test::init_print({SG::slab()}, print, model, filled_config());
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    ExPolygons demand;
    const size_t roof = demand_layer(object, demand);
    REQUIRE(roof != size_t(-1));
    REQUIRE(roof > 0);
    size_t demanded_layers = 0;
    for (const Layer &layer : object.layers())
        for (const LayerSliceIsland &island : layer.islands())
            if (!island.support_demand().empty()) ++demanded_layers;
    REQUIRE(demanded_layers == 1);
    const Generated generated(print, object);
    const Common::SupportParams params = SG::params(print, object);
    double total = 0.;
    for (size_t id = 0; id < roof; ++id) {
        const Layer &layer = object.layer(id);
        Polygons actual = slabs_at(generated.support.top, layer.scaled_print_z());
        append(actual, slabs_at(generated.support.interfaces, layer.scaled_print_z()));
        const Polygons expected = diff(to_polygons(demand), offset(layer.lslices(), double(params.gap_xy), ClipperLib::jtSquare));
        CHECK(Test::PolygonMetrics::sym_diff_mm2(actual, expected) < 0.01);
        total += area_mm2(actual);
    }
    REQUIRE(total > 0.);
    for (const Common::GeneratorLayer &slab : generated.support.slabs)
        CHECK(slab.print_z < object.layer(roof).scaled_print_z());
}

// Inspect nonempty slabs against a slightly reduced XY mask, keeping the configured spacing observable.
TEST_CASE("Filled support keeps the XY gap to the object", "[plugins][support-filled]")
{
    const char *gap = GENERATE("0.3", "1.5");
    DynamicPrintConfig config = filled_config();
    config.set_deserialize_strict({{"support_material_xy_spacing", gap}});
    Print print;
    Model model;
    Test::init_print({SG::slab()}, print, model, config);
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    const Common::SupportParams params = SG::params(print, object);
    CHECK(unscaled(params.gap_xy) == Approx(std::stod(gap)));
    const Generated generated(print, object);
    REQUIRE_FALSE(generated.support.slabs.empty());
    double total = 0.;
    for (const Common::GeneratorLayer &slab : generated.support.slabs) {
        for (const Layer &layer : object.layers())
            if (layer.scaled_print_z() == slab.print_z)
                CHECK(area_mm2(intersection(SG::native(slab.polygons),
                    offset(layer.lslices(), double(params.gap_xy - scale_i(0.01)), ClipperLib::jtSquare))) < 1e-4);
        total += area_mm2(SG::native(slab.polygons));
    }
    REQUIRE(total > 0.);
}

// A covering object platform must stop the carried column permanently, not only hide its intersecting layer.
TEST_CASE("Filled support stops on an object that covers it", "[plugins][support-filled]")
{
    Print print;
    Model model;
    Test::init_print({SG::slab(true)}, print, model, filled_config());
    SG::prefix(print);
    const Generated generated(print, print.object(0));
    REQUIRE_FALSE(generated.support.top.empty());
    CHECK(generated.support.base.empty());
    coord_t lowest = std::numeric_limits<coord_t>::max();
    for (const Common::GeneratorLayer &slab : generated.support.slabs) {
        CHECK(slab.print_z > scale_to_layer_coord(0.8));
        lowest = std::min(lowest, slab.print_z);
    }
    for (const Layer &layer : print.object(0).layers())
        if (layer.scaled_print_z() > scale_to_layer_coord(0.8)) {
            CHECK(lowest == layer.scaled_print_z());
            break;
        }
}

// Restrict final demand to the upper roof so an intermediate shelf cannot seed replacement columns below itself.
TEST_CASE("Filled support does not restart below an intermediate obstacle", "[plugins][support-filled]")
{
    TriangleMesh mesh = make_cube(4., 8., 6.);
    TriangleMesh roof = make_cube(12., 8., 1.);
    roof.translate(0., 0., 6.);
    mesh.merge(roof);
    TriangleMesh shelf = make_cube(12., 8., 0.6);
    shelf.translate(0., 0., 3.);
    mesh.merge(shelf);
    Print print;
    Model model;
    Test::init_print({mesh}, print, model, filled_config());
    SG::prefix(print);
    PrintObject &object = print.object(0);
    PluginStorage storage;
    run_ctx_support_demand payload{reinterpret_cast<const print_handle *>(&print),
                                   reinterpret_cast<const object_handle *>(&object)};
    plugin_run_context context = {};
    context.step = STEP_SUPPORT_DEMAND;
    context.data = &payload;
    context.plugin_storage = SG::handle(storage);
    ExPolygons roof_demand;
    const ExPolygons empty;
    size_t roof_layers = 0;
    // Keep only the final roof's demand; the lower shelf remains an obstacle, not another source of support.
    for (Layer &layer : object.layers())
        for (LayerSliceIsland &island : layer.islands()) {
            if (layer.scaled_bottom_z() >= scale_to_layer_coord(6.) && !island.support_demand().empty()) {
                append(roof_demand, island.support_demand());
                ++roof_layers;
            } else
                expolygons_copy(support_demand_get_island_mutable(&context,
                    reinterpret_cast<const layer_island_handle *>(&island)),
                    reinterpret_cast<const expolygon_collection_handle *>(&empty));
        }
    REQUIRE(roof_layers == 1);
    REQUIRE_FALSE(roof_demand.empty());
    const Generated generated(print, object);
    const Common::SupportParams params = SG::params(print, object);
    bool above = false, free_below = false;
    for (const Layer &layer : object.layers()) {
        Polygons support = slabs_at(generated.support.top, layer.scaled_print_z());
        append(support, slabs_at(generated.support.interfaces, layer.scaled_print_z()));
        if (layer.scaled_bottom_z() >= scale_to_layer_coord(3.6) &&
            layer.scaled_print_z() <= scale_to_layer_coord(6.))
            above = above || area_mm2(intersection(support, to_polygons(roof_demand))) > 1.;
        if (layer.scaled_print_z() <= scale_to_layer_coord(3.)) {
            const Polygons free = diff(to_polygons(roof_demand),
                offset(layer.lslices(), double(params.gap_xy), ClipperLib::jtSquare));
            REQUIRE(area_mm2(free) > 1.);
            CHECK(area_mm2(intersection(support, free)) < 1e-4);
            free_below = true;
        }
    }
    REQUIRE(above);
    REQUIRE(free_below);
    CHECK(generated.support.base.empty());
}

// Compare both temporary and published Z/height with the actual variable object layers, not the requested profile.
TEST_CASE("Filled support layers take the Z of the object layers", "[plugins][support-filled]")
{
    Print print;
    Model model;
    SG::init_print_with_profile(print, model, SG::slab(), {0., 0.2, 2., 0.2, 2.01, 0.1, 5., 0.1}, filled_config());
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    const Generated generated(print, object);
    REQUIRE_FALSE(generated.support.slabs.empty());
    std::set<coord_t> heights;
    for (const Common::GeneratorLayer &slab : generated.support.slabs) {
        bool found = false;
        for (const Layer &layer : object.layers())
            if (layer.scaled_print_z() == slab.print_z) {
                found = true;
                CHECK(slab.bottom_z == layer.scaled_bottom_z());
                CHECK(slab.height == layer.scaled_height());
                heights.insert(slab.height);
            }
        CHECK(found);
    }
    REQUIRE(heights.size() > 1);
    SG::family(print);
    REQUIRE_FALSE(object.auxiliary_layers().empty());
    std::set<coord_t> published;
    for (const Layer &auxiliary : object.auxiliary_layers()) {
        CHECK(published.insert(auxiliary.scaled_print_z()).second);
        bool found = false;
        for (const Layer &layer : object.layers())
            if (layer.scaled_print_z() == auxiliary.scaled_print_z()) {
                found = true;
                CHECK(auxiliary.scaled_bottom_z() == layer.scaled_bottom_z());
                CHECK(auxiliary.scaled_height() == layer.scaled_height());
            }
        CHECK(found);
    }
}

// The final contact-only layer receives the next interface id without a second family-local counter.
TEST_CASE("Filled support numbers one interface id per layer", "[plugins][support-filled]")
{
    Print print;
    Model model;
    Test::init_print({SG::slab()}, print, model, filled_config());
    SG::prefix(print);
    ExPolygons demand;
    const size_t roof = demand_layer(print.object(0), demand);
    REQUIRE(roof != size_t(-1));
    SG::family(print);
    const PrintObject &object = print.object(0);
    REQUIRE(object.auxiliary_layers().size() == roof);
    size_t id = 0;
    for (const Layer &layer : object.auxiliary_layers()) {
        const LayerSupportProperty *property = layer.get_property<LayerSupportProperty>();
        REQUIRE(property != nullptr);
        CHECK(property->interface_id == id++);
    }
}

// Splitting at the upper object must produce disjoint contact/interface areas and exact published surface roles.
TEST_CASE("Filled support splits the contact and the interface without overlap", "[plugins][support-filled]")
{
    Print print;
    Model model;
    Test::init_print({two_roofs()}, print, model, filled_config());
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    const Generated generated(print, object);
    CHECK(generated.support.bottom.empty());
    bool mixed = false;
    for (size_t id = 1; id < object.layer_count(); ++id) {
        const coord_t z = object.layer(id - 1).scaled_print_z();
        const Polygons contact = slabs_at(generated.support.top, z);
        const Polygons interface = slabs_at(generated.support.interfaces, z);
        const Polygons upper = to_polygons(object.layer(id).lslices());
        CHECK(area_mm2(diff(contact, upper)) < 1e-4);
        CHECK(area_mm2(intersection(interface, upper)) < 1e-4);
        CHECK(area_mm2(intersection(contact, interface)) < 1e-4);
        Polygons combined = contact;
        append(combined, interface);
        CHECK(area_mm2(combined) == Approx(area_mm2(contact) + area_mm2(interface)).margin(1e-4));
        mixed = mixed || (!contact.empty() && !interface.empty());
    }
    REQUIRE(mixed);
    SG::family(print);
    REQUIRE_FALSE(object.auxiliary_layers().empty());
    for (const Layer &layer : object.auxiliary_layers()) {
        Polygons published;
        Polygons contacts, interfaces, sparse;
        double sum = 0.;
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands())
                for (const Surface &surface : region.fill_surfaces()) {
                    const SurfaceType type = surface.surface_type;
                    CHECK((type == (stPosTop | stDensSolid | stModSupport | stModContact) ||
                           type == (stPosTop | stDensSolid | stModSupport) ||
                           type == (stPosInternal | stDensSparse | stModSupport)));
                    CHECK(region.extruder_id() == (surface.has_pos_top() ? 1 : 0));
                    append(surface.has_mod_contact() ? contacts : surface.has_pos_top() ? interfaces : sparse,
                        to_polygons(surface.expolygon));
                    append(published, to_polygons(surface.expolygon));
                    sum += unscaled(unscaled(surface.expolygon.area()));
                }
        Polygons expected = slabs_at(generated.support.top, layer.scaled_print_z());
        append(expected, slabs_at(generated.support.interfaces, layer.scaled_print_z()));
        append(expected, slabs_at(generated.support.base, layer.scaled_print_z()));
        CHECK(Test::PolygonMetrics::sym_diff_mm2(contacts, slabs_at(generated.support.top, layer.scaled_print_z())) < 1e-4);
        CHECK(Test::PolygonMetrics::sym_diff_mm2(interfaces, slabs_at(generated.support.interfaces, layer.scaled_print_z())) < 1e-4);
        CHECK(Test::PolygonMetrics::sym_diff_mm2(sparse, slabs_at(generated.support.base, layer.scaled_print_z())) < 1e-4);
        CHECK(Test::PolygonMetrics::sym_diff_mm2(published, expected) < 1e-4);
        CHECK(area_mm2(published) == Approx(sum).margin(1e-4));
    }
}

// Common foot expansion leaves only a sparse base-tool ring, disjoint from dense support and the object gap.
TEST_CASE("Filled support widens the first-layer foot", "[plugins][support-filled]")
{
    const char *expansion = GENERATE("0", "3");
    Print print;
    Model model;
    Test::init_print({SG::slab()}, print, model, filled_config(expansion));
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    const Generated generated(print, object);
    const Layer &first = object.layer(0);
    const Polygons foot = slabs_at(generated.support.base, first.scaled_print_z());
    Polygons dense = slabs_at(generated.support.top, first.scaled_print_z());
    append(dense, slabs_at(generated.support.interfaces, first.scaled_print_z()));
    CHECK(area_mm2(intersection(foot, dense)) < 1e-4);
    const Common::SupportParams params = SG::params(print, object);
    CHECK(area_mm2(intersection(foot, offset(first.lslices(), double(params.gap_xy - scale_i(0.01)), ClipperLib::jtSquare))) < 1e-4);
    if (std::string(expansion) == "0") CHECK(area_mm2(foot) < 1e-4);
    else {
        REQUIRE(generated.support.base.size() == 1);
        CHECK(generated.support.base.front()->bottom_print_z() == 0);
        REQUIRE(area_mm2(foot) > 1.);
        SG::family(print);
        bool sparse = false;
        for (const LayerSliceIsland &island : object.auxiliary_layers().front().islands())
            for (const LayerRegionIsland &region : island.regions_islands())
                for (const Surface &surface : region.fill_surfaces())
                    if (surface.surface_type == (stPosInternal | stDensSparse | stModSupport)) {
                        sparse = true;
                        CHECK(region.extruder_id() == 0);
                    }
        REQUIRE(sparse);
    }
}

// Unsupported settings must stop the real runner visibly and leave no auxiliary layers, but empty demand is skipped.
TEST_CASE("Filled support refuses a contact distance or a raft", "[plugins][support-filled]")
{
    const bool raft = GENERATE(false, true);
    DynamicPrintConfig config = raft ? filled_config() : SG::config("filled", false);
    if (raft) config.set_deserialize_strict({{"raft_layers", "1"}});
    Print print;
    Model model;
    Test::init_print({SG::slab()}, print, model, config);
    SG::prefix(print);
    Orchestrator &host = Orchestrator::instance();
    struct Messages {
        Orchestrator &host;
        // Restore cancellation and drain diagnostics even when an assertion ends the test.
        ~Messages() { host.reset_plugin_cancel(); host.consume_plugin_messages(); }
    } reset{host};
    host.reset_plugin_cancel();
    host.consume_plugin_messages();
    CHECK_THROWS_AS(SG::family(print), RuntimeError);
    bool error = false;
    for (const Orchestrator::PluginMessage &message : host.consume_plugin_messages())
        if (message.plugin_id == "support.family.filled" && message.level == Orchestrator::PluginMessageLevel::Error &&
            message.message.find(raft ? "raft_layers" : "support_material_contact_distance_type") != std::string::npos)
            error = true;
    CHECK(error);
    CHECK(print.object(0).auxiliary_layers().empty());
    host.reset_plugin_cancel();
    Print empty;
    Model empty_model;
    Test::init_print({make_cube(4., 8., 5.)}, empty, empty_model, config);
    SG::prefix(empty);
    REQUIRE_NOTHROW(SG::family(empty));
    CHECK(empty.object(0).auxiliary_layers().empty());
}

// Registration must expose geometry keys plus the refused raft key and explicit active preferences for every role.
TEST_CASE("Filled family is registered with its keys", "[plugins][support-filled]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    REQUIRE(host.is_plugin_active("support.family.filled"));
    const Plugin *family = host.get_plugin("support.family.filled");
    REQUIRE(family != nullptr);
    CHECK(family->get_step() == STEP_SUPPORT);
    std::ifstream activation(std::string(TEST_DATA_DIR) + "/../../src/plugins/default_activated.ini.in");
    REQUIRE(activation.is_open());
    const std::string ini{std::istreambuf_iterator<char>(activation), std::istreambuf_iterator<char>()};
    CHECK(ini.find("support.family.filled = 1") != std::string::npos);
    const Common::UsedKeyList keys = Common::support_geometry_keys();
    REQUIRE(family->get_used_config_keys().size() == keys.size + 1);
    for (size_t id = 0; id < keys.size; ++id)
        CHECK(std::any_of(family->get_used_config_keys().begin(), family->get_used_config_keys().end(),
            [&](const Plugin::UsedConfigKey &key) { return key.key == keys.keys[id].key; }));
    CHECK(std::any_of(family->get_used_config_keys().begin(), family->get_used_config_keys().end(),
        [](const Plugin::UsedConfigKey &key) { return key.key == "raft_layers" && key.type == RAW_CO_INT; }));
    for (const Common::ToolpathPreference &preference : Common::legacy_toolpath_preferences(Common::k_legacy_sparse_id)) {
        CHECK(std::count_if(host.selector_auto_choices().begin(), host.selector_auto_choices().end(),
            [&](const SelectorAutoChoice &choice) {
                return choice.step == STEP_SUPPORT_TOOLPATH && choice.exclusive_group == preference.role &&
                    choice.condition_key == "support_material_style" && choice.condition_value == "filled" &&
                    choice.provider_id == preference.provider_id;
            }) == 1);
        REQUIRE(host.is_plugin_active(preference.provider_id));
        CHECK(host.get_plugin(preference.provider_id)->get_exclusive_group() == preference.role);
    }
    REQUIRE(host.is_plugin_active(Common::k_branch_sheath_id));
    const DynamicPrintConfig config = filled_config();
    const Plugin *selected = Steps::selected_active_plugin_from_group(host, STEP_SUPPORT_TOOLPATH, Common::k_sparse_role, &config);
    REQUIRE(selected != nullptr);
    CHECK(selected->get_id() == Common::k_legacy_sparse_id);
}

// Inject a conflicting copied preference only for initialization, then restore the registry and all active providers.
TEST_CASE("Rejected filled preferences disable the family with a diagnostic", "[plugins][support-filled][selector-auto]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    // The process registry has no removal API; scoped test-only mutation avoids adding a production reset operation.
    std::vector<SelectorAutoChoice> &choices = const_cast<std::vector<SelectorAutoChoice> &>(host.selector_auto_choices());
    struct Restore {
        Orchestrator &host;
        std::vector<SelectorAutoChoice> &choices;
        std::vector<SelectorAutoChoice> saved_choices;
        std::vector<Plugin *> saved_active;
        // Restore copied preferences and borrowed active providers even after an assertion; discard injected diagnostics.
        ~Restore() {
            choices.swap(saved_choices);
            host.clear_active_plugins();
            for (Plugin *plugin : saved_active) host.set_plugin_active(plugin, true);
            host.reset_plugin_cancel();
            host.consume_plugin_messages();
        }
    } restore{host, choices, choices, std::vector<Plugin *>(host.active_plugins().begin(), host.active_plugins().end())};
    bool injected = false;
    for (SelectorAutoChoice &choice : choices)
        if (choice.step == STEP_SUPPORT_TOOLPATH && choice.condition_key == "support_material_style" &&
            choice.condition_value == "filled" && choice.exclusive_group == Common::k_sparse_role) {
            choice.provider_id = Common::k_branch_sheath_id;
            injected = true;
        }
    REQUIRE(injected);
    host.clear_active_plugins();
    REQUIRE(host.set_plugin_active("support.family.filled", true));
    host.consume_plugin_messages();
    REQUIRE_NOTHROW(host.initialize_plugins());
    CHECK_FALSE(host.is_plugin_active("support.family.filled"));
    size_t errors = 0;
    for (const Orchestrator::PluginMessage &message : host.consume_plugin_messages())
        if (message.plugin_id == "support.family.filled" && message.level == Orchestrator::PluginMessageLevel::Error) {
            ++errors;
            CHECK(message.message.find("Conflicting automatic selector preference") != std::string::npos);
            CHECK(message.message.find(Common::k_sparse_role) != std::string::npos);
        }
    CHECK(errors == 1);
}

// Zero interface layers uses the existing Common publication and path settings, with either shared or separate tools.
TEST_CASE("Filled support keeps the common zero-interface behavior", "[plugins][support-filled]")
{
    const bool same_tool = GENERATE(false, true);
    DynamicPrintConfig config = SG::config("filled", true, 0, same_tool);
    Print print;
    Model model;
    Test::init_print({SG::slab()}, print, model, config);
    SG::prefix(print);
    REQUIRE_NOTHROW(SG::family(print));
    REQUIRE_FALSE(print.object(0).auxiliary_layers().empty());
    bool sparse = false;
    bool contact = false;
    for (const Layer &layer : print.object(0).auxiliary_layers())
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands())
                for (const Surface &surface : region.fill_surfaces()) {
                    if (same_tool) CHECK_FALSE(surface.has_mod_contact());
                    contact = contact || surface.has_mod_contact();
                    sparse = sparse || surface.has_fill_sparse();
                }
    CHECK(sparse == same_tool);
    CHECK(contact == !same_tool);
    REQUIRE_NOTHROW(SG::toolpaths(print));
}
