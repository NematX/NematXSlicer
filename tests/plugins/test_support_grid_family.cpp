#include "polygon_metrics_test_helpers.hpp"
#include "support_grid_test_helpers.hpp"
#include "gcode_test_helpers.hpp"
#include "libslic3r/Plugins/Support/Grid/SupportFamilyGrid.hpp"
#include "libslic3r/Plugins/Support/Common/SupportPublication.hpp"
#include "libslic3r/Plugins/Support/Common/SupportZPlacement.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_support_demand.h"

/* Grid-family tests inspect geometric output before toolpath generation. A
   slab on a post needs bed-reaching columns; a low platform also needs bottom
   contacts. Surface position bits encode which side of the object is supported.
   These tests exercise the real STEP_SUPPORT dispatcher and API publication.
   Variable-height cantilevers additionally check temporary slab placement and
   the published support tops before a complete G-code export. */

using namespace Slic3r;
namespace SG = Slic3r::Test::SupportGrid;

// Slice four separate cantilevers with borrowed production helpers, inspect their owned slabs, then process and export them.
// Roof demand below the first layer is dropped, near it is snapped, and higher roofs retain the configured gap relative to
// the bottom of their own sliced layer. Generation/export failures propagate after output cleanup.
TEST_CASE("Grid variable height contacts slabs and export follow each roof", "[plugins][support-grid][variable-layer-height][gcode]")
{
    namespace Common = slic3r_api::Support::Common;
    const bool sync = GENERATE(false, true);
    CAPTURE(sync);
    DynamicPrintConfig config = SG::config();
    config.set_deserialize_strict({{"support_material_synchronize_layers", sync ? "1" : "0"},
        {"support_material_contact_distance", "0.19"}, {"support_material_bottom_contact_distance", "0.23"},
        {"support_material_layer_height", "0.3"}, {"support_material_interface_layer_height", "0.3"},
        {"step_gcode_plugin", "gcode.printing_plan_file_writer"}, {"gcode_firmware_plugin", "gcode.firmware.marlin2"},
        {"filament_diameter", "1.75,1.75"}});
    // XY bays isolate footprints for attribution; roof Z spacing avoids contact merges. The last bay has a thin-band shelf.
    const double roofs[] = {0.25, 0.45, 3., 6.};
    TriangleMesh mesh;
    for (size_t bay = 0; bay < 4; ++bay) {
        TriangleMesh post = make_cube(4., 8., roofs[bay]);
        post.translate(float(20 * bay), 0.f, 0.f);
        TriangleMesh roof = make_cube(12., 8., 0.6);
        roof.translate(float(20 * bay), 0.f, float(roofs[bay]));
        mesh.merge(post);
        mesh.merge(roof);
        if (bay == 3) {
            TriangleMesh shelf = make_cube(12., 8., 2.4);
            shelf.translate(float(20 * bay), 0.f, 0.f);
            mesh.merge(shelf);
        }
    }
    Print print;
    Model model;
    SG::init_print_with_profile(print, model, std::move(mesh),
        std::vector<coordf_t>{0., 0.2, 2., 0.2, 2.01, 0.1, 4., 0.1, 4.01, 0.3, 6.6, 0.3}, config);
    REQUIRE(print.validate().first == PrintBase::PrintValidationError::pveNone);
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    const Common::SupportParams params = SG::params(print, object);
    REQUIRE(params.distance_type == Common::ContactDistanceType::Plane);
    REQUIRE(params.gap_above == Approx(0.19));
    REQUIRE(params.gap_below == Approx(0.23));
    const SG::Generated generated(print, object);
    REQUIRE_FALSE(generated.support.top.empty());
    REQUIRE_FALSE(generated.support.bottom.empty());

    // Wide disconnected slices identify each roof's first sliced layer, independently of mesh/slicing Z rounding.
    const coord_t origin_x = get_extents(object.layers().front().lslices()).min.x();
    std::vector<size_t> roof_layers(4, size_t(-1));
    std::vector<ExPolygons> roof_footprints(4);
    for (size_t id = 0; id < object.layer_count(); ++id)
        for (const ExPolygon &slice : object.layer(id).lslices()) {
            const BoundingBox box = get_extents(slice);
            if (unscaled(box.size().x()) < 10.) continue;
            const size_t bay = size_t(std::lround(unscaled(box.min.x() - origin_x) / 20.));
            REQUIRE(bay < 4);
            if (roof_layers[bay] == size_t(-1) && (bay != 3 || object.layer(id).unscaled_print_z() > 5.)) {
                roof_layers[bay] = id;
                roof_footprints[bay] = {slice};
            }
        }
    for (size_t id : roof_layers) {
        REQUIRE(id != size_t(-1));
        size_t demand = 0;
        for (const LayerSliceIsland &island : object.layer(id).islands()) demand += island.support_demand().size();
        REQUIRE(demand > 0);
    }
    CHECK(object.layer(roof_layers[2]).unscaled_height() == Approx(0.1));
    CHECK(object.layer(roof_layers[3]).unscaled_height() == Approx(0.3));
    std::set<size_t> contacted_roofs;
    bool snapped_first_layer_contact = false;
    for (const Common::GeneratorLayer *contact : generated.support.top) {
        REQUIRE(contact->object_above < object.layer_count());
        const Layer &above = object.layer(contact->object_above);
        const coord_t expected = above.scaled_bottom_z() - scale_to_layer_coord(params.gap_above);
        INFO("roof layer " << contact->object_above << " height " << above.unscaled_height()
             << " bottom " << unscaled(above.scaled_bottom_z()) << " support top " << unscaled(contact->print_z));
        if (contact->object_above == roof_layers[1]) {
            CHECK(expected >= scale_to_layer_coord(params.first_layer_height));
            CHECK(expected < scale_to_layer_coord(params.first_layer_height) + params.min_height);
            CHECK(contact->print_z == scale_to_layer_coord(params.first_layer_height));
            snapped_first_layer_contact = contact->print_z == scale_to_layer_coord(params.first_layer_height) &&
                contact->bottom_z == 0 && contact->height == contact->print_z;
            CHECK(snapped_first_layer_contact);
        } else {
            // Plane distance adds no distinct bridge contact; roof Z spacing avoids merges.
            CHECK(contact->print_z == expected);
        }
        contacted_roofs.insert(contact->object_above);
    }
    CHECK(contacted_roofs.count(roof_layers[0]) == 0);
    CHECK(object.layer(roof_layers[0]).scaled_bottom_z() - scale_to_layer_coord(params.gap_above)
          < scale_to_layer_coord(params.first_layer_height));
    for (size_t bay = 1; bay < 4; ++bay) CHECK(contacted_roofs.count(roof_layers[bay]) == 1);
    for (const Common::GeneratorLayer *contact : generated.support.bottom) {
        REQUIRE(contact->object_below < object.layer_count());
        INFO("bottom contact " << unscaled(contact->bottom_z) << " above object top "
             << object.layer(contact->object_below).unscaled_print_z());
        CHECK(contact->bottom_z == object.layer(contact->object_below).scaled_print_z() + scale_to_layer_coord(params.gap_below));
        CHECK(object.layer(contact->object_below).unscaled_print_z() == Approx(2.4));
        CHECK(unscaled(contact->bottom_z) == Approx(2.63));
        REQUIRE(contact->object_below + 1 < object.layer_count());
        CHECK(object.layer(contact->object_below).unscaled_height() == Approx(0.1));
        CHECK(object.layer(contact->object_below).unscaled_print_z() !=
              Approx(params.first_layer_height + contact->object_below * 0.2));
        CHECK(contact->height_block == object.layer(contact->object_below + 1).scaled_height());
        CHECK(object.layer(contact->object_below + 1).unscaled_height() == Approx(0.1));
        CHECK(contact->height == object.layer(contact->object_below + 1).scaled_height());
        CHECK(contact->bridging);
    }

    // Base slabs tile the intervals between extremes (top-contact bottoms and bottom-contact tops).
    // Contacts and carved interfaces may overlap those spans in Z, but have separately bounded printable thicknesses.
    std::set<std::pair<coord_t, coord_t>> spans;
    bool object_aligned = false, displaced = false;
    for (const Common::GeneratorLayers *group : {&generated.support.base, &generated.support.top, &generated.support.interfaces, &generated.support.bottom})
        for (const Common::GeneratorLayer *slab : *group) {
            CHECK(slab->height >= params.min_height);
            CHECK(slab->height <= scale_to_layer_coord(params.max_support_height));
            if (group == &generated.support.base) {
                const bool inserted = spans.insert({slab->print_z - slab->height, slab->print_z}).second;
                CHECK(inserted);
                bool on_object = false;
                for (const Layer &layer : object.layers()) on_object = on_object || layer.scaled_print_z() == slab->print_z;
                object_aligned = object_aligned || on_object;
                displaced = displaced || !on_object;
                // Away from a contact endpoint, synchronized slabs end on object layers.
                bool near_contact = false;
                for (const Common::GeneratorLayer *top : generated.support.top)
                    near_contact = near_contact || std::abs(slab->print_z - top->print_z) <= scale_to_layer_coord(params.max_support_height);
                for (const Common::GeneratorLayer *bottom : generated.support.bottom)
                    near_contact = near_contact || std::abs(slab->print_z - bottom->print_z) <= scale_to_layer_coord(params.max_support_height);
                if (sync && !near_contact) CHECK(on_object);
            }
        }
    coord_t previous = 0;
    for (const std::pair<coord_t, coord_t> &span : spans) {
        CHECK(span.first == previous);
        previous = span.second;
    }
    for (const Common::GeneratorLayers *group : {&generated.support.top, &generated.support.bottom})
        for (const Common::GeneratorLayer *contact : *group) {
            const coord_t extreme = contact->extreme_z();
            bool boundary = extreme == 0;
            for (const std::pair<coord_t, coord_t> &span : spans)
                boundary = boundary || span.first == extreme || span.second == extreme;
            CHECK(boundary);
        }
    CHECK(previous == generated.support.top.back()->bottom_z);
    if (sync) CHECK(object_aligned);
    else CHECK(displaced);

    const boost::filesystem::path path = Test::GCode::temporary_gcode_path();
    std::string code;
    try {
        print.process();
        Orchestrator::instance().export_gcode(print, path.string(), nullptr, nullptr);
        code = Test::GCode::read_text_file(path);
    } catch (...) {
        Test::GCode::remove_output_pair(path);
        throw;
    }
    Test::GCode::remove_output_pair(path);
    CHECK(code.find(";TYPE:Support material") != std::string::npos);
    // The pipeline publishes bridge bottom surfaces; toolpaths read their fill surfaces without consuming them.
    bool published_bottom_bridge = false;
    for (const Layer &layer : object.auxiliary_layers())
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands())
                for (const Surface &surface : region.fill_surfaces())
                    if (surface.surface_type == (stModSupport | stPosBottom | stDensSolid | stModBridge))
                        published_bottom_bridge = true;
    CHECK(published_bottom_bridge);
    for (size_t bay = 1; bay < 4; ++bay) {
        double highest = 0.;
        for (const Layer &layer : object.auxiliary_layers())
            for (const LayerSliceIsland &island : layer.islands())
                if (!intersection_ex(ExPolygons{island.get_slice()}, roof_footprints[bay]).empty())
                    highest = std::max(highest, layer.unscaled_print_z());
        const double expected = bay == 1 ? params.first_layer_height :
            unscaled(object.layer(roof_layers[bay]).scaled_bottom_z()) - params.gap_above;
        CAPTURE(bay, highest, expected);
        CHECK(highest == Approx(expected).margin(1e-4));
    }
}

// Slice a thin cantilever with nozzle-sized bridge threads, compare raw contact placement with generated contacts, and retain
// the lower contact when its ordinary neighbour is closer than min_height. Test-owned storage keeps the inspected slabs alive;
// slicing or generation failures propagate and missing bridge geometry fails the assertions.
TEST_CASE("Grid variable height filament bridge contacts retain or merge their neighbour", "[plugins][support-grid][variable-layer-height][bridge]")
{
    namespace Common = slic3r_api::Support::Common;
    const bool close_contacts = GENERATE(false, true);
    CAPTURE(close_contacts);
    DynamicPrintConfig config = SG::config();
    config.set_deserialize_strict({{"support_material_contact_distance_type", "filament"},
        {"support_material_contact_distance", "0.19"}, {"support_material_bottom_contact_distance", "0.23"}, {"bridge_type", "nozzle"},
        {"bridge_flow_ratio", close_contacts ? "4%" : "25%"}});
    TriangleMesh mesh = make_cube(2., 8., 4.);
    TriangleMesh roof = make_cube(6., 8., 0.05);
    roof.translate(0.f, 0.f, 4.f);
    mesh.merge(roof);
    Print print;
    Model model;
    SG::init_print_with_profile(print, model, std::move(mesh),
        std::vector<coordf_t>{0., 0.2, 3.999, 0.2, 4., 0.05, 4.05, 0.05}, config);
    REQUIRE(print.validate().first == PrintBase::PrintValidationError::pveNone);
    SG::prefix(print);
    const PrintObject &object = print.object(0);
    const Common::SupportParams params = SG::params(print, object);
    REQUIRE(params.gap_above == Approx(0.19));
    REQUIRE(params.gap_below == Approx(0.23));
    REQUIRE(params.distance_type == Common::ContactDistanceType::Filament);
    std::vector<slic3r_api::Layer> layers;
    const slic3r_api::Object view = SG::object_view(object);
    size_t roof_id = size_t(-1);
    for (uint32_t id = 0; id < view.layer_count(); ++id) {
        layers.push_back(view.layer(id));
        if (object.layer(id).unscaled_height() < 0.1) roof_id = id;
    }
    REQUIRE(roof_id != size_t(-1));
    const slic3r_api::Layer &above = layers[roof_id];
    REQUIRE(unscaled(above.height()) == Approx(0.05));
    REQUIRE(above.region_count() == 1);
    const double bridge_height = Common::bridging_height_avg_mm(SG::print_view(print).config(), above, above.region(0));
    REQUIRE(bridge_height == Approx(close_contacts ? 0.08 : 0.2).margin(1e-4));
    REQUIRE(bridge_height != Approx(unscaled(above.height())));
    const coord_t bridge_z = above.print_z() - scale_to_layer_coord(bridge_height) - scale_to_layer_coord(0.19);
    const coord_t ordinary_z = above.bottom_z() - scale_to_layer_coord(0.19);
    PluginStorage storage;
    Common::GeneratedSupport raw{SG::handle(storage)};
    const Common::TopContactSlabs contacts = Common::add_top_contacts(layers, SG::print_view(print).config(), params, roof_id, raw);
    REQUIRE(contacts.contact != nullptr);
    REQUIRE(contacts.bridge != nullptr);
    CHECK(contacts.contact->print_z == ordinary_z);
    CHECK(contacts.bridge->print_z == bridge_z);
    REQUIRE(bridge_z < ordinary_z);
    INFO("bridge height " << bridge_height << " bridge contact " << unscaled(bridge_z)
         << " ordinary contact " << unscaled(ordinary_z) << " minimum " << unscaled(params.min_height));
    if (close_contacts) REQUIRE(ordinary_z - bridge_z < params.min_height);
    else REQUIRE(ordinary_z - bridge_z > params.min_height);
    const SG::Generated generated(print, object);
    REQUIRE(generated.support.top.size() == (close_contacts ? 1 : 2));
    CHECK(generated.support.top.front()->print_z == bridge_z);
    CHECK_FALSE(generated.support.top.front()->polygons.empty());
    // A probe just inside the bridge-thread band obtains its footprint without including the ordinary roof contact band.
    Common::GeneratorLayer &probe = Common::add_layer(raw, Common::LayerType::TopContact, bridge_z + scale_to_layer_coord(0.001),
        bridge_z + scale_to_layer_coord(0.001) - scale_to_layer_coord(0.2), scale_to_layer_coord(0.2));
    const slic3r_api::StoredExPolygonCollection threads = Common::bridge_footprint(raw.storage, layers,
        SG::print_view(print).config(), params, probe.print_z, probe);
    REQUIRE_FALSE(threads.empty());
    CHECK(area(intersection_ex(SG::native(generated.support.top.front()->polygons), to_polygons(SG::native(threads)))) > 0.);
    if (close_contacts) {
        // merge_contacts resets the upper ordinary contact and retains the lower contact under the bridge threads.
        size_t surviving = 0, reset = 0;
        for (const Common::GeneratorLayer &slab : generated.support.slabs) {
            if (slab.object_above == roof_id && slab.type == Common::LayerType::TopContact) ++surviving;
            if (slab.type == Common::LayerType::Intermediate && slab.print_z == 0 && slab.object_above == size_t(-1)) ++reset;
        }
        CHECK(surviving == 1);
        CHECK(reset == 1);
    } else CHECK(generated.support.top.back()->print_z == ordinary_z);
}

// Dispatch sliced slabs with real or cleared demand: every style with nonempty demand is published by its family, and cleared demand publishes nothing.
TEST_CASE("Support families publish each style with demand and skip empty demand", "[plugins][support-grid]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *style = GENERATE("grid", "snug", "organic");
    const bool empty = GENERATE(false, true);
    CAPTURE(style, empty);
    Print print;
    Model model;
    Test::init_print({SG::slab()}, print, model, SG::config(style));
    SG::prefix(print);
    size_t demand_polygons = 0;
    for (const Layer &layer : print.object(0).layers())
        for (const LayerSliceIsland &island : layer.islands()) demand_polygons += island.support_demand().size();
    CAPTURE(demand_polygons);
    REQUIRE(Orchestrator::instance().is_plugin_active("support.family.grid"));
    const Plugin &provider = *Orchestrator::instance().get_plugin("support.family.grid");
    for (const char *expected_key : {"bridge_type", "infill_extrusion_width", "infill_extrusion_spacing",
            "infill_extrusion_change_odd_layers", "solid_infill_extruder", "first_layer_infill_extrusion_width",
            "first_layer_infill_extrusion_spacing", "perimeters", "fill_density", "top_solid_layers",
            "bottom_solid_layers", "solid_infill_every_layers"}) {
        CAPTURE(expected_key);
        CHECK(std::any_of(provider.get_used_config_keys().begin(), provider.get_used_config_keys().end(),
            [expected_key](const Plugin::UsedConfigKey &key) { return key.key == expected_key; }));
    }
    if (empty) {
        plugin_run_context ctx{};
        ctx.step = STEP_SUPPORT_DEMAND;
        for (Layer &layer : print.object(0).layers())
            for (LayerSliceIsland &island : layer.islands())
                expolygons_clear(support_demand_get_island_mutable(&ctx, reinterpret_cast<const layer_island_handle *>(&island)));
    }
    SG::family(print);
    const bool expected = !empty;
    CHECK(print.object(0).auxiliary_layers().empty() == !expected);
    for (const Layer &layer : print.object(0).auxiliary_layers())
        CHECK(layer.get_property<LayerSupportProperty>() != nullptr);
}

// Slice a roof above a platform with both contact materials and styles, then check clipped surface positions, tool ownership and empty extrusion buckets.
TEST_CASE("Grid family publishes top bottom and base surfaces without extrusions", "[plugins][support-grid]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool soluble = GENERATE(false, true);
    const char *style = GENERATE("grid", "snug");
    Print print;
    Model model;
    Test::init_print({SG::slab(true)}, print, model, SG::config(style, soluble));
    SG::prefix(print);
    SG::family(print);
    REQUIRE_FALSE(print.object(0).auxiliary_layers().empty());
    bool top = false, bottom = false, sparse = false;
    const SurfaceType top_type = stModSupport | stPosTop | stDensSolid;
    const SurfaceType bottom_type = stModSupport | stPosBottom | stDensSolid;
    const SurfaceType base_type = stModSupport | stPosInternal | stDensSparse;
    for (const Layer &layer : print.object(0).auxiliary_layers()) {
        REQUIRE(layer.get_property<LayerSupportProperty>() != nullptr);
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands()) {
                CHECK_FALSE(region.has_extrusion(LayerRegionIsland::SUPPORT));
                CHECK_FALSE(region.has_extrusion(LayerRegionIsland::SUPPORT_INTERFACE));
                for (const Surface &surface : region.fill_surfaces()) {
                    const SurfaceType type = surface.surface_type;
                    CHECK((type == top_type || type == bottom_type || type == (bottom_type | stModBridge) || type == base_type));
                    CHECK(diff_ex(ExPolygons{surface.expolygon}, ExPolygons{island.get_slice()}).empty());
                    if (type == top_type) { top = true; CHECK(region.extruder_id() == 1); }
                    if (type == bottom_type || type == (bottom_type | stModBridge)) { bottom = true; CHECK(region.extruder_id() == 1); }
                    if (type == base_type) { sparse = true; CHECK(region.extruder_id() == 0); }
                    if (type == (bottom_type | stModBridge)) CHECK_FALSE(soluble);
                }
            }
    }
    CHECK(top);
    CHECK(bottom);
    CHECK(sparse);
}

// Slice two roofs whose columns share an island and check separate work regions for different tools or one unknown-tool region when both settings are zero.
TEST_CASE("Grid family shares work regions only for equal support extruders", "[plugins][support-grid]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool unknown = GENERATE(false, true);
    DynamicPrintConfig config = SG::config();
    if (unknown) config.set_deserialize_strict({{"support_material_extruder", "0"}, {"support_material_interface_extruder", "0"}});
    Print print;
    Model model;
    // Two roofs at different heights put interface and sparse support into the same connected island.
    TriangleMesh mesh = SG::slab();
    TriangleMesh taller_post = make_cube(4., 8., 7.);
    mesh.merge(taller_post);
    TriangleMesh second = make_cube(16., 8., 1.);
    second.translate(0., 0., 6.);
    mesh.merge(second);
    Test::init_print({mesh}, print, model, config);
    SG::prefix(print);
    SG::family(print);
    bool mixed = false;
    for (const Layer &layer : print.object(0).auxiliary_layers())
        for (const LayerSliceIsland &island : layer.islands()) {
            if (unknown) {
                REQUIRE(island.regions_islands().size() == 1);
                CHECK(island.regions_island(0).extruder_id() == EXTRUDER_ID_UNKNOWN);
            } else if (island.regions_islands().size() == 2) {
                mixed = true;
                CHECK(island.regions_island(0).extruder_id() != island.regions_island(1).extruder_id());
            }
        }
    if (!unknown) CHECK(mixed);
}

// Publish a slab with displaced contact gaps and optional synchronization, checking the highest dense top, bounded thicknesses and synchronized sparse Z positions.
TEST_CASE("Grid family layer heights and contact gaps respect object Z", "[plugins][support-grid]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool sync = GENERATE(false, true);
    const char *gap = GENERATE("0.17", "0.19", "0.23");
    DynamicPrintConfig config = SG::config();
    // A non-multiple contact gap exercises the displaced top contact under synchronization.
    config.set_deserialize_strict({{"support_material_synchronize_layers", sync ? "1" : "0"},
        {"support_material_contact_distance", gap}});
    Print print;
    Model model;
    Test::init_print({SG::slab()}, print, model, config);
    SG::prefix(print);
    SG::family(print);
    const PrintObject &object = print.object(0);
    REQUIRE_FALSE(object.auxiliary_layers().empty());
    const slic3r_api::Support::Common::SupportParams params = SG::params(print, object);
    CHECK(object.auxiliary_layers().front().unscaled_print_z() == Approx(params.first_layer_height));
    coord_t highest_top = 0;
    bool synchronized_base = false;
    for (const Layer &layer : object.auxiliary_layers()) {
        CHECK(layer.unscaled_height() >= unscaled(params.min_height) - EPSILON);
        CHECK(layer.unscaled_height() <= params.max_support_height + EPSILON);
        CHECK(scale_to_layer_coord(layer.unscaled_print_z()) == layer.scaled_print_z());
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands())
                for (const Surface &surface : region.fill_surfaces()) {
                    if (surface.surface_type == (stModSupport | stPosTop | stDensSolid)) {
                        // Contact and intermediate interface share a type; the highest dense top slab touches this single roof.
                        highest_top = std::max(highest_top, layer.scaled_print_z());
                        CHECK(layer.scaled_print_z() <= scale_to_layer_coord(4. - params.gap_above));
                    }
                    if (sync && surface.surface_type == (stModSupport | stPosInternal | stDensSparse) && layer.unscaled_print_z() < 3.) {
                        bool found = false;
                        for (const Layer &object_layer : object.layers()) found = found || object_layer.scaled_print_z() == layer.scaled_print_z();
                        CHECK(found);
                        synchronized_base = true;
                    }
                }
    }
    CHECK(highest_top == scale_to_layer_coord(4. - params.gap_above));
    if (sync) CHECK(synchronized_base);
}

// Publish support above a 0.8-mm platform and check that every bridge bottom contact keeps the requested 0.23-mm gap and printable thickness.
TEST_CASE("Synchronized grid bottom contacts retain separation above the object", "[plugins][support-grid]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = SG::config();
    config.set_deserialize_strict({{"support_material_synchronize_layers", "1"},
        {"support_material_bottom_contact_distance", "0.23"}});
    Print print;
    Model model;
    Test::init_print({SG::slab(true)}, print, model, config);
    SG::prefix(print);
    SG::family(print);
    bool found = false;
    for (const Layer &layer : print.object(0).auxiliary_layers())
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands())
                for (const Surface &surface : region.fill_surfaces())
                    if (surface.surface_type == (stModSupport | stPosBottom | stDensSolid | stModBridge)) {
                        found = true;
                        CHECK(layer.unscaled_print_z() - layer.unscaled_height() == Approx(0.8 + 0.23));
                        CHECK(layer.unscaled_height() >= 0.05 - EPSILON);
                        CHECK(layer.unscaled_height() <= 0.3 + EPSILON);
                    }
    CHECK(found);
}

// Limit dense thickness to 0.1 mm on a 0.01-mm machine grid and check inserted slab Z/thickness while retaining the 3.77-mm roof contact.
TEST_CASE("Synchronized grid inserts a bounded slab below a displaced top contact", "[plugins][support-grid]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *style = GENERATE("grid", "snug");
    DynamicPrintConfig config = SG::config(style);
    config.set_deserialize_strict({{"support_material_synchronize_layers", "1"},
        {"support_material_interface_layer_height", "0.1"}, {"support_material_contact_distance", "0.23"}, {"z_step", "0.01"}});
    Print print;
    Model model;
    Test::init_print({SG::slab()}, print, model, config);
    SG::prefix(print);
    SG::family(print);
    const PrintObject &object = print.object(0);
    const coord_t contact_z = scale_to_layer_coord(3.77);
    bool contact = false, inserted = false;
    for (const Layer &layer : object.auxiliary_layers()) {
        CHECK(layer.scaled_print_z() % scale_to_layer_coord(0.01) == 0);
        CHECK(layer.scaled_height() % scale_to_layer_coord(0.01) == 0);
        CHECK(layer.unscaled_height() >= 0.05 - EPSILON);
        CHECK(layer.unscaled_height() <= 0.3 + EPSILON);
        if (layer.scaled_print_z() == contact_z) {
            contact = true;
            CHECK(layer.unscaled_height() <= 0.1 + EPSILON);
        } else if (layer.unscaled_print_z() > 3.6 && layer.scaled_print_z() < contact_z) {
            // The last object Z is 3.6; splitting the remaining 0.17 interval creates a non-object slab before the contact.
            inserted = true;
            CHECK(layer.unscaled_print_z() == Approx(3.69));
            CHECK(layer.unscaled_height() == Approx(0.09));
            CHECK(layer.unscaled_height() <= 0.1 + EPSILON);
        }
    }
    CHECK(contact);
    CHECK(inserted);
}

// Align contacts separated by only 0.02 mm without moving the roof gap or platform gap, and check shared printed thickness for both synchronized styles.
TEST_CASE("Synchronized support shares close top and bottom contacts without a subminimum slab", "[plugins][support-grid]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *style = GENERATE("grid", "snug");
    const int interfaces = GENERATE(0, 3);
    CAPTURE(style, interfaces);
    DynamicPrintConfig config = SG::config(style, false, interfaces);
    config.set_deserialize_strict({{"support_material_synchronize_layers", "1"},
        {"support_material_bottom_contact_distance", "0.23"}, {"support_material_contact_distance", "0.35"}});
    TriangleMesh mesh = make_cube(4., 8., 1.6);
    mesh.merge(make_cube(12., 8., 0.8));
    TriangleMesh roof = make_cube(12., 8., 1.);
    roof.translate(0.f, 0.f, 1.6f);
    mesh.merge(roof);
    Print print;
    Model model;
    Test::init_print({mesh}, print, model, config);
    SG::prefix(print);
    SG::family(print);
    const PrintObject &object = print.object(0);
    const slic3r_api::Support::Common::SupportParams params = SG::params(print, object);
    const SG::Generated run(print, object);
    const slic3r_api::Support::Common::GeneratedSupport &generated = run.support;
    REQUIRE_FALSE(generated.top.empty());
    REQUIRE_FALSE(generated.bottom.empty());
    for (const slic3r_api::Support::Common::GeneratorLayer *top : generated.top)
        CHECK(top->print_z == scale_to_layer_coord(1.6 - 0.35));
    for (const slic3r_api::Support::Common::GeneratorLayer *bottom : generated.bottom) {
        CHECK(bottom->bottom_z == scale_to_layer_coord(0.8 + 0.23));
        CHECK(bottom->print_z == scale_to_layer_coord(1.25));
        CHECK(bottom->height == scale_to_layer_coord(0.22));
    }
    REQUIRE_FALSE(object.auxiliary_layers().empty());
    bool bottom_contact = false;
    for (const Layer &layer : object.auxiliary_layers()) {
        CHECK(layer.scaled_height() >= params.min_height);
        CHECK(layer.unscaled_height() <= params.max_support_height + EPSILON);
        CHECK(layer.scaled_print_z() <= scale_to_layer_coord(1.25));
        CHECK_FALSE(layer.has_extrusions());
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &work : island.regions_islands())
                for (const Surface &surface : work.fill_surfaces())
                    if (surface.surface_type == (stModSupport | stPosBottom | stDensSolid | stModBridge)) {
                        bottom_contact = true;
                        CHECK(layer.unscaled_print_z() == Approx(1.25));
                        CHECK(layer.unscaled_height() == Approx(0.22));
                        CHECK(layer.unscaled_print_z() - layer.unscaled_height() == Approx(0.8 + 0.23));
                    }
    }
    CHECK(bottom_contact);
}

// Process a rotated cube at two first-layer heights; check one bed layer, unique Z/thickness pairs and minimum spacing between distinct Z groups.
TEST_CASE("Grid first-layer and height limits cover the cube-with-hole support case", "[plugins][support-grid]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *first = GENERATE("0.3", "0.4");
    DynamicPrintConfig config = SG::config();
    config.set_deserialize_strict({{"first_layer_height", first}});
    TriangleMesh mesh = Test::mesh(Test::TestMesh::cube_with_hole);
    mesh.rotate_x(float(M_PI / 2));
    Print print;
    Model model;
    Test::init_print({mesh}, print, model, config);
    print.process();
    const LayerRefs support = print.object(0).auxiliary_layers();
    REQUIRE_FALSE(support.empty());
    CHECK(support.front().unscaled_print_z() == Approx(std::stod(first)));
    std::set<size_t> ids;
    std::set<std::pair<coord_t, coord_t>> slabs;
    size_t first_layers = 0;
    for (const Layer &layer : support) {
        CHECK(ids.insert(layer.id()).second);
        CHECK(slabs.insert({layer.scaled_print_z(), layer.scaled_height()}).second);
        if (layer.scaled_print_z() == support.front().scaled_print_z()) {
            ++first_layers;
            CHECK(layer.unscaled_height() == Approx(std::stod(first)));
        }
    }
    CHECK(first_layers == 1);
    size_t shared_z = 0;
    for (size_t i = 1; i < support.size(); ++i) {
        const double distance = support[i].unscaled_print_z() - support[i - 1].unscaled_print_z();
        if (support[i].scaled_print_z() == support[i - 1].scaled_print_z()) {
            // Layers of one print Z keep the native order: the thickest layer first.
            ++shared_z;
            CHECK(support[i].scaled_height() != support[i - 1].scaled_height());
            CHECK(support[i - 1].scaled_height() > support[i].scaled_height());
        }
        else CHECK(distance >= 0.05 - EPSILON);
        CHECK(distance <= 0.4 + EPSILON);
    }
    CHECK(shared_z > 0);
}

// The support ignores the brim and the skirt: with a brim_width of 2, a brim_width_interior of 1 and a skirt, every slab of every list, floors included,
// equals the slab of the same print without a brim (same print Z, type and area), and the first layer holds support.
TEST_CASE("Grid first layer slabs ignore the brim", "[plugins][support-grid][brim]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *style = GENERATE("grid", "snug");
    const int interfaces = GENERATE(0, 3);
    CAPTURE(style, interfaces);
    // Slice a print of the slab with the given brim widths and generate its grid slabs.
    struct Run {
        Print print;
        Model model;
        std::unique_ptr<SG::Generated> generated;
        Run(const char *style, int interfaces, const char *brim_width, const char *interior, const char *skirts)
        {
            DynamicPrintConfig config = SG::config(style, false, interfaces);
            config.set_deserialize_strict({{"brim_width", brim_width}, {"brim_width_interior", interior},
                                           {"skirts", skirts}, {"skirt_distance", "1"}});
            Test::init_print({SG::slab()}, print, model, config);
            SG::prefix(print);
            generated = std::make_unique<SG::Generated>(print, print.object(0));
        }
    };
    const Run plain(style, interfaces, "0", "0", "0"), brimmed(style, interfaces, "2", "1", "1");
    // Only the second print has a brim and a skirt.
    REQUIRE_FALSE(plain.print.object(0).has_brim());
    REQUIRE(brimmed.print.object(0).has_brim());
    const slic3r_api::Support::Common::GeneratedSupport &before = plain.generated->support, &after = brimmed.generated->support;
    const std::vector<std::pair<const slic3r_api::Support::Common::GeneratorLayers *, const slic3r_api::Support::Common::GeneratorLayers *>> lists = {
        {&before.top, &after.top}, {&before.base, &after.base}, {&before.interfaces, &after.interfaces}, {&before.bottom, &after.bottom}};
    // The plain print has the slabs that the brim used to cut.
    REQUIRE_FALSE(before.top.empty());
    REQUIRE_FALSE(before.base.empty());
    if (interfaces > 0)
        REQUIRE_FALSE(before.interfaces.empty());
    const coord_t first = scale_to_layer_coord(SG::params(plain.print, plain.print.object(0)).first_layer_height);
    double first_layer_mm2 = 0.;
    for (const std::pair<const slic3r_api::Support::Common::GeneratorLayers *, const slic3r_api::Support::Common::GeneratorLayers *> &list : lists) {
        REQUIRE(list.first->size() == list.second->size());
        for (size_t i = 0; i < list.first->size(); ++i) {
            const slic3r_api::Support::Common::GeneratorLayer &a = *(*list.first)[i], &b = *(*list.second)[i];
            const Polygons &polygons_a = SG::native(a.polygons), &polygons_b = SG::native(b.polygons);
            CHECK(a.print_z == b.print_z);
            CHECK(a.bottom_z == b.bottom_z);
            CHECK(a.height == b.height);
            CHECK(a.bridging == b.bridging);
            CHECK(a.type == b.type);
            CHECK(Slic3r::Test::PolygonMetrics::sym_diff_mm2(polygons_a, polygons_b) < 1e-4);
            if (a.print_z <= first)
                first_layer_mm2 += unscaled(unscaled(area(union_ex(polygons_b))));
        }
    }
    // The case is not vacuous: the first layer holds support.
    CHECK(first_layer_mm2 > 0.);
}

// The family gives every area of one auxiliary layer a single owner: a lower surface loses the
// areas an upper surface already prints, and the base loses the areas any interface covers.
TEST_CASE("Grid family resolves the top and bottom overlap of one layer", "[plugins][support-grid]")
{
    // Two 4x4-mm squares overlapping over half their width, and a base square crossing both.
    const Polygons top{Polygon{Point(0, 0), Point(scale_i(4.), 0), Point(scale_i(4.), scale_i(4.)), Point(0, scale_i(4.))}};
    const Polygons bottom{Polygon{Point(scale_i(2.), 0), Point(scale_i(6.), 0), Point(scale_i(6.), scale_i(4.)), Point(scale_i(2.), scale_i(4.))}};
    const Polygons sparse{Polygon{Point(scale_i(5.), 0), Point(scale_i(7.), 0), Point(scale_i(7.), scale_i(4.)), Point(scale_i(5.), scale_i(4.))}};
    const Polygons shared = intersection(top, bottom);
    REQUIRE(!shared.empty());
    // The family works on API collections: copy the native polygons into plugin storage.
    PluginStorage storage;
    const auto stored = [&storage](const Polygons &polygons) {
        slic3r_api::StoredPolygonCollection out(reinterpret_cast<storage_handle *>(&storage));
        polygons_copy(out.mutable_handle(), reinterpret_cast<const polygon_collection_handle *>(&polygons));
        return out;
    };
    const Polygons resolved_bottom = SG::native(slic3r_api::Support::Common::Internal::resolve_overlap(stored(bottom), stored(top)));
    // The shared area stays on the top surface only, and the rest of the bottom area is kept.
    CHECK(intersection(resolved_bottom, top).empty());
    CHECK(area(resolved_bottom) == Approx(area(bottom) - area(shared)));
    const Polygons interfaces = union_(top, resolved_bottom);
    const Polygons resolved_sparse = SG::native(slic3r_api::Support::Common::Internal::resolve_overlap(stored(sparse), stored(interfaces)));
    // The base keeps only the area no interface covers, so the three kinds are disjoint.
    CHECK(intersection(resolved_sparse, interfaces).empty());
    CHECK(area(resolved_sparse) == Approx(area(sparse) - area(intersection(sparse, interfaces))));
    CHECK(!resolved_sparse.empty());
}

// The family types a contact as base exactly when the native generator would have merged it into the
// base: one extruder, no bridging contact, and the merged base layer of that print Z, whose thickness
// is the smallest of that Z, has the contact's thickness or does not exist.
TEST_CASE("Grid family types a mergeable contact as base", "[plugins][support-grid]")
{
    using slic3r_api::Support::Common::Internal::merged_into_base;
    CHECK_FALSE(merged_into_base(false, true, true, false));  // Separate extruders never merge.
    CHECK(merged_into_base(true, true, true, false));         // Same layer, same thickness, same flag.
    CHECK_FALSE(merged_into_base(true, true, true, true));    // A bridging contact never merges.
    CHECK_FALSE(merged_into_base(true, false, true, false));  // A base elsewhere at this print Z.
    CHECK(merged_into_base(true, false, false, false));       // An empty base takes the contact.
    CHECK_FALSE(merged_into_base(true, false, false, true));  // An empty base never takes a bridge.
}

// Dispatch a roof with a requested raft and count family warning messages while requiring geometric support, without expecting raft layers.
TEST_CASE("Grid family ignores raft generation but emits one warning", "[plugins][support-grid]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = SG::config();
    config.set_deserialize_strict({{"raft_layers", "3"}});
    Print print;
    Model model;
    Test::init_print({SG::slab()}, print, model, config);
    SG::prefix(print);
    Orchestrator::instance().consume_plugin_messages();
    SG::family(print);
    REQUIRE_FALSE(print.object(0).auxiliary_layers().empty());
    size_t warnings = 0;
    for (const Orchestrator::PluginMessage &message : Orchestrator::instance().consume_plugin_messages())
        if (message.plugin_id == "support.family.grid" && message.level == Orchestrator::PluginMessageLevel::Warning) ++warnings;
    CHECK(warnings == 1);
}
