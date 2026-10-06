#include "support_grid_test_helpers.hpp"
#include "libslic3r/Plugins/Support/Grid/GridSupportParams.hpp"
#include "libslic3r/Plugins/Support/Grid/GridSupportGenerator.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_support_demand.h"

/* Grid-family tests inspect geometric output before toolpath generation. A
   slab on a post needs bed-reaching columns; a low platform also needs bottom
   contacts. Surface position bits encode which side of the object is supported.
   These tests exercise the real STEP_SUPPORT dispatcher and API publication. */

using namespace Slic3r;
namespace SG = Slic3r::Test::SupportGrid;

// Dispatch sliced slabs with real or cleared demand and check that only grid/snug with nonempty demand publish owned auxiliary layers.
TEST_CASE("Grid family filters styles and empty demand", "[plugins][support-grid]")
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
    const bool expected = !empty && std::string(style) != "organic";
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
    const Slic3r::SupportGrid::GridSupportParams params(object);
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
    const Slic3r::SupportGrid::GridSupportParams params(object);
    Slic3r::SupportGrid::GeneratedSupport generated;
    Slic3r::SupportGrid::generate_support(object, params, generated);
    REQUIRE_FALSE(generated.top.empty());
    REQUIRE_FALSE(generated.bottom.empty());
    for (const Slic3r::SupportGrid::GeneratorLayer *top : generated.top)
        CHECK(top->print_z == scale_to_layer_coord(1.6 - 0.35));
    for (const Slic3r::SupportGrid::GeneratorLayer *bottom : generated.bottom) {
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

// Process a rotated cube with a hole at two first-layer heights and check nonempty support, bed contact Z and spacing between published slabs.
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
    for (size_t i = 1; i < support.size(); ++i) {
        const double distance = support[i].unscaled_print_z() - support[i - 1].unscaled_print_z();
        CHECK(distance >= 0.05 - EPSILON);
        CHECK(distance <= 0.4 + EPSILON);
    }
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
