/* LayerRegion flow tests compare the C API with the native support helpers.
   A first auxiliary layer can touch the bed or start higher: only the bed
   contact uses first-layer width. Support base and interface use separate
   nozzle settings, and bridge contacts use layer height as their diameter.
   Invalid support extruder ids exercise the C boundary's zero-flow result. */
#include <catch2/catch.hpp>

#include "plugin_test_helpers.hpp"
#include "test_data.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/c/slic3r_data_tree.h"
#include "libslic3r/Flow.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Steps/StepLayerHeightGeneration.hpp"
#include "libslic3r/Steps/StepSlicing.hpp"
#include "libslic3r/TriangleMesh.hpp"

namespace {
using namespace Slic3r;

// Compare every flow field after converting native lengths to the API's scaled coordinates.
void check_flow(const c_flow &actual, const Flow &expected)
{
    CHECK(actual.width == expected.scaled_width());
    CHECK(actual.spacing == expected.scaled_spacing());
    CHECK(actual.height == expected.scaled_height());
    CHECK(actual.nozzle_diameter == scale_i(expected.nozzle_diameter()));
    CHECK(actual.is_bridge == (expected.bridge() ? 1 : 0));
    CHECK(actual.spacing_ratio == Approx(expected.spacing_ratio()));
    CHECK(actual.mm3_per_mm == Approx(expected.mm3_per_mm()));
}
} // namespace

TEST_CASE("Layer region support flows match native base interface first-layer and bridge flows", "[plugins][support-flow]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool bed_contact = GENERATE(false, true);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"nozzle_diameter", "0.4,0.6"}, {"support_material_extruder", "1"},
                                   {"support_material_interface_extruder", "2"},
                                   {"support_material_extrusion_width", "0.5"}, {"first_layer_extrusion_width", "0.7"}});
    Print print;
    Model model;
    Test::init_print({make_cube(4., 4., 0.4)}, print, model, config);
    Orchestrator &orchestrator = Orchestrator::instance();
    Steps::StepLayerHeightGeneration::run_step(orchestrator, print);
    Steps::StepSlicing::run_step(orchestrator, print);
    PrintObject &object = print.object(0);
    const coord_t height = scale_i(bed_contact ? 0.2 : 0.17);
    layer_handle *layer = object_add_auxiliary_layer(reinterpret_cast<const object_handle *>(&object), height,
                                                    bed_contact ? height : scale_i(0.8), scale_i(0.1));
    REQUIRE(layer != nullptr);
    const layer_region_handle *region = layer_get_region(layer, 0);
    REQUIRE(region != nullptr);
    CHECK(reinterpret_cast<const Layer *>(layer)->id() == 0);
    const float unscaled_height = float(unscaled(height));
    for (raw_extrusion_role role : {raw_extrusion_role(RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL),
                                    raw_extrusion_role(RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL_INTERFACE)}) {
        const Flow normal = role == RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL ? support_material_flow(&object, unscaled_height) :
                                                                         support_material_interface_flow(&object, unscaled_height);
        const Flow expected = bed_contact ? support_material_1st_layer_flow(&object, unscaled_height) : normal;
        REQUIRE_NOTHROW(layer_region_get_flow(region, role));
        check_flow(layer_region_get_flow(region, role), expected);
        REQUIRE_NOTHROW(layer_region_get_bridging_flow(region, role));
        check_flow(layer_region_get_bridging_flow(region, role), Flow::bridging_flow(unscaled_height, normal.nozzle_diameter()));
    }

    // Ordinary regional flow and the infill fallback keep their native calculation.
    const LayerRegion &native_region = object.layer(0).region(0);
    const layer_region_handle *native_handle = reinterpret_cast<const layer_region_handle *>(&native_region);
    check_flow(layer_region_get_flow(native_handle, RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER), native_region.flow(frExternalPerimeter));
    check_flow(layer_region_get_bridging_flow(native_handle, RAW_EXTRUSION_ROLE_INFILL), native_region.bridging_flow(frInfill));
    check_flow(layer_region_get_flow(native_handle, RAW_EXTRUSION_ROLE_NONE), native_region.flow(frInfill));
}

TEST_CASE("Uncalculable layer-region support flows return zero without crossing the C boundary", "[plugins][support-flow]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    Test::init_print({make_cube(4., 4., 0.4)}, print, model);
    Orchestrator &orchestrator = Orchestrator::instance();
    Steps::StepLayerHeightGeneration::run_step(orchestrator, print);
    Steps::StepSlicing::run_step(orchestrator, print);
    PrintObject &object = print.object(0);
    PrintObjectConfig &object_config = const_cast<PrintObjectConfig &>(object.config());
    object_config.support_material_extruder.value = 32769;
    object_config.support_material_interface_extruder.value = 32769;
    REQUIRE_THROWS_AS(support_material_flow(&object, 0.2f), InvalidArgument);
    REQUIRE_THROWS_AS(support_material_interface_flow(&object, 0.2f), InvalidArgument);
    const layer_region_handle *region = reinterpret_cast<const layer_region_handle *>(&object.layer(0).region(0));
    for (const layer_region_handle *handle : {region, static_cast<const layer_region_handle *>(nullptr)})
        for (raw_extrusion_role role : {raw_extrusion_role(RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL),
                                        raw_extrusion_role(RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL_INTERFACE)}) {
            REQUIRE_NOTHROW(layer_region_get_flow(handle, role));
            REQUIRE_NOTHROW(layer_region_get_bridging_flow(handle, role));
            for (const c_flow &flow : {layer_region_get_flow(handle, role), layer_region_get_bridging_flow(handle, role)}) {
                CHECK(flow.width == 0);
                CHECK(flow.spacing == 0);
                CHECK(flow.height == 0);
                CHECK(flow.nozzle_diameter == 0);
                CHECK(flow.is_bridge == 0);
                CHECK(flow.spacing_ratio == 0);
                CHECK(flow.mm3_per_mm == 0);
            }
        }
}
