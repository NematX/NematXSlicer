/* LayerRegion flow tests compare the C API with the native support helpers.
   A first auxiliary layer can touch the bed or start higher: only the bed
   contact uses first-layer width, and a later auxiliary layer at the bed does
   too because the rule reads the bottom, not the id. Support base and
   interface use separate nozzle settings, and bridge contacts use layer height
   as their diameter. Invalid support extruder ids exercise the C boundary's
   zero-flow result.
   The object-level flows (object_get_support_flow, object_get_region_flow) are
   compared with the native calls of the legacy grid support at several
   heights, soluble or not. A modifier volume gives a second region, to check
   that the bottom contact uses the bridge flow ratio averaged over regions.
   The millimeter variants (object_get_support_flow_mm, object_get_region_flow_mm)
   must give the native float values exactly, without the scaled truncation. */
#include <catch2/catch.hpp>

#include <cmath>

#include "plugin_test_helpers.hpp"
#include "test_data.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/c/slic3r_data_tree.h"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/PrintRegion.hpp"
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

// Compare the millimeter fields with the native float values, exactly.
void check_flow_mm(const c_flow_mm &actual, const Flow &expected)
{
    CHECK(actual.width == double(expected.width()));
    CHECK(actual.spacing == double(expected.spacing()));
    CHECK(actual.height == double(expected.height()));
    CHECK(actual.nozzle_diameter == double(expected.nozzle_diameter()));
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

TEST_CASE("A later auxiliary layer at the bed uses the first-layer support flow", "[plugins][support-flow]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"support_material_extrusion_width", "0.5"}, {"first_layer_extrusion_width", "0.7"}});
    Print print;
    Model model;
    Test::init_print({make_cube(4., 4., 0.4)}, print, model, config);
    Orchestrator &orchestrator = Orchestrator::instance();
    Steps::StepLayerHeightGeneration::run_step(orchestrator, print);
    Steps::StepSlicing::run_step(orchestrator, print);
    PrintObject &object = print.object(0);
    const object_handle *object_handle_value = reinterpret_cast<const object_handle *>(&object);
    const coord_t height = scale_i(0.2);
    // Created first but above the bed, then created second but at the bed.
    REQUIRE(object_add_auxiliary_layer(object_handle_value, height, scale_i(0.8), scale_i(0.7)) != nullptr);
    layer_handle *bed_layer = object_add_auxiliary_layer(object_handle_value, height, height, scale_i(0.1));
    REQUIRE(bed_layer != nullptr);
    CHECK(reinterpret_cast<const Layer *>(bed_layer)->id() == 1);
    const layer_region_handle *region = layer_get_region(bed_layer, 0);
    REQUIRE(region != nullptr);
    check_flow(layer_region_get_flow(region, RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL),
               support_material_1st_layer_flow(&object, 0.2f));
}

TEST_CASE("Object support and region flows match the native flows at any height", "[plugins][support-flow]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool soluble = GENERATE(false, true);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"nozzle_diameter", "0.4,0.6"}, {"support_material_extruder", "1"},
                                   {"support_material_interface_extruder", "2"},
                                   {"support_material_extrusion_width", "0.5"}, {"first_layer_extrusion_width", "0.7"},
                                   {"external_perimeter_extrusion_change_odd_layers", "110%"},
                                   {"support_material_contact_distance_type", soluble ? "none" : "plane"}});
    config.set_deserialize_strict({{"bridge_flow_ratio", "80%"}});
    Print print;
    Model model;
    Test::init_print({make_cube(4., 4., 0.4)}, print, model, config);
    Orchestrator &orchestrator = Orchestrator::instance();
    Steps::StepLayerHeightGeneration::run_step(orchestrator, print);
    Steps::StepSlicing::run_step(orchestrator, print);
    const PrintObject &object = print.object(0);
    const object_handle *handle = reinterpret_cast<const object_handle *>(&object);
    const raw_extrusion_role base_role = RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL;
    const raw_extrusion_role interface_role = RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL_INTERFACE;
    const double bridge_ratio = object.printing_region(0).config().bridge_flow_ratio.get_effective_value(1.);
    REQUIRE(object.num_printing_regions() == 1);
    REQUIRE(bridge_ratio == Approx(0.8));

    for (const double height : {0., 0.15, 0.2, 0.3}) {
        INFO("height " << height);
        const float native_height = float(height);
        const Flow base_flow = support_material_flow(&object, native_height);
        const Flow interface_flow = support_material_interface_flow(&object, native_height);
        check_flow(object_get_support_flow(handle, base_role, height, 0., 0), base_flow);
        check_flow(object_get_support_flow(handle, interface_role, height, 0., 0), interface_flow);
        check_flow(object_get_support_flow(handle, base_role, height, 0., OBJECT_SUPPORT_FLOW_FIRST_LAYER),
                   support_material_1st_layer_flow(&object, native_height));
        // Bottom contact, as the native support geometry derives it from the interface flow.
        const Flow bottom = soluble ? interface_flow.with_flow_ratio(bridge_ratio) :
            Flow::bridging_flow(std::sqrt(bridge_ratio) * interface_flow.nozzle_diameter(), interface_flow.nozzle_diameter());
        check_flow(object_get_support_flow(handle, interface_role, height, 0., OBJECT_SUPPORT_FLOW_BOTTOM_CONTACT), bottom);
        check_flow_mm(object_get_support_flow_mm(handle, base_role, height, 0., 0), base_flow);
        check_flow_mm(object_get_support_flow_mm(handle, interface_role, height, 0., 0), interface_flow);
        check_flow_mm(object_get_support_flow_mm(handle, interface_role, height, 0., OBJECT_SUPPORT_FLOW_BOTTOM_CONTACT), bottom);
        if (height <= 0.)
            continue;
        // A bridge thread at the printed height and a nominal flow printed at another height.
        check_flow(object_get_support_flow(handle, interface_role, 0.2, height, OBJECT_SUPPORT_FLOW_BRIDGE),
                   Flow::bridging_flow(native_height, support_material_interface_flow(&object, 0.2f).nozzle_diameter()));
        check_flow(object_get_support_flow(handle, base_role, 0.2, height, 0), support_material_flow(&object, 0.2f).with_height(native_height));
        if (soluble)
            check_flow(object_get_support_flow(handle, interface_role, 0.2, height, OBJECT_SUPPORT_FLOW_BOTTOM_CONTACT),
                       support_material_interface_flow(&object, 0.2f).with_flow_ratio(bridge_ratio).with_height(native_height));
        // The external perimeter width the grid support reads with layer id 2, and the first and odd layer rules.
        for (const uint32_t layer_id : {0u, 1u, 2u})
            check_flow(object_get_region_flow(handle, 0, RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER, height, layer_id),
                       object.printing_region(0).flow(object, frExternalPerimeter, height, layer_id));
        check_flow(object_get_region_flow(handle, 0, RAW_EXTRUSION_ROLE_INFILL, height, 2),
                   object.printing_region(0).flow(object, frInfill, height, 2));
        check_flow_mm(object_get_region_flow_mm(handle, 0, RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER, height, 2),
                      object.printing_region(0).flow(object, frExternalPerimeter, height, 2));
    }

    // Inputs the host cannot turn into a flow give a zero flow without throwing.
    const c_flow zero_flows[] = {
        object_get_support_flow(nullptr, base_role, 0.2, 0., 0),
        object_get_support_flow(handle, RAW_EXTRUSION_ROLE_INFILL, 0.2, 0., 0),
        object_get_support_flow(handle, base_role, 0.2, 0., OBJECT_SUPPORT_FLOW_BRIDGE),
        object_get_region_flow(nullptr, 0, RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER, 0.2, 2),
        object_get_region_flow(handle, 1, RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER, 0.2, 2),
        object_get_region_flow(handle, 0, base_role, 0.2, 2),
        object_get_region_flow(handle, 0, RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER, 0., 2),
    };
    for (const c_flow &flow : zero_flows) {
        CHECK(flow.width == 0);
        CHECK(flow.spacing == 0);
        CHECK(flow.height == 0);
        CHECK(flow.nozzle_diameter == 0);
        CHECK(flow.mm3_per_mm == 0);
    }
    const c_flow_mm zero_flows_mm[] = {
        object_get_support_flow_mm(handle, RAW_EXTRUSION_ROLE_INFILL, 0.2, 0., 0),
        object_get_region_flow_mm(handle, 1, RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER, 0.2, 2),
    };
    for (const c_flow_mm &flow : zero_flows_mm) {
        CHECK(flow.width == 0);
        CHECK(flow.spacing == 0);
        CHECK(flow.height == 0);
        CHECK(flow.nozzle_diameter == 0);
    }
}

TEST_CASE("Object bottom-contact support flow averages the bridge flow ratio of every region", "[plugins][support-flow]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool soluble = GENERATE(false, true);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"nozzle_diameter", "0.4,0.6"}, {"support_material_extruder", "1"},
                                   {"support_material_interface_extruder", "2"},
                                   {"support_material_extrusion_width", "0.5"}, {"bridge_flow_ratio", "80%"},
                                   {"support_material_contact_distance_type", soluble ? "none" : "plane"}});
    // A modifier volume over part of the cube gives a second printing region with its own bridge flow ratio.
    Print print;
    Model model;
    ModelObject *model_object = model.add_object();
    model_object->add_volume(make_cube(20., 20., 2.), ModelVolumeType::MODEL_PART, false)->set_type(ModelVolumeType::MODEL_PART);
    TriangleMesh modifier_mesh = make_cube(5., 20., 2.);
    modifier_mesh.translate(Vec3f(15.f, 0.f, 0.f));
    ModelVolume *modifier = model_object->add_volume(std::move(modifier_mesh), ModelVolumeType::PARAMETER_MODIFIER, false);
    modifier->set_type(ModelVolumeType::PARAMETER_MODIFIER);
    DynamicPrintConfig modifier_config;
    modifier_config.set_deserialize_strict({{"bridge_flow_ratio", "50%"}});
    modifier->config.assign_config(modifier_config);
    model_object->add_instance();
    model.center_instances_around_point({100., 100.});
    model_object->ensure_on_bed();
    print.auto_assign_extruders(model_object);
    print.apply(model, config);
    print.validate();
    print.set_status_silent();
    Orchestrator &orchestrator = Orchestrator::instance();
    Steps::StepLayerHeightGeneration::run_step(orchestrator, print);
    Steps::StepSlicing::run_step(orchestrator, print);

    const PrintObject &object = print.object(0);
    REQUIRE(object.num_printing_regions() == 2);
    const double first_ratio = object.printing_region(0).config().bridge_flow_ratio.get_effective_value(1.);
    const double second_ratio = object.printing_region(1).config().bridge_flow_ratio.get_effective_value(1.);
    REQUIRE(first_ratio != Approx(second_ratio));
    const double average = (first_ratio + second_ratio) / 2.;
    const Flow interface_flow = support_material_interface_flow(&object, 0.2f);
    const Flow expected = soluble ? interface_flow.with_flow_ratio(average) :
        Flow::bridging_flow(std::sqrt(average) * interface_flow.nozzle_diameter(), interface_flow.nozzle_diameter());

    const object_handle *handle = reinterpret_cast<const object_handle *>(&object);
    check_flow(object_get_support_flow(handle, RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL_INTERFACE, 0.2, 0.,
                                       OBJECT_SUPPORT_FLOW_BOTTOM_CONTACT), expected);
    // The C++ view forwards the same request.
    check_flow(slic3r_api::Object(handle).support_flow(RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL_INTERFACE, 0.2, 0.,
                                                       OBJECT_SUPPORT_FLOW_BOTTOM_CONTACT), expected);
    check_flow_mm(slic3r_api::Object(handle).support_flow_mm(RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL_INTERFACE, 0.2, 0.,
                                                             OBJECT_SUPPORT_FLOW_BOTTOM_CONTACT), expected);
}

// A thin flow printed too high can have a positive spacing but a negative cross section: the host's
// Flow::mm3_per_mm() throws for it. The C functions must not let that exception out and return a
// zero flow, in both result forms.
TEST_CASE("Object flows without a positive cross section are zero flows", "[plugins][support-flow]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"external_perimeter_extrusion_width", "0.1"}, {"external_perimeter_overlap", "10%"},
                                   {"support_material_extrusion_width", "0.1"}});
    Print print;
    Model model;
    Test::init_print({make_cube(4., 4., 2.)}, print, model, config);
    Orchestrator &orchestrator = Orchestrator::instance();
    Steps::StepLayerHeightGeneration::run_step(orchestrator, print);
    Steps::StepSlicing::run_step(orchestrator, print);
    const PrintObject &object = print.object(0);
    const object_handle *handle = reinterpret_cast<const object_handle *>(&object);

    // The native region flow exists, with a positive spacing, but has no volume. The support flow
    // printed 1 mm high already fails on its spacing; it must give a zero flow too.
    const Flow region_flow = object.printing_region(0).flow(object, frExternalPerimeter, 1., 2);
    REQUIRE(region_flow.spacing() > 0);
    REQUIRE_THROWS_AS(region_flow.mm3_per_mm(), FlowErrorNegativeFlow);
    REQUIRE_THROWS(support_material_flow(&object, 0.2f).with_height(1.f));

    const c_flow flows[] = {
        object_get_region_flow(handle, 0, RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER, 1., 2),
        object_get_support_flow(handle, RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL, 0.2, 1., 0),
    };
    for (const c_flow &flow : flows) {
        CHECK(flow.width == 0);
        CHECK(flow.spacing == 0);
        CHECK(flow.height == 0);
        CHECK(flow.mm3_per_mm == 0);
    }
    const c_flow_mm flows_mm[] = {
        object_get_region_flow_mm(handle, 0, RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER, 1., 2),
        object_get_support_flow_mm(handle, RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL, 0.2, 1., 0),
    };
    for (const c_flow_mm &flow : flows_mm) {
        CHECK(flow.width == 0);
        CHECK(flow.spacing == 0);
        CHECK(flow.height == 0);
    }
}