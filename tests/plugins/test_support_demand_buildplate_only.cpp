#include <catch2/catch.hpp>

#include <algorithm>
#include <fstream>
#include <sstream>

#include "plugin_test_helpers.hpp"
#include "test_data.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_support_demand.h"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Steps/StepLayerHeightGeneration.hpp"
#include "libslic3r/Steps/StepPostSlicing.hpp"
#include "libslic3r/Steps/StepSlicing.hpp"
#include "libslic3r/TriangleMesh.hpp"

/* These tests use the same demand registrations as production startup.
   A direct filter call uses demand placed through the public mutator on three
   layers: separate lower footprints show cumulative coverage and its 0.01 mm
   expansion. A full slicing run also checks automatic demand above a step. */

namespace {
using namespace Slic3r;

// Build a rectangular slice in millimetres for exact coverage comparisons.
ExPolygon rectangle(double left, double right)
{
    return ExPolygon(Polygon({Point(scale_i(left), 0), Point(scale_i(right), 0),
                              Point(scale_i(right), scale_i(10.)), Point(scale_i(left), scale_i(10.))}));
}
} // namespace

TEST_CASE("Production demand registrations include all producers and filters", "[plugins][support-demand]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const Orchestrator &orchestrator = Orchestrator::instance();
    std::ifstream stream(std::string(TEST_DATA_DIR) + "/../../src/plugins/default_activated.ini.in");
    REQUIRE(stream.good());
    std::ostringstream contents;
    contents << stream.rdbuf();
    for (const char *id : {"support.demand.overhangs", "support.demand.painting", "support.demand.modifiers",
                           "support.demand.bridge_removal", "support.demand.buildplate_only"}) {
        REQUIRE(orchestrator.get_plugin(id) != nullptr);
        CHECK(orchestrator.is_plugin_active(id));
        CHECK(contents.str().find(std::string(id) + " = 1") != std::string::npos);
    }
    const Plugin &filter = *orchestrator.get_plugin("support.demand.buildplate_only");
    CHECK(filter.get_step() == STEP_SUPPORT_DEMAND);
    CHECK(filter.get_priority() == 100);
    CHECK(std::any_of(filter.get_used_config_keys().begin(), filter.get_used_config_keys().end(),
                      [](const Plugin::UsedConfigKey &key) { return key.key == "support_material_buildplate_only"; }));
}

TEST_CASE("Buildplate-only demand removes expanded lower coverage for grid and snug objects only", "[plugins][support-demand]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool enabled = GENERATE(false, true);
    const char *style = GENERATE("grid", "snug", "organic");
    const bool filtered = enabled && std::string(style) != "organic";
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"layer_height", "1"}, {"first_layer_height", "1"},
                                   {"nozzle_diameter", "2"}, {"max_layer_height", "1"},
                                   {"support_material_style", style},
                                   {"support_material_buildplate_only", enabled ? "1" : "0"}});
    Print print;
    Model model;
    Test::init_print({make_cube(10., 10., 3.)}, print, model, config);
    Orchestrator &orchestrator = Orchestrator::instance();
    Steps::StepLayerHeightGeneration::run_step(orchestrator, print);
    Steps::StepSlicing::run_step(orchestrator, print);
    Steps::StepPostSlicing::run_step(orchestrator, print);
    PrintObject &object = print.object(0);
    REQUIRE(object.layers().size() == 3);

    // Use separate lower footprints: the third layer must still remember the first.
    const ExPolygons first {rectangle(0., 4.)};
    const ExPolygons second {rectangle(6., 8.)};
    expolygons_copy(layer_borrow_mutable_slices(reinterpret_cast<layer_handle *>(&object.layer(0))),
                    reinterpret_cast<const expolygon_collection_handle *>(&first));
    expolygons_copy(layer_borrow_mutable_slices(reinterpret_cast<layer_handle *>(&object.layer(1))),
                    reinterpret_cast<const expolygon_collection_handle *>(&second));

    PluginStorage storage;
    run_ctx_support_demand payload {reinterpret_cast<const print_handle *>(&print),
                                    reinterpret_cast<const object_handle *>(&object)};
    plugin_run_context context = {};
    context.step = STEP_SUPPORT_DEMAND;
    context.data = &payload;
    context.plugin_storage = reinterpret_cast<storage_handle *>(&storage);
    const ExPolygons demand {rectangle(0., 10.)};
    for (Layer &layer : object.layers()) {
        REQUIRE(layer.islands().size() == 1);
        expolygons_copy(support_demand_get_island_mutable(&context,
                            reinterpret_cast<const layer_island_handle *>(&layer.island(0))),
                        reinterpret_cast<const expolygon_collection_handle *>(&demand));
    }

    const Plugin &filter = *orchestrator.get_plugin("support.demand.buildplate_only");
    filter.setup(context, 1);
    filter.setup_run(context);
    filter.run(context);
    const ExPolygons expected[] = {
        demand,
        filtered ? ExPolygons {rectangle(4.01, 10.)} : demand,
        filtered ? ExPolygons {rectangle(4.01, 5.99), rectangle(8.01, 10.)} : demand
    };
    for (size_t idx = 0; idx < 3; ++idx) {
        const ExPolygons &actual = object.layer(idx).island(0).support_demand();
        CHECK(unscaled(unscaled(area(diff_ex(actual, expected[idx])))) < 0.00001);
        CHECK(unscaled(unscaled(area(diff_ex(expected[idx], actual)))) < 0.00001);
    }
}

TEST_CASE("Automatic support above a step is filtered during a full slicing run", "[plugins][support-demand]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool enabled = GENERATE(false, true);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"layer_height", "0.5"}, {"first_layer_height", "0.5"},
                                   {"nozzle_diameter", "0.8"}, {"support_material", "1"},
                                   {"support_material_auto", "1"}, {"dont_support_bridges", "0"},
                                   {"support_material_buildplate_only", enabled ? "1" : "0"}});
    TriangleMesh mesh = make_cube(10., 10., 1.);
    TriangleMesh pillar = make_cube(2., 10., 1.);
    pillar.translate(4., 0., 1.);
    mesh.merge(pillar);
    TriangleMesh roof = make_cube(14., 10., 1.);
    roof.translate(0., 0., 2.);
    mesh.merge(roof);
    Print print;
    Model model;
    Test::init_print({mesh}, print, model, config);
    print.process();
    const PrintObject &object = print.object(0);
    REQUIRE(object.layers().size() >= 6);
    const ExPolygons covered = offset_ex(object.layer(0).lslices(), scale_d(0.01));
    double above_object = 0.;
    double above_bed = 0.;
    for (const Layer &layer : object.layers())
        if (layer.unscaled_print_z() > 2.)
            for (const LayerSliceIsland &island : layer.islands()) {
                above_object += unscaled(unscaled(area(intersection_ex(island.support_demand(), covered))));
                above_bed += unscaled(unscaled(area(diff_ex(island.support_demand(), covered))));
            }
    if (enabled)
        CHECK(above_object < 0.00001);
    else
        CHECK(above_object > 1.);
    CHECK(above_bed > 1.);
}
