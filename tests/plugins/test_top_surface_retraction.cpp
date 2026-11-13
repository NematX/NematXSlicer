///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.

#include "layer_extrusion_edit_transition_test_helpers.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Steps/StepLayerHeightGeneration.hpp"
#include "libslic3r/Steps/StepPostSlicing.hpp"
#include "libslic3r/Steps/StepSlicing.hpp"

/* Drive the real NematX policy editor and the common transition pipeline.
   A narrow published top strip lies between two endpoints outside that strip.
   The old LayerRegion surface container stays empty, so this also detects reads
   of obsolete geometry. Threshold and explicit modifier variations check that
   the policy changes only the interior mask, never the request precedence.

       init_print -> slice -> publish final top strip
       run_layer_plugins -> scopes -> NematX policy -> retraction -> travel
       collect_transition_events -> check the paired semantic E-axis events */
namespace {
using namespace Slic3r;
using namespace Slic3r::Printing;
using namespace Slic3r::Test::TransitionPipeline;
using slic3r_api::LayerExtrusionEdit::ExtrusionScope::OrderedExtrusionScope;

TEST_CASE("Top-surface retraction uses final surfaces without bypassing thresholds",
          "[plugins][top-surface-retraction]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool enabled = GENERATE(false, true);
    const bool top = GENERATE(false, true);
    const bool full_top = GENERATE(false, true);
    const int mode = GENERATE(0, 1, 2, 3);
    CAPTURE(enabled, top, full_top, mode);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"enforce_retract_top_surface", enabled ? "1" : "0"},
        {"only_retract_when_crossing_perimeters", "1"}, {"enforce_retract_first_layer", "0"},
        {"retract_before_travel", mode == 1 ? "100" : "0.5"}, {"retract_layer_change", "0"},
        {"retract_length", "2"}, {"fill_density", "20%"}
    });
    Print print;
    Model model;
    Slic3r::Test::init_print({Slic3r::Test::TestMesh::cube_20x20x20}, print, model, config);
    REQUIRE(print.config().option("enforce_retract_top_surface") != nullptr);
    CHECK(print.config().option("enforce_retract_top_surface")->serialize() == (enabled ? "1" : "0"));
    CHECK(slic3r_api::Print(reinterpret_cast<const print_handle *>(&print)).config()
        .bool_or_default("enforce_retract_top_surface", true) == enabled);
    Orchestrator &host = Orchestrator::instance();
    REQUIRE(host.get_plugin("nematx_top_surface_retraction") != nullptr);
    Steps::StepLayerHeightGeneration::run_step(host, print);
    Steps::StepSlicing::run_step(host, print);
    Steps::StepPostSlicing::run_step(host, print);
    PrintObject &object = print.object(0);
    Layer &native_layer = object.layer(1);
    LayerSliceIsland &island = native_layer.island(0);
    LayerRegionIsland &region = island.get_or_add_region_island(island.regions(), 0);
    const Point center = get_extents(native_layer.lslices()).center();
    const Point shift = object.instances().front().shift;
    if (top) {
        Polygon strip({center + Point(scale_i(-.5), scale_i(-2.)),
                       center + Point(scale_i(.5), scale_i(-2.)),
                       center + Point(scale_i(.5), scale_i(2.)),
                       center + Point(scale_i(-.5), scale_i(2.))});
        strip.make_counter_clockwise();
        region.set_fill_surfaces().surfaces.emplace_back(stPosTop | stDensSolid,
            full_top ? native_layer.lslices().front() : ExPolygon(std::move(strip)));
    }
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.clear();
    const PrintingObjectInstanceId catalog = plan.find_or_add_object_instance(object, 0);
    plan.groups.emplace_back();
    plan.groups.back().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.back().layers.back();
    layer.print_z = native_layer.scaled_print_z();
    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool = layer.tool_groups.back();
    tool.extruder_id = 0;
    for (const double x : {-3., 1.}) {
        std::unique_ptr<ExtrusionPath> path = make_print_path(0., 1.);
        path->set_polyline(ArcPolyline(Points{center + shift + Point(scale_i(x), 0),
            center + shift + Point(scale_i(x + 1.), 0)}));
        if (mode == 2) path->get_or_add_property<ExtrusionPropertyModifier>().enforce_retraction = 1;
        if (mode == 3) path->get_or_add_property<ExtrusionPropertyModifier>().disable_retraction = 1;
        append_printing_extrusion(tool, path->clone_canonical());
        tool.extrusions.back().region_island = &region;
        tool.extrusions.back().printing_catalog_id = catalog;
    }
    run_layer_plugins(print, {"layer_extrusion_edit.transition_scope.default",
        "nematx_top_surface_retraction", "layer_extrusion_edit.retraction.default",
        "layer_extrusion_edit.entry_state.default", "layer_extrusion_edit.travel.default"});
    const OrderedExtrusionScope source(find_scope(entity_view(*tool.extrusions.front().root)), scope_property_key());
    const OrderedExtrusionScope target(find_scope(entity_view(*tool.extrusions.back().root)), scope_property_key());
    std::vector<std::string> outgoing;
    std::vector<std::string> incoming;
    collect_transition_events(source.after().readonly(), outgoing);
    collect_transition_events(target.before().readonly(), incoming);
    const bool expected = mode == 2 || (mode == 0 && enabled && top);
    CHECK(outgoing == (expected ? std::vector<std::string>{"retract"} : std::vector<std::string>{}));
    CHECK(incoming == (expected ? std::vector<std::string>{"unretract"} : std::vector<std::string>{}));
}
}
