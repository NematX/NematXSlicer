///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
/* Test uniform/end bridge extensions and flow corrections on small shapes whose
   expected geometry and extrusion amounts can be calculated independently.

   Geometry cases compare rectangle/capsule results with explicit polygon bounds,
   analytic areas and symmetry/monotonicity invariants, not another axial builder.
   view() only borrows a native polygon vector for an API call; it does not run
   an expansion or clearance calculation. same_area() checks both differences
   within SCALED_EPSILON boundary displacement after integer rounding.

   Fixture cases replace linked layers of a small Print with exact bridge cores
   and receiving areas. Typical setup is to prepare surfaces, seed controlled
   extrusion paths and support-completion flags, then dispatch finalization and
   inspect the result. These are separate test operations, not a chain of calls
   made by Fixture::prepare(). Some recipe cases instead generate real pattern
   output before checking its direction and flow.

   Representative direct calls from separate cases:

       TEST_CASE [axial expansion]
           |-- view() [borrow core and allowed polygons]
           |-- BridgeGeometry::axial_expand()
           `-- same_area() [compare with explicit expected polygons]

       TEST_CASE [extrusion-footprint clearance]
           |-- BridgeClearance::Index::add()
           `-- BridgeClearance::Index::clearance()

       TEST_CASE [section ramps and printable floor]
           `-- BridgeClearance::section_pieces()

       TEST_CASE [host publication contract]
           |-- Fixture::prepare()
           |   `-- StepSurfaceGeneration::run_step()
           |-- Fixture::add_paths() [synthetic paths and completion flags]
           `-- StepFinalizeSlicing::run_step() [dispatch selected plugins]

   The publication case installs a scoped test action on FinalProbe. The host
   dispatcher reaches the override through PluginBase; it is not a direct call
   from StepFinalizeSlicing::run_step() to FinalProbe::run_impl(). In that probe:

       FinalProbe::run_impl() [dispatched plugin override]
           `-- action() [test callback]
               `-- replace_infill_buckets() [host publication callback]

   The same probe mechanism also tests cancellation, replay protection and
   surface-density budgets. Each action is synchronous and borrows its payload.
   Fixture owns its model, print and native geometry; API views remain borrowed
   while those owners live. Native XY uses scaled integers; clearance lengths
   use mm, sections mm2 and volumes mm3. Injected support states test contracts,
   not production support generation; real-provider slicing/export integration
   is in test_bridge_pipeline.cpp. Active providers and callbacks are restored
   by their test scopes.
   Rounded-row cases exercise both finalizers on a quantized 30-degree strand;
   malformed directions, bends, returns, arcs and Z offsets must still fail.
*/
#include <catch2/catch.hpp>
#include <algorithm>
#include <cmath>
#include <functional>
#include <string>
#include <vector>
#include "perimeter_test_helpers.hpp"
#include "plugin_test_helpers.hpp"
#include "libslic3r/Api/host/InfillBridgeFlow.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/internal/LayerAccess.hpp"
#include "libslic3r/Api/internal/LayerIslandAccess.hpp"
#include "libslic3r/Api/internal/LayerRegionAccess.hpp"
#include "libslic3r/Api/plugin/c/slic3r_data_tree.h"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/cpp/properties/BridgeProperties.hpp"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_finalize_slicing.h"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Plugins/Infill/BridgeClearance.hpp"
#include "libslic3r/Plugins/Infill/BridgeSurfaceVolume.hpp"
#include "libslic3r/Plugins/Surface/BridgeGeometry.hpp"
#include "libslic3r/Plugins/Surface/BridgePreparation.hpp"
#include "libslic3r/Plugins/Surface/BridgeRows.hpp"
#include "libslic3r/Steps/StepFinalizeSlicing.hpp"
#include "libslic3r/Steps/StepGenerateInfill.hpp"
#include "libslic3r/Steps/StepPostInfillGeneration.hpp"
#include "libslic3r/Steps/StepSupportDemand.hpp"
#include "libslic3r/Steps/StepGenerateSupport.hpp"
#include "libslic3r/Steps/StepSupportToolpath.hpp"
#include "libslic3r/Steps/StepSurfaceGeneration.hpp"

namespace {
using namespace Slic3r;
using namespace Slic3r::Test::PerimeterPluginTests;
namespace Clearance=slic3r_api::BridgeClearance;
namespace BridgeGeometry=slic3r_api::BridgeGeometry;
namespace BridgeRows=slic3r_api::BridgeRows;

// Return an immutable C API view without transferring ownership. The caller's
// polygon vector must outlive every use of the returned view; no allocation occurs.
slic3r_api::ExPolygonCollection view(const ExPolygons &areas);
// Check both polygon differences within SCALED_EPSILON boundary displacement,
// without using the axial builder as an oracle.
void same_area(const slic3r_api::ExPolygonCollection &actual,const ExPolygons &expected);
// Scope provider selection for direct dispatch and restore it when the scope exits.
class Active {
public:
    // Save activation, then select the requested ids; an invalid id fails the test.
    explicit Active(std::initializer_list<const char *> ids) {
        Orchestrator &orch=Orchestrator::instance();
        for (Plugin *plugin : orch.active_plugins()) previous.push_back(plugin);
        orch.clear_active_plugins();
        for (const char *id : ids) REQUIRE(orch.set_plugin_active(id,true));
    }
    // Restore the saved providers and cancellation after a constructed test scope.
    ~Active() { Orchestrator &orch=Orchestrator::instance();orch.clear_active_plugins();for (Plugin *p : previous) orch.set_plugin_active(p,true);orch.reset_plugin_cancel(); }
private:
    std::vector<Plugin *> previous;
};
// Rebuild two linked layers and seed one exact bridge with known metadata.
struct Fixture {
    PreparedPerimeterPrint prepared;
    // Own a print/model fixture with two replaced layers, exact core/receivers
    // and distinct metadata; deserialize caller overrides before seeding geometry.
    explicit Fixture(std::initializer_list<std::pair<std::string,std::string>> overrides={});
    // Borrow the target island on the second object layer.
    LayerSliceIsland &island() { return prepared.print.object(0).layer(1).island(0); }
    // Borrow the first prepared bridge surface; throw when no such output exists.
    Surface &bridge();
    // Replace the two linked layers with a broad frame around these unsupported
    // cores, leaving ordinary infill beside them for uniform expansion.
    void frame(const ExPolygons &cores=ExPolygons{rectangle_expolygon(-2,-1,2,1)});
    // Put the solid core inside a complete lower sparse slice, with solid side
    // anchors. The next solid layer stays ordinary and top position is retained.
    void internal(SurfaceType position=stPosInternal);
    // Supply typed input for untouched islands, then run only bridge preparation.
    void prepare();
    // Seed a straight X-axis bridge and optional lower contact paths, then mark
    // synthetic demand/material as known; this does not run support providers.
    void add_paths(bool contacts=true);
};
// Invoke callback on borrowed printable leaves with resolved inherited attributes.
// Missing attributes fail the test; the tree and attribute ownership stay unchanged.
void visit_leaves(const ExtrusionEntity &root,const std::function<void(const ExtrusionEntity &,const ExtrusionAttributes &)> &callback,
    const ExtrusionAttributes *inherited=nullptr);
// Run finalization, which must not stop slicing, and return its warnings about
// compensation it skipped; earlier queued plugin messages are discarded.
std::vector<std::string> finalize_skipped(Print &print);

// Run a scoped action through the actual dispatcher after the bridge finalizer.
// The action borrows its payload only for the call; PluginBase guards the C boundary.
class FinalProbe final : public slic3r_api::PluginBase {
public:
    // Bind the process-held probe to the test host without taking ownership.
    explicit FinalProbe(orchestrator_handle *orch) : PluginBase(orch) {}
    std::function<void(const run_ctx_finalize_slicing &,const plugin_run_context &)> action;
private:
    // Expose a stable id for the test's direct activation scopes.
    const char *id_impl() const noexcept override { return "test.bridge.final_probe"; }
    // Let each test choose whether any production finalizer participates.
    const char *const *dependencies_impl() const noexcept override { static const char *none[]={nullptr};return none; }
    // Use the same host payload and publication capability as real finalizers.
    slicing_step_t step_impl() const noexcept override { return STEP_FINALIZE_SLICING; }
    // Observe the result after the bridge finalizer at priority 20.
    int32_t priority_impl() const noexcept override { return 90; }
    // Invoke the installed action; thrown failures are translated by PluginBase.
    void run_impl(const plugin_run_context *ctx) const override { action(*plugin_ctx_as_finalize_slicing(ctx),*ctx); }
};
// Register once, then reset the callback at the end of every test scope.
struct ProbeScope {
    FinalProbe &probe;
    // Borrow the single registered probe for this test scope.
    ProbeScope() : probe(instance()) {}
    // Drop the action so no captured fixture references escape the test.
    ~ProbeScope() { probe.action={}; }
    // Return the process-held probe, registering it once with the test host.
    static FinalProbe &instance() {
        Orchestrator &orch=Orchestrator::instance();
        static FinalProbe probe(reinterpret_cast<orchestrator_handle *>(&orch));
        if (orch.get_plugin("test.bridge.final_probe")==nullptr)
            orchestrator_register_plugin(reinterpret_cast<orchestrator_handle *>(&orch),probe.c_instance());
        return probe;
    }
};

slic3r_api::ExPolygonCollection view(const ExPolygons &areas)
{
    return slic3r_api::ExPolygonCollection(reinterpret_cast<const expolygon_collection_handle *>(&areas));
}
void same_area(const slic3r_api::ExPolygonCollection &actual,const ExPolygons &expected)
{
    const ExPolygons &native=*reinterpret_cast<const ExPolygons *>(actual.handle());
    // Rotation rounds vertices independently: enforce the specified maximum
    // boundary displacement, not exact equality of two integer constructions.
    CHECK(diff_ex(native,offset_ex(expected,SCALED_EPSILON)).empty());
    CHECK(diff_ex(expected,offset_ex(native,SCALED_EPSILON)).empty());
}
Fixture::Fixture(std::initializer_list<std::pair<std::string,std::string>> overrides)
{
    DynamicPrintConfig config=perimeter_config({{"layer_height","0.2"},{"first_layer_height","0.2"},
        {"perimeters","0"},{"bridged_infill_margin","1"},{"bridged_infill_uniform_margin","0"},{"bridge_angle","0"},
        {"bridge_volume_compensation","bridge.finalize_volume"}});
    for (const std::pair<std::string,std::string> &entry : overrides)
        config.set_deserialize_strict(entry.first,entry.second);
    prepare_cube_print(prepared,config);
    PrintObject &object=prepared.print.object(0);
    // Detach overlap links before replacing islands so no old neighbor pointer
    // can survive into the two-layer geometry built below.
    for (Layer &layer : object.layers()) for (LayerSliceIsland &i : layer.islands()) { i.overlaps_above.clear();i.overlaps_below.clear(); }
    const ExPolygons lower{rectangle_expolygon(-8,-1,-2,1),rectangle_expolygon(2,-1,8,1)};
    const ExPolygons upper{rectangle_expolygon(-8,-1,8,1)};
    for (size_t idx=0;idx<2;++idx) {
        Layer &layer=object.layer(idx);
        const ExPolygons areas=idx==0 ? lower : upper;
        ApiInternal::LayerRegionAccess::slices_mutable(layer.region(0))=areas;
        ApiInternal::LayerAccess::set_islands(layer,ExPolygons(areas));
        for (LayerSliceIsland &i : layer.islands()) { i.fill_regions(layer);ApiInternal::LayerIslandAccess::set_infill_areas(i,ExPolygons{i.get_slice()}); }
    }
    Layer::build_up_down_graph(object.layer(0),object.layer(1));
    LayerRegionIsland &ri=island().get_or_add_region_island(island().regions(),0);
    ri.set_fill_surfaces().append(ExPolygons{rectangle_expolygon(-2,-1,2,1)},stPosBottom|stDensSolid|stModBridge);
    ri.set_fill_surfaces().append(lower,stPosInternal|stDensSparse);
    Surface &source=ri.set_fill_surfaces().surfaces.front();
    // Distinct sentinel values make dropped or reset metadata observable after each pass.
    source.priority=7;source.thickness_layers=2;source.extra_perimeters=3;source.maxNbSolidLayersOnTop=9;
}
void Fixture::frame(const ExPolygons &cores)
{
    PrintObject &object=prepared.print.object(0);
    for (Layer &layer : object.layers()) for (LayerSliceIsland &island : layer.islands()) {
        island.overlaps_above.clear();island.overlaps_below.clear();
    }
    const ExPolygons upper{rectangle_expolygon(-8,-4,8,4)},lower=diff_ex(upper,cores);
    for (size_t idx=0;idx<2;++idx) {
        Layer &layer=object.layer(idx);
        const ExPolygons &areas=idx==0 ? lower : upper;
        ApiInternal::LayerRegionAccess::slices_mutable(layer.region(0))=areas;
        ApiInternal::LayerAccess::set_islands(layer,ExPolygons(areas));
        for (LayerSliceIsland &island : layer.islands()) {
            island.fill_regions(layer);ApiInternal::LayerIslandAccess::set_infill_areas(island,ExPolygons{island.get_slice()});
        }
    }
    Layer::build_up_down_graph(object.layer(0),object.layer(1));
    SurfaceCollection &input=island().get_or_add_region_island(island().regions(),0).set_fill_surfaces();
    input.append(cores,stPosBottom|stDensSolid|stModBridge);
    for (Surface &surface : input) {
        surface.priority=7;surface.thickness_layers=2;surface.extra_perimeters=3;surface.maxNbSolidLayersOnTop=9;
    }
    input.append(lower,stPosInternal|stDensSparse);
}
void Fixture::prepare()
{
    // The host pass also visits the untouched cube layers and lower islands.
    // Supply their typed input instead of bypassing its prerequisite checks.
    for (Layer &layer : prepared.print.object(0).layers())
        for (LayerSliceIsland &island : layer.islands()) {
            bool has_surfaces=false;
            for (const LayerRegionIsland &region : island.regions_islands())
                has_surfaces=has_surfaces || !region.fill_surfaces().empty();
            if (!has_surfaces && !island.infill_areas().empty())
                island.get_or_add_region_island(island.regions(),0).set_fill_surfaces().append(island.infill_areas(),stPosInternal|stDensSparse);
        }
    Active active({"surface.bridge_preparation","bridge_detector.default"});
    Plugin *plugin=Orchestrator::instance().get_plugin("surface.bridge_preparation");
    REQUIRE(plugin!=nullptr);
    // The fixture supplies the post-SolidShells snapshot; run only preparation.
    Steps::StepSurfaceGeneration::run_step(Orchestrator::instance(),prepared.print);
}
void Fixture::internal(SurfaceType position)
{
    PrintObject &object=prepared.print.object(0);
    for (Layer &layer : object.layers()) for (LayerSliceIsland &island : layer.islands()) {
        island.overlaps_above.clear();island.overlaps_below.clear();
    }
    const ExPolygons full{rectangle_expolygon(-8,-2,8,2)},core{rectangle_expolygon(-2,-2,2,2)};
    const ExPolygons sides=diff_ex(full,core);
    for (size_t idx=0;idx<3;++idx) {
        Layer &layer=object.layer(idx);
        ApiInternal::LayerRegionAccess::slices_mutable(layer.region(0))=full;
        ApiInternal::LayerAccess::set_islands(layer,ExPolygons(full));
        LayerSliceIsland &island=layer.island(0);island.fill_regions(layer);
        ApiInternal::LayerIslandAccess::set_infill_areas(island,ExPolygons(full));
        SurfaceCollection &surfaces=island.get_or_add_region_island(island.regions(),0).set_fill_surfaces();
        surfaces.append(core,idx==1 ? position|stDensSolid : idx==0 ? stPosInternal|stDensSparse : stPosInternal|stDensSolid);
        surfaces.append(sides,idx==1 ? stPosInternal|stDensSparse : stPosInternal|stDensSolid);
        if (idx==1) {
            Surface &source=surfaces.surfaces.front();source.priority=7;source.thickness_layers=2;
            source.extra_perimeters=3;source.maxNbSolidLayersOnTop=9;
        }
    }
    Layer::build_up_down_graph(object.layer(0),object.layer(1));Layer::build_up_down_graph(object.layer(1),object.layer(2));
}
Surface &Fixture::bridge()
{
    for (LayerRegionIsland &ri : island().regions_islands())
        for (Surface &surface : ri.set_fill_surfaces())
            if (surface.get_property<slic3r_api::BridgeSurfaceInfo>()!=nullptr) return surface;
    throw std::runtime_error("Missing prepared bridge in fixture.");
}
void Fixture::add_paths(bool contacts)
{
    PrintObject &object=prepared.print.object(0);
    if (contacts) for (size_t idx=0;idx<2;++idx) {
        LayerSliceIsland &lower=object.layer(0).island(idx);
        LayerRegionIsland &ri=lower.get_or_add_region_island(lower.regions(),0);
        const double left=idx==0 ? -8. : 2.,right=idx==0 ? -2. : 8.;
        ExtrusionEntity path(true,ArcPolyline(Points{Point(scale_i(left),0),Point(scale_i(right),0)}));
        path.add_property(ExtrusionAttributes(ExtrusionRole::InternalInfill,ExtrusionFlow(.08,.4f,.2f)));
        ri.mutable_extrusion(LayerRegionIsland::INFILLS).append_child(std::move(path));
    }
    for (LayerRegionIsland &ri : island().regions_islands())
        for (const Surface &surface : ri.fill_surfaces()) {
            if (surface.get_property<slic3r_api::BridgeSurfaceInfo>()==nullptr) continue;
            ExtrusionEntity path(true,ArcPolyline(Points{Point(scale_i(-3.),0),Point(scale_i(3.),0)}));
            path.add_property(ExtrusionAttributes(ExtrusionRole::BridgeInfill,Flow::bridging_flow(.4f,.4f)));
            path.get_or_add_property<ExtrusionPropertyInfill>().source_surface_id=surface.id();
            ri.mutable_extrusion(LayerRegionIsland::INFILLS).append_child(std::move(path));
    }
    // This fixture explicitly supplies known synthetic material/demand. Cases
    // exercising unavailable producers overwrite these flags before dispatch.
    slic3r_api::SupportPipelineInfo &state=object.get_or_add_property<slic3r_api::SupportPipelineInfo>();
    state.demand_computed=1;state.material_generated=1;
}
std::vector<std::string> finalize_skipped(Print &print)
{
    Orchestrator &orch=Orchestrator::instance();
    (void)orch.consume_plugin_messages();
    REQUIRE_NOTHROW(Steps::StepFinalizeSlicing::run_step(orch,print));
    std::vector<std::string> out;
    for (const Orchestrator::PluginMessage &message : orch.consume_plugin_messages())
        if (message.level==Orchestrator::PluginMessageLevel::Warning && message.message.find("bridge compensation skipped")!=std::string::npos)
            out.push_back(message.message);
    return out;
}

void visit_leaves(const ExtrusionEntity &root,const std::function<void(const ExtrusionEntity &,const ExtrusionAttributes &)> &callback,
    const ExtrusionAttributes *inherited)
{
    const ExtrusionAttributes *attributes=root.get_property<ExtrusionAttributes>();
    if (attributes==nullptr) attributes=inherited;
    if (root.is_leaf() && root.has_polyline()) { REQUIRE(attributes!=nullptr);callback(root,*attributes); }
    for (const ExtrusionEntityUPtr &child : root.children()) if (child) visit_leaves(*child,callback,attributes);
}
}

TEST_CASE("Bridge finalizers accept integer-rounded short rows in either direction","[plugins][bridge-volume][bridge-direction-rounding]")
{
    for (const char *provider : {"bridge.finalize_surface_volume","bridge.finalize_volume"})
        for (const bool reversed : {false,true}) {
            INFO(provider<<" reversed="<<reversed);
            Fixture fixture({{"bridge_angle","30"},{"bridge_volume_compensation",provider},
                {"fill_density","30%"},{"internal_bridge_expansion","0"}});
            fixture.internal();fixture.prepare();
            Points points{Point(6368327,1360725),Point(6368327,1905952)};
            float rotation=float(PI/6.);rotation+=float(PI/2.);
            for (Point &point : points) point.rotate(rotation);
            REQUIRE(points.front()==Point(-4362586,4834770));
            REQUIRE(points.back()==Point(-4834767,4562157));
            // Translation keeps the recorded row vector but places it inside the fixture's prepared core.
            for (Point &point : points) point+=Point(4500000,-4700000);
            if (reversed) std::reverse(points.begin(),points.end());
            ExtrusionEntity path(true,ArcPolyline(points));
            path.add_property(ExtrusionAttributes(ExtrusionRole::InternalBridgeInfill,Flow::bridging_flow(.4f,.4f)));
            path.get_or_add_property<ExtrusionPropertyInfill>().source_surface_id=fixture.bridge().id();
            fixture.island().regions_islands().front().mutable_extrusion(LayerRegionIsland::INFILLS).append_child(std::move(path));
            Active active({provider,"surface.bridge_preparation","infill.recipe.bridge_preparation","surface.solid_shells"});
            REQUIRE_NOTHROW(Steps::StepFinalizeSlicing::run_step(Orchestrator::instance(),fixture.prepared.print));
            size_t leaves=0;
            for (const LayerRegionIsland &region : fixture.island().regions_islands())
                if (region.has_extrusion(LayerRegionIsland::INFILLS))
                    visit_leaves(region.extrusion(LayerRegionIsland::INFILLS),[&](const ExtrusionEntity &leaf,const ExtrusionAttributes &attributes) {
                        ++leaves;
                        REQUIRE(leaf.get_property<slic3r_api::BridgeVolumeInfo>()!=nullptr);
                        CHECK(attributes.role==ExtrusionRole::InternalBridgeInfill);
                        CHECK(leaf.first_point()==points.front());
                        CHECK(leaf.last_point()==points.back());
                    });
            CHECK(leaves==1);
        }
}

TEST_CASE("Bridge finalizers leave malformed rows uncorrected beyond endpoint rounding","[plugins][bridge-volume][bridge-direction-rounding]")
{
    for (const char *provider : {"bridge.finalize_surface_volume","bridge.finalize_volume"})
        for (const std::string defect : {"short direction","long direction","bend","return","arc","Z offset"}) {
            INFO(provider<<" defect="<<defect);
            Fixture fixture({{"bridge_angle","30"},{"bridge_volume_compensation",provider},
                {"fill_density","30%"},{"internal_bridge_expansion","0"}});
            fixture.internal();fixture.prepare();
            Points points{Point(137414,134770),Point(-334767,-137843)};
            if (defect=="short direction") points.back().y()+=10;
            else if (defect=="long direction") points={Point(-1000000,-577350),Point(1000000,577365)};
            else if (defect=="bend") points.insert(points.begin()+1,Point(-98676,98463));
            else if (defect=="return") points={points.front(),points.back(),points.front(),points.back()};
            ExtrusionEntity path(true,ArcPolyline(points));
            path.add_property(ExtrusionAttributes(ExtrusionRole::InternalBridgeInfill,Flow::bridging_flow(.4f,.4f)));
            path.get_or_add_property<ExtrusionPropertyInfill>().source_surface_id=fixture.bridge().id();
            if (defect=="arc" || defect=="Z offset") {
                slic3r_api::MutableExtrusionEntity writable(reinterpret_cast<extrusion_entity_handle *>(&path));
                std::vector<c_extrusion_segment> segments=writable.segments();
                REQUIRE(segments.size()==1);
                if (defect=="arc") {
                    segments.front().radius=float(scale_i(1.));
                    segments.front().orientation=RAW_EXTRUSION_ARC_ORIENTATION_CCW;
                } else segments.front().z_offset_b=scale_i(.01);
                REQUIRE(writable.set_segments(segments));
            }
            LayerRegionIsland &owner=fixture.island().regions_islands().front();
            owner.mutable_extrusion(LayerRegionIsland::INFILLS).append_child(std::move(path));
            Active active({provider,"surface.bridge_preparation","infill.recipe.bridge_preparation","surface.solid_shells"});
            // The malformed row refuses its bucket, which keeps its incoming flow uncorrected.
            CHECK(finalize_skipped(fixture.prepared.print).size()==1);
            visit_leaves(owner.extrusion(LayerRegionIsland::INFILLS),[](const ExtrusionEntity &leaf,const ExtrusionAttributes &) {
                CHECK(leaf.get_property<slic3r_api::BridgeVolumeInfo>()==nullptr);
            });
        }
}

TEST_CASE("First solid covers on internal sparse become internal bridges including top covers","[plugins][bridge-internal][bridge-preparation]")
{
    for (const SurfaceType position : {stPosInternal,stPosTop}) for (const char *density : {"0%","5%","50%","100%"}) {
        INFO(int(position)<<" density "<<density);
        Fixture fixture({{"fill_density",density},{"internal_bridge_expansion","0"}});fixture.internal(position);fixture.prepare();
        size_t bridges=0;ExPolygons partition;double area=0.;
        for (const LayerRegionIsland &group : fixture.island().regions_islands())
            for (const Surface &surface : group.fill_surfaces()) {
                partition.push_back(surface.expolygon);area+=surface.expolygon.area();
                const slic3r_api::BridgeSurfaceInfo *info=surface.get_property<slic3r_api::BridgeSurfaceInfo>();
                if (info==nullptr) continue;
                ++bridges;CHECK(surface.surface_type==(position|stDensSolid|stModBridge));CHECK(info->core_rule==2);
                CHECK(surface.bridge_angle==0.);CHECK(info->angle_from_user==1);CHECK_FALSE(info->extension_requested);
                CHECK(group.regions().size()==1);CHECK(group.extruder_id()==0);
                CHECK(surface.priority==7);CHECK(surface.thickness_layers==2);CHECK(surface.extra_perimeters==3);CHECK(surface.maxNbSolidLayersOnTop==9);
                same_area(view(ExPolygons{surface.expolygon}),ExPolygons{rectangle_expolygon(-2,-2,2,2)});
            }
        CHECK(bridges==(std::string(density)=="100%" ? 0 : 1));
        same_area(view(partition),ExPolygons{rectangle_expolygon(-8,-2,8,2)});CHECK(area*SCALING_FACTOR*SCALING_FACTOR==Approx(64.));
        for (const LayerRegionIsland &group : fixture.prepared.print.object(0).layer(2).island(0).regions_islands())
            for (const Surface &surface : group.fill_surfaces())
                if (surface.get_property<slic3r_api::BridgeSurfaceInfo>()!=nullptr)
                    CHECK(intersection_ex(ExPolygons{surface.expolygon},ExPolygons{rectangle_expolygon(-2,-2,2,2)}).empty());
    }
    Fixture narrow({{"internal_bridge_min_width","5"}});narrow.internal();narrow.prepare();
    for (const LayerRegionIsland &group : narrow.island().regions_islands())
        for (const Surface &surface : group.fill_surfaces()) CHECK(surface.get_property<slic3r_api::BridgeSurfaceInfo>()==nullptr);
    for (const SurfaceType lower_type : {stPosInternal|stDensVoid,stPosInternal|stDensSparse|stModBridge}) {
        Fixture explicit_type({{"fill_density","100%"},{"internal_bridge_expansion","0"}});explicit_type.internal();
        explicit_type.prepared.print.object(0).layer(0).island(0).regions_islands().front().set_fill_surfaces().surfaces.front().surface_type=lower_type;
        explicit_type.prepare();CHECK(explicit_type.bridge().get_property<slic3r_api::BridgeSurfaceInfo>()->core_rule==2);
    }
}

TEST_CASE("Internal bridge expansion ignores external margins and its automatic fallback crosses lower sparse","[plugins][bridge-internal][bridge-preparation]")
{
    uint64_t reference=0;
    for (const bool expansion : {false,true}) for (const char *margin : {"0","20"}) {
        Fixture fixture({{"internal_bridge_expansion",expansion ? "1" : "0"},{"bridged_infill_margin",margin},
            {"bridged_infill_uniform_margin",margin}});fixture.internal();fixture.prepare();
        const Surface &surface=fixture.bridge();
        const BoundingBox bounds=get_extents(surface.expolygon);
        CHECK((bounds.max.x()>scale_i(2.))==expansion);CHECK(bounds.max.y()==scale_i(2.));
        const uint64_t key=BridgeGeometry::geometry_key(view(ExPolygons{surface.expolygon}));
        if (std::string(margin)=="0") reference=key;else CHECK(key==reference);
    }
    // With no full material below, angle detection cannot choose an anchor axis.
    // A lower rectilinear recipe at 17 degrees gives the selected 107-degree fallback.
    Fixture automatic({{"bridge_angle","!0"},{"fill_angle","17"},{"fill_angle_increment","0"},{"internal_bridge_expansion","0"}});
    automatic.internal();
    LayerRegionIsland &lower=automatic.prepared.print.object(0).layer(0).island(0).regions_islands().front();
    lower.set_fill_surfaces().clear();lower.set_fill_surfaces().append(ExPolygons{rectangle_expolygon(-8,-2,8,2)},stPosInternal|stDensSparse);
    automatic.prepare();CHECK(automatic.bridge().bridge_angle==Approx(107.*PI/180.).margin(1e-6));
}

TEST_CASE("Lower internal bridge extensions count as solid before preparing the next cover","[plugins][bridge-internal][bridge-preparation]")
{
    Fixture fixture({{"fill_density","30%"},{"internal_bridge_min_width","0"},{"internal_bridge_expansion","1"}});fixture.internal();
    LayerSliceIsland &above=fixture.prepared.print.object(0).layer(2).island(0);
    const ExPolygons cover{rectangle_expolygon(-3,-2,3,2)};
    SurfaceCollection &surfaces=above.regions_islands().front().set_fill_surfaces();
    surfaces.clear();surfaces.append(cover,stPosInternal|stDensSolid);
    surfaces.append(diff_ex(above.infill_areas(),cover),stPosInternal|stDensSparse);
    fixture.prepare();
    CHECK(get_extents(fixture.bridge().expolygon).max.x()>scale_i(3.));
    for (const LayerRegionIsland &group : above.regions_islands())
        for (const Surface &surface : group.fill_surfaces()) CHECK(surface.get_property<slic3r_api::BridgeSurfaceInfo>()==nullptr);
}

TEST_CASE("Prepared top bridges retain their metadata through top expansion and cleanup","[plugins][bridge-internal][bridge-preparation]")
{
    Fixture fixture({{"internal_bridge_expansion","0"}});fixture.internal(stPosTop);fixture.prepare();
    const Surface expected=fixture.bridge();
    Active active({"surface.top_surface_expansion","surface.clean_infill_surfaces","surface.infill_region_compatibility_splitter","dense_infill.surface_marker"});
    Steps::StepSurfaceGeneration::run_step(Orchestrator::instance(),fixture.prepared.print);
    const Surface &actual=fixture.bridge();CHECK(actual.surface_type==(stPosTop|stDensSolid|stModBridge));
    CHECK(actual.properties_equal(expected));CHECK(actual.bridge_angle==expected.bridge_angle);
    same_area(view(ExPolygons{actual.expolygon}),ExPolygons{expected.expolygon});
}

TEST_CASE("External margins cannot claim the cores of neighboring internal bridges","[plugins][bridge-internal][bridge-preparation]")
{
    Fixture fixture({{"fill_density","15%"},{"bridged_infill_uniform_margin","25%"}});
    SurfaceCollection &input=fixture.island().regions_islands().front().set_fill_surfaces();input.clear();
    input.append(ExPolygons{rectangle_expolygon(-2,-1,2,1)},stPosBottom|stDensSolid|stModBridge);
    input.append(ExPolygons{rectangle_expolygon(-4,-1,-2,1),rectangle_expolygon(2,-1,4,1)},stPosInternal|stDensSolid);
    input.append(ExPolygons{rectangle_expolygon(-8,-1,-4,1),rectangle_expolygon(4,-1,8,1)},stPosInternal|stDensSparse);
    fixture.prepare();
    ExPolygons partition;double sum=0.;size_t external=0,internal=0;
    for (const LayerRegionIsland &group : fixture.island().regions_islands()) for (const Surface &surface : group.fill_surfaces()) {
        partition.push_back(surface.expolygon);sum+=surface.expolygon.area();
        if (const slic3r_api::BridgeSurfaceInfo *info=surface.get_property<slic3r_api::BridgeSurfaceInfo>()) {
            if (info->core_rule==1) ++external;else ++internal;
        }
    }
    CHECK(external==1);CHECK(internal==2);CHECK(sum*SCALING_FACTOR*SCALING_FACTOR==Approx(32.));
    same_area(view(partition),ExPolygons{rectangle_expolygon(-8,-1,8,1)});
}

TEST_CASE("Surface volume searches the requested excess below variable layers and stops locally at solid","[plugins][bridge-surface-volume]")
{
    Fixture fixture({{"fill_density","80%"}});fixture.internal();
    PrintObject &object=fixture.prepared.print.object(0);
    ApiInternal::LayerAccess::set_z(object.layer(0),scale_i(.1),scale_i(.1));
    ApiInternal::LayerAccess::set_z(object.layer(1),scale_i(.2),scale_i(.1));
    const ExPolygons full{rectangle_expolygon(-8,-2,8,2)};
    SurfaceCollection &first=object.layer(0).island(0).regions_islands().front().set_fill_surfaces();
    first.clear();first.append(ExPolygons{rectangle_expolygon(-8,-2,0,2)},stPosInternal|stDensSolid);
    first.append(ExPolygons{rectangle_expolygon(0,-2,8,2)},stPosInternal|stDensVoid);
    Layer &middle=object.layer(1);
    PrintRegionConfig lower_config=middle.region(0).region().config();lower_config.set_deserialize_strict("fill_density","30%");
    ApiInternal::LayerAccess::add_region(middle,fixture.prepared.print.register_print_region(lower_config));
    ApiInternal::LayerRegionAccess::slices_mutable(middle.region(0)).clear();
    ApiInternal::LayerRegionAccess::slices_mutable(middle.region(1))=full;
    middle.island(0).mutable_regions_islands().clear();middle.island(0).fill_regions(middle);
    SurfaceCollection &second=middle.island(0).get_or_add_region_island(middle.island(0).regions(),0).set_fill_surfaces();
    second.append(full,stPosInternal|stDensSparse);
    ProbeScope scope;
    scope.probe.action=[&](const run_ctx_finalize_slicing &ctx,const plugin_run_context &run) {
        slic3r_api::BridgeSurfaceVolume::Index index(slic3r_api::Object(ctx.object),ctx,&run);
        // A=.128, pitch=.4 and h=.2 means 160% deposition: .12 mm extra.
        // The 30% sparse band gives .07, leaving .05 to seek in the older band.
        const std::vector<Clearance::SectionPiece> pieces=index.sections({scale_i(-4.),0},{scale_i(4.),0},.2,.2,.128,.4,.5,.02);
        REQUIRE(pieces.size()==2);
        CHECK(pieces[0].begin==0.);CHECK(pieces[0].end==Approx(4.));CHECK(pieces[1].begin==Approx(4.));CHECK(pieces[1].end==Approx(8.));
        CHECK(pieces[0].clearance==Approx(.27));CHECK(pieces[0].area==Approx(.108));
        CHECK(pieces[1].clearance==Approx(.32));CHECK(pieces[1].area==Approx(.128));
        for (const Clearance::SectionPiece &piece : pieces) CHECK(piece.confidence==BRIDGE_CONFIDENCE_DENSITY_ESTIMATE);
        // Change only sparse paths, not surfaces: neither the budget nor its
        // replay signature may acquire line-scale fragmentation or phase dependence.
        const uint64_t signature=index.signature();
        for (size_t i=0;i<20;++i) {
            ExtrusionEntity path(false,ArcPolyline(Points{Point(scale_i(-7.+i*.5),scale_i(-2.)),Point(scale_i(-6.+i*.5),scale_i(2.))}));
            path.add_property(ExtrusionAttributes(ExtrusionRole::InternalInfill,ExtrusionFlow(.08,.4f,.2f)));
            object.layer(1).island(0).regions_islands().front().mutable_extrusion(LayerRegionIsland::INFILLS).append_child(std::move(path));
        }
        slic3r_api::BridgeSurfaceVolume::Index changed(slic3r_api::Object(ctx.object),ctx,&run);
        const std::vector<Clearance::SectionPiece> repeated=changed.sections({scale_i(-4.),0},{scale_i(4.),0},.2,.2,.128,.4,.5,.02);
        REQUIRE(repeated.size()==pieces.size());CHECK(changed.signature()==signature);
        for (size_t i=0;i<pieces.size();++i) { CHECK(repeated[i].area==pieces[i].area);CHECK(repeated[i].clearance==pieces[i].clearance); }
    };
    Active active({"test.bridge.final_probe"});REQUIRE_NOTHROW(Steps::StepFinalizeSlicing::run_step(Orchestrator::instance(),fixture.prepared.print));
}

TEST_CASE("Surface volume treats perimeter as a barrier and never credits void below the platform","[plugins][bridge-surface-volume]")
{
    Fixture fixture({{"fill_density","0%"}});fixture.internal();
    PrintObject &object=fixture.prepared.print.object(0);
    LayerSliceIsland &lower=object.layer(0).island(0);
    ApiInternal::LayerIslandAccess::set_infill_areas(lower,ExPolygons{rectangle_expolygon(-2,-2,2,2)});
    SurfaceCollection &surfaces=lower.regions_islands().front().set_fill_surfaces();
    surfaces.clear();surfaces.append(lower.infill_areas(),stPosInternal|stDensVoid);
    ProbeScope scope;
    scope.probe.action=[&](const run_ctx_finalize_slicing &ctx,const plugin_run_context &run) {
        const slic3r_api::BridgeSurfaceVolume::Index index(slic3r_api::Object(ctx.object),ctx,&run);
        const std::vector<Clearance::SectionPiece> pieces=index.sections({scale_i(-4.),0},{scale_i(4.),0},.2,.2,.18,.4,.6,.02);
        REQUIRE(pieces.size()==3);
        CHECK(pieces[0].area==Approx(.08));CHECK(pieces[1].area==Approx(.16));CHECK(pieces[2].area==Approx(.08));
        CHECK(pieces[1].clearance==Approx(.4));
    };
    Active active({"test.bridge.final_probe"});REQUIRE_NOTHROW(Steps::StepFinalizeSlicing::run_step(Orchestrator::instance(),fixture.prepared.print));
}

TEST_CASE("Surface volume stops once sufficient and keeps only unusable support portions unknown","[plugins][bridge-surface-volume]")
{
    Fixture fixture;fixture.frame();
    PrintObject &object=fixture.prepared.print.object(0);
    Layer &support=object.add_auxiliary_layer(0,scale_i(.1),scale_i(.1));
    support.get_or_add_property<LayerSupportProperty>().interface_id=0;
    ApiInternal::LayerAccess::set_islands(support,ExPolygons{rectangle_expolygon(-2,-2,0,2)});
    ProbeScope scope;
    scope.probe.action=[&](const run_ctx_finalize_slicing &ctx,const plugin_run_context &run) {
        const slic3r_api::BridgeSurfaceVolume::Index index(slic3r_api::Object(ctx.object),ctx,&run);
        // .04 mm excess is satisfied in the .1 mm air gap above the unknown
        // support; the search must stop without consulting that lower material.
        const std::vector<Clearance::SectionPiece> enough=index.sections({scale_i(-1.5),0},{scale_i(1.5),0},.2,.2,.096,.4,.5,.02);
        REQUIRE(enough.size()==1);CHECK(enough[0].area==Approx(.096));CHECK(enough[0].confidence==BRIDGE_CONFIDENCE_DENSITY_ESTIMATE);
        const std::vector<Clearance::SectionPiece> more=index.sections({scale_i(-1.5),0},{scale_i(1.5),0},.2,.2,.15,.4,.5,.02);
        REQUIRE(more.size()==2);CHECK(more[0].confidence==BRIDGE_CONFIDENCE_UNKNOWN);CHECK(more[0].area==.15);
        CHECK(more[1].confidence==BRIDGE_CONFIDENCE_DENSITY_ESTIMATE);CHECK(more[1].area==.15);
    };
    Active active({"test.bridge.final_probe"});REQUIRE_NOTHROW(Steps::StepFinalizeSlicing::run_step(Orchestrator::instance(),fixture.prepared.print));
}

TEST_CASE("Surface finalization stays coarse and independent of experimental activation","[plugins][bridge-surface-volume][bridge-internal]")
{
    for (const bool experimental : {false,true}) {
        Fixture fixture({{"bridge_volume_compensation","bridge.finalize_surface_volume"},{"fill_density","30%"},{"internal_bridge_expansion","0"}});
        fixture.internal(stPosTop);fixture.prepare();
        PrintObject &object=fixture.prepared.print.object(0);
        slic3r_api::SupportPipelineInfo &state=object.get_or_add_property<slic3r_api::SupportPipelineInfo>();state.demand_computed=1;state.material_generated=1;
        {
            Active active({"infill.generator.default","infill.recipe.bridge_preparation","surface.bridge_preparation","surface.solid_shells","rectilinear"});
            Steps::StepGenerateInfill::run_step(Orchestrator::instance(),fixture.prepared.print);
        }
        size_t rows=0;
        for (const LayerRegionIsland &group : fixture.island().regions_islands()) if (group.has_extrusion(LayerRegionIsland::INFILLS))
            visit_leaves(group.extrusion(LayerRegionIsland::INFILLS),[&](const ExtrusionEntity &,const ExtrusionAttributes &attr) { if (attr.role==ExtrusionRole::InternalBridgeInfill) ++rows; });
        REQUIRE(rows>0);
        Active active({"bridge.finalize_surface_volume","surface.bridge_preparation","infill.recipe.bridge_preparation","surface.solid_shells"});
        REQUIRE(Orchestrator::instance().set_plugin_active("bridge.finalize_volume",experimental));
        REQUIRE_NOTHROW(Steps::StepFinalizeSlicing::run_step(Orchestrator::instance(),fixture.prepared.print));
        size_t pieces=0;
        for (const LayerRegionIsland &group : fixture.island().regions_islands()) if (group.has_extrusion(LayerRegionIsland::INFILLS))
            visit_leaves(group.extrusion(LayerRegionIsland::INFILLS),[&](const ExtrusionEntity &leaf,const ExtrusionAttributes &attr) {
                if (attr.role!=ExtrusionRole::InternalBridgeInfill) return;
                const slic3r_api::BridgeVolumeInfo *stamp=leaf.get_property<slic3r_api::BridgeVolumeInfo>();REQUIRE(stamp!=nullptr);
                CHECK(stamp->confidence==BRIDGE_CONFIDENCE_DENSITY_ESTIMATE);CHECK(attr.width==stamp->nominal.width);
                CHECK(attr.mm3_per_mm<=stamp->nominal.mm3_per_mm);++pieces;
            });
        CHECK(pieces==rows);REQUIRE_NOTHROW(Steps::StepFinalizeSlicing::run_step(Orchestrator::instance(),fixture.prepared.print));
    }
}

TEST_CASE("Bridge axial expansion preserves rows and stops before holes","[plugins][bridge-preparation]")
{
    PluginStorage storage;storage_handle *handle=reinterpret_cast<storage_handle *>(&storage);
    for (const double angle : {0.,PI/6.,PI/2.,PI*.499999}) {
        INFO("angle "<<angle);
        ExPolygons core{rectangle_expolygon(-2,-1,2,1)},domain{rectangle_expolygon(-5,-1,5,1)},expected{rectangle_expolygon(-3,-1,3,1)};
        for (ExPolygons *areas : {&core,&domain,&expected}) for (ExPolygon &p : *areas) p.rotate(angle);
        slic3r_api::StoredExPolygonCollection expanded=BridgeGeometry::axial_expand(handle,view(core),view(domain),angle,scale_i(1.));
        same_area(expanded.readonly(),expected);
    }
    const ExPolygons core{rectangle_expolygon(0,0,1,2)};
    const ExPolygons domain{rectangle_expolygon(0,0,3,2),rectangle_expolygon(4,0,10,2)};
    slic3r_api::StoredExPolygonCollection expanded=BridgeGeometry::axial_expand(handle,view(core),view(domain),0.,scale_i(20.));
    same_area(expanded.readonly(),ExPolygons{rectangle_expolygon(0,0,3,2)});
    slic3r_api::StoredExPolygonCollection zero=BridgeGeometry::axial_expand(handle,view(core),view(domain),0.,0);
    same_area(zero.readonly(),core);

    ExPolygon holed=rectangle_expolygon(-4,-2,4,2);
    Polygon hole=rectangle_expolygon(-1,-1,1,1).contour;
    hole.reverse();holed.holes.push_back(std::move(hole));
    const ExPolygons seed{rectangle_expolygon(-3,-.5,-2,.5)},holed_domain{holed};
    slic3r_api::StoredExPolygonCollection stopped=BridgeGeometry::axial_expand(handle,view(seed),view(holed_domain),0.,scale_i(10.));
    same_area(stopped.readonly(),ExPolygons{rectangle_expolygon(-4,-.5,-1,.5)});

    const ExPolygons wedge{ExPolygon(Polygon(Points{Point(0,0),Point(scale_i(5.),0),Point(scale_i(1.),scale_i(2.)),Point(0,scale_i(2.))}))};
    const ExPolygons wedge_core{rectangle_expolygon(0,0,1,2)};
    const ExPolygons clipped_expected{ExPolygon(Polygon(Points{Point(0,0),Point(scale_i(2.),0),Point(scale_i(2.),scale_i(1.5)),Point(scale_i(1.),scale_i(2.)),Point(0,scale_i(2.))}))};
    slic3r_api::StoredExPolygonCollection clipped=BridgeGeometry::axial_expand(handle,view(wedge_core),view(wedge),0.,scale_i(1.));
    same_area(clipped.readonly(),clipped_expected);
}

// A 3 mm wide bridge holds exactly six rows of the 0.5 mm sparsest pitch, which the fit must find; a triangle has no
// whole number of rows, but searching the pitch range must still cover it at least as well as the densest pitch alone.
// Every fit stays in its pitch range and overflows at most what it leaves uncovered.
TEST_CASE("Bridge rows fit their pitch and phase to the bridge area","[plugins][bridge-preparation][bridge-rows]")
{
    PluginStorage storage;storage_handle *handle=reinterpret_cast<storage_handle *>(&storage);
    const BridgeRows::PitchRange range{.4/.9,.4/.8};
    const ExPolygon rectangle=rectangle_expolygon(0,0,10,3);
    // Rows along X (angle 0) are spaced along Y.
    const BridgeRows::Fit fit=BridgeRows::fit(handle,slic3r_api::ExPolygon(reinterpret_cast<const expolygon_handle *>(&rectangle)),0.,.4,range);
    REQUIRE(fit.found());
    CHECK(fit.pitch==Approx(.5).margin(1e-3));
    CHECK(fit.coverage>.98);
    CHECK(fit.overflow<.02);
    const ExPolygon triangle(Polygon(Points{Point(0,0),Point(scale_i(6.),0),Point(scale_i(1.6),scale_i(4.8))}));
    for (const double angle : {0.,PI/5.,PI/2.}) {
        INFO("angle "<<angle);
        const slic3r_api::ExPolygon view_triangle(reinterpret_cast<const expolygon_handle *>(&triangle));
        const BridgeRows::Fit best=BridgeRows::fit(handle,view_triangle,angle,.4,range);
        const BridgeRows::Fit densest=BridgeRows::fit(handle,view_triangle,angle,.4,BridgeRows::PitchRange{.4/.9,.4/.9});
        REQUIRE(best.found());REQUIRE(densest.found());
        CHECK(best.pitch>=range.minimum-1e-9);CHECK(best.pitch<=range.maximum+1e-9);
        INFO("best "<<best.coverage<<" at "<<best.pitch<<", densest "<<densest.coverage);
        CHECK(best.coverage>=densest.coverage-1e-9);
        CHECK(best.overflow<=1.-best.coverage+1e-9);
    }
    // Rows along the inner edge of an L must not spill over its corner: the sparsest 0.5 mm pitch covers more of it but
    // its third row overflows beyond what the rows leave uncovered, so the fit takes a denser pitch.
    const ExPolygon l_shape(Polygon(Points{Point(0,0),Point(scale_i(5.),0),Point(scale_i(5.),scale_i(1.3)),
        Point(scale_i(1.3),scale_i(1.3)),Point(scale_i(1.3),scale_i(4.5)),Point(0,scale_i(4.5))}));
    const BridgeRows::Fit l_fit=BridgeRows::fit(handle,slic3r_api::ExPolygon(reinterpret_cast<const expolygon_handle *>(&l_shape)),0.,.4,range);
    INFO("L "<<l_fit.coverage<<" overflow "<<l_fit.overflow<<" at "<<l_fit.pitch);
    REQUIRE(l_fit.found());
    CHECK(l_fit.overflow<=1.-l_fit.coverage+1e-9);
    CHECK(l_fit.pitch<range.maximum-1e-3);
}

// Core and allowed domain of a sliced part's bridge printed along Y. A core edge ends at
// a domain vertex almost along a domain edge: uniting them cuts a thin sliver off the core.
TEST_CASE("Axial bridge expansion accepts a core sliver lost by uniting it with its domain","[plugins][bridge-preparation]")
{
    const ExPolygons core{
        ExPolygon(
            Polygon(Points{
                {-8281304,20952346},{-8376684,21083396},{-8389019,21098106},{-8392593,21095340},{-8398240,21089679},
                {-8403118,21083343},{-8407147,21076437},{-8410262,21069072},{-8412410,21061370},{-8413558,21053457},
                {-8413686,21045462},{-8412792,21037517},{-8410891,21029750},{-8408015,21022290},{-8402510,21012691},
                {-8314902,20887166},{-8309915,20880915},{-8304172,20875352},{-8297765,20870568},{-8290800,20866641},
                {-8283390,20863635},{-8275658,20861600},{-8267729,20860569},{-8259733,20860559},{-8251801,20861569},
                {-8244063,20863584},{-8236646,20866571},{-8235622,20867145},{-8250445,20897066},{-8271630,20938569}}))};
    const ExPolygons domain{
        ExPolygon(
            Polygon(Points{
                {-8281304,20952346},{-8376684,21083396},{-12074783,25493376},{-12098476,25515783},{-12124889,25538809},
                {-12188383,25567319},{-12271614,25581313},{-12372011,25575516},{-12487410,25548047},{-12540401,25526717},
                {-12610316,25497924},{-12736090,25425959},{-12863334,25330971},{-22702735,17079895},{-22780520,17006678},
                {-22820877,16954215},{-22910122,16859287},{-22980549,16748236},{-23018317,16663418},{-23027745,16640958},
                {-23049661,16543058},{-23050313,16458879},{-23033261,16391392},{-22996917,16331048},{-19300182,11922698},
                {-19230180,11849054},{-19175989,11793778},{-19033526,11687219},{-18871086,11599446},{-18689825,11531682},
                {-18650313,11522398},{-18490539,11487066},{-18419728,11479874},{-18272026,11466930},{-18035545,11471101},
                {-17904960,11485821},{-17785611,11500926},{-17726003,11513659},{-17598300,11542590},{-17514540,11562458},
                {-17241566,11636097},{-16820224,11758940},{-16687420,11798467},{-16552791,11842908},{-15980805,12030675},
                {-15323325,12275774},{-14718598,12530588},{-14158536,12796257},{-13636545,13074011},{-13147022,13365289},
                {-12685208,13671727},{-12246966,13995221},{-11828727,14337932},{-11427244,14702428},{-11039682,15091612},
                {-10663326,15509003},{-10295743,15958621},{-9934483,16445366},{-9577173,16975059},{-9221420,17554668},
                {-8864621,18192845},{-8779213,18360361},{-8555743,18795375},{-8369865,19180518},{-8347595,19227891},
                {-8332546,19261057},{-8228701,19515302},{-8151784,19760955},{-8143205,19801236},{-8108947,19988158},
                {-8092284,20207035},{-8100654,20413456},{-8133442,20605576},{-8168107,20714038},{-8186371,20767734},
                {-8250445,20897066},{-8271630,20938569}}),
            Polygons{
            Polygon(Points{
                    {-17259316,13405879},{-17304426,13429379},{-17365568,13448221},{-17419022,13495113},{-19132600,15538550},
                    {-19078814,15583654},{-19159132,15679433},{-12102498,21596953},{-12022179,21501173},{-11964612,21549444},
                    {-10251038,19506009},{-10243136,19493822},{-10206947,19429677},{-10196770,19373556},{-10190325,19333756},
                    {-10202316,19220628},{-10223939,19151214},{-10235415,19062160},{-10285362,18912603},{-10294393,18894688},
                    {-10369152,18754494},{-10425578,18658869},{-10471920,18584281},{-10684550,18248668},{-11023726,17751993},
                    {-11059661,17702768},{-11183523,17538855},{-11416188,17229560},{-11807911,16758736},{-12210332,16322023},
                    {-12626222,15915136},{-13058534,15534466},{-13510519,15176944},{-13985761,14839967},{-14488398,14521271},
                    {-15023203,14218891},{-15321874,14068777},{-15580790,13938284},{-15679921,13892669},{-16068094,13718650},
                    {-16352224,13599150},{-16471526,13551963},{-16659255,13479569},{-16826401,13425930},{-16925541,13408874},
                    {-16989098,13400685},{-17078503,13406780},{-17145633,13398128}})}),
        ExPolygon(
            Polygon(Points{
                {-8190250,20779139},{-8250445,20897066},{-8186371,20767735}}))};
    PluginStorage storage;storage_handle *handle=reinterpret_cast<storage_handle *>(&storage);
    const auto expand=[&]() { return BridgeGeometry::axial_expand(handle,view(core),view(domain),PI/2.,500000); };
    REQUIRE_NOTHROW(expand());
    const slic3r_api::StoredExPolygonCollection expanded=expand();
    const ExPolygons &result=*reinterpret_cast<const ExPolygons *>(expanded.readonly().handle());
    // The exact core is kept, extended along Y, and the extension stays in the allowed area.
    CHECK(diff_ex(core,result).empty());
    CHECK(area(result)>area(core));
    ExPolygons allowed=domain;append(allowed,core);
    CHECK(diff_ex(result,offset_ex(union_ex(allowed),10.f)).empty());
}

// Bridge core, island slice and lower slices of a sliced part, with the bridge printed
// along X. The long edges of the core run almost along X, nearly parallel to the slice
// edges that continue them, so the expansion booleans add or drop sub-epsilon slivers.
TEST_CASE("Bridge preparation tolerates sub-epsilon slivers along nearly axial core edges","[plugins][bridge-preparation]")
{
    const ExPolygons core{
        ExPolygon(
            Polygon(Points{
                {-41499774,796166},{-49499734,798629},{-49500213,-757967},{-41500253,-760431}}))};
    const ExPolygons domain{
        ExPolygon(
            Polygon(Points{
                {-41378402,-10983169},{-41370685,-11101444},{-41348400,-11213663},{-41311649,-11322037},
                {-41261068,-11424681},{-41197534,-11519832},{-41122116,-11605882},{-41036112,-11681354},
                {-40940997,-11744948},{-40838380,-11795594},{-40730039,-11832410},{-40617833,-11854763},
                {-40499564,-11862551},{-40310664,-11862609},{-40264882,-11857465},{-40228054,-11844591},
                {-40195011,-11823844},{-40167414,-11796263},{-40146648,-11763236},{-40133750,-11726409},
                {-40128572,-11680626},{-40121378,11705346},{-40126519,11751117},{-40139397,11787949},{-40160146,11820998},
                {-40187723,11848590},{-40220753,11869358},{-40257577,11882257},{-40303348,11887428},{-40492248,11887487},
                {-40610522,11879771},{-40722749,11857484},{-40831112,11820736},{-40933761,11770155},{-41028906,11706622},
                {-41114966,11631196},{-41190435,11545198},{-41254025,11450087},{-41304674,11347464},{-41341490,11239127},
                {-41363839,11126917},{-41371628,11008655},{-41374774,796128},{-49624733,798667},{-49623641,4348525},
                {-49632800,4552200},{-49641338,4613616},{-49659141,4731537},{-49701460,4890433},{-49756825,5025798},
                {-49779264,5062737},{-49804365,5101144},{-50158750,4748424},{-50163567,4744610},{-50170419,4740489},
                {-50177741,4737276},{-50185414,4735024},{-50193311,4733771},{-50201303,4733535},{-50209260,4734322},
                {-50217052,4736119},{-50224550,4738896},{-50231632,4742607},{-50278250,4770976},{-50303106,4775892},
                {-50358489,4786073},{-52323443,4786678},{-52337944,4732362},{-52355833,4614389},{-52364403,4553045},
                {-52373681,4349379},{-52376350,-4317282},{-52367201,-4521193},{-52355971,-4600699},{-52340709,-4704434},
                {-52298808,-4858741},{-52243677,-4990491},{-52209379,-5045688},{-51663944,-5045688},{-51655902,-5046412},
                {-51648170,-5048113},{-51640644,-5050814},{-51632080,-5055313},{-51576567,-5088334},{-51495596,-5103988},
                {-50505953,-5104293},{-50426573,-5088996},{-50369599,-5055152},{-50359410,-5050285},{-50351750,-5047991},
                {-50343860,-5046692},{-50335869,-5046412},{-49793720,-5046412},{-49759400,-4991259},{-49704193,-4859539},
                {-49662186,-4705259},{-49646857,-4601476},{-49635586,-4522038},{-49626310,-4318128},{-49625213,-757929},
                {-41375254,-760469}}))};
    const ExPolygons lower{
        ExPolygon(
            Polygon(Points{
                {-40236927,-11980111},{-40173510,-11957941},{-40116610,-11922214},{-40069092,-11874724},
                {-40033328,-11817845},{-40011123,-11754442},{-40003575,-11687692},{-39996375,11712322},
                {-40003875,11779076},{-40026048,11842493},{-40061772,11899394},{-40109266,11946914},{-40166142,11982675},
                {-40229543,12004885},{-40296293,12012426},{-40496296,12012488},{-40626836,12003972},{-40755134,11978494},
                {-40879015,11936484},{-40996357,11878663},{-41105140,11806023},{-41203513,11719806},{-41289794,11621487},
                {-41362495,11512748},{-41420389,11395444},{-41462481,11271584},{-41488030,11143305},{-41496627,11012781},
                {-41503403,-10987219},{-41494886,-11117757},{-41469409,-11246052},{-41427397,-11369938},
                {-41369574,-11487280},{-41296938,-11596061},{-41210721,-11694434},{-41112404,-11780711},
                {-41003661,-11853417},{-40886359,-11911310},{-40762502,-11953398},{-40634220,-11978954},
                {-40503689,-11987550},{-40303687,-11987612}})),
        ExPolygon(
            Polygon(Points{
                {-49904034,-5148303},{-49810218,-5092573},{-49723740,-5002052},{-49647930,-4880223},{-49616459,-4805133},
                {-49585708,-4728096},{-49571674,-4674157},{-49539439,-4545112},{-49510929,-4344152},{-49501249,-4123383},
                {-49498701,4154545},{-49508248,4370254},{-49536631,4574417},{-49564035,4686149},{-49582783,4756545},
                {-49644918,4908455},{-49720652,5033132},{-49807070,5125785},{-49900854,5182855},{-49937036,5190011},
                {-49998397,5201291},{-51998405,5201907},{-52059773,5190665},{-52095960,5183533},{-52189778,5126519},
                {-52276254,5033918},{-52352061,4909291},{-52414289,4757415},{-52433080,4687033},{-52460551,4575317},
                {-52489068,4371172},{-52498741,4155475},{-52501289,-4122453},{-52491749,-4343235},{-52463359,-4544223},
                {-52431209,-4673275},{-52417213,-4727223},{-52386504,-4804280},{-52355076,-4879389},{-52279346,-5001265},
                {-52192926,-5091840},{-52099141,-5147627},{-52001593,-5166486},{-50001589,-5167102}}))};
    Fixture fixture({{"bridged_infill_margin","0.5"},{"bridged_infill_uniform_margin","0.0625"}});
    PrintObject &object=fixture.prepared.print.object(0);
    for (Layer &layer : object.layers()) for (LayerSliceIsland &island : layer.islands()) {
        island.overlaps_above.clear();island.overlaps_below.clear();
    }
    for (size_t idx=0;idx<2;++idx) {
        Layer &layer=object.layer(idx);
        const ExPolygons &areas=idx==0 ? lower : domain;
        ApiInternal::LayerRegionAccess::slices_mutable(layer.region(0))=areas;
        ApiInternal::LayerAccess::set_islands(layer,ExPolygons(areas));
        for (LayerSliceIsland &island : layer.islands()) {
            island.fill_regions(layer);ApiInternal::LayerIslandAccess::set_infill_areas(island,ExPolygons{island.get_slice()});
        }
    }
    Layer::build_up_down_graph(object.layer(0),object.layer(1));
    SurfaceCollection &input=fixture.island().get_or_add_region_island(fixture.island().regions(),0).set_fill_surfaces();
    input.append(core,stPosBottom|stDensSolid|stModBridge);
    input.append(diff_ex(domain,core),stPosInternal|stDensSparse);
    REQUIRE_NOTHROW(fixture.prepare());
    // The bridge keeps its core and gains anchors over the lower slices.
    const ExPolygons bridge{fixture.bridge().expolygon};
    CHECK(area(bridge)>area(core));
    CHECK(area(diff_ex(core,bridge))<double(SCALED_EPSILON)*SCALED_EPSILON);
}

// A sparse source surface of a sliced part reduced to a sliver: a triangle minus an almost
// identical hole, thinner than the boolean precision, moved next to the bridge.
TEST_CASE("Bridge preparation drops a degenerate source sliver outside every region intersection","[plugins][bridge-preparation]")
{
    Fixture fixture;
    const ExPolygon sliver(Polygon(Points{{5010795,-6425},{4987188,6626},{4989156,4823}}),Polygon(Points{{4989156,4823},{4987188,6626},{5009807,-5911}}));
    fixture.island().get_or_add_region_island(fixture.island().regions(),0).set_fill_surfaces().append(
        ExPolygons{sliver},stPosInternal|stDensSparse);
    REQUIRE_NOTHROW(fixture.prepare());
    same_area(view(ExPolygons{fixture.bridge().expolygon}),ExPolygons{rectangle_expolygon(-3,-1,3,1)});
}

TEST_CASE("Uniform bridge expansion adds side area before axial anchors and clips detached receivers","[plugins][bridge-preparation][bridge-uniform]")
{
    PluginStorage storage;storage_handle *handle=reinterpret_cast<storage_handle *>(&storage);
    for (const double angle : {0.,PI/6.,PI/2.}) {
        INFO(angle);
        ExPolygons core{rectangle_expolygon(-2,-1,2,1)},domain{rectangle_expolygon(-8,-4,8,4)};
        for (ExPolygons *areas : {&core,&domain}) for (ExPolygon &area : *areas) area.rotate(angle);
        slic3r_api::StoredExPolygonCollection expanded=BridgeGeometry::expand_bridge(handle,view(core),view(domain),angle,scale_i(1.),scale_i(.5));
        const ExPolygons &native=*reinterpret_cast<const ExPolygons *>(expanded.readonly().handle());
        REQUIRE(native.size()==1);
        // A round .5 mm offset adds perimeter*r + pi*r*r. The following axial
        // sweep adds 2*1 mm times the widened height of 3 mm.
        CHECK(native.front().area()*SCALING_FACTOR*SCALING_FACTOR==Approx(8.+12.*.5+PI*.25+6.).margin(.002));
        ExPolygons aligned=native;
        for (ExPolygon &area : aligned) area.rotate(-angle);
        const BoundingBox bounds=get_extents(aligned);
        CHECK(unscaled(bounds.min.x())==Approx(-3.5).margin(unscaled(SCALED_EPSILON)));
        CHECK(unscaled(bounds.max.x())==Approx(3.5).margin(unscaled(SCALED_EPSILON)));
        CHECK(unscaled(bounds.min.y())==Approx(-1.5).margin(unscaled(SCALED_EPSILON)));
        CHECK(unscaled(bounds.max.y())==Approx(1.5).margin(unscaled(SCALED_EPSILON)));
        CHECK(diff_ex(core,native).empty());CHECK(diff_ex(native,domain).empty());
    }
    const ExPolygons core{rectangle_expolygon(0,0,1,2)};
    ExPolygon allowed=rectangle_expolygon(0,-1,3,3);
    Polygon hole=rectangle_expolygon(1.5,.5,2.5,1.5).contour;hole.reverse();allowed.holes.push_back(hole);
    const ExPolygons domain{allowed,rectangle_expolygon(4,-1,10,3)};
    slic3r_api::StoredExPolygonCollection expanded=BridgeGeometry::expand_bridge(handle,view(core),view(domain),0.,scale_i(10.),scale_i(10.));
    same_area(expanded.readonly(),ExPolygons{allowed});
    slic3r_api::StoredExPolygonCollection axial=BridgeGeometry::expand_bridge(handle,view(core),view(domain),0.,scale_i(1.),0);
    slic3r_api::StoredExPolygonCollection original=BridgeGeometry::axial_expand(handle,view(core),view(domain),0.,scale_i(1.));
    CHECK(BridgeGeometry::geometry_key(axial.readonly())==BridgeGeometry::geometry_key(original.readonly()));
}

TEST_CASE("Uniform bridge margins use external perimeter width and preserve the complete frame partition","[plugins][bridge-preparation][bridge-uniform]")
{
    for (const std::pair<const char *,double> setting : {std::pair<const char *,double>{"0",0.},{"25%",.2},{"0.3",.3},{"100%",.8}}) {
        INFO(setting.first);
        Fixture fixture({{"bridged_infill_uniform_margin",setting.first},{"external_perimeter_extrusion_width","0.8"}});
        fixture.frame();fixture.prepare();
        const Surface &bridge=fixture.bridge();
        const BoundingBox bounds=get_extents(bridge.expolygon);
        CHECK(unscaled(bounds.min.x())==Approx(-3.-setting.second).margin(unscaled(SCALED_EPSILON)));
        CHECK(unscaled(bounds.max.x())==Approx(3.+setting.second).margin(unscaled(SCALED_EPSILON)));
        CHECK(unscaled(bounds.min.y())==Approx(-1.-setting.second).margin(unscaled(SCALED_EPSILON)));
        CHECK(unscaled(bounds.max.y())==Approx(1.+setting.second).margin(unscaled(SCALED_EPSILON)));
        CHECK(bridge.priority==7);CHECK(bridge.thickness_layers==2);CHECK(bridge.extra_perimeters==3);CHECK(bridge.maxNbSolidLayersOnTop==9);
        const LayerSliceIsland &lower=fixture.prepared.print.object(0).layer(0).island(0);
        same_area(view(diff_ex(ExPolygons{bridge.expolygon},ExPolygons{lower.get_slice()})),ExPolygons{rectangle_expolygon(-2,-1,2,1)});
        ExPolygons published;
        double sum=0.;
        for (const LayerRegionIsland &region : fixture.island().regions_islands())
            for (const Surface &surface : region.fill_surfaces()) { published.push_back(surface.expolygon);sum+=surface.expolygon.area(); }
        same_area(view(published),ExPolygons{rectangle_expolygon(-8,-4,8,4)});
        CHECK(sum*SCALING_FACTOR*SCALING_FACTOR==Approx(128.).margin(1e-6));
        const Surface expected=bridge;
        Active active({"surface.top_surface_expansion","surface.clean_infill_surfaces","surface.infill_region_compatibility_splitter","dense_infill.surface_marker"});
        Steps::StepSurfaceGeneration::run_step(Orchestrator::instance(),fixture.prepared.print);
        same_area(view(ExPolygons{fixture.bridge().expolygon}),ExPolygons{expected.expolygon});
        CHECK(fixture.bridge().properties_equal(expected));CHECK(fixture.bridge().bridge_angle==expected.bridge_angle);
    }
}

TEST_CASE("Hollow bridge margins share the existing shell cap and both settings affect their signature","[plugins][bridge-preparation][bridge-uniform]")
{
    for (const char *count : {"0","1","2"}) {
        Fixture fixture({{"perimeters",count},{"fill_density","0%"},{"bridged_infill_margin","10"},{"bridged_infill_uniform_margin","0.2"}});
        fixture.frame();fixture.prepare();
        const LayerRegion &region=fixture.prepared.print.object(0).layer(1).region(0);
        const Flow external=region.flow(frExternalPerimeter),internal=region.flow(frPerimeter);
        const int perimeters=std::stoi(count);
        const coord_t shell=perimeters==0 ? 0 : coord_t(.5*external.scaled_width()+external.scaled_spacing())+(perimeters-1)*internal.scaled_spacing();
        const BoundingBox bounds=get_extents(fixture.bridge().expolygon);
        CHECK(bounds.max.x()==scale_i(2.)+shell);
        CHECK(bounds.max.y()==scale_i(1.)+std::min(scale_i(.2),shell));
    }
    uint64_t signature=0;
    for (const bool swapped : {false,true}) {
        Fixture fixture({{"bridged_infill_margin",swapped ? "0.5" : "1"},{"bridged_infill_uniform_margin",swapped ? "1" : "0.5"}});
        fixture.frame();fixture.prepare();
        const uint64_t current=fixture.bridge().get_property<slic3r_api::BridgeSurfaceInfo>()->policy_signature;
        if (swapped) CHECK(current!=signature);else signature=current;
    }
}

TEST_CASE("Uniform bridge overlap groups compatible neighbors and arbitrates incompatible receivers once","[plugins][bridge-preparation][bridge-uniform]")
{
    for (const bool incompatible : {false,true}) {
        uint64_t key=0;
        for (const bool reversed : {false,true}) {
            Fixture fixture({{"bridged_infill_margin","0"},{"bridged_infill_uniform_margin","1.1"}});
            ExPolygons cores{rectangle_expolygon(-2,-2,2,-1),rectangle_expolygon(-2,1,2,2)};
            if (reversed) std::reverse(cores.begin(),cores.end());
            fixture.frame(cores);
            if (incompatible) for (Surface &surface : fixture.island().regions_islands().front().set_fill_surfaces())
                if ((surface.surface_type&stModBridge)!=stNone && get_extents(surface.expolygon).min.y()>0) surface.priority=8;
            fixture.prepare();
            size_t count=0;ExPolygons bridges,published;double sum=0.;
            for (const LayerRegionIsland &region : fixture.island().regions_islands())
                for (const Surface &surface : region.fill_surfaces()) {
                    published.push_back(surface.expolygon);sum+=surface.expolygon.area();
                    if (surface.get_property<slic3r_api::BridgeSurfaceInfo>()!=nullptr) { bridges.push_back(surface.expolygon);++count; }
                }
            CHECK(count==(incompatible ? 2 : 1));
            const ExPolygons lower=diff_ex(ExPolygons{rectangle_expolygon(-8,-4,8,4)},cores);
            same_area(view(diff_ex(bridges,lower)),cores);
            same_area(view(published),ExPolygons{rectangle_expolygon(-8,-4,8,4)});
            CHECK(sum*SCALING_FACTOR*SCALING_FACTOR==Approx(128.).margin(1e-6));
            const uint64_t current=BridgeGeometry::geometry_key(view(bridges));
            if (reversed) CHECK(current==key);else key=current;
        }
    }
}

TEST_CASE("Peripheral bridge rows use actual lower contacts and local requests in both compensation modes","[plugins][bridge-preparation][bridge-volume][bridge-uniform]")
{
    for (const bool enabled : {false,true}) for (int scenario=0;scenario<7;++scenario) {
        INFO("compensation "<<enabled<<" scenario "<<scenario);
        Fixture fixture({{"bridged_infill_uniform_margin","0.6"},{"bridge_volume_compensation_enabled",enabled ? "1" : "0"}});
        fixture.frame();fixture.prepare();fixture.add_paths(false);
        LayerRegionIsland &upper=fixture.island().regions_islands().front();
        upper.mutable_extrusion(LayerRegionIsland::INFILLS).clear();
        ExtrusionEntity path(true,ArcPolyline(Points{Point(scale_i(-3.),scale_i(1.5)),Point(scale_i(3.),scale_i(1.5))}));
        path.add_property(ExtrusionAttributes(ExtrusionRole::BridgeInfill,Flow::bridging_flow(.4f,.4f)));
        path.get_or_add_property<ExtrusionPropertyInfill>().source_surface_id=fixture.bridge().id();
        upper.mutable_extrusion(LayerRegionIsland::INFILLS).append_child(std::move(path));
        if (scenario<2 || scenario==6) {
            LayerSliceIsland &lower=fixture.prepared.print.object(0).layer(0).island(0);
            LayerRegionIsland &owner=lower.get_or_add_region_island(lower.regions(),0);
            const std::vector<std::pair<double,double>> ranges=scenario==0 ? std::vector<std::pair<double,double>>{{-4.,4.}} :
                scenario==6 ? std::vector<std::pair<double,double>>{{-2.,2.}} : std::vector<std::pair<double,double>>{{-4.,-2.},{2.,4.}};
            for (const std::pair<double,double> &range : ranges) {
                ExtrusionEntity support(true,ArcPolyline(Points{Point(scale_i(range.first),scale_i(1.5)),Point(scale_i(range.second),scale_i(1.5))}));
                support.add_property(ExtrusionAttributes(ExtrusionRole::InternalInfill,ExtrusionFlow(.08,.4f,.2f)));
                owner.mutable_extrusion(LayerRegionIsland::INFILLS).append_child(std::move(support));
            }
        }
        if (scenario==3 || scenario==4)
            ApiInternal::LayerIslandAccess::support_demand_mutable(fixture.island())=ExPolygons{rectangle_expolygon(-4,1,scenario==3 ? 0. : 4.,2)};
        if (scenario==5) fixture.prepared.print.object(0).get_or_add_property<slic3r_api::SupportPipelineInfo>().material_generated=0;
        Active active({"bridge.finalize_volume","surface.bridge_preparation","infill.recipe.bridge_preparation","surface.solid_shells"});
        if (scenario==2 || scenario==3) {
            CHECK(finalize_skipped(fixture.prepared.print).size()==1);
            CHECK(upper.extrusion(LayerRegionIsland::INFILLS).child(0).get_property<slic3r_api::BridgeVolumeInfo>()==nullptr);
            continue;
        }
        REQUIRE_NOTHROW(Steps::StepFinalizeSlicing::run_step(Orchestrator::instance(),fixture.prepared.print));
        size_t reduced=0,nominal=0;
        visit_leaves(upper.extrusion(LayerRegionIsland::INFILLS),[&](const ExtrusionEntity &leaf,const ExtrusionAttributes &attributes) {
            CHECK(attributes.width==Approx(.4));CHECK(leaf.first_point().y()==scale_i(1.5));CHECK(leaf.last_point().y()==scale_i(1.5));
            if (attributes.height<.399) ++reduced;else ++nominal;
            const slic3r_api::BridgeVolumeInfo *stamp=leaf.get_property<slic3r_api::BridgeVolumeInfo>();
            if (enabled) { REQUIRE(stamp!=nullptr);CHECK((stamp->confidence==BRIDGE_CONFIDENCE_UNKNOWN)==(scenario==5)); }
            else CHECK(stamp==nullptr);
        });
        CHECK((reduced>0)==(enabled && (scenario<2 || scenario==6)));
        CHECK((nominal>0)==(!enabled || scenario!=0));
        REQUIRE_NOTHROW(Steps::StepFinalizeSlicing::run_step(Orchestrator::instance(),fixture.prepared.print));
    }
}

TEST_CASE("A peripheral bridge row outside its surface or the uniform source area stays uncorrected","[plugins][bridge-volume][bridge-uniform]")
{
    for (const char *provider : {"bridge.finalize_surface_volume","bridge.finalize_volume"}) for (int scenario=0;scenario<3;++scenario) {
        INFO(provider<<" scenario "<<scenario);
        Fixture fixture({{"bridged_infill_uniform_margin",scenario==2 ? "0" : "0.6"},{"bridge_volume_compensation",provider}});
        fixture.frame();fixture.prepare();fixture.add_paths(false);
        LayerRegionIsland &owner=fixture.island().regions_islands().front();
        owner.mutable_extrusion(LayerRegionIsland::INFILLS).clear();
        const double y=scenario==0 ? 3. : 0.;
        const double a=scenario==0 ? -3. : scenario==1 ? -3.4 : -2.9;
        const double b=scenario==0 ? 3. : scenario==1 ? -2.9 : -2.3;
        ExtrusionEntity path(true,ArcPolyline(Points{Point(scale_i(a),scale_i(y)),Point(scale_i(b),scale_i(y))}));
        path.add_property(ExtrusionAttributes(ExtrusionRole::BridgeInfill,Flow::bridging_flow(.4f,.4f)));
        path.get_or_add_property<ExtrusionPropertyInfill>().source_surface_id=fixture.bridge().id();
        owner.mutable_extrusion(LayerRegionIsland::INFILLS).append_child(std::move(path));
        ApiInternal::LayerIslandAccess::support_demand_mutable(fixture.island())=ExPolygons{rectangle_expolygon(-8,-4,8,4)};
        Active active({provider,"surface.bridge_preparation","infill.recipe.bridge_preparation","surface.solid_shells"});
        CHECK(finalize_skipped(fixture.prepared.print).size()==1);
        CHECK(owner.extrusion(LayerRegionIsland::INFILLS).child(0).get_property<slic3r_api::BridgeVolumeInfo>()==nullptr);
    }
}

TEST_CASE("Uniform bridge rows keep their region settings and cannot consume protected surfaces","[plugins][bridge-preparation][bridge-uniform]")
{
    Fixture fixture({{"bridged_infill_uniform_margin","0.6"}});fixture.frame();
    Layer &layer=fixture.prepared.print.object(0).layer(1);
    SurfaceCollection original=fixture.island().regions_islands().front().fill_surfaces();
    // A nearby exposed top remains outside both margins' receiver set.
    const ExPolygons top{rectangle_expolygon(-1,1.1,1,1.4)};
    LayerSliceIsland &below=fixture.prepared.print.object(0).layer(0).island(0);
    SurfaceCollection &lower_surfaces=below.get_or_add_region_island(below.regions(),0).set_fill_surfaces();
    lower_surfaces.append(top,stPosInternal|stDensSolid);
    lower_surfaces.append(diff_ex(below.infill_areas(),top),stPosInternal|stDensSparse);
    SurfaceCollection input;
    for (const Surface &surface : original) {
        const ExPolygons remaining=diff_ex(ExPolygons{surface.expolygon},top);
        for (const ExPolygon &area : remaining) input.surfaces.emplace_back(surface,area);
    }
    input.append(top,stPosTop|stDensSolid);
    PrintRegionConfig right=layer.region(0).region().config();
    right.set_deserialize_strict("bridged_infill_uniform_margin","0.3");right.set_deserialize_strict("bridge_flow_ratio","150%");
    ApiInternal::LayerAccess::add_region(layer,fixture.prepared.print.register_print_region(right));
    ApiInternal::LayerRegionAccess::slices_mutable(layer.region(0))=ExPolygons{rectangle_expolygon(-8,-4,0,4)};
    ApiInternal::LayerRegionAccess::slices_mutable(layer.region(1))=ExPolygons{rectangle_expolygon(0,-4,8,4)};
    fixture.island().mutable_regions_islands().clear();fixture.island().fill_regions(layer);
    fixture.island().get_or_add_region_island(fixture.island().regions(),0).set_fill_surfaces().append(input);
    fixture.prepare();
    size_t count=0;ExPolygons actual_top;
    for (const LayerRegionIsland &group : fixture.island().regions_islands())
        for (const Surface &surface : group.fill_surfaces()) {
            if (surface.surface_type==(stPosTop|stDensSolid)) actual_top.push_back(surface.expolygon);
            if (surface.get_property<slic3r_api::BridgeSurfaceInfo>()==nullptr) continue;
            REQUIRE(group.regions().size()==1);
            const LayerRegion &owner=**group.regions().begin();
            CHECK(diff_ex(ExPolygons{surface.expolygon},owner.get_raw_slices()).empty());
            CHECK(intersection_ex(ExPolygons{surface.expolygon},top).empty());
            const bool left=&owner==&layer.region(0);
            const BoundingBox bounds=get_extents(surface.expolygon);
            CHECK(unscaled(bounds.min.y())==Approx(left ? -1.6 : -1.3).margin(unscaled(SCALED_EPSILON)));
            ++count;
        }
    CHECK(count==2);same_area(view(actual_top),top);
}

TEST_CASE("Real bridge patterns generate and finalize the new peripheral rows","[plugins][bridge-preparation][bridge-volume][bridge-uniform]")
{
    for (const char *pattern : {"rectilinear","monotonic"}) {
        INFO(pattern);
        Fixture fixture({{"bridged_infill_uniform_margin","0.6"},{"fill_density","100%"},{"fill_pattern","rectilinear"},{"bridge_fill_pattern",pattern}});
        fixture.frame();fixture.prepare();fixture.add_paths(false);
        for (Layer &layer : fixture.prepared.print.object(0).layers())
            for (LayerSliceIsland &island : layer.islands())
                for (LayerRegionIsland &region : island.regions_islands())
                    if (region.has_extrusion(LayerRegionIsland::INFILLS)) region.mutable_extrusion(LayerRegionIsland::INFILLS).clear();
        Active active({"bridge.finalize_volume","surface.bridge_preparation","infill.recipe.bridge_preparation","surface.solid_shells",
            "infill.generator.default","rectilinear","monotonic","infill.post_process.full_fill_volume","infill.post_process.gap_fill"});
        Steps::StepGenerateInfill::run_step(Orchestrator::instance(),fixture.prepared.print);
        Steps::StepPostInfillGeneration::run_step(Orchestrator::instance(),fixture.prepared.print);
        size_t peripheral=0;
        for (const LayerRegionIsland &region : fixture.island().regions_islands()) if (region.has_extrusion(LayerRegionIsland::INFILLS))
            visit_leaves(region.extrusion(LayerRegionIsland::INFILLS),[&](const ExtrusionEntity &leaf,const ExtrusionAttributes &attributes) {
                if (attributes.role==ExtrusionRole::BridgeInfill && std::abs(leaf.first_point().y())>scale_i(1.)) ++peripheral;
            });
        REQUIRE(peripheral>0);
        REQUIRE_NOTHROW(Steps::StepFinalizeSlicing::run_step(Orchestrator::instance(),fixture.prepared.print));
        size_t corrected=0;
        for (const LayerRegionIsland &region : fixture.island().regions_islands()) if (region.has_extrusion(LayerRegionIsland::INFILLS))
            visit_leaves(region.extrusion(LayerRegionIsland::INFILLS),[&](const ExtrusionEntity &leaf,const ExtrusionAttributes &attributes) {
                if (attributes.role!=ExtrusionRole::BridgeInfill || std::abs(leaf.first_point().y())<=scale_i(1.)) return;
                REQUIRE(leaf.get_property<slic3r_api::BridgeVolumeInfo>()!=nullptr);
                if (attributes.height<.399) ++corrected;
            });
        CHECK(corrected>0);
    }
}

TEST_CASE("Bridge clearance uses actual footprint and highest real Z","[plugins][bridge-volume]")
{
    Clearance::Index index;
    index.add({{2,-1},{2,1},.2,.6,.8,false});
    index.add({{2,-1},{2,1},.2,.4,.6,false});
    const std::vector<Clearance::Interval> intervals=index.clearance({0,0},{5,0},.4,.4,1.);
    REQUIRE(intervals.size()==3);
    CHECK(intervals[0].end==Approx(1.6));CHECK(intervals[1].end==Approx(2.4));
    CHECK(intervals[1].clearance==Approx(.2));CHECK(std::isinf(intervals[0].clearance));
    CHECK(index.contacts({0,0},{5,0},.4,1.).front().first==Approx(1.8));
    const std::vector<Clearance::Interval> hole=index.clearance({0,3},{5,3},.4,.4,1.);
    REQUIRE(hole.size()==1);CHECK(std::isinf(hole.front().clearance));
    index.add({{4,-1},{4,1},.1,.7,.9,true});
    const std::vector<Clearance::Interval> unknown=index.clearance({0,0},{5,0},.4,.4,1.);
    CHECK(std::any_of(unknown.begin(),unknown.end(),[](const Clearance::Interval &i) { return i.unknown; }));
}

TEST_CASE("Bridge sections keep an absolute monotone floor and symmetric minorant ramps","[plugins][bridge-volume]")
{
    CHECK(Clearance::area(.4,.2)==Approx(.0714159265359).epsilon(1e-10));
    CHECK(Clearance::height_for_area(.4,.0714159265359)==Approx(.2).epsilon(1e-10));
    const std::vector<Clearance::Interval> input{{0,2,std::numeric_limits<double>::infinity(),false},{2,4,.1,false},{4,6,std::numeric_limits<double>::infinity(),false}};
    const std::vector<Clearance::SectionPiece> pieces=Clearance::section_pieces(input,.6,.05);
    for (const Clearance::SectionPiece &piece : pieces) {
        const double midpoint=(piece.begin+piece.end)*.5;
        CHECK(piece.area>=Clearance::area(.6,.05)-1e-12);
        CHECK(piece.area<=Clearance::area(.6,.6)+1e-12);
        if (midpoint>=2. && midpoint<=4.) CHECK(piece.area<=Clearance::area(.6,.1)+1e-12);
        if (midpoint<1.4 || midpoint>4.6) CHECK(piece.area==Approx(Clearance::area(.6,.6)));
        const std::vector<Clearance::SectionPiece>::const_iterator mirror=std::find_if(pieces.begin(),pieces.end(),[midpoint](const Clearance::SectionPiece &p) { return 6.-midpoint>p.begin && 6.-midpoint<p.end; });
        REQUIRE(mirror!=pieces.end());CHECK(piece.area==Approx(mirror->area));
    }
    double previous=1.;
    for (const double gap : {.1,.05,.025,.001,0.}) {
        const std::vector<Clearance::SectionPiece> floor=Clearance::section_pieces({{0,1,gap,false}},.6,.05);
        REQUIRE(floor.size()==1);CHECK(floor.front().area<=previous);previous=floor.front().area;
        CHECK(floor.front().height==Approx(std::max(gap,.05)));
        CHECK(floor.front().confidence==(gap<.05 ? BRIDGE_CONFIDENCE_FLOOR_LIMITED : BRIDGE_CONFIDENCE_CERTIFIED));
    }
    const std::vector<Clearance::SectionPiece> unknown=Clearance::section_pieces({{0,1,.01,false},{1,2,0.,true}},.4,.05);
    CHECK(unknown.back().confidence==BRIDGE_CONFIDENCE_UNKNOWN);
    CHECK(unknown.back().area==Approx(PI*.4*.4/4.));
}

TEST_CASE("Sub-grid bridge sections extend a smaller neighbor without changing its flow","[plugins][bridge-volume][bridge-uniform]")
{
    const std::vector<Clearance::SectionPiece> input{
        {31.653776046662294,31.67378907837605,.13852195247390992,.33064851932593009,.4,.16162289269754676,BRIDGE_CONFIDENCE_CERTIFIED},
        {31.67378907837605,31.673813397828276,.14622226588178833,.35308748996479478,.4,.16162289269754676,BRIDGE_CONFIDENCE_CERTIFIED},
        {31.673813397828276,31.69381426981592,.14623162315492558,.35311514592836518,.4,.16162289269754676,BRIDGE_CONFIDENCE_CERTIFIED}};
    for (const bool reversed : {false,true}) {
        std::vector<Clearance::SectionPiece> pieces=input;
        if (reversed) {
            std::reverse(pieces.begin(),pieces.end());
            for (Clearance::SectionPiece &piece : pieces) {
                const double begin=piece.begin;piece.begin=-piece.end;piece.end=-begin;
            }
        }
        const std::vector<Clearance::SectionPiece> result=Clearance::coalesce_short_sections(pieces,2.*unscaled(SCALED_EPSILON));
        REQUIRE(result.size()==2);
        CHECK(result.front().begin==pieces.front().begin);CHECK(result.back().end==pieces.back().end);
        CHECK(result.front().end==result.back().begin);
        double old_volume=0.,new_volume=0.;
        for (const Clearance::SectionPiece &piece : pieces) old_volume+=(piece.end-piece.begin)*piece.area;
        for (const Clearance::SectionPiece &piece : result) {
            new_volume+=(piece.end-piece.begin)*piece.area;
            CHECK(piece.end-piece.begin>2.*unscaled(SCALED_EPSILON));
            CHECK(piece.area<=piece.cap);CHECK(piece.confidence==BRIDGE_CONFIDENCE_CERTIFIED);
        }
        CHECK(new_volume<=old_volume);
        CHECK(result.front().area==pieces.front().area);CHECK(result.back().area==pieces.back().area);
        CHECK(result.front().height==pieces.front().height);CHECK(result.back().height==pieces.back().height);
    }
    std::vector<Clearance::SectionPiece> unknown=input;
    unknown.front().confidence=BRIDGE_CONFIDENCE_UNKNOWN;
    CHECK(Clearance::coalesce_short_sections(unknown,2.*unscaled(SCALED_EPSILON)).size()==3);
}

TEST_CASE("Bridge angle writes require owned scratch and metadata stays complete","[plugins][bridge-preparation]")
{
    PluginStorage storage;storage_handle *handle=reinterpret_cast<storage_handle *>(&storage);
    SurfaceCollection borrowed;borrowed.append(ExPolygons{rectangle_expolygon(0,0,1,1)},stPosBottom|stDensSolid|stModBridge);
    CHECK_FALSE(surface_collection_set_bridge_angle(handle,reinterpret_cast<surface_collection_handle *>(&borrowed),0,0.));
    slic3r_api::StoredSurfaceCollection scratch(handle);
    scratch.append(slic3r_api::ExPolygon(reinterpret_cast<const expolygon_handle *>(&borrowed.surfaces.front().expolygon)),RAW_SURFACE_TYPE_POS_BOTTOM|RAW_SURFACE_TYPE_DENS_SOLID|RAW_SURFACE_TYPE_MOD_BRIDGE);
    CHECK(scratch.set_bridge_angle(0,0.));CHECK(surface_get_bridge_angle(scratch.readonly().front().handle())==0.);
    CHECK(scratch.set_bridge_angle(0,PI));CHECK(surface_get_bridge_angle(scratch.readonly().front().handle())==Approx(0.));
    CHECK_FALSE(scratch.set_bridge_angle(0,-1.));
    CHECK_FALSE(surface_collection_set_type(handle,reinterpret_cast<surface_collection_handle *>(&borrowed),0,RAW_SURFACE_TYPE_POS_INTERNAL|RAW_SURFACE_TYPE_DENS_SOLID));
    CHECK(scratch.set_type(0,RAW_SURFACE_TYPE_POS_INTERNAL|RAW_SURFACE_TYPE_DENS_SOLID|RAW_SURFACE_TYPE_MOD_BRIDGE));
    CHECK(scratch.readonly().front().type()==(RAW_SURFACE_TYPE_POS_INTERNAL|RAW_SURFACE_TYPE_DENS_SOLID|RAW_SURFACE_TYPE_MOD_BRIDGE));
    CHECK(scratch.readonly().front().bridge_angle()==0.);
    CHECK_FALSE(scratch.set_type(0,RAW_SURFACE_TYPE_POS_INTERNAL|RAW_SURFACE_TYPE_POS_TOP|RAW_SURFACE_TYPE_DENS_SOLID));
    CHECK_FALSE(scratch.set_type(0,RAW_SURFACE_TYPE_POS_INTERNAL));
    CHECK_FALSE(scratch.set_type(1,RAW_SURFACE_TYPE_POS_INTERNAL|RAW_SURFACE_TYPE_DENS_SOLID));
    Fixture fixture;fixture.prepare();
    Surface &prepared=fixture.bridge();
    CHECK(prepared.bridge_angle==0.);CHECK(prepared.priority==7);CHECK(prepared.thickness_layers==2);
    CHECK(prepared.extra_perimeters==3);CHECK(prepared.maxNbSolidLayersOnTop==9);
    const ExPolygons expected{rectangle_expolygon(-3,-1,3,1)};
    CHECK(diff_ex(ExPolygons{prepared.expolygon},expected).empty());CHECK(diff_ex(expected,ExPolygons{prepared.expolygon}).empty());
}

TEST_CASE("Bridge finalization is local coherent and idempotent","[plugins][bridge-volume]")
{
    Fixture fixture;fixture.prepare();fixture.add_paths();
    Active active({"bridge.finalize_volume","surface.bridge_preparation","infill.recipe.bridge_preparation","surface.solid_shells"});
    Steps::StepFinalizeSlicing::run_step(Orchestrator::instance(),fixture.prepared.print);
    double first_volume=0.;size_t count=0,corrected=0,nominal=0;
    for (const LayerRegionIsland &ri : fixture.island().regions_islands()) if (ri.has_extrusion(LayerRegionIsland::INFILLS))
        visit_leaves(ri.extrusion(LayerRegionIsland::INFILLS),[&](const ExtrusionEntity &leaf,const ExtrusionAttributes &attr) {
            const slic3r_api::BridgeVolumeInfo *stamp=leaf.get_property<slic3r_api::BridgeVolumeInfo>();REQUIRE(stamp!=nullptr);
            CHECK(stamp->correction_state==BRIDGE_CORRECTION_APPLIED);
            CHECK(attr.extrusion_role()==ExtrusionRole::BridgeInfill);CHECK(attr.width==Approx(.4));
            CHECK(attr.mm3_per_mm==Approx(attr.width*attr.height-(1.-PI/4.)*attr.height*attr.height));
            first_volume+=unscaled(leaf.length())*attr.mm3_per_mm;++count;
            if (attr.height<.399) ++corrected;else ++nominal;
        });
    CHECK(count>3);CHECK(corrected>0);CHECK(nominal>0);
    Steps::StepFinalizeSlicing::run_step(Orchestrator::instance(),fixture.prepared.print);
    double repeated=0.;
    for (const LayerRegionIsland &ri : fixture.island().regions_islands()) if (ri.has_extrusion(LayerRegionIsland::INFILLS))
        visit_leaves(ri.extrusion(LayerRegionIsland::INFILLS),[&](const ExtrusionEntity &leaf,const ExtrusionAttributes &attr) { repeated+=unscaled(leaf.length())*attr.mm3_per_mm; });
    CHECK(repeated==Approx(first_volume));
}

TEST_CASE("Bridge local support requests suppress missing-anchor diagnostics without inventing material","[plugins][bridge-volume]")
{
    Fixture fixture;fixture.prepare();fixture.add_paths(false);
    ApiInternal::LayerIslandAccess::support_demand_mutable(fixture.island())=ExPolygons{fixture.bridge().expolygon};
    Active active({"bridge.finalize_volume","surface.bridge_preparation","infill.recipe.bridge_preparation","surface.solid_shells"});
    REQUIRE_NOTHROW(Steps::StepFinalizeSlicing::run_step(Orchestrator::instance(),fixture.prepared.print));
    for (const LayerRegionIsland &ri : fixture.island().regions_islands()) if (ri.has_extrusion(LayerRegionIsland::INFILLS))
        visit_leaves(ri.extrusion(LayerRegionIsland::INFILLS),[](const ExtrusionEntity &,const ExtrusionAttributes &attr) { CHECK(attr.height==Approx(.4)); });
}

TEST_CASE("Prepared bridge recipes generate actual parallel rectilinear and monotonic strands","[plugins][bridge-preparation][bridge-volume]")
{
    for (const char *pattern : {"rectilinear","monotonic"}) {
        INFO(pattern);
        Fixture fixture({{"bridge_fill_pattern",pattern},{"enforce_full_fill_volume","1"}});
        fixture.prepare();
        fixture.add_paths(false);
        // Remove seed paths so the assertions inspect real pattern output rather
        // than synthetic geometry that already satisfies the strand shape.
        for (LayerRegionIsland &region : fixture.island().regions_islands())
            if (region.has_extrusion(LayerRegionIsland::INFILLS))
                region.mutable_extrusion(LayerRegionIsland::INFILLS).clear();
        ApiInternal::LayerIslandAccess::support_demand_mutable(fixture.island())=ExPolygons{fixture.bridge().expolygon};
        Active active({"bridge.finalize_volume","surface.bridge_preparation","infill.recipe.bridge_preparation","surface.solid_shells",
            "infill.generator.default","rectilinear","monotonic","infill.post_process.full_fill_volume","infill.post_process.gap_fill"});
        Steps::StepGenerateInfill::run_step(Orchestrator::instance(),fixture.prepared.print);
        Steps::StepPostInfillGeneration::run_step(Orchestrator::instance(),fixture.prepared.print);
        size_t strands=0;
        for (const LayerRegionIsland &region : fixture.island().regions_islands())
            if (region.has_extrusion(LayerRegionIsland::INFILLS))
                visit_leaves(region.extrusion(LayerRegionIsland::INFILLS),[&](const ExtrusionEntity &leaf,const ExtrusionAttributes &attr) {
                    if (!attr.extrusion_role().is_bridge()) return;
                    CHECK(leaf.first_point().y()==leaf.last_point().y());
                    CHECK(std::min(leaf.first_point().x(),leaf.last_point().x())<scale_i(-1.));
                    CHECK(std::max(leaf.first_point().x(),leaf.last_point().x())>scale_i(1.));
                    ++strands;
                });
        CHECK(strands>1);
        REQUIRE_NOTHROW(Steps::StepFinalizeSlicing::run_step(Orchestrator::instance(),fixture.prepared.print));
        size_t pieces=0;
        for (const LayerRegionIsland &region : fixture.island().regions_islands())
            if (region.has_extrusion(LayerRegionIsland::INFILLS))
                visit_leaves(region.extrusion(LayerRegionIsland::INFILLS),[&](const ExtrusionEntity &leaf,const ExtrusionAttributes &attr) {
                    if (!attr.extrusion_role().is_bridge()) return;
                    REQUIRE(leaf.get_property<slic3r_api::BridgeVolumeInfo>()!=nullptr);
                    CHECK(attr.mm3_per_mm>0.);
                    CHECK(attr.mm3_per_mm<=leaf.get_property<slic3r_api::BridgeVolumeInfo>()->nominal.mm3_per_mm);
                    ++pieces;
                });
        CHECK(pieces>=strands);
    }
}

TEST_CASE("A local support request never hides an uncovered missing anchor from its warning","[plugins][bridge-volume]")
{
    for (const bool disabled : {false,true}) {
        Fixture fixture({{"bridge_volume_compensation_enabled",disabled ? "0" : "1"}});
        fixture.prepare();fixture.add_paths(false);
        ApiInternal::LayerIslandAccess::support_demand_mutable(fixture.island())=ExPolygons{rectangle_expolygon(-2,-1,-1.9,1)};
        const ExtrusionEntity *root=nullptr;
        for (const LayerRegionIsland &region : fixture.island().regions_islands())
            if (region.has_extrusion(LayerRegionIsland::INFILLS)) root=&region.extrusion(LayerRegionIsland::INFILLS);
        REQUIRE(root!=nullptr);
        Active active({"bridge.finalize_volume","surface.bridge_preparation","infill.recipe.bridge_preparation","surface.solid_shells"});
        REQUIRE(finalize_skipped(fixture.prepared.print).size()==1);
        REQUIRE(root->child_count()==1);
        CHECK(root->child(0).is_leaf());
        CHECK(root->child(0).get_property<slic3r_api::BridgeVolumeInfo>()==nullptr);
    }
}

TEST_CASE("Unavailable supports retain nominal bridge pieces and keep stale stamps unchanged","[plugins][bridge-volume]")
{
    Fixture fixture;fixture.prepare();fixture.add_paths(false);
    PrintObject &object=fixture.prepared.print.object(0);
    object.get_or_add_property<slic3r_api::SupportPipelineInfo>().material_generated=0;
    Active active({"bridge.finalize_volume","surface.bridge_preparation","infill.recipe.bridge_preparation","surface.solid_shells"});
    for (int repeat=0;repeat<2;++repeat) {
        REQUIRE_NOTHROW(Steps::StepFinalizeSlicing::run_step(Orchestrator::instance(),fixture.prepared.print));
        for (const LayerRegionIsland &region : fixture.island().regions_islands()) if (region.has_extrusion(LayerRegionIsland::INFILLS))
            visit_leaves(region.extrusion(LayerRegionIsland::INFILLS),[](const ExtrusionEntity &leaf,const ExtrusionAttributes &attr) {
                const slic3r_api::BridgeVolumeInfo *stamp=leaf.get_property<slic3r_api::BridgeVolumeInfo>();REQUIRE(stamp!=nullptr);
                CHECK(stamp->confidence==BRIDGE_CONFIDENCE_UNKNOWN);
                CHECK(stamp->correction_state==BRIDGE_CORRECTION_KEPT_NOMINAL_UNKNOWN);
                CHECK(attr.mm3_per_mm==stamp->nominal.mm3_per_mm);
                CHECK(attr.height==stamp->nominal.height);
            });
    }
    // Generated support changes the inputs of these stamps: the rows keep them and their flow, with a warning.
    object.get_or_add_property<slic3r_api::SupportPipelineInfo>().material_generated=1;
    CHECK(finalize_skipped(fixture.prepared.print).size()==1);
    for (const LayerRegionIsland &region : fixture.island().regions_islands()) if (region.has_extrusion(LayerRegionIsland::INFILLS))
        visit_leaves(region.extrusion(LayerRegionIsland::INFILLS),[](const ExtrusionEntity &leaf,const ExtrusionAttributes &attr) {
            const slic3r_api::BridgeVolumeInfo *stamp=leaf.get_property<slic3r_api::BridgeVolumeInfo>();REQUIRE(stamp!=nullptr);
            CHECK(stamp->confidence==BRIDGE_CONFIDENCE_UNKNOWN);
            CHECK(attr.mm3_per_mm==stamp->nominal.mm3_per_mm);
        });
}

TEST_CASE("The finalization host refuses foreign duplicate and non-extrusion donors atomically","[plugins][bridge-volume][bridge-api]")
{
    Fixture fixture;fixture.prepare();fixture.add_paths();
    ProbeScope scope;
    scope.probe.action=[&](const run_ctx_finalize_slicing &ctx,const plugin_run_context &run) {
        LayerRegionIsland &lower=fixture.prepared.print.object(0).layer(0).island(0).regions_islands().front();
        LayerRegionIsland &upper=fixture.island().regions_islands().front();
        ExtrusionEntity &a=lower.mutable_extrusion(LayerRegionIsland::INFILLS);
        ExtrusionEntity &b=upper.mutable_extrusion(LayerRegionIsland::INFILLS);
        const uint64_t id=a.get_id();
        const double amount=a.child(0).get_property<ExtrusionAttributes>()->mm3_per_mm;
        slic3r_api::StoredExtrusionEntity donor(run.plugin_storage,slic3r_api::ExtrusionEntity(reinterpret_cast<const extrusion_entity_handle *>(&a)));
        donor.mutable_view().child_mutable(0).get_or_add(slic3r_api::EPropertyAttributes::key).mm3_per_mm(.01);
        raw_final_infill_replacement entries[]={{reinterpret_cast<const extrusion_entity_handle *>(&a),donor.mutable_handle()},
            {reinterpret_cast<const extrusion_entity_handle *>(&a),donor.mutable_handle()}};
        CHECK_FALSE(ctx.replace_infill_buckets(&ctx,run.plugin_storage,entries,2));
        slic3r_api::StoredPolyline wrong_type(run.plugin_storage);
        entries[1]={reinterpret_cast<const extrusion_entity_handle *>(&b),reinterpret_cast<extrusion_entity_handle *>(wrong_type.mutable_handle())};
        CHECK_FALSE(ctx.replace_infill_buckets(&ctx,run.plugin_storage,entries,2));
        PluginStorage foreign;
        CHECK_FALSE(ctx.replace_infill_buckets(&ctx,reinterpret_cast<storage_handle *>(&foreign),entries,1));
        entries[1]={reinterpret_cast<const extrusion_entity_handle *>(&lower.mutable_extrusion(LayerRegionIsland::PERIMETERS)),donor.mutable_handle()};
        CHECK_FALSE(ctx.replace_infill_buckets(&ctx,run.plugin_storage,entries,2));
        CHECK(a.child(0).get_property<ExtrusionAttributes>()->mm3_per_mm==amount);
        CHECK(ctx.replace_infill_buckets(&ctx,run.plugin_storage,entries,1));
        CHECK(a.get_id()==id);
        CHECK(a.child(0).get_property<ExtrusionAttributes>()->mm3_per_mm==.01);
        CHECK(donor.readonly().child(0).get(slic3r_api::EPropertyAttributes::key)->mm3_per_mm()==amount);
    };
    Active active({"test.bridge.final_probe"});
    REQUIRE_NOTHROW(Steps::StepFinalizeSlicing::run_step(Orchestrator::instance(),fixture.prepared.print));
}

TEST_CASE("Later finalization plugins cannot change or remove bridge certificates","[plugins][bridge-volume]")
{
    for (const bool surface_model : {false,true}) for (const bool remove_stamp : {false,true}) {
        Fixture fixture({{"bridge_volume_compensation",surface_model ? "bridge.finalize_surface_volume" : "bridge.finalize_volume"}});
        fixture.prepare();fixture.add_paths();
        ProbeScope scope;
        scope.probe.action=[&](const run_ctx_finalize_slicing &,const plugin_run_context &) {
            for (LayerRegionIsland &region : fixture.island().regions_islands())
                if (region.has_extrusion(LayerRegionIsland::INFILLS)) {
                    ExtrusionEntity *leaf=&region.mutable_extrusion(LayerRegionIsland::INFILLS);
                    while (leaf->child_count()>0) leaf=&leaf->child(0);
                    if (remove_stamp) leaf->remove_property<slic3r_api::BridgeVolumeInfo>();
                    else leaf->get_or_add_property<ExtrusionAttributes>().mm3_per_mm*=2.;
                }
        };
        Active active({surface_model ? "bridge.finalize_surface_volume" : "bridge.finalize_volume",
            "surface.bridge_preparation","infill.recipe.bridge_preparation","surface.solid_shells","test.bridge.final_probe"});
        REQUIRE_THROWS(Steps::StepFinalizeSlicing::run_step(Orchestrator::instance(),fixture.prepared.print));
    }
}

TEST_CASE("Prepared bridge metadata survives all downstream surface passes","[plugins][bridge-preparation]")
{
    Fixture fixture;fixture.prepare();
    const Surface expected=fixture.bridge();
    Active active({"surface.top_surface_expansion","surface.clean_infill_surfaces","surface.infill_region_compatibility_splitter",
        "dense_infill.surface_marker"});
    Steps::StepSurfaceGeneration::run_step(Orchestrator::instance(),fixture.prepared.print);
    const Surface &actual=fixture.bridge();
    CHECK(actual.bridge_angle==expected.bridge_angle);
    CHECK(actual.priority==expected.priority);
    CHECK(actual.scaled_thickness()==expected.scaled_thickness());
    CHECK(actual.thickness_layers==expected.thickness_layers);
    CHECK(actual.extra_perimeters==expected.extra_perimeters);
    CHECK(actual.maxNbSolidLayersOnTop==expected.maxNbSolidLayersOnTop);
    CHECK(actual.properties_equal(expected));
    same_area(view(ExPolygons{actual.expolygon}),ExPolygons{expected.expolygon});
}

TEST_CASE("A real auxiliary support layer supplies its own actual Z to bridge clearance","[plugins][bridge-volume]")
{
    Fixture fixture({{"min_layer_height","0.05"}});fixture.prepare();fixture.add_paths(false);
    PrintObject &object=fixture.prepared.print.object(0);
    const coord_t support_z=object.layer(1).scaled_print_z()-scale_i(.1);
    Layer &support=object.add_auxiliary_layer(0,scale_i(.1),support_z);
    support.get_or_add_property<LayerSupportProperty>().interface_id=0;
    ApiInternal::LayerAccess::add_region(support,object.printing_region(0));
    const ExPolygons areas{rectangle_expolygon(-4,-1,4,1)};
    ApiInternal::LayerRegionAccess::slices_mutable(support.region(0))=areas;
    ApiInternal::LayerAccess::set_islands(support,ExPolygons(areas));
    support.island(0).fill_regions(support);
    LayerRegionIsland &region=support.island(0).get_or_add_region_island(support.island(0).regions(),0);
    ExtrusionEntity path(false,ArcPolyline(Points{Point(scale_i(-4.),0),Point(scale_i(4.),0)}));
    path.add_property(ExtrusionAttributes(ExtrusionRole::SupportMaterialInterface,ExtrusionFlow(.04,.4f,.1f)));
    region.mutable_extrusion(LayerRegionIsland::SUPPORT_INTERFACE).append_child(std::move(path));
    ApiInternal::LayerIslandAccess::support_demand_mutable(fixture.island())=ExPolygons{fixture.bridge().expolygon};
    Active active({"bridge.finalize_volume","surface.bridge_preparation","infill.recipe.bridge_preparation","surface.solid_shells"});
    REQUIRE_NOTHROW(Steps::StepFinalizeSlicing::run_step(Orchestrator::instance(),fixture.prepared.print));
    for (const LayerRegionIsland &ri : fixture.island().regions_islands()) if (ri.has_extrusion(LayerRegionIsland::INFILLS))
        visit_leaves(ri.extrusion(LayerRegionIsland::INFILLS),[](const ExtrusionEntity &leaf,const ExtrusionAttributes &attr) {
            CHECK(attr.height==Approx(.1));
            CHECK(attr.mm3_per_mm==Approx(.03785398163397));
            REQUIRE(leaf.get_property<slic3r_api::BridgeVolumeInfo>()!=nullptr);
            CHECK(leaf.get_property<slic3r_api::BridgeVolumeInfo>()->confidence==BRIDGE_CONFIDENCE_CERTIFIED);
        });
}

TEST_CASE("Bridge preparation detects its direction and respects explicit zero and half turns","[plugins][bridge-preparation]")
{
    for (const char *angle : {"!0","0","180"}) {
        INFO(angle);
        Fixture fixture({{"bridge_angle",angle}});
        fixture.prepare();
        const Surface &bridge=fixture.bridge();
        CHECK(std::abs(std::remainder(bridge.bridge_angle,PI))<1e-5);
        REQUIRE(bridge.get_property<slic3r_api::BridgeSurfaceInfo>()!=nullptr);
        CHECK(bool(bridge.get_property<slic3r_api::BridgeSurfaceInfo>()->angle_from_user)==(std::string(angle)!="!0"));
        same_area(view(ExPolygons{bridge.expolygon}),ExPolygons{rectangle_expolygon(-3,-1,3,1)});
    }
}

// Without any anchor, no bridge direction can be detected: the external bridge
// follows the fill direction of its layer instead of stopping the slicing.
TEST_CASE("An external bridge without a detectable direction follows the fill direction of its layer","[plugins][bridge-preparation]")
{
    Fixture fixture({{"bridge_angle","!0"},{"fill_angle","30"},{"fill_angle_template",""},
        {"fill_angle_increment","0"},{"fill_angle_cross","0"}});
    // Keep a single lower support, far from the bridge core spanning x = -2..2.
    PrintObject &object=fixture.prepared.print.object(0);
    for (Layer &layer : object.layers()) for (LayerSliceIsland &island : layer.islands()) {
        island.overlaps_above.clear();island.overlaps_below.clear();
    }
    const ExPolygons lower{rectangle_expolygon(-8,-1,-6,1)};
    Layer &first=object.layer(0);
    ApiInternal::LayerRegionAccess::slices_mutable(first.region(0))=lower;
    ApiInternal::LayerAccess::set_islands(first,ExPolygons(lower));
    for (LayerSliceIsland &island : first.islands()) {
        island.fill_regions(first);ApiInternal::LayerIslandAccess::set_infill_areas(island,ExPolygons{island.get_slice()});
    }
    Layer::build_up_down_graph(object.layer(0),object.layer(1));
    fixture.prepare();
    const Surface &bridge=fixture.bridge();
    CHECK(std::abs(std::remainder(bridge.bridge_angle-30.*PI/180.,PI))<1e-5);
    REQUIRE(bridge.get_property<slic3r_api::BridgeSurfaceInfo>()!=nullptr);
    CHECK_FALSE(bool(bridge.get_property<slic3r_api::BridgeSurfaceInfo>()->angle_from_user));
}

// A second region covering the first one gives the bridge two physical owners, which
// preparation refuses: the island keeps its unprepared bridge surface and one warning is reported.
TEST_CASE("A bridge island that cannot be prepared keeps its surfaces and warns","[plugins][bridge-preparation]")
{
    Fixture fixture;
    Layer &layer=fixture.prepared.print.object(0).layer(1);
    ApiInternal::LayerAccess::add_region(layer,fixture.prepared.print.register_print_region(layer.region(0).region().config()));
    ApiInternal::LayerRegionAccess::slices_mutable(layer.region(1))=layer.region(0).get_raw_slices();
    LayerSliceIsland &island=fixture.island();
    island.fill_regions(layer);
    REQUIRE(island.regions().size()==2);
    // Move the seeded surfaces to a region island owned by both regions.
    LayerRegionIsland &seeded=island.regions_islands().front();
    const SurfaceCollection surfaces=seeded.fill_surfaces();
    seeded.set_fill_surfaces().clear();
    island.get_or_add_region_island(island.regions(),0).set_fill_surfaces()=surfaces;
    Orchestrator &orch=Orchestrator::instance();
    (void)orch.consume_plugin_messages();
    REQUIRE_NOTHROW(fixture.prepare());
    size_t warnings=0;
    for (const Orchestrator::PluginMessage &message : orch.consume_plugin_messages())
        if (message.message.find("bridge preparation skipped 1 island")!=std::string::npos) ++warnings;
    CHECK(warnings==1);
    bool bridge=false;
    for (const LayerRegionIsland &region : island.regions_islands())
        for (const Surface &surface : region.fill_surfaces()) {
            CHECK(surface.get_property<slic3r_api::BridgeSurfaceInfo>()==nullptr);
            bridge=bridge || surface.surface_type==(stPosBottom|stDensSolid|stModBridge);
        }
    CHECK(bridge);
}

TEST_CASE("Three connected bridge seeds are consumed once with stable grouping","[plugins][bridge-preparation]")
{
    uint64_t key=0;
    for (const bool reversed : {false,true}) {
        Fixture fixture;
        PrintObject &object=fixture.prepared.print.object(0);
        for (Layer &layer : object.layers()) for (LayerSliceIsland &island : layer.islands()) {
            island.overlaps_above.clear();island.overlaps_below.clear();
        }
        const ExPolygons lower{rectangle_expolygon(-8,-1,-2,1),rectangle_expolygon(-1,-1,-.25,1),
            rectangle_expolygon(.25,-1,1,1),rectangle_expolygon(2,-1,8,1)};
        ApiInternal::LayerRegionAccess::slices_mutable(object.layer(0).region(0))=lower;
        ApiInternal::LayerAccess::set_islands(object.layer(0),ExPolygons(lower));
        for (LayerSliceIsland &island : object.layer(0).islands()) island.fill_regions(object.layer(0));
        Layer::build_up_down_graph(object.layer(0),object.layer(1));
        ExPolygons cores{rectangle_expolygon(-2,-1,-1,1),rectangle_expolygon(-.25,-1,.25,1),rectangle_expolygon(1,-1,2,1)};
        if (reversed) std::reverse(cores.begin(),cores.end());
        SurfaceCollection &input=fixture.island().regions_islands().front().set_fill_surfaces();
        input.clear();input.append(cores,stPosBottom|stDensSolid|stModBridge);input.append(lower,stPosInternal|stDensSparse);
        fixture.prepare();
        const Surface &bridge=fixture.bridge();
        size_t count=0;
        for (const LayerRegionIsland &region : fixture.island().regions_islands())
            for (const Surface &surface : region.fill_surfaces())
                if (surface.get_property<slic3r_api::BridgeSurfaceInfo>()!=nullptr) ++count;
        CHECK(count==1);
        same_area(view(ExPolygons{bridge.expolygon}),ExPolygons{rectangle_expolygon(-3,-1,3,1)});
        same_area(view(diff_ex(ExPolygons{bridge.expolygon},lower)),cores);
        const uint64_t current=bridge.get_property<slic3r_api::BridgeSurfaceInfo>()->logical_key;
        if (reversed) CHECK(current==key);else key=current;
    }
}

TEST_CASE("Prepared bridge destinations keep distinct region flows through actual generation","[plugins][bridge-preparation][bridge-flow]")
{
    Fixture fixture;
    Layer &layer=fixture.prepared.print.object(0).layer(1);
    const SurfaceCollection original=fixture.island().regions_islands().front().fill_surfaces();
    PrintRegionConfig right_config=layer.region(0).region().config();
    right_config.set_deserialize_strict("bridge_flow_ratio","150%");
    ApiInternal::LayerAccess::add_region(layer,fixture.prepared.print.register_print_region(right_config));
    ApiInternal::LayerRegionAccess::slices_mutable(layer.region(0))=ExPolygons{rectangle_expolygon(-8,-1,0,1)};
    ApiInternal::LayerRegionAccess::slices_mutable(layer.region(1))=ExPolygons{rectangle_expolygon(0,-1,8,1)};
    fixture.island().mutable_regions_islands().clear();
    fixture.island().fill_regions(layer);
    LayerRegionIsland &input=fixture.island().get_or_add_region_island(fixture.island().regions(),0);
    input.set_fill_surfaces().append(original);
    fixture.prepare();
    {
        Active active({"infill.generator.default","infill.recipe.bridge_preparation","surface.bridge_preparation","surface.solid_shells","rectilinear"});
        Steps::StepGenerateInfill::run_step(Orchestrator::instance(),fixture.prepared.print);
    }
    size_t regions=0;
    for (const LayerRegionIsland &region : fixture.island().regions_islands()) {
        bool prepared=false;
        for (const Surface &surface : region.fill_surfaces()) prepared=prepared || surface.get_property<slic3r_api::BridgeSurfaceInfo>()!=nullptr;
        if (!prepared) continue;
        REQUIRE(region.regions().size()==1);
        CHECK(region.extruder_id()==0);
        const LayerRegion &owner=**region.regions().begin();
        const Flow expected=infill_bridge_flow(layer,owner,frSolidInfill);
        REQUIRE(region.has_extrusion(LayerRegionIsland::INFILLS));
        size_t leaves=0;
        visit_leaves(region.extrusion(LayerRegionIsland::INFILLS),[&](const ExtrusionEntity &,const ExtrusionAttributes &attr) {
            if (!attr.extrusion_role().is_bridge()) return;
            CHECK(attr.width==expected.width());CHECK(attr.height==expected.height());
            CHECK(attr.mm3_per_mm==expected.mm3_per_mm());
            ++leaves;
        });
        CHECK(leaves>0);++regions;
    }
    CHECK(regions==2);
}

// Check fixed ids and host layouts so instance ownership cannot alias bridge metadata.
TEST_CASE("Bridge and instance metadata keep distinct registered layouts","[plugins][bridge-preparation][bridge-api]")
{
    CHECK(SLIC3R_PROPERTY_TYPE_LAYER_INSTANCE==16u);
    struct Layout {
        slic3r_property_type type;
        const char *name;
        uint32_t size;
        uint32_t alignment;
    };
    const Layout layouts[] = {
        {SLIC3R_PROPERTY_TYPE_LAYER_INSTANCE,"slic3r.layer.instance",sizeof(c_layer_instance_property),alignof(c_layer_instance_property)},
        {SLIC3R_PROPERTY_TYPE_BRIDGE_SURFACE,"bridge_preparation.surface.v1",sizeof(c_bridge_surface_property),alignof(c_bridge_surface_property)},
        {SLIC3R_PROPERTY_TYPE_BRIDGE_VOLUME,"bridge_preparation.volume.v1",sizeof(c_bridge_volume_property),alignof(c_bridge_volume_property)},
        {SLIC3R_PROPERTY_TYPE_BRIDGE_STRAND,"bridge_preparation.atomic_strand.v1",sizeof(c_bridge_strand_property),alignof(c_bridge_strand_property)},
        {SLIC3R_PROPERTY_TYPE_SUPPORT_PIPELINE,"support.pipeline_state.v1",sizeof(c_support_pipeline_property),alignof(c_support_pipeline_property)}
    };
    std::vector<slic3r_property_type> ids;
    for (const Layout &layout : layouts) {
        INFO(layout.name);
        CHECK(std::find(ids.begin(),ids.end(),layout.type)==ids.end());
        ids.push_back(layout.type);
        const char *name=orchestrator_property_name(nullptr,layout.type);
        REQUIRE(name!=nullptr);
        CHECK(std::string(name)==layout.name);
        CHECK(orchestrator_property_byte_count(nullptr,layout.type)==layout.size);
        CHECK(orchestrator_property_alignment(nullptr,layout.type)==layout.alignment);
    }
}

TEST_CASE("Bridge metadata compatibility checks every semantic field","[plugins][bridge-preparation][bridge-api]")
{
    CHECK(extrusion_property_byte_count(nullptr,SLIC3R_PROPERTY_TYPE_BRIDGE_VOLUME)==sizeof(c_bridge_volume_property));
    CHECK(extrusion_property_byte_count(nullptr,SLIC3R_PROPERTY_TYPE_BRIDGE_STRAND)==sizeof(c_bridge_strand_property));
    PluginStorage storage;
    slic3r_api::StoredExtrusionEntity scratch(reinterpret_cast<storage_handle *>(&storage));
    scratch.mutable_view().get_or_add(slic3r_api::BridgeVolumeInfo::key).logical_key=42;
    CHECK(scratch.readonly().get(slic3r_api::BridgeVolumeInfo::key)->logical_key==42);
    SurfaceCollection source;
    source.append(ExPolygons{rectangle_expolygon(0,0,1,1)},stPosBottom|stDensSolid|stModBridge);
    const Surface base=source.surfaces.front();
    const auto compatible=[&](const Surface &other) {
        return surface_metadata_compatible_for_bridge(reinterpret_cast<const surface_handle *>(&base),
            reinterpret_cast<const surface_handle *>(&other))!=0;
    };
    REQUIRE(compatible(base));
    for (int field=0;field<7;++field) {
        Surface other=base;
        if (field==0) other.set_scaled_thickness(scale_i(.2));
        if (field==1) ++other.thickness_layers;
        if (field==2) ++other.extra_perimeters;
        if (field==3) ++other.maxNbSolidLayersOnTop;
        if (field==4) ++other.priority;
        if (field==5) other.get_or_add_property<slic3r_api::BridgeSurfaceInfo>().logical_key=42;
        if (field==6) other.surface_type=stPosInternal|stDensSolid|stModBridge;
        INFO("metadata field "<<field);
        CHECK_FALSE(compatible(other));
    }
    Surface zero=base,half_turn=base,distinct=base;
    zero.bridge_angle=0.;half_turn.bridge_angle=PI;distinct.bridge_angle=PI/180.;
    CHECK(surface_metadata_compatible_for_bridge(reinterpret_cast<const surface_handle *>(&zero),reinterpret_cast<const surface_handle *>(&half_turn)));
    CHECK_FALSE(surface_metadata_compatible_for_bridge(reinterpret_cast<const surface_handle *>(&zero),reinterpret_cast<const surface_handle *>(&distinct)));
}

TEST_CASE("Two islands sharing one region keep separate lower slices directions and destinations","[plugins][bridge-preparation]")
{
    Fixture fixture({{"bridge_angle","!0"}});
    PrintObject &object=fixture.prepared.print.object(0);
    for (Layer &layer : object.layers()) for (LayerSliceIsland &island : layer.islands()) {
        island.overlaps_above.clear();island.overlaps_below.clear();
    }
    const ExPolygons first_lower{rectangle_expolygon(-8,-1,-2,1),rectangle_expolygon(2,-1,8,1)};
    const ExPolygon first_upper=rectangle_expolygon(-8,-1,8,1);
    const ExPolygon first_core=rectangle_expolygon(-2,-1,2,1);
    ExPolygons second_lower=first_lower;
    ExPolygon second_upper=first_upper,second_core=first_core;
    for (ExPolygon *area : {&second_upper,&second_core}) { area->rotate(PI/2.);area->translate(0,scale_i(24.)); }
    for (ExPolygon &area : second_lower) { area.rotate(PI/2.);area.translate(0,scale_i(24.)); }
    ExPolygons lower=first_lower;append(lower,second_lower);
    const ExPolygons upper{first_upper,second_upper};
    for (size_t idx=0;idx<2;++idx) {
        Layer &layer=object.layer(idx);
        const ExPolygons areas=idx==0 ? lower : upper;
        ApiInternal::LayerRegionAccess::slices_mutable(layer.region(0))=areas;
        ApiInternal::LayerAccess::set_islands(layer,ExPolygons(areas));
        for (LayerSliceIsland &island : layer.islands()) {
            island.fill_regions(layer);ApiInternal::LayerIslandAccess::set_infill_areas(island,ExPolygons{island.get_slice()});
        }
    }
    Layer::build_up_down_graph(object.layer(0),object.layer(1));
    for (size_t idx=0;idx<2;++idx) {
        LayerSliceIsland &island=object.layer(1).island(idx);
        SurfaceCollection &surfaces=island.get_or_add_region_island(island.regions(),0).set_fill_surfaces();
        surfaces.append(ExPolygons{idx==0 ? first_core : second_core},stPosBottom|stDensSolid|stModBridge);
        surfaces.append(idx==0 ? first_lower : second_lower,stPosInternal|stDensSparse);
    }
    fixture.prepare();
    std::vector<uint64_t> keys;
    for (size_t idx=0;idx<2;++idx) {
        size_t count=0;
        for (const LayerRegionIsland &region : object.layer(1).island(idx).regions_islands())
            for (const Surface &surface : region.fill_surfaces())
                if (const slic3r_api::BridgeSurfaceInfo *info=surface.get_property<slic3r_api::BridgeSurfaceInfo>()) {
                    CHECK(region.regions().size()==1);CHECK(region.extruder_id()==0);
                    CHECK(std::abs(std::remainder(surface.bridge_angle-(idx==0 ? 0. : PI/2.),PI))<1e-5);
                    keys.push_back(info->logical_key);++count;
                }
        CHECK(count==1);
    }
    REQUIRE(keys.size()==2);CHECK(keys[0]!=keys[1]);
}

TEST_CASE("Support completeness separates unavailable producers from computed empty results","[plugins][bridge-volume][bridge-support]")
{
    Fixture fixture({{"support_material","1"},{"support_material_style","grid"}});
    Orchestrator &orch=Orchestrator::instance();
    Print &print=fixture.prepared.print;
    PrintObject &object=print.object(0);
    {
        Active active({});
        Steps::StepSupportDemand::clear(print);
        Steps::StepSupportDemand::run_step(orch,print);
        const slic3r_api::SupportPipelineInfo *state=object.get_property<slic3r_api::SupportPipelineInfo>();
        REQUIRE(state!=nullptr);CHECK(state->demand_computed==1);
        Steps::StepGenerateSupport::run_step(orch,print);
        Steps::StepSupportToolpath::run_step(orch,print);
        CHECK(object.get_property<slic3r_api::SupportPipelineInfo>()->material_generated==0);
    }
    {
        Active active({"support.family.tree"});
        Steps::StepGenerateSupport::run_step(orch,print);
        Steps::StepSupportToolpath::run_step(orch,print);
        CHECK(object.get_property<slic3r_api::SupportPipelineInfo>()->surfaces_generated==0);
        CHECK(object.get_property<slic3r_api::SupportPipelineInfo>()->material_generated==0);
    }
    {
        Active active({"support.family.grid"});
        Steps::StepGenerateSupport::run_step(orch,print);
        Steps::StepSupportToolpath::run_step(orch,print);
        CHECK(object.get_property<slic3r_api::SupportPipelineInfo>()->surfaces_generated==1);
        CHECK(object.get_property<slic3r_api::SupportPipelineInfo>()->material_generated==1);
    }
    Steps::StepSupportToolpath::clear(print);
    CHECK(object.get_property<slic3r_api::SupportPipelineInfo>()->material_generated==0);
    Steps::StepSupportDemand::clear(print);
    CHECK(object.get_property<slic3r_api::SupportPipelineInfo>()==nullptr);
}

TEST_CASE("Actual gap-fill buckets are occupied material below a bridge","[plugins][bridge-volume]")
{
    Fixture fixture;fixture.prepare();fixture.add_paths(false);
    LayerSliceIsland &lower=fixture.prepared.print.object(0).layer(0).island(0);
    LayerRegionIsland &owner=lower.get_or_add_region_island(lower.regions(),0);
    ExtrusionEntity gap(false,ArcPolyline(Points{Point(scale_i(-4.),0),Point(scale_i(-2.8),0)}));
    gap.add_property(ExtrusionAttributes(ExtrusionRole::GapFill,ExtrusionFlow(.08,.4f,.2f)));
    owner.mutable_extrusion(LayerRegionIsland::GAP_FILLS).append_child(std::move(gap));
    ApiInternal::LayerIslandAccess::support_demand_mutable(fixture.island())=ExPolygons{fixture.bridge().expolygon};
    Active active({"bridge.finalize_volume","surface.bridge_preparation","infill.recipe.bridge_preparation","surface.solid_shells"});
    REQUIRE_NOTHROW(Steps::StepFinalizeSlicing::run_step(Orchestrator::instance(),fixture.prepared.print));
    bool gap_corrected=false,core_nominal=false;
    for (const LayerRegionIsland &region : fixture.island().regions_islands()) if (region.has_extrusion(LayerRegionIsland::INFILLS))
        visit_leaves(region.extrusion(LayerRegionIsland::INFILLS),[&](const ExtrusionEntity &leaf,const ExtrusionAttributes &attr) {
            const coord_t left=std::min(leaf.first_point().x(),leaf.last_point().x());
            const coord_t right=std::max(leaf.first_point().x(),leaf.last_point().x());
            if (left<scale_i(-2.9) && right>scale_i(-2.9)) { CHECK(attr.height==Approx(.2));gap_corrected=true; }
            if (left<0 && right>0) { CHECK(attr.height==Approx(.4));core_nominal=true; }
        });
    CHECK(gap_corrected);CHECK(core_nominal);
}

TEST_CASE("Forced layers and explicit local demand require a real support producer","[plugins][bridge-volume][bridge-support]")
{
    for (int kind=0;kind<3;++kind) {
        Fixture fixture({{"support_material","0"},{"support_material_enforce_layers",kind==0 ? "2" : "0"}});
        Print &print=fixture.prepared.print;
        Active active({});
        Steps::StepSupportDemand::clear(print);
        Steps::StepSupportDemand::run_step(Orchestrator::instance(),print);
        if (kind==1)
            ApiInternal::LayerIslandAccess::support_demand_mutable(fixture.island())=ExPolygons{rectangle_expolygon(-2,-1,2,1)};
        Steps::StepSupportToolpath::run_step(Orchestrator::instance(),print);
        const slic3r_api::SupportPipelineInfo *state=print.object(0).get_property<slic3r_api::SupportPipelineInfo>();
        REQUIRE(state!=nullptr);
        CHECK(state->material_generated==uint32_t(kind==2));
    }
}

TEST_CASE("A post-infill producer bypassing the prepared bridge role contract stays uncorrected","[plugins][bridge-volume]")
{
    Fixture fixture;fixture.prepare();fixture.add_paths();
    const uint64_t source_id=fixture.bridge().id();
    LayerRegionIsland &owner=fixture.island().regions_islands().front();
    ExtrusionEntity gap(false,ArcPolyline(Points{Point(scale_i(-1.),0),Point(scale_i(1.),0)}));
    gap.add_property(ExtrusionAttributes(ExtrusionRole::GapFill,ExtrusionFlow(.08,.4f,.2f)));
    gap.get_or_add_property<ExtrusionPropertyInfill>().source_surface_id=source_id;
    owner.mutable_extrusion(LayerRegionIsland::GAP_FILLS).append_child(std::move(gap));
    Active active({"bridge.finalize_volume","surface.bridge_preparation","infill.recipe.bridge_preparation","surface.solid_shells"});
    // Only the gap-fill group is refused: it keeps its flow, while the bridge rows are still corrected.
    CHECK(finalize_skipped(fixture.prepared.print).size()==1);
    visit_leaves(owner.extrusion(LayerRegionIsland::GAP_FILLS),[](const ExtrusionEntity &leaf,const ExtrusionAttributes &attr) {
        CHECK(leaf.get_property<slic3r_api::BridgeVolumeInfo>()==nullptr);
        CHECK(attr.mm3_per_mm==Approx(.08));
    });
    for (const LayerRegionIsland &region : fixture.island().regions_islands()) if (region.has_extrusion(LayerRegionIsland::INFILLS))
        visit_leaves(region.extrusion(LayerRegionIsland::INFILLS),[](const ExtrusionEntity &leaf,const ExtrusionAttributes &) {
            CHECK(leaf.get_property<slic3r_api::BridgeVolumeInfo>()!=nullptr);
        });
}

TEST_CASE("Cancellation is observed before publishing a final infill batch","[plugins][bridge-volume][bridge-api]")
{
    Fixture fixture;fixture.prepare();fixture.add_paths();
    ProbeScope scope;
    bool checked=false;
    scope.probe.action=[&](const run_ctx_finalize_slicing &ctx,const plugin_run_context &run) {
        ExtrusionEntity &root=fixture.island().regions_islands().front().mutable_extrusion(LayerRegionIsland::INFILLS);
        const double nominal=root.child(0).get_property<ExtrusionAttributes>()->mm3_per_mm;
        slic3r_api::StoredExtrusionEntity donor(run.plugin_storage,slic3r_api::ExtrusionEntity(reinterpret_cast<const extrusion_entity_handle *>(&root)));
        donor.mutable_view().child_mutable(0).get_or_add(slic3r_api::EPropertyAttributes::key).mm3_per_mm(.01);
        const raw_final_infill_replacement replacement{reinterpret_cast<const extrusion_entity_handle *>(&root),donor.mutable_handle()};
        Orchestrator::instance().request_plugin_cancel();
        CHECK_FALSE(ctx.replace_infill_buckets(&ctx,run.plugin_storage,&replacement,1));
        CHECK(root.child(0).get_property<ExtrusionAttributes>()->mm3_per_mm==nominal);
        checked=true;
    };
    Active active({"test.bridge.final_probe"});
    REQUIRE_THROWS(Steps::StepFinalizeSlicing::run_step(Orchestrator::instance(),fixture.prepared.print));
    CHECK(checked);
}

TEST_CASE("Clearance queries remain cancellable inside an occupied region","[plugins][bridge-volume]")
{
    int cancelled=0;
    plugin_run_context context={};
    context.host_context=reinterpret_cast<plugin_host_context *>(&cancelled);
    context.is_cancelled=[](plugin_host_context *host) { return *reinterpret_cast<int *>(host); };
    Clearance::Index index(&context);
    index.add({{0,0},{3,0},.2,.5,.8,false});
    cancelled=1;
    REQUIRE_THROWS_AS(index.clearance({0,0},{3,0},.4,.4,1.),slic3r_api::PluginCancelled);
}

TEST_CASE("Final bridge recipes reject late changes to pattern flow angle role or tool","[plugins][bridge-preparation]")
{
    Fixture fixture;fixture.prepare();
    const Surface &surface=fixture.bridge();
    const LayerRegion &region=fixture.prepared.print.object(0).layer(1).region(0);
    const layer_region_handle *owner=reinterpret_cast<const layer_region_handle *>(&region);
    const surface_handle *source=reinterpret_cast<const surface_handle *>(&surface);
    const c_flow_mm flow=layer_region_get_infill_bridge_flow_mm(owner,RAW_EXTRUSION_ROLE_SOLID_INFILL);
    raw_infill_pattern_params nominal={};
    nominal.surface_type=static_cast<raw_surface_type>(surface.surface_type);
    nominal.extrusion_role=RAW_EXTRUSION_ROLE_BRIDGE_INFILL;
    nominal.flow=layer_region_get_infill_bridge_flow(owner,RAW_EXTRUSION_ROLE_SOLID_INFILL);
    nominal.spacing=flow.spacing;nominal.density=1.f;nominal.flow_mult=1.f;
    nominal.extruder=1;nominal.bridge_angle=surface.bridge_angle;
    nominal.connection=C_INFILL_CONNECTION_NOT_CONNECTED;
    REQUIRE_NOTHROW(slic3r_api::BridgePreparation::validate_recipe(source,nominal,"rectilinear",owner));
    REQUIRE_NOTHROW(slic3r_api::BridgePreparation::validate_recipe(source,nominal,"monotonic",owner));
    CHECK_THROWS(slic3r_api::BridgePreparation::validate_recipe(source,nominal,"grid",owner));
    for (int field=0;field<10;++field) {
        raw_infill_pattern_params modified=nominal;
        if (field==0) modified.bridge_angle+=.1;
        if (field==1) modified.flow_mult=1.1f;
        if (field==2) modified.extrusion_role=RAW_EXTRUSION_ROLE_GAP_FILL;
        if (field==3) modified.extruder=2;
        if (field==4) modified.anchor_length=1.;
        if (field==5) modified.infill_flags|=C_INFILL_FLAG_ADD_GAP_FILL;
        if (field==6) modified.spacing+=.01;
        if (field==7) modified.flow.mm3_per_mm*=1.1;
        if (field==8) modified.flow.height+=SCALED_LAYER_Z_STEP;
        if (field==9) modified.flow.spacing_ratio+=.1f;
        INFO("recipe field "<<field);
        CHECK_THROWS(slic3r_api::BridgePreparation::validate_recipe(source,modified,"rectilinear",owner));
    }
}
