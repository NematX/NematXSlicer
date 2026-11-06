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

   The same probe mechanism also tests cancellation and replay protection. Each action is synchronous and borrows its payload.
   Fixture owns its model, print and native geometry; API views remain borrowed
   while those owners live. Native XY uses scaled integers; clearance lengths
   use mm, sections mm2 and volumes mm3. Injected support states test contracts,
   not production support generation; real-provider slicing/export integration
   is in test_bridge_pipeline.cpp. Active providers and callbacks are restored
   by their test scopes.
*/
#include <catch2/catch.hpp>
#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>
#include "perimeter_test_helpers.hpp"
#include "plugin_test_helpers.hpp"
#include "libslic3r/Api/host/InfillBridgeFlow.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/internal/LayerAccess.hpp"
#include "libslic3r/Api/internal/LayerIslandAccess.hpp"
#include "libslic3r/Api/internal/LayerRegionAccess.hpp"
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
#include "libslic3r/Plugins/Surface/BridgeGeometry.hpp"
#include "libslic3r/Plugins/Surface/BridgePreparation.hpp"
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
        {"perimeters","0"},{"bridged_infill_margin","1"},{"bridged_infill_uniform_margin","0"},{"bridge_angle","0"}});
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
void visit_leaves(const ExtrusionEntity &root,const std::function<void(const ExtrusionEntity &,const ExtrusionAttributes &)> &callback,
    const ExtrusionAttributes *inherited)
{
    const ExtrusionAttributes *attributes=root.get_property<ExtrusionAttributes>();
    if (attributes==nullptr) attributes=inherited;
    if (root.is_leaf() && root.has_polyline()) { REQUIRE(attributes!=nullptr);callback(root,*attributes); }
    for (const ExtrusionEntityUPtr &child : root.children()) if (child) visit_leaves(*child,callback,attributes);
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
            CHECK_THROWS(Steps::StepFinalizeSlicing::run_step(Orchestrator::instance(),fixture.prepared.print));
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

TEST_CASE("A peripheral bridge row must stay in its surface and cross the uniform source area","[plugins][bridge-volume][bridge-uniform]")
{
    for (int scenario=0;scenario<3;++scenario) {
        Fixture fixture({{"bridged_infill_uniform_margin",scenario==2 ? "0" : "0.6"}});
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
        Active active({"bridge.finalize_volume","surface.bridge_preparation","infill.recipe.bridge_preparation","surface.solid_shells"});
        CHECK_THROWS(Steps::StepFinalizeSlicing::run_step(Orchestrator::instance(),fixture.prepared.print));
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

TEST_CASE("A local support request never hides an uncovered missing anchor","[plugins][bridge-volume]")
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
        REQUIRE_THROWS(Steps::StepFinalizeSlicing::run_step(Orchestrator::instance(),fixture.prepared.print));
        REQUIRE(root->child_count()==1);
        CHECK(root->child(0).is_leaf());
        CHECK(root->child(0).get_property<slic3r_api::BridgeVolumeInfo>()==nullptr);
    }
}

TEST_CASE("Unavailable supports retain nominal bridge pieces and invalidate stale stamps","[plugins][bridge-volume]")
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
    object.get_or_add_property<slic3r_api::SupportPipelineInfo>().material_generated=1;
    REQUIRE_THROWS(Steps::StepFinalizeSlicing::run_step(Orchestrator::instance(),fixture.prepared.print));
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
    for (const bool remove_stamp : {false,true}) {
        Fixture fixture;
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
        Active active({"bridge.finalize_volume",
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

TEST_CASE("A post-infill producer cannot bypass the prepared bridge role contract","[plugins][bridge-volume]")
{
    Fixture fixture;fixture.prepare();fixture.add_paths();
    const uint64_t source_id=fixture.bridge().id();
    LayerRegionIsland &owner=fixture.island().regions_islands().front();
    ExtrusionEntity gap(false,ArcPolyline(Points{Point(scale_i(-1.),0),Point(scale_i(1.),0)}));
    gap.add_property(ExtrusionAttributes(ExtrusionRole::GapFill,ExtrusionFlow(.08,.4f,.2f)));
    gap.get_or_add_property<ExtrusionPropertyInfill>().source_surface_id=source_id;
    owner.mutable_extrusion(LayerRegionIsland::GAP_FILLS).append_child(std::move(gap));
    Active active({"bridge.finalize_volume","surface.bridge_preparation","infill.recipe.bridge_preparation","surface.solid_shells"});
    REQUIRE_THROWS(Steps::StepFinalizeSlicing::run_step(Orchestrator::instance(),fixture.prepared.print));
    for (const LayerRegionIsland &region : fixture.island().regions_islands()) if (region.has_extrusion(LayerRegionIsland::INFILLS))
        visit_leaves(region.extrusion(LayerRegionIsland::INFILLS),[](const ExtrusionEntity &leaf,const ExtrusionAttributes &) {
            CHECK(leaf.get_property<slic3r_api::BridgeVolumeInfo>()==nullptr);
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
