///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
/* Check bridge preparation on small shapes with independently known geometry.
   Fixture::prepare() runs the surface pass on exact linked-layer inputs;
   view(), axial_expand() and same_area() check bounds and partitions.
   Recipe cases generate real infill and reject incompatible late edits.

   Representative direct calls from separate cases:

       TEST_CASE [axial geometry]
           |-- view() [borrow the native polygon vector]
           |-- BridgeGeometry::axial_expand()
           `-- same_area() [explicit expected polygons, not another builder]

       TEST_CASE [surface preparation]
           `-- Fixture::prepare()
               `-- StepSurfaceGeneration::run_step() [dispatch selected providers]

   Internal-cover cases also check the first eligible solid/top cover, its lower
   sparse angle fallback, regional ownership and preservation by later surface
   passes. These cases do not require a volume finalizer.

   Fixtures own model, print and native geometry. API views borrow those
   objects; provider selection and cancellation are restored by scope.
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
};
// Invoke callback on borrowed printable leaves with resolved inherited attributes.
// Missing attributes fail the test; the tree and attribute ownership stay unchanged.
void visit_leaves(const ExtrusionEntity &root,const std::function<void(const ExtrusionEntity &,const ExtrusionAttributes &)> &callback,
    const ExtrusionAttributes *inherited=nullptr);

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
