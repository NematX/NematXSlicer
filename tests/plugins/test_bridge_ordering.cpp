///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
/* Check that bridges use the right nozzle and that ordering never disconnects
   the pieces of one bridge path.

   Flow cases compare native/C-adapter results with an independently selected
   nozzle, height or solid-infill area and the circular-section formula. Ordering
   cases use a fixed prefix ending at x=4 and a 0->1->10 strand: considering either
   child alone can violate the required sequence. Expected flow is tied to the
   piece's X position, not read back from the ordering algorithm.

   Representative direct calls from separate cases:

       TEST_CASE [flow]
           |-- prepare_cube_print()
           |-- infill_bridge_flow()
           `-- layer_region_get_infill_bridge_flow_mm(), layer_region_get_infill_bridge_flow()

       TEST_CASE [ordering]
           |-- path() [build the source bucket and matching plan clone]
           `-- OrchestratorView::execute() [dispatch the selected real orderer]

   Continuity, orientation and position-specific flow are checked after execution;
   those assertions are not calls made by the orderer itself.

   Fixtures own the borrowed regions until direct ordering returns. Activation
   is restored by scope; all geometry and plan changes belong to these tests.
*/
#include <catch2/catch.hpp>
#include <cmath>
#include <memory>
#include <vector>
#include "perimeter_test_helpers.hpp"
#include "plugin_test_helpers.hpp"
#include "libslic3r/Api/host/InfillBridgeFlow.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_ordering.h"
#include "libslic3r/Api/plugin/cpp/OrchestratorViews.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/properties/BridgeProperties.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"
#include "libslic3r/Plugins/PrintingPlan/EntryPointProperty.h"

namespace {
using namespace Slic3r;
using namespace Slic3r::Test::PerimeterPluginTests;
// Return an owned straight X-axis leaf: begin/end are mm, amount is mm3/mm,
// and reversible controls the permission later ordering must preserve.
ExtrusionEntityUPtr path(double begin,double end,double amount,bool reversible);
// Restore a provider's original activation after direct execution.
struct Activation {
    Plugin *plugin;
    bool active;
    // Enable one provider and remember its activation flag for this scope.
    explicit Activation(const char *id) : plugin(Orchestrator::instance().get_plugin(id)),active(Orchestrator::instance().is_plugin_active(plugin)) {
        REQUIRE(plugin!=nullptr);REQUIRE(Orchestrator::instance().set_plugin_active(plugin,true));
    }
    // Restore that flag and clear cancellation before another test uses the host.
    ~Activation() { Orchestrator::instance().set_plugin_active(plugin,active);Orchestrator::instance().reset_plugin_cancel(); }
};
ExtrusionEntityUPtr path(double begin,double end,double amount,bool reversible)
{
    ExtrusionEntityUPtr result=std::make_unique<ExtrusionEntity>(reversible,
        ArcPolyline(Points{Point(scale_i(begin),0),Point(scale_i(end),0)}));
    result->add_property(ExtrusionAttributes(ExtrusionRole::BridgeInfill,ExtrusionFlow(amount,.4f,.2f)));
    return result;
}
}

TEST_CASE("All bridge modes preserve the generator's role-specific nozzle","[plugins][bridge-preparation][bridge-flow]")
{
    using namespace Slic3r;
    using namespace Slic3r::Test::PerimeterPluginTests;
    // Unequal tools distinguish role-specific nozzle selection from the shared
    // solid-infill area reference used only by the flow-based bridge mode.
    for (const char *mode : {"nozzle","height","flow"})
        for (const char *ratio : {"50%","100%","150%"}) {
            INFO("mode "<<mode<<" ratio "<<ratio);
            PreparedPerimeterPrint fixture;
            prepare_cube_print(fixture,perimeter_config({{"nozzle_diameter","0.4,0.6"},
                {"min_layer_height","5%,5%"},{"max_layer_height","75%,75%"},
                {"solid_infill_extruder","1"},{"infill_extruder","2"},{"bridge_type",mode},{"bridge_flow_ratio",ratio}}));
            const PrintObject &object=fixture.print.object(0);
            const Layer &layer=object.layer(1);const LayerRegion &region=layer.region(0);
            for (const FlowRole role : {frSolidInfill,frInfill}) {
                const Flow actual=infill_bridge_flow(layer,region,role);
                const float nozzle=role==frSolidInfill ? .4f : .6f;
                const double base=std::string(mode)=="height" ? layer.unscaled_height() :
                    std::string(mode)=="flow" ? std::sqrt(4.*region.flow(frSolidInfill).mm3_per_mm()/PI) : nozzle;
                const float diameter=float(base*std::sqrt(region.region().config().bridge_flow_ratio.get_effective_value(1.)));
                CHECK(actual.nozzle_diameter()==nozzle);CHECK(actual.width()==diameter);CHECK(actual.height()==diameter);
                CHECK(actual.mm3_per_mm()==Approx(PI*diameter*diameter/4.));
                const raw_extrusion_role raw=role==frSolidInfill ? RAW_EXTRUSION_ROLE_SOLID_INFILL : RAW_EXTRUSION_ROLE_INTERNAL_INFILL;
                const layer_region_handle *handle=reinterpret_cast<const layer_region_handle *>(&region);
                const c_flow_mm adapter=layer_region_get_infill_bridge_flow_mm(handle,raw);
                CHECK(adapter.width==double(actual.width()));CHECK(adapter.spacing==double(actual.spacing()));
                const c_flow transport=layer_region_get_infill_bridge_flow(handle,raw);
                CHECK(transport.width==actual.scaled_width());CHECK(transport.height==actual.scaled_height());
                CHECK(transport.spacing==actual.scaled_spacing());CHECK(transport.mm3_per_mm==actual.mm3_per_mm());
                CHECK(transport.spacing_ratio==actual.spacing_ratio());
                CHECK(layer_region_get_role_extruder(handle,raw)==(role==frSolidInfill ? 0 : 1));
            }
        }
}

TEST_CASE("Every built-in ordering keeps split bridges continuous and correctly associated with flow","[plugins][bridge-volume][bridge-ordering]")
{
    using namespace Slic3r;
    using namespace Slic3r::Test::PerimeterPluginTests;
    for (const char *provider : {"ordering.extrusion_tree.advanced","ordering.extrusion_tree.xy_serpentine","ordering.extrusion_tree.default"})
        for (const bool reverse_source : {false,true})
            for (const bool reversible : {false,true}) {
                INFO(provider<<" reversed source "<<reverse_source<<" reversible "<<reversible);
                PreparedPerimeterPrint fixture;prepare_cube_print(fixture,perimeter_config({}));
                Activation activation(provider);
                const LayerRegion &region=fixture.print.object(0).layer(0).region(0);
                LayerRegionIsland source({&region},0);
                ExtrusionEntityUPtr strand=std::make_unique<ExtrusionEntity>(reversible);
                strand->append_child(path(0.,1.,.05,reversible));
                strand->append_child(path(1.,10.,.12,reversible));
                strand->set_can_sort_reverse(false,reversible);
                strand->get_or_add_property<slic3r_api::BridgeAtomicStrand>().atomic_orientation=1;
                if (reverse_source) {
                    // Build the inverse orientation even for a monotonic source.
                    strand->set_can_sort_reverse(false,true);strand->child(0).set_can_sort_reverse(false,true);strand->child(1).set_can_sort_reverse(false,true);
                    strand->reverse();
                    strand->set_can_sort_reverse(false,reversible);strand->child(0).set_can_sort_reverse(false,reversible);strand->child(1).set_can_sort_reverse(false,reversible);
                }
                ExtrusionEntityUPtr root=std::make_unique<ExtrusionEntity>(false);
                root->append_child(path(-10.,4.,.1,false));root->append_child(std::move(strand));
                root->set_can_sort_reverse(false,false);
                // Orderers use source context as well as printable clones; keep
                // both trees consistent while fixing the prefix's position.
                source.mutable_extrusion(LayerRegionIsland::INFILLS).append_child(*root);
                Printing::PrintingPlan &plan=fixture.print.mutable_printing_plan();
                plan.groups.emplace_back();plan.groups.back().layers.emplace_back();plan.groups.back().layers.back().tool_groups.emplace_back();
                Printing::PrintingToolGroup &tool=plan.groups.back().layers.back().tool_groups.back();tool.extruder_id=0;
                Printing::PrintingExtrusion extrusion;extrusion.region_island=&source;extrusion.sregion_island_role=LayerRegionIsland::INFILLS;
                extrusion.root=std::move(root);
                slic3r_api::MutableExtrusionEntity output(reinterpret_cast<extrusion_entity_handle *>(extrusion.root.get()));
                slic3r_api::EntryPointProperty &entry=slic3r_api::entry_point_property_key(
                    reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance())).get_or_add(output);
                entry.entry=output.front();entry.exit=output.back();
                tool.region_islands.push_back(&source);tool.extrusions.push_back(std::move(extrusion));
                run_ctx_extrusion_ordering context={};context.print=reinterpret_cast<const print_handle *>(&fixture.print);
                context.plan=reinterpret_cast<printing_plan_handle *>(&plan);
                slic3r_api::OrchestratorView orch(reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
                REQUIRE(orch.execute(orch.find_plugin(provider),context.print,context)==RAW_PLUGIN_EXECUTION_SUCCESS);
                const ExtrusionEntity &result=tool.extrusions.front().root->child(1);
                REQUIRE(result.child_count()==2);
                CHECK(result.child(0).last_point()==result.child(1).first_point());
                CHECK(result.length()==Approx(scale_d(10.)));
                for (const ExtrusionEntityUPtr &piece : result.children()) {
                    const double midpoint=unscaled((piece->first_point().x()+piece->last_point().x())/2);
                    const ExtrusionAttributes *attr=piece->get_property<ExtrusionAttributes>();REQUIRE(attr!=nullptr);
                    CHECK(attr->mm3_per_mm==Approx(midpoint<1. ? .05 : .12));
                }
                if (!reversible) CHECK(result.first_point().x()==scale_i(reverse_source ? 10. : 0.));
            }
}
