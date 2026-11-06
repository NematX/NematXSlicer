///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
/* Check native and plugin bridge flow against role-specific nozzle dimensions.
   prepare_cube_print() supplies borrowed regions to infill_bridge_flow() and
   the C adapters. Independent nozzle, height and solid-area formulas verify
   all modes without relying on a bridge preparation or finalization provider.
   Each fixture owns its model and print until its flow checks finish.

   Representative sibling calls in the flow TEST_CASE:

       TEST_CASE
           |-- prepare_cube_print() [provide real borrowed regions]
           |-- infill_bridge_flow() [native result]
           |-- layer_region_get_infill_bridge_flow_mm() [unscaled C result]
           `-- layer_region_get_infill_bridge_flow() [scaled C transport]
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
