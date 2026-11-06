///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
/* Check that end-to-end test cases vary settings independently and reject
   incomplete case definitions. These tests do not run a slicer.

   The test calls linear_cases() with independent domains, one coupled group,
   duplicate configurations and invalid definitions, then checks apply_case().
   Expected counts are explicit sums of domain cases, not a second matrix builder.
   Configuration comparisons check that overrides do not accumulate or alter the
   caller's baseline. Runtime registration is initialized, not reconfigured here.

   Representative direct calls from TEST_CASE:

       TEST_CASE
           |-- linear_cases() [case definitions and declared-key coverage]
           `-- apply_case() [independent effective configurations]

   The assertions check counts, rejected definitions and the unchanged baseline.
*/
#include <catch2/catch.hpp>
#include "plugin_e2e_test_helpers.hpp"
#include "plugin_test_helpers.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"

TEST_CASE("Plugin E2E domains stay linear and reject incomplete case definitions", "[plugins][plugin-e2e-catalog]")
{
    using namespace Slic3r;
    namespace E2E = Slic3r::Test::PluginE2E;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const DynamicPrintConfig baseline = DynamicPrintConfig::full_print_config();
    const std::vector<E2E::Domain> independent{
        {{"perimeters"}, {{"one wall", {{"perimeters", "1"}}}, {"two walls", {{"perimeters", "2"}}}}},
        {{"fill_density"}, {{"sparse", {{"fill_density", "5%"}}}, {"full", {{"fill_density", "100%"}}}}}
    };
    Orchestrator &host = Orchestrator::instance();
    const std::vector<E2E::Case> cases = E2E::linear_cases(host, {}, baseline, independent);
    REQUIRE(cases.size() == 5);
    CHECK(cases.front().name == "default");
    for (const E2E::Case &test : cases) {
        const DynamicPrintConfig config = E2E::apply_case(baseline, test);
        CHECK(baseline.diff(config).size() <= 1);
    }
    std::vector<E2E::Domain> coupled = independent;
    coupled.push_back({{"perimeters", "fill_density"}, {{"wall-free full fill", {{"perimeters", "0"}, {"fill_density", "100%"}}}}});
    CHECK(E2E::linear_cases(host, {}, baseline, coupled).size() == 6);
    std::vector<E2E::Domain> invalid = independent;
    invalid.front().cases.front().settings.push_back({"nozzle_diameter", "0.6"});
    CHECK_THROWS_WITH(E2E::linear_cases(host, {}, baseline, invalid), Catch::Contains("outside its group"));
    invalid = independent;
    invalid.back().cases.front().name = "one wall";
    CHECK_THROWS_WITH(E2E::linear_cases(host, {}, baseline, invalid), Catch::Contains("Duplicate"));
    invalid = {{{"fill_density"}, {{"half 1", {{"fill_density", "50%"}}}, {"half 2", {{"fill_density", "50%"}}}}}};
    CHECK(E2E::linear_cases(host, {}, baseline, invalid).size() == 2);
    CHECK_THROWS_WITH(E2E::apply_case(baseline, {"invalid toggle", {{"bridged_infill_margin", "!0"}}}),
        Catch::Contains("cannot be disabled"));
    CHECK(baseline.optptr("perimeters")->serialize() == DynamicPrintConfig::full_print_config().optptr("perimeters")->serialize());
}
