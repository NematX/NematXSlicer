///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
/* Keep plugin end-to-end tests small by varying one setting at a time and
   reusing tiny STL models. Reject missing coverage of a plugin's declared settings.

   The caller supplies a working, feature-enabled baseline. Define one Domain per
   setting, or per tightly coupled group, then pass the targeted plugin ids to
   linear_cases(). It checks their used/defined keys before returning baseline plus
   independent overrides. apply_case() makes a fresh config for each run; domains
   are never combined into a Cartesian or pairwise matrix.

   Example of one domain, without plugin-declaration checks, in namespace
   Slic3r::Test::PluginE2E:
       const Domain density{{"fill_density"}, {{"sparse", {{"fill_density", "5%"}}}}};
       const std::vector<Case> cases = linear_cases(host, {}, baseline, {density});
       const DynamicPrintConfig sparse = apply_case(baseline, cases.back());

   Add all declared-key domains when supplying plugin ids. Names, keys and values
   are owned strings; no registry pointer is retained. stl_fixture() returns an
   owned mesh after a temporary STL write/read round trip.
*/
#ifndef test_plugins_plugin_e2e_test_helpers_hpp_
#define test_plugins_plugin_e2e_test_helpers_hpp_

#include <string>
#include <utility>
#include <vector>
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/TriangleMesh.hpp"

namespace Slic3r { class Orchestrator; }
namespace Slic3r::Test::PluginE2E {
// Serialized option overrides, not a mutable configuration shared between runs.
using Settings = std::vector<std::pair<std::string, std::string>>;
// A unique case name and only its overrides relative to the unchanged baseline.
struct Case {
    std::string name;
    Settings settings;
};
// Allowed keys and finite variations for one setting or an explicitly coupled group.
struct Domain {
    std::vector<std::string> keys;
    std::vector<Case> cases;
};

// Return baseline plus independent cases, deduplicated by serialized effective
// overrides. Target plugins must be active, and each used/defined key must actually
// vary. Missing coverage, duplicate case names, repeated override keys within a
// case and out-of-group changes throw.
std::vector<Case> linear_cases(const Orchestrator &host, const std::vector<std::string> &plugins,
    const DynamicPrintConfig &baseline, const std::vector<Domain> &domains);
// Copy the baseline and deserialize only this case's overrides; errors propagate
// without mutating baseline. This is not whole-print configuration validation.
DynamicPrintConfig apply_case(const DynamicPrintConfig &baseline, const Case &test);
// Return one case per declared enum value; throw if the option has no enum definition.
Domain enum_domain(const std::string &key);
// Return a notched roof/frame with a raised pad when true, otherwise a solid cube.
// Both use a binary STL round trip. Remove its temporary file on success and try
// cleanup on failure; filesystem and import failures propagate.
TriangleMesh stl_fixture(bool bridge_frame);
}
#endif
