///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#ifndef slic3r_Api_plugin_cpp_RetractionCrossingPolicy_hpp_
#define slic3r_Api_plugin_cpp_RetractionCrossingPolicy_hpp_

#include <cstdint>
#include "libslic3r/Api/plugin/cpp/PluginPropertyKey.hpp"

/* Policy for ordinary travel retraction, stored on each PrintingExtrusion root.
   Register the key once per orchestrator. A layer editor running after transition
   creation and before retraction planning writes the policy on its own roots:

       retraction_crossing_policy_key(host).get_or_add(root).exclude_top_surfaces = 1;

   The common retraction planner then treats published top surfaces as exposed
   instead of safe interior. This does not force a retraction: distance, explicit
   disable modifiers and tool/layer requests retain their existing semantics.
   Absence and zero both retain the ordinary perimeter-only mask. The payload is
   copied into endpoint snapshots; it holds no pointers and is not serialized. */
namespace slic3r_api {

struct RetractionCrossingPolicy
{
    uint8_t exclude_top_surfaces = 0;
};

// Share one checked payload layout between the host and external policy editors.
inline PluginPropertyKey<RetractionCrossingPolicy>
retraction_crossing_policy_key(orchestrator_handle *orchestrator)
{
    return PluginPropertyKey<RetractionCrossingPolicy>::register_dynamic(
        orchestrator, "slic3r.printing_plan.retraction_crossing_policy");
}

}
#endif
