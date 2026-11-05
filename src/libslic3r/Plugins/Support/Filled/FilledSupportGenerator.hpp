///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* The filled generator projects final island demand into dense slabs at object-layer heights.
   Call generate_support, then publish through Common/SupportPublication to assign contact marking.
   The supplied run storage owns the slabs; this generator creates no extrusion paths. */
#ifndef plugins_support_filled_generator_hpp_
#define plugins_support_filled_generator_hpp_
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Plugins/Support/Common/SupportLayers.hpp"
#include "libslic3r/Plugins/Support/Common/SupportParams.hpp"

namespace slic3r_api::Support::Filled {
// Refuse non-soluble support or a raft, then append storage-owned slabs and a foot without changing the object.
// Null context disables cancellation; cancellation, geometry and allocation failures propagate.
void generate_support(const plugin_run_context *context, const Print &print, const Object &object,
                      const Common::SupportParams &params, Common::GeneratedSupport &out);
}
#endif
