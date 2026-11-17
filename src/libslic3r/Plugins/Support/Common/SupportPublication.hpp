///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* Publication of the temporary support slabs as auxiliary layers of an object.

   Each support family that fills a GeneratedSupport (SupportLayers.hpp) calls
   publish() once per object. It creates the auxiliary layers, their support
   properties and their typed fill surfaces through the plugin API, so every
   family publishes its slabs the same way. The grouping and numbering rules are
   explained in SupportPublication.cpp.

   run_support_family() shares the style and demand filters, constructs the run's
   parameters once, slices the object's no-support zones, calls the family's slab
   generator, publishes its slabs and finishes progress. The generated slabs live until publication returns.

       Common::publish(context, object, params, support, orchestrator); */
#ifndef plugins_support_common_publication_hpp_
#define plugins_support_common_publication_hpp_

#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Plugins/Support/Common/SupportLayers.hpp"
#include "libslic3r/Plugins/Support/Common/SupportParams.hpp"

namespace slic3r_api { class PluginProgress; }

namespace slic3r_api::Support::Common {
namespace Internal {
/* Every area of one auxiliary layer is printed by exactly one role. A support layer holds three
   kinds of surfaces: the upper ones (contacts touching the object above and the interfaces below
   them), the lower ones (contacts touching the object below and the interfaces above them) and the
   sparse ones: the base that carries the load down to the bed, and the branch bodies. They must stay
   disjoint, otherwise the same area is extruded twice. When the gap between the object above and the
   object below is thinner than the support layers, one area is both upper and lower: the upper
   surface keeps it, as the native generator merged the lower contact into the upper one. */
// Decide whether the native generator would have merged a contact area into the base layer, which
// makes publication type that contact as base; a bridging contact never merges.
bool merged_into_base(bool merge, bool base_takes_contact, bool z_has_base, bool bridging);
// Remove from the surfaces every area the covering surfaces already print, so each area keeps one owner;
// the result lives in the storage of surfaces.
StoredPolygonCollection resolve_overlap(const StoredPolygonCollection &surfaces, const StoredPolygonCollection &covering);
}
// Remove the no-support zones of support from its slabs, then publish the slabs as auxiliary layers, grouped by print Z and thickness, with one region-island per extruder and clipped fill surfaces; a branch slab becomes sparse support of the base extruder, and one interface id per print Z and the thinnest group published first keep the native numbering and order.
// The slabs and storage stay owned by the caller. Rejected writes leave published layers in place and throw silent
// PluginCancelled when stopping (the cause has its message, the step names it, or the runner rethrows the print
// cancellation), otherwise std::runtime_error. context is the calling plugin's borrowed run context, used only after
// a host callback failure to exit through PluginCancelled when stopping; nullptr is accepted and disables this check.
// Publishes borrowed slabs using the same object's per-run tool and interface parameters.
void publish(const plugin_run_context *context, const Object &object, const SupportParams &params, GeneratedSupport &support, orchestrator_handle *orchestrator);
// A family fills the borrowed run's slabs using its single shared parameter object.
using GenerateSupport = void (*)(const plugin_run_context *, const Print &, const Object &, const SupportParams &, GeneratedSupport &);
// Filters styles and demand, constructs one parameter object, slices the no-support zones, generates and publishes slabs, then finishes valid runs' progress.
void run_support_family(const plugin_run_context *context, orchestrator_handle *orchestrator, PluginProgress &progress,
                        SupportStyle first_style, SupportStyle second_style, GenerateSupport generate,
                        SupportStyle third_style = SupportStyle::Unknown);
} // namespace slic3r_api::Support::Common
#endif
