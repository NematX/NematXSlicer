///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

// LayerInputCache supplies copied layer geometry to travel and wipe queries.
// Call get() with a public layer from the same stable print and a non-null run
// context whose plugin_storage is valid. On the first lookup it finds the owning
// object (including the print's global auxiliary object), resolves the normal or
// auxiliary layer index, and calls build_input(). Later lookups use only the layer
// handle as a key: edits to the print do not refresh the copy.
// Use one cache per worker and one unchanged print. Returned references stay valid
// until the cache is destroyed; keep it alive while crossing tests or routers
// borrow its inputs. The cache owns these copies, not the print or run context.
// Example, with print, layer and ctx from the current worker:
//     LayerInputCache cache;
//     const Input &input = cache.get(print, layer, ctx);
//     // Use *input.target while cache and the unchanged print remain alive.
#ifndef slic3r_plugins_layer_input_cache_hpp_
#define slic3r_plugins_layer_input_cache_hpp_

#include <map>
#include <stdexcept>
#include "AvoidCrossingInput.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"

namespace slic3r_api { namespace LayerExtrusionEdit { namespace AvoidCrossing {

class LayerInputCache
{
    std::map<const layer_handle *, Input> m_inputs;
public:
    // Return the copy keyed by layer handle; only misses check cancellation and scan the print tree.
    // Require non-null ctx with valid storage; throw if the layer is absent, or if copying or cancellation fails.
    const Input &get(const Print &print, const Layer &layer, const plugin_run_context *ctx)
    {
        const auto found = m_inputs.find(layer.handle());
        if (found != m_inputs.end())
            return found->second;
        // The final iteration resolves platter-space brim and skirt layers outside the object list.
        for (uint32_t object_idx = 0; object_idx <= print.object_count(); ++object_idx) {
            throw_if_cancelled(ctx);
            const bool global_object = object_idx == print.object_count();
            const Object object = global_object ? print.auxiliary_object() : print.object(object_idx);
            if (object.handle() != layer.object().handle())
                continue;
            for (const bool auxiliary : {false, true}) {
                const uint32_t count = auxiliary ? object.auxiliary_layer_count() : object.layer_count();
                for (uint32_t idx = 0; idx < count; ++idx) {
                    throw_if_cancelled(ctx);
                    const Layer candidate = auxiliary ? object.auxiliary_layer(idx) : object.layer(idx);
                    if (candidate.handle() == layer.handle())
                        return m_inputs.emplace(layer.handle(), build_input(
                            ctx->plugin_storage, print, {global_object ? 0 : object_idx, idx, auxiliary, global_object})).first->second;
                }
            }
        }
        throw std::runtime_error("A target layer is absent from the public print tree.");
    }
};

}}}
#endif
