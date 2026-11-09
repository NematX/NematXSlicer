///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#pragma once

// Fan policies share eligibility and inherited-state traversal, not numerical
// policy. Only the current layer's borrowed roots are edited; no handles survive
// the callback. Normalization removes collection requests without creating a
// request on a connector, script or empty service leaf.
//
//     normalize_requests(root, run);
//     for_each_leaf(root, action, run);
//
// Actions may edit only fan pairs, not the structure being traversed. Read a
// context with read_context(tool, key, expected_phase) before applying a policy.
#include <functional>

#include "ExtrusionProcessParameterHelpers.hpp"
#include "libslic3r/Api/plugin/cpp/FanPolicy.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"

namespace slic3r_api { namespace LayerExtrusionEdit { namespace FanTreeHelpers {
using LeafAction = std::function<void(MutableExtrusionEntity, const ProcessParameterHelpers::EffectiveTreeState &)>;

// Materialize effective requests on eligible deposits, then clear collections.
void normalize_requests(MutableExtrusionEntity root, const plugin_run_context *run);
// Visit eligible deposit leaves with their effective attributes/fan/overhang.
void for_each_leaf(MutableExtrusionEntity root, const LeafAction &action, const plugin_run_context *run);
// Read and validate a property produced by a previous plugin, returning a copy.
FanPolicy::Context read_context(const PrintingToolGroup &tool, const PluginPropertyKey<FanPolicy::Context> &key,
                               raw_fan_policy_phase expected);
// Unconfigured overhangs use their perimeter family after late closure.
float closed_reference(const FanPolicy::Context &context, raw_extrusion_role role);
}}} // namespace slic3r_api::LayerExtrusionEdit::FanTreeHelpers
