///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#include "DynamicFan.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

#include "FanTreeHelpers.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_layer_extrusion_edit.h"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"

/*
This layer editor raises modifiable overhang fan requests using the filament's
geometry graph and CLOSED dynamic bounds. It never lowers an existing request,
repeats time interpolation or changes Override/foreign policy modes.

    run_impl(): validate CLOSED even when the optional graph is disabled
    `-- for_each_leaf(): interpolate overhang endpoint distances, clamp and raise

The context key belongs to this host. Configuration and tree views are borrowed
for one callback only; geometry, ordering and service requests remain unchanged.
Layer callbacks edit disjoint trees and propagate cancellation or context errors.
*/

namespace slic3r_api { namespace LayerExtrusionEdit { namespace DynamicFanPlugin {
namespace {
const char *const k_no_dependencies[] = {nullptr};
const raw_used_config_key k_used[] = {
    {"overhangs_dynamic_fan_speed", RAW_CO_VECTOR_GRAPH, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE}
};

class DynamicFan : public PluginBase {
public:
    // Bind the public context key to this provider's host.
    explicit DynamicFan(orchestrator_handle *orch) : PluginBase(orch), m_context(FanPolicy::context_property_key(orch)) {}
private:
    // Return the stable geometry-policy identity.
    const char *id_impl() const noexcept override { return "layer_extrusion_edit.fan.dynamic"; }
    // Return borrowed display metadata.
    const char *name_impl() const noexcept override { return "Dynamic overhang fan"; }
    // Geometric cooling composes with other policies rather than selecting one.
    const char *exclusive_group_impl() const noexcept override { return ""; }
    // Describe the bounded increase without claiming ownership of all fan modes.
    const char *description_impl() const noexcept override { return "Increase overhang fan requests using geometry and closed layer limits."; }
    // Edit the ordered plan's layer clones.
    slicing_step_t step_impl() const noexcept override { return STEP_LAYER_EXTRUSION_EDIT; }
    // Require the CLOSED public phase, not a particular provider ID.
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }
    // Run after first-layer closure has published dynamic bounds.
    int32_t priority_impl() const noexcept override { return LAYER_EXTRUSION_EDIT_PRIORITY_FAN_DYNAMIC; }
    // Copy the graph declaration, or return its count for null output.
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override {
        if (keys) keys[0] = k_used[0];
        return 1;
    }
    // Return the borrowed per-tree progress format.
    const char *progress_message_format_impl() const noexcept override { return "Applying dynamic overhang fan: %u / %u trees"; }
    // Count borrowed trees without editing them.
    void setup_run_impl(const plugin_run_context *run) const override;
    // Increase eligible overhang requests without changing published references.
    void run_impl(const plugin_run_context *run) const override;
    PluginPropertyKey<FanPolicy::Context> m_context;
};

void DynamicFan::setup_run_impl(const plugin_run_context *run) const
{
    const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run);
    if (ctx && ctx->layer_group)
        progress().add_max(ProcessParameterHelpers::extrusion_tree_count(PrintingLayerGroup(ctx->layer_group)));
}

void DynamicFan::run_impl(const plugin_run_context *run) const
{
    const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run);
    if (!ctx || !ctx->print || !ctx->layer_group) return;
    const Config config = Print(ctx->print).config();
    const ConfigOption option = config.get("overhangs_dynamic_fan_speed");
    PrintingLayerGroup layer(ctx->layer_group);
    for (uint32_t idx = 0; idx < layer.tool_group_count(); ++idx) {
        throw_if_cancelled(run);
        PrintingToolGroup tool = layer.tool_group(idx);
        const FanPolicy::Context closed = FanTreeHelpers::read_context(tool, m_context, RAW_FAN_POLICY_PHASE_CLOSED);
        const extruder_index_t extruder = tool.extruder_id();
        const std::optional<GraphData> graph = option.valid() && size_t(extruder) < option.size() && option.is_enabled(extruder) ?
            option.graph(extruder) : std::nullopt;
        if (!graph || !graph->validate()) {
            progress().increment_by(int32_t(tool.extrusion_count()));
            continue;
        }
        for (uint32_t extrusion = 0; extrusion < tool.extrusion_count(); ++extrusion) {
            FanTreeHelpers::for_each_leaf(tool.extrusion(extrusion).mutable_root(),
                [&](MutableExtrusionEntity leaf, const ProcessParameterHelpers::EffectiveTreeState &state) {
                    const raw_extrusion_role role = state.attributes.extrusion_role();
                    if (!RAW_EXTRUSION_ROLE_IS_PERIMETER(role) || !RAW_EXTRUSION_ROLE_HAS(role, RAW_EXTRUSION_ROLE_BRIDGE) ||
                        !state.fan.can_modify(FAN_REQUEST_MODE_AUTOMATIC) || !state.has_overhang ||
                        state.overhang.start_distance_from_prev_layer <= 0.f || state.overhang.end_distance_from_prev_layer <= 0.f)
                        return;
                    const double start = graph->interpolate(100.0 - 100.0 * std::min(1.f, state.overhang.start_distance_from_prev_layer));
                    const double end = graph->interpolate(100.0 - 100.0 * std::min(1.f, state.overhang.end_distance_from_prev_layer));
                    const float dynamic = float(std::clamp(int(std::min(start, end) + 0.5), closed.dynamic_min, closed.dynamic_max));
                    const float percent = std::max(state.fan.is_set() ? state.fan.percent : -1.f, dynamic);
                    leaf.get_or_add(EPropertyFan::key) = FanPolicy::transform_request(state.fan, percent, FAN_REQUEST_MODE_AUTOMATIC);
                }, run);
            progress().increment();
        }
    }
}
} // namespace

void register_dynamic_fan_plugin(orchestrator_handle *orch)
{
    static DynamicFan plugin(orch);
    orchestrator_register_plugin(orch, plugin.c_instance());
}
}}} // namespace slic3r_api::LayerExtrusionEdit::DynamicFanPlugin
