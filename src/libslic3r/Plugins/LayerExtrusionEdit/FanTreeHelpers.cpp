///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#include "FanTreeHelpers.hpp"

#include <stdexcept>

// Traversal preserves all unrelated properties, geometry, ordering and direct
// service requests. Downstream policies never compact across role boundaries.
//
//     normalize_requests() -> normalize_tree(): materialize inherited deposition requests
//     for_each_leaf() -> visit_tree(): visit only eligible leaves with effective state
//     read_context(): validate and copy the previous policy's public record
//     closed_reference(): select the closed role fallback without editing it
//
// Traversal borrows the current layer's trees only. Actions must not mutate their
// structure; cancellation and action exceptions propagate to the owning policy.
namespace slic3r_api { namespace LayerExtrusionEdit { namespace FanTreeHelpers {
using namespace ProcessParameterHelpers;
namespace {
// Resolve inheritance before clearing a collection so descendants retain it.
void normalize_tree(MutableExtrusionEntity entity, const EffectiveTreeState &parent, const plugin_run_context *run);
// The action may only edit the leaf's fan pair, not the tree structure.
void visit_tree(MutableExtrusionEntity entity, const EffectiveTreeState &parent,
                const LeafAction &action, const plugin_run_context *run);

void normalize_tree(MutableExtrusionEntity entity, const EffectiveTreeState &parent, const plugin_run_context *run)
{
    throw_if_cancelled(run);
    const EffectiveTreeState state = effective_state(entity, parent);
    if (entity.child_count() > 0) {
        entity.remove(EPropertyFan::key);
        for (uint32_t idx = 0; idx < entity.child_count(); ++idx)
            normalize_tree(entity.child_mutable(idx), state, run);
    } else if (leaf_disposition(entity, state, LeafPolicy::Fan) == LeafDisposition::Editable && state.fan.is_set()) {
        const EPropertyFan *direct = entity.get(EPropertyFan::key);
        if (direct == nullptr || !direct->is_set())
            entity.get_or_add(EPropertyFan::key) = state.fan;
    }
}

void visit_tree(MutableExtrusionEntity entity, const EffectiveTreeState &parent,
                const LeafAction &action, const plugin_run_context *run)
{
    throw_if_cancelled(run);
    const EffectiveTreeState state = effective_state(entity, parent);
    if (entity.child_count() > 0) {
        for (uint32_t idx = 0; idx < entity.child_count(); ++idx)
            visit_tree(entity.child_mutable(idx), state, action, run);
    } else if (leaf_disposition(entity, state, LeafPolicy::Fan) == LeafDisposition::Editable) {
        action(entity, state);
    }
}
} // namespace

void normalize_requests(MutableExtrusionEntity root, const plugin_run_context *run)
{
    normalize_tree(root, EffectiveTreeState{}, run);
}

void for_each_leaf(MutableExtrusionEntity root, const LeafAction &action, const plugin_run_context *run)
{
    visit_tree(root, EffectiveTreeState{}, action, run);
}

FanPolicy::Context read_context(const PrintingToolGroup &tool, const PluginPropertyKey<FanPolicy::Context> &key,
                               raw_fan_policy_phase expected)
{
    const FanPolicy::Context *context = key.get(tool.properties());
    if (context == nullptr)
        throw std::runtime_error("Fan policy needs a context from the preceding stage.");
    context->validate(expected);
    return *context;
}

float closed_reference(const FanPolicy::Context &context, raw_extrusion_role role)
{
    FanPolicy::Role fan_role = FanPolicy::fan_role(role);
    if (fan_role == RAW_FAN_ROLE_COUNT)
        return -1.f;
    // A directly configured overhang was filled by BASE already. Remaining
    // absences inherit the closed perimeter family, not a synthesized bridge
    // reference that would bypass the perimeter's first-layer ramp.
    if (fan_role == RAW_FAN_ROLE_OVERHANG_PERIMETER)
        fan_role = RAW_EXTRUSION_ROLE_HAS(role, RAW_EXTRUSION_ROLE_EXTERNAL) ?
            RAW_FAN_ROLE_EXTERNAL_PERIMETER : RAW_FAN_ROLE_PERIMETER;
    return float(context.roles[fan_role].percent);
}
}}} // namespace slic3r_api::LayerExtrusionEdit::FanTreeHelpers
