///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "DefaultLayerEntryState.hpp"
#include "ExtrusionScopeHelpers.hpp"

#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include "libslic3r/Api/plugin/c/steps/slic3r_step_layer_extrusion_edit.h"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/Api/plugin/cpp/properties/ExtrusionProperties.hpp"
#include "libslic3r/Plugins/PrintingPlan/PrintingLayerEntryStateProperties.h"

namespace slic3r_api { namespace LayerExtrusionEdit { namespace DefaultLayerEntryStatePlugin {
// Register the permanent borrowed plugin and copy its alternative-selection setting description into the application.
// Print data is unchanged; allocation failures or rejected descriptions throw.
void register_default_layer_entry_state_plugin(orchestrator_handle *orchestrator);

namespace {

/*
Default layer entry-state producer
==================================

This plugin publishes two properties on every PrintingLayerGroup:

- position is the planned machine position immediately before the current
   layer-group starts. It is the last printable endpoint, segmented or DEPOSIT,
   visited earlier in PrintingPlan order. The position is unknown before the
   first such endpoint. Other leaves without segments do not change it.
- tool is the extruder selected before the current layer-group is visited.
  UINT16_MAX means that no tool has been selected yet.

The values are deliberately snapshots of the ordered plan, not the current
G-code position. Firmware events and synthetic moves added by other plugins
are not part of the scan. Travel plugins use the snapshot to decide whether
such a move still needs to be materialized.

The host calls setup() once, then calls setup_run() and run() for each
PrintingLayerGroup. Because an entry state depends on all preceding groups,
setup() performs one sequential prefix scan and stores a value copy indexed by
{group_idx, layer_group_idx}. After the host's setup barrier, each run only
reads its own cache entry and writes its own layer-group properties. No worker
needs to inspect or modify a predecessor or successor.
*/

struct LayerEntryState
{
    // State immediately before one layer-group in the ordered plan.
    PrintingLayerEntryPositionProperty position = {};

    // Tool selected immediately before that layer-group starts.
    PrintingLayerEntryToolProperty tool = {};
};

/* This producer reads the ordered plan directly; its consumers declare the dependency. */
const char *k_no_dependencies[] = { nullptr };

/* Add two scaled coordinates and report overflow instead of allowing Z to wrap. */
coord_t checked_add(coord_t lhs, coord_t rhs);

/*
Update the running position with the last segmented or DEPOSIT endpoint in one extrusion
tree. Children are visited in stored execution order, followed by the entity's
own printable endpoints. A direct Z-offset property replaces the inherited offset for
the complete subtree; passing the resolved value by copy restores the parent's
offset automatically for the next sibling. Other leaves without segments do not
change the running position.
*/
// Scan borrowed geometry in machine order and update the next layer's absolute entry position, including deposits; checked Z overflow throws and the extrusion tree is unchanged.
void update_position_from_entity(const ExtrusionEntity &entity,
                                 coord_t print_z,
                                 coord_t inherited_z_offset,
                                 PrintingLayerEntryPositionProperty &position);

/*
Build the cache in the same structural order as the plan writer: groups,
layer-groups, tool-groups, then extrusion roots. Store the current state before
visiting each layer-group; the following visit updates the state for its
successor. Empty layers therefore copy the previous state, while an empty
tool-group still changes the selected tool. The running state intentionally
continues across group boundaries.
*/
std::vector<std::vector<LayerEntryState>> collect_layer_entry_states(const PrintingPlan &plan);

/*
PluginBase adapter for the default entry-state producer. The cache is rebuilt
for each setup and is read-only once the host starts parallel run callbacks.
*/
class DefaultLayerEntryState : public PluginBase
{
public:
    static DefaultLayerEntryState &instance(orchestrator_handle *orchestrator);
    explicit DefaultLayerEntryState(orchestrator_handle *orchestrator);

private:
    const char *id_impl() const noexcept override;
    const char *name_impl() const noexcept override;
    const char *description_impl() const noexcept override;
    const char *exclusive_group_impl() const noexcept override;
    slicing_step_t step_impl() const noexcept override;
    const char *const *dependencies_impl() const noexcept override;
    int32_t priority_impl() const noexcept override;
    const char *progress_message_format_impl() const noexcept override;
    void setup_impl(const plugin_run_context *run_ctx, uint32_t run_count) const override;
    void setup_run_impl(const plugin_run_context *run_ctx) const override;
    void run_impl(const plugin_run_context *run_ctx) const override;

    // Runtime keys for the two properties consumed by later layer-edit plugins.
    PluginPropertyKey<PrintingLayerEntryPositionProperty> m_position_property;
    PluginPropertyKey<PrintingLayerEntryToolProperty> m_tool_property;

    // Setup is invalid until a complete cache has been built for the current plan.
    mutable bool m_setup_valid = false;

    // [group_idx][layer_group_idx], with one value copy per planned layer-group.
    mutable std::vector<std::vector<LayerEntryState>> m_entry_states;
};

coord_t checked_add(const coord_t lhs, const coord_t rhs)
{
    if ((rhs > 0 && lhs > (std::numeric_limits<coord_t>::max)() - rhs) ||
        (rhs < 0 && lhs < (std::numeric_limits<coord_t>::min)() - rhs))
        throw std::overflow_error("A PrintingLayerGroup entry Z position exceeds coord_t.");
    return lhs + rhs;
}

void update_position_from_entity(const ExtrusionEntity &entity,
                                 const coord_t print_z,
                                 const coord_t inherited_z_offset,
                                 PrintingLayerEntryPositionProperty &position)
{
    coord_t z_offset = inherited_z_offset;
    if (const EPropertyZOffset *direct_offset = entity.get(EPropertyZOffset::key))
        z_offset = direct_offset->get();

    // Pass the resolved offset by value so a child cannot change its siblings'
    // inherited Z offset.
    for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx)
        update_position_from_entity(entity.child(child_idx), print_z, z_offset, position);

    ExtrusionScope::LeafEndpoints endpoints;
    if (!ExtrusionScope::leaf_endpoints(entity, endpoints))
        return;

    position.x = endpoints.last.x;
    position.y = endpoints.last.y;
    position.z = checked_add(checked_add(print_z, z_offset), endpoints.last_z);
    position.state = RAW_PRINTING_LAYER_ENTRY_POSITION_KNOWN;
}

std::vector<std::vector<LayerEntryState>> collect_layer_entry_states(
    const ConstPrintingPlan &plan)
{
    std::vector<std::vector<LayerEntryState>> states(plan.group_count());
    LayerEntryState current = {};
    current.position.state = RAW_PRINTING_LAYER_ENTRY_POSITION_UNKNOWN;
    current.tool.extruder_id = EXTRUDER_ID_UNKNOWN;

    for (uint32_t group_idx = 0; group_idx < plan.group_count(); ++group_idx) {
        const ConstPrintingGroup group = plan.group(group_idx);
        states[group_idx].reserve(group.layer_group_count());
        for (uint32_t layer_idx = 0; layer_idx < group.layer_group_count(); ++layer_idx) {
            const ConstPrintingLayerGroup layer = group.layer_group(layer_idx);

            // The cached value is the state on entry, not the state after this
            // layer. This also makes empty layers preserve continuity.
            states[group_idx].push_back(current);

            for (uint32_t tool_idx = 0; tool_idx < layer.tool_group_count(); ++tool_idx) {
                const ConstPrintingToolGroup tool = layer.tool_group(tool_idx);

                // Tool selection is part of the serialized order even when this
                // tool-group contains no extrusion geometry.
                current.tool.extruder_id = tool.extruder_id();
                for (uint32_t extrusion_idx = 0; extrusion_idx < tool.extrusion_count(); ++extrusion_idx)
                    update_position_from_entity(
                        tool.extrusion(extrusion_idx).root(), layer.print_z(), 0, current.position);
            }
        }
    }
    return states;
}

DefaultLayerEntryState &DefaultLayerEntryState::instance(orchestrator_handle *orchestrator)
{
    // The C ABI stores a non-owning pointer, so keep the plugin alive for the
    // lifetime of the orchestrator and all callbacks made through it.
    static DefaultLayerEntryState plugin(orchestrator);
    return plugin;
}

DefaultLayerEntryState::DefaultLayerEntryState(orchestrator_handle *orchestrator) :
    PluginBase(orchestrator),
    m_position_property(printing_layer_entry_position_property_key(orchestrator)),
    m_tool_property(printing_layer_entry_tool_property_key(orchestrator))
{}

const char *DefaultLayerEntryState::id_impl() const noexcept
{
    return "layer_extrusion_edit.entry_state.default";
}

const char *DefaultLayerEntryState::name_impl() const noexcept
{
    return "Default layer entry state";
}

const char *DefaultLayerEntryState::description_impl() const noexcept
{
    return "Publishes the planned entry position and active tool of every ordered layer.";
}

const char *DefaultLayerEntryState::exclusive_group_impl() const noexcept
{
    return "layer_extrusion_edit.entry_state";
}

slicing_step_t DefaultLayerEntryState::step_impl() const noexcept
{
    return STEP_LAYER_EXTRUSION_EDIT;
}

const char *const *DefaultLayerEntryState::dependencies_impl() const noexcept
{
    // The plan already exists when this provider runs; only downstream
    // layer-edit providers depend on the properties published here.
    return k_no_dependencies;
}

int32_t DefaultLayerEntryState::priority_impl() const noexcept
{
    // Publish the entry state before providers that consume it run.
    return -70;
}

const char *DefaultLayerEntryState::progress_message_format_impl() const noexcept
{
    return "Publishing layer entry state: %u / %u layers";
}

void DefaultLayerEntryState::setup_impl(const plugin_run_context *run_ctx, uint32_t) const
{
    // Discard the previous plan's cache before validating the new context.
    m_setup_valid = false;
    m_entry_states.clear();

    const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run_ctx);
    if (ctx == nullptr || ctx->plan == nullptr)
        return;

    // Build every entry state before any parallel worker starts. Only value
    // copies are retained, and run callbacks address them by context index.
    m_entry_states = collect_layer_entry_states(ConstPrintingPlan(ctx->plan));
    m_setup_valid = true;
}

void DefaultLayerEntryState::setup_run_impl(const plugin_run_context *run_ctx) const
{
    // State computation is already complete. This phase only accounts for one
    // future run so the progress total matches the number of layer-groups.
    if (!m_setup_valid)
        return;
    const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run_ctx);
    if (ctx != nullptr && ctx->layer_group != nullptr)
        progress().add_max(1);
}

void DefaultLayerEntryState::run_impl(const plugin_run_context *run_ctx) const
{
    // Runs may be concurrent. The cache is immutable after setup, and each
    // context identifies a distinct layer-group whose property bag it updates.
    if (!m_setup_valid)
        return;
    const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run_ctx);
    if (ctx == nullptr || ctx->layer_group == nullptr || ctx->group_idx >= m_entry_states.size() ||
        ctx->layer_group_idx >= m_entry_states[ctx->group_idx].size())
        return;

    const LayerEntryState &entry = m_entry_states[ctx->group_idx][ctx->layer_group_idx];
    PrintingLayerGroup layer(ctx->layer_group);
    PluginProperties properties = layer.properties();
    m_position_property.get_or_add(properties) = entry.position;
    m_tool_property.get_or_add(properties) = entry.tool;
    progress().increment();
}

} // namespace

void register_default_layer_entry_state_plugin(orchestrator_handle *orchestrator)
{
    orchestrator_register_plugin(
        orchestrator, DefaultLayerEntryState::instance(orchestrator).c_instance());
    const plugin_instance member = DefaultLayerEntryState::instance(orchestrator).c_instance();
    raw_exclusive_group_def definition = raw_exclusive_group_def_init();
    definition.exclusive_group = member.vt->get_exclusive_group(member.ctx);
    definition.label = "Layer entry state";
    definition.tooltip = "Selects how parallel layer editors receive their planned entry state.";
    DefaultLayerEntryState::instance(orchestrator).register_exclusive_group(STEP_LAYER_EXTRUSION_EDIT, definition);
}

}}} // namespace slic3r_api::LayerExtrusionEdit::DefaultLayerEntryStatePlugin
