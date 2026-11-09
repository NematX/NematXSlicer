///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
CreateToolChange.cpp
====================

Implements the built-in tool-selection producer declared in CreateToolChange.hpp:
the toolchange_gcode option definition and its activation rule, the provider
callbacks, and the registration entry point. The setup scan and the per-layer
run that materialize every real tool change in the ordered PrintingPlan are
documented in the header.
*/

#include "CreateToolChange.hpp"

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <iterator>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_layer_extrusion_edit.h"
#include "libslic3r/Api/plugin/cpp/ConfigViews.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/Api/plugin/cpp/properties/ExtrusionProperties.hpp"

/*
Default tool-change event producer
==================================

The layer-extrusion-edit step executes one run per PrintingLayerGroup, possibly
in parallel. Tool selection, however, is a prefix state across the complete
plan. setup_impl() therefore performs one cheap hierarchy-only scan and stores
the tool active before each layer. Unconstrained visits
(EXTRUDER_ID_UNKNOWN) receive that tool during this serialized scan.
No vectors are resized or reordered; subsequent plugins see concrete IDs.
Leading unconstrained visits use the first explicit tool, or tool 0 if none is
requested anywhere in the plan. run_impl() can then replay only its local
tool-groups and append events without consulting or modifying another layer.

No extrusion tree is visited here. CreateTransitionScope later observes the
same ordered tool-group sequence and marks its geometric boundaries, while
CreateRetraction uses those flags only to choose tool-change E settings.
*/

namespace slic3r_api { namespace LayerExtrusionEdit { namespace CreateToolChangePlugin {
// Register the permanent borrowed plugin and copy its alternative-selection setting description into the application.
// Print data is unchanged; allocation failures or rejected descriptions throw.
void register_create_tool_change_plugin(orchestrator_handle *orchestrator);

namespace {

const char *const k_no_dependencies[] = {nullptr};

const char *const k_defined_config_keys[] = {
    "toolchange_gcode"
};

void initialize_create_tool_change_settings(orchestrator_handle *orchestrator)
{
    /*
    Option definition owned by this module. It fixes the option type, override
    scope, default and GUI metadata; the defined key is published in the print
    presets, so profiles read and write it by name like any built-in setting.
    Called once from inilialize_impl().
    */
    raw_config_option_def def = raw_config_option_def_init();
    def.opt_key = "toolchange_gcode";
    def.type = RAW_CO_STRING;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINTER;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_CUSTOMGCODE;
    def.label = "Tool change G-code";
    def.tooltip = "This custom code is inserted at every extruder change. If you don't leave this empty, you are "
        "expected to take care of the toolchange yourself - Slic3r will not output any other G-code to "
        "change the filament. You can use placeholder variables for all Slic3r settings as well as {toolchange_z}, {layer_z}, {layer_num}, {max_layer_z}, {previous_extruder} "
        "and {next_extruder}, so e.g. the standard toolchange command can be scripted as T{next_extruder}."
        "!! Warning !!: if any character is written here, Slic3r won't output any toolchange command by itself. "
        "A negative E move counts as a retraction only with '; retraction' on the same line, "
        "and a positive E move restores it only with '; unretraction'; other E moves do not "
        "change the retraction state.";
    def.multiline = 1;
    def.full_width = 1;
    def.height = 5;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_PRUSA;
    def.default_serialized_value = "";
    orchestrator_create_option_def(orchestrator, &def);
}

/* Immutable prefix facts consumed by one parallel layer run. */
struct LayerToolState
{
    int32_t layer_number = -1;
    extruder_index_t incoming_extruder = EXTRUDER_ID_UNKNOWN;
    coord_t print_z = 0;
};

/* Find the initial tool for leading unconstrained visits; fall back to tool 0. */
extruder_index_t initial_plan_extruder(const PrintingPlan &plan);

/* Return true when a setting contains at least one non-whitespace character. */
bool has_visible_text(const std::string &text);

/* Add one typed integer argument to a script's temporary Config. */
void set_int_argument(MutableConfig &config, const char *key, int32_t value);

/* Add one typed floating-point argument to a script's temporary Config. */
void set_float_argument(MutableConfig &config, const char *key, double value);

/* Append a configured script or semantic fallback to one target tool visit. */
void append_tool_change_event(storage_handle *storage,
                              PrintingToolGroup tool_group,
                              const std::string &script,
                              int32_t layer_number,
                              coord_t layer_z,
                              coord_t max_layer_z,
                              extruder_index_t previous_extruder,
                              extruder_index_t next_extruder);

/* Built-in producer registered in the layer-extrusion-edit plugin chain. */
class CreateToolChange final : public PluginBase
{
public:
    /* Return the process-wide built-in instance registered in one orchestrator. */
    static CreateToolChange &instance(orchestrator_handle *orchestrator);

    /* Initialize the generic plugin facade; this provider owns no property key. */
    explicit CreateToolChange(orchestrator_handle *orchestrator);

private:
    /* Plugin identity and user-facing metadata consumed by the orchestrator. */
    const char *id_impl() const noexcept override;
    const char *name_impl() const noexcept override;
    const char *description_impl() const noexcept override;
    const char *exclusive_group_impl() const noexcept override;
    slicing_step_t step_impl() const noexcept override;
    const char *const *dependencies_impl() const noexcept override;
    int32_t priority_impl() const noexcept override;
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override;
    int32_t defined_config_keys(const char **keys) const noexcept override;
    void inilialize_impl(storage_handle *) const override;
    const char *progress_message_format_impl() const noexcept override;

    /* Prefix-scan tool visits without traversing any extrusion tree. */
    void setup_impl(const plugin_run_context *run_ctx, uint32_t run_count) const override;

    /* Count one work unit for each valid future layer run. */
    void setup_run_impl(const plugin_run_context *run_ctx) const override;

    /* Materialize only the tool changes owned by the current layer. */
    void run_impl(const plugin_run_context *run_ctx) const override;

    /* False until setup_impl() publishes a complete layer-indexed cache. */
    mutable bool m_setup_valid = false;

    /* One immutable prefix-state entry for every final group/layer pair. */
    mutable std::vector<std::vector<LayerToolState>> m_layer_states;

    /* Highest final layer Z made available to toolchange_gcode placeholders. */
    mutable coord_t m_max_layer_z = 0;

    /* Serialize event construction; PluginStorage synchronizes allocation/free bookkeeping, not edits or object lifetimes. */
    mutable std::mutex m_storage_mutex;
};

extruder_index_t initial_plan_extruder(const PrintingPlan &plan)
{
    for (uint32_t group_idx = 0; group_idx < plan.group_count(); ++group_idx) {
        const ConstPrintingGroup group = plan.group(group_idx);
        for (uint32_t layer_idx = 0; layer_idx < group.layer_group_count(); ++layer_idx) {
            const ConstPrintingLayerGroup layer = group.layer_group(layer_idx);
            for (uint32_t tool_idx = 0; tool_idx < layer.tool_group_count(); ++tool_idx) {
                const extruder_index_t tool = layer.tool_group(tool_idx).extruder_id();
                if (tool != EXTRUDER_ID_UNKNOWN)
                    return tool;
            }
        }
    }
    return 0;
}

bool has_visible_text(const std::string &text)
{
    return text.find_first_not_of(" \t\r\n") != std::string::npos;
}

void set_int_argument(MutableConfig &config, const char *key, const int32_t value)
{
    config.get_or_add(key, SLIC3R_CONFIG_OPTION_INT).set_int(value);
}

void set_float_argument(MutableConfig &config, const char *key, const double value)
{
    config.get_or_add(key, SLIC3R_CONFIG_OPTION_FLOAT).set_float(value);
}

void append_tool_change_event(storage_handle *storage,
                              PrintingToolGroup tool_group,
                              const std::string &script,
                              const int32_t layer_number,
                              const coord_t layer_z,
                              const coord_t max_layer_z,
                              const extruder_index_t previous_extruder,
                              const extruder_index_t next_extruder)
{
    if (storage == nullptr)
        throw std::invalid_argument("A tool-change event needs plugin storage.");

    // Build the complete temporary event before transferring it into the
    // tool-group. append_before_move() then moves both text and Config buffers
    // into the PrintingPlan-owned event tree.
    StoredExtrusionEntity event(storage);
    event.set_can_rotate(false);
    if (has_visible_text(script)) {
        StoredConfig arguments(storage);
        set_int_argument(arguments, "layer_num", layer_number);
        set_float_argument(arguments, "layer_z", unscaled(layer_z));
        set_float_argument(arguments, "max_layer_z", unscaled(max_layer_z));
        set_int_argument(arguments, "previous_extruder", previous_extruder);
        set_int_argument(arguments, "next_extruder", next_extruder);
        set_float_argument(arguments, "toolchange_z", unscaled(layer_z));
        event.script_gcode(
            script, GCODE_SCRIPT_TYPE_TOOLCHANGE_GCODE, arguments,
            next_extruder);
    } else {
        event.get_or_add(EPropertySpecialCommand::key).set(
            C_EXTRUSION_SPECIAL_COMMAND_TOOLCHANGE, double(next_extruder));
    }

    MutableExtrusionEntity appended =
        tool_group.events().append_before_move(event.mutable_view());
    if (!appended.valid())
        throw std::runtime_error(
            "Unable to append a tool-change event to the PrintingToolGroup.");
}

CreateToolChange &CreateToolChange::instance(orchestrator_handle *orchestrator)
{
    static CreateToolChange plugin(orchestrator);
    return plugin;
}

CreateToolChange::CreateToolChange(orchestrator_handle *orchestrator) :
    PluginBase(orchestrator)
{
}

const char *CreateToolChange::id_impl() const noexcept
{
    return "layer_extrusion_edit.toolchange.default";
}

const char *CreateToolChange::name_impl() const noexcept
{
    return "Default tool changes";
}

const char *CreateToolChange::description_impl() const noexcept
{
    return "Resolves unconstrained tool visits to the current tool and places tool-selection events before actual changes.";
}

const char *CreateToolChange::exclusive_group_impl() const noexcept
{
    return "layer_extrusion_edit.toolchange";
}

slicing_step_t CreateToolChange::step_impl() const noexcept
{
    return STEP_LAYER_EXTRUSION_EDIT;
}

const char *const *CreateToolChange::dependencies_impl() const noexcept
{
    return k_no_dependencies;
}

int32_t CreateToolChange::priority_impl() const noexcept
{
    return -110;
}

int32_t CreateToolChange::used_config_keys(
    raw_used_config_key *keys) const noexcept
{
    if (keys != nullptr)
        keys[0] = raw_used_config_key{
            "toolchange_gcode", RAW_CO_STRING,
            RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE};
    return 1;
}

int32_t CreateToolChange::defined_config_keys(const char **keys) const noexcept
{
    if (keys != nullptr)
        for (size_t idx = 0; idx < std::size(k_defined_config_keys); ++idx)
            keys[idx] = k_defined_config_keys[idx];
    return int32_t(std::size(k_defined_config_keys));
}

void CreateToolChange::inilialize_impl(storage_handle *) const
{
    initialize_create_tool_change_settings(m_orchestrator);
}

const char *CreateToolChange::progress_message_format_impl() const noexcept
{
    return "Creating tool changes: %u / %u layers";
}

void CreateToolChange::setup_impl(const plugin_run_context *run_ctx,
                                  const uint32_t run_count) const
{
    m_setup_valid = false;
    m_layer_states.clear();
    m_max_layer_z = 0;

    const run_ctx_layer_extrusion_edition *ctx =
        plugin_ctx_as_layer_extrusion_edition(run_ctx);
    if (ctx == nullptr || ctx->plan == nullptr)
        return;

    PrintingPlan plan(ctx->plan);
    m_layer_states.resize(plan.group_count());
    extruder_index_t current_extruder = initial_plan_extruder(plan);
    int32_t layer_number = 0;
    uint32_t counted_layers = 0;

    // This prefix scan visits only the three PrintingPlan vectors. Empty tool
    // visits deliberately update current_extruder because the file writer will
    // still enter their tool-group and therefore change the selection history.
    for (uint32_t group_idx = 0; group_idx < plan.group_count(); ++group_idx) {
        PrintingGroup group = plan.group(group_idx);
        std::vector<LayerToolState> &group_states = m_layer_states[group_idx];
        group_states.reserve(group.layer_group_count());
        for (uint32_t layer_idx = 0;
             layer_idx < group.layer_group_count(); ++layer_idx) {
            PrintingLayerGroup layer = group.layer_group(layer_idx);
            group_states.push_back(
                LayerToolState{layer_number++, current_extruder, layer.print_z()});
            m_max_layer_z = std::max(m_max_layer_z, layer.print_z());
            ++counted_layers;

            for (uint32_t tool_idx = 0;
                 tool_idx < layer.tool_group_count(); ++tool_idx) {
                PrintingToolGroup tool_group = layer.tool_group(tool_idx);
                const extruder_index_t tool = tool_group.extruder_id();
                if (tool == EXTRUDER_ID_UNKNOWN)
                    tool_group.set_extruder_id(current_extruder);
                else
                    current_extruder = tool;
            }
        }
    }
    if (counted_layers != run_count)
        throw std::runtime_error(
            "The tool-change run count does not match the PrintingPlan.");
    m_setup_valid = true;
}

void CreateToolChange::setup_run_impl(
    const plugin_run_context *run_ctx) const
{
    if (!m_setup_valid)
        return;
    const run_ctx_layer_extrusion_edition *ctx =
        plugin_ctx_as_layer_extrusion_edition(run_ctx);
    if (ctx != nullptr && ctx->layer_group != nullptr)
        progress().add_max(1);
}

void CreateToolChange::run_impl(const plugin_run_context *run_ctx) const
{
    if (!m_setup_valid)
        return;
    const run_ctx_layer_extrusion_edition *ctx =
        plugin_ctx_as_layer_extrusion_edition(run_ctx);
    if (ctx == nullptr || ctx->print == nullptr || ctx->layer_group == nullptr ||
        run_ctx->plugin_storage == nullptr)
        return;
    if (ctx->group_idx >= m_layer_states.size() ||
        ctx->layer_group_idx >= m_layer_states[ctx->group_idx].size())
        throw std::runtime_error(
            "A tool-change run references an unknown PrintingLayerGroup.");

    const LayerToolState &state =
        m_layer_states[ctx->group_idx][ctx->layer_group_idx];
    PrintingLayerGroup layer(ctx->layer_group);
    const std::string script = Print(ctx->print).config().string_or_default(
        "toolchange_gcode", std::string());
    extruder_index_t current_extruder = state.incoming_extruder;

    // Each tool-group belongs to this worker's layer. The first global tool and
    // repeated visits need no explicit event; every actual transition is added
    // before entering its target group.
    for (uint32_t tool_idx = 0; tool_idx < layer.tool_group_count(); ++tool_idx) {
        PrintingToolGroup tool_group = layer.tool_group(tool_idx);
        const extruder_index_t target_extruder = tool_group.extruder_id();
        if (target_extruder == EXTRUDER_ID_UNKNOWN)
            throw std::runtime_error(
                "A PrintingToolGroup has no valid extruder for tool-change planning.");
        if (current_extruder != EXTRUDER_ID_UNKNOWN &&
            current_extruder != target_extruder) {
            // Runs own separate tool-groups. This local lock serializes event
            // construction and publication; PluginStorage already synchronizes
            // allocation/free bookkeeping, not edits or use of a freed config.
            const std::lock_guard<std::mutex> storage_lock(m_storage_mutex);
            append_tool_change_event(
                run_ctx->plugin_storage, tool_group, script,
                state.layer_number, state.print_z, m_max_layer_z,
                current_extruder, target_extruder);
        }
        current_extruder = target_extruder;
    }
    progress().increment();
}

} // namespace

void register_create_tool_change_plugin(orchestrator_handle *orchestrator)
{
    orchestrator_register_plugin(
        orchestrator, CreateToolChange::instance(orchestrator).c_instance());
    const plugin_instance member = CreateToolChange::instance(orchestrator).c_instance();
    raw_exclusive_group_def definition = raw_exclusive_group_def_init();
    definition.exclusive_group = member.vt->get_exclusive_group(member.ctx);
    definition.label = "Tool changes";
    definition.tooltip = "Selects how ordered tool visits materialize physical tool selection.";
    CreateToolChange::instance(orchestrator).register_exclusive_group(STEP_LAYER_EXTRUSION_EDIT, definition);
}

}}} // namespace slic3r_api::LayerExtrusionEdit::CreateToolChangePlugin
