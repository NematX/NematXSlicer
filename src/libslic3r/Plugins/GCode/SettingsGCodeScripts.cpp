///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "SettingsGCodeScripts.hpp"

#include <algorithm>
#include <cassert>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>

#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_extrusion_edit.h"
#include "libslic3r/Api/plugin/cpp/ConfigViews.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PrintHelpers.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/Api/plugin/cpp/properties/ExtrusionProperties.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/ExtrusionScopeHelpers.hpp"
#include "libslic3r/Plugins/PrintingPlan/PrintingExtrusionScopeProperty.h"

/*
Configured G-code script placement
==================================

This plugin translates configured start, layer, object, and filament scripts
into ordered events in the PrintingPlan. It runs at STEP_EXTRUSION_EDIT after
the plan hierarchy and tool-change scopes have been created. The final
firmware stage later evaluates these events; this plugin only places them and
stores their static placeholder values.

The normal execution flow is:

    run_impl()
    |-- summarize the plan's layers, Z positions, tools, and objects
    |-- append `start_gcode` before the plan
    |-- for each sequential printing group:
    |   |-- place `between_objects_gcode` before the next object or after the
    |   |   previous object according to its setting
    |   |-- place `before_layer_gcode` at each layer boundary
    |   |-- place `layer_gcode` before each layer except a new object's first
    |   |-- on tool changes, append end-filament and start-filament scripts
    |   |   with the corresponding source and destination extruder
    |   `-- retain the last layer and tool state for the next group
    |-- append final end-filament scripts for the tools that were used
    `-- append `end_gcode` after the plan

Each event is stored in the fixed scope that owns its boundary. Start-filament
scripts are inserted immediately before the semantic Unretract event when an
incoming tool-change scope contains one; otherwise they are placed at the
beginning of that scope, or in the tool group's before-events when no scope is
available. This preserves the required order relative to tool selection and
the restoration of extrusion state.

Each event snapshots only values that its producer fixes. Start, before-layer,
and layer scripts snapshot layer number, Z, and source height; start-filament
scripts snapshot layer number and Z; end-filament and end scripts keep layer
state live in firmware. The separate tool-change producer snapshots layer
number, Z, and maximum layer Z, while firmware supplies the source height.
Other snapshots carry previous layer Z, object IDs, and previous/next or
filament extruder IDs as needed. The plan hierarchy is not resized while scripts
are appended, so borrowed event roots remain valid.

A source layer height is read through the first source extrusion in plan order;
when a layer has no source extrusion, scripts use its Z gap from the preceding
plan layer.
Empty scripts produce no event. A plan with no tool group still receives
plan- and layer-level scripts, but cannot receive a tool-specific start script
until a tool group and its transition scope exist. In single-extruder
multi-material mode, final filament cleanup is limited to the active tool;
otherwise each tool used by the plan receives its own final script.
Synthetic catalog entries label generated geometry but have no model Object;
groups containing only such an entry do not select an object for object scripts.
*/

namespace slic3r_api { namespace GCodeGeneration { namespace SettingsGCodeScriptsPlugin {
namespace {

const char *const k_no_dependencies[] = {nullptr};

const char *const k_defined_config_keys[] = {
    "start_gcode",
    "end_gcode",
    "between_objects_gcode",
    "between_objects_gcode_before_move",
    "start_filament_gcode",
    "end_filament_gcode"
};

enum class ScopeEventPosition : uint8_t { Before, After };

struct LayerContext
{
    int32_t number = -1;
    coord_t print_z = 0;
    // First ordered source-extrusion height, or the finalized group's Z gap.
    coord_t height = 0;
};

struct PlanSummary
{
    coord_t max_print_z = 0;
    std::optional<extruder_index_t> first_extruder;
    std::optional<extruder_index_t> last_extruder;
    std::optional<LayerContext> first_layer;
    std::optional<LayerContext> previous_layer;
    std::optional<LayerContext> last_layer;
};

// Read first-source heights in plan order plus global Z and tool facts; return
// scalar summary data without owning plan data, and propagate cancellation or
// invalid-view failures.
PlanSummary summarize_plan(const ConstPrintingPlan &plan,
                           const plugin_run_context *run_ctx);
// Resolve a single real-object catalog group to its stable placeholder object index.
std::optional<uint32_t> group_object_index(const Print &print,
                                           const ConstPrintingPlan &plan,
                                           const ConstPrintingGroup &group,
                                           const plugin_run_context *run_ctx);
// Register the option definitions owned by this plugin; it runs once from
// inilialize_impl().
void initialize_settings_gcode_scripts_settings(orchestrator_handle *orchestrator);
// Add one typed scalar to the temporary Config transported with a script.
void set_int_argument(MutableConfig &config, const char *key, int32_t value);
void set_float_argument(MutableConfig &config, const char *key, double value);
// Build an owned producer Config from the final global layer position, including actual height and preceding Z; storage must remain valid for the returned handle and Config failures propagate.
StoredConfig layer_arguments(storage_handle *storage,
                             const LayerContext &layer,
                             const std::optional<LayerContext> &previous_layer,
                             coord_t max_print_z);
// Clone a small producer Config before adding values specific to one event.
StoredConfig copy_arguments(storage_handle *storage, const Config &source);
// Build one temporary event and move its contents into the selected fixed scope root.
void append_script_event(storage_handle *storage,
                         PrintingScopeEvents events,
                         const std::string &script,
                         gcode_script_type script_type,
                         const Config &arguments,
                         ScopeEventPosition position,
                         extruder_index_t processing_extruder_id = EXTRUDER_ID_UNKNOWN);
// Search one tree for the first scope whose incoming boundary selects a tool.
MutableExtrusionEntity first_toolchange_scope_in_entity(
    MutableExtrusionEntity entity,
    const PluginPropertyKey<PrintingExtrusionScopeProperty> &scope_key,
    const plugin_run_context *run_ctx);
// Return the first tool-change scope owned by one tool visit.
MutableExtrusionEntity first_toolchange_scope(
    PrintingToolGroup tool_group,
    const PluginPropertyKey<PrintingExtrusionScopeProperty> &scope_key,
    const plugin_run_context *run_ctx);
// Find the semantic Unretract event in one phase subtree.
MutableExtrusionEntity find_unretract(MutableExtrusionEntity entity,
                                      const plugin_run_context *run_ctx);
// Insert start-filament immediately before Unretract, or first in before.
void append_start_filament_script(
    MutableExtrusionEntity scope_root,
    const PluginPropertyKey<PrintingExtrusionScopeProperty> &scope_key,
    const std::string &script,
    const Config &arguments,
    extruder_index_t processing_extruder_id,
    const plugin_run_context *run_ctx);

// Scan borrowed plan groups and layers in print order, recording each layer's
// first source height or Z gap. Return scalar summary data without mutating or
// retaining the plan, and propagate cancellation or invalid-view failures.
PlanSummary summarize_plan(const ConstPrintingPlan &plan,
                           const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    PlanSummary summary;
    int32_t layer_number = 0;
    for (uint32_t group_idx = 0; group_idx < plan.group_count(); ++group_idx) {
        if ((group_idx & 63u) == 0)
            throw_if_cancelled(run_ctx);
        const ConstPrintingGroup group = plan.group(group_idx);
        for (uint32_t layer_idx = 0; layer_idx < group.layer_group_count(); ++layer_idx) {
            if ((layer_idx & 63u) == 0)
                throw_if_cancelled(run_ctx);
            const ConstPrintingLayerGroup layer = group.layer_group(layer_idx);
            summary.max_print_z = std::max(summary.max_print_z, layer.print_z());
            const coord_t previous_print_z = summary.last_layer ? summary.last_layer->print_z : 0;
            const std::optional<coord_t> source_height = first_source_layer_height(layer);
            const LayerContext current_layer{
                layer_number++, layer.print_z(),
                source_height.value_or(layer.print_z() - previous_print_z)};
            if (!summary.first_layer)
                summary.first_layer = current_layer;
            summary.previous_layer = summary.last_layer;
            summary.last_layer = current_layer;
            for (uint32_t tool_idx = 0; tool_idx < layer.tool_group_count(); ++tool_idx) {
                if ((tool_idx & 63u) == 0)
                    throw_if_cancelled(run_ctx);
                const ConstPrintingToolGroup tool_group = layer.tool_group(tool_idx);
                const extruder_index_t extruder_id = tool_group.extruder_id();
                if (extruder_id == EXTRUDER_ID_UNKNOWN)
                    continue;
                if (!summary.first_extruder)
                    summary.first_extruder = extruder_id;
                summary.last_extruder = extruder_id;
            }
        }
    }
    return summary;
}

// Inspect a borrowed single-entry group and map only a real catalog owner to a Print object index; missing/synthetic owners return no index, cancellation throws and no ownership or plan state changes.
std::optional<uint32_t> group_object_index(const Print &print,
                                           const ConstPrintingPlan &plan,
                                           const ConstPrintingGroup &group,
                                           const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    if (group.object_instance_count() != 1)
        return std::nullopt;
    const uint32_t printing_catalog_id = group.printing_catalog_id(0);
    if (printing_catalog_id == UINT32_MAX ||
        printing_catalog_id >= plan.object_instance_count())
        return std::nullopt;
    const PrintingObjectInstance instance = plan.object_instance(printing_catalog_id);
    if (!instance.has_object())
        return std::nullopt;
    const Object object = instance.object();
    for (uint32_t object_idx = 0; object_idx < print.object_count(); ++object_idx) {
        if ((object_idx & 63u) == 0)
            throw_if_cancelled(run_ctx);
        if (print.object(object_idx).handle() == object.handle())
            return object_idx;
    }
    return std::nullopt;
}

void set_int_argument(MutableConfig &config, const char *key, int32_t value)
{
    config.get_or_add(key, SLIC3R_CONFIG_OPTION_INT).set_int(value);
}

void set_float_argument(MutableConfig &config, const char *key, double value)
{
    config.get_or_add(key, SLIC3R_CONFIG_OPTION_FLOAT).set_float(value);
}

StoredConfig layer_arguments(storage_handle *storage,
                             const LayerContext &layer,
                             const std::optional<LayerContext> &previous_layer,
                             coord_t max_print_z)
{
    StoredConfig arguments(storage);
    set_int_argument(arguments, "layer_num", layer.number);
    set_float_argument(arguments, "layer_z", unscaled(layer.print_z));
    set_float_argument(arguments, "previous_layer_z",
                       previous_layer ? unscaled(previous_layer->print_z) : 0.0);
    set_float_argument(arguments, "max_layer_z", unscaled(max_print_z));
    // before_layer_gcode runs before the firmware begins its layer, so its actual height comes from this producer snapshot like layer_num and layer_z.
    set_float_argument(arguments, "layer_current_height", unscaled(layer.height));
    return arguments;
}

StoredConfig copy_arguments(storage_handle *storage, const Config &source)
{
    StoredConfig copy(storage);
    copy.deserialize_all(source.serialize_all());
    return copy;
}

void append_script_event(storage_handle *storage,
                         PrintingScopeEvents events,
                         const std::string &script,
                         gcode_script_type script_type,
                         const Config &arguments,
                         ScopeEventPosition position,
                         extruder_index_t processing_extruder_id)
{
    if (script.empty())
        return;

    // The fixed scope root takes ownership of the complete temporary event,
    // including its text and typed argument buffers.
    StoredExtrusionEntity event(storage);
    event.script_gcode(script, script_type, arguments, processing_extruder_id);
    if (position == ScopeEventPosition::Before)
        events.append_before_move(event.mutable_view());
    else
        events.append_after_move(event.mutable_view());
}

MutableExtrusionEntity first_toolchange_scope_in_entity(
    MutableExtrusionEntity entity,
    const PluginPropertyKey<PrintingExtrusionScopeProperty> &scope_key,
    const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    if (LayerExtrusionEdit::ExtrusionScope::is_scope(
            entity.readonly(), scope_key)) {
        const PrintingExtrusionScopeProperty *property = entity.get(scope_key);
        if (property != nullptr && printing_extrusion_scope_has_flag(
                *property, PRINTING_EXTRUSION_SCOPE_INCOMING_TOOLCHANGE))
            return entity;
        // Compact scopes never nest, so their large content subtree cannot
        // contain another matching tool-change scope.
        return MutableExtrusionEntity();
    }
    for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx) {
        if ((child_idx & 63u) == 0)
            throw_if_cancelled(run_ctx);
        MutableExtrusionEntity found = first_toolchange_scope_in_entity(
            entity.child_mutable(child_idx), scope_key, run_ctx);
        if (found.valid())
            return found;
    }
    return MutableExtrusionEntity();
}

MutableExtrusionEntity first_toolchange_scope(
    PrintingToolGroup tool_group,
    const PluginPropertyKey<PrintingExtrusionScopeProperty> &scope_key,
    const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    for (uint32_t extrusion_idx = 0;
         extrusion_idx < tool_group.extrusion_count(); ++extrusion_idx) {
        if ((extrusion_idx & 63u) == 0)
            throw_if_cancelled(run_ctx);
        MutableExtrusionEntity found = first_toolchange_scope_in_entity(
            tool_group.extrusion(extrusion_idx).mutable_root(), scope_key, run_ctx);
        if (found.valid())
            return found;
    }
    return MutableExtrusionEntity();
}

MutableExtrusionEntity find_unretract(MutableExtrusionEntity entity,
                                      const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    const EPropertyAttributes *attributes = entity.get(EPropertyAttributes::key);
    if (attributes != nullptr &&
        RAW_EXTRUSION_ROLE_IS_UNRETRACT(attributes->extrusion_role()))
        return entity;
    for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx) {
        if ((child_idx & 63u) == 0)
            throw_if_cancelled(run_ctx);
        MutableExtrusionEntity found = find_unretract(
            entity.child_mutable(child_idx), run_ctx);
        if (found.valid())
            return found;
    }
    return MutableExtrusionEntity();
}

void append_start_filament_script(
    MutableExtrusionEntity scope_root,
    const PluginPropertyKey<PrintingExtrusionScopeProperty> &scope_key,
    const std::string &script,
    const Config &arguments,
    const extruder_index_t processing_extruder_id,
    const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    if (script.empty())
        return;
    const LayerExtrusionEdit::ExtrusionScope::OrderedExtrusionScope scope(
        scope_root, scope_key);
    MutableExtrusionEntity before = scope.before();
    if (!before.valid())
        throw std::runtime_error(
            "A start-filament script needs an incoming transition phase.");

    // When retraction exists, anchor directly on its Unretract event so the
    // script remains the last operation before restoring E. Otherwise place
    // it at the beginning of the target's before phase; tool selection lives
    // outside the scope in PrintingToolGroup::events().before.
    MutableExtrusionEntity unretract = find_unretract(before, run_ctx);
    MutableExtrusionEntity event;
    if (unretract.valid()) {
        event = unretract.emplace_ordered_leaf(
            OrderedLeafPosition::Before,
            ExistingPropertyPlacement::MoveWithExistingContent);
    } else if (before.segment_count() == 0 && before.child_count() == 0 &&
               before.property_count() == 0) {
        event = before;
    } else {
        event = before.emplace_ordered_leaf(
            OrderedLeafPosition::Before,
            ExistingPropertyPlacement::MoveWithExistingContent);
    }
    if (!event.valid())
        throw std::runtime_error(
            "Unable to insert start-filament into an ordered tool change.");
    event.script_gcode(
        script, GCODE_SCRIPT_TYPE_START_FILAMENT_GCODE,
        arguments, processing_extruder_id);
}

void initialize_settings_gcode_scripts_settings(orchestrator_handle *orchestrator)
{
    /*
    Option definitions owned by this module. Each entry fixes the option type,
    override scope, default and GUI metadata; defined keys are published in the
    print presets, so profiles read and write them by name like any built-in
    setting. Called once from inilialize_impl().
    */
    raw_config_option_def def = raw_config_option_def_init();
    def.opt_key = "start_gcode";
    def.type = RAW_CO_STRING;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINTER;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_CUSTOMGCODE;
    def.label = "Start G-code";
    def.tooltip = "This start procedure is inserted at the beginning, possibly prepended by "
                  "temperature-changing commands and others. See 'autoemit_temperature_commands' and 'start_gcode_manual'. "
                  "A negative E move counts as a retraction only with '; retraction' on the same line, "
                  "and a positive E move restores it only with '; unretraction'; other E moves do not "
                  "change the retraction state.";
    def.multiline = 1;
    def.full_width = 1;
    def.height = 12;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_PRUSA;
    def.default_serialized_value = "G28 ; home all axes\\nG1 Z5 F5000 ; lift nozzle\\n";
    orchestrator_create_option_def(orchestrator, &def);

    def = raw_config_option_def_init();
    def.opt_key = "end_gcode";
    def.type = RAW_CO_STRING;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINTER;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_CUSTOMGCODE;
    def.label = "End G-code";
    def.tooltip = "This end procedure is inserted at the end of the output file. "
                  "Note that you can use placeholder variables for all Slic3r settings. "
                  "A negative E move counts as a retraction only with '; retraction' on the same line, "
                  "and a positive E move restores it only with '; unretraction'; other E moves do not "
                  "change the retraction state.";
    def.multiline = 1;
    def.full_width = 1;
    def.height = 12;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_PRUSA;
    def.default_serialized_value = "M104 S0 ; turn off temperature\\nG28 X0  ; home X axis\\nM84     ; disable motors\\n";
    orchestrator_create_option_def(orchestrator, &def);

    def = raw_config_option_def_init();
    def.opt_key = "between_objects_gcode";
    def.type = RAW_CO_STRING;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINTER;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_CUSTOMGCODE;
    def.label = "Between objects G-code";
    def.tooltip = "This code is inserted between objects when using sequential printing. By default extruder and bed temperature are reset using non-wait command; however if M104, M109, M140 or M190 are detected in this custom code, Slic3r will not add temperature commands. Note that you can use placeholder variables for all Slic3r settings, so you can put a \"M109 S{first_layer_temperature}\" command wherever you want. A negative E move counts as a retraction only with '; retraction' on the same line, and a positive E move restores it only with '; unretraction'; other E moves do not change the retraction state.";
    def.multiline = 1;
    def.full_width = 1;
    def.height = 12;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_PRUSA;
    def.default_serialized_value = "";
    orchestrator_create_option_def(orchestrator, &def);

    def = raw_config_option_def_init();
    def.opt_key = "between_objects_gcode_before_move";
    def.type = RAW_CO_BOOL;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINTER;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_CUSTOMGCODE;
    def.label = "Put gcode before the move";
    def.tooltip = "'This'between_objects_gcode' code is inserted before moving to the next object, instead of after moving to the next object.";
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
    def.default_serialized_value = "0";
    orchestrator_create_option_def(orchestrator, &def);

    def = raw_config_option_def_init();
    def.opt_key = "start_filament_gcode";
    def.type = RAW_CO_VECTOR_STRING;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_FILAMENT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_CUSTOMGCODE;
    def.label = "Start G-code";
    def.full_label = "Filament start G-code";
    def.tooltip = "This start procedure is inserted at the beginning, after any printer start gcode (and "
                  "after any toolchange to this filament in case of multi-material printers). "
                  "This is used to override settings for a specific filament. If Slic3r detects "
                  "M104, M109, M140 or M190 in your custom codes, such commands will "
                  "not be prepended automatically so you're free to customize the order "
                  "of heating commands and other custom actions. Note that you can use placeholder variables "
                  "for all Slic3r settings, so you can put a \"M109 S{first_layer_temperature}\" command "
                  "wherever you want. If you have multiple extruders, the gcode is processed "
                  "in extruder order. A negative E move counts as a retraction only with "
                  "'; retraction' on the same line, and a positive E move restores it only with "
                  "'; unretraction'; other E moves do not change the retraction state.";
    def.multiline = 1;
    def.full_width = 1;
    def.height = 12;
    def.is_vector_extruder = 1;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_PRUSA;
    def.default_serialized_value = "\"; Filament gcode\\n\"";
    orchestrator_create_option_def(orchestrator, &def);

    def = raw_config_option_def_init();
    def.opt_key = "end_filament_gcode";
    def.type = RAW_CO_VECTOR_STRING;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_FILAMENT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_CUSTOMGCODE;
    def.label = "End G-code";
    def.full_label = "Filament end G-code";
    def.tooltip = "This end procedure is inserted at the end of the output file, before the printer end gcode (and "
                  "before any toolchange from this filament in case of multimaterial printers). "
                  "Note that you can use placeholder variables for all Slic3r settings. "
                  "If you have multiple extruders, the gcode is processed in extruder order. "
                  "A negative E move counts as a retraction only with '; retraction' on the same line, "
                  "and a positive E move restores it only with '; unretraction'; other E moves do not "
                  "change the retraction state.";
    def.multiline = 1;
    def.full_width = 1;
    def.height = 120;
    def.is_vector_extruder = 1;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_PRUSA;
    def.default_serialized_value = "\"; Filament-specific end gcode \\n;END gcode for filament\\n\"";
    orchestrator_create_option_def(orchestrator, &def);
}

class SettingsGCodeScripts final : public PluginBase
{
public:
    static SettingsGCodeScripts &instance(orchestrator_handle *orchestrator) {
        static SettingsGCodeScripts plugin(orchestrator);
        return plugin;
    }

    explicit SettingsGCodeScripts(orchestrator_handle *orchestrator) :
        PluginBase(orchestrator),
        m_scope_property(printing_extrusion_scope_property_key(orchestrator))
    {}

private:
    const char *id_impl() const noexcept override { return "gcode.settings_scripts"; }
    const char *name_impl() const noexcept override { return "Configured G-code scripts"; }
    const char *description_impl() const noexcept override {
        return "Places configured print, layer, tool and object scripts in the ordered printing plan.";
    }
    slicing_step_t step_impl() const noexcept override { return STEP_EXTRUSION_EDIT; }
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }
    int32_t priority_impl() const noexcept override { return 0; }
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override {
        if (keys == nullptr)
            return 10;
        keys[0] = raw_used_config_key{"start_gcode", RAW_CO_STRING, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE};
        keys[1] = raw_used_config_key{"end_gcode", RAW_CO_STRING, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE};
        keys[2] = raw_used_config_key{"before_layer_gcode", RAW_CO_STRING, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE};
        keys[3] = raw_used_config_key{"layer_gcode", RAW_CO_STRING, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE};
        keys[4] = raw_used_config_key{"between_objects_gcode", RAW_CO_STRING, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE};
        keys[5] = raw_used_config_key{"between_objects_gcode_before_move", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE};
        keys[6] = raw_used_config_key{"start_filament_gcode", RAW_CO_VECTOR_STRING, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE};
        keys[7] = raw_used_config_key{"end_filament_gcode", RAW_CO_VECTOR_STRING, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE};
        keys[8] = raw_used_config_key{"single_extruder_multi_material", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE};
        keys[9] = raw_used_config_key{"single_extruder_multi_material_priming", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE};
        return 10;
    }
    int32_t defined_config_keys(const char **keys) const noexcept override {
        if (keys != nullptr)
            for (size_t idx = 0; idx < std::size(k_defined_config_keys); ++idx)
                keys[idx] = k_defined_config_keys[idx];
        return int32_t(std::size(k_defined_config_keys));
    }
    void inilialize_impl(storage_handle *) const override {
        initialize_settings_gcode_scripts_settings(m_orchestrator);
    }
    // Read print, plan and storage from the run context and append owned events
    // to fixed scopes. Incomplete contexts leave the plan unchanged; cancellation
    // or storage/plan failures propagate without rolling back earlier appends,
    // and borrowed views are not retained.
    void run_impl(const plugin_run_context *run_context) const override {
        throw_if_cancelled(run_context);
        const run_ctx_extrusion_edition *context = plugin_ctx_as_extrusion_edition(run_context);
        assert(context != nullptr && context->print != nullptr && context->plan != nullptr);
        if (context == nullptr || context->print == nullptr || context->plan == nullptr ||
            run_context->plugin_storage == nullptr)
            return;

        const Print print(context->print);
        const Config config = print.config();
        PrintingPlan plan(context->plan);
        const ConstPrintingPlan read_only_plan(plan);
        PrintingScopeEvents plan_events = plan.events();
        const PlanSummary summary = summarize_plan(read_only_plan, run_context);

        const LayerContext first_layer = summary.first_layer.value_or(LayerContext{});
        StoredConfig start_arguments = layer_arguments(
            run_context->plugin_storage, first_layer, std::nullopt, summary.max_print_z);
        set_int_argument(start_arguments, "previous_extruder", -1);
        set_int_argument(start_arguments, "next_extruder",
                         summary.first_extruder ? int32_t(*summary.first_extruder) : -1);
        set_int_argument(start_arguments, "filament_extruder_id",
                         summary.first_extruder ? int32_t(*summary.first_extruder) : 0);
        append_script_event(run_context->plugin_storage, plan_events,
                            config.string_or_default("start_gcode", std::string()),
                            GCODE_SCRIPT_TYPE_START_GCODE, start_arguments,
                            ScopeEventPosition::Before,
                            summary.first_extruder.value_or(EXTRUDER_ID_UNKNOWN));

        const std::string before_layer = config.string_or_default("before_layer_gcode", std::string());
        const std::string after_layer = config.string_or_default("layer_gcode", std::string());
        const std::string between_objects = config.string_or_default("between_objects_gcode", std::string());
        const bool between_before_move = config.bool_or_default("between_objects_gcode_before_move", false);
        const bool single_extruder_multi_material =
            config.bool_or_default("single_extruder_multi_material", false);
        extruder_index_t current_tool = EXTRUDER_ID_UNKNOWN;
        int32_t layer_number = 0;
        bool seen_layer = false;
        std::set<extruder_index_t> used_tools;
        std::optional<uint32_t> previous_object;
        std::optional<LayerContext> previous_layer;
        std::optional<PrintingScopeEvents> previous_object_events;
        std::optional<PrintingScopeEvents> previous_tool_events;

        // The plan hierarchy is not resized during this pass. Borrowed event
        // roots therefore remain valid while scripts are appended to them.
        for (uint32_t group_idx = 0; group_idx < plan.group_count(); ++group_idx) {
            if ((group_idx & 63u) == 0)
                throw_if_cancelled(run_context);
            PrintingGroup group = plan.group(group_idx);
            PrintingScopeEvents group_events = group.events();
            const std::optional<uint32_t> current_object =
                group_object_index(print, read_only_plan, ConstPrintingGroup(group), run_context);
            if (current_object && previous_object && !between_objects.empty()) {
                StoredConfig arguments(run_context->plugin_storage);
                const LayerContext object_layer = previous_layer.value_or(LayerContext{});
                set_int_argument(arguments, "layer_num", object_layer.number);
                set_float_argument(arguments, "layer_z", unscaled(object_layer.print_z));
                set_int_argument(arguments, "previous_object_id", int32_t(*previous_object));
                set_int_argument(arguments, "next_object_id", int32_t(*current_object));
                if (between_before_move) {
                    assert(previous_object_events.has_value());
                    append_script_event(run_context->plugin_storage, *previous_object_events,
                                        between_objects, GCODE_SCRIPT_TYPE_BETWEEN_OBJECTS_GCODE,
                                        arguments, ScopeEventPosition::After);
                } else {
                    append_script_event(run_context->plugin_storage, group_events,
                                        between_objects, GCODE_SCRIPT_TYPE_BETWEEN_OBJECTS_GCODE,
                                        arguments, ScopeEventPosition::Before);
                }
            }
            if (current_object) {
                previous_object = current_object;
                previous_object_events = group_events;
            }

            std::optional<PrintingScopeEvents> previous_layer_events;
            for (uint32_t layer_idx = 0; layer_idx < group.layer_group_count(); ++layer_idx) {
                if ((layer_idx & 63u) == 0)
                    throw_if_cancelled(run_context);
                PrintingLayerGroup layer = group.layer_group(layer_idx);
                PrintingScopeEvents layer_events = layer.events();
                const coord_t previous_print_z = previous_layer ? previous_layer->print_z : 0;
                const std::optional<coord_t> source_height = first_source_layer_height(
                    ConstPrintingLayerGroup(layer));
                const LayerContext current_layer{
                    layer_number++, layer.print_z(),
                    source_height.value_or(layer.print_z() - previous_print_z)};
                const StoredConfig current_layer_arguments = layer_arguments(
                    run_context->plugin_storage, current_layer, previous_layer, summary.max_print_z);

                PrintingScopeEvents before_layer_events = previous_layer_events ?
                    *previous_layer_events : group_events;
                const ScopeEventPosition before_layer_position = previous_layer_events ?
                    ScopeEventPosition::After : ScopeEventPosition::Before;
                append_script_event(run_context->plugin_storage, before_layer_events, before_layer,
                                    GCODE_SCRIPT_TYPE_BEFORE_LAYER_GCODE, current_layer_arguments,
                                    before_layer_position);
                const bool first_layer_of_sequential_object = current_object && layer_idx == 0;
                if (seen_layer && !first_layer_of_sequential_object)
                    append_script_event(run_context->plugin_storage, layer_events, after_layer,
                                        GCODE_SCRIPT_TYPE_LAYER_GCODE, current_layer_arguments,
                                        ScopeEventPosition::Before);
                seen_layer = true;

                for (uint32_t tool_idx = 0; tool_idx < layer.tool_group_count(); ++tool_idx) {
                    if ((tool_idx & 63u) == 0)
                        throw_if_cancelled(run_context);
                    PrintingToolGroup tool_group = layer.tool_group(tool_idx);
                    PrintingScopeEvents tool_events = tool_group.events();
                    const extruder_index_t target_tool = tool_group.extruder_id();
                    used_tools.insert(target_tool);
                    if (current_tool == target_tool) {
                        previous_tool_events = tool_events;
                        continue;
                    }

                    StoredConfig tool_arguments(run_context->plugin_storage);
                    set_float_argument(tool_arguments, "previous_layer_z",
                                       previous_layer ? unscaled(previous_layer->print_z) : 0.0);
                    set_float_argument(tool_arguments, "max_layer_z",
                                       unscaled(summary.max_print_z));
                    set_int_argument(tool_arguments, "previous_extruder", current_tool);
                    set_int_argument(tool_arguments, "next_extruder", int32_t(target_tool));
                    set_float_argument(tool_arguments, "toolchange_z", unscaled(current_layer.print_z));
                    if (current_tool != EXTRUDER_ID_UNKNOWN) {
                        assert(previous_tool_events.has_value());
                        StoredConfig end_filament_arguments = copy_arguments(
                            run_context->plugin_storage, tool_arguments);
                        set_int_argument(end_filament_arguments, "filament_extruder_id", current_tool);
                        append_script_event(
                            run_context->plugin_storage, *previous_tool_events,
                            config.vector_string_or_default("end_filament_gcode", uint32_t(current_tool), std::string()),
                            GCODE_SCRIPT_TYPE_END_FILAMENT_GCODE, end_filament_arguments,
                            ScopeEventPosition::After, current_tool);
                    }
                    const MutableExtrusionEntity transition_scope = current_tool != EXTRUDER_ID_UNKNOWN ?
                        first_toolchange_scope(tool_group, m_scope_property, run_context) :
                        MutableExtrusionEntity();
                    StoredConfig start_filament_arguments = copy_arguments(
                        run_context->plugin_storage, tool_arguments);
                    set_int_argument(start_filament_arguments, "layer_num", current_layer.number);
                    set_float_argument(start_filament_arguments, "layer_z",
                                       unscaled(current_layer.print_z));
                    set_int_argument(start_filament_arguments, "filament_extruder_id", int32_t(target_tool));
                    const std::string start_filament =
                        config.vector_string_or_default(
                            "start_filament_gcode", target_tool, std::string());
                    if (transition_scope.valid())
                        append_start_filament_script(
                            transition_scope, m_scope_property, start_filament,
                            start_filament_arguments,
                            target_tool,
                            run_context);
                    else
                        append_script_event(
                            run_context->plugin_storage, tool_events,
                            start_filament,
                            GCODE_SCRIPT_TYPE_START_FILAMENT_GCODE,
                            start_filament_arguments,
                            ScopeEventPosition::Before, target_tool);
                    current_tool = target_tool;
                    previous_tool_events = tool_events;
                }
                previous_layer = current_layer;
                previous_layer_events = layer_events;
            }
        }

        if (current_tool != EXTRUDER_ID_UNKNOWN) {
            const std::set<extruder_index_t> final_tools = single_extruder_multi_material ?
                std::set<extruder_index_t>{extruder_index_t(current_tool)} : used_tools;
            for (extruder_index_t tool_id : final_tools) {
                throw_if_cancelled(run_context);
                StoredConfig arguments(run_context->plugin_storage);
                set_float_argument(arguments, "previous_layer_z",
                                   summary.previous_layer ?
                                       unscaled(summary.previous_layer->print_z) : 0.0);
                set_float_argument(arguments, "max_layer_z", unscaled(summary.max_print_z));
                set_float_argument(arguments, "toolchange_z",
                                   summary.last_layer ?
                                       unscaled(summary.last_layer->print_z) : 0.0);
                set_int_argument(arguments, "filament_extruder_id", int32_t(tool_id));
                set_int_argument(arguments, "previous_extruder", current_tool);
                set_int_argument(arguments, "next_extruder", -1);
                append_script_event(
                    run_context->plugin_storage, plan_events,
                    config.vector_string_or_default("end_filament_gcode", tool_id, std::string()),
                    GCODE_SCRIPT_TYPE_END_FILAMENT_GCODE, arguments,
                    ScopeEventPosition::After, tool_id);
            }
        }

        StoredConfig end_arguments(run_context->plugin_storage);
        set_float_argument(end_arguments, "previous_layer_z",
                           summary.previous_layer ?
                               unscaled(summary.previous_layer->print_z) : 0.0);
        set_float_argument(end_arguments, "max_layer_z", unscaled(summary.max_print_z));
        set_int_argument(end_arguments, "previous_extruder",
                         summary.last_extruder ? int32_t(*summary.last_extruder) : -1);
        set_int_argument(end_arguments, "next_extruder", -1);
        set_int_argument(end_arguments, "filament_extruder_id",
                         summary.last_extruder ? int32_t(*summary.last_extruder) : 0);
        append_script_event(run_context->plugin_storage, plan_events,
                            config.string_or_default("end_gcode", std::string()),
                            GCODE_SCRIPT_TYPE_END_GCODE, end_arguments,
                            ScopeEventPosition::After,
                            summary.last_extruder.value_or(EXTRUDER_ID_UNKNOWN));
    }

    PluginPropertyKey<PrintingExtrusionScopeProperty> m_scope_property;
};

} // namespace

void register_settings_gcode_scripts_plugin(orchestrator_handle *orchestrator)
{
    orchestrator_register_plugin(orchestrator, SettingsGCodeScripts::instance(orchestrator).c_instance());
}

}}} // namespace slic3r_api::GCodeGeneration::SettingsGCodeScriptsPlugin
