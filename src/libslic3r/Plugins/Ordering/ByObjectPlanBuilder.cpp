///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "DefaultOrdering.hpp"
#include "IslandStacks.hpp"
#include "PlanBuilderCommon.hpp"

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <vector>

#include "libslic3r/Api/plugin/c/steps/slic3r_step_ordering.h"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Api/plugin/cpp/GuiRuleHelpers.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/Plugins/GCode/CustomGCodePerPrintZRecord.hpp"

/*
Provides the By object plan builder to the ordering chain. Borrowed print layers
are cloned through PlanBuilderCommon into owned groups with physical instance ids.
Height G-code rows partition all work into half-open Z bands, including the shared
prefix, global auxiliary layers and by-layer tail. Only boundary heights are
deduplicated. MultiAsSingle means a multi-tool print treated as one logical tool.
With several configured tools, at most one known object-source tool and that
effective record mode (explicit or inferred from rows),
geometric queries above a tool change include the selected tool's clearance
profile without changing the source tools. Other modes retain the barriers only.
All modes use translated plate-wide units and the same stack process. Island mode
uses one island per unit; otherwise all islands of one copy at a Z form one unit,
including its selected auxiliary layers. The height lead is capped by the printer
clearance height, or uses that height when the step is zero. An optional shared prefix
contains every copy's object and auxiliary layers up to the highest index-zero object layer.
That prefix remains unconditional even above a height limit. Other layers at or above
the limit share a tail in increasing Z using the By layer construction. Each band
emits its prefix intersection, global work and stacks, then its tail intersection.
Support and instance-owned auxiliaries join the clearance search; lower links
remain within their instance. Whole-copy units link to the previous available Z.
Each stack becomes one group, with several instances when close copies join.
Global auxiliary geometry inside the shared first-layer prefix joins its band's
shared group in increasing Z, even above the height limit. Outside that prefix,
global geometry below the limit precedes the same band's stacks, and geometry at
or above the limit joins its tail group. Local auxiliary layers
marked with an owner are included only for that instance; unmarked local layers
are reproduced for every copy. Sources remain unchanged.
The provider owns object order, height step, height limit, island and first-layer settings.
Their activation rules keep them editable only for the selected By object mode.
XY orders compare instance positions on the plate, keeping source order on ties.

    register_by_object_plan_builder_plugin() -> register the alternative provider
        ByObjectPlanBuilder::inilialize_impl() -> define settings and activation rules
        ByObjectPlanBuilder::run_impl() -> replace the plan
            build_plan_by_object() -> order instances and visit event bands
                append_shared_band() -> clone prefix and tail intersections by Z
                    append_by_layer_object_layers() -> merge selected instance layers
                    append_print_auxiliary_layers_to_group() -> merge global layers
                append_print_auxiliary_group() -> retain the band's global auxiliaries
                append_island_groups() -> emit plate-wide stacks with instance filters
                    IslandStacks::separate_islands() -> group and follow source islands
*/
namespace slic3r_api { namespace Ordering { namespace DefaultOrderingPlugin {

// Register the permanent borrowed provider; rejected registration or allocation failures propagate.
void register_by_object_plan_builder_plugin(orchestrator_handle *orch);

namespace {
using namespace PlanBuilderCommon;
const char *k_no_dependencies[] = { nullptr };
const char *k_defined_config_keys[] = {
    "complete_objects_sort", "complete_objects_step", "complete_objects_step_max_z", "complete_islands",
    "complete_objects_one_first_layer"
};

// Publish copied option definitions and activation rules to the borrowed host; registration failures follow the host API.
void initialize_by_object_settings(orchestrator_handle *orch);

// Copy only heights, event kinds, mode and tool numbers needed by band geometry.
// Ignore unrelated table columns and clamp ToolChange numbers to the current printer;
// stale tool numbers therefore cannot reject the builder. Required-column API and
// allocation failures propagate. The print retains ownership of the record.
GCodeGeneration::CustomGCodePerPrintZPlugin::CustomGCodeTable read_barrier_events(const Print &print);

// Replace the plan with plate-wide stacks, optionally sharing all copy layers up
// to the highest first object layer. Height events split all parts into bands;
// a positive height limit merges the remainder by Z. Invalid records, auxiliary owners, cancellation
// and allocation failures propagate; the borrowed print is unchanged.
void build_plan_by_object(const Print &print, PrintingPlan &plan, const plugin_run_context *run_ctx);

// Clone selected object and global auxiliary layers in one half-open Z band.
// Scan before appending so an empty band adds no group. Supplied instances retain
// the configured copy order for shared prefixes; tails use model order. Sources remain borrowed;
// cancellation, invalid owners and allocation failures propagate.
void append_shared_band(const Print &print, PrintingPlan &plan, coord_t lo, coord_t hi,
    const plugin_run_context *run_ctx, const std::vector<SourceInstance> *instances = nullptr);

// Build plate-wide stacks from selected layers in a half-open Z band,
// including each copy's owned auxiliaries, then clone their selected islands into
// one group per stack. Clearance profiles are copied
// with the printer radius and height as their final point. Source views remain borrowed;
// cancellation, invalid owners and allocation failures propagate.
void append_island_groups(const Print &print, PrintingPlan &plan,
    const std::vector<SourceInstance> &instances, coord_t height_step_range,
    coord_t lo, coord_t hi, extruder_index_t clearance_tool, const plugin_run_context *run_ctx);

void build_plan_by_object(const Print &print, PrintingPlan &plan, const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    using namespace GCodeGeneration::CustomGCodePerPrintZPlugin;
    const CustomGCodeTable events = read_barrier_events(print);
    const uint32_t extruder_count = print.config().get("nozzle_diameter").size();
    std::set<extruder_index_t> object_tools;
    for (uint32_t idx = 0; idx < print.object_count(); ++idx) {
        const Object object = print.object(idx);
        for (uint32_t layer_idx = 0; layer_idx < object.layer_count(); ++layer_idx) {
            const Layer layer = object.layer(layer_idx);
            for (uint32_t island_idx = 0; island_idx < layer.island_count(); ++island_idx) {
                const LayerIsland island = layer.island(island_idx);
                for (uint32_t region_idx = 0; region_idx < island.region_island_count(); ++region_idx)
                    if (island.region_island(region_idx).region_count() > 0 && island.region_island(region_idx).has_extrusions() &&
                        island.region_island(region_idx).extruder_id() != EXTRUDER_ID_UNKNOWN)
                        object_tools.insert(island.region_island(region_idx).extruder_id());
            }
        }
    }
    // Keep this mode predicate aligned with printing_plan_mode() in
    // CustomGCodePerPrintZRecord.cpp: only known object tools distinguish MultiExtruder.
    const bool applies_tool_changes = extruder_count > 1 && object_tools.size() <= 1 &&
        effective_record_mode(events) == Slic3r::CustomGCode::MultiAsSingle;
    /* Do not clear a usable plan after cancellation has already been requested. */
    plan.clear();
    throw_if_cancelled(run_ctx);
    const std::vector<SourceInstance> instances =
        ordered_source_instances_for_object_plan(print, run_ctx);

    const double object_step = print.config().float_or_default("complete_objects_step", 0.);
    const double clearance_height = print.config().float_or_default("extruder_clearance_height", 0.);
    const coord_t height_step_range = scale_to_layer_coord(object_step > 0. ?
        std::min(object_step, clearance_height) : clearance_height);
    const bool first_layer_group = print.config().bool_or_default("complete_objects_one_first_layer", false);
    coord_t first_group_z = std::numeric_limits<coord_t>::lowest();
    if (first_layer_group)
        for (const SourceInstance &source : instances)
            if (source.object.layer_count() != 0)
                first_group_z = std::max(first_group_z, source.object.layer(0).print_z());
    const double max_z = print.config().float_or_default("complete_objects_step_max_z", 0.);
    const coord_t object_step_max_z = max_z > 0. ? scale_to_layer_coord(max_z) : 0;
    const coord_t minimum = std::numeric_limits<coord_t>::lowest();
    const coord_t maximum = std::numeric_limits<coord_t>::max();
    const coord_t prefix_end = first_layer_group ? first_group_z + 1 : minimum;
    const coord_t stack_end = object_step_max_z > 0 ? object_step_max_z : maximum;
    std::vector<coord_t> boundaries{minimum, maximum};
    for (const CustomGCodeRow &row : events.rows)
        boundaries.push_back(scale_to_layer_coord(row.print_z_mm));
    std::sort(boundaries.begin(), boundaries.end());
    boundaries.erase(std::unique(boundaries.begin(), boundaries.end()), boundaries.end());
    size_t next_row = 0;
    extruder_index_t clearance_tool = EXTRUDER_ID_UNKNOWN;
    for (size_t band = 1; band < boundaries.size(); ++band) {
        throw_if_cancelled(run_ctx);
        const coord_t lo = boundaries[band - 1], hi = boundaries[band];
        // Events remain distinct in the record; only their geometric boundaries
        // are deduplicated. The last same-Z ToolChange supplies the extra profile.
        while (next_row < events.rows.size() && scale_to_layer_coord(events.rows[next_row].print_z_mm) <= lo) {
            const CustomGCodeRow &row = events.rows[next_row++];
            if (applies_tool_changes && row.type == Slic3r::CustomGCode::ToolChange)
                clearance_tool = extruder_index_t(row.extruder - 1);
        }
        if (first_layer_group)
            append_shared_band(print, plan, lo, std::min(hi, prefix_end), run_ctx, &instances);
        const coord_t stack_lo = std::max(lo, prefix_end), stack_hi = std::min(hi, stack_end);
        if (stack_lo < stack_hi) {
            LayerExtrusionFilter filter;
            filter.print_z_interval = std::make_pair(stack_lo, stack_hi);
            append_print_auxiliary_group(print, plan, run_ctx, filter);
            append_island_groups(print, plan, instances, height_step_range, stack_lo, stack_hi, clearance_tool, run_ctx);
        }
        if (object_step_max_z > 0)
            append_shared_band(print, plan, std::max({lo, prefix_end, object_step_max_z}), hi, run_ctx);
    }
}

void append_shared_band(const Print &print, PrintingPlan &plan, coord_t lo, coord_t hi,
    const plugin_run_context *run_ctx, const std::vector<SourceInstance> *instances)
{
    if (lo >= hi)
        return;
    bool has_layers = false;
    for (uint32_t idx = 0; idx <= print.object_count(); ++idx) {
        const Object object = idx == print.object_count() ? print.auxiliary_object() : print.object(idx);
        for (uint32_t kind = 0; kind < 2; ++kind) {
            const uint32_t count = kind == 0 ? object.layer_count() : object.auxiliary_layer_count();
            for (uint32_t layer_idx = 0; layer_idx < count; ++layer_idx) {
                const Layer layer = kind == 0 ? object.layer(layer_idx) : object.auxiliary_layer(layer_idx);
                has_layers |= layer.print_z() >= lo && layer.print_z() < hi;
            }
        }
    }
    if (!has_layers)
        return;
    PrintingGroup group = plan.append_group();
    if (!group.valid())
        return;
    std::map<coord_t, uint32_t> indices;
    LayerExtrusionFilter filter;
    filter.print_z_interval = std::make_pair(lo, hi);
    if (instances) {
        for (const SourceInstance &source : *instances) {
            throw_if_cancelled(run_ctx);
            uint32_t catalog_id = UINT32_MAX;
            bool added_instance = false;
            for (uint32_t kind = 0; kind < 2; ++kind) {
                const uint32_t count = kind == 0 ? source.object.layer_count() : source.object.auxiliary_layer_count();
                for (uint32_t idx = 0; idx < count; ++idx) {
                    const Layer layer = kind == 0 ? source.object.layer(idx) : source.object.auxiliary_layer(idx);
                    if (layer.print_z() < lo || layer.print_z() >= hi)
                        continue;
                    const std::optional<uint32_t> owner = kind == 0 ? std::nullopt : layer_instance_owner(layer);
                    if (owner && *owner >= source.object.instance_count())
                        throw std::out_of_range("Auxiliary layer owner instance index is outside the Object.");
                    if (owner && *owner != source.object_instance_idx)
                        continue;
                    if (!added_instance) {
                        catalog_id = plan.find_or_add_object_instance(source.object, source.object_instance_idx);
                        group.append_printing_catalog_id(plan, catalog_id);
                        added_instance = true;
                    }
                    PrintingLayerGroup layer_group = layer_group_for_print_z(group, indices, layer.print_z(), run_ctx);
                    append_layer_once(layer_group, layer, run_ctx);
                    append_layer_instance_extrusions(plan, layer_group, layer, source.shift, catalog_id, run_ctx);
                }
            }
        }
    } else {
        append_by_layer_object_layers(print, plan, group, indices, run_ctx, filter, false);
    }
    append_print_auxiliary_layers_to_group(print, plan, group, indices, run_ctx, filter);
    sort_layer_groups_by_print_z(group, run_ctx);
}

GCodeGeneration::CustomGCodePerPrintZPlugin::CustomGCodeTable read_barrier_events(const Print &print)
{
    using namespace GCodeGeneration::CustomGCodePerPrintZPlugin;
    CustomGCodeTable events;
    const std::optional<Config> record = print.records().find(Slic3r::CustomGCode::PrintRecordChannel);
    if (!record)
        return events;
    events.mode = static_cast<Slic3r::CustomGCode::Mode>(record->get(Slic3r::CustomGCode::PrintRecordModeKey).get_int());
    const ConfigOption heights = record->get(Slic3r::CustomGCode::PrintRecordPrintZKey);
    const ConfigOption types = record->get(Slic3r::CustomGCode::PrintRecordTypeKey);
    const ConfigOption tools = record->get(Slic3r::CustomGCode::PrintRecordExtruderKey);
    const int32_t count = int32_t(print.config().get("nozzle_diameter").size());
    for (uint32_t idx = 0; idx < heights.size(); ++idx) {
        CustomGCodeRow row;
        row.print_z_mm = heights.get_float(idx);
        row.type = static_cast<Slic3r::CustomGCode::Type>(types.get_int(idx));
        if (row.type == Slic3r::CustomGCode::ToolChange)
            row.extruder = std::clamp(tools.get_int(idx), int32_t(1), count);
        else if (row.type == Slic3r::CustomGCode::ColorChange)
            row.extruder = tools.get_int(idx);
        events.rows.push_back(row);
    }
    std::stable_sort(events.rows.begin(), events.rows.end(), [](const CustomGCodeRow &a, const CustomGCodeRow &b) {
        return a.print_z_mm < b.print_z_mm;
    });
    return events;
}

void append_island_groups(const Print &print, PrintingPlan &plan,
    const std::vector<SourceInstance> &instances, coord_t height_step_range,
    coord_t lo, coord_t hi, extruder_index_t clearance_tool, const plugin_run_context *run_ctx)
{
    using IslandStacks::ObjectLayerToPrint;
    using IslandStacks::ObjectsLayerToPrint;
    std::vector<OwnedGraphData> owned_clearance;
    std::vector<GraphData> extruder_clearance;
    const ConfigOption clearance = print.config().get("extruder_clearance");
    const double radius = print.config().float_or_default("extruder_clearance_radius", 0.);
    const double height = print.config().float_or_default("extruder_clearance_height", 0.);
    for (uint32_t idx = 0; idx < clearance.size(); ++idx) {
        const GraphData graph = clearance.graph(idx).value();
        std::vector<c_vec2d> points;
        // The printer's radius and height cap the usable nozzle profile.
        for (const ConfigPoint &point : graph.points())
            if (point.x < radius && point.y < height)
                points.push_back(c_vec2d{point.x, point.y});
        points.push_back(c_vec2d{radius, height});
        owned_clearance.emplace_back(points, graph.type());
        extruder_clearance.push_back(owned_clearance.back().view());
    }
    ObjectsLayerToPrint object_and_support_layers;
    std::map<uint32_t, const SourceInstance *> sources;
    for (uint32_t rank = 0; rank < instances.size(); ++rank) {
        const SourceInstance &source = instances[rank];
        throw_if_cancelled(run_ctx);
        const Object object = source.object;
        const uint32_t catalog_id = plan.find_or_add_object_instance(object, source.object_instance_idx);
        sources.emplace(catalog_id, &source);
        for (uint32_t idx = 0; idx < object.layer_count(); ++idx) {
            const Layer layer = object.layer(idx);
            if (layer.print_z() < lo || layer.print_z() >= hi)
                continue;
            ObjectLayerToPrint selection;
            selection.instance_id = catalog_id;
            selection.copy_rank = rank;
            selection.shift = source.shift;
            selection.object_layer = layer;
            object_and_support_layers.push_back(std::move(selection));
        }
        for (uint32_t idx = 0; idx < object.auxiliary_layer_count(); ++idx) {
            const Layer layer = object.auxiliary_layer(idx);
            const std::optional<uint32_t> owner = layer_instance_owner(layer);
            if (owner && *owner >= object.instance_count())
                throw std::out_of_range("Auxiliary layer owner instance index is outside the Object.");
            if ((owner && *owner != source.object_instance_idx) || layer.print_z() < lo || layer.print_z() >= hi)
                continue;
            ObjectLayerToPrint selection;
            selection.instance_id = catalog_id;
            selection.copy_rank = rank;
            selection.shift = source.shift;
            selection.auxiliary_layer = layer;
            object_and_support_layers.push_back(std::move(selection));
        }
    }
    // Continue the preceding plan group's last tool when the first pass supplies one.
    extruder_index_t previous_tool = EXTRUDER_ID_UNKNOWN;
    for (uint32_t idx = plan.group_count(); idx > 0 && previous_tool == EXTRUDER_ID_UNKNOWN; --idx) {
        PrintingGroup previous = plan.group(idx - 1);
        for (uint32_t layer_idx = previous.layer_group_count(); layer_idx > 0; --layer_idx) {
            PrintingLayerGroup layer = previous.layer_group(layer_idx - 1);
            if (layer.tool_group_count() != 0) {
                previous_tool = layer.tool_group(layer.tool_group_count() - 1).extruder_id();
                break;
            }
        }
    }
    const std::vector<ObjectsLayerToPrint> stacks = IslandStacks::separate_islands(
        object_and_support_layers, height_step_range, extruder_clearance,
        previous_tool, run_ctx->plugin_storage, print.config().bool_or_default("complete_islands", false), clearance_tool);
    for (const ObjectsLayerToPrint &stack : stacks) {
        throw_if_cancelled(run_ctx);
        PrintingGroup group = plan.append_group();
        if (!group.valid())
            continue;
        std::set<uint32_t> catalog_ids;
        std::map<coord_t, uint32_t> indices;
        for (const ObjectLayerToPrint &selection : stack) {
            const uint32_t catalog_id = selection.instance_id;
            const SourceInstance &source = *sources.at(catalog_id);
            if (catalog_ids.insert(catalog_id).second)
                group.append_printing_catalog_id(plan, catalog_id);
            for (const Layer &layer : {selection.object_layer, selection.auxiliary_layer}) {
                if (!layer.valid())
                    continue;
                std::vector<uint32_t> island_indices;
                for (uint32_t idx = 0; idx < layer.island_count(); ++idx)
                    if (selection.islands.count(layer.island(idx).handle()) != 0)
                        island_indices.push_back(idx);
                LayerExtrusionFilter filter;
                filter.island_indices = &island_indices;
                PrintingLayerGroup layers = layer_group_for_print_z(group, indices, layer.print_z(), run_ctx);
                append_layer_once(layers, layer, run_ctx);
                append_layer_instance_extrusions(plan, layers, layer, source.shift, catalog_id, run_ctx, filter);
            }
        }
        // IslandStacks supplies the shared tool order for every selected Z.
        std::set<coord_t> ordered_z;
        for (const ObjectLayerToPrint &selection : stack) {
            const coord_t z = selection.layer().print_z();
            if (!ordered_z.insert(z).second)
                continue;
            PrintingLayerGroup layer = group.layer_group(indices.at(z));
            for (uint32_t target = 0; target < selection.extruders_order.size(); ++target)
                for (uint32_t tool = target; tool < layer.tool_group_count(); ++tool)
                    if (layer.tool_group(tool).extruder_id() == selection.extruders_order[target]) {
                        layer.move_tool_group(tool, target);
                        break;
                    }
        }
        sort_layer_groups_by_print_z(group, run_ctx);
    }
}

void initialize_by_object_settings(orchestrator_handle *orch)
{
    const key_value_string_pair_t order_values[] = {
        { "object", "Right panel" }, { "lowx", "Min X" }, { "highx", "Max X" },
        { "lowy", "lowest Y" }, { "highy", "Max Y" },
        { "lowz", "lowest Z" }, { "nearest", "Nearest" }
    };
    raw_config_option_def def = raw_config_option_def_init();
    def.opt_key = "complete_objects_sort";
    def.type = RAW_CO_ENUM;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_ORDERING;
    def.category = RAW_OPTION_CATEGORY_OUTPUT;
    def.mode = RAW_CONFIG_OPTION_MODE_ADV_EXP | RAW_CONFIG_OPTION_MODE_SUSI;
    def.precision = 6;
    def.label = "Object sort";
    def.tooltip = "When printing multiple objects or copies on after another, this will help you to choose how it's ordered."
        "\nObject will sort them by the order of the right panel."
        "\nMin X sorts copy positions from the lowest to the highest X coordinate; Max X reverses that order."
        "\nLowest Y sorts copy positions from the lowest to the highest Y coordinate; Max Y reverses that order."
        " Useful for printers with a X-bar. Equal coordinates keep the object and copy order."
        "\nLowest Z will sort them by their height, useful for delta printers."
        "\nNearest will try to jump to the nearest.";
    def.enum_def.value_label_pairs = { order_values, 7 };
    def.default_serialized_value = "object";
    orchestrator_create_option_def(orch, &def);

    const char *step_alias = "parallel_objects_step";
    def = raw_config_option_def_init();
    def.opt_key = "complete_objects_step";
    def.type = RAW_CO_FLOAT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    // The skirt reader runs before ordering and does not require this optional definition.
    def.invalidates_step = STEP_SUPPORT;
    def.category = RAW_OPTION_CATEGORY_OUTPUT;
    def.mode = RAW_CONFIG_OPTION_MODE_ADV_EXP | RAW_CONFIG_OPTION_MODE_SUSI;
    def.precision = 6;
    def.default_serialized_value = "0";
    def.aliases = { &step_alias, 1 };
    def.label = "Object step height";
    def.tooltip = "Limit the height lead between object stacks. The step is capped by 'extruder_clearance_height'."
        "\n0 = the lead allowed by the clearance graph.";
    def.sidetext = "mm";
    orchestrator_create_option_def(orch, &def);

    const char *limit_alias = "parallel_objects_step_max_z";
    def = raw_config_option_def_init();
    def.opt_key = "complete_objects_step_max_z";
    def.type = RAW_CO_FLOAT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_ORDERING;
    def.category = RAW_OPTION_CATEGORY_OUTPUT;
    def.mode = RAW_CONFIG_OPTION_MODE_ADV_EXP | RAW_CONFIG_OPTION_MODE_SUSI;
    def.precision = 6;
    def.default_serialized_value = "0";
    def.aliases = { &limit_alias, 1 };
    def.label = "Object step max height";
    def.tooltip = "Above this height, printing switches back to layer-by-layer printing. Set to 0 for no height limit.";
    def.sidetext = "mm";
    orchestrator_create_option_def(orch, &def);

    const char *islands_alias = "parallel_islands";
    def = raw_config_option_def_init();
    def.opt_key = "complete_islands";
    def.type = RAW_CO_BOOL;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_ORDERING;
    def.category = RAW_OPTION_CATEGORY_OUTPUT;
    def.mode = RAW_CONFIG_OPTION_MODE_ADV_EXP | RAW_CONFIG_OPTION_MODE_SUSI;
    def.precision = 6;
    def.default_serialized_value = "0";
    def.aliases = { &islands_alias, 1 };
    def.label = "Print islands separately";
    def.tooltip = "Consider each object island as a separate unit, with or without a height step, if far enough."
        "\nTwo islands are considered separate if they are farther than the extruder clearance.";
    orchestrator_create_option_def(orch, &def);

    def = raw_config_option_def_init();
    def.opt_key = "complete_objects_one_first_layer";
    def.type = RAW_CO_BOOL;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_ORDERING;
    def.category = RAW_OPTION_CATEGORY_OUTPUT;
    def.mode = RAW_CONFIG_OPTION_MODE_ADV_EXP | RAW_CONFIG_OPTION_MODE_SUSI;
    def.default_serialized_value = "0";
    def.label = "Print first layers together";
    def.tooltip = "Print every copy's object and auxiliary layers together up to the highest first object layer, in increasing Z, before starting the stacks.";
    orchestrator_create_option_def(orch, &def);

    const GuiCondition by_object = string_equals("print_plan_builder", "ordering.plan_builder.by_object");
    for (const char *key : { "complete_objects_sort", "complete_objects_step", "complete_objects_step_max_z",
            "complete_islands", "complete_objects_one_first_layer" })
        GuiRule::activate(key).when(by_object).scope(RAW_PRESET_TYPE_FFF_PRINT).add_to(orch);
}

class ByObjectPlanBuilder : public PluginBase
{
public:
    // Borrow the permanent provider; initialization failures propagate.
    static ByObjectPlanBuilder &instance(orchestrator_handle *orch)
    {
        static ByObjectPlanBuilder s_instance(orch);
        return s_instance;
    }
    // Bind the borrowed orchestrator without modifying print data.
    explicit ByObjectPlanBuilder(orchestrator_handle *orch) : PluginBase(orch) {}

private:
    // Return permanent registration strings without changing state.
    const char *id_impl() const noexcept override { return "ordering.plan_builder.by_object"; }
    // Return the display name without effects.
    const char *name_impl() const noexcept override { return "By object"; }
    // Describe the owned group arrangement without effects.
    const char *description_impl() const noexcept override { return "Creates plate-wide stacks of object copies or islands, with optional shared first layers and a final by-layer height limit."; }
    // Join the plan builder alternatives without declaring another selector.
    const char *exclusive_group_impl() const noexcept override { return "ordering.plan_builder"; }
    // Run in ordering without effects.
    slicing_step_t step_impl() const noexcept override { return STEP_ORDERING; }
    // Return the empty dependency list borrowed for the application lifetime.
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }
    // Build before the other ordering phases.
    int32_t priority_impl() const noexcept override { return 0; }
    // Leave By layer as the preferred default.
    int32_t exclusive_group_priority_impl() const noexcept override { return 10; }
    // Report the count or copy the five owned setting names into caller-owned storage.
    int32_t defined_config_keys(const char **keys) const noexcept override
    {
        if (keys != nullptr)
            for (size_t idx = 0; idx < 5; ++idx)
                keys[idx] = k_defined_config_keys[idx];
        return 5;
    }
    // Report setting types so changes to the step cap or nozzle graph invalidate ordering too.
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override
    {
        if (keys != nullptr) {
            keys[0] = raw_used_config_key{ "complete_objects_sort", RAW_CO_ENUM, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE };
            keys[1] = raw_used_config_key{ "complete_objects_step", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE };
            keys[2] = raw_used_config_key{ "complete_objects_step_max_z", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE };
            keys[3] = raw_used_config_key{ "complete_islands", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE };
            keys[4] = raw_used_config_key{ "extruder_clearance_height", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE };
            keys[5] = raw_used_config_key{ "extruder_clearance", RAW_CO_VECTOR_GRAPH, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE };
            keys[6] = raw_used_config_key{ "extruder_clearance_radius", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE };
            keys[7] = raw_used_config_key{ "complete_objects_one_first_layer", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE };
        }
        return 8;
    }
    // Define the provider's settings and rules once without taking ownership of host storage.
    void inilialize_impl(storage_handle *) const override
    {
        initialize_by_object_settings(m_orchestrator);
    }
    // Replace the plan from borrowed ordering context; source and cancellation failures propagate.
    void run_impl(const plugin_run_context *run_ctx) const override
    {
        const run_ctx_extrusion_ordering *ctx = plugin_ctx_as_extrusion_ordering(run_ctx);
        assert(ctx != nullptr);
        assert(ctx->print != nullptr);
        assert(ctx->plan != nullptr);
        if (ctx == nullptr || ctx->print == nullptr || ctx->plan == nullptr)
            return;
        throw_if_cancelled(run_ctx);
        PrintingPlan plan(ctx->plan);
        const Print print(ctx->print);
        build_plan_by_object(print, plan, run_ctx);
    }
};
} // namespace

void register_by_object_plan_builder_plugin(orchestrator_handle *orch)
{
    orchestrator_register_plugin(orch, ByObjectPlanBuilder::instance(orch).c_instance());
}

}}} // namespace slic3r_api::Ordering::DefaultOrderingPlugin
