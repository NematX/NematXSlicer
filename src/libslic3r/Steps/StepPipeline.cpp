///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/* The pipeline defines executable order, checkpoint replay, and metadata for
   built-in producer and service steps. Exclusive groups select one provider;
   active setting requirements check that provider's declared option keys.
   Requested preview reads also fill Print's measured statistics; CLI reads are explicit. */
#include "StepPipeline.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/internal/LayerAccess.hpp"
#include "libslic3r/Api/internal/LayerRegionAccess.hpp"
#include "libslic3r/Api/internal/PrintObjectAccess.hpp"
#include "libslic3r/Config/ConfigDef.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/I18N.hpp"
#include "libslic3r/format.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Slicing.hpp"
#include "libslic3r/Steps/StepDetectSupportSpots.hpp"
#include "libslic3r/Steps/StepExtrusionEdition.hpp"
#include "libslic3r/Steps/StepExtrusionOrdering.hpp"
#include "libslic3r/Steps/StepExtrusionSimplification.hpp"
#include "libslic3r/Steps/StepFinalizeSlicing.hpp"
#include "libslic3r/Steps/StepGenerateGcode.hpp"
#include "libslic3r/Steps/StepReadGeneratedGcode.hpp"
#include "libslic3r/Steps/StepGenerateInfill.hpp"
#include "libslic3r/Steps/StepGeneratePerimeter.hpp"
#include "libslic3r/Steps/StepGenerateSupport.hpp"
#include "libslic3r/Steps/StepGenerateWipeTower.hpp"
#include "libslic3r/Steps/StepGroupInfillRegions.hpp"
#include "libslic3r/Steps/StepLayerExtrusionEdition.hpp"
#include "libslic3r/Steps/StepLayerHeightGeneration.hpp"
#include "libslic3r/Steps/StepLayerStiching.hpp"
#include "libslic3r/Steps/StepPostInfillGeneration.hpp"
#include "libslic3r/Steps/StepPostPerimeterGeneration.hpp"
#include "libslic3r/Steps/StepPostSlicing.hpp"
#include "libslic3r/Steps/StepPrepareForPeriemters.hpp"
#include "libslic3r/Steps/StepPrepareInfill.hpp"
#include "libslic3r/Steps/StepSlicing.hpp"
#include "libslic3r/Steps/StepRunner.hpp"
#include "libslic3r/Steps/StepSkirtBrim.hpp"
#include "libslic3r/Steps/StepSupportDemand.hpp"
#include "libslic3r/Steps/StepSupportToolpath.hpp"
#include "libslic3r/Steps/StepSurfaceGeneration.hpp"
#include "libslic3r/SurfaceCollection.hpp"
#include "libslic3r/Thread.hpp"

#include <boost/log/trivial.hpp>

#include <algorithm>
#include <cassert>
#include <cctype>
#include <map>
#include <set>
#include <string>
#include <stdexcept>
#include <vector>

namespace Slic3r::Steps {

// Built-in steps in execution order; dispatch and invalidation follow this order.
const std::vector<slicing_step_t> &execution_order()
{
    static const std::vector<slicing_step_t> steps {
        STEP_LAYER_HEIGHT,
        STEP_SLICING,
        STEP_POST_SLICING,
        STEP_SUPPORT_DEMAND,
        STEP_PRE_PERIMETER,
        STEP_PERIMETER,
        STEP_POST_PERIMETER,
        STEP_SURFACE_GENERATION,
        STEP_PRE_INFILL,
        STEP_INFILL_GROUP,
        STEP_INFILL,
        STEP_POST_INFILL,
        STEP_SUPPORT,
        STEP_SUPPORT_TOOLPATH,
        STEP_SKIRT_BRIM,
        STEP_FINALIZE_SLICING,
        STEP_ORDERING,
        STEP_WIPETOWER,
        STEP_SUPPORT_SPOT,
        STEP_LAYER_EXTRUSION_EDIT,
        STEP_LAYER_STICHING,
        STEP_EXTRUSION_EDIT,
        STEP_EXTRUSION_SIMPLIFICATION,
        STEP_GCODE,
    };
    return steps;
}

size_t execution_index(const slicing_step_t step)
{
    const std::vector<slicing_step_t> &steps = execution_order();
    const std::vector<slicing_step_t>::const_iterator found =
        std::find(steps.begin(), steps.end(), step);
    return found == steps.end() ? steps.size() : size_t(std::distance(steps.begin(), found));
}

bool executes_before(const slicing_step_t lhs, const slicing_step_t rhs)
{
    const size_t lhs_index = execution_index(lhs);
    const size_t rhs_index = execution_index(rhs);
    assert(lhs_index < execution_order().size());
    assert(rhs_index < execution_order().size());
    return lhs_index < rhs_index;
}

slicing_step_t tightened_invalidation_step(const slicing_step_t current,
                                           const slicing_step_t consumer_step)
{
    // STEP_ANY already replays every producer. STEP_NONE declares that a value
    // change never affects slicing, for example an import-only placement
    // setting, so a reader must not turn it into a replay.
    if (current == STEP_ANY || current == STEP_NONE)
        return current;
    // Replaying a step also replays every later step through the dependency
    // chain, so the earlier of the two covers both producers.
    return executes_before(consumer_step, current) ? consumer_step : current;
}

Checkpoint checkpoint_before_step(const slicing_step_t step)
{
    if (step == STEP_ANY)
        return Checkpoint::None;

    const size_t step_index = execution_index(step);
    assert(step_index < execution_order().size());
    if (step_index >= execution_order().size())
        return Checkpoint::None;

    // Each checkpoint sits right before its resume step, so the latest
    // checkpoint whose resume step is not after `step` is the one to use.
    if (step_index < execution_index(checkpoint_resume_step(Checkpoint::Slicing)))
        return Checkpoint::None;
    if (step_index < execution_index(checkpoint_resume_step(Checkpoint::Print)))
        return Checkpoint::Slicing;
    if (step_index < execution_index(checkpoint_resume_step(Checkpoint::Ordering)))
        return Checkpoint::Print;
    if (step_index < execution_index(checkpoint_resume_step(Checkpoint::Plan)))
        return Checkpoint::Ordering;
    return Checkpoint::Plan;
}

slicing_step_t checkpoint_resume_step(const Checkpoint checkpoint)
{
    switch (checkpoint) {
    case Checkpoint::None:     return STEP_LAYER_HEIGHT;
    case Checkpoint::Slicing:  return STEP_SUPPORT_DEMAND;
    case Checkpoint::Print:    return STEP_ORDERING;
    case Checkpoint::Ordering: return STEP_WIPETOWER;
    case Checkpoint::Plan:     return STEP_GCODE;
    case Checkpoint::GCode:    return STEP_NONE;
    }
    assert(false);
    return STEP_LAYER_HEIGHT;
}

// For each step, the steps that consume its outputs directly.
const std::map<slicing_step_t, std::vector<slicing_step_t>> &step_dependents()
{
    static const std::map<slicing_step_t, std::vector<slicing_step_t>> dependents {
        {STEP_LAYER_HEIGHT, {STEP_SLICING}},
        {STEP_SLICING, {STEP_POST_SLICING}},
        {STEP_POST_SLICING, {STEP_SUPPORT_DEMAND}},
        {STEP_SUPPORT_DEMAND, {STEP_PRE_PERIMETER}},
        {STEP_PRE_PERIMETER, {STEP_PERIMETER}},
        {STEP_PERIMETER, {STEP_POST_PERIMETER}},
        {STEP_POST_PERIMETER, {STEP_SURFACE_GENERATION}},
        {STEP_SURFACE_GENERATION, {STEP_PRE_INFILL}},
        {STEP_PRE_INFILL, {STEP_INFILL_GROUP}},
        {STEP_INFILL_GROUP, {STEP_INFILL}},
        {STEP_INFILL, {STEP_POST_INFILL}},
        {STEP_POST_INFILL, {STEP_SUPPORT}},
        {STEP_SUPPORT, {STEP_SUPPORT_TOOLPATH}},
        {STEP_SUPPORT_TOOLPATH, {STEP_SKIRT_BRIM}},
        {STEP_SKIRT_BRIM, {STEP_FINALIZE_SLICING}},
        {STEP_FINALIZE_SLICING, {STEP_ORDERING}},
        {STEP_ORDERING, {STEP_WIPETOWER}},
        {STEP_WIPETOWER, {STEP_SUPPORT_SPOT}},
        {STEP_SUPPORT_SPOT, {STEP_LAYER_EXTRUSION_EDIT}},
        {STEP_LAYER_EXTRUSION_EDIT, {STEP_LAYER_STICHING}},
        {STEP_LAYER_STICHING, {STEP_EXTRUSION_EDIT}},
        {STEP_EXTRUSION_EDIT, {STEP_EXTRUSION_SIMPLIFICATION}},
        {STEP_EXTRUSION_SIMPLIFICATION, {STEP_GCODE}},
    };
    return dependents;
}

std::vector<slicing_step_t> dependent_steps_closure(slicing_step_t step)
{
    const std::map<slicing_step_t, std::vector<slicing_step_t>> &dependents = step_dependents();
    std::set<slicing_step_t> visited;
    std::vector<slicing_step_t> stack;

    if (auto it = dependents.find(step); it != dependents.end())
        stack.insert(stack.end(), it->second.begin(), it->second.end());

    while (!stack.empty()) {
        const slicing_step_t current = stack.back();
        stack.pop_back();
        if (!visited.insert(current).second)
            continue;

        if (auto it = dependents.find(current); it != dependents.end())
            stack.insert(stack.end(), it->second.begin(), it->second.end());
    }

    std::vector<slicing_step_t> out;
    out.reserve(visited.size());
    for (slicing_step_t ordered_step : execution_order())
        if (visited.find(ordered_step) != visited.end())
            out.push_back(ordered_step);
    return out;
}

#ifdef _DEBUG
bool validate_execution_order_against_dependencies()
{
    std::map<slicing_step_t, size_t> order_index;
    const std::vector<slicing_step_t> &order = execution_order();
    for (size_t idx = 0; idx < order.size(); ++idx)
        order_index.emplace(order[idx], idx);

    for (const auto &[producer, consumers] : step_dependents()) {
        const auto producer_it = order_index.find(producer);
        assert(producer_it != order_index.end());
        if (producer_it == order_index.end())
            return false;

        for (slicing_step_t consumer : consumers) {
            const auto consumer_it = order_index.find(consumer);
            assert(consumer_it != order_index.end());
            if (consumer_it == order_index.end())
                return false;
            assert(producer_it->second < consumer_it->second);
            if (producer_it->second >= consumer_it->second)
                return false;
        }
    }
    return true;
}
#endif

namespace {

// Recognize built-in producer and service identities without changing scheduling state; unknown values return false.
bool is_known_builtin_step(slicing_step_t step);
// Map producer and service identities to option categories; unclassified steps use the slicing category.
raw_option_category option_category_for_step(slicing_step_t step);

std::string exclusive_group_ui_fragment(const std::string &key,
                                        const std::string &line_label)
{
    // This is the neutral fallback for exclusive groups that do not provide
    // their own placement fragment. Feature plugins may register a fragment
    // with the same group id to put the selector next to their settings.
    return std::string("page:Notes\n") +
           "group:Exclusive step plugins\n" +
           "line:" + line_label + "\n" +
           "setting:" + key + "\n" +
           "end_line\n";
}

std::string sanitized_config_key_part(const std::string &text)
{
    std::string out;
    out.reserve(text.size());
    bool previous_was_separator = false;

    for (const char raw_ch : text) {
        const unsigned char ch = static_cast<unsigned char>(raw_ch);
        if (std::isalnum(ch) != 0) {
            out.push_back(static_cast<char>(std::tolower(ch)));
            previous_was_separator = false;
        } else if (!previous_was_separator && !out.empty()) {
            out.push_back('_');
            previous_was_separator = true;
        }
    }

    while (!out.empty() && out.back() == '_')
        out.pop_back();
    return out.empty() ? "plugin_group" : out;
}

std::string option_key_for_plugin_group(const Orchestrator &orchestrator,
                                        slicing_step_t step,
                                        const std::string &group_id)
{
    // Preserve the historical key; its dynamic enum now stores provider ids.
    if (step == GCODE_THUMBNAIL && group_id == "thumbnail_encoder_plugin")
        return "thumbnails_format";
    const Orchestrator::DynamicStepInfo *step_info = orchestrator.step_info(step);
    const std::string step_identity = step_info != nullptr ?
        sanitized_config_key_part(step_info->name) : std::to_string(int(step));
    return "exclusive_group_" + step_identity + "_" +
           sanitized_config_key_part(group_id) + "_plugin";
}

// Compare the numeric identity with the built-in list; services are recognized but are not added to execution order.
bool is_known_builtin_step(slicing_step_t step)
{
    switch (step) {
    case STEP_NONE:
    case STEP_ANY:
    case STEP_LAYER_HEIGHT:
    case STEP_SLICING:
    case STEP_POST_SLICING:
    case STEP_ALERT_SUPPORTS_NEEDED:
    case STEP_PRE_PERIMETER:
    case STEP_PERIMETER:
    case STEP_POST_PERIMETER:
    case STEP_SURFACE_GENERATION:
    case STEP_SKIRT_BRIM:
    case STEP_PRE_INFILL:
    case STEP_INFILL_GROUP:
    case STEP_INFILL:
    case STEP_POST_INFILL:
    case STEP_SUPPORT_DEMAND:
    case STEP_SUPPORT:
    case STEP_SUPPORT_TOOLPATH:
    case STEP_FINALIZE_SLICING:
    case STEP_CHECK_CONFLICT:
    case STEP_ORDERING:
    case STEP_WIPETOWER:
    case STEP_SUPPORT_SPOT:
    case STEP_LAYER_EXTRUSION_EDIT:
    case STEP_LAYER_STICHING:
    case STEP_EXTRUSION_EDIT:
    case STEP_EXTRUSION_SIMPLIFICATION:
    case STEP_GCODE:
    case INFILL_PATTERN:
    case INFILL_SURFACE_RECIPE_MODIFIER:
    case BRIDGE_DETECTOR:
    case PERIMETER_GENERATION_MODULE:
    case GCODE_FIRMWARE:
    case GCODE_THUMBNAIL:
    case GCODE_READER:
    case WIPE_TOWER_WIPE:
        return true;
    default:
        return false;
    }
}

// Classify the supplied step for option presentation, returning slicing for unclassified values without mutation.
raw_option_category option_category_for_step(slicing_step_t step)
{
    switch (step) {
    case STEP_PERIMETER:
    case STEP_POST_PERIMETER:
    case STEP_PRE_PERIMETER:
        return RAW_OPTION_CATEGORY_PERIMETER;
    case STEP_INFILL:
    case STEP_INFILL_GROUP:
    case STEP_PRE_INFILL:
    case STEP_POST_INFILL:
        return RAW_OPTION_CATEGORY_INFILL;
    case STEP_SUPPORT:
    case STEP_SUPPORT_TOOLPATH:
    case STEP_SUPPORT_DEMAND:
    case STEP_SUPPORT_SPOT:
        return RAW_OPTION_CATEGORY_SUPPORT;
    case STEP_GCODE:
    case GCODE_FIRMWARE:
    case GCODE_THUMBNAIL:
    case GCODE_READER:
    case STEP_ORDERING:
    case STEP_WIPETOWER:
    case WIPE_TOWER_WIPE:
    case STEP_LAYER_STICHING:
        return RAW_OPTION_CATEGORY_OUTPUT;
    default:
        return RAW_OPTION_CATEGORY_SLICING;
    }
}

raw_option_preset_type option_preset_type_for_step(slicing_step_t step)
{
    /*
    Exclusive selectors are normal config options, so their preset family must
    match the GUI file that displays them. Most step selectors are print-profile
    choices, but G-code backend selection belongs next to printer firmware
    settings and therefore has to be stored in the printer preset.
    */
    static const std::map<slicing_step_t, raw_option_preset_type> s_step_preset_types = {
        { STEP_GCODE, RAW_PRESET_TYPE_FFF_PRINTER },
        { GCODE_FIRMWARE, RAW_PRESET_TYPE_FFF_PRINTER },
        { GCODE_THUMBNAIL, RAW_PRESET_TYPE_FFF_PRINTER },
        { GCODE_READER, RAW_PRESET_TYPE_FFF_PRINTER }
    };

    const std::map<slicing_step_t, raw_option_preset_type>::const_iterator found =
        s_step_preset_types.find(step);
    return found == s_step_preset_types.end() ? RAW_PRESET_TYPE_FFF_PRINT : found->second;
}

/*
Populate the returned group with owned display strings, borrowed raw-option pointers, and its UI
fragment; copied strings become group-owned buffers, while the raw option definition borrows those
buffers. Mark the selector enum for inactive-ID pruning during finalization. That finalization
replaces the enum table, which ConfigOptionEnumGeneric only borrows, so no selector-bearing config
may exist before finalize_plugin_option_defs(); this function creates metadata only. Allocation
failures propagate and no option-definition validation occurs here.
*/
StepExclusiveGroup make_exclusive_group(slicing_step_t step,
                                        const std::string &group_id,
                                        const std::string &key,
                                        const std::string &label,
                                        raw_option_category category,
                                        const std::string &line_label,
                                        const std::string &tooltip,
                                        raw_option_preset_type option_preset_type)
{
    StepExclusiveGroup group = {};
    group.group_id = group_id;
    group.option_key_storage = key;
    group.label_storage = label;
    group.tooltip_storage = tooltip;
    group.step = step;

    raw_config_option_def def = raw_config_option_def_init();
    def.type = RAW_CO_ENUM;
    def.gui_type = RAW_GUI_TYPE_SELECT_CLOSE;
    def.container_type = RAW_CONTAINER_TYPE_PROJECT;
    // Request removal of inactive plugin IDs from this selector during finalization.
    def.contains_plugin_id = 1;
    def.option_preset_type = option_preset_type;
    def.printer_technology = RAW_PT_FFF;
    def.category = category;
    // Service selectors invalidate the consuming pipeline step rather than
    // pretending that the service itself is independently executable.
    def.invalidates_step =
        (step == GCODE_FIRMWARE || step == GCODE_THUMBNAIL || step == GCODE_READER) ?
            STEP_GCODE : step;
    def.mode = RAW_CONFIG_OPTION_MODE_ADV_EXP | RAW_CONFIG_OPTION_MODE_SUSI;
    group.option_def = def;

    group.ui_fragment = exclusive_group_ui_fragment(group.option_key_storage, line_label);
    group.refresh_storage_pointers();
    return group;
}

StepExclusiveGroup make_exclusive_step_group(slicing_step_t step,
                                             const char *key,
                                             const char *label,
                                             raw_option_category category,
                                             const char *line_label)
{
    return make_exclusive_group(step,
                                key,
                                key,
                                label,
                                category,
                                line_label,
                                "Choose which active plugin owns this exclusive slicing step.",
                                option_preset_type_for_step(step));
}

StepExclusiveGroup make_plugin_exclusive_group(const Orchestrator &orchestrator,
                                               slicing_step_t step,
                                               const std::string &group_id,
                                               const Plugin &first_plugin)
{
    const std::string key = plugin_exclusive_group_option_key(
        orchestrator, step, group_id);
    const std::string label = first_plugin.get_exclusive_group_label().empty() ?
        group_id :
        first_plugin.get_exclusive_group_label();
    const std::string tooltip = first_plugin.get_exclusive_group_tooltip().empty() ?
        "Choose which active plugin owns this exclusive plugin group." :
        first_plugin.get_exclusive_group_tooltip();

    StepExclusiveGroup group = make_exclusive_group(step,
                                                    group_id,
                                                    key,
                                                    label,
                                                    option_category_for_step(step),
                                                    label,
                                                    tooltip,
                                                    option_preset_type_for_step(step));
    const Orchestrator::DynamicStepInfo *step_info = orchestrator.step_info(step);
    if (step_info != nullptr)
        group.option_def.invalidates_step = step_info->invalidates_step;
    else if (!is_known_builtin_step(step))
        group.option_def.invalidates_step = STEP_ANY;

    /*
    Seam placement is selected by the effective PrintRegion configuration.
    Other plugin groups keep their project-level selector because their
    consumers execute one implementation for the complete print.
    */
    if (step == SEAM_PLACER && group_id == "seam_placer_plugin") {
        group.option_def.container_type = RAW_CONTAINER_TYPE_REGION;
        group.option_def.invalidates_step = STEP_ORDERING;
        group.ui_fragment =
            "page:Perimeters & Shell\n"
            "group:Seam\n"
            "line:" + label + "\n"
            "setting:" + key + "\n"
            "end_line\n";
    }
    group.refresh_storage_pointers();
    return group;
}

void apply_plugin_group_text(StepExclusiveGroup &group, const std::vector<Plugin *> &plugins)
{
    // The selector belongs to the group, not to one plugin. When plugin authors
    // provide group text, the first active plugin in execution order owns the
    // wording so every project sees one stable label for that group.
    for (const Plugin *plugin : plugins) {
        if (plugin == nullptr)
            continue;
        if (plugin->get_exclusive_group_label().empty() && plugin->get_exclusive_group_tooltip().empty())
            continue;

        if (!plugin->get_exclusive_group_label().empty()) {
            group.label_storage = plugin->get_exclusive_group_label();
            if (group.step == SEAM_PLACER &&
                group.group_id == "seam_placer_plugin") {
                // Keep this regional service selector beside the other seam
                // settings even when a provider supplies a friendlier label.
                group.ui_fragment =
                    "page:Perimeters & Shell\n"
                    "group:Seam\n"
                    "line:" + group.label_storage + "\n"
                    "setting:" + group.option_key_storage + "\n"
                    "end_line\n";
            } else {
                group.ui_fragment = exclusive_group_ui_fragment(
                    group.option_key_storage, group.label_storage);
            }
        }
        if (!plugin->get_exclusive_group_tooltip().empty())
            group.tooltip_storage = plugin->get_exclusive_group_tooltip();
        group.refresh_storage_pointers();
        return;
    }
}

} // namespace

std::string plugin_exclusive_group_option_key(const Orchestrator &orchestrator,
                                              const slicing_step_t step,
                                              const std::string &group_id)
{
    return option_key_for_plugin_group(orchestrator, step, group_id);
}

void StepExclusiveGroup::refresh_storage_pointers()
{
    if (!option_key_storage.empty())
        option_def.opt_key = option_key_storage.c_str();
    if (!label_storage.empty()) {
        option_def.label = label_storage.c_str();
        option_def.full_label = label_storage.c_str();
    }
    if (!tooltip_storage.empty())
        option_def.tooltip = tooltip_storage.c_str();

    enum_pairs.clear();
    enum_pairs.reserve(enum_values.size());
    for (size_t i = 0; i < enum_values.size(); ++i) {
        key_value_string_pair_t pair = {};
        pair.value = enum_values[i].c_str();
        pair.label = i < enum_labels.size() ? enum_labels[i].c_str() : enum_values[i].c_str();
        enum_pairs.push_back(pair);
    }

    option_def.enum_def.value_label_pairs.items = enum_pairs.empty() ? nullptr : enum_pairs.data();
    option_def.enum_def.value_label_pairs.count = uint32_t(enum_pairs.size());
    option_def.default_serialized_value = enum_values.empty() ? "" : enum_values.front().c_str();
    // PNG remains the standard default even if a third-party encoder sorts first.
    if (step == GCODE_THUMBNAIL && group_id == "thumbnail_encoder_plugin")
        for (const std::string &id : enum_values)
            if (id == "PNG") option_def.default_serialized_value = id.c_str();
}

void StepExclusiveGroup::set_enum_plugins(const std::vector<std::pair<std::string, std::string>> &plugin_ids_and_labels)
{
    enum_values.clear();
    enum_labels.clear();
    enum_values.reserve(plugin_ids_and_labels.size());
    enum_labels.reserve(plugin_ids_and_labels.size());
    for (const std::pair<std::string, std::string> &plugin : plugin_ids_and_labels) {
        enum_values.emplace_back(plugin.first);
        enum_labels.emplace_back(plugin.second.empty() ? plugin.first : plugin.second);
    }
    refresh_storage_pointers();
}

const std::map<slicing_step_t, StepExclusiveGroup> &get_exclusive_steps()
{
    static const std::map<slicing_step_t, StepExclusiveGroup> s_groups = {
        {STEP_LAYER_HEIGHT,       make_exclusive_step_group(STEP_LAYER_HEIGHT,       "step_layer_height_plugin",       "Layer height plugin",       RAW_OPTION_CATEGORY_SLICING,   "Layer height step plugin")},
        {STEP_SLICING,            make_exclusive_step_group(STEP_SLICING,            "step_slicing_plugin",            "Slicing plugin",            RAW_OPTION_CATEGORY_SLICING,   "Slicing step plugin")},
        {STEP_PERIMETER,          make_exclusive_step_group(STEP_PERIMETER,          "step_perimeter_plugin",          "Perimeter plugin",          RAW_OPTION_CATEGORY_PERIMETER, "Perimeter step plugin")},
        {STEP_INFILL_GROUP,       make_exclusive_step_group(STEP_INFILL_GROUP,       "step_infill_group_plugin",       "Infill grouping plugin",    RAW_OPTION_CATEGORY_INFILL,    "Infill grouping step plugin")},
        {STEP_INFILL,             make_exclusive_step_group(STEP_INFILL,             "step_infill_plugin",             "Infill plugin",             RAW_OPTION_CATEGORY_INFILL,    "Infill step plugin")},
        {STEP_SUPPORT,            make_exclusive_step_group(STEP_SUPPORT,            "step_support_plugin",            "Support plugin",            RAW_OPTION_CATEGORY_SUPPORT,   "Support step plugin")},
        {STEP_WIPETOWER,          make_exclusive_step_group(STEP_WIPETOWER,          "step_wipetower_plugin",          "Wipe tower plugin",         RAW_OPTION_CATEGORY_OUTPUT,    "Wipe tower step plugin")},
        {STEP_LAYER_STICHING,     make_exclusive_step_group(STEP_LAYER_STICHING,     "step_layer_stiching_plugin",     "Layer stitching plugin",    RAW_OPTION_CATEGORY_OUTPUT,    "Layer stitching step plugin")},
        {STEP_GCODE,              make_exclusive_step_group(STEP_GCODE,              "step_gcode_plugin",              "G-code plugin",             RAW_OPTION_CATEGORY_OUTPUT,    "G-code step plugin")},
        {GCODE_FIRMWARE,          make_exclusive_step_group(GCODE_FIRMWARE,          "gcode_firmware_plugin",          "G-code firmware",           RAW_OPTION_CATEGORY_OUTPUT,    "G-code firmware")}
    };
    return s_groups;
}

std::vector<StepExclusivePluginGroup> active_exclusive_plugin_groups(Orchestrator &orchestrator)
{
    std::vector<StepExclusivePluginGroup> out;
    const std::map<slicing_step_t, StepExclusiveGroup> &exclusive_steps = get_exclusive_steps();

    // Host-defined unique steps always have exactly one exclusive group. This
    // keeps old "replace this whole step" plugins and plugin-declared groups in
    // the same selection model.
    for (const std::pair<const slicing_step_t, StepExclusiveGroup> &entry : exclusive_steps) {
        std::vector<Plugin *> active_plugins = orchestrator.get_active_plugins_for_step(entry.first);
        // The printer layout anchors the firmware selector to the G-code selector even with one writer.
        if (active_plugins.empty() || (active_plugins.size() == 1 && entry.first != GCODE_FIRMWARE && entry.first != STEP_GCODE))
            continue;

        StepExclusivePluginGroup plugin_group;
        plugin_group.group = entry.second;
        plugin_group.group.refresh_storage_pointers();
        apply_plugin_group_text(plugin_group.group, active_plugins);
        plugin_group.plugins = std::move(active_plugins);
        out.push_back(std::move(plugin_group));
    }

    std::map<slicing_step_t, std::vector<Plugin *>> active_plugins_by_step;
    for (Plugin *plugin : orchestrator.registered_plugins()) {
        if (plugin != nullptr && orchestrator.is_plugin_active(plugin))
            active_plugins_by_step[plugin->get_step()].push_back(plugin);
    }

    for (std::pair<const slicing_step_t, std::vector<Plugin *>> &step_plugins : active_plugins_by_step) {
        if (exclusive_steps.find(step_plugins.first) != exclusive_steps.end())
            continue;

        std::stable_sort(step_plugins.second.begin(), step_plugins.second.end(), [](const Plugin *lhs, const Plugin *rhs) {
            return lhs->get_priority() < rhs->get_priority();
        });

        std::map<std::string, std::vector<Plugin *>> plugins_by_group_id;
        for (Plugin *plugin : step_plugins.second)
            if (plugin != nullptr && !plugin->get_exclusive_group().empty())
                plugins_by_group_id[plugin->get_exclusive_group()].push_back(plugin);

        for (const std::pair<const std::string, std::vector<Plugin *>> &entry : plugins_by_group_id) {
            if (entry.second.size() <= 1)
                continue;

            StepExclusivePluginGroup plugin_group;
            plugin_group.group = make_plugin_exclusive_group(
                orchestrator, step_plugins.first, entry.first, *entry.second.front());
            apply_plugin_group_text(plugin_group.group, entry.second);
            plugin_group.plugins = entry.second;
            out.push_back(std::move(plugin_group));
        }
    }

    return out;
}

/*
Resolve providers by serialized enum ID because activation may reorder entries in the registered
table. The group, configuration, and plugin list are borrowed and unchanged; the returned plugin is
borrowed. An empty list returns null, while missing configuration, a missing option, or one
non-thumbnail provider returns the first provider. A matching ID returns its provider. Unknown
regional seam or thumbnail IDs return null so a stale seam choice cannot silently move seam
placement; other unknown IDs retain the first-provider fallback. An out-of-table enum serializes to
an empty string and follows this same unknown-ID path.
*/
Plugin *selected_plugin_from_group(const StepExclusiveGroup &group,
                                   const std::vector<Plugin *> &plugins,
                                   const ConfigBase *config)
{
    if (plugins.empty())
        return nullptr;
    if (config == nullptr || (plugins.size() == 1 && group.step != GCODE_THUMBNAIL))
        return plugins.front();

    const ConfigOption *option = config->option(group.option_def.opt_key);
    if (option == nullptr)
        return plugins.front();

    const std::string selected_id = option->serialize();
    for (Plugin *plugin : plugins)
        if (plugin->get_id() == selected_id)
            return plugin;

    if ((group.step == SEAM_PLACER && group.group_id == "seam_placer_plugin") ||
        (group.step == GCODE_THUMBNAIL && group.group_id == "thumbnail_encoder_plugin"))
        return nullptr;
    return plugins.front();
}

// Plugins to run for a step: one per exclusive group, sorted by priority.
std::vector<Plugin *> selected_or_active_plugins_for_step(Orchestrator &orchestrator,
                                                          slicing_step_t step,
                                                          const ConfigBase *config)
{
    std::vector<Plugin *> active_plugins = orchestrator.get_active_plugins_for_step(step);
    if (active_plugins.size() <= 1)
        return active_plugins;

    const std::map<slicing_step_t, StepExclusiveGroup> &exclusive_steps = get_exclusive_steps();
    const std::map<slicing_step_t, StepExclusiveGroup>::const_iterator exclusive_it = exclusive_steps.find(step);
    if (exclusive_it != exclusive_steps.end()) {
        StepExclusiveGroup group = exclusive_it->second;
        group.refresh_storage_pointers();
        Plugin *selected = selected_plugin_from_group(group, active_plugins, config);
        return selected != nullptr ? std::vector<Plugin *>{ selected } : std::vector<Plugin *>{};
    }

    std::map<std::string, std::vector<Plugin *>> plugins_by_group_id;
    for (Plugin *plugin : active_plugins)
        if (plugin != nullptr && !plugin->get_exclusive_group().empty())
            plugins_by_group_id[plugin->get_exclusive_group()].push_back(plugin);

    std::map<std::string, bool> group_already_emitted;
    std::vector<Plugin *> selected_plugins;
    selected_plugins.reserve(active_plugins.size());
    for (Plugin *plugin : active_plugins) {
        if (plugin == nullptr)
            continue;

        const std::string &group_id = plugin->get_exclusive_group();
        if (group_id.empty()) {
            selected_plugins.push_back(plugin);
            continue;
        }
        if (group_already_emitted[group_id])
            continue;
        group_already_emitted[group_id] = true;

        const std::vector<Plugin *> &group_plugins = plugins_by_group_id[group_id];
        StepExclusiveGroup group = make_plugin_exclusive_group(
            orchestrator, step, group_id, *group_plugins.front());
        group.refresh_storage_pointers();
        Plugin *selected = selected_plugin_from_group(group, group_plugins, config);
        if (selected != nullptr)
            selected_plugins.push_back(selected);
    }

    // Selected providers run by priority, not by each group's first candidate.
    // Stable sorting preserves their discovery order when priorities are equal.
    std::stable_sort(selected_plugins.begin(), selected_plugins.end(), [](const Plugin *lhs, const Plugin *rhs) {
        return lhs->get_priority() < rhs->get_priority();
    });
    return selected_plugins;
}

Plugin *selected_or_active_plugin_for_step(Orchestrator &orchestrator,
                                           slicing_step_t step,
                                           const ConfigBase *config)
{
    const std::vector<Plugin *> plugins = selected_or_active_plugins_for_step(orchestrator, step, config);
    assert(plugins.size() <= 1);
    return plugins.empty() ? nullptr : plugins.front();
}

Plugin *selected_active_plugin_from_group(Orchestrator &orchestrator,
                                          slicing_step_t step,
                                          const std::string &exclusive_group,
                                          const ConfigBase *config)
{
    if (exclusive_group.empty())
        return nullptr;

    std::vector<Plugin *> group_plugins;
    for (Plugin *plugin : orchestrator.get_active_plugins_for_step(step))
        if (plugin != nullptr && plugin->get_exclusive_group() == exclusive_group)
            group_plugins.push_back(plugin);
    if (group_plugins.empty())
        return nullptr;

    StepExclusiveGroup group = make_plugin_exclusive_group(
        orchestrator, step, exclusive_group, *group_plugins.front());
    group.refresh_storage_pointers();
    return selected_plugin_from_group(group, group_plugins, config);
}

// Borrow definitions/config and a resolver returning borrowed group providers;
// compare required values with their typed defaults and return the first owned,
// translatable error, or nullopt. Required definitions need compatible defaults;
// resolver/comparison/formatting failures propagate. No input is mutated or retained.
std::optional<std::string> unsupported_active_setting(
    const ConfigDef &definitions, const ConfigBase &config,
    const std::function<const Plugin *(const std::string &group_id)> &selected_plugin_of_group)
{
    for (const auto &[key, definition] : definitions.options) {
        if (definition.required_plugin_group.empty())
            continue;
        const ConfigOption *value = config.option(key);
        if (value == nullptr || *value == *definition.default_value)
            continue;

        const Plugin *plugin = selected_plugin_of_group(definition.required_plugin_group);
        if (plugin == nullptr)
            return format(_u8L("The setting \"%1%\" (%2%) is active, but no plugin is selected in the \"%3%\" group."),
                          key, definition.translated_label(), definition.required_plugin_group);
        const std::vector<Plugin::UsedConfigKey> &used_keys = plugin->get_used_config_keys();
        if (std::any_of(used_keys.begin(), used_keys.end(), [&key](const Plugin::UsedConfigKey &used) {
                return used.key == key;
            }))
            continue;

        const std::string &group = plugin->get_exclusive_group_label().empty() ?
            definition.required_plugin_group : plugin->get_exclusive_group_label();
        return format(_u8L("The setting \"%1%\" (%2%) is active, but the plugin \"%3%\" (%4%) selected in the \"%5%\" group does not support it."),
                      key, definition.translated_label(), plugin->get_name(), plugin->get_id(), group);
    }
    return std::nullopt;
}

namespace {

void initialize_pipeline_progress(Orchestrator &orchestrator,
                                  Print &print,
                                  const slicing_step_t first_step)
{
    if (print.pipeline_progress().active() && print.pipeline_progress().percent() < 100)
        return;

    // A completed export may be followed by another export without a new
    // slicing pass. Its selected writers and ordering providers form a fresh
    // execution, so discard the completed accounting table before rebuilding
    // the scoped plan below.
    print.pipeline_progress().reset();

    std::map<slicing_step_t, std::vector<std::string>> units_by_step;
    bool in_requested_scope = false;
    for (const slicing_step_t step : execution_order()) {
        in_requested_scope = in_requested_scope || step == first_step;
        if (!in_requested_scope)
            continue;
        if (!print.should_execute_step(step))
            continue;

        const std::vector<Plugin *> plugins = selected_or_active_plugins_for_step(
            orchestrator, step, &print.full_print_config());
        std::vector<std::string> &unit_ids = units_by_step[step];
        for (const Plugin *plugin : plugins)
            if (plugin != nullptr)
                unit_ids.push_back(plugin->get_id());
        if (unit_ids.empty())
            unit_ids.emplace_back();
    }
    print.pipeline_progress().initialize(units_by_step);
}

void begin_step(Print &print, slicing_step_t step, const std::string &message)
{
    /*
    Step labels are plain status messages, not format strings. The export path
    is reported by Orchestrator::export_gcode() before the export pipeline
    starts; forwarding it here would give boost::format an argument for labels
    such as "Ordering extrusions", which have no placeholder and crash the GUI.
    */
    print.pipeline_progress().begin_step(step, message);
}

bool stop_after(slicing_step_t current, slicing_step_t until)
{
    return current == until;
}

template<class RunFn>
void run_step_if_requested(Print &print, slicing_step_t step, RunFn &&run_fn)
{
    // Cancellation is checked even for an already valid step. This makes each
    // pipeline entry a stop boundary instead of allowing the runner to visit
    // later steps after the user has cancelled the operation.
    print.throw_if_canceled();
    if (!print.should_execute_step(step))
        return;

    // Publish the running step to the host API freeze checks. The scope also
    // resets it when the step throws, so later edits made outside the pipeline
    // are never attributed to this step.
    struct RunningStepScope
    {
        Print &print;
        ~RunningStepScope() { print.set_running_step(STEP_NONE); }
    };
    // Metadata checkpoints bracket the slicing prefix: None before its first step, Slicing after its last.
    if (step == STEP_LAYER_HEIGHT)
        print.save_metadata_checkpoint(Checkpoint::None);
    print.set_running_step(step);
    {
        const RunningStepScope running_step_scope{print};
        run_fn();
    }

    // A step may observe cancellation cooperatively and return without
    // throwing. Do not publish such partial work as a completed step.
    print.throw_if_canceled();
    print.pipeline_progress().complete_step(step);
    print.pipeline_progress().verify_step_complete(step);
    if (step == STEP_POST_SLICING) {
        print.throw_if_canceled();
        print.save_metadata_checkpoint(Checkpoint::Slicing);
    }
    print.mark_step_executed(step);
}

void run_layer_height_generation(Orchestrator &orchestrator, Print &print, const std::string &path)
{
    run_step_if_requested(print, STEP_LAYER_HEIGHT, [&] {
        begin_step(print, STEP_LAYER_HEIGHT, L("Creating the layer height"));
        StepLayerHeightGeneration::prepare(print);
        StepLayerHeightGeneration::run_step(orchestrator, print);
    });
}

void run_slicing(Orchestrator &orchestrator, Print &print, const std::string &path)
{
    run_step_if_requested(print, STEP_SLICING, [&] {
        begin_step(print, STEP_SLICING, L("StepSlicing"));
        StepSlicing::run_step(orchestrator, print);
    });
}

void run_post_slicing(Orchestrator &orchestrator, Print &print, const std::string &path)
{
    run_step_if_requested(print, STEP_POST_SLICING, [&] {
        begin_step(print, STEP_POST_SLICING, L("Post-processing slices"));
        StepPostSlicing::prepare(print);
        StepPostSlicing::run_step(orchestrator, print);
    });
}

// Borrow the host/Print and optional renderer/result destinations, executing
// marked steps through until. Export publishes the source/reader pair and only
// reads when a viewer result is requested, filling measured statistics on success.
// No borrowed pointers are retained; pipeline failures/cancellation propagate,
// while optional preview-read failures remain non-critical warnings.
void run_remaining_steps(Orchestrator &orchestrator, Print &print, const std::string &path,
                         slicing_step_t until = STEP_GCODE,
                         const raw_gcode_thumbnail_renderer *thumbnail_renderer = nullptr,
                         std::string *reader_plugin_id = nullptr,
                         GCodeProcessorResult *gcode_result = nullptr)
{
    // This helper is shared by slice preparation and export. Print::process()
    // stops it at STEP_FINALIZE_SLICING, while Orchestrator::export_gcode() calls it
    // again with only the export-side steps marked for execution.
    run_step_if_requested(print, STEP_SUPPORT_DEMAND, [&] {
        begin_step(print, STEP_SUPPORT_DEMAND, L("Detecting support demand"));
        StepSupportDemand::prepare(print);
        StepSupportDemand::run_step(orchestrator, print);
    });
    if (stop_after(STEP_SUPPORT_DEMAND, until)) return;

    run_step_if_requested(print, STEP_PRE_PERIMETER, [&] {
        begin_step(print, STEP_PRE_PERIMETER, L("Preparing perimeters"));
        StepPrepareForPeriemters::prepare(print);
        StepPrepareForPeriemters::run_step(orchestrator, print);
    });
    if (stop_after(STEP_PRE_PERIMETER, until)) return;
    
    run_step_if_requested(print, STEP_PERIMETER, [&] {
        begin_step(print, STEP_PERIMETER, L("Generating perimeters"));
        StepGeneratePerimeter::prepare(print);
        StepGeneratePerimeter::run_step(orchestrator, print);
    });
    if (stop_after(STEP_PERIMETER, until)) return;
    
    run_step_if_requested(print, STEP_POST_PERIMETER, [&] {
        begin_step(print, STEP_POST_PERIMETER, L("Post-processing perimeters"));
        StepPostPerimeterGeneration::prepare(print);
        StepPostPerimeterGeneration::run_step(orchestrator, print);
    });
    if (stop_after(STEP_POST_PERIMETER, until)) return;

    run_step_if_requested(print, STEP_SURFACE_GENERATION, [&] {
        begin_step(print, STEP_SURFACE_GENERATION, L("Generating surfaces"));
        StepSurfaceGeneration::prepare(print);
#ifdef _DEBUG
        Detail::validate_or_report(StepSurfaceGeneration::validate_pre, print, "Surface-generation pre-step validation");
#endif
        StepSurfaceGeneration::run_step(orchestrator, print);
#ifdef _DEBUG
        Detail::validate_or_report(StepSurfaceGeneration::validate_post, print, "Surface-generation post-step validation");
#endif
    });
    if (stop_after(STEP_SURFACE_GENERATION, until)) return;

    run_step_if_requested(print, STEP_PRE_INFILL, [&] {
        begin_step(print, STEP_PRE_INFILL, L("Preparing infill"));
        StepPrepareInfill::prepare(print);
        StepPrepareInfill::run_step(orchestrator, print);
    });
    if (stop_after(STEP_PRE_INFILL, until)) return;

    run_step_if_requested(print, STEP_INFILL_GROUP, [&] {
        begin_step(print, STEP_INFILL_GROUP, L("Grouping infill regions"));
        StepGroupInfillRegions::prepare(print);
        StepGroupInfillRegions::run_step(orchestrator, print);
    });
    if (stop_after(STEP_INFILL_GROUP, until)) return;

    run_step_if_requested(print, STEP_INFILL, [&] {
        begin_step(print, STEP_INFILL, L("Generating infill"));
        StepGenerateInfill::prepare(print);
        StepGenerateInfill::run_step(orchestrator, print);
    });
    if (stop_after(STEP_INFILL, until)) return;

    run_step_if_requested(print, STEP_POST_INFILL, [&] {
        begin_step(print, STEP_POST_INFILL, L("Post-processing infill"));
        StepPostInfillGeneration::prepare(print);
        StepPostInfillGeneration::run_step(orchestrator, print);
    });
    if (stop_after(STEP_POST_INFILL, until)) return;

    run_step_if_requested(print, STEP_SUPPORT, [&] {
        begin_step(print, STEP_SUPPORT, L("Generating support material"));
        StepGenerateSupport::prepare(print);
        StepGenerateSupport::run_step(orchestrator, print);
    });
    if (stop_after(STEP_SUPPORT, until)) return;

    run_step_if_requested(print, STEP_SUPPORT_TOOLPATH, [&] {
        begin_step(print, STEP_SUPPORT_TOOLPATH, L("Generating support toolpaths"));
        StepSupportToolpath::prepare(print);
        StepSupportToolpath::run_step(orchestrator, print);
    });
    if (stop_after(STEP_SUPPORT_TOOLPATH, until)) return;

    run_step_if_requested(print, STEP_SKIRT_BRIM, [&] {
        begin_step(print, STEP_SKIRT_BRIM, L("Generating skirt and brim"));
        StepSkirtBrim::prepare(print);
        StepSkirtBrim::run_step(orchestrator, print);
    });
    if (stop_after(STEP_SKIRT_BRIM, until)) return;

    run_step_if_requested(print, STEP_FINALIZE_SLICING, [&] {
        begin_step(print, STEP_FINALIZE_SLICING, L("Finalizing slicing"));
        StepFinalizeSlicing::prepare(print);
        StepFinalizeSlicing::run_step(orchestrator, print);
    });
    if (stop_after(STEP_FINALIZE_SLICING, until)) return;

    run_step_if_requested(print, STEP_ORDERING, [&] {
        begin_step(print, STEP_ORDERING, L("Ordering extrusions"));
        StepExtrusionOrdering::prepare(print);
        StepExtrusionOrdering::run_step(orchestrator, print);
        // Checkpoint::Ordering: keep a copy of the freshly ordered plan so a
        // later plan-editor invalidation resumes here instead of re-ordering.
        // A resume from this checkpoint skips STEP_ORDERING, so the retained
        // copy is never replaced by an equivalent one.
        print.save_ordering_checkpoint();
    });
    if (stop_after(STEP_ORDERING, until)) return;

    run_step_if_requested(print, STEP_WIPETOWER, [&] {
        begin_step(print, STEP_WIPETOWER, L("Generating wipe tower"));
        StepGenerateWipeTower::prepare(print);
        StepGenerateWipeTower::run_step(orchestrator, print);
    });
    if (stop_after(STEP_WIPETOWER, until)) return;

    run_step_if_requested(print, STEP_SUPPORT_SPOT, [&] {
        begin_step(print, STEP_SUPPORT_SPOT, L("Detecting support spots"));
        StepDetectSupportSpots::prepare(print);
        StepDetectSupportSpots::run_step(orchestrator, print);
    });
    if (stop_after(STEP_SUPPORT_SPOT, until)) return;

    run_step_if_requested(print, STEP_LAYER_EXTRUSION_EDIT, [&] {
        begin_step(print, STEP_LAYER_EXTRUSION_EDIT, L("Editing layers extrusions"));
        StepLayerExtrusionEdition::prepare(print);
        StepLayerExtrusionEdition::run_step(orchestrator, print);
    });
    if (stop_after(STEP_LAYER_EXTRUSION_EDIT, until)) return;

    run_step_if_requested(print, STEP_LAYER_STICHING, [&] {
        begin_step(print, STEP_LAYER_STICHING, L("Stitching layers"));
        StepLayerStiching::prepare(print);
        StepLayerStiching::run_step(orchestrator, print);
    });
    if (stop_after(STEP_LAYER_STICHING, until)) return;

    run_step_if_requested(print, STEP_EXTRUSION_EDIT, [&] {
        begin_step(print, STEP_EXTRUSION_EDIT, L("Editing extrusions"));
        StepExtrusionEdition::prepare(print);
        StepExtrusionEdition::run_step(orchestrator, print);
    });
    if (stop_after(STEP_EXTRUSION_EDIT, until)) return;

    run_step_if_requested(print, STEP_EXTRUSION_SIMPLIFICATION, [&] {
        begin_step(print, STEP_EXTRUSION_SIMPLIFICATION, L("Simplifying extrusions"));
        StepExtrusionSimplification::prepare(print);
        StepExtrusionSimplification::run_step(orchestrator, print);
    });
    if (stop_after(STEP_EXTRUSION_SIMPLIFICATION, until)) return;

    run_step_if_requested(print, STEP_GCODE, [&] {
        begin_step(print, STEP_GCODE, L("Generating G-code"));
        StepGenerateGcode::prepare(print);
        const std::string selected_reader = StepGenerateGcode::run_step(
            orchestrator, print, path, thumbnail_renderer);
        // The writer published the source; associate its already-selected
        // reader so an explicit CLI reread needs no new firmware session.
        print.set_gcode_reader_source_path(print.gcode_reader_source_path(), selected_reader);
        if (reader_plugin_id != nullptr)
            *reader_plugin_id = selected_reader;

        if (gcode_result != nullptr) {
            try {
                StepReadGeneratedGcode::run(
                    orchestrator, print, selected_reader, *gcode_result);
                StepReadGeneratedGcode::fill_print_statistics(
                    *gcode_result, print.config(), print.print_statistics());
            } catch (const CanceledException &) {
                // Preview generation is optional when it fails, but an
                // explicit slicing cancellation must still unwind the whole
                // pipeline with the standard host exception.
                throw;
            } catch (const std::exception &error) {
                gcode_result->reset();
                gcode_result->reader_plugin_id = selected_reader;
                const std::string message =
                    "G-code preview is unavailable because the generated text could not be read: " +
                    std::string(error.what());
                BOOST_LOG_TRIVIAL(warning) << message;
                Print::StatusMonitor status(print);
                status.active_step_add_warning(
                    PrintStateBase::WarningLevel::NON_CRITICAL, message);
            } catch (...) {
                gcode_result->reset();
                gcode_result->reader_plugin_id = selected_reader;
                const std::string message =
                    "G-code preview is unavailable because the generated text reader failed unexpectedly.";
                BOOST_LOG_TRIVIAL(warning) << message;
                Print::StatusMonitor status(print);
                status.active_step_add_warning(
                    PrintStateBase::WarningLevel::NON_CRITICAL, message);
            }
        }
        // The Reader and firmware are services nested under the G-code writer.
        // Complete the writer's single top-level unit only after those services
        // and the optional viewer result have also finished.
        Plugin *writer = selected_or_active_plugin_for_step(
            orchestrator, STEP_GCODE, &print.full_print_config());
        if (writer != nullptr)
            print.pipeline_progress().complete_plugin(STEP_GCODE, writer->get_id());
    });
    if (stop_after(STEP_GCODE, until)) return;
}

} // namespace

void StepPipeline::clear_checkpoint_outputs(Print &print, const Checkpoint checkpoint)
{
    if (checkpoint == Checkpoint::GCode)
        return;
    StepGenerateGcode::clear(print);
    if (checkpoint <= Checkpoint::Print)
        StepExtrusionOrdering::clear(print);
    // The edited plan is stale; the resume clones a new one from the kept Ordering copy.
    if (checkpoint == Checkpoint::Ordering)
        print.release_live_printing_plan();
    // At None the layers themselves will be deleted, so visiting their islands would duplicate work.
    if (checkpoint == Checkpoint::Slicing) {
        StepGeneratePerimeter::clear(print);
        StepSupportDemand::clear(print);
    }
    // Auxiliary layers may borrow main-layer data and must disappear before those main layers.
    if (checkpoint <= Checkpoint::Slicing)
        StepPrepareForPeriemters::clear(print);
    if (checkpoint == Checkpoint::None) {
        StepSlicing::clear(print);
        StepLayerHeightGeneration::clear(print);
    }
}

void StepPipeline::run_slice(Orchestrator &orchestrator, Print &print)
{
#ifdef _DEBUG
    assert(validate_execution_order_against_dependencies());
#endif

    const std::string path;
    orchestrator.reset_plugin_cancel();
    const Checkpoint checkpoint = print.current_checkpoint();
    if (checkpoint == Checkpoint::None ||
        checkpoint == Checkpoint::Slicing ||
        checkpoint == Checkpoint::Print) {
        print.prepare_checkpoint_resume();
    }
    // run_slice() starts a new end-to-end execution. run_gcode() will retain
    // this table while it is incomplete so slicing and export share one
    // denominator.
    print.pipeline_progress().reset();
    initialize_pipeline_progress(orchestrator, print, STEP_LAYER_HEIGHT);

    run_layer_height_generation(orchestrator, print, path);
    run_slicing(orchestrator, print, path);
    run_post_slicing(orchestrator, print, path);
    run_remaining_steps(orchestrator, print, path, STEP_FINALIZE_SLICING);
}

std::string StepPipeline::run_gcode(Orchestrator &orchestrator, Print &print,
                             const std::string &path,
                             const raw_gcode_thumbnail_renderer *thumbnail_renderer,
                             GCodeProcessorResult *result)
{
#ifdef _DEBUG
    assert(validate_execution_order_against_dependencies());
#endif

    // Reject before cleanup can delete layers whose replacement this export entry point cannot build.
    if (print.current_checkpoint() == Checkpoint::None)
        throw SlicingError("G-code export requires slicing first: run_slice() must complete before run_gcode().");
    orchestrator.reset_plugin_cancel();
    // A direct G-code invocation does not execute the three slice-prefix
    // steps. A continuation after run_slice() keeps the complete plan already
    // stored in Print and therefore preserves its original denominator.
    // Resume from the latest valid checkpoint, restoring the ordered plan copy
    // when plan editors are the first steps to replay.
    print.prepare_checkpoint_resume();
    initialize_pipeline_progress(orchestrator, print, checkpoint_resume_step(Checkpoint::Slicing));
    std::string reader_plugin_id;
    run_remaining_steps(orchestrator, print, path, STEP_GCODE, thumbnail_renderer,
                        &reader_plugin_id, result);
    return reader_plugin_id;
}

void StepPipeline::run(Orchestrator &orchestrator, Print &print, const std::string &path)
{
    StepPipeline::run_slice(orchestrator, print);
    if (!path.empty())
        StepPipeline::run_gcode(orchestrator, print, path);
}


} // namespace Slic3r::Steps
