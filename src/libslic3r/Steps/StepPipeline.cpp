///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "StepPipeline.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/internal/LayerAccess.hpp"
#include "libslic3r/Api/internal/LayerRegionAccess.hpp"
#include "libslic3r/Api/internal/PrintObjectAccess.hpp"
#include "libslic3r/Config/ConfigDef.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
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
#include "libslic3r/Steps/StepExport.hpp"
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

#include <algorithm>
#include <cassert>
#include <cctype>
#include <map>
#include <set>
#include <string>
#include <stdexcept>
#include <vector>

/*
This file coordinates slicing stages and selects the plugins that perform them.
A pipeline is an ordered series of processing stages, called steps. Print
holds one slicing job's data; Orchestrator is the slicer application's manager
of plugins, components that implement jobs within those steps.

StepPipeline::run_slice() runs geometry processing; run_gcode() produces the
printer-command file, and run() combines both. Checkpoint identifies a saved
boundary between stages. clear_checkpoint_outputs() discards results after
that boundary when changed settings require recomputation. Producer steps
create results; service steps describe callable jobs used by those producers.

An exclusive group is a set of alternative plugins for one step. Its selector
is a setting naming a member. StepExclusiveGroup owns the text and enum (list
of choices) used to describe that setting; set_enum_plugins() fills choices
ordered by group preference, the rank among alternatives, then id.
plugin_exclusive_group_definition() uses descriptions already retained by
Orchestrator. selected_plugin_from_group() reads settings, tries automatic
key/value conditions when requested, then resolves a selected or default member.
Required-setting checks ensure selected members can read their declared keys.

Output reading is separate from slicing. Requested preview reads also fill
Print statistics measured from generated commands. Command-line output reading
occurs only when explicitly requested.

run_slice() and run_gcode() reset stop state for their supplied Print.
When that Print has a registered RunContext, the reset affects only that execution.
Without one, the orchestrator uses its shared stop state.

Operation notes:
- run_step_if_requested
- run_step_if_requested
- selected_active_plugin_from_group
- selected_plugin_from_group

Representative direct calls (branches are not a timeline):

    StepPipeline::run()
        |-- StepPipeline::run_slice()
        `-- StepPipeline::run_gcode()

    StepPipeline::run_slice()
        |-- print.prepare_checkpoint_resume()
        |-- run_layer_height_generation()
        |-- run_slicing()
        |-- run_post_slicing()
        `-- run_remaining_steps()

    StepPipeline::run_gcode()
        |-- print.prepare_checkpoint_resume()
        |-- initialize_pipeline_progress()
        `-- run_remaining_steps()
*/

namespace Slic3r::Steps {

// Install copied host traits through owner modules without changing print data; modules must not recurse into Orchestrator::instance().
// Called during host construction; registration conflicts assert, allocation errors propagate, and provider ownership is unchanged.
void register_builtin_step_selectors(Orchestrator &orchestrator);
// Build owned descriptions for borrowed active providers without registering settings; omit empty or hidden singleton groups.
// Provider pointers stay borrowed, and allocation errors propagate.
std::vector<StepExclusivePluginGroup> active_exclusive_plugin_groups(Orchestrator &orchestrator);
// Resolve borrowed active providers/settings through whole-step identities or named-group descriptions, including singleton groups.
// Returned pointers stay host-owned, settings/print data are unchanged, and allocation failures propagate.
std::vector<Plugin *> selected_or_active_plugins_for_step(Orchestrator &orchestrator, slicing_step_t step, const ConfigBase *config);
// Resolve each borrowed object's effective settings, then group valid object indices by active provider in execution order.
// Return owned index vectors with host-owned provider pointers; print/configuration are unchanged and allocation failures propagate.
std::vector<ObjectStepPlugin> selected_plugins_per_object(Orchestrator &orchestrator, slicing_step_t step, const Print &print);
// Resolve only active members of the requested group; absent groups return null and a sole member is returned directly.
// With several members, read the stored choice from the whole-step selector when present, otherwise the group's own option.
// Providers stay host-owned, settings are unchanged, and allocation failures propagate.
Plugin *selected_active_plugin_from_group(Orchestrator &orchestrator, slicing_step_t step, const std::string &exclusive_group, const ConfigBase *config);
// Resolve by serialized enum id because activation can reorder entries; group, configuration and candidates stay borrowed and unchanged.
// Empty lists return null; missing configuration uses the group-priority default, and absent options use declared/automatic/default choices.
// Try automatic conditions in registration order, ignoring one disabled marker; the first matching condition naming an active member selects it, otherwise use the default.
// Singleton non-auto ids are bypassed unless read_with_one_provider is set.
// Unknown ids return null for strict selection, otherwise the active default; out-of-table enums serialize empty, and allocation/serialization failures propagate.
Plugin *selected_plugin_from_group(const Orchestrator &orchestrator, const StepExclusiveGroup &group, const std::vector<Plugin *> &plugins,
                                   const ConfigBase *config);
// Borrow the supplied candidate with smallest group priority, then smallest id; an empty list returns null.
// Candidates must be nonnull; activation is not checked.
Plugin *default_provider_from_group(const std::vector<Plugin *> &plugins);
// Recognize built-in and registered dynamic stages, including seam placement, but not NONE or ANY; the borrowed application is unchanged.
bool selector_step_is_registered(const Orchestrator &orchestrator, slicing_step_t step);
// Return an owned default key from borrowed stage/group names without reading descriptions or changing registrations.
// Unknown stages keep their numeric id, and allocation failures propagate.
std::string generated_plugin_exclusive_group_option_key(const Orchestrator &orchestrator, slicing_step_t step, const std::string &group_id);
// Borrow setting data and member lookup, returning the first owned support error or nullopt; required defaults must match their value types.
// Inputs are not retained or changed; lookup, comparison and formatting failures propagate.
std::optional<std::string> unsupported_active_setting(const Orchestrator &orchestrator, const ConfigDef &definitions, const ConfigBase &config,
    const std::function<const Plugin *(const std::string &group_id)> &selected_plugin_of_group);
// Resolve the replay checkpoint of a key from the print configuration catalogue (see the declaration).
// This only reads the catalogue: the active plugin readers are looked up here because
// tighten_used_config_key_invalidations() cannot record them for a bookkeeping key (it keeps
// STEP_ANY, which is also the catalogue default), and it would have to rewrite the definition.
std::optional<Checkpoint> config_option_replay_checkpoint(const std::string &key)
{
    const ConfigOptionDef *definition = PrintConfigDef::instance().get(key);
    if (definition == nullptr)
        return Checkpoint::None;
    slicing_step_t step = definition->invalidates_step;
    if (step == STEP_NONE)
        return std::nullopt;
    if (definition->nature == ConfigOptionNature::Bookkeeping) {
        if (step == STEP_ANY)
            step = STEP_NONE;
        // Keep the earliest step among the active plugins that read the key; STEP_NONE means none found yet.
        for (const Plugin *plugin : Orchestrator::instance().active_plugins()) {
            for (const Plugin::UsedConfigKey &used_key : plugin->get_used_config_keys()) {
                if (used_key.key != key)
                    continue;
                const slicing_step_t owner = plan_owner_step(*plugin);
                if (owner != STEP_NONE && (step == STEP_NONE || executes_before(owner, step)))
                    step = owner;
            }
        }
    }
    return step == STEP_NONE ? std::nullopt : std::optional<Checkpoint>(checkpoint_before_step(step));
}

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
        STEP_EXPORT,
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
    // The export is the only step after the reference: it resumes from the reference itself.
    if (step_index < execution_index(checkpoint_resume_step(Checkpoint::GCode)))
        return Checkpoint::Plan;
    return Checkpoint::GCode;
}

slicing_step_t checkpoint_resume_step(const Checkpoint checkpoint)
{
    switch (checkpoint) {
    case Checkpoint::None:     return STEP_LAYER_HEIGHT;
    case Checkpoint::Slicing:  return STEP_SUPPORT_DEMAND;
    case Checkpoint::Print:    return STEP_ORDERING;
    case Checkpoint::Ordering: return STEP_WIPETOWER;
    case Checkpoint::Plan:     return STEP_GCODE;
    case Checkpoint::GCode:    return STEP_EXPORT;
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
// Copy the named group's selector traits, falling back to the whole step and then default traits.
StepSelectorTraits selector_traits_of(const Orchestrator &orchestrator, slicing_step_t step, const std::string &group_id);
// Return the declared invalidation step, then the registered dynamic or built-in step, or STEP_ANY if unknown.
slicing_step_t selector_invalidation_step(const Orchestrator &orchestrator, slicing_step_t step, const StepSelectorTraits &traits);
// Build UI layout text from the group's placement and line label, using Notes when no placement is declared.
std::string plugin_group_ui_fragment(const StepExclusiveGroup &group, const std::string &line_label);
// Build a selector description; its raw option pointers refer to its own strings and need refreshing after moves.
StepExclusiveGroup make_exclusive_group(slicing_step_t step, const std::string &group_id, const std::string &key,
                                       const std::string &label, const std::string &line_label, const std::string &tooltip,
                                       StepSelectorTraits traits, slicing_step_t invalidates_step);
// Copy setting rules and retained group text into an owned description without creating a setting.
// Raw pointers borrow returned storage and require refresh after moves; allocation errors propagate.
StepExclusiveGroup make_plugin_exclusive_group(const Orchestrator &orchestrator, slicing_step_t step,
                                               const std::string &group_id, const Plugin &first_plugin);
// Copy retained group text and catalogue name, preserving application line text, then rebuild window-layout instructions.
// No description is chosen again; the group owns copied strings, settings are unchanged, and allocations may throw.
void apply_plugin_group_text(const Orchestrator &orchestrator, StepExclusiveGroup &group, const std::string &line_label);
// Use the borrowed group's explicit key unless it belongs to the entire stage, which needs a separate generated group key.
// Return an owned key without changing registrations; allocation failures propagate.
std::string option_key_for_plugin_group(const Orchestrator &orchestrator, slicing_step_t step, const std::string &group_id);

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
    const StepSelectorTraits *traits = orchestrator.step_selector(step, group_id);
    const std::map<slicing_step_t, StepExclusiveIdentity> &whole_steps = get_exclusive_steps();
    const std::map<slicing_step_t, StepExclusiveIdentity>::const_iterator whole = whole_steps.find(step);
    if (traits != nullptr && !traits->option_key.empty() &&
        (whole == whole_steps.end() || traits->option_key != whole->second.key)) return traits->option_key;
    return generated_plugin_exclusive_group_option_key(orchestrator, step, group_id);
}

// Return whether the supplied ID is a recognized built-in step or service; unknown IDs return false without mutation.
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
    case STEP_EXPORT:
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

StepSelectorTraits selector_traits_of(const Orchestrator &orchestrator, slicing_step_t step, const std::string &group_id)
{
    const StepSelectorTraits *declared = orchestrator.step_selector(step, group_id);
    return declared != nullptr ? *declared : StepSelectorTraits();
}

slicing_step_t selector_invalidation_step(const Orchestrator &orchestrator, slicing_step_t step, const StepSelectorTraits &traits)
{
    if (traits.invalidates_step) return *traits.invalidates_step;
    const Orchestrator::DynamicStepInfo *info = orchestrator.step_info(step);
    if (info != nullptr) return info->invalidates_step;
    return is_known_builtin_step(step) ? step : STEP_ANY;
}

std::string plugin_group_ui_fragment(const StepExclusiveGroup &group, const std::string &line_label)
{
    if (group.traits.ui_page.empty()) return exclusive_group_ui_fragment(group.option_key_storage, line_label);
    return "page:" + group.traits.ui_page + "\n" + "group:" + group.traits.ui_group + "\n" +
           "line:" + line_label + "\n" + "setting:" + group.option_key_storage + "\n" + "end_line\n";
}

/*
Populate one exclusive group with owned display strings, borrowed raw-option pointers, and a UI
fragment; the raw definition borrows strings stored by the group. Mark plugin IDs for pruning during
finalization and keep STEP_EXPORT's selector at STEP_NONE because it changes only output format.
The group must outlive registration, while option finalization replaces borrowed enum storage only
before selector-bearing configs are created. Allocation failures propagate; no option definition is
registered here.
*/
StepExclusiveGroup make_exclusive_group(slicing_step_t step,
                                        const std::string &group_id,
                                        const std::string &key,
                                        const std::string &label,
                                        const std::string &line_label,
                                        const std::string &tooltip,
                                        StepSelectorTraits traits,
                                        slicing_step_t invalidates_step)
{
    StepExclusiveGroup group = {};
    group.group_id = group_id;
    group.option_key_storage = key;
    group.label_storage = label;
    group.tooltip_storage = tooltip;
    group.step = step;
    group.traits = std::move(traits);

    raw_config_option_def def = raw_config_option_def_init();
    def.type = RAW_CO_ENUM;
    def.gui_type = RAW_GUI_TYPE_SELECT_CLOSE;
    def.container_type = group.traits.container_type;
    // Request removal of inactive plugin IDs from this selector during finalization.
    def.contains_plugin_id = 1;
    def.option_preset_type = group.traits.option_preset_type;
    def.printer_technology = RAW_PT_FFF;
    def.category = group.traits.category;
    // Use the caller's resolved invalidation step so all selector descriptions follow the same rules.
    def.invalidates_step = invalidates_step;
    def.mode = RAW_CONFIG_OPTION_MODE_ADV_EXP | RAW_CONFIG_OPTION_MODE_SUSI;
    group.option_def = def;

    group.ui_fragment = plugin_group_ui_fragment(group, line_label);
    group.refresh_storage_pointers();
    return group;
}

StepExclusiveGroup make_plugin_exclusive_group(const Orchestrator &orchestrator,
                                               slicing_step_t step,
                                               const std::string &group_id,
                                               const Plugin &first_plugin)
{
    const std::string key = plugin_exclusive_group_option_key(
        orchestrator, step, group_id);
    (void)first_plugin;
    const Orchestrator::ExclusiveGroupDefinition definition = orchestrator.exclusive_group_definition_or_default(step, group_id);
    const StepSelectorTraits &traits = definition.traits;
    StepExclusiveGroup group = make_exclusive_group(step,
                                                    group_id,
                                                    key,
                                                    definition.label,
                                                    definition.label,
                                                    definition.tooltip,
                                                    traits,
                                                    selector_invalidation_step(orchestrator, step, traits));

    apply_plugin_group_text(orchestrator, group, definition.label);
    return group;
}

void apply_plugin_group_text(const Orchestrator &orchestrator, StepExclusiveGroup &group, const std::string &line_label)
{
    const Orchestrator::ExclusiveGroupDefinition definition = orchestrator.exclusive_group_definition_or_default(group.step, group.group_id);
    group.label_storage = definition.label;
    group.tooltip_storage = definition.tooltip;
    group.translation_domain_storage = definition.translation_domain;
    group.member_declared = !definition.plugin_id.empty();
    if (group.traits.auto_value)
        group.tooltip_storage += " Automatic selection uses the first matching active preference, otherwise the active group default.";
    const std::string &host_line = definition.line_label.empty() ? line_label : definition.line_label;
    group.ui_fragment = plugin_group_ui_fragment(group, group.member_declared ? std::string() : host_line);
    group.refresh_storage_pointers();
}

} // namespace

StepExclusiveGroup make_exclusive_step_group(const Orchestrator &orchestrator, slicing_step_t step,
                                             const StepExclusiveIdentity &identity)
{
    const StepSelectorTraits traits = selector_traits_of(orchestrator, step, identity.key);
    return make_exclusive_group(step, identity.key, identity.key, identity.label, identity.line_label,
                                "Choose which active plugin owns this exclusive slicing step.", traits,
                                selector_invalidation_step(orchestrator, step, traits));
}

void register_builtin_step_selectors(Orchestrator &orchestrator)
{
    StepLayerHeightGeneration::register_selector_traits(orchestrator);
    StepSlicing::register_selector_traits(orchestrator);
    StepPrepareForPeriemters::register_selector_traits(orchestrator);
    StepGeneratePerimeter::register_selector_traits(orchestrator);
    StepPostPerimeterGeneration::register_selector_traits(orchestrator);
    StepPrepareInfill::register_selector_traits(orchestrator);
    StepGroupInfillRegions::register_selector_traits(orchestrator);
    StepGenerateInfill::register_selector_traits(orchestrator);
    StepPostInfillGeneration::register_selector_traits(orchestrator);
    StepSupportDemand::register_selector_traits(orchestrator);
    StepGenerateSupport::register_selector_traits(orchestrator);
    StepSupportToolpath::register_selector_traits(orchestrator);
    StepDetectSupportSpots::register_selector_traits(orchestrator);
    StepExtrusionOrdering::register_selector_traits(orchestrator);
    StepGenerateWipeTower::register_selector_traits(orchestrator);
    StepLayerStiching::register_selector_traits(orchestrator);
    StepGenerateGcode::register_selector_traits(orchestrator);
    StepExport::register_selector_traits(orchestrator);
    StepReadGeneratedGcode::register_selector_traits(orchestrator);
    for (const std::pair<const slicing_step_t, StepExclusiveIdentity> &entry : get_exclusive_steps()) {
        if (orchestrator.exclusive_group_definition(entry.first, entry.second.key) == nullptr) {
            Orchestrator::ExclusiveGroupDefinition definition;
            definition.traits = selector_traits_of(orchestrator, entry.first, entry.second.key);
            definition.traits.exclusive_group = entry.second.key;
            definition.traits.option_key = entry.second.key;
            definition.label = entry.second.label;
            definition.tooltip = "Choose which active plugin owns this exclusive slicing step.";
            definition.translation_domain = "Slic3r";
            const int32_t result = orchestrator.register_builtin_exclusive_group(entry.first, std::move(definition));
            assert(result == 1);
            (void)result;
        }
        orchestrator.register_selector_option(entry.second.key,
            {Orchestrator::SelectorIdentityKind::WholeStep, entry.first, {}}, false);
    }
}

bool selector_step_is_registered(const Orchestrator &orchestrator, slicing_step_t step)
{
    return step != STEP_NONE && step != STEP_ANY &&
        (step == SEAM_PLACER || is_known_builtin_step(step) || orchestrator.step_info(step) != nullptr);
}

std::string generated_plugin_exclusive_group_option_key(const Orchestrator &orchestrator,
                                                        slicing_step_t step, const std::string &group_id)
{
    const Orchestrator::DynamicStepInfo *info = orchestrator.step_info(step);
    const std::string identity = info != nullptr ? sanitized_config_key_part(info->name) : std::to_string(int(step));
    return "exclusive_group_" + identity + "_" + sanitized_config_key_part(group_id) + "_plugin";
}

Plugin *default_provider_from_group(const std::vector<Plugin *> &plugins)
{
    Plugin *best = nullptr;
    for (Plugin *plugin : plugins)
        if (best == nullptr || plugin->get_exclusive_group_priority() < best->get_exclusive_group_priority() ||
            (plugin->get_exclusive_group_priority() == best->get_exclusive_group_priority() && plugin->get_id() < best->get_id()))
            best = plugin;
    return best;
}

StepExclusiveGroup plugin_exclusive_group_definition(const Orchestrator &orchestrator,
                                                      slicing_step_t step,
                                                      const std::string &group_id,
                                                      const Plugin &first_plugin)
{
    return make_plugin_exclusive_group(orchestrator, step, group_id, first_plugin);
}

std::string plugin_exclusive_group_option_key(const Orchestrator &orchestrator,
                                              const slicing_step_t step,
                                              const std::string &group_id)
{
    return option_key_for_plugin_group(orchestrator, step, group_id);
}

// Rebuild option-definition pointers into this group's owned strings and enum arrays, retaining the fixed PNG or simple-exporter defaults; no storage is transferred and allocation failures propagate.
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
    option_def.translation_domain = translation_domain_storage.empty() ? nullptr : translation_domain_storage.c_str();

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
    option_def.default_serialized_value = traits.auto_value ? SLIC3R_SELECTOR_AUTO_VALUE : default_provider_id.c_str();
}

void StepExclusiveGroup::set_enum_plugins(const Orchestrator &orchestrator,
                                         const std::vector<std::pair<std::string, std::string>> &plugin_ids_and_labels,
                                         const std::string &preferred_id)
{
    std::vector<std::pair<std::string, std::string>> ordered = plugin_ids_and_labels;
    std::sort(ordered.begin(), ordered.end(), [&orchestrator](const std::pair<std::string, std::string> &lhs,
                                                           const std::pair<std::string, std::string> &rhs) {
        const Plugin *left = orchestrator.get_plugin(lhs.first);
        const Plugin *right = orchestrator.get_plugin(rhs.first);
        const int32_t left_priority = left != nullptr ? left->get_exclusive_group_priority() : 100;
        const int32_t right_priority = right != nullptr ? right->get_exclusive_group_priority() : 100;
        return left_priority != right_priority ? left_priority < right_priority : lhs.first < rhs.first;
    });
    default_provider_id = preferred_id;
    enum_values.clear();
    enum_labels.clear();
    enum_values.reserve(plugin_ids_and_labels.size());
    enum_labels.reserve(plugin_ids_and_labels.size());
    if (traits.auto_value) {
        enum_values.emplace_back(SLIC3R_SELECTOR_AUTO_VALUE);
        enum_labels.emplace_back("Automatic");
    }
    for (const std::pair<std::string, std::string> &plugin : ordered) {
        enum_values.emplace_back(plugin.first);
        enum_labels.emplace_back(plugin.second.empty() ? plugin.first : plugin.second);
    }
    refresh_storage_pointers();
}

const std::map<slicing_step_t, StepExclusiveIdentity> &get_exclusive_steps()
{
    static const std::map<slicing_step_t, StepExclusiveIdentity> s_groups = {
        {STEP_LAYER_HEIGHT,   {"step_layer_height_plugin",   "Layer height plugin",    "Layer height step plugin"}},
        {STEP_SLICING,        {"step_slicing_plugin",        "Slicing plugin",         "Slicing step plugin"}},
        {STEP_PERIMETER,      {"step_perimeter_plugin",      "Perimeter plugin",       "Perimeter step plugin"}},
        {STEP_INFILL_GROUP,   {"step_infill_group_plugin",   "Infill grouping plugin", "Infill grouping step plugin"}},
        {STEP_WIPETOWER,      {"step_wipetower_plugin",      "Wipe tower plugin",      "Wipe tower step plugin"}},
        {STEP_LAYER_STICHING, {"step_layer_stiching_plugin", "Layer stitching plugin", "Layer stitching step plugin"}},
        {GCODE_FIRMWARE,      {"gcode_flavor",               "G-code flavor",          "G-code flavor"}},
        {STEP_EXPORT,        {"gcode_exporter_plugin",      "G-code exporter",        "G-code exporter"}}
    };
    return s_groups;
}

// Build owned group snapshots with borrowed provider pointers, retaining the G-code exporter selector with one provider; empty groups are omitted and allocation failures propagate.
std::vector<StepExclusivePluginGroup> active_exclusive_plugin_groups(Orchestrator &orchestrator)
{
    std::vector<StepExclusivePluginGroup> out;
    const std::map<slicing_step_t, StepExclusiveIdentity> &exclusive_steps = get_exclusive_steps();

    // A whole-step selector covers every provider of that step, regardless of each provider's own group name.
    for (const std::pair<const slicing_step_t, StepExclusiveIdentity> &entry : exclusive_steps) {
        std::vector<Plugin *> active_plugins = orchestrator.get_active_plugins_for_step(entry.first);
        StepExclusiveGroup definition = make_exclusive_step_group(orchestrator, entry.first, entry.second);
        // Some declarations display a selector even when only one provider is available.
        if (active_plugins.empty() || (active_plugins.size() == 1 && !definition.traits.show_with_one_provider))
            continue;

        StepExclusivePluginGroup plugin_group;
        plugin_group.whole_step = true;
        plugin_group.group = std::move(definition);
        plugin_group.group.refresh_storage_pointers();
        apply_plugin_group_text(orchestrator, plugin_group.group, entry.second.line_label);
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
            const StepSelectorTraits traits = selector_traits_of(orchestrator, step_plugins.first, entry.first);
            if (entry.second.empty() || (entry.second.size() == 1 && !traits.show_with_one_provider))
                continue;

            StepExclusivePluginGroup plugin_group;
            plugin_group.group = make_plugin_exclusive_group(
                orchestrator, step_plugins.first, entry.first, *entry.second.front());
            plugin_group.plugins = entry.second;
            out.push_back(std::move(plugin_group));
        }
    }

    return out;
}

Plugin *selected_plugin_from_group(const Orchestrator &orchestrator, const StepExclusiveGroup &group,
                                   const std::vector<Plugin *> &plugins,
                                   const ConfigBase *config)
{
    if (plugins.empty())
        return nullptr;
    if (config == nullptr && group.traits.strict_selection && group.traits.value_when_absent) {
        for (Plugin *plugin : plugins)
            if (plugin->get_id() == *group.traits.value_when_absent) return plugin;
        return nullptr;
    }
    if (config == nullptr)
        return default_provider_from_group(plugins);

    const ConfigOption *option = config->option(group.option_def.opt_key);
    const std::optional<std::string> absent = option == nullptr ? group.traits.value_when_absent : std::nullopt;
    if (!group.traits.auto_value && plugins.size() == 1 && !group.traits.read_with_one_provider) return plugins.front();
    const std::string selected_id = option != nullptr ? option->serialize() : absent ? *absent : std::string();
    if (group.traits.auto_value && ((option == nullptr && !absent) || selected_id == SLIC3R_SELECTOR_AUTO_VALUE)) {
        for (const SelectorAutoChoice &choice : orchestrator.selector_auto_choices()) {
            if (choice.step != group.step || choice.exclusive_group != group.group_id) continue;
            const ConfigOption *condition = config->option(choice.condition_key);
            if (condition == nullptr) continue;
            std::string value = condition->serialize();
            if (!value.empty() && value.front() == '!') value.erase(value.begin());
            if (value != choice.condition_value) continue;
            for (Plugin *plugin : plugins)
                if (plugin->get_id() == choice.provider_id) return plugin;
        }
        return default_provider_from_group(plugins);
    }
    if (plugins.size() == 1 && !group.traits.read_with_one_provider) return plugins.front();
    if (option == nullptr && !absent) return default_provider_from_group(plugins);
    for (Plugin *plugin : plugins)
        if (plugin->get_id() == selected_id)
            return plugin;

    if (group.traits.strict_selection)
        return nullptr;
    return default_provider_from_group(plugins);
}

std::vector<Plugin *> selected_or_active_plugins_for_step(Orchestrator &orchestrator,
                                                          slicing_step_t step,
                                                          const ConfigBase *config)
{
    std::vector<Plugin *> active_plugins = orchestrator.get_active_plugins_for_step(step);
    if (active_plugins.empty())
        return active_plugins;

    const std::map<slicing_step_t, StepExclusiveIdentity> &exclusive_steps = get_exclusive_steps();
    const std::map<slicing_step_t, StepExclusiveIdentity>::const_iterator exclusive_it = exclusive_steps.find(step);
    if (exclusive_it != exclusive_steps.end()) {
        StepExclusiveGroup group = make_exclusive_step_group(orchestrator, step, exclusive_it->second);
        group.refresh_storage_pointers();
        Plugin *selected = selected_plugin_from_group(orchestrator, group, active_plugins, config);
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
        Plugin *selected = selected_plugin_from_group(orchestrator, group, group_plugins, config);
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

std::vector<ObjectStepPlugin> selected_plugins_per_object(Orchestrator &orchestrator,
                                                          slicing_step_t step,
                                                          const Print &print)
{
    std::vector<std::vector<Plugin *>> selected_by_object;
    selected_by_object.reserve(print.objects().size());
    for (const PrintObject &object : print.objects())
        selected_by_object.push_back(selected_or_active_plugins_for_step(orchestrator, step, &object.config()));

    std::vector<ObjectStepPlugin> plugins;
    for (Plugin *plugin : orchestrator.get_active_plugins_for_step(step)) {
        ObjectStepPlugin entry {plugin, {}};
        for (size_t object_idx = 0; object_idx < selected_by_object.size(); ++object_idx) {
            const std::vector<Plugin *> &selected = selected_by_object[object_idx];
            if (std::find(selected.begin(), selected.end(), plugin) != selected.end())
                entry.object_indices.push_back(object_idx);
        }
        if (!entry.object_indices.empty())
            plugins.push_back(std::move(entry));
    }
    return plugins;
}

Plugin *selected_or_active_plugin_for_step(Orchestrator &orchestrator,
                                           slicing_step_t step,
                                           const ConfigBase *config)
{
    const std::vector<Plugin *> plugins = selected_or_active_plugins_for_step(orchestrator, step, config);
    assert(plugins.size() <= 1);
    return plugins.empty() ? nullptr : plugins.front();
}

// Collect the active providers of `exclusive_group` on `step` (borrowed) and select one with the borrowed config. A built-in
// selector (the entry of get_exclusive_steps() with the same step and group ID, such as the G-code firmware or exporter)
// takes priority and reads its own option key; any other group reads the key derived from its step and ID. Return null
// when the group has no active provider or the group rules reject the configured ID; nothing is mutated.
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

    const std::map<slicing_step_t, StepExclusiveIdentity> &exclusive_steps = get_exclusive_steps();
    const std::map<slicing_step_t, StepExclusiveIdentity>::const_iterator exclusive_it = exclusive_steps.find(step);
    StepExclusiveGroup group = exclusive_it != exclusive_steps.end() ?
        make_exclusive_step_group(orchestrator, step, exclusive_it->second) :
        make_plugin_exclusive_group(orchestrator, step, exclusive_group, *group_plugins.front());
    group.refresh_storage_pointers();
    return selected_plugin_from_group(orchestrator, group, group_plugins, config);
}

std::optional<std::string> unsupported_active_setting(
    const Orchestrator &orchestrator, const ConfigDef &definitions, const ConfigBase &config,
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

        const std::map<slicing_step_t, StepExclusiveIdentity>::const_iterator whole = get_exclusive_steps().find(plugin->get_step());
        const std::string group_id = whole != get_exclusive_steps().end() ? whole->second.key : plugin->get_exclusive_group();
        const Orchestrator::ExclusiveGroupDefinition group_definition =
            orchestrator.exclusive_group_definition_or_default(plugin->get_step(), group_id);
        return format(_u8L("The setting \"%1%\" (%2%) is active, but the plugin \"%3%\" (%4%) selected in the \"%5%\" group does not support it."),
                      key, definition.translated_label(), plugin->get_name(), plugin->get_id(), group_definition.label);
    }
    return std::nullopt;
}

namespace {

// Build the progress plan of the steps from first_step to last_step (both included) that are
// pending, unless an unfinished plan is still active. Steps after last_step are not counted:
// the preview ends at STEP_GCODE, so a pending export must not hold its percentage below 100. The plan starts at
// initial_percent, which the main bar never falls below.
void initialize_pipeline_progress(Orchestrator &orchestrator,
                                  Print &print,
                                  const slicing_step_t first_step,
                                  const slicing_step_t last_step,
                                  const int initial_percent = 0)
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
        if (executes_before(last_step, step))
            break;
        if (!print.should_execute_step(step))
            continue;

        std::vector<std::string> &unit_ids = units_by_step[step];
        // Support toolpath selectors are per object, so the plan lists every provider that any object selects.
        if (step == STEP_SUPPORT_TOOLPATH) {
            for (const ObjectStepPlugin &entry : selected_plugins_per_object(orchestrator, step, print))
                unit_ids.push_back(entry.plugin->get_id());
        } else {
            const std::vector<Plugin *> plugins = selected_or_active_plugins_for_step(
                orchestrator, step, &print.full_print_config());
            for (const Plugin *plugin : plugins)
                if (plugin != nullptr)
                    unit_ids.push_back(plugin->get_id());
        }
        // After its plugins the host owns one more unit. It is completed by the end of the
        // step (complete_step), after the Reader pass, so the percentage stays below 100
        // until the pass is done.
        if (step == STEP_GCODE)
            unit_ids.push_back(k_gcode_read_progress_unit);
        if (unit_ids.empty())
            unit_ids.emplace_back();
    }
    print.pipeline_progress().initialize(units_by_step, initial_percent);
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
    try {
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
    } catch (...) {
        // A failed or cancelled step may have completed only some of its
        // progress units, and the pipeline would then refuse the units of the
        // next attempt as out of order. Drop the whole table: the next
        // run_gcode() rebuilds a fresh one, because it only keeps a table that
        // is active and incomplete.
        print.pipeline_progress().reset();
        throw;
    }
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

// Borrow the host/Print and optional result destinations, executing
// marked steps through until. Export publishes the source/reader pair and only
// reads when a viewer result is requested, filling measured statistics on success.
// No borrowed pointers are retained; pipeline failures/cancellation propagate,
// including a failure to read the generated G-code when a viewer result is requested:
// STEP_GCODE is then not marked as done.
void run_remaining_steps(Orchestrator &orchestrator, Print &print, const std::string &path,
                         slicing_step_t until = STEP_GCODE,
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
            orchestrator, print, path);
        // The STEP_GCODE plugins have all finished and the writer published the
        // source; associate its already-selected reader so an explicit CLI reread needs no new firmware session.
        print.set_gcode_reader_source_path(print.gcode_reader_source_path(), selected_reader);
        if (reader_plugin_id != nullptr)
            *reader_plugin_id = selected_reader;

        // The Reader pass is a host progress unit of this step: announce it, then
        // complete the unit below once the pass is over.
        if (gcode_result != nullptr) {
            begin_step(print, STEP_GCODE, L("Reading the generated G-code"));
            try {
                StepReadGeneratedGcode::run(
                    orchestrator, print, selected_reader, *gcode_result);
                StepReadGeneratedGcode::fill_print_statistics(
                    *gcode_result, print.config(), print.print_statistics());
            } catch (const CanceledException &) {
                throw;
            } catch (...) {
                // The step stays undone, so the caller's result must not keep
                // the reading of an earlier reference.
                gcode_result->reset();
                throw;
            }
        }
    });
    if (stop_after(STEP_GCODE, until)) return;
    // STEP_EXPORT does not run here: it needs a request and a ready reference, and must never trigger an earlier
    // step that an invalidation made pending. StepPipeline::run_export() runs it alone.
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
    orchestrator.reset_plugin_cancel(&print);
    const Checkpoint checkpoint = print.current_checkpoint();
    if (checkpoint == Checkpoint::None ||
        checkpoint == Checkpoint::Slicing ||
        checkpoint == Checkpoint::Print) {
        print.prepare_checkpoint_resume();
    }
    // run_slice() starts a new end-to-end execution. run_gcode() will retain
    // this table while it is incomplete so slicing and the G-code reference of
    // the preview share one denominator. It ends at STEP_GCODE: the export has
    // its own table (see run_export()).
    print.pipeline_progress().reset();
    initialize_pipeline_progress(orchestrator, print, STEP_LAYER_HEIGHT, STEP_GCODE);

    run_layer_height_generation(orchestrator, print, path);
    run_slicing(orchestrator, print, path);
    run_post_slicing(orchestrator, print, path);
    run_remaining_steps(orchestrator, print, path, STEP_FINALIZE_SLICING);
}

std::string StepPipeline::run_gcode(Orchestrator &orchestrator, Print &print,
                             const std::string &path,
                             GCodeProcessorResult *result)
{
#ifdef _DEBUG
    assert(validate_execution_order_against_dependencies());
#endif

    // Reject before cleanup can delete layers whose replacement this export entry point cannot build.
    if (print.current_checkpoint() == Checkpoint::None)
        throw SlicingError("G-code export requires slicing first: run_slice() must complete before run_gcode().");
    orchestrator.reset_plugin_cancel(&print);
    // A direct G-code invocation does not execute the three slice-prefix
    // steps. A continuation after run_slice() keeps the complete plan already
    // stored in Print and therefore preserves its original denominator.
    // Resume from the latest valid checkpoint, restoring the ordered plan copy
    // when plan editors are the first steps to replay.
    print.prepare_checkpoint_resume();
    initialize_pipeline_progress(orchestrator, print, checkpoint_resume_step(Checkpoint::Slicing), STEP_GCODE);
    std::string reader_plugin_id;
    run_remaining_steps(orchestrator, print, path, STEP_GCODE, &reader_plugin_id, result);
    return reader_plugin_id;
}

// Run STEP_EXPORT alone. It deliberately does not go through run_remaining_steps(): that function visits every step and
// runs those that are pending, so a STEP_GCODE invalidated between the request and this call (or between a check made
// by the caller and the loop) would be replayed and the writer would run again to serve an export. Here only the export
// is run; its runner fails when the reference is not ready. Throws when no export was queued. The request is consumed on
// every outcome. Locking: the caller must not hold state_mutex(); the step takes it where it needs it.
void StepPipeline::run_export(Orchestrator &orchestrator, Print &print)
{
    if (!print.should_execute_step(STEP_EXPORT))
        throw RuntimeError("No export was requested: deposit an export request before running the export.");

    orchestrator.reset_plugin_cancel();
    // The export is an operation of its own: it gets a fresh progress plan holding this step only, whatever
    // the unfinished plan of an earlier operation was. The plan starts at the percentage already shown, which is
    // not the percentage of the previous plan: process() resets that plan when nothing is left to slice, and
    // export_gcode() publishes its 100 % without a plan. The main bar never goes back; the progress inside the step
    // is the secondary bar. A new slicing is a new operation and may restart at 0.
    const int shown_percent = print.pipeline_progress().shown_percent();
    print.pipeline_progress().reset();
    initialize_pipeline_progress(orchestrator, print, STEP_EXPORT, STEP_EXPORT, shown_percent);
    try {
        run_step_if_requested(print, STEP_EXPORT, [&] {
            begin_step(print, STEP_EXPORT, L("Exporting G-code"));
            StepExport::run_step(orchestrator, print);
        });
    } catch (...) {
        // A request serves one attempt: a failed or cancelled attempt must not leave it for a later one.
        print.take_export_request();
        throw;
    }
}

void StepPipeline::run(Orchestrator &orchestrator, Print &print, const std::string &path)
{
    StepPipeline::run_slice(orchestrator, print);
    if (!path.empty())
        StepPipeline::run_gcode(orchestrator, print, path);
}


} // namespace Slic3r::Steps
