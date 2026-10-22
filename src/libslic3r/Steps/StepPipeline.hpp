///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef steps_steppipeline_hpp_
#define steps_steppipeline_hpp_

/*
StepPipeline prepares and runs the FFF processing stages for a Print. A checkpoint
is a saved boundary from which invalidated work can be recomputed; it is not a
copy of the entire application. Run slicing before the plan/G-code suffix unless
the caller already has a valid result at the required boundary.

Plugin selection decides which implementation performs a stage. Export requests
are separate actions on a ready command reference and are not implicitly queued
by every invalidation or replay of geometry processing.

Pipeline execution and selection metadata. Exclusive groups select one
provider independently of other groups on the same step. Active setting
requirements use that group selection to check declared option support.
Whole-step identities hold keys and display text; StepSelectorTraits.hpp defines the settings
and UI placement rules declared by each step module.
*/

#include "libslic3r/Api/plugin/c/slic3r_gcode_thumbnail.h"
#include "libslic3r/Api/host/StepSelectorTraits.hpp"
#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/slic3r_plugin_types.h"

namespace Slic3r {

class ConfigBase;
class ConfigDef;
struct GCodeProcessorResult;
class Orchestrator;
class Plugin;
class Print;
class PrintObject;

namespace Steps {

// Checkpoints are the fixed, host-owned points from which the pipeline can
// resume. Each one names the data that stays valid when every later step is
// replayed. They are ordered: an earlier checkpoint keeps less data.
enum class Checkpoint : uint8_t
{
    // Nothing is reusable: slice everything from STEP_LAYER_HEIGHT.
    None,
    // Layer heights and slices (STEP_LAYER_HEIGHT to STEP_POST_SLICING) are
    // kept and frozen; the pipeline resumes at STEP_SUPPORT_DEMAND.
    Slicing,
    // The whole Print data tree (objects, layers, regions, islands) is kept
    // and frozen after STEP_FINALIZE_SLICING; resumes at STEP_ORDERING.
    Print,
    // Optional deep copy of the PrintingPlan taken right after STEP_ORDERING;
    // resumes at STEP_WIPETOWER from a fresh clone of that copy.
    Ordering,
    // The finished PrintingPlan, read-only for STEP_GCODE; resumes at STEP_GCODE.
    Plan,
    // The G-code reference is ready (STEP_GCODE and its reading are done). The
    // export is not part of the reference: it is requested explicitly and
    // resumes here, so keeping this checkpoint discards nothing. Asking to
    // replay from here forces no step.
    GCode
};

struct StepExclusiveGroup
{
    std::string group_id;
    std::string option_key_storage;
    std::string label_storage;
    std::string tooltip_storage;
    std::string translation_domain_storage;
    std::string default_provider_id;
    bool member_declared = false;
    slicing_step_t step;
    // These are declared or default traits; option_def stores the resolved invalidation step, including any registry fallback.
    StepSelectorTraits traits;
    raw_config_option_def option_def;
    std::string ui_fragment;
    std::vector<raw_gui_rule> gui_activation_rules;
    // The option enum serializes plugin IDs. Its integer value is only the
    // current enum-table rank; generated provider rules compare the serialized ID.
    std::vector<std::string> enum_values;
    std::vector<std::string> enum_labels;
    std::vector<key_value_string_pair_t> enum_pairs;

    // Point the C description at this group's owned text, catalogue name, choices and default; call after moving the group.
    // Settings and plugins are unchanged; rebuilding choice pointers may allocate.
    void refresh_storage_pointers();
    // Copy choices in group-priority/id order and retain their default, keeping automatic selection first when enabled.
    // The group owns choice/default strings and borrows the application; unknown ids use 100, and allocations may throw.
    void set_enum_plugins(const Orchestrator &orchestrator,
                          const std::vector<std::pair<std::string, std::string>> &plugin_ids_and_labels,
                          const std::string &default_provider_id);
};

struct StepExclusivePluginGroup
{
    StepExclusiveGroup group;
    std::vector<Plugin *> plugins;
    bool whole_step = false;
};

// The host accounts for rereading the generated G-code separately from writer plugins.
inline constexpr const char *k_gcode_read_progress_unit = "host:gcode_read";

// These keys and labels identify settings that select a provider for an entire step.
struct StepExclusiveIdentity
{
    const char *key;
    const char *label;
    const char *line_label;
};

// Return the keys and labels of selectors for entire steps.
const std::map<slicing_step_t, StepExclusiveIdentity> &get_exclusive_steps();
// Recognize built-in and registered dynamic selector stages, excluding NONE and ANY.
bool selector_step_is_registered(const Orchestrator &orchestrator, slicing_step_t step);
// Borrow the supplied candidate with smallest group priority, then smallest id; an empty list returns null.
// Candidates must be nonnull; activation is not checked.
Plugin *default_provider_from_group(const std::vector<Plugin *> &plugins);

// Return the stable generated option key used by a plugin-defined exclusive
// group. Consumers use this to report the exact regional selector that failed.
std::string plugin_exclusive_group_option_key(const Orchestrator &orchestrator,
                                              slicing_step_t step,
                                              const std::string &group_id);
// Return a generated key without consulting descriptions or reserving it.
std::string generated_plugin_exclusive_group_option_key(const Orchestrator &orchestrator,
                                                        slicing_step_t step, const std::string &group_id);

// Return the generated selector definition for the given plugin-defined exclusive group.
StepExclusiveGroup plugin_exclusive_group_definition(const Orchestrator &orchestrator,
                                                      slicing_step_t step,
                                                      const std::string &group_id,
                                                      const Plugin &first_plugin);

// Build a selector definition and UI fragment from its identity and module traits without registration or configuration changes.
StepExclusiveGroup make_exclusive_step_group(const Orchestrator &orchestrator, slicing_step_t step,
                                             const StepExclusiveIdentity &identity);
// Declare built-in selector traits through modules using only the supplied host, including during host construction.
void register_builtin_step_selectors(Orchestrator &orchestrator);

// Build selector descriptions for enabled groups using registered setting rules and retained text.
std::vector<StepExclusivePluginGroup> active_exclusive_plugin_groups(Orchestrator &orchestrator);
// Return the active providers selected for the whole step or its named groups by the supplied settings.
std::vector<Plugin *> selected_or_active_plugins_for_step(Orchestrator &orchestrator,
                                                          slicing_step_t step,
                                                          const ConfigBase *config);

// A provider of a per-object step with the indices of the objects it runs on.
struct ObjectStepPlugin
{
    Plugin *plugin = nullptr;
    std::vector<size_t> object_indices;
};
// Providers of a per-object step, in priority order. Each object selects its own providers from its
// configuration, so a provider appears once, with every object that selected it.
std::vector<ObjectStepPlugin> selected_plugins_per_object(Orchestrator &orchestrator,
                                                          slicing_step_t step,
                                                          const Print &print);
Plugin *selected_or_active_plugin_for_step(Orchestrator &orchestrator,
                                           slicing_step_t step,
                                           const ConfigBase *config);

// Select only among active providers in the requested group; return a sole member without reading the stored choice
// unless the group's selector reads that choice even with one active member.
// With several members, read the whole-step selector when present, otherwise the group's own option.
Plugin *selected_active_plugin_from_group(Orchestrator &orchestrator,
                                          slicing_step_t step,
                                          const std::string &exclusive_group,
                                          const ConfigBase *config);

// Return the first setting unsupported by its enabled member, using retained group text from the borrowed application.
std::optional<std::string> unsupported_active_setting(
    const Orchestrator &orchestrator, const ConfigDef &definitions, const ConfigBase &config,
    const std::function<const Plugin *(const std::string &group_id)> &selected_plugin_of_group);

// Written execution order for the migrated pipeline. The order is intentionally
// not inferred from the dependency graph: a missing step in this list is a
// pipeline definition error, not something the graph should silently repair.
const std::vector<slicing_step_t> &execution_order();

// Return a step's position in execution_order(), or execution_order().size()
// when the value is not an executable pipeline step.
size_t execution_index(slicing_step_t step);

// Compare executable steps by the written pipeline order, never by their ABI
// numeric values. Both arguments must belong to execution_order().
bool executes_before(slicing_step_t lhs, slicing_step_t rhs);

// Return the invalidation step of a setting once a plugin running at
// consumer_step reads it. STEP_ANY and an explicit STEP_NONE are kept; any other
// value becomes the earlier of both steps in execution_order(), so a changed
// value replays both its original producer and the consumer. Apart from
// STEP_ANY and STEP_NONE, both steps must belong to execution_order().
slicing_step_t tightened_invalidation_step(slicing_step_t current, slicing_step_t consumer_step);

// Return the latest checkpoint that does not contain the output of step, that
// is the checkpoint to resume from when step must run again. STEP_ANY maps to
// Checkpoint::None. A step outside execution_order() is a caller error; it
// asserts and conservatively maps to Checkpoint::None.
Checkpoint checkpoint_before_step(slicing_step_t step);
// First step executed when resuming from checkpoint. For Checkpoint::GCode this
// is STEP_EXPORT, which runs only when requested (see Print::request_export).
slicing_step_t checkpoint_resume_step(Checkpoint checkpoint);

// Direct dependency graph used by Print's new invalidation plan. If A maps to
// B, invalidating A also asks B to run on the next process(). STEP_EXPORT has no
// incoming edge: its output (a published file) is not a reusable result, so no
// invalidation queues it; only Print::request_export does.
const std::map<slicing_step_t, std::vector<slicing_step_t>> &step_dependents();

// Stable transitive closure of step_dependents(). The returned vector follows
// execution_order(), so callers may insert it into sets or display it without
// depending on std::map iteration details.
std::vector<slicing_step_t> dependent_steps_closure(slicing_step_t step);

#ifdef _DEBUG
// Debug guard for the central graph definition. Each edge A -> B must respect
// execution_order(); otherwise the pipeline would need to run B before A has
// produced its new data.
bool validate_execution_order_against_dependencies();
#endif

// Central entry point for the step-based slicing pipeline.
//
// Orchestrator owns plugin registration and host callbacks; StepPipeline owns
// the ordered execution of slicing steps. Keeping the ordering here avoids
// growing Orchestrator with step-specific code.
class StepPipeline
{
public:
    // Release step outputs and plugin-visible owners that a checkpoint resume
    // rebuilds, before the steps after the checkpoint run again.
    static void clear_checkpoint_outputs(Print &print, Checkpoint checkpoint);
    // Prepare the sliced print tree up to the point where G-code ordering can
    // start. Export callers run this first through Print::process(), then call
    // run_gcode() later with the concrete output path.
    static void run_slice(Orchestrator &orchestrator, Print &print);

    // Finish the export-side pipeline. This starts at STEP_ORDERING because the
    // G-code writer consumes the ordered PrintingPlan, then runs every
    // post-ordering step, STEP_GCODE, and the host-only preview reader phase.
    static std::string run_gcode(Orchestrator &orchestrator, Print &print, const std::string &path,
        GCodeProcessorResult *result = nullptr);

    // Run STEP_EXPORT alone on the G-code reference that STEP_GCODE already prepared. Print::request_export
    // must have queued it: otherwise this throws. No earlier step runs, even when an invalidation made one
    // pending since: the export then fails with a "reference not ready" error (see StepExport::run_step) and
    // writes nothing. It resets the pipeline progress and plans a one-step table, consumes the request on
    // every outcome, and stores the published path in the Print. The caller must not hold state_mutex().
    static void run_export(Orchestrator &orchestrator, Print &print);

    // Compatibility wrapper for code that still wants the whole migrated
    // pipeline in one call. Normal GUI/CLI flow uses run_slice() and run_gcode()
    // separately so the final output path is available only during export.
    static void run(Orchestrator &orchestrator, Print &print, const std::string &path = std::string());

};

} // namespace Steps
} // namespace Slic3r

#endif // steps_steppipeline_hpp_
