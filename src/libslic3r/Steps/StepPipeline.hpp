///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef steps_steppipeline_hpp_
#define steps_steppipeline_hpp_

/* Pipeline execution and selection metadata. Exclusive groups select one
   provider independently of other groups on the same step. Active setting
   requirements use that group selection to check declared option support. */

#include "libslic3r/Api/plugin/c/slic3r_gcode_thumbnail.h"
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
    // kept and frozen; the pipeline resumes at STEP_PRE_PERIMETER.
    Slicing,
    // The whole Print data tree (objects, layers, regions, islands) is kept
    // and frozen after STEP_FINALIZE_SLICING; resumes at STEP_ORDERING.
    Print,
    // Optional deep copy of the PrintingPlan taken right after STEP_ORDERING;
    // resumes at STEP_WIPETOWER from a fresh clone of that copy.
    Ordering,
    // The finished PrintingPlan, read-only for STEP_GCODE; resumes at STEP_GCODE.
    Plan,
    // Every step has run. Asking to resume here forces nothing.
    GCode
};

struct StepExclusiveGroup
{
    std::string group_id;
    std::string option_key_storage;
    std::string label_storage;
    std::string tooltip_storage;
    slicing_step_t step;
    raw_config_option_def option_def;
    std::string ui_fragment;
    std::vector<raw_gui_rule> gui_activation_rules;
    // The option enum serializes plugin IDs. Its integer value is only the
    // current enum-table rank; generated provider rules compare the serialized ID.
    std::vector<std::string> enum_values;
    std::vector<std::string> enum_labels;
    std::vector<key_value_string_pair_t> enum_pairs;

    void refresh_storage_pointers();
    void set_enum_plugins(const std::vector<std::pair<std::string, std::string>> &plugin_ids_and_labels);
};

struct StepExclusivePluginGroup
{
    StepExclusiveGroup group;
    std::vector<Plugin *> plugins;
};

const std::map<slicing_step_t, StepExclusiveGroup> &get_exclusive_steps();

// Return the stable generated option key used by a plugin-defined exclusive
// group. Consumers use this to report the exact regional selector that failed.
std::string plugin_exclusive_group_option_key(const Orchestrator &orchestrator,
                                              slicing_step_t step,
                                              const std::string &group_id);

// Return the generated selector definition for the given plugin-defined exclusive group.
StepExclusiveGroup plugin_exclusive_group_definition(const Orchestrator &orchestrator,
                                                      slicing_step_t step,
                                                      const std::string &group_id,
                                                      const Plugin &first_plugin);

std::vector<StepExclusivePluginGroup> active_exclusive_plugin_groups(Orchestrator &orchestrator);
std::vector<Plugin *> selected_or_active_plugins_for_step(Orchestrator &orchestrator,
                                                          slicing_step_t step,
                                                          const ConfigBase *config);
Plugin *selected_or_active_plugin_for_step(Orchestrator &orchestrator,
                                           slicing_step_t step,
                                           const ConfigBase *config);

// Select one active provider from a specific plugin-defined exclusive group.
// This is the service-plugin counterpart of selecting a complete built-in
// step: unrelated groups registered on the same numeric step are ignored.
Plugin *selected_active_plugin_from_group(Orchestrator &orchestrator,
                                          slicing_step_t step,
                                          const std::string &exclusive_group,
                                          const ConfigBase *config);

// Return the first active setting unsupported by its selected exclusive-group provider.
std::optional<std::string> unsupported_active_setting(
    const ConfigDef &definitions, const ConfigBase &config,
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
// First step executed when resuming from checkpoint, or STEP_NONE for
// Checkpoint::GCode, which has nothing left to run.
slicing_step_t checkpoint_resume_step(Checkpoint checkpoint);

// Direct dependency graph used by Print's new invalidation plan. If A maps to
// B, invalidating A also asks B to run on the next process().
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
        const raw_gcode_thumbnail_renderer *thumbnail_renderer = nullptr,
        GCodeProcessorResult *result = nullptr);

    // Compatibility wrapper for code that still wants the whole migrated
    // pipeline in one call. Normal GUI/CLI flow uses run_slice() and run_gcode()
    // separately so the final output path is available only during export.
    static void run(Orchestrator &orchestrator, Print &print, const std::string &path = std::string());

};

} // namespace Steps
} // namespace Slic3r

#endif // steps_steppipeline_hpp_
