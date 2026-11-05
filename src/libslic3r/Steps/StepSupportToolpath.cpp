///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "StepSupportToolpath.hpp"

#include <optional>

#include "libslic3r/Api/plugin/c/steps/slic3r_step_support_toolpath.h"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/PrintObject.hpp"
#include "InfillPatternService.hpp"
#include "SupportPatternCallbacks.hpp"
#include "StepPipeline.hpp"
#include "StepRunner.hpp"

/*
This step creates toolpaths on support surfaces.
Each role group contains alternative algorithms for one support task.
Each object's configuration selects one algorithm per group.
The dispatcher runs algorithms by priority on the objects that selected them.
Callbacks add extrusion through the data tree; the dispatcher stores no geometry.
register_selector_traits registers per-object selection and the Support material /
Support toolpaths GUI placement before plugins register.

Algorithms may fill surfaces with INFILL_PATTERN plugins through the payload
callbacks. One InfillPatternService is built before the first algorithm runs
and shared by every object run, which may be concurrent: the service is
read-only after construction (see InfillPatternService.hpp).

Errors: the callbacks are called through the C ABI, also by Python
algorithms through ctypes, which cannot catch a C++ exception, so they catch
everything. An ordinary refusal (unknown runtime id, pattern needing the
STEP_INFILL initialization pass) is a message from the service and the step
goes on. A failure (a pattern reports an error, or any other exception)
empties the output, returns the failure value and requests the plugin
cancellation (a PluginStopped adds no message, any other exception adds an
error message), so that the runner stops the step when the
algorithm returns (contain_pattern_callback_failure). A cancellation of the
print is not recorded: the runner rethrows it. Stop checks select the supplied
Print's execution. Once that execution stops or the Print is cancelled,
callbacks return failure at once, without adding another message.

Representative direct calls (branches are not a timeline):

    run_step()
        |-- Detail::validate_or_report()
        |-- orchestrator.get_active_plugins_for_step()
        |-- selected_plugins_per_object()
        |-- Detail::run_object_step_plugin()
        `-- complete_plugin()
*/

namespace Slic3r::Steps::StepSupportToolpath {
// Copy object-scope automatic selector metadata into the borrowed host without changing print data or provider ownership;
// registration conflicts assert and allocation failures propagate.
void register_selector_traits(Orchestrator &orchestrator);

void register_selector_traits(Orchestrator &orchestrator)
{
    StepSelectorTraits traits;
    traits.container_type = RAW_CONTAINER_TYPE_OBJECT;
    traits.category = RAW_OPTION_CATEGORY_SUPPORT;
    traits.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    traits.invalidates_step = STEP_SUPPORT_TOOLPATH;
    traits.ui_target_file = "print.ui";
    traits.ui_page = "Support material";
    traits.ui_group = "Support toolpaths";
    traits.auto_value = true;
    const int32_t registered = orchestrator.register_builtin_step_selector(STEP_SUPPORT_TOOLPATH, std::move(traits));
    assert(registered == 1);
    (void)registered;
}

namespace {

// What the payload's host_context points to: the pattern service (null when no algorithm is active, then
// every callback refuses) and the orchestrator that receives the contained failures.
struct HostContext
{
    InfillPatternService *service;
    Orchestrator &owner;

    // Reports whether this borrowed prebuilt service exists, without changing ownership or throwing.
    bool available() const noexcept { return service != nullptr; }
    // Returns the borrowed service after available() succeeds; no ownership changes or failures occur.
    InfillPatternService &get() const noexcept { return *service; }
    // Returns the borrowed orchestrator used for containment, without mutation or failure.
    Orchestrator &orchestrator() const noexcept { return owner; }
    // Read cancellation and execution-stop state for the supplied Print.
    // With a null Print, read only the thread's execution stop state, or the shared stop state.
    bool stop_requested(const print_handle *handle) const noexcept
    {
        const Print *print = reinterpret_cast<const Print *>(handle);
        return owner.is_plugin_cancelled(print) || (print != nullptr && print->canceled());
    }
};

// Number of auxiliary fill surfaces of the print, advertised to the pattern
// plugins as their maximum run count.
size_t count_auxiliary_surfaces(const Print &print);
// Bind the shared noexcept relay to this step's prebuilt service policy.
using PatternCallbacks = SupportPatternCallbacks<run_ctx_support_toolpath, HostContext, STEP_SUPPORT_TOOLPATH>;

size_t count_auxiliary_surfaces(const Print &print)
{
    size_t count = 0;
    for (const PrintObject &object : print.objects())
        for (const Layer &layer : object.auxiliary_layers())
            for (const LayerSliceIsland &island : layer.islands())
                for (const LayerRegionIsland &region_island : island.regions_islands())
                    count += region_island.fill_surfaces().size();
    return count;
}

} // namespace

// Nothing to clear: the extrusions live in the support auxiliary layers, which a Slicing replay destroys.
void clear(Print &) {}

// Nothing to prepare: the algorithms read the surfaces left by the support families.
void prepare(Print &) {}

// Accepts borrowed input without algorithm-specific checks or modifying the optional error string; always returns true without failure.
bool validate_pre(const Print &, std::string *)
{
    return true;
}

// Accepts borrowed output without algorithm-specific checks or modifying the optional error string; always returns true without failure.
bool validate_post(const Print &, std::string *)
{
    return true;
}

// Borrows print/orchestrator, builds a step-owned pattern service before joined object runs, and dispatches selected algorithms in priority order; algorithms publish layer-owned paths, and validation/runner/cancellation/allocation failures propagate.
void run_step(Orchestrator &orchestrator, Print &print)
{
    Detail::validate_or_report(validate_pre, print, "Support-toolpath pre-step validation");
    // Built before any object run so concurrent calls never create pattern
    // contexts, storages or error slots. Patterns needing the STEP_INFILL
    // initialization pass are refused because this step has none. The service
    // sets up the patterns, so it is skipped when no algorithm can call them.
    std::optional<InfillPatternService> patterns;
    if (!orchestrator.get_active_plugins_for_step(STEP_SUPPORT_TOOLPATH).empty())
        patterns.emplace(orchestrator, print, count_auxiliary_surfaces(print),
                         InfillPatternService::SetupPatterns::Refuse);
    // Without an active algorithm the runner never builds a payload; the null
    // check keeps the factory safe even if that selection rule changes.
    HostContext host_context { patterns.has_value() ? &*patterns : nullptr, orchestrator };
    for (const ObjectStepPlugin &entry : selected_plugins_per_object(orchestrator, STEP_SUPPORT_TOOLPATH, print)) {
        Detail::run_object_step_plugin(orchestrator,
                                       print,
                                       STEP_SUPPORT_TOOLPATH,
                                       *entry.plugin,
                                       entry.object_indices,
                                       print.objects().size(),
                                        // Builds a borrowed payload for a valid selected object index; the joined run owns its lifetime, and no geometry is copied.
                                        [&print, &host_context](const size_t object_idx) {
            run_ctx_support_toolpath context = {};
            context.print = reinterpret_cast<const print_handle *>(&print);
            context.object = reinterpret_cast<const object_handle *>(&print.object(object_idx));
            context.host_context = &host_context;
            context.resolve_pattern_id = &PatternCallbacks::resolve;
            context.generate_pattern = &PatternCallbacks::generate;
            return context;
        });
        print.pipeline_progress().complete_plugin(STEP_SUPPORT_TOOLPATH, entry.plugin->get_id());
    }
    Detail::validate_or_report(validate_post, print, "Support-toolpath post-step validation");
}

} // namespace Slic3r::Steps::StepSupportToolpath
