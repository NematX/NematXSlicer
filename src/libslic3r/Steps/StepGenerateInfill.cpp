///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
/* Run STEP_INFILL plugins in priority order with the common object-step runner.
   Each plugin gets phase-local services; the generator, not this step, starts
   pattern initialization after all object jobs have been discovered. */
#include "StepGenerateInfill.hpp"
#include "StepRunner.hpp"
#include "libslic3r/Api/host/steps/InfillStep.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/PrintObject.hpp"

namespace Slic3r::Steps::StepGenerateInfill {
void register_selector_traits(Orchestrator &orchestrator)
{
    StepSelectorTraits traits;
    traits.category = RAW_OPTION_CATEGORY_INFILL;
    traits.invalidates_step = STEP_INFILL;
    const int32_t step_registered = orchestrator.register_builtin_step_selector(STEP_INFILL, traits);
    assert(step_registered == 1);
    (void)step_registered;
    Orchestrator::ExclusiveGroupDefinition definition;
    definition.translation_domain = "Slic3r";
    definition.label = "Infill plugin";
    definition.line_label = "Infill step plugin";
    definition.tooltip = "Choose which active plugin generates infill.";
    definition.traits = traits;
    definition.traits.exclusive_group = "generator";
    definition.traits.option_key = "step_infill_plugin";
    const int32_t registered = orchestrator.register_builtin_exclusive_group(STEP_INFILL, std::move(definition));
    assert(registered == 1);
    (void)registered;
}

void clear(Print &) {}
void prepare(Print &) {}
bool validate_pre(const Print &, std::string *) { return true; }
bool validate_post(const Print &, std::string *) { return true; }

void run_step(Orchestrator &orchestrator, Print &print)
{
    for (Plugin *plugin : selected_or_active_plugins_for_step(orchestrator, STEP_INFILL, &print.full_print_config())) {
        if (plugin == nullptr) continue;
        ApiHost::Steps::InfillGenerationContext context(orchestrator, print);
        Detail::run_object_step_plugin(orchestrator, print, STEP_INFILL, *plugin, print.objects().size(),
            [&context, &print](size_t index) { return context.payload(print.object(index)); },
            [&context] { context.finish_discovery(); });
        print.pipeline_progress().complete_plugin(STEP_INFILL, plugin->get_id());
    }
}
}
