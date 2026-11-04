///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
/* StepSelectorTraits describes settings that choose plugins for a slicing step or a named group.
   A provider is a registered plugin; the selector is the setting that chooses it.

   Fill StepSelectorTraits and pass it to Orchestrator::register_builtin_step_selector()
   before registering providers. An exact group's declaration replaces, rather than merges with,
   the declaration for its entire step. Undeclared steps use the defaults below.
   SelectorAutoChoice names a preferred provider when a setting has a particular serialized value.
   Example: StepSelectorTraits traits; traits.category = RAW_OPTION_CATEGORY_INFILL; */
#ifndef slic3r_StepSelectorTraits_hpp_
#define slic3r_StepSelectorTraits_hpp_

#include <optional>
#include <string>

#include "libslic3r/Api/plugin/c/slic3r_config_def.h"

namespace Slic3r {

struct StepSelectorTraits
{
    // An empty name describes the entire step; a nonempty name describes only that group.
    std::string exclusive_group;
    raw_container_type container_type = RAW_CONTAINER_TYPE_PROJECT;
    // If absent, use the dynamic step's registered invalidation, the built-in step itself, or STEP_ANY if unknown.
    std::optional<slicing_step_t> invalidates_step;
    raw_option_category category = RAW_OPTION_CATEGORY_SLICING;
    raw_option_preset_type option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    std::string ui_target_file = "print.ui";
    // An empty page and group place the selector under Notes / Exclusive step plugins.
    std::string ui_page;
    std::string ui_group;
    // An empty key is generated from the step and group ids.
    std::string option_key;
    // These flags control invalid stored choices and whether a lone provider needs a selector.
    bool strict_selection = false;
    bool read_with_one_provider = false;
    bool show_with_one_provider = false;
    // Use this serialized choice when the setting is absent, if a value is supplied.
    std::optional<std::string> value_when_absent;
    bool provider_settings_rules = true;
    // Allow an automatic choice resolved by SelectorAutoChoice conditions.
    bool auto_value = false;
};

// A setting key/value condition names the member preferred during automatic selection.
struct SelectorAutoChoice
{
    slicing_step_t step = STEP_NONE;
    std::string exclusive_group;
    std::string condition_key;
    std::string condition_value;
    std::string provider_id;
};

} // namespace Slic3r
#endif // slic3r_StepSelectorTraits_hpp_
