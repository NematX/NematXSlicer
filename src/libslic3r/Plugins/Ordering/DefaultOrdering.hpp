///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Plugins_Ordering_DefaultOrdering_hpp_
#define slic3r_Plugins_Ordering_DefaultOrdering_hpp_

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"

namespace slic3r_api { namespace Ordering { namespace DefaultOrderingPlugin {

/*
Register the built-in STEP_ORDERING plugins one by one.

Each function registers one phase of the ordering chain. Tests or experiments
may call a single registration helper when they want to replace only one phase
with another plugin. The aggregate helper is what the normal plugin loader uses:
it registers both plan builders, the instance-catalog completion phase,
the tool-group sorter, the coarse PrintingExtrusion pre-sort and both internal
tree-ordering alternatives. The pre-sort and final tree phase remain
independently replaceable through their own exclusive groups.
*/
// Register the By layer builder and its print-sequence selector.
void register_by_layer_plan_builder_plugin(orchestrator_handle *orch);
// Register the alternative stack builder, whose groups may share copies, a prefix or a tail.
void register_by_object_plan_builder_plugin(orchestrator_handle *orch);
void register_default_tool_group_ordering_plugin(orchestrator_handle *orch);
void register_printing_extrusion_pre_sort_plugin(orchestrator_handle *orch);
void register_wipe_into_first_ordering_plugin(orchestrator_handle *orch);
void register_advanced_extrusion_tree_ordering_plugin(orchestrator_handle *orch);
void register_default_extrusion_tree_ordering_plugin(orchestrator_handle *orch);
void register_default_ordering_plugins(orchestrator_handle *orch);

}}} // namespace slic3r_api::Ordering::DefaultOrderingPlugin

#endif // slic3r_Plugins_Ordering_DefaultOrdering_hpp_
