///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_Printing_PrintingPlanValidation_hpp_
#define slic3r_Printing_PrintingPlanValidation_hpp_

#include <string>
#include <typeinfo>

#include "PrintingPlan.hpp"

/* Check canonical plan storage at debug stage boundaries and in tests:
       const bool valid = validate_canonical_plan(plan, &error);
   Every owned node must be a base ExtrusionEntity, with no null children. Scope
   roots must also remain fixed Children sequences. The first violation reports
   its complete storage path; no tree, permission or handle is repaired or replaced.
   Source-region and catalogue contracts stay with each stage's own validator.
   The plan is borrowed read-only; diagnostics may allocate and propagate failures.
*/

namespace Slic3r { namespace Printing {

// Report the first non-base or null owned node with its full plan path.
inline bool validate_canonical_tree(const ExtrusionEntity &node, const std::string &path, std::string *error)
{
    if (typeid(node) != typeid(ExtrusionEntity)) {
        if (error) *error = path + ": legacy extrusion type crossed the plan boundary";
        return false;
    }
    for (size_t i = 0; i < node.children().size(); ++i) {
        const ExtrusionEntityUPtr &child = node.children()[i];
        const std::string child_path = path + ".child[" + std::to_string(i) + "]";
        if (!child) {
            if (error) *error = child_path + ": null owned extrusion child";
            return false;
        }
        if (!validate_canonical_tree(*child, child_path, error))
            return false;
    }
    return true;
}

// Require fixed Children event roots as well as canonical descendants.
inline bool validate_canonical_events(const PrintingScopeEvents &events, const std::string &path, std::string *error)
{
    for (const bool before : {true, false}) {
        const ExtrusionEntity &root = before ? events.before() : events.after();
        const std::string root_path = path + (before ? ".events.before" : ".events.after");
        if (root.is_leaf() || root.can_sort() || root.can_reverse() || root.can_rotate()) {
            if (error) *error = root_path + ": event root must be a fixed Children sequence";
            return false;
        }
        if (!validate_canonical_tree(root, root_path, error))
            return false;
    }
    return true;
}

// Inspect all owned plan trees at a stage boundary. Source geometry and catalog
// checks remain with their stage validators; this function never consumes flags.
inline bool validate_canonical_plan(const PrintingPlan &plan, std::string *error = nullptr)
{
    if (error) error->clear();
    if (!validate_canonical_events(plan.events, "plan", error)) return false;
    for (size_t g = 0; g < plan.groups.size(); ++g) {
        const PrintingGroup &group = plan.groups[g];
        const std::string group_path = "group[" + std::to_string(g) + "]";
        if (!validate_canonical_events(group.events, group_path, error)) return false;
        for (size_t l = 0; l < group.layers.size(); ++l) {
            const PrintingLayerGroup &layer = group.layers[l];
            const std::string layer_path = group_path + ".layer[" + std::to_string(l) + "]";
            if (!validate_canonical_events(layer.events, layer_path, error)) return false;
            for (size_t t = 0; t < layer.tool_groups.size(); ++t) {
                const PrintingToolGroup &tool = layer.tool_groups[t];
                const std::string tool_path = layer_path + ".tool[" + std::to_string(t) + "]";
                if (!validate_canonical_events(tool.events, tool_path, error)) return false;
                for (size_t e = 0; e < tool.extrusions.size(); ++e) {
                    const std::string root_path = tool_path + ".extrusion[" + std::to_string(e) + "].root";
                    if (!tool.extrusions[e].root) {
                        if (error) *error = root_path + ": null owned extrusion root";
                        return false;
                    }
                    if (!validate_canonical_tree(*tool.extrusions[e].root, root_path, error)) return false;
                }
            }
        }
    }
    return true;
}

}} // namespace Slic3r::Printing

#endif // slic3r_Printing_PrintingPlanValidation_hpp_
