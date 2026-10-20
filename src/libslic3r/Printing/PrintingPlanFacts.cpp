///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "PrintingPlanFacts.hpp"

#include <algorithm>
#include <optional>

#include "libslic3r/Layer.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"

/*
Facts about a finalized PrintingPlan
====================================

summarize_plan() walks groups, layer groups and tool groups in print order once.
layer_count and max_print_z are updated for every layer group, before any tool
visit is looked at. Only the tool facts depend on concrete visits: a tool group is a
"visit"; a visit whose extruder is EXTRUDER_ID_UNKNOWN is not a concrete tool and is
skipped, while a concrete visit without extrusions still counts, because the writer
enters it and selects its tool. The first concrete visit gives the initial tool, and
a change between two consecutive concrete visits is one tool change, even across
layers. The used tools are the tools of the concrete visits. The `T` commands of custom G-code
scripts are not part of the plan and are never counted.

object_layer_count() counts distinct scaled Z values among object layers that
have extrusions (support layers are separate auxiliary layers and are not read).
With complete objects every instance prints its own copy of the layers, so the
count is multiplied per object by its number of instances.

Representative direct calls (branches are not a timeline):

    summarize_plan()
        |-- std::sort()
        `-- std::unique()

    object_layer_count()
        |-- object.layers()
        |-- layer.has_extrusions()
        |-- layer.scaled_print_z()
        `-- object.instances()
*/

namespace Slic3r::Printing {

// Scan a possibly null finalized plan once in print order and return its layer and tool facts; the plan is not modified.
PlanFacts summarize_plan(const PrintingPlan *plan);

// Count the distinct scaled Z values of the object layers that have extrusions (per object and instance with complete objects).
int32_t object_layer_count(const Print &print);

// Scan the borrowed plan once in print order. Expects a finalized plan or null (default facts). Guarantees a sorted,
// duplicate-free used_tools list; the tool facts (used tools, initial tool, tool changes) come from concrete tool visits only, while layer_count counts every layer group and max_print_z covers every layer group, before any tool filtering. The plan is not modified.
PlanFacts summarize_plan(const PrintingPlan *plan)
{
    PlanFacts facts;
    if (plan == nullptr)
        return facts;

    std::optional<extruder_index_t> previous_tool;
    for (const PrintingGroup &group : plan->groups)
        for (const PrintingLayerGroup &layer : group.layers) {
            ++facts.layer_count;
            facts.max_print_z = std::max(facts.max_print_z, layer.print_z);
            for (const PrintingToolGroup &tool : layer.tool_groups) {
                if (tool.extruder_id == EXTRUDER_ID_UNKNOWN)
                    continue;
                facts.used_tools.push_back(tool.extruder_id);
                if (!previous_tool)
                    facts.initial_tool = tool.extruder_id;
                else if (*previous_tool != tool.extruder_id)
                    ++facts.toolchanges;
                previous_tool = tool.extruder_id;
            }
        }
    std::sort(facts.used_tools.begin(), facts.used_tools.end());
    facts.used_tools.erase(std::unique(facts.used_tools.begin(), facts.used_tools.end()), facts.used_tools.end());
    return facts;
}

// Collect the scaled Z of every object layer that has extrusions, deduplicate them (per object when complete objects
// are printed, because each instance repeats its layers) and return the count. Expects a sliced Print; it is not modified.
int32_t object_layer_count(const Print &print)
{
    int32_t count = 0;
    if (print.config().complete_objects.value) {
        for (const PrintObject &object : print.objects()) {
            std::vector<coord_t> z_values;
            for (const Layer &layer : object.layers())
                if (layer.has_extrusions())
                    z_values.push_back(layer.scaled_print_z());
            std::sort(z_values.begin(), z_values.end());
            const std::vector<coord_t>::iterator unique_end =
                std::unique(z_values.begin(), z_values.end());
            count += int32_t(object.instances().size() * (unique_end - z_values.begin()));
        }
        return count;
    }

    std::vector<coord_t> z_values;
    for (const PrintObject &object : print.objects())
        for (const Layer &layer : object.layers())
            if (layer.has_extrusions())
                z_values.push_back(layer.scaled_print_z());
    std::sort(z_values.begin(), z_values.end());
    return int32_t(std::unique(z_values.begin(), z_values.end()) - z_values.begin());
}

} // namespace Slic3r::Printing
