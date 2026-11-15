///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "PrintingPlanFacts.hpp"

#include <algorithm>
#include <optional>
#include <set>

#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
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

object_layer_count counts plan layer groups with at least one extrusion sourced
from an object's sliced layers. A shared group counts once; repeated island or
copy passes count separately, even at the same Z. Auxiliary-only groups do not
count. Source owners are indexed once per call to distinguish their object layers
from support and adhesion layers without relying on roles or group copy lists.

Representative direct calls (branches are not a timeline):

    summarize_plan()
        |-- inspect extrusion source regions and index each owner's object layers
        |-- count all layer groups and object-containing passes independently
        |-- std::sort()
        `-- std::unique()
*/

namespace Slic3r::Printing {

// Scan a possibly null finalized plan once in print order and return its layer and tool facts; the plan is not modified.
PlanFacts summarize_plan(const PrintingPlan *plan);

PlanFacts summarize_plan(const PrintingPlan *plan)
{
    PlanFacts facts;
    if (plan == nullptr)
        return facts;

    std::optional<extruder_index_t> previous_tool;
    std::set<const PrintObject *> indexed_objects;
    std::set<const Layer *> object_layers;
    for (const PrintingGroup &group : plan->groups)
        for (const PrintingLayerGroup &layer : group.layers) {
            ++facts.layer_count;
            facts.max_print_z = std::max(facts.max_print_z, layer.print_z);
            bool object_pass = false;
            for (const PrintingToolGroup &tool : layer.tool_groups) {
                // Source ownership distinguishes physical object work from auxiliary geometry.
                for (const PrintingExtrusion &extrusion : tool.extrusions) {
                    if (object_pass) break;
                    if (extrusion.region_island == nullptr || extrusion.region_island->regions().empty() ||
                        !extrusion.root || extrusion.root->empty()) continue;
                    const Layer *source = (*extrusion.region_island->regions().begin())->layer();
                    const PrintObject *owner = source->object();
                    if (indexed_objects.insert(owner).second)
                        for (const Layer &object_layer : owner->layers()) object_layers.insert(&object_layer);
                    object_pass = object_layers.count(source) != 0;
                }
                if (tool.extruder_id == EXTRUDER_ID_UNKNOWN)
                    continue;
                facts.used_tools.push_back(tool.extruder_id);
                if (!previous_tool)
                    facts.initial_tool = tool.extruder_id;
                else if (*previous_tool != tool.extruder_id)
                    ++facts.toolchanges;
                previous_tool = tool.extruder_id;
            }
            if (object_pass) ++facts.object_layer_count;
        }
    std::sort(facts.used_tools.begin(), facts.used_tools.end());
    facts.used_tools.erase(std::unique(facts.used_tools.begin(), facts.used_tools.end()), facts.used_tools.end());
    return facts;
}

} // namespace Slic3r::Printing
