///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#ifndef slic3r_SourceLayerIndices_hpp_
#define slic3r_SourceLayerIndices_hpp_

#include <algorithm>
#include <map>
#include <optional>
#include <utility>
#include <vector>
#include "libslic3r/Api/plugin/cpp/AuxiliaryLayerHelpers.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"

/*
Provides source-layer indices for first-layer consumers. Build the table once in
serialized plugin setup and read it during layer runs. Each copy's sequence is
its object layers plus its local auxiliary layers (including support, raft and
instance-owned work), sorted by print_z and deduplicated by Z. Index zero is the
lowest layer of that copy, not necessarily its first object layer or bed contact.
A plan layer takes the minimum index of the copies present in its extrusion
sources, never the group's instance list. A global-only layer takes the number
of distinct plan layer heights strictly below it, including global auxiliaries.
Synthetic auxiliary catalog entries, such as the wipe tower, are never copies.
For a real copy with no sliced source layers, the distinct Z values of plan
passes containing its extrusions supply its sequence instead. This structural
fallback never gives synthetic catalog entries a copy sequence.
The owned table retains no borrowed views. For example:
    const SourceLayerIndices indices = source_layer_indices(print, plan);
    const bool first = indices[group][layer].index == 0;
*/
namespace slic3r_api {
struct CopyLayerIndex { uint32_t id; uint32_t index; };
struct SourceLayerIndex { uint32_t index = 0; std::vector<CopyLayerIndex> copies; };
using SourceLayerIndices = std::vector<std::vector<SourceLayerIndex>>;

// Build owned copy and plate Z ranks from borrowed sources and plan extrusions.
inline SourceLayerIndices source_layer_indices(const Print &print, const ConstPrintingPlan &plan)
{
    std::vector<std::vector<coord_t>> sequences(plan.object_instance_count());
    std::vector<coord_t> plate;
    for (uint32_t id = 0; id < plan.object_instance_count(); ++id) {
        const PrintingObjectInstance instance = plan.object_instance(id);
        if (!instance.has_object()) continue;
        const Object object = instance.object();
        for (uint32_t kind = 0; kind < 2; ++kind) {
            const uint32_t count = kind == 0 ? object.layer_count() : object.auxiliary_layer_count();
            for (uint32_t idx = 0; idx < count; ++idx) {
                const Layer layer = kind == 0 ? object.layer(idx) : object.auxiliary_layer(idx);
                const std::optional<uint32_t> owner = kind == 0 ? std::nullopt : layer_instance_owner(layer);
                if (!owner || *owner == instance.object_instance_idx())
                    sequences[id].push_back(layer.print_z());
            }
        }
        std::sort(sequences[id].begin(), sequences[id].end());
        sequences[id].erase(std::unique(sequences[id].begin(), sequences[id].end()), sequences[id].end());
    }
    // A real copy without sliced source layers needs ranks from its plan geometry.
    // Only those real entries may use structural Z ranks;
    // auxiliary/synthetic entries must remain outside per-copy bookkeeping.
    std::vector<std::vector<coord_t>> structural(sequences.size());
    for (uint32_t g = 0; g < plan.group_count(); ++g)
        for (uint32_t l = 0; l < plan.group(g).layer_group_count(); ++l) {
            const ConstPrintingLayerGroup layer = plan.group(g).layer_group(l);
            plate.push_back(layer.print_z());
            for (uint32_t t = 0; t < layer.tool_group_count(); ++t)
                for (uint32_t e = 0; e < layer.tool_group(t).extrusion_count(); ++e) {
                    const uint32_t id = layer.tool_group(t).extrusion(e).printing_catalog_id();
                    if (id < sequences.size() && plan.object_instance(id).has_object() && sequences[id].empty())
                        structural[id].push_back(layer.print_z());
                }
        }
    for (size_t id = 0; id < sequences.size(); ++id)
        if (sequences[id].empty()) {
            sequences[id] = std::move(structural[id]);
            std::sort(sequences[id].begin(), sequences[id].end());
            sequences[id].erase(std::unique(sequences[id].begin(), sequences[id].end()), sequences[id].end());
        }
    std::sort(plate.begin(), plate.end());
    plate.erase(std::unique(plate.begin(), plate.end()), plate.end());
    SourceLayerIndices result(plan.group_count());
    for (uint32_t g = 0; g < plan.group_count(); ++g) {
        const ConstPrintingGroup group = plan.group(g);
        result[g].resize(group.layer_group_count());
        for (uint32_t l = 0; l < group.layer_group_count(); ++l) {
            const ConstPrintingLayerGroup layer = group.layer_group(l);
            SourceLayerIndex &entry = result[g][l];
            entry.index = uint32_t(std::lower_bound(plate.begin(), plate.end(), layer.print_z()) - plate.begin());
            std::map<uint32_t, uint32_t> copies;
            for (uint32_t t = 0; t < layer.tool_group_count(); ++t) {
                const ConstPrintingToolGroup tool = layer.tool_group(t);
                for (uint32_t e = 0; e < tool.extrusion_count(); ++e) {
                    const ConstPrintingExtrusion extrusion = tool.extrusion(e);
                    const uint32_t id = extrusion.printing_catalog_id();
                    if (id >= sequences.size() || sequences[id].empty()) continue;
                    const LayerRegionIsland source = extrusion.region_island();
                    const coord_t z = source.valid() && source.region_count() > 0 ?
                        source.region(0).layer().print_z() : layer.print_z();
                    const uint32_t rank = uint32_t(std::lower_bound(sequences[id].begin(), sequences[id].end(), z) - sequences[id].begin());
                    const std::map<uint32_t, uint32_t>::iterator found = copies.find(id);
                    if (found == copies.end()) copies.emplace(id, rank);
                    else found->second = std::min(found->second, rank);
                }
            }
            for (const auto &[id, rank] : copies) {
                if (entry.copies.empty()) entry.index = rank;
                else entry.index = std::min(entry.index, rank);
                entry.copies.push_back({id, rank});
            }
        }
    }
    return result;
}
}
#endif
