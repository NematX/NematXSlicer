#include "GCodeExportObjects.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <iterator>
#include <map>
#include <stdexcept>
#include <utility>
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"
#include "libslic3r/Printing/PrintingPlanFacts.hpp"

/*
Provide G-code exporters with object identities, tool information and measurements
of generated brim, support and wipe-tower geometry.

A sliced Print contains objects and layers. Its printing plan contains the
extrusions in print order, already placed on the plate. This file reads both
and returns a GCodeExportDescription without exposing native host pointers.

The plan's object list contains printed object instances and generated items,
such as wipe towers. The result copies their identifiers and names. Real object
instances also carry their object and instance indices, which exporters can use
with the public data-tree API declared in slic3r_data_tree.h.

A footprint is the XY area covered by the full width of deposited paths.
Per-instance brim and support measurements use their lowest extrusion Z.
Brim and support are identified from their source layers because their paths
may be stored alongside perimeters. Skirts are excluded. Global brim and the
combined wipe-tower footprint are returned separately.

Coverage is merged before computing its area and bounding box. Coordinates
remain in the plan's plate coordinates, in millimeters; areas are in square
millimeters. This helper applies no additional placement or tool offset.

    copy_gcode_export_objects()
        |-- Printing::summarize_plan() [tool usage and initial tool]
        |-- record_export_sources() [source-layer identities]
        |-- collect_export_leaves() [coverage of depositing paths]
        |   `-- collect_export_leaves() [children]
        |-- first_coverage() [lowest extrusion Z]
        `-- export_footprint() [merged area and bounds]

The source Print, plan and layers must remain alive and unchanged during the
call. They are not modified. The returned description owns its data and retains
no pointer to those source objects.
*/
namespace Slic3r::ApiHost {
namespace {
struct ExportLeaf
{
    coord_t z = 0;
    double height = 0.;
    Polygons coverage;
};
// Collect depositing leaves with inherited width, height and Z; clone leaves only to resolve coverage attributes.
void collect_export_leaves(const ExtrusionEntity &entity, coord_t print_z, ExtrusionAttributes attributes,
                           coord_t z_offset, std::vector<ExportLeaf> &out);
// Merge only leaves at the lowest physical Z, returning their actual coverage and minimum thickness.
Polygons first_coverage(const std::vector<ExportLeaf> &leaves, double &height);
// Convert a coverage union to an owned mm/mm2 footprint; empty coverage stays absent.
GCodeExportFootprint export_footprint(const Polygons &polygons, double height = 0.);
// Index every source region in normal and auxiliary layers independently of a Z group's layer list.
void record_export_sources(const PrintObject &object, std::map<const LayerRegionIsland *, const Layer *> &sources);

void collect_export_leaves(const ExtrusionEntity &entity, const coord_t print_z, ExtrusionAttributes attributes,
                           coord_t z_offset, std::vector<ExportLeaf> &out)
{
    if (const ExtrusionAttributes *direct = entity.get_property<ExtrusionAttributes>()) attributes = *direct;
    if (const ExtrusionPropertyZOffset *direct = entity.get_property<ExtrusionPropertyZOffset>()) z_offset = direct->z_offset;
    if (entity.has_polyline()) {
        if (entity.polyline_ref().size() < 2 || !(attributes.mm3_per_mm > 0.) || !(attributes.width > 0.f)) return;
        if (!std::isfinite(attributes.width) || !std::isfinite(attributes.height) || attributes.height <= 0.f)
            throw std::runtime_error("Bambu export found invalid deposited geometry attributes.");
        ExtrusionEntityUPtr resolved(entity.clone());
        resolved->get_or_add_property<ExtrusionAttributes>() = attributes;
        Polygons coverage = resolved->polygons_covered_by_width(0.f);
        if (!coverage.empty()) out.push_back({print_z + z_offset, attributes.height, std::move(coverage)});
        return;
    }
    for (const ExtrusionEntityUPtr &child : entity.children())
        if (child) collect_export_leaves(*child, print_z, attributes, z_offset, out);
}

Polygons first_coverage(const std::vector<ExportLeaf> &leaves, double &height)
{
    Polygons result;
    if (leaves.empty()) return result;
    coord_t first = leaves.front().z;
    for (const ExportLeaf &leaf : leaves) first = std::min(first, leaf.z);
    height = std::numeric_limits<double>::max();
    for (const ExportLeaf &leaf : leaves) {
        if (leaf.z != first) continue;
        height = std::min(height, leaf.height);
        result.insert(result.end(), leaf.coverage.begin(), leaf.coverage.end());
    }
    return result;
}

GCodeExportFootprint export_footprint(const Polygons &polygons, const double height)
{
    GCodeExportFootprint out;
    BoundingBox box;
    for (const ExPolygon &area : union_ex(polygons)) {
        box.merge(area.contour.points);
        out.area += area.area() * SCALING_FACTOR * SCALING_FACTOR;
    }
    if (box.defined) {
        out.present = true;
        out.bbox = {unscaled(box.min.x()), unscaled(box.min.y()), unscaled(box.max.x()), unscaled(box.max.y())};
        out.layer_height = height;
    }
    return out;
}

void record_export_sources(const PrintObject &object, std::map<const LayerRegionIsland *, const Layer *> &sources)
{
    for (size_t idx = 0; idx < object.layer_count() + object.auxiliary_layer_count(); ++idx) {
        const Layer &layer = idx < object.layer_count() ? object.layer(idx) : object.auxiliary_layer(idx - object.layer_count());
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands()) sources.emplace(&region, &layer);
    }
}
}

GCodeExportDescription copy_gcode_export_objects(const print_handle *handle)
{
    const Print *print = reinterpret_cast<const Print *>(handle);
    if (print == nullptr || print->printing_plan() == nullptr)
        throw std::runtime_error("Bambu export requires a finalized printing plan.");
    GCodeExportDescription result;
    const Printing::PrintingPlan &plan = *print->printing_plan();
    const Printing::PlanFacts facts = Printing::summarize_plan(&plan);
    result.used_tools.assign(facts.used_tools.begin(), facts.used_tools.end());
    result.initial_tool = facts.initial_tool;
    std::map<const PrintObject *, uint32_t> object_indices;
    for (uint32_t idx = 0; idx < print->objects().size(); ++idx) object_indices.emplace(&print->object(idx), idx);
    for (uint32_t id = 0; id < plan.object_instances.size(); ++id) {
        const Printing::PrintingObjectInstance &instance = plan.object_instances[id];
        GCodeExportObject copied;
        copied.id = id;
        copied.name = instance.name;
        copied.real = instance.object != nullptr;
        if (copied.real) {
            const std::map<const PrintObject *, uint32_t>::const_iterator found = object_indices.find(instance.object);
            if (found == object_indices.end() || instance.object_instance_idx >= instance.object->instances().size())
                throw std::runtime_error("Bambu export found a stale object instance in the printing catalog.");
            copied.object_index = found->second;
            copied.instance_index = uint32_t(instance.object_instance_idx);
        }
        result.objects.push_back(std::move(copied));
    }
    // Recover the exact source layer without assuming the first region owns every extrusion.
    std::map<const LayerRegionIsland *, const Layer *> sources;
    for (const PrintObject &object : print->objects()) record_export_sources(object, sources);
    if (print->auxiliary_object()) record_export_sources(*print->auxiliary_object(), sources);
    std::vector<std::vector<ExportLeaf>> brims(result.objects.size()), supports(result.objects.size()), towers(result.objects.size());
    Polygons global_brim;
    for (const Printing::PrintingGroup &group : plan.groups)
        for (const Printing::PrintingLayerGroup &layer : group.layers)
            for (const Printing::PrintingToolGroup &tool : layer.tool_groups)
                for (const Printing::PrintingExtrusion &extrusion : tool.extrusions) {
                    if (!extrusion.root) continue;
                    const bool tower = extrusion.sregion_island_role.is_wipetower();
                    const std::map<const LayerRegionIsland *, const Layer *>::const_iterator source = sources.find(extrusion.region_island);
                    const bool brim = source != sources.end() && LayerAdhesionProperty::layer_is_brim(*source->second);
                    const bool support = source != sources.end() && source->second->get_property<LayerSupportProperty>() != nullptr;
                    if (!tower && !brim && !support) continue;
                    std::vector<ExportLeaf> leaves;
                    collect_export_leaves(*extrusion.root, layer.print_z, ExtrusionAttributes(), 0, leaves);
                    if (leaves.empty()) continue;
                    const uint32_t id = extrusion.printing_catalog_id;
                    if (brim && source->second->object() == print->auxiliary_object()) {
                        for (const ExportLeaf &leaf : leaves) global_brim.insert(global_brim.end(), leaf.coverage.begin(), leaf.coverage.end());
                        continue;
                    }
                    if (id >= result.objects.size() || (tower ? result.objects[id].real : !result.objects[id].real))
                        throw std::runtime_error("Bambu export cannot attribute generated geometry to its printing catalog instance.");
                    std::vector<ExportLeaf> &destination = tower ? towers[id] : brim ? brims[id] : supports[id];
                    destination.insert(destination.end(), std::make_move_iterator(leaves.begin()), std::make_move_iterator(leaves.end()));
                }
    Polygons tower_union;
    double tower_height = std::numeric_limits<double>::max();
    for (size_t idx = 0; idx < result.objects.size(); ++idx) {
        double height = 0.;
        result.objects[idx].brim = export_footprint(first_coverage(brims[idx], height));
        result.objects[idx].support = export_footprint(first_coverage(supports[idx], height));
        Polygons part = first_coverage(towers[idx], height);
        if (!part.empty()) {
            tower_height = std::min(tower_height, height);
            tower_union.insert(tower_union.end(), std::make_move_iterator(part.begin()), std::make_move_iterator(part.end()));
        }
    }
    result.global_brim = export_footprint(global_brim);
    result.wipe_tower = export_footprint(tower_union, tower_union.empty() ? 0. : tower_height);
    return result;
}
}
