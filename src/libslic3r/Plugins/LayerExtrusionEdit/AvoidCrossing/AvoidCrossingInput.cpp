#include "AvoidCrossingInput.hpp"

#include <cassert>
#include <set>
#include <utility>

#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/PrintHelpers.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"
#include "libslic3r/Plugins/Common/ClipperHelpers.hpp"

/*
This file supplies copied layer inputs to crossing tests and travel routing.
Public print views are read during construction; returned buffers retain ring and
region order without retaining those views. It does not choose or insert travels.

build_input first selects the target layer and copies its settings and slices.
default_spacing collects printing tools from every target region against every
print object's settings, including the other objects' brims, then averages their
scaled nozzle diameters. At least one tool is required. copy_region reconstructs
fill areas from islands containing that region. With several such islands it
grows them by ten scaled units and intersects with the region's raw slices before
public ensure_valid cleanup, matching the core's fill-area preparation. Top
surfaces shared by several regions are clipped to this region's raw slices.
fill_expolygons reconstructs LayerRegion::fill_expolygons(), the fill areas published
by perimeter processing. Solid surfaces and the fill used for the sparse/solid split
are restricted to raw slices. Subtracting solid surfaces gives sparse fill.
Support copies no tops. is_first_layer compares the target Z with normal object layer zero; auxiliary
layers at that exact height also qualify, while owners without normal layers do not.
Regional flow spacing and external width are averaged
over regions with raw slices, with the nozzle average as fallback.

For support targets build_input also copies the owner's object-layer perimeter slices
strictly below target Z + SCALED_EPSILON for routing. platter_blockers separately
unions last normal object slices below a supplied Z plus epsilon, translated for
real catalog instances in the current and earlier groups. Synthetic entries are
ignored. is_off_object_layer compares a layer handle with the normal layer at its Z.
build_external_input independently reads
each object's exact-Z layer for holes and the global perimeter-spacing average.
For support it appends holes from the last object layer strictly below target Z +
SCALED_EPSILON, even when that layer is at the target Z and repeats the exact-Z
holes. Holes retain their clockwise order and
instance shifts remain separate for the boundary consumer to place and reverse.
LayerIdentity first selects an owner: global_object chooses Print::auxiliary_object,
otherwise object indexes the print objects. Then auxiliary selects that owner's
auxiliary layer collection rather than its normal layer collection, and layer
indexes that collection. The global owner holds platter-space skirt and brim
layers. External geometry still visits only the ordinary print objects.
An owner without normal layers cannot mark a target as first_layer.

    build_input: retain the target and additional support boundary input
    |-- copy_layer: copy geometry and compute regional flow averages
    |   |-- is_first_layer: compare print Z with normal object layer zero
    |   |-- default_spacing: average nozzles over the collected printing tools
    |   `-- copy_region: rebuild fill and solid fill areas and select regional tops
    `-- first_layer_below_print_z: find the last object-layer index below the supplied Z
    build_external_input: collect holes, shifts and spacing across print objects
    |-- layer_at_print_z: find an exact-Z object-layer index
    |-- first_layer_below_print_z: find the last object-layer index below the supplied Z
    `-- default_spacing: supply the fallback when no exact-Z region has slices
    platter_blockers: union translated object slices from eligible printing groups
    `-- first_layer_below_print_z: select each object's last normal layer below Z plus epsilon
    is_off_object_layer: compare the normal layer at Z with the supplied layer
    tool_mask: union regional perimeter, sparse and solid areas for one tool

The caller supplies temporary API storage and keeps the print unchanged during
these reads. Returned buffers own their geometry independently of that storage.
Allocation storage may be shared; the host synchronizes its allocation bookkeeping.
*/

namespace slic3r_api { namespace LayerExtrusionEdit { namespace AvoidCrossing {
namespace pw = PluginCommon::polygon_work;
namespace clipper = PluginCommon::clipper;

// Compare scaled print Z with object layer zero; auxiliary layers at that height also qualify, and empty objects do not.
bool is_first_layer(const Object &object, const Layer &layer);

// Native union_safety_offset_ex grows fill areas by ten scaled coordinate units.
static constexpr double ClipperSafetyOffset = 10.;

// Copy layer geometry and average flows from an existing normal object or the global auxiliary object.
static LayerInput copy_layer(storage_handle *storage, const Print &print, LayerIdentity identity);
// Copy region settings and rebuild fill_expolygons() and solid fill areas in island order; temporary handles stay in caller storage.
static RegionInput copy_region(storage_handle *storage, const Layer &layer, const LayerRegion &region, bool support);
// Average scaled nozzle diameters over collected printing tools; at least one tool must be collected.
static coord_t default_spacing(const Print &print, const Object &object);
// Compare the supplied layer with its owner's normal layer at the same Z; the print's global auxiliary owner has no normal layers.
bool is_off_object_layer(const Layer &layer);
// Copy and union translated normal slices for real instances in existing groups 0 through group_idx; geometry failures propagate.
pw::ExPolygons platter_blockers(storage_handle *storage, const PrintingPlan &plan, uint32_t group_idx, coord_t print_z);
// Copy the target and the owner's lower perimeter slices for support routing into independent buffers.
Input build_input(storage_handle *storage, const Print &print, LayerIdentity target);
// Return role areas for a zero-based tool; split fill within raw slices only when infill tools differ.
// Area can also be empty for a tool owning no region; callers decide clipping from multiple_extruders, not area emptiness.
ToolMask tool_mask(storage_handle *storage, const LayerInput &layer, int extruder);

ToolMask tool_mask(storage_handle *storage, const LayerInput &layer, int extruder)
{
    ToolMask result;
    for (const RegionInput &region : layer.regions) {
        result.multiple_extruders = region.perimeter_extruder != extruder + 1 ||
            region.infill_extruder != extruder + 1 || region.solid_infill_extruder != extruder + 1;
        if (result.multiple_extruders) break;
    }
    if (!result.multiple_extruders) return result;
    for (const RegionInput &region : layer.regions) {
        if (region.perimeter_extruder == region.infill_extruder && region.infill_extruder == region.solid_infill_extruder) {
            if (region.perimeter_extruder == extruder + 1)
                result.area = clipper::union_ex(storage, result.area, region.raw_slices);
        } else {
            if (region.perimeter_extruder == extruder + 1)
                result.area = clipper::union_ex(storage, result.area, clipper::diff_ex(storage, region.raw_slices, region.fill_expolygons));
            // Preserve the direct fill union when its two roles share a tool.
            if (region.infill_extruder == region.solid_infill_extruder) {
                if (region.infill_extruder == extruder + 1)
                    result.area = clipper::union_ex(storage, result.area, region.fill_expolygons);
            } else {
                const pw::ExPolygons fill = clipper::intersection_ex(storage, region.fill_expolygons, region.raw_slices);
                if (region.infill_extruder == extruder + 1)
                    result.area = clipper::union_ex(storage, result.area, clipper::diff_ex(storage, fill, region.solid_fill_expolygons));
                if (region.solid_infill_extruder == extruder + 1)
                    result.area = clipper::union_ex(storage, result.area, region.solid_fill_expolygons);
            }
        }
    }
    return result;
}

std::optional<uint32_t> layer_at_print_z(const Object &object, coord_t z)
{
    for (uint32_t i = 0; i < object.layer_count(); ++i) {
        if (object.layer(i).print_z() == z)
            return i;
    }
    return std::nullopt;
}

std::optional<uint32_t> first_layer_below_print_z(const Object &object, coord_t z)
{
    std::optional<uint32_t> result;
    for (uint32_t i = 0; i < object.layer_count(); ++i) {
        if (object.layer(i).print_z() >= z)
            break;
        result = i;
    }
    return result;
}

bool is_first_layer(const Object &object, const Layer &layer)
{
    return object.layer_count() != 0 && layer.print_z() == object.layer(0).print_z();
}

Input build_input(storage_handle *storage, const Print &print, LayerIdentity target)
{
    Input input;
    // Keep the target at a stable address shared by its extruder-specific consumers.
    input.target = std::make_unique<const LayerInput>(copy_layer(storage, print, target));
    // Copy the object-layer perimeter slices used as additional boundary input for support targets.
    if (input.target->support) {
        const Object object = target.global_object ? print.auxiliary_object() : print.object(target.object);
        const std::optional<uint32_t> below = first_layer_below_print_z(object, input.target->print_z + SCALED_EPSILON);
        if (below) {
            const Layer lower = object.layer(*below);
            // Concatenate published island collections without a union, retaining their order.
            for (uint32_t i = 0; i < lower.island_count(); ++i) {
                const pw::ExPolygons slices = clipper::from_expolygon_collection_view(lower.island(i).perimeter_slices());
                input.lower_perimeter_slices.insert(input.lower_perimeter_slices.end(), slices.begin(), slices.end());
            }
        }
    }
    return input;
}

bool is_off_object_layer(const Layer &layer)
{
    const Object object = layer.object();
    const std::optional<uint32_t> normal = layer_at_print_z(object, layer.print_z());
    return !normal || object.layer(*normal).handle() != layer.handle();
}

pw::ExPolygons platter_blockers(storage_handle *storage, const PrintingPlan &plan,
                              uint32_t group_idx, coord_t print_z)
{
    pw::ExPolygons blockers;
    for (uint32_t gi = 0; gi <= group_idx; ++gi) {
        const ConstPrintingGroup group = plan.group(gi);
        for (uint32_t ii = 0; ii < group.object_instance_count(); ++ii) {
            const PrintingObjectInstance instance = plan.object_instance(group.printing_catalog_id(ii));
            if (!instance.has_object())
                continue;
            const Object object = instance.object();
            const std::optional<uint32_t> below = first_layer_below_print_z(object, print_z + SCALED_EPSILON);
            if (!below)
                continue;
            pw::ExPolygons slices = clipper::from_expolygon_collection_view(object.layer(*below).slices());
            const c_point shift = object.instance_shift(instance.object_instance_idx());
            for (pw::ExPolygon &polygon : slices) {
                for (pw::Point &point : polygon.contour.points)
                    point += pw::Point(shift.x, shift.y);
                for (pw::Polygon &hole : polygon.holes)
                    for (pw::Point &point : hole.points)
                        point += pw::Point(shift.x, shift.y);
            }
            blockers.insert(blockers.end(), slices.begin(), slices.end());
        }
    }
    return clipper::union_ex(storage, blockers);
}

ExternalInput build_external_input(const Print &print, LayerIdentity target)
{
    ExternalInput input;
    const Object target_object = target.global_object ? print.auxiliary_object() : print.object(target.object);
    const Layer target_layer = target.auxiliary ? target_object.auxiliary_layer(target.layer) : target_object.layer(target.layer);
    const coord_t z = target_layer.print_z();
    const bool support = target_layer.properties().get(LayerSupportProperty::key) != nullptr;
    size_t count = 0;
    coord_t spacing = 0;
    for (uint32_t oi = 0; oi < print.object_count(); ++oi) {
        const Object object = print.object(oi);
        ExternalObjectInput external;
        external.object = oi;
        const std::optional<uint32_t> same = layer_at_print_z(object, z);
        if (same) {
            const Layer layer = object.layer(*same);
            const pw::ExPolygons slices = clipper::from_expolygon_collection_view(layer.slices());
            for (const pw::ExPolygon &slice : slices)
                external.holes.insert(external.holes.end(), slice.holes.begin(), slice.holes.end());
            for (uint32_t ri = 0; ri < layer.region_count(); ++ri) {
                const LayerRegion region = layer.region(ri);
                if (!region.slices().empty()) {
                    spacing += region.flow(RAW_EXTRUSION_ROLE_PERIMETER).spacing;
                    ++count;
                }
            }
        }
        // Append the predecessor of Z + epsilon even when it is the same-Z layer already copied.
        if (support) {
            const std::optional<uint32_t> below = first_layer_below_print_z(object, z + SCALED_EPSILON);
            if (below) {
                const pw::ExPolygons slices = clipper::from_expolygon_collection_view(object.layer(*below).slices());
                for (const pw::ExPolygon &slice : slices)
                    external.holes.insert(external.holes.end(), slice.holes.begin(), slice.holes.end());
            }
        }
        for (uint32_t ii = 0; ii < object.instance_count(); ++ii)
            external.instance_shifts.emplace_back(object.instance_shift(ii));
        input.external_objects.push_back(std::move(external));
    }
    input.external_perimeter_spacing = count == 0 ? default_spacing(print, target_object) : spacing / count;
    return input;
}

static LayerInput copy_layer(storage_handle *storage, const Print &print, LayerIdentity identity)
{
    const Object object = identity.global_object ? print.auxiliary_object() : print.object(identity.object);
    const Layer layer = identity.auxiliary ? object.auxiliary_layer(identity.layer) : object.layer(identity.layer);
    LayerInput out;
    out.identity = identity;
    out.print_z = layer.print_z();
    out.support = layer.properties().get(LayerSupportProperty::key) != nullptr;
    out.first_layer = is_first_layer(object, layer);
    out.lslices = clipper::from_expolygon_collection_view(layer.slices());
    for (uint32_t i = 0; i < layer.island_count(); ++i) {
        const pw::ExPolygons slices = clipper::from_expolygon_collection_view(layer.island(i).perimeter_slices());
        out.perimeter_slices.insert(out.perimeter_slices.end(), slices.begin(), slices.end());
    }
    out.default_perimeter_spacing = default_spacing(print, object);
    size_t count = 0;
    for (uint32_t ri = 0; ri < layer.region_count(); ++ri) {
        out.regions.push_back(copy_region(storage, layer, layer.region(ri), out.support));
        const RegionInput &region = out.regions.back();
        if (!region.raw_slices.empty()) {
            const coord_t perimeter_spacing = layer.region(ri).flow(RAW_EXTRUSION_ROLE_PERIMETER).spacing;
            const coord_t external_perimeter_width = layer.region(ri).flow(RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER).width;
            out.perimeter_spacing += perimeter_spacing;
            out.external_perimeter_width += float(external_perimeter_width);
            ++count;
        }
    }
    if (count != 0) {
        out.perimeter_spacing /= count;
        out.external_perimeter_width /= float(count);
    } else {
        out.perimeter_spacing = out.default_perimeter_spacing;
        out.external_perimeter_width = float(out.default_perimeter_spacing);
    }
    return out;
}

static RegionInput copy_region(storage_handle *storage, const Layer &layer, const LayerRegion &region, bool support)
{
    RegionInput out;
    out.raw_slices = clipper::from_expolygon_collection_view(region.slices());
    const Config config = region.print_region().config();
    out.perimeter_extruder = config.get("perimeter_extruder").get_int();
    out.infill_extruder = config.get("infill_extruder").get_int();
    out.solid_infill_extruder = config.get("solid_infill_extruder").get_int();
    out.avoid_crossing_top = config.get("avoid_crossing_top").get_bool();
    out.perimeters = config.get("perimeters").get_int();
    out.ironing = config.get("ironing").get_bool();
    out.avoid_travel_island = config.get("avoid_travel_island").get_bool();
    out.avoid_travel_island_weight = config.get("avoid_travel_island_weight").get_float();
    size_t added = 0;
    for (uint32_t i = 0; i < layer.island_count(); ++i) {
        const LayerIsland island = layer.island(i);
        for (uint32_t ri = 0; ri < island.region_count(); ++ri) {
            if (island.region(ri).handle() == region.handle()) {
                const pw::ExPolygons areas = clipper::from_expolygon_collection_view(island.infill_areas());
                out.fill_expolygons.insert(out.fill_expolygons.end(), areas.begin(), areas.end());
                ++added;
                break;
            }
        }
        for (uint32_t gi = 0; gi < island.region_island_count(); ++gi) {
            const LayerRegionIsland group = island.region_island(gi);
            bool contains = false;
            for (uint32_t ri = 0; ri < group.region_count(); ++ri)
                contains |= group.region(ri).handle() == region.handle();
            if (!contains)
                continue;
            for (const Surface surface : group.fill_surfaces()) {
                if (surface_type_is_solid(surface.type()))
                    out.solid_fill_expolygons.push_back(clipper::from_expolygon_view(surface.expolygon()));
                // Support inputs omit tops rather than treating the support subject as an object top surface.
                if (support || !surface_type_is_top(surface.type()))
                    continue;
                pw::ExPolygons tops{clipper::from_expolygon_view(surface.expolygon())};
                // Shared top surfaces must be restricted to this region before crossing and boundary preparation.
                if (group.region_count() > 1)
                    tops = clipper::intersection_ex(storage, tops, out.raw_slices);
                out.top_surfaces.insert(out.top_surfaces.end(), tops.begin(), tops.end());
            }
        }
    }
    if (added > 1) {
        // Merge nearby fill areas before restricting the combined islands to this region's raw slices.
        out.fill_expolygons = clipper::offset_ex(storage, out.fill_expolygons, ClipperSafetyOffset);
        out.fill_expolygons = clipper::intersection_ex(storage, out.fill_expolygons, out.raw_slices);
    }
    // Run the public cleanup to match the core fill-area validity cleanup before copying the result.
    StoredExPolygonCollection valid = clipper::to_stored_expolygons(storage, out.fill_expolygons);
    valid.ensure_valid();
    out.fill_expolygons = clipper::from_expolygon_collection_view(valid.readonly());
    out.solid_fill_expolygons = clipper::intersection_ex(storage, out.solid_fill_expolygons,
        clipper::intersection_ex(storage, out.raw_slices, out.fill_expolygons));
    return out;
}

static coord_t default_spacing(const Print &print, const Object &object)
{
    std::set<extruder_index_t> tools;
    // Combine each target region with every object configuration so other objects' brim tools are included.
    for (uint32_t ri = 0; ri < object.print_region_count(); ++ri)
        for (uint32_t oi = 0; oi < print.object_count(); ++oi)
            collect_object_printing_extruders(print, print.object(oi), object.print_region(ri), tools);
    assert(!tools.empty());
    coord_t sum = 0;
    for (extruder_index_t tool : tools)
        sum += scale_i(print.config().get("nozzle_diameter").get_float(tool));
    return sum / tools.size();
}

}}}
