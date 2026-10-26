///|/ Copyright (c) Prusa Research 2016-2023 Vojtech Bubnik and contributors.
///|/ Copyright (c) SuperSlicer 2023 Remi Durand.
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#include "GridSupportGenerator.hpp"

#include <agg/agg_path_storage.h>
#include <agg/agg_pixfmt_gray.h>
#include <agg/agg_rasterizer_scanline_aa.h>
#include <agg/agg_renderer_scanline.h>
#include <agg/agg_scanline_p.h>
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/MutablePolygon.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/PrintRegion.hpp"
#include "libslic3r/Slicing.hpp"

/* A contact is a dense slab next to the object; a column is its projection
   towards the bed. Grid support rasterizes columns into small AGG pixels and
   fills larger grid cells without crossing the object mask. Snug support uses
   closing and smoothing instead. Z slabs divide the distance between contacts;
   interfaces take portions of base slabs near contacts. All geometry here is
   temporary and belongs to GeneratedSupport. Demand already includes painting,
   modifiers, bridge removal and the buildplate-only filter, so it follows one
   projection path rather than a separate enforced-support path. Region raw
   slices supply the object footprint; subtracting the upper layer finds exposed
   tops including their perimeter strips. Typed fill surfaces live in island
   work regions and supply bridge footprints. height_block is the downward Z
   span used for object collisions, independently of printable height. */

namespace Slic3r::SupportGrid {
namespace {
constexpr ClipperLib::JoinType join_type = ClipperLib::jtSquare;
struct Grid {
    const Polygons &source, &mask;
    SupportMaterialStyle style;
    double angle, width, closing_radius, pixel_size = 0;
    BoundingBox bbox;
    Vec2i32 size;
    Polygons rotated_source, rotated_mask;
    std::vector<unsigned char> pixels;
    // Prepare raster cells for grid support, or retain contours for snug support.
    Grid(const Polygons &source, const Polygons &mask, const PrintObjectConfig &config, const Flow &flow);
    // Extract expanded print contours or slightly shrunken propagation contours.
    Polygons extract(coord_t offset, bool fill_holes) const;
};
// Build and merge top-contact slabs from the sliced object's island demand.
void top_contacts(const PrintObject &object, const GridSupportParams &params, GeneratedSupport &out);
// Project each top contact downwards and create bottom contacts on object top surfaces.
std::vector<Polygons> project_columns(const PrintObject &object, const GridSupportParams &params, GeneratedSupport &out);
// Place intermediate slabs between contact extremes and assign missing top-contact heights.
void intermediate_layers(const PrintObject &object, const GridSupportParams &params, GeneratedSupport &out);
// Subtract object slices and bridge footprints from slabs with the configured clearance.
void trim_by_object(const PrintObject &object, const GridSupportParams &params, GeneratorLayers &layers);
// Fill intermediate slabs from projected columns and remove portions inside contacts.
void base_layers(const PrintObject &object, const GridSupportParams &params, const std::vector<Polygons> &areas, GeneratedSupport &out);
// Carve dense top and bottom interface regions from the sparse base slabs.
void interface_layers(const PrintObject &object, const GridSupportParams &params, GeneratedSupport &out);
// Expand column feet in several steps while avoiding object and brim areas.
void expand_feet(const PrintObject &object, const GridSupportParams &params, GeneratedSupport &out);
// Rasterize polygons into a zero-bordered grayscale image using AGG's winding fill.
std::vector<unsigned char> rasterize(const Vec2i32 &size, double pixel, const Point &origin, const Polygons &polygons);
// Chain raster boundary edges into scaled contours, optionally closing single-pixel holes.
Polygons contours(const Grid &grid, coord_t offset, bool fill_holes);
// Detect bridge extrusion paths so thicker object bridges get a separate contact Z.
bool has_bridges(const Layer &layer);
// Detect bridge paths with the nearest attributes in a generic extrusion tree.
bool bridging_fills(const ExtrusionEntity &entity, const ExtrusionAttributes *attributes = nullptr);
// Collect the XY footprint of overhang perimeter paths, enlarged by the object clearance.
void bridge_areas(const ExtrusionEntity &entity, coord_t gap, ExPolygons &out, const ExtrusionAttributes *attributes = nullptr);
// Allocate a slab owned by the result's deque and initialize its integer Z dimensions.
GeneratorLayer &add_layer(GeneratedSupport &out, LayerType type, coord_t z, coord_t bottom, coord_t height);
// Merge close top contacts into the lower contact, preserving the first-layer thickness.
void merge_contacts(const GridSupportParams &params, GeneratorLayers &layers);
// Return the number of equal steps needed to cover a positive distance.
size_t ceil_steps(coord_t distance, coord_t step);
} // namespace

// Run geometric phases on a sliced object with demand; the result owns every slab and the object receives no layers or extrusions here.
void generate_support(const PrintObject &object, const GridSupportParams &params, GeneratedSupport &out)
{
    top_contacts(object, params, out);
    if (out.top.empty()) return;
    const std::vector<Polygons> areas = project_columns(object, params, out);
    intermediate_layers(object, params, out);
    trim_by_object(object, params, out.top);
    base_layers(object, params, areas, out);
    for (GeneratorLayer *top : out.top)
        for (const GeneratorLayer *bottom : out.bottom)
            if (bottom->bottom_print_z() <= top->bottom_z && top->print_z <= bottom->print_z)
                top->polygons = diff(top->polygons, bottom->polygons);
    interface_layers(object, params, out);
    expand_feet(object, params, out);
}

namespace {
// Intersect demand with each region, apply the active lower-slice margin, and rasterize contacts; allocated slabs remain in out even when merging empties them.
void top_contacts(const PrintObject &object, const GridSupportParams &p, GeneratedSupport &out)
{
    const PrintObjectConfig &c = object.config();
    const coord_t first_z = scale_to_layer_coord(p.first_layer_height);
    for (size_t id = 1; id < object.layer_count(); ++id) {
        object.print()->throw_if_canceled();
        const Layer &layer = object.layer(id);
        const Layer &lower = object.layer(id - 1);
        Polygons demand;
        for (const LayerSliceIsland &island : layer.islands())
            polygons_append(demand, island.support_demand());
        if (demand.empty()) continue;
        const Polygons lower_polygons = to_polygons(lower.lslices());
        Polygons overhangs, contacts, margin;
        float no_interface_offset = FLT_MAX;
        for (const LayerRegion &region : layer.regions())
            no_interface_offset = std::min(no_interface_offset, float(region.flow(frExternalPerimeter).scaled_width()));
        const double threshold = c.support_material_threshold.value > 0 ?
            M_PI * double(c.support_material_threshold.value + 1) / 180. : 0.;
        float cached_offset = -1;
        for (const LayerRegion &region : layer.regions()) {
            ExPolygons raw = intersection_ex(demand, region.get_raw_slices());
            if (raw.empty()) continue;
            polygons_append(overhangs, raw);
            const coord_t lower_offset = id < size_t(c.support_material_enforce_layers.value) ? 0 :
                threshold > 0 ? scale_to_layer_coord(lower.unscaled_height() / tan(threshold)) :
                region.flow(frExternalPerimeter).scaled_width() / 2;
            const float offset = float(std::min(lower_offset, p.gap_xy));
            if (offset != cached_offset) {
                cached_offset = offset;
                margin = offset == 0 ? lower_polygons : offset2(lower.lslices(), -no_interface_offset * 0.5f,
                    offset + no_interface_offset * 0.5f, join_type, 0.);
            }
            // The active margin removes thin lower features before expanding the collision mask.
            contacts = union_(contacts, ensure_valid(diff_ex(raw, margin), p.resolution));
        }
        if (contacts.empty() && overhangs.empty()) continue;
        coord_t z = layer.scaled_bottom_z() - scale_to_layer_coord(p.gap_above);
        coord_t bottom = z, height = 0;
        if (p.soluble) {
            height = lower.scaled_height();
            bottom = id == 1 ? 0 : object.layer(id - 2).scaled_print_z();
        } else {
            if (z < first_z) continue;
            if (z < first_z + p.min_height) { z = first_z; bottom = 0; height = first_z; }
        }
        GeneratorLayer &contact = add_layer(out, LayerType::TopContact, z, bottom, height);
        contact.object_above = id;
        const Grid grid(contacts, margin, c, p.base_flow);
        contact.contact_polygons = std::make_unique<Polygons>(grid.extract(-3, true));
        ensure_valid(*contact.contact_polygons, p.resolution);
        if (c.support_material_style.value == smsGrid && !p.soluble) {
            Polygons dense = diff(overhangs, opening(lower_polygons, no_interface_offset * 0.5f,
                no_interface_offset * 1.1f, join_type, 0.));
            if (!dense.empty()) {
                dense = diff(expand(dense, no_interface_offset * 0.1f), margin);
                const Polygons trimmed = intersection(dense, *contact.contact_polygons);
                contact.polygons = Grid(trimmed, margin, c, p.base_flow).extract(p.base_flow.scaled_spacing() / 2 + 5, false);
            }
        } else
            contact.polygons = grid.extract(p.base_flow.scaled_spacing() / 2 + 5, true);
        ensure_valid(contact.polygons, p.resolution);
        contact.overhang_polygons = std::make_unique<Polygons>(overhangs);
        out.top.push_back(&contact);
        if (!p.soluble && has_bridges(layer)) {
            coord_t bridge_height = layer.scaled_height();
            if (c.support_material_contact_distance_type.value == zdFilament) {
                double total = 0;
                for (const LayerRegion &region : layer.regions()) total += region.bridging_height_avg_mm();
                bridge_height = scale_to_layer_coord(total / layer.region_count());
            }
            coord_t bridge_z = layer.scaled_print_z() - bridge_height - scale_to_layer_coord(p.gap_above);
            if (bridge_z >= first_z && bridge_z < z) {
                if (bridge_z < first_z + p.min_height) bridge_z = first_z;
                GeneratorLayer &bridge = add_layer(out, LayerType::TopContact, bridge_z,
                    bridge_z == first_z ? 0 : bridge_z, bridge_z == first_z ? first_z : 0);
                bridge.object_above = id;
                bridge.polygons = contact.polygons;
                bridge.contact_polygons = std::make_unique<Polygons>(*contact.contact_polygons);
                bridge.overhang_polygons = std::make_unique<Polygons>(*contact.overhang_polygons);
                out.top.push_back(&bridge);
            }
        }
    }
    merge_contacts(p, out.top);
}

// Project a sliced object's owned contacts into one area per object layer; bottom collision spans use the next object height for plane gaps or initial contact height otherwise, unchanged by Z alignment; cancellation propagates.
std::vector<Polygons> project_columns(const PrintObject &object, const GridSupportParams &p, GeneratedSupport &out)
{
    const PrintObjectConfig &c = object.config();
    std::vector<Polygons> areas(object.layer_count());
    Polygons projection;
    int contact_idx = int(out.top.size()) - 1;
    for (int id = int(object.layer_count()) - 2; id >= 0; --id) {
        object.print()->throw_if_canceled();
        const Layer &layer = object.layer(id);
        for (; contact_idx >= 0 && out.top[contact_idx]->print_z >= layer.scaled_print_z(); --contact_idx) {
            GeneratorLayer &top = *out.top[contact_idx];
            Polygons polygons = std::move(*top.contact_polygons);
            polygons_append(polygons, expand(*top.overhang_polygons, double(SCALED_EPSILON)));
            polygons_append(projection, union_(polygons));
        }
        if (projection.empty()) continue;
        projection = union_(projection);
        ensure_valid(projection, p.resolution);
        const Polygons top_surfaces = diff(layer.lslices(), object.layer(id + 1).lslices());
        Polygons touching = intersection(top_surfaces, projection);
        ensure_valid(touching, p.resolution);
        if (!touching.empty()) {
            const coord_t height = p.soluble ? layer.upper_layer->scaled_height() : scale_to_layer_coord(p.bottom_flow.height());
            const coord_t height_block = c.support_material_contact_distance_type.value == zdPlane ?
                object.layer(id + 1).scaled_height() : height;
            const coord_t bottom = layer.scaled_print_z() + scale_to_layer_coord(p.gap_below);
            const coord_t z = p.soluble ? layer.upper_layer->scaled_print_z() : bottom + height_block;
            GeneratorLayer &contact = add_layer(out, LayerType::BottomContact, z, bottom, z - bottom);
            contact.height_block = height_block;
            contact.object_below = id;
            contact.bridging = !p.soluble;
            contact.polygons = touching;
            if (p.bottom_expansion.value > 0)
                contact.polygons = intersection(top_surfaces, expand(touching,
                    scale_d(p.bottom_expansion.get_effective_value(p.bottom_flow.width())), join_type, 0.));
            if (!p.soluble) {
                for (size_t idx = size_t(std::max(0, contact_idx)); idx < out.top.size() && out.top[idx]->print_z <= z + p.min_height; ++idx) {
                    const coord_t height = out.top[idx]->print_z - contact.bottom_z;
                    if (out.top[idx]->print_z >= z - p.min_height && height >= p.min_height &&
                        height <= scale_to_layer_coord(p.max_support_height)) {
                        contact.print_z = out.top[idx]->print_z;
                        contact.height = contact.print_z - contact.bottom_z;
                        assert(contact.height >= p.min_height);
                        break;
                    }
                }
            }
            ensure_valid(contact.polygons, p.resolution);
            out.bottom.push_back(&contact);
            touching = expand(touching, double(SCALED_EPSILON));
            for (size_t above = size_t(id + 1); above < object.layer_count() && object.layer(above).scaled_print_z() < contact.print_z; ++above)
                areas[above] = diff(areas[above], touching);
        }
        const Polygons mask = offset(layer.lslices(), double(SCALED_EPSILON));
        Polygons raw = diff(projection, mask);
        remove_sticks(raw);
        remove_degenerate(raw);
        const Grid grid(raw, mask, c, p.base_flow);
        areas[id] = grid.extract(p.base_flow.scaled_spacing() / 2 + 5, true);
        projection = grid.extract(-3, true);
    }
    std::reverse(out.bottom.begin(), out.bottom.end());
    trim_by_object(object, p, out.bottom);
    return areas;
}

// Divide owned contact intervals into printable slabs, sharing synchronized heights and rounding inserted boundaries to the machine step; only contacts below the first-layer height acquire collision spans, and incompatible thicknesses assert.
void intermediate_layers(const PrintObject &object, const GridSupportParams &p, GeneratedSupport &out)
{
    const PrintObjectConfig &c = object.config();
    const bool sync = c.support_material_synchronize_layers.value;
    if (sync) {
        // Contacts at one print Z need a common printable bottom, otherwise a thinner top slab would shorten the published bottom contact's gap.
        for (GeneratorLayer *top : out.top)
            for (const GeneratorLayer *bottom : out.bottom)
                if (top->height == 0 && top->print_z == bottom->print_z) {
                    top->bottom_z = bottom->bottom_z;
                    top->height = bottom->height;
                }
    }
    GeneratorLayers extremes = out.top;
    append(extremes, out.bottom);
    std::sort(extremes.begin(), extremes.end(), [](const GeneratorLayer *a, const GeneratorLayer *b) {
        return a->extreme_z() < b->extreme_z() || (a->extreme_z() == b->extreme_z() &&
            a->type == LayerType::TopContact && b->type == LayerType::BottomContact);
    });
    const coord_t first = scale_to_layer_coord(p.first_layer_height);
    const coord_t base_step = scale_to_layer_coord(c.support_material_layer_height.value == 0 ? p.max_support_height :
        std::min(p.max_support_height, c.support_material_layer_height.get_effective_value(p.base_flow.nozzle_diameter())));
    const coord_t interface_step = scale_to_layer_coord(c.support_material_interface_layer_height.value == 0 ? p.max_support_height :
        std::min(p.max_support_height, c.support_material_interface_layer_height.get_effective_value(p.interface_flow.nozzle_diameter())));
    const coord_t machine_step = scale_to_layer_coord(p.z_step);
    coord_t previous_extreme = 0;
    GeneratorLayer *previous = nullptr;
    for (GeneratorLayer *current : extremes) {
        coord_t end = current->extreme_z();
        if (end < first) {
            assert(current->type == LayerType::TopContact);
            current->height = current->print_z;
            // The first bed contact already has its thickness and is excluded from collision-span planning.
            if (current != extremes.front() || current->bottom_z != 0)
                current->height_block = current->print_z;
            previous = current;
            previous_extreme = end;
            continue;
        }
        coord_t start = previous_extreme;
        if (start == 0) {
            if (out.base.empty() || out.base.back()->print_z < first)
                out.base.push_back(&add_layer(out, LayerType::Intermediate, first, 0, first));
            start = first;
        }
        if (end <= start) { previous = current; previous_extreme = end; continue; }
        coord_t distance = end - start;
        assert(distance >= p.min_height);
        if (sync) {
            if (previous && previous->type == LayerType::TopContact && current->type == LayerType::BottomContact &&
                previous->print_z == current->print_z && previous->bottom_z == current->bottom_z) {
                // This interval is the shared contact slab, not several smaller object-Z slabs; sparse columns at the same Z keep that thickness too.
                out.base.push_back(&add_layer(out, LayerType::Intermediate, end, start, distance));
                previous = current;
                previous_extreme = end;
                continue;
            }
            // Soluble contacts already have a bottom on an object Z. Non-soluble contacts retain their top Z and acquire a printable thickness here.
            const bool unknown_top = current->type == LayerType::TopContact && current->height == 0;
            coord_t last = start;
            for (const Layer &layer : object.layers()) {
                const coord_t z = layer.scaled_print_z();
                if (z <= start) continue;
                if (z > end || (unknown_top && z == end)) break;
                if (z - last < p.min_height) continue;
                out.base.push_back(&add_layer(out, LayerType::Intermediate, z, last, z - last));
                last = z;
            }
            if (unknown_top) {
                if (end - last < p.min_height && !out.base.empty() && out.base.back()->print_z == last) {
                    last = out.base.back()->bottom_z;
                    out.base.pop_back();
                }
                // Round cumulative boundaries, not one repeated thickness, so residual machine steps reach the unchanged contact Z.
                const coord_t limit = machine_step > SCALED_EPSILON ? interface_step - interface_step % machine_step : interface_step;
                const size_t count = ceil_steps(end - last, limit);
                const coord_t start_z = last;
                for (size_t i = 1; i < count; ++i) {
                    const coord_t next = check_z_step(scale_to_layer_coord(unscaled(start_z) + unscaled(end - start_z) * i / count), machine_step);
                    assert(next - last >= p.min_height && next - last <= interface_step);
                    out.base.push_back(&add_layer(out, LayerType::Intermediate, next, last, next - last));
                    last = next;
                }
                current->bottom_z = last;
                current->height = end - last;
                assert(current->height >= p.min_height && current->height <= interface_step);
                end = last;
            } else if (last < end) {
                // A bottom contact keeps its gap above the object; close the interval at its top before returning to object Z positions.
                if (end - last < p.min_height && !out.base.empty() && out.base.back()->print_z == last) {
                    last = out.base.back()->bottom_z;
                    out.base.pop_back();
                }
                const coord_t limit = machine_step > SCALED_EPSILON ? base_step - base_step % machine_step : base_step;
                const size_t count = ceil_steps(end - last, limit);
                const coord_t start_z = last;
                for (size_t i = 1; i <= count; ++i) {
                    const coord_t z = i == count ? end : check_z_step(scale_to_layer_coord(unscaled(start_z) + unscaled(end - start_z) * i / count), machine_step);
                    assert(z - last >= p.min_height && z - last <= base_step);
                    out.base.push_back(&add_layer(out, LayerType::Intermediate, z, last, z - last));
                    last = z;
                }
            }
        } else {
            // Distribute the interval across the requested top, bottom and sparse thicknesses, as in native support generation.
            size_t top = size_t(std::max(0, c.support_material_interface_layers.value));
            const int bottom_count = c.support_material_bottom_interface_layers.is_enabled() ?
                c.support_material_bottom_interface_layers.value : int(top);
            size_t bottom = bottom_count > 0 ? size_t(bottom_count - 1) : 0;
            size_t middle = 0;
            double step = 0, dense_step = interface_step;
            const coord_t dense_height = interface_step * coord_t(top + bottom);
            if (distance <= dense_height - EPSILON) {
                top = ceil_steps(distance, interface_step);
                dense_step = double(distance) / top;
                bottom = std::min(bottom, top / 2);
                top -= bottom;
            } else {
                middle = ceil_steps(distance - dense_height, base_step);
                step = double(distance) / (top + bottom + middle);
                if (step <= base_step && step <= interface_step) dense_step = step;
                else if (interface_step <= base_step) step = (distance - dense_step * (top + bottom)) / middle;
                else { step = base_step; dense_step = (distance - step * middle) / (top + bottom); }
            }
            size_t total = top + bottom + middle;
            if (previous && previous->type == LayerType::TopContact &&
                previous->print_z + p.min_height > previous->bottom_z + scale_to_layer_coord(unscaled(step))) {
                start = previous->print_z;
                out.base.push_back(&add_layer(out, LayerType::Intermediate, start, previous->bottom_z, previous->height));
                distance = end - start;
                if (distance == 0) { previous = current; previous_extreme = end; continue; }
                --total;
                if (bottom > 0) --bottom;
                else if (middle > 0) --middle;
                else --top;
                const coord_t remaining_dense = interface_step * coord_t(top + bottom);
                if (distance <= remaining_dense) {
                    top = ceil_steps(distance, interface_step);
                    dense_step = double(distance) / top;
                    middle = 0;
                    bottom = std::min(bottom, top - 1);
                    top -= bottom;
                } else {
                    middle = ceil_steps(distance - remaining_dense, base_step);
                    step = double(distance) / (top + bottom + middle);
                    if (step <= base_step && step <= interface_step) dense_step = step;
                    else if (interface_step <= base_step) { dense_step = interface_step; step = double(distance - remaining_dense) / middle; }
                    else { step = base_step; dense_step = double(remaining_dense) / (top + bottom); }
                }
                total = top + bottom + middle;
            }
            if (!p.soluble && current->type == LayerType::TopContact && current->height == 0) {
                if (top > 0) { --top; current->height = scale_to_layer_coord(unscaled(dense_step)); }
                else if (middle > 0) { --middle; current->height = scale_to_layer_coord(unscaled(step)); }
                else { --bottom; current->height = scale_to_layer_coord(unscaled(dense_step)); }
                end = current->bottom_z = current->print_z - current->height;
                --total;
            }
            coord_t last = start;
            double wanted = unscaled(start);
            for (size_t i = 0; i < total; ++i) {
                coord_t z = end;
                if (i + 1 < total) {
                    const bool dense = i < bottom || i >= total - top;
                    wanted += unscaled(dense ? dense_step : step);
                    z = last + std::min(check_z_step(scale_to_layer_coord(wanted) - last,
                        scale_to_layer_coord(p.z_step)), dense ? interface_step : base_step);
                }
                assert(z > last);
                out.base.push_back(&add_layer(out, LayerType::Intermediate, z, last, z - last));
                last = z;
            }
        }
        previous = current;
        previous_extreme = end;
    }
    for (const GeneratorLayer *layer : out.top) assert(layer->height > 0);
}

// Remove object silhouettes within the slab's gap-expanded Z range, including thick bridge fills and perimeter footprints above it; slab geometry is replaced in place.
void trim_by_object(const PrintObject &object, const GridSupportParams &p, GeneratorLayers &layers)
{
    for (GeneratorLayer *support : layers) {
        if (support->polygons.empty()) continue;
        ExPolygons trimming;
        size_t id = 0;
        const coord_t lower = support->print_z - support->height_block - scale_to_layer_coord(p.gap_below);
        while (id < object.layer_count() && object.layer(id).scaled_print_z() <= lower) ++id;
        const coord_t upper = support->print_z + scale_to_layer_coord(p.gap_above);
        for (; id < object.layer_count(); ++id) {
            const Layer &layer = object.layer(id);
            if (layer.scaled_bottom_z() >= upper) break;
            append(trimming, offset_ex(layer.lslices(), double(p.gap_xy), join_type, 0.));
        }
        if (!p.soluble) {
            for (; id < object.layer_count(); ++id) {
                const Layer &layer = object.layer(id);
                bool overlaps = false;
                for (const LayerRegion &region : layer.regions()) {
                    const coord_t bridge_height = object.config().support_material_contact_distance_type.value == zdFilament ?
                        scale_to_layer_coord(region.bridging_height_avg_mm()) : layer.scaled_height();
                    if (layer.scaled_print_z() - bridge_height >= upper) break;
                    overlaps = true;
                    for (const LayerSliceIsland &island : layer.islands())
                        for (const LayerRegionIsland &work : island.regions_islands())
                            if (work.regions().count(&region))
                                append(trimming, offset_ex(to_expolygons(work.fill_surfaces().filter_by_type(stPosBottom | stDensSolid | stModBridge)), p.gap_xy, join_type, 0.));
                }
                for (const LayerSliceIsland &island : layer.islands())
                    for (const LayerRegionIsland &region : island.regions_islands()) {
                        if (!region.has_extrusion(LayerRegionIsland::PERIMETERS)) continue;
                        bool too_high = false;
                        for (const LayerRegion *lr : region.regions()) {
                            const coord_t bridge_height = object.config().support_material_contact_distance_type.value == zdFilament ?
                                scale_to_layer_coord(lr->bridging_height_avg_mm()) : layer.scaled_height();
                            if (layer.scaled_print_z() - bridge_height >= upper) { too_high = true; break; }
                        }
                        if (!too_high) bridge_areas(region.extrusion(LayerRegionIsland::PERIMETERS), p.gap_xy, trimming);
                    }
                if (!overlaps) break;
            }
        }
        ensure_valid(trimming, p.resolution);
        support->polygons = diff(support->polygons, union_ex(trimming));
        ensure_valid(support->polygons, p.resolution);
    }
}

// Select the cached projection at each slab's top, subtract enclosing contacts, and trim against the object; base slabs stay owned by out.
void base_layers(const PrintObject &object, const GridSupportParams &p, const std::vector<Polygons> &areas, GeneratedSupport &out)
{
    for (GeneratorLayer *base : out.base) {
        int below = -1;
        for (size_t id = 0; id < object.layer_count() && object.layer(id).scaled_print_z() <= base->print_z; ++id) below = int(id);
        Polygons polygons = below < 0 ? areas.front() : areas[below];
        if (below < 0)
            for (const GeneratorLayer *top : out.top)
                if (top->bottom_z >= base->print_z && top->print_z <= object.layer(0).scaled_print_z()) polygons_append(polygons, top->polygons);
        Polygons trimming;
        for (const GeneratorLayer *top : out.top) {
            assert(!(base->print_z > top->bottom_z + SCALED_EPSILON && base->bottom_z < top->bottom_z - SCALED_EPSILON));
            if (top->bottom_z < base->print_z && top->print_z > base->bottom_z &&
                base->print_z <= top->print_z && base->bottom_z >= top->bottom_z) polygons_append(trimming, top->polygons);
        }
        for (const GeneratorLayer *bottom : out.bottom) {
            assert(!(base->print_z > bottom->print_z + SCALED_EPSILON && base->bottom_z < bottom->print_z - SCALED_EPSILON));
            if (bottom->bottom_print_z() < base->print_z && bottom->print_z > base->bottom_print_z() &&
                base->print_z <= bottom->print_z && base->bottom_z >= bottom->bottom_print_z()) polygons_append(trimming, bottom->polygons);
        }
        base->polygons = trimming.empty() ? std::move(polygons) : diff(polygons, trimming, ApplySafetyOffset::Yes);
        ensure_valid(base->polygons, p.resolution);
        base->type = LayerType::Base;
    }
    trim_by_object(object, p, out.base);
}

// Project contacts through the configured number of base slabs and regularize their union; top and bottom regions are separated after union so their published surface types stay distinct.
void interface_layers(const PrintObject &object, const GridSupportParams &p, GeneratedSupport &out)
{
    const bool snug = object.config().support_material_style.value == smsSnug;
    const double smoothing = p.interface_flow.scaled_spacing() * 1.5;
    const double radius = p.interface_flow.scaled_spacing() / p.interface_density;
    for (size_t i = 0; i < out.base.size(); ++i) {
        GeneratorLayer &base = *out.base[i];
        Polygons top, bottom;
        if (p.top_interfaces > 0) {
            const coord_t z = out.base[std::min(out.base.size() - 1, i + p.top_interfaces - 1)]->print_z;
            for (const GeneratorLayer *contact : out.top)
                if (contact->print_z >= base.print_z && contact->bottom_z <= z)
                    polygons_append(top, snug ? *contact->overhang_polygons : contact->polygons);
        }
        if (p.bottom_interfaces > 0) {
            const coord_t z = out.base[i + 1 > p.bottom_interfaces ? i + 1 - p.bottom_interfaces : 0]->bottom_z;
            for (const GeneratorLayer *contact : out.bottom)
                if (contact->print_z >= z && contact->print_z <= base.bottom_z) polygons_append(bottom, contact->polygons);
        }
        if (top.empty() && bottom.empty()) continue;
        Polygons united = bottom;
        append(united, top);
        united = intersection(snug ? smooth_outward(closing(united, smoothing + radius, smoothing, join_type, 0.), smoothing) :
            union_safety_offset(united), base.polygons);
        ensure_valid(united, p.resolution);
        if (united.empty()) continue;
        // When both projections meet, the regularized top projection takes the overlap and the bottom interface takes the remaining union.
        Polygons top_part = top.empty() ? Polygons() : bottom.empty() ? united : intersection(united, snug ?
            smooth_outward(closing(top, smoothing + radius, smoothing, join_type, 0.), smoothing) : union_safety_offset(top));
        Polygons bottom_part = diff(united, top_part);
        for (const std::pair<LayerType, Polygons *> &part : {
                std::pair<LayerType, Polygons *>{LayerType::TopInterface, &top_part},
                std::pair<LayerType, Polygons *>{LayerType::BottomInterface, &bottom_part}}) {
            if (part.second->empty()) continue;
            GeneratorLayer &layer = add_layer(out, part.first, base.print_z, base.bottom_z, base.height);
            layer.polygons = std::move(*part.second);
            out.interfaces.push_back(&layer);
        }
        base.polygons = diff(base.polygons, united);
        ensure_valid(base.polygons, p.resolution);
    }
}

// Enlarge the first base slab by the configured foot expansion without crossing the first object slice; subtract the configured brim from all bed-contacting slabs.
void expand_feet(const PrintObject &object, const GridSupportParams &p, GeneratedSupport &out)
{
    const PrintObjectConfig &c = object.config();
    const coord_t first = scale_to_layer_coord(p.first_layer_height);
    Polygons interface;
    for (GeneratorLayers *layers : {&out.top, &out.interfaces})
        for (const GeneratorLayer *layer : *layers)
            if (layer->bottom_print_z() == 0) polygons_append(interface, expand(layer->polygons, float(scale_d(EPSILON)), join_type, 0.));
    if (!out.base.empty() && out.base.front()->bottom_print_z() == 0) {
        Polygons &foot = out.base.front()->polygons;
        const Polygons trimming = offset(object.layer(0).lslices(), double(p.gap_xy), join_type, 0.);
        const double expansion = std::max(0., scale_d(c.raft_first_layer_expansion.value) - scale_d(EPSILON));
        if (expansion > SCALED_EPSILON) {
            const int steps = std::max(5, int(ceil(expansion / p.first_flow.scaled_width())));
            for (int i = 0; i < steps; ++i) foot = diff(expand(foot, float(expansion / steps)), trimming);
        } else foot = diff(foot, trimming);
        foot = diff(foot, interface);
        ensure_valid(foot, p.resolution);
    }
    if (object.has_brim()) {
        Polygons brim;
        const double distance = scale_d(c.brim_width.value);
        for (const ExPolygon &slice : object.layer(0).lslices()) {
            if (c.brim_width.value > 0 && c.brim_width_interior.value > 0) polygons_append(brim, offset(slice, distance));
            else {
                if (c.brim_width.value > 0) polygons_append(brim, offset(slice.contour, distance, ClipperLib::jtRound, float(scale_d(0.1))));
                else brim.push_back(slice.contour);
                Polygons holes = slice.holes;
                if (c.brim_width_interior.value > 0) {
                    polygons_reverse(holes);
                    holes = shrink(holes, distance, ClipperLib::jtRound, float(scale_d(0.1)));
                    polygons_reverse(holes);
                }
                append(brim, holes);
            }
        }
        brim = union_(brim);
        for (GeneratorLayers *layers : {&out.top, &out.base, &out.interfaces})
            for (GeneratorLayer *layer : *layers)
                if (layer->print_z <= first) layer->polygons = diff(layer->polygons, brim);
    }
}

// Map scaled polygon coordinates to grayscale pixels; holes follow polygon winding and the returned vector owns the complete raster.
std::vector<unsigned char> rasterize(const Vec2i32 &size, double pixel, const Point &origin, const Polygons &polygons)
{
    std::vector<unsigned char> data(size.x() * size.y());
    agg::rendering_buffer buffer(data.data(), unsigned(size.x()), unsigned(size.y()), size.x());
    agg::pixfmt_gray8 pixels(buffer);
    agg::renderer_base<agg::pixfmt_gray8> base(pixels);
    agg::renderer_scanline_aa_solid<agg::renderer_base<agg::pixfmt_gray8>> renderer(base);
    renderer.color(agg::pixfmt_gray8::color_type(255));
    base.clear(agg::pixfmt_gray8::color_type(0));
    agg::scanline_p8 scanline;
    agg::rasterizer_scanline_aa<> rasterizer;
    for (const Polygon &polygon : polygons) {
        if (polygon.points.empty()) continue;
        agg::path_storage path;
        const Point &first = polygon.points.front();
        path.move_to((first.x() - origin.x()) / pixel, (first.y() - origin.y()) / pixel);
        for (size_t i = 1; i < polygon.points.size(); ++i) {
            const Point &point = polygon.points[i];
            path.line_to((point.x() - origin.x()) / pixel, (point.y() - origin.y()) / pixel);
        }
        path.line_to((first.x() - origin.x()) / pixel, (first.y() - origin.y()) / pixel);
        rasterizer.add_path(std::move(path));
    }
    agg::render_scanlines(rasterizer, scanline, renderer);
    return data;
}

// Rotate grid inputs, oversample each support cell, and propagate occupied pixels while avoiding the eroded object mask; input contours must outlive this temporary grid.
Grid::Grid(const Polygons &src, const Polygons &trim, const PrintObjectConfig &c, const Flow &flow)
    : source(src), mask(trim), style(c.support_material_style.value), angle(Geometry::deg2rad(c.support_material_angle.value)),
      width(flow.spacing()), closing_radius(c.support_material_closing_radius.value)
{
    if (style != smsGrid || source.empty()) return;
    rotated_source = source;
    rotated_mask = mask;
    if (angle != 0) { polygons_rotate(rotated_source, -angle); polygons_rotate(rotated_mask, -angle); }
    const double spacing = c.support_material_spacing.value + flow.spacing();
    bbox = get_extents(rotated_source);
    bbox.offset(20);
    bbox.align_to_grid(scale_i(spacing));
    const coord_t extrusion = scale_i(width);
    const int oversampling = std::clamp(int(scale_i(spacing) / (extrusion + 100)), 1, 8);
    pixel_size = double(std::max(extrusion + 21, scale_i(spacing / oversampling)));
    bbox.offset(pixel_size);
    const Vec2i32 raw(int(ceil(double(bbox.max.x() - bbox.min.x() - 1) / pixel_size)),
                     int(ceil(double(bbox.max.y() - bbox.min.y() - 1) / pixel_size)));
    const Vec2i32 blocks((raw.x() + oversampling - 3) / oversampling, (raw.y() + oversampling - 3) / oversampling);
    size = blocks * oversampling + Vec2i32(2, 2);
    assert(size.x() >= raw.x() && size.y() >= raw.y());
    pixels = rasterize(size, pixel_size, bbox.min, rotated_source);
    const std::vector<unsigned char> trimming = rasterize(size, pixel_size, bbox.min, rotated_mask);
    std::vector<unsigned char> dilated(trimming.size(), 0);
    for (int r = 1; r + 1 < size.y(); ++r)
        for (int col = 1; col + 1 < size.x(); ++col) {
            bool inside = true;
            for (int y = -1; y <= 1; ++y)
                for (int x = -1; x <= 1; ++x) inside = inside && trimming[col + x + (r + y) * size.x()] != 0;
            dilated[col + r * size.x()] = inside;
        }
    for (int r = 0; r < blocks.y(); ++r)
        for (int col = 0; col < blocks.x(); ++col) {
            const int origin = col * oversampling + 1 + (r * oversampling + 1) * size.x();
            // Two sweeps spread support in each cell, stopping at the eroded object mask.
            const auto propagate = [&](int address, int offset) {
                address += origin;
                if (pixels[address + offset] && !dilated[address] && !dilated[address + offset]) pixels[address] = 1;
            };
            for (int y = 0; y < oversampling; ++y) {
                if (y > 0) for (int x = 0; x < oversampling; ++x) propagate(y * size.x() + x, -size.x());
                for (int x = 1; x < oversampling; ++x) propagate(y * size.x() + x, -1);
                for (int x = oversampling - 2; x >= 0; --x) propagate(y * size.x() + x, 1);
            }
            for (int y = oversampling - 2; y >= 0; --y) {
                for (int x = 0; x < oversampling; ++x) propagate(y * size.x() + x, size.x());
                for (int x = 1; x < oversampling; ++x) propagate(y * size.x() + x, -1);
                for (int x = oversampling - 2; x >= 0; --x) propagate(y * size.x() + x, 1);
            }
        }
}

// Convert the zero-bordered raster to closed contours; offset must be less than half a pixel and adjusts only corners so repeated projection does not grow columns.
Polygons contours(const Grid &grid, coord_t offset, bool fill_holes)
{
    assert(std::abs(2 * offset) < grid.pixel_size - 10);
    std::vector<unsigned char> pixels = grid.pixels;
    const int stride = grid.size.x();
    if (fill_holes)
        for (int r = 1; r + 1 < grid.size.y(); ++r)
            for (int c = 1; c + 1 < grid.size.x(); ++c) {
                const int addr = r * stride + c;
                if ((grid.pixels[addr - 1] && grid.pixels[addr + 1]) || (grid.pixels[addr - stride] && grid.pixels[addr + stride])) pixels[addr] = 1;
            }
    std::vector<Line> lines;
    std::vector<std::pair<Point, int>> starts;
    for (int r = 1; r < grid.size.y(); ++r)
        for (int c = 1; c < grid.size.x(); ++c) {
            const int addr = r * stride + c;
            const bool left = pixels[addr - 1] != 0, top = pixels[addr - stride] != 0, current = pixels[addr] != 0;
            if (left != current) {
                lines.emplace_back(left ? Line(Point(c, r + 1), Point(c, r)) : Line(Point(c, r), Point(c, r + 1)));
                starts.emplace_back(lines.back().a, int(lines.size()) - 1);
            }
            if (top != current) {
                lines.emplace_back(top ? Line(Point(c, r), Point(c + 1, r)) : Line(Point(c + 1, r), Point(c, r)));
                starts.emplace_back(lines.back().a, int(lines.size()) - 1);
            }
        }
    const auto point_order =
        [](const std::pair<Point, int> &a, const std::pair<Point, int> &b) { return a.first < b.first; };
    std::sort(starts.begin(), starts.end(), point_order);
    std::vector<char> processed(lines.size(), false);
    Polygons out;
    for (int candidate = 0; candidate < int(lines.size()); ++candidate) {
        if (processed[candidate]) continue;
        Polygon polygon;
        processed[candidate] = true;
        polygon.points.push_back(lines[candidate].b);
        int current = candidate;
        bool closed = false;
        while (!closed) {
            const std::pair<std::vector<std::pair<Point, int>>::iterator, std::vector<std::pair<Point, int>>::iterator> range =
                std::equal_range(starts.begin(), starts.end(), std::make_pair(lines[current].b, 0), point_order);
            assert(range.first != range.second);
            int next = -1;
            for (std::vector<std::pair<Point, int>>::iterator it = range.first; it != range.second; ++it) {
                if (it->second == candidate) { closed = true; break; }
                if (processed[it->second]) continue;
                if (next == -1) next = it->second;
                else {
                    const Vector a = lines[current].vector(), b = lines[it->second].vector();
                    if (int64_t(a(0)) * b(1) - int64_t(b(0)) * a(1) > 0) { next = it->second; break; }
                }
            }
            if (closed) break;
            assert(next >= 0);
            processed[next] = true;
            current = next;
            polygon.points.push_back(lines[current].b);
        }
        // The polygon is stored while its points are still grid cells: moving it here
        // drops a last point that nearly coincides with the first one, as the native
        // generator does with its cell coordinates.
        out.push_back(std::move(polygon));
    }
    for (Polygon &polygon : out) {
        for (Point &point : polygon.points) { point *= grid.pixel_size; point += grid.bbox.min; }
        Points corners;
        for (size_t i = 0; i < polygon.points.size(); ++i) {
            const Point v = polygon.points[(i + 1) % polygon.points.size()] - polygon.points[i == 0 ? polygon.points.size() - 1 : i - 1];
            if (v(0) != 0 && v(1) != 0) {
                Point point = polygon.points[i];
                point(1) += v(0) < 0 ? -offset : offset;
                point(0) += v(1) > 0 ? -offset : offset;
                corners.push_back(point);
            }
        }
        polygon.points = std::move(corners);
    }
    return out;
}

// Extract grid contours, trim by the object and retain only islands containing source samples; snug contours instead undergo closing and outward smoothing.
Polygons Grid::extract(coord_t offset, bool fill_holes) const
{
    if (source.empty()) return {};
    if (style == smsSnug) return smooth_outward(closing(source, scale_d(closing_radius), join_type, 0.), scale_d(width));
    ExPolygons islands = diff_ex(contours(*this, offset, fill_holes), rotated_mask);
    const ExPolygons sample_source = offset > 0 ? union_ex(rotated_source) : intersection_ex(rotated_source, islands);
    Points samples;
    for (const ExPolygon &polygon : sample_source) {
        const Polygons shrunk = Slic3r::offset(polygon, -20.f);
        for (const Polygon &contour : shrunk)
            if (!contour.points.empty()) {
                const int count = int(contour.points.size()), stride = count / std::min(count, 4);
                for (int i = 0; i < count; i += stride) samples.push_back(contour.points[i]);
                break;
            }
    }
    std::sort(samples.begin(), samples.end());
    Polygons result;
    for (ExPolygon &island : islands) {
        const BoundingBox bounds = get_extents(island.contour);
        const Points::const_iterator begin = std::lower_bound(samples.begin(), samples.end(), Point(bounds.min - Point(1, 1)));
        const Points::const_iterator end = std::upper_bound(samples.begin(), samples.end(), Point(bounds.max + Point(1, 1)));
        for (Points::const_iterator sample = begin; sample != end; ++sample)
            if (bounds.contains(*sample) && island.contains(*sample)) { polygons_append(result, std::move(island)); break; }
    }
    if (angle != 0) polygons_rotate(result, angle);
    return result;
}

// Check perimeter loops and the INFILLS bucket independently; bridge fills never require a perimeter bucket to exist.
bool has_bridges(const Layer &layer)
{
    for (const LayerSliceIsland &island : layer.islands())
        for (const LayerRegionIsland &region : island.regions_islands()) {
            if (region.has_extrusion(LayerRegionIsland::PERIMETERS)) {
                ExPolygons areas;
                bridge_areas(region.extrusion(LayerRegionIsland::PERIMETERS), 0, areas);
                if (!areas.empty()) return true;
            }
            if (region.has_extrusion(LayerRegionIsland::INFILLS) && bridging_fills(region.extrusion(LayerRegionIsland::INFILLS))) return true;
        }
    return false;
}

// Read generic tree children and inherited flow attributes, returning true for a bridge polyline without assuming a concrete path subclass.
bool bridging_fills(const ExtrusionEntity &entity, const ExtrusionAttributes *attributes)
{
    if (const ExtrusionAttributes *local = entity.get_property<ExtrusionAttributes>()) attributes = local;
    if (entity.has_polyline() && attributes && attributes->extrusion_role().is_bridge()) return true;
    for (const ExtrusionEntityUPtr &child : entity.children())
        if (bridging_fills(*child, attributes)) return true;
    return false;
}

// Expand overhang polylines using their nearest flow attributes and append scaled XY footprints; generic nodes and descendants remain owned by the object.
void bridge_areas(const ExtrusionEntity &entity, coord_t gap, ExPolygons &out, const ExtrusionAttributes *attributes)
{
    if (const ExtrusionAttributes *local = entity.get_property<ExtrusionAttributes>()) attributes = local;
    if (entity.has_polyline() && attributes && attributes->extrusion_role().has(ExtrusionRole::OverhangPerimeter)) {
        const ArcPolyline &path = entity.polyline_ref();
        const double expansion = scale_d(attributes->width / 2) + gap;
        if (path.size() >= 3 && path.front() == path.back()) {
            Polygon polygon(path.to_polyline().points);
            if (polygon.area() < 0) polygon.reverse();
            append(out, diff_ex(offset(polygon, expansion, join_type, 0.), offset(polygon, -expansion, join_type, 0.)));
        } else if (path.size() >= 2) {
            append(out, offset_ex(path.to_polyline(), scale_d(attributes->width / 2) + expansion, join_type, 0.));
        }
    }
    for (const ExtrusionEntityUPtr &child : entity.children()) bridge_areas(*child, gap, out, attributes);
}

// Append a slab to the owning deque and return its stable reference; Z dimensions must lie on the layer grid or assert, and only intermediate/base slabs start with a full-height collision span.
GeneratorLayer &add_layer(GeneratedSupport &out, LayerType type, coord_t z, coord_t bottom, coord_t height)
{
    assert(scale_to_layer_coord(unscaled(z)) == z && scale_to_layer_coord(unscaled(height)) == height);
    out.storage.emplace_back();
    GeneratorLayer &layer = out.storage.back();
    layer.type = type;
    layer.print_z = z;
    layer.bottom_z = bottom;
    layer.height = height;
    // A top contact is already cleared against the lower layer; its zero collision span excludes the coplanar object layer from trimming.
    layer.height_block = type == LayerType::Intermediate || type == LayerType::Base ? height : 0;
    return layer;
}

// Sort contacts and union slabs closer than the minimum thickness into the lower slab without changing its collision span; moved source geometry is cleared while deque storage stays alive.
void merge_contacts(const GridSupportParams &p, GeneratorLayers &layers)
{
    std::sort(layers.begin(), layers.end(), [](const GeneratorLayer *a, const GeneratorLayer *b) { return a->print_z < b->print_z; });
    GeneratorLayers merged;
    for (size_t i = 0; i < layers.size();) {
        GeneratorLayer &target = *layers[i];
        const coord_t first = scale_to_layer_coord(p.first_layer_height);
        const bool first_group = target.print_z < first + p.min_height;
        const coord_t limit = first_group ? first + p.min_height - 1 : target.print_z + p.min_height;
        size_t j = i + 1;
        for (; j < layers.size() && layers[j]->print_z <= limit; ++j) {
            GeneratorLayer &source = *layers[j];
            target.polygons = union_(target.polygons, source.polygons);
            *target.contact_polygons = union_(*target.contact_polygons, *source.contact_polygons);
            *target.overhang_polygons = union_(*target.overhang_polygons, *source.overhang_polygons);
            source = GeneratorLayer();
        }
        if (first_group) { target.print_z = first; target.bottom_z = 0; target.height = first; }
        ensure_valid(target.polygons, p.resolution);
        merged.push_back(&target);
        i = j;
    }
    layers = std::move(merged);
}

// Compute ceiling division with the native rounding tolerance; distance and step must both be positive and the result is at least one.
size_t ceil_steps(coord_t distance, coord_t step)
{
    assert(distance > 0 && step > 0);
    const size_t count = size_t(ceil((double(distance) - EPSILON) / double(step)));
    assert(count > 0);
    return count;
}
} // namespace
} // namespace Slic3r::SupportGrid
