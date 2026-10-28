///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#include "SupportZPlacement.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/PrintHelpers.hpp"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"
#include "libslic3r/Plugins/Support/Common/ClipperResults.hpp"

/* Implementation of the shared Z placement (see SupportZPlacement.hpp).

   place_intermediate_slabs walks the extremes upward and remembers the previous one, so each interval starts where
   the lower extreme's support ends. The grid layout runs the native arithmetic unchanged: it computes the dense and
   middle thicknesses together and spaces the whole interval in one pass. The handed-over layout
   (place_handed_over_interval) places the dense stacks at the interface step, gives the middle to the splitter, and
   splits the whole interval evenly when the stacks leave no usable middle. The synchronized layout is grid-only:
   contacts that share a print Z take the bottom contact's span first, and each interval is then cut on the Z of the
   object layers. */

namespace slic3r_api::Support::Common {
namespace {
constexpr clipper_join_type_t join_type = CLIPPER_JOIN_SQUARE;

// Place one interval of the handed-over layout between start and end. The previous top contact keeps its span as one slab first;
// the current top contact, when it has no thickness yet, becomes the top dense slab. Returns the Z where the current extreme ends.
coord_t place_handed_over_interval(const SlabSpacing &spacing, const MiddleSplitter &split_middle, GeneratedSupport &out,
                                   const GeneratorLayer *previous, GeneratorLayer *current, coord_t start, coord_t end);
// Check perimeter loops and the INFILLS bucket independently; bridge fills never require a perimeter bucket to exist.
bool has_bridges(storage_handle *storage, const Layer &layer);
// Read generic tree children and inherited flow attributes, returning true for a bridge polyline without assuming a concrete path subclass.
bool bridging_fills(const ExtrusionEntity &entity, const EPropertyAttributes *attributes = nullptr);
// Return the number of equal steps needed to cover a positive distance.
size_t ceil_steps(coord_t distance, coord_t step);
// Round a length up to a whole number of grid steps.
coord_t round_up_to_grid(coord_t value, coord_t grid);
} // namespace

// Resolve the borrowed setting in mm (percent of nozzle_mm), clamp nonzero values to [0, max_height_mm], then scale to the layer grid.
// Zero selects the maximum; finite inputs and a nonnegative maximum are expected. No state changes, ownership transfers or allocations occur.
coord_t slab_step(const c_float_or_percent &setting, double nozzle_mm, double max_height_mm)
{
    return scale_to_layer_coord(setting.value == 0 ? max_height_mm :
        std::min(max_height_mm, std::max(0., c_float_or_percent_get_effective_value(&setting, nozzle_mm))));
}

// Create the contact of object layer id under that layer, in the Z of the native rules: a soluble contact fills the object layer
// just below, a non-soluble one lies the gap above the object, is dropped below the first layer and snaps onto it within the
// minimum height. With a filament distance, a layer with bridges also gets a bridge contact at the bridge height.
TopContactSlabs add_top_contacts(const std::vector<Layer> &layers, const Config &print_config, const SupportParams &p, size_t id,
                                 GeneratedSupport &out)
{
    const coord_t first_z = scale_to_layer_coord(p.first_layer_height);
    const Layer &layer = layers[id];
    coord_t z = layer.bottom_z() - scale_to_layer_coord(p.gap_above);
    coord_t bottom = z, height = 0;
    if (p.soluble) {
        height = layers[id - 1].height();
        bottom = id == 1 ? 0 : layers[id - 2].print_z();
    } else {
        if (z < first_z) return {};
        if (z < first_z + p.min_height) { z = first_z; bottom = 0; height = first_z; }
    }
    TopContactSlabs slabs;
    slabs.contact = &add_layer(out, LayerType::TopContact, z, bottom, height);
    slabs.contact->object_above = id;
    out.top.push_back(slabs.contact);
    if (!p.soluble && has_bridges(out.storage, layer)) {
        coord_t bridge_height = layer.height();
        if (p.distance_type == ContactDistanceType::Filament) {
            double total = 0;
            for (uint32_t region = 0; region < layer.region_count(); ++region)
                total += bridging_height_avg_mm(print_config, layer, layer.region(region));
            bridge_height = scale_to_layer_coord(total / layer.region_count());
        }
        coord_t bridge_z = layer.print_z() - bridge_height - scale_to_layer_coord(p.gap_above);
        if (bridge_z >= first_z && bridge_z < z) {
            if (bridge_z < first_z + p.min_height) bridge_z = first_z;
            slabs.bridge = &add_layer(out, LayerType::TopContact, bridge_z, bridge_z == first_z ? 0 : bridge_z,
                                      bridge_z == first_z ? first_z : 0);
            slabs.bridge->object_above = id;
            out.top.push_back(slabs.bridge);
        }
    }
    return slabs;
}

// Sort the contacts by print Z, then union each contact within the minimum height above a lower one into that lower contact; the
// lower contact keeps its own collision span, and a contact on the first layer keeps the first-layer thickness.
void merge_contacts(const SupportParams &p, GeneratedSupport &out)
{
    const ClipperContext clipper(out.storage);
    GeneratorLayers &layers = out.top;
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
            target.polygons = clipper_union2(clipper(target.polygons), clipper(source.polygons)).to_polygon_collection();
            *target.contact_polygons = clipper_union2(clipper(*target.contact_polygons), clipper(*source.contact_polygons)).to_polygon_collection();
            *target.overhang_polygons = clipper_union2(clipper(*target.overhang_polygons), clipper(*source.overhang_polygons)).to_polygon_collection();
            source.reset();
        }
        if (first_group) { target.print_z = first; target.bottom_z = 0; target.height = first; }
        target.polygons.ensure_valid(p.resolution);
        merged.push_back(&target);
        i = j;
    }
    layers = std::move(merged);
}

// Place the bottom contact on object layer id. Its Z starts above the layer by the gap below, and its thickness is, by case: the
// height of the upper layer for the Plane distance type; for a soluble support, the contact is filled up to the top of the upper
// layer; otherwise the height of the bottom contact flow (bottom_flow_mm). Non-soluble contacts whose Z lands within the minimum
// height of a top contact take that contact's Z, as the native generator does, so the list is searched from the first contact that
// can match. The returned reference belongs to out, which owns the layer.
GeneratorLayer &add_bottom_contact(const std::vector<Layer> &layers, const SupportParams &p, size_t id,
                                   const StoredPolygonCollection &touching, const StoredPolygonCollection &top_surfaces,
                                   GeneratedSupport &out)
{
    const ClipperContext clipper(out.storage);
    const Layer &layer = layers[id];
    const coord_t height_block = p.distance_type == ContactDistanceType::Plane ? layers[id + 1].height() :
        (p.soluble ? layers[id + 1].height() : scale_to_layer_coord(p.bottom_flow_mm.height));
    const coord_t bottom = layer.print_z() + scale_to_layer_coord(p.gap_below);
    const coord_t z = p.soluble ? layers[id + 1].print_z() : bottom + height_block;
    GeneratorLayer &contact = add_layer(out, LayerType::BottomContact, z, bottom, z - bottom);
    contact.height_block = height_block;
    contact.object_below = id;
    contact.bridging = !p.soluble;
    contact.polygons = touching.readonly().clone(out.storage);
    if (p.bottom_expansion.value > 0)
        contact.polygons = clipper_intersection(clipper(top_surfaces), clipper_offset(clipper(touching),
            scale_d(c_float_or_percent_get_effective_value(&p.bottom_expansion, p.bottom_flow_mm.width)), join_type, 0.)).to_polygon_collection();
    if (!p.soluble) {
        const auto first_at_or_above = std::lower_bound(out.top.begin(), out.top.end(), layer.print_z(),
            [](const GeneratorLayer *top, coord_t print_z) { return top->print_z < print_z; });
        const size_t first = size_t(first_at_or_above - out.top.begin());
        for (size_t idx = first > 0 ? first - 1 : 0; idx < out.top.size() && out.top[idx]->print_z <= z + p.min_height; ++idx) {
            const coord_t thickness = out.top[idx]->print_z - contact.bottom_z;
            if (out.top[idx]->print_z >= z - p.min_height && thickness >= p.min_height &&
                thickness <= scale_to_layer_coord(p.max_support_height)) {
                contact.print_z = out.top[idx]->print_z;
                contact.height = contact.print_z - contact.bottom_z;
                assert(contact.height >= p.min_height);
                break;
            }
        }
    }
    contact.polygons.ensure_valid(p.resolution);
    out.bottom.push_back(&contact);
    return contact;
}

// Copies effective nonnegative contact-inclusive counts into borrowed spacing; the disabled bottom option follows the top, and this writes only the two counters of SlabSpacing.
void set_interface_counts(SlabSpacing &spacing, const SupportParams &params)
{
    spacing.interface_layers = std::max(0, params.top_interface_layers);
    spacing.bottom_interface_layers = params.bottom_interface_count_enabled ?
        std::max(0, params.bottom_interface_layers) : spacing.interface_layers;
}

// Divide the owned contact intervals into printable slabs, bottom first. Synchronized slabs follow the object layers; a splitter
// selects the handed-over layout; otherwise the native grid arithmetic spaces each interval in one pass.
// Borrow spacing, layers and splitter; append slabs owned by out and set contact thicknesses. Invalid spacing asserts; allocation failures propagate.
void place_intermediate_slabs(const SlabSpacing &spacing, const std::vector<Layer> &layers, GeneratedSupport &out, const MiddleSplitter &split_middle)
{
    const bool sync = spacing.synchronize;
    assert(!(sync && split_middle));
    const coord_t first = spacing.first_z, min_height = spacing.min_height, base_step = spacing.base_step;
    const coord_t interface_step = spacing.interface_step, machine_step = spacing.machine_step;
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
    extremes.insert(extremes.end(), out.bottom.begin(), out.bottom.end());
    std::sort(extremes.begin(), extremes.end(), [](const GeneratorLayer *a, const GeneratorLayer *b) {
        return a->extreme_z() < b->extreme_z() || (a->extreme_z() == b->extreme_z() &&
            a->type == LayerType::TopContact && b->type == LayerType::BottomContact);
    });
    coord_t previous_extreme = 0;
    GeneratorLayer *previous = nullptr;
    for (GeneratorLayer *current : extremes) {
        coord_t end = current->extreme_z();
        if (end < first) {
            assert(current->type == LayerType::TopContact);
            current->height = current->print_z;
            // The first bed contact already has its thickness and is excluded from collision-span planning.
            if (current != extremes.front() || current->bottom_z != 0) current->height_block = current->print_z;
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
        assert(split_middle || distance >= min_height);
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
            for (const Layer &layer : layers) {
                const coord_t z = layer.print_z();
                if (z <= start) continue;
                if (z > end || (unknown_top && z == end)) break;
                if (z - last < min_height) continue;
                out.base.push_back(&add_layer(out, LayerType::Intermediate, z, last, z - last));
                last = z;
            }
            if (unknown_top) {
                if (end - last < min_height && !out.base.empty() && out.base.back()->print_z == last) {
                    last = out.base.back()->bottom_z;
                    out.base.pop_back();
                }
                // Round cumulative boundaries, not one repeated thickness, so residual machine steps reach the unchanged contact Z.
                const coord_t limit = machine_step > SCALED_EPSILON ? interface_step - interface_step % machine_step : interface_step;
                const size_t count = ceil_steps(end - last, limit);
                const coord_t start_z = last;
                for (size_t i = 1; i < count; ++i) {
                    const coord_t next = check_z_step(scale_to_layer_coord(unscaled(start_z) + unscaled(end - start_z) * i / count), machine_step);
                    assert(next - last >= min_height && next - last <= interface_step);
                    out.base.push_back(&add_layer(out, LayerType::Intermediate, next, last, next - last));
                    last = next;
                }
                current->bottom_z = last;
                current->height = end - last;
                assert(current->height >= min_height && current->height <= interface_step);
                end = last;
            } else if (last < end) {
                // A bottom contact keeps its gap above the object; close the interval at its top before returning to object Z positions.
                if (end - last < min_height && !out.base.empty() && out.base.back()->print_z == last) {
                    last = out.base.back()->bottom_z;
                    out.base.pop_back();
                }
                const coord_t limit = machine_step > SCALED_EPSILON ? base_step - base_step % machine_step : base_step;
                const size_t count = ceil_steps(end - last, limit);
                const coord_t start_z = last;
                for (size_t i = 1; i <= count; ++i) {
                    const coord_t z = i == count ? end : check_z_step(scale_to_layer_coord(unscaled(start_z) + unscaled(end - start_z) * i / count), machine_step);
                    assert(z - last >= min_height && z - last <= base_step);
                    out.base.push_back(&add_layer(out, LayerType::Intermediate, z, last, z - last));
                    last = z;
                }
            }
        } else if (split_middle) {
            end = place_handed_over_interval(spacing, split_middle, out, previous, current, start, end);
        } else {
            // Distribute the interval across the requested top, bottom and sparse thicknesses, as in native support generation.
            size_t top = size_t(std::max(0, spacing.interface_layers));
            const int bottom_count = spacing.bottom_interface_layers;
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
            // A bed contact is already covered by the first-layer slab; preserve a contact span only above start.
            if (previous && previous->type == LayerType::TopContact && previous->print_z > start &&
                previous->print_z + min_height > previous->bottom_z + scale_to_layer_coord(unscaled(step))) {
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
            if (!spacing.soluble && current->type == LayerType::TopContact && current->height == 0) {
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
                        machine_step), dense ? interface_step : base_step);
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

namespace {
// The stacks are placed at the interface step rounded down to the Z grid. They are kept when the middle they leave is zero or at least
// min_height; otherwise the interval is split evenly into dense slabs, within the interface step when feasible (see the header), and an unknown top contact is the last of them.
coord_t place_handed_over_interval(const SlabSpacing &spacing, const MiddleSplitter &split_middle, GeneratedSupport &out,
                                   const GeneratorLayer *previous, GeneratorLayer *current, coord_t start, coord_t end)
{
    const coord_t min_height = spacing.min_height;
    // The Z grid of the boundaries: the machine step, or the layer grid when the machine step is disabled.
    const coord_t zgrid = std::max(spacing.machine_step, SCALED_LAYER_Z_STEP);
    const coord_t dense = handed_over_dense_height(spacing.interface_step, min_height, spacing.machine_step);
    size_t bottom = spacing.bottom_interface_layers > 0 ? size_t(spacing.bottom_interface_layers - 1) : 0;
    // The previous top contact keeps its span as one slab and the interval starts above it. A contact on the first layer is
    // already covered by the first-layer slab, so it adds nothing here.
    if (previous && previous->type == LayerType::TopContact && previous->print_z > start) {
        const coord_t from = std::max(previous->bottom_z, start);
        out.base.push_back(&add_layer(out, LayerType::Intermediate, previous->print_z, from, previous->print_z - from));
        start = previous->print_z;
        if (bottom > 0) --bottom;
    }
    const coord_t distance = end - start;
    assert(distance >= 0);
    if (distance == 0) return end;
    // An unknown top contact is the first dense slab under the object. It takes one of the interface layers, or it is the only slab
    // of an interval when there are none.
    const bool unknown_top = !spacing.soluble && current->type == LayerType::TopContact && current->height == 0;
    const size_t contact = unknown_top ? 1 : 0;
    const size_t interfaces = size_t(std::max(0, spacing.interface_layers));
    const size_t top = interfaces > 0 ? interfaces - contact : 0;
    const coord_t middle = distance - dense * coord_t(top + contact + bottom);
    if (middle == 0 || middle >= min_height) {
        if (unknown_top) {
            current->height = dense;
            end = current->bottom_z = current->print_z - dense;
        }
        coord_t last = start;
        for (size_t i = 0; i < bottom; ++i) {
            out.base.push_back(&add_layer(out, LayerType::Intermediate, last + dense, last, dense));
            last += dense;
        }
        const coord_t middle_top = end - dense * coord_t(top);
        if (middle_top > last) {
            assert(middle_top - last >= min_height);
            const size_t first_new = out.base.size();
            split_middle(last, middle_top);
            assert(out.base.size() > first_new && out.base[first_new]->bottom_z == last && out.base.back()->print_z == middle_top);
        }
        last = middle_top;
        for (size_t i = 0; i < top; ++i) {
            out.base.push_back(&add_layer(out, LayerType::Intermediate, last + dense, last, dense));
            last += dense;
        }
        assert(last == end);
        return end;
    }
    // Even split in whole grid steps: every slab takes the floor or the ceiling of the average, so it stays within the dense step.
    // The count is the fewest that keeps the dense step, and it is capped so that the floor still reaches the minimum height;
    // when no count does both, the slabs are at least the minimum height and may exceed the dense step, and an interval shorter
    // than the minimum keeps one slab thinner than the minimum.
    const coord_t units = distance / zgrid;
    const coord_t dense_units = dense / zgrid;
    const coord_t min_units = std::max<coord_t>(1, round_up_to_grid(min_height, zgrid) / zgrid);
    const coord_t count = std::max<coord_t>(1, std::min((units + dense_units - 1) / dense_units, units / min_units));
    coord_t last = start;
    for (coord_t i = 1; i < count; ++i) {
        const coord_t z = start + zgrid * (units * i / count);
        out.base.push_back(&add_layer(out, LayerType::Intermediate, z, last, z - last));
        last = z;
    }
    if (unknown_top) {
        current->bottom_z = last;
        current->height = end - last;
        return last;
    }
    out.base.push_back(&add_layer(out, LayerType::Intermediate, end, last, end - last));
    return end;
}
} // namespace

// The dense step is the interface step on the machine Z grid (the layer grid when the machine step is off), raised to the minimum height.
coord_t handed_over_dense_height(coord_t interface_step, coord_t min_height, coord_t machine_step)
{
    const coord_t zgrid = std::max(machine_step, SCALED_LAYER_Z_STEP);
    return std::max({interface_step - interface_step % zgrid, round_up_to_grid(min_height, zgrid), zgrid});
}

// Allocate the slab in out.slabs and set its type and Z; only intermediate and base slabs start with a full-height collision span.
GeneratorLayer &add_layer(GeneratedSupport &out, LayerType type, coord_t z, coord_t bottom, coord_t height)
{
    assert(scale_to_layer_coord(unscaled(z)) == z && scale_to_layer_coord(unscaled(height)) == height);
    out.slabs.emplace_back(out.storage);
    GeneratorLayer &layer = out.slabs.back();
    layer.type = type;
    layer.print_z = z;
    layer.bottom_z = bottom;
    layer.height = height;
    // A top contact is already cleared against the lower layer; its zero collision span excludes the coplanar object layer from trimming.
    layer.height_block = type == LayerType::Intermediate || type == LayerType::Base ? height : 0;
    return layer;
}

// Expand overhang polylines using their nearest flow attributes and append scaled XY footprints; generic nodes and descendants remain owned by the object.
// An arc in a path (a plugin may store some) is replaced by points on it, like the native ArcPolyline::to_polyline().
void bridge_areas(storage_handle *storage, const ExtrusionEntity &entity, coord_t gap, StoredExPolygonCollection &out, const EPropertyAttributes *attributes)
{
    const ClipperContext clipper(storage);
    if (const EPropertyAttributes *local = entity.get(EPropertyAttributes::key)) attributes = local;
    const int32_t overhang = RAW_EXTRUSION_ROLE_OVERHANG_PERIMETER;
    if (entity.has_polyline() && attributes && (attributes->extrusion_role() & overhang) == overhang) {
        const uint32_t size = entity.point_count();
        const double expansion = scale_d(attributes->c_extrusion_property_attributes::width / 2) + gap;
        if (size >= 3 && entity.local_front() == entity.local_back()) {
            // The native Polygon built from the closed path drops its repeated last point.
            std::vector<c_point> points = entity.linear_points();
            points.pop_back();
            StoredPolygon polygon(storage);
            polygon.insert_array(0, points.data(), uint32_t(points.size()));
            if (polygon.area() < 0) polygon.reverse();
            out.append_move_from(native_expolygons(clipper_diff(clipper_offset(clipper(polygon), expansion, join_type, 0.),
                                                                clipper_offset(clipper(polygon), -expansion, join_type, 0.))));
        } else if (size >= 2) {
            StoredPolylineCollection path(storage);
            StoredPolyline polyline(storage);
            const std::vector<c_point> points = entity.linear_points();
            polyline.insert_array(0, points.data(), uint32_t(points.size()));
            path.push_back(polyline);
            out.append_move_from(clipper_union(clipper_offset_polylines(storage, path, scale_d(attributes->c_extrusion_property_attributes::width / 2) + expansion,
                                                                        join_type, 0.)).to_expolygon_collection());
        }
    }
    for (const ExtrusionEntity child : entity.children()) bridge_areas(storage, child, gap, out, attributes);
}

// The three native bridge types: the averaged region nozzles, the layer height, or a round thread with the cross
// section of the region's infill flow; the last two are float flow heights in the native code.
double bridging_height_avg_mm(const Config &print_config, const Layer &layer, const LayerRegion &region)
{
    const Config config = region.print_region().config();
    const std::string type = config.get("bridge_type").serialize();
    const double ratio = config.get("bridge_flow_ratio").get_effective_value(1.);
    if (type == "nozzle") {
        const ConfigOption nozzle = print_config.get("nozzle_diameter");
        const auto nozzle_of = [&](const char *key) { return nozzle.get_float(uint32_t(config.get(key).get_int() - 1)); };
        const double average = (nozzle_of("perimeter_extruder") + nozzle_of("infill_extruder") + nozzle_of("solid_infill_extruder")) / 3.;
        return average * std::sqrt(ratio);
    }
    if (type == "height")
        return unscaled(layer.height());
    if (type == "flow") {
        const double diameter = std::sqrt(4 * region.flow(raw_extrusion_role(RAW_EXTRUSION_ROLE_INFILL)).mm3_per_mm / PI);
        return float(std::sqrt(ratio) * diameter);
    }
    throw std::invalid_argument("Unknown bridge type " + type);
}

// Collect, for a slab whose top is print_z + gap_above (the limit), the footprints of the object bridges that hang in its way.
// A bridge thread is printed in [layer.print_z - bridge_height, layer.print_z], so it matters when it starts below the limit.
// The walk starts at the first object layer with bottom_z >= from_z: layers that start below from_z are the caller's own outline
// trimming. A caller that trims no layer beyond the slab's own Z span passes the slab's print_z; a caller whose outline trimming
// reaches up to the limit passes the limit. For each layer the walk does two independent passes:
//  - Region pass: regions are visited in order and the pass stops at the FIRST region whose thread starts at or above the
//    limit; later regions of that layer are not examined. Each region visited before that adds the bridge fill surfaces of
//    the work region islands it belongs to, grown by gap_xy, and sets overlaps. If the first region already stops the pass,
//    overlaps stays false.
//  - Perimeter pass: for each work region island with perimeters, unless one of its regions has a thread starting at or above
//    the limit, the overhang perimeter footprints are added through bridge_areas (grown by gap_xy).
// The layer walk ends after the first layer whose region pass left overlaps false. A soluble support gets nothing.
// The result is a new collection owned by the caller.
StoredExPolygonCollection bridge_footprint(storage_handle *storage, const std::vector<Layer> &layers, const Config &print_config,
                                           const SupportParams &p, coord_t from_z, const GeneratorLayer &slab)
{
    StoredExPolygonCollection footprint(storage);
    if (p.soluble) return footprint;
    const ClipperContext clipper(storage);
    const bool filament = p.distance_type == ContactDistanceType::Filament;
    const raw_surface_type bridge_fill = srf_type().bottom().solid().bridge().raw();
    const coord_t upper = slab.print_z + scale_to_layer_coord(p.gap_above);
    size_t id = 0;
    while (id < layers.size() && layers[id].bottom_z() < from_z) ++id;
    for (; id < layers.size(); ++id) {
        const Layer &layer = layers[id];
        bool overlaps = false;
        for (uint32_t region_idx = 0; region_idx < layer.region_count(); ++region_idx) {
            const LayerRegion region = layer.region(region_idx);
            const coord_t bridge_height = filament ?
                scale_to_layer_coord(bridging_height_avg_mm(print_config, layer, region)) : layer.height();
            if (layer.print_z() - bridge_height >= upper) break;
            overlaps = true;
            for (uint32_t island = 0; island < layer.island_count(); ++island) {
                const LayerIsland layer_island = layer.island(island);
                for (uint32_t work_idx = 0; work_idx < layer_island.region_island_count(); ++work_idx) {
                    const LayerRegionIsland work = layer_island.region_island(work_idx);
                    bool member = false;
                    for (uint32_t idx = 0; idx < work.region_count(); ++idx)
                        member = member || work.region(idx).handle() == region.handle();
                    if (!member) continue;
                    StoredExPolygonCollection bridges(storage);
                    for (const Surface &surface : work.fill_surfaces())
                        if (surface.type() == bridge_fill) bridges.push_back(surface.expolygon());
                    footprint.append_move_from(clipper_offset(clipper(bridges), double(p.gap_xy), join_type, 0.).to_expolygon_collection());
                }
            }
        }
        for (uint32_t island = 0; island < layer.island_count(); ++island) {
            const LayerIsland layer_island = layer.island(island);
            for (uint32_t work_idx = 0; work_idx < layer_island.region_island_count(); ++work_idx) {
                const LayerRegionIsland work = layer_island.region_island(work_idx);
                if (!work.has_extrusion(raw_extrusion_role(RAW_EXTRUSION_ROLE_PERIMETER))) continue;
                bool too_high = false;
                for (uint32_t idx = 0; idx < work.region_count(); ++idx) {
                    const coord_t bridge_height = filament ?
                        scale_to_layer_coord(bridging_height_avg_mm(print_config, layer, work.region(idx))) : layer.height();
                    if (layer.print_z() - bridge_height >= upper) { too_high = true; break; }
                }
                if (!too_high)
                    bridge_areas(storage, ExtrusionEntity(work.extrusion(raw_extrusion_role(RAW_EXTRUSION_ROLE_PERIMETER))), p.gap_xy, footprint);
            }
        }
        if (!overlaps) break;
    }
    return footprint;
}

namespace {
// Detect bridge extrusion paths so thicker object bridges get a separate contact Z.
bool has_bridges(storage_handle *storage, const Layer &layer)
{
    for (uint32_t island = 0; island < layer.island_count(); ++island) {
        const LayerIsland layer_island = layer.island(island);
        for (uint32_t work_idx = 0; work_idx < layer_island.region_island_count(); ++work_idx) {
            const LayerRegionIsland region = layer_island.region_island(work_idx);
            if (region.has_extrusion(raw_extrusion_role(RAW_EXTRUSION_ROLE_PERIMETER))) {
                StoredExPolygonCollection areas(storage);
                bridge_areas(storage, ExtrusionEntity(region.extrusion(raw_extrusion_role(RAW_EXTRUSION_ROLE_PERIMETER))), 0, areas);
                if (!areas.empty()) return true;
            }
            if (region.has_extrusion(raw_extrusion_role(RAW_EXTRUSION_ROLE_INFILL)) &&
                bridging_fills(ExtrusionEntity(region.extrusion(raw_extrusion_role(RAW_EXTRUSION_ROLE_INFILL)))))
                return true;
        }
    }
    return false;
}

bool bridging_fills(const ExtrusionEntity &entity, const EPropertyAttributes *attributes)
{
    if (const EPropertyAttributes *local = entity.get(EPropertyAttributes::key)) attributes = local;
    if (entity.has_polyline() && attributes && (attributes->extrusion_role() & RAW_EXTRUSION_ROLE_BRIDGE) != 0) return true;
    for (const ExtrusionEntity child : entity.children())
        if (bridging_fills(child, attributes)) return true;
    return false;
}

size_t ceil_steps(coord_t distance, coord_t step)
{
    assert(distance > 0 && step > 0);
    const size_t count = size_t(std::ceil((double(distance) - EPSILON) / double(step)));
    assert(count > 0);
    return count;
}

coord_t round_up_to_grid(coord_t value, coord_t grid)
{
    return (value + grid - 1) / grid * grid;
}
} // namespace
} // namespace slic3r_api::Support::Common
