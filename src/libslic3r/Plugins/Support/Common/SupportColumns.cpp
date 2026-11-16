///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#include "SupportColumns.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_geometry.h"
#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Api/plugin/cpp/VolumeViews.hpp"
#include "libslic3r/Plugins/Support/Common/SupportInterfaces.hpp"
#include "libslic3r/Plugins/Support/Common/SupportZPlacement.hpp"

/* Implementation of the support columns (see SupportColumns.hpp).

   The columns are added after the family, on its finished slabs, in four phases:
   1. Heights: the Z span of every column comes from its transformed mesh, rounded to the machine Z step; the spans are united.
      A span is never lowered to the bed: the column starts where its mesh starts.
   2. Slabs: the sparse slabs of out.base that meet a span carry the column there. The parts of the spans that no sparse slab
      covers are cut at the object layer tops (above the object, every top layer height) into new base slabs; a piece thinner
      than the minimum support height joins its neighbor.
   3. Areas: every column is sliced once at the middle Z of every carrying slab. A section loses the object clearance (the object
      layers within the slab's Z gaps, with their no-support zones, grown by the XY gap, as the grid family trims its slabs) and
      the areas of the contacts and interfaces that overlap the slab in Z. A sparse section then joins the slab's polygons by a
      union, after the first-layer widening of the family's foot (widen_first_layer_area()) when the slab lies on the bed; an interface section becomes a new top interface slab with the slab's print Z and height, so that publication gives
      the dense role the area both kinds cover.
   4. Tops: under an object surface, a column ends where the family's own top contact would lie (the contact distance below the
      surface, lower under hanging bridge threads), not where the object layer grid happens to cut it: its area is removed from the
      column slabs rising above that Z, and caps on the grid fill it from the bottom of those slabs up to that Z.
   Sparse slabs do not overlap each other in Z, so a column area is printed by one slab only. The geometry lives in out's storage.

       add_support_columns()
       |-- column_spans(): Z spans of the columns
       |-- uncovered() / gap_pieces(): new base slabs where no sparse slab covers a span
       |-- slice_support_volumes(): sparse and interface column sections at the middle of every carrying slab
       |-- column_clearance(): object, zones, contacts and interfaces removed before the union
       `-- cap_column_tops(): column tops at the top contact Z under the object surfaces */

namespace slic3r_api::Support::Common {
namespace {
// A Z interval [first, second] in scaled print Z.
using Interval = std::pair<coord_t, coord_t>;

// A height in mm on the machine Z step, or only on the layer coordinates when there is no step.
coord_t on_z_grid(double z_mm, double z_step);
// The intervals united into sorted disjoint intervals.
std::vector<Interval> united(std::vector<Interval> intervals);
// The united Z spans of the support column volumes of the object, bounded below by the bed.
std::vector<Interval> column_spans(const Object &object, double z_step);
// The parts of the spans that no interval of covered (sorted and disjoint) overlaps.
std::vector<Interval> uncovered(const std::vector<Interval> &spans, const std::vector<Interval> &covered);
// The gap cut at the tops of the object layers inside it, and every step above the last object layer; a piece thinner than
// min_height joins the piece below it, or the one above for the first piece. A gap thinner than min_height stays one thin piece.
std::vector<Interval> gap_pieces(const Interval &gap, const std::vector<Layer> &layers, coord_t step, coord_t min_height);
// What a column section loses on the slab: the object layers within the slab's Z gaps with their zones, grown by the XY gap,
// and the contacts and interfaces of out that overlap the slab in Z. Empty when nothing is near.
std::optional<ClipperOperand> column_clearance(const ClipperContext &clipper, const std::vector<Layer> &layers,
                                               const SupportParams &params, const GeneratedSupport &out, const GeneratorLayer &slab);
// Ends the columns of each kind under an object surface at the Z of the family's top contact under that surface
// (top_contact_z(); the bridge contact's lower Z for the whole surface when its layer has bridges). Under object layer id, the area
// of the column section of layer id - 1 that layer id - 1 does not cover is removed from the column slabs of that kind (out.base,
// or dense_slabs for the interface columns) that rise above that Z below the surface; new slabs of that kind then fill this area
// from the bottom of the lowest of them up to that Z, cut like the gaps and trimmed like any column section.
void cap_column_tops(const plugin_run_context *context, const Print &print, const Object &object, const SupportParams &params,
                     const std::vector<Layer> &layers, GeneratedSupport &out, GeneratorLayers &dense_slabs);

coord_t on_z_grid(double z_mm, double z_step)
{
    return scale_to_layer_coord(z_step > 0. ? std::round(z_mm / z_step) * z_step : z_mm);
}

std::vector<Interval> united(std::vector<Interval> intervals)
{
    std::sort(intervals.begin(), intervals.end());
    std::vector<Interval> result;
    for (const Interval &interval : intervals) {
        if (!result.empty() && interval.first <= result.back().second)
            result.back().second = std::max(result.back().second, interval.second);
        else
            result.push_back(interval);
    }
    return result;
}

std::vector<Interval> column_spans(const Object &object, double z_step)
{
    std::vector<Interval> spans;
    const c_matrix4d centered = object.transform_centered();
    for (uint32_t volume_idx = 0; volume_idx < object.volume_count(); ++volume_idx) {
        const Volume volume = object.volume(volume_idx);
        if (!volume.is_support_column() && !volume.is_support_interface_column())
            continue;
        const TriangleMesh mesh = volume.mesh();
        if (mesh.empty())
            continue;
        // The slicing transform of the volume: only its Z row is needed for the height of a vertex.
        const c_matrix4d matrix = matrix4d_mul(centered, volume.matrix());
        double low = std::numeric_limits<double>::max(), high = std::numeric_limits<double>::lowest();
        for (uint32_t vertex_idx = 0; vertex_idx < mesh.vertex_count(); ++vertex_idx) {
            const c_vec3f vertex = mesh.vertex(vertex_idx);
            const double z = matrix.value[8] * vertex.x + matrix.value[9] * vertex.y + matrix.value[10] * vertex.z + matrix.value[11];
            low = std::min(low, z);
            high = std::max(high, z);
        }
        const coord_t bottom = on_z_grid(std::max(0., low), z_step);
        const coord_t top = on_z_grid(high, z_step);
        if (top > bottom)
            spans.emplace_back(bottom, top);
    }
    return united(std::move(spans));
}

std::vector<Interval> uncovered(const std::vector<Interval> &spans, const std::vector<Interval> &covered)
{
    std::vector<Interval> gaps;
    for (Interval span : spans) {
        for (const Interval &cover : covered) {
            if (cover.second <= span.first || cover.first >= span.second)
                continue;
            if (cover.first > span.first)
                gaps.emplace_back(span.first, cover.first);
            span.first = std::max(span.first, cover.second);
            if (span.first >= span.second)
                break;
        }
        if (span.first < span.second)
            gaps.push_back(span);
    }
    return gaps;
}

std::vector<Interval> gap_pieces(const Interval &gap, const std::vector<Layer> &layers, coord_t step, coord_t min_height)
{
    std::vector<coord_t> cuts{gap.first};
    for (const Layer &layer : layers)
        if (layer.print_z() > gap.first && layer.print_z() < gap.second)
            cuts.push_back(layer.print_z());
    const coord_t object_top = layers.empty() ? 0 : layers.back().print_z();
    if (step > 0)
        for (coord_t z = std::max(object_top, gap.first) + step; z < gap.second; z += step)
            cuts.push_back(z);
    cuts.push_back(gap.second);
    std::vector<Interval> pieces;
    for (size_t idx = 1; idx < cuts.size(); ++idx) {
        if (!pieces.empty() && cuts[idx] - cuts[idx - 1] < min_height)
            pieces.back().second = cuts[idx];
        else
            pieces.emplace_back(cuts[idx - 1], cuts[idx]);
    }
    if (pieces.size() > 1 && pieces.front().second - pieces.front().first < min_height) {
        pieces[1].first = pieces.front().first;
        pieces.erase(pieces.begin());
    }
    return pieces;
}

std::optional<ClipperOperand> column_clearance(const ClipperContext &clipper, const std::vector<Layer> &layers,
                                               const SupportParams &params, const GeneratedSupport &out, const GeneratorLayer &slab)
{
    std::optional<ClipperOperand> clearance;
    const auto add = [&clearance](ClipperOperand &&area) {
        if (clearance)
            *clearance += area;
        else
            clearance.emplace(std::move(area));
    };
    // The same Z window as the grid family's object trimming: the slab's collision span grown by both Z gaps.
    const coord_t lower = slab.print_z - std::max(slab.height, slab.height_block) - scale_to_layer_coord(params.gap_below);
    const coord_t upper = slab.print_z + scale_to_layer_coord(params.gap_above);
    for (size_t layer_idx = 0; layer_idx < layers.size(); ++layer_idx) {
        const Layer &layer = layers[layer_idx];
        if (layer.print_z() <= lower || layer.bottom_z() >= upper)
            continue;
        add(clipper_offset(support_obstacle(clipper, layer, layer_idx, out.no_support_zones), double(params.gap_xy),
                           CLIPPER_JOIN_SQUARE, 0.));
    }
    for (const GeneratorLayers *dense : {&out.top, &out.bottom, &out.interfaces})
        for (const GeneratorLayer *other : *dense)
            if (!other->polygons.empty() && other->bottom_print_z() < slab.print_z && other->print_z > slab.bottom_print_z())
                add(clipper(other->polygons));
    return clearance;
}

void cap_column_tops(const plugin_run_context *context, const Print &print, const Object &object, const SupportParams &params,
                     const std::vector<Layer> &layers, GeneratedSupport &out, GeneratorLayers &dense_slabs)
{
    if (layers.size() < 2)
        return;
    const Config print_config = print.config();
    const ClipperContext clipper(out.storage);
    const coord_t step = layers.back().height();
    for (const bool dense : {false, true}) {
        const std::vector<StoredExPolygonCollection> sections = slice_support_volumes(print, object,
            dense ? RAW_VOLUME_TYPE_SUPPORT_INTERFACE_COLUMN : RAW_VOLUME_TYPE_SUPPORT_COLUMN, out.storage, context);
        GeneratorLayers &slabs = dense ? dense_slabs : out.base;
        GeneratorLayers caps;
        for (size_t id = 1; id < layers.size() && id < sections.size(); ++id) {
            if (sections[id - 1].empty())
                continue;
            throw_if_cancelled(context);
            const StoredPolygonCollection under = clipper_intersection(clipper(sections[id - 1]),
                clipper_diff(clipper(layers[id].slices()), clipper(layers[id - 1].slices()))).to_polygon_collection();
            if (under.empty())
                continue;
            const std::optional<TopContactZ> contact = top_contact_z(layers, print_config, params, id, out.storage);
            if (!contact)
                continue;
            const coord_t ceiling = contact->bridge_z ? *contact->bridge_z : contact->z;
            const coord_t surface = layers[id].bottom_z();
            coord_t from = ceiling;
            for (GeneratorLayer *slab : slabs) {
                if (slab->print_z <= ceiling || slab->bottom_print_z() >= surface)
                    continue;
                // A slab that the clearance already emptied under the surface still bounds the part to fill below the ceiling.
                from = std::min(from, slab->bottom_print_z());
                if (slab->polygons.empty())
                    continue;
                slab->polygons = clipper_diff(clipper(slab->polygons), clipper(under)).to_polygon_collection();
                slab->polygons.ensure_valid(params.resolution);
            }
            if (ceiling - from < params.min_height)
                continue;
            for (const Interval &piece : gap_pieces({from, ceiling}, layers, step, params.min_height)) {
                if (piece.second - piece.first < params.min_height)
                    continue;
                GeneratorLayer &cap = add_layer(out, dense ? LayerType::TopInterface : LayerType::Base, piece.second, piece.first,
                                                piece.second - piece.first);
                ClipperOperand area = clipper(under);
                const std::optional<ClipperOperand> clearance = column_clearance(clipper, layers, params, out, cap);
                cap.polygons = (clearance ? clipper_diff(area, *clearance) : clipper_union(area)).to_polygon_collection();
                cap.polygons.ensure_valid(params.resolution);
                caps.push_back(&cap);
            }
        }
        slabs.insert(slabs.end(), caps.begin(), caps.end());
    }
}
} // namespace

bool has_support_columns(const Object &object)
{
    if (!object.config().get("support_material").get_bool())
        return false;
    for (uint32_t volume_idx = 0; volume_idx < object.volume_count(); ++volume_idx)
        if (object.volume(volume_idx).is_support_column() || object.volume(volume_idx).is_support_interface_column())
            return true;
    return false;
}

void add_support_columns(const plugin_run_context *context, const Print &print, const Object &object, const SupportParams &params,
                         GeneratedSupport &out)
{
    if (!has_support_columns(object))
        return;
    const std::vector<Interval> spans = column_spans(object, params.z_step);
    if (spans.empty())
        return;
    const std::vector<Layer> layers = object_layers(object);

    // Phase 2: the sparse slabs that meet a column, then new base slabs where none covers it.
    GeneratorLayers carriers;
    std::vector<Interval> covered;
    for (GeneratorLayer *slab : out.base) {
        if (slab->height <= 0)
            continue;
        covered.emplace_back(slab->bottom_print_z(), slab->print_z);
        for (const Interval &span : spans)
            if (slab->bottom_print_z() < span.second && slab->print_z > span.first) {
                carriers.push_back(slab);
                break;
            }
    }
    const coord_t step = layers.empty() ? scale_to_layer_coord(params.layer_height) : layers.back().height();
    for (const Interval &gap : uncovered(spans, united(std::move(covered))))
        for (const Interval &piece : gap_pieces(gap, layers, step, params.min_height)) {
            if (piece.second - piece.first < params.min_height)
                continue;
            GeneratorLayer &slab = add_layer(out, LayerType::Base, piece.second, piece.first, piece.second - piece.first);
            out.base.push_back(&slab);
            carriers.push_back(&slab);
        }
    std::stable_sort(out.base.begin(), out.base.end(),
        [](const GeneratorLayer *a, const GeneratorLayer *b) { return a->print_z < b->print_z; });
    if (carriers.empty())
        return;

    // Phase 3: one slicing of every column at the middles of the carriers, in increasing Z as the slicer requires.
    std::sort(carriers.begin(), carriers.end(), [](const GeneratorLayer *a, const GeneratorLayer *b) {
        return a->print_z - a->height / 2 < b->print_z - b->height / 2;
    });
    std::vector<float> slice_zs;
    slice_zs.reserve(carriers.size());
    for (const GeneratorLayer *slab : carriers)
        slice_zs.push_back(float(unscaled(slab->print_z - slab->height / 2)));
    const std::vector<StoredExPolygonCollection> sparse =
        slice_support_volumes(print, object, RAW_VOLUME_TYPE_SUPPORT_COLUMN, slice_zs, out.storage, context);
    const std::vector<StoredExPolygonCollection> dense =
        slice_support_volumes(print, object, RAW_VOLUME_TYPE_SUPPORT_INTERFACE_COLUMN, slice_zs, out.storage, context);
    const ClipperContext clipper(out.storage);
    // The interface slabs of the dense columns are listed after every clearance is taken, so a column never trims another.
    GeneratorLayers dense_slabs;
    for (size_t idx = 0; idx < carriers.size(); ++idx) {
        const bool has_sparse = idx < sparse.size() && !sparse[idx].empty();
        const bool has_dense = idx < dense.size() && !dense[idx].empty();
        if (!has_sparse && !has_dense)
            continue;
        throw_if_cancelled(context);
        GeneratorLayer &slab = *carriers[idx];
        const std::optional<ClipperOperand> clearance = column_clearance(clipper, layers, params, out, slab);
        if (has_sparse) {
            ClipperOperand column = clipper(sparse[idx]);
            StoredPolygonCollection area = (clearance ? clipper_diff(column, *clearance) : clipper_union(column)).to_polygon_collection();
            // On the bed layer the column gets the same widened foot as the family's own support.
            if (slab.bottom_print_z() == 0 && !layers.empty())
                area = widen_first_layer_area(object, layers, params, out, area);
            ClipperOperand both = clipper(slab.polygons);
            both += clipper(area);
            slab.polygons = clipper_union(both).to_polygon_collection();
            slab.polygons.ensure_valid(params.resolution);
        }
        if (has_dense) {
            // The same print Z and height as the carrier: publication then removes its area from the carrier's sparse support.
            GeneratorLayer &dense_slab = add_layer(out, LayerType::TopInterface, slab.print_z, slab.bottom_print_z(), slab.height);
            ClipperOperand column = clipper(dense[idx]);
            dense_slab.polygons = (clearance ? clipper_diff(column, *clearance) : clipper_union(column)).to_polygon_collection();
            dense_slab.polygons.ensure_valid(params.resolution);
            dense_slabs.push_back(&dense_slab);
        }
    }
    // Phase 4: the column tops under the object surfaces.
    cap_column_tops(context, print, object, params, layers, out, dense_slabs);
    out.interfaces.insert(out.interfaces.end(), dense_slabs.begin(), dense_slabs.end());
}
} // namespace slic3r_api::Support::Common
