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
#include "libslic3r/Plugins/Support/Common/SupportZPlacement.hpp"

/* Implementation of the support columns (see SupportColumns.hpp).

   The columns are added after the family, on its finished slabs, in three phases:
   1. Heights: the Z span of every column comes from its transformed mesh, rounded to the machine Z step; the spans are united.
      A span is never lowered to the bed: the column starts where its mesh starts.
   2. Slabs: the sparse slabs of out.base that meet a span carry the column there. The parts of the spans that no sparse slab
      covers are cut at the object layer tops (above the object, every top layer height) into new base slabs; a piece thinner
      than the minimum support height joins its neighbor.
   3. Areas: every column is sliced once at the middle Z of every carrying slab. A section loses the object clearance (the object
      layers within the slab's Z gaps, with their no-support zones, grown by the XY gap, as the grid family trims its slabs) and
      the areas of the contacts and interfaces that overlap the slab in Z, then joins the slab's polygons by a union.
   Sparse slabs do not overlap each other in Z, so a column area is printed by one slab only. The geometry lives in out's storage.

       add_support_columns()
       |-- column_spans(): Z spans of the columns
       |-- uncovered() / gap_pieces(): new base slabs where no sparse slab covers a span
       |-- slice_support_volumes(): column sections at the middle of every carrying slab
       `-- column_clearance(): object, zones, contacts and interfaces removed before the union */

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
        if (!volume.is_support_column())
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
} // namespace

bool has_support_columns(const Object &object)
{
    if (!object.config().get("support_material").get_bool())
        return false;
    for (uint32_t volume_idx = 0; volume_idx < object.volume_count(); ++volume_idx)
        if (object.volume(volume_idx).is_support_column())
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
    const std::vector<StoredExPolygonCollection> sections =
        slice_support_volumes(print, object, RAW_VOLUME_TYPE_SUPPORT_COLUMN, slice_zs, out.storage, context);
    const ClipperContext clipper(out.storage);
    for (size_t idx = 0; idx < carriers.size() && idx < sections.size(); ++idx) {
        if (sections[idx].empty())
            continue;
        throw_if_cancelled(context);
        GeneratorLayer &slab = *carriers[idx];
        ClipperOperand column = clipper(sections[idx]);
        if (std::optional<ClipperOperand> clearance = column_clearance(clipper, layers, params, out, slab))
            column = clipper_diff(column, *clearance);
        ClipperOperand both = clipper(slab.polygons);
        both += column;
        slab.polygons = clipper_union(both).to_polygon_collection();
        slab.polygons.ensure_valid(params.resolution);
    }
}
} // namespace slic3r_api::Support::Common
