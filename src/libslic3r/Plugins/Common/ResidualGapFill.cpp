///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "ResidualGapFill.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/ParallelFor.hpp"

/* Shared residual geometry for SDK toolpath providers. Snapshots separate reading
   from insertion so changes to trees cannot invalidate inherited attributes.
   Coverage uses millimeter dimensions; the SDK offsets and medial axes use scaled
   distances. Generation leaves flow nominal for the caller's later adjustments.
   Storage is borrowed from the host and must outlive returned collections.
   Callers own snapshots; no state is shared between workers.

       collect_paths -> copy linear points and effective inherited attributes
       coverage -> offset each snapshot, retaining offset and append order
       resolve_limits -> resolve width-relative settings
       generate -> width_range -> open residuals and intersect the allowed domain
           cancellation and area filter -> SDK medial-axis factory
       ordered_boundary -> move old properties with old content
       append_additions -> move new paths into the returned queue

   Exceptions propagate to the caller, which contains them at its C ABI boundary. */
namespace slic3r_api { namespace GapFill {

// Resolve millimeter thresholds through the caller's setting reader; propagate its failures.
Limits resolve_limits(const EffectiveValue &value, double width, coord_t resolution);
// Reject nonpositive nominal flow and return permitted medial widths, retaining bridge diameter.
WidthRange width_range(const c_flow &flow, const Limits &limits);
// Copy linear paths in depth-first order, inheriting attributes; missing or invalid attributes throw.
void collect_paths(const ExtrusionEntity &tree, std::vector<Path> &out, const EPropertyAttributes *inherited);
// Append one offset per snapshot in storage; nonpositive or nonfinite spacing throws.
StoredPolygonCollection coverage(storage_handle *, const std::vector<Path> &, float, Footprint);
// Open residuals, check cancellation before area filtering, and move generated paths into storage.
// Optional extension stays inside the first allowed bound crossing the component; insertion failure throws.
StoredExtrusionEntity generate(storage_handle *, const ExPolygonCollection &, const ExPolygonCollection &,
    const c_flow &, const Limits &, raw_extrusion_role, const plugin_run_context *);
// Move existing content and its properties into an ordered leaf; invalid insertion throws.
MutableExtrusionEntity ordered_boundary(MutableExtrusionEntity tree);
// Transfer additions to the borrowed queue; failed insertion throws without hiding the failure.
void append_additions(MutableExtrusionEntity queue, StoredExtrusionEntity &additions);

Limits resolve_limits(const EffectiveValue &value, double width, coord_t resolution)
{
    Limits limits;
    limits.min_width = value("gap_fill_min_width", width);
    limits.max_width = value("gap_fill_max_width", width);
    limits.min_area = value("gap_fill_min_area", width * width);
    limits.min_length = value("gap_fill_min_length", width);
    limits.extension = value("gap_fill_extension", width);
    limits.resolution = resolution;
    return limits;
}

void collect_paths(const ExtrusionEntity &tree, std::vector<Path> &out, const EPropertyAttributes *inherited)
{
    const EPropertyAttributes *attributes = tree.get(EPropertyAttributes::key);
    if (!attributes)
        attributes = inherited;
    if (tree.has_polyline()) {
        if (!attributes)
            throw std::runtime_error("Missing extrusion attributes for gap fill");
        const c_extrusion_property_attributes &raw = *attributes;
        if (!(raw.width > 0.f && raw.height > 0.f && raw.mm3_per_mm > 0.) ||
            !std::isfinite(raw.width) || !std::isfinite(raw.height) || !std::isfinite(raw.mm3_per_mm))
            throw std::runtime_error("Invalid extrusion dimensions or volume for gap fill");
        out.push_back({tree.linear_points(), *attributes});
    }
    for (uint32_t index = 0; index < tree.child_count(); ++index)
        collect_paths(tree.child(index), out, attributes);
}

StoredPolygonCollection coverage(storage_handle *storage, const std::vector<Path> &paths,
    float spacing_ratio, Footprint footprint)
{
    StoredPolygonCollection result(storage);
    for (const Path &path : paths) {
        const c_extrusion_property_attributes &a = path.attributes;
        const double spacing = footprint == Footprint::Physical ? double(a.width) :
            a.width - a.height * k_rounded_section * spacing_ratio;
        if (!(spacing > 0.) || !std::isfinite(spacing))
            throw std::runtime_error("Invalid gap fill coverage spacing");
        StoredPolyline line(storage);
        line.insert_array(0, path.points.data(), uint32_t(path.points.size()));
        StoredPolylineCollection lines(storage);
        lines.push_back(line);
        StoredPolygonCollection occupied = clipper_offset_polylines(storage, lines,
            scale_d(spacing / 2.) + SCALED_EPSILON).to_polygon_collection();
        result.append_move_from(occupied);
    }
    return result;
}

StoredExtrusionEntity generate(storage_handle *storage, const ExPolygonCollection &residual,
    const ExPolygonCollection &allowed, const c_flow &flow, const Limits &limits,
    raw_extrusion_role role, const plugin_run_context *run_ctx)
{
    const WidthRange range = width_range(flow, limits);
    StoredExtrusionEntity output(storage);
    if (range.minimum > range.maximum)
        return output;
    const ClipperContext clip(storage);
    StoredExPolygonCollection opened = clipper_offset2(clip(residual), -scale_d(range.minimum / 2.),
        scale_d(range.minimum / 2.)).to_expolygon_collection();
    StoredExPolygonCollection areas = clipper_intersection(clip(opened), clip(allowed)).to_expolygon_collection();
    areas.ensure_valid(limits.resolution);
    for (const ExPolygon area : areas) {
        throw_if_cancelled(run_ctx);
        if (unscaled(unscaled(area.area())) <= limits.min_area)
            continue;
        MedialAxisExtrusionFactory factory(role, flow);
        factory.medial_widths(scale_i(range.minimum), scale_i(range.maximum))
            .min_centerline_length(scale_i(limits.min_length)).min_extrusion_length(scale_i(limits.min_length))
            .variable_width_resolution(limits.resolution).endpoint_extension(scale_i(limits.extension))
            .constant_width(flow.is_bridge != 0).can_reverse();
        // Only an explicit extension may anchor beyond the residual component.
        if (limits.extension > 0.) {
            for (const ExPolygon bound : allowed) {
                StoredExPolygonCollection crossing = clipper_intersection(clip(area), clip(bound)).to_expolygon_collection();
                if (!crossing.empty()) {
                    factory.extension_area(bound);
                    break;
                }
            }
        }
        StoredExtrusionEntity generated = factory.build(storage, area);
        if (!generated.readonly().empty() &&
            output.append_child_move(generated.mutable_view()) == EXTRUSION_INDEX_INVALID)
            throw std::runtime_error("Cannot collect residual gap fill");
    }
    return output;
}

WidthRange width_range(const c_flow &flow, const Limits &limits)
{
    const double w = unscaled(flow.width), h = unscaled(flow.height), d = unscaled(flow.nozzle_diameter);
    if (!(w > 0. && h > 0. && d > 0. && flow.mm3_per_mm > 0.))
        throw std::runtime_error("Invalid nominal flow for gap fill");
    // The minimum is the largest of the historical 0.4 * (1 - 0.4) nozzle
    // fraction, the configured minimum, and just above h * (1 - pi/4),
    // below which spacing is nonpositive.
    double minimum = std::max({0.24 * d, limits.min_width, h * k_rounded_section + EPSILON});
    // Permit at most two widths, or the smaller nonzero configured maximum.
    const double maximum = limits.max_width > 0. ? std::min(2. * w, limits.max_width) : 2. * w;
    // A bridge keeps its full width instead of shrinking its circular section.
    if (flow.is_bridge)
        minimum = std::max(minimum, w);
    return {minimum, maximum};
}

MutableExtrusionEntity ordered_boundary(MutableExtrusionEntity tree)
{
    MutableExtrusionEntity queue = tree.emplace_ordered_leaf(OrderedLeafPosition::After,
        ExistingPropertyPlacement::MoveWithExistingContent);
    if (!queue.valid())
        throw std::runtime_error("Cannot order residual gap fill");
    return queue;
}

void append_additions(MutableExtrusionEntity queue, StoredExtrusionEntity &additions)
{
    if (queue.append_child_move(additions.mutable_view()) == EXTRUSION_INDEX_INVALID)
        throw std::runtime_error("Cannot publish residual gap fill");
}

}} // namespace slic3r_api::GapFill
