///|/ Released under the GNU Affero General Public License, version 3 or later.
#include "InterfaceGapFill.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "libslic3r/Plugins/Support/Common/ClipperResults.hpp"

/* Residual geometry for dense support toolpaths. Reading the old trees is
   separate from generation so no insertion can invalidate the snapshots.
   Widths and heights in attributes are millimeters; the SDK flow and medial
   axis limits use scaled distances. Each caller owns its paths and storage.

       collect_paths -> resolve inherited attributes into independent snapshots
       coverage -> offset snapshots with the effective printed dimensions
       generate -> open residuals, filter areas, build nominal SDK extrusions

   Exceptions propagate to PluginBase, which contains them at the C ABI. */
namespace nematx::interface_gaps {
namespace Common = slic3r_api::Support::Common;
constexpr double k_rounded_section = 1. - 3.14159265358979323846 / 4.;

void collect_paths(const ExtrusionEntity &tree, std::vector<Path> &out, const EPropertyAttributes *inherited)
{
    const EPropertyAttributes *attributes = tree.get(EPropertyAttributes::key);
    if (!attributes) attributes = inherited;
    if (tree.has_polyline()) {
        if (!attributes) throw std::runtime_error("Missing support extrusion attributes");
        const c_extrusion_property_attributes &raw = *attributes;
        if (!(raw.width > 0.f && raw.height > 0.f && raw.mm3_per_mm > 0.) ||
            !std::isfinite(raw.width) || !std::isfinite(raw.height) || !std::isfinite(raw.mm3_per_mm))
            throw std::runtime_error("Invalid support extrusion dimensions or volume");
        out.push_back({tree.points(), *attributes});
    }
    for (uint32_t index = 0; index < tree.child_count(); ++index)
        collect_paths(tree.child(index), out, attributes);
}

StoredPolygonCollection coverage(storage_handle *storage, const std::vector<Path> &paths,
                                 const c_flow &flow, bool physical)
{
    StoredPolygonCollection result(storage);
    for (const Path &path : paths) {
        const c_extrusion_property_attributes &a = path.attributes;
        // A path's section is a rectangle with rounded ends: neighbouring paths overlap
        // by height * (1 - pi/4), so a path covers its spacing. Bridges and obstacles
        // from another recipe count with their full width.
        const double spacing = physical || flow.is_bridge ? double(a.width) :
            a.width - a.height * k_rounded_section * flow.spacing_ratio;
        if (!(spacing > 0.)) throw std::runtime_error("Nonpositive support coverage spacing");
        StoredPolyline line(storage);
        line.insert_array(0, path.points.data(), uint32_t(path.points.size()));
        StoredPolylineCollection lines(storage);
        lines.push_back(line);
        StoredExPolygonCollection footprint = clipper_offset_polylines(storage, lines,
            scale_d(spacing / 2.) + SCALED_EPSILON).to_expolygon_collection();
        Common::append_polygons(result, footprint);
    }
    return result;
}

StoredExtrusionEntity generate(storage_handle *storage, const ExPolygonCollection &residual,
                               const ExPolygonCollection &allowed, c_flow flow, const Limits &limits)
{
    const double w = unscaled(flow.width), h = unscaled(flow.height), d = unscaled(flow.nozzle_diameter);
    if (!(w > 0. && h > 0. && d > 0. && flow.mm3_per_mm > 0.))
        throw std::runtime_error("Cannot compute support interface flow");
    // Narrowest gap: the largest of the historical 0.4 * (1 - 0.4) of the nozzle, the
    // configured minimum, and just above height * (1 - pi/4), below which a path would
    // have no positive spacing; a bridge keeps its full width. Widest gap: two path
    // widths, or the smaller configured maximum.
    double minimum = std::max({0.24 * d, limits.min_width, h * k_rounded_section + EPSILON});
    const double maximum = limits.max_width > 0. ? std::min(2. * w, limits.max_width) : 2. * w;
    if (flow.is_bridge) minimum = std::max(minimum, w);
    StoredExtrusionEntity output(storage);
    if (minimum > maximum) return output;
    const ClipperContext clipper(storage);
    StoredExPolygonCollection opened = clipper_offset2(clipper(residual), -scale_d(minimum / 2.),
        scale_d(minimum / 2.)).to_expolygon_collection();
    StoredExPolygonCollection areas = clipper_intersection(clipper(opened), clipper(allowed)).to_expolygon_collection();
    areas.ensure_valid(limits.resolution);
    for (const ExPolygon area : areas) {
        if (unscaled(unscaled(area.area())) <= limits.min_area) continue;
        MedialAxisExtrusionFactory factory(RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL_INTERFACE, flow);
        factory.medial_widths(scale_i(minimum), scale_i(maximum))
            .min_centerline_length(scale_i(limits.min_length))
            .min_extrusion_length(scale_i(limits.min_length))
            .variable_width_resolution(limits.resolution)
            .endpoint_extension(scale_i(limits.extension))
            .constant_width(flow.is_bridge != 0).can_reverse();
        // Only an explicit extension may anchor beyond the residual component.
        if (limits.extension > 0.) {
            for (const ExPolygon bound : allowed) {
                StoredExPolygonCollection crossing = clipper_intersection(clipper(area), clipper(bound)).to_expolygon_collection();
                if (!crossing.empty()) { factory.extension_area(bound); break; }
            }
        }
        StoredExtrusionEntity generated = factory.build(storage, area);
        if (!generated.readonly().empty() && output.append_child_move(generated.mutable_view()) == EXTRUSION_INDEX_INVALID)
            throw std::runtime_error("Cannot collect support interface gap fill");
    }
    return output;
}
}
