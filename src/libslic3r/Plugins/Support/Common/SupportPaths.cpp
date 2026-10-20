///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#include "SupportPaths.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "PerimeterFill.hpp"
#include "libslic3r/Api/plugin/c/slic3r_extrusion_entity.h"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"

/* Implementation of the shared support path helpers (see SupportPaths.hpp). Each helper keeps the statements,
   constants and rounding of the legacy code it was taken from, so the base paths of the legacy support are
   unchanged. Stationary support deposits are built directly as one-point entities, with geometric volume before
   the extrusion multiplier and with the support flow attributes used by the writer. */

namespace slic3r_api::Support::Common {

// Degrees to radians in float, as the native Geometry::deg2rad<float>().
float radians(float degrees)
{
    return float(PI) * degrees / float(180.0);
}

// Set every field the legacy base fills share; only the pattern, role, flow, spacing, density, angle and phase vary.
raw_infill_pattern_params base_fill_recipe(const run_ctx_support_toolpath &payload, const std::string &pattern,
                                           raw_extrusion_role role, const c_flow &flow, double spacing, float density,
                                           float angle, double z)
{
    raw_infill_pattern_params recipe = {};
    recipe.pattern_id = payload.resolve_pattern_id(&payload, pattern.c_str());
    recipe.surface_type = raw_surface_type(RAW_SURFACE_TYPE_POS_INTERNAL | RAW_SURFACE_TYPE_DENS_SPARSE);
    recipe.extrusion_role = role;
    recipe.flow = flow;
    recipe.density = density;
    recipe.flow_mult = 1.f;
    recipe.connection = C_INFILL_CONNECTION_CONNECTED;
    // The legacy fills keep Fill's default can_angle_cross, so a smooth ironing pass turns a quarter turn.
    recipe.infill_flags = C_INFILL_FLAG_DONT_ADJUST | C_INFILL_FLAG_CAN_ANGLE_CROSS;
    recipe.spacing = spacing;
    recipe.angle = angle;
    recipe.bridge_angle = -1.;
    recipe.z = z;
    recipe.layer_height = unscaled(flow.height);
    recipe.bridge_offset = -1;
    recipe.fill_resolution = scale_i(0.0125);
    recipe.anchor_length = 1000.f;
    recipe.anchor_length_max = 1000.f;
    recipe.object_bounding_box.min = c_point{-scale_i(1.), -scale_i(1.)};
    recipe.object_bounding_box.max = c_point{scale_i(1.), scale_i(1.)};
    return recipe;
}

// Each area is filled by the recipe pattern alone, or inside a perimeter loop with the 0.2 inset ratio of the native flange.
// Borrowed context, payload and recipe outlive the synchronous fill; storage owns temporaries and out receives paths.
// Stopping exits through silent PluginCancelled (the cause has its message, the step names it, or the runner rethrows
// the print cancellation); ordinary refusals
// throw std::runtime_error.
void fill_base_areas(const plugin_run_context *context, const run_ctx_support_toolpath &payload, storage_handle *storage, MutableExtrusionEntity out,
                     const ExPolygonCollection &areas, const layer_handle *layer, const raw_infill_pattern_params &recipe,
                     bool perimeter)
{
    const InnerFill inner = pattern_service_fill(context, payload, recipe.pattern_id, recipe, layer, nullptr, nullptr, layer_get_region(layer, 0));
    const raw_surface_type surface_type = raw_surface_type(RAW_SURFACE_TYPE_POS_INTERNAL | RAW_SURFACE_TYPE_DENS_SPARSE);
    for (const ExPolygon area : areas) {
        if (perimeter) {
            PerimeterFillSettings settings;
            settings.spacing = recipe.spacing;
            settings.ratio_fill_inside = 0.2f;
            settings.role = recipe.extrusion_role;
            settings.flow = recipe.flow;
            fill_with_perimeter(storage, Surface(area, surface_type), settings, inner, out);
            continue;
        }
        StoredSurfaceCollection surfaces(storage);
        surfaces.append(area, surface_type);
        StoredExtrusionEntity pattern_output(storage);
        inner(surface_collection_at(surfaces.handle(), 0), pattern_output.mutable_view());
        move_children(out, pattern_output.mutable_view());
    }
}

// Each polyline becomes a path of the given reversibility; a closed one is wrapped in a loop, which is never reversible.
void append_paths(storage_handle *storage, MutableExtrusionEntity parent, const std::vector<std::vector<c_point>> &lines,
                  const EPropertyAttributes &attributes, bool reversible)
{
    for (const std::vector<c_point> &points : lines) {
        if (points.size() < 2) continue;
        StoredExtrusionEntity path(storage, points);
        path.set_flags(reversible ? uint32_t(RAW_EXTRUSION_FLAG_REVERSIBLE) : 0u);
        path.get_or_add(EPropertyAttributes::key) = attributes;
        const bool closed = points.front().x == points.back().x && points.front().y == points.back().y;
        if (!closed) {
            parent.append_child_move(path.mutable_view());
            continue;
        }
        // A closed path is an ExtrusionLoop: neither sortable nor reversible.
        StoredExtrusionEntity loop(storage);
        loop.append_child_move(path.mutable_view());
        loop.set_flags(0u);
        parent.append_child_move(loop.mutable_view());
    }
}

// Build a one-point leaf from a fused island and support flow, then move it into a parent that accepts children. storage
// owns the temporary entity; parent receives its contents. Holes reduce volume but not the contour centroid.
void append_deposit(storage_handle *storage, MutableExtrusionEntity parent, const ExPolygon &island, const c_flow &flow)
{
    const double volume = unscaled(unscaled(island.area())) * unscaled(flow.height);
    // A degenerate island prints nothing.
    if (!std::isfinite(volume) || volume <= 0.)
        return;
    StoredExtrusionEntity deposit(storage, std::vector<c_point>{island.contour().centroid()});
    deposit.set_flags(0u);
    deposit.get_or_add(EPropertyAttributes::key).extrusion_role(int32_t(RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL))
        .mm3_per_mm(flow.mm3_per_mm).width(float(unscaled(flow.width))).height(float(unscaled(flow.height)));
    deposit.get_or_add(EPropertySpecialCommand::key).set(C_EXTRUSION_SPECIAL_COMMAND_DEPOSIT, volume);
    parent.append_child_move(deposit.mutable_view());
}

// The polylines are copied into a host collection, which simplifies its own copy in place; the input is not modified.
std::vector<std::vector<c_point>> valid_polylines(storage_handle *storage, const std::vector<std::vector<c_point>> &lines,
                                                  coord_t resolution)
{
    StoredPolylineCollection collection(storage);
    for (const std::vector<c_point> &points : lines) {
        StoredPolyline line(storage);
        line.insert_array(0, points.data(), uint32_t(points.size()));
        collection.push_back(line);
    }
    collection.ensure_valid(resolution);
    std::vector<std::vector<c_point>> result;
    for (const Polyline line : collection)
        result.push_back(line.points());
    return result;
}

// The children are moved one by one, so src keeps its own flags while dst receives the paths.
void move_children(MutableExtrusionEntity dst, MutableExtrusionEntity src)
{
    for (uint32_t index = 0; index < src.readonly().child_count(); ++index)
        dst.append_child_move(src.child_mutable(index));
}

} // namespace slic3r_api::Support::Common
