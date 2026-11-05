///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#include "SupportPaths.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "PerimeterFill.hpp"
#include "libslic3r/Api/plugin/c/slic3r_extrusion_entity.h"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"

/* Implementation of the shared support path helpers (see SupportPaths.hpp). These helpers reproduce the native
   base support paths; do not change their statements, constants or rounding without updating the legacy comparisons.
   Stationary support deposits are built directly as one-point entities, with geometric volume before
   the extrusion multiplier and with the support flow attributes used by the writer. Toolpath callbacks share the
   marked-layer walk and per-execution parameters. The recipe builder accepts the layer identity and anchoring box;
   sampled tree patterns use an empty box, whereas toolpaths use a +/-1 mm box. Bed flanges use the common
   first-layer recipe and the perimeter fill. */

namespace slic3r_api::Support::Common {

// Borrows matching print/object views and derives owned geometry; sheaths alone read their double-wall diameter, and allocation/config failures propagate.
ToolpathParams::ToolpathParams(const Print &print, const Object &object, bool sheaths)
    : geometry(print, object)
{
    if (sheaths) {
        const double diameter = object.config().get("support_tree_branch_diameter_double_wall").get_float();
        double_wall_area = 0.25 * scale_d(diameter) * scale_d(diameter) * PI;
    }
}

// Validates a borrowed payload, constructs one owned parameter set when auxiliary layers exist, and calls generate for marked layers in host order; layers own published paths, missing payload throws, and cancellation/callback failures propagate.
void for_support_layers(const plugin_run_context *context, bool sheaths, const SupportLayerPaths &generate)
{
    const run_ctx_support_toolpath *payload = plugin_ctx_as_support_toolpath(context);
    if (!payload || !payload->object || !payload->print)
        throw std::runtime_error("Missing support toolpath payload");
    if (object_count_auxiliary_layer(payload->object) == 0)
        return;
    const Print print(payload->print);
    const Object object(payload->object);
    const ToolpathParams params(print, object, sheaths);
    for (uint32_t index = 0; index < object_count_auxiliary_layer(payload->object); ++index) {
        throw_if_cancelled(context);
        layer_handle *layer = object_get_auxiliary_layer(payload->object, index);
        if (plugin_property_has(layer_get_properties(layer), PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT))
            generate(*payload, object, params, layer);
    }
}

// Zero-initializes owned attributes, then copies role and flow values with dimensions unscaled before float conversion; no_seam remains zero, the borrowed flow is unchanged and no host calls are made.
EPropertyAttributes path_attributes(raw_extrusion_role role, const c_flow &flow)
{
    EPropertyAttributes attributes{};
    attributes.extrusion_role(int32_t(role)).mm3_per_mm(flow.mm3_per_mm)
        .width(float(unscaled(flow.width))).height(float(unscaled(flow.height)));
    return attributes;
}

// Degrees to radians in float, as the native Geometry::deg2rad<float>().
float radians(float degrees)
{
    return float(PI) * degrees / float(180.0);
}

// Copies the supplied flow, millimeter spacing/Z, radian angle, layer index and box into an owned sparse recipe with fixed support flags; density is in (0,1], inputs remain unchanged and no host calls are made.
// Fixed fields are an internal sparse surface, DONT_ADJUST, multiplier 1, 1000 mm anchors and 0.0125 mm resolution; the default box is empty, while toolpaths supply +/-1 mm.
raw_infill_pattern_params fill_recipe(infill_pattern_runtime_id pattern, raw_extrusion_role role, const c_flow &flow,
                                      double spacing, float density, float angle, double z, uint32_t layer_id,
                                      c_bounding_box object_box)
{
    raw_infill_pattern_params recipe = {};
    recipe.pattern_id = pattern;
    recipe.surface_type = raw_surface_type(RAW_SURFACE_TYPE_POS_INTERNAL | RAW_SURFACE_TYPE_DENS_SPARSE);
    recipe.extrusion_role = role;
    recipe.flow = flow;
    recipe.density = density;
    recipe.flow_mult = 1.f;
    recipe.connection = C_INFILL_CONNECTION_CONNECTED;
    // CAN_ANGLE_CROSS turns rectilinear fill by a quarter turn every other layer_id.
    recipe.infill_flags = C_INFILL_FLAG_DONT_ADJUST | C_INFILL_FLAG_CAN_ANGLE_CROSS;
    recipe.spacing = spacing;
    recipe.angle = angle;
    recipe.bridge_angle = -1.;
    recipe.z = z;
    recipe.layer_id = layer_id;
    recipe.layer_height = unscaled(flow.height);
    recipe.bridge_offset = -1;
    recipe.fill_resolution = scale_i(0.0125);
    recipe.anchor_length = 1000.f;
    recipe.anchor_length_max = 1000.f;
    recipe.object_bounding_box = object_box;
    return recipe;
}

// Resolves a pattern through the borrowed synchronous payload and builds a recipe anchored to a +/-1 mm box; valid callbacks are required and host failures propagate.
raw_infill_pattern_params base_fill_recipe(const run_ctx_support_toolpath &payload, const std::string &pattern,
                                           raw_extrusion_role role, const c_flow &flow, double spacing, float density,
                                           float angle, double z)
{
    c_bounding_box box = {};
    box.min = c_point{-scale_i(1.), -scale_i(1.)};
    box.max = c_point{scale_i(1.), scale_i(1.)};
    return fill_recipe(payload.resolve_pattern_id(&payload, pattern.c_str()), role, flow, spacing, density, angle, z, 0, box);
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

// Builds a rectilinear recipe at phase zero and base angle plus 90 degrees, then appends perimeter-wrapped paths for borrowed fused bed areas; storage owns temporaries, out receives paths, and pattern failures/cancellation propagate.
void append_base_flange(const plugin_run_context *context, const run_ctx_support_toolpath &payload, storage_handle *storage,
                        MutableExtrusionEntity out, const ExPolygonCollection &areas, const layer_handle *layer,
                        const SupportParams &params)
{
    const raw_infill_pattern_params recipe = base_fill_recipe(payload, "rectilinear", RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL,
        params.first_flow, params.first_flow_mm.spacing, float(params.raft_first_layer_density),
        radians(float(params.base_angle_degrees + 90.)), 0.);
    fill_base_areas(context, payload, storage, out, areas, layer, recipe, true);
}

// Each borrowed polyline with at least two points becomes a storage-owned path with the supplied flags/attributes, moved into parent; closed paths get fixed loop parents, and allocation failures propagate.
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

namespace {
// Where an island holds its deposit: the contour centroid when the island contains it, else the point of the boundary (contour and
// holes) closest to the centroid, found by projecting the centroid on every segment. A crescent or a C-shaped island has its
// centroid outside.
c_point deposit_point(const ExPolygon &island)
{
    const c_point centroid = island.contour().centroid();
    if (island.contains(centroid))
        return centroid;
    c_point best = centroid;
    double best_squared = std::numeric_limits<double>::max();
    const auto visit = [&](const Polygon &ring) {
        const multipoint_const_view points = ring.view();
        for (uint32_t index = 0; index < points.size; ++index) {
            const c_point a = points.array[index], b = points.array[(index + 1) % points.size];
            const double abx = double(b.x) - double(a.x), aby = double(b.y) - double(a.y);
            const double length_squared = abx * abx + aby * aby;
            const double t = length_squared > 0. ?
                std::clamp(((double(centroid.x) - double(a.x)) * abx + (double(centroid.y) - double(a.y)) * aby) / length_squared, 0., 1.) : 0.;
            const double dx = double(a.x) + t * abx - double(centroid.x), dy = double(a.y) + t * aby - double(centroid.y);
            if (dx * dx + dy * dy < best_squared) {
                best_squared = dx * dx + dy * dy;
                best = c_point{ coord_t(std::llround(double(a.x) + t * abx)), coord_t(std::llround(double(a.y) + t * aby)) };
            }
        }
    };
    visit(island.contour());
    for (uint32_t hole = 0; hole < island.hole_size(); ++hole)
        visit(island.hole(hole));
    return best;
}
} // namespace

// Build a one-point leaf from a fused island and support flow, then move it into a parent that accepts children. storage
// owns the temporary entity; parent receives its contents. Holes reduce the volume (net area times layer height). The centroid is the
// contour's: when it falls in a hole or outside the contour, the deposit goes to the closest boundary point, holes included.
// Nonfinite or nonpositive volume appends nothing; valid island views are required and geometry/allocation failures propagate.
void append_deposit(storage_handle *storage, MutableExtrusionEntity parent, const ExPolygon &island, const c_flow &flow)
{
    const double volume = unscaled(unscaled(island.area())) * unscaled(flow.height);
    // A degenerate island prints nothing.
    if (!std::isfinite(volume) || volume <= 0.)
        return;
    StoredExtrusionEntity deposit(storage, std::vector<c_point>{deposit_point(island)});
    deposit.set_flags(0u);
    deposit.get_or_add(EPropertyAttributes::key) = path_attributes(RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL, flow);
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
