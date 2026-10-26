///|/ Copyright (c) 2026 NematX
///|/
///|/ Released under the GNU Affero General Public License, version 3 or later.

/*
"Rectilinear around holes" infill pattern plugin
================================================

SDK adapter around AroundHolesKernel.hpp. The host calls run_impl() once per fill
surface (INFILL_PATTERN service). This file:

1. registers the pattern under the id "rectilineararoundholes", the value already
   stored in NematXSlicer 2.7 profiles, and adds it to the sparse, top, bottom and
   solid pattern lists; defines the two region options of the pattern;
2. rotates the surface into the vertical-line frame and builds the two offset
   contour sets with the public Clipper API (the kernel itself never calls the host);
3. publishes the kernel chains as extrusion entities.

A hole bypass is NOT a travel. v3 rejects travel geometry inside printable content
(retraction/lift plugins) and the firmware writes no E on a travel. A bypass is
therefore a thin infill path: same role, flow * fill_rectilinearholes_travel_flow_ratio,
width * 0.1, and its own speed written directly (DefaultSpeed keeps preset speeds).
With a 0% ratio, an EXTRUSION special command with a zero amount makes the firmware
move without extruding; the attributes keep the nominal positive flow because the
flow plugins reject non-positive flows.

Full-fill volume correction is not done here: infill.post_process.full_fill_volume
scales the whole surface tree afterwards, bypasses included, as the legacy code did.
*/

#include <algorithm>
#include <cmath>
#include <limits>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

#include "AroundHolesKernel.hpp"

#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_infill.h"
#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"

namespace nematx_aroundholes {
namespace {

using namespace slic3r_api;
namespace k = nematx_aroundholes;

constexpr char k_plugin_id[] = "rectilineararoundholes";
constexpr char k_speed_key[] = "fill_rectilinearholes_travel_speed";
constexpr char k_flow_key[] = "fill_rectilinearholes_travel_flow_ratio";
constexpr double k_pi = 3.14159265358979323846;

const char *const k_no_dependencies[] = { nullptr };
const char *const k_defined_config_keys[] = { k_speed_key, k_flow_key };
const raw_used_config_key k_used_config_keys[] = {
    { k_speed_key, RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_REGION, RAW_PRESET_TYPE_FFF_PRINT },
    { k_flow_key, RAW_CO_PERCENT, RAW_CONTAINER_TYPE_REGION, RAW_PRESET_TYPE_FFF_PRINT },
    { "travel_speed", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
};
const char *const k_pattern_lists[] = { "fill_pattern", "top_fill_pattern", "bottom_fill_pattern", "solid_fill_pattern" };

bool is_full_infill(const raw_infill_pattern_params &params)
{
    return params.flow.is_bridge != 0 || (params.density > 0.9999f && params.density < 1.0001f);
}

// Fill direction of the legacy Fill::_infill_direction(): bridge angle, or the
// fill angle plus a quarter turn on odd layers when crossing is allowed; then the
// quarter turn that makes the lines vertical.
double infill_angle(const raw_infill_pattern_params &params)
{
    double angle = params.angle;
    if (params.bridge_angle >= 0)
        angle = params.bridge_angle;
    else if ((params.infill_flags & C_INFILL_FLAG_CAN_ANGLE_CROSS) != 0 && (params.layer_id & 1) != 0)
        angle += k_pi / 2.;
    return angle + k_pi / 2.;
}

k::Polygon rotated_polygon(const slic3r_api::Polygon &polygon, double angle)
{
    k::Polygon out;
    out.reserve(polygon.size());
    for (uint32_t i = 0; i < polygon.size(); ++i) {
        const c_point p = polygon.at(i);
        const k::Point q = k::rotated({ p.x, p.y }, angle);
        if (out.empty() || out.back() != q)
            out.push_back(q);
    }
    if (out.size() > 1 && out.front() == out.back())
        out.pop_back();
    return out;
}

StoredPolygonCollection to_stored(storage_handle *storage, const k::Polygons &polygons)
{
    StoredPolygonCollection out(storage);
    for (const k::Polygon &polygon : polygons) {
        StoredPolygon stored(storage);
        for (const k::Point &p : polygon)
            stored.push_back(c_point{ coord_t(p.x), coord_t(p.y) });
        out.push_back_move(std::move(stored));
    }
    return out;
}

// Clipper offset (miter 3) of closed polygons. Contours are forced CCW and holes
// CW, so the material is always on the left of an edge (the kernel relies on it).
k::Polygons offset(storage_handle *storage, const k::Polygons &polygons, double delta)
{
    k::Polygons out;
    if (polygons.empty())
        return out;
    StoredPolygonCollection stored = to_stored(storage, polygons);
    ClipperOperand subject(storage, stored.readonly());
    StoredExPolygonCollection result = clipper_offset(storage, subject, delta).to_expolygon_collection();
    for (const slic3r_api::ExPolygon expolygon : result.readonly()) {
        out.push_back(rotated_polygon(expolygon.contour(), 0.));
        if (k::signed_area(out.back()) < 0.)
            std::reverse(out.back().begin(), out.back().end());
        for (const slic3r_api::Polygon hole : expolygon.holes()) {
            out.push_back(rotated_polygon(hole, 0.));
            if (k::signed_area(out.back()) > 0.)
                std::reverse(out.back().begin(), out.back().end());
        }
    }
    return out;
}

// Drop contours with fewer than 3 points or a tiny area, like the host filter.
void remove_small(k::Polygons &polygons, double min_area)
{
    polygons.erase(std::remove_if(polygons.begin(), polygons.end(), [&](const k::Polygon &p) {
                       return p.size() < 3 || std::abs(k::signed_area(p)) < min_area;
                   }), polygons.end());
}

// Prepare the vertical-line frame exactly like FillRectilinear: contours offset
// by -5% and -48% of the spacing, line spacing from the density, sparse grid
// aligned on the object, first line centred on full surfaces.
k::KernelInput prepare(storage_handle *storage, const slic3r_api::ExPolygon &surface,
                       const raw_infill_pattern_params &params)
{
    k::KernelInput in;
    const double spacing = params.spacing;
    if (spacing <= 0. || params.density <= 0.f)
        return in;
    const double angle = infill_angle(params);
    in.angle = angle;

    k::Polygons source;
    source.push_back(rotated_polygon(surface.contour(), -angle));
    for (const slic3r_api::Polygon hole : surface.holes())
        source.push_back(rotated_polygon(hole, -angle));

    const double offset_outer = double(scale_i(-0.05 * spacing));
    const double offset_inner = double(scale_i(-0.48 * spacing));
    const double min_area = 0.01 * offset_inner * offset_inner;
    // The inner set is offset from the unfiltered outer set, then both are
    // filtered, in the same order as the host ExPolygonWithOffset.
    in.outer = offset(storage, source, offset_outer);
    in.inner = offset(storage, in.outer, offset_inner - offset_outer);
    remove_small(in.outer, min_area);
    remove_small(in.inner, min_area);
    if (in.inner.empty())
        return in;

    k::coord min_x = std::numeric_limits<k::coord>::max(), max_x = std::numeric_limits<k::coord>::min();
    for (const k::Polygon &p : source)
        for (const k::Point &q : p) {
            min_x = std::min(min_x, q.x);
            max_x = std::max(max_x, q.x);
        }

    const double sparse = params.max_sparse_infill_spacing > 0.f ? double(params.max_sparse_infill_spacing) : spacing;
    k::coord line_spacing = scale_i(sparse / double(params.density));
    const bool full = is_full_infill(params);
    if ((full && (params.infill_flags & C_INFILL_FLAG_DONT_ADJUST) == 0) || line_spacing == 0) {
        line_spacing = scale_i(spacing);
    } else if (!full) {
        // Align the sparse grid on the object so lines stay in place between layers.
        const c_bounding_box &box = params.object_bounding_box;
        k::Point center{ (box.min.x + box.max.x) / 2, (box.min.y + box.max.y) / 2 };
        if (box.min.x == box.max.x && box.min.y == box.max.y) {
            const c_bounding_box sbox = surface.contour().bounding_box();
            center = { (sbox.min.x + sbox.max.x) / 2, (sbox.min.y + sbox.max.y) / 2 };
        }
        const k::coord ref = k::rotated(center, -angle).x;
        const k::coord d = min_x - ref;
        const k::coord aligned = ref + (d < 0 ? ((d - line_spacing + 1) / line_spacing) * line_spacing
                                              : (d / line_spacing) * line_spacing);
        min_x = std::min(min_x, aligned);
    }
    if (line_spacing <= 0)
        return in;
    in.line_spacing = line_spacing;
    in.line_count = size_t(1 + (max_x - min_x - 10) / line_spacing);
    in.x0 = min_x;
    if (params.flow.is_bridge != 0 && params.bridge_offset >= 0)
        in.x0 += params.bridge_offset;
    else if (full)
        in.x0 += (line_spacing + k::coord(SCALED_EPSILON)) / 2;
    in.min_length = params.flow.spacing;
    in.bypass_offset = params.flow.spacing / 2;
    return in;
}

// Set the ordering permissions of one entity (after its children are added).
template<class Entity> void set_order(Entity &&entity, bool reversible)
{
    uint32_t flags = entity.flags() & ~uint32_t(RAW_EXTRUSION_FLAG_SORTABLE | RAW_EXTRUSION_FLAG_REVERSIBLE);
    if (reversible)
        flags |= RAW_EXTRUSION_FLAG_REVERSIBLE;
    if (!entity.set_flags(flags))
        throw std::runtime_error("Rectilinear around holes: cannot set extrusion flags.");
}

class AroundHolesPattern final : public PluginBase
{
public:
    explicit AroundHolesPattern(orchestrator_handle *orchestrator) : PluginBase(orchestrator) {}

private:
    const char *id_impl() const noexcept override { return k_plugin_id; }
    const char *name_impl() const noexcept override { return "Rectilinear around holes"; }
    const char *description_impl() const noexcept override
    {
        return "Rectilinear lines that go around the holes with a low-flow path instead of travelling over them.";
    }
    slicing_step_t step_impl() const noexcept override { return INFILL_PATTERN; }
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }
    int32_t priority_impl() const noexcept override { return 30; }

    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override
    {
        if (keys != nullptr)
            std::copy(std::begin(k_used_config_keys), std::end(k_used_config_keys), keys);
        return int32_t(std::size(k_used_config_keys));
    }

    int32_t defined_config_keys(const char **keys) const noexcept override
    {
        if (keys != nullptr)
            std::copy(std::begin(k_defined_config_keys), std::end(k_defined_config_keys), keys);
        return int32_t(std::size(k_defined_config_keys));
    }

    void define_option(const char *key, raw_config_option_type type, const char *label, const char *full_label,
                       const char *tooltip, const char *sidetext, const char *default_value, const char *ratio_over) const
    {
        raw_config_option_def def = raw_config_option_def_init();
        def.opt_key = key;
        def.type = type;
        def.category = RAW_OPTION_CATEGORY_INFILL;
        def.level = RAW_OPTION_LEVEL_EXPERT;
        def.mode = RAW_CONFIG_OPTION_MODE_EXPERT;
        def.container_type = RAW_CONTAINER_TYPE_REGION;
        def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
        def.printer_technology = RAW_PT_FFF;
        def.invalidates_step = STEP_INFILL;
        def.label = label;
        def.full_label = full_label;
        def.tooltip = tooltip;
        def.sidetext = sidetext;
        def.ratio_over = ratio_over;
        def.has_min = 1;
        def.min_value = 0.;
        def.default_serialized_value = default_value;
        if (orchestrator_create_option_def(m_orchestrator, &def) != OPTION_DEF_ERROR_OK)
            throw std::runtime_error(std::string("Cannot define ") + key);
    }

    void inilialize_impl(storage_handle *) const override
    {
        define_option(k_speed_key, RAW_CO_FLOAT_OR_PERCENT, "Travel speed", "Rectilinear around holes infill: travel speed",
                      "Speed of the low-flow path that goes around the holes, in mm/s or as a percentage of the travel speed."
                      "\nIf set to 0, the travel speed is used (same as 100%).",
                      "mm/s or %", "100%", "travel_speed");
        define_option(k_flow_key, RAW_CO_PERCENT, "Travel flow ratio", "Rectilinear around holes infill: travel flow ratio",
                      "Percentage of the infill flow still extruded on the path that goes around the holes, to keep the "
                      "pressure in the hotend. 0% moves without extruding.",
                      "%", "1%", nullptr);
        for (const char *list : k_pattern_lists) {
            key_value_string_pair_t entry = { k_plugin_id, "Rectilinear around holes" };
            key_value_string_pair_array_t entries = { &entry, 1 };
            raw_option_def_patch patch = raw_option_def_patch_init();
            patch.kind = RAW_OPTION_DEF_PATCH_ADD_ENUM_VALUES;
            patch.opt_key = list;
            patch.enum_values = entries;
            orchestrator_modify_option_def(m_orchestrator, &patch);
        }
    }

    void run_impl(const plugin_run_context *run_ctx) const override
    {
        throw_if_cancelled(run_ctx);
        const run_ctx_infill_pattern *ctx = plugin_ctx_as_infill_pattern(run_ctx);
        if (ctx == nullptr || ctx->surface == nullptr || ctx->params == nullptr || ctx->output == nullptr ||
            ctx->print == nullptr || ctx->primary_region == nullptr || run_ctx->plugin_storage == nullptr)
            return;
        storage_handle *storage = run_ctx->plugin_storage;
        const raw_infill_pattern_params &params = *ctx->params;

        const k::KernelInput in = prepare(storage, Surface(ctx->surface).expolygon(), params);
        const std::vector<k::Chain> chains = k::build_chains(in);
        throw_if_cancelled(run_ctx);
        if (chains.empty())
            return;

        // Bypass settings, from the surface's region.
        const Config region = LayerRegion(ctx->primary_region).print_region().config();
        const double travel_speed = Print(ctx->print).config().float_or_default("travel_speed", 0.);
        double bypass_speed = region.effective_float_or_percent_or_default(k_speed_key, travel_speed, 0.);
        if (!(bypass_speed > 0.))
            bypass_speed = travel_speed;
        if (!std::isfinite(bypass_speed) || bypass_speed <= 0.)
            throw std::runtime_error("Rectilinear around holes needs a positive travel speed.");
        const double flow_ratio = region.percent_or_default(k_flow_key, 0.01);
        if (!std::isfinite(flow_ratio) || flow_ratio < 0.)
            throw std::runtime_error(std::string(k_flow_key) + " must be a finite non-negative percentage.");

        const double mm3 = params.flow.mm3_per_mm * double(params.flow_mult);
        const float width = float(unscaled(params.flow.width));
        const float height = float(unscaled(params.flow.height));

        auto make_leaf = [&](const k::Element &element) {
            std::vector<c_point> points;
            points.reserve(element.points.size());
            for (const k::Point &p : element.points)
                points.push_back(c_point{ coord_t(p.x), coord_t(p.y) });
            StoredExtrusionEntity leaf(storage, points);
            EPropertyAttributes &attributes = leaf.get_or_add(EPropertyAttributes::key);
            attributes.extrusion_role(int32_t(params.extrusion_role)).height(height);
            if (!element.bypass) {
                attributes.mm3_per_mm(mm3).width(width * params.flow_mult);
                return leaf;
            }
            // At 0% keep a tiny positive flow: flow plugins reject zero, the zero
            // EXTRUSION command below overrides E, and the full-fill volume step
            // then counts (almost) no material for the bypass, like the legacy code.
            attributes.mm3_per_mm(mm3 * (flow_ratio > 0. ? flow_ratio : 1e-6)).width(width * 0.1f);
            // A new speed payload is zero-filled: clear every field, then set the speed.
            EPropertySpeed &speed = leaf.get_or_add(EPropertySpeed::key);
            speed.speed_mm_per_s = float(bypass_speed);
            speed.accel_mm_per_s2 = -1.f;
            speed.pressure_adv = -1.f;
            speed.fan_speed_percent = -1.f;
            if (flow_ratio == 0.)
                leaf.get_or_add(EPropertySpecialCommand::key).set(C_EXTRUSION_SPECIAL_COMMAND_EXTRUSION, 0.);
            return leaf;
        };

        // Root and chains are ordered (not sortable) like the legacy collection;
        // a chain may be reversed as a whole, never element by element, so the
        // printed lines and their bypasses stay connected.
        StoredExtrusionEntity root(storage);
        for (const k::Chain &chain : chains) {
            throw_if_cancelled(run_ctx);
            StoredExtrusionEntity group(storage);
            for (const k::Element &element : chain) {
                StoredExtrusionEntity leaf = make_leaf(element);
                group.append_child_move(leaf.mutable_view());
            }
            for (uint32_t i = 0; i < group.child_count(); ++i)
                set_order(group.child_mutable(i), chain.size() == 1);
            set_order(group, true);
            root.append_child_move(group.mutable_view());
        }
        // Flags after the children: the first insertion makes an entity sortable.
        set_order(root, true);
        MutableExtrusionEntity(ctx->output).append_child_move(root.mutable_view());
    }
};

} // namespace
} // namespace nematx_aroundholes

extern "C" SLIC3R_PLUGIN_API void register_plugin(orchestrator_handle *orchestrator)
{
    static nematx_aroundholes::AroundHolesPattern plugin(orchestrator);
    orchestrator_register_plugin(orchestrator, plugin.c_instance());
}

// Keep this header last so its ABI registration includes every public API header above.
#include "libslic3r/Api/plugin/c/slic3r_plugin_register_version.h"
