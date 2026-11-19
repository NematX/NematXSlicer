///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "NoTravelRouting.hpp"

#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Api/plugin/cpp/VolumeViews.hpp"
#include "libslic3r/Plugins/Common/ClipperHelpers.hpp"

/*
Best-effort NoTravel routing shared by travel providers
=======================================================

A NoTravel volume is a model volume that adds no material; travels should go
around its section at the current layer. Default straight travel uses these
helpers for every ordinary gap. Advanced travel uses them for gaps outside
perimeter avoidance and for routes its zone planner could not admit; its zone
planner also receives the same sections as forbidden zones.

sections() reads the immutable plan once per layer worker. It places every
NoTravel volume of every real instance with the object, volume and instance
transforms, slices it at print_z, and unions the rings in platter coordinates.
A thin or sloped volume that does not cut print_z is not avoided.

path() then works per gap. It exempts every section containing an endpoint, so a
travel starting or ending inside a volume may cross it. It tests the direct gap
against the remaining sections shrunk by SCALED_EPSILON, so a gap along a border
is admitted. A crossing gap gets one CLIPPER_DETOUR_SHORTEST detour on the
section rings, validated against the same shrunk sections; a rejected detour
keeps the direct gap. Each call owns its Clipper operands in the worker's plugin
storage; nothing survives the call except the returned path and counts.

    sections          slice NoTravel volumes at print_z and union them in platter space
    path              direct gap or one validated detour
    |-- exemption     drop sections containing an endpoint
    `-- detour        clipper_detour_polyline_expolygons, then validation
    warning_message   execution summary of exemptions and fallbacks
*/

namespace slic3r_api { namespace LayerExtrusionEdit { namespace NoTravel {

using namespace TravelConnection;
namespace clipper = PluginCommon::clipper;

polygon_work::ExPolygons sections(const ConstPrintingPlan &plan, coord_t print_z,
                                  const plugin_run_context *run_ctx)
{
    StoredPolygonCollection rings(run_ctx->plugin_storage);
    for (uint32_t i = 0; i < plan.object_instance_count(); ++i) {
        throw_if_cancelled(run_ctx);
        const PrintingObjectInstance instance = plan.object_instance(i);
        if (!instance.has_object()) continue;
        const Object object = instance.object();
        const c_point center = object.center_offset();
        const c_point shift = object.instance_shift(uint32_t(instance.object_instance_idx()));
        const c_matrix4d transform = matrix4d_mul(matrix4d_translation(
            unscaled(shift.x) - unscaled(center.x), unscaled(shift.y) - unscaled(center.y), 0), object.transform());
        for (uint32_t v = 0; v < object.volume_count(); ++v) {
            throw_if_cancelled(run_ctx);
            const Volume volume = object.volume(v);
            if (!volume.is_no_travel()) continue;
            c_mesh_slicing_params params{};
            params.transform = matrix4d_mul(transform, volume_get_matrix(volume.handle()));
            auto slices = volume.mesh().slice_to_polygons(run_ctx->plugin_storage, params,
                {float(unscaled(print_z))}, run_ctx);
            rings.append_move_from(slices.front());
        }
    }
    if (rings.empty()) return {};
    const auto merged = clipper_union_ex(run_ctx->plugin_storage, rings.readonly(), CLIPPER_FILL_NONZERO);
    return clipper::from_expolygon_collection_view(merged.readonly());
}

std::vector<c_point> path(const TravelEndpoint &source, const TravelEndpoint &target, extruder_index_t tool,
                          const bool layer_entry, const coord_t print_z, const polygon_work::ExPolygons &sections,
                          const plugin_run_context *run_ctx, Counts &counts)
{
    const std::vector<c_point> direct = straight_path(source, target, tool);
    if (sections.empty() || layer_entry || source.position.z != print_z || target.position.z != print_z)
        return direct;
    polygon_work::ExPolygons obstacles;
    bool exempt = false;
    for (const polygon_work::ExPolygon &section : sections) {
        throw_if_cancelled(run_ctx);
        if (section.contains(polygon_work::Point(direct.front()), false) ||
            section.contains(polygon_work::Point(direct.back()), false))
            exempt = true;
        else
            obstacles.push_back(section);
    }
    counts.exemptions += exempt;
    if (obstacles.empty()) return direct;
    const auto stored = clipper::to_stored_expolygons(run_ctx->plugin_storage, obstacles);
    const auto interior = clipper_offset_ex(run_ctx->plugin_storage, stored.readonly(), -double(SCALED_EPSILON));
    StoredPolyline line(run_ctx->plugin_storage);
    for (const c_point point : direct) line.push_back(point);
    if (clipper_intersection_polyline_expolygons(run_ctx->plugin_storage, line.readonly(), interior.readonly()).empty())
        return direct;
    const ClipperOperand rings(run_ctx->plugin_storage, stored.readonly());
    const auto candidate = clipper_detour_polyline_expolygons(
        run_ctx->plugin_storage, line.readonly(), rings, CLIPPER_DETOUR_SHORTEST);
    throw_if_cancelled(run_ctx);
    if (!clipper_intersection_polyline_expolygons(run_ctx->plugin_storage, candidate.readonly(), interior.readonly()).empty()) {
        ++counts.fallbacks;
        return direct;
    }
    std::vector<c_point> result;
    for (uint32_t i = 0; i < candidate.size(); ++i) result.push_back(candidate.at(i));
    return result;
}

std::string warning_message(const uint64_t exemptions, const uint64_t fallbacks)
{
    if (exemptions == 0 && fallbacks == 0) return {};
    return "NoTravel best-effort: " + std::to_string(exemptions) +
        " travels exempted endpoint-containing components; " + std::to_string(fallbacks) +
        " inadmissible detours kept their straight path. NoTravel is not a strict barrier.";
}

}}} // namespace slic3r_api::LayerExtrusionEdit::NoTravel
