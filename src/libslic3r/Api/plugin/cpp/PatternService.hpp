///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_plugin_cpp_PatternService_hpp_
#define slic3r_Api_plugin_cpp_PatternService_hpp_

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "libslic3r/Api/plugin/c/steps/slic3r_step_support.h"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"

namespace slic3r_api {

/*
Sampling the INFILL_PATTERN plugins from STEP_SUPPORT
=====================================================

A support family that wants the lines of a fill pattern, for example to place the
tips of a tree, calls the pattern callbacks of run_ctx_generate_support (ABI 1.2).
The raw callbacks need a recipe (raw_infill_pattern_params), a surface, an output
entity and the flattening of the result. PatternService does these four steps:

    const slic3r_api::PatternService patterns(plugin_ctx_as_generate_support(context));
    const infill_pattern_runtime_id id = patterns.resolve("rectilinear");
    PatternService::Fill fill;
    fill.flow = params.base_flow;
    fill.spacing = params.base_flow_mm.spacing;
    fill.density = 0.25f;
    const raw_infill_pattern_params recipe = PatternService::recipe(id, fill);
    for (const std::vector<c_point> &line : patterns.fill_lines(storage, expolygon, recipe)) ...

Only the patterns that do not read the layer, island and region (rectilinear,
supportbase and honeycomb) can be sampled this way: at STEP_SUPPORT the support layers do
not exist yet, so the service passes NULL for them (see slic3r_step_support.h).

Failure: a call that the host reports as failed returns 0 and gives no line, as an empty fill; an unknown runtime id is no explicit
failure, it is such a call. A real error inside a pattern is fatal for the step (see StepGenerateSupport.cpp): the host
records an error message and requests the cancellation, the call still returns 0, and the step stops at the end of the phase, once the
family returns. A family should therefore call throw_if_cancelled() after each call. The legacy code ignored an InfillFailedException
part by part: that is a known limit of the equivalence with the native filler.

Concurrency: the callbacks may be called from several threads at once; PatternService
holds no state beyond the payload pointer, so one instance can be shared by the
worker threads of a family. The payload must outlive the instance.

C++ only: no C function is added.
*/
class PatternService
{
public:
    // What changes between two fills; the other fields of the recipe are the ones of the legacy support.
    struct Fill
    {
        // Support flow of the lines (the flow of the base for a tree).
        c_flow flow = {};
        // Spacing of the flow, in millimeters.
        double spacing = 0.;
        // Line density in (0, 1].
        float density = 0.f;
        // Direction of the lines, in radians.
        float angle = 0.f;
        // Object layer, from which the patterns alternate their direction.
        uint32_t layer_id = 0;
    };

    // The payload received by the family. Throws std::runtime_error when it is null or has no pattern callback (a host older
    // than slic3r_step_support.h 1.2 leaves them NULL).
    explicit PatternService(const run_ctx_generate_support *payload) : m_payload(payload)
    {
        if (payload == nullptr || payload->resolve_pattern_id == nullptr || payload->generate_pattern == nullptr)
            throw std::runtime_error("The support step does not provide the infill pattern callbacks.");
    }

    // The runtime id of a pattern plugin id such as "rectilinear" (INFILL_PATTERN_RUNTIME_ID_INVALID when no pattern is
    // active). The first call of the step builds the pattern service of the host, which runs the setup() of every pattern.
    infill_pattern_runtime_id resolve(const std::string &pattern_plugin_id) const
    {
        return m_payload->resolve_pattern_id(m_payload, pattern_plugin_id.c_str());
    }

    // The recipe of a sparse support fill: an internal sparse surface, DONT_ADJUST (the legacy support never stretches the
    // spacing) and CAN_ANGLE_CROSS (the legacy Fill keeps its default, so a rectilinear fill turns a quarter turn on every other
    // layer_id), a flow multiplier of 1, no bridge, 1000 mm anchors and a 0.0125 mm resolution. The object bounding box is empty,
    // so rectilinear and supportbase anchor their lines on the centre of the contour they fill (RectilinearFills.hpp, as the native
    // FillBase.cpp), like the legacy filler that did not receive a bounding box; the alignment of honeycomb to the world is separate.
    static raw_infill_pattern_params recipe(infill_pattern_runtime_id pattern_id, const Fill &fill)
    {
        raw_infill_pattern_params recipe = {};
        recipe.pattern_id = pattern_id;
        recipe.surface_type = raw_surface_type(RAW_SURFACE_TYPE_POS_INTERNAL | RAW_SURFACE_TYPE_DENS_SPARSE);
        recipe.extrusion_role = RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL;
        recipe.flow = fill.flow;
        recipe.density = fill.density;
        recipe.flow_mult = 1.f;
        recipe.connection = C_INFILL_CONNECTION_CONNECTED;
        recipe.infill_flags = C_INFILL_FLAG_DONT_ADJUST | C_INFILL_FLAG_CAN_ANGLE_CROSS;
        recipe.spacing = fill.spacing;
        recipe.angle = fill.angle;
        recipe.bridge_angle = -1.;
        recipe.layer_height = unscaled(fill.flow.height);
        recipe.layer_id = fill.layer_id;
        recipe.bridge_offset = -1;
        recipe.fill_resolution = scale_i(0.0125);
        recipe.anchor_length = 1000.f;
        recipe.anchor_length_max = 1000.f;
        return recipe;
    }

    // Fills one area with the recipe and returns the points of its lines, in the order the pattern generated them (the widths
    // and the connections the pattern marked are dropped). The temporary surface and output live in storage and are freed
    // before the return. An empty result is either a fill that produced nothing or a failed call (see the header comment).
    std::vector<std::vector<c_point>> fill_lines(storage_handle *storage, const ExPolygon &area, const raw_infill_pattern_params &recipe) const
    {
        StoredSurfaceCollection surface(storage);
        surface.append(area, recipe.surface_type);
        StoredExtrusionEntity output(storage);
        std::vector<std::vector<c_point>> lines;
        const int32_t ran = m_payload->generate_pattern(m_payload, recipe.pattern_id, nullptr, nullptr, nullptr, nullptr,
                                                        surface_collection_at(surface.handle(), 0), nullptr, &recipe,
                                                        output.mutable_handle());
        if (ran != 0)
            collect_leaf_polylines(output.readonly(), lines);
        return lines;
    }

private:
    // Appends the polyline of every entity of the tree that carries one, in pre-order: the polyline of an entity, then its children in order.
    // This is a copy of the helper in the anonymous namespace of Plugins/Support/Common/PerimeterFill.cpp, because Api/plugin/cpp must not
    // depend on Plugins/Support.
    static void collect_leaf_polylines(const ExtrusionEntity &entity, std::vector<std::vector<c_point>> &lines)
    {
        if (entity.has_polyline())
            lines.push_back(entity.points());
        for (uint32_t idx = 0; idx < entity.child_count(); ++idx)
            collect_leaf_polylines(entity.child(idx), lines);
    }

    const run_ctx_generate_support *m_payload;
};

} // namespace slic3r_api

#endif // slic3r_Api_plugin_cpp_PatternService_hpp_
