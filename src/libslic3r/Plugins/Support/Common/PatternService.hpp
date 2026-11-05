///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef plugins_support_common_pattern_service_hpp_
#define plugins_support_common_pattern_service_hpp_

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "libslic3r/Api/plugin/c/steps/slic3r_step_support.h"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"
#include "libslic3r/Plugins/Support/Common/ExtrusionLines.hpp"

namespace slic3r_api::Support::Common {

/*
Sampling the INFILL_PATTERN plugins from STEP_SUPPORT
=====================================================

A support family that wants the lines of a fill pattern, for example to place the
tips of a tree, calls the pattern callbacks of run_ctx_generate_support (ABI 1.2).
The raw callbacks need a recipe (raw_infill_pattern_params), a surface, an output
entity and the flattening of the result. Common::fill_recipe (include SupportPaths.hpp directly) builds the recipe;
PatternService resolves its pattern and fills a temporary surface, returning flattened lines:

    const slic3r_api::Support::Common::PatternService patterns(plugin_ctx_as_generate_support(context));
    const infill_pattern_runtime_id id = patterns.resolve("rectilinear");
    const raw_infill_pattern_params recipe = fill_recipe(id, RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL,
        params.base_flow, params.base_flow_mm.spacing, 0.25f, params.base_angle);
    for (const std::vector<c_point> &line : patterns.fill_lines(storage, expolygon, recipe)) ...

Only the patterns that do not read the layer, island and region (rectilinear,
supportbase and honeycomb) can be sampled this way: at STEP_SUPPORT the support layers do
not exist yet, so the service passes NULL for them (see slic3r_step_support.h).

Failure: a call that the host reports as failed returns 0 and gives no line, as an empty fill; an unknown runtime id is no explicit
failure, it is such a call. A real error inside a pattern is fatal for the step (see StepGenerateSupport.cpp): the host
records an error message and requests the cancellation, the call still returns 0, and the step stops at the end of the phase, once the
family returns. A family should therefore call throw_if_cancelled() after each call. The native filler ignores an InfillFailedException
part by part: this limits equivalence with the native filler.

Concurrency: the callbacks may be called from several threads at once; PatternService
holds no state beyond the payload pointer, so one instance can be shared by the
worker threads of a family. The payload must outlive the instance.

*/
class PatternService
{
public:
    // The payload received by the family. Throws std::runtime_error when it is null (plugin_ctx_as_generate_support returns NULL in
    // the context of another step) or has no pattern callback. A plugin that requires slic3r_step_support.h 1.2 is not loaded by an
    // older host, whose payload has no such fields, so the callback checks reject a payload built by hand (the tests).
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
    const run_ctx_generate_support *m_payload;
};

} // namespace slic3r_api::Support::Common

#endif // plugins_support_common_pattern_service_hpp_
