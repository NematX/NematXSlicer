///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ Copyright (c) Prusa Research 2016 - 2021 Vojtech Bubnik @bubnikv
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
Line infill pattern
===================

Standalone port of the former FillLine. The pattern draws parallel lines;
every odd line is tilted by the "oscillation" (line spacing minus extrusion
spacing), so consecutive lines form a zig-zag that can be connected into one
continuous path.

How one surface is filled:
1. line_spacing_for() derives the extrusion spacing, the line spacing from the
   density and the connection thresholds. A full, adjustable surface stretches
   the line spacing (at most +20%) to fit the object width.
2. The surface goes through Fill::fill_surface()'s half-spacing shrink. FillLine
   never initialized that spacing, so it is zero unless the line spacing was
   stretched; the island inset of step 3 does the real work.
3. fill_component() insets each island by 0.7 spacing (kept only when it stays
   one island) and works in a frame rotated so the lines are vertical: it
   generates the lines, clips them against the island, extends
   their ends, chains them and joins consecutive lines when the joining segment
   stays inside the island (grown by half a spacing).
4. publish_lines() simplifies the paths and publishes one sortable group.

The geometry follows FillLine exactly, with one intentional difference: with
infill_connection = "not connected" the legacy filler produced no line at all;
here the chained lines are emitted without being joined.
*/

#include "SimpleInfillPatterns.hpp"

#include <cassert>
#include <cmath>
#include <utility>

#include "libslic3r/Api/plugin/c/slic3r_def.h"
#include "libslic3r/Api/plugin/c/slic3r_geometry.h"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_infill.h"
#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"
#include "libslic3r/Plugins/Infill/InfillPaths.hpp"
#include "libslic3r/Plugins/Common/WorkBuffers.hpp"
#include "libslic3r/Plugins/Infill/PatternPluginHelpers.hpp"

namespace slic3r_api { namespace Infill {
namespace work = PluginCommon::work;
namespace {

const char *k_id = "line";
const char *k_no_dependencies[] = { nullptr };
// Legacy INFILL_OVERLAP_OVER_SPACING: how far a line end is extended past the island.
const float k_overlap_over_spacing = 0.3f;
const double k_half_pi = 1.57079632679489661923;

// Spacings of one surface, all scaled except fill_spacing.
struct LineSpacing
{
    // Extrusion spacing, the distance between two touching lines.
    coord_t min_spacing = 0;
    // Distance between two consecutive lines.
    coord_t line_spacing = 0;
    // Maximum Y distance of two line ends that may still be joined.
    coord_t diagonal_distance = 0;
    // X offset of the ends of odd lines.
    coord_t line_oscillation = 0;
    // Unscaled half-spacing shrink base: zero, or the stretched spacing.
    double fill_spacing = 0.;
    // True when the line spacing was stretched to the object width.
    bool adjusted = false;
};

// Rotation of the pattern and reference point of its grid.
struct LineDirection
{
    float angle = 0.f;
    work::Point reference;
};

// Derive every spacing of the surface from the recipe, like FillLine::init_spacing().
LineSpacing line_spacing_for(const raw_infill_pattern_params &params);
// Port of Fill::_infill_direction(): bridge angle or alternating layer angle,
// plus the quarter turn, around the object center.
LineDirection line_direction(const raw_infill_pattern_params &params, const ExPolygon &surface);
// Fill one shrunk island and append its paths, in the surface frame, to out.
void fill_component(storage_handle *storage, const ExPolygon &component, const LineSpacing &spacing,
                    const LineDirection &direction, const raw_infill_pattern_params &params,
                    const plugin_run_context *run_ctx, work::Polylines &out);
// Rotate the contour and holes of one stored expolygon around the origin.
void rotate_expolygon(expolygon_handle *expolygon, double angle);
// Build the vertical lines of the pattern over the rotated bounding box.
StoredPolylineCollection make_lines(storage_handle *storage, const work::BoundingBox &bounding_box,
                                    const LineSpacing &spacing, const plugin_run_context *run_ctx);
// Join consecutive chained lines when the connecting segment is short and inside boundary.
work::Polylines connect_lines(storage_handle *storage, work::Polylines chained, const LineSpacing &spacing,
                              const ExPolygonCollection &boundary, const plugin_run_context *run_ctx);
// Publish the paths into the pattern output as one sortable group.
void publish_lines(const run_ctx_infill_pattern &ctx, storage_handle *storage, const work::Polylines &lines,
                   const raw_infill_pattern_params &params, const plugin_run_context *run_ctx);
class LinePattern;
// Single plugin instance registered with the orchestrator.
LinePattern &instance(orchestrator_handle *orch);

LineSpacing line_spacing_for(const raw_infill_pattern_params &params)
{
    LineSpacing spacing;
    spacing.min_spacing = scale_i(params.spacing);
    assert(params.density > 0.0001f && params.density <= 1.f);
    spacing.line_spacing = coord_t(coordf_t(spacing.min_spacing) / params.density);
    spacing.diagonal_distance = spacing.line_spacing * 2;
    spacing.line_oscillation = spacing.line_spacing - spacing.min_spacing;
    // FillLine::init_spacing() never set Fill::spacing_priv, so the surface was
    // not shrunk by half a spacing; only the stretched spacing below shrinks it.
    spacing.fill_spacing = 0.;

    // A full surface may stretch its spacing to fill the object width evenly.
    if (params.density > 0.9999f && (params.infill_flags & C_INFILL_FLAG_DONT_ADJUST) == 0) {
        const coord_t width = params.object_bounding_box.max.x - params.object_bounding_box.min.x;
        spacing.line_spacing = PatternHelpers::adjust_solid_spacing(width, spacing.line_spacing, 1.2);
        spacing.fill_spacing = unscaled(spacing.line_spacing);
        spacing.adjusted = true;
    }
    return spacing;
}

LineDirection line_direction(const raw_infill_pattern_params &params, const ExPolygon &surface)
{
    LineDirection direction;
    direction.angle = float(params.angle);
    const c_bounding_box &object = params.object_bounding_box;
    // The object box is only undefined in unit tests; then the surface center is used.
    if (object.min.x < object.max.x && object.min.y < object.max.y) {
        direction.reference = work::Point((object.min.x + object.max.x) / 2, (object.min.y + object.max.y) / 2);
    } else {
        const c_bounding_box contour = surface.contour().bounding_box();
        direction.reference = work::Point((contour.min.x + contour.max.x) / 2, (contour.min.y + contour.max.y) / 2);
    }

    if (params.bridge_angle >= 0) {
        direction.angle = float(params.bridge_angle);
    } else if ((params.infill_flags & C_INFILL_FLAG_CAN_ANGLE_CROSS) != 0 && (params.layer_id & 1) != 0) {
        // The default layer angle alternates by a quarter turn on odd layers.
        direction.angle += float(k_half_pi);
    }
    direction.angle += float(k_half_pi);
    return direction;
}

void fill_component(storage_handle *storage, const ExPolygon &component, const LineSpacing &spacing,
                    const LineDirection &direction, const raw_infill_pattern_params &params,
                    const plugin_run_context *run_ctx, work::Polylines &out)
{
    // Legacy quick hack: move the island inward, but only when it stays one island.
    StoredExPolygonCollection island(storage);
    {
        ClipperOperand subject(storage, component);
        StoredExPolygonCollection shrunk =
            clipper_offset(storage, subject, -spacing.min_spacing * (1. - 0.3)).to_expolygon_collection();
        if (shrunk.size() == 1)
            island = std::move(shrunk);
        else
            island.push_back(component);
    }

    // Work in a frame where the lines are vertical.
    rotate_expolygon(expolygons_at(island.mutable_handle(), 0), -direction.angle);
    const ExPolygon rotated = island.readonly().at(0);
    work::BoundingBox bounding_box;
    for (const c_point point : rotated.contour().points())
        bounding_box.merge(point);
    if (!(params.density > 0.9999f && spacing.adjusted)) {
        // Align the pattern with the other layers through the rotated reference point.
        bounding_box.merge(work::align_to_grid(bounding_box.min, work::Point(spacing.line_spacing, spacing.line_spacing),
                                               direction.reference.rotated(-direction.angle)));
    }

    // Clip against a slightly larger island so vertical island sides keep their lines.
    const StoredPolylineCollection lines = make_lines(storage, bounding_box, spacing, run_ctx);
    ClipperOperand rotated_subject(storage, rotated);
    const StoredExPolygonCollection grown = clipper_offset(storage, rotated_subject, scale_d(0.02)).to_expolygon_collection();
    throw_if_cancelled(run_ctx);
    const StoredPolylineCollection clipped = clipper_intersection_polylines_expolygons(storage, lines, grown);
    work::Polylines polylines = Paths::read_polylines(clipped, run_ctx);

    // Extend both ends of every line outside the island by the legacy overlap.
    const coord_t extra = coord_t(std::floor(spacing.min_spacing * k_overlap_over_spacing + 0.5f));
    for (work::Polyline &polyline : polylines) {
        work::Point *first_point = &polyline.points.front();
        work::Point *last_point = &polyline.points.back();
        if (first_point->y() > last_point->y())
            std::swap(first_point, last_point);
        first_point->y() -= extra;
        last_point->y() += extra;
    }
    if (polylines.empty())
        return;

    work::Polylines chained = Paths::chain_polylines(std::move(polylines), run_ctx);
    work::Polylines component_lines;
    if (params.connection == C_INFILL_CONNECTION_NOT_CONNECTED) {
        component_lines = std::move(chained);
    } else {
        ClipperOperand island_subject(storage, rotated);
        const StoredExPolygonCollection boundary =
            clipper_offset(storage, island_subject, coordf_t(spacing.min_spacing / 2)).to_expolygon_collection();
        // Growing one island can only give one island.
        assert(boundary.size() <= 1);
        component_lines = connect_lines(storage, std::move(chained), spacing, boundary.readonly(), run_ctx);
    }

    // Back to the surface frame.
    for (work::Polyline &polyline : component_lines)
        polyline.rotate(direction.angle);
    work::ensure_valid(component_lines, params.fill_resolution);
    for (work::Polyline &polyline : component_lines)
        out.push_back(std::move(polyline));
}

void rotate_expolygon(expolygon_handle *expolygon, double angle)
{
    multipoint_rotate_origin(polygon_as_multipoint(expolygon_contour(expolygon)), angle);
    for (uint32_t index = 0; index < expolygon_hole_size(expolygon); ++index)
        multipoint_rotate_origin(polygon_as_multipoint(expolygon_hole_at(expolygon, index)), angle);
}

StoredPolylineCollection make_lines(storage_handle *storage, const work::BoundingBox &bounding_box,
                                    const LineSpacing &spacing, const plugin_run_context *run_ctx)
{
    StoredPolylineCollection lines(storage);
    const coord_t x_max = bounding_box.max.x() + SCALED_EPSILON;
    uint32_t index = 0;
    for (coord_t x = bounding_box.min.x(); x <= x_max; x += spacing.line_spacing, ++index) {
        if ((index & 1023u) == 0)
            throw_if_cancelled(run_ctx);
        // Odd lines are tilted so that consecutive lines meet in a zig-zag.
        const coord_t oscillation = (index & 1) != 0 ? spacing.line_oscillation : 0;
        StoredPolyline line(storage);
        line.push_back(c_point{ x - oscillation, bounding_box.min.y() });
        line.push_back(c_point{ x + oscillation, bounding_box.max.y() });
        lines.push_back_move(std::move(line));
    }
    return lines;
}

work::Polylines connect_lines(storage_handle *storage, work::Polylines chained, const LineSpacing &spacing,
                              const ExPolygonCollection &boundary, const plugin_run_context *run_ctx)
{
    const coord_t tolerance = coord_t(10 * SCALED_EPSILON);
    work::Polylines out;
    for (work::Polyline &polyline : chained) {
        if (!out.empty()) {
            work::Points &points_end = out.back().points;
            const work::Point &first_point = polyline.points.front();
            const work::Point &last_point = points_end.back();
            const coord_t distance_x = std::abs(last_point.x() - first_point.x());
            const coord_t distance_y = std::abs(last_point.y() - first_point.y());
            // Only neighbouring lines, close enough in Y, may be joined.
            const bool can_connect =
                distance_x >= (spacing.line_spacing - spacing.line_oscillation) - tolerance &&
                distance_x <= (spacing.line_spacing + spacing.line_oscillation) + tolerance &&
                distance_y <= spacing.diagonal_distance;
            if (can_connect && !boundary.empty()) {
                // Each candidate costs one Clipper call.
                throw_if_cancelled(run_ctx);
                StoredPolyline segment(storage);
                segment.push_back(c_point{ last_point.x(), last_point.y() });
                segment.push_back(c_point{ first_point.x(), first_point.y() });
                // The joining segment must stay inside the grown island (ExPolygon::contains(Line)).
                if (clipper_diff_polyline_expolygons(storage, segment, boundary).empty()) {
                    points_end.insert(points_end.end(), polyline.points.begin(), polyline.points.end());
                    continue;
                }
            }
        }
        out.push_back(std::move(polyline));
    }
    return out;
}

void publish_lines(const run_ctx_infill_pattern &ctx, storage_handle *storage, const work::Polylines &lines,
                   const raw_infill_pattern_params &params, const plugin_run_context *run_ctx)
{
    StoredExtrusionEntity group(storage);
    for (size_t index = 0; index < lines.size(); ++index) {
        if ((index & 1023u) == 0)
            throw_if_cancelled(run_ctx);
        const work::Polyline &line = lines[index];
        StoredExtrusionEntity path = PatternHelpers::make_path(storage, line.points, params);
        group.append_child_move(path.mutable_view());
    }
    if (group.empty())
        return;
    group.set_flags(RAW_EXTRUSION_FLAG_SORTABLE | RAW_EXTRUSION_FLAG_REVERSIBLE);
    throw_if_cancelled(run_ctx);
    MutableExtrusionEntity(ctx.output).append_child_move(group.mutable_view());
}

class LinePattern : public PluginBase
{
public:
    explicit LinePattern(orchestrator_handle *orch) : PluginBase(orch) {}

private:
    const char *id_impl() const noexcept override { return k_id; }
    const char *name_impl() const noexcept override { return "Line"; }
    const char *description_impl() const noexcept override { return "Simple alternating line infill pattern."; }
    slicing_step_t step_impl() const noexcept override { return INFILL_PATTERN; }
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }
    int32_t priority_impl() const noexcept override { return 7; }
    const char *progress_message_format_impl() const noexcept override
    {
        return "Generating line pattern: %u / %u surfaces";
    }

    void inilialize_impl(storage_handle *) const override
    {
        PatternHelpers::add_self_to_pattern_list(m_orchestrator, k_id, "Line", { "fill_pattern" });
    }

    // Shrink the surface, fill each island and publish the paths.
    void run_impl(const plugin_run_context *run_ctx) const override
    {
        throw_if_cancelled(run_ctx);
        const run_ctx_infill_pattern *ctx = plugin_ctx_as_infill_pattern(run_ctx);
        if (ctx == nullptr || ctx->surface == nullptr || ctx->params == nullptr || ctx->output == nullptr ||
            run_ctx == nullptr || run_ctx->plugin_storage == nullptr)
            return;
        fill_line_surface(*ctx, run_ctx->plugin_storage, run_ctx);
        progress().increment();
    }
};

LinePattern &instance(orchestrator_handle *orch)
{
    static LinePattern s_instance(orch);
    return s_instance;
}

} // namespace

void fill_line_surface(const run_ctx_infill_pattern &ctx, storage_handle *storage, const plugin_run_context *run_ctx)
{
    const raw_infill_pattern_params &params = *ctx.params;
    if (params.spacing <= 0. || params.density <= 0.f)
        return;

    const ExPolygon surface = Surface(ctx.surface).expolygon();
    const LineSpacing spacing = line_spacing_for(params);
    const LineDirection direction = line_direction(params, surface);
    StoredExPolygonCollection islands = PatternHelpers::shrink_for_fill(storage, surface, spacing.fill_spacing);
    islands.ensure_valid();

    work::Polylines lines;
    for (const ExPolygon island : islands) {
        throw_if_cancelled(run_ctx);
        fill_component(storage, island, spacing, direction, params, run_ctx, lines);
    }
    publish_lines(ctx, storage, lines, params, run_ctx);
}

void register_line_pattern_plugin(orchestrator_handle *orch)
{
    orchestrator_register_plugin(orch, instance(orch).c_instance());
}

}} // namespace slic3r_api::Infill
