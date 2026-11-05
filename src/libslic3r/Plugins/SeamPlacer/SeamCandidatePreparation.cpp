///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
Shared seam candidate preparation
=================================
This file prepares only material that is independent of a seam provider's
selection policy. It walks extrusion trees, generates geometric and painted-
constraint candidates, computes the legacy layer attributes, applies the
ordered filtering policy, and projects a target onto line or arc geometry.

The Cost and Aligned providers own their session state, graph traversal, and
scoring. They borrow these value-oriented helpers during initialization and
publish their own immutable results afterward.
Handles are borrowed: the collector neither owns nor mutates host-owned
extrusion trees.
*/

#include "SeamCandidatePreparation.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <utility>

#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/LineDistancer.hpp"
#include "libslic3r/Api/plugin/cpp/SeamPlacerViews.hpp"
#include "libslic3r/Api/plugin/cpp/properties/ExtrusionProperties.hpp"
#include "libslic3r/Plugins/Ordering/ExtrusionTreeOrderingGeometry.hpp"
#include "SeamCostModel.hpp"
#include "SeamPlacementConstraints.hpp"
#include "SeamInitializationCancellation.hpp"

namespace slic3r_api { namespace SeamPlacement { namespace CandidatePreparation {

namespace {

struct ExternalGeometry {
    bool present = false;
    double width = 0;
};

struct TreeSummary {
    bool has_perimeter_marker = false;
    bool has_perimeter_role = false;
    ExternalGeometry external;
};

/* Return scaled XY distance; callers decide whether the result is in mm. */
double point_distance(c_point a, c_point b);

/* Apply attributes inherited from a parent extrusion node. */
void update_attributes(const ExtrusionEntity &entity, raw_extrusion_role &role,
                       double &width);

/* Inherit roles downward, summarize only segment-bearing perimeter geometry,
   and collect borrowed atomic loop handles with local SKIRT flags taking priority. */
TreeSummary collect_tree(const ExtrusionEntity &entity,
                         raw_extrusion_role role, double width,
                         bool perimeter,
                         std::vector<CollectedLoop> &loops,
                         const plugin_run_context *run_context,
                         bool ancestors_allow_seams = true);

/* Find the LayerIsland containing a source LayerRegionIsland. */
LayerIsland parent_island(const Layer &layer,
                          const LayerRegionIsland &region_island);

/* Test whether a point lies on or inside a neighboring region surface. */
bool is_boundary_point(const LayerRegionIsland &source, c_point point,
                       double tolerance);

/* Copy neighboring region surfaces into an offset boundary zone. */
StoredExPolygonCollection boundary_zone(storage_handle *storage,
                                        const LayerRegionIsland &source,
                                        double tolerance);

/* Check membership in any copied boundary polygon. */
bool inside_boundary_zone(const StoredExPolygonCollection &zone,
                          c_point point);

/* Add candidates where a loop crosses the neighboring-region boundary zone. */
void add_boundary_candidates(
    storage_handle *storage, const SeamPlacementConstraints &constraints,
    const LayerRegionIsland &island, const ExtrusionEntity &loop,
    c_point shift, double width, std::vector<Candidate> &out,
    const plugin_run_context *run_context);

/* Add the endpoints of every painted-constraint portion. */
void add_constraint_candidates(const SeamPlacementConstraints &constraints,
                                const ExtrusionEntity &loop,
                                const LayerRegion &region, c_point shift,
                                double width, std::vector<Candidate> &out,
                                const plugin_run_context *run_context);

/* Project a target onto every real segment in an extrusion tree. */
std::pair<c_point, double> project_onto_loop_impl(const ExtrusionEntity &entity,
                                                  c_point target);

} // namespace

/* Collect every non-empty perimeter loop, classify its effective role, and
   retain the external width when its subtree supplies candidate geometry. */
std::vector<CollectedLoop> collect_perimeter_loops(
    const ExtrusionEntity &root, raw_extrusion_role inherited_role,
    const plugin_run_context *run_context);

/* Build the loop's original vertices, painted-constraint endpoints, and
   neighboring-region transitions in discovery order. Coordinates are stored in
   object-local scaled units; widths and the legacy flow attributes are mm. */
std::vector<Candidate> prepare_loop_candidates(
    storage_handle *storage, const SeamPlacementConstraints &constraints,
    const LayerRegionIsland &island, const ExtrusionEntity &loop,
    const LayerRegion &region, c_point shift, double width, double nozzle,
    const plugin_run_context *run_context);

/* Enrich candidates with legacy overhang from the previous layer, embedding
   from the current layer, and neighboring-region boundary membership. */
void calculate_layer_attributes(const LayerRegionIsland &island,
                                std::vector<Candidate> &candidates,
                                const plugin_run_context *run_context);

/* Apply the common ordered blocker, enforcer, boundary, support, and embedding
   policy while preserving discovery order. The all-blocked fallback is kept so
   a provider always retains a usable source candidate. */
void filter_candidates(std::vector<Candidate> &candidates,
                       const plugin_run_context *run_context);

/* Project a scaled platter-coordinate target onto line and arc geometry. */
std::pair<c_point, double> project_onto_loop(const ExtrusionEntity &entity,
                                             c_point target);

namespace {

double point_distance(c_point a, c_point b)
{
    return std::hypot(double(a.x) - b.x, double(a.y) - b.y);
}

void update_attributes(const ExtrusionEntity &entity, raw_extrusion_role &role,
                       double &width)
{
    const c_extrusion_property_attributes *attributes = static_cast<
        const c_extrusion_property_attributes *>(
            entity.property_data(EXTRUSION_PROPERTY_TYPE_ATTRIBUTES));
    if (attributes == nullptr) return;
    role = attributes->role;
    if (attributes->width > 0) width = attributes->width;
}

/* Traverse borrowed nodes without changing them: roles and widths descend, but
   only nodes with segments prove perimeter roles. A collected loop's own SKIRT
   marker forces distance-only placement; nested records collapse to its borrowed
   identity. The caller owns the output vector; cancellation propagates. */
TreeSummary collect_tree(const ExtrusionEntity &entity,
                         raw_extrusion_role role, double width,
                         bool perimeter,
                         std::vector<CollectedLoop> &loops,
                         const plugin_run_context *run_context,
                         const bool ancestors_allow_seams)
{
    InitializationCancellation::check(run_context);
    update_attributes(entity, role, width);
    const c_extrusion_property_perimeter *perimeter_marker = static_cast<
        const c_extrusion_property_perimeter *>(
            entity.property_data(EXTRUSION_PROPERTY_TYPE_PERIMETER));
    if (perimeter_marker != nullptr)
        perimeter = true;
    TreeSummary summary;
    summary.has_perimeter_marker = perimeter;
    summary.has_perimeter_role =
        perimeter && entity.segment_count() != 0 && RAW_EXTRUSION_ROLE_IS_PERIMETER(role);
    if (perimeter && entity.segment_count() != 0 &&
        RAW_EXTRUSION_ROLE_IS_EXTERNAL_PERIMETER(role)) {
        summary.external.present = true;
        summary.external.width = width;
    }
    const size_t first_nested_loop = loops.size();
    const bool seam_unit = ancestors_allow_seams && entity.is_seam_unit();
    for (uint32_t index = 0; index < entity.child_count(); ++index) {
        const TreeSummary child = collect_tree(
            entity.child(index), role, width, perimeter, loops, run_context,
            !seam_unit && Ordering::TreeOrderingGeometry::can_choose_child_seam(
                entity, entity.child(index), ancestors_allow_seams));
        summary.has_perimeter_marker =
            summary.has_perimeter_marker || child.has_perimeter_marker;
        summary.has_perimeter_role =
            summary.has_perimeter_role || child.has_perimeter_role;
        if (!summary.external.present && child.external.present)
            summary.external = child.external;
    }

    /* Ordering and seam placement operate on the continuous closed parent, even
       when Arachne stores perimeter properties on its open child paths. Keep one
       binding for that atomic identity and suppress records nested inside it. */
    if (seam_unit &&
        summary.has_perimeter_marker) {
        const bool skirt_marker = perimeter_marker != nullptr &&
            (perimeter_marker->perimeter_flags & C_EXTRUSION_PERIMETER_FLAG_SKIRT) != 0;
        loops.resize(first_nested_loop);
        loops.push_back({entity.handle(),
                         skirt_marker || !summary.has_perimeter_role,
                         summary.external.present,
                         summary.external.width});
    }
    return summary;
}

} // namespace

std::vector<CollectedLoop> collect_perimeter_loops(
    const ExtrusionEntity &root, raw_extrusion_role inherited_role,
    const plugin_run_context *run_context)
{
    std::vector<CollectedLoop> loops;
    collect_tree(root, inherited_role, 0, false, loops, run_context);
    return loops;
}

namespace {

LayerIsland parent_island(const Layer &layer,
                          const LayerRegionIsland &region_island)
{
    for (uint32_t island_index = 0; island_index < layer.island_count();
         ++island_index) {
        const LayerIsland candidate = layer.island(island_index);
        for (uint32_t region_island_index = 0;
             region_island_index < candidate.region_island_count();
             ++region_island_index) {
            const LayerRegionIsland child =
                candidate.region_island(region_island_index);
            if (child.handle() == region_island.handle()) return candidate;
        }
    }
    return LayerIsland();
}

bool is_boundary_point(const LayerRegionIsland &source, c_point point,
                       double tolerance)
{
    if (source.region_count() == 0) return false;
    const Layer layer = source.region(0).layer();
    const LayerIsland parent = parent_island(layer, source);
    if (!parent.valid()) return false;
    for (uint32_t island_index = 0; island_index < parent.region_island_count();
         ++island_index) {
        const LayerRegionIsland other = parent.region_island(island_index);
        if (other.handle() == source.handle()) continue;
        for (uint32_t region_index = 0; region_index < other.region_count();
             ++region_index) {
            const ExPolygonCollection slices =
                other.region(region_index).slices();
            const LineDistancer boundaries(slices);
            if (boundaries.distance_from_lines(point) <= tolerance ||
                boundaries.distance_from_lines(point, true) < 0)
                return true;
        }
    }
    return false;
}

StoredExPolygonCollection boundary_zone(storage_handle *storage,
                                        const LayerRegionIsland &source,
                                        double tolerance)
{
    StoredExPolygonCollection geometry(storage);
    if (source.region_count() == 0) return geometry;
    const Layer layer = source.region(0).layer();
    const LayerIsland parent = parent_island(layer, source);
    if (!parent.valid()) return geometry;
    const ClipperContext clip(storage);
    for (uint32_t island_index = 0; island_index < parent.region_island_count();
         ++island_index) {
        const LayerRegionIsland other = parent.region_island(island_index);
        if (other.handle() == source.handle()) continue;
        for (uint32_t region_index = 0; region_index < other.region_count();
             ++region_index) {
            const ExPolygonCollection slices =
                other.region(region_index).slices();
            for (const ExPolygon polygon : slices) {
                ClipperOperand subject = clip(polygon);
                ClipperOperand expanded = clipper_offset(subject, tolerance);
                StoredExPolygonCollection expanded_geometry =
                    expanded.to_expolygon_collection();
                geometry.append_move_from(std::move(expanded_geometry));
            }
        }
    }
    return geometry;
}

bool inside_boundary_zone(const StoredExPolygonCollection &zone,
                          c_point point)
{
    for (const ExPolygon polygon : zone)
        if (polygon.contains(point)) return true;
    return false;
}

void add_boundary_candidates(
    storage_handle *storage, const SeamPlacementConstraints &constraints,
    const LayerRegionIsland &island, const ExtrusionEntity &loop,
    c_point shift, double width, std::vector<Candidate> &out,
    const plugin_run_context *run_context)
{
    InitializationCancellation::check(run_context);
    const StoredExPolygonCollection zone =
        boundary_zone(storage, island, scale_d(width / 2));
    if (zone.empty()) return;
    std::function<void(const ExtrusionEntity &)> visit;
    visit = [&](const ExtrusionEntity &node) {
        for (uint32_t segment_index = 0;
             segment_index < node.segment_count(); ++segment_index) {
            InitializationCancellation::check_periodic(
                run_context, segment_index);
            c_extrusion_segment segment = node.segment(segment_index);
            segment.point_a.x -= shift.x;
            segment.point_a.y -= shift.y;
            segment.point_b.x -= shift.x;
            segment.point_b.y -= shift.y;
            const SeamPlacementConstraints::Curve curve =
                SeamPlacementConstraints::Curve::from_segment(segment);
            std::vector<double> cuts;
            for (const ExPolygon polygon : zone) {
                const std::function<void(const Polygon &)> intersect =
                    [&](const Polygon &edge_loop) {
                        for (size_t edge_index = 0;
                             edge_index < edge_loop.size(); ++edge_index) {
                            InitializationCancellation::check_periodic(
                                run_context, edge_index);
                            c_extrusion_segment edge{};
                            edge.point_a = edge_loop[edge_index];
                            edge.point_b =
                                edge_loop[(edge_index + 1) % edge_loop.size()];
                            curve.intersection_cuts(
                                SeamPlacementConstraints::Curve::from_segment(
                                    edge),
                                cuts);
                        }
                    };
                intersect(polygon.contour());
                for (uint32_t hole_index = 0;
                     hole_index < polygon.hole_size(); ++hole_index)
                    intersect(polygon.hole(hole_index));
            }
            std::sort(cuts.begin(), cuts.end());
            cuts.erase(std::unique(cuts.begin(), cuts.end(),
                                   [](double left, double right) {
                                       return std::abs(left - right) < 1e-12;
                                   }),
                       cuts.end());
            const double epsilon = curve.length() > 0 ?
                std::min(1e-4, 1.0 / curve.length()) : 0;
            size_t cut_index = 0;
            for (const double cut : cuts) {
                InitializationCancellation::check_periodic(
                    run_context, cut_index++);
                if (cut < 0 || cut > 1) continue;
                const bool before = inside_boundary_zone(
                    zone, curve.point(std::max(0.0, cut - epsilon)));
                const bool after = inside_boundary_zone(
                    zone, curve.point(std::min(1.0, cut + epsilon)));
                if (before == after) continue;
                Candidate candidate;
                candidate.point = curve.point(cut);
                candidate.width = width;
                candidate.boundary = true;
                const c_point platter = {
                    candidate.point.x + shift.x, candidate.point.y + shift.y};
                const SeamPlacementConstraints::Status status =
                    constraints.classify_point(loop, platter, scale_d(width));
                candidate.blocked =
                    status == SeamPlacementConstraints::Status::Blocked;
                candidate.enforced =
                    status == SeamPlacementConstraints::Status::Enforced;
                out.push_back(candidate);
            }
        }
        for (uint32_t child_index = 0; child_index < node.child_count();
             ++child_index) {
            InitializationCancellation::check_periodic(
                run_context, child_index);
            visit(node.child(child_index));
        }
    };
    visit(loop);
}

void add_constraint_candidates(const SeamPlacementConstraints &constraints,
                               const ExtrusionEntity &loop,
                               const LayerRegion &region, c_point shift,
                               double width, std::vector<Candidate> &out,
                               const plugin_run_context *run_context)
{
    InitializationCancellation::check(run_context);
    const SeamPlacementConstraints::Result portions =
        constraints.portions(loop, region, true);
    for (const SeamPlacementConstraints::Portion &portion :
         portions.portions) {
        for (const double parameter : {portion.begin, portion.end}) {
            const c_point platter = portion.curve.point(parameter);
            const SeamPlacementConstraints::Status status =
                constraints.classify_point(loop, platter, portion.paint_margin);
            Candidate candidate;
            candidate.point = {platter.x - shift.x, platter.y - shift.y};
            candidate.width = width;
            candidate.blocked =
                status == SeamPlacementConstraints::Status::Blocked;
            candidate.enforced =
                status == SeamPlacementConstraints::Status::Enforced;
            out.push_back(candidate);
        }
    }
}

} // namespace

std::vector<Candidate> prepare_loop_candidates(
    storage_handle *storage, const SeamPlacementConstraints &constraints,
    const LayerRegionIsland &island, const ExtrusionEntity &loop,
    const LayerRegion &region, c_point shift, double width, double nozzle,
    const plugin_run_context *run_context)
{
    InitializationCancellation::check(run_context);
    const std::vector<c_point> path = loop.collect_points();
    std::vector<c_point> points;
    points.reserve(path.size());
    for (const c_point point : path)
        if (points.empty() || points.back().x != point.x ||
            points.back().y != point.y)
            points.push_back(point);
    const std::vector<float> angles =
        Slic3r::SeamCostModel::calculate_cost_angles(points, float(nozzle));
    std::vector<Candidate> candidates;
    candidates.reserve(points.size());
    for (size_t point_index = 0; point_index < points.size(); ++point_index) {
        InitializationCancellation::check_periodic(run_context, point_index);
        Candidate candidate;
        const c_point point = points[point_index];
        candidate.point = {point.x - shift.x, point.y - shift.y};
        candidate.angle = angles[point_index];
        candidate.width = width;
        candidate.nozzle = nozzle;
        candidate.stable_order = candidates.size();
        const SeamPlacementConstraints::Status status =
            constraints.classify_point(loop, point, scale_d(width));
        candidate.blocked =
            status == SeamPlacementConstraints::Status::Blocked;
        candidate.enforced =
            status == SeamPlacementConstraints::Status::Enforced;
        candidates.push_back(candidate);
    }
    add_constraint_candidates(constraints, loop, region, shift, width,
                              candidates, run_context);
    add_boundary_candidates(storage, constraints, island, loop, shift, width,
                            candidates, run_context);
    for (size_t index = 0; index < candidates.size(); ++index) {
        candidates[index].nozzle = nozzle;
        candidates[index].stable_order = index;
    }
    return candidates;
}

void calculate_layer_attributes(const LayerRegionIsland &island,
                                std::vector<Candidate> &candidates,
                                const plugin_run_context *run_context)
{
    InitializationCancellation::check(run_context);
    if (island.region_count() == 0) return;
    const Layer layer = island.region(0).layer();
    const LineDistancer current(layer.slices());
    size_t islands_with_perimeters = 0;
    for (uint32_t island_index = 0; island_index < layer.island_count();
         ++island_index) {
        const LayerIsland layer_island = layer.island(island_index);
        for (uint32_t region_island_index = 0;
             region_island_index < layer_island.region_island_count();
             ++region_island_index) {
            const LayerRegionIsland region_island =
                layer_island.region_island(region_island_index);
            if (region_island.has_extrusion(
                    RAW_EXTRUSION_ROLE_PERIMETER) &&
                !ExtrusionEntity(region_island.extrusion(
                                    RAW_EXTRUSION_ROLE_PERIMETER)).empty())
                ++islands_with_perimeters;
        }
    }
    LineDistancer previous;
    bool has_previous = false;
    const Object object = layer.object();
    for (uint32_t index = 0; index < object.layer_count(); ++index) {
        if (object.layer(index).handle() != layer.handle()) continue;
        if (index != 0) {
            previous = LineDistancer(object.layer(index - 1).slices());
            has_previous = true;
        }
        break;
    }
    for (size_t candidate_index = 0;
         candidate_index < candidates.size(); ++candidate_index) {
        InitializationCancellation::check_periodic(
            run_context, candidate_index);
        Candidate &candidate = candidates[candidate_index];
        if (has_previous)
            candidate.overhang = std::max(
                0.0, unscaled(previous.distance_from_lines(
                                   candidate.point, true)));
        if (islands_with_perimeters > 1)
            candidate.embedded = unscaled(current.distance_from_lines(
                                               candidate.point, true)) +
                                 0.6 * candidate.width;
        candidate.boundary = candidate.boundary ||
            is_boundary_point(island, candidate.point,
                              scale_d(candidate.width / 2));
    }
}

void filter_candidates(std::vector<Candidate> &candidates,
                       const plugin_run_context *run_context)
{
    InitializationCancellation::check(run_context);
    const std::vector<Candidate> original = candidates;
    candidates.erase(std::remove_if(
        candidates.begin(), candidates.end(),
        [](const Candidate &candidate) { return candidate.blocked; }),
        candidates.end());
    InitializationCancellation::check(run_context);
    if (candidates.empty()) candidates = original;
    const bool enforced = std::any_of(
        candidates.begin(), candidates.end(),
        [](const Candidate &candidate) { return candidate.enforced; });
    InitializationCancellation::check(run_context);
    if (enforced)
        candidates.erase(std::remove_if(
            candidates.begin(), candidates.end(),
            [](const Candidate &candidate) { return !candidate.enforced; }),
            candidates.end());
    const bool boundary = std::any_of(
        candidates.begin(), candidates.end(),
        [](const Candidate &candidate) { return candidate.boundary; });
    InitializationCancellation::check(run_context);
    if (boundary)
        candidates.erase(std::remove_if(
            candidates.begin(), candidates.end(),
            [](const Candidate &candidate) { return !candidate.boundary; }),
            candidates.end());
    const bool supported = std::any_of(
        candidates.begin(), candidates.end(),
        [](const Candidate &candidate) { return candidate.overhang == 0; });
    InitializationCancellation::check(run_context);
    if (supported)
        candidates.erase(std::remove_if(
            candidates.begin(), candidates.end(),
            [](const Candidate &candidate) {
                return candidate.overhang > candidate.width / 4;
            }),
            candidates.end());
    const bool embedded = std::any_of(
        candidates.begin(), candidates.end(),
        [](const Candidate &candidate) { return candidate.embedded < -0.5; });
    if (embedded)
        candidates.erase(std::remove_if(
            candidates.begin(), candidates.end(),
            [](const Candidate &candidate) {
                return candidate.embedded > -0.5;
            }),
            candidates.end());
}

namespace {

std::pair<c_point, double> project_onto_loop_impl(const ExtrusionEntity &entity,
                                                  c_point target)
{
    c_point best{};
    double best_distance = std::numeric_limits<double>::infinity();
    std::function<void(const ExtrusionEntity &)> visit =
        [&](const ExtrusionEntity &node) {
            for (uint32_t index = 0; index < node.segment_count(); ++index) {
                const SeamPlacementConstraints::Curve curve =
                    SeamPlacementConstraints::Curve::from_segment(
                        node.segment(index));
                const c_point point =
                    curve.point(curve.closest_parameter(target));
                const double distance = point_distance(point, target);
                if (distance < best_distance) {
                    best = point;
                    best_distance = distance;
                }
            }
            if (node.segment_count() == 0 && node.point_count() == 1) {
                const double distance = point_distance(node.point(0), target);
                if (distance < best_distance) {
                    best = node.point(0);
                    best_distance = distance;
                }
            }
            for (uint32_t index = 0; index < node.child_count(); ++index)
                visit(node.child(index));
        };
    visit(entity);
    return {best, best_distance};
}

} // namespace

std::pair<c_point, double> project_onto_loop(const ExtrusionEntity &entity,
                                             c_point target)
{
    return project_onto_loop_impl(entity, target);
}

}}} // namespace slic3r_api::SeamPlacement::CandidatePreparation
