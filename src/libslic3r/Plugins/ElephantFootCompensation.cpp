///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/ Copyright (c) Prusa Research 2019 - 2023 Vojtěch Bubník @bubnikv, Lukáš Hejl @hejllukas, Tomáš Mészáros @tamasmeszaros
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "ElephantFootCompensation.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"
#include "libslic3r/Api/plugin/cpp/LineDistancer.hpp"

/*
This SDK-only geometry routine preserves narrow first-layer walls for the XY
compensation plugin. It samples contour segments, measures opposing boundaries
through LineDistancer, then limits the withdrawal at each vertex. A variable
inward offset publishes the result in caller storage without changing input.
Distances follow the native ElephantFootCompensation routine for parity.

    elephant_foot_compensation()
    |-- resample_ring(): retain the source segment and interpolation position
    |-- contour_distance2(): measure nearby opposing segments
    |-- smooth_compensation_banded(): limit changes along the sampled boundary
    `-- clipper_variable_offset_inner(): own the compensated polygon collection

Calls share no mutable state. The caller keeps storage alive until all result
views have been destroyed; allocation and geometry errors propagate to the plugin.
*/

namespace slic3r_api { namespace XYSizeCompensationPlugin { namespace detail {
namespace {

using slic3r_api::LineDistancer;

struct ResampledPoint
{
    std::size_t source_index = 0;
    bool interpolated = false;
    double curve_parameter = 0.;
};

struct SegmentMetadata
{
    std::size_t ring_index = 0;
    std::size_t start_index = 0;
};

double cross(double ax, double ay, double bx, double by)
{
    return ax * by - ay * bx;
}

double dot(double ax, double ay, double bx, double by)
{
    return ax * bx + ay * by;
}

std::size_t previous_index(std::size_t index, std::size_t size)
{
    return index == 0 ? size - 1 : index - 1;
}

std::size_t next_index(std::size_t index, std::size_t size)
{
    return index + 1 == size ? 0 : index + 1;
}

std::vector<c_point> resample_ring(const std::vector<c_point> &ring,
                                   double interval,
                                   std::vector<ResampledPoint> &parameters)
{
    std::vector<c_point> sampled;
    if (ring.size() <= 2)
        return sampled;

    sampled.reserve(ring.size());
    parameters.reserve(ring.size());
    double previous_x = double(ring.back().x);
    double previous_y = double(ring.back().y);
    for (std::size_t point_index = 0; point_index < ring.size(); ++point_index) {
        const c_point point = ring[point_index];
        const double x = double(point.x);
        const double y = double(point.y);
        const double dx = x - previous_x;
        const double dy = y - previous_y;
        // Match the native Eigen norm calculation used by the original resampler.
        const double length = std::sqrt(dx * dx + dy * dy);
        const std::size_t steps = std::size_t(std::ceil(length / interval));
        if (steps == 0)
            return {};
        const double step_length = length / double(steps);
        for (std::size_t step = 1; step < steps; ++step) {
            const double t = double(step) / double(steps);
            sampled.push_back(c_point{
                coord_t(std::llround(previous_x + dx * t)),
                coord_t(std::llround(previous_y + dy * t))
            });
            parameters.push_back({ point_index, true, step_length });
        }
        sampled.push_back(point);
        parameters.push_back({ point_index, false, step_length });
        previous_x = x;
        previous_y = y;
    }
    for (std::size_t idx = 1; idx < parameters.size(); ++idx)
        parameters[idx].curve_parameter += parameters[idx - 1].curve_parameter;
    return sampled;
}

std::vector<SegmentMetadata> segment_metadata(const std::vector<std::vector<c_point>> &rings)
{
    std::vector<SegmentMetadata> segments;
    for (std::size_t ring_index = 0; ring_index < rings.size(); ++ring_index) {
        const std::vector<c_point> &ring = rings[ring_index];
        if (ring.size() < 2)
            continue;
        for (std::size_t idx = 1; idx < ring.size(); ++idx)
            segments.push_back({ ring_index, idx - 1 });
        if (ring.front().x != ring.back().x || ring.front().y != ring.back().y)
            segments.push_back({ ring_index, ring.size() - 1 });
    }
    return segments;
}

bool left_of_segment(const std::vector<c_point> &ring, std::size_t start, c_point point)
{
    const c_point first = ring[start];
    const c_point second = ring[next_index(start, ring.size())];
    return cross(double(second.x - first.x), double(second.y - first.y),
                 double(point.x - first.x), double(point.y - first.y)) > 0.;
}

bool inside_corner(const std::vector<c_point> &ring, std::size_t index, c_point opposite)
{
    const c_point previous = ring[previous_index(index, ring.size())];
    const c_point current = ring[index];
    const c_point next = ring[next_index(index, ring.size())];
    const double v1x = double(current.x - previous.x);
    const double v1y = double(current.y - previous.y);
    const double v2x = double(next.x - current.x);
    const double v2y = double(next.y - current.y);
    const double point1x = double(opposite.x - previous.x);
    const double point1y = double(opposite.y - previous.y);
    const double point2x = double(opposite.x - current.x);
    const double point2y = double(opposite.y - current.y);
    const bool left_of_first = cross(v1x, v1y, point1x, point1y) > 0.;
    const bool left_of_second = cross(v2x, v2y, point2x, point2y) > 0.;
    return cross(v1x, v1y, v2x, v2y) > 0. ? left_of_first && left_of_second
                                           : left_of_first || left_of_second;
}

std::vector<float> contour_distance2(const LineDistancer &index,
                                     const std::vector<SegmentMetadata> &segments,
                                     const std::vector<std::vector<c_point>> &rings,
                                     std::size_t ring_index,
                                     const std::vector<c_point> &ring,
                                     const std::vector<ResampledPoint> &parameters,
                                     double compensation,
                                     double search_radius)
{
    std::vector<float> distances;
    if (ring.size() <= 2)
        return distances;

    std::vector<double> vertex_parameters(ring.size(), 0.);
    for (const ResampledPoint &sample : parameters)
        if (!sample.interpolated && sample.source_index < vertex_parameters.size())
            vertex_parameters[sample.source_index] = sample.curve_parameter;
    const double contour_parameter_end = parameters.empty() ? 0. : parameters.back().curve_parameter;
    const double same_contour_accept = 0.5 * compensation * PI;
    distances.reserve(ring.size());
    for (std::size_t point_index = 0; point_index < ring.size(); ++point_index) {
        const c_point point = ring[point_index];
        const c_point previous = ring[previous_index(point_index, ring.size())];
        const c_point next = ring[next_index(point_index, ring.size())];
        const double v1x = double(point.x - previous.x);
        const double v1y = double(point.y - previous.y);
        const double v2x = double(next.x - point.x);
        const double v2y = double(next.y - point.y);
        const double inside_x = -v1y - v2y;
        const double inside_y = v1x + v2x;

        bool found = false;
        double closest_distance = std::numeric_limits<double>::max();
        for (const std::size_t segment_index : index.all_lines_in_radius(point, search_radius)) {
            if (segment_index >= segments.size())
                continue;
            const SegmentMetadata meta = segments[segment_index];
            if (meta.ring_index >= rings.size())
                continue;
            const std::vector<c_point> &segment_ring = rings[meta.ring_index];
            if (segment_ring.size() < 2 || meta.start_index >= segment_ring.size())
                continue;
            const c_point first = segment_ring[meta.start_index];
            const c_point second = segment_ring[next_index(meta.start_index, segment_ring.size())];
            const double vx = double(second.x - first.x);
            const double vy = double(second.y - first.y);
            const double ax = double(point.x - first.x);
            const double ay = double(point.y - first.y);
            const double length_squared = vx * vx + vy * vy;
            const double t = length_squared == 0. ? 0. :
                std::clamp((ax * vx + ay * vy) / length_squared, 0., 1.);
            const double foot_x = double(first.x) + t * vx;
            const double foot_y = double(first.y) + t * vy;
            const double bisector_x = foot_x - double(point.x);
            const double bisector_y = foot_y - double(point.y);
            const double distance = std::sqrt(bisector_x * bisector_x + bisector_y * bisector_y);

            if (inside_x * bisector_x + inside_y * bisector_y <= 0. ||
                (found && distance >= closest_distance))
                continue;

            bool accept = true;
            if (meta.ring_index == ring_index) {
                double parameter_low = parameters[point_index].curve_parameter;
                double parameter_high = t * std::sqrt(length_squared);
                if (meta.start_index + 1 < segment_ring.size())
                    parameter_high += vertex_parameters[meta.start_index];
                if (parameter_low > parameter_high)
                    std::swap(parameter_low, parameter_high);
                const double along_contour = std::min(parameter_high - parameter_low,
                    parameter_low + contour_parameter_end - parameter_high);
                if (along_contour < same_contour_accept) {
                    accept = false;
                } else if (distance < search_radius + double(SCALED_EPSILON)) {
                    const bool inside = t == 0. ? inside_corner(segment_ring, meta.start_index, point) :
                        t == 1. ? inside_corner(segment_ring, next_index(meta.start_index, segment_ring.size()), point) :
                                  left_of_segment(segment_ring, meta.start_index, point);
                    accept = inside && along_contour > 0.6 * PI * distance;
                }
            }
            if (accept) {
                closest_distance = distance;
                found = true;
            }
        }
        distances.push_back(float(found ? std::min(closest_distance, search_radius) : search_radius));
    }
    return distances;
}

float lerp(float first, float second, float t)
{
    return first + (second - first) * t;
}

void smooth_compensation_banded(const std::vector<c_point> &contour,
                               float band,
                               std::vector<float> &compensation,
                               float strength,
                               std::size_t iterations)
{
    if (contour.size() <= 2 || contour.size() != compensation.size())
        return;
    std::vector<float> output(compensation);
    const float minimum_distance_squared = band * band;
    for (std::size_t iteration = 0; iteration < iterations; ++iteration) {
        for (std::size_t idx = 0; idx < compensation.size(); ++idx) {
            const c_point current = contour[idx];
            std::size_t neighbor = previous_index(idx, contour.size());
            c_point previous_point = contour[neighbor];
            float previous = compensation[neighbor];
            float dx = float(current.x) - float(previous_point.x);
            float dy = float(current.y) - float(previous_point.y);
            float distance_squared = dx * dx + dy * dy;
            if (distance_squared < minimum_distance_squared) {
                float length = std::sqrt(distance_squared);
                std::size_t previous_index_used = neighbor;
                neighbor = previous_index(neighbor, contour.size());
                while (neighbor != idx) {
                    const c_point point = contour[neighbor];
                    const float point_x = float(point.x);
                    const float point_y = float(point.y);
                    const float previous_x = float(previous_point.x);
                    const float previous_y = float(previous_point.y);
                    const float this_dx = point_x - previous_x;
                    const float this_dy = point_y - previous_y;
                    const float this_length = std::sqrt(this_dx * this_dx + this_dy * this_dy);
                    const float next_length = length + this_length;
                    if (next_length > band) {
                        previous = lerp(compensation[previous_index_used], compensation[neighbor],
                                        (band - length) / this_length);
                        break;
                    }
                    previous = compensation[neighbor];
                    previous_point = point;
                    length = next_length;
                    previous_index_used = neighbor;
                    neighbor = previous_index(neighbor, contour.size());
                }
            }

            neighbor = next_index(idx, contour.size());
            previous_point = contour[neighbor];
            float next = compensation[neighbor];
            dx = float(previous_point.x) - float(current.x);
            dy = float(previous_point.y) - float(current.y);
            distance_squared = dx * dx + dy * dy;
            if (distance_squared < minimum_distance_squared) {
                float length = std::sqrt(distance_squared);
                std::size_t previous_index_used = neighbor;
                neighbor = next_index(neighbor, contour.size());
                while (neighbor != idx) {
                    const c_point point = contour[neighbor];
                    const float point_x = float(point.x);
                    const float point_y = float(point.y);
                    const float previous_x = float(previous_point.x);
                    const float previous_y = float(previous_point.y);
                    const float this_dx = point_x - previous_x;
                    const float this_dy = point_y - previous_y;
                    const float this_length = std::sqrt(this_dx * this_dx + this_dy * this_dy);
                    const float next_length = length + this_length;
                    if (next_length > band) {
                        next = lerp(compensation[previous_index_used], compensation[neighbor],
                                    (band - length) / this_length);
                        break;
                    }
                    next = compensation[neighbor];
                    previous_point = point;
                    length = next_length;
                    previous_index_used = neighbor;
                    neighbor = next_index(neighbor, contour.size());
                }
            }

            const float laplacian = compensation[idx] * (1.f - strength) +
                                    0.5f * strength * (previous + next);
            // Compensation is negative; smoothing may only reduce its magnitude.
            output[idx] = std::max(laplacian, compensation[idx]);
        }
        compensation.swap(output);
    }
}

void write_ring(polygon_handle *destination, const std::vector<c_point> &points)
{
    multipoint_handle *mutable_points = polygon_as_multipoint(destination);
    multipoint_clear(mutable_points);
    for (const c_point point : points)
        multipoint_push_back(mutable_points, point);
}

StoredExPolygonCollection unchanged(storage_handle *storage, const ExPolygon &input)
{
    StoredExPolygonCollection result(storage);
    result.push_back(input);
    return result;
}

bool valid_orientation(const ExPolygon &area)
{
    if (!area.contour().is_counter_clockwise())
        return false;
    for (const Polygon hole : area.holes())
        if (!hole.is_clockwise())
            return false;
    return true;
}

} // namespace

StoredExPolygonCollection elephant_foot_compensation(storage_handle *storage,
                                                     const ExPolygon &input,
                                                     double minimum_contour_width_mm,
                                                     coord_t compensation_scaled)
{
    if (storage == nullptr || !std::isfinite(minimum_contour_width_mm) ||
        minimum_contour_width_mm <= 0. || compensation_scaled < 0)
        throw std::invalid_argument("Invalid input for first-layer no-collapse protection.");
    if (input.contour().empty())
        return StoredExPolygonCollection(storage);
    // The island comes straight out of earlier geometry operations, so it can
    // contain points almost on top of each other, or even a broken outline
    // (fewer than three points, empty hole). We do not throw an error for that:
    // - close points are cleaned up a few lines below, on a copy (ensure_valid);
    // - a shape that cannot be used is returned as it was, without compensation.
    if (!valid_orientation(input))
        return unchanged(storage, input);

    const double scaled_compensation = double(compensation_scaled);
    const double scaled_minimum_width = scale_d(minimum_contour_width_mm);
    const double minimum_width_compensated = scaled_minimum_width + 2. * scaled_compensation;
    const double search_radius = minimum_width_compensated + scaled_minimum_width * 0.5;
    const c_bounding_box input_bounds = input.contour().bounding_box();
    const double width = double(input_bounds.max.x - input_bounds.min.x);
    const double height = double(input_bounds.max.y - input_bounds.min.y);
    if (width < minimum_width_compensated + double(SCALED_EPSILON) ||
        height < minimum_width_compensated + double(SCALED_EPSILON))
        return unchanged(storage, input);

    // ensure_valid() on a copy runs the same per-ring Douglas-Peucker and repair
    // as the legacy ExPolygon::simplify(), whose front() the legacy routine takes.
    // Degenerate holes may be dropped, so only an empty result needs the fallback.
    StoredExPolygonCollection simplified(storage, input);
    simplified.ensure_valid(SCALED_EPSILON);
    if (simplified.empty())
        return unchanged(storage, input);
    const ExPolygon simplified_area = simplified.front();
    // The SDK is_valid() checks point spacing and ring orientations.
    if (!simplified_area.is_valid())
        return unchanged(storage, input);

    std::vector<std::vector<c_point>> rings;
    rings.reserve(std::size_t(simplified_area.hole_size()) + 1);
    rings.push_back(simplified_area.contour().points());
    for (const Polygon hole : simplified_area.holes())
        rings.push_back(hole.points());

    // LineDistancer supplies an owned AABB index and returns every segment in
    // the query radius in stable insertion order. The parallel table preserves
    // ring and segment identity for the native same-contour acceptance rules.
    LineDistancer segment_index(simplified.readonly());
    const std::vector<SegmentMetadata> segments = segment_metadata(rings);
    StoredExPolygon resampled(storage);
    resampled.copy_from(simplified_area);
    std::vector<std::vector<float>> deltas;
    deltas.reserve(rings.size());
    constexpr double resample_interval = scale_d(0.5);

    for (std::size_t ring_index = 0; ring_index < rings.size(); ++ring_index) {
        std::vector<ResampledPoint> parameters;
        std::vector<c_point> sampled = resample_ring(rings[ring_index], resample_interval, parameters);
        if (sampled.size() <= 2 || sampled.size() != parameters.size())
            return unchanged(storage, input);

        std::vector<float> distances = contour_distance2(segment_index, segments, rings,
            ring_index, sampled, parameters, scaled_compensation, search_radius);
        if (distances.size() != sampled.size())
            return unchanged(storage, input);
        for (float &distance : distances) {
            if (distance < scaled_minimum_width)
                distance = 0.f;
            else if (distance > minimum_width_compensated)
                distance = -float(scaled_compensation);
            else
                distance = -(distance - float(scaled_minimum_width)) / 2.f;
            assert(distance >= -float(scaled_compensation) && distance <= 0.f);
        }
        smooth_compensation_banded(sampled, float(0.8 * resample_interval), distances, 0.3f, 3);
        deltas.push_back(std::move(distances));

        polygon_handle *destination = ring_index == 0 ?
            expolygon_contour(resampled.mutable_handle()) :
            expolygon_hole_at(resampled.mutable_handle(), uint32_t(ring_index - 1));
        write_ring(destination, sampled);
    }

    StoredExPolygonCollection output = clipper_variable_offset_inner(storage, resampled.readonly(), deltas, 2.);
    // Match the legacy topology/orientation checks. Point spacing is not checked:
    // rejecting two close points would cancel the whole island's compensation,
    // and the Clipper operations that use this result clean its points anyway.
    if (output.size() != 1 || output.front().hole_size() != resampled.hole_size() ||
        !valid_orientation(output.front()))
        return unchanged(storage, input);
    return output;
}

}}} // namespace slic3r_api::XYSizeCompensationPlugin::detail
