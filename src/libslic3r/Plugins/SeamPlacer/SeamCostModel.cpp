///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "SeamCostModel.hpp"

#include <algorithm>
#include <atomic>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <functional>
#include <stdexcept>
#include <vector>

#include <array>
#include <limits>
#include "libslic3r/Geometry/Curves.hpp"
#include "libslic3r/Api/plugin/cpp/KDTreeIndirect.hpp"
#include "libslic3r/Api/plugin/cpp/MeshQueryViews.hpp"
#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"
#include "libslic3r/Api/plugin/cpp/VolumeViews.hpp"

/* Scalar seam costs and visibility sampling for built-in seam placement.
   Angle and travel costs use local geometry. Visibility builds an object-space
   mesh scene, samples its surface and casts rays before scoring candidates.
   The optional observer runs before this work and does not alter sample density.
   A process-wide diagnostic counts permitted build calls independently of
   progress callbacks, so tests can detect repeated construction during scoring.

       calculate_cost_visibility()
       |-- VisibilityModel::build(): check permission, sample the scene and cast rays
       `-- VisibilityModel::calculate(): interpolate costs for each candidate set
           `-- Impl::calculate_point_visibility(): query the stable sample tree
*/

namespace Slic3r {
namespace SeamCostModel {

using namespace slic3r_api;

namespace {
std::atomic<VisibilityCalculationObserver> visibility_observer{nullptr};
std::atomic<uint64_t> visibility_build_count{0};
}

// Return the diagnostic count without synchronizing model data between threads.
uint64_t visibility_model_build_count();

uint64_t visibility_model_build_count()
{
    return visibility_build_count.load(std::memory_order_relaxed);
}

void set_visibility_calculation_observer(VisibilityCalculationObserver observer)
{
    visibility_observer.store(observer, std::memory_order_release);
}

void check_visibility_calculation(const char *path)
{
    const VisibilityCalculationObserver observer = visibility_observer.load(std::memory_order_acquire);
    if (observer != nullptr)
        observer(path);
}

// Keep the legacy visibility density in every configuration. Changing the
// sample count changes the selected seam, so Debug must use the same cost
// policy as Release while preserving the legacy mesh and ray density.
constexpr size_t kVisibilitySamplesCount = 30000;
constexpr size_t kDecimationTriangleCountTarget = 16000;
constexpr size_t kRaysPerSampleAxis = 5;

/* Evaluate the legacy normalized falloff used for travel and angle costs. */
float gauss(float value, float mean_x_coord, float mean_value, float falloff_speed);

/* Convert a signed corner angle into the legacy asymmetric corner penalty. */
float compute_angle_penalty(float ccw_angle);


/* Compute Cost visibility with an object-local mesh and the legacy ray model. */
std::vector<float> calculate_cost_visibility(const slic3r_api::Object &object,
                                             const std::vector<c_vec3f> &points,
                                             const mesh_query_cancel *cancel,
                                             const std::function<void(double)> &report_progress);

namespace {

// Small public-value vector operations keep the visibility policy independent
// of Eigen and host mesh types. Vectors remain unscaled millimetres.
c_vec3f add(c_vec3f a, c_vec3f b);
c_vec3f mul(c_vec3f a, float b);
c_vec3f sub(c_vec3f a, c_vec3f b);
float dot(c_vec3f a, c_vec3f b);
c_vec3f cross(c_vec3f a, c_vec3f b);
float norm(c_vec3f a);
c_vec3f normalized(c_vec3f a);
void throw_if_mesh_cancelled(const mesh_query_cancel *cancel);
// Wrap the angle-sampling walk around a closed polyline.
size_t prev_idx_modulo(size_t i, size_t n);
size_t next_idx_modulo(size_t i, size_t n);
c_vec3f add(c_vec3f a, c_vec3f b) { return {a.x+b.x,a.y+b.y,a.z+b.z}; }
c_vec3f mul(c_vec3f a, float b) { return {a.x*b,a.y*b,a.z*b}; }
c_vec3f sub(c_vec3f a, c_vec3f b) { return add(a,mul(b,-1)); }
float dot(c_vec3f a,c_vec3f b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
c_vec3f cross(c_vec3f a,c_vec3f b) { return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x}; }
float norm(c_vec3f a) { return std::sqrt(dot(a,a)); }
c_vec3f normalized(c_vec3f a) { return mul(a,1.f/norm(a)); }
void throw_if_mesh_cancelled(const mesh_query_cancel *cancel)
{
    if (cancel != nullptr && cancel->is_cancelled != nullptr &&
        cancel->is_cancelled(cancel->context) != 0)
        throw MeshQueryError(MESH_QUERY_CANCELLED,
                             "Cost visibility initialization was cancelled.");
}
size_t prev_idx_modulo(size_t i,size_t n) { return i ? i-1 : n-1; }
size_t next_idx_modulo(size_t i,size_t n) { return i+1 == n ? 0 : i+1; }
struct CoordinateFunctor {
    const std::vector<mesh_query_sample> *samples;
    float operator()(size_t index, size_t dimension) const {
        const c_vec3f p = (*samples)[index].position;
        return dimension == 0 ? p.x : dimension == 1 ? p.y : p.z;
    }
};
} // namespace

// The pimpl stays at one address because the tree functor borrows its samples.
struct VisibilityModel::Impl {
    std::vector<mesh_query_sample> samples;
    std::vector<float> visibility;
    slic3r_api::KDTreeIndirect<3,float,CoordinateFunctor> tree{CoordinateFunctor{&samples}, size_t(0), double(EPSILON)};
    float radius = 0;
    bool built = false;
    // Interpolate the native plane/distance weights at one object-space point.
    float calculate_point_visibility(c_vec3f point) const;
};

namespace {
// Generate the native hemisphere grid in the local normal frame.
c_vec3f hemisphere(float x,float y);
c_vec3f normal_frame(c_vec3f normal,c_vec3f direction);
c_vec3f hemisphere(float x,float y)
{
    const float a=2.f*float(PI)*x, r=2.f*std::sqrt(y-y*y);
    return {std::cos(a)*r,std::sin(a)*r,std::abs(1.f-2.f*y)};
}
c_vec3f normal_frame(c_vec3f normal,c_vec3f direction)
{
    const c_vec3f z=normalized(normal);
    const c_vec3f seed=std::abs(z.x)>.99f ? c_vec3f{0,1,0} : c_vec3f{1,0,0};
    const c_vec3f y=normalized(cross(z,seed)), x=cross(y,z);
    return add(add(mul(x,direction.x),mul(y,direction.y)),mul(z,direction.z));
}
} // namespace

VisibilityModel::VisibilityModel() : m_impl(std::make_unique<Impl>()) {}
VisibilityModel::~VisibilityModel() = default;

void VisibilityModel::build(const Object &object,
                            const mesh_query_cancel *cancel,
                            const std::function<void(double)> &report_progress)
{
    check_visibility_calculation("plugin");
    visibility_build_count.fetch_add(1, std::memory_order_relaxed);
    m_impl = std::make_unique<Impl>();
    Impl &model = *m_impl;
    if (report_progress)
        report_progress(0.0);
    // Work meshes die after scene construction. The scene owns transformed copies
    // through sampling and raycasting, then dies when build() returns; only the
    // samples, visibility values and search tree remain in the reusable model.
    const MeshQueryScene scene = [&] {
        WorkingMesh positives, negatives;
        for (uint32_t i=0; i<object.volume_count(); ++i) {
            const Volume volume=object.volume(i);
            if (volume.is_model_part()) positives.append(volume.mesh().handle(),volume.matrix(),cancel);
            else if (volume.is_negative()) negatives.append(volume.mesh().handle(),volume.matrix(),cancel);
        }
        positives.decimate(kDecimationTriangleCountTarget,cancel);
        negatives.decimate(kDecimationTriangleCountTarget,cancel);
        return MeshQueryScene({{positives.handle(),object.transform_centered(),0},
                               {negatives.handle(),object.transform_centered(),1}},cancel);
    }();
    if (report_progress)
        report_progress(0.1 / 0.9);
    const MeshSurfaceSamples sampled=scene.sample(kVisibilitySamplesCount,cancel);
    throw_if_mesh_cancelled(cancel);
    model.samples=sampled.values();
    if (report_progress)
        report_progress(0.2 / 0.9);
    if (model.samples.empty()) {
        model.built = true;
        if (report_progress)
            report_progress(1.0);
        return;
    }
    model.tree.build(model.samples.size());
    throw_if_mesh_cancelled(cancel);
    const float density=float(kVisibilitySamplesCount)/float(sampled.area());
    model.radius=std::sqrt(4.f/(-std::log(.9f)*density)/float(PI));
    model.visibility.assign(model.samples.size(),1.f);
    // A negative part matters even when random sampling misses its surface.
    const bool negative=scene.triangle_count(1)!=0;
    constexpr size_t rays_per_sample=kRaysPerSampleAxis*kRaysPerSampleAxis;
    const float step=1.f/float(kRaysPerSampleAxis);
    std::vector<c_vec3f> directions;
    for (size_t x=0; x<kRaysPerSampleAxis; ++x)
        for (size_t y=0; y<kRaysPerSampleAxis; ++y)
            directions.push_back(hemisphere(float(x)*step+step/2,float(y)*step+step/2));
    const size_t ray_batch_count = (model.samples.size() + 255) / 256;
    for (size_t begin=0; begin<model.samples.size(); begin+=256) {
        throw_if_mesh_cancelled(cancel);
        const size_t end=std::min(model.samples.size(),begin+256);
        const size_t batch_index = begin / 256;
        std::vector<mesh_query_ray> rays;
        rays.reserve((end-begin)*rays_per_sample);
        for (size_t i=begin; i<end; ++i) {
            const mesh_query_sample &sample=model.samples[i];
            const float sign=negative && sample.geometry_id==1 ? -1.f : 1.f;
            const c_vec3f origin=add(sample.position,mul(sample.normal,.01f*sign));
            for (c_vec3f local:directions) {
                const c_vec3f direction=mul(normal_frame(sample.normal,local),sign);
                rays.push_back({{origin.x,origin.y,origin.z},{direction.x,direction.y,direction.z},0,std::numeric_limits<double>::infinity()});
            }
        }
        const MeshRayHits intersections=scene.cast(rays,negative,cancel);
        for (size_t r=0; r<rays.size(); ++r) {
            if (r % 256 == 0) throw_if_mesh_cancelled(cancel);
            const std::vector<mesh_query_hit> hits=intersections.ray(r);
            if (hits.empty()) continue;
            const c_vec3f direction{float(rays[r].direction.x),float(rays[r].direction.y),float(rays[r].direction.z)};
            bool occluded=false;
            if (!negative) occluded=dot(hits.front().normal,direction)<=0;
            else {
                int counter=0;
                size_t hit_index=0;
                for (std::vector<mesh_query_hit>::const_reverse_iterator hit=hits.rbegin(); hit!=hits.rend(); ++hit) {
                    if (hit_index++ % 256 == 0)
                        throw_if_mesh_cancelled(cancel);
                    const float orientation=dot(hit->normal,direction);
                    const int sign=int(orientation>0)-int(orientation<0);
                    counter += hit->geometry_id==1 ? -sign : sign;
                }
                occluded=counter==0;
            }
            if (occluded) model.visibility[begin+r/rays_per_sample]-=1.f/float(rays_per_sample);
        }
        if (report_progress)
            report_progress((0.2 + 0.7 * double(batch_index + 1) /
                                      double(std::max(size_t(1), ray_batch_count))) / 0.9);
    }
    model.built = true;
    if (report_progress)
        report_progress(1.0);
}
float VisibilityModel::Impl::calculate_point_visibility(c_vec3f point) const
{
    const Impl &model = *this;
    if (model.samples.empty()) return 1.f;
    const std::array<float,3> query{point.x,point.y,point.z};
    const std::vector<size_t> nearby=slic3r_api::find_nearby_points(model.tree,query,model.radius);
    if (nearby.empty()) return 1.f;
    float weights=0, visibility=0;
    for (size_t i:nearby) {
        const c_vec3f delta=sub(point,model.samples[i].position);
        const float weight=model.radius-std::abs(dot(delta,model.samples[i].normal))+model.radius-norm(delta);
        visibility+=weight*model.visibility[i]; weights+=weight;
    }
    return visibility/weights;
}

namespace {
std::vector<float> calculate_polyline_angles_at_vertices(const std::vector<c_point> &polyline,
                                                         const std::vector<float> &lengths,
                                                         float min_arm_length)
{
    std::vector<float> result(polyline.size());
    assert(polyline.size() - 1 == lengths.size());
    if (polyline.size() == 1) {
        result[0] = 0.0f;
        return result;
    }

    size_t index_previous = 0;
    size_t index_current = 0;
    size_t index_next = 0;
    size_t index_start = 0;
    size_t index_end = 0;
    float distance_to_previous = 0.0f;
    float distance_to_next = 0.0f;
    const bool is_polygon = slic3r_api::operator==(polyline.front(), polyline.back());
    if (is_polygon) {
        index_end = polyline.size() - 1;
        while (distance_to_previous < min_arm_length) {
            index_previous = prev_idx_modulo(index_previous, lengths.size());
            distance_to_previous += lengths[index_previous];
        }
    } else {
        index_start = 1;
        index_end = polyline.size() - 1;
        result.front() = -float(PI) / 2.0f;
        result.back() = -float(PI) / 2.0f;
    }

    for (size_t index = index_start; index < index_end; ++index) {
        while (index_previous < lengths.size() &&
               distance_to_previous - lengths[index_previous] > min_arm_length) {
            distance_to_previous -= lengths[index_previous];
            index_previous = is_polygon ? next_idx_modulo(index_previous, lengths.size()) : index_previous + 1;
        }
        while (distance_to_next < min_arm_length && index_next < lengths.size()) {
            distance_to_next += lengths[index_next];
            index_next = is_polygon ? next_idx_modulo(index_next, lengths.size()) : index_next + 1;
        }

        const c_point &point_previous = polyline[index_previous];
        const c_point &point_current = polyline[index_current];
        const c_point &point_next = polyline[index_next];
        result[index_current] = float(slic3r_api::angle_ccw(
            slic3r_api::operator-(point_current, point_previous), slic3r_api::operator-(point_next, point_current)));
        const float current_distance = lengths[index_current];
        ++index_current;
        distance_to_previous += current_distance;
        distance_to_next -= current_distance;
    }
    if (is_polygon) result.back() = result.front();
    return result;
}

} // namespace

float gauss(float value, float mean_x_coord, float mean_value, float falloff_speed)
{
    // base function: ((e^((1/(x^2+1)))-1)/(e-1))
    const float shifted = value - mean_x_coord;
    const float denominator = falloff_speed * shifted * shifted + 1.0f;
    const float exponent = 1.0f / denominator;
    return mean_value * (std::exp(exponent) - 1.0f) / (std::exp(1.0f) - 1.0f);
}

float compute_angle_penalty(float ccw_angle)
{
    // The Gaussian and sigmoid terms deliberately make concave corners less
    // attractive than convex corners while retaining a smooth score.
    return gauss(ccw_angle, 0.0f, 1.0f, 3.0f) +
           1.0f / (2.0f + std::exp(-ccw_angle));
}

std::vector<float> calculate_cost_angles(const std::vector<c_point> &points, float min_arm_length)
{
    if (points.size() < 2) return std::vector<float>(points.size(), 0.0f);

    std::vector<c_point> ordered_points = points;
    const bool reverse = is_clockwise(points);
    if (reverse) std::reverse(ordered_points.begin(), ordered_points.end());

    std::vector<float> lengths;
    lengths.reserve(ordered_points.size() - 1);
    for (size_t index = 0; index + 1 < ordered_points.size(); ++index)
        lengths.push_back(float(unscaled(c_point_distance_to(ordered_points[index], ordered_points[index + 1]))));
    if (std::none_of(lengths.begin(), lengths.end(), [](float length) { return length > 0.0f; }))
        return std::vector<float>(points.size(), 0.0f);

    std::vector<float> angles = calculate_polyline_angles_at_vertices(
        ordered_points, lengths, min_arm_length);
    if (reverse) std::reverse(angles.begin(), angles.end());
    return angles;
}

std::vector<float> calculate_cost_visibility(const slic3r_api::Object &object,
                                             const std::vector<c_vec3f> &points,
                                             const mesh_query_cancel *cancel,
                                             const std::function<void(double)> &report_progress)
{
    VisibilityModel model;
    model.build(object, cancel, [&](double fraction) {
        if (report_progress) report_progress(0.9 * fraction);
    });
    return model.calculate(points, cancel, [&](double fraction) {
        if (report_progress) report_progress(0.9 + 0.1 * fraction);
    });
}

std::vector<float> VisibilityModel::calculate(const std::vector<c_vec3f> &points,
                                             const mesh_query_cancel *cancel,
                                             const std::function<void(double)> &report_progress) const
{
    if (!m_impl || !m_impl->built)
        throw std::logic_error("Visibility model is not built.");
    if (report_progress)
        report_progress(0.0);
    std::vector<float> visibility;
    visibility.reserve(points.size());
    for (size_t index = 0; index < points.size(); ++index) {
        if (index % 64 == 0) throw_if_mesh_cancelled(cancel);
        visibility.push_back(m_impl->calculate_point_visibility(points[index]));
        if (report_progress && (index % 64 == 0 || index + 1 == points.size()))
            report_progress(double(index + 1) / double(std::max(size_t(1), points.size())));
    }
    if (report_progress)
        report_progress(1.0);
    return visibility;
}

bool fit_weighted_cubic_bspline(const std::vector<WeightedSplinePoint> &points,
                                std::vector<c_vec2d> &fitted_points)
{
    fitted_points.clear();
    if (points.size() < 4) return false;
    std::vector<Vec2f> observations;
    std::vector<float> parameters;
    std::vector<float> weights;
    observations.reserve(points.size());
    parameters.reserve(points.size());
    weights.reserve(points.size());
    double total_length = 0;
    for (size_t index = 0; index < points.size(); ++index) {
        const WeightedSplinePoint &point = points[index];
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z) || !std::isfinite(point.weight) ||
            point.weight <= 0 || (index != 0 && point.z <= points[index - 1].z))
            return false;
        const float x = float(point.x);
        const float y = float(point.y);
        const float parameter = float(point.z);
        const float weight = float(point.weight);
        if (!std::isfinite(x) || !std::isfinite(y) ||
            !std::isfinite(parameter) || !std::isfinite(weight) || weight <= 0 ||
            (!parameters.empty() && parameter <= parameters.back()))
            return false;
        observations.emplace_back(x, y);
        parameters.push_back(parameter);
        weights.push_back(weight);
        if (index != 0) {
            const double dx = point.x - points[index - 1].x;
            const double dy = point.y - points[index - 1].y;
            const double dz = point.z - points[index - 1].z;
            total_length += std::hypot(dx, dy, dz);
            if (!std::isfinite(total_length)) return false;
        }
    }
    const float parameter_span = parameters.back() - parameters.front();
    if (!std::isfinite(parameter_span) || parameter_span <= 0) return false;
    const double requested_segments = total_length / 4.0;
    // fit_curve requires at least one segment and no more segments than
    // observations; cap geometric detail before entering its assert boundary.
    const size_t segment_count = requested_segments >= double(points.size()) ?
        points.size() : std::max(size_t(1), size_t(requested_segments));
    const float segment_size = parameter_span / float(segment_count);
    if (!std::isfinite(segment_size) || segment_size <= 0) return false;
    try {
        const Geometry::PiecewiseFittedCurve<2, float,
            Geometry::CubicBSplineKernel<float>> curve =
                Geometry::fit_cubic_bspline(observations, parameters, weights,
                                            segment_count);
        fitted_points.reserve(points.size());
        for (const WeightedSplinePoint &point : points) {
            const Vec2f fitted = curve.get_fitted_value(float(point.z));
            if (!std::isfinite(fitted.x()) || !std::isfinite(fitted.y())) {
                fitted_points.clear();
                return false;
            }
            fitted_points.push_back({fitted.x(), fitted.y()});
        }
    } catch (const std::exception &) {
        fitted_points.clear();
        return false;
    }
    return fitted_points.size() == points.size();
}

} // namespace SeamCostModel
} // namespace Slic3r
