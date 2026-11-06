/*
 * Headless facet-painting implementation.
 *
 * The scene keeps annotation data in volume-local TriangleSelector form.  This
 * module builds platter-space AABB trees for point resolution, converts each
 * resolved hit back to the volume mesh for GUI-compatible selectors, and only
 * writes changed selector data after all request points have resolved.
 */

#include "paint.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/TriangleSelector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace Slic3r::Rest {

namespace {

enum class PaintChannel {
    support,
    mmu,
    generic
};

struct ResolvedPoint {
    std::size_t target_index = 0;
    int face = -1;
    Vec3d hit_platter = Vec3d::Zero();
    Vec3f hit_mesh = Vec3f::Zero();
    Vec3f camera_mesh = Vec3f::Zero();
};

// Classify a fixed or registered channel without creating scene data.
PaintChannel channel_for_kind(const std::string &kind)
{
    if (kind == "support")
        return PaintChannel::support;
    if (kind == "mmu")
        return PaintChannel::mmu;
    const std::vector<GenericFacetsAnnotationDefinition> &definitions =
        Orchestrator::instance().generic_facets_annotations();
    for (const GenericFacetsAnnotationDefinition &definition : definitions)
        if (definition.key == kind)
            return PaintChannel::generic;
    throw SceneEditError(422, "invalid_paint", "The paint kind is unknown.");
}

// Return stored channel bytes without creating an absent generic entry.
const std::pair<std::vector<std::pair<int, int>>, std::vector<bool>> *annotation_data(
    const ModelVolume &volume, const std::string &kind)
{
    const PaintChannel channel = channel_for_kind(kind);
    if (channel == PaintChannel::support)
        return &volume.supported_facets.get_data();
    if (channel == PaintChannel::mmu)
        return &volume.mm_segmentation_facets.get_data();
    const FacetsAnnotation *annotation = volume.facets_annotation(kind);
    return annotation == nullptr ? nullptr : &annotation->get_data();
}

// Check whether serialized channel data has no original-facet entries.
bool annotation_data_empty(const std::pair<std::vector<std::pair<int, int>>, std::vector<bool>> &data)
{
    return data.first.empty();
}

// Load existing bytes into a selector; absent generic channels stay empty.
void load_selector(TriangleSelector &selector, const ModelVolume &volume, const std::string &kind)
{
    const std::pair<std::vector<std::pair<int, int>>, std::vector<bool>> *data = annotation_data(volume, kind);
    if (data != nullptr && !annotation_data_empty(*data))
        selector.deserialize(*data);
}

// Store selector bytes and remove an empty generic entry; return whether data changed.
bool store_selector(ModelVolume &volume, const std::string &kind, const TriangleSelector &selector)
{
    const std::pair<std::vector<std::pair<int, int>>, std::vector<bool>> empty_data;
    const std::pair<std::vector<std::pair<int, int>>, std::vector<bool>> *before = annotation_data(volume, kind);
    const std::pair<std::vector<std::pair<int, int>>, std::vector<bool>> &before_data = before == nullptr ? empty_data : *before;
    const std::pair<std::vector<std::pair<int, int>>, std::vector<bool>> after = selector.serialize();
    if (after == before_data)
        return false;

    const PaintChannel channel = channel_for_kind(kind);
    if (channel == PaintChannel::support)
        volume.supported_facets.set(selector);
    else if (channel == PaintChannel::mmu)
        volume.mm_segmentation_facets.set(selector);
    else if (after.first.empty())
        volume.reset_facets_annotation(kind);
    else
        volume.facets_annotation_mutable(kind).set(selector);
    return true;
}

// Resolve one part or all solid parts of an object.
std::vector<ModelVolume *> target_volumes(ModelObject &object, ModelVolume *part)
{
    std::vector<ModelVolume *> result;
    if (part != nullptr) {
        if (part->get_object() != &object || !part->is_model_part())
            throw SceneEditError(422, "invalid_paint", "Only solid model parts can be painted.");
        result.push_back(part);
        return result;
    }
    for (ModelVolume *volume : object.volumes)
        if (volume->is_model_part())
            result.push_back(volume);
    if (result.empty())
        throw SceneEditError(422, "invalid_paint", "The target object has no solid model parts.");
    return result;
}

// Build heap-stable platter meshes and AABB trees for selected parts.
std::vector<std::unique_ptr<PaintTarget>> make_targets(ModelObject &object, ModelVolume *part,
    std::size_t instance_index)
{
    if (instance_index >= object.instances.size())
        throw SceneEditError(422, "invalid_paint", "The selected instance is outside the target object.");
    const ModelInstance &reference = *object.instances[instance_index];
    const std::vector<ModelVolume *> volumes = target_volumes(object, part);
    std::vector<std::unique_ptr<PaintTarget>> result;
    result.reserve(volumes.size());
    for (ModelVolume *volume : volumes) {
        std::unique_ptr<PaintTarget> target = std::make_unique<PaintTarget>();
        target->volume = volume;
        target->trafo = reference.get_matrix() * volume->get_matrix();
        target->trafo_no_translate = reference.get_matrix_no_offset() * volume->get_matrix_no_offset();
        target->platter = volume->mesh().its;
        for (Vec3f &vertex : target->platter.vertices)
            vertex = (target->trafo * vertex.cast<double>()).cast<float>();
        target->tree = std::make_unique<AABBMesh>(target->platter);
        result.push_back(std::move(target));
    }
    return result;
}

// Map request coordinates to platter space; invalid spaces throw 422.
Transform3d input_mapping(const ModelInstance &reference, const ModelVolume *part, PaintSpace space)
{
    if (space == PaintSpace::platter)
        return Transform3d::Identity();
    if (space == PaintSpace::object)
        return reference.get_matrix();
    if (space == PaintSpace::volume && part != nullptr)
        return reference.get_matrix() * part->get_matrix();
    if (space == PaintSpace::volume)
        return reference.get_matrix();
    throw SceneEditError(422, "invalid_paint", "The paint space is unknown.");
}

// Transform and normalize a direction; invalid results throw 422.
Vec3d transformed_direction(const Transform3d &mapping, const Vec3d &direction)
{
    const Vec3d transformed = mapping.linear() * direction;
    if (!transformed.allFinite() || transformed.norm() <= 1e-12)
        throw SceneEditError(422, "invalid_paint", "The transformed paint direction is invalid.");
    return transformed.normalized();
}

// Resolve one point or ray to selector coordinates; failures carry point indices.
ResolvedPoint resolve_point(const std::vector<std::unique_ptr<PaintTarget>> &targets,
    const Transform3d &mapping, const PaintRequest &request, const PaintPoint &point,
    std::size_t stroke_index, std::size_t point_index)
{
    const Vec3d input_position = mapping * point.position;
    if (!input_position.allFinite())
        throw PaintPointError(stroke_index, point_index, "off_surface");

    std::size_t best_target = targets.size();
    int best_face = -1;
    Vec3d best_hit = Vec3d::Zero();
    double best_distance = std::numeric_limits<double>::infinity();
    Vec3d view = Vec3d::Zero();
    if (point.ray) {
        const Vec3d direction = transformed_direction(mapping, point.direction);
        for (std::size_t index = 0; index < targets.size(); ++index) {
            const AABBMesh::hit_result hit = targets[index]->tree->query_ray_hit(input_position, direction);
            if (hit.is_hit() && hit.distance() < best_distance) {
                best_target = index;
                best_face = hit.face();
                best_hit = hit.position();
                best_distance = hit.distance();
            }
        }
        if (best_target == targets.size())
            throw PaintPointError(stroke_index, point_index, "no_hit");
        if (request.view_direction.has_value())
            view = transformed_direction(mapping, request.view_direction.value());
        else
            view = direction;
    } else {
        for (std::size_t index = 0; index < targets.size(); ++index) {
            int face = -1;
            Vec3d closest = Vec3d::Zero();
            const double distance = targets[index]->tree->squared_distance(input_position, face, closest);
            if (face >= 0 && distance < best_distance) {
                best_target = index;
                best_face = face;
                best_hit = closest;
                best_distance = distance;
            }
        }
        if (best_target == targets.size() || !std::isfinite(best_distance) ||
            std::sqrt(std::max(0., best_distance)) > request.tolerance)
            throw PaintPointError(stroke_index, point_index, "off_surface");
        if (request.view_direction.has_value())
            view = transformed_direction(mapping, request.view_direction.value());
        else {
            view = -targets[best_target]->tree->normal_by_face_id(best_face);
            if (!view.allFinite() || view.norm() <= 1e-12)
                throw PaintPointError(stroke_index, point_index, "off_surface");
            view.normalize();
        }
    }
    const Transform3d inverse = targets[best_target]->trafo.inverse();
    const Vec3d mesh_hit = inverse * best_hit;
    const Vec3d camera = inverse * (best_hit - view * 1000.);
    if (!mesh_hit.allFinite() || !camera.allFinite())
        throw PaintPointError(stroke_index, point_index, "off_surface");
    ResolvedPoint result;
    result.target_index = best_target;
    result.face = best_face;
    result.hit_platter = best_hit;
    result.hit_mesh = mesh_hit.cast<float>();
    result.camera_mesh = camera.cast<float>();
    return result;
}

// Resolve all points before painting and preserve same-part brush runs.
std::vector<std::vector<ResolvedPoint>> resolve_strokes(
    const std::vector<std::unique_ptr<PaintTarget>> &targets, const ModelInstance &reference,
    ModelVolume *part, const PaintRequest &request)
{
    const Transform3d mapping = input_mapping(reference, part, request.space);
    std::vector<std::vector<ResolvedPoint>> result;
    result.reserve(request.strokes.size());
    for (std::size_t stroke_index = 0; stroke_index < request.strokes.size(); ++stroke_index) {
        std::vector<ResolvedPoint> resolved;
        resolved.reserve(request.strokes[stroke_index].size());
        for (std::size_t point_index = 0; point_index < request.strokes[stroke_index].size(); ++point_index)
            resolved.push_back(resolve_point(targets, mapping, request,
                request.strokes[stroke_index][point_index], stroke_index, point_index));
        result.push_back(std::move(resolved));
    }
    return result;
}

// Apply circle/sphere cursors to each same-part run.
void apply_brush(const std::vector<std::vector<ResolvedPoint>> &strokes,
    const std::vector<std::unique_ptr<PaintTarget>> &targets,
    std::vector<std::unique_ptr<TriangleSelector>> &selectors, const PaintRequest &request)
{
    const TriangleSelector::CursorType cursor_type = request.tool == PaintTool::sphere
        ? TriangleSelector::CursorType::SPHERE : TriangleSelector::CursorType::CIRCLE;
    for (const std::vector<ResolvedPoint> &stroke : strokes) {
        std::size_t first = 0;
        while (first < stroke.size()) {
            std::size_t end = first + 1;
            while (end < stroke.size() && stroke[end].target_index == stroke[first].target_index)
                ++end;
            PaintTarget &target = *targets[stroke[first].target_index];
            TriangleSelector &selector = *selectors[stroke[first].target_index];
            if (end - first == 1) {
                const ResolvedPoint &point = stroke[first];
                std::unique_ptr<TriangleSelector::Cursor> cursor = TriangleSelector::SinglePointCursor::cursor_factory(
                    point.hit_mesh, point.camera_mesh, static_cast<float>(request.radius), cursor_type,
                    target.trafo, TriangleSelector::ClippingPlane());
                selector.select_patch(point.face, std::move(cursor), request.state, target.trafo_no_translate,
                    request.split_triangles, static_cast<float>(request.overhangs_only));
            } else {
                for (std::size_t index = first; index + 1 < end; ++index) {
                    const ResolvedPoint &current = stroke[index];
                    const ResolvedPoint &next = stroke[index + 1];
                    std::unique_ptr<TriangleSelector::Cursor> cursor = TriangleSelector::DoublePointCursor::cursor_factory(
                        current.hit_mesh, next.hit_mesh, current.camera_mesh, static_cast<float>(request.radius),
                        cursor_type, target.trafo, TriangleSelector::ClippingPlane());
                    selector.select_patch(current.face, std::move(cursor), request.state, target.trafo_no_translate,
                        request.split_triangles, static_cast<float>(request.overhangs_only));
                }
            }
            first = end;
        }
    }
}

// Apply fill or triangle seeds in request order; selector failures propagate.
void apply_fill(const std::vector<std::vector<ResolvedPoint>> &strokes,
    const std::vector<std::unique_ptr<PaintTarget>> &targets,
    std::vector<std::unique_ptr<TriangleSelector>> &selectors, const PaintRequest &request)
{
    for (const std::vector<ResolvedPoint> &stroke : strokes)
        for (const ResolvedPoint &point : stroke) {
            PaintTarget &target = *targets[point.target_index];
            TriangleSelector &selector = *selectors[point.target_index];
            if (request.tool == PaintTool::smart_fill) {
                selector.seed_fill_select_triangles(point.hit_mesh, point.face, target.trafo_no_translate,
                    TriangleSelector::ClippingPlane(), static_cast<float>(request.smart_fill_angle),
                    static_cast<float>(request.overhangs_only), true);
            } else {
                selector.bucket_fill_select_triangles(point.hit_mesh, point.face, TriangleSelector::ClippingPlane(),
                    request.tool == PaintTool::bucket_fill, true);
            }
            selector.seed_fill_apply_on_triangles(request.state);
        }
}

// Apply downward-angle selection to every matching original face.
void apply_by_angle(const std::vector<std::unique_ptr<PaintTarget>> &targets,
    std::vector<std::unique_ptr<TriangleSelector>> &selectors, const PaintRequest &request)
{
    const double radians = request.angle * std::acos(-1.) / 180.;
    for (std::size_t target_index = 0; target_index < targets.size(); ++target_index) {
        const PaintTarget &target = *targets[target_index];
        const Transform3d inverse = target.trafo_no_translate.inverse();
        Vec3f down = (inverse.linear() * (-Vec3d::UnitZ())).cast<float>().normalized();
        Vec3f limit = (inverse.linear() * Vec3d(std::sin(radians), 0., -std::cos(radians))).cast<float>().normalized();
        const float dot_limit = limit.dot(down);
        const indexed_triangle_set &mesh = target.volume->mesh().its;
        for (std::size_t face_index = 0; face_index < mesh.indices.size(); ++face_index)
            if (its_face_normal(mesh, static_cast<int>(face_index)).dot(down) > dot_limit)
                selectors[target_index]->set_facet(static_cast<int>(face_index), request.state);
    }
}

// Map painted mesh vertices to the requested output space; invalid spaces throw 422.
Transform3d output_mapping(const ModelInstance &reference, const ModelVolume &part, PaintSpace space)
{
    if (space == PaintSpace::platter)
        return reference.get_matrix() * part.get_matrix();
    if (space == PaintSpace::object)
        return part.get_matrix();
    if (space == PaintSpace::volume)
        return Transform3d::Identity();
    throw SceneEditError(422, "invalid_paint", "The paint space is unknown.");
}

// Copy strict triangles and transform their vertices without changing the selector.
indexed_triangle_set transformed_triangles(const TriangleSelector &selector, EnforcerBlockerType state,
    const Transform3d &mapping)
{
    indexed_triangle_set result = selector.get_facets_strict(state);
    for (Vec3f &vertex : result.vertices)
        vertex = (mapping * vertex.cast<double>()).cast<float>();
    return result;
}

// List readable states: enforce/block or MMU extruders 1 through 15.
std::vector<EnforcerBlockerType> channel_states(const std::string &kind)
{
    const PaintChannel channel = channel_for_kind(kind);
    std::vector<EnforcerBlockerType> states;
    if (channel == PaintChannel::mmu) {
        for (int extruder = 1; extruder <= 15; ++extruder)
            states.push_back(static_cast<EnforcerBlockerType>(extruder));
    } else {
        states.push_back(EnforcerBlockerType::ENFORCER);
        states.push_back(EnforcerBlockerType::BLOCKER);
    }
    return states;
}

// Validate request values and throw 422 without changing a scene.
void validate_request_values(const PaintRequest &request, std::size_t extruder_count)
{
    const PaintChannel channel = channel_for_kind(request.kind);
    if (channel == PaintChannel::mmu) {
        if (extruder_count < 2)
            throw SceneEditError(422, "invalid_paint", "MMU painting requires at least two extruders.");
        const int state = static_cast<int>(request.state);
        const std::size_t maximum = std::min<std::size_t>(extruder_count, 15);
        if (state < 0 || static_cast<std::size_t>(state) > maximum)
            throw SceneEditError(422, "invalid_paint", "The MMU extruder state is outside the session range.");
    } else if (request.state != EnforcerBlockerType::NONE &&
        request.state != EnforcerBlockerType::ENFORCER && request.state != EnforcerBlockerType::BLOCKER) {
        throw SceneEditError(422, "invalid_paint", "The paint state is unknown.");
    }
    if (!std::isfinite(request.radius) || request.radius < 0.05 || request.radius > 2000.)
        throw SceneEditError(422, "invalid_paint", "The brush radius must be between 0.05 and 2000 mm.");
    if (!std::isfinite(request.smart_fill_angle) || request.smart_fill_angle < 0. || request.smart_fill_angle > 90. ||
        !std::isfinite(request.overhangs_only) || request.overhangs_only < 0. || request.overhangs_only > 90. ||
        !std::isfinite(request.angle) || request.angle < 0. || request.angle > 90.)
        throw SceneEditError(422, "invalid_paint", "Paint angles must be between 0 and 90 degrees.");
    if (!std::isfinite(request.tolerance) || request.tolerance <= 0.)
        throw SceneEditError(422, "invalid_paint", "The paint surface tolerance must be finite and positive.");
    if (request.view_direction.has_value() &&
        (!request.view_direction->allFinite() || request.view_direction->norm() <= 1e-12))
        throw SceneEditError(422, "invalid_paint", "The view direction must be finite and non-zero.");
    if (request.tool == PaintTool::by_angle) {
        if (!request.strokes.empty())
            throw SceneEditError(422, "invalid_paint", "The by_angle tool does not accept strokes.");
    } else {
        if (request.strokes.empty())
            throw SceneEditError(422, "invalid_paint", "The paint request requires at least one stroke.");
    }

    std::size_t point_count = 0;
    for (const std::vector<PaintPoint> &stroke : request.strokes) {
        if (stroke.empty())
            throw SceneEditError(422, "invalid_paint", "Paint strokes must not be empty.");
        const bool ray = stroke.front().ray;
        for (const PaintPoint &point : stroke) {
            if (point.ray != ray)
                throw SceneEditError(422, "invalid_paint", "All points in one stroke must have the same form.");
            if (!point.position.allFinite() || (point.ray &&
                (!point.direction.allFinite() || point.direction.norm() <= 1e-12)))
                throw SceneEditError(422, "invalid_paint", "Paint coordinates and directions must be finite and valid.");
            ++point_count;
            if (point_count > 10000)
                throw SceneEditError(422, "invalid_paint", "A paint request may contain at most 10000 points.");
        }
    }
}

} // namespace

// Return fixed channels followed by copied generic definitions.
std::vector<PaintKind> paint_kinds()
{
    std::vector<PaintKind> result;
    result.push_back({"support", "Support painting", "Enforce supports", "Block supports", false});
    result.push_back({"mmu", "Multimaterial painting", {}, {}, true});
    for (const GenericFacetsAnnotationDefinition &definition : Orchestrator::instance().generic_facets_annotations())
        result.push_back({definition.key, definition.label, definition.enforce_label, definition.block_label, false});
    return result;
}

// Store a 422 point-resolution error with zero-based indices.
PaintPointError::PaintPointError(std::size_t stroke, std::size_t point, std::string reason)
    : SceneEditError(422, "invalid_paint", "Paint point " + std::to_string(stroke) + "/" +
        std::to_string(point) + " could not be resolved: " + reason),
      stroke_(stroke), point_(point), reason_(std::move(reason))
{
}

// Return the stored zero-based stroke index.
std::size_t PaintPointError::stroke() const noexcept
{
    return stroke_;
}

// Return the stored zero-based point index.
std::size_t PaintPointError::point() const noexcept
{
    return point_;
}

// Return the stored point-resolution reason.
const std::string &PaintPointError::reason() const noexcept
{
    return reason_;
}

// Validate enum and request values without mutating a scene.
void validate_paint_request(const PaintRequest &request, std::size_t extruder_count)
{
    bool valid_space = request.space == PaintSpace::platter || request.space == PaintSpace::object ||
        request.space == PaintSpace::volume;
    bool valid_tool = request.tool == PaintTool::circle || request.tool == PaintTool::sphere ||
        request.tool == PaintTool::triangle || request.tool == PaintTool::smart_fill ||
        request.tool == PaintTool::bucket_fill || request.tool == PaintTool::by_angle;
    if (!valid_space || !valid_tool)
        throw SceneEditError(422, "invalid_paint", "The paint space or tool is unknown.");
    validate_request_values(request, extruder_count);
}

// Paint a validated request and return pointers to changed volumes.
PaintOutcome paint(ModelObject &object, ModelVolume *part, std::size_t instance_index, const PaintRequest &request)
{
    const std::vector<std::unique_ptr<PaintTarget>> targets = make_targets(object, part, instance_index);
    const ModelInstance &reference = *object.instances[instance_index];
    std::vector<std::unique_ptr<TriangleSelector>> selectors;
    selectors.reserve(targets.size());
    for (const std::unique_ptr<PaintTarget> &target : targets) {
        std::unique_ptr<TriangleSelector> selector = std::make_unique<TriangleSelector>(target->volume->mesh());
        load_selector(*selector, *target->volume, request.kind);
        selectors.push_back(std::move(selector));
    }

    if (request.tool == PaintTool::by_angle)
        apply_by_angle(targets, selectors, request);
    else {
        const std::vector<std::vector<ResolvedPoint>> resolved = resolve_strokes(targets, reference, part, request);
        if (request.tool == PaintTool::circle || request.tool == PaintTool::sphere)
            apply_brush(resolved, targets, selectors, request);
        else
            apply_fill(resolved, targets, selectors, request);
    }

    PaintOutcome outcome;
    for (std::size_t index = 0; index < targets.size(); ++index)
        if (store_selector(*targets[index]->volume, request.kind, *selectors[index]))
            outcome.changed.push_back(targets[index]->volume);
    return outcome;
}

// Reset one or all non-empty channels and return changed volumes.
std::vector<ModelVolume *> clear_paint(ModelObject &object, ModelVolume *part, const std::string &kind)
{
    const std::vector<ModelVolume *> volumes = target_volumes(object, part);
    const bool clear_all = kind == "all";
    if (!clear_all)
        (void) channel_for_kind(kind);
    std::vector<ModelVolume *> changed;
    for (ModelVolume *volume : volumes) {
        bool volume_changed = false;
        if (clear_all || kind == "support") {
            if (!volume->supported_facets.empty()) {
                volume->supported_facets.reset();
                volume_changed = true;
            }
        }
        if (clear_all || kind == "mmu") {
            if (!volume->mm_segmentation_facets.empty()) {
                volume->mm_segmentation_facets.reset();
                volume_changed = true;
            }
        }
        if (clear_all) {
            const std::vector<std::string> keys = volume->facets_annotation_keys();
            for (const std::string &key : keys) {
                volume->reset_facets_annotation(key);
                volume_changed = true;
            }
        } else if (kind != "support" && kind != "mmu" && volume->has_facets_annotation(kind)) {
            volume->reset_facets_annotation(kind);
            volume_changed = true;
        }
        if (volume_changed)
            changed.push_back(volume);
    }
    return changed;
}

// Read strict painted triangles in the requested frame without scene mutation.
std::vector<PaintedState> painted_triangles(const ModelInstance &reference, const ModelVolume &part,
    const std::string &kind, PaintSpace space)
{
    const std::pair<std::vector<std::pair<int, int>>, std::vector<bool>> *data = annotation_data(part, kind);
    std::vector<PaintedState> result;
    if (data == nullptr || annotation_data_empty(*data))
        return result;
    TriangleSelector selector(part.mesh());
    selector.deserialize(*data);
    const Transform3d mapping = output_mapping(reference, part, space);
    const std::vector<EnforcerBlockerType> states = channel_states(kind);
    for (const EnforcerBlockerType state : states)
        if (selector.has_facets(state))
            result.push_back({state, transformed_triangles(selector, state, mapping)});
    return result;
}

} // namespace Slic3r::Rest
