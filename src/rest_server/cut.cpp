/*
 * Headless plane-cut implementation for REST scene edits.
 *
 * Plane coordinates are converted with the full relevant affine linear
 * transform, including inverse-transpose normal conversion for non-uniform
 * scales and mirrors. Object cuts clone libslic3r results into the candidate
 * scene; part cuts use a temporary one-volume object and replace only the
 * selected volume, so unrelated IDs, transforms, and metadata remain stable.
 */
#include "cut.hpp"

#include "libslic3r/CutUtils.hpp"
#include "libslic3r/Geometry.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <stdexcept>
#include <utility>

namespace Slic3r::Rest {

namespace {

constexpr double cut_epsilon = 1e-6;
constexpr double determinant_epsilon = 1e-12;

// Map request flags to engine attributes and optionally mark cut metadata invalid.
ModelObjectCutAttributes cut_attributes(const CutRequest &request, bool include_invalidate)
{
    ModelObjectCutAttributes attributes =
        only_if(request.keep_upper, ModelObjectCutAttribute::KeepUpper) |
        only_if(request.keep_lower, ModelObjectCutAttribute::KeepLower) |
        only_if(request.keep_as_parts, ModelObjectCutAttribute::KeepAsParts) |
        only_if(request.flip_upper, ModelObjectCutAttribute::FlipUpper) |
        only_if(request.flip_lower, ModelObjectCutAttribute::FlipLower) |
        only_if(request.place_on_cut_upper, ModelObjectCutAttribute::PlaceOnCutUpper) |
        only_if(request.place_on_cut_lower, ModelObjectCutAttribute::PlaceOnCutLower);
    if (include_invalidate)
        attributes = attributes | ModelObjectCutAttribute::InvalidateCutInfo;
    return attributes;
}

// Build the affine request-space mapping and reject unknown or singular transforms.
Transform3d space_mapping(const ModelInstance &reference, const ModelVolume *volume, CutSpace space)
{
    Transform3d mapping = Transform3d::Identity();
    if (space == CutSpace::object || space == CutSpace::volume)
        mapping = reference.get_matrix();
    if (space == CutSpace::volume && volume != nullptr)
        mapping = mapping * volume->get_matrix();
    if (space != CutSpace::platter && space != CutSpace::object && space != CutSpace::volume)
        throw SceneEditError(422, "invalid_cut", "The cut space is not supported.");
    if (std::abs(mapping.linear().determinant()) <= determinant_epsilon)
        throw SceneEditError(422, "invalid_cut", "The cut space transformation is singular.");
    return mapping;
}

// Translate a generated volume's cut-origin flag into the stable REST side label.
const char *part_side(const ModelVolume &volume)
{
    return volume.is_from_upper() ? "upper" : "lower";
}

} // namespace

// Validate finite plane data and request-only option combinations.
void validate_cut_request(const CutRequest &request)
{
    if (!request.point.allFinite() || !request.normal.allFinite())
        throw SceneEditError(422, "invalid_cut", "The cut point and normal must be finite.");
    if (request.normal.norm() <= 1e-12)
        throw SceneEditError(422, "invalid_cut", "The cut normal must be non-zero.");
    if (!request.keep_upper && !request.keep_lower)
        throw SceneEditError(422, "invalid_cut", "At least one cut side must be kept.");
    if (request.keep_as_parts && (!request.keep_upper || !request.keep_lower ||
        request.flip_upper || request.flip_lower || request.place_on_cut_upper || request.place_on_cut_lower))
        throw SceneEditError(422, "invalid_cut", "Keeping both sides as parts cannot use side pose options.");
}

// Convert the point directly and the normal with the mapping's inverse-transpose.
std::pair<Vec3d, Vec3d> plane_in_platter(const ModelInstance &reference,
    const ModelVolume *volume, const CutRequest &request)
{
    const Transform3d mapping = space_mapping(reference, volume, request.space);
    const Vec3d point = mapping * request.point;
    const Vec3d transformed_normal = mapping.linear().inverse().transpose() * request.normal;
    if (!point.allFinite() || !transformed_normal.allFinite() || transformed_normal.norm() <= 1e-12)
        throw SceneEditError(422, "invalid_cut", "The transformed cut plane is invalid.");
    return {point, transformed_normal.normalized()};
}

// Build the GUI-compatible cut frame, omitting the reference instance offset.
Transform3d cut_matrix_from_plane(const ModelInstance &reference,
    const Vec3d &point, const Vec3d &normal)
{
    if (!point.allFinite() || !normal.allFinite() || normal.norm() <= 1e-12)
        throw SceneEditError(422, "invalid_cut", "The cut plane must have finite point and non-zero normal.");
    const Vec3d normalized = normal.normalized();
    Transform3d rotation = Transform3d::Identity();
    if (normalized.dot(Vec3d::UnitZ()) < -1. + 1e-12)
        rotation = Geometry::rotation_transform(PI * Vec3d::UnitX());
    else
        rotation = Transform3d(Eigen::Quaterniond::FromTwoVectors(Vec3d::UnitZ(), normalized));
    return Geometry::translation_transform(point - reference.get_offset()) * rotation;
}

// Require target vertices on both strict sides of the cut plane.
void require_plane_crosses(const std::vector<const ModelVolume *> &solids,
    const ModelInstance &reference, const Transform3d &cut_matrix)
{
    const Transform3d to_cut = cut_matrix.inverse() * reference.get_matrix_no_offset();
    bool has_upper = false;
    bool has_lower = false;
    for (const ModelVolume *volume : solids) {
        if (volume == nullptr)
            continue;
        const Transform3d to_cut_volume = to_cut * volume->get_matrix();
        for (const Vec3f &vertex : volume->mesh().its.vertices) {
            const double z = (to_cut_volume * vertex.cast<double>()).z();
            has_upper = has_upper || z > cut_epsilon;
            has_lower = has_lower || z < -cut_epsilon;
            if (has_upper && has_lower)
                return;
        }
    }
    throw SceneEditError(422, "invalid_cut", "The plane does not cross the geometry.");
}

// Cut an object on the candidate scene, replace its entity bindings, and synchronize its cut group.
CutOutcome cut_object(Model &scene, ModelObject &object, const std::size_t instance_index,
    const CutRequest &request)
{
    validate_cut_request(request);
    if (instance_index >= object.instances.size())
        throw SceneEditError(422, "invalid_cut", "The selected instance is not in the object.");

    const ModelInstance &reference = *object.instances[instance_index];
    const std::pair<Vec3d, Vec3d> plane = plane_in_platter(reference, nullptr, request);
    const Transform3d cut_matrix = cut_matrix_from_plane(reference, plane.first, plane.second);
    std::vector<const ModelVolume *> solids;
    for (const ModelVolume *volume : object.volumes)
        if (volume->is_model_part())
            solids.push_back(volume);
    require_plane_crosses(solids, reference, cut_matrix);

    const ModelObjectCutAttributes attributes = cut_attributes(request, !object.cut_id.id().valid());
    update_object_cut_id(object.cut_id, attributes, 0);
    const CutObjectBase cut_id = object.cut_id;
    Cut cut(&object, static_cast<int>(instance_index), cut_matrix, attributes);
    const ModelObjectPtrs &results = cut.perform_with_plane();

    CutOutcome outcome;
    append_object_entities(object, outcome.change.unbindings);
    for (const ModelObject *result : results) {
        if (result == nullptr)
            throw std::logic_error("The plane cut returned a null object.");
        ModelObject *added = scene.add_object(*result);
        added->sort_volumes(true);
        added->ensure_on_bed();
        outcome.objects.push_back(added);
        append_object_entities(*added, outcome.change.bindings);
    }

    const std::size_t expected = request.keep_as_parts ? 1U :
        static_cast<std::size_t>(request.keep_upper) + static_cast<std::size_t>(request.keep_lower);
    if (outcome.objects.size() != expected)
        throw std::logic_error("The plane cut returned an unexpected number of objects.");
    if (request.keep_as_parts)
        outcome.sides.push_back("parts");
    else {
        if (request.keep_upper)
            outcome.sides.push_back("upper");
        if (request.keep_lower)
            outcome.sides.push_back("lower");
    }

    scene.delete_object(&object);
    synchronize_model_after_cut(scene, cut_id);
    return outcome;
}

// Cut one part through a temporary object, then replace only that part and bind its new IDs.
CutOutcome cut_part(ModelObject &object, ModelVolume &part, const std::size_t instance_index,
    const CutRequest &request)
{
    validate_cut_request(request);
    if (!part.is_model_part())
        throw SceneEditError(422, "invalid_cut", "Only solid model parts can be cut.");
    if (request.keep_as_parts || request.flip_upper || request.flip_lower ||
        request.place_on_cut_upper || request.place_on_cut_lower || request.arrange)
        throw SceneEditError(422, "invalid_cut", "The selected part does not support these cut options.");
    if (instance_index >= object.instances.size())
        throw SceneEditError(422, "invalid_cut", "The selected instance is not in the object.");
    if (object.is_cut()) {
        if (!request.invalidate_cut)
            throw SceneEditError(422, "cut_object", "Cut parts require cut invalidation.");
        invalidate_cut_group(object);
    }

    const ModelInstance &reference = *object.instances[instance_index];
    const std::pair<Vec3d, Vec3d> plane = plane_in_platter(reference, &part, request);
    const Transform3d cut_matrix = cut_matrix_from_plane(reference, plane.first, plane.second);
    require_plane_crosses({&part}, reference, cut_matrix);

    ModelObject *scratch = nullptr;
    object.clone_for_cut(&scratch);
    Model scratch_model;
    scratch_model.add_object(ModelObjectUPtr(scratch));
    scratch->add_volume(part);
    // This temporary cut has a new upper/lower orientation, independent of the
    // selected part's side in any older cut group.
    scratch->volumes.back()->reset_from_upper();

    ModelObjectCutAttributes attributes = only_if(request.keep_upper, ModelObjectCutAttribute::KeepUpper) |
        only_if(request.keep_lower, ModelObjectCutAttribute::KeepLower) |
        ModelObjectCutAttribute::KeepAsParts | ModelObjectCutAttribute::InvalidateCutInfo;
    if (!request.keep_upper && request.keep_lower)
        attributes = attributes | ModelObjectCutAttribute::KeepUpper;
    Cut cut(scratch, static_cast<int>(instance_index), cut_matrix, attributes);
    const ModelObjectPtrs &results = cut.perform_with_plane();
    if (results.size() != 1)
        throw std::logic_error("The part cut returned an unexpected number of objects.");
    const ModelObject *result = results.front();
    const Transform3d reference_matrix = object.instances[instance_index]->get_matrix();
    const Transform3d result_matrix = result->instances[instance_index]->get_matrix();
    const Transform3d source_to_object = reference_matrix.inverse() * result_matrix;

    const std::size_t original_index = static_cast<std::size_t>(
        std::find(object.volumes.begin(), object.volumes.end(), &part) - object.volumes.begin());
    if (original_index == object.volumes.size())
        throw std::logic_error("The part is not owned by its object.");

    CutOutcome outcome;
    outcome.change.unbindings.push_back({EntityKind::volume, part.id().id});
    std::vector<ModelVolume *> added;
    for (const ModelVolume *result_volume : result->volumes) {
        const bool upper = result_volume->is_from_upper();
        if ((upper && !request.keep_upper) || (!upper && !request.keep_lower))
            continue;
        ModelVolume *replacement = object.add_volume(*result_volume);
        replacement->set_transformation(Geometry::Transformation(source_to_object * result_volume->get_matrix()));
        added.push_back(replacement);
        outcome.parts.push_back(replacement);
        outcome.sides.emplace_back(part_side(*result_volume));
        outcome.change.bindings.push_back({EntityKind::volume, replacement->id().id});
    }
    if (added.empty())
        throw std::logic_error("The part cut returned no requested replacement.");

    const ModelVolumePtrs::iterator original = std::find(object.volumes.begin(), object.volumes.end(), &part);
    delete *original;
    object.volumes.erase(original);
    std::rotate(object.volumes.begin() + static_cast<std::ptrdiff_t>(original_index),
        object.volumes.end() - static_cast<std::ptrdiff_t>(added.size()), object.volumes.end());
    object.sort_volumes(true);
    object.invalidate_bounding_box();
    return outcome;
}

} // namespace Slic3r::Rest
