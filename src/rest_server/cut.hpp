/*
 * Plane cuts for REST scene edits.
 *
 * Requests describe a plane in platter, object, or volume coordinates. The
 * module converts that plane to the engine's platter-frame cut matrix, checks
 * that it crosses the requested solid geometry, and applies the existing
 * libslic3r cut operation to a private scene. HTTP code only consumes the
 * resulting scene-change bindings and side labels.
 */
#pragma once

#include <utility>
#include <vector>

#include "libslic3r/Model.hpp"
#include "libslic3r/Point.hpp"
#include "scene_edit.hpp"

namespace Slic3r::Rest {

enum class CutSpace {
    platter,
    object,
    volume
};

struct CutRequest {
    Vec3d point;
    Vec3d normal;
    CutSpace space = CutSpace::platter;
    bool keep_upper = true;
    bool keep_lower = true;
    bool keep_as_parts = false;
    bool flip_upper = false;
    bool flip_lower = false;
    bool place_on_cut_upper = false;
    bool place_on_cut_lower = false;
    bool invalidate_cut = false;
    bool arrange = false;
};

struct CutOutcome {
    SceneChange change;
    std::vector<ModelObject *> objects;
    std::vector<std::string> sides;
    std::vector<ModelVolume *> parts;
};

// Validate plane values and option combinations that do not require a scene target.
void validate_cut_request(const CutRequest &request);

// Convert a request plane from its selected local space to platter coordinates.
std::pair<Vec3d, Vec3d> plane_in_platter(const ModelInstance &reference,
    const ModelVolume *volume, const CutRequest &request);

// Build the engine cut frame from a platter-space plane and omit the instance offset.
Transform3d cut_matrix_from_plane(const ModelInstance &reference,
    const Vec3d &point, const Vec3d &normal);

// Reject a cut whose target solid vertices do not occur on both sides of the plane.
void require_plane_crosses(const std::vector<const ModelVolume *> &solids,
    const ModelInstance &reference, const Transform3d &cut_matrix);

// Cut one object instance, replacing the source object and returning its new objects.
CutOutcome cut_object(Model &scene, ModelObject &object, std::size_t instance_index,
    const CutRequest &request);

// Cut one solid part while retaining every other volume and instance unchanged.
CutOutcome cut_part(ModelObject &object, ModelVolume &part, std::size_t instance_index,
    const CutRequest &request);

} // namespace Slic3r::Rest
