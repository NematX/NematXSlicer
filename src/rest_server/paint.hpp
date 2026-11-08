/*
 * Headless facet painting for solid model parts.
 *
 * Requests are resolved in platter coordinates, while TriangleSelector keeps
 * the stored annotations in each volume's mesh coordinates.  The module owns
 * no scene state: callers provide a private Model copy for mutations and an
 * immutable instance/part pair for painted-triangle reads.
 */
#pragma once

#include "libslic3r/AABBMesh.hpp"
#include "libslic3r/Model.hpp"
#include "scene_edit.hpp"

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace Slic3r::Rest {

// Describe one support, MMU, or generic facet-annotation channel.
struct PaintKind {
    std::string id;
    std::string label;
    std::string enforce_label;
    std::string block_label;
    bool extruder_states = false;
};

// Return the fixed channels followed by the active generic annotation definitions.
std::vector<PaintKind> paint_kinds();

// Identify the coordinate frame in which request points and directions are supplied.
enum class PaintSpace {
    platter,
    object,
    volume
};

// Select the facet operation applied to each resolved point or stroke.
enum class PaintTool {
    circle,
    sphere,
    triangle,
    smart_fill,
    bucket_fill,
    by_angle
};

// Hold either a surface point or a ray origin and direction from the request.
struct PaintPoint {
    bool ray = false;
    Vec3d position;
    Vec3d direction;
};

// Hold all validated options needed to paint one channel of one target.
struct PaintRequest {
    std::string kind;
    EnforcerBlockerType state = EnforcerBlockerType::NONE;
    PaintSpace space = PaintSpace::platter;
    PaintTool tool = PaintTool::circle;
    double radius = 2.;
    std::optional<Vec3d> view_direction;
    std::vector<std::vector<PaintPoint>> strokes;
    bool split_triangles = true;
    double overhangs_only = 0.;
    double smart_fill_angle = 30.;
    double angle = 0.;
    double tolerance = 0.05;
};

// Report a point-specific surface or ray resolution failure with its indices.
class PaintPointError final : public SceneEditError {
public:
    // Construct a 422 invalid_paint error for one stroke point and its reason.
    PaintPointError(std::size_t stroke, std::size_t point, std::string reason);
    // Return the zero-based stroke index carried by the error.
    std::size_t stroke() const noexcept;
    // Return the zero-based point index carried by the error.
    std::size_t point() const noexcept;
    // Return the stable resolution reason such as off_surface or no_hit.
    const std::string &reason() const noexcept;

private:
    std::size_t stroke_;
    std::size_t point_;
    std::string reason_;
};

// One targeted solid part and the transforms/tree used while painting it.
struct PaintTarget {
    ModelVolume *volume = nullptr;
    Transform3d trafo = Transform3d::Identity();
    Transform3d trafo_no_translate = Transform3d::Identity();
    indexed_triangle_set platter;
    std::unique_ptr<AABBMesh> tree;
};

// Describe volume IDs whose stored annotation changed during one operation.
struct PaintOutcome {
    std::vector<ModelVolume *> changed;
};

// Validate request-only values before a scene mutation is started.
void validate_paint_request(const PaintRequest &request, std::size_t extruder_count);

// Paint the selected object parts or one selected solid part in a private scene.
PaintOutcome paint(ModelObject &object, ModelVolume *part, std::size_t instance_index,
    const PaintRequest &request);

// Clear one channel or every present channel on selected object parts.
std::vector<ModelVolume *> clear_paint(ModelObject &object, ModelVolume *part, const std::string &kind);

// Return strict painted triangles in the requested output coordinate frame.
struct PaintedState {
    EnforcerBlockerType state = EnforcerBlockerType::NONE;
    indexed_triangle_set triangles;
};

// Read painted states from one solid part using the supplied reference instance.
std::vector<PaintedState> painted_triangles(const ModelInstance &reference, const ModelVolume &part,
    const std::string &kind, PaintSpace space);

} // namespace Slic3r::Rest
