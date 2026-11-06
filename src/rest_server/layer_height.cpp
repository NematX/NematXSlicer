/*
 * Variable layer-height profile calculations for REST requests.
 *
 * Operations use temporary slicing parameters from one solid object, flatten
 * and validate requested profiles locally, and resolve effective profiles with
 * the same engine helper used during slicing and in the GUI.
 */

#include "layer_height.hpp"

#include "libslic3r/PrintObject.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>

namespace Slic3r::Rest {

namespace {

// Carry one profile-wide failure reason and, when possible, its zero-based point index.
struct LayerHeightProfileIssue {
    std::optional<std::size_t> point;
    std::string reason;
};

// Return owned slicing parameters for the first instance; objects without a solid part throw 422.
std::shared_ptr<SlicingParameters> object_slicing_parameters(const ModelObject &object,
    const DynamicPrintConfig &session)
{
    bool has_solid_part = false;
    for (const ModelVolume *volume : object.volumes)
        if (volume->is_model_part()) {
            has_solid_part = true;
            break;
        }
    if (!has_solid_part)
        throw SceneEditError(422, "invalid_layer_height_profile", "The object has no solid part.");
    const float top = static_cast<float>(object.instance_bounding_box(0).max.z());
    // A non-positive top makes the engine read raw_bounding_box(), which writes a cache on the shared scene.
    const float object_max_z = top > 0.f ? top : static_cast<float>(object.instance_bounding_box(0, true).size().z());
    return PrintObject::slicing_parameters(session, object, object_max_z);
}

// Fill a local vector through the engine update helper and return it by value; stale storage may resolve to ranges without changing the object.
std::vector<double> effective_layer_height_profile(const ModelObject &object,
    const SlicingParameters &parameters)
{
    std::vector<double> effective;
    PrintObject::update_layer_height_profile(object, parameters, effective);
    return effective;
}

// Check flat Z/height pairs against slicing parameters and the supplied top tolerance; leave inputs unchanged and return an indexed/global issue.
std::optional<LayerHeightProfileIssue> validate_layer_height_profile(const std::vector<double> &profile,
    const SlicingParameters &parameters, double endpoint_tolerance)
{
    if (profile.size() % 2 != 0)
        return LayerHeightProfileIssue{std::nullopt, "invalid_length"};
    if (profile.size() < 4)
        return LayerHeightProfileIssue{std::nullopt, "too_few_points"};

    const std::size_t point_count = profile.size() / 2;
    const double object_height = parameters.object_print_z_height();
    const std::size_t last_z_index = profile.size() - 2;
    if (!std::isfinite(profile[last_z_index]) ||
        std::abs(profile[last_z_index] - object_height) > endpoint_tolerance)
        return LayerHeightProfileIssue{point_count - 1, "last_z"};

    if (!std::isfinite(profile.front()) || profile.front() != 0.)
        return LayerHeightProfileIssue{0, "first_z"};

    std::size_t repeated_z_count = 1;
    const double first_layer_height = parameters.first_object_layer_height;
    for (std::size_t point_index = 0; point_index < point_count; ++point_index) {
        const double z = profile[point_index * 2];
        const double height = profile[point_index * 2 + 1];
        if (!std::isfinite(z))
            return LayerHeightProfileIssue{point_index, "z_order"};
        if (!std::isfinite(height))
            return LayerHeightProfileIssue{point_index, "height_range"};
        if (point_index > 0) {
            const double previous_z = profile[(point_index - 1) * 2];
            if (z < previous_z)
                return LayerHeightProfileIssue{point_index, "z_order"};
            if (z == previous_z) {
                if (++repeated_z_count > 2)
                    return LayerHeightProfileIssue{point_index, "z_order"};
            } else {
                repeated_z_count = 1;
            }
        }

        // A point bypasses regular bounds only when its next point stays within the fixed first-layer Z.
        const bool first_layer_anchor = point_index + 1 < point_count &&
            profile[(point_index + 1) * 2] <= first_layer_height;
        if (!first_layer_anchor &&
            (height < parameters.min_layer_height - 1e-6 || height > parameters.max_layer_height + 1e-6))
            return LayerHeightProfileIssue{point_index, "height_range"};
    }
    return std::nullopt;
}

// Select stored data when present and otherwise validate the engine's range profile; range errors stay profile-wide.
std::optional<LayerHeightProfileIssue> validate_profile_source(const std::vector<double> &stored,
    const std::vector<double> &effective, const SlicingParameters &parameters)
{
    if (!stored.empty()) {
        if (std::optional<LayerHeightProfileIssue> issue =
                validate_layer_height_profile(stored, parameters, 10. * EPSILON))
            return issue;
        if (effective != stored)
            return LayerHeightProfileIssue{stored.size() / 2 - 1, "last_z"};
        return std::nullopt;
    }

    std::optional<LayerHeightProfileIssue> issue =
        validate_layer_height_profile(effective, parameters, 10. * EPSILON);
    if (issue)
        issue->point.reset();
    return issue;
}

// Raise point-specific 422 errors with indices and global errors without indices; callers retain no profile ownership through the exception.
void throw_layer_height_profile_issue(const LayerHeightProfileIssue &issue)
{
    if (issue.point)
        throw LayerHeightPointError(*issue.point, issue.reason);
    throw LayerHeightProfileError(issue.reason);
}

// Serialize one global or point-indexed validation error without exposing the profile storage vector.
nlohmann::json layer_height_profile_errors_json(const LayerHeightProfileIssue &issue)
{
    nlohmann::json error = {{"reason", issue.reason}};
    if (issue.point)
        error["point"] = *issue.point;
    return nlohmann::json::array({std::move(error)});
}

} // namespace

// Move an invalid zero-based point index and reason into a 422 profile exception; no scene state is owned or changed.
LayerHeightPointError::LayerHeightPointError(std::size_t point, std::string reason)
    : SceneEditError(422, "invalid_layer_height_profile", "Layer height profile point " + std::to_string(point) +
        " is invalid: " + reason),
      point_(point), reason_(std::move(reason))
{
}

// Return the stored zero-based point index without throwing or transferring ownership.
std::size_t LayerHeightPointError::point() const noexcept
{
    return point_;
}

// Return a borrowed reason reference that remains valid for this exception's lifetime.
const std::string &LayerHeightPointError::reason() const noexcept
{
    return reason_;
}

// Own a global profile-validation reason for a 422 response; the scene and profile remain unchanged.
LayerHeightProfileError::LayerHeightProfileError(std::string reason)
    : SceneEditError(422, "invalid_layer_height_profile", "Layer height profile is invalid: " + reason),
      reason_(std::move(reason))
{
}

// Return a borrowed reason reference that remains valid for this exception's lifetime.
const std::string &LayerHeightProfileError::reason() const noexcept
{
    return reason_;
}

// Validate one borrowed object/config/request, normalize accepted explicit tops, and use engine generators; return owned values without changing the object or throw 400/422 errors.
std::vector<double> compute_layer_height_profile(const ModelObject &object,
    const DynamicPrintConfig &session, const LayerHeightRequest &request)
{
    const std::shared_ptr<SlicingParameters> parameters = object_slicing_parameters(object, session);

    if (request.profile.has_value()) {
        std::vector<double> flat;
        flat.reserve(request.profile->size() * 2);
        for (const std::pair<double, double> &point : request.profile.value()) {
            flat.push_back(point.first);
            flat.push_back(point.second);
        }
        if (flat.size() >= 4) {
            const std::size_t last_z_index = flat.size() - 2;
            const double object_height = parameters->object_print_z_height();
            if (std::abs(flat[last_z_index] - object_height) <= 1e-3)
                flat[last_z_index] = object_height;
        }
        if (std::optional<LayerHeightProfileIssue> issue =
                validate_layer_height_profile(flat, *parameters, 1e-3)) {
            if (issue->reason == "too_few_points")
                throw SceneEditError(422, "invalid_layer_height_profile",
                    "An explicit layer height profile needs at least two points (too_few_points).");
            throw_layer_height_profile_issue(issue.value());
        }
        return flat;
    }

    if (request.adaptive.has_value()) {
        const LayerHeightAdaptiveRequest &requested = request.adaptive.value();
        if (!std::isfinite(requested.quality) || requested.quality < 0. || requested.quality > 1.)
            throw SceneEditError(422, "invalid_layer_height_profile", "Adaptive quality must be between 0 and 1.");
        double min_height = requested.min_height.value_or(parameters->min_layer_height);
        double max_height = requested.max_height.value_or(parameters->max_layer_height);
        if (!std::isfinite(min_height) || !std::isfinite(max_height) || min_height > max_height ||
            min_height < parameters->min_layer_height || max_height > parameters->max_layer_height)
            throw SceneEditError(422, "invalid_layer_height_profile", "Adaptive heights must be ordered within printer bounds.");
        min_height = check_z_step(min_height, parameters->z_step);
        max_height = check_z_step(max_height, parameters->z_step);
        min_height = std::clamp(min_height, parameters->min_layer_height, parameters->max_layer_height);
        max_height = std::clamp(max_height, parameters->min_layer_height, parameters->max_layer_height);
        const HeightProfileAdaptiveParams adaptive(static_cast<float>(requested.quality),
            static_cast<float>(min_height), static_cast<float>(max_height));
        std::vector<double> generated = layer_height_profile_adaptive(*parameters, object, adaptive);
        if (std::optional<LayerHeightProfileIssue> issue =
                validate_layer_height_profile(generated, *parameters, 10. * EPSILON))
            throw LayerHeightProfileError(issue->reason);
        return generated;
    }

    if (request.smooth.has_value()) {
        const HeightProfileSmoothingParams &smoothing = request.smooth.value();
        if (smoothing.radius == 0 || smoothing.radius > 10)
            throw SceneEditError(422, "invalid_layer_height_profile", "Smoothing radius must be between 1 and 10.");
        const std::vector<double> stored = object.layer_height_profile.get();
        std::vector<double> effective = effective_layer_height_profile(object, *parameters);
        if (std::optional<LayerHeightProfileIssue> issue = validate_profile_source(stored, effective, *parameters))
            throw_layer_height_profile_issue(issue.value());
        std::vector<double> smoothed = smooth_height_profile(effective, *parameters, smoothing);
        if (std::optional<LayerHeightProfileIssue> issue =
                validate_layer_height_profile(smoothed, *parameters, 10. * EPSILON))
            throw LayerHeightProfileError(issue->reason);
        return smoothed;
    }

    if (request.reset)
        return {};
    throw SceneEditError(400, "invalid_request", "Exactly one layer height operation is required.");
}

// Validate stored or range-derived points and report engine retention; return owning JSON without mutating inputs, omit layers for invalid data, or throw for missing solids.
nlohmann::json layer_height_json(const ModelObject &object, const DynamicPrintConfig &session)
{
    const std::shared_ptr<SlicingParameters> parameters = object_slicing_parameters(object, session);
    const std::vector<double> stored = object.layer_height_profile.get();

    nlohmann::json profile = nlohmann::json::array();
    for (std::size_t index = 0; index + 1 < stored.size(); index += 2)
        profile.push_back({stored[index], stored[index + 1]});

    const std::vector<double> effective = effective_layer_height_profile(object, *parameters);
    const std::optional<LayerHeightProfileIssue> issue = validate_profile_source(stored, effective, *parameters);
    const bool applied = !stored.empty() && effective == stored;

    nlohmann::json result = {
        {"profile", std::move(profile)},
        {"object_height", parameters->object_print_z_height()},
        {"min_layer_height", parameters->min_layer_height},
        {"max_layer_height", parameters->max_layer_height}
    };
    if (issue) {
        result["valid"] = false;
        result["applied"] = applied;
        result["errors"] = layer_height_profile_errors_json(issue.value());
        return result;
    }

    const std::vector<double> layer_pairs = generate_object_layers(*parameters, effective);
    nlohmann::json layers = nlohmann::json::array();
    for (std::size_t index = 1; index < layer_pairs.size(); index += 2)
        layers.push_back(layer_pairs[index]);
    result["valid"] = true;
    result["applied"] = applied;
    result["layers"] = std::move(layers);
    return result;
}

} // namespace Slic3r::Rest
