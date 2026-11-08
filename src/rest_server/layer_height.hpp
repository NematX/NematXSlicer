/*
 * Headless variable layer-height profile operations.
 *
 * ModelObject stores profiles as flat Z/height pairs. This module validates
 * requested profiles, asks the slicing engine for adaptive/smoothed values,
 * and serializes the profile and effective object layers without owning state.
 */
#pragma once

#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Slicing.hpp"
#include "scene_edit.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

namespace Slic3r::Rest {

// Carry adaptive bounds in double precision until printer validation and Z-step normalization finish.
struct LayerHeightAdaptiveRequest {
    double quality = 0.5;
    std::optional<double> min_height;
    std::optional<double> max_height;
};

// Describe one explicit, adaptive, smoothing, or reset operation without changing scene state.
struct LayerHeightRequest {
    std::optional<std::vector<std::pair<double, double>>> profile;
    std::optional<LayerHeightAdaptiveRequest> adaptive;
    std::optional<HeightProfileSmoothingParams> smooth;
    bool reset = false;
};

// Report an invalid profile point with its index and stable reason.
class LayerHeightPointError final : public SceneEditError {
public:
    // Construct a 422 invalid_layer_height_profile error for one offending point.
    LayerHeightPointError(std::size_t point, std::string reason);
    // Return the zero-based point index carried by the error.
    std::size_t point() const noexcept;
    // Return a borrowed reference to the stable reason stored by the exception.
    const std::string &reason() const noexcept;

private:
    std::size_t point_;
    std::string reason_;
};

// Carry a profile-wide 422 reason when no individual point can identify the failure.
class LayerHeightProfileError final : public SceneEditError {
public:
    // Construct a 422 invalid_layer_height_profile error with a global reason.
    explicit LayerHeightProfileError(std::string reason);
    // Return a borrowed reference to the stable reason stored by the exception.
    const std::string &reason() const noexcept;

private:
    std::string reason_;
};

// Return owned profile values for one operation; invalid profile values throw 422, an empty operation throws 400, and the object is unchanged.
std::vector<double> compute_layer_height_profile(const ModelObject &object,
    const DynamicPrintConfig &session, const LayerHeightRequest &request);

// Return owning validity/profile/layer JSON; invalid selected profiles omit layers and objects without solids throw 422.
nlohmann::json layer_height_json(const ModelObject &object, const DynamicPrintConfig &session);

} // namespace Slic3r::Rest
