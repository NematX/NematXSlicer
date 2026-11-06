// Affine instance and volume operations use explicit spaces without additional GUI pivots.
#pragma once

#include <optional>
#include <stdexcept>
#include <string>
#include <nlohmann/json_fwd.hpp>
#include "libslic3r/Point.hpp"

namespace Slic3r::Rest {

class TransformError final : public std::invalid_argument {
public:
    TransformError(unsigned int status, const std::string &message);
    unsigned int status() const noexcept { return status_; }
private:
    unsigned int status_;
};

struct TransformRequest {
    Transform3d matrix = Transform3d::Identity();
    std::string action = "compose";
    std::string space;
    bool ensure_on_bed = false;
    std::optional<std::string> instance;
};

TransformRequest parse_transform_request(const nlohmann::json &body);
// Validate the result, since composing non-uniform scales and rotations can introduce skew.
void validate_result(const Transform3d &matrix);
Transform3d instance_matrix(const Transform3d &current, const TransformRequest &request);
// Relative platter operations use GUI axes; absolute replacement uses the full instance frame.
Transform3d volume_matrix(const Transform3d &current, const Transform3d &reference, const TransformRequest &request);

} // namespace Slic3r::Rest
