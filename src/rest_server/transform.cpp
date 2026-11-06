// Parse and compose affine matrices independently of HTTP and scene ownership.
#include "transform.hpp"

#include <cmath>
#include <nlohmann/json.hpp>

namespace Slic3r::Rest {

TransformError::TransformError(unsigned int status, const std::string &message)
    : std::invalid_argument(message), status_(status)
{
}

TransformRequest parse_transform_request(const nlohmann::json &body)
{
    if (!body.is_object())
        throw TransformError(400, "The transform body must be an object.");
    for (const auto &item : body.items())
        if (item.key() != "matrix" && item.key() != "action" && item.key() != "space" &&
            item.key() != "ensure_on_bed" && item.key() != "instance")
            throw TransformError(400, "Unknown transform field: " + item.key());
    if (!body.contains("matrix") || !body.at("matrix").is_array() ||
        !body.contains("space") || !body.at("space").is_string() ||
        (body.contains("action") && !body.at("action").is_string()) ||
        (body.contains("ensure_on_bed") && !body.at("ensure_on_bed").is_boolean()) ||
        (body.contains("instance") && !body.at("instance").is_string()))
        throw TransformError(400, "The transform fields have missing or invalid types.");

    TransformRequest request;
    request.space = body.at("space").get<std::string>();
    if (body.contains("action"))
        request.action = body.at("action").get<std::string>();
    if (body.contains("ensure_on_bed"))
        request.ensure_on_bed = body.at("ensure_on_bed").get<bool>();
    if (body.contains("instance"))
        request.instance = body.at("instance").get<std::string>();
    if (request.action != "compose" && request.action != "replace")
        throw TransformError(422, "The transform action must be compose or replace.");
    if (request.space != "platter" && request.space != "object" && request.space != "volume")
        throw TransformError(422, "The transform space must be platter, object or volume.");
    const nlohmann::json &matrix = body.at("matrix");
    if (matrix.size() != 16)
        throw TransformError(422, "The transform matrix must contain 16 finite numbers.");
    for (int row = 0; row < 4; ++row)
        for (int column = 0; column < 4; ++column) {
            const nlohmann::json &value = matrix.at(row * 4 + column);
            if (!value.is_number() || !std::isfinite(value.get<double>()))
                throw TransformError(422, "The transform matrix must contain 16 finite numbers.");
            request.matrix.matrix()(row, column) = value.get<double>();
        }
    if (request.matrix.matrix()(3, 0) != 0 || request.matrix.matrix()(3, 1) != 0 ||
        request.matrix.matrix()(3, 2) != 0 || request.matrix.matrix()(3, 3) != 1)
        throw TransformError(422, "The transform matrix must have the last row [0,0,0,1].");
    return request;
}

void validate_result(const Transform3d &matrix)
{
    const double determinant = matrix.linear().determinant();
    if (!matrix.matrix().allFinite() || !std::isfinite(determinant) || std::abs(determinant) <= 1e-9)
        throw TransformError(422, "The resulting transform must be finite and invertible.");
    // Column orthogonality avoids the engine heuristic's false positives for rotated mirrors.
    Matrix3d directions = matrix.linear();
    for (int column = 0; column < 3; ++column) {
        // Bound the coefficients before computing a norm to avoid overflow or underflow.
        directions.col(column) /= directions.col(column).cwiseAbs().maxCoeff();
        directions.col(column).normalize();
    }
    for (int first = 0; first < 3; ++first)
        for (int second = first + 1; second < 3; ++second)
            if (std::abs(directions.col(first).dot(directions.col(second))) > 1e-9)
                throw TransformError(422, "The resulting transform must not contain shear.");
}

Transform3d instance_matrix(const Transform3d &current, const TransformRequest &request)
{
    Transform3d result;
    if (request.action == "replace")
        result = request.matrix;
    else if (request.space == "platter")
        result = request.matrix * current;
    else
        result = current * request.matrix;
    validate_result(result);
    return result;
}

Transform3d volume_matrix(const Transform3d &current, const Transform3d &reference, const TransformRequest &request)
{
    Transform3d result;
    if (request.action == "replace") {
        if (request.space == "platter")
            result = reference.inverse() * request.matrix;
        else
            result = request.matrix;
    } else if (request.space == "object") {
        result = request.matrix * current;
    } else if (request.space == "volume") {
        result = current * request.matrix;
    } else {
        // GUI platter composition changes axes without conjugating by the instance offset.
        Transform3d linear_reference = Transform3d::Identity();
        linear_reference.linear() = reference.linear();
        result = linear_reference.inverse() * request.matrix * linear_reference * current;
    }
    validate_result(result);
    return result;
}

} // namespace Slic3r::Rest
