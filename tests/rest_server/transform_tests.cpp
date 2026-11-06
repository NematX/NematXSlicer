// Pure transform tests distinguish coordinate bases and validate composed results.
#include <catch2/catch.hpp>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>
#include <nlohmann/json.hpp>
#include "transform.hpp"

using namespace Slic3r;
using namespace Slic3r::Rest;

TEST_CASE("Transform request parsing checks shape types and affine values", "[rest_transform]")
{
    const nlohmann::json valid{
        {"space", "platter"},
        {"matrix", {1,0,0,5,0,1,0,0,0,0,1,0,0,0,0,1}}
    };
    const TransformRequest parsed = parse_transform_request(valid);
    CHECK(parsed.action == "compose");
    CHECK_FALSE(parsed.ensure_on_bed);
    CHECK_FALSE(parsed.instance.has_value());
    CHECK(parsed.matrix.translation().x() == 5);
    nlohmann::json with_instance = valid;
    with_instance["instance"] = "reference-id";
    CHECK(parse_transform_request(with_instance).instance.value() == "reference-id");
    const std::vector<std::pair<nlohmann::json, unsigned int>> invalid = {
        {nlohmann::json::array(), 400},
        {{{"matrix", valid.at("matrix")}}, 400},
        {{{"space", "platter"}}, 400},
        {{{"space", 7}, {"matrix", valid.at("matrix")}}, 400},
        {{{"space", "platter"}, {"matrix", 7}}, 400},
        {{{"space", "platter"}, {"matrix", valid.at("matrix")}, {"ensure_on_bed", 1}}, 400},
        {{{"space", "platter"}, {"matrix", valid.at("matrix")}, {"action", true}}, 400},
        {{{"space", "platter"}, {"matrix", valid.at("matrix")}, {"instance", 7}}, 400},
        {{{"space", "platter"}, {"matrix", valid.at("matrix")}, {"instance", nullptr}}, 400},
        {{{"space", "platter"}, {"matrix", valid.at("matrix")}, {"pivot", {1,2,3}}}, 400},
        {{{"space", "unknown"}, {"matrix", valid.at("matrix")}}, 422},
        {{{"space", "platter"}, {"matrix", valid.at("matrix")}, {"action", "unknown"}}, 422},
        {{{"space", "platter"}, {"matrix", {1,2,3}}}, 422}
    };
    for (const std::pair<nlohmann::json, unsigned int> &test : invalid) {
        INFO(test.first.dump());
        try {
            parse_transform_request(test.first);
            FAIL("Invalid transform accepted");
        } catch (const TransformError &error) {
            CHECK(error.status() == test.second);
        }
    }
    for (const nlohmann::json &value : {nlohmann::json("text"), nlohmann::json(nullptr),
             nlohmann::json(std::numeric_limits<double>::infinity()),
             nlohmann::json(std::numeric_limits<double>::quiet_NaN())}) {
        nlohmann::json body = valid;
        body["matrix"][0] = value;
        REQUIRE_THROWS_AS(parse_transform_request(body), TransformError);
    }
    nlohmann::json projective = valid;
    projective["matrix"][12] = 1;
    CHECK_THROWS_AS(parse_transform_request(projective), TransformError);
}

TEST_CASE("Instance transforms use global or local composition without implicit pivots", "[rest_transform]")
{
    Transform3d current = Transform3d::Identity();
    current.linear() << 0,-1,0, 1,0,0, 0,0,1;
    current.translation() = Vec3d(10,20,3);
    TransformRequest request;
    request.matrix.translation().x() = 5;
    request.space = "platter";
    CHECK((instance_matrix(current, request) * Vec3d(1,0,0)).isApprox(Vec3d(15,21,3)));
    for (const std::string &space : {"object", "volume"}) {
        request.space = space;
        CHECK((instance_matrix(current, request) * Vec3d(1,0,0)).isApprox(Vec3d(10,26,3)));
    }
    request.action = "replace";
    for (const std::string &space : {"platter", "object", "volume"}) {
        request.space = space;
        CHECK((instance_matrix(current, request) * Vec3d(1,0,0)).isApprox(Vec3d(6,0,0)));
    }
}

TEST_CASE("Transform validation rejects singular and composed shear but accepts mirrors and scales", "[rest_transform]")
{
    Transform3d matrix = Transform3d::Identity();
    matrix.linear()(0,0) = 0;
    CHECK_THROWS_AS(validate_result(matrix), TransformError);
    matrix.linear()(0,0) = 1e-10;
    CHECK_THROWS_AS(validate_result(matrix), TransformError);
    matrix.linear() << -1,0,0, 0,2,0, 0,0,3;
    CHECK_NOTHROW(validate_result(matrix));
    matrix.linear()(0,1) = 0.5;
    CHECK_THROWS_AS(validate_result(matrix), TransformError);

    Transform3d current = Transform3d::Identity();
    current.linear()(0,0) = 2;
    TransformRequest request;
    const double diagonal = std::sqrt(0.5);
    request.matrix.linear() << diagonal,-diagonal,0, diagonal,diagonal,0, 0,0,1;
    CHECK_NOTHROW(validate_result(request.matrix));
    request.space = "object";
    CHECK_THROWS_AS(instance_matrix(current, request), TransformError);
    request.space = "platter";
    CHECK_NOTHROW(instance_matrix(current, request));
}

TEST_CASE("Transform validation accepts mirrors with arbitrary 3D rotations", "[rest_transform][rest_mirror_3d]")
{
    const Matrix3d rotation = Eigen::AngleAxisd(0.7, Vec3d(1,2,3).normalized()).toRotationMatrix();
    Matrix3d mirror = Matrix3d::Identity();
    mirror(0,0) = -1;
    Transform3d matrix = Transform3d::Identity();
    matrix.linear() = rotation * mirror;
    CHECK((matrix.linear().transpose() * matrix.linear()).isApprox(Matrix3d::Identity()));
    CHECK(matrix.linear().determinant() == Approx(-1));
    CHECK_NOTHROW(validate_result(matrix));
    matrix.linear() = mirror * rotation;
    CHECK_NOTHROW(validate_result(matrix));
    Matrix3d scale = Matrix3d::Identity();
    scale.diagonal() << 2,3,0.5;
    matrix.linear() = rotation * mirror * scale;
    CHECK_NOTHROW(validate_result(matrix));
    matrix.linear() = 2 * rotation * mirror;
    CHECK_NOTHROW(validate_result(matrix));
    matrix.linear() = scale * rotation;
    CHECK_THROWS_AS(validate_result(matrix), TransformError);
}

TEST_CASE("Transform orthogonality checks are relative to each column scale", "[rest_transform]")
{
    Transform3d matrix = Transform3d::Identity();
    matrix.linear().diagonal() << 1e200,1e-100,1e-100;
    CHECK(matrix.linear().determinant() == Approx(1));
    CHECK_NOTHROW(validate_result(matrix));
    matrix.linear()(0,1) = 1e-100;
    CHECK_THROWS_AS(validate_result(matrix), TransformError);
    matrix = Transform3d::Identity();
    matrix.linear()(0,1) = 0.5e-9;
    CHECK_NOTHROW(validate_result(matrix));
    matrix.linear()(0,1) = 2e-9;
    CHECK_THROWS_AS(validate_result(matrix), TransformError);
}

TEST_CASE("Volume composition uses GUI platter axes and distinct parent and local spaces", "[rest_transform][rest_volume_transform]")
{
    Transform3d volume = Transform3d::Identity();
    volume.linear() << 1,0,0, 0,0,-1, 0,1,0;
    volume.translation() = Vec3d(1,2,3);
    Transform3d reference = Transform3d::Identity();
    reference.linear() << 0,-1,0, 1,0,0, 0,0,1;
    reference.translation() = Vec3d(10,20,30);
    TransformRequest request;
    request.matrix.linear() << 0,0,1, 0,1,0, -1,0,0;
    request.matrix.translation() = Vec3d(4,5,6);
    const Vec3d point(1,2,3);
    request.space = "object";
    CHECK((volume_matrix(volume, reference, request) * point).isApprox(Vec3d(9,4,4)));
    request.space = "volume";
    CHECK((volume_matrix(volume, reference, request) * point).isApprox(Vec3d(8,-3,10)));
    request.space = "platter";
    const Transform3d global = volume_matrix(volume, reference, request);
    CHECK((global * point).isApprox(Vec3d(7,-9,5)));
    CHECK((reference * global * point).isApprox(Vec3d(19,27,35)));
    Transform3d other_offset = reference;
    other_offset.translation() = Vec3d(-8,4,12);
    CHECK(volume_matrix(volume, other_offset, request).matrix().isApprox(global.matrix()));

    request.action = "replace";
    for (const std::string &space : {"object", "volume"}) {
        request.space = space;
        CHECK((volume_matrix(volume, reference, request) * point).isApprox(Vec3d(7,7,5)));
    }
    request.space = "platter";
    CHECK((reference * volume_matrix(volume, reference, request) * point).isApprox(Vec3d(7,7,5)));
    request.matrix.linear()(0,2) = 0;
    CHECK_THROWS_AS(volume_matrix(volume, reference, request), TransformError);
    request.matrix = Transform3d::Identity();
    reference.linear()(0,0) = 0;
    reference.linear()(1,0) = 0;
    CHECK_THROWS_AS(volume_matrix(volume, reference, request), TransformError);
}
