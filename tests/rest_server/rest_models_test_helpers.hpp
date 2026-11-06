// Shared helpers keep model uploads and slicing tests on the same real request pipeline.

#pragma once

#include <catch2/catch.hpp>
#include <nlohmann/json.hpp>

#include "engine.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "rest_server_fixture.hpp"
#include "session.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <future>
#include <iterator>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace RestServerTest {

inline const std::string test_boundary = "restBoundary7";

inline void ensure_engine_initialized()
{
    Slic3r::Rest::Engine &engine = Slic3r::Rest::Engine::instance();
    if (engine.initialized())
        return;

    static TempDirectory engine_root;
    Slic3r::Rest::EnginePaths paths;
    paths.resource_directory = SLIC3R_REST_RESOURCE_DIRECTORY;
    paths.data_directory = engine_root.path() / "engine_data";
    engine.initialize(paths);
}

inline std::string create_session(RestServerFixture &fixture)
{
    const HttpResponseMessage created = fixture.request(http::verb::post, "/create_session");
    REQUIRE(created.result() == http::status::created);
    return nlohmann::json::parse(created.body()).at("session_id").get<std::string>();
}

// Controls how a multipart request is fragmented on the wire.
struct UploadOptions {
    bool chunked = false;
    std::size_t fragment_size = 7;
    std::size_t body_bytes = std::string::npos;
    std::optional<std::string> token;
    std::chrono::milliseconds fragment_delay{0};
};

inline std::string make_part(const std::string &disposition, const std::string &file)
{
    return "--" + test_boundary + "\r\n"
        "Content-Disposition: " + disposition + "\r\n"
        "Content-Type: application/octet-stream\r\n\r\n" + file + "\r\n";
}

inline std::string make_multipart_body(const std::string &filename, const std::string &file)
{
    return make_part(
        "form-data; name=\"file\"; filename=\"" + filename + "\"",
        file) + "--" + test_boundary + "--\r\n";
}

// Writes small fragments so multipart boundary handling is exercised.
inline HttpResponseMessage send_multipart(
    const RestServerFixture &fixture,
    const std::string &target,
    const std::string &body,
    const UploadOptions &options = {})
{
    asio::io_context context;
    Tcp::socket socket = fixture.connect_socket(context);
    const std::string token = options.token.value_or(fixture.token());
    std::ostringstream header;
    header << "POST " << target << " HTTP/1.1\r\n"
           << "Host: 127.0.0.1\r\n"
           << "User-Agent: rest_model_tests\r\n";
    if (!token.empty())
        header << "Authorization: Bearer " << token << "\r\n";
    header << "Content-Type: multipart/form-data; boundary=" << test_boundary << "\r\n";
    if (options.chunked)
        header << "Transfer-Encoding: chunked\r\n";
    else
        header << "Content-Length: " << body.size() << "\r\n";
    header << "Connection: close\r\n\r\n";
    asio::write(socket, asio::buffer(header.str()));

    const std::size_t step = std::max<std::size_t>(options.fragment_size, 1);
    const std::size_t bytes_to_send = std::min(options.body_bytes, body.size());
    const bool complete_body = bytes_to_send == body.size();
    try {
        for (std::size_t offset = 0; offset < bytes_to_send; offset += step) {
            if (options.fragment_delay.count() > 0)
                std::this_thread::sleep_for(options.fragment_delay);
            const std::size_t size = std::min(step, bytes_to_send - offset);
            const std::string fragment = body.substr(offset, size);
            if (options.chunked) {
                std::ostringstream chunk_header;
                chunk_header << std::hex << fragment.size() << "\r\n";
                asio::write(socket, asio::buffer(chunk_header.str()));
                asio::write(socket, asio::buffer(fragment));
                asio::write(socket, asio::buffer(std::string("\r\n")));
            } else {
                asio::write(socket, asio::buffer(fragment));
            }
        }
        if (options.chunked && complete_body)
            asio::write(socket, asio::buffer(std::string("0\r\n\r\n")));
    } catch (const boost::system::system_error &) {
    }
    if (!complete_body) {
        ErrorCode ignored;
        socket.shutdown(Tcp::socket::shutdown_send, ignored);
    }

    beast::flat_buffer buffer;
    HttpResponseMessage response;
    http::read(socket, buffer, response);
    ErrorCode ignored;
    socket.shutdown(Tcp::socket::shutdown_both, ignored);
    return response;
}

inline HttpResponseMessage upload_model(
    const RestServerFixture &fixture,
    const std::string &target,
    const std::string &filename,
    const std::string &file,
    const UploadOptions &options = {})
{
    return send_multipart(
        fixture,
        target,
        make_multipart_body(filename, file),
        options);
}

inline void require_error(
    const HttpResponseMessage &response,
    http::status status,
    const std::string &code,
    const std::string &message_fragment = {})
{
    REQUIRE(response.result() == status);
    const nlohmann::json error = nlohmann::json::parse(response.body()).at("error");
    REQUIRE(error.at("code") == code);
    REQUIRE(error.at("message").get<std::string>().find(message_fragment) != std::string::npos);
}

inline std::vector<std::string> object_ids(const HttpResponseMessage &response)
{
    REQUIRE(response.result() == http::status::ok);
    const nlohmann::json body = nlohmann::json::parse(response.body());
    const std::vector<std::string> ids = body.at("object_ids").get<std::vector<std::string>>();
    REQUIRE(body.at("object_count") == ids.size());
    return ids;
}

inline std::string read_file(const std::filesystem::path &path)
{
    std::ifstream input(path, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

inline std::string make_stl()
{
    TempDirectory directory;
    const std::filesystem::path path = directory.path() / "triangle.stl";
    const std::vector<Slic3r::Vec3f> vertices = {
        Slic3r::Vec3f(0.f, 0.f, 0.f),
        Slic3r::Vec3f(20.f, 0.f, 0.f),
        Slic3r::Vec3f(0.f, 20.f, 0.f),
        Slic3r::Vec3f(0.f, 0.f, 20.f)
    };
    const std::vector<Slic3r::Vec3i32> faces = {
        Slic3r::Vec3i32(0, 2, 1),
        Slic3r::Vec3i32(0, 1, 3),
        Slic3r::Vec3i32(0, 3, 2),
        Slic3r::Vec3i32(1, 2, 3)
    };
    Slic3r::TriangleMesh mesh(vertices, faces);
    REQUIRE(mesh.write_binary(path.string().c_str()));
    return read_file(path);
}

inline std::string make_cube_stl()
{
    TempDirectory directory;
    const std::filesystem::path path = directory.path() / "cube.stl";
    Slic3r::TriangleMesh mesh = Slic3r::make_cube(20.0, 20.0, 20.0);
    REQUIRE(mesh.write_binary(path.string().c_str()));
    return read_file(path);
}

inline std::shared_ptr<Slic3r::Rest::SessionContext> find_session(
    RestServerFixture &fixture,
    const std::string &session_id)
{
    return fixture.sessions().find(
        Slic3r::Rest::normalize_username("Alice"),
        session_id,
        Slic3r::Rest::SessionRegistry::Clock::now());
}

} // namespace RestServerTest
