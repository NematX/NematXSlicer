/*
 * Multipart integration tests for REST model and project uploads.
 *
 * The client deliberately writes small request fragments and chunked bodies so
 * the server must parse boundaries across transport reads instead of relying on
 * one complete in-memory request. Other cases cover rejected uploads (which
 * must never leave a temporary file), the import thread pool, the upload idle
 * timeout, and the bounded drain that keeps early error responses readable.
 * Project tests check that load_project replaces the scene, the configuration
 * and the entity IDs atomically, keeps unknown keys as custom settings, and
 * leaves the session untouched when the project is refused. Per-profile metadata
 * travels in the core's project sections, independently of the flat settings.
 */

#include <catch_main.hpp>
#include <nlohmann/json.hpp>
#include <boost/property_tree/ini_parser.hpp>

#include "engine.hpp"
#include "libslic3r/Format/3mf.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/miniz_extension.hpp"
#include "multipart_upload.hpp"
#include "rest_server_fixture.hpp"
#include "session.hpp"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <future>
#include <iterator>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace RestServerTest;

namespace {

const std::string test_boundary = "restBoundary7";
const std::vector<std::string> model_extensions = {".stl", ".obj", ".3mf"};

void ensure_engine_initialized()
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

std::string create_session(RestServerFixture &fixture)
{
    const HttpResponseMessage created = fixture.request(http::verb::post, "/create_session");
    REQUIRE(created.result() == http::status::created);
    return nlohmann::json::parse(created.body()).at("session_id").get<std::string>();
}

std::string make_part(const std::string &disposition, const std::string &file)
{
    return "--" + test_boundary + "\r\n"
        "Content-Disposition: " + disposition + "\r\n"
        "Content-Type: application/octet-stream\r\n\r\n" + file + "\r\n";
}

std::string make_multipart_body(const std::string &filename, const std::string &file)
{
    return make_part("form-data; name=\"file\"; filename=\"" + filename + "\"", file) +
        "--" + test_boundary + "--\r\n";
}

// How the test client sends one multipart body.
struct UploadOptions {
    bool chunked = false;
    std::size_t fragment_size = 7;
    // Send only this prefix of the body, then half-close the connection.
    std::size_t body_bytes = std::string::npos;
    // Absent: the fixture token; empty string: no Authorization header.
    std::optional<std::string> token;
    std::chrono::milliseconds fragment_delay{0};
};

// Write the request by fragments, optionally chunked or delayed, then read the
// single response. Write errors are ignored because the server may answer and
// stop reading before the body is complete.
HttpResponseMessage send_multipart(
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

HttpResponseMessage upload_model(
    const RestServerFixture &fixture,
    const std::string &target,
    const std::string &filename,
    const std::string &file,
    const UploadOptions &options = {})
{
    return send_multipart(fixture, target, make_multipart_body(filename, file), options);
}

void require_error(
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

std::vector<std::string> object_ids(const HttpResponseMessage &response)
{
    REQUIRE(response.result() == http::status::ok);
    const nlohmann::json body = nlohmann::json::parse(response.body());
    const std::vector<std::string> ids = body.at("object_ids").get<std::vector<std::string>>();
    REQUIRE(body.at("object_count") == ids.size());
    return ids;
}

std::string make_obj_with_padding(std::size_t padding_bytes = 0)
{
    std::string obj;
    obj.reserve(padding_bytes + 80);
    while (obj.size() < padding_bytes)
        obj += "# padding for fragmented upload\n";
    obj += "v 0 0 0\n"
           "v 20 0 0\n"
           "v 0 20 0\n"
           "f 1 2 3\n";
    return obj;
}

std::string read_file(const std::filesystem::path &path)
{
    std::ifstream input(path, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

std::string make_stl()
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

// A 3MF with `object_count` triangles and, optionally, an embedded project
// configuration (ignored by add_model, applied by load_project) and an
// extruder assigned to the first object.
std::string make_3mf(
    std::size_t object_count = 1,
    const Slic3r::DynamicPrintConfig *config = nullptr,
    int first_object_extruder = 0)
{
    TempDirectory directory;
    const std::filesystem::path path = directory.path() / "model.3mf";
    const std::vector<Slic3r::Vec3f> vertices = {
        Slic3r::Vec3f(0.f, 0.f, 0.f),
        Slic3r::Vec3f(20.f, 0.f, 0.f),
        Slic3r::Vec3f(0.f, 20.f, 0.f)
    };
    const std::vector<Slic3r::Vec3i32> faces = {Slic3r::Vec3i32(0, 1, 2)};
    Slic3r::Model model;
    for (std::size_t index = 0; index < object_count; ++index)
        model.add_object(
            ("triangle" + std::to_string(index)).c_str(),
            "triangle.stl",
            Slic3r::TriangleMesh(vertices, faces));
    REQUIRE(model.add_default_instances());
    if (first_object_extruder > 0)
        model.object_ptrs().front()->config.set_key_value("extruder", new Slic3r::ConfigOptionInt(first_object_extruder));
    REQUIRE(Slic3r::store_3mf(path.string().c_str(), &model, config));
    return read_file(path);
}

// Append raw "; key = value" lines to the project configuration of a 3MF.
// The engine cannot write unknown keys itself, so this is how a test makes a
// project carry settings from a newer or foreign slicer.
std::string add_project_config_lines(const std::string &archive_bytes, const std::string &lines)
{
    TempDirectory directory;
    const std::filesystem::path input = directory.path() / "input.3mf";
    const std::filesystem::path output = directory.path() / "output.3mf";
    {
        std::ofstream stream(input, std::ios::binary);
        stream.write(archive_bytes.data(), static_cast<std::streamsize>(archive_bytes.size()));
    }
    mz_zip_archive reader;
    mz_zip_zero_struct(&reader);
    REQUIRE(Slic3r::open_zip_reader(&reader, input.string()));
    mz_zip_archive writer;
    mz_zip_zero_struct(&writer);
    REQUIRE(Slic3r::open_zip_writer(&writer, output.string()));
    bool config_found = false;
    for (mz_uint index = 0; index < mz_zip_reader_get_num_files(&reader); ++index) {
        mz_zip_archive_file_stat stat;
        REQUIRE(mz_zip_reader_file_stat(&reader, index, &stat));
        std::size_t size = 0;
        void *data = mz_zip_reader_extract_to_heap(&reader, index, &size, 0);
        REQUIRE(data != nullptr);
        std::string content(static_cast<const char *>(data), size);
        mz_free(data);
        if (std::string(stat.m_filename) == "Metadata/Slic3r.config") {
            content += lines;
            config_found = true;
        }
        REQUIRE(mz_zip_writer_add_mem(&writer, stat.m_filename, content.data(), content.size(), MZ_DEFAULT_COMPRESSION));
    }
    REQUIRE(config_found);
    REQUIRE(mz_zip_writer_finalize_archive(&writer));
    Slic3r::close_zip_writer(&writer);
    Slic3r::close_zip_reader(&reader);
    return read_file(output);
}

// Read settings through the typed route, as a client would.
nlohmann::json read_settings(const RestServerFixture &fixture, const std::string &route, const nlohmann::json &ids)
{
    const HttpResponseMessage response = fixture.request(
        http::verb::post,
        route + "/get_settings",
        nlohmann::json{{"setting_ids", ids}}.dump(),
        "application/json");
    REQUIRE(response.result() == http::status::ok);
    return nlohmann::json::parse(response.body());
}

std::shared_ptr<Slic3r::Rest::SessionContext> find_session(
    RestServerFixture &fixture,
    const std::string &session_id)
{
    return fixture.sessions().find(
        Slic3r::Rest::normalize_username("Alice"),
        session_id,
        Slic3r::Rest::SessionRegistry::Clock::now());
}

} // namespace

TEST_CASE("Multipart parser accepts fragmented STL body", "[rest_models]")
{
    TempDirectory directory;
    const std::string file = make_stl();
    const std::string body = make_multipart_body("triangle.stl", file);
    Slic3r::Rest::MultipartUploadParser parser(
        "multipart/form-data; boundary=" + test_boundary,
        directory.path(),
        1024 * 1024,
        16 * 1024,
        model_extensions);
    for (std::size_t offset = 0; offset < body.size(); offset += 7)
        parser.consume(body.data() + offset, std::min<std::size_t>(7, body.size() - offset));
    const std::shared_ptr<const Slic3r::Rest::HttpUpload> upload = parser.finish();
    REQUIRE(upload->byte_count == file.size());
    REQUIRE(read_file(upload->temporary_path) == file);
}

TEST_CASE("Multipart parser ignores the epilogue whatever the read split", "[rest_models]")
{
    TempDirectory directory;
    const std::string file = make_obj_with_padding();
    const std::string body = make_multipart_body("part.obj", file);
    REQUIRE(body.substr(body.size() - 4) == "--\r\n");
    Slic3r::Rest::MultipartUploadParser parser(
        "multipart/form-data; boundary=" + test_boundary,
        directory.path(),
        1024 * 1024,
        16 * 1024,
        model_extensions);
    // The closing "--" ends one read; its CRLF and an epilogue arrive later.
    parser.consume(body.data(), body.size() - 2);
    parser.consume("\r\n", 2);
    const std::string epilogue = "ignored epilogue\r\n";
    parser.consume(epilogue.data(), epilogue.size());
    const std::shared_ptr<const Slic3r::Rest::HttpUpload> upload = parser.finish();
    REQUIRE(upload->byte_count == file.size());
    REQUIRE(read_file(upload->temporary_path) == file);
}

TEST_CASE("Multipart parser stores uploads under random exclusive names", "[rest_models]")
{
    TempDirectory directory;
    const std::string body = make_multipart_body("part.obj", make_obj_with_padding());
    std::set<std::filesystem::path> paths;
    std::vector<std::shared_ptr<const Slic3r::Rest::HttpUpload>> uploads;
    for (int index = 0; index < 2; ++index) {
        Slic3r::Rest::MultipartUploadParser parser(
            "multipart/form-data; boundary=" + test_boundary,
            directory.path(),
            1024 * 1024,
            16 * 1024,
        model_extensions);
        parser.consume(body.data(), body.size());
        uploads.push_back(parser.finish());
        const std::filesystem::path path = uploads.back()->temporary_path;
        REQUIRE(path.parent_path() == directory.path());
        REQUIRE(path.filename().string().rfind("rest-upload-", 0) == 0);
        REQUIRE(path.extension() == ".obj");
        // The client file name never reaches the server path.
        REQUIRE(path.filename().string().find("part") == std::string::npos);
        paths.insert(path);
    }
    REQUIRE(paths.size() == 2);

    // Releasing the last owner deletes the temporary file.
    const std::filesystem::path first = uploads.front()->temporary_path;
    uploads.clear();
    REQUIRE_FALSE(std::filesystem::exists(first));
}

TEST_CASE("REST server removes stale uploads when it starts", "[rest_models]")
{
    RestServerFixture fixture;
    fixture.stop();
    const std::filesystem::path uploads = Slic3r::Rest::upload_directory(fixture.work_directory());
    std::filesystem::create_directories(uploads);
    std::ofstream(uploads / "rest-upload-stale.stl") << "left by a crash";
    std::ofstream(uploads / "unrelated.txt") << "not an upload";

    fixture.start();
    REQUIRE_FALSE(std::filesystem::exists(uploads / "rest-upload-stale.stl"));
    REQUIRE(std::filesystem::exists(uploads / "unrelated.txt"));
}

TEST_CASE("REST add_model imports STL OBJ and 3MF", "[rest_models]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string target = "/session/" + create_session(fixture) + "/add_model";

    UploadOptions fragmented;
    fragmented.fragment_size = 7;
    const std::vector<std::string> stl = object_ids(
        upload_model(fixture, target, "triangle.stl", make_stl(), fragmented));
    REQUIRE(stl.size() == 1);

    UploadOptions chunked;
    chunked.chunked = true;
    const std::vector<std::string> obj = object_ids(
        upload_model(fixture, target, "part.obj", make_obj_with_padding(), chunked));
    REQUIRE(obj.size() == 1);

    chunked.fragment_size = 11;
    const std::vector<std::string> three_mf = object_ids(
        upload_model(fixture, target, "project.3MF", make_3mf(2), chunked));
    REQUIRE(three_mf.size() == 2);

    // Every returned object ID is distinct, across objects and uploads.
    std::set<std::string> all_ids;
    for (const std::vector<std::string> *ids : {&stl, &obj, &three_mf})
        for (const std::string &id : *ids) {
            REQUIRE(!id.empty());
            REQUIRE(all_ids.insert(id).second);
        }

    // An epilogue after the closing boundary is ignored.
    const HttpResponseMessage epilogue = send_multipart(
        fixture,
        target,
        make_multipart_body("part.obj", make_obj_with_padding()) + "epilogue\r\n");
    REQUIRE(object_ids(epilogue).size() == 1);
    REQUIRE(fixture.work_file_count() == 0);
}

TEST_CASE("REST add_model names entities after the client file", "[rest_models]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string session_id = create_session(fixture);
    const std::string target = "/session/" + session_id + "/add_model";

    // A 3MF whose object and volume carry no name: the loader falls back to
    // "<file stem>_N", which must use the client file name.
    TempDirectory directory;
    const std::filesystem::path unnamed_path = directory.path() / "unnamed.3mf";
    Slic3r::Model unnamed;
    Slic3r::ModelObject *object = unnamed.add_object("", "", Slic3r::make_cube(5, 5, 5));
    object->volumes.front()->name.clear();
    object->add_instance();
    REQUIRE(Slic3r::store_3mf(unnamed_path.string().c_str(), &unnamed, nullptr));

    REQUIRE(object_ids(upload_model(fixture, target, "triangle.stl", make_stl())).size() == 1);
    REQUIRE(object_ids(upload_model(fixture, target, "part.obj", make_obj_with_padding())).size() == 1);
    REQUIRE(object_ids(upload_model(fixture, target, "bracket.3mf", read_file(unnamed_path))).size() == 1);

    const std::shared_ptr<Slic3r::Model> scene = find_session(fixture, session_id)->data_snapshot()->scene;
    std::vector<std::string> object_names;
    std::vector<std::string> volume_names;
    for (const Slic3r::ModelObject *model_object : scene->object_ptrs()) {
        object_names.push_back(model_object->name);
        CHECK(model_object->input_file.find("rest-upload-") == std::string::npos);
        for (const Slic3r::ModelVolume *volume : model_object->volumes) {
            volume_names.push_back(volume->name);
            CHECK(volume->source.input_file.find("rest-upload-") == std::string::npos);
        }
    }
    CHECK(object_names == std::vector<std::string>{"triangle.stl", "part.obj", "bracket_1"});
    CHECK(volume_names == std::vector<std::string>{"triangle.stl", "part.obj", "bracket_1"});
}

TEST_CASE("REST add_model ignores the project configuration of a 3MF", "[rest_models]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string route = "/session/" + create_session(fixture);

    Slic3r::DynamicPrintConfig project;
    project.set_deserialize_strict("layer_height", "0.5");
    REQUIRE(object_ids(upload_model(fixture, route + "/add_model", "project.3mf", make_3mf(1, &project))).size() == 1);

    const HttpResponseMessage layer_height = fixture.request(http::verb::get, route + "/get_setting/layer_height");
    REQUIRE(layer_height.result() == http::status::ok);
    REQUIRE(nlohmann::json::parse(layer_height.body()).at("serialized_value") == "0.2");
}

TEST_CASE("REST add_model rejects oversized and invalid uploads transactionally", "[rest_models]")
{
    ensure_engine_initialized();
    RestServerFixture::Limits limits;
    limits.max_upload_bytes = 32;
    RestServerFixture fixture(limits, true, true);
    const std::string target = "/session/" + create_session(fixture) + "/add_model";

    UploadOptions chunked;
    chunked.chunked = true;
    chunked.fragment_size = 5;
    const HttpResponseMessage oversized = upload_model(
        fixture, target, "too-large.obj", make_obj_with_padding(128), chunked);
    require_error(oversized, http::status::payload_too_large, "request_too_large");
    REQUIRE(fixture.work_file_count() == 0);

    // The client keeps sending a large body after the early 413: the server
    // drains it, so the response is still readable instead of a reset.
    UploadOptions large_fragments;
    large_fragments.fragment_size = 64 * 1024;
    const HttpResponseMessage drained = upload_model(
        fixture, target, "large.obj", make_obj_with_padding(512 * 1024), large_fragments);
    require_error(drained, http::status::payload_too_large, "request_too_large");
    REQUIRE(fixture.work_file_count() == 0);

    UploadOptions small_fragments;
    small_fragments.fragment_size = 3;
    const HttpResponseMessage invalid = upload_model(fixture, target, "invalid.stl", "not a model", small_fragments);
    require_error(invalid, http::status::unprocessable_entity, "invalid_model");
    // The server-side temporary path never appears in a client message.
    REQUIRE(invalid.body().find("rest-upload-") == std::string::npos);
    REQUIRE(fixture.work_file_count() == 0);
}

TEST_CASE("REST add_model rejects truncated uploads", "[rest_models]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string target = "/session/" + create_session(fixture) + "/add_model";
    const std::string file = make_obj_with_padding(256);

    // The connection ends in the middle of the file data.
    UploadOptions interrupted;
    interrupted.body_bytes = make_multipart_body("part.obj", file).size() / 2;
    require_error(
        upload_model(fixture, target, "part.obj", file, interrupted),
        http::status::bad_request,
        "invalid_request",
        "truncated");
    REQUIRE(fixture.work_file_count() == 0);

    // The HTTP body is complete but the multipart closing boundary is missing.
    const std::string unterminated = make_part("form-data; name=\"file\"; filename=\"part.obj\"", file);
    require_error(
        send_multipart(fixture, target, unterminated),
        http::status::bad_request,
        "invalid_multipart",
        "no closing boundary");
    REQUIRE(fixture.work_file_count() == 0);
}

TEST_CASE("REST add_model rejects malformed multipart bodies", "[rest_models]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string target = "/session/" + create_session(fixture) + "/add_model";
    const std::string file = make_obj_with_padding();
    const std::string closing = "--" + test_boundary + "--\r\n";

    require_error(
        send_multipart(fixture, target, make_part("form-data; name=\"document\"; filename=\"part.obj\"", file) + closing),
        http::status::bad_request,
        "invalid_multipart",
        "must be named file");
    require_error(
        send_multipart(fixture, target, make_part("form-data; name=\"file\"", file) + closing),
        http::status::bad_request,
        "invalid_multipart",
        "file name is missing");
    const std::string file_part = make_part("form-data; name=\"file\"; filename=\"part.obj\"", file);
    require_error(
        send_multipart(fixture, target, file_part + file_part + closing),
        http::status::bad_request,
        "invalid_multipart",
        "Only one multipart file part");
    REQUIRE(fixture.work_file_count() == 0);
}

TEST_CASE("REST add_model rejects unsupported multipart filenames", "[rest_models]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string target = "/session/" + create_session(fixture) + "/add_model";
    const std::string body = make_multipart_body("../../model.txt", "not accepted");
    UploadOptions headers_only;
    headers_only.fragment_size = 2;
    headers_only.body_bytes = body.find("\r\n\r\n") + 4;
    const HttpResponseMessage response = upload_model(
        fixture, target, "../../model.txt", "not accepted", headers_only);
    require_error(response, http::status::unprocessable_entity, "unsupported_model_format");
    REQUIRE(fixture.work_file_count() == 0);
}

TEST_CASE("REST add_model authorizes before storing any byte", "[rest_models]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string session_id = create_session(fixture);
    const std::string file = make_obj_with_padding();

    UploadOptions anonymous;
    anonymous.token = std::string();
    require_error(
        upload_model(fixture, "/session/" + session_id + "/add_model", "part.obj", file, anonymous),
        http::status::unauthorized,
        "unauthorized");

    UploadOptions invalid_token;
    invalid_token.token = std::string("not-a-token");
    require_error(
        upload_model(fixture, "/session/" + session_id + "/add_model", "part.obj", file, invalid_token),
        http::status::unauthorized,
        "unauthorized");

    UploadOptions other_owner;
    other_owner.token = fixture.login_token("Bob", "secret");
    require_error(
        upload_model(fixture, "/session/" + session_id + "/add_model", "part.obj", file, other_owner),
        http::status::not_found,
        "session_not_found");
    require_error(
        upload_model(fixture, "/session/session_missing/add_model", "part.obj", file),
        http::status::not_found,
        "session_not_found");
    REQUIRE(fixture.work_file_count() == 0);
}

TEST_CASE("REST add_model respects the slicing state and invalidates results", "[rest_models]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string session_id = create_session(fixture);
    const std::string target = "/session/" + session_id + "/add_model";
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = find_session(fixture, session_id);
    REQUIRE(session != nullptr);

    const std::optional<Slic3r::Rest::SessionContext::WorkToken> work =
        session->begin_work(Slic3r::Rest::SessionRegistry::Clock::now());
    REQUIRE(work.has_value());
    require_error(
        upload_model(fixture, target, "part.obj", make_obj_with_padding()),
        http::status::conflict,
        "session_busy");
    REQUIRE(fixture.work_file_count() == 0);
    REQUIRE(session->data_snapshot()->scene->object_ptrs().empty());

    REQUIRE(session->finish_work(
        work.value(),
        "result.gcode",
        Slic3r::GCodeProcessorResult(),
        Slic3r::Rest::SessionRegistry::Clock::now()));
    REQUIRE(object_ids(upload_model(fixture, target, "part.obj", make_obj_with_padding())).size() == 1);
    const Slic3r::Rest::SessionContext::Snapshot after = session->snapshot();
    REQUIRE(after.status == Slic3r::Rest::SessionStatus::ready);
    REQUIRE_FALSE(after.current_gcode);
    REQUIRE(session->viewer_snapshot()->outdated);
    REQUIRE(session->data_snapshot()->scene->object_ptrs().size() == 1);
}

TEST_CASE("REST add_model parses away from the network thread", "[rest_models]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);

    // The observer holds the import until the test has been served another
    // request, which would deadlock if both ran on the network thread. The
    // bounded wait keeps a failed assertion from blocking the server shutdown.
    std::promise<void> import_started;
    std::promise<void> release_import;
    std::shared_future<void> released = release_import.get_future().share();
    fixture.stop();
    fixture.set_import_observer([&import_started, released]() {
        import_started.set_value();
        released.wait_for(std::chrono::seconds(10));
    });
    fixture.start();

    const std::string target = "/session/" + create_session(fixture) + "/add_model";
    std::future<HttpResponseMessage> upload = std::async(std::launch::async, [&fixture, &target]() {
        return upload_model(fixture, target, "part.obj", make_obj_with_padding());
    });
    std::future<void> started = import_started.get_future();
    REQUIRE(started.wait_for(std::chrono::seconds(10)) == std::future_status::ready);

    const HttpResponseMessage concurrent = fixture.request(http::verb::post, "/create_session");
    REQUIRE(concurrent.result() == http::status::created);

    release_import.set_value();
    REQUIRE(upload.wait_for(std::chrono::seconds(10)) == std::future_status::ready);
    REQUIRE(object_ids(upload.get()).size() == 1);
}

TEST_CASE("REST add_model uses an idle timeout for slow uploads", "[rest_models]")
{
    ensure_engine_initialized();
    RestServerFixture::Limits limits;
    limits.timeout = std::chrono::seconds(1);
    RestServerFixture fixture(limits, true, true);
    const std::string target = "/session/" + create_session(fixture) + "/add_model";

    // The whole upload lasts longer than the timeout, but no gap exceeds it.
    UploadOptions slow;
    slow.fragment_size = 48;
    slow.fragment_delay = std::chrono::milliseconds(300);
    const std::string file = make_obj_with_padding(256);
    REQUIRE(make_multipart_body("part.obj", file).size() / slow.fragment_size >= 6);
    REQUIRE(object_ids(upload_model(fixture, target, "part.obj", file, slow)).size() == 1);
}

TEST_CASE("REST load_project replaces a modified session", "[rest_projects]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string session_id = create_session(fixture);
    const std::string route = "/session/" + session_id;
    const std::string other_route = "/session/" + create_session(fixture);
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = find_session(fixture, session_id);
    REQUIRE(session != nullptr);
    const std::map<std::string, std::string> profile_files = fixture.profile_files();
    const nlohmann::json defaults = read_settings(fixture, other_route, {"layer_height", "perimeters"});

    // Local edits, a scene object, preferences, and a finished result.
    REQUIRE(fixture.request(http::verb::post, route + "/set_setting/layer_height/0.25").result() == http::status::ok);
    REQUIRE(fixture.request(http::verb::post, route + "/set_setting/perimeters/7").result() == http::status::ok);
    const std::vector<std::string> old_ids = object_ids(
        upload_model(fixture, route + "/add_model", "part.obj", make_obj_with_padding()));
    {
        const std::optional<Slic3r::Rest::SessionContext::MutationToken> mutation = session->begin_mutation();
        Slic3r::Rest::SessionData data = session->data_snapshot().value();
        data.preferences = nlohmann::json{{"units", "mm"}};
        REQUIRE(session->commit_mutation(mutation.value(), std::move(data), false, Slic3r::Rest::SessionRegistry::Clock::now()));
    }
    const std::optional<Slic3r::Rest::SessionContext::WorkToken> work =
        session->begin_work(Slic3r::Rest::SessionRegistry::Clock::now());
    REQUIRE(session->finish_work(
        work.value(), "result.gcode", Slic3r::GCodeProcessorResult(), Slic3r::Rest::SessionRegistry::Clock::now()));

    // A partial project: only two known keys plus unknown keys of each origin.
    Slic3r::DynamicPrintConfig partial;
    partial.set_deserialize_strict("layer_height", "0.3");
    partial.set_deserialize_strict("post_process", "\"C:/witness/script.bat\"");
    const std::string project = add_project_config_lines(make_3mf(2, &partial),
        "; future_print_key = 42\n"
        "; printer_future_key = pf\n"
        "; retract_future_key = 1\n"
        "; filament_future_key = 7\n");
    const std::vector<std::string> new_ids = object_ids(
        upload_model(fixture, route + "/load_project", "project.3mf", project));
    REQUIRE(new_ids.size() == 2);

    // The scene and its IDs are replaced.
    for (const std::string &id : old_ids)
        REQUIRE_FALSE(session->resolve_entity(id).has_value());
    for (const std::string &id : new_ids) {
        REQUIRE(std::find(old_ids.begin(), old_ids.end(), id) == old_ids.end());
        REQUIRE(session->resolve_entity(id).has_value());
    }
    REQUIRE(session->data_snapshot()->scene->object_ptrs().size() == 2);

    // Present keys are applied, absent ones return to the system defaults,
    // and the local edits are gone. post_process is kept but never run.
    const nlohmann::json values = read_settings(fixture, route,
        {"layer_height", "perimeters", "post_process", "print_custom_setting", "filament_custom_setting"});
    REQUIRE(values.at("layer_height").at("value") == 0.3);
    REQUIRE(values.at("perimeters") == defaults.at("perimeters"));
    REQUIRE(values.at("post_process").at("value") == nlohmann::json::array({"C:/witness/script.bat"}));
    REQUIRE(nlohmann::json::parse(values.at("print_custom_setting").at("value").get<std::string>()) == nlohmann::json{
        {"print", {{"future_print_key", "42"}}},
        {"printer", {{"printer_future_key", "pf"}, {"retract_future_key", "1"}}}});
    const nlohmann::json filament_custom = values.at("filament_custom_setting").at("value");
    REQUIRE(filament_custom.size() == 1);
    REQUIRE(nlohmann::json::parse(filament_custom.at(0).get<std::string>()) ==
        nlohmann::json{{"filament_future_key", "7"}});

    // Session-only data and lifecycle: preferences kept, embedded profiles are
    // not catalog profiles, the result is outdated.
    const Slic3r::Rest::SessionData data = session->data_snapshot().value();
    REQUIRE(data.preferences == nlohmann::json{{"units", "mm"}});
    REQUIRE(data.profiles.print.empty());
    REQUIRE(data.profiles.printer.empty());
    const Slic3r::Rest::SessionContext::Snapshot snapshot = session->snapshot();
    REQUIRE(snapshot.status == Slic3r::Rest::SessionStatus::ready);
    REQUIRE_FALSE(snapshot.current_gcode);
    REQUIRE(session->viewer_snapshot()->outdated);

    // Nothing leaks into another session or into the installed profiles.
    REQUIRE(read_settings(fixture, other_route, {"layer_height", "perimeters"}) == defaults);
    REQUIRE(fixture.profile_files() == profile_files);
    REQUIRE(fixture.work_file_count() == 0);
}

TEST_CASE("REST load_project keeps separate profile metadata without trusting section settings", "[rest_projects][rest_profile_sections]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string id = create_session(fixture);
    const std::string route = "/session/" + id;
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = find_session(fixture, id);
    const std::string other_id = create_session(fixture);
    const Slic3r::Rest::SessionData other_before = find_session(fixture, other_id)->data_snapshot().value();
    const std::map<std::string, std::string> files_before = fixture.profile_files();
    Slic3r::DynamicPrintConfig configuration(session->data_snapshot()->configuration);
    configuration.set_deserialize_strict("print_settings_id", "Embedded print");
    configuration.set_deserialize_strict("printer_settings_id", "Embedded printer");
    configuration.set_deserialize_strict("filament_settings_id", "Embedded A;Embedded B");
    configuration.set_deserialize_strict("nozzle_diameter", "0.4,0.4");
    configuration.set_deserialize_strict("extruder_offset", "0x0,0x0");
    const std::string metadata = R"ini([print:Embedded print]
inherits = Print parent
compatible_printers_condition = nozzle_diameter[0]==0.4
post_process = section-script-witness

[printer:Embedded printer]
inherits = Printer parent
default_print_profile = Embedded print
default_filament_profile = Embedded A;Embedded B
printhost_apikey = section-local-witness

[filament:Embedded A]
inherits = Filament parent A
compatible_printers_condition = nozzle_diameter[0]==0.4

[filament:Embedded B]
inherits = Filament parent B
compatible_prints_condition = layer_height>0
)ini";
    TempDirectory directory;
    const std::filesystem::path path = directory.path() / "metadata.3mf";
    Slic3r::Model scene;
    scene.add_object("cube", "cube.stl", Slic3r::make_cube(20, 20, 20))->add_instance();
    REQUIRE(Slic3r::store_3mf(path.u8string().c_str(), &scene, &configuration,
        Slic3r::OptionStore3mf{}.set_profile_sections(metadata)));
    REQUIRE(object_ids(upload_model(fixture, route + "/load_project", "metadata.3mf", read_file(path))).size() == 1);
    const Slic3r::Rest::SessionData loaded = session->data_snapshot().value();
    boost::property_tree::ptree sections;
    std::istringstream input(loaded.profile_sections);
    boost::property_tree::read_ini(input, sections);
    REQUIRE(sections.size() == 4);
    const boost::property_tree::ptree::const_assoc_iterator print = sections.find(
        "print:" + loaded.configuration.opt_string("print_settings_id"));
    const boost::property_tree::ptree::const_assoc_iterator printer = sections.find(
        "printer:" + loaded.configuration.opt_string("printer_settings_id"));
    REQUIRE(print != sections.not_found());
    REQUIRE(printer != sections.not_found());
    CHECK(print->second.get<std::string>("inherits") == "Print parent");
    CHECK(print->second.get<std::string>("compatible_printers_condition") == "nozzle_diameter[0]==0.4");
    CHECK(printer->second.get<std::string>("inherits") == "Printer parent");
    CHECK(printer->second.get<std::string>("default_print_profile") == "Embedded print");
    const std::vector<std::string> &filaments = loaded.configuration.option<Slic3r::ConfigOptionStrings>(
        "filament_settings_id")->get_values();
    REQUIRE(filaments.size() == 2);
    for (std::size_t slot = 0; slot < filaments.size(); ++slot) {
        CAPTURE(slot);
        const boost::property_tree::ptree::const_assoc_iterator filament = sections.find("filament:" + filaments[slot]);
        REQUIRE(filament != sections.not_found());
        CHECK(filament->second.get<std::string>("inherits") == (slot == 0 ? "Filament parent A" : "Filament parent B"));
        CHECK(filament->second.get<std::string>(slot == 0 ? "compatible_printers_condition" : "compatible_prints_condition") ==
            (slot == 0 ? "nozzle_diameter[0]==0.4" : "layer_height>0"));
    }
    CHECK(loaded.profile_sections.find("section-script-witness") == std::string::npos);
    CHECK(loaded.profile_sections.find("section-local-witness") == std::string::npos);
    CHECK(loaded.configuration.option<Slic3r::ConfigOptionStrings>("post_process")->empty());
    CHECK_FALSE(loaded.configuration.has("printhost_apikey"));
    REQUIRE(fixture.request(http::verb::post, route + "/set_setting/layer_height/0.25").result() == http::status::ok);
    CHECK(session->data_snapshot()->profile_sections == loaded.profile_sections);
    CHECK(find_session(fixture, other_id)->data_snapshot()->profile_sections == other_before.profile_sections);
    CHECK(fixture.profile_files() == files_before);
}

TEST_CASE("REST load_project keeps installed profile IDs and accepts geometry only", "[rest_projects]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string session_id = create_session(fixture);
    const std::string route = "/session/" + session_id;
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = find_session(fixture, session_id);
    const Slic3r::Rest::SessionData initial = session->data_snapshot().value();
    REQUIRE_FALSE(initial.profiles.print.empty());

    // A project naming the unchanged installed profiles selects them again.
    // This minimal flat project only names installed profiles. Metadata that
    // the project does not provide stays as defined in the catalog.
    Slic3r::DynamicPrintConfig named;
    for (const char *key : {"print_settings_id", "printer_settings_id", "filament_settings_id", "default_filament_profile"})
        named.set_key_value(key, initial.configuration.option(key)->clone());
    REQUIRE(object_ids(upload_model(
        fixture, route + "/load_project", "project.3mf", make_3mf(1, &named))).size() == 1);
    const Slic3r::Rest::ProfileSelection selection = session->data_snapshot()->profiles;
    REQUIRE(selection.print == initial.profiles.print);
    REQUIRE(selection.printer == initial.profiles.printer);
    REQUIRE(selection.filaments == initial.profiles.filaments);

    // The same installed profiles with one modified value are no longer the
    // catalog profiles: the modified category loses its ID, the others keep it.
    Slic3r::DynamicPrintConfig modified_print = named;
    modified_print.set_deserialize_strict("layer_height", "0.3");
    REQUIRE(object_ids(upload_model(
        fixture, route + "/load_project", "print.3mf", make_3mf(1, &modified_print))).size() == 1);
    const Slic3r::Rest::ProfileSelection print_changed = session->data_snapshot()->profiles;
    REQUIRE(print_changed.print.empty());
    REQUIRE(print_changed.printer == initial.profiles.printer);
    REQUIRE(print_changed.filaments == initial.profiles.filaments);

    Slic3r::DynamicPrintConfig modified_filament = named;
    modified_filament.set_deserialize_strict("filament_diameter", "2.85");
    REQUIRE(object_ids(upload_model(
        fixture, route + "/load_project", "filament.3mf", make_3mf(1, &modified_filament))).size() == 1);
    const Slic3r::Rest::ProfileSelection filament_changed = session->data_snapshot()->profiles;
    REQUIRE(filament_changed.print == initial.profiles.print);
    REQUIRE(filament_changed.printer == initial.profiles.printer);
    REQUIRE(filament_changed.filaments == std::vector<std::string>{""});

    // A 3MF without configuration resets every setting to the defaults.
    REQUIRE(fixture.request(http::verb::post, route + "/set_setting/layer_height/0.25").result() == http::status::ok);
    REQUIRE(object_ids(upload_model(fixture, route + "/load_project", "geometry.3mf", make_3mf(3))).size() == 3);
    REQUIRE(read_settings(fixture, route, {"layer_height"}).at("layer_height").at("value") == 0.2);

    // Unnamed project objects take the "<file stem>_N" fallback of the client
    // file, never the server temporary file name.
    TempDirectory directory;
    const std::filesystem::path unnamed_path = directory.path() / "unnamed.3mf";
    Slic3r::Model unnamed;
    Slic3r::ModelObject *object = unnamed.add_object("", "", Slic3r::make_cube(5, 5, 5));
    object->volumes.front()->name.clear();
    object->add_instance();
    REQUIRE(Slic3r::store_3mf(unnamed_path.string().c_str(), &unnamed, nullptr));
    REQUIRE(object_ids(upload_model(fixture, route + "/load_project", "bracket.3mf", read_file(unnamed_path))).size() == 1);
    const std::shared_ptr<Slic3r::Model> loaded_scene = session->data_snapshot()->scene;
    const Slic3r::ModelObject *loaded = loaded_scene->object_ptrs().front();
    CHECK(loaded->name == "bracket_1");
    CHECK(loaded->volumes.front()->name == "bracket_1");
    CHECK(loaded->input_file.find("rest-upload-") == std::string::npos);
}

TEST_CASE("REST load_project refusals leave the session intact", "[rest_projects]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string session_id = create_session(fixture);
    const std::string route = "/session/" + session_id;
    const std::string target = route + "/load_project";
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = find_session(fixture, session_id);
    REQUIRE(fixture.request(http::verb::post, route + "/set_setting/layer_height/0.25").result() == http::status::ok);
    const std::vector<std::string> ids = object_ids(
        upload_model(fixture, route + "/add_model", "part.obj", make_obj_with_padding()));
    const std::uint64_t revision = session->snapshot().revision;
    const std::string profile_sections = session->data_snapshot()->profile_sections;

    Slic3r::DynamicPrintConfig sla;
    sla.set_deserialize_strict("printer_technology", "SLA");
    Slic3r::DynamicPrintConfig incompatible;
    incompatible.set_deserialize_strict("use_firmware_retraction", "1");
    incompatible.set_deserialize_strict("wipe", "1");
    Slic3r::DynamicPrintConfig single_extruder;
    single_extruder.set_deserialize_strict("layer_height", "0.3");
    Slic3r::DynamicPrintConfig malformed_custom;
    malformed_custom.set_deserialize_strict("print_custom_setting", "not json");
    // The recomposition clamps global extruder options, so they must be
    // refused as loaded.
    Slic3r::DynamicPrintConfig global_extruder;
    global_extruder.set_deserialize_strict("perimeter_extruder", "3");
    // An empty nozzle list cannot be written by the engine; the raw line is
    // appended to the project configuration, where the last value wins.
    const std::string no_extruder = add_project_config_lines(
        make_3mf(1, &single_extruder), "; nozzle_diameter = \n");

    struct Refusal {
        std::string filename;
        std::string file;
        std::string code;
        std::string message;
    };
    const std::vector<Refusal> refusals = {
        {"sla.3mf", make_3mf(1, &sla), "invalid_project", "not an FFF project"},
        {"incompatible.3mf", make_3mf(1, &incompatible), "invalid_project", "not compatible"},
        {"extruder.3mf", make_3mf(1, &single_extruder, 3), "invalid_project", "uses extruder 3"},
        {"global_extruder.3mf", make_3mf(1, &global_extruder), "invalid_project",
            "uses extruder 3 in perimeter_extruder"},
        {"no_extruder.3mf", no_extruder, "invalid_project", "defines no extruder"},
        {"custom.3mf", make_3mf(1, &malformed_custom), "invalid_project", "custom setting"},
        {"corrupt.3mf", "not a zip archive", "invalid_project", ""},
        {"part.obj", make_obj_with_padding(), "unsupported_model_format", ".3mf"}
    };
    for (const Refusal &refusal : refusals) {
        INFO(refusal.filename);
        require_error(
            upload_model(fixture, target, refusal.filename, refusal.file),
            http::status::unprocessable_entity,
            refusal.code,
            refusal.message);
    }

    REQUIRE(session->snapshot().revision == revision);
    REQUIRE(session->data_snapshot()->profile_sections == profile_sections);
    REQUIRE(read_settings(fixture, route, {"layer_height"}).at("layer_height").at("value") == 0.25);
    REQUIRE(session->data_snapshot()->scene->object_ptrs().size() == 1);
    REQUIRE(session->resolve_entity(ids.front()).has_value());
    REQUIRE(fixture.work_file_count() == 0);

    // A running slicing job refuses the upload before it is stored.
    const std::optional<Slic3r::Rest::SessionContext::WorkToken> work =
        session->begin_work(Slic3r::Rest::SessionRegistry::Clock::now());
    REQUIRE(work.has_value());
    require_error(
        upload_model(fixture, target, "project.3mf", make_3mf()),
        http::status::conflict,
        "session_busy");
    REQUIRE(fixture.work_file_count() == 0);
}
