/*
 * Headless engine integration tests for the REST server.
 *
 * The first test initializes the same resource/plugin/configuration sequence as
 * the standalone process, slices an in-memory FFF cube and checks the viewer
 * result. The concurrency test overlaps two real runs and cancels one of them,
 * the viewer test covers page boundaries and speed fields, and the last test
 * checks that a cancellation requested before the slice starts is honoured.
 */

#include <catch_main.hpp>

#include "engine.hpp"
#include "settings.hpp"

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <exception>
#include <fstream>
#include <iterator>
#include <mutex>
#include <optional>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "libslic3r/Exception.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/TriangleMesh.hpp"

namespace {

class TemporaryRoot {
public:
    TemporaryRoot()
    {
        static std::uint64_t next_id = 1;
        root_ = std::filesystem::temp_directory_path() /
            ("superslicer_rest_engine_" + std::to_string(next_id++));
        std::filesystem::create_directories(root_);
    }

    ~TemporaryRoot()
    {
        std::error_code ignored;
        std::filesystem::remove_all(root_, ignored);
    }

    const std::filesystem::path &path() const
    {
        return root_;
    }

private:
    std::filesystem::path root_;
};

bool spatial(Slic3r::EMoveType type)
{
    return type == Slic3r::EMoveType::Travel ||
        type == Slic3r::EMoveType::Wipe ||
        type == Slic3r::EMoveType::Extrude;
}

TemporaryRoot &engine_data_root()
{
    static TemporaryRoot root;
    return root;
}

void ensure_engine_initialized()
{
    Slic3r::Rest::Engine &engine = Slic3r::Rest::Engine::instance();
    if (engine.initialized())
        return;

    Slic3r::Rest::EnginePaths paths;
    paths.resource_directory = SLIC3R_REST_RESOURCE_DIRECTORY;
    paths.data_directory = engine_data_root().path() / "engine_data";
    engine.initialize(paths);
}

Slic3r::Model make_cube_model(const std::string &name, double size)
{
    Slic3r::Model model;
    Slic3r::ModelObject *object = model.add_object(
        name.c_str(), name.c_str(), Slic3r::make_cube(size, size, size));
    object->add_instance();
    object->ensure_on_bed();
    return model;
}

Slic3r::DynamicPrintConfig make_slice_config()
{
    Slic3r::DynamicPrintConfig config = Slic3r::DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"layer_height", "0.2"},
        {"first_layer_height", "0.2"},
        {"nozzle_diameter", "0.4"},
        {"fill_density", "0%"},
        {"skirts", "0"},
        {"brim_width", "0"}
    });
    return config;
}

struct ConcurrentSliceResult
{
    std::optional<Slic3r::Rest::SliceResult> first;
    std::optional<Slic3r::Rest::SliceResult> second;
    std::exception_ptr first_error;
    std::exception_ptr second_error;
    bool first_cancel_requested = false;
    bool second_cancel_requested = false;
    std::size_t first_progress_events = 0;
    std::size_t second_progress_events = 0;
};

ConcurrentSliceResult run_concurrent_pair(
    const std::filesystem::path &first_output,
    const std::filesystem::path &second_output,
    bool cancel_first)
{
    Slic3r::Rest::Engine &engine = Slic3r::Rest::Engine::instance();
    Slic3r::Rest::SliceControl first_control;
    Slic3r::Rest::SliceControl second_control;
    std::atomic<std::size_t> first_events{0};
    std::atomic<std::size_t> second_events{0};
    std::mutex progress_mutex;
    std::condition_variable progress_condition;
    bool first_progress = false;
    bool second_progress = false;
    bool release_cancelled = false;
    bool cancelled_callback_entered = false;

    // Both prints report progress before either is cancelled, and the print
    // to cancel is held inside its status callback until the request is made,
    // so the cancellation provably lands while both are computing.
    auto wait_for_pair = [&](bool first) {
        (first ? first_events : second_events).fetch_add(1);
        std::unique_lock<std::mutex> lock(progress_mutex);
        bool &progress = first ? first_progress : second_progress;
        progress = true;
        progress_condition.notify_all();
        if ((first && cancel_first) || (!first && !cancel_first)) {
            cancelled_callback_entered = true;
            progress_condition.wait(lock, [&]() { return release_cancelled; });
        }
    };
    first_control.set_progress_callback([&](const Slic3r::PrintStatus::SlicingStatus &) {
        wait_for_pair(true);
    });
    second_control.set_progress_callback([&](const Slic3r::PrintStatus::SlicingStatus &) {
        wait_for_pair(false);
    });

    ConcurrentSliceResult result;
    std::thread first_thread([&]() {
        try {
            result.first = engine.slice(
                make_cube_model("concurrent_first.stl", 30.0),
                make_slice_config(),
                first_output,
                &first_control);
        } catch (...) {
            result.first_error = std::current_exception();
        }
    });
    std::thread second_thread([&]() {
        try {
            result.second = engine.slice(
                make_cube_model("concurrent_second.stl", 22.0),
                make_slice_config(),
                second_output,
                &second_control);
        } catch (...) {
            result.second_error = std::current_exception();
        }
    });

    {
        std::unique_lock<std::mutex> lock(progress_mutex);
        progress_condition.wait(lock, [&]() {
            return first_progress && second_progress && cancelled_callback_entered;
        });
    }
    if (cancel_first)
        first_control.request_cancel();
    else
        second_control.request_cancel();
    {
        std::lock_guard<std::mutex> lock(progress_mutex);
        release_cancelled = true;
    }
    progress_condition.notify_all();

    first_thread.join();
    second_thread.join();
    result.first_cancel_requested = first_control.cancel_requested();
    result.second_cancel_requested = second_control.cancel_requested();
    result.first_progress_events = first_events.load();
    result.second_progress_events = second_events.load();
    return result;
}

} // namespace

TEST_CASE("headless engine performs a real FFF export", "[rest_engine]")
{
    TemporaryRoot root;
    ensure_engine_initialized();
    Slic3r::Rest::Engine &engine = Slic3r::Rest::Engine::instance();
    REQUIRE(engine.initialized());

    Slic3r::Model model;
    Slic3r::ModelObject *object = model.add_object(
        "rest_smoke.stl", "rest_smoke.stl", Slic3r::make_cube(20.0, 20.0, 20.0));
    object->add_instance();
    object->ensure_on_bed();

    Slic3r::DynamicPrintConfig config = Slic3r::DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"layer_height", "0.2"},
        {"first_layer_height", "0.2"},
        {"nozzle_diameter", "0.4"},
        {"fill_density", "0%"},
        {"skirts", "0"},
        {"brim_width", "0"}
    });

    const std::filesystem::path output = root.path() / "work" / "real.gcode";
    const Slic3r::Rest::SliceResult result = engine.slice(
        std::move(model), std::move(config), output);

    REQUIRE(result.output_file == output);
    REQUIRE(std::filesystem::is_regular_file(output));
    REQUIRE(std::filesystem::file_size(output) > 0);
    REQUIRE(result.viewer.filename == output.string());
    REQUIRE(result.viewer.reader_plugin_id == "gcode.reader.default");
    REQUIRE(!result.viewer.moves.empty());
    REQUIRE(!result.viewer.lines_ends.empty());

    std::ifstream gcode(output, std::ios::binary);
    const std::string contents{
        std::istreambuf_iterator<char>(gcode), std::istreambuf_iterator<char>()};
    REQUIRE(contents.find("G1") != std::string::npos);

    float previous_time = 0.0f;
    bool has_spatial_move = false;
    for (const Slic3r::GCodeProcessorResult::MoveVertex &move : result.viewer.moves) {
        REQUIRE(move.move_time >= previous_time);
        previous_time = move.move_time;
        has_spatial_move = has_spatial_move || spatial(move.type);
    }
    REQUIRE(has_spatial_move);
}

TEST_CASE("settings catalog exposes finalized definitions", "[rest_engine]")
{
    ensure_engine_initialized();

    const nlohmann::json catalog = Slic3r::Rest::settings_catalog();
    for (const char *category : {"preference", "print", "filament", "extruder", "printer"}) {
        REQUIRE(catalog.contains(category));
        REQUIRE(catalog.at(category).is_array());
        for (const nlohmann::json &setting : catalog.at(category)) {
            REQUIRE(setting.contains("id"));
            REQUIRE(setting.contains("type"));
            REQUIRE(setting.contains("label"));
            REQUIRE(setting.contains("default"));
        }
    }

    const nlohmann::json::const_iterator printer_setting = std::find_if(
        catalog.at("printer").begin(),
        catalog.at("printer").end(),
        [](const nlohmann::json &setting) { return setting.at("id") == "nozzle_diameter"; });
    REQUIRE(printer_setting != catalog.at("printer").end());
    REQUIRE(printer_setting->at("array") == "extruder");
    REQUIRE(!printer_setting->at("default").at("value").is_array());

    // Preferences come from AppConfig, which the headless test engine does not
    // project yet.
    REQUIRE(catalog.at("preference").empty());

    // The catalog reports extruder-sized vectors and leaves every other vector
    // variable; fixed cardinalities are not emitted yet.
    for (const char *category : {"preference", "print", "filament", "extruder", "printer"}) {
        for (const nlohmann::json &setting : catalog.at(category)) {
            INFO(setting.at("id"));
            if (setting.contains("array")) {
                const std::string array = setting.at("array");
                CHECK((array == "extruder" || array == "variable"));
            }
        }
    }

    const nlohmann::json::const_iterator filament_diameter = std::find_if(
        catalog.at("filament").begin(),
        catalog.at("filament").end(),
        [](const nlohmann::json &setting) { return setting.at("id") == "filament_diameter"; });
    REQUIRE(filament_diameter != catalog.at("filament").end());
    REQUIRE(filament_diameter->at("array") == "extruder");
    REQUIRE(!filament_diameter->at("default").at("value").is_array());

    // Profile metadata is exposed as a variable array. Scalar-like overrides
    // generated for filament presets inherit the host generator's type and are
    // reported as variable because that generator does not mark them as
    // extruder-sized; this is a core-side limitation, not a REST one.
    const nlohmann::json::const_iterator compatible_printers = std::find_if(
        catalog.at("filament").begin(),
        catalog.at("filament").end(),
        [](const nlohmann::json &setting) { return setting.at("id") == "compatible_printers"; });
    REQUIRE(compatible_printers != catalog.at("filament").end());
    REQUIRE(compatible_printers->at("array") == "variable");

    // Variable-length vectors are reported as variable until fixed cardinalities
    // are emitted.
    const nlohmann::json::const_iterator thumbnails = std::find_if(
        catalog.at("printer").begin(),
        catalog.at("printer").end(),
        [](const nlohmann::json &setting) { return setting.at("id") == "thumbnails"; });
    REQUIRE(thumbnails != catalog.at("printer").end());
    REQUIRE(thumbnails->at("array") == "variable");
}

TEST_CASE("headless runs isolate concurrent cancellation and restart", "[rest_engine]")
{
    ensure_engine_initialized();

    SECTION("cancel the first run while the second exports")
    {
        TemporaryRoot root;
        const std::filesystem::path first_output = root.path() / "first.gcode";
        const std::filesystem::path second_output = root.path() / "second.gcode";
        const ConcurrentSliceResult result = run_concurrent_pair(first_output, second_output, true);

        REQUIRE(result.first_cancel_requested);
        REQUIRE(result.first_progress_events > 0);
        REQUIRE(result.second_progress_events > 0);
        REQUIRE(result.first_error != nullptr);
        REQUIRE(!result.first.has_value());
        REQUIRE(result.second_error == nullptr);
        REQUIRE(result.second.has_value());
        REQUIRE(std::filesystem::is_regular_file(second_output));
        REQUIRE(std::filesystem::file_size(second_output) > 0);

        Slic3r::Rest::SliceControl retry_control;
        const std::filesystem::path retry_output = root.path() / "first_retry.gcode";
        const Slic3r::Rest::SliceResult retry = Slic3r::Rest::Engine::instance().slice(
            make_cube_model("first_retry.stl", 30.0),
            make_slice_config(),
            retry_output,
            &retry_control);
        REQUIRE(!retry.viewer.moves.empty());
        REQUIRE(std::filesystem::is_regular_file(retry_output));
    }

    SECTION("cancel the second run while the first exports")
    {
        TemporaryRoot root;
        const std::filesystem::path first_output = root.path() / "first.gcode";
        const std::filesystem::path second_output = root.path() / "second.gcode";
        const ConcurrentSliceResult result = run_concurrent_pair(first_output, second_output, false);

        REQUIRE(result.first_error == nullptr);
        REQUIRE(result.first.has_value());
        REQUIRE(result.first_progress_events > 0);
        REQUIRE(std::filesystem::is_regular_file(first_output));
        REQUIRE(std::filesystem::file_size(first_output) > 0);
        REQUIRE(result.second_cancel_requested);
        REQUIRE(result.second_progress_events > 0);
        REQUIRE(result.second_error != nullptr);
        REQUIRE(!result.second.has_value());

        Slic3r::Rest::SliceControl retry_control;
        const std::filesystem::path retry_output = root.path() / "second_retry.gcode";
        const Slic3r::Rest::SliceResult retry = Slic3r::Rest::Engine::instance().slice(
            make_cube_model("second_retry.stl", 22.0),
            make_slice_config(),
            retry_output,
            &retry_control);
        REQUIRE(!retry.viewer.moves.empty());
        REQUIRE(std::filesystem::is_regular_file(retry_output));
    }
}

TEST_CASE("viewer pages retain complete paths and engine speed semantics", "[rest_engine]")
{
    Slic3r::GCodeProcessorResult result;
    result.moves.emplace_back(
        1, Slic3r::EMoveType::Travel, Slic3r::GCodeExtrusionRole::None, 0, 0,
        Slic3r::GCodeProcessorResult::OBJECT_INSTANCE_ID_NONE,
        Slic3r::Vec3f(10.0f, 10.0f, 0.2f), 0.0f, 40.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0, false);
    result.moves.emplace_back(
        2, Slic3r::EMoveType::Retract, Slic3r::GCodeExtrusionRole::None, 0, 0,
        Slic3r::GCodeProcessorResult::OBJECT_INSTANCE_ID_NONE,
        Slic3r::Vec3f(10.0f, 10.0f, 0.2f), -1.0f, 35.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0, false);
    result.moves.emplace_back(
        3, Slic3r::EMoveType::Extrude, Slic3r::GCodeExtrusionRole::Perimeter, 0, 0,
        0, Slic3r::Vec3f(20.0f, 10.0f, 0.2f), 0.5f, 30.0f, 0.45f, 0.2f, 0.09f,
        0.0f, 210.0f, 2.0f, 0, false);

    const Slic3r::Rest::ViewerPage page = Slic3r::Rest::make_viewer_page(result, 1, 2);
    REQUIRE(page.total_moves == 3);
    REQUIRE(page.moves.size() == 2);
    REQUIRE(page.moves[0].points.empty());
    REQUIRE(page.moves[0].feedrate_start_mm_s == 35.0f);
    REQUIRE(page.moves[0].feedrate_end_mm_s == 35.0f);
    REQUIRE(page.moves[0].time_s == 1.0f);
    REQUIRE(page.moves[1].points.size() == 2);
    REQUIRE((page.moves[1].points.front() - Slic3r::Vec3f(10.0f, 10.0f, 0.2f)).norm() < 0.0001f);
    REQUIRE((page.moves[1].points.back() - Slic3r::Vec3f(20.0f, 10.0f, 0.2f)).norm() < 0.0001f);
    REQUIRE(page.moves[1].feedrate_start_mm_s == 30.0f);
    REQUIRE(page.moves[1].feedrate_end_mm_s == 30.0f);
    REQUIRE(page.moves[1].time_s == 2.0f);
}

// A cancellation requested before the print exists must not be lost: the
// slice stops as soon as the print is attached and publishes no output.
TEST_CASE("a slice cancelled before it starts is not run", "[rest_engine]")
{
    ensure_engine_initialized();
    TemporaryRoot root;
    const std::filesystem::path output = root.path() / "cancelled.gcode";
    Slic3r::Rest::SliceControl control;
    control.request_cancel();
    REQUIRE_THROWS_AS(
        Slic3r::Rest::Engine::instance().slice(
            make_cube_model("cancelled.stl", 20.0),
            make_slice_config(),
            output,
            &control),
        Slic3r::CanceledException);
    REQUIRE(!std::filesystem::exists(output));
}