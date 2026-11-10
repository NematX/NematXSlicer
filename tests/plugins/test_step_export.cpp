/*
Tests cover STEP_EXPORT: its place in the pipeline order and checkpoints, the rule that it is queued only by an
explicit export request, the single-use request owned by Print, and the export entry that runs only this step on a
ready G-code reference.

STEP_EXPORT tests
=================

The export is the step that follows STEP_GCODE. It publishes the finished
reference and is queued by an export request only: an invalidation, a full reset
or a replay never queues it. The request is a single-use object of the Print
that carries the configuration copy, the destination and the thumbnail renderer
of one export. The export entry runs this step alone and never replays an
earlier step. Expected values are literals; exported files live in unique
directories below the working directory and are removed at scope exit.

Representative direct calls (branches are not a timeline):

    TEST_CASE
        |-- cube.prepare_reference()
        |-- directory.path()
        |-- read_file()
        |-- cube.writer_runs()
        |-- export_requested_path()
        `-- cube.print.export_result()
*/

#include <catch2/catch.hpp>

#include <algorithm>
#include <functional>
#include <iterator>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/nowide/fstream.hpp>

#include "plugin_test_helpers.hpp"
#include "test_data.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/Exception.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/ScopedTemporaryFile.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"



namespace {

using namespace Slic3r;

constexpr const char *k_not_requested =
    "No export was requested: deposit an export request before running the export.";
constexpr const char *k_reference_not_ready = "The G-code reference is not ready: slice again before exporting.";

// Owns a unique directory below the working directory for the reference and the exported files.
class ScopedDirectory
{
public:
    ScopedDirectory()
        : m_path(boost::filesystem::current_path() / boost::filesystem::unique_path("slic3r-step-export-%%%%-%%%%-%%%%"))
    {
        boost::filesystem::create_directories(m_path);
    }

    ~ScopedDirectory()
    {
        boost::system::error_code ignored;
        boost::filesystem::remove_all(m_path, ignored);
    }

    ScopedDirectory(const ScopedDirectory &) = delete;
    ScopedDirectory &operator=(const ScopedDirectory &) = delete;

    const boost::filesystem::path &path() const { return m_path; }

private:
    boost::filesystem::path m_path;
};

// Read one whole file as bytes.
std::string read_file(const boost::filesystem::path &path)
{
    boost::nowide::ifstream input(path.string().c_str(), std::ios::in | std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

// Run the requested export and return the local file that the exporter reported.
std::string export_requested_path(Print &print)
{
    Orchestrator::instance().export_requested(print);
    return print.export_result().local_path;
}

// A sliced cube with the simple exporter. The number of runs of the G-code writer plugin since the construction tells
// whether STEP_GCODE ran again; the main percentages published while ecording is set tell how the bar moved.
class SlicedCube
{
public:
    // The settings are applied over a Marlin configuration without seam visibility computation.
    explicit SlicedCube(const std::vector<std::pair<std::string, std::string>> &settings = {})
    {
        Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
        m_config = DynamicPrintConfig::full_print_config();
        m_config.set_deserialize_strict({
            {"gcode_flavor", "marlin2"},
            {"gcode_exporter_plugin", "gcode.exporter.simple"},
            {"seam_visibility", "0"},
            {"binary_gcode", "1"}});
        for (const std::pair<std::string, std::string> &setting : settings)
            m_config.set_deserialize_strict(setting.first, setting.second);
        DynamicPrintConfig applied = m_config;
        Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, applied);
        print.status().set_callback([this](const PrintBase::SlicingStatus &status) {
            if ((status.flags & PrintBase::SlicingStatus::SECONDARY_STATE) != 0 || status.percent < 0)
                return;
            last_main_percent = status.percent;
            if (recording)
                main_percents.push_back(status.percent);
        });
        writer_runs_at_start = writer_run_count();
        print.process();
    }

    // Total runs of the G-code writer plugin since the process started.
    static uint64_t writer_run_count()
    {
        const Plugin *writer = Orchestrator::instance().get_plugin("gcode.printing_plan_file_writer");
        REQUIRE(writer != nullptr);
        return writer->get_run_count();
    }

    // Runs of the writer since this cube was built.
    uint64_t writer_runs() const { return writer_run_count() - writer_runs_at_start; }

    // Produce the reference the way the preview does and return its path.
    boost::filesystem::path prepare_reference(const boost::filesystem::path &directory)
    {
        const boost::filesystem::path reference = directory / "reference.gcode";
        Orchestrator::instance().reset_plugin_cancel();
        Orchestrator::instance().export_gcode(print, reference.string(), nullptr);
        return reference;
    }

    // A request for the given destination, with the current configuration copy.
    Print::ExportRequest request(const boost::filesystem::path &destination,
                                 ThumbnailsGeneratorCallback thumbnails = ThumbnailsGeneratorCallback())
    {
        Print::ExportRequest result = print.make_export_request(std::move(thumbnails));
        result.set_output_path(destination.string());
        return result;
    }

    // Apply a change of the temperature, which replays STEP_GCODE once the reference exists.
    Print::ApplyStatus apply_temperature_change()
    {
        DynamicPrintConfig changed = m_config;
        changed.set_deserialize_strict("temperature", "233");
        return print.apply(model, changed);
    }

    Print print;
    Model model;
    // Last main percentage published, and every one published while ecording is set.
    int last_main_percent = -1;
    bool recording = false;
    std::vector<int> main_percents;

private:
    DynamicPrintConfig m_config;
    uint64_t           writer_runs_at_start = 0;
};

} // namespace

TEST_CASE("STEP_EXPORT follows STEP_GCODE and resumes from the reference checkpoint",
          "[plugins][export][step-export][step-pipeline]")
{
    using Steps::Checkpoint;

    const std::vector<slicing_step_t> &order = Steps::execution_order();
    REQUIRE(order.size() >= 2);
    CHECK(order[order.size() - 2] == STEP_GCODE);
    CHECK(order.back() == STEP_EXPORT);
    CHECK(int(STEP_EXPORT) == 2100);
    CHECK(Steps::executes_before(STEP_GCODE, STEP_EXPORT));
    CHECK(Steps::checkpoint_before_step(STEP_EXPORT) == Checkpoint::GCode);
    CHECK(Steps::checkpoint_before_step(STEP_GCODE) == Checkpoint::Plan);
    CHECK(Steps::checkpoint_resume_step(Checkpoint::GCode) == STEP_EXPORT);
    // No invalidation reaches the export through the dependency graph.
    CHECK(Steps::dependent_steps_closure(STEP_GCODE).empty());
    const std::vector<slicing_step_t> from_slicing = Steps::dependent_steps_closure(STEP_SLICING);
    CHECK(std::find(from_slicing.begin(), from_slicing.end(), STEP_EXPORT) == from_slicing.end());
    CHECK(std::find(from_slicing.begin(), from_slicing.end(), STEP_GCODE) != from_slicing.end());
}

TEST_CASE("STEP_EXPORT is never queued by a reset or an invalidation",
          "[plugins][export][step-export][step-pipeline]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    print.reset_step_execution_plan_all();
    CHECK(print.should_execute_step(STEP_GCODE));
    CHECK_FALSE(print.should_execute_step(STEP_EXPORT));

    for (const slicing_step_t step : Steps::execution_order())
        print.mark_step_executed(step);
    CHECK(print.current_checkpoint() == Steps::Checkpoint::GCode);

    print.mark_step_and_dependents_for_execution(STEP_SLICING);
    CHECK(print.should_execute_step(STEP_GCODE));
    CHECK_FALSE(print.should_execute_step(STEP_EXPORT));
    CHECK(print.is_step_done(STEP_EXPORT));

    print.reset_step_execution_plan_all();
    CHECK_FALSE(print.should_execute_step(STEP_EXPORT));
}

TEST_CASE("Only an export request queues STEP_EXPORT after a slicing",
          "[plugins][export][step-export]")
{
    ScopedDirectory directory;
    SlicedCube cube;
    cube.prepare_reference(directory.path());
    // The reference is ready and no export was asked for.
    CHECK(cube.print.is_step_done(STEP_GCODE));
    CHECK_FALSE(cube.print.should_execute_step(STEP_EXPORT));
    CHECK(cube.print.current_checkpoint() == Steps::Checkpoint::GCode);

    SECTION("a request queues it and keeps the reference checkpoint") {
        cube.print.request_export(cube.request(directory.path() / "exported.gcode"));
        CHECK(cube.print.should_execute_step(STEP_EXPORT));
        CHECK_FALSE(cube.print.is_step_done(STEP_EXPORT));
        CHECK(cube.print.current_checkpoint() == Steps::Checkpoint::GCode);
        CHECK(cube.print.is_step_done(STEP_GCODE));
    }
    SECTION("an invalidation of STEP_GCODE does not queue it") {
        CHECK(cube.apply_temperature_change() == Print::APPLY_STATUS_INVALIDATED);
        CHECK(cube.print.should_execute_step(STEP_GCODE));
        CHECK_FALSE(cube.print.should_execute_step(STEP_EXPORT));
    }
}

TEST_CASE("An export on a ready reference runs STEP_EXPORT alone",
          "[plugins][export][step-export]")
{
    ScopedDirectory directory;
    SlicedCube cube;
    const boost::filesystem::path reference = cube.prepare_reference(directory.path());
    const std::string reference_text = read_file(reference);
    REQUIRE_FALSE(reference_text.empty());
    const PrintStateBase::TimeStamp gcode_timestamp = cube.print.step_state_with_timestamp(STEP_GCODE).timestamp;
    REQUIRE(cube.writer_runs() == 1);

    const boost::filesystem::path destination = directory.path() / "exported.gcode";
    cube.print.request_export(cube.request(destination));
    std::string published;
    CHECK_NOTHROW(published = export_requested_path(cube.print));

    CHECK(published == destination.string());
    CHECK(cube.print.export_result().local_path == destination.string());
    const std::string exported = read_file(destination);
    CHECK(exported.find("; generated by ") == 0);
    CHECK(exported.find(reference_text) != std::string::npos);
    // The writer did not run again and the reference is untouched.
    CHECK(cube.writer_runs() == 1);
    CHECK(cube.print.step_state_with_timestamp(STEP_GCODE).timestamp == gcode_timestamp);
    CHECK(read_file(reference) == reference_text);
    CHECK(cube.print.is_step_done(STEP_EXPORT));
    CHECK_FALSE(cube.print.should_execute_step(STEP_EXPORT));
    CHECK(cube.print.current_checkpoint() == Steps::Checkpoint::GCode);
}

TEST_CASE("Two identical requests export twice without replaying the writer",
          "[plugins][export][step-export]")
{
    ScopedDirectory directory;
    SlicedCube cube({{"thumbnails", "4x4"}, {"thumbnails_format", "PNG"}});
    const boost::filesystem::path reference = cube.prepare_reference(directory.path());
    const boost::filesystem::path destination = directory.path() / "exported.gcode";
    uint32_t renders = 0;
    const ThumbnailsGeneratorCallback renderer = [&renders](const ThumbnailsParams &params) {
        ++renders;
        ThumbnailData image;
        image.set(uint32_t(params.sizes.front().x()), uint32_t(params.sizes.front().y()));
        std::fill(image.pixels.begin(), image.pixels.end(), 255);
        return ThumbnailsList{image};
    };

    cube.print.request_export(cube.request(destination, renderer));
    CHECK_NOTHROW(Orchestrator::instance().export_requested(cube.print));
    const std::string first_content = read_file(destination);
    REQUIRE_FALSE(first_content.empty());
    CHECK(renders == 1);

    // A witness replaces the first output: the second, identical request must replace the witness in turn.
    {
        boost::nowide::ofstream witness(destination.string().c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
        witness << "witness";
    }
    cube.print.request_export(cube.request(destination, renderer));
    CHECK(cube.print.should_execute_step(STEP_EXPORT));
    std::string published;
    CHECK_NOTHROW(published = export_requested_path(cube.print));

    CHECK(published == destination.string());
    CHECK(renders == 2);
    CHECK(read_file(destination) == first_content);
    CHECK(cube.writer_runs() == 1);
    CHECK(cube.print.is_step_done(STEP_GCODE));
    CHECK(boost::filesystem::exists(reference));
}

TEST_CASE("The published path lasts until the next request or the next clear",
          "[plugins][export][step-export]")
{
    ScopedDirectory directory;
    SlicedCube cube;
    cube.prepare_reference(directory.path());
    const boost::filesystem::path destination = directory.path() / "exported.gcode";

    cube.print.request_export(cube.request(destination));
    CHECK_NOTHROW(Orchestrator::instance().export_requested(cube.print));
    CHECK(cube.print.export_result().local_path == destination.string());

    // An invalidation of the reference leaves the path of a file that was already published.
    CHECK(cube.apply_temperature_change() == Print::APPLY_STATUS_INVALIDATED);
    CHECK(cube.print.export_result().local_path == destination.string());

    // A new request forgets it.
    cube.print.request_export(cube.request(destination));
    CHECK(cube.print.export_result().local_path.empty());

    cube.print.clear();
    CHECK(cube.print.export_result().local_path.empty());
}

TEST_CASE("A private file guard removes its file unless released",
          "[plugins][export][step-export]")
{
    ScopedDirectory directory;
    const boost::filesystem::path removed = directory.path() / "removed.gcode";
    const boost::filesystem::path kept = directory.path() / "kept.gcode";
    const boost::filesystem::path missing = directory.path() / "missing.gcode";
    for (const boost::filesystem::path &path : {removed, kept}) {
        boost::nowide::ofstream file(path.string().c_str(), std::ios::out | std::ios::binary);
        file << "x";
    }
    {
        ScopedTemporaryFile guard_removed(removed.string());
        ScopedTemporaryFile guard_kept(kept.string());
        ScopedTemporaryFile guard_missing(missing.string());
        guard_kept.release();
        CHECK(boost::filesystem::exists(removed));
    }
    CHECK_FALSE(boost::filesystem::exists(removed));
    CHECK(boost::filesystem::exists(kept));
    CHECK_FALSE(boost::filesystem::exists(missing));
}

TEST_CASE("A guard removes an export that was written and then cancelled",
          "[plugins][export][step-export]")
{
    ScopedDirectory directory;
    SlicedCube cube;
    cube.prepare_reference(directory.path());
    const boost::filesystem::path destination = directory.path() / "private.gcode";
    cube.print.status().set_callback([&cube](const PrintBase::SlicingStatus &status) {
        if (status.percent == 100 && !cube.print.export_result().local_path.empty())
            cube.print.cancel();
    });
    {
        ScopedTemporaryFile guard(destination.string());
        cube.print.request_export(cube.request(destination));
        CHECK_THROWS_AS(Orchestrator::instance().export_requested(cube.print), CanceledException);
        // The file exists until the owner lets the guard go, as the upload and copy paths do on failure.
        CHECK(boost::filesystem::exists(destination));
    }
    CHECK_FALSE(boost::filesystem::exists(destination));
}
TEST_CASE("An export fails when the reference was invalidated since the request",
          "[plugins][export][step-export]")
{
    ScopedDirectory directory;
    SlicedCube cube;
    const boost::filesystem::path reference = cube.prepare_reference(directory.path());
    const boost::filesystem::path destination = directory.path() / "exported.gcode";
    cube.print.request_export(cube.request(destination));
    SECTION("an apply that invalidates the reference") {
        REQUIRE(cube.apply_temperature_change() == Print::APPLY_STATUS_INVALIDATED);
    }
    SECTION("a replay request that leaves the reference file in place") {
        // The step is queued again but its outputs are not destroyed yet: the file and its path still exist.
        cube.print.request_replay_from_worker(Steps::Checkpoint::Plan);
        REQUIRE(boost::filesystem::exists(reference));
    }
    REQUIRE(cube.print.should_execute_step(STEP_GCODE));

    CHECK_THROWS_WITH(Orchestrator::instance().export_requested(cube.print), k_reference_not_ready);

    // Nothing was written, and the writer was not replayed to serve the export.
    CHECK_FALSE(boost::filesystem::exists(destination));
    CHECK(cube.writer_runs() == 1);
    CHECK(cube.print.should_execute_step(STEP_GCODE));
    CHECK_FALSE(cube.print.is_step_done(STEP_GCODE));
    CHECK(cube.print.export_result().local_path.empty());
}

TEST_CASE("An export without request or destination fails clearly and writes nothing",
          "[plugins][export][step-export]")
{
    ScopedDirectory directory;
    SlicedCube cube;
    cube.prepare_reference(directory.path());

    // No request was deposited: the export is not queued.
    CHECK_THROWS_WITH(Orchestrator::instance().export_requested(cube.print), k_not_requested);

    // A request without destination and without requester: the built-in exporter asks for the destination and nobody can answer.
    Print::ExportRequest request = cube.print.make_export_request(ThumbnailsGeneratorCallback());
    CHECK(request.output_path().empty());
    cube.print.request_export(std::move(request));
    CHECK_THROWS_WITH(Orchestrator::instance().export_requested(cube.print),
                      "This export needs inputs, but no input form is available.");
    CHECK(cube.print.export_result().local_path.empty());

    // A requester that answers an empty destination gives it to the exporter, which refuses it.
    request = cube.print.make_export_request(ThumbnailsGeneratorCallback());
    size_t asked = 0;
    ExportInputForm seen;
    request.inputs = [&](ExportInputForm &form) {
        ++asked;
        seen = form;
        form.values.set_key_value("output_path", new ConfigOptionString(std::string()));
        return true;
    };
    cube.print.request_export(std::move(request));
    CHECK_THROWS_WITH(Orchestrator::instance().export_requested(cube.print),
                      "G-code export requires a non-empty destination (output_path).");
    // The refusal comes from the answer: the exporter did ask for a path to save.
    CHECK(asked == 1);
    REQUIRE(seen.definitions.size() == 1);
    CHECK(seen.definitions[0].opt_key == "output_path");
    CHECK(seen.definitions[0].gui_type == ConfigOptionDef::GUIType::save_path);
    CHECK(cube.print.export_result().local_path.empty());
    CHECK(cube.writer_runs() == 1);
}

TEST_CASE("The request is isolated from the full print configuration and from apply",
          "[plugins][export][step-export]")
{
    ScopedDirectory directory;
    SlicedCube cube;
    cube.prepare_reference(directory.path());
    const boost::filesystem::path destination = directory.path() / "exported.gcode";

    // The destination exists in the definitions as a volatile setting, but no configuration of the Print holds it.
    const ConfigOptionDef *definition = PrintConfigDef::instance().get("output_path");
    REQUIRE(definition != nullptr);
    CHECK(static_cast<uint32_t>(definition->nature) == static_cast<uint32_t>(ConfigOptionNature::Volatile));
    CHECK(definition->invalidates_step == STEP_NONE);
    CHECK(definition->cli == ConfigOptionDef::nocli);
    CHECK(cube.print.full_print_config().option("output_path") == nullptr);

    Print::ExportRequest request = cube.request(destination);
    REQUIRE(request.config.option("output_path") != nullptr);
    CHECK(request.output_path() == destination.string());
    CHECK(cube.print.full_print_config().option("output_path") == nullptr);

    // Depositing and applying the same inputs leaves both sides as they were.
    cube.print.request_export(std::move(request));
    CHECK(cube.print.full_print_config().option("output_path") == nullptr);
    CHECK(cube.apply_temperature_change() == Print::APPLY_STATUS_INVALIDATED);
    CHECK(cube.print.full_print_config().option("output_path") == nullptr);
    const std::optional<Print::ExportRequest> taken = cube.print.take_export_request();
    REQUIRE(taken.has_value());
    CHECK(taken->output_path() == destination.string());
    // The request holds the configuration of the moment it was made, not the one applied afterwards.
    CHECK(taken->config.opt_serialize("temperature") != "233");
    CHECK(cube.print.full_print_config().opt_serialize("temperature") == "233");
    CHECK_FALSE(cube.print.take_export_request().has_value());
}

TEST_CASE("A request does not make the next apply stop the worker",
          "[plugins][export][step-export]")
{
    ScopedDirectory directory;
    SlicedCube cube;
    cube.prepare_reference(directory.path());
    size_t stops = 0;
    cube.print.set_cancel_callback([&stops]() { ++stops; });

    cube.print.request_export(cube.request(directory.path() / "exported.gcode"));
    // Applying the unchanged inputs changes nothing and must not interrupt the worker.
    CHECK(cube.print.apply(cube.model, cube.print.full_print_config()) == Print::APPLY_STATUS_UNCHANGED);
    CHECK(stops == 0);
    cube.print.set_cancel_callback([]() {});
}

TEST_CASE("The preview does not count STEP_EXPORT and the export never moves the main bar back",
          "[plugins][export][step-export]")
{
    ScopedDirectory directory;
    SlicedCube cube;
    // The request is queued before the preview ends: the preview must still reach 100 %.
    cube.print.request_export(cube.request(directory.path() / "exported.gcode"));
    cube.prepare_reference(directory.path());
    CHECK(cube.print.pipeline_progress().percent() == 100);
    CHECK(cube.print.should_execute_step(STEP_EXPORT));

    // The export continues from the percentage already shown: the main bar never goes back, and ends at 100 %.
    const int before = cube.print.pipeline_progress().percent();
    cube.recording = true;
    CHECK_NOTHROW(Orchestrator::instance().export_requested(cube.print));
    cube.recording = false;
    REQUIRE_FALSE(cube.main_percents.empty());
    CHECK(*std::min_element(cube.main_percents.begin(), cube.main_percents.end()) >= before);
    CHECK(cube.main_percents.back() == 100);
    CHECK(cube.print.pipeline_progress().percent() == 100);
}

// Export the way the worker does after a slicing: process(), then the reference (export_gcode), then the request.
// Return the last main percentage published before the export, and leave the percentages of the export in the cube.
static int export_after_worker_preview(SlicedCube &cube, const boost::filesystem::path &directory)
{
    cube.print.process();
    cube.prepare_reference(directory);
    const int before = cube.last_main_percent;
    cube.print.request_export(cube.request(directory / "exported.gcode"));
    cube.recording = true;
    CHECK_NOTHROW(Orchestrator::instance().export_requested(cube.print));
    cube.recording = false;
    return before;
}

TEST_CASE("The main bar does not go back when the worker finds the reference already prepared",
          "[plugins][export][step-export]")
{
    ScopedDirectory directory;
    SlicedCube cube;
    cube.prepare_reference(directory.path());
    // process() now has nothing to run and resets its plan; export_gcode() reuses the reference and shows 100 %.
    const int before = export_after_worker_preview(cube, directory.path());

    CHECK(before == 100);
    REQUIRE_FALSE(cube.main_percents.empty());
    CHECK(*std::min_element(cube.main_percents.begin(), cube.main_percents.end()) >= before);
    CHECK(cube.main_percents.back() == 100);
}

TEST_CASE("The main bar does not go back when the worker prepares the reference again before the export",
          "[plugins][export][step-export]")
{
    ScopedDirectory directory;
    SlicedCube cube;
    cube.prepare_reference(directory.path());
    // A new G-code is generated for the new temperature, then exported.
    REQUIRE(cube.apply_temperature_change() == Print::APPLY_STATUS_INVALIDATED);
    const int before = export_after_worker_preview(cube, directory.path());

    CHECK(cube.writer_runs() == 2);
    CHECK(before == 100);
    REQUIRE_FALSE(cube.main_percents.empty());
    CHECK(*std::min_element(cube.main_percents.begin(), cube.main_percents.end()) >= before);
    CHECK(cube.main_percents.back() == 100);
}

TEST_CASE("A failed or cancelled attempt consumes its request",
          "[plugins][export][step-export]")
{
    ScopedDirectory directory;
    SlicedCube cube;
    cube.prepare_reference(directory.path());
    const boost::filesystem::path destination = directory.path() / "exported.gcode";

    // A cancellation before the step starts leaves the request unread.
    cube.print.request_export(cube.request(destination));
    cube.print.cancel();
    CHECK_THROWS_AS(Orchestrator::instance().export_requested(cube.print), CanceledException);
    cube.print.restart();
    CHECK_FALSE(boost::filesystem::exists(destination));

    // The attempt used its request up: another export needs a new one.
    CHECK_THROWS_WITH(Orchestrator::instance().export_requested(cube.print), k_not_requested);

    // A new request succeeds.
    cube.print.request_export(cube.request(destination));
    std::string published;
    CHECK_NOTHROW(published = export_requested_path(cube.print));
    CHECK(published == destination.string());
    CHECK(boost::filesystem::exists(destination));
}

TEST_CASE("An export written and then cancelled keeps its path for the caller",
          "[plugins][export][step-export]")
{
    ScopedDirectory directory;
    SlicedCube cube;
    cube.prepare_reference(directory.path());
    const boost::filesystem::path destination = directory.path() / "exported.gcode";

    // The progress reaches 100 % after the exporter returned: cancel at that moment.
    cube.print.status().set_callback([&cube](const PrintBase::SlicingStatus &status) {
        if (status.percent == 100 && !cube.print.export_result().local_path.empty())
            cube.print.cancel();
    });
    cube.print.request_export(cube.request(destination));
    CHECK_THROWS_AS(Orchestrator::instance().export_requested(cube.print), CanceledException);

    // The file was published before the cancellation: the caller can find it and remove it.
    CHECK(cube.print.export_result().local_path == destination.string());
    CHECK(boost::filesystem::exists(destination));
    CHECK_FALSE(cube.print.is_step_done(STEP_EXPORT));

    // The export can be requested again afterwards.
    cube.print.status().set_callback([](const PrintBase::SlicingStatus &) {});
    cube.print.restart();
    cube.print.request_export(cube.request(destination));
    CHECK_NOTHROW(Orchestrator::instance().export_requested(cube.print));
    CHECK(cube.print.is_step_done(STEP_EXPORT));
}

TEST_CASE("The export uses the configuration copy of the request",
          "[plugins][export][step-export]")
{
    ScopedDirectory directory;
    SlicedCube cube;
    cube.prepare_reference(directory.path());
    const boost::filesystem::path destination = directory.path() / "exported.gcode";

    // A value that only the request holds: the Print keeps its own layer height.
    Print::ExportRequest request = cube.request(destination);
    request.config.set_deserialize_strict("layer_height", "0.77");
    const std::string print_layer_height = cube.print.full_print_config().opt_serialize("layer_height");
    REQUIRE(print_layer_height != "0.77");
    cube.print.request_export(std::move(request));
    CHECK_NOTHROW(Orchestrator::instance().export_requested(cube.print));

    const std::string exported = read_file(destination);
    CHECK(exported.find("; layer_height = 0.77\n") != std::string::npos);
    CHECK(exported.find("; layer_height = " + print_layer_height + "\n") == std::string::npos);
    CHECK(cube.print.full_print_config().opt_serialize("layer_height") == print_layer_height);
}

TEST_CASE("The export renders its thumbnails through the callback of the request",
          "[plugins][export][step-export]")
{
    ScopedDirectory directory;
    SlicedCube cube({{"thumbnails", "4x4"}, {"thumbnails_format", "PNG"}});
    cube.prepare_reference(directory.path());
    const boost::filesystem::path destination = directory.path() / "exported.gcode";

    uint32_t calls = 0;
    cube.print.request_export(cube.request(destination, [&calls](const ThumbnailsParams &params) {
        ++calls;
        ThumbnailData image;
        image.set(uint32_t(params.sizes.front().x()), uint32_t(params.sizes.front().y()));
        std::fill(image.pixels.begin(), image.pixels.end(), 255);
        return ThumbnailsList{image};
    }));
    CHECK_NOTHROW(Orchestrator::instance().export_requested(cube.print));

    CHECK(calls == 1);
    CHECK(read_file(destination).find("; thumbnail begin 4x4 ") != std::string::npos);
}

TEST_CASE("A request does not survive Print::clear",
          "[plugins][export][step-export]")
{
    ScopedDirectory directory;
    SlicedCube cube;
    cube.print.request_export(cube.request(directory.path() / "exported.gcode"));
    REQUIRE(cube.print.should_execute_step(STEP_EXPORT));

    cube.print.clear();

    CHECK_FALSE(cube.print.should_execute_step(STEP_EXPORT));
    CHECK_FALSE(cube.print.take_export_request().has_value());
    CHECK(cube.print.export_result().local_path.empty());
}
