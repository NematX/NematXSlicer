/*
 * Headless libslic3r runtime and viewer move projection.
 *
 * The runtime owns the one-time process bootstrap required by the core engine.
 * A slice call is synchronous and keeps its mutable state local, so a caller
 * may run it on its own worker thread. Viewer pages are derived only from data
 * emitted by the engine.
 */

#include "engine.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>

#include <boost/filesystem/path.hpp>
#include <boost/format.hpp>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Config/FFFPrintConfig.hpp"
#include "libslic3r/Plugins/PluginLoader.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/SLA/SLAPrintConfig.hpp"
#include "libslic3r/Utils.hpp"

namespace Slic3r::Rest {

namespace {

bool is_spatial_move(EMoveType type)
{
    return type == EMoveType::Travel || type == EMoveType::Wipe || type == EMoveType::Extrude;
}

void validate_engine_paths(const EnginePaths &paths)
{
    if (paths.resource_directory.empty())
        throw std::invalid_argument("REST engine resource directory must not be empty.");
    if (paths.data_directory.empty())
        throw std::invalid_argument("REST engine data directory must not be empty.");

    std::error_code error;
    if (!std::filesystem::is_directory(paths.resource_directory, error) || error)
        throw std::invalid_argument("REST engine resource directory must exist.");

    const std::filesystem::path default_plugins =
        paths.resource_directory / "plugins" / "default_activated.ini";
    if (!std::filesystem::is_regular_file(default_plugins, error) || error)
        throw std::invalid_argument(
            "REST engine resources must contain plugins/default_activated.ini.");
}

EnginePaths normalized_engine_paths(const EnginePaths &paths)
{
    EnginePaths normalized = paths;
    if (!paths.resource_directory.empty())
        normalized.resource_directory = std::filesystem::absolute(paths.resource_directory);
    if (!paths.data_directory.empty())
        normalized.data_directory = std::filesystem::absolute(paths.data_directory);
    if (normalized.binary_file.empty())
        normalized.binary_file = std::filesystem::current_path() / "slic3r_rest_server";
    else
        normalized.binary_file = std::filesystem::absolute(paths.binary_file);
    return normalized;
}

// Install the process-wide libslic3r paths. They are read by the engine from
// every thread, so this runs once during initialize().
void configure_engine_paths(const EnginePaths &paths)
{
    const std::filesystem::path install_directory = paths.binary_file.parent_path().empty()
        ? std::filesystem::current_path()
        : paths.binary_file.parent_path();
    const std::filesystem::path resource_directory = paths.resource_directory;
    const std::filesystem::path data_directory = paths.data_directory;

    set_install_path(boost::filesystem::path(install_directory.string()));
    set_binary_file(boost::filesystem::path(paths.binary_file.string()));
    set_resources_dir(resource_directory.string());
    set_var_dir((resource_directory / "icons").string());
    set_local_dir((resource_directory / "localization").string());
    set_sys_shapes_dir((resource_directory / "shapes").string());
    set_custom_gcodes_dir((resource_directory / "custom_gcodes").string());
    set_data_dir(data_directory.string());
}

// One-time bootstrap: plugins are loaded before the FFF/SLA configuration
// caches are finalized, because plugin definitions must be part of the final
// option set.
void initialize_config_definitions()
{
    PrintConfigDef::instance_mutable().init_common_params();
    init_fff_params(PrintConfigDef::instance_mutable());
    init_sla_params(PrintConfigDef::instance_mutable());
    load_plugins();
    initialize_fff_print_config_cache();
    initialize_sla_print_config_cache();
    PrintConfigDef::instance_mutable().finalize();
}

} // namespace

std::string format_status_text(const std::string &text, const std::vector<std::string> &args)
{
    try {
        boost::format formatter(text);
        for (const std::string &arg : args)
            formatter % arg;
        return formatter.str();
    } catch (const boost::io::format_error &) {
        return text;
    }
}

Engine &Engine::instance()
{
    static Engine engine;
    return engine;
}

void Engine::initialize(const EnginePaths &paths)
{
    const EnginePaths normalized = normalized_engine_paths(paths);
    std::lock_guard<std::mutex> lock(mutex_);
    if (initialized_) {
        if (paths_.resource_directory != normalized.resource_directory ||
            paths_.data_directory != normalized.data_directory ||
            paths_.binary_file != normalized.binary_file)
            throw std::logic_error("REST engine was already initialized with different paths.");
        return;
    }

    validate_engine_paths(normalized);
    std::error_code error;
    std::filesystem::create_directories(normalized.data_directory, error);
    if (error)
        throw std::runtime_error("Cannot create REST engine data directory: " + error.message());

    configure_engine_paths(normalized);
    initialize_config_definitions();
    paths_ = normalized;
    initialized_ = true;
}

bool Engine::initialized() const noexcept
{
    return initialized_.load();
}

void SliceControl::set_progress_callback(ProgressCallback callback)
{
    std::lock_guard<std::mutex> lock(mutex_);
    progress_callback_ = std::move(callback);
}

// The flag is set under the lock so a concurrent attach() either sees it or
// has already registered the print that is cancelled here.
void SliceControl::request_cancel() noexcept
{
    std::lock_guard<std::mutex> lock(mutex_);
    cancel_requested_.store(true, std::memory_order_release);
    if (print_ != nullptr)
        print_->cancel();
}

bool SliceControl::cancel_requested() const noexcept
{
    return cancel_requested_.load(std::memory_order_acquire);
}

// A cancellation requested before the print existed is applied now.
SliceControl::ProgressCallback SliceControl::attach(Print &print)
{
    std::lock_guard<std::mutex> lock(mutex_);
    print_ = &print;
    if (cancel_requested_.load(std::memory_order_acquire))
        print.cancel();
    return progress_callback_;
}

void SliceControl::detach() noexcept
{
    std::lock_guard<std::mutex> lock(mutex_);
    print_ = nullptr;
}

SliceResult Engine::slice(
    Model model,
    DynamicPrintConfig config,
    const std::filesystem::path &output_file,
    SliceControl *control) const
{
    if (!initialized_)
        throw std::logic_error("REST engine must be initialized before slicing.");
    if (output_file.empty())
        throw std::invalid_argument("REST slice output file must not be empty.");

    std::error_code error;
    if (std::filesystem::exists(output_file, error) && error)
        throw std::runtime_error("Cannot inspect REST slice output file: " + error.message());
    if (std::filesystem::is_directory(output_file, error) && error)
        throw std::runtime_error("Cannot inspect REST slice output path: " + error.message());
    if (std::filesystem::is_directory(output_file))
        throw std::invalid_argument("REST slice output must be a file path.");

    std::filesystem::path parent = output_file.parent_path();
    if (parent.empty())
        parent = std::filesystem::current_path();
    std::filesystem::create_directories(parent, error);
    if (error)
        throw std::runtime_error("Cannot create REST slice output directory: " + error.message());

    // The run context keeps plugin failures, storage and progress of this
    // print apart from any other print sliced at the same time.
    Orchestrator::RunContext run_context;
    Print print;
    Orchestrator &orchestrator = Orchestrator::instance();
    Orchestrator::RunScope run_scope = orchestrator.enter_run(run_context, &print);
    // Detach the control before the print is destroyed, on every exit path.
    struct ControlAttachment {
        SliceControl *control;
        ~ControlAttachment()
        {
            if (control != nullptr)
                control->detach();
        }
    } attachment{control};
    SliceControl::ProgressCallback progress;
    if (control != nullptr)
        progress = control->attach(print);
    const ModelObjectPtrs objects = model.object_ptrs();
    for (ModelObject *object : objects)
        print.auto_assign_extruders(object);

    print.apply(model, std::move(config));
    const std::pair<PrintBase::PrintValidationError, std::string> validation = print.validate();
    if (validation.first != PrintBase::PrintValidationError::pveNone)
        throw std::runtime_error(validation.second);
    if (print.empty())
        throw std::runtime_error("The REST slice model is empty.");

    // TODO(progress rework): callback percent-only updates look like clears; use GUI wakeups or the future libslic3r model.
    if (progress)
        print.set_status_callback(std::move(progress));
    else
        print.set_status_silent();
    print.process();

    GCodeProcessorResult viewer;
    const std::string exported_path = orchestrator.export_gcode(
        print, output_file.string(), &viewer, nullptr);
    return SliceResult{std::filesystem::path(exported_path), std::move(viewer)};
}

// Project engine moves into viewer pages. The path of a spatial move starts at
// the previous spatial position, including when the page starts mid-run; a move
// without a path keeps its engine metadata and exposes no points.
ViewerPage make_viewer_page(
    const GCodeProcessorResult &result,
    std::size_t offset,
    std::size_t limit)
{
    ViewerPage page;
    page.total_moves = result.moves.size();
    if (offset >= result.moves.size() || limit == 0)
        return page;

    // The last spatial move before the page is found backward, so a page costs
    // its own size and reading a result page by page stays linear.
    Vec3f previous_position = Vec3f::Zero();
    for (std::size_t index = offset; index > 0; --index) {
        if (is_spatial_move(result.moves[index - 1].type)) {
            previous_position = result.moves[index - 1].position;
            break;
        }
    }

    const std::size_t remaining = result.moves.size() - offset;
    const std::size_t page_size = std::min(limit, remaining);
    page.moves.reserve(page_size);
    for (std::size_t index = offset; index < offset + page_size; ++index) {
        ViewerMove move;
        move.source = result.moves[index];
        move.feedrate_start_mm_s = move.source.feedrate;
        move.feedrate_end_mm_s = move.source.feedrate;
        move.time_s = move.source.move_time;
        if (is_spatial_move(move.source.type)) {
            move.points.push_back(previous_position);
            move.points.push_back(move.source.position);
            previous_position = move.source.position;
        }
        page.moves.push_back(std::move(move));
    }
    return page;
}

} // namespace Slic3r::Rest
