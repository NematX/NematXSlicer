/*
 * Headless libslic3r runtime and viewer move projection.
 *
 * Engine initialization installs the process-wide resource paths, registers
 * built-in plugins, and finalizes the FFF/SLA configuration definitions before
 * any print is created. Slicing then follows the same apply/validate/process/
 * export sequence as the command-line application. The viewer projection keeps
 * the raw engine move beside a complete two-point path for each spatial move.
 */

#pragma once

#include <atomic>
#include <cstddef>
#include <filesystem>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/PrintStatus.hpp"

namespace Slic3r {
class Print;
}

namespace Slic3r::Rest {

struct EnginePaths {
    std::filesystem::path resource_directory;
    std::filesystem::path data_directory;
    std::filesystem::path binary_file;
};

struct ViewerMove {
    GCodeProcessorResult::MoveVertex source;
    std::vector<Vec3f> points;
    float feedrate_start_mm_s = 0.0f;
    float feedrate_end_mm_s = 0.0f;
    float time_s = 0.0f;
};

struct ViewerPage {
    std::size_t total_moves = 0;
    std::vector<ViewerMove> moves;
};

struct SliceResult {
    std::filesystem::path output_file;
    GCodeProcessorResult viewer;
};

// Match Plater::priv::process_slicing_status formatting without translation; malformed templates remain readable.
std::string format_status_text(const std::string &text, const std::vector<std::string> &args);

// Caller-side control of one slice, shared between the thread running
// Engine::slice() and the threads that observe or cancel it.
// - Cancellation may be requested at any time, including before the print is
//   created: the request is remembered and applied as soon as the print is
//   attached, so an early cancel is never lost.
// - Progress is the engine status stream of the print (main and secondary
//   channels, plugin progress included), delivered on producer threads.
// One control drives one slice at a time; the progress callback is read when
// the slice starts.
class SliceControl {
public:
    using ProgressCallback = std::function<void(const PrintStatus::SlicingStatus &)>;

    void set_progress_callback(ProgressCallback callback);
    void request_cancel() noexcept;
    bool cancel_requested() const noexcept;

private:
    friend class Engine;

    // Attach the running print and return the progress callback to install.
    ProgressCallback attach(Print &print);
    void detach() noexcept;

    mutable std::mutex mutex_;
    std::atomic<bool> cancel_requested_{false};
    Print *print_ = nullptr;
    ProgressCallback progress_callback_;
};

class Engine {
public:
    static Engine &instance();

    void initialize(const EnginePaths &paths);
    bool initialized() const noexcept;
    // Slice and export one model. Each call runs with its own plugin run
    // state, so concurrent calls neither cancel nor corrupt each other.
    SliceResult slice(
        Model model,
        DynamicPrintConfig config,
        const std::filesystem::path &output_file,
        SliceControl *control = nullptr) const;

private:
    Engine() = default;

    mutable std::mutex mutex_;
    EnginePaths paths_;
    std::atomic<bool> initialized_{false};
};

ViewerPage make_viewer_page(
    const GCodeProcessorResult &result,
    std::size_t offset,
    std::size_t limit);

} // namespace Slic3r::Rest
