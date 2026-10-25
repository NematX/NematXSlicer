/* The INFILL_PATTERN probe plugin "test.pattern.probe" shared by tests of steps
   that call patterns through payload callbacks (test_infill_pattern_init.cpp for
   STEP_INFILL).

   The probe is registered once by ensure_plugin_test_runtime_initialized() (plugin_test_helpers.cpp) and is
   inactive outside the tests that activate it. It writes three horizontal lines across the object bounding box of the
   recipe, in a sortable child group of the output. The tests drive it through the inline state below, which they must
   reset themselves when they start and finish:
   - pattern_setup_count counts the setup() calls; pattern_fail_setup makes setup() throw;
   - pattern_fail_run makes run() throw after a partial write: 1 writes a group in a child of the output first, 2 writes
     a path directly on the output (which turns it into a leaf) first; pattern_wrote_before_failure tells that the
     partial write happened;
   - pattern_cancel_print, when set, makes run() cancel that print as a user would, then return without writing;
   - pattern_rendezvous makes each run wait (wait_limit at most) until two runs are seen at the same time, to force an
     overlap of callers; rendezvous_timed_out is set when a bounded wait ran out.
   The C++ exceptions of the probe are turned into plugin errors by the *_safe wrappers of PluginBase, as for any plugin. */
#ifndef probe_pattern_plugin_hpp_
#define probe_pattern_plugin_hpp_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <thread>
#include <vector>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Plugins/Infill/PatternPluginHelpers.hpp"
#include "libslic3r/Print.hpp"

namespace Slic3r::Test::ProbePatternPlugin {

// Plugin id of the probe pattern.
inline constexpr const char *probe_pattern_id = "test.pattern.probe";

// State of the probe pattern, set by the tests.
inline std::atomic<int> pattern_setup_count { 0 };
inline std::atomic<int> pattern_active_runs { 0 };
inline std::atomic<int> pattern_max_active_runs { 0 };
// Each run waits until another run is seen, so that the overlap of two callers is forced.
inline std::atomic_bool pattern_rendezvous { false };
inline std::atomic_bool pattern_fail_setup { false };
// How the run fails: 0 never; 1 after writing a group of paths in a child of the output; 2 after writing a path
// directly on the output, which makes it a leaf.
inline std::atomic<int> pattern_fail_run { 0 };
inline std::atomic_bool pattern_wrote_before_failure { false };
// The print the run cancels instead of generating (null: none).
inline std::atomic<Print *> pattern_cancel_print { nullptr };
// Set when a bounded wait of the probes ran out, so that the tests fail with a diagnosable message.
inline std::atomic_bool rendezvous_timed_out { false };
// Longest wait of a probe for another caller.
inline constexpr std::chrono::seconds wait_limit { 10 };

// The pattern the tests control: it counts its setup() calls, writes three lines across the bounding box of the
// recipe, and can observe overlapping runs, fail, or cancel the print.
class ProbePattern final : public slic3r_api::PluginBase
{
public:
    // Create the pattern; it joins no user option list.
    explicit ProbePattern(orchestrator_handle *orchestrator) : PluginBase(orchestrator) {}

private:
    // This provider's plugin id.
    const char *id_impl() const noexcept override { return probe_pattern_id; }
    // The step of the service this plugin implements.
    slicing_step_t step_impl() const noexcept override { return INFILL_PATTERN; }
    // No dependencies on other plugin ids.
    const char *const *dependencies_impl() const noexcept override { return nullptr; }
    // Last of the patterns, so that it is never the fallback of an unknown id.
    int32_t priority_impl() const noexcept override { return 10000; }

    // Count the call and throw when pattern_fail_setup is set.
    void setup_impl(const plugin_run_context *, uint32_t) const override
    {
        ++pattern_setup_count;
        if (pattern_fail_setup)
            throw std::runtime_error("probe pattern setup failure");
    }

    // Record the highest number of runs seen at the same time and wait (wait_limit at most) until it is two.
    static void rendezvous_with_another_run()
    {
        const int active = ++pattern_active_runs;
        int max_seen = pattern_max_active_runs.load();
        while (active > max_seen && !pattern_max_active_runs.compare_exchange_weak(max_seen, active)) {}
        const std::chrono::steady_clock::time_point deadline = std::chrono::steady_clock::now() + wait_limit;
        while (pattern_max_active_runs.load() < 2 && std::chrono::steady_clock::now() < deadline)
            std::this_thread::yield();
        if (pattern_max_active_runs.load() < 2)
            rendezvous_timed_out = true;
        --pattern_active_runs;
    }

    // Write three lines into the output (after the rendezvous when asked), or cancel the print, or fail after
    // a partial write as pattern_fail_run says.
    void run_impl(const plugin_run_context *run_ctx) const override
    {
        const run_ctx_infill_pattern *ctx = plugin_ctx_as_infill_pattern(run_ctx);
        if (ctx == nullptr || ctx->params == nullptr || ctx->output == nullptr || run_ctx->plugin_storage == nullptr)
            return;
        if (Print *print = pattern_cancel_print.load()) {
            print->cancel();
            return;
        }
        if (pattern_rendezvous)
            rendezvous_with_another_run();

        // Three lines across the bounding box of the recipe.
        // The lines are published as a sortable group, like the real patterns do.
        const c_bounding_box &box = ctx->params->object_bounding_box;
        slic3r_api::StoredExtrusionEntity group(run_ctx->plugin_storage);
        group.set_flags(group.flags() | RAW_EXTRUSION_FLAG_SORTABLE | RAW_EXTRUSION_FLAG_REVERSIBLE);
        for (int line = 1; line <= 3; ++line) {
            const coord_t y = box.min.y + (box.max.y - box.min.y) * line / 4;
            slic3r_api::Infill::PatternHelpers::append_polyline(group.mutable_view(), run_ctx->plugin_storage,
                                                                std::vector<c_point>{{box.min.x, y}, {box.max.x, y}}, *ctx->params);
            if (line == 1 && pattern_fail_run == 1) {
                // Fail after a partial write: the first line is already in the output, in a child.
                slic3r_api::MutableExtrusionEntity(ctx->output).append_child_move(group.mutable_view());
                pattern_wrote_before_failure = true;
                throw std::runtime_error("probe pattern run failure");
            }
            if (line == 1 && pattern_fail_run == 2) {
                // Fail after writing a path directly on the output, which becomes a leaf.
                slic3r_api::StoredExtrusionEntity leaf(run_ctx->plugin_storage, std::vector<c_point>{{box.min.x, y}, {box.max.x, y}});
                extrusion_move_from(ctx->output, leaf.mutable_handle());
                pattern_wrote_before_failure = true;
                throw std::runtime_error("probe pattern run failure");
            }
        }
        slic3r_api::MutableExtrusionEntity(ctx->output).append_child_move(group.mutable_view());
    }
};

// The process-lifetime probe shared by registration and activation.
inline ProbePattern &pattern_probe()
{
    static ProbePattern probe(reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
    return probe;
}

} // namespace Slic3r::Test::ProbePatternPlugin

#endif // probe_pattern_plugin_hpp_
