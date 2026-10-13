#include <catch2/catch.hpp>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <tbb/global_control.h>
#include <tbb/task_arena.h>

#include "plugin_test_helpers.hpp"
#include "probe_pattern_plugin.hpp"
#include "test_data.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_support.h"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/Plugins/Infill/PatternPluginHelpers.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Steps/StepGenerateSupport.hpp"
#include "libslic3r/Surface.hpp"

/* A support family can sample INFILL_PATTERN plugins from STEP_SUPPORT through the pattern callbacks of
   run_ctx_generate_support (ABI 1.2), as a tree family does to place its tips. At this step the support
   layers do not exist yet, so the family passes NULL for layer, island, region island, primary region and
   no-overlap areas. These tests register a probe family that fills a square through the callbacks, and a
   probe pattern that counts its setup() calls, observes overlapping runs and can be told to fail. They check:
   - rectilinear, supportbase and honeycomb produce lines inside the square with NULL handles;
   - a failed call (unknown runtime id, a pattern needing the STEP_INFILL initialization pass) returns 0 and
     generates nothing;
   - the service is built by the first callback only: no setup() without a call, exactly one with several
     concurrent first calls;
   - two threads of one family, and two objects, call the patterns at the same time (the overlap is observed
     by the probe pattern and required) and get the coordinates of a serial run;
   - a pattern whose setup() or run fails does not throw through the callbacks: they return 0 with the output
     cleared, the error is recorded, and the step stops with it once the family returns;
   - the members added in minor version 2 come after the original ones (a family compiled against 1.1 reads
     only print and object), and the header compiles as C with the callback types matching the payload.
   The probes are registered before option finalization and disabled outside these tests. */

// Compiled as C by support_pattern_c_fixture.c.
extern "C" int test_support_pattern_callbacks_c(const run_ctx_generate_support *payload);

namespace {
using namespace Slic3r;
using namespace Slic3r::Test::ProbePatternPlugin;

// Side of the square the probe fills, in mm; it is centered on the origin.
constexpr double square_size_mm = 20.;
// Number of fills of each pattern per probe call.
constexpr size_t fills_per_pattern = 4;
// Number of calls of one sequence: the probe pattern, three real patterns, and two failing calls.
constexpr size_t calls_per_repeat = 6;
// What the probe family does when it runs on an object.
enum class ProbeMode {
    // Nothing: it never calls a pattern.
    None,
    // One sequence of pattern calls.
    Sample,
    // Two std::threads started by the family, each making one sequence of calls.
    Threads,
    // One call of the probe pattern, which is told to fail.
    Fail
};

// What one pattern call returned and generated.
struct FillResult {
    std::string pattern;
    int32_t code = -1;
    // The coordinates of every point of every path, in order.
    std::vector<Point> points;
    size_t paths = 0;
    // True when every point of every path is inside the square.
    bool inside = true;

    bool operator==(const FillResult &other) const
    {
        return pattern == other.pattern && code == other.code && paths == other.paths && inside == other.inside &&
               points == other.points;
    }
};

// What the probe saw for one object, or for one thread of the family.
struct ObjectRecord {
    const object_handle *object = nullptr;
    std::vector<FillResult> results;
    // Runtime id returned for "rectilinear" (Fail mode).
    infill_pattern_runtime_id resolved_id = INFILL_PATTERN_RUNTIME_ID_INVALID;
};

std::mutex record_mutex;
std::vector<ObjectRecord> records;
ProbeMode probe_mode = ProbeMode::None;

// The two threads of the family wait for each other here before their first pattern call.
std::atomic<int> threads_at_barrier { 0 };

// The square every pattern fills.
ExPolygon test_square();
// The complete recipe a support algorithm must provide for one pattern.
raw_infill_pattern_params pattern_params(infill_pattern_runtime_id pattern_id, const BoundingBox &box);
// Fill the square once through the payload callbacks, with NULL layer, island, region island, primary region and no-overlap areas.
FillResult fill_square(const run_ctx_generate_support *payload, const char *pattern, infill_pattern_runtime_id pattern_id);
// The calls of one probe sequence: the probe pattern, three real patterns, an unknown id and a refused pattern, repeated.
std::vector<FillResult> sample_all(const run_ctx_generate_support *payload);

class SupportPatternFamilyProbe final : public slic3r_api::PluginBase
{
public:
    // Create the family that calls INFILL_PATTERN plugins.
    explicit SupportPatternFamilyProbe(orchestrator_handle *orchestrator) : PluginBase(orchestrator) {}

    static constexpr const char *k_id = "test.support.patterns";

private:
    // This provider's plugin id.
    const char *id_impl() const noexcept override { return k_id; }
    // Dispatch the family at STEP_SUPPORT.
    slicing_step_t step_impl() const noexcept override { return STEP_SUPPORT; }
    // No dependencies on other plugin ids.
    const char *const *dependencies_impl() const noexcept override { return nullptr; }
    // Run after the production families, which leave these objects intact.
    int32_t priority_impl() const noexcept override { return 1000; }
    // Call the patterns as the current ProbeMode says and record the results.
    void run_impl(const plugin_run_context *context) const override;
};

// The process-lifetime probes shared by registration and activation.
SupportPatternFamilyProbe &family_probe();
// Wait (wait_limit at most) until both threads of the family are here; sets rendezvous_timed_out on a timeout.
void wait_for_both_threads();

ExPolygon test_square()
{
    const coord_t half = scale_i(square_size_mm / 2.);
    return ExPolygon(Polygon({Point(-half, -half), Point(half, -half), Point(half, half), Point(-half, half)}));
}

raw_infill_pattern_params pattern_params(infill_pattern_runtime_id pattern_id, const BoundingBox &box)
{
    const Flow flow = Flow::new_from_width(0.44f, 0.4f, 0.2f, 1.f);
    raw_infill_pattern_params params = {};
    params.pattern_id = pattern_id;
    params.surface_type = slic3r_api::srf_type().internal().sparse().support().raw();
    params.extrusion_role = RAW_EXTRUSION_ROLE_SUPPORT;
    params.flow.width = flow.scaled_width();
    params.flow.spacing = flow.scaled_spacing();
    params.flow.height = flow.scaled_height();
    params.flow.nozzle_diameter = scale_i(0.4);
    params.flow.spacing_ratio = flow.spacing_ratio();
    params.flow.mm3_per_mm = flow.mm3_per_mm();
    params.density = 0.25f;
    params.flow_mult = 1.f;
    params.connection = C_INFILL_CONNECTION_CONNECTED;
    params.infill_flags = C_INFILL_FLAG_DONT_ADJUST;
    params.spacing = flow.spacing();
    params.bridge_angle = -1.;
    params.layer_height = 0.2;
    params.z = 0.2;
    params.bridge_offset = -1;
    params.fill_resolution = scale_i(0.0125);
    params.anchor_length = 1000.f;
    params.anchor_length_max = 1000.f;
    params.object_bounding_box.min = {box.min.x(), box.min.y()};
    params.object_bounding_box.max = {box.max.x(), box.max.y()};
    return params;
}

FillResult fill_square(const run_ctx_generate_support *payload, const char *pattern, infill_pattern_runtime_id pattern_id)
{
    const ExPolygon square = test_square();
    const Surface surface(stPosInternal | stDensSparse, square);
    const BoundingBox box = get_extents(square);
    const raw_infill_pattern_params params = pattern_params(pattern_id, box);
    ExtrusionEntityCollection output;
    FillResult result;
    result.pattern = pattern;
    result.code = payload->generate_pattern(payload, pattern_id, nullptr, nullptr, nullptr, nullptr,
                                            reinterpret_cast<const surface_handle *>(&surface), nullptr, &params,
                                            reinterpret_cast<extrusion_entity_handle *>(&output));
    // A line may end on the border; allow one nozzle width for the extrusion rounding of the patterns.
    const coord_t margin = scale_i(0.5);
    for (const ArcPolyline &polyline : output.as_polylines()) {
        ++result.paths;
        for (size_t idx = 0; idx < polyline.size(); ++idx) {
            const Point &point = polyline.get_point(idx);
            result.points.push_back(point);
            if (point.x() < box.min.x() - margin || point.x() > box.max.x() + margin ||
                point.y() < box.min.y() - margin || point.y() > box.max.y() + margin)
                result.inside = false;
        }
    }
    return result;
}

std::vector<FillResult> sample_all(const run_ctx_generate_support *payload)
{
    std::vector<FillResult> results;
    for (size_t repeat = 0; repeat < fills_per_pattern; ++repeat) {
        for (const char *pattern : {probe_pattern_id, "rectilinear", "supportbase", "honeycomb"})
            results.push_back(fill_square(payload, pattern, payload->resolve_pattern_id(payload, pattern)));
        // Failed calls: an id the service never created, and a pattern that needs the STEP_INFILL initialization pass.
        results.push_back(fill_square(payload, "unknown id", infill_pattern_runtime_id(100000)));
        results.push_back(fill_square(payload, "lightning", payload->resolve_pattern_id(payload, "lightning")));
    }
    return results;
}

void SupportPatternFamilyProbe::run_impl(const plugin_run_context *context) const
{
    const run_ctx_generate_support *payload = plugin_ctx_as_generate_support(context);
    if (payload == nullptr || payload->resolve_pattern_id == nullptr || payload->generate_pattern == nullptr)
        throw std::runtime_error("Missing support pattern callbacks");

    ObjectRecord record;
    record.object = payload->object;
    switch (probe_mode) {
    case ProbeMode::None:
        return;
    case ProbeMode::Sample:
        record.results = sample_all(payload);
        break;
    case ProbeMode::Threads: {
        // The family's own worker threads call the callbacks at the same time.
        ObjectRecord other = record;
        // Both threads wait for each other before their first resolve_pattern_id(), so that the first
        // accesses (which build the service) really are simultaneous.
        std::thread worker([payload, &other] {
            wait_for_both_threads();
            other.results = sample_all(payload);
        });
        wait_for_both_threads();
        record.results = sample_all(payload);
        worker.join();
        const std::lock_guard<std::mutex> lock(record_mutex);
        records.push_back(std::move(other));
        break;
    }
    case ProbeMode::Fail:
        record.resolved_id = payload->resolve_pattern_id(payload, "rectilinear");
        record.results.push_back(fill_square(payload, probe_pattern_id, payload->resolve_pattern_id(payload, probe_pattern_id)));
        break;
    }
    const std::lock_guard<std::mutex> lock(record_mutex);
    records.push_back(std::move(record));
}

void wait_for_both_threads()
{
    ++threads_at_barrier;
    const std::chrono::steady_clock::time_point deadline = std::chrono::steady_clock::now() + wait_limit;
    while (threads_at_barrier.load() < 2 && std::chrono::steady_clock::now() < deadline)
        std::this_thread::yield();
    if (threads_at_barrier.load() < 2)
        rendezvous_timed_out = true;
}

SupportPatternFamilyProbe &family_probe()
{
    static SupportPatternFamilyProbe probe(reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
    return probe;
}

// Makes the probe family and the probe pattern active, activates the support base pattern that the test runtime
// leaves inactive, and restores everything (including the plugin cancellation a failure leaves) when the test ends.
class ScopedPatternFamily
{
public:
    explicit ScopedPatternFamily(ProbeMode mode)
    {
        Test::Plugins::ensure_plugin_test_runtime_initialized();
        records.clear();
        probe_mode = mode;
        pattern_setup_count = 0;
        pattern_active_runs = 0;
        pattern_max_active_runs = 0;
        pattern_rendezvous = false;
        pattern_fail_setup = false;
        pattern_fail_run = 0;
        pattern_wrote_before_failure = false;
        pattern_cancel_print = nullptr;
        rendezvous_timed_out = false;
        threads_at_barrier = 0;
        m_supportbase_was_active = Orchestrator::instance().is_plugin_active("supportbase");
        REQUIRE(Orchestrator::instance().set_plugin_active("supportbase", true));
        REQUIRE(Orchestrator::instance().set_plugin_active(probe_pattern_id, true));
        REQUIRE(Orchestrator::instance().set_plugin_active(SupportPatternFamilyProbe::k_id, true));
    }
    ~ScopedPatternFamily()
    {
        probe_mode = ProbeMode::None;
        pattern_rendezvous = false;
        pattern_fail_setup = false;
        pattern_fail_run = 0;
        pattern_cancel_print = nullptr;
        Orchestrator::instance().set_plugin_active(SupportPatternFamilyProbe::k_id, false);
        Orchestrator::instance().set_plugin_active(probe_pattern_id, false);
        Orchestrator::instance().set_plugin_active("supportbase", m_supportbase_was_active);
        Orchestrator::instance().reset_plugin_cancel();
        Orchestrator::instance().consume_plugin_messages();
    }
    ScopedPatternFamily(const ScopedPatternFamily &) = delete;
    ScopedPatternFamily &operator=(const ScopedPatternFamily &) = delete;

private:
    bool m_supportbase_was_active = false;
};

// Run STEP_SUPPORT on a print of the given test meshes.
void run_support_step(Print &print, Model &model, std::initializer_list<Test::TestMesh> meshes)
{
    Test::init_print(meshes, print, model);
    Steps::StepGenerateSupport::run_step(Orchestrator::instance(), print);
}

// The message of the exception run_step throws, or "" when it does not throw.
std::string step_failure_message(Print &print)
{
    try {
        Steps::StepGenerateSupport::run_step(Orchestrator::instance(), print);
    } catch (const std::exception &error) {
        return error.what();
    }
    return {};
}

// True when one of the plugin messages is an error containing the text, reported by the given plugin
// ("" matches any plugin, including the host).
bool has_error_message(const std::vector<Orchestrator::PluginMessage> &messages, const std::string &text,
                       const std::string &plugin_id = {})
{
    for (const Orchestrator::PluginMessage &message : messages)
        if (message.level == Orchestrator::PluginMessageLevel::Error && message.message.find(text) != std::string::npos &&
            (plugin_id.empty() || message.plugin_id == plugin_id))
            return true;
    return false;
}

// What a family that sees its pattern call fail leaves behind: the step throws through the runner, naming the
// family, and the plugin messages hold the text of the failing pattern, reported by the pattern plugin.
void check_contained_failure(Print &print, const char *pattern_text)
{
    // Drain earlier diagnostics so that the messages below are only those of this step run.
    (void) Orchestrator::instance().consume_plugin_messages();
    const std::string failure = step_failure_message(print);
    // The callbacks  returned to the family, which recorded its results...
    REQUIRE(records.size() == 1);
    REQUIRE(records.front().results.size() == 1);
    CHECK(records.front().results.front().code == 0);
    CHECK(records.front().results.front().paths == 0);
    // ... then the runner stopped the step: the plugin error requested the plugin cancellation.
    CAPTURE(failure);
    CHECK(!failure.empty());
    CHECK(failure.find("STEP_SUPPORT stopped during ") != std::string::npos);
    CHECK(failure.find("of plugin '" + std::string(SupportPatternFamilyProbe::k_id) + "' after plugin '" +
                       std::string(probe_pattern_id) + "' reported an error.") != std::string::npos);
    const std::vector<Orchestrator::PluginMessage> messages = Orchestrator::instance().consume_plugin_messages();
    CHECK(has_error_message(messages, pattern_text, probe_pattern_id));
    // The pattern plugin's own error is the only error: the callback and the runner add no host message for
    // the contained stop, whatever its wording. The service was built once (one setup()): in the setup-failure
    // case the family's later resolve returned at once instead of rebuilding it and adding a second message.
    CHECK(std::count_if(messages.begin(), messages.end(), [](const Orchestrator::PluginMessage &message) {
        return message.level == Orchestrator::PluginMessageLevel::Error;
    }) == 1);
    CHECK(pattern_setup_count == 1);
}
} // namespace

namespace Slic3r::Test::Plugins {
// Register the probe family before selector settings are finalized (the shared pattern probe is registered by ensure_plugin_test_runtime_initialized(); the test setup deactivates both); rejected registration or activation throws.
void register_support_pattern_service_test_provider(orchestrator_handle *orchestrator)
{
    Orchestrator &host = *reinterpret_cast<Orchestrator *>(orchestrator);
    if (!host.register_plugin(family_probe().c_instance()) || !host.set_plugin_active(SupportPatternFamilyProbe::k_id, true))
        throw std::runtime_error("Cannot register the support pattern service test provider");
}
} // namespace Slic3r::Test::Plugins

TEST_CASE("STEP_SUPPORT patterns fill a square with NULL island and region", "[plugins][support-pattern-service]")
{
    const ScopedPatternFamily activation(ProbeMode::Sample);
    Print print;
    Model model;
    run_support_step(print, model, {Test::TestMesh::cube_20x20x20});

    REQUIRE(records.size() == 1);
    const std::vector<FillResult> &results = records.front().results;
    REQUIRE(results.size() == fills_per_pattern * calls_per_repeat);
    for (const FillResult &result : results) {
        CAPTURE(result.pattern);
        if (result.pattern == "unknown id" || result.pattern == "lightning") {
            // A failed call generates nothing.
            CHECK(result.code == 0);
            CHECK(result.paths == 0);
        } else {
            CHECK(result.code == 1);
            CHECK(result.paths > 0);
            CHECK(result.inside);
        }
    }
    // The same fill gives the same coordinates each time.
    for (size_t idx = calls_per_repeat; idx < results.size(); ++idx)
        CHECK(results[idx] == results[idx % calls_per_repeat]);
}

TEST_CASE("STEP_SUPPORT builds its pattern service on the first callback only", "[plugins][support-pattern-service]")
{
    {
        // A family that never samples a pattern does not pay for the setup() of the patterns.
        const ScopedPatternFamily activation(ProbeMode::None);
        Print print;
        Model model;
        run_support_step(print, model, {Test::TestMesh::cube_20x20x20});
        CHECK(pattern_setup_count == 0);
    }
    // Two threads make their first call together (the family holds them at a barrier, and the test fails
    // if the barrier times out): one builds the service, once.
    const ScopedPatternFamily activation(ProbeMode::Threads);
    Print print;
    Model model;
    run_support_step(print, model, {Test::TestMesh::cube_20x20x20});
    CHECK_FALSE(rendezvous_timed_out);
    CHECK(pattern_setup_count == 1);
}

TEST_CASE("STEP_SUPPORT patterns called by two threads of one family match a serial run", "[plugins][support-pattern-service]")
{
    std::vector<FillResult> serial;
    {
        const ScopedPatternFamily activation(ProbeMode::Sample);
        Print print;
        Model model;
        run_support_step(print, model, {Test::TestMesh::cube_20x20x20});
        REQUIRE(records.size() == 1);
        serial = records.front().results;
    }

    const ScopedPatternFamily activation(ProbeMode::Threads);
    pattern_rendezvous = true;
    Print print;
    Model model;
    run_support_step(print, model, {Test::TestMesh::cube_20x20x20});
    // The probe pattern saw two runs at the same time: the callers did overlap.
    INFO("the probe pattern waited " << wait_limit.count() << " s without seeing a second concurrent run");
    CHECK_FALSE(rendezvous_timed_out);
    CHECK(pattern_max_active_runs >= 2);
    REQUIRE(records.size() == 2);
    for (const ObjectRecord &record : records)
        CHECK(record.results == serial);
}

TEST_CASE("STEP_SUPPORT patterns called by two objects at once match a serial run", "[plugins][support-pattern-service]")
{
    std::vector<ObjectRecord> serial;
    {
        const ScopedPatternFamily activation(ProbeMode::Sample);
        Print print;
        Model model;
        Test::init_print({Test::TestMesh::cube_20x20x20, Test::TestMesh::cube_20x20x20}, print, model);
        REQUIRE(print.objects().size() == 2);
        // One worker only: the objects run one after the other.
        tbb::global_control single_thread(tbb::global_control::max_allowed_parallelism, 1);
        Steps::StepGenerateSupport::run_step(Orchestrator::instance(), print);
        serial = records;
    }
    REQUIRE(serial.size() == 2);
    CHECK(serial.front().results == serial.back().results);

    const ScopedPatternFamily activation(ProbeMode::Sample);
    // Two objects can only overlap when TBB has two workers; with one, the comparison below is still done
    // but the overlap requirement is skipped.
    const bool can_overlap = tbb::this_task_arena::max_concurrency() >= 2;
    if (!can_overlap)
        WARN("overlap check skipped: tbb::this_task_arena::max_concurrency() is 1");
    pattern_rendezvous = can_overlap;
    Print print;
    Model model;
    run_support_step(print, model, {Test::TestMesh::cube_20x20x20, Test::TestMesh::cube_20x20x20});
    if (can_overlap) {
        // The two objects were inside the probe pattern at the same time.
        INFO("the probe pattern waited " << wait_limit.count() << " s without seeing a second concurrent run");
        CHECK_FALSE(rendezvous_timed_out);
        CHECK(pattern_max_active_runs >= 2);
    }
    REQUIRE(records.size() == 2);
    for (const ObjectRecord &record : records)
        CHECK(record.results == serial.front().results);
}

TEST_CASE("STEP_SUPPORT pattern callbacks contain a failing setup()", "[plugins][support-pattern-service]")
{
    const ScopedPatternFamily activation(ProbeMode::Fail);
    pattern_fail_setup = true;
    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model);

    // The construction of the service throws inside the first callback; no exception reaches the family.
    check_contained_failure(print, "probe pattern setup failure");
}

TEST_CASE("STEP_SUPPORT pattern callbacks contain a failing run and empty the partial output", "[plugins][support-pattern-service]")
{
    // 1: the pattern wrote a group in a child of the output; 2: it wrote a path directly on the output (a leaf).
    const int how = GENERATE(1, 2);
    CAPTURE(how);
    const ScopedPatternFamily activation(ProbeMode::Fail);
    pattern_fail_run = how;
    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model);

    // The callback returned 0 and left an empty collection (reading it asserts on a leaf); see check_contained_failure.
    check_contained_failure(print, "probe pattern run failure");
    CHECK(pattern_wrote_before_failure);
}
TEST_CASE("STEP_SUPPORT payload keeps its 1.1 members first and compiles as C", "[plugins][support-pattern-service]")
{
    // A family built against 1.1 reads print and object at these offsets; the pattern members come after them.
    CHECK(SLIC3R_PLUGIN_API_STEP_SUPPORT_MINOR == 2u);
    CHECK(offsetof(run_ctx_generate_support, print) == 0);
    CHECK(offsetof(run_ctx_generate_support, object) == sizeof(void *));
    CHECK(offsetof(run_ctx_generate_support, host_context) == 2 * sizeof(void *));
    CHECK(offsetof(run_ctx_generate_support, resolve_pattern_id) > offsetof(run_ctx_generate_support, object));
    CHECK(offsetof(run_ctx_generate_support, generate_pattern) > offsetof(run_ctx_generate_support, resolve_pattern_id));

    // The C fixture assigns C callbacks to the payload (the compiler rejects mismatching types) and calls those of this payload.
    run_ctx_generate_support payload = {};
    payload.resolve_pattern_id = [](const run_ctx_generate_support *, const char *) -> infill_pattern_runtime_id { return 7; };
    payload.generate_pattern = [](const run_ctx_generate_support *, infill_pattern_runtime_id id, const layer_handle *,
                                  const layer_island_handle *, const layer_region_island_handle *, const layer_region_handle *,
                                  const surface_handle *, const expolygon_collection_handle *, const raw_infill_pattern_params *,
                                  extrusion_entity_handle *) -> int32_t { return int32_t(id) + 1; };
    CHECK(test_support_pattern_callbacks_c(&payload) == 8 * 1000 + 7);
}
