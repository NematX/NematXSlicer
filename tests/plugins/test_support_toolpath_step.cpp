#include <catch2/catch.hpp>

#include <algorithm>
#include <atomic>
#include <mutex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "plugin_test_helpers.hpp"
#include "probe_pattern_plugin.hpp"
#include "test_data.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_support_toolpath.h"
#include "libslic3r/Api/plugin/cpp/AuxiliaryLayerHelpers.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"
#include "libslic3r/Steps/StepRunner.hpp"
#include "libslic3r/Steps/StepSupportToolpath.hpp"
#include "libslic3r/TriangleMesh.hpp"

/* Support toolpath tests register process-lifetime recording providers before
   configuration finalization. Scoped activation keeps them out of other tests.
   Calls are recorded under a mutex because objects may run in parallel; each
   role still completes before the next selected role starts.
   The pattern probe fills support surfaces through the INFILL_PATTERN callbacks
   of the payload and compares the result with a direct pattern call. */

namespace {
using namespace Slic3r;

struct RecordedCall {
    std::string id;
    const object_handle *object;
    size_t matched_surfaces;
};

// What setup() of one provider received: the number of runs it prepares and the object count of the print.
struct SetupRecord {
    std::string id;
    uint32_t run_count;
    size_t object_count;
};

// The object index and object that one setup_run() or run() call of a provider was given.
struct ObjectRecord {
    std::string id;
    size_t object_idx;
    const object_handle *object;
};

std::mutex call_mutex;
std::vector<RecordedCall> calls;
std::vector<SetupRecord> setup_records;
std::vector<ObjectRecord> setup_run_records;
std::vector<ObjectRecord> run_records;

// The records of one provider, in the order they were made.
template<class Record>
std::vector<Record> records_of(const std::vector<Record> &records, const char *id)
{
    std::vector<Record> out;
    for (const Record &record : records)
        if (record.id == id)
            out.push_back(record);
    return out;
}

class SupportToolpathProbe final : public slic3r_api::PluginBase
{
public:
    // Set the provider's id, role group and priority using names that outlive it.
    SupportToolpathProbe(orchestrator_handle *orchestrator, const char *id,
                        const char *group, int32_t priority)
        : PluginBase(orchestrator), m_id(id), m_group(group), m_priority(priority) {}

    // This provider's plugin id.
    const char *id() const { return m_id; }

private:
    // This provider's plugin id.
    const char *id_impl() const noexcept override { return m_id; }
    // Assign the provider to the per-object support toolpath step.
    slicing_step_t step_impl() const noexcept override { return STEP_SUPPORT_TOOLPATH; }
    // Return the role group used by the generated selector.
    const char *exclusive_group_impl() const noexcept override { return m_group; }
    // Declare no dependencies on other plugin ids.
    const char *const *dependencies_impl() const noexcept override { return nullptr; }
    // Return the order used between selected role algorithms.
    int32_t priority_impl() const noexcept override { return m_priority; }
    // Prefer the alternate top algorithm without changing execution order between roles.
    int32_t exclusive_group_priority_impl() const noexcept override
    {
        return std::string_view(m_id) == "test.support_toolpath.alternate" ? 0 : 100;
    }
    // Find sparse support surfaces and record this object's callback.
    void run_impl(const plugin_run_context *context) const override;
    // Record the run count and the print's object count that setup() is given.
    void setup_impl(const plugin_run_context *context, uint32_t run_count) const override;
    // Record the object index and object that this setup_run() call is given.
    void setup_run_impl(const plugin_run_context *context) const override;

    const char *m_id;
    const char *m_group;
    int32_t m_priority;
};

// Test providers shared by registration and scoped activation.
const std::vector<SupportToolpathProbe *> &probes();
// Run the dispatcher under its real freeze step; reset that step even if a plugin fails.
void run_toolpaths(Print &print);

// Add a two-point extrusion for each sparse support surface and record the call; missing payloads or failed publication throw.
void SupportToolpathProbe::run_impl(const plugin_run_context *context) const
{
    const run_ctx_support_toolpath *payload = plugin_ctx_as_support_toolpath(context);
    if (payload == nullptr || payload->print == nullptr || payload->object == nullptr)
        throw std::runtime_error("Missing support toolpath object payload");

    size_t matched = 0;
    if (std::string(m_group) == "support_toolpath_sparse") {
        for (uint32_t layer_idx = 0; layer_idx < object_count_auxiliary_layer(payload->object); ++layer_idx) {
            const layer_handle *layer = object_get_auxiliary_layer(payload->object, layer_idx);
            if (!plugin_property_has(layer_get_properties(layer), PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT))
                continue;
            for (uint32_t island_idx = 0; island_idx < layer_count_island(layer); ++island_idx) {
                const layer_island_handle *island = layer_get_island(layer, island_idx);
                layer_island_handle *mutable_island = layer_get_island_mutable(
                    object_get_auxiliary_layer(payload->object, layer_idx), island_idx);
                for (uint32_t region_idx = 0; region_idx < layer_island_count_region_island(island); ++region_idx) {
                    layer_region_island_handle *region = layer_island_get_region_island_mutable(mutable_island, region_idx);
                    for (uint32_t surface_idx = 0; surface_idx < layer_region_island_count_fill_surface(region); ++surface_idx) {
                        const raw_surface_type type = surface_get_type(layer_region_island_get_fill_surface(region, surface_idx));
                        if ((type & (RAW_SURFACE_TYPE_MOD_SUPPORT | RAW_SURFACE_TYPE_DENS_SPARSE)) !=
                            (RAW_SURFACE_TYPE_MOD_SUPPORT | RAW_SURFACE_TYPE_DENS_SPARSE))
                            continue;
                        extrusion_entity_handle *root = layer_region_island_get_mutable_extrusion(region, RAW_EXTRUSION_ROLE_SUPPORT);
                        slic3r_api::StoredExtrusionEntity leaf(context->plugin_storage);
                        const c_point points[] = {{scale_i(1.), scale_i(1.)}, {scale_i(2.), scale_i(1.)}};
                        // Build in scratch storage, then move the content into one
                         // source child. The scratch wrapper then frees the empty temporary.
                        if (!extrusion_polyline_set_points(leaf.mutable_handle(), points, 2) ||
                            extrusion_insert_child_move(root, extrusion_child_count(root), leaf.mutable_handle()) == EXTRUSION_INDEX_INVALID)
                            throw std::runtime_error("Cannot publish test support extrusion");
                        ++matched;
                    }
                }
            }
        }
    }
    const plugin_host_context *host_context = static_cast<const plugin_host_context *>(context->host_context);
    const std::lock_guard<std::mutex> lock(call_mutex);
    calls.push_back({m_id, payload->object, matched});
    run_records.push_back({m_id, host_context->object_idx, payload->object});
}

// Record the run count and the object count of the host context, which the runner fills before setup().
void SupportToolpathProbe::setup_impl(const plugin_run_context *context, uint32_t run_count) const
{
    const plugin_host_context *host_context = static_cast<const plugin_host_context *>(context->host_context);
    const std::lock_guard<std::mutex> lock(call_mutex);
    setup_records.push_back({m_id, run_count, host_context->object_count});
}

// Record the object index of the host context and the object of the payload, as this setup_run() call sees them.
void SupportToolpathProbe::setup_run_impl(const plugin_run_context *context) const
{
    const run_ctx_support_toolpath *payload = plugin_ctx_as_support_toolpath(context);
    if (payload == nullptr || payload->object == nullptr)
        throw std::runtime_error("Missing support toolpath object payload");
    const plugin_host_context *host_context = static_cast<const plugin_host_context *>(context->host_context);
    const std::lock_guard<std::mutex> lock(call_mutex);
    setup_run_records.push_back({m_id, host_context->object_idx, payload->object});
}

// Create the test providers once and share the same instances across tests.
const std::vector<SupportToolpathProbe *> &probes()
{
    orchestrator_handle *orchestrator = reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance());
    static SupportToolpathProbe sparse(orchestrator, "test.support_toolpath.sparse", "support_toolpath_sparse", 0);
    static SupportToolpathProbe bottom(orchestrator, "test.support_toolpath.bottom", "support_toolpath_bottom_interface", 100);
    static SupportToolpathProbe top(orchestrator, "test.support_toolpath.top", "support_toolpath_top_interface", 200);
    static SupportToolpathProbe alternate(orchestrator, "test.support_toolpath.alternate", "support_toolpath_top_interface", 200);
    static const std::vector<SupportToolpathProbe *> providers {&sparse, &bottom, &top, &alternate};
    return providers;
}

class ScopedToolpathProbes
{
public:
    // Replace built-in algorithms with test providers and clear their call log; activation is restored by the destructor.
    ScopedToolpathProbes()
    {
        Test::Plugins::ensure_plugin_test_runtime_initialized();
        calls.clear();
        setup_records.clear();
        setup_run_records.clear();
        run_records.clear();
        for (const char *id : {"support.toolpath.legacy_sparse", "support.toolpath.branch_sheath", "support.toolpath.legacy_bottom", "support.toolpath.legacy_top"})
            REQUIRE(Orchestrator::instance().set_plugin_active(id, false));
        for (SupportToolpathProbe *probe : probes())
            REQUIRE(Orchestrator::instance().set_plugin_active(probe->id(), true));
    }
    // Disable test providers and restore production algorithms after the scoped test.
    ~ScopedToolpathProbes()
    {
        for (SupportToolpathProbe *probe : probes())
            Orchestrator::instance().set_plugin_active(probe->id(), false);
        for (const char *id : {"support.toolpath.legacy_sparse", "support.toolpath.branch_sheath", "support.toolpath.legacy_bottom", "support.toolpath.legacy_top"})
            Orchestrator::instance().set_plugin_active(id, true);
    }
};

// Set the running step for source-edit checks, run the algorithms, and reset the step even if a plugin throws.
void run_toolpaths(Print &print)
{
    struct RunningStep {
        Print &print;
        // Restore idle access after the dispatcher finishes or throws.
        ~RunningStep() { print.set_running_step(STEP_NONE); }
    };
    const RunningStep reset {print};
    print.set_running_step(STEP_SUPPORT_TOOLPATH);
    Steps::StepSupportToolpath::prepare(print);
    Steps::StepSupportToolpath::run_step(Orchestrator::instance(), print);
}

// Number of generated paths and their total length, used to compare two fills.
struct PatternStats {
    size_t paths = 0;
    double length = 0.;
};

// What the pattern probe saw for one object: the return code and output of
// each pattern call made through the payload callbacks.
struct PatternRecord {
    const object_handle *object = nullptr;
    int32_t rectilinear_result = -1;
    int32_t supportbase_result = -1;
    int32_t unknown_result = -1;
    int32_t lightning_result = -1;
    PatternStats rectilinear;
    PatternStats supportbase;
    size_t refused_paths = 0;
};

// Handles of the single support surface added to an object by the test.
struct SupportSurface {
    const layer_handle *layer = nullptr;
    const layer_island_handle *island = nullptr;
    const layer_region_island_handle *region_island = nullptr;
    const surface_handle *surface = nullptr;
};

// What the pattern probe saw when its call of the failing probe pattern came back (failure scenario).
struct FailureRecord {
    // True when the algorithm ran to its end: the statements after the failed callback were executed.
    bool reached_end = false;
    // Runtime id of the probe pattern, and the result and paths of the call that failed.
    infill_pattern_runtime_id probe_id = INFILL_PATTERN_RUNTIME_ID_INVALID;
    int32_t result = -1;
    size_t paths = SIZE_MAX;
    // A second call once the step is stopping, and a resolve after it (the sentinel is not a possible answer).
    int32_t again_result = -1;
    size_t again_paths = SIZE_MAX;
    infill_pattern_runtime_id resolve_after = infill_pattern_runtime_id(0x7fffffff);
};

std::mutex pattern_mutex;
std::vector<PatternRecord> pattern_records;
// When true the pattern probe only runs the failure scenario and fills failure_records.
bool failure_scenario = false;
std::vector<FailureRecord> failure_records;
class SupportPatternProbe final : public slic3r_api::PluginBase
{
public:
    // Create the sparse-role provider that calls INFILL_PATTERN plugins.
    explicit SupportPatternProbe(orchestrator_handle *orchestrator) : PluginBase(orchestrator) {}

    static constexpr const char *k_id = "test.support_toolpath.patterns";

private:
    // This provider's plugin id.
    const char *id_impl() const noexcept override { return k_id; }
    // Assign the provider to the per-object support toolpath step.
    slicing_step_t step_impl() const noexcept override { return STEP_SUPPORT_TOOLPATH; }
    // Share the sparse role group; the test runs it as the only active provider.
    const char *exclusive_group_impl() const noexcept override { return "support_toolpath_sparse"; }
    // Declare no dependencies on other plugin ids.
    const char *const *dependencies_impl() const noexcept override { return nullptr; }
    // Run with the sparse role priority.
    int32_t priority_impl() const noexcept override { return 0; }
    // Fill the object's support surface with several patterns and record the results.
    void run_impl(const plugin_run_context *context) const override;
};

// The process-lifetime pattern probe shared by registration and activation.
SupportPatternProbe &pattern_probe();
// Find the first fill surface of the object's first auxiliary layer.
SupportSurface find_support_surface(const object_handle *object);
// Build the complete recipe a support algorithm must provide for one pattern.
raw_infill_pattern_params support_params(const object_handle *object, infill_pattern_runtime_id pattern_id);
// Call the failing probe pattern, then the callbacks again, and record what each call returned.
FailureRecord run_failure_scenario(const run_ctx_support_toolpath *payload, const SupportSurface &support);
// Count the generated paths and sum their lengths.
PatternStats pattern_stats(const ExtrusionEntityCollection &output);
// Fill the surface by calling one pattern plugin like STEP_INFILL does, without the support service.
PatternStats direct_pattern_stats(Print &print, const object_handle *object, const char *pattern_plugin_id);
// Publish one auxiliary support layer holding a sparse support surface on the object.
void add_sparse_support_surface(PrintObject &object, PluginStorage &storage, const ExPolygon &area);

void SupportPatternProbe::run_impl(const plugin_run_context *context) const
{
    const run_ctx_support_toolpath *payload = plugin_ctx_as_support_toolpath(context);
    if (payload == nullptr || payload->resolve_pattern_id == nullptr || payload->generate_pattern == nullptr)
        throw std::runtime_error("Missing support toolpath pattern callbacks");

    const SupportSurface support = find_support_surface(payload->object);
    if (support.surface == nullptr)
        throw std::runtime_error("Missing test support surface");

    // Each call gets its own empty host collection, like the infill generator does.
    const auto generate = [payload, &support](infill_pattern_runtime_id pattern_id, ExtrusionEntityCollection &output) {
        const raw_infill_pattern_params params = support_params(payload->object, pattern_id);
        return payload->generate_pattern(payload, pattern_id, support.layer, support.island, support.region_island,
                                         nullptr, support.surface, nullptr, &params,
                                         reinterpret_cast<extrusion_entity_handle *>(&output));
    };

    if (failure_scenario) {
        const FailureRecord failure = run_failure_scenario(payload, support);
        const std::lock_guard<std::mutex> lock(pattern_mutex);
        failure_records.push_back(failure);
        return;
    }

    PatternRecord record;
    record.object = payload->object;
    ExtrusionEntityCollection rectilinear;
    record.rectilinear_result = generate(payload->resolve_pattern_id(payload, "rectilinear"), rectilinear);
    record.rectilinear = pattern_stats(rectilinear);
    ExtrusionEntityCollection supportbase;
    record.supportbase_result = generate(payload->resolve_pattern_id(payload, "supportbase"), supportbase);
    record.supportbase = pattern_stats(supportbase);
    ExtrusionEntityCollection refused;
    record.unknown_result = generate(infill_pattern_runtime_id(100000), refused);
    record.lightning_result = generate(payload->resolve_pattern_id(payload, "lightning"), refused);
    record.refused_paths = pattern_stats(refused).paths;

    const std::lock_guard<std::mutex> lock(pattern_mutex);
    pattern_records.push_back(record);
}

// The probe statements after the failed callback are what only a contained failure lets run: an
// exception leaving the callback would end the algorithm at the call.
FailureRecord run_failure_scenario(const run_ctx_support_toolpath *payload, const SupportSurface &support)
{
    const auto generate = [payload, &support](infill_pattern_runtime_id pattern_id, ExtrusionEntityCollection &output) {
        const raw_infill_pattern_params params = support_params(payload->object, pattern_id);
        return payload->generate_pattern(payload, pattern_id, support.layer, support.island, support.region_island,
                                         nullptr, support.surface, nullptr, &params,
                                         reinterpret_cast<extrusion_entity_handle *>(&output));
    };
    FailureRecord record;
    record.probe_id = payload->resolve_pattern_id(payload, Test::ProbePatternPlugin::probe_pattern_id);
    ExtrusionEntityCollection output;
    record.result = generate(record.probe_id, output);
    record.paths = pattern_stats(output).paths;
    ExtrusionEntityCollection again;
    record.again_result = generate(record.probe_id, again);
    record.again_paths = pattern_stats(again).paths;
    record.resolve_after = payload->resolve_pattern_id(payload, "rectilinear");
    record.reached_end = true;
    return record;
}

SupportPatternProbe &pattern_probe()
{
    static SupportPatternProbe probe(reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
    return probe;
}

SupportSurface find_support_surface(const object_handle *object)
{
    SupportSurface found;
    if (object_count_auxiliary_layer(object) == 0)
        return found;
    found.layer = object_get_auxiliary_layer(object, 0);
    if (layer_count_island(found.layer) == 0)
        return found;
    found.island = layer_get_island(found.layer, 0);
    if (layer_island_count_region_island(found.island) == 0)
        return found;
    found.region_island = layer_island_get_region_island(found.island, 0);
    if (layer_region_island_count_fill_surface(found.region_island) == 0)
        return found;
    found.surface = layer_region_island_get_fill_surface(found.region_island, 0);
    return found;
}

raw_infill_pattern_params support_params(const object_handle *object, infill_pattern_runtime_id pattern_id)
{
    const Flow flow = Flow::new_from_width(0.44f, 0.4f, 0.2f, 1.f);
    const BoundingBox bounding_box = reinterpret_cast<const PrintObject *>(object)->bounding_box();
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
    params.object_bounding_box.min = {bounding_box.min.x(), bounding_box.min.y()};
    params.object_bounding_box.max = {bounding_box.max.x(), bounding_box.max.y()};
    return params;
}

PatternStats pattern_stats(const ExtrusionEntityCollection &output)
{
    PatternStats stats;
    for (const ArcPolyline &polyline : output.as_polylines()) {
        ++stats.paths;
        stats.length += polyline.length();
    }
    return stats;
}

PatternStats direct_pattern_stats(Print &print, const object_handle *object, const char *pattern_plugin_id)
{
    Orchestrator &host = Orchestrator::instance();
    Plugin *plugin = host.get_plugin(pattern_plugin_id);
    REQUIRE(plugin != nullptr);
    std::atomic_bool execution_error { false };
    plugin_host_context host_context =
        Steps::Detail::prepare_guarded_plugin_host_context(host, INFILL_PATTERN, *plugin, print, execution_error);
    plugin_run_context context = host.prepare_plugin_run_context(INFILL_PATTERN, plugin, &host_context);
    // Like STEP_INFILL, run setup() first so the pattern's progress state is bound to this context.
    run_ctx_infill_pattern_init init = {};
    init.print = reinterpret_cast<const print_handle *>(&print);
    context.data = &init;
    plugin->setup(context, 1);

    // The runtime id is not read by the pattern, which is selected here directly.
    const SupportSurface support = find_support_surface(object);
    const raw_infill_pattern_params params = support_params(object, INFILL_PATTERN_RUNTIME_ID_INVALID);
    ExtrusionEntityCollection output;
    run_ctx_infill_pattern payload = {};
    payload.print = reinterpret_cast<const print_handle *>(&print);
    payload.object = object;
    payload.layer = support.layer;
    payload.island = support.island;
    payload.region_island = support.region_island;
    payload.surface = support.surface;
    payload.params = &params;
    payload.output = reinterpret_cast<extrusion_entity_handle *>(&output);
    context.data = &payload;
    plugin->setup_run(context);
    plugin->run(context);
    CHECK_FALSE(execution_error.load());
    return pattern_stats(output);
}

void add_sparse_support_surface(PrintObject &object, PluginStorage &storage, const ExPolygon &area)
{
    const Print &print = *object.print();
    const ExPolygons subject {area};
    const slic3r_api::AuxiliaryLayerBuildResult built = slic3r_api::build_auxiliary_layer_regions_from_subject(
        reinterpret_cast<storage_handle *>(&storage), slic3r_api::Print(reinterpret_cast<const print_handle *>(&print)),
        slic3r_api::Object(reinterpret_cast<const object_handle *>(&object)),
        slic3r_api::ExPolygonCollection(reinterpret_cast<const expolygon_collection_handle *>(&subject)),
        scale_i(0.2), scale_i(0.2), scale_i(0.1));
    REQUIRE(built.created);
    layer_handle *layer = object_get_auxiliary_layer(reinterpret_cast<const object_handle *>(&object), 0);
    REQUIRE(plugin_property_get_or_add_data_mutable(
        reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()), layer_get_properties_mutable(layer),
        PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT) != nullptr);
    layer_region_island_handle *region = layer_island_get_or_create_region_island(
        layer_get_island_mutable(layer, 0), nullptr, 0, 0);
    REQUIRE(region != nullptr);
    slic3r_api::StoredSurfaceCollection surfaces(reinterpret_cast<storage_handle *>(&storage));
    surfaces.append(slic3r_api::ExPolygon(reinterpret_cast<const expolygon_handle *>(&area)),
                    slic3r_api::srf_type().internal().sparse().support().raw());
    REQUIRE(slic3r_api::LayerRegionIsland(region).set_fill_surfaces(&surfaces));
}

class ScopedPatternProbe
{
public:
    // Make the pattern probe the only support toolpath provider and activate the
    // support base pattern, which the test runtime leaves inactive; the
    // destructor restores both activations.
    ScopedPatternProbe()
    {
        Test::Plugins::ensure_plugin_test_runtime_initialized();
        pattern_records.clear();
        m_supportbase_was_active = Orchestrator::instance().is_plugin_active("supportbase");
        for (const char *id : {"support.toolpath.legacy_sparse", "support.toolpath.branch_sheath", "support.toolpath.legacy_bottom", "support.toolpath.legacy_top"})
            REQUIRE(Orchestrator::instance().set_plugin_active(id, false));
        REQUIRE(Orchestrator::instance().set_plugin_active(SupportPatternProbe::k_id, true));
        REQUIRE(Orchestrator::instance().set_plugin_active("supportbase", true));
    }
    // Disable the probe and restore the production algorithms and pattern activation.
    ~ScopedPatternProbe()
    {
        Orchestrator::instance().set_plugin_active("supportbase", m_supportbase_was_active);
        Orchestrator::instance().set_plugin_active(SupportPatternProbe::k_id, false);
        for (const char *id : {"support.toolpath.legacy_sparse", "support.toolpath.branch_sheath", "support.toolpath.legacy_bottom", "support.toolpath.legacy_top"})
            Orchestrator::instance().set_plugin_active(id, true);
    }

private:
    bool m_supportbase_was_active = false;
};

// Make the failing probe pattern active next to the pattern probe, with the given failure mode of
// its run() (see probe_pattern_plugin.hpp), and the pattern probe run the failure scenario. The destructor
// restores the activation and the probe state, clears the plugin cancellation a failure leaves, and drops
// the messages.
class ScopedFailingPattern
{
public:
    // mode: 1 and 2 make run() fail after a partial write (child group, leaf); 0 leaves it intact.
    // cancel_print: the print that run() cancels instead of generating, or null.
    ScopedFailingPattern(int mode, Print *cancel_print)
    {
        namespace Probe = Test::ProbePatternPlugin;
        m_probe_was_active = Orchestrator::instance().is_plugin_active(Probe::probe_pattern_id);
        REQUIRE(Orchestrator::instance().set_plugin_active(Probe::probe_pattern_id, true));
        Probe::pattern_fail_run = mode;
        Probe::pattern_wrote_before_failure = false;
        Probe::pattern_cancel_print = cancel_print;
        failure_records.clear();
        failure_scenario = true;
        Orchestrator::instance().reset_plugin_cancel();
        (void) Orchestrator::instance().consume_plugin_messages();
    }
    ~ScopedFailingPattern()
    {
        namespace Probe = Test::ProbePatternPlugin;
        failure_scenario = false;
        Probe::pattern_fail_run = 0;
        Probe::pattern_cancel_print = nullptr;
        Orchestrator::instance().set_plugin_active(Probe::probe_pattern_id, m_probe_was_active);
        Orchestrator::instance().reset_plugin_cancel();
        (void) Orchestrator::instance().consume_plugin_messages();
    }
    ScopedFailingPattern(const ScopedFailingPattern &) = delete;
    ScopedFailingPattern &operator=(const ScopedFailingPattern &) = delete;

private:
    bool m_probe_was_active = false;
};

// Count the plugin messages of the level whose text contains `text` (an empty text matches every message)
// and, when plugin_id is not empty, that were reported by that plugin (an empty plugin_id matches any plugin).
size_t count_messages(const std::vector<Orchestrator::PluginMessage> &messages, Orchestrator::PluginMessageLevel level,
                      const std::string &text, const std::string &plugin_id = {})
{
    return size_t(std::count_if(messages.begin(), messages.end(), [&](const Orchestrator::PluginMessage &message) {
        return message.level == level && message.message.find(text) != std::string::npos &&
               (plugin_id.empty() || message.plugin_id == plugin_id);
    }));
}

// Print with one object holding one sparse support surface, as the pattern tests need.
struct SupportedPrint {
    Print print;
    Model model;
    PluginStorage storage;

    SupportedPrint()
    {
        Test::init_print({Test::TestMesh::cube_20x20x20}, print, model);
        const ExPolygon area(Polygon({Point(0, 0), Point(scale_i(10.), 0),
                                      Point(scale_i(10.), scale_i(10.)), Point(0, scale_i(10.))}));
        add_sparse_support_surface(print.object(0), storage, area);
    }
};
} // namespace

namespace Slic3r::Test::Plugins {
// Register and activate the test providers before selector settings are finalized; rejected registration or activation throws.
void register_support_toolpath_test_providers(orchestrator_handle *orchestrator)
{
    Orchestrator &host = *reinterpret_cast<Orchestrator *>(orchestrator);
    for (SupportToolpathProbe *probe : probes()) {
        if (!host.register_plugin(probe->c_instance()) || !host.set_plugin_active(probe->id(), true))
            throw std::runtime_error("Cannot register support toolpath test provider");
    }
    if (!host.register_plugin(pattern_probe().c_instance()) || !host.set_plugin_active(SupportPatternProbe::k_id, true))
        throw std::runtime_error("Cannot register support toolpath pattern probe");
}
} // namespace Slic3r::Test::Plugins

TEST_CASE("Support toolpath roles run once per object in priority order", "[plugins][support-toolpath]")
{
    const ScopedToolpathProbes activation;
    const StepSelectorTraits *traits = Orchestrator::instance().step_selector(STEP_SUPPORT_TOOLPATH, "");
    REQUIRE(traits != nullptr);
    CHECK(traits->container_type == RAW_CONTAINER_TYPE_OBJECT);
    CHECK(traits->category == RAW_OPTION_CATEGORY_SUPPORT);
    CHECK(traits->option_preset_type == RAW_PRESET_TYPE_FFF_PRINT);
    REQUIRE(traits->invalidates_step.has_value());
    CHECK(*traits->invalidates_step == STEP_SUPPORT_TOOLPATH);
    CHECK(traits->ui_page == "Support material");
    CHECK(traits->ui_group == "Support toolpaths");
    CHECK_FALSE(traits->auto_value);
    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20, Test::TestMesh::cube_2x20x10}, print, model);
    REQUIRE(print.objects().size() == 2);
    REQUIRE(Steps::get_exclusive_steps().count(STEP_SUPPORT_TOOLPATH) == 0);
    REQUIRE(Orchestrator::instance().set_plugin_active(probes()[3]->id(), false));
    run_toolpaths(print);
    REQUIRE(calls.size() == 6);
    for (size_t role = 0; role < 3; ++role) {
        CHECK(calls[role * 2].id == probes()[role]->id());
        CHECK(calls[role * 2 + 1].id == probes()[role]->id());
        CHECK(calls[role * 2].object != calls[role * 2 + 1].object);
        for (const PrintObject &object : print.objects()) {
            const object_handle *handle = reinterpret_cast<const object_handle *>(&object);
            CHECK(std::count_if(calls.begin(), calls.end(), [role, handle](const RecordedCall &call) {
                return call.id == probes()[role]->id() && call.object == handle;
            }) == 1);
        }
    }
}

TEST_CASE("Support toolpath role selectors choose only one top algorithm", "[plugins][support-toolpath]")
{
    const ScopedToolpathProbes activation;
    const Plugin *top = Orchestrator::instance().get_plugin(probes()[2]->id());
    const Plugin *alternate = Orchestrator::instance().get_plugin(probes()[3]->id());
    REQUIRE(top != nullptr);
    REQUIRE(alternate != nullptr);
    CHECK(top->get_priority() == 200);
    CHECK(alternate->get_priority() == 200);
    CHECK(top->get_exclusive_group_priority() == 100);
    CHECK(alternate->get_exclusive_group_priority() == 0);
    const size_t selected = GENERATE(size_t(2), size_t(3));
    const std::string key = Steps::plugin_exclusive_group_option_key(
        Orchestrator::instance(), STEP_SUPPORT_TOOLPATH, "support_toolpath_top_interface");
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    REQUIRE(config.option(key) != nullptr);
    bool found_group = false;
    for (const Steps::StepExclusivePluginGroup &group : Steps::active_exclusive_plugin_groups(Orchestrator::instance())) {
        if (group.group.step == STEP_SUPPORT_TOOLPATH && group.group.group_id == "support_toolpath_top_interface") {
            found_group = true;
            CHECK(group.group.option_def.category == RAW_OPTION_CATEGORY_SUPPORT);
            CHECK(group.group.option_def.option_preset_type == RAW_PRESET_TYPE_FFF_PRINT);
        }
    }
    REQUIRE(found_group);
    config.set_deserialize_strict({{key, std::string(probes()[selected]->id())}});
    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
    run_toolpaths(print);
    REQUIRE(calls.size() == 3);
    CHECK(calls[0].id == probes()[0]->id());
    CHECK(calls[1].id == probes()[1]->id());
    CHECK(calls[2].id == probes()[selected]->id());
}

// Each object picks its own top-interface algorithm; a role runs only on the objects that picked its algorithm, in priority order.
TEST_CASE("Support toolpath role selectors are chosen per object", "[plugins][support-toolpath]")
{
    const ScopedToolpathProbes activation;
    const std::string key = Steps::plugin_exclusive_group_option_key(
        Orchestrator::instance(), STEP_SUPPORT_TOOLPATH, "support_toolpath_top_interface");
    bool found_group = false;
    for (const Steps::StepExclusivePluginGroup &group : Steps::active_exclusive_plugin_groups(Orchestrator::instance())) {
        if (group.group.step == STEP_SUPPORT_TOOLPATH && group.group.group_id == "support_toolpath_top_interface") {
            found_group = true;
            CHECK(group.group.option_def.container_type == RAW_CONTAINER_TYPE_OBJECT);
        }
    }
    REQUIRE(found_group);

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20, Test::TestMesh::cube_20x20x20}, print, model, config);
    REQUIRE(print.objects().size() == 2);
    model.objects()[0].config.set_deserialize_strict(key, std::string(probes()[2]->id()));
    model.objects()[1].config.set_deserialize_strict(key, std::string(probes()[3]->id()));
    print.apply(model, config);
    run_toolpaths(print);

    const object_handle *first = reinterpret_cast<const object_handle *>(&print.object(0));
    const object_handle *second = reinterpret_cast<const object_handle *>(&print.object(1));
    // Sparse (0) and bottom (100) run on both objects; alternate and top (200) each run on their selected object.
    REQUIRE(calls.size() == 6);
    CHECK(calls[0].id == probes()[0]->id());
    CHECK(calls[1].id == probes()[0]->id());
    CHECK(((calls[0].object == first && calls[1].object == second) ||
           (calls[0].object == second && calls[1].object == first)));
    CHECK(calls[2].id == probes()[1]->id());
    CHECK(calls[3].id == probes()[1]->id());
    CHECK(((calls[2].object == first && calls[3].object == second) ||
           (calls[2].object == second && calls[3].object == first)));
    CHECK(calls[4].id == probes()[2]->id());
    CHECK(calls[4].object == first);
    CHECK(calls[5].id == probes()[3]->id());
    CHECK(calls[5].object == second);

    // alternate runs on the second object only: setup() prepares one run and the host reports both objects, and both
    // per-run phases of alternate see index 1 and the second object.
    const char *alternate = probes()[3]->id();
    const std::vector<SetupRecord> alternate_setups = records_of(setup_records, alternate);
    const std::vector<ObjectRecord> alternate_setup_runs = records_of(setup_run_records, alternate);
    const std::vector<ObjectRecord> alternate_runs = records_of(run_records, alternate);
    REQUIRE(alternate_setups.size() == 1);
    REQUIRE(alternate_setup_runs.size() == 1);
    REQUIRE(alternate_runs.size() == 1);
    CHECK(alternate_setups.front().run_count == 1);
    CHECK(alternate_setups.front().object_count == 2);
    CHECK(alternate_setup_runs.front().object_idx == 1);
    CHECK(alternate_setup_runs.front().object == second);
    CHECK(alternate_runs.front().object_idx == 1);
    CHECK(alternate_runs.front().object == second);
}

// The progress plan lists every provider that some object selects, so a full pipeline with per-object selectors completes it.
TEST_CASE("Per-object support toolpath selectors complete the pipeline progress plan", "[plugins][support-toolpath]")
{
    const ScopedToolpathProbes activation;
    const std::string key = Steps::plugin_exclusive_group_option_key(
        Orchestrator::instance(), STEP_SUPPORT_TOOLPATH, "support_toolpath_top_interface");
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20, Test::TestMesh::cube_20x20x20}, print, model, config);
    model.objects()[0].config.set_deserialize_strict(key, std::string(probes()[2]->id()));
    model.objects()[1].config.set_deserialize_strict(key, std::string(probes()[3]->id()));
    print.apply(model, config);
    REQUIRE_NOTHROW(print.process());

    const object_handle *first = reinterpret_cast<const object_handle *>(&print.object(0));
    const object_handle *second = reinterpret_cast<const object_handle *>(&print.object(1));
    const auto count_calls = [](const char *id, const object_handle *object) {
        return std::count_if(calls.begin(), calls.end(), [id, object](const RecordedCall &call) {
            return call.id == id && call.object == object;
        });
    };
    CHECK(count_calls(probes()[0]->id(), first) == 1);
    CHECK(count_calls(probes()[0]->id(), second) == 1);
    CHECK(count_calls(probes()[1]->id(), first) == 1);
    CHECK(count_calls(probes()[1]->id(), second) == 1);
    CHECK(count_calls(probes()[2]->id(), first) == 1);
    CHECK(count_calls(probes()[2]->id(), second) == 0);
    CHECK(count_calls(probes()[3]->id(), first) == 0);
    CHECK(count_calls(probes()[3]->id(), second) == 1);
}

// Publish typed auxiliary surfaces through the API before checking the sparse algorithm's extrusion.
TEST_CASE("Sparse support toolpath reads its surface and publishes a source extrusion", "[plugins][support-toolpath]")
{
    const ScopedToolpathProbes activation;
    REQUIRE(Orchestrator::instance().set_plugin_active(probes()[3]->id(), false));
    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model);
    PrintObject &object = print.object(0);
    const ExPolygon area(Polygon({Point(0, 0), Point(scale_i(4.), 0),
                                  Point(scale_i(4.), scale_i(4.)), Point(0, scale_i(4.))}));
    const ExPolygons subject {area};
    PluginStorage storage;
    const slic3r_api::AuxiliaryLayerBuildResult built = slic3r_api::build_auxiliary_layer_regions_from_subject(
        reinterpret_cast<storage_handle *>(&storage), slic3r_api::Print(reinterpret_cast<const print_handle *>(&print)),
        slic3r_api::Object(reinterpret_cast<const object_handle *>(&object)),
        slic3r_api::ExPolygonCollection(reinterpret_cast<const expolygon_collection_handle *>(&subject)),
        scale_i(0.2), scale_i(0.2), scale_i(0.1));
    REQUIRE(built.created);
    layer_handle *layer = object_get_auxiliary_layer(reinterpret_cast<const object_handle *>(&object), 0);
    REQUIRE(plugin_property_get_or_add_data_mutable(
        reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()), layer_get_properties_mutable(layer),
        PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT) != nullptr);
    REQUIRE(layer_count_island(layer) == 1);
    layer_region_island_handle *region = layer_island_get_or_create_region_island(
        layer_get_island_mutable(layer, 0), nullptr, 0, 0);
    REQUIRE(region != nullptr);
    slic3r_api::StoredSurfaceCollection surfaces(reinterpret_cast<storage_handle *>(&storage));
    const slic3r_api::ExPolygon surface_area(reinterpret_cast<const expolygon_handle *>(&area));
    surfaces.append(surface_area, slic3r_api::srf_type().internal().sparse().support().raw());
    surfaces.append(surface_area, slic3r_api::srf_type().top().solid().support().raw());
    REQUIRE(slic3r_api::LayerRegionIsland(region).set_fill_surfaces(&surfaces));
    CHECK(surfaces.empty());
    run_toolpaths(print);
    REQUIRE(calls.size() == 3);
    CHECK(calls.front().matched_surfaces == 1);
    const extrusion_entity_handle *root = layer_region_island_get_extrusion(region, RAW_EXTRUSION_ROLE_SUPPORT);
    REQUIRE(root != nullptr);
    REQUIRE(extrusion_child_count(root) == 1);
    CHECK(extrusion_polyline_point_count(extrusion_child(root, 0)) == 2);
    CHECK(layer_region_island_count_fill_surface(region) == 2);
    Steps::StepSupportToolpath::clear(print);
    CHECK(object.auxiliary_layers().size() == 1);
    CHECK(extrusion_child_count(root) == 1);
}

// Four objects run in parallel and each one calls patterns through the shared service.
TEST_CASE("Support toolpath algorithms fill surfaces through infill pattern plugins", "[plugins][support-toolpath]")
{
    const ScopedPatternProbe activation;
    Orchestrator &host = Orchestrator::instance();
    for (const char *id : {"rectilinear", "supportbase", "lightning"}) {
        INFO(id);
        REQUIRE(host.is_plugin_active(id));
    }
    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20, Test::TestMesh::cube_20x20x20,
                      Test::TestMesh::cube_20x20x20, Test::TestMesh::cube_20x20x20}, print, model);
    REQUIRE(print.objects().size() == 4);
    PluginStorage storage;
    const ExPolygon area(Polygon({Point(0, 0), Point(scale_i(10.), 0),
                                  Point(scale_i(10.), scale_i(10.)), Point(0, scale_i(10.))}));
    for (size_t object_idx = 0; object_idx < print.objects().size(); ++object_idx)
        add_sparse_support_surface(print.object(object_idx), storage, area);

    (void) host.consume_plugin_messages();
    run_toolpaths(print);
    const std::vector<Orchestrator::PluginMessage> messages = host.consume_plugin_messages();

    REQUIRE(pattern_records.size() == 4);
    for (const PatternRecord &record : pattern_records) {
        CHECK(record.rectilinear_result == 1);
        CHECK(record.supportbase_result == 1);
        CHECK(record.rectilinear.paths > 0);
        CHECK(record.supportbase.paths > 0);
        // The service call matches a direct STEP_INFILL-style call with the same recipe.
        const PatternStats direct_rectilinear = direct_pattern_stats(print, record.object, "rectilinear");
        const PatternStats direct_supportbase = direct_pattern_stats(print, record.object, "supportbase");
        CHECK(record.rectilinear.paths == direct_rectilinear.paths);
        CHECK(record.rectilinear.length == Approx(direct_rectilinear.length));
        CHECK(record.supportbase.paths == direct_supportbase.paths);
        CHECK(record.supportbase.length == Approx(direct_supportbase.length));
        // Unknown ids and patterns needing the STEP_INFILL initialization generate nothing.
        CHECK(record.unknown_result == 0);
        CHECK(record.lightning_result == 0);
        CHECK(record.refused_paths == 0);
    }
    for (const PrintObject &object : print.objects()) {
        const object_handle *handle = reinterpret_cast<const object_handle *>(&object);
        CHECK(std::count_if(pattern_records.begin(), pattern_records.end(), [handle](const PatternRecord &record) {
            return record.object == handle;
        }) == 1);
    }

    const size_t refusals = std::count_if(messages.begin(), messages.end(), [](const Orchestrator::PluginMessage &message) {
        return message.level == Orchestrator::PluginMessageLevel::Error && message.plugin_id == "lightning" &&
               message.message.find("initialization") != std::string::npos;
    });
    const size_t unknown = std::count_if(messages.begin(), messages.end(), [](const Orchestrator::PluginMessage &message) {
        return message.message.find("Unknown runtime infill pattern id '100000'") != std::string::npos;
    });
    CHECK(refusals == 4);
    CHECK(unknown == 4);
}

// A pattern that fails in run() is a contained failure: the callbacks return the failure value and
// empty the output, the algorithm goes on, and the step stops once it returns, as a plugin stopped by the
// pattern plugin's error. The statements the probe runs after the failed callback prove the containment
// (an exception leaving the callback would end the algorithm there), and the stop text naming the pattern
// plugin (not "stopped after plugin ... reported an error") proves that the algorithm itself never raised.
TEST_CASE("Support toolpath pattern callbacks contain a failing pattern", "[plugins][support-toolpath][support-pattern-service]")
{
    // 1: the pattern wrote a group in a child of the output; 2: it wrote a path directly on it (a leaf).
    const int how = GENERATE(1, 2);
    CAPTURE(how);
    const ScopedPatternProbe activation;
    const ScopedFailingPattern failing(how, nullptr);
    SupportedPrint supported;
    Orchestrator &host = Orchestrator::instance();
    (void) host.consume_plugin_messages();

    std::string failure;
    try {
        run_toolpaths(supported.print);
    } catch (const std::exception &error) {
        failure = error.what();
    }
    const std::vector<Orchestrator::PluginMessage> messages = host.consume_plugin_messages();

    REQUIRE(failure_records.size() == 1);
    const FailureRecord &record = failure_records.front();
    CHECK(Test::ProbePatternPlugin::pattern_wrote_before_failure);
    // The algorithm ran to its end: the failed call returned the failure value and left an empty output.
    CHECK(record.reached_end);
    CHECK(record.probe_id != INFILL_PATTERN_RUNTIME_ID_INVALID);
    CHECK(record.result == 0);
    CHECK(record.paths == 0);
    // The step is stopping: the next calls fail at once.
    CHECK(record.again_result == 0);
    CHECK(record.again_paths == 0);
    CHECK(record.resolve_after == INFILL_PATTERN_RUNTIME_ID_INVALID);
    // The runner then stopped the step as a healthy plugin, and names the pattern plugin that failed.
    namespace Probe = Test::ProbePatternPlugin;
    CAPTURE(failure);
    CHECK(failure.find("STEP_SUPPORT_TOOLPATH stopped during ") != std::string::npos);
    CHECK(failure.find("of plugin '" + std::string(SupportPatternProbe::k_id) + "' after plugin '" +
                       std::string(Probe::probe_pattern_id) + "' reported an error.") != std::string::npos);
    // Earlier diagnostics were drained: the only error is the original message of the pattern plugin; the
    // callbacks and the runner add no message of their own, whatever their wording.
    CHECK(count_messages(messages, Orchestrator::PluginMessageLevel::Error, "probe pattern run failure", Probe::probe_pattern_id) == 1);
    CHECK(count_messages(messages, Orchestrator::PluginMessageLevel::Error, "") == 1);
}

// A cancellation of the print inside a pattern is not an error: the callbacks return the failure value
// like for any failure, but record nothing and request no plugin cancellation; the runner rethrows the
// cancellation when the algorithm returns.
TEST_CASE("Support toolpath pattern callbacks leave a print cancellation to the runner", "[plugins][support-toolpath][support-pattern-service]")
{
    const ScopedPatternProbe activation;
    SupportedPrint supported;
    const ScopedFailingPattern failing(0, &supported.print);
    Orchestrator &host = Orchestrator::instance();
    (void) host.consume_plugin_messages();

    CHECK_THROWS_AS(run_toolpaths(supported.print), CanceledException);
    const std::vector<Orchestrator::PluginMessage> messages = host.consume_plugin_messages();

    REQUIRE(failure_records.size() == 1);
    const FailureRecord &record = failure_records.front();
    CHECK(record.reached_end);
    CHECK(record.result == 0);
    CHECK(record.paths == 0);
    CHECK(record.again_result == 0);
    CHECK(record.resolve_after == INFILL_PATTERN_RUNTIME_ID_INVALID);
    CHECK_FALSE(host.is_plugin_cancelled());
    CHECK(count_messages(messages, Orchestrator::PluginMessageLevel::Error, "") == 0);
}

TEST_CASE("Support toolpath dispatch without active providers does nothing", "[plugins][support-toolpath]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    calls.clear();
    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model);
    run_toolpaths(print);
    CHECK(calls.empty());
    CHECK(print.object(0).auxiliary_layers().empty());
}

TEST_CASE("Support toolpath surfaces are published through the island mutator", "[plugins][support-toolpath][surface]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    Test::init_print({make_cube(4., 4., 0.4)}, print, model);
    print.process();
    PrintObject &object = print.object(0);
    REQUIRE_FALSE(object.layers().empty());
    REQUIRE_FALSE(object.layer(0).islands().empty());
    REQUIRE_FALSE(object.layer(0).island(0).regions_islands().empty());
    LayerRegionIsland &object_region = object.layer(0).island(0).regions_island(0);
    const size_t original_count = object_region.fill_surfaces().size();
    REQUIRE(original_count > 0);
    const SurfaceType original_type = object_region.fill_surfaces().at(0).surface_type;

    const ExPolygon area(Polygon({Point(0, 0), Point(scale_i(4.), 0),
                                  Point(scale_i(4.), scale_i(4.)), Point(0, scale_i(4.))}));
    const ExPolygons subject {area};
    PluginStorage storage;
    const slic3r_api::AuxiliaryLayerBuildResult built = slic3r_api::build_auxiliary_layer_regions_from_subject(
        reinterpret_cast<storage_handle *>(&storage), slic3r_api::Print(reinterpret_cast<const print_handle *>(&print)),
        slic3r_api::Object(reinterpret_cast<const object_handle *>(&object)),
        slic3r_api::ExPolygonCollection(reinterpret_cast<const expolygon_collection_handle *>(&subject)),
        scale_i(0.2), scale_i(0.2), scale_i(0.1));
    REQUIRE(built.created);
    layer_handle *layer = object_get_auxiliary_layer(reinterpret_cast<const object_handle *>(&object), 0);
    layer_region_island_handle *region = layer_island_get_or_create_region_island(
        layer_get_island_mutable(layer, 0), nullptr, 0, 0);
    REQUIRE(region != nullptr);
    slic3r_api::StoredSurfaceCollection surfaces(reinterpret_cast<storage_handle *>(&storage));
    const slic3r_api::ExPolygon surface_area(reinterpret_cast<const expolygon_handle *>(&area));
    const raw_surface_type type = slic3r_api::srf_type().internal().sparse().support().raw();

    struct RunningStep {
        Print &print;
        // Restore idle access even if a test assertion stops the loop.
        ~RunningStep() { print.set_running_step(STEP_NONE); }
    } reset {print};
    for (slicing_step_t step : {STEP_SUPPORT, STEP_SUPPORT_TOOLPATH, STEP_FINALIZE_SLICING, STEP_NONE}) {
        print.set_running_step(step);
        surfaces.append(surface_area, type);
        REQUIRE(layer_region_island_set_fill_surfaces(region, surfaces.mutable_handle()) == 1);
        CHECK(surfaces.empty());
        CHECK(layer_region_island_count_fill_surface(region) == 1);
        CHECK(surface_get_type(layer_region_island_get_fill_surface(region, 0)) == type);
    }
    REQUIRE(layer_region_island_set_fill_surfaces(region, nullptr) == 1);
    CHECK(layer_region_island_count_fill_surface(region) == 0);
    surfaces.append(surface_area, type);
    CHECK(layer_region_island_set_fill_surfaces(reinterpret_cast<layer_region_island_handle *>(&object_region),
                                              surfaces.mutable_handle()) == 0);
    CHECK(object_region.fill_surfaces().size() == original_count);
    CHECK(object_region.fill_surfaces().at(0).surface_type == original_type);
    CHECK(surfaces.size() == 1);
    CHECK(layer_region_island_set_fill_surfaces(nullptr, surfaces.mutable_handle()) == 0);
    CHECK(surfaces.size() == 1);
}
