/*
Check which pipeline results survive an invalidation and which must be rebuilt.
A checkpoint is a saved processing boundary. Fixtures run to selected boundaries,
change inputs or stop a plugin, then observe replay and cleanup through controlled
providers. Assertions distinguish completion from partial or cancelled work so
old owners cannot be reused while callbacks still depend on their data.

Checkpoint tests exercise rollback and cleanup barriers before source owners are reused.

Representative direct calls (branches are not a timeline):

    TEST_CASE
        |-- Test::init_print()
        |-- Steps::StepPipeline::run_slice()
        |-- Steps::StepPipeline::run_gcode()
        |-- apply_option()
        |-- prepare_lifecycle_probe()
        `-- print.apply()
*/
#include <catch2/catch.hpp>

#include <algorithm>
#include <future>
#include <thread>
#include <tbb/global_control.h>

#include "gcode_test_helpers.hpp"
#include "perimeter_test_helpers.hpp"
#include "plugin_test_helpers.hpp"
#include "test_data.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/internal/LayerIslandAccess.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PluginPropertyKey.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Exception.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"
#include "libslic3r/Slicing.hpp"
#include "libslic3r/Steps/StepGenerateGcode.hpp"
#include "libslic3r/Steps/StepExtrusionOrdering.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"
#include "libslic3r/Steps/StepLayerHeightGeneration.hpp"
#include "libslic3r/Steps/StepSlicing.hpp"

namespace {

class LifecyclePrint final : public Slic3r::Print
{
public:
    using Slic3r::Print::state_mutex;
};

class ObservedAuxiliaryLayer final : public Slic3r::Layer
{
public:
    ObservedAuxiliaryLayer(Slic3r::PrintObject &owner, const Slic3r::PrintObject &source, bool &source_alive)
        : Layer(0, &owner, Slic3r::scale_i(0.2), Slic3r::scale_i(0.2), 0.1, true),
          m_source(source), m_source_alive(source_alive) {}
    ~ObservedAuxiliaryLayer() override { m_source_alive = m_source.layer_count() != 0; }

private:
    const Slic3r::PrintObject &m_source;
    bool &m_source_alive;
};

class LifecycleClearPlugin final : public slic3r_api::PluginBase
{
public:
    explicit LifecycleClearPlugin(orchestrator_handle *orchestrator) : PluginBase(orchestrator) {}

    mutable size_t clears = 0;
    mutable bool owners_alive = false;
    mutable bool mutex_unlocked = false;
    mutable bool fail_clear = false;
    // Number of successful clears before fail_clear takes effect.
    mutable size_t fail_after = 0;
    std::mutex *mutex = nullptr;

private:
    const char *id_impl() const noexcept override { return "test.lifecycle.clear"; }
    const char *name_impl() const noexcept override { return "Lifecycle clear"; }
    slicing_step_t step_impl() const noexcept override { return STEP_EXTRUSION_EDIT; }
    const char *const *dependencies_impl() const noexcept override { return nullptr; }
    int32_t priority_impl() const noexcept override { return 0; }
    bool clear_enabled_impl() const noexcept override { return true; }
    void run_impl(const plugin_run_context *) const override {}

    void setup_impl(const plugin_run_context *context, uint32_t) const override
    {
        const Slic3r::Print &print = *reinterpret_cast<const Slic3r::Print *>(context->print);
        expected_plan = print.printing_plan();
        expected_objects = print.objects().size();
        clears = 0;
        owners_alive = false;
        mutex_unlocked = false;
        fail_clear = false;
        fail_after = 0;
    }

    int32_t clear_impl(const print_handle *handle, storage_handle *) const override
    {
        ++clears;
        const Slic3r::Print &print = *reinterpret_cast<const Slic3r::Print *>(handle);
        owners_alive = print.printing_plan() == expected_plan && print.objects().size() == expected_objects;
        std::thread lock_probe([this]() {
            mutex_unlocked = mutex->try_lock();
            if (mutex_unlocked)
                mutex->unlock();
        });
        lock_probe.join();
        if (fail_clear && clears > fail_after)
            throw std::runtime_error("Intentional lifecycle cleanup failure");
        return PLUGIN_ERROR_OK;
    }

    mutable const Slic3r::Printing::PrintingPlan *expected_plan = nullptr;
    mutable size_t expected_objects = 0;
};

LifecycleClearPlugin &prepare_lifecycle_probe(LifecyclePrint &print, const slicing_step_t step = STEP_EXTRUSION_EDIT)
{
    Slic3r::Orchestrator &orchestrator = Slic3r::Orchestrator::instance();
    static LifecycleClearPlugin probe(reinterpret_cast<orchestrator_handle *>(&orchestrator));
    probe.mutex = &print.state_mutex();
    if (orchestrator.get_plugin("test.lifecycle.clear") == nullptr) {
        REQUIRE(orchestrator.register_plugin(probe.c_instance()));
        REQUIRE(orchestrator.set_plugin_active("test.lifecycle.clear", false));
    }
    Slic3r::Plugin *plugin = orchestrator.get_plugin("test.lifecycle.clear");
    REQUIRE(plugin != nullptr);
    plugin_host_context host = orchestrator.prepare_plugin_host_context(step, plugin, &print);
    const plugin_run_context context = orchestrator.prepare_plugin_run_context(step, plugin, &host);
    plugin->setup(context, 0);
    return probe;
}

bool contains_step(const std::vector<slicing_step_t> &steps, slicing_step_t step)
{
    return std::find(steps.begin(), steps.end(), step) != steps.end();
}

struct MetadataPayload { int value; };

slic3r_property_type metadata_property_type()
{
    static const slic3r_api::PluginPropertyKey<MetadataPayload> key =
        slic3r_api::PluginPropertyKey<MetadataPayload>::register_dynamic(
            reinterpret_cast<orchestrator_handle *>(&Slic3r::Orchestrator::instance()), "tests.checkpoint.metadata");
    return key.type();
}

void set_metadata_property(Slic3r::PrintObject &object, const int value)
{
    MetadataPayload *payload = static_cast<MetadataPayload *>(object.get_or_add_property_data_mutable(
        metadata_property_type(), sizeof(MetadataPayload), alignof(MetadataPayload)));
    REQUIRE(payload != nullptr);
    payload->value = value;
}

int metadata_property(const Slic3r::PrintObject &object)
{
    const MetadataPayload *payload = static_cast<const MetadataPayload *>(object.property_data(metadata_property_type()));
    return payload == nullptr ? -1 : payload->value;
}

class MetadataPlugin final : public slic3r_api::PluginBase
{
public:
    MetadataPlugin(orchestrator_handle *orchestrator, const char *id, slicing_step_t step, int value)
        : PluginBase(orchestrator), m_id(id), m_step(step), m_value(value)
    {
        phase_property_type = slic3r_api::PluginPropertyKey<MetadataPayload>::register_dynamic(orchestrator, id).type();
    }
    const char *id() const { return m_id; }
    slic3r_property_type phase_property_type;
    mutable int observed_record = -1;
    mutable int observed_property = -1;
    mutable bool observed_slices_ready = true;
    bool cancel_setup = false;
    bool fail_setup = false;

private:
    const char *id_impl() const noexcept override { return m_id; }
    const char *name_impl() const noexcept override { return m_id; }
    slicing_step_t step_impl() const noexcept override { return m_step; }
    int32_t priority_impl() const noexcept override { return 10000; }
    const char *const *dependencies_impl() const noexcept override { return nullptr; }
    void run_impl(const plugin_run_context *) const override {}
    void setup_impl(const plugin_run_context *context, uint32_t) const override
    {
        Slic3r::Print &print = *const_cast<Slic3r::Print *>(reinterpret_cast<const Slic3r::Print *>(context->print));
        observed_slices_ready = print.slices_ready();
        Slic3r::DynamicConfig &record = print.records().get_or_add("test.metadata");
        const Slic3r::ConfigOptionInt *previous = record.option<Slic3r::ConfigOptionInt>("value");
        observed_record = previous == nullptr ? -1 : previous->value;
        observed_property = metadata_property(print.object(0));
        // setup is sequential; record-channel writes must not occur in parallel object callbacks.
        record.set_key_value("value", new Slic3r::ConfigOptionInt(m_value));
        print.records().get_or_add(m_id).set_key_value("value", new Slic3r::ConfigOptionInt(m_value));
        for (Slic3r::PrintObject &object : print.objects()) {
            set_metadata_property(object, m_value);
            MetadataPayload *phase = static_cast<MetadataPayload *>(object.get_or_add_property_data_mutable(
                phase_property_type, sizeof(MetadataPayload), alignof(MetadataPayload)));
            phase->value = m_value;
        }
        if (cancel_setup)
            print.cancel_internal();
        if (fail_setup)
            throw std::runtime_error("Intentional metadata setup failure");
    }
    const char *m_id;
    slicing_step_t m_step;
    int m_value;
};

class ScopedMetadataPlugins
{
public:
    ScopedMetadataPlugins()
    {
        Slic3r::Orchestrator &orchestrator = Slic3r::Orchestrator::instance();
        for (MetadataPlugin *plugin : {&post(), &skirt()}) {
            if (orchestrator.get_plugin(plugin->id()) == nullptr) {
                REQUIRE(orchestrator.register_plugin(plugin->c_instance()));
                REQUIRE(orchestrator.set_plugin_active(plugin->id(), false));
            }
            REQUIRE(orchestrator.set_plugin_active(plugin->id(), true));
        }
    }
    ~ScopedMetadataPlugins()
    {
        for (MetadataPlugin *plugin : {&post(), &skirt()}) {
            Slic3r::Orchestrator::instance().set_plugin_active(plugin->id(), false);
            plugin->cancel_setup = false;
            plugin->fail_setup = false;
        }
        Slic3r::Orchestrator::instance().reset_plugin_cancel();
    }
    static MetadataPlugin &post()
    {
        static MetadataPlugin plugin(reinterpret_cast<orchestrator_handle *>(&Slic3r::Orchestrator::instance()),
                                     "test.metadata.post", STEP_POST_SLICING, 20);
        return plugin;
    }
    static MetadataPlugin &skirt()
    {
        static MetadataPlugin plugin(reinterpret_cast<orchestrator_handle *>(&Slic3r::Orchestrator::instance()),
                                     "test.metadata.skirt", STEP_SKIRT_BRIM, 30);
        return plugin;
    }
};

} // namespace

TEST_CASE("Pipeline state invalidation preserves checkpoint prefixes and is idempotent",
          "[plugins][step-pipeline][pipeline-state]")
{
    using namespace Slic3r;
    using Steps::Checkpoint;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool worker_owned = GENERATE(false, true);
    LifecyclePrint print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model,
                     Test::PerimeterPluginTests::perimeter_config({}));
    print.process();
    REQUIRE(print.slices_ready());
    for (const Checkpoint checkpoint : {Checkpoint::None, Checkpoint::Slicing, Checkpoint::Print,
                                        Checkpoint::Ordering, Checkpoint::Plan, Checkpoint::GCode}) {
        std::map<slicing_step_t, PrintStateBase::StateWithWarnings> before;
        for (const slicing_step_t step : Steps::execution_order()) {
            print.mark_step_executed(step);
            before.emplace(step, print.step_state_with_warnings(step));
        }
        if (worker_owned) {
            CHECK(print.request_replay_from_worker(checkpoint) == (checkpoint != Checkpoint::GCode));
            CHECK_FALSE(print.request_replay_from_worker(checkpoint));
        }
        else {
            std::scoped_lock<std::mutex> lock(print.state_mutex());
            CHECK(print.request_replay_from(checkpoint) == (checkpoint != Checkpoint::GCode));
            CHECK_FALSE(print.request_replay_from(checkpoint));
        }
        for (const slicing_step_t step : Steps::execution_order()) {
            // The export is never invalidated by a replay: it is queued by an export request only.
            const bool invalid = checkpoint != Checkpoint::GCode && step != STEP_EXPORT &&
                !Steps::executes_before(step, Steps::checkpoint_resume_step(checkpoint));
            const PrintStateBase::StateWithWarnings state = print.step_state_with_warnings(step);
            CAPTURE(checkpoint, step);
            CHECK(state.enabled);
            CHECK(state.is_done() == !invalid);
            if (invalid)
                CHECK(state.timestamp > before.at(step).timestamp);
            else
                CHECK(state.timestamp == before.at(step).timestamp);
        }
    }
    print.set_running_step(STEP_GCODE);
    print.clear();
    CHECK(print.running_step() == STEP_NONE);
    CHECK(print.step_state_with_warnings(STEP_SLICING).timestamp == 0);
}

TEST_CASE("Pipeline control replay captures a running step before the stop callback clears it",
          "[plugins][step-pipeline][pipeline-state][cancellation]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    using namespace Slic3r;
    LifecyclePrint print;
    print.set_running_step(STEP_PERIMETER);
    print.set_cancel_callback([&]() {
        print.state_mutex().unlock();
        print.set_running_step(STEP_NONE);
        print.state_mutex().lock();
    });
    {
        std::scoped_lock<std::mutex> lock(print.state_mutex());
        CHECK(print.request_replay_from(Steps::Checkpoint::Slicing));
        CHECK_FALSE(print.request_replay_from(Steps::Checkpoint::Slicing));
    }
    CHECK_FALSE(print.step_state_with_warnings(STEP_PERIMETER).is_done());
    print.set_cancel_callback([]() {});
}

TEST_CASE("Apply status distinguishes fresh pending steps from calculated outputs",
          "[plugins][step-pipeline][invalidation][config][apply-status]")
{
    using namespace Slic3r;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    DynamicPrintConfig config = Test::PerimeterPluginTests::perimeter_config({});
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
    config.set_deserialize_strict("perimeters", "2");
    CHECK(print.apply(model, config) == Print::APPLY_STATUS_CHANGED);
    CHECK(print.apply(model, config) == Print::APPLY_STATUS_UNCHANGED);
    print.process();
    const Layer *retained_layer = &print.object(0).layer(0);
    config.set_deserialize_strict("temperature", "210");
    CHECK(print.apply(model, config) == Print::APPLY_STATUS_CHANGED);
    CHECK(print.slices_ready());
    CHECK(&print.object(0).layer(0) == retained_layer);
    config.set_deserialize_strict("perimeters", "3");
    CHECK(print.apply(model, config) == Print::APPLY_STATUS_INVALIDATED);
    CHECK_FALSE(print.slices_ready());
    CHECK(&print.object(0).layer(0) == retained_layer);
    CHECK(print.apply(model, config) == Print::APPLY_STATUS_UNCHANGED);
}

TEST_CASE("Apply retains the affected first running step across stops and deeper replay",
          "[plugins][step-pipeline][invalidation][apply-status]")
{
    using namespace Slic3r;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool running = GENERATE(false, true);
    LifecyclePrint print;
    Model model;
    DynamicPrintConfig config = Test::PerimeterPluginTests::perimeter_config({});
    Test::init_print({Test::TestMesh::cube_20x20x20, Test::TestMesh::cube_20x20x20}, print, model, config);
    print.prepare_checkpoint_resume();
    for (const slicing_step_t step : Steps::execution_order())
        REQUIRE_FALSE(print.is_step_done(step));
    if (running)
        print.set_running_step(STEP_LAYER_HEIGHT);
    bool stopped_height = false;
    print.set_cancel_callback([&print, &stopped_height]() {
        stopped_height |= print.running_step() == STEP_LAYER_HEIGHT;
        print.state_mutex().unlock();
        print.set_running_step(STEP_NONE);
        print.state_mutex().lock();
    });
    Print::ApplyStatus status = Print::APPLY_STATUS_UNCHANGED;
    SECTION("model replacement pre-stop") {
        Model replacement;
        replacement.add_object(model.objects()[0]);
        status = print.apply(replacement, config);
        CHECK(print.apply(replacement, config) == Print::APPLY_STATUS_UNCHANGED);
    }
    SECTION("removal after an early export-only replay") {
        config.set_deserialize_strict("temperature", "210");
        model.delete_object(size_t(1));
        status = print.apply(model, config);
        CHECK(print.apply(model, config) == Print::APPLY_STATUS_UNCHANGED);
    }
    SECTION("object addition after an early export-only replay") {
        config.set_deserialize_strict("temperature", "210");
        model.add_object(model.objects()[0]);
        status = print.apply(model, config);
        CHECK(print.apply(model, config) == Print::APPLY_STATUS_UNCHANGED);
    }
    SECTION("export-only replay does not affect the running prefix") {
        config.set_deserialize_strict("temperature", "210");
        status = print.apply(model, config);
        CHECK(status == Print::APPLY_STATUS_CHANGED);
        CHECK(print.apply(model, config) == Print::APPLY_STATUS_UNCHANGED);
        print.set_cancel_callback([]() {});
        return;
    }
    CHECK(status == (running ? Print::APPLY_STATUS_INVALIDATED : Print::APPLY_STATUS_CHANGED));
    CHECK(stopped_height == running);
    CHECK(print.current_checkpoint() == Steps::Checkpoint::None);
    for (const slicing_step_t step : Steps::execution_order()) {
        CHECK_FALSE(print.is_step_done(step));
        // A full replay queues every step except the export, which only an export request queues.
        CHECK(print.should_execute_step(step) == (step != STEP_EXPORT));
    }
    print.set_cancel_callback([]() {});
}

TEST_CASE("Instance shifts and count changes replay adhesion while retaining slicing",
          "[plugins][step-pipeline][invalidation][apply-status]")
{
    using namespace Slic3r;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    DynamicPrintConfig config = Test::PerimeterPluginTests::perimeter_config({});
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
    print.process();
    for (const slicing_step_t step : Steps::execution_order())
        print.mark_step_executed(step);
    const Layer *retained_layer = &print.object(0).layer(0);
    const PrintStateBase::TimeStamp slicing_timestamp = print.step_state_with_timestamp(STEP_POST_SLICING).timestamp;
    SECTION("shift") {
        model.objects()[0].instances.front()->set_offset(Vec3d(40., 40., 0.));
    }
    SECTION("count") {
        model.objects()[0].add_instance()->set_offset(Vec3d(40., 40., 0.));
    }
    CHECK(print.apply(model, config) == Print::APPLY_STATUS_INVALIDATED);
    CHECK(print.object(0).instances().size() == model.objects()[0].instances.size());
    CHECK(&print.object(0).layer(0) == retained_layer);
    CHECK(print.is_step_done(STEP_POST_SLICING));
    CHECK(print.step_state_with_timestamp(STEP_POST_SLICING).timestamp == slicing_timestamp);
    CHECK(print.should_execute_step(STEP_SKIRT_BRIM));
    CHECK(print.should_execute_step(STEP_WIPETOWER));
    CHECK(print.should_execute_step(STEP_GCODE));
    CHECK_FALSE(print.slices_ready());
    CHECK(print.apply(model, config) == Print::APPLY_STATUS_UNCHANGED);
}

TEST_CASE("Canonical config invalidation follows metadata and returns affected booleans",
          "[plugins][step-pipeline][invalidation][config][apply-status]")
{
    using namespace Slic3r;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    LifecyclePrint print;
    Model model;
    DynamicPrintConfig config = Test::PerimeterPluginTests::perimeter_config({});
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
    for (const slicing_step_t step : Steps::execution_order())
        print.mark_step_executed(step);
    {
        std::scoped_lock<std::mutex> lock(print.state_mutex());
        CHECK(print.invalidate_state_by_config_options(config, {"perimeters"}));
        CHECK_FALSE(print.invalidate_state_by_config_options(config, {"perimeters"}));
        CHECK_FALSE(print.invalidate_state_by_config_options(config, {"temperature"}));
    }
    CHECK(print.is_step_done(STEP_POST_SLICING));
    CHECK(print.should_execute_step(STEP_PERIMETER));
    CHECK_FALSE(print.is_step_done(STEP_FINALIZE_SLICING));
}

TEST_CASE("Task bounds only stop the worker and slicing still completes the full prefix",
          "[plugins][step-pipeline][pipeline-state]")
{
    using namespace Slic3r;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model,
                     Test::PerimeterPluginTests::perimeter_config({}));
    Print::TaskParams task;
    task.to_object_step = STEP_SLICING;
    task.single_model_object = print.object(0).model_object()->id();
    task.single_model_instance_only = true;
    size_t stops = 0;
    print.set_cancel_callback([&]() { ++stops; });
    print.set_task(task);
    CHECK(stops == 1);
    print.process();
    CHECK(print.step_state_with_warnings(STEP_PERIMETER).is_done());
    CHECK(print.step_state_with_warnings(STEP_PERIMETER).enabled);
    CHECK(print.is_step_done(STEP_PERIMETER));
    CHECK(print.step_state_with_timestamp(STEP_PERIMETER).enabled);
    REQUIRE(print.object(0).layer_count() > 0);
    bool has_perimeters = false;
    for (const LayerSliceIsland &island : print.object(0).layer(0).islands())
        for (const LayerRegionIsland &region_island : island.regions_islands())
            has_perimeters |= region_island.has_extrusion(LayerRegionIsland::PERIMETERS);
    CHECK(has_perimeters);
    CHECK(print.slices_ready());
    for (const slicing_step_t step : Steps::execution_order())
        if (!Steps::executes_before(STEP_FINALIZE_SLICING, step))
            CHECK(print.is_step_done(step));
    const PrintStateBase::TimeStamp finalized = print.step_state_with_timestamp(STEP_FINALIZE_SLICING).timestamp;
    print.finalize();
    CHECK(print.step_state_with_timestamp(STEP_FINALIZE_SLICING).timestamp == finalized);
    CHECK(stops == 1);
    print.set_cancel_callback([]() {});
}

TEST_CASE("Pipeline completion cannot publish done after cancellation",
          "[plugins][step-pipeline][pipeline-state][cancellation]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    using namespace Slic3r;
    Print print;
    print.set_running_step(STEP_PERIMETER);
    const PrintStateBase::TimeStamp started = print.step_state_with_warnings(STEP_PERIMETER).timestamp;
    print.cancel_internal();
    print.set_running_step(STEP_NONE);
    CHECK_THROWS_AS(print.mark_step_executed(STEP_PERIMETER), CanceledException);
    CHECK_FALSE(print.step_state_with_warnings(STEP_PERIMETER).is_done());
    CHECK(print.step_state_with_warnings(STEP_PERIMETER).timestamp > started);
    print.restart();
}

TEST_CASE("Pipeline warnings change the clock and are retired on successful retry",
          "[plugins][step-pipeline][pipeline-state][warnings]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    using namespace Slic3r;
    Print print;
    size_t notifications = 0;
    Print::StatusMonitor facade(print);
    print.set_status_callback([&](const PrintBase::SlicingStatus &status) {
        if (status.flags & PrintBase::SlicingStatus::UPDATE_PRINT_STEP_WARNINGS) {
            CHECK(status.warning_step == STEP_GCODE);
            print.step_state_with_warnings(STEP_GCODE);
            ++notifications;
        }
    });
    print.set_running_step(STEP_GCODE);
    const PrintStateBase::TimeStamp started = print.step_state_with_warnings(STEP_GCODE).timestamp;
    CHECK(print.step_state_with_warnings(STEP_GCODE).state == PrintStateBase::State::Started);
    facade.active_step_add_warning(PrintStateBase::WarningLevel::NON_CRITICAL, "first", 71);
    const PrintStateBase::TimeStamp added = print.step_state_with_warnings(STEP_GCODE).timestamp;
    CHECK(added > started);
    facade.active_step_add_warning(PrintStateBase::WarningLevel::NON_CRITICAL, "first", 71);
    CHECK(print.step_state_with_warnings(STEP_GCODE).timestamp == added);
    facade.active_step_add_warning(PrintStateBase::WarningLevel::CRITICAL, "updated", 71);
    CHECK(print.step_state_with_warnings(STEP_GCODE).timestamp > added);
    REQUIRE(print.step_state_with_warnings(STEP_GCODE).warnings.size() == 1);
    CHECK(print.step_state_with_warnings(STEP_GCODE).warnings.front().message == "updated");
    print.set_running_step(STEP_NONE);
    print.mark_step_executed(STEP_GCODE);
    REQUIRE(print.request_replay_from_worker(Steps::Checkpoint::Plan));
    CHECK_FALSE(print.step_state_with_warnings(STEP_GCODE).warnings[0].current);
    print.set_running_step(STEP_GCODE);
    print.set_running_step(STEP_NONE);
    print.mark_step_executed(STEP_GCODE);
    CHECK(print.step_state_with_warnings(STEP_GCODE).warnings.empty());
    CHECK(notifications == 3);
}

TEST_CASE("Idle host warnings are transient and real-step null messages remain unknown",
          "[plugins][step-pipeline][pipeline-state][warnings][host-attribution]")
{
    using namespace Slic3r;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    orchestrator.consume_plugin_messages();
    orchestrator.reset_plugin_cancel();
    Print print;
    Print::StatusMonitor facade(print);
    facade.active_step_add_warning(PrintStateBase::WarningLevel::NON_CRITICAL, "host warning");
    facade.active_step_add_warning(PrintStateBase::WarningLevel::CRITICAL, "host error");
    const std::vector<Orchestrator::PluginMessage> messages = orchestrator.consume_plugin_messages();
    REQUIRE(messages.size() == 2);
    CHECK(messages[0].level == Orchestrator::PluginMessageLevel::Warning);
    CHECK(messages[1].level == Orchestrator::PluginMessageLevel::Error);
    CHECK(messages[0].message == "host warning");
    CHECK(messages[1].message == "host error");
    for (const Orchestrator::PluginMessage &message : messages) {
        CHECK(message.plugin_id == "<host>");
        CHECK(message.step == STEP_NONE);
    }
    for (const slicing_step_t step : Steps::execution_order()) {
        CHECK(print.step_state_with_warnings(step).warnings.empty());
        CHECK(print.step_state_with_timestamp(step).timestamp == 0);
    }
    CHECK_FALSE(print.canceled());
    CHECK_FALSE(orchestrator.is_plugin_cancelled());
    Plugin *writer = orchestrator.get_plugin("gcode.printing_plan_file_writer");
    REQUIRE(writer != nullptr);
    orchestrator.add_plugin_message(Orchestrator::PluginMessageLevel::Error, nullptr, STEP_ORDERING, "unattributed step");
    orchestrator.add_plugin_message(Orchestrator::PluginMessageLevel::Warning, writer, STEP_NONE, "plugin initialization");
    const std::vector<Orchestrator::PluginMessage> attributed = orchestrator.consume_plugin_messages();
    REQUIRE(attributed.size() == 2);
    CHECK(attributed[0].plugin_id == "<unknown>");
    CHECK(attributed[0].step == STEP_ORDERING);
    CHECK(attributed[1].plugin_id == writer->get_id());
    CHECK(attributed[1].step == STEP_NONE);
}

TEST_CASE("Slices readiness waits for Finalize and closes on a Slicing replay",
          "[plugins][step-pipeline][pipeline-state][readiness]")
{
    using namespace Slic3r;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const ScopedMetadataPlugins plugins;
    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model,
                     Test::PerimeterPluginTests::perimeter_config({}));
    CHECK_FALSE(print.slices_ready());
    print.process();
    CHECK_FALSE(ScopedMetadataPlugins::post().observed_slices_ready);
    CHECK_FALSE(ScopedMetadataPlugins::skirt().observed_slices_ready);
    CHECK(print.slices_ready());
    CHECK(print.is_step_done(STEP_FINALIZE_SLICING));
    REQUIRE(print.request_replay_from_worker(Steps::Checkpoint::Slicing));
    CHECK(print.is_step_done(STEP_POST_SLICING));
    CHECK_FALSE(print.is_step_done(STEP_FINALIZE_SLICING));
    CHECK_FALSE(print.slices_ready());
    print.process();
    CHECK_FALSE(ScopedMetadataPlugins::skirt().observed_slices_ready);
    CHECK(print.slices_ready());
}

TEST_CASE("Metadata checkpoints retain PostSlicing and roll back SkirtBrim across retries",
          "[plugins][checkpoint][metadata]")
{
    using namespace Slic3r;
    using Steps::Checkpoint;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const ScopedMetadataPlugins plugins;
    LifecyclePrint print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model,
                     Test::PerimeterPluginTests::perimeter_config({}));
    print.records().get_or_add("test.metadata").set_key_value("value", new ConfigOptionInt(10));
    set_metadata_property(print.object(0), 10);
    Steps::StepPipeline::run_slice(Orchestrator::instance(), print);
    REQUIRE(print.has_metadata_checkpoint(Checkpoint::None));
    REQUIRE(print.has_metadata_checkpoint(Checkpoint::Slicing));
    CHECK(ScopedMetadataPlugins::post().observed_record == 10);
    CHECK(ScopedMetadataPlugins::post().observed_property == 10);
    CHECK(ScopedMetadataPlugins::skirt().observed_record == 20);
    CHECK(ScopedMetadataPlugins::skirt().observed_property == 20);
    CHECK(metadata_property(print.object(0)) == 30);
    const DynamicConfig *record = print.records().find("test.metadata");
    REQUIRE(record != nullptr);

    print.request_replay_from_worker(Checkpoint::Slicing);
    LifecycleClearPlugin &probe = prepare_lifecycle_probe(print);
    probe.fail_clear = true;
    CHECK_THROWS_AS(print.prepare_checkpoint_resume(), PluginCleanupError);
    CHECK(print.has_metadata_checkpoint(Checkpoint::None));
    CHECK(print.has_metadata_checkpoint(Checkpoint::Slicing));
    CHECK(metadata_property(print.object(0)) == 30);
    CHECK(record->option<ConfigOptionInt>("value")->value == 30);
    probe.fail_clear = false;
    for (int retry = 0; retry < 2; ++retry) {
        print.prepare_checkpoint_resume();
        CHECK(print.records().find("test.metadata") == record);
        CHECK(record->option<ConfigOptionInt>("value")->value == 20);
        CHECK(metadata_property(print.object(0)) == 20);
        REQUIRE(print.records().find("test.metadata.post") != nullptr);
        CHECK(print.records().find("test.metadata.post")->option<ConfigOptionInt>("value")->value == 20);
        CHECK(print.records().find("test.metadata.skirt") == nullptr);
        CHECK(print.object(0).has_property(ScopedMetadataPlugins::post().phase_property_type));
        CHECK_FALSE(print.object(0).has_property(ScopedMetadataPlugins::skirt().phase_property_type));
        set_metadata_property(print.object(0), 99);
        print.records().get_or_add("test.metadata").set_key_value("value", new ConfigOptionInt(99));
    }
    Steps::StepPipeline::run_slice(Orchestrator::instance(), print);
    CHECK(ScopedMetadataPlugins::skirt().observed_record == 20);
    CHECK(metadata_property(print.object(0)) == 30);

    print.request_replay_from_worker(Checkpoint::None);
    CHECK_FALSE(print.has_metadata_checkpoint(Checkpoint::Slicing));
    print.prepare_checkpoint_resume();
    CHECK(record->option<ConfigOptionInt>("value")->value == 10);
    CHECK(metadata_property(print.object(0)) == 10);
    CHECK(print.records().find("test.metadata.post") == nullptr);
    CHECK(print.records().find("test.metadata.skirt") == nullptr);
    CHECK_FALSE(print.object(0).has_property(ScopedMetadataPlugins::post().phase_property_type));
    CHECK_FALSE(print.object(0).has_property(ScopedMetadataPlugins::skirt().phase_property_type));
    Steps::StepPipeline::run_slice(Orchestrator::instance(), print);
    CHECK(ScopedMetadataPlugins::post().observed_record == 10);
    CHECK(ScopedMetadataPlugins::post().observed_property == 10);
    print.clear();
    CHECK_FALSE(print.has_metadata_checkpoint(Checkpoint::None));
    CHECK_FALSE(print.has_metadata_checkpoint(Checkpoint::Slicing));
}

TEST_CASE("PostSlicing failures never publish a metadata checkpoint",
          "[plugins][checkpoint][metadata][cancellation]")
{
    using namespace Slic3r;
    using Steps::Checkpoint;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const ScopedMetadataPlugins plugins;
    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model,
                     Test::PerimeterPluginTests::perimeter_config({}));
    SECTION("cancellation") { ScopedMetadataPlugins::post().cancel_setup = true; }
    SECTION("plugin error") { ScopedMetadataPlugins::post().fail_setup = true; }
    CHECK_THROWS(Steps::StepPipeline::run_slice(Orchestrator::instance(), print));
    CHECK(print.has_metadata_checkpoint(Checkpoint::None));
    CHECK_FALSE(print.has_metadata_checkpoint(Checkpoint::Slicing));
    CHECK(print.should_execute_step(STEP_POST_SLICING));
    CHECK_FALSE(print.step_state_with_warnings(STEP_POST_SLICING).is_done());
    CHECK(print.running_step() == STEP_NONE);
    CHECK(print.step_state_with_warnings(STEP_POST_SLICING).timestamp >
          print.step_state_with_warnings(STEP_SLICING).timestamp);
    ScopedMetadataPlugins::post().cancel_setup = false;
    ScopedMetadataPlugins::post().fail_setup = false;
    print.restart();
    Steps::StepPipeline::run_slice(Orchestrator::instance(), print);
    CHECK(print.has_metadata_checkpoint(Checkpoint::Slicing));
}

TEST_CASE("Metadata restoration clears newly added objects and forgets removed owners",
          "[plugins][checkpoint][metadata][lifetime]")
{
    using namespace Slic3r;
    using Steps::Checkpoint;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    LifecyclePrint print;
    Model model;
    const DynamicPrintConfig config = Test::PerimeterPluginTests::perimeter_config({});
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
    set_metadata_property(print.object(0), 10);
    print.save_metadata_checkpoint(Checkpoint::None);
    print.save_metadata_checkpoint(Checkpoint::Slicing);
    model.add_object(model.objects()[0]);
    print.apply(model, config);
    REQUIRE(print.objects().size() == 2);
    set_metadata_property(print.object(0), 90);
    set_metadata_property(print.object(1), 99);
    print.prepare_checkpoint_resume();
    CHECK(metadata_property(print.object(0)) == 10);
    CHECK_FALSE(print.object(1).has_properties());

    LifecycleClearPlugin &probe = prepare_lifecycle_probe(print);
    model.delete_object(size_t(0));
    probe.fail_clear = true;
    CHECK_THROWS_AS(print.apply(model, config), PluginCleanupError);
    CHECK(print.has_metadata_checkpoint(Checkpoint::None));
    REQUIRE(print.objects().size() == 2);
    probe.fail_clear = false;
    print.apply(model, config);
    REQUIRE(print.objects().size() == 1);
    set_metadata_property(print.object(0), 77);
    print.prepare_checkpoint_resume();
    CHECK_FALSE(print.object(0).has_properties());
}

TEST_CASE("Print releases plugin state before destroying its owners", "[plugins][clear][checkpoint][lifetime]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    std::unique_ptr<LifecyclePrint> print = std::make_unique<LifecyclePrint>();
    print->mutable_printing_plan().groups.emplace_back();
    LifecycleClearPlugin &probe = prepare_lifecycle_probe(*print);
    SECTION("explicit clear") {
        print->clear();
        CHECK(print->plugin_cleanup_participants().empty());
        CHECK(print->printing_plan() == nullptr);
        print.reset();
    }
    SECTION("destruction") {
        print.reset();
    }
    CHECK(probe.clears == 1);
    CHECK(probe.owners_alive);
    CHECK(probe.mutex_unlocked);
}

TEST_CASE("Invalidation retains the live plan until plugin cleanup succeeds", "[plugins][clear][checkpoint][lifetime]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    LifecyclePrint print;
    for (const slicing_step_t step : Slic3r::Steps::execution_order())
        print.mark_step_executed(step);
    print.mutable_printing_plan().groups.emplace_back();
    const Slic3r::Printing::PrintingPlan *plan = print.printing_plan();
    LifecycleClearPlugin &probe = prepare_lifecycle_probe(print);
    print.request_replay_from_worker(Slic3r::Steps::Checkpoint::Slicing);
    CHECK(print.printing_plan() == plan);
    CHECK(probe.clears == 0);

    probe.fail_clear = true;
    CHECK_THROWS(print.prepare_checkpoint_resume());
    CHECK(print.printing_plan() == plan);
    CHECK(print.plugin_cleanup_participants().size() == 1);
    CHECK(probe.owners_alive);

    probe.fail_clear = false;
    print.prepare_checkpoint_resume();
    CHECK(print.printing_plan() == nullptr);
    CHECK(print.plugin_cleanup_participants().empty());
    CHECK(probe.clears == 2);
    CHECK(probe.mutex_unlocked);
}

TEST_CASE("Apply notifies plugins before replacing model owners", "[plugins][clear][checkpoint][lifetime]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    LifecyclePrint print;
    Slic3r::Model model;
    const Slic3r::DynamicPrintConfig config = Slic3r::DynamicPrintConfig::full_print_config();
    Slic3r::Test::init_print({Slic3r::Test::TestMesh::cube_20x20x20}, print, model, config);
    for (const slicing_step_t step : Slic3r::Steps::execution_order())
        print.mark_step_executed(step);
    print.mutable_printing_plan().groups.emplace_back();
    LifecycleClearPlugin &probe = prepare_lifecycle_probe(print);

    Slic3r::Model replacement;
    print.apply(replacement, config);
    CHECK(probe.clears == 1);
    CHECK(probe.owners_alive);
    CHECK(probe.mutex_unlocked);
    CHECK(print.objects().empty());
    CHECK(print.printing_plan() == nullptr);
    CHECK(print.plugin_cleanup_participants().empty());
}

TEST_CASE("Failed apply cleanup preserves owners for retry", "[plugins][clear][checkpoint][lifetime]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    LifecyclePrint print;
    Slic3r::Model model;
    const Slic3r::DynamicPrintConfig config = Slic3r::DynamicPrintConfig::full_print_config();
    Slic3r::Test::init_print({Slic3r::Test::TestMesh::cube_20x20x20,
                            Slic3r::Test::TestMesh::cube_20x20x20}, print, model, config);
    for (const slicing_step_t step : Slic3r::Steps::execution_order())
        print.mark_step_executed(step);
    print.mutable_printing_plan().groups.emplace_back();
    const Slic3r::PrintObject *first_object = &print.object(0);
    const Slic3r::ModelObject *first_model = first_object->model_object();
    const Slic3r::Printing::PrintingPlan *plan = print.printing_plan();
    LifecycleClearPlugin &probe = prepare_lifecycle_probe(print);

    SECTION("removed model object") {
        model.delete_object(size_t(0));
    }
    SECTION("replaced transformed object") {
        model.objects()[1].instances.front()->set_scaling_factor(Slic3r::Vec3d(2., 2., 2.));
    }
    SECTION("reused object with changed XY offset") {
        model.objects()[1].instances.front()->set_offset(Slic3r::Vec3d(40., 40., 0.));
    }
    probe.fail_clear = true;
    CHECK_THROWS(print.apply(model, config));
    REQUIRE(print.objects().size() == 2);
    CHECK(&print.object(0) == first_object);
    CHECK(print.object(0).model_object() == first_model);
    CHECK(print.printing_plan() == plan);
    CHECK(print.plugin_cleanup_participants().size() == 1);
    CHECK(probe.owners_alive);
    CHECK(probe.mutex_unlocked);

    probe.fail_clear = false;
    CHECK_NOTHROW(print.apply(model, config));
    CHECK(probe.clears == 2);
    CHECK(print.plugin_cleanup_participants().empty());
}

TEST_CASE("Apply cleanup retry synchronizes effective object configs", "[plugins][clear][config][checkpoint][lifetime]")
{
    using namespace Slic3r;
    // Concurrent geometry creation can abort before this fixture reaches the cleanup barrier.
    const tbb::global_control slicing_concurrency(tbb::global_control::max_allowed_parallelism, 1);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    LifecyclePrint print;
    Model model;
    DynamicPrintConfig config = Test::PerimeterPluginTests::perimeter_config({{"support_material_spacing", "2.5"}});
    Test::init_print({Test::TestMesh::cube_20x20x20, Test::TestMesh::cube_20x20x20}, print, model, config);
    Steps::StepLayerHeightGeneration::run_step(Orchestrator::instance(), print);
    Steps::StepSlicing::run_step(Orchestrator::instance(), print);
    for (const slicing_step_t step : Steps::execution_order())
        print.mark_step_executed(step);
    REQUIRE(print.objects().size() == 2);

    struct SliceSnapshot {
        const Layer *owner;
        ExPolygons raw_slices;
        ExPolygons islands;
    };
    std::vector<const PrintObject *> objects;
    std::vector<const ModelObject *> model_objects;
    std::vector<std::vector<coord_t>> profiles;
    std::vector<std::vector<SliceSnapshot>> slices;
    for (const PrintObject &object : print.objects()) {
        objects.push_back(&object);
        model_objects.push_back(object.model_object());
        profiles.push_back(object.layer_profile());
        REQUIRE_FALSE(profiles.back().empty());
        REQUIRE(object.layer_count() > 0);
        slices.emplace_back();
        for (size_t idx = 0; idx < object.layer_count(); ++idx) {
            const Layer &layer = object.layer(idx);
            REQUIRE(layer.region_count() == 1);
            SliceSnapshot snapshot {&layer, layer.region(0).get_raw_slices(), {}};
            for (const LayerSliceIsland &island : layer.islands())
                snapshot.islands.push_back(island.get_slice());
            slices.back().push_back(std::move(snapshot));
        }
    }
    const auto check_owners = [&]() {
        REQUIRE(print.objects().size() == objects.size());
        for (size_t idx = 0; idx < objects.size(); ++idx) {
            CAPTURE(idx);
            const PrintObject &object = print.object(idx);
            CHECK(&object == objects[idx]);
            CHECK(object.model_object() == model_objects[idx]);
            CHECK(object.layer_profile() == profiles[idx]);
            REQUIRE(object.layer_count() == slices[idx].size());
            for (size_t layer_idx = 0; layer_idx < slices[idx].size(); ++layer_idx) {
                const Layer &layer = object.layer(layer_idx);
                const SliceSnapshot &snapshot = slices[idx][layer_idx];
                CHECK(&layer == snapshot.owner);
                REQUIRE(layer.region_count() == 1);
                CHECK(layer.region(0).get_raw_slices() == snapshot.raw_slices);
                REQUIRE(layer.islands().size() == snapshot.islands.size());
                for (size_t island_idx = 0; island_idx < snapshot.islands.size(); ++island_idx)
                    CHECK(layer.island(island_idx).get_slice() == snapshot.islands[island_idx]);
            }
        }
        CHECK(print.current_checkpoint() == Steps::Checkpoint::Slicing);
        CHECK_FALSE(print.should_execute_step(STEP_LAYER_HEIGHT));
        CHECK_FALSE(print.should_execute_step(STEP_SLICING));
        CHECK_FALSE(print.should_execute_step(STEP_POST_SLICING));
    };
    print.mutable_printing_plan().groups.emplace_back();
    const Printing::PrintingPlan *plan = print.printing_plan();
    LifecycleClearPlugin &probe = prepare_lifecycle_probe(print);
    bool object_override = false;
    bool newer_request = false;
    SECTION("global change on multiple objects") {}
    SECTION("object override with matching timestamp on retry") { object_override = true; }
    SECTION("new global value on retry") { newer_request = true; }
    SECTION("new object override on retry") { object_override = true; newer_request = true; }
    if (object_override)
        model.objects()[0].config.set_deserialize_strict("support_material_spacing", "3.5");
    else
        config.set_deserialize_strict("support_material_spacing", "3.5");
    probe.fail_clear = true;
    for (size_t attempt = 0; attempt < 2; ++attempt) {
        CHECK_THROWS_AS(print.apply(model, config), PluginCleanupError);
        CHECK(probe.clears == attempt + 1);
        CHECK(probe.owners_alive);
        CHECK(probe.mutex_unlocked);
        CHECK(print.printing_plan() == plan);
        REQUIRE(print.plugin_cleanup_participants().size() == 1);
        CHECK(print.plugin_cleanup_participants().front().plugin ==
              Orchestrator::instance().get_plugin("test.lifecycle.clear"));
        check_owners();
        for (size_t idx = 0; idx < objects.size(); ++idx)
            CHECK(print.object(idx).config().support_material_spacing.value == 2.5);
    }
    if (newer_request) {
        if (object_override)
            model.objects()[0].config.set_deserialize_strict("support_material_spacing", "4.5");
        else
            config.set_deserialize_strict("support_material_spacing", "4.5");
    }
    probe.fail_clear = false;
    CHECK(print.apply(model, config) == Print::APPLY_STATUS_INVALIDATED);
    CHECK(probe.clears == 3);
    CHECK(print.plugin_cleanup_participants().empty());
    CHECK(print.printing_plan() == nullptr);
    check_owners();
    for (size_t idx = 0; idx < objects.size(); ++idx) {
        const double expected = object_override && idx != 0 ? 2.5 : newer_request ? 4.5 : 3.5;
        CHECK(print.object(idx).config().support_material_spacing.value == expected);
    }
    CHECK(print.apply(model, config) == Print::APPLY_STATUS_UNCHANGED);
    CHECK(probe.clears == 3);
    check_owners();
}

TEST_CASE("Unchanged apply keeps plugin state and source owners", "[plugins][clear][checkpoint][lifetime]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    LifecyclePrint print;
    Slic3r::Model model;
    const Slic3r::DynamicPrintConfig config = Slic3r::DynamicPrintConfig::full_print_config();
    Slic3r::Test::init_print({Slic3r::Test::TestMesh::cube_20x20x20}, print, model, config);
    for (const slicing_step_t step : Slic3r::Steps::execution_order())
        print.mark_step_executed(step);
    print.mutable_printing_plan().groups.emplace_back();
    const Slic3r::Printing::PrintingPlan *plan = print.printing_plan();
    LifecycleClearPlugin &probe = prepare_lifecycle_probe(print);

    CHECK(print.apply(model, config) == Slic3r::Print::APPLY_STATUS_UNCHANGED);
    CHECK(probe.clears == 0);
    CHECK(print.printing_plan() == plan);
    CHECK(print.plugin_cleanup_participants().size() == 1);
}

TEST_CASE("Unchanged apply does not cancel an active execution with plugin state",
          "[plugins][clear][checkpoint][threading]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    LifecyclePrint print;
    Slic3r::Model model;
    const Slic3r::DynamicPrintConfig config = Slic3r::DynamicPrintConfig::full_print_config();
    Slic3r::Test::init_print({Slic3r::Test::TestMesh::cube_20x20x20}, print, model, config);
    for (const slicing_step_t step : Slic3r::Steps::execution_order())
        print.mark_step_executed(step);
    print.mutable_printing_plan().groups.emplace_back();
    print.request_replay_from_worker(Slic3r::Steps::Checkpoint::Plan);
    print.prepare_checkpoint_resume();
    LifecycleClearPlugin &probe = prepare_lifecycle_probe(print);
    Slic3r::Orchestrator &orchestrator = Slic3r::Orchestrator::instance();
    Slic3r::Plugin *plugin = orchestrator.get_plugin("test.lifecycle.clear");
    REQUIRE(plugin != nullptr);

    std::promise<void> started;
    std::promise<void> finish;
    std::future<void> finished = finish.get_future();
    std::atomic_bool released { false };
    const auto release_worker = [&]() {
        if (!released.exchange(true))
            finish.set_value();
    };
    std::thread worker([&]() {
        plugin_host_context host = orchestrator.prepare_plugin_host_context(STEP_EXTRUSION_EDIT, plugin, &print);
        const plugin_run_context context = orchestrator.prepare_plugin_run_context(STEP_EXTRUSION_EDIT, plugin, &host);
        plugin->setup(context, 0);
        print.request_replay_from_worker(Slic3r::Steps::Checkpoint::Plan);
        started.set_value();
        finished.wait();
    });
    size_t cancellations = 0;
    print.set_cancel_callback([&]() {
        ++cancellations;
        release_worker();
        if (worker.joinable())
            worker.join();
    });
    started.get_future().wait();

    CHECK(print.apply(model, config) == Slic3r::Print::APPLY_STATUS_UNCHANGED);
    CHECK(cancellations == 0);
    CHECK(probe.clears == 0);
    release_worker();
    if (worker.joinable())
        worker.join();
    print.set_cancel_callback([]() {});
    CHECK(print.plugin_cleanup_participants().size() == 1);
}

TEST_CASE("Terminal Print reset abandons failed cleanup and permits a new project",
          "[plugins][clear][checkpoint][lifetime]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    LifecyclePrint print;
    print.mutable_printing_plan().groups.emplace_back();
    LifecycleClearPlugin &probe = prepare_lifecycle_probe(print);
    probe.fail_clear = true;
    CHECK_NOTHROW(print.clear());
    CHECK(probe.clears == 1);
    CHECK(probe.owners_alive);
    CHECK(print.plugin_cleanup_participants().empty());
    CHECK(print.printing_plan() == nullptr);
    CHECK_NOTHROW(print.clear());
    CHECK(probe.clears == 1);

    Slic3r::Model model;
    CHECK_NOTHROW(print.apply(model, Slic3r::DynamicPrintConfig::full_print_config()));
    CHECK(probe.clears == 1);
}

TEST_CASE("Direct G-code export rejects an invalid slicing prefix before clearing owners",
          "[plugins][step-pipeline][checkpoint][gcode]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    using namespace Slic3r::Test::PerimeterPluginTests;
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({}));
    Slic3r::Print &print = prepared.print;
    REQUIRE(print.object(0).layer_count() > 0);
    const Slic3r::Layer *first_layer = &print.object(0).layer(0);
    const size_t layers = print.object(0).layer_count();
    print.mutable_printing_plan().groups.emplace_back();
    const Slic3r::Printing::PrintingPlan *plan = print.printing_plan();
    print.request_replay_from_worker(Slic3r::Steps::Checkpoint::None);

    CHECK_THROWS_AS(Slic3r::Steps::StepPipeline::run_gcode(
        Slic3r::Orchestrator::instance(), print, "must-not-be-written.gcode"), Slic3r::SlicingError);
    CHECK(print.object(0).layer_count() == layers);
    CHECK(&print.object(0).layer(0) == first_layer);
    CHECK(print.printing_plan() == plan);
    CHECK(print.should_execute_step(STEP_LAYER_HEIGHT));
    CHECK(print.should_execute_step(STEP_SLICING));
    CHECK(print.should_execute_step(STEP_POST_SLICING));
}

TEST_CASE("Checkpoint cleanup discards derived groups and auxiliary owners without changing source slices",
          "[plugins][step-pipeline][checkpoint][cleanup]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    using namespace Slic3r;
    using namespace Slic3r::Test::PerimeterPluginTests;
    bool local_source_alive = false;
    bool global_source_alive = false;
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({}));
    Print &print = prepared.print;
    PrintObject &object = print.object(0);
    REQUIRE(object.layer_count() > 0);
    Layer &layer = object.layer(0);
    REQUIRE_FALSE(layer.islands().empty());
    LayerSliceIsland &island = layer.island(0);
    const Layer *source_layer = &layer;
    const ExPolygon source_slice = island.get_slice();
    const ExPolygons raw_slices = layer.region(0).get_raw_slices();
    const std::vector<coord_t> profile = object.layer_profile();
    REQUIRE_FALSE(profile.empty());
    island.get_or_add_region_island(island.regions(), 0).mutable_extrusion(LayerRegionIsland::INFILLS);
    island.get_or_add_region_island(island.regions(), 1).mutable_extrusion(LayerRegionIsland::GAP_FILLS);
    ApiInternal::LayerIslandAccess::set_infill_areas(island, ExPolygons{rectangle_expolygon(0., 0., 1., 1.)});
    const ExPolygons &fill_cache = layer.region(0).fill_expolygons();
    REQUIRE_FALSE(fill_cache.empty());
    object.mutable_auxiliary_layers().emplace_back(
        std::make_unique<ObservedAuxiliaryLayer>(object, object, local_source_alive));
    PrintObject &global = print.mutable_auxiliary_object();
    global.mutable_auxiliary_layers().emplace_back(
        std::make_unique<ObservedAuxiliaryLayer>(global, object, global_source_alive));
    for (const slicing_step_t step : Steps::execution_order())
        print.mark_step_executed(step);

    SECTION("Slicing preserves its skeleton and removes every later group") {
        print.request_replay_from_worker(Steps::Checkpoint::Slicing);
        print.prepare_checkpoint_resume();
        CHECK(&object.layer(0) == source_layer);
        CHECK(object.layer(0).island(0).get_slice() == source_slice);
        CHECK(object.layer(0).region(0).get_raw_slices() == raw_slices);
        CHECK(object.layer_profile() == profile);
        CHECK(object.layer(0).island(0).regions_islands().empty());
        CHECK(object.layer(0).island(0).infill_areas().empty());
        CHECK(fill_cache.empty());
    }
    SECTION("None destroys auxiliary layers before clearing main layers and profile") {
        print.request_replay_from_worker(Steps::Checkpoint::None);
        print.prepare_checkpoint_resume();
        CHECK(object.layer_count() == 0);
        CHECK(object.layer_profile().empty());
    }
    CHECK(local_source_alive);
    CHECK(global_source_alive);
    CHECK(object.auxiliary_layers().empty());
    CHECK(print.auxiliary_object() == nullptr);
}

TEST_CASE("Control-thread invalidation stops the worker before scheduling or snapshot mutation",
          "[plugins][step-pipeline][invalidation][threading]")
{
    using namespace Slic3r;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    LifecyclePrint print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, DynamicPrintConfig::full_print_config());
    for (const slicing_step_t step : Steps::execution_order())
        print.mark_step_executed(step);
    print.mutable_printing_plan().groups.emplace_back();
    REQUIRE(print.save_ordering_checkpoint());

    std::promise<void> started;
    std::promise<void> finish;
    std::future<void> finish_requested = finish.get_future();
    bool worker_saw_old_schedule = false;
    bool worker_saw_old_snapshot = false;
    std::thread worker([&]() {
        started.set_value();
        finish_requested.wait();
        worker_saw_old_schedule = !print.should_execute_step(STEP_PERIMETER);
        worker_saw_old_snapshot = print.has_ordering_checkpoint();
        print.save_ordering_checkpoint();
        // Completion during the stop must not erase the replay that the UI is about to request.
        print.mark_step_executed(STEP_GCODE);
    });
    started.get_future().wait();
    size_t stops = 0;
    print.set_cancel_callback([&]() {
        ++stops;
        if (!worker.joinable())
            return;
        print.state_mutex().unlock();
        finish.set_value();
        worker.join();
        print.state_mutex().lock();
    });
    {
        std::unique_lock<std::mutex> lock(print.state_mutex());
        SECTION("checkpoint replay") {
            print.request_replay_from(Steps::Checkpoint::Slicing);
        }
        SECTION("complete invalidation") {
            print.request_replay_from(Steps::Checkpoint::None);
        }
        print.set_cancel_callback([]() {});
    }
    if (worker.joinable()) {
        finish.set_value();
        worker.join();
    }
    CHECK(stops == 1);
    CHECK(worker_saw_old_schedule);
    CHECK(worker_saw_old_snapshot);
    CHECK(print.should_execute_step(STEP_PERIMETER));
    CHECK(print.should_execute_step(STEP_GCODE));
    CHECK_FALSE(print.has_ordering_checkpoint());
}

TEST_CASE("Checkpoint replay invalidates and republishes every canonical Print reader",
          "[plugins][step-pipeline][invalidation][checkpoint][milestones]")
{
    using namespace Slic3r;
    using Steps::Checkpoint;
    const Checkpoint checkpoint = GENERATE(Checkpoint::None, Checkpoint::Slicing, Checkpoint::Print,
                                           Checkpoint::Ordering, Checkpoint::Plan, Checkpoint::GCode);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    LifecyclePrint print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, DynamicPrintConfig::full_print_config());
    const std::vector<slicing_step_t> &milestones = Steps::execution_order();
    std::vector<PrintStateBase::TimeStamp> timestamps;
    for (const slicing_step_t step : milestones) {
        print.mark_step_executed(step);
        timestamps.push_back(print.step_state_with_timestamp(step).timestamp);
    }
    // Phantom legacy milestones have no canonical producer or visible completion.
    for (const slicing_step_t step : {STEP_ALERT_SUPPORTS_NEEDED, STEP_CHECK_CONFLICT}) {
        CHECK_FALSE(print.is_step_done(step));
        CHECK(print.step_state_with_timestamp(step).timestamp == 0);
    }
    size_t stops = 0;
    print.set_cancel_callback([&]() { ++stops; });
    SECTION("control-thread request") {
        std::scoped_lock<std::mutex> lock(print.state_mutex());
        print.request_replay_from(checkpoint);
        CHECK(stops == (checkpoint == Checkpoint::GCode ? 0 : 1));
    }
    SECTION("worker request") {
        REQUIRE(print.object(0).slicing_parameters().valid);
        print.request_replay_from_worker(checkpoint);
        CHECK(stops == 0);
        CHECK(print.object(0).slicing_parameters().valid);
    }
    print.set_cancel_callback([]() {});
    for (size_t index = 0; index < milestones.size(); ++index) {
        const slicing_step_t step = milestones[index];
        // The export is never queued by a replay: it stays done until an export is requested.
        const bool invalidated = checkpoint != Checkpoint::GCode && step != STEP_EXPORT &&
            !Steps::executes_before(step, Steps::checkpoint_resume_step(checkpoint));
        CAPTURE(checkpoint, step);
        CHECK(print.is_step_done(step) == !invalidated);
        if (invalidated) {
            CHECK(print.step_state_with_timestamp(step).timestamp > timestamps[index]);
            print.set_running_step(step);
            CHECK(print.step_state_with_timestamp(step).state == PrintStateBase::State::Started);
            CHECK_FALSE(print.is_step_done(step));
            print.set_running_step(STEP_NONE);
            print.mark_step_executed(step);
            CHECK(print.step_state_with_timestamp(step).timestamp > timestamps[index]);
        } else {
            CHECK(print.step_state_with_timestamp(step).timestamp == timestamps[index]);
        }
        CHECK(print.is_step_done(step));
    }
}

TEST_CASE("Ordering clear destroys output without scheduling any step",
          "[plugins][step-pipeline][checkpoint][cleanup]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Slic3r::Print print;
    for (const slicing_step_t step : Slic3r::Steps::execution_order())
        print.mark_step_executed(step);
    print.mutable_printing_plan().groups.emplace_back();
    REQUIRE(print.save_ordering_checkpoint());
    Slic3r::Steps::StepExtrusionOrdering::clear(print);
    CHECK(print.printing_plan() == nullptr);
    CHECK_FALSE(print.has_ordering_checkpoint());
    for (const slicing_step_t step : Slic3r::Steps::execution_order())
        CHECK_FALSE(print.should_execute_step(step));
}

TEST_CASE("Step pipeline DAG invalidates only skirt/brim dependents", "[plugins][step-pipeline][invalidation]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    // The new pipeline is not topologically sorted at runtime. This test
    // protects the central dependency table used by Print invalidation: changing
    // skirt/brim must re-run its finalization consumers, but it must not force
    // upstream support producers or unrelated perimeter, surface and infill steps.
    const std::vector<slicing_step_t> dependents = Slic3r::Steps::dependent_steps_closure(STEP_SKIRT_BRIM);

    CHECK_FALSE(contains_step(dependents, STEP_SUPPORT_DEMAND));
    CHECK_FALSE(contains_step(dependents, STEP_SUPPORT));
    CHECK(contains_step(dependents, STEP_FINALIZE_SLICING));
    CHECK(contains_step(dependents, STEP_ORDERING));
    CHECK(contains_step(dependents, STEP_WIPETOWER));
    CHECK(contains_step(dependents, STEP_GCODE));

    CHECK_FALSE(contains_step(dependents, STEP_PERIMETER));
    CHECK_FALSE(contains_step(dependents, STEP_SURFACE_GENERATION));
    CHECK_FALSE(contains_step(dependents, STEP_INFILL));
    CHECK_FALSE(contains_step(dependents, STEP_POST_INFILL));

    // A fresh Print starts with a full run requested. Marking every known step
    // as executed gives the same empty plan that a successful process() leaves
    // behind, then the single-step invalidation should rebuild only the
    // skirt/brim branch and its declared downstream consumers.
    Slic3r::Print print;
    for (slicing_step_t step : Slic3r::Steps::execution_order())
        print.mark_step_executed(step);

    print.mark_step_and_dependents_for_execution(STEP_SKIRT_BRIM);

    CHECK(print.should_execute_step(STEP_SKIRT_BRIM));
    CHECK(print.should_execute_step(STEP_FINALIZE_SLICING));
    CHECK(print.should_execute_step(STEP_ORDERING));
    CHECK(print.should_execute_step(STEP_GCODE));

    CHECK_FALSE(print.should_execute_step(STEP_SUPPORT_DEMAND));
    CHECK_FALSE(print.should_execute_step(STEP_SUPPORT));
    CHECK_FALSE(print.should_execute_step(STEP_PERIMETER));
    CHECK_FALSE(print.should_execute_step(STEP_SURFACE_GENERATION));
    CHECK_FALSE(print.should_execute_step(STEP_INFILL));
    CHECK_FALSE(print.should_execute_step(STEP_POST_INFILL));
}

TEST_CASE("Step pipeline dependency order matches written execution order", "[plugins][step-pipeline][invalidation]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    // The dependency graph is only an invalidation graph. Runtime still follows
    // StepPipeline::execution_order(), so every edge must point forward in that
    // written order or a dependent step could be requested before its producer.
#ifdef _DEBUG
    CHECK(Slic3r::Steps::validate_execution_order_against_dependencies());
#else
    SUCCEED("Dependency order validation is a debug-only guard.");
#endif
}

TEST_CASE("Every executable step resumes from the last checkpoint before it",
          "[plugins][step-pipeline][invalidation][checkpoint]")
{
    using Slic3r::Steps::Checkpoint;

    const std::vector<std::pair<slicing_step_t, Checkpoint>> expected {
        { STEP_LAYER_HEIGHT, Checkpoint::None },
        { STEP_SLICING, Checkpoint::None },
        { STEP_POST_SLICING, Checkpoint::None },
        { STEP_SUPPORT_DEMAND, Checkpoint::Slicing },
        { STEP_PRE_PERIMETER, Checkpoint::Slicing },
        { STEP_PERIMETER, Checkpoint::Slicing },
        { STEP_POST_PERIMETER, Checkpoint::Slicing },
        { STEP_SURFACE_GENERATION, Checkpoint::Slicing },
        { STEP_PRE_INFILL, Checkpoint::Slicing },
        { STEP_INFILL_GROUP, Checkpoint::Slicing },
        { STEP_INFILL, Checkpoint::Slicing },
        { STEP_POST_INFILL, Checkpoint::Slicing },
        { STEP_SUPPORT, Checkpoint::Slicing },
        { STEP_SUPPORT_TOOLPATH, Checkpoint::Slicing },
        { STEP_SKIRT_BRIM, Checkpoint::Slicing },
        { STEP_FINALIZE_SLICING, Checkpoint::Slicing },
        { STEP_ORDERING, Checkpoint::Print },
        { STEP_WIPETOWER, Checkpoint::Ordering },
        { STEP_SUPPORT_SPOT, Checkpoint::Ordering },
        { STEP_LAYER_EXTRUSION_EDIT, Checkpoint::Ordering },
        { STEP_LAYER_STICHING, Checkpoint::Ordering },
        { STEP_EXTRUSION_EDIT, Checkpoint::Ordering },
        { STEP_EXTRUSION_SIMPLIFICATION, Checkpoint::Ordering },
        { STEP_GCODE, Checkpoint::Plan },
        { STEP_EXPORT, Checkpoint::GCode }
    };

    REQUIRE(expected.size() == Slic3r::Steps::execution_order().size());
    for (size_t index = 0; index < expected.size(); ++index) {
        CHECK(expected[index].first == Slic3r::Steps::execution_order()[index]);
        CHECK(Slic3r::Steps::checkpoint_before_step(expected[index].first) ==
              expected[index].second);
    }
    CHECK(Slic3r::Steps::checkpoint_before_step(STEP_ANY) == Checkpoint::None);

    // Each checkpoint resumes at the first step it does not contain.
    CHECK(Slic3r::Steps::checkpoint_resume_step(Checkpoint::None) == STEP_LAYER_HEIGHT);
    CHECK(Slic3r::Steps::checkpoint_resume_step(Checkpoint::Slicing) == STEP_SUPPORT_DEMAND);
    CHECK(Slic3r::Steps::checkpoint_resume_step(Checkpoint::Print) == STEP_ORDERING);
    CHECK(Slic3r::Steps::checkpoint_resume_step(Checkpoint::Ordering) == STEP_WIPETOWER);
    CHECK(Slic3r::Steps::checkpoint_resume_step(Checkpoint::Plan) == STEP_GCODE);
    CHECK(Slic3r::Steps::checkpoint_resume_step(Checkpoint::GCode) == STEP_EXPORT);

    // Numeric ABI values deliberately disagree with the execution order here.
    CHECK(STEP_SKIRT_BRIM < STEP_SUPPORT);
    CHECK(Slic3r::Steps::executes_before(STEP_SUPPORT, STEP_SKIRT_BRIM));
    CHECK(Slic3r::Steps::executes_before(STEP_SUPPORT, STEP_SUPPORT_TOOLPATH));
    CHECK(Slic3r::Steps::executes_before(STEP_SUPPORT_TOOLPATH, STEP_SKIRT_BRIM));
    CHECK(contains_step(Slic3r::Steps::dependent_steps_closure(STEP_SUPPORT), STEP_SUPPORT_TOOLPATH));
    CHECK(contains_step(Slic3r::Steps::dependent_steps_closure(STEP_SUPPORT_TOOLPATH), STEP_SKIRT_BRIM));
}

TEST_CASE("A setting read by a plugin invalidates no later than that plugin",
          "[plugins][step-pipeline][invalidation][config]")
{
    using Slic3r::Steps::tightened_invalidation_step;

    // A full replay and an explicit no-replay declaration are kept as is.
    CHECK(tightened_invalidation_step(STEP_ANY, STEP_INFILL) == STEP_ANY);
    CHECK(tightened_invalidation_step(STEP_NONE, STEP_INFILL) == STEP_NONE);

    // An earlier or equal step already replays the reader.
    CHECK(tightened_invalidation_step(STEP_PERIMETER, STEP_INFILL) == STEP_PERIMETER);
    CHECK(tightened_invalidation_step(STEP_INFILL, STEP_INFILL) == STEP_INFILL);
    // A later legacy step moves back to the reader.
    CHECK(tightened_invalidation_step(STEP_GCODE, STEP_INFILL) == STEP_INFILL);

    // Support runs before skirt/brim although its ABI value is larger.
    CHECK(tightened_invalidation_step(STEP_SKIRT_BRIM, STEP_SUPPORT) == STEP_SUPPORT);
    CHECK(tightened_invalidation_step(STEP_SUPPORT, STEP_SKIRT_BRIM) == STEP_SUPPORT);
}

TEST_CASE("Replay requests resume from the earliest invalid checkpoint",
          "[plugins][step-pipeline][invalidation][checkpoint]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    using Slic3r::Steps::Checkpoint;

    Slic3r::Print print;
    for (const slicing_step_t step : Slic3r::Steps::execution_order())
        print.mark_step_executed(step);
    print.mutable_printing_plan().groups.emplace_back();
    CHECK(print.current_checkpoint() == Checkpoint::GCode);

    // Resuming from the final checkpoint forces nothing.
    print.request_replay_from_worker(Checkpoint::GCode);
    CHECK(print.current_checkpoint() == Checkpoint::GCode);

    // The plan checkpoint forces at least the G-code writer.
    print.request_replay_from_worker(Checkpoint::Plan);
    CHECK(print.current_checkpoint() == Checkpoint::Plan);
    CHECK(print.should_execute_step(STEP_GCODE));
    CHECK_FALSE(print.should_execute_step(STEP_EXTRUSION_SIMPLIFICATION));
    CHECK(print.printing_plan() != nullptr);

    // An earlier invalid checkpoint wins over a later request.
    print.request_replay_from_worker(Checkpoint::Slicing);
    CHECK(print.current_checkpoint() == Checkpoint::Slicing);
    CHECK(print.should_execute_step(STEP_PRE_PERIMETER));
    CHECK_FALSE(print.should_execute_step(STEP_POST_SLICING));
    print.request_replay_from_worker(Checkpoint::Plan);
    CHECK(print.current_checkpoint() == Checkpoint::Slicing);

    // Invalidation preserves borrowed handles until the resume cleanup barrier.
    CHECK(print.printing_plan() != nullptr);
    print.prepare_checkpoint_resume();
    CHECK(print.printing_plan() == nullptr);
}

TEST_CASE("Plan checkpoints fall back to the Print checkpoint without their plan",
          "[plugins][step-pipeline][invalidation][checkpoint]")
{
    using Slic3r::Steps::Checkpoint;
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Slic3r::Print print;
    for (const slicing_step_t step : Slic3r::Steps::execution_order())
        print.mark_step_executed(step);

    // Without a live plan, STEP_GCODE has nothing to write.
    print.request_replay_from_worker(Checkpoint::Plan);
    CHECK(print.current_checkpoint() == Checkpoint::Print);

    // With a plan but without its ordered copy, plan editors cannot restart.
    print.mutable_printing_plan().groups.emplace_back();
    print.request_replay_from_worker(Checkpoint::Ordering);
    CHECK_FALSE(print.has_ordering_checkpoint());
    CHECK(print.current_checkpoint() == Checkpoint::Print);

    // Resuming then requests STEP_ORDERING again and releases the stale plan.
    print.prepare_checkpoint_resume();
    CHECK(print.should_execute_step(STEP_ORDERING));
    CHECK(print.printing_plan() == nullptr);
}

TEST_CASE("Ordering checkpoint survives downstream invalidation only",
          "[plugins][step-pipeline][invalidation][checkpoint]")
{
    using Slic3r::Steps::Checkpoint;
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Slic3r::Print print;
    for (const slicing_step_t step : Slic3r::Steps::execution_order())
        print.mark_step_executed(step);
    print.mutable_printing_plan().groups.emplace_back();
    REQUIRE(print.save_ordering_checkpoint());

    // Plan editors restart from the ordered copy. Ordering itself stays done.
    const size_t ordered_group_count = print.printing_plan()->groups.size();
    print.mutable_printing_plan().groups.emplace_back();
    print.request_replay_from_worker(Slic3r::Steps::checkpoint_before_step(STEP_EXTRUSION_EDIT));
    CHECK(print.current_checkpoint() == Checkpoint::Ordering);
    CHECK(print.has_ordering_checkpoint());
    CHECK_FALSE(print.should_execute_step(STEP_ORDERING));
    CHECK(print.should_execute_step(STEP_WIPETOWER));
    CHECK(print.should_execute_step(STEP_GCODE));

    // A resume restores a fresh clone of the ordered copy.
    print.prepare_checkpoint_resume();
    REQUIRE(print.printing_plan() != nullptr);
    CHECK(print.printing_plan()->groups.size() == ordered_group_count);
    CHECK(print.has_ordering_checkpoint());
    CHECK_FALSE(print.should_execute_step(STEP_ORDERING));
    CHECK(print.should_execute_step(STEP_WIPETOWER));
    CHECK(print.should_execute_step(STEP_GCODE));

    // Invalidating ordering's own inputs makes the copy stale.
    print.request_replay_from_worker(Slic3r::Steps::checkpoint_before_step(STEP_SLICING));
    CHECK_FALSE(print.has_ordering_checkpoint());
    CHECK(print.should_execute_step(STEP_ORDERING));
}

// Change global or object tower XY through normal apply, preserving the caller-owned ordered snapshot; assert that only tower generation and downstream steps replay, with apply/allocation failures visible.
TEST_CASE("Tower position changes replay only from the ordering checkpoint",
          "[plugins][wipe-tower][step-pipeline][invalidation][checkpoint]")
{
    using namespace Slic3r;
    using Steps::Checkpoint;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    DynamicPrintConfig config = Test::PerimeterPluginTests::perimeter_config({});
    config.set_num_extruders(2);
    config.set_deserialize_strict("wipe_tower", "1");
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
    for (const slicing_step_t step : Steps::execution_order())
        print.mark_step_executed(step);
    print.mutable_printing_plan().groups.emplace_back();
    REQUIRE(print.save_ordering_checkpoint());
    REQUIRE(print.current_checkpoint() == Checkpoint::GCode);

    SECTION("global X") {
        config.set_deserialize_strict("wipe_tower_x", "210");
    }
    SECTION("object X") {
        model.objects()[0].config.set_deserialize_strict("wipe_tower_object_x", "70");
    }
    CHECK(print.apply(model, config) == Print::APPLY_STATUS_INVALIDATED);
    CHECK(print.current_checkpoint() == Checkpoint::Ordering);
    CHECK(print.has_ordering_checkpoint());
    for (const slicing_step_t step : Steps::execution_order()) {
        CAPTURE(step);
        // The export is queued by an export request only, never by a replay.
        CHECK(print.should_execute_step(step) ==
              (step != STEP_EXPORT && !Steps::executes_before(step, STEP_WIPETOWER)));
    }
    print.prepare_checkpoint_resume();
    CHECK(print.current_checkpoint() == Checkpoint::Ordering);
    CHECK_FALSE(print.should_execute_step(STEP_ORDERING));
    CHECK_FALSE(print.should_execute_step(STEP_SKIRT_BRIM));
    CHECK(print.should_execute_step(STEP_WIPETOWER));
    CHECK(print.apply(model, config) == Print::APPLY_STATUS_UNCHANGED);
}

TEST_CASE("Disabling the ordering checkpoint releases its copy",
          "[plugins][step-pipeline][invalidation][checkpoint]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Slic3r::Print print;
    print.mutable_printing_plan().groups.emplace_back();
    REQUIRE(print.save_ordering_checkpoint());
    REQUIRE(print.has_ordering_checkpoint());

    print.set_keep_ordering_checkpoint(false);
    CHECK_FALSE(print.has_ordering_checkpoint());
    CHECK_FALSE(print.save_ordering_checkpoint());
}

TEST_CASE("G-code clear resets every published statistic",
          "[plugins][step-pipeline][cleanup][gcode]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Slic3r::Print print;
    Slic3r::PrintStatistics &statistics = print.print_statistics();
    statistics.estimated_print_time[0] = 12.5;
    statistics.estimated_print_time_str[0] = "12s";
    statistics.color_extruderid_to_used_filament.emplace_back(0, 42.0);
    statistics.color_extruderid_to_used_weight.emplace_back(0, 3.0);
    statistics._layer_area_stats.emplace_back(1, 4.0f);
    statistics.total_toolchanges = 3;
    statistics.object_layer_count = 5;
    statistics.total_layer_count = 6;
    statistics.has_support = true;
    statistics.statistics_available = true;
    statistics.is_computing_gcode = true;
    print.set_gcode_reader_source_path("old.gcode");

    Slic3r::Steps::StepGenerateGcode::clear(print);

    CHECK(statistics.estimated_print_time.empty());
    CHECK(statistics.estimated_print_time_str.empty());
    CHECK(statistics.color_extruderid_to_used_filament.empty());
    CHECK(statistics.color_extruderid_to_used_weight.empty());
    CHECK(statistics._layer_area_stats.empty());
    CHECK(statistics.total_toolchanges == 0);
    CHECK(statistics.object_layer_count == 0);
    CHECK(statistics.total_layer_count == 0);
    CHECK_FALSE(statistics.has_support);
    CHECK_FALSE(statistics.statistics_available.load());
    CHECK_FALSE(statistics.is_computing_gcode.load());
    CHECK(print.gcode_reader_source_path().empty());
}

namespace {

size_t count_main_region_islands(const Slic3r::Print &print)
{
    size_t count = 0;
    for (const Slic3r::PrintObject &object : print.objects())
        for (const Slic3r::Layer &layer : object.layers())
            for (const Slic3r::LayerSliceIsland &island : layer.islands())
                count += island.regions_islands().size();
    return count;
}

} // namespace

TEST_CASE("Apply destroys invalidated outputs and the next run restores saved checkpoint data",
          "[plugins][checkpoint][invalidation][cleanup]")
{
    using namespace Slic3r;
    using Steps::Checkpoint;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict("gcode_flavor", "marlin");
    config.set_deserialize_strict("gcode_firmware_plugin", "gcode.firmware.marlin2");
    LifecyclePrint print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
    const boost::filesystem::path path = Test::GCode::temporary_gcode_path();
    struct OutputCleanup {
        boost::filesystem::path path;
        ~OutputCleanup() { Test::GCode::remove_output_pair(path); }
    } output_cleanup{path};

    // A complete run publishes every checkpoint, including the Ordering copy.
    Steps::StepPipeline::run_slice(Orchestrator::instance(), print);
    Steps::StepPipeline::run_gcode(Orchestrator::instance(), print, path.string());
    REQUIRE(print.current_checkpoint() == Checkpoint::GCode);
    REQUIRE(print.has_ordering_checkpoint());
    REQUIRE(print.printing_plan() != nullptr);
    REQUIRE(count_main_region_islands(print) > 0);
    const Layer *source_layer = &print.object(0).layer(0);
    const ExPolygons raw_slices = source_layer->region(0).get_raw_slices();
    // A record created after PostSlicing is absent from the Slicing snapshot.
    print.records().get_or_add("test.downstream");

    const auto apply_option = [&](const char *key, const char *value) {
        DynamicPrintConfig changed = print.full_print_config();
        changed.set_deserialize_strict(key, value);
        return print.apply(model, changed);
    };

    SECTION("Slicing destroys downstream outputs at apply and restores records at run") {
        CHECK(apply_option("perimeters", "4") == Print::APPLY_STATUS_INVALIDATED);
        REQUIRE(print.current_checkpoint() == Checkpoint::Slicing);
        CHECK(&print.object(0).layer(0) == source_layer);
        CHECK(source_layer->region(0).get_raw_slices() == raw_slices);
        CHECK(count_main_region_islands(print) == 0);
        CHECK(print.object(0).auxiliary_layers().empty());
        CHECK(print.auxiliary_object() == nullptr);
        CHECK(print.printing_plan() == nullptr);
        CHECK_FALSE(print.has_ordering_checkpoint());
        CHECK(print.gcode_reader_source_path().empty());
        // Restoration waits for the run, which knows the final checkpoint.
        CHECK(print.records().find("test.downstream") != nullptr);

        Steps::StepPipeline::run_slice(Orchestrator::instance(), print);
        CHECK(print.records().find("test.downstream") == nullptr);
        CHECK(&print.object(0).layer(0) == source_layer);
        CHECK(count_main_region_islands(print) > 0);
    }
    SECTION("None destroys the layers and the profile at apply") {
        CHECK(apply_option("layer_height", "0.3") == Print::APPLY_STATUS_INVALIDATED);
        REQUIRE(print.current_checkpoint() == Checkpoint::None);
        CHECK(print.object(0).layer_count() == 0);
        CHECK(print.object(0).layer_profile().empty());
        CHECK(print.printing_plan() == nullptr);

        Steps::StepPipeline::run_slice(Orchestrator::instance(), print);
        CHECK(print.object(0).layer_count() > 0);
        CHECK(count_main_region_islands(print) > 0);
    }
    SECTION("Ordering releases the live plan at apply and restores the copy at run") {
        CHECK(apply_option("travel_speed", "187") == Print::APPLY_STATUS_INVALIDATED);
        REQUIRE(print.current_checkpoint() == Checkpoint::Ordering);
        CHECK(print.printing_plan() == nullptr);
        CHECK(print.has_ordering_checkpoint());
        CHECK(count_main_region_islands(print) > 0);

        Steps::StepPipeline::run_gcode(Orchestrator::instance(), print, path.string());
        CHECK(print.printing_plan() != nullptr);
        CHECK(print.current_checkpoint() == Checkpoint::GCode);
    }
    SECTION("A deeper second invalidation destroys only the remaining outputs") {
        apply_option("travel_speed", "187");
        REQUIRE(print.current_checkpoint() == Checkpoint::Ordering);
        CHECK(count_main_region_islands(print) > 0);
        apply_option("perimeters", "4");
        REQUIRE(print.current_checkpoint() == Checkpoint::Slicing);
        CHECK(count_main_region_islands(print) == 0);
        CHECK_FALSE(print.has_ordering_checkpoint());
        CHECK(&print.object(0).layer(0) == source_layer);
    }
    SECTION("A failed plugin clear destroys nothing until the retry succeeds") {
        LifecycleClearPlugin &probe = prepare_lifecycle_probe(print);
        const Printing::PrintingPlan *plan = print.printing_plan();
        probe.fail_clear = true;
        CHECK_THROWS_AS(apply_option("perimeters", "4"), PluginCleanupError);
        CHECK(print.printing_plan() == plan);
        CHECK(count_main_region_islands(print) > 0);
        CHECK(print.plugin_cleanup_participants().size() == 1);

        probe.fail_clear = false;
        CHECK(print.apply(model, print.full_print_config()) == Print::APPLY_STATUS_INVALIDATED);
        CHECK(print.printing_plan() == nullptr);
        CHECK(count_main_region_islands(print) == 0);
        CHECK(print.plugin_cleanup_participants().empty());
    }
    SECTION("A clear failing after an earlier successful clear defers destruction to the retry") {
        // One participant per invalidation: travel_speed clears the editor, perimeters the perimeter step.
        prepare_lifecycle_probe(print, STEP_PERIMETER);
        LifecycleClearPlugin &probe = prepare_lifecycle_probe(print);
        const Printing::PrintingPlan *plan = print.printing_plan();
        probe.fail_clear = true;
        probe.fail_after = 1;
        DynamicPrintConfig changed = print.full_print_config();
        changed.set_deserialize_strict("travel_speed", "187");
        changed.set_deserialize_strict("perimeters", "4");
        CHECK_THROWS_AS(print.apply(model, changed), PluginCleanupError);
        CHECK(probe.clears == 2);
        REQUIRE(print.current_checkpoint() == Checkpoint::Slicing);
        CHECK(print.printing_plan() == plan);
        CHECK(count_main_region_islands(print) > 0);
        CHECK(print.plugin_cleanup_participants().size() == 1);

        probe.fail_clear = false;
        CHECK(print.apply(model, changed) == Print::APPLY_STATUS_INVALIDATED);
        CHECK(print.printing_plan() == nullptr);
        CHECK(count_main_region_islands(print) == 0);
        CHECK_FALSE(print.has_ordering_checkpoint());
        CHECK(print.plugin_cleanup_participants().empty());

        Steps::StepPipeline::run_slice(Orchestrator::instance(), print);
        CHECK(count_main_region_islands(print) > 0);
        CHECK(print.records().find("test.downstream") == nullptr);
    }
}
