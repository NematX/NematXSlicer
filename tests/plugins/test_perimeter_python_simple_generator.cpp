#include <catch2/catch.hpp>

#include "perimeter_test_helpers.hpp"
#include "plugin_test_helpers.hpp"

#include <atomic>
#include <stdexcept>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_perimeter.h"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"

#ifdef SLIC3R_TEST_PYTHON_PLUGINS

namespace {
using namespace Slic3r;
using namespace Slic3r::Test::PerimeterPluginTests;

constexpr const char *CANCEL_BEFORE_REMOVE_GAP_FILL = "tests.perimeter.cancel_before_remove_gap_fill";
constexpr const char *BOUNDARY_EXCEPTION_MODULE = "tests.perimeter.boundary_exception_module";
const char *k_no_dependencies[] = { nullptr };

// Test-only perimeter module that requests cancellation immediately before the
// production RemoveGapFillOnOverhangs module runs its after() callback.
class CancelBeforeRemoveGapFill : public slic3r_api::PluginBase
{
public:
    static CancelBeforeRemoveGapFill &instance(orchestrator_handle *orchestrator)
    {
        static CancelBeforeRemoveGapFill plugin(orchestrator);
        return plugin;
    }

    void reset() { m_after_calls.store(0, std::memory_order_relaxed); }
    uint32_t after_calls() const { return m_after_calls.load(std::memory_order_relaxed); }

private:
    explicit CancelBeforeRemoveGapFill(orchestrator_handle *orchestrator) : PluginBase(orchestrator) {}

    const char *id_impl() const noexcept override { return CANCEL_BEFORE_REMOVE_GAP_FILL; }
    const char *name_impl() const noexcept override { return "Cancel before remove gap fill"; }
    const char *description_impl() const noexcept override { return "Requests cancellation from module_after()."; }
    const char *exclusive_group_impl() const noexcept override { return ""; }
    slicing_step_t step_impl() const noexcept override { return PERIMETER_GENERATION_MODULE; }
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }
    int32_t priority_impl() const noexcept override { return 0; }

    void run_impl(const plugin_run_context *run_ctx) const override
    {
        run_ctx_perimeter_generation_module *context = plugin_ctx_as_perimeter_generation_module(run_ctx);
        if (context == nullptr)
            return;
        context->module.ctx = const_cast<CancelBeforeRemoveGapFill *>(this);
        context->module.vt = &module_vtable();
    }

    static void module_after(void *plugin_context,
                             void *,
                             perimeter_generation_context *,
                             perimeter_node *)
    {
        CancelBeforeRemoveGapFill &plugin = *static_cast<CancelBeforeRemoveGapFill *>(plugin_context);
        plugin.m_after_calls.fetch_add(1, std::memory_order_relaxed);
        Orchestrator::instance().request_plugin_cancel();
    }

    static const perimeter_generation_module_vtable &module_vtable()
    {
        static const perimeter_generation_module_vtable vtable = {
            nullptr,
            nullptr,
            &CancelBeforeRemoveGapFill::module_after,
            nullptr
        };
        return vtable;
    }

    mutable std::atomic_uint32_t m_after_calls { 0 };
};

CancelBeforeRemoveGapFill &registered_cancel_before_remove_gap_fill()
{
    Orchestrator &orchestrator = Orchestrator::instance();
    orchestrator_handle *handle = reinterpret_cast<orchestrator_handle *>(&orchestrator);
    CancelBeforeRemoveGapFill &plugin = CancelBeforeRemoveGapFill::instance(handle);
    if (orchestrator.get_plugin(CANCEL_BEFORE_REMOVE_GAP_FILL) == nullptr)
        REQUIRE(orchestrator.register_plugin(plugin.c_instance()));
    return plugin;
}

// Exercises exceptions raised by native module hooks while the host traversal
// is executing as a callback of a Python ctypes call.
class BoundaryExceptionModule : public slic3r_api::PluginBase
{
public:
    enum class Failure
    {
        HostCancellation,
        Ordinary
    };

    static BoundaryExceptionModule &instance(orchestrator_handle *orchestrator)
    {
        static BoundaryExceptionModule plugin(orchestrator);
        return plugin;
    }

    void reset(const Failure failure)
    {
        m_failure = failure;
        m_after_calls.store(0, std::memory_order_relaxed);
        m_end_calls.store(0, std::memory_order_relaxed);
    }

    uint32_t after_calls() const { return m_after_calls.load(std::memory_order_relaxed); }
    uint32_t end_calls() const { return m_end_calls.load(std::memory_order_relaxed); }

private:
    explicit BoundaryExceptionModule(orchestrator_handle *orchestrator) : PluginBase(orchestrator) {}

    const char *id_impl() const noexcept override { return BOUNDARY_EXCEPTION_MODULE; }
    const char *name_impl() const noexcept override { return "Perimeter boundary exception module"; }
    const char *description_impl() const noexcept override { return "Throws from perimeter module hooks."; }
    const char *exclusive_group_impl() const noexcept override { return ""; }
    slicing_step_t step_impl() const noexcept override { return PERIMETER_GENERATION_MODULE; }
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }
    int32_t priority_impl() const noexcept override { return 0; }

    void run_impl(const plugin_run_context *run_ctx) const override
    {
        run_ctx_perimeter_generation_module *context = plugin_ctx_as_perimeter_generation_module(run_ctx);
        if (context == nullptr)
            return;
        context->module.ctx = const_cast<BoundaryExceptionModule *>(this);
        context->module.vt = &module_vtable();
    }

    static void *module_start(void *plugin_context, perimeter_generation_context *)
    {
        return plugin_context;
    }

    static void module_after(void *plugin_context,
                             void *,
                             perimeter_generation_context *,
                             perimeter_node *)
    {
        BoundaryExceptionModule &plugin = *static_cast<BoundaryExceptionModule *>(plugin_context);
        plugin.m_after_calls.fetch_add(1, std::memory_order_relaxed);
        if (plugin.m_failure == Failure::HostCancellation)
            throw CanceledException();
        throw std::runtime_error("ordinary perimeter module failure");
    }

    static void module_end(void *plugin_context, void *, perimeter_generation_context *)
    {
        BoundaryExceptionModule &plugin = *static_cast<BoundaryExceptionModule *>(plugin_context);
        plugin.m_end_calls.fetch_add(1, std::memory_order_relaxed);
        if (plugin.m_failure == Failure::Ordinary)
            throw std::runtime_error("secondary perimeter cleanup failure");
    }

    static const perimeter_generation_module_vtable &module_vtable()
    {
        static const perimeter_generation_module_vtable vtable = {
            &BoundaryExceptionModule::module_start,
            nullptr,
            &BoundaryExceptionModule::module_after,
            &BoundaryExceptionModule::module_end
        };
        return vtable;
    }

    Failure m_failure = Failure::HostCancellation;
    mutable std::atomic_uint32_t m_after_calls { 0 };
    mutable std::atomic_uint32_t m_end_calls { 0 };
};

BoundaryExceptionModule &registered_boundary_exception_module()
{
    Orchestrator &orchestrator = Orchestrator::instance();
    orchestrator_handle *handle = reinterpret_cast<orchestrator_handle *>(&orchestrator);
    BoundaryExceptionModule &plugin = BoundaryExceptionModule::instance(handle);
    if (orchestrator.get_plugin(BOUNDARY_EXCEPTION_MODULE) == nullptr)
        REQUIRE(orchestrator.register_plugin(plugin.c_instance()));
    return plugin;
}

bool has_error_message(const PerimeterCancellationCapture &capture, const std::string &expected)
{
    for (const std::string &message : capture.error_messages)
        if (message.find(expected) != std::string::npos)
            return true;
    return false;
}
}

TEST_CASE("Python SimplePerimeterGenerator publishes perimeter and fill output", "[plugins][perimeter][python]")
{
    // Python exercises the same STEP_PERIMETER host loop as the native simple
    // generator. The plugin should generate one external loop and return child
    // fill surfaces through run_region_group().
    REQUIRE(Slic3r::Test::Plugins::python_plugin_test_runtime_available());

    const DynamicPrintConfig config = perimeter_config({});
    const ExPolygon surface = rectangle_expolygon(-10., -10., 10., 10.);

    const PerimeterRunCapture generated =
        run_perimeter_case(config, {PYTHON_SIMPLE_PERIMETER_GENERATOR}, surface, 0);
    REQUIRE(external_perimeter_count(generated) > 0);
    REQUIRE(extrusion_length(generated.external_perimeters) > 0.);
    REQUIRE_FALSE(generated.fill_surfaces.empty());
    REQUIRE_FALSE(generated.fill_no_overlap_surfaces.empty());
    require_simple_generator_first_child_area_partition(generated, surface);
}

TEST_CASE("Python perimeter generation contains native module cancellation at the ctypes boundary",
          "[plugins][perimeter][python][cancellation]")
{
    // The first native module requests cancellation and the production remove-
    // gap-fill module observes it from module_after(). The host must return to
    // Python before rethrowing and must leave all island outputs unpublished.
    REQUIRE(Slic3r::Test::Plugins::python_plugin_test_runtime_available());

    CancelBeforeRemoveGapFill &probe = registered_cancel_before_remove_gap_fill();
    probe.reset();
    const PerimeterCancellationCapture capture = run_perimeter_cancellation_case(
        perimeter_config({{"gap_fill_no_overhang", "1"}}),
        {PYTHON_SIMPLE_PERIMETER_GENERATOR, CANCEL_BEFORE_REMOVE_GAP_FILL, REMOVE_GAP_FILL_ON_OVERHANGS});

    REQUIRE(probe.after_calls() == 1);
    REQUIRE(capture.plugin_cancelled);
    REQUIRE(capture.perimeter_output_count == 0);
    REQUIRE(capture.infill_area_count == 0);
    REQUIRE(capture.infill_free_area_count == 0);
}

TEST_CASE("Python perimeter generation defers host cancellation from a native module",
          "[plugins][perimeter][python][cancellation]")
{
    REQUIRE(Slic3r::Test::Plugins::python_plugin_test_runtime_available());

    BoundaryExceptionModule &probe = registered_boundary_exception_module();
    probe.reset(BoundaryExceptionModule::Failure::HostCancellation);
    const PerimeterCancellationCapture capture = run_perimeter_cancellation_case(
        perimeter_config({}),
        {PYTHON_SIMPLE_PERIMETER_GENERATOR, BOUNDARY_EXCEPTION_MODULE});

    REQUIRE(probe.after_calls() == 1);
    REQUIRE(probe.end_calls() == 1);
    REQUIRE(capture.host_cancelled);
    REQUIRE_FALSE(capture.runtime_error);
    REQUIRE(capture.perimeter_output_count == 0);
    REQUIRE(capture.infill_area_count == 0);
    REQUIRE(capture.infill_free_area_count == 0);
}

TEST_CASE("Python perimeter generation reports native module failures without publishing",
          "[plugins][perimeter][python][failure]")
{
    REQUIRE(Slic3r::Test::Plugins::python_plugin_test_runtime_available());

    BoundaryExceptionModule &probe = registered_boundary_exception_module();
    probe.reset(BoundaryExceptionModule::Failure::Ordinary);
    const PerimeterCancellationCapture capture = run_perimeter_cancellation_case(
        perimeter_config({}),
        {PYTHON_SIMPLE_PERIMETER_GENERATOR, BOUNDARY_EXCEPTION_MODULE});

    REQUIRE(probe.after_calls() == 1);
    REQUIRE(probe.end_calls() == 1);
    REQUIRE(capture.runtime_error);
    REQUIRE(has_error_message(capture, "ordinary perimeter module failure"));
    REQUIRE_FALSE(has_error_message(capture, "secondary perimeter cleanup failure"));
    REQUIRE(capture.perimeter_output_count == 0);
    REQUIRE(capture.infill_area_count == 0);
    REQUIRE(capture.infill_free_area_count == 0);
}

#endif

namespace {
using namespace Slic3r;
using namespace Slic3r::Test::PerimeterPluginTests;

constexpr const char *FAILING_NATIVE_MODULE = "tests.perimeter.failing_native_module";
const char *k_failing_module_dependencies[] = { nullptr };

// Perimeter module that throws in the chosen phase; the plugin base reports the exception as the
// module's own error, which must be the only error the user gets.
class FailingNativeModule : public slic3r_api::PluginBase
{
public:
    enum class Phase { Setup, SetupRun, Run };

    static FailingNativeModule &instance(orchestrator_handle *orchestrator)
    {
        static FailingNativeModule plugin(orchestrator);
        return plugin;
    }

    void set_phase(const Phase phase) { m_phase = phase; }

private:
    explicit FailingNativeModule(orchestrator_handle *orchestrator) : PluginBase(orchestrator) {}

    const char *id_impl() const noexcept override { return FAILING_NATIVE_MODULE; }
    const char *name_impl() const noexcept override { return "Failing native perimeter module"; }
    const char *description_impl() const noexcept override { return "Fails in the chosen phase."; }
    const char *exclusive_group_impl() const noexcept override { return ""; }
    slicing_step_t step_impl() const noexcept override { return PERIMETER_GENERATION_MODULE; }
    const char *const *dependencies_impl() const noexcept override { return k_failing_module_dependencies; }
    int32_t priority_impl() const noexcept override { return 0; }

    void setup_impl(const plugin_run_context *, uint32_t) const override
    {
        if (m_phase == Phase::Setup)
            throw std::runtime_error("failing module error");
    }

    void setup_run_impl(const plugin_run_context *) const override
    {
        if (m_phase == Phase::SetupRun)
            throw std::runtime_error("failing module error");
    }

    void run_impl(const plugin_run_context *) const override
    {
        if (m_phase == Phase::Run)
            throw std::runtime_error("failing module error");
    }

    Phase m_phase = Phase::Setup;
};

} // namespace

TEST_CASE("A failing perimeter module reports only its own error",
          "[plugins][perimeter][module-failure]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    FailingNativeModule &module = FailingNativeModule::instance(reinterpret_cast<orchestrator_handle *>(&orchestrator));
    if (orchestrator.get_plugin(FAILING_NATIVE_MODULE) == nullptr)
        REQUIRE(orchestrator.register_plugin(module.c_instance()));

    const FailingNativeModule::Phase phase = GENERATE(FailingNativeModule::Phase::Setup,
                                                      FailingNativeModule::Phase::SetupRun,
                                                      FailingNativeModule::Phase::Run);
    CAPTURE(int(phase));
    module.set_phase(phase);
    const PerimeterCancellationCapture capture =
        run_perimeter_cancellation_case(perimeter_config({}), {SIMPLE_PERIMETER_GENERATOR, FAILING_NATIVE_MODULE});

    // The step stops, nothing is published, and the queue holds the module's error alone: the generator
    // adds no report about the stop.
    CHECK(capture.runtime_error);
    CAPTURE(capture.runtime_error_message);
    CHECK(capture.runtime_error_message.find("stopped during ") != std::string::npos);
    CHECK(capture.runtime_error_message.find("of plugin '" + std::string(SIMPLE_PERIMETER_GENERATOR) + "' after plugin '" +
                                              FAILING_NATIVE_MODULE + "' reported an error.") != std::string::npos);
    CHECK(capture.perimeter_output_count == 0);
    REQUIRE(capture.error_messages.size() == 1);
    CHECK(capture.error_messages.front().find("failing module error") != std::string::npos);
}
