#include <catch2/catch.hpp>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "plugin_test_helpers.hpp"

#include "libslic3r/Api/host/ApiHostUtils.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/c/slic3r_gcode_reader.h"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/cpp/OrchestratorViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Config/ConfigDef.hpp"
#include "libslic3r_version.h"
#include "libslic3r/Print.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"
#include "libslic3r/Steps/StepRunner.hpp"
#include "libslic3r/Utils.hpp"

namespace {

using namespace Slic3r;

constexpr const char *ServiceStepName = "slic3r.test.generic_service";
constexpr const char *ServiceGroup = "slic3r.test.generic_service.provider";

struct TestPayload
{
    uint32_t expected_index = 0;
    bool setup_run_seen = false;
    bool run_seen = false;
};

enum class CallbackAction
{
    None,
    Cancel,
    ReportError
};

struct ServicePluginState
{
    const char *id = nullptr;
    std::vector<const char *> dependencies;
    slicing_step_t step = STEP_NONE;
    int32_t priority = 0;
    CallbackAction setup_action = CallbackAction::None;
    CallbackAction setup_run_action = CallbackAction::None;
    CallbackAction run_action = CallbackAction::None;
    bool allow_null_payload = false;
    uint32_t expected_run_count = 0;
    std::atomic<uint32_t> setup_run_count { 0 };
    std::atomic<uint32_t> run_count { 0 };
    std::atomic<bool> barrier_respected { true };
    std::atomic<bool> callback_contract_respected { true };
    std::atomic<bool> action_performed { false };
    storage_handle *storage = nullptr;
};

ServicePluginState g_service_low;
ServicePluginState g_service_high;
ServicePluginState g_private_numeric;
ServicePluginState g_private_numeric_alt;

struct ProgressCapture
{
    std::vector<double> values;
    std::vector<std::string> messages;
};

// Store progress reports in a test-owned object passed through the otherwise
// opaque host context pointer.
void capture_progress(plugin_host_context *host_context, double progress, const char *message)
{
    ProgressCapture &capture = *reinterpret_cast<ProgressCapture *>(host_context);
    capture.values.push_back(progress);
    capture.messages.emplace_back(message != nullptr ? message : "");
}

void perform_callback_action(ServicePluginState &state,
                             const plugin_run_context *run_context,
                             CallbackAction action)
{
    if (action == CallbackAction::None ||
        state.action_performed.exchange(true, std::memory_order_relaxed))
        return;
    if (action == CallbackAction::Cancel) {
        Orchestrator::instance().request_plugin_cancel();
    } else if (run_context != nullptr && run_context->report_error != nullptr) {
        run_context->report_error(run_context->host_context, "intentional service error");
    }
}

const char *service_id(void *context)
{
    return static_cast<ServicePluginState *>(context)->id;
}

const char *service_name(void *context)
{
    return service_id(context);
}

const char *service_description(void *)
{
    return "Test provider for generic orchestrator execution.";
}

const char *service_group(void *)
{
    return ServiceGroup;
}

const char *service_group_label(void *)
{
    return "Generic service test provider";
}

const char *service_group_tooltip(void *)
{
    return "Choose the provider used by the generic execution API test.";
}

slicing_step_t service_step(void *context)
{
    return static_cast<ServicePluginState *>(context)->step;
}

const_strings_t service_dependencies(void *context)
{
    const ServicePluginState &state = *static_cast<ServicePluginState *>(context);
    return {state.dependencies.data(), static_cast<uint32_t>(state.dependencies.size())};
}

int32_t service_priority(void *context)
{
    return static_cast<ServicePluginState *>(context)->priority;
}

int32_t service_used_config_keys(void *, raw_used_config_key *)
{
    return 0;
}

int32_t service_defined_config_keys(void *, const char **)
{
    return 0;
}

void service_initialize(void *, storage_handle *) {}

void service_setup(void *context, const plugin_run_context *run_context, uint32_t run_count)
{
    ServicePluginState &state = *static_cast<ServicePluginState *>(context);
    state.expected_run_count = run_count;
    state.setup_run_count.store(0, std::memory_order_relaxed);
    state.run_count.store(0, std::memory_order_relaxed);
    state.barrier_respected.store(true, std::memory_order_relaxed);
    state.callback_contract_respected.store(true, std::memory_order_relaxed);
    state.action_performed.store(false, std::memory_order_relaxed);
    state.storage = run_context != nullptr ? run_context->plugin_storage : nullptr;
    if (run_context == nullptr || run_context->data != nullptr || run_context->step != state.step)
        state.callback_contract_respected.store(false, std::memory_order_relaxed);
    perform_callback_action(state, run_context, state.setup_action);
}

void service_setup_run(void *context, const plugin_run_context *run_context)
{
    ServicePluginState &state = *static_cast<ServicePluginState *>(context);
    TestPayload *payload = run_context != nullptr ? static_cast<TestPayload *>(run_context->data) : nullptr;
    if (run_context == nullptr || run_context->host_context == nullptr ||
        (payload == nullptr && !state.allow_null_payload)) {
        state.callback_contract_respected.store(false, std::memory_order_relaxed);
        return;
    }
    const plugin_host_context *host = run_context->host_context;
    if (host->object_count != state.expected_run_count ||
        (payload != nullptr && host->object_idx != payload->expected_index) ||
        run_context->plugin_storage != state.storage)
        state.callback_contract_respected.store(false, std::memory_order_relaxed);
    if (payload != nullptr)
        payload->setup_run_seen = true;
    state.setup_run_count.fetch_add(1, std::memory_order_relaxed);

    perform_callback_action(state, run_context, state.setup_run_action);
}

void service_run(void *context, const plugin_run_context *run_context)
{
    ServicePluginState &state = *static_cast<ServicePluginState *>(context);
    TestPayload *payload = run_context != nullptr ? static_cast<TestPayload *>(run_context->data) : nullptr;
    if (payload == nullptr && !state.allow_null_payload) {
        state.callback_contract_respected.store(false, std::memory_order_relaxed);
        return;
    }
    if (state.setup_run_count.load(std::memory_order_relaxed) != state.expected_run_count)
        state.barrier_respected.store(false, std::memory_order_relaxed);
    if (payload != nullptr && !payload->setup_run_seen)
        state.callback_contract_respected.store(false, std::memory_order_relaxed);
    if (payload != nullptr)
        payload->run_seen = true;
    state.run_count.fetch_add(1, std::memory_order_relaxed);
    perform_callback_action(state, run_context, state.run_action);
}

const plugin_vtable *service_vtable()
{
    static const plugin_vtable table = {
        &service_id,
        &service_name,
        &service_description,
        &service_group,
        &service_group_label,
        &service_group_tooltip,
        &service_step,
        &service_dependencies,
        &service_priority,
        &service_used_config_keys,
        &service_defined_config_keys,
        &service_initialize,
        &service_setup,
        &service_setup_run,
        &service_run
    };
    return &table;
}

void register_service_plugin(Orchestrator &orchestrator, ServicePluginState &state)
{
    if (orchestrator.get_plugin(state.id) != nullptr)
        return;
    plugin_instance instance = {};
    instance.ctx = &state;
    instance.vt = service_vtable();
    REQUIRE(orchestrator.register_plugin(instance));
}

class ClearTrackingPlugin final : public slic3r_api::PluginBase
{
public:
    explicit ClearTrackingPlugin(orchestrator_handle *orchestrator,
                                 const char *id = "slic3r.test.clear_tracking",
                                 slicing_step_t step = STEP_POST_SLICING)
        : PluginBase(orchestrator), m_id(id), m_step(step) {}

    void reset()
    {
        m_setup_count = 0;
        m_clear_count = 0;
        m_setup_print = nullptr;
        m_clear_print = nullptr;
        m_storage_was_valid = false;
        m_fail_next_clear = false;
        m_clear_code = PLUGIN_ERROR_OK;
    }

    void fail_next_clear() { m_fail_next_clear = true; }
    void set_clear_code(int32_t code) { m_clear_code = code; }
    uint32_t setup_count() const { return m_setup_count.load(); }
    uint32_t clear_count() const { return m_clear_count.load(); }
    const print_handle *setup_print() const { return m_setup_print.load(); }
    const print_handle *clear_print() const { return m_clear_print.load(); }
    bool storage_was_valid() const { return m_storage_was_valid.load(); }

private:
    const char *id_impl() const noexcept override { return m_id; }
    const char *name_impl() const noexcept override { return "Clear tracking plugin"; }
    slicing_step_t step_impl() const noexcept override { return m_step; }
    const char *const *dependencies_impl() const noexcept override
    {
        static const char *const dependencies[] = { nullptr };
        return dependencies;
    }
    int32_t priority_impl() const noexcept override { return 0; }
    bool clear_enabled_impl() const noexcept override { return true; }

    void setup_impl(const plugin_run_context *run_ctx, uint32_t) const override
    {
        m_setup_print = run_ctx->print;
        ++m_setup_count;
    }

    int32_t clear_impl(const print_handle *print, storage_handle *storage) const override
    {
        m_clear_print = print;
        m_storage_was_valid = storage != nullptr;
        ++m_clear_count;
        if (m_fail_next_clear.exchange(false))
            throw std::runtime_error("intentional clear failure");
        return m_clear_code;
    }

    void run_impl(const plugin_run_context *) const override {}

    const char *m_id;
    slicing_step_t m_step;
    int32_t m_clear_code = PLUGIN_ERROR_OK;
    mutable std::atomic<bool> m_fail_next_clear { false };
    mutable std::atomic<uint32_t> m_setup_count { 0 };
    mutable std::atomic<uint32_t> m_clear_count { 0 };
    mutable std::atomic<const print_handle *> m_setup_print { nullptr };
    mutable std::atomic<const print_handle *> m_clear_print { nullptr };
    mutable std::atomic<bool> m_storage_was_valid { false };
};

ClearTrackingPlugin &clear_tracking_plugin(Orchestrator &orchestrator)
{
    static ClearTrackingPlugin plugin(reinterpret_cast<orchestrator_handle *>(&orchestrator));
    if (orchestrator.get_plugin("slic3r.test.clear_tracking") == nullptr)
        REQUIRE(orchestrator.register_plugin(plugin.c_instance()));
    return plugin;
}

void ensure_service_plugins_registered()
{
    Orchestrator &orchestrator = Orchestrator::instance();
    const slicing_step_t step = orchestrator.register_step(ServiceStepName, STEP_LAYER_EXTRUSION_EDIT);
    REQUIRE(step >= SLICING_STEP_CUSTOM_BEGIN);
    REQUIRE(step <= SLICING_STEP_CUSTOM_END);

    g_service_low.id = "slic3r.test.generic_service.low";
    g_service_low.step = step;
    g_service_low.priority = -10;
    g_service_high.id = "slic3r.test.generic_service.high";
    g_service_high.step = step;
    g_service_high.priority = 10;
    g_private_numeric.id = "slic3r.test.private_numeric_service";
    g_private_numeric.step = static_cast<slicing_step_t>(12001);
    g_private_numeric.priority = 0;
    g_private_numeric_alt.id = "slic3r.test.private_numeric_service_alt";
    g_private_numeric_alt.step = g_private_numeric.step;
    g_private_numeric_alt.priority = 1;

    register_service_plugin(orchestrator, g_service_low);
    register_service_plugin(orchestrator, g_service_high);
    register_service_plugin(orchestrator, g_private_numeric);
    register_service_plugin(orchestrator, g_private_numeric_alt);
}

class ActivePluginGuard
{
public:
    ActivePluginGuard(Orchestrator &orchestrator, const std::vector<const char *> &plugin_ids)
        : m_orchestrator(orchestrator)
    {
        for (Plugin *plugin : m_orchestrator.active_plugins())
            m_previous.push_back(plugin);
        m_orchestrator.clear_active_plugins();
        for (const char *id : plugin_ids)
            REQUIRE(m_orchestrator.set_plugin_active(id, true));
    }

    ~ActivePluginGuard()
    {
        m_orchestrator.clear_active_plugins();
        for (Plugin *plugin : m_previous)
            m_orchestrator.set_plugin_active(plugin, true);
        m_orchestrator.reset_plugin_cancel();
        g_service_low.setup_action = CallbackAction::None;
        g_service_low.setup_run_action = CallbackAction::None;
        g_service_low.run_action = CallbackAction::None;
        g_service_low.allow_null_payload = false;
        g_service_high.setup_action = CallbackAction::None;
        g_service_high.setup_run_action = CallbackAction::None;
        g_service_high.run_action = CallbackAction::None;
    }

private:
    Orchestrator &m_orchestrator;
    std::vector<Plugin *> m_previous;
};

} // namespace

TEST_CASE("PluginProgress publishes terminal completion without hiding later phases",
          "[plugins][progress]")
{
    ProgressCapture capture;
    plugin_run_context context = {};
    context.host_context = reinterpret_cast<plugin_host_context *>(&capture);
    context.report_progress = &capture_progress;
    slic3r_api::PluginProgress progress;

    SECTION("the last run fills missing work and reports 100 percent") {
        progress.reset(&context, 1, "Work: %u / %u");
        progress.add_max(10);
        progress.increment_by(4);
        progress.finish_run();

        REQUIRE(capture.values.size() == 2);
        CHECK(capture.values[0] == Approx(0.4));
        CHECK(capture.values[1] == Approx(1.0));
        CHECK(capture.messages[1] == "Work: 10 / 10");
        CHECK(progress.done() == 10);
    }

    SECTION("completion waits for every planned run") {
        progress.reset(&context, 2, "Work: %u / %u");
        progress.add_max(10);
        progress.increment_by(4);
        progress.finish_run();
        REQUIRE(capture.values.size() == 1);
        CHECK(capture.values.back() == Approx(0.4));

        progress.finish_run();
        REQUIRE(capture.values.size() == 2);
        CHECK(capture.values.back() == Approx(1.0));
    }

    SECTION("finish does not duplicate an existing terminal report") {
        progress.reset(&context, 1, "Work: %u / %u");
        progress.add_max(10);
        progress.increment_by(10);
        REQUIRE(capture.values.size() == 1);

        progress.finish_run();
        CHECK(capture.values.size() == 1);
    }

    SECTION("the next active phase remains visible") {
        progress.reset(&context, 1, "First: %u / %u");
        progress.add_max(0, 10);
        progress.set_phase_format(1, "Second: %u / %u");
        progress.add_max(1, 5);
        progress.increment_by(0, 4);
        progress.finish_run(0);

        REQUIRE(capture.values.size() == 2);
        CHECK(capture.values.back() == Approx(0.0));
        CHECK(capture.messages.back() == "Second: 0 / 5");
    }
}

TEST_CASE("Missing dependencies block indirect consumers without executing them",
          "[plugins][orchestrator][dependencies]")
{
    Orchestrator &orchestrator = Orchestrator::instance();
    // Separate registered fixtures keep a real missing edge in the metadata.
    // The indirect consumer never names the missing plugin itself.
    static ServicePluginState direct;
    static ServicePluginState indirect;
    direct.id = "slic3r.test.dependency.direct";
    indirect.id = "slic3r.test.dependency.indirect";
    direct.step = indirect.step = static_cast<slicing_step_t>(12002);
    direct.dependencies = {"slic3r.test.dependency.not_registered"};
    indirect.dependencies = {direct.id};
    register_service_plugin(orchestrator, direct);
    register_service_plugin(orchestrator, indirect);
    ActivePluginGuard active(orchestrator, {indirect.id, direct.id});
    const orchestrator_plugin_handle *plugin = orchestrator_find_plugin(
        reinterpret_cast<orchestrator_handle *>(&orchestrator), indirect.id);
    REQUIRE(plugin != nullptr);
    std::vector<std::string> closure;
    std::string error;
    REQUIRE_FALSE(orchestrator.plugin_dependency_closure({indirect.id}, closure, error));
    CHECK(closure.empty());
    CHECK(error.find(direct.id) != std::string::npos);
    CHECK(error.find("not_registered") != std::string::npos);
    orchestrator.block_unsatisfied_plugin_dependencies();
    CHECK(orchestrator.active_plugins().empty());
    REQUIRE(orchestrator.blocked_plugin_activations().size() == 2);
    CHECK(orchestrator.blocked_plugin_activations().at(direct.id).find("not loaded") != std::string::npos);
    CHECK(orchestrator.blocked_plugin_activations().at(indirect.id).find(direct.id) != std::string::npos);
    CHECK(orchestrator_find_plugin(
        reinterpret_cast<orchestrator_handle *>(&orchestrator), indirect.id) == nullptr);
    CHECK(orchestrator_execute_plugin(reinterpret_cast<orchestrator_handle *>(&orchestrator),
        plugin, nullptr, nullptr, 0) == RAW_PLUGIN_EXECUTION_INACTIVE);
    CHECK(indirect.run_count == 0);
}

// Test zero-, four-, exact- and extended-size local structs; compare rejected bytes and returned strings.
TEST_CASE("The orchestrator fills the version information C struct",
          "[plugins][orchestrator][api-c]")
{
    const char *unchanged = "unchanged";
    raw_slic3r_version_info undersized = {};
    undersized.struct_size = 0;
    undersized.application_name = unchanged;
    undersized.generated_header = unchanged;
    unsigned char undersized_before[sizeof(undersized)];
    std::memcpy(undersized_before, &undersized, sizeof(undersized));
    CHECK(orchestrator_get_version_info(&undersized) == 0);
    CHECK(std::memcmp(undersized_before, &undersized, sizeof(undersized)) == 0);

    raw_slic3r_version_info four_byte_size = {};
    four_byte_size.struct_size = static_cast<uint32_t>(sizeof(four_byte_size.struct_size));
    four_byte_size.application_name = unchanged;
    four_byte_size.generated_header = unchanged;
    unsigned char four_byte_size_before[sizeof(four_byte_size)];
    std::memcpy(four_byte_size_before, &four_byte_size, sizeof(four_byte_size));
    CHECK(orchestrator_get_version_info(&four_byte_size) == 0);
    CHECK(std::memcmp(four_byte_size_before, &four_byte_size, sizeof(four_byte_size)) == 0);

    raw_slic3r_version_info missing_generated_header = {};
    missing_generated_header.struct_size = static_cast<uint32_t>(
        offsetof(raw_slic3r_version_info, generated_header));
    missing_generated_header.application_name = unchanged;
    missing_generated_header.generated_header = unchanged;
    unsigned char missing_generated_header_before[sizeof(missing_generated_header)];
    std::memcpy(missing_generated_header_before, &missing_generated_header,
                sizeof(missing_generated_header));
    CHECK(orchestrator_get_version_info(&missing_generated_header) == 0);
    CHECK(std::memcmp(missing_generated_header_before, &missing_generated_header,
                      sizeof(missing_generated_header)) == 0);

    raw_slic3r_version_info info = {};
    info.struct_size = static_cast<uint32_t>(sizeof(info));
    REQUIRE(orchestrator_get_version_info(&info) == 1);
    CHECK(std::string(info.application_name) == SLIC3R_APP_NAME);
    CHECK(std::string(info.application_key) == SLIC3R_APP_KEY);
    CHECK(std::string(info.application_command) == SLIC3R_APP_CMD);
    CHECK(std::string(info.version) == SLIC3R_VERSION);
    CHECK(std::string(info.version_full) == SLIC3R_VERSION_FULL);
    CHECK(std::string(info.build_id) == SLIC3R_BUILD_ID);
    CHECK(std::string(info.gcodeviewer_name) == GCODEVIEWER_APP_NAME);
    CHECK(std::string(info.gcodeviewer_key) == GCODEVIEWER_APP_KEY);
    CHECK(std::string(info.gcodeviewer_command) == GCODEVIEWER_APP_CMD);
    CHECK(std::string(info.gcodeviewer_build_id) == GCODEVIEWER_BUILD_ID);
    CHECK(std::string(info.based_on) == SLIC3R_BASED_ON);
    CHECK(std::string(info.github_url) == SLIC3R_GITHUB);
    CHECK(std::string(info.download_url) == SLIC3R_DOWNLOAD);
    CHECK(std::string(info.documentation_url) == SLIC3R_DOC_URL);
    CHECK(std::string(info.intro_url) == SLIC3R_INTRO);
    CHECK(std::string(info.generated_header) ==
          std::string("generated by ") + SLIC3R_APP_KEY + " " + SLIC3R_VERSION + " on");

    struct VersionInfoWithSuffix {
        raw_slic3r_version_info info;
        uint32_t suffix[2];
    } extended = {};
    extended.info.struct_size = static_cast<uint32_t>(sizeof(extended));
    extended.suffix[0] = 0xA17E5EEDu;
    extended.suffix[1] = 0xC0FFEE42u;
    REQUIRE(orchestrator_get_version_info(&extended.info) == 1);
    CHECK(std::string(extended.info.application_name) == SLIC3R_APP_NAME);
    CHECK(extended.suffix[0] == 0xA17E5EEDu);
    CHECK(extended.suffix[1] == 0xC0FFEE42u);
}

TEST_CASE("The orchestrator registers service steps by stable name",
          "[plugins][orchestrator][service-step]")
{
    ensure_service_plugins_registered();
    Orchestrator &orchestrator = Orchestrator::instance();
    orchestrator_handle *handle = reinterpret_cast<orchestrator_handle *>(&orchestrator);

    const slicing_step_t step = orchestrator_register_step(
        handle, ServiceStepName, STEP_LAYER_EXTRUSION_EDIT);
    CHECK(step == g_service_low.step);
    CHECK(std::string(orchestrator_step_name(handle, step)) == ServiceStepName);
    CHECK(orchestrator_register_step(handle, ServiceStepName, STEP_GCODE) == STEP_NONE);
    CHECK(orchestrator_register_step(handle, nullptr, STEP_GCODE) == STEP_NONE);
    CHECK(orchestrator_register_step(handle, "", STEP_GCODE) == STEP_NONE);
    CHECK(orchestrator_register_step(handle, "slic3r.test.invalid", STEP_NONE) == STEP_NONE);

    const slic3r_api::OrchestratorView view(handle);
    CHECK(view.register_step(ServiceStepName, STEP_LAYER_EXTRUSION_EDIT) == step);
    CHECK(view.step_name(step) == ServiceStepName);

    CHECK(orchestrator.get_plugin(g_private_numeric.id) != nullptr);
    CHECK(orchestrator_step_name(handle, g_private_numeric.step) == nullptr);
}

TEST_CASE("The orchestrator interns typed runtime identities",
          "[plugins][orchestrator][runtime-id]")
{
    Orchestrator &orchestrator = Orchestrator::instance();
    orchestrator_handle *handle = reinterpret_cast<orchestrator_handle *>(&orchestrator);

    CHECK(orchestrator_register_runtime_id(
              handle, GCODE_READER_STATE_ID_TYPE, "slic3r.portable") ==
          GCODE_READER_STATE_ID_PORTABLE);
    CHECK(std::string(orchestrator_runtime_id_name(
              handle, GCODE_READER_STATE_ID_TYPE, GCODE_READER_STATE_ID_PORTABLE)) ==
          "slic3r.portable");

    const orchestrator_runtime_id first = orchestrator_register_runtime_id(
        handle, "test.runtime.alpha", "example.shared");
    const orchestrator_runtime_id second = orchestrator_register_runtime_id(
        handle, "test.runtime.alpha", "example.second");
    const orchestrator_runtime_id independent = orchestrator_register_runtime_id(
        handle, "test.runtime.beta", "example.shared");
    REQUIRE(first >= ORCHESTRATOR_RUNTIME_ID_DYNAMIC_BEGIN);
    CHECK(second == first + 1u);
    CHECK(independent == ORCHESTRATOR_RUNTIME_ID_DYNAMIC_BEGIN);
    CHECK(orchestrator_register_runtime_id(
              handle, "test.runtime.alpha", "example.shared") == first);
    CHECK(std::string(orchestrator_runtime_id_name(
              handle, "test.runtime.alpha", first)) == "example.shared");
    CHECK(std::string(orchestrator_runtime_id_name(
              handle, "test.runtime.beta", independent)) == "example.shared");

    const slic3r_api::OrchestratorView view(handle);
    CHECK(view.register_runtime_id("test.runtime.alpha", "example.second") == second);
    CHECK(view.runtime_id_name("test.runtime.alpha", second) == "example.second");
    CHECK(view.runtime_id_name("test.runtime.alpha", ORCHESTRATOR_RUNTIME_ID_INVALID).empty());

    CHECK(orchestrator_register_runtime_id(nullptr, "test.runtime", "example") ==
          ORCHESTRATOR_RUNTIME_ID_INVALID);
    CHECK(orchestrator_register_runtime_id(handle, nullptr, "example") ==
          ORCHESTRATOR_RUNTIME_ID_INVALID);
    CHECK(orchestrator_register_runtime_id(handle, "", "example") ==
          ORCHESTRATOR_RUNTIME_ID_INVALID);
    CHECK(orchestrator_register_runtime_id(handle, "test.runtime", nullptr) ==
          ORCHESTRATOR_RUNTIME_ID_INVALID);
    CHECK(orchestrator_register_runtime_id(handle, "test.runtime", "") ==
          ORCHESTRATOR_RUNTIME_ID_INVALID);
    CHECK(orchestrator_runtime_id_name(nullptr, "test.runtime.alpha", first) == nullptr);
    CHECK(orchestrator_runtime_id_name(handle, nullptr, first) == nullptr);
    CHECK(orchestrator_runtime_id_name(handle, "", first) == nullptr);
    CHECK(orchestrator_runtime_id_name(handle, "test.runtime.alpha",
                                       ORCHESTRATOR_RUNTIME_ID_INVALID) == nullptr);
}

TEST_CASE("Activation group declarations merge atomically and defer member lookup",
          "[plugins][orchestrator][activation-groups]")
{
    Orchestrator &orchestrator = Orchestrator::instance();
    orchestrator_handle *handle = reinterpret_cast<orchestrator_handle *>(&orchestrator);
    const slic3r_api::OrchestratorView view(handle);
    const char *ab[] = {"group.test.a", "group.test.b"};
    REQUIRE(orchestrator_register_activation_group(handle, "test.union", {ab, 2}) == 1);
    view.register_activation_group("test.union", {"group.test.b", "group.test.c"});
    view.register_activation_group("test.union", {"group.test.d", "group.test.e"});
    view.register_activation_group("test.union", {"group.test.b", "group.test.a"});
    view.register_activation_group("test.union.reverse", {"group.test.e", "group.test.d"});
    view.register_activation_group("test.union.reverse", {"group.test.c", "group.test.b"});
    view.register_activation_group("test.union.reverse", {"group.test.a", "group.test.b"});
    const std::set<std::string> expected {"group.test.a", "group.test.b", "group.test.c", "group.test.d", "group.test.e"};
    CHECK(orchestrator.activation_groups().at("test.union") == expected);
    CHECK(orchestrator.activation_groups().at("test.union.reverse") == expected);
    CHECK(orchestrator.incomplete_activation_groups().count("test.union") == 1);
    CHECK(orchestrator.plugin_activation_errors({}).empty());
    CHECK(orchestrator_register_activation_group(nullptr, "test.invalid", {ab, 2}) == 0);
    CHECK(orchestrator_register_activation_group(handle, nullptr, {ab, 2}) == 0);
    CHECK(orchestrator_register_activation_group(handle, "", {ab, 2}) == 0);
    CHECK(orchestrator_register_activation_group(handle, "test.invalid", {nullptr, 2}) == 0);
    CHECK(orchestrator_register_activation_group(handle, "test.invalid", {ab, 1}) == 0);
    const char *bad[] = {"group.test.a", nullptr};
    CHECK(orchestrator_register_activation_group(handle, "test.invalid", {bad, 2}) == 0);
    CHECK_THROWS_AS(view.register_activation_group("test.union", {"new", "new"}), std::invalid_argument);
    CHECK_THROWS_AS(view.register_activation_group("test.union", {"new", ""}), std::invalid_argument);
    CHECK(orchestrator.activation_groups().at("test.union") == expected);
    CHECK(orchestrator.activation_groups().count("test.invalid") == 0);
    const Orchestrator::ActivationChange refused = orchestrator.propose_plugin_activation({}, {"group.test.a"}, true);
    CHECK_FALSE(refused.error.empty());
    CHECK(refused.error.find("test.union") != std::string::npos);
    CHECK(refused.selected.empty());
    ActivePluginGuard guard(orchestrator, {});
    orchestrator.block_unsatisfied_plugin_dependencies({"group.test.a"});
    CHECK(orchestrator.blocked_plugin_activations().count("group.test.a") == 1);
    CHECK(orchestrator.active_plugins().empty());
    orchestrator.block_unsatisfied_plugin_dependencies();
    CHECK(orchestrator.blocked_plugin_activations().empty());
    CHECK(orchestrator.incomplete_activation_groups().count("test.union") == 1);
}

TEST_CASE("Solidarity and dependencies reach one non-mutating activation proposal",
          "[plugins][orchestrator][activation-groups]")
{
    Orchestrator &orchestrator = Orchestrator::instance();
    const slic3r_api::OrchestratorView view(reinterpret_cast<orchestrator_handle *>(&orchestrator));
    static ServicePluginState states[6];
    const char *ids[] = {"solid.a", "solid.b", "solid.c", "solid.d", "solid.e", "solid.consumer"};
    // Register declarations before their members and overlap two groups.
    view.register_activation_group("test.solid.ab", {ids[0], ids[1]});
    view.register_activation_group("test.solid.bc", {ids[1], ids[2]});
    view.register_activation_group("test.solid.de", {ids[3], ids[4]});
    for (size_t i = 0; i < 6; ++i) {
        states[i].id = ids[i];
        states[i].step = static_cast<slicing_step_t>(12003);
        if (i == 2) states[i].dependencies = {ids[3]};
        if (i == 5) states[i].dependencies = {ids[0]};
        register_service_plugin(orchestrator, states[i]);
    }
    // Another declaration after registration is idempotent.
    view.register_activation_group("test.solid.ab", {ids[1], ids[0]});
    CHECK(orchestrator.incomplete_activation_groups().count("test.solid.ab") == 0);
    ActivePluginGuard guard(orchestrator, {});
    const std::set<std::string> expected {ids[0], ids[1], ids[2], ids[3], ids[4]};
    const Orchestrator::ActivationChange enable = orchestrator.propose_plugin_activation({}, {ids[0]}, true);
    REQUIRE(enable.error.empty());
    CHECK(enable.selected == expected);
    CHECK(enable.solidarity_changes.count(ids[1]) == 1);
    CHECK(enable.groups.size() == 3);
    CHECK(orchestrator.active_plugins().empty());
    const std::vector<std::string> all {ids[0], ids[1], ids[2], ids[3], ids[4], ids[5]};
    const Orchestrator::ActivationChange disable = orchestrator.propose_plugin_activation(all, {ids[1]}, false);
    CHECK(disable.selected == std::set<std::string>{ids[3], ids[4]});
    CHECK(disable.dependency_changes.count(ids[5]) == 1);
    const Orchestrator::ActivationChange disable_dependency = orchestrator.propose_plugin_activation(all, {ids[3]}, false);
    CHECK(disable_dependency.selected.empty());
    // Refusing activation uses the deactivation proposal; nothing runs until
    // the caller explicitly commits the selected IDs after confirmation.
    CHECK(orchestrator.propose_plugin_activation({}, {ids[0]}, false).selected.empty());
    CHECK(orchestrator.propose_plugin_activation({}, {ids[0], ids[1], ids[2], ids[3], ids[4]}, true).solidarity_changes.empty());
    std::string error;
    CHECK(orchestrator.validate_plugin_activation(all, error));
    CHECK_FALSE(orchestrator.validate_plugin_activation({ids[0], ids[1], ids[3], ids[4]}, error));
    for (const char *id : {ids[0], ids[1], ids[3], ids[4], ids[5]})
        REQUIRE(orchestrator.set_plugin_active(id, true));
    orchestrator.block_unsatisfied_plugin_dependencies();
    CHECK(orchestrator.blocked_plugin_activations().size() == 3);
    CHECK(orchestrator.is_plugin_active(ids[3]));
    CHECK(orchestrator.is_plugin_active(ids[4]));
    CHECK_FALSE(orchestrator.is_plugin_active(ids[5]));
}

/*
Exercise service lookup and group selection through the C and C++ host APIs using the high-priority
provider's serialized ID. ActivePluginGuard restores Orchestrator activation, the local config owns
its selector, and missing registrations or selections fail Catch2 assertions.
*/
TEST_CASE("The orchestrator finds and selects generic service plugins",
          "[plugins][orchestrator][service-plugin]")
{
    ensure_service_plugins_registered();
    Orchestrator &orchestrator = Orchestrator::instance();
    orchestrator_handle *handle = reinterpret_cast<orchestrator_handle *>(&orchestrator);
    ActivePluginGuard active(orchestrator, {g_service_low.id, g_service_high.id});

    const orchestrator_plugin_handle *found = orchestrator_find_plugin(handle, g_service_low.id);
    REQUIRE(found != nullptr);
    CHECK(std::string(orchestrator_plugin_id(found)) == g_service_low.id);
    CHECK(orchestrator_plugin_step(found) == g_service_low.step);
    CHECK(orchestrator_find_plugin(handle, "slic3r.test.missing") == nullptr);

    const orchestrator_plugin_handle *fallback = orchestrator_select_plugin(
        handle, nullptr, g_service_low.step, ServiceGroup);
    REQUIRE(fallback != nullptr);
    CHECK(std::string(orchestrator_plugin_id(fallback)) == g_service_low.id);

    const slic3r_api::OrchestratorView view(handle);
    CHECK(view.select_plugin(nullptr, g_service_low.step, ServiceGroup).id() == g_service_low.id);

    DynamicConfig config;
    const std::string selector_key =
        "exclusive_group_slic3r_test_generic_service_slic3r_test_generic_service_provider_plugin";
    config.set_key_value(selector_key, new ConfigOptionString(g_service_high.id));
    const orchestrator_plugin_handle *selected = orchestrator_select_plugin(
        handle, ApiHost::to_config_handle(&config), g_service_low.step, ServiceGroup);
    REQUIRE(selected != nullptr);
    CHECK(std::string(orchestrator_plugin_id(selected)) == g_service_high.id);

    const std::vector<Steps::StepExclusivePluginGroup> groups =
        Steps::active_exclusive_plugin_groups(orchestrator);
    const Steps::StepExclusivePluginGroup *service_group = nullptr;
    for (const Steps::StepExclusivePluginGroup &group : groups)
        if (group.group.step == g_service_low.step && group.group.group_id == ServiceGroup)
            service_group = &group;
    REQUIRE(service_group != nullptr);
    CHECK(service_group->group.option_key_storage == selector_key);
    CHECK(service_group->group.option_def.invalidates_step == STEP_LAYER_EXTRUSION_EDIT);

    {
        ActivePluginGuard private_active(
            orchestrator, {g_private_numeric.id, g_private_numeric_alt.id});
        const std::vector<Steps::StepExclusivePluginGroup> private_groups =
            Steps::active_exclusive_plugin_groups(orchestrator);
        const Steps::StepExclusivePluginGroup *private_group = nullptr;
        for (const Steps::StepExclusivePluginGroup &group : private_groups)
            if (group.group.step == g_private_numeric.step && group.group.group_id == ServiceGroup)
                private_group = &group;
        REQUIRE(private_group != nullptr);
        CHECK(private_group->group.option_def.invalidates_step == STEP_ANY);
    }
}

TEST_CASE("Generic plugin execution preserves its two-phase barrier",
          "[plugins][orchestrator][service-plugin]")
{
    ensure_service_plugins_registered();
    Orchestrator &orchestrator = Orchestrator::instance();
    orchestrator.reset_plugin_cancel();
    ActivePluginGuard active(orchestrator, {g_service_low.id});
    slic3r_api::OrchestratorView view(reinterpret_cast<orchestrator_handle *>(&orchestrator));
    const slic3r_api::PluginView plugin = view.find_plugin(g_service_low.id);
    REQUIRE(plugin.valid());

    std::vector<TestPayload> payloads(8);
    for (uint32_t idx = 0; idx < payloads.size(); ++idx)
        payloads[idx].expected_index = idx;

    CHECK(view.execute(plugin, nullptr, payloads) == RAW_PLUGIN_EXECUTION_SUCCESS);
    CHECK(g_service_low.setup_run_count.load(std::memory_order_relaxed) == payloads.size());
    CHECK(g_service_low.run_count.load(std::memory_order_relaxed) == payloads.size());
    CHECK(g_service_low.barrier_respected.load(std::memory_order_relaxed));
    CHECK(g_service_low.callback_contract_respected.load(std::memory_order_relaxed));
    CHECK(g_service_low.storage != nullptr);
    for (const TestPayload &payload : payloads) {
        CHECK(payload.setup_run_seen);
        CHECK(payload.run_seen);
    }

    TestPayload single;
    CHECK(view.execute(plugin, nullptr, single) == RAW_PLUGIN_EXECUTION_SUCCESS);
    CHECK(single.setup_run_seen);
    CHECK(single.run_seen);

    g_service_low.allow_null_payload = true;
    const std::vector<raw_plugin_run_payload> null_payloads(1, raw_plugin_run_payload { nullptr });
    CHECK(view.execute(plugin, nullptr, null_payloads) == RAW_PLUGIN_EXECUTION_SUCCESS);
    CHECK(g_service_low.callback_contract_respected.load(std::memory_order_relaxed));
    g_service_low.allow_null_payload = false;

    CHECK(view.execute_empty(plugin) == RAW_PLUGIN_EXECUTION_SUCCESS);
    CHECK(g_service_low.expected_run_count == 0);
}

TEST_CASE("Clear callbacks track Print state and checkpoint boundaries",
          "[plugins][orchestrator][clear][checkpoint]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    ClearTrackingPlugin &probe = clear_tracking_plugin(orchestrator);
    ActivePluginGuard active(orchestrator, {"slic3r.test.clear_tracking"});
    Plugin *plugin = orchestrator.get_plugin("slic3r.test.clear_tracking");
    REQUIRE(plugin != nullptr);
    probe.reset();

    Print print;
    Print other_print;
    CHECK(orchestrator.execute_plugin(*plugin, &print, nullptr, 0, true) ==
          RAW_PLUGIN_EXECUTION_SUCCESS);
    CHECK(probe.setup_count() == 1);
    CHECK(probe.setup_print() == reinterpret_cast<const print_handle *>(&print));
    CHECK(print.plugin_cleanup_participants().size() == 1);
    CHECK(other_print.plugin_cleanup_participants().empty());

    CHECK(orchestrator.execute_plugin(*plugin, nullptr, nullptr, 0, true) ==
          RAW_PLUGIN_EXECUTION_INVALID_ARGUMENT);
    CHECK(probe.setup_count() == 1);

    orchestrator.clear_plugin_participants(other_print, STEP_LAYER_HEIGHT);
    CHECK(probe.clear_count() == 0);

    // A later invalid step leaves the plugin's earlier state reusable.
    orchestrator.clear_plugin_participants(print, STEP_PRE_PERIMETER);
    CHECK(probe.clear_count() == 0);
    CHECK(print.plugin_cleanup_participants().size() == 1);

    orchestrator.clear_plugin_participants(print, STEP_POST_SLICING);
    CHECK(probe.clear_count() == 1);
    CHECK(probe.clear_print() == reinterpret_cast<const print_handle *>(&print));
    CHECK(probe.storage_was_valid());
    CHECK(print.plugin_cleanup_participants().empty());
}

TEST_CASE("Clear failures retain their ledger entry and do not stop remaining callbacks",
          "[plugins][orchestrator][clear]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    ClearTrackingPlugin &failing = clear_tracking_plugin(orchestrator);
    static ClearTrackingPlugin remaining(orchestrator.handle(), "slic3r.test.clear_remaining");
    if (orchestrator.get_plugin("slic3r.test.clear_remaining") == nullptr)
        REQUIRE(orchestrator.register_plugin(remaining.c_instance()));
    ActivePluginGuard active(orchestrator, {"slic3r.test.clear_tracking", "slic3r.test.clear_remaining"});
    orchestrator.reset_plugin_cancel();
    failing.reset();
    remaining.reset();
    Plugin *failed_plugin = orchestrator.get_plugin("slic3r.test.clear_tracking");
    Plugin *remaining_plugin = orchestrator.get_plugin("slic3r.test.clear_remaining");
    REQUIRE(failed_plugin != nullptr);
    REQUIRE(remaining_plugin != nullptr);
    Print print;
    REQUIRE(orchestrator.execute_plugin(*remaining_plugin, &print, nullptr, 0, true) ==
            RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(orchestrator.execute_plugin(*failed_plugin, &print, nullptr, 0, true) ==
            RAW_PLUGIN_EXECUTION_SUCCESS);
    failing.fail_next_clear();
    orchestrator.consume_plugin_messages();

    CHECK_FALSE(orchestrator.clear_plugin_participants(print, STEP_POST_SLICING));
    CHECK(failing.clear_count() == 1);
    CHECK(remaining.clear_count() == 1);
    CHECK(failing.clear_print() == reinterpret_cast<const print_handle *>(&print));
    CHECK_FALSE(orchestrator.is_plugin_cancelled());
    CHECK_FALSE(print.canceled());
    REQUIRE(print.plugin_cleanup_participants().size() == 1);
    CHECK(print.plugin_cleanup_participants().front().plugin == failed_plugin);
    const std::vector<Orchestrator::PluginMessage> messages = orchestrator.consume_plugin_messages();
    REQUIRE(messages.size() == 1);
    CHECK(messages.front().level == Orchestrator::PluginMessageLevel::Error);
    CHECK(messages.front().plugin_id == "slic3r.test.clear_tracking");
    CHECK(messages.front().step == STEP_POST_SLICING);
    CHECK(messages.front().message == "intentional clear failure");

    CHECK(orchestrator.clear_plugin_participants(print, STEP_POST_SLICING));
    CHECK(failing.clear_count() == 2);
    CHECK(remaining.clear_count() == 1);
    CHECK(print.plugin_cleanup_participants().empty());
    orchestrator_plugin_report_error(nullptr, "unattributed error outside clear");
    CHECK(orchestrator.consume_plugin_messages().empty());
    CHECK_FALSE(orchestrator.is_plugin_cancelled());
}

TEST_CASE("Error diagnostics are registered once and resolved by id",
          "[plugins][orchestrator][clear][abi]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();

    // A null orchestrator or an absent/empty message cannot be stored.
    CHECK(orchestrator.store_error_message(nullptr) == PLUGIN_ERROR_GENERAL);
    CHECK(orchestrator.store_error_message("") == PLUGIN_ERROR_GENERAL);

    const int32_t first = orchestrator.store_error_message("first clear failure");
    REQUIRE(first >= PLUGIN_ERROR_FIRST_DYNAMIC);
    CHECK(std::string(orchestrator.error_message(first)) == "first clear failure");
    const int32_t second = orchestrator.store_error_message("second clear failure");
    CHECK(second > first);
    CHECK(std::string(orchestrator.error_message(second)) == "second clear failure");
    // Resolving an id does not consume it: another callback can return it again.
    CHECK(std::string(orchestrator.error_message(first)) == "first clear failure");

    // Long UTF-8 text is kept whole: no fixed buffer and no truncation.
    const std::string long_message = std::string(600, 'x') + "\xc3\xa9" + "tail";
    const int32_t long_id = orchestrator.store_error_message(long_message.c_str());
    CHECK(long_id > second);
    CHECK(std::string(orchestrator.error_message(long_id)) == long_message);

    // Reserved, unknown and negative codes resolve to a generic diagnostic.
    CHECK(std::string(orchestrator.error_message(PLUGIN_ERROR_GENERAL)) ==
          "Plugin reported an error without a diagnostic.");
    CHECK(std::string(orchestrator.error_message(2)) == "Plugin reported an unknown error code.");
    CHECK(std::string(orchestrator.error_message(-37)) == "Plugin reported an unknown error code.");
    CHECK(std::string(orchestrator.error_message(PLUGIN_ERROR_FIRST_DYNAMIC + 100000)) ==
          "Plugin reported an unknown diagnostic id.");
}

TEST_CASE("Host cleanup resolves native clear codes and keeps failed entries",
          "[plugins][orchestrator][clear]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    ClearTrackingPlugin &probe = clear_tracking_plugin(orchestrator);
    ActivePluginGuard active(orchestrator, {"slic3r.test.clear_tracking"});
    Plugin *plugin = orchestrator.get_plugin("slic3r.test.clear_tracking");
    REQUIRE(plugin != nullptr);
    probe.reset();
    Print print;
    REQUIRE(orchestrator.execute_plugin(*plugin, &print, nullptr, 0, true) == RAW_PLUGIN_EXECUTION_SUCCESS);
    const int32_t registered_id = orchestrator.store_error_message("returned clear failure");
    REQUIRE(registered_id >= PLUGIN_ERROR_FIRST_DYNAMIC);
    int32_t clear_code = registered_id;
    std::string expected = "returned clear failure";
    SECTION("registered diagnostic id") {}
    SECTION("reserved generic code") {
        clear_code = PLUGIN_ERROR_GENERAL;
        expected = "Plugin reported an error without a diagnostic.";
    }
    SECTION("unknown dynamic id") {
        clear_code = PLUGIN_ERROR_FIRST_DYNAMIC + 100000;
        expected = "Plugin reported an unknown diagnostic id.";
    }
    SECTION("negative code is invalid") {
        clear_code = -37;
        expected = "Plugin reported an unknown error code.";
    }
    SECTION("success ignores the diagnostic") {
        clear_code = PLUGIN_ERROR_OK;
    }
    probe.set_clear_code(clear_code);
    orchestrator.consume_plugin_messages();
    const bool succeeded = clear_code == PLUGIN_ERROR_OK;
    CHECK(orchestrator.clear_plugin_participants(print, STEP_POST_SLICING) == succeeded);
    CHECK_FALSE(orchestrator.is_plugin_cancelled());
    CHECK_FALSE(print.canceled());
    const std::vector<Orchestrator::PluginMessage> messages = orchestrator.consume_plugin_messages();
    if (succeeded) {
        CHECK(messages.empty());
        CHECK(print.plugin_cleanup_participants().empty());
    } else {
        REQUIRE(messages.size() == 1);
        CHECK(messages.front().plugin_id == plugin->get_id());
        CHECK(messages.front().step == STEP_POST_SLICING);
        CHECK(messages.front().message == expected);
        CHECK(print.plugin_cleanup_participants().size() == 1);
        probe.set_clear_code(PLUGIN_ERROR_OK);
        CHECK(orchestrator.clear_plugin_participants(print, STEP_POST_SLICING));
        CHECK(print.plugin_cleanup_participants().empty());
    }
}

TEST_CASE("Missing clear storage retains its participant until storage is restored",
          "[plugins][orchestrator][clear]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    ClearTrackingPlugin &probe = clear_tracking_plugin(orchestrator);
    static ClearTrackingPlugin remaining(orchestrator.handle(), "slic3r.test.clear_missing_storage_remaining");
    if (orchestrator.get_plugin("slic3r.test.clear_missing_storage_remaining") == nullptr)
        REQUIRE(orchestrator.register_plugin(remaining.c_instance()));
    ActivePluginGuard active(orchestrator, {"slic3r.test.clear_tracking", "slic3r.test.clear_missing_storage_remaining"});
    orchestrator.reset_plugin_cancel();
    probe.reset();
    remaining.reset();
    Plugin *plugin = orchestrator.get_plugin("slic3r.test.clear_tracking");
    Plugin *remaining_plugin = orchestrator.get_plugin("slic3r.test.clear_missing_storage_remaining");
    REQUIRE(plugin != nullptr);
    REQUIRE(remaining_plugin != nullptr);
    Print print;
    REQUIRE(orchestrator.execute_plugin(*remaining_plugin, &print, nullptr, 0, true) ==
            RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(orchestrator.execute_plugin(*plugin, &print, nullptr, 0, true) ==
            RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(probe.setup_count() == 1);
    orchestrator.consume_plugin_messages();

    // Keep storage alive and restore it before Print destruction even if an assertion aborts.
    std::map<Plugin *, PluginStorage>::node_type storage = orchestrator.plugin_storage().extract(plugin);
    const ScopeGuard restore_storage([&]() {
        if (!storage.empty())
            orchestrator.plugin_storage().insert(std::move(storage));
    });
    REQUIRE_FALSE(storage.empty());
    CHECK_FALSE(orchestrator.clear_plugin_participants(print, STEP_POST_SLICING));
    CHECK(orchestrator.plugin_storage().count(plugin) == 0);
    CHECK(probe.clear_count() == 0);
    CHECK(remaining.clear_count() == 1);
    REQUIRE(print.plugin_cleanup_participants().size() == 1);
    CHECK(print.plugin_cleanup_participants().front().plugin == plugin);
    CHECK(print.plugin_cleanup_participants().front().step == STEP_POST_SLICING);
    const std::vector<Orchestrator::PluginMessage> messages = orchestrator.consume_plugin_messages();
    REQUIRE(messages.size() == 1);
    CHECK(messages.front().level == Orchestrator::PluginMessageLevel::Error);
    CHECK(messages.front().plugin_id == "slic3r.test.clear_tracking");
    CHECK(messages.front().step == STEP_POST_SLICING);
    CHECK(messages.front().message == "Cannot clear Print state: plugin storage is missing.");
    CHECK_FALSE(orchestrator.is_plugin_cancelled());
    CHECK_FALSE(print.canceled());

    orchestrator.plugin_storage().insert(std::move(storage));
    CHECK(orchestrator.clear_plugin_participants(print, STEP_POST_SLICING));
    CHECK(probe.clear_count() == 1);
    CHECK(probe.storage_was_valid());
    CHECK(remaining.clear_count() == 1);
    CHECK(print.plugin_cleanup_participants().empty());
    CHECK(orchestrator.consume_plugin_messages().empty());
}

TEST_CASE("Clear callbacks are rejected on unsupported service steps",
          "[plugins][orchestrator][clear]")
{
    Orchestrator &orchestrator = Orchestrator::instance();
    static ClearTrackingPlugin unsupported(orchestrator.handle(), "slic3r.test.clear_unsupported", BRIDGE_DETECTOR);
    CHECK_FALSE(orchestrator.register_plugin(unsupported.c_instance()));
    CHECK(orchestrator.get_plugin("slic3r.test.clear_unsupported") == nullptr);
}

TEST_CASE("Generic plugin execution reports inactive cancellation and plugin errors",
          "[plugins][orchestrator][service-plugin]")
{
    ensure_service_plugins_registered();
    Orchestrator &orchestrator = Orchestrator::instance();
    orchestrator.reset_plugin_cancel();
    orchestrator_handle *handle = reinterpret_cast<orchestrator_handle *>(&orchestrator);
    const orchestrator_plugin_handle *plugin = nullptr;
    {
        ActivePluginGuard active(orchestrator, {g_service_low.id});
        plugin = orchestrator_find_plugin(handle, g_service_low.id);
        REQUIRE(plugin != nullptr);
        CHECK(slic3r_api::OrchestratorView(handle).find_plugin(g_service_low.id).valid());
    }
    const orchestrator_plugin_handle *invalid_plugin =
        reinterpret_cast<const orchestrator_plugin_handle *>(uintptr_t(1));
    CHECK(orchestrator_execute_plugin(handle, invalid_plugin, nullptr, nullptr, 0) ==
          RAW_PLUGIN_EXECUTION_INVALID_ARGUMENT);

    {
        ActivePluginGuard active(orchestrator, {});
        CHECK(orchestrator_find_plugin(handle, g_service_low.id) == nullptr);
        CHECK_FALSE(slic3r_api::OrchestratorView(handle).find_plugin(g_service_low.id).valid());
        // Handles are borrowed snapshots of a lookup. Execution must reject a
        // handle acquired before its provider was deactivated.
        CHECK(orchestrator_execute_plugin(handle, plugin, nullptr, nullptr, 0) ==
              RAW_PLUGIN_EXECUTION_INACTIVE);
    }

    {
        ActivePluginGuard active(orchestrator, {g_service_low.id});
        orchestrator.request_plugin_cancel();
        CHECK(orchestrator_execute_plugin(handle, plugin, nullptr, nullptr, 0) ==
              RAW_PLUGIN_EXECUTION_CANCELLED);
        orchestrator.reset_plugin_cancel();

        CHECK(orchestrator_execute_plugin(handle, plugin, nullptr, nullptr, 1) ==
              RAW_PLUGIN_EXECUTION_INVALID_ARGUMENT);

        TestPayload payload;
        raw_plugin_run_payload raw_payload { &payload };
        g_service_low.setup_run_action = CallbackAction::ReportError;
        CHECK(orchestrator_execute_plugin(handle, plugin, nullptr, &raw_payload, 1) ==
              RAW_PLUGIN_EXECUTION_PLUGIN_ERROR);
        CHECK_FALSE(payload.run_seen);
        g_service_low.setup_run_action = CallbackAction::None;
        orchestrator.reset_plugin_cancel();

        g_service_low.setup_action = CallbackAction::Cancel;
        CHECK(orchestrator_execute_plugin(handle, plugin, nullptr, &raw_payload, 1) ==
              RAW_PLUGIN_EXECUTION_CANCELLED);
        CHECK(g_service_low.setup_run_count.load(std::memory_order_relaxed) == 0);
        g_service_low.setup_action = CallbackAction::None;
        orchestrator.reset_plugin_cancel();

        payload = TestPayload {};
        g_service_low.setup_run_action = CallbackAction::Cancel;
        CHECK(orchestrator_execute_plugin(handle, plugin, nullptr, &raw_payload, 1) ==
              RAW_PLUGIN_EXECUTION_CANCELLED);
        CHECK_FALSE(payload.run_seen);
        g_service_low.setup_run_action = CallbackAction::None;
        orchestrator.reset_plugin_cancel();

        payload = TestPayload {};
        g_service_low.run_action = CallbackAction::Cancel;
        CHECK(orchestrator_execute_plugin(handle, plugin, nullptr, &raw_payload, 1) ==
              RAW_PLUGIN_EXECUTION_CANCELLED);
        CHECK(payload.run_seen);
        g_service_low.run_action = CallbackAction::None;
        orchestrator.reset_plugin_cancel();

        payload = TestPayload {};
        g_service_low.setup_action = CallbackAction::ReportError;
        CHECK(orchestrator_execute_plugin(handle, plugin, nullptr, &raw_payload, 1) ==
              RAW_PLUGIN_EXECUTION_PLUGIN_ERROR);
        CHECK_FALSE(payload.setup_run_seen);
        g_service_low.setup_action = CallbackAction::None;
        orchestrator.reset_plugin_cancel();

        payload = TestPayload {};
        g_service_low.run_action = CallbackAction::ReportError;
        CHECK(orchestrator_execute_plugin(handle, plugin, nullptr, &raw_payload, 1) ==
              RAW_PLUGIN_EXECUTION_PLUGIN_ERROR);
        CHECK(payload.run_seen);
    }
}

TEST_CASE("Print cancellation reaches plugins and stops the step pipeline",
          "[plugins][orchestrator][cancellation][pipeline]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ensure_service_plugins_registered();
    Orchestrator &orchestrator = Orchestrator::instance();
    orchestrator.reset_plugin_cancel();
    ActivePluginGuard active(orchestrator, {g_service_low.id});
    Plugin *plugin = orchestrator.get_plugin(g_service_low.id);
    REQUIRE(plugin != nullptr);

    Print print;
    plugin_host_context host_context =
        orchestrator.prepare_plugin_host_context(plugin->get_step(), plugin, &print);
    plugin_run_context run_context = orchestrator.prepare_plugin_run_context(
        plugin->get_step(), plugin, &host_context);
    REQUIRE(run_context.is_cancelled != nullptr);
    CHECK(run_context.is_cancelled(run_context.host_context) == 0);

    print.cancel();
    CHECK(run_context.is_cancelled(run_context.host_context) == 1);

    TestPayload payload;
    raw_plugin_run_payload raw_payload { &payload };
    CHECK(orchestrator.execute_plugin(
              *plugin, &print, &raw_payload, 1, true) ==
          RAW_PLUGIN_EXECUTION_CANCELLED);
    CHECK_FALSE(payload.setup_run_seen);
    CHECK_FALSE(payload.run_seen);

    const std::atomic_bool execution_error { false };
    CHECK_THROWS_AS(
        Slic3r::Steps::Detail::throw_if_plugin_stopped(
            orchestrator, print, plugin->get_step(), *plugin,
            execution_error, "test"),
        CanceledException);
    CHECK_THROWS_AS(
        Slic3r::Steps::StepPipeline::run_slice(orchestrator, print),
        CanceledException);
}
