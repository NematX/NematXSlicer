#include <catch2/catch.hpp>

#include <algorithm>
#include <mutex>
#include <stdexcept>
#include <vector>

#include "plugin_test_helpers.hpp"
#include "test_data.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_support.h"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Steps/StepGenerateSupport.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"

/* These recording families test STEP_SUPPORT without generating geometry.
   Both are registered before selector settings are finalized, then disabled
   outside this test. Calls are recorded under a mutex because the host can
   process objects in parallel; all calls of one priority finish before the next. */

namespace {
using namespace Slic3r;

struct SupportCall {
    int32_t priority;
    const object_handle *object;
};
std::mutex call_mutex;
std::vector<SupportCall> calls;

class SupportFamilyProbe final : public slic3r_api::PluginBase
{
public:
    // Set the provider's stable id and priority for recording callbacks.
    SupportFamilyProbe(orchestrator_handle *orch, const char *id, int32_t priority)
        : PluginBase(orch), m_id(id), m_priority(priority) {}
    // Return the stable id used to activate this recording family.
    const char *id() const { return m_id; }
private:
    // Expose the provider's id to registration and activation.
    const char *id_impl() const noexcept override { return m_id; }
    // Dispatch this family at STEP_SUPPORT.
    slicing_step_t step_impl() const noexcept override { return STEP_SUPPORT; }
    // This test provider has no plugin dependencies.
    const char *const *dependencies_impl() const noexcept override { return nullptr; }
    // Return the priority recorded by the callback.
    int32_t priority_impl() const noexcept override { return m_priority; }
    // Record the callback's object under a lock; reject missing support payloads.
    void run_impl(const plugin_run_context *context) const override
    {
        const run_ctx_generate_support *payload = plugin_ctx_as_generate_support(context);
        if (payload == nullptr || payload->print == nullptr || payload->object == nullptr)
            throw std::runtime_error("Missing support family object payload");
        const std::lock_guard<std::mutex> lock(call_mutex);
        calls.push_back({m_priority, payload->object});
    }
    const char *m_id;
    int32_t m_priority;
};
} // namespace

namespace Slic3r::Test::Plugins {
// Register the slower family first so the test can distinguish priority order from registration order; registration failures throw.
void register_support_step_test_providers(orchestrator_handle *orchestrator)
{
    static SupportFamilyProbe late(orchestrator, "test.support.family.late", 100);
    static SupportFamilyProbe early(orchestrator, "test.support.family.early", 0);
    Orchestrator &host = *reinterpret_cast<Orchestrator *>(orchestrator);
    for (SupportFamilyProbe *provider : {&late, &early})
        if (!host.register_plugin(provider->c_instance()) || !host.set_plugin_active(provider->id(), true))
            throw std::runtime_error("Cannot register support family test provider");
}
} // namespace Slic3r::Test::Plugins

TEST_CASE("Support families all run once per object in priority order without a host selector", "[plugins][support-step]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    struct ActiveFamilies {
        // Disable the recording families even when an assertion fails.
        ~ActiveFamilies()
        {
            Orchestrator::instance().set_plugin_active("test.support.family.early", false);
            Orchestrator::instance().set_plugin_active("test.support.family.late", false);
        }
    } activation;
    REQUIRE(orchestrator.set_plugin_active("test.support.family.early", true));
    REQUIRE(orchestrator.set_plugin_active("test.support.family.late", true));
    calls.clear();

    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20, Test::TestMesh::cube_2x20x10}, print, model);
    REQUIRE(print.objects().size() == 2);
    CHECK(Steps::get_exclusive_steps().count(STEP_SUPPORT) == 0);
    CHECK(print.full_print_config().option("step_support_plugin") == nullptr);
    CHECK(SLIC3R_PLUGIN_API_STEP_SUPPORT_MINOR == 2u);
    Steps::StepGenerateSupport::run_step(orchestrator, print);
    REQUIRE(calls.size() == 4);
    CHECK(calls[0].priority == 0);
    CHECK(calls[1].priority == 0);
    CHECK(calls[2].priority == 100);
    CHECK(calls[3].priority == 100);
    for (const PrintObject &object : print.objects())
        for (int32_t priority : {0, 100})
            CHECK(std::count_if(calls.begin(), calls.end(), [&object, priority](const SupportCall &call) {
                return call.object == reinterpret_cast<const object_handle *>(&object) && call.priority == priority;
            }) == 1);
}
