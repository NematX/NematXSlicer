// Entry point of the plugins_tests executable: <catch_main.hpp> supplies main(), and the
// listener below initializes the shared plugin test runtime before any test case runs.
#include <catch_main.hpp>

#include "plugin_test_helpers.hpp"

namespace {

// Parallel launcher buckets run in separate processes. The shared "Scenario: init_print
// functionality" of tests/data/cpp/test_data.cpp cannot initialize plugin providers, so this
// target-specific listener does so at run start.
class PluginTestRuntimeListener final : public Catch::TestEventListenerBase
{
public:
    using Catch::TestEventListenerBase::TestEventListenerBase;

    // Catch2 run metadata is ignored. This void callback initializes Orchestrator-owned plugin
    // state before cases; registration exceptions propagate and missing plugins log then assert.
    void testRunStarting(const Catch::TestRunInfo &) override
    {
        Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    }
};

CATCH_REGISTER_LISTENER(PluginTestRuntimeListener)

} // namespace
