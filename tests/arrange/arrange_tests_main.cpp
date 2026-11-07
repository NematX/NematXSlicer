#include <catch_main.hpp>

#include <libslic3r/Api/host/Orchestrator.hpp>
#include <libslic3r/Config/FFFPrintConfig.hpp>
#include <libslic3r/Config/PrintConfig.hpp>
#include <libslic3r/Plugins/LayerExtrusionEdit/DefaultFan.hpp>
#include <libslic3r/Plugins/LayerExtrusionEdit/DefaultSpeed.hpp>
#include <libslic3r/Plugins/SkirtBrim/DefaultBrimGenerator.hpp>
#include <libslic3r/SLA/SLAPrintConfig.hpp>

#include <stdexcept>

/* Provide the arrange tests with configuration definitions before Catch runs them.
   Shared INI profiles contain cooling, speed and brim options owned by plugins.
   Define the core settings, register and activate those providers, then initialize
   them before building typed configuration caches and finalizing the definitions.

   Catch invokes testRunStarting(). Its function-local static initializer runs the
   lambda once per process; the configuration calls below belong to that lambda,
   not to a sequence of ordinary direct calls on every test-run notification.

       ArrangeTestRuntime::testRunStarting()
           `-- one-time initializer lambda [executed on the first invocation]
               |-- init_common_params(), init_fff_params(), init_sla_params()
               |-- register_default_fan_plugin(), register_default_speed_plugin()
               |-- register_default_brim_generator_plugin()
               |-- set_plugin_active() [make their options available]
               |-- initialize_plugins() [populate plugin-owned definitions]
               |-- initialize_fff_print_config_cache(), initialize_sla_print_config_cache()
               `-- finalize() [complete the process-wide configuration definitions]
*/
namespace {

struct ArrangeTestRuntime : Catch::TestEventListenerBase {
    using Catch::TestEventListenerBase::TestEventListenerBase;

    // Populate process-wide configuration definitions and caches once before profile-loading tests run.
    void testRunStarting(const Catch::TestRunInfo &) override;
};

void ArrangeTestRuntime::testRunStarting(const Catch::TestRunInfo &)
{
    using namespace Slic3r;
    static const bool initialized = []() {
        PrintConfigDef::instance_mutable().init_common_params();
        init_fff_params(PrintConfigDef::instance_mutable());
        init_sla_params(PrintConfigDef::instance_mutable());

        Orchestrator::PluginRegistrationScope builtin_scope =
            Orchestrator::instance().plugin_registration_scope({}, false);
        slic3r_api::LayerExtrusionEdit::DefaultFanPlugin::register_default_fan_plugin(
            Orchestrator::instance().handle());
        slic3r_api::LayerExtrusionEdit::DefaultSpeedPlugin::register_default_speed_plugin(
            Orchestrator::instance().handle());
        slic3r_api::SkirtBrim::DefaultBrimGeneratorPlugin::register_default_brim_generator_plugin(
            Orchestrator::instance().handle());
        if (!Orchestrator::instance().set_plugin_active("layer_extrusion_edit.fan.default", true))
            throw std::runtime_error("Cannot activate the default fan plugin for arrange configuration loading.");
        if (!Orchestrator::instance().set_plugin_active("layer_extrusion_edit.speed.default", true))
            throw std::runtime_error("Cannot activate the default speed plugin for arrange configuration loading.");
        if (!Orchestrator::instance().set_plugin_active("skirt_brim.brim.default", true))
            throw std::runtime_error("Cannot activate the default brim plugin for arrange configuration loading.");
        Orchestrator::instance().initialize_plugins();
        initialize_fff_print_config_cache();
        initialize_sla_print_config_cache();
        PrintConfigDef::instance_mutable().finalize();
        return true;
    }();
    (void)initialized;
}

CATCH_REGISTER_LISTENER(ArrangeTestRuntime)

} // namespace
