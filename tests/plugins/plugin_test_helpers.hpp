#ifndef test_plugins_plugin_test_helpers_hpp_
#define test_plugins_plugin_test_helpers_hpp_

struct orchestrator_handle;
namespace Slic3r { struct PluginPackageLoadReport; }

namespace Slic3r::Test::Plugins {

/* Implemented by the regional SeamPlacer test fixture and called before config finalization. */
void register_regional_seam_test_provider(orchestrator_handle *orchestrator);
/* Implemented by the G-code Reader tests and registered before initialization. */
void register_gcode_reader_test_providers(orchestrator_handle *orchestrator);
// Register the shared-storage allocation probe before option finalization.
void register_plugin_storage_test_provider(orchestrator_handle *orchestrator);
// Register and initialize the shared plugin runtime and FFF/SLA print config caches once; call before constructing Slic3r::Print or DynamicPrintConfig::full_print_config(). Missing expected plugins log an error and call assert(false), not a return value.
void ensure_plugin_test_runtime_initialized();
bool python_plugin_test_runtime_available();
// Return the owned startup snapshot, unaffected by tests clearing live reports.
const PluginPackageLoadReport &python_invalid_clear_package_report();
/* True when the runtime reported the empty plugin-id option list fixture. */
bool plugin_test_empty_option_list_error_reported();

} // namespace Slic3r::Test::Plugins

#endif // test_plugins_plugin_test_helpers_hpp_
