#ifndef test_plugins_plugin_test_helpers_hpp_
#define test_plugins_plugin_test_helpers_hpp_

#include <set>
#include <string>
#include <utility>

/* Shared provider startup finalizes option definitions once; before that freeze it records
   rejection results for nature values 16 and 0x100, and acceptance results for 5 and 10.
   Call ensure_plugin_test_runtime_initialized() before using the shared runtime.
   Then plugin_test_startup_sets() returns the provider ids, activation flags, selector keys
   and layout fragment ids copied at startup, unaffected by later tests.
   A provider is a registered plugin; a selector is a setting that chooses a provider.
   A layout fragment describes part of the settings UI.
   Example: const auto &sets = Slic3r::Test::Plugins::plugin_test_startup_sets(); */

struct orchestrator_handle;
namespace Slic3r { struct PluginPackageLoadReport; }

namespace Slic3r::Test::Plugins {

// These sets record the providers, selectors and layout fragments present before individual tests run.
struct PluginTestStartupSets
{
    std::set<std::string> provider_ids;
    // Tests use these ids to restore startup activation after other tests change it.
    std::set<std::string> active_provider_ids;
    std::set<std::string> selector_keys;
    std::set<std::pair<std::string, std::string>> ui_fragments;
};

// Initialize the shared runtime if needed and return its unchanged startup sets.
const PluginTestStartupSets &plugin_test_startup_sets();

// C API result recorded for an invalid nature during shared option registration.
extern int plugin_test_invalid_nature_result;
// Registration results for a high unknown bit and an unnamed valid combination.
extern int plugin_test_high_invalid_nature_result;
extern int plugin_test_combined_nature_result;
// Registration result for the unnamed combination queried through definition views.
extern int plugin_test_roundtrip_nature_result;

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
