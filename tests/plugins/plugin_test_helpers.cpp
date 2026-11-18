/*
This file initializes the plugin runtime shared by plugin tests.
Providers are plugins registered with Orchestrator, the runtime's plugin manager.
Selectors are settings that choose a provider; layout fragments describe pieces of the settings UI.

Initialization registers providers and settings, applies activation choices, then copies
provider ids, selector keys and layout fragment ids before individual tests run.
The copied sets remain unchanged when later tests register or activate providers.
When SLIC3R_TEST_EXTERNAL_PLUGIN_REPOSITORY names a package directory, its packages
are loaded and the providers listed in SLIC3R_TEST_EXTERNAL_PLUGIN_IDS are activated
before the exclusive selectors are built, so a selector can choose them.
The configuration/activation helpers run inside the function-local static
initializer lambda, once per process. The startup query only ensures that
initialization has finished before returning the copied sets.
All three travel providers remain active while the travel selector is created and
option definitions are finalized, preserving its avoidance-provider default.
They are then deactivated before FFF/SLA configuration caches are created, so
unrelated tests can choose editor chains without inserting travel geometry.

Representative direct calls (branches are not a timeline):

    plugin_test_startup_sets()
        `-- ensure_plugin_test_runtime_initialized()

    one-time initialization lambda [not a direct call of the enclosing function]
        |-- init_common_params()
        |-- init_fff_params()
        |-- init_sla_params()
        |-- register_config_introspection_fixture()
        |   `-- register_config_option_or_fail()
        |-- register_plugin_option_list_fixtures()
        |-- activate_plugin_or_fail()
        |-- activate_plugin_or_fail: enable providers for initialization
        |-- initialize_plugins: initialize active providers and their options
        |-- finalize_plugin_option_defs: finish selector definitions
        |-- set_plugin_active: disable travel and other test providers
        |-- initialize_fff_print_config_cache: cache finalized FFF defaults
        `-- initialize_sla_print_config_cache: cache finalized SLA defaults
*/
#include "plugin_test_helpers.hpp"

#include "probe_pattern_plugin.hpp"

#include <cassert>
#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <sstream>

#include <boost/log/trivial.hpp>
#include <boost/log/core.hpp>
#include <boost/log/sinks/sync_frontend.hpp>
#include <boost/log/sinks/text_ostream_backend.hpp>
#include <boost/make_shared.hpp>

#ifdef SLIC3R_TEST_PYTHON_PLUGINS
#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#else
#include <dlfcn.h>
#include <limits.h>
#include <unistd.h>
#endif

#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#endif

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Config/FFFPrintConfig.hpp"
#include "libslic3r/Utils.hpp"
#include "libslic3r/Plugins/GCode/FeatureGCode.hpp"
#include "libslic3r/Plugins/GCode/CustomGCodePerPrintZ.hpp"
#include "libslic3r/Plugins/GCode/Exporter/BinaryGCodeExporter.hpp"
#include "libslic3r/Plugins/GCode/Exporter/BambuGCodeExporter.hpp"
#include "libslic3r/Plugins/GCode/Exporter/DefaultGCodeExporter.hpp"
#include "libslic3r/Plugins/GCode/Firmware/BuiltinGCodeFirmwares.hpp"
#include "libslic3r/Plugins/GCode/Thumbnail/BuiltinGCodeThumbnails.hpp"
#include "libslic3r/Plugins/GCode/PostProcessScripts.hpp"
#include "libslic3r/Plugins/GCode/PrintingPlanFileWriter.hpp"
#include "libslic3r/Plugins/GCode/Reader/DefaultGCodeReader.hpp"
#include "libslic3r/Plugins/GCode/SettingsGCodeScripts.hpp"
#include "libslic3r/Plugins/GuiRulesCore.hpp"
#include "libslic3r/Plugins/Infill/AdaptiveCubic/AdaptiveCubicPattern.hpp"
#include "libslic3r/Plugins/Infill/Lightning/LightningPattern.hpp"
#include "libslic3r/Plugins/Infill/ConcentricPattern.hpp"
#include "libslic3r/Plugins/Infill/DefaultInfillGenerator.hpp"
#include "libslic3r/Plugins/Infill/EnsuringPattern.hpp"
#include "libslic3r/Plugins/Infill/GyroidPattern.hpp"
#include "libslic3r/Plugins/Infill/InternalBridgeAnchors.hpp"
#include "libslic3r/Plugins/Infill/PostFullFillVolume.hpp"
#include "libslic3r/Plugins/Infill/PostInfillGapFill.hpp"
#include "libslic3r/Plugins/Infill/Rectilinear/RectilinearPatterns.hpp"
#include "libslic3r/Plugins/Infill/SimpleInfillPatterns.hpp"
#include "libslic3r/Plugins/Infill/SmoothPattern.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/AdvancedTravel.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/AvoidCrossingPerimetersTravel.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/CreateRetraction.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/CreateRetractionLift.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/CreateRetractionWipe.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/CreateToolChange.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/CreateTransitionScope.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/DefaultAcceleration.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/DefaultFan.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/DefaultLayerEntryState.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/DefaultSpeed.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/DefaultTemperature.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/DynamicFan.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/FanCooling.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/FirstLayerFan.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/DefaultTravel.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/CoolingSlowdown.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/DefaultTravelDeceleration.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/FirstLayerFlow.hpp"
#include "libslic3r/Plugins/Ordering/DefaultOrdering.hpp"
#include "libslic3r/Plugins/PluginLoader.hpp"
#include "libslic3r/Plugins/SeamPlacer/AlignedSeamPlacer.hpp"
#include "libslic3r/Plugins/SeamPlacer/ContiguousSeamPlacer.hpp"
#include "libslic3r/Plugins/SeamPlacer/FixedDirectionSeamPlacer.hpp"
#include "libslic3r/Plugins/SeamPlacer/CostSeamPlacer.hpp"
#include "libslic3r/Plugins/SeamPlacer/RandomSeamPlacer.hpp"
#include "libslic3r/Plugins/SkirtBrim/DefaultBrimSkirtTrim.hpp"
#include "libslic3r/Plugins/SkirtBrim/DefaultBrimGenerator.hpp"
#include "libslic3r/Plugins/SkirtBrim/DefaultSkirtGenerator.hpp"
#include "libslic3r/Plugins/Perimeter/ArachnePerimeterGenerator.hpp"
#include "libslic3r/Plugins/Perimeter/ClassicPerimeterGenerator.hpp"
#include "libslic3r/Plugins/Perimeter/ExtraPerimeterBelowArea.hpp"
#include "libslic3r/Plugins/Perimeter/ExtraPerimeterCount.hpp"
#include "libslic3r/Plugins/Perimeter/ExtraPerimeterOddLayer.hpp"
#include "libslic3r/Plugins/Perimeter/DetectOverhang.hpp"
#include "libslic3r/Plugins/Perimeter/ExtraPerimetersOnOverhangs.hpp"
#include "libslic3r/Plugins/Perimeter/FuzzySkin.hpp"
#include "libslic3r/Plugins/Perimeter/LoopDirection.hpp"
#include "libslic3r/Plugins/Perimeter/MarkFirstLoop.hpp"
#include "libslic3r/Plugins/Perimeter/OnlyOnePerimeterFirstLayer.hpp"
#include "libslic3r/Plugins/Perimeter/OnlyOnePerimeterOnTop.hpp"
#include "libslic3r/Plugins/Perimeter/RemoveGapFillOnOverhangs.hpp"
#include "libslic3r/Plugins/Perimeter/SeparateHoleContour.hpp"
#include "libslic3r/Plugins/Perimeter/SimplePerimeterGenerator.hpp"
#include "libslic3r/Plugins/Slicing/MaxOverhangThreshold.hpp"
#include "libslic3r/Plugins/Slicing/OverhangSlopeFill.hpp"
#include "libslic3r/Plugins/Slicing/SliceVolume.hpp"
#include "libslic3r/Plugins/Slicing/StandardLayerHeightGenerator.hpp"
#include "libslic3r/Plugins/Slicing/VaseMultiIslandConnector.hpp"
#include "libslic3r/Plugins/HoleSizeCompensation.hpp"
#include "libslic3r/Plugins/XYSizeCompensation.hpp"
#include "libslic3r/Plugins/Surface/CleanInfillSurfaces.hpp"
#include "libslic3r/Plugins/Surface/InitialTypedSurfaceBuilder.hpp"
#include "libslic3r/Plugins/Surface/InfillRegionCompatibilitySplitter.hpp"
#include "libslic3r/Plugins/Surface/SolidShells.hpp"
#include "libslic3r/Plugins/Surface/TopSurfaceExpansion.hpp"
#include "libslic3r/Plugins/Surface/BridgePreparation.hpp"
#include "libslic3r/Plugins/Infill/BridgeVolume.hpp"




#include "libslic3r/Plugins/WipeTower/ToolChangeUnretract.hpp"
#include "libslic3r/Plugins/WipeTower/ToolChangeWipe.hpp"
#include "libslic3r/Plugins/WipeTower/WipeTowerManager.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/SLA/SLAPrintConfig.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"
#include "plugins_cpp/FlatAreaLayerHeight/FlatAreaLayerHeight.hpp"
#include "plugins_cpp/DenseInfill/DenseInfill.hpp"
#include "plugins_cpp/ExtraPerimeterOverhangWave/ExtraPerimeterOverhangWave.hpp"
#include "plugins_cpp/NozzleCleaning/NozzleCleaning.hpp"
#include "plugins_cpp/Polyholes/Polyholes.hpp"

namespace Slic3r::Test::Plugins {

#ifdef SLIC3R_TEST_TOP_SURFACE_RETRACTION
// Register the policy before the focused runtime finalizes configuration caches.
extern "C" void nematx_builtin_register_top_surface_retraction(orchestrator_handle *);
#endif
// Borrow immutable startup identity sets after shared initialization; later test registrations are excluded and startup/allocation failures propagate.
const PluginTestStartupSets &plugin_test_startup_sets();
// Complete shared startup and borrow its saved report for the lower or upper signed-32-bit priority error.
// Reports outlive clearing application reports; plugin state is unchanged, and startup failures propagate.
const PluginPackageLoadReport &python_priority_package_report(bool upper);
// Complete shared startup and borrow immutable saved log text for obsolete Python attributes; startup/allocation failures propagate.
const std::string &python_metadata_warnings();
// Register/activate permanent providers once, finalize settings and FFF/SLA config caches, and copy startup identities before tests add registrations.
// The manager owns wrappers borrowing static providers; call before Print or full_print_config().
// Disable regional seam/storage probes and aligned, contiguous, cost and random seam placers; the failed selector probe is disabled during initialization,
// its expected diagnostic is filtered, and the valid selector probe is disabled after finalization.
// Disable all three travel providers after registering their shared selector.
// Missing expected plugins log and assert; registration, Python loading and allocation failures propagate.
void ensure_plugin_test_runtime_initialized();

int plugin_test_invalid_nature_result = OPTION_DEF_ERROR_INTERNAL;
int plugin_test_high_invalid_nature_result = OPTION_DEF_ERROR_INTERNAL;
int plugin_test_combined_nature_result = OPTION_DEF_ERROR_INTERNAL;
int plugin_test_roundtrip_nature_result = OPTION_DEF_ERROR_INTERNAL;
}

namespace {

Slic3r::Test::Plugins::PluginTestStartupSets g_startup_sets;

constexpr char k_exclusive_pruning_failed_plugin_id[] = "slic3r.test.exclusive_pruning.failed";
constexpr char k_exclusive_pruning_valid_plugin_id[] = "slic3r.test.exclusive_pruning.valid";
constexpr char k_exclusive_pruning_group_id[] = "test.exclusive.selector_pruning";
constexpr char k_exclusive_pruning_failed_setting[] = "test.s2.failed_provider_setting";
constexpr char k_exclusive_pruning_valid_setting[] = "test.s2.valid_provider_setting";

/*
Two startup providers exercise post-initialization pruning and selector-driven GUI rules. Their
provider metadata is borrowed from static strings; initialization creates only the valid provider's
own setting, while the lower-priority provider deliberately conflicts with an existing option.
*/
class ExclusivePruningTestPlugin final : public slic3r_api::PluginBase
{
public:
    ExclusivePruningTestPlugin(orchestrator_handle *orchestrator,
                               const char *id,
                               const char *read_key,
                               const char *defined_key,
                               const int32_t priority,
                               const bool fail_initialization) :
        PluginBase(orchestrator),
        m_orchestrator(orchestrator),
        m_id(id),
        m_read_key(read_key),
        m_defined_key(defined_key),
        m_priority(priority),
        m_fail_initialization(fail_initialization)
    {}

private:
    const char *id_impl() const noexcept override { return m_id; }

    const char *exclusive_group_impl() const noexcept override
    {
        return k_exclusive_pruning_group_id;
    }

    slicing_step_t step_impl() const noexcept override { return STEP_POST_SLICING; }

    const char *const *dependencies_impl() const noexcept override
    {
        static const char *const no_dependencies[] = { nullptr };
        return no_dependencies;
    }

    // Return the common execution priority for test alternatives. Nothing is changed or transferred, and the call cannot fail.
    int32_t priority_impl() const noexcept override { return 0; }
    // Return this provider's copied preference without effects or failures.
    int32_t exclusive_group_priority_impl() const noexcept override { return m_priority; }

    // The host first queries the count with a null buffer, then supplies one slot. This callback
    // writes one borrowed key description without changing provider state or failing.
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override
    {
        if (keys != nullptr) {
            keys[0] = raw_used_config_key{
                m_read_key,
                m_fail_initialization ? RAW_CO_FLOAT : RAW_CO_BOOL,
                RAW_CONTAINER_TYPE_PROJECT,
                RAW_PRESET_TYPE_FFF_PRINT};
        }
        return 1;
    }

    // The host uses the same count-then-fill calls. This callback returns zero for the failing
    // provider or one borrowed key string for the valid provider, without changing state or failing.
    int32_t defined_config_keys(const char **keys) const noexcept override
    {
        if (m_defined_key == nullptr)
            return 0;
        if (keys != nullptr)
            keys[0] = m_defined_key;
        return 1;
    }

    // Ignore the unused storage handle and register one option through the borrowed orchestrator.
    // The valid provider adds its bool setting; the other submits a string definition for an
    // existing float key, which the host rejects and uses to disable that provider. The C API
    // copies the definition, produces no direct output, and reports registration failure through
    // the startup diagnostic.
    void inilialize_impl(storage_handle *) const override
    {
        raw_config_option_def definition = raw_config_option_def_init();
        definition.opt_key = m_fail_initialization ? m_read_key : m_defined_key;
        definition.type = m_fail_initialization ? RAW_CO_STRING : RAW_CO_BOOL;
        definition.container_type = RAW_CONTAINER_TYPE_PROJECT;
        definition.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
        definition.printer_technology = RAW_PT_FFF;
        definition.category = RAW_OPTION_CATEGORY_SLICING;
        definition.invalidates_step = STEP_POST_SLICING;
        definition.default_serialized_value = m_fail_initialization ? "invalid" : "0";
        definition.label = m_fail_initialization ? "Conflicting test option" : "Valid provider setting";
        definition.full_label = definition.label;
        definition.mode = RAW_CONFIG_OPTION_MODE_ADV_EXP | RAW_CONFIG_OPTION_MODE_SUSI;
        (void)orchestrator_create_option_def(m_orchestrator, &definition);
    }

    void run_impl(const plugin_run_context *) const override {}

    orchestrator_handle *m_orchestrator;
    const char *m_id;
    const char *m_read_key;
    const char *m_defined_key;
    int32_t m_priority;
    bool m_fail_initialization;
};

// Register copied metadata, graphs and nature examples before the shared catalogue is finalized.
void register_config_introspection_fixture(orchestrator_handle *orchestrator);
void register_config_option_or_fail(orchestrator_handle *orchestrator,
                                    const raw_config_option_def &definition);
// Register choice-list examples before finalization applies stored patches and removes unavailable members.
// The application copies strings without changing slicing data; rejected registrations or patches throw.
void register_plugin_option_list_fixtures(orchestrator_handle *orchestrator);
// Register the selector providers before the shared menu and GUI rules are built.
void register_exclusive_pruning_test_plugins(orchestrator_handle *orchestrator);

// Set while the runtime finalizes plugin option definitions when the empty
// plugin-id list fixture reports its non-fatal error.
bool g_empty_option_list_error_reported = false;

#ifdef SLIC3R_TEST_PYTHON_PLUGINS

bool g_python_plugins_loaded = false;
Slic3r::PluginPackageLoadReport g_python_invalid_clear_package_report;
Slic3r::PluginPackageLoadReport g_python_priority_low_report;
Slic3r::PluginPackageLoadReport g_python_priority_high_report;
std::string g_python_metadata_warnings;

// Load the process-owned Python library and save startup diagnostics and application-log output before tests can clear reports.
// Temporarily admit warnings and restore the log level on failure; missing runtime returns false, and missing reports throw.
bool load_python_plugins_for_tests(orchestrator_handle *orchestrator);

boost::filesystem::path current_executable_dir()
{
#ifdef _WIN32
    std::vector<wchar_t> buffer(MAX_PATH);
    DWORD length = 0;
    for (;;) {
        length = GetModuleFileNameW(NULL, buffer.data(), DWORD(buffer.size()));
        if (length == 0)
            return {};
        if (length < buffer.size() - 1)
            break;
        buffer.resize(buffer.size() * 2);
    }
    return boost::filesystem::path(std::wstring(buffer.data(), length)).parent_path();
#else
    std::vector<char> buffer(PATH_MAX);
    for (;;) {
        const ssize_t length = readlink("/proc/self/exe", buffer.data(), buffer.size());
        if (length < 0)
            return boost::filesystem::current_path();
        if (size_t(length) < buffer.size())
            return boost::filesystem::path(std::string(buffer.data(), size_t(length))).parent_path();
        buffer.resize(buffer.size() * 2);
    }
#endif
}

bool load_python_plugins_for_tests(orchestrator_handle *orchestrator)
{
#ifdef _WIN32
    const boost::filesystem::path loader_path = current_executable_dir() / "plugins" / "python" / "plugin.dll";
    if (!boost::filesystem::exists(loader_path)) {
        BOOST_LOG_TRIVIAL(warning) << "Python plugin loader not found for plugin tests: " << loader_path.string();
        return false;
    }

    static std::vector<HMODULE> loaded_modules;
    HMODULE module = LoadLibraryW(loader_path.wstring().c_str());
    if (module == NULL) {
        BOOST_LOG_TRIVIAL(error) << "Cannot load Python plugin loader '" << loader_path.string()
                                 << "': error " << GetLastError();
        return false;
    }
    loaded_modules.push_back(module);

    FARPROC proc = GetProcAddress(module, "register_plugin");
    if (proc == NULL) {
        BOOST_LOG_TRIVIAL(error) << "Python plugin loader '" << loader_path.string()
                                 << "' does not export register_plugin.";
        return false;
    }
#else
    const boost::filesystem::path loader_path = current_executable_dir() / "plugins" / "python" / "plugin.so";
    if (!boost::filesystem::exists(loader_path)) {
        BOOST_LOG_TRIVIAL(warning) << "Python plugin loader not found for plugin tests: " << loader_path.string();
        return false;
    }

    static std::vector<void *> loaded_modules;
    void *module = dlopen(loader_path.string().c_str(), RTLD_NOW | RTLD_GLOBAL);
    if (module == nullptr) {
        BOOST_LOG_TRIVIAL(error) << "Cannot load Python plugin loader '" << loader_path.string()
                                 << "': " << dlerror();
        return false;
    }
    loaded_modules.push_back(module);

    void *proc = dlsym(module, "register_plugin");
    if (proc == nullptr) {
        BOOST_LOG_TRIVIAL(error) << "Python plugin loader '" << loader_path.string()
                                 << "' does not export register_plugin: " << dlerror();
        return false;
    }
#endif

    using RegisterPluginFn = void (*)(orchestrator_handle *);
    typedef boost::log::sinks::synchronous_sink<boost::log::sinks::text_ostream_backend> LogSink;
    const boost::shared_ptr<std::ostringstream> messages = boost::make_shared<std::ostringstream>();
    const boost::shared_ptr<LogSink> sink = boost::make_shared<LogSink>();
    sink->locked_backend()->add_stream(messages);
    boost::log::core::get()->add_sink(sink);
    const unsigned previous_logging_level = Slic3r::get_logging_level();
    // The application defaults to errors only, which rejects warnings before any capture sink sees them.
    Slic3r::set_logging_level(previous_logging_level < 2 ? 2 : previous_logging_level);
    try { reinterpret_cast<RegisterPluginFn>(proc)(orchestrator); }
    catch (...) {
        Slic3r::set_logging_level(previous_logging_level);
        boost::log::core::get()->remove_sink(sink);
        throw;
    }
    Slic3r::set_logging_level(previous_logging_level);
    sink->flush();
    boost::log::core::get()->remove_sink(sink);
    g_python_metadata_warnings = messages->str();
    const Slic3r::PluginPackageLoadReport *report = Slic3r::Orchestrator::instance().plugin_package_load_report(
        "python.test.clear.invalid_registration");
    if (report == nullptr)
        throw std::runtime_error("Invalid Python clear package was not discovered.");
    g_python_invalid_clear_package_report = *report;
    report = Slic3r::Orchestrator::instance().plugin_package_load_report("python.test.priority.low");
    if (report == nullptr) throw std::runtime_error("Low Python priority package was not discovered.");
    g_python_priority_low_report = *report;
    report = Slic3r::Orchestrator::instance().plugin_package_load_report("python.test.priority.high");
    if (report == nullptr) throw std::runtime_error("High Python priority package was not discovered.");
    g_python_priority_high_report = *report;
    return true;
}

#endif // SLIC3R_TEST_PYTHON_PLUGINS

void register_config_option_or_fail(orchestrator_handle *orchestrator,
                                    const raw_config_option_def &definition)
{
    const option_def_error_code result = orchestrator_create_option_def(orchestrator, &definition);
    if (result != OPTION_DEF_ERROR_OK) {
        const std::string key = definition.opt_key == nullptr ? "<null>" : definition.opt_key;
        throw std::runtime_error(
            "Failed to register configuration introspection fixture '" + key +
            "' (error " + std::to_string(int(result)) + ").");
    }
}

/*
With a valid borrowed orchestrator handle, register a float option used by the failing provider and
then add two process-lifetime plugin objects before selector options are built. The host retains
borrowed callback contexts, while option-definition strings are copied by the C API; a definition or
registration failure throws through the test startup path, and active state is chosen separately.
*/
void register_exclusive_pruning_test_plugins(orchestrator_handle *orchestrator)
{
    Slic3r::Orchestrator &host = *reinterpret_cast<Slic3r::Orchestrator *>(orchestrator);
    raw_config_option_def failed_setting = raw_config_option_def_init();
    failed_setting.opt_key = k_exclusive_pruning_failed_setting;
    failed_setting.type = RAW_CO_FLOAT;
    failed_setting.container_type = RAW_CONTAINER_TYPE_PROJECT;
    failed_setting.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    failed_setting.printer_technology = RAW_PT_FFF;
    failed_setting.category = RAW_OPTION_CATEGORY_SLICING;
    failed_setting.invalidates_step = STEP_POST_SLICING;
    failed_setting.default_serialized_value = "0.25";
    failed_setting.mode = RAW_CONFIG_OPTION_MODE_ADV_EXP | RAW_CONFIG_OPTION_MODE_SUSI;
    register_config_option_or_fail(orchestrator, failed_setting);

    static ExclusivePruningTestPlugin failed(
        orchestrator, k_exclusive_pruning_failed_plugin_id, k_exclusive_pruning_failed_setting,
        nullptr, -100, true);
    static ExclusivePruningTestPlugin valid(
        orchestrator, k_exclusive_pruning_valid_plugin_id, k_exclusive_pruning_valid_setting,
        k_exclusive_pruning_valid_setting, 100, false);

    if (host.get_plugin(k_exclusive_pruning_failed_plugin_id) == nullptr &&
        !host.register_plugin(failed.c_instance()))
        throw std::runtime_error("Could not register the failing exclusive-selector test provider.");
    if (host.get_plugin(k_exclusive_pruning_valid_plugin_id) == nullptr &&
        !host.register_plugin(valid.c_instance()))
        throw std::runtime_error("Could not register the valid exclusive-selector test provider.");
}

void register_plugin_option_list_fixtures(orchestrator_handle *orchestrator)
{
    const auto register_enum = [orchestrator](const char *key,
                                              const key_value_string_pair_t *entries,
                                              uint32_t entry_count,
                                              int32_t contains_plugin_id) {
        raw_config_option_def def = raw_config_option_def_init();
        def.opt_key = key;
        def.type = RAW_CO_ENUM;
        def.gui_type = RAW_GUI_TYPE_SELECT_CLOSE;
        def.container_type = RAW_CONTAINER_TYPE_PROJECT;
        def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
        def.printer_technology = RAW_PT_FFF;
        def.category = RAW_OPTION_CATEGORY_INFILL;
        def.contains_plugin_id = contains_plugin_id;
        def.default_serialized_value = entries[0].value;
        def.enum_def.value_label_pairs = { entries, entry_count };
        register_config_option_or_fail(orchestrator, def);
    };
    const auto replicate = [orchestrator](const char *key, const char *source) {
        raw_option_def_patch patch = raw_option_def_patch_init();
        patch.kind = RAW_OPTION_DEF_PATCH_REPLICATE_ENUM;
        patch.opt_key = key;
        patch.source_option_key = source;
        if (orchestrator_modify_option_def(orchestrator, &patch) != OPTION_DEF_ERROR_OK)
            throw std::runtime_error(std::string("Failed to record a replicate patch for '") + key + "'.");
    };

    static const key_value_string_pair_t source_entries[] = { { "rectilinear", "Rectilinear" } };
    static const key_value_string_pair_t target_entries[] = { { "monotonic", "Monotonic" } };
    static const key_value_string_pair_t prune_entries[] = {
        { "rectilinear", "Rectilinear" },
        { "test.infill.missing_pattern", "Missing pattern" }
    };
    static const key_value_string_pair_t empty_entries[] = {
        { "test.infill.missing_pattern", "Missing pattern" }
    };

    register_enum("test.replicate.source", source_entries, 1, 0);
    register_enum("test.replicate.target", target_entries, 1, 0);
    replicate("test.replicate.target", "test.replicate.source");

    register_enum("test.cycle.a", source_entries, 1, 0);
    register_enum("test.cycle.b", target_entries, 1, 0);
    replicate("test.cycle.a", "test.cycle.b");
    replicate("test.cycle.b", "test.cycle.a");

    register_enum("test.infill.prune_list", prune_entries, 2, 1);
    register_enum("test.infill.empty_list", empty_entries, 1, 1);
    static const key_value_string_pair_t nonselector_auto_entries[] = {
        {"auto", "Not a provider"},
        {k_exclusive_pruning_failed_plugin_id, "Failed provider"},
        {k_exclusive_pruning_valid_plugin_id, "Valid provider"}
    };
    register_enum("test.selector.nonselector_auto", nonselector_auto_entries, 3, 1);

    // A contribution recorded before the target list exists must survive a
    // replication patch: replication is applied first, then contributions, so
    // the result does not depend on plugin initialization order.
    static const key_value_string_pair_t contributed_entry[] = { { "gyroid", "Gyroid" } };

    register_enum("test.replicate.contrib_source", source_entries, 1, 0);

    raw_option_def_patch contrib_add = raw_option_def_patch_init();
    contrib_add.kind = RAW_OPTION_DEF_PATCH_ADD_ENUM_VALUES;
    contrib_add.opt_key = "test.replicate.contrib_target";
    contrib_add.enum_values = { contributed_entry, 1 };
    if (orchestrator_modify_option_def(orchestrator, &contrib_add) != OPTION_DEF_ERROR_OK)
        throw std::runtime_error("Failed to record the pre-creation contribution patch.");

    register_enum("test.replicate.contrib_target", target_entries, 1, 0);
    replicate("test.replicate.contrib_target", "test.replicate.contrib_source");

    // Same contribution after the target exists must give the same result.
    register_enum("test.replicate.order_source", source_entries, 1, 0);
    register_enum("test.replicate.order_target", target_entries, 1, 0);

    raw_option_def_patch order_add = raw_option_def_patch_init();
    order_add.kind = RAW_OPTION_DEF_PATCH_ADD_ENUM_VALUES;
    order_add.opt_key = "test.replicate.order_target";
    order_add.enum_values = { contributed_entry, 1 };
    if (orchestrator_modify_option_def(orchestrator, &order_add) != OPTION_DEF_ERROR_OK)
        throw std::runtime_error("Failed to record the post-creation contribution patch.");

    replicate("test.replicate.order_target", "test.replicate.order_source");
}

// Register metadata and withdrawal combinations through a borrowed handle before finalization; the host owns copies, named registration failures throw, and validation results are retained for assertions.
void register_config_introspection_fixture(orchestrator_handle *orchestrator)
{
    static const char *aliases[] = {"test_introspection_old", "test_introspection_legacy"};
    static const char *shortcuts[] = {"perimeter_speed", "infill_speed"};
    static const char *dependencies[] = {"layer_height#0", "nozzle_diameter#0"};
    static const key_value_string_pair_t enum_entries[] = {
        {"0.5", "Half"},
        {"1.25", "Nominal"},
    };
    static const raw_graph_type graph_types[] = {
        RAW_GRAPH_TYPE_LINEAR,
        RAW_GRAPH_TYPE_SPLINE,
    };
    static const c_vec2d graph_reset_points[] = {
        {0.0, 10.0},
        {100.0, 20.0},
    };
    static const c_vec2d graph_enforced_values[] = {
        {std::numeric_limits<double>::quiet_NaN(), 10.0},
        {100.0, 20.0},
    };
    static const c_vec2d small_graph_points[] = {
        {0.0, 0.0},
        {100.0, 0.001},
    };

    raw_config_option_def definition = raw_config_option_def_init();
    definition.opt_key = "test_config_definition_introspection";
    definition.type = RAW_CO_FLOAT;
    definition.category = RAW_OPTION_CATEGORY_ADVANCED;
    definition.level = RAW_OPTION_LEVEL_EXPERT;
    definition.container_type = RAW_CONTAINER_TYPE_PROJECT;
    definition.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    definition.gui_type = RAW_GUI_TYPE_F_ENUM_OPEN;
    definition.printer_technology = RAW_PT_FFF;
    definition.invalidates_step = STEP_INFILL;
    definition.is_optional = 1;
    definition.gui_flags = "serialized,show_value";
    definition.label = "Introspection value";
    definition.full_label = "Plugin configuration introspection value";
    definition.tooltip = "Exercises the complete normalized definition snapshot.";
    definition.sidetext = "mm";
    definition.cli = "test-config-definition-introspection=f";
    definition.ratio_over = "nozzle_diameter";
    definition.multiline = 1;
    definition.full_width = 1;
    definition.is_code = 1;
    definition.is_vector_extruder = 0;
    definition.readonly = 1;
    definition.can_phony = 1;
    definition.can_be_disabled = 1;
    definition.height = 7;
    definition.width = 19;
    definition.label_width = 23;
    definition.aligned_label_left = 1;
    definition.sidetext_width = 11;
    definition.min_value = -2.5;
    definition.max_value = 8.5;
    definition.has_min = 1;
    definition.has_max = 1;
    definition.max_literal_value = 4.5;
    definition.max_literal_is_percent = 1;
    definition.has_max_literal = 1;
    definition.precision = 4;
    definition.mode = RAW_CONFIG_OPTION_MODE_EXPERT;
    definition.flags = uint64_t(0x1020304050607080ULL);
    definition.nature = RAW_CONFIG_NATURE_BOOKKEEPING;
    definition.aliases = {aliases, 2};
    definition.shortcut = {shortcuts, 2};
    definition.depends_on = {dependencies, 2};
    definition.default_serialized_value = "1.25";
    definition.enum_def.value_label_pairs = {enum_entries, 2};

    register_config_option_or_fail(orchestrator, definition);

    raw_config_option_def vector_definition = raw_config_option_def_init();
    vector_definition.opt_key = "test_config_definition_vector_introspection";
    vector_definition.type = RAW_CO_VECTOR_FLOAT;
    vector_definition.container_type = RAW_CONTAINER_TYPE_PROJECT;
    vector_definition.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    vector_definition.printer_technology = RAW_PT_FFF;
    vector_definition.is_vector_extruder = 1;
    vector_definition.default_serialized_value = "1.25";
    register_config_option_or_fail(orchestrator, vector_definition);

    raw_config_option_def point3_definition = raw_config_option_def_init();
    point3_definition.opt_key = "test_config_definition_point3_introspection";
    point3_definition.type = RAW_CO_POINT3;
    point3_definition.container_type = RAW_CONTAINER_TYPE_PROJECT;
    point3_definition.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    point3_definition.printer_technology = RAW_PT_FFF;
    point3_definition.default_serialized_value = "1,2,3";
    register_config_option_or_fail(orchestrator, point3_definition);

    raw_graph_settings graph_settings = {};
    graph_settings.title = "Plugin graph";
    graph_settings.description = "Configure a value along the X axis.";
    graph_settings.y_label = "Value";
    graph_settings.x_label = "Position";
    graph_settings.null_label = "No graph values";
    graph_settings.min_x = 0.0;
    graph_settings.max_x = 100.0;
    graph_settings.step_x = 1.0;
    graph_settings.min_y = 0.0;
    graph_settings.max_y = 50.0;
    graph_settings.step_y = 0.5;
    graph_settings.label_min_x = "Minimum position";
    graph_settings.label_max_x = "Maximum position";
    graph_settings.label_min_y = "Minimum value";
    graph_settings.label_max_y = "Maximum value";
    graph_settings.allowed_types = graph_types;
    graph_settings.allowed_type_count = 2;
    graph_settings.reset_values = {
        0, 2, RAW_GRAPH_TYPE_SPLINE, graph_reset_points, 2};
    graph_settings.enforced_values = graph_enforced_values;
    graph_settings.enforced_value_count = 2;

    raw_config_option_def graph_definition = raw_config_option_def_init();
    graph_definition.opt_key = "test_config_definition_graph_introspection";
    graph_definition.type = RAW_CO_GRAPH;
    graph_definition.container_type = RAW_CONTAINER_TYPE_PROJECT;
    graph_definition.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    graph_definition.printer_technology = RAW_PT_FFF;
    graph_definition.default_serialized_value = "0:2:1:0x10:100x20";
    graph_definition.graph_settings = &graph_settings;
    register_config_option_or_fail(orchestrator, graph_definition);
    // Alternative providers may publish the same definition. Register it a
    // second time to exercise compatibility with the native NaN endpoint
    // sentinel before PrintConfigDef is finalized.
    register_config_option_or_fail(orchestrator, graph_definition);

    raw_graph_settings small_graph_settings = graph_settings;
    small_graph_settings.max_y = 0.001;
    small_graph_settings.step_y = 0.0001;
    small_graph_settings.reset_values = {
        0, 2, RAW_GRAPH_TYPE_LINEAR, small_graph_points, 2};
    small_graph_settings.enforced_values = nullptr;
    small_graph_settings.enforced_value_count = 0;

    raw_config_option_def small_graph_definition = graph_definition;
    small_graph_definition.opt_key = "test_config_definition_small_graph_range";
    small_graph_definition.default_serialized_value = "0:2:1:0x0:100x0.001";
    small_graph_definition.graph_settings = &small_graph_settings;
    register_config_option_or_fail(orchestrator, small_graph_definition);

    const struct NatureDefinition { const char *key; uint32_t nature; } nature_definitions[] = {
        {"test_config_nature_normal", RAW_CONFIG_NATURE_NORMAL},
        {"test_config_nature_local", RAW_CONFIG_NATURE_LOCAL},
        {"test_config_nature_bookkeeping", RAW_CONFIG_NATURE_BOOKKEEPING},
        {"test_config_nature_preset_meta", RAW_CONFIG_NATURE_PRESET_META},
        {"test_config_nature_volatile", RAW_CONFIG_NATURE_VOLATILE}};
    for (const NatureDefinition &entry : nature_definitions) {
        raw_config_option_def nature_definition = raw_config_option_def_init();
        nature_definition.opt_key = entry.key;
        nature_definition.type = RAW_CO_INT;
        nature_definition.container_type = RAW_CONTAINER_TYPE_PROJECT;
        nature_definition.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
        nature_definition.printer_technology = RAW_PT_FFF;
        nature_definition.default_serialized_value = "7";
        nature_definition.nature = entry.nature;
        register_config_option_or_fail(orchestrator, nature_definition);
    }

    raw_config_option_def invalid_nature = raw_config_option_def_init();
    invalid_nature.opt_key = "test_config_nature_invalid";
    invalid_nature.type = RAW_CO_INT;
    invalid_nature.container_type = RAW_CONTAINER_TYPE_PROJECT;
    invalid_nature.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    invalid_nature.printer_technology = RAW_PT_FFF;
    invalid_nature.default_serialized_value = "7";
    invalid_nature.nature = 16;
    Slic3r::Test::Plugins::plugin_test_invalid_nature_result = orchestrator_create_option_def(orchestrator, &invalid_nature);
    invalid_nature.opt_key = "test_config_nature_high_invalid";
    invalid_nature.nature = 0x100;
    Slic3r::Test::Plugins::plugin_test_high_invalid_nature_result = orchestrator_create_option_def(orchestrator, &invalid_nature);
    invalid_nature.opt_key = "test_config_nature_valid_unnamed";
    invalid_nature.nature = 5;
    Slic3r::Test::Plugins::plugin_test_combined_nature_result = orchestrator_create_option_def(orchestrator, &invalid_nature);
    invalid_nature.opt_key = "test_config_nature_combined";
    invalid_nature.nature = 10;
    Slic3r::Test::Plugins::plugin_test_roundtrip_nature_result = orchestrator_create_option_def(orchestrator, &invalid_nature);
}

void activate_plugin_or_fail(Slic3r::Orchestrator &orchestrator, const char *plugin_id)
{
    if (!orchestrator.set_plugin_active(plugin_id, true)) {
        BOOST_LOG_TRIVIAL(error) << "Test plugin '" << plugin_id << "' was not registered.";
        assert(false);
    }
}

} // namespace

namespace Slic3r::Test::Plugins {

// A function-local static caches successful initialization once per process; this no-argument
// void function registers and activates providers in the process-wide Orchestrator.
// The Orchestrator owns the registered Plugin objects, which borrow the provider instances; those
// instances stay owned by function-local statics in plugin sources and test files. Option
// definitions are finalized and FFF/SLA print-config caches are initialized. Exporter, regional seam, and
// storage test providers, plus the aligned, contiguous, cost, and random seam placers, are
// deactivated. The failing exclusive-selector test provider is disabled during initialization, and
// its one expected option diagnostic is filtered; the valid test provider is disabled after option
// finalization. Call before Slic3r::Print or DynamicPrintConfig::full_print_config(). A missing
// expected plugin is logged then asserted; a failed option or test-provider registration, or a
// Python plugin load, can throw std::runtime_error.
void ensure_plugin_test_runtime_initialized()
{
    static bool initialized = []() {
        PrintConfigDef::instance_mutable().init_common_params();
        init_fff_params(PrintConfigDef::instance_mutable());
        init_sla_params(PrintConfigDef::instance_mutable());

        Orchestrator &orchestrator = Orchestrator::instance();
        orchestrator_handle *orchestrator_handle_value =
            reinterpret_cast<orchestrator_handle *>(&orchestrator);
        Orchestrator::PluginRegistrationScope builtin_scope = orchestrator.plugin_registration_scope({}, false);
#ifdef SLIC3R_TEST_TOP_SURFACE_RETRACTION
        nematx_builtin_register_top_surface_retraction(orchestrator_handle_value);
        activate_plugin_or_fail(orchestrator, "nematx_top_surface_retraction");
#endif
        register_config_introspection_fixture(orchestrator_handle_value);
        register_plugin_option_list_fixtures(orchestrator_handle_value);
        slic3r_api::StandardLayerHeightGeneratorPlugin::register_standard_layer_height_generator_plugin(
            orchestrator_handle_value);
        slic3r_api::SliceVolumePlugin::register_slice_volume_plugin(orchestrator_handle_value);
        slic3r_api::MaxOverhangThresholdPlugin::register_max_overhang_threshold_plugin(orchestrator_handle_value);
        slic3r_api::OverhangSlopeFillPlugin::register_overhang_slope_fill_plugin(orchestrator_handle_value);
        slic3r_api::HoleSizeCompensationPlugin::register_hole_size_compensation_plugin(orchestrator_handle_value);
        slic3r_api::XYSizeCompensationPlugin::register_simple_xy_size_compensation_plugin(
            orchestrator_handle_value);
        slic3r_api::VaseMultiIslandConnectorPlugin::register_vase_multi_island_connector_plugin(
            orchestrator_handle_value);
        slic3r_api::GCodeGeneration::PrintingPlanFileWriterPlugin::register_printing_plan_file_writer_plugin(
            orchestrator_handle_value);
        slic3r_api::GCodeGeneration::Exporter::register_default_gcode_exporter_plugin(
            orchestrator_handle_value);
        slic3r_api::GCodeGeneration::Exporter::register_binary_gcode_exporter_plugin(
            orchestrator_handle_value);
        slic3r_api::GCodeGeneration::Exporter::register_bambu_gcode_exporter_plugin(
            orchestrator_handle_value);
        slic3r_api::GCodeGeneration::PostProcessScriptsPlugin::register_post_process_scripts_plugin(
            orchestrator_handle_value);
        register_gcode_exporter_test_providers(orchestrator_handle_value);
        slic3r_api::GCodeGeneration::SettingsGCodeScriptsPlugin::register_settings_gcode_scripts_plugin(
            orchestrator_handle_value);
        slic3r_api::GCodeGeneration::FeatureGCodePlugin::register_feature_gcode_plugin(
            orchestrator_handle_value);
        slic3r_api::GCodeGeneration::Reader::register_builtin_gcode_reader_plugins(
            orchestrator_handle_value);
        register_gcode_reader_test_providers(orchestrator_handle_value);
        slic3r_api::GCodeGeneration::CustomGCodePerPrintZPlugin::register_custom_gcode_per_print_z_plugins(
            orchestrator_handle_value);
        slic3r_api::GCodeGeneration::Firmware::register_builtin_gcode_firmware_plugins(
            orchestrator_handle_value);
        slic3r_api::GCodeGeneration::Thumbnail::register_builtin_gcode_thumbnail_plugins(
            orchestrator_handle_value);
        slic3r_api::FlatAreaLayerHeightPlugin::register_flat_area_layer_height_plugin(orchestrator_handle_value);
        slic3r_api::Infill::DefaultInfillGeneratorPlugin::register_default_infill_generator_plugin(
            orchestrator_handle_value);
        slic3r_api::Infill::AdaptiveCubicPatternPlugin::register_adaptive_cubic_pattern_plugin(
            orchestrator_handle_value);
        slic3r_api::Infill::AdaptiveCubicPatternPlugin::register_support_cubic_pattern_plugin(
            orchestrator_handle_value);
        slic3r_api::Infill::Lightning::register_lightning_pattern_plugin(orchestrator_handle_value);
        slic3r_api::Infill::GyroidPatternPlugin::register_gyroid_pattern_plugin(orchestrator_handle_value);
        slic3r_api::Infill::register_line_pattern_plugin(orchestrator_handle_value);
        slic3r_api::Infill::register_concentric_pattern_plugin(orchestrator_handle_value);
        slic3r_api::Infill::register_smooth_pattern_plugin(orchestrator_handle_value);
        slic3r_api::Infill::register_ensuring_pattern_plugin(orchestrator_handle_value);
        slic3r_api::Infill::register_honeycomb_pattern_plugin(orchestrator_handle_value);
        slic3r_api::Infill::register_honeycomb_3d_pattern_plugin(orchestrator_handle_value);
        slic3r_api::Infill::register_hilbert_curve_pattern_plugin(orchestrator_handle_value);
        slic3r_api::Infill::register_archimedean_chords_pattern_plugin(orchestrator_handle_value);
        slic3r_api::Infill::register_octagram_spiral_pattern_plugin(orchestrator_handle_value);
        slic3r_api::Infill::register_rectilinear_pattern_plugins(orchestrator_handle_value);
        slic3r_api::Infill::PostInfillGapFillPlugin::register_post_infill_gap_fill_plugin(
            orchestrator_handle_value);
        slic3r_api::Infill::PostFullFillVolumePlugin::register_post_full_fill_volume_plugin(
            orchestrator_handle_value);
        slic3r_api::Infill::InternalBridgeAnchorsPlugin::register_internal_bridge_anchors_plugin(
            orchestrator_handle_value);
        slic3r_api::DenseInfillPlugin::register_dense_infill_plugins(orchestrator_handle_value);
        slic3r_api::Ordering::DefaultOrderingPlugin::register_default_ordering_plugins(orchestrator_handle_value);
        slic3r_api::WipeTower::register_wipe_tower_manager_plugin(orchestrator_handle_value);
        slic3r_api::WipeTower::register_tool_change_wipe_plugin(orchestrator_handle_value);
        slic3r_api::WipeTower::ToolChangeUnretractPlugin::
            register_tool_change_unretract_plugin(orchestrator_handle_value);
        slic3r_api::SeamPlacement::AlignedPlugin::register_aligned_seam_placer_plugin(orchestrator_handle_value);
        slic3r_api::SeamPlacement::ContiguousPlugin::register_contiguous_seam_placer_plugin(
            orchestrator_handle_value);
        slic3r_api::SeamPlacement::CostPlugin::register_cost_seam_placer_plugin(orchestrator_handle_value);
        slic3r_api::SeamPlacement::FixedDirectionPlugin::register_fixed_direction_seam_placer_plugin(
            orchestrator_handle_value);
        slic3r_api::SeamPlacement::RandomPlugin::register_random_seam_placer_plugin(orchestrator_handle_value);
        register_regional_seam_test_provider(orchestrator_handle_value);
        register_plugin_storage_test_provider(orchestrator_handle_value);
        {
            // Register the shared pattern probe before selector settings are finalized (deactivated below); rejected registration or activation throws.
            Orchestrator &host = *reinterpret_cast<Orchestrator *>(orchestrator_handle_value);
            if (!host.register_plugin(Test::ProbePatternPlugin::pattern_probe().c_instance()) ||
                !host.set_plugin_active(Test::ProbePatternPlugin::probe_pattern_id, true))
                throw std::runtime_error("Cannot register the pattern probe test provider");
        }
        register_exclusive_pruning_test_plugins(orchestrator_handle_value);
        register_external_selector_catalog_providers(orchestrator_handle_value);
        register_support_toolpath_test_providers(orchestrator_handle_value);
        register_support_step_test_providers(orchestrator_handle_value);
        register_support_pattern_service_test_provider(orchestrator_handle_value);
        slic3r_api::LayerExtrusionEdit::CreateToolChangePlugin::register_create_tool_change_plugin(
            orchestrator_handle_value);
        slic3r_api::LayerExtrusionEdit::CreateTransitionScopePlugin::register_create_transition_scope_plugin(
            orchestrator_handle_value);
        slic3r_api::LayerExtrusionEdit::CreateRetractionPlugin::register_create_retraction_plugins(
            orchestrator_handle_value);
        slic3r_api::LayerExtrusionEdit::CreateRetractionLiftPlugin::
            register_create_retraction_lift_plugin(orchestrator_handle_value);
        slic3r_api::LayerExtrusionEdit::CreateRetractionWipePlugin::
            register_create_retraction_wipe_plugin(orchestrator_handle_value);
        slic3r_api::LayerExtrusionEdit::CreateRetractionWipePlugin::
            register_create_crossing_wipe_plugin(orchestrator_handle_value);
        slic3r_api::LayerExtrusionEdit::DefaultLayerEntryStatePlugin::register_default_layer_entry_state_plugin(
            orchestrator_handle_value);
        slic3r_api::LayerExtrusionEdit::DefaultTravelPlugin::register_default_travel_plugin(
            orchestrator_handle_value);
        slic3r_api::LayerExtrusionEdit::AvoidCrossingPerimetersTravelPlugin::
            register_avoid_crossing_perimeters_travel_plugin(orchestrator_handle_value);
        slic3r_api::LayerExtrusionEdit::AdvancedTravelPlugin::
            register_advanced_travel_plugin(orchestrator_handle_value);
        slic3r_api::LayerExtrusionEdit::AdvancedTravelPlugin::
            register_advanced_travel_planners(orchestrator_handle_value);
        slic3r_api::LayerExtrusionEdit::AdvancedTravelPlugin::
            register_advanced_travel_contributors(orchestrator_handle_value);
        slic3r_api::LayerExtrusionEdit::FirstLayerFlowPlugin::register_first_layer_flow_plugin(
            orchestrator_handle_value);
        slic3r_api::LayerExtrusionEdit::DefaultSpeedPlugin::register_default_speed_plugin(
            orchestrator_handle_value);
        slic3r_api::LayerExtrusionEdit::DefaultAccelerationPlugin::register_default_acceleration_plugin(
            orchestrator_handle_value);
        slic3r_api::LayerExtrusionEdit::DefaultTravelDecelerationPlugin::register_default_travel_deceleration_plugin(
            orchestrator_handle_value);
        slic3r_api::LayerExtrusionEdit::DefaultFanPlugin::register_default_fan_plugin(
            orchestrator_handle_value);
        slic3r_api::LayerExtrusionEdit::CoolingSlowdownPlugin::register_cooling_slowdown_plugin(orchestrator_handle_value);
        slic3r_api::LayerExtrusionEdit::FanCoolingPlugin::register_fan_cooling_plugin(orchestrator_handle_value);
        slic3r_api::LayerExtrusionEdit::FirstLayerFanPlugin::register_first_layer_fan_plugin(orchestrator_handle_value);
        slic3r_api::LayerExtrusionEdit::DynamicFanPlugin::register_dynamic_fan_plugin(orchestrator_handle_value);
        slic3r_api::LayerExtrusionEdit::DefaultTemperaturePlugin::register_default_temperature_plugins(
            orchestrator_handle_value);
        slic3r_api::SkirtBrim::DefaultBrimGeneratorPlugin::register_default_brim_generator_plugin(
            orchestrator_handle_value);
        slic3r_api::SkirtBrim::DefaultSkirtGeneratorPlugin::register_default_skirt_generator_plugin(
            orchestrator_handle_value);
        slic3r_api::SkirtBrim::DefaultBrimSkirtTrimPlugin::register_default_brim_skirt_trim_plugin(
            orchestrator_handle_value);
        slic3r_api::PolyholesPlugin::register_polyholes_plugin(orchestrator_handle_value);
        slic3r_api::NozzleCleaningPlugin::register_nozzle_cleaning_plugin(orchestrator_handle_value);
        register_builtin_support_demand_plugins(orchestrator_handle_value);
        slic3r_api::GuiRulesCorePlugin::register_gui_rules_core_plugin(orchestrator_handle_value);
        slic3r_api::Perimeter::ArachnePerimeterGeneratorPlugin::register_arachne_perimeter_generator_plugin(
            orchestrator_handle_value);
        slic3r_api::Perimeter::ClassicPerimeterGeneratorPlugin::register_classic_perimeter_generator_plugin(
            orchestrator_handle_value);
        slic3r_api::Perimeter::SimplePerimeterGeneratorPlugin::register_simple_perimeter_generator_plugin(
            orchestrator_handle_value);
        slic3r_api::Perimeter::ExtraPerimeterCountPlugin::register_extra_perimeter_count_plugin(
            orchestrator_handle_value);
        slic3r_api::Perimeter::ExtraPerimeterBelowAreaPlugin::register_extra_perimeter_below_area_plugin(
            orchestrator_handle_value);
        slic3r_api::Perimeter::ExtraPerimeterOddLayerPlugin::register_extra_perimeter_odd_layer_plugin(
            orchestrator_handle_value);
        slic3r_api::Perimeter::OnlyOnePerimeterFirstLayerPlugin::register_only_one_perimeter_first_layer_plugin(
            orchestrator_handle_value);
        slic3r_api::Perimeter::OnlyOnePerimeterOnTopPlugin::register_only_one_perimeter_on_top_plugin(
            orchestrator_handle_value);
        slic3r_api::Perimeter::SeparateHoleContourPlugin::register_separate_hole_contour_plugin(
            orchestrator_handle_value);
        slic3r_api::Perimeter::RemoveGapFillOnOverhangsPlugin::register_remove_gap_fill_on_overhangs_plugin(
            orchestrator_handle_value);
        slic3r_api::Perimeter::MarkFirstLoopPlugin::register_mark_first_loop_plugin(orchestrator_handle_value);
        slic3r_api::Perimeter::ExtraPerimetersOnOverhangsPlugin::register_extra_perimeters_on_overhangs_plugin(
            orchestrator_handle_value);
        slic3r_api::Perimeter::ExtraPerimeterOverhangWavePlugin::register_extra_perimeter_overhang_wave_plugin(
            orchestrator_handle_value);
        slic3r_api::Perimeter::DetectOverhangPlugin::register_detect_overhang_plugin(orchestrator_handle_value);
        slic3r_api::Perimeter::FuzzySkinPlugin::register_fuzzy_skin_plugin(orchestrator_handle_value);
        slic3r_api::Perimeter::LoopDirectionPlugin::register_loop_direction_plugin(orchestrator_handle_value);
        slic3r_api::SurfaceGeneration::InitialTypedSurfaceBuilderPlugin::register_initial_typed_surface_builder_plugin(
            orchestrator_handle_value);
        slic3r_api::SurfaceGeneration::SolidShellsPlugin::register_solid_shells_plugin(
            orchestrator_handle_value);
        // Register the complete bridge group so its options and callbacks exist
        // even in tests that leave the feature inactive.
        slic3r_api::BridgePreparation::register_surface(orchestrator_handle_value);
        slic3r_api::BridgePreparation::register_recipe(orchestrator_handle_value);
        slic3r_api::BridgeVolume::register_surface_plugin(orchestrator_handle_value);
        slic3r_api::BridgeVolume::register_plugin(orchestrator_handle_value);
        slic3r_api::SurfaceGeneration::TopSurfaceExpansionPlugin::register_top_surface_expansion_plugin(
            orchestrator_handle_value);
        slic3r_api::SurfaceGeneration::CleanInfillSurfacesPlugin::register_clean_infill_surfaces_plugin(
            orchestrator_handle_value);
        slic3r_api::SurfaceGeneration::InfillRegionCompatibilitySplitterPlugin::
            register_infill_region_compatibility_splitter_plugin(orchestrator_handle_value);
#ifdef SLIC3R_TEST_PYTHON_PLUGINS
        g_python_plugins_loaded = load_python_plugins_for_tests(orchestrator_handle_value) &&
                                  orchestrator.get_plugin("python.polyholes") != nullptr &&
                                  orchestrator.get_plugin("python.polyholes.high_level") != nullptr &&
                                  orchestrator.get_plugin("python.perimeter.post_process.hairy_object") != nullptr &&
                                  orchestrator.get_plugin("python.perimeter.generator.simple") != nullptr &&
                                  orchestrator.get_plugin("python.external.package_root") != nullptr;
#endif

        activate_plugin_or_fail(orchestrator, "bridge_detector.default");
        activate_plugin_or_fail(orchestrator, "standard_layer_height_generator");
        activate_plugin_or_fail(orchestrator, "slice_volume");
        activate_plugin_or_fail(orchestrator, "max_overhang_threshold");
        activate_plugin_or_fail(orchestrator, "overhang_slope_fill");
        activate_plugin_or_fail(orchestrator, "slicing.hole_size_compensation");
        activate_plugin_or_fail(orchestrator, "slicing.xy_size_compensation");
        activate_plugin_or_fail(orchestrator, "vase.multi_island_connector");
        activate_plugin_or_fail(orchestrator, "gcode.printing_plan_file_writer");
        activate_plugin_or_fail(orchestrator, "gcode.post_process");
        // A child test process can start without the built-in exporter to check how an unavailable default behaves.
        if (std::getenv("SLIC3R_TEST_SIMPLE_EXPORTER_DISABLED") == nullptr)
            activate_plugin_or_fail(orchestrator, "gcode.exporter.simple");
        else
            orchestrator.set_plugin_active("gcode.exporter.simple", false);
        activate_plugin_or_fail(orchestrator, "gcode.exporter.bgcode");
        activate_plugin_or_fail(orchestrator, "gcode.exporter.bambu3mf");
        activate_plugin_or_fail(orchestrator, "test.gcode.exporter.probe");
        activate_plugin_or_fail(orchestrator, "test.gcode.exporter.raw_query");
        activate_plugin_or_fail(orchestrator, "test.gcode.exporter.nofile");
        activate_plugin_or_fail(orchestrator, "test.gcode.exporter.inputs");
        activate_plugin_or_fail(orchestrator, "test.gcode.exporter.raw_inputs");
        activate_plugin_or_fail(orchestrator, "gcode.settings_scripts");
        activate_plugin_or_fail(orchestrator, "gcode.feature_gcode");
        activate_plugin_or_fail(orchestrator, "gcode.reader.default");
        activate_plugin_or_fail(orchestrator, "gcode.reader.bambu");
        activate_plugin_or_fail(orchestrator, "gcode.reader.klipper");
        activate_plugin_or_fail(orchestrator, "gcode.reader.marlin1");
        activate_plugin_or_fail(orchestrator, "gcode.reader.marlin2");
        activate_plugin_or_fail(orchestrator, "gcode.reader.prusa");
        activate_plugin_or_fail(orchestrator, "gcode.reader.reprap");
        activate_plugin_or_fail(orchestrator, "gcode.reader.sprinter");
        activate_plugin_or_fail(orchestrator, "gcode.reader.test.a");
        activate_plugin_or_fail(orchestrator, "gcode.reader.test.b");
        activate_plugin_or_fail(orchestrator, "gcode.reader.test.lower_priority");
        activate_plugin_or_fail(orchestrator, "gcode.reader.test.failing_probe");
        activate_plugin_or_fail(orchestrator, "gcode.reader.test.common_handlers");
        activate_plugin_or_fail(orchestrator, "gcode.reader.test.reprap_handlers");
        activate_plugin_or_fail(orchestrator, "gcode.reader.test.marlin_handlers");
        activate_plugin_or_fail(orchestrator, "gcode.reader.test.klipper_handlers");
        activate_plugin_or_fail(orchestrator, "gcode.custom_gcode_per_print_z");
        activate_plugin_or_fail(orchestrator, "marlin");
        activate_plugin_or_fail(orchestrator, "marlin2");
        activate_plugin_or_fail(orchestrator, "prusa");
        activate_plugin_or_fail(orchestrator, "reprapfirmware");
        activate_plugin_or_fail(orchestrator, "sprinter");
        activate_plugin_or_fail(orchestrator, "klipper");
        activate_plugin_or_fail(orchestrator, "bambu");
        activate_plugin_or_fail(orchestrator, "PNG");
        activate_plugin_or_fail(orchestrator, "JPG");
        activate_plugin_or_fail(orchestrator, "QOI");
        activate_plugin_or_fail(orchestrator, "BIQU");
        activate_plugin_or_fail(orchestrator, "flat_area_layer_height");
        // Infill generation mirrors the application default activation: the
        // generator owns the solid pattern setting and pattern plugins fill it.
        activate_plugin_or_fail(orchestrator, "infill.generator.default");
        activate_plugin_or_fail(orchestrator, "rectilinear");
        activate_plugin_or_fail(orchestrator, "monotonic");
        activate_plugin_or_fail(orchestrator, "alignedrectilinear");
        activate_plugin_or_fail(orchestrator, "grid");
        activate_plugin_or_fail(orchestrator, "triangles");
        activate_plugin_or_fail(orchestrator, "stars");
        activate_plugin_or_fail(orchestrator, "cubic");
        activate_plugin_or_fail(orchestrator, "line");
        activate_plugin_or_fail(orchestrator, "concentric");
        activate_plugin_or_fail(orchestrator, "honeycomb");
        activate_plugin_or_fail(orchestrator, "supportbase");
        activate_plugin_or_fail(orchestrator, "3dhoneycomb");
        activate_plugin_or_fail(orchestrator, "gyroid");
        activate_plugin_or_fail(orchestrator, "hilbertcurve");
        activate_plugin_or_fail(orchestrator, "archimedeanchords");
        activate_plugin_or_fail(orchestrator, "octagramspiral");
        activate_plugin_or_fail(orchestrator, "adaptivecubic");
        activate_plugin_or_fail(orchestrator, "supportcubic");
        activate_plugin_or_fail(orchestrator, "smooth");
        activate_plugin_or_fail(orchestrator, "rectiwithperimeter");
        activate_plugin_or_fail(orchestrator, "scatteredrectilinear");
        activate_plugin_or_fail(orchestrator, "sawtooth");
        activate_plugin_or_fail(orchestrator, "lightning");
        activate_plugin_or_fail(orchestrator, "ensuring");
        activate_plugin_or_fail(orchestrator, "dense_infill.surface_marker");
        activate_plugin_or_fail(orchestrator, "dense_infill.recipe_modifier");
        activate_plugin_or_fail(orchestrator, "dense_infill.post_infill_order");
        activate_plugin_or_fail(orchestrator, "ordering.plan_builder.by_layer");
        activate_plugin_or_fail(orchestrator, "ordering.plan_builder.by_object");
        activate_plugin_or_fail(orchestrator, "ordering.object_instance_catalog");
        activate_plugin_or_fail(orchestrator, "ordering.custom_gcode_tool_overrides");
        activate_plugin_or_fail(orchestrator, "ordering.tool_groups.default");
        activate_plugin_or_fail(orchestrator, "ordering.custom_gcode_event_tools");
        activate_plugin_or_fail(orchestrator, "ordering.printing_extrusion.presort.default");
        activate_plugin_or_fail(orchestrator, "ordering.wipe_into_first.default");
        activate_plugin_or_fail(orchestrator, "ordering.extrusion_tree.default");
        activate_plugin_or_fail(orchestrator, "wipe_tower.manager.default");
        activate_plugin_or_fail(orchestrator, "wipe_tower.wipe.toolchange.default");
        activate_plugin_or_fail(orchestrator, "wipe_tower.toolchange_unretract.default");
        activate_plugin_or_fail(orchestrator, "seam_placer.aligned");
        activate_plugin_or_fail(orchestrator, "seam_placer.contiguous");
        activate_plugin_or_fail(orchestrator, "seam_placer.cost");
        activate_plugin_or_fail(orchestrator, "seam_placer.fixed");
        activate_plugin_or_fail(orchestrator, "seam_placer.random");
        activate_plugin_or_fail(orchestrator, k_exclusive_pruning_failed_plugin_id);
        activate_plugin_or_fail(orchestrator, k_exclusive_pruning_valid_plugin_id);
        // Keep a second SeamPlacer active only while the generated regional
        // selector is registered and initialized for its dedicated tests.
        activate_plugin_or_fail(orchestrator, "slic3r.test.seam_placer.regional");
        activate_plugin_or_fail(orchestrator, "layer_extrusion_edit.transition_scope.default");
        activate_plugin_or_fail(orchestrator, "layer_extrusion_edit.entry_state.default");
        activate_plugin_or_fail(orchestrator, "layer_extrusion_edit.first_layer_flow");
        // Register the travel selector before typed configs cache its default; individual tests choose their editor chain.
        activate_plugin_or_fail(orchestrator, "layer_extrusion_edit.travel.default");
        activate_plugin_or_fail(orchestrator, "layer_extrusion_edit.travel.avoid_crossing_perimeters");
        activate_plugin_or_fail(orchestrator, "layer_extrusion_edit.travel.advanced");
        activate_plugin_or_fail(orchestrator, "layer_extrusion_edit.travel.advanced.planner.legacy");
        activate_plugin_or_fail(orchestrator, "layer_extrusion_edit.travel.advanced.planner.advanced");
        activate_plugin_or_fail(orchestrator, "layer_extrusion_edit.travel.advanced.zones.island_attractors");
        activate_plugin_or_fail(orchestrator, "layer_extrusion_edit.travel.advanced.zones.forbidden_tops");
        activate_plugin_or_fail(orchestrator, "layer_extrusion_edit.speed.default");
        activate_plugin_or_fail(orchestrator, "layer_extrusion_edit.acceleration.default");
        activate_plugin_or_fail(orchestrator, "layer_extrusion_edit.travel_deceleration.default");
        activate_plugin_or_fail(orchestrator, "layer_extrusion_edit.fan.default");
        activate_plugin_or_fail(orchestrator, "layer_extrusion_edit.speed.slowdown");
        activate_plugin_or_fail(orchestrator, "layer_extrusion_edit.fan.cooling");
        activate_plugin_or_fail(orchestrator, "layer_extrusion_edit.fan.first_layers");
        activate_plugin_or_fail(orchestrator, "layer_extrusion_edit.fan.dynamic");
        activate_plugin_or_fail(orchestrator, "extrusion_edit.startup_temperature.default");
        activate_plugin_or_fail(orchestrator, "layer_extrusion_edit.temperature.default");
        activate_plugin_or_fail(orchestrator, "layer_extrusion_edit.toolchange.default");
        activate_plugin_or_fail(orchestrator, "layer_extrusion_edit.retraction.default");
        activate_plugin_or_fail(orchestrator, "layer_extrusion_edit.lift.default");
        activate_plugin_or_fail(orchestrator, "skirt_brim.brim.default");
        activate_plugin_or_fail(orchestrator, "skirt_brim.skirt.default");
        activate_plugin_or_fail(orchestrator, "skirt_brim.brim_skirt_trim.default");
        activate_plugin_or_fail(orchestrator, "polyholes");
        activate_plugin_or_fail(orchestrator, "nozzle_cleaning");
        activate_plugin_or_fail(orchestrator, "support.demand.overhangs");
        activate_plugin_or_fail(orchestrator, "support.demand.painting");
        activate_plugin_or_fail(orchestrator, "support.demand.modifiers");
        activate_plugin_or_fail(orchestrator, "support.demand.buildplate_only");
        activate_plugin_or_fail(orchestrator, "support.demand.bridge_removal");
        activate_plugin_or_fail(orchestrator, "support.demand.bridge_infill_removal");
        activate_plugin_or_fail(orchestrator, "support.family.grid");
        activate_plugin_or_fail(orchestrator, "support.family.filled");
        activate_plugin_or_fail(orchestrator, "support.family.tree");
        activate_plugin_or_fail(orchestrator, "support.toolpath.legacy_sparse");
        activate_plugin_or_fail(orchestrator, "support.toolpath.legacy_bottom");
        activate_plugin_or_fail(orchestrator, "support.toolpath.legacy_bottom_contact");
        activate_plugin_or_fail(orchestrator, "support.toolpath.legacy_top");
        activate_plugin_or_fail(orchestrator, "support.toolpath.legacy_top_contact");
        activate_plugin_or_fail(orchestrator, "support.toolpath.interface_gap_fill");
        activate_plugin_or_fail(orchestrator, "support.toolpath.branch_sheath");
        activate_plugin_or_fail(orchestrator, "gui_rules.core");
        activate_plugin_or_fail(orchestrator, "perimeter.generator.arachne");
        activate_plugin_or_fail(orchestrator, "perimeter.generator.classic");
        activate_plugin_or_fail(orchestrator, "perimeter.generator.simple");
        activate_plugin_or_fail(orchestrator, "perimeter.module.extra_perimeter_count");
        activate_plugin_or_fail(orchestrator, "perimeter.module.extra_perimeter_below_area");
        activate_plugin_or_fail(orchestrator, "perimeter.module.extra_perimeter_odd_layer");
        activate_plugin_or_fail(orchestrator, "perimeter.module.only_one_perimeter_first_layer");
        activate_plugin_or_fail(orchestrator, "perimeter.module.only_one_perimeter_on_top");
        activate_plugin_or_fail(orchestrator, "perimeter.module.separate_hole_contour");
        activate_plugin_or_fail(orchestrator, "perimeter.module.remove_gap_fill_on_overhangs");
        activate_plugin_or_fail(orchestrator, "perimeter.module.mark_first_loop");
        activate_plugin_or_fail(orchestrator, "perimeter.post_process.extra_perimeters_on_overhangs");
        activate_plugin_or_fail(orchestrator, "perimeter.post_process.extra_perimeter_overhang_wave");
        activate_plugin_or_fail(orchestrator, "perimeter.post_process.detect_overhang");
        activate_plugin_or_fail(orchestrator, "perimeter.post_process.fuzzy_skin");
        activate_plugin_or_fail(orchestrator, "perimeter.post_process.loop_direction");
        activate_plugin_or_fail(orchestrator, "surface.initial_typed_surface_builder");
        activate_plugin_or_fail(orchestrator, "surface.solid_shells");
        activate_plugin_or_fail(orchestrator, "surface.bridge_preparation");
        activate_plugin_or_fail(orchestrator, "infill.recipe.bridge_preparation");
        activate_plugin_or_fail(orchestrator, "infill.post_process.internal_bridge_anchors");
        activate_plugin_or_fail(orchestrator, "bridge.finalize_surface_volume");
        activate_plugin_or_fail(orchestrator, "bridge.finalize_volume");
        activate_plugin_or_fail(orchestrator, "surface.top_surface_expansion");
        activate_plugin_or_fail(orchestrator, "surface.clean_infill_surfaces");
        activate_plugin_or_fail(orchestrator, "surface.infill_region_compatibility_splitter");
#ifdef SLIC3R_TEST_PYTHON_PLUGINS
        if (g_python_plugins_loaded) {
            activate_plugin_or_fail(orchestrator, "python.polyholes");
            activate_plugin_or_fail(orchestrator, "python.polyholes.high_level");
            activate_plugin_or_fail(orchestrator, "python.perimeter.post_process.hairy_object");
            activate_plugin_or_fail(orchestrator, "python.perimeter.generator.simple");
        }
#endif

        // Optional external-package integration checks use the real loader and
        // initialize settings before config definitions become immutable.
        if (const char *repository = std::getenv("SLIC3R_TEST_EXTERNAL_PLUGIN_REPOSITORY")) {
            load_plugin_packages_from_repository(boost::filesystem::path(repository), orchestrator);
            // Keep external dependency failures visible before activation's assertion.
            for (const auto &[id, report] : orchestrator.plugin_package_load_reports())
                if (boost::filesystem::path(report.package_path).parent_path() == boost::filesystem::path(repository))
                    for (const Slic3r::PluginPackageLoadIssue &issue : report.issues)
                        BOOST_LOG_TRIVIAL(error) << "External test package '" << id << "': " << issue.detail;
            if (const char *ids = std::getenv("SLIC3R_TEST_EXTERNAL_PLUGIN_IDS")) {
                std::string remaining(ids);
                while (!remaining.empty()) {
                    const auto comma = remaining.find(',');
                    activate_plugin_or_fail(orchestrator, remaining.substr(0,comma).c_str());
                    if (comma == std::string::npos) break;
                    remaining.erase(0,comma+1);
                }
            }
        }
        // An isolated import case needs the startup enum built without the object provider.
        if (std::getenv("SLIC3R_TEST_WITHOUT_BY_OBJECT") != nullptr)
            orchestrator.set_plugin_active("ordering.plan_builder.by_object", false);
        // External exclusive providers must be active when selectors collect their values.
        register_exclusive_step_group_options(orchestrator);
        // An isolated test run can prove that another encoder publishes every
        // shared option before PNG has ever been initialized.
        const bool without_png = std::getenv("SLIC3R_TEST_THUMBNAILS_WITHOUT_PNG") != nullptr;
        if (without_png) orchestrator.set_plugin_active("PNG", false);
        orchestrator.initialize_plugins();
        if (without_png) orchestrator.set_plugin_active("PNG", true);
        // Test-only defaults must precede option finalization and typed-config caches. The registered definition owns its
        // replacement bool; a missing/incompatible visibility option fails startup rather than silently enabling raytracing.
        t_optiondef_map &definitions = PrintConfigDef::instance_mutable().options;
        t_optiondef_map::iterator visibility = definitions.find("seam_visibility");
        if (visibility == definitions.end() || visibility->second.type != coBool ||
            dynamic_cast<const ConfigOptionBool *>(visibility->second.default_value.get()) == nullptr)
            throw std::runtime_error("Test runtime requires a registered bool seam_visibility before config caches");
        visibility->second.set_default_value(new ConfigOptionBool(false));
        orchestrator.finalize_plugin_option_defs();
        orchestrator.set_plugin_active("test.gcode.exporter.probe", false);
        orchestrator.set_plugin_active("test.gcode.exporter.raw_query", false);
        orchestrator.set_plugin_active("test.gcode.exporter.nofile", false);
        orchestrator.set_plugin_active("test.gcode.exporter.inputs", false);
        orchestrator.set_plugin_active("test.gcode.exporter.raw_inputs", false);
        // Preserve the shared test baseline; bridge integration cases opt in by scope.
        orchestrator.set_plugin_active("bridge.finalize_surface_volume", false);
        orchestrator.set_plugin_active("bridge.finalize_volume", false);
        orchestrator.set_plugin_active("infill.recipe.bridge_preparation", false);
        orchestrator.set_plugin_active("infill.post_process.internal_bridge_anchors", false);
        orchestrator.set_plugin_active("surface.bridge_preparation", false);
        orchestrator.set_plugin_active(k_exclusive_pruning_valid_plugin_id, false);
        orchestrator.set_plugin_active("com.example.selector", false);
        orchestrator.set_plugin_active("com.example.selector.foreign", false);
        orchestrator.set_plugin_active("com.example.selector.missing", false);
        {
            // Capture the empty plugin-id list diagnostic without hiding it
            // from tests that assert plugin messages later.
            std::vector<Orchestrator::PluginMessage> messages = orchestrator.consume_plugin_messages();
            for (const Orchestrator::PluginMessage &message : messages) {
                if (message.plugin_id == k_exclusive_pruning_failed_plugin_id &&
                    message.message.find(
                        "Plugin config option 'test.s2.failed_provider_setting' is already defined with incompatible type.") !=
                        std::string::npos)
                    continue;
                if (message.message.find("test.infill.empty_list") != std::string::npos) {
                    g_empty_option_list_error_reported = true;
                    continue;
                }
                orchestrator.add_plugin_message(message.level,
                                                orchestrator.get_plugin(message.plugin_id),
                                                message.step,
                                                message.message.c_str());
            }
        }
        register_exclusive_step_group_ui_fragments(orchestrator);
        orchestrator.set_plugin_active("layer_extrusion_edit.travel.default", false);
        orchestrator.set_plugin_active("layer_extrusion_edit.travel.avoid_crossing_perimeters", false);
        orchestrator.set_plugin_active("layer_extrusion_edit.travel.advanced", false);
        orchestrator.set_plugin_active("layer_extrusion_edit.travel.advanced.planner.legacy", false);
        orchestrator.set_plugin_active("layer_extrusion_edit.travel.advanced.planner.advanced", false);
        orchestrator.set_plugin_active("layer_extrusion_edit.travel.advanced.zones.island_attractors", false);
        orchestrator.set_plugin_active("layer_extrusion_edit.travel.advanced.zones.forbidden_tops", false);
        orchestrator.set_plugin_active("slic3r.test.seam_placer.regional", false);
        orchestrator.set_plugin_active("test.plugin_storage.support", false);
        orchestrator.set_plugin_active(Test::ProbePatternPlugin::probe_pattern_id, false);
        orchestrator.set_plugin_active("test.support.family.early", false);
        orchestrator.set_plugin_active("test.support.family.late", false);
        orchestrator.set_plugin_active("test.support.patterns", false);

        for (const char *id : {"test.support_toolpath.sparse", "test.support_toolpath.bottom",
                               "test.support_toolpath.top", "test.support_toolpath.alternate",
                               "test.support_toolpath.patterns"})
            orchestrator.set_plugin_active(id, false);
        orchestrator.set_plugin_active("seam_placer.aligned", false);
        orchestrator.set_plugin_active("seam_placer.contiguous", false);
        orchestrator.set_plugin_active("seam_placer.cost", false);
        orchestrator.set_plugin_active("seam_placer.random", false);
        initialize_fff_print_config_cache();
        initialize_sla_print_config_cache();
        PrintConfigDef::instance_mutable().finalize();
        // Copy every startup identity after option and fragment creation; later test registrations cannot enter these sets.
        for (Plugin *plugin : orchestrator.registered_plugins()) {
            g_startup_sets.provider_ids.insert(plugin->get_id());
            if (orchestrator.is_plugin_active(plugin)) g_startup_sets.active_provider_ids.insert(plugin->get_id());
            const std::map<slicing_step_t, Steps::StepExclusiveIdentity> &whole_steps = Steps::get_exclusive_steps();
            const std::map<slicing_step_t, Steps::StepExclusiveIdentity>::const_iterator whole = whole_steps.find(plugin->get_step());
            std::string key;
            if (whole != whole_steps.end())
                key = Steps::make_exclusive_step_group(orchestrator, plugin->get_step(), whole->second).option_key_storage;
            else if (!plugin->get_exclusive_group().empty())
                key = Steps::plugin_exclusive_group_option_key(orchestrator, plugin->get_step(), plugin->get_exclusive_group());
            if (!key.empty() && PrintConfigDef::instance().get(key) != nullptr) g_startup_sets.selector_keys.insert(key);
        }
        for (const std::string &layout : Orchestrator::ui_layout_files())
            for (const Orchestrator::PluginUiFragment &fragment : orchestrator.ui_fragments_for_file(layout))
                g_startup_sets.ui_fragments.emplace(layout, fragment.fragment_id);
        return true;
    }();
    (void)initialized;
}

const PluginTestStartupSets &plugin_test_startup_sets()
{
    ensure_plugin_test_runtime_initialized();
    return g_startup_sets;
}

bool python_plugin_test_runtime_available()
{
    ensure_plugin_test_runtime_initialized();
#ifdef SLIC3R_TEST_PYTHON_PLUGINS
    return g_python_plugins_loaded;
#else
    return false;
#endif
}

bool plugin_test_empty_option_list_error_reported()
{
    ensure_plugin_test_runtime_initialized();
    return g_empty_option_list_error_reported;
}

#ifdef SLIC3R_TEST_PYTHON_PLUGINS
const PluginPackageLoadReport &python_priority_package_report(bool upper)
{
    ensure_plugin_test_runtime_initialized();
    return upper ? g_python_priority_high_report : g_python_priority_low_report;
}

const std::string &python_metadata_warnings()
{
    ensure_plugin_test_runtime_initialized();
    return g_python_metadata_warnings;
}

const PluginPackageLoadReport &python_invalid_clear_package_report()
{
    ensure_plugin_test_runtime_initialized();
    return g_python_invalid_clear_package_report;
}
#endif

} // namespace Slic3r::Test::Plugins
