///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
Plugin loading and activation
=============================

This file coordinates the complete plugin startup sequence. It prepares the
installed package set, registers built-in and external plugins, creates the
configuration and GUI entries for exclusive plugin groups, and finally applies
the user's activation choices.

The normal execution flow is:

Native package loading is guarded by a package-registration scope. The loader
checks that the library exports `slic3r_plugin_abi_version()` and
`register_plugin()`, rejects incompatible C header versions, catches registration
exceptions, and records a per-package diagnostic instead of aborting the
whole plugin library. Loaded native modules remain resident for the process
lifetime because registered plugin instances may contain function pointers into
those modules.

Registration and activation are deliberately separate. Disabled plugins are
still registered so they can appear in the configuration dialog and provide
their metadata, but they are not selected for execution. A package is therefore
considered loaded when its registration succeeds, not only when it becomes
active. Exclusive step groups use stable plugin IDs as stored enum values and
user-facing names as labels; their setting and GUI fragment are created only
after the available plugin set is known.
StepSelectorTraits.hpp defines how step modules place selectors in the UI and enable
plugin settings. A plugin's own layout fragment takes precedence over generated placement.

Operation notes:
- reconcile the requested package configuration with the local cache
- register built-in plugins
- scan the plugins repository
- validate each package description
- load native libraries and call register_plugin()
- leave pure Python packages to PythonPluginLoader
- record package associations for successfully registered plugins
- activate the configured plugin IDs
- build dynamic choices for infill patterns and exclusive step groups
- initialize all registered plugins
- register GUI fragments that depend on initialized exclusive groups

Representative direct calls (branches are not a timeline):

    load_plugins()
        |-- register_builtin_plugins()
        |-- load_plugins_from_repository_impl()
        |-- activate_plugins_from_ids()
        |-- register_exclusive_step_group_options_impl()
        |-- orchestrator.initialize_plugins()
        |-- orchestrator.finalize_plugin_option_defs()
        `-- register_exclusive_step_group_ui_fragments_impl()
*/

#include "PluginLoader.hpp"
#include "PluginRepository.hpp"
#include "NematXBuiltinPlugins.hpp"

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
#endif

#include <array>
#include <chrono>
#include <iterator>
#include <map>
#include <new>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/log/trivial.hpp>
#include <boost/nowide/convert.hpp>
#include <boost/nowide/fstream.hpp>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/c/slic3r_plugin.h"
#include "libslic3r/Plugins/PluginApiCompatibility.hpp"
#include "libslic3r/Config/FFFPrintConfig.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/Plugins/GCode/CustomGCodePerPrintZ.hpp"
#include "libslic3r/Plugins/GCode/Exporter/BinaryGCodeExporter.hpp"
#include "libslic3r/Plugins/GCode/Exporter/BambuGCodeExporter.hpp"
#include "libslic3r/Plugins/GCode/Exporter/DefaultGCodeExporter.hpp"
#include "libslic3r/Plugins/GCode/FeatureGCode.hpp"
#include "libslic3r/Plugins/GCode/Firmware/BuiltinGCodeFirmwares.hpp"
#include "libslic3r/Plugins/GCode/Thumbnail/BuiltinGCodeThumbnails.hpp"
#include "libslic3r/Plugins/GCode/PostProcessScripts.hpp"
#include "libslic3r/Plugins/GCode/PrintingPlanFileWriter.hpp"
#include "libslic3r/Plugins/SeamPlacer/AlignedSeamPlacer.hpp"
#include "libslic3r/Plugins/SeamPlacer/ContiguousSeamPlacer.hpp"
#include "libslic3r/Plugins/SeamPlacer/CostSeamPlacer.hpp"
#include "libslic3r/Plugins/GCode/Reader/DefaultGCodeReader.hpp"
#include "libslic3r/Plugins/GCode/SettingsGCodeScripts.hpp"
#include "libslic3r/Plugins/GuiRulesCore.hpp"
#include "libslic3r/Plugins/Infill/AdaptiveCubic/AdaptiveCubicPattern.hpp"
#include "libslic3r/Plugins/Infill/Lightning/LightningPattern.hpp"
#include "libslic3r/Plugins/Infill/ConcentricPattern.hpp"
#include "libslic3r/Plugins/Infill/DefaultInfillGenerator.hpp"
#include "libslic3r/Plugins/Infill/BridgeVolume.hpp"
#include "libslic3r/Plugins/Infill/EnsuringPattern.hpp"
#include "libslic3r/Plugins/Infill/GyroidPattern.hpp"
#include "libslic3r/Plugins/Infill/PostFullFillVolume.hpp"
#include "libslic3r/Plugins/Infill/PostInfillGapFill.hpp"
#include "libslic3r/Plugins/Infill/Rectilinear/RectilinearPatterns.hpp"
#include "libslic3r/Plugins/Infill/SimpleInfillPatterns.hpp"
#include "libslic3r/Plugins/Infill/SmoothPattern.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/AdvancedTravel.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/AvoidCrossingPerimetersTravel.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/CreateRetraction.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/CoolingSlowdown.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/CreateRetractionLift.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/CreateRetractionWipe.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/CreateToolChange.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/CreateTransitionScope.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/DefaultAcceleration.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/DefaultFan.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/DefaultLayerEntryState.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/DefaultSpeed.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/DefaultTemperature.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/DefaultTravel.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/DefaultTravelDeceleration.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/DynamicFan.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/FanCooling.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/FirstLayerFan.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/FirstLayerFlow.hpp"
#include "libslic3r/Plugins/Ordering/DefaultOrdering.hpp"
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
#include "libslic3r/Plugins/Perimeter/MarkFirstLoop.hpp"
#include "libslic3r/Plugins/Perimeter/OnlyOnePerimeterFirstLayer.hpp"
#include "libslic3r/Plugins/Perimeter/OnlyOnePerimeterOnTop.hpp"
#include "libslic3r/Plugins/Perimeter/RemoveGapFillOnOverhangs.hpp"
#include "libslic3r/Plugins/Perimeter/SeparateHoleContour.hpp"
#include "libslic3r/Plugins/Perimeter/SimplePerimeterGenerator.hpp"
#include "libslic3r/Plugins/SeamPlacer/FixedDirectionSeamPlacer.hpp"
#include "libslic3r/Plugins/SeamPlacer/RandomSeamPlacer.hpp"
#include "libslic3r/Plugins/Slicing/MaxOverhangThreshold.hpp"
#include "libslic3r/Plugins/Slicing/OverhangSlopeFill.hpp"
#include "libslic3r/Plugins/Slicing/SliceVolume.hpp"
#include "libslic3r/Plugins/Slicing/StandardLayerHeightGenerator.hpp"
#include "libslic3r/Plugins/Slicing/VaseMultiIslandConnector.hpp"
#include "libslic3r/Plugins/HoleSizeCompensation.hpp"
#include "libslic3r/Plugins/XYSizeCompensation.hpp"
#include "libslic3r/Plugins/Surface/CleanInfillSurfaces.hpp"
#include "libslic3r/Plugins/Surface/BridgePreparation.hpp"
#include "libslic3r/Plugins/Surface/InitialTypedSurfaceBuilder.hpp"
#include "libslic3r/Plugins/Surface/InfillRegionCompatibilitySplitter.hpp"
#include "libslic3r/Plugins/Surface/SolidShells.hpp"
#include "libslic3r/Plugins/Surface/TopSurfaceExpansion.hpp"
#include "libslic3r/Plugins/Support/SupportDemandBridgeInfillRemoval.hpp"
#include "libslic3r/Plugins/Support/SupportDemandBridgeRemoval.hpp"
#include "libslic3r/Plugins/WipeTower/ToolChangeUnretract.hpp"
#include "libslic3r/Plugins/WipeTower/ToolChangeWipe.hpp"
#include "libslic3r/Plugins/WipeTower/WipeTowerManager.hpp"
#include "libslic3r/Plugins/Support/Filled/SupportFamilyFilled.hpp"
#include "libslic3r/Plugins/Support/Grid/SupportFamilyGrid.hpp"
#include "libslic3r/Plugins/Support/Tree/SupportFamilyTree.hpp"
#include "libslic3r/Plugins/Support/Toolpath/SupportToolpathBranch.hpp"
#include "libslic3r/Plugins/Support/Toolpath/SupportToolpathLegacy.hpp"
#include "libslic3r/Plugins/Support/Toolpath/SupportInterfaceGapFill.hpp"
#include "libslic3r/Plugins/Support/SupportDemandBuildplateOnly.hpp"
#include "libslic3r/Plugins/Support/SupportDemandModifiers.hpp"
#include "libslic3r/Plugins/Support/SupportDemandOverhangs.hpp"
#include "libslic3r/Plugins/Support/SupportDemandPainting.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"
#include "libslic3r/Utils.hpp"

namespace Slic3r {
namespace {

// Retain group descriptions before creating settings, copy choices in group-priority/id order and record their stage/group identity.
// The application owns descriptions/rules; slicing data is unchanged, and failed creation rolls back new records and propagates errors.
void register_exclusive_step_group_options_impl(Orchestrator &orchestrator);

// Return whether any supported layout file contains a UI fragment with this id.
bool has_ui_fragment_in_layout(Orchestrator &orchestrator, const std::string &fragment_id);

// Register module-owned built-ins with a non-null borrowed host; registrations and diagnostics are retained by the host.
void register_builtin_plugins(orchestrator_handle *orchestrator);

// When enabled by group rules, register STRING_EQUALS reasons activating settings read by each active nonnull provider, including automatic selection.
// Other selections keep settings visible, and other rules may activate them; the group key must be populated, and configuration values are unchanged.
// Borrow group/plugins/strings; GuiRuleRegistry copies or queues clauses, logs invalid registrations, and propagates allocation failures.
void add_exclusive_step_used_setting_rules(Orchestrator &orchestrator,
                                          const Steps::StepExclusiveGroup &group,
                                          const std::vector<Plugin *> &plugins);
// Build a STRING_EQUALS rule enabling a setting when its selector names the member or automatic selection.
// GuiRuleRegistry copies or queues the borrowed strings; setting values are unchanged, and allocations may throw.
void add_selector_rule(Orchestrator &orchestrator, const Steps::StepExclusiveGroup &group,
                       const std::string &setting, const char *selection, const std::string &owner);
// Add generated UI fragments at the declared locations unless a plugin already placed the group in a supported layout.
void register_exclusive_step_group_ui_fragments_impl(Orchestrator &orchestrator);

using RegisterPluginFn = void (*)(orchestrator_handle *);
using PluginAbiVersionFn = int32_t (*)(slic3r_major_minor_version *);
using PluginLoadClock = std::chrono::steady_clock;

// Startup runs before the GUI exists. Preserve a recoverable activation error
// until GUI_App can present it after creating its main frame.
std::optional<PluginActivationStartupError> g_plugin_activation_startup_error;

// Read the package descriptor before loading executable code. The loader uses
// its internal flag to accept runtime-support packages that intentionally
// register no plugin instance of their own.
bool read_plugin_package_description(const boost::filesystem::path &manifest,
                                     RepositoryDescription &description,
                                     std::string &error_message);

// Store one issue in the process report and mirror the same detail to logs.
void report_plugin_package_issue(Orchestrator &orchestrator,
                                 const std::string &package_id,
                                 PluginPackageLoadIssue issue);

// A pure Python package is loaded by the Python runtime package rather than by
// the native dynamic-library branch.
bool is_python_plugin_package(const boost::filesystem::path &package_root);

#ifdef _WIN32
// Convert a Windows loader code to the text shown by the operating system.
std::string windows_error_message(DWORD error_code);
#endif

std::chrono::milliseconds elapsed_ms(const PluginLoadClock::time_point &start)
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(PluginLoadClock::now() - start);
}

bool read_plugin_package_description(const boost::filesystem::path &manifest,
                                     RepositoryDescription &description,
                                     std::string &error_message)
{
    boost::nowide::ifstream stream(manifest.string(), std::ios::in | std::ios::binary);
    if (!stream) {
        error_message = "Cannot read package description '" + manifest.string() + "'.";
        return false;
    }
    const std::string contents((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
    return parse_repository_description(contents, RepositoryPackageType::Plugin, description, error_message);
}

void report_plugin_package_issue(Orchestrator &orchestrator,
                                 const std::string &package_id,
                                 PluginPackageLoadIssue issue)
{
    BOOST_LOG_TRIVIAL(warning) << "Plugin package '" << package_id << "' failed to load: " << issue.detail;
    orchestrator.report_plugin_package_load_issue(package_id, std::move(issue));
}

bool is_python_plugin_package(const boost::filesystem::path &package_root)
{
    const std::string package_id = package_root.filename().string();
    if (boost::filesystem::is_regular_file(package_root / "plugin.py") ||
        boost::filesystem::is_regular_file(package_root / (package_id + ".py")))
        return true;

    size_t python_file_count = 0;
    for (boost::filesystem::directory_iterator it(package_root), end; it != end; ++it)
        if (boost::filesystem::is_regular_file(it->path()) && it->path().extension() == ".py")
            ++python_file_count;
    return python_file_count == 1;
}

#ifdef _WIN32
std::string windows_error_message(DWORD error_code)
{
    wchar_t *message = nullptr;
    const DWORD flags = FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM |
                        FORMAT_MESSAGE_IGNORE_INSERTS;
    const DWORD length = FormatMessageW(flags, nullptr, error_code, 0,
                                        reinterpret_cast<wchar_t *>(&message), 0, nullptr);
    if (length == 0 || message == nullptr)
        return "Windows error " + std::to_string(error_code) + ".";

    std::wstring text(message, length);
    LocalFree(message);
    while (!text.empty() && (text.back() == L'\r' || text.back() == L'\n' || text.back() == L' '))
        text.pop_back();
    return boost::nowide::narrow(text);
}
#endif

void activate_plugins_from_ids(Orchestrator &orchestrator,
                               const std::vector<std::string> &plugin_ids,
                               bool from_user_config)
{
    orchestrator.clear_active_plugins();

    for (const std::string &plugin_id : plugin_ids) {
        if (orchestrator.set_plugin_active(plugin_id, true)) {
            continue;
        }

        if (from_user_config)
            BOOST_LOG_TRIVIAL(warning) << "Active plugin '" << plugin_id << "' is listed in "
                                       << "activated.ini but is not loaded.";
        else
            BOOST_LOG_TRIVIAL(trace) << "Default active plugin '" << plugin_id << "' is not loaded.";
    }
    // Validate after assembling the complete set, never against a partial list.
    orchestrator.block_unsatisfied_plugin_dependencies(plugin_ids);
    for (const auto &[id, error] : orchestrator.blocked_plugin_activations())
        BOOST_LOG_TRIVIAL(error) << "Activation blocked: " << error;
    for (const std::string &id : plugin_ids)
        if (orchestrator.is_plugin_active(id))
            BOOST_LOG_TRIVIAL(info) << "Activated plugin '" << id << "'.";
}

const char *plugin_package_library_filename()
{
#ifdef _WIN32
    return "plugin.dll";
#elif defined(__APPLE__)
    return "plugin.dylib";
#else
    return "plugin.so";
#endif
}


/*
Read the package table before registration. Its length is not a compatibility
version: newer SDKs can append unused entries. Keep allocation failures and
inconsistent exports as package diagnostics rather than aborting startup.
*/
bool read_plugin_api_versions(PluginAbiVersionFn version_fn, std::string &error_message)
{
    try {
        const int32_t count = version_fn(nullptr);
        if (count <= 0) {
            error_message = "The library returned an invalid API header version table size.";
            return false;
        }
        std::vector<slic3r_major_minor_version> versions(static_cast<size_t>(count));
        if (version_fn(versions.data()) != count) {
            error_message = "The library changed its API header version table size while it was queried.";
            return false;
        }
        const PluginApiCompatibility compatibility = validate_plugin_api_table(versions);
        error_message = compatibility.message();
        return compatibility.compatible();
    } catch (const std::bad_alloc &) {
        error_message = "Unable to allocate the library's API header version table.";
    } catch (const std::length_error &) {
        error_message = "The library's API header version table exceeds allocation limits.";
    }
    return false;
}

void load_plugin_library(const boost::filesystem::path &plugin_path,
                         const boost::filesystem::path &package_root,
                         const std::string &package_id,
                         bool internal_package,
                         orchestrator_handle *orchestrator)
{
    const PluginLoadClock::time_point start = PluginLoadClock::now();
    Orchestrator &host = *reinterpret_cast<Orchestrator *>(orchestrator);
    host.begin_plugin_package_load(package_id, package_root.string(), internal_package);
    // Validate the sidecar before executing any native code. The export check
    // below remains mandatory even when this package declaration is compatible.
    PluginPackageMetadata metadata;
    std::string metadata_error;
    if (!read_plugin_package_metadata((package_root / "version.ini").string(), metadata, metadata_error) ||
        !metadata.compatibility.compatible()) {
        PluginPackageLoadIssue issue;
        issue.code = PluginPackageLoadErrorCode::ApiHeaderVersionMismatch;
        issue.detail = metadata.compatibility.message();
        report_plugin_package_issue(host, package_id, std::move(issue));
        host.finish_plugin_package_load(package_id);
        return;
    }
#ifdef _WIN32
    static std::vector<HMODULE> loaded_modules;
    HMODULE module = LoadLibraryW(plugin_path.wstring().c_str());
    if (module == NULL) {
        const DWORD error_code = GetLastError();
        PluginPackageLoadIssue issue;
        issue.code = error_code == ERROR_MOD_NOT_FOUND || error_code == ERROR_DLL_NOT_FOUND ?
                         PluginPackageLoadErrorCode::DependencyMissing :
                         PluginPackageLoadErrorCode::LibraryOpenFailed;
        issue.system_error = uint32_t(error_code);
        issue.detail = "Cannot load '" + plugin_path.string() + "': " + windows_error_message(error_code);
        report_plugin_package_issue(host, package_id, std::move(issue));
        host.finish_plugin_package_load(package_id);
        return;
    }

    FARPROC abi_farproc = GetProcAddress(module, "slic3r_plugin_abi_version");
    if (abi_farproc == NULL) {
        PluginPackageLoadIssue issue;
        issue.code = PluginPackageLoadErrorCode::MissingAbiExport;
        issue.detail = "The library does not export slic3r_plugin_abi_version().";
        report_plugin_package_issue(host, package_id, std::move(issue));
        FreeLibrary(module);
        host.finish_plugin_package_load(package_id);
        return;
    }

    PluginAbiVersionFn abi_version_fn = reinterpret_cast<PluginAbiVersionFn>(abi_farproc);
    std::string api_version_error;
    if (!read_plugin_api_versions(abi_version_fn, api_version_error)) {
        PluginPackageLoadIssue issue;
        issue.code = PluginPackageLoadErrorCode::ApiHeaderVersionMismatch;
        issue.detail = std::move(api_version_error);
        report_plugin_package_issue(host, package_id, std::move(issue));
        FreeLibrary(module);
        host.finish_plugin_package_load(package_id);
        return;
    }

    FARPROC farproc = GetProcAddress(module, "register_plugin");
    if (farproc == NULL) {
        PluginPackageLoadIssue issue;
        issue.code = PluginPackageLoadErrorCode::MissingRegistrationExport;
        issue.detail = "The library does not export register_plugin().";
        report_plugin_package_issue(host, package_id, std::move(issue));
        FreeLibrary(module);
        host.finish_plugin_package_load(package_id);
        return;
    }

    RegisterPluginFn register_plugin_fn = reinterpret_cast<RegisterPluginFn>(farproc);
    try {
        Orchestrator::PluginRegistrationScope registration_scope(
            host.plugin_registration_scope(package_root.string(), true));
        register_plugin_fn(orchestrator);
    } catch (const std::exception &error) {
        PluginPackageLoadIssue issue;
        issue.code = PluginPackageLoadErrorCode::RegistrationFailed;
        issue.detail = error.what();
        report_plugin_package_issue(host, package_id, std::move(issue));
    } catch (...) {
        PluginPackageLoadIssue issue;
        issue.code = PluginPackageLoadErrorCode::RegistrationFailed;
        issue.detail = "register_plugin() threw an unknown exception.";
        report_plugin_package_issue(host, package_id, std::move(issue));
    }
    host.finish_plugin_package_load(package_id);
    const PluginPackageLoadReport *report = host.plugin_package_load_report(package_id);
    if (!internal_package && report != nullptr && report->registered_plugin_ids.empty())
        FreeLibrary(module);
    else
        loaded_modules.push_back(module);
    BOOST_LOG_TRIVIAL(debug) << "Loaded plugin DLL '" << plugin_path.string() << "' in "
                             << elapsed_ms(start).count() << " ms.";
#else
    static std::vector<void *> loaded_modules;
    void *module = dlopen(plugin_path.string().c_str(), RTLD_NOW | RTLD_GLOBAL);
    if (module == nullptr) {
        const char *loader_error = dlerror();
        const std::string detail = loader_error == nullptr ? "Unknown dynamic loader error." : loader_error;
        PluginPackageLoadIssue issue;
        issue.code = detail.find("not found") != std::string::npos ||
                     detail.find("No such file") != std::string::npos ?
                         PluginPackageLoadErrorCode::DependencyMissing :
                         PluginPackageLoadErrorCode::LibraryOpenFailed;
        issue.detail = "Cannot load '" + plugin_path.string() + "': " + detail;
        report_plugin_package_issue(host, package_id, std::move(issue));
        host.finish_plugin_package_load(package_id);
        return;
    }

    void *abi_symbol = dlsym(module, "slic3r_plugin_abi_version");
    if (abi_symbol == nullptr) {
        PluginPackageLoadIssue issue;
        issue.code = PluginPackageLoadErrorCode::MissingAbiExport;
        issue.detail = "The library does not export slic3r_plugin_abi_version().";
        report_plugin_package_issue(host, package_id, std::move(issue));
        dlclose(module);
        host.finish_plugin_package_load(package_id);
        return;
    }

    PluginAbiVersionFn abi_version_fn = reinterpret_cast<PluginAbiVersionFn>(abi_symbol);
    std::string api_version_error;
    if (!read_plugin_api_versions(abi_version_fn, api_version_error)) {
        PluginPackageLoadIssue issue;
        issue.code = PluginPackageLoadErrorCode::ApiHeaderVersionMismatch;
        issue.detail = std::move(api_version_error);
        report_plugin_package_issue(host, package_id, std::move(issue));
        dlclose(module);
        host.finish_plugin_package_load(package_id);
        return;
    }

    void *symbol = dlsym(module, "register_plugin");
    if (symbol == nullptr) {
        const char *symbol_error = dlerror();
        PluginPackageLoadIssue issue;
        issue.code = PluginPackageLoadErrorCode::MissingRegistrationExport;
        issue.detail = "The library does not export register_plugin()";
        if (symbol_error != nullptr)
            issue.detail += ": " + std::string(symbol_error);
        report_plugin_package_issue(host, package_id, std::move(issue));
        dlclose(module);
        host.finish_plugin_package_load(package_id);
        return;
    }

    RegisterPluginFn register_plugin_fn = reinterpret_cast<RegisterPluginFn>(symbol);
    try {
        Orchestrator::PluginRegistrationScope registration_scope(
            host.plugin_registration_scope(package_root.string(), true));
        register_plugin_fn(orchestrator);
    } catch (const std::exception &error) {
        PluginPackageLoadIssue issue;
        issue.code = PluginPackageLoadErrorCode::RegistrationFailed;
        issue.detail = error.what();
        report_plugin_package_issue(host, package_id, std::move(issue));
    } catch (...) {
        PluginPackageLoadIssue issue;
        issue.code = PluginPackageLoadErrorCode::RegistrationFailed;
        issue.detail = "register_plugin() threw an unknown exception.";
        report_plugin_package_issue(host, package_id, std::move(issue));
    }
    host.finish_plugin_package_load(package_id);
    const PluginPackageLoadReport *report = host.plugin_package_load_report(package_id);
    if (!internal_package && report != nullptr && report->registered_plugin_ids.empty())
        dlclose(module);
    else
        loaded_modules.push_back(module);
    BOOST_LOG_TRIVIAL(debug) << "Loaded plugin library '" << plugin_path.string() << "' in "
                             << elapsed_ms(start).count() << " ms.";
#endif
}

void load_plugins_from_repository_impl(
    const boost::filesystem::path &repository,
    orchestrator_handle *orchestrator,
    const std::map<std::string, PluginInstalledVersion> *bundled_exact_packages)
{
    Orchestrator &host = *reinterpret_cast<Orchestrator *>(orchestrator);
    if (!boost::filesystem::exists(repository)) {
        BOOST_LOG_TRIVIAL(trace) << "Plugin repository '" << repository.string() << "' does not exist.";
        if (bundled_exact_packages != nullptr)
            for (const auto &[package_name, version] : *bundled_exact_packages) {
                host.begin_plugin_package_load(package_name, (repository / package_name).string(), true);
                PluginPackageLoadIssue issue;
                issue.code = PluginPackageLoadErrorCode::PackageMissing;
                issue.detail = "The exact system package bundled with this application is unavailable.";
                report_plugin_package_issue(host, package_name, std::move(issue));
                host.finish_plugin_package_load(package_name);
            }
        return;
    }
    if (!boost::filesystem::is_directory(repository)) {
        BOOST_LOG_TRIVIAL(warning) << "Plugin repository path '" << repository.string() << "' is not a directory.";
        if (bundled_exact_packages != nullptr)
            for (const auto &[package_name, version] : *bundled_exact_packages) {
                host.begin_plugin_package_load(package_name, (repository / package_name).string(), true);
                PluginPackageLoadIssue issue;
                issue.code = PluginPackageLoadErrorCode::PackageMissing;
                issue.detail = "The exact system package bundled with this application is unavailable because "
                               "the plugin repository path is not a directory.";
                report_plugin_package_issue(host, package_name, std::move(issue));
                host.finish_plugin_package_load(package_name);
            }
        return;
    }

    std::set<std::string> found_bundled_exact;
    for (boost::filesystem::directory_iterator it(repository), end; it != end; ++it) {
        const boost::filesystem::path plugin_path = it->path();
        if (boost::filesystem::is_directory(plugin_path)) {
            const std::string package_name = plugin_path.filename().string();
            // Interrupted install transactions keep their staging and backup
            // folders beside the real package. Their leading dot makes them
            // ineligible for loading on the next launch.
            if (package_name.empty() || package_name.front() == '.')
                continue;

            const boost::filesystem::path package_library = plugin_path / plugin_package_library_filename();
            const boost::filesystem::path package_manifest = plugin_path / "description.ini";
            const boost::filesystem::path package_version = plugin_path / "version.ini";
            if (!boost::filesystem::is_regular_file(package_manifest) ||
                !boost::filesystem::is_regular_file(package_version)) {
                host.begin_plugin_package_load(package_name, plugin_path.string());
                PluginPackageLoadIssue issue;
                issue.code = PluginPackageLoadErrorCode::InvalidPackage;
                issue.detail = "The installed package must contain description.ini and version.ini.";
                report_plugin_package_issue(host, package_name, std::move(issue));
                host.finish_plugin_package_load(package_name);
                continue;
            }

            RepositoryDescription description;
            std::string description_error;
            if (!read_plugin_package_description(package_manifest, description, description_error) ||
                description.id != package_name) {
                host.begin_plugin_package_load(package_name, plugin_path.string());
                PluginPackageLoadIssue issue;
                issue.code = PluginPackageLoadErrorCode::InvalidPackage;
                issue.detail = description_error.empty() ?
                    "description.ini does not match the installed package directory." : description_error;
                report_plugin_package_issue(host, package_name, std::move(issue));
                host.finish_plugin_package_load(package_name);
                continue;
            }

            // A system-owned package must match the identity generated for
            // this application build. This guard prevents a stale live DLL
            // from loading when cache extraction or publication failed.
            const std::map<std::string, PluginInstalledVersion>::const_iterator expected =
                bundled_exact_packages == nullptr ?
                    std::map<std::string, PluginInstalledVersion>::const_iterator() :
                    bundled_exact_packages->find(package_name);
            const bool expected_exact = bundled_exact_packages != nullptr &&
                expected != bundled_exact_packages->end();
            if (expected_exact)
                found_bundled_exact.insert(package_name);
            if (description.is_bundled_exact || expected_exact) {
                PluginPackageMetadata metadata;
                std::string metadata_error;
                const bool identity_matches = expected_exact && description.is_bundled_exact &&
                    read_plugin_package_metadata(package_version.string(), metadata, metadata_error) &&
                    metadata.package_version == expected->second.package_version &&
                    metadata.slicer_version == expected->second.slicer_version &&
                    metadata.compatibility.compatible();
                if (!identity_matches) {
                    host.begin_plugin_package_load(package_name, plugin_path.string(), description.is_internal);
                    PluginPackageLoadIssue issue;
                    issue.code = PluginPackageLoadErrorCode::InvalidPackage;
                    issue.detail = expected_exact ?
                        "The system package does not match the exact version bundled with this application." :
                        "The system package is not declared by this application build.";
                    if (!metadata_error.empty())
                        issue.detail += " " + metadata_error;
                    report_plugin_package_issue(host, package_name, std::move(issue));
                    host.finish_plugin_package_load(package_name);
                    continue;
                }
            }

            if (boost::filesystem::is_regular_file(package_library)) {
                BOOST_LOG_TRIVIAL(info) << "Loading plugin package '" << plugin_path.string() << "'.";
                load_plugin_library(package_library, plugin_path, package_name,
                                    description.is_internal, orchestrator);
            } else if (!is_python_plugin_package(plugin_path)) {
                host.begin_plugin_package_load(package_name, plugin_path.string());
                PluginPackageLoadIssue issue;
                issue.code = PluginPackageLoadErrorCode::InvalidPackage;
                issue.detail = "The installed package contains neither a native plugin library nor a Python entry point.";
                report_plugin_package_issue(host, package_name, std::move(issue));
                host.finish_plugin_package_load(package_name);
            }
        }
    }

    if (bundled_exact_packages != nullptr) {
        for (const auto &[package_name, version] : *bundled_exact_packages) {
            if (found_bundled_exact.count(package_name) != 0)
                continue;
            const boost::filesystem::path package_root = repository / package_name;
            host.begin_plugin_package_load(package_name, package_root.string(), true);
            PluginPackageLoadIssue issue;
            issue.code = PluginPackageLoadErrorCode::PackageMissing;
            issue.detail = "The exact system package bundled with this application is unavailable.";
            report_plugin_package_issue(host, package_name, std::move(issue));
            host.finish_plugin_package_load(package_name);
        }
    }

    // The Python runtime reports each package it attempted. Anything still
    // absent after native loading could not reach that runtime at all.
    const PluginPackageLoadReport *python_runtime = host.plugin_package_load_report("python");
    for (boost::filesystem::directory_iterator it(repository), end; it != end; ++it) {
        const boost::filesystem::path package_root = it->path();
        if (!boost::filesystem::is_directory(package_root) ||
            boost::filesystem::is_regular_file(package_root / plugin_package_library_filename()) ||
            !is_python_plugin_package(package_root))
            continue;
        const std::string package_id = package_root.filename().string();
        if (host.plugin_package_load_report(package_id) != nullptr)
            continue;
        host.begin_plugin_package_load(package_id, package_root.string());
        PluginPackageLoadIssue issue;
        if (python_runtime == nullptr || python_runtime->state == PluginPackageLoadState::Failed) {
            issue.code = PluginPackageLoadErrorCode::PythonRuntimeUnavailable;
            issue.detail = "The Python plugin runtime package is not installed or did not load.";
        } else {
            issue.code = PluginPackageLoadErrorCode::PythonRegistrationFailed;
            issue.detail = "The Python runtime did not load this package.";
        }
        report_plugin_package_issue(host, package_id, std::move(issue));
        host.finish_plugin_package_load(package_id);
    }
}

void register_builtin_plugin(orchestrator_handle *orchestrator,
                             const char *plugin_name,
                             RegisterPluginFn register_plugin_fn)
{
    const PluginLoadClock::time_point start = PluginLoadClock::now();
    Orchestrator::PluginRegistrationScope registration_scope(
        reinterpret_cast<Orchestrator *>(orchestrator)->plugin_registration_scope(std::string(), false));
    register_plugin_fn(orchestrator);
    BOOST_LOG_TRIVIAL(debug) << "Loaded built-in plugin '" << plugin_name << "' in "
                             << elapsed_ms(start).count() << " ms.";
}

// Invoke each built-in registration helper with the borrowed host; instances remain owned by their modules.

void register_builtin_plugins(orchestrator_handle *orchestrator)
{
    register_builtin_plugin(orchestrator, "standard_layer_height_generator",
        slic3r_api::StandardLayerHeightGeneratorPlugin::register_standard_layer_height_generator_plugin);
    register_builtin_plugin(orchestrator, "slice_volume",
        slic3r_api::SliceVolumePlugin::register_slice_volume_plugin);
    register_builtin_plugin(orchestrator, "overhang_slope_fill",
        slic3r_api::OverhangSlopeFillPlugin::register_overhang_slope_fill_plugin);
    register_builtin_plugin(orchestrator, "slicing.hole_size_compensation",
        slic3r_api::HoleSizeCompensationPlugin::register_hole_size_compensation_plugin);
    register_builtin_plugin(orchestrator, "slicing.xy_size_compensation",
        slic3r_api::XYSizeCompensationPlugin::register_simple_xy_size_compensation_plugin);
    register_builtin_plugin(orchestrator, "max_overhang_threshold",
        slic3r_api::MaxOverhangThresholdPlugin::register_max_overhang_threshold_plugin);
    register_builtin_plugin(orchestrator, "vase.multi_island_connector",
        slic3r_api::VaseMultiIslandConnectorPlugin::register_vase_multi_island_connector_plugin);
    register_builtin_plugin(orchestrator, "gcode.printing_plan_file_writer",
        slic3r_api::GCodeGeneration::PrintingPlanFileWriterPlugin::register_printing_plan_file_writer_plugin);
    register_builtin_plugin(orchestrator, "gcode.post_process",
        slic3r_api::GCodeGeneration::PostProcessScriptsPlugin::register_post_process_scripts_plugin);
    register_builtin_plugin(orchestrator, "gcode.exporter.simple",
        slic3r_api::GCodeGeneration::Exporter::register_default_gcode_exporter_plugin);
    register_builtin_plugin(orchestrator, "gcode.exporter.bgcode",
        slic3r_api::GCodeGeneration::Exporter::register_binary_gcode_exporter_plugin);
    register_builtin_plugin(orchestrator, "gcode.exporter.bambu3mf",
        slic3r_api::GCodeGeneration::Exporter::register_bambu_gcode_exporter_plugin);
    register_builtin_plugin(
        orchestrator, "gcode.settings_scripts",
        slic3r_api::GCodeGeneration::SettingsGCodeScriptsPlugin::register_settings_gcode_scripts_plugin);
    register_builtin_plugin(
        orchestrator, "gcode.feature_gcode",
        slic3r_api::GCodeGeneration::FeatureGCodePlugin::register_feature_gcode_plugin);
    register_builtin_plugin(
        orchestrator, "built-in G-code readers",
        slic3r_api::GCodeGeneration::Reader::register_builtin_gcode_reader_plugins);
    register_builtin_plugin(
        orchestrator, "custom G-code at height",
        slic3r_api::GCodeGeneration::CustomGCodePerPrintZPlugin::register_custom_gcode_per_print_z_plugins);
    register_builtin_plugin(orchestrator, "gcode firmware dialects",
        slic3r_api::GCodeGeneration::Firmware::register_builtin_gcode_firmware_plugins);
    register_builtin_plugin(orchestrator, "G-code thumbnail encoders",
        slic3r_api::GCodeGeneration::Thumbnail::register_builtin_gcode_thumbnail_plugins);
    register_builtin_plugin(orchestrator, "infill.generator.default",
        slic3r_api::Infill::DefaultInfillGeneratorPlugin::register_default_infill_generator_plugin);
    register_builtin_plugin(orchestrator, "adaptivecubic",
        slic3r_api::Infill::AdaptiveCubicPatternPlugin::register_adaptive_cubic_pattern_plugin);
    register_builtin_plugin(orchestrator, "supportcubic",
        slic3r_api::Infill::AdaptiveCubicPatternPlugin::register_support_cubic_pattern_plugin);
    register_builtin_plugin(orchestrator, "lightning",
        slic3r_api::Infill::Lightning::register_lightning_pattern_plugin);
    register_builtin_plugin(orchestrator, "gyroid",
        slic3r_api::Infill::GyroidPatternPlugin::register_gyroid_pattern_plugin);
    register_builtin_plugin(orchestrator, "line", slic3r_api::Infill::register_line_pattern_plugin);
    register_builtin_plugin(orchestrator, "concentric", slic3r_api::Infill::register_concentric_pattern_plugin);
    register_builtin_plugin(orchestrator, "smooth", slic3r_api::Infill::register_smooth_pattern_plugin);
    register_builtin_plugin(orchestrator, "ensuring", slic3r_api::Infill::register_ensuring_pattern_plugin);
    register_builtin_plugin(orchestrator, "honeycomb",
        slic3r_api::Infill::register_honeycomb_pattern_plugin);
    register_builtin_plugin(orchestrator, "3dhoneycomb",
        slic3r_api::Infill::register_honeycomb_3d_pattern_plugin);
    register_builtin_plugin(orchestrator, "hilbertcurve",
        slic3r_api::Infill::register_hilbert_curve_pattern_plugin);
    register_builtin_plugin(orchestrator, "archimedeanchords",
        slic3r_api::Infill::register_archimedean_chords_pattern_plugin);
    register_builtin_plugin(orchestrator, "octagramspiral",
        slic3r_api::Infill::register_octagram_spiral_pattern_plugin);
    register_builtin_plugin(orchestrator, "rectilinear_patterns",
        slic3r_api::Infill::register_rectilinear_pattern_plugins);
    register_builtin_plugin(orchestrator, "infill.post_process.gap_fill",
        slic3r_api::Infill::PostInfillGapFillPlugin::register_post_infill_gap_fill_plugin);
    register_builtin_plugin(orchestrator, "infill.post_process.full_fill_volume",
        slic3r_api::Infill::PostFullFillVolumePlugin::register_post_full_fill_volume_plugin);
    register_builtin_plugin(orchestrator, "ordering.default",
        slic3r_api::Ordering::DefaultOrderingPlugin::register_default_ordering_plugins);
    register_builtin_plugin(orchestrator, "wipe_tower.manager.default",
        slic3r_api::WipeTower::register_wipe_tower_manager_plugin);
    register_builtin_plugin(orchestrator, "wipe_tower.wipe.toolchange.default",
        slic3r_api::WipeTower::register_tool_change_wipe_plugin);
    register_builtin_plugin(orchestrator, "wipe_tower.toolchange_unretract.default",
        slic3r_api::WipeTower::ToolChangeUnretractPlugin::register_tool_change_unretract_plugin);
    register_builtin_plugin(orchestrator, "seam_placer.aligned",
        slic3r_api::SeamPlacement::AlignedPlugin::register_aligned_seam_placer_plugin);
    register_builtin_plugin(orchestrator, "seam_placer.contiguous",
        slic3r_api::SeamPlacement::ContiguousPlugin::register_contiguous_seam_placer_plugin);
    register_builtin_plugin(orchestrator, "seam_placer.cost",
        slic3r_api::SeamPlacement::CostPlugin::register_cost_seam_placer_plugin);
    register_builtin_plugin(orchestrator, "seam_placer.fixed",
        slic3r_api::SeamPlacement::FixedDirectionPlugin::register_fixed_direction_seam_placer_plugin);
    register_builtin_plugin(orchestrator, "seam_placer.random",
        slic3r_api::SeamPlacement::RandomPlugin::register_random_seam_placer_plugin);
    register_builtin_plugin(orchestrator, "layer_extrusion_edit.toolchange.default",
        slic3r_api::LayerExtrusionEdit::CreateToolChangePlugin::register_create_tool_change_plugin);
    register_builtin_plugin(orchestrator, "layer_extrusion_edit.transition_scope.default",
        slic3r_api::LayerExtrusionEdit::CreateTransitionScopePlugin::register_create_transition_scope_plugin);
    register_builtin_plugin(orchestrator, "layer_extrusion_edit.retraction.default",
        slic3r_api::LayerExtrusionEdit::CreateRetractionPlugin::register_create_retraction_plugins);
    register_builtin_plugin(orchestrator, "layer_extrusion_edit.wipe.default",
        slic3r_api::LayerExtrusionEdit::CreateRetractionWipePlugin::
            register_create_retraction_wipe_plugin);
    register_builtin_plugin(orchestrator, "layer_extrusion_edit.wipe.crossing",
        slic3r_api::LayerExtrusionEdit::CreateRetractionWipePlugin::
            register_create_crossing_wipe_plugin);
    register_builtin_plugin(orchestrator, "layer_extrusion_edit.lift.default",
        slic3r_api::LayerExtrusionEdit::CreateRetractionLiftPlugin::
            register_create_retraction_lift_plugin);
    register_builtin_plugin(orchestrator, "layer_extrusion_edit.entry_state.default",
        slic3r_api::LayerExtrusionEdit::DefaultLayerEntryStatePlugin::register_default_layer_entry_state_plugin);
    register_builtin_plugin(orchestrator, "layer_extrusion_edit.travel.default",
        slic3r_api::LayerExtrusionEdit::DefaultTravelPlugin::register_default_travel_plugin);
    register_builtin_plugin(orchestrator, "layer_extrusion_edit.travel.avoid_crossing_perimeters",
        slic3r_api::LayerExtrusionEdit::AvoidCrossingPerimetersTravelPlugin::
            register_avoid_crossing_perimeters_travel_plugin);
    register_builtin_plugin(orchestrator, "layer_extrusion_edit.travel.advanced",
        slic3r_api::LayerExtrusionEdit::AdvancedTravelPlugin::register_advanced_travel_plugin);
    register_builtin_plugin(orchestrator, "layer_extrusion_edit.first_layer_flow",
        slic3r_api::LayerExtrusionEdit::FirstLayerFlowPlugin::register_first_layer_flow_plugin);
    register_builtin_plugin(orchestrator, "layer_extrusion_edit.speed.default",
        slic3r_api::LayerExtrusionEdit::DefaultSpeedPlugin::register_default_speed_plugin);
    register_builtin_plugin(orchestrator, "layer_extrusion_edit.acceleration.default",
        slic3r_api::LayerExtrusionEdit::DefaultAccelerationPlugin::register_default_acceleration_plugin);
    register_builtin_plugin(orchestrator, "layer_extrusion_edit.travel_deceleration.default",
        slic3r_api::LayerExtrusionEdit::DefaultTravelDecelerationPlugin::register_default_travel_deceleration_plugin);
    register_builtin_plugin(orchestrator, "layer_extrusion_edit.speed.slowdown",
        slic3r_api::LayerExtrusionEdit::CoolingSlowdownPlugin::register_cooling_slowdown_plugin);
    register_builtin_plugin(orchestrator, "layer_extrusion_edit.fan.cooling",
        slic3r_api::LayerExtrusionEdit::FanCoolingPlugin::register_fan_cooling_plugin);
    register_builtin_plugin(orchestrator, "layer_extrusion_edit.fan.first_layers",
        slic3r_api::LayerExtrusionEdit::FirstLayerFanPlugin::register_first_layer_fan_plugin);
    register_builtin_plugin(orchestrator, "layer_extrusion_edit.fan.dynamic",
        slic3r_api::LayerExtrusionEdit::DynamicFanPlugin::register_dynamic_fan_plugin);
    register_builtin_plugin(orchestrator, "layer_extrusion_edit.fan.default",
        slic3r_api::LayerExtrusionEdit::DefaultFanPlugin::register_default_fan_plugin);
    register_builtin_plugin(orchestrator, "layer_extrusion_edit.temperature.default",
        slic3r_api::LayerExtrusionEdit::DefaultTemperaturePlugin::register_default_temperature_plugins);
    register_builtin_plugin(orchestrator, "skirt_brim.brim.default",
        slic3r_api::SkirtBrim::DefaultBrimGeneratorPlugin::register_default_brim_generator_plugin);
    register_builtin_plugin(orchestrator, "skirt_brim.skirt.default",
        slic3r_api::SkirtBrim::DefaultSkirtGeneratorPlugin::register_default_skirt_generator_plugin);
    register_builtin_plugin(orchestrator, "skirt_brim.brim_skirt_trim.default",
        slic3r_api::SkirtBrim::DefaultBrimSkirtTrimPlugin::register_default_brim_skirt_trim_plugin);
    register_builtin_plugin(orchestrator, "perimeter.module.extra_perimeter_count",
        slic3r_api::Perimeter::ExtraPerimeterCountPlugin::register_extra_perimeter_count_plugin);
    register_builtin_plugin(orchestrator, "perimeter.module.extra_perimeter_below_area",
        slic3r_api::Perimeter::ExtraPerimeterBelowAreaPlugin::register_extra_perimeter_below_area_plugin);
    register_builtin_plugin(orchestrator, "perimeter.module.extra_perimeter_odd_layer",
        slic3r_api::Perimeter::ExtraPerimeterOddLayerPlugin::register_extra_perimeter_odd_layer_plugin);
    register_builtin_plugin(orchestrator, "perimeter.module.only_one_perimeter_first_layer",
        slic3r_api::Perimeter::OnlyOnePerimeterFirstLayerPlugin::register_only_one_perimeter_first_layer_plugin);
    register_builtin_plugin(orchestrator, "perimeter.module.only_one_perimeter_on_top",
        slic3r_api::Perimeter::OnlyOnePerimeterOnTopPlugin::register_only_one_perimeter_on_top_plugin);
    register_builtin_plugin(orchestrator, "perimeter.module.separate_hole_contour",
        slic3r_api::Perimeter::SeparateHoleContourPlugin::register_separate_hole_contour_plugin);
    register_builtin_plugin(orchestrator, "perimeter.module.remove_gap_fill_on_overhangs",
        slic3r_api::Perimeter::RemoveGapFillOnOverhangsPlugin::register_remove_gap_fill_on_overhangs_plugin);
    register_builtin_plugin(orchestrator, "perimeter.module.mark_first_loop",
        slic3r_api::Perimeter::MarkFirstLoopPlugin::register_mark_first_loop_plugin);
    register_builtin_plugin(orchestrator, "perimeter.post_process.extra_perimeters_on_overhangs",
        slic3r_api::Perimeter::ExtraPerimetersOnOverhangsPlugin::register_extra_perimeters_on_overhangs_plugin);
    register_builtin_plugin(orchestrator, "perimeter.post_process.detect_overhang",
        slic3r_api::Perimeter::DetectOverhangPlugin::register_detect_overhang_plugin);
    register_builtin_plugin(orchestrator, "perimeter.post_process.fuzzy_skin",
        slic3r_api::Perimeter::FuzzySkinPlugin::register_fuzzy_skin_plugin);
    register_builtin_plugin(orchestrator, "perimeter.generator.arachne",
        slic3r_api::Perimeter::ArachnePerimeterGeneratorPlugin::register_arachne_perimeter_generator_plugin);
    register_builtin_plugin(orchestrator, "perimeter.generator.classic",
        slic3r_api::Perimeter::ClassicPerimeterGeneratorPlugin::register_classic_perimeter_generator_plugin);
    register_builtin_plugin(orchestrator, "perimeter.generator.simple",
        slic3r_api::Perimeter::SimplePerimeterGeneratorPlugin::register_simple_perimeter_generator_plugin);
    register_builtin_plugin(orchestrator, "surface.initial_typed_surface_builder",
        slic3r_api::SurfaceGeneration::InitialTypedSurfaceBuilderPlugin::register_initial_typed_surface_builder_plugin);
    register_builtin_plugin(orchestrator, "surface.solid_shells",
        slic3r_api::SurfaceGeneration::SolidShellsPlugin::register_solid_shells_plugin);
    register_builtin_plugin(orchestrator, "surface.bridge_preparation", slic3r_api::BridgePreparation::register_surface);
    register_builtin_plugin(orchestrator, "infill.recipe.bridge_preparation", slic3r_api::BridgePreparation::register_recipe);
    register_builtin_plugin(orchestrator, "bridge.finalize_surface_volume", slic3r_api::BridgeVolume::register_surface_plugin);
    register_builtin_plugin(orchestrator, "bridge.finalize_volume", slic3r_api::BridgeVolume::register_plugin);
    register_builtin_plugin(orchestrator, "surface.top_surface_expansion",
        slic3r_api::SurfaceGeneration::TopSurfaceExpansionPlugin::register_top_surface_expansion_plugin);
    register_builtin_plugin(orchestrator, "surface.clean_infill_surfaces",
        slic3r_api::SurfaceGeneration::CleanInfillSurfacesPlugin::register_clean_infill_surfaces_plugin);
    register_builtin_plugin(orchestrator, "surface.infill_region_compatibility_splitter",
        slic3r_api::SurfaceGeneration::InfillRegionCompatibilitySplitterPlugin::
            register_infill_region_compatibility_splitter_plugin);
    register_builtin_support_demand_plugins(orchestrator);
    // Last: the built-in activation rules reference settings created by the
    // other plugins; the pending-rule queue attaches the ones that appear later.
    register_builtin_plugin(orchestrator, "gui_rules.core",
        slic3r_api::GuiRulesCorePlugin::register_gui_rules_core_plugin);
}


void add_exclusive_step_used_setting_rules(Orchestrator &orchestrator,
                                           const Steps::StepExclusiveGroup &group,
                                           const std::vector<Plugin *> &plugins)
{
    if (!group.traits.provider_settings_rules)
        return;
    for (const Plugin *plugin : plugins) {
        for (const Plugin::UsedConfigKey &setting : plugin->get_used_config_keys()) {
            add_selector_rule(orchestrator, group, setting.key, plugin->get_id().c_str(), plugin->get_id());
            if (group.traits.auto_value)
                add_selector_rule(orchestrator, group, setting.key, SLIC3R_SELECTOR_AUTO_VALUE, plugin->get_id());
        }
    }
}

void add_selector_rule(Orchestrator &orchestrator, const Steps::StepExclusiveGroup &group,
                       const std::string &setting, const char *selection, const std::string &owner)
{
    raw_gui_rule_clause clause = raw_gui_rule_clause_init();
    clause.condition = RAW_GUI_RULE_CONDITION_STRING_EQUALS;
    clause.condition_key = group.option_def.opt_key;
    clause.condition_string_value = selection;
    raw_gui_rule rule = raw_gui_rule_init();
    rule.target_key = setting.c_str();
    rule.clauses = &clause;
    rule.clause_count = 1;
    orchestrator.gui_rules().add(&rule, owner.c_str());
}

void register_exclusive_step_group_options_impl(Orchestrator &orchestrator)
{
    orchestrator.elect_exclusive_group_definitions();
    for (Steps::StepExclusivePluginGroup plugin_group : Steps::active_exclusive_plugin_groups(orchestrator)) {
        // The selector must use the stable plugin id as the stored enum value,
        // while showing the user-facing plugin name in the GUI. This lets a
        // preset survive a label change without changing its serialized value.
        std::vector<std::pair<std::string, std::string>> plugin_ids_and_labels;
        plugin_ids_and_labels.reserve(plugin_group.plugins.size());
        for (const Plugin *plugin : plugin_group.plugins)
            plugin_ids_and_labels.emplace_back(plugin->get_id(), plugin->get_name());

        Steps::StepExclusiveGroup &group = plugin_group.group;
        const Plugin *preferred = Steps::default_provider_from_group(plugin_group.plugins);
        group.set_enum_plugins(orchestrator, plugin_ids_and_labels, preferred != nullptr ? preferred->get_id() : std::string());
        const Orchestrator::SelectorIdentity identity {
            plugin_group.whole_step ? Orchestrator::SelectorIdentityKind::WholeStep : Orchestrator::SelectorIdentityKind::PluginGroup,
            group.step, plugin_group.whole_step ? std::string() : group.group_id};
        if (orchestrator.create_selector_print_config(group.option_def, identity, group.traits.auto_value) != OPTION_DEF_ERROR_OK)
            throw std::runtime_error("Cannot create selector option '" + group.option_key_storage + "'");
        add_exclusive_step_used_setting_rules(orchestrator, group, plugin_group.plugins);
        for (const raw_gui_rule &rule : group.gui_activation_rules)
            orchestrator.gui_rules().add(&rule);
    }
}

bool has_ui_fragment(Orchestrator &orchestrator,
                     const std::string &target_file,
                     const std::string &fragment_id)
{
    const std::vector<Orchestrator::PluginUiFragment> fragments =
        orchestrator.ui_fragments_for_file(target_file);
    for (const Orchestrator::PluginUiFragment &fragment : fragments)
        if (fragment.fragment_id == fragment_id)
            return true;
    return false;
}

bool has_ui_fragment_in_layout(Orchestrator &orchestrator, const std::string &fragment_id)
{
    /*
    Exclusive group selectors have one logical fragment id. A plugin may place
    that selector in a file that better matches the setting category, for
    example printer_fff.ui for firmware/G-code choices. The generic print.ui
    fallback must then stay silent, otherwise the same setting appears twice in
    the GUI.
    */
    for (const std::string &target_file : Orchestrator::ui_layout_files())
        if (has_ui_fragment(orchestrator, target_file, fragment_id))
            return true;
    return false;
}

void register_exclusive_step_group_ui_fragments_impl(Orchestrator &orchestrator)
{
    for (Steps::StepExclusivePluginGroup plugin_group : Steps::active_exclusive_plugin_groups(orchestrator)) {
        // The fragment id is the group id, not the generated option key. This
        // lets several plugins in the same group provide the same placement
        // fragment while keeping de-duplication stable and independent from the
        // generated setting name.
        const Steps::StepExclusiveGroup &group = plugin_group.group;
        // A feature plugin may place the selector next to its own controls.
        // Generate placement from the group's declaration only for groups that
        // did not already publish an explicit placement fragment.
        if (has_ui_fragment_in_layout(orchestrator, group.group_id))
            continue;
        orchestrator.add_ui_fragment(group.traits.ui_target_file.c_str(), group.group_id.c_str(), group.ui_fragment.c_str(), 0);
    }
}

} // namespace

// Register demand, grid geometry and native toolpath providers before initialization; startup and tests share these registrations and activation defaults.
void register_builtin_support_demand_plugins(orchestrator_handle *orchestrator)
{
    register_builtin_plugin(orchestrator, "support.demand.overhangs",
        slic3r_api::Support::SupportDemandOverhangsPlugin::register_support_demand_overhangs_plugin);
    register_builtin_plugin(orchestrator, "support.demand.painting",
        slic3r_api::Support::SupportDemandPaintingPlugin::register_support_demand_painting_plugin);
    register_builtin_plugin(orchestrator, "support.demand.modifiers",
        slic3r_api::Support::SupportDemandModifiersPlugin::register_support_demand_modifiers_plugin);
    register_builtin_plugin(orchestrator, "support.demand.buildplate_only",
        slic3r_api::Support::SupportDemandBuildplateOnlyPlugin::register_support_demand_buildplate_only_plugin);
    register_builtin_plugin(orchestrator, "support.demand.bridge_removal",
        slic3r_api::Support::SupportDemandBridgeRemovalPlugin::register_support_demand_bridge_removal_plugin);
    register_builtin_plugin(orchestrator, "support.demand.bridge_infill_removal",
        slic3r_api::Support::SupportDemandBridgeInfillRemovalPlugin::register_support_demand_bridge_infill_removal_plugin);
    register_builtin_plugin(orchestrator, "support.family.grid",
        slic3r_api::Support::Grid::register_support_family_grid_plugin);
    register_builtin_plugin(orchestrator, "support.family.filled",
        slic3r_api::Support::Filled::register_support_family_filled_plugin);
    register_builtin_plugin(orchestrator, "support.family.tree",
        slic3r_api::Support::Tree::register_support_family_tree_plugin);
    register_builtin_plugin(orchestrator, "support.toolpath.legacy_sparse",
        slic3r_api::Support::Toolpath::register_support_toolpath_sparse_plugin);
    register_builtin_plugin(orchestrator, "support.toolpath.legacy_bottom",
        slic3r_api::Support::Toolpath::register_support_toolpath_bottom_plugin);
    register_builtin_plugin(orchestrator, "support.toolpath.legacy_bottom_contact",
        slic3r_api::Support::Toolpath::register_support_toolpath_bottom_contact_plugin);
    register_builtin_plugin(orchestrator, "support.toolpath.legacy_top",
        slic3r_api::Support::Toolpath::register_support_toolpath_top_plugin);
    register_builtin_plugin(orchestrator, "support.toolpath.legacy_top_contact",
        slic3r_api::Support::Toolpath::register_support_toolpath_top_contact_plugin);
    register_builtin_plugin(orchestrator, "support.toolpath.interface_gap_fill",
        slic3r_api::Support::Toolpath::register_support_interface_gap_fill_plugin);
    register_builtin_plugin(orchestrator, "support.toolpath.branch_sheath",
        slic3r_api::Support::Toolpath::register_support_toolpath_branch_plugin);
}

bool resolve_plugin_startup_activation_config(const boost::filesystem::path &data_directory,
                                              PluginActivationConfig &config,
                                              PluginActivationConfigSource &source,
                                              std::string &error_message)
{
    g_plugin_activation_startup_error.reset();
    config = {};
    source = PluginActivationConfigSource::DefaultsFallback;

    // Tools without a writable data directory intentionally consume the
    // resource configuration directly and have no user file to repair.
    if (data_directory.empty()) {
        bool ignored_from_user_config = false;
        return ensure_plugin_activation_config(data_directory, config,
                                               ignored_from_user_config, error_message);
    }

    std::string user_error;
    const boost::filesystem::path user_config_path = plugin_activation_config_path(data_directory);
    try {
        if (!boost::filesystem::exists(user_config_path)) {
            bool ignored_from_user_config = false;
            if (ensure_plugin_activation_config(data_directory, config,
                                                ignored_from_user_config, user_error)) {
                source = PluginActivationConfigSource::UserValid;
                error_message.clear();
                return true;
            }
        } else {
            PluginActivationConfigReadResult read_result =
                read_plugin_activation_config_tolerant(user_config_path);
            if (read_result.status == PluginActivationConfigStatus::Valid) {
                config = std::move(read_result.config);
                source = PluginActivationConfigSource::UserValid;
                error_message.clear();
                return true;
            }
            if (read_result.status == PluginActivationConfigStatus::PartiallyValid) {
                config = std::move(read_result.config);
                source = PluginActivationConfigSource::UserSanitized;

                PluginActivationStartupError startup_error;
                startup_error.config_path = user_config_path.string();
                startup_error.detail = std::move(read_result.error_message);
                startup_error.source = source;
                startup_error.issues = std::move(read_result.issues);
                startup_error.removed_packages = std::move(read_result.rejected_packages);
                startup_error.sanitized_config = config;
                g_plugin_activation_startup_error = std::move(startup_error);
                error_message.clear();
                return true;
            }
            user_error = std::move(read_result.error_message);
        }
    } catch (const boost::filesystem::filesystem_error &error) {
        user_error = "Cannot inspect plugin configuration '" + user_config_path.string() + "': " + error.what();
    }

    PluginActivationConfig default_config;
    bool ignored_from_user_config = false;
    std::string default_error;
    if (!ensure_plugin_activation_config(boost::filesystem::path(), default_config,
                                         ignored_from_user_config, default_error)) {
        config = {};
        source = PluginActivationConfigSource::DefaultsFallback;
        error_message = user_error + " Default plugin activations are also unavailable: " + default_error;
        return false;
    }

    // Copy only [activated]. Omitting the default package sections makes it
    // impossible for this fallback value to become a desired installation
    // state, even if a future caller forgets to inspect the source flag.
    config = {};
    config.activated = std::move(default_config.activated);
    source = PluginActivationConfigSource::DefaultsFallback;
    PluginActivationStartupError startup_error;
    startup_error.config_path = user_config_path.string();
    startup_error.detail = std::move(user_error);
    startup_error.source = source;
    startup_error.default_activations_used = true;
    startup_error.package_changes_skipped = true;
    g_plugin_activation_startup_error = std::move(startup_error);
    error_message.clear();
    return true;
}

std::optional<PluginActivationStartupError> take_plugin_activation_startup_error()
{
    std::optional<PluginActivationStartupError> error = std::move(g_plugin_activation_startup_error);
    g_plugin_activation_startup_error.reset();
    return error;
}

void register_exclusive_step_group_options(Orchestrator &orchestrator)
{
    register_exclusive_step_group_options_impl(orchestrator);
}

void load_plugin_packages_from_repository(const boost::filesystem::path &repository,
                                          Orchestrator &orchestrator)
{
    load_plugins_from_repository_impl(repository,
        reinterpret_cast<orchestrator_handle *>(&orchestrator), nullptr);
}

void register_exclusive_step_group_ui_fragments(Orchestrator &orchestrator)
{
    register_exclusive_step_group_ui_fragments_impl(orchestrator);
}

void register_exclusive_step_groups(Orchestrator &orchestrator)
{
    register_exclusive_step_group_options_impl(orchestrator);
    register_exclusive_step_group_ui_fragments_impl(orchestrator);
}

void load_plugins()
{
    const PluginLoadClock::time_point start = PluginLoadClock::now();
    Orchestrator &orchestrator = Orchestrator::instance();
    orchestrator.clear_plugin_package_load_reports();
    orchestrator_handle *orchestrator_handle_ptr = reinterpret_cast<orchestrator_handle *>(&orchestrator);

    // Package installation happens before any external DLL is loaded. This
    // keeps a running process from replacing a library that Windows or the
    // dynamic linker may still hold open.
    PluginActivationConfigSource activation_config_source = PluginActivationConfigSource::DefaultsFallback;
    const boost::filesystem::path config_dir = has_data_dir() ? boost::filesystem::path(data_dir()) :
                                                                boost::filesystem::path();
    PluginActivationConfig plugin_config;
    std::string plugin_config_error;
    if (!resolve_plugin_startup_activation_config(config_dir, plugin_config,
                                                  activation_config_source,
                                                  plugin_config_error))
        throw std::runtime_error(plugin_config_error);

    const bool user_configuration_available =
        activation_config_source != PluginActivationConfigSource::DefaultsFallback;
    const bool automatic_configuration_writes_allowed =
        activation_config_source == PluginActivationConfigSource::UserValid;

    if (const std::optional<PluginActivationStartupError> startup_error =
            g_plugin_activation_startup_error; startup_error.has_value()) {
        if (startup_error->source == PluginActivationConfigSource::UserSanitized) {
            BOOST_LOG_TRIVIAL(error) << startup_error->detail << " Valid plugin configuration entries are used "
                                     << "for this session. Rejected packages are removed from the desired state; "
                                     << "the user file remains unchanged.";
        } else {
            BOOST_LOG_TRIVIAL(error) << startup_error->detail << " Using default plugin activations for this "
                                     << "session. The user configuration remains unchanged and package changes "
                                     << "were not applied.";
        }
    }

    PluginActivationConfig default_plugin_config;
    bool default_plugin_config_available = false;
    if (!config_dir.empty()) {
        // A profile created before a later built-in plugin existed should pick
        // up the new default unless it explicitly keeps that id disabled.
        bool ignored_from_user_config = false;

        // Split the former combined editor before merging defaults. Copying its
        // false state now prevents the default merge from re-enabling either
        // independent successor in a profile that disabled the old plugin.
        bool activation_config_changed = user_configuration_available && migrate_plugin_activation_id(
            plugin_config,
            "layer_extrusion_edit.speed_acceleration.default",
            {"layer_extrusion_edit.speed.default", "layer_extrusion_edit.acceleration.default"});
        activation_config_changed = (user_configuration_available && migrate_plugin_activation_id(
            plugin_config, "layer_extrusion_edit.fan.default",
            {"layer_extrusion_edit.fan.default", "layer_extrusion_edit.fan.cooling",
             "layer_extrusion_edit.fan.first_layers", "layer_extrusion_edit.fan.dynamic"})) ||
            activation_config_changed;
        activation_config_changed = (user_configuration_available && migrate_plugin_activation_id(
            plugin_config, "ordering.plan_builder.default", {"ordering.plan_builder.by_layer"})) ||
            activation_config_changed;
        if (ensure_plugin_activation_config(boost::filesystem::path(), default_plugin_config,
                                            ignored_from_user_config, plugin_config_error)) {
            default_plugin_config_available = true;
            if (user_configuration_available) {
                for (const auto &[plugin_id, enabled] : default_plugin_config.activated)
                    if (enabled && plugin_config.activated.find(plugin_id) == plugin_config.activated.end()) {
                        plugin_config.activated.emplace(plugin_id, true);
                        activation_config_changed = true;
                    }
                for (const auto &[plugin_id, package_id] : default_plugin_config.plugin_packages)
                    if (plugin_config.plugin_packages.emplace(plugin_id, package_id).second)
                        activation_config_changed = true;
            }
        } else {
            BOOST_LOG_TRIVIAL(warning) << plugin_config_error;
        }
        const bool packages_prepared = prepare_plugin_bundle_cache(
            boost::filesystem::path(resources_dir()), config_dir, plugin_config, plugin_config_error);
        if (packages_prepared && default_plugin_config_available) {
            activation_config_changed = activation_config_changed ||
                plugin_config.bundled_exact != default_plugin_config.bundled_exact;
            plugin_config.bundled_exact = default_plugin_config.bundled_exact;
            for (const auto &[package_name, version] : plugin_config.bundled_exact) {
                std::string validation_error;
                if (bundled_exact_plugin_package_is_valid(
                        config_dir, package_name, version, validation_error)) {
                    const std::map<std::string, PluginInstalledVersion>::const_iterator selected =
                        plugin_config.installed.find(package_name);
                    activation_config_changed = activation_config_changed ||
                        selected == plugin_config.installed.end() || selected->second != version;
                    plugin_config.installed[package_name] = version;
                } else {
                    activation_config_changed = activation_config_changed ||
                        plugin_config.installed.count(package_name) != 0;
                    plugin_config.installed.erase(package_name);
                    BOOST_LOG_TRIVIAL(error) << validation_error;
                }
            }
            std::vector<std::string> exact_diagnostics;
            std::string exact_error;
            if (!reconcile_bundled_exact_plugin_packages(
                    config_dir, plugin_config.bundled_exact, exact_diagnostics, exact_error) &&
                !exact_error.empty())
                BOOST_LOG_TRIVIAL(error) << exact_error;
            for (const std::string &diagnostic : exact_diagnostics)
                BOOST_LOG_TRIVIAL(error) << diagnostic;
        }
        const bool packages_applied = packages_prepared && user_configuration_available &&
            reconcile_installed_plugin_packages(config_dir, plugin_config, plugin_config_error);
        if (!packages_applied)
            if (user_configuration_available)
                BOOST_LOG_TRIVIAL(warning) << plugin_config_error;
        if (activation_config_changed && automatic_configuration_writes_allowed &&
            !write_plugin_activation_config(plugin_activation_config_path(config_dir),
                                            plugin_config, plugin_config_error))
            BOOST_LOG_TRIVIAL(warning) << plugin_config_error;
    }

    register_builtin_plugins(orchestrator_handle_ptr);
    if (!config_dir.empty())
        load_plugins_from_repository_impl(
            config_dir / "plugins", orchestrator_handle_ptr,
            default_plugin_config_available ? &default_plugin_config.bundled_exact : nullptr);

    register_builtin_plugin(orchestrator_handle_ptr, "NematX modules", register_nematx_builtin_plugins);

    // A successfully registered external plugin is the authoritative source
    // for its provider association. Backfill old profiles when that relation
    // was not yet persisted, without replacing an explicit existing mapping.
    bool learned_package_association = false;
    if (user_configuration_available) {
        for (const Plugin *plugin : orchestrator.registered_plugins()) {
            if (plugin->get_package_root().empty() ||
                plugin_config.activated.find(plugin->get_id()) == plugin_config.activated.end() ||
                plugin_config.plugin_packages.find(plugin->get_id()) != plugin_config.plugin_packages.end())
                continue;
            const std::string package_id = boost::filesystem::path(plugin->get_package_root()).filename().string();
            if (!package_id.empty()) {
                plugin_config.plugin_packages[plugin->get_id()] = package_id;
                learned_package_association = true;
            }
        }
    }
    if (learned_package_association && automatic_configuration_writes_allowed && !config_dir.empty() &&
        !write_plugin_activation_config(plugin_activation_config_path(config_dir),
                                        plugin_config, plugin_config_error))
        BOOST_LOG_TRIVIAL(warning) << plugin_config_error;

    // Publish the final in-memory value only through the deferred diagnostic.
    // The original partial INI remains byte-identical until the user chooses
    // whether this sanitized value should become durable.
    if (activation_config_source == PluginActivationConfigSource::UserSanitized &&
        g_plugin_activation_startup_error.has_value())
        g_plugin_activation_startup_error->sanitized_config = plugin_config;

    // Loading and activation are intentionally separate. A disabled plugin is
    // still registered so the configuration dialog can show it, but it cannot
    // publish settings or run until its id appears in activated.ini.
    std::vector<std::string> active_plugin_ids;
    for (const auto &[plugin_id, is_enabled] : plugin_config.activated)
        if (is_enabled)
            active_plugin_ids.push_back(plugin_id);
    BOOST_LOG_TRIVIAL(info) << "Loaded " << active_plugin_ids.size() << " active plugin id(s) from "
                            << (user_configuration_available ? plugin_activation_config_path(config_dir).string() :
                                (boost::filesystem::path(resources_dir()) / "plugins/default_activated.ini").string()) << ".";
    activate_plugins_from_ids(orchestrator, active_plugin_ids, user_configuration_available);
    register_exclusive_step_group_options_impl(orchestrator);

    orchestrator.initialize_plugins();
    // Plugins may have recorded deferred option definition patches while they
    // initialized. Apply them before the GUI or presets read the definitions.
    orchestrator.finalize_plugin_option_defs();
    register_exclusive_step_group_ui_fragments_impl(orchestrator);
    //note: --loglevel 4 is read too late for this log, use $env:SLIC3R_LOGLEVEL = "4" (or SLIC3R_LOGLEVEL=4 in visual studio environement line)
    BOOST_LOG_TRIVIAL(debug) << "Loaded the entire plugin library in "
                             << elapsed_ms(start).count() << " ms.";
}

} // namespace Slic3r
