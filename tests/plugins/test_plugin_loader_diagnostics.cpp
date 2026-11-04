///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

// These tests build minimal installed-package directories and pass them to the
// production loader. They verify that failures remain structured data suitable
// for Plugin Package Manager instead of existing only as startup log messages.

#include <catch2/catch.hpp>

#include <string>
#include <limits>
#include <utility>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/nowide/fstream.hpp>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Plugins/PluginLoader.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Utils.hpp"
#include "plugin_test_helpers.hpp"

namespace {
// Write package identity/version files using ABI major 4 under the borrowed test-owned directory; directory errors propagate.
void write_package_metadata(const boost::filesystem::path &package_root, const std::string &package_id);
}

#ifdef SLIC3R_TEST_PYTHON_PLUGINS
// Enable a Python object only after both C queries confirm registration without activation, then initialize and execute it.
// The loader owns the retained object. The test restores previous activation after checking its counters.
TEST_CASE("Python providers registered while inactive survive later activation and initialization",
          "[plugins][python][loader][initialization]")
{
    REQUIRE(Slic3r::Test::Plugins::python_plugin_test_runtime_available());
    Slic3r::Orchestrator &host = Slic3r::Orchestrator::instance();
    orchestrator_handle *handle = reinterpret_cast<orchestrator_handle *>(&host);
    const char *id = "python.test.registration_lifetime";
    const Slic3r::Test::Plugins::PluginTestStartupSets &startup = Slic3r::Test::Plugins::plugin_test_startup_sets();
    REQUIRE(startup.provider_ids.count(id) == 1);
    REQUIRE(startup.active_provider_ids.count(id) == 0);
    Slic3r::Plugin *plugin = host.get_plugin(id);
    REQUIRE(plugin != nullptr);
    std::vector<Slic3r::Plugin *> previous(host.active_plugins().begin(), host.active_plugins().end());
    const Slic3r::ScopeGuard restore([&]() {
        host.clear_active_plugins();
        for (Slic3r::Plugin *saved : previous) host.set_plugin_active(saved, true);
        host.reset_plugin_cancel();
    });
    host.clear_active_plugins();
    CHECK(orchestrator_is_plugin_registered(handle, id) == 1);
    CHECK(orchestrator_find_plugin(handle, id) == nullptr);
    REQUIRE(host.set_plugin_active(plugin, true));
    host.initialize_plugins();
    REQUIRE(host.is_plugin_active(plugin));
    CHECK(orchestrator_find_plugin(handle, id) == reinterpret_cast<const orchestrator_plugin_handle *>(plugin));
    host.reset_plugin_cancel();
    uint32_t counts[2] = {0, 0};
    raw_plugin_run_payload payload = {counts};
    Slic3r::Print print;
    CHECK(host.execute_plugin(*plugin, &print, &payload, 1, true) == RAW_PLUGIN_EXECUTION_SUCCESS);
    CHECK(counts[0] == 1);
    CHECK(counts[1] == 1);
}

// Initialize only the Python plugin that raises and require one application-owned explanation with the plugin disabled.
// Application storage remains borrowed, and the test restores previous activation.
TEST_CASE("Python initializer exceptions disable providers with attributed diagnostics", "[plugins][python][initialization]")
{
    REQUIRE(Slic3r::Test::Plugins::python_plugin_test_runtime_available());
    Slic3r::Orchestrator &host = Slic3r::Orchestrator::instance();
    const char *id = "python.test.initialize.failure";
    REQUIRE(host.get_plugin(id) != nullptr);
    std::vector<Slic3r::Plugin *> previous(host.active_plugins().begin(), host.active_plugins().end());
    const Slic3r::ScopeGuard restore([&]() {
        host.clear_active_plugins();
        for (Slic3r::Plugin *saved : previous) host.set_plugin_active(saved, true);
        host.reset_plugin_cancel();
    });
    host.clear_active_plugins();
    REQUIRE(host.set_plugin_active(id, true));
    host.consume_plugin_messages();
    CHECK_NOTHROW(host.initialize_plugins());
    CHECK_FALSE(host.is_plugin_active(id));
    size_t attributed = 0;
    for (const Slic3r::Orchestrator::PluginMessage &message : host.consume_plugin_messages())
        if (message.plugin_id == id) {
            ++attributed;
            CHECK(message.level == Slic3r::Orchestrator::PluginMessageLevel::Error);
            CHECK(message.step == STEP_POST_SLICING);
            CHECK(message.message.find("Python initializer deliberate failure") != std::string::npos);
        }
    CHECK(attributed == 1);
}

// Inspect saved reports and copied preferences from the embedded Python loader without registering more objects.
// Activation and settings are unchanged. Both invalid packages must fail before exposing a plugin.
TEST_CASE("Python loader bounds group preferences and defaults missing attributes",
          "[plugins][python][loader][abi][selector-group-priority]")
{
    REQUIRE(Slic3r::Test::Plugins::python_plugin_test_runtime_available());
    Slic3r::Orchestrator &host = Slic3r::Orchestrator::instance();
    const std::pair<const char *, int32_t> cases[] = {
        {"python.test.metadata.absent", 100},
        {"python.test.metadata.minimum", (std::numeric_limits<int32_t>::min)()},
        {"python.test.metadata.maximum", (std::numeric_limits<int32_t>::max)()}
    };
    for (const std::pair<const char *, int32_t> &entry : cases) {
        const Slic3r::Plugin *plugin = host.get_plugin(entry.first);
        REQUIRE(plugin != nullptr);
        CHECK(plugin->get_exclusive_group_priority() == entry.second);
    }
    for (bool upper : {false, true}) {
        const Slic3r::PluginPackageLoadReport &report = Slic3r::Test::Plugins::python_priority_package_report(upper);
        CHECK(report.state == Slic3r::PluginPackageLoadState::Failed);
        CHECK(report.registered_plugin_ids.empty());
        REQUIRE(report.issues.size() == 1);
        CHECK(report.issues.front().code == Slic3r::PluginPackageLoadErrorCode::PythonRegistrationFailed);
        const std::string id = upper ? "python.test.priority.high" : "python.test.priority.low";
        CHECK(report.issues.front().detail.find(id) != std::string::npos);
        CHECK(report.issues.front().detail.find("signed 32-bit integer") != std::string::npos);
        CHECK(host.get_plugin(id) == nullptr);
    }
}

// Inspect saved application-log text for one warning per supplied obsolete attribute and none for omitted attributes.
// Saved text survives report clearing. The registered objects must remain loadable without implicit group descriptions.
TEST_CASE("Python loader warns once for supplied obsolete group text slots",
          "[plugins][python][loader][diagnostics][selector-traits]")
{
    REQUIRE(Slic3r::Test::Plugins::python_plugin_test_runtime_available());
    const std::string &warnings = Slic3r::Test::Plugins::python_metadata_warnings();
    Slic3r::Orchestrator &host = Slic3r::Orchestrator::instance();
    for (const char *case_name : {"label", "tooltip", "positional", "positional_label", "keyword", "absent"}) {
        const std::string id = std::string("python.test.metadata.") + case_name;
        const Slic3r::Plugin *plugin = host.get_plugin(id);
        REQUIRE(plugin != nullptr);
        CHECK(plugin->get_exclusive_group().empty());
        for (const char *attribute : {"exclusive_group_label", "exclusive_group_tooltip"}) {
            const std::string diagnostic = "Python package 'python.external.package_root', plugin '" + id +
                "': obsolete attribute '" + attribute + "'";
            const bool expected = std::string(case_name) == "positional" || std::string(case_name) == "keyword" ||
                (std::string(case_name) == "positional_label" && std::string(attribute) == "exclusive_group_label") ||
                (std::string(case_name) == "label" && std::string(attribute) == "exclusive_group_label") ||
                (std::string(case_name) == "tooltip" && std::string(attribute) == "exclusive_group_tooltip");
            const size_t found = warnings.find(diagnostic);
            CHECK((found != std::string::npos) == expected);
            if (expected && found != std::string::npos)
                CHECK(warnings.find(diagnostic, found + diagnostic.size()) == std::string::npos);
        }
    }
}

TEST_CASE("Python loader rejects an initially noncallable clear hook without losing valid plugins",
          "[plugins][python][clear][loader][diagnostics]")
{
    REQUIRE(Slic3r::Test::Plugins::python_plugin_test_runtime_available());
    const Slic3r::PluginPackageLoadReport &report =
        Slic3r::Test::Plugins::python_invalid_clear_package_report();
    CHECK(report.state == Slic3r::PluginPackageLoadState::Failed);
    CHECK(report.registered_plugin_ids.empty());
    REQUIRE(report.issues.size() == 1);
    CHECK(report.issues.front().code == Slic3r::PluginPackageLoadErrorCode::PythonRegistrationFailed);
    CHECK(report.issues.front().detail ==
          "Python plugin 'python.test.clear.invalid_registration' clear attribute is not callable.");
    Slic3r::Orchestrator &orchestrator = Slic3r::Orchestrator::instance();
    CHECK(orchestrator.get_plugin("python.test.clear.invalid_registration") == nullptr);
    CHECK(orchestrator.get_plugin("python.external.package_root") != nullptr);
    CHECK(orchestrator.get_plugin("python.test.clear.noncallable") != nullptr);
    CHECK(orchestrator.get_plugin("python.polyholes") != nullptr);
}
#endif

namespace {

const char *plugin_library_filename()
{
#ifdef _WIN32
    return "plugin.dll";
#elif defined(__APPLE__)
    return "plugin.dylib";
#else
    return "plugin.so";
#endif
}

void write_package_metadata(const boost::filesystem::path &package_root,
                            const std::string &package_id)
{
    boost::filesystem::create_directories(package_root);
    {
        boost::nowide::ofstream stream((package_root / "description.ini").string());
        stream << "[plugin]\nid = " << package_id << "\nname = " << package_id
               << "\nfull_name = " << package_id << "\nslicer = SuperSlicer\n";
    }
    {
        boost::nowide::ofstream stream((package_root / "version.ini").string());
        stream << "[plugin]\npackage_version = 1.0.0\nslicer_version = 2.7.63.0\n"
                   "[abi]\nslic3r_plugin_types.h = 4.0\n";
    }
}

const Slic3r::PluginPackageLoadReport &load_fixture(const boost::filesystem::path &library,
                                                    const std::string &package_id)
{
    const boost::filesystem::path repository = boost::filesystem::temp_directory_path() /
        boost::filesystem::unique_path("slic3r-plugin-loader-%%%%-%%%%");
    const boost::filesystem::path package_root = repository / package_id;
    write_package_metadata(package_root, package_id);
    boost::filesystem::copy_file(library, package_root / plugin_library_filename());

    Slic3r::Orchestrator &orchestrator = Slic3r::Orchestrator::instance();
    orchestrator.clear_plugin_package_load_reports();
    Slic3r::load_plugin_packages_from_repository(repository, orchestrator);
    const Slic3r::PluginPackageLoadReport *report = orchestrator.plugin_package_load_report(package_id);
    REQUIRE(report != nullptr);

    // Failed libraries are unloaded before the loader returns, so the fixture
    // repository can be removed immediately even on Windows.
    boost::filesystem::remove_all(repository);
    return *report;
}

} // namespace

TEST_CASE("Plugin loader reports a missing registration export",
          "[plugins][loader][diagnostics]")
{
    const Slic3r::PluginPackageLoadReport &report = load_fixture(
        boost::filesystem::path(SLIC3R_TEST_PLUGIN_MISSING_REGISTER_DLL), "diagnostic.no_register");
    REQUIRE(report.issues.size() == 1);
    CHECK(report.issues.front().code == Slic3r::PluginPackageLoadErrorCode::MissingRegistrationExport);
}

TEST_CASE("Plugin loader refuses a missing ABI section before opening a binary",
          "[plugins][loader][diagnostics][abi]")
{
    const boost::filesystem::path repository = boost::filesystem::temp_directory_path() /
        boost::filesystem::unique_path("slic3r-plugin-abi-%%%%-%%%%");
    const boost::filesystem::path package = repository / "missing.abi";
    write_package_metadata(package, "missing.abi");
    {
        boost::nowide::ofstream stream((package / "version.ini").string());
        stream << "[plugin]\npackage_version=1.0.0\nslicer_version=1.0.0\n";
    }
    {
        boost::nowide::ofstream stream((package / plugin_library_filename()).string());
        stream << "not a library";
    }
    Slic3r::Orchestrator &host = Slic3r::Orchestrator::instance();
    host.clear_plugin_package_load_reports();
    Slic3r::load_plugin_packages_from_repository(repository, host);
    const Slic3r::PluginPackageLoadReport *report = host.plugin_package_load_report("missing.abi");
    REQUIRE(report != nullptr);
    REQUIRE(report->issues.size() == 1);
    CHECK(report->issues.front().code == Slic3r::PluginPackageLoadErrorCode::ApiHeaderVersionMismatch);
    boost::filesystem::remove_all(repository);
}

TEST_CASE("Plugin loader reports a missing ABI export",
          "[plugins][loader][diagnostics]")
{
    const Slic3r::PluginPackageLoadReport &report = load_fixture(
        boost::filesystem::path(SLIC3R_TEST_PLUGIN_MISSING_ABI_DLL), "diagnostic.no_abi");
    REQUIRE(report.issues.size() == 1);
    CHECK(report.issues.front().code == Slic3r::PluginPackageLoadErrorCode::MissingAbiExport);
}

TEST_CASE("Plugin loader rejects an incompatible API header major version before registration",
          "[plugins][loader][diagnostics]")
{
    const Slic3r::PluginPackageLoadReport &report = load_fixture(
        boost::filesystem::path(SLIC3R_TEST_PLUGIN_BAD_API_MAJOR_DLL),
        "diagnostic.bad_api_major");
    REQUIRE(report.issues.size() == 1);
    CHECK(report.issues.front().code == Slic3r::PluginPackageLoadErrorCode::ApiHeaderVersionMismatch);
    CHECK(report.issues.front().detail.find("slic3r_plugin_types.h") != std::string::npos);
    CHECK(report.issues.front().detail.find("Registration fixture failure") == std::string::npos);
}

// Load a package requiring interface major 3 and require version rejection before its throwing registration function runs.
// The package helper owns cleanup and reports. No incompatible function table enters application execution.
TEST_CASE("Packages built without the group-priority vtable entry are rejected before registration",
          "[plugins][plugin-loader][abi][selector-group-priority]")
{
    const Slic3r::PluginPackageLoadReport &report = load_fixture(
        boost::filesystem::path(SLIC3R_TEST_PLUGIN_ABI3_DLL), "diagnostic.abi3");
    REQUIRE(report.issues.size() == 1);
    CHECK(report.issues.front().code == Slic3r::PluginPackageLoadErrorCode::ApiHeaderVersionMismatch);
    CHECK(report.issues.front().detail.find("slic3r_plugin_types.h") != std::string::npos);
    CHECK(report.issues.front().detail.find("Registration fixture failure") == std::string::npos);
}

TEST_CASE("Plugin loader rejects an API header minor version newer than the host",
          "[plugins][loader][diagnostics]")
{
    const Slic3r::PluginPackageLoadReport &report = load_fixture(
        boost::filesystem::path(SLIC3R_TEST_PLUGIN_BAD_API_MINOR_DLL),
        "diagnostic.bad_api_minor");
    REQUIRE(report.issues.size() == 1);
    CHECK(report.issues.front().code == Slic3r::PluginPackageLoadErrorCode::ApiHeaderVersionMismatch);
}

TEST_CASE("Plugin loader accepts a shorter compatible API header table",
          "[plugins][loader][diagnostics]")
{
    const Slic3r::PluginPackageLoadReport &report = load_fixture(
        boost::filesystem::path(SLIC3R_TEST_PLUGIN_SHORT_API_TABLE_DLL),
        "diagnostic.short_api_table");
    REQUIRE(report.issues.size() == 1);
    CHECK(report.issues.front().code == Slic3r::PluginPackageLoadErrorCode::NoPluginsRegistered);
}

TEST_CASE("Plugin loader accepts unused API headers beyond the host table",
          "[plugins][loader][diagnostics]")
{
    const Slic3r::PluginPackageLoadReport &report = load_fixture(
        boost::filesystem::path(SLIC3R_TEST_PLUGIN_LONG_API_TABLE_DLL),
        "diagnostic.long_api_table");
    REQUIRE(report.issues.size() == 1);
    CHECK(report.issues.front().code == Slic3r::PluginPackageLoadErrorCode::NoPluginsRegistered);
}

TEST_CASE("Plugin loader rejects legacy abi before registration",
          "[plugins][loader][diagnostics]")
{
    const Slic3r::PluginPackageLoadReport &report = load_fixture(
        boost::filesystem::path(SLIC3R_TEST_PLUGIN_LEGACY_ABI_DLL),
        "diagnostic.legacy_abi");
    REQUIRE(report.issues.size() == 1);
    CHECK(report.state == Slic3r::PluginPackageLoadState::Failed);
    CHECK(report.issues.front().code == Slic3r::PluginPackageLoadErrorCode::ApiHeaderVersionMismatch);
    CHECK_FALSE(report.issues.front().detail.empty());
    CHECK(report.issues.front().detail.find("Registration fixture failure") == std::string::npos);
}

TEST_CASE("Plugin loader rejects unknown api before registration",
          "[plugins][loader][diagnostics]")
{
    const Slic3r::PluginPackageLoadReport &report = load_fixture(
        boost::filesystem::path(SLIC3R_TEST_PLUGIN_UNKNOWN_API_DLL),
        "diagnostic.unknown_api");
    REQUIRE(report.issues.size() == 1);
    CHECK(report.state == Slic3r::PluginPackageLoadState::Failed);
    CHECK(report.issues.front().code == Slic3r::PluginPackageLoadErrorCode::ApiHeaderVersionMismatch);
    CHECK_FALSE(report.issues.front().detail.empty());
    CHECK(report.issues.front().detail.find("Registration fixture failure") == std::string::npos);
}

TEST_CASE("Plugin loader rejects zero api table before registration",
          "[plugins][loader][diagnostics]")
{
    const Slic3r::PluginPackageLoadReport &report = load_fixture(
        boost::filesystem::path(SLIC3R_TEST_PLUGIN_ZERO_API_TABLE_DLL),
        "diagnostic.zero_api_table");
    REQUIRE(report.issues.size() == 1);
    CHECK(report.state == Slic3r::PluginPackageLoadState::Failed);
    CHECK(report.issues.front().code == Slic3r::PluginPackageLoadErrorCode::ApiHeaderVersionMismatch);
    CHECK_FALSE(report.issues.front().detail.empty());
    CHECK(report.issues.front().detail.find("Registration fixture failure") == std::string::npos);
}

TEST_CASE("Plugin loader rejects missing vtable before registration",
          "[plugins][loader][diagnostics]")
{
    const Slic3r::PluginPackageLoadReport &report = load_fixture(
        boost::filesystem::path(SLIC3R_TEST_PLUGIN_MISSING_VTABLE_DLL),
        "diagnostic.missing_vtable");
    REQUIRE(report.issues.size() == 1);
    CHECK(report.state == Slic3r::PluginPackageLoadState::Failed);
    CHECK(report.issues.front().code == Slic3r::PluginPackageLoadErrorCode::ApiHeaderVersionMismatch);
    CHECK_FALSE(report.issues.front().detail.empty());
    CHECK(report.issues.front().detail.find("Registration fixture failure") == std::string::npos);
}

TEST_CASE("Plugin loader rejects invalid api size before registration",
          "[plugins][loader][diagnostics]")
{
    const Slic3r::PluginPackageLoadReport &report = load_fixture(
        boost::filesystem::path(SLIC3R_TEST_PLUGIN_INVALID_API_SIZE_DLL),
        "diagnostic.invalid_api_size");
    REQUIRE(report.issues.size() == 1);
    CHECK(report.state == Slic3r::PluginPackageLoadState::Failed);
    CHECK(report.issues.front().code == Slic3r::PluginPackageLoadErrorCode::ApiHeaderVersionMismatch);
    CHECK_FALSE(report.issues.front().detail.empty());
    CHECK(report.issues.front().detail.find("Registration fixture failure") == std::string::npos);
}

TEST_CASE("Plugin loader rejects changing api size before registration",
          "[plugins][loader][diagnostics]")
{
    const Slic3r::PluginPackageLoadReport &report = load_fixture(
        boost::filesystem::path(SLIC3R_TEST_PLUGIN_CHANGING_API_SIZE_DLL),
        "diagnostic.changing_api_size");
    REQUIRE(report.issues.size() == 1);
    CHECK(report.state == Slic3r::PluginPackageLoadState::Failed);
    CHECK(report.issues.front().code == Slic3r::PluginPackageLoadErrorCode::ApiHeaderVersionMismatch);
    CHECK_FALSE(report.issues.front().detail.empty());
    CHECK(report.issues.front().detail.find("Registration fixture failure") == std::string::npos);
}

TEST_CASE("Plugin loader reports a registration exception",
          "[plugins][loader][diagnostics]")
{
    const Slic3r::PluginPackageLoadReport &report = load_fixture(
        boost::filesystem::path(SLIC3R_TEST_PLUGIN_THROW_REGISTER_DLL), "diagnostic.throw_register");
    REQUIRE(report.issues.size() == 1);
    CHECK(report.issues.front().code == Slic3r::PluginPackageLoadErrorCode::RegistrationFailed);
    CHECK(report.issues.front().detail.find("Registration fixture failure") != std::string::npos);
}

TEST_CASE("Plugin loader reports an invalid native library",
          "[plugins][loader][diagnostics]")
{
    const boost::filesystem::path invalid_library = boost::filesystem::temp_directory_path() /
        boost::filesystem::unique_path("invalid-plugin-%%%%.bin");
    {
        boost::nowide::ofstream stream(invalid_library.string(), std::ios::binary);
        stream << "not a dynamic library";
    }
    const Slic3r::PluginPackageLoadReport &report = load_fixture(invalid_library, "diagnostic.invalid_library");
    boost::filesystem::remove(invalid_library);
    REQUIRE(report.issues.size() == 1);
    CHECK(report.issues.front().code == Slic3r::PluginPackageLoadErrorCode::LibraryOpenFailed);
    CHECK_FALSE(report.issues.front().detail.empty());
}

TEST_CASE("Plugin load report distinguishes empty and partial registration",
          "[plugins][loader][diagnostics]")
{
    Slic3r::Orchestrator &orchestrator = Slic3r::Orchestrator::instance();
    orchestrator.clear_plugin_package_load_reports();
    orchestrator.begin_plugin_package_load("diagnostic.empty", "plugins/diagnostic.empty");
    orchestrator.finish_plugin_package_load("diagnostic.empty");
    const Slic3r::PluginPackageLoadReport *empty =
        orchestrator.plugin_package_load_report("diagnostic.empty");
    REQUIRE(empty != nullptr);
    REQUIRE(empty->issues.size() == 1);
    CHECK(empty->issues.front().code == Slic3r::PluginPackageLoadErrorCode::NoPluginsRegistered);

    orchestrator.begin_plugin_package_load("diagnostic.partial", "plugins/diagnostic.partial");
    Slic3r::PluginPackageLoadReport *partial = const_cast<Slic3r::PluginPackageLoadReport *>(
        orchestrator.plugin_package_load_report("diagnostic.partial"));
    REQUIRE(partial != nullptr);
    partial->registered_plugin_ids.push_back("diagnostic.partial.loaded");
    Slic3r::PluginPackageLoadIssue issue;
    issue.code = Slic3r::PluginPackageLoadErrorCode::RegistrationFailed;
    issue.plugin_id = "diagnostic.partial.missing";
    issue.detail = "The second plugin registration failed.";
    orchestrator.report_plugin_package_load_issue("diagnostic.partial", std::move(issue));
    orchestrator.finish_plugin_package_load("diagnostic.partial");
    CHECK(partial->state == Slic3r::PluginPackageLoadState::LoadedWithErrors);
}

TEST_CASE("Secondary runtimes report package errors through the host ABI",
          "[plugins][loader][diagnostics][python]")
{
    Slic3r::Orchestrator &orchestrator = Slic3r::Orchestrator::instance();
    orchestrator.clear_plugin_package_load_reports();
    const std::string package_root = "plugins/diagnostic.python";
    orchestrator_handle *handle = reinterpret_cast<orchestrator_handle *>(&orchestrator);
    orchestrator_begin_plugin_package_load(handle, package_root.c_str());
    orchestrator_report_plugin_package_load_error(
        handle, package_root.c_str(), RAW_PLUGIN_PACKAGE_LOAD_ERROR_PYTHON_COMPILE_FAILED,
        "SyntaxError: invalid syntax\n  File plugin.py, line 3");
    orchestrator_finish_plugin_package_load(handle, package_root.c_str());

    const Slic3r::PluginPackageLoadReport *report =
        orchestrator.plugin_package_load_report("diagnostic.python");
    REQUIRE(report != nullptr);
    REQUIRE(report->issues.size() == 1);
    CHECK(report->issues.front().code == Slic3r::PluginPackageLoadErrorCode::PythonCompileFailed);
    CHECK(report->issues.front().detail.find("SyntaxError") != std::string::npos);
}

TEST_CASE("Python package reports an unavailable runtime",
          "[plugins][loader][diagnostics][python]")
{
    const boost::filesystem::path repository = boost::filesystem::temp_directory_path() /
        boost::filesystem::unique_path("slic3r-python-runtime-missing-%%%%-%%%%");
    const std::string package_id = "diagnostic.python.runtime_missing";
    const boost::filesystem::path package_root = repository / package_id;
    write_package_metadata(package_root, package_id);
    {
        boost::nowide::ofstream stream((package_root / "plugin.py").string());
        stream << "def register_plugin(api):\n    return None\n";
    }

    Slic3r::Orchestrator &orchestrator = Slic3r::Orchestrator::instance();
    orchestrator.clear_plugin_package_load_reports();
    Slic3r::load_plugin_packages_from_repository(repository, orchestrator);

    const Slic3r::PluginPackageLoadReport *report =
        orchestrator.plugin_package_load_report(package_id);
    REQUIRE(report != nullptr);
    REQUIRE(report->issues.size() == 1);
    CHECK(report->issues.front().code ==
          Slic3r::PluginPackageLoadErrorCode::PythonRuntimeUnavailable);
    boost::filesystem::remove_all(repository);
}
