/*
Tests of the STEP_GCODE plugin that runs the post-processing scripts on the generated reference.

Post-processing plugin tests
============================

The plugin "gcode.post_process" runs after the G-code writer inside STEP_GCODE. These tests slice a small cube, let the
plugin start real scripts (a batch file on Windows, a shell script elsewhere) and check, on the reference file and on the
result of the Reader, what the scripts did.

Every script is written during the test into a unique directory below the working directory; the directory is removed at
scope exit. A script receives the path of the temporary copy as its last argument and records what it observed by
appending lines to that copy: a G-code move to prove the Reader reads the processed text, and comment lines for the
environment variables. The expected texts are literals; the text of the reference without any script is the baseline.

Representative direct calls (branches are not a timeline):

    TEST_CASE
        |-- Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized()
        |-- append_line()
        |-- write_script()
        |-- cube.export_reference()
        |-- read_file()
        `-- Test::export_print_to()
*/

#include <catch2/catch.hpp>

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iterator>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/nowide/cenv.hpp>
#include <boost/nowide/fstream.hpp>

#include "gcode_test_helpers.hpp"
#include "gcode_export_test_helpers.hpp"
#include "plugin_test_helpers.hpp"
#include "test_data.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/libslic3r.h"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"



namespace {

using namespace Slic3r;

#ifdef _WIN32
// echo ends its line with CRLF in a batch file.
const char *const k_script_eol = "\r\n";
#else
const char *const k_script_eol = "\n";
#endif

// The move the test scripts append to the file they receive.
const char *const k_appended_move = "G1 X10 Y10 F1000";

// Owns a unique directory below the working directory for the scripts and the exported files.
class ScopedDirectory
{
public:
    ScopedDirectory()
        : m_path(boost::filesystem::current_path() / boost::filesystem::unique_path("slic3r-postprocess-%%%%-%%%%-%%%%"))
    {
        boost::filesystem::create_directories(m_path);
    }

    ~ScopedDirectory()
    {
        boost::system::error_code ignored;
        boost::filesystem::remove_all(m_path, ignored);
    }

    ScopedDirectory(const ScopedDirectory &) = delete;
    ScopedDirectory &operator=(const ScopedDirectory &) = delete;

    const boost::filesystem::path &path() const { return m_path; }

    // The launcher splits a script line on spaces, so the tests only run in a directory without any.
    bool usable_for_scripts() const { return m_path.string().find(' ') == std::string::npos; }

private:
    boost::filesystem::path m_path;
};

// Read one whole file as bytes.
std::string read_file(const boost::filesystem::path &path)
{
    boost::nowide::ifstream input(path.string().c_str(), std::ios::in | std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

// Write the lines of a script, in the line ending of the platform, and make it runnable.
// Returns the path to put in the post_process option.
std::string write_script(const boost::filesystem::path &directory, const std::string &name, const std::vector<std::string> &lines)
{
#ifdef _WIN32
    const boost::filesystem::path path = directory / (name + ".bat");
    const char *eol = "\r\n";
#else
    const boost::filesystem::path path = directory / (name + ".sh");
    const char *eol = "\n";
#endif
    {
        boost::nowide::ofstream output(path.string().c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
        for (const std::string &line : lines)
            output << line << eol;
    }
#ifndef _WIN32
    boost::filesystem::permissions(path, boost::filesystem::owner_all | boost::filesystem::group_read | boost::filesystem::others_read);
#endif
    return path.string();
}

// One line of a script that appends the text to the file the script receives. The parentheses keep a final digit of
// the text from being taken for a redirected handle by the batch interpreter.
std::string append_line(const std::string &text)
{
#ifdef _WIN32
    return "(echo " + text + ")>>\"%~1\"";
#else
    return "echo \"" + text + "\" >> \"$1\"";
#endif
}

// A script prologue. The batch interpreter must not echo its commands.
std::vector<std::string> script_header()
{
#ifdef _WIN32
    return { "@echo off" };
#else
    return { "#!/bin/sh" };
#endif
}

// A script line that ends the script with the exit code.
std::string exit_line(int code)
{
#ifdef _WIN32
    return "exit /b " + std::to_string(code);
#else
    return "exit " + std::to_string(code);
#endif
}

// A script line that exits with the code when the flag file exists.
std::string exit_if_exists_line(const boost::filesystem::path &flag, int code)
{
#ifdef _WIN32
    return "if exist \"" + flag.string() + "\" exit /b " + std::to_string(code);
#else
    return "[ -e \"" + flag.string() + "\" ] && exit " + std::to_string(code);
#endif
}

// A script line that records an environment variable in the file it receives: "; NAME=value" when the variable exists
// (even with an empty value), "; NAME unset" when it does not.
std::string record_env_line(const std::string &name)
{
#ifdef _WIN32
    return "if defined " + name + " ((echo ; " + name + "=%" + name + "%)>>\"%~1\") else ((echo ; " + name + " unset)>>\"%~1\")";
#else
    return "if [ -n \"${" + name + "+x}\" ]; then echo \"; " + name + "=${" + name + "}\" >> \"$1\"; else echo \"; " + name + " unset\" >> \"$1\"; fi";
#endif
}

// Change the activation of a registered plugin for the scope of a test and restore it afterwards.
class ScopedActivation
{
public:
    ScopedActivation(Orchestrator &orchestrator, const char *plugin_id, bool active)
        : m_orchestrator(orchestrator), m_plugin(orchestrator.get_plugin(plugin_id))
    {
        REQUIRE(m_plugin != nullptr);
        m_was_active = orchestrator.is_plugin_active(m_plugin);
        REQUIRE(orchestrator.set_plugin_active(m_plugin, active));
    }

    ~ScopedActivation() { m_orchestrator.set_plugin_active(m_plugin, m_was_active); }

    ScopedActivation(const ScopedActivation &) = delete;
    ScopedActivation &operator=(const ScopedActivation &) = delete;

private:
    Orchestrator &m_orchestrator;
    Plugin *m_plugin;
    bool m_was_active = false;
};

// A printed cube and the means to export its reference. The configuration is applied through Print::apply(), so the
// complete print configuration of the Print holds the post_process option and the witness keys of the tests.
class CubeExport
{
public:
    // The extra settings are applied over a Marlin configuration without seam visibility computation.
    CubeExport(const std::string &post_process,
               const std::vector<std::pair<std::string, std::string>> &settings = {})
    {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.set_deserialize_strict({
            {"gcode_flavor", "marlin2"},
            {"seam_visibility", "0"}});
        config.set_deserialize_strict("post_process", post_process);
        for (const std::pair<std::string, std::string> &setting : settings)
            config.set_deserialize_strict(setting.first, setting.second);
        Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
        print.process();
    }

    // Export the reference, rereading it into the result when asked. Returns the error message, empty on success.
    std::string export_reference(const boost::filesystem::path &reference, GCodeProcessorResult *result)
    {
        Orchestrator::instance().reset_plugin_cancel();
        try {
            Orchestrator::instance().export_gcode(print, reference.string(), result);
        } catch (const std::exception &error) {
            return error.what();
        }
        return std::string();
    }

    Model model;
    Print print;
};

// The reference of a print exported without any script, with the same extra settings, which the tests compare with.
std::string baseline_reference(const boost::filesystem::path &directory,
                               const std::vector<std::pair<std::string, std::string>> &settings = {})
{
    CubeExport cube("", settings);
    const boost::filesystem::path reference = directory / "baseline.gcode";
    REQUIRE(cube.export_reference(reference, nullptr).empty());
    const std::string text = read_file(reference);
    REQUIRE_FALSE(text.empty());
    return text;
}

// The names of the files and directories directly inside the directory.
std::vector<std::string> directory_entries(const boost::filesystem::path &directory)
{
    std::vector<std::string> names;
    for (boost::filesystem::directory_iterator it(directory), end; it != end; ++it)
        names.push_back(it->path().filename().string());
    return names;
}

} // namespace

TEST_CASE("A post-processing script modifies the reference and the Reader sees the change",
          "[plugins][gcode][post-process]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ScopedDirectory directory;
    if (!directory.usable_for_scripts()) {
        WARN("Skipped: the working directory contains a space: " << directory.path().string());
        return;
    }
    const std::string baseline = baseline_reference(directory.path());

    std::vector<std::string> lines = script_header();
    lines.push_back(append_line(k_appended_move));
    lines.push_back(exit_line(0));
    CubeExport cube(write_script(directory.path(), "append", lines));
    const boost::filesystem::path reference = directory.path() / "reference.gcode";
    GCodeProcessorResult result;

    CHECK(cube.export_reference(reference, &result).empty());

    // The reference is the writer's text followed by the move of the script, and nothing else.
    CHECK(read_file(reference) == baseline + k_appended_move + k_script_eol);
    // The Reader runs after the plugin: the last move it reports is the appended one.
    REQUIRE_FALSE(result.moves.empty());
    CHECK(result.moves.back().position.x() == Approx(10.f));
    CHECK(result.moves.back().position.y() == Approx(10.f));
    CHECK(cube.print.is_step_done(STEP_GCODE));
    // The copy of the script is gone; only the reference and the script remain.
    for (const std::string &name : directory_entries(directory.path()))
        CHECK((name == "baseline.gcode" || name == "reference.gcode" || name.find("append") == 0));
}

TEST_CASE("A post-processing script receives the configuration kept in a project, and nothing else",
          "[plugins][gcode][post-process]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ScopedDirectory directory;
    if (!directory.usable_for_scripts()) {
        WARN("Skipped: the working directory contains a space: " << directory.path().string());
        return;
    }
    const std::vector<std::pair<std::string, std::string>> settings = {
        {"layer_height", "0.31"},
        {"printhost_apikey", "SECRET"},
        {"test_config_nature_volatile", "15"},
        {"test_config_nature_preset_meta", "14"}};
    const std::string baseline = baseline_reference(directory.path(), settings);

    // A variable left by an earlier test must not make this one pass: remove the witnesses from the environment.
    const std::vector<std::string> witnesses = {
        "SLIC3R_LAYER_HEIGHT", "SLIC3R_PRINTHOST_APIKEY", "SLIC3R_TEST_CONFIG_NATURE_PRESET_META",
        "SLIC3R_TEST_CONFIG_NATURE_VOLATILE", "SLIC3R_PP_HOST", "SLIC3R_PP_OUTPUT_NAME"};
    for (const std::string &name : witnesses)
        boost::nowide::unsetenv(name.c_str());

    // The script records, as comment lines of the file, whether each witness exists and its value.
    std::vector<std::string> lines = script_header();
    for (const std::string &name : witnesses)
        lines.push_back(record_env_line(name));
    lines.push_back(exit_line(0));
    CubeExport cube(write_script(directory.path(), "environment", lines), settings);
    const boost::filesystem::path reference = directory.path() / "reference.gcode";

    REQUIRE(cube.export_reference(reference, nullptr).empty());

    // Only the keys a project may hold exist: the printer connection key (local), the one-shot key (volatile) and the
    // variables of the former rename protocol do not.
    const std::string expected = std::string("; SLIC3R_LAYER_HEIGHT=0.31") + k_script_eol +
                                 "; SLIC3R_PRINTHOST_APIKEY unset" + k_script_eol +
                                 "; SLIC3R_TEST_CONFIG_NATURE_PRESET_META=14" + k_script_eol +
                                 "; SLIC3R_TEST_CONFIG_NATURE_VOLATILE unset" + k_script_eol +
                                 "; SLIC3R_PP_HOST unset" + k_script_eol +
                                 "; SLIC3R_PP_OUTPUT_NAME unset" + k_script_eol;
    CHECK(read_file(reference) == baseline + expected);
}

TEST_CASE("Without a script line the plugin neither reads nor writes any file",
          "[plugins][gcode][post-process]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ScopedDirectory directory;
    const std::string baseline = baseline_reference(directory.path());

    // Lines of blanks are not scripts.
    CubeExport cube("  ");
    const boost::filesystem::path reference = directory.path() / "reference.gcode";

    CHECK(cube.export_reference(reference, nullptr).empty());

    CHECK(read_file(reference) == baseline);
    CHECK_FALSE(boost::filesystem::exists(reference.string() + ".pp"));
    CHECK(directory_entries(directory.path()).size() == 2);
}

TEST_CASE("A failing post-processing script fails STEP_GCODE and leaves the writer's file untouched",
          "[plugins][gcode][post-process]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ScopedDirectory directory;
    if (!directory.usable_for_scripts()) {
        WARN("Skipped: the working directory contains a space: " << directory.path().string());
        return;
    }
    const std::string baseline = baseline_reference(directory.path());

    // The script modifies its file, then fails while the flag file exists.
    const boost::filesystem::path flag = directory.path() / "fail.flag";
    std::vector<std::string> lines = script_header();
    lines.push_back(append_line(k_appended_move));
    lines.push_back(exit_if_exists_line(flag, 7));
    lines.push_back(exit_line(0));
    CubeExport cube(write_script(directory.path(), "failing", lines));
    const boost::filesystem::path reference = directory.path() / "reference.gcode";
    { boost::nowide::ofstream(flag.string().c_str()) << "x"; }

    (void) Orchestrator::instance().consume_plugin_messages();
    const std::string error = cube.export_reference(reference, nullptr);

    // The step names the plugin; the script's own message is a plugin message.
    INFO(error);
    CHECK(error.find("gcode.post_process") != std::string::npos);
    CHECK(error.find("reported an error") != std::string::npos);
    bool script_failure_reported = false;
    for (const Orchestrator::PluginMessage &message : Orchestrator::instance().consume_plugin_messages())
        if (message.message.find("Error code: 7") != std::string::npos)
            script_failure_reported = true;
    CHECK(script_failure_reported);
    CHECK_FALSE(cube.print.is_step_done(STEP_GCODE));
    // The script worked on a copy: the reference is exactly what the writer produced.
    CHECK(read_file(reference) == baseline);
    CHECK_FALSE(boost::filesystem::exists(reference.string() + ".pp"));

    // The next attempt starts again from the writer: the move appears once.
    boost::filesystem::remove(flag);
    CHECK(cube.export_reference(reference, nullptr).empty());
    CHECK(read_file(reference) == baseline + k_appended_move + k_script_eol);
    CHECK(cube.print.is_step_done(STEP_GCODE));
    CHECK_FALSE(boost::filesystem::exists(reference.string() + ".pp"));
}

TEST_CASE("A file named output_name created by a script is ignored and removed",
          "[plugins][gcode][post-process]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ScopedDirectory directory;
    if (!directory.usable_for_scripts()) {
        WARN("Skipped: the working directory contains a space: " << directory.path().string());
        return;
    }
    const std::string baseline = baseline_reference(directory.path());

    // The script writes the old rename request next to its file.
    std::vector<std::string> lines = script_header();
#ifdef _WIN32
    lines.push_back("(echo renamed.gcode)>\"%~1.output_name\"");
#else
    lines.push_back("echo renamed.gcode > \"$1.output_name\"");
#endif
    lines.push_back(exit_line(0));
    CubeExport cube(write_script(directory.path(), "rename", lines));
    const boost::filesystem::path reference = directory.path() / "reference.gcode";

    CHECK(cube.export_reference(reference, nullptr).empty());

    CHECK(read_file(reference) == baseline);
    CHECK_FALSE(boost::filesystem::exists(reference.string() + ".pp.output_name"));
    CHECK_FALSE(boost::filesystem::exists(reference.string() + ".output_name"));
    CHECK_FALSE(boost::filesystem::exists(directory.path() / "renamed.gcode"));
    CHECK_FALSE(boost::filesystem::exists(reference.string() + ".pp"));
}

TEST_CASE("Exporting a valid reference does not run the scripts again",
          "[plugins][gcode][post-process]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ScopedDirectory directory;
    if (!directory.usable_for_scripts()) {
        WARN("Skipped: the working directory contains a space: " << directory.path().string());
        return;
    }

    // The script counts its runs in a file of its own.
    const boost::filesystem::path counter = directory.path() / "runs.txt";
    std::vector<std::string> lines = script_header();
    lines.push_back(append_line(k_appended_move));
#ifdef _WIN32
    lines.push_back("(echo run)>>\"" + counter.string() + "\"");
#else
    lines.push_back("echo run >> \"" + counter.string() + "\"");
#endif
    lines.push_back(exit_line(0));
    CubeExport cube(write_script(directory.path(), "counting", lines));
    const boost::filesystem::path reference = directory.path() / "reference.gcode";
    REQUIRE(cube.export_reference(reference, nullptr).empty());
    const std::string run_text = std::string("run") + k_script_eol;
    REQUIRE(read_file(counter) == run_text);

    // The exporter publishes copies of the reference: the scripts are not part of it.
    const DynamicPrintConfig config = cube.print.full_print_config();
    const Orchestrator::GCodeExporterSelection selection = Orchestrator::instance().select_gcode_exporter(cube.print, config);
    const boost::filesystem::path first = directory.path() / ("first" + selection.extension);
    const boost::filesystem::path second = directory.path() / ("second" + selection.extension);
    CHECK(Test::export_print_to(Orchestrator::instance(), cube.print, config, reference.string(), first.string(), selection) ==
          first.string());
    CHECK(Test::export_print_to(Orchestrator::instance(), cube.print, config, reference.string(), second.string(), selection) ==
          second.string());

    CHECK(read_file(counter) == run_text);
    CHECK(boost::filesystem::exists(first));
    CHECK(boost::filesystem::exists(second));
}

TEST_CASE("The post-processing plugin cannot run without the G-code writer",
          "[plugins][gcode][post-process]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    const Plugin *plugin = orchestrator.get_plugin("gcode.post_process");
    REQUIRE(plugin != nullptr);
    REQUIRE(orchestrator.is_plugin_active(plugin));
    CHECK(plugin->get_dependencies() == std::vector<std::string>{"gcode.printing_plan_file_writer"});

    const std::string expected_error =
        "Active plugin 'gcode.post_process' requires plugin 'gcode.printing_plan_file_writer', which is inactive.";
    const std::function<bool()> has_expected_error = [&orchestrator, &expected_error]() {
        const std::vector<std::string> errors = orchestrator.active_plugin_dependency_errors();
        return std::find(errors.begin(), errors.end(), expected_error) != errors.end();
    };
    CHECK_FALSE(has_expected_error());
    {
        // The plugin modifies the writer's file: with the writer inactive the dependency is reported.
        const ScopedActivation writer_off(orchestrator, "gcode.printing_plan_file_writer", false);
        CHECK(has_expected_error());
    }
    CHECK_FALSE(has_expected_error());
}
