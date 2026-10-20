/*
Tests cover G-code exporter selection, synchronous QUERY validation, file publication, and per-call failures.

G-code exporter service tests
=============================

The host selects one active FFF exporter from the printer configuration, asks
it synchronously for compatibility and a file suffix, then executes it against
an existing G-code reference. Test providers include one byte-copying exporter
with controllable per-call failures and one raw C++ provider for malformed
QUERY answers. Test files live in unique directories under the current project
working directory and are removed at scope exit; the print pipeline and the
exporter action are checked separately.

Representative direct calls (branches are not a timeline):

    TEST_CASE
        |-- initialize_fff_print()
        |-- print.process()
        |-- orchestrator.export_gcode()
        |-- read_binary_file()
        |-- write_binary_file()
        `-- orchestrator.export_print()
*/

#include <catch2/catch.hpp>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iterator>
#include <map>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <boost/dll/runtime_symbol_info.hpp>
#include <boost/filesystem.hpp>
#include <boost/nowide/fstream.hpp>
#include <boost/process/args.hpp>
#include <boost/process/child.hpp>
#include <boost/process/env.hpp>
#include <boost/process/environment.hpp>
#include <boost/process/exe.hpp>
#include <boost/process/io.hpp>


#include "plugin_test_helpers.hpp"
#include "gcode_export_test_helpers.hpp"
#include "test_data.hpp"

#include "libslic3r/Api/host/ApiHostUtils.hpp"
#include "libslic3r/Api/host/GCodeThumbnailRenderer.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/slic3r_gcode_firmware.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_gcode_export.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_gcode_firmware.h"
#include "libslic3r/Api/plugin/cpp/ConfigViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeExportMetadataViews.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeExporterPlugin.hpp"
#include "libslic3r/Config/ConfigDef.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/GCode.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/libslic3r.h"
#include "libslic3r/Model.hpp"
#include "libslic3r/Plugins/PluginApiCompatibility.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"



namespace {

using namespace Slic3r;

constexpr const char *k_probe_exporter_id = "test.gcode.exporter.probe";
constexpr const char *k_raw_query_exporter_id = "test.gcode.exporter.raw_query";
constexpr const char *k_failing_encoder_id = "test.thumbnail.failing";
constexpr const char *k_pruned_exporter_id = "test.gcode.exporter.pruned";
constexpr const char *k_probe_option_key = "test.gcode_exporter_option";
const char *const k_no_dependencies[] = {nullptr};

enum class ProbeQueryMode { Compatible, Incompatible };
enum class ProbeExportMode { Copy, FailOpen, ReportErrorAfterWrite, CancelAfterWrite, FailingEncoder };
enum class RawQueryMode {
    MissingAnswer,
    RepeatedAnswer,
    ShortResult,
    UnknownStatus,
    NullExtension,
    ForwardSlashExtension,
    BackslashExtension,
    MissingDotExtension,
    IncompatibleWithoutMessage
};

struct ExporterProbeState
{
    ProbeQueryMode query_mode = ProbeQueryMode::Compatible;
    ProbeExportMode export_mode = ProbeExportMode::Copy;
    RawQueryMode raw_query_mode = RawQueryMode::MissingAnswer;
    size_t query_count = 0;
    size_t export_count = 0;
    // Last values seen by the probe: the exporter selector read from the EXPORT config, and the firmware ID given to QUERY.
    std::string export_config_selector;
    std::string query_firmware_id;
    std::vector<std::pair<std::string, std::string>> export_configuration;
    std::vector<std::pair<std::string, std::string>> export_extrusion_widths;
    std::vector<std::pair<std::string, std::string>> export_statistics;
    // Print whose user cancellation the CancelAfterWrite mode raises, as the GUI Cancel button does.
    Print *print_to_cancel = nullptr;
};

ExporterProbeState g_exporter_probe_state;

// Find one exact body occurrence and require its application configuration marker immediately afterward.
size_t expect_body_then_configuration(const std::string &exported, const std::string &body);

// Check the generated header, contiguous width block, exact reference body and filtered configuration footer.
void expect_simple_export(const std::string &exported, const std::string &body);

// Return the text written between one exact body and the configuration marker: empty without statistics, else the statistics lines.
std::string text_between_body_and_configuration(const std::string &exported, const std::string &body);

class ScopedTestDirectory
{
public:
    // Create one unique directory below the current working directory for test-owned output files.
    ScopedTestDirectory()
        : m_path(boost::filesystem::current_path() /
                 boost::filesystem::unique_path("slic3r-gcode-export-%%%%-%%%%-%%%%"))
    {
        boost::filesystem::create_directories(m_path);
    }

    // Remove the test-owned directory without throwing during stack unwinding.
    ~ScopedTestDirectory()
    {
        boost::system::error_code ignored;
        boost::filesystem::remove_all(m_path, ignored);
    }

    // Return the borrowed path owned by this test-directory scope.
    const boost::filesystem::path &path() const { return m_path; }

private:
    boost::filesystem::path m_path;
};

class ScopedPluginActivation
{
public:
    // Activate one registered provider and remember its prior state for restoration.
    ScopedPluginActivation(Orchestrator &orchestrator, const char *plugin_id)
        : m_orchestrator(orchestrator), m_plugin_id(plugin_id)
    {
        Plugin *plugin = m_orchestrator.get_plugin(m_plugin_id);
        REQUIRE(plugin != nullptr);
        m_was_active = m_orchestrator.is_plugin_active(plugin);
        REQUIRE(m_orchestrator.set_plugin_active(m_plugin_id, true));
    }

    // Restore the provider's activation state without throwing during cleanup.
    ~ScopedPluginActivation()
    {
        if (!m_was_active)
            m_orchestrator.set_plugin_active(m_plugin_id, false);
    }

private:
    Orchestrator &m_orchestrator;
    std::string m_plugin_id;
    bool m_was_active = false;
};

// Ask the C selection API for the exporter chosen by a borrowed configuration; return its ID or "<none>".
std::string select_exporter_id_through_c_api(const ConfigBase *config)
{
    orchestrator_handle *handle = reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance());
    const orchestrator_plugin_handle *selected = orchestrator_select_plugin(
        handle, config == nullptr ? nullptr : ApiHost::to_config_handle(config), GCODE_EXPORT, k_gcode_exporter_option_key);
    return selected == nullptr ? std::string("<none>") : std::string(orchestrator_plugin_id(selected));
}

// Write all bytes to a UTF-8 path and fail the current test if opening, writing, or closing fails.
void write_binary_file(const boost::filesystem::path &path, const std::string &contents)
{
    boost::nowide::ofstream output(path.string().c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
    REQUIRE(output.good());
    output.write(contents.data(), static_cast<std::streamsize>(contents.size()));
    output.close();
    REQUIRE(output.good());
}

// Read all bytes from a UTF-8 path and return an owned string; opening failures fail the current test.
std::string read_binary_file(const boost::filesystem::path &path)
{
    boost::nowide::ifstream input(path.string().c_str(), std::ios::in | std::ios::binary);
    REQUIRE(input.good());
    return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

// Borrow exported bytes and body unchanged, validate each complete comment block once, require the exact header/body/configuration boundaries, and reject trailing content; Catch failures identify the first malformed line.
void expect_simple_export(const std::string &exported, const std::string &body)
{
    const std::string header = "; generated by " SLIC3R_APP_KEY " " SLIC3R_VERSION " on\n\n";
    REQUIRE(exported.compare(0, header.size(), header) == 0);
    const size_t body_position = expect_body_then_configuration(exported, body);
    REQUIRE(body_position >= header.size());
    const std::string_view width_block(exported.data() + header.size(), body_position - header.size());
    std::string first_bad_width_line;
    CHECK_MESSAGE(Slic3r::Test::Plugins::GCodeExportTest::is_width_block(width_block, first_bad_width_line),
                  "Malformed extrusion-width line: " << first_bad_width_line);

    const size_t body_end = body_position + body.size();
    const std::string begin_marker = "\n; " SLIC3R_APP_NAME "_config = begin\n";
    const size_t configuration_begin = body_end + begin_marker.size();
    const std::string end_marker = "; " SLIC3R_APP_NAME "_config = end\n";
    const size_t configuration_end = exported.find(end_marker, body_end);
    REQUIRE(configuration_end != std::string::npos);
    REQUIRE(exported.compare(body_end, begin_marker.size(), begin_marker) == 0);
    const std::string_view configuration_block(exported.data() + configuration_begin,
                                               configuration_end - configuration_begin);
    std::string first_bad_configuration_line;
    CHECK_MESSAGE(Slic3r::Test::Plugins::GCodeExportTest::is_configuration_block(
                      configuration_block, first_bad_configuration_line),
                  "Malformed configuration line: " << first_bad_configuration_line);
    CHECK(configuration_end + end_marker.size() == exported.size());
}

// Find the borrowed body exactly once, verify its adjacent configuration marker and return its byte offset; exported bytes stay unchanged and Catch reports boundary failures.
size_t expect_body_then_configuration(const std::string &exported, const std::string &body)
{
    const size_t body_position = exported.find(body);
    REQUIRE(body_position != std::string::npos);
    CHECK(exported.compare(body_position, body.size(), body) == 0);
    CHECK(exported.find(body, body_position + 1) == std::string::npos);
    const std::string begin_marker = "\n; " SLIC3R_APP_NAME "_config = begin\n";
    REQUIRE(exported.compare(body_position + body.size(), begin_marker.size(), begin_marker) == 0);
    return body_position;
}

// Find the borrowed body exactly once and return the owned text that separates it from the configuration marker; Catch reports a missing body or marker.
std::string text_between_body_and_configuration(const std::string &exported, const std::string &body)
{
    const size_t body_position = exported.find(body);
    REQUIRE(body_position != std::string::npos);
    CHECK(exported.find(body, body_position + 1) == std::string::npos);
    const std::string begin_marker = "\n; " SLIC3R_APP_NAME "_config = begin\n";
    const size_t marker_position = exported.find(begin_marker, body_position + body.size());
    REQUIRE(marker_position != std::string::npos);
    return exported.substr(body_position + body.size(), marker_position - body_position - body.size());
}

// Snapshot every file's relative path and bytes so failed exports can prove that the reference and destination directory stayed unchanged.
std::map<std::string, std::string> snapshot_directory(const boost::filesystem::path &root)
{
    std::map<std::string, std::string> snapshot;
    for (boost::filesystem::recursive_directory_iterator it(root), end; it != end; ++it) {
        if (!boost::filesystem::is_regular_file(it->path()))
            continue;
        const std::string relative_path = it->path().lexically_relative(root).generic_string();
        snapshot.emplace(relative_path, read_binary_file(it->path()));
    }
    return snapshot;
}

// Initialize a small FFF print with an explicit firmware and exporter selection; processing remains the caller's choice.
// The optional settings are applied last, so they can replace the defaults chosen here.
void initialize_fff_print(Print &print, Model &model, const std::string &exporter_id,
                          const std::vector<std::pair<std::string, std::string>> &settings = {})
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_extruders(1);
    config.set_deserialize_strict({
        {"step_gcode_plugin", "gcode.printing_plan_file_writer"},
        {"gcode_firmware_plugin", "gcode.firmware.marlin2"},
        {"binary_gcode", "1"}});
    config.set_deserialize_strict("gcode_exporter_plugin", exporter_id);
    for (const std::pair<std::string, std::string> &setting : settings)
        config.set_deserialize_strict(setting.first, setting.second);
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
}

class ProbeExporter final : public slic3r_api::GCodeExporterPluginBase
{
public:
    // Bind the test provider to the process-wide orchestrator used by plugin tests.
    explicit ProbeExporter(orchestrator_handle *orchestrator)
        : GCodeExporterPluginBase(orchestrator)
    {}

private:
    // Return the stable test exporter identity.
    const char *id_impl() const noexcept override { return k_probe_exporter_id; }
    // Label the provider in the generated printer selector.
    const char *name_impl() const noexcept override { return "Exporter probe"; }
    // Describe the test provider's selectable byte-copy behavior.
    const char *description_impl() const noexcept override { return "Exercises G-code exporter host calls."; }
    // Use no dependencies so selection tests isolate the exporter service.
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }
    // Keep the provider at the default selection priority.
    int32_t priority_impl() const noexcept override { return 0; }

    // Declare one FFF setting whose host-resolved invalidation is checked by the tests.
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override
    {
        if (keys != nullptr)
            keys[0] = raw_used_config_key{
                k_probe_option_key, RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_FFF_PRINTER};
        return 1;
    }

    // Identify the test setting as owned by this exporter.
    int32_t defined_config_keys(const char **keys) const noexcept override
    {
        if (keys != nullptr)
            keys[0] = k_probe_option_key;
        return 1;
    }

    // Register the printer-profile setting with a pipeline invalidation witness for host resolution.
    void inilialize_impl(storage_handle *) const override
    {
        raw_config_option_def definition = raw_config_option_def_init();
        definition.opt_key = k_probe_option_key;
        definition.type = RAW_CO_BOOL;
        definition.container_type = RAW_CONTAINER_TYPE_PROJECT;
        definition.option_preset_type = RAW_PRESET_TYPE_FFF_PRINTER;
        definition.printer_technology = RAW_PT_FFF;
        definition.invalidates_step = STEP_GCODE;
        definition.category = RAW_OPTION_CATEGORY_OUTPUT;
        definition.label = "Test exporter option";
        definition.tooltip = "Exercises exporter-specific profile settings.";
        definition.mode = RAW_CONFIG_OPTION_MODE_EXPERT;
        definition.default_serialized_value = "0";
        if (orchestrator_create_option_def(m_orchestrator, &definition) != OPTION_DEF_ERROR_OK)
            throw std::runtime_error("Cannot register the test G-code exporter option.");
    }

    // Return owned compatibility status and suffix for the active test mode; record the call count and the firmware ID received, and throw nothing.
    slic3r_api::GCodeExportQuery query_impl(const slic3r_api::Config &, std::string_view firmware_plugin_id) const override
    {
        ++g_exporter_probe_state.query_count;
        g_exporter_probe_state.query_firmware_id = std::string(firmware_plugin_id);
        slic3r_api::GCodeExportQuery query;
        if (g_exporter_probe_state.query_mode == ProbeQueryMode::Incompatible) {
            query.status = RAW_GCODE_EXPORT_QUERY_INCOMPATIBLE;
            query.message = "probe exporter is incompatible";
            return query;
        }
        query.status = RAW_GCODE_EXPORT_QUERY_COMPATIBLE;
        query.extension = ".probe";
        return query;
    }

    // Copy borrowed metadata strings into probe state, read the UTF-8 reference, and write only the temporary path; I/O errors, selected reports, and cancellation propagate through the host.
    void export_impl(const plugin_run_context &run_context,
                     const slic3r_api::Print &print,
                     const slic3r_api::Config &config,
                     std::string_view reference_path_utf8,
                     std::string_view temporary_output_path_utf8,
                     slic3r_api::GCodeThumbnailRendererView thumbnail_renderer,
                     const raw_gcode_export_metadata *metadata) const override
    {
        ++g_exporter_probe_state.export_count;
        g_exporter_probe_state.export_config_selector = config.get(k_gcode_exporter_option_key).serialize();
        const slic3r_api::GCodeExportMetadataView metadata_view(metadata);
        g_exporter_probe_state.export_configuration.clear();
        g_exporter_probe_state.export_extrusion_widths.clear();
        g_exporter_probe_state.export_statistics.clear();
        const slic3r_api::GCodeExportMetadataListView configuration = metadata_view.configuration();
        for (size_t index = 0; index < configuration.size(); ++index) {
            const slic3r_api::GCodeExportMetadataEntryView entry = configuration.at(index);
            g_exporter_probe_state.export_configuration.emplace_back(std::string(entry.key), std::string(entry.value));
        }
        const slic3r_api::GCodeExportMetadataListView extrusion_widths = metadata_view.extrusion_widths();
        for (size_t index = 0; index < extrusion_widths.size(); ++index) {
            const slic3r_api::GCodeExportMetadataEntryView entry = extrusion_widths.at(index);
            g_exporter_probe_state.export_extrusion_widths.emplace_back(std::string(entry.key), std::string(entry.value));
        }
        const slic3r_api::GCodeExportMetadataListView statistics = metadata_view.statistics();
        for (size_t index = 0; index < statistics.size(); ++index) {
            const slic3r_api::GCodeExportMetadataEntryView entry = statistics.at(index);
            g_exporter_probe_state.export_statistics.emplace_back(std::string(entry.key), std::string(entry.value));
        }
        if (g_exporter_probe_state.export_mode == ProbeExportMode::FailOpen)
            throw std::runtime_error("probe exporter could not open its output");
        if (g_exporter_probe_state.export_mode == ProbeExportMode::FailingEncoder) {
            // The encoder refuses to start: the SDK reports it as an ordinary error of this export.
            const slic3r_api::GCodeThumbnailEncoder encoder(slic3r_api::OrchestratorView(m_orchestrator), print, config,
                                                            thumbnail_renderer.handle(), k_failing_encoder_id);
            return;
        }
        const std::string output_path(temporary_output_path_utf8);
        boost::nowide::ofstream output(output_path.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
        if (!output)
            throw std::runtime_error("probe exporter could not open its output");

        if (g_exporter_probe_state.export_mode != ProbeExportMode::Copy) {
            output.write("partial", 7);
            output.flush();
            output.close();
            if (!output)
                throw std::runtime_error("probe exporter could not close its partial output");
            if (g_exporter_probe_state.export_mode == ProbeExportMode::ReportErrorAfterWrite) {
                if (run_context.report_error == nullptr)
                    throw std::runtime_error("probe exporter has no error callback");
                run_context.report_error(run_context.host_context, "probe exporter reports failure");
            } else {
                g_exporter_probe_state.print_to_cancel->cancel();
            }
            return;
        }

        const std::string reference_path(reference_path_utf8);
        boost::nowide::ifstream input(reference_path.c_str(), std::ios::in | std::ios::binary);
        if (!input)
            throw std::runtime_error("probe exporter could not open its reference");
        std::vector<char> buffer(16384);
        while (true) {
            slic3r_api::throw_if_cancelled(&run_context);
            input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
            const std::streamsize count = input.gcount();
            if (count > 0)
                output.write(buffer.data(), count);
            if (!output || input.bad() || (input.fail() && !input.eof()))
                throw std::runtime_error("probe exporter encountered a file error");
            if (input.eof())
                break;
        }
        output.flush();
        output.close();
        if (!output)
            throw std::runtime_error("probe exporter could not close its output");
    }
};

class RawQueryExporter final : public slic3r_api::PluginBase
{
public:
    // Bind the malformed-answer provider to the process-wide plugin-test orchestrator.
    explicit RawQueryExporter(orchestrator_handle *orchestrator)
        : PluginBase(orchestrator)
    {}

private:
    // Return the stable raw QUERY provider identity.
    const char *id_impl() const noexcept override { return k_raw_query_exporter_id; }
    // Label the raw QUERY provider in the generated profile selector.
    const char *name_impl() const noexcept override { return "Raw QUERY probe"; }
    // Describe the provider's malformed synchronous answers.
    const char *description_impl() const noexcept override { return "Exercises G-code exporter QUERY validation."; }
    // Declare the same selection group as other G-code exporters.
    const char *exclusive_group_impl() const noexcept override { return "gcode_exporter_plugin"; }
    // Identify the provider as a G-code export service.
    slicing_step_t step_impl() const noexcept override { return GCODE_EXPORT; }
    // Use no dependencies for the raw callback probe.
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }
    // Keep the raw provider at default priority.
    int32_t priority_impl() const noexcept override { return 0; }

    // Emit a valid or malformed synchronous result according to the selected test mode.
    void run_impl(const plugin_run_context *run_context) const override
    {
        const run_ctx_gcode_export *context = plugin_ctx_as_gcode_export(run_context);
        if (context == nullptr || context->operation != RAW_GCODE_EXPORT_OPERATION_QUERY ||
            context->emit_query_result == nullptr)
            throw std::runtime_error("raw QUERY probe received an invalid context");
        if (g_exporter_probe_state.raw_query_mode == RawQueryMode::MissingAnswer)
            return;

        raw_gcode_export_query_result result = {};
        result.struct_size = sizeof(result);
        result.status = RAW_GCODE_EXPORT_QUERY_COMPATIBLE;
        result.extension = ".raw";

        switch (g_exporter_probe_state.raw_query_mode) {
        case RawQueryMode::RepeatedAnswer:
            context->emit_query_result(context->query_callback_context, &result);
            context->emit_query_result(context->query_callback_context, &result);
            return;
        case RawQueryMode::ShortResult:
            result.struct_size = uint32_t(offsetof(raw_gcode_export_query_result, message) +
                                           sizeof(result.message) - 1);
            break;
        case RawQueryMode::UnknownStatus:
            result.status = static_cast<raw_gcode_export_query_status>(99);
            break;
        case RawQueryMode::NullExtension:
            result.extension = nullptr;
            break;
        case RawQueryMode::ForwardSlashExtension:
            result.extension = ".bad/path";
            break;
        case RawQueryMode::BackslashExtension:
            result.extension = ".bad\\path";
            break;
        case RawQueryMode::MissingDotExtension:
            result.extension = "invalid";
            break;
        case RawQueryMode::IncompatibleWithoutMessage:
            result.status = RAW_GCODE_EXPORT_QUERY_INCOMPATIBLE;
            result.extension = nullptr;
            result.message = nullptr;
            break;
        case RawQueryMode::MissingAnswer:
            return;
        }
        context->emit_query_result(context->query_callback_context, &result);
    }
};

// Thumbnail encoder whose session cannot be created; it is selected only by an explicit provider ID.
class FailingEncoder final : public slic3r_api::PluginBase
{
public:
    // Bind the failing encoder to the process-wide plugin-test orchestrator.
    explicit FailingEncoder(orchestrator_handle *orchestrator)
        : PluginBase(orchestrator)
    {}

private:
    // Return the stable test encoder identity.
    const char *id_impl() const noexcept override { return k_failing_encoder_id; }
    // Label the test encoder.
    const char *name_impl() const noexcept override { return "Failing thumbnail encoder"; }
    // Describe the encoder whose creation always fails.
    const char *description_impl() const noexcept override { return "Fails when its session is created."; }
    // Identify the provider as a thumbnail encoder service without an exclusive group.
    slicing_step_t step_impl() const noexcept override { return GCODE_THUMBNAIL; }
    // Use no dependencies for the failing encoder.
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }
    // Keep the provider at default priority.
    int32_t priority_impl() const noexcept override { return 0; }
    // Fail instead of publishing an encoder session.
    void run_impl(const plugin_run_context *) const override
    {
        throw std::runtime_error("test encoder cannot create a session");
    }
};

// Registered but never activated, so option finalization prunes it from the exporter selector like any inactive provider.
class PrunedExporter final : public slic3r_api::GCodeExporterPluginBase
{
public:
    // Bind the never-activated provider to the process-wide plugin-test orchestrator.
    explicit PrunedExporter(orchestrator_handle *orchestrator)
        : GCodeExporterPluginBase(orchestrator)
    {}

private:
    // Return the stable identity of the inactive provider.
    const char *id_impl() const noexcept override { return k_pruned_exporter_id; }
    // Label the provider that is absent from the generated selector.
    const char *name_impl() const noexcept override { return "Pruned exporter"; }
    // Describe the provider that stays inactive for the whole run.
    const char *description_impl() const noexcept override { return "Stays inactive to model an uninstalled exporter."; }
    // Use no dependencies so only its inactivity matters.
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }
    // Keep the provider at the default selection priority.
    int32_t priority_impl() const noexcept override { return 0; }
    // Accept every configuration; the provider is never selected for a run.
    slic3r_api::GCodeExportQuery query_impl(const slic3r_api::Config &, std::string_view) const override
    {
        slic3r_api::GCodeExportQuery query;
        query.status = RAW_GCODE_EXPORT_QUERY_COMPATIBLE;
        query.extension = ".pruned";
        return query;
    }
    // Fail loudly if the host ever runs the inactive provider.
    void export_impl(const plugin_run_context &, const slic3r_api::Print &, const slic3r_api::Config &,
                     std::string_view, std::string_view,
                     slic3r_api::GCodeThumbnailRendererView, const raw_gcode_export_metadata *) const override
    {
        throw std::runtime_error("inactive exporter must not run");
    }
};

} // namespace

namespace Slic3r::Test::Plugins {

// Register process-lived exporters before the runtime builds the printer selector and their setting definitions.
void register_gcode_exporter_test_providers(orchestrator_handle *orchestrator_handle_value)
{
    Orchestrator &orchestrator = *reinterpret_cast<Orchestrator *>(orchestrator_handle_value);
    static ProbeExporter probe(orchestrator_handle_value);
    static RawQueryExporter raw_query(orchestrator_handle_value);
    static PrunedExporter pruned(orchestrator_handle_value);
    static FailingEncoder failing_encoder(orchestrator_handle_value);
    if (!orchestrator.register_plugin(probe.c_instance()) ||
        !orchestrator.register_plugin(raw_query.c_instance()) ||
        !orchestrator.register_plugin(pruned.c_instance()) ||
        !orchestrator.register_plugin(failing_encoder.c_instance()))
        throw std::runtime_error("Cannot register G-code exporter test providers.");
}

} // namespace Slic3r::Test::Plugins

// Verify the service IDs, default query, inline accessor, and printer-layout insertion with one active provider.
TEST_CASE("G-code exporter is the visible FFF profile service", "[plugins][gcode][exporter]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();

    CHECK(GCODE_EXPORT == 10500);
    CHECK(SLIC3R_PLUGIN_API_STEP_GCODE_EXPORT == 59);
    const ConfigOptionDef *selector = PrintConfigDef::instance().get("gcode_exporter_plugin");
    REQUIRE(selector != nullptr);
    CHECK(selector->option_preset_type == RAW_PRESET_TYPE_FFF_PRINTER);
    CHECK(selector->invalidates_step == STEP_NONE);
    CHECK(static_cast<bool>(selector->default_value));
    CHECK(selector->default_value->serialize() == "gcode.exporter.simple");

    const std::vector<Plugin *> active = orchestrator.get_active_plugins_for_step(GCODE_EXPORT);
    REQUIRE(active.size() == 1);
    CHECK(active.front()->get_id() == "gcode.exporter.simple");
    Print print;
    const Orchestrator::GCodeExporterSelection selection = orchestrator.select_gcode_exporter(print.full_print_config());
    CHECK(selection.plugin_id == "gcode.exporter.simple");
    CHECK(selection.extension == ".gcode");

    plugin_run_context no_context = {};
    CHECK(plugin_ctx_as_gcode_export(nullptr) == nullptr);
    CHECK(plugin_ctx_as_gcode_export(&no_context) == nullptr);
    int payload_marker = 0;
    no_context.step = GCODE_READER;
    no_context.data = &payload_marker;
    CHECK(plugin_ctx_as_gcode_export(&no_context) == nullptr);
    no_context.step = GCODE_EXPORT;
    CHECK(plugin_ctx_as_gcode_export(&no_context) != nullptr);

    bool found_selector_fragment = false;
    for (const Orchestrator::PluginUiFragment &fragment : orchestrator.ui_fragments_for_file("printer_fff.ui")) {
        if (fragment.fragment_id == "gcode_exporter_plugin") {
            found_selector_fragment = true;
            CHECK(fragment.content.find("insert$aftersetting$gcode_firmware_plugin:gcode_exporter_plugin") !=
                  std::string::npos);
        }
    }
    CHECK(found_selector_fragment);
    const std::string merged_layout = orchestrator.merged_ui_layout(
        "printer_fff.ui",
        "page:General:printer\n"
        "group:Firmware\n"
        "\tsetting:step_gcode_plugin\n"
        "\tsetting:gcode_firmware_plugin\n"
        "\tsetting:gcode_flavor\n");
    const size_t writer_position = merged_layout.find("setting:step_gcode_plugin");
    const size_t firmware_position = merged_layout.find("setting:gcode_firmware_plugin");
    const size_t exporter_position = merged_layout.find("setting:gcode_exporter_plugin");
    const size_t flavor_position = merged_layout.find("setting:gcode_flavor");
    REQUIRE(writer_position != std::string::npos);
    REQUIRE(firmware_position != std::string::npos);
    REQUIRE(exporter_position != std::string::npos);
    REQUIRE(flavor_position != std::string::npos);
    CHECK(writer_position < firmware_position);
    CHECK(firmware_position < exporter_position);
    CHECK(exporter_position < flavor_position);
    for (const Orchestrator::PluginUiFragment &fragment : orchestrator.ui_fragments_for_file("print.ui"))
        CHECK(fragment.fragment_id != "gcode_exporter_plugin");
}

// Preserve the generated reference prefix while appending more than one copy buffer, compare every output byte, and distinguish output-setting changes from G-code invalidation.
TEST_CASE("G-code export copies the reference without rerunning slicing", "[plugins][gcode][exporter]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    ScopedTestDirectory directory;
    Print print;
    Model model;
    initialize_fff_print(print, model, "gcode.exporter.simple");
    print.process();

    const boost::filesystem::path reference = directory.path() / "reference.gcode";
    const boost::filesystem::path first_target = directory.path() / "piece";
    const boost::filesystem::path second_target = directory.path() / "piece.gco";
    const boost::filesystem::path third_target = directory.path() / "piece.g";
    CHECK(orchestrator.export_gcode(print, reference.string(), nullptr) == reference.string());
    std::string expected = read_binary_file(reference);
    const size_t generated_size = expected.size();
    expected.resize(generated_size + 128 * 1024 + 19);
    for (size_t idx = generated_size; idx < expected.size(); ++idx)
        expected[idx] = static_cast<char>((idx * 37u) & 0xffu);
    write_binary_file(reference, expected);

    REQUIRE(print.is_step_done(STEP_GCODE));
    const size_t gcode_timestamp = print.step_state_with_timestamp(STEP_GCODE).timestamp;
    // The sentinel survives only if no export rereads the reference into the print statistics.
    const double sentinel_volume = 4242.5;
    print.print_statistics().total_extruded_volume = sentinel_volume;
    const Orchestrator::GCodeExporterSelection selection = orchestrator.select_gcode_exporter(print.full_print_config());
    const std::string first_path = orchestrator.export_print(print, print.full_print_config(), reference.string(), first_target.string(), selection);
    const std::string second_path = orchestrator.export_print(print, print.full_print_config(), reference.string(), second_target.string(), selection);

    CHECK(first_path == (directory.path() / "piece.gcode").string());
    CHECK(second_path == second_target.string());
    expect_simple_export(read_binary_file(boost::filesystem::path(first_path)), expected);
    expect_simple_export(read_binary_file(second_target), expected);
    CHECK(orchestrator.export_print(print, print.full_print_config(), reference.string(), third_target.string(), selection) == third_target.string());
    expect_simple_export(read_binary_file(third_target), expected);
    CHECK(read_binary_file(reference) == expected);
    const boost::filesystem::path unsupported_target = directory.path() / "wrong.bgcode";
    CHECK_THROWS_WITH(orchestrator.export_print(print, print.full_print_config(), reference.string(), unsupported_target.string(), selection),
                      "The G-code exporter 'gcode.exporter.simple' does not support the destination extension '.bgcode'.");
    CHECK_FALSE(boost::filesystem::exists(unsupported_target));
    CHECK(print.is_step_done(STEP_GCODE));
    CHECK(print.step_state_with_timestamp(STEP_GCODE).timestamp == gcode_timestamp);
    CHECK(print.print_statistics().total_extruded_volume == sentinel_volume);
    CHECK_FALSE(orchestrator.is_plugin_cancelled());

    {
        ScopedPluginActivation probe(orchestrator, k_probe_exporter_id);
        DynamicPrintConfig exporter_config = print.full_print_config();
        exporter_config.set_deserialize_strict("gcode_exporter_plugin", k_probe_exporter_id);
        exporter_config.set_deserialize_strict(k_probe_option_key, "1");
        CHECK(print.apply(model, exporter_config) == PrintBase::APPLY_STATUS_CHANGED);
        // The new values must really be in the Print, not just accepted.
        const ConfigOption *applied_selector = print.full_print_config().option(k_gcode_exporter_option_key);
        REQUIRE(applied_selector != nullptr);
        CHECK(applied_selector->serialize() == k_probe_exporter_id);
        const ConfigOption *applied_option = print.full_print_config().option(k_probe_option_key);
        REQUIRE(applied_option != nullptr);
        CHECK(applied_option->serialize() == "1");
        CHECK(print.is_step_done(STEP_GCODE));

        DynamicPrintConfig firmware_config = print.full_print_config();
        firmware_config.set_deserialize_strict("gcode_exporter_plugin", "gcode.exporter.simple");
        firmware_config.set_deserialize_strict("gcode_firmware_plugin", "gcode.firmware.prusa");
        CHECK(print.apply(model, firmware_config) == PrintBase::APPLY_STATUS_INVALIDATED);
        CHECK_FALSE(print.is_step_done(STEP_GCODE));
    }
}

// Verify partial report_error, cancellation, and incompatible QUERY preserve complete source/destination snapshots and allow a later successful export.
TEST_CASE("G-code export failures preserve files and do not poison later calls", "[plugins][gcode][exporter]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    ScopedPluginActivation probe(orchestrator, k_probe_exporter_id);
    ScopedTestDirectory directory;
    Print print;
    Model model;
    initialize_fff_print(print, model, k_probe_exporter_id);
    const boost::filesystem::path reference = directory.path() / "reference.gcode";
    const boost::filesystem::path destination = directory.path() / "final.probe";
    write_binary_file(reference, std::string("reference bytes\0with tail", 25));
    write_binary_file(destination, "old destination bytes");
    const Orchestrator::GCodeExporterSelection selection = orchestrator.select_gcode_exporter(print.full_print_config());
    REQUIRE(selection.plugin_id == k_probe_exporter_id);
    REQUIRE(selection.extension == ".probe");

    const boost::filesystem::path missing_reference = directory.path() / "missing.gcode";
    const std::map<std::string, std::string> original_files = snapshot_directory(directory.path());
    CHECK_THROWS(orchestrator.export_print(print, print.full_print_config(), missing_reference.string(), destination.string(), selection));
    CHECK(snapshot_directory(directory.path()) == original_files);
    g_exporter_probe_state.export_mode = ProbeExportMode::Copy;
    CHECK(orchestrator.export_print(print, print.full_print_config(), reference.string(), destination.string(), selection) == destination.string());
    CHECK(read_binary_file(destination) == read_binary_file(reference));
    write_binary_file(destination, "old destination bytes");
    CHECK(snapshot_directory(directory.path()) == original_files);

    g_exporter_probe_state.export_mode = ProbeExportMode::FailOpen;
    CHECK_THROWS_WITH(orchestrator.export_print(print, print.full_print_config(), reference.string(), destination.string(), selection),
                      "probe exporter could not open its output");
    CHECK(snapshot_directory(directory.path()) == original_files);
    CHECK_FALSE(orchestrator.is_plugin_cancelled());

    g_exporter_probe_state.export_mode = ProbeExportMode::ReportErrorAfterWrite;
    CHECK_THROWS_WITH(orchestrator.export_print(print, print.full_print_config(), reference.string(), destination.string(), selection),
                      "probe exporter reports failure");
    CHECK(snapshot_directory(directory.path()) == original_files);
    CHECK_FALSE(orchestrator.is_plugin_cancelled());

    g_exporter_probe_state.export_mode = ProbeExportMode::Copy;
    CHECK(orchestrator.export_print(print, print.full_print_config(), reference.string(), destination.string(), selection) == destination.string());
    CHECK(read_binary_file(destination) == read_binary_file(reference));
    write_binary_file(destination, "old destination bytes");
    CHECK(snapshot_directory(directory.path()) == original_files);

    // The user cancels the Print while the exporter runs; the global plugin flag is never raised.
    g_exporter_probe_state.export_mode = ProbeExportMode::CancelAfterWrite;
    g_exporter_probe_state.print_to_cancel = &print;
    CHECK_THROWS_WITH(orchestrator.export_print(print, print.full_print_config(), reference.string(), destination.string(), selection),
                      "G-code export was cancelled.");
    g_exporter_probe_state.print_to_cancel = nullptr;
    print.restart();
    CHECK(snapshot_directory(directory.path()) == original_files);
    CHECK_FALSE(orchestrator.is_plugin_cancelled());

    g_exporter_probe_state.export_mode = ProbeExportMode::Copy;
    CHECK(orchestrator.export_print(print, print.full_print_config(), reference.string(), destination.string(), selection) == destination.string());
    CHECK(read_binary_file(destination) == read_binary_file(reference));
    write_binary_file(destination, "old destination bytes");
    CHECK(snapshot_directory(directory.path()) == original_files);

    g_exporter_probe_state.query_mode = ProbeQueryMode::Incompatible;
    CHECK_THROWS_WITH(orchestrator.export_print(print, print.full_print_config(), reference.string(), destination.string(), selection),
                      "probe exporter is incompatible");
    CHECK(snapshot_directory(directory.path()) == original_files);
    CHECK_FALSE(orchestrator.is_plugin_cancelled());

    g_exporter_probe_state.query_mode = ProbeQueryMode::Compatible;
    g_exporter_probe_state.export_mode = ProbeExportMode::Copy;
    CHECK(orchestrator.export_print(print, print.full_print_config(), reference.string(), destination.string(), selection) == destination.string());
    CHECK(read_binary_file(destination) == read_binary_file(reference));
    CHECK_FALSE(orchestrator.is_plugin_cancelled());
}

// Exercise CLI destination resolution for automatic, directory and explicit paths while keeping every artifact below the test directory.
TEST_CASE("CLI G-code export publishes the requested exporter extension", "[plugins][gcode][exporter]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ScopedTestDirectory directory;
    Print print;
    Model model;
    initialize_fff_print(print, model, "gcode.exporter.simple");
    model.objects().front().name = (directory.path() / "auto-name.stl").string();
    CHECK(print.apply(model, print.full_print_config()) != PrintBase::APPLY_STATUS_INVALIDATED);
    print.process();

    const std::string automatic_path = Orchestrator::instance().export_gcode_cli(print, std::string());
    const boost::filesystem::path automatic_file(automatic_path);
    CHECK(automatic_file.parent_path() == directory.path());
    CHECK(automatic_file.extension().string() == ".gcode");
    const std::string expected = read_binary_file(automatic_file);
    // The exported file adds one versioned prefix and filtered trailer around the writer reference; the CLI rereads its
    // reference, so a statistics block sits between the reference and the configuration.
    const boost::filesystem::path direct_reference = directory.path() / "direct-reference.gcode";
    Orchestrator::instance().export_gcode(print, direct_reference.string(), nullptr);
    const std::string direct_body = read_binary_file(direct_reference);
    const std::string statistics_block = text_between_body_and_configuration(expected, direct_body);
    std::string first_bad_line;
    REQUIRE_FALSE(statistics_block.empty());
    CHECK_MESSAGE(Slic3r::Test::Plugins::GCodeExportTest::is_statistics_block(statistics_block, first_bad_line),
                  "Malformed statistics line: " << first_bad_line);
    expect_simple_export(expected, direct_body + statistics_block);

    const boost::filesystem::path output_directory = directory.path() / "café output";
    boost::filesystem::create_directories(output_directory);
    const std::string directory_path = Orchestrator::instance().export_gcode_cli(print, output_directory.string());
    CHECK(boost::filesystem::path(directory_path).parent_path() == output_directory);
    CHECK(read_binary_file(boost::filesystem::path(directory_path)) == expected);

    const boost::filesystem::path explicit_path = directory.path() / "café piece.gcode";
    const std::string explicit_export = Orchestrator::instance().export_gcode_cli(print, explicit_path.string());
    CHECK(explicit_export == explicit_path.string());
    CHECK(read_binary_file(explicit_path) == expected);

    const boost::filesystem::path suffixless_path = directory.path() / "café suffixless";
    const boost::filesystem::path suffixless_expected = suffixless_path.parent_path() /
        boost::filesystem::path(suffixless_path.filename().string() + ".gcode");
    const std::string suffixless_export = Orchestrator::instance().export_gcode_cli(print, suffixless_path.string());
    CHECK(suffixless_export == suffixless_expected.string());
    CHECK(read_binary_file(suffixless_expected) == expected);

    const std::map<std::string, std::string> files_before_rejection = snapshot_directory(directory.path());
    const boost::filesystem::path unsupported_path = directory.path() / "unsupported.bgcode";
    CHECK_THROWS_WITH(Orchestrator::instance().export_gcode_cli(print, unsupported_path.string()),
                      "The G-code exporter 'gcode.exporter.simple' does not support the destination extension '.bgcode'.");
    CHECK(snapshot_directory(directory.path()) == files_before_rejection);

    const std::map<std::string, std::string> files = snapshot_directory(directory.path());
    for (const std::pair<const std::string, std::string> &file : files)
        CHECK(file.first.find(".slic3r-cli-reference-") == std::string::npos);
}

// Reject absent/repeated callbacks, short results, unsupported statuses, and invalid compatibility fields before any output action.
TEST_CASE("G-code exporter QUERY rejects malformed synchronous answers", "[plugins][gcode][exporter]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    ScopedPluginActivation raw_query(orchestrator, k_raw_query_exporter_id);
    Print print;
    Model model;
    initialize_fff_print(print, model, k_raw_query_exporter_id);
    ScopedTestDirectory directory;
    const std::map<std::string, std::string> files_before = snapshot_directory(directory.path());
    const size_t exports_before = g_exporter_probe_state.export_count;

    g_exporter_probe_state.raw_query_mode = RawQueryMode::MissingAnswer;
    CHECK_THROWS_WITH(orchestrator.select_gcode_exporter(print.full_print_config()), "G-code exporter QUERY did not return a result.");
    g_exporter_probe_state.raw_query_mode = RawQueryMode::RepeatedAnswer;
    CHECK_THROWS_WITH(orchestrator.select_gcode_exporter(print.full_print_config()), "G-code exporter QUERY returned more than one result.");
    g_exporter_probe_state.raw_query_mode = RawQueryMode::ShortResult;
    CHECK_THROWS_WITH(orchestrator.select_gcode_exporter(print.full_print_config()),
                      "G-code exporter QUERY result is smaller than the version 1.0 prefix.");
    g_exporter_probe_state.raw_query_mode = RawQueryMode::UnknownStatus;
    CHECK_THROWS_WITH(orchestrator.select_gcode_exporter(print.full_print_config()), "G-code exporter QUERY returned an unknown status.");
    g_exporter_probe_state.raw_query_mode = RawQueryMode::NullExtension;
    CHECK_THROWS_WITH(orchestrator.select_gcode_exporter(print.full_print_config()), "G-code exporter QUERY returned a null extension.");
    g_exporter_probe_state.raw_query_mode = RawQueryMode::ForwardSlashExtension;
    CHECK_THROWS_WITH(orchestrator.select_gcode_exporter(print.full_print_config()), "G-code exporter QUERY returned an invalid extension.");
    g_exporter_probe_state.raw_query_mode = RawQueryMode::BackslashExtension;
    CHECK_THROWS_WITH(orchestrator.select_gcode_exporter(print.full_print_config()), "G-code exporter QUERY returned an invalid extension.");
    g_exporter_probe_state.raw_query_mode = RawQueryMode::MissingDotExtension;
    CHECK_THROWS_WITH(orchestrator.select_gcode_exporter(print.full_print_config()), "G-code exporter QUERY returned an invalid extension.");
    g_exporter_probe_state.raw_query_mode = RawQueryMode::IncompatibleWithoutMessage;
    CHECK_THROWS_WITH(orchestrator.select_gcode_exporter(print.full_print_config()),
                      "G-code exporter QUERY incompatibility has no message.");
    CHECK_FALSE(orchestrator.is_plugin_cancelled());
    // QUERY never reaches EXPORT and never creates a file.
    CHECK(g_exporter_probe_state.export_count == exports_before);
    CHECK(snapshot_directory(directory.path()) == files_before);
}

// Parse a manifest that requires the given headers and tell whether the host accepts it.
static bool manifest_is_compatible(const std::string &abi_lines)
{
    PluginPackageMetadata metadata;
    std::string error;
    REQUIRE(parse_plugin_package_metadata(
        "[plugin]\npackage_version=1.0.0\nslicer_version=1.0.0\n[abi]\nslic3r_plugin_types.h=4.1\n" + abi_lines,
        metadata, error));
    return metadata.compatibility.compatible();
}

// The ABI registry exposes the exporter header at 2.2, accepts older 2.0 and 2.1 manifests, and rejects unsupported majors/minors.
TEST_CASE("G-code exporter header is part of the plugin ABI registry", "[plugins][gcode][exporter][abi]")
{
    const std::vector<PluginApiRequirement> &host = host_plugin_api_contracts();
    REQUIRE(host.size() > size_t(SLIC3R_PLUGIN_API_STEP_GCODE_EXPORT));
    CHECK(host[SLIC3R_PLUGIN_API_STEP_GCODE_EXPORT].header == "steps/slic3r_step_gcode_export.h");
    CHECK(host[SLIC3R_PLUGIN_API_STEP_GCODE_EXPORT].version.major == 2);
    CHECK(host[SLIC3R_PLUGIN_API_STEP_GCODE_EXPORT].version.minor == 2);
    CHECK(manifest_is_compatible("steps/slic3r_step_gcode_export.h=2.0\n"));
    CHECK(manifest_is_compatible("steps/slic3r_step_gcode_export.h=2.1\n"));
    CHECK(manifest_is_compatible("steps/slic3r_step_gcode_export.h=2.2\n"));
    CHECK_FALSE(manifest_is_compatible("steps/slic3r_step_gcode_export.h=2.3\n"));
    CHECK_FALSE(manifest_is_compatible("steps/slic3r_step_gcode_export.h=1.0\n"));
    CHECK_FALSE(manifest_is_compatible("steps/slic3r_step_gcode_export.h=3.0\n"));

    // The firmware context lost its renderer: only a 6.x manifest is accepted.
    CHECK(SLIC3R_PLUGIN_API_STEP_GCODE_FIRMWARE_MAJOR == 6);
    CHECK(manifest_is_compatible("steps/slic3r_step_gcode_firmware.h=6.0\n"));
    CHECK_FALSE(manifest_is_compatible("steps/slic3r_step_gcode_firmware.h=5.0\n"));
    // The firmware table kept its layout and gained comments only, so a writer built against 6.0 still loads.
    CHECK(SLIC3R_PLUGIN_API_GCODE_FIRMWARE_MAJOR == 6);
    CHECK(SLIC3R_PLUGIN_API_GCODE_FIRMWARE_MINOR == 1);
    CHECK(manifest_is_compatible("slic3r_gcode_firmware.h=6.0\n"));
    CHECK(manifest_is_compatible("slic3r_gcode_firmware.h=6.1\n"));
    CHECK_FALSE(manifest_is_compatible("slic3r_gcode_firmware.h=7.0\n"));
}

// Construct short, extended, empty, and malformed ABI 2.1 and 2.2 records; assertions verify borrowed views accept the prefix and reject invalid sizes, lists, and strings without taking ownership.
TEST_CASE("G-code export metadata views validate the ABI 2.1 prefix and 2.2 statistics", "[plugins][gcode][exporter][abi]")
{
    raw_gcode_export_metadata truncated = {};
    truncated.struct_size = uint32_t(offsetof(raw_gcode_export_metadata, extrusion_widths));
    truncated.configuration.count = 1;
    CHECK_THROWS_WITH((slic3r_api::GCodeExportMetadataView(&truncated)),
                      "G-code export metadata has an incomplete 2.1 prefix.");

    struct ExtendedMetadata
    {
        raw_gcode_export_metadata metadata;
        uint64_t extension;
    } extended = {};
    extended.metadata.struct_size = uint32_t(sizeof(extended));
    const slic3r_api::GCodeExportMetadataView extended_view(&extended.metadata);
    CHECK(extended_view.configuration().size() == 0);
    CHECK(extended_view.extrusion_widths().size() == 0);

    raw_gcode_export_metadata invalid_list = {};
    invalid_list.struct_size = uint32_t(sizeof(invalid_list));
    invalid_list.configuration.count = 1;
    CHECK_THROWS_AS((slic3r_api::GCodeExportMetadataView(&invalid_list)), std::invalid_argument);

    const raw_gcode_export_entry null_string_entry = {nullptr, "value"};
    raw_gcode_export_metadata invalid_string = {};
    invalid_string.struct_size = uint32_t(sizeof(invalid_string));
    invalid_string.configuration.items = &null_string_entry;
    invalid_string.configuration.count = 1;
    const slic3r_api::GCodeExportMetadataView invalid_string_view(&invalid_string);
    CHECK_THROWS_AS(invalid_string_view.configuration().at(0), std::invalid_argument);

    // A structure that stops after extrusion_widths (ABI 2.1) has no statistics, even if the bytes after it are not zero.
    const raw_gcode_export_entry statistic = {"has_support", "1"};
    struct StatisticsMetadata
    {
        raw_gcode_export_metadata metadata;
        uint64_t extension;
    } with_statistics = {};
    with_statistics.metadata.statistics.items = &statistic;
    with_statistics.metadata.statistics.count = 1;
    with_statistics.metadata.struct_size = uint32_t(offsetof(raw_gcode_export_metadata, statistics));
    CHECK(slic3r_api::GCodeExportMetadataView(&with_statistics.metadata).statistics().size() == 0);
    with_statistics.metadata.struct_size = uint32_t(sizeof(with_statistics));
    const slic3r_api::GCodeExportMetadataListView statistics_view =
        slic3r_api::GCodeExportMetadataView(&with_statistics.metadata).statistics();
    REQUIRE(statistics_view.size() == 1);
    CHECK(statistics_view.at(0).key == "has_support");
    CHECK(statistics_view.at(0).value == "1");
    raw_gcode_export_metadata invalid_statistics = {};
    invalid_statistics.struct_size = uint32_t(sizeof(invalid_statistics));
    invalid_statistics.statistics.count = 1;
    CHECK_THROWS_AS((slic3r_api::GCodeExportMetadataView(&invalid_statistics)), std::invalid_argument);

    const raw_gcode_export_entry separator = {"", ""};
    raw_gcode_export_metadata valid_empty = {};
    valid_empty.struct_size = uint32_t(sizeof(valid_empty));
    valid_empty.extrusion_widths.items = &separator;
    valid_empty.extrusion_widths.count = 1;
    const slic3r_api::GCodeExportMetadataView valid_empty_view(&valid_empty);
    const slic3r_api::GCodeExportMetadataEntryView entry = valid_empty_view.extrusion_widths().at(0);
    CHECK(entry.key.empty());
    CHECK(entry.value.empty());
}

// Export with a changed config copy and a secret sentinel, import the output, and copy probe-visible metadata during its callback; assertions verify source, ordering, filtering, and region separators.
TEST_CASE("G-code export metadata matches the export configuration", "[plugins][gcode][exporter][abi][config]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    g_exporter_probe_state.query_mode = ProbeQueryMode::Compatible;
    g_exporter_probe_state.export_mode = ProbeExportMode::Copy;
    Orchestrator &orchestrator = Orchestrator::instance();
    ScopedPluginActivation probe(orchestrator, k_probe_exporter_id);
    ScopedTestDirectory directory;
    Print print;
    Model model;
    initialize_fff_print(print, model, "gcode.exporter.simple");
    print.process();

    DynamicPrintConfig exported_config = print.full_print_config();
    exported_config.set_deserialize_strict("layer_height", "0.32");
    exported_config.set_deserialize_strict("printhost_apikey", "E2B_SECRET_SENTINEL");
    // One key of each other nature, with a distinct non-empty value: the LOCAL, PRESET_META and VOLATILE ones must not reach the file.
    exported_config.set_deserialize_strict({{"host_type", "octoprint"},
                                            {"compatible_printers", "E2B_PRESET_META_SENTINEL"},
                                            {"inherits", "E2B_INHERITS_SENTINEL"},
                                            {"default_print_profile", "E2B_DEFAULT_PROFILE_SENTINEL"},
                                            {"print_settings_id", "E2B_BOOKKEEPING_SENTINEL"},
                                            {"test_config_nature_volatile", "15"}});
    const boost::filesystem::path reference = directory.path() / "reference.gcode";
    const boost::filesystem::path exported_file = directory.path() / "exported.gcode";
    write_binary_file(reference, "G1 X1\n");
    const Orchestrator::GCodeExporterSelection simple_selection = orchestrator.select_gcode_exporter(exported_config);
    orchestrator.export_print(print, exported_config, reference.string(), exported_file.string(), simple_selection);

    const std::string exported = read_binary_file(exported_file);
    CHECK(exported.find("; layer_height = 0.32\n") != std::string::npos);
    CHECK(exported.find("E2B_SECRET_SENTINEL") == std::string::npos);
    // Complete lines are compared: "inherits" is a prefix of "inherits_cummulative". The destination of the export is a one-shot setting.
    CHECK(exported.find("\n; host_type = ") == std::string::npos);
    CHECK(exported.find("\n; compatible_printers = ") == std::string::npos);
    CHECK(exported.find("\n; inherits = ") == std::string::npos);
    CHECK(exported.find("\n; default_print_profile = ") == std::string::npos);
    CHECK(exported.find("\n; test_config_nature_volatile = ") == std::string::npos);
    CHECK(exported.find("\n; output_path = ") == std::string::npos);
    CHECK(exported.find("E2B_PRESET_META_SENTINEL") == std::string::npos);
    CHECK(exported.find("E2B_INHERITS_SENTINEL") == std::string::npos);
    CHECK(exported.find("E2B_DEFAULT_PROFILE_SENTINEL") == std::string::npos);
    CHECK(exported.find("\n; print_settings_id = E2B_BOOKKEEPING_SENTINEL\n") != std::string::npos);
    CHECK(print.full_print_config().opt_serialize("layer_height") != "0.32");
    DynamicPrintConfig imported_config = DynamicPrintConfig::full_print_config();
    imported_config.load_from_gcode_file(exported_file.string(), ForwardCompatibilitySubstitutionRule::EnableSilent);
    CHECK(imported_config.opt_float("layer_height") == Approx(0.32));

    DynamicPrintConfig probe_config = exported_config;
    probe_config.set_deserialize_strict("gcode_exporter_plugin", k_probe_exporter_id);
    probe_config.set_deserialize_strict(k_probe_option_key, "1");
    g_exporter_probe_state.export_mode = ProbeExportMode::Copy;
    const Orchestrator::GCodeExporterSelection probe_selection = orchestrator.select_gcode_exporter(probe_config);
    orchestrator.export_print(print, probe_config, reference.string(), (directory.path() / "probe.probe").string(), probe_selection);

    CHECK(std::is_sorted(g_exporter_probe_state.export_configuration.begin(),
                         g_exporter_probe_state.export_configuration.end(),
                         [](const std::pair<std::string, std::string> &lhs,
                            const std::pair<std::string, std::string> &rhs) { return lhs.first < rhs.first; }));
    CHECK(std::none_of(g_exporter_probe_state.export_configuration.begin(),
                       g_exporter_probe_state.export_configuration.end(),
                       [](const std::pair<std::string, std::string> &entry) {
                           return entry.first == "printhost_apikey" || entry.first == "host_type" ||
                                  entry.first == "compatible_printers" || entry.first == "inherits" ||
                                  entry.first == "default_print_profile" || entry.first == "output_path" ||
                                  entry.first == "test_config_nature_volatile";
                       }));
    CHECK(std::any_of(g_exporter_probe_state.export_configuration.begin(), g_exporter_probe_state.export_configuration.end(),
                      [](const std::pair<std::string, std::string> &entry) {
                          return entry.first == "print_settings_id" && entry.second == "E2B_BOOKKEEPING_SENTINEL";
                      }));
    const std::vector<std::pair<std::string, std::string>>::const_iterator layer_height_entry = std::find_if(
        g_exporter_probe_state.export_configuration.begin(), g_exporter_probe_state.export_configuration.end(),
        [](const std::pair<std::string, std::string> &entry) { return entry.first == "layer_height"; });
    REQUIRE(layer_height_entry != g_exporter_probe_state.export_configuration.end());
    CHECK(layer_height_entry->second == "0.32");
    const size_t region_separators = size_t(std::count_if(
        g_exporter_probe_state.export_extrusion_widths.begin(), g_exporter_probe_state.export_extrusion_widths.end(),
        [](const std::pair<std::string, std::string> &entry) { return entry.first.empty() && entry.second.empty(); }));
    CHECK(region_separators == print.num_print_regions());
    // The Print was processed but its reference was never reread, so the host sends no statistics.
    CHECK(g_exporter_probe_state.export_statistics.empty());
}

// Export one print twice through the probe provider: after a reread (a Reader result was requested) it receives the
// statistics list in file order, and after a generation without result it receives an empty list.
TEST_CASE("G-code exporter receives the statistics list only after a Reader result", "[plugins][gcode][exporter][abi][print-statistics]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    g_exporter_probe_state.query_mode = ProbeQueryMode::Compatible;
    g_exporter_probe_state.export_mode = ProbeExportMode::Copy;
    Orchestrator &orchestrator = Orchestrator::instance();
    ScopedPluginActivation probe(orchestrator, k_probe_exporter_id);
    ScopedTestDirectory directory;
    Print print;
    Model model;
    initialize_fff_print(print, model, "gcode.exporter.simple");
    print.process();

    const boost::filesystem::path reference = directory.path() / "reference.gcode";
    GCodeProcessorResult result;
    orchestrator.export_gcode(print, reference.string(), &result);
    DynamicPrintConfig probe_config = print.full_print_config();
    probe_config.set_deserialize_strict("gcode_exporter_plugin", k_probe_exporter_id);
    probe_config.set_deserialize_strict(k_probe_option_key, "1");
    const Orchestrator::GCodeExporterSelection selection = orchestrator.select_gcode_exporter(probe_config);
    orchestrator.export_print(print, probe_config, reference.string(), (directory.path() / "one.probe").string(), selection);

    REQUIRE(g_exporter_probe_state.export_statistics.size() >= 9);
    CHECK(g_exporter_probe_state.export_statistics.front().first == "filament used [mm]");
    CHECK(g_exporter_probe_state.export_statistics.back().first == "has_support");
    CHECK(g_exporter_probe_state.export_statistics.back().second == "0");
    std::string block;
    for (const std::pair<std::string, std::string> &entry : g_exporter_probe_state.export_statistics)
        block += "; " + entry.first + " = " + entry.second + "\n";
    std::string first_bad_line;
    CHECK_MESSAGE(Slic3r::Test::Plugins::GCodeExportTest::is_statistics_block(block, first_bad_line),
                  "Malformed statistics line: " << first_bad_line);

    orchestrator.export_gcode(print, reference.string(), nullptr);
    orchestrator.export_print(print, probe_config, reference.string(), (directory.path() / "two.probe").string(), selection);
    CHECK(g_exporter_probe_state.export_statistics.empty());
}
// The exporter and configuration copy given by the caller drive the export, not the Print's own configuration, and a unique
// temporary path built for it keeps the exporter extension even when the prefix contains dots.
TEST_CASE("G-code export goes through the selected exporter service", "[plugins][gcode][exporter]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    ScopedPluginActivation probe(orchestrator, k_probe_exporter_id);
    ScopedTestDirectory directory;
    Print print;
    Model model;
    initialize_fff_print(print, model, "gcode.exporter.simple");
    g_exporter_probe_state.query_mode = ProbeQueryMode::Compatible;
    g_exporter_probe_state.export_mode = ProbeExportMode::Copy;
    DynamicPrintConfig probe_config = print.full_print_config();
    probe_config.set_deserialize_strict(k_gcode_exporter_option_key, k_probe_exporter_id);
    const boost::filesystem::path reference = directory.path() / "reference.gcode";
    write_binary_file(reference, "G1 X1\n");

    const Orchestrator::GCodeExporterSelection selection = orchestrator.select_gcode_exporter(probe_config);
    REQUIRE(selection.plugin_id == k_probe_exporter_id);
    const size_t exports_before = g_exporter_probe_state.export_count;
    const std::string first = orchestrator.export_print(
        print, probe_config, reference.string(), (directory.path() / "one").string(), selection);
    const std::string second = orchestrator.export_print(
        print, probe_config, reference.string(), (directory.path() / "two").string(), selection);
    CHECK(g_exporter_probe_state.export_count == exports_before + 2);
    // EXPORT reads its settings from the configuration copy, while the Print still names the simple exporter.
    CHECK(g_exporter_probe_state.export_config_selector == k_probe_exporter_id);
    CHECK(print.full_print_config().opt_serialize(k_gcode_exporter_option_key) == k_default_gcode_exporter_id);
    CHECK(boost::filesystem::path(first).extension().string() == ".probe");
    CHECK(read_binary_file(boost::filesystem::path(first)) == "G1 X1\n");
    CHECK(read_binary_file(boost::filesystem::path(second)) == "G1 X1\n");
    CHECK(read_binary_file(reference) == "G1 X1\n");

    // The Print's own configuration still names the simple exporter, so it cannot stand in for the copy.
    CHECK_THROWS_WITH(orchestrator.export_print(print, print.full_print_config(), reference.string(),
                                                (directory.path() / "three").string(), selection),
                      "The configured G-code exporter changed before export.");

    const std::string temporary = selection.temporary_path(".app.upload.");
    CHECK(boost::filesystem::path(temporary).extension().string() == ".probe");
    CHECK(orchestrator.export_print(print, probe_config, reference.string(), temporary, selection) == temporary);
    boost::filesystem::remove(temporary);
    CHECK_THROWS_WITH(orchestrator.export_print(print, probe_config, reference.string(),
                                                (directory.path() / ".app.upload.ab12-cd34").string(), selection),
                      "The G-code exporter 'test.gcode.exporter.probe' does not support the destination extension '.ab12-cd34'.");
}

// Selection and export never clear a cancellation raised elsewhere: the flag belongs to the slicing run that raised it.
TEST_CASE("G-code exporter calls leave the global cancellation flag alone", "[plugins][gcode][exporter]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    ScopedTestDirectory directory;
    Print print;
    Model model;
    initialize_fff_print(print, model, "gcode.exporter.simple");
    const boost::filesystem::path reference = directory.path() / "reference.gcode";
    write_binary_file(reference, "G1 X1\n");
    const Orchestrator::GCodeExporterSelection selection = orchestrator.select_gcode_exporter(print.full_print_config());

    orchestrator.request_plugin_cancel();
    CHECK_THROWS(orchestrator.select_gcode_exporter(print.full_print_config()));
    CHECK(orchestrator.is_plugin_cancelled());
    CHECK_THROWS(orchestrator.export_print(print, print.full_print_config(), reference.string(),
                                           (directory.path() / "out").string(), selection));
    CHECK(orchestrator.is_plugin_cancelled());
    orchestrator.reset_plugin_cancel();
    CHECK(snapshot_directory(directory.path()).size() == 1);
}

// Generated names follow the exporter extension, whether it comes from the placeholder or from a suffix written in the template.
TEST_CASE("Automatic output names follow the exporter extension", "[plugins][gcode][exporter]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ScopedTestDirectory directory;
    Print print;
    Model model;
    initialize_fff_print(print, model, "gcode.exporter.simple");
    DynamicPrintConfig config = print.full_print_config();
    REQUIRE(config.opt_bool("binary_gcode"));

    const std::vector<std::string> formats = {
        "{input_filename_base}{default_output_extension}", "{input_filename_base}.gcode", "{input_filename_base}.bgcode"};
    for (const std::string &format : formats) {
        config.set_deserialize_strict("output_filename_format", format);
        REQUIRE(print.apply(model, config) != PrintBase::APPLY_STATUS_INVALIDATED);
        CHECK(print.output_filename("name", ".probe") == "name.probe");
        CHECK(print.output_filename("name", ".gcode") == "name.gcode");
        CHECK(boost::filesystem::path(print.output_filepath(directory.path().string(), "name", ".probe")).filename() ==
              "name.probe");
        CHECK(boost::filesystem::path(print.output_filepath(std::string(), "name", ".probe")).extension() == ".probe");
    }

    // The text aliases accepted by the built-in exporter stay as written; another exporter extension replaces them.
    config.set_deserialize_strict("output_filename_format", "{input_filename_base}.gco");
    REQUIRE(print.apply(model, config) != PrintBase::APPLY_STATUS_INVALIDATED);
    CHECK(print.output_filename("name", ".gcode") == "name.gco");
    CHECK(print.output_filename("name", ".probe") == "name.probe");
}

// Check how a serialized exporter ID behaves when a real profile INI is loaded: known IDs load as written, unknown or
// pruned (inactive at finalization) IDs fall back to the built-in exporter, reported unless the rule is silent.
void check_exporter_profile_loading()
{
    const std::string key = k_gcode_exporter_option_key;
    const std::vector<std::string> replaced_ids = {"test.gcode.exporter.missing", k_pruned_exporter_id};
    for (const std::string &id : replaced_ids) {
        DynamicPrintConfig reported = DynamicPrintConfig::full_print_config();
        const ConfigSubstitutions substitutions = reported.load_from_ini_string(
            key + " = " + id + "\n", ForwardCompatibilitySubstitutionRule::Enable);
        CHECK(reported.opt_serialize(key) == k_default_gcode_exporter_id);
        REQUIRE(substitutions.size() == 1);
        CHECK(substitutions.front().old_value == id);
        CHECK(substitutions.front().new_value->serialize() == k_default_gcode_exporter_id);

        DynamicPrintConfig silent = DynamicPrintConfig::full_print_config();
        CHECK(silent.load_from_ini_string(key + " = " + id + "\n",
                                          ForwardCompatibilitySubstitutionRule::EnableSilent).empty());
        CHECK(silent.opt_serialize(key) == k_default_gcode_exporter_id);
    }

    // A provider that is registered but inactive only at run time stays in the list and is refused at export time.
    DynamicPrintConfig known = DynamicPrintConfig::full_print_config();
    CHECK(known.load_from_ini_string(key + " = " + k_probe_exporter_id + "\n",
                                     ForwardCompatibilitySubstitutionRule::Enable).empty());
    CHECK(known.opt_serialize(key) == k_probe_exporter_id);
}

// Profiles replace unknown or inactive exporters by the built-in one while loading, and the substitution is reported.
TEST_CASE("Profile loading replaces an unavailable exporter by the built-in one", "[plugins][gcode][exporter]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    REQUIRE_FALSE(Orchestrator::instance().is_plugin_active(Orchestrator::instance().get_plugin(k_pruned_exporter_id)));
    check_exporter_profile_loading();

    DynamicPrintConfig strict = DynamicPrintConfig::full_print_config();
    CHECK_THROWS(strict.set_deserialize_strict(k_gcode_exporter_option_key, "test.gcode.exporter.missing"));
}

// The exporter group uses a strict resolution in every caller: C selection API, Print validation and the export action.
TEST_CASE("Exporter selection through the C API never falls back to another provider", "[plugins][gcode][exporter][api-c]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    DynamicConfig probe_choice;
    probe_choice.set_key_value(k_gcode_exporter_option_key, new ConfigOptionString(k_probe_exporter_id));
    DynamicConfig missing_choice;
    missing_choice.set_key_value(k_gcode_exporter_option_key, new ConfigOptionString("test.gcode.exporter.missing"));

    CHECK(select_exporter_id_through_c_api(nullptr) == k_default_gcode_exporter_id);
    CHECK(select_exporter_id_through_c_api(&missing_choice) == "<none>");
    CHECK(select_exporter_id_through_c_api(&probe_choice) == "<none>");
    {
        ScopedPluginActivation probe(orchestrator, k_probe_exporter_id);
        CHECK(select_exporter_id_through_c_api(&probe_choice) == k_probe_exporter_id);
        CHECK(select_exporter_id_through_c_api(&missing_choice) == "<none>");
    }
}

// A clean reference is exported twice: neither the writer nor the reader runs again, and the kept results stay as they were.
TEST_CASE("G-code export leaves the writer, the reader and their results untouched", "[plugins][gcode][exporter]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    ScopedTestDirectory directory;
    Print print;
    Model model;
    initialize_fff_print(print, model, "gcode.exporter.simple");
    print.process();

    const boost::filesystem::path reference = directory.path() / "reference.gcode";
    GCodeProcessorResult result;
    orchestrator.export_gcode(print, reference.string(), &result);
    const Plugin *writer = orchestrator.get_plugin("gcode.printing_plan_file_writer");
    const Plugin *reader = orchestrator.get_plugin(print.gcode_reader_plugin_id());
    REQUIRE(writer != nullptr);
    REQUIRE(reader != nullptr);
    const uint64_t writer_runs = writer->get_run_count();
    const uint64_t reader_runs = reader->get_run_count();
    REQUIRE(writer_runs >= 1);
    REQUIRE(reader_runs >= 1);

    // Age the reference so that a rewrite changes its time, and keep real reader results to compare afterwards.
    boost::filesystem::last_write_time(reference, boost::filesystem::last_write_time(reference) - 3600);
    const std::time_t reference_time = boost::filesystem::last_write_time(reference);
    const std::string reference_bytes = read_binary_file(reference);
    REQUIRE_FALSE(result.moves.empty());
    const size_t move_count = result.moves.size();
    const Vec3f last_position = result.moves.back().position;
    REQUIRE(print.print_statistics().total_extruded_volume > 0.);
    REQUIRE(print.print_statistics().statistics_available);
    const double sentinel_volume = 4242.5;
    print.print_statistics().total_extruded_volume = sentinel_volume;
    const size_t gcode_timestamp = print.step_state_with_timestamp(STEP_GCODE).timestamp;

    const Orchestrator::GCodeExporterSelection selection = orchestrator.select_gcode_exporter(print.full_print_config());
    const std::string first = orchestrator.export_print(
        print, print.full_print_config(), reference.string(), (directory.path() / "one").string(), selection);
    const std::string second = orchestrator.export_print(
        print, print.full_print_config(), reference.string(), (directory.path() / "two").string(), selection);

    // The reread made the statistics available, so both files carry the same statistics block before the configuration.
    const std::string first_text = read_binary_file(boost::filesystem::path(first));
    const std::string second_text = read_binary_file(boost::filesystem::path(second));
    const std::string statistics_block = text_between_body_and_configuration(first_text, reference_bytes);
    std::string first_bad_line;
    REQUIRE_FALSE(statistics_block.empty());
    CHECK_MESSAGE(Slic3r::Test::Plugins::GCodeExportTest::is_statistics_block(statistics_block, first_bad_line),
                  "Malformed statistics line: " << first_bad_line);
    CHECK(text_between_body_and_configuration(second_text, reference_bytes) == statistics_block);
    expect_simple_export(first_text, reference_bytes + statistics_block);
    expect_simple_export(second_text, reference_bytes + statistics_block);
    // Run counters, plus witnesses of a rewrite: the reference time and content, and the step timestamp.
    CHECK(writer->get_run_count() == writer_runs);
    CHECK(reader->get_run_count() == reader_runs);
    CHECK(boost::filesystem::last_write_time(reference) == reference_time);
    CHECK(read_binary_file(reference) == reference_bytes);
    CHECK(print.step_state_with_timestamp(STEP_GCODE).timestamp == gcode_timestamp);
    CHECK(result.moves.size() == move_count);
    CHECK(result.moves.back().position == last_position);
    CHECK(print.print_statistics().total_extruded_volume == sentinel_volume);
}
// The EXPORT payload carries a valid empty metadata prefix, configuration copy, and renderer; the adapter refuses any missing required input.
TEST_CASE("G-code exporter EXPORT requires the host configuration", "[plugins][gcode][exporter]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    ScopedPluginActivation probe(orchestrator, k_probe_exporter_id);
    ScopedTestDirectory directory;
    Print print;
    Model model;
    initialize_fff_print(print, model, k_probe_exporter_id);
    g_exporter_probe_state.export_mode = ProbeExportMode::Copy;
    const boost::filesystem::path reference = directory.path() / "reference.gcode";
    const boost::filesystem::path output = directory.path() / "output.probe";
    write_binary_file(reference, "G1 X1\n");
    const std::string reference_path = reference.string();
    const std::string output_path = output.string();
    raw_gcode_export_metadata metadata = {};
    metadata.struct_size = uint32_t(sizeof(metadata));

    run_ctx_gcode_export context = {};
    context.operation = RAW_GCODE_EXPORT_OPERATION_EXPORT;
    context.print = reinterpret_cast<const print_handle *>(&print);
    context.reference_path = reference_path.c_str();
    context.output_path = output_path.c_str();
    context.metadata = &metadata;
    raw_plugin_run_payload payload = {};
    payload.data = &context;
    Plugin *plugin = orchestrator.get_plugin(k_probe_exporter_id);
    REQUIRE(plugin != nullptr);

    const size_t exports_before = g_exporter_probe_state.export_count;
    std::string reported_error;
    CHECK(orchestrator.execute_plugin(*plugin, &print, &payload, 1, false, &reported_error) !=
          RAW_PLUGIN_EXECUTION_SUCCESS);
    CHECK(reported_error.find("EXPORT received invalid operation fields") != std::string::npos);
    CHECK(g_exporter_probe_state.export_count == exports_before);
    CHECK_FALSE(boost::filesystem::exists(output));

    // A configuration alone is not enough: the renderer bridge is always present in EXPORT.
    context.config = ApiHost::to_config_handle(&print.full_print_config());
    CHECK(orchestrator.execute_plugin(*plugin, &print, &payload, 1, false, &reported_error) !=
          RAW_PLUGIN_EXECUTION_SUCCESS);
    CHECK(reported_error.find("EXPORT received invalid operation fields") != std::string::npos);
    CHECK(g_exporter_probe_state.export_count == exports_before);

    const GCodeThumbnailRenderer renderer(print, ThumbnailsGeneratorCallback());
    context.thumbnail_renderer = renderer.c_renderer();
    CHECK(orchestrator.execute_plugin(*plugin, &print, &payload, 1, false, &reported_error) ==
          RAW_PLUGIN_EXECUTION_SUCCESS);
    CHECK(g_exporter_probe_state.export_count == exports_before + 1);
    CHECK(read_binary_file(output) == "G1 X1\n");

    context.metadata = nullptr;
    CHECK(orchestrator.execute_plugin(*plugin, &print, &payload, 1, false, &reported_error) !=
          RAW_PLUGIN_EXECUTION_SUCCESS);
    CHECK(reported_error.find("EXPORT received invalid operation fields") != std::string::npos);
    CHECK(g_exporter_probe_state.export_count == exports_before + 1);
}

// Build a render callback that returns a white square of the requested size and counts its calls.
static ThumbnailsGeneratorCallback white_square_renderer(uint32_t &calls)
{
    return [&calls](const ThumbnailsParams &params) {
        ++calls;
        ThumbnailData image;
        image.set(uint32_t(params.sizes.front().x()), uint32_t(params.sizes.front().y()));
        std::fill(image.pixels.begin(), image.pixels.end(), 255);
        return ThumbnailsList{image};
    };
}

// Return a deterministic RGBA renderer that increments a borrowed counter and uses the first requested size; the request must contain a size and image allocation failures propagate.
static ThumbnailsGeneratorCallback large_noise_renderer(uint32_t &calls)
{
    return [&calls](const ThumbnailsParams &params) {
        ++calls;
        ThumbnailData image;
        image.set(uint32_t(params.sizes.front().x()), uint32_t(params.sizes.front().y()));
        uint32_t state = 0x6d2b79f5u;
        const size_t pixel_count = image.pixels.size() / 4;
        for (size_t pixel = 0; pixel < pixel_count; ++pixel) {
            state = state * 1664525u + 1013904223u;
            image.pixels[pixel * 4] = uint8_t(state >> 24);
            state = state * 1664525u + 1013904223u;
            image.pixels[pixel * 4 + 1] = uint8_t(state >> 24);
            state = state * 1664525u + 1013904223u;
            image.pixels[pixel * 4 + 2] = uint8_t(state >> 24);
            image.pixels[pixel * 4 + 3] = 255;
        }
        return ThumbnailsList{image};
    };
}

// Export a Print without objects for four note forms into test-scoped files; require the exact header/notes prefix, one adjacent body, no width comments, and an immediate configuration footer.
TEST_CASE("Exporter writes the notes after the producer line", "[plugins][gcode][exporter][notes]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    ScopedTestDirectory directory;
    Print print;
    const boost::filesystem::path reference = directory.path() / "reference.gcode";
    write_binary_file(reference, "G1 X1\n");

    const std::string generated_header = "; generated by " SLIC3R_APP_KEY " " SLIC3R_VERSION " on\n\n";
    const std::vector<std::pair<std::string, std::string>> cases = {
        {"", generated_header},
        {"single", generated_header + "; single\n\n"},
        {"first\r\nsecond", generated_header + "; first\n; second\n\n"},
        {"ends\n", generated_header + "; ends\n; \n\n"}};
    for (const std::pair<std::string, std::string> &entry : cases) {
        INFO("notes: " << entry.first);
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.set_key_value("notes", new ConfigOptionString(entry.first));
        const Orchestrator::GCodeExporterSelection selection = orchestrator.select_gcode_exporter(config);
        const boost::filesystem::path destination = directory.path() / "notes.gcode";
        orchestrator.export_print(print, config, reference.string(), destination.string(), selection);
        const std::string exported = read_binary_file(destination);
        REQUIRE(exported.compare(0, entry.second.size(), entry.second) == 0);
        CHECK(exported.find("extrusion width =") == std::string::npos);
        const std::string body = "G1 X1\n";
        const size_t body_position = entry.second.size();
        REQUIRE(exported.compare(body_position, body.size(), body) == 0);
        CHECK(exported.find(body, body_position + 1) == std::string::npos);
        const std::string begin_marker = "\n; " SLIC3R_APP_NAME "_config = begin\n";
        const size_t configuration_position = body_position + body.size();
        REQUIRE(exported.compare(configuration_position, begin_marker.size(), begin_marker) == 0);
        const std::string end_marker = "; " SLIC3R_APP_NAME "_config = end\n";
        REQUIRE(exported.size() >= end_marker.size());
        CHECK(exported.compare(exported.size() - end_marker.size(), end_marker.size(), end_marker) == 0);
    }
}

// Export oversized configuration and test legacy imports in scoped files; assert exact output boundaries, rollback after a failed delimiter import, old-header compatibility, and the viewer's 65,535-byte fallback, with Catch failures identifying mismatches.
TEST_CASE("Large G-code configuration imports with and without an end thumbnail",
          "[plugins][gcode][exporter][config][thumbnails][viewer]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();

    SECTION("configuration closes a file without an end thumbnail")
    {
        ScopedTestDirectory directory;
        Print print;
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.set_deserialize_strict("default_acceleration", "42");
        config.set_deserialize_strict("before_layer_gcode", std::string(18000, 'G'));
        config.set_deserialize_strict("end_gcode", std::string(18000, 'G'));
        config.set_deserialize_strict("layer_gcode", std::string(18000, 'G'));
        config.set_deserialize_strict("start_gcode", std::string(18000, 'G'));
        config.set_deserialize_strict("toolchange_gcode", std::string(18000, 'G'));
        const boost::filesystem::path reference = directory.path() / "reference.gcode";
        const boost::filesystem::path destination = directory.path() / "exported.gcode";
        write_binary_file(reference, "G1 X1\n");
        const Orchestrator::GCodeExporterSelection selection = orchestrator.select_gcode_exporter(config);
        orchestrator.export_print(print, config, reference.string(), destination.string(), selection);

        const std::string exported = read_binary_file(destination);
        const std::string generated_header = "; generated by " SLIC3R_APP_KEY " " SLIC3R_VERSION " on\n\n";
        REQUIRE(exported.compare(0, generated_header.size(), generated_header) == 0);
        const std::string body = "G1 X1\n";
        const size_t body_position = generated_header.size();
        REQUIRE(exported.compare(body_position, body.size(), body) == 0);
        CHECK(exported.find(body, body_position + 1) == std::string::npos);
        const std::string begin_marker = "\n; " SLIC3R_APP_NAME "_config = begin\n";
        const std::string end_marker = "; " SLIC3R_APP_NAME "_config = end\n";
        const size_t begin_position = body_position + body.size();
        REQUIRE(exported.compare(begin_position, begin_marker.size(), begin_marker) == 0);
        const size_t end_position = exported.find(end_marker);
        REQUIRE(end_position != std::string::npos);
        CHECK(exported.find("; default_acceleration = 42\n", begin_position + begin_marker.size()) != std::string::npos);
        CHECK(end_position - begin_position > 65535);
        CHECK(exported.compare(exported.size() - end_marker.size(), end_marker.size(), end_marker) == 0);

        DynamicPrintConfig imported = DynamicPrintConfig::full_print_config();
        imported.load_from_gcode_file(destination.string(), ForwardCompatibilitySubstitutionRule::EnableSilent);
        const ConfigOptionFloatOrPercent *acceleration = imported.option<ConfigOptionFloatOrPercent>("default_acceleration");
        REQUIRE(acceleration != nullptr);
        CHECK(acceleration->value == Approx(42.0));
        CHECK_FALSE(acceleration->percent);
    }

    SECTION("configuration precedes a large end thumbnail and the viewer reads its early options")
    {
        ScopedTestDirectory directory;
        Print print;
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.set_deserialize_strict("max_print_height", "321");
        config.set_deserialize_strict("default_acceleration", "42");
        config.set_deserialize_strict("before_layer_gcode", std::string(18000, 'G'));
        config.set_deserialize_strict("end_gcode", std::string(18000, 'G'));
        config.set_deserialize_strict("layer_gcode", std::string(18000, 'G'));
        config.set_deserialize_strict("start_gcode", std::string(18000, 'G'));
        config.set_deserialize_strict("toolchange_gcode", std::string(18000, 'G'));
        config.set_deserialize_strict({{"thumbnails", "512x512"},
                                       {"thumbnails_format", "PNG"},
                                       {"thumbnails_end_file", "1"}});
        const boost::filesystem::path reference = directory.path() / "reference.gcode";
        const boost::filesystem::path destination = directory.path() / "exported.gcode";
        write_binary_file(reference, "G1 X1\n");
        const Orchestrator::GCodeExporterSelection selection = orchestrator.select_gcode_exporter(config);
        uint32_t calls = 0;
        orchestrator.export_print(print, config, reference.string(), destination.string(), selection,
                                  large_noise_renderer(calls));

        const std::string exported = read_binary_file(destination);
        const std::string generated_header = "; generated by " SLIC3R_APP_KEY " " SLIC3R_VERSION " on\n\n";
        REQUIRE(exported.compare(0, generated_header.size(), generated_header) == 0);
        const std::string body = "G1 X1\n";
        const size_t body_position = generated_header.size();
        REQUIRE(exported.compare(body_position, body.size(), body) == 0);
        CHECK(exported.find(body, body_position + 1) == std::string::npos);
        const std::string config_end = "; " SLIC3R_APP_NAME "_config = end\n";
        const std::string thumbnail_begin = "; thumbnail begin 512x512";
        const size_t config_position = exported.find(config_end);
        const size_t thumbnail_position = exported.find(thumbnail_begin);
        REQUIRE(config_position != std::string::npos);
        REQUIRE(thumbnail_position != std::string::npos);
        const std::string config_begin = "\n; " SLIC3R_APP_NAME "_config = begin\n";
        const size_t body_end = body_position + body.size();
        REQUIRE(exported.compare(body_end, config_begin.size(), config_begin) == 0);
        CHECK(config_position < thumbnail_position);
        CHECK(exported.size() - (config_position + config_end.size()) > 65535);
        CHECK(exported.compare(exported.size() - std::string("thumbnail end\n;\n").size(),
                               std::string("thumbnail end\n;\n").size(), "thumbnail end\n;\n") == 0);
        CHECK(calls == 1);

        DynamicPrintConfig imported = DynamicPrintConfig::full_print_config();
        imported.load_from_gcode_file(destination.string(), ForwardCompatibilitySubstitutionRule::EnableSilent);
        CHECK(imported.opt_float("max_print_height") == Approx(321.0));
        const ConfigOptionFloatOrPercent *acceleration = imported.option<ConfigOptionFloatOrPercent>("default_acceleration");
        REQUIRE(acceleration != nullptr);
        CHECK(acceleration->value == Approx(42.0));
        CHECK_FALSE(acceleration->percent);

        GCodeProcessor viewer;
        viewer.process_file(destination.string());
        CHECK(viewer.get_result().max_print_height == Approx(321.0f));
    }

    SECTION("postprocessed legacy files use the tail reader")
    {
        ScopedTestDirectory directory;
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.set_deserialize_strict("max_print_height", "177");
        std::string legacy = "; postprocessed: generated by " SLIC3R_APP_KEY "\n\n";
        legacy += "G1 X1\n";
        const std::vector<std::string> keys = config.keys();
        size_t legacy_entries = 0;
        for (const std::string &key : keys) {
            if (key == "max_print_height")
                continue;
            const std::string value = config.opt_serialize(key);
            if (value.find_first_of("\r\n") == std::string::npos) {
                legacy += "; " + key + " = " + value + "\n";
                ++legacy_entries;
                if (legacy_entries == 80)
                    break;
            }
        }
        REQUIRE(legacy_entries == 80);
        legacy += "; max_print_height = 177\n";
        REQUIRE(legacy.size() < 65535);
        const boost::filesystem::path path = directory.path() / "legacy.gcode";
        write_binary_file(path, legacy);

        DynamicPrintConfig imported = DynamicPrintConfig::full_print_config();
        CHECK_THROWS(imported.load_from_gcode_file(path.string(), ForwardCompatibilitySubstitutionRule::EnableSilent));

        GCodeProcessor viewer;
        viewer.process_file(path.string());
        CHECK(viewer.get_result().max_print_height == Approx(177.0f));
    }

    SECTION("a failed delimiter import does not leak a setting into the tail fallback")
    {
        ScopedTestDirectory directory;
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        const double default_max_print_height = config.opt_float("max_print_height");
        REQUIRE(default_max_print_height != 177.0);
        std::string gcode = "; generated by " SLIC3R_APP_KEY " " SLIC3R_VERSION " on\n\n";
        gcode += "; max_print_height = 177\n";
        gcode += "; " + std::string(70000, 'X') + "\n";

        size_t valid_entries = 0;
        for (const std::string &key : config.keys()) {
            if (key == "max_print_height")
                continue;
            const std::string value = config.opt_serialize(key);
            if (value.find_first_of("\r\n") != std::string::npos)
                continue;
            gcode += "; " + key + " = " + value + "\n";
            if (++valid_entries == 80)
                break;
        }
        REQUIRE(valid_entries == 80);
        gcode += "; " SLIC3R_APP_NAME "_config = end\n";
        const size_t witness_position = gcode.find("; max_print_height = 177\n");
        REQUIRE(witness_position != std::string::npos);
        REQUIRE(gcode.size() - witness_position > 65535);

        const boost::filesystem::path path = directory.path() / "missing-begin.gcode";
        write_binary_file(path, gcode);
        GCodeProcessor viewer;
        viewer.process_file(path.string());
        CHECK(viewer.get_result().max_print_height == Approx(static_cast<float>(default_max_print_height)));
    }

    SECTION("an older recognized header without a version still imports the tail")
    {
        ScopedTestDirectory directory;
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        std::string gcode = "; generated by " SLIC3R_APP_KEY "\n\nG1 X1\n";
        size_t valid_entries = 0;
        for (const std::string &key : config.keys()) {
            if (key == "max_print_height")
                continue;
            const std::string value = config.opt_serialize(key);
            if (value.find_first_of("\r\n") != std::string::npos)
                continue;
            gcode += "; " + key + " = " + value + "\n";
            if (++valid_entries == 80)
                break;
        }
        REQUIRE(valid_entries == 80);
        gcode += "; max_print_height = 177\n";
        REQUIRE(gcode.size() < 65535);

        const boost::filesystem::path path = directory.path() / "old-header.gcode";
        write_binary_file(path, gcode);
        DynamicPrintConfig imported = DynamicPrintConfig::full_print_config();
        imported.load_from_gcode_file(path.string(), ForwardCompatibilitySubstitutionRule::EnableSilent);
        CHECK(imported.opt_float("max_print_height") == Approx(177.0));

        GCodeProcessor viewer;
        viewer.process_file(path.string());
        CHECK(viewer.get_result().max_print_height == Approx(177.0f));
    }
}

// Slice two volume-specific regions through three first-layer-option states and compare each test-scoped export's literal width block with the legacy writer before the unique body; Catch failures report mismatches.
TEST_CASE("G-code export widths match legacy regions and first-layer options", "[plugins][gcode][exporter][gcode]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    struct WidthScenario
    {
        const char *name;
        bool first_layer_perimeter;
        bool first_layer_infill;
        const char *expected;
    };
    const WidthScenario scenarios[] = {
        {"perimeter only", true, false,
         "; external perimeters extrusion width = 0.42mm\n"
         "; perimeters extrusion width = 0.45mm\n"
         "; infill extrusion width = 0.45mm\n"
         "; solid infill extrusion width = 0.45mm\n"
         "; top infill extrusion width = 0.42mm\n"
         "; support material extrusion width = 0.43mm\n"
         "; support material interface extrusion width = 0.43mm\n"
         "; first layer extrusion width = 0.49mm\n\n"
         "; external perimeters extrusion width = 0.40mm\n"
         "; perimeters extrusion width = 0.45mm\n"
         "; infill extrusion width = 0.50mm\n"
         "; solid infill extrusion width = 0.55mm\n"
         "; top infill extrusion width = 0.60mm\n"
         "; support material extrusion width = 0.43mm\n"
         "; support material interface extrusion width = 0.43mm\n"
         "; first layer extrusion width = 0.49mm\n\n"
         "; external perimeters extrusion width = 0.65mm\n"
         "; perimeters extrusion width = 0.70mm\n"
         "; infill extrusion width = 0.75mm\n"
         "; solid infill extrusion width = 0.80mm\n"
         "; top infill extrusion width = 0.85mm\n"
         "; support material extrusion width = 0.43mm\n"
         "; support material interface extrusion width = 0.43mm\n"
         "; first layer extrusion width = 0.49mm\n\n"},
        {"infill only", false, true,
         "; external perimeters extrusion width = 0.42mm\n"
         "; perimeters extrusion width = 0.45mm\n"
         "; infill extrusion width = 0.45mm\n"
         "; solid infill extrusion width = 0.45mm\n"
         "; top infill extrusion width = 0.42mm\n"
         "; support material extrusion width = 0.43mm\n"
         "; support material interface extrusion width = 0.43mm\n"
         "; first layer infill extrusion width = 0.54mm\n\n"
         "; external perimeters extrusion width = 0.40mm\n"
         "; perimeters extrusion width = 0.45mm\n"
         "; infill extrusion width = 0.50mm\n"
         "; solid infill extrusion width = 0.55mm\n"
         "; top infill extrusion width = 0.60mm\n"
         "; support material extrusion width = 0.43mm\n"
         "; support material interface extrusion width = 0.43mm\n"
         "; first layer infill extrusion width = 0.54mm\n\n"
         "; external perimeters extrusion width = 0.65mm\n"
         "; perimeters extrusion width = 0.70mm\n"
         "; infill extrusion width = 0.75mm\n"
         "; solid infill extrusion width = 0.80mm\n"
         "; top infill extrusion width = 0.85mm\n"
         "; support material extrusion width = 0.43mm\n"
         "; support material interface extrusion width = 0.43mm\n"
         "; first layer infill extrusion width = 0.54mm\n\n"},
        {"both first-layer options", true, true,
         "; external perimeters extrusion width = 0.42mm\n"
         "; perimeters extrusion width = 0.45mm\n"
         "; infill extrusion width = 0.45mm\n"
         "; solid infill extrusion width = 0.45mm\n"
         "; top infill extrusion width = 0.42mm\n"
         "; support material extrusion width = 0.43mm\n"
         "; support material interface extrusion width = 0.43mm\n"
         "; first layer extrusion width = 0.49mm\n"
         "; first layer infill extrusion width = 0.54mm\n\n"
         "; external perimeters extrusion width = 0.40mm\n"
         "; perimeters extrusion width = 0.45mm\n"
         "; infill extrusion width = 0.50mm\n"
         "; solid infill extrusion width = 0.55mm\n"
         "; top infill extrusion width = 0.60mm\n"
         "; support material extrusion width = 0.43mm\n"
         "; support material interface extrusion width = 0.43mm\n"
         "; first layer extrusion width = 0.49mm\n"
         "; first layer infill extrusion width = 0.54mm\n\n"
         "; external perimeters extrusion width = 0.65mm\n"
         "; perimeters extrusion width = 0.70mm\n"
         "; infill extrusion width = 0.75mm\n"
         "; solid infill extrusion width = 0.80mm\n"
         "; top infill extrusion width = 0.85mm\n"
         "; support material extrusion width = 0.43mm\n"
         "; support material interface extrusion width = 0.43mm\n"
         "; first layer extrusion width = 0.49mm\n"
         "; first layer infill extrusion width = 0.54mm\n\n"}
    };

    for (const WidthScenario &scenario : scenarios) {
        INFO(scenario.name);
        ScopedTestDirectory directory;
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.set_num_extruders(1);
        config.set_deserialize_strict({{"step_gcode_plugin", "gcode.printing_plan_file_writer"},
                                       {"gcode_firmware_plugin", "gcode.firmware.marlin2"},
                                       {"gcode_exporter_plugin", "gcode.exporter.simple"},
                                       {"binary_gcode", "0"},
                                       {"layer_height", "0.20"},
                                       {"support_material", "1"},
                                       {"support_material_interface_layers", "3"},
                                       {"support_material_extrusion_width", "0.43"},
                                       {"first_layer_extrusion_width", "0.49"},
                                       {"first_layer_infill_extrusion_width", "0.54"}});
        config.option("first_layer_extrusion_width")->set_enabled(scenario.first_layer_perimeter);
        config.option("first_layer_infill_extrusion_width")->set_enabled(scenario.first_layer_infill);
        Model model;
        Print print;
        Test::init_print({Test::TestMesh::cube_20x20x20, Test::TestMesh::cube_20x20x20}, print, model, config);
        const std::vector<std::pair<std::string, std::string>> first_region = {
            {"external_perimeter_extrusion_width", "0.40"}, {"perimeter_extrusion_width", "0.45"},
            {"infill_extrusion_width", "0.50"}, {"solid_infill_extrusion_width", "0.55"},
            {"top_infill_extrusion_width", "0.60"}};
        const std::vector<std::pair<std::string, std::string>> second_region = {
            {"external_perimeter_extrusion_width", "0.65"}, {"perimeter_extrusion_width", "0.70"},
            {"infill_extrusion_width", "0.75"}, {"solid_infill_extrusion_width", "0.80"},
            {"top_infill_extrusion_width", "0.85"}};
        for (const std::pair<std::string, std::string> &entry : first_region)
            model.objects()[0].volumes.front()->config.set_deserialize_strict(entry.first, entry.second);
        for (const std::pair<std::string, std::string> &entry : second_region)
            model.objects()[1].volumes.front()->config.set_deserialize_strict(entry.first, entry.second);
        CHECK(print.apply(model, config) != PrintBase::APPLY_STATUS_INVALIDATED);
        print.process();
        // The two volume-specific configurations coexist with the active default region.
        REQUIRE(print.num_print_regions() == 3);

        const boost::filesystem::path reference = directory.path() / "reference.gcode";
        const std::string reference_path = orchestrator.export_gcode(print, reference.string(), nullptr);
        const boost::filesystem::path legacy_path = directory.path() / "legacy.gcode";
        GCodeGenerator legacy_generator;
        const std::string legacy_path_string = legacy_path.string();
        legacy_generator.do_export(&print, legacy_path_string.c_str(), nullptr);
        const std::string legacy = read_binary_file(legacy_path);
        const std::string first_width_label = "; external perimeters extrusion width =";
        const size_t legacy_width_begin = legacy.find(first_width_label);
        REQUIRE(legacy_width_begin != std::string::npos);
        size_t legacy_width_end = legacy_width_begin;
        for (size_t region = 0; region < print.num_print_regions(); ++region) {
            legacy_width_end = legacy.find("\n\n", legacy_width_end);
            REQUIRE(legacy_width_end != std::string::npos);
            legacy_width_end += 2;
        }
        const std::string legacy_widths = legacy.substr(legacy_width_begin, legacy_width_end - legacy_width_begin);

        const DynamicPrintConfig export_config = print.full_print_config();
        const Orchestrator::GCodeExporterSelection selection = orchestrator.select_gcode_exporter(export_config);
        const boost::filesystem::path exported_path = directory.path() / "exported.gcode";
        orchestrator.export_print(print, export_config, reference_path, exported_path.string(), selection);
        const std::string exported = read_binary_file(exported_path);
        const size_t exported_width_begin = exported.find(first_width_label);
        REQUIRE(exported_width_begin != std::string::npos);
        size_t exported_width_end = exported_width_begin;
        for (size_t region = 0; region < print.num_print_regions(); ++region) {
            exported_width_end = exported.find("\n\n", exported_width_end);
            REQUIRE(exported_width_end != std::string::npos);
            exported_width_end += 2;
        }
        const std::string exported_widths = exported.substr(exported_width_begin, exported_width_end - exported_width_begin);
        const std::string reference_body = read_binary_file(reference);
        const size_t body_position = exported.find(reference_body);
        REQUIRE(body_position != std::string::npos);
        CHECK(exported.find(reference_body, body_position + 1) == std::string::npos);
        CHECK(exported_width_end <= body_position);
        CHECK(exported_widths == scenario.expected);
        CHECK(exported_widths == legacy_widths);
    }
}

// Export a rendered thumbnail around a processed reference, then check the producer, notes, adjacent width block, exact body, and configuration boundaries; temporary files are test-owned and Catch reports output mismatches.
TEST_CASE("Reference holds G-code only while the exported file carries notes and thumbnails",
          "[plugins][gcode][exporter][thumbnails]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    ScopedTestDirectory directory;
    Print print;
    Model model;
    initialize_fff_print(print, model, "gcode.exporter.simple",
                         {{"thumbnails", "4x4"}, {"thumbnails_format", "PNG"}, {"notes", "secret note"}});
    print.process();
    uint32_t calls = 0;
    const boost::filesystem::path reference = directory.path() / "reference.gcode";
    orchestrator.export_gcode(print, reference.string(), nullptr);
    const std::string body = read_binary_file(reference);
    CHECK(body.find("secret note") == std::string::npos);
    CHECK(body.find("generated by") == std::string::npos);
    CHECK(body.find("thumbnail") == std::string::npos);

    const DynamicPrintConfig config = print.full_print_config();
    const Orchestrator::GCodeExporterSelection selection = orchestrator.select_gcode_exporter(config);
    const boost::filesystem::path destination = directory.path() / "exported.gcode";
    orchestrator.export_print(print, config, reference.string(), destination.string(), selection, white_square_renderer(calls));
    const std::string exported = read_binary_file(destination);
    CHECK(calls == 1);
    REQUIRE(exported.size() >= body.size());
    // The checked prefix covers producer, before-start thumbnail, and notes; widths follow before the body.
    const std::string expected_header =
        "; generated by " SLIC3R_APP_KEY " " SLIC3R_VERSION " on\n\n"
        "\n;\n; thumbnail begin 4x4 112\n"
        "; iVBORw0KGgoAAAANSUhEUgAAAAQAAAAECAYAAACp8Z5+AAAAGUlEQVR4AZXIIQEAAACDMPqX/j2OyT\n"
        "; GhxwF4PD/BCv/g/AAAAABJRU5ErkJggg==\n"
        "; thumbnail end\n;\n"
        "; secret note\n\n";
    CHECK(exported.compare(0, expected_header.size(), expected_header) == 0);
    const size_t body_position = expect_body_then_configuration(exported, body);
    const size_t width_position = exported.find("; external perimeters extrusion width = ", expected_header.size());
    REQUIRE(width_position != std::string::npos);
    REQUIRE(width_position == expected_header.size());
    REQUIRE(width_position < body_position);
    const std::string_view width_block(exported.data() + width_position, body_position - width_position);
    std::string first_bad_width_line;
    CHECK_MESSAGE(Slic3r::Test::Plugins::GCodeExportTest::is_width_block(width_block, first_bad_width_line),
                  "Malformed extrusion-width line: " << first_bad_width_line);
    const std::string config_end = "; " SLIC3R_APP_NAME "_config = end\n";
    REQUIRE(exported.size() >= config_end.size());
    CHECK(exported.compare(exported.size() - config_end.size(), config_end.size(), config_end) == 0);
}

// Export a BIQU thumbnail with firmware object definitions and assert byte-zero placement, exact reference bytes, an adjacent configuration footer, and its EOF boundary in scoped files.
TEST_CASE("BIQU thumbnails stay at byte zero with firmware object definitions",
          "[plugins][gcode][exporter][thumbnails]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    ScopedTestDirectory directory;
    Print print;
    Model model;
    initialize_fff_print(print, model, "gcode.exporter.simple",
                         {{"gcode_firmware_plugin", "gcode.firmware.klipper"}, {"gcode_label_objects", "firmware"},
                          {"thumbnails", "4x4"}, {"thumbnails_format", "BIQU"}});
    print.process();
    uint32_t calls = 0;
    const boost::filesystem::path reference = directory.path() / "reference.gcode";
    orchestrator.export_gcode(print, reference.string(), nullptr);
    const std::string body = read_binary_file(reference);
    REQUIRE(body.compare(0, 21, "EXCLUDE_OBJECT_DEFINE") == 0);

    const DynamicPrintConfig config = print.full_print_config();
    const Orchestrator::GCodeExporterSelection selection = orchestrator.select_gcode_exporter(config);
    const boost::filesystem::path destination = directory.path() / "exported.gcode";
    orchestrator.export_print(print, config, reference.string(), destination.string(), selection, white_square_renderer(calls));
    const std::string exported = read_binary_file(destination);
    CHECK(calls == 1);
    CHECK(exported.compare(0, 9, ";00040004") == 0);
    const std::string generated_header = "; generated by " SLIC3R_APP_KEY " " SLIC3R_VERSION " on\n\n";
    CHECK(exported.find("EXCLUDE_OBJECT_DEFINE") > exported.find(generated_header));
    REQUIRE(exported.size() >= body.size());
    expect_body_then_configuration(exported, body);
    const std::string config_end = "; " SLIC3R_APP_NAME "_config = end\n";
    REQUIRE(exported.size() >= config_end.size());
    CHECK(exported.compare(exported.size() - config_end.size(), config_end.size(), config_end) == 0);
}

// Thumbnail settings only change what the exporter writes, so they never replay the G-code of an unchanged print.
TEST_CASE("Thumbnail settings leave the G-code reference alone", "[plugins][gcode][exporter][thumbnails][config]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    const std::vector<std::string> silent_keys = {
        "thumbnails", "thumbnails_color", "thumbnail_color_mode", "thumbnails_end_file", "thumbnails_with_bed",
        "thumbnails_tag_format", "thumbnails_with_support", "thumbnail_background", "thumbnail_background_color",
        "thumbnails_format"};
    for (const std::string &key : silent_keys) {
        INFO(key);
        const ConfigOptionDef *definition = PrintConfigDef::instance().get(key);
        REQUIRE(definition != nullptr);
        CHECK(definition->invalidates_step == STEP_NONE);
    }
    // Witnesses: settings that can reach the G-code stay replaying it.
    for (const std::string &key : {"notes", "printer_notes", "filament_notes", "gcode_label_objects"}) {
        INFO(key);
        const ConfigOptionDef *definition = PrintConfigDef::instance().get(key);
        REQUIRE(definition != nullptr);
        CHECK(definition->invalidates_step != STEP_NONE);
    }

    ScopedTestDirectory directory;
    Print print;
    Model model;
    initialize_fff_print(print, model, "gcode.exporter.simple");
    print.process();
    orchestrator.export_gcode(print, (directory.path() / "reference.gcode").string(), nullptr);
    REQUIRE(print.is_step_done(STEP_GCODE));
    const size_t gcode_timestamp = print.step_state_with_timestamp(STEP_GCODE).timestamp;

    DynamicPrintConfig changed = print.full_print_config();
    changed.set_deserialize_strict({
        {"thumbnails", "4x4"}, {"thumbnails_format", "JPG"}, {"thumbnails_end_file", "1"},
        {"thumbnails_with_support", "1"}, {"thumbnails_tag_format", "1"}, {"thumbnail_background", "solid"},
        {"thumbnail_background_color", "#102030"}, {"thumbnail_color_mode", "uniform"}, {"thumbnails_color", "#ff0000"},
        {"thumbnails_with_bed", "0"}});
    CHECK(print.apply(model, changed) == PrintBase::APPLY_STATUS_CHANGED);
    CHECK(print.is_step_done(STEP_GCODE));
    CHECK(print.step_state_with_timestamp(STEP_GCODE).timestamp == gcode_timestamp);
    // The new values are really in the Print, not only accepted.
    CHECK(print.full_print_config().opt_serialize("thumbnails_format") == "JPG");
    CHECK(print.full_print_config().opt_serialize("thumbnail_background") == "solid");
    CHECK(print.full_print_config().opt_serialize("thumbnails_color") == "#ff0000");
    CHECK(print.full_print_config().opt_bool("thumbnails_with_support"));
}

// A start script may print {notes}, so a change of the notes must regenerate the G-code reference.
TEST_CASE("Notes still replay the G-code when a script reads them", "[plugins][gcode][exporter][config]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    ScopedTestDirectory directory;
    Print print;
    Model model;
    initialize_fff_print(print, model, "gcode.exporter.simple",
                         {{"start_gcode", ";NOTES {notes}"}, {"notes", "first"}});
    print.process();
    const boost::filesystem::path reference = directory.path() / "reference.gcode";
    orchestrator.export_gcode(print, reference.string(), nullptr);
    CHECK(read_binary_file(reference).find(";NOTES first") != std::string::npos);
    REQUIRE(print.is_step_done(STEP_GCODE));

    DynamicPrintConfig changed = print.full_print_config();
    changed.set_deserialize_strict("notes", "second");
    CHECK(print.apply(model, changed) == PrintBase::APPLY_STATUS_INVALIDATED);
    CHECK_FALSE(print.is_step_done(STEP_GCODE));
    print.process();
    orchestrator.export_gcode(print, reference.string(), nullptr);
    const std::string regenerated = read_binary_file(reference);
    CHECK(regenerated.find(";NOTES second") != std::string::npos);
    CHECK(regenerated.find(";NOTES first") == std::string::npos);
}
// A thumbnail encoder that cannot start fails its export only: the global stop flag stays down, and the next export works.
TEST_CASE("Encoder errors stay local to their export", "[plugins][gcode][exporter][thumbnails]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    ScopedPluginActivation probe(orchestrator, k_probe_exporter_id);
    ScopedPluginActivation encoder(orchestrator, k_failing_encoder_id);
    ScopedTestDirectory directory;
    Print print;
    Model model;
    initialize_fff_print(print, model, k_probe_exporter_id);
    const boost::filesystem::path reference = directory.path() / "reference.gcode";
    const boost::filesystem::path destination = directory.path() / "final.probe";
    write_binary_file(reference, "G1 X1\n");
    write_binary_file(destination, "old destination bytes");
    const Orchestrator::GCodeExporterSelection selection = orchestrator.select_gcode_exporter(print.full_print_config());
    const std::map<std::string, std::string> original_files = snapshot_directory(directory.path());

    g_exporter_probe_state.query_mode = ProbeQueryMode::Compatible;
    g_exporter_probe_state.export_mode = ProbeExportMode::FailingEncoder;
    CHECK_THROWS_WITH(orchestrator.export_print(print, print.full_print_config(), reference.string(),
                                                destination.string(), selection),
                      Catch::Matchers::Contains("Thumbnail provider 'test.thumbnail.failing' failed"));
    CHECK_FALSE(orchestrator.is_plugin_cancelled());
    CHECK(snapshot_directory(directory.path()) == original_files);

    g_exporter_probe_state.export_mode = ProbeExportMode::Copy;
    CHECK(orchestrator.export_print(print, print.full_print_config(), reference.string(),
                                    destination.string(), selection) == destination.string());
    CHECK(read_binary_file(destination) == "G1 X1\n");
    CHECK_FALSE(orchestrator.is_plugin_cancelled());
}

// A built-in selector group reads its own option key, so a firmware that is not the first provider is found by the
// group resolution, by the C selection API and by the firmware ID that QUERY receives.
TEST_CASE("Built-in selector groups resolve a firmware that is not the first", "[plugins][gcode][exporter][api-c]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    ScopedPluginActivation probe(orchestrator, k_probe_exporter_id);
    const char *const firmware_id = "gcode.firmware.klipper";
    DynamicPrintConfig plain = DynamicPrintConfig::full_print_config();
    REQUIRE(plain.opt_serialize("gcode_firmware_plugin") != firmware_id);
    // The target must not be the first active firmware, or the test would pass without the group lookup.
    REQUIRE(orchestrator.get_active_plugins_for_step(GCODE_FIRMWARE).front()->get_id() != firmware_id);
    DynamicPrintConfig config = plain;
    config.set_deserialize_strict("gcode_firmware_plugin", firmware_id);
    config.set_deserialize_strict(k_gcode_exporter_option_key, k_probe_exporter_id);

    const Plugin *grouped = Steps::selected_active_plugin_from_group(
        orchestrator, GCODE_FIRMWARE, "gcode_firmware_plugin", &config);
    REQUIRE(grouped != nullptr);
    CHECK(grouped->get_id() == firmware_id);

    orchestrator_handle *handle = reinterpret_cast<orchestrator_handle *>(&orchestrator);
    const orchestrator_plugin_handle *selected = orchestrator_select_plugin(
        handle, ApiHost::to_config_handle(&config), GCODE_FIRMWARE, "gcode_firmware_plugin");
    REQUIRE(selected != nullptr);
    CHECK(std::string(orchestrator_plugin_id(selected)) == firmware_id);

    g_exporter_probe_state.query_mode = ProbeQueryMode::Compatible;
    CHECK(orchestrator.select_gcode_exporter(config).plugin_id == k_probe_exporter_id);
    CHECK(g_exporter_probe_state.query_firmware_id == firmware_id);
}

// Child half of the next test: the process starts without the built-in exporter, as an installation could. Run only by it.
TEST_CASE("Exporter startup without the built-in exporter", "[.][exporter-startup-child]")
{
    REQUIRE(std::getenv("SLIC3R_TEST_SIMPLE_EXPORTER_DISABLED") != nullptr);
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    REQUIRE_FALSE(orchestrator.is_plugin_active(orchestrator.get_plugin(k_default_gcode_exporter_id)));

    const ConfigOptionDef *selector = PrintConfigDef::instance().get(k_gcode_exporter_option_key);
    REQUIRE(selector != nullptr);
    REQUIRE(static_cast<bool>(selector->default_value));
    CHECK(selector->default_value->serialize() == k_default_gcode_exporter_id);
    const DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    CHECK(config.opt_serialize(k_gcode_exporter_option_key) == k_default_gcode_exporter_id);
    CHECK_THROWS_WITH(orchestrator.select_gcode_exporter(config),
                      "The configured G-code exporter 'gcode.exporter.simple' is inactive.");
    CHECK_THROWS_WITH(orchestrator.select_gcode_exporter(DynamicPrintConfig()),
                      "The configured G-code exporter 'gcode.exporter.simple' is inactive.");
    check_exporter_profile_loading();
}

// Start a second test process without the built-in exporter, since the exporter list is fixed once the runtime starts.
// Boost.Process is already part of the Boost used by the project, starts the executable without a shell (no quoting
// rules), and boost::dll finds the running executable on every platform. The child inherits the working directory,
// gets one extra environment variable, and is killed if it does not finish within the delay.
TEST_CASE("An unavailable default exporter is never replaced by another provider", "[plugins][gcode][exporter]")
{
    ScopedTestDirectory directory;
    const boost::filesystem::path log = directory.path() / "startup-child.txt";
    namespace bp = boost::process;
    // Keep the native (wide on Windows) path: a narrow conversion would break Unicode installation folders.
    const boost::filesystem::path program = boost::dll::program_location();
    bp::child child(bp::exe = program,
                    bp::args = {"Exporter startup without the built-in exporter"},
                    bp::env["SLIC3R_TEST_SIMPLE_EXPORTER_DISABLED"] = "1",
                    (bp::std_out & bp::std_err) > log);
    const bool finished = child.wait_for(std::chrono::seconds(300));
    if (!finished)
        child.terminate();
    child.wait();
    // Only the end of the child output tells which of its checks failed; the start is plugin loading noise.
    const std::string child_output = read_binary_file(log);
    INFO(child_output.substr(child_output.size() > 4000 ? child_output.size() - 4000 : 0));
    CHECK(finished);
    CHECK(child.exit_code() == 0);
}
// Verify the CLI parser accepts and selects an active exporter, exporter settings resolve to STEP_NONE, firmware still invalidates STEP_GCODE, and unavailable IDs fail visibly.
TEST_CASE("G-code exporter selection is strict and its options do not invalidate slicing", "[plugins][gcode][exporter]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();

    const ConfigOptionDef *exporter_option = PrintConfigDef::instance().get(k_probe_option_key);
    REQUIRE(exporter_option != nullptr);
    CHECK(exporter_option->option_preset_type == RAW_PRESET_TYPE_FFF_PRINTER);
    CHECK(exporter_option->invalidates_step == STEP_NONE);
    const ConfigOptionDef *firmware_option = PrintConfigDef::instance().get("gcode_firmware_plugin");
    REQUIRE(firmware_option != nullptr);
    CHECK(firmware_option->invalidates_step == STEP_GCODE);

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    CHECK_THROWS(config.set_deserialize_strict("gcode_exporter_plugin", "test.gcode.exporter.missing"));

    if (!DynamicPrintAndCLIConfig::is_cli_def_initialized())
        DynamicPrintAndCLIConfig::initialize_cli_def();
    const char *cli_arguments[] = {
        "Slic3r", "--gcode-exporter-plugin", k_probe_exporter_id
    };
    DynamicPrintAndCLIConfig cli_config;
    t_config_option_keys extra_arguments;
    REQUIRE(cli_config.read_cli(3, cli_arguments, &extra_arguments));
    const ConfigOption *cli_exporter_option = cli_config.option("gcode_exporter_plugin");
    REQUIRE(cli_exporter_option != nullptr);
    CHECK(cli_exporter_option->serialize() == k_probe_exporter_id);

    {
        ScopedPluginActivation probe(orchestrator, k_probe_exporter_id);
        Print cli_selected_print;
        Model cli_selected_model;
        initialize_fff_print(cli_selected_print, cli_selected_model, "gcode.exporter.simple");
        DynamicPrintConfig cli_selected_config = cli_selected_print.full_print_config();
        cli_selected_config.set_deserialize_strict(
            "gcode_exporter_plugin", cli_exporter_option->serialize());
        REQUIRE(cli_selected_print.apply(cli_selected_model, cli_selected_config) !=
                PrintBase::APPLY_STATUS_INVALIDATED);
        const Orchestrator::GCodeExporterSelection cli_selection =
            orchestrator.select_gcode_exporter(cli_selected_print.full_print_config());
        CHECK(cli_selection.plugin_id == k_probe_exporter_id);
        CHECK(cli_selection.extension == ".probe");
    }

    Print print;
    Model model;
    initialize_fff_print(print, model, k_probe_exporter_id);
    CHECK_THROWS_WITH(orchestrator.select_gcode_exporter(print.full_print_config()),
                      "The configured G-code exporter 'test.gcode.exporter.probe' is inactive.");
}
