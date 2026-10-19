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
#include "test_data.hpp"

#include "libslic3r/Api/host/ApiHostUtils.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_gcode_export.h"
#include "libslic3r/Api/plugin/cpp/ConfigViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeExporterPlugin.hpp"
#include "libslic3r/Config/ConfigDef.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Plugins/PluginApiCompatibility.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"



namespace {

using namespace Slic3r;

constexpr const char *k_probe_exporter_id = "test.gcode.exporter.probe";
constexpr const char *k_raw_query_exporter_id = "test.gcode.exporter.raw_query";
constexpr const char *k_pruned_exporter_id = "test.gcode.exporter.pruned";
constexpr const char *k_probe_option_key = "test.gcode_exporter_option";
const char *const k_no_dependencies[] = {nullptr};

enum class ProbeQueryMode { Compatible, Incompatible };
enum class ProbeExportMode { Copy, FailOpen, ReportErrorAfterWrite, CancelAfterWrite };
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
    // Print whose user cancellation the CancelAfterWrite mode raises, as the GUI Cancel button does.
    Print *print_to_cancel = nullptr;
};

ExporterProbeState g_exporter_probe_state;

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
void initialize_fff_print(Print &print, Model &model, const std::string &exporter_id)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_extruders(1);
    config.set_deserialize_strict({
        {"step_gcode_plugin", "gcode.printing_plan_file_writer"},
        {"gcode_firmware_plugin", "gcode.firmware.marlin2"},
        {"binary_gcode", "1"}});
    config.set_deserialize_strict("gcode_exporter_plugin", exporter_id);
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

    // Read the borrowed UTF-8 reference and write the borrowed temporary path in bounded blocks; throw on I/O failure or report/cancel after partial output in the selected mode.
    void export_impl(const plugin_run_context &run_context,
                     const slic3r_api::Print &,
                     const slic3r_api::Config &config,
                     std::string_view reference_path_utf8,
                     std::string_view temporary_output_path_utf8,
                     slic3r_api::GCodeThumbnailRendererView,
                     const raw_gcode_export_metadata *) const override
    {
        ++g_exporter_probe_state.export_count;
        g_exporter_probe_state.export_config_selector = config.get(k_gcode_exporter_option_key).serialize();
        if (g_exporter_probe_state.export_mode == ProbeExportMode::FailOpen)
            throw std::runtime_error("probe exporter could not open its output");
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
    if (!orchestrator.register_plugin(probe.c_instance()) ||
        !orchestrator.register_plugin(raw_query.c_instance()) ||
        !orchestrator.register_plugin(pruned.c_instance()))
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
    CHECK(orchestrator.export_gcode(print, reference.string(), nullptr, nullptr) == reference.string());
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
    CHECK(read_binary_file(boost::filesystem::path(first_path)) == expected);
    CHECK(read_binary_file(second_target) == expected);
    CHECK(orchestrator.export_print(print, print.full_print_config(), reference.string(), third_target.string(), selection) == third_target.string());
    CHECK(read_binary_file(third_target) == expected);
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
    // The exported file is the writer reference itself: generate it directly and compare every byte.
    const boost::filesystem::path direct_reference = directory.path() / "direct-reference.gcode";
    Orchestrator::instance().export_gcode(print, direct_reference.string(), nullptr, nullptr);
    CHECK(expected == read_binary_file(direct_reference));

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

// The ABI registry exposes the exporter header at version 1.0 and package manifests naming it are validated against it.
TEST_CASE("G-code exporter header is part of the plugin ABI registry", "[plugins][gcode][exporter][abi]")
{
    const std::vector<PluginApiRequirement> &host = host_plugin_api_contracts();
    REQUIRE(host.size() > size_t(SLIC3R_PLUGIN_API_STEP_GCODE_EXPORT));
    CHECK(host[SLIC3R_PLUGIN_API_STEP_GCODE_EXPORT].header == "steps/slic3r_step_gcode_export.h");
    CHECK(host[SLIC3R_PLUGIN_API_STEP_GCODE_EXPORT].version.major == 1);
    CHECK(host[SLIC3R_PLUGIN_API_STEP_GCODE_EXPORT].version.minor == 0);

    PluginPackageMetadata accepted;
    std::string error;
    REQUIRE(parse_plugin_package_metadata(
        "[plugin]\npackage_version=1.0.0\nslicer_version=1.0.0\n"
        "[abi]\nslic3r_plugin_types.h=4.1\nsteps/slic3r_step_gcode_export.h=1.0\n", accepted, error));
    CHECK(accepted.compatibility.compatible());
    PluginPackageMetadata rejected;
    REQUIRE(parse_plugin_package_metadata(
        "[plugin]\npackage_version=1.0.0\nslicer_version=1.0.0\n"
        "[abi]\nslic3r_plugin_types.h=4.1\nsteps/slic3r_step_gcode_export.h=2.0\n", rejected, error));
    CHECK_FALSE(rejected.compatibility.compatible());
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
    orchestrator.export_gcode(print, reference.string(), &result, nullptr);
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
    const double sentinel_volume = 4242.5;
    print.print_statistics().total_extruded_volume = sentinel_volume;
    const size_t gcode_timestamp = print.step_state_with_timestamp(STEP_GCODE).timestamp;

    const Orchestrator::GCodeExporterSelection selection = orchestrator.select_gcode_exporter(print.full_print_config());
    const std::string first = orchestrator.export_print(
        print, print.full_print_config(), reference.string(), (directory.path() / "one").string(), selection);
    const std::string second = orchestrator.export_print(
        print, print.full_print_config(), reference.string(), (directory.path() / "two").string(), selection);

    CHECK(read_binary_file(boost::filesystem::path(first)) == reference_bytes);
    CHECK(read_binary_file(boost::filesystem::path(second)) == reference_bytes);
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
// The EXPORT payload carries the configuration copy; the adapter refuses a payload without it.
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

    run_ctx_gcode_export context = {};
    context.operation = RAW_GCODE_EXPORT_OPERATION_EXPORT;
    context.print = reinterpret_cast<const print_handle *>(&print);
    context.reference_path = reference_path.c_str();
    context.output_path = output_path.c_str();
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

    context.config = ApiHost::to_config_handle(&print.full_print_config());
    CHECK(orchestrator.execute_plugin(*plugin, &print, &payload, 1, false, &reported_error) ==
          RAW_PLUGIN_EXECUTION_SUCCESS);
    CHECK(g_exporter_probe_state.export_count == exports_before + 1);
    CHECK(read_binary_file(output) == "G1 X1\n");
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
