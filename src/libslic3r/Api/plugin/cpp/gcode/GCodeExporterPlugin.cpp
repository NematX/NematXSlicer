///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "GCodeExporterPlugin.hpp"

#include <stdexcept>

#include "libslic3r/Api/plugin/cpp/ConfigViews.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"

/*
G-code exporter provider adapter
=================================

This file validates one synchronous STEP_EXPORT payload and dispatches its
operation to the derived C++ provider. QUERY strings stay owned by a local
GCodeExportQuery until the host callback copies them. EXPORT views and metadata
refer to host storage only for the duration of the provider call. The reporter
gives the host the outcome of the export; the host refuses a second report.

Representative direct calls (branches are not a timeline):

    GCodeExporterPluginBase::run_impl()
        |-- plugin_ctx_as_gcode_export()
        |-- query_impl()
        |-- context->emit_query_result()
        `-- export_impl()
*/

namespace slic3r_api {

// Read the destination of the export from its configuration; empty when the request set none.
std::string gcode_export_output_path(const Config &config)
{
    if (!config.has("output_path"))
        return std::string();
    return config.get("output_path").get_string();
}

// Copy both strings into owned storage, give them to the host and throw when the host refuses the report.
void GCodeExportResultReporter::report(std::string_view local_path_utf8, std::string_view label) const
{
    if (m_context.report_export_result == nullptr)
        throw std::runtime_error("The host did not provide an export result callback.");
    const std::string local_path(local_path_utf8);
    const std::string owned_label(label);
    if (!m_context.report_export_result(m_context.result_callback_context, local_path.c_str(), owned_label.c_str()))
        throw std::runtime_error("The host refused the export result: an export reports one result.");
}

// Default: the export asks for nothing.
void GCodeExporterPluginBase::input_definitions(const Config &, std::vector<raw_config_option_def> &) const {}

// In EXPORT only, collect the definitions of the exporter and give them to the host, which asks the requester and blocks until
// the answer; throw an ordinary exception when the host answers that the export has to stop, which makes the host skip run.
// The host already holds the cause of the stop, so the message here is not shown. A cancellation of the plugin is not used:
// PluginBase would swallow it and the host would still run.
void GCodeExporterPluginBase::setup_run_impl(const plugin_run_context *run_context) const
{
    const run_ctx_gcode_export *context = plugin_ctx_as_gcode_export(run_context);
    if (context == nullptr || context->operation != RAW_GCODE_EXPORT_OPERATION_EXPORT || context->config == nullptr ||
        context->request_export_inputs == nullptr)
        return;

    const Config config(context->config);
    std::vector<raw_config_option_def> definitions;
    this->input_definitions(config, definitions);
    if (definitions.empty())
        return;
    if (!context->request_export_inputs(context->input_callback_context, definitions.data(), definitions.size()))
        throw std::runtime_error("The export was stopped while its inputs were asked.");
}
// Validate borrowed run data including the payload Print, lend owned QUERY text and suitability during its synchronous callback, or dispatch EXPORT views with non-null metadata; invalid fields and provider failures throw without retaining caller storage.
void GCodeExporterPluginBase::run_impl(const plugin_run_context *run_context) const
{
    throw_if_cancelled(run_context);
    const run_ctx_gcode_export *context = plugin_ctx_as_gcode_export(run_context);
    if (context == nullptr)
        throw std::invalid_argument("G-code exporter received an incomplete synchronous context.");

    if (context->operation == RAW_GCODE_EXPORT_OPERATION_QUERY) {
        if (context->config == nullptr || context->firmware_plugin_id == nullptr ||
            context->query_callback_context == nullptr || context->emit_query_result == nullptr ||
            context->print == nullptr || context->reference_path != nullptr ||
            context->thumbnail_renderer != nullptr || context->metadata != nullptr ||
            context->result_callback_context != nullptr || context->report_export_result != nullptr ||
            context->input_callback_context != nullptr || context->request_export_inputs != nullptr)
            throw std::invalid_argument("G-code exporter QUERY received invalid operation fields.");

        const Config config(context->config);
        const Print print(context->print);
        const GCodeExportQuery query = query_impl(print, config, context->firmware_plugin_id);
        throw_if_cancelled(run_context);
        raw_gcode_export_query_result result = {};
        result.struct_size = sizeof(result);
        result.status = query.status;
        result.extension = query.extension.empty() ? nullptr : query.extension.c_str();
        result.message = query.message.empty() ? nullptr : query.message.c_str();
        if (query.status == RAW_GCODE_EXPORT_QUERY_COMPATIBLE) {
            result.action_label = query.action_label.empty() ? nullptr : query.action_label.c_str();
            result.supports_removable_media = query.supports_removable_media ? 1 : 0;
        }
        context->emit_query_result(context->query_callback_context, &result);
        throw_if_cancelled(run_context);
        return;
    }

    if (context->operation != RAW_GCODE_EXPORT_OPERATION_EXPORT || context->print == nullptr ||
        context->config == nullptr ||
        context->reference_path == nullptr || context->reference_path[0] == '\0' ||
        context->thumbnail_renderer == nullptr || context->metadata == nullptr ||
        context->report_export_result == nullptr || context->request_export_inputs == nullptr ||
        context->firmware_plugin_id != nullptr ||
        context->query_callback_context != nullptr || context->emit_query_result != nullptr)
        throw std::invalid_argument("G-code exporter EXPORT received invalid operation fields.");

    const Print print(context->print);
    const Config config(context->config);
    const GCodeThumbnailRendererView renderer(context->thumbnail_renderer);
    const GCodeExportResultReporter result(*context);
    export_impl(*run_context, print, config, context->reference_path, renderer, context->metadata, result);
    throw_if_cancelled(run_context);
}

} // namespace slic3r_api
