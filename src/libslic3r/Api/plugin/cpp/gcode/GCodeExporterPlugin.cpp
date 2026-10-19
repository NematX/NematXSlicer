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

This file validates one synchronous GCODE_EXPORT payload and dispatches its
operation to the derived C++ provider. QUERY strings stay owned by a local
GCodeExportQuery until the host callback copies them. EXPORT views refer to the
host's Print and paths only for the duration of the provider call.

Representative direct calls (branches are not a timeline):

    GCodeExporterPluginBase::run_impl()
        |-- plugin_ctx_as_gcode_export()
        |-- query_impl()
        |-- context->emit_query_result()
        `-- export_impl()
*/

namespace slic3r_api {

// Validate one borrowed run context, copy QUERY text during its callback or dispatch EXPORT views, and throw on invalid fields while retaining no caller-owned data.
void GCodeExporterPluginBase::run_impl(const plugin_run_context *run_context) const
{
    throw_if_cancelled(run_context);
    const run_ctx_gcode_export *context = plugin_ctx_as_gcode_export(run_context);
    if (context == nullptr)
        throw std::invalid_argument("G-code exporter received an incomplete synchronous context.");

    if (context->operation == RAW_GCODE_EXPORT_OPERATION_QUERY) {
        if (context->config == nullptr || context->firmware_plugin_id == nullptr ||
            context->query_callback_context == nullptr || context->emit_query_result == nullptr ||
            context->print != nullptr || context->reference_path != nullptr || context->output_path != nullptr ||
            context->thumbnail_renderer != nullptr || context->metadata != nullptr)
            throw std::invalid_argument("G-code exporter QUERY received invalid operation fields.");

        const Config config(context->config);
        const GCodeExportQuery query = query_impl(config, context->firmware_plugin_id);
        throw_if_cancelled(run_context);
        raw_gcode_export_query_result result = {};
        result.struct_size = sizeof(result);
        result.status = query.status;
        result.extension = query.extension.empty() ? nullptr : query.extension.c_str();
        result.message = query.message.empty() ? nullptr : query.message.c_str();
        context->emit_query_result(context->query_callback_context, &result);
        throw_if_cancelled(run_context);
        return;
    }

    if (context->operation != RAW_GCODE_EXPORT_OPERATION_EXPORT || context->print == nullptr ||
        context->config == nullptr ||
        context->reference_path == nullptr || context->reference_path[0] == '\0' ||
        context->output_path == nullptr || context->output_path[0] == '\0' ||
        context->firmware_plugin_id != nullptr ||
        context->query_callback_context != nullptr || context->emit_query_result != nullptr)
        throw std::invalid_argument("G-code exporter EXPORT received invalid operation fields.");

    const Print print(context->print);
    const Config config(context->config);
    const GCodeThumbnailRendererView renderer(context->thumbnail_renderer);
    export_impl(*run_context, print, config, context->reference_path, context->output_path,
                renderer, context->metadata);
    throw_if_cancelled(run_context);
}

} // namespace slic3r_api
